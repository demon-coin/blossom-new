"""宿主侧 ktest 测试目录（catalog）解析、过滤与分批调度回归测试。

本模块验证 `ktest.catalog` 与命令行 `list` 子命令在测试用例筛选（精确匹配、通配符、
参数、标签、排除、忽略项与哈希分片）、随机种子推导、批次划分以及报告导出方面的行为约束。
运行方式：`python3 -m unittest discover -s tools/ktest`。
"""

from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from host_helpers import CLI, CONFIG, case_record, catalog_of, options
from ktest.catalog import (Catalog, HostError, batches, case_seed, invocations,
                           run_command, select, stable_hash)


class SelectionTests(unittest.TestCase):
    """验证精确名称、glob 通配符、参数、标签、排除规则与哈希分片的筛选语义。"""

    def test_exact_parameter_is_not_glob_or_prefix(self):
        """验证带方括号的参数化用例名称按字面精确匹配，不会误当作 glob 字符集或前缀匹配。"""
        rows = [case_record(0, name="os::s::x[a]"), case_record(1, name="os::s::x[a]_tail"),
                case_record(2, name="os::s::xa")]
        actual = select(catalog_of(rows), options(exact=["os::s::x[a]"]), execution=True)
        self.assertEqual([r["name"] for r in actual], ["os::s::x[a]"])

    def test_union_tags_exclude_and_ignore_precedence(self):
        """验证正向匹配规则（exact/include）取并集后，仍须同时满足所有标签过滤并剔除 exclude 项。"""
        rows = [case_record(0, tags=["smp", "mm"]), case_record(1, tags=["smp"]),
                case_record(2, tags=["smp", "mm"], ignore="manual"),
                case_record(3, tags=["smp", "mm"])]
        args = options(exact=[rows[0]["name"]], include=["*case*"], tag=["smp", "mm"], exclude=["*case3*"])
        self.assertEqual(select(catalog_of(rows), args, execution=True), [rows[0]])
        args.include_ignored = True
        self.assertEqual(select(catalog_of(rows), args, execution=True), [rows[0], rows[2]])

    def test_list_keeps_ignored_reasons_and_run_requires_opt_in(self):
        """验证 `list` 模式保留被忽略用例及其原因，而执行模式默认跳过忽略项、仅在显式开启时纳入。"""
        rows = [case_record(0, name="os::s::z", ignore="requires manual hardware"),
                case_record(1, name="os::s::a"),
                case_record(2, name="os::s::m", ignore="destructive fixture")]
        catalog = catalog_of(rows)
        args = options()
        listed = select(catalog, args, execution=False)
        self.assertEqual(listed, [rows[1], rows[2], rows[0]])
        self.assertEqual([row["ignore"] for row in listed],
                         [None, "destructive fixture", "requires manual hardware"])
        self.assertEqual(select(catalog, args, execution=True), [rows[1]])
        args.include_ignored = True
        self.assertEqual(select(catalog, args, execution=True), listed)

    def test_only_ignored_selection_explains_actual_reasons(self):
        """验证当筛选结果仅包含被忽略的用例时，执行模式会报错列出各项忽略原因并提示 `--include-ignored`。"""
        rows = [case_record(0, ignore="manual device"), case_record(1, ignore="writes scratch")]
        catalog = catalog_of(rows)
        for args in (options(), options(exact=[rows[0]["name"]])):
            with self.subTest(exact=args.exact):
                self.assertTrue(select(catalog, args, execution=False))
                with self.assertRaises(HostError) as raised:
                    select(catalog, args, execution=True)
                message = str(raised.exception)
                self.assertIn("--include-ignored", message)
                matched = rows if not args.exact else rows[:1]
                for row in matched:
                    self.assertIn(row["name"], message)
                    self.assertIn(row["ignore"], message)

    def test_each_unknown_exact_name_rejects_valid_union_matches(self):
        """验证只要存在未知的精确用例名即报错，不会被其他命中的精确名、通配符或参数规则掩盖。"""
        row = case_record()
        catalog = catalog_of([row])
        args = options(exact=[row["name"], "os::missing::first", "os::missing::second"],
                       include=["*"], parameter=["one"])
        for execution in (False, True):
            with self.subTest(execution=execution), self.assertRaises(HostError) as raised:
                select(catalog, args, execution=execution)
            self.assertIn("os::missing::first", str(raised.exception))
            self.assertIn("os::missing::second", str(raised.exception))

    def test_parameter_union_tags_exclude_and_list_ignore(self):
        """验证参数匹配与精确名、glob 通配符取并集，且 `list` 模式在应用标签和排除规则后仍保留忽略项。"""
        rows = [case_record(0, parameter="alpha", tags=["mm"]),
                case_record(1, parameter="beta", tags=["mm"], ignore="manual"),
                case_record(2, parameter="gamma", tags=["mm"]),
                case_record(3, parameter="beta", tags=[]),
                case_record(4, parameter="beta", tags=["mm"])]
        args = options(exact=[rows[0]["name"]], include=["*case2*"], parameter=["beta"],
                       tag=["mm"], exclude=["*case4*"])
        catalog = catalog_of(rows)
        self.assertEqual(select(catalog, args, execution=False), rows[:3])
        self.assertEqual(select(catalog, args, execution=True), [rows[0], rows[2]])

    def test_list_shards_partition_ignored_and_regular_names(self):
        """验证 `list` 模式下分片使用统一的完整名称哈希切分普通项与忽略项，不重不漏且不提前按 CPU 数量过滤。"""
        rows = [case_record(i, ignore="manual" if i % 2 else None, min_cpus=4)
                for i in range(100)]
        catalog = catalog_of(rows)
        partitions = [select(catalog, options(shard=(i, 3), cpus=1), execution=False)
                      for i in range(3)]
        names = [row["name"] for part in partitions for row in part]
        self.assertEqual(set(names), {row["name"] for row in rows})
        self.assertEqual(len(names), len(rows))
        self.assertEqual(select(catalog, options(cpus=1), execution=True),
                         sorted(rows[::2], key=lambda row: row["name"]))

    def test_registration_order_does_not_change_selection_or_shuffle(self):
        """验证相同的用例集合在筛选后先按完整名称排序，ELF 内部注册顺序变化不影响最终执行顺序与随机种子。"""
        names = ["os::s::case[z]", "os::s::case[a]", "os::s::case[m]"]
        catalogs = [
            catalog_of([case_record(index, name=name) for index, name in enumerate(order)])
            for order in (names, list(reversed(names)))
        ]
        expected = ["os::s::case[a]", "os::s::case[m]", "os::s::case[z]"]
        sequences = []
        args = options(shuffle=True, repeat=2)
        for catalog in catalogs:
            selected = select(catalog, args, execution=True)
            self.assertEqual([row["name"] for row in selected], expected)
            sequences.append([
                (item.case["name"], item.repeat, item.seed)
                for item in invocations(selected, args)
            ])
        self.assertEqual(sequences[0], sequences[1])

    def test_no_match_is_error(self):
        """验证筛选结果为空时必须抛出 HostError，防止无匹配时静默返回成功。"""
        with self.assertRaises(HostError):
            select(catalog_of([case_record()]), options(exact=["missing"]), execution=True)

    def test_shard_partition_and_fixed_hash(self):
        """验证哈希分片使用规定的 SHA-256 大端前缀，并将全部用例无重叠、无遗漏地划分到各分片。"""
        self.assertEqual(stable_hash("abc"), 0xba7816bf8f01cfea)
        rows = [case_record(i) for i in range(1000)]
        catalog = catalog_of(rows)
        partitions = [select(catalog, options(shard=(i, 7)), execution=True) for i in range(7)]
        names = [r["name"] for part in partitions for r in part]
        self.assertEqual(set(names), {r["name"] for r in rows})
        self.assertEqual(len(names), len(set(names)))

    def test_jobs_do_not_change_seed_or_shuffle(self):
        """验证并发 VM 数量（`--jobs`）不会改变重复执行实例的排列顺序和每个实例的随机种子。"""
        rows = [case_record(i) for i in range(40)]
        one = invocations(rows, options(jobs=1, repeat=3, shuffle=True))
        many = invocations(rows, options(jobs=8, repeat=3, shuffle=True))
        self.assertEqual(one, many)
        expected = int.from_bytes(hashlib.sha256(b"17:0:os::tests0::case0[one]").digest()[:8], "big")
        self.assertEqual(next(item.seed for item in one if item.case == rows[0] and item.repeat == 0), expected)
        self.assertNotEqual(case_seed(17, 0, rows[0]["name"]), case_seed(17, 1, rows[0]["name"]))

    def test_large_catalog_and_command_batches_preserve_every_instance(self):
        """验证大规模测试目录在按批次切分时不丢失、不重复，且单批用例数与串口命令长度均不超过上限。"""
        rows = [case_record(i) for i in range(10000)]
        catalog = catalog_of(rows)
        items = invocations(select(catalog, options(), execution=True), options())
        groups = batches(items, options())
        self.assertEqual([item for group in groups for item in group], items)
        for group in groups:
            self.assertLessEqual(len(group), 64)
            self.assertLessEqual(len(run_command(group, 17)), 1024)

    def test_case_isolation_only_strengthens(self):
        """验证声明了独立隔离级别（Case）或预期触发致命错误的用例各自独占批次，不与其他用例共用 VM。"""
        rows = [case_record(0), case_record(1, isolation="Case"), case_record(2),
                case_record(3, expected_fatal="panic")]
        groups = batches(invocations(rows, options()), options())
        self.assertEqual([len(group) for group in groups], [1, 1, 1, 1])
        self.assertTrue(all(len(group) == 1 for group in batches(invocations(rows, options()), options(isolation="case"))))

    def test_catalog_rejects_duplicate_identity_and_counts(self):
        """验证目录构建器会拒绝重复的用例全名，并在结尾统计数量与实际用例数不符时报错。"""
        with self.assertRaises(HostError):
            catalog_of([case_record(0), case_record(1, name=case_record(0)["name"])])
        catalog = Catalog(CONFIG)
        catalog.feed(dict(type="catalog", version=1, **CONFIG))
        with self.assertRaises(HostError):
            catalog.feed(dict(type="catalog_end", suites=0, cases=1))


class CatalogCommandTests(unittest.TestCase):
    """验证 `list` 命令仅输出筛选后的目录记录与空结果报告，不会触发用例执行或运行时资源校验。"""

    def test_list_prints_ignored_without_execution_or_resource_validation(self):
        """验证 `list` 子命令保留资源元数据与忽略项信息，生成的报告中不包含任何执行结果记录。"""
        rows = [case_record(0, resources=["root", "net-user"], ignore="manual NIC"),
                case_record(1, resources=["uart"])]
        catalog = catalog_of(rows)

        def fixed_input(args, directory):
            """生成测试用的占位内核文件及其 SHA-256 摘要，跳过真实 ELF 结构校验。"""
            kernel = directory / "kernel.elf"
            kernel.write_bytes(b"fixed catalog input")
            return kernel, hashlib.sha256(kernel.read_bytes()).hexdigest()

        with tempfile.TemporaryDirectory() as temporary:
            output = io.StringIO()
            # 拦截执行与资源校验入口，确保 list 命令只走目录发现与输出流程
            with patch.object(CLI, "fixed_kernel", side_effect=fixed_input), \
                    patch.object(CLI, "discover", return_value=catalog), \
                    patch.object(CLI, "execute", side_effect=AssertionError("list executed cases")), \
                    patch.object(CLI, "validate_resources",
                                 side_effect=AssertionError("list validated runtime resources")), \
                    redirect_stdout(output), redirect_stderr(io.StringIO()):
                status = CLI.main(["list", "--arch", "riscv64", "--seed", "17",
                                   "--report-dir", temporary])
            self.assertEqual(status, 0)
            # 验证标准输出的 JSON 行记录以及结果报告目录中的元数据内容
            records = [json.loads(line) for line in output.getvalue().splitlines()]
            self.assertEqual([row for row in records if row["type"] == "case"], rows)
            self.assertEqual(records[-1], dict(type="catalog_end", suites=1, cases=2))
            reports = list(Path(temporary).glob("run-*/results.json"))
            self.assertEqual(len(reports), 1)
            report = json.loads(reports[0].read_text())
            self.assertEqual(report["results"], [])
            self.assertEqual(report["selected"], [row["name"] for row in rows])
            self.assertEqual(report["exit_code"], 0)
            self.assertFalse((reports[0].parent / "kernel.elf").exists())


if __name__ == "__main__":
    unittest.main()
