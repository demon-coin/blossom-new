"""内核测试目录（Catalog）协议校验、用例过滤选择与批次切分。

本模块负责解析内核在 `catalog` 模式下通过串口输出的 JSONL 测试清单，校验套件与用例的
静态元数据合法性，并提供与并发工作线程数（`jobs`）无关的确定性用例过滤、分片（shard）、
随机种子派生及单次串口 `run` 命令批次打包逻辑。
"""

from dataclasses import dataclass
import fnmatch
import hashlib
import random


class HostError(RuntimeError):
    """表示宿主机侧的用例选择、外部资源准备或测试协议违反等基础设施错误。"""


def require(condition, message):
    """断言协议或配置条件成立；若不满足则抛出 `HostError(message)`。"""
    if not condition:
        raise HostError(message)


def uint(value, bits, label):
    """校验 `value` 是严格位于 `[0, 2**bits)` 范围内的 Python `int`（拒绝 `bool` 与负数）。"""
    require(type(value) is int and 0 <= value < 1 << bits, f"invalid {label}")
    return value


def text(value, label):
    """校验 `value` 是非空字符串且不含 ASCII 控制字符（`< 32` 或 `127`），确保按行协议安全。"""
    require(isinstance(value, str) and value and not any(ord(c) < 32 or ord(c) == 127 for c in value),
            f"invalid {label}")
    return value


def metadata(row):
    """校验套件或用例记录中的公共静态属性（执行上下文、隔离级别、标签、资源列表、CPU 数与超时）。"""
    require(row.get("context") in ("Boot", "Thread"), "invalid context")
    require(row.get("isolation") in ("Suite", "Case"), "invalid isolation")
    for field in ("tags", "resources"):
        values = row.get(field)
        require(isinstance(values, list), f"missing {field}")
        for value in values:
            text(value, field)
        require(len(set(values)) == len(values), f"duplicate {field}")
    uint(row.get("min_cpus"), 32, "min_cpus")
    require(row["min_cpus"] > 0, "zero min_cpus")
    uint(row.get("timeout_ms"), 64, "timeout_ms")
    require(row["timeout_ms"] > 0, "zero timeout_ms")


class Catalog:
    """单次内核 ELF 输出的测试目录容器，负责流式接收并校验 JSONL 目录记录。"""

    def __init__(self, config):
        """初始化空目录并绑定预期的架构、CPU 数、构建模式与 panic 策略配置。"""
        self.config = config
        self.header = None
        self.suites = {}
        self.cases = []
        self.names = set()
        self.ids = set()
        self.done = False

    def feed(self, row):
        """向目录喂入一条解析后的 JSONL 字典记录（`catalog` / `suite` / `case` / `catalog_end`）。

        逐条校验记录顺序、编号唯一性、名称唯一性以及用例与所属套件之间的隔离和上下文约束。
        """
        require(isinstance(row, dict) and not self.done, "record after catalog_end")
        kind = row.get("type")
        # 首条记录必须是版本为 1 的 catalog 头部，且配置与宿主机启动参数一致
        if self.header is None:
            require(kind == "catalog" and row.get("version") == 1, "missing catalog header")
            uint(row.get("version"), 32, "catalog version")
            uint(row.get("cpus"), 32, "catalog cpus")
            require(all(row.get(k) == v for k, v in self.config.items()), "catalog configuration mismatch")
            self.header = row
            return
        # 解析测试套件记录，登记套件编号与全名
        if kind == "suite":
            sid = uint(row.get("suite"), 32, "suite")
            name = text(row.get("name"), "suite name")
            metadata(row)
            require(sid not in self.suites and name not in self.names, "duplicate suite identity")
            self.names.add(name)
            self.suites[sid] = row
        # 解析测试用例记录，核验所属套件已注册且上下文/隔离级别不弱于套件要求
        elif kind == "case":
            sid = uint(row.get("suite"), 32, "suite")
            cid = uint(row.get("case"), 32, "case")
            name = text(row.get("name"), "case name")
            require(sid in self.suites, "case before its suite")
            metadata(row)
            require((sid, cid) not in self.ids and name not in self.names, "duplicate case identity")
            parameter = row.get("parameter")
            require(parameter is None or isinstance(parameter, str), "invalid parameter")
            if parameter is not None:
                text(parameter, "parameter")
            text(row.get("source"), "source")
            uint(row.get("line"), 32, "source line")
            for field in ("ignore", "expected_fatal", "fatal_message"):
                require(field in row and (row[field] is None or isinstance(row[field], str)), f"invalid {field}")
                if row[field] is not None:
                    text(row[field], field)
            fatal = row["expected_fatal"]
            require(fatal is None or fatal in ("illegal", "refill", "irq-schedule", "preempt-schedule",
                                             "oom", "timekeeper", "panic", "double-panic", "abi"),
                    "unknown expected fatal type")
            require(fatal != "panic" or row["fatal_message"] is not None, "panic fatal requires fatal_message")
            require(fatal is not None or row["fatal_message"] is None, "fatal_message without expected_fatal")
            require(row["context"] == self.suites[sid]["context"], "case context differs from suite")
            require(self.suites[sid]["isolation"] != "Case" or row["isolation"] == "Case",
                    "case weakens suite isolation")
            self.ids.add((sid, cid))
            self.names.add(name)
            self.cases.append(row)
        # 解析目录结束标记，核对接收到的套件数和用例总数是否与声明完全一致
        elif kind == "catalog_end":
            uint(row.get("suites"), 32, "suite count")
            uint(row.get("cases"), 32, "case count")
            require(row.get("suites") == len(self.suites) and row.get("cases") == len(self.cases),
                    "catalog count mismatch")
            self.done = True
        else:
            raise HostError(f"unexpected catalog record: {kind}")


def stable_hash(value):
    """计算字符串 UTF-8 编码的 SHA-256 前 8 字节无符号大端整数，提供跨进程稳定的哈希值。"""
    return int.from_bytes(hashlib.sha256(value.encode("utf-8")).digest()[:8], "big")


def case_seed(seed, repeat, name):
    """根据全局种子、重复轮次（`repeat`）和用例全名派生单个用例的确定性随机种子。"""
    return stable_hash(f"{seed}:{repeat}:{name}")


def select(catalog, args, *, execution):
    """根据精确名称、通配模式、参数名、标签、排除规则与分片配置筛选测试用例。

    参数:
        catalog: 已完成读取的 `Catalog` 实例。
        args: 包含筛选条件（`exact`、`include`、`parameter`、`tag`、`exclude`、`shard` 等）的参数对象。
        execution: 为 `True` 时处于实际执行模式（默认跳过带 `ignore` 标记的用例，除非指定 `--include-ignored`）；
            为 `False` 时处于仅列出目录模式（保留 `ignore` 用例）。

    返回:
        按用例全名升序排列的选中用例字典列表。若无匹配项则抛出 `HostError` 并说明原因。
    """
    names = {case["name"] for case in catalog.cases}
    missing = sorted(set(args.exact) - names)
    require(not missing, "unknown exact case names: " + ", ".join(missing))
    selected = []
    ignored = []
    for case in catalog.cases:
        name = case["name"]
        positive = not (args.exact or args.include or args.parameter)
        positive |= name in args.exact
        positive |= any(fnmatch.fnmatchcase(name, p) for p in args.include)
        positive |= case["parameter"] in args.parameter
        if not positive or not set(args.tag).issubset(case["tags"]):
            continue
        if any(fnmatch.fnmatchcase(name, p) for p in args.exclude):
            continue
        if args.shard is not None:
            index, count = args.shard
            if stable_hash(name) % count != index:
                continue
        if execution and case["ignore"] is not None and not args.include_ignored:
            ignored.append(case)
            continue
        selected.append(case)
    if not selected and ignored:
        reasons = "; ".join(f"{case['name']}: {case['ignore']}"
                            for case in sorted(ignored, key=lambda case: case["name"]))
        raise HostError(f"selection matched only ignored cases: {reasons}; use --include-ignored to run")
    require(selected, "selection matched no cases")
    selected.sort(key=lambda case: case["name"])
    return selected


@dataclass(frozen=True)
class Invocation:
    """表示单个测试用例在某一轮重复中的一次具体调用计划及其稳定种子与全局序号。"""
    case: dict
    repeat: int
    seed: int
    order: int


def invocations(cases, args):
    """将选中的用例按重复轮次展开为 `Invocation` 列表；若启用 `shuffle` 则按轮次确定性洗牌。"""
    result = []
    for repeat in range(args.repeat):
        ordered = list(cases)
        if args.shuffle:
            random.Random(case_seed(args.seed, repeat, "shuffle")).shuffle(ordered)
        for case in ordered:
            result.append(Invocation(case, repeat, case_seed(args.seed, repeat, case["name"]), len(result)))
    return result


def batches(items, args):
    """将调用序列切分为可由单次 QEMU `run` 命令顺序执行的批次列表。

    仅当相邻调用属于同一轮次、同一套件、均为 `Suite` 级隔离且不含预期致命异常（`expected_fatal`）时，
    才将它们合并到同一批次，同时确保单批不超过 64 个用例且编码后的串口命令不超过 1024 字节。
    """
    result = []
    for item in items:
        isolated = args.isolation == "case" or item.case["isolation"] == "Case" or item.case["expected_fatal"] is not None
        if result and not isolated:
            previous = result[-1]
            first = previous[0]
            same = (first.repeat == item.repeat and first.case["suite"] == item.case["suite"]
                    and first.case["isolation"] == "Suite" and first.case["expected_fatal"] is None)
            candidate = previous + [item]
            if same and len(candidate) <= 64 and len(run_command(candidate, args.seed)) <= 1024:
                previous.append(item)
                continue
        result.append([item])
    return result


def run_command(items, seed):
    """将同一批次的调用列表编码为发往内核串口的 `run <suite> <seed> <repeat> <case_ids>\\n` 字节串。"""
    first = items[0]
    command = f"run {first.case['suite']} {seed} {first.repeat} "
    command += ",".join(str(item.case["case"]) for item in items)
    encoded = (command + "\n").encode("ascii")
    require(len(encoded) <= 1024 and len(items) <= 64, "run command exceeds UART limit")
    return encoded
