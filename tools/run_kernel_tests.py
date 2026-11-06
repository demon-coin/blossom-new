#!/usr/bin/env python3
"""内核测试（ktest）统一命令行入口脚本。

本脚本负责在宿主机上驱动 `riscv64` 与 `loongarch64` 架构的 QEMU 内核测试，支持 `list`（列举用例）
和 `run`（执行用例）两个子命令。核心工作流程包括：
1. 解析命令行过滤条件、硬件配置、磁盘挂载与各阶段超时预算；
2. 在本次运行的专属报告目录中构建或复制独立的只读测试内核 `kernel.elf`，并校验其 ELF 段布局；
3. 启动 QEMU 完成测试目录（catalog）发现，并按过滤条件、分片、重复次数与随机种子筛选排列用例；
4. 按隔离级别（`suite` 或 `case`）分批启动 QEMU 执行测试、解析串口协议结果，并生成 `results.json` 与 JUnit `results.xml` 报告。
"""

import argparse
import json
import math
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

from ktest.catalog import HostError, require, select, invocations, batches
from ktest.resources import regular_file, hash_file, resource_names, validate_resources
from ktest.boot import validate_test_kernel_layout
from ktest.batch import discover, execute, config
from ktest.qemu_session import CANCELLED

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def positive_seconds(value):
    """解析并校验命令行超时秒数参数，要求必须为有限正浮点数。"""
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def natural(value):
    """解析并校验正整数参数（如 CPU 核数、重复次数、并发任务数）。"""
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def seed_value(value):
    """解析并校验随机种子参数，要求在无符号 64 位整数（`u64`）范围内。"""
    number = int(value, 0)
    if not 0 <= number < 1 << 64:
        raise argparse.ArgumentTypeError("seed must be u64")
    return number


def shard_value(value):
    """解析形如 `INDEX/COUNT` 的分片参数，要求 `0 <= INDEX < COUNT`。"""
    try:
        index, count = map(int, value.split("/"))
        if not 0 <= index < count:
            raise ValueError()
    except ValueError:
        raise argparse.ArgumentTypeError("shard must be INDEX/COUNT with 0 <= INDEX < COUNT") from None
    return index, count


def arguments(argv=None):
    """定义并解析内核测试命令行的全部参数，完成跨选项约束检查与默认值填充。

    参数：
    - `argv`：可选的参数字符串列表；为 `None` 时解析 `sys.argv[1:]`。

    返回：填充了默认内存大小、64 位随机种子与规范化磁盘路径字典 `disks` 的 `argparse.Namespace`。
    """
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("command", choices=("list", "run"))
    parser.add_argument("--arch", required=True, choices=("riscv64", "loongarch64"))
    parser.add_argument("--cpus", type=natural, default=1)
    parser.add_argument("--memory")
    parser.add_argument("--mode", choices=("debug", "release"), default="debug")
    parser.add_argument("--panic", choices=("unwind", "abort"), default="unwind")
    parser.add_argument("--kernel", type=Path)
    parser.add_argument("--qemu", help="explicit QEMU binary")
    parser.add_argument("--exact", action="append", default=[])
    parser.add_argument("--glob", "--include", dest="include", action="append", default=[])
    parser.add_argument("--parameter", action="append", default=[])
    parser.add_argument("--exclude", action="append", default=[])
    parser.add_argument("--tag", action="append", default=[])
    parser.add_argument("--include-ignored", action="store_true")
    parser.add_argument("--shard", type=shard_value)
    parser.add_argument("--repeat", type=natural, default=1)
    parser.add_argument("--shuffle", action="store_true")
    parser.add_argument("--seed", type=seed_value)
    parser.add_argument("--jobs", type=natural, default=1)
    parser.add_argument("--isolation", choices=("suite", "case"), default="suite")
    parser.add_argument("--disk", action="append", default=[], metavar="root|extra|scratch=PATH")
    parser.add_argument("--root", help="explicit root block name, such as /dev/vda")
    parser.add_argument("--root-mode", choices=("ro", "rw"), default="ro", help="guest root mount mode; independent of device read-only")
    parser.add_argument("--extra-root", help="explicit extra filesystem block name")
    parser.add_argument("--read-only", action="append", default=[], choices=("root", "extra", "scratch"))
    parser.add_argument("--scratch-bytes", type=natural, default=64 * 1024 * 1024)
    parser.add_argument("--logical-block-size", type=int, choices=(512, 1024, 2048, 4096), default=512)
    parser.add_argument("--modern", action="store_true")
    parser.add_argument("--uart-payload", type=Path)
    parser.add_argument("--execution-uart", action="store_true",
                        help="enable the concrete opaque execution-log backpressure fixture")
    parser.add_argument("--ap-freeze", action="store_true", help="freeze the actual first AP entry with GDB")
    parser.add_argument("--gdb", help="explicit GDB binary for the AP freeze fixture")
    parser.add_argument("--iommu-platform", action="store_true", help="require ACCESS_PLATFORM in its declared status-only fixture")
    parser.add_argument("--startup-timeout", dest="startup_timeout_seconds", metavar="STARTUP_TIMEOUT", type=positive_seconds, default=30)
    parser.add_argument("--setup-timeout", dest="setup_timeout_seconds", metavar="SETUP_TIMEOUT", type=positive_seconds, default=30)
    parser.add_argument("--case-timeout", dest="case_timeout_seconds", metavar="CASE_TIMEOUT", type=positive_seconds, default=120)
    parser.add_argument("--cleanup-timeout", dest="cleanup_timeout_seconds", metavar="CLEANUP_TIMEOUT", type=positive_seconds, default=30)
    parser.add_argument("--build-timeout", dest="build_timeout_seconds", metavar="BUILD_TIMEOUT", type=positive_seconds, default=600)
    parser.add_argument("--report-dir", type=Path, default=REPOSITORY_ROOT / "build" / "ktest")
    parser.add_argument("--uart-expect", type=Path, help="expected full tx/pause/slow payload")
    args = parser.parse_args(argv)

    # 校验跨选项约束并填充架构默认内存、随机种子和磁盘映射。
    if 1 << 32 <= args.repeat:
        parser.error("repeat must be u32")
    if args.modern and args.arch != "riscv64":
        parser.error("--modern requires riscv64")
    args.memory = args.memory or ("512M" if args.arch == "riscv64" else "768M")
    if not args.memory or any(c in args.memory for c in "\n\r\0"):
        parser.error("invalid memory")
    args.seed = args.seed if args.seed is not None else secrets.randbits(64)
    args.disks = {}
    for specification in args.disk:
        name, separator, path = specification.partition("=")
        if separator != "=" or name not in ("root", "extra", "scratch") or not path or name in args.disks:
            parser.error("--disk requires unique root|extra|scratch=PATH")
        args.disks[name] = Path(path).resolve()
    return args


def terminate_group(process):
    """向子进程所属的独立进程组发送终止信号并等待其完全退出，防止构建超时后遗留后台编译或链接进程。"""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def fixed_kernel(args, directory):
    """在运行目录中准备独立的只读测试内核 `kernel.elf`，确保测试目录发现与各批次 QEMU 执行使用同一份二进制文件。

    参数：
    - `args`：已解析的命令行参数（若指定了 `--kernel` 则直接复制，否则调用 `make build-kernel-test-elf` 构建）。
    - `directory`：本次运行的报告目录。

    返回：`(kernel_path, sha256_hex_digest)` 二元组。
    异常：构建失败、文件非独立普通文件或 ELF 布局校验不通过时抛出 `HostError` / `OSError` / ` subprocess.TimeoutExpired`。
    """
    kernel = directory / "kernel.elf"
    if args.kernel is not None:
        # 使用外部指定的内核镜像时，将其完整复制为报告目录内的独立文件。
        source = args.kernel.resolve()
        regular_file(source)
        with source.open("rb") as input_file, kernel.open("xb") as output:
            shutil.copyfileobj(input_file, output, 1024 * 1024)
    else:
        # 未指定现有内核时，在独立会话中调用 Makefile 构建测试 ELF，并记录构建命令与日志。
        command = ["make", "build-kernel-test-elf", f"ARCH={args.arch}",
                   "TARGET_PLATFORM=qemu-virt", f"KERNEL_BUILD_PROFILE={args.mode}",
                   f"KTEST_PANIC_STRATEGY={args.panic}", f"KERNEL_ELF_OUTPUT={kernel}"]
        (directory / "build-command.json").write_text(json.dumps(command) + "\n", encoding="utf-8")
        with (directory / "build.log").open("xb") as log:
            process = subprocess.Popen(command, cwd=REPOSITORY_ROOT, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                status = process.wait(timeout=args.build_timeout_seconds)
            finally:
                terminate_group(process)
        require(status == 0, f"build-kernel-test-elf failed: {status}")

    # 校验生成的内核文件为无硬链接的独立普通文件，检查 ELF 段布局并设为只读权限。
    regular_file(kernel)
    require(not kernel.is_symlink() and kernel.stat().st_nlink == 1, "fixed ELF must be an independent file")
    layout = validate_test_kernel_layout(kernel, args.arch)
    (directory / "elf-layout.json").write_text(json.dumps(layout, indent=2) + "\n", encoding="utf-8")
    kernel.chmod(0o400)
    return kernel, hash_file(kernel)


def exit_status(results, infrastructure=None):
    """根据所有用例的执行结果汇总进程退出码。

    退出码约定：
    - `2`：发生宿主机或协议层基础设施错误，或结果集为空；
    - `1`：存在用例失败（`Fail`、`Crash`、`Timeout`、`NotRun`）；
    - `4`：所有已选中的用例状态均为跳过（`Skip`）；
    - `0`：全部通过（或部分通过、部分跳过，且无失败与错误）。
    """
    if infrastructure or any(r["status"] == "Error" or r.get("infrastructure_error") for r in results):
        return 2
    if any(r["status"] in ("Fail", "Crash", "Timeout", "NotRun") for r in results):
        return 1
    if results and all(r["status"] == "Skip" for r in results):
        return 4
    return 0 if results else 2


def export_reports(directory, report):
    """将内存中的结构化测试结果同时写出为 `results.json` 和 JUnit 格式的 `results.xml`。"""
    (directory / "results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    results = report["results"]
    suite = ET.Element("testsuite", name="ktest", tests=str(len(results) + bool(report.get("error"))),
                       failures=str(sum(r["status"] in ("Fail", "Crash", "Timeout") for r in results)),
                       errors=str(sum(r["status"] in ("Error", "NotRun") or bool(r.get("infrastructure_error")) for r in results)
                                  + bool(report.get("error"))),
                       skipped=str(sum(r["status"] == "Skip" for r in results)))
    properties = ET.SubElement(suite, "properties")
    for name in ("elf_sha256", "seed", "arch", "cpus", "mode", "panic"):
        ET.SubElement(properties, "property", name=name, value=str(report.get(name, "")))
    for row in results:
        case = ET.SubElement(suite, "testcase", name=row["name"], classname=str(row["suite"]))
        attrs = ET.SubElement(case, "properties")
        for name in ("status", "repeat", "seed", "order", "log"):
            ET.SubElement(attrs, "property", name=name, value=str(row[name]))
        status = row["status"]
        if status != "Pass":
            kind = "skipped" if status == "Skip" else "failure" if status in ("Fail", "Crash", "Timeout") else "error"
            ET.SubElement(case, kind, type=status, message=row["reason"] or status)
        if row.get("infrastructure_error"):
            ET.SubElement(case, "error", type="Protocol", message=row["infrastructure_error"])
        ET.SubElement(case, "system-out").text = "raw UART log: " + row["log"]
    if report.get("error"):
        case = ET.SubElement(suite, "testcase", name="host infrastructure")
        ET.SubElement(case, "error", type="HostError", message=report["error"])
    ET.ElementTree(suite).write(directory / "results.xml", encoding="utf-8", xml_declaration=True)


def main(argv=None):
    """执行一次完整的 `ktest` 列表查询或测试运行，并在退出前清理临时内核副本并导出报告。"""
    args = arguments(argv)
    CANCELLED.clear()
    try:
        args.report_dir.mkdir(parents=True, exist_ok=True)
        directory = Path(tempfile.mkdtemp(prefix="run-", dir=args.report_dir)).resolve()
    except OSError as error:
        print(f"KTEST host error: {error}", file=sys.stderr)
        return 2
    report = dict(version=1, command=sys.argv if argv is None else argv, seed=args.seed,
                  memory=args.memory, jobs=args.jobs, isolation=args.isolation,
                  **config(args), results=[], report_dir=str(directory))
    kernel = directory / "kernel.elf"
    try:
        # 阶段 1：准备只读测试内核 ELF，并启动 QEMU 发现全部内建测试目录（catalog）。
        kernel, digest = fixed_kernel(args, directory)
        report["elf_sha256"] = digest
        report["kernel"] = str(kernel)
        catalog_dir = directory / "catalog"
        catalog_dir.mkdir()
        catalog = discover(args, kernel, catalog_dir)
        selected = select(catalog, args, execution=args.command == "run")
        report["selected"] = [case["name"] for case in selected]

        # 阶段 2：按子命令输出测试目录 JSON 行，或分批校验资源并启动 QEMU 执行测试。
        if args.command == "list":
            print(json.dumps(catalog.header))
            for sid in dict.fromkeys(case["suite"] for case in selected):
                print(json.dumps(catalog.suites[sid], ensure_ascii=False))
            for case in selected:
                print(json.dumps(case, ensure_ascii=False))
            print(json.dumps(dict(type="catalog_end", suites=len({case["suite"] for case in selected}),
                                  cases=len(selected))))
            report["exit_code"] = 0
        else:
            items = invocations(selected, args)
            groups = batches(items, args)
            for group in groups:
                validate_resources(resource_names(catalog.suites[group[0].case["suite"]], group), args)
            report["resources"] = {name: str(path) for name, path in args.disks.items()}
            report["results"] = execute(args, kernel, catalog, groups, directory)
            report["exit_code"] = exit_status(report["results"])
        require(hash_file(kernel) == digest, "fixed ELF content changed during discovery/execution")
    except (HostError, OSError, RuntimeError, ValueError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
        report["error"] = str(error) or "host run interrupted"
        report["exit_code"] = 2
    finally:
        # discover 与 execute 已负责回收各自的 QEMU 子进程与临时盘；此处仅删除体积较大的临时内核副本。
        try:
            kernel.unlink(missing_ok=True)
        except OSError as error:
            report.setdefault("error", f"fixed ELF cleanup failed: {error}")
            report["exit_code"] = 2
            print(f"KTEST cleanup error: {error}", file=sys.stderr)
    export_reports(directory, report)
    print(f"KTEST report={directory} seed={args.seed} exit={report['exit_code']}", file=sys.stderr)
    return report["exit_code"]


def interrupted(signum, frame):
    """捕获 `SIGTERM` 信号：置位全局取消标志并抛出 `KeyboardInterrupt`，交由主流程的 `finally` 统一回收 QEMU 子进程。"""
    CANCELLED.set()
    raise KeyboardInterrupt()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupted)
    sys.exit(main())
