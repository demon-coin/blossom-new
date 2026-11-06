"""宿主机侧 `ktest` 单元测试共用的目录构造、命令行配置及轻量会话夹具。

本模块为宿主机回归测试提供免 QEMU 启动的辅助构造函数，包括：
- 标准化的测试套件（`suite_record`）、参数化用例（`case_record`）及完整 `Catalog` 实例；
- 默认命令行选项命名空间（`options`）与不拉起子进程的 `QemuTestSession` / `UartChannel` 内存桩（`session_fixture`）；
- 已完成前置套件级握手的 `KtapParser` 实例（`parser_for`）及标准用例 KTAP 输出行序列生成（`case_lines`）。
"""

import importlib.util
import io
from pathlib import Path
import sys
from types import SimpleNamespace

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
from ktest.catalog import Catalog, Invocation, case_seed
from ktest.execution_uart import ExecutionUART
from ktest.protocol import KtapParser
from ktest.qemu_session import QemuTestSession
from ktest.uart_channel import UartChannel

SPEC = importlib.util.spec_from_file_location("ktest_cli", TOOLS / "run_kernel_tests.py")
CLI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLI)
CONFIG = dict(arch="riscv64", cpus=2, mode="debug", panic="unwind")


def suite_record(sid=0):
    """构造一条符合目录协议约束的默认测试套件（`suite`）JSONL 字典记录。"""
    return dict(type="suite", suite=sid, name=f"os::tests{sid}", context="Thread",
                isolation="Suite", tags=[], min_cpus=1, timeout_ms=10000, resources=[])


def case_record(cid=0, sid=0, **overrides):
    """构造一条带默认参数名与源码位置属性的测试用例（`case`）JSONL 字典记录，支持关键字参数覆盖字段。"""
    row = dict(type="case", suite=sid, case=cid, name=f"os::tests{sid}::case{cid}[one]",
               parameter="one", source="os/tests.rs", line=10, context="Thread", isolation="Suite",
               tags=[], ignore=None, min_cpus=1, timeout_ms=10000, resources=[], expected_fatal=None,
               fatal_message=None)
    row.update(overrides)
    return row


def catalog_of(cases):
    """通过 `Catalog.feed` 接口依次喂入头部、套件、用例与结束记录，构造经过完整协议校验的 `Catalog` 对象。"""
    catalog = Catalog(CONFIG)
    catalog.feed(dict(type="catalog", version=1, **CONFIG))
    for sid in sorted({row["suite"] for row in cases}):
        catalog.feed(suite_record(sid))
        for row in cases:
            if row["suite"] == sid:
                catalog.feed(row)
    catalog.feed(dict(type="catalog_end", suites=len(catalog.suites), cases=len(cases)))
    return catalog


def options(**overrides):
    """构造宿主机测试默认使用的命令行参数 `SimpleNamespace`，可通过关键字参数覆盖特定选项。"""
    args = SimpleNamespace(exact=[], include=[], parameter=[], tag=[], exclude=[], include_ignored=False,
                           shard=None, repeat=1, shuffle=False, seed=17, isolation="suite", jobs=2,
                           arch="riscv64", cpus=2, mode="debug", panic="unwind", disks={}, read_only=[],
                           uart_payload=None, uart_expect=None, scratch_bytes=4096, logical_block_size=512,
                           ap_freeze=False, gdb=None,
                           iommu_platform=False, modern=False, root=None, root_mode="ro", extra_root=None,
                           startup_timeout_seconds=10, setup_timeout_seconds=5, case_timeout_seconds=10, cleanup_timeout_seconds=5)
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def session_fixture():
    """构造不启动真实 QEMU 进程的 `QemuTestSession` 与内存态 `UartChannel` 组合桩，供串口与会话单元测试使用。"""
    session = QemuTestSession.__new__(QemuTestSession)
    session.debugger = None
    session.gdb_command = None
    session.net_user = None
    uart = UartChannel.__new__(UartChannel)
    uart.args = options()
    uart.directory = None
    uart.check = session.check
    uart.serial = None
    uart.raw = io.BytesIO()
    uart.buffer = bytearray()
    uart.rx_offset = 0
    uart.rx_used = False
    uart.tx_offset = 0
    uart.tx_used = False
    uart.execution_uart = ExecutionUART(2, None)
    session.uart = uart
    return session


def parser_for(cases):
    """针对给定用例列表构造 `KtapParser`，并预先喂入直到 `suite-begin setup` 为止的头部协议行。"""
    items = [Invocation(row, 0, case_seed(17, 0, row["name"]), i) for i, row in enumerate(cases)]
    parser = KtapParser(suite_record(cases[0]["suite"]), items)
    for line in ("KTAP version 1", "1..1", f"  # Subtest: os::tests{cases[0]['suite']}",
                 f"  1..{len(cases)}", "  # KTEST suite-begin setup"):
        parser.feed(line)
    return parser


def case_lines(case, position=1, status="Pass"):
    """生成单个测试用例从 `case-begin`、`setup`、`body`、`cleanup` 到最终 `ok`/`not ok` 结果行的完整 KTAP 文本行列表。"""
    cid = case["case"]
    result = "ok" if status in ("Pass", "Skip") else "not ok"
    directive = " # SKIP insufficient CPUs" if status == "Skip" else (" # ERROR" if status == "Error" else "")
    name = case["name"] + directive
    return [f"  # KTEST case-begin {cid}", f"  # KTEST phase {cid} setup",
            f"  # KTEST phase {cid} body", f"  # KTEST phase {cid} cleanup",
            f"  {result} {position} {name}"]
