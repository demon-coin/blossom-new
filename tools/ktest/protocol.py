"""ktest 串口握手、KTAP 状态机与生命周期协议解析器。

本模块负责解析客体内核通过串口输出的 KTAP 测试结果流与 `# KTEST ...` 扩展控制行：
- 校验启动握手 `KTEST READY` 的架构、CPU 核数、构建模式与 panic 策略；
- 严格按外层计划、子测试头、内层计划、套件启动、用例生命周期阶段（setup -> body -> quiesce -> join -> cleanup）
  及最终完成标记驱动状态机；
- 识别预期致命异常（armed/fatal）、清理失败中止（case-abort）、套件级错误（suite-error）以及原始串口 I/O 声明；
- 拦截格式错误或顺序颠倒的协议记录，防止客体业务日志被误判为测试通过。
"""

from dataclasses import dataclass
import json
import re
from .catalog import HostError, require

READY = re.compile(r"KTEST READY 1 arch=(riscv64|loongarch64) cpus=(\d+) mode=(debug|release) panic=(unwind|abort)")
RESULT = re.compile(r"(  )?(not ok|ok) (\d+) (.+)")
CONTROL = re.compile(r"  # KTEST (case-begin|phase|armed|fatal|io|case-abort) (\d+)(?: (.*))?")
SUITE_CONTROL = re.compile(r"  # KTEST (suite-begin|suite-error) (setup|execution|cleanup)(?: ([^\x00-\x1f\x7f]+))?")
PHASES = ("setup", "body", "quiesce", "join", "cleanup")
FATAL_KINDS = ("illegal", "refill", "irq-schedule", "preempt-schedule", "oom",
               "timekeeper", "panic", "double-panic", "abi")


def ready(line, config):
    """解析并校验 `KTEST READY` 握手行，确保客体报告的架构、核数、构建模式与 panic 策略与宿主期望完全一致。"""
    match = READY.fullmatch(line)
    require(match is not None, "malformed READY")
    arch, cpus, mode, panic = match.groups()
    require(dict(arch=arch, cpus=int(cpus), mode=mode, panic=panic) == config,
            "READY configuration mismatch")


@dataclass(frozen=True)
class CaseResult:
    """经完整 KTAP 结果行确认的单个测试用例执行结果（用例编号、状态与可选原因）。"""

    case_id: int
    status: str
    reason: str


class KtapParser:
    """解析单台 QEMU 虚拟机内单个测试套件的 KTAP 输出流与生命周期控制指令的状态机。"""

    def __init__(self, suite, items):
        """初始化套件解析状态机，按命令行请求的用例顺序记录期望列表。"""
        self.suite = suite
        self.items = items
        self.results = {}
        self.started = set()
        self.active = None
        self.phase = None
        self.armed = None
        self.fatal = None
        self.fatal_message_seen = False
        self.version = False
        self.outer_plan = False
        self.inner_plan = False
        self.inner_version = False
        self.subtest = False
        self.suite_started = False
        self.suite_error = None
        self.case_abort = None
        self.aborted_case_id = None
        self.outer_result = False
        self.complete = False

    def feed(self, line):
        """向状态机输入单行串口文本，返回生命周期事件、原始 UART I/O 声明或 `CaseResult`；普通业务日志返回 `None`。"""
        # 1. 解析 KTAP 外层与子测试头部声明，以及最终的 `# KTEST complete` 结束标记
        if line == "KTAP version 1":
            require(not self.version and not self.outer_plan, "duplicate KTAP version")
            self.version = True
            return None
        if line == "1..1":
            require(self.version and not self.outer_plan, "invalid outer plan")
            self.outer_plan = True
            return None
        if line == f"  # Subtest: {self.suite['name']}":
            require(self.outer_plan and not self.subtest, "invalid subtest")
            self.subtest = True
            return None
        if line == "  KTAP version 1":
            require(self.subtest and not self.inner_plan and not self.inner_version, "invalid inner version")
            self.inner_version = True
            return None
        if line.startswith("  1.."):
            require(self.subtest and not self.inner_plan and line == f"  1..{len(self.items)}",
                    "invalid inner plan")
            self.inner_plan = True
            return None
        if line == "# KTEST complete":
            require(self.outer_result and not self.complete, "premature or duplicate complete")
            self.complete = True
            return "complete"
        # 2. 解析套件级控制行（suite-begin 与 suite-error），校验其所处生命周期位置
        suite_control = SUITE_CONTROL.fullmatch(line)
        if suite_control:
            require(self.inner_plan and not self.outer_result and not self.complete
                    and self.case_abort is None, "suite control outside suite")
            kind, phase, reason = suite_control.groups()
            if kind == "suite-begin":
                require(phase == "setup" and reason is None and not self.suite_started
                        and not self.started and self.suite_error is None, "invalid suite-begin")
                self.suite_started = True
                return "suite-setup"
            require(self.suite_started and self.suite_error is None and reason is not None
                    and reason.strip() and self.active is None, "invalid suite-error")
            require(phase != "setup" or not self.started, "suite setup error after case-begin")
            require(phase != "cleanup" or len(self.results) == len(self.items),
                    "suite cleanup error before case results")
            self.suite_error = dict(phase=phase, reason=reason)
            return "suite-error"
        # 3. 解析用例级控制行（case-begin、case-abort、phase、armed、fatal、io）
        control = CONTROL.fullmatch(line)
        if control:
            require(not self.complete and not self.outer_result and self.inner_plan
                    and self.suite_started and self.suite_error is None
                    and self.case_abort is None, "control outside suite")
            require(self.fatal is None, "control after actual fatal")
            kind, cid, value = control.groups()
            cid = int(cid)
            if kind == "case-begin":
                require(value is None and self.active is None and not self.outer_result, "overlapping case")
                position = len(self.results)
                require(position < len(self.items) and self.items[position].case["case"] == cid,
                        "case-begin identity/order mismatch")
                require(cid not in self.started, "duplicate case-begin")
                self.started.add(cid)
                self.active = cid
                self.phase = None
                self.armed = None
                self.fatal = None
                self.fatal_message_seen = False
                return "begin"
            require(self.active == cid, "control identity mismatch")
            if kind == "case-abort":
                phase, separator, payload = (value or "").partition(" ")
                require(separator and phase in ("quiesce", "join", "cleanup")
                        and phase == self.phase, "case-abort outside actual cleanup phase")
                # 使用 object_pairs_hook=list 捕获所有键值对，防止重复 JSON 键静默覆盖此前的失败记录
                try:
                    pairs = json.loads(payload, object_pairs_hook=list)
                except json.JSONDecodeError as error:
                    raise HostError("invalid case-abort JSON") from error
                keys = ("primary", "child", "cleanup", "expectations", "reason")
                require(isinstance(pairs, list) and len(pairs) == len(keys)
                        and all(isinstance(pair, tuple) and len(pair) == 2 for pair in pairs),
                        "case-abort must be a JSON object")
                require({key for key, _ in pairs} == set(keys), "invalid case-abort fields")
                ledger = dict(pairs)
                require(all(ledger[key] is None or isinstance(ledger[key], str)
                            for key in ("primary", "child", "cleanup"))
                        and type(ledger["expectations"]) is int and 0 <= ledger["expectations"]
                        and isinstance(ledger["reason"], str), "invalid case-abort field types")
                self.case_abort = dict(phase=phase, **ledger)
                self.aborted_case_id = cid
                return "case-abort"
            if kind == "phase":
                require(value in PHASES, "unknown case phase")
                require((self.phase is None and value == "setup") or
                        (self.phase is not None and PHASES.index(self.phase) < PHASES.index(value)),
                        "case phase regression or missing setup")
                self.phase = value
                if value != "body":
                    self.armed = None
                return value
            if kind == "armed":
                case = self.items[len(self.results)].case
                require(self.phase == "body" and self.armed is None
                        and case["expected_fatal"] is not None and value == case["expected_fatal"],
                        "unexpected fatal arming")
                self.armed = value
                return "armed"
            if kind == "fatal":
                require(self.phase in PHASES and value in FATAL_KINDS,
                        "unknown or out-of-phase actual fatal")
                self.fatal = value
                self.fatal_message_seen = False
                return "fatal"
            # 解析原始 UART 字节流测试声明（支持 duplex 双工或 tx/rx/pause/slow 单向模式）
            require(kind == "io" and self.phase in ("setup", "body", "quiesce", "cleanup"),
                    "raw UART outside fixture phase")
            parts = (value or "").split()
            if parts and parts[0] == "duplex":
                require(len(parts) == 7 and parts[1].isdigit()
                        and re.fullmatch(r"[0-9a-f]{8}", parts[2]) and parts[3].isdigit()
                        and parts[4].isdigit() and re.fullmatch(r"[0-9a-f]{8}", parts[5])
                        and parts[6].isdigit(), "invalid UART duplex declaration")
                _, rx_length, rx_digest, rx_delay, tx_length, tx_digest, tx_delay = parts
                rx_length, rx_delay, tx_length, tx_delay = map(
                    int, (rx_length, rx_delay, tx_length, tx_delay))
                require(rx_length + tx_length <= 64 * 1024 * 1024 and rx_delay <= 60000
                        and tx_delay <= 60000 and (rx_length or tx_length)
                        and (rx_length or rx_digest == "00000000")
                        and (tx_length or tx_digest == "00000000")
                        and (rx_length or rx_delay == 0)
                        and (tx_length or tx_delay == 0),
                        "UART duplex declaration exceeds bounds")
                return dict(direction="duplex", rx_length=rx_length, rx_digest=rx_digest,
                            rx_delay_ms=rx_delay, tx_length=tx_length, tx_digest=tx_digest,
                            tx_delay_ms=tx_delay, case=cid)
            require(len(parts) == 4 and parts[0] in ("tx", "rx", "pause", "slow")
                    and parts[1].isdigit() and re.fullmatch(r"[0-9a-f]{8}", parts[2])
                    and parts[3].isdigit(), "invalid UART declaration")
            direction, length, digest, delay = parts
            length, delay = int(length), int(delay)
            require(0 <= length <= 64 * 1024 * 1024 and delay <= 60000
                    and (length != 0 or (direction == "pause" and digest == "00000000")),
                    "UART declaration exceeds bounds")
            return dict(direction=direction, length=length, digest=digest, delay_ms=delay,
                        case=cid)
        # 4. 解析带缩进的内层用例结果行（ok / not ok）与不带缩进的外层套件汇总结果行
        result = RESULT.fullmatch(line)
        if result:
            indent, polarity, number, name = result.groups()
            number = int(number)
            if indent:
                require(self.case_abort is None and self.fatal is None,
                        "ordinary result after non-returning termination")
                position = len(self.results)
                require(self.active is not None and number == position + 1
                        and position < len(self.items), "missing, duplicate or out-of-order result")
                case = self.items[position].case
                expected = case["name"]
                require(self.active == case["case"]
                        and (name == expected or name.startswith(expected + " # ")),
                        "case result identity mismatch")
                directive = name[len(expected):]
                reason = ""
                if directive:
                    instruction, _, reason = directive[3:].partition(" ")
                    require(instruction in ("SKIP", "ERROR"), "unknown case result directive")
                    require(polarity == ("ok" if instruction == "SKIP" else "not ok"),
                            "case result directive contradicts polarity")
                    status = "Skip" if instruction == "SKIP" else "Error"
                else:
                    status = "Pass" if polarity == "ok" else "Fail"
                require(self.phase == "cleanup" or (self.phase == "setup" and status != "Pass"),
                        "case result before cleanup")
                require(status != "Pass" or case["expected_fatal"] is None,
                        "expected fatal requires actual fault and VM termination")
                event = CaseResult(self.active, status, reason)
                self.results[event.case_id] = dict(status=status, reason=reason)
                self.active = None
                self.phase = None
                return event
            # 外层套件汇总结果行：必须所有内层用例均已结束（或已触发套件错误/用例中止），且汇总极性与实际结果一致
            require(self.inner_plan and self.suite_started
                    and (self.active is None or self.case_abort is not None)
                    and (len(self.results) == len(self.items) or self.suite_error is not None
                         or self.case_abort is not None)
                    and not self.outer_result and number == 1 and name == self.suite["name"],
                    "invalid outer result")
            passed = self.suite_error is None and self.case_abort is None and all(
                r["status"] in ("Pass", "Skip") for r in self.results.values())
            require(polarity == ("ok" if passed else "not ok"), "false suite PASS")
            self.outer_result = True
            return "outer"
        # 拦截前缀像 KTAP 或 KTEST 协议行但未能通过上方正则校验的畸形行
        if line.lstrip().startswith(("ok ", "not ok ", "KTAP version", "KTEST READY", "1..", "# KTEST", "# Subtest:")):
            raise HostError(f"malformed or unexpected KTAP record: {line[:160]}")
        # 普通业务日志不改变协议状态机；若当前用例已触发预期致命异常，则顺带检查是否出现了预期的致命诊断子串
        if self.active is not None and self.fatal is not None:
            message = self.items[len(self.results)].case["fatal_message"]
            if message is not None and message in line:
                self.fatal_message_seen = True
        return None
