"""内核测试目录发现与批次执行器。

本模块在宿主机侧驱动 QEMU 测试会话，负责两个核心阶段：
1. 目录发现（discover）：启动一次轻量 QEMU 会话，发送 `catalog` 命令并从串口读取
   JSONL 格式的测试套件与用例清单，过程中不执行任何测试固件。
2. 批次执行（execute / execute_batch）：按分组并发启动 QEMU 实例，完成串口握手、
   KTAP 协议流式解析、每阶段超时看门狗切换、宿主机外部观测（UART 压测 / 网络对端）、
   预期致命异常（fatal）核验及失败后的剩余未启动用例续跑。
"""

from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import subprocess
import time

from . import boot
from .catalog import Catalog, HostError, require, run_command
from .protocol import CaseResult, KtapParser
from .qemu_session import CANCELLED, QemuTestSession
from .uart_channel import DeadlineExpired, VMExited
from .resources import resource_names, validate_resources


def config(args):
    """从命令行参数提取目录发现与测试运行共用的核心构建与机器配置字典。"""
    return dict(arch=args.arch, cpus=args.cpus, mode=args.mode, panic=args.panic)


def discover(args, kernel, directory):
    """启动 QEMU 并发送 `catalog` 指令，从串口流式读取测试注册表 JSONL。

    参数:
        args: 命令行解析后的配置命名空间。
        kernel: 待测内核 ELF 文件路径。
        directory: 本次目录发现会话的工作目录（用于存放串口日志等）。

    返回:
        解析并校验完毕的 `Catalog` 对象。若握手失败、输出非法记录或超时则抛出异常。
    """
    catalog = Catalog(config(args))
    vm = QemuTestSession(args, kernel, directory, set())
    deadline = time.monotonic() + args.startup_timeout_seconds
    try:
        # 与内核完成启动握手并请求输出测试目录，此阶段不运行任何测试用例
        vm.handshake(config(args), b"catalog\n", deadline)
        while not catalog.done:
            line = vm.uart.line(deadline)
            if line.startswith("KTEST READY"):
                raise HostError("duplicate READY")
            if line.startswith("{"):
                catalog.feed(json.loads(line))
            else:
                require(not line.strip() or line.lstrip().startswith("#"), "unexpected catalog output")
        # 目录读取完毕后，等待 QEMU 在清理时限内正常关机退出
        finish_vm(vm, time.monotonic() + args.cleanup_timeout_seconds, catalog=True)
        return catalog
    finally:
        vm.close()


def looks_control(value):
    """判断一段串口残留文本是否像未完整输出的 KTAP 或目录 JSONL 控制记录。

    QEMU 退出时，普通的无换行调试打印允许保留在缓冲区末尾；但如果残留内容是半截
    KTAP 结果行或目录 JSONL 记录，说明串口输出被截断，必须判定为协议错误。
    """
    value = value.lstrip()
    if not value:
        return False
    markers = ("KTAP version", "KTEST READY", "# KTEST", "# Subtest:", "ok ", "not ok ", "1..")
    if any(value.startswith(marker) or marker.startswith(value) for marker in markers):
        return True
    if value.startswith("{"):
        kinds = ("catalog", "suite", "case", "catalog_end")
        try:
            row = json.loads(value)
            return isinstance(row, dict) and row.get("type") in kinds
        except json.JSONDecodeError:
            compact = "".join(value.split())
            prefixes = [f'{{"type":"{kind}' for kind in kinds]
            return any(compact.startswith(prefix) or prefix.startswith(compact) for prefix in prefixes)
    return False


def finish_vm(vm, deadline, parser=None, catalog=False):
    """在收到最终协议结束标记后继续消费串口尾部输出，并等待 QEMU 进程正常退出。

    参数:
        vm: 当前 `QemuTestSession` 实例。
        deadline: 关机清理阶段的绝对超时时间戳（`time.monotonic()`）。
        parser: 运行模式下使用的 `KtapParser`，用于继续喂入尾部诊断行。
        catalog: 是否处于目录发现模式。
    """
    while True:
        try:
            line = vm.uart.line(deadline)
        except VMExited:
            # 串口到达 EOF，检查行缓冲区中是否残留了半截控制协议记录
            fragment = bytes(vm.uart.buffer).decode("utf-8", errors="replace").rstrip("\r")
            require(not looks_control(fragment), "partial control record at EOF")
            if parser is not None and parser.case_abort is not None:
                require(not fragment.strip(), "residual UART bytes after case-abort")
            # 串口断开后等待 QEMU 子进程彻底退出，校验退出码为 0 并核验磁盘等外部资源状态
            while vm.process.poll() is None:
                vm.check(deadline)
                time.sleep(min(0.01, max(0, deadline - time.monotonic())))
            vm.check(deadline)
            require(vm.process.returncode == 0, f"QEMU failed after final: {vm.process.returncode}")
            vm.verify_resources(deadline)
            return
        if catalog:
            require(not looks_control(line), "control record after catalog_end")
        else:
            require(not (line.lstrip().startswith("{") and looks_control(line)),
                    "catalog control record after run complete")
            parser.feed(line)




def result_row(item, status, reason, directory, case_abort=None):
    """构造单个测试用例的结果字典，供汇总报告和 JUnit XML 生成使用。

    若用例触发了中止协议（`case_abort`），会将结构化中止信息一并附入原因描述。
    """
    case = item.case
    row = dict(name=case["name"], suite=case["suite"], case=case["case"], parameter=case["parameter"],
               source=case["source"], line=case["line"], repeat=item.repeat, seed=item.seed,
               order=item.order, status=status, reason=reason, log=str(directory / "uart.log"))
    if case_abort is not None:
        row["case_abort"] = dict(case_abort)
        row["reason"] += "\ncase-abort: " + json.dumps(case_abort, ensure_ascii=False, sort_keys=True)
    return row


def suite_error_row(suite, item, error, directory):
    """构造测试套件级（suite setup / cleanup 阶段）错误的独立报告记录。

    套件级夹具错误独立记录为 `SuiteError`，避免挂靠到尚未开始的测试用例上。
    """
    return dict(name=f"{suite['name']} [SuiteError:{error['phase']}]", suite=suite["suite"],
                case=None, parameter=None, source=None, line=None, repeat=item.repeat,
                seed=item.seed, order=item.order, status="Error", kind="SuiteError",
                reason=f"{error['phase']}: {error['reason']}", suite_error=dict(error),
                log=str(directory / "uart.log"))


def finish_case(vm, event, case_deadline, deadline):
    """在单个用例收到客体 KTAP 结果行后，完成宿主机侧外部观测核验并发送握手回执。

    若启用了执行日志 UART 观测或宿主机网络对端（`net_user`），先完成它们的数据核验，
    随后向串口发送 `KTEST PEER END <case_id>` 通知内核放行后续流程。
    """
    errors = []
    try:
        vm.uart.execution_uart.finish(event.status)
    except (HostError, OSError, ValueError) as error:
        errors.append("UART observation: " + str(error))
    if vm.net_user is not None:
        try:
            vm.net_user.finish(event.status, case_deadline)
        except (HostError, OSError, ValueError) as error:
            errors.append("network observation: " + str(error))
        vm.uart.send(f"KTEST PEER END {event.case_id}\n".encode("ascii"), deadline)
    vm.check(deadline)
    require(not errors, f"Guest {event.status}; " + "; ".join(errors))


def execute_batch(args, kernel, catalog, items, directory):
    """在单个串行批次中依次运行一组同属一个测试套件的用例。

    若批次中途因预期致命异常（`expected_fatal`）或明确的用例中止协议（`case_abort`）
    导致 QEMU 干净退出，且后续用例尚未启动，则自动创建新的 `part-N` 子会话继续执行剩余用例。
    """
    output = []
    pending = list(items)
    attempt = 0
    while pending:
        if CANCELLED.is_set():
            output.extend(result_row(item, "NotRun", "host run cancelled before VM start", directory)
                          for item in pending)
            break
        current = directory / f"part-{attempt}"
        current.mkdir()
        attempt += 1
        suite = catalog.suites[pending[0].case["suite"]]
        parser = KtapParser(suite, pending)
        vm = None
        infrastructure = None
        external = None
        safe_suffix = False
        deadline = time.monotonic() + args.startup_timeout_seconds
        setup_deadline = case_deadline = cleanup_deadline = None
        try:
            # 准备本批用例所需的外部磁盘与网络资源，启动 QEMU 并下发 run 命令
            names = resource_names(suite, pending)
            validate_resources(names, args)
            vm = QemuTestSession(args, kernel, current, names)
            vm.handshake(config(args), run_command(pending, args.seed), deadline)
            # 主循环：根据当前所处生命周期阶段选取最早到期的超时时间，逐行解析 KTAP 事件
            while not parser.complete:
                effective = min(d for d in (deadline, setup_deadline, case_deadline, cleanup_deadline) if d is not None)
                event = parser.feed(vm.uart.line(effective))
                now = time.monotonic()
                if event == "suite-setup":
                    setup_deadline = now + args.setup_timeout_seconds
                elif event == "case-abort":
                    cleanup_deadline = effective
                    setup_deadline = case_deadline = None
                    deadline = effective
                elif event == "suite-error":
                    setup_deadline = case_deadline = None
                    if cleanup_deadline is None:
                        cleanup_deadline = now + args.cleanup_timeout_seconds
                    deadline = cleanup_deadline
                elif event == "begin":
                    item = pending[len(parser.results)]
                    vm.uart.execution_uart.begin(item.case)
                    case_deadline = now + min(item.case["timeout_ms"] / 1000, args.case_timeout_seconds)
                    peer = vm.net_user
                    if peer is not None:
                        peer.begin(item.case, case_deadline)
                    setup_deadline = now + args.setup_timeout_seconds
                    cleanup_deadline = None
                    deadline = case_deadline
                    if peer is not None:
                        vm.uart.send(f"KTEST PEER BEGIN {item.case['case']}\n".encode("ascii"),
                                min(case_deadline, setup_deadline))
                elif event == "body":
                    setup_deadline = None
                    vm.uart.execution_uart.body = True
                elif event in ("quiesce", "join", "cleanup"):
                    setup_deadline = None
                    if cleanup_deadline is None:
                        cleanup_deadline = now + args.cleanup_timeout_seconds
                elif isinstance(event, dict):
                    vm.uart.raw_io(event, effective, names)
                elif isinstance(event, CaseResult):
                    try:
                        finish_case(vm, event, case_deadline, effective)
                    except (HostError, OSError, ValueError, DeadlineExpired, VMExited) as error:
                        result = parser.results[event.case_id]
                        result["status"] = "Error"
                        result["reason"] = f"Guest {event.status}; external completion failed: {error}"
                        raise
                    setup_deadline = case_deadline = None
                    if cleanup_deadline is None:
                        cleanup_deadline = now + args.cleanup_timeout_seconds
                    deadline = cleanup_deadline
            finish_vm(vm, cleanup_deadline or time.monotonic() + args.cleanup_timeout_seconds, parser)
            if parser.case_abort is not None:
                # 仅当外层 KTAP 计划完整结束且 QEMU 正常退出后，才将中止的用例登记为 Error，并允许后续未启动用例在下一轮续跑
                parser.results[parser.aborted_case_id] = dict(
                    status="Error", reason=parser.case_abort["reason"], case_abort=parser.case_abort)
                parser.active = None
                safe_suffix = True
        except DeadlineExpired as error:
            external = ("Timeout", str(error))
        except VMExited as error:
            # QEMU 在 KTAP 顶层计划完成前提前退出了串口：检查是否为预期内的致命异常（expected_fatal）
            terminated = vm is not None and vm.process.poll() is not None
            if (parser.active is not None and parser.case_abort is None
                    and parser.fatal is not None and not terminated):
                try:
                    vm.process.wait(timeout=max(0, min(args.cleanup_timeout_seconds, effective - time.monotonic())))
                    terminated = True
                except subprocess.TimeoutExpired:
                    pass
            case = pending[len(parser.results)].case if parser.active is not None else None
            fragment = bytes(vm.uart.buffer).decode("utf-8", errors="replace").rstrip("\r")
            if looks_control(fragment):
                infrastructure = "partial control record at EOF"
            elif case is not None and effective <= time.monotonic():
                external = ("Timeout", "absolute host deadline expired")
            else:
                # 预期致命异常（fatal）必须发生在 body 阶段、类型与预期一致，且 QEMU 自行以返回码 0 退出、串口无残留垃圾字节；
                # 只有满足这些条件，才能确认后续排队的用例尚未被内核触碰，从而安全地在下一轮 QEMU 会话中续跑剩余用例。
                fatal_matches = (case is not None and parser.phase == "body"
                                 and parser.fatal is not None
                                 and parser.armed == parser.fatal == case["expected_fatal"]
                                 and (case["fatal_message"] is None or parser.fatal_message_seen))
                safe_suffix = (case is not None and parser.case_abort is None
                               and parser.fatal is not None and terminated
                               and vm.process.returncode == 0
                               and not vm.uart.buffer.strip())
                if safe_suffix:
                    try:
                        vm.verify_resources(effective)
                        if fatal_matches:
                            boot.verify_trap(args.arch, case, vm.directory, parser.fatal)
                    except DeadlineExpired as resource_error:
                        external = ("Timeout", str(resource_error))
                        safe_suffix = False
                    except (HostError, OSError, ValueError, KeyError) as resource_error:
                        infrastructure = str(resource_error)
                        safe_suffix = False
                if safe_suffix and fatal_matches:
                    parser.results[parser.active] = dict(
                        status="Pass", reason=f"expected fatal: {parser.fatal}; QEMU={vm.process.returncode}")
                    parser.active = None
                else:
                    external = external or ("Crash", str(error))
        except (HostError, OSError, ValueError, UnicodeError) as error:
            infrastructure = str(error)
        finally:
            if vm is not None:
                vm.close()
        # 汇总本轮已完成的用例与当前中断在途的用例结果
        for item in pending:
            cid = item.case["case"]
            if cid in parser.results:
                result = parser.results[cid]
                peer = vm.net_user if vm is not None else None
                peer_error = peer.failures.get(cid) if peer is not None else None
                if result["status"] == "Pass" and peer_error:
                    result = dict(result, status="Error", reason=peer_error)
                output.append(result_row(item, result["status"], result["reason"], current,
                                         result.get("case_abort")))
            elif cid == parser.active:
                status, reason = ("Error", infrastructure) if infrastructure else external or ("Error", "missing final result")
                output.append(result_row(item, status, reason, current, parser.case_abort))
        # 处理本批次中尚未收到 case-begin 确认的剩余用例
        unstarted = [item for item in pending if item.case["case"] not in parser.started]
        if parser.case_abort is not None:
            if safe_suffix and not CANCELLED.is_set():
                pending = unstarted
                continue
            for item in unstarted:
                output.append(result_row(
                    item, "NotRun", "Host did not confirm case start; incomplete abort protocol", current))
            if infrastructure or external:
                aborted = next(row for row in output if row["case"] == parser.aborted_case_id)
                aborted["infrastructure_error"] = infrastructure or (
                    "VM failed case-abort protocol/shutdown completion: " + external[1])
            break
        suite_error = parser.suite_error
        if (suite_error is None and parser.suite_started and not parser.started
                and (infrastructure or external)):
            suite_error = dict(phase="setup", reason=infrastructure or external[1])
        elif (suite_error is None and external and parser.active is None
              and len(parser.results) == len(pending)):
            suite_error = dict(phase="cleanup", reason=external[1])
        if suite_error is not None:
            for item in pending:
                if item.case["case"] not in parser.started:
                    output.append(result_row(item, "NotRun",
                                             f"suite {suite_error['phase']}: {suite_error['reason']}", current))
            row = suite_error_row(suite, pending[-1], suite_error, current)
            if infrastructure:
                row["infrastructure_error"] = infrastructure
            if external:
                row["infrastructure_error"] = "VM failed protocol/shutdown completion: " + external[1]
            output.append(row)
            break
        if external and parser.active is None and parser.results and output:
            output[-1]["infrastructure_error"] = "VM failed protocol/shutdown completion: " + external[1]
        if safe_suffix and not CANCELLED.is_set():
            pending = unstarted
            continue
        if infrastructure or (external and not parser.started):
            reason = infrastructure or external[1]
            # 若在收到任何 case-begin 之前就发生崩溃或基础设施错误，无法保证内核状态安全，直接将未启动项标为 NotRun 并终止本批次
            for item in unstarted:
                output.append(result_row(item, "NotRun",
                                         "Host did not confirm case start; incomplete protocol: " + reason,
                                         current))
            if not parser.active and not parser.results and output:
                output[-len(unstarted)]["infrastructure_error"] = reason
            elif infrastructure and output:
                output[-1]["infrastructure_error"] = infrastructure
            break
        if external:
            # 发生非预期的超时或崩溃时，串口缓冲区中可能仍有未刷出的记录，不再自动重放剩余用例
            for item in unstarted:
                output.append(result_row(item, "NotRun",
                                         "Host did not confirm case start; incomplete protocol: " + external[1],
                                         current))
            break
        if unstarted:
            for item in unstarted:
                output.append(result_row(item, "NotRun",
                                         "Host did not confirm case start; incomplete protocol", current))
        break
    return output


def execute(args, kernel, catalog, groups, directory):
    """使用线程池并发执行多个测试批次，并按原始调用顺序汇总返回所有结果。

    并发粒度为独立的 QEMU 虚拟机批次（`vm-00000`、`vm-00001` 等）；每个批次内部
    仍按顺序串行执行，若收到 Ctrl-C 中断则设置全局取消标志并标记未启动批次为 `NotRun`。
    """
    completed = {}
    pool = ThreadPoolExecutor(max_workers=args.jobs)
    futures = {}
    interrupted = False
    try:
        for index, group in enumerate(groups):
            target = directory / f"vm-{index:05d}"
            target.mkdir()
            futures[pool.submit(execute_batch, args, kernel, catalog, group, target)] = index
        for future in as_completed(futures):
            completed[futures[future]] = future.result()
    except KeyboardInterrupt:
        interrupted = True
        CANCELLED.set()
        for future in futures:
            future.cancel()
    except BaseException:
        CANCELLED.set()
        for future in futures:
            future.cancel()
        raise
    finally:
        pool.shutdown(wait=True, cancel_futures=True)
    if interrupted:
        started_groups = set()
        for future, index in futures.items():
            if not future.cancelled():
                started_groups.add(index)
                if index not in completed:
                    completed[index] = future.result()
        for index, group in enumerate(groups):
            if index not in started_groups:
                target = directory / f"vm-{index:05d}"
                completed[index] = [result_row(item, "NotRun", "host interrupted before VM start", target)
                                    for item in group]
        results = [row for rows in completed.values() for row in rows]
        if results:
            results[0]["infrastructure_error"] = "host run interrupted"
    results = [row for rows in completed.values() for row in rows]
    return sorted(results, key=lambda row: row["order"])
