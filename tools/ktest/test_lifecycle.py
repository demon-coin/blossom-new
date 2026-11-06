"""宿主侧 ktest VM 生命周期、超时控制、崩溃续跑与测试报告生成回归测试。

本模块验证 `ktest.batch`、`ktest.uart_channel` 与 `ktest.net_user` 在测试执行全过程中的行为：
1. 绝对超时控制、进程退出同步以及末尾串口缓冲区排空与非法尾缀拦截；
2. 用例失败、套件级错误（`suite-error`）与用例清理中止（`case-abort`）的状态流转与报告导出；
3. 用户态网络对端（`NetUserPeer`）握手回执与跨用例消息收发时序；
4. 致命错误（`fatal`）或进程异常退出后的剩余用例续跑边界与防重复执行保护。
运行方式：`python3 -m unittest discover -s tools/ktest`。
"""

import io
import json
import os
from pathlib import Path
import select as select_io
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

from host_helpers import (CLI, CONFIG, case_record, case_lines, catalog_of,
                          options, parser_for, session_fixture)
from ktest.catalog import HostError, invocations
from ktest.net_user import NetUserPeer
from ktest.batch import execute_batch, finish_vm
from ktest.uart_channel import DeadlineExpired, UartChannel


class TerminalLifecycleTests(unittest.TestCase):
    """验证串口通道的绝对超时截止时间、VM 进程退出等待与末尾串口输出收尾顺序。"""

    def test_buffered_log_flood_cannot_extend_deadline(self):
        """验证即使串口接收缓冲区中已积压大量日志行，逐行读取时仍会检查绝对超时截止时间。"""
        vm = session_fixture()
        vm.uart.buffer = bytearray(b"log\n" * 1000)
        with self.assertRaises(DeadlineExpired):
            vm.uart.line(time.monotonic() - 1)

    def test_final_waits_for_exit_and_rejects_late_or_partial_control(self):
        """验证收到 `# KTEST complete` 后等待连接关闭，允许普通诊断日志但拒绝迟到的控制行或残缺结果行。"""
        for payload, rejected in ((b'ordinary diagnostic\n{"type":"business","value":1}\nunfinished business log', False),
                                  (b"# KTEST complete\n", True), (b"  ok 1", True),
                                  (b'{"type":"catalog_end"', True)):
            with self.subTest(payload=payload), tempfile.TemporaryDirectory() as temporary:
                row = case_record()
                parser = parser_for([row])
                for line in case_lines(row):
                    parser.feed(line)
                parser.feed("ok 1 os::tests0")
                parser.feed("# KTEST complete")
                host, guest = socket.socketpair()
                try:
                    vm = session_fixture()
                    vm.uart.serial = host
                    host.setblocking(False)
                    vm.uart.buffer = bytearray()
                    vm.uart.raw = io.BytesIO()
                    vm.process = SimpleNamespace(poll=lambda: 0, returncode=0)
                    guest.sendall(payload)
                    guest.shutdown(socket.SHUT_WR)
                    if rejected:
                        with self.assertRaises(HostError):
                            finish_vm(vm, time.monotonic() + 2, parser)
                    else:
                        finish_vm(vm, time.monotonic() + 2, parser)
                        self.assertEqual(vm.uart.raw.getvalue(), payload)
                finally:
                    host.close()
                    guest.close()

    def test_uart_exit_does_not_skip_bytes_after_readiness_check(self):
        """验证当子进程在首次 `select` 就绪检查返回空之后才写入末尾输出并退出时，宿主仍能完整排空管道并拦截非法控制行。"""
        for payload, rejected in ((b"final diagnostic\n", False),
                                  (b"# KTEST complete\n", True), (b"  ok 1", True)):
            with self.subTest(payload=payload):
                row = case_record()
                parser = parser_for([row])
                for line in case_lines(row):
                    parser.feed(line)
                parser.feed("ok 1 os::tests0")
                parser.feed("# KTEST complete")
                host, guest = socket.socketpair()
                script = ("import socket,sys\n"
                          "uart=socket.socket(fileno=int(sys.argv[1]))\n"
                          "sys.stdin.buffer.read(1)\n"
                          "uart.sendall(bytes.fromhex(sys.argv[2]))\n"
                          "uart.close()\n")
                process = subprocess.Popen(
                    [sys.executable, "-c", script, str(guest.fileno()), payload.hex()],
                    stdin=subprocess.PIPE, pass_fds=(guest.fileno(),))
                guest.close()
                original_select = select_io.select
                crossed = False

                def exit_after_readiness(readers, writers, errors, timeout):
                    """首次调用返回空就绪集并唤醒子进程写入末尾数据并退出，后续调用恢复正常 `select`。"""
                    nonlocal crossed
                    if not crossed:
                        crossed = True
                        result = original_select(readers, writers, errors, 0)
                        self.assertEqual(result, ([], [], []))
                        process.stdin.write(b"G")
                        process.stdin.flush()
                        self.assertEqual(process.wait(timeout=2), 0)
                        return result
                    return original_select(readers, writers, errors, timeout)

                try:
                    vm = session_fixture()
                    vm.uart.serial = host
                    host.setblocking(False)
                    vm.uart.buffer = bytearray()
                    vm.uart.raw = io.BytesIO()
                    vm.process = process
                    with patch("ktest.uart_channel.select.select", side_effect=exit_after_readiness):
                        if rejected:
                            with self.assertRaises(HostError):
                                finish_vm(vm, time.monotonic() + 2, parser)
                        else:
                            finish_vm(vm, time.monotonic() + 2, parser)
                            self.assertEqual(vm.uart.raw.getvalue(), payload)
                finally:
                    host.close()
                    process.stdin.close()
                    if process.poll() is None:
                        process.terminate()
                    process.wait(timeout=2)


class LifecycleAndReportTests(unittest.TestCase):
    """验证批次执行中的普通失败、网络对端同步、套件错误、用例清理中止与结果报告导出。"""

    def execute_serial(self, rows, lines, directory, args=None, guest_tail=""):
        """启动模拟客体串口输出的子进程，并通过 `execute_batch` 运行单批次测试返回结果列表。"""
        args = args or options()
        host, guest = socket.socketpair()
        script = (
            "import socket,sys,time\n"
            "uart=socket.socket(fileno=int(sys.argv[1]))\n"
            "uart.sendall(b'KTEST READY 1 arch=riscv64 cpus=2 mode=debug panic=unwind\\n')\n"
            "command=b''\n"
            "while not command.endswith(b'\\n'): command+=uart.recv(1024)\n"
            "uart.sendall(sys.argv[2].encode('utf-8'))\n"
            "uart.close()\n"
            + guest_tail + "\n"
        )
        process = subprocess.Popen([sys.executable, "-c", script, str(guest.fileno()),
                                    "\n".join(lines) + "\n"], pass_fds=(guest.fileno(),))
        guest.close()
        vm = session_fixture()
        vm.args = vm.uart.args = args
        vm.directory = vm.uart.directory = directory
        vm.uart.serial = host
        host.setblocking(False)
        vm.uart.buffer = bytearray()
        vm.uart.raw = io.BytesIO()
        vm.process = process
        vm.connect = lambda deadline: None

        def close():
            """关闭宿主端串口套接字并确保模拟客体的子进程已终止回收。"""
            host.close()
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=2)

        vm.close = close
        with patch("ktest.batch.QemuTestSession", return_value=vm) as factory:
            results = execute_batch(args, Path("kernel"), catalog_of(rows),
                                    invocations(rows, args), directory)
        self.assertEqual(factory.call_count, 1, "suite failure must not retry unstarted cases")
        return results

    def test_ordinary_failure_continues_same_vm_and_all_skip_remains_distinct(self):
        """验证普通 `Fail` 用例走完清理阶段后允许后续用例在同一 VM 中继续执行，且全部 `Skip` 时返回独立退出码 4。"""
        rows = [case_record(0), case_record(1)]
        for statuses, outer, code in ((("Fail", "Pass"), "not ok", 1),
                                      (("Skip", "Skip"), "ok", 4)):
            with self.subTest(statuses=statuses), tempfile.TemporaryDirectory() as temporary:
                lines = ["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                         "  1..2", "  # KTEST suite-begin setup"]
                for position, (row, status) in enumerate(zip(rows, statuses), 1):
                    lines.extend(case_lines(row, position, status))
                lines.extend([f"{outer} 1 os::tests0", "# KTEST complete"])
                results = self.execute_serial(rows, lines, Path(temporary))
                self.assertEqual([row["status"] for row in results], list(statuses))
                self.assertEqual(CLI.exit_status(results), code)

    def test_network_peer_ack_orders_real_consumers_and_successor(self):
        """验证 `NetUserPeer` 的 `BEGIN` 与 `END` 回执严格界定每个用例的网络收发窗口，防止跨用例消息串扰。"""
        rows = [case_record(index, tags=["network-nic-native"], resources=["net-user"])
                for index in range(2)]
        script = """
import json,os,select,socket,sys,time
uart=socket.socket(fileno=int(sys.argv[1]))
ports_reader,gate_reader,feedback_writer=map(int,sys.argv[2:5])
rows=json.loads(sys.argv[5])
deadline=time.monotonic()+5

def ready(readers):
    '''所有协调和真实 socket 读取共用子进程的一次绝对期限。'''
    remaining=deadline-time.monotonic()
    assert 0 < remaining, "child absolute deadline expired"
    readable,_,_=select.select(readers,[],[],remaining)
    assert readable, "child absolute deadline expired"
    return readable

def line():
    '''逐字节消费真实 UART，避免吞掉后继回执或运行命令。'''
    value=b""
    while not value.endswith(b"\\n"):
        ready([uart])
        part=uart.recv(1)
        assert part, "UART EOF before acknowledgement"
        value+=part
        assert len(value)<=1024, "UART line exceeds test bound"
    return value

def acknowledgement(phase,cid):
    '''只有完整真实回执才放行，窗口管道同时记录客体是否已经越过消费边界。'''
    expected=f"KTEST PEER {phase} {cid}\\n".encode("ascii")
    gate_seen=False
    while True:
        readable=ready([uart,gate_reader])
        if uart in readable:
            assert line()==expected, "wrong peer acknowledgement"
            return gate_seen
        marker=os.read(gate_reader,1)
        assert marker, "window pipe closed"
        matching=(phase=="BEGIN" and marker==b"B") or (phase=="END" and marker==b"F")
        os.write(feedback_writer,b"b" if phase=="BEGIN" and matching else
                 (b"f" if phase=="END" and matching else b"x"))
        gate_seen=matching

ready([ports_reader])
port=int(os.read(ports_reader,32))
udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp.bind(("127.0.0.1",0))
uart.sendall(b"KTEST READY 1 arch=riscv64 cpus=2 mode=debug panic=unwind\\n")
assert line().startswith(b"run "), "missing real run command"
uart.sendall(b"KTAP version 1\\n1..1\\n  # Subtest: os::tests0\\n"
             b"  1..2\\n  # KTEST suite-begin setup\\n")
for position,row in enumerate(rows,1):
    cid=row["case"]
    uart.sendall(f"  # KTEST case-begin {cid}\\n".encode("ascii"))
    begin_gate_seen=acknowledgement("BEGIN",cid)
    uart.sendall(f"  # KTEST phase {cid} setup\\n  # KTEST phase {cid} body\\n".encode("ascii"))
    assert udp.sendto(b"P",("127.0.0.1",port))==1
    if not begin_gate_seen:
        os.write(feedback_writer,b"p")
    ready([udp])
    data,address=udp.recvfrom(2)
    assert data==b"Q" and address==("127.0.0.1",port), "real native peer response missing"
    uart.sendall((f"  # KTEST phase {cid} cleanup\\n"
                  f"  ok {position} {row['name']}\\n").encode("utf-8"))
    acknowledgement("END",cid)
uart.sendall(b"ok 1 os::tests0\\n# KTEST complete\\n")
udp.close()
uart.close()
os._exit(0)
"""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            host, guest = socket.socketpair()
            ports_reader, ports_writer = os.pipe()
            gate_reader, gate_writer = os.pipe()
            feedback_reader, feedback_writer = os.pipe()
            process = subprocess.Popen(
                [sys.executable, "-c", script, str(guest.fileno()), str(ports_reader),
                 str(gate_reader), str(feedback_writer), json.dumps(rows)],
                pass_fds=(guest.fileno(), ports_reader, gate_reader, feedback_writer))
            guest.close()
            os.close(ports_reader)
            os.close(gate_reader)
            os.close(feedback_writer)
            windows = []
            peers = []
            raw = None

            def window(marker, deadline):
                """向门控管道写入标记并等待子进程从反馈管道回传状态字节，以此精确观测子进程所处的同步窗口。"""
                os.write(gate_writer, marker)
                readable, _, _ = select_io.select(
                    [feedback_reader], [], [], max(0, deadline - time.monotonic()))
                self.assertTrue(readable, "child did not reach the fixed consumer window")
                feedback = os.read(feedback_reader, 1)
                self.assertTrue(feedback, "child closed before the fixed consumer window")
                return feedback

            def construct_vm(args, kernel, current, names):
                """构造挂载了 `NetUserPeer` 与子进程串口套接字的测试会话，用于拦截并检验 `begin` / `finish` 时序。"""
                nonlocal raw
                self.assertIn("net-user", names)
                peer = NetUserPeer(current)
                peers.append(peer)
                os.write(ports_writer, str(peer.ports[1]).encode("ascii"))
                vm = session_fixture()
                vm.args = args
                vm.directory = current
                vm.uart.args = args
                vm.uart.directory = current
                vm.uart.serial = host
                host.setblocking(False)
                vm.uart.buffer = bytearray()
                raw = (current / "uart.log").open("xb", buffering=0)
                vm.uart.raw = raw
                vm.process = process
                vm.net_user = peer
                original_begin = peer.begin
                original_finish = peer.finish

                def begin(case, deadline):
                    """在向客体发送 `BEGIN` 回执前探测子进程状态；若子进程提前发包则确认数据已到达 UDP 套接字。"""
                    feedback = window(b"B", deadline)
                    windows.append(("begin", case["case"], feedback))
                    if feedback == b"p":
                        readable, _, _ = select_io.select(
                            [peer.udp], [], [], max(0, deadline - time.monotonic()))
                        self.assertTrue(readable, "early producer did not reach the actual UDP socket")
                    original_begin(case, deadline)

                def finish(status, deadline):
                    """在回收工作线程并发送 `END` 回执前探测子进程状态，确保后继用例尚未提前写入串口。"""
                    feedback = window(b"F", deadline)
                    windows.append(("finish", peer.active["row"]["case"], feedback))
                    if feedback == b"x":
                        readable, _, _ = select_io.select(
                            [host], [], [], max(0, deadline - time.monotonic()))
                        self.assertTrue(readable, "successor did not enter the actual UART socket")
                    original_finish(status, deadline)

                def connect(deadline):
                    """跳过 QEMU 启动，仅在握手前检查模拟子进程是否存活。"""
                    vm.check(deadline)

                def close():
                    """依次终止回收子进程、关闭 `NetUserPeer`、串口套接字与原始日志文件。"""
                    if process.poll() is None:
                        process.terminate()
                    process.wait(timeout=2)
                    peer.close()
                    host.close()
                    raw.close()

                peer.begin = begin
                peer.finish = finish
                vm.connect = connect
                vm.close = close
                return vm

            try:
                args = options(startup_timeout_seconds=2, setup_timeout_seconds=2, case_timeout_seconds=2, cleanup_timeout_seconds=2)
                with patch("ktest.batch.QemuTestSession", side_effect=construct_vm) as factory:
                    results = execute_batch(args, Path("kernel"), catalog_of(rows),
                                            invocations(rows, args), directory)
                self.assertEqual(factory.call_count, 1)
                self.assertEqual([row["status"] for row in results], ["Pass", "Pass"])
                self.assertEqual(process.returncode, 0)
                self.assertEqual(windows, [("begin", 0, b"b"), ("finish", 0, b"f"),
                                           ("begin", 1, b"b"), ("finish", 1, b"f")])
                report = json.loads((directory / "part-0" / "peer.json").read_text())
                self.assertFalse(report["failures"])
                self.assertEqual(len(report["cases"]), 2)
                for row in report["cases"]:
                    received = [event for event in row["events"] if event["kind"] == "udp-recv"]
                    sent = [event for event in row["events"] if event["kind"] == "udp-send"]
                    self.assertEqual([event["bytes_hex"] for event in received], ["50"])
                    self.assertEqual([event["bytes_hex"] for event in sent], ["51"])
                    self.assertEqual(received[0]["source"], sent[0]["destination"])
                    self.assertTrue(row["complete"])
                    self.assertFalse(row["errors"])
                    self.assertIsNotNone(row["thread_exited_at"])
                    self.assertIsNotNone(row["joined_at"])
                self.assertIsNotNone(report["closed_at"])
            finally:
                if process.poll() is None:
                    process.terminate()
                process.wait(timeout=2)
                for peer in peers:
                    peer.close()
                host.close()
                if raw is not None:
                    raw.close()
                for descriptor in (ports_writer, gate_writer, feedback_reader):
                    os.close(descriptor)

    def test_suite_errors_keep_case_states_and_independent_json_junit_error(self):
        """验证套件在 `setup`、`execution`、`cleanup` 阶段报错时保留各用例真实状态，并在 JSON 与 JUnit 报告中生成独立的 `SuiteError` 记录。"""
        rows = [case_record(0), case_record(1)]
        for phase, statuses in (("setup", ["NotRun", "NotRun"]),
                                ("execution", ["Fail", "NotRun"]),
                                ("cleanup", ["Pass", "Pass"])):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                lines = ["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                         "  1..2", "  # KTEST suite-begin setup"]
                if phase == "cleanup":
                    for position, row in enumerate(rows, 1):
                        lines.extend(case_lines(row, position))
                elif phase == "execution":
                    lines.extend(case_lines(rows[0], status="Fail"))
                lines.extend([f"  # KTEST suite-error {phase} fixture failed",
                              "not ok 1 os::tests0", "# KTEST complete",
                              "ordinary shutdown diagnostic"])
                results = self.execute_serial(rows, lines, directory)
                cases = [row for row in results if row["case"] is not None]
                errors = [row for row in results if row.get("kind") == "SuiteError"]
                self.assertEqual([row["status"] for row in cases], statuses)
                self.assertEqual(len(errors), 1)
                self.assertEqual(errors[0]["suite_error"], dict(phase=phase, reason="fixture failed"))
                self.assertEqual(errors[0]["status"], "Error")
                self.assertEqual(CLI.exit_status(results), 2)
                CLI.export_reports(directory, dict(results=results, **CONFIG))
                self.assertEqual(json.loads((directory / "results.json").read_text())["results"], results)
                xml = ET.parse(directory / "results.xml")
                for row in cases:
                    node = next(node for node in xml.findall("testcase") if node.attrib["name"] == row["name"])
                    self.assertEqual(node.find("properties/property[@name='status']").attrib["value"],
                                     row["status"])
                    self.assertEqual(node.find("error") is not None, row["status"] == "NotRun")
                node = next(node for node in xml.findall("testcase")
                            if node.attrib["name"] == errors[0]["name"])
                self.assertEqual(node.find("error").attrib["message"], errors[0]["reason"])

    def test_case_abort_cleanup_failures_preserve_independent_ledgers_in_json_and_junit(self):
        """验证在 `quiesce`、`join`、`cleanup` 阶段发生 `case-abort` 时，JSON 与 JUnit 报告完整保留主体、子任务与清理失败账本。"""
        rows = [case_record(0), case_record(1)]
        for phase in ("quiesce", "join", "cleanup"):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                ledger = dict(primary='body "failed"\n主体', child="child returned error",
                              cleanup=f"{phase} cannot converge", expectations=3,
                              reason="活资源不得释放")
                lines = ["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                         "  1..2", "  # KTEST suite-begin setup",
                         *case_lines(rows[0], status="Fail"), *case_lines(rows[1], 2)[:3],
                         f"  # KTEST phase 1 {phase}",
                         f"  # KTEST case-abort 1 {phase} " + json.dumps(ledger),
                         "not ok 1 os::tests0", "# KTEST complete"]
                results = self.execute_serial(rows, lines, directory)
                self.assertEqual([row["status"] for row in results], ["Fail", "Error"])
                self.assertEqual([row["case"] for row in results], [0, 1])
                self.assertFalse(any(row.get("kind") == "SuiteError" for row in results))
                aborted = results[1]
                self.assertNotIn("infrastructure_error", aborted)
                self.assertEqual(aborted["case_abort"], dict(phase=phase, **ledger))
                self.assertEqual(CLI.exit_status(results), 2)
                CLI.export_reports(directory, dict(results=results, **CONFIG))
                self.assertEqual(json.loads((directory / "results.json").read_text())["results"], results)
                nodes = ET.parse(directory / "results.xml").findall("testcase")
                self.assertEqual(nodes[0].find("failure").attrib["type"], "Fail")
                message = nodes[1].find("error").attrib["message"]
                self.assertEqual(message, aborted["reason"])
                self.assertEqual(json.loads(message.split("\ncase-abort: ", 1)[1]),
                                 dict(phase=phase, **ledger))

    def test_case_abort_requires_valid_terminal_protocol_and_real_zero_exit_without_replay(self):
        """验证 `case-abort` 之后必须跟随合法的 `not ok` 汇总行、`# KTEST complete` 以及进程 0 退出，否则标记为基础设施错误且不重跑后续用例。"""
        rows = [case_record(0), case_record(1), case_record(2)]
        ledger = dict(primary=None, child="child failed", cleanup="join unsafe",
                      expectations=1, reason="cannot safely converge")
        abort = "  # KTEST case-abort 1 join " + json.dumps(ledger)
        prefix = ["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                  "  1..3", "  # KTEST suite-begin setup", *case_lines(rows[0]),
                  *case_lines(rows[1], 2)[:3], "  # KTEST phase 1 join", abort]
        ending = ["not ok 1 os::tests0", "# KTEST complete"]
        scenarios = (([], "", "Crash"),
                     (ending[:1], "", "Crash"),
                     (["ok 1 os::tests0", "# KTEST complete"], "", "Error"),
                     ([abort, *ending], "", "Error"),
                     ([*ending, abort], "", "Error"),
                     ([*ending, "  # KTEST suite-error cleanup late"], "", "Error"),
                     ([*ending, "  # KTEST status 1 Error"], "", "Error"),
                     (ending, "import os; os._exit(7)", "Error"),
                     (ending, "time.sleep(1)", "Timeout"),
                     ([], "time.sleep(1)", "Crash"))
        for suffix, tail, status in scenarios:
            with self.subTest(suffix=suffix, tail=tail), tempfile.TemporaryDirectory() as temporary:
                results = self.execute_serial(rows, prefix + suffix, Path(temporary),
                                              options(cleanup_timeout_seconds=0.15), tail)
                self.assertEqual([row["status"] for row in results], ["Pass", status, "NotRun"])
                self.assertEqual(results[1]["case_abort"], dict(phase="join", **ledger))
                self.assertTrue(results[1]["infrastructure_error"])
                self.assertEqual(CLI.exit_status(results), 2)

    def test_case_abort_does_not_renew_existing_absolute_cleanup_deadline(self):
        """验证 `case-abort` 控制行不会重置已启动的清理阶段绝对超时计时器。"""
        rows = [case_record(0), case_record(1), case_record(2)]
        ledger = dict(primary=None, child=None, cleanup="join failed", expectations=0, reason="unsafe")
        lines = ["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                 "  1..3", "  # KTEST suite-begin setup", *case_lines(rows[0], status="Fail"),
                 *case_lines(rows[1], 2)[:3], "  # KTEST phase 1 join", "before-abort progress",
                 "  # KTEST case-abort 1 join " + json.dumps(ledger), "after-abort progress",
                 "not ok 1 os::tests0", "# KTEST complete"]
        clock = [100.0]
        vm = session_fixture()
        vm.uart.buffer = bytearray(("\n".join(lines) + "\n").encode("utf-8"))
        vm.handshake = lambda config, command, deadline: None
        vm.close = lambda: None

        def line(deadline):
            """读取串口缓冲行，并在遇到特定进度日志时推进模拟单调时钟以触发清理超时。"""
            value = UartChannel.line(vm.uart, deadline)
            if value.endswith("progress"):
                clock[0] += 1.5
            return value

        vm.uart.line = line
        args = options(cleanup_timeout_seconds=2)
        with tempfile.TemporaryDirectory() as temporary, \
                patch("ktest.batch.QemuTestSession", return_value=vm) as factory, \
                patch("ktest.batch.time.monotonic", side_effect=lambda: clock[0]):
            results = execute_batch(args, Path("kernel"), catalog_of(rows),
                                    invocations(rows, args), Path(temporary))
        self.assertEqual(factory.call_count, 1)
        self.assertEqual([row["status"] for row in results], ["Fail", "Timeout", "NotRun"])
        self.assertEqual(results[1]["case_abort"], dict(phase="join", **ledger))
        self.assertTrue(results[1]["infrastructure_error"])
        self.assertEqual(CLI.exit_status(results), 2)

    def test_suite_stage_deadlines_are_absolute_and_do_not_rewrite_cases(self):
        """验证套件级 `setup` 与末尾 `cleanup` 阶段均受绝对超时约束，持续输出普通业务日志无法延长截止时间。"""
        rows = [case_record(0), case_record(1)]
        for phase, statuses in (("setup", ["NotRun", "NotRun"]),
                                ("cleanup", ["Pass", "Pass"])):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as temporary:
                lines = ["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                         "  1..2", "  # KTEST suite-begin setup"]
                if phase == "cleanup":
                    for position, row in enumerate(rows, 1):
                        lines.extend(case_lines(row, position))
                lines.extend(["ordinary progress", "ordinary progress"])
                clock = [100.0]
                vm = session_fixture()
                vm.uart.buffer = bytearray(("\n".join(lines) + "\n").encode("utf-8"))
                vm.handshake = lambda config, command, deadline: None
                vm.close = lambda: None

                def line(deadline):
                    """读取串口缓冲行，并在遇到普通进度日志时推进模拟单调时钟。"""
                    value = UartChannel.line(vm.uart, deadline)
                    if value == "ordinary progress":
                        clock[0] += 3
                    return value

                vm.uart.line = line
                args = options(setup_timeout_seconds=2, cleanup_timeout_seconds=2)
                with patch("ktest.batch.QemuTestSession", return_value=vm), \
                        patch("ktest.batch.time.monotonic", side_effect=lambda: clock[0]):
                    results = execute_batch(args, Path("kernel"), catalog_of(rows),
                                            invocations(rows, args), Path(temporary))
                self.assertEqual([row["status"] for row in results if row["case"] is not None],
                                 statuses)
                error = next(row for row in results if row.get("kind") == "SuiteError")
                self.assertEqual(error["suite_error"]["phase"], phase)
                self.assertEqual(CLI.exit_status(results), 2)

    def test_exit_priority_and_same_run_json_junit(self):
        """验证命令行退出码的优先级（`Error` > `Fail` > `Skip` > `Pass`），以及 JSON 与 JUnit 报告内容的一致性。"""
        self.assertEqual(CLI.exit_status([dict(status="Skip")]), 4)
        self.assertEqual(CLI.exit_status([dict(status="Pass")]), 0)
        self.assertEqual(CLI.exit_status([dict(status="Fail"), dict(status="Error")]), 2)
        self.assertEqual(CLI.exit_status([dict(status="NotRun")]), 1)
        row = dict(name="os::s::x[p]", suite=0, case=0, status="Timeout", reason="absolute deadline",
                   repeat=2, seed=123, order=0, log="raw.log")
        report = dict(results=[row], elf_sha256="actual elf", seed=17, **CONFIG)
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            CLI.export_reports(base, report)
            self.assertEqual(json.loads((base / "results.json").read_text())["results"], [row])
            case = ET.parse(base / "results.xml").find("testcase")
            self.assertEqual(case.attrib["name"], row["name"])
            self.assertEqual(case.find("failure").attrib["type"], "Timeout")
            self.assertEqual(case.find("properties/property[@name='seed']").attrib["value"], "123")


class HostFatalReplayTests(unittest.TestCase):
    """通过模拟 QEMU 客体可执行脚本，验证致命错误（`fatal`）判定与剩余未执行用例的安全续跑边界。"""

    def execute_guest(self, rows, body, directory, args=None):
        """生成模拟 QEMU 的可执行 Python 脚本并调用 `execute_batch`，记录每次启动收到的 `run` 命令。"""
        executable = directory / "guest.py"
        executable.write_text(
            "#!" + sys.executable + "\n"
            "import pathlib,socket,sys,time\n"
            f"base=pathlib.Path({str(directory)!r})\n"
            "endpoint=next(value for value in sys.argv if value.startswith('socket,id=ktest,'))"
            ".split('path=',1)[1].split(',',1)[0]\n"
            "listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)\n"
            "listener.bind(endpoint)\nlistener.listen(1)\n"
            "uart,_=listener.accept()\n"
            "uart.sendall(b'KTEST READY 1 arch=riscv64 cpus=2 mode=debug panic=unwind\\n')\n"
            "command=b''\n"
            "while not command.endswith(b'\\n'): command+=uart.recv(1024)\n"
            "with (base/'commands').open('ab') as output: output.write(command)\n"
            "ids=[int(value) for value in command.split()[4].split(b',')]\n"
            + body + "\nuart.close()\nlistener.close()\n",
            encoding="utf-8")
        executable.chmod(0o700)
        args = args or options()
        args.qemu = str(executable)
        args.memory = "128M"
        args.modern = False
        return execute_batch(args, Path("unchanged-kernel.elf"), catalog_of(rows),
                             invocations(rows, args), directory)

    def payload(self, rows, tail):
        """拼接包含 KTAP 头部、套件计划行、首个用例前序阶段行与指定尾部行序列的串口输出文本。"""
        return "\n".join(["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                          f"  1..{len(rows)}", "  # KTEST suite-begin setup",
                          *case_lines(rows[0])[:3], *tail]) + "\n"

    def assert_reports(self, directory, results, statuses, exit_code):
        """断言批次执行结果的状态列表、退出码以及导出的 JSON / JUnit 报告均符合预期。"""
        self.assertEqual([row["status"] for row in results], statuses)
        self.assertEqual(CLI.exit_status(results), exit_code)
        CLI.export_reports(directory, dict(results=results, **CONFIG))
        self.assertEqual(json.loads((directory / "results.json").read_text())["results"], results)
        nodes = ET.parse(directory / "results.xml").findall("testcase")
        self.assertEqual([node.find("properties/property[@name='status']").attrib["value"]
                          for node in nodes], statuses)
        for node, status in zip(nodes, statuses):
            tag = "failure" if status in ("Fail", "Crash", "Timeout") else "error"
            if status in ("Pass", "Skip"):
                self.assertIsNone(node.find("failure"))
                self.assertIsNone(node.find("error"))
            else:
                self.assertEqual(node.find(tag).attrib["type"], status)


    def test_guest_pass_with_missing_external_uart_records_is_real_error(self):
        """验证当客体上报 `ok` 但宿主侧外部串口日志校验发现缺失预期记录时，用例最终判定为 `Error` 且原始串口日志保留不变。"""
        row = case_record(
            name="os::ktest::execution_services::klog::records::allocation_free_record_identity")
        payload = self.payload([row], [*case_lines(row)[3:],
                                     "ok 1 os::tests0", "# KTEST complete"])
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            results = self.execute_guest([row], f"uart.sendall({payload.encode()!r})", directory)
            self.assert_reports(directory, results, ["Error"], 2)
            self.assertIn("Guest Pass", results[0]["reason"])
            self.assertIn(f"  ok 1 {row['name']}\n".encode(),
                          (directory / "part-0" / "uart.log").read_bytes())

    def test_guest_pass_with_incomplete_network_peer_is_real_error(self):
        """验证当客体上报 `ok` 但 `NetUserPeer` 未收到预期的网络交互流量时，用例最终判定为 `Error` 并保留原始串口日志。"""
        row = case_record(tags=["network-nic-udp"], resources=["net-user"])
        start = "\n".join(["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                           "  1..1", "  # KTEST suite-begin setup",
                           case_lines(row)[0]]) + "\n"
        end = "\n".join([*case_lines(row)[1:], "ok 1 os::tests0", "# KTEST complete"]) + "\n"
        body = (f"uart.sendall({start.encode()!r})\n"
                "ack=b''\n"
                "while not ack.endswith(b'\\n'): ack+=uart.recv(1)\n"
                "assert ack==b'KTEST PEER BEGIN 0\\n'\n"
                f"uart.sendall({end.encode()!r})\n"
                "ack=b''\n"
                "while not ack.endswith(b'\\n'): ack+=uart.recv(1)\n"
                "assert ack==b'KTEST PEER END 0\\n'")
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            results = self.execute_guest([row], body, directory)
            self.assert_reports(directory, results, ["Error"], 2)
            peer = json.loads((directory / "part-0" / "peer.json").read_text())
            self.assertTrue(peer["failures"])
            self.assertIn(f"  ok 1 {row['name']}\n".encode(),
                          (directory / "part-0" / "uart.log").read_bytes())

    def test_buffered_executed_successor_is_not_replayed_after_deadline(self):
        """验证当前序用例因日志耗时触发超时时，即使串口缓冲区中后续用例已执行完毕，宿主也不会在新 VM 中重复执行后续用例。"""
        rows = [case_record(0), case_record(1)]
        first = self.payload(rows, ["deadline-consuming-business-log",
                                  *case_lines(rows[0])[3:], *case_lines(rows[1], 2),
                                  "ok 1 os::tests0", "# KTEST complete"])
        successor = self.payload([rows[1]], [*case_lines(rows[1])[3:],
                                             "ok 1 os::tests0", "# KTEST complete"])
        body = ("with (base/'effects').open('ab') as output: output.write(b'B\\n')\n"
                f"uart.sendall(({first!r} if ids==[0,1] else {successor!r}).encode())")
        original_line = UartChannel.line

        def consuming_line(vm, deadline):
            """读取串口行，并在读到特定耗时日志行时休眠以触发用例超时。"""
            line = original_line(vm, deadline)
            if line == "deadline-consuming-business-log":
                time.sleep(0.6)
            return line

        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with patch.object(UartChannel, "line", consuming_line):
                results = self.execute_guest(rows, body, directory, options(case_timeout_seconds=0.5))
            self.assertEqual((directory / "effects").read_bytes(), b"B\n")
            self.assertEqual((directory / "commands").read_bytes(), b"run 0 17 0 0,1\n")
            self.assert_reports(directory, results, ["Timeout", "NotRun"], 1)

    def test_body_return_disarms_expected_fatal_before_quiesce_oom(self):
        """验证用例主体正常返回进入 `quiesce` 阶段后，原本在 `body` 阶段注册的预期致命错误（`armed`）自动失效，此时发生 OOM 仍判定为 `Crash`。"""
        rows = [case_record(expected_fatal="oom")]
        payload = self.payload(rows, ["  # KTEST armed 0 oom",
                                      "  # KTEST phase 0 quiesce", "  # KTEST fatal 0 oom"])
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            results = self.execute_guest(rows, f"uart.sendall({payload.encode()!r})", directory)
            self.assert_reports(directory, results, ["Crash"], 1)
            self.assertNotIn("infrastructure_error", results[0])

    def test_prefatal_business_message_cannot_match_wrong_actual_panic(self):
        """验证预期 panic 消息子串必须出现在 `KTEST fatal` 事件之后；仅在事件前的业务日志中出现目标词而实际 panic 消息不符时判定为 `Crash`。"""
        rows = [case_record(expected_fatal="panic", fatal_message="wanted")]
        payload = self.payload(rows, ["  # KTEST armed 0 panic", "business wanted",
                                      "  # KTEST fatal 0 panic", "actual panic: wrong"])
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            results = self.execute_guest(rows, f"uart.sendall({payload.encode()!r})", directory)
            self.assert_reports(directory, results, ["Crash"], 1)
            self.assertNotIn("infrastructure_error", results[0])

    def test_valid_unexpected_or_mismatched_fatal_is_crash_not_infrastructure(self):
        """验证未提前声明 `armed` 或实际故障类型与预期不符的合法 `KTEST fatal` 事件判定为 `Crash`，而不标记为协议基础设施错误。"""
        for expected, tail in ((None, ["  # KTEST fatal 0 oom"]),
                               ("illegal", ["  # KTEST armed 0 illegal",
                                            "  # KTEST fatal 0 oom"])):
            with self.subTest(expected=expected), tempfile.TemporaryDirectory() as temporary:
                rows = [case_record(expected_fatal=expected)]
                directory = Path(temporary)
                payload = self.payload(rows, tail)
                results = self.execute_guest(rows, f"uart.sendall({payload.encode()!r})", directory)
                self.assert_reports(directory, results, ["Crash"], 1)
                self.assertNotIn("infrastructure_error", results[0])

    def test_noreturn_fatal_self_exit_confirms_safe_successor_without_repeating_failure(self):
        """验证 `KTEST fatal` 触发后客体立即以退出码 0 自退出时，宿主能够确认后续用例尚未启动并在新 VM 中仅续跑剩余用例一次。"""
        rows = [case_record(0), case_record(1), case_record(2)]
        first = "\n".join(["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                           "  1..3", "  # KTEST suite-begin setup",
                           *case_lines(rows[0], status="Fail"), *case_lines(rows[1], 2)[:3],
                           "  # KTEST fatal 1 oom", "actual allocation failure"]) + "\n"
        suffix = self.payload([rows[2]], [*case_lines(rows[2])[3:],
                                          "ok 1 os::tests0", "# KTEST complete"])
        body = (f"if ids==[0,1,2]: uart.sendall({first.encode()!r})\n"
                "else:\n"
                " with (base/'effects').open('ab') as output: output.write(b'C\\n')\n"
                f" uart.sendall({suffix.encode()!r})")
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            results = self.execute_guest(rows, body, directory)
            self.assertEqual((directory / "commands").read_bytes(),
                             b"run 0 17 0 0,1,2\nrun 0 17 0 2\n")
            self.assertEqual((directory / "effects").read_bytes(), b"C\n")
            self.assert_reports(directory, results, ["Fail", "Crash", "Pass"], 1)
            self.assertFalse(any("infrastructure_error" in row for row in results))

    def test_complete_case_abort_self_exit_runs_only_confirmed_suffix_once(self):
        """验证 `case-abort` 完整走完终止协议并以退出码 0 退出后，宿主会在新 VM 中安全续跑剩余未开始的用例，且各用例只执行一次。"""
        rows = [case_record(0), case_record(1), case_record(2)]
        for phase in ("quiesce", "join", "cleanup"):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                ledger = dict(primary="body failed", child="child failed",
                              cleanup="cannot release owner", expectations=2, reason="stop")
                first = "\n".join([
                    "KTAP version 1", "1..1", "  # Subtest: os::tests0",
                    "  1..3", "  # KTEST suite-begin setup",
                    *case_lines(rows[0], status="Fail"), *case_lines(rows[1], 2)[:3],
                    f"  # KTEST phase 1 {phase}",
                    f"  # KTEST case-abort 1 {phase} " + json.dumps(ledger),
                    "not ok 1 os::tests0", "# KTEST complete"]) + "\n"
                suffix = self.payload([rows[2]], [
                    *case_lines(rows[2])[3:], "ok 1 os::tests0", "# KTEST complete"])
                body = (
                    "with (base/'effects').open('ab') as output:\n"
                    " output.write(b'AB\\n' if ids==[0,1,2] else b'C\\n')\n"
                    f"uart.sendall(({first!r} if ids==[0,1,2] else {suffix!r}).encode())\n"
                    "uart.close()\nlistener.close()\nimport os\nos._exit(0)")
                results = self.execute_guest(rows, body, directory)
                self.assertEqual((directory / "commands").read_bytes(),
                                 b"run 0 17 0 0,1,2\nrun 0 17 0 2\n")
                self.assertEqual((directory / "effects").read_bytes(), b"AB\nC\n")
                self.assert_reports(directory, results, ["Fail", "Error", "Pass"], 2)
                self.assertEqual(results[1]["case_abort"], dict(phase=phase, **ledger))
                self.assertFalse(any("infrastructure_error" in row for row in results))

    def test_unconfirmed_process_crash_preserves_failure_and_does_not_retry_suffix(self):
        """验证若用例仅输出阶段行便以非零退出码崩溃且无 `fatal`/`case-abort` 确认，宿主不会盲目重跑后续用例。"""
        rows = [case_record(0), case_record(1), case_record(2)]
        payload = "\n".join(["KTAP version 1", "1..1", "  # Subtest: os::tests0",
                             "  1..3", "  # KTEST suite-begin setup",
                             *case_lines(rows[0], status="Fail"),
                             *case_lines(rows[1], 2)[:3]]) + "\n"
        body = f"uart.sendall({payload.encode()!r})\nuart.close()\nraise SystemExit(9)"
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            results = self.execute_guest(rows, body, directory)
            self.assertEqual((directory / "commands").read_bytes(), b"run 0 17 0 0,1,2\n")
            self.assert_reports(directory, results, ["Fail", "Crash", "NotRun"], 1)

    def test_expected_fatal_needs_postevent_message_and_real_zero_exit(self):
        """验证预期致命错误（`expected_fatal`）必须在 `KTEST fatal` 事件后输出匹配消息且进程以 0 退出才判定为 `Pass`。"""
        rows = [case_record(expected_fatal="panic", fatal_message="wanted")]
        for tail, process_tail, status in (
                (["  # KTEST armed 0 panic", "business wanted",
                  "  # KTEST fatal 0 panic", "actual panic: wanted"],
                 "uart.close()\nlistener.close()\nimport os\nos._exit(0)", "Pass"),
                (["  # KTEST armed 0 panic", "  # KTEST fatal 0 panic",
                  "actual panic: wanted"], "uart.close()\nraise SystemExit(9)", "Crash"),
                (["  # KTEST armed 0 panic", "  # KTEST fatal 0 panic",
                  "actual panic: wanted"], "uart.shutdown(socket.SHUT_WR)\ntime.sleep(1)", "Crash"),
                (["  # KTEST armed 0 panic", "actual panic: wanted"], "", "Crash")):
            with self.subTest(status=status, process_tail=process_tail), \
                    tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                payload = self.payload(rows, tail)
                body = f"uart.sendall({payload.encode()!r})\n{process_tail}"
                results = self.execute_guest(rows, body, directory, options(cleanup_timeout_seconds=0.1))
                self.assert_reports(directory, results, [status], 0 if status == "Pass" else 1)
                self.assertEqual((directory / "commands").read_bytes(), b"run 0 17 0 0\n")

    def test_wrong_identity_duplicate_unknown_or_late_fatal_remains_protocol_error(self):
        """验证错误的用例 ID、未知故障类型、重复 `fatal` 或在 `# KTEST complete` 之后迟到的 `fatal` 均会被记为协议错误（退出码 2）。"""
        rows = [case_record(0)]
        for tail, status in ((["  # KTEST fatal 9 oom"], "Error"),
                             (["  # KTEST fatal 0 invented"], "Error"),
                             (["  # KTEST fatal 0 oom", "  # KTEST fatal 0 oom"], "Error"),
                             ([*case_lines(rows[0])[3:], "ok 1 os::tests0", "# KTEST complete",
                               "  # KTEST fatal 0 oom"], "Pass")):
            with self.subTest(tail=tail), tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                payload = self.payload(rows, tail)
                results = self.execute_guest(rows, f"uart.sendall({payload.encode()!r})", directory)
                # 在 # KTEST complete 之后收到的迟到控制行不会篡改已确认的用例 Pass 状态，但会附加 infrastructure_error 标记使退出码变为 2。
                if status == "Pass":
                    self.assertEqual([row["status"] for row in results], ["Pass"])
                    self.assertEqual(CLI.exit_status(results), 2)
                    self.assertTrue(results[0]["infrastructure_error"])
                    CLI.export_reports(directory, dict(results=results, **CONFIG))
                    self.assertEqual(json.loads((directory / "results.json").read_text())["results"], results)
                    self.assertEqual(ET.parse(directory / "results.xml").find("testcase/error").attrib["type"],
                                     "Protocol")
                else:
                    self.assert_reports(directory, results, ["Error"], 2)


if __name__ == "__main__":
    unittest.main()
