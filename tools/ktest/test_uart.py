"""宿主侧 UART 原始字节流传输与执行日志（ExecutionUART）校验的回归测试。

本模块验证 ktest 串口通道在单向接收（rx）、单向发送（tx）、双工（duplex）及背压暂停（pause）模式下的行为：
- Castagnoli CRC32C 校验计算；
- 多阶段串口收发的游标推进、完整载荷比对与未消费残留字节检测；
- 原始字节载荷与后继 KTAP 控制行的精确切分；
- 内核执行日志（KLOG）与诊断元数据的独立逐条校验。
"""

import copy
import io
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest

from host_helpers import case_record, options, session_fixture
from ktest.catalog import HostError
from ktest.execution_uart import ExecutionUART
from ktest.resources import crc32c_update


class UARTTests(unittest.TestCase):
    """基于真实 `socketpair` 验证 UART 原始字节传输协议、全文内容校验与执行日志核对。"""

    def test_crc32c_castagnoli_vector(self):
        """验证 `crc32c_update` 对标准输入 `"123456789"` 计算出正确的 Castagnoli CRC32C 值。"""
        self.assertEqual(crc32c_update(0xFFFFFFFF, b"123456789") ^ 0xFFFFFFFF, 0xe3069283)

    def test_rx_phases_preserve_cursor_and_require_complete_input(self):
        """验证多阶段 `rx` 按游标顺序逐段发送输入文件字节，并拒绝长度不足、CRC 摘要不匹配及末尾存在未消费字节的输入。"""
        for payload, second_error, unused in ((b"LR", False, False), (b"L", True, False),
                                               (b"LX", True, False), (b"LRX", False, True)):
            with self.subTest(payload=payload), tempfile.TemporaryDirectory() as temporary:
                base = Path(temporary)
                source = base / "input"
                source.write_bytes(payload)
                host, guest = socket.socketpair()
                guest.settimeout(2)
                try:
                    vm = session_fixture()
                    vm.uart.args = options(uart_payload=source)
                    vm.uart.directory = base
                    vm.uart.serial = host
                    host.setblocking(False)
                    vm.uart.raw = io.BytesIO()
                    vm.uart.buffer = bytearray()
                    with guest.makefile("rb") as received:
                        # 分两个阶段依次向客体发送单字节载荷，核对 `H` 握手字节、载荷内容与 `KTEST IO OK` 确认行。
                        for index, byte in enumerate((b"L", b"R")):
                            declaration = dict(direction="rx", length=1,
                                               digest=f"{crc32c_update(0xFFFFFFFF, byte) ^ 0xFFFFFFFF:08x}",
                                               delay_ms=0, case=index)
                            if index == 1 and second_error:
                                with self.assertRaises(HostError):
                                    vm.uart.raw_io(declaration, time.monotonic() + 2, {"uart"})
                                self.assertEqual(received.readline(), b"KTEST IO ERROR 1\n")
                                self.assertEqual(vm.uart.rx_offset, 1)
                            else:
                                vm.uart.raw_io(declaration, time.monotonic() + 2, {"uart"})
                                self.assertEqual(received.read(2), b"H" + byte)
                                self.assertEqual(received.readline(), f"KTEST IO OK {index}\n".encode("ascii"))
                                self.assertEqual(vm.uart.rx_offset, index + 1)
                    # 若输入文件末尾仍有未消费字节，资源核查阶段必须报错。
                    if unused:
                        with self.assertRaisesRegex(HostError, "unconsumed"):
                            vm.verify_resources(time.monotonic() + 2)
                    elif not second_error:
                        vm.verify_resources(time.monotonic() + 2)
                    self.assertEqual(source.read_bytes(), payload)
                finally:
                    host.close()
                    guest.close()

    def test_duplex_payloads_use_independent_offsets_and_preserve_full_duplex_bytes(self):
        """验证 `duplex` 双工模式在常规收发及握手合包场景下均能维护独立的收发偏移，并完整保留各阶段原始字节。"""
        payload = b"\x00\xff\xe7\x95\x8c\nsecond-rx"
        transmitted = b"echo\x00\xff\xe7\x95\x8c\nsecond-tx"
        phases = ((payload[:3], transmitted[:6]), (payload[3:], transmitted[6:]))
        for mode in ("normal", "coalesced"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temporary:
                base = Path(temporary)
                source = base / "input"
                expected = base / "expected"
                source.write_bytes(payload)
                expected.write_bytes(transmitted)
                host, guest = socket.socketpair()
                guest.settimeout(2)
                peer_errors = []
                next_handshake = threading.Event()

                def peer():
                    """模拟客体端串口对端：按阶段接收 `H` 握手、发送 `tx` 字节、接收 `rx` 字节并核对 `KTEST IO OK` 响应。"""
                    try:
                        for index, (rx, tx) in enumerate(phases):
                            self.assertEqual(guest.recv(1), b"H")
                            guest.sendall(tx)
                            received = bytearray()
                            while len(received) < len(rx):
                                received.extend(guest.recv(len(rx) - len(received)))
                            self.assertEqual(received, rx)
                            expected_ack = f"KTEST IO OK {index}\n".encode()
                            if mode == "coalesced" and index == 0:
                                self.assertTrue(next_handshake.wait(2), "next handshake was not sent before ACK")
                            ack = bytearray()
                            while len(ack) < len(expected_ack):
                                part = guest.recv(len(expected_ack) - len(ack))
                                self.assertTrue(part, "peer stream closed before ACK")
                                ack.extend(part)
                            self.assertEqual(ack, expected_ack)
                    except BaseException as error:
                        peer_errors.append(error)

                worker = threading.Thread(target=peer)
                try:
                    vm = session_fixture()
                    if mode == "coalesced":
                        original_send = vm.uart.send
                        handshake_count = 0

                        def observed_send(data, deadline):
                            """包装宿主 `send` 方法，统计 `H` 握手发送次数，并在第二次握手发出后通知对端线程。"""
                            nonlocal handshake_count
                            original_send(data, deadline)
                            if data == b"H":
                                handshake_count += 1
                                if handshake_count == 2:
                                    next_handshake.set()

                        vm.uart.send = observed_send
                    vm.uart.args = options(uart_payload=source, uart_expect=expected)
                    vm.uart.directory = base
                    vm.uart.serial = host
                    host.setblocking(False)
                    vm.uart.raw = io.BytesIO()
                    vm.uart.buffer = bytearray()
                    worker.start()
                    # 依次执行两个双工阶段，验证收发偏移、资源消费完整性与落盘的 `.raw` 切片文件。
                    for index, (rx, tx) in enumerate(phases):
                        checksum = lambda data: f"{crc32c_update(0xFFFFFFFF, data) ^ 0xFFFFFFFF:08x}"
                        declaration = dict(direction="duplex", rx_length=len(rx), rx_digest=checksum(rx),
                                           rx_delay_ms=2, tx_length=len(tx), tx_digest=checksum(tx),
                                           tx_delay_ms=1, case=index)
                        vm.uart.raw_io(declaration, time.monotonic() + 2, {"uart"})
                    worker.join(timeout=2)
                    self.assertFalse(worker.is_alive())
                    self.assertEqual(peer_errors, [])
                    self.assertEqual((vm.uart.rx_offset, vm.uart.tx_offset), (len(payload), len(transmitted)))
                    vm.verify_resources(time.monotonic() + 2)
                    self.assertEqual(source.read_bytes(), payload)
                    self.assertEqual(expected.read_bytes(), transmitted)
                    raw_parts = sorted(base.glob("uart-*.raw"))
                    self.assertEqual([path.read_bytes() for path in raw_parts], [part[1] for part in phases])
                finally:
                    host.close()
                    guest.close()
                    worker.join(timeout=2)

    def test_duplex_consumes_buffered_prefix_and_leaves_next_control_line(self):
        """验证当串口预读缓冲区同时包含双工 `tx` 载荷与下一条控制行时，仅消费声明长度的载荷并保留后续控制行。"""
        transmitted = b"\x00\xffecho\n"
        tail = b"next-control\n"
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            expected = base / "expected"
            expected.write_bytes(transmitted)
            host, guest = socket.socketpair()
            guest.settimeout(2)
            try:
                vm = session_fixture()
                vm.uart.args = options(uart_expect=expected)
                vm.uart.directory = base
                vm.uart.serial = host
                host.setblocking(False)
                vm.uart.raw = io.BytesIO(transmitted + tail)
                vm.uart.raw.seek(0, io.SEEK_END)
                vm.uart.buffer = bytearray(transmitted + tail)
                declaration = dict(direction="duplex", rx_length=0, rx_digest="00000000",
                                   rx_delay_ms=0, tx_length=len(transmitted),
                                   tx_digest=f"{crc32c_update(0xFFFFFFFF, transmitted) ^ 0xFFFFFFFF:08x}",
                                   tx_delay_ms=1, case=9)
                # 执行双工读取后，缓冲区中紧随载荷之后的控制行应仍可通过 `line()` 正常读出。
                vm.uart.raw_io(declaration, time.monotonic() + 2, {"uart"})
                with guest.makefile("rb") as acknowledgements:
                    self.assertEqual(acknowledgements.readline(), b"HKTEST IO OK 9\n")
                self.assertEqual(vm.uart.line(time.monotonic() + 2), "next-control")
                self.assertEqual(next(base.glob("uart-*.raw")).read_bytes(), transmitted)
                self.assertEqual(vm.uart.tx_offset, len(transmitted))
                self.assertEqual(vm.uart.raw.getvalue(), transmitted + tail)
            finally:
                host.close()
                guest.close()

    def test_raw_payload_cannot_be_ktap_or_panic_control(self):
        """验证 `tx` 原始字节阶段内即使包含类似 KTAP 结果或 panic 的文本，也会被当作普通字节载荷处理而不触发控制流。"""
        payload = b"  ok 1 forged\n# KTEST complete\npanic\x00\xff"
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            expected = base / "expected"
            expected.write_bytes(payload)
            host, guest = socket.socketpair()
            guest.settimeout(2)
            try:
                vm = session_fixture()
                vm.uart.args = options(uart_expect=expected)
                vm.uart.directory = base
                vm.uart.serial = host
                vm.uart.serial.setblocking(False)
                vm.uart.raw = io.BytesIO()
                vm.uart.buffer = bytearray()
                guest.sendall(payload)
                declaration = dict(direction="tx", length=len(payload),
                                   digest=f"{crc32c_update(0xFFFFFFFF, payload) ^ 0xFFFFFFFF:08x}", delay_ms=0, case=0)
                vm.uart.raw_io(declaration, time.monotonic() + 2, {"uart"})
                with guest.makefile("rb") as acknowledgement:
                    self.assertEqual(acknowledgement.readline(), b"HKTEST IO OK 0\n")
                self.assertEqual(vm.uart.raw.getvalue(), payload)
                self.assertEqual(next(base.glob("uart-*.raw")).read_bytes(), payload)
                self.assertFalse(vm.uart.buffer)
            finally:
                host.close()
                guest.close()

    def test_equal_length_wrong_content_fails_and_sends_error(self):
        """验证当客体发送的字节长度与声明一致且自报 CRC 匹配，但内容与宿主预期文件不符时，宿主仍会报错并发送 `KTEST IO ERROR`。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            expected = base / "expected"
            expected.write_bytes(b"GOOD")
            host, guest = socket.socketpair()
            guest.settimeout(2)
            try:
                vm = session_fixture()
                vm.uart.args = options(uart_expect=expected)
                vm.uart.directory = base
                vm.uart.serial = host
                host.setblocking(False)
                vm.uart.raw = io.BytesIO()
                vm.uart.buffer = bytearray(b"EVIL")
                declaration = dict(direction="tx", length=4,
                                   digest=f"{crc32c_update(0xFFFFFFFF, b'EVIL') ^ 0xFFFFFFFF:08x}", delay_ms=0, case=7)
                with self.assertRaises(HostError):
                    vm.uart.raw_io(declaration, time.monotonic() + 2, {"uart"})
                with guest.makefile("rb") as acknowledgement:
                    self.assertEqual(acknowledgement.readline(), b"HKTEST IO ERROR 7\n")
            finally:
                host.close()
                guest.close()

    def test_backpressure_ack_bound_is_checked_before_h(self):
        """验证背压测试在发送 `H` 握手字节前先检查用例编号和 ACK 长度上限（不超过 16 字节），不合法时直接回复 `KTEST IO ERROR`。"""
        for cid, declared in ((0, 1), (1000, 1000)):
            with self.subTest(cid=cid, declared=declared), tempfile.TemporaryDirectory() as temporary:
                base = Path(temporary)
                fixture = ExecutionUART(2, base)
                fixture.begin(case_record(cid=cid, name="os::ktest::execution_services::klog::uart::socket_printer_backpressure"))
                fixture.body = True
                host, guest = socket.socketpair()
                guest.settimeout(2)
                try:
                    vm = session_fixture()
                    vm.uart.args = options(execution_uart=True)
                    vm.uart.serial = host
                    host.setblocking(False)
                    vm.uart.execution_uart = fixture
                    declaration = dict(direction="pause", length=0, digest="00000000", delay_ms=3000, case=declared)
                    with self.assertRaises(HostError):
                        vm.uart.raw_io(declaration, time.monotonic() + 2, {"uart"})
                    with guest.makefile("rb") as acknowledgement:
                        self.assertEqual(acknowledgement.readline(), f"KTEST IO ERROR {declared}\n".encode("ascii"))
                finally:
                    host.close()
                    guest.close()

    def test_backpressure_opaque_boundary_and_full_payload_failures(self):
        """验证串口背压暂停阶段保留前导填充字节与全部 65 条 KLOG 记录，且任何正文篡改、丢失计数不符或尾行缺失均会被检出。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            fixture = ExecutionUART(2, base)
            fixture.begin(case_record(name="os::ktest::execution_services::klog::uart::socket_printer_backpressure"))
            fixture.body = True
            lines = []
            metadata = []
            for index in range(65):
                seq = 100 if index == 0 else 108 + index
                lost = 8 if index == 1 else 0
                body = b"X26 partial " + b"P" * 450 if index == 0 else f"X26 record={index + 7}".encode("ascii")
                lines.append(f"KLOG seq={seq} level=INFO ns={1000000 + index} cpu=0 task=27 truncated=0 lost={lost} ".encode("ascii") + body + b"\n")
                metadata.append(f"  # diagnostic: execution-backpressure record index={index} seq={seq} level=INFO ns={1000000 + index} cpu=0 task=27 truncated=0 lost={lost}\n".encode("ascii"))
            prefix = b"\n" * 17
            tail = b"  # KTEST phase 0 quiesce\n"
            host, guest = socket.socketpair()
            guest.settimeout(5)
            try:
                vm = session_fixture()
                vm.uart.args = options(execution_uart=True)
                vm.uart.directory = base
                vm.uart.serial = host
                host.setblocking(False)
                vm.uart.raw = io.BytesIO()
                vm.uart.buffer = bytearray()
                vm.uart.execution_uart = fixture
                # 发送 17 字节换行前缀、65 条 KLOG 记录与阶段结束控制行，执行背压暂停接收并核对诊断元数据。
                guest.sendall(prefix + b"".join(lines) + tail)
                declaration = dict(direction="pause", length=0, digest="00000000", delay_ms=3000, case=0)
                vm.uart.raw_io(declaration, time.monotonic() + 8, {"uart"})
                with guest.makefile("rb") as acknowledgement:
                    self.assertEqual(acknowledgement.readline(), b"HKTEST IO OK 0\n")
                self.assertEqual(vm.uart.line(time.monotonic() + 2), tail[:-1].decode("ascii"))
                for raw in metadata:
                    fixture.observe(raw)
                fixture.observe(b"  # diagnostic: execution-backpressure fill-bytes=17\n")
                fixture.finish("Pass")
                self.assertEqual((base / "execution-uart-0.raw").read_bytes(), prefix + b"".join(lines))
                # 对已通过校验的记录集合分别注入正文字符替换、丢失计数篡改、删除末行和删除末条元数据，均须触发 HostError。
                for mutation in ("body", "lost", "last-line", "last-metadata"):
                    with self.subTest(mutation=mutation):
                        broken = copy.deepcopy(fixture)
                        if mutation == "body":
                            broken.lines[0] = broken.lines[0].replace(b"P", b"Q", 1)
                        elif mutation == "lost":
                            broken.lines[1] = broken.lines[1].replace(b"lost=8", b"lost=0")
                        elif mutation == "last-line":
                            broken.lines.pop()
                        else:
                            broken.records.pop(64)
                        with self.assertRaises(HostError):
                            broken.finish("Pass")
            finally:
                host.close()
                guest.close()

    def test_mixed_logs_require_every_cpu_round_and_exact_body(self):
        """验证多核混合执行日志要求每个 CPU 的全部 8 轮 `timer` 与 `work` 记录完整无误，缺行或篡改内容均会触发 `HostError`。"""
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ExecutionUART(2, Path(temporary))
            fixture.begin(case_record(name="os::ktest::execution_services::mixed::cases::timer_work_task_log_convergence"))
            fixture.body = True
            seq = 1
            for cpu in range(2):
                for round_number in range(8):
                    for source in ("timer", "work"):
                        task = 0 if source == "timer" else 20 + cpu
                        printed_task = "none" if task == 0 else str(task)
                        fixture.observe(f"KLOG seq={seq} level=INFO ns={seq * 1000} cpu={cpu} task={printed_task} truncated=0 lost=0 X28 {source} slot={cpu} round={round_number}\n".encode("ascii"))
                        fixture.observe(f"  # diagnostic: execution-log record seq={seq} ns={seq * 1000} cpu={cpu} task={task} source={source} slot={cpu} round={round_number}\n".encode("ascii"))
                        seq += 1
            fixture.finish("Pass")
            for mutation in ("last-line", "body", "lost"):
                with self.subTest(mutation=mutation):
                    broken = copy.deepcopy(fixture)
                    if mutation == "last-line":
                        broken.lines.pop()
                    elif mutation == "body":
                        broken.lines[-1] = broken.lines[-1].replace(b"round=7", b"round=6")
                    else:
                        broken.lines[-1] = broken.lines[-1].replace(b"lost=0", b"lost=1")
                    with self.assertRaises(HostError):
                        broken.finish("Pass")

    def test_execution_records_compare_independent_body_without_pinning_locked_labels(self):
        """验证执行记录校验器逐条比对固定业务日志、UTF-8 截断日志及诊断 `body_hex` 提供的动态标签正文，拒绝任何单字节篡改。"""
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ExecutionUART(2, Path(temporary))
            fixture.begin(case_record(name="os::ktest::execution_services::klog::records::allocation_free_record_identity"))
            fixture.body = True
            bodies = (b"X25 ordinary value=37", "界".encode("utf-8") * 170,
                      "调度锁标签可变".encode("utf-8"), b"pool label may change", b"X25 precise timer")
            for index, (level, body) in enumerate(zip(("INFO", "WARN", "DEBUG", "DEBUG", "INFO"), bodies)):
                truncated = int(index == 1)
                fixture.observe(f"KLOG seq={index + 1} level={level} ns={index + 100} cpu=0 task=27 truncated={truncated} lost=0 ".encode("ascii") + body + b"\n")
                body_hex = " body_hex=" + body.hex() if index in (2, 3) else ""
                fixture.observe(f"  # diagnostic: execution-record record index={index} seq={index + 1} level={level} ns={index + 100} cpu=0 task=27 truncated={truncated}{body_hex}\n".encode("ascii"))
            fixture.finish("Pass")
            for index in range(5):
                broken = copy.deepcopy(fixture)
                original = broken.lines[index]
                position = original.index(b" lost=0 ") + len(b" lost=0 ")
                broken.lines[index] = original[:position] + b"?" + original[position + 1:]
                with self.subTest(index=index), self.assertRaises(HostError):
                    broken.finish("Pass")


if __name__ == "__main__":
    unittest.main()
