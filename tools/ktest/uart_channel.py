"""ktest 串口字节流通道、行协议读取器与原始 UART I/O 测试执行器。

本模块封装了单台 QEMU 虚拟机独占的串口通信细节：
- 将客体发出的所有原始串口字节实时写入 `uart.log`，并提供最大 64KiB 的行缓冲解码与 `ExecutionUART` 观察钩子；
- 在收到 `# KTEST io ...` 声明后，接管串口执行单向（rx/tx/pause/slow）或双工（duplex）原始字节流测试，
  维护跨阶段的发送/接收文件偏移游标，并逐段核对文件长度、期望字节与 Castagnoli CRC32C 校验和；
- 所有阻塞等待均通过会话注入的 `check(deadline)` 回调受绝对截止时刻与取消标志约束。
"""

import select
import time

from .catalog import HostError, require
from .execution_uart import ExecutionUART
from .resources import crc32c_update


class DeadlineExpired(RuntimeError):
    """表示当前操作的宿主单调时钟已达到或超过预设的绝对截止时刻。"""


class VMExited(RuntimeError):
    """表示 QEMU 子进程已退出，或其关联的 UART Unix 域套接字已关闭（读到 EOF）。"""


class UartChannel:
    """管理单台虚拟机串口套接字、行读取缓冲区、原始日志落盘以及测试负载游标状态。"""

    def __init__(self, args, directory, check):
        """初始化行缓冲区、RX/TX 负载文件游标、`ExecutionUART` 观察器及 `uart.log` 原始日志文件；底层套接字由会话在连接时绑定。"""
        self.args = args
        self.directory = directory
        self.check = check
        self.serial = None
        self.buffer = bytearray()
        self.rx_offset = 0
        self.rx_used = False
        self.tx_offset = 0
        self.tx_used = False
        self.execution_uart = ExecutionUART(args.cpus, directory)
        self.raw = (directory / "uart.log").open("xb", buffering=0)

    def receive(self, deadline, limit=65536):
        """在绝对截止时刻前从串口接收单块原始数据并同步写入 `uart.log`；若遇到对端关闭则抛出 `VMExited`。"""
        while True:
            self.check(deadline)
            readable, _, _ = select.select([self.serial], [], [], max(0, min(0.1, deadline - time.monotonic())))
            self.check(deadline)
            if not readable:
                continue
            try:
                data = self.serial.recv(limit)
            except BlockingIOError:
                continue
            if not data:
                raise VMExited("UART closed")
            self.raw.write(data)
            return data


    def line(self, deadline):
        """从串口缓冲读取并 UTF-8 解码单行协议文本（上限 64KiB），同时将原始行字节通知 `ExecutionUART` 观察器。"""
        while True:
            self.check(deadline)
            newline = self.buffer.find(b"\n")
            if 0 <= newline:
                require(newline <= 65536, "UART protocol line exceeds limit")
                raw = bytes(self.buffer[:newline + 1])
                del self.buffer[:newline + 1]
                fixture = getattr(self, "execution_uart", None)
                if fixture is not None:
                    fixture.observe(raw)
                return raw[:-1].rstrip(b"\r").decode("utf-8", errors="strict")
            require(len(self.buffer) <= 65536, "UART protocol line exceeds limit")
            self.buffer.extend(self.receive(deadline))


    def send(self, data, deadline):
        """在绝对截止时刻内通过 select 循环非阻塞发送完整字节序列，确保每次可写事件都有实际字节发出。"""
        view = memoryview(data)
        while view:
            self.check(deadline)
            _, writable, _ = select.select([], [self.serial], [], max(0, min(0.1, deadline - time.monotonic())))
            self.check(deadline)
            if writable:
                try:
                    count = self.serial.send(view)
                except BlockingIOError:
                    continue
                require(0 < count, "UART send made no progress")
                view = view[count:]


    def delay(self, seconds, deadline):
        """在遵守宿主绝对截止时刻与取消检查的前提下暂停读取指定秒数（用于模拟串口慢速消费或暂停场景）。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check(deadline)
            time.sleep(max(0, min(0.01, end - time.monotonic(), deadline - time.monotonic())))


    def raw_io(self, declaration, deadline, names):
        """执行客体声明的原始 UART I/O 测试阶段：发送握手字节 `H`，按游标发送或接收指定长度的原始字节流并核对 CRC32C，最后回发完成状态。"""
        require("uart" in names, "UART IO without declared resource")
        direction = declaration["direction"]
        length = declaration["length"] if direction != "duplex" else 0
        chunk = 64 if direction == "slow" else 65536
        state = 0xFFFFFFFF
        payload = self.args.uart_payload
        expected_path = self.args.uart_expect
        expected = None
        try:
            # 双工模式或反压观察模式单独分发处理
            if direction == "duplex":
                self._duplex_io(declaration, deadline)
                return
            fixture = getattr(self, "execution_uart", None)
            if fixture is not None and fixture.kind == "backpressure":
                require(getattr(self.args, "execution_uart", False),
                        "backpressure fixture requires --execution-uart")
                fixture.capture(self, declaration, deadline)
                return
            # 正式发送握手字节 H 之前，预先校验输入负载（rx）的 CRC32C 或期望输出文件（tx）的长度
            if direction == "rx":
                require(payload is not None, "UART receive fixture requires --uart-payload")
                input_state = 0xFFFFFFFF
                remaining = length
                with payload.open("rb") as source:
                    source.seek(self.rx_offset)
                    while remaining:
                        self.check(deadline)
                        block = source.read(min(chunk, remaining))
                        require(block, "UART input ended before declared phase length")
                        input_state = crc32c_update(input_state, block)
                        remaining -= len(block)
                require(f"{input_state ^ 0xFFFFFFFF:08x}" == declaration["digest"], "UART input CRC mismatch")
                self.rx_used = True
            else:
                require(expected_path is not None, "UART transmit fixture requires --uart-expect")
                require(expected_path.stat().st_size == length, "UART expected length mismatch")
                expected = expected_path.open("rb")
            # 发送单字节 H 通知客体开始原始字节流传输，并按方向执行发送或接收校验
            self.send(b"H", deadline)
            if direction == "pause":
                self.delay(declaration["delay_ms"] / 1000, deadline)
            if direction == "rx":
                remaining = length
                with payload.open("rb") as source:
                    source.seek(self.rx_offset)
                    while remaining:
                        block = source.read(min(chunk, remaining))
                        require(block, "UART input ended during declared phase")
                        state = crc32c_update(state, block)
                        self.send(block, deadline)
                        remaining -= len(block)
                        self.rx_offset += len(block)
            else:
                remaining = length
                with (self.directory / f"uart-{declaration['case']}-{time.monotonic_ns()}.raw").open("xb") as output:
                    while remaining:
                        self.check(deadline)
                        if direction == "slow":
                            self.delay(declaration["delay_ms"] / 1000, deadline)
                        count = min(chunk, remaining)
                        if self.buffer:
                            block = bytes(self.buffer[:count])
                            del self.buffer[:len(block)]
                        else:
                            block = self.receive(deadline, count)
                        state = crc32c_update(state, block)
                        output.write(block)
                        if expected is not None:
                            require(expected.read(len(block)) == block, "UART full payload mismatch")
                        remaining -= len(block)
            require(f"{state ^ 0xFFFFFFFF:08x}" == declaration["digest"], "UART full payload CRC mismatch")
            self.send(f"KTEST IO OK {declaration['case']}\n".encode("ascii"), deadline)
        except (HostError, OSError) as error:
            try:
                self.send(f"KTEST IO ERROR {declaration['case']}\n".encode("ascii"), deadline)
            except (OSError, DeadlineExpired, VMExited):
                pass
            raise error
        finally:
            if expected is not None:
                expected.close()


    def _duplex_io(self, declaration, deadline):
        """在同一个串口上通过单线程 select 交错执行双工（duplex）收发，维护各自的延迟窗口、文件游标并校验双向 CRC32C。"""
        rx_length = declaration["rx_length"]
        tx_length = declaration["tx_length"]
        payload = self.args.uart_payload
        expected_path = self.args.uart_expect
        # 启动双工传输前，预先核对本次接收段（宿主发往客体）的输入文件 CRC32C 与发送段（客体发往宿主）的期望文件剩余长度
        if rx_length:
            require(payload is not None, "UART duplex receive requires --uart-payload")
            state = 0xFFFFFFFF
            with payload.open("rb") as source:
                source.seek(self.rx_offset)
                remaining = rx_length
                while remaining:
                    self.check(deadline)
                    block = source.read(min(65536, remaining))
                    require(block, "UART duplex input ended before declared phase length")
                    state = crc32c_update(state, block)
                    remaining -= len(block)
            require(f"{state ^ 0xFFFFFFFF:08x}" == declaration["rx_digest"],
                    "UART duplex input CRC mismatch")
            self.rx_used = True
        if tx_length:
            require(expected_path is not None, "UART duplex transmit requires --uart-expect")
            require(self.tx_offset + tx_length <= expected_path.stat().st_size,
                    "UART duplex expectation ended before declared phase length")
            self.tx_used = True

        source = expected = output = None

        try:
            source = payload.open("rb") if rx_length else None
            expected = expected_path.open("rb") if tx_length else None
            if source is not None:
                source.seek(self.rx_offset)
            if expected is not None:
                expected.seek(self.tx_offset)
                output = (self.directory / f"uart-{declaration['case']}-{time.monotonic_ns()}.raw").open("xb")


            # 发送握手字节 H 启动双工阶段，并在单循环中按延迟窗口交错驱动串口读写与 CRC32C 累积
            self.send(b"H", deadline)
            receive_at = time.monotonic() + declaration["rx_delay_ms"] / 1000
            receive_after = time.monotonic()
            pending = memoryview(b"")
            rx_remaining = rx_length
            tx_remaining = tx_length
            rx_state = tx_state = 0xFFFFFFFF
            while rx_remaining or tx_remaining:
                self.check(deadline)
                now = time.monotonic()
                if rx_remaining and not pending and receive_at <= now:
                    block = source.read(min(65536, rx_remaining))
                    require(block, "UART duplex input ended during declared phase")
                    pending = memoryview(block)

                read_sockets = [self.serial] if tx_remaining and receive_after <= now else []
                write_sockets = [self.serial] if pending else []
                timeout = max(0, min(0.1, deadline - now))
                if read_sockets and self.buffer:
                    timeout = 0
                if rx_remaining and not pending and now < receive_at:
                    timeout = min(timeout, receive_at - now)
                if tx_remaining and not read_sockets and now < receive_after:
                    timeout = min(timeout, receive_after - now)
                readable, writable, _ = select.select(read_sockets, write_sockets, [], timeout)
                self.check(deadline)

                if readable or (read_sockets and self.buffer):
                    chunk = 64 if declaration["tx_delay_ms"] else 65536
                    count = min(chunk, tx_remaining)
                    if self.buffer:
                        block = bytes(self.buffer[:count])
                        del self.buffer[:len(block)]
                    else:
                        try:
                            block = self.serial.recv(count)
                        except BlockingIOError:
                            continue
                        if not block:
                            raise VMExited("UART closed during duplex payload")
                        self.raw.write(block)
                    output.write(block)
                    tx_state = crc32c_update(tx_state, block)
                    require(expected.read(len(block)) == block, "UART duplex payload mismatch")
                    tx_remaining -= len(block)
                    self.tx_offset += len(block)
                    receive_after = time.monotonic() + declaration["tx_delay_ms"] / 1000

                if writable:
                    try:
                        count = self.serial.send(pending)
                    except BlockingIOError:
                        continue
                    require(0 < count, "UART duplex input made no progress")
                    sent = pending[:count]
                    rx_state = crc32c_update(rx_state, sent)
                    pending = pending[count:]
                    rx_remaining -= count
                    self.rx_offset += count

            if rx_length:
                require(f"{rx_state ^ 0xFFFFFFFF:08x}" == declaration["rx_digest"],
                        "UART duplex transmitted input CRC mismatch")
            if tx_length:
                require(f"{tx_state ^ 0xFFFFFFFF:08x}" == declaration["tx_digest"],
                        "UART duplex output CRC mismatch")
        finally:
            if source is not None:
                source.close()
            if expected is not None:
                expected.close()
            if output is not None:
                output.close()

        self.send(f"KTEST IO OK {declaration['case']}\n".encode("ascii"), deadline)


    def verify_consumed(self):
        """在 QEMU 进程退出后，校验所有启用过的输入负载文件（`--uart-payload`）与期望输出文件（`--uart-expect`）均已按游标完整消费到末尾。"""
        if self.rx_used:
            require(self.args.uart_payload.stat().st_size == self.rx_offset,
                    "UART input has unconsumed bytes at VM exit")
        if self.tx_used:
            require(self.args.uart_expect.stat().st_size == self.tx_offset,
                    "UART expected output has unconsumed bytes at VM exit")

    def close(self):
        """关闭串口套接字（若已连接）与 `uart.log` 原始日志文件句柄。"""
        if self.serial is not None:
            self.serial.close()
        self.raw.close()
