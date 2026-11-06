"""ktest 单台 QEMU 虚拟机测试会话生命周期与外设资源管理器。

本模块封装了单次 QEMU 启动与测试所需的全部宿主侧资源：
- 创建独占的临时工作盘目录（root/extra/scratch）、UART Unix 域套接字目录与 `qemu.log` 控制台日志；
- 按需启动宿主侧 `NetUserPeer` 网络端点与多核 AP 冻结 GDB 调试器；
- 负责在绝对截止时刻内完成 UART 连接与 `KTEST READY` 握手，并在虚拟机退出后统一核对资源消耗、
  回收子进程组及清理临时文件。
"""

import errno
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time

from . import ap_freeze
from .catalog import require
from .net_user import NetUserPeer
from .protocol import ready
from .resources import (build_test_disk_arguments, build_test_network_arguments,
                        prepare_disks)
from .uart_channel import DeadlineExpired, UartChannel, VMExited

CANCELLED = threading.Event()


class QemuTestSession:
    """管理单次测试会话的 QEMU 进程、GDB 调试器、临时工作盘、网络对端及独占 UART 通道。"""

    def check(self, deadline):
        """检查全局取消标志、当前单调时钟是否已超过绝对截止时刻，以及 GDB 调试器是否异常退出。"""
        require(not CANCELLED.is_set(), "host run cancelled")
        if deadline <= time.monotonic():
            raise DeadlineExpired("absolute host deadline expired")
        if self.debugger is not None:
            require(self.debugger.poll() in (None, 0), "AP freeze GDB failed; inspect ap-freeze-gdb.log")


    def handshake(self, config, command, deadline):
        """建立 UART 套接字连接，等待客体输出 `KTEST READY` 并校验配置，随后发送单条测试控制命令（catalog 或 run）。"""
        self.connect(deadline)
        while True:
            line = self.uart.line(deadline)
            if line.startswith("KTEST READY"):
                ready(line, config)
                self.uart.send(command, deadline)
                return


    def verify_resources(self, deadline):
        """在 QEMU 进程退出后，核对 UART 声明的输入/输出字节、GDB 冻结现场及宿主网络对端状态是否全部符合预期。"""
        self.uart.verify_consumed()
        if self.debugger is not None:
            try:
                self.debugger.wait(timeout=max(0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired as error:
                raise DeadlineExpired("AP freeze debugger exceeded cleanup deadline") from error
            require(self.debugger.returncode == 0, "AP freeze GDB failed; inspect ap-freeze-gdb.log")
            ap_freeze.verify(self.directory)
        peer = getattr(self, "net_user", None)
        if peer is not None:
            peer.verify(deadline)


    def __init__(self, args, kernel, directory, names):
        """准备独占的工作盘副本、宿主网络对端与 UART 通道，拼装命令行并启动独立会话组的 QEMU 子进程。"""
        self.args = args
        self.directory = directory
        self.process = None
        self.uart = None
        self._closed = False
        self.net_user = None
        self.debugger = None
        self.debug_log = None
        self.gdb_command = None
        self.socket_directory = tempfile.TemporaryDirectory(prefix="ktest-uart-")
        self.endpoint = str(Path(self.socket_directory.name) / "uart.sock")
        self.console = None
        self.disk_directory = None
        try:
            # 初始化串口通道、QEMU 标准输出日志与本次会话独占的工作盘副本
            self.uart = UartChannel(args, directory, self.check)
            self.console = (directory / "qemu.log").open("xb", buffering=0)
            self.disk_directory = tempfile.TemporaryDirectory(prefix="disks-", dir=directory)
            disks = prepare_disks(names, args, Path(self.disk_directory.name))
            if "net-user" in names:
                self.net_user = NetUserPeer(directory)
            # 拼装 QEMU 基础机器参数、串口套接字后端、网络参数与块设备参数
            command = [args.qemu or f"qemu-system-{args.arch}", "-machine", "virt", "-accel", "tcg",
                       "-kernel", str(kernel), "-m", args.memory, "-smp", str(args.cpus),
                       "-display", "none", "-monitor", "none", "-no-reboot",
                       "-chardev", f"socket,id=ktest,path={self.endpoint},server=on,wait=on",
                       "-serial", "chardev:ktest"]
            if args.arch == "riscv64":
                command += ["-bios", "default"]
                if args.modern:
                    command += ["-global", "virtio-mmio.force-legacy=false"]
            command += build_test_network_arguments(
                args.arch, "user" if self.net_user is not None else "none", None)
            ports = self.net_user.ports if self.net_user is not None else None
            command += build_test_disk_arguments(
                disks, architecture=args.arch, read_only_roles=args.read_only,
                logical_block_size=args.logical_block_size,
                virtio_access_platform="virtio-access-platform" in names,
                root_device=args.root, root_mode=args.root_mode,
                extra_root_device=args.extra_root, network_peer_ports=ports)
            # 若当前测试声明了多核 ap-freeze 资源，则开启 QEMU GDB stub 并准备 GDB 自动化脚本命令
            if "ap-freeze" in names and 2 <= args.cpus:
                endpoint = str(Path(self.socket_directory.name) / "gdb.sock")
                command += ["-chardev", f"socket,id=ktest-gdb,path={endpoint},server=on,wait=off",
                            "-gdb", "chardev:ktest-gdb", "-S"]
                self.gdb_command = ap_freeze.command(args, kernel, directory, endpoint)
                (directory / "gdb-command.json").write_text(json.dumps(self.gdb_command) + "\n", encoding="utf-8")
                self.debug_log = (directory / "ap-freeze-gdb.log").open("xb", buffering=0)
            # 保存完整 QEMU 命令行快照并启动独立进程组的 QEMU 实例
            (directory / "command.json").write_text(json.dumps(command) + "\n", encoding="utf-8")
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=self.console,
                                            stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            self.close()
            raise


    def connect(self, deadline):
        """在启动绝对截止时刻内轮询连接 QEMU 创建的 Unix 域串口套接字，并在连接成功后按需启动 GDB 调试器。"""
        self.uart.serial = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        while True:
            self.check(deadline)
            if self.process.poll() is not None:
                raise VMExited(f"QEMU exited {self.process.returncode}")
            try:
                self.uart.serial.connect(self.endpoint)
                self.uart.serial.setblocking(False)
                break
            except OSError as error:
                if error.errno not in (errno.ENOENT, errno.ECONNREFUSED):
                    raise
                time.sleep(min(0.01, max(0, deadline - time.monotonic())))
        if self.gdb_command is not None:
            self.debugger = subprocess.Popen(self.gdb_command, stdin=subprocess.DEVNULL,
                                             stdout=self.debug_log, stderr=subprocess.STDOUT,
                                             start_new_session=True)


    def close(self):
        """按序终止并 wait 回收 GDB 与 QEMU 进程组，随后关闭网络对端、串口、日志文件并清理临时目录。"""
        if self._closed:
            return
        # 先向 GDB 和 QEMU 所在进程组发送 SIGTERM 并等待退出，超时未退则发送 SIGKILL 强制回收
        for process in (self.debugger, self.process):
            if process is None:
                continue
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
        # 进程完全退出后再关闭网络对端、UART 通道、日志文件并删除临时目录
        peer = getattr(self, "net_user", None)
        if peer is not None:
            peer.close()
        if self.uart is not None:
            self.uart.close()
        if self.console is not None:
            self.console.close()
        if self.debug_log is not None:
            self.debug_log.close()
        self.socket_directory.cleanup()
        if self.disk_directory is not None:
            self.disk_directory.cleanup()

        self._closed = True
