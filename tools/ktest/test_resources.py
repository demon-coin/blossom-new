"""ktest 宿主测试磁盘资源管理与 QEMU 测试会话生命周期的单元测试模块。

本模块验证内核测试框架在准备与回收磁盘资源时的安全性与隔离性保证：
1. CLI 入口在准备磁盘资源前对非法 `--root-mode` 挂载模式的拦截，确保不破坏既有文件；
2. 声明磁盘角色但缺失源镜像时的错误报告，以及工作盘独立拷贝与拒绝硬链接/软链接别名的保护；
3. `QemuTestSession` 多实例并发工作盘隔离、等待子进程完全退出后再清理工作盘的生命周期控制，
   以及在复制中途失败（如 `ENOSPC`）或进程启动失败时对半成品工作盘的清理和对已有日志证据的保留。
"""

import errno
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from host_helpers import TOOLS, options
from ktest.catalog import HostError
from ktest.resources import aliases, prepare_disks, validate_resources
from ktest.qemu_session import QemuTestSession


class RootModeInputTests(unittest.TestCase):
    """验证 CLI 入口在接触磁盘资源前对 `--root-mode` 参数合法性的校验。"""

    def test_invalid_mode_is_rejected_without_replacing_work_disks(self):
        """验证传入非法的根文件系统挂载模式时 CLI 立即报错退出，且不修改已有工作盘、源镜像或创建报告目录。"""
        with tempfile.TemporaryDirectory() as temporary:
            # 准备测试用的源镜像与已有工作盘文件，用于验证失败时文件不会被触碰
            base = Path(temporary)
            source = base / "source.img"
            source.write_bytes(b"original source")
            work = base / "work"
            work.mkdir()
            disk = work / "root.raw"
            disk.write_bytes(b"preserved working disk")
            commands = [
                [sys.executable, str(TOOLS / "run_kernel_tests.py"), "list", "--arch", "riscv64",
                 "--report-dir", str(base / "report")],
                [sys.executable, str(TOOLS / "run_kernel_tests.py"), "run", "--arch", "riscv64",
                 "--report-dir", str(base / "report")],
            ]
            # 遍历空字符串、大写、多余权限字符及包含空白或等号的非法挂载模式
            for mode in ("", "RO", "rwx", "ro rw", "ro=foo", "rw=foo", "%"):
                for command in commands:
                    with self.subTest(command=command, mode=mode):
                        result = subprocess.run(command + ["--root-mode", mode],
                                                capture_output=True, text=True)
                        self.assertEqual(result.returncode, 2)
                self.assertEqual(source.read_bytes(), b"original source")
                self.assertEqual(disk.read_bytes(), b"preserved working disk")
                self.assertFalse((base / "report").exists())


class ResourceTests(unittest.TestCase):
    """验证 ktest 工作盘副本隔离、链接别名拦截、QEMU 会话生命周期及异常清理行为。"""

    def test_missing_declared_root_is_error(self):
        """验证测试声明需要 `root` 磁盘资源但未提供源镜像路径时抛出 `HostError`。"""
        with self.assertRaises(HostError):
            validate_resources({"root"}, options())

    def test_work_copy_does_not_modify_source_and_rejects_alias(self):
        """验证准备的工作盘为独立文件副本（修改不影响源镜像），且拒绝覆盖指向源镜像的软硬链接。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source.raw"
            source.write_bytes(b"original disk")
            work = base / "work"
            work.mkdir()
            # 正常工作盘准备：修改生成的工作盘不应改变源镜像内容
            args = options(disks={"root": source})
            disks = prepare_disks({"root"}, args, work)
            disks[0][1].write_bytes(b"changed work disk")
            self.assertEqual(source.read_bytes(), b"original disk")
            # 若目标工作盘路径已是指向源镜像的硬链接或软链接，准备过程必须报错拒绝以保护源镜像
            for symbolic in (False, True):
                alias_dir = base / ("symbolic" if symbolic else "hard")
                alias_dir.mkdir()
                target = alias_dir / "root.raw"
                if symbolic:
                    target.symlink_to(source)
                else:
                    target.hardlink_to(source)
                self.assertTrue(aliases(target, source))
                with self.assertRaises(HostError):
                    prepare_disks({"root"}, args, alias_dir)
                self.assertEqual(source.read_bytes(), b"original disk")

    def test_vm_disks_live_until_process_wait_and_cleanup_partial_construction(self):
        """验证工作盘在 QEMU 及其子进程完全退出前保持存在，多会话互不干扰，且复制或启动失败时自动清理半成品工作盘并保留已有日志。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source.raw"
            source.write_bytes(b"original disk")
            source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
            # 构造模拟 QEMU 的测试脚本：打开所有工作盘并派生子进程共同持有，收到 SIGTERM 时记录磁盘是否仍存在
            executable = base / "disk-user"
            executable.write_text(
                f"#!{sys.executable}\n"
                "import json,signal,subprocess,sys,time\n"
                "from pathlib import Path\n"
                "child_mode=sys.argv[1:2]==['--child']\n"
                "paths=[Path(p) for p in sys.argv[2:]] if child_mode else "
                "[Path(a.split('file=',1)[1].split(',readonly=',1)[0]) "
                "for a in sys.argv[1:] if a.startswith('if=none,')]\n"
                "report=paths[0].parent.parent\n"
                "handles=[p.open('r+b') for p in paths]\n"
                "child=None if child_mode else subprocess.Popen("
                "[sys.executable,__file__,'--child',*[str(p) for p in paths]])\n"
                "def stop(*unused):\n"
                "    if child is not None: child.wait(timeout=1)\n"
                "    (report/('child-stopped' if child_mode else 'stopped')).write_text("
                "json.dumps([p.exists() for p in paths]))\n"
                "    raise SystemExit(0)\n"
                "signal.signal(signal.SIGTERM,stop)\n"
                "if child_mode:\n"
                "    (report/'child-ready').write_text('ready')\n"
                "else:\n"
                "    while not (report/'child-ready').exists(): time.sleep(0.001)\n"
                "    print('working disks opened',flush=True)\n"
                "    (report/'ready').write_text('ready')\n"
                "while True: time.sleep(1)\n", encoding="utf-8")
            executable.chmod(0o700)
            args = options(qemu=str(executable), memory="128M", disks={"root": source, "extra": source})
            vms = []
            working_disks = []
            try:
                # 启动两个并行的 QemuTestSession 实例，并在报告目录预置日志证据文件
                for index in range(2):
                    directory = base / str(index)
                    directory.mkdir()
                    (directory / "uart-segment.raw").write_bytes(b"raw UART evidence")
                    (directory / "root.raw").write_bytes(b"unowned report evidence")
                    vm = QemuTestSession(args, Path("kernel.elf"), directory, {"root", "extra", "scratch"})
                    vms.append(vm)
                    deadline = time.monotonic() + 5
                    while not (directory / "ready").exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertTrue((directory / "ready").exists())
                    self.assertIsNone(vm.process.poll())
                    command = json.loads((directory / "command.json").read_text())
                    paths = [Path(a.split("file=", 1)[1]) for a in command if a.startswith("if=none,")]
                    working_disks.append(paths)
                    for path in paths:
                        self.assertTrue(path.is_file())
                    vm.uart.raw.write(b"UART original log\n")
                # 验证两个并发会话分配了各自独立的工作盘路径
                self.assertTrue(set(working_disks[0]).isdisjoint(working_disks[1]))
                working_disks[0][0].write_bytes(b"guest mutation")
                # 逐个关闭会话：验证进程终止时磁盘仍存在，关闭完成后临时工作盘被清理且报告目录中的日志证据完好保留
                for index, vm in enumerate(vms):
                    vm.close()
                    self.assertEqual(vm.process.returncode, 0)
                    for path in working_disks[index]:
                        self.assertFalse(path.exists())
                    self.assertEqual(json.loads((vm.directory / "stopped").read_text()), [True] * 3)
                    self.assertEqual(json.loads((vm.directory / "child-stopped").read_text()), [True] * 3)
                    self.assertEqual((vm.directory / "uart.log").read_bytes(), b"UART original log\n")
                    self.assertEqual((vm.directory / "qemu.log").read_bytes(), b"working disks opened\n")
                    self.assertTrue((vm.directory / "command.json").is_file())
                    self.assertEqual((vm.directory / "uart-segment.raw").read_bytes(), b"raw UART evidence")
                    self.assertEqual((vm.directory / "root.raw").read_bytes(), b"unowned report evidence")
                    # 关闭第一个会话不应影响第二个仍在运行的会话及其工作盘
                    if index == 0:
                        for path in working_disks[1]:
                            self.assertTrue(path.is_file())
                        self.assertIsNone(vms[1].process.poll())
            finally:
                for vm in vms:
                    vm.close()
            partial = []
            no_space = OSError(errno.ENOSPC, "No space left on device")

            def fail_copy(source_file, target_file, length):
                """模拟复制工作盘中途发生磁盘空间不足（`ENOSPC`），先写入部分字节再抛出异常。"""
                target_file.write(source_file.read(3))
                partial.append(Path(target_file.name))
                raise no_space

            # 验证复制工作盘中途失败时，半成品工作盘目录被清理，同时保留已有的串口证据文件
            directory = base / "partial"
            directory.mkdir()
            (directory / "uart-segment.raw").write_bytes(b"keep")
            with patch("ktest.resources.shutil.copyfileobj", side_effect=fail_copy):
                with self.assertRaises(OSError) as caught:
                    QemuTestSession(args, Path("kernel.elf"), directory, {"root"})
            self.assertIs(caught.exception, no_space)
            self.assertFalse(partial[0].parent.exists())
            self.assertTrue((directory / "uart.log").is_file())
            self.assertTrue((directory / "qemu.log").is_file())
            self.assertEqual((directory / "uart-segment.raw").read_bytes(), b"keep")
            # 验证 QEMU 可执行文件不存在导致启动失败时，已准备的临时工作盘目录同样被清理且源镜像完好无损
            directory = base / "launch-failure"
            directory.mkdir()
            args.qemu = str(base / "missing-executable")
            with self.assertRaises(FileNotFoundError):
                QemuTestSession(args, Path("kernel.elf"), directory, {"root"})
            command = json.loads((directory / "command.json").read_text())
            working = Path(next(a.split("file=", 1)[1] for a in command if a.startswith("if=none,")))
            self.assertFalse(working.parent.exists())
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), source_hash)


if __name__ == "__main__":
    unittest.main()
