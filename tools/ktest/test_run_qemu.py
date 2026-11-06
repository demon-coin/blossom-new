"""`run_qemu.py` 命令行入口的端到端行为单元测试模块。

本模块通过启动真实的 `run_qemu.py` 子进程，验证生产环境 QEMU 启动器的命令行接口与安全边界：
1. `--help` 与 `--print-command` 只读展示模式不触碰文件系统、不创建目录且不争抢已有工作盘锁；
2. `riscv64` 与 `loongarch64` 架构的默认内核路径、源镜像路径及硬件参数映射；
3. `--shell`、`--debug`、`--system-only`、`--init-program` 与 `--root-mode` 等选项的正交组合与非法参数组合拦截；
4. 相对路径按仓库根目录解析、特殊字符转义以及 `--working-disk-action` 摘要输出；
5. 实际启动前对内核 ELF 架构匹配与 QEMU 可执行权限的校验，以及 `exec` 子进程对目录 `flock` 文件锁的继承。
"""

import fcntl
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest

from test_boot import kernel_image

TOOLS = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = TOOLS.parent


class RunQemuTests(unittest.TestCase):
    """通过调用真实 `run_qemu.py` 子进程验证 CLI 参数解析、命令展示与工作盘锁行为的测试集。"""

    def invoke(self, arguments, cwd=None):
        """在指定工作目录下启动 `run_qemu.py` 子进程并返回执行结果。

        参数:
            arguments (Sequence[str]): 传递给 `run_qemu.py` 的命令行参数序列。
            cwd (Path | str | None): 子进程的工作目录；为 `None` 时继承当前进程目录。

        返回:
            subprocess.CompletedProcess[str]: 包含退出码、标准输出与标准错误的执行结果。
        """
        return subprocess.run([sys.executable, str(TOOLS / "run_qemu.py"), *arguments],
                              cwd=cwd, capture_output=True, text=True, timeout=10)

    def printed_command(self, arguments, cwd=None):
        """附带 `--print-command` 运行 `run_qemu.py`，断言退出码为 0 并将标准输出按 shell 规则拆分为参数列表。

        返回:
            tuple[list[str], str]: 解析后的 QEMU 命令行参数列表与标准错误摘要文本。
        """
        result = self.invoke([*arguments, "--print-command"], cwd)
        self.assertEqual(result.returncode, 0, result.stderr)
        return shlex.split(result.stdout), result.stderr

    def test_help_and_print_do_not_touch_selected_paths(self):
        """验证 `--help` 与 `--print-command` 在输入文件缺失时仍能成功返回，且不会在磁盘上创建或修改任何文件。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            marker = base / "preserved"
            marker.write_bytes(b"untouched")
            common = ["--arch", "riscv64", "--kernel-elf", str(base / "missing.elf"),
                      "--system-image", str(base / "missing-system.img"),
                      "--contest-image", str(base / "missing-contest.img"),
                      "--working-disk-dir", str(base / "missing-work"),
                      "--qemu-binary", str(base / "missing-qemu")]
            before = sorted(base.iterdir())
            # 依次测试 --help 和不同工作盘动作下的 --print-command，确保目录内容完全不变
            for tail in (["--help"], ["--print-command"],
                         ["--working-disk-action", "reuse", "--print-command"],
                         ["--working-disk-action", "refresh-system", "--print-command"]):
                with self.subTest(tail=tail):
                    result = self.invoke([*common, *tail], base)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(sorted(base.iterdir()), before)
                    self.assertEqual(marker.read_bytes(), b"untouched")
            result = self.invoke(["--help"], base)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(sorted(base.iterdir()), before)

    def test_default_paths_and_machine_options_are_architecture_specific(self):
        """验证 `riscv64` 与 `loongarch64` 使用各自默认的内核与磁盘镜像路径，且默认不附带 `-append` 或调试参数。"""
        with tempfile.TemporaryDirectory() as temporary:
            for architecture, suffix, system in (("riscv64", "rv", "disk.img"),
                                                 ("loongarch64", "la", "disk-la.img")):
                with self.subTest(architecture=architecture):
                    command, summary = self.printed_command(["--arch", architecture], temporary)
                    self.assertEqual(command[0], f"qemu-system-{architecture}")
                    self.assertEqual(command[command.index("-kernel") + 1], str(REPOSITORY_ROOT / f"kernel-{suffix}"))
                    self.assertEqual(command[command.index("-smp") + 1], "1")
                    self.assertEqual(command[command.index("-m") + 1], "1G")
                    self.assertNotIn("-append", command)
                    self.assertNotIn("-S", command)
                    self.assertNotIn("-gdb", command)
                    self.assertIn(f"source={REPOSITORY_ROOT / system}", summary)
                    self.assertIn(f"source={REPOSITORY_ROOT / f'sdcard-{suffix}.img'}", summary)
                    self.assertEqual(command.count("-drive"), 2)
                    self.assertIn("user,id=", command[command.index("-netdev") + 1])

    def test_print_does_not_take_an_existing_active_disk_lock(self):
        """验证 `--print-command` 在工作盘目录已被其他进程持有独占 `flock` 锁时仍能正常执行，且不修改锁文件或工作盘。"""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            disk = directory / "root.raw"
            disk.write_bytes(b"active working disk")
            # 先对工作目录的 .lock 文件加上非阻塞独占锁，模拟另一个正在运行的会话
            with (directory / ".lock").open("a+b") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                inode = os.fstat(lock.fileno()).st_ino
                self.printed_command(["--arch", "riscv64", "--working-disk-dir", str(directory)])
                self.assertEqual((directory / ".lock").stat().st_ino, inode)
                self.assertEqual(disk.read_bytes(), b"active working disk")
                self.assertEqual(sorted(path.name for path in directory.iterdir()), [".lock", "root.raw"])

    def test_orthogonal_overrides_only_add_explicit_boot_arguments(self):
        """验证 `--shell`、`--debug`、`--system-only`、`--init-program` 和 `--root-mode` 仅追加对应的启动或调试参数，不影响其他配置。"""
        base = ["--arch", "riscv64", "--kernel-elf", "chosen/kernel.elf",
                "--system-image", "chosen/system.img", "--contest-image", "chosen/contest.img"]
        default, _ = self.printed_command(base)
        shell, _ = self.printed_command([*base, "--shell"])
        debug, _ = self.printed_command([*base, "--debug"])
        both, _ = self.printed_command([*base, "--shell", "--debug"])
        self.assertEqual(shell, default + ["-append", "blossom.boot=shell"])
        self.assertEqual(debug, default + ["-S", "-gdb", "tcp:127.0.0.1:1234"])
        self.assertEqual(both, shell + ["-S", "-gdb", "tcp:127.0.0.1:1234"])
        # 验证单盘模式下挂载设备号调整为 /dev/vda，且不输出比赛盘摘要
        single, summary = self.printed_command([*base, "--system-only", "--shell", "--root-mode", "rw"])
        self.assertEqual(single.count("-drive"), 1)
        self.assertEqual(single[single.index("-append") + 1], "root=/dev/vda rw blossom.boot=shell")
        self.assertNotIn("role=contest", summary)
        self.assertIn("-netdev", single)
        init, _ = self.printed_command([*base, "--system-only", "--init-program", "/bin/probe"])
        self.assertEqual(init[init.index("-append") + 1], "root=/dev/vda init=/bin/probe")
        root_mode, _ = self.printed_command([*base, "--root-mode", "ro"])
        self.assertEqual(root_mode[root_mode.index("-append") + 1], "ro")

    def test_invalid_options_fail_before_any_file_change(self):
        """验证缺失必填项、未知架构、冲突选项、缩写参数或非法数值/字符均以退出码 2 失败，且不改动工作盘目录。"""
        # 汇总各类非法的命令行参数组合与边界值
        invalid = [[], ["--arch", "mips"], ["--arc", "riscv64"],
                   ["--arch", "riscv64", "--system-only"],
                   ["--arch", "riscv64", "--shell", "--init-program", "/bin/sh"],
                   ["--arch", "riscv64", "--network", "none", "--host-forward", "tcp::1234-:80"],
                   ["--arch", "riscv64", "--gdb-port", "1234"]]
        invalid += [["--arch", "riscv64", "--debug", "--gdb-port", value]
                    for value in ("0", "65536", "not-a-port")]
        invalid += [["--arch", "riscv64", "--cpus", value] for value in ("0", "-1", "one")]
        invalid += [["--arch", "riscv64", "--init-program", value]
                    for value in ("", "/bin/a b", "/bin/a\tb", "/bin/a\nb", "/bin/a\rb", "/bin/a\vb", "/bin/a\fb")]
        invalid += [["--arch", "riscv64", "--root-mode", value] for value in ("", "RO", "ro rw")]
        invalid += [["--arch", "riscv64", option] for option in ("--single", "--interactive", "--boot-mode", "--rootdir")]
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            work = base / "work"
            work.mkdir()
            disk = work / "root.raw"
            disk.write_bytes(b"preserved")
            for arguments in invalid:
                with self.subTest(arguments=arguments):
                    result = self.invoke([*arguments, "--working-disk-dir", str(work), "--print-command"], base)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertEqual(sorted(path.name for path in work.iterdir()), ["root.raw"])
                    self.assertEqual(disk.read_bytes(), b"preserved")

    def test_valid_numeric_edges_and_memory_are_forwarded_without_extra_policy(self):
        """验证正整数 CPU 数量、任意内存大小字符串以及 GDB 端口边界值（1 和 65535）均被原样转发给 QEMU。"""
        for port in (1, 65535):
            with self.subTest(port=port):
                command, _ = self.printed_command(["--arch", "loongarch64", "--cpus", "5",
                                                  "--memory", "QEMU-validates-this", "--debug",
                                                  "--gdb-port", str(port), "--network", "none"])
                self.assertEqual(command[command.index("-smp") + 1], "5")
                self.assertEqual(command[command.index("-m") + 1], "QEMU-validates-this")
                self.assertEqual(command[command.index("-gdb") + 1], f"tcp:127.0.0.1:{port}")
                self.assertNotIn("-netdev", command)

    def test_special_paths_are_repository_relative_and_round_trip_argv(self):
        """验证相对路径无论调用方当前工作目录如何均相对于仓库根目录解析，且含空格、逗号和引号的路径能正确转义。"""
        with tempfile.TemporaryDirectory() as temporary:
            relative = Path(os.path.relpath(
                Path(temporary) / "missing parent" / "lexical" / ".." / "chosen , 'quoted'", REPOSITORY_ROOT))
            absolute = Path(os.path.abspath(REPOSITORY_ROOT / relative))
            command, summary = self.printed_command([
                "--arch", "riscv64", "--kernel-elf", str(relative / "kernel.elf"),
                "--system-image", str(relative / "system.img"),
                "--contest-image", str(relative / "contest.img"),
                "--working-disk-dir", str(relative / "work , 'dir'"),
                "--qemu-binary", str(relative / "qemu binary"),
            ], temporary)
            self.assertEqual(command[0], str(absolute / "qemu binary"))
            self.assertEqual(command[command.index("-kernel") + 1], str(absolute / "kernel.elf"))
            self.assertIn(f"source={absolute / 'system.img'}", summary)
            self.assertIn(f"source={absolute / 'contest.img'}", summary)
            for value in (value for value in command if value.startswith("if=none,")):
                self.assertIn(str(absolute).replace(",", ",,"), value)
            self.assertEqual(list(Path(temporary).iterdir()), [])
            self.assertFalse(absolute.exists())

    def test_source_consumption_summary_follows_action(self):
        """验证标准错误中的 `source-consumed` 摘要字段准确反映 `fresh`、`reuse` 和 `refresh-system` 三种工作盘策略。"""
        for action, system, contest in (("fresh", "true", "true"), ("reuse", "false", "false"),
                                        ("refresh-system", "true", "false")):
            with self.subTest(action=action):
                _, summary = self.printed_command(["--arch", "riscv64", "--working-disk-action", action])
                roles = {line.split("role=", 1)[1].split()[0]: line
                         for line in summary.splitlines() if "role=" in line}
                self.assertIn(f"source-consumed={system}", roles["system"])
                self.assertIn(f"source-consumed={contest}", roles["contest"])

    def test_runtime_validates_elf_and_executable_before_preparing_disks(self):
        """验证真正执行启动时，缺失内核、非法 ELF、架构不匹配的 ELF 或不可执行的 QEMU 二进制均在创建工作盘前报错退出。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            kernel = base / "kernel.elf"
            work = base / "work"
            binary = base / "not-executable"
            binary.write_bytes(b"not an executable")
            common = ["--arch", "riscv64", "--kernel-elf", str(kernel),
                      "--qemu-binary", str(binary), "--working-disk-dir", str(work)]
            # 依次测试内核不存在、非 ELF 文件、架构不匹配的 ELF 以及 QEMU 二进制不可执行四种失败场景
            for image in (None, b"not ELF", bytes(kernel_image("loongarch64")[0]),
                          bytes(kernel_image("riscv64")[0])):
                with self.subTest(image=None if image is None else len(image)):
                    if image is not None:
                        kernel.write_bytes(image)
                    result = self.invoke(common, base)
                    self.assertEqual(result.returncode, 1, result.stderr)
                    self.assertFalse(work.exists())

    def test_exec_inherits_lock_and_exit_status_and_reuse_ignores_missing_sources(self):
        """验证 `run_qemu.py` 通过 `exec` 启动的子进程继承工作目录独占文件锁并透传退出码，且 `reuse` 模式无需源镜像继续存在。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            kernel = base / "kernel.elf"
            kernel.write_bytes(kernel_image("riscv64")[0])
            system = base / "system.img"
            contest = base / "contest.img"
            system.write_bytes(b"system input")
            contest.write_bytes(b"contest input")
            work = base / "work"
            # 构造模拟 QEMU 的探针脚本：尝试非阻塞获取同一个 .lock 文件的独占锁，若锁已被继承则返回退出码 37
            executable = base / "lock-observer"
            executable.write_text(
                f"#!{sys.executable}\n"
                "import fcntl,sys\n"
                f"with open({str(work / '.lock')!r}, 'a+b') as lock:\n"
                "    try:\n"
                "        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)\n"
                "    except BlockingIOError:\n"
                "        print('inherited lock held', flush=True)\n"
                "        sys.exit(37)\n"
                "sys.exit(0)\n", encoding="utf-8")
            executable.chmod(0o700)
            arguments = ["--arch", "riscv64", "--kernel-elf", str(kernel),
                         "--system-image", str(system), "--contest-image", str(contest),
                         "--working-disk-dir", str(work), "--qemu-binary", str(executable)]
            fresh = self.invoke(arguments, base)
            self.assertEqual(fresh.returncode, 37, fresh.stderr)
            self.assertIn("inherited lock held", fresh.stdout)
            self.assertEqual(system.read_bytes(), b"system input")
            self.assertEqual(contest.read_bytes(), b"contest input")
            # 删除源镜像后使用 reuse 模式再次启动，验证无需读取源镜像即可直接复用已有工作盘
            (work / "root.raw").write_bytes(b"preserved system work")
            (work / "contest.raw").write_bytes(b"preserved contest work")
            system.unlink()
            contest.unlink()
            reused = self.invoke([*arguments, "--working-disk-action", "reuse"], base)
            self.assertEqual(reused.returncode, 37, reused.stderr)
            self.assertEqual((work / "root.raw").read_bytes(), b"preserved system work")
            self.assertEqual((work / "contest.raw").read_bytes(), b"preserved contest work")


if __name__ == "__main__":
    unittest.main()
