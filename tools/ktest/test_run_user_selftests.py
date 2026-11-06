"""`run_user_selftests.py` 宿主自测驱动回归测试。

本模块验证用户态自测宿主脚本的关键行为边界：
- 准备阶段输出路径与工作盘目录的符号链接/硬链接冲突检查；
- 从 ext4 镜像 `/test/selftests/exit-status` 读取并校验单轮退出状态记录；
- 在 QEMU 子进程运行及结果读取期间保持工作盘 `.lock` 独占文件锁；
- 整轮运行超时、信号取消（SIGINT/SIGTERM）下的子进程组终止回收与信号处理器恢复。
"""

import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_user_selftest_rootfs
from host_helpers import TOOLS
from run_qemu import configure_qemu, parse_arguments, prepare_qemu
from run_user_selftests import (HOST_CANCELLATION_SIGNALS, check_preparation_outputs,
                               host_deadline, read_result, run_host_process,
                               stop_and_wait, wait_for_qemu)
from test_boot import kernel_image

RUN_ID = "0123456789abcdef0123456789abcdef"
HAS_E2FSPROGS = all(shutil.which(name) for name in ("mke2fs", "debugfs"))


class UserSelftestHostTests(unittest.TestCase):
    """验证 `run_user_selftests.py` 的结果镜像解析、进程超时回收与工作盘锁生命周期。"""

    def setUp(self):
        """为每个测试用例创建独立的临时工作目录与源镜像路径，并在测试结束后自动清理。"""
        temporary = tempfile.TemporaryDirectory(prefix="selftest-host-")
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.image = self.base / "source.ext4"

    def make_volume(self):
        """调用 `mke2fs` 创建一个包含 `/test/selftests` 目录的 8 MiB 无日志 ext4 测试镜像。"""
        tree = self.base / "tree"
        (tree / "test/selftests").mkdir(parents=True)
        with self.image.open("wb") as stream:
            stream.truncate(8 * 1024 * 1024)
        subprocess.run(["mke2fs", "-q", "-F", "-t", "ext4", "-O", "^has_journal",
                        "-d", str(tree), str(self.image)], capture_output=True, check=True)

    def replace_result(self, image, content):
        """使用 `debugfs` 更新指定 ext4 镜像内的 `/test/selftests/exit-status` 文件内容。

        参数：
            image: 目标 ext4 镜像路径。
            content: 要写入的字节内容；若为 `None`，则仅删除该状态文件。
        """
        subprocess.run(["debugfs", "-w", "-R", "rm /test/selftests/exit-status", str(image)],
                       capture_output=True, check=True)
        if content is not None:
            payload = self.base / "result.bytes"
            payload.write_bytes(content)
            subprocess.run(["debugfs", "-w", "-R",
                            f"write {payload} /test/selftests/exit-status", str(image)],
                           capture_output=True, check=True)

    def wait_for_path(self, path, process):
        """轮询等待受控子进程创建阶段标记文件；若子进程提前退出或超时则直接断言失败。"""
        deadline = time.monotonic() + 5
        while not path.exists():
            self.assertIsNone(process.poll(), "host exited before reaching the controlled stage")
            self.assertLess(time.monotonic(), deadline, f"missing lifecycle marker: {path}")
            time.sleep(0.01)

    def test_preparation_rejects_output_aliases_of_live_working_directory(self):
        """验证准备阶段会拒绝通过父目录符号链接或文件硬链接指向活动工作盘目录的输出路径。"""
        work = self.base / "work"
        work.mkdir()
        lock = work / ".lock"
        lock.write_bytes(b"active work lock")
        # 构造指向工作目录的父目录符号链接，验证输出目录落在工作目录内时被拒绝。
        parent_link = self.base / "ordinary-parent-link"
        parent_link.symlink_to(work, target_is_directory=True)
        arguments = SimpleNamespace(output_directory=parent_link / "root")
        configuration = SimpleNamespace(
            working_disk_directory=work, system_image_path=self.image)
        with self.assertRaises(ValueError):
            check_preparation_outputs(arguments, configuration)
        # 构造与工作目录锁文件同 inode 的硬链接作为系统镜像输出路径，验证被拒绝。
        arguments.output_directory = self.base / "published-root"
        os.link(lock, self.image)
        with self.assertRaises(ValueError):
            check_preparation_outputs(arguments, configuration)

    @unittest.skipUnless(HAS_E2FSPROGS, "requires e2fsprogs")
    def test_complete_current_record_preserves_status_and_rejects_other_records(self):
        """验证 `read_result` 仅接受格式完整且 `run_id` 匹配的本轮合法状态码（0、1、2、4），拒绝其他异常格式。"""
        self.make_volume()
        # 合法的四种状态码应原样解析返回。
        for status in (0, 1, 2, 4):
            with self.subTest(status=status):
                self.replace_result(self.image, f"run={RUN_ID} status={status}\n".encode())
                self.assertEqual(read_result(self.image, RUN_ID, time.monotonic() + 5), status)
        # 缺失文件、空文件、旧格式纯数字、截断、多行、非法状态码、CRLF、非本轮或大写 RUN_ID 均须抛出 ValueError。
        valid = f"run={RUN_ID} status=0\n".encode()
        invalid = [None, b"", b"0\n", valid[:-1], valid + b"\n", valid + valid,
                   valid.replace(b"status=0", b"status=3"), valid.replace(b"\n", b"\r\n"),
                   valid.replace(RUN_ID.encode(), b"f" * 32),
                   valid.replace(RUN_ID.encode(), RUN_ID.upper().encode())]
        for record in invalid:
            with self.subTest(record=record):
                self.replace_result(self.image, record)
                with self.assertRaises(ValueError):
                    read_result(self.image, RUN_ID, time.monotonic() + 5)
        # 损坏的非 ext4 镜像文件应触发解析或 debugfs 异常。
        self.image.write_bytes(b"not an ext4 volume")
        with self.assertRaises((ValueError, subprocess.CalledProcessError)):
            read_result(self.image, RUN_ID, time.monotonic() + 5)

    @unittest.skipUnless(HAS_E2FSPROGS, "requires e2fsprogs")
    def test_production_context_keeps_lock_through_wait_and_working_volume_read(self):
        """验证 `prepare_qemu` 上下文在子进程退出后及读取工作盘结果期间持续持有 `.lock` 独占锁，且只从工作盘读取结果。"""
        self.make_volume()
        kernel = self.base / "kernel.elf"
        kernel.write_bytes(kernel_image("riscv64")[0])
        work = self.base / "work"
        _, arguments = parse_arguments([
            "--arch", "riscv64", "--kernel-elf", str(kernel),
            "--system-image", str(self.image), "--working-disk-dir", str(work),
            "--system-only", "--init-program", "/sbin/init", "--qemu-binary", sys.executable])
        configuration = configure_qemu(arguments)
        with prepare_qemu(configuration) as (_, environment, disks, _):
            # 分别向工作盘和源镜像写入不同状态码，确认后续读取的是工作盘而非源镜像。
            self.replace_result(disks["system"], f"run={RUN_ID} status=4\n".encode())
            self.replace_result(self.image, f"run={RUN_ID} status=0\n".encode())
            child = (
                "import fcntl,sys\n"
                f"with open({str(work / '.lock')!r}, 'r+b') as lock:\n"
                "    try:\n"
                "        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)\n"
                "    except BlockingIOError:\n"
                "        sys.exit(0)\n"
                "sys.exit(1)\n")
            # 子进程运行期间及退出后，外部进程尝试对 `.lock` 加非阻塞排他锁均应触发 BlockingIOError。
            self.assertEqual(wait_for_qemu([sys.executable, "-c", child], environment,
                                          work / "child.uart", time.monotonic() + 5), 0)
            with (work / ".lock").open("r+b") as competitor:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(competitor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertEqual(read_result(disks["system"], RUN_ID, time.monotonic() + 5), 4)
        # 退出 `prepare_qemu` 上下文后，`.lock` 锁应已释放，外部可正常获取排他锁。
        with (work / ".lock").open("r+b") as competitor:
            fcntl.flock(competitor, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_output_does_not_renew_deadline_and_timeout_reaps_child(self):
        """验证子进程持续输出串口日志不会刷新截止时间，超时后会终止并回收整个子进程组且保留已写入日志。"""
        uart = self.base / "flood.uart"
        child = (
            "import os,signal,subprocess,sys,time\n"
            "sleeper=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])\n"
            "def stop(*args):\n"
            "    sleeper.wait(timeout=2)\n"
            "    os.write(1,b'STOPPED\\n')\n"
            "    sys.exit(0)\n"
            "signal.signal(signal.SIGTERM,stop)\n"
            "print(os.getpid(),flush=True)\n"
            "while True:\n"
            "    os.write(1,b'progress\\n')\n"
            "    time.sleep(0.01)\n")
        # 即使子进程以 10ms 间隔不断输出日志，到达固定截止时间后仍须抛出 TimeoutExpired。
        with self.assertRaises(subprocess.TimeoutExpired):
            wait_for_qemu([sys.executable, "-c", child], os.environ.copy(), uart,
                          time.monotonic() + 0.5)
        # 验证串口日志完整保留了超时前的输出与 SIGTERM 处理收尾输出，且子进程已被 wait 回收。
        output = uart.read_bytes()
        pid = int(output.splitlines()[0])
        self.assertIn(b"progress\n", output)
        self.assertTrue(output.endswith(b"STOPPED\n"))
        with self.assertRaises(ChildProcessError):
            os.waitpid(pid, os.WNOHANG)

    def test_creation_cancellation_is_delivered_only_after_child_is_owned_and_reaped(self):
        """验证当 `Popen` 刚创建子进程但尚未返回时收到取消信号，宿主会先终止并回收子进程，再向上抛出取消异常。"""
        ready = self.base / "created-child-ready"
        mask = sorted(int(number) for number in signal.pthread_sigmask(signal.SIG_BLOCK, []))
        handlers = {number: signal.getsignal(number) for number in HOST_CANCELLATION_SIGNALS}
        child = (
            "import signal,time\n"
            "from pathlib import Path\n"
            "blocked=sorted(int(n) for n in signal.pthread_sigmask(signal.SIG_BLOCK, []))\n"
            f"assert blocked=={mask!r}\n"
            f"Path({str(ready)!r}).touch()\n"
            "time.sleep(30)\n")
        original_popen = subprocess.Popen
        created = []
        creation_returned = []

        def interrupt_after_creation(*args, **kwargs):
            """包装 `subprocess.Popen`：等待子进程启动并校验继承的信号掩码后，向当前宿主进程发送 SIGTERM。"""
            process = original_popen(*args, **kwargs)
            created.append(process)
            self.wait_for_path(ready, process)
            self.assertIsNone(process.poll())
            os.kill(os.getpid(), signal.SIGTERM)
            creation_returned.append(True)
            return process

        deadline = time.monotonic() + 10
        try:
            with (self.base / "creation.uart").open("xb") as output:
                with self.assertRaisesRegex(KeyboardInterrupt, "SIGTERM"), host_deadline(deadline):
                    with patch("run_user_selftests.subprocess.Popen", new=interrupt_after_creation):
                        run_host_process([sys.executable, "-c", child],
                                         os.environ.copy(), output, deadline)
            # 确认子进程已被 SIGTERM 终止并彻底回收，且宿主的信号掩码、信号处理器和定时器均恢复原状。
            self.assertEqual(creation_returned, [True])
            self.assertEqual(len(created), 1)
            self.assertEqual(created[0].returncode, -signal.SIGTERM)
            with self.assertRaises(ChildProcessError):
                os.waitpid(created[0].pid, os.WNOHANG)
            self.assertEqual(
                sorted(int(number) for number in signal.pthread_sigmask(signal.SIG_BLOCK, [])), mask)
            self.assertEqual(
                {number: signal.getsignal(number) for number in HOST_CANCELLATION_SIGNALS}, handlers)
            self.assertEqual(signal.getitimer(signal.ITIMER_REAL), (0.0, 0.0))
        finally:
            for process in created:
                stop_and_wait(process)

    def test_deadline_scope_restores_handlers_and_disarms_timer(self):
        """验证 `host_deadline` 上下文在正常结束、超时或收到取消信号后，均会关闭定时器并恢复原信号处理器。"""
        handlers = {number: signal.getsignal(number) for number in HOST_CANCELLATION_SIGNALS}
        with host_deadline(time.monotonic() + 5):
            pass
        # 定时器到期时应在阻塞调用中抛出 TimeoutError；收到 SIGINT/SIGTERM 时应转换为 KeyboardInterrupt。
        with self.assertRaises(TimeoutError):
            with host_deadline(time.monotonic() + 0.05):
                time.sleep(5)
        for number in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=number), self.assertRaises(KeyboardInterrupt):
                with host_deadline(time.monotonic() + 5):
                    os.kill(os.getpid(), number)
        self.assertEqual(signal.getitimer(signal.ITIMER_REAL), (0.0, 0.0))
        self.assertEqual(
            {number: signal.getsignal(number) for number in HOST_CANCELLATION_SIGNALS}, handlers)

    @unittest.skipUnless(HAS_E2FSPROGS, "requires e2fsprogs")
    def test_cancellation_reaps_live_process_before_releasing_work_lock(self):
        """验证在运行或清理阶段收到 SIGINT/SIGTERM 时，宿主会先等待子进程完全退出并回收，随后才释放工作盘锁。"""
        self.make_volume()
        kernel = self.base / "kernel.elf"
        kernel.write_bytes(kernel_image("riscv64")[0])
        for number in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=number):
                work = self.base / f"work-{number}"
                ready = self.base / f"ready-{number}"
                stopping = self.base / f"stopping-{number}"
                held = self.base / f"held-{number}"
                arguments = [
                    "--arch", "riscv64", "--kernel-elf", str(kernel),
                    "--system-image", str(self.image), "--working-disk-dir", str(work),
                    "--system-only", "--init-program", "/sbin/init",
                    "--qemu-binary", sys.executable]
                child = (
                    "import fcntl,os,signal,sys,time\n"
                    "from pathlib import Path\n"
                    "def stop(*args):\n"
                    "    \"\"\"延迟正常退出期间检查 Host 仍持有工作盘锁。\"\"\"\n"
                    f"    Path({str(stopping)!r}).touch()\n"
                    "    time.sleep(0.3)\n"
                    f"    with open({str(work / '.lock')!r},'r+b') as lock:\n"
                    "        try:\n"
                    "            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)\n"
                    "        except BlockingIOError:\n"
                    f"            Path({str(held)!r}).touch()\n"
                    "        else:\n"
                    "            sys.exit(1)\n"
                    "    sys.exit(0)\n"
                    "signal.signal(signal.SIGTERM,stop)\n"
                    f"ready=Path({str(ready)!r})\n"
                    "pending=ready.with_suffix('.pending')\n"
                    "pending.write_text(str(os.getpid()))\n"
                    "pending.replace(ready)\n"
                    "while True:\n"
                    "    time.sleep(1)\n")
                bootstrap = (
                    "import os,signal,sys,time\n"
                    "from pathlib import Path\n"
                    f"sys.path.insert(0,{str(TOOLS)!r})\n"
                    "from run_qemu import configure_qemu,parse_arguments,prepare_qemu\n"
                    "from run_user_selftests import HOST_CANCELLATION_SIGNALS,host_deadline,wait_for_qemu\n"
                    f"_,args=parse_arguments({arguments!r})\n"
                    "handlers={n:signal.getsignal(n) for n in HOST_CANCELLATION_SIGNALS}\n"
                    "deadline=time.monotonic()+5\n"
                    "try:\n"
                    "    with host_deadline(deadline):\n"
                    "        with prepare_qemu(configure_qemu(args)) as (_,env,disks,lock):\n"
                    f"            wait_for_qemu([sys.executable,'-c',{child!r}],env,\n"
                    f"                          {str(work / 'child.uart')!r},deadline)\n"
                    "except KeyboardInterrupt:\n"
                    f"    pid=int(Path({str(ready)!r}).read_text())\n"
                    "    try:\n"
                    "        os.waitpid(pid,os.WNOHANG)\n"
                    "    except ChildProcessError:\n"
                    "        pass\n"
                    "    else:\n"
                    "        raise AssertionError('child was not reaped')\n"
                    "    assert signal.getitimer(signal.ITIMER_REAL)==(0.0,0.0)\n"
                    "    assert {n:signal.getsignal(n) for n in HOST_CANCELLATION_SIGNALS}==handlers\n"
                    "    sys.exit(2)\n"
                    "sys.exit(1)\n")
                host = subprocess.Popen([sys.executable, "-c", bootstrap], start_new_session=True,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                pid = None
                try:
                    # 等待受控子进程就绪，确认此时工作盘锁正被持有。
                    self.wait_for_path(ready, host)
                    pid = int(ready.read_text())
                    with (work / ".lock").open("r+b") as competitor:
                        with self.assertRaises(BlockingIOError):
                            fcntl.flock(competitor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    # 发送首次取消信号触发子进程清理逻辑，并在清理窗口内连续追加取消信号，验证不会提前释放锁。
                    host.send_signal(number)
                    self.wait_for_path(stopping, host)
                    host.send_signal(signal.SIGINT)
                    host.send_signal(signal.SIGTERM)
                    _, error = host.communicate(timeout=2)
                    self.assertEqual(host.returncode, 2, error.decode(errors="replace"))
                    self.assertTrue(held.exists(), "work lock was released while child was alive")
                    with self.assertRaises(ProcessLookupError):
                        os.kill(pid, 0)
                    with (work / ".lock").open("r+b") as competitor:
                        fcntl.flock(competitor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                finally:
                    if host.poll() is None:
                        host.kill()
                    host.communicate()
                    if pid is not None:
                        try:
                            os.killpg(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass

    @unittest.skipUnless(test_user_selftest_rootfs.HAS_E2FSPROGS, "requires e2fsprogs")
    def test_main_deadline_interrupts_real_copy_without_publishing_work_volume(self):
        """验证整轮运行截止时间能够中断缓慢的工作盘复制过程，不会发布半成品工作盘且能正确释放文件锁。"""
        fixture = test_user_selftest_rootfs.UserSelftestRootfsTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.make_volume()
        self.replace_result(self.image, f"run={RUN_ID} status=4\n".encode())
        work = self.base / "work"
        work.mkdir()
        disk = work / "root.raw"
        shutil.copyfile(self.image, disk)
        previous = disk.read_bytes()
        kernel = self.base / "kernel.elf"
        kernel.write_bytes(kernel_image("riscv64")[0])
        marker = self.base / "copy-started"
        bootstrap = (
            "import shutil,sys,time\n"
            "from pathlib import Path\n"
            f"sys.path.insert(0,{str(TOOLS)!r})\n"
            "import run_user_selftests\n"
            "original=shutil.copyfileobj\n"
            "def slow_copy(source,target,length):\n"
            "    \"\"\"由原复制实现读取真实文件，只在每次读取后加入受控延迟。\"\"\"\n"
            "    class SlowSource:\n"
            "        def read(self,size):\n"
            "            \"\"\"保留真实字节和 EOF，延迟发生在正式工作盘发布之前。\"\"\"\n"
            "            data=source.read(size)\n"
            "            if data:\n"
            f"                Path({str(marker)!r}).touch()\n"
            "                time.sleep(0.1)\n"
            "            return data\n"
            "    return original(SlowSource(),target,length)\n"
            "shutil.copyfileobj=slow_copy\n"
            "sys.exit(run_user_selftests.main(sys.argv[1:]))\n")
        budget = 3
        command = [
            sys.executable, "-c", bootstrap, "--arch", "riscv64",
            "--guest-userland-directory", str(fixture.userland),
            "--runtime-libraries-directory", str(fixture.runtime),
            "--output-directory", str(fixture.output), "--system-image", str(fixture.image),
            "--size-mib", "128", "--kernel-elf", str(kernel),
            "--working-disk-dir", str(work), "--qemu-binary", sys.executable,
            "--host-timeout-seconds", str(budget)]
        # 启动注入了慢速复制的 `run_user_selftests.main`，验证超时后返回错误码 2、保留原有工作盘内容并释放锁。
        started = time.monotonic()
        host = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            _, error = host.communicate(timeout=budget + 2)
            self.assertLess(time.monotonic() - started, budget + 1)
            self.assertEqual(host.returncode, 2, error.decode(errors="replace"))
            self.assertTrue(marker.exists(), "deadline did not reach the real working-volume copy")
            self.assertEqual(disk.read_bytes(), previous)
            self.assertEqual(sorted(path.name for path in work.iterdir()), [".lock", "root.raw"])
            with (work / ".lock").open("r+b") as competitor:
                fcntl.flock(competitor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
            if host.poll() is None:
                host.kill()
            host.communicate()

    def test_preparation_output_parent_file_is_host_error(self):
        """验证当输出目录的父路径是已存在的普通文件时，脚本返回宿主错误码 2，且不修改该文件或创建工作目录。"""
        parent = self.base / "ordinary-file"
        parent.write_bytes(b"preserved input")
        result = subprocess.run(
            [sys.executable, str(TOOLS / "run_user_selftests.py"), "--arch", "riscv64",
             "--guest-userland-directory", str(self.base / "missing-userland"),
             "--runtime-libraries-directory", str(self.base / "missing-runtime"),
             "--output-directory", str(parent / "root"),
             "--system-image", str(self.image),
             "--working-disk-dir", str(self.base / "work")],
            capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(parent.read_bytes(), b"preserved input")
        self.assertFalse((self.base / "work").exists())
        self.assertFalse(self.image.exists())

    def test_cli_rejects_unsafe_outputs_reuse_and_invalid_deadlines_before_preparation(self):
        """验证 CLI 在实际准备根文件系统前先拦截路径冲突、`reuse` 动作与非法超时参数，且不产生任何文件。"""
        common = [sys.executable, str(TOOLS / "run_user_selftests.py"), "--arch", "riscv64",
                  "--guest-userland-directory", str(self.base / "missing-userland"),
                  "--runtime-libraries-directory", str(self.base / "missing-runtime"),
                  "--output-directory", str(self.base / "output"),
                  "--system-image", str(self.image), "--working-disk-dir", str(self.base / "work")]
        invalid = [["--working-disk-action", "reuse"]]
        invalid += [["--host-timeout-seconds", value] for value in ("0", "-1", "nan", "inf", "bad")]
        invalid += [["--output-directory", str(path)] for path in
                    (self.base, self.base / "work", self.base / "work/nested")]
        invalid += [["--system-image", str(path)] for path in
                    (self.base / "work", self.base / "work/root.raw")]
        invalid += [["--system-image", str(self.base / "work.img"),
                     "--working-disk-dir", str(self.base / "work.log")]]
        for arguments in invalid:
            with self.subTest(arguments=arguments):
                result = subprocess.run([*common, *arguments], capture_output=True, timeout=5)
                self.assertEqual(result.returncode, 2)
                self.assertEqual(list(self.base.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
