#!/usr/bin/env python3
"""用户态自测（user selftests）自动化镜像准备与 QEMU 运行入口脚本。

本脚本负责在宿主机总超时预算内完成一轮完整的用户态自测：
1. 解析自测目录、测试用例过滤项、镜像制作配置及 QEMU 启动参数，校验输出路径与工作盘目录不发生重叠；
2. 生成本轮唯一的 128 位十六进制运行标识（`run_id`），并设置覆盖整轮执行的单调时钟截止期限与 `ITIMER_REAL` 定时器；
3. 调用 `prepare_user_selftest_rootfs.py` 打包用户态自测根树并制作为 ext4 系统镜像；
4. 通过 `prepare_qemu` 获取工作盘排他锁、复制系统工作盘并启动 QEMU，将串口输出完整记录到日志文件；
5. 待 QEMU 正常退出后，在仍持有工作盘锁的状态下用 `debugfs` 读取工作盘中的 `/test/selftests/exit-status`，
   核对 `run_id` 与本轮一致后返回客户机自测退出码（若遇超时、信号取消或宿主机错误，先终止回收子进程组再释放工作盘锁，并统一返回 2）。
"""

import sys

sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import math
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import time

from create_ext4_image import PROFILES
from rootfs_paths import output_path, reject_overlap
from run_qemu import configure_qemu, parse_arguments, prepare_qemu, repository_path

DEFAULT_HOST_TIMEOUT_SECONDS = 900
HOST_CANCELLATION_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGALRM)


def positive_seconds(value):
    """解析并校验宿主机总超时秒数参数，要求必须为有限正浮点数。"""
    try:
        seconds = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("host timeout must be a positive finite number") from None
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("host timeout must be a positive finite number")
    return seconds


def remaining_seconds(deadline):
    """计算距离整轮自测单调时钟截止期限（`deadline`）的剩余秒数；若已超时则立即抛出 `TimeoutError`。"""
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("host selftest deadline expired")
    return remaining


@contextmanager
def host_deadline(deadline):
    """为整轮自测安装信号处理器与 `ITIMER_REAL` 实时定时器上下文。

    首次收到 `SIGINT`、`SIGTERM` 或 `SIGALRM` 时立即停止定时器、忽略后续同类取消信号并抛出对应异常，
    从而让外层 `finally` 块在不被二次信号打断的情况下完成子进程回收与工作盘锁释放。
    """
    handlers = {number: signal.getsignal(number) for number in HOST_CANCELLATION_SIGNALS}

    def cancel(number, _):
        """处理首次取消或超时信号：关闭实时定时器并屏蔽后续信号，转化为 Python 异常以触发清理流程。"""
        signal.setitimer(signal.ITIMER_REAL, 0)
        for pending in HOST_CANCELLATION_SIGNALS:
            signal.signal(pending, signal.SIG_IGN)
        if number == signal.SIGALRM:
            raise TimeoutError("host selftest deadline expired")
        raise KeyboardInterrupt(f"host selftest cancelled by {signal.Signals(number).name}")

    try:
        for number in HOST_CANCELLATION_SIGNALS:
            signal.signal(number, cancel)
        signal.setitimer(signal.ITIMER_REAL, remaining_seconds(deadline))
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        for number, handler in handlers.items():
            signal.signal(number, handler)


def stop_and_wait(process):
    """终止子进程所属的独立进程组并等待其彻底退出。

    回收期间临时阻塞宿主机取消信号，先发送 `SIGTERM` 给予 5 秒优雅退出时间，
    超时后补发 `SIGKILL` 强制清理整个进程组，确保不会遗留孤儿 QEMU 或构建进程。
    """
    mask = signal.pthread_sigmask(signal.SIG_BLOCK, HOST_CANCELLATION_SIGNALS)
    try:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            finally:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        process.wait()
    finally:
        signal.pthread_sigmask(signal.SIG_SETMASK, mask)


@contextmanager
def _defer_host_cancellation():
    """在 `subprocess.Popen` 创建子进程并赋值给局部变量期间暂缓抛出取消异常。

    如果在 `Popen` 返回前后瞬间直接抛出信号异常，调用栈可能来不及把句柄赋给 `process` 变量，
    导致 `finally` 块无法回收刚刚启动的子进程。此处通过暂存首个信号编号并在退出上下文时重发来消除该竞态窗口。
    """
    handlers = {number: signal.getsignal(number) for number in HOST_CANCELLATION_SIGNALS}
    pending = None

    def record(number, _):
        """记录临界区内收到的首个取消信号编号，留待退出上下文并恢复原处理器后再触发。"""
        nonlocal pending
        if pending is None:
            pending = number

    try:
        for number in HOST_CANCELLATION_SIGNALS:
            signal.signal(number, record)
        yield
    finally:
        for number, handler in handlers.items():
            signal.signal(number, handler)
        if pending is not None:
            signal.raise_signal(pending)


def run_host_process(command, environment, output, deadline):
    """在独立进程组中启动宿主机子进程并按总截止期限等待其完成，确保任何异常退出路径都会终止并回收该进程组。"""
    remaining_seconds(deadline)
    process = None
    try:
        with _defer_host_cancellation():
            process = subprocess.Popen(
                command, env=environment, stdin=subprocess.DEVNULL,
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        return process.wait(timeout=remaining_seconds(deadline))
    finally:
        if process is not None:
            stop_and_wait(process)


def wait_for_qemu(command, environment, uart_path, deadline):
    """启动 QEMU 并将标准输出与标准错误合并写入新建的串口日志文件（`uart_path`），等待虚拟机退出。"""
    with Path(uart_path).open("xb") as uart:
        return run_host_process(command, environment, uart, deadline)


def read_result(system_working_disk, run_id, deadline):
    """通过 `debugfs` 从系统工作盘读取 `/test/selftests/exit-status`，校验运行标识并返回自测退出码。

    参数：
    - `system_working_disk`：QEMU 退出后的系统工作盘镜像路径。
    - `run_id`：本轮预期的 32 位十六进制运行标识。
    - `deadline`：整轮自测单调时钟截止期限。

    返回：客户机写入的自测状态码（`0`、`1`、`2` 或 `4`）。
    异常：当 `debugfs` 读取失败、内容格式不合法或记录中的 `run_id` 与本轮不一致（旧盘残留结果）时抛出 `ValueError`。
    """
    result = subprocess.run(
        ["debugfs", "-R", "cat /test/selftests/exit-status", str(system_working_disk)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
        timeout=remaining_seconds(deadline))
    if result.returncode != 0 or not result.stdout:
        diagnostic = result.stderr.decode("utf-8", errors="replace").strip()
        raise ValueError(
            f"selftest result read failed in {system_working_disk} "
            f"(debugfs status={result.returncode}): {diagnostic}")
    match = re.fullmatch(rb"run=([0-9a-f]{32}) status=([0124])\n", result.stdout)
    if match is None:
        raise ValueError(f"malformed selftest result in {system_working_disk}")
    if match[1].decode("ascii") != run_id:
        raise ValueError(f"stale selftest result in {system_working_disk}")
    remaining_seconds(deadline)
    return int(match[2])


def run_selftest_qemu(configuration, run_id, uart_path, deadline):
    """在持有工作盘排他锁的上下文中准备镜像、运行 QEMU 并从系统工作盘读取自测结果。"""
    with prepare_qemu(configuration) as (command, environment, disk_paths, _):
        print(f"user-selftests run={run_id} uart={uart_path} "
              f"working-disk={disk_paths['system']}", file=sys.stderr, flush=True)
        status = wait_for_qemu(command, environment, uart_path, deadline)
        if status != 0:
            raise ValueError(f"QEMU exited with status {status}")
        return read_result(disk_paths["system"], run_id, deadline)


def prepare_run_input(args, configuration, run_id, deadline):
    """调用 `prepare_user_selftest_rootfs.py` 子进程构建包含本轮 `run_id` 的自测根文件系统与 ext4 镜像。"""
    command = [
        sys.executable, str(Path(__file__).with_name("prepare_user_selftest_rootfs.py")),
        "--arch", args.arch,
        "--guest-userland-directory", str(args.guest_userland_directory),
        "--runtime-libraries-directory", str(args.runtime_libraries_directory),
        "--output-directory", str(args.output_directory),
        "--output-image", str(configuration.system_image_path),
        "--profile", args.profile, "--size-mib", str(args.size_mib), "--run-id", run_id]
    for option, values in (("--collection", args.collection), ("--program", args.program)):
        for value in values:
            command.extend((option, value))
    if args.program_timeout_seconds is not None:
        command.extend(("--program-timeout-seconds", str(args.program_timeout_seconds)))
    status = run_host_process(command, None, sys.stderr, deadline)
    if status != 0:
        raise ValueError(f"selftest preparation exited with status {status}")


def check_preparation_outputs(args, configuration):
    """校验自测根树输出目录、系统镜像及其日志文件不与 QEMU 工作盘目录发生包含或硬链接重叠。"""
    directory = output_path(configuration.working_disk_directory, directory=True)
    image = configuration.system_image_path
    for output in (
            output_path(args.output_directory, directory=True),
            output_path(image),
            output_path(image.with_suffix(".log"))):
        reject_overlap(directory, output)


def main(argv=None):
    """脚本主入口：完成参数校验、自测镜像制作、QEMU 执行与退出码汇总（宿主机错误统一返回 2）。"""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--guest-userland-directory", type=repository_path, required=True)
    parser.add_argument("--runtime-libraries-directory", type=repository_path, required=True)
    parser.add_argument("--output-directory", type=repository_path, required=True)
    parser.add_argument("--collection", action="append", default=[])
    parser.add_argument("--program", action="append", default=[])
    parser.add_argument("--program-timeout-seconds", type=int)
    parser.add_argument("--host-timeout-seconds", type=positive_seconds,
                        default=DEFAULT_HOST_TIMEOUT_SECONDS,
                        help="absolute Host deadline in seconds (default: %(default)s)")
    parser.add_argument("--profile", choices=PROFILES, default="seed4k")
    parser.add_argument("--size-mib", type=int, default=256)
    parser.set_defaults(system_only=True, init_program="/sbin/init", root_mode="rw")
    _, args = parse_arguments(argv, parser)

    # 校验自动化自测专属的启动模式与工作盘刷新约束。
    if args.system_image is None:
        parser.error("--system-image is required for the prepared selftest image")
    if args.shell or args.init_program != "/sbin/init" or args.root_mode != "rw":
        parser.error("automatic selftests require ordinary /sbin/init and a writable root")
    if args.print_command:
        parser.error("--print-command belongs to the manual run-qemu entry")
    if args.working_disk_action == "reuse":
        parser.error("automatic selftests require fresh or refresh-system to consume this run's input")
    configuration = configure_qemu(args)
    try:
        check_preparation_outputs(args, configuration)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    # 生成本轮唯一标识与总截止时间，并在受控超时上下文中依次完成镜像准备与 QEMU 运行。
    run_id = secrets.token_hex(16)
    deadline = time.monotonic() + args.host_timeout_seconds
    uart_path = configuration.working_disk_directory / f"selftests-{run_id}.uart.log"
    print(f"user-selftests run={run_id} host-timeout-seconds={args.host_timeout_seconds} "
          f"uart={uart_path}", file=sys.stderr, flush=True)
    try:
        with host_deadline(deadline):
            prepare_run_input(args, configuration, run_id, deadline)
            remaining_seconds(deadline)
            status = run_selftest_qemu(configuration, run_id, uart_path, deadline)
    except (OSError, ValueError, subprocess.SubprocessError, TimeoutError, KeyboardInterrupt) as error:
        print(f"user-selftests: {error}", file=sys.stderr, flush=True)
        return 2
    print(f"user-selftests run={run_id} status={status} uart={uart_path}",
          file=sys.stderr, flush=True)
    return status


if __name__ == "__main__":
    sys.exit(main())
