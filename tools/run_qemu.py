#!/usr/bin/env python3
"""生产环境 QEMU 启动与配置管理入口脚本。

本脚本负责为 `riscv64` 与 `loongarch64` 架构组装 QEMU 命令行、准备独占工作盘副本并启动虚拟机。
同时向自动化自测脚本（如 `run_user_selftests.py`）导出参数解析、配置构造与持锁准备上下文函数。
核心工作流程包括：
1. 解析架构、内核 ELF、系统盘/比赛盘镜像路径、工作盘刷新策略（`fresh`/`reuse`/`refresh-system`）、网络与 GDB 调试选项；
2. 构造不可变的 `QemuConfiguration` 对象（若指定 `--print-command` 则仅打印配置摘要与转义命令行后退出，不访问磁盘文件）；
3. 校验内核 ELF 格式与 QEMU 可执行文件，获取工作盘目录 `flock` 排他锁并按策略复制或复用工作盘镜像；
4. 将排他锁文件描述符设为可继承后，通过 `execve` 原地替换为 QEMU 进程，使锁生命周期覆盖整个虚拟机运行期。
"""

import sys

# 禁用 .pyc 字节码缓存生成，保持 tools 目录干净。
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import os
from pathlib import Path
import shlex
import shutil
import stat

from kernel_elf import read_kernel_elf
from qemu_command import build_qemu_command
from working_disks import (default_working_disk_directory, prepare_working_disks,
                           working_disk_paths)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CPU_COUNT = 1
DEFAULT_MEMORY_SIZE = "1G"
DEFAULT_NETWORK_MODE = "user"
DEFAULT_GDB_PORT = 1234


def repository_path(value):
    """将命令行传入的相对路径按仓库根目录规范化为绝对路径（纯词法拼接，不解引用符号链接或检查文件是否存在）。"""
    return Path(os.path.abspath(REPOSITORY_ROOT / value))


def parse_arguments(argv, parser=None):
    """定义并解析 QEMU 启动参数，支持外部调用者传入已有 `ArgumentParser` 扩展自定义选项。

    参数：
    - `argv`：待解析的参数列表；为 `None` 时读取 `sys.argv[1:]`。
    - `parser`：可选的外部 `ArgumentParser` 实例（如 `run_user_selftests.py` 传入的解析器）。

    返回：`(parser, args)` 二元组。
    """
    if parser is None:
        parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--arch", required=True, choices=("riscv64", "loongarch64"))
    parser.add_argument("--kernel-elf", type=repository_path)
    parser.add_argument("--system-image", type=repository_path)
    parser.add_argument("--contest-image", type=repository_path)
    parser.add_argument("--working-disk-dir", type=repository_path)
    parser.add_argument("--working-disk-action", choices=("fresh", "reuse", "refresh-system"),
                        default="fresh")
    session = parser.add_mutually_exclusive_group()
    session.add_argument("--shell", action="store_true")
    session.add_argument("--init-program")
    parser.add_argument("--system-only", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--gdb-port", type=int)
    parser.add_argument("--cpus", type=int, default=DEFAULT_CPU_COUNT)
    parser.add_argument("--memory", default=DEFAULT_MEMORY_SIZE)
    parser.add_argument("--network", choices=("user", "none"), default=DEFAULT_NETWORK_MODE)
    parser.add_argument("--host-forward")
    parser.add_argument("--root-mode", choices=("ro", "rw"))
    parser.add_argument("--qemu-binary", type=repository_path)
    parser.add_argument("--print-command", action="store_true")
    args = parser.parse_args(argv)

    # 校验选项之间的依赖关系与取值范围。
    if args.system_only and not (args.shell or args.init_program is not None):
        parser.error("--system-only requires --shell or --init-program")
    if args.host_forward is not None and args.network == "none":
        parser.error("--host-forward requires --network user")
    if args.gdb_port is not None:
        if not args.debug:
            parser.error("--gdb-port requires --debug")
        if not 1 <= args.gdb_port <= 65535:
            parser.error("--gdb-port must be in 1..65535")
    if args.cpus < 1:
        parser.error("--cpus must be a positive integer")
    if args.init_program is not None and (not args.init_program or any(
            character in " \t\n\r\v\f\0" for character in args.init_program)):
        parser.error("--init-program must be nonempty and contain no ASCII whitespace or NUL")
    return parser, args


def boot_arguments(system_only, root_mode, shell, init_program):
    """根据命令行覆盖项拼装内核启动参数字符串（`-append`）。

    默认双盘评测模式不传入任何额外启动参数（返回 `None`）；仅当显式指定单系统盘模式、
    根挂载读写模式、交互式 shell 或自定义初始进程时才生成对应的内核命令行键值。
    """
    arguments = []
    if system_only:
        arguments.append("root=/dev/vda")
    if root_mode is not None:
        arguments.append(root_mode)
    if shell:
        arguments.append("blossom.boot=shell")
    if init_program is not None:
        arguments.append(f"init={init_program}")
    return " ".join(arguments) if arguments else None


def print_summary(kernel_elf_path, system_image_path, contest_image_path,
                  disk_paths, working_disk_directory, working_disk_action,
                  debug_enabled, gdb_port):
    """向标准错误输出打印本次 QEMU 运行的内核路径、工作盘目录、源镜像是否被读取复制以及 GDB 监听地址。"""
    print(f"run-qemu kernel-elf={kernel_elf_path}", file=sys.stderr, flush=True)
    print(f"run-qemu working-disk-dir={working_disk_directory} "
          f"working-disk-action={working_disk_action}", file=sys.stderr, flush=True)
    print(f"run-qemu role=system source={system_image_path} "
          f"source-consumed={str(working_disk_action != 'reuse').lower()} "
          f"working-disk={disk_paths['system']}", file=sys.stderr, flush=True)
    if contest_image_path is not None:
        print(f"run-qemu role=contest source={contest_image_path} "
              f"source-consumed={str(working_disk_action == 'fresh').lower()} "
              f"working-disk={disk_paths['contest']}", file=sys.stderr, flush=True)
    if debug_enabled:
        print(f"run-qemu gdb-endpoint=127.0.0.1:{gdb_port}", file=sys.stderr, flush=True)


@dataclass(frozen=True)
class QemuConfiguration:
    """不可变的 QEMU 启动配置，保存解析后的路径与完整命令行元组（尚未接触文件系统或启动进程）。"""

    architecture: str
    kernel_elf_path: Path
    system_image_path: Path
    contest_image_path: Path | None
    working_disk_directory: Path
    working_disk_action: str
    debug_enabled: bool
    gdb_port: int
    command: tuple


def configure_qemu(args):
    """根据已解析的命令行参数补齐架构默认路径，并构造 `QemuConfiguration` 配置对象。"""
    suffix = "rv" if args.arch == "riscv64" else "la"
    kernel_elf_path = args.kernel_elf or REPOSITORY_ROOT / f"kernel-{suffix}"
    system_image_path = args.system_image or REPOSITORY_ROOT / ("disk.img" if suffix == "rv" else "disk-la.img")
    contest_image_path = None if args.system_only else (
        args.contest_image or REPOSITORY_ROOT / f"sdcard-{suffix}.img")
    directory = args.working_disk_dir or default_working_disk_directory(args.arch)
    disk_paths = working_disk_paths(directory, contest_image_path is not None)
    qemu_binary = str(args.qemu_binary) if args.qemu_binary is not None else f"qemu-system-{args.arch}"
    gdb_port = DEFAULT_GDB_PORT if args.gdb_port is None else args.gdb_port
    command = build_qemu_command(
        architecture=args.arch, qemu_binary=qemu_binary, kernel_elf_path=kernel_elf_path,
        system_working_disk_path=disk_paths["system"],
        contest_working_disk_path=disk_paths.get("contest"), cpu_count=args.cpus,
        memory_size=args.memory, network_mode=args.network, host_forward_rule=args.host_forward,
        boot_arguments=boot_arguments(args.system_only, args.root_mode, args.shell, args.init_program),
        debug_enabled=args.debug, gdb_port=gdb_port)
    return QemuConfiguration(
        args.arch, kernel_elf_path, system_image_path, contest_image_path, directory,
        args.working_disk_action, args.debug, gdb_port, tuple(command))


def summarize_configuration(configuration, disk_paths):
    """根据 `QemuConfiguration` 与工作盘路径字典输出启动配置摘要。"""
    print_summary(
        configuration.kernel_elf_path, configuration.system_image_path,
        configuration.contest_image_path, disk_paths, configuration.working_disk_directory,
        configuration.working_disk_action, configuration.debug_enabled, configuration.gdb_port)


@contextmanager
def prepare_qemu(configuration):
    """校验内核 ELF 与 QEMU 可执行文件，获取工作盘排他锁并按策略准备工作盘镜像。

    作为上下文管理器使用时，工作盘 `flock` 锁在整个 `with` 块期间保持持有，
    确保外部调用者（如自测运行器）在 QEMU 退出并读取完工作盘结果后才释放锁。

    生成：`(command, environment, disk_paths, lock_file)` 四元组。
    异常：当内核 ELF 非普通文件、架构不匹配、找不到 QEMU 二进制或工作盘准备失败时抛出 `ValueError` / `OSError`。
    """
    # 校验内核 ELF 为普通文件且架构头部合法。
    kernel = configuration.kernel_elf_path
    if not stat.S_ISREG(kernel.stat().st_mode):
        raise ValueError(f"kernel ELF is not a regular file: {kernel}")
    read_kernel_elf(kernel, configuration.architecture)

    # 解析 QEMU 可执行文件绝对路径，并清理可能干扰宿主机动态链接的环境变量。
    command = list(configuration.command)
    executable = shutil.which(command[0])
    if executable is None:
        raise ValueError(f"required QEMU executable missing: {command[0]}")
    command[0] = os.path.abspath(executable)
    environment = os.environ.copy()
    environment.pop("LD_LIBRARY_PATH", None)
    environment.pop("LD_PRELOAD", None)

    # 获取工作盘排他锁并准备镜像副本，打印摘要后交由调用者启动 QEMU。
    with prepare_working_disks(
            configuration.working_disk_directory, configuration.system_image_path,
            configuration.contest_image_path, configuration.working_disk_action
    ) as (disk_paths, lock_file):
        summarize_configuration(configuration, disk_paths)
        print(f"run-qemu qemu-binary={command[0]}", file=sys.stderr, flush=True)
        yield command, environment, disk_paths, lock_file


def main(argv=None):
    """脚本主入口：支持 `--print-command` 纯展示模式，或准备环境后通过 `execve` 启动 QEMU。"""
    parser, args = parse_arguments(argv)
    configuration = configure_qemu(args)
    if args.print_command:
        disk_paths = working_disk_paths(configuration.working_disk_directory,
                                        configuration.contest_image_path is not None)
        summarize_configuration(configuration, disk_paths)
        print(shlex.join(configuration.command))
        return
    try:
        with prepare_qemu(configuration) as (command, environment, _, lock_file):
            # 使排他锁文件描述符在 execve 后继续保持打开，直到 QEMU 进程退出才释放锁。
            os.set_inheritable(lock_file.fileno(), True)
            os.execve(command[0], command, environment)
    except (ValueError, OSError) as error:
        parser.exit(1, f"run-qemu: {error}\n")


if __name__ == "__main__":
    main()
