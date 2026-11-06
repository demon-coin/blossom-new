#!/usr/bin/env python3
"""根文件系统目录树组装脚本。

本脚本位于用户态构建与文件系统镜像制作之间，负责将预先编译安装好的用户态程序、
C 运行库（musl 与 glibc）以及仓库内的静态系统配置合并为单个架构（`riscv64` 或 `loongarch64`）
的完整根文件系统目录树，供后续 `create_ext4_image.py` 打包为 ext4 镜像。

输入：
- 目标架构名称、已安装的客户机用户态目录、运行库目录，以及可选的自定义 `inittab` 配置文件；
- 仓库内置的 `userland/system`（系统脚本与 `/etc` 配置）和 `userland/contest`（比赛运行配置）。

输出：
- 组装完成的根文件系统目录树。组装过程先在同级临时目录中构建完整目录并设置好权限，
  最后通过目录重命名替换旧输出目录，确保所有输入源保持只读且失败时不会留下半成品目录。
"""

import argparse
import os
from pathlib import Path
import shutil
import sys
import tempfile
from rootfs_paths import output_path, reject_overlap


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def copy_rootfs_entries(source_directory, destination_directory):
    """将源目录树递归合并到目标目录树中，并安全处理符号链接与文件类型替换。

    复制时若目标位置已存在符号链接，会先删除该链接再创建目录或写入文件，防止沿着符号链接
    意外修改只读安装源中的文件。普通文件使用 `copy2` 保留元数据，符号链接按原样重建链接目标。

    参数：
    - `source_directory`: 只读源目录 `Path`。
    - `destination_directory`: 待写入的目标目录 `Path`。
    """
    # 若目标目录本身是符号链接或普通文件，先将其移除，避免 mkdir 失败或沿链接写入源目录
    if destination_directory.is_symlink() or destination_directory.is_file():
        destination_directory.unlink()
    destination_directory.mkdir(parents=True, exist_ok=True)
    for source in source_directory.iterdir():
        destination = destination_directory / source.name
        if source.is_dir() and not source.is_symlink():
            copy_rootfs_entries(source, destination)
        else:
            # 源条目为普通文件或符号链接时，先清理目标位置已有的同名目录或文件，再按源类型复制或重建链接
            if destination.is_dir() and not destination.is_symlink():
                shutil.rmtree(destination)
            else:
                destination.unlink(missing_ok=True)
            if source.is_symlink():
                destination.symlink_to(os.readlink(source))
            else:
                shutil.copy2(source, destination)
    shutil.copystat(source_directory, destination_directory)


def assemble_rootfs_tree(architecture, guest_userland_directory,
                         runtime_libraries_directory, output_directory,
                         init_config_path=None):
    """校验安装输入并在临时目录中组装完整的单架构根文件系统目录树，完成后替换到输出目录。

    参数：
    - `architecture`: 目标架构，支持 `"riscv64"` 或 `"loongarch64"`。
    - `guest_userland_directory`: 已安装的客户机用户态程序根目录。
    - `runtime_libraries_directory`: 已准备好的动态运行库目录（包含 `musl` 和 `glibc` 子目录）。
    - `output_directory`: 输出根文件系统目录路径。
    - `init_config_path`: 可选的自定义 `inittab` 文件路径；若指定则覆盖默认的 `/etc/inittab`。

    返回：
    - 规范化后的输出目录 `Path`。

    异常：
    - `ValueError`: 架构不支持、输入源缺失关键程序或运行库、输入输出路径重叠，或运行时目录被符号链接篡改。
    - `OSError`: 文件复制、权限修改或输出目录替换失败。
    """
    if architecture not in ("riscv64", "loongarch64"):
        raise ValueError(f"unsupported architecture: {architecture}")
    userland = Path(guest_userland_directory).resolve(strict=True)
    runtime = Path(runtime_libraries_directory).resolve(strict=True)
    system = REPOSITORY_ROOT / "userland/system"
    contest = REPOSITORY_ROOT / "userland/contest"
    output = output_path(output_directory, directory=True)
    init_config = Path(init_config_path).resolve(strict=True) if init_config_path is not None else None
    # 校验所有输入目录与可选配置文件存在且类型正确，并确保输出目录不与任何输入源重叠或互为硬链接
    sources = (userland, runtime, system.resolve(), contest.resolve())
    for source in sources:
        if not source.is_dir():
            raise ValueError(f"required source is not a directory: {source}")
        reject_overlap(source, output)
    if init_config is not None:
        if not init_config.is_file():
            raise ValueError(f"init config is not a regular file: {init_config}")
        reject_overlap(init_config, output)
    # 检查系统启动、Shell 交互与比赛执行所必需的基础用户态程序和 C 运行库是否齐全
    for name in ("bin/busybox", "bin/sh", "sbin/init", "sbin/contest-run", "bin/libc-smoke"):
        if not (userland / name).is_file():
            raise ValueError(f"required installed guest program missing: {userland / name}")
    for name in ("musl/lib/libc.so", "glibc/lib/libc.so.6", "glibc/lib/libm.so.6"):
        if not (runtime / name).is_file():
            raise ValueError(f"required runtime library missing: {runtime / name}")
    output.parent.mkdir(parents=True, exist_ok=True)
    # 在输出目录所在的父目录下创建临时工作目录，确保最终可以通过同文件系统的 rename 原子切换输出目录
    with tempfile.TemporaryDirectory(prefix="rootfs-", dir=output.parent) as temporary:
        tree = Path(temporary) / "root"
        # 依次叠加客户机程序、系统静态配置、musl/glibc 运行库以及比赛脚本配置
        copy_rootfs_entries(userland, tree)
        copy_rootfs_entries(system / "etc", tree / "etc")
        copy_rootfs_entries(system / "sbin", tree / "sbin")
        for family in ("musl", "glibc"):
            if (tree / "opt").is_symlink() or (tree / "opt/runtime").is_symlink():
                raise ValueError("installed runtime parent must be a directory, not a symbolic link")
            copy_rootfs_entries(runtime / family, tree / "opt/runtime" / family)
        copy_rootfs_entries(contest, tree / "etc/contest")
        # 若调用方提供了专用的 init 配置（例如自动化测试负载），用其覆盖默认的 /etc/inittab
        if init_config is not None:
            (tree / "etc/inittab").unlink(missing_ok=True)
            shutil.copy2(init_config, tree / "etc/inittab")
        # 补齐内核挂载点与系统运行所需的标准空目录，并设置根目录及临时目录的粘滞位权限（01777）
        for name in ("dev", "proc", "run", "tmp", "var", "var/tmp", "root", "home",
                     "usr", "usr/bin", "usr/sbin", "mnt", "mnt/extra", "mnt/contest"):
            destination = tree / name
            if destination.is_symlink():
                raise ValueError(f"root filesystem directory must not be a symbolic link: {name}")
            destination.mkdir(parents=True, exist_ok=True)
        tree.chmod(0o755)
        (tree / "tmp").chmod(0o1777)
        (tree / "var/tmp").chmod(0o1777)
        # 先将旧输出目录移入临时目录暂存，再将新目录树重命名就位；若切换失败则恢复旧目录
        previous = Path(temporary) / "previous"
        if output.exists():
            output.rename(previous)
        try:
            tree.rename(output)
        except OSError:
            if previous.exists():
                previous.rename(output)
            raise
    print(f"Root filesystem assembled: arch={architecture} output={output}")
    return output


def main(argv=None):
    """解析命令行参数并组装根文件系统目录树；仅消费现成的安装目录，不触发编译。

    参数：
    - `argv`: 可选的命令行参数列表；默认读取 `sys.argv[1:]`。

    返回：
    - 进程退出码：组装成功返回 `0`，参数校验或文件系统操作失败返回 `1`。
    """
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--arch", choices=("riscv64", "loongarch64"), required=True)
    parser.add_argument("--guest-userland-directory", type=Path, required=True)
    parser.add_argument("--runtime-libraries-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--init-config", type=Path)
    args = parser.parse_args(argv)
    try:
        assemble_rootfs_tree(args.arch, args.guest_userland_directory,
                             args.runtime_libraries_directory, args.output_directory,
                             args.init_config)
    except (OSError, ValueError) as error:
        print(f"Root filesystem assembly failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
