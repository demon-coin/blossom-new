#!/usr/bin/env python3
"""管理生产模式 QEMU 启动所需的系统盘与比赛盘工作副本，提供基于文件锁的准备与清理能力。"""

import argparse
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import shutil
import stat
import tempfile

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def default_working_disk_directory(architecture):
    """返回指定架构默认的 QEMU 工作盘目录路径（`build/qemu-<arch>`），供启动与清理流程共用。"""
    return REPOSITORY_ROOT / "build" / f"qemu-{architecture}"


def working_disk_paths(working_disk_directory, include_contest):
    """根据工作盘目录拼接系统盘（`root.raw`）及可选比赛盘（`contest.raw`）路径，不访问文件系统。"""
    directory = Path(working_disk_directory)
    paths = {"system": directory / "root.raw"}
    if include_contest:
        paths["contest"] = directory / "contest.raw"
    return paths


@contextmanager
def _working_disk_lock(directory, create):
    """获取工作盘目录的非阻塞独占文件锁（`.lock`）；当 `create=False` 且目录不存在时产出 `None`。"""
    if create:
        directory.mkdir(parents=True, exist_ok=True)
    try:
        mode = directory.lstat().st_mode
    except FileNotFoundError:
        if create:
            raise
        yield None
        return
    if not stat.S_ISDIR(mode):
        raise ValueError(f"working disk directory is not an ordinary directory: {directory}")
    descriptor = os.open(directory / ".lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    with os.fdopen(descriptor, "r+b") as lock_file:
        status = os.fstat(lock_file.fileno())
        if not stat.S_ISREG(status.st_mode) or status.st_nlink != 1:
            raise ValueError(f"working disk lock is not an independent ordinary file: {directory / '.lock'}")
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield lock_file


def _check_working_disk(path, required):
    """校验工作盘是否为无硬链接别名的独立普通文件；仅当 `required=False` 时允许文件不存在。"""
    try:
        status = path.lstat()
    except FileNotFoundError:
        if required:
            raise ValueError(f"working disk missing: {path}") from None
        return False
    if not stat.S_ISREG(status.st_mode):
        raise ValueError(f"working disk is not an ordinary file: {path}")
    if status.st_nlink != 1:
        raise ValueError(f"working disk aliases another file: {path}")
    return True


@contextmanager
def prepare_working_disks(working_disk_directory, system_image_path,
                          contest_image_path, working_disk_action):
    """在持有目录独占锁期间按动作（`fresh`/`reuse`/`refresh-system`）准备磁盘工作副本；调用方应在整个 QEMU 运行期保持该上下文。"""
    if working_disk_action not in ("fresh", "reuse", "refresh-system"):
        raise ValueError(f"unsupported working disk action: {working_disk_action}")
    directory = Path(working_disk_directory)
    paths = working_disk_paths(directory, contest_image_path is not None)
    sources = {"system": Path(system_image_path)}
    if contest_image_path is not None:
        sources["contest"] = Path(contest_image_path)
    with _working_disk_lock(directory, create=True) as lock_file:
        # 校验目录中已有工作盘的状态；除 fresh 全新复制模式外，目标工作盘必须已存在。
        existing = {role: _check_working_disk(path, working_disk_action != "fresh")
                    for role, path in paths.items()}
        # 先从词法路径层面拒绝将工作盘自身作为源镜像；reuse 模式下不访问源镜像文件。
        for source in sources.values():
            if any(os.path.abspath(source) == os.path.abspath(path) for path in paths.values()):
                raise ValueError(f"source image names a working disk: {source}")
        # 按动作筛选需要复制的磁盘角色，并确认源镜像为普通文件且未与现有工作盘指向同一 inode。
        copied_roles = (list(paths) if working_disk_action == "fresh" else
                        ["system"] if working_disk_action == "refresh-system" else [])
        for role in copied_roles:
            source = sources[role]
            if not stat.S_ISREG(source.stat().st_mode):
                raise ValueError(f"source image is not a regular file: {source}")
            if any(existing[name] and path.samefile(source) for name, path in paths.items()):
                raise ValueError(f"source image aliases a working disk: {source}")
        if copied_roles:
            # 先将所有待更新镜像完整写入同目录的临时子目录，全部复制完成后再逐个原子替换目标文件。
            with tempfile.TemporaryDirectory(prefix="fresh-", dir=directory) as temporary:
                prepared = []
                for role in copied_roles:
                    copy = Path(temporary) / paths[role].name
                    with sources[role].open("rb") as source, copy.open("xb") as target:
                        shutil.copyfileobj(source, target, 1024 * 1024)
                    prepared.append((copy, paths[role]))
                for copy, target in prepared:
                    os.replace(copy, target)
        yield paths, lock_file


def clean_working_disks(working_disk_directory):
    """在持有非阻塞独占锁的前提下删除 `root.raw` 和 `contest.raw` 工作副本，保留目录、锁文件与其他文件。"""
    directory = Path(working_disk_directory)
    with _working_disk_lock(directory, create=False) as lock_file:
        if lock_file is None:
            return
        targets = [path for path in working_disk_paths(directory, True).values()
                   if _check_working_disk(path, required=False)]
        for target in targets:
            target.unlink()


def main(argv=None):
    """命令行入口：解析 `clean` 子命令参数，规范化工作盘目录路径并清理工作盘副本。"""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    commands = parser.add_subparsers(dest="command", required=True)
    clean = commands.add_parser("clean", allow_abbrev=False)
    clean.add_argument("--arch", choices=("riscv64", "loongarch64"), required=True)
    clean.add_argument("--working-disk-dir", type=Path)
    args = parser.parse_args(argv)
    directory = args.working_disk_dir
    if directory is None:
        directory = default_working_disk_directory(args.arch)
    directory = Path(os.path.abspath(REPOSITORY_ROOT / directory))
    try:
        clean_working_disks(directory)
    except (ValueError, OSError) as error:
        parser.exit(1, f"working-disks: {error}\n")


if __name__ == "__main__":
    main()
