#!/usr/bin/env python3
"""构建内核 ext4 文件系统回归测试所需的只读、故障注入（损坏）与可写磁盘镜像夹具。

本脚本在宿主机临时目录中组装测试文件树，并调用 `mke2fs`、`debugfs`、`e2fsck`、`dumpe2fs` 等标准
e2fsprogs 工具生成三类测试镜像：
1. 只读基准镜像（`readonly`）：覆盖 `seed4k`、`nojournal4k`、`legacy1k` 三种几何配置，包含硬链接、
   快速/块级符号链接、非 UTF-8 原始字节文件名、多层深孔洞稀疏文件、二层 extent 树、间接块映射及
   HTree（dx）索引大目录。
2. 损坏与特性边界镜像（`*-bad-*` / `*-unknown-*`）：从只读正盘复制独立副本，定点翻转超级块、块组描述符、
   inode、extent 节点、目录块或 HTree 块的校验字节，或注入不兼容布局与未知兼容位，用于测试内核挂载与读取的防御能力。
3. 可写回归镜像（`writable`）：在源根文件系统的私有副本中安装 `user-write` 测试程序、空占槽文件、
   预分配 unwritten 区及接近满盘（`--nearly-full`）的保留空间，供内核与用户态写路径测试使用。
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from create_ext4_image import (
    PROFILES, check_debugfs_result, check_ext4_image, create_ext4_image,
    run_checked_command, run_debugfs_command,
)

PATTERN_SIZE = 65573
DIRECTORY_ENTRIES = 12000
DEEP_EXTENTS = 1500


def punch_holes(image, block_size):
    """使用 `debugfs` 的 `punch` 命令在 `/deep` 与 `/sparse` 文件中显式打出逻辑块孔洞。

    直接通过 `debugfs` 按文件系统块粒度打孔，避免受宿主机临时文件系统 `SEEK_HOLE` 最小粒度（通常为 4KiB）的影响。
    """
    with tempfile.NamedTemporaryFile(mode="w", encoding="ascii", prefix="holes-",
                                     suffix=".commands", dir=image.parent) as requests:
        for logical in range(1, (DEEP_EXTENTS - 1) * 2, 2):
            requests.write(f"punch /deep {logical} {logical}\n")
        requests.write(f"punch /sparse {4096 // block_size} {12288 // block_size - 1}\n")
        requests.flush()
        text = run_checked_command(["debugfs", "-w", "-f", requests.name, image], image.with_suffix(".log"))
        check_debugfs_result(text, "punch sparse logical blocks")


def pattern(offset, length):
    """根据文件逻辑字节偏移生成确定性的非零周期字节图样，供内核测试端按相同公式核对读取内容。"""
    return bytes((37 + (position // 512) * 17 + position % 512) & 255
                 for position in range(offset, offset + length))


def write_pattern(path, length):
    """以 64KiB 为分块向目标文件写入确定性字节图样，避免一次性在内存中分配整块大缓冲区。"""
    with path.open("xb") as stream:
        for offset in range(0, length, 65536):
            stream.write(pattern(offset, min(65536, length - offset)))


def create_readonly_fixture_tree(directory, block_size):
    """在临时目录中构造只读测试镜像所需的各类文件、链接、稀疏布局与目录结构。"""
    directory.mkdir()
    write_pattern(directory / "pattern", PATTERN_SIZE)
    os.link(directory / "pattern", directory / "hardlink")
    (directory / "empty").touch()
    (directory / "unwritten").touch()
    os.symlink("pattern", directory / "fastlink")
    os.symlink("t" * 180, directory / "blocklink")
    raw = os.fsencode(directory) + b"/name-\xff\xfe"
    with open(raw, "xb") as stream:
        stream.write(b"raw-name\n")
    walk = directory / "walk"
    walk.mkdir()
    (walk / "sub").mkdir()
    (directory / "mnt").mkdir()
    (directory / "mnt" / "extra").mkdir()
    for name, target in (
        ("relative", "../pattern"),
        ("absolute", "/pattern"),
        ("directory", "sub"),
        ("loop", "loop"),
    ):
        os.symlink(target, walk / name)
    for index in range(1, 42):
        target = f"chain{index + 1:02d}" if index < 41 else "../pattern"
        os.symlink(target, walk / f"chain{index:02d}")
    with (directory / "sparse").open("xb") as stream:
        stream.write(pattern(0, 4096))
        stream.seek(12288)
        stream.write(pattern(12288, 37))
    with (directory / "deep").open("xb") as stream:
        for index in range(DEEP_EXTENTS):
            offset = index * 2 * block_size
            stream.seek(offset)
            stream.write(pattern(offset, block_size))
    if block_size == 1024:
        with (directory / "indirect").open("xb") as stream:
            for logical in (0, 12, 268, 65804):
                offset = logical * block_size
                stream.seek(offset)
                stream.write(pattern(offset, block_size))
    plain = directory / "plain"
    plain.mkdir()
    for index in range(40):
        (plain / f"item-{index:04d}").touch()
    indexed = directory / "indexed"
    indexed.mkdir()
    for index in range(DIRECTORY_ENTRIES):
        (indexed / (f"item-{index:05d}-" + "n" * 180)).touch()


def optimize_fixture_directories(image, profile):
    """运行 `e2fsck -fyD` 构建 HTree（dx）目录索引并更新校验和，再用只读检查确认文件系统完全一致。"""
    run_checked_command(["e2fsck", "-fyD", image], image.with_suffix(".log"), accepted=(0, 1))
    run_checked_command(["e2fsck", "-fn", image], image.with_suffix(".log"))
    run_checked_command(["dumpe2fs", "-h", image], image.with_suffix(".log"))
    for name in ("pattern", "deep", "indexed", "fastlink", "blocklink", "unwritten"):
        run_debugfs_command(image, f"stat /{name}")
    run_debugfs_command(image, "htree_dump /indexed")
    if profile != "legacy1k":
        run_debugfs_command(image, "dump_extents /deep")


def create_readonly_fixture_image(output, profile, size_mib):
    """按指定几何配置（`profile`）构建单个只读 ext4 测试镜像，并校验二层 extent 树与 HTree 索引深度。"""
    block_size = PROFILES[profile][0]
    with tempfile.TemporaryDirectory(prefix="staging-", dir=output.parent) as temporary:
        source = Path(temporary) / "root"
        create_readonly_fixture_tree(source, block_size)
        create_ext4_image(source, output, profile, size_mib)
    punch_holes(output, block_size)
    if profile != "legacy1k":
        run_debugfs_command(output, f"fallocate /unwritten 0 {16384 // block_size - 1}", writable=True)
    run_debugfs_command(output, "set_inode_field /unwritten size 16384", writable=True)
    optimize_fixture_directories(output, profile)
    if profile != "legacy1k":
        stat = run_debugfs_command(output, "stat /deep")
        # 确认 debugfs 报告中出现 (ETB1)，即 /deep 文件已形成深度至少为 2 的多级 extent 树
        if "(ETB1)" not in stat:
            raise ValueError("debugfs did not produce a depth-two extent tree")
    tree = run_debugfs_command(output, "htree_dump /indexed")
    if not re.search(r"Indirect levels:\s*[1-9]", tree):
        raise ValueError("e2fsck did not produce a deeper indexed directory")


def duplicate(source, name):
    """将基准镜像复制到同目录下的 `<name>.img` 新文件，作为故障注入的独立副本。"""
    target = source.with_name(name + ".img")
    with source.open("rb") as input_file, target.open("xb") as output:
        shutil.copyfileobj(input_file, output, 1024 * 1024)
    return target


def flip(image, offset):
    """翻转镜像指定字节偏移处的最低位（异或 `0x01`），故意保持原有 CRC 不变以触发内核校验和失败。"""
    with image.open("r+b") as stream:
        stream.seek(offset)
        original = stream.read(1)
        if len(original) != 1:
            raise ValueError("corruption offset outside generated image")
        stream.seek(offset)
        stream.write(bytes([original[0] ^ 1]))
    with image.with_suffix(".log").open("ab") as stream:
        stream.write(f"corruption byte-offset={offset} old={original[0]:02x} new={original[0] ^ 1:02x} CRC-retained=1\n".encode())


def inode_offset(image, name, block_size):
    """通过 `debugfs imap` 查询指定路径的 inode 在镜像文件中的精确字节偏移。"""
    text = run_debugfs_command(image, f"imap /{name}")
    found = re.search(r"located at block\s+(\d+),\s+offset\s+(0x[0-9a-fA-F]+)", text)
    if found is None:
        raise ValueError("debugfs imap did not report inode position")
    return int(found[1]) * block_size + int(found[2], 16)


def physical_block(image, name, logical):
    """通过 `debugfs bmap` 查询文件指定逻辑块对应的物理块号（兼容带 `(uninit)` 标记的预分配块）。"""
    text = run_debugfs_command(image, f"bmap /{name} {logical}")
    numbers = re.findall(r"^([0-9]+)(?: \(uninit\))?$", text, re.MULTILINE)
    if len(numbers) != 1 or int(numbers[0]) == 0:
        raise ValueError("debugfs bmap did not report one mapped block")
    return int(numbers[0])


def overwrite(image, offset, data):
    """覆盖镜像指定字节偏移处的一段数据，并将修改前后的十六进制前缀追加记录到同名 `.log` 日志中。"""
    with image.open("r+b") as stream:
        stream.seek(offset)
        old = stream.read(len(data))
        if len(old) != len(data):
            raise ValueError("mutation range outside generated image")
        stream.seek(offset)
        stream.write(data)
    with image.with_suffix(".log").open("ab") as stream:
        stream.write(f"mutation byte-offset={offset} length={len(data)} old-prefix={old[:16].hex()} new-prefix={data[:16].hex()} CRC-retained=1\n".encode())


def corrupt_indirect_data(image):
    """将 `/pattern` 文件一级间接块（`IND`）中的首个数据块指针覆写为 `0xffffffff`，用于测试间接块越界错误处理。"""
    text = run_debugfs_command(image, "stat /pattern")
    found = re.search(r"\(IND\):([0-9]+)", text)
    if found is None:
        raise ValueError("debugfs did not report pattern single-indirect node")
    overwrite(image, int(found[1]) * 1024, (0xffffffff).to_bytes(4, "little"))


def create_damaged_fixture_images(directory):
    """以 `seed4k.img` 和 `legacy1k.img` 为基准，派生各类校验和损坏、特性位不兼容与结构畸变的测试镜像。"""
    source = directory / "seed4k.img"
    block_size = 4096
    flip(duplicate(source, "seed4k-bad-super"), 1024 + 1020)
    flip(duplicate(source, "seed4k-bad-group"), block_size + 30)
    inode = duplicate(source, "seed4k-bad-inode")
    flip(inode, inode_offset(inode, "pattern", block_size) + 124)
    extent = duplicate(source, "seed4k-bad-extent")
    tree = run_debugfs_command(extent, "stat /deep")
    node = re.search(r"\(ETB[01]\):([0-9]+)", tree)
    if node is None:
        raise ValueError("debugfs did not report an external extent block")
    flip(extent, int(node[1]) * block_size + block_size - 4)
    directory_image = duplicate(source, "seed4k-bad-dir")
    flip(directory_image, physical_block(directory_image, "plain", 0) * block_size + block_size - 4)
    dx = duplicate(source, "seed4k-bad-dx")
    flip(dx, physical_block(dx, "indexed", 0) * block_size + block_size - 4)
    layout = duplicate(source, "seed4k-bad-layout")
    run_debugfs_command(layout, "set_super_value inode_size 192", writable=True)
    feature = duplicate(source, "seed4k-bad-features")
    run_debugfs_command(feature, "feature encrypt", writable=True)
    for field in ("compat", "ro_compat"):
        unknown = duplicate(source, f"seed4k-unknown-{field}")
        run_checked_command(["dumpe2fs", "-h", unknown], unknown.with_suffix(".log"))
        # 在保留原有兼容/只读兼容特性位的基础上置入最高位未知标志，并由 debugfs 自动重算超级块 CRC
        offset = 92 if field == "compat" else 100
        with unknown.open("rb") as stream:
            stream.seek(1024 + offset)
            value = int.from_bytes(stream.read(4), "little")
        run_debugfs_command(unknown, f"set_super_value feature_{field} {value | 0x80000000}", writable=True)
    legacy = directory / "legacy1k.img"
    flip(duplicate(legacy, "legacy1k-bad-group"), 2 * 1024 + 30)
    malformed = duplicate(legacy, "legacy1k-bad-layout")
    corrupt_indirect_data(malformed)
    prefix = duplicate(legacy, "legacy1k-bad-prefix")
    corrupt_indirect_data(prefix)
    malformed_extent = duplicate(source, "seed4k-bad-extent-layout")
    run_debugfs_command(malformed_extent, "set_inode_field /pattern block[0] 0", writable=True)
    malformed_dir = duplicate(legacy, "legacy1k-bad-dir-layout")
    with malformed_dir.open("r+b") as stream:
        stream.seek(physical_block(malformed_dir, "plain", 0) * 1024 + 4)
        stream.write(b"\x00\x00")
    malformed_dx = duplicate(legacy, "legacy1k-bad-dx-layout")
    with malformed_dx.open("r+b") as stream:
        stream.seek(physical_block(malformed_dx, "indexed", 0) * 1024 + 34)
        stream.write(b"\x00\x00")
    for image in sorted(directory.glob("*-bad-*.img")):
        print(f"corrupt root={image} hardware-EIO-covered=0")


def copy_writable_fixture_tree(source, destination):
    """将源根文件系统目录树复制到私有临时目录，同时保持目录内的硬链接关系与符号链接不变。"""
    copied = {}

    def copy_file(source_path, destination_path):
        """复制单个普通文件；若同一源 `(st_dev, st_ino)` 已复制过，则在目标树中建立内部硬链接，避免与外部源目录共享 inode。"""
        status = os.stat(source_path)
        identity = (status.st_dev, status.st_ino)
        if identity in copied:
            os.link(copied[identity], destination_path)
        else:
            shutil.copy2(source_path, destination_path)
            copied[identity] = destination_path
        return destination_path

    shutil.copytree(source, destination, symlinks=True, copy_function=copy_file)


def install_writable_fixture_files(directory, user_write_program):
    """在私有副本的 `/test` 目录下安装用户态写测试程序（`user-write`）及各个写测试用例专用的预置文件。"""
    test = directory / "test"
    binary_directory = test / "bin"
    if test.is_symlink() or binary_directory.is_symlink():
        raise ValueError("writable fixture test directories must not be symbolic links")
    binary_directory.mkdir(parents=True, exist_ok=True)
    target = binary_directory / "user-write"
    if target.exists() or target.is_symlink():
        raise ValueError("write fixture requires a source without user-write")
    shutil.copy2(user_write_program, target)
    next_program = binary_directory / "user-write-next"
    if next_program.exists() or next_program.is_symlink():
        raise ValueError("write fixture requires a source without user-write-next")
    shutil.copy2(user_write_program, next_program)
    for name in ("write-kernel", "write-user", "write-append", "write-append-kernel",
                 "write-oom", "write-unwritten", "write-merge", "write-tree", "write-rollback",
                 "write-full", "write-reserve"):
        if (test / name).is_symlink():
            raise ValueError(f"writable fixture file must not be a symbolic link: {name}")
        with (test / name).open("xb"):
            pass
    if (test / "write-map").is_symlink():
        raise ValueError("writable fixture write-map must not be a symbolic link")
    with (test / "write-map").open("xb") as stream:
        stream.write(bytes(range(256)) * 32)
    (test / "write-map").chmod(0o755)


def create_writable_fixture_image(source_tree_dir, user_write_program, output_image_path,
                                  profile="seed4k", image_size_mib=256, nearly_full=False):
    """构建可写 ext4 测试镜像：复制源树、安装写测试文件、注入预分配块（及可选的近满盘占空比），通过一致性检查后原子发布。"""
    source = Path(source_tree_dir).resolve(strict=True)
    program = Path(user_write_program).resolve(strict=True)
    output = Path(os.path.abspath(output_image_path))
    log = output.with_suffix(".log")
    if not source.is_dir() or not program.is_file():
        raise ValueError("writable fixture requires an existing source tree and user-write program")
    if profile not in PROFILES or image_size_mib < 128:
        raise ValueError("writable fixture requires a known profile and at least 128 MiB")
    for target in (output, log):
        if (target == log == output or target.exists()
                or any(path.is_symlink() for path in (target, *target.parents))
                or target == source or source in target.resolve().parents
                or target == program):
            raise ValueError("writable fixture requires new output paths outside its sources")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="writable-fixture-", dir=output.parent) as temporary:
        directory = Path(temporary)
        tree = directory / "source"
        copy_writable_fixture_tree(source, tree)
        install_writable_fixture_files(tree, program)
        image = directory / "root.ext4"
        try:
            create_ext4_image(tree, image, profile, image_size_mib)
            block_size = PROFILES[profile][0]
            # 为非 legacy1k 镜像预分配 unwritten 范围，并在物理块中填入非零脏字节（0xa7），验证内核读取 unwritten 块时返回零而非底层脏数据
            preallocated = () if profile == "legacy1k" else (
                ("write-unwritten", 16384 // block_size), ("write-merge", 64),
            )
            for name, blocks in preallocated:
                run_debugfs_command(image, f"fallocate /test/{name} 0 {blocks - 1}", writable=True)
                for logical in range(blocks):
                    physical = physical_block(image, f"test/{name}", logical)
                    with image.open("r+b") as stream:
                        stream.seek(physical * block_size)
                        stream.write(b"\xa7" * block_size)
                run_debugfs_command(image, f"set_inode_field /test/{name} size {blocks * block_size}", writable=True)
            # 若请求构造接近满盘的镜像，则用 fallocate 占用大部分空闲块，仅留出少量空间供 ENOSPC 边界测试使用
            if nearly_full:
                with image.open("rb") as stream:
                    stream.seek(1024 + 12)
                    free = int.from_bytes(stream.read(4), "little")
                if free <= 256:
                    raise ValueError("ordinary ENOSPC fixture has insufficient initial free blocks")
                run_debugfs_command(image, f"fallocate /test/write-reserve 0 {free - 129}", writable=True)
                run_debugfs_command(image, f"set_inode_field /test/write-reserve size {(free - 128) * block_size}", writable=True)
            check_ext4_image(image, profile)
        finally:
            temporary_log = image.with_suffix(".log")
            if temporary_log.exists():
                os.replace(temporary_log, log)
        os.replace(image, output)
    print(f"Writable fixture created: output={output} profile={profile} nearly-full={int(nearly_full)}")
    return output


def main(argv=None):
    """命令行入口：支持 `readonly`（生成三种几何只读盘与全套损坏盘）和 `writable`（生成可写测试盘）两个子命令。"""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    actions = parser.add_subparsers(dest="action", required=True)
    readonly = actions.add_parser("readonly", allow_abbrev=False)
    readonly.add_argument("--output-directory", type=Path, required=True)
    readonly.add_argument("--size-mib", type=int, default=256)
    writable = actions.add_parser("writable", allow_abbrev=False)
    writable.add_argument("--source-tree", type=Path, required=True)
    writable.add_argument("--user-write-program", type=Path, required=True)
    writable.add_argument("--output-image", type=Path, required=True)
    writable.add_argument("--profile", choices=PROFILES, default="seed4k")
    writable.add_argument("--size-mib", type=int, default=256)
    writable.add_argument("--nearly-full", action="store_true")
    args = parser.parse_args(argv)
    if args.size_mib < 128:
        parser.error("--size-mib must be at least 128")
    try:
        if args.action == "readonly":
            output = Path(os.path.abspath(args.output_directory))
            if output.exists() or any(path.is_symlink() for path in (output, *output.parents)):
                raise ValueError("readonly fixtures require a new output directory without symbolic links")
            output.mkdir(parents=True)
            for profile in PROFILES:
                image = output / f"{profile}.img"
                create_readonly_fixture_image(image, profile, args.size_mib)
                print(f"Readonly fixture created: output={image} profile={profile}")
            create_damaged_fixture_images(output)
        else:
            create_writable_fixture_image(args.source_tree, args.user_write_program,
                                          args.output_image, args.profile, args.size_mib,
                                          args.nearly_full)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Ext4 fixture creation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
