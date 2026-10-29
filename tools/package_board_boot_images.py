#!/usr/bin/env python3
"""从实际 ELF 生成离线板启动镜像；不连接板卡、不修改固件或物理介质。"""

import argparse
import os
from pathlib import Path
import re
import shutil
import stat
import struct
import subprocess
import tempfile
import zlib

from kernel_elf import read_kernel_elf

LEGACY_HEADER = struct.Struct(">7I4B32s")
U32_LIMIT = 1 << 32
COPY_CHUNK_BYTES = 1024 * 1024


def require_regular_file(path):
    """包装输入必须是普通文件，不读取设备、目录或管道。"""
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError(f"not a regular file: {path}")


def align_staging_address(address):
    """暂存对象按页分离；此布局不声明目标板的可用 RAM 容量。"""
    return (address + 4095) & ~4095


def kernel_physical_entry(layout):
    """通过实际可执行 LOAD 的地址映射取得固件使用的物理入口。"""
    entry = layout["entry"]
    matches = [load for load in layout["loads"] if load["flags"] == 5
               and load["address"] <= entry < load["address"] + load["file_size"]]
    if len(matches) != 1:
        raise ValueError("kernel entry lacks a unique executable LOAD")
    load = matches[0]
    return load["physical"] + entry - load["address"]


def validate_board_kernel_layout(layout, architecture, target_platform):
    """板平台身份和映射只消费链接符号，普通生产节不依赖测试注册或展开信息。"""
    symbols = layout["symbols"]
    wanted = {"__blossom_platform", "__kernel_phys_start", "__kernel_virt_start",
              "__kernel_phys_end", "__kernel_virt_end", "__direct_base", "__phys_ram_base"}
    if not wanted <= symbols.keys():
        raise ValueError("kernel ELF lacks public board layout symbols")
    if symbols["__blossom_platform"] != (1 if target_platform == "visionfive2" else 2):
        raise ValueError("kernel ELF platform identity mismatch")
    physical_base = symbols["__kernel_phys_start"]
    virtual_base = symbols["__kernel_virt_start"]
    if not (symbols["__phys_ram_base"] <= physical_base < symbols["__kernel_phys_end"]
            and virtual_base < symbols["__kernel_virt_end"]):
        raise ValueError("kernel ELF layout bounds invalid")
    if architecture == "loongarch64" and virtual_base - physical_base != 0x9000000000000000:
        raise ValueError("kernel ELF cached direct-window mapping mismatch")
    expected_entry = physical_base if architecture == "riscv64" else virtual_base
    if layout["entry"] != expected_entry:
        raise ValueError("kernel ELF entry mismatch")
    loads = layout["loads"]
    for load in loads:
        if load["flags"] not in (4, 5, 6):
            raise ValueError("kernel ELF load permissions invalid")
        address = load["address"]
        physical = (address if architecture == "riscv64" and address == expected_entry else
                    address - virtual_base + physical_base)
        if load["physical"] != physical:
            raise ValueError("kernel ELF virtual/physical load mapping mismatch")
    if min(load["physical"] for load in loads) != physical_base:
        raise ValueError("kernel ELF physical load base mismatch")
    if symbols["__kernel_phys_end"] < max(load["physical"] + load["memory_size"] for load in loads):
        raise ValueError("kernel ELF loads exceed linked memory end")
    permissions = {".text": 5, ".rodata": 4, ".data": 6, ".bss": 6,
                   ".boot": 5 if architecture == "riscv64" else 6}
    for name, permission in permissions.items():
        section = layout["sections"].get(name)
        if not section or not section["size"] or not section["section_flags"] & 2:
            raise ValueError("kernel ELF required load section is missing, empty or not allocated")
        if section["flags"] != permission:
            raise ValueError("kernel ELF section permissions mismatch")
        address, length = section["address"], section["size"]
        matching = [load for load in loads if load["flags"] == permission
                    and load["address"] <= address
                    and address + length <= load["address"] + load["memory_size"]]
        if len(matching) != 1:
            raise ValueError("kernel ELF section lacks its permitted load segment")
        if section["type"] != 8:
            load = matching[0]
            if not (section["offset"] == load["offset"] + address - load["address"]
                    and section["offset"] + length <= load["offset"] + load["file_size"]):
                raise ValueError("kernel ELF section file mapping mismatch")


def write_raw_kernel_image(kernel_elf_path, raw_kernel_path, layout):
    """仅复制已验证 LOAD 的文件字节，按物理间隔补零，不把调试节放入 raw。"""
    loads = layout["loads"]
    start = min(load["physical"] for load in loads)
    end = max(load["physical"] + load["file_size"] for load in loads)
    if not 0 < end - start < U32_LIMIT:
        raise ValueError("kernel raw span exceeds legacy size limit")
    with kernel_elf_path.open("rb") as source, raw_kernel_path.open("xb") as target:
        for load in loads:
            source.seek(load["offset"])
            target.seek(load["physical"] - start)
            remaining = load["file_size"]
            while remaining:
                data = source.read(min(COPY_CHUNK_BYTES, remaining))
                if not data:
                    raise ValueError("kernel LOAD became truncated")
                target.write(data)
                remaining -= len(data)
        target.truncate(end - start)


def make_legacy_header(size, checksum, architecture_id, image_type, load, entry, epoch):
    """生成无压缩 legacy 头；所有长度、地址和时间戳保留真实的 32 位边界错误。"""
    if not all(0 <= value < U32_LIMIT for value in (size, checksum, load, entry, epoch)):
        raise ValueError("legacy image size/address/timestamp exceeds 32 bits")
    name = b"blossom-kernel" if image_type == 2 else b"blossom-root"
    fields = (0x27051956, 0, epoch, size, load, entry, checksum, 5, architecture_id, image_type, 0,
              name.ljust(32, b"\0"))
    header = LEGACY_HEADER.pack(*fields)
    return LEGACY_HEADER.pack(fields[0], zlib.crc32(header), *fields[2:])


def write_legacy_image(source_path, output_path, architecture_id, image_type, load, entry, epoch):
    """流式复制实际 payload 并计算 CRC，避免将整个 ext4 卷同时放入宿主内存。"""
    size = source_path.stat().st_size
    if not 0 < size < U32_LIMIT:
        raise ValueError("legacy image payload is empty or exceeds 32 bits")
    checksum = 0
    copied = 0
    with source_path.open("rb") as payload, output_path.open("xb") as image:
        image.write(bytes(LEGACY_HEADER.size))
        while data := payload.read(COPY_CHUNK_BYTES):
            checksum = zlib.crc32(data, checksum)
            image.write(data)
            copied += len(data)
        if copied != size:
            raise ValueError("legacy image input size changed while reading")
        image.seek(0)
        image.write(make_legacy_header(size, checksum, architecture_id, image_type, load, entry, epoch))
    return size + LEGACY_HEADER.size


def validate_root_ext4_image(root_ext4_image_path):
    """RAM 根只包装未压缩 raw ext4；本项目的普通卷不带 journal 或待恢复标记。"""
    require_regular_file(root_ext4_image_path)
    with root_ext4_image_path.open("rb") as source:
        source.seek(1024)
        superblock = source.read(1024)
    if not (len(superblock) == 1024 and superblock[56:58] == b"\x53\xef"):
        raise ValueError("root ext4 image must be an uncompressed raw ext4 volume")
    compatible, incompatible = struct.unpack_from("<II", superblock, 92)
    if compatible & 4 or incompatible & 4:
        raise ValueError("root ext4 image journal/recovery features are not supported")


def board_runtime_address(architecture, physical):
    """LoongArch 固件命令通过 cached DMW 访问镜像，容器头和范围记录仍使用物理地址。"""
    return physical | 0x9000000000000000 if architecture == "loongarch64" else physical


def write_os_dtb(source_path, target_path, initrd_range):
    """只修改独立 OS 树副本的 RAM 根范围，再为固件原地修改树保留 64KiB 空间。"""
    require_regular_file(source_path)
    with source_path.open("rb") as source:
        header = source.read(8)
    if not (len(header) == 8 and header[:4] == b"\xd0\x0d\xfe\xed"):
        raise ValueError("OS DTB header is invalid")
    total = struct.unpack_from(">I", header, 4)[0]
    if not 40 <= total <= source_path.stat().st_size:
        raise ValueError("OS DTB size is invalid")
    shutil.copyfile(source_path, target_path)
    if initrd_range is not None:
        for name, address in zip(("linux,initrd-start", "linux,initrd-end"), initrd_range):
            result = subprocess.run(
                ["fdtput", "-p", "-t", "x", str(target_path), "/chosen", name,
                 f"{address >> 32:x}", f"{address & (U32_LIMIT - 1):x}"],
                capture_output=True, text=True)
            if result.returncode != 0:
                raise ValueError(f"fdtput failed for {name}: {result.stderr.strip()}")
    size = target_path.stat().st_size + 64 * 1024
    if not size < U32_LIMIT:
        raise ValueError("padded OS DTB exceeds format bounds")
    with target_path.open("r+b") as target:
        target.truncate(size)
        target.seek(4)
        target.write(struct.pack(">I", size))
    return size


def write_boot_environment(architecture, target_platform, layout, raw_kernel_path, output_directory,
                           epoch, root_ext4_image_path, os_dtb_path, root_device, root_mode, init_program):
    """按物理范围安排独立暂存区，输出装载地址与内核参数，不生成固件执行命令。"""
    architecture_id = 26 if architecture == "riscv64" else 27
    load = min(segment["physical"] for segment in layout["loads"])
    memory_end = max(segment["physical"] + segment["memory_size"] for segment in layout["loads"])
    entry = kernel_physical_entry(layout)
    staging = align_staging_address(memory_end)
    kernel_size = write_legacy_image(raw_kernel_path, output_directory / "kernel.uImage",
                                     architecture_id, 2, load, entry, epoch)
    spans = [(load, memory_end, "kernel memory"),
             (staging, staging + kernel_size, "kernel container")]
    rows = ["# 仅为离线装载布局；装载前须核对实际 RAM、固件及保留区范围。",
            "# 本次生成未修改固件环境或物理介质。",
            "# kernel_elf 是内核输入，root_ext4_image 是可选的 RAM 根卷输入。",
            "# 单架构板卷使用 rootdir=/。",
            f"blossom_platform={target_platform}", "blossom_rootdir=/",
            "kernel_file=kernel.uImage",
            f"kernel_addr_r={board_runtime_address(architecture, staging):#x}",
            f"kernel_load={load:#x}", f"kernel_entry={entry:#x}",
            f"kernel_elf_entry={layout['entry']:#x}",
            f"kernel_raw_size={raw_kernel_path.stat().st_size:#x}", f"kernel_image_size={kernel_size:#x}",
            f"kernel_mem_end={memory_end:#x}"]
    cursor = align_staging_address(staging + kernel_size)
    initrd_range = None
    if root_ext4_image_path is not None:
        validate_root_ext4_image(root_ext4_image_path)
        payload = cursor + LEGACY_HEADER.size
        size = write_legacy_image(root_ext4_image_path, output_directory / "root.uInitrd",
                                  architecture_id, 3, payload, payload, epoch)
        spans.append((cursor, cursor + size, "ramdisk container"))
        initrd_range = (payload, cursor + size)
        rows += ["ramdisk_file=root.uInitrd",
                 f"ramdisk_addr_r={board_runtime_address(architecture, cursor):#x}",
                 f"ramdisk_image_size={size:#x}", f"initrd_start={payload:#x}",
                 f"initrd_end={cursor + size:#x}", "initrd_high=0xffffffffffffffff"]
        cursor = align_staging_address(cursor + size)
    if os_dtb_path is not None:
        size = write_os_dtb(os_dtb_path, output_directory / "board.dtb", initrd_range)
        spans.append((cursor, cursor + size, "OS DTB"))
        fdt_address = board_runtime_address(architecture, cursor)
        rows += ["fdt_file=board.dtb", f"fdt_addr={fdt_address:#x}", f"fdt_size={size:#x}",
                 "fdt_high=0xffffffffffffffff"]
    else:
        rows.append("# 未提供 os_dtb；启动前须准备实际传给内核的 OS DTB。")
    if not all(0 <= start < end <= U32_LIMIT for start, end, _ in spans):
        raise ValueError("board image ranges exceed legacy physical address bounds")
    ordered = sorted(spans)
    if not all(left[1] <= right[0] for left, right in zip(ordered, ordered[1:])):
        raise ValueError("board image memory ranges overlap")
    if root_device is not None:
        if not re.fullmatch(r"/dev/[A-Za-z0-9]+", root_device):
            raise ValueError("invalid board root device")
        if root_device == "/dev/ram0" and root_ext4_image_path is None:
            raise ValueError("RAM root requires --root-ext4-image")
        if not re.fullmatch(r"/[A-Za-z0-9_./-]+", init_program):
            raise ValueError("invalid board init path")
        rows.append(f"bootargs=root={root_device} rootdir=/ {root_mode} init={init_program}")
    else:
        rows.append("# 未选择 root_device；未生成 bootargs。")
    (output_directory / "boot.env").write_text("\n".join(rows) + "\n", encoding="utf-8")


def package_board_boot_images(architecture, target_platform, kernel_elf_path, output_directory,
                              root_ext4_image_path=None, os_dtb_path=None, root_device=None,
                              root_mode="ro", init_program="/sbin/init"):
    """固定本轮 ELF 副本，全部离线包装成功才发表对应文件，不接收 CLI Namespace。"""
    if (architecture, target_platform) not in (("riscv64", "visionfive2"),
                                               ("loongarch64", "ls2k1000la")):
        raise ValueError("board packaging requires a matching architecture/target platform")
    if root_mode not in ("ro", "rw"):
        raise ValueError("invalid board root mode")
    require_regular_file(kernel_elf_path)
    epoch = int(os.environ.get("SOURCE_DATE_EPOCH", "0"))
    if not 0 <= epoch < U32_LIMIT:
        raise ValueError("SOURCE_DATE_EPOCH exceeds legacy timestamp bounds")
    output_directory.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".board-", dir=output_directory) as temporary:
        directory = Path(temporary)
        fixed_kernel_elf_path = directory / "kernel.elf"
        shutil.copyfile(kernel_elf_path, fixed_kernel_elf_path)
        layout = read_kernel_elf(fixed_kernel_elf_path, architecture)
        validate_board_kernel_layout(layout, architecture, target_platform)
        raw_kernel_path = directory / "kernel.bin"
        write_raw_kernel_image(fixed_kernel_elf_path, raw_kernel_path, layout)
        write_boot_environment(architecture, target_platform, layout, raw_kernel_path, directory,
                               epoch, root_ext4_image_path, os_dtb_path, root_device, root_mode, init_program)
        names = ["kernel.elf", "kernel.bin", "kernel.uImage", "boot.env"]
        names += ["root.uInitrd"] if root_ext4_image_path is not None else []
        names += ["board.dtb"] if os_dtb_path is not None else []
        for name in names:
            os.replace(directory / name, output_directory / name)
        for name, supplied in (("root.uInitrd", root_ext4_image_path), ("board.dtb", os_dtb_path)):
            if supplied is None:
                (output_directory / name).unlink(missing_ok=True)


def main(argv=None):
    """只解析正式离线输入并生成文件，不调用下载器、串口、写盘或固件命令。"""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--arch", choices=("riscv64", "loongarch64"), required=True)
    parser.add_argument("--target-platform", choices=("visionfive2", "ls2k1000la"), required=True)
    parser.add_argument("--kernel-elf", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--root-ext4-image", type=Path, help="uncompressed raw ext4 input; never modified")
    parser.add_argument("--os-dtb", type=Path, help="actual OS DTB input; not a firmware control tree")
    parser.add_argument("--root-device", help="explicit board root device; single-architecture rootdir is /")
    parser.add_argument("--root-mode", choices=("ro", "rw"), default="ro")
    parser.add_argument("--init-program", default="/sbin/init")
    args = parser.parse_args(argv)
    try:
        package_board_boot_images(args.arch, args.target_platform, args.kernel_elf, args.output_directory,
                                  args.root_ext4_image, args.os_dtb, args.root_device, args.root_mode,
                                  args.init_program)
        print(f"Board boot images generated offline: {args.output_directory}")
    except (OSError, ValueError, struct.error) as error:
        parser.exit(1, f"package-board-boot-images: {error}\n")


if __name__ == "__main__":
    main()
