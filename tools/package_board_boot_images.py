#!/usr/bin/env python3
"""开发板离线启动镜像与引导参数打包脚本。

本脚本将编译好的真实板卡内核 ELF（支持 RISC-V `visionfive2` 与 LoongArch `ls2k1000la`）
以及可选的原始 ext4 根镜像、操作系统设备树（DTB）打包为 U-Boot / PMON 兼容的离线启动产物。
本脚本仅在宿主机本地生成文件，不连接开发板串口、不烧写物理介质，也不修改固件环境变量。

输入：
- 目标架构与开发板型号、内核 ELF 文件路径；
- 可选的无压缩原始 ext4 根镜像（用于 RAM 根启动）、操作系统 DTB 文件、根块设备路径、挂载读写模式与 `init` 路径。

输出（在临时目录全部生成校验通过后原子发布至输出目录）：
- `kernel.elf`（本轮打包固定的内核副本）、`kernel.bin`（按物理地址展开的裸二进制映像）、`kernel.uImage`（U-Boot legacy 内核镜像）；
- 可选的 `root.uInitrd`（U-Boot legacy RAM 根镜像）与 `board.dtb`（写入 `initrd` 物理范围并预留 64KiB 扩展空间的设备树）；
- `boot.env`（互不重叠的离线物理/运行时暂存布局参数与可选 `bootargs`）。
"""

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

# U-Boot legacy image 64 字节大端头部格式：魔数、头 CRC、时间戳、数据长度、加载地址、入口地址、数据 CRC、OS/Arch/Type/Comp 及 32 字节名称
LEGACY_HEADER = struct.Struct(">7I4B32s")
U32_LIMIT = 1 << 32
COPY_CHUNK_BYTES = 1024 * 1024


def require_regular_file(path):
    """校验输入路径存在且为普通文件，拒绝目录、块设备或管道。

    参数：
    - `path`: 待检查的 `Path` 对象。

    异常：
    - `OSError`: 路径不存在或无法获取文件状态。
    - `ValueError`: 路径不是普通文件（`S_ISREG` 为假）。
    """
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError(f"not a regular file: {path}")


def align_staging_address(address):
    """将物理暂存地址向上按 4KiB 页边界对齐。

    各启动镜像（`uImage`、`uInitrd`、`dtb`）在内存中按页隔离存放；实际装载前仍需由操作者核对开发板可用 RAM 范围。

    参数：
    - `address`: 待对齐的字节物理地址。

    返回：
    - 向上对齐到 4096 字节整数倍后的物理地址。
    """
    return (address + 4095) & ~4095


def kernel_physical_entry(layout):
    """根据内核虚拟入口地址及其所在的可执行 `PT_LOAD` 段计算固件跳转所需的物理入口地址。

    参数：
    - `layout`: `read_kernel_elf` 返回的内核 ELF 结构字典。

    返回：
    - 对应的物理入口地址整数。

    异常：
    - `ValueError`: 未找到唯一覆盖入口地址的只读可执行（`flags == 5`）装载段。
    """
    entry = layout["entry"]
    matches = [load for load in layout["loads"] if load["flags"] == 5
               and load["address"] <= entry < load["address"] + load["file_size"]]
    if len(matches) != 1:
        raise ValueError("kernel entry lacks a unique executable LOAD")
    load = matches[0]
    return load["physical"] + entry - load["address"]


def validate_board_kernel_layout(layout, architecture, target_platform):
    """校验开发板内核 ELF 的链接器导出符号、虚实地址映射关系及核心节权限。

    参数：
    - `layout`: `read_kernel_elf` 返回的内核 ELF 结构字典。
    - `architecture`: 目标架构（`"riscv64"` 或 `"loongarch64"`）。
    - `target_platform`: 目标开发板平台（`"visionfive2"` 或 `"ls2k1000la"`）。

    异常：
    - `ValueError`: 缺少板级链接符号、平台标识不匹配、LoongArch DMW 偏移错误、装载段虚实映射或节权限不符合链接脚本约定。
    """
    symbols = layout["symbols"]
    # 1. 核对链接脚本导出的平台标识符号与内核物理/虚拟起止边界
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
    # 2. 核对每个 PT_LOAD 段的读写执行权限与虚实地址线性偏移关系
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
    # 3. 核对核心代码段与数据段均非空、带有 SHF_ALLOC 标志，且精确落在对应权限的 PT_LOAD 段内
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
    """按物理地址跨度将内核各 `PT_LOAD` 段的文件内容导出为连续的裸二进制文件 `kernel.bin`。

    段与段之间的物理地址空隙由文件偏移定位自动补零，`.bss` 等无文件字节的尾部内存及 ELF 调试节不写入裸镜像。

    参数：
    - `kernel_elf_path`: 输入内核 ELF 文件路径 `Path`。
    - `raw_kernel_path`: 输出裸二进制文件路径 `Path`（必须尚不存在）。
    - `layout`: 已通过板级布局校验的内核 ELF 结构字典。

    异常：
    - `ValueError`: 裸镜像总跨度为空或超出 32 位无符号整数上限，或读取时 ELF 文件被截断。
    - `OSError`: 文件读写失败。
    """
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
    """构造 64 字节的未压缩 U-Boot legacy 镜像头并计算头部 CRC32 校验和。

    参数：
    - `size`: 载荷字节长度。
    - `checksum`: 载荷数据的无符号 32 位 CRC32 校验和。
    - `architecture_id`: U-Boot 架构编号（RISC-V 为 `26`，LoongArch 为 `27`）。
    - `image_type`: U-Boot 镜像类型（内核 `IH_TYPE_KERNEL=2`，RAM 盘 `IH_TYPE_RAMDISK=3`）。
    - `load`: 固件搬运载荷的目标物理加载地址。
    - `entry`: 固件跳转执行的物理入口地址。
    - `epoch`: 32 位镜像构建时间戳（秒）。

    返回：
    - 填入头部自身 CRC32 后的 64 字节 `bytes`。

    异常：
    - `ValueError`: 长度、校验和、地址或时间戳超出 32 位无符号整数表示范围。
    """
    if not all(0 <= value < U32_LIMIT for value in (size, checksum, load, entry, epoch)):
        raise ValueError("legacy image size/address/timestamp exceeds 32 bits")
    name = b"blossom-kernel" if image_type == 2 else b"blossom-root"
    fields = (0x27051956, 0, epoch, size, load, entry, checksum, 5, architecture_id, image_type, 0,
              name.ljust(32, b"\0"))
    header = LEGACY_HEADER.pack(*fields)
    return LEGACY_HEADER.pack(fields[0], zlib.crc32(header), *fields[2:])


def write_legacy_image(source_path, output_path, architecture_id, image_type, load, entry, epoch):
    """以分块流式读取方式将载荷封装为 U-Boot legacy 镜像（`uImage` 或 `uInitrd`）。

    先在文件开头预留 64 字节头部空间，边复制载荷边累加计算 CRC32，最后回填完整头部，避免一次性将大体积 ext4 镜像读入内存。

    参数：
    - `source_path`: 原始载荷文件路径 `Path`。
    - `output_path`: 输出 legacy 镜像路径 `Path`（必须尚不存在）。
    - `architecture_id`: U-Boot 架构编号。
    - `image_type`: U-Boot 镜像类型编号。
    - `load`: 32 位物理加载地址。
    - `entry`: 32 位物理入口地址。
    - `epoch`: 32 位镜像时间戳。

    返回：
    - 生成的 legacy 镜像文件总字节数（头部 64 字节 + 载荷大小）。

    异常：
    - `ValueError`: 载荷为空、超过 4GiB 上限、读取期间文件大小发生变化，或头部字段超出 32 位范围。
    - `OSError`: 文件读写失败。
    """
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
    """校验作为 RAM 根文件系统输入的 ext4 镜像是未压缩且不带日志特性的原始卷。

    参数：
    - `root_ext4_image_path`: 待包装的根文件系统 ext4 镜像路径 `Path`。

    异常：
    - `ValueError`: 不是普通文件、超级块魔数无效（例如传入了压缩包），或启用了内核当前不支持的 `has_journal` / `recover` 特性。
    - `OSError`: 文件读取失败。
    """
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
    """将物理地址转换为开发板固件命令行读写内存时使用的运行时地址。

    RISC-V U-Boot 直接使用恒等映射物理地址；LoongArch PMON/U-Boot 通过 `0x9000_0000_0000_0000` 缓存直接映射窗口访问内存，
    但 legacy 容器头部和 DTB `initrd` 属性仍记录纯物理地址。

    参数：
    - `architecture`: 目标架构名称。
    - `physical`: 物理内存地址。

    返回：
    - 固件侧命令使用的地址整数。
    """
    return physical | 0x9000000000000000 if architecture == "loongarch64" else physical


def write_os_dtb(source_path, target_path, initrd_range):
    """校验并复制操作系统 DTB，可选写入 RAM 根物理地址范围并追加 64KiB 空白余量。

    不在源文件上原地修改；追加 64KiB 并同步更新 FDT 头部的 `totalsize`，以便固件在启动时原地注入命令行或内存节点而不溢出。

    参数：
    - `source_path`: 输入的原始操作系统 DTB 路径 `Path`。
    - `target_path`: 输出的打包后 DTB 路径 `Path`。
    - `initrd_range`: 若包含 RAM 根镜像，则为 `(initrd_start, initrd_end)` 物理地址二元组；否则为 `None`。

    返回：
    - 扩容后的输出 DTB 文件总字节数。

    异常：
    - `ValueError`: FDT 魔数或头部长度字段非法、`fdtput` 写入 `/chosen` 属性失败，或扩容后大小超出 32 位上限。
    - `OSError`: 文件读写或调用 `fdtput` 失败。
    """
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
    """规划内核、RAM 根与 DTB 的物理暂存地址布局，生成 `uImage` / `uInitrd` / `board.dtb` 及 `boot.env`。

    暂存区紧接在内核运行内存末尾（`__kernel_phys_end` 范围内最后一段之后）按页对齐向高地址排列，
    避免固件解压/搬运 `kernel.uImage` 时覆盖尚未读取完的自身容器或紧随其后的 `uInitrd` 与 `board.dtb`。

    参数：
    - `architecture`: 目标架构（`"riscv64"` 或 `"loongarch64"`）。
    - `target_platform`: 目标开发板名称（`"visionfive2"` 或 `"ls2k1000la"`）。
    - `layout`: 内核 ELF 结构字典。
    - `raw_kernel_path`: 已生成的裸内核二进制路径 `Path`。
    - `output_directory`: 当前打包临时输出目录 `Path`。
    - `epoch`: 32 位镜像时间戳。
    - `root_ext4_image_path`: 可选的原始 ext4 根镜像路径 `Path`。
    - `os_dtb_path`: 可选的操作系统 DTB 路径 `Path`。
    - `root_device`: 可选的内核根设备路径（如 `"/dev/ram0"` 或 `"/dev/mmcblk0p2"`）。
    - `root_mode`: 根文件系统挂载模式（`"ro"` 或 `"rw"`）。
    - `init_program`: 内核启动的首个用户态程序路径。

    异常：
    - `ValueError`: 物理地址区间越出 32 位边界、各镜像内存区间发生重叠，或根设备/`init` 参数不合法。
    - `OSError`: 镜像或环境文件写入失败。
    """
    architecture_id = 26 if architecture == "riscv64" else 27
    load = min(segment["physical"] for segment in layout["loads"])
    memory_end = max(segment["physical"] + segment["memory_size"] for segment in layout["loads"])
    entry = kernel_physical_entry(layout)
    # 将 kernel.uImage 暂存在内核运行内存结束位置之后，防止固件将载荷搬运到 load 地址时踩踏容器自身
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
    # 若提供了 RAM 根镜像，紧随内核容器之后按页对齐放置 root.uInitrd，并记录剥去 64 字节头后的真实载荷物理区间
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
    # 若提供了操作系统 DTB，写入 initrd 物理区间后紧接放置在后续页对齐位置
    if os_dtb_path is not None:
        size = write_os_dtb(os_dtb_path, output_directory / "board.dtb", initrd_range)
        spans.append((cursor, cursor + size, "OS DTB"))
        fdt_address = board_runtime_address(architecture, cursor)
        rows += ["fdt_file=board.dtb", f"fdt_addr={fdt_address:#x}", f"fdt_size={size:#x}",
                 "fdt_high=0xffffffffffffffff"]
    else:
        rows.append("# 未提供 os_dtb；启动前须准备实际传给内核的 OS DTB。")
    # 统一验证内核运行区与所有暂存容器均位于 32 位物理地址空间内且互不重叠
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
    """离线生成并校验整套开发板启动文件，全部成功后替换发布到目标输出目录。

    打包开始时先将输入 `kernel.elf` 复制到临时目录固定下来，后续解析、校验与裸二进制提取均针对同一副本执行，
    防止并发构建修改源 ELF 导致元数据与载荷不一致；若本次未提供可选的 RAM 根或 DTB，则清理输出目录中可能残留的旧同名产物。

    参数：
    - `architecture`: 目标架构（`"riscv64"` 或 `"loongarch64"`）。
    - `target_platform`: 与架构配对的目标开发板名称（`"visionfive2"` 或 `"ls2k1000la"`）。
    - `kernel_elf_path`: 输入内核 ELF 文件路径 `Path`。
    - `output_directory`: 目标输出目录 `Path`。
    - `root_ext4_image_path`: 可选的原始 ext4 根镜像路径 `Path`。
    - `os_dtb_path`: 可选的操作系统 DTB 路径 `Path`。
    - `root_device`: 可选的显式根设备路径字符串。
    - `root_mode`: 根挂载模式，默认 `"ro"`。
    - `init_program`: 初始用户态进程路径，默认 `"/sbin/init"`。

    异常：
    - `ValueError`: 架构与开发板型号不匹配、环境变量 `SOURCE_DATE_EPOCH` 越界，或任一输入校验未通过。
    - `OSError` / `struct.error`: 文件操作或二进制打包失败。
    """
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
    # 在目标输出目录下创建隐藏临时目录，全部镜像生成并校验无误后再用 os.replace 逐个原子发布
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
    """解析命令行参数并执行离线开发板启动镜像打包。

    参数：
    - `argv`: 可选的命令行参数列表；默认读取 `sys.argv[1:]`。

    异常：
    - `SystemExit`: 参数解析错误或打包失败时以非零状态码退出。
    """
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
