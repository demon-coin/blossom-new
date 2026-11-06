"""内核 ELF64 可执行文件解析与结构边界校验模块。

本模块供板卡离线打包脚本（如 `package_board_boot_images.py`）和宿主侧 ktest 运行器共享调用，
负责在不启动外部 `readelf` 进程的情况下，通过只读内存映射一次性解析并校验内核 ELF64 映像。

输入：
- 内核 ELF 文件路径与期望的目标架构名称（`"riscv64"` 或 `"loongarch64"`）。

输出：
- 包含入口虚拟地址（`entry`）、ELF 头标志（`flags`）、所有 `PT_LOAD` 装载段列表（`loads`）、
  按节名索引的节表字典（`sections`）以及已定义全局/弱符号地址字典（`symbols`）的结构化字典；
- 本模块仅提取并校验通用 ELF64 结构与文件偏移边界，具体板卡平台符号约束或 ktest 元数据格式由上层调用方检查。
"""

import mmap
import os
from pathlib import Path
import struct


def read_string(image, start, size, offset, label):
    """在已验证文件边界的字符串表切片内读取一个以 NUL 结尾的 UTF-8 字符串。

    参数：
    - `image`: 整个 ELF 文件的只读 `mmap` 视图。
    - `start`: 字符串表节在 ELF 文件中的起始字节偏移。
    - `size`: 字符串表节的字节总长度。
    - `offset`: 目标字符串相对于字符串表起点的字节偏移。
    - `label`: 用于构造错误提示的字段名称（如 `"section name"` 或 `"symbol name"`）。

    返回：
    - 解码后的 UTF-8 字符串（不含末尾 NUL 字节）。

    异常：
    - `ValueError`: `offset` 越出字符串表范围，或在字符串表末尾前未找到 NUL 终止符。
    - `UnicodeDecodeError`: 字符串内容不是合法 UTF-8。
    """
    if not 0 <= offset < size:
        raise ValueError(f"kernel ELF {label} offset is invalid")
    end = image.find(b"\0", start + offset, start + size)
    if end < 0:
        raise ValueError(f"kernel ELF {label} is unterminated")
    return image[start + offset:end].decode("utf-8")


def read_kernel_elf(kernel_elf_path, architecture):
    """映射并解析内核 ELF64 可执行文件，校验头部、装载段、节表和公开符号表边界。

    参数：
    - `kernel_elf_path`: 内核 ELF 文件路径。
    - `architecture`: 期望的目标架构，支持 `"riscv64"` 或 `"loongarch64"`。

    返回：
    - 包含以下键的字典：
      - `entry`: 内核入口虚拟地址（`e_entry`）；
      - `flags`: ELF 处理器特定标志（`e_flags`）；
      - `loads`: `PT_LOAD` 段字典列表，每项含 `flags`、`offset`、`address`、`physical`、`file_size`、`memory_size`、`alignment`；
      - `sections`: 非空节名到节元数据字典的映射；
      - `symbols`: 已定义的全局（`STB_GLOBAL`）或弱（`STB_WEAK`）符号名到符号值（`st_value`）的映射。

    异常：
    - `ValueError`: 架构不支持、ELF 头/ABI 不匹配、程序头或节表越界、物理装载段重叠、入口不在可执行文件段内，或节名/公开符号重名。
    - `OSError`: 文件打开或内存映射失败。
    """
    if architecture not in ("riscv64", "loongarch64"):
        raise ValueError("unsupported ELF architecture")
    with Path(kernel_elf_path).open("rb") as source:
        size = os.fstat(source.fileno()).st_size
        if size < 64:
            raise ValueError("kernel ELF header is truncated")
        with mmap.mmap(source.fileno(), 0, access=mmap.ACCESS_READ) as image:
            # 1. 解析并核对 ELF64 文件头：小端序、可执行文件类型 (ET_EXEC=2)、机器码与软浮点/ABI 标志
            (ident, kind, machine, version, entry, phoff, shoff, flags, ehsize,
             phsize, phnum, shsize, shnum, strings_index) = struct.unpack_from(
                 "<16sHHIQQQIHHHHHH", image)
            if not (ident[:7] == b"\x7fELF\x02\x01\x01" and kind == 2
                    and version == 1 and ehsize == 64):
                raise ValueError("kernel must be a version-one little-endian ELF64 executable")
            if machine != (243 if architecture == "riscv64" else 258):
                raise ValueError("ELF architecture mismatch")
            if not (flags & 7 == (5 if architecture == "riscv64" else 3)
                    and (architecture == "riscv64" or flags & 0xC0 == 0x40)):
                raise ValueError("kernel ELF ABI mismatch")
            if not (phsize == 56 and 0 < phnum < 0xFFFF
                    and 64 <= phoff <= size - phnum * phsize):
                raise ValueError("kernel ELF program table is invalid")
            if not (shsize == 64 and 0 < strings_index < shnum
                    and 64 <= shoff <= size - shnum * shsize):
                raise ValueError("kernel ELF section table is invalid")
            # 2. 遍历程序头表，提取所有 PT_LOAD (p_type=1) 装载段并校验文件偏移、64 位地址空间与对齐约束
            loads = []
            for index in range(phnum):
                segment = struct.unpack_from("<IIQQQQQQ", image, phoff + index * phsize)
                if segment[0] != 1:
                    continue
                _, permission, offset, address, physical, file_size, memory_size, alignment = segment
                if not (permission < 8 and 0 < memory_size and file_size <= memory_size
                        and offset <= size - file_size and address + memory_size <= 1 << 64
                        and physical + memory_size <= 1 << 64):
                    raise ValueError("kernel ELF load bounds or permissions invalid")
                if not (0 < alignment and alignment & (alignment - 1) == 0
                        and address % alignment == offset % alignment):
                    raise ValueError("kernel ELF load alignment invalid")
                loads.append(dict(flags=permission, offset=offset, address=address, physical=physical,
                                  file_size=file_size, memory_size=memory_size, alignment=alignment))
            if not loads:
                raise ValueError("kernel ELF has no LOAD segments")
            # 确保所有装载段在物理内存上互不重叠，且内核入口落在某个具备执行权限 (PF_X=1) 的文件支撑段内
            ordered = sorted(loads, key=lambda load: load["physical"])
            if not all(left["physical"] + left["memory_size"] <= right["physical"]
                       for left, right in zip(ordered, ordered[1:])):
                raise ValueError("kernel ELF physical loads overlap")
            if not any(load["flags"] & 1 and load["address"] <= entry
                       < load["address"] + load["file_size"] for load in loads):
                raise ValueError("kernel ELF entry is not file-backed executable code")
            # 3. 解析节头表与节名字符串表 (SHT_STRTAB=3)，将节读写执行标志归一化为与 PF_R/W/X 一致的位掩码
            headers = [struct.unpack_from("<IIQQQQIIQQ", image, shoff + index * shsize)
                       for index in range(shnum)]
            strings = headers[strings_index]
            strings_start, strings_size = strings[4:6]
            if not (strings[1] == 3 and strings_start <= size - strings_size):
                raise ValueError("kernel ELF section names are invalid")
            sections = {}
            for header in headers:
                name_offset, section_type, section_flags, address, offset, length, *_ = header
                if (1 << 64) < address + length or (section_type != 8 and size - length < offset):
                    raise ValueError("kernel ELF section bounds are invalid")
                name = read_string(image, strings_start, strings_size, name_offset, "section name")
                if not name:
                    continue
                if name in sections:
                    raise ValueError("kernel ELF section name is duplicate")
                permission = 4 | (2 if section_flags & 1 else 0) | (1 if section_flags & 4 else 0)
                sections[name] = dict(address=address, offset=offset, size=length, flags=permission,
                                      type=section_type, section_flags=section_flags)
            # 4. 扫描静态符号表 (SHT_SYMTAB=2)，收集所有已定义的非空全局符号 (bind=1) 与弱符号 (bind=2)
            symbols = {}
            for header in headers:
                if header[1] != 2:
                    continue
                offset, length, link, _, _, entsize = header[4:]
                if not (entsize == 24 and length % entsize == 0 and link < shnum):
                    raise ValueError("kernel ELF symbol table is invalid")
                strings = headers[link]
                start, length_strings = strings[4:6]
                if not (strings[1] == 3 and start <= size - length_strings):
                    raise ValueError("kernel ELF symbol strings are invalid")
                for at in range(offset, offset + length, entsize):
                    name_offset, info, _, defined, value, _ = struct.unpack_from("<IBBHQQ", image, at)
                    name = read_string(image, start, length_strings, name_offset, "symbol name")
                    if not defined or info >> 4 not in (1, 2) or not name:
                        continue
                    if name in symbols:
                        raise ValueError("kernel ELF public symbol is duplicate")
                    symbols[name] = value
    return dict(entry=entry, flags=flags, loads=loads, sections=sections, symbols=symbols)
