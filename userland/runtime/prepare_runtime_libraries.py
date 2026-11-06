#!/usr/bin/env python3
"""从宿主机交叉工具链提取并校验 musl 与 glibc 动态运行库，安装到目标运行库目录并建立加载器符号链接。"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


# 各架构默认的交叉工具链运行库路径、期望的 readelf Machine 标识及动态链接器文件名。
RUNTIME_LIBRARY_SOURCES = {
    "riscv64": (
        "/opt/riscv64--musl--bleeding-edge-2020.08-1/"
        "riscv64-buildroot-linux-musl/sysroot/lib/libc.so",
        "/usr/riscv64-linux-gnu/lib",
        "RISC-V",
        "ld-musl-riscv64-sf.so.1",
        "ld-linux-riscv64-lp64d.so.1",
    ),
    "loongarch64": (
        "/opt/loongarch64-linux-musl-cross/loongarch64-linux-musl/lib/libc.so",
        "/opt/gcc-13.2.0-loongarch64-linux-gnu/sysroot/usr/lib64",
        "LoongArch",
        "ld-musl-loongarch-lp64d.so.1",
        "ld-linux-loongarch-lp64d.so.1",
    ),
}


def check_runtime_library_elf(source_path, expected_machine):
    """调用 `readelf` 检查源库文件并透传原始输出，验证其为匹配目标架构的 `ET_DYN` 共享库。"""
    print(f"Checking ELF source: {source_path}", flush=True)
    result = subprocess.run(
        ["readelf", "-h", "-l", str(source_path)],
        stdout=subprocess.PIPE,
        env={**os.environ, "LC_ALL": "C"},
        check=False,
    )
    sys.stdout.buffer.write(result.stdout)
    sys.stdout.buffer.flush()
    result.check_returncode()
    header = result.stdout.decode("utf-8", errors="replace")
    kind = re.search(r"^\s*Type:\s+(\S+)", header, re.MULTILINE)
    actual_machine = re.search(r"^\s*Machine:\s+(.+)$", header, re.MULTILINE)
    if kind is None or kind.group(1) != "DYN":
        raise ValueError(f"expected ET_DYN runtime library: {source_path}")
    if actual_machine is None or actual_machine.group(1).strip() != expected_machine:
        raise ValueError(f"expected {expected_machine} runtime library: {source_path}")


def copy_runtime_library(source_path, destination_path):
    """通过同目录临时文件复制并原子替换目标运行库，避免跟随目标路径上可能已有的符号链接。"""
    with tempfile.NamedTemporaryFile(
        dir=destination_path.parent, prefix=f".{destination_path.name}.", delete=False
    ) as stream:
        temporary_path = Path(stream.name)
    try:
        shutil.copy2(source_path, temporary_path)
        temporary_path.replace(destination_path)
    finally:
        temporary_path.unlink(missing_ok=True)
    print(f"Copied: {source_path} -> {destination_path}")


def install_loader_link(link_path, target):
    """在 `link_path` 创建指向 `target` 的符号链接；若原路径为普通文件或符号链接则先移除。"""
    if link_path.is_symlink() or link_path.is_file():
        link_path.unlink()
    link_path.symlink_to(target)
    print(f"Linked: {link_path} -> {target}")


def prepare_runtime_libraries(architecture, musl_libc_path, glibc_library_directory,
                              output_directory):
    """先完整校验指定架构所需的 musl 与 glibc 源运行库，再统一复制到输出目录并建立动态链接器符号链接。"""
    _, _, expected_machine, musl_loader, glibc_loader = RUNTIME_LIBRARY_SOURCES[architecture]
    output_directory = output_directory.absolute()
    libraries = [
        (musl_libc_path, output_directory / "musl/lib/libc.so"),
        (glibc_library_directory / "libc.so.6", output_directory / "glibc/lib/libc.so.6"),
        (glibc_library_directory / "libm.so.6", output_directory / "glibc/lib/libm.so.6"),
        (glibc_library_directory / glibc_loader, output_directory / "glibc/lib" / glibc_loader),
    ]
    # 解析源文件的真实路径，确保源文件为存在的普通文件且不与输出目标重合。
    resolved_libraries = []
    for source_path, destination_path in libraries:
        source_path = source_path.resolve(strict=True)
        if not source_path.is_file():
            raise ValueError(f"required source is not a regular file: {source_path}")
        if source_path == destination_path.resolve():
            raise ValueError(f"source and output must be separate: {source_path}")
        resolved_libraries.append((source_path, destination_path))
    # 在写入任何输出文件前，先统一校验全部源运行库的 ELF 类型与目标架构。
    for source_path, _ in resolved_libraries:
        check_runtime_library_elf(source_path, expected_machine)
    # 逐一复制运行库到输出目录，并创建动态链接器入口与 LoongArch64 的 lib64 兼容符号链接。
    for source_path, destination_path in resolved_libraries:
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        copy_runtime_library(source_path, destination_path)
    install_loader_link(output_directory / "musl/lib" / musl_loader, "libc.so")
    if architecture == "loongarch64":
        install_loader_link(output_directory / "musl/lib64", "lib")
        install_loader_link(output_directory / "glibc/lib64", "lib")
    print(f"Runtime libraries prepared: arch={architecture} output_directory={output_directory}")


def main(argv=None):
    """命令行入口：解析架构与运行库路径参数并执行准备流程，失败时输出错误并返回非零退出码。"""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--arch", choices=RUNTIME_LIBRARY_SOURCES, required=True)
    parser.add_argument("--musl-libc", type=Path)
    parser.add_argument("--glibc-libdir", type=Path)
    parser.add_argument("--output-directory", type=Path, required=True)
    args = parser.parse_args(argv)
    default_musl, default_glibc, _, _, _ = RUNTIME_LIBRARY_SOURCES[args.arch]
    try:
        prepare_runtime_libraries(
            args.arch,
            args.musl_libc if args.musl_libc is not None else Path(default_musl),
            args.glibc_libdir if args.glibc_libdir is not None else Path(default_glibc),
            args.output_directory,
        )
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Runtime library preparation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
