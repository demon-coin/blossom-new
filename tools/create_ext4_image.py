#!/usr/bin/env python3
"""ext4 文件系统镜像生成与离线一致性校验脚本。

本脚本在构建与测试流水线中负责将已组装好的根文件系统目录树（或其他测试目录树）打包为
无日志（`^has_journal`）的原始 ext4 镜像文件，供 QEMU 虚拟块设备、板卡 RAM 根文件系统（`uInitrd`）
或文件系统自测使用。

输入：
- 完整的只读源目录树路径、输出镜像路径、ext4 规格配置（`legacy1k` / `metadata2k` / `seed4k`）及镜像容量（MiB）。

输出：
- 通过 `mke2fs -d` 填充并通过超级块几何核对、`dumpe2fs`、`debugfs` 与 `e2fsck -fn` 只读检查的 ext4 镜像文件；
- 同名的 `.log` 工具执行日志文件（即使制盘或检查失败也会保留日志以便排查）。
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from rootfs_paths import output_path, reject_overlap


# 预定义的 ext4 规格表：名称 -> (块大小字节数, inode 大小字节数, mke2fs -O 特性列表)
PROFILES = {
    "legacy1k": (1024, 128, "none,ext_attr,resize_inode,dir_index,filetype,sparse_super,large_file,uninit_bg"),
    "metadata2k": (2048, 256, "none,ext_attr,resize_inode,dir_index,filetype,sparse_super,large_file,extent,metadata_csum,extra_isize"),
    "seed4k": (4096, 256, "none,ext_attr,resize_inode,dir_index,filetype,sparse_super,large_file,extent,64bit,metadata_csum,metadata_csum_seed,extra_isize"),
}


def run_checked_command(arguments, log_path, accepted=(0,)):
    """运行外部命令并将命令行与合并输出追加到日志文件，仅接受指定的退出状态码。

    执行时固定 `LC_ALL=C` 与 `E2FSPROGS_FAKE_TIME`，保证工具输出语言一致且生成的 ext4 时间戳可复现。

    参数：
    - `arguments`: 命令及参数序列（元素会被转换为字符串）。
    - `log_path`: 追加记录命令与输出的日志文件路径 `Path`。
    - `accepted`: 允许视为成功的进程退出码集合，默认仅接受 `(0,)`。

    返回：
    - 解码后的标准输出/错误合并文本（非 ASCII 字节替换为占位符）。

    异常：
    - `subprocess.CalledProcessError`: 进程退出码不在 `accepted` 集合中。
    """
    result = subprocess.run([str(item) for item in arguments], check=False,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            env={**os.environ, "LC_ALL": "C", "E2FSPROGS_FAKE_TIME": "1700000000"})
    with log_path.open("ab") as stream:
        stream.write(("$ " + " ".join(map(str, arguments)) + "\n").encode())
        stream.write(result.stdout)
    if result.returncode not in accepted:
        raise subprocess.CalledProcessError(result.returncode, arguments, output=result.stdout)
    return result.stdout.decode("ascii", errors="replace")


def check_debugfs_result(text, request):
    """检查 `debugfs` 的输出文本，拦截退出码为 0 但实际报错的请求。

    `debugfs -R` 在遇到未知命令、缺少文件或无法打开文件系统等错误时仍可能以状态码 0 退出，
    因此必须扫描输出文本中的典型错误提示。

    参数：
    - `text`: `debugfs` 命令的标准输出与标准错误合并文本。
    - `request`: 传给 `debugfs -R` 的原始请求字符串，用于构造异常提示。

    返回：
    - 校验通过的原输出文本 `text`。

    异常：
    - `ValueError`: 输出文本中包含 `debugfs` 错误诊断关键字。
    """
    if re.search(r"(?:Command not found|Usage:|Invalid field|Unknown feature|Filesystem not open|File not found|while (?:opening|setting|allocating|punching))", text, re.IGNORECASE):
        raise ValueError(f"debugfs rejected request: {request}")
    return text


def run_debugfs_command(image_path, request, writable=False):
    """在指定镜像上执行单条 `debugfs` 请求，并将输出追加到镜像同名的 `.log` 日志。

    默认以只读模式打开镜像；测试夹具需要构造特定磁盘布局时必须显式传入 `writable=True`。

    参数：
    - `image_path`: 目标 ext4 镜像路径 `Path`。
    - `request`: 传给 `debugfs -R` 的单条命令字符串。
    - `writable`: 是否启用 `-w` 读写模式，默认为 `False`。

    返回：
    - 经 `check_debugfs_result` 校验后的命令输出文本。

    异常：
    - `subprocess.CalledProcessError`: `debugfs` 进程返回非零退出码。
    - `ValueError`: `debugfs` 输出中包含错误诊断文本。
    """
    arguments = ["debugfs"]
    if writable:
        arguments.append("-w")
    arguments.extend(["-R", request, image_path])
    return check_debugfs_result(
        run_checked_command(arguments, image_path.with_suffix(".log")), request)


def check_ext4_image(image_path, profile):
    """对生成的 ext4 镜像执行只读超级块几何校验与 `e2fsprogs` 离线一致性检查。

    不执行任何自动修复或日志回放，确保镜像在交付给内核前本身就是干净一致的状态。

    参数：
    - `image_path`: 待检查的 ext4 镜像路径 `Path`。
    - `profile`: `PROFILES` 中的规格名称，用于核对块大小和 inode 大小。

    异常：
    - `ValueError`: 超级块魔数、块大小编码或 inode 大小与请求的规格不符，或 `debugfs` 报错。
    - `subprocess.CalledProcessError`: `dumpe2fs`、`debugfs` 或 `e2fsck -fn` 返回非零退出码。
    """
    # 直接读取偏移 1024 字节处的主超级块，核对魔数 (0xEF53)、s_log_block_size 与 s_inode_size
    with image_path.open("rb") as stream:
        stream.seek(1024)
        superblock = stream.read(1024)
    block_size, inode_size, _ = PROFILES[profile]
    if (len(superblock) != 1024 or superblock[56:58] != b"\x53\xef"
            or int.from_bytes(superblock[24:28], "little") != block_size.bit_length() - 11
            or int.from_bytes(superblock[88:90], "little") != inode_size):
        raise ValueError("generated superblock does not match requested geometry")
    # 依次导出超级块摘要、验证根目录可读，并用 e2fsck -fn 在只读模式下做全盘一致性检查
    run_checked_command(["dumpe2fs", "-h", image_path], image_path.with_suffix(".log"))
    run_debugfs_command(image_path, "stat /")
    run_checked_command(["e2fsck", "-fn", image_path], image_path.with_suffix(".log"))


def create_ext4_image(source_tree_dir, output_image_path, profile="seed4k", image_size_mib=256):
    """以只读源目录树为内容生成并校验 ext4 镜像，成功后原子替换到目标路径。

    先在目标父目录的临时子目录中创建稀疏镜像文件、调用 `mke2fs -d` 写入目录树并完成只读自检；
    若中途失败，仅将临时日志落盘到目标 `.log` 供排查，不会覆盖已有的目标镜像文件。

    参数：
    - `source_tree_dir`: 只读源目录树路径。
    - `output_image_path`: 输出 ext4 镜像文件路径。
    - `profile`: ext4 特性与块大小规格名称，默认为 `"seed4k"`。
    - `image_size_mib`: 镜像文件总大小（单位 MiB），必须为正整数，默认为 `256`。

    返回：
    - 规范化后的输出镜像 `Path`。

    异常：
    - `ValueError`: 参数非法、输入输出路径重叠、宿主机缺少 `e2fsprogs` 工具，或生成的镜像校验未通过。
    - `OSError` / `OverflowError` / `subprocess.CalledProcessError`: 文件操作或外部制盘命令失败。
    """
    source = Path(source_tree_dir).resolve(strict=True)
    output = output_path(output_image_path)
    log = output_path(output.with_suffix(".log"))
    if not source.is_dir():
        raise ValueError("source tree must be an existing directory")
    if profile not in PROFILES:
        raise ValueError(f"unsupported ext4 profile: {profile}")
    if image_size_mib <= 0:
        raise ValueError("image size must be positive")
    # 拒绝输出镜像、日志文件与源目录树之间出现路径包含或硬链接复用
    reject_overlap(output, log)
    for target in (output, log):
        reject_overlap(source, target)
    for binary in ("mke2fs", "debugfs", "e2fsck", "dumpe2fs"):
        if shutil.which(binary) is None:
            raise ValueError(f"required tool missing: {binary}")
    output.parent.mkdir(parents=True, exist_ok=True)
    # 在输出文件同级临时目录中完成制盘与校验，保证最终 os.replace 在同一文件系统内原子生效
    with tempfile.TemporaryDirectory(prefix="ext4-image-", dir=output.parent) as temporary:
        image = Path(temporary) / "root.ext4"
        temporary_log = image.with_suffix(".log")
        try:
            block_size, inode_size, features = PROFILES[profile]
            # 预分配指定 MiB 大小的空镜像文件，固定 UUID 并关闭延迟 inode 表初始化，生成确定性镜像
            with image.open("xb") as stream:
                stream.truncate(image_size_mib * 1024 * 1024)
            run_checked_command([
                "mke2fs", "-q", "-F", "-t", "ext4", "-b", block_size,
                "-I", inode_size, "-O", features,
                "-E", "lazy_itable_init=0", "-L", profile,
                "-U", "ae649880-7392-4424-a21c-392fdc0d1357", "-d", source, image,
            ], temporary_log)
            check_ext4_image(image, profile)
        except (ValueError, OSError, OverflowError, subprocess.CalledProcessError):
            # 制盘或校验失败时仍保存工具日志，但不触碰原有的目标镜像文件
            if temporary_log.exists():
                os.replace(temporary_log, log)
            raise
        os.replace(temporary_log, log)
        os.replace(image, output)
    print(f"Ext4 image created: output={output} profile={profile} source={source}")
    return output


def main(argv=None):
    """解析命令行参数并从现成目录树创建 ext4 镜像。

    参数：
    - `argv`: 可选的命令行参数列表；默认读取 `sys.argv[1:]`。

    返回：
    - 进程退出码：镜像创建并校验成功返回 `0`，失败返回 `1`。
    """
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--source-tree", type=Path, required=True)
    parser.add_argument("--output-image", type=Path, required=True)
    parser.add_argument("--profile", choices=PROFILES, default="seed4k")
    parser.add_argument("--size-mib", type=int, default=256)
    args = parser.parse_args(argv)
    try:
        create_ext4_image(args.source_tree, args.output_image, args.profile, args.size_mib)
    except (OSError, ValueError, OverflowError, subprocess.CalledProcessError) as error:
        print(f"Ext4 image creation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
