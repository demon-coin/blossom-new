#!/usr/bin/env python3
"""用户态自测（user-selftests）专用根文件系统与 ext4 镜像准备脚本。

本脚本在启动 QEMU 运行用户态自测前调用，负责复用 `assemble_rootfs` 与 `create_ext4_image`
生成包含自测启动入口、过滤条件和本轮唯一运行标识（`run_id`）的根文件系统目录树及 ext4 镜像。
本脚本只准备测试磁盘镜像，不触发代码编译，也不直接启动 QEMU。

输入：
- 目标架构、已安装的客户机用户态目录（须含 `/sbin/user-selftests` 及用例清单）、运行库目录；
- 本轮 32 位十六进制 `run_id`、可选的集合/单程序过滤条件、单程序超时秒数，以及目标目录与镜像路径。

输出：
- 写入专用 `/etc/inittab` 与 `/sbin/run-user-selftests` 启动脚本的根文件系统目录树；
- 打包并通过离线一致性校验的 ext4 根镜像文件。客户机启动后将自动运行自测、落盘带 `run_id` 的退出状态并执行关机。
"""

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

from assemble_rootfs import REPOSITORY_ROOT, assemble_rootfs_tree
from create_ext4_image import PROFILES, create_ext4_image
from rootfs_paths import output_path, reject_overlap


def prepare_user_selftest_rootfs(architecture, guest_userland_directory,
                                 runtime_libraries_directory, output_directory,
                                 output_image, collections=(), programs=(),
                                 profile="seed4k", image_size_mib=256, *,
                                 run_id, program_timeout_seconds=None):
    """组装本轮用户态自测专用的根文件系统目录树并打包为 ext4 镜像。

    每次调用都会清除可能残留的历史 `exit-status` 文件，并根据调用方传入的过滤条件重新生成
    `/sbin/run-user-selftests`；若未显式传入 `program_timeout_seconds`，则不向命令行追加该选项，
    交由客户机内的 `/sbin/user-selftests` 使用其内置默认超时。

    参数：
    - `architecture`: 目标架构（`"riscv64"` 或 `"loongarch64"`）。
    - `guest_userland_directory`: 已安装的客户机用户态程序目录。
    - `runtime_libraries_directory`: 动态运行库目录。
    - `output_directory`: 输出根文件系统目录路径。
    - `output_image`: 输出 ext4 镜像文件路径。
    - `collections`: 要筛选运行的测试集合名称序列，默认为空（运行全部）。
    - `programs`: 要筛选运行的单个测试程序名称序列，默认为空。
    - `profile`: ext4 镜像规格名称，默认 `"seed4k"`。
    - `image_size_mib`: 镜像大小（单位 MiB），默认 `256`。
    - `run_id`: 必填关键字参数，32 位小写十六进制字符串，用于宿主侧核对退出状态归属本轮运行。
    - `program_timeout_seconds`: 可选的单程序超时秒数（正整数）。

    返回：
    - 生成并校验通过的 ext4 镜像 `Path`。

    异常：
    - `ValueError`: `run_id` 或超时参数格式非法、输入输出路径重叠、缺少自测程序或清单，或筛选参数含空串/NUL。
    - `OSError` / `OverflowError` / `subprocess.CalledProcessError`: 目录组装或 ext4 制盘失败。
    """
    if re.fullmatch(r"[0-9a-f]{32}", run_id) is None:
        raise ValueError("run ID must contain exactly 32 lowercase hexadecimal digits")
    if program_timeout_seconds is not None and (
            not isinstance(program_timeout_seconds, int) or program_timeout_seconds < 1):
        raise ValueError("program timeout must be a positive integer")
    userland = Path(guest_userland_directory).resolve(strict=True)
    runtime = Path(runtime_libraries_directory).resolve(strict=True)
    output = output_path(output_directory, directory=True)
    image = output_path(output_image)
    log = output_path(image.with_suffix(".log"))
    # 1. 检查所有只读输入目录与本轮三个输出产物（目录树、ext4 镜像、制盘日志）之间互不重叠
    sources = (userland, runtime, (REPOSITORY_ROOT / "userland/system").resolve(),
               (REPOSITORY_ROOT / "userland/contest").resolve())
    for target in (output, image, log):
        for source in sources:
            reject_overlap(source, target)
    reject_overlap(output, image)
    reject_overlap(output, log)
    reject_overlap(image, log)
    if profile not in PROFILES or image_size_mib <= 0:
        raise ValueError("unsupported ext4 profile or nonpositive image size")
    # 2. 校验用户态自测执行器与测试清单文件已安装，并对转交到 Shell 脚本的筛选参数做非空与无 NUL 校验
    for name in ("sbin/user-selftests", "test/selftests/kselftest-list.txt"):
        if not (userland / name).is_file():
            raise ValueError(f"required installed selftest asset missing: {userland / name}")
    arguments = ["/sbin/user-selftests"]
    for option, values in (("--collection", collections), ("--program", programs)):
        for value in values:
            if not value or "\0" in value:
                raise ValueError(f"{option} must be nonempty and contain no NUL")
            arguments.extend((option, value))
    if program_timeout_seconds is not None:
        arguments.extend(("--program-timeout-seconds", str(program_timeout_seconds)))
    # 3. 生成专用的 inittab：系统初始化后在串口控制台单次执行 /sbin/run-user-selftests，再组装根目录树
    with tempfile.TemporaryDirectory(prefix="user-selftest-init-") as temporary:
        config = Path(temporary) / "inittab"
        config.write_text(
            "::sysinit:/bin/sh /etc/init.d/rcS\n"
            "ttyS0::once:/bin/sh /sbin/run-user-selftests\n"
            "::shutdown:/bin/sh /etc/init.d/rcK\n", encoding="utf-8")
        tree = assemble_rootfs_tree(architecture, userland, runtime, output, config)
    # 4. 清理旧状态文件并写入启动脚本：执行自测、将非约定退出码收敛为 2、打印并落盘带 run_id 的结果后触发关机
    (tree / "test/selftests/exit-status").unlink(missing_ok=True)
    action = tree / "sbin/run-user-selftests"
    action.unlink(missing_ok=True)
    action.write_text(
        "#!/bin/sh\n"
        "/bin/busybox rm -f /test/selftests/exit-status || exit 2\n"
        + shlex.join(arguments) + "\n"
        "status=$?\n"
        "case \"$status\" in 0|1|2|4) ;; *) status=2 ;; esac\n"
        "printf 'user-selftests exit=%s\\n' \"$status\"\n"
        f"printf 'run={run_id} status=%s\\n' \"$status\" > /test/selftests/exit-status\n"
        "/bin/busybox poweroff\n"
        "exit \"$status\"\n", encoding="utf-8")
    action.chmod(0o755)
    return create_ext4_image(tree, image, profile, image_size_mib)


def main(argv=None):
    """解析命令行参数并准备单次用户态自测的根文件系统目录树与 ext4 镜像。

    参数：
    - `argv`: 可选的命令行参数列表；默认读取 `sys.argv[1:]`。

    返回：
    - 进程退出码：准备成功返回 `0`，参数校验或镜像生成失败返回 `1`。
    """
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--arch", choices=("riscv64", "loongarch64"), required=True)
    parser.add_argument("--guest-userland-directory", type=Path, required=True)
    parser.add_argument("--runtime-libraries-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--output-image", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--program-timeout-seconds", type=int)
    parser.add_argument("--collection", action="append", default=[])
    parser.add_argument("--program", action="append", default=[])
    parser.add_argument("--profile", choices=PROFILES, default="seed4k")
    parser.add_argument("--size-mib", type=int, default=256)
    args = parser.parse_args(argv)
    try:
        prepare_user_selftest_rootfs(
            args.arch, args.guest_userland_directory, args.runtime_libraries_directory,
            args.output_directory, args.output_image, args.collection, args.program,
            args.profile, args.size_mib, run_id=args.run_id,
            program_timeout_seconds=args.program_timeout_seconds)
    except (OSError, ValueError, OverflowError, subprocess.CalledProcessError) as error:
        print(f"User selftest preparation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
