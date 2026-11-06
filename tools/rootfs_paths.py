"""根文件系统目录与磁盘镜像制作的路径安全校验工具。

本模块供根文件系统打包与 ext4 制卷脚本共用，负责规范化输出路径、校验输出文件类型（普通文件或普通目录），
并检查输入路径与输出路径之间是否存在目录包含关系或硬链接别名重叠，防止构建或制卷过程意外覆盖源文件。
"""

from pathlib import Path
import stat


def output_path(value, *, directory=False):
    """解析输出路径的父目录符号链接，并校验已存在的目标必须为预期的普通目录或普通文件。

    参数：
    - `value`：待校验的输出路径。
    - `directory`：为 `True` 时要求已存在的目标为普通目录；为 `False` 时要求为普通文件。

    返回：规范化后的 `Path` 对象（若目标尚不存在，则返回解析父目录后的目标路径）。
    异常：当目标已存在但为符号链接、设备节点或类型与 `directory` 不匹配时抛出 `ValueError`。
    """
    path = Path(value)
    # 先解析父目录中的符号链接，保留最后一级文件名以便用 lstat 检查其本身是否为符号链接。
    target = path.parent.resolve() / path.name
    try:
        mode = target.lstat().st_mode
    except FileNotFoundError:
        return target
    expected = stat.S_ISDIR if directory else stat.S_ISREG
    if not expected(mode):
        role = "directory" if directory else "regular file"
        raise ValueError(f"output must be an ordinary {role}: {target}")
    return target.resolve()


def reject_overlap(first, second):
    """检查两个路径是否存在相同、父子目录包含或硬链接指向同一 inode 的重叠情况。

    参数：
    - `first`：第一个路径（通常为输入源目录或工作目录）。
    - `second`：第二个路径（通常为输出目录或输出镜像文件）。

    异常：当两路径相互包含、指向同一文件，或输出文件与源目录树中的任一文件共享硬链接 inode 时抛出 `ValueError`。
    """
    # 检查路径完全相等、互为父子目录，或两侧均已存在且指向同一底层文件（inode）。
    if (first == second or first in second.parents or second in first.parents
            or first.exists() and second.exists() and first.samefile(second)):
        raise ValueError(f"input/output paths overlap or alias: {first}, {second}")
    # 若输出是一个链接数大于 1 的已有普通文件，还需遍历源目录，防止它与源目录内的文件互为硬链接。
    if first.is_dir() and second.is_file() and 1 < second.stat().st_nlink:
        if any(path.is_file() and path.samefile(second) for path in first.rglob("*")):
            raise ValueError(f"output aliases a source file: {second}")
