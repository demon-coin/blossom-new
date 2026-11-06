"""`prepare_user_selftest_rootfs.py` 用户态自测根文件系统与 ext4 镜像制作回归测试。

本模块验证单轮用户态自测根文件系统准备器的关键行为：
- 从已安装的用户态目录和运行库目录装配根文件系统树并制作 ext4 镜像；
- 自动剔除残留的 `/test/selftests/exit-status` 旧结果文件，且保持只读输入目录不被修改；
- 重新准备时完整清理旧输出文件并同步反映上游安装文件的增删变化；
- 在缺失必要输入文件、路径重叠、符号链接越界或缺少宿主制卷工具时安全报错并保留原输出。
"""

from contextlib import redirect_stderr
import io
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from host_helpers import TOOLS
from prepare_user_selftest_rootfs import main, prepare_user_selftest_rootfs
from test_rootfs import tree_contents


HAS_E2FSPROGS = all(shutil.which(name) for name in ("mke2fs", "debugfs", "e2fsck", "dumpe2fs"))
RUN_ID = "0123456789abcdef0123456789abcdef"


class UserSelftestRootfsTests(unittest.TestCase):
    """验证用户态自测根文件系统准备器的文件装配、镜像构建与输入输出边界检查。"""

    def setUp(self):
        """创建带有空格和特殊 Shell 字符的临时安装源目录、运行库目录和输出路径，并写入测试用程序与清单文件。"""
        temporary = tempfile.TemporaryDirectory(prefix="selftest-rootfs-")
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.userland = self.base / "installed user ' $ files"
        self.runtime = self.base / "installed libraries"
        self.output = self.base / "owned root ' ; $(not-a-command)"
        self.image = self.base / "owned image ' ; $(not-a-command).ext4"
        for name in ("bin/busybox", "bin/libc-smoke", "sbin/contest-run", "sbin/user-selftests",
                     "test/selftests/custom/probe", "test/selftests/custom/helper"):
            path = self.userland / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"installed:" + name.encode())
            path.chmod(0o755)
        (self.userland / "bin/sh").symlink_to("busybox")
        (self.userland / "sbin/init").symlink_to("../bin/busybox")
        self.manifest = self.userland / "test/selftests/kselftest-list.txt"
        self.manifest.write_text("custom:probe\n", encoding="utf-8")
        for relative in ("musl/lib/libc.so", "glibc/lib/libc.so.6", "glibc/lib/libm.so.6"):
            path = self.runtime / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"runtime:" + relative.encode())

    def inputs(self):
        """采集所有输入目录（用户态、运行库与系统配置目录）的完整快照，用于断言准备过程不修改任何输入源。"""
        return tuple(tree_contents(path) for path in (
            self.userland, self.runtime, TOOLS.parent / "userland/system"))

    def prepare(self, **kwargs):
        """使用默认测试参数（`riscv64`、128 MiB 镜像大小、固定 `RUN_ID`）调用 `prepare_user_selftest_rootfs`。"""
        return prepare_user_selftest_rootfs(
            "riscv64", self.userland, self.runtime, self.output, self.image,
            image_size_mib=128, run_id=RUN_ID, **kwargs)

    @unittest.skipUnless(HAS_E2FSPROGS, "requires e2fsprogs")
    def test_preparation_removes_installed_stale_results_from_tree_and_volume(self):
        """验证准备过程保持输入源目录不变，但会清除复制出的根文件系统树和新建 ext4 镜像中的旧 `exit-status` 文件。"""
        stale = self.userland / "test/selftests/exit-status"
        stale.write_bytes(b"run=ffffffffffffffffffffffffffffffff status=0\n")
        before = self.inputs()
        self.prepare(collections=["custom"], programs=["custom:probe"],
                     program_timeout_seconds=7)
        # 确认输入源未被改动，输出树与 ext4 镜像中均已剔除旧的 exit-status，且自测程序已写入镜像。
        self.assertEqual(self.inputs(), before)
        self.assertFalse((self.output / "test/selftests/exit-status").exists())
        self.assertTrue(os.access(self.output / "sbin/run-user-selftests", os.X_OK))
        result = subprocess.run(
            ["debugfs", "-R", "cat /test/selftests/exit-status", str(self.image)],
            capture_output=True, check=True)
        self.assertEqual(result.stdout, b"")
        for relative in ("custom/probe", "custom/helper"):
            result = subprocess.run(
                ["debugfs", "-R", f"cat /test/selftests/{relative}", str(self.image)],
                capture_output=True, check=True)
            self.assertEqual(result.stdout, (self.userland / "test/selftests" / relative).read_bytes())

    @unittest.skipUnless(HAS_E2FSPROGS, "requires e2fsprogs")
    def test_repreparation_replaces_only_owned_tree_and_reflects_installed_deletion(self):
        """验证再次执行准备时会清除输出目录中的陈旧文件，并同步移除上游安装源中已删除的程序。"""
        self.prepare()
        (self.output / "stale-output").write_bytes(b"obsolete")
        (self.userland / "test/selftests/custom/helper").unlink()
        self.manifest.write_text("custom:replacement\n", encoding="utf-8")
        before = self.inputs()
        self.prepare()
        # 确认重新准备后陈旧输出和已删除的 helper 程序均不复存在，且更新后的清单已生效。
        self.assertEqual(self.inputs(), before)
        self.assertFalse((self.output / "stale-output").exists())
        self.assertFalse((self.output / "test/selftests/custom/helper").exists())
        self.assertEqual((self.output / "test/selftests/kselftest-list.txt").read_bytes(),
                         b"custom:replacement\n")
        missing = self.base / "removed-helper.dump"
        subprocess.run(["debugfs", "-R", f"dump /test/selftests/custom/helper {missing}",
                        str(self.image)], capture_output=True, check=True)
        self.assertFalse(missing.exists())

    @unittest.skipUnless(HAS_E2FSPROGS, "requires e2fsprogs")
    def test_symlink_workspace_parent_prepares_tree_and_real_small_volume(self):
        """验证当工作区父目录是普通目录符号链接时，仍能正常完成根文件系统树装配与 8 MiB ext4 镜像制作。"""
        workspace = self.base / "workspace"
        workspace.symlink_to(self.base, target_is_directory=True)
        output = workspace / "small-root"
        image = workspace / "small.ext4"
        before = self.inputs()
        prepare_user_selftest_rootfs(
            "riscv64", self.userland, self.runtime, output, image,
            image_size_mib=8, run_id=RUN_ID)
        self.assertEqual(self.inputs(), before)
        self.assertEqual((output / "test/selftests/custom/helper").read_bytes(),
                         (self.userland / "test/selftests/custom/helper").read_bytes())
        result = subprocess.run(
            ["debugfs", "-R", "cat /test/selftests/custom/helper", str(image)],
            capture_output=True, check=True)
        self.assertEqual(result.stdout, b"installed:test/selftests/custom/helper")

    def test_missing_installed_runner_or_manifest_preserves_existing_outputs(self):
        """验证当缺少必需的 `sbin/user-selftests` 或 `kselftest-list.txt` 时，在修改现有输出目录或镜像前直接报错。"""
        self.output.mkdir()
        (self.output / "preserved").write_bytes(b"old tree")
        self.image.write_bytes(b"old volume")
        before = tree_contents(self.output)
        for relative in ("sbin/user-selftests", "test/selftests/kselftest-list.txt"):
            path = self.userland / relative
            original = path.read_bytes()
            path.unlink()
            with self.subTest(relative=relative), self.assertRaises(ValueError):
                self.prepare()
            self.assertEqual(tree_contents(self.output), before)
            self.assertEqual(self.image.read_bytes(), b"old volume")
            path.write_bytes(original)

    def test_input_overlap_symlinks_and_invalid_selection_do_not_write_inputs(self):
        """验证当输出路径与输入目录重叠、经过符号链接指向输入源或包含非法选择参数时，直接拒绝且不改动任何输入。"""
        before = self.inputs()
        # 输出目录或输出镜像与输入目录重叠时必须抛出 ValueError。
        for output, image in ((self.userland, self.image),
                              (self.userland / "nested", self.image),
                              (self.output, self.runtime / "new.ext4"),
                              (self.output, self.output / "nested.ext4")):
            with self.subTest(output=output, image=image), self.assertRaises(ValueError):
                prepare_user_selftest_rootfs("riscv64", self.userland, self.runtime,
                                             output, image, image_size_mib=128, run_id=RUN_ID)
        # 输出目录本身或其父目录通过符号链接指向输入目录时必须抛出 ValueError。
        self.output.symlink_to(self.userland, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.prepare()
        self.output.unlink()
        ancestor = self.base / "aliased-parent"
        ancestor.symlink_to(self.runtime, target_is_directory=True)
        with self.assertRaises(ValueError):
            prepare_user_selftest_rootfs("riscv64", self.userland, self.runtime,
                                         ancestor / "root", self.image, run_id=RUN_ID)
        # 空字符串或包含 NUL 字节的程序名、非正整数超时参数均须被拒绝。
        for selection in ("", "custom:\0probe"):
            with self.subTest(selection=selection), self.assertRaises(ValueError):
                self.prepare(programs=[selection])
        for timeout in (0, -1, 1.5):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                self.prepare(program_timeout_seconds=timeout)
        self.assertEqual(self.inputs(), before)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.image.exists())

    def test_real_tool_discovery_failure_keeps_previous_volume_and_input_bytes(self):
        """验证当 `PATH` 中找不到宿主制卷工具时，抛出 `ValueError` 并保留已有的旧镜像文件与输入源内容。"""
        self.image.write_bytes(b"previous volume")
        before = self.inputs()
        with patch.dict(os.environ, {"PATH": ""}), self.assertRaises(ValueError):
            self.prepare()
        self.assertEqual(self.image.read_bytes(), b"previous volume")
        self.assertEqual(self.inputs(), before)

    def test_cli_reports_preparation_error_without_creating_outputs(self):
        """验证 CLI 入口在输入缺失时返回非零退出码并向 `stderr` 输出错误信息，且不创建任何输出目录或镜像。"""
        self.manifest.unlink()
        before = self.inputs()
        error = io.StringIO()
        with redirect_stderr(error):
            status = main(["--arch", "loongarch64", "--guest-userland-directory", str(self.userland),
                           "--runtime-libraries-directory", str(self.runtime),
                           "--output-directory", str(self.output), "--output-image", str(self.image),
                           "--run-id", RUN_ID])
        self.assertEqual(status, 1)
        self.assertTrue(error.getvalue())
        self.assertEqual(self.inputs(), before)
        self.assertFalse(self.output.exists())
        self.assertFalse(self.image.exists())


if __name__ == "__main__":
    unittest.main()
