"""根文件系统目录树装配与 ext4 磁盘镜像制作的宿主端单元测试模块。

本模块验证从用户态构建产物装配根文件系统目录树（`assemble_rootfs`）以及调用 `e2fsprogs`
生成 ext4 磁盘镜像（`create_ext4_image` 和 `ext4_fixtures`）的完整流程与异常回滚边界：
1. 用户态可执行文件、musl/glibc 运行库、系统启动脚本（`inittab`、`start-session`）与评测配置的完整拷贝；
2. 重复装配时对已删文件的清理与自定义 `init` 配置覆盖的重置；
3. 缺失输入、输出路径与输入树重叠、软硬链接别名等非法场景下的原子性保护（不损坏源目录与已发布产物）；
4. 使用 `e2fsprogs` 制作多种规格（profile 与容量）的 ext4 镜像及可写测试夹具镜像时的内容一致性与失败隔离。
"""

import hashlib
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from host_helpers import TOOLS
import assemble_rootfs
from assemble_rootfs import assemble_rootfs_tree
from create_ext4_image import PROFILES, create_ext4_image
from ktest.ext4_fixtures import create_writable_fixture_image


def tree_contents(directory):
    """递归采集目录树中所有条目的权限位、纳秒级修改时间以及文件内容摘要（或符号链接目标）。

    采集过程使用 `lstat` 并仅统计权限与修改时间（`st_mtime_ns`），避免因读取文件导致访问时间（`atime`）变化而被误判为被修改。

    参数:
        directory (Path): 要扫描的根目录路径。

    返回:
        dict[str, tuple[int, int, str | bytes | None]]: 以相对路径字符串为键、`(权限位, 修改时间纳秒, 内容SHA256或链接目标)` 为值的字典。
    """
    result = {}
    for path in sorted(directory.rglob("*")):
        metadata = path.lstat()
        if path.is_symlink():
            content = os.readlink(path)
        elif path.is_file():
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                while chunk := stream.read(1024 * 1024):
                    digest.update(chunk)
            content = digest.digest()
        else:
            content = None
        result[str(path.relative_to(directory))] = (
            stat.S_IMODE(metadata.st_mode), metadata.st_mtime_ns, content,
        )
    return result


class RootfsAssemblyTests(unittest.TestCase):
    """验证 `assemble_rootfs_tree` 从用户态安装树、运行库与系统配置装配根文件系统目录树的行为与异常隔离。"""

    def setUp(self):
        """在临时目录中构造模拟的用户态安装树、musl/glibc 运行库及仓库系统配置目录。"""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.project = self.base / "project"
        shutil.copytree(TOOLS.parent / "userland/system", self.project / "userland/system", symlinks=True)
        shutil.copytree(TOOLS.parent / "userland/contest", self.project / "userland/contest", symlinks=True)
        self.userland = self.base / "installed guest"
        self.runtime = self.base / "installed runtime"
        self.output = self.base / "assembled root"
        for name in ("bin/busybox", "sbin/contest-run", "bin/libc-smoke", "usr/share/installed-data"):
            path = self.userland / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"installed:" + name.encode())
            path.chmod(0o755 if "share" not in name else 0o640)
        (self.userland / "bin/sh").symlink_to("busybox")
        (self.userland / "sbin/init").symlink_to("../bin/busybox")
        for family, names in (("musl", ("libc.so",)), ("glibc", ("libc.so.6", "libm.so.6"))):
            for name in names:
                path = self.runtime / family / "lib" / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(f"{family}:{name}".encode())
        (self.runtime / "musl/lib/loader.so").symlink_to("libc.so")
        self.root_patch = patch.object(assemble_rootfs, "REPOSITORY_ROOT", self.project)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)

    def assemble(self, architecture="riscv64", init_config=None):
        """调用 `assemble_rootfs_tree` 将当前测试夹具中的输入装配到 `self.output` 目录。"""
        return assemble_rootfs_tree(architecture, self.userland, self.runtime,
                                    self.output, init_config)

    def test_installed_tree_libraries_system_assets_and_full_configuration_are_copied(self):
        """验证用户态程序、符号链接、musl/glibc 运行库、系统启动配置及权限位均完整拷贝到目标根目录，且源目录不被修改。"""
        # 记录装配前所有输入目录树的快照，以便验证装配过程只读不改源
        inputs = tuple(tree_contents(path) for path in (self.userland, self.runtime, self.project))
        self.assemble()
        self.assertEqual((self.output / "usr/share/installed-data").read_bytes(),
                         b"installed:usr/share/installed-data")
        self.assertEqual(stat.S_IMODE((self.output / "usr/share/installed-data").stat().st_mode), 0o640)
        self.assertEqual(os.readlink(self.output / "bin/sh"), "busybox")
        self.assertEqual(os.readlink(self.output / "sbin/init"), "../bin/busybox")
        self.assertEqual(os.readlink(self.output / "opt/runtime/musl/lib/loader.so"), "libc.so")
        self.assertEqual((self.output / "opt/runtime/glibc/lib/libm.so.6").read_bytes(), b"glibc:libm.so.6")
        for relative in ("etc/inittab", "sbin/start-session"):
            source = self.project / "userland/system" / relative
            destination = self.output / relative
            self.assertEqual(destination.read_bytes(), source.read_bytes())
            self.assertEqual(stat.S_IMODE(destination.stat().st_mode), stat.S_IMODE(source.stat().st_mode))
        self.assertTrue(os.access(self.output / "sbin/start-session", os.X_OK))
        self.assertEqual(tree_contents(self.output / "etc/contest"),
                         tree_contents(self.project / "userland/contest"))
        self.assertEqual(stat.S_IMODE((self.output / "tmp").stat().st_mode), 0o1777)
        self.assertEqual(inputs, tuple(tree_contents(path) for path in (self.userland, self.runtime, self.project)))

    def test_reassembly_reflects_deletions_and_resets_explicit_init_override(self):
        """验证重新装配时会清理已删除的源文件与残留文件，并在未指定自定义 `init_config` 时恢复默认 `inittab`。"""
        # 首次装配：带入一个待删配置并使用自定义 inittab 覆盖默认配置
        removed = self.project / "userland/contest/deleted.toml"
        removed.write_text("old = true\n", encoding="utf-8")
        custom = self.base / "custom inittab"
        custom.write_text("::once:/bin/sh\n", encoding="utf-8")
        self.assemble(init_config=custom)
        self.assertEqual((self.output / "etc/inittab").read_bytes(), custom.read_bytes())
        # 删除旧文件、新增安装文件并在输出目录植入多余文件后重新执行默认装配
        removed.unlink()
        (self.userland / "usr/share/installed-data").unlink()
        (self.userland / "usr/share/new-data").write_bytes(b"new installation")
        (self.output / "stale-output-only").touch()
        self.assemble("loongarch64")
        self.assertFalse((self.output / "etc/contest/deleted.toml").exists())
        self.assertFalse((self.output / "usr/share/installed-data").exists())
        self.assertFalse((self.output / "stale-output-only").exists())
        self.assertEqual((self.output / "usr/share/new-data").read_bytes(), b"new installation")
        self.assertEqual((self.output / "etc/inittab").read_bytes(),
                         (self.project / "userland/system/etc/inittab").read_bytes())
        self.assertEqual(custom.read_text(encoding="utf-8"), "::once:/bin/sh\n")

    def test_missing_input_and_overlapping_output_leave_published_tree_unchanged(self):
        """验证缺少必需源文件、复制中途失败或输出目录与输入目录重叠时，已发布的根目录树和源文件保持原状。"""
        # 先完成一次正常发布并保存输出目录快照
        self.assemble()
        previous = tree_contents(self.output)
        # 缺失运行库文件或系统目录时应抛出异常，且已发布的输出目录不被破坏
        (self.runtime / "musl/lib/libc.so").unlink()
        with self.assertRaises(ValueError):
            self.assemble()
        self.assertEqual(tree_contents(self.output), previous)
        (self.runtime / "musl/lib/libc.so").write_bytes(b"restored source")
        (self.project / "userland/system/sbin").rename(self.project / "userland/system/missing-sbin")
        with self.assertRaises(OSError):
            self.assemble()
        self.assertEqual(tree_contents(self.output), previous)
        # 输出路径指向输入目录本身、其子目录或包含输入目录的父目录时，应直接拒绝
        for output in (self.userland, self.userland / "nested", self.base):
            with self.subTest(output=output), self.assertRaises(ValueError):
                assemble_rootfs_tree("riscv64", self.userland, self.runtime, output)
        self.assertEqual((self.runtime / "musl/lib/libc.so").read_bytes(), b"restored source")

    def test_existing_output_link_does_not_replace_its_source_directory(self):
        """验证当输出路径本身是指向外部目录的符号链接时，装配操作报错拒绝且不修改外部目录内容。"""
        external = self.base / "external root"
        external.mkdir()
        (external / "preserved").write_bytes(b"external content")
        self.output.symlink_to(external, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.assemble()
        self.assertTrue(self.output.is_symlink())
        self.assertEqual((external / "preserved").read_bytes(), b"external content")

    def test_symlink_workspace_parent_publishes_real_tree_and_rejects_input_alias(self):
        """验证通过符号链接父目录发布合法输出路径可以成功，但通过符号链接别名指向输入目录内部时仍会被识别并拒绝。"""
        # 父目录是普通工作区符号链接时，应正常解析并完成根目录树发布
        workspace = self.base / "workspace"
        workspace.symlink_to(self.base, target_is_directory=True)
        self.output = workspace / "published root"
        before = tuple(tree_contents(path) for path in (self.userland, self.runtime))
        self.assemble()
        self.assertEqual((self.base / "published root/bin/libc-smoke").read_bytes(),
                         (self.userland / "bin/libc-smoke").read_bytes())
        self.assertEqual(before, tuple(tree_contents(path) for path in (self.userland, self.runtime)))
        # 父目录是指向输入目录的符号链接时，应按真实路径识别出目录重叠并拒绝写入
        alias = self.base / "input-alias"
        alias.symlink_to(self.userland, target_is_directory=True)
        with self.assertRaises(ValueError):
            assemble_rootfs_tree("riscv64", self.userland, self.runtime, alias / "nested")
        self.assertFalse((self.userland / "nested").exists())


@unittest.skipUnless(all(shutil.which(binary) for binary in ("mke2fs", "dumpe2fs", "debugfs", "e2fsck")),
                     "requires e2fsprogs")
class Ext4ImageTests(unittest.TestCase):
    """验证 `create_ext4_image` 与 `create_writable_fixture_image` 调用真实 `e2fsprogs` 制作 ext4 镜像的内容准确性与异常隔离。"""

    def read_image_file(self, image, guest_path, destination):
        """调用 `debugfs` 的 `dump` 命令从 ext4 镜像中导出指定客体路径的文件内容并返回字节串。"""
        subprocess.run(["debugfs", "-R", f"dump {guest_path} {destination}", str(image)],
                       capture_output=True, check=True)
        return destination.read_bytes()

    def test_external_tree_cli_creates_ext4_without_build_tools_and_preserves_source(self):
        """验证在仅包含 `e2fsprogs` 工具的 `PATH` 环境下，CLI 可为所有预设规格（profile）生成包含正确文件内容与日志的 ext4 镜像。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "external complete tree"
            source.mkdir()
            (source / "payload").write_bytes(b"external root bytes\x00\xff")
            (source / "payload").chmod(0o640)
            (source / "alias").hardlink_to(source / "payload")
            (source / "symbolic").symlink_to("payload")
            before = tree_contents(source)
            # 构造仅包含 e2fsprogs 四个核心工具的隔离 PATH 目录，验证不依赖其他宿主构建工具
            binaries = base / "e2fsprogs-only"
            binaries.mkdir()
            for binary in ("mke2fs", "dumpe2fs", "debugfs", "e2fsck"):
                (binaries / binary).symlink_to(shutil.which(binary))
            for profile in PROFILES:
                with self.subTest(profile=profile):
                    image = base / f"{profile}.ext4"
                    result = subprocess.run([
                        sys.executable, str(TOOLS / "create_ext4_image.py"),
                        "--source-tree", str(source), "--output-image", str(image),
                        "--profile", profile, "--size-mib", "128",
                    ], capture_output=True, text=True, env={**os.environ, "PATH": str(binaries)})
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertEqual(self.read_image_file(image, "/payload", base / f"{profile}.dump"),
                                     b"external root bytes\x00\xff")
                    self.assertTrue(image.with_suffix(".log").is_file())
                    self.assertEqual(tree_contents(source), before)

    def test_repeated_image_creation_removes_deleted_content_and_replaces_old_bytes(self):
        """验证向同一输出路径重复制作 ext4 镜像时，新镜像准确反映源目录的文件增删与修改，不会残留旧镜像内容。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            source.mkdir()
            (source / "removed").write_bytes(b"old file")
            (source / "current").write_bytes(b"old bytes")
            image = base / "root.ext4"
            create_ext4_image(source, image, image_size_mib=128)
            (source / "removed").unlink()
            (source / "current").write_bytes(b"new bytes")
            before = tree_contents(source)
            create_ext4_image(source, image, image_size_mib=128)
            self.assertEqual(self.read_image_file(image, "/current", base / "current.dump"), b"new bytes")
            missing = base / "removed.dump"
            subprocess.run(["debugfs", "-R", f"dump /removed {missing}", str(image)],
                           capture_output=True, check=True)
            self.assertFalse(missing.exists())
            self.assertEqual(tree_contents(source), before)

    def test_small_volumes_and_symlink_workspace_keep_real_payload(self):
        """验证不同 profile 在 8/16/32 MiB 小容量配置及符号链接工作区下均能生成大小精确且内容完整的 ext4 镜像。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            source.mkdir()
            payload = b"small-volume bytes\x00\xff" * 257
            (source / "payload").write_bytes(payload)
            workspace = base / "workspace"
            workspace.symlink_to(base, target_is_directory=True)
            before = tree_contents(source)
            for profile in PROFILES:
                for size in (8, 16, 32):
                    with self.subTest(profile=profile, size=size):
                        image = workspace / f"{profile}-{size}.ext4"
                        create_ext4_image(source, image, profile, size)
                        self.assertEqual(image.stat().st_size, size * 1024 * 1024)
                        self.assertEqual(self.read_image_file(
                            image, "/payload", base / "payload.dump"), payload)
            self.assertEqual(tree_contents(source), before)

    def test_physical_source_alias_and_image_log_hardlink_preserve_old_bytes(self):
        """验证当输出路径通过符号链接落入源目录内部，或镜像文件与 `.log` 日志文件互为硬链接时，制盘在写入前报错并保护原文件。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            source.mkdir()
            (source / "payload").write_bytes(b"input")
            alias = base / "source-alias"
            alias.symlink_to(source, target_is_directory=True)
            before = tree_contents(source)
            with self.assertRaises(ValueError):
                create_ext4_image(source, alias / "nested.ext4", image_size_mib=8)
            image = base / "old.ext4"
            image.write_bytes(b"previous volume")
            image.with_suffix(".log").hardlink_to(image)
            with self.assertRaises(ValueError):
                create_ext4_image(source, image, image_size_mib=8)
            self.assertEqual(image.read_bytes(), b"previous volume")
            self.assertEqual(tree_contents(source), before)

    def test_source_file_hardlink_and_nonpositive_sizes_preserve_inputs_and_old_volume(self):
        """验证当输出镜像与源文件互为硬链接，或传入非正数容量（0 或负数）时，制盘报错拒绝且不破坏源文件与已有镜像。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            source.mkdir()
            payload = source / "payload"
            payload.write_bytes(b"input bytes")
            image = base / "root.ext4"
            image.hardlink_to(payload)
            before = tree_contents(source)
            with self.assertRaises(ValueError):
                create_ext4_image(source, image, image_size_mib=8)
            self.assertEqual(image.read_bytes(), b"input bytes")
            self.assertEqual(tree_contents(source), before)
            image.unlink()
            image.write_bytes(b"old volume")
            for size in (0, -1):
                with self.subTest(size=size), self.assertRaises(ValueError):
                    create_ext4_image(source, image, image_size_mib=size)
                self.assertEqual(image.read_bytes(), b"old volume")
            self.assertEqual(tree_contents(source), before)

    def test_invalid_outputs_and_generation_failure_preserve_source_and_previous_image(self):
        """验证当输出镜像或日志路径为符号链接、位于源目录内部，或因源文件过大导致容量不足失败时，源目录和已有镜像均保持不变。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            source.mkdir()
            (source / "payload").write_bytes(b"source content")
            image = base / "root.ext4"
            image.write_bytes(b"previous image")
            alias = base / "alias.ext4"
            alias.symlink_to(source / "payload")
            log_alias = image.with_suffix(".log")
            for output in (alias, source / "inside.ext4"):
                with self.subTest(output=output), self.assertRaises(ValueError):
                    create_ext4_image(source, output, image_size_mib=128)
            log_alias.symlink_to(source / "payload")
            with self.assertRaises(ValueError):
                create_ext4_image(source, image, image_size_mib=128)
            log_alias.unlink()
            # 写入超过 128 MiB 镜像容量的文件，验证 e2fsprogs 报错时不会覆盖已有的旧镜像
            with (source / "too-large").open("wb") as stream:
                for _ in range(140):
                    stream.write(b"x" * (1024 * 1024))
            before = tree_contents(source)
            with self.assertRaises(subprocess.CalledProcessError):
                create_ext4_image(source, image, image_size_mib=128)
            self.assertEqual(image.read_bytes(), b"previous image")
            self.assertEqual(tree_contents(source), before)
            self.assertTrue(log_alias.is_file())

    def test_writable_fixture_installs_only_in_private_copy(self):
        """验证 `create_writable_fixture_image` 在私有副本中安装测试程序、预映射数据文件及未写入预分配区，且不修改源目录。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            source.mkdir()
            (source / "payload").write_bytes(b"source content")
            program = base / "user-write"
            program.write_bytes(b"already built program")
            program.chmod(0o755)
            before = tree_contents(source)
            image = base / "writable.ext4"
            create_writable_fixture_image(source, program, image, image_size_mib=128)
            self.assertEqual(self.read_image_file(image, "/test/bin/user-write", base / "program.dump"),
                             program.read_bytes())
            self.assertEqual(self.read_image_file(image, "/test/write-map", base / "map.dump"),
                             bytes(range(256)) * 32)
            self.assertEqual(self.read_image_file(image, "/test/write-unwritten", base / "unwritten.dump"),
                             b"\0" * 16384)
            self.assertEqual(tree_contents(source), before)
            self.assertEqual(program.read_bytes(), b"already built program")

    def test_writable_fixture_rejects_links_before_they_can_write_outside_private_tree(self):
        """验证当源目录中的测试写入路径是指向外部的悬空符号链接时，夹具制作在写入前报错拒绝，防止越界创建外部文件。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            (source / "test").mkdir(parents=True)
            external = base / "external-file"
            (source / "test/write-kernel").symlink_to(external)
            program = base / "user-write"
            program.write_bytes(b"existing program")
            before = tree_contents(source)
            image = base / "writable.ext4"
            with self.assertRaises(ValueError):
                create_writable_fixture_image(source, program, image, image_size_mib=128)
            self.assertFalse(external.exists())
            self.assertFalse(image.exists())
            self.assertEqual(tree_contents(source), before)


if __name__ == "__main__":
    unittest.main()
