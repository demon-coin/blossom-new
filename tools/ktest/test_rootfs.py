"""根树装配、真实 ext4 内容与会话选择的宿主行为回归。"""

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
    """记录源树的实际字节、链接、权限与修改时间，不把读取导致的 atime 当写入。"""
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
    """从安装产物到发布目录的真实内容与失败隔离。"""

    def setUp(self):
        """构造可独立增删的安装输入，系统资产使用仓库的真实普通文件。"""
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
        """通过公开装配接口发布本例输入。"""
        return assemble_rootfs_tree(architecture, self.userland, self.runtime,
                                    self.output, init_config)

    def test_installed_tree_libraries_system_assets_and_full_configuration_are_copied(self):
        """非固定目录资产、两族库、普通 init 与可执行会话资产都进入单根。"""
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
        """每次重建拥有的输出，删除的配置、程序和上次 init 覆盖不会残留。"""
        removed = self.project / "userland/contest/deleted.toml"
        removed.write_text("old = true\n", encoding="utf-8")
        custom = self.base / "custom inittab"
        custom.write_text("::once:/bin/sh\n", encoding="utf-8")
        self.assemble(init_config=custom)
        self.assertEqual((self.output / "etc/inittab").read_bytes(), custom.read_bytes())
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
        """源缺失、装配中复制失败与源输出重叠都不能覆盖既有发布或改写输入。"""
        self.assemble()
        previous = tree_contents(self.output)
        (self.runtime / "musl/lib/libc.so").unlink()
        with self.assertRaises(ValueError):
            self.assemble()
        self.assertEqual(tree_contents(self.output), previous)
        (self.runtime / "musl/lib/libc.so").write_bytes(b"restored source")
        (self.project / "userland/system/sbin").rename(self.project / "userland/system/missing-sbin")
        with self.assertRaises(OSError):
            self.assemble()
        self.assertEqual(tree_contents(self.output), previous)
        for output in (self.userland, self.userland / "nested", self.base):
            with self.subTest(output=output), self.assertRaises(ValueError):
                assemble_rootfs_tree("riscv64", self.userland, self.runtime, output)
        self.assertEqual((self.runtime / "musl/lib/libc.so").read_bytes(), b"restored source")

    def test_existing_output_link_does_not_replace_its_source_directory(self):
        """发布路径不能借已有链接指向外部完整树。"""
        external = self.base / "external root"
        external.mkdir()
        (external / "preserved").write_bytes(b"external content")
        self.output.symlink_to(external, target_is_directory=True)
        with self.assertRaises(ValueError):
            self.assemble()
        self.assertTrue(self.output.is_symlink())
        self.assertEqual((external / "preserved").read_bytes(), b"external content")

    def test_symlink_workspace_parent_publishes_real_tree_and_rejects_input_alias(self):
        """普通链接父目录可发布，指向输入的父目录仍按实际路径拒绝。"""
        workspace = self.base / "workspace"
        workspace.symlink_to(self.base, target_is_directory=True)
        self.output = workspace / "published root"
        before = tuple(tree_contents(path) for path in (self.userland, self.runtime))
        self.assemble()
        self.assertEqual((self.base / "published root/bin/libc-smoke").read_bytes(),
                         (self.userland / "bin/libc-smoke").read_bytes())
        self.assertEqual(before, tuple(tree_contents(path) for path in (self.userland, self.runtime)))
        alias = self.base / "input-alias"
        alias.symlink_to(self.userland, target_is_directory=True)
        with self.assertRaises(ValueError):
            assemble_rootfs_tree("riscv64", self.userland, self.runtime, alias / "nested")
        self.assertFalse((self.userland / "nested").exists())


@unittest.skipUnless(all(shutil.which(binary) for binary in ("mke2fs", "dumpe2fs", "debugfs", "e2fsck")),
                     "requires e2fsprogs")
class Ext4ImageTests(unittest.TestCase):
    """真实 e2fsprogs 的完整树输入、卷内内容和输出失败隔离。"""

    def read_image_file(self, image, guest_path, destination):
        """使用真实 debugfs 导出客体文件，缺文件或工具失败直接使断言失败。"""
        subprocess.run(["debugfs", "-R", f"dump {guest_path} {destination}", str(image)],
                       capture_output=True, check=True)
        return destination.read_bytes()

    def test_external_tree_cli_creates_ext4_without_build_tools_and_preserves_source(self):
        """只有 e2fsprogs 的 PATH 仍能直接制卷，root.ext4 与所有原几何均可消费。"""
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "external complete tree"
            source.mkdir()
            (source / "payload").write_bytes(b"external root bytes\x00\xff")
            (source / "payload").chmod(0o640)
            (source / "alias").hardlink_to(source / "payload")
            (source / "symbolic").symlink_to("payload")
            before = tree_contents(source)
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
        """同一输出的新卷反映完整树的增删，不沿用上次镜像内容。"""
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
        """可容纳输入的三种小卷由真实工具决定，父目录链接不阻止发布。"""
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
        """实际父目录重叠与卷日志同 inode 在写临时卷前拒绝。"""
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
        """输出不得借硬链接别名源文件，无效容量不得覆盖已发布卷。"""
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
        """链接输出、源内输出与真实容量不足都不写坏来源或已发布镜像。"""
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
        """真实可写预分配夹具包含程序和未写区，原完整树与程序字节均保持不变。"""
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
        """输入中的悬空写路径保持悬空，失败不创建外部文件或发布夹具。"""
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
