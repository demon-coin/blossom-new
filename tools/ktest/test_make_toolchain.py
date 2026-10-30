"""验证 Make 的工具链选择能编译并运行真实用户程序。"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPOSITORY = Path(__file__).resolve().parents[2]


class MakeToolchainTests(unittest.TestCase):
    """单架构默认值与显式参数保持独立的工具选择。"""

    def test_arch_prefix_and_explicit_override(self):
        """按架构选择和命令行覆盖均可编译、执行实际文件自测。"""
        compiler = shutil.which("gcc")
        if not shutil.which("make") or not compiler:
            self.skipTest("make and native gcc are required")
        native_prefix = str(Path(compiler).parent) + "/"

        with tempfile.TemporaryDirectory() as temporary:
            for architecture, variable in (
                ("riscv64", "MUSL_CROSS_RV"),
                ("loongarch64", "MUSL_CROSS_LA"),
            ):
                cases = ((native_prefix, None), ("", None),
                         ("/nonexistent/selected-", native_prefix),
                         ("/nonexistent/selected-", ""))
                for index, (selected_prefix, override) in enumerate(cases):
                    with self.subTest(architecture=architecture, selected=selected_prefix,
                                      override=override):
                        environment = os.environ.copy()
                        for name in ("MUSL_CROSS", "MUSL_CROSS_RV", "MUSL_CROSS_LA",
                                     "MAKEFLAGS", "MFLAGS"):
                            environment.pop(name, None)
                        environment["MUSL_CROSS_RV"] = "/nonexistent/rv-"
                        environment["MUSL_CROSS_LA"] = "/nonexistent/la-"
                        environment[variable] = selected_prefix

                        output = Path(temporary) / architecture / str(index)
                        program = output / "test/selftests/fs/file_shared"
                        command = [
                            "make", f"ARCH={architecture}", "TARGET_PLATFORM=qemu-virt",
                            f"USER_SELFTEST_BUILD_DIR={output}",
                            "MUSL_CFLAGS=-std=c11 -O2 -Wall -Wextra",
                        ]
                        if override is not None:
                            command.append(f"MUSL_CROSS={override}")
                        command.append(str(program))
                        build = subprocess.run(command, cwd=REPOSITORY, env=environment,
                                               capture_output=True, text=True, timeout=60)
                        self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
                        result = subprocess.run([program], cwd=program.parent,
                                                capture_output=True, text=True, timeout=60)
                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                        self.assertEqual(sum(line.startswith("ok ") for line in result.stdout.splitlines()), 2)


if __name__ == "__main__":
    unittest.main()
