"""宿主侧 Make 工具链选择回归测试。

本模块验证仓库顶层 Makefile 在构建用户态自测程序时，能够根据目标架构（`ARCH=riscv64`
或 `ARCH=loongarch64`）正确选取对应的交叉编译前缀（`MUSL_CROSS_RV` / `MUSL_CROSS_LA`），
并支持通过命令行显式传入 `MUSL_CROSS` 覆盖默认前缀。
运行方式：`python3 -m unittest discover -s tools/ktest`。
"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPOSITORY = Path(__file__).resolve().parents[2]


class MakeToolchainTests(unittest.TestCase):
    """验证不同架构的默认工具链前缀与命令行 `MUSL_CROSS` 覆盖行为互不干扰。"""

    def test_arch_prefix_and_explicit_override(self):
        """验证按架构配置的环境变量前缀与命令行显式覆盖均可成功编译并运行自测程序。"""
        compiler = shutil.which("gcc")
        if not shutil.which("make") or not compiler:
            self.skipTest("make and native gcc are required")
        native_prefix = str(Path(compiler).parent) + "/"

        with tempfile.TemporaryDirectory() as temporary:
            # 分别针对 riscv64 和 loongarch64 测试各自对应的环境变量：
            # 将非当前架构的交叉前缀设为不存在的路径，验证各架构只读取自身对应的变量。
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
                        # 清理宿主环境中可能残留的工具链与 Make 标志，构造纯净的测试环境
                        environment = os.environ.copy()
                        for name in ("MUSL_CROSS", "MUSL_CROSS_RV", "MUSL_CROSS_LA",
                                     "MAKEFLAGS", "MFLAGS"):
                            environment.pop(name, None)
                        environment["MUSL_CROSS_RV"] = "/nonexistent/rv-"
                        environment["MUSL_CROSS_LA"] = "/nonexistent/la-"
                        environment[variable] = selected_prefix

                        # 调用顶层 Makefile 编译用户态自测程序，并验证编译产物可正常执行且输出符合预期
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
