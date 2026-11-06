"""QEMU 命令行参数构建器的单元测试模块。

本模块在不启动真实 QEMU 子进程的前提下，直接调用 `qemu_command` 与 `ktest.resources`
中的纯参数构建函数，验证以下核心行为：
1. 生产环境默认评测设备布局（比赛盘、网卡、系统盘的挂载顺序与架构特定参数）；
2. 调用方显式指定的内核 ELF、工作盘、CPU、内存与网络配置覆盖；
3. 单盘模式、内核启动参数（`-append`）与 GDB 调试暂停选项之间的正交组合；
4. 含空格、引号、非 ASCII 字符及逗号的路径与主机端口转发规则的 QEMU 字段转义；
5. ktest 内核测试专用的多磁盘槽位分配、只读控制、块大小、IOMMU 属性与网络对端端口参数。
"""

from pathlib import Path
import shlex
import unittest

from host_helpers import TOOLS
from qemu_command import (build_block_backend_arguments, build_qemu_command,
                          build_user_network_backend_arguments)
from ktest.resources import build_test_disk_arguments, build_test_network_arguments


def command_for(architecture, **overrides):
    """使用预设的默认运行参数调用 `build_qemu_command`，并允许按需覆盖指定字段。

    参数:
        architecture (str): 目标架构名称（如 `"riscv64"` 或 `"loongarch64"`）。
        **overrides: 需要覆盖默认值的 `build_qemu_command` 关键字参数。

    返回:
        list[str]: 生成的完整 QEMU 命令行参数列表。
    """
    arguments = dict(architecture=architecture, qemu_binary=f"qemu-system-{architecture}",
                     kernel_elf_path=Path("/selected/kernel.elf"),
                     system_working_disk_path=Path("/selected/root.raw"),
                     contest_working_disk_path=Path("/selected/contest.raw"),
                     cpu_count=3, memory_size="1536M", network_mode="user",
                     host_forward_rule=None, boot_arguments=None, debug_enabled=False,
                     gdb_port=23456)
    arguments.update(overrides)
    return build_qemu_command(**arguments)


def option_values(command, option):
    """从命令行参数列表中提取指定选项（如 `"-device"` 或 `"-drive"`）紧邻的参数值。

    参数:
        command (Sequence[str]): 完整的命令行参数序列。
        option (str): 要查找的命令行选项名称。

    返回:
        list[str]: 所有匹配选项后紧跟的参数值列表，按出现顺序排列。
    """
    return [command[index + 1] for index, value in enumerate(command) if value == option]


class QemuCommandTests(unittest.TestCase):
    """验证生产环境 QEMU 命令生成与 ktest 测试设备布局参数的单元测试集。"""

    def test_evaluation_devices_keep_role_order_and_architecture_contract(self):
        """验证评测模式下两套架构的设备顺序（比赛盘、网卡、系统盘）及默认架构参数约定。"""
        # 依次验证 riscv64 和 loongarch64 两种目标架构的默认评测设备布局
        for architecture in ("riscv64", "loongarch64"):
            with self.subTest(architecture=architecture):
                command = command_for(architecture)
                devices = option_values(command, "-device")
                drives = option_values(command, "-drive")
                model = "device" if architecture == "riscv64" else "pci"
                backend_id = "net" if architecture == "riscv64" else "net0"
                # 校验设备挂载顺序：比赛盘 (x0) 先于网卡，系统盘 (x1) 位于最后
                self.assertEqual(len(devices), 3)
                self.assertTrue(devices[0].startswith(f"virtio-blk-{model},drive=x0"))
                self.assertEqual(devices[1], f"virtio-net-{model},netdev={backend_id}")
                self.assertTrue(devices[2].startswith(f"virtio-blk-{model},drive=x1"))
                self.assertLess(command.index(drives[0]), command.index(devices[1]))
                self.assertLess(command.index(devices[1]), command.index(drives[1]))
                self.assertIn("id=x0,format=raw,file=/selected/contest.raw", drives[0])
                self.assertIn("id=x1,format=raw,file=/selected/root.raw", drives[1])
                self.assertEqual(option_values(command, "-netdev"), [f"user,id={backend_id}"])
                self.assertEqual(option_values(command, "-rtc"), ["base=utc"])
                # 校验通用运行标志，并确认默认不附加启动参数、GDB 调试或特定 CPU 型号
                self.assertIn("-nographic", command)
                self.assertIn("-no-reboot", command)
                self.assertNotIn("-append", command)
                self.assertNotIn("-gdb", command)
                self.assertNotIn("-cpu", command)
                # 确保生产环境设备不携带 ktest 测试专用的 MAC、总线地址、IOMMU 或块大小属性
                for device in devices:
                    for attribute in ("mac=", "addr=", "disable-legacy=", "iommu_platform=",
                                      "logical_block_size="):
                        self.assertNotIn(attribute, device)
                # 校验架构特定的机器类型、BIOS 固件与 virtio-mmio 总线槽位绑定
                if architecture == "riscv64":
                    self.assertEqual(option_values(command, "-machine"), ["virt"])
                    self.assertEqual(option_values(command, "-bios"), ["default"])
                    self.assertIn("bus=virtio-mmio-bus.0", devices[0])
                    self.assertIn("bus=virtio-mmio-bus.1", devices[2])
                else:
                    self.assertNotIn("-machine", command)
                    self.assertNotIn("-bios", command)

    def test_selected_inputs_are_used_without_replacing_other_roles(self):
        """验证显式指定的 QEMU 可执行文件、内核、工作盘、CPU、内存及网络模式能准确写入命令行。"""
        # 传入自定义的二进制路径、硬件规格、工作盘路径以及禁用网络的运行配置
        command = command_for("riscv64", qemu_binary="/chosen/qemu", cpu_count=5,
                              memory_size="2304M", kernel_elf_path="/chosen/kernel",
                              system_working_disk_path="/chosen/system",
                              contest_working_disk_path="/chosen/contest", network_mode="none")
        # 验证各项自定义参数准确映射到对应的 QEMU 选项，且各磁盘后端互不干扰
        self.assertEqual(command[0], "/chosen/qemu")
        self.assertEqual(option_values(command, "-kernel"), ["/chosen/kernel"])
        self.assertEqual(option_values(command, "-smp"), ["5"])
        self.assertEqual(option_values(command, "-m"), ["2304M"])
        self.assertEqual(option_values(command, "-nic"), ["none"])
        self.assertEqual(option_values(command, "-netdev"), [])
        self.assertEqual(option_values(command, "-drive"), [
            "if=none,id=x0,format=raw,file=/chosen/contest",
            "if=none,id=x1,format=raw,file=/chosen/system"])

    def test_single_disk_shell_append_and_debug_are_orthogonal(self):
        """验证单盘模式、内核启动参数（`-append`）与 GDB 调试选项彼此独立、互不隐式干扰。"""
        for architecture in ("riscv64", "loongarch64"):
            with self.subTest(architecture=architecture):
                # 验证传入内核启动参数（如进入交互式 Shell）与开启 GDB 调试时各自独立追加对应选项
                base = command_for(architecture)
                shell_arguments = "root=/dev/vda rw blossom.boot=shell"
                shell = command_for(architecture, boot_arguments=shell_arguments)
                self.assertEqual(shell, base + ["-append", shell_arguments])
                self.assertEqual(command_for(architecture, debug_enabled=True),
                                 base + ["-S", "-gdb", "tcp:127.0.0.1:23456"])
                self.assertEqual(command_for(architecture, boot_arguments=shell_arguments,
                                             debug_enabled=True),
                                 shell + ["-S", "-gdb", "tcp:127.0.0.1:23456"])
                # 验证省略比赛工作盘（单盘模式）时仅移除比赛盘，不自动注入启动参数或调试选项
                single = command_for(architecture, contest_working_disk_path=None)
                self.assertEqual(option_values(single, "-drive"), option_values(base, "-drive")[1:])
                self.assertEqual(option_values(single, "-netdev"), option_values(base, "-netdev"))
                self.assertEqual(option_values(single, "-kernel"), option_values(base, "-kernel"))
                self.assertNotIn("-append", single)
                self.assertNotIn("-gdb", single)
                self.assertEqual(command_for(architecture, boot_arguments="")[-2:], ["-append", ""])

    def test_special_paths_preserve_argv_and_escape_qemu_fields(self):
        """验证包含空格、引号、中文字符及逗号的路径与转发规则在 QEMU 参数中被正确转义且可无损还原。"""
        # 构造包含空格、单双引号、中文字符以及逗号的路径与端口转发规则
        kernel = "/selected/内核 'quoted', file.elf"
        disk = Path('/selected/工作盘 "quoted", volume.raw')
        forward = "tcp:127.0.0.1:43210-:4321,udp:127.0.0.1:43211-:4322"
        command = command_for("loongarch64", kernel_elf_path=kernel,
                              system_working_disk_path=disk, host_forward_rule=forward)
        # 验证 -kernel 参数原样保留，而 -drive 和 -netdev 字段内的逗号按 QEMU 语法双写转义
        self.assertEqual(option_values(command, "-kernel"), [kernel])
        self.assertEqual(option_values(command, "-drive")[1],
                         f"if=none,id=x1,format=raw,file={str(disk).replace(',', ',,')}")
        self.assertEqual(build_block_backend_arguments("chosen", disk, True),
                         ["-drive", f"if=none,id=chosen,format=raw,file={str(disk).replace(',', ',,')},readonly=on"])
        self.assertEqual(build_user_network_backend_arguments("chosen", forward),
                         ["-netdev", "user,id=chosen,hostfwd=" + forward.replace(",", ",,")])
        # 验证生成的参数列表经 shell 字符串拼接与再拆分后与原列表完全一致
        self.assertEqual(shlex.split(shlex.join(command)), command)

    def test_test_layout_keeps_slots_features_and_actual_peer_ports(self):
        """验证 ktest 测试磁盘与网卡的物理槽位分配、设备特性开关及内核命令行端口参数生成。"""
        disks = [(role, Path(f"/{role}.raw")) for role in ("extra", "root", "scratch", "fourth")]
        for architecture in ("riscv64", "loongarch64"):
            with self.subTest(architecture=architecture):
                # 构造包含 4 个角色的测试磁盘列表，启用 4K 块大小、IOMMU 平台属性与网络对端端口
                command = build_test_disk_arguments(
                    disks, architecture=architecture, read_only_roles={"root"},
                    logical_block_size=4096, virtio_access_platform=True,
                    root_device=None, root_mode="rw", extra_root_device="/dev/vda",
                    network_peer_ports=(32101, 32102))
                drives = option_values(command, "-drive")
                self.assertNotIn("readonly=on", drives[0])
                self.assertIn("readonly=on", drives[1])
                devices = option_values(command, "-device")
                # 第 4 块磁盘（索引 3）在物理槽位上需跳过预留给测试网卡的槽位（bus.3 或 0x11）
                slot = "bus=virtio-mmio-bus.4" if architecture == "riscv64" else "addr=0x15"
                self.assertIn(slot, devices[3])
                for device in devices:
                    self.assertIn("logical_block_size=4096,physical_block_size=4096", device)
                    self.assertIn("iommu_platform=on", device)
                    self.assertEqual("disable-legacy=on" in device, architecture == "loongarch64")
                self.assertEqual(option_values(command, "-append"), [
                    "rw rootdir=/ root=/dev/vdb extra=/dev/vda ktest-net-tcp=32101 ktest-net-udp=32102"])
                # 验证测试网卡使用固定 MAC 地址并绑定在预留的物理槽位上
                network = build_test_network_arguments(architecture, "user", None)
                nic = option_values(network, "-device")[0]
                self.assertIn("mac=52:54:00:12:34:56", nic)
                self.assertIn("bus=virtio-mmio-bus.3" if architecture == "riscv64" else "addr=0x11", nic)
                # 验证关闭 IOMMU 且显式指定根设备名与只读模式时，生成对应的精简参数
                plain = build_test_disk_arguments(
                    disks[:1], architecture=architecture, read_only_roles=set(),
                    logical_block_size=512, virtio_access_platform=False,
                    root_device="/dev/vdz", root_mode="ro", extra_root_device=None)
                self.assertNotIn("iommu_platform=on", " ".join(plain))
                self.assertEqual(option_values(plain, "-append"), ["ro rootdir=/ root=/dev/vdz"])
