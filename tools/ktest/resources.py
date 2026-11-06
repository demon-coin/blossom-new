"""ktest 宿主测试资源校验、工作盘准备、CRC32C 计算与 QEMU 设备参数构建工具。

本模块负责在启动测试会话前校验套件与用例声明的外部资源（`root`、`extra`、`scratch`、
`uart`、`ap-freeze`、`virtio-access-platform`、`net-user`），为每台 QEMU 虚拟机复制独占的
工作盘镜像或创建指定大小的空白 scratch 盘，并提供 UART 负载校验所需的 Castagnoli CRC32C
查表计算，以及测试专用的 VirtIO 网卡和块设备 QEMU 参数拼装函数。
"""

import hashlib
from pathlib import Path
import shutil
import stat
from qemu_command import (build_block_backend_arguments,
                          build_user_network_backend_arguments)

from .catalog import require

RESOURCES = {"uart", "scratch", "root", "extra", "ap-freeze", "virtio-access-platform", "net-user"}


def aliases(path, target):
    """判断两个路径在解析符号链接后是否指向同一位置，或是否为指向同一底层文件的硬链接。"""
    path, target = Path(path), Path(target)
    return path.resolve() == target.resolve() or (path.exists() and target.exists() and path.samefile(target))


def regular_file(path):
    """校验给定路径存在且为普通文件（拒绝目录、设备节点、FIFO 或套接字）。"""
    require(path.is_file() and stat.S_ISREG(path.stat().st_mode), f"not a regular file: {path}")


def hash_file(path):
    """按 1MiB 分块流式计算并返回文件的十六进制 SHA-256 摘要，避免将整个大文件一次性读入内存。"""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while block := source.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def resource_names(suite, items):
    """汇总测试套件与所有选中用例声明的资源名称集合，并校验不存在未知资源名。"""
    names = set(suite["resources"])
    for item in items:
        names.update(item.case["resources"])
    require(names <= RESOURCES, f"unsupported resources: {sorted(names - RESOURCES)}")
    return names


def crc_table():
    """生成 Castagnoli CRC32C（反射多项式 0x82F63B78）的 256 项字节查表。"""
    table = []
    for byte in range(256):
        value = byte
        for _ in range(8):
            value = (value >> 1) ^ (0x82F63B78 if value & 1 else 0)
        table.append(value)
    return tuple(table)


CRC_TABLE = crc_table()


def crc32c_update(state, data):
    """使用 `CRC_TABLE` 对一段字节流增量更新 CRC32C 中间状态（调用方需传入全 1 初值并在结束时按位取反）。"""
    for byte in data:
        state = CRC_TABLE[(state ^ byte) & 255] ^ (state >> 8)
    return state


def validate_resources(names, args):
    """校验资源集合所需的命令行参数、外部工具（如 GDB）及输入镜像/负载文件是否齐备有效。"""
    if "ap-freeze" in names:
        require(args.ap_freeze, "AP freeze resource requires --ap-freeze")
        require(shutil.which(args.gdb or "gdb") is not None, "AP freeze requires an available GDB binary")
    if "virtio-access-platform" in names:
        require(args.iommu_platform, "VirtIO feature fixture requires --iommu-platform")
        require(args.arch != "riscv64" or args.modern, "VirtIO feature fixture requires modern MMIO")
    for name in ("root", "extra"):
        if name in names:
            require(name in args.disks, f"resource {name} requires --disk {name}=PATH")
    for name in names & args.disks.keys():
        regular_file(args.disks[name])
    if "uart" in names:
        require(args.uart_payload is not None or args.uart_expect is not None
                or getattr(args, "execution_uart", False),
                "UART resource requires --uart-payload, --uart-expect or --execution-uart")
    if "uart" in names and args.uart_payload is not None:
        regular_file(args.uart_payload)
    if "uart" in names and args.uart_expect is not None:
        regular_file(args.uart_expect)


def prepare_disks(names, args, directory):
    """校验资源并按 `root`、`extra`、`scratch` 的固定顺序在临时目录中复制源盘或创建空白稀疏工作盘。"""
    validate_resources(names, args)
    disks = []
    # 按固定角色顺序在会话独占目录中创建工作盘：有源盘则流式复制，无源盘的 scratch 则按指定字节数截断生成空白盘
    for name in ("root", "extra", "scratch"):
        if name not in names:
            continue
        target = directory / f"{name}.raw"
        require(not target.exists() and not target.is_symlink(), "working disk already exists")
        with target.open("xb") as output:
            if name in args.disks:
                with args.disks[name].open("rb") as source:
                    shutil.copyfileobj(source, output, 1024 * 1024)
            else:
                require(name == "scratch", "missing source disk")
                output.truncate(args.scratch_bytes)
        disks.append((name, target))
    return disks



def build_test_network_arguments(architecture, network_mode, host_forward_rule):
    """构建测试会话的 QEMU VirtIO 网卡参数（固定绑定槽位 3 与测试 MAC 地址），或返回禁用网络参数。"""
    if network_mode == "none":
        if host_forward_rule:
            raise ValueError("host forwarding requires user networking")
        return ["-nic", "none"]
    if network_mode != "user":
        raise ValueError(f"unsupported network mode: {network_mode}")
    if architecture == "riscv64":
        device = "virtio-net-device,netdev=net0,bus=virtio-mmio-bus.3"
    elif architecture == "loongarch64":
        device = "virtio-net-pci,netdev=net0,addr=0x11,disable-legacy=on"
    else:
        raise ValueError(f"unsupported network architecture: {architecture}")
    return (build_user_network_backend_arguments("net0", host_forward_rule)
            + ["-device", device + ",mac=52:54:00:12:34:56"])


def build_test_disk_arguments(disks, *, architecture, read_only_roles,
                              logical_block_size, virtio_access_platform,
                              root_device, root_mode, extra_root_device,
                              network_peer_ports=None):
    """根据已准备的工作盘列表、目标架构、块大小、IOMMU 平台标志及宿主网络对端端口，构建 QEMU 块设备与 `-append` 内核启动参数。"""
    command = []
    # 为每块工作盘分配 VirtIO 槽位（跳过被测试网卡占用的槽位 3），并拼装块设备后端与前端参数
    for index, (role, path) in enumerate(disks):
        slot = index + int(3 <= index)
        drive_id = f"disk{index}"
        command += build_block_backend_arguments(drive_id, path, role in read_only_roles)
        if architecture == "riscv64":
            device = f"virtio-blk-device,drive={drive_id},bus=virtio-mmio-bus.{slot}"
        elif architecture == "loongarch64":
            device = f"virtio-blk-pci,drive={drive_id},addr=0x{5 + 4 * slot:x}"
        else:
            raise ValueError(f"unsupported disk architecture: {architecture}")
        device += f",logical_block_size={logical_block_size},physical_block_size={logical_block_size}"
        if virtio_access_platform:
            device += ",iommu_platform=on"
            if architecture == "loongarch64":
                device += ",disable-legacy=on"
        command += ["-device", device]
    # 组装内核命令行参数（根文件系统模式、root/extra 设备路径，以及宿主网络对端的 TCP/UDP 端口号）
    if root_device is None:
        index = next((index for index, (role, _) in enumerate(disks) if role == "root"), None)
        if index is not None:
            root_device = f"/dev/vd{chr(ord('a') + index)}"
    bootargs = [root_mode, "rootdir=/"]
    if root_device is not None:
        bootargs.append(f"root={root_device}")
    if extra_root_device is not None:
        bootargs.append(f"extra={extra_root_device}")
    if network_peer_ports is not None:
        tcp, udp = network_peer_ports
        if not (1 <= tcp <= 65535 and 1 <= udp <= 65535):
            raise ValueError("network peer ports must be in 1..65535")
        bootargs += [f"ktest-net-tcp={tcp}", f"ktest-net-udp={udp}"]
    return command + ["-append", " ".join(bootargs)]
