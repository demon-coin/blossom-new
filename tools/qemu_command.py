"""QEMU 命令行参数构造模块。

本模块负责为生产运行（`run_qemu.py`）和自动化测试生成 QEMU 进程的参数列表（`argv`），
并提供块设备后端（`-drive`）与用户态网络后端（`-netdev`）的通用参数格式化函数。
所有函数均为纯内存字符串拼装，不检查宿主机文件系统，也不直接启动子进程。
"""


def build_block_backend_arguments(drive_id, image_path, read_only=False):
    """构造 QEMU 原始（raw）格式块设备后端的 `-drive` 参数对。

    参数：
    - `drive_id`：块设备后端 ID（如 `"x0"`、`"x1"`），供后续 `-device` 引用。
    - `image_path`：磁盘镜像文件路径；路径中的逗号会按 QEMU 语法转义为双逗号 `,,`。
    - `read_only`：是否以只读模式打开镜像文件。

    返回：形如 `["-drive", "if=none,id=...,format=raw,file=..."]` 的参数列表。
    """
    image_value = str(image_path).replace(",", ",,")
    backend = f"if=none,id={drive_id},format=raw,file={image_value}"
    if read_only:
        backend += ",readonly=on"
    return ["-drive", backend]


def build_user_network_backend_arguments(backend_id, host_forward_rule=None):
    """构造 QEMU 用户态网络（SLIRP）后端的 `-netdev` 参数对。

    参数：
    - `backend_id`：网络后端 ID（如 `"net"` 或 `"net0"`），供网卡 `-device` 引用。
    - `host_forward_rule`：可选的端口转发规则（`hostfwd=...` 的值），其中的逗号会按 QEMU 语法转义。

    返回：形如 `["-netdev", "user,id=..."]` 的参数列表。
    """
    backend = f"user,id={backend_id}"
    if host_forward_rule:
        backend += ",hostfwd=" + host_forward_rule.replace(",", ",,")
    return ["-netdev", backend]


def build_qemu_command(architecture, qemu_binary, kernel_elf_path,
                       system_working_disk_path, contest_working_disk_path,
                       cpu_count, memory_size, network_mode, host_forward_rule,
                       boot_arguments, debug_enabled, gdb_port):
    """按目标架构与评测硬件拓扑顺序构造完整的 QEMU 启动命令行。

    参数：
    - `architecture`：目标架构，支持 `"riscv64"` 或 `"loongarch64"`。
    - `qemu_binary`：QEMU 可执行文件名或路径。
    - `kernel_elf_path`：内核 ELF 镜像路径。
    - `system_working_disk_path`：系统根工作盘镜像路径（挂载在 `x1` 后端）。
    - `contest_working_disk_path`：可选的比赛测试工作盘路径（挂载在 `x0` 后端）；为 `None` 时仅挂载系统盘。
    - `cpu_count`：虚拟 CPU 核数（`-smp`）。
    - `memory_size`：客户机内存大小字符串（如 `"1G"`）。
    - `network_mode`：网络模式，支持 `"user"`（用户态网络）或 `"none"`（无网卡）。
    - `host_forward_rule`：可选的宿主机端口转发规则。
    - `boot_arguments`：可选的内核启动命令行（`-append`）；为 `None` 时不添加。
    - `debug_enabled`：是否开启启动前冻结并等待 GDB 连接（`-S -gdb`）。
    - `gdb_port`：GDB 远程调试监听的本地 TCP 端口。

    返回：可直接传给 `execve` 或 `subprocess` 的字符串参数列表。
    异常：当 `architecture` 或 `network_mode` 不受支持时抛出 `ValueError`。
    """
    if architecture not in ("riscv64", "loongarch64"):
        raise ValueError(f"unsupported QEMU architecture: {architecture}")

    # 组装基础机器类型、内核镜像、内存、无图形终端与多核参数。
    command = [str(qemu_binary)]
    if architecture == "riscv64":
        command += ["-machine", "virt"]
    command += ["-kernel", str(kernel_elf_path), "-m", memory_size,
                "-nographic", "-smp", str(cpu_count)]
    if architecture == "riscv64":
        command += ["-bios", "default"]

    # 若存在比赛测试盘，将其挂载为首个块设备（x0），保证双盘模式下比赛盘先于系统盘被探测。
    if contest_working_disk_path is not None:
        command += build_block_backend_arguments("x0", contest_working_disk_path)
        if architecture == "riscv64":
            command += ["-device", "virtio-blk-device,drive=x0,bus=virtio-mmio-bus.0"]
        else:
            command += ["-device", "virtio-blk-pci,drive=x0"]
    command += ["-no-reboot"]

    # 配置网络设备与后端：RISC-V 使用 VirtIO MMIO，LoongArch64 使用 VirtIO PCI。
    if network_mode == "none":
        command += ["-nic", "none"]
    elif network_mode == "user":
        backend_id = "net" if architecture == "riscv64" else "net0"
        device = "virtio-net-device" if architecture == "riscv64" else "virtio-net-pci"
        command += ["-device", f"{device},netdev={backend_id}"]
        command += build_user_network_backend_arguments(backend_id, host_forward_rule)
    else:
        raise ValueError(f"unsupported network mode: {network_mode}")

    # 配置 UTC 实时时钟，并挂载系统根工作盘（x1）。
    command += ["-rtc", "base=utc"]
    command += build_block_backend_arguments("x1", system_working_disk_path)
    if architecture == "riscv64":
        command += ["-device", "virtio-blk-device,drive=x1,bus=virtio-mmio-bus.1"]
    else:
        command += ["-device", "virtio-blk-pci,drive=x1"]

    # 按需附加内核启动参数与 GDB 调试等待选项。
    if boot_arguments is not None:
        command += ["-append", boot_arguments]
    if debug_enabled:
        command += ["-S", "-gdb", f"tcp:127.0.0.1:{gdb_port}"]
    return command
