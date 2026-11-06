"""`ap-freeze` 专项测试支持模块：通过 GDB 将从核（AP）首条访存指令改写为死循环，并在主核超时后核对从核内存与状态未被破坏。"""

import json

from .catalog import require


def command(args, kernel, directory, endpoint):
    """生成当前虚拟机的 `ap-freeze.gdb` 批处理脚本并返回 GDB 启动命令；仅在运行期修改内存指令，不修改磁盘上的内核 ELF。"""
    configuration = dict(arch=args.arch, endpoint=endpoint,
                         evidence=str((directory / "ap-freeze.json").resolve()))
    script = directory / "ap-freeze.gdb"
    script.write_text("set pagination off\nset confirm off\npython\n" +
                      "configuration = " + repr(configuration) + "\n" + GDB_SCRIPT +
                      "\nend\n", encoding="utf-8")
    return [args.gdb or "gdb", "--batch", "--nx", "--quiet", str(kernel), "-x", str(script)]


def verify(directory):
    """在 GDB 会话结束后核对 `ap-freeze.json` 与串口日志，确认从核在冻结期间启动记录、启动栈与未上线状态均保持不变。"""
    with (directory / "ap-freeze.json").open(encoding="utf-8") as source:
        evidence = json.load(source)
    # 校验 GDB 脚本产出的证据文件字段完整，且确实捕获到了逻辑核 1 的启动记录。
    require(isinstance(evidence, dict), "AP freeze evidence must be an object")
    fields = ("logical_cpu", "record_physical", "record_before", "record_after",
              "stack_before_sha256", "stack_after_sha256", "online_before", "online_after",
              "instruction_after", "self_loop")
    require(all(field in evidence for field in fields), "AP freeze evidence is missing actual fields")
    require(evidence.get("complete") is True, "AP freeze evidence is incomplete")
    require(evidence["logical_cpu"] == 1 and evidence["record_physical"] != 0,
            "AP freeze did not observe the actual first AP record")
    # 校验从核在冻结期间未修改启动记录和启动栈内容，且始终处于未上线及死循环指令状态。
    require(evidence["record_before"] == evidence["record_after"], "AP boot record changed while frozen")
    require(evidence["stack_before_sha256"] == evidence["stack_after_sha256"],
            "AP boot stack changed while frozen")
    require(evidence["online_before"] == evidence["online_after"] == 0,
            "frozen AP was published online")
    require(evidence["instruction_after"] == evidence["self_loop"], "AP self-loop was not retained")
    # 确认主核因从核启动超时而终止，未进入正常生产态 READY 阶段。
    with (directory / "uart.log").open("rb") as source:
        require(not any(line.startswith(b"READY arch=") for line in source),
                "AP timeout VM reached production READY")


GDB_SCRIPT = r'''
import gdb
import hashlib
import json
import struct

# 所有读写都走本 VM 的实际物理地址，避免 BSP/AP 当前地址空间不同。
gdb.execute("set architecture " + ("riscv:rv64" if configuration["arch"] == "riscv64" else "Loongarch64"))
gdb.execute("set language c")
gdb.execute("target remote " + configuration["endpoint"])
gdb.execute("set schedule-multiple on")
reply = gdb.execute("maintenance packet Qqemu.PhyMemMode:1", to_string=True)
assert '"OK"' in reply, "QEMU does not support physical-memory GDB access"
inferior = gdb.selected_inferior()
direct_base = int(gdb.parse_and_eval("&__direct_base"))
kernel_virtual = int(gdb.parse_and_eval("&__kernel_virt_start"))
kernel_physical = int(gdb.parse_and_eval("&__kernel_phys_start"))
ram_base = int(gdb.parse_and_eval("&__phys_ram_base")) if configuration["arch"] == "riscv64" else 0
entry = int(gdb.parse_and_eval("&__secondary_entry_phys"))


def physical(address):
    """按当前 ELF 的真实 kernel 与 RAM 两类映射转换物理地址。"""
    if configuration["arch"] == "riscv64" and address >= kernel_virtual:
        return address - kernel_virtual + kernel_physical
    return address - direct_base + ram_base if address >= direct_base else address


def offset(name):
    """从本次 ELF 的真实 DWARF 布局取得 CpuLocal 字段偏移。"""
    return next(field.bitpos // 8 for field in gdb.lookup_type("os::cpu::CpuLocal").fields()
                if field.name == name)


def memory(address, length):
    """完整读取实际物理内存，失败直接使具体资源不成立。"""
    return bytes(inferior.read_memory(physical(address), length))


instructions = inferior.architecture().disassemble(entry, entry + 256)
first_load = next(item for item in instructions if item["asm"].split()[0] in ("ld", "ld.d"))
load_physical = first_load["addr"]
load_pc = load_physical if configuration["arch"] == "riscv64" else direct_base + load_physical
entry_break = gdb.Breakpoint("*%#x" % load_pc, type=gdb.BP_HARDWARE_BREAKPOINT)
gdb.execute("continue")
assert int(gdb.parse_and_eval("$pc")) == load_pc, "AP did not stop at its first memory instruction"
record = physical(int(gdb.parse_and_eval("$a1" if configuration["arch"] == "riscv64" else "$r13")))
record_before = memory(record, 16)
stack_top, local = struct.unpack("<QQ", record_before)
assert stack_top and local, "AP record has no stack or CpuLocal"
stack_size = gdb.lookup_type("os::cpu::BootStack").sizeof
stack_base = stack_top - stack_size
logical_cpu = struct.unpack("<Q", memory(local + offset("id"), 8))[0]
assert logical_cpu == 1, "freeze selected another CPU"
online_before = memory(local + offset("online"), 1)[0]
assert online_before == 0, "AP was already online before its first memory instruction"
stack_before = memory(stack_base, stack_size)
original = memory(load_physical, 4)
self_loop = struct.pack("<I", 0x0000006f if configuration["arch"] == "riscv64" else 0x50000000)
inferior.write_memory(load_physical, self_loop)
assert memory(load_physical, 4) == self_loop, "AP self-loop write was not observed"
entry_break.delete()
panic_break = gdb.Breakpoint("os::panic", type=gdb.BP_HARDWARE_BREAKPOINT)
gdb.execute("continue")
record_after = memory(record, 16)
stack_after = memory(stack_base, stack_size)
online_after = memory(local + offset("online"), 1)[0]
instruction_after = memory(load_physical, 4)
evidence = dict(complete=True, entry_physical=entry, frozen_pc=load_pc,
                first_memory_instruction=first_load["asm"], logical_cpu=logical_cpu,
                record_physical=record, record_before=record_before.hex(), record_after=record_after.hex(),
                stack_top=stack_top, stack_size=stack_size, cpu_local=local,
                stack_before_sha256=hashlib.sha256(stack_before).hexdigest(),
                stack_after_sha256=hashlib.sha256(stack_after).hexdigest(),
                online_before=online_before, online_after=online_after,
                original_instruction=original.hex(), self_loop=self_loop.hex(),
                instruction_after=instruction_after.hex(), panic_pc=int(gdb.parse_and_eval("$pc")))
assert record_before == record_after and stack_before == stack_after, "frozen AP boot storage changed"
assert online_after == 0 and instruction_after == self_loop, "frozen AP resumed or went online"
with open(configuration["evidence"], "x", encoding="utf-8") as output:
    json.dump(evidence, output, sort_keys=True, indent=2)
    output.write("\n")
panic_break.delete()
gdb.execute("maintenance packet Qqemu.PhyMemMode:0")
gdb.execute("detach")
'''
