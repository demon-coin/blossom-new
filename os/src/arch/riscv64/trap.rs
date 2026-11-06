//! RISC-V 64 位 S-mode 共享陷阱入口（Trap Entry）与内核态中断/异常现场保存恢复。
//!
//! # 模块职责与架构定位
//! 本模块定义写入 `stvec` 寄存器的唯一硬件陷阱向量入口 [`rv_trap_entry`]。
//! 硬件发生异常、中断或系统调用陷入 S-mode 时首先跳转到此处：
//! - 若陷入前处于 U-mode，入口将控制流转发给用户态陷阱处理入口 `user::rv_user_trap_entry`；
//! - 若陷入前处于 S-mode，入口在当前内核栈上分配并填充 [`KernelTrapFrame`]，调用 Rust 分派函数
//!   `rv_handle_trap`，并在返回后恢复现场、执行 `sret` 回到被中断的内核指令。
//!
//! # 核心机制：`sscratch` 分流与 `KernelTrapFrame` 栈帧布局
//! 1. **`sscratch` 特权级分流契约**：
//!    - 当 CPU 在 **S-mode（内核态）** 运行时，`sscratch` 恒为 `0`；
//!    - 当 CPU 进入 **U-mode（用户态）** 运行前，内核会将当前任务的私有 `UserContext` 指针写入 `sscratch`。
//!    - 陷入发生时，入口第一条指令执行 `csrrw sp, sscratch, sp` 交换 `sp` 与 `sscratch`：
//!      - 若交换后的 `sp != 0`，说明陷入来自 U-mode，此时 `sp` 指向 `UserContext`，`sscratch` 暂存用户栈指针；
//!      - 若交换后的 `sp == 0`，说明陷入来自 S-mode，再次执行 `csrrw sp, sscratch, sp` 即可还原内核栈指针 `sp` 并保持 `sscratch == 0`。
//! 2. **按需保存浮点现场**：
//!    普通内核代码在 `sstatus.FS = Off` 下运行，不触碰浮点寄存器。仅当受控的浮点测试或特殊路径使陷入前的
//!    `sstatus.FS != Off` 时，汇编才会保存并在返回前恢复 `f0`–`f31` 与 `fcsr`，随后在进入 Rust 处理函数前关闭 `FS`。
//!
//! ```text
//! KernelTrapFrame 栈帧布局 (16 字节对齐，共 560 字节)
//! +---------+-------------------+-----------------------------------------------+
//! | 偏移    | 字段              | 说明                                          |
//! +---------+-------------------+-----------------------------------------------+
//! | 0x000   | gpr[0..32]        | 32 个通用整数寄存器 (x0 恒写 0，x2 为原 sp)   |
//! | 0x100   | sepc              | 陷入时的程序计数器 (sepc CSR)                 |
//! | 0x108   | sstatus           | 陷入时的处理器状态字 (sstatus CSR)            |
//! | 0x110   | scause            | 陷入原因编码 (scause CSR)                     |
//! | 0x118   | stval             | 陷入附加信息/故障虚拟地址 (stval CSR)         |
//! | 0x120   | fpr[0..32]        | 32 个 64 位浮点寄存器 (仅 sstatus.FS!=Off 有效)|
//! | 0x220   | fcsr              | 浮点控制与状态寄存器 (仅 sstatus.FS!=Off 有效)|
//! | 0x228   | (padding)         | 尾随 8 字节填充，凑齐 16 字节栈对齐 (0x230)   |
//! +---------+-------------------+-----------------------------------------------+
//! ```
//!
//! # 锁与并发契约
//! - 硬件陷入 S-mode 时会自动清除 `sstatus.SIE`（关闭本地中断），整个汇编保存/恢复阶段与 `rv_handle_trap`
//!   均在本地中断关闭的状态下执行。
//! - 内核陷阱帧直接分配在当前 CPU 正在使用的内核栈上，由当前执行流独占，不涉及跨核共享锁。

use crate::arch::riscv64::{
    rv_handle_trap, user::rv_user_trap_entry, KernelTrapFrame, UserContext, SR_FS,
};
use core::{
    arch::naked_asm,
    mem::{offset_of, size_of},
};

/// RISC-V S-mode 统一硬件陷阱向量入口（`stvec` 目标地址）。
///
/// 根据 `sscratch` 是否为零区分陷入来源（U-mode 或 S-mode）：用户态陷入跳转至 `rv_user_trap_entry`，
/// 内核态陷入则在当前内核栈上构建 [`KernelTrapFrame`] 并调用 `rv_handle_trap`，处理完毕后执行 `sret` 返回。
///
/// # Safety
/// 1. 本函数是裸汇编硬件陷阱入口，只能由 CPU 在发生 S-mode 异常或中断时通过 `stvec` 硬件机制跳入，严禁在代码中直接调用。
/// 2. 若从 S-mode 陷入，进入时 `sscratch` 必须为 `0`，`sp` 必须指向当前 CPU 有效且按 16 字节对齐的内核栈，
///    且栈剩余空间足以容纳完整的 [`KernelTrapFrame`] 及后续 Rust 中断处理调用链。
/// 3. 若从 U-mode 陷入，进入时 `sscratch` 必须指向当前任务独占、有效且已初始化的 [`UserContext`]。
/// 4. 内核页表映射与当前 CPU 的 `tp`（`CpuLocal` 基址）必须在整个陷入处理与返回期间保持有效。
#[naked]
#[no_mangle]
#[link_section = ".text.trap.entry"]
pub(in crate::arch::riscv64) unsafe extern "C" fn rv_trap_entry() -> ! {
    // SAFETY:
    // 1. CPU 硬件陷阱机制进入时已自动关闭本地中断（`sstatus.SIE = 0`）。
    // 2. 依据 `sscratch` 分流契约：当 `sscratch != 0` 时其值为合法的 `UserContext` 指针，先把 `t0` 保存至
    //    `UserContext.gpr[5]` 再跳转 `rv_user_trap_entry`；当 `sscratch == 0` 时第二次 `csrrw` 无损还原内核 `sp`。
    // 3. 在 16 字节对齐的内核栈上减去 `size_of::<KernelTrapFrame>()` 后，所有 `sd`/`fsd` 均按 `offset_of!`
    //    写入当前栈帧内部；进入 Rust 前清除 `sstatus.FS`，确保编译生成的 Rust 代码不会意外破坏浮点现场。
    unsafe {
        naked_asm!(
        // Step 1: 交换 sp 与 sscratch 分流 U/S 模式陷入；若为内核态则在栈上分配 KernelTrapFrame 并保存整数与 CSR 现场。
        r#"
.option push
.option arch, +d
    // sscratch 为零时保留原内核栈；用户执行时只接受内核预置的私有现场指针。
    csrrw sp, sscratch, sp
    beqz sp, 2f
    // 不依赖外部入口位于 JAL 的 ±1MiB 内；先保存用户 t0，再用 PC 相对长跳转。
    sd t0, ({user_gpr} + 5 * {word})(sp)
    lla t0, {user_trap}
    jr t0
2:
    csrrw sp, sscratch, sp
    addi sp, sp, -{frame_size}
    sd x1, ({gpr} + 1 * {word})(sp)
    sd x3, ({gpr} + 3 * {word})(sp)
    sd x4, ({gpr} + 4 * {word})(sp)
    sd x5, ({gpr} + 5 * {word})(sp)
    sd x6, ({gpr} + 6 * {word})(sp)
    sd x7, ({gpr} + 7 * {word})(sp)
    sd x8, ({gpr} + 8 * {word})(sp)
    sd x9, ({gpr} + 9 * {word})(sp)
    sd x10, ({gpr} + 10 * {word})(sp)
    sd x11, ({gpr} + 11 * {word})(sp)
    sd x12, ({gpr} + 12 * {word})(sp)
    sd x13, ({gpr} + 13 * {word})(sp)
    sd x14, ({gpr} + 14 * {word})(sp)
    sd x15, ({gpr} + 15 * {word})(sp)
    sd x16, ({gpr} + 16 * {word})(sp)
    sd x17, ({gpr} + 17 * {word})(sp)
    sd x18, ({gpr} + 18 * {word})(sp)
    sd x19, ({gpr} + 19 * {word})(sp)
    sd x20, ({gpr} + 20 * {word})(sp)
    sd x21, ({gpr} + 21 * {word})(sp)
    sd x22, ({gpr} + 22 * {word})(sp)
    sd x23, ({gpr} + 23 * {word})(sp)
    sd x24, ({gpr} + 24 * {word})(sp)
    sd x25, ({gpr} + 25 * {word})(sp)
    sd x26, ({gpr} + 26 * {word})(sp)
    sd x27, ({gpr} + 27 * {word})(sp)
    sd x28, ({gpr} + 28 * {word})(sp)
    sd x29, ({gpr} + 29 * {word})(sp)
    sd x30, ({gpr} + 30 * {word})(sp)
    sd x31, ({gpr} + 31 * {word})(sp)
    sd zero, {gpr}(sp)
    addi t0, sp, {frame_size}
    sd t0, ({gpr} + 2 * {word})(sp)
    csrr t0, sepc
    sd t0, {sepc}(sp)
    csrr t0, sstatus
    sd t0, {sstatus}(sp)
    csrr t0, scause
    sd t0, {scause}(sp)
    csrr t0, stval
    sd t0, {stval}(sp)
"#,
        // Step 2: 若陷入前开启了浮点单元（sstatus.FS != Off），保存全部 32 个浮点寄存器与 fcsr。
        r#"
    ld t0, {sstatus}(sp)
    li t1, {fs}
    and t0, t0, t1
    beqz t0, 3f
    fsd f0, ({fpr} + 0 * {float_word})(sp)
    fsd f1, ({fpr} + 1 * {float_word})(sp)
    fsd f2, ({fpr} + 2 * {float_word})(sp)
    fsd f3, ({fpr} + 3 * {float_word})(sp)
    fsd f4, ({fpr} + 4 * {float_word})(sp)
    fsd f5, ({fpr} + 5 * {float_word})(sp)
    fsd f6, ({fpr} + 6 * {float_word})(sp)
    fsd f7, ({fpr} + 7 * {float_word})(sp)
    fsd f8, ({fpr} + 8 * {float_word})(sp)
    fsd f9, ({fpr} + 9 * {float_word})(sp)
    fsd f10, ({fpr} + 10 * {float_word})(sp)
    fsd f11, ({fpr} + 11 * {float_word})(sp)
    fsd f12, ({fpr} + 12 * {float_word})(sp)
    fsd f13, ({fpr} + 13 * {float_word})(sp)
    fsd f14, ({fpr} + 14 * {float_word})(sp)
    fsd f15, ({fpr} + 15 * {float_word})(sp)
    fsd f16, ({fpr} + 16 * {float_word})(sp)
    fsd f17, ({fpr} + 17 * {float_word})(sp)
    fsd f18, ({fpr} + 18 * {float_word})(sp)
    fsd f19, ({fpr} + 19 * {float_word})(sp)
    fsd f20, ({fpr} + 20 * {float_word})(sp)
    fsd f21, ({fpr} + 21 * {float_word})(sp)
    fsd f22, ({fpr} + 22 * {float_word})(sp)
    fsd f23, ({fpr} + 23 * {float_word})(sp)
    fsd f24, ({fpr} + 24 * {float_word})(sp)
    fsd f25, ({fpr} + 25 * {float_word})(sp)
    fsd f26, ({fpr} + 26 * {float_word})(sp)
    fsd f27, ({fpr} + 27 * {float_word})(sp)
    fsd f28, ({fpr} + 28 * {float_word})(sp)
    fsd f29, ({fpr} + 29 * {float_word})(sp)
    fsd f30, ({fpr} + 30 * {float_word})(sp)
    fsd f31, ({fpr} + 31 * {float_word})(sp)
    frcsr t0
    sd t0, {fcsr}(sp)
3:
"#,
        // Step 3: 关闭浮点单元（清除 sstatus.FS），将栈上的 KernelTrapFrame 指针作为参数 a0 传给 Rust 处理函数。
        r#"
    li t0, {fs}
    csrc sstatus, t0
    mv a0, sp
    call {handle_trap}
"#,
        // Step 4: 从 Rust 返回后先写回原 sstatus；若陷入前启用了浮点单元，则恢复全部浮点寄存器与 fcsr。
        r#"
    ld t0, {sstatus}(sp)
    csrw sstatus, t0
    li t1, {fs}
    and t0, t0, t1
    beqz t0, 4f
    fld f0, ({fpr} + 0 * {float_word})(sp)
    fld f1, ({fpr} + 1 * {float_word})(sp)
    fld f2, ({fpr} + 2 * {float_word})(sp)
    fld f3, ({fpr} + 3 * {float_word})(sp)
    fld f4, ({fpr} + 4 * {float_word})(sp)
    fld f5, ({fpr} + 5 * {float_word})(sp)
    fld f6, ({fpr} + 6 * {float_word})(sp)
    fld f7, ({fpr} + 7 * {float_word})(sp)
    fld f8, ({fpr} + 8 * {float_word})(sp)
    fld f9, ({fpr} + 9 * {float_word})(sp)
    fld f10, ({fpr} + 10 * {float_word})(sp)
    fld f11, ({fpr} + 11 * {float_word})(sp)
    fld f12, ({fpr} + 12 * {float_word})(sp)
    fld f13, ({fpr} + 13 * {float_word})(sp)
    fld f14, ({fpr} + 14 * {float_word})(sp)
    fld f15, ({fpr} + 15 * {float_word})(sp)
    fld f16, ({fpr} + 16 * {float_word})(sp)
    fld f17, ({fpr} + 17 * {float_word})(sp)
    fld f18, ({fpr} + 18 * {float_word})(sp)
    fld f19, ({fpr} + 19 * {float_word})(sp)
    fld f20, ({fpr} + 20 * {float_word})(sp)
    fld f21, ({fpr} + 21 * {float_word})(sp)
    fld f22, ({fpr} + 22 * {float_word})(sp)
    fld f23, ({fpr} + 23 * {float_word})(sp)
    fld f24, ({fpr} + 24 * {float_word})(sp)
    fld f25, ({fpr} + 25 * {float_word})(sp)
    fld f26, ({fpr} + 26 * {float_word})(sp)
    fld f27, ({fpr} + 27 * {float_word})(sp)
    fld f28, ({fpr} + 28 * {float_word})(sp)
    fld f29, ({fpr} + 29 * {float_word})(sp)
    fld f30, ({fpr} + 30 * {float_word})(sp)
    fld f31, ({fpr} + 31 * {float_word})(sp)
    ld t0, {fcsr}(sp)
    fscsr t0
4:
"#,
        // Step 5: 恢复 sepc、sstatus 与全部通用整数寄存器（最后恢复 sp），执行 sret 返回被中断的内核上下文。
        r#"
    ld t0, {sepc}(sp)
    csrw sepc, t0
    ld t0, {sstatus}(sp)
    csrw sstatus, t0
    ld x1, ({gpr} + 1 * {word})(sp)
    ld x3, ({gpr} + 3 * {word})(sp)
    ld x4, ({gpr} + 4 * {word})(sp)
    ld x5, ({gpr} + 5 * {word})(sp)
    ld x6, ({gpr} + 6 * {word})(sp)
    ld x7, ({gpr} + 7 * {word})(sp)
    ld x8, ({gpr} + 8 * {word})(sp)
    ld x9, ({gpr} + 9 * {word})(sp)
    ld x10, ({gpr} + 10 * {word})(sp)
    ld x11, ({gpr} + 11 * {word})(sp)
    ld x12, ({gpr} + 12 * {word})(sp)
    ld x13, ({gpr} + 13 * {word})(sp)
    ld x14, ({gpr} + 14 * {word})(sp)
    ld x15, ({gpr} + 15 * {word})(sp)
    ld x16, ({gpr} + 16 * {word})(sp)
    ld x17, ({gpr} + 17 * {word})(sp)
    ld x18, ({gpr} + 18 * {word})(sp)
    ld x19, ({gpr} + 19 * {word})(sp)
    ld x20, ({gpr} + 20 * {word})(sp)
    ld x21, ({gpr} + 21 * {word})(sp)
    ld x22, ({gpr} + 22 * {word})(sp)
    ld x23, ({gpr} + 23 * {word})(sp)
    ld x24, ({gpr} + 24 * {word})(sp)
    ld x25, ({gpr} + 25 * {word})(sp)
    ld x26, ({gpr} + 26 * {word})(sp)
    ld x27, ({gpr} + 27 * {word})(sp)
    ld x28, ({gpr} + 28 * {word})(sp)
    ld x29, ({gpr} + 29 * {word})(sp)
    ld x30, ({gpr} + 30 * {word})(sp)
    ld x31, ({gpr} + 31 * {word})(sp)
    ld sp, ({gpr} + 2 * {word})(sp)
    sret
.option pop
"#,
        frame_size = const size_of::<KernelTrapFrame>(),
        gpr = const offset_of!(KernelTrapFrame, gpr),
        user_gpr = const offset_of!(UserContext, gpr),
        word = const size_of::<usize>(),
        sepc = const offset_of!(KernelTrapFrame, sepc),
        sstatus = const offset_of!(KernelTrapFrame, sstatus),
        scause = const offset_of!(KernelTrapFrame, scause),
        stval = const offset_of!(KernelTrapFrame, stval),
        fpr = const offset_of!(KernelTrapFrame, fpr),
        float_word = const size_of::<u64>(),
        fcsr = const offset_of!(KernelTrapFrame, fcsr),
        fs = const SR_FS,
        handle_trap = sym rv_handle_trap,
        user_trap = sym rv_user_trap_entry,
            );
    }
}
