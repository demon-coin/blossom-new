//! RISC-V 64 用户态进入（[`rv_run_user`]）、陷入返回（[`rv_user_trap_entry`]）与内核调用续接裸汇编实现。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::riscv64::user` 底层，承接上层 [`super::run_user`] 与共享硬件中断入口
//! （`crate::arch::riscv64::trap::rv_trap_entry`）之间的特权级切换：
//! 1. **进入用户态（[`rv_run_user`]）**：将当前内核执行流的被调用者保存寄存器（Callee-saved）、`ra`、`gp`、`tp`
//!    与 `sstatus` 保存在当前任务内核栈的 128 字节续接帧中，把 `UserContext` 地址写入 `sscratch`，随后恢复
//!    用户浮点与整数寄存器并通过 `sret` 降至 U-mode。
//! 2. **从用户态陷入（[`rv_user_trap_entry`]）**：当 U-mode 发生异常或中断时，先由 `rv_trap_entry` 通过
//!    `csrrw sp, sscratch, sp` 换入 `UserContext` 指针并保存用户 `t0`，随后跳转到本模块的
//!    [`rv_user_trap_entry`] 保存完整用户寄存器与异常 CSR，最后从内核栈续接帧恢复内核上下文并 `ret`
//!    回到当初调用 [`rv_run_user`] 的同一处 Rust 调用点。
//!
//! # 核心设计与栈帧/寄存器交换契约
//!
//! ## 1. 内核栈续接帧布局（`KERNEL_CONTINUATION_SIZE = 128` 字节）
//! 调用 [`rv_run_user`] 时在当前任务内核栈顶向下分配 128 字节（16 字节对齐），其栈顶地址存入
//! `UserContext.kernel_sp`：
//!
//! ```text
//! 高地址 (原内核 sp)
//! +---------------------------------------------------------------+ 128 (KERNEL_CONTINUATION_SIZE)
//! | sstatus : 进入前内核态 sstatus 快照 (FS=Off, SIE=0)           | 120 (KERNEL_STATUS_OFFSET)
//! | s11     : 被调用者保存寄存器 x27                              | 112
//! | s10     : 被调用者保存寄存器 x26                              | 104
//! | ...     : s1..s9 (x9, x18..x25)                               | 32..96
//! | s0 / fp : 帧指针 x8                                           | 24
//! | tp      : 当前 CPU 的 CpuLocal 指针 x4                        | 16
//! | gp      : 内核全局指针 x3                                     | 8
//! | ra      : 返回 run_user 的返回地址 x1                         | 0   <-- UserContext.kernel_sp
//! +---------------------------------------------------------------+ 低地址
//! ```
//!
//! ## 2. `sscratch` 寄存器状态转换机
//! `sscratch` 是区分 Trap 来源（U-mode 还是 S-mode）的唯一硬件判据：
//!
//! ```text
//! [S-mode 正常运行]
//!    sscratch == 0
//!        |
//!        | rv_run_user: 保存续接帧，写入 sscratch = &mut UserContext，执行 sret
//!        v
//! [U-mode 用户态运行]
//!    sscratch == &mut UserContext, sp == 用户栈指针
//!        |
//!        | 触发中断/异常 -> 硬件跳入 rv_trap_entry:
//!        |   csrrw sp, sscratch, sp (交换后 sp = &mut UserContext, sscratch = 用户 sp)
//!        |   保存用户 t0 到 UserContext.gpr[5]，跳转至 rv_user_trap_entry
//!        v
//! [rv_user_trap_entry 保存用户现场]
//!    csrr t0, sscratch (取出用户 sp 存入 gpr[2])
//!    csrw sscratch, sp (在恢复内核 tp/gp 前保持 sscratch 指向 UserContext，以备窗口期故障诊断)
//!    保存全部用户 GPR / FPR / FCSR / sepc / scause / stval
//!    从 UserContext.kernel_sp 恢复内核 gp / tp / sstatus，随后清零 kernel_sp 与 sscratch
//!        |
//!        | 恢复 ra, s0..s11，释放 128B 续接帧，执行 ret
//!        v
//! [回到 S-mode run_user]
//!    sscratch == 0
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块全部运行在硬中断关闭（`sstatus.SIE == 0`）区间或 U-mode 下，不获取任何锁。
//! - 续接帧中保存了当前 CPU 的 `tp`（`CpuLocal` 指针），因此从 `rv_run_user` 保存续接帧到
//!   `rv_user_trap_entry` 返回的整个生命周期内**严禁调用调度器或跨 CPU 迁移任务**。

use crate::arch::riscv64::{
    rv_kernel_fault, user::UserContext, SR_FS_DIRTY, SR_SPIE, SR_SPP, SR_UXL_64,
};
#[cfg(feature = "ktest")]
use core::arch::global_asm;
use core::{
    arch::naked_asm,
    mem::{offset_of, size_of},
};

// 内核栈续接帧的偏移与总字节数：依次保存 ra/gp/tp（3 个字）、s0..s11（12 个字）以及 sstatus（1 个字），共 16 * 8 = 128 字节。
const KERNEL_STATUS_OFFSET: usize = (3 + 12) * size_of::<usize>();
const KERNEL_CONTINUATION_SIZE: usize = KERNEL_STATUS_OFFSET + size_of::<usize>();

/// 保存当前任务的内核态被调用者保存寄存器与状态字至内核栈续接帧，恢复 `context` 中的用户态现场并通过 `sret` 进入 U-mode；
/// 待用户态触发下一次中断或异常后，由 [`rv_user_trap_entry`] 恢复续接帧并直接从本函数返回。
///
/// # Safety
/// 1. `_context` 必须指向当前普通任务独占持有的有效 [`UserContext`]，且位于当前活动页表的内核高半映射中；
/// 2. 调用前必须已关闭 S-mode 硬中断（`SIE == 0`）且内核浮点单元处于关闭状态（`sstatus.FS == Off`）；
/// 3. 当前激活的用户页表、内核高半映射、`_context` 以及当前任务内核栈必须在本次调用返回前保持有效；
/// 4. 在本函数返回前严禁触发线程调度或更改当前 CPU 的 `CpuLocal` 绑定。
#[naked]
#[no_mangle]
pub(in crate::arch::riscv64) unsafe extern "C" fn rv_run_user(_context: *mut UserContext) {
    // SAFETY:
    // 1. 调用方 `run_user` 已通过前置断言确保硬中断关闭、`sstatus.FS == Off`，且 `a0`（`_context`）独占可写；
    // 2. 在当前任务内核栈上向下分配的 128 字节续接帧完整保存了 C ABI 要求的 `ra`、`gp`、`tp`、`s0..s11` 与 `sstatus`，
    //    由 `rv_user_trap_entry` 在陷入时对称恢复并平衡栈指针；
    // 3. `sstatus` 被显式重写为仅开启 `UXL=64 | FS=Dirty | SPIE`（清除 `SPP`/`SUM`/`MXR`），并通过向 `context.stval`
    //    执行哑 `sc.d` 清除可能残留的内核态 LR/SC 原子预约。
    unsafe {
        naked_asm!(
        r#"
.option push
.option arch, +a, +d
    // 本次调用的续接始终放在当前 Task 栈；TP/GP 仅在此不调度的汇编区间保活。
    addi sp, sp, -{continuation_size}
    sd ra, 0(sp)
    sd gp, 8(sp)
    sd tp, 16(sp)
    sd s0, 24(sp)
    sd s1, 32(sp)
    sd s2, 40(sp)
    sd s3, 48(sp)
    sd s4, 56(sp)
    sd s5, 64(sp)
    sd s6, 72(sp)
    sd s7, 80(sp)
    sd s8, 88(sp)
    sd s9, 96(sp)
    sd s10, 104(sp)
    sd s11, 112(sp)
    csrr t0, sstatus
    sd t0, {continuation_status}(sp)
    sd sp, {kernel_sp}(a0)
    csrw sscratch, a0
    mv sp, a0
"#,
        // Step 2: 临时开启浮点单元（FS=Dirty）以加载用户态 f0..f31 与 fcsr，设置 sepc 与干净的用户态 sstatus，并清除 LR/SC 预约。
        r#"
    li t0, {fs_dirty}
    csrs sstatus, t0
    fld f0, ({fpr} + 0 * 8)(sp)
    fld f1, ({fpr} + 1 * 8)(sp)
    fld f2, ({fpr} + 2 * 8)(sp)
    fld f3, ({fpr} + 3 * 8)(sp)
    fld f4, ({fpr} + 4 * 8)(sp)
    fld f5, ({fpr} + 5 * 8)(sp)
    fld f6, ({fpr} + 6 * 8)(sp)
    fld f7, ({fpr} + 7 * 8)(sp)
    fld f8, ({fpr} + 8 * 8)(sp)
    fld f9, ({fpr} + 9 * 8)(sp)
    fld f10, ({fpr} + 10 * 8)(sp)
    fld f11, ({fpr} + 11 * 8)(sp)
    fld f12, ({fpr} + 12 * 8)(sp)
    fld f13, ({fpr} + 13 * 8)(sp)
    fld f14, ({fpr} + 14 * 8)(sp)
    fld f15, ({fpr} + 15 * 8)(sp)
    fld f16, ({fpr} + 16 * 8)(sp)
    fld f17, ({fpr} + 17 * 8)(sp)
    fld f18, ({fpr} + 18 * 8)(sp)
    fld f19, ({fpr} + 19 * 8)(sp)
    fld f20, ({fpr} + 20 * 8)(sp)
    fld f21, ({fpr} + 21 * 8)(sp)
    fld f22, ({fpr} + 22 * 8)(sp)
    fld f23, ({fpr} + 23 * 8)(sp)
    fld f24, ({fpr} + 24 * 8)(sp)
    fld f25, ({fpr} + 25 * 8)(sp)
    fld f26, ({fpr} + 26 * 8)(sp)
    fld f27, ({fpr} + 27 * 8)(sp)
    fld f28, ({fpr} + 28 * 8)(sp)
    fld f29, ({fpr} + 29 * 8)(sp)
    fld f30, ({fpr} + 30 * 8)(sp)
    fld f31, ({fpr} + 31 * 8)(sp)
    ld t0, {fcsr}(sp)
    fscsr t0
    ld t0, {pc}(sp)
    csrw sepc, t0
    // 只构造 UXL=64、FS=Dirty、SPIE；禁止 SPP/SUM/MXR/VS 从私有现场回流。
    li t0, {user_status}
    csrw sstatus, t0
    // 用户与内核不共享 LR/SC reservation；此 scratch 槽的值将在 trap 时覆盖。
    addi t0, sp, {stval}
    sc.d zero, zero, (t0)
"#,
        // Step 3: 恢复用户态通用整数寄存器 x1、x3..x31，最后将 sp 切换为用户栈指针 x2 并执行 sret 降至 U-mode。
        r#"
    ld x1, ({gpr} + 1 * 8)(sp)
    ld x3, ({gpr} + 3 * 8)(sp)
    ld x4, ({gpr} + 4 * 8)(sp)
    ld x5, ({gpr} + 5 * 8)(sp)
    ld x6, ({gpr} + 6 * 8)(sp)
    ld x7, ({gpr} + 7 * 8)(sp)
    ld x8, ({gpr} + 8 * 8)(sp)
    ld x9, ({gpr} + 9 * 8)(sp)
    ld x10, ({gpr} + 10 * 8)(sp)
    ld x11, ({gpr} + 11 * 8)(sp)
    ld x12, ({gpr} + 12 * 8)(sp)
    ld x13, ({gpr} + 13 * 8)(sp)
    ld x14, ({gpr} + 14 * 8)(sp)
    ld x15, ({gpr} + 15 * 8)(sp)
    ld x16, ({gpr} + 16 * 8)(sp)
    ld x17, ({gpr} + 17 * 8)(sp)
    ld x18, ({gpr} + 18 * 8)(sp)
    ld x19, ({gpr} + 19 * 8)(sp)
    ld x20, ({gpr} + 20 * 8)(sp)
    ld x21, ({gpr} + 21 * 8)(sp)
    ld x22, ({gpr} + 22 * 8)(sp)
    ld x23, ({gpr} + 23 * 8)(sp)
    ld x24, ({gpr} + 24 * 8)(sp)
    ld x25, ({gpr} + 25 * 8)(sp)
    ld x26, ({gpr} + 26 * 8)(sp)
    ld x27, ({gpr} + 27 * 8)(sp)
    ld x28, ({gpr} + 28 * 8)(sp)
    ld x29, ({gpr} + 29 * 8)(sp)
    ld x30, ({gpr} + 30 * 8)(sp)
    ld x31, ({gpr} + 31 * 8)(sp)
    ld sp, ({gpr} + 2 * 8)(sp)
    sret
.option pop
"#,
        gpr = const offset_of!(UserContext, gpr),
        pc = const offset_of!(UserContext, pc),
        fpr = const offset_of!(UserContext, fpr),
        fcsr = const offset_of!(UserContext, fcsr),
        kernel_sp = const offset_of!(UserContext, kernel_sp),
        stval = const offset_of!(UserContext, stval),
        fs_dirty = const SR_FS_DIRTY,
        user_status = const SR_UXL_64 | SR_FS_DIRTY | SR_SPIE,
        continuation_size = const KERNEL_CONTINUATION_SIZE,
        continuation_status = const KERNEL_STATUS_OFFSET,
            );
    }
}

/// 承接来自公共陷阱向量 `rv_trap_entry` 的用户态陷入（或切换窗口期的 S-mode 致命故障），
/// 将用户态寄存器完整保存至 [`UserContext`] 后恢复内核栈续接帧，返回到 [`rv_run_user`] 的调用点。
///
/// # Safety
/// 1. 严禁从普通 Rust/C 代码直接调用，只能由 `rv_trap_entry` 在检测到 `sscratch != 0` 时通过 `jr` 跳入；
/// 2. 跳入时 `sp` 必须指向本次 `rv_run_user` 设置的独占 [`UserContext`]，用户 `t0`（`x5`）已保存在 `gpr[5]`，
///    且 `sscratch` 中暂存着被中断时刻的原 `sp`；
/// 3. `UserContext.kernel_sp` 指向的内核栈续接帧必须存活且完好；若 `sstatus.SPP != 0`（表示在入口/出口极窄窗口内
///    发生了 S-mode 异常），则必须沿标号 `2:` 转交 `rv_kernel_fault` 终止诊断，不得伪装成用户态 Trap 返回。
#[naked]
#[no_mangle]
pub(in crate::arch::riscv64) unsafe extern "C" fn rv_user_trap_entry() -> ! {
    // SAFETY:
    // 1. 前置入口 `rv_trap_entry` 已完成 `sp` 与 `sscratch` 的原子交换并将用户 `t0` 存入 `gpr[5]`，当前 `sp` 指向有效的 `UserContext`；
    // 2. 通过检查 `sstatus.SPP` 分流：若来自 U-mode（`SPP == 0`），则保存浮点上下文并从 `kernel_sp` 恢复内核 `gp`/`tp`/`sstatus`
    //    及 `ra`/`s0..s11`，清零 `sscratch` 后安全 `ret` 回 `run_user`；
    // 3. 若 `SPP != 0`，则同样先恢复内核 `gp`/`tp`/`sp` 并清零 `sscratch`，再调用永不返回的 `rv_kernel_fault`。
    unsafe {
        naked_asm!(
        r#"
.option push
.option arch, +a, +d
    // 通用入口已保存用户 t0 并换入受信 context；sscratch 暂存被中断的原 SP。
    csrr t0, sscratch
    csrw sscratch, sp
    sd t0, ({gpr} + 2 * 8)(sp)
    sd x1, ({gpr} + 1 * 8)(sp)
    sd x3, ({gpr} + 3 * 8)(sp)
    sd x4, ({gpr} + 4 * 8)(sp)
    sd x6, ({gpr} + 6 * 8)(sp)
    sd x7, ({gpr} + 7 * 8)(sp)
    sd x8, ({gpr} + 8 * 8)(sp)
    sd x9, ({gpr} + 9 * 8)(sp)
    sd x10, ({gpr} + 10 * 8)(sp)
    sd x11, ({gpr} + 11 * 8)(sp)
    sd x12, ({gpr} + 12 * 8)(sp)
    sd x13, ({gpr} + 13 * 8)(sp)
    sd x14, ({gpr} + 14 * 8)(sp)
    sd x15, ({gpr} + 15 * 8)(sp)
    sd x16, ({gpr} + 16 * 8)(sp)
    sd x17, ({gpr} + 17 * 8)(sp)
    sd x18, ({gpr} + 18 * 8)(sp)
    sd x19, ({gpr} + 19 * 8)(sp)
    sd x20, ({gpr} + 20 * 8)(sp)
    sd x21, ({gpr} + 21 * 8)(sp)
    sd x22, ({gpr} + 22 * 8)(sp)
    sd x23, ({gpr} + 23 * 8)(sp)
    sd x24, ({gpr} + 24 * 8)(sp)
    sd x25, ({gpr} + 25 * 8)(sp)
    sd x26, ({gpr} + 26 * 8)(sp)
    sd x27, ({gpr} + 27 * 8)(sp)
    sd x28, ({gpr} + 28 * 8)(sp)
    sd x29, ({gpr} + 29 * 8)(sp)
    sd x30, ({gpr} + 30 * 8)(sp)
    sd x31, ({gpr} + 31 * 8)(sp)
    sd zero, {gpr}(sp)
    csrr t0, sepc
    sd t0, {pc}(sp)
    csrr t0, scause
    sd t0, {scause}(sp)
    csrr t0, stval
    sd t0, {stval}(sp)
    csrr t0, sstatus
    andi t0, t0, {spp}
    bnez t0, 2f
"#,
        // Step 2: 确认陷入源自 U-mode（SPP == 0）后，将用户态全部 32 个浮点寄存器与 fcsr 保存至 UserContext。
        r#"
    fsd f0, ({fpr} + 0 * 8)(sp)
    fsd f1, ({fpr} + 1 * 8)(sp)
    fsd f2, ({fpr} + 2 * 8)(sp)
    fsd f3, ({fpr} + 3 * 8)(sp)
    fsd f4, ({fpr} + 4 * 8)(sp)
    fsd f5, ({fpr} + 5 * 8)(sp)
    fsd f6, ({fpr} + 6 * 8)(sp)
    fsd f7, ({fpr} + 7 * 8)(sp)
    fsd f8, ({fpr} + 8 * 8)(sp)
    fsd f9, ({fpr} + 9 * 8)(sp)
    fsd f10, ({fpr} + 10 * 8)(sp)
    fsd f11, ({fpr} + 11 * 8)(sp)
    fsd f12, ({fpr} + 12 * 8)(sp)
    fsd f13, ({fpr} + 13 * 8)(sp)
    fsd f14, ({fpr} + 14 * 8)(sp)
    fsd f15, ({fpr} + 15 * 8)(sp)
    fsd f16, ({fpr} + 16 * 8)(sp)
    fsd f17, ({fpr} + 17 * 8)(sp)
    fsd f18, ({fpr} + 18 * 8)(sp)
    fsd f19, ({fpr} + 19 * 8)(sp)
    fsd f20, ({fpr} + 20 * 8)(sp)
    fsd f21, ({fpr} + 21 * 8)(sp)
    fsd f22, ({fpr} + 22 * 8)(sp)
    fsd f23, ({fpr} + 23 * 8)(sp)
    fsd f24, ({fpr} + 24 * 8)(sp)
    fsd f25, ({fpr} + 25 * 8)(sp)
    fsd f26, ({fpr} + 26 * 8)(sp)
    fsd f27, ({fpr} + 27 * 8)(sp)
    fsd f28, ({fpr} + 28 * 8)(sp)
    fsd f29, ({fpr} + 29 * 8)(sp)
    fsd f30, ({fpr} + 30 * 8)(sp)
    fsd f31, ({fpr} + 31 * 8)(sp)
    frcsr t0
    sd t0, {fcsr}(sp)
"#,
        // Step 3: 从 UserContext.kernel_sp 指向的内核栈续接帧恢复内核 gp、tp 与 sstatus（此后再清零 kernel_sp 与 sscratch），
        // 最后恢复 ra 与 s0..s11，释放 128 字节续接帧并通过 ret 返回 run_user。
        r#"
    ld t0, {kernel_sp}(sp)
    // 受信 TP/GP/状态恢复前保留 context 入口，S 来源故障仍能取得真实内核栈。
    ld gp, 8(t0)
    ld tp, 16(t0)
    ld t1, {continuation_status}(t0)
    csrw sstatus, t1
    sd zero, {kernel_sp}(sp)
    csrw sscratch, zero
    mv sp, t0
    ld ra, 0(sp)
    ld s0, 24(sp)
    ld s1, 32(sp)
    ld s2, 40(sp)
    ld s3, 48(sp)
    ld s4, 56(sp)
    ld s5, 64(sp)
    ld s6, 72(sp)
    ld s7, 80(sp)
    ld s8, 88(sp)
    ld s9, 96(sp)
    ld s10, 104(sp)
    ld s11, 112(sp)
    addi sp, sp, {continuation_size}
    ret
"#,
        // Step 4 (异常分支 2f): 若在进入/退出窗口内发生 S-mode 异常（SPP != 0），提取故障参数并切回内核续接栈调用 rv_kernel_fault。
        r#"

2:
    // 入口或退出窗口内的 S 来源异常仍致命，用真实续接栈执行原内核诊断。
    ld a0, {pc}(sp)
    ld a1, ({gpr} + 2 * 8)(sp)
    ld a2, {scause}(sp)
    ld a3, {stval}(sp)
    ld t0, {kernel_sp}(sp)
    ld gp, 8(t0)
    ld tp, 16(t0)
    ld t1, {continuation_status}(t0)
    csrw sstatus, t1
    sd zero, {kernel_sp}(sp)
    csrw sscratch, zero
    mv sp, t0
    call {kernel_fault}
.option pop
"#,
        gpr = const offset_of!(UserContext, gpr),
        pc = const offset_of!(UserContext, pc),
        fpr = const offset_of!(UserContext, fpr),
        fcsr = const offset_of!(UserContext, fcsr),
        kernel_sp = const offset_of!(UserContext, kernel_sp),
        scause = const offset_of!(UserContext, scause),
        stval = const offset_of!(UserContext, stval),
        spp = const SR_SPP,
        continuation_size = const KERNEL_CONTINUATION_SIZE,
        continuation_status = const KERNEL_STATUS_OFFSET,
        kernel_fault = sym rv_kernel_fault,
            );
    }
}

// 内核测试（ktest）专用的位置无关用户态探测指令块：仅包含局部相对跳转，可整体拷贝至任意用户可执行页运行；使用 .option push/pop 隔离扩展指令集设置。
#[cfg(feature = "ktest")]
global_asm!(
    r#"
.option push
.option arch, +a, +d
.pushsection .text.user_probe, "ax", @progbits
.balign 4
.global rv_user_probe_start
.global rv_user_probe_end
.global rv_user_probe_loop
.global rv_user_probe_spin
.global rv_user_probe_exit
.global rv_user_probe_read
.global rv_user_probe_write
.global rv_user_probe_execute
rv_user_probe_start:
    // 所有地址来自用户寄存器；局部分支没有外部重定位，可整体复制到用户指令页。
    sd sp, 0(a6)
    sd tp, 8(a6)
    sd gp, 16(a6)
    fadd.d f2, f0, f1
    fsd f2, 24(a6)
    frcsr t0
    sd t0, 32(a6)
    sd tp, -16(sp)
    ld t0, -16(sp)
    sd t0, 40(a6)
    li a7, 93
    li a0, 41
    ecall
    li a0, 42
    ecall

.balign 4
rv_user_probe_loop:
    sd sp, 0(a6)
    sd tp, 8(a6)
    sd gp, 16(a6)
    fadd.d f2, f0, f1
    fsd f2, 24(a6)
    fld f3, 24(a6)
    feq.d t0, f2, f3
    sd t0, 56(a6)
    frcsr t0
    sd t0, 32(a6)
    sd tp, -16(sp)
    ld t0, -16(sp)
    sd t0, 40(a6)
rv_user_probe_spin:
    j rv_user_probe_spin

.balign 4
rv_user_probe_exit:
    li a7, 93
    li a0, 43
    ecall
    j rv_user_probe_exit

.balign 4
rv_user_probe_read:
    ld t0, 0(a5)
    j rv_user_probe_exit
rv_user_probe_write:
    sd zero, 0(a5)
    j rv_user_probe_exit
rv_user_probe_execute:
    jr a5
rv_user_probe_end:
.popsection
.option pop
"#,
);
