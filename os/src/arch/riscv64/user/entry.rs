//! 用户进入、陷入与同一 Rust 调用续接；所有入口共享唯一 UserContext 布局。

use crate::arch::riscv64::{rv_kernel_fault, user::UserContext};
#[cfg(feature = "ktest")]
use core::arch::global_asm;
use core::{arch::naked_asm, mem::offset_of};

/// 保存当前 Task 的真实 Rust 续接，通过 sret 执行用户现场，陷入后返回同一次调用。
///
/// # Safety
/// context 必须由当前普通 Task 独占并位于活动根的内核映射；IRQ 关闭且内核 FS Off。
/// 用户根、内核映射、context 与当前 Task 栈必须保活到返回，期间不得调用调度器。
#[naked]
#[no_mangle]
pub(in crate::arch::riscv64) unsafe extern "C" fn rv_run_user(_context: *mut UserContext) {
    // SAFETY: 调用者满足活动根、独占现场和栈寿命；128 字节续接保留完整内核 callee-saved 与状态。
    unsafe {
        naked_asm!(
        r#"
.option push
.option arch, +a, +d
    // 本次调用的续接始终放在当前 Task 栈；TP/GP 仅在此不调度的汇编区间保活。
    addi sp, sp, -128
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
    sd t0, 120(sp)
    sd sp, {kernel_sp}(a0)
    csrw sscratch, a0
    mv sp, a0
"#,
        // 恢复全部用户浮点现场，并只构造许可的 sstatus 位，清除跨特权 LR/SC reservation。
        r#"
    li t0, 0x6000
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
    li t0, 0x200006020
    csrw sstatus, t0
    // 用户与内核不共享 LR/SC reservation；此 scratch 槽的值将在 trap 时覆盖。
    addi t0, sp, {stval}
    sc.d zero, zero, (t0)
"#,
        // 最后恢复用户整数寄存器与用户 SP，通过 sret 降到用户特权级。
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
            );
    }
}

/// 从公共 trap 向量接收受信 context，保存用户现场后恢复当前 Task 的原 Rust 续接。
///
/// # Safety
/// 只能从公共向量跳入；SP 必须指向本次 rv_run_user 的独占 context，
/// 用户 t0 已保存且 sscratch 仍持被中断的原 SP。context 中的真实 Task 续接必须存活；
/// S 来源异常只能沿原 fatal 路径处理，不得当作用户 trap 返回。
#[naked]
#[no_mangle]
pub(in crate::arch::riscv64) unsafe extern "C" fn rv_user_trap_entry() -> ! {
    // SAFETY: 公共向量已按 scratch 合同保存 t0；受信现场和续接在恢复 GP/TP/状态前保持可用。
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
    andi t0, t0, 0x100
    bnez t0, 2f
"#,
        // 确认用户来源后保存全部 FPR/FCSR，不能把用户 TLS 当作 CpuLocal。
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
        // 从本次 Task 续接恢复受信 GP/TP/状态，清除 scratch 和 kernel_sp 后返回原 Rust 调用。
        r#"
    ld t0, {kernel_sp}(sp)
    // 受信 TP/GP/状态恢复前保留 context 入口，S 来源故障仍能取得真实内核栈。
    ld gp, 8(t0)
    ld tp, 16(t0)
    ld t1, 120(t0)
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
    addi sp, sp, 128
    ret
"#,
        // 进入或退出窗口的 S 来源故障使用真实续接栈，保留原致命诊断路径。
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
    ld t1, 120(t0)
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
        kernel_fault = sym rv_kernel_fault,
            );
    }
}

// 固定指令块只含局部分支，可整体复制到用户页；扩展选项不影响其他裸函数。
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
