//! 跨异常展开边界的被调用者保存（Callee-Saved）寄存器与控制状态往返自检模块。
//!
//! ## 模块职责与架构定位
//! 本模块归属内核测试框架（`ktest`）自检子系统，用于验证当 Rust 代码在触发 `panic!` 并完成栈展开（Unwind）与捕获（`panic::catch`）后，
//! 底层 DWARF 展开器与编译器生成的着陆垫（Landing Pad）是否正确恢复或保持了目标架构 ABI 规定的所有被调用者保存寄存器及控制状态。
//! - **上游调用者**：`ktest::selftest` 中的 `boot::registers` 与 `thread::registers` 自检用例。
//! - **下游依赖**：`ktest::panic::catch`（内核态异常捕获器）与 `sync::IrqSaveGuard`（中断屏蔽守卫）。
//!
//! ## 核心设计与栈帧布局
//! 测试通过一段架构相关的汇编桩函数 `ktest_register_roundtrip_asm` 驱动，整体执行流程如下：
//! 1. **保存调用方现场**：在当前栈上开辟固定栈帧，保存所有被调用者保存通用寄存器（GPR）、线程指针（`tp` / `$r21`）、栈指针（`sp`）以及浮点控制状态与被调用者保存浮点寄存器（FPR）。
//! 2. **注入特征哨兵值**：临时开启硬件浮点单元（FPU），向 `FCSR` 与所有被调用者保存浮点寄存器写入特征浮点位模式，随即关闭 FPU；向所有被调用者保存通用寄存器写入特征整数值。
//! 3. **触发并捕获栈展开**：调用 C ABI 回调函数 `caught_panic`。该函数在内部触发一次 Rust `panic!` 并立即通过 `panic::catch` 拦截，确保栈展开完全收敛在 Rust 栈帧内部，绝不跨越 `extern "C"` 或汇编边界。
//! 4. **校验寄存器与控制位**：回调返回后，汇编桩逐一比对中断使能位、FPU 开关位、`sp`、Per-CPU 基址寄存器、被调用者保存 GPR、FPU 状态字以及被调用者保存 FPR 是否与调用前注入的哨兵值完全一致。
//! 5. **恢复现场并返回**：无论校验是否通过，均将栈帧中备份的调用方原始 GPR/FPR 与 FCSR 完整恢复，维持 FPU 关闭状态后返回布尔结果（`1` 表示通过，`0` 表示失败）。
//!
//! ### 汇编保存栈帧布局（以 `riscv64` 256 字节栈帧为例）
//! ```text
//! 高地址 (调用前原始 sp)
//! +-------------------------------+ sp + 256
//! | 240(sp): 校验结果标志 (0 / 1) |
//! | 232(sp): 回调函数指针 (a0)    |
//! | 224(sp): 分配栈帧后的基准 sp  |
//! | 128..216: fs0 .. fs11 备份    |
//! | 120(sp): 原始 fcsr 备份       |
//! | 104(sp): 原始 tp 备份         |
//! |   8..96: s0 .. s11 备份       |
//! |   0(sp): 返回地址 ra 备份     |
//! +-------------------------------+ sp + 0 (当前栈顶)
//! 低地址
//! ```
//! （`loongarch64` 采用类似的 224 字节栈帧，额外备份 `$fp`、`$r21` 与 `CSR_KS0`。）
//!
//! ## 锁获取顺序与并发契约
//! - 本模块不获取任何自旋锁或睡眠锁。
//! - `selftest_register_roundtrip` 全程持有 `IrqSaveGuard` 屏蔽本地 CPU 中断，从而固定当前执行 CPU（禁止抢占与跨核迁移），并确保临时开启 FPU 读写浮点哨兵寄存器期间不会被中断处理程序打断。

use crate::sync::IrqSaveGuard;

/// 供汇编桩调用的 C ABI 回调函数：在内部触发一次栈展开并原地捕获。
///
/// 调用上下文：在本地中断关闭且 FPU 关闭的状态下由 `ktest_register_roundtrip_asm` 调用。
/// 内部所有的栈展开（Unwind）都在返回此 `extern "C"` 函数帧之前由 `panic::catch` 拦截完毕，
/// 若成功捕获目标异常且展开状态已完全清理，则返回 `1`，否则返回 `0`。
#[inline(never)]
extern "C" fn caught_panic() -> usize {
    match super::panic::catch(|| panic!("vertical register roundtrip")) {
        Err(record) => usize::from(
            record.matches_message("vertical register roundtrip") && !super::panic::is_unwinding(),
        ),
        Ok(()) => 0,
    }
}

extern "C" {
    /// 架构相关的寄存器往返校验汇编入口：备份原始寄存器现场，注入哨兵值后调用 `callback`，校验返回后的寄存器与控制位并恢复原始现场。
    ///
    /// # Safety
    /// 1. 调用时必须已关闭本地 CPU 中断（禁止抢占与跨核迁移），且硬件浮点单元（FPU）处于关闭状态（Off）。
    /// 2. `callback` 必须在内部捕获所有 Rust `panic!`，不得让栈展开跨越 C/汇编调用边界，且执行期间不得触发任务调度或开启中断。
    fn ktest_register_roundtrip_asm(callback: extern "C" fn() -> usize) -> usize;
}

/// 执行被调用者保存通用寄存器（GPR）、浮点寄存器（FPR）、栈指针（SP）、Per-CPU 基址寄存器及控制状态的栈展开往返自检。
///
/// 调用上下文：可在 Boot 阶段早期栈或 Thread 阶段内核线程栈上调用。
/// 函数内部通过 `IrqSaveGuard` 关闭本地中断以固定当前 CPU；主体回调执行期间维持 FPU 关闭，仅在汇编桩注入与核对浮点哨兵值的受控窗口内临时启用 FPU。
///
/// 返回值：若所有被调用者保存寄存器与控制状态在经历 `panic!` 栈展开后均完好无损，则返回 `true`。
pub(crate) fn selftest_register_roundtrip() -> bool {
    let _irq = IrqSaveGuard::new();
    // SAFETY:
    // 1. `_irq` 守卫已屏蔽本地 CPU 中断，保证当前执行流不会发生任务抢占或跨核迁移。
    // 2. 内核常规执行流默认处于 FPU 关闭状态，汇编入口也会前置检查 FPU 开关位并完整保存/恢复所有被调用者保存寄存器。
    // 3. `caught_panic` 在函数内部通过 `panic::catch` 拦截全部栈展开且不触发调度，不会向汇编调用帧传播异常。
    unsafe { ktest_register_roundtrip_asm(caught_panic) != 0 }
}

// SAFETY:
// 1. 以下两段全局汇编仅定义 `ktest_register_roundtrip_asm` 符号，且仅由上方持有 `IrqSaveGuard` 的 `selftest_register_roundtrip` 调用。
// 2. 汇编在访问浮点寄存器前先检查并临时置位 `sstatus.FS` / `CSR_EUEN.FPE`，调用 `callback` 前将其清零，
//    并在退出前完整恢复调用方原有的 `FCSR`、所有被调用者保存浮点与通用寄存器以及栈指针 `sp`，严格遵循目标平台 C ABI。
#[cfg(target_arch = "riscv64")]
core::arch::global_asm!(
    r#"
.text
.option push
.option arch, +d
.balign 4
.global ktest_register_roundtrip_asm
.type ktest_register_roundtrip_asm, @function
ktest_register_roundtrip_asm:
    addi sp, sp, -256
    sd ra, 0(sp)
    sd s0, 8(sp)
    sd s1, 16(sp)
    sd s2, 24(sp)
    sd s3, 32(sp)
    sd s4, 40(sp)
    sd s5, 48(sp)
    sd s6, 56(sp)
    sd s7, 64(sp)
    sd s8, 72(sp)
    sd s9, 80(sp)
    sd s10, 88(sp)
    sd s11, 96(sp)
    sd tp, 104(sp)
    csrr t0, sstatus
    sd sp, 224(sp)
    sd a0, 232(sp)
    sd zero, 240(sp)
    li t1, {sr_fs}
    and t2, t0, t1
    bnez t2, .Lktest_rv_exit
    csrs sstatus, t1
    frcsr t0
    sd t0, 120(sp)
    fsd fs0, 128(sp)
    fsd fs1, 136(sp)
    fsd fs2, 144(sp)
    fsd fs3, 152(sp)
    fsd fs4, 160(sp)
    fsd fs5, 168(sp)
    fsd fs6, 176(sp)
    fsd fs7, 184(sp)
    fsd fs8, 192(sp)
    fsd fs9, 200(sp)
    fsd fs10, 208(sp)
    fsd fs11, 216(sp)
    li t0, 0x61
    fscsr t0
    li t0, 0x3ff0000000000100
    fmv.d.x fs0, t0
    li t0, 0x3ff0000000000101
    fmv.d.x fs1, t0
    li t0, 0x3ff0000000000102
    fmv.d.x fs2, t0
    li t0, 0x3ff0000000000103
    fmv.d.x fs3, t0
    li t0, 0x3ff0000000000104
    fmv.d.x fs4, t0
    li t0, 0x3ff0000000000105
    fmv.d.x fs5, t0
    li t0, 0x3ff0000000000106
    fmv.d.x fs6, t0
    li t0, 0x3ff0000000000107
    fmv.d.x fs7, t0
    li t0, 0x3ff0000000000108
    fmv.d.x fs8, t0
    li t0, 0x3ff0000000000109
    fmv.d.x fs9, t0
    li t0, 0x3ff000000000010a
    fmv.d.x fs10, t0
    li t0, 0x3ff000000000010b
    fmv.d.x fs11, t0
    li t0, {sr_fs}
    csrc sstatus, t0
    li s0, 0x510
    li s1, 0x511
    li s2, 0x512
    li s3, 0x513
    li s4, 0x514
    li s5, 0x515
    li s6, 0x516
    li s7, 0x517
    li s8, 0x518
    li s9, 0x519
    li s10, 0x51a
    li s11, 0x51b
    ld t0, 232(sp)
    jalr ra, t0, 0
    csrr t0, sstatus
    csrci sstatus, {sr_sie}
    beqz a0, .Lktest_rv_restore
    li t1, {sr_fs_sie}
    and t0, t0, t1
    bnez t0, .Lktest_rv_restore
    ld t0, 224(sp)
    bne sp, t0, .Lktest_rv_restore
    ld t0, 104(sp)
    bne tp, t0, .Lktest_rv_restore
    li t0, 0x510
    bne s0, t0, .Lktest_rv_restore
    li t0, 0x511
    bne s1, t0, .Lktest_rv_restore
    li t0, 0x512
    bne s2, t0, .Lktest_rv_restore
    li t0, 0x513
    bne s3, t0, .Lktest_rv_restore
    li t0, 0x514
    bne s4, t0, .Lktest_rv_restore
    li t0, 0x515
    bne s5, t0, .Lktest_rv_restore
    li t0, 0x516
    bne s6, t0, .Lktest_rv_restore
    li t0, 0x517
    bne s7, t0, .Lktest_rv_restore
    li t0, 0x518
    bne s8, t0, .Lktest_rv_restore
    li t0, 0x519
    bne s9, t0, .Lktest_rv_restore
    li t0, 0x51a
    bne s10, t0, .Lktest_rv_restore
    li t0, 0x51b
    bne s11, t0, .Lktest_rv_restore
    li t0, {sr_fs}
    csrs sstatus, t0
    frcsr t0
    li t1, 0x61
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs0
    li t1, 0x3ff0000000000100
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs1
    li t1, 0x3ff0000000000101
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs2
    li t1, 0x3ff0000000000102
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs3
    li t1, 0x3ff0000000000103
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs4
    li t1, 0x3ff0000000000104
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs5
    li t1, 0x3ff0000000000105
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs6
    li t1, 0x3ff0000000000106
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs7
    li t1, 0x3ff0000000000107
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs8
    li t1, 0x3ff0000000000108
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs9
    li t1, 0x3ff0000000000109
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs10
    li t1, 0x3ff000000000010a
    bne t0, t1, .Lktest_rv_restore
    fmv.x.d t0, fs11
    li t1, 0x3ff000000000010b
    bne t0, t1, .Lktest_rv_restore
    li t0, 1
    sd t0, 240(sp)
.Lktest_rv_restore:
    li t0, {sr_fs}
    csrs sstatus, t0
    fld fs0, 128(sp)
    fld fs1, 136(sp)
    fld fs2, 144(sp)
    fld fs3, 152(sp)
    fld fs4, 160(sp)
    fld fs5, 168(sp)
    fld fs6, 176(sp)
    fld fs7, 184(sp)
    fld fs8, 192(sp)
    fld fs9, 200(sp)
    fld fs10, 208(sp)
    fld fs11, 216(sp)
    ld t0, 120(sp)
    fscsr t0
    li t0, {sr_fs}
    csrc sstatus, t0
.Lktest_rv_exit:
    ld a0, 240(sp)
    ld tp, 104(sp)
    ld s0, 8(sp)
    ld s1, 16(sp)
    ld s2, 24(sp)
    ld s3, 32(sp)
    ld s4, 40(sp)
    ld s5, 48(sp)
    ld s6, 56(sp)
    ld s7, 64(sp)
    ld s8, 72(sp)
    ld s9, 80(sp)
    ld s10, 88(sp)
    ld s11, 96(sp)
    ld ra, 0(sp)
    addi sp, sp, 256
    ret
.size ktest_register_roundtrip_asm, .-ktest_register_roundtrip_asm
.option pop
"#,
    sr_fs = const crate::arch::SR_FS,
    sr_sie = const crate::arch::SR_SIE,
    sr_fs_sie = const crate::arch::SR_FS | crate::arch::SR_SIE,
);

#[cfg(target_arch = "loongarch64")]
core::arch::global_asm!(
    r#"
.text
.balign 16
.global ktest_register_roundtrip_asm
.type ktest_register_roundtrip_asm, @function
ktest_register_roundtrip_asm:
    addi.d $sp, $sp, -224
    st.d $ra, $sp, 0
    st.d $fp, $sp, 8
    st.d $s0, $sp, 16
    st.d $s1, $sp, 24
    st.d $s2, $sp, 32
    st.d $s3, $sp, 40
    st.d $s4, $sp, 48
    st.d $s5, $sp, 56
    st.d $s6, $sp, 64
    st.d $s7, $sp, 72
    st.d $s8, $sp, 80
    st.d $tp, $sp, 88
    csrrd $t0, {csr_ks0}
    st.d $t0, $sp, 96
    csrrd $t0, {csr_euen}
    st.d $sp, $sp, 184
    st.d $a0, $sp, 192
    st.d $zero, $sp, 200
    st.d $r21, $sp, 208
    andi $t1, $t0, {euen_fp}
    bnez $t1, .Lktest_la_exit
    li.w $t0, {euen_fp}
    li.w $t1, {euen_fp}
    csrxchg $t0, $t1, {csr_euen}
    movfcsr2gr $t0, $fcsr0
    st.d $t0, $sp, 112
    fst.d $fs0, $sp, 120
    fst.d $fs1, $sp, 128
    fst.d $fs2, $sp, 136
    fst.d $fs3, $sp, 144
    fst.d $fs4, $sp, 152
    fst.d $fs5, $sp, 160
    fst.d $fs6, $sp, 168
    fst.d $fs7, $sp, 176
    li.w $t0, 0x100
    movgr2fcsr $fcsr0, $t0
    li.d $t0, 0x3ff0000000000100
    movgr2fr.d $fs0, $t0
    li.d $t0, 0x3ff0000000000101
    movgr2fr.d $fs1, $t0
    li.d $t0, 0x3ff0000000000102
    movgr2fr.d $fs2, $t0
    li.d $t0, 0x3ff0000000000103
    movgr2fr.d $fs3, $t0
    li.d $t0, 0x3ff0000000000104
    movgr2fr.d $fs4, $t0
    li.d $t0, 0x3ff0000000000105
    movgr2fr.d $fs5, $t0
    li.d $t0, 0x3ff0000000000106
    movgr2fr.d $fs6, $t0
    li.d $t0, 0x3ff0000000000107
    movgr2fr.d $fs7, $t0
    li.w $t1, {euen_fp}
    csrxchg $zero, $t1, {csr_euen}
    li.d $fp, 0x50f
    li.d $s0, 0x510
    li.d $s1, 0x511
    li.d $s2, 0x512
    li.d $s3, 0x513
    li.d $s4, 0x514
    li.d $s5, 0x515
    li.d $s6, 0x516
    li.d $s7, 0x517
    li.d $s8, 0x518
    ld.d $t0, $sp, 192
    jirl $ra, $t0, 0
    csrrd $t2, {csr_crmd}
    li.w $t3, {crmd_ie}
    csrxchg $zero, $t3, {csr_crmd}
    beqz $a0, .Lktest_la_restore
    csrrd $t0, {csr_euen}
    andi $t0, $t0, {euen_fp}
    bnez $t0, .Lktest_la_restore
    andi $t2, $t2, {crmd_ie}
    bnez $t2, .Lktest_la_restore
    ld.d $t0, $sp, 184
    bne $sp, $t0, .Lktest_la_restore
    ld.d $t0, $sp, 88
    bne $tp, $t0, .Lktest_la_restore
    csrrd $t0, {csr_ks0}
    ld.d $t1, $sp, 96
    bne $t0, $t1, .Lktest_la_restore
    ld.d $t0, $sp, 208
    bne $r21, $t0, .Lktest_la_restore
    li.d $t0, 0x50f
    bne $fp, $t0, .Lktest_la_restore
    li.d $t0, 0x510
    bne $s0, $t0, .Lktest_la_restore
    li.d $t0, 0x511
    bne $s1, $t0, .Lktest_la_restore
    li.d $t0, 0x512
    bne $s2, $t0, .Lktest_la_restore
    li.d $t0, 0x513
    bne $s3, $t0, .Lktest_la_restore
    li.d $t0, 0x514
    bne $s4, $t0, .Lktest_la_restore
    li.d $t0, 0x515
    bne $s5, $t0, .Lktest_la_restore
    li.d $t0, 0x516
    bne $s6, $t0, .Lktest_la_restore
    li.d $t0, 0x517
    bne $s7, $t0, .Lktest_la_restore
    li.d $t0, 0x518
    bne $s8, $t0, .Lktest_la_restore
    li.w $t0, {euen_fp}
    li.w $t1, {euen_fp}
    csrxchg $t0, $t1, {csr_euen}
    movfcsr2gr $t0, $fcsr0
    li.w $t1, 0x100
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs0
    li.d $t1, 0x3ff0000000000100
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs1
    li.d $t1, 0x3ff0000000000101
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs2
    li.d $t1, 0x3ff0000000000102
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs3
    li.d $t1, 0x3ff0000000000103
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs4
    li.d $t1, 0x3ff0000000000104
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs5
    li.d $t1, 0x3ff0000000000105
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs6
    li.d $t1, 0x3ff0000000000106
    bne $t0, $t1, .Lktest_la_restore
    movfr2gr.d $t0, $fs7
    li.d $t1, 0x3ff0000000000107
    bne $t0, $t1, .Lktest_la_restore
    li.w $t0, 1
    st.d $t0, $sp, 200
.Lktest_la_restore:
    li.w $t0, {euen_fp}
    li.w $t1, {euen_fp}
    csrxchg $t0, $t1, {csr_euen}
    fld.d $fs0, $sp, 120
    fld.d $fs1, $sp, 128
    fld.d $fs2, $sp, 136
    fld.d $fs3, $sp, 144
    fld.d $fs4, $sp, 152
    fld.d $fs5, $sp, 160
    fld.d $fs6, $sp, 168
    fld.d $fs7, $sp, 176
    ld.d $t0, $sp, 112
    movgr2fcsr $fcsr0, $t0
    li.w $t1, {euen_fp}
    csrxchg $zero, $t1, {csr_euen}
.Lktest_la_exit:
    ld.d $a0, $sp, 200
    ld.d $tp, $sp, 88
    ld.d $t0, $sp, 96
    csrwr $t0, {csr_ks0}
    ld.d $r21, $sp, 208
    ld.d $fp, $sp, 8
    ld.d $s0, $sp, 16
    ld.d $s1, $sp, 24
    ld.d $s2, $sp, 32
    ld.d $s3, $sp, 40
    ld.d $s4, $sp, 48
    ld.d $s5, $sp, 56
    ld.d $s6, $sp, 64
    ld.d $s7, $sp, 72
    ld.d $s8, $sp, 80
    ld.d $ra, $sp, 0
    addi.d $sp, $sp, 224
    jr $ra
.size ktest_register_roundtrip_asm, .-ktest_register_roundtrip_asm
"#,
    csr_euen = const crate::arch::CSR_EUEN,
    euen_fp = const crate::arch::EUEN_FP,
    csr_crmd = const crate::arch::CSR_CRMD,
    crmd_ie = const crate::arch::CRMD_IE,
    csr_ks0 = const crate::arch::CSR_KS0,
);
