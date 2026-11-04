//! 同核 IRQ-off 的真实 ABI 哨兵；异常在内部 Rust catch 返回，不跨 C/汇编边界。

use crate::sync::IrqSaveGuard;

/// 固定 C 回调只在内部普通 Rust 栈展开；任何异常都在返回此 C 帧之前归还。
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
    /// 保存原 GPR/FP/FCSR 与控制状态，以真实哨兵调用内部捕获回调，核验并恢复。
    /// # Safety
    /// 当前必须同 CPU、IRQ-off、FP Off；callback 必须自己捕获全部 Rust panic，不得调度。
    fn ktest_register_roundtrip_asm(callback: extern "C" fn() -> usize) -> usize;
}

/// 验证硬浮点目标 ABI 的全部 callee-saved GPR/FP、SP、per-CPU 基址和控制状态。
/// 主体回调维持 FP Off；只在 IRQ-off 汇编测量窗口临时启 FP，不允许迁核。
pub(crate) fn selftest_register_roundtrip() -> bool {
    let _irq = IrqSaveGuard::new();
    // SAFETY: IRQ guard 固定本 CPU；汇编拒绝非 FP Off 入口，完整保存原现场，
    // caught_panic 在内部 Rust catch 收回异常且不调度，从不向 C/汇编调用帧展开。
    unsafe { ktest_register_roundtrip_asm(caught_panic) != 0 }
}

// SAFETY: 两段汇编只由 IRQ guard 下的窄入口调用；临时 FP 访问前显式启用，
// 回调前关闭，返回后核验，最终恢复原 FCSR/全部 ABI callee-saved FP/GPR。
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
