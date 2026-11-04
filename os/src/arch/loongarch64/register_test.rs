//! 真实 timer IRQ 的寄存器探针；手工栈帧同时保存原控制状态。

use crate::arch::loongarch64::register as reg;
use core::arch::naked_asm;
use core::sync::atomic::AtomicUsize;

/// 等待本核真实 timer IRQ 并比较整数、SP、KS0 及所选 FP/FCSR/FCC 图样。
///
/// # Safety
/// 必须处于 PLV0，本核异常入口、timer IRQ 源、CpuLocal 和当前 ABI 栈均已建立；
/// counter 须指向本核永久、自然对齐的 timer 原子计数，ticks 为有效的四字节对齐周期。
/// 调用期间须固定 CPU、禁止调度，不得持 handler 所需锁或有活跃 FP 上下文；
/// fp 仅取零或一，非零时硬件须支持标量 FP，fcc 须为八位值。
/// 返回恢复进入时的 IE/EUEN，但临时 timer deadline 须由调用者在同核 IRQ-off 下交还生产层，
/// 包括检查失败后的清理；不得跨此裸帧展开。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_test_register_preservation(
    counter: *const AtomicUsize,
    ticks: usize,
    fcc: usize,
    fp: usize,
) -> usize {
    // SAFETY: 调用者保证本核异常/timer、永久计数器及 ABI 栈有效，期间不调度；
    // 手动帧保留原整数、控制和被测 FP 现场，临时 IRQ/FP 开关均在本次调用内恢复。
    unsafe {
        naked_asm!(
            // 192 字节栈帧保留调用者整数、原 EUEN/CRMD、KS0 及真实计数器快照。
            r#"
    addi.d $sp, $sp, -192
    st.d $ra, $sp, 0
    st.d $s0, $sp, 8
    st.d $s1, $sp, 16
    csrrd $t2, {csr_euen}
    st.d $t2, $sp, 24
    csrrd $t2, {csr_crmd}
    st.d $t2, $sp, 32
    st.d $a0, $sp, 40
    ld.d $t2, $a0, 0
    st.d $t2, $sp, 48
    st.d $a1, $sp, 56
    st.d $a2, $sp, 64
    st.d $a3, $sp, 72
    csrrd $t2, {csr_ks0}
    st.d $t2, $sp, 80
    st.d $sp, $sp, 88
"#,
            // FP 分支暂开标量单元并保存 f0/f24、FCSR 与全部 FCC，再布置独立图样。
            r#"
    beqz $a3, 2f
    li.w $t2, {reg_euen_fp}
    csrwr $t2, {csr_euen}
    fst.d $f0, $sp, 96
    fst.d $f24, $sp, 104
    movfcsr2gr $t2, $fcsr0
    st.d $t2, $sp, 112
    movcf2gr $t2, $fcc0
    st.d $t2, $sp, 120
    movcf2gr $t2, $fcc1
    st.d $t2, $sp, 128
    movcf2gr $t2, $fcc2
    st.d $t2, $sp, 136
    movcf2gr $t2, $fcc3
    st.d $t2, $sp, 144
    movcf2gr $t2, $fcc4
    st.d $t2, $sp, 152
    movcf2gr $t2, $fcc5
    st.d $t2, $sp, 160
    movcf2gr $t2, $fcc6
    st.d $t2, $sp, 168
    movcf2gr $t2, $fcc7
    st.d $t2, $sp, 176
    li.d $t2, 0x3ff123456789abcd
    movgr2fr.d $f0, $t2
    li.d $t2, 0x400fedcba9876543
    movgr2fr.d $f24, $t2
    li.w $t2, 0x100
    movgr2fcsr $fcsr0, $t2
    srli.d $t2, $a2, 0
    andi $t2, $t2, 1
    movgr2cf $fcc0, $t2
    srli.d $t2, $a2, 1
    andi $t2, $t2, 1
    movgr2cf $fcc1, $t2
    srli.d $t2, $a2, 2
    andi $t2, $t2, 1
    movgr2cf $fcc2, $t2
    srli.d $t2, $a2, 3
    andi $t2, $t2, 1
    movgr2cf $fcc3, $t2
    srli.d $t2, $a2, 4
    andi $t2, $t2, 1
    movgr2cf $fcc4, $t2
    srli.d $t2, $a2, 5
    andi $t2, $t2, 1
    movgr2cf $fcc5, $t2
    srli.d $t2, $a2, 6
    andi $t2, $t2, 1
    movgr2cf $fcc6, $t2
    srli.d $t2, $a2, 7
    andi $t2, $t2, 1
    movgr2cf $fcc7, $t2
    b 3f
2:
    csrwr $zero, {csr_euen}
"#,
            // 布置整数图样并安装 timer，临时开 IE 等待真实计数变化后重新关 IE。
            r#"
3:
    li.d $t0, 0x123
    li.d $t1, 0x456
    li.d $s0, 0x789
    li.d $s1, 0xabc
    li.d $a2, 0xdef
    ld.d $t2, $sp, 56
    ori $t2, $t2, {reg_tcfg_enable}
    csrwr $t2, {csr_tcfg}
    li.w $t2, {reg_crmd_ie}
    li.w $t3, {reg_crmd_ie}
    csrxchg $t2, $t3, {csr_crmd}
4:
    ld.d $t2, $sp, 40
    ld.d $t2, $t2, 0
    ld.d $t3, $sp, 48
    beq $t2, $t3, 4b
    li.w $t2, {reg_crmd_ie}
    csrxchg $zero, $t2, {csr_crmd}
"#,
            // 检查全部被测整数与扩展现场，未启用 FP 时只核 EUEN 保持关闭。
            r#"
    li.d $t4, 0x123
    bne $t0, $t4, 7f
    li.d $t4, 0x456
    bne $t1, $t4, 7f
    li.d $t4, 0x789
    bne $s0, $t4, 7f
    li.d $t4, 0xabc
    bne $s1, $t4, 7f
    li.d $t4, 0xdef
    bne $a2, $t4, 7f
    ld.d $t4, $sp, 88
    bne $sp, $t4, 7f
    csrrd $t4, {csr_ks0}
    ld.d $t5, $sp, 80
    bne $t4, $t5, 7f
    ld.d $t4, $sp, 72
    beqz $t4, 5f
    movfr2gr.d $t4, $f0
    li.d $t5, 0x3ff123456789abcd
    bne $t4, $t5, 7f
    movfr2gr.d $t4, $f24
    li.d $t5, 0x400fedcba9876543
    bne $t4, $t5, 7f
    movfcsr2gr $t4, $fcsr0
    li.w $t5, 0x100
    bne $t4, $t5, 7f
    move $t4, $zero
    movcf2gr $t5, $fcc0
    slli.d $t5, $t5, 0
    or $t4, $t4, $t5
    movcf2gr $t5, $fcc1
    slli.d $t5, $t5, 1
    or $t4, $t4, $t5
    movcf2gr $t5, $fcc2
    slli.d $t5, $t5, 2
    or $t4, $t4, $t5
    movcf2gr $t5, $fcc3
    slli.d $t5, $t5, 3
    or $t4, $t4, $t5
    movcf2gr $t5, $fcc4
    slli.d $t5, $t5, 4
    or $t4, $t4, $t5
    movcf2gr $t5, $fcc5
    slli.d $t5, $t5, 5
    or $t4, $t4, $t5
    movcf2gr $t5, $fcc6
    slli.d $t5, $t5, 6
    or $t4, $t4, $t5
    movcf2gr $t5, $fcc7
    slli.d $t5, $t5, 7
    or $t4, $t4, $t5
    ld.d $t5, $sp, 64
    bne $t4, $t5, 7f
    b 6f
5:
    csrrd $t4, {csr_euen}
    bnez $t4, 7f
6:
    li.w $a0, 1
    b 8f
7:
    move $a0, $zero
"#,
            // 按原模式恢复 FP、EUEN、KS0、IE 和调用者整数现场，然后退还手工栈帧。
            r#"
8:
    ld.d $t2, $sp, 72
    beqz $t2, 9f
    li.w $t2, {reg_euen_fp}
    csrwr $t2, {csr_euen}
    fld.d $f0, $sp, 96
    fld.d $f24, $sp, 104
    ld.d $t2, $sp, 112
    movgr2fcsr $fcsr0, $t2
    ld.d $t2, $sp, 120
    movgr2cf $fcc0, $t2
    ld.d $t2, $sp, 128
    movgr2cf $fcc1, $t2
    ld.d $t2, $sp, 136
    movgr2cf $fcc2, $t2
    ld.d $t2, $sp, 144
    movgr2cf $fcc3, $t2
    ld.d $t2, $sp, 152
    movgr2cf $fcc4, $t2
    ld.d $t2, $sp, 160
    movgr2cf $fcc5, $t2
    ld.d $t2, $sp, 168
    movgr2cf $fcc6, $t2
    ld.d $t2, $sp, 176
    movgr2cf $fcc7, $t2
9:
    ld.d $t2, $sp, 24
    csrwr $t2, {csr_euen}
    ld.d $t2, $sp, 80
    csrwr $t2, {csr_ks0}
    ld.d $t2, $sp, 32
    li.w $t3, {reg_crmd_ie}
    csrxchg $t2, $t3, {csr_crmd}
    ld.d $ra, $sp, 0
    ld.d $s0, $sp, 8
    ld.d $s1, $sp, 16
    addi.d $sp, $sp, 192
    jirl $zero, $ra, 0
"#,
            csr_crmd = const reg::CSR_CRMD,
            csr_euen = const reg::CSR_EUEN,
            csr_ks0 = const reg::CSR_KS0,
            csr_tcfg = const reg::CSR_TCFG,
            reg_crmd_ie = const reg::CRMD_IE,
            reg_euen_fp = const reg::EUEN_FP,
            reg_tcfg_enable = const reg::TCFG_ENABLE,
        );
    }
}
