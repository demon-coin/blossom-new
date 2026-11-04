//! 用户进入、异常返回及入口故障共用本次 Task 的受信续接帧。

use crate::arch::loongarch64::register as reg;
use crate::arch::loongarch64::user::{loongarch_user_kernel_fault, UserContext};
#[cfg(feature = "ktest")]
use core::arch::global_asm;
use core::arch::naked_asm;
use core::mem::{offset_of, size_of};

const CONTINUATION_GPRS: usize = 13;
const CONTINUATION_KS3: usize = CONTINUATION_GPRS * core::mem::size_of::<usize>();
const CONTINUATION_EENTRY: usize = CONTINUATION_KS3 + core::mem::size_of::<usize>();
const CONTINUATION_PRMD: usize = CONTINUATION_EENTRY + core::mem::size_of::<usize>();
const CONTINUATION_ERA: usize = CONTINUATION_PRMD + core::mem::size_of::<usize>();
const CONTINUATION_KS1: usize = CONTINUATION_ERA + core::mem::size_of::<usize>();
const CONTINUATION_KS2: usize = CONTINUATION_KS1 + core::mem::size_of::<usize>();
const CONTINUATION_CRMD: usize = CONTINUATION_KS2 + core::mem::size_of::<usize>();
const CONTINUATION_SIZE: usize = CONTINUATION_CRMD + core::mem::size_of::<usize>();

/// 保存当前 Task 的 160 字节 ABI 续接，恢复最宽用户现场并进入 PLV3，异常后回到同一次调用。
///
/// # Safety
/// context 必须是当前 Task 独占、完整初始化且保活的私有现场；Mm、当前栈与目的 CPU 的 KS0 必须有效。
/// 本核 IRQ 和内核 EUEN 必须关闭，栈满足 16 字节对齐，不得持有跨用户进入的 FP 状态或允许跨裸帧展开。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_run_user(_context: *mut UserContext) {
    // SAFETY: 调用者独占有效 context 和 Task 栈，IRQ/EUEN 已关闭；160 字节续接保活到同次用户异常返回。
    unsafe {
        naked_asm!(
            r#"
# /// 按共享最宽槽访问整组寄存器，标量和 LSX 只触及实际支持的低 lanes。
.macro LA_FP_REGS insn, prefix
    .irp n, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
        \insn $\prefix\()\n, $sp, {fpr}+\n*{fp_stride}
    .endr
.endm
    addi.d $sp, $sp, -{continuation_size}
    st.d $r1, $sp, 0
    st.d $r2, $sp, 8
    st.d $r21, $sp, 16
    st.d $r22, $sp, 24
    st.d $r23, $sp, 32
    st.d $r24, $sp, 40
    st.d $r25, $sp, 48
    st.d $r26, $sp, 56
    st.d $r27, $sp, 64
    st.d $r28, $sp, 72
    st.d $r29, $sp, 80
    st.d $r30, $sp, 88
    st.d $r31, $sp, 96
    csrrd $t0, {csr_ks3}
    st.d $t0, $sp, {continuation_ks3}
    csrrd $t0, {csr_eentry}
    st.d $t0, $sp, {continuation_eentry}
    csrrd $t0, {csr_prmd}
    st.d $t0, $sp, {continuation_prmd}
    csrrd $t0, {csr_era}
    st.d $t0, $sp, {continuation_era}
    csrrd $t0, {csr_ks1}
    st.d $t0, $sp, {continuation_ks1}
    csrrd $t0, {csr_ks2}
    st.d $t0, $sp, {continuation_ks2}
    csrrd $t0, {csr_crmd}
    st.d $t0, $sp, {continuation_crmd}
    move $t0, $sp
    st.d $t0, $a0, {kernel_sp}
    # KS3 保持受信 Task 续接 SP，覆盖入口及退出期间所有 context/用户 SP 窗口。
    move $t0, $sp
    csrwr $t0, {csr_ks3}
    move $t0, $a0
    csrwr $t0, {csr_ks1}
    la.abs $t0, {user_trap}
    csrwr $t0, {csr_eentry}
    move $sp, $a0

    # EUEN 只取 CPUCFG2 中具有完整依赖的 FP/LSX/LASX 能力。
    li.w $t0, {reg_cpucfg_features}
    cpucfg $t0, $t0
    andi $t1, $t0, {reg_cpucfg_fp}
    beqz $t1, 2f
    andi $t2, $t0, {reg_cpucfg_lsx}
    beqz $t2, 2f
    ori $t1, $t1, {reg_euen_lsx}
    andi $t2, $t0, {reg_cpucfg_lasx}
    beqz $t2, 2f
    ori $t1, $t1, {reg_euen_lasx}
2:
    csrwr $t1, {csr_euen}
    csrrd $t0, {csr_euen}
    andi $t1, $t0, {reg_euen_lasx}
    bnez $t1, 4f
    andi $t1, $t0, {reg_euen_lsx}
    bnez $t1, 3f
    beqz $t0, 6f
    LA_FP_REGS fld.d, f
    b 5f
3:
    LA_FP_REGS vld, vr
    b 5f
4:
    LA_FP_REGS xvld, xr
5:
    ld.d $t0, $sp, {fcsr}
    movgr2fcsr $fcsr0, $t0
    ld.d $t0, $sp, {fcc}
    bstrpick.d $t1, $t0, 7, 0
    movgr2cf $fcc0, $t1
    bstrpick.d $t1, $t0, 15, 8
    movgr2cf $fcc1, $t1
    bstrpick.d $t1, $t0, 23, 16
    movgr2cf $fcc2, $t1
    bstrpick.d $t1, $t0, 31, 24
    movgr2cf $fcc3, $t1
    bstrpick.d $t1, $t0, 39, 32
    movgr2cf $fcc4, $t1
    bstrpick.d $t1, $t0, 47, 40
    movgr2cf $fcc5, $t1
    bstrpick.d $t1, $t0, 55, 48
    movgr2cf $fcc6, $t1
    bstrpick.d $t1, $t0, 63, 56
    movgr2cf $fcc7, $t1
6:

    ld.d $t0, $sp, {pc}
    csrwr $t0, {csr_era}
    # 特权和 IRQ 状态只由内核常量构造，用户寄存器不参与 PRMD。
    li.w $t0, {reg_prmd_user}
    csrwr $t0, {csr_prmd}
    ld.d $r1, $sp, {gpr}+1*8
    ld.d $r2, $sp, {gpr}+2*8
    ld.d $r4, $sp, {gpr}+4*8
    ld.d $r5, $sp, {gpr}+5*8
    ld.d $r6, $sp, {gpr}+6*8
    ld.d $r7, $sp, {gpr}+7*8
    ld.d $r8, $sp, {gpr}+8*8
    ld.d $r9, $sp, {gpr}+9*8
    ld.d $r10, $sp, {gpr}+10*8
    ld.d $r11, $sp, {gpr}+11*8
    ld.d $r12, $sp, {gpr}+12*8
    ld.d $r13, $sp, {gpr}+13*8
    ld.d $r14, $sp, {gpr}+14*8
    ld.d $r15, $sp, {gpr}+15*8
    ld.d $r16, $sp, {gpr}+16*8
    ld.d $r17, $sp, {gpr}+17*8
    ld.d $r18, $sp, {gpr}+18*8
    ld.d $r19, $sp, {gpr}+19*8
    ld.d $r20, $sp, {gpr}+20*8
    ld.d $r21, $sp, {gpr}+21*8
    ld.d $r22, $sp, {gpr}+22*8
    ld.d $r23, $sp, {gpr}+23*8
    ld.d $r24, $sp, {gpr}+24*8
    ld.d $r25, $sp, {gpr}+25*8
    ld.d $r26, $sp, {gpr}+26*8
    ld.d $r27, $sp, {gpr}+27*8
    ld.d $r28, $sp, {gpr}+28*8
    ld.d $r29, $sp, {gpr}+29*8
    ld.d $r30, $sp, {gpr}+30*8
    ld.d $r31, $sp, {gpr}+31*8
    ld.d $sp, $sp, {gpr}+3*8
    ertn


.purgem LA_FP_REGS
"#,
            gpr = const offset_of!(UserContext, gpr),
            pc = const offset_of!(UserContext, pc),
            fpr = const offset_of!(UserContext, fpr),
            fcsr = const offset_of!(UserContext, fcsr),
            fcc = const offset_of!(UserContext, fcc),
            kernel_sp = const offset_of!(UserContext, kernel_sp),
            user_trap = sym la_user_trap,
            csr_crmd = const reg::CSR_CRMD,
            csr_eentry = const reg::CSR_EENTRY,
            csr_era = const reg::CSR_ERA,
            csr_euen = const reg::CSR_EUEN,
            csr_ks1 = const reg::CSR_KS1,
            csr_ks2 = const reg::CSR_KS2,
            csr_ks3 = const reg::CSR_KS3,
            csr_prmd = const reg::CSR_PRMD,
            reg_cpucfg_features = const reg::CPUCFG_FEATURES,
            reg_cpucfg_fp = const reg::CPUCFG_FP,
            reg_cpucfg_lasx = const reg::CPUCFG_LASX,
            reg_cpucfg_lsx = const reg::CPUCFG_LSX,
            reg_euen_lasx = const reg::EUEN_LASX,
            reg_euen_lsx = const reg::EUEN_LSX,
            reg_prmd_user = const reg::PRMD_USER,
            continuation_crmd = const CONTINUATION_CRMD,
            continuation_eentry = const CONTINUATION_EENTRY,
            continuation_era = const CONTINUATION_ERA,
            continuation_ks1 = const CONTINUATION_KS1,
            continuation_ks2 = const CONTINUATION_KS2,
            continuation_ks3 = const CONTINUATION_KS3,
            continuation_prmd = const CONTINUATION_PRMD,
            continuation_size = const CONTINUATION_SIZE,
            fp_stride = const size_of::<crate::arch::loongarch64::user::FpRegister>(),
        );
    }
}

/// 在用户异常中保存完整 GPR、最宽 FP/FCSR/FCC，并恢复当前 Task 的原内核续接与入口 CSR。
///
/// # Safety
/// 仅由已安装的 EENTRY 进入；KS1 必须定位独占 UserContext，KS3 必须保留本次调用的受信 Task 续接。
/// 用户来源不得用用户 SP 建帧；PLV0 入口故障只能在恢复 KS3 栈后进入 Rust fatal，KS0 始终属于目的 CPU。
#[naked]
#[no_mangle]
#[link_section = ".text.vector.user_trap"]
pub(in crate::arch::loongarch64) unsafe extern "C" fn la_user_trap() -> ! {
    // SAFETY: KS1/KS3 由本次用户调用建立并保活；用户寄存器只写入私有现场，恢复受信栈后才回到内核调用。
    unsafe {
        naked_asm!(
            r#"
# /// 按共享最宽槽访问整组寄存器，标量和 LSX 只触及实际支持的低 lanes。
.macro LA_FP_REGS insn, prefix
    .irp n, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
        \insn $\prefix\()\n, $sp, {fpr}+\n*{fp_stride}
    .endr
.endm
    # KS0 永久保留 CpuLocal；KS2 只在入口暂存用户 t0。
    csrwr $t0, {csr_ks2}
    csrrd $t0, {csr_prmd}
    andi $t0, $t0, {reg_prmd_pplv}
    bnez $t0, 2f
    b 7f
2:
    csrwr $sp, {csr_ks1}
    st.d $r0, $sp, {gpr}+0*8
    st.d $r1, $sp, {gpr}+1*8
    st.d $r2, $sp, {gpr}+2*8
    st.d $r4, $sp, {gpr}+4*8
    st.d $r5, $sp, {gpr}+5*8
    st.d $r6, $sp, {gpr}+6*8
    st.d $r7, $sp, {gpr}+7*8
    st.d $r8, $sp, {gpr}+8*8
    st.d $r9, $sp, {gpr}+9*8
    st.d $r10, $sp, {gpr}+10*8
    st.d $r11, $sp, {gpr}+11*8
    st.d $r13, $sp, {gpr}+13*8
    st.d $r14, $sp, {gpr}+14*8
    st.d $r15, $sp, {gpr}+15*8
    st.d $r16, $sp, {gpr}+16*8
    st.d $r17, $sp, {gpr}+17*8
    st.d $r18, $sp, {gpr}+18*8
    st.d $r19, $sp, {gpr}+19*8
    st.d $r20, $sp, {gpr}+20*8
    st.d $r21, $sp, {gpr}+21*8
    st.d $r22, $sp, {gpr}+22*8
    st.d $r23, $sp, {gpr}+23*8
    st.d $r24, $sp, {gpr}+24*8
    st.d $r25, $sp, {gpr}+25*8
    st.d $r26, $sp, {gpr}+26*8
    st.d $r27, $sp, {gpr}+27*8
    st.d $r28, $sp, {gpr}+28*8
    st.d $r29, $sp, {gpr}+29*8
    st.d $r30, $sp, {gpr}+30*8
    st.d $r31, $sp, {gpr}+31*8
    csrrd $t0, {csr_ks2}
    st.d $t0, $sp, {gpr}+12*8
    csrrd $t0, {csr_ks1}
    st.d $t0, $sp, {gpr}+3*8
    csrrd $t0, {csr_era}
    st.d $t0, $sp, {pc}
    csrrd $t0, {csr_estat}
    st.d $t0, $sp, {estat}
    csrrd $t0, {csr_badv}
    st.d $t0, $sp, {badv}

    csrrd $t0, {csr_euen}
    andi $t1, $t0, {reg_euen_lasx}
    bnez $t1, 4f
    andi $t1, $t0, {reg_euen_lsx}
    bnez $t1, 3f
    beqz $t0, 6f
    LA_FP_REGS fst.d, f
    b 5f
3:
    LA_FP_REGS vst, vr
    b 5f
4:
    LA_FP_REGS xvst, xr
5:
    movfcsr2gr $t0, $fcsr0
    st.d $t0, $sp, {fcsr}
    movcf2gr $t0, $fcc0
    movcf2gr $t1, $fcc1
    bstrins.d $t0, $t1, 15, 8
    movcf2gr $t1, $fcc2
    bstrins.d $t0, $t1, 23, 16
    movcf2gr $t1, $fcc3
    bstrins.d $t0, $t1, 31, 24
    movcf2gr $t1, $fcc4
    bstrins.d $t0, $t1, 39, 32
    movcf2gr $t1, $fcc5
    bstrins.d $t0, $t1, 47, 40
    movcf2gr $t1, $fcc6
    bstrins.d $t0, $t1, 55, 48
    movcf2gr $t1, $fcc7
    bstrins.d $t0, $t1, 63, 56
    st.d $t0, $sp, {fcc}
6:

    csrwr $zero, {csr_euen}
    ld.d $sp, $sp, {kernel_sp}
    ld.d $tp, $sp, 8
    ld.d $r21, $sp, 16
    ld.d $t0, $sp, {continuation_eentry}
    csrwr $t0, {csr_eentry}
    ld.d $t0, $sp, {continuation_ks3}
    csrwr $t0, {csr_ks3}
    ld.d $t0, $sp, {continuation_prmd}
    csrwr $t0, {csr_prmd}
    ld.d $t0, $sp, {continuation_era}
    csrwr $t0, {csr_era}
    ld.d $t0, $sp, {continuation_ks1}
    csrwr $t0, {csr_ks1}
    ld.d $t0, $sp, {continuation_ks2}
    csrwr $t0, {csr_ks2}
    ld.d $t0, $sp, {continuation_crmd}
    csrwr $t0, {csr_crmd}
    ld.d $r1, $sp, 0
    ld.d $r2, $sp, 8
    ld.d $r21, $sp, 16
    ld.d $r22, $sp, 24
    ld.d $r23, $sp, 32
    ld.d $r24, $sp, 40
    ld.d $r25, $sp, 48
    ld.d $r26, $sp, 56
    ld.d $r27, $sp, 64
    ld.d $r28, $sp, 72
    ld.d $r29, $sp, 80
    ld.d $r30, $sp, 88
    ld.d $r31, $sp, 96
    addi.d $sp, $sp, {continuation_size}
    jr $ra


7:
    # PLV0 入口故障不使用当前不可信 SP 压帧，独立快照保留真正故障 CSR 和故障 SP。
    move $a1, $sp
    csrrd $a0, {csr_era}
    csrrd $a2, {csr_estat}
    csrrd $a3, {csr_badv}
    move $a4, $zero
    b {fault_restore}


.purgem LA_FP_REGS
"#,
            gpr = const offset_of!(UserContext, gpr),
            pc = const offset_of!(UserContext, pc),
            fpr = const offset_of!(UserContext, fpr),
            fcsr = const offset_of!(UserContext, fcsr),
            fcc = const offset_of!(UserContext, fcc),
            estat = const offset_of!(UserContext, estat),
            badv = const offset_of!(UserContext, badv),
            kernel_sp = const offset_of!(UserContext, kernel_sp),
            fault_restore = sym la_user_kernel_fault_restore,
            csr_badv = const reg::CSR_BADV,
            csr_crmd = const reg::CSR_CRMD,
            csr_eentry = const reg::CSR_EENTRY,
            csr_era = const reg::CSR_ERA,
            csr_estat = const reg::CSR_ESTAT,
            csr_euen = const reg::CSR_EUEN,
            csr_ks1 = const reg::CSR_KS1,
            csr_ks2 = const reg::CSR_KS2,
            csr_ks3 = const reg::CSR_KS3,
            csr_prmd = const reg::CSR_PRMD,
            reg_euen_lasx = const reg::EUEN_LASX,
            reg_euen_lsx = const reg::EUEN_LSX,
            reg_prmd_pplv = const reg::PRMD_PPLV,
            continuation_crmd = const CONTINUATION_CRMD,
            continuation_eentry = const CONTINUATION_EENTRY,
            continuation_era = const CONTINUATION_ERA,
            continuation_ks1 = const CONTINUATION_KS1,
            continuation_ks2 = const CONTINUATION_KS2,
            continuation_ks3 = const CONTINUATION_KS3,
            continuation_prmd = const CONTINUATION_PRMD,
            continuation_size = const CONTINUATION_SIZE,
            fp_stride = const size_of::<crate::arch::loongarch64::user::FpRegister>(),
        );
    }
}

/// 接收用户入口窗口的内核 refill，在 DA 下快照专属 CSR，先跳高别名再恢复 DMW。
///
/// # Safety
/// 只能由 refill 入口在确认 EENTRY 为用户向量后跳入；KS3 必须是本次调用仍保活的受信续接，当前 SP 不能用于压帧。
/// TLBRERA/TLBRBADV 必须仍是原故障现场，普通 Rust 调用只能发生在高地址和受信栈恢复之后。
#[naked]
#[no_mangle]
pub(in crate::arch::loongarch64) unsafe extern "C" fn la_user_kernel_refill_fault() -> ! {
    // SAFETY: refill 前驱已确认用户入口窗口且保活 KS3；此段不使用当前 SP 压帧，先完成 DA 到高别名的转换。
    unsafe {
        naked_asm!(
            r#"
    # refill 保留专属 CSR 语义；DA 下只跳转高地址别名，再恢复 PLV0 DMW。
    move $a1, $sp
    csrrd $a0, {csr_tlbrera}
    andi $a2, $a0, {reg_tlbrera_is_tlbr}
    srli.d $a0, $a0, 1
    slli.d $a0, $a0, 1
    csrrd $a3, {csr_tlbrbadv}
    li.w $a4, 1
    la.abs $t0, 2f
    jirl $zero, $t0, 0
2:
    li.w $t0, {reg_crmd_pg}
    li.w $t1, {reg_crmd_address_mode}
    csrxchg $t0, $t1, {csr_crmd}
    b {fault_restore}
"#,
            fault_restore = sym la_user_kernel_fault_restore,
            csr_crmd = const reg::CSR_CRMD,
            csr_tlbrbadv = const reg::CSR_TLBRBADV,
            csr_tlbrera = const reg::CSR_TLBRERA,
            reg_crmd_address_mode = const reg::CRMD_ADDRESS_MODE,
            reg_crmd_pg = const reg::CRMD_PG,
            reg_tlbrera_is_tlbr = const reg::TLBRERA_IS_TLBR,
        );
    }
}

/// 从 KS3 恢复受信 Task 栈和入口内核状态，保留 a0–a4 的原故障 pc/sp/cause/bad/refill 值交给 fatal。
///
/// # Safety
/// 只能由两个入口故障分支跳入；DMW 必须已可用，KS3 指向本次有效 160 字节续接帧，a0–a4 已快照原故障。
/// 不返回原用户现场，不得将故障 SP 当作 Rust 栈；目的 CPU 的 KS0 不得被覆盖。
#[naked]
#[no_mangle]
unsafe extern "C" fn la_user_kernel_fault_restore(
    _pc: usize,
    _sp: usize,
    _cause: usize,
    _bad: usize,
    _is_refill: usize,
) -> ! {
    // SAFETY: 两个故障分支已恢复 DMW 并快照 a0–a4；KS3 仍指向有效续接，恢复内核状态后才能跳入 fatal。
    unsafe {
        naked_asm!(
            r#"
    csrrd $sp, {csr_ks3}
    ld.d $tp, $sp, 8
    ld.d $r21, $sp, 16
    csrwr $zero, {csr_euen}
    ld.d $t0, $sp, {continuation_eentry}
    csrwr $t0, {csr_eentry}
    ld.d $t0, $sp, {continuation_prmd}
    csrwr $t0, {csr_prmd}
    ld.d $t0, $sp, {continuation_era}
    csrwr $t0, {csr_era}
    ld.d $t0, $sp, {continuation_ks1}
    csrwr $t0, {csr_ks1}
    ld.d $t0, $sp, {continuation_ks2}
    csrwr $t0, {csr_ks2}
    ld.d $t0, $sp, {continuation_ks3}
    csrwr $t0, {csr_ks3}
    ld.d $t0, $sp, {continuation_crmd}
    csrwr $t0, {csr_crmd}
    bl {handler}
"#,
            handler = sym loongarch_user_kernel_fault,
            csr_crmd = const reg::CSR_CRMD,
            csr_eentry = const reg::CSR_EENTRY,
            csr_era = const reg::CSR_ERA,
            csr_euen = const reg::CSR_EUEN,
            csr_ks1 = const reg::CSR_KS1,
            csr_ks2 = const reg::CSR_KS2,
            csr_ks3 = const reg::CSR_KS3,
            csr_prmd = const reg::CSR_PRMD,
            continuation_crmd = const CONTINUATION_CRMD,
            continuation_eentry = const CONTINUATION_EENTRY,
            continuation_era = const CONTINUATION_ERA,
            continuation_ks1 = const CONTINUATION_KS1,
            continuation_ks2 = const CONTINUATION_KS2,
            continuation_ks3 = const CONTINUATION_KS3,
            continuation_prmd = const CONTINUATION_PRMD,
        );
    }
}

#[cfg(feature = "ktest")]
global_asm!(
    r#"
# /// 用户消费者从独立输入加载实际高 lanes 并写回结果，浮点循环不依赖内核初值。
.macro LA_PROBE_VECTOR
    li.w $t0, 2
    cpucfg $t0, $t0
    andi $t1, $t0, 0x80
    bnez $t1, .Lprobe_lasx\@
    andi $t1, $t0, 0x40
    beqz $t1, .Lprobe_scalar\@
    vld $vr31, $a6, 96
    vst $vr31, $a6, 128
    b .Lprobe_vector_done\@
.Lprobe_lasx\@:
    xvld $xr31, $a6, 96
    xvst $xr31, $a6, 128
    b .Lprobe_vector_done\@
.Lprobe_scalar\@:
    fld.d $f31, $a6, 96
    fst.d $f31, $a6, 128
.Lprobe_vector_done\@:
.endm

.section .text.la_user_probe, "ax"
.balign 4096
.global la_user_code_start
.global la_user_code_end
.global la_user_loop_entry
.global la_user_loop_spin
.global la_user_exit_entry
.global la_user_fault_read
.global la_user_fault_write
.global la_user_fault_execute
la_user_code_start:
    # 页内仅用相对分支和寄存器地址，复制后不依赖原内核虚拟地址。
    st.d $sp, $a6, 0
    st.d $tp, $a6, 8
    st.d $r21, $a6, 16
    addi.d $sp, $sp, -16
    st.d $tp, $sp, 0
    ld.d $t0, $sp, 0
    addi.d $sp, $sp, 16
    st.d $t0, $a6, 40
    fadd.d $f2, $f0, $f1
    fst.d $f2, $a6, 24
    movfcsr2gr $t0, $fcsr0
    st.d $t0, $a6, 32
    fcmp.clt.d $fcc3, $f0, $f1
    movcf2gr $t0, $fcc3
    st.d $t0, $a6, 48
    fcmp.clt.d $fcc7, $f1, $f0
    movcf2gr $t0, $fcc7
    st.d $t0, $a6, 56
    li.w $a7, 93
    li.w $a0, 41
    syscall 0
    li.w $a0, 42
    syscall 0
.Lla_probe_completed:
    b .Lla_probe_completed

la_user_loop_entry:
    st.d $sp, $a6, 0
    st.d $tp, $a6, 8
    st.d $r21, $a6, 16
    addi.d $sp, $sp, -16
    st.d $tp, $sp, 0
    ld.d $t0, $sp, 0
    addi.d $sp, $sp, 16
    st.d $t0, $a6, 40
    fadd.d $f2, $f0, $f1
    fst.d $f2, $a6, 24
    movfcsr2gr $t0, $fcsr0
    st.d $t0, $a6, 32
    fcmp.clt.d $fcc3, $f0, $f1
    movcf2gr $t0, $fcc3
    st.d $t0, $a6, 48
    fcmp.clt.d $fcc7, $f1, $f0
    movcf2gr $t0, $fcc7
    st.d $t0, $a6, 56
    LA_PROBE_VECTOR
la_user_loop_spin:
    b la_user_loop_spin

la_user_exit_entry:
    li.w $a7, 93
    li.w $a0, 43
    syscall 0
    b .Lla_probe_completed

la_user_fault_read:
    ld.d $t0, $a5, 0
    b la_user_exit_entry
la_user_fault_write:
    # 先使同一只读页进入 TLB，再执行 store 触发 modify 而非仅观察首次 refill。
    ld.d $t0, $a5, 0
    st.d $zero, $a5, 0
    b la_user_exit_entry
la_user_fault_execute:
    jirl $zero, $a5, 0
la_user_code_end:
.purgem LA_PROBE_VECTOR
"#,
);
