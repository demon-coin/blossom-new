//! LoongArch64 用户态进出切换、陷入保存与内核入口故障恢复汇编实现。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::loongarch64::user` 底层，提供 [`la_run_user`]（进入 PLV3）、[`la_user_trap`]
//! （从 PLV3 异常/中断返回 PLV0）以及 [`la_user_kernel_refill_fault`] / [`la_user_kernel_fault_restore`]
//! （处理切换窗口内的 PLV0 致命异常），向上由 [`super::run_user`] 及顶层 TLB 重填分发器调用。
//!
//! # 核心设计与 160 字节内核栈续接帧（Continuation Frame）
//! 调用 [`la_run_user`] 时，汇编在当前任务的内核栈上压入一个 160 字节（20 个 `usize`）的续接帧，
//! 并将其栈顶地址同时保存在 [`UserContext::kernel_sp`] 与 `CSR_KS3` 中：
//!
//! ```text
//! 高地址（调用 la_run_user 时的原内核栈顶 SP）
//! +---------------------------------------------------------------+ 偏移 160 (CONTINUATION_SIZE)
//! | [19] CSR_CRMD   (8 B)  入口前当前模式信息                     | 偏移 152 (CONTINUATION_CRMD)
//! | [18] CSR_KS2    (8 B)  入口前 KS2 暂存寄存器                  | 偏移 144 (CONTINUATION_KS2)
//! | [17] CSR_KS1    (8 B)  入口前 KS1 暂存寄存器                  | 偏移 136 (CONTINUATION_KS1)
//! | [16] CSR_ERA    (8 B)  入口前异常返回地址                     | 偏移 128 (CONTINUATION_ERA)
//! | [15] CSR_PRMD   (8 B)  入口前例外前模式信息                   | 偏移 120 (CONTINUATION_PRMD)
//! | [14] CSR_EENTRY (8 B)  入口前内核异常向量基址                 | 偏移 112 (CONTINUATION_EENTRY)
//! | [13] CSR_KS3    (8 B)  入口前外层 KS3 值                      | 偏移 104 (CONTINUATION_KS3)
//! +---------------------------------------------------------------+
//! | [0..12] 被调用者保存及内核专有 GPR (13 * 8 B = 104 B):        |
//! |   - [0]  r1  (ra)      - [1]  r2  (tp)      - [2]  r21        |
//! |   - [3]  r22 (fp/s9)   - [4]  r23 (s0)      - [5]  r24 (s1)   |
//! |   - [6]  r25 (s2)      - [7]  r26 (s3)      - [8]  r27 (s4)   |
//! |   - [9]  r28 (s5)      - [10] r29 (s6)      - [11] r30 (s7)   |
//! |   - [12] r31 (s8)                                             |
//! +---------------------------------------------------------------+ 偏移 0 <- kernel_sp & CSR_KS3
//! 低地址
//! ```
//!
//! ## `CSR_KS0`–`CSR_KS3` 暂存寄存器分工
//! - `CSR_KS0`：永久存放当前 CPU 的 `CpuLocal` 指针，任何用户态切换或故障路径均不得覆盖；
//! - `CSR_KS1`：在 PLV3 运行期间存放当前任务的 `*mut UserContext` 指针；进入 [`la_user_trap`] 时
//!   与 `$sp` 互换，从而在不破坏用户通用寄存器的前提下以 `$sp` 作为基址保存用户上下文；
//! - `CSR_KS2`：在 [`la_user_trap`] 入口处临时保存用户态 `$t0`（`$r12`），以便用 `$t0` 检查 `CSR_PRMD`；
//! - `CSR_KS3`：在整个用户态运行及进出切换窗口内，始终指向上述 160 字节可信内核续接帧。即使切换期间
//!   `$sp` 指向 `UserContext` 或用户栈并意外触发 PLV0 异常/TLB 重填，故障路径也能直接从 `CSR_KS3` 找回可信内核栈。
//!
//! # 锁获取顺序与并发契约
//! - 本模块所有汇编入口均在本地 CPU 关中断状态下执行，内部不获取任何锁。

use crate::arch::loongarch64::register as reg;
use crate::arch::loongarch64::user::{loongarch_user_kernel_fault, UserContext};
#[cfg(feature = "ktest")]
use core::arch::global_asm;
use core::arch::naked_asm;
use core::mem::{offset_of, size_of};

/// 内核栈续接帧中保存的通用寄存器槽位数（`r1`, `r2`, `r21`..=`r31`，共 13 个寄存器）。
const CONTINUATION_GPRS: usize = 13;
/// 续接帧中保存入口前 `CSR_KS3` 的字节偏移（104）。
const CONTINUATION_KS3: usize = CONTINUATION_GPRS * core::mem::size_of::<usize>();
/// 续接帧中保存入口前 `CSR_EENTRY` 的字节偏移（112）。
const CONTINUATION_EENTRY: usize = CONTINUATION_KS3 + core::mem::size_of::<usize>();
/// 续接帧中保存入口前 `CSR_PRMD` 的字节偏移（120）。
const CONTINUATION_PRMD: usize = CONTINUATION_EENTRY + core::mem::size_of::<usize>();
/// 续接帧中保存入口前 `CSR_ERA` 的字节偏移（128）。
const CONTINUATION_ERA: usize = CONTINUATION_PRMD + core::mem::size_of::<usize>();
/// 续接帧中保存入口前 `CSR_KS1` 的字节偏移（136）。
const CONTINUATION_KS1: usize = CONTINUATION_ERA + core::mem::size_of::<usize>();
/// 续接帧中保存入口前 `CSR_KS2` 的字节偏移（144）。
const CONTINUATION_KS2: usize = CONTINUATION_KS1 + core::mem::size_of::<usize>();
/// 续接帧中保存入口前 `CSR_CRMD` 的字节偏移（152）。
const CONTINUATION_CRMD: usize = CONTINUATION_KS2 + core::mem::size_of::<usize>();
/// 内核栈续接帧的总字节大小（160 字节，满足 16 字节栈对齐要求）。
const CONTINUATION_SIZE: usize = CONTINUATION_CRMD + core::mem::size_of::<usize>();

/// 在当前任务内核栈上保存 160 字节续接帧，加载 [`UserContext`] 中的用户态寄存器并执行 `ertn` 进入 PLV3；
/// 当用户态触发异常或中断陷入 [`la_user_trap`] 后，恢复续接帧并从本函数返回。
///
/// # Safety
/// 1. `_context`（`$a0`）必须指向当前任务独占、已完成初始化且 32 字节对齐的 [`UserContext`]；
/// 2. 当前激活的用户页表（`Mm`）、当前任务内核栈及当前 CPU 的 `CSR_KS0`（`CpuLocal`）必须在整个调用期间有效；
/// 3. 调用时当前 CPU 必须处于关中断状态且 `CSR_EUEN == 0`（内核态未占用浮点/向量寄存器），内核栈顶 `$sp` 满足 16 字节对齐。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_run_user(_context: *mut UserContext) {
    // SAFETY:
    // 1. 调用方 `super::run_user` 已通过 `assert!` 确认本地中断与 `CSR_EUEN` 关闭，并传入独占的 `UserContext` 可变引用；
    // 2. 裸汇编在当前内核栈上分配 160 字节续接帧并记录到 `context.kernel_sp` 与 `CSR_KS3`，在 `la_user_trap` 中配对弹出并恢复所有被调用者保存寄存器及 CSR。
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

/// 用户态（PLV3）运行期间的异常与中断统一入口向量（安装于 `CSR_EENTRY`）。
///
/// - 若 `CSR_PRMD.PPLV != 0`（来自用户态）：将全部用户通用寄存器、PC、`ESTAT`、`BADV` 及当前开启的
///   FP/LSX/LASX 浮点状态保存到 `CSR_KS1` 指向的 [`UserContext`]，随后关闭 `CSR_EUEN`，从 `kernel_sp`
///   恢复内核栈续接帧及入口 CSR，最终执行 `jr $ra` 返回到 [`la_run_user`] 的调用点。
/// - 若 `CSR_PRMD.PPLV == 0`（在进出用户态的内核切换窗口内意外触发同步异常）：不触碰当前不可信的 `$sp`，
///   直接将故障 `ERA`/`SP`/`ESTAT`/`BADV` 快照到 `$a0`–`$a4`，跳转到 [`la_user_kernel_fault_restore`]。
///
/// # Safety
/// 1. 只能作为硬件异常入口由 CPU 硬件在 `CSR_EENTRY` 指向本函数时跳入；
/// 2. 进入时 `CSR_KS1` 必须指向当前任务的 [`UserContext`]，`CSR_KS3` 必须指向有效的 160 字节内核栈续接帧，`CSR_KS0` 必须保持为当前 CPU 的 `CpuLocal`。
#[naked]
#[no_mangle]
#[link_section = ".text.vector.user_trap"]
pub(in crate::arch::loongarch64) unsafe extern "C" fn la_user_trap() -> ! {
    // SAFETY:
    // 1. `la_run_user` 在将 `CSR_EENTRY` 指向本函数前，已将 `UserContext` 地址写入 `CSR_KS1`、将内核续接帧栈顶写入 `CSR_KS3`；
    // 2. 用户态陷入分支仅向 `UserContext` 写入寄存器快照，内核态故障分支不向当前 `$sp` 压栈而是直接恢复 `CSR_KS3` 可信栈。
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

/// 处理用户异常向量激活窗口内发生的内核态（PLV0）TLB 重填故障。
///
/// 当 CPU 因 TLB Refill 进入直接地址翻译模式（DA 模式）且顶层重填入口判定故障发生在用户态切换窗口的内核代码中时，
/// 跳转到本函数：先读取 `CSR_TLBRERA` 与 `CSR_TLBRBADV` 构造故障参数，再通过高地址别名跳转并切回映射地址模式（PG/DMW），
/// 最后交由 [`la_user_kernel_fault_restore`] 恢复内核栈并报告致命错误。
///
/// # Safety
/// 1. 只能由 TLB Refill 汇编入口在确认 `CSR_EENTRY == la_user_trap` 且故障发生在内核态时跳入；
/// 2. `CSR_KS3` 必须指向当前任务有效的 160 字节内核栈续接帧，`CSR_TLBRERA` 与 `CSR_TLBRBADV` 仍保持原始重填故障现场。
#[naked]
#[no_mangle]
pub(in crate::arch::loongarch64) unsafe extern "C" fn la_user_kernel_refill_fault() -> ! {
    // SAFETY:
    // 1. 调用方（TLB Refill 入口）已确认当前处于用户切换窗口且 `CSR_KS3` 保存了可信内核栈续接帧；
    // 2. 本函数在 DA 模式下不使用当前不可信 `$sp` 访存，先跳转到 DMW 高地址别名并恢复 `CRMD.PG` 后，再进入 `la_user_kernel_fault_restore`。
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

/// 从 `CSR_KS3` 恢复可信的内核任务栈与入口 CSR 状态，并携带 `$a0`–`$a4` 中的故障现场调用 [`loongarch_user_kernel_fault`]。
///
/// # Safety
/// 1. 只能由 [`la_user_trap`] 的 PLV0 故障分支或 [`la_user_kernel_refill_fault`] 跳入；
/// 2. 进入时内核高地址直接映射窗口（DMW）与分页模式（`CRMD.PG`）必须可用，`CSR_KS3` 必须指向有效的 160 字节续接帧；
/// 3. `$a0`–`$a4` 已依次填入故障 `pc`、`fault_sp`、`cause`、`badv` 与 `is_refill` 标志。
#[naked]
#[no_mangle]
unsafe extern "C" fn la_user_kernel_fault_restore(
    _pc: usize,
    _sp: usize,
    _cause: usize,
    _bad: usize,
    _is_refill: usize,
) -> ! {
    // SAFETY:
    // 1. 两个唯一前驱分支已确保 DMW 可用且 `$a0`–`$a4` 保存了完整故障参数；
    // 2. `CSR_KS3` 在 `la_run_user` 入口处被初始化为 16 字节对齐的内核续接帧栈顶，从中恢复 `$sp`、`$tp`、`$r21` 及入口 CSR 后即可安全调用 Rust `loongarch_user_kernel_fault`。
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
