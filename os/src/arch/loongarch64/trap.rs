//! LoongArch64 内核态普通异常/中断入口（`la_kernel_trap`）与 TLB 重填物理入口（`la_kernel_refill`）。
//!
//! ## 模块职责与架构定位
//! 本模块提供安装在 `CSR_EENTRY` 与 `CSR_TLBRENTRY` 上的两个硬件向量裸汇编入口：
//! 1. `la_kernel_trap`：处理 CPU 在内核态（PLV0）执行期间触发的硬件中断与同步异常，将寄存器现场保存为栈上的 `KernelTrapFrame`，调用 `loongarch_trap` 分派处理后再通过 `ertn` 返回。
//! 2. `la_kernel_refill`：响应全机（含用户态 PLV3 与内核态 PLV0）的 TLB 重填异常（TLB Refill）。由于硬件在触发 TLB 重填时会自动切换到直接地址翻译模式（DA，`CRMD.DA = 1, PG = 0`）并跳转到 `CSR_TLBRENTRY` 物理地址，且使用独立的一套 CSR（`TLBRERA`、`TLBRPRMD`、`TLBRBADV`、`TLBREHI`、`TLBRELO0/1`、`TLBRSAVE`），因此不能与普通异常混用同一套保存栈帧。
//!
//! ## 核心机制与 TLB 重填分流流程
//! ```text
//!                        [硬件触发 TLB Refill 异常]
//!            (自动进入 PLV0 + DA 物理寻址模式，跳入 la_kernel_refill)
//!                                    │
//!                    检查 TLBRPRMD.PPLV 是否为用户态 (PLV3)?
//!                       ┌────────────┴────────────┐
//!                  是 (PLV3)                  否 (PLV0)
//!                       │                         │
//!                       ▼                         ▼
//!           检查 TLBRBADV 是否 < 2^39?     检查 EENTRY == la_user_trap?
//!              ┌────────┴────────┐           ┌────┴─────────────┐
//!             是                 否          是                 否
//!              ▼                 ▼           ▼                  ▼
//!          读取 PGDL       加载 LA_INVALID_ROOT  跳入 la_user_kernel_refill_fault  标签 6 (致命内核 Refill):
//!              └────────┬────────┘        (借用 KS3 切换回内核栈转为普通页故障)    1. 在当前内核栈压入 RefillFrame
//!                       ▼                                                          2. 跳转到高地址并恢复 CRMD.PG=1
//!        执行 lddir (Level 3) + lddir (Level 1)                                    3. 调用 loongarch_refill 报告致命错误
//!        加载偶奇双页表项写入 TLBRELO0 / TLBRELO1
//!        执行 tlbfill 填入 TLB -> ertn 返回用户态
//! ```
//!
//! ## 临时寄存器（Scratch CSR）分配约定
//! - `CSR_KS0`：永久保存当前 CPU 的 `CpuLocal` 指针（在 `la_kernel_refill` 中严禁破坏）。
//! - `CSR_KS1` / `CSR_KS3`：由 `user.rs` 在进入用户态前保存用户上下文指针与内核续接栈顶，供用户态异常入口与用户态窗口内的 Refill 故障路径读取。
//! - `CSR_TLBRSAVE`：在 `la_kernel_refill` 入口暂存通用寄存器 `$t0`。
//! - `CSR_KS2`：在 `la_kernel_refill` 内部暂存通用寄存器 `$t1`，在执行 `ertn` 或构造 `RefillFrame` 前恢复。
//!
//! ## 并发与锁契约
//! - 硬件进入 `la_kernel_trap` 或 `la_kernel_refill` 时会自动清除 `CRMD.IE`（关闭本核硬件中断）。
//! - 汇编入口本身不获取任何锁；页表遍历指令 `lddir` 在本核关闭中断状态下只读访问当前激活的页表页或静态只读的 `LA_INVALID_ROOT` 页表链。

use crate::arch::loongarch64::mmu::{PHYSICAL_BITS, PT_INDEX_BITS, USER_ADDRESS_BITS};
use crate::arch::loongarch64::register as reg;
use crate::arch::loongarch64::user::{la_user_kernel_refill_fault, la_user_trap};
use crate::arch::loongarch64::{loongarch_refill, loongarch_trap, KernelTrapFrame, RefillFrame};
use crate::mm::PAGE_SHIFT;
use core::arch::naked_asm;

/// `lddir` 指令遍历三级用户页表时的顶层目录级别编号（Level 3，对应 `PWCH.Dir3` 配置的 PGD）。
const LDDIR_ROOT: usize = 3;
/// `lddir` 指令遍历三级用户页表时的中间目录级别编号（Level 1，对应 `PWCL.Dir1` 配置的 PMD）。
const LDDIR_MIDDLE: usize = 1;
use core::mem::{offset_of, size_of};

extern "C" {
    /// 静态全无效三级页表根（定义于 `mmu.rs`），当用户态访问超出 39 位有效虚址范围时作为兜底页表根。
    static LA_INVALID_ROOT: u8;
}

/// 内核态（PLV0）普通中断与异常的硬件向量入口：在当前内核栈上构造 `KernelTrapFrame`，调用 `loongarch_trap` 分派处理后恢复现场并执行 `ertn`。
///
/// 当进入异常时若浮点扩展单元处于开启状态（`CSR_EUEN.FPE != 0`），一并保存和恢复 32 个 64 位浮点寄存器（`$f0`–`$f31`）、`FCSR0` 与 8 个条件标志寄存器（`$fcc0`–`$fcc7`）；若浮点单元未启用则跳过浮点区域读写。
///
/// # Safety
/// - 只能作为 `CSR_EENTRY` 的内核态硬件中断/异常入口由 CPU 硬件直接跳入，进入时必须处于 PLV0 且 `CRMD.IE == 0`。
/// - 当前 `$sp` 必须指向本核独占、16 字节对齐且剩余空间足以容纳 `KernelTrapFrame` 的有效内核栈，且 DMW0/DMW1 与 `CSR_KS0` 已正确初始化。
/// - 下游 Rust 处理函数 `loongarch_trap` 严禁跨此裸汇编栈帧发生栈展开（unwind）。
#[naked]
#[no_mangle]
#[link_section = ".text.vector.kernel_trap"]
pub(super) unsafe extern "C" fn la_kernel_trap() -> ! {
    // SAFETY: 1. 硬件在 PLV0 触发异常或中断时进入本入口，当前 `$sp` 为有效内核栈；
    // 2. 汇编严格按照 `KernelTrapFrame` 的字段偏移保存全部 32 个通用寄存器、异常相关 CSR（ERA/PRMD/ESTAT/BADV/EUEN/KS0）以及条件启用的浮点寄存器；
    // 3. 将栈顶指针通过 `$a0` 传给 `loongarch_trap(&mut KernelTrapFrame)`，返回后按相反顺序恢复寄存器并执行 `ertn`。
    unsafe {
        naked_asm!(
                r#"
    addi.d $sp, $sp, -{frame_size}
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
    st.d $r12, $sp, {gpr}+12*8
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
    addi.d $t0, $sp, {frame_size}
    st.d $t0, $sp, {gpr}+3*8
    csrrd $t0, {csr_era}
    st.d $t0, $sp, {era}
    csrrd $t0, {csr_prmd}
    st.d $t0, $sp, {prmd}
    csrrd $t0, {csr_estat}
    st.d $t0, $sp, {estat}
    csrrd $t0, {csr_badv}
    st.d $t0, $sp, {badv}
    csrrd $t0, {csr_euen}
    st.d $t0, $sp, {euen}
    csrrd $t0, {csr_ks0}
    st.d $t0, $sp, {cpu_base}

    ld.d $t0, $sp, {euen}
    andi $t0, $t0, {reg_euen_fp}
    beqz $t0, 2f
    fst.d $f0, $sp, {fpr}+0*8
    fst.d $f1, $sp, {fpr}+1*8
    fst.d $f2, $sp, {fpr}+2*8
    fst.d $f3, $sp, {fpr}+3*8
    fst.d $f4, $sp, {fpr}+4*8
    fst.d $f5, $sp, {fpr}+5*8
    fst.d $f6, $sp, {fpr}+6*8
    fst.d $f7, $sp, {fpr}+7*8
    fst.d $f8, $sp, {fpr}+8*8
    fst.d $f9, $sp, {fpr}+9*8
    fst.d $f10, $sp, {fpr}+10*8
    fst.d $f11, $sp, {fpr}+11*8
    fst.d $f12, $sp, {fpr}+12*8
    fst.d $f13, $sp, {fpr}+13*8
    fst.d $f14, $sp, {fpr}+14*8
    fst.d $f15, $sp, {fpr}+15*8
    fst.d $f16, $sp, {fpr}+16*8
    fst.d $f17, $sp, {fpr}+17*8
    fst.d $f18, $sp, {fpr}+18*8
    fst.d $f19, $sp, {fpr}+19*8
    fst.d $f20, $sp, {fpr}+20*8
    fst.d $f21, $sp, {fpr}+21*8
    fst.d $f22, $sp, {fpr}+22*8
    fst.d $f23, $sp, {fpr}+23*8
    fst.d $f24, $sp, {fpr}+24*8
    fst.d $f25, $sp, {fpr}+25*8
    fst.d $f26, $sp, {fpr}+26*8
    fst.d $f27, $sp, {fpr}+27*8
    fst.d $f28, $sp, {fpr}+28*8
    fst.d $f29, $sp, {fpr}+29*8
    fst.d $f30, $sp, {fpr}+30*8
    fst.d $f31, $sp, {fpr}+31*8
    movfcsr2gr $t0, $fcsr0
    st.d $t0, $sp, {fcsr}
    movcf2gr $t0, $fcc0
    st.d $t0, $sp, {fcc}+0*8
    movcf2gr $t0, $fcc1
    st.d $t0, $sp, {fcc}+1*8
    movcf2gr $t0, $fcc2
    st.d $t0, $sp, {fcc}+2*8
    movcf2gr $t0, $fcc3
    st.d $t0, $sp, {fcc}+3*8
    movcf2gr $t0, $fcc4
    st.d $t0, $sp, {fcc}+4*8
    movcf2gr $t0, $fcc5
    st.d $t0, $sp, {fcc}+5*8
    movcf2gr $t0, $fcc6
    st.d $t0, $sp, {fcc}+6*8
    movcf2gr $t0, $fcc7
    st.d $t0, $sp, {fcc}+7*8

2:
    move $a0, $sp
    bl {handler}
    ld.d $t0, $sp, {euen}
    csrwr $t0, {csr_euen}
    ld.d $t0, $sp, {euen}
    andi $t0, $t0, {reg_euen_fp}
    beqz $t0, 3f
    fld.d $f0, $sp, {fpr}+0*8
    fld.d $f1, $sp, {fpr}+1*8
    fld.d $f2, $sp, {fpr}+2*8
    fld.d $f3, $sp, {fpr}+3*8
    fld.d $f4, $sp, {fpr}+4*8
    fld.d $f5, $sp, {fpr}+5*8
    fld.d $f6, $sp, {fpr}+6*8
    fld.d $f7, $sp, {fpr}+7*8
    fld.d $f8, $sp, {fpr}+8*8
    fld.d $f9, $sp, {fpr}+9*8
    fld.d $f10, $sp, {fpr}+10*8
    fld.d $f11, $sp, {fpr}+11*8
    fld.d $f12, $sp, {fpr}+12*8
    fld.d $f13, $sp, {fpr}+13*8
    fld.d $f14, $sp, {fpr}+14*8
    fld.d $f15, $sp, {fpr}+15*8
    fld.d $f16, $sp, {fpr}+16*8
    fld.d $f17, $sp, {fpr}+17*8
    fld.d $f18, $sp, {fpr}+18*8
    fld.d $f19, $sp, {fpr}+19*8
    fld.d $f20, $sp, {fpr}+20*8
    fld.d $f21, $sp, {fpr}+21*8
    fld.d $f22, $sp, {fpr}+22*8
    fld.d $f23, $sp, {fpr}+23*8
    fld.d $f24, $sp, {fpr}+24*8
    fld.d $f25, $sp, {fpr}+25*8
    fld.d $f26, $sp, {fpr}+26*8
    fld.d $f27, $sp, {fpr}+27*8
    fld.d $f28, $sp, {fpr}+28*8
    fld.d $f29, $sp, {fpr}+29*8
    fld.d $f30, $sp, {fpr}+30*8
    fld.d $f31, $sp, {fpr}+31*8
    ld.d $t0, $sp, {fcsr}
    movgr2fcsr $fcsr0, $t0
    ld.d $t0, $sp, {fcc}+0*8
    movgr2cf $fcc0, $t0
    ld.d $t0, $sp, {fcc}+1*8
    movgr2cf $fcc1, $t0
    ld.d $t0, $sp, {fcc}+2*8
    movgr2cf $fcc2, $t0
    ld.d $t0, $sp, {fcc}+3*8
    movgr2cf $fcc3, $t0
    ld.d $t0, $sp, {fcc}+4*8
    movgr2cf $fcc4, $t0
    ld.d $t0, $sp, {fcc}+5*8
    movgr2cf $fcc5, $t0
    ld.d $t0, $sp, {fcc}+6*8
    movgr2cf $fcc6, $t0
    ld.d $t0, $sp, {fcc}+7*8
    movgr2cf $fcc7, $t0

3:
    ld.d $t0, $sp, {era}
    csrwr $t0, {csr_era}
    ld.d $t0, $sp, {prmd}
    csrwr $t0, {csr_prmd}
    ld.d $t0, $sp, {cpu_base}
    csrwr $t0, {csr_ks0}
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
"#,
        frame_size = const size_of::<KernelTrapFrame>(),
        gpr = const offset_of!(KernelTrapFrame, gpr),
        era = const offset_of!(KernelTrapFrame, era),
        prmd = const offset_of!(KernelTrapFrame, prmd),
        estat = const offset_of!(KernelTrapFrame, estat),
        badv = const offset_of!(KernelTrapFrame, badv),
        euen = const offset_of!(KernelTrapFrame, euen),
        cpu_base = const offset_of!(KernelTrapFrame, cpu_base),
        fpr = const offset_of!(KernelTrapFrame, fpr),
        fcsr = const offset_of!(KernelTrapFrame, fcsr),
        fcc = const offset_of!(KernelTrapFrame, fcc),
                handler = sym loongarch_trap,
            csr_badv = const reg::CSR_BADV,
            csr_era = const reg::CSR_ERA,
            csr_estat = const reg::CSR_ESTAT,
            csr_euen = const reg::CSR_EUEN,
            csr_ks0 = const reg::CSR_KS0,
            csr_prmd = const reg::CSR_PRMD,
            reg_euen_fp = const reg::EUEN_FP,
            );
    }
}

/// LoongArch64 硬件 TLB 重填（TLB Refill）物理入口：在硬件自动切换的 DA 直接地址翻译模式下，快速填充用户态页表项或捕获内核态致命 TLB 重填异常。
///
/// 硬件触发 TLB 重填时会把 `CRMD.DA` 置 1、`CRMD.PG` 清 0，并将故障信息写入 `TLBRERA`、`TLBRPRMD`、`TLBRBADV` 与 `TLBREHI`，不会修改普通异常使用的 `ERA`、`PRMD` 与 `BADV`。
/// - 若故障源于用户态（`TLBRPRMD.PPLV == PLV3`）：利用 `lddir` 指令遍历 `CSR_PGDL`（越界地址则导向 `LA_INVALID_ROOT`），将偶奇相邻的一对 64 位页表项装入 `TLBRELO0`/`TLBRELO1` 后执行 `tlbfill` 并立即返回。
/// - 若故障发生在 `la_user_trap` 刚进入 PLV0 但尚未切换完用户栈/内核栈的窗口期：直接跳转到 `la_user_kernel_refill_fault`，借用 `CSR_KS3` 恢复内核栈并按用户访存异常处理。
/// - 若故障源于普通内核态代码：在当前内核栈上保存 `RefillFrame`，跳回高地址虚拟空间并重新置位 `CRMD.PG`，最后交由 `loongarch_refill` 输出致命错误报告。
///
/// # Safety
/// - 只能将其物理地址写入 `CSR_TLBRENTRY` 供 CPU 硬件在触发 TLB Refill 时直接跳入。
/// - 调用前必须已通过 `mmu::initialize_current_cpu` 配置好 `PWCL`/`PWCH` 页表遍历寄存器，并通过 `mmu::initialize_invalid_tables` 初始化好 `LA_INVALID_ROOT` 兜底页表链。
/// - 若从普通内核态进入，当前 `$sp` 除去高地址 DMW 前缀后的物理地址必须指向合法可写的内核栈；若从 `la_user_trap` 窗口进入，`CSR_KS1` 与 `CSR_KS3` 必须持有有效的用户上下文与内核栈顶。
#[naked]
#[no_mangle]
#[link_section = ".text.vector.kernel_refill"]
pub(super) unsafe extern "C" fn la_kernel_refill() -> ! {
    // SAFETY: 1. 硬件在触发 TLB Refill 时自动进入 PLV0 + DA 模式，并保证 `TLBR*` 系列 CSR 包含当前故障现场；
    // 2. 用户态快速路径仅使用 `TLBRSAVE`（暂存 $t0）与 `CSR_KS2`（暂存 $t1），只读遍历 `CSR_PGDL` 或 `LA_INVALID_ROOT` 后执行 `tlbfill`，不破坏用户栈或 `CSR_KS0`；
    // 3. 内核态致命路径先在 DA 模式下通过物理地址掩码向当前内核栈写入 `RefillFrame`，随后跳转到 DMW1 高地址并恢复 `CRMD.PG = 1`，满足调用 Rust 函数 `loongarch_refill` 的全部 ABI 前提。
    unsafe {
        naked_asm!(
                r#"
    # 用户 refill 把 t0 存入 TLBRSAVE、t1 存入临时 KS2，不触碰用户 SP 或 KS0。
    csrwr $t0, {csr_tlbrsave}
    csrrd $t0, {csr_tlbrprmd}
    andi $t0, $t0, {reg_prmd_pplv}
    xori $t0, $t0, {reg_plv_user}
    beqz $t0, 3f
    # PLV0 的用户入口窗口仍可能持有 context/用户 SP；只在该窗口借 KS3 续接栈。
    # KS2 是非返回的内核 refill 临时槽，原 GPR 在压致命 frame 前恢复。
    csrwr $t1, {csr_ks2}
    csrrd $t0, {csr_eentry}
    la.abs $t1, {user_trap}
    bne $t0, $t1, 2f
    b {user_refill_fault}
2:
    csrrd $t1, {csr_ks2}
    csrrd $t0, {csr_tlbrsave}
    b 6f
3:
    # 三层只索引低 39 位；超出政策范围的地址只能走永久 invalid 链。
    csrrd $t0, {csr_tlbrbadv}
    srli.d $t0, $t0, {user_address_bits}
    bnez $t0, 4f
    csrrd $t0, {csr_pgdl}
    b 5f
4:
    la.abs $t0, {invalid_root}
    bstrpick.d $t0, $t0, {physical_last_bit}, 0
5:
    # PWCH DIR2 / PWCL DIR0 / PT 构成固定三级；空目录总指向永久 invalid 页。
    lddir $t0, $t0, {lddir_root}
    lddir $t0, $t0, {lddir_middle}
    # QEMU 9.2.1 的 LDPTE 会截断高权限位；固定 4KiB 叶以完整 64 位装入偶奇 CSR。
    # KS2 只暂存 t1，真正用户 trap 会重新保存 t0；TLBREHI 保留本次 VPPN 和既有 PS12。
    csrwr $t1, {csr_ks2}
    csrrd $t1, {csr_tlbrbadv}
    bstrpick.d $t1, $t1, {pair_index_last}, {pair_index_first}
    slli.d $t1, $t1, {pair_stride_shift}
    add.d $t0, $t0, $t1
    ld.d $t1, $t0, 8
    csrwr $t1, {csr_tlbrelo1}
    ld.d $t0, $t0, 0
    csrwr $t0, {csr_tlbrelo0}
    csrrd $t1, {csr_ks2}
    tlbfill
    csrrd $t0, {csr_tlbrsave}
    ertn

6:
    # refill 自动处于 DA；高地址栈仍以物理掩码可访问。
    addi.d $sp, $sp, -{refill_size}
    st.d $r0, $sp, {refill_gpr}+0*8
    st.d $r1, $sp, {refill_gpr}+1*8
    st.d $r2, $sp, {refill_gpr}+2*8
    st.d $r4, $sp, {refill_gpr}+4*8
    st.d $r5, $sp, {refill_gpr}+5*8
    st.d $r6, $sp, {refill_gpr}+6*8
    st.d $r7, $sp, {refill_gpr}+7*8
    st.d $r8, $sp, {refill_gpr}+8*8
    st.d $r9, $sp, {refill_gpr}+9*8
    st.d $r10, $sp, {refill_gpr}+10*8
    st.d $r11, $sp, {refill_gpr}+11*8
    st.d $r12, $sp, {refill_gpr}+12*8
    st.d $r13, $sp, {refill_gpr}+13*8
    st.d $r14, $sp, {refill_gpr}+14*8
    st.d $r15, $sp, {refill_gpr}+15*8
    st.d $r16, $sp, {refill_gpr}+16*8
    st.d $r17, $sp, {refill_gpr}+17*8
    st.d $r18, $sp, {refill_gpr}+18*8
    st.d $r19, $sp, {refill_gpr}+19*8
    st.d $r20, $sp, {refill_gpr}+20*8
    st.d $r21, $sp, {refill_gpr}+21*8
    st.d $r22, $sp, {refill_gpr}+22*8
    st.d $r23, $sp, {refill_gpr}+23*8
    st.d $r24, $sp, {refill_gpr}+24*8
    st.d $r25, $sp, {refill_gpr}+25*8
    st.d $r26, $sp, {refill_gpr}+26*8
    st.d $r27, $sp, {refill_gpr}+27*8
    st.d $r28, $sp, {refill_gpr}+28*8
    st.d $r29, $sp, {refill_gpr}+29*8
    st.d $r30, $sp, {refill_gpr}+30*8
    st.d $r31, $sp, {refill_gpr}+31*8
    addi.d $t0, $sp, {refill_size}
    st.d $t0, $sp, {refill_gpr}+3*8
    csrrd $t0, {csr_tlbrera}
    st.d $t0, $sp, {refill_era}
    csrrd $t0, {csr_tlbrprmd}
    st.d $t0, $sp, {refill_prmd}
    csrrd $t0, {csr_tlbrbadv}
    st.d $t0, $sp, {refill_badv}
    # TLBRENTRY 是物理入口；DA 下先转高别名再恢复 DMW。
    la.abs $t0, 7f
    jirl $zero, $t0, 0
7:
    li.w $t0, {reg_crmd_pg}
    li.w $t1, {reg_crmd_address_mode}
    csrxchg $t0, $t1, {csr_crmd}
    move $a0, $sp
    bl {handler}
"#,
        refill_size = const size_of::<RefillFrame>(),
        refill_gpr = const offset_of!(RefillFrame, gpr),
        refill_era = const offset_of!(RefillFrame, era),
        refill_prmd = const offset_of!(RefillFrame, prmd),
        refill_badv = const offset_of!(RefillFrame, badv),
                user_trap = sym la_user_trap,
                user_refill_fault = sym la_user_kernel_refill_fault,
                invalid_root = sym LA_INVALID_ROOT,
                user_address_bits = const USER_ADDRESS_BITS,
                physical_last_bit = const PHYSICAL_BITS - 1,
                lddir_root = const LDDIR_ROOT,
                lddir_middle = const LDDIR_MIDDLE,
                pair_index_last = const PAGE_SHIFT + PT_INDEX_BITS - 1,
                pair_index_first = const PAGE_SHIFT + 1,
                pair_stride_shift = const core::mem::size_of::<[usize; 2]>().trailing_zeros(),
                handler = sym loongarch_refill,
            csr_crmd = const reg::CSR_CRMD,
            csr_eentry = const reg::CSR_EENTRY,
            csr_ks2 = const reg::CSR_KS2,
            csr_pgdl = const reg::CSR_PGDL,
            csr_tlbrbadv = const reg::CSR_TLBRBADV,
            csr_tlbrelo0 = const reg::CSR_TLBRELO0,
            csr_tlbrelo1 = const reg::CSR_TLBRELO1,
            csr_tlbrera = const reg::CSR_TLBRERA,
            csr_tlbrprmd = const reg::CSR_TLBRPRMD,
            csr_tlbrsave = const reg::CSR_TLBRSAVE,
            reg_crmd_address_mode = const reg::CRMD_ADDRESS_MODE,
            reg_crmd_pg = const reg::CRMD_PG,
            reg_plv_user = const reg::PLV_USER,
            reg_prmd_pplv = const reg::PRMD_PPLV,
            );
    }
}
