//! LoongArch64 时钟中断上下文保存与恢复的硬件寄存器自检探针。
//!
//! # 模块职责与架构定位
//! 本模块服务于 LoongArch64 架构层的中断上下文自检测试，提供裸函数 [`la_test_register_preservation`]。
//! 它在内核态（PLV0）预置已知的通用寄存器、栈指针、`CSR_KS0` 以及可选的标量浮点寄存器（`$f0`、`$f24`、`FCSR0`、`FCC0`..`FCC7`）测试图样，
//! 随后临时编程并使能硬件定时器、开启全局中断（`CRMD.IE`），自旋等待一次真实的时钟中断触发并由内核异常入口完成上下文保存与恢复。
//! 中断返回后，探针逐一比对各寄存器是否与中断前布置的图样完全一致，并在返回前无损还原调用者的原始寄存器与控制状态。
//!
//! # 手工栈帧布局（192 字节，16 字节对齐）
//! ```text
//! 高地址 (进入函数前的原始 $sp)
//! +---------------------------------------------------------------+ +192
//! | (未用对齐填充)                                                | +184
//! | $fcc7 原始值 (movcf2gr 扩展为 64 位)                          | +176
//! | $fcc6 原始值                                                  | +168
//! | $fcc5 原始值                                                  | +160
//! | $fcc4 原始值                                                  | +152
//! | $fcc3 原始值                                                  | +144
//! | $fcc2 原始值                                                  | +136
//! | $fcc1 原始值                                                  | +128
//! | $fcc0 原始值                                                  | +120
//! | $fcsr0 原始值 (movfcsr2gr)                                    | +112
//! | $f24 原始双精度浮点值 (callee-saved FP)                       | +104
//! | $f0  原始双精度浮点值 (caller-saved FP)                       | +96
//! | 分配栈帧后的基准 $sp 快照 (用于校验中断返回后栈指针未偏移)    | +88
//! | CSR_KS0 原始值                                                | +80
//! | $a3 (fp 测试使能标志: 0=仅整数, 非0=含标量浮点)               | +72
//! | $a2 (fcc 8 位测试图样)                                        | +64
//! | $a1 (ticks 定时器周期配置值)                                  | +56
//! | *counter 初始计数值快照 (用于忙等检测时钟中断已发生)          | +48
//! | $a0 (counter 原子计数器地址)                                  | +40
//! | CSR_CRMD 原始值 (用于退出前恢复进入时的 IE 状态)              | +32
//! | CSR_EUEN 原始值 (用于退出前恢复扩展单元使能状态)              | +24
//! | $s1 原始值 (callee-saved)                                     | +16
//! | $s0 原始值 (callee-saved)                                     | +8
//! | $ra 返回地址                                                  | +0
//! +---------------------------------------------------------------+ <-- 当前 $sp
//! 低地址
//! ```
//!
//! # 锁与并发契约
//! - 本函数本身不获取任何锁，但要求调用方已绑定当前 CPU 核且禁止任务切换。
//! - 调用方在调用期间严禁持有时钟中断处理路径可能获取的任何自旋锁，否则在开启 `CRMD.IE` 等待中断时会发生同核死锁。

use crate::arch::loongarch64::register as reg;
use core::arch::naked_asm;
use core::sync::atomic::AtomicUsize;

/// 触发一次本核硬件时钟中断，并校验中断前后通用寄存器、`SP`、`CSR_KS0` 及可选标量浮点寄存器（`FP`/`FCSR0`/`FCC`）是否被完整保存与恢复。
///
/// - `counter`：指向本核时钟中断处理函数递增的 `AtomicUsize` 计数器指针。
/// - `ticks`：写入 `CSR_TCFG` 的定时器触发周期（必须满足硬件要求的低位对齐）。
/// - `fcc`：当 `fp != 0` 时写入 `$fcc0..$fcc7` 的 8 位测试图样（每位对应一个条件标志寄存器）。
/// - `fp`：是否同时测试标量浮点上下文（`0` 表示关闭 `EUEN` 仅测试整数与控制寄存器，非 `0` 表示开启 `EUEN_FP` 并测试标量浮点寄存器）。
/// - 返回值：所有被测寄存器在中断返回后均与预期图样一致时返回 `1`，任一寄存器被破坏时返回 `0`。
///
/// # Safety
/// 1. 调用时必须处于内核态（`PLV0`），且本核的异常入口、`CpuLocal`、时钟中断源以及当前线程的 C ABI 调用栈均已正确初始化。
/// 2. `counter` 必须指向生命周期覆盖整个调用过程、按 `usize` 自然对齐的有效 `AtomicUsize` 实例；`ticks` 必须为合法的定时器周期值。
/// 3. 调用期间必须固定在当前 CPU 核且禁止任务抢占或切换，不得持有时钟中断处理程序需要获取的任何锁，也不得依赖当前尚未保存的活跃浮点上下文。
/// 4. `fp` 只能取值 `0` 或 `1`（非零时当前硬件必须支持基础标量浮点扩展），`fcc` 必须为 `0..=0xff` 范围内的 8 位无符号整数。
/// 5. 函数返回前会恢复进入时的 `CRMD.IE` 与 `CSR_EUEN`，但被改写的定时器配置（`CSR_TCFG`）必须由调用方在同核关中断状态下重新编程恢复（无论本函数返回 `1` 还是 `0`），且严禁跨越此裸汇编栈帧触发栈展开（unwind）。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_test_register_preservation(
    counter: *const AtomicUsize,
    ticks: usize,
    fcc: usize,
    fp: usize,
) -> usize {
    // SAFETY:
    // 1. 调用方契约保证了当前处于 PLV0、栈指针 `$sp` 有效且对齐、`counter` 可安全解引用。
    // 2. 栈帧在入口处分配 192 字节，完整保存了 `$ra`、`$s0`、`$s1`、`CSR_EUEN`、`CSR_CRMD`、`CSR_KS0` 以及（当 `fp != 0` 时）被修改的浮点寄存器，并在标号 `8`/`9` 处悉数恢复后平衡栈指针返回。
    // 3. 开启 `CRMD.IE` 仅发生在测试图样就绪之后，且在检测到 `*counter` 递增后立即关闭 `CRMD.IE`。
    unsafe {
        naked_asm!(
            // Step 1: 分配 192 字节栈帧，保存调用者保存/被调用者保存整数寄存器、原始 EUEN/CRMD/KS0 状态及计数器初始快照。
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
            // Step 2: 若启用浮点测试（$a3 != 0），临时开启 EUEN.FP，保存原始 $f0/$f24/FCSR0/FCC0..7，并写入双精度与条件码测试图样；否则关闭 EUEN。
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
            // Step 3: 在通用寄存器中写入测试图样，配置 CSR_TCFG 启动定时器，并临时开启 CRMD.IE 自旋等待中断计数器递增，触发后立即关闭 CRMD.IE。
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
            // Step 4: 逐项校验中断返回后的通用寄存器、$sp、CSR_KS0 以及浮点寄存器（或未启用浮点时验证 EUEN 仍为 0）；任一不匹配则跳转至标号 7 返回 0。
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
            // Step 5: 恢复原始浮点寄存器（若曾开启 FP）、CSR_EUEN、CSR_KS0、CRMD.IE 及调用者整数寄存器，释放 192 字节栈帧并返回测试结果（$a0）。
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
