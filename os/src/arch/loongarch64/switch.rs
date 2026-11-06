//! LoongArch64 内核任务上下文切换（`TaskContext`）与保存恢复自检探针。
//!
//! ## 模块职责与架构定位
//! 本模块实现调度器在两个内核任务执行流之间切换的底层裸汇编入口 `la_switch_context`，
//! 以及在 `ktest` 模式下验证上下文切换完备性的测试探针 `la_test_task_context_preservation`。
//! 上游由 `crate::arch::loongarch64::switch_context`（进而由调度器 `sched`）在关中断状态下调用；
//! 下游直接读写 `TaskContext` 结构体的字段并完成栈与返回地址切换。
//!
//! ## 核心机制与寄存器布局
//! LoongArch64 LP64D C ABI 规定跨函数调用需由被调用者保存（Callee-saved）的整数寄存器如下：
//! ```text
//! TaskContext 内存布局 (共 12 个 usize = 96 字节):
//! ┌─────────────────────────┬────────────────────────────────────────────────┐
//! │ 字段偏移                │ 对应硬件寄存器与职责                           │
//! ├─────────────────────────┼────────────────────────────────────────────────┤
//! │ context_ra    (0x00)    │ $ra ($r1)  : 函数返回地址 / 下次恢复执行的起点 │
//! │ context_sp    (0x08)    │ $sp ($r3)  : 任务独占内核栈指针 (16 字节对齐)  │
//! │ saved[0..8]   (0x10..50)│ $s0–$s8 ($r23–$r31): 被调用者保存通用寄存器    │
//! │ saved[9]      (0x58)    │ $fp ($r22) : 帧指针寄存器                      │
//! └─────────────────────────┴────────────────────────────────────────────────┘
//! ```
//! - **Per-CPU 指针不随任务迁移**：`CSR_KS0` 始终指向当前物理 CPU 的 `CpuLocal` 结构，不写入 `TaskContext`。
//!   即使任务在唤醒后被调度到另一颗物理 CPU 上恢复执行，读取 `CSR_KS0` 得到的依然是新 CPU 的本地状态。
//! - **浮点与向量单元约束**：内核态任务切换要求扩展单元使能寄存器 `CSR_EUEN == 0`（用户态浮点状态在进出用户态边界时已独立保存）。
//!
//! ## 并发与锁契约
//! - 调用 `la_switch_context` 时必须处于本核硬件中断关闭状态（`CRMD.IE == 0`）。
//! - 调用方不得持有任何普通自旋锁、`IrqSaveGuard` 或 `PreemptGuard`，也不得在栈上跨 `la_switch_context` 保留对当前 CPU `CpuLocal` 的引用。

use crate::sched::context::TaskContext;
use core::arch::naked_asm;
use core::mem::{offset_of, size_of};

/// 保存当前任务（`prev`）的被调用者保存整数寄存器、栈指针与返回地址，并恢复目标任务（`next`）的整数执行现场。
///
/// 本函数仅切换整数 ABI 上下文，不读写浮点寄存器、`$tp`（`$r2`）或代表当前物理 CPU 的 `CSR_KS0`。
///
/// # Safety
/// - 调用时必须处于本核中断关闭（`CRMD.IE == 0`）且扩展单元关闭（`CSR_EUEN == 0`）的状态。
/// - 调用方必须拥有对 `prev` 的独占写权限与对 `next` 的独占读权限；两个指针均须非空、满足 `TaskContext` 对齐且内存区间互不重叠。
/// - `next` 必须已初始化为合法的任务恢复现场，且 `prev` 与 `next` 关联的内核栈均满足 16 字节 ABI 对齐并在各自再次被调度恢复前保持有效。
/// - 调用点不得持有任何自旋锁、中断/抢占守卫（Guard），也不得跨此次切换保存指向旧 CPU `CpuLocal` 的借用。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_switch_context(
    prev: *mut TaskContext,
    next: *const TaskContext,
) {
    // SAFETY: 1. 上层 `switch_context` 已通过断言确认 `CRMD.IE == 0` 且 `CSR_EUEN == 0`，调用方契约保证 `prev`（$a0）与 `next`（$a1）指针有效、对齐且互不重叠；
    // 2. 汇编严格按照 `offset_of!(TaskContext, ...)` 计算的偏移顺序保存和恢复 `$ra`、`$sp`、`$s0`–`$s8` 及 `$fp`，不修改 `CSR_KS0`。
    unsafe {
        naked_asm!(
        // Step 1: 将当前任务的返回地址 ($ra)、栈指针 ($sp) 与 10 个被调用者保存整数寄存器 ($s0..$s8, $fp) 写入 `prev` ($a0)。
        r#"
    # 普通调用现场只含整数 callee-saved；KS0 留在目的 CPU，普通 tp 不承担 Task TLS。
    st.d $ra, $a0, {context_ra}
    st.d $sp, $a0, {context_sp}
    st.d $s0, $a0, {context_saved}+0*{word_size}
    st.d $s1, $a0, {context_saved}+1*{word_size}
    st.d $s2, $a0, {context_saved}+2*{word_size}
    st.d $s3, $a0, {context_saved}+3*{word_size}
    st.d $s4, $a0, {context_saved}+4*{word_size}
    st.d $s5, $a0, {context_saved}+5*{word_size}
    st.d $s6, $a0, {context_saved}+6*{word_size}
    st.d $s7, $a0, {context_saved}+7*{word_size}
    st.d $s8, $a0, {context_saved}+8*{word_size}
    st.d $fp, $a0, {context_saved}+9*{word_size}
"#,
        // Step 2: 从 `next` ($a1) 恢复目标任务的栈指针、被调用者保存寄存器与返回地址，并跳转至 `$ra` 切换到目标任务执行流。
        r#"
    ld.d $ra, $a1, {context_ra}
    ld.d $sp, $a1, {context_sp}
    ld.d $s0, $a1, {context_saved}+0*{word_size}
    ld.d $s1, $a1, {context_saved}+1*{word_size}
    ld.d $s2, $a1, {context_saved}+2*{word_size}
    ld.d $s3, $a1, {context_saved}+3*{word_size}
    ld.d $s4, $a1, {context_saved}+4*{word_size}
    ld.d $s5, $a1, {context_saved}+5*{word_size}
    ld.d $s6, $a1, {context_saved}+6*{word_size}
    ld.d $s7, $a1, {context_saved}+7*{word_size}
    ld.d $s8, $a1, {context_saved}+8*{word_size}
    ld.d $fp, $a1, {context_saved}+9*{word_size}
    jirl $zero, $ra, 0
"#,
            context_ra = const offset_of!(TaskContext, ra),
            context_sp = const offset_of!(TaskContext, sp),
            context_saved = const offset_of!(TaskContext, saved),
            word_size = const size_of::<usize>(),
        );
    }
}

/// `ktest` 专用探针：在调用真实调度让出函数（`yield_fn`）前后填充并校验全部被调用者保存整数寄存器（`$s0`–`$s8`、`$fp`）与栈指针（`$sp`）。
///
/// - **测试目标**：验证内核任务在经历一次真实的 `yield_now` 上下文切换（及潜在的跨核迁移）后，栈指针与所有 Callee-saved 寄存器均被无损恢复。
/// - **场景步骤（Given-When-Then）**：
///   1. Given：在当前任务栈上分配 96 字节对齐栈帧保存调用者寄存器，并向 `$s0`–`$s8`、`$fp` 写入固定特征魔数，同时把当前 `$sp` 记录在栈帧偏移 88 处；
///   2. When：通过 `jirl $ra, $a0, 0` 调用传入的 `yield_fn`（即 `crate::sched::yield_now`）触发调度切换；
///   3. Then：当任务再次被调度恢复返回后，逐一比对 `$sp` 及 `$s0`–`$s8`、`$fp` 是否与切换前写入的特征值完全一致，全部匹配返回 `1`，任一失配返回 `0`，最后恢复调用者原始寄存器并返回。
///
/// # Safety
/// - 必须在合法的可调度普通内核任务上下文中调用，当前内核栈须满足 16 字节 ABI 对齐，且未启用浮点单元、未持有禁止调度的锁或守卫。
/// - `yield_fn` 必须是一个无参数、遵循 C ABI 正常返回且不会跨本裸汇编栈帧发生栈展开（unwind）的有效函数入口地址。
#[cfg(feature = "ktest")]
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_test_task_context_preservation(yield_fn: usize) -> usize {
    // SAFETY: 1. 调用方保证当前处于拥有合法内核栈的普通任务上下文，且 `yield_fn`（$a0）为有效函数地址；
    // 2. 栈帧大小 96 字节保持 16 字节栈对齐，所有被测试覆盖的 Callee-saved 寄存器均在入口压栈保存并在出口完整恢复。
    unsafe {
        naked_asm!(
            // Step 1: 在栈上开辟 96 字节保存调用者的原始 Callee-saved 寄存器，装填固定测试位图后调用 `yield_fn` ($a0)。
            r#"
    # 保存调用者现场；96 字节保持普通函数 ABI 的 16 字节栈对齐。
    addi.d $sp, $sp, -96
    st.d $ra, $sp, 0
    st.d $s0, $sp, 8
    st.d $s1, $sp, 16
    st.d $s2, $sp, 24
    st.d $s3, $sp, 32
    st.d $s4, $sp, 40
    st.d $s5, $sp, 48
    st.d $s6, $sp, 56
    st.d $s7, $sp, 64
    st.d $s8, $sp, 72
    st.d $fp, $sp, 80
    st.d $sp, $sp, 88
    li.d $s0, 0x13579bdf2468ace0
    li.d $s1, 0x2468ace013579bdf
    li.d $s2, 0x3141592653589793
    li.d $s3, 0x2718281828459045
    li.d $s4, 0x0123456789abcdef
    li.d $s5, 0xfedcba9876543210
    li.d $s6, 0x55aa55aa33cc33cc
    li.d $s7, 0xaa55aa55cc33cc33
    li.d $s8, 0x0f0ff0f096966969
    li.d $fp, 0xf0f00f0f69699696
    # a0 是 Rust 包装传入的真实 yield 入口，返回路径仍位于同一 Task 栈。
    jirl $ra, $a0, 0
"#,
            // Step 2: 从 `yield_fn` 返回后，逐一核对 `$sp` 与 `$s0`–`$s8`、`$fp` 是否保持测试位图不变；全部一致置返回值为 1，否则置为 0。
            r#"
2:
    ld.d $t0, $sp, 88
    bne $sp, $t0, 3f
    li.d $t0, 0x13579bdf2468ace0
    bne $s0, $t0, 3f
    li.d $t0, 0x2468ace013579bdf
    bne $s1, $t0, 3f
    li.d $t0, 0x3141592653589793
    bne $s2, $t0, 3f
    li.d $t0, 0x2718281828459045
    bne $s3, $t0, 3f
    li.d $t0, 0x0123456789abcdef
    bne $s4, $t0, 3f
    li.d $t0, 0xfedcba9876543210
    bne $s5, $t0, 3f
    li.d $t0, 0x55aa55aa33cc33cc
    bne $s6, $t0, 3f
    li.d $t0, 0xaa55aa55cc33cc33
    bne $s7, $t0, 3f
    li.d $t0, 0x0f0ff0f096966969
    bne $s8, $t0, 3f
    li.d $t0, 0xf0f00f0f69699696
    bne $fp, $t0, 3f
    li.w $a0, 1
    b 4f
3:
    move $a0, $zero
"#,
            // Step 3: 从栈帧恢复调用者的原始寄存器与栈指针，按原始 `$ra` 返回测试结果。
            r#"
4:
    ld.d $ra, $sp, 0
    ld.d $s0, $sp, 8
    ld.d $s1, $sp, 16
    ld.d $s2, $sp, 24
    ld.d $s3, $sp, 32
    ld.d $s4, $sp, 40
    ld.d $s5, $sp, 48
    ld.d $s6, $sp, 56
    ld.d $s7, $sp, 64
    ld.d $s8, $sp, 72
    ld.d $fp, $sp, 80
    addi.d $sp, $sp, 96
    jirl $zero, $ra, 0
"#,
        );
    }
}
