//! 普通函数 ABI 的整数切换；KS0 始终属于当前执行 CPU。

use crate::sched::context::TaskContext;
use core::arch::naked_asm;
use core::mem::{offset_of, size_of};

/// 保存旧执行者的整数续接并恢复下一执行者，不保存 FP、tp 或 KS0。
///
/// # Safety
/// 调用者必须在 IRQ 关闭且 EUEN 为零时独占两个非空、对齐、互不重叠的现场；
/// prev 可写，next 已初始化为合法返回现场，两个独占栈满足 16 字节 ABI 对齐并保活到恢复。
/// 不得持普通锁、IRQ/preempt guard 或跨切换保存本核借用。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_switch_context(
    prev: *mut TaskContext,
    next: *const TaskContext,
) {
    // SAFETY: 调用者保证两份现场独占、栈和返回地址有效且 IRQ/EUEN 关闭；
    // 汇编按同一 TaskContext 布局搬运整数 ABI 现场，始终保留目的 CPU 的 KS0。
    unsafe {
        naked_asm!(
        // 先将 ra、sp 与十个 callee-saved 整数寄存器写入 prev。
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
        // 再从 next 恢复同一布局；返回地址和栈共同决定目的续接。
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

/// 在真实 yield 前后比较 callee-saved、SP 与返回路径，并恢复调用者整数现场。
///
/// # Safety
/// 必须由可调度的当前普通 Task 调用，栈须满足 ABI 且无活跃 FP、锁或禁止调度的 guard。
/// yield_fn 必须是遵守当前 Task 调度协议、正常返回且不跨此裸帧展开的有效无参数入口；
/// 其调用过程必须保活本 Task 栈，允许迁核但不得把旧 CPU 的 KS0 带回目的 CPU。
#[cfg(feature = "ktest")]
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn la_test_task_context_preservation(yield_fn: usize) -> usize {
    // SAFETY: 调用者保证有效的普通 Task 栈和不跨此帧展开的 yield 入口；
    // 手动帧保存全部被改写的 callee-saved，恢复后仍使用目的 CPU 的 KS0。
    unsafe {
        naked_asm!(
            // 保存调用者的 96 字节 ABI 栈帧并布置图样，再调用真实 yield 入口。
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
            // 恢复执行后逐项检查原 SP 与 callee-saved 图样，结果只取零或一。
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
            // 恢复调用者寄存器及栈后按原 ra 返回，整个探针不读写 KS0。
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
