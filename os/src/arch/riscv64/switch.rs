//! 普通整数调用现场；切换只搬运 ABI callee-saved，tp 始终属于当前 CPU。

use crate::sched::context::TaskContext;
#[cfg(feature = "ktest")]
use core::mem::size_of;
use core::{arch::naked_asm, mem::offset_of};

/// 保存当前整数调用现场，并在目的现场的有效栈和返回地址上继续执行。
///
/// # Safety
/// 调用者必须关闭 IRQ、保持 FS Off，独占有效、对齐且不重叠的 prev/next；prev 可写，
/// next 已初始化且含有效的返回地址、16 字节对齐栈与 callee-saved 现场。
/// 两份执行栈和上下文必须保活到各自恢复或退出，不能持普通锁或 IRQ/preempt guard 跨切换。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn rv_switch_context(
    prev: *mut TaskContext,
    next: *const TaskContext,
) {
    // SAFETY: 调用者保证两份现场独占、栈和返回地址有效且 IRQ/FS 关闭；
    // 汇编仅按 TaskContext 的实际偏移访问整数现场，不改写目的 CPU 的 tp。
    unsafe {
        naked_asm!(
        // 先保存当前调用的整数 ABI 现场，不保存旧 CPU 的 tp。
        r#"
    // 普通调用只拥有整数现场；tp 始终指向正在执行此代码的 CPU。
    sd ra, {context_ra}(a0)
    sd sp, {context_sp}(a0)
    sd s0, ({context_saved} + 0 * {word})(a0)
    sd s1, ({context_saved} + 1 * {word})(a0)
    sd s2, ({context_saved} + 2 * {word})(a0)
    sd s3, ({context_saved} + 3 * {word})(a0)
    sd s4, ({context_saved} + 4 * {word})(a0)
    sd s5, ({context_saved} + 5 * {word})(a0)
    sd s6, ({context_saved} + 6 * {word})(a0)
    sd s7, ({context_saved} + 7 * {word})(a0)
    sd s8, ({context_saved} + 8 * {word})(a0)
    sd s9, ({context_saved} + 9 * {word})(a0)
    sd s10, ({context_saved} + 10 * {word})(a0)
    sd s11, ({context_saved} + 11 * {word})(a0)
"#,
        // 恢复目的调用的现场并由其 ra 继续，目的 CPU 的 CpuLocal 基址不变。
        r#"
    ld ra, {context_ra}(a1)
    ld sp, {context_sp}(a1)
    ld s0, ({context_saved} + 0 * {word})(a1)
    ld s1, ({context_saved} + 1 * {word})(a1)
    ld s2, ({context_saved} + 2 * {word})(a1)
    ld s3, ({context_saved} + 3 * {word})(a1)
    ld s4, ({context_saved} + 4 * {word})(a1)
    ld s5, ({context_saved} + 5 * {word})(a1)
    ld s6, ({context_saved} + 6 * {word})(a1)
    ld s7, ({context_saved} + 7 * {word})(a1)
    ld s8, ({context_saved} + 8 * {word})(a1)
    ld s9, ({context_saved} + 9 * {word})(a1)
    ld s10, ({context_saved} + 10 * {word})(a1)
    ld s11, ({context_saved} + 11 * {word})(a1)
    ret
"#,
            context_ra = const offset_of!(TaskContext, ra),
            context_sp = const offset_of!(TaskContext, sp),
            context_saved = const offset_of!(TaskContext, saved),
            word = const core::mem::size_of::<usize>(),
        );
    }
}

/// 在普通 Task 中跨真实 yield 检查 SP 与全部 callee-saved，并恢复调用者现场。
///
/// # Safety
/// 必须由可合法 yield 的普通 Task 调用，FS 为 Off，当前栈满足 16 字节 ABI 对齐且保活。
/// 不得持有锁或 IRQ/preempt guard 跨 yield；恢复时可以迁核，tp 必须保持目的 CPU 的基址。
#[cfg(feature = "ktest")]
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn rv_task_context_roundtrip() -> usize {
    // SAFETY: 调用者保证当前普通 Task 可合法 yield，栈满足 ABI 且跨恢复保活；
    // 手动帧保存全部被改写的 callee-saved，调度返回后按原 SP 和 ra 继续。
    unsafe {
        naked_asm!(
        // 用同一 TaskContext 布局保留调用者，写入图样后调用真实调度入口。
        r#"
    // 复用上下文布局保留调用者；帧长为 16 字节的整数倍，不保存旧 CPU 的 tp。
    addi sp, sp, -{context_size}
    sd ra, {context_ra}(sp)
    sd sp, {context_sp}(sp)
    sd s0, ({context_saved} + 0 * {word})(sp)
    sd s1, ({context_saved} + 1 * {word})(sp)
    sd s2, ({context_saved} + 2 * {word})(sp)
    sd s3, ({context_saved} + 3 * {word})(sp)
    sd s4, ({context_saved} + 4 * {word})(sp)
    sd s5, ({context_saved} + 5 * {word})(sp)
    sd s6, ({context_saved} + 6 * {word})(sp)
    sd s7, ({context_saved} + 7 * {word})(sp)
    sd s8, ({context_saved} + 8 * {word})(sp)
    sd s9, ({context_saved} + 9 * {word})(sp)
    sd s10, ({context_saved} + 10 * {word})(sp)
    sd s11, ({context_saved} + 11 * {word})(sp)
    li s0, 0x1122334455667700
    li s1, 0x1122334455667701
    li s2, 0x1122334455667702
    li s3, 0x1122334455667703
    li s4, 0x1122334455667704
    li s5, 0x1122334455667705
    li s6, 0x1122334455667706
    li s7, 0x1122334455667707
    li s8, 0x1122334455667708
    li s9, 0x1122334455667709
    li s10, 0x112233445566770a
    li s11, 0x112233445566770b
    call {yield_now}
"#,
        // 返回到调用后标签才校验 SP 与十二个保存寄存器，ra 按 caller-saved 处理。
        r#"
2:
    // 到达调用后的标签验证返回路径；ra 是 caller-saved，不要求 Rust 返回时保留其数值。
    li a0, 0
    ld t0, {context_sp}(sp)
    bne sp, t0, 3f
    li t0, 0x1122334455667700
    bne s0, t0, 3f
    li t0, 0x1122334455667701
    bne s1, t0, 3f
    li t0, 0x1122334455667702
    bne s2, t0, 3f
    li t0, 0x1122334455667703
    bne s3, t0, 3f
    li t0, 0x1122334455667704
    bne s4, t0, 3f
    li t0, 0x1122334455667705
    bne s5, t0, 3f
    li t0, 0x1122334455667706
    bne s6, t0, 3f
    li t0, 0x1122334455667707
    bne s7, t0, 3f
    li t0, 0x1122334455667708
    bne s8, t0, 3f
    li t0, 0x1122334455667709
    bne s9, t0, 3f
    li t0, 0x112233445566770a
    bne s10, t0, 3f
    li t0, 0x112233445566770b
    bne s11, t0, 3f
    li a0, 1
"#,
        // 无论检查结果如何均恢复原调用者，返回值只表示图样是否完整。
        r#"
3:
    ld ra, {context_ra}(sp)
    ld s0, ({context_saved} + 0 * {word})(sp)
    ld s1, ({context_saved} + 1 * {word})(sp)
    ld s2, ({context_saved} + 2 * {word})(sp)
    ld s3, ({context_saved} + 3 * {word})(sp)
    ld s4, ({context_saved} + 4 * {word})(sp)
    ld s5, ({context_saved} + 5 * {word})(sp)
    ld s6, ({context_saved} + 6 * {word})(sp)
    ld s7, ({context_saved} + 7 * {word})(sp)
    ld s8, ({context_saved} + 8 * {word})(sp)
    ld s9, ({context_saved} + 9 * {word})(sp)
    ld s10, ({context_saved} + 10 * {word})(sp)
    ld s11, ({context_saved} + 11 * {word})(sp)
    addi sp, sp, {context_size}
    ret
"#,
            context_size = const size_of::<TaskContext>(),
            context_ra = const offset_of!(TaskContext, ra),
            context_sp = const offset_of!(TaskContext, sp),
            context_saved = const offset_of!(TaskContext, saved),
            word = const size_of::<usize>(),
            yield_now = sym crate::sched::yield_now,
        );
    }
}
