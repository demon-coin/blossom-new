//! RISC-V 64 位的内核任务上下文切换（Context Switch）实现。
//!
//! # 模块职责与架构定位
//! 本模块位于架构抽象层底部，向调度器（`crate::sched`）提供底层整数调用现场切换原语
//! `rv_switch_context`，并在内核自测（`ktest`）中提供跨真实调度让出（`yield_now`）的
//! 寄存器保活验证入口 `rv_task_context_roundtrip`。
//!
//! # 核心设计与寄存器布局
//! 1. **仅保存 Callee-Saved 整数现场**：
//!    任务主动或被动进入调度切换时，走的都是普通 C ABI 函数调用边界。根据 RISC-V ELF psABI，
//!    调用者保存（Caller-Saved）寄存器已由编译器在调用栈上妥善处理，因此 `TaskContext`
//!    只需保存返回地址 `ra`、栈指针 `sp` 以及 `s0`–`s11` 共 12 个被调用者保存寄存器。
//! 2. **浮点状态不跨内核任务切换保存**：
//!    普通内核代码始终在 `sstatus.FS = Off` 下运行（用户态浮点现场在进出 U-mode 的边界单独保存与恢复），
//!    因此内核任务切换无需读写浮点寄存器。
//! 3. **`tp`（`x4`）寄存器始终绑定当前物理核**：
//!    `tp` 固定指向当前运行 CPU 的 `CpuLocal` 结构。任务在切换后可能被迁移到其他核恢复执行，
//!    故切换过程绝不能保存或覆盖 `tp`，以确保恢复后后续代码始终访问当前所属 CPU 的本地状态。
//!
//! ```text
//! TaskContext 内存布局 (8 字节对齐，共 112 字节)
//! +--------+------------------------+------------------------------+
//! | 偏移   | 字段                   | 对应硬件寄存器               |
//! +--------+------------------------+------------------------------+
//! | 0x00   | ra                     | x1 (Return Address)          |
//! | 0x08   | sp                     | x2 (Stack Pointer)           |
//! | 0x10   | saved[0..12]           | x8..x9 (s0..s1), x18..x27    |
//! |        |                        | (s2..s11)                    |
//! +--------+------------------------+------------------------------+
//! (注：tp/x4 与 gp/x3 属于每核或全局固定寄存器，不在 TaskContext 中保存)
//! ```
//!
//! # 锁与并发契约
//! - 调用切换函数前必须已关闭本地中断（`sstatus.SIE = 0`），防止切换中途被中断打断导致栈指针与上下文半更新。
//! - 严禁持有普通自旋锁、`IrqSaveGuard` 或抢占守卫跨越上下文切换边界，否则会导致死锁或跨核锁状态错乱。

use crate::sched::context::TaskContext;
#[cfg(feature = "ktest")]
use core::mem::size_of;
use core::{arch::naked_asm, mem::offset_of};

/// 保存当前任务的整数调用现场到 `prev`，并恢复 `next` 任务的现场继续执行。
///
/// 该函数在调度器关闭本地中断的临界区内调用。当一个任务再次被调度器选中切回时，
/// 控制流会从它当初调用 `rv_switch_context` 时保存的 `ra` 处（即本函数末尾的 `ret`）返回。
///
/// # Safety
/// 1. 调用方必须已关闭本地中断（`sstatus.SIE == 0`），且当前 `sstatus.FS` 必须为 `Off`。
/// 2. `prev` 与 `next` 必须非空、按 `TaskContext` 对齐且互不重叠：`prev` 指向当前任务独占的可写上下文，
///    `next` 指向目标任务已完成初始化的只读上下文。
/// 3. `next` 中保存的 `sp` 必须满足 RISC-V 16 字节 ABI 栈对齐要求，`ra` 必须为合法的可执行内核返回地址；
///    新旧两个任务的内核栈与 `TaskContext` 结构体必须在各自任务恢复或彻底退出前保持有效。
/// 4. 调用方不得持有任何普通自旋锁、`IrqSaveGuard` 或抢占计数守卫跨越此调用。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn rv_switch_context(
    prev: *mut TaskContext,
    next: *const TaskContext,
) {
    // SAFETY:
    // 1. 上游 `switch_context` 已通过 `assert!` 校验本地中断已关闭且 `sstatus.FS == Off`。
    // 2. 调用方契约保证了 `a0`（`prev`）可写、`a1`（`next`）可读且各自指向有效的 `TaskContext`。
    // 3. 汇编严格按照 `offset_of!` 计算的字段偏移读写寄存器，不触碰 `tp`（保持当前 CPU 的 `CpuLocal` 指针）和 `gp`。
    unsafe {
        naked_asm!(
        // Step 1: 将当前任务的返回地址、栈指针与 s0-s11 保存到 prev (a0) 指向的 TaskContext。
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
        // Step 2: 从 next (a1) 恢复目标任务的寄存器现场，并通过 ret 跳转到目标任务的 ra。
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

/// 在普通内核任务中填充固定特征值并执行一次真实的 `yield_now`，验证调度切换前后的 `sp` 与 `s0`–`s11` 完整性。
///
/// 返回 `1` 表示所有被调用者保存寄存器和栈指针均完好无损，返回 `0` 表示检测到寄存器被意外篡改。
///
/// # Safety
/// 1. 必须在可合法让出 CPU 的普通可调度任务（Task）上下文中调用，且当前 `sstatus.FS` 为 `Off`。
/// 2. 当前内核栈必须满足 16 字节 ABI 对齐要求，并在整个让出和恢复期间保持有效。
/// 3. 调用方不得持有自旋锁、关中断守卫（`IrqSaveGuard`）或关抢占守卫跨越此调用；
///    任务恢复时可能已迁移到其他 CPU，`tp` 寄存器必须始终保持指向当前运行 CPU 的 `CpuLocal`。
#[cfg(feature = "ktest")]
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn rv_task_context_roundtrip() -> usize {
    // SAFETY:
    // 1. 调用方契约保证当前处于合法的可调度普通任务上下文，栈满足 16 字节对齐且在 `yield_now` 期间持续有效。
    // 2. 函数在栈上分配与 `TaskContext` 同尺寸（112 字节，16 字节整数倍）的保存帧，先备份调用者的 `ra` 与 `s0`–`s11`，
    //    再向 `s0`–`s11` 写入测试特征值并调用 `yield_now`，返回后无论校验是否通过都会恢复调用者现场并平衡栈指针。
    unsafe {
        naked_asm!(
        // Step 1: 在当前内核栈上开辟保存帧备份调用者现场，随后写入特征值并触发真实的调度让出。
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
        // Step 2: 从 yield_now 返回后，逐一核对栈指针 sp 以及 s0-s11 中的特征值是否与让出前一致。
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
        // Step 3: 从栈帧恢复调用者原本的 ra 与 s0-s11，释放栈帧并返回校验结果 (a0)。
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
