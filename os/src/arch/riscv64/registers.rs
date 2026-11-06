//! RISC-V 64 时钟中断上下文寄存器保存与恢复探针（仅 `ktest` 启用）。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::riscv64` 架构测试支撑层，仅在启用 `ktest` 特性时编译。向上供 `arch::riscv64`
//! 中的 `ktest_register_pattern` 与 `test_integer_register_preservation` 调用，用于验证内核在响应
//! 硬件时钟中断（Supervisor Timer Interrupt）并从 `rv_trap_entry` 返回后，被中断上下文的调用者保存寄存器
//! （Caller-saved）、被调用者保存寄存器（Callee-saved）、栈指针（`sp`）、线程指针（`tp`）以及可选的
//! 双精度浮点寄存器（`f0`、`f31`、`fcsr`）是否完好无损。
//!
//! # 核心设计与栈帧布局
//! `rv_register_roundtrip` 是一个 `#[naked]` 裸汇编函数，在当前内核栈上分配 112 字节（16 字节对齐）
//! 的临时保存帧：
//!
//! ```text
//! rv_register_roundtrip 栈帧布局 (共 112 字节，sp 向下生长):
//! +-----------------------+ <-- 原 sp (sp + 112)
//! | 72..112: 对齐填充区   | (40 字节，保持 16 字节 ABI 栈对齐)
//! +-----------------------+
//! | 64..72 : 原 fcsr      | (仅当 fp != 0 时保存并在退出前恢复)
//! +-----------------------+
//! | 56..64 : 原 sstatus   | (保存进入时的 SIE 与 FS 状态位)
//! +-----------------------+
//! | 48..56 : 原 s5        |
//! | 40..48 : 原 s4        |
//! | 32..40 : 原 s3        |
//! | 24..32 : 原 s2        |
//! | 16..24 : 原 s1        |
//! |  8..16 : 原 s0        |
//! |  0..8  : 原 ra        |
//! +-----------------------+ <-- 探针运行期间的 sp (记录于 s0)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本函数内部会临时置位 `sstatus.SIE` 打开本地中断以等待硬件时钟中断到来，因此**调用方在进入本函数前严禁持有任何自旋锁**，
//!   否则时钟中断处理路径一旦尝试获取同一把锁将导致本核死锁。
//! - 调用期间当前执行流必须固定在同一物理 CPU 上，禁止触发任务抢占或跨核迁移。

use crate::arch::riscv64::{SR_FS, SR_FS_DIRTY, SR_SIE};
use core::{arch::naked_asm, sync::atomic::AtomicUsize};

/// 寄存器往返探针在内核栈上分配的保存帧大小（112 字节，满足 RISC-V 16 字节栈对齐要求）。
const REGISTER_PROBE_FRAME_BYTES: usize = 112;

/// 在受控测试环境中写入特征寄存器图样、临时开启本地中断等待一次硬件时钟中断完成，并校验中断返回后所有测试寄存器是否保持原值。
///
/// - **测试目标**：验证 `rv_trap_entry` 中断入口与返回路径是否正确保存并恢复通用整数寄存器（含 `sp`、`tp`、`t0`、`t1`、`a3`、`s2`）
///   及按需启用的双精度浮点寄存器（`f0`、`f31`、`fcsr`）。
/// - **场景步骤（Given-When-Then）**：
///   1. **Given**：调用方已关闭本地中断、设定下一次硬件时钟中断截止时间，并传入本核时钟中断计数器地址 `counter`、目标计数值 `expected` 及浮点测试开关 `fp`（0 表示仅测整数，非 0 表示同时测浮点）。
///   2. **When**：函数保存调用者 ABI 现场，向寄存器写入固定魔数图样，随后置位 `sstatus.SIE` 开启中断并自旋读取 `*counter` 直至其达到或超过 `expected`，最后再次关闭中断。
///   3. **Then**：逐一比对各寄存器与魔数图样是否完全一致（一致返回 `1`，任一寄存器被破坏则返回 `0`），并恢复进入时的 `fcsr`、`sstatus` 及被调用者保存寄存器。
///
/// # Safety
/// 1. `counter` 必须指向当前 CPU 专属、8 字节对齐且在内核生命周期内永久有效的 `AtomicUsize`（如 `CpuLocal::timer_interrupts`），
///    且硬件定时器必须已编程，确保开启中断后 `*counter` 能够推进到 `expected`。
/// 2. 当前 CPU 的异常入口（`stvec`）、时钟中断源、`CpuLocal`（`tp`）及当前内核栈必须已初始化且可正常响应中断；调用期间必须固定在当前 CPU 上，禁止发生任务调度或跨核迁移。
/// 3. 目标硬件必须支持 RISC-V `D`（双精度浮点）扩展；调用时当前上下文不得持有未保存的活跃浮点状态，因为当 `fp != 0` 时本函数会直接覆盖 `f0` 与 `f31`。
/// 4. 调用时严禁持有任何中断处理路径可能获取的自旋锁；调用方必须在函数返回后（仍处于关中断状态）重新设置正常的系统时钟中断截止时间。
#[naked]
#[no_mangle]
pub(super) unsafe extern "C" fn rv_register_roundtrip(
    counter: *const AtomicUsize,
    expected: usize,
    fp: usize,
) -> usize {
    // SAFETY:
    // 1. 调用方契约保证 `counter` 指向本核有效的 `AtomicUsize` 且当前栈具备容纳 112 字节栈帧的空间；
    // 2. 栈帧显式保存并恢复 `ra`、`s0..s5`、`sstatus` 及（`fp != 0` 时的）`fcsr`，严格遵守 RISC-V C ABI 调用约定；
    // 3. 仅当 `fp != 0` 时才先将 `sstatus.FS` 置为 `Dirty` 再执行 `fcsr`/`f0`/`f31` 浮点指令，避免在 `FS=Off` 时触发非法指令异常。
    unsafe {
        naked_asm!(
            ".option push",
            ".option arch, +d",
            // Step 1: 在栈上分配 112 字节帧，保存 ra、s0..s5 与原 sstatus，并将 sp、tp 及入参备份到 s0..s5。
            r#"
    addi sp, sp, -{frame_size}
    sd ra, 0(sp)
    sd s0, 8(sp)
    sd s1, 16(sp)
    sd s2, 24(sp)
    sd s3, 32(sp)
    sd s4, 40(sp)
    sd s5, 48(sp)
    csrr t0, sstatus
    sd t0, 56(sp)
    mv s0, sp
    mv s1, tp
    mv s3, a0
    mv s4, a1
    mv s5, a2
"#,
            // Step 2: 若 fp != 0 则开启 sstatus.FS 并装填 fcsr/f0/f31 浮点图样，否则关闭 FS；随后装填 t0/t1/a3/s2 整数图样。
            r#"
    beqz s5, 1f
    li t0, {fs_dirty}
    csrs sstatus, t0
    frcsr t0
    sd t0, 64(sp)
    li t0, 0x61
    fscsr t0
    li t0, 0x3ff123456789abcd
    fmv.d.x f0, t0
    li t0, 0x400fedcba9876543
    fmv.d.x f31, t0
    j 2f
1:  li t0, {fs}
    csrc sstatus, t0
2:  li t0, 0x123456789abcdef
    li t1, 0x76543210fedcba98
    li a3, 0x13579bdf2468ace0
    li s2, 0x1122334455667788
"#,
            // Step 3: 置位 sstatus.SIE 开启中断，自旋等待本核时钟中断计数达到 expected，然后关中断并逐一校验寄存器图样。
            r#"
    csrsi sstatus, {sie}
3:  ld a0, 0(s3)
    fence r, r
    bltu a0, s4, 3b
    csrci sstatus, {sie}
    li a0, 0
    bne sp, s0, 9f
    bne tp, s1, 9f
    li a1, 0x123456789abcdef
    bne t0, a1, 9f
    li a1, 0x76543210fedcba98
    bne t1, a1, 9f
    li a1, 0x13579bdf2468ace0
    bne a3, a1, 9f
    li a1, 0x1122334455667788
    bne s2, a1, 9f
    beqz s5, 8f
    fmv.x.d a1, f0
    li a2, 0x3ff123456789abcd
    bne a1, a2, 9f
    fmv.x.d a1, f31
    li a2, 0x400fedcba9876543
    bne a1, a2, 9f
    frcsr a1
    li a2, 0x61
    bne a1, a2, 9f
8:  li a0, 1
"#,
            // Step 4: 按需恢复原 fcsr，恢复原 sstatus 与 ra、s0..s5 整数寄存器，释放栈帧并返回校验结果（a0）。
            r#"
9:  beqz s5, 10f
    ld t0, 64(sp)
    fscsr t0
10: ld t0, 56(sp)
    csrw sstatus, t0
    ld ra, 0(sp)
    ld s0, 8(sp)
    ld s1, 16(sp)
    ld s2, 24(sp)
    ld s3, 32(sp)
    ld s4, 40(sp)
    ld s5, 48(sp)
    addi sp, sp, {frame_size}
    ret
"#,
            ".option pop",
            fs = const SR_FS,
            fs_dirty = const SR_FS_DIRTY,
            sie = const SR_SIE,
            frame_size = const REGISTER_PROBE_FRAME_BYTES,
        );
    }
}
