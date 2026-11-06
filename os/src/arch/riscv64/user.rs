//! RISC-V 64 用户态寄存器上下文（[`UserContext`]）、用户态进入/返回主入口（[`run_user`]）及信号跳板切片。
//!
//! # 模块职责与架构定位
//! 本模块作为 `arch::riscv64` 面向任务执行循环（`crate::task`）的核心桥梁，定义了每个用户线程私有的
//! 硬件上下文 [`UserContext`]，并封装了从内核态切换到 U-mode 运行、在发生中断或异常时捕获 Trap 快照
//! 并返回 Rust 调用点的完整闭环。底层汇编入口由子模块 [`entry`] 实现，信号返回的只读跳板机器码由
//! 子模块 [`signal_return`] 提供。
//!
//! # 核心设计与内存布局
//! [`UserContext`] 按 16 字节对齐，采用 `#[repr(C)]` 固定布局，供 Rust 代码与 [`entry`] 中的裸汇编
//! 通过 `offset_of!` 直接按字节偏移读写：
//!
//! ```text
//! Offset  Size   Field           Description
//! ------  -----  --------------  ---------------------------------------------------------
//! 0       256    gpr[0..32]      32 个 64 位通用寄存器（x0 恒为 0，x2 为用户栈指针 sp）
//! 256     8      pc              用户态程序计数器（进入时写入 sepc，陷入时从 sepc 保存）
//! 264     256    fpr[0..32]      32 个 64 位双精度浮点寄存器（f0..f31）
//! 520     8      fcsr            浮点控制与状态寄存器（仅低 32 位有效）
//! 528     8      kernel_sp       单次 rv_run_user 调用期间暂存的内核栈续接帧地址，返回后清零
//! 536     8      scause          最近一次从用户态陷入时的 scause 寄存器快照
//! 544     8      stval           最近一次从用户态陷入时的 stval 寄存器快照
//! 552     8      (padding)       结构体尾部填充至 16 字节对齐（总大小 560 字节）
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`UserContext`] 由所属用户线程独占访问，在线程运行期间通过 `&mut UserContext` 唯一借用。
//! - 调用 [`run_user`] 前**必须关闭本地硬中断（IRQ）且不得持有任何自旋锁或睡眠锁**；
//!   进入 U-mode 后硬件通过 `sret` 根据 `SPIE` 位自动开启用户态中断响应，陷入 S-mode 时硬件自动关闭中断。
//! - 在 `rv_run_user` 汇编区间内严禁触发线程调度；所有跨核迁移或阻塞睡眠只能在 [`run_user`] 返回到 Rust
//!   且外层释放中断守卫（`HardIrqGuard`）之后发生，从而保证内核栈上暂存的 `tp`（`CpuLocal`）和 `gp` 不会跨核失效。

use crate::arch::{
    riscv64::{
        irq_enabled, rv_dispatch_interrupt, CAUSE_IRQ_FLAG, EXC_INST_PAGE_FAULT,
        EXC_LOAD_PAGE_FAULT, EXC_STORE_PAGE_FAULT, EXC_SYSCALL, SR_FS,
    },
    UserAccess, UserTrap,
};
use core::arch::asm;

mod entry;
mod signal_return;

use crate::arch::riscv64::user::entry::rv_run_user;
pub(in crate::arch::riscv64) use crate::arch::riscv64::user::entry::rv_user_trap_entry;

/// RISC-V 64 用户线程的完整硬件上下文与单次陷入快照。
///
/// 该结构体由每个用户线程独占持有，在 `rv_run_user` 与 `rv_user_trap_entry` 汇编路径中通过
/// `sscratch` 寄存器传递其内核虚拟地址。
#[repr(C, align(16))]
pub struct UserContext {
    /// 32 个 64 位通用整数寄存器（`x0`..`x31`）；其中 `gpr[0]` 恒为 0，`gpr[2]` 为用户栈指针 `sp`。
    pub gpr: [usize; 32],
    /// 用户态指令指针（对应硬件 `sepc` 寄存器，单位：字节地址）。
    pub pc: usize,
    /// 32 个 64 位双精度浮点寄存器（`f0`..`f31`）。
    pub fpr: [u64; 32],
    /// 浮点控制与状态寄存器（`fcsr`）。
    pub fcsr: usize,
    /// 单次 `rv_run_user` 汇编调用期间指向当前内核栈续接帧（`KERNEL_CONTINUATION_SIZE`）的栈指针。
    ///
    /// 仅在处于用户态或陷入返回路径中有效，每次回到 Rust 前清零，不跨调度保存 `CpuLocal` 状态。
    kernel_sp: usize,
    /// 最近一次用户态陷入时硬件写入的异常/中断原因寄存器（`scause`）。
    scause: usize,
    /// 最近一次用户态陷入时硬件写入的附加陷阱值寄存器（`stval`，如缺页故障虚拟地址）。
    stval: usize,
}

impl UserContext {
    /// 创建初始清零的用户态上下文，并将程序计数器设为 `pc`、用户栈指针（`gpr[2]`，即 `x2`）设为 `sp`。
    pub fn new(pc: usize, sp: usize) -> Self {
        let mut context = Self {
            gpr: [0; 32],
            pc,
            fpr: [0; 32],
            fcsr: 0,
            kernel_sp: 0,
            scause: 0,
            stval: 0,
        };
        context.gpr[2] = sp;
        context
    }

    /// 为 `fork` / `clone` 派生子线程的独立用户态上下文：仅复制用户可见的整数/浮点寄存器与 `pc`，
    /// 将内核栈续接指针（`kernel_sp`）与上一次陷入的异常寄存器记录（`scause` / `stval`）清零。
    pub(crate) fn fork(&self) -> Self {
        Self {
            gpr: self.gpr,
            pc: self.pc,
            fpr: self.fpr,
            fcsr: self.fcsr,
            kernel_sp: 0,
            scause: 0,
            stval: 0,
        }
    }
}

extern "C" {
    static rv_user_signal_return_start: u8;
    static rv_user_signal_return_end: u8;
}

/// 返回内核只读段中预编译的 `rt_sigreturn` 信号返回跳板机器码切片，供地址空间初始化时映射为用户态只读可执行（RX）跳板页。
pub fn signal_return_code() -> &'static [u8] {
    let start = core::ptr::addr_of!(rv_user_signal_return_start);
    let end = core::ptr::addr_of!(rv_user_signal_return_end) as usize;
    // SAFETY:
    // 1. `rv_user_signal_return_start` 与 `rv_user_signal_return_end` 由 `signal_return.rs` 在同一 `.rodata.user_signal_return`
    //    只读节中相邻定义，`end >= start` 且其间字节在内核镜像加载时已完整初始化；
    // 2. 该内存位于内核高半永久只读映射中，整个内核生命周期内地址有效且内容不可变，满足 `'static` 切片要求。
    unsafe { core::slice::from_raw_parts(start, end - start as usize) }
}

/// 在当前普通任务上下文中切换至 U-mode 执行 `context` 描述的用户线程，直到下一次中断或异常陷入内核，
/// 完成底层硬中断分派后将陷入原因封装为 [`UserTrap`] 返回（不会自动推进 `context.pc`）。
///
/// # Safety
/// 1. 调用者必须是当前普通任务（Task）的唯一执行者，独占持有 `context`，且不得在中断上下文或持有自旋锁时调用；
/// 2. 调用前必须已关闭本地硬中断（`!irq_enabled()`），且内核态不得存在活跃的浮点上下文（`sstatus.FS == Off`）；
/// 3. 当前激活的用户页表根（`Mm`）及内核高半地址空间映射必须在整个 `run_user` 调用期间保持存活且有效；
/// 4. 从进入本函数到返回 Rust 并释放外层 `HardIrqGuard` 之前，严禁触发任务切换或跨核迁移。
pub unsafe fn run_user(context: &mut UserContext) -> UserTrap {
    // Step 1: 断言本地硬中断已关闭，并读取 sstatus 确认内核态浮点单元处于关闭状态（FS == Off），
    // 防止用户态浮点寄存器恢复时覆盖未保存的内核浮点状态。
    assert!(!irq_enabled());
    let status: usize;
    // SAFETY: 当前 CPU 运行在 S-mode，`csrr sstatus` 仅只读获取当前核状态字，不修改内存或栈。
    unsafe {
        asm!("csrr {status}, sstatus", status = out(reg) status, options(nomem, nostack));
    }
    assert_eq!(
        status & SR_FS,
        0,
        "kernel FP must be disabled before user entry"
    );

    // Step 2: 调用底层裸汇编入口保存内核续接帧并执行 sret 进入用户态，待用户态发生 trap 后原路返回此处。
    // SAFETY:
    // 1. 前置 `assert!` 已确认本地硬中断关闭且 `sstatus.FS == Off`；
    // 2. 调用方契约保证了 `context` 独占有效、当前任务栈与页表在返回前持续存活且期间不发生调度；
    // 3. `rv_run_user` 与 `rv_user_trap_entry` 配对恢复内核 `sp`/`tp`/`gp`/callee-saved 寄存器，并在返回前将 `sscratch` 清零及关闭 `FS`。
    unsafe {
        rv_run_user(context);
    }

    // Step 3: 根据陷入时捕获的 scause 最高位区分异步硬中断与同步异常，完成中断分派或构造 UserTrap 返回值。
    if context.scause & CAUSE_IRQ_FLAG != 0 {
        rv_dispatch_interrupt(context.scause, context.pc, context.gpr[2], context.stval);
        return UserTrap::Interrupt;
    }
    match context.scause {
        EXC_SYSCALL => UserTrap::Syscall,
        EXC_INST_PAGE_FAULT => UserTrap::PageFault {
            address: context.stval,
            access: UserAccess::Execute,
        },
        EXC_LOAD_PAGE_FAULT => UserTrap::PageFault {
            address: context.stval,
            access: UserAccess::Read,
        },
        EXC_STORE_PAGE_FAULT => UserTrap::PageFault {
            address: context.stval,
            access: UserAccess::Write,
        },
        code => UserTrap::Exception {
            code,
            address: context.stval,
        },
    }
}

#[cfg(feature = "ktest")]
extern "C" {
    static rv_user_probe_start: u8;
    static rv_user_probe_end: u8;
    static rv_user_probe_loop: u8;
    static rv_user_probe_spin: u8;
    static rv_user_probe_exit: u8;
    static rv_user_probe_read: u8;
    static rv_user_probe_write: u8;
    static rv_user_probe_execute: u8;
}

/// 获取内核测试（`ktest`）专用的位置无关用户态探测机器码切片，可整体拷贝至用户可执行页运行（所有访存地址均通过用户寄存器 `sp`/`a5`/`a6` 传入）。
#[cfg(feature = "ktest")]
pub fn user_code() -> &'static [u8] {
    // SAFETY:
    // 1. `rv_user_probe_start` 与 `rv_user_probe_end` 在 `entry.rs` 的同一 `.text.user_probe` 节中定义，
    //    界定了连续且已初始化的只读指令字节范围（`end >= start`）；
    // 2. 内核代码段映射在运行期间永久有效且只读，构造 `'static` 字节切片不会产生悬垂指针或数据竞争。
    unsafe {
        let start = core::ptr::addr_of!(rv_user_probe_start);
        let end = core::ptr::addr_of!(rv_user_probe_end);
        core::slice::from_raw_parts(start, end as usize - start as usize)
    }
}

/// 获取探测代码段中 `rv_user_probe_loop` 入口相对于起始地址的字节偏移（先向用户内存/栈写入寄存器与浮点运算结果，随后进入死循环等待中断抢占）。
#[cfg(feature = "ktest")]
pub fn loop_offset() -> usize {
    core::ptr::addr_of!(rv_user_probe_loop) as usize
        - core::ptr::addr_of!(rv_user_probe_start) as usize
}

/// 获取探测代码段中 `rv_user_probe_spin` 死循环指令相对于起始地址的字节偏移（进入该位置后不再改动任何寄存器，用于验证跨中断或跨核迁移前后的上下文一致性）。
#[cfg(feature = "ktest")]
pub fn loop_spin_offset() -> usize {
    core::ptr::addr_of!(rv_user_probe_spin) as usize
        - core::ptr::addr_of!(rv_user_probe_start) as usize
}

/// 获取探测代码段中触发 `exit` 系统调用（`a7 = 93`）的指令相对于起始地址的字节偏移。
#[cfg(feature = "ktest")]
pub fn exit_offset() -> usize {
    core::ptr::addr_of!(rv_user_probe_exit) as usize
        - core::ptr::addr_of!(rv_user_probe_start) as usize
}

/// 获取探测代码段中通过用户寄存器 `a5` 触发指定类型（读取/写入/跳转执行）内存访问故障的指令字节偏移。
#[cfg(feature = "ktest")]
pub fn fault_offset(access: UserAccess) -> usize {
    let target = match access {
        UserAccess::Read => core::ptr::addr_of!(rv_user_probe_read),
        UserAccess::Write => core::ptr::addr_of!(rv_user_probe_write),
        UserAccess::Execute => core::ptr::addr_of!(rv_user_probe_execute),
    };
    target as usize - core::ptr::addr_of!(rv_user_probe_start) as usize
}
