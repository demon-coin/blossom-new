//! 每个用户线程独占的整数、浮点现场及本次调用的受信续接。

use crate::arch::{UserAccess, UserTrap};
use core::arch::asm;

mod entry;
mod signal_return;

#[repr(C, align(16))]
pub struct UserContext {
    pub gpr: [usize; 32],
    pub pc: usize,
    pub fpr: [u64; 32],
    pub fcsr: usize,
    // 只在汇编调用内有效，每次返回后清零，不携带跨调度的 CpuLocal。
    kernel_sp: usize,
    scause: usize,
    stval: usize,
}

impl UserContext {
    /// 建立完整清零的私有用户现场，SP 采用用户 ABI 指定的 x2。
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

    /// 派生独立用户现场，只复制用户寄存器，受信续接和上次异常记录保持清零。
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

/// 借用内核持有的固定 rt_sigreturn 指令字节，供独立 Mm 安装用户 RX 返回页。
pub fn signal_return_code() -> &'static [u8] {
    let start = core::ptr::addr_of!(rv_user_signal_return_start);
    let end = core::ptr::addr_of!(rv_user_signal_return_end) as usize;
    // SAFETY: 两个链接符号界定同一永久只读片段，end 位于 start 之后，字节内容全部已初始化；
    // 返回切片只借内核静态字节，不借用户映射或建立可并发写的用户内容引用。
    unsafe { core::slice::from_raw_parts(start, end - start as usize) }
}

extern "C" {
    /// 保存当前 Rust 调用续接并通过 sret 执行用户现场，trap 后回到同一调用栈。
    ///
    /// # Safety
    /// context 须独占且位于当前根的内核映射中；IRQ 关闭，FS Off，用户根及内核高半存活。
    fn rv_run_user(context: *mut UserContext);
}

/// 在当前普通 Task 中执行用户现场，返回已完成硬件分派的小 trap 快照且不推进 PC。
///
/// # Safety
/// 调用者是当前普通 Task 的唯一执行者，独占 context，当前 MM 根与内核高半映射有效。
/// 调用前 IRQ 关闭且没有活跃内核 FP；不得在中断域或持普通锁时调用。
/// MM 与完整页表必须保活到返回，只有返回到 Rust 且退出 HardIrqGuard 后才允许调度。
pub unsafe fn run_user(context: &mut UserContext) -> UserTrap {
    assert!(!super::irq_enabled());
    let status: usize;
    // SAFETY: 本核处于 S-mode，仅读取状态以拒绝覆盖活跃的内核 FP 现场。
    unsafe {
        asm!("csrr {status}, sstatus", status = out(reg) status, options(nomem, nostack));
    }
    assert_eq!(
        status & 0x6000,
        0,
        "kernel FP must be disabled before user entry"
    );
    // SAFETY: 调用者承担当前 Task、独占现场、活动 MM 与映射寿命；IRQ 与 FS 已核对。
    // 汇编只在此调用内保存受信续接 SP/TP/GP，用户 trap 返回后已清零 sscratch 并关闭 FS。
    unsafe {
        rv_run_user(context);
    }
    if context.scause & (1usize << 63) != 0 {
        super::rv_dispatch_interrupt(context.scause, context.pc, context.gpr[2], context.stval);
        return UserTrap::Interrupt;
    }
    match context.scause {
        8 => UserTrap::Syscall,
        12 => UserTrap::PageFault {
            address: context.stval,
            access: UserAccess::Execute,
        },
        13 => UserTrap::PageFault {
            address: context.stval,
            access: UserAccess::Read,
        },
        15 => UserTrap::PageFault {
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

/// 取得可整体复制到用户页的真实指令，所有数据地址由用户 SP/a5/a6 提供。
#[cfg(feature = "ktest")]
pub fn user_code() -> &'static [u8] {
    // SAFETY: 汇编符号界定同一静态只读指令块，end 位于 start 之后；内核映射永久有效。
    unsafe {
        let start = core::ptr::addr_of!(rv_user_probe_start);
        let end = core::ptr::addr_of!(rv_user_probe_end);
        core::slice::from_raw_parts(start, end as usize - start as usize)
    }
}

/// 取得写入真实用户数据、栈和浮点结果后进入长循环的入口偏移。
#[cfg(feature = "ktest")]
pub fn loop_offset() -> usize {
    core::ptr::addr_of!(rv_user_probe_loop) as usize
        - core::ptr::addr_of!(rv_user_probe_start) as usize
}

/// 取得不再修改用户寄存器的稳态跳转偏移，供跨中断与迁核现场比较。
#[cfg(feature = "ktest")]
pub fn loop_spin_offset() -> usize {
    core::ptr::addr_of!(rv_user_probe_spin) as usize
        - core::ptr::addr_of!(rv_user_probe_start) as usize
}

/// 取得发出正常退出 syscall 的用户指令偏移。
#[cfg(feature = "ktest")]
pub fn exit_offset() -> usize {
    core::ptr::addr_of!(rv_user_probe_exit) as usize
        - core::ptr::addr_of!(rv_user_probe_start) as usize
}

/// 取得通过用户 a5 访问受控故障地址的真实 load/store/execute 指令偏移。
#[cfg(feature = "ktest")]
pub fn fault_offset(access: UserAccess) -> usize {
    let target = match access {
        UserAccess::Read => core::ptr::addr_of!(rv_user_probe_read),
        UserAccess::Write => core::ptr::addr_of!(rv_user_probe_write),
        UserAccess::Execute => core::ptr::addr_of!(rv_user_probe_execute),
    };
    target as usize - core::ptr::addr_of!(rv_user_probe_start) as usize
}
