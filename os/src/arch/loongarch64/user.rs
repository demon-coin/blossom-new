//! PLV3 私有现场；返回同一普通 Task 的 Rust 调用栈。

use super::{csr_read, handle_hardware_interrupt, irq_enabled};
use crate::arch::{UserAccess, UserTrap};

mod entry;
mod signal_return;

/// FP、LSX 与 LASX 共享同一组寄存器，最宽槽同时保证真实 32 字节对齐。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C, align(32))]
pub struct FpRegister {
    pub lanes: [u64; 4],
}

#[repr(C, align(32))]
pub struct UserContext {
    pub gpr: [usize; 32],
    pub pc: usize,
    pub fpr: [FpRegister; 32],
    pub fcsr: usize,
    /// FCC0–7 各占一个字节，与浮点条件寄存器 ABI 一致。
    pub fcc: usize,
    estat: usize,
    badv: usize,
    kernel_sp: usize,
}

impl UserContext {
    /// 初始化完整私有现场，SP 使用 r3，用户 TP/r2 保持零 TLS。
    pub fn new(pc: usize, sp: usize) -> Self {
        let mut context = Self {
            gpr: [0; 32],
            pc,
            fpr: [FpRegister { lanes: [0; 4] }; 32],
            fcsr: 0,
            fcc: 0,
            estat: 0,
            badv: 0,
            kernel_sp: 0,
        };
        context.gpr[3] = sp;
        context
    }

    /// 派生完整整数与最宽浮点现场，FCC 按用户 ABI 复制，不继承内核续接或异常 CSR。
    pub(crate) fn fork(&self) -> Self {
        Self {
            gpr: self.gpr,
            pc: self.pc,
            fpr: self.fpr,
            fcsr: self.fcsr,
            fcc: self.fcc,
            estat: 0,
            badv: 0,
            kernel_sp: 0,
        }
    }
}

extern "C" {
    static la_user_signal_return_start: u8;
    static la_user_signal_return_end: u8;
}

/// 借用内核持有的固定 rt_sigreturn 指令字节，供独立 Mm 安装用户 RX 返回页。
pub fn signal_return_code() -> &'static [u8] {
    let start = core::ptr::addr_of!(la_user_signal_return_start);
    let end = core::ptr::addr_of!(la_user_signal_return_end) as usize;
    // SAFETY: 两个链接符号界定同一永久只读片段，end 位于 start 之后，字节内容全部已初始化；
    // 返回切片只借内核静态字节，不借用户映射或建立可并发写的用户内容引用。
    unsafe { core::slice::from_raw_parts(start, end - start as usize) }
}

extern "C" {
    /// 保存本次内核续接并执行 PLV3，异常保存完整用户现场后返回原续接。
    fn la_run_user(context: *mut UserContext);
}

/// 真实进入 PLV3；中断沿正常硬中断路径完成后返回值快照，不推进用户 PC。
///
/// # Safety
/// 仅当前普通 Task 的唯一执行者可调用；context 独占且保活，当前 Mm 根及全部可达页保活。
/// 本核 IRQ 必须关闭，当前 Rust 栈、代码和 CpuLocal 永久映射有效；不得跨入口保留活跃 FP。
pub unsafe fn run_user(context: &mut UserContext) -> UserTrap {
    assert!(!irq_enabled(), "user entry requires IRQ off");
    assert_eq!(csr_read::<2>(), 0, "user entry requires kernel EUEN off");
    // SAFETY: 调用者独占私有现场并保活当前 Mm；汇编保存本次真实 Rust SP，返回时恢复入口 CSR。
    unsafe { la_run_user(context) };
    assert!(!irq_enabled(), "user trap returned with IRQ enabled");
    assert_eq!(
        csr_read::<2>(),
        0,
        "user trap returned with kernel FP enabled"
    );
    match (context.estat >> 16) & 0x3f {
        0 => {
            handle_hardware_interrupt(context.estat);
            UserTrap::Interrupt
        }
        11 => UserTrap::Syscall,
        1 | 5 => UserTrap::PageFault {
            address: context.badv,
            access: UserAccess::Read,
        },
        2 | 4 => UserTrap::PageFault {
            address: context.badv,
            access: UserAccess::Write,
        },
        3 | 6 => UserTrap::PageFault {
            address: context.badv,
            access: UserAccess::Execute,
        },
        code => UserTrap::Exception {
            code,
            address: context.badv,
        },
    }
}

/// 在汇编恢复受信 Task 栈及内核状态后终止入口内核故障，保留原 fault SP 和专属 CSR 原因。
#[no_mangle]
extern "C" fn loongarch_user_kernel_fault(
    pc: usize,
    fault_sp: usize,
    cause: usize,
    bad: usize,
    is_refill: usize,
) -> ! {
    #[cfg(feature = "ktest")]
    if is_refill != 0 {
        crate::ktest::fatal_event(
            "refill",
            format_args!(
                "refill trap pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
                pc, fault_sp, cause, bad
            ),
        );
    } else {
        crate::ktest::fatal_event(
            if (cause >> 16) & 0x3f == 0x0d {
                "illegal"
            } else {
                "panic"
            },
            format_args!(
                "synchronous trap pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
                pc, fault_sp, cause, bad
            ),
        );
    }
    #[cfg(not(feature = "ktest"))]
    {
        let _ = is_refill;
        crate::console::fatal(pc, fault_sp, cause, bad)
    }
}

#[cfg(feature = "ktest")]
extern "C" {
    static la_user_code_start: u8;
    static la_user_code_end: u8;
    static la_user_loop_entry: u8;
    static la_user_loop_spin: u8;
    static la_user_exit_entry: u8;
    static la_user_fault_read: u8;
    static la_user_fault_write: u8;
    static la_user_fault_execute: u8;
}

/// 借用可复制到用户指令页的永久相对寻址代码；片段不含内核地址重定位。
#[cfg(feature = "ktest")]
pub(crate) fn user_code() -> &'static [u8] {
    let start = core::ptr::addr_of!(la_user_code_start);
    let end = core::ptr::addr_of!(la_user_code_end) as usize;
    // SAFETY: 两个汇编标签界定同一永久可读 text 片段，链接后不会移动或释放；u8 无对齐限制。
    unsafe { core::slice::from_raw_parts(start, end - start as usize) }
}

/// 给出完整寄存器循环的初始化入口相对于用户代码起点的偏移。
#[cfg(feature = "ktest")]
pub(crate) fn loop_offset() -> usize {
    core::ptr::addr_of!(la_user_loop_entry) as usize
        - core::ptr::addr_of!(la_user_code_start) as usize
}

/// 给出只改变 PC 的稳态分支偏移，便于在真实中断后比较私有完整现场。
#[cfg(feature = "ktest")]
pub(crate) fn loop_spin_offset() -> usize {
    core::ptr::addr_of!(la_user_loop_spin) as usize
        - core::ptr::addr_of!(la_user_code_start) as usize
}

/// 给出请求号 93、参数 43 的正常 syscall 指令入口偏移。
#[cfg(feature = "ktest")]
pub(crate) fn exit_offset() -> usize {
    core::ptr::addr_of!(la_user_exit_entry) as usize
        - core::ptr::addr_of!(la_user_code_start) as usize
}

/// 按访问类型选择真实用户指令入口，目标地址由用户 a5 提供。
#[cfg(feature = "ktest")]
pub(crate) fn fault_offset(access: UserAccess) -> usize {
    let entry = match access {
        UserAccess::Read => core::ptr::addr_of!(la_user_fault_read),
        UserAccess::Write => core::ptr::addr_of!(la_user_fault_write),
        UserAccess::Execute => core::ptr::addr_of!(la_user_fault_execute),
    };
    entry as usize - core::ptr::addr_of!(la_user_code_start) as usize
}
