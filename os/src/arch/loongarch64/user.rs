//! LoongArch64 用户态（PLV3）上下文定义与进出用户态运行接口。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 用户态执行层，向上为任务调度与系统调用主循环提供用户态寄存器上下文
//! （[`UserContext`]）、进入 PLV3 并等待下一次陷入的同步入口（[`run_user`]），以及信号返回跳板代码切片
//! （[`signal_return_code`]）；向下调用 `user::entry` 中的裸汇编入口与异常向量完成硬件状态切换。
//!
//! # 核心设计与 `UserContext` 内存布局
//! 每个用户态线程独占一个 32 字节对齐的 [`UserContext`]，其内存布局直接由 `entry.rs` 中的汇编通过
//! `offset_of!` 访问：
//!
//! ```text
//! +---------------------------------------------------------------+ 偏移 0 (32 字节对齐)
//! | gpr: [usize; 32]       (256 B) 通用寄存器 r0..r31             |
//! |   - r1 (ra): 返回地址          - r2 (tp): 线程指针 (TLS)      |
//! |   - r3 (sp): 用户栈指针        - r4..r11 (a0..a7): 参数/返回值|
//! +---------------------------------------------------------------+ 偏移 256
//! | pc: usize              (8 B)   用户态程序计数器（对应 CSR ERA）|
//! | (对齐填充 24 B，使 fpr 满足 32 字节对齐)                       |
//! +---------------------------------------------------------------+ 偏移 288
//! | fpr: [FpRegister; 32]  (1024 B) 32 个 256 位浮点/向量寄存器槽 |
//! |   - 标量 FP 使用 lanes[0] (64 位)                             |
//! |   - LSX 使用 lanes[0..2] (128 位)                             |
//! |   - LASX 使用 lanes[0..4] (256 位)                            |
//! +---------------------------------------------------------------+ 偏移 1312
//! | fcsr: usize            (8 B)   浮点控制与状态寄存器 ($fcsr0)  |
//! | fcc: usize             (8 B)   8 个条件标志位打包 (FCC0..7)   |
//! | estat: usize           (8 B)   陷入时的 CSR ESTAT 快照        |
//! | badv: usize            (8 B)   陷入时的 CSR BADV 出错虚址快照 |
//! | kernel_sp: usize       (8 B)   本次进入 PLV3 时保存的内核栈 SP|
//! +---------------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`UserContext`] 由当前正在运行的普通任务独占访问，不跨核共享。
//! - 调用 [`run_user`] 前必须关闭当前 CPU 中断且不得持有任何自旋锁；当用户态因外部硬件中断陷入时，
//!   [`run_user`] 会在返回 `UserTrap::Interrupt` 前直接调用硬件中断分发器。

use crate::arch::loongarch64::register as reg;
use crate::arch::loongarch64::user::entry::la_run_user;
pub(in crate::arch::loongarch64) use crate::arch::loongarch64::user::entry::{
    la_user_kernel_refill_fault, la_user_trap,
};
use crate::arch::loongarch64::{csr_read, handle_hardware_interrupt, irq_enabled};
use crate::arch::{UserAccess, UserTrap};

mod entry;
mod signal_return;

/// 单个浮点/向量寄存器的 256 位物理存储槽。
///
/// LoongArch64 的 64 位标量浮点寄存器（`$f0`–`$f31`）、128 位向量寄存器（`$vr0`–`$vr31`）与
/// 256 位高级向量寄存器（`$xr0`–`$xr31`）在硬件上共享同一组寄存器堆。按最大宽度（256 位）
/// 并配合 32 字节对齐存放，可同时满足 `fld.d`/`vld`/`xvld` 指令的访存对齐要求。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C, align(32))]
pub struct FpRegister {
    /// 按 64 位通道拆分的 256 位寄存器内容：`lanes[0]` 为标量/低 64 位，`lanes[0..2]` 为 LSX，`lanes[0..4]` 为 LASX。
    pub lanes: [u64; 4],
}

/// LoongArch64 用户态（PLV3）完整执行现场与陷入快照。
///
/// 包含用户态通用寄存器、PC、浮点/向量寄存器组，以及从 PLV3 陷入 PLV0 时由汇编入口回填的异常 CSR
/// 快照与内核续接栈指针。
#[repr(C, align(32))]
pub struct UserContext {
    /// 32 个通用寄存器（`r0`–`r31`）；其中 `gpr[3]` 为用户栈指针 `sp`，`gpr[4]`–`gpr[11]` 为 `a0`–`a7`。
    pub gpr: [usize; 32],
    /// 用户态程序计数器（进入 PLV3 时写入 `CSR_ERA`，陷入时从 `CSR_ERA` 保存）。
    pub pc: usize,
    /// 32 个浮点/向量寄存器槽（按 CPU 实际开启的 FP/LSX/LASX 宽度保存和恢复）。
    pub fpr: [FpRegister; 32],
    /// 浮点控制与状态寄存器（`$fcsr0`）。
    pub fcsr: usize,
    /// 浮点条件标志寄存器打包值：`FCC0`–`FCC7` 依次存放在第 0–7 字节中，与 Linux 信号帧 ABI 一致。
    pub fcc: usize,
    /// 最近一次从用户态陷入内核时由汇编保存的 `CSR_ESTAT`（异常与中断状态寄存器）快照。
    estat: usize,
    /// 最近一次从用户态陷入内核时由汇编保存的 `CSR_BADV`（出错虚拟地址寄存器）快照。
    badv: usize,
    /// 本次调用 [`run_user`] 进入 PLV3 前在内核栈上分配的 160 字节续接帧栈顶地址。
    kernel_sp: usize,
}

impl UserContext {
    /// 创建初始用户态上下文：设置入口 `pc` 与用户栈指针 `sp`（`r3`），其余通用寄存器（含 `r2/tp`）、浮点寄存器与异常状态全部清零。
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

    /// 为 `fork` / `clone` 派生子任务的用户态上下文：完整复制通用寄存器、PC 与浮点/向量状态，并将内核续接栈指针与异常 CSR 快照清零。
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

/// 返回内核只读段中预编译的 `rt_sigreturn` 用户态跳板机器码切片，供 [`Mm`](crate::mm::space::Mm) 复制到用户态信号返回可执行页。
pub fn signal_return_code() -> &'static [u8] {
    let start = core::ptr::addr_of!(la_user_signal_return_start);
    let end = core::ptr::addr_of!(la_user_signal_return_end) as usize;
    // SAFETY:
    // 1. `la_user_signal_return_start` 与 `la_user_signal_return_end` 由 `signal_return.rs` 在同一 `.rodata.user_signal_return`
    //    只读节中定义，`end >= start` 且整个区间在内核生命周期内静态常驻、已完成初始化且不可修改；
    // 2. `u8` 对齐要求为 1，转换为 `&'static [u8]` 不会产生未对齐访问或可变别名冲突。
    unsafe { core::slice::from_raw_parts(start, end - start as usize) }
}

/// 切换到 PLV3 执行用户态代码，直到发生系统调用、页面异常、同步异常或外部中断后返回对应的 [`UserTrap`]。
///
/// 若陷入原因为外部硬件中断，本函数会在内核态直接调用 [`handle_hardware_interrupt`] 完成中断分发，
/// 随后返回 [`UserTrap::Interrupt`]，且不推进用户态 `pc`。
///
/// # Safety
/// 1. 只能由当前普通任务的唯一执行流调用，`context` 必须由当前任务独占且在本次陷入返回前保持有效；
/// 2. 当前激活的用户地址空间（`Mm`）根页表及所有已映射物理页必须在整个 PLV3 执行期间保持存活；
/// 3. 调用时当前 CPU 必须处于关中断状态（`!irq_enabled()`）且内核未启用浮点扩展单元（`CSR_EUEN == 0`）；
/// 4. 当前任务的内核栈、内核代码段及 `CpuLocal`（`KS0`）必须映射在有效的永久内核虚拟地址上。
pub unsafe fn run_user(context: &mut UserContext) -> UserTrap {
    // Step 1: 校验进入用户态前的特权前置条件（本地中断已关闭，且内核态未开启浮点/向量单元）。
    assert!(!irq_enabled(), "user entry requires IRQ off");
    assert_eq!(
        csr_read::<{ reg::CSR_EUEN }>(),
        0,
        "user entry requires kernel EUEN off"
    );

    // Step 2: 调用裸汇编入口保存内核被调用者保存寄存器与入口 CSR，加载用户现场并执行 `ertn` 进入 PLV3。
    // SAFETY:
    // 1. 前置 `assert!` 已确认本地中断与内核 `CSR_EUEN` 关闭；
    // 2. 调用方契约保证 `context` 独占有效、当前任务内核栈与 `Mm` 页表存活；
    // 3. `la_run_user` 会在用户态陷入后精确恢复当前 Rust 栈帧及入口 CSR 状态。
    unsafe { la_run_user(context) };

    // Step 3: 校验从汇编异常入口返回后的内核态不变量（中断保持关闭、浮点扩展单元已在汇编中关闭）。
    assert!(!irq_enabled(), "user trap returned with IRQ enabled");
    assert_eq!(
        csr_read::<{ reg::CSR_EUEN }>(),
        0,
        "user trap returned with kernel FP enabled"
    );

    // Step 4: 解析汇编保存在 `context.estat` 与 `context.badv` 中的硬件异常码，转换为架构无关的 `UserTrap`。
    match (context.estat >> reg::ESTAT_ECODE_SHIFT) & reg::ESTAT_ECODE_MASK {
        0 => {
            handle_hardware_interrupt(context.estat);
            UserTrap::Interrupt
        }
        reg::ECODE_SYS => UserTrap::Syscall,
        reg::ECODE_PIL | reg::ECODE_PNR => UserTrap::PageFault {
            address: context.badv,
            access: UserAccess::Read,
        },
        reg::ECODE_PIS | reg::ECODE_PME => UserTrap::PageFault {
            address: context.badv,
            access: UserAccess::Write,
        },
        reg::ECODE_PIF | reg::ECODE_PNX => UserTrap::PageFault {
            address: context.badv,
            access: UserAccess::Execute,
        },
        code => UserTrap::Exception {
            code,
            address: context.badv,
        },
    }
}

/// 处理用户异常向量激活窗口内发生的内核态（PLV0）致命异常或 TLB 重填故障。
///
/// 当 `CSR_EENTRY` 已切换为 `la_user_trap` 后，若在内核态切换窗口内意外触发同步异常或 TLB Refill，
/// 汇编故障恢复路径会先从 `CSR_KS3` 恢复可信的内核任务栈与入口 CSR，再调用本函数输出故障现场并终止执行。
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
            if (cause >> reg::ESTAT_ECODE_SHIFT) & reg::ESTAT_ECODE_MASK == reg::ECODE_INE {
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

/// 返回内核测试专用的位置无关用户态探测机器码切片，可复制到任意用户态可执行页中运行。
#[cfg(feature = "ktest")]
pub(crate) fn user_code() -> &'static [u8] {
    let start = core::ptr::addr_of!(la_user_code_start);
    let end = core::ptr::addr_of!(la_user_code_end) as usize;
    // SAFETY:
    // 1. `la_user_code_start` 与 `la_user_code_end` 由 `entry.rs` 在 `.text.la_user_probe` 节中定义，
    //    界定了同一段编译期静态生成的只读机器码，`end >= start`；
    // 2. 该代码段在整个内核生命周期内常驻内存且不会被修改，`u8` 对齐为 1，构造 `'static` 切片完全合法。
    unsafe { core::slice::from_raw_parts(start, end - start as usize) }
}

/// 返回用户态寄存器完整性探测入口（`la_user_loop_entry`）相对于 [`user_code`] 起始处的字节偏移。
#[cfg(feature = "ktest")]
pub(crate) fn loop_offset() -> usize {
    core::ptr::addr_of!(la_user_loop_entry) as usize
        - core::ptr::addr_of!(la_user_code_start) as usize
}

/// 返回用户态探测代码完成寄存器写入后的原地自旋指令（`la_user_loop_spin`）相对于 [`user_code`] 起始处的字节偏移。
#[cfg(feature = "ktest")]
pub(crate) fn loop_spin_offset() -> usize {
    core::ptr::addr_of!(la_user_loop_spin) as usize
        - core::ptr::addr_of!(la_user_code_start) as usize
}

/// 返回发起退出系统调用（`SYS_exit = 93, a0 = 43`）的指令入口（`la_user_exit_entry`）相对于 [`user_code`] 起始处的字节偏移。
#[cfg(feature = "ktest")]
pub(crate) fn exit_offset() -> usize {
    core::ptr::addr_of!(la_user_exit_entry) as usize
        - core::ptr::addr_of!(la_user_code_start) as usize
}

/// 根据内存访问类型返回触发用户态读、写或取指异常的指令入口相对于 [`user_code`] 起始处的字节偏移（目标故障地址由用户寄存器 `a5` 指定）。
#[cfg(feature = "ktest")]
pub(crate) fn fault_offset(access: UserAccess) -> usize {
    let entry = match access {
        UserAccess::Read => core::ptr::addr_of!(la_user_fault_read),
        UserAccess::Write => core::ptr::addr_of!(la_user_fault_write),
        UserAccess::Execute => core::ptr::addr_of!(la_user_fault_execute),
    };
    entry as usize - core::ptr::addr_of!(la_user_code_start) as usize
}
