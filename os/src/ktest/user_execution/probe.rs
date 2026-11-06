//! 用户态执行与虚拟内存测试的底层探针与映像构造辅助模块。
//!
//! # 模块职责与架构定位
//! 本模块为 `user_execution` 测试套件家族（如寄存器上下文切换、缺页异常处理、浮点/向量现场隔离、
//! 跨核 TLB 同步等）提供统一的最小用户地址空间构造器、寄存器现场初始化器以及单步/循环用户态陷入驱动器。
//! - **上游调用者**：`user_execution` 下的各个子测试套件（如 `vm`、`context`、`fpu` 等）。
//! - **下游依赖**：`arch` 层的用户态陷入入口与预置机器码探针、`mm` 层的用户地址空间（`Mm`）与页分配器、
//!   以及 `task` 层的 `UserThread` 执行上下文。
//!
//! # 核心设计与地址空间布局
//! 每个测试探针通过 [`image`] 构造一个独立的 `Mm` 地址空间与初始 `UserContext`，其虚拟地址布局如下：
//!
//! ```text
//! 虚拟地址范围                 权限    用途说明
//! +-------------------------+-------+-----------------------------------------------+
//! | 0x10000 .. 0x11000      | R-X   | CODE: 架构相关的微探针指令页 (arch::user_code) |
//! +-------------------------+-------+-----------------------------------------------+
//! | 0x20000 .. 0x21000      | RW-   | DATA: 探针指令写出寄存器/浮点结果的数据缓冲页 |
//! +-------------------------+-------+-----------------------------------------------+
//! | 0x30000 .. 0x31000      | RW-   | STACK: 用户态测试栈，初始 SP 指向 0x31000     |
//! +-------------------------+-------+-----------------------------------------------+
//! | 0x40000 ..              | 按需  | TARGET: 供缺页、COW、uaccess 自由映射的测试区 |
//! +-------------------------+-------+-----------------------------------------------+
//! ```
//!
//! # 并发与生命周期契约
//! - [`page`] 仅在 `Mm` 尚未包装进 `Arc` 共享前调用，直接通过内核直映区完成物理页内容初始化并移交所有权。
//! - [`probe_words`] 仅在关联的 `UserThread` 已经从用户态返回内核且不再并发写 `DATA` 页时，从内核直映地址读取结果。

use alloc::sync::Arc;
use core::hint::black_box;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::arch::{self, UserAccess, UserContext, UserTrap};
#[cfg(target_arch = "loongarch64")]
use crate::arch::{user_fp_lanes, FpRegister};
use crate::ktest::ChildToken;
use crate::mm::space::Mm;
use crate::mm::user_table::PagePermissions;
use crate::task::UserThread;
use crate::{cpu, mm, sched, task};
use ::ktest::TestError;

/// 用户态探针代码页起始虚拟地址（`0x10000`）。
pub(super) const CODE: usize = 0x10000;
/// 用户态探针数据页起始虚拟地址（`0x20000`），用于存放指令执行后的写回观测数据。
pub(super) const DATA: usize = 0x20000;
/// 用户态栈页起始虚拟地址（`0x30000`）。
const STACK: usize = 0x30000;
/// 用户态初始栈顶虚拟地址（`STACK + PAGE_SIZE`）。
pub(super) const STACK_TOP: usize = STACK + mm::PAGE_SIZE;
/// 供虚拟内存与缺页测试自由映射的目标虚拟地址起点（`0x40000`）。
pub(super) const TARGET: usize = 0x40000;

/// 只读可执行权限（代码页）。
const RX: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: true,
};
/// 可读可写不可执行权限（数据页与栈页）。
pub(super) const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
/// 只读不可执行权限。
pub(super) const RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};
/// 纯可执行不可读写权限。
pub(super) const EX: PagePermissions = PagePermissions {
    read: false,
    write: false,
    execute: true,
};
/// 无任何访问权限（用于验证权限拒绝故障）。
pub(super) const NONE: PagePermissions = PagePermissions {
    read: false,
    write: false,
    execute: false,
};

/// RISC-V 架构下线程指针寄存器 `tp` (`x4`) 在 `gpr` 数组中的下标。
#[cfg(target_arch = "riscv64")]
const TP: usize = 4;
/// LoongArch64 架构下线程指针寄存器 `tp` (`$r2`) 在 `gpr` 数组中的下标。
#[cfg(target_arch = "loongarch64")]
const TP: usize = 2;
/// RISC-V 架构下全局指针寄存器 `gp` (`x3`) 在 `gpr` 数组中的下标。
#[cfg(target_arch = "riscv64")]
const GP: usize = 3;
/// LoongArch64 架构下保留寄存器 `$r21` 在 `gpr` 数组中的下标（测试中用作 GP 探针槽位）。
#[cfg(target_arch = "loongarch64")]
const GP: usize = 21;
/// RISC-V 架构下参数/返回值寄存器 `a0` (`x10`) 在 `gpr` 数组中的下标。
#[cfg(target_arch = "riscv64")]
pub(super) const A0: usize = 10;
/// LoongArch64 架构下参数/返回值寄存器 `a0` (`$r4`) 在 `gpr` 数组中的下标。
#[cfg(target_arch = "loongarch64")]
pub(super) const A0: usize = 4;
/// RISC-V 架构下临时参数寄存器 `a5` (`x15`) 在 `gpr` 数组中的下标（探针中存放访存目标地址）。
#[cfg(target_arch = "riscv64")]
pub(super) const A5: usize = 15;
/// LoongArch64 架构下临时参数寄存器 `a5` (`$r9`) 在 `gpr` 数组中的下标（探针中存放访存目标地址）。
#[cfg(target_arch = "loongarch64")]
pub(super) const A5: usize = 9;
/// RISC-V 架构下参数寄存器 `a6` (`x16`) 在 `gpr` 数组中的下标（探针中存放 `DATA` 页基址）。
#[cfg(target_arch = "riscv64")]
const A6: usize = 16;
/// LoongArch64 架构下参数寄存器 `a6` (`$r10`) 在 `gpr` 数组中的下标（探针中存放 `DATA` 页基址）。
#[cfg(target_arch = "loongarch64")]
const A6: usize = 10;
/// RISC-V 架构下系统调用号寄存器 `a7` (`x17`) 在 `gpr` 数组中的下标。
#[cfg(target_arch = "riscv64")]
pub(super) const A7: usize = 17;
/// LoongArch64 架构下系统调用号寄存器 `a7` (`$r11`) 在 `gpr` 数组中的下标。
#[cfg(target_arch = "loongarch64")]
pub(super) const A7: usize = 11;
/// RISC-V 架构下临时寄存器 `t0` (`x5`) 在 `gpr` 数组中的下标（探针中存放读出的数值）。
#[cfg(target_arch = "riscv64")]
pub(super) const T0: usize = 5;
/// LoongArch64 架构下临时寄存器 `t0` (`$r12`) 在 `gpr` 数组中的下标（探针中存放读出的数值）。
#[cfg(target_arch = "loongarch64")]
pub(super) const T0: usize = 12;

/// 驱动用户线程对指定虚拟地址执行一次指定方向的访存指令，并在中途自动处理缺页异常。
///
/// # 执行流程
/// 1. 将用户上下文的 `PC` 设置为对应访问方向（读/写/执行）的探针指令入口，`A5` 设置为目标虚拟地址。
/// 2. 循环进入用户态执行：若遇到 `PageFault`，校验故障地址与访问方向后调用 `mm.fault()` 修复页表，并在同一 `PC` 重新进入用户态。
/// 3. 当探针指令顺利执行完毕并触发 `Syscall` 陷入时返回。
///
/// # 返回值
/// 成功时返回 `(触发并修复的缺页次数, 探针指令存入 T0 寄存器的值)`。
pub(super) fn access(
    user: &UserThread,
    token: &ChildToken,
    stop: &AtomicBool,
    address: usize,
    direction: UserAccess,
) -> Result<(usize, usize), TestError> {
    // Step 1: 设置用户态探针指令入口地址与目标访存地址寄存器 A5。
    {
        let mut context = user.context();
        context.pc = CODE + arch::fault_offset(direction);
        context.gpr[A5] = address;
    }
    let mm = user.mm().expect("user access has no MM");
    let mut faults = 0;
    // Step 2: 循环进入用户态执行，处理中途的缺页异常直到探针通过系统调用退出。
    loop {
        match event(user, token, stop)? {
            UserTrap::PageFault {
                address: actual,
                access,
            } => {
                assert_eq!(actual, address);
                assert!(
                    access == direction
                        || direction == UserAccess::Write && access == UserAccess::Read
                );
                let pc = user.context().pc;
                mm.fault(actual, access)
                    .map_err(|_| TestError("user VM fault failed"))?;
                assert_eq!(user.context().pc, pc);
                faults += 1;
                assert!(faults <= 2, "resolved user fault repeated");
            }
            UserTrap::Syscall => return Ok((faults, user.context().gpr[T0])),
            trap => panic!("unexpected user VM trap: {:?}", trap),
        }
    }
}

/// 通过内核安全访存接口 `read_u64` 读取用户虚拟地址处的 64 位无符号整数。
pub(super) fn word(mm: &Mm, address: usize) -> u64 {
    mm.read_u64(address).expect("user VM word is inaccessible")
}

/// 分配一个清零的物理页，填入初始字节内容后以指定权限映射到尚未共享的 `Mm` 地址空间中。
///
/// # 返回值
/// 返回该物理页在内核直接映射区中的虚拟地址（`usize`），供测试在用户线程退出后直接读取页内结果。
pub(super) fn page(
    mm: &mut Mm,
    address: usize,
    permissions: PagePermissions,
    contents: &[u8],
) -> Result<usize, TestError> {
    if mm::PAGE_SIZE < contents.len() {
        return Err(TestError("user probe does not fit a basic page"));
    }
    // Step 1: 分配单个清零物理页，并将初始数据拷贝到页首。
    let mut pages = mm::alloc_pages(0, true).ok_or(TestError("NoMemory: user probe page"))?;
    let direct = pages.as_mut_ptr().cast::<u8>();
    // SAFETY:
    // 1. `pages` 是刚刚分配且完整清零的独占物理页，`direct` 指向其内核直映区起始地址，容量为 `PAGE_SIZE`。
    // 2. 前置条件 `mm::PAGE_SIZE < contents.len()` 已排除越界可能，且内核切片 `contents` 与新分配物理页不重叠。
    unsafe { core::ptr::copy_nonoverlapping(contents.as_ptr(), direct, contents.len()) };
    // Step 2: 将初始化完毕的物理页所有权移交给用户地址空间 `mm`。
    // SAFETY:
    // 1. `pages` 已完成初始化且不存在任何存活的 Rust 引用或并发访问。
    // 2. `mm` 为调用方独占的可变引用，尚未向其他线程发布，`pages` 的释放责任自此唯一移交 `mm` 管理。
    unsafe { mm.map_owned(address, pages, permissions, permissions) }
        .map_err(|_| TestError("user probe mapping failed"))?;
    Ok(direct as usize)
}

/// 构造包含代码页、数据页和栈页的标准测试用户地址空间（`Mm`）及带槽位特征值的初始寄存器上下文（`UserContext`）。
///
/// # 参数
/// - `offset`：初始 `PC` 相对于 `CODE` 页起始地址的字节偏移。
/// - `slot`：测试并发槽位编号，用于为不同用户线程生成可区分的通用寄存器、TLS/GP 与浮点/向量寄存器初值。
///
/// # 返回值
/// 返回 `(构造好的 Mm, 初始 UserContext, DATA 页的内核直映地址)`。
pub(super) fn image(offset: usize, slot: usize) -> Result<(Mm, UserContext, usize), TestError> {
    // Step 1: 创建空的用户地址空间并映射代码页、数据页与栈页。
    let mut mm = Mm::try_new().map_err(|_| TestError("NoMemory: user probe root"))?;
    page(&mut mm, CODE, RX, arch::user_code())?;
    let data = page(&mut mm, DATA, RW, &[])?;
    page(&mut mm, STACK, RW, &[])?;

    // Step 2: 初始化通用寄存器、栈指针、线程指针（TP）、全局指针（GP）以及状态控制寄存器。
    let mut context = UserContext::new(CODE + offset, STACK_TOP);
    for (index, register) in context.gpr.iter_mut().enumerate().skip(1) {
        *register = 0x7100 + slot * 0x100 + index;
    }
    #[cfg(target_arch = "riscv64")]
    {
        context.gpr[2] = STACK_TOP;
        context.fcsr = 1 << 5;
    }
    #[cfg(target_arch = "loongarch64")]
    {
        context.gpr[3] = STACK_TOP;
        context.fcsr = 1 << 8;
        context.fcc = 0x0100010001000100;
    }
    context.gpr[TP] = 0x5100 + slot;
    context.gpr[GP] = 0x6100 + slot;
    context.gpr[A6] = DATA;

    // Step 3: 按槽位特征填充浮点与向量寄存器初值，供浮点运算探针验证上下文保存与恢复。
    let mut scalar: [u64; 32] =
        core::array::from_fn(|index| 0x3ff0000000000000 + (slot * 32 + index) as u64);
    scalar[0] = if slot == 0 {
        0x3ff4000000000000
    } else {
        0x4020000000000000
    };
    scalar[1] = if slot == 0 {
        0x4004000000000000
    } else {
        0x4030000000000000
    };
    #[cfg(target_arch = "riscv64")]
    {
        context.fpr = scalar;
    }
    #[cfg(target_arch = "loongarch64")]
    {
        let lanes = user_fp_lanes();
        context.fpr = core::array::from_fn(|index| FpRegister {
            lanes: core::array::from_fn(|lane| {
                if lane == 0 {
                    scalar[index]
                } else if lane < lanes {
                    0x7180 + (slot * 128 + index * 4 + lane) as u64
                } else {
                    0
                }
            }),
        });
        for lane in 0..4 {
            let value = 0x8190 + (slot * 4 + lane) as u64;
            let progress = mm.copyout(DATA + 96 + lane * 8, &value.to_le_bytes());
            assert_eq!((progress.copied, progress.error), (8, None));
        }
    }
    Ok((mm, context, data))
}

/// 将独占构造的 `Mm` 封装为 `Arc<Mm>`，并与 `UserContext` 绑定创建可执行的 `UserThread`。
pub(super) fn user(mm: Mm, context: UserContext) -> Result<Arc<UserThread>, TestError> {
    let mm = Arc::try_new(mm).map_err(|_| TestError("NoMemory: user probe MM owner"))?;
    UserThread::try_new(mm, context).map_err(|_| TestError("NoMemory: user probe context"))
}

/// 循环运行用户线程直到产生非硬件中断的同步陷入事件（如系统调用、缺页或非法指令），若收到外部停止信号则提前中止。
///
/// 当用户态因外部时钟或设备中断返回 `UserTrap::Interrupt` 时，本函数在内核态执行一次抢占检查（`cond_resched`）后继续重入用户态。
pub(super) fn event(
    user: &UserThread,
    token: &ChildToken,
    stop: &AtomicBool,
) -> Result<UserTrap, TestError> {
    loop {
        if stop.load(Ordering::Acquire) || token.stop_requested() {
            return Err(TestError("user probe was stopped"));
        }
        let trap = user.run_user();
        if trap != UserTrap::Interrupt {
            return Ok(trap);
        }
        sched::cond_resched();
    }
}

/// 从 `DATA` 页的内核直映地址读取探针指令写出的前 8 个 64 位字（共 64 字节）。
///
/// # Safety
/// 1. `direct` 必须是由当前测试中仍存活的 `Mm` 所持有的 `DATA` 页内核直映地址，且至少包含 64 字节已初始化内存。
/// 2. 调用时关联的所有用户线程必须已返回内核态且不再对该数据页发起并发写入。
pub(super) unsafe fn probe_words(direct: usize) -> [u64; 8] {
    let mut words = [0; 8];
    // SAFETY:
    // 1. 调用方保证 `direct` 指向的物理页在 `Mm` 中保持存活、按页对齐（自然满足 `u64` 对齐）且无并发写入。
    // 2. `words` 是当前栈上的独占局部数组，与目标物理页内存完全不重叠。
    unsafe {
        core::ptr::copy_nonoverlapping(direct as *const u64, words.as_mut_ptr(), words.len())
    };
    words
}

/// 校验用户态探针指令写回到 `DATA` 页的前 6 个 64 位字是否与预期的栈顶、TLS、GP、浮点加法结果及 FCSR 状态一致。
pub(super) fn check_probe(words: [u64; 8], slot: usize, fcsr: usize) {
    assert_eq!(words[0], STACK_TOP as u64);
    assert_eq!(words[1], (0x5100 + slot) as u64);
    assert_eq!(words[2], (0x6100 + slot) as u64);
    assert_eq!(
        words[3],
        if slot == 0 {
            0x400e000000000000
        } else {
            0x4038000000000000
        }
    );
    assert_eq!(words[4], fcsr as u64);
    assert_eq!(words[5], (0x5100 + slot) as u64);
}

/// 用户态通用寄存器与浮点/向量寄存器现场的值快照，用于跨核迁移或陷入前后的现场比对。
#[derive(PartialEq, Eq)]
pub(super) struct Registers {
    /// 32 个通用寄存器值快照。
    pub(super) gpr: [usize; 32],
    /// RISC-V 架构下的 32 个 64 位浮点寄存器快照。
    #[cfg(target_arch = "riscv64")]
    pub(super) fpr: [u64; 32],
    /// LoongArch64 架构下的 32 个浮点/向量寄存器快照。
    #[cfg(target_arch = "loongarch64")]
    pub(super) fpr: [FpRegister; 32],
    /// 浮点控制与状态寄存器（FCSR）快照。
    fcsr: usize,
    /// LoongArch64 架构下的浮点条件标志寄存器（FCC）快照。
    #[cfg(target_arch = "loongarch64")]
    fcc: usize,
}

impl Registers {
    /// 借用用户线程上下文并拷贝一份完整的寄存器值快照，随即释放上下文可变借用。
    pub(super) fn read(user: &UserThread) -> Self {
        let context = user.context();
        Self {
            gpr: context.gpr,
            fpr: context.fpr,
            fcsr: context.fcsr,
            #[cfg(target_arch = "loongarch64")]
            fcc: context.fcc,
        }
    }
}

/// 执行一次受中断打断的用户态运行，并验证从用户态中断返回后当前 CPU 身份、`CpuLocal` 指针、任务 ID 以及内核栈帧内容完好无损。
pub(super) fn interrupted(user: &UserThread, expected_cpu: usize) {
    let local = [0x8165usize; 16];
    let stack = black_box(&local) as *const _ as usize;
    let id = task::current_task().id();
    assert_eq!(user.run_user(), UserTrap::Interrupt);
    assert_eq!(cpu::current().id, expected_cpu);
    assert_eq!(arch::cpu_local(), cpu::get(expected_cpu) as *const _);
    assert_eq!(task::current_task().id(), id);
    assert_eq!(black_box(&local) as *const _ as usize, stack);
    assert!(black_box(&local).iter().all(|word| *word == 0x8165));
}
