//! 真实用户页、寄存器现场及用户指令进入和返回的受控探针。

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

pub(super) const CODE: usize = 0x10000;
pub(super) const DATA: usize = 0x20000;
const STACK: usize = 0x30000;
pub(super) const STACK_TOP: usize = STACK + mm::PAGE_SIZE;
pub(super) const TARGET: usize = 0x40000;

const RX: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: true,
};
pub(super) const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
pub(super) const RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};
pub(super) const EX: PagePermissions = PagePermissions {
    read: false,
    write: false,
    execute: true,
};
pub(super) const NONE: PagePermissions = PagePermissions {
    read: false,
    write: false,
    execute: false,
};

#[cfg(target_arch = "riscv64")]
const TP: usize = 4;
#[cfg(target_arch = "loongarch64")]
const TP: usize = 2;
#[cfg(target_arch = "riscv64")]
const GP: usize = 3;
#[cfg(target_arch = "loongarch64")]
const GP: usize = 21;
#[cfg(target_arch = "riscv64")]
pub(super) const A0: usize = 10;
#[cfg(target_arch = "loongarch64")]
pub(super) const A0: usize = 4;
#[cfg(target_arch = "riscv64")]
pub(super) const A5: usize = 15;
#[cfg(target_arch = "loongarch64")]
pub(super) const A5: usize = 9;
#[cfg(target_arch = "riscv64")]
const A6: usize = 16;
#[cfg(target_arch = "loongarch64")]
const A6: usize = 10;
#[cfg(target_arch = "riscv64")]
pub(super) const A7: usize = 17;
#[cfg(target_arch = "loongarch64")]
pub(super) const A7: usize = 11;
#[cfg(target_arch = "riscv64")]
pub(super) const T0: usize = 5;
#[cfg(target_arch = "loongarch64")]
pub(super) const T0: usize = 12;

/// 受控访问每次从真实用户指令入口开始，缺页只交给当前 Mm 解决并在同一 PC 重进。
pub(super) fn access(
    user: &UserThread,
    token: &ChildToken,
    stop: &AtomicBool,
    address: usize,
    direction: UserAccess,
) -> Result<(usize, usize), TestError> {
    {
        let mut context = user.context();
        context.pc = CODE + arch::fault_offset(direction);
        context.gpr[A5] = address;
    }
    let mm = user.mm().expect("user access has no MM");
    let mut faults = 0;
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

/// 完整字段由逐页入口复制，失败前缀不得被当作已初始化的用户数值。
pub(super) fn word(mm: &Mm, address: usize) -> u64 {
    mm.read_u64(address).expect("user VM word is inaccessible")
}

/// 在尚未共享的 Mm 中安装完整初始化的真实单页；返回整数只用于同例受约束观察。
pub(super) fn page(
    mm: &mut Mm,
    address: usize,
    permissions: PagePermissions,
    contents: &[u8],
) -> Result<usize, TestError> {
    if mm::PAGE_SIZE < contents.len() {
        return Err(TestError("user probe does not fit a basic page"));
    }
    let mut pages = mm::alloc_pages(0, true).ok_or(TestError("NoMemory: user probe page"))?;
    let direct = pages.as_mut_ptr().cast::<u8>();
    // SAFETY: pages 独占完整清零的真实页，contents 为稳定内核字节且长度已检查；
    // 两块存储不重叠，用户根尚未发表，本次原始初始化在转交拥有前结束。
    unsafe { core::ptr::copy_nonoverlapping(contents.as_ptr(), direct, contents.len()) };
    // SAFETY: 全页已清零并完成内容复制，无排他引用或并发访问；只作为用户代码或数据，
    // 原页唯一归还权移入尚未共享的 Mm，旧裸地址从不用于恢复或释放拥有。
    unsafe { mm.map_owned(address, pages, permissions, permissions) }
        .map_err(|_| TestError("user probe mapping failed"))?;
    Ok(direct as usize)
}

/// 构造同一正式 Mm 和硬件页表，受控代码仅用于验证真实用户特权入口。
pub(super) fn image(offset: usize, slot: usize) -> Result<(Mm, UserContext, usize), TestError> {
    let mut mm = Mm::try_new().map_err(|_| TestError("NoMemory: user probe root"))?;
    page(&mut mm, CODE, RX, arch::user_code())?;
    let data = page(&mut mm, DATA, RW, &[])?;
    page(&mut mm, STACK, RW, &[])?;

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

/// 在完整映像发表前建立用户私有拥有，失败仍按未发表对象正常归还。
pub(super) fn user(mm: Mm, context: UserContext) -> Result<Arc<UserThread>, TestError> {
    let mm = Arc::try_new(mm).map_err(|_| TestError("NoMemory: user probe MM owner"))?;
    UserThread::try_new(mm, context).map_err(|_| TestError("NoMemory: user probe context"))
}

/// 只跳过已完成分派的硬件中断，不推进 PC；停止仅在返回原 Rust 栈后消费。
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

/// 在本例唯一用户执行者已返回内核后读取其实际 store 结果，不产生指向用户页的 Rust 引用。
///
/// # Safety
/// direct 必须仍由本例 Mm 保活且完整初始化，至少可读 64 字节；所有用户执行者已停止写入。
pub(super) unsafe fn probe_words(direct: usize) -> [u64; 8] {
    let mut words = [0; 8];
    // SAFETY: 调用者保活完整初始化的数据页并保证没有并发写，words 是独占内核局部；
    // 两块存储不重叠且按 u64 对齐，访问在 Mm 解除或归页之前结束。
    unsafe {
        core::ptr::copy_nonoverlapping(direct as *const u64, words.as_mut_ptr(), words.len())
    };
    words
}

/// 比较用户真正算出的浮点和栈/TLS/GP存取结果，未执行过的 Rust 初始值不能满足结果。
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

#[derive(PartialEq, Eq)]
pub(super) struct Registers {
    pub(super) gpr: [usize; 32],
    #[cfg(target_arch = "riscv64")]
    pub(super) fpr: [u64; 32],
    #[cfg(target_arch = "loongarch64")]
    pub(super) fpr: [FpRegister; 32],
    fcsr: usize,
    #[cfg(target_arch = "loongarch64")]
    fcc: usize,
}

impl Registers {
    /// 短暂复制用户真实现场后立即结束 RefMut，快照可以跨原 Task 的迁核交接。
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

/// 确认一次完整用户中断已经在本 Task 栈返回，CpuLocal 和原局部帧没有被用户 TP/SP 替换。
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
