//! 每 CPU 的执行片段只由所属 CPU 在 IRQ 关闭时结算，跨核观察只读原子总账。

use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::cpu;
use crate::sync::IrqSaveGuard;
use crate::task::{Task, UserThread};
use crate::time::{self, Duration};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Dormant,
    Scheduler,
    Task(*const Task),
    Idle,
    Irq,
}

struct Segment {
    mode: Mode,
    interrupted: Option<Mode>,
    baseline_ns: u64,
    user: *const UserThread,
    in_user: bool,
}

pub(crate) struct CpuAccount {
    segment: UnsafeCell<Segment>,
    task_ns: AtomicU64,
    irq_ns: AtomicU64,
    scheduler_ns: AtomicU64,
    idle_ns: AtomicU64,
    idle_entries: AtomicU64,
    #[cfg(feature = "ktest")]
    idle_checks: AtomicU64,
}

// SAFETY: segment 只在所属 CPU 的 IRQ-off 段内短借用；跨 CPU 只访问原子总账。
unsafe impl Sync for CpuAccount {}

#[derive(Clone, Copy, Debug)]
pub struct CpuSnapshot {
    #[expect(
        dead_code,
        reason = "四账快照保留各 CPU 已结算的 Task 时间；当前无直接字段读取，远端观察仍不得重读私有片段"
    )]
    pub task_ns: u64,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行时间测试比较各 CPU 的硬 IRQ 总账，普通镜像不读取此快照字段"
        )
    )]
    pub irq_ns: u64,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行时间测试比较调度片段总账，普通镜像不读取此快照字段"
        )
    )]
    pub scheduler_ns: u64,
    pub idle_ns: u64,
    #[expect(
        dead_code,
        reason = "快照记录 idle 协议入口次数；当前无直接字段读取，此值不等同于硬件等待驻留次数"
    )]
    pub idle_entries: u64,
    #[cfg(feature = "ktest")]
    #[expect(
        dead_code,
        reason = "ktest 快照保留 IRQ-off 最终检查次数；当前无直接字段读取，不改变调度等待窗口的原子观察"
    )]
    pub idle_checks: u64,
}

impl CpuAccount {
    /// 在永久 CPU 对象内准备尚未开始计账的私有片段与原子观察值。
    pub(crate) const fn new() -> Self {
        Self {
            segment: UnsafeCell::new(Segment {
                mode: Mode::Dormant,
                interrupted: None,
                baseline_ns: 0,
                user: core::ptr::null(),
                in_user: false,
            }),
            task_ns: AtomicU64::new(0),
            irq_ns: AtomicU64::new(0),
            scheduler_ns: AtomicU64::new(0),
            idle_ns: AtomicU64::new(0),
            idle_entries: AtomicU64::new(0),
            #[cfg(feature = "ktest")]
            idle_checks: AtomicU64::new(0),
        }
    }

    /// 以同一单调时间结算当前片段；唯一写者不需要原子读改写重试。
    fn settle(&self, segment: &mut Segment, now_ns: u64) {
        let delta = now_ns
            .checked_sub(segment.baseline_ns)
            .expect("CPU accounting time moved backwards");
        let total = match segment.mode {
            Mode::Dormant => panic!("settling inactive CPU accounting"),
            Mode::Scheduler => &self.scheduler_ns,
            Mode::Task(task) => {
                assert!(!task.is_null(), "task accounting has no owner");
                // SAFETY: Task 模式只在本核取得执行权后建立；切出先结算再发表 Ready，
                // IRQ 保存模式期间不能调度，活登记始终保持指针与独占执行权。
                unsafe { (*task).account_execution(delta) };
                if !segment.user.is_null() {
                    // SAFETY: Task 的用户绑定只在离栈后解除；片段结算在唯一执行者
                    // 切出以前完成，IRQ 保存期间不调度，用户对象地址保持有效。
                    unsafe { (*segment.user).account_execution(segment.in_user, delta) };
                }
                &self.task_ns
            }
            Mode::Idle => &self.idle_ns,
            Mode::Irq => &self.irq_ns,
        };
        add_total(total, delta);
        segment.baseline_ns = now_ns;
    }
}

/// 所属 CPU 是总账的唯一写者；溢出直接暴露，不让累计值回绕。
fn add_total(total: &AtomicU64, delta: u64) {
    let value = total.load(Ordering::Relaxed);
    total.store(
        value.checked_add(delta).expect("CPU accounting overflow"),
        Ordering::Relaxed,
    );
}

/// 在 IRQ-off 段短借用本核私有状态，借用不越过切换或跨核观察。
fn with_current<T>(action: impl FnOnce(&CpuAccount, &mut Segment) -> T) -> T {
    assert!(
        !crate::arch::irq_enabled(),
        "CPU accounting requires IRQ off"
    );
    let account = &cpu::current().accounting;
    // SAFETY: IRQ 已关闭；所有本核调用只在这里短借用，闭包不会恢复 IRQ 或切换，
    // 远端 snapshot 不访问此 UnsafeCell，因此没有并行或重入可变访问。
    action(account, unsafe { &mut *account.segment.get() })
}

/// 本 CPU 首次从永久调度栈开始计账；引导阶段 IRQ 不产生伪执行账。
pub(crate) fn start_scheduler() {
    with_current(|_, segment| {
        assert!(
            segment.mode == Mode::Dormant,
            "CPU accounting already started"
        );
        assert!(segment.interrupted.is_none());
        segment.baseline_ns = time::now().as_nanos();
        segment.mode = Mode::Scheduler;
    });
}

/// 在片段边界结算旧模式并开始新模式；普通转换不能发生在 IRQ 片段内。
fn transition(expected: Mode, next: Mode) {
    with_current(|account, segment| {
        assert!(segment.mode == expected, "CPU accounting mode mismatch");
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        segment.mode = next;
    });
}

/// Task 首次入口或恢复后、开 IRQ 前开始本 CPU 的执行片段。
pub(crate) fn task_enter() {
    assert!(
        !crate::arch::irq_enabled(),
        "task accounting entry requires IRQ off"
    );
    // SAFETY: 调度器已发表本核 current，IRQ 关闭，活登记保护唯一执行者。
    let task = unsafe { *cpu::current().current_task.get() };
    assert!(!task.is_null(), "task accounting entry has no task");
    // SAFETY: current 活登记保活 Task；绑定只在离栈后解除，快照退还后仍由 Task 保活。
    let user = unsafe { (*task).user_thread() };
    let user = user.filter(|user| user.user_time().is_none_or(|time| time.is_active()));
    with_current(|_, segment| {
        assert!(segment.user.is_null());
        assert!(!segment.in_user);
        segment.user = user.as_ref().map_or(core::ptr::null(), Arc::as_ptr);
    });
    transition(Mode::Scheduler, Mode::Task(task));
}

/// SwitchingOut 已提交并放锁后结算 Task，后续 ctx 保存与 finish 归调度账。
pub(crate) fn task_leave() {
    assert!(
        !crate::arch::irq_enabled(),
        "task accounting exit requires IRQ off"
    );
    // SAFETY: 当前 Task 尚未离栈，IRQ 关闭，唯一执行权与活登记保持到 finish。
    let task = unsafe { *cpu::current().current_task.get() };
    assert!(!task.is_null(), "task accounting exit has no task");
    transition(Mode::Task(task), Mode::Scheduler);
    with_current(|_, segment| {
        assert!(!segment.in_user, "task switched out in user accounting");
        segment.user = core::ptr::null();
    });
}

/// 在真实用户指令入口前结算内核片段；当前绑定和 IRQ-off 执行权必须保持不变。
pub(crate) fn user_enter(user: &UserThread) {
    with_current(|account, segment| {
        assert!(matches!(segment.mode, Mode::Task(_)));
        assert!(core::ptr::eq(segment.user, user));
        assert!(!segment.in_user);
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        segment.in_user = true;
    });
}

/// 用户 trap 返回原内核栈后结算用户片段，硬 IRQ 已由独立片段剔除。
pub(crate) fn user_leave() {
    with_current(|account, segment| {
        assert!(matches!(segment.mode, Mode::Task(_)));
        assert!(segment.in_user);
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        segment.in_user = false;
    });
}

/// 在普通用户任务的合法锁外观察点补结算正在运行的内核片段。
pub(crate) fn settle_current() {
    let _irq = IrqSaveGuard::new();
    with_current(|account, segment| {
        assert!(matches!(segment.mode, Mode::Task(_)));
        assert!(!segment.in_user);
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
    });
}

/// 用户最后退出在释放进程资源以前结清并撤掉片段关联，后续内核离栈工作不进 Linux 账。
pub(crate) fn detach_user() {
    let _irq = IrqSaveGuard::new();
    with_current(|account, segment| {
        assert!(matches!(segment.mode, Mode::Task(_)));
        assert!(!segment.in_user);
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        if !segment.user.is_null() {
            // SAFETY: 当前片段仍由 Task 活绑定保活，刚结算完成且 IRQ-off；
            // 只访问固定的私有时间对象，不取进程或信号锁，随后立即撤掉裸关联。
            if let Some(time) = unsafe { (*segment.user).user_time() } {
                time.retire();
            }
        }
        segment.user = core::ptr::null();
    });
}
/// 最终无工作检查通过以后开始 idle 片段；次数表示协议入口，架构仍会检查新提示。
pub(crate) fn idle_enter() {
    transition(Mode::Scheduler, Mode::Idle);
    add_total(&cpu::current().accounting.idle_entries, 1);
}

/// 硬件等待返回且 IRQ 已关闭后恢复调度片段。
pub(crate) fn idle_leave() {
    transition(Mode::Idle, Mode::Scheduler);
}

/// 外层硬 IRQ 入口保存被打断模式，嵌套 IRQ 由深度 guard 合并为一段。
pub(crate) fn irq_enter() {
    with_current(|account, segment| {
        if segment.mode == Mode::Dormant {
            return;
        }
        assert!(
            segment.mode != Mode::Irq,
            "nested IRQ entered outer accounting"
        );
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        segment.interrupted = Some(segment.mode);
        segment.mode = Mode::Irq;
    });
}

/// 最外层硬 IRQ 返回前结算 IRQ，并以当前时间重建被打断片段的基准。
pub(crate) fn irq_exit() {
    with_current(|account, segment| {
        if segment.mode == Mode::Dormant {
            return;
        }
        assert!(segment.mode == Mode::Irq, "IRQ accounting exit outside IRQ");
        account.settle(segment, time::now().as_nanos());
        segment.mode = segment
            .interrupted
            .take()
            .expect("IRQ accounting lost interrupted mode");
    });
}

/// 普通 Task 短暂关闭本核 IRQ，结算当前片段后读取迁核累计总账，不取调度锁。
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "执行时间测试读取并比较阻塞与迁核前后的真实 Task 总账"
    )
)]
pub fn current_execution_time() -> Duration {
    let _irq = IrqSaveGuard::new();
    let local = cpu::current();
    assert_eq!(local.hardirq_depth.load(Ordering::Relaxed), 0);
    assert_eq!(local.bh_depth.load(Ordering::Relaxed), 0);
    // SAFETY: 本核 IRQ 关闭且不会切换，current 指针只由本核修改，活登记保护当前 Task。
    let task = unsafe { *local.current_task.get() };
    assert!(!task.is_null(), "execution time outside a task");
    with_current(|account, segment| {
        assert!(
            segment.mode == Mode::Task(task),
            "execution time outside task mode"
        );
        account.settle(segment, time::now().as_nanos());
    });
    // SAFETY: 上述 IRQ-off 段内 Task 保持唯一执行权，仅共享读取原子总账。
    unsafe { (*task).execution_time() }
}

/// 只读目标 CPU 已结算总账；各原子字段允许来自不同结算时刻。
pub fn snapshot(owner: usize) -> CpuSnapshot {
    let account = &cpu::get(owner).accounting;
    CpuSnapshot {
        task_ns: account.task_ns.load(Ordering::Relaxed),
        irq_ns: account.irq_ns.load(Ordering::Relaxed),
        scheduler_ns: account.scheduler_ns.load(Ordering::Relaxed),
        idle_ns: account.idle_ns.load(Ordering::Relaxed),
        idle_entries: account.idle_entries.load(Ordering::Relaxed),
        #[cfg(feature = "ktest")]
        idle_checks: account.idle_checks.load(Ordering::Relaxed),
    }
}

/// 记录最终无 ready 与提示的检查已通过，只供真实 idle 握手验收观察。
#[cfg(feature = "ktest")]
pub(crate) fn test_idle_checked() {
    assert!(!crate::arch::irq_enabled());
    add_total(&cpu::current().accounting.idle_checks, 1);
}
