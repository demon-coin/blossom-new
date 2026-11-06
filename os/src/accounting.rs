//! 每 CPU 执行时间与活动片段统计（CPU Accounting）。
//!
//! ## 模块职责与架构定位
//! 本模块负责在每个 CPU 本地精确统计各类执行上下文（调度器、内核/用户任务、空闲等待、硬中断）消耗的单调时钟时间（纳秒），并同步累加到任务及用户线程的时间账本中。
//! - **上游调用者**：`crate::sched`（调度器启动、任务切入/切出、进入/退出 Idle）、`crate::arch` 与 `crate::task`（用户态 Trap 进出、进程退出解绑、硬中断进出、时间查询）。
//! - **下游依赖**：`crate::cpu`（存放每 CPU 的 `CpuAccount` 实例）、`crate::time`（获取单调递增的纳秒时间戳 `time::now()`）、`crate::task`（更新 `Task` 和 `UserThread` 的累计执行时间）。
//!
//! ## 核心设计与模式状态机
//! 每个 CPU 拥有独立的 `CpuAccount`，包含一个仅在本 CPU 关中断（IRQ-off）期间访问的当前活动片段 `Segment`，以及一组可供跨核无锁读取的原子累计计数器（`AtomicU64`）。
//! 在每次执行模式切换边界，当前 CPU 读取单调时钟，将 `[baseline_ns, now_ns]` 的时间增量结算到旧模式对应的原子计数器中，并推进 `baseline_ns = now_ns`。
//!
//! ```text
//!                        start_scheduler()
//!   +---------+ ---------------------------------> +-----------+
//!   | Dormant |                                    | Scheduler |
//!   +---------+                                    +-----------+
//!                                                   /    ^   |  ^
//!                   task_enter()                   /     |   |  | idle_leave()
//!            +------------------------------------+      |   |  +--------------+
//!            |                                           |   | idle_enter()    |
//!            v              task_leave()                 |   v                 |
//!   +-----------------+ ---------------------------+     | +------+            |
//!   |   Task(*Task)   |                                  | | Idle | -----------+
//!   | (in_user: bool) | <--> user_enter() / user_leave() | +------+
//!   +-----------------+                                  |
//!            |                                           |
//!            +-------> [ 任意非 Dormant 模式 ] <----------+
//!                            |          ^
//!                irq_enter() |          | irq_exit()
//!                (保存旧模式) v          | (恢复 interrupted)
//!                         +----------------+
//!                         |      Irq       |
//!                         +----------------+
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - **无锁单写者契约**：本模块不使用自旋锁。`Segment` 放在 `UnsafeCell` 中，**仅允许所属 CPU 在本地中断关闭（IRQ-off）的短暂临界区内借用和修改**；在借用期间严禁开启中断或触发上下文切换。
//! - **跨核只读快照**：其他 CPU 只能通过 `snapshot()` 以 `Ordering::Relaxed` 读取已结算的 `AtomicU64` 总账，绝不触碰目标 CPU 正在运行的 `Segment`。

use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::cpu;
use crate::sync::IrqSaveGuard;
use crate::task::{Task, UserThread};
use crate::time::{self, Duration};

/// CPU 当前正在计时的执行模式。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// CPU 尚未进入调度器主循环前的初始休眠状态（引导早期），此时不统计执行时间。
    Dormant,
    /// CPU 正在静态调度栈上执行调度器选核、上下文切换准备或收尾逻辑。
    Scheduler,
    /// CPU 正在执行指定的普通任务（携带当前运行 `Task` 的裸指针）。
    Task(*const Task),
    /// CPU 已确认无就绪任务并进入底层硬件低功耗等待（WFI / IDLE）状态。
    Idle,
    /// CPU 正在处理最外层硬件中断（嵌套硬中断会被合并在同一个 `Irq` 片段内统计）。
    Irq,
}

/// 单个 CPU 当前正在运行的计时片段私有状态。
///
/// 仅由所属 CPU 在关中断（IRQ-off）状态下通过 `with_current` 访问。
struct Segment {
    /// 当前片段所处的执行模式。
    mode: Mode,
    /// 当 `mode` 为 `Mode::Irq` 时，保存被最外层硬中断打断前的执行模式；非中断模式下为 `None`。
    interrupted: Option<Mode>,
    /// 当前计时片段的起始单调时钟时间戳（单位：纳秒）。
    baseline_ns: u64,
    /// 若当前运行的 `Task` 绑定了活跃的用户线程，则缓存该 `UserThread` 的裸指针，否则为空指针。
    user: *const UserThread,
    /// 标记当前 `Mode::Task` 片段是否正处于用户态指令执行区间（`user_enter` 与 `user_leave` 之间）。
    in_user: bool,
}

/// 单个 CPU 的时间统计器，内嵌于每 CPU 结构 `CpuLocal` 中。
pub(crate) struct CpuAccount {
    /// 当前活动计时片段，仅由本 CPU 在关中断时独占修改。
    segment: UnsafeCell<Segment>,
    /// 本 CPU 已结算的普通任务（含内核态与用户态）累计执行时间（单位：纳秒）。
    task_ns: AtomicU64,
    /// 本 CPU 已结算的硬件中断处理累计时间（单位：纳秒）。
    irq_ns: AtomicU64,
    /// 本 CPU 已结算的调度器上下文切换与选核累计时间（单位：纳秒）。
    scheduler_ns: AtomicU64,
    /// 本 CPU 已结算的空闲等待（Idle）累计时间（单位：纳秒）。
    idle_ns: AtomicU64,
    /// 本 CPU 进入 Idle 等待协议的累计次数。
    idle_entries: AtomicU64,
    /// 测试模式下记录本 CPU 在关中断状态下通过最终无工作检查、准备进入硬件休眠的累计次数。
    #[cfg(feature = "ktest")]
    idle_checks: AtomicU64,
}

// SAFETY:
// 1. `segment` 仅由所属 CPU 在本地中断关闭（`!arch::irq_enabled()`）的临界区内通过 `with_current` 短暂可变借用，期间不开启中断也不发生调度切换，不存在并发或中断重入冲突；
// 2. 跨 CPU 调用（如 `snapshot`）仅通过原子操作读取 `AtomicU64` 累计字段，不访问 `segment`。
unsafe impl Sync for CpuAccount {}

/// 单个 CPU 在某一时刻已结算的各项执行时间与计数快照。
#[derive(Clone, Copy, Debug)]
pub struct CpuSnapshot {
    #[expect(
        dead_code,
        reason = "四账快照保留各 CPU 已结算的 Task 时间；当前无直接字段读取，远端观察仍不得重读私有片段"
    )]
    /// 已结算的任务执行总时间（单位：纳秒）。
    pub task_ns: u64,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行时间测试比较各 CPU 的硬 IRQ 总账，普通镜像不读取此快照字段"
        )
    )]
    /// 已结算的硬件中断处理总时间（单位：纳秒）。
    pub irq_ns: u64,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行时间测试比较调度片段总账，普通镜像不读取此快照字段"
        )
    )]
    /// 已结算的调度器运行总时间（单位：纳秒）。
    pub scheduler_ns: u64,
    /// 已结算的空闲（Idle）总时间（单位：纳秒）。
    pub idle_ns: u64,
    #[expect(
        dead_code,
        reason = "快照记录 idle 协议入口次数；当前无直接字段读取，此值不等同于硬件等待驻留次数"
    )]
    /// 进入 Idle 等待协议的累计次数。
    pub idle_entries: u64,
    #[cfg(feature = "ktest")]
    #[expect(
        dead_code,
        reason = "ktest 快照保留 IRQ-off 最终检查次数；当前无直接字段读取，不改变调度等待窗口的原子观察"
    )]
    /// 测试模式下记录的关中断最终 Idle 检查通过次数。
    pub idle_checks: u64,
}

impl CpuAccount {
    /// 创建初始处于 `Mode::Dormant` 休眠状态、各项累计计数为零的每 CPU 统计器。
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

    /// 使用给定的单调时钟读数 `now_ns`（纳秒）结算当前活动片段 `segment`，并将时间增量累加到对应账本。
    ///
    /// 调用方必须保证在所属 CPU 关中断状态下调用。由于所属 CPU 是各累计计数器的唯一写者，无需使用 CAS 循环。
    fn settle(&self, segment: &mut Segment, now_ns: u64) {
        let delta = now_ns
            .checked_sub(segment.baseline_ns)
            .expect("CPU accounting time moved backwards");
        let total = match segment.mode {
            Mode::Dormant => panic!("settling inactive CPU accounting"),
            Mode::Scheduler => &self.scheduler_ns,
            Mode::Task(task) => {
                assert!(!task.is_null(), "task accounting has no owner");
                // SAFETY:
                // 1. `Mode::Task(task)` 仅在本 CPU 获得该任务的独占 `Running` 执行权后由 `task_enter` 设置，并在切出任务前由 `task_leave` 清除；
                // 2. 当前处于关中断状态，不可能发生抢占或任务退出，调度器的活跃任务链表（`live`）持有该 `Task` 的强引用，保证 `task` 指针有效且由本 CPU 独占更新其执行时间。
                unsafe { (*task).account_execution(delta) };
                if !segment.user.is_null() {
                    // SAFETY:
                    // 1. `segment.user` 来自当前 `Task` 绑定的 `Arc<UserThread>`，任务在退出并调用 `detach_user` 或回到调度栈前不会释放该绑定；
                    // 2. 当前关中断且由本 CPU 唯一执行该任务，`segment.user` 指向的 `UserThread` 实例在本次结算期间始终存活有效。
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

/// 将增量 `delta`（纳秒或次数）累加到由当前 CPU 独占写入的原子总账 `total` 中，若发生溢出则触发 panic。
fn add_total(total: &AtomicU64, delta: u64) {
    let value = total.load(Ordering::Relaxed);
    total.store(
        value.checked_add(delta).expect("CPU accounting overflow"),
        Ordering::Relaxed,
    );
}

/// 在本地中断关闭的前提下，借用当前 CPU 的 `CpuAccount` 与可变 `Segment` 执行闭包 `action`。
///
/// 调用时若本地中断处于开启状态则会直接触发断言 panic。
fn with_current<T>(action: impl FnOnce(&CpuAccount, &mut Segment) -> T) -> T {
    assert!(
        !crate::arch::irq_enabled(),
        "CPU accounting requires IRQ off"
    );

    let account = &cpu::current().accounting;
    // SAFETY:
    // 1. 上方 `assert!(!crate::arch::irq_enabled())` 已确认本地中断关闭，且所有传入 `with_current` 的内部闭包均只做同步内存更新、不开启中断也不触发上下文切换；
    // 2. `account.segment` 属于当前物理 CPU 私有，其他 CPU 的 `snapshot()` 不访问 `segment`，因此在此作用域内构造 `&mut Segment` 满足独占别名约束。
    action(account, unsafe { &mut *account.segment.get() })
}

/// 在当前 CPU 进入调度器主循环（`sched::run`）时启动时间统计，将模式从 `Mode::Dormant` 切换为 `Mode::Scheduler`。
///
/// 必须在本地中断关闭状态下调用；引导阶段在此之前的硬中断由于处于 `Dormant` 模式会被忽略。
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

/// 在非中断上下文的模式边界结算旧模式 `expected`，并将当前 CPU 切换为新模式 `next`。
///
/// 必须在本地中断关闭状态下调用，且当前不能处于硬中断嵌套（`interrupted` 必须为 `None`）。
fn transition(expected: Mode, next: Mode) {
    with_current(|account, segment| {
        assert!(segment.mode == expected, "CPU accounting mode mismatch");
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        segment.mode = next;
    });
}

/// 在任务首次启动或上下文恢复后、开启中断之前调用，将当前 CPU 的统计模式从 `Mode::Scheduler` 切换为 `Mode::Task`。
///
/// 若当前任务绑定了处于活跃统计状态的用户线程（`UserThread`），同时在 `Segment` 中缓存其指针以便后续入账。
pub(crate) fn task_enter() {
    assert!(
        !crate::arch::irq_enabled(),
        "task accounting entry requires IRQ off"
    );

    // SAFETY:
    // 1. 上方已断言本地中断关闭，且调度器在切入本任务前已将 `current_task` 设为当前独占 `Running` 的任务；
    // 2. 仅当前 CPU 会读写自身的 `current_task` 槽位。
    let task = unsafe { *cpu::current().current_task.get() };
    assert!(!task.is_null(), "task accounting entry has no task");
    // SAFETY:
    // 1. `task` 由调度器活跃任务链表（`live`）持有强引用并处于本 CPU 的 `Running` 状态，在切出回到调度栈前不会被销毁；
    // 2. `user_thread()` 内部通过自旋锁克隆 `Option<Arc<UserThread>>`，并在下方过滤后将裸指针存入 `segment.user`（其生命周期由 `task.user` 中的 `Arc` 持续保活直到 `task_leave` 或 `detach_user`）。
    let user = unsafe { (*task).user_thread() };
    let user = user.filter(|user| user.user_time().is_none_or(|time| time.is_active()));

    with_current(|_, segment| {
        assert!(segment.user.is_null());
        assert!(!segment.in_user);
        segment.user = user.as_ref().map_or(core::ptr::null(), Arc::as_ptr);
    });
    transition(Mode::Scheduler, Mode::Task(task));
}

/// 在当前任务已提交切换意图（`SwitchingOut`）并释放调度锁后、执行汇编上下文切换前调用。
///
/// 必须在关中断状态下调用：先结算当前 `Mode::Task` 片段的执行时间，再将 CPU 统计模式切回 `Mode::Scheduler`，使后续寄存器保存与 `finish_current` 计入调度器开销。
pub(crate) fn task_leave() {
    assert!(
        !crate::arch::irq_enabled(),
        "task accounting exit requires IRQ off"
    );
    // SAFETY:
    // 1. 上方已断言本地中断关闭，当前任务尚未切换回调度栈，`current_task` 仅由本 CPU 访问；
    // 2. 该任务在调度器 `finish` 处理完毕前始终由活跃任务链表保活。
    let task = unsafe { *cpu::current().current_task.get() };
    assert!(!task.is_null(), "task accounting exit has no task");

    transition(Mode::Task(task), Mode::Scheduler);
    with_current(|_, segment| {
        assert!(!segment.in_user, "task switched out in user accounting");
        segment.user = core::ptr::null();
    });
}

/// 在当前用户任务即将从内核态返回用户态执行用户指令前调用（关中断状态）。
///
/// 先将此前在内核态（如系统调用或异常入口）消耗的时间结算为内核态任务时间，随后将 `in_user` 标记设为 `true`，开始统计用户态执行时间。
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

/// 在用户态触发 Trap（系统调用、异常或中断）刚返回内核栈时调用（关中断状态）。
///
/// 将刚刚结束的用户态运行区间结算为用户态执行时间（期间若发生硬中断，其耗时已由 `irq_enter`/`irq_exit` 独立扣除），并将 `in_user` 恢复为 `false`。
pub(crate) fn user_leave() {
    with_current(|account, segment| {
        assert!(matches!(segment.mode, Mode::Task(_)));
        assert!(segment.in_user);
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        segment.in_user = false;
    });
}

/// 在正在运行的普通任务上下文中，主动关中断并结算当前尚未结束的内核态执行片段。
///
/// 用于在读取进程或线程的用户/内核时间统计（如 `clock_gettime`、`times`、`getrusage`）前刷新最新账目。
pub(crate) fn settle_current() {
    let _irq = IrqSaveGuard::new();
    with_current(|account, segment| {
        assert!(matches!(segment.mode, Mode::Task(_)));
        assert!(!segment.in_user);
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
    });
}

/// 在用户线程退出并释放进程资源前调用，结算最后一段任务时间并解除当前 `Segment` 对 `UserThread` 的关联。
///
/// 同时调用 `time.retire()` 冻结该用户线程的统计账本，使后续内核任务退出收尾工作不再计入用户进程时间。
pub(crate) fn detach_user() {
    let _irq = IrqSaveGuard::new();
    with_current(|account, segment| {
        assert!(matches!(segment.mode, Mode::Task(_)));
        assert!(!segment.in_user);
        assert!(segment.interrupted.is_none());
        account.settle(segment, time::now().as_nanos());
        if !segment.user.is_null() {
            // SAFETY:
            // 1. `with_current` 保证了本地中断关闭，且当前任务仍处于 `Mode::Task` 并持有 `task.user` 中的 `Arc<UserThread>` 强引用；
            // 2. 此处仅通过有效指针读取 `user_time()` 并标记 `retire()`，随后立即将 `segment.user` 置空，不会产生悬垂指针访问。
            if let Some(time) = unsafe { (*segment.user).user_time() } {
                time.retire();
            }
        }
        segment.user = core::ptr::null();
    });
}

/// 在调度器通过最终无工作检查、即将进入底层硬件休眠前调用（关中断状态），将模式从 `Mode::Scheduler` 切换为 `Mode::Idle` 并递增 `idle_entries`。
pub(crate) fn idle_enter() {
    transition(Mode::Scheduler, Mode::Idle);
    add_total(&cpu::current().accounting.idle_entries, 1);
}

/// 在底层硬件休眠指令返回后调用（关中断状态），结算 `Mode::Idle` 休眠时长并将模式切回 `Mode::Scheduler`。
pub(crate) fn idle_leave() {
    transition(Mode::Idle, Mode::Scheduler);
}

/// 在最外层硬件中断入口处调用（关中断状态），结算被打断的执行模式并将其保存在 `interrupted` 中，随后切换到 `Mode::Irq`。
///
/// 若 CPU 尚处于引导早期的 `Mode::Dormant`，则直接忽略；嵌套硬中断由中断深度计数拦截，不会重复调用本函数。
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

/// 在最外层硬件中断即将退出前调用（关中断状态），结算本次硬中断耗时并将统计模式恢复为被打断前的 `interrupted` 模式。
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

/// 在普通任务上下文中临时关闭本地中断，刷新当前执行片段后返回当前任务累计的总执行时间（含跨核迁移历史）。
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
    // SAFETY:
    // 1. `_irq` 守卫已关闭本地中断，且上方已断言不处于硬中断或软中断下半部上下文，本函数不会触发任务切换；
    // 2. `current_task` 仅由当前 CPU 修改，且当前运行任务由调度器活跃任务链表持有强引用保活。
    let task = unsafe { *local.current_task.get() };
    assert!(!task.is_null(), "execution time outside a task");
    with_current(|account, segment| {
        assert!(
            segment.mode == Mode::Task(task),
            "execution time outside task mode"
        );
        account.settle(segment, time::now().as_nanos());
    });
    // SAFETY:
    // 同上，`task` 在整个关中断作用域内指向当前 CPU 独占运行的 `Task` 实例，读取其原子字段 `execution_ns` 安全有效。
    unsafe { (*task).execution_time() }
}

/// 无锁读取指定 CPU（`owner`）目前已结算的各项时间与计数快照。
///
/// 各字段通过独立的 `Ordering::Relaxed` 原子加载读取，适用于监控和测试统计，不阻塞目标 CPU 的执行。
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

/// 测试辅助函数：在关中断状态下记录当前 CPU 已通过最终 Idle 唤醒条件复查，即将进入底层休眠。
#[cfg(feature = "ktest")]
pub(crate) fn test_idle_checked() {
    assert!(!crate::arch::irq_enabled());
    add_total(&cpu::current().accounting.idle_checks, 1);
}
