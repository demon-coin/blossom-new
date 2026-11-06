//! 任务阻塞等待（Wait）与唤醒通知（Wake）核心原语。
//!
//! # 模块职责与架构定位
//! - **职责**：为内核提供零堆内存分配、代次安全（Generation-Safe）的睡眠等待与跨核异步唤醒机制。
//! - **上游调用者**：等待队列（[`queue::WaitQueue`]）、互斥锁与条件变量、管道/套接字/终端 I/O 阻塞、进程等待（`wait4`）、定时器睡眠及支持信号打断的系统调用。
//! - **下游依赖**：调度器（[`crate::sched`]）负责管理任务控制块中的等待代次与睡眠/就绪状态转换；线程信号模块（[`crate::task::signal::ThreadSignals`]）负责在信号敏感等待期间登记唤醒器并检查待处理信号。
//!
//! # 核心设计与状态流转
//! 1. **代次隔离（`WaitId`）防止虚假跨轮次唤醒**：
//!    每个任务（[`Task`]）内部维护单调递增的等待序号 [`WaitId`]。每次创建 [`Waiter`] 开启新一轮等待时分配新的 [`WaitId`]，而 [`Waker`] 同时携带 `Weak<Task>` 与该轮 [`WaitId`]。当迟到的通知（如已过期的定时器或残留唤醒）调用 [`Waker::notify`] 时，调度器会比对任务的当前活跃 [`WaitId`]；若该轮等待已结束或已进入后续轮次，旧通知会被忽略。
//! 2. **防丢失唤醒（Lost-Wakeup Prevention）与信号协同流程**：
//!
//! ```text
//! 调用方任务 (Waiter)                                    唤醒源 / 信号发送方 (Waker)
//! ──────────────────                                    ─────────────────────────
//! 1. Waiter::new() / new_interruptible() / new_killable()
//!    ├─ sched::begin_wait(task) -> 分配本轮唯一 WaitId(N)
//!    └─ (若为信号等待) 向 ThreadSignals 登记 Waker(N)，
//!       并立即复查一次登记前已到达的 pending 信号
//! 2. 将 Waker(N) 注册到业务等待源 (如 WaitQueue)
//! 3. 检查业务条件是否已满足
//!    ├─ 已满足 -> 直接退出作用域                         ──┐ (若在 park 前到达)
//!    └─ 未满足 -> 调用 park() / park_signal()              ├─ waker.notify()
//!                 ├─ 若 notified 已置位，立即返回 <────────┘    └─ 匹配 WaitId(N)，置位 notified
//!                 └─ 否则切换为阻塞态让出 CPU
//!                    ... 睡眠等待 ...                    ──► waker.notify()
//!                 ◄─ 被唤醒恢复执行 ─────────────────────────    └─ 匹配 WaitId(N)，重新入队就绪
//! 4. Drop Waiter
//!    ├─ (若为信号等待) ThreadSignals::unregister_wait(N)
//!    └─ sched::end_wait(task, N) 清除活跃等待代次
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`Waiter`] 通过 `PhantomData<*mut ()>` 约束为 `!Send` 且 `!Sync`，只能由创建它的任务在普通可调度上下文中使用，禁止转交其他任务；但允许随当前任务在睡眠与唤醒之间被调度器迁移到其他 CPU。
//! - **锁顺序**：业务等待源锁（如 [`queue::WaitQueue`] 自旋锁）与信号锁（`ThreadSignals` 内部锁）均在进入调度器（`sched::begin_wait`、`sched::park`、`sched::notify`、`sched::end_wait`）之前释放或由各自模块独立获取，绝不在持有外部自旋锁时调用 [`Waiter::park`] 或析构 [`Task`]。

pub mod queue;

pub use queue::{WaitNode, WaitQueue};

use alloc::sync::{Arc, Weak};
use core::marker::PhantomData;

use crate::sched;
use crate::task::signal::ThreadSignals;
use crate::task::{self, Task};

/// 标识单次等待轮次的单调递增代次序号。
///
/// 用于将某次具体的阻塞等待与异步触发的 [`Waker`] 绑定，防止上一轮等待残留的通知误唤醒当前轮次。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WaitId(usize);

impl WaitId {
    /// 返回等待代次序号的底层 `usize` 数值（用于调度器比对或诊断事件记录）。
    pub(crate) const fn as_usize(self) -> usize {
        self.0
    }

    /// 返回新任务创建时的初始代次序号（`0` 表示尚未进入过任何等待）。
    pub(crate) const fn initial() -> Self {
        Self(0)
    }

    /// 递增并返回下一轮等待的代次序号；若序号空间耗尽则直接 panic，不回绕复用旧代次。
    pub(crate) fn next(self) -> Self {
        Self(self.0.checked_add(1).expect("wait ID space exhausted"))
    }
}

/// 用户态线程在等待期间对信号的响应模式。
///
/// 内核内部同步等待默认不响应信号；仅当系统调用需要支持信号打断时才指定此模式。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitMode {
    /// 可中断等待：任意未屏蔽的待处理信号（含普通可捕获信号与致命信号）均可唤醒并打断等待。
    Interruptible,
    /// 可杀死等待：仅响应导致进程终止的致命信号，忽略普通信号。
    Killable,
}

/// 信号敏感等待（[`Waiter::park_signal`]）返回时的唤醒结果分类。
///
/// [`WaitResult::Woken`] 仅表示任务收到唤醒通知得以继续执行，调用方仍需在循环中重新检查业务条件是否成立。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitResult {
    /// 任务因正常业务通知（或睡眠前已到达的唤醒标志）恢复执行，且当前无待处理的打断信号。
    Woken,
    /// 任务在等待期间检测到符合 [`WaitMode`] 的待处理信号，由调用方决定返回 `EINTR`/重启或退出。
    Interrupted,
}

/// 当前任务单轮阻塞等待的生命周期守卫（RAII Guard）。
///
/// 在创建时向调度器注册本轮等待代次（及可选的信号唤醒源），并在析构时自动注销信号源并关闭该轮等待。
/// 该类型不可跨任务发送或共享（`!Send`、`!Sync`），但可随所属任务在 CPU 间迁移。
pub struct Waiter {
    /// 正在执行本轮等待的当前任务控制块强引用。
    task: Arc<Task>,
    /// 本轮等待在 `task` 上分配的唯一代次序号。
    id: WaitId,
    /// 若为信号敏感等待，保存关联的线程信号控制器与信号响应模式；普通等待为 `None`。
    signal: Option<(Arc<ThreadSignals>, WaitMode)>,
    /// 将 `Waiter` 约束为 `!Send` 与 `!Sync` 的零大小标记，防止跨任务转交等待所有权。
    _task_local: PhantomData<*mut ()>,
}

impl Waiter {
    /// 在当前普通可调度任务上下文中开启一轮不可中断的等待。
    ///
    /// 调用方必须处于允许上下文切换的普通任务环境（非中断上下文、未持有禁止调度的自旋锁）。
    pub fn new() -> Self {
        sched::assert_switchable();
        let task = task::current_task();
        let id = sched::begin_wait(&task);
        Self {
            task,
            id,
            signal: None,
            _task_local: PhantomData,
        }
    }

    /// 为当前用户线程开启一轮可被任意未屏蔽信号打断的等待（[`WaitMode::Interruptible`]）。
    ///
    /// 调用者必须是已绑定用户线程（`UserThread`）及信号控制器的任务，否则直接 panic。
    pub fn new_interruptible() -> Self {
        Self::new_signal(WaitMode::Interruptible)
    }

    /// 为当前用户线程开启一轮仅响应致命退出信号的等待（[`WaitMode::Killable`]）。
    ///
    /// 调用者必须是已绑定用户线程（`UserThread`）及信号控制器的任务，否则直接 panic。
    pub fn new_killable() -> Self {
        Self::new_signal(WaitMode::Killable)
    }

    /// 开启一轮信号敏感等待：先向调度器建立活跃等待代次，再向信号模块登记 `Waker` 并复查已有信号。
    fn new_signal(mode: WaitMode) -> Self {
        // Step 1: 开启基础调度等待代次，并获取当前任务关联的用户态信号控制器。
        let mut waiter = Self::new();
        let signals = waiter
            .task
            .user_thread()
            .and_then(|user| user.signals())
            .expect("signal wait requires a bound user thread");
        waiter.signal = Some((signals, mode));
        // Step 2: 向信号控制器注册本轮 Waker，并立即检查注册前是否已有待处理信号到达。
        let (signals, mode) = waiter.signal.as_ref().unwrap();
        signals.register_wait(waiter.id, waiter.waker(), *mode);
        if signals.interruption_pending(*mode) {
            waiter.waker().notify();
        }
        waiter
    }

    /// 返回本轮等待的唯一代次序号 [`WaitId`]。
    pub fn id(&self) -> WaitId {
        self.id
    }

    /// 创建绑定到本轮等待的轻量级唤醒句柄 [`Waker`]。
    ///
    /// 内部仅持有任务的 [`Weak`] 引用，不产生堆内存分配，也不会延长目标任务或内核栈的生命周期。
    pub fn waker(&self) -> Waker {
        Waker {
            task: Arc::downgrade(&self.task),
            id: self.id,
        }
    }

    /// 阻塞当前任务直到本轮等待收到 [`Waker::notify`] 通知。
    ///
    /// 若在调用前通知已经到达，调度器会直接消费通知并立即返回，不会发生上下文切换。
    /// 只能由创建该 [`Waiter`] 的任务自身调用。
    pub fn park(&self) {
        assert_eq!(task::current_task_id(), Some(self.task.id()));
        sched::park(self.id);
    }

    /// 在信号敏感等待模式下阻塞当前任务，并返回唤醒原因（业务唤醒或信号中断）。
    ///
    /// 调用方在收到返回值后仍须优先复查业务条件是否已经完成，以决定完成业务还是处理信号中断。
    pub fn park_signal(&self) -> WaitResult {
        assert_eq!(task::current_task_id(), Some(self.task.id()));
        let (signals, mode) = self
            .signal
            .as_ref()
            .expect("park_signal requires a signal wait");
        // Step 1: 睡眠前再次检查是否已有符合模式的待处理信号，若有则预先触发自身 Waker。
        if signals.interruption_pending(*mode) {
            self.waker().notify();
        }
        // Step 2: 进入调度器睡眠仲裁；若复查与真正切出 CPU 之间有新信号到达，信号发送方会通过已登记的同一 Waker 唤醒本任务。
        self.park();
        // Step 3: 从睡眠恢复后根据当前实时待处理信号状态判定是否被信号中断。
        if signals.interruption_pending(*mode) {
            WaitResult::Interrupted
        } else {
            WaitResult::Woken
        }
    }

    /// 测试辅助接口：模拟任务进入 `SwitchingOut` 状态但尚未完成上下文保存时的并发通知，验证不会提前将任务放入就绪队列。
    #[cfg(feature = "ktest")]
    pub fn test_park_notify(&self) {
        assert_eq!(task::current_task_id(), Some(self.task.id()));
        let waker = self.waker();
        sched::test_park_notify(&self.task, self.id, &waker);
    }
}

impl Drop for Waiter {
    /// 析构时自动结束本轮等待：先注销信号唤醒源（若有），再通知调度器清除活跃等待代次。
    fn drop(&mut self) {
        if let Some((signals, _)) = &self.signal {
            signals.unregister_wait(self.id);
        }
        sched::end_wait(&self.task, self.id);
    }
}

/// 指向特定任务特定等待轮次的异步唤醒句柄。
///
/// 支持克隆并跨线程/跨中断上下文传递；内部通过 [`Weak`] 引用指向目标任务，不会阻止任务退出或资源回收。
#[derive(Clone)]
pub struct Waker {
    /// 目标等待任务的弱引用；若任务已退出销毁则升级返回 `None`。
    task: Weak<Task>,
    /// 目标等待轮次的唯一代次序号，用于向调度器核对目标任务是否仍处于该轮等待中。
    id: WaitId,
}

impl Waker {
    /// 尝试唤醒目标任务的指定等待轮次。
    ///
    /// 若目标任务仍处于 `self.id` 对应的等待轮次且本次为首次有效通知，则返回 `true`；
    /// 若任务已销毁、该轮等待已结束或此前已被唤醒过，则返回 `false`。
    /// `Weak::upgrade` 与临时强引用 `Arc<Task>` 的析构均在调度器自旋锁外完成。
    pub fn notify(&self) -> bool {
        let Some(task) = self.task.upgrade() else {
            return false;
        };
        sched::notify(&task, self.id)
    }
}
