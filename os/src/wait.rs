//! 一次等待只拥有当前 Task 的现存控制块；通知通过弱引用核对唯一代次。

pub mod queue;

pub use queue::{WaitNode, WaitQueue};

use alloc::sync::{Arc, Weak};
use core::marker::PhantomData;

use crate::sched;
use crate::task::signal::ThreadSignals;
use crate::task::{self, Task};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WaitId(usize);

impl WaitId {
    /// 返回本任务本轮等待的稳定序号，供实际恢复事件保存小值身份。
    pub(crate) const fn as_usize(self) -> usize {
        self.0
    }

    /// 新任务从未使用的零序号仅作为下一次等待的初始值。
    pub(crate) const fn initial() -> Self {
        Self(0)
    }

    /// 每次开始等待取得新身份，耗尽后直接暴露错误而不重用旧代次。
    pub(crate) fn next(self) -> Self {
        Self(self.0.checked_add(1).expect("wait ID space exhausted"))
    }
}

/// 仅用户显式选择的等待响应信号；内核普通等待不登记这些模式。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitMode {
    Interruptible,
    Killable,
}

/// 通知只表示本轮可继续检查业务条件，信号中断由调用者决定返回或重启。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitResult {
    Woken,
    Interrupted,
}

/// 等待拥有者不能转交其他 Task，但可随当前 Task 迁移到另一 CPU。
pub struct Waiter {
    task: Arc<Task>,
    id: WaitId,
    signal: Option<(Arc<ThreadSignals>, WaitMode)>,
    _task_local: PhantomData<*mut ()>,
}

impl Waiter {
    /// 在合法普通 Task 中开始唯一等待，克隆现存控制块而不分配等待对象。
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

    /// 当前用户线程开始可被未屏蔽信号中断的唯一等待，缺少用户身份时拒绝使用。
    pub fn new_interruptible() -> Self {
        Self::new_signal(WaitMode::Interruptible)
    }

    /// 当前用户线程开始仅响应致命退出请求的唯一等待，普通信号不提前归还资源。
    pub fn new_killable() -> Self {
        Self::new_signal(WaitMode::Killable)
    }

    /// 原 active wait 先建立，再登记信号源并重查登记前已经到达的 pending。
    fn new_signal(mode: WaitMode) -> Self {
        let mut waiter = Self::new();
        let signals = waiter
            .task
            .user_thread()
            .and_then(|user| user.signals())
            .expect("signal wait requires a bound user thread");
        waiter.signal = Some((signals, mode));
        let (signals, mode) = waiter.signal.as_ref().unwrap();
        signals.register_wait(waiter.id, waiter.waker(), *mode);
        if signals.interruption_pending(*mode) {
            waiter.waker().notify();
        }
        waiter
    }

    /// 返回本次等待身份，供通知来源区分已结束的旧轮次。
    pub fn id(&self) -> WaitId {
        self.id
    }

    /// 创建不保活任务或执行栈的通知引用，不分配新的控制块。
    pub fn waker(&self) -> Waker {
        Waker {
            task: Arc::downgrade(&self.task),
            id: self.id,
        }
    }

    /// 只允许原 Task 阻塞本轮等待，已经接受通知时立即返回。
    pub fn park(&self) {
        assert_eq!(task::current_task_id(), Some(self.task.id()));
        sched::park(self.id);
    }

    /// 登记后再次检查信号，再经原 notified 仲裁睡眠；返回前以实时 pending 判定中断。
    /// 业务条件仍须由等待源重查，已完成的业务可优先于同时到达的信号。
    pub fn park_signal(&self) -> WaitResult {
        assert_eq!(task::current_task_id(), Some(self.task.id()));
        let (signals, mode) = self
            .signal
            .as_ref()
            .expect("park_signal requires a signal wait");
        if signals.interruption_pending(*mode) {
            self.waker().notify();
        }
        // 重查与真正睡眠之间的到达仍通过同一 Waker 置位；SwitchingOut 只在保存 ctx 后入队。
        self.park();
        if signals.interruption_pending(*mode) {
            WaitResult::Interrupted
        } else {
            WaitResult::Woken
        }
    }

    /// 真实提交 SwitchingOut 后通知，在保存 ctx 前验证没有提前发表 ready。
    #[cfg(feature = "ktest")]
    pub fn test_park_notify(&self) {
        assert_eq!(task::current_task_id(), Some(self.task.id()));
        let waker = self.waker();
        sched::test_park_notify(&self.task, self.id, &waker);
    }
}

impl Drop for Waiter {
    /// 先按本轮身份撤销信号源，再由原 end_wait 唯一清 active；强引用均在锁外析构。
    fn drop(&mut self) {
        if let Some((signals, _)) = &self.signal {
            signals.unregister_wait(self.id);
        }
        sched::end_wait(&self.task, self.id);
    }
}

#[derive(Clone)]
pub struct Waker {
    task: Weak<Task>,
    id: WaitId,
}

impl Waker {
    /// 返回本轮是否首次接受通知，不代表业务条件成立；升级与临时强引用析构均在锁外。
    pub fn notify(&self) -> bool {
        let Some(task) = self.task.upgrade() else {
            return false;
        };
        sched::notify(&task, self.id)
    }
}
