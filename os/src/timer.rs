//! 每核精确定时树与粗粒度轮共用稳定节点、停止协议和永久完成等待源。

mod queue;
mod sleep;
mod wheel;

pub(crate) use sleep::sleep_until_interruptible;
#[cfg(feature = "ktest")]
pub(crate) use sleep::test_sleep_until;
#[cfg(feature = "ktest")]
pub use sleep::{after, periodic_deadline};
pub use sleep::{sleep, sleep_until};
pub(crate) use wheel::TICK_NS;

use alloc::sync::Arc;
use core::alloc::AllocError;
use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::cpu;
use crate::sync::{IrqSaveGuard, SpinLock};
use crate::time::{self, MonotonicInstant};
use crate::wait::{WaitNode, WaitQueue, Waiter};

const HARD_BUDGET: usize = 32;
const BH_BUDGET: usize = 32;
static SERVICES_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// 精确节点在硬 IRQ 执行短回调，粗粒度节点在固定服务任务的底半部执行。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerKind {
    Precise,
    Coarse,
}

/// 关闭后的节点与粗粒度轮无法表示的期限都不改变已有登记。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmError {
    Closed,
    OutOfRange,
}

/// 非等待取消不会摘掉正在运行节点的下一轮，也不阻止生产者继续登记。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CancelResult {
    Idle,
    Cancelled,
    Busy,
}

pub(super) struct Node {
    rb: queue::Links,
    wheel: wheel::Links,
    expires: MonotonicInstant,
    queued: bool,
    running: bool,
    closed: bool,
    queued_owned: bool,
}

/// 固定 CPU、固定函数与标量上下文；节点一旦登记，地址必须保持固定。
/// 回调不能睡眠、修改 IRQ 状态或同步停止自身；指针上下文的存活由调用者负责。
pub struct Timer {
    cpu: usize,
    kind: TimerKind,
    callback: fn(Pin<&Timer>),
    context: usize,
    pub(super) node: UnsafeCell<Node>,
    owned_raw: *const Self,
    _pinned: PhantomPinned,
}

// SAFETY: Node 仅经固定 CPU 的 base 锁访问，回调由 running 执行权串行；其余字段不可变。
unsafe impl Send for Timer {}
// SAFETY: 共享接口只修改锁内 UnsafeCell；借用登记另有显式 stable 存储责任。
unsafe impl Sync for Timer {}

impl Timer {
    /// 构造未登记的借用节点；登记必须显式采用 unsafe arm_borrowed。
    pub const fn new(
        cpu: usize,
        kind: TimerKind,
        callback: fn(Pin<&Timer>),
        context: usize,
    ) -> Self {
        Self {
            cpu,
            kind,
            callback,
            context,
            node: UnsafeCell::new(Node {
                rb: queue::Links::new(),
                wheel: wheel::Links::new(),
                expires: MonotonicInstant::from_nanos(0),
                queued: false,
                running: false,
                closed: false,
                queued_owned: false,
            }),
            owned_raw: core::ptr::null(),
            _pinned: PhantomPinned,
        }
    }

    /// 先申请唯一控制块，之后入队只取得已有 Arc 的额外强引用。
    /// 未发表时保留 into_raw 的控制块 provenance，后续队列转移不从 payload 借用重造拥有指针。
    pub fn try_new(
        cpu: usize,
        kind: TimerKind,
        callback: fn(Pin<&Timer>),
        context: usize,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        assert!(cpu < cpu::count(), "timer CPU is out of range");
        let pointer = Arc::into_raw(Arc::try_new(Self::new(cpu, kind, callback, context))?);
        // SAFETY: 唯一强引用刚转为 raw，无弱引用或已发表借用，可写入固定分配内的构造字段。
        unsafe { (*pointer.cast_mut()).owned_raw = pointer };
        // SAFETY: pointer 来自本次 into_raw，恢复恰好同一份强引用；保留的指针本身不额外拥有。
        let timer = unsafe { Arc::from_raw(pointer) };
        // SAFETY: Arc 指向独立分配，不会再移出 Timer；公开接口不提供可变解包。
        Ok(unsafe { Pin::new_unchecked(timer) })
    }

    /// 返回构造时固定的标量上下文，不赋予整数所指存储任何拥有权。
    pub fn context(&self) -> usize {
        self.context
    }

    /// 返回节点固定归属的逻辑 CPU。
    #[cfg(feature = "ktest")]
    pub fn cpu(&self) -> usize {
        self.cpu
    }

    /// 拥有节点返回构造时保留的 into_raw 指针；借用节点只返回 payload 地址，不赋予 Arc 拥有权。
    fn node_pointer(&self) -> *const Self {
        if self.owned_raw.is_null() {
            self as *const Self
        } else {
            self.owned_raw
        }
    }

    /// 登记拥有节点；true 表示新增下一轮，false 表示更新已有下一轮。
    /// 借用节点调用此安全接口直接报告错误，不能隐式制造存储保活。
    pub fn arm(self: Pin<&Self>, deadline: MonotonicInstant) -> Result<bool, ArmError> {
        assert!(
            !self.owned_raw.is_null(),
            "borrowed timer requires unsafe arm_borrowed"
        );
        self.arm_inner(deadline)
    }

    /// 登记由调用者保护的稳定存储，运行中可登记独立下一轮。
    ///
    /// # Safety
    /// 节点和回调使用的上下文必须保持稳定且有效，直到 cancel_sync 或 close_sync
    /// 确认真实回调尾部；不得遗忘该责任后移动、释放存储或提前退出栈帧。
    pub unsafe fn arm_borrowed(
        self: Pin<&Self>,
        deadline: MonotonicInstant,
    ) -> Result<bool, ArmError> {
        self.arm_inner(deadline)
    }

    /// 在所属 base 锁中更新下一轮；拥有节点仅在新增登记时取得一份 queued 强引用。
    /// 借用入口须已承担稳定存储和同步停止责任；解锁后才通知期限变化。
    fn arm_inner(self: Pin<&Self>, deadline: MonotonicInstant) -> Result<bool, ArmError> {
        let base = base(self.cpu);
        let pointer = self.node_pointer();
        let mut state = base.state.lock();
        // SAFETY: base 锁保护节点；owned 的现存 Arc 保活保留的 into_raw 指针，borrowed 入口承担稳定存储责任。
        unsafe {
            if (*self.node.get()).closed {
                return Err(ArmError::Closed);
            }
            let now = time::now();
            if self.kind == TimerKind::Coarse {
                state.wheel.accepts(deadline, now)?;
            }
            let added = !(*self.node.get()).queued;
            if !added {
                state.unlink(pointer);
            } else if !self.owned_raw.is_null() {
                Arc::increment_strong_count(pointer);
                (*self.node.get()).queued_owned = true;
            }
            (*self.node.get()).expires = deadline;
            match self.kind {
                TimerKind::Precise => state.precise.insert(pointer),
                TimerKind::Coarse => state
                    .wheel
                    .insert(pointer, now)
                    .expect("prechecked wheel arm failed"),
            }
            (*self.node.get()).queued = true;
            drop(state);
            changed(self.cpu);
            Ok(added)
        }
    }

    /// 只在不运行时摘掉下一轮，正在运行则返回 Busy 且不等待。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "定时器测试逐一验证 Idle、Cancelled 和 Busy 的非等待取消合同"
        )
    )]
    pub fn try_cancel(self: Pin<&Self>) -> CancelResult {
        let pointer = self.node_pointer();
        let mut state = base(self.cpu).state.lock();
        // SAFETY: base 锁串行状态与摘链，返回的 Arc 拥有权只在锁外释放。
        let (result, release) = unsafe {
            if (*self.node.get()).running {
                return CancelResult::Busy;
            }
            if (*self.node.get()).queued {
                (CancelResult::Cancelled, state.cancel(pointer))
            } else {
                (CancelResult::Idle, false)
            }
        };
        drop(state);
        release_owned(pointer, release);
        if result == CancelResult::Cancelled {
            changed(self.cpu);
        }
        result
    }

    /// 只撤下一轮，不等待或改变当前回调；运行中也可摘掉独立的 queued 登记。
    /// 返回是否实际摘链，拥有引用仍在 base 锁外归还；不改变 try_cancel 的 Busy 契约。
    pub(crate) fn cancel_pending(self: Pin<&Self>) -> bool {
        let pointer = self.node_pointer();
        let mut state = base(self.cpu).state.lock();
        // SAFETY: base 锁保护 queued 状态和摘链；运行票独立，始终留给真实回调尾部。
        let (queued, release) = unsafe {
            let queued = (*self.node.get()).queued;
            (queued, state.cancel(pointer))
        };
        drop(state);
        release_owned(pointer, release);
        if queued {
            changed(self.cpu);
        }
        queued
    }

    /// 在可睡普通任务中撤当前登记并等待真实回调尾部，不阻止将来重新登记。
    /// 调用者先停止外部生产者；不能从自身回调调用。
    pub fn cancel_sync(self: Pin<&Self>) -> bool {
        self.stop_sync(false)
    }

    /// 先永久关闭节点，再撤下一轮并在可睡普通任务中等待真实回调尾部。
    /// 关闭后所有登记返回 Closed；不能从自身回调调用。
    pub fn close_sync(self: Pin<&Self>) -> bool {
        self.stop_sync(true)
    }

    /// 仅供可睡普通任务调用，且当前不得有其他活 Waiter；每轮登记后复查 running 再等待尾部。
    /// close 先在锁内永久闭门；普通取消只撤当前及回调重排的登记，不阻止后续 arm。
    /// 摘出的强引用在锁外释放；撤销本轮等待后才可开始下一轮唯一等待。
    fn stop_sync(self: Pin<&Self>, close: bool) -> bool {
        crate::sched::assert_switchable();
        let base = base(self.cpu);
        let pointer = self.node_pointer();
        let mut cancelled = false;
        loop {
            let mut state = base.state.lock();
            // SAFETY: 关闭、摘链和 running 复查共用所属 base 的唯一锁。
            let (release, running, queued) = unsafe {
                if close {
                    (*self.node.get()).closed = true;
                }
                let queued = (*self.node.get()).queued;
                cancelled |= queued;
                let release = state.cancel(pointer);
                (release, (*self.node.get()).running, queued)
            };
            drop(state);
            release_owned(pointer, release);
            if queued {
                changed(self.cpu);
            }
            if !running {
                return cancelled;
            }
            let waiter = Waiter::new();
            let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 永久 base 与固定栈节点存活至 registration 先于 node 析构；不遗忘登记。
            let registration = unsafe { base.finished.register(node.as_mut()) };
            let state = base.state.lock();
            // SAFETY: 登记完成后在同一 base 锁复查；尾部清位后的 wake 不会丢失。
            let running = unsafe { (*self.node.get()).running };
            drop(state);
            if running {
                waiter.park();
            }
            drop(registration);
            drop(waiter);
        }
    }
}

impl Drop for Timer {
    /// 析构只核对存储停止责任；不隐式等待远端 CPU 或替代调用者停止生产者。
    fn drop(&mut self) {
        let _state = base(self.cpu).state.lock();
        // SAFETY: 析构持所属 base 锁；合法 owned 最后引用与 borrowed 同步停止后均无活登记。
        let node = unsafe { &*self.node.get() };
        assert!(!node.queued && !node.running, "dropping an active timer");
        assert!(!node.queued_owned, "dropping queued timer ownership");
    }
}

struct BaseState {
    precise: queue::Queue,
    wheel: wheel::Wheel,
    service_pending: bool,
    #[cfg(feature = "ktest")]
    stats: testing::TimerStats,
}

// SAFETY: 全部 raw 队列成员受稳定存储协议保护，状态只在所属 TimerBase 锁中访问。
unsafe impl Send for BaseState {}

impl BaseState {
    /// 为尚未发表的 base 构造空队列、无待处理服务和零计数，不取得节点拥有权。
    const fn new() -> Self {
        Self {
            precise: queue::Queue::new(),
            wheel: wheel::Wheel::new(),
            service_pending: false,
            #[cfg(feature = "ktest")]
            stats: testing::TimerStats::new(),
        }
    }

    /// 只撤 intrusive 链接，不改变 queued 或强引用；调用者继续重排或显式转移拥有权。
    ///
    /// # Safety
    /// 调用者必须持所属 base 锁，timer 是本队列中仍有效的固定地址成员。
    unsafe fn unlink(&mut self, timer: *const Timer) {
        // SAFETY: 私有调用者证明该节点是本队列成员且所属 base 锁已持有。
        unsafe {
            match (*timer).kind {
                TimerKind::Precise => self.precise.remove(timer),
                TimerKind::Coarse => self.wheel.remove(timer),
            }
        }
    }

    /// 返回需要在锁外释放的 queued 强拥有，借用节点始终返回 false。
    ///
    /// # Safety
    /// 调用者必须持所属 base 锁，timer 存储保持有效；若 queued 为真，它属于本队列。
    unsafe fn cancel(&mut self, timer: *const Timer) -> bool {
        // SAFETY: 私有调用者持 base 锁；只修改节点内部状态，不建立覆盖 Timer 的可变借用。
        unsafe {
            let node = (*timer).node.get();
            if !(*node).queued {
                return false;
            }
            self.unlink(timer);
            (*node).queued = false;
            let release = (*node).queued_owned;
            (*node).queued_owned = false;
            release
        }
    }

    /// 在所属 base 锁保护下检查轮中最早有效期限，不摘链或改变服务状态。
    fn coarse_due(&self, now: MonotonicInstant) -> bool {
        self.wheel.next().is_some_and(|deadline| deadline <= now)
    }
}

/// CPU 永久存储中的两个队列和等待源；尾部唤醒绝不再访问可退栈的 Timer。
pub(crate) struct TimerBase {
    state: SpinLock<BaseState>,
    service: WaitQueue,
    finished: WaitQueue,
    #[cfg(feature = "ktest")]
    service_task: SpinLock<Option<Arc<crate::task::Task>>>,
}

impl TimerBase {
    /// 在 CpuLocal 静态构造阶段准备空队列和永久等待源，不申请控制对象。
    pub(crate) const fn new() -> Self {
        Self {
            state: SpinLock::new(BaseState::new()),
            service: WaitQueue::new(),
            finished: WaitQueue::new(),
            #[cfg(feature = "ktest")]
            service_task: SpinLock::new(None),
        }
    }
}

/// 取得指定有效逻辑 CPU 的永久 base；这里只定位存储，不取得状态锁。
fn base(cpu: usize) -> &'static TimerBase {
    &cpu::get(cpu).timer_base
}

/// 队列修改解锁后调用：本核重查并编程，远核只发表重编程原因，不传旧期限。
/// 临时屏蔽本核 IRQ，不能在持有任何 base 状态锁时调用。
fn changed(target: usize) {
    let _irq = IrqSaveGuard::new();
    if target == cpu::current().id {
        time::reprogram_current();
    } else {
        cpu::kick(target, cpu::KICK_TIMER_REPROGRAM);
    }
}

/// 仅在 base 锁外消费此前由取消或回调尾部转出的一份 raw Arc 强引用。
/// release 为真须对应尚未释放的 queued/running 拥有权；借用节点始终为假且不解引用。
/// 有拥有权时 timer 必须沿队列转移构造时保留的 into_raw 指针，不得由 payload 借用重新构造。
fn release_owned(timer: *const Timer, release: bool) {
    if release {
        // SAFETY: 私有调用者传入保留的 into_raw 指针及唯一转出的 queued/running raw strong，base 锁已释放。
        drop(unsafe { Arc::from_raw(timer) });
    }
}

/// 持本 CPU 的 base 锁读最终期限并执行硬件提交，operation 只能做换算和寄存器编程。
pub(crate) fn with_next_deadline<R>(operation: impl FnOnce(Option<MonotonicInstant>) -> R) -> R {
    let _irq = IrqSaveGuard::new();
    let state = cpu::current().timer_base.state.lock();
    let precise = state.precise.first();
    // 已有服务承担到期粗节点时不反复安装 immediate，服务结束会重查下一期限。
    let coarse = if state.service_pending {
        None
    } else {
        state.wheel.next()
    };
    let deadline = match (precise, coarse) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    };
    operation(deadline)
}

/// 硬 IRQ 或服务底半部在锁内摘到期节点，将 queued 执行权及其强引用转为 running。
/// 已 running 的下一轮放回原队列；成功结果只交给 run_callback，借用保活责任不变。
/// 拥有节点的原始指针及 strong 原样转交，不从节点引用重新构造 Arc 指针。
fn take_due(base: &TimerBase, kind: TimerKind) -> Option<(*const Timer, bool)> {
    let mut state = base.state.lock();
    let now = time::now();
    // SAFETY: 两个 intrusive 队列只在 base 锁下摘取，存储由 queued 拥有或 borrowed 责任保护。
    unsafe {
        let timer = match kind {
            TimerKind::Precise => state.precise.pop_due(now),
            TimerKind::Coarse => state.wheel.pop_due(now),
        }?;
        let node = (*timer).node.get();
        if (*node).running {
            match kind {
                TimerKind::Precise => state.precise.insert(timer),
                TimerKind::Coarse => state
                    .wheel
                    .insert(timer, now)
                    .expect("running wheel node out of range"),
            }
            return None;
        }
        (*node).queued = false;
        (*node).running = true;
        let owned = (*node).queued_owned;
        (*node).queued_owned = false;
        Some((timer, owned))
    }
}

/// 硬 IRQ 或不可睡底半部在 base 锁外执行 take_due 交出的唯一 running 回调。
/// 完成时锁内清 running 是最后一次借用节点访问；锁外仅唤醒永久 finished 并释放运行强引用。
/// base、节点归属和 kind 必须匹配；owned 只表示此次转入的运行引用，不包含重排的 queued 引用。
fn run_callback(base: &'static TimerBase, timer: *const Timer, owned: bool, kind: TimerKind) {
    #[cfg(feature = "ktest")]
    testing::pause(timer, testing::PausePoint::Dequeued);
    // SAFETY: running 独占执行权和 transferred Arc 或 borrowed 停止责任保活，回调仅共享固定节点。
    unsafe {
        let pinned = Pin::new_unchecked(&*timer);
        ((*timer).callback)(pinned);
    }
    #[cfg(feature = "ktest")]
    testing::pause(timer, testing::PausePoint::Tail);
    let state = base.state.lock();
    #[cfg(feature = "ktest")]
    let mut state = state;
    #[cfg(feature = "ktest")]
    match kind {
        TimerKind::Precise => state.stats.hard_callbacks += 1,
        TimerKind::Coarse => state.stats.bh_callbacks += 1,
    }
    #[cfg(not(feature = "ktest"))]
    let _ = kind;
    // SAFETY: 仍持 running 执行权及 base 锁；清 running 是 runner 对 borrowed 节点最后一次访问。
    unsafe { (*(*timer).node.get()).running = false };
    drop(state);
    base.finished.wake_all();
    release_owned(timer, owned);
}

/// 在真实硬 IRQ 运行有界精确批次，只发表粗到期并唤醒预建服务任务。
pub(crate) fn handle_interrupt() {
    let base = &cpu::current().timer_base;
    for _ in 0..HARD_BUDGET {
        let Some((timer, owned)) = take_due(base, TimerKind::Precise) else {
            break;
        };
        run_callback(base, timer, owned, TimerKind::Precise);
    }
    let mut state = base.state.lock();
    let wake = !state.service_pending && state.coarse_due(time::now());
    if wake {
        state.service_pending = true;
    }
    drop(state);
    if wake {
        base.service.wake_one();
    }
}

/// BSP 在 IRQ/AP 启动之前为每 CPU 准备真实常驻服务任务和执行栈。
pub fn initialize_services() -> Result<(), AllocError> {
    assert_eq!(cpu::current().id, 0, "timer services require BSP");
    assert!(
        !SERVICES_INITIALIZED.swap(true, Ordering::AcqRel),
        "timer services initialized twice"
    );
    for cpu in 0..cpu::count() {
        drop(crate::task::spawn_kernel_on(cpu, move || service(cpu))?);
    }
    Ok(())
}

/// 固定 CPU 的常驻普通任务登记永久服务源，无到期工作时真实阻塞。
/// 粗回调在不可睡底半部中按预算执行；先退出底半部并重编程，再 yield 续批。
fn service(cpu: usize) -> i32 {
    let base = base(cpu);
    #[cfg(feature = "ktest")]
    {
        let task = crate::task::current_task();
        let mut observation = base.service_task.lock();
        assert!(observation.is_none(), "duplicate timer service");
        *observation = Some(task);
    }
    loop {
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: 永久 base 与固定节点存活至登记撤销，不遗忘 registration。
        let registration = unsafe { base.service.register(node.as_mut()) };
        let pending = base.state.lock().service_pending;
        if !pending {
            waiter.park();
        }
        drop(registration);
        drop(waiter);
        loop {
            let bh = cpu::BottomHalfGuard::enter();
            let mut callbacks = 0;
            while callbacks < BH_BUDGET {
                let Some((timer, owned)) = take_due(base, TimerKind::Coarse) else {
                    break;
                };
                run_callback(base, timer, owned, TimerKind::Coarse);
                callbacks += 1;
            }
            let mut state = base.state.lock();
            let more = state.coarse_due(time::now());
            state.service_pending = more;
            #[cfg(feature = "ktest")]
            if 0 < callbacks {
                state.stats.bh_batches += 1;
                state.stats.max_bh_callbacks = state.stats.max_bh_callbacks.max(callbacks);
                if more {
                    state.stats.bh_continuations += 1;
                }
            }
            drop(state);
            drop(bh);
            changed(cpu);
            if !more {
                break;
            }
            crate::sched::yield_now();
        }
    }
}

/// 验收仅观察真实队列、执行计数与固定服务任务，不提供另一套后端。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    use super::*;
    use core::hint::spin_loop;
    use core::sync::atomic::AtomicUsize;

    pub(crate) const BH_BUDGET: usize = super::BH_BUDGET;
    pub(crate) const TICK_NS: u64 = super::wheel::TICK_NS;
    pub(crate) const MAX_TIMEOUT_TICKS: u64 = super::wheel::MAX_TIMEOUT_TICKS;

    #[derive(Clone, Copy)]
    pub(crate) struct TimerSnapshot {
        pub queued: bool,
        pub running: bool,
        pub closed: bool,
        pub expiry: MonotonicInstant,
    }

    #[derive(Clone, Copy)]
    pub(crate) struct TimerStats {
        pub hard_callbacks: usize,
        pub bh_callbacks: usize,
        pub bh_batches: usize,
        pub bh_continuations: usize,
        pub max_bh_callbacks: usize,
    }

    impl TimerStats {
        /// 预备零计数快照，仅由所属 base 在真实执行边界累计。
        pub(super) const fn new() -> Self {
            Self {
                hard_callbacks: 0,
                bh_callbacks: 0,
                bh_batches: 0,
                bh_continuations: 0,
                max_bh_callbacks: 0,
            }
        }
    }

    /// 在唯一 base 锁中快照节点生命周期与实际登记的请求期限。
    pub(crate) fn snapshot(timer: Pin<&Timer>) -> TimerSnapshot {
        let _state = base(timer.cpu).state.lock();
        // SAFETY: 共享 Pin 保活稳定节点，全部字段由所属 base 锁保护。
        let node = unsafe { &*timer.node.get() };
        TimerSnapshot {
            queued: node.queued,
            running: node.running,
            closed: node.closed,
            expiry: node.expires,
        }
    }

    /// 返回真实硬回调和服务批次累计，不以登记次数冒充执行次数。
    pub(crate) fn stats(cpu: usize) -> TimerStats {
        base(cpu).state.lock().stats
    }

    /// 克隆服务真实入口已发表的观察强引用，不改变服务任务的执行与存储责任。
    pub(crate) fn service_task(cpu: usize) -> Option<Arc<crate::task::Task>> {
        base(cpu).service_task.lock().as_ref().map(Arc::clone)
    }

    /// 在生产树锁域中核对真实成员、黑高与完整结构不变量。
    pub(crate) fn validate_tree(cpu: usize) -> (usize, usize) {
        let state = base(cpu).state.lock();
        // SAFETY: base 锁与各 queued 存储责任保护验证遍历的全部成员。
        unsafe { state.precise.validate() }
    }

    #[derive(Clone, Copy)]
    pub(crate) enum PausePoint {
        Dequeued,
        Tail,
    }

    const ARMED: usize = 1;
    const PAUSED: usize = 2;
    const RELEASED: usize = 3;
    const PAUSE_TIMEOUT_SECONDS: u64 = 5;

    static PAUSE_TIMER: AtomicUsize = AtomicUsize::new(0);
    static PAUSE_POINT: AtomicUsize = AtomicUsize::new(0);
    static PAUSE_STATE: AtomicUsize = AtomicUsize::new(0);

    /// 武装一个真实摘离或回调尾部窗口，只允许另一 CPU 控制有限暂停。
    /// 同一停点仅在原 timer 真实同步停止、所有生产者和控制子任务 join 后才可重武装。
    pub(crate) fn arm_pause(timer: Pin<&Timer>, point: PausePoint) {
        assert_ne!(timer.cpu, cpu::current().id, "timer pause needs remote CPU");
        assert_eq!(
            PAUSE_STATE.load(Ordering::Acquire),
            0,
            "timer pause already armed"
        );
        PAUSE_TIMER.store(timer.get_ref() as *const Timer as usize, Ordering::Relaxed);
        PAUSE_POINT.store(point as usize, Ordering::Relaxed);
        PAUSE_STATE.store(ARMED, Ordering::Release);
    }

    /// 观察真实 runner 已在无锁窗口暂停，不读取另一套生命周期模型。
    pub(crate) fn is_paused() -> bool {
        PAUSE_STATE.load(Ordering::Acquire) == PAUSED
    }

    /// 在控制者观察暂停以后释放 runner，让其走真实回调尾部。
    pub(crate) fn release_pause() {
        PAUSE_STATE
            .compare_exchange(PAUSED, RELEASED, Ordering::Release, Ordering::Relaxed)
            .expect("timer runner was not paused");
    }

    /// 撤销尚武装的停点或放行真实 runner；空状态和已放行状态可重复取消。
    /// 不改变 queued/running、不归还节点引用，也不等待；控制者必须锁外真实关闭 timer、
    /// 等待回调尾部并 join 全部生产者，保持上下文有效到停止返回后才允许重武装或释放。
    pub(crate) fn cancel_pause() {
        loop {
            let phase = PAUSE_STATE.load(Ordering::Acquire);
            let next = match phase {
                0 | RELEASED => return,
                ARMED => 0,
                PAUSED => RELEASED,
                _ => panic!("invalid timer pause state"),
            };
            if PAUSE_STATE
                .compare_exchange(phase, next, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return;
            }
        }
    }

    /// runner 在锁外且仍持 running 执行权时匹配指定窗口，不解引用或接管节点。
    /// 仅等待另一 CPU 控制者释放，使用有限硬件计数器超时，不睡眠或切换任务。
    pub(super) fn pause(timer: *const Timer, point: PausePoint) {
        if PAUSE_STATE.load(Ordering::Acquire) != ARMED
            || PAUSE_TIMER.load(Ordering::Relaxed) != timer as usize
            || PAUSE_POINT.load(Ordering::Relaxed) != point as usize
        {
            return;
        }
        if PAUSE_STATE
            .compare_exchange(ARMED, PAUSED, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return;
        }
        let start = crate::arch::read_counter();
        let limit = crate::arch::counter_frequency()
            .checked_mul(PAUSE_TIMEOUT_SECONDS)
            .expect("timer pause raw timeout overflow");
        while PAUSE_STATE.load(Ordering::Acquire) == PAUSED {
            assert!(
                crate::arch::read_counter().wrapping_sub(start) < limit,
                "timer runner pause timed out"
            );
            spin_loop();
        }
        PAUSE_TIMER.store(0, Ordering::Relaxed);
        PAUSE_STATE.store(0, Ordering::Release);
    }

    /// 使用轮子拥有者的生产边界验收，不复制放置或到期算法。
    pub(crate) fn self_check_wheel() -> usize {
        super::wheel::self_check()
    }

    /// 返回验收睡眠最近一次真实登记的期限，只由专用 hook 路径发表。
    pub(crate) fn sleep_deadline() -> MonotonicInstant {
        super::sleep::test_sleep_deadline()
    }
}
