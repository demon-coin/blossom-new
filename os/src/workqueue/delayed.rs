//! 粗定时登记与工作池共享一个下一轮，真正的运行票在两种回调尾部分别归还。

use alloc::sync::Arc;
use core::alloc::AllocError;
use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::ptr;

use crate::time::{self, Duration, MonotonicInstant};
use crate::timer::{ArmError, Timer, TimerKind};

use super::{QueueError, RemovedTicket, Work, WorkQueue, STATE_LOCK};

// 一次更新至多撤旧 pending、旧 timer 和未采用的候选 timer 三份强引用。
const RELEASE_CAPACITY: usize = 3;

/// 关闭与无法表示的延迟均不破坏先前已接受的下一轮。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DelayedError {
    Closed,
    OutOfRange,
}

#[derive(Clone, Copy)]
enum Pending {
    None,
    Delay(MonotonicInstant),
    Pool,
}

struct Metadata {
    pending: Pending,
    owner: *const DelayedWork,
}

/// 固定域、实际 CPU 和业务函数的拥有式延迟工作；停止责任必须显式履行。
/// 同步操作不能从自身回调调用，业务上下文须存活至 close_sync 返回。
pub struct DelayedWork {
    work: Option<Pin<Arc<Work>>>,
    timer: Option<Pin<Arc<Timer>>>,
    callback: fn(Pin<&Self>),
    context: usize,
    owned_raw: *const Self,
    metadata: UnsafeCell<Metadata>,
    _pinned: PhantomPinned,
}

// SAFETY: 构造后的字段不变；全部下一轮状态由唯一 STATE_LOCK 保护，业务函数由 Work 串行执行。
unsafe impl Send for DelayedWork {}
// SAFETY: 共享接口只修改锁保护的元数据，异步路径均持真实 Arc 强票保活。
unsafe impl Sync for DelayedWork {}

/// 锁下只收集拥有票，离开 STATE_LOCK 后再归还引用和移出的 Work 票。
struct Releases {
    raw: [*const DelayedWork; RELEASE_CAPACITY],
    count: usize,
    work: Option<RemovedTicket>,
}

impl Releases {
    /// 准备足以容纳一次更新的旧 pending、撤销 timer 和候选 timer 三份引用。
    fn new() -> Self {
        Self {
            raw: [ptr::null(); RELEASE_CAPACITY],
            count: 0,
            work: None,
        }
    }

    /// 记录一份真实 into_raw 指针对应的待归还强引用，不在状态锁中析构。
    fn push(&mut self, raw: *const DelayedWork) {
        if !raw.is_null() {
            self.raw[self.count] = raw;
            self.count += 1;
        }
    }
}

impl Drop for Releases {
    /// 调用栈先释放状态锁；每份票只归还一次，不等待任何执行者。
    fn drop(&mut self) {
        for index in 0..self.count {
            // SAFETY: push 只接收真实转出的强票，raw 来自构造时保留的 Arc::into_raw。
            drop(unsafe { Arc::from_raw(self.raw[index]) });
        }
        drop(self.work.take());
    }
}

impl DelayedWork {
    /// 所有稳定控制对象先 fallible 准备，再关联未发表的 Work；失败没有已接受登记。
    pub fn try_new(
        queue: &WorkQueue,
        cpu: usize,
        callback: fn(Pin<&Self>),
        context: usize,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        let owner = Arc::try_new(Self {
            work: None,
            timer: None,
            callback,
            context,
            owned_raw: ptr::null(),
            metadata: UnsafeCell::new(Metadata {
                pending: Pending::None,
                owner: ptr::null(),
            }),
            _pinned: PhantomPinned,
        })?;
        let raw = Arc::into_raw(owner);
        // SAFETY: 恢复同一真实强引用；对象仍未发表，局部 owner 保证后续 fallible 构造失败正常析构。
        let owner = unsafe { Arc::from_raw(raw) };
        let work = Work::try_new(queue, cpu, run_work, raw as usize)?;
        let timer = Timer::try_new(work.cpu(), TimerKind::Coarse, transfer, raw as usize)?;
        // SAFETY: 唯一 owner 尚未发表、无弱引用，固定分配内初始化一次，不移动 pinned 字段。
        unsafe {
            (*raw.cast_mut()).owned_raw = raw;
            (*raw.cast_mut()).work = Some(work);
            (*raw.cast_mut()).timer = Some(timer);
        }
        // SAFETY: 独立 Arc 分配不再移出；公开接口不提供可变解包。
        let owner = unsafe { Pin::new_unchecked(owner) };
        owner.work().set_delayed_owner(raw);
        Ok(owner)
    }

    /// 返回固定业务上下文；整数不赋予其目标存储任何拥有权。
    pub fn context(&self) -> usize {
        self.context
    }

    /// 返回 Work 与粗 Timer 共同绑定的实际 CPU，ordered 域已经规范到 CPU0。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "延后工作测试核对 Timer 与 Work 的共同绑定 CPU")
    )]
    pub fn cpu(&self) -> usize {
        self.work().cpu()
    }

    /// 仅无下一轮时登记；重复或同步取消暂拒返回 false，不代表当前业务函数已停止。
    pub fn queue(self: Pin<&Self>, delay: Duration) -> Result<bool, DelayedError> {
        self.schedule(delay, false)
    }

    /// 原子替换 Delay 或 Pool 下一轮；true 表示原先有 pending，失败完整保留旧登记。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "延后工作测试原子替换 Delay 和 Pool 阶段的下一轮票"
        )
    )]
    pub fn modify(self: Pin<&Self>, delay: Duration) -> Result<bool, DelayedError> {
        self.schedule(delay, true)
    }

    /// 先通过内层 Work 拒绝新生产，再停真实 timer 尾部，最后撤工作并等待业务尾部。
    /// 普通取消返回后同一对象仍可复用。
    pub fn cancel_sync(self: Pin<&Self>) -> bool {
        self.stop(false)
    }

    /// 只由内层 Work 永久闭门，随后依次同步 timer 与业务尾部；Drop 不另做停止。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "延后工作 owner 测试永久闭门并等待 timer 与业务回调的真实尾部"
        )
    )]
    pub fn close_sync(self: Pin<&Self>) -> bool {
        self.stop(true)
    }

    /// 将调用边界内的 Delay 立即移交同一 Work，并只等待同锁捕获的固定工作票边界。
    /// 边界后的生产不延长本次等待，不能从自身业务函数同步调用。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "延后工作测试将未到期 Delay 移交 Work 并等待捕获的固定票边界"
        )
    )]
    pub fn flush(self: Pin<&Self>) -> bool {
        crate::sched::assert_switchable();
        let mut releases = Releases::new();
        let serial = self.work().lock_stop_serial();
        let state = STATE_LOCK.lock();
        // SAFETY: 同一状态锁保护 pending，Work 的边界也在这把锁中捕获。
        let delayed = unsafe { matches!((*self.metadata.get()).pending, Pending::Delay(_)) };
        if delayed && self.work().accepts_locked() {
            self.cancel_timer_locked(&mut releases);
            // SAFETY: Delay 的强票保持不变，仅转交至同一 Work 的下一轮。
            unsafe { (*self.metadata.get()).pending = Pending::Pool };
            assert!(self
                .work()
                .queue_locked()
                .expect("accepted delayed flush closed"));
        }
        let discard = delayed && self.work().is_closed_locked();
        if discard {
            self.cancel_timer_locked(&mut releases);
            // SAFETY: 域或 owner 已闭门，不再可能移交；当前 Delay 票只在锁外归还。
            let raw = unsafe {
                let metadata = &mut *self.metadata.get();
                metadata.pending = Pending::None;
                core::mem::replace(&mut metadata.owner, ptr::null())
            };
            releases.push(raw);
        }
        let boundary = self.work().capture_locked();

        drop(state);
        drop(serial);
        drop(releases);

        if discard {
            self.timer().cancel_sync();
        }
        self.work().notify();
        delayed | self.work().flush_boundary(boundary)
    }

    /// 读取构造完成的固定 Work；未发表失败对象只在 Drop 中读取可选字段。
    fn work(&self) -> Pin<&Work> {
        self.work
            .as_ref()
            .expect("unpublished delayed work")
            .as_ref()
    }

    /// 读取构造完成的固定粗 Timer，节点拥有与上下文拥有分开记账。
    fn timer(&self) -> Pin<&Timer> {
        self.timer
            .as_ref()
            .expect("unpublished delayed timer")
            .as_ref()
    }

    /// 在同一状态锁中完成可失败期限检查、旧登记撤销与新单 pending 发布。
    fn schedule(self: Pin<&Self>, delay: Duration, modify: bool) -> Result<bool, DelayedError> {
        let mut releases = Releases::new();
        let state = STATE_LOCK.lock();
        if self.work().is_closed_locked() {
            return Err(DelayedError::Closed);
        }
        if !self.work().accepts_locked() {
            return Ok(false);
        }
        // SAFETY: pending 的读取与所有修改都持 STATE_LOCK；owned receiver 保活固定分配。
        let pending = unsafe { (*self.metadata.get()).pending };
        let existed = !matches!(pending, Pending::None);
        if existed && !modify {
            return Ok(false);
        }

        let deadline = time::now()
            .checked_add(delay)
            .ok_or(DelayedError::OutOfRange)?;
        if !delay.is_zero() {
            // SAFETY: 指针来自真实 into_raw，当前 owner 保活；候选票在 timer 发表前取得。
            unsafe { Arc::increment_strong_count(self.owned_raw) };
            match self.timer().arm(deadline) {
                Ok(true) => {}
                Ok(false) => releases.push(self.owned_raw),
                Err(error) => {
                    releases.push(self.owned_raw);
                    return Err(match error {
                        ArmError::Closed => DelayedError::Closed,
                        ArmError::OutOfRange => DelayedError::OutOfRange,
                    });
                }
            }
        } else {
            self.cancel_timer_locked(&mut releases);
        }

        match pending {
            Pending::Pool => releases.work = self.work().cancel_queued_locked(),
            Pending::Delay(_) => {
                // SAFETY: 撤出的唯一 Delay pending 强票仅移到锁外释放集合。
                let raw = unsafe {
                    let metadata = &mut *self.metadata.get();
                    metadata.pending = Pending::None;
                    core::mem::replace(&mut metadata.owner, ptr::null())
                };
                releases.push(raw);
            }
            Pending::None => {}
        }

        // SAFETY: 真实 Arc raw 仍由 receiver 保活；单独取得新 pending 票，运行票不受影响。
        unsafe {
            Arc::increment_strong_count(self.owned_raw);
            let metadata = &mut *self.metadata.get();
            assert!(matches!(metadata.pending, Pending::None));
            metadata.owner = self.owned_raw;
            metadata.pending = if delay.is_zero() {
                Pending::Pool
            } else {
                Pending::Delay(deadline)
            };
        }
        if delay.is_zero() {
            assert!(self
                .work()
                .queue_locked()
                .expect("accepted delayed queue closed"));
        }

        drop(state);
        drop(releases);
        if delay.is_zero() {
            self.work().notify();
        }
        Ok(if modify { existed } else { true })
    }

    /// 持 STATE_LOCK 摘掉实际 timer queued 下一轮，其上下文强票在状态锁外归还。
    fn cancel_timer_locked(self: Pin<&Self>, releases: &mut Releases) {
        if self.timer().cancel_pending() {
            releases.push(self.owned_raw);
        }
    }

    /// 只复用 Work 的串行取消事实，timer 永久关闭不建立第二套 owner closed 状态。
    fn stop(self: Pin<&Self>, close: bool) -> bool {
        let guard = self.work().begin_stop(close);
        let mut releases = Releases::new();
        let state = STATE_LOCK.lock();
        // SAFETY: begin_stop 已拒新生产，当前 Delay 的拥有票在同一锁中摘除。
        let delayed = unsafe { matches!((*self.metadata.get()).pending, Pending::Delay(_)) };
        self.cancel_timer_locked(&mut releases);
        if delayed {
            // SAFETY: 所有 timer 运行上下文各有独立强票，摘 pending 不损害其真实存活。
            let raw = unsafe {
                let metadata = &mut *self.metadata.get();
                metadata.pending = Pending::None;
                core::mem::replace(&mut metadata.owner, ptr::null())
            };
            releases.push(raw);
        }
        drop(state);
        drop(releases);

        self.timer().cancel_sync();
        delayed | guard.finish()
    }

    /// 供 Core 在拒生产的同一锁下捕获取消边界，覆盖 timer callback 抢先撤票的竞态。
    ///
    /// # Safety
    /// 调用者必须持 STATE_LOCK，且构造期关联的 DelayedWork 仍由 owner 或异步票保活。
    pub(super) unsafe fn has_pending_locked(self: Pin<&Self>) -> bool {
        // SAFETY: Core 在唯一状态锁中读取当前单 pending，不额外创造状态或拥有票。
        unsafe { !matches!((*self.metadata.get()).pending, Pending::None) }
    }

    /// Core 真实 claim 时清 Pool pending，把其真实强票原样转入运行拥有票。
    ///
    /// # Safety
    /// 调用者必须持 STATE_LOCK；同一 Work 已取得唯一运行执行权，返回票只能在真实尾部锁外归还。
    pub(super) unsafe fn claim_locked(self: Pin<&Self>) -> *const Self {
        // SAFETY: Core 持共享状态锁，唯一 Pool pending 被 claim 成独立运行票。
        unsafe {
            let metadata = &mut *self.metadata.get();
            assert!(matches!(metadata.pending, Pending::Pool));
            metadata.pending = Pending::None;
            core::mem::replace(&mut metadata.owner, ptr::null())
        }
    }

    /// Core 摘 queued Work 时同时摘唯一 Pool pending，票放进锁外 RemovedTicket。
    ///
    /// # Safety
    /// 调用者必须持 STATE_LOCK；queued Work 必须对应本对象当前 Pool pending。
    pub(super) unsafe fn cancel_locked(self: Pin<&Self>) -> *const Self {
        // SAFETY: Core 已在同一锁中摘 queued，运行票不在 pending 中，因此不会被取消。
        unsafe { self.claim_locked() }
    }
}

impl Drop for DelayedWork {
    /// 最后引用只检查显式停止责任；字段析构在状态锁离开以后执行，不隐等远端。
    fn drop(&mut self) {
        let _state = STATE_LOCK.lock();
        // SAFETY: 最后引用独占析构，锁仍与生产状态访问共用，未发表失败对象同样为空。
        let metadata = unsafe { &*self.metadata.get() };
        assert!(
            matches!(metadata.pending, Pending::None),
            "dropping pending delayed work"
        );
        assert!(metadata.owner.is_null(), "dropping delayed ownership");
    }
}

/// 真实 Work 运行票保活其唯一 DelayedWork，业务函数只在绑定的 worker 上执行。
fn run_work(work: Pin<&Work>) {
    let raw = work.context() as *const DelayedWork;
    // SAFETY: Core claim 已将真实 pending Arc 票移入运行拥有票，覆盖本业务函数及真实尾部。
    let owner = unsafe { Pin::new_unchecked(&*raw) };
    (owner.callback)(owner);
}

/// 粗 callback 只移交当前已到期的下一轮；独立 timer 上下文票覆盖整个函数。
fn transfer(timer: Pin<&Timer>) {
    let raw = timer.context() as *const DelayedWork;
    // SAFETY: 每次新增 timer queued 先取得真实 D raw 强票，take_due 原样转成这次运行上下文票。
    let owner = unsafe { Pin::new_unchecked(&*raw) };
    #[cfg(feature = "ktest")]
    testing::pause(raw, testing::PausePoint::BeforeTransfer);
    let mut releases = Releases::new();
    let state = STATE_LOCK.lock();
    // SAFETY: pending 与修改期限均受同一锁保护；旧已取出 callback 不会提前移交新期限。
    let due = unsafe {
        matches!((*owner.metadata.get()).pending, Pending::Delay(deadline) if deadline <= time::now())
    };
    let mut transferred = false;
    if due {
        owner.cancel_timer_locked(&mut releases);
        if owner.work().accepts_locked() {
            // SAFETY: 当前 Delay pending 的真实强票直接转为 Pool，callback 自身另持 timer 运行票。
            unsafe { (*owner.metadata.get()).pending = Pending::Pool };
            match owner.work().queue_locked() {
                Ok(true) => transferred = true,
                Ok(false) | Err(QueueError::Closed) => panic!("accepted delayed transfer refused"),
            }
        } else {
            // SAFETY: 门已关闭或同步停止；到期 Delay 不再有后续 timer 执行者，原 pending 票必须撤掉。
            // 与 stopper 共用 STATE_LOCK，只有取得 owner 的一方在锁外归还一次，运行上下文票独立。
            let pending = unsafe {
                let metadata = &mut *owner.metadata.get();
                metadata.pending = Pending::None;
                core::mem::replace(&mut metadata.owner, ptr::null())
            };
            releases.push(pending);
        }
    }

    drop(state);
    drop(releases);
    if transferred {
        owner.work().notify();
        #[cfg(feature = "ktest")]
        testing::pause(raw, testing::PausePoint::AfterTransfer);
    }

    // SAFETY: 这是本 callback 最后一次 D 访问，归还由新增 timer queued 转入的唯一运行上下文票。
    // Timer 自己的 owned running 票仍覆盖 run_callback 的真实尾部，不再读取 D 上下文。
    drop(unsafe { Arc::from_raw(raw) });
}

/// 仅观察真实生产状态并在真实移交的锁外窗口提供有限一次握手。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    use super::*;
    use core::hint::spin_loop;
    use core::sync::atomic::{AtomicUsize, Ordering};

    #[derive(Clone, Copy)]
    pub(crate) struct DelayedSnapshot {
        pub timer_pending: bool,
        pub work_pending: bool,
        pub running: bool,
        pub closed: bool,
    }

    #[derive(Clone, Copy)]
    pub(crate) enum PausePoint {
        BeforeTransfer,
        AfterTransfer,
    }

    const ARMED: usize = 1;
    const PAUSED: usize = 2;
    const RELEASED: usize = 3;
    const PAUSE_TIMEOUT_SECONDS: u64 = 5;

    static PAUSE_OWNER: AtomicUsize = AtomicUsize::new(0);
    static PAUSE_POINT: AtomicUsize = AtomicUsize::new(0);
    static PAUSE_STATE: AtomicUsize = AtomicUsize::new(0);

    /// 快照同一 Work 的真实运行状态与唯一 pending 所在阶段。
    pub(crate) fn snapshot(owner: Pin<&DelayedWork>) -> DelayedSnapshot {
        let _state = STATE_LOCK.lock();
        let work = super::super::testing::snapshot_locked(owner.work());
        // SAFETY: pending 在唯一状态锁中读取，owner 的 Pin 借用保活稳定对象。
        let pending = unsafe { (*owner.metadata.get()).pending };
        DelayedSnapshot {
            timer_pending: matches!(pending, Pending::Delay(_)),
            work_pending: matches!(pending, Pending::Pool),
            running: work.running,
            closed: work.closed,
        }
    }

    /// 只武装另一真实 CPU 的一次移交窗口，不能在本核假造 callback 暂停。
    /// 同一 fixture 的 Thread 控制者串行 configure/release/cancel；上一 callback
    /// 退出停点并经真实同步关闭或取消确认尾部结束前不得复武装。
    pub(crate) fn configure(owner: Pin<&DelayedWork>, point: PausePoint) {
        assert_ne!(
            owner.cpu(),
            crate::cpu::current().id,
            "delayed pause needs remote CPU"
        );
        assert_eq!(
            PAUSE_STATE.load(Ordering::Acquire),
            0,
            "delayed pause already armed"
        );
        PAUSE_OWNER.store(owner.owned_raw as usize, Ordering::Relaxed);
        PAUSE_POINT.store(point as usize, Ordering::Relaxed);
        PAUSE_STATE.store(ARMED, Ordering::Release);
    }

    /// 观察真实 coarse callback 已进入无锁窗口，不修改任何生产生命周期状态。
    pub(crate) fn arrived() -> bool {
        PAUSE_STATE.load(Ordering::Acquire) == PAUSED
    }

    /// 控制者确认真实同步调用未早返后，以原 2→3 握手放行 callback。
    /// Thread 控制者不得持状态锁或 callback 退出所需锁；与取消由同一 fixture 串行。
    pub(crate) fn release() {
        assert_eq!(
            PAUSE_STATE.compare_exchange(PAUSED, RELEASED, Ordering::AcqRel, Ordering::Acquire),
            Ok(PAUSED),
            "delayed callback was not paused"
        );
    }

    /// 取消未到达的武装，或用原 2→3 协议放行已暂停的真实 timer callback。
    ///
    /// 仅 Thread 控制者在无状态锁、调度锁及 callback 退出依赖锁时调用；同一 fixture
    /// 串行负责武装和清理，0/已放行的 3 幂等返回。到达 CAS 的竞争只重读状态，
    /// 不清 pending、Work/timer 票或上下文强票，也不等待 BH 或任务离栈。
    /// 放行后仍由 fixture 真实 close/join 确认尾部，完成之前不得复武装。
    pub(crate) fn cancel_pause() {
        loop {
            let state = PAUSE_STATE.load(Ordering::Acquire);
            let next = match state {
                0 | RELEASED => return,
                ARMED => 0,
                PAUSED => RELEASED,
                _ => unreachable!("invalid delayed pause state"),
            };
            if PAUSE_STATE
                .compare_exchange(state, next, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return;
            }
        }
    }

    /// 在真实 timer 上下文票保活下有限等待另一 CPU 控制者，不睡 BH 或持元数据锁。
    /// 只有原到达者清除停点身份；控制者取消时只做 CAS，避免清除下一次武装的身份。
    pub(super) fn pause(raw: *const DelayedWork, point: PausePoint) {
        if PAUSE_STATE.load(Ordering::Acquire) != ARMED
            || PAUSE_OWNER.load(Ordering::Relaxed) != raw as usize
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
            .expect("delayed pause timeout overflow");
        while PAUSE_STATE.load(Ordering::Acquire) == PAUSED {
            assert!(
                crate::arch::read_counter().wrapping_sub(start) < limit,
                "delayed callback pause timed out"
            );
            spin_loop();
        }
        PAUSE_OWNER.store(0, Ordering::Relaxed);
        PAUSE_STATE.store(0, Ordering::Release);
    }
}
