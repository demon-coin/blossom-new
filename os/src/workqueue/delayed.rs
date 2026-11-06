//! 延迟工作项（`DelayedWork`）模块。
//!
//! # 模块职责与架构定位
//! 本模块将粗粒度定时器（`TimerKind::Coarse`）与工作队列工作项（`Work`）组合为一体，
//! 支持在指定延迟时间后将任务投递到工作池的内核线程中执行。
//! - **上游调用者**：需要延迟执行可睡眠回调的内核子系统（如定时刷新、超时重传、周期性维护任务等）。
//! - **下游依赖**：`timer` 模块（提供粗粒度定时器的设定与同步取消）以及父模块 `workqueue`
//!   （提供底层 `Work` 入队、排空、取消及 `STATE_LOCK` 同步）。
//!
//! # 核心设计与关键状态机
//!
//! ## 1. 统一挂起状态机（`Pending`）与强引用所有权流转
//! 每个 `DelayedWork` 在任意时刻最多只有一个“下一轮待执行”的挂起状态（`Metadata::pending`），
//! 定时器等待阶段（`Delay`）与工作池排队阶段（`Pool`）共享这唯一一个挂起槽位：
//!
//! ```text
//!   [Pending::None] (owner = null)
//!        │
//!        ├── queue/modify(delay > 0) ──► 1. 为 Timer 回调增加一份 Arc<DelayedWork> 强引用并 arm(deadline)
//!        │                               2. 为 Pending 状态增加一份 Arc<DelayedWork> 强引用存入 metadata.owner
//!        ▼
//!   [Pending::Delay(deadline)] (owner = raw)
//!        │
//!        ├── 定时器到期执行 transfer() 或调用 flush() 提前移交：
//!        │     将 metadata.pending 原地切换为 Pending::Pool，复用同一个 metadata.owner 强引用，
//!        │     并调用内层 work.queue_locked() 入队（transfer 自身持有的 Timer 上下文强引用在函数末尾独立释放）
//!        ▼
//!   [Pending::Pool] (owner = raw)
//!        │
//!        ├── Worker 线程 claim_locked() 领取工作项（或 cancel_locked() 取消排队）：
//!        │     将 metadata.pending 重置为 Pending::None，取出 metadata.owner 强引用装入 RemovedTicket.delayed，
//!        │     待业务回调 run_work() 彻底执行完毕（或取消完成）后，在 STATE_LOCK 锁外释放该强引用
//!        ▼
//!   [Pending::None] (owner = null)
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **停止与排空串行睡眠锁**：`Work::stop_serial`（通过 `work.lock_stop_serial()` 或 `work.begin_stop()` 获取）。
//! 2. **全局工作队列自旋锁 `STATE_LOCK`**：保护 `DelayedWork::metadata` 与内层 `WorkState` 的原子状态转换。
//! 3. **定时器内部锁**：在持有 `STATE_LOCK` 期间可调用非阻塞的 `timer.arm()` 与 `timer.cancel_pending()`；
//!    但阻塞等待定时器回调退出的 `timer.cancel_sync()` 以及所有 `Releases` 中的 `Arc` 强引用析构，**必须**在释放 `STATE_LOCK` 之后执行。

use alloc::sync::Arc;
use core::alloc::AllocError;
use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::ptr;

use crate::time::{self, Duration, MonotonicInstant};
use crate::timer::{ArmError, Timer, TimerKind};

use super::{QueueError, RemovedTicket, Work, WorkQueue, STATE_LOCK};

// 单次调度或取消操作在持锁期间最多摘除 3 份 DelayedWork 强引用（旧 pending、被替换的旧 timer、以及未能采用的候选 timer）。
const RELEASE_CAPACITY: usize = 3;

/// 调度或修改延迟工作项时可能返回的错误。
///
/// 当返回错误时，工作项先前已存在的挂起状态（若有）保持完整不变。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DelayedError {
    /// 延迟工作项或所属工作域已永久关闭，拒绝新的调度请求。
    Closed,
    /// 请求的延迟时长与当前时间相加后超出了单调时钟可表示的范围。
    OutOfRange,
}

/// `DelayedWork` 当前下一轮任务的挂起阶段状态。
#[derive(Clone, Copy)]
enum Pending {
    /// 当前没有任何挂起的延迟定时或工作池排队任务。
    None,
    /// 任务正处于粗粒度定时器倒计时阶段，记录其到期时间点 `MonotonicInstant`。
    Delay(MonotonicInstant),
    /// 定时器已到期（或以零延迟直接提交），任务已移交至内层 `Work` 的工作池队列中等待领取。
    Pool,
}

/// 受全局自旋锁 `STATE_LOCK` 保护的延迟工作项可变元数据。
struct Metadata {
    /// 当前下一轮任务所处的挂起阶段。
    pending: Pending,
    /// 与非空 `pending` 状态一一对应的 `Arc<DelayedWork>` 强引用裸指针；当 `pending` 为 `None` 时为 `null`。
    owner: *const DelayedWork,
}

/// 绑定固定工作域、目标 CPU 与业务回调函数的可延迟工作项。
///
/// 销毁前调用者必须显式履行停止责任（确保无挂起任务或调用 [`DelayedWork::close_sync`]）。
/// 同步等待接口（`flush`、`cancel_sync`、`close_sync`）严禁在自身业务回调内调用，且外部业务上下文须保持有效直至 `close_sync` 返回。
pub struct DelayedWork {
    /// 内层固定绑定的工作项（构造完成后始终为 `Some`）。
    work: Option<Pin<Arc<Work>>>,
    /// 内层固定绑定的粗粒度定时器（构造完成后始终为 `Some`）。
    timer: Option<Pin<Arc<Timer>>>,
    /// 工作项被 Worker 线程领取后执行的业务回调函数。
    callback: fn(Pin<&Self>),
    /// 构造时传入的调用方自定义上下文标量。
    context: usize,
    /// 指向当前 `DelayedWork` 自身 `Arc` 负载的裸指针，用于在调度时通过 `Arc::increment_strong_count` 增加异步保活强引用。
    owned_raw: *const Self,
    /// 受 `STATE_LOCK` 保护的挂起阶段与强引用指针元数据。
    metadata: UnsafeCell<Metadata>,
    /// 标记为 `!Unpin`，因为内部 `Work` 和 `Timer` 回调上下文均保存了指向本对象的自引用裸指针 `owned_raw`。
    _pinned: PhantomPinned,
}

// SAFETY: DelayedWork 构造完成后除 metadata 外的所有字段均不可变；metadata 完全由全局自旋锁 STATE_LOCK 保护，
// 且业务回调函数由内层 Work 串行化执行，因此可安全跨线程转移所有权。
unsafe impl Send for DelayedWork {}
// SAFETY: 共享引用下的所有可变状态访问均通过 STATE_LOCK 同步，定时器回调与工作池回调各自持有独立的 Arc 强引用保活。
unsafe impl Sync for DelayedWork {}

/// 在持有 `STATE_LOCK` 期间收集的待释放强引用集合，确保所有 `Arc` 析构与唤醒操作均延迟到释放自旋锁之后执行。
struct Releases {
    /// 待调用 `Arc::from_raw` 释放的 `DelayedWork` 强引用指针数组。
    raw: [*const DelayedWork; RELEASE_CAPACITY],
    /// 当前已推入 `raw` 数组的有效指针数量（不超过 `RELEASE_CAPACITY`）。
    count: usize,
    /// 从内层 `Work` 队列中摘除的排队票据守卫（若有）。
    work: Option<RemovedTicket>,
}

impl Releases {
    /// 创建一个空的待释放强引用收集器。
    fn new() -> Self {
        Self {
            raw: [ptr::null(); RELEASE_CAPACITY],
            count: 0,
            work: None,
        }
    }

    /// 记录一份需要在离开 `STATE_LOCK` 后释放的 `Arc<DelayedWork>` 强引用裸指针（忽略空指针）。
    fn push(&mut self, raw: *const DelayedWork) {
        if !raw.is_null() {
            self.raw[self.count] = raw;
            self.count += 1;
        }
    }
}

impl Drop for Releases {
    /// 在释放 `STATE_LOCK` 之后，逐一归还收集到的 `DelayedWork` 强引用及内层 `Work` 票据。
    fn drop(&mut self) {
        for index in 0..self.count {
            // SAFETY: push 仅存入由 Arc::increment_strong_count(owned_raw) 真实创建的强引用指针，且每个槽位仅在此处释放一次。
            drop(unsafe { Arc::from_raw(self.raw[index]) });
        }
        drop(self.work.take());
    }
}

impl DelayedWork {
    /// 创建一个绑定到指定工作队列与目标 CPU 的延迟工作项（`Pin<Arc<DelayedWork>>`）。
    ///
    /// 先完成外层 `DelayedWork`、内层 `Work` 和粗粒度 `Timer` 的可失败内存分配，最后再建立自引用关联；
    /// 若任何一步分配失败，已分配的局部对象会自动析构清理，不会残留半初始化注册。
    pub fn try_new(
        queue: &WorkQueue,
        cpu: usize,
        callback: fn(Pin<&Self>),
        context: usize,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        // Step 1: 分配外层 DelayedWork 的 Arc 存储，获取其固定的堆内存裸指针 raw。
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
        // SAFETY: raw 刚由上一行 Arc::into_raw 转出，此处立即恢复为局部 Arc 所有权，确保后续步骤若返回错误时能正常释放内存。
        let owner = unsafe { Arc::from_raw(raw) };
        // Step 2: 分配内层 Work 与绑定相同目标 CPU 的粗粒度 Timer，将 raw 作为回调上下文传入。
        let work = Work::try_new(queue, cpu, run_work, raw as usize)?;
        let timer = Timer::try_new(work.cpu(), TimerKind::Coarse, transfer, raw as usize)?;
        // Step 3: 填充自引用指针与内层组件，固定内存位置并关联内层 Work 的 delayed_owner。
        // SAFETY: owner 尚未对外发布且无任何其他强/弱引用，当前线程独占该堆内存；在此写入初始化字段不会移动任何已固定的数据。
        unsafe {
            (*raw.cast_mut()).owned_raw = raw;
            (*raw.cast_mut()).work = Some(work);
            (*raw.cast_mut()).timer = Some(timer);
        }
        // SAFETY: DelayedWork 位于独立的 Arc 堆分配中，后续永不移出其底层值，且公开 API 不暴露 &mut Self。
        let owner = unsafe { Pin::new_unchecked(owner) };
        owner.work().set_delayed_owner(raw);
        Ok(owner)
    }

    /// 返回构造时传入的调用方上下文标量值。
    pub fn context(&self) -> usize {
        self.context
    }

    /// 返回内层 `Work` 与粗粒度 `Timer` 共同绑定的实际 CPU 编号（若所属工作域为严格有序域则已归一化为 CPU0）。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "延后工作测试核对 Timer 与 Work 的共同绑定 CPU")
    )]
    pub fn cpu(&self) -> usize {
        self.work().cpu()
    }

    /// 当且仅当当前不存在未执行的挂起任务（`Pending::None`）时，安排在 `delay` 之后执行工作项。
    ///
    /// - 若已有挂起任务（处于定时器等待或工作池排队阶段）或正处于同步取消期间，返回 `Ok(false)` 且保持原计划不变；
    /// - 返回 `Ok(false)` 仅表示未新增调度，不代表当前正在运行的业务回调已经结束。
    pub fn queue(self: Pin<&Self>, delay: Duration) -> Result<bool, DelayedError> {
        self.schedule(delay, false)
    }

    /// 原子地设置或替换下一轮延迟执行计划（无论当前处于 `Delay` 还是 `Pool` 挂起阶段）。
    ///
    /// - 成功时返回 `Ok(true)` 表示替换了原先已存在的挂起计划，返回 `Ok(false)` 表示原先无挂起计划；
    /// - 若因工作项关闭或时间溢出而返回 `Err`，原先已接受的挂起计划保持完整不变。
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

    /// 同步取消当前的延迟定时或排队任务，并阻塞等待正在运行的定时器回调与业务回调彻底执行完毕。
    ///
    /// 取消完成后，同一个 `DelayedWork` 实例仍可继续调用 `queue` 或 `modify` 复用。
    pub fn cancel_sync(self: Pin<&Self>) -> bool {
        self.stop(false)
    }

    /// 永久关闭延迟工作项，撤销所有挂起任务并阻塞等待定时器回调与业务回调彻底退出。
    ///
    /// `Drop` 不会隐式等待异步回调结束，持有者必须在销毁前显式调用本方法或确保无活跃任务。
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

    /// 若当前存在尚未到期的定时器挂起任务（`Pending::Delay`），立即将其移交至工作池入队，并阻塞等待本次捕获边界内的工作项执行完毕。
    ///
    /// 在本次 `flush` 捕获边界之后新提交的任务不会延长本次等待；严禁在自身业务回调函数中同步调用。
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
        // Step 1: 持 stop_serial 与 STATE_LOCK 检查当前挂起阶段，若处于 Delay 阶段则立即取消定时器并转入工作池队列。
        let state = STATE_LOCK.lock();
        // SAFETY: 已持有全局自旋锁 STATE_LOCK，安全读取 metadata.pending 并捕获内层 Work 的票据边界。
        let delayed = unsafe { matches!((*self.metadata.get()).pending, Pending::Delay(_)) };
        if delayed && self.work().accepts_locked() {
            self.cancel_timer_locked(&mut releases);
            // SAFETY: 已持有 STATE_LOCK；直接复用原 Delay 阶段持有的 metadata.owner 强引用，将其状态切换为 Pool。
            unsafe { (*self.metadata.get()).pending = Pending::Pool };
            assert!(self
                .work()
                .queue_locked()
                .expect("accepted delayed flush closed"));
        }
        // Step 2: 若内层工作项或所属域已关闭，清理残留的 Delay 挂起状态并收集强引用待锁外释放。
        let discard = delayed && self.work().is_closed_locked();
        if discard {
            self.cancel_timer_locked(&mut releases);
            // SAFETY: 已持有 STATE_LOCK；工作项已关闭故不再入队，取出 metadata.owner 强引用交由 releases 在锁外释放。
            let raw = unsafe {
                let metadata = &mut *self.metadata.get();
                metadata.pending = Pending::None;
                core::mem::replace(&mut metadata.owner, ptr::null())
            };
            releases.push(raw);
        }
        // Step 3: 捕获内层 Work 当前的排队与运行票据边界，随后释放所有锁并等待该边界内的票据执行完毕。
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

    /// 获取内层固定绑定的 `Work` 引用。
    fn work(&self) -> Pin<&Work> {
        self.work
            .as_ref()
            .expect("unpublished delayed work")
            .as_ref()
    }

    /// 获取内层固定绑定的粗粒度 `Timer` 引用。
    fn timer(&self) -> Pin<&Timer> {
        self.timer
            .as_ref()
            .expect("unpublished delayed timer")
            .as_ref()
    }

    /// 在同一把 `STATE_LOCK` 临界区内完成准入检查、到期时间计算、旧挂起任务撤销以及新挂起状态发布。
    fn schedule(self: Pin<&Self>, delay: Duration, modify: bool) -> Result<bool, DelayedError> {
        let mut releases = Releases::new();
        let state = STATE_LOCK.lock();
        // Step 1: 检查工作项是否已关闭或正在同步停止，并判断非 modify 模式下是否已有挂起任务。
        if self.work().is_closed_locked() {
            return Err(DelayedError::Closed);
        }
        if !self.work().accepts_locked() {
            return Ok(false);
        }
        // SAFETY: 已持有 STATE_LOCK，且 self: Pin<&Self> 保证当前 DelayedWork 实例存活，可安全读取 metadata.pending。
        let pending = unsafe { (*self.metadata.get()).pending };
        let existed = !matches!(pending, Pending::None);
        if existed && !modify {
            return Ok(false);
        }

        // Step 2: 计算到期时间点；若 delay > 0，先为定时器回调预增一份强引用并武装定时器。
        let deadline = time::now()
            .checked_add(delay)
            .ok_or(DelayedError::OutOfRange)?;
        if !delay.is_zero() {
            // SAFETY: self.owned_raw 指向当前有效的 DelayedWork Arc 负载；在武装定时器前预先递增一次强引用作为 Timer 回调的上下文所有权。
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

        // Step 3: 撤销先前已存在的 Pool 排队票据或 Delay 挂起强引用（放入 releases 待锁外释放）。
        match pending {
            Pending::Pool => releases.work = self.work().cancel_queued_locked(),
            Pending::Delay(_) => {
                // SAFETY: 已持有 STATE_LOCK，将旧 Delay 阶段持有的唯一 owner 强引用取出并交由 releases 在锁外安全析构。
                let raw = unsafe {
                    let metadata = &mut *self.metadata.get();
                    metadata.pending = Pending::None;
                    core::mem::replace(&mut metadata.owner, ptr::null())
                };
                releases.push(raw);
            }
            Pending::None => {}
        }

        // Step 4: 为新的挂起状态增加一份强引用并写入 metadata；若 delay == 0 则立即推入内层 Work 队列。
        // SAFETY: 已持有 STATE_LOCK，self.owned_raw 有效；此处递增的强引用由 metadata.owner 持有，与正在运行的回调互不干扰。
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

    /// 在持有 `STATE_LOCK` 的前提下尝试撤销内层定时器中尚未到期的排队状态；若撤销成功，将其关联的上下文强引用推入 `releases` 待锁外释放。
    fn cancel_timer_locked(self: Pin<&Self>, releases: &mut Releases) {
        if self.timer().cancel_pending() {
            releases.push(self.owned_raw);
        }
    }

    /// 执行同步取消（`close = false`）或永久关闭（`close = true`）的完整流程。
    ///
    /// 依次通过内层 `Work::begin_stop` 封锁新提交、在 `STATE_LOCK` 下撤销定时器与 `Delay` 挂起票据、
    /// 在锁外等待定时器回调（`timer.cancel_sync()`）与工作项业务回调（`guard.finish()`）彻底退出。
    fn stop(self: Pin<&Self>, close: bool) -> bool {
        let guard = self.work().begin_stop(close);
        let mut releases = Releases::new();
        let state = STATE_LOCK.lock();
        // SAFETY: begin_stop 已置位 stopping 阻止新任务提交，且当前已持有 STATE_LOCK，可安全检查并摘除 Delay 挂起状态。
        let delayed = unsafe { matches!((*self.metadata.get()).pending, Pending::Delay(_)) };
        self.cancel_timer_locked(&mut releases);
        if delayed {
            // SAFETY: 已持有 STATE_LOCK；正在运行的定时器回调拥有各自独立的上下文强引用，此处仅摘除 metadata.owner 中的挂起强引用。
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

    /// 检查当前 `DelayedWork` 是否存在尚未执行完毕的挂起状态（`Delay` 或 `Pool`）。
    ///
    /// 供外层 `Work::begin_stop` 在持有 `STATE_LOCK` 封锁提交入口的同一临界区内调用，准确记录本次取消是否影响了挂起任务。
    ///
    /// # Safety
    /// 调用者必须已持有全局自旋锁 `STATE_LOCK`，且当前 `DelayedWork` 实例必须由调用者或异步强引用保持有效。
    pub(super) unsafe fn has_pending_locked(self: Pin<&Self>) -> bool {
        // SAFETY: 调用者契约保证已持有 STATE_LOCK 且 self 指向有效的 DelayedWork，可安全读取 metadata.pending。
        unsafe { !matches!((*self.metadata.get()).pending, Pending::None) }
    }

    /// 当内层 `Work` 被 Worker 线程成功领取（`claim`）时，清除 `Pending::Pool` 状态并将对应的 `Arc<DelayedWork>` 强引用移交给运行期守卫。
    ///
    /// # Safety
    /// 调用者必须已持有全局自旋锁 `STATE_LOCK`，且内层 `Work` 必须正处于从排队转为运行的过程中；返回的裸指针必须在回调彻底结束后于锁外通过 `Arc::from_raw` 释放。
    pub(super) unsafe fn claim_locked(self: Pin<&Self>) -> *const Self {
        // SAFETY: 调用者契约保证已持有 STATE_LOCK；此时 metadata.pending 必定为 Pool，将其重置为 None 并移交唯一的 owner 强引用指针。
        unsafe {
            let metadata = &mut *self.metadata.get();
            assert!(matches!(metadata.pending, Pending::Pool));
            metadata.pending = Pending::None;
            core::mem::replace(&mut metadata.owner, ptr::null())
        }
    }

    /// 当内层排队中的 `Work` 被取消摘除时，同步清除 `Pending::Pool` 状态并返回待在锁外释放的 `Arc<DelayedWork>` 强引用指针。
    ///
    /// # Safety
    /// 调用者必须已持有全局自旋锁 `STATE_LOCK`，且内层 `Work` 当前确实处于排队状态（对应本对象的 `Pending::Pool`）。
    pub(super) unsafe fn cancel_locked(self: Pin<&Self>) -> *const Self {
        // SAFETY: 调用者契约保证已持有 STATE_LOCK 且内层排队 Work 正在被摘除，直接复用 claim_locked 取出 Pool 阶段的强引用。
        unsafe { self.claim_locked() }
    }
}

impl Drop for DelayedWork {
    /// 析构延迟工作项时断言无任何未完成的挂起状态或强引用残留。
    fn drop(&mut self) {
        let _state = STATE_LOCK.lock();
        // SAFETY: Drop 期间当前线程独占该对象，且已持有 STATE_LOCK 读取 metadata。
        let metadata = unsafe { &*self.metadata.get() };
        assert!(
            matches!(metadata.pending, Pending::None),
            "dropping pending delayed work"
        );
        assert!(metadata.owner.is_null(), "dropping delayed ownership");
    }
}

/// 内层 `Work` 的跳板回调函数：将上下文还原为 `Pin<&DelayedWork>` 并调用用户注册的业务回调。
fn run_work(work: Pin<&Work>) {
    let raw = work.context() as *const DelayedWork;
    // SAFETY: Worker 线程在 claim_locked 时已将 Pool 阶段的 Arc<DelayedWork> 强引用移入当前运行票据（RemovedTicket.delayed），
    // 该强引用在当前函数返回且工作项彻底完成前不会被释放，且 DelayedWork 自创建起固定在堆上不移动。
    let owner = unsafe { Pin::new_unchecked(&*raw) };
    (owner.callback)(owner);
}

/// 内层粗粒度 `Timer` 的到期回调函数：在锁下检查当前 `Pending::Delay` 是否已到期，若是则将其移交至内层 `Work` 工作池队列。
fn transfer(timer: Pin<&Timer>) {
    let raw = timer.context() as *const DelayedWork;
    // SAFETY: 每次调用 timer.arm() 入队时都会预先增加一份 Arc<DelayedWork> 强引用，定时器到期取出回调时将该强引用移交给本次
    // transfer 调用独占持有，直到本函数末尾最后一行才通过 Arc::from_raw 释放，保证整个函数执行期间 raw 始终有效。
    let owner = unsafe { Pin::new_unchecked(&*raw) };
    #[cfg(feature = "ktest")]
    testing::pause(raw, testing::PausePoint::BeforeTransfer);
    let mut releases = Releases::new();
    let state = STATE_LOCK.lock();
    // SAFETY: 已持有 STATE_LOCK，metadata.pending 与到期时间均受该锁保护；即使存在旧定时器回调与新 modify 并发，此处也会重新校验最新 deadline。
    let due = unsafe {
        matches!((*owner.metadata.get()).pending, Pending::Delay(deadline) if deadline <= time::now())
    };
    let mut transferred = false;
    if due {
        owner.cancel_timer_locked(&mut releases);
        if owner.work().accepts_locked() {
            // SAFETY: 已持有 STATE_LOCK；直接将原 Delay 挂起状态的强引用转为 Pool 阶段，本次定时器回调自身的上下文强引用在函数末尾独立释放。
            unsafe { (*owner.metadata.get()).pending = Pending::Pool };
            match owner.work().queue_locked() {
                Ok(true) => transferred = true,
                Ok(false) | Err(QueueError::Closed) => panic!("accepted delayed transfer refused"),
            }
        } else {
            // SAFETY: 已持有 STATE_LOCK；当前工作项已关闭或正在同步停止，无法转入工作池，因此取出 Delay 阶段的挂起强引用放入 releases 待锁外释放。
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

    // SAFETY: 此处之后本函数不再访问 owner；将武装定时器时预先增加并移交给本回调的唯一一份上下文 Arc<DelayedWork> 强引用在此恢复并释放。
    drop(unsafe { Arc::from_raw(raw) });
}

/// 延迟工作项内核测试辅助模块，提供状态快照观察与定时器移交窗口的同步停点握手（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    use super::*;
    use core::hint::spin_loop;
    use core::sync::atomic::{AtomicUsize, Ordering};

    /// 延迟工作项在某一时刻的综合状态快照（仅用于内核测试）。
    #[derive(Clone, Copy)]
    pub(crate) struct DelayedSnapshot {
        /// 是否正处于粗粒度定时器倒计时挂起阶段（`Pending::Delay`）。
        pub timer_pending: bool,
        /// 是否已移交至工作池处于排队等待阶段（`Pending::Pool`）。
        pub work_pending: bool,
        /// 业务回调函数是否正在 Worker 线程上运行。
        pub running: bool,
        /// 工作项或所属工作域是否已永久关闭。
        pub closed: bool,
    }

    /// 定时器到期移交回调（`transfer`）中的测试停点位置（仅用于内核测试）。
    #[derive(Clone, Copy)]
    pub(crate) enum PausePoint {
        /// 在获取 `STATE_LOCK` 执行移交之前暂停。
        BeforeTransfer,
        /// 在完成向工作池移交并调用 `notify` 之后、释放定时器上下文强引用之前暂停。
        AfterTransfer,
    }

    const ARMED: usize = 1;
    const PAUSED: usize = 2;
    const RELEASED: usize = 3;
    const PAUSE_TIMEOUT_SECONDS: u64 = 5;

    static PAUSE_OWNER: AtomicUsize = AtomicUsize::new(0);
    static PAUSE_POINT: AtomicUsize = AtomicUsize::new(0);
    static PAUSE_STATE: AtomicUsize = AtomicUsize::new(0);

    /// 获取 `STATE_LOCK` 并读取指定 `DelayedWork` 的完整状态快照（仅用于内核测试）。
    pub(crate) fn snapshot(owner: Pin<&DelayedWork>) -> DelayedSnapshot {
        let _state = STATE_LOCK.lock();
        let work = super::super::testing::snapshot_locked(owner.work());
        // SAFETY: 已持有全局自旋锁 STATE_LOCK，且 owner: Pin<&DelayedWork> 保证对象有效，可安全读取 metadata.pending。
        let pending = unsafe { (*owner.metadata.get()).pending };
        DelayedSnapshot {
            timer_pending: matches!(pending, Pending::Delay(_)),
            work_pending: matches!(pending, Pending::Pool),
            running: work.running,
            closed: work.closed,
        }
    }

    /// 为绑定在远端 CPU 上的指定 `DelayedWork` 武装一次定时器移交停点（仅用于内核测试）。
    ///
    /// 调用线程不得与目标 `DelayedWork` 位于同一 CPU，以免在软中断/定时器上下文中自旋死锁；
    /// 同一测试夹具必须等待上一轮暂停的回调完全退出并完成同步清理后，方可重新武装。
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

    /// 检查目标定时器移交回调是否已到达并暂停在预设的无锁停点窗口中（仅用于内核测试）。
    pub(crate) fn arrived() -> bool {
        PAUSE_STATE.load(Ordering::Acquire) == PAUSED
    }

    /// 放行当前正暂停在停点处的定时器移交回调（将状态从 `PAUSED` 切换为 `RELEASED`，仅用于内核测试）。
    ///
    /// 调用者线程在调用时不得持有 `STATE_LOCK` 或阻碍回调退出的锁。
    pub(crate) fn release() {
        assert_eq!(
            PAUSE_STATE.compare_exchange(PAUSED, RELEASED, Ordering::AcqRel, Ordering::Acquire),
            Ok(PAUSED),
            "delayed callback was not paused"
        );
    }

    /// 取消尚未触发的停点武装，或放行已经到达停点暂停的定时器回调（仅用于内核测试）。
    ///
    /// 供测试控制线程在未持有自旋锁时幂等调用；放行后调用者仍需通过正常的 `cancel_sync` 或 `close_sync` 等待回调彻底退出。
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

    /// 若当前 `DelayedWork` 与停点位置匹配已武装的设置，则在锁外自旋等待测试控制线程放行（仅用于内核测试）。
    ///
    /// 仅由成功将状态从 `ARMED` 切换为 `PAUSED` 的到达线程在退出前负责清零 `PAUSE_OWNER` 与 `PAUSE_STATE`。
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
