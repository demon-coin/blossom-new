//! 内核软件定时器核心调度模块。
//!
//! # 模块职责与架构定位
//! 本模块向上层（任务睡眠 `timer::sleep`、网络超时、设备驱动轮询与内核自检）提供统一的
//! 软件定时器（[`Timer`]）注册、取消与同步停止接口；向下层通过 `time::reprogram_current`
//! 及跨核重编程 IPI（`cpu::KICK_TIMER_REPROGRAM`）驱动底层硬件时钟事件，并在时钟硬中断
//! （[`handle_interrupt`]）与每核常驻底半部服务线程（[`service`]）中分发到期回调。
//!
//! # 核心设计与关键状态机
//!
//! ## 1. 双轨定时引擎（Per-CPU [`TimerBase`]）
//! 每个逻辑 CPU 在静态 `CpuLocal` 中拥有独立的 [`TimerBase`]，内含两套侵入式定时容器：
//! - **高精度定时器（[`TimerKind::Precise`]，基于 [`queue::Queue`] 侵入式红黑树）**：
//!   按纳秒级单调截止时间（[`MonotonicInstant`]）精确排序，缓存最左（最早到期）节点并驱动硬件比较器。
//!   到期后直接在时钟硬中断上下文中执行回调（单次中断最多执行 [`HARD_BUDGET`] 个）。
//! - **粗粒度定时器（[`TimerKind::Coarse`]，基于 [`wheel::Wheel`] 八层非级联时间轮）**：
//!   按 10ms（`HZ = 100`）基础粒度向上取整入桶。时钟硬中断仅置位 `service_pending` 并唤醒本核
//!   常驻服务线程，在关抢占底半部（[`cpu::BottomHalfGuard`]）中按批（每批最多 [`BH_BUDGET`] 个）执行回调。
//!
//! ## 2. 节点内存所有权模型
//! - **引用计数堆所有权模式（Owned，[`Timer::try_new`]）**：
//!   节点分配在 `Arc<Timer>` 堆内存中，`owned_raw` 保存构造时由 `Arc::into_raw` 导出的原始指针。
//!   当定时器入队（`queued = true`）时调用 `Arc::increment_strong_count` 增加一份队列持有的强引用
//!   （`queued_owned = true`）；出队进入回调执行（`running = true`）时，该强引用移交给执行上下文，
//!   直到被取消或回调彻底执行完毕后，在自旋锁外通过 `Arc::from_raw` 释放，保证队列持有与回调执行期间内存有效。
//! - **调用方固定存储借用模式（Borrowed，[`Timer::new`] + [`Timer::arm_borrowed`]）**：
//!   `owned_raw` 为空指针，节点可位于栈帧或静态内存中。调用方必须在移动、释放内存或退出栈帧前调用
//!   [`Timer::cancel_sync`] 或 [`Timer::close_sync`]，同步等待正在执行的回调彻底退出。
//!
//! ## 3. 定时器节点生命周期状态机
//! `Node` 中的 `queued`（已入队等待到期）与 `running`（已出队正在执行回调）是两个正交状态位，
//! 允许回调函数在执行自身期间再次调用 `arm` 登记下一轮超时：
//! ```text
//! +-------------------+     arm / arm_borrowed      +-------------------+
//! |  Idle             | --------------------------> |  Queued           |
//! |  queued = false   |                             |  queued = true    |
//! |  running = false  | <-------------------------- |  running = false  |
//! +-------------------+    cancel / try_cancel      +-------------------+
//!           ^                                                 |
//!           |                                                 | take_due (到期摘链)
//!           | run_callback 退出                                v
//!           | (清 running 并唤醒 finished)           +-------------------+
//!           +-------------------------------------- |  Running          |
//!           |                                       |  queued = false   |
//!           |                                       |  running = true   |
//!           |                                       +-------------------+
//!           |                                                 |
//!           |     回调退出时若 queued 仍为 true，              | 回调内重调 arm()
//!           |     则保留在队列中等待下一轮到期                  v
//!           |                                       +-------------------+
//!           |   cancel_pending / stop_sync 摘链     |  Running + Queued |
//!           +-------------------------------------- |  queued = true    |
//!                                                   |  running = true   |
//!                                                   +-------------------+
//! ```
//! 在任意状态下调用 [`Timer::close_sync`] 会将 `closed` 永久置为 `true`，此后一切 `arm` 均返回
//! `Err(ArmError::Closed)`。
//!
//! # 锁获取顺序与并发契约
//! - 每个 CPU 的 `TimerBase::state` 自旋锁（`SpinLock<BaseState>`）保护该核红黑树、时间轮以及归属该核的
//!   所有 `Timer` 内部 `Node` 字段。
//! - **严禁在持有 `TimerBase::state` 自旋锁期间执行以下操作**：
//!   1. **执行定时器回调 `callback`**：回调内部可能再次调用 `arm` 或 `cancel_pending` 获取同一把 `base.state` 锁，
//!      持锁回调会导致自旋锁重入死锁。
//!   2. **释放 `Arc<Timer>`（[`release_owned`]）**：当释放最后一个强引用触发 [`Timer::drop`] 时，`drop` 会获取
//!      `base.state` 锁断言节点已停止，持锁释放会导致自旋锁重入死锁。
//!   3. **调用 [`changed`] 触发重编程**：`changed` 在本核上会调用 `time::reprogram_current()`，进而调用
//!      [`with_next_deadline`] 重新获取本核 `base.state` 锁。
//!   4. **睡眠或阻塞等待**：所有同步等待（[`Timer::stop_sync`]、[`service`]）均遵循“先注册 `WaitNode` ->
//!      持 `base.state` 锁复查状态 -> 释放 `base.state` 锁 -> 调用 `park()` 阻塞”的顺序，防止唤醒丢失与死锁。
//! - **借用节点退栈安全契约**：[`run_callback`] 在持有 `base.state` 锁时将 `node.running` 置为 `false`，
//!   随后释放锁并唤醒静态 `base.finished` 等待队列，此后执行路径**绝不再解引用 `timer` 指针**。因此栈上借用
//!   `Timer` 的任务一旦从 `stop_sync` 观察到 `running == false` 并返回，即可安全弹出栈帧而不会引发悬垂指针访问。

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

/// 单次时钟硬中断最多连续处理的到期高精度定时器（[`TimerKind::Precise`]）回调数量上限。
const HARD_BUDGET: usize = 32;
/// 底半部服务线程单批最多连续处理的到期粗粒度定时器（[`TimerKind::Coarse`]）回调数量上限。
const BH_BUDGET: usize = 32;
/// 标记每核粗粒度定时器底半部服务线程是否已由 BSP 完成全局初始化。
static SERVICES_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// 软件定时器的精度与执行上下文分类。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerKind {
    /// 高精度定时器：纳入每核红黑树，按纳秒级绝对截止时间排序，到期后在时钟硬中断上下文中执行回调。
    Precise,
    /// 粗粒度定时器：纳入每核八层时间轮（10ms 粒度），到期后唤醒每核常驻服务线程在关抢占底半部中执行回调。
    Coarse,
}

/// 注册（Arm）定时器失败的错误原因。
///
/// 当返回此错误时，定时器的原有入队状态与截止时间保持不变。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmError {
    /// 定时器已通过 [`Timer::close_sync`] 永久关闭，不再接受任何新的超时注册。
    Closed,
    /// 请求的截止时间超出了粗粒度时间轮所能表示的最大跨度（[`wheel::MAX_TIMEOUT_TICKS`]）或发生纳秒乘法溢出。
    OutOfRange,
}

/// 非阻塞取消（[`Timer::try_cancel`]）的执行结果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CancelResult {
    /// 定时器当前既未在队列中等待，也未在执行回调，无需执行取消操作。
    Idle,
    /// 定时器原本在队列中等待且未处于回调执行态，已成功从队列中摘除。
    Cancelled,
    /// 定时器当前正在执行回调（`running == true`）；非阻塞取消不会等待回调结束，也不会摘除回调期间重注册的下一轮超时。
    Busy,
}

/// 定时器内嵌的队列链接与可变生命周期状态。
///
/// 该结构体的所有字段均由所属 CPU 的 `TimerBase::state` 自旋锁排他保护。
pub(super) struct Node {
    /// 高精度定时器在侵入式红黑树（[`queue::Queue`]）中的父子与颜色链接。
    rb: queue::Links,
    /// 粗粒度定时器在侵入式时间轮（[`wheel::Wheel`]）中的双向桶链表节点与生效 tick。
    wheel: wheel::Links,
    /// 当前已注册超时的单调时钟绝对截止时间（单位：纳秒）。
    expires: MonotonicInstant,
    /// 标记该节点当前是否挂在所属 CPU 的红黑树或时间轮队列中等待到期。
    queued: bool,
    /// 标记该节点是否已被 [`take_due`] 摘出并正在 [`run_callback`] 中执行回调函数。
    running: bool,
    /// 标记该节点是否已被 [`Timer::close_sync`] 永久关闭（关闭后拒绝一切新的 `arm` 请求）。
    closed: bool,
    /// 当节点处于 `queued` 状态且为堆分配的 `Arc<Timer>` 时，标记队列当前是否持有一份独立的 `Arc` 强引用。
    queued_owned: bool,
}

/// 内核软件定时器节点。
///
/// 每个 `Timer` 在构造时固定绑定目标逻辑 CPU、精度类型（[`TimerKind`]）、回调函数与标量上下文。
/// 一旦定时器被注册入队，其内存地址在从队列摘除且回调彻底退出之前必须保持固定不变（`!Unpin`）。
///
/// # 回调函数约束
/// 回调函数 `fn(Pin<&Timer>)` 运行在硬中断或关抢占底半部上下文中，**严禁睡眠、修改中断开关状态
/// 或调用 [`Timer::cancel_sync`] / [`Timer::close_sync`] 同步等待自身**。若 `context` 存放了指针，
/// 调用方必须保证目标内存在回调彻底退出前始终有效。
pub struct Timer {
    /// 定时器固定归属的逻辑 CPU 编号（决定由哪个核的 `TimerBase` 管理与触发）。
    cpu: usize,
    /// 定时器的精度与执行上下文类型（高精度硬中断 [`TimerKind::Precise`] 或粗粒度底半部 [`TimerKind::Coarse`]）。
    kind: TimerKind,
    /// 定时器到期时调用的回调函数指针。
    callback: fn(Pin<&Timer>),
    /// 构造时传入的无类型标量上下文（如 `Waker` 或业务结构体的指针地址），可通过 [`Timer::context`] 读取。
    context: usize,
    /// 受所属 CPU 的 `TimerBase::state` 自旋锁保护的侵入式容器链接与生命周期状态。
    pub(super) node: UnsafeCell<Node>,
    /// 若由 [`Timer::try_new`] 分配在 `Arc` 堆中，保存构造时 `Arc::into_raw` 导出的原始指针以维护引用计数；
    /// 若由 [`Timer::new`] 构造为借用节点，则为空指针（`null`）。
    owned_raw: *const Self,
    /// 标记类型不可移动（`!Unpin`），确保已入队节点的侵入式链表指针不会因内存移动而悬垂。
    _pinned: PhantomPinned,
}

// SAFETY:
// 1. `Timer` 的不可变字段（`cpu`、`kind`、`callback`、`context`、`owned_raw`）在构造后永不修改；
// 2. 内部可变的 `node`（`UnsafeCell<Node>`）仅在持有固定归属 CPU 的 `TimerBase::state` 自旋锁时访问；
// 3. 跨线程移动或共享 `Timer` 不会产生未同步的数据竞争。
unsafe impl Send for Timer {}
// SAFETY:
// 1. 对 `&Timer` 的所有并发状态修改（`arm`、`try_cancel`、`stop_sync`）均通过所属 CPU 的 `TimerBase::state` 自旋锁串行化；
// 2. 借用模式（`arm_borrowed`）另有 `unsafe` 契约要求调用方保证底层存储存活至同步停止返回。
unsafe impl Sync for Timer {}
impl Timer {
    /// 构造一个未注册的借用模式定时器节点（可放置在栈帧或静态存储中）。
    ///
    /// 借用模式节点不包含 `Arc` 控制块（`owned_raw` 为空），后续注册超时时必须显式调用
    /// `unsafe` 的 [`Timer::arm_borrowed`]，并在释放存储前通过 [`Timer::cancel_sync`] 或
    /// [`Timer::close_sync`] 同步等待回调退出。
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

    /// 在堆上分配并初始化一个由 `Arc` 管理生命周期的定时器节点。
    ///
    /// 构造时预先提取并缓存 `Arc::into_raw` 的原始指针（保留完整的堆控制块指针来源 Provenance），
    /// 后续每次新入队时仅通过 `Arc::increment_strong_count` 递增引用计数，避免从内部借用引用重新构造 `Arc`。
    pub fn try_new(
        cpu: usize,
        kind: TimerKind,
        callback: fn(Pin<&Timer>),
        context: usize,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        assert!(cpu < cpu::count(), "timer CPU is out of range");
        let pointer = Arc::into_raw(Arc::try_new(Self::new(cpu, kind, callback, context))?);
        // SAFETY:
        // 1. `pointer` 刚刚由上方 `Arc::try_new` 的唯一强引用通过 `Arc::into_raw` 导出，
        //    此时尚未对外发布任何共享引用或弱引用；
        // 2. 在独占访问权下将 `pointer` 写入堆分配内部的 `owned_raw` 字段不会产生别名冲突。
        unsafe { (*pointer.cast_mut()).owned_raw = pointer };
        // SAFETY: `pointer` 来自正上方同一处 `Arc::into_raw` 调用，此处恢复该唯一强引用归还给调用方；
        // 字段 `owned_raw` 仅缓存地址值，在后续实际入队前不额外持有引用计数。
        let timer = unsafe { Arc::from_raw(pointer) };
        // SAFETY: `timer` 位于独立的 `Arc` 堆分配中，且本模块不提供任何将 `Timer` 移出 `Arc` 的接口，
        // 内存地址在整个生命周期内恒定不变。
        Ok(unsafe { Pin::new_unchecked(timer) })
    }

    /// 返回构造时传入的标量上下文值。
    pub fn context(&self) -> usize {
        self.context
    }

    /// 返回该定时器固定绑定的逻辑 CPU 编号。
    #[cfg(feature = "ktest")]
    pub fn cpu(&self) -> usize {
        self.cpu
    }

    /// 获取用于在侵入式队列中登记的 `Timer` 原始指针。
    ///
    /// 对于 `Arc` 堆拥有节点，返回构造时缓存的 `owned_raw`（带有完整 `Arc::into_raw` Provenance）；
    /// 对于借用节点，直接返回 `self` 的引用地址。
    fn node_pointer(&self) -> *const Self {
        if self.owned_raw.is_null() {
            self as *const Self
        } else {
            self.owned_raw
        }
    }

    /// 为 `Arc` 堆拥有模式的定时器注册或更新绝对截止时间 `deadline`。
    ///
    /// # 返回值
    /// - `Ok(true)`：节点原本未在队列中，本次调用将其新插入队列；
    /// - `Ok(false)`：节点原本已在队列中，本次调用就地更新了其截止时间并重排位置；
    /// - `Err(ArmError)`：节点已关闭（[`ArmError::Closed`]）或期限超出时间轮范围（[`ArmError::OutOfRange`]）。
    ///
    /// # Panics
    /// 若对 [`Timer::new`] 构造的借用模式节点调用本方法，将直接触发断言 panic（借用节点必须调用 [`Timer::arm_borrowed`]）。
    pub fn arm(self: Pin<&Self>, deadline: MonotonicInstant) -> Result<bool, ArmError> {
        assert!(
            !self.owned_raw.is_null(),
            "borrowed timer requires unsafe arm_borrowed"
        );
        self.arm_inner(deadline)
    }

    /// 为调用方管理存储的借用模式定时器注册或更新绝对截止时间 `deadline`。
    ///
    /// 即使当前节点正在执行上一轮回调（`running == true`），也可安全调用本方法登记下一轮独立超时。
    ///
    /// # Safety
    /// 1. `self` 所在的内存地址以及回调依赖的 `context` 外部数据必须保持固定且有效，
    ///    直到调用方通过 [`Timer::cancel_sync`] 或 [`Timer::close_sync`] 确认定时器已不在队列中且正在运行的回调已彻底退出；
    /// 2. 在同步停止完成之前，严禁移动、覆写、释放 `Timer` 内存或提前退出其所在的栈帧。
    pub unsafe fn arm_borrowed(
        self: Pin<&Self>,
        deadline: MonotonicInstant,
    ) -> Result<bool, ArmError> {
        self.arm_inner(deadline)
    }

    /// 在所属 CPU 的 `TimerBase::state` 锁内将节点插入或重排到对应队列，并在解锁后通知硬件时钟重编程。
    fn arm_inner(self: Pin<&Self>, deadline: MonotonicInstant) -> Result<bool, ArmError> {
        let base = base(self.cpu);
        let pointer = self.node_pointer();
        let mut state = base.state.lock();
        // SAFETY:
        // 1. 已持有 `self.cpu` 对应的 `base.state` 自旋锁，排他访问 `self.node` 及所属红黑树/时间轮；
        // 2. 对于 `Arc` 拥有节点，调用方持有的 `Pin<&Self>` 保证了至少存在一份存活强引用，
        //    `Arc::increment_strong_count(pointer)` 为队列额外增加一份强引用并由 `queued_owned` 记录；
        // 3. 对于借用节点，调用方通过 `arm_borrowed` 的契约保证节点在同步取消前地址稳定且有效。
        unsafe {
            // Step 1: 检查节点是否已被永久关闭；若为粗粒度定时器，在修改原状态前预先校验新期限是否在时间轮可表示范围内。
            if (*self.node.get()).closed {
                return Err(ArmError::Closed);
            }
            let now = time::now();
            if self.kind == TimerKind::Coarse {
                state.wheel.accepts(deadline, now)?;
            }

            // Step 2: 若节点已在队列中则先摘除旧位置；若是首次入队且为 Arc 拥有节点，则增加一份队列持有的 Arc 强引用。
            let added = !(*self.node.get()).queued;
            if !added {
                state.unlink(pointer);
            } else if !self.owned_raw.is_null() {
                Arc::increment_strong_count(pointer);
                (*self.node.get()).queued_owned = true;
            }

            // Step 3: 写入新的截止时间并将节点插入红黑树或时间轮，随后释放 base 锁并通知目标 CPU 重编程硬件时钟。
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

    /// 尝试非阻塞地取消定时器：仅当节点未处于回调执行态（`!running`）时摘除队列中的待触发超时。
    ///
    /// 若回调正在执行，立即返回 [`CancelResult::Busy`] 而不阻塞等待，也不会摘除回调期间可能已重排的下一轮超时。
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
        // SAFETY:
        // 1. 已持有所属 CPU 的 `base.state` 自旋锁，排他访问 `self.node` 字段与侵入式队列；
        // 2. 若摘链产生了需要归还的 `Arc` 强引用（`release == true`），延后到 `drop(state)` 解锁后再释放，避免死锁。
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

    /// 仅摘除队列中尚未触发的下一轮超时登记，不阻塞等待正在运行的回调。
    ///
    /// 与 [`Timer::try_cancel`] 不同，即使当前回调正在运行（`running == true`），本函数也会将其在回调期间
    /// 重新注册的 `queued` 下一轮超时从队列中摘除。返回值表示本次调用是否实际从队列摘除了待触发登记。
    pub(crate) fn cancel_pending(self: Pin<&Self>) -> bool {
        let pointer = self.node_pointer();
        let mut state = base(self.cpu).state.lock();
        // SAFETY:
        // 1. 已持有所属 CPU 的 `base.state` 自旋锁，排他访问 `self.node.queued` 状态与队列链表；
        // 2. `state.cancel` 仅摘除 `queued` 链表并转出 `queued_owned` 强引用，不影响正在执行回调所持有的 `running` 状态。
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

    /// 在可睡眠的普通任务上下文中取消当前定时器，并阻塞等待正在执行的回调彻底退出。
    ///
    /// 取消完成后该节点仍可再次调用 `arm` 注册超时。调用方应先停止外部并发调用 `arm` 的生产者线程，
    /// 且**严禁在定时器自身的回调函数内部调用本方法**（会导致死锁或不可睡眠断言失败）。
    /// 返回值表示在停止过程中是否至少从队列中摘除过一次尚未触发的超时登记。
    pub fn cancel_sync(self: Pin<&Self>) -> bool {
        self.stop_sync(false)
    }

    /// 在可睡眠的普通任务上下文中永久关闭定时器：拒绝一切后续 `arm` 请求，摘除待触发超时，并阻塞等待正在执行的回调彻底退出。
    ///
    /// 关闭后所有对该节点的 `arm` / `arm_borrowed` 调用均返回 `Err(ArmError::Closed)`。
    /// **严禁在定时器自身的回调函数内部调用本方法**。返回值表示是否从队列中摘除过待触发的超时登记。
    pub fn close_sync(self: Pin<&Self>) -> bool {
        self.stop_sync(true)
    }

    /// 同步取消或永久关闭定时器的内部循环实现。
    ///
    /// 必须在可睡眠的普通任务上下文（且当前任务未持有其他活跃 `Waiter`）中调用。
    /// 若回调在执行期间自我重注册（re-arm），循环会在每一轮唤醒后再次摘除新登记并继续等待，直到 `!queued && !running`。
    fn stop_sync(self: Pin<&Self>, close: bool) -> bool {
        crate::sched::assert_switchable();
        let base = base(self.cpu);
        let pointer = self.node_pointer();
        let mut cancelled = false;
        loop {
            // Step 1: 持 base.state 锁标记关闭（若请求 close）、摘除队列中的待触发节点，并检查回调是否正在运行。
            let mut state = base.state.lock();
            // SAFETY: 已持有所属 CPU 的 `base.state` 自旋锁，排他读写 `self.node` 的 `closed`、`queued`、`running` 状态及队列链接。
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

            // Step 2: 在 base.state 锁外释放摘除的 Arc 强引用并通知硬件时钟重编程；若回调未在运行则同步停止已完成。
            release_owned(pointer, release);
            if queued {
                changed(self.cpu);
            }
            if !running {
                return cancelled;
            }

            // Step 3: 若回调正在远端 CPU 或底半部中执行，先向永久存活的 base.finished 等待队列注册栈上等待节点，
            // 随后重新获取 base.state 锁复查 running 状态，确认仍为 true 后再阻塞，从而杜绝唤醒丢失（Lost Wakeup）。
            let waiter = Waiter::new();
            let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `base.finished` 位于静态 `CpuLocal` 中永久有效；
            // 2. `node` 在当前栈帧中通过 `pin!` 固定，且下方 `drop(registration)` 在 `node` 离开作用域前必定执行，不会遗忘注销。
            let registration = unsafe { base.finished.register(node.as_mut()) };
            let state = base.state.lock();
            // SAFETY: 已在 `base.finished` 完成等待节点登记，并持有同一把 `base.state` 自旋锁复查 `running`；
            // 若 `run_callback` 在此前已清零 `running`，此处会读到 `false` 并跳过 `park()`；若在此后清零，必会唤醒已登记的 `node`。
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
    /// 析构定时器时校验其已完全脱离队列且无正在执行的回调。
    ///
    /// `Drop` 不会隐式阻塞等待远端 CPU 回调结束；借用模式节点必须由调用方在析构前完成 `cancel_sync` / `close_sync`。
    fn drop(&mut self) {
        let _state = base(self.cpu).state.lock();
        // SAFETY: 已持有所属 CPU 的 `base.state` 自旋锁，且 `&mut self` 保证当前处于析构独占阶段，可安全读取 `self.node`。
        let node = unsafe { &*self.node.get() };
        assert!(!node.queued && !node.running, "dropping an active timer");
        assert!(!node.queued_owned, "dropping queued timer ownership");
    }
}

/// 每核定时器引擎受自旋锁保护的可变状态集合。
struct BaseState {
    /// 高精度定时器（[`TimerKind::Precise`]）的侵入式红黑树队列。
    precise: queue::Queue,
    /// 粗粒度定时器（[`TimerKind::Coarse`]）的侵入式八层时间轮。
    wheel: wheel::Wheel,
    /// 标记是否已有到期粗粒度定时器正在等待或正由本核底半部服务线程处理。
    service_pending: bool,
    /// 内核自检模式下统计硬中断与底半部回调执行次数的计数器。
    #[cfg(feature = "ktest")]
    stats: testing::TimerStats,
}

// SAFETY:
// 1. `BaseState` 内部的红黑树与时间轮裸指针成员均受 `Arc` 强引用（`queued_owned`）或 `arm_borrowed` 同步停止协议保活；
// 2. `BaseState` 仅存放在所属 CPU 的 `TimerBase::state` 自旋锁内部，所有跨核访问均由该锁串行化。
unsafe impl Send for BaseState {}

impl BaseState {
    /// 构造空的每核定时器队列状态。
    const fn new() -> Self {
        Self {
            precise: queue::Queue::new(),
            wheel: wheel::Wheel::new(),
            service_pending: false,
            #[cfg(feature = "ktest")]
            stats: testing::TimerStats::new(),
        }
    }

    /// 仅将 `timer` 从所属侵入式容器（红黑树或时间轮）中摘除，不修改 `queued` 标志或引用计数。
    ///
    /// # Safety
    /// 1. 调用方必须已持有该节点所属 CPU 的 `TimerBase::state` 自旋锁；
    /// 2. `timer` 必须是非空、有效且当前确实挂在 `self.precise` 或 `self.wheel` 中的节点指针。
    unsafe fn unlink(&mut self, timer: *const Timer) {
        // SAFETY: 调用方已持有 `base.state` 自旋锁并保证 `timer` 是当前队列中的有效成员，满足底层容器 `remove` 的前置条件。
        unsafe {
            match (*timer).kind {
                TimerKind::Precise => self.precise.remove(timer),
                TimerKind::Coarse => self.wheel.remove(timer),
            }
        }
    }

    /// 若节点当前在队列中，将其摘除并清除 `queued` 与 `queued_owned` 标志。
    ///
    /// 返回值表示调用方是否需要在释放 `base.state` 自旋锁后调用 [`release_owned`] 归还一份 `Arc` 强引用。
    ///
    /// # Safety
    /// 1. 调用方必须已持有该节点所属 CPU 的 `TimerBase::state` 自旋锁；
    /// 2. `timer` 指向的 `Timer` 内存必须有效；若其 `node.queued` 为 `true`，则它必定属于当前 `BaseState` 的队列。
    unsafe fn cancel(&mut self, timer: *const Timer) -> bool {
        // SAFETY: 调用方已持有 `base.state` 自旋锁且 `timer` 内存有效；仅通过 `UnsafeCell::get()` 修改 `Node` 内部字段。
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

    /// 在持有 `base.state` 锁的前提下，检查时间轮中最早桶的处理期限是否已不晚于当前时刻 `now`。
    fn coarse_due(&self, now: MonotonicInstant) -> bool {
        self.wheel.next().is_some_and(|deadline| deadline <= now)
    }
}

/// 每个逻辑 CPU 静态拥有的定时器调度基座（存放于 `CpuLocal` 中永久存活）。
///
/// 包含受自旋锁保护的双队列状态、唤醒底半部服务线程的等待队列，以及通知同步取消者回调已退出的等待队列。
pub(crate) struct TimerBase {
    /// 保护本核红黑树、时间轮以及归属本核的所有 `Timer` 节点 `Node` 状态的中断安全自旋锁。
    state: SpinLock<BaseState>,
    /// 本核常驻底半部服务线程（[`service`]）阻塞等待粗粒度定时器到期的等待队列。
    service: WaitQueue,
    /// 供 [`Timer::cancel_sync`] / [`Timer::close_sync`] 等待正在执行的回调彻底退出的等待队列。
    finished: WaitQueue,
    /// 内核自检模式下记录本核常驻底半部服务线程的任务引用，用于观测其调度与绑定状态。
    #[cfg(feature = "ktest")]
    service_task: SpinLock<Option<Arc<crate::task::Task>>>,
}

impl TimerBase {
    /// 在 `CpuLocal` 编译期静态初始化阶段构造空的 `TimerBase`。
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

/// 获取指定逻辑 CPU 编号对应的静态 `TimerBase` 引用（不加锁）。
fn base(cpu: usize) -> &'static TimerBase {
    &cpu::get(cpu).timer_base
}

/// 在修改了 `target` CPU 的定时器队列并释放 `base.state` 锁后调用，触发硬件时钟比较器重编程。
///
/// 若目标为当前 CPU 则直接关中断重算并写入硬件比较器；若为远端 CPU 则发送 `KICK_TIMER_REPROGRAM` IPI。
/// **调用前必须已释放所有 `TimerBase::state` 锁**，否则在本核重编程读取期限时会导致自旋锁重入死锁。
fn changed(target: usize) {
    let _irq = IrqSaveGuard::new();
    if target == cpu::current().id {
        time::reprogram_current();
    } else {
        cpu::kick(target, cpu::KICK_TIMER_REPROGRAM);
    }
}

/// 在 `TimerBase::state` 锁外释放先前从队列或回调上下文中移交出来的一份 `Arc<Timer>` 强引用。
///
/// 当 `release` 为 `true` 时，`timer` 必须是构造时由 `Arc::into_raw` 导出的 `owned_raw` 指针；
/// 若为借用模式节点，`release` 恒为 `false`，本函数为空操作且不解引用 `timer`。
fn release_owned(timer: *const Timer, release: bool) {
    if release {
        // SAFETY:
        // 1. `release == true` 仅在 `Arc` 拥有节点先前通过 `Arc::increment_strong_count` 获取过队列强引用，
        //    且该强引用已被 `cancel` 或 `run_callback` 唯一转出时成立；
        // 2. `timer` 来自 `Timer::node_pointer()` 返回的 `owned_raw`（即 `Arc::into_raw` 原始指针）；
        // 3. 调用点均位于 `base.state` 自旋锁之外，即使触发最后一个强引用析构 `Timer::drop` 获取 `base.state` 锁也不会死锁。
        drop(unsafe { Arc::from_raw(timer) });
    }
}

/// 在关中断并持有当前 CPU `TimerBase::state` 锁的临界区内计算下一个最早硬件中断截止时间，并交由 `operation` 编程硬件比较器。
///
/// `operation` 仅可用于执行时钟滴答换算与硬件寄存器写入，严禁在闭包内再次获取 `base.state` 锁或睡眠。
pub(crate) fn with_next_deadline<R>(operation: impl FnOnce(Option<MonotonicInstant>) -> R) -> R {
    let _irq = IrqSaveGuard::new();
    let state = cpu::current().timer_base.state.lock();
    let precise = state.precise.first();
    // 若本核底半部服务线程已被唤醒（service_pending 为 true），则在它处理完本批粗粒度定时器并重查前，
    // 无需重复为已到期的粗粒度桶安装立即触发的硬件时钟中断。
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

/// 在持有 `base.state` 锁的临界区内从指定队列（高精度红黑树或粗粒度时间轮）摘出一个已到期的定时器，
/// 并将其状态从入队等待（`queued`）切换为正在执行回调（`running`）。
///
/// 若摘出的节点恰好仍在另一个上下文中执行上一轮回调（`running == true`，即回调期间自我重注册且立即到期），
/// 则将其重新放回原队列并返回 `None`，避免同一节点并发重入执行回调。
fn take_due(base: &TimerBase, kind: TimerKind) -> Option<(*const Timer, bool)> {
    let mut state = base.state.lock();
    let now = time::now();
    // SAFETY:
    // 1. 已持有 `base.state` 自旋锁，排他访问 `state.precise`、`state.wheel` 及队列中节点的 `Node` 状态；
    // 2. 队列中的节点由 `queued_owned` 的 `Arc` 强引用或调用方 `arm_borrowed` 同步停止契约保证内存存活且地址固定。
    unsafe {
        // Step 1: 从对应队列弹出一个截止时间不晚于 now 的节点。
        let timer = match kind {
            TimerKind::Precise => state.precise.pop_due(now),
            TimerKind::Coarse => state.wheel.pop_due(now),
        }?;
        let node = (*timer).node.get();

        // Step 2: 若该节点上一轮回调尚未退出（running 为 true），将其放回原队列等待，防止同一节点并发执行回调。
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

        // Step 3: 将节点状态从 queued 转移为 running，并将队列持有的 Arc 强引用标记（queued_owned）转交给回调执行者。
        (*node).queued = false;
        (*node).running = true;
        let owned = (*node).queued_owned;
        (*node).queued_owned = false;
        Some((timer, owned))
    }
}

/// 在 `base.state` 自旋锁外执行由 [`take_due`] 摘出的到期定时器回调，并在执行完毕后完成状态清理与完成通知。
///
/// 在重新获取 `base.state` 锁将 `node.running` 清零之后，本函数绝不再解引用 `timer` 指针，
/// 仅在锁外唤醒静态的 `base.finished` 等待队列并释放转入的 `Arc` 强引用，从而保证栈上借用节点在 `stop_sync` 返回后即可安全退栈。
fn run_callback(base: &'static TimerBase, timer: *const Timer, owned: bool, kind: TimerKind) {
    #[cfg(feature = "ktest")]
    testing::pause(timer, testing::PausePoint::Dequeued);
    // Step 1: 在不持有 base.state 锁的状态下调用定时器回调函数（允许回调内部重调 arm 或 cancel_pending）。
    // SAFETY:
    // 1. `take_due` 已将 `node.running` 置为 `true`，确保当前上下文独占该节点的回调执行权；
    // 2. 在 `running` 清零前，节点内存由转入的 `Arc` 强引用（`owned == true`）或调用方的 `arm_borrowed` 契约（等待 `running` 清零后才退栈）保活；
    // 3. 节点地址固定，构造 `Pin<&Timer>` 共享引用安全。
    unsafe {
        let pinned = Pin::new_unchecked(&*timer);
        ((*timer).callback)(pinned);
    }
    #[cfg(feature = "ktest")]
    testing::pause(timer, testing::PausePoint::Tail);

    // Step 2: 重新获取 base.state 锁，更新统计计数并将 running 置为 false（这是执行路径对借用节点的最后一次内存访问）。
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
    // SAFETY: 已持有 `base.state` 自旋锁且仍持有 `running` 执行权；将 `running` 清零后不再访问 `timer` 内部字段。
    unsafe { (*(*timer).node.get()).running = false };
    drop(state);

    // Step 3: 在 base.state 锁外唤醒所有正在 stop_sync 中等待回调退出的任务，并归还本次回调持有的 Arc 强引用。
    base.finished.wake_all();
    release_owned(timer, owned);
}

/// 时钟硬中断处理入口：按配额执行本核到期的高精度定时器回调，并在存在到期粗粒度定时器时唤醒本核底半部服务线程。
pub(crate) fn handle_interrupt() {
    let base = &cpu::current().timer_base;
    // Step 1: 在硬中断上下文中连续摘取并执行最多 HARD_BUDGET 个到期的高精度定时器回调。
    for _ in 0..HARD_BUDGET {
        let Some((timer, owned)) = take_due(base, TimerKind::Precise) else {
            break;
        };
        run_callback(base, timer, owned, TimerKind::Precise);
    }

    // Step 2: 检查粗粒度时间轮是否有到期桶；若服务线程尚未处于待运行状态，则置位 service_pending 并在锁外唤醒它。
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

/// 在系统启动阶段（开启定时中断与启动从核之前）由主核（BSP）为每个逻辑 CPU 创建常驻的粗粒度定时器底半部服务线程。
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

/// 绑定在指定逻辑 CPU 上的常驻内核服务线程主循环，负责在底半部上下文中批量执行到期的粗粒度定时器回调。
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
        // Step 1: 向本核永久存活的 base.service 等待队列注册栈上等待节点，若 service_pending 为 false 则睡眠让出 CPU。
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY:
        // 1. `base.service` 位于静态 `CpuLocal` 中永久有效；
        // 2. `node` 在当前栈帧中通过 `pin!` 固定，且在离开本轮作用域前必定通过 `drop(registration)` 取消登记。
        let registration = unsafe { base.service.register(node.as_mut()) };
        let pending = base.state.lock().service_pending;
        if !pending {
            waiter.park();
        }
        drop(registration);
        drop(waiter);

        // Step 2: 进入关抢占底半部（BottomHalfGuard），按批（每批最多 BH_BUDGET 个）摘取并执行到期的粗粒度定时器回调。
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

            // Step 3: 批次结束后检查时间轮中是否仍有到期节点，更新 service_pending 与统计计数，
            // 先退出底半部并重编程硬件时钟，若仍有剩余到期工作则主动 yield 让出一次调度后再继续下一批。
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

/// 仅供内核自检（`ktest`）使用的定时器状态快照、统计观测与并发交错暂停探针接口。
#[cfg(feature = "ktest")]
pub(crate) mod testing {
    use super::*;
    use core::hint::spin_loop;
    use core::sync::atomic::AtomicUsize;

    /// 导出底半部单批回调预算常量供自检断言使用。
    pub(crate) const BH_BUDGET: usize = super::BH_BUDGET;
    /// 导出粗粒度时间轮基础 tick 纳秒数（10ms）供自检计算截止时间使用。
    pub(crate) const TICK_NS: u64 = super::wheel::TICK_NS;
    /// 导出粗粒度时间轮允许的最大相对 tick 跨度供边界自检使用。
    pub(crate) const MAX_TIMEOUT_TICKS: u64 = super::wheel::MAX_TIMEOUT_TICKS;

    /// 单个 `Timer` 节点在某一时刻的生命周期状态快照。
    #[derive(Clone, Copy)]
    pub(crate) struct TimerSnapshot {
        /// 节点当前是否挂在红黑树或时间轮队列中等待到期。
        pub queued: bool,
        /// 节点当前是否正在执行回调函数。
        pub running: bool,
        /// 节点是否已被 [`Timer::close_sync`] 永久关闭。
        pub closed: bool,
        /// 节点内部记录的最近一次注册截止时间。
        pub expiry: MonotonicInstant,
    }

    /// 单个逻辑 CPU 上的定时器回调与底半部批次累计执行统计。
    #[derive(Clone, Copy)]
    pub(crate) struct TimerStats {
        /// 时钟硬中断上下文已执行完的高精度定时器回调总数。
        pub hard_callbacks: usize,
        /// 底半部服务线程已执行完的粗粒度定时器回调总数。
        pub bh_callbacks: usize,
        /// 底半部服务线程非空批次的执行总次数。
        pub bh_batches: usize,
        /// 底半部服务线程因超出单批预算（`BH_BUDGET`）而主动 `yield_now` 续批的次数。
        pub bh_continuations: usize,
        /// 观测到的单个底半部批次内连续执行的粗粒度回调最大数量。
        pub max_bh_callbacks: usize,
    }

    impl TimerStats {
        /// 构造全部计数为零的初始统计结构。
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

    /// 在持有 `timer` 所属 CPU 的 `base.state` 锁下读取其生命周期状态与截止时间快照。
    pub(crate) fn snapshot(timer: Pin<&Timer>) -> TimerSnapshot {
        let _state = base(timer.cpu).state.lock();
        // SAFETY: `timer` 由调用方传入的 `Pin<&Timer>` 保证存活，且已持有其所属 CPU 的 `base.state` 自旋锁排他保护 `node` 字段。
        let node = unsafe { &*timer.node.get() };
        TimerSnapshot {
            queued: node.queued,
            running: node.running,
            closed: node.closed,
            expiry: node.expires,
        }
    }

    /// 读取指定逻辑 CPU 的定时器回调与批次累计统计快照。
    pub(crate) fn stats(cpu: usize) -> TimerStats {
        base(cpu).state.lock().stats
    }

    /// 获取指定逻辑 CPU 上常驻粗粒度定时器服务线程的 `Task` 引用副本。
    pub(crate) fn service_task(cpu: usize) -> Option<Arc<crate::task::Task>> {
        base(cpu).service_task.lock().as_ref().map(Arc::clone)
    }

    /// 在持有指定逻辑 CPU 的 `base.state` 锁下完整校验高精度定时器红黑树的结构不变量，返回 `(节点总数, 黑高)`。
    pub(crate) fn validate_tree(cpu: usize) -> (usize, usize) {
        let state = base(cpu).state.lock();
        // SAFETY: 已持有 `base.state` 自旋锁，树中所有已入队节点均由各自的 `Arc` 强引用或借用同步停止契约保活。
        unsafe { state.precise.validate() }
    }

    /// 自检并发交错测试中，在 [`run_callback`] 内部指定的无锁暂停拦截位置。
    #[derive(Clone, Copy)]
    pub(crate) enum PausePoint {
        /// 节点刚从队列摘除（`queued = false, running = true`）、尚未调用 `callback` 函数之前的无锁窗口。
        Dequeued,
        /// `callback` 函数刚返回、尚未重新获取 `base.state` 锁清零 `running` 之前的无锁窗口。
        Tail,
    }

    /// 暂停探针状态：已由控制核设置目标定时器与拦截点，等待目标核 [`run_callback`] 命中的状态。
    const ARMED: usize = 1;
    /// 暂停探针状态：目标核 [`run_callback`] 已命中拦截点并正在自旋等待放行。
    const PAUSED: usize = 2;
    /// 暂停探针状态：控制核已发出放行信号，允许目标核退出自旋继续执行后续步骤。
    const RELEASED: usize = 3;
    /// 暂停探针自旋等待的最大超时秒数（防止测试异常时永久卡死系统）。
    const PAUSE_TIMEOUT_SECONDS: u64 = 5;

    /// 当前暂停探针关注的目标 `Timer` 裸指针地址。
    static PAUSE_TIMER: AtomicUsize = AtomicUsize::new(0);
    /// 当前暂停探针关注的拦截位置（[`PausePoint`] 转换的整数值）。
    static PAUSE_POINT: AtomicUsize = AtomicUsize::new(0);
    /// 当前暂停探针的原子状态机（`0` 空闲 / [`ARMED`] / [`PAUSED`] / [`RELEASED`]）。
    static PAUSE_STATE: AtomicUsize = AtomicUsize::new(0);

    /// 为运行在远端 CPU 上的指定 `timer` 设置回调执行过程中的暂停拦截点 `point`。
    ///
    /// 仅用于跨核并发取消/重注册竞态测试；重新设置前必须确保上一轮被拦截的定时器已彻底停止且状态归零。
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

    /// 检查远端 CPU 的回调执行路径是否已命中预设的拦截点并处于暂停自旋状态。
    pub(crate) fn is_paused() -> bool {
        PAUSE_STATE.load(Ordering::Acquire) == PAUSED
    }

    /// 放行当前正停留在拦截点自旋的远端回调执行路径，使其继续执行后续逻辑。
    pub(crate) fn release_pause() {
        PAUSE_STATE
            .compare_exchange(PAUSED, RELEASED, Ordering::Release, Ordering::Relaxed)
            .expect("timer runner was not paused");
    }

    /// 取消尚未命中的暂停探针或放行已命中的远端回调执行路径（幂等操作，供测试清理路径调用）。
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

    /// 由 [`run_callback`] 在无锁窗口调用：若当前 `timer` 与 `point` 匹配已激活的暂停探针，则自旋等待远端控制核调用 [`release_pause`]。
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

    /// 执行粗粒度时间轮的单元测试套件，返回通过的子用例数量。
    pub(crate) fn self_check_wheel() -> usize {
        super::wheel::self_check()
    }

    /// 读取 [`sleep::test_sleep_until`] 最近一轮实际注册到定时器中的绝对截止时间。
    pub(crate) fn sleep_deadline() -> MonotonicInstant {
        super::sleep::test_sleep_deadline()
    }
}
