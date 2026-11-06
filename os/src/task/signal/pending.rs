//! 待处理信号队列（Pending Signal Queue）管理与延迟锁外析构。
//!
//! ## 模块职责与架构定位
//! 本子模块负责单个线程（`ThreadState::pending`）或线程组（`GroupState::pending`）内按信号编号（1..=64）组织的待处理信号实例存储、RLIMIT 配额计费以及摘除节点的延迟回收。
//! - **上游调用者**：仅限父模块 `crate::task::signal` 在持有信号锁（`siglock` 与组/线程自旋锁）时调用。
//! - **下游依赖**：`crate::task::process`（执行 `RLIMIT_SIGPENDING` 用户信号配额扣减与归还句柄 `SignalCharge`）。
//!
//! ## 核心设计与混合排队布局
//! 1. **标准信号合并与实时信号排队**：编号 `< SIGRTMIN`（1..31）的标准信号在 `bits` 位图中已置位时直接合并忽略重复实例；编号 `>= SIGRTMIN`（32..64）的实时信号支持同号多实例按 FIFO 顺序排队。
//! 2. **用户堆节点与内核内联槽位（`reserve`）混合 FIFO**：
//!    - 用户态触发的信号（如 `kill`、`tkill`、`rt_sigqueueinfo`）在进入关系锁和信号锁之前预先分配 `Box<QueuedSignal>`（`PreparedSignal::user`），入队时绑定 `SignalCharge` 计费凭证并挂入单向链表 `queues[i]`。
//!    - 内核触发的信号（如定时器到期、管道断开、`SIGCHLD`，使用 `PreparedSignal::kernel`）不能在关中断或自旋锁内分配堆内存，因此每个信号号提供一个免分配的内联槽位 `reserve[i]`。
//!    - 为严格保持同一信号号下用户信号与内核信号的先后到达顺序，`before_reserve[i]` 记录内核信号写入 `reserve[i]` 时队列中排在它前面的用户节点数。出队（`take`）时必须先弹完这批早到的用户节点，随后才消费 `reserve[i]`，最后继续消费晚到的用户节点。
//!
//! ```text
//!   同一信号号 index 的逻辑 FIFO 顺序：
//!   queues[index]: [User #1] -> [User #2] -> [User #3 (晚于 reserve 到达)]
//!                   \_________ _________/
//!                             v
//!                  before_reserve[index] = 2
//!                  reserve[index] = Some(Kernel SignalInfo)
//!
//!   take() 消费顺序：User #1 -> User #2 -> reserve[index] -> User #3
//! ```
//!
//! ## 锁获取顺序与延迟释放契约
//! - `Pending` 的所有方法均在持有外层 `Sighand::actions` 与 `GroupState`/`ThreadState` 自旋锁的临界区内执行，内部禁止执行堆内存分配或释放。
//! - 被批量丢弃（`discard`，如 `SIGCONT` 清除停止信号、信号改为忽略、线程退出）的堆节点及其 `SignalCharge` 先挂入侵入式单向链表 `Retired`，由外层在释放全部 `TaskGraph` 锁与信号自旋锁后通过 `Pending::retired()` 取走并迭代析构，避免在自旋锁临界区内释放堆内存或销毁最后一个用户配额计数，同时防止深链表递归析构撑爆内核栈。

use alloc::boxed::Box;
use core::ptr;

use super::{signal_bit, SignalError, SignalInfo, SIGNAL_COUNT, SIGRTMIN};
use crate::task::process::{Resource, SignalCharge, ThreadGroup};

/// 用户态排队信号的堆分配链表节点，携带信号详情与 `RLIMIT_SIGPENDING` 配额凭证。
pub(super) struct QueuedSignal {
    /// 待递送的完整信号信息（编号、`si_code` 与附加载荷）。
    info: SignalInfo,
    /// 计入目标用户 `RLIMIT_SIGPENDING` 的配额占用凭证；节点析构时自动归还配额。
    charge: Option<SignalCharge>,
    /// 同信号号队列或延迟回收链表（`Retired`）中的下一个节点。
    next: Option<Box<QueuedSignal>>,
}

/// 在获取进程关系锁和信号自旋锁之前预先准备的待入队信号载体。
pub(crate) struct PreparedSignal {
    /// 用户发送信号时预先在锁外分配的堆节点；内核发送信号时为 `None`（使用内联 `reserve` 槽位）。
    node: Option<Box<QueuedSignal>>,
}

impl PreparedSignal {
    /// 构造内核信号预备载体：仅使用每信号号的内联 `reserve` 槽位，不分配堆节点也不占用用户配额。
    pub(crate) fn kernel() -> Self {
        Self { node: None }
    }

    /// 在所有锁外为用户发送信号预分配堆节点；`RLIMIT_SIGPENDING` 计费推迟到持锁确认入队时再绑定。
    pub(crate) fn user(info: SignalInfo) -> Result<Self, SignalError> {
        let node = Box::try_new(QueuedSignal {
            info,
            charge: None,
            next: None,
        })
        .map_err(|_| SignalError::NoMemory)?;
        Ok(Self { node: Some(node) })
    }
}

/// 单个信号编号下的侵入式单向 FIFO 队列，持有头节点所有权与尾节点裸指针以支持 $O(1)$ 尾插。
struct Queue {
    /// 队列首节点所有权；队列为空时为 `None`。
    head: Option<Box<QueuedSignal>>,
    /// 指向队列最后一个 `Box<QueuedSignal>` 堆内存的裸指针；队列为空时为 null。
    tail: *mut QueuedSignal,
    /// 当前队列中串联的用户堆节点数量。
    len: usize,
}

// SAFETY: `tail` 仅指向本 `Queue` 从 `head` 串联独占拥有的 `Box<QueuedSignal>` 堆节点，且 `Queue` 的全部读写均由所属 `Pending` 外层的信号自旋锁串行化。
unsafe impl Send for Queue {}

impl Queue {
    /// 创建空的单向信号队列。
    fn new() -> Self {
        Self {
            head: None,
            tail: ptr::null_mut(),
            len: 0,
        }
    }

    /// 将锁外预分配好的节点追加到队列尾部（$O(1)$ 时间复杂度）。
    fn push(&mut self, mut node: Box<QueuedSignal>) {
        assert!(node.next.is_none());
        let tail = ptr::from_mut(node.as_mut());
        if self.tail.is_null() {
            self.head = Some(node);
        } else {
            // SAFETY: 队列非空时 `self.tail` 指向由 `self.head` 链条独占保活的尾节点 `Box`；当前持有 `&mut self` 独占借用且尾节点的 `next` 必为 `None`。
            unsafe {
                (*self.tail).next = Some(node);
            }
        }
        self.tail = tail;
        self.len += 1;
    }

    /// 弹出队列首节点并断开其 `next` 链接，节点所有权交由调用方带出信号锁外释放。
    fn pop(&mut self) -> Option<Box<QueuedSignal>> {
        let mut node = self.head.take()?;
        self.head = node.next.take();
        self.len -= 1;
        if self.head.is_none() {
            self.tail = ptr::null_mut();
        }
        Some(node)
    }
}

/// 从 `Pending` 取出单个信号实例的结果，分离值拷贝的 `SignalInfo` 与需在锁外释放的可选堆节点。
pub(super) struct Removed {
    /// 被取出的信号元数据。
    pub(super) info: SignalInfo,
    /// 若该实例来自用户态堆队列，则携带被摘下的 `Box<QueuedSignal>` 以便转入 `Retired` 在锁外释放。
    node: Option<Box<QueuedSignal>>,
}

/// 已从待处理队列摘除、等待在全部自旋锁和关系锁外集中迭代释放的节点单向链表。
pub(super) struct Retired(Option<Box<QueuedSignal>>);

impl Retired {
    /// 创建空的待回收节点链表（可在获取任何自旋锁之前构造）。
    pub(super) fn new() -> Self {
        Self(None)
    }

    /// 将 `Removed` 中携带的堆节点头插并入待回收链表（纯指针操作，无堆分配或析构），并返回其 `SignalInfo`。
    pub(super) fn add(&mut self, mut removed: Removed) -> SignalInfo {
        if let Some(mut node) = removed.node.take() {
            node.next = self.0.take();
            self.0 = Some(node);
        }
        removed.info
    }
}

impl Drop for Retired {
    /// 循环逐个摘除并释放链表节点，避免长链表默认递归 Drop 导致内核栈溢出。
    fn drop(&mut self) {
        while let Some(mut node) = self.0.take() {
            self.0 = node.next.take();
        }
    }
}

/// 单个线程或线程组的 64 个信号待处理集合。
pub(super) struct Pending {
    /// 每个信号编号对应的用户态信号 FIFO 堆节点队列（索引为 `signum - 1`）。
    queues: [Queue; SIGNAL_COUNT],
    /// 每个信号编号供内核免分配发送使用的单槽位内联缓存（索引为 `signum - 1`）。
    reserve: [Option<SignalInfo>; SIGNAL_COUNT],
    /// 记录对应 `reserve[i]` 入队时尚未消费的前置用户堆节点数量，用于维持同号 FIFO 顺序。
    before_reserve: [usize; SIGNAL_COUNT],
    /// 在持锁 `discard` 期间暂存的已摘除堆节点，等待锁外通过 `retired()` 取走析构。
    retired: Retired,
    /// 待处理信号位图：第 `signum - 1` 位为 1 表示该信号号在 `queues` 或 `reserve` 中至少有一个待消费实例。
    pub(super) bits: u64,
}

impl Pending {
    /// 创建空的待处理信号集合，所有位图、队列与内联槽位初始化为空。
    pub(super) fn new() -> Self {
        Self {
            queues: core::array::from_fn(|_| Queue::new()),
            reserve: [None; SIGNAL_COUNT],
            before_reserve: [0; SIGNAL_COUNT],
            retired: Retired::new(),
            bits: 0,
        }
    }

    /// 将信号压入待处理集合：标准信号若已在位图中置位则直接合并返回；用户信号扣减目标 `RLIMIT_SIGPENDING` 配额后入堆队列，内核信号写入内联 `reserve` 槽位。
    pub(super) fn push(
        &mut self,
        info: SignalInfo,
        prepared: &mut PreparedSignal,
        target: Option<&ThreadGroup>,
    ) -> Result<(), SignalError> {
        let bit = signal_bit(info.signum)?;
        if info.signum < SIGRTMIN && self.bits & bit != 0 {
            return Ok(());
        }

        let index = info.signum as usize - 1;
        if prepared.node.is_some() {
            let target = target.expect("queued signal has no target account");
            let charge =
                target.charge_signal(target.resource_limit(Resource::SigPending).current)?;
            let mut node = prepared.node.take().expect("prepared signal disappeared");
            node.charge = Some(charge);
            self.queues[index].push(node);
        } else if self.reserve[index].is_none() {
            self.reserve[index] = Some(info);
            self.before_reserve[index] = self.queues[index].len;
        }
        self.bits |= bit;
        Ok(())
    }

    /// 在 `allowed` 允许的信号位掩码中按最小信号编号优先弹出一个实例；同一编号内严格遵守 `before_reserve` 记录的先后顺序。
    pub(super) fn take(&mut self, allowed: u64) -> Option<Removed> {
        let bits = self.bits & allowed;
        if bits == 0 {
            return None;
        }

        let index = bits.trailing_zeros() as usize;
        let removed = if self.reserve[index].is_some() && self.before_reserve[index] == 0 {
            Removed {
                info: self.reserve[index].take().unwrap(),
                node: None,
            }
        } else {
            let node = self.queues[index]
                .pop()
                .expect("signal bit has no queued instance");
            if self.reserve[index].is_some() {
                self.before_reserve[index] -= 1;
            }
            Removed {
                info: node.info,
                node: Some(node),
            }
        };

        if self.queues[index].len == 0 && self.reserve[index].is_none() {
            self.bits &= !(1u64 << index);
        }
        Some(removed)
    }

    /// 批量丢弃匹配 `mask` 位掩码的全部待处理信号实例，摘下的堆节点暂存入内部 `retired` 链表以便锁外释放。
    pub(super) fn discard(&mut self, mask: u64) {
        while let Some(removed) = self.take(mask) {
            self.retired.add(removed);
        }
    }

    /// 取走内部累积的全部待回收节点链表，供调用方在释放 `TaskGraph` 锁与信号自旋锁后安全析构。
    pub(super) fn retired(&mut self) -> Retired {
        core::mem::replace(&mut self.retired, Retired::new())
    }
}

impl Drop for Pending {
    /// 析构时将残留队列节点全部移入 `retired`，复用 `Retired::drop` 的迭代释放逻辑。
    fn drop(&mut self) {
        self.discard(u64::MAX);
    }
}
