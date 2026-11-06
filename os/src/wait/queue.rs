//! 基于栈上侵入式双向链表的等待队列（`WaitQueue`）。
//!
//! # 模块职责与架构定位
//! - **职责**：为内核同步原语与各类 I/O 子系统提供零堆分配、FIFO 顺序唤醒的等待队列实现。
//! - **上游调用者**：睡眠锁（`Mutex`）、条件变量、管道、套接字、终端（TTY）、进程退出等待及块设备请求队列。
//! - **下游依赖**：自旋锁（[`SpinLock`]）保护双向链表结构；通过 [`Waker::notify`] 将唤醒事件提交给调度器（[`crate::sched`]）。
//!
//! # 核心设计与链表内存布局
//! 1. **栈上固定节点（Pinned Stack Node）与零堆分配**：
//!    等待方在当前内核栈帧上构造 [`WaitNode`] 并使用 `pin!` 固定地址，调用 [`WaitQueue::register`] 将节点尾插到队列链表中，返回一个绑定生命周期的 [`QueueRegistration`] 守卫。
//! 2. **侵入式双向链表布局与状态不变量**：
//!
//! ```text
//! WaitQueue (inner: SpinLock<Queue>)
//! ┌───────────────────────────────────────┐
//! │ members: 2                            │
//! │ head ──────┐             tail ────────┼──────────────────────┐
//! └────────────┼──────────────────────────┘                      │
//!              ▼                                                 ▼
//!   WaitNode A (栈上 Pin 固定)                        WaitNode B (栈上 Pin 固定)
//! ┌────────────────────────────┐                  ┌────────────────────────────┐
//! │ linked: true               │                  │ linked: true               │
//! │ data (UnsafeCell):         │                  │ data (UnsafeCell):         │
//! │   previous: null           │◄─────────────────│   previous: &WaitNode A    │
//! │   next: &WaitNode B ───────┼─────────────────►│   next: null               │
//! │   waker: Some(Waker)       │                  │   waker: Some(Waker)       │
//! └────────────────────────────┘                  └────────────────────────────┘
//! ```
//!
//! 3. **唤醒与退列的两路交汇**：
//!    - **主动唤醒路径（`wake_one` / `wake_all`）**：在队列自旋锁内从队头摘除节点、取出 `waker: Option<Waker>` 并以 `Release` 内存序将 `linked` 置为 `false`；**释放队列自旋锁后**再调用 `waker.notify()` 进入调度器。
//!    - **等待方退出路径（`Drop for QueueRegistration`）**：当等待方被唤醒或因超时/信号提前退出作用域时，析构 [`QueueRegistration`] 获取队列自旋锁；若节点仍在队列中（未被 `wake_*` 摘走），则将其从链表摘除并在锁外析构残留的 `Waker`。
//!
//! # 锁获取顺序与并发契约
//! - **锁顺序**：`WaitQueue::inner` 自旋锁 -> **释放自旋锁** -> `Waker::notify()`（内部获取调度器锁）或 `Drop for Waker`（若触发最后一个任务强引用释放）。严禁在持有 `WaitQueue::inner` 自旋锁期间调用 `Waker::notify()` 或析构 `Waker`，以避免锁嵌套死锁及扩大关中断临界区。
//! - **生命周期安全契约**：在节点处于已入队状态（`linked == true`）期间，[`WaitNode`] 与 [`WaitQueue`] 的内存地址必须保持有效；一旦 `Queue::remove` 在自旋锁内完成链表摘除并执行 `linked.store(false, Ordering::Release)`，唤醒方便不再触碰该 [`WaitNode`] 内存。

use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::sync::SpinLock;
use crate::wait::Waker;

/// 受所属 [`WaitQueue`] 自旋锁保护的链表指针与唤醒句柄数据。
struct NodeData {
    /// 指向链表前驱节点的指针；若当前节点为队头或未入队则为 `null`。由所属队列的 `inner` 自旋锁保护。
    previous: *const WaitNode,
    /// 指向链表后继节点的指针；若当前节点为队尾或未入队则为 `null`。由所属队列的 `inner` 自旋锁保护。
    next: *const WaitNode,
    /// 当前节点携带的唤醒句柄；入队时为 `Some`，被摘除（唤醒或退列）时通过 `take()` 移出。由所属队列的 `inner` 自旋锁保护。
    waker: Option<Waker>,
}

/// 侵入式等待队列的栈上固定节点。
///
/// 节点内部的链表指针与 [`Waker`] 仅在持有所属 [`WaitQueue`] 的自旋锁时访问；
/// 通过 [`Pin`] 保证节点在入队期间内存地址不变，并通过 `linked` 原子标志在析构时断言节点已安全脱链。
pub struct WaitNode {
    /// 链表前后指针与唤醒句柄的内部可变容器，仅在持有所属 [`WaitQueue`] 的 `inner` 自旋锁时访问。
    data: UnsafeCell<NodeData>,
    /// 标记当前节点是否仍挂载在某个 [`WaitQueue`] 链表中；入队置 `true`，脱链时以 `Release` 置 `false`。
    linked: AtomicBool,
    /// 标记类型不可安全移动（`!Unpin`），确保节点一旦固定（`Pin`）其栈上地址在生命周期内保持稳定。
    _pinned: PhantomPinned,
}

impl WaitNode {
    /// 创建携带指定 [`Waker`] 的未入队等待节点（不产生堆内存分配，每个节点实例仅支持单次注册消费）。
    pub fn new(waker: Waker) -> Self {
        Self {
            data: UnsafeCell::new(NodeData {
                previous: ptr::null(),
                next: ptr::null(),
                waker: Some(waker),
            }),
            linked: AtomicBool::new(false),
            _pinned: PhantomPinned,
        }
    }
}

impl Drop for WaitNode {
    /// 析构时断言当前节点已从等待队列中脱链（`linked == false`），防止悬垂链表指针导致内存破坏。
    fn drop(&mut self) {
        assert!(
            !self.linked.load(Ordering::Acquire),
            "dropping a registered wait node"
        );
    }
}

/// [`WaitQueue`] 自旋锁内部维护的侵入式双向链表头尾状态。
struct Queue {
    /// 指向队列头部（最早入队）节点的指针；队列为空时为 `null`。
    head: *const WaitNode,
    /// 指向队列尾部（最晚入队）节点的指针；队列为空时为 `null`。
    tail: *const WaitNode,
    /// 当前挂载在链表中的有效节点数量。
    members: usize,
}

// SAFETY:
// 1. `Queue` 仅存放在 `WaitQueue::inner` (`SpinLock<Queue>`) 内部，所有对 `head`/`tail` 及节点 `NodeData` 的读写均由该自旋锁串行化；
// 2. `WaitQueue::register` 的调用方契约与 `QueueRegistration` 的共享 `Pin` 借用保证了入队节点在 `remove` 脱链前始终存活且地址固定；
// 3. 节点内部可变性被严格限制在受锁保护的 `UnsafeCell<NodeData>` 与原子标志 `linked` 中，因此跨线程转移 `Queue` 所有权是安全的。
unsafe impl Send for Queue {}

impl Queue {
    /// 创建空的等待队列链表状态。
    const fn new() -> Self {
        Self {
            head: ptr::null(),
            tail: ptr::null(),
            members: 0,
        }
    }

    /// 在持有队列自旋锁的前提下，将已固定的未入队节点追加到双向链表尾部。
    fn push(&mut self, node: Pin<&WaitNode>) {
        let node = node.get_ref();
        assert!(
            !node.linked.load(Ordering::Relaxed),
            "wait node already registered"
        );
        let members = self
            .members
            .checked_add(1)
            .expect("wait queue member count overflow");
        // SAFETY:
        // 1. 当前持有 `WaitQueue::inner` 自旋锁的独占可变引用 `&mut self`，无并发读写竞争；
        // 2. 上方 `assert!(!node.linked.load(...))` 确认 `node` 尚未加入任何队列，因此独占访问 `node.data` 是安全的；
        // 3. 若 `self.tail` 非空，该尾节点由其活跃的 `QueueRegistration` 保证在脱链前内存有效且地址固定，通过自旋锁修改其 `next` 指针不会产生别名冲突。
        unsafe {
            let data = &mut *node.data.get();
            assert!(data.waker.is_some(), "wait node already consumed");
            data.previous = self.tail;
            data.next = ptr::null();
            if self.tail.is_null() {
                self.head = node;
            } else {
                (*(*self.tail).data.get()).next = node;
            }
            self.tail = node;
        }
        self.members = members;
        node.linked.store(true, Ordering::Relaxed);
    }

    /// 在持有队列自旋锁的前提下，将指定节点从双向链表中摘除并移出其 [`Waker`]。
    ///
    /// 若节点已不在链表中（此前已被唤醒方摘走），则返回 `None`。
    /// 函数末尾以 `Release` 内存序将 `node.linked` 置为 `false`，此操作是队列对该节点内存的最后一次访问。
    fn remove(&mut self, node: &WaitNode) -> Option<Waker> {
        if !node.linked.load(Ordering::Relaxed) {
            return None;
        }
        let members = self
            .members
            .checked_sub(1)
            .expect("wait queue member count underflow");
        // SAFETY:
        // 1. 当前持有 `WaitQueue::inner` 自旋锁的独占可变引用 `&mut self`，串行化所有链表指针修改；
        // 2. `remove` 仅由本模块的 `pop`（针对当前队列头节点）和 `QueueRegistration::drop`（针对注册在本队列的节点）调用，且上方已检查 `node.linked == true`；
        // 3. 链表中的当前节点及其前驱 `previous`、后继 `next` 均处于已注册未脱链状态，由各自的 `QueueRegistration` 与 `register` 安全契约保证栈内存存活且固定。
        let waker = unsafe {
            let data = &mut *node.data.get();
            if data.previous.is_null() {
                self.head = data.next;
            } else {
                (*(*data.previous).data.get()).next = data.next;
            }
            if data.next.is_null() {
                self.tail = data.previous;
            } else {
                (*(*data.next).data.get()).previous = data.previous;
            }
            data.previous = ptr::null();
            data.next = ptr::null();
            data.waker
                .take()
                .expect("registered wait node has no waker")
        };
        self.members = members;
        node.linked.store(false, Ordering::Release);
        Some(waker)
    }

    /// 在持有队列自旋锁的前提下，弹出队头节点并返回其 [`Waker`]。
    ///
    /// 仅向锁外返回独立的 [`Waker`] 值，不在解锁后保留任何指向栈上 [`WaitNode`] 的指针。
    fn pop(&mut self) -> Option<Waker> {
        if self.head.is_null() {
            return None;
        }
        // SAFETY:
        // 1. 当前持有 `WaitQueue::inner` 自旋锁，且上方已确认 `self.head` 非空；
        // 2. `self.head` 指向的头节点仍处于入队状态（`linked == true`），由其对应的 `QueueRegistration` 保证在本次 `remove` 脱链前内存有效且不被移动。
        let node = unsafe { &*self.head };
        self.remove(node)
    }
}

/// 多任务睡眠等待与唤醒的侵入式等待队列。
///
/// 内部通过自旋锁保护双向链表；在存在活跃 [`QueueRegistration`] 借用期间，编译期借用检查器会阻止含节点的队列被移动或提前释放。
pub struct WaitQueue {
    /// 保护双向链表头尾指针、节点计数与节点内部链接状态的自旋锁。
    inner: SpinLock<Queue>,
}

impl WaitQueue {
    /// 创建空的等待队列（支持静态常量初始化，不分配堆内存）。
    pub const fn new() -> Self {
        Self {
            inner: SpinLock::new(Queue::new()),
        }
    }

    /// 将栈上固定的 [`WaitNode`] 注册到当前等待队列尾部，并返回绑定生命周期的注销守卫 [`QueueRegistration`]。
    ///
    /// 入队后可变引用 `Pin<&'a mut WaitNode>` 会降级为共享引用 `Pin<&'a WaitNode>` 保存在守卫中，后续链表修改仅在队列自旋锁内通过 `UnsafeCell` 完成。
    ///
    /// # Safety
    /// 1. 调用者必须保证在返回的 [`QueueRegistration`] 完成析构（或节点被队列在自旋锁内脱链）之前，`self`（等待队列）与 `node`（等待节点）的底层存储始终有效且不被覆写。
    /// 2. 严禁对返回的 [`QueueRegistration`] 调用 `core::mem::forget` 或以其他方式跳过其 `Drop` 析构函数；否则当栈帧退栈释放 `node` 后，队列链表中将残留指向已释放栈内存的悬垂指针。
    pub unsafe fn register<'a>(&'a self, node: Pin<&'a mut WaitNode>) -> QueueRegistration<'a> {
        let node = node.into_ref();
        self.inner.lock().push(node);
        QueueRegistration { queue: self, node }
    }

    /// 按 FIFO 顺序从队头唤醒第一个成功接收通知的等待任务。
    ///
    /// 若队头节点的等待轮次已过期或此前已被其他路径唤醒（`waker.notify()` 返回 `false`），则继续弹出下一个节点，直到成功唤醒一个任务（返回 `true`）或队列排空（返回 `false`）。
    pub fn wake_one(&self) -> bool {
        loop {
            let waker = self.inner.lock().pop();
            let Some(waker) = waker else {
                return false;
            };
            if waker.notify() {
                return true;
            }
        }
    }

    /// 唤醒当前等待队列中的所有任务，并返回实际首次接收有效通知的任务数量。
    ///
    /// 每次仅在自旋锁内弹出一个节点，释放锁后再执行 `waker.notify()` 与 `Waker` 析构；遍历上限为进入本函数时的队列成员数（`members`），避免被唤醒任务立即重新入队导致无限循环。
    pub fn wake_all(&self) -> usize {
        self.wake_all_after_notify(|| {})
    }

    /// 测试辅助接口：在每次弹出节点并在锁外完成通知后执行 `after` 钩子闭包，用于确定性地构造唤醒期间并发重入队的交错场景。
    #[cfg(feature = "ktest")]
    pub fn test_wake_all_after_notify(&self, after: impl FnMut()) -> usize {
        self.wake_all_after_notify(after)
    }

    /// 测试辅助接口：读取当前等待队列中登记的节点数量。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_members(&self) -> usize {
        self.inner.lock().members
    }

    /// 批量唤醒内部实现：以入口时刻的队列长度 `members` 作为最大弹出配额，逐个在锁内弹出并在锁外通知。
    fn wake_all_after_notify(&self, mut after: impl FnMut()) -> usize {
        let mut remaining = self.inner.lock().members;
        let mut accepted = 0;
        while 0 < remaining {
            let waker = self.inner.lock().pop();
            let Some(waker) = waker else {
                return accepted;
            };
            remaining -= 1;
            if waker.notify() {
                accepted += 1;
            }
            drop(waker);
            after();
        }
        accepted
    }
}

impl Drop for WaitQueue {
    /// 析构时断言等待队列为空（无残留挂载节点且头指针与计数一致）。
    fn drop(&mut self) {
        let queue = self.inner.lock();
        assert_eq!(
            queue.head.is_null(),
            queue.members == 0,
            "wait queue head and member count disagree"
        );
        assert_eq!(queue.members, 0, "dropping a nonempty wait queue");
    }
}

/// 等待节点在 [`WaitQueue`] 上的注册守卫（RAII Guard）。
///
/// 持有对等待队列与固定节点的共享借用；当守卫离开作用域析构时，确保节点从队列链表中安全摘除后才允许释放栈上节点内存。
pub struct QueueRegistration<'a> {
    /// 节点所注册的目标等待队列引用。
    queue: &'a WaitQueue,
    /// 已注册的栈上固定等待节点引用。
    node: Pin<&'a WaitNode>,
}

impl Drop for QueueRegistration<'_> {
    /// 析构时确保节点从等待队列中脱链：若节点尚未被 `wake_one`/`wake_all` 摘走，则在队列锁内将其移除并在锁外析构剩余的 [`Waker`]。
    fn drop(&mut self) {
        let waker = self.queue.inner.lock().remove(self.node.get_ref());
        drop(waker);
    }
}
