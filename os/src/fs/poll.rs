//! I/O 事件轮询与就绪通知核心基础设施。
//!
//! # 模块职责与架构定位
//! 本模块连接底层的异步/流式事件源（管道 `pipe`、终端 `tty`、内核日志 `kmsg`、网络套接字 `socket`、
//! 以及 `epoll` 实例自身）与上层的事件多路复用及阻塞等待调用者（`ppoll`、`pselect`、`epoll_pwait`
//! 及各文件对象的阻塞读写循环）。
//!
//! # 核心设计与关键状态机
//!
//! ## 1. 持续事件登记与单次睡眠等待分离
//! - **持续事件源（[`PollWaitHead`]）**：嵌入在具体的文件或端点对象中，维护一条由 [`PollNode`]
//!   组成的单链表。当底层状态变为可读、可写、挂断或出错时，事件源调用 [`PollWaitHead::notify`]
//!   遍历链表并回调各订阅者的 [`PollNotify::notify`]。
//! - **有限容量登记表（[`PollTable`]）**：调用者在首次发现需要阻塞或建立持续监听时创建，一次性向事件源
//!   注册订阅节点并持有对应的 [`PollRegistration`] 句柄；析构时自动从各事件源链表中同步摘除节点。
//! - **防唤醒丢失的单线程阻塞上下文（[`PollContext`]）**：记录自上次清除以来是否收到过通知（`triggered`），
//!   仅在实际进入睡眠窗口（[`PollContext::park_until`]）时临时挂载栈上 `Waker`。
//!
//! ## 2. 标准阻塞等待时序（避免唤醒丢失）
//! ```text
//! 调用者线程                                      底层事件生产者 / 时钟中断
//! ----------------------------------------        ----------------------------------------
//! 1. 首次扫描发现未就绪 -> PollTable::register()
//! 2. loop:
//!      context.clear_triggered() (清 triggered)
//!      扫描业务状态 (poll(None) / 尝试读写)
//!                                                 状态就绪 -> PollWaitHead::notify()
//!                                                           -> context.notify() (triggered = true)
//!      发现仍需等待，释放业务锁
//!      context.park_until():
//!        持 state 自旋锁安装 waker
//!        检查 triggered == true 或已超时 ---------> 立即自唤醒 waker.notify()，避免睡死
//!        释放 state 锁并调用 waiter.park*()
//!        唤醒后摘除 waker，cancel_sync 停止定时器
//! ```
//!
//! ## 3. 事件源通知链表结构
//! ```text
//! PollWaitHead (state: SpinLock)
//!   └── chain ──> Pin<Arc<PollNode>> ──(next: SpinLock)──> Pin<Arc<PollNode>> ──> None
//!                   └── sink: Arc<dyn PollNotify>            └── sink: Arc<dyn PollNotify>
//!                         (PollContext 或 epoll EntrySink)
//! ```
//!
//! # 锁获取顺序与并发契约
//! 1. **禁止持锁进入睡眠**：调用 [`PollContext::park_until`] 时不得持有任何业务睡眠锁或文件表锁，
//!    也不得处于用户内存访问临界区。
//! 2. **事件源链表锁顺序**：[`PollWaitHead::state`] 自旋锁 -> 链表前驱 `PollNode::next` 自旋锁 ->
//!    后继 `PollNode::next` 自旋锁（严格沿链表正向获取）。
//! 3. **通知回调路径锁顺序**：[`PollWaitHead::state`] 自旋锁 -> 具体订阅者内部自旋锁（如
//!    [`PollContext`] 的 `state` 自旋锁，或 `epoll` 的 `ReadyCore::state` 自旋锁）-> 底层调度器唤醒锁。
//!    [`PollNotify::notify`] 实现严禁在回调内反向获取事件源的业务锁、分配堆内存或睡眠，否则会导致死锁。
//! 4. **锁外释放引用计数**：[`PollWaitHead::unlink`] 在自旋锁内仅断开链表指针，将摘下的节点强引用移交到
//!    自旋锁外再 `drop`，防止最后一个强引用析构时重入锁或执行耗时清理。

use alloc::{sync::Arc, vec::Vec};
use core::{
    ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not},
    pin::Pin,
};

use crate::time::{self, MonotonicInstant};
use crate::timer::{Timer, TimerKind};
use crate::wait::{WaitResult, Waiter, Waker};
use crate::{fs::FsError, sync::SpinLock};

/// 单线程轮询阻塞上下文，负责衔接异步就绪通知与线程睡眠唤醒。
///
/// 事件源的通知只在本结构中记录 `triggered` 标志并唤醒当前挂载的 `Waker`；
/// 线程的 `Waiter` 仅在调用 [`Self::park_until`] 睡眠期间临时挂载。
pub struct PollContext {
    /// 保护通知标志与当前睡眠唤醒句柄的自旋锁（可在硬中断与事件源通知路径中获取）。
    state: SpinLock<PollContextState>,
}

/// [`PollContext`] 受自旋锁保护的内部状态。
struct PollContextState {
    /// 自上次调用 [`PollContext::clear_triggered`] 以来，是否至少收到过一次事件源通知。
    triggered: bool,
    /// 当前正在 [`PollContext::park_until`] 中睡眠等待的线程唤醒句柄；未进入睡眠窗口时为 `None`。
    waker: Option<Waker>,
}

impl PollContext {
    /// 分配并初始化一个新的轮询阻塞上下文（快速非阻塞 I/O 路径无需创建本对象）。
    pub fn try_new() -> Result<Arc<Self>, FsError> {
        Arc::try_new(Self {
            state: SpinLock::new(PollContextState {
                triggered: false,
                waker: None,
            }),
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 在开始新一轮业务状态扫描前清除旧的 `triggered` 通知标志（严禁在业务状态扫描之后调用，否则会丢失扫描与睡眠之间的并发通知）。
    pub fn clear_triggered(&self) {
        self.state.lock().triggered = false;
    }

    /// 读取当前是否已记录未消费的通知标志（仅供内核测试观察，不查询底层事件源）。
    #[cfg(feature = "ktest")]
    pub fn triggered(&self) -> bool {
        self.state.lock().triggered
    }

    /// 在不持有业务锁的窗口内挂载当前线程的 `Waiter` 并阻塞等待，直到收到事件通知、到达截止时间或被信号中断。
    ///
    /// - `interruptible`：为 `true` 时允许被待处理的用户态信号中断并返回 [`WaitResult::Interrupted`]；
    /// - `deadline`：单调时钟绝对截止时间（单位：纳秒级 [`MonotonicInstant`]），为 `None` 时无限期等待且不注册定时器。
    pub fn park_until(
        &self,
        interruptible: bool,
        deadline: Option<MonotonicInstant>,
    ) -> WaitResult {
        // Step 1: 在当前栈帧上准备 Waiter，并在存在超时截止时间时固定（Pin）栈上 Waker 与高精度借用定时器。
        let waiter = if interruptible {
            Waiter::new_interruptible()
        } else {
            Waiter::new()
        };
        let waker = core::pin::pin!(deadline.map(|_| waiter.waker()));
        let timer = core::pin::pin!(waker.as_ref().as_pin_ref().map(|waker| {
            Timer::new(
                crate::cpu::current().id,
                TimerKind::Precise,
                poll_timeout,
                waker.get_ref() as *const Waker as usize,
            )
        }));
        if let Some(limit) = deadline {
            let timer = timer
                .as_ref()
                .as_pin_ref()
                .expect("poll deadline has no timer");
            // SAFETY:
            // 1. `timer` 与 `waker` 均通过 `core::pin::pin!` 固定在当前 `park_until` 栈帧上，地址在函数返回前保持稳定且不会移动；
            // 2. 函数下方唯一的返回路径在退栈前必定执行 `timer.cancel_sync()`，同步等待正在运行的时钟硬中断回调彻底退出后才销毁 `timer` 与 `waker`。
            unsafe { timer.arm_borrowed(limit) }.expect("fresh precise poll timer rejected");
        }

        // Step 2: 持自旋锁挂载 Waker；若在扫描后已收到通知（triggered 为 true）或截止时间已到，立即触发自唤醒以防漏掉通知而死睡。
        {
            let mut state = self.state.lock();
            assert!(state.waker.is_none(), "poll context already parked");
            state.waker = Some(waiter.waker());
            if state.triggered || deadline.is_some_and(|limit| limit <= time::now()) {
                state.waker.as_ref().unwrap().notify();
            }
        }

        // Step 3: 进入调度器阻塞等待，直至被事件源、超时定时器或信号唤醒。
        let result = if interruptible {
            waiter.park_signal()
        } else {
            waiter.park();
            WaitResult::Woken
        };

        // Step 4: 按严格顺序清理：先从上下文摘除 Waker 并销毁 Waiter，再调用 cancel_sync 同步等待高精度定时器回调彻底退出，最后安全退栈。
        let installed = self.state.lock().waker.take();
        drop(installed);
        drop(waiter);
        if let Some(timer) = timer.as_ref().as_pin_ref() {
            timer.cancel_sync();
        }
        result
    }
}

/// 高精度轮询超时定时器的时钟硬中断回调函数，负责唤醒固定在 `park_until` 栈帧上的 `Waker`。
fn poll_timeout(timer: Pin<&Timer>) {
    // SAFETY:
    // 1. `park_until` 构造 `Timer` 时传入的 `context` 是通过 `core::pin::pin!` 固定在其栈帧上的 `Waker` 共享引用地址，类型对齐与初始化均有效；
    // 2. `park_until` 在弹出栈帧并销毁该 `Waker` 之前必定调用 `timer.cancel_sync()`，保证本硬中断回调执行期间该栈上 `Waker` 持续存活且仅通过共享引用并发调用线程安全的 `Waker::notify()`。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

impl PollNotify for PollContext {
    /// 记录事件已触发标志，若当前存在正在 `park_until` 中睡眠的线程则立即将其唤醒；若线程尚未进入睡眠，保留 `triggered` 供其入睡前检查。
    fn notify(&self, _mask: PollMask) {
        let mut state = self.state.lock();
        state.triggered = true;
        if let Some(waker) = &state.waker {
            waker.notify();
        }
    }
}

/// 表示 `poll` / `select` / `epoll` 通用的 UAPI I/O 事件位掩码（元组字段 `0` 为按位或组合的 `POLL*` 标志位集合）。
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PollMask(pub u32);

impl PollMask {
    /// 表示有数据可读。
    pub const POLLIN: Self = Self(0x001);
    /// 表示有带外数据可读。
    pub const POLLPRI: Self = Self(0x002);
    /// 表示写入不会阻塞。
    pub const POLLOUT: Self = Self(0x004);
    /// 表示文件对象发生错误。
    pub const POLLERR: Self = Self(0x008);
    /// 表示文件对象已挂断。
    pub const POLLHUP: Self = Self(0x010);
    /// 表示文件描述符无效。
    pub const POLLNVAL: Self = Self(0x020);
    /// 表示普通数据可读。
    pub const POLLRDNORM: Self = Self(0x040);
    /// 表示带优先级数据可读。
    pub const POLLRDBAND: Self = Self(0x080);
    /// 表示普通写入不会阻塞。
    pub const POLLWRNORM: Self = Self(0x100);
    /// 表示带优先级写入不会阻塞。
    pub const POLLWRBAND: Self = Self(0x200);
    /// 表示消息事件。
    #[expect(
        dead_code,
        reason = "保留 poll UAPI 的 0x400 消息位定义，当前 provider 不产生该事件"
    )]
    pub const POLLMSG: Self = Self(0x400);
    /// 表示删除事件。
    #[expect(
        dead_code,
        reason = "保留 poll UAPI 的 0x1000 删除位定义，当前 provider 不产生该事件"
    )]
    pub const POLLREMOVE: Self = Self(0x1000);
    /// 表示对端已半关闭写侧。
    pub const POLLRDHUP: Self = Self(0x2000);

    /// 返回该掩码是否包含给定的全部位。
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// 返回该掩码是否没有任何位。
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl BitOr for PollMask {
    type Output = Self;

    /// 合并两个 poll 位集合。
    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for PollMask {
    /// 将右侧 poll 位加入当前集合。
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl BitAnd for PollMask {
    type Output = Self;

    /// 保留两个 poll 位集合的交集。
    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl BitAndAssign for PollMask {
    /// 将当前集合限制为与右侧集合的交集。
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}

impl BitXor for PollMask {
    type Output = Self;

    /// 保留两个 poll 位集合中只出现一侧的位。
    fn bitxor(self, rhs: Self) -> Self::Output {
        Self(self.0 ^ rhs.0)
    }
}

impl BitXorAssign for PollMask {
    /// 将当前集合与右侧集合做异或。
    fn bitxor_assign(&mut self, rhs: Self) {
        self.0 ^= rhs.0;
    }
}

impl Not for PollMask {
    type Output = Self;

    /// 反转 poll 位集合中的全部 UAPI 位。
    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// 支持 I/O 就绪状态查询与持续事件源注册的文件或端点抽象接口。
///
/// 普通磁盘文件总是立即读写就绪，不实现本 trait；仅管道、终端、套接字、`kmsg`、`epoll` 等流式或异步事件对象实现本接口。
pub trait Pollable: Send + Sync {
    /// 可选地将底层事件源注册到 `table`，并查询返回对象当前的即时就绪事件掩码。
    ///
    /// - 当 `table` 为 `Some` 时（首轮建立监听），实现必须先将自身所有可能触发状态变化的 [`PollWaitHead`] 注册到 `table`，再采样返回当前状态，确保不会漏掉注册后发生的状态变化；
    /// - 当 `table` 为 `None` 时（后续复查状态），仅无副作用地采样返回当前就绪事件掩码，不重复分配或注册等待节点。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError>;
}

/// 接收底层事件源状态变化通知的订阅者回调接口。
///
/// 实现方必须在极短时间内完成处理（可在硬中断或事件源内部自旋锁临界区中被调用），严禁在 `notify` 内睡眠、分配堆内存或反向获取事件源的业务锁。
pub trait PollNotify: Send + Sync {
    /// 通知订阅者底层事件源产生了 `mask` 所指示的状态变化（`mask` 为空时表示通用唤醒重查）。
    fn notify(&self, mask: PollMask);
}

/// 挂载在 [`PollWaitHead`] 单链表上的单个持续事件订阅节点。
struct PollNode {
    /// 接收就绪通知的目标回调对象（如 [`PollContext`] 或 `epoll` 的 `EntrySink`）。
    sink: Arc<dyn PollNotify>,
    /// 指向链表下一个订阅节点的指针，由自旋锁保护以支持在持有头锁时安全遍历与摘链。
    next: SpinLock<Option<Pin<Arc<PollNode>>>>,
}

/// [`PollWaitHead`] 受自旋锁保护的链表头状态。
struct PollWaitState {
    /// 已注册订阅节点的单链表首元指针；无订阅者时为 `None`。
    chain: Option<Pin<Arc<PollNode>>>,
}

/// 由具体文件或端点对象持有的持续 `poll` 事件等待队列头。
pub struct PollWaitHead {
    /// 保护订阅者单链表增删与遍历的自旋锁。
    state: SpinLock<PollWaitState>,
}

impl PollWaitHead {
    /// 创建一个尚未挂载任何订阅节点的空事件等待队列头。
    pub const fn new() -> Self {
        Self {
            state: SpinLock::new(PollWaitState { chain: None }),
        }
    }

    /// 在队列头自旋锁保护下顺序遍历并通知所有已注册的订阅者；全程不分配内存、不睡眠。
    pub fn notify(&self, mask: PollMask) {
        let state = self.state.lock();
        let mut current = state.chain.as_ref().cloned();
        while let Some(node) = current {
            node.sink.notify(mask);
            current = {
                let next = node.next.lock();
                next.as_ref().cloned()
            };
        }
    }

    /// 分配新的订阅节点并将其头插法挂入当前等待队列头，返回负责生命周期自动注销的 [`PollRegistration`] 句柄。
    pub fn register(
        self: &Arc<Self>,
        sink: Arc<dyn PollNotify>,
    ) -> Result<PollRegistration, FsError> {
        let node = Pin::new(
            Arc::try_new(PollNode {
                sink,
                next: SpinLock::new(None),
            })
            .map_err(|_| FsError::NoMemory)?,
        );

        let mut state = self.state.lock();
        {
            let mut next = node.next.lock();
            *next = state.chain.take();
        }
        state.chain = Some(node.clone());
        Ok(PollRegistration {
            head: Arc::clone(self),
            node,
        })
    }

    /// 从等待队列链表中同步查找并摘除指定节点 `target`，并将被摘下的节点强引用移交到自旋锁外释放。
    fn unlink(&self, target: &Pin<Arc<PollNode>>) {
        // Step 1: 持有队列头自旋锁，定位目标节点并修改前驱指针将其从链表中断开。
        let detached = {
            let mut state = self.state.lock();
            if state
                .chain
                .as_ref()
                .is_some_and(|node| same_node(node, target))
            {
                let detached = state.chain.take();
                let successor = detached.as_ref().and_then(|node| {
                    let mut next = node.next.lock();
                    next.take()
                });
                state.chain = successor;
                detached
            } else {
                let mut current = state.chain.as_ref().cloned();
                let mut detached = None;
                while let Some(node) = current {
                    let mut next = node.next.lock();
                    if next
                        .as_ref()
                        .is_some_and(|candidate| same_node(candidate, target))
                    {
                        let removed = next.take();
                        let successor = removed.as_ref().and_then(|candidate| {
                            let mut following = candidate.next.lock();
                            following.take()
                        });
                        *next = successor;
                        detached = removed;
                        break;
                    }
                    current = next.as_ref().cloned();
                }
                detached
            }
        };

        // Step 2: 在释放队列头自旋锁之后再销毁摘下的节点强引用，防止析构逻辑在自旋锁临界区内重入。
        drop(detached);
    }
}

/// 单个持续 `poll` 事件源注册关系的 RAII 注销句柄。
///
/// 当本句柄被销毁（Drop）时，会自动将对应的订阅节点从 [`PollWaitHead`] 链表中同步摘除。
pub struct PollRegistration {
    /// 订阅节点所属的事件源等待队列头强引用。
    head: Arc<PollWaitHead>,
    /// 挂载在队列链表中的固定地址订阅节点。
    node: Pin<Arc<PollNode>>,
}

impl Drop for PollRegistration {
    /// 销毁句柄时从事件源队列头同步摘除节点，并在锁外完成引用释放。
    fn drop(&mut self) {
        self.head.unlink(&self.node);
    }
}

/// 比较两个固定订阅节点是否指向同一块底层 `PollNode` 内存分配。
fn same_node(left: &Pin<Arc<PollNode>>, right: &Pin<Arc<PollNode>>) -> bool {
    core::ptr::eq(Pin::as_ref(left).get_ref(), Pin::as_ref(right).get_ref())
}

/// 管理一次轮询或一个 `epoll` 监听项所订阅的全部事件源句柄集合。
///
/// 构造时一次性预留固定容量，避免在持有业务锁或首轮扫描中途触发隐式扩容。
pub struct PollTable {
    /// 所有注册项共享的事件通知回调接收端。
    sink: Arc<dyn PollNotify>,
    /// 已成功注册的事件源句柄列表（容量在构造时固定，每个不同的 [`PollWaitHead`] 至多保存一项）。
    registrations: Vec<PollRegistration>,
}

impl PollTable {
    /// 创建绑定到 `sink` 且精确预分配 `capacity` 个事件源槽位的登记表；内存不足时返回 `FsError::NoMemory`。
    pub fn new(sink: Arc<dyn PollNotify>, capacity: usize) -> Result<Self, FsError> {
        let mut registrations = Vec::new();
        registrations
            .try_reserve_exact(capacity)
            .map_err(|_| FsError::NoMemory)?;
        Ok(Self {
            sink,
            registrations,
        })
    }

    /// 将事件源 `head` 注册到当前登记表中；若 `head` 已在本表中注册过则直接成功返回，若超出预留容量则返回 `FsError::NoMemory`。
    pub fn register(&mut self, head: &Arc<PollWaitHead>) -> Result<(), FsError> {
        if self
            .registrations
            .iter()
            .any(|registration| Arc::ptr_eq(&registration.head, head))
        {
            return Ok(());
        }
        if self.registrations.len() == self.registrations.capacity() {
            return Err(FsError::NoMemory);
        }
        let registration = head.register(Arc::clone(&self.sink))?;
        self.registrations.push(registration);
        Ok(())
    }

    /// 返回当前登记表中已注册的不同事件源数量。
    #[cfg(feature = "ktest")]
    pub fn len(&self) -> usize {
        self.registrations.len()
    }

    /// 返回当前登记表是否尚未注册任何事件源。
    #[cfg(feature = "ktest")]
    #[expect(
        dead_code,
        reason = "与已有 len 共同提供同一 poll 登记集合的长度和空集合观察接口"
    )]
    pub fn is_empty(&self) -> bool {
        self.registrations.is_empty()
    }
}

impl Drop for PollTable {
    /// 按后进先出顺序依次销毁所有事件源注册句柄，同步从各 [`PollWaitHead`] 摘链并释放引用。
    fn drop(&mut self) {
        while let Some(registration) = self.registrations.pop() {
            drop(registration);
        }
    }
}
