//! 网络出站容量等待队列与协议端点异步回收链表。
//!
//! # 模块职责与架构定位
//! 本模块位于网络协议栈根实例（`crate::net::NetStack`）内部，负责管理两类生命周期敏感的侵入式单向链表：
//! 1. **出站发送容量等待链表（[`EgressWaiting`] / [`EgressWait`]）**：当网络栈共享出站队列满时，TCP/UDP 端点将轻量级等待节点挂入此链表；待出站队列腾出槽位后，由网络栈按先进先出（FIFO）顺序摘取节点并按端点 ID 定向唤醒。
//! 2. **已退役端点回收链表（[`RetiredEndpoints`] / [`RetiredEndpoint`]）**：当 TCP/UDP 端点关闭并从全局端点表（`NetTables`）中移出后，不能在自身工作任务（Work）或定时器回调上下文中直接同步等待自身退出（否则会导致死锁）。因此将端点强引用挂入此链表，交由独立的后台回收任务（Reaper Work）在锁外调用 `stop_sync()` 等待所有异步回调与工作任务彻底执行完毕。
//!
//! # 核心设计与链表结构
//!
//! ```text
//! 1. 出站容量等待队列 (FIFO, 轻量 ID 节点，不持有端点强引用):
//! EgressWaiting
//!   +-- head --> [ EgressWait (id=1) ] --next--> [ EgressWait (id=2) ] --next--> None
//!   +-- tail ------------------------------------^
//!
//! 2. 退役端点回收栈 (LIFO, 内嵌 RetirementLink 侵入式链接，持有端点 Arc):
//! RetiredEndpoints
//!   +-- head --> [ Tcp/UdpEndpoint A ] --retirement.next--> [ Tcp/UdpEndpoint B ] --> None
//! ```
//!
//! # 锁获取顺序与并发契约
//! - **出站等待链**：外层为 `NetStack::egress` 自旋锁，内层为 [`EgressWait::projection`] 自旋锁。所有队列修改必须先持有外层 `egress` 锁，再短暂获取单个节点的 `projection` 锁；严禁在持有 `projection` 锁时反向获取 `egress` 锁，也不得同时锁住两个不同节点的 `projection` 锁。
//! - **退役回收链**：外层为 `NetStack::retired` 自旋锁，内层为 [`RetirementLink::projection`] 自旋锁。入链与出链均由外层 `retired` 锁串行化，摘取出的 [`RetiredEndpoint`] 必须在释放外层 `retired` 锁之后再执行同步停止（`stop_sync`）与析构。

use alloc::sync::Arc;

use crate::fs::FsError;
use crate::net::tcp::TcpEndpoint;
use crate::net::udp::UdpEndpoint;
use crate::sync::SpinLock;

/// 出站发送容量等待节点。
///
/// 在 TCP/UDP 端点创建时预先分配。节点仅保存端点的唯一整数编号，不持有协议端点本身的强引用（`Arc`），
/// 从而避免等待队列意外延长已关闭端点的生命周期。
pub(crate) struct EgressWait {
    /// 关联的协议端点全局唯一编号（Endpoint ID）。
    id: u64,
    /// 侵入式单向链表链接状态，由自旋锁保护。
    projection: SpinLock<WaitProjection>,
}

/// [`EgressWait`] 内部受自旋锁保护的链表指针与在链标志。
struct WaitProjection {
    /// 标记当前节点是否已挂入 [`EgressWaiting`] 队列中，防止重复入队破坏链表结构。
    linked: bool,
    /// 指向队列中的下一个等待节点。
    next: Option<Arc<EgressWait>>,
}
impl EgressWait {
    /// 在端点初始化阶段预先分配等待节点，初始状态为未入链（`linked = false`）。
    ///
    /// 若内核堆内存不足则返回 [`FsError::NoMemory`]。
    pub(crate) fn try_new(id: u64) -> Result<Arc<Self>, FsError> {
        Arc::try_new(Self {
            id,
            projection: SpinLock::new(WaitProjection {
                linked: false,
                next: None,
            }),
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 返回该等待节点对应的协议端点唯一编号（Endpoint ID）。
    ///
    /// 唤醒路径仅凭此编号在 `NetTables` 中查找存活的协议端点，而不直接持有端点引用。
    pub(crate) fn id(&self) -> u64 {
        self.id
    }

    /// 查询当前节点是否已经挂入出站等待队列。
    ///
    /// 调用者通常在持有外层 `NetStack::egress` 自旋锁时调用此方法，使容量检查与节点入队保持原子性。
    pub(crate) fn linked(&self) -> bool {
        self.projection.lock().linked
    }
}

impl Drop for EgressWait {
    /// 最后一个强引用（`Arc<EgressWait>`）被释放（Drop）时，节点必须已从等待队列中摘除，
    /// 既防止丢失唤醒，也避免顺着 `next` 指针链式递归析构后续节点导致内核栈溢出。
    fn drop(&mut self) {
        let projection = self.projection.lock();
        assert!(
            !projection.linked && projection.next.is_none(),
            "linked egress waiter dropped"
        );
    }
}

/// 网络栈出站发送容量等待队列（FIFO 单向链表）。
///
/// 所有公开方法均由外层 `NetStack::egress` 自旋锁保护并串行执行。
pub(crate) struct EgressWaiting {
    /// 队列头部节点（最早进入等待的节点）。
    head: Option<Arc<EgressWait>>,
    /// 队列尾部节点（最近进入等待的节点）。
    tail: Option<Arc<EgressWait>>,
    /// 当前在队列中等待的节点总数（单位：个）。
    len: usize,
}

impl EgressWaiting {
    /// 创建一个空的出站容量等待队列。
    pub(crate) const fn new() -> Self {
        Self {
            head: None,
            tail: None,
            len: 0,
        }
    }

    /// 返回当前等待队列中的节点数量，供唤醒循环确定本轮最多处理的节点数上限。
    pub(crate) fn len(&self) -> usize {
        self.len
    }

    /// 将等待节点追加到队列尾部。
    ///
    /// 若该节点已在队列中（`linked == true`），则直接返回并保持其原有排队位置不变，不会触发堆内存分配。
    pub(crate) fn push(&mut self, node: &Arc<EgressWait>) {
        let mut projection = node.projection.lock();
        if projection.linked {
            return;
        }
        assert!(
            projection.next.is_none(),
            "unlinked waiter retains successor"
        );
        projection.linked = true;
        drop(projection);

        if let Some(tail) = self.tail.as_ref() {
            tail.projection.lock().next = Some(node.clone());
        } else {
            self.head = Some(node.clone());
        }
        self.tail = Some(node.clone());
        self.len = self
            .len
            .checked_add(1)
            .expect("egress waiter count overflow");
    }

    /// 从队列头部弹出一个等待节点并清空其后继指针。
    ///
    /// 调用方应在释放外层 `egress` 自旋锁后再根据返回的节点 ID 唤醒端点并释放该 `Arc`。
    pub(crate) fn pop(&mut self) -> Option<Arc<EgressWait>> {
        let node = self.head.take()?;
        let mut projection = node.projection.lock();
        assert!(projection.linked, "egress head is not linked");
        self.head = projection.next.take();
        projection.linked = false;
        drop(projection);

        self.len -= 1;
        if self.head.is_none() {
            self.tail.take();
        }
        Some(node)
    }

    /// 检查指定端点编号 `id` 是否存在于当前等待队列中（仅用于 `ktest` 断言观察）。
    #[cfg(feature = "ktest")]
    pub(crate) fn contains(&self, id: u64) -> bool {
        let mut current = self.head.clone();
        while let Some(node) = current {
            if node.id == id {
                return true;
            }
            current = node.projection.lock().next.clone();
        }
        false
    }

    /// 从等待队列中查找并移除指定端点编号 `id` 对应的等待节点。
    ///
    /// 用于端点关闭或退役时主动取消出站等待；调用方应在释放外层 `egress` 自旋锁后释放返回的 `Arc`。
    pub(crate) fn remove(&mut self, id: u64) -> Option<Arc<EgressWait>> {
        if self.head.as_ref().is_some_and(|node| node.id == id) {
            return self.pop();
        }

        let mut previous = self.head.clone()?;
        loop {
            let node = previous.projection.lock().next.clone()?;
            if node.id == id {
                let mut projection = node.projection.lock();
                let next = projection.next.take();
                projection.linked = false;
                drop(projection);

                previous.projection.lock().next = next;
                if self.tail.as_ref().is_some_and(|tail| tail.id == id) {
                    self.tail = Some(previous);
                }
                self.len -= 1;
                return Some(node);
            }
            previous = node;
        }
    }
}

impl Drop for EgressWaiting {
    /// 网络栈销毁前必须逐个弹出并清空所有等待节点，严禁带着非空链表直接析构。
    fn drop(&mut self) {
        assert!(
            self.head.is_none() && self.tail.is_none() && self.len == 0,
            "egress waiting chain not drained"
        );
    }
}

/// 已从全局端点表（`NetTables`）中移除、等待后台回收任务同步停止的协议端点。
pub(crate) enum RetiredEndpoint {
    /// 待回收的 TCP 端点强引用。
    Tcp(Arc<TcpEndpoint>),
    /// 待回收的 UDP 端点强引用。
    Udp(Arc<UdpEndpoint>),
}

/// [`RetirementLink`] 内部受自旋锁保护的侵入式单向链表状态。
struct RetirementProjection {
    /// 标记该端点当前是否已挂入 [`RetiredEndpoints`] 回收链表中。
    linked: bool,
    /// 指向回收链表中的下一个待回收端点。
    next: Option<RetiredEndpoint>,
}

/// 内嵌在每个 TCP/UDP 端点中的退役回收链表节点。
///
/// 与端点自身的协议状态锁完全分离，使网络栈无需获取端点业务锁即可将其挂入回收链表。
pub(crate) struct RetirementLink {
    /// 受自旋锁保护的链表指针与在链状态。
    projection: SpinLock<RetirementProjection>,
}

impl RetirementLink {
    /// 创建初始未入链的退役链表节点。
    pub(crate) const fn new() -> Self {
        Self {
            projection: SpinLock::new(RetirementProjection {
                linked: false,
                next: None,
            }),
        }
    }
}

impl Drop for RetirementLink {
    /// 端点的最后一个强引用（`Arc`）被释放（Drop）前，回收任务必须已将其从退役链表中摘除。
    fn drop(&mut self) {
        let projection = self.projection.lock();
        assert!(
            !projection.linked && projection.next.is_none(),
            "linked protocol endpoint dropped"
        );
    }
}

impl RetiredEndpoint {
    /// 获取内嵌在具体 TCP 或 UDP 端点中的 [`RetirementLink`] 引用。
    fn link(&self) -> &RetirementLink {
        match self {
            Self::Tcp(endpoint) => &endpoint.retirement,
            Self::Udp(endpoint) => &endpoint.retirement,
        }
    }

    /// 在外层 `retired` 自旋锁之外同步停止端点的定时器（Timer）与工作任务（Work），
    /// 确保所有异步回调彻底执行完毕并退出后再释放端点内存。
    pub(crate) fn stop_sync(self) {
        match self {
            Self::Tcp(endpoint) => endpoint.stop_sync(),
            Self::Udp(endpoint) => endpoint.stop_sync(),
        }
    }
}

/// 待回收协议端点的侵入式单向链表（LIFO 栈结构）。
///
/// 根节点仅保存链表头 `head`，后续节点通过各自内嵌的 [`RetirementLink`] 依次串联保活。
/// 所有操作由外层 `NetStack::retired` 自旋锁串行保护。
pub(crate) struct RetiredEndpoints {
    /// 链表头部的待回收端点。
    head: Option<RetiredEndpoint>,
}

impl RetiredEndpoints {
    /// 创建一个空的退役端点回收链表。
    pub(crate) const fn new() -> Self {
        Self { head: None }
    }

    /// 将刚从全局端点表中移出的端点压入回收链表头部。
    ///
    /// 每个端点在其生命周期内至多只能入链一次；通过入链转交所有权，避免在当前工作任务或定时器回调中同步等待自身停止。
    pub(crate) fn push(&mut self, endpoint: RetiredEndpoint) {
        let mut projection = endpoint.link().projection.lock();
        assert!(
            !projection.linked && projection.next.is_none(),
            "protocol endpoint retired twice"
        );
        projection.linked = true;
        projection.next = self.head.take();
        drop(projection);

        self.head = Some(endpoint);
    }

    /// 从回收链表头部弹出一个待回收端点并断开其 `next` 链接。
    ///
    /// 调用者（后台 Reaper 任务）必须在释放外层 `retired` 自旋锁之后，再对该端点调用 [`RetiredEndpoint::stop_sync`]。
    pub(crate) fn pop(&mut self) -> Option<RetiredEndpoint> {
        let endpoint = self.head.take()?;
        let mut projection = endpoint.link().projection.lock();
        assert!(projection.linked, "retired head is not linked");
        self.head = projection.next.take();
        projection.linked = false;
        drop(projection);
        Some(endpoint)
    }

    /// 判断回收链表当前是否为空，用于后台回收任务决定是否需要继续调度下一轮回收。
    pub(crate) fn is_empty(&self) -> bool {
        self.head.is_none()
    }
}

impl Drop for RetiredEndpoints {
    /// 网络栈停止并销毁前必须排空并同步停止所有退役端点，防止链表递归析构或遗留未停止的后台任务。
    fn drop(&mut self) {
        assert!(self.head.is_none(), "retired endpoint chain not drained");
    }
}
