//! TCP 传输端点（`TcpEndpoint`）与连接生命周期管理。
//!
//! # 模块职责与架构定位
//! 本模块位于网络协议栈的传输层，向上承接 `net::socket` 中的流式套接字文件描述，向下依赖：
//! - `net::core::TcpCore`：封装单连接 TCP 状态机、收发环形缓冲区与超时计算；
//! - `net::table::NetTables`：维护全局端口绑定租约（`BindLease`）、监听表与四元组索引；
//! - `net::NetStack`：负责底层路由解析、IP 报文发送与接收分发，以及端点异步退休回收；
//! - `workqueue::Work` 与 `timer::Timer`：分别在绑定的目标 CPU 上推进后台协议收发和精确超时重传。
//!
//! # 核心设计与状态流转
//!
//! ## 1. 操作互斥与后台 Work 交接
//! 每个 `TcpEndpoint` 拥有一把可睡眠互斥锁 `core: SleepMutex<TcpCore>`，用于在用户态任务和后台 `Work`
//! 之间串行化协议状态访问。当网卡或回环收到输入报文时，只获取短时间的 `input` 自旋锁将报文推入队列，
//! 并调度后台 `Work`。若后台 `Work` 执行时发现用户态正持有 `core` 睡眠锁，则将 `input.owner_waiting`
//! 置为 `true` 后立即退出；用户态操作守卫 `TcpOperationGuard` 释放 `core` 锁时，会检查该标志并替后台
//! 重新调度 `Work`，避免后台工作线程阻塞等待。
//!
//! ```text
//!   [用户态 syscall]                         [网卡 / 回环收包]
//!          |                                        |
//!   lock() / observe()                        enqueue(packet)
//!   获取 core 睡眠锁                          获取 input 自旋锁入队
//!          |                                        |
//!          v                                        v
//!   修改或观察 TcpCore                       schedule() 投递 Work
//!          |                                        |
//!          |                              run_tcp() -> try_lock(core)
//!          |                              若失败：置 input.owner_waiting = true
//!          v                                        |
//!   Drop(TcpOperationGuard) <-----------------------+
//!   1. 计算 PollMask 增量并唤醒等待者
//!   2. 解锁 core
//!   3. 若为 Modify 或 owner_waiting 为 true，调用 schedule() 重新推进后台 Work
//! ```
//!
//! ## 2. 监听套接字与被动连接队列（Backlog）
//! ```text
//!   Listener (State::Listen)
//!     |
//!     +-- 收到 SYN 报文 (receive_syn)
//!           |
//!           v
//!     创建子端点 (Child TcpEndpoint, State::Listen -> SynReceived)
//!     加入 listener.children.pending (半连接队列)
//!           |
//!           +-- 收到三次握手 ACK (advance -> report_established)
//!                 |
//!                 v
//!           从 pending 移入 listener.children.ready (全连接队列)
//!           唤醒父监听端点的 POLLIN 等待队列 -> accept() 取出返回给用户态
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 严禁违反以下锁层级顺序，否则会导致死锁：
//! 1. **连接核心睡眠锁 `TcpEndpoint::core`（`SleepMutex`）**：最外层锁，持锁期间可分配内存，但严禁反向等待自身 `Work` 结束。
//!    在被动建连 `receive_syn` 初始化尚未发布的子端点时，允许在持有父端点 `core` 锁的情况下短暂获取新子端点的 `child.core` 锁。
//! 2. **网络栈控制准入锁 `NetStack::admission()`（`SleepMutex`）**：在 `bind`、`listen`、`connect`、`receive_syn`
//!    及套接字选项修改时获取，位于 `core` 锁之内、所有 `SpinLock` 之外。
//! 3. **端点内部自旋锁（`SpinLock`）**：包括 `input`、`metadata`、`children` 以及 `NetTables` 内部自旋锁。
//!    这些自旋锁临界区极短且互不嵌套；必须在释放自旋锁之后才能调用 `PollWaitHead::notify`、`schedule` 或析构子端点 `Arc`。

use alloc::collections::VecDeque;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::ops::{Deref, DerefMut};
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use smoltcp::socket::tcp::{CongestionControl, State};
use smoltcp::wire::{IpAddress, TcpControl};
use spin::Once;

use crate::fs::poll::{PollMask, PollTable, PollWaitHead};
use crate::fs::{FsError, ReadBuffer, WriteBuffer};
use crate::net::address::{AddressFamily, InetEndpoint};
use crate::net::core::{ProtocolDeadline, TcpCore};
use crate::net::lifecycle::{EgressWait, RetirementLink};
use crate::net::packet::OwnedPacket;
use crate::net::socket::{SocketTimeouts, SHUT_RD, SHUT_RDWR, SHUT_WR};
use crate::net::table::{BindLease, TcpTuple, TransportProtocol};
use crate::net::{
    EndpointSlot, IoProgress, NetStack, NetStackRef, LOOPBACK_CAPACITY, PROTOCOL_BUDGET, SOMAXCONN,
};
use crate::sync::{SleepMutex, SleepMutexGuard, SpinLock};
use crate::timer::{Timer, TimerKind};
use crate::workqueue::Work;
use crate::{time, trace};

/// 端点的输入缓冲队列与后台调度交接状态（由 `TcpEndpoint::input` 自旋锁保护）。
struct Input {
    /// 端点是否已关闭输入接收；置位后拒绝新报文入队。
    closed: bool,
    /// 标记后台 `Work` 在尝试获取 `core` 睡眠锁时是否因锁被占用而退让；
    /// 当前持有 `core` 锁的操作守卫在释放锁时负责接力重新调度该 `Work`。
    owner_waiting: bool,
    /// 待由后台 `Work` 消费的输入 IP/TCP 报文有界队列（容量上限为 `LOOPBACK_CAPACITY`）。
    packets: VecDeque<OwnedPacket>,
    /// 上一次处理输入报文时产生但因发送通道暂满（`WouldBlock`）尚未发出的立即控制应答报文（如 SYN-ACK、ACK、RST）。
    reply: Option<OwnedPacket>,
}

/// 端点的绑定租约、父子关联及套接字控制属性（由 `TcpEndpoint::metadata` 自旋锁保护）。
struct Metadata {
    /// 当前占用的本地地址与端口绑定租约；未绑定时为 `None`，被接受的子连接与父监听端点共享同一租约。
    lease: Option<Arc<BindLease>>,
    /// 若当前端点为监听套接字派生的被动连接子端点，指向其父监听端点的弱引用。
    parent: Weak<TcpEndpoint>,
    /// 标记端点是否已进入活动生命周期（如已执行 `listen`、`connect`、`close`、`abandon` 或作为被动子连接建立）。
    engaged: bool,
    /// 待由用户态通过读写操作或 `SO_ERROR` 取走的异步协议错误（如 `ConnectionRefused`、`ConnectionReset`）。
    error: Option<FsError>,
    /// 标记连接是否曾成功完成三次握手进入 `Established` 或 `CloseWait` 状态。
    connected: bool,
    /// 读方向是否已通过 `shutdown(SHUT_RD)` 或 `close` 关闭。
    read_closed: bool,
    /// 写方向是否已通过 `shutdown(SHUT_WR)` 或 `close` 关闭。
    write_closed: bool,
    /// 是否启用 `SO_REUSEADDR` 地址复用选项。
    reuse: bool,
    /// IPv6 端点是否启用 `IPV6_V6ONLY`（仅限 IPv6 通信，禁止 IPv4 映射地址）。
    v6_only: bool,
    /// 收发方向的阻塞等待超时配置（`SO_RCVTIMEO` 与 `SO_SNDTIMEO`）。
    timeouts: SocketTimeouts,
}

/// 监听端点的被动连接子端点队列（由 `TcpEndpoint::children` 自旋锁保护）。
struct Children {
    /// 半连接队列与全连接队列共享的容量上限（由 `listen(backlog)` 设定，限制在 `1..=SOMAXCONN`）。
    limit: usize,
    /// 收到 SYN 后已创建但尚未完成三次握手的半连接子端点列表。
    pending: Vec<Arc<TcpEndpoint>>,
    /// 已完成三次握手（进入 `Established` 或 `CloseWait`）、等待 `accept` 取走的全连接子端点队列。
    ready: VecDeque<Arc<TcpEndpoint>>,
}

/// TCP 传输层端点，封装单条连接或监听套接字的完整协议状态、队列与异步执行上下文。
pub struct TcpEndpoint {
    /// 网络栈内全局唯一且不复用的端点标识号。
    id: u64,
    /// 端点创建时固定的地址族（`AF_INET` 或 `AF_INET6`）。
    family: AddressFamily,
    /// 所属网络栈实例的弱引用。
    stack: Weak<NetStack>,
    /// 保护底层 `TcpCore` 协议状态机与收发环形缓冲区的可睡眠互斥锁。
    core: SleepMutex<TcpCore>,
    /// 保护输入报文队列与后台 `Work` 交接标志的自旋锁。
    input: SpinLock<Input>,
    /// 保护端口租约、连接标志、异步错误与套接字选项的自旋锁。
    metadata: SpinLock<Metadata>,
    /// 保护监听套接字半连接与全连接子端点队列的自旋锁。
    children: SpinLock<Children>,
    /// 供 `poll` / `select` / `epoll` 注册和唤醒的轮询等待队列头。
    source: Arc<PollWaitHead>,
    /// 发送通道（Egress）容量不足时的轻量级等待链表节点。
    egress_wait: Arc<EgressWait>,
    /// 端点进入异步退休链表时的侵入式链接状态。
    pub(crate) retirement: RetirementLink,
    /// 固定在指定 CPU 上推进协议收发处理的后台工作项。
    work: Once<Pin<Arc<Work>>>,
    /// 固定在同一 CPU 上处理协议重传、延时 ACK 与 `TIME_WAIT` 超时的高精度定时器。
    timer: Once<Pin<Arc<Timer>>>,
    /// 标记端点是否已进入同步停止流程（`stop_sync`），置位后拒绝新调度请求。
    stopped: AtomicBool,
    /// 占用网络栈全局端点配额的槽位守卫，随端点析构自动归还配额。
    _slot: EndpointSlot,
}

/// 获取 `TcpEndpoint::core` 睡眠锁时的操作模式。
#[derive(Clone, Copy, Eq, PartialEq)]
enum OperationMode {
    /// 修改模式：操作可能改变协议状态或收发队列，释放锁后需要计算事件通知并主动调度后台 `Work`。
    Modify,
    /// 观察模式：仅读取协议状态，释放锁时不触发事件通知，仅在后台 `Work` 曾受阻退让（`owner_waiting`）时接力调度。
    Observe,
}

/// 单次持有 `core` 锁前后的状态快照，用于对比并提取新产生的 `PollMask` 事件通知。
struct Observation {
    /// 接收环形缓冲区中的可读字节数。
    received: usize,
    /// 发送环形缓冲区中尚未发完或未确认的字节数。
    sent: usize,
    /// 当前端点计算出的完整 `PollMask` 就绪掩码。
    mask: PollMask,
    /// 监听端点全连接队列（`ready`）中的子连接数量。
    ready_children: usize,
}

impl Observation {
    /// 对比操作前后的两次状态快照，提取本次操作期间新变为就绪的轮询事件掩码。
    fn notification_since(&self, previous: &Self) -> PollMask {
        let mut notification = self.mask & !previous.mask;
        if previous.received < self.received || previous.ready_children < self.ready_children {
            notification |= self.mask & (PollMask::POLLIN | PollMask::POLLRDNORM);
        }
        if self.mask.contains(PollMask::POLLRDHUP) && !previous.mask.contains(PollMask::POLLRDHUP) {
            notification |=
                self.mask & (PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLRDHUP);
        }
        if self.sent < previous.sent {
            notification |= self.mask & (PollMask::POLLOUT | PollMask::POLLWRNORM);
        }
        notification
    }
}

/// TCP 端点核心操作守卫，封装 `core` 睡眠锁并在析构时统一完成状态追踪、轮询唤醒与后台 `Work` 交接。
pub struct TcpOperationGuard<'a> {
    /// 所属的 TCP 端点引用。
    endpoint: &'a TcpEndpoint,
    /// 底层 `TcpCore` 睡眠互斥锁守卫；在 `Drop::drop` 中提前 `take()` 释放，以便在锁外执行通知和调度。
    core: Option<SleepMutexGuard<'a, TcpCore>>,
    /// 本次持锁的操作模式（修改或只读观察）。
    mode: OperationMode,
    /// 刚获取 `core` 锁时采集的初始状态快照。
    initial: Observation,
    /// 若开启了内核追踪（`trace`）且处于修改模式，记录操作开始时的 TCP 协议状态。
    initial_trace_state: Option<State>,
}

impl Deref for TcpOperationGuard<'_> {
    type Target = TcpCore;
    /// 不可变借用受保护的 `TcpCore` 实例。
    fn deref(&self) -> &TcpCore {
        self.core.as_deref().unwrap()
    }
}

impl DerefMut for TcpOperationGuard<'_> {
    /// 可变借用受保护的 `TcpCore` 实例。
    fn deref_mut(&mut self) -> &mut TcpCore {
        self.core.as_deref_mut().unwrap()
    }
}

impl Drop for TcpOperationGuard<'_> {
    /// 在持有 `core` 锁的末尾采集最新状态快照并计算新增就绪事件，随后释放 `core` 锁，
    /// 在锁外记录状态迁移追踪、唤醒 `PollWaitHead` 等待者并按需调度后台 `Work`。
    fn drop(&mut self) {
        // Step 1: 在释放 core 锁前更新连接建立标志，并对比快照计算新增的轮询就绪位。
        let notification = if self.mode == OperationMode::Modify {
            let core = self.core.as_deref().unwrap();
            self.endpoint.record_connected(core);
            let current = self.endpoint.observation(core);
            current.notification_since(&self.initial)
        } else {
            PollMask::default()
        };
        let state_change =
            observed_state_change(self.initial_trace_state, self.core.as_deref().unwrap());
        // Step 2: 释放 core 睡眠锁，确保后续唤醒和队列调度不在持锁状态下执行。
        drop(self.core.take());

        // Step 3: 记录 TCP 状态机变迁事件，并检查持锁期间是否有后台 Work 受阻退让。
        if let Some(value) = state_change {
            trace::record(trace::Kind::TcpState, self.endpoint.id, value);
        }
        let owner_waiting = {
            let mut input = self.endpoint.input.lock();
            core::mem::take(&mut input.owner_waiting)
        };
        // Step 4: 唤醒轮询等待者，并在修改操作后或后台 Work 退让时调度后台协议任务。
        if !notification.is_empty() {
            self.endpoint.source.notify(notification);
        }
        if self.mode == OperationMode::Modify || owner_waiting {
            self.endpoint.schedule();
        }
    }
}

impl TcpEndpoint {
    /// 在指定网络栈中创建并初始化一个新的 `TcpEndpoint` 实例。
    ///
    /// 在普通可睡眠上下文中调用。预分配输入报文队列、轮询等待队列头和出口等待节点，
    /// 并为该端点分配固定的协议执行 CPU，将后台 `Work` 和高精度 `Timer` 绑定到同一 CPU 上。
    pub(crate) fn try_new(
        net: &NetStackRef,
        id: u64,
        slot: EndpointSlot,
        family: AddressFamily,
        rx_len: usize,
        tx_len: usize,
    ) -> Result<Arc<Self>, FsError> {
        let core = net.new_core(id, rx_len, tx_len)?;
        let mut packets = VecDeque::new();
        packets
            .try_reserve_exact(LOOPBACK_CAPACITY)
            .map_err(|_| FsError::NoMemory)?;
        let endpoint = Arc::try_new(Self {
            id,
            family,
            stack: Arc::downgrade(net),
            core: SleepMutex::new(core),
            input: SpinLock::new(Input {
                closed: false,
                owner_waiting: false,
                packets,
                reply: None,
            }),
            metadata: SpinLock::new(Metadata {
                lease: None,
                parent: Weak::new(),
                engaged: false,
                error: None,
                connected: false,
                read_closed: false,
                write_closed: false,
                reuse: false,
                v6_only: false,
                timeouts: SocketTimeouts::default(),
            }),
            children: SpinLock::new(Children {
                limit: 0,
                pending: Vec::new(),
                ready: VecDeque::new(),
            }),
            source: Arc::try_new(PollWaitHead::new()).map_err(|_| FsError::NoMemory)?,
            egress_wait: EgressWait::try_new(id)?,
            retirement: RetirementLink::new(),
            work: Once::new(),
            timer: Once::new(),
            stopped: AtomicBool::new(false),
            _slot: slot,
        })
        .map_err(|_| FsError::NoMemory)?;

        // 将端点的后台 Work 与超时 Timer 固定到同一个轮转分配的 CPU，减少跨核竞争。
        let cpu = net.allocate_protocol_cpu();
        let context = Arc::as_ptr(&endpoint) as usize;
        let work = Work::try_new(net.work_queue(), cpu, run_tcp, context)
            .map_err(|_| FsError::NoMemory)?;
        let timer = Timer::try_new(cpu, TimerKind::Precise, tcp_expired, context)
            .map_err(|_| FsError::NoMemory)?;
        endpoint.work.call_once(|| work);
        endpoint.timer.call_once(|| timer);
        Ok(endpoint)
    }

    /// 读取当前端点的收发超时时间配置（`SO_RCVTIMEO` / `SO_SNDTIMEO`）。
    ///
    /// 仅获取 `metadata` 自旋锁，无需等待 `core` 睡眠锁。
    pub(crate) fn timeouts(&self) -> SocketTimeouts {
        self.metadata.lock().timeouts
    }

    /// 设置收发方向的阻塞等待超时时间（单位：纳秒，`None` 表示无限等待）。
    ///
    /// 仅修改当前端点的 `metadata`；被动监听子连接在收到 SYN 创建时会复制当时的超时配置快照。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        self.metadata.lock().timeouts.set(receive, timeout);
    }

    /// 返回端点在所属网络栈中的唯一标识号。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 返回端点的发送通道（Egress）容量等待链表节点引用。
    pub(crate) fn egress_wait(&self) -> &Arc<EgressWait> {
        &self.egress_wait
    }

    /// 供内核测试（`ktest`）读取当前端点持有的端口绑定租约 ID。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_binding_identity(&self) -> Option<u64> {
        self.metadata.lock().lease.as_ref().map(|lease| lease.id())
    }

    /// 供内核测试（`ktest`）查询后台 `Work` 与高精度 `Timer` 共同绑定的目标 CPU 编号。
    #[cfg(feature = "ktest")]
    pub fn work_cpu(&self) -> usize {
        self.work.get().unwrap().cpu()
    }

    /// 检查当前端点是否隶属于指定的网络栈实例 `net`。
    pub fn belongs_to(&self, net: &NetStackRef) -> bool {
        self.stack.as_ptr() == Arc::as_ptr(net)
    }

    /// 以修改模式（`OperationMode::Modify`）获取 `core` 睡眠锁，返回操作守卫 `TcpOperationGuard`。
    ///
    /// 守卫析构时会自动比对操作前后的状态差异、唤醒就绪等待者并调度后台 `Work`。
    pub fn lock(&self) -> TcpOperationGuard<'_> {
        self.operation(self.core.lock(), OperationMode::Modify)
    }

    /// 查询当前 TCP 状态机的状态（如 `Closed`、`Listen`、`Established` 等）。
    pub fn state(&self) -> State {
        self.observe().state()
    }

    /// 以只读观察模式（`OperationMode::Observe`）获取 `core` 睡眠锁，返回操作守卫 `TcpOperationGuard`。
    ///
    /// 守卫析构时不会主动发起新的协议处理，仅在持锁期间有后台 `Work` 受阻退让时负责接力调度。
    pub(crate) fn observe(&self) -> TcpOperationGuard<'_> {
        self.operation(self.core.lock(), OperationMode::Observe)
    }

    /// 构造 `TcpOperationGuard` 操作守卫，并在持锁初期采集状态快照与追踪起点。
    fn operation<'a>(
        &'a self,
        core: SleepMutexGuard<'a, TcpCore>,
        mode: OperationMode,
    ) -> TcpOperationGuard<'a> {
        let initial = self.observation(&core);
        let initial_trace_state = if mode == OperationMode::Modify && trace::enabled() {
            Some(core.state())
        } else {
            None
        };
        TcpOperationGuard {
            endpoint: self,
            core: Some(core),
            mode,
            initial,
            initial_trace_state,
        }
    }

    /// 根据 `TcpCore` 状态机、本地关闭标志及监听子连接队列，计算当前的 `PollMask` 事件就绪掩码。
    ///
    /// 调用方必须已持有 `core` 睡眠锁。
    fn ready_mask(&self, core: &TcpCore) -> PollMask {
        if core.state() == State::Listen {
            return if self.children.lock().ready.is_empty() {
                PollMask::default()
            } else {
                PollMask::POLLIN | PollMask::POLLRDNORM
            };
        }

        let metadata = self.metadata.lock();
        let mut mask = PollMask::default();
        if metadata.error.is_some() {
            mask |= PollMask::POLLERR;
        }
        let closed = core.state() == State::Closed;
        if closed || (metadata.read_closed && metadata.write_closed) {
            mask |= PollMask::POLLHUP;
        }
        if core.receive_finished() || metadata.read_closed || (closed && metadata.connected) {
            mask |= PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLRDHUP;
        }
        if core.can_receive() {
            mask |= PollMask::POLLIN | PollMask::POLLRDNORM;
        }
        if core.can_send() || metadata.write_closed || closed {
            mask |= PollMask::POLLOUT | PollMask::POLLWRNORM;
        }
        mask
    }

    /// 采集当前 `TcpCore` 收发队列长度、轮询就绪掩码及全连接子队列长度的观察快照。
    fn observation(&self, core: &TcpCore) -> Observation {
        Observation {
            received: core.receive_queue(),
            sent: core.send_queue(),
            mask: self.ready_mask(core),
            ready_children: self.children.lock().ready.len(),
        }
    }

    /// 若底层状态机已进入 `Established` 或 `CloseWait`，将 `metadata.connected` 标记为 `true`。
    ///
    /// 该标志一旦置位，即使后续连接正常关闭回到 `Closed` 也会保留，用于区分“从未连接”与“连接后已关闭（EOF/EPIPE）”。
    fn record_connected(&self, core: &TcpCore) {
        if matches!(core.state(), State::Established | State::CloseWait) {
            self.metadata.lock().connected = true;
        }
    }

    /// 注册可选的轮询等待表，并查询当前端点的 `PollMask` 就绪掩码。
    pub fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&self.source)?;
        }
        let core = self.observe();
        Ok(self.ready_mask(&core))
    }

    /// 查询非阻塞或阻塞 `connect` 的当前握手结果：
    /// - 若存在异步错误，返回 `Err(error)`（不清除 `SO_ERROR`）；
    /// - 若仍在 `SynSent` 或 `SynReceived` 握手中，返回 `Ok(None)`；
    /// - 若已完成握手进入 `Established` 或 `CloseWait`，返回 `Ok(Some(()))`；
    /// - 其他未连接状态返回 `Err(FsError::NotConnected)`。
    pub fn connection_result(&self) -> Result<Option<()>, FsError> {
        let core = self.observe();
        let metadata = self.metadata.lock();
        if let Some(error) = metadata.error {
            return Err(error);
        }
        if matches!(core.state(), State::SynSent | State::SynReceived) {
            return Ok(None);
        }
        if metadata.connected && matches!(core.state(), State::Established | State::CloseWait) {
            return Ok(Some(()));
        }
        Err(FsError::NotConnected)
    }

    /// 从 TCP 接收缓冲区读取数据到 `target`（支持 `MSG_PEEK` 预览模式）。
    ///
    /// 在可睡眠任务上下文中调用。若接收缓冲区中已有可读数据，优先交付数据；
    /// 当缓冲区为空时，依次检查并消费挂起的异步错误（`SO_ERROR`）或返回读关闭 EOF。
    pub fn receive(&self, target: &mut ReadBuffer<'_>, peek: bool) -> Result<IoProgress, FsError> {
        if target.copied() == target.len() {
            return Ok(IoProgress::Ready(0));
        }
        let mut core = self.lock();
        if core.can_receive() {
            return core.receive(target, peek);
        }

        let mut metadata = self.metadata.lock();
        if let Some(error) = metadata.error.take() {
            return Err(error);
        }
        if metadata.read_closed || (metadata.connected && core.state() == State::Closed) {
            return Ok(IoProgress::Eof);
        }
        drop(metadata);
        core.receive(target, peek)
    }

    /// 将 `source` 中的数据写入 TCP 发送缓冲区。
    ///
    /// 在可睡眠任务上下文中调用。写操作前先检查并消费异步错误；若写方向已关闭或连接已断开，
    /// 即使请求写入长度为 0 也会返回 `Err(FsError::BrokenPipe)`（触发上层 `EPIPE` / `SIGPIPE`）。
    pub fn send(&self, source: &mut WriteBuffer<'_>) -> Result<IoProgress, FsError> {
        let mut core = self.lock();
        let mut metadata = self.metadata.lock();
        if let Some(error) = metadata.error.take() {
            return Err(error);
        }
        if metadata.write_closed || (metadata.connected && core.state() == State::Closed) {
            return Err(FsError::BrokenPipe);
        }
        drop(metadata);
        core.send(source)
    }

    /// 关闭连接的指定传输方向（`SHUT_RD`、`SHUT_WR` 或 `SHUT_RDWR`）。
    ///
    /// 关闭写方向会向 `TcpCore` 发起 FIN 挥手流程，此时读方向仍可继续接收对端发来的数据。
    pub fn shutdown(&self, how: u32) -> Result<(), FsError> {
        if SHUT_RDWR < how {
            return Err(FsError::InvalidOperation);
        }

        let mut core = self.lock();
        let mut metadata = self.metadata.lock();
        if !metadata.connected || core.state() == State::Closed {
            return Err(FsError::NotConnected);
        }
        if how == SHUT_RD || how == SHUT_RDWR {
            metadata.read_closed = true;
        }
        if how == SHUT_WR || how == SHUT_RDWR {
            metadata.write_closed = true;
            core.close();
        }
        Ok(())
    }

    /// 获取当前端点的本地绑定地址与端口（`getsockname`）。
    ///
    /// 若已建立连接则返回路由选定的具体源地址；若仅完成 `bind` 则返回租约记录的地址；未绑定时返回通配零地址和 0 端口。
    pub fn local_address(&self) -> Result<InetEndpoint, FsError> {
        let core = self.observe();
        Ok(core
            .local_endpoint()
            .or_else(|| {
                self.metadata
                    .lock()
                    .lease
                    .as_ref()
                    .map(|lease| lease.local())
            })
            .unwrap_or(InetEndpoint {
                address: self.family.unspecified(),
                port: 0,
            }))
    }

    /// 获取当前已连接对端的远端地址与端口（`getpeername`）。
    pub fn peer_address(&self) -> Result<InetEndpoint, FsError> {
        let core = self.observe();
        if !self.metadata.lock().connected {
            return Err(FsError::NotConnected);
        }
        core.remote_endpoint().ok_or(FsError::NotConnected)
    }

    /// 读取并清除挂起的异步套接字错误（对应 `getsockopt(SO_ERROR)`）。
    pub fn take_error(&self) -> Option<FsError> {
        let core = self.observe();
        let error = self.metadata.lock().error.take();
        drop(core);
        error
    }

    /// 判断当前端点是否处于 `Listen` 监听状态（对应 `SO_ACCEPTCONN`）。
    pub fn is_listener(&self) -> bool {
        self.state() == State::Listen
    }

    /// 查询当前端点是否开启了 `SO_REUSEADDR` 地址复用选项。
    pub fn reuse_address(&self) -> bool {
        self.metadata.lock().reuse
    }

    /// 设置 `SO_REUSEADDR` 地址复用选项，并在当前端点直接拥有端口租约时同步更新全局绑定表策略。
    pub fn set_reuse_address(&self, reuse: bool) -> Result<(), FsError> {
        let _core = self.observe();
        let net = self.net()?;
        let _admission = net.admission()?;
        let mut metadata = self.metadata.lock();
        if let Some(lease) = metadata.lease.as_ref() {
            if lease.owner_id() == self.id {
                net.tables().set_reuse(lease.id(), reuse)?;
            }
        }
        metadata.reuse = reuse;
        Ok(())
    }

    /// 查询当前端点是否开启了 `TCP_NODELAY`（即禁用 Nagle 算法）。
    pub fn nodelay(&self) -> Result<bool, FsError> {
        Ok(!self.observe().nagle_enabled())
    }

    /// 设置 `TCP_NODELAY` 选项（启用时关闭底层 `TcpCore` 的 Nagle 合并算法）。
    pub fn set_nodelay(&self, enabled: bool) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_nagle_enabled(!enabled);
        Ok(())
    }

    /// 返回底层接收环形缓冲区的物理总容量（字节）。
    pub fn receive_capacity(&self) -> usize {
        self.observe().receive_capacity()
    }

    /// 返回底层发送环形缓冲区的物理总容量（字节）。
    pub fn send_capacity(&self) -> usize {
        self.observe().send_capacity()
    }

    /// 返回端点创建时固定的地址族（`AF_INET` 或 `AF_INET6`）。
    pub fn family(&self) -> AddressFamily {
        self.family
    }

    /// 查询 IPv6 端点是否开启了 `IPV6_V6ONLY` 选项。
    pub fn v6_only(&self) -> bool {
        let _core = self.observe();
        self.metadata.lock().v6_only
    }

    /// 设置 IPv6 端点的 `IPV6_V6ONLY` 选项（仅允许在未绑定且未进入活动状态的 `AF_INET6` 端点上修改）。
    pub fn set_v6_only(&self, enabled: bool) -> Result<(), FsError> {
        let _core = self.lock();
        if self.family != AddressFamily::Ipv6 {
            return Err(FsError::Unsupported);
        }
        let net = self.net()?;
        let _admission = net.admission()?;
        let mut metadata = self.metadata.lock();
        if metadata.lease.is_some() || metadata.engaged {
            return Err(FsError::InvalidOperation);
        }
        metadata.v6_only = enabled;
        Ok(())
    }

    /// 查询当前端点是否开启了 `SO_KEEPALIVE` 保活探测。
    pub fn keepalive(&self) -> bool {
        self.observe().keepalive()
    }

    /// 设置 `SO_KEEPALIVE` 保活探测开关。
    pub fn set_keepalive(&self, enabled: bool) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_keepalive(enabled);
        Ok(())
    }

    /// 返回当前接收缓冲区的通告窗口接纳预算（字节，对应 `SO_RCVBUF`）。
    pub fn receive_budget(&self) -> usize {
        self.observe().receive_budget()
    }

    /// 返回当前发送缓冲区的写入接纳预算（字节，对应 `SO_SNDBUF`）。
    pub fn send_budget(&self) -> usize {
        self.observe().send_budget()
    }

    /// 调整接收缓冲区的接纳预算（`SO_RCVBUF`）。
    pub fn set_receive_budget(&self, budget: usize) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_receive_budget(budget)
    }

    /// 调整发送缓冲区的接纳预算（`SO_SNDBUF`）；若扩容后恢复可写，守卫释放时会自动唤醒 `POLLOUT` 等待者。
    pub fn set_send_budget(&self, budget: usize) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_send_budget(budget)
    }

    /// 查询当前连接生效的最大报文段长度（`TCP_MAXSEG` / MSS，单位：字节）。
    pub fn effective_mss(&self) -> usize {
        self.observe().effective_mss(self.family)
    }

    /// 查询当前连接启用的拥塞控制算法（如 Reno）。
    pub fn congestion_control(&self) -> CongestionControl {
        self.observe().congestion_control()
    }

    /// 校验目标或本地 IP 地址是否符合当前端点的地址族与 `IPV6_V6ONLY` 策略。
    ///
    /// 当 `IPV6_V6ONLY` 开启的 IPv6 端点尝试使用 IPv4 地址时，连接操作返回 `NetworkUnreachable`，绑定操作返回 `InvalidOperation`。
    fn check_address(&self, address: IpAddress, connecting: bool) -> Result<(), FsError> {
        match (self.family, address) {
            (AddressFamily::Ipv4, IpAddress::Ipv6(_)) => Err(FsError::AddressFamilyNotSupported),
            (AddressFamily::Ipv6, IpAddress::Ipv4(_)) if self.metadata.lock().v6_only => {
                Err(if connecting {
                    FsError::NetworkUnreachable
                } else {
                    FsError::InvalidOperation
                })
            }
            _ => Ok(()),
        }
    }

    /// 将弱引用升级为所属网络栈的强引用；若网络栈已销毁则返回 `Err(FsError::NotConnected)`。
    fn net(&self) -> Result<NetStackRef, FsError> {
        self.stack.upgrade().ok_or(FsError::NotConnected)
    }

    /// 将端点显式绑定到指定的本地地址与端口（`bind` 系统调用）。
    ///
    /// 在持有 `core` 睡眠锁与网络栈 `admission` 准入锁的情况下向全局表申请端口租约；
    /// 若成功绑定到具体的非通配 IPv6 地址，则同时将 `v6_only` 固定为 `true`。
    pub fn bind(self: &Arc<Self>, local: InetEndpoint) -> Result<InetEndpoint, FsError> {
        let core = self.lock();
        if core.state() != State::Closed || self.metadata.lock().engaged {
            return Err(FsError::InvalidOperation);
        }
        self.check_address(local.address, false)?;
        let net = self.net()?;
        let _admission = net.admission()?;
        if self.metadata.lock().lease.is_some() {
            return Err(FsError::InvalidOperation);
        }

        let dual_stack =
            local.address == AddressFamily::Ipv6.unspecified() && !self.metadata.lock().v6_only;
        let lease = net.tables().reserve_binding(
            &net,
            self.id,
            TransportProtocol::Tcp,
            local,
            self.reuse_address(),
            dual_stack,
        )?;
        let local = lease.local();

        let mut metadata = self.metadata.lock();
        if matches!(local.address, IpAddress::Ipv6(_)) && !local.is_unspecified() {
            metadata.v6_only = true;
        }
        metadata.lease = Some(lease);
        Ok(local)
    }

    /// 将端点转换为 `Listen` 监听状态，或更新已处于监听状态的端点的全/半连接队列容量上限（`backlog`）。
    ///
    /// 在获取 `children` 自旋锁前先完成新队列容量的堆内存预分配；首次进入监听时自动完成未绑定端点的临时端口分配并在全局表中发布监听索引。
    pub fn listen(self: &Arc<Self>, backlog: usize) -> Result<(), FsError> {
        let limit = backlog.clamp(1, SOMAXCONN);
        let mut core = self.lock();
        let listening = core.state() == State::Listen;
        if !listening && (core.state() != State::Closed || self.metadata.lock().engaged) {
            return Err(FsError::InvalidOperation);
        }

        // Step 1: 计算需要预分配的队列容量，并在自旋锁外完成堆内存预留。
        let capacity = {
            let children = self.children.lock();
            if limit <= children.pending.capacity() && limit <= children.ready.capacity() {
                0
            } else {
                limit.max(children.pending.len() + children.ready.len())
            }
        };
        let mut pending = Vec::new();
        let mut ready = VecDeque::new();
        if 0 < capacity {
            pending
                .try_reserve_exact(capacity)
                .map_err(|_| FsError::NoMemory)?;
            ready
                .try_reserve_exact(capacity)
                .map_err(|_| FsError::NoMemory)?;
        }

        // Step 2: 若尚未处于 Listen 状态，准备底层缓冲区、获取绑定租约并在全局表中发布监听端点。
        let net = self.net()?;
        let _admission = net.admission()?;
        if !listening {
            core.prepare_buffers()?;
            let lease = self.binding_or_ephemeral(&net)?;
            core.listen(lease.local())?;
            if let Err(error) = net.tables().publish_listener(self.id, lease.local()) {
                core.close();
                return Err(error);
            }
        }

        // Step 3: 将旧队列中的子连接迁移到扩容后的容器中，并更新 backlog 上限与 engaged 标志。
        {
            let mut children = self.children.lock();
            if 0 < capacity {
                pending.append(&mut children.pending);
                ready.extend(children.ready.drain(..));
                core::mem::swap(&mut pending, &mut children.pending);
                core::mem::swap(&mut ready, &mut children.ready);
            }
            children.limit = limit;
        }
        self.metadata.lock().engaged = true;
        Ok(())
    }

    /// 发起主动 TCP 连接握手（`connect` 系统调用）。
    ///
    /// 依次完成连接状态检查、对端地址族校验、底层收发缓冲区分配、本地端口租约获取、
    /// 路由源地址与路径 MTU 解析，随后启动 `TcpCore` 握手并在全局表中注册 TCP 四元组。
    pub fn connect(self: &Arc<Self>, peer: InetEndpoint) -> Result<(), FsError> {
        // Step 1: 检查当前连接状态，排除已连接、正在握手或已关闭过的端点。
        let mut core = self.lock();
        let metadata = self.metadata.lock();
        if metadata.connected {
            return Err(FsError::AlreadyConnected);
        }
        if matches!(core.state(), State::SynSent | State::SynReceived) {
            return Err(FsError::AlreadyInProgress);
        }
        if core.state() != State::Closed || metadata.engaged {
            return Err(metadata.error.unwrap_or(FsError::NotConnected));
        }
        drop(metadata);

        // Step 2: 校验对端地址，准备收发环形缓冲区、本地端口租约并解析出口路由与 MTU。
        self.check_address(peer.address, true)?;
        let net = self.net()?;
        let admission = net.admission()?;
        core.prepare_buffers()?;
        let lease = self.binding_or_ephemeral(&net)?;
        let route = net.resolve_route(peer.address, lease.local().address)?;
        let local = InetEndpoint {
            address: route.source,
            port: lease.local().port,
        };
        core.set_ip_mtu(route.mtu);

        // Step 3: 启动底层 TCP 状态机三次握手（进入 SynSent），并在全局连接表中注册四元组。
        core.connect(local, peer, time::now())?;
        if let Err(error) = net
            .tables()
            .publish_tcp_tuple(self.id, TcpTuple { local, peer })
        {
            core.abort();
            self.metadata.lock().error = Some(error);
            self.metadata.lock().engaged = true;
            drop(admission);
            drop(core);
            net.retire_tcp(self.id);
            return Err(error);
        }
        self.metadata.lock().engaged = true;
        drop(admission);
        drop(core);
        Ok(())
    }

    /// 获取当前已有的绑定租约；若尚未绑定，则向全局表申请本地址族通配地址上的临时端口租约。
    ///
    /// 调用方必须已持有当前端点的 `core` 睡眠锁及网络栈 `admission` 准入锁。
    fn binding_or_ephemeral(&self, net: &NetStackRef) -> Result<Arc<BindLease>, FsError> {
        if let Some(lease) = self.metadata.lock().lease.as_ref().cloned() {
            return Ok(lease);
        }
        let dual_stack = self.family == AddressFamily::Ipv6 && !self.metadata.lock().v6_only;
        let lease = net.tables().reserve_binding(
            net,
            self.id,
            TransportProtocol::Tcp,
            InetEndpoint {
                address: self.family.unspecified(),
                port: 0,
            },
            self.reuse_address(),
            dual_stack,
        )?;
        self.metadata.lock().lease = Some(lease.clone());
        Ok(lease)
    }

    /// 从监听端点的全连接队列（`ready`）队首取出一个已完成三次握手的子连接端点。
    pub fn accept(&self) -> Result<Option<Arc<TcpEndpoint>>, FsError> {
        let core = self.lock();
        if core.state() != State::Listen {
            return Err(FsError::InvalidOperation);
        }
        let child = self.children.lock().ready.pop_front();
        Ok(child)
    }

    /// 当最后一个关联的套接字文件描述关闭时调用，关闭读写双方向并触发底层 TCP 优雅关闭挥手（FIN / `TIME_WAIT`）。
    pub fn close(&self) {
        let mut core = self.lock();
        core.close();
        let mut metadata = self.metadata.lock();
        metadata.engaged = true;
        metadata.read_closed = true;
        metadata.write_closed = true;
    }

    /// 强制中止尚未被用户态接受的被动子连接，向对端触发 RST 并丢弃协议连接。
    pub fn abandon(&self) {
        let mut core = self.lock();
        core.abort();
        self.metadata.lock().engaged = true;
    }

    /// 将收到的 IP/TCP 报文推入端点的有界输入队列，并调度后台 `Work` 异步消费。
    ///
    /// 仅获取 `input` 自旋锁，无需等待用户态释放 `core` 睡眠锁；若队列已满则返回 `Err(FsError::WouldBlock)`。
    pub(crate) fn enqueue(&self, packet: OwnedPacket) -> Result<(), FsError> {
        {
            let mut input = self.input.lock();
            if input.closed {
                return Err(FsError::NotConnected);
            }
            if input.packets.len() == LOOPBACK_CAPACITY {
                return Err(FsError::WouldBlock);
            }
            input.packets.push_back(packet);
        }

        self.schedule();
        Ok(())
    }

    /// 将端点的后台协议处理任务排入工作队列。
    ///
    /// 若端点已同步停止或网络栈正在停止，则直接忽略调度请求；重复入队由 `Work::queue` 内部自动去重合并。
    pub(crate) fn schedule(&self) {
        if self.stopped.load(Ordering::Acquire) {
            return;
        }
        let Some(net) = self.stack.upgrade() else {
            return;
        };
        if net.is_stopping() {
            return;
        }
        let _ = self.work.get().unwrap().as_ref().queue();
    }

    /// 后台 `Work` 的核心执行入口：尝试获取 `core` 睡眠锁，批量处理输入报文与待发数据，更新定时器并唤醒等待队列。
    fn run(&self) {
        let Some(net) = self.stack.upgrade() else {
            return;
        };
        if net.is_stopping() || self.stopped.load(Ordering::Acquire) {
            return;
        }

        // Step 1: 登记 owner_waiting 标志并尝试非阻塞获取 core 睡眠锁；
        // 若用户态正持有 core 锁，则立即返回，由 TcpOperationGuard::drop 负责在解锁后重新调度。
        self.input.lock().owner_waiting = true;
        let Some(mut core) = self.core.try_lock() else {
            return;
        };
        self.input.lock().owner_waiting = false;
        let previous = self.observation(&core);
        let initial_trace_state = trace::enabled().then(|| core.state());

        // Step 2: 推进一轮输入报文处理与输出报文分发；若发生非阻塞类致命错误则中止连接并记录错误。
        let mut notification = None;
        let result = self.advance(&net, &mut core, &mut notification);
        let blocked = result == Err(FsError::WouldBlock);
        if let Err(error) = result {
            if !blocked {
                self.metadata.lock().error.get_or_insert(error);
                core.abort();
                crate::klog!(
                    crate::klog::Level::Debug,
                    "TCP endpoint {} failed: {:?}",
                    self.id,
                    error
                );
            }
        }

        // Step 3: 采集处理后的状态快照、下一步超时时间及队列剩余情况，随后释放 core 睡眠锁。
        self.record_connected(&core);
        let current = self.observation(&core);
        let mask = current.notification_since(&previous);
        let deadline = core.deadline(time::now());
        let (queued, reply_pending) = {
            let input = self.input.lock();
            (!input.packets.is_empty(), input.reply.is_some())
        };
        let ended = core.state() == State::Closed
            && deadline == ProtocolDeadline::Ingress
            && self.metadata.lock().engaged
            && !reply_pending;
        let state_change = observed_state_change(initial_trace_state, &core);
        drop(core);

        // Step 4: 在锁外记录追踪事件，唤醒当前端点及父监听端点的轮询等待者。
        if let Some(value) = state_change {
            trace::record(trace::Kind::TcpState, self.id, value);
        }
        if !mask.is_empty() {
            self.source.notify(mask);
        }
        if let Some(parent) = notification {
            parent
                .source
                .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        }

        // Step 5: 若连接已彻底结束则移交异步退休；若因出口通道满受阻则等待容量恢复；否则按协议期限设置定时器或继续调度。
        if ended {
            net.retire_tcp(self.id);
            return;
        }
        if blocked {
            self.timer.get().unwrap().as_ref().cancel_pending();
            if net.egress_ready(self.egress_wait()) == Ok(true) {
                net.notify_egress(self.id);
            }
            return;
        }

        if net.is_stopping() {
            return;
        }
        match deadline {
            ProtocolDeadline::At(at) => {
                let _ = self.timer.get().unwrap().as_ref().arm(at);
            }
            ProtocolDeadline::Now => {
                self.timer.get().unwrap().as_ref().cancel_pending();
                self.schedule();
            }
            ProtocolDeadline::Ingress => {
                self.timer.get().unwrap().as_ref().cancel_pending();
            }
        }
        if queued {
            self.schedule();
        }
    }

    /// 在单轮 `PROTOCOL_BUDGET` 配额内，依次发送挂起的立即应答、处理输入报文队列并分发待发送的 TCP 报文段。
    fn advance(
        &self,
        net: &NetStackRef,
        core: &mut TcpCore,
        notification: &mut Option<Arc<TcpEndpoint>>,
    ) -> Result<(), FsError> {
        // Step 1: 优先发出上一轮因发送通道满而暂存的立即应答包（如 SYN-ACK / ACK / RST）。
        self.emit_reply(net)?;
        // Step 2: 在 PROTOCOL_BUDGET 配额内消费输入队列中的报文。
        let mut remaining = PROTOCOL_BUDGET;
        while 0 < remaining {
            let packet = self.input.lock().packets.pop_front();
            let Some(packet) = packet else {
                break;
            };
            remaining -= 1;
            if core.state() == State::Listen && self.children.lock().limit != 0 {
                if let Err(error) = self.receive_syn(net, core, packet) {
                    crate::klog!(
                        crate::klog::Level::Debug,
                        "TCP listener {} child preparation failed: {:?}",
                        self.id,
                        error
                    );
                }
            } else if core.state() != State::Closed {
                let previous = core.state();
                let reset = packet.tcp()?.1.control == TcpControl::Rst;
                let reply = core.process(&packet, time::now())?;
                self.record_connected(core);
                if reset && core.state() == State::Closed {
                    let mut metadata = self.metadata.lock();
                    let error = if previous == State::SynSent {
                        Some(FsError::ConnectionRefused)
                    } else if metadata.connected {
                        Some(FsError::ConnectionReset)
                    } else {
                        None
                    };
                    if let Some(error) = error {
                        metadata.error.get_or_insert(error);
                    }
                }
                if previous == State::SynReceived && core.state() == State::Listen {
                    // 若被动子连接在 SynReceived 阶段握手失败（如收到 RST）回退到了 Listen 状态，
                    // 立即将其关闭为 Closed，避免子连接以 Listen 状态滞留在半连接队列中。
                    if self.metadata.lock().parent.upgrade().is_some() {
                        core.close();
                    }
                }
                if let Some(parent) = self.report_established(core.state()) {
                    *notification = Some(parent);
                }
                if let Some(reply) = reply {
                    self.input.lock().reply = Some(reply);
                    self.emit_reply(net)?;
                }
            }
        }

        // Step 3: 使用剩余配额分发协议栈中待发送的数据段、重传段或 FIN 控制段。
        while 0 < remaining {
            remaining -= 1;
            core.dispatch(time::now(), |packet| net.send_ip(packet))?;
            if core.deadline(time::now()) != ProtocolDeadline::Now {
                break;
            }
        }
        Ok(())
    }

    /// 尝试向网络栈发送 `input.reply` 中暂存的立即控制应答报文；若发送通道暂满则将报文放回 `input.reply` 留待下次重试。
    fn emit_reply(&self, net: &NetStackRef) -> Result<(), FsError> {
        let reply = self.input.lock().reply.take();
        let Some(reply) = reply else {
            return Ok(());
        };
        match net.try_emit(reply) {
            Ok(()) => Ok(()),
            Err((error, reply)) => {
                self.input.lock().reply = Some(reply);
                Err(error)
            }
        }
    }

    /// 处理监听端点收到的 SYN 握手请求：创建并初始化被动子连接端点，将其注册到全局连接表并加入半连接队列。
    fn receive_syn(
        &self,
        net: &NetStackRef,
        core: &TcpCore,
        packet: OwnedPacket,
    ) -> Result<(), FsError> {
        // Step 1: 解析 TCP 头部，忽略非 SYN 报文，校验本地地址策略并排查已有四元组连接。
        let (_, repr) = packet.tcp()?;
        if repr.control != TcpControl::Syn {
            return Ok(());
        }
        let (peer, local) = packet.tcp_endpoints()?;
        self.check_address(local.address, false)?;
        let tuple = TcpTuple { local, peer };
        if let Some(child) = net.tables().lookup_tcp(tuple) {
            return child.enqueue(packet);
        }
        // Step 2: 检查监听端点的半连接与全连接总队列长度是否已达 backlog 上限。
        {
            let children = self.children.lock();
            if children.limit <= children.pending.len() + children.ready.len() {
                return Ok(());
            }
        }

        // Step 3: 快照父监听端点的绑定租约与选项，分配新端点槽位并初始化子连接的 TcpCore 与 Metadata。
        let (lease, reuse, v6_only, timeouts) = {
            let metadata = self.metadata.lock();
            (
                metadata
                    .lease
                    .as_ref()
                    .cloned()
                    .expect("listener has no binding"),
                metadata.reuse,
                metadata.v6_only,
                metadata.timeouts,
            )
        };
        let slot = net.claim_endpoint_slot()?;
        let id = net.allocate_identity();
        let child = Self::try_new(
            net,
            id,
            slot,
            self.family,
            core.receive_capacity(),
            core.send_capacity(),
        )?;
        let route = net.resolve_route(peer.address, local.address)?;
        {
            let mut child_core = child.core.lock();
            child_core.set_ip_mtu(route.mtu);
            child_core.set_receive_budget(core.receive_budget())?;
            child_core.set_send_budget(core.send_budget())?;
            child_core.set_nagle_enabled(core.nagle_enabled());
            child_core.set_keepalive(core.keepalive());
            child_core.listen(local)?;
        }

        let listener = net
            .tables()
            .tcp_by_id(self.id)
            .ok_or(FsError::NotConnected)?;
        {
            let mut metadata = child.metadata.lock();
            metadata.lease = Some(lease);
            metadata.parent = Arc::downgrade(&listener);
            metadata.engaged = true;
            metadata.reuse = reuse;
            metadata.v6_only = v6_only;
            metadata.timeouts = timeouts;
        }

        // Step 4: 获取网络栈准入锁，二次确认四元组未被并发创建后，在全局表中发布子连接并推入半连接队列，最后将 SYN 包入队给子连接处理。
        let admission = net.admission()?;
        if let Some(existing) = net.tables().lookup_tcp(tuple) {
            drop(admission);
            return existing.enqueue(packet);
        }
        net.tables().insert_tcp(child.clone())?;
        if let Err(error) = net.tables().publish_tcp_tuple(id, tuple) {
            let unpublished = net.tables().take_tcp(id);
            drop(admission);
            drop(unpublished);
            return Err(error);
        }
        self.children.lock().pending.push(child.clone());
        drop(admission);
        child.enqueue(packet)
    }

    /// 当被动子连接完成三次握手（进入 `Established` 或 `CloseWait`）时，将其从父监听端点的 `pending` 半连接队列移入 `ready` 全连接队列，
    /// 并返回父监听端点强引用以便在释放 `core` 锁后唤醒 `accept` 等待者。
    fn report_established(&self, state: State) -> Option<Arc<TcpEndpoint>> {
        if !matches!(state, State::Established | State::CloseWait) {
            return None;
        }
        let parent = self.metadata.lock().parent.upgrade()?;
        let ready = {
            let mut children = parent.children.lock();
            if let Some(index) = children
                .pending
                .iter()
                .position(|child| child.id == self.id)
            {
                let child = children.pending.swap_remove(index);
                children.ready.push_back(child);
                true
            } else {
                false
            }
        };
        if ready {
            Some(parent)
        } else {
            None
        }
    }

    /// 同步停止端点并清理所有后台活动与挂起资源。
    ///
    /// 由网络栈异步退休任务或网络栈整体销毁流程在持有端点局部 `Arc` 强引用的前提下调用。
    /// 先关闭高精度 `Timer` 并等待后台 `Work` 彻底执行完毕，再中止底层 `TcpCore`、清空报文队列与子连接队列。
    pub(crate) fn stop_sync(&self) {
        assert!(
            !self.stopped.swap(true, Ordering::AcqRel),
            "TCP endpoint stopped twice"
        );
        // Step 1: 封闭输入队列，并同步停止 Timer 与后台 Work，等待正在执行的回调彻底退出。
        self.input.lock().closed = true;
        self.timer.get().unwrap().as_ref().close_sync();
        self.work.get().unwrap().as_ref().close_sync();

        // Step 2: 获取 core 睡眠锁，将未关闭的连接标记为 NotConnected 并强制中止协议状态机。
        {
            let mut core = self.core.lock();
            if core.state() != State::Closed {
                self.metadata
                    .lock()
                    .error
                    .get_or_insert(FsError::NotConnected);
            }
            core.abort();
        }

        // Step 3: 取出未处理的输入报文、挂起应答、父端点引用及所有半连接/全连接子端点。
        let (packets, reply) = {
            let mut input = self.input.lock();
            (core::mem::take(&mut input.packets), input.reply.take())
        };
        // 端口绑定租约保留在 metadata.lease 中供存活的套接字文件描述查询 getsockname，待 TcpEndpoint 最终析构时自动归还。
        let parent = {
            let mut metadata = self.metadata.lock();
            core::mem::take(&mut metadata.parent)
        };
        let children = {
            let mut children = self.children.lock();
            children.limit = 0;
            (
                core::mem::take(&mut children.pending),
                core::mem::take(&mut children.ready),
            )
        };

        // Step 4: 在自旋锁外中止所有子连接、从父监听队列中移除自身、释放报文并唤醒所有轮询等待者。
        for child in children.0.into_iter().chain(children.1) {
            child.abandon();
        }
        if let Some(parent) = parent.upgrade() {
            parent.remove_child(self.id);
        }
        drop((packets, reply));
        self.source
            .notify(PollMask::POLLIN | PollMask::POLLOUT | PollMask::POLLERR | PollMask::POLLHUP);
    }

    /// 从监听端点的半连接或全连接队列中移除指定 ID 的子连接，并在释放 `children` 自旋锁后再析构其 `Arc` 引用。
    fn remove_child(&self, id: u64) {
        let removed = {
            let mut children = self.children.lock();
            if let Some(index) = children.pending.iter().position(|child| child.id == id) {
                Some(children.pending.swap_remove(index))
            } else if let Some(index) = children.ready.iter().position(|child| child.id == id) {
                children.ready.remove(index)
            } else {
                None
            }
        };

        drop(removed);
    }
}

/// 若开启了内核追踪且操作前后 TCP 状态发生变化，将旧状态与新状态打包为 16 位整数（高 8 位旧状态、低 8 位新状态）返回。
fn observed_state_change(previous: Option<State>, core: &TcpCore) -> Option<u64> {
    let previous = previous?;
    if !trace::enabled() {
        return None;
    }
    let current = core.state();
    if previous == current {
        return None;
    }
    Some((trace_state_code(previous) << u8::BITS) | trace_state_code(current))
}

/// 将 `smoltcp` 的 TCP `State` 枚举映射为内核追踪事件使用的固定数值编码。
fn trace_state_code(state: State) -> u64 {
    match state {
        State::Closed => 0,
        State::Listen => 1,
        State::SynSent => 2,
        State::SynReceived => 3,
        State::Established => 4,
        State::FinWait1 => 5,
        State::FinWait2 => 6,
        State::CloseWait => 7,
        State::Closing => 8,
        State::LastAck => 9,
        State::TimeWait => 10,
    }
}

/// 后台工作队列回调入口：还原 `TcpEndpoint` 引用并推进协议收发。
fn run_tcp(work: Pin<&Work>) {
    // SAFETY:
    // 1. `work.context()` 在 `TcpEndpoint::try_new` 中由 `Arc::as_ptr(&endpoint) as usize` 初始化，指向堆上地址稳定的 `TcpEndpoint`；
    // 2. 端点发布后由全局连接表、异步退休队列或调用 `stop_sync` 的局部 `Arc` 强引用保活；
    // 3. 在最后一个 `Arc` 强引用释放前，`stop_sync` 必定先调用 `work.close_sync()` 等待本回调彻底执行完毕并退出，因此解引用在回调生命周期内始终有效。
    let endpoint = unsafe { &*(work.context() as *const TcpEndpoint) };
    endpoint.run();
}

/// 高精度协议定时器到期回调：记录超时追踪事件并调度端点的后台 `Work`。
fn tcp_expired(timer: Pin<&Timer>) {
    // SAFETY:
    // 1. `timer.context()` 在 `TcpEndpoint::try_new` 中由 `Arc::as_ptr(&endpoint) as usize` 初始化，指向堆上地址稳定的 `TcpEndpoint`；
    // 2. 端点在最后释放前必定经过 `stop_sync`，其中先调用 `timer.close_sync()` 停止定时器并等待正在运行的到期回调彻底执行完毕，保证解引用期间 `TcpEndpoint` 始终存活。
    let endpoint = unsafe { &*(timer.context() as *const TcpEndpoint) };
    trace::record(trace::Kind::ProtocolDeadline, endpoint.id, 0);
    endpoint.schedule();
}
