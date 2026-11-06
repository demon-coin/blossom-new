//! UDP 传输端点（`UdpEndpoint`）与数据报收发状态管理。
//!
//! # 模块职责与架构定位
//! 本模块位于网络协议栈的传输层，向上承接 `net::socket` 中的数据报套接字文件描述，向下依赖：
//! - `smoltcp::socket::udp::Socket`：管理收发方向的离散数据报记录槽位（`PACKET_RECORDS`）与有效载荷环形缓冲区（`PAYLOAD_CAPACITY`）；
//! - `net::table::NetTables`：维护全局 UDP 端口绑定租约（`BindLease`）及接收查找索引；
//! - `net::NetStack`：执行出口路由解析、IP/UDP 报文发送与接收分发，以及端点的异步退休回收；
//! - `workqueue::Work`：在后台异步消费输入数据报队列并向网络栈刷出待发送数据报。
//!
//! # 核心设计与数据流转
//!
//! ## 1. 延迟缓冲区分配与租约旁表（`leases`）
//! - **冷启动与延迟分配**：`UdpEndpoint::try_new` 创建时仅初始化零容量的 `udp::Socket`，不在套接字刚打开时分配
//!   两份 128 KiB 的收发缓冲区；直到首次执行 `bind`、`connect` 或 `send` 时才通过 `State::prepare_buffers` 在堆上分配。
//! - **逐包端口租约保活**：每次 `send` 成功将一条数据报写入 `udp::Socket` 发送队列时，都会在 `State::leases`
//!   队列末尾同步推入一份当前 `BindLease` 的 `Arc` 克隆；后台 `dispatch` 取出并发送该条数据报时再从 `leases`
//!   队首弹出并附加到 `OwnedPacket` 上。这样即使应用层在数据报发出前断开重绑或关闭套接字，在途数据报占用的源端口也不会被提前复用。
//!
//! ```text
//!   [用户态 send / sendto]                     [后台 Work: run_udp]
//!             |                                         |
//!   1. 解析路由与源端口租约 (BindLease)         1. process_input: 从 input.packets
//!   2. prepare_buffers() 按需分配 128KiB 缓冲      取出报文写入 socket 接收队列
//!   3. socket.send_slice(payload, meta)        2. dispatch: 从 socket 发送队列取出报文
//!   4. leases.push_back(lease.clone())            与 leases.pop_front() 配对组装 OwnedPacket
//!             |                                         |
//!             +----------------> 唤醒 / 调度 -----------+
//! ```
//!
//! ## 2. 关闭与异步排空退休生命周期
//! ```text
//!   Created / Bound / Connected
//!       |
//!       +-- 用户态关闭最后一个套接字文件描述 (close)
//!             |
//!             v
//!   标记 state.closed = true, input.closed = true
//!   触发 OperationGuard::drop -> schedule() 调度后台 Work
//!             |
//!             +-- 若 state.leases 非空（仍有待发 TX 数据报）：
//!             |     继续在后台 dispatch 发送剩余数据报（遇 WouldBlock 则等待出口容量恢复）
//!             |
//!             +-- 当 state.closed && state.leases.is_empty() (ended)：
//!                   关闭底层 socket，取消出口等待，从全局表注销 UDP 索引，
//!                   调用 net.retire_udp(id) 移交异步退休线程执行 stop_sync()
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 严禁违反以下锁层级顺序，否则会导致死锁：
//! 1. **端点状态睡眠锁 `UdpEndpoint::state`（`SleepMutex`）**：最外层锁，保护 `udp::Socket`、端口租约与连接状态。
//! 2. **网络栈控制准入锁 `NetStack::admission()`（`SleepMutex`）**：在 `bind`、`connect`、`send`、`set_reuse_address`
//!    中更新全局绑定表或解析路由时获取，位于 `state` 锁之内、所有自旋锁之外。
//! 3. **端点内部自旋锁（`SpinLock`）**：包括 `input`、`timeouts` 以及 `NetTables` 内部自旋锁。
//!    自旋锁临界区极短且互不嵌套；必须在释放 `state` 与自旋锁之后才能调用 `PollWaitHead::notify` 或 `schedule`。

use alloc::collections::VecDeque;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::ops::{Deref, DerefMut};
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use smoltcp::iface::IpPayload;
use smoltcp::phy::PacketMeta;
use smoltcp::socket::udp::{self, UdpContextState, UdpMetadata};
use smoltcp::wire::{IpAddress, IpListenEndpoint};
use spin::Once;

use crate::fs::poll::{PollMask, PollTable, PollWaitHead};
use crate::fs::{FsError, ReadBuffer};
use crate::net::address::{AddressFamily, AddressOperation, InetEndpoint, SocketAddress};
use crate::net::lifecycle::{EgressWait, RetirementLink};
use crate::net::packet::{
    zeroed_bytes, OwnedPacket, IPV6_UDP_HEADER_LEN, MAX_IPV4_UDP_PAYLOAD_LEN,
    MAX_IPV6_UDP_PAYLOAD_LEN,
};
use crate::net::socket::{SocketTimeouts, MSG_TRUNC, SHUT_RD, SHUT_RDWR, SHUT_WR};
use crate::net::table::{BindLease, TransportProtocol};
use crate::net::{EndpointSlot, IoProgress, MessageResult, NetStack, NetStackRef, PROTOCOL_BUDGET};
use crate::sync::{SleepMutex, SleepMutexGuard, SpinLock};
use crate::workqueue::Work;

const PACKET_RECORDS: usize = 32;
const PAYLOAD_CAPACITY: usize = 128 * 1024;

/// 端点的输入数据报缓冲队列与后台调度交接标志（由 `UdpEndpoint::input` 自旋锁保护）。
struct Input {
    /// 端点是否已关闭输入接收；置位后拒绝新报文入队。
    closed: bool,
    /// 标记后台 `Work` 在尝试获取 `state` 睡眠锁时是否因锁被占用而退让；
    /// 当前持有 `state` 锁的 `OperationGuard` 在释放锁时负责接力重新调度该 `Work`。
    owner_waiting: bool,
    /// 待由后台 `Work` 消费的输入 IP/UDP 报文有界队列（容量上限为 `PACKET_RECORDS`）。
    packets: VecDeque<OwnedPacket>,
}

/// UDP 端点的核心协议状态、收发队列与绑定控制属性（由 `UdpEndpoint::state` 睡眠锁保护）。
struct State {
    /// 底层 `smoltcp` UDP 套接字实例，维护收发环形缓冲区与报文元数据槽位。
    socket: udp::Socket<'static>,
    /// 标记是否已为 `socket` 分配并安装了收发后备缓冲区（延迟分配标志）。
    buffers_ready: bool,
    /// 接收缓冲区的字节配额上限（对应 `SO_RCVBUF`，单位：字节）。
    receive_budget: usize,
    /// 发送缓冲区的字节配额上限（对应 `SO_SNDBUF`，单位：字节）。
    send_budget: usize,
    /// 与 `socket` 发送队列中待发数据报一一对应的本地端口绑定租约队列，确保在途数据报完成发送前源端口租约保持有效。
    leases: VecDeque<Arc<BindLease>>,
    /// 当前端点持有的本地地址与端口绑定租约（显式 `bind` 或自动分配产生）。
    binding: Option<Arc<BindLease>>,
    /// 通过 `connect` 设置的默认远端对端地址与端口；未连接时为 `None`。
    peer: Option<InetEndpoint>,
    /// 已调用 `connect` 时根据路由解析出的具体本地源地址与端口，供 `getsockname` 返回。
    route_local: Option<InetEndpoint>,
    /// 标记当前 `binding` 是否由用户态显式调用 `bind` 产生；用于区分 `connect(None)` 断开时是否应保留本地绑定。
    explicit_bind: bool,
    /// 是否启用 `SO_REUSEADDR` 地址复用选项。
    reuse: bool,
    /// IPv6 端点是否启用 `IPV6_V6ONLY`（仅限 IPv6 通信，禁止 IPv4 映射地址）。
    v6_only: bool,
    /// 读方向是否已通过 `shutdown(SHUT_RD)` 或 `close` 关闭。
    read_closed: bool,
    /// 写方向是否已通过 `shutdown(SHUT_WR)` 或 `close` 关闭。
    write_closed: bool,
    /// 端点是否已调用 `close` 进入关闭状态。
    closed: bool,
    /// 待由用户态通过 `SO_ERROR` 取走的后台异步发送错误。
    error: Option<FsError>,
}

impl State {
    /// 在持有 `state` 睡眠锁的前提下按需分配并安装收发缓冲区；若已分配则直接返回。
    fn prepare_buffers(&mut self) -> Result<(), FsError> {
        if self.buffers_ready {
            return Ok(());
        }

        let rx = UdpEndpoint::packet_buffer()?;
        let tx = UdpEndpoint::packet_buffer()?;
        let mut socket = udp::Socket::new(rx, tx);
        socket.set_recv_budget(self.receive_budget);
        socket.set_send_budget(self.send_budget);
        socket.set_hop_limit(self.socket.hop_limit());

        self.socket = socket;
        self.buffers_ready = true;
        Ok(())
    }

    /// 判断当前发送缓冲区是否可接纳新数据报（尚未分配缓冲区的冷端点视为可写）。
    fn can_send(&self) -> bool {
        !self.buffers_ready || self.socket.can_send()
    }

    /// 根据当前收发缓冲区状态、方向关闭标志及异步错误计算 `PollMask` 就绪掩码。
    fn mask(&self) -> PollMask {
        if self.closed {
            return PollMask::POLLERR | PollMask::POLLHUP;
        }
        let mut mask = PollMask::default();
        if self.socket.can_recv() || self.read_closed {
            mask |= PollMask::POLLIN | PollMask::POLLRDNORM;
        }
        if self.can_send() || self.write_closed {
            mask |= PollMask::POLLOUT | PollMask::POLLWRNORM;
        }
        if self.read_closed {
            mask |= PollMask::POLLRDHUP;
        }
        if self.read_closed && self.write_closed {
            mask |= PollMask::POLLHUP;
        }
        if self.error.is_some() {
            mask |= PollMask::POLLERR;
        }
        mask
    }

    /// 检查当前端点是否允许发送数据报（即便发送 0 字节数据报也必须检查关闭与 `SHUT_WR` 状态）。
    fn check_send(&self) -> Result<(), FsError> {
        if self.closed {
            return Err(FsError::NotConnected);
        }
        if self.write_closed {
            return Err(FsError::BrokenPipe);
        }
        Ok(())
    }
}

/// UDP 传输层端点，封装数据报套接字的协议队列、绑定状态与后台工作项。
pub struct UdpEndpoint {
    /// 网络栈内全局唯一且不复用的端点标识号。
    id: u64,
    /// 端点创建时固定的地址族（`AF_INET` 或 `AF_INET6`）。
    family: AddressFamily,
    /// 所属网络栈实例的弱引用。
    stack: Weak<NetStack>,
    /// 保护 `State` 协议状态与收发队列的可睡眠互斥锁。
    state: SleepMutex<State>,
    /// 保护输入数据报队列与后台 `Work` 交接标志的自旋锁。
    input: SpinLock<Input>,
    /// 保护收发超时配置（`SO_RCVTIMEO` / `SO_SNDTIMEO`）的自旋锁。
    timeouts: SpinLock<SocketTimeouts>,
    /// 供 `poll` / `select` / `epoll` 注册和唤醒的轮询等待队列头。
    source: Arc<PollWaitHead>,
    /// 推进后台收包入队与发送分发的异步工作项。
    work: Once<Pin<Arc<Work>>>,
    /// 标记端点是否已进入同步停止流程（`stop_sync`），置位后拒绝新调度请求。
    stopped: AtomicBool,
    /// 端点进入异步退休链表时的侵入式链接状态。
    pub(crate) retirement: RetirementLink,
    /// 发送通道（Egress）容量不足时的轻量级等待链表节点。
    egress_wait: Arc<EgressWait>,
    /// 占用网络栈全局端点配额的槽位守卫，随端点析构自动归还配额。
    _slot: EndpointSlot,
}

/// 获取 `UdpEndpoint::state` 睡眠锁时的操作模式。
#[derive(Clone, Copy, Eq, PartialEq)]
enum OperationMode {
    /// 修改模式：操作可能改变协议状态、关闭标志或队列配额，释放锁后需要计算新增事件通知并主动调度后台 `Work`。
    Modify,
    /// 观察模式：仅读取状态，释放锁时不触发事件通知，仅在后台 `Work` 曾受阻退让（`owner_waiting`）时负责接力调度。
    Observe,
}

/// UDP 端点状态操作守卫，封装 `state` 睡眠锁并在析构时统一完成轮询唤醒与后台 `Work` 调度交接。
struct OperationGuard<'a> {
    /// 所属的 UDP 端点引用。
    endpoint: &'a UdpEndpoint,
    /// 底层 `State` 睡眠互斥锁守卫；在 `Drop::drop` 中提前 `take()` 释放，以便在锁外执行通知和调度。
    state: Option<SleepMutexGuard<'a, State>>,
    /// 本次持锁的操作模式（修改或只读观察）。
    mode: OperationMode,
    /// 获取锁时的初始 `PollMask` 就绪掩码。
    initial_mask: PollMask,
    /// 获取锁时的初始发送配额（`send_budget`，单位：字节）。
    initial_send_budget: usize,
    /// 获取锁时的初始读关闭标志。
    initial_read_closed: bool,
    /// 获取锁时的初始写关闭标志。
    initial_write_closed: bool,
    /// 获取锁时的初始端点关闭标志。
    initial_closed: bool,
}

impl Deref for OperationGuard<'_> {
    type Target = State;

    /// 不可变借用受保护的 `State` 实例。
    fn deref(&self) -> &State {
        self.state.as_deref().unwrap()
    }
}

impl DerefMut for OperationGuard<'_> {
    /// 可变借用受保护的 `State` 实例。
    fn deref_mut(&mut self) -> &mut State {
        self.state.as_deref_mut().unwrap()
    }
}

impl Drop for OperationGuard<'_> {
    /// 在持有 `state` 锁的末尾对比初始快照计算新增的轮询就绪事件，随后释放 `state` 锁，
    /// 在锁外唤醒等待队列并按需调度后台 `Work`。
    fn drop(&mut self) {
        // Step 1: 在修改模式下比对操作前后的状态差异，汇总新产生的轮询就绪掩码。
        let notification = if self.mode == OperationMode::Modify {
            let state = self.state.as_deref().unwrap();
            let mut mask = state.mask() & !self.initial_mask;
            if !self.initial_read_closed && state.read_closed {
                mask |= PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLRDHUP;
            }
            if !self.initial_write_closed && state.write_closed {
                mask |= PollMask::POLLOUT | PollMask::POLLWRNORM;
            }
            if !self.initial_closed && state.closed {
                mask |= PollMask::POLLERR | PollMask::POLLHUP;
            }
            if self.initial_send_budget < state.send_budget && state.can_send() {
                mask |= PollMask::POLLOUT | PollMask::POLLWRNORM;
            }
            mask
        } else {
            PollMask::default()
        };
        // Step 2: 释放 state 睡眠锁。
        drop(self.state.take());

        // Step 3: 检查持锁期间是否有后台 Work 因锁被占用而退让，并在锁外完成轮询通知与后台任务调度。
        let owner_waiting = {
            let mut input = self.endpoint.input.lock();
            core::mem::take(&mut input.owner_waiting)
        };
        if !notification.is_empty() {
            self.endpoint.source.notify(notification);
        }
        if self.mode == OperationMode::Modify || owner_waiting {
            self.endpoint.schedule();
        }
    }
}

impl UdpEndpoint {
    /// 在指定网络栈中创建并初始化一个新的 `UdpEndpoint` 实例。
    ///
    /// 初始创建零容量的 `udp::Socket`（延迟到首次绑定、连接或发送时才分配 128 KiB 缓冲区），
    /// 预留有界输入队列与租约队列，并初始化后台 `Work` 与轮询等待队列头。
    pub(crate) fn try_new(
        net: &NetStackRef,
        id: u64,
        slot: EndpointSlot,
        family: AddressFamily,
    ) -> Result<Arc<Self>, FsError> {
        let socket = udp::Socket::new(
            udp::PacketBuffer::new(Vec::new(), Vec::new()),
            udp::PacketBuffer::new(Vec::new(), Vec::new()),
        );
        let mut packets = VecDeque::new();
        packets
            .try_reserve_exact(PACKET_RECORDS)
            .map_err(|_| FsError::NoMemory)?;
        let mut leases = VecDeque::new();
        leases
            .try_reserve_exact(PACKET_RECORDS)
            .map_err(|_| FsError::NoMemory)?;

        let endpoint = Arc::try_new(Self {
            id,
            family,
            stack: Arc::downgrade(net),
            state: SleepMutex::new(State {
                socket,
                buffers_ready: false,
                receive_budget: PAYLOAD_CAPACITY,
                send_budget: PAYLOAD_CAPACITY,
                leases,
                binding: None,
                peer: None,
                route_local: None,
                explicit_bind: false,
                reuse: false,
                v6_only: false,
                read_closed: false,
                write_closed: false,
                closed: false,
                error: None,
            }),
            input: SpinLock::new(Input {
                closed: false,
                owner_waiting: false,
                packets,
            }),
            timeouts: SpinLock::new(SocketTimeouts::default()),
            source: Arc::try_new(PollWaitHead::new()).map_err(|_| FsError::NoMemory)?,
            work: Once::new(),
            stopped: AtomicBool::new(false),
            retirement: RetirementLink::new(),
            egress_wait: EgressWait::try_new(id)?,
            _slot: slot,
        })
        .map_err(|_| FsError::NoMemory)?;

        // 初始化端点的后台处理 Work，交由所属网络栈的工作队列调度执行。
        let work = Work::try_new(
            net.work_queue(),
            0,
            run_udp,
            Arc::as_ptr(&endpoint) as usize,
        )
        .map_err(|_| FsError::NoMemory)?;
        endpoint.work.call_once(|| work);
        Ok(endpoint)
    }

    /// 读取当前端点的收发超时时间配置（`SO_RCVTIMEO` / `SO_SNDTIMEO`）。
    ///
    /// 仅获取 `timeouts` 自旋锁，无需等待 `state` 睡眠锁。
    pub(crate) fn timeouts(&self) -> SocketTimeouts {
        *self.timeouts.lock()
    }

    /// 设置收发方向的阻塞等待超时时间（单位：纳秒，`None` 表示无限等待）。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        self.timeouts.lock().set(receive, timeout);
    }

    /// 在堆上分配单向 `udp::PacketBuffer`（包含 `PACKET_RECORDS` 个报文元数据槽位与 `PAYLOAD_CAPACITY` 字节载荷缓冲）。
    fn packet_buffer() -> Result<udp::PacketBuffer<'static>, FsError> {
        let mut metadata = Vec::new();
        metadata
            .try_reserve_exact(PACKET_RECORDS)
            .map_err(|_| FsError::NoMemory)?;
        metadata.resize(PACKET_RECORDS, udp::PacketMetadata::EMPTY);
        Ok(udp::PacketBuffer::new(
            metadata,
            zeroed_bytes(PAYLOAD_CAPACITY)?,
        ))
    }

    /// 按指定操作模式（修改或只读观察）获取 `state` 睡眠锁，并采集初始状态快照构造 `OperationGuard`。
    fn operation(&self, mode: OperationMode) -> OperationGuard<'_> {
        let state = self.state.lock();
        let initial_mask = state.mask();
        let initial_send_budget = state.send_budget;
        let initial_read_closed = state.read_closed;
        let initial_write_closed = state.write_closed;
        let initial_closed = state.closed;
        OperationGuard {
            endpoint: self,
            state: Some(state),
            mode,
            initial_mask,
            initial_send_budget,
            initial_read_closed,
            initial_write_closed,
            initial_closed,
        }
    }

    /// 返回端点在所属网络栈中的唯一标识号。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 供内核测试（`ktest`）观察底层 `udp::Socket` 是否已分配后备缓冲区及其实际报文槽位和字节容量。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_buffer_storage(&self) -> (bool, [usize; 4]) {
        let state = self.operation(OperationMode::Observe);
        (
            state.buffers_ready,
            [
                state.socket.packet_recv_capacity(),
                state.socket.payload_recv_capacity(),
                state.socket.packet_send_capacity(),
                state.socket.payload_send_capacity(),
            ],
        )
    }

    /// 返回端点的发送通道（Egress）容量等待链表节点引用。
    pub(crate) fn egress_wait(&self) -> &Arc<EgressWait> {
        &self.egress_wait
    }

    /// 返回端点创建时固定的地址族（`AF_INET` 或 `AF_INET6`）。
    pub fn family(&self) -> AddressFamily {
        self.family
    }

    /// 查询 IPv6 端点是否开启了 `IPV6_V6ONLY` 选项。
    pub fn v6_only(&self) -> bool {
        self.operation(OperationMode::Observe).v6_only
    }

    /// 设置 IPv6 端点的 `IPV6_V6ONLY` 选项（仅允许在尚未绑定本地端口的 `AF_INET6` 端点上修改）。
    pub fn set_v6_only(&self, value: bool) -> Result<(), FsError> {
        let mut state = self.operation(OperationMode::Modify);
        if self.family != AddressFamily::Ipv6 || state.binding.is_some() {
            return Err(FsError::InvalidOperation);
        }
        if state.closed {
            return Err(FsError::NotConnected);
        }
        state.v6_only = value;
        Ok(())
    }

    /// 校验目标或本地 IP 地址是否符合当前端点的地址族与 `IPV6_V6ONLY` 策略，并按操作类型映射对应的错误码。
    fn check_family(
        &self,
        state: &State,
        address: IpAddress,
        operation: AddressOperation,
    ) -> Result<(), FsError> {
        match (self.family, address) {
            (AddressFamily::Ipv4, IpAddress::Ipv6(_)) => Err(FsError::AddressFamilyNotSupported),
            (AddressFamily::Ipv6, IpAddress::Ipv4(_)) if state.v6_only => Err(match operation {
                AddressOperation::Bind => FsError::InvalidOperation,
                AddressOperation::Connect => FsError::AddressFamilyNotSupported,
                AddressOperation::Send => FsError::NetworkUnreachable,
            }),
            _ => Ok(()),
        }
    }

    /// 判断本次绑定是否应作为 IPv6 双栈通配绑定（即 `AF_INET6`、未开启 `IPV6_V6ONLY` 且绑定在 `::` 未指定地址上）。
    fn dual_stack(&self, state: &State, local: InetEndpoint) -> bool {
        self.family == AddressFamily::Ipv6
            && !state.v6_only
            && matches!(local.address, IpAddress::Ipv6(address) if address.is_unspecified())
    }

    /// 将端点显式绑定到指定的本地地址与端口（`bind` 系统调用）。
    ///
    /// 先按需分配收发缓冲区并向全局表申请端口租约，随后在全局 UDP 表中发布接收索引；
    /// 若绑定到具体的非通配 IPv6 地址，则同时将 `v6_only` 固定为 `true`。
    pub fn bind(self: &Arc<Self>, requested: InetEndpoint) -> Result<InetEndpoint, FsError> {
        // Step 1: 校验端点未关闭、未重复绑定且请求地址符合地址族策略。
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        if state.binding.is_some() {
            return Err(FsError::InvalidOperation);
        }
        self.check_family(&state, requested.address, AddressOperation::Bind)?;

        // Step 2: 获取网络栈准入锁，分配收发缓冲区并申请本地端口绑定租约。
        let net = self.stack.upgrade().ok_or(FsError::NotConnected)?;
        let _admission = net.admission()?;
        state.prepare_buffers()?;
        let lease = net.tables().reserve_binding(
            &net,
            self.id,
            TransportProtocol::Udp,
            requested,
            state.reuse,
            self.dual_stack(&state, requested),
        )?;
        let local = lease.local();

        // Step 3: 在全局表中发布 UDP 接收索引，并更新端点绑定状态。
        net.tables().publish_udp(self.id, local, None)?;
        state.socket.set_endpoint(local.raw_listen());
        if matches!(local.address, IpAddress::Ipv6(address) if !address.is_unspecified()) {
            state.v6_only = true;
        }
        state.binding = Some(lease);
        state.explicit_bind = true;
        Ok(local)
    }

    /// 为尚未显式绑定的端点向全局表申请本地址族通配地址上的临时端口租约。
    fn automatic_binding(
        &self,
        net: &NetStackRef,
        state: &State,
    ) -> Result<Arc<BindLease>, FsError> {
        net.tables().reserve_binding(
            net,
            self.id,
            TransportProtocol::Udp,
            InetEndpoint {
                address: self.family.unspecified(),
                port: 0,
            },
            state.reuse,
            self.family == AddressFamily::Ipv6 && !state.v6_only,
        )
    }

    /// 设置或清除 UDP 端点的默认远端对端地址（`connect` 系统调用，`peer` 为 `None` 表示 `AF_UNSPEC` 断开连接）。
    ///
    /// 若断开连接且此前为自动分配的临时端口绑定，则一并撤销全局 UDP 接收索引并释放当前绑定租约（已在发送队列中的报文由 `leases` 旁表继续保活租约）。
    pub fn connect(self: &Arc<Self>, peer: Option<InetEndpoint>) -> Result<(), FsError> {
        // Step 1: 获取状态锁与网络栈准入锁；若 peer 为 None，执行断开连接逻辑。
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        let net = self.stack.upgrade().ok_or(FsError::NotConnected)?;
        let _admission = net.admission()?;
        let Some(peer) = peer else {
            if state.explicit_bind {
                let local = state
                    .binding
                    .as_ref()
                    .expect("explicit UDP binding")
                    .local();
                net.tables().publish_udp(self.id, local, None)?;
                state.socket.set_endpoint(local.raw_listen());
            } else {
                net.tables().unpublish_udp(self.id);
                state.socket.set_endpoint(IpListenEndpoint::default());
                // 已进入发送队列的数据报在 state.leases 或已发出的 OwnedPacket 中各自持有旧租约的 Arc，
                // 此处移除当前 binding 不会影响在途数据报占用的源端口。
                state.binding.take();
            }
            state.peer = None;
            state.route_local = None;
            return Ok(());
        };

        // Step 2: 校验远端地址与端口有效性，解析出口路由并按需分配收发缓冲区与本地端口租约。
        if peer.port == 0 {
            return Err(FsError::InvalidOperation);
        }
        self.check_family(&state, peer.address, AddressOperation::Connect)?;
        let bound = state
            .binding
            .as_ref()
            .map_or(self.family.unspecified(), |lease| lease.local().address);
        let route = net.resolve_route(peer.address, bound)?;

        state.prepare_buffers()?;
        let lease = match state.binding.as_ref() {
            Some(lease) => Arc::clone(lease),
            None => self.automatic_binding(&net, &state)?,
        };
        let local = InetEndpoint {
            address: route.source,
            port: lease.local().port,
        };

        // Step 3: 在全局 UDP 表中更新带对端过滤的索引，并记录默认对端与路由源地址。
        net.tables().publish_udp(self.id, local, Some(peer))?;
        state.socket.set_endpoint(local.raw_listen());
        state.binding = Some(lease);
        state.route_local = Some(local);
        state.peer = Some(peer);
        Ok(())
    }

    /// 将一条完整的 UDP 数据报写入端点发送队列（`send` / `sendto` / `sendmsg`）。
    ///
    /// 依次校验发送方向状态、目的地址与报文长度上限，解析出口路由与 MTU，按需分配缓冲区与自动端口租约，
    /// 最后将有效载荷写入 `udp::Socket` 发送缓冲区，并将关联的端口租约推入 `state.leases` 队列。
    pub fn send(
        self: &Arc<Self>,
        payload: &[u8],
        destination: Option<InetEndpoint>,
    ) -> Result<IoProgress, FsError> {
        // Step 1: 校验端点写状态、目的地址有效性以及协议与发送缓冲区配额上限。
        let mut state = self.operation(OperationMode::Modify);
        state.check_send()?;
        let destination = destination
            .or(state.peer)
            .ok_or(FsError::DestinationRequired)?;
        if destination.port == 0 {
            return Err(FsError::InvalidOperation);
        }
        self.check_family(&state, destination.address, AddressOperation::Send)?;
        let protocol_limit = match destination.address {
            IpAddress::Ipv4(_) => MAX_IPV4_UDP_PAYLOAD_LEN,
            IpAddress::Ipv6(_) => MAX_IPV6_UDP_PAYLOAD_LEN,
        };
        if protocol_limit < payload.len() || state.send_budget < payload.len() {
            return Err(FsError::MessageTooLarge);
        }

        // Step 2: 获取网络栈准入锁，解析出口路由并检查 IPv6 不分片路径 MTU 限制。
        let net = self.stack.upgrade().ok_or(FsError::NotConnected)?;
        let _admission = net.admission()?;
        let bound = state
            .binding
            .as_ref()
            .map_or(self.family.unspecified(), |lease| lease.local().address);
        let route = net.resolve_route(destination.address, bound)?;
        if matches!(destination.address, IpAddress::Ipv6(_))
            && route.mtu.saturating_sub(IPV6_UDP_HEADER_LEN) < payload.len()
        {
            return Err(FsError::MessageTooLarge);
        }

        // Step 3: 按需准备收发缓冲区，检查发送队列剩余空间，并获取或自动分配本地端口绑定租约。
        state.prepare_buffers()?;
        if !state.socket.can_send_size(payload.len()) {
            return Ok(IoProgress::Blocked);
        }
        let lease = match state.binding.as_ref() {
            Some(lease) => Arc::clone(lease),
            None => {
                let lease = self.automatic_binding(&net, &state)?;
                net.tables()
                    .publish_udp(self.id, lease.local(), state.peer)?;
                state.socket.set_endpoint(lease.local().raw_listen());
                state.binding = Some(Arc::clone(&lease));
                lease
            }
        };
        let metadata = UdpMetadata {
            endpoint: destination.raw(),
            local_address: Some(route.source),
            local_port: Some(lease.local().port),
            meta: PacketMeta::default(),
        };
        // 端口绑定与路由解析已完成，提前释放网络栈全局准入锁，避免后续复制数据报载荷时阻塞其他端点的控制操作。
        drop(_admission);

        // Step 4: 将数据报写入底层发送缓冲区，并在成功后将对应端口租约推入 leases 队列保活。
        match state.socket.send_slice(payload, metadata) {
            Ok(()) => {
                assert!(state.leases.len() < PACKET_RECORDS);
                state.leases.push_back(lease);
                Ok(IoProgress::Ready(payload.len()))
            }
            Err(udp::SendError::BufferFull) => Ok(IoProgress::Blocked),
            Err(udp::SendError::Unaddressable) => Err(FsError::AddressNotAvailable),
        }
    }

    /// 验证收到的 IP/UDP 报文格式并将其推入端点的有界输入队列，随后调度后台 `Work` 消费。
    ///
    /// 若输入队列已满（达到 `PACKET_RECORDS` 上限），则返回 `Err(FsError::WouldBlock)` 丢弃该报文。
    pub fn enqueue(&self, packet: OwnedPacket) -> Result<(), FsError> {
        packet.udp_payload()?;
        {
            let mut input = self.input.lock();
            if input.closed {
                return Err(FsError::NotConnected);
            }
            if PACKET_RECORDS <= input.packets.len() {
                return Err(FsError::WouldBlock);
            }
            input.packets.push_back(packet);
        }

        self.schedule();
        Ok(())
    }

    /// 从 UDP 接收队列读取一条数据报到 `target`（支持 `MSG_PEEK` 预览模式）。
    ///
    /// 当接收队列为空但读方向已关闭（`SHUT_RD`）时返回 0 字节结果；非预览模式下无论用户缓冲区是否发生截断或页错误，都会消费整条数据报。
    pub fn receive(
        &self,
        target: &mut ReadBuffer<'_>,
        peek: bool,
    ) -> Result<Option<MessageResult>, FsError> {
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        if !state.socket.can_recv() {
            return Ok(state.read_closed.then_some(MessageResult {
                copied: 0,
                original_len: 0,
                source: None,
                flags: 0,
            }));
        }

        if peek {
            let (payload, metadata) = state.socket.peek().expect("UDP readable record");
            self.copy_datagram(target, payload, InetEndpoint::from_raw(metadata.endpoint))
                .map(Some)
        } else {
            // 非预览模式下先从接收队列弹出整条数据报记录；即便用户缓冲区较小发生截断或中途缺页失败，剩余尾部字节也会按 UDP 数据报语义直接丢弃。
            let (payload, metadata) = state.socket.recv().expect("UDP readable record");
            self.copy_datagram(target, payload, InetEndpoint::from_raw(metadata.endpoint))
                .map(Some)
        }
    }

    /// 将单条 UDP 数据报载荷复制到 `target` 缓冲区，构造包含源地址、原始长度及截断标志（`MSG_TRUNC`）的 `MessageResult`。
    fn copy_datagram(
        &self,
        target: &mut ReadBuffer<'_>,
        payload: &[u8],
        source: InetEndpoint,
    ) -> Result<MessageResult, FsError> {
        let original_len = payload.len();
        let copied = target.push(payload)?;
        Ok(MessageResult {
            copied,
            original_len,
            source: Some(SocketAddress::from_endpoint(source, self.family)),
            flags: if copied < original_len { MSG_TRUNC } else { 0 },
        })
    }

    /// 供内核测试（`ktest`）查询当前端点持有的本地绑定租约地址。
    #[cfg(feature = "ktest")]
    pub fn local(&self) -> Option<InetEndpoint> {
        self.operation(OperationMode::Observe)
            .binding
            .as_ref()
            .map(|lease| lease.local())
    }

    /// 设置 `SO_REUSEADDR` 地址复用选项，并在已持有绑定租约时同步更新全局绑定表策略。
    pub fn set_reuse_address(&self, reuse: bool) -> Result<(), FsError> {
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        let net = self.stack.upgrade().ok_or(FsError::NotConnected)?;
        let _admission = net.admission()?;
        if let Some(lease) = state.binding.as_ref() {
            net.tables().set_reuse(lease.id(), reuse)?;
        }
        state.reuse = reuse;
        Ok(())
    }

    /// 查询当前端点是否开启了 `SO_REUSEADDR` 地址复用选项。
    pub fn reuse_address(&self) -> bool {
        self.operation(OperationMode::Observe).reuse
    }

    /// 获取当前端点的本地绑定地址与端口（`getsockname`）。
    ///
    /// 若已通过 `connect` 建立连接则返回路由选定的具体源地址；若仅完成绑定则返回租约记录的地址；未绑定时返回通配零地址和 0 端口。
    pub fn local_address(&self) -> Result<InetEndpoint, FsError> {
        let state = self.operation(OperationMode::Observe);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        Ok(state
            .route_local
            .or_else(|| state.binding.as_ref().map(|lease| lease.local()))
            .unwrap_or(InetEndpoint {
                address: self.family.unspecified(),
                port: 0,
            }))
    }

    /// 获取通过 `connect` 设置的默认远端对端地址与端口（`getpeername`）。
    pub fn peer_address(&self) -> Result<InetEndpoint, FsError> {
        let state = self.operation(OperationMode::Observe);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        state.peer.ok_or(FsError::NotConnected)
    }

    /// 返回当前配置的接收缓冲区字节配额（`SO_RCVBUF`）。
    pub fn receive_budget(&self) -> usize {
        self.operation(OperationMode::Observe).receive_budget
    }

    /// 返回当前配置的发送缓冲区字节配额（`SO_SNDBUF`）。
    pub fn send_budget(&self) -> usize {
        self.operation(OperationMode::Observe).send_budget
    }

    /// 调整接收缓冲区的字节配额（`SO_RCVBUF`，有效范围为 `1..=PAYLOAD_CAPACITY`）。
    pub fn set_receive_budget(&self, budget: usize) -> Result<(), FsError> {
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        if budget == 0 || PAYLOAD_CAPACITY < budget {
            return Err(FsError::InvalidOperation);
        }
        if state.buffers_ready {
            state.socket.set_recv_budget(budget);
        }
        state.receive_budget = budget;
        Ok(())
    }

    /// 调整发送缓冲区的字节配额（`SO_SNDBUF`，有效范围为 `1..=PAYLOAD_CAPACITY`）；若扩容后恢复可写，守卫析构时会自动唤醒 `POLLOUT` 等待者。
    pub fn set_send_budget(&self, budget: usize) -> Result<(), FsError> {
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        if budget == 0 || PAYLOAD_CAPACITY < budget {
            return Err(FsError::InvalidOperation);
        }
        if state.buffers_ready {
            state.socket.set_send_budget(budget);
        }
        state.send_budget = budget;
        Ok(())
    }

    /// 返回接收缓冲区的最大物理字节容量（`PAYLOAD_CAPACITY`）。
    pub fn receive_capacity(&self) -> usize {
        PAYLOAD_CAPACITY
    }

    /// 返回发送缓冲区的最大物理字节容量（`PAYLOAD_CAPACITY`）。
    pub fn send_capacity(&self) -> usize {
        PAYLOAD_CAPACITY
    }

    /// 读取并清除挂起的后台异步发送错误（对应 `getsockopt(SO_ERROR)`）。
    pub fn take_error(&self) -> Option<FsError> {
        self.operation(OperationMode::Modify).error.take()
    }

    /// 关闭指定传输方向（`SHUT_RD`、`SHUT_WR` 或 `SHUT_RDWR`）；若端点尚未通过 `connect` 关联默认对端，则在更新关闭标志后返回 `Err(FsError::NotConnected)`。
    pub fn shutdown(&self, how: u32) -> Result<(), FsError> {
        if SHUT_RDWR < how {
            return Err(FsError::InvalidOperation);
        }
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        if how == SHUT_RD || how == SHUT_RDWR {
            state.read_closed = true;
        }
        if how == SHUT_WR || how == SHUT_RDWR {
            state.write_closed = true;
        }
        if state.peer.is_some() {
            Ok(())
        } else {
            Err(FsError::NotConnected)
        }
    }

    /// 注册可选的轮询等待表，并查询当前端点的 `PollMask` 就绪掩码。
    pub fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&self.source)?;
        }
        Ok(self.operation(OperationMode::Observe).mask())
    }

    /// 当网络栈发送通道（Egress）容量恢复时由网络栈调用，调度后台 `Work` 继续刷出待发数据报。
    pub fn notify_egress(&self) {
        self.schedule();
    }

    /// 将端点的后台处理任务排入工作队列；若端点已同步停止或网络栈正在停止则忽略。
    fn schedule(&self) {
        if self.stopped.load(Ordering::Acquire) {
            return;
        }
        let Some(net) = self.stack.upgrade() else {
            return;
        };
        if !net.is_stopping() {
            let _ = self
                .work
                .get()
                .expect("UDP work is not prepared")
                .as_ref()
                .queue();
        }
    }

    /// 后台 `Work` 的核心执行入口：尝试获取 `state` 睡眠锁，批量处理输入数据报与待发送数据报，并在已关闭且发送队列排空后触发异步退休。
    fn run(&self) {
        let Some(net) = self.stack.upgrade() else {
            return;
        };
        if net.is_stopping() || self.stopped.load(Ordering::Acquire) {
            return;
        }

        // Step 1: 登记 owner_waiting 标志并尝试非阻塞获取 state 睡眠锁；
        // 若用户态正持有 state 锁，则立即返回，由 OperationGuard::drop 负责在解锁后重新调度。
        self.input.lock().owner_waiting = true;
        let Some(mut state) = self.state.try_lock() else {
            return;
        };
        self.input.lock().owner_waiting = false;

        // Step 2: 在未关闭状态下消费输入队列数据报，随后向网络栈分发待发送队列中的数据报。
        let mut notification = PollMask::default();
        if !state.closed {
            self.process_input(&mut state, &mut notification);
        }
        let result = self.dispatch(&net, &mut state, &mut notification);
        if let Err(error) = result {
            if error != FsError::WouldBlock {
                state.error.get_or_insert(error);
                notification |= PollMask::POLLERR;
            }
        }

        // Step 3: 判断端点是否已关闭且所有已入队的待发数据报均已排空（ended），并在锁外唤醒轮询等待者。
        let pending_tx = !state.leases.is_empty();
        let closed = state.closed;
        let ended = closed && !pending_tx;
        let binding = if ended {
            // 所有待发数据报均已移交网络栈或因致命错误终止，关闭底层 socket 并取出本地端口绑定租约以便在锁外释放。
            state.socket.close();
            state.binding.take()
        } else {
            None
        };
        drop(state);

        if !notification.is_empty() {
            self.source.notify(notification);
        }
        // Step 4: 若已彻底结束，封闭输入队列、取消出口等待、从全局表注销索引并移交异步退休；否则按剩余队列状态继续调度。
        if ended {
            let packets = {
                let mut input = self.input.lock();
                input.closed = true;
                core::mem::take(&mut input.packets)
            };
            net.cancel_egress_wait(self.id);
            net.tables().unpublish_udp(self.id);
            drop((binding, packets));
            net.retire_udp(self.id);
            return;
        }

        let pending_input = !closed && !self.input.lock().packets.is_empty();
        if pending_input || (result.is_ok() && pending_tx) {
            self.schedule();
        }
        if result == Err(FsError::WouldBlock) && net.egress_ready(self.egress_wait()) == Ok(true) {
            self.schedule();
        }
    }

    /// 在 `PROTOCOL_BUDGET` 配额内从 `input.packets` 取出收到的 UDP 报文，校验读关闭、默认对端及地址族过滤后写入 `state.socket` 接收缓冲区。
    fn process_input(&self, state: &mut State, notification: &mut PollMask) {
        let mut remaining = PROTOCOL_BUDGET;
        while 0 < remaining {
            let packet = self.input.lock().packets.pop_front();
            let Some(packet) = packet else {
                break;
            };
            remaining -= 1;
            if state.read_closed || state.binding.is_none() {
                continue;
            }
            let (peer, _) = packet.udp_endpoints().expect("UDP backlog protocol");
            if state.peer.is_some_and(|expected| expected != peer)
                || self
                    .check_family(state, peer.address, AddressOperation::Send)
                    .is_err()
            {
                continue;
            }

            let raw = packet.as_packet();
            let ip = raw.ip_repr();
            let IpPayload::Udp(repr, payload) = raw.payload() else {
                unreachable!("UDP backlog protocol");
            };
            let mut context = UdpContextState::new(None, None);
            if state.socket.accepts(&mut context, &ip, repr)
                && state
                    .socket
                    .process(&mut context, PacketMeta::default(), &ip, repr, *payload)
            {
                *notification |= PollMask::POLLIN | PollMask::POLLRDNORM;
            }
        }
    }

    /// 在 `PROTOCOL_BUDGET` 配额内将 `state.socket` 发送缓冲区中的数据报封装为 `OwnedPacket` 并提交给网络栈发送。
    ///
    /// 每成功发出一包（或遇到非 `WouldBlock` 的致命错误），同步弹出 `state.leases` 队首对应的端口租约并置位 `POLLOUT` 通知；
    /// 若网络栈发送通道暂满（`WouldBlock`），则保留当前报文与队首租约留待下次重试。
    fn dispatch(
        &self,
        net: &NetStackRef,
        state: &mut State,
        notification: &mut PollMask,
    ) -> Result<(), FsError> {
        let mut remaining = PROTOCOL_BUDGET;
        while 0 < remaining {
            let Some(lease) = state.leases.front().cloned() else {
                return Ok(());
            };
            remaining -= 1;
            if !net.egress_ready(self.egress_wait())? {
                return Err(FsError::WouldBlock);
            }

            let mut context = UdpContextState::new(None, None);
            let mut completed: Option<Result<(), FsError>> = None;
            state
                .socket
                .dispatch(&mut context, |_, _, (ip, udp, payload)| {
                    let result = (|| {
                        let mut storage = Vec::new();
                        storage
                            .try_reserve_exact(payload.len())
                            .map_err(|_| FsError::NoMemory)?;
                        let packet = OwnedPacket::capture_udp(&ip, &udp, payload, storage)
                            .with_binding(lease);
                        net.try_emit(packet).map_err(|(error, _packet)| error)
                    })();
                    if result == Err(FsError::WouldBlock) {
                        return result;
                    }
                    // 返回 Ok(()) 让 smoltcp 提交并弹出本条发送记录；实际发送结果保存在 completed 中供外层处理。
                    completed = Some(result);
                    Ok::<(), FsError>(())
                })?;

            let result = completed.expect("UDP lease has no corresponding TX record");
            state
                .leases
                .pop_front()
                .expect("UDP TX record has no matching lease");
            *notification |= PollMask::POLLOUT | PollMask::POLLWRNORM;
            if let Err(error) = result {
                state.error.get_or_insert(error);
                *notification |= PollMask::POLLERR;
            }
        }
        Ok(())
    }

    /// 当最后一个关联的套接字文件描述关闭时调用，标记端点与读写双方向关闭并封闭输入队列，交由后台 `Work` 排空剩余待发数据报后异步退休。
    pub fn close(&self) {
        let mut state = self.operation(OperationMode::Modify);
        state.closed = true;
        state.read_closed = true;
        state.write_closed = true;
        state.peer = None;
        state.route_local = None;
        self.input.lock().closed = true;
    }

    /// 同步停止端点并清理所有后台活动与挂起资源。
    ///
    /// 由网络栈异步退休任务或网络栈整体销毁流程在持有端点局部 `Arc` 强引用的前提下调用。
    /// 先等待后台 `Work` 彻底执行完毕，再清空底层套接字队列、端口租约与全局表索引。
    pub(crate) fn stop_sync(&self) {
        assert!(
            !self.stopped.swap(true, Ordering::AcqRel),
            "UDP endpoint stopped twice"
        );
        // Step 1: 封闭输入队列并同步停止后台 Work，等待正在执行的回调彻底退出。
        self.input.lock().closed = true;
        self.work
            .get()
            .expect("UDP work is not prepared")
            .as_ref()
            .close_sync();

        // Step 2: 清空协议状态、底层收发缓冲区、待发租约队列与残留输入报文。
        let binding = {
            let mut state = self.state.lock();
            state.closed = true;
            state.read_closed = true;
            state.write_closed = true;
            state.peer = None;
            state.route_local = None;
            state.socket.close();
            state.leases.clear();
            state.binding.take()
        };
        let packets = {
            let mut input = self.input.lock();
            input.owner_waiting = false;
            core::mem::take(&mut input.packets)
        };

        // Step 3: 取消出口等待、注销全局 UDP 索引，在锁外释放租约与报文并唤醒所有轮询等待者。
        if let Some(net) = self.stack.upgrade() {
            net.cancel_egress_wait(self.id);
            net.tables().unpublish_udp(self.id);
        }
        drop((binding, packets));
        self.source
            .notify(PollMask::POLLIN | PollMask::POLLOUT | PollMask::POLLERR | PollMask::POLLHUP);
    }
}

/// 后台工作队列回调入口：还原 `UdpEndpoint` 引用并推进数据报收发。
fn run_udp(work: Pin<&Work>) {
    // SAFETY:
    // 1. `work.context()` 在 `UdpEndpoint::try_new` 中由 `Arc::as_ptr(&endpoint) as usize` 初始化，指向堆上地址稳定的 `UdpEndpoint`；
    // 2. 端点发布后由全局 UDP 表、异步退休容器或调用 `stop_sync` 的局部 `Arc` 强引用保活；
    // 3. 在最后一个 `Arc` 强引用释放前，`stop_sync` 必定先调用 `work.close_sync()` 等待本回调彻底执行完毕并退出，因此解引用在回调生命周期内始终有效。
    let endpoint = unsafe { &*(work.context() as *const UdpEndpoint) };
    endpoint.run();
}
