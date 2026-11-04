//! UDP 每个端点独立拥有原协议队列、操作权和后台工作；输入短锁只移动完整拥有。

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

/// 后台输入在普通操作持有协议对象时保留；预留槽位内的移动不分配。
struct Input {
    closed: bool,
    owner_waiting: bool,
    packets: VecDeque<OwnedPacket>,
}

/// 原 raw 队列唯一保存消息；发送旁表只延长每条记录对应的绑定租约。
struct State {
    socket: udp::Socket<'static>,
    buffers_ready: bool,
    receive_budget: usize,
    send_budget: usize,
    leases: VecDeque<Arc<BindLease>>,
    binding: Option<Arc<BindLease>>,
    peer: Option<InetEndpoint>,
    route_local: Option<InetEndpoint>,
    explicit_bind: bool,
    reuse: bool,
    v6_only: bool,
    read_closed: bool,
    write_closed: bool,
    closed: bool,
    error: Option<FsError>,
}

impl State {
    /// 持操作锁准备两向后备，全部成功后才安装；热 raw 永不替换。
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

    /// 冷对象尚无记录且配置预算非零，不能由零容量 raw 伪造发送背压。
    fn can_send(&self) -> bool {
        !self.buffers_ready || self.socket.can_send()
    }

    /// 从原队列和方向关闭事实计算就绪，不驱动 dispatch 或领取输入。
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

    /// 完整消息的零长度也须经过真实关闭检查，写关闭不能伪成功。
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

/// 表、退休容器和根停止者保活稳定节点，节点只弱持网络域。
pub struct UdpEndpoint {
    id: u64,
    family: AddressFamily,
    stack: Weak<NetStack>,
    state: SleepMutex<State>,
    input: SpinLock<Input>,
    timeouts: SpinLock<SocketTimeouts>,
    source: Arc<PollWaitHead>,
    work: Once<Pin<Arc<Work>>>,
    stopped: AtomicBool,
    pub(crate) retirement: RetirementLink,
    egress_wait: Arc<EgressWait>,
    _slot: EndpointSlot,
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum OperationMode {
    Modify,
    Observe,
}

/// 普通操作释放原对象后才通知；观察也必须交接已受阻的 Work。
struct OperationGuard<'a> {
    endpoint: &'a UdpEndpoint,
    state: Option<SleepMutexGuard<'a, State>>,
    mode: OperationMode,
    initial_mask: PollMask,
    initial_send_budget: usize,
    initial_read_closed: bool,
    initial_write_closed: bool,
    initial_closed: bool,
}

impl Deref for OperationGuard<'_> {
    type Target = State;

    /// 借用同一操作拥有保护内的协议对象。
    fn deref(&self) -> &State {
        self.state.as_deref().unwrap()
    }
}

impl DerefMut for OperationGuard<'_> {
    /// 修改只发生于端点自己的 SleepMutex 拥有期。
    fn deref_mut(&mut self) -> &mut State {
        self.state.as_deref_mut().unwrap()
    }
}

impl Drop for OperationGuard<'_> {
    /// 先释放操作锁，再交接短锁中登记的真实拥有责任和新增方向通知。
    fn drop(&mut self) {
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
        drop(self.state.take());
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
    /// 域先取得节点拥有，创建零后备 raw 并预备有界输入、租约旁表及等待节点。
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
        // UDP 不消耗 TCP 的协议 CPU 轮转序号；同一 Work 由原执行域调度。
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

    /// 查询端点共享的两向期限，不取得可能等待 raw 拥有者的操作锁。
    pub(crate) fn timeouts(&self) -> SocketTimeouts {
        *self.timeouts.lock()
    }

    /// 控制值只占短锁，设置不能阻塞协议 Work 或修改 raw 消息责任。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        self.timeouts.lock().set(receive, timeout);
    }

    /// metadata 和 payload 都在堆上预备，128KiB 后备不落任务栈。
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

    /// 普通操作共享同一释放协议，观察不得主动安排新的协议进度。
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

    /// 返回域内不复用的身份，关闭后外部拥有仍保留该身份。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 只观察原 raw 的真实后备，冷对象检查不得通过配置容量推断已分配字节。
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

    /// 根容量等待持同一轻量节点，不延长协议端点的业务寿命。
    pub(crate) fn egress_wait(&self) -> &Arc<EgressWait> {
        &self.egress_wait
    }

    /// OFD 地址族由构造固定，不从某条报文的协议族推测。
    pub fn family(&self) -> AddressFamily {
        self.family
    }

    /// 与 raw 和真实 binding 在同一操作状态观察 V6ONLY。
    pub fn v6_only(&self) -> bool {
        self.operation(OperationMode::Observe).v6_only
    }

    /// 仅未绑定的 IPv6 端点可修改 V6ONLY，旧在途记录不改变该边界。
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

    /// 查询本端发送关门，不取得根或接口短锁。
    pub fn check_send(&self) -> Result<(), FsError> {
        self.operation(OperationMode::Observe).check_send()
    }

    /// 已正规化的核心地址仍须符合端点族、V6ONLY 和实际操作的错误边界。
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

    /// 双面只属于 IPv6 :: 通配且 V6ONLY 关闭的同一租约。
    fn dual_stack(&self, state: &State, local: InetEndpoint) -> bool {
        self.family == AddressFamily::Ipv6
            && !state.v6_only
            && matches!(local.address, IpAddress::Ipv6(address) if address.is_unspecified())
    }

    /// 持操作权先准备两向后备再取得显式绑定，OOM 不发表地址或租约。
    pub fn bind(self: &Arc<Self>, requested: InetEndpoint) -> Result<InetEndpoint, FsError> {
        let mut state = self.operation(OperationMode::Modify);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        if state.binding.is_some() {
            return Err(FsError::InvalidOperation);
        }
        self.check_family(&state, requested.address, AddressOperation::Bind)?;
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
        net.tables().publish_udp(self.id, local, None)?;
        state.socket.set_endpoint(local.raw_listen());
        if matches!(local.address, IpAddress::Ipv6(address) if !address.is_unspecified()) {
            state.v6_only = true;
        }
        state.binding = Some(lease);
        state.explicit_bind = true;
        Ok(local)
    }

    /// 自动绑定只保存一份真正通配租约，发送 metadata 单独保存选定源。
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

    /// 默认 peer 和每次目的独立；自动断开只撤当前租约，不清原消息队列。
    pub fn connect(self: &Arc<Self>, peer: Option<InetEndpoint>) -> Result<(), FsError> {
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
                // 已接纳 TX 的旁表和 Root owned 包各持旧 lease，最后 put 只撤己 lease_id。
                state.binding.take();
            }
            state.peer = None;
            state.route_local = None;
            return Ok(());
        };
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
        net.tables().publish_udp(self.id, local, Some(peer))?;
        state.socket.set_endpoint(local.raw_listen());
        state.binding = Some(lease);
        state.route_local = Some(local);
        state.peer = Some(peer);
        Ok(())
    }

    /// 完整已复制消息只在原 TX record 接纳后报告成功；暂满不拆消息或提交旁表。
    pub fn send(
        self: &Arc<Self>,
        payload: &[u8],
        destination: Option<InetEndpoint>,
    ) -> Result<IoProgress, FsError> {
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
        // 准入只保护绑定发表和关门；完整 payload 复制不得占用全域操作权。
        drop(_admission);
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

    /// 输入仅发表有界 owned backlog，满时丢未接纳消息，不向 sender 提供可靠回压。
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

    /// peek 与 recv 在同一操作权下直接借原 record；用户复制不持表或接口短锁。
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
            // 原 recv 先消费整条，短复制和部分 fault 都不会把剩余尾部留给后继接收者。
            let (payload, metadata) = state.socket.recv().expect("UDP readable record");
            self.copy_datagram(target, payload, InetEndpoint::from_raw(metadata.endpoint))
                .map(Some)
        }
    }

    /// UDP 的部分 fault 直接传播；成功的短复制保原长和同条 source。
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

    /// 观察真正租约地址，自动断开后不从旧在途包补回当前绑定。
    pub fn local(&self) -> Option<InetEndpoint> {
        self.operation(OperationMode::Observe)
            .binding
            .as_ref()
            .map(|lease| lease.local())
    }

    /// 显式或自动绑定的复用策略只修改当前 lease_id。
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

    /// 观察当前真实复用策略，不修改租约或协议索引。
    pub fn reuse_address(&self) -> bool {
        self.operation(OperationMode::Observe).reuse
    }

    /// 弱域身份判断不延长网络服务寿命。
    pub fn belongs_to(&self, net: &NetStackRef) -> bool {
        Weak::ptr_eq(&self.stack, &Arc::downgrade(net))
    }

    /// connected 名称使用实际路由源，其余保持当前租约的真实通配值。
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

    /// 单次 sendto 的目的从不替换 connected 默认 peer 名称。
    pub fn peer_address(&self) -> Result<InetEndpoint, FsError> {
        let state = self.operation(OperationMode::Observe);
        if state.closed {
            return Err(FsError::NotConnected);
        }
        state.peer.ok_or(FsError::NotConnected)
    }

    /// 返回配置的 RX 预算，冷对象保持与激活后相同的用户可见值。
    pub fn receive_budget(&self) -> usize {
        self.operation(OperationMode::Observe).receive_budget
    }

    /// 返回配置的 TX 预算，不把冷 raw 的零容量作为 SO_SNDBUF。
    pub fn send_budget(&self) -> usize {
        self.operation(OperationMode::Observe).send_budget
    }

    /// 冷对象只修改配置；热队列缩预算保留旧记录，提高后安排原 backlog。
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

    /// 冷对象保存 TX 配置，热对象只改原队列预算，OUT 通知仍在锁外发生。
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

    /// 返回固定配置的 RX 后备容量，冷对象也允许完整范围内的早期预算。
    pub fn receive_capacity(&self) -> usize {
        PAYLOAD_CAPACITY
    }

    /// 返回固定配置的 TX 后备容量，与是否已安装 payload 无关。
    pub fn send_capacity(&self) -> usize {
        PAYLOAD_CAPACITY
    }

    /// 一次消费真正后台 dispatch 错误，不合成 ICMP 或重复回显。
    pub fn take_error(&self) -> Option<FsError> {
        self.operation(OperationMode::Modify).error.take()
    }

    /// 未连接也先提交方向关闭；已有原 RX record 先于读 EOF。
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

    /// 登记持续源后只 Observe 原 raw 事实，不 pump、查路由或登记 Root 容量等待。
    pub fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&self.source)?;
        }
        Ok(self.operation(OperationMode::Observe).mask())
    }

    /// Root 容量恢复只安排原 Work，释放 TX record 后才通知发送者。
    pub fn notify_egress(&self) {
        self.schedule();
    }

    /// 原 Work 合并 pending 票，根停止或本端点全尾后拒绝新增工作。
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

    /// 后台先登记拥有责任；末 OFD 后继续提交或终结已接纳 TX，暂满不自旋。
    fn run(&self) {
        let Some(net) = self.stack.upgrade() else {
            return;
        };
        if net.is_stopping() || self.stopped.load(Ordering::Acquire) {
            return;
        }
        self.input.lock().owner_waiting = true;
        let Some(mut state) = self.state.try_lock() else {
            return;
        };
        self.input.lock().owner_waiting = false;
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
        let pending_tx = !state.leases.is_empty();
        let closed = state.closed;
        let ended = closed && !pending_tx;
        let binding = if ended {
            // raw TX 已全部交给 Root 或以终结错误结束；已交包的后续 frame/DMA 仍持真实拥有。
            state.socket.close();
            state.binding.take()
        } else {
            None
        };
        drop(state);
        if !notification.is_empty() {
            self.source.notify(notification);
        }
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

    /// 原 RX record 的真实接纳才通知 IN；当前 peer、族和关闭事实再次过滤旧查找。
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

    /// 队首 raw record 与 lease 同时结束；WouldBlock 保留二者，终结错误释放本条责任。
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
                    // Ok 只提交本条 raw 消费；真实发送结果另存，错误不能冒充 Root 接纳。
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

    /// 末 OFD 只提交关闭并安排异步退休，不在本端点 Work 上 join 自己。
    pub fn close(&self) {
        let mut state = self.operation(OperationMode::Modify);
        state.closed = true;
        state.read_closed = true;
        state.write_closed = true;
        state.peer = None;
        state.route_local = None;
        self.input.lock().closed = true;
    }

    /// reaper 或根停止局部拥有覆盖 Work 全尾，关门后再清原队列及 lease。
    pub(crate) fn stop_sync(&self) {
        assert!(
            !self.stopped.swap(true, Ordering::AcqRel),
            "UDP endpoint stopped twice"
        );
        self.input.lock().closed = true;
        self.work
            .get()
            .expect("UDP work is not prepared")
            .as_ref()
            .close_sync();
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
        if let Some(net) = self.stack.upgrade() {
            net.cancel_egress_wait(self.id);
            net.tables().unpublish_udp(self.id);
        }
        drop((binding, packets));
        self.source
            .notify(PollMask::POLLIN | PollMask::POLLOUT | PollMask::POLLERR | PollMask::POLLHUP);
    }
}

/// 表或退休容器的真实拥有覆盖回调到返回的全部节点访问。
fn run_udp(work: Pin<&Work>) {
    // SAFETY: context 来自稳定 Arc，发表后表、退休容器或根停止的局部 Arc 持续保活；
    // 最后拥有释放前先同步关闭本 Work，回调不延长裸指针的寿命。
    let endpoint = unsafe { &*(work.context() as *const UdpEndpoint) };
    endpoint.run();
}
