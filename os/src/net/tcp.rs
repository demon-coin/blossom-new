//! 每个 TCP 端点独立拥有核心、输入和期限；表与退休拥有覆盖回调的完整寿命。

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

/// 输入在用户持有核心时保持拥有，立即回应未接受时也留在同一节点。
struct Input {
    closed: bool,
    // 真实 Work 正在尝试或等待操作拥有，释放者移交此责任；它不代表另一个 pending 票。
    owner_waiting: bool,
    packets: VecDeque<OwnedPacket>,
    reply: Option<OwnedPacket>,
}

/// 绑定和 listener 关联只保存真实拥有，不复制 raw 协议状态。
struct Metadata {
    lease: Option<Arc<BindLease>>,
    parent: Weak<TcpEndpoint>,
    engaged: bool,
    error: Option<FsError>,
    connected: bool,
    read_closed: bool,
    write_closed: bool,
    reuse: bool,
    v6_only: bool,
    timeouts: SocketTimeouts,
}

/// 半连接与可接受连接共享实际 backlog 上限，只有成熟 Established 才进入 ready。
struct Children {
    limit: usize,
    pending: Vec<Arc<TcpEndpoint>>,
    ready: VecDeque<Arc<TcpEndpoint>>,
}

/// 表强持稳定节点，节点弱指网络域和 listener；普通 Work 固定轮转 CPU。
pub struct TcpEndpoint {
    id: u64,
    family: AddressFamily,
    stack: Weak<NetStack>,
    core: SleepMutex<TcpCore>,
    input: SpinLock<Input>,
    metadata: SpinLock<Metadata>,
    children: SpinLock<Children>,
    source: Arc<PollWaitHead>,
    egress_wait: Arc<EgressWait>,
    pub(crate) retirement: RetirementLink,
    work: Once<Pin<Arc<Work>>>,
    timer: Once<Pin<Arc<Timer>>>,
    stopped: AtomicBool,
    _slot: EndpointSlot,
}

/// 修改操作主动安排协议；只读观察只移交已经受阻的真实 Work。
#[derive(Clone, Copy, Eq, PartialEq)]
enum OperationMode {
    Modify,
    Observe,
}

/// 一次拥有期的原事实快照只用于识别通知，不保存另一份协议状态。
struct Observation {
    received: usize,
    sent: usize,
    mask: PollMask,
    ready_children: usize,
}

impl Observation {
    /// 从本次拥有前后的事实提取新就绪方向，队列消费不产生数据到达通知。
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

/// 普通操作保护只在释放核心后安排同一个协议 Work。
pub struct TcpOperationGuard<'a> {
    endpoint: &'a TcpEndpoint,
    core: Option<SleepMutexGuard<'a, TcpCore>>,
    mode: OperationMode,
    initial: Observation,
    initial_trace_state: Option<State>,
}

impl Deref for TcpOperationGuard<'_> {
    type Target = TcpCore;
    /// 借用本次真正持有的核心保护，不创建第二个协议对象。
    fn deref(&self) -> &TcpCore {
        self.core.as_deref().unwrap()
    }
}

impl DerefMut for TcpOperationGuard<'_> {
    /// 可变借用仅在本连接 SleepMutex 的操作拥有期间存在。
    fn deref_mut(&mut self) -> &mut TcpCore {
        self.core.as_deref_mut().unwrap()
    }
}

impl Drop for TcpOperationGuard<'_> {
    /// 核心保护内采实际修改后的状态，释放后记录变化并通知方向；只读保护仅移交已有 Work。
    fn drop(&mut self) {
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
        drop(self.core.take());
        if let Some(value) = state_change {
            trace::record(trace::Kind::TcpState, self.endpoint.id, value);
        }
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

impl TcpEndpoint {
    /// 在原域环境准备唯一核心和异步节点，Work 与 Timer 固定在独立轮转的同一 CPU。
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

    /// 两向等待值属于同一端点，查询不取得可睡协议操作权。
    pub(crate) fn timeouts(&self) -> SocketTimeouts {
        self.metadata.lock().timeouts
    }

    /// 控制属性只在 metadata 短锁内修改，监听子对象独立持有握手时的快照。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        self.metadata.lock().timeouts.set(receive, timeout);
    }

    /// 返回本域不复用的身份，描述符编号不进入协议索引。
    pub fn id(&self) -> u64 {
        self.id
    }

    /// 等待登记只借创建期准备的轻量节点，不延长协议对象寿命。
    pub(crate) fn egress_wait(&self) -> &Arc<EgressWait> {
        &self.egress_wait
    }

    /// 测试只观察当前真实租约身份，不取得会延长端口占用的强引用。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_binding_identity(&self) -> Option<u64> {
        self.metadata.lock().lease.as_ref().map(|lease| lease.id())
    }

    /// 返回普通 Work 和精确 Timer 共同固定的执行 CPU。
    #[cfg(feature = "ktest")]
    pub fn work_cpu(&self) -> usize {
        self.work.get().unwrap().cpu()
    }

    /// 只比较弱域与真实 Arc 的身份，不延长域的服务寿命。
    pub fn belongs_to(&self, net: &NetStackRef) -> bool {
        self.stack.as_ptr() == Arc::as_ptr(net)
    }

    /// 普通任务取得本连接拥有，结束后由 guard 安排积存输入和发送责任。
    pub fn lock(&self) -> TcpOperationGuard<'_> {
        self.operation(self.core.lock(), OperationMode::Modify)
    }

    /// 只观察成熟状态；释放时仅移交已经因本操作拥有受阻的真实 Work，不主动开始协议进度。
    pub fn state(&self) -> State {
        self.observe().state()
    }

    /// 普通只读访问也参与操作拥有交接，未有真实 Work 受阻时不安排后台票。
    pub(crate) fn observe(&self) -> TcpOperationGuard<'_> {
        self.operation(self.core.lock(), OperationMode::Observe)
    }

    /// 本保护的前后事实用于通知；仅采集开启时保存真实修改操作的起始协议状态。
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

    /// 就绪位只来自本核心、关闭事实与 child 队列；读取不安排协议处理。
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

    /// 读取原收发队列、完整就绪位与 ready child 数量，快照只存活于本次拥有期。
    fn observation(&self, core: &TcpCore) -> Observation {
        Observation {
            received: core.receive_queue(),
            sent: core.send_queue(),
            mask: self.ready_mask(core),
            ready_children: self.children.lock().ready.len(),
        }
    }

    /// 曾建立连接只在成熟核心确认握手后记录，正常关闭不删除这一 ABI 事实。
    fn record_connected(&self, core: &TcpCore) {
        if matches!(core.state(), State::Established | State::CloseWait) {
            self.metadata.lock().connected = true;
        }
    }

    /// 先登记唯一持续源再观察，不从查询路径收包、dispatch 或主动通知。
    pub fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&self.source)?;
        }
        let core = self.observe();
        Ok(self.ready_mask(&core))
    }

    /// 返回同一次真实握手的结果，错误查询不消费 SO_ERROR 或重新发表 tuple。
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

    /// 同一连接拥有覆盖暂存、copyout 和消费；已有数据先于错误及 EOF 交付。
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

    /// 先查询关闭和错误事实，再由原核心核发送许可及真正接受量，零请求也保留 EPIPE。
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

    /// 关闭指定方向；发送关闭沿 raw FIN，接收方向仍能在 SHUT_WR 后独立推进。
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

    /// 已连接返回实际路由源，绑定保原通配，未绑定返回零地址及零端口。
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

    /// 只返回真正建立连接且 raw 仍拥有的远端端点。
    pub fn peer_address(&self) -> Result<InetEndpoint, FsError> {
        let core = self.observe();
        if !self.metadata.lock().connected {
            return Err(FsError::NotConnected);
        }
        core.remote_endpoint().ok_or(FsError::NotConnected)
    }

    /// 在只读核心保护下消费一次 pending error，不主动通知或安排握手。
    pub fn take_error(&self) -> Option<FsError> {
        let core = self.observe();
        let error = self.metadata.lock().error.take();
        drop(core);
        error
    }

    /// 只观察原核心是否仍为 Listen。
    pub fn is_listener(&self) -> bool {
        self.state() == State::Listen
    }

    /// 复用选项属于这个打开端点，accepted child 不读取 parent 的后来修改。
    pub fn reuse_address(&self) -> bool {
        self.metadata.lock().reuse
    }

    /// 本核心串行控制选项；child 共享租约时只改自己的事实，不修改 parent 绑定。
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

    /// 只读原 raw Nagle 标志，不驱动发送或产生就绪通知。
    pub fn nodelay(&self) -> Result<bool, FsError> {
        Ok(!self.observe().nagle_enabled())
    }

    /// 修改真实 raw 标志，保护释放后由既有 Work 处理受策略影响的发送。
    pub fn set_nodelay(&self, enabled: bool) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_nagle_enabled(!enabled);
        Ok(())
    }

    /// 观察本连接原接收 ring 容量，listener child 使用相同真实容量。
    pub fn receive_capacity(&self) -> usize {
        self.observe().receive_capacity()
    }

    /// 观察本连接原发送 ring 容量，不给已接受量增加用户准备前缀。
    pub fn send_capacity(&self) -> usize {
        self.observe().send_capacity()
    }

    /// 端点族由创建时确定，mapped child 不从实际报文族改写 OFD 身份。
    pub fn family(&self) -> AddressFamily {
        self.family
    }

    /// 与绑定及 raw 配置共用本连接的操作拥有，观察不驱动协议。
    pub fn v6_only(&self) -> bool {
        let _core = self.observe();
        self.metadata.lock().v6_only
    }

    /// 只允许未绑定的 IPv6 端点修改双族策略。
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

    /// 查询原核心的探测配置，不保存第二份 KEEPALIVE 开关。
    pub fn keepalive(&self) -> bool {
        self.observe().keepalive()
    }

    /// 释放修改保护后沿原 Work 和方向通知安排探测配置变化。
    pub fn set_keepalive(&self, enabled: bool) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_keepalive(enabled);
        Ok(())
    }

    /// 返回真实接收接纳预算，独立于 child 所需的物理后备容量。
    pub fn receive_budget(&self) -> usize {
        self.observe().receive_budget()
    }

    /// 返回 raw 未 ACK 字节共同占用的发送接纳预算。
    pub fn send_budget(&self) -> usize {
        self.observe().send_budget()
    }

    /// 在同一 raw 对象上调节窗口预算，原数据和旧信用保持拥有。
    pub fn set_receive_budget(&self, budget: usize) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_receive_budget(budget)
    }

    /// 在同一 raw 对象上调节发送预算，真实恢复的 OUT 在核心解锁后通知。
    pub fn set_send_budget(&self, budget: usize) -> Result<(), FsError> {
        let mut core = self.lock();
        let net = self.net()?;
        let _admission = net.admission()?;
        core.set_send_budget(budget)
    }

    /// 使用原分段决策查询 MSS，已连接 tuple 由 raw 选择实际协议族。
    pub fn effective_mss(&self) -> usize {
        self.observe().effective_mss(self.family)
    }

    /// 返回该端点实际启用的原拥塞控制器。
    pub fn congestion_control(&self) -> CongestionControl {
        self.observe().congestion_control()
    }

    /// ABI 已完成 mapped 正规化，端点仅核对核心小值与本端点策略。
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

    /// 取得仍存在的网络域，停止后的操作不得再发表新索引。
    fn net(&self) -> Result<NetStackRef, FsError> {
        self.stack.upgrade().ok_or(FsError::NotConnected)
    }

    /// 同一操作拥有核对族及绑定策略，成功具体 IPv6 绑定同时固定 V6ONLY。
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

    /// 重复 listen 只更新有限 backlog，既有半连接及 ready 仍由原队列拥有。
    /// backlog 零保留一项正常队列容量；扩容准备不持 child 短锁。
    pub fn listen(self: &Arc<Self>, backlog: usize) -> Result<(), FsError> {
        let limit = backlog.clamp(1, SOMAXCONN);
        let mut core = self.lock();
        let listening = core.state() == State::Listen;
        if !listening && (core.state() != State::Closed || self.metadata.lock().engaged) {
            return Err(FsError::InvalidOperation);
        }
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

    /// 首次发表实际双族 tuple；路由选源及 MTU 更新发生在同一核心握手之前。
    pub fn connect(self: &Arc<Self>, peer: InetEndpoint) -> Result<(), FsError> {
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

    /// 调用者持本核心及控制准入，首次自动绑定保留本端点族的通配地址及双面责任。
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

    /// 只领取已完成握手的真实 child，不替换 listener 或提前创建用户描述符。
    pub fn accept(&self) -> Result<Option<Arc<TcpEndpoint>>, FsError> {
        let core = self.lock();
        if core.state() != State::Listen {
            return Err(FsError::InvalidOperation);
        }
        let child = self.children.lock().ready.pop_front();
        Ok(child)
    }

    /// 最后打开对象请求双方向关闭；FIN 和 TIME_WAIT 继续由原 Work/Timer 推进。
    pub fn close(&self) {
        let mut core = self.lock();
        core.close();
        let mut metadata = self.metadata.lock();
        metadata.engaged = true;
        metadata.read_closed = true;
        metadata.write_closed = true;
    }

    /// 未交给打开对象的节点终止协议，仍保留真实待发 RST 的 dispatch 责任。
    pub fn abandon(&self) {
        let mut core = self.lock();
        core.abort();
        self.metadata.lock().engaged = true;
    }

    /// 输入接收不等待用户核心，只在预留队列发表完整拥有，然后安排同 Work。
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

    /// 原 Work 的 pending 协议负责合并；停止时拒绝新增票，不维护第二 scheduled 位。
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

    /// 先登记真实拥有责任；核心保护内采协议前后状态，释放后记录变化并通知原持续源。
    fn run(&self) {
        let Some(net) = self.stack.upgrade() else {
            return;
        };
        if net.is_stopping() || self.stopped.load(Ordering::Acquire) {
            return;
        }
        self.input.lock().owner_waiting = true;
        let Some(mut core) = self.core.try_lock() else {
            return;
        };
        self.input.lock().owner_waiting = false;
        let previous = self.observation(&core);
        let initial_trace_state = trace::enabled().then(|| core.state());
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

    /// 每轮先交付旧应答，再处理有限输入；未接受应答禁止继续消费 backlog。
    fn advance(
        &self,
        net: &NetStackRef,
        core: &mut TcpCore,
        notification: &mut Option<Arc<TcpEndpoint>>,
    ) -> Result<(), FsError> {
        self.emit_reply(net)?;
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
                    // 被动 child 的原握手已失败，不能将其回到 Listen 后留在半连接队列。
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
        while 0 < remaining {
            remaining -= 1;
            core.dispatch(time::now(), |packet| net.send_ip(packet))?;
            if core.deadline(time::now()) != ProtocolDeadline::Now {
                break;
            }
        }
        Ok(())
    }

    /// try_emit 的失败完整包重新成为 pending，成功才结束立即应答责任。
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

    /// listener 操作拥有提供 raw 配置；metadata 同次快照在 child 发表前继承。
    fn receive_syn(
        &self,
        net: &NetStackRef,
        core: &TcpCore,
        packet: OwnedPacket,
    ) -> Result<(), FsError> {
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
        {
            let children = self.children.lock();
            if children.limit <= children.pending.len() + children.ready.len() {
                return Ok(());
            }
        }
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

    /// child 完成被动握手后转移同一 pending Arc，返回锁外通知所需的真实 parent 拥有。
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

    /// 退休者或根拥有局部 Arc 后才能调用，先停 Timer 再等 Work，不持核心或短锁等待。
    pub(crate) fn stop_sync(&self) {
        assert!(
            !self.stopped.swap(true, Ordering::AcqRel),
            "TCP endpoint stopped twice"
        );
        self.input.lock().closed = true;
        self.timer.get().unwrap().as_ref().close_sync();
        self.work.get().unwrap().as_ref().close_sync();
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
        let (packets, reply) = {
            let mut input = self.input.lock();
            (core::mem::take(&mut input.packets), input.reply.take())
        };
        // 已退休协议仍可由 OFD 查询绑定；租约仅随 endpoint 最后拥有归还。
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

    /// 撤下已结束的 child 只缩减队列，不产生新增读事件；最后 Arc 析构在锁外发生。
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

/// 只在采集开启的操作边界读取末状态，编码已观察到的变化，不补造中间协议状态。
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

/// 固定事件文本使用独立于库枚举布局的协议状态码。
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

/// 表、退休队列或停止者的局部 Arc 保活节点，Work 本身不猜测业务 Arc 控制块。
fn run_tcp(work: Pin<&Work>) {
    // SAFETY: context 来自构造期稳定 Arc；发表后真实表/退休/停止局部拥有一直覆盖 Work 全尾。
    let endpoint = unsafe { &*(work.context() as *const TcpEndpoint) };
    endpoint.run();
}

/// 精确期限在时间基准维护结束后到达，记录实际到期并安排同一普通 Work，不运行协议。
fn tcp_expired(timer: Pin<&Timer>) {
    // SAFETY: Timer context 来自稳定节点 Arc；退休与根停止均先等本 Timer 全尾再释放节点。
    let endpoint = unsafe { &*(timer.context() as *const TcpEndpoint) };
    trace::record(trace::Kind::ProtocolDeadline, endpoint.id, 0);
    endpoint.schedule();
}
