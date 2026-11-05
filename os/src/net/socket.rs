//! 打开对象持有具体协议端点；流前缀与完整报文分别在真实接受点提交。

use alloc::sync::Arc;
use core::ops::Range;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::fs::epoll::WatchHead;
use crate::fs::poll::{PollContext, PollMask, PollNotify, PollTable, Pollable};
use crate::fs::{DirectoryEntry, FileStat, FsError, InodeKind, InodeRef, InodeTime, Path};
use crate::fs::{
    FileIoContext, FileLike, ReadBuffer, SeekFrom, WriteBuffer, O_CLOEXEC, O_NONBLOCK, O_RDWR,
    S_IFSOCK, S_IRWXUGO,
};
use crate::net::address::{AddressFamily, AddressOperation, InetEndpoint, SocketAddress, AF_UNIX};
use crate::net::packet::{zeroed_bytes, MAX_IPV4_UDP_PAYLOAD_LEN};
use crate::net::tcp::TcpEndpoint;
use crate::net::udp::UdpEndpoint;
use crate::net::unix::UnixEndpoint;
use crate::net::{IoProgress, MessageResult, NetStackRef, UnixSocketType};
use crate::task::signal::SIGPIPE;
use crate::time::{self, MonotonicInstant};
use crate::wait::WaitResult;
use core::time::Duration;
use smoltcp::socket::tcp::CongestionControl;

pub const TCP_BUFFER_LEN: usize = 128 * 1024;

pub(crate) const SOCK_STREAM: u32 = 1;
pub(crate) const SOCK_DGRAM: u32 = 2;
pub(crate) const SOCK_SEQPACKET: u32 = 5;
pub(crate) const SOCK_TYPE_MASK: u32 = 0xf;
pub(crate) const SOCK_MAX: u32 = 11;
pub(crate) const SOCK_NONBLOCK: u32 = O_NONBLOCK;
pub(crate) const SOCK_CLOEXEC: u32 = O_CLOEXEC;

pub(crate) const SHUT_RD: u32 = 0;
pub(crate) const SHUT_WR: u32 = 1;
pub(crate) const SHUT_RDWR: u32 = 2;

pub(crate) const IPPROTO_TCP: i32 = 6;
pub(crate) const IPPROTO_UDP: i32 = 17;
pub(crate) const IPPROTO_IPV6: i32 = 41;
pub(crate) const IPPROTO_MAX: i32 = 263;

pub(crate) const SOL_SOCKET: i32 = 1;
pub(crate) const SO_REUSEADDR: i32 = 2;
pub(crate) const SO_TYPE: i32 = 3;
pub(crate) const SO_ERROR: i32 = 4;
pub(crate) const SO_KEEPALIVE: i32 = 9;
pub(crate) const SO_SNDBUF: i32 = 7;
pub(crate) const SO_RCVBUF: i32 = 8;
pub(crate) const SO_RCVTIMEO: i32 = 20;
pub(crate) const SO_SNDTIMEO: i32 = 21;
pub(crate) const SO_ACCEPTCONN: i32 = 30;
pub(crate) const SO_PROTOCOL: i32 = 38;
pub(crate) const SO_DOMAIN: i32 = 39;

pub(crate) const MSG_PEEK: u32 = 0x2;
pub(crate) const MSG_TRUNC: u32 = 0x20;
pub(crate) const MSG_DONTWAIT: u32 = 0x40;
pub(crate) const MSG_NOSIGNAL: u32 = 0x4000;

pub(crate) const STREAM_PREFIX_LEN: usize = 4096;
const BUFFER_HINT_MAX: u32 = 65536;
const BUFFER_BUDGET_MIN: usize = 1024;
const BUFFER_BUDGET_MULTIPLIER: usize = 2;

/// 端点的两向等待配置使用纳秒；无限与立即返回分别由 None 和 Some(0) 表示。
#[derive(Clone, Copy, Default)]
pub(crate) struct SocketTimeouts {
    pub(crate) receive: Option<u64>,
    pub(crate) send: Option<u64>,
}

impl SocketTimeouts {
    /// 在端点短锁内只替换所选方向，另一向保持同一对象的原配置。
    pub(crate) fn set(&mut self, receive: bool, timeout: Option<u64>) {
        if receive {
            self.receive = timeout;
        } else {
            self.send = timeout;
        }
    }
}

/// 一个打开实例只选择一种真实协议端点，不引入另一层 socket 表。
#[derive(Clone)]
enum Endpoint {
    Tcp(Arc<TcpEndpoint>),
    Udp(Arc<UdpEndpoint>),
    Unix(Arc<UnixEndpoint>),
}

impl Endpoint {
    /// 构造失败时撤销未发表端点，TCP 保留原协议终止责任。
    fn abandon(&self) {
        match self {
            Self::Tcp(endpoint) => endpoint.abandon(),
            Self::Udp(endpoint) => endpoint.close(),
            Self::Unix(endpoint) => endpoint.close(),
        }
    }

    /// 末 OFD 请求正常关闭，不同步等待对端或协议 Work。
    fn close(&self) {
        match self {
            Self::Tcp(endpoint) => endpoint.close(),
            Self::Udp(endpoint) => endpoint.close(),
            Self::Unix(endpoint) => endpoint.close(),
        }
    }
}

/// 打开对象持稳定协议端点、网络域和创建者凭据，并保存共享 OFD 状态位。
/// 构造只准备打开实例，不发表用户 fd。
pub struct SocketFile {
    endpoint: Endpoint,
    net: NetStackRef,
    uid: u32,
    gid: u32,
    flags: AtomicU32,
    watches: Arc<WatchHead>,
    /// 阻塞 connect 尚未领取完成时保留该事实，信号重启仍消费原握手结果。
    connect_unclaimed: AtomicBool,
    /// 非 TCP 的通用开关只属于 OFD，不产生协议探测。
    generic_keepalive: AtomicBool,
    opened: bool,
}

impl SocketFile {
    /// 从同一网络域准备默认有界 TCP 核心与新的打开实例，CLOEXEC 留给描述符槽。
    pub fn try_new_tcp(
        net: NetStackRef,
        family: AddressFamily,
        flags: u32,
        uid: u32,
        gid: u32,
    ) -> Result<Arc<Self>, FsError> {
        if flags & !(O_NONBLOCK | O_CLOEXEC) != 0 {
            return Err(FsError::InvalidOperation);
        }
        let endpoint = net.try_new_tcp(family, TCP_BUFFER_LEN, TCP_BUFFER_LEN)?;
        Self::allocate(net, Endpoint::Tcp(endpoint), flags, uid, gid)
    }

    /// 从本域准备 UDP 打开实例，零长消息仍由真实报文端点提交。
    pub fn try_new_udp(
        net: NetStackRef,
        family: AddressFamily,
        flags: u32,
        uid: u32,
        gid: u32,
    ) -> Result<Arc<Self>, FsError> {
        if flags & !(O_NONBLOCK | O_CLOEXEC) != 0 {
            return Err(FsError::InvalidOperation);
        }
        let endpoint = net.try_new_udp(family)?;
        Self::allocate(net, Endpoint::Udp(endpoint), flags, uid, gid)
    }

    /// 完整准备真实匿名 pair 的两个 OFD，任一分配失败关闭尚未发表的两侧。
    pub fn try_new_unix_pair(
        net: NetStackRef,
        kind: UnixSocketType,
        flags: u32,
        uid: u32,
        gid: u32,
    ) -> Result<[Arc<Self>; 2], FsError> {
        if flags & !(O_NONBLOCK | O_CLOEXEC) != 0 {
            return Err(FsError::InvalidOperation);
        }
        let [first, second] = net.try_new_unix_pair(kind)?;
        let first = match Self::allocate(net.clone(), Endpoint::Unix(first), flags, uid, gid) {
            Ok(first) => first,
            Err(error) => {
                second.close();
                return Err(error);
            }
        };
        let second = Self::allocate(net, Endpoint::Unix(second), flags, uid, gid)?;
        Ok([first, second])
    }

    /// 为已拥有的本域协议端点构造独立 OFD，不搬走 raw 核心或替换稳定 listener。
    pub fn from_endpoint(
        net: NetStackRef,
        endpoint: Arc<TcpEndpoint>,
        flags: u32,
        uid: u32,
        gid: u32,
    ) -> Result<Arc<Self>, FsError> {
        if flags & !(O_NONBLOCK | O_CLOEXEC) != 0 {
            return Err(FsError::InvalidOperation);
        }
        if !endpoint.belongs_to(&net) {
            return Err(FsError::InvalidOperation);
        }

        Self::allocate(net, Endpoint::Tcp(endpoint), flags, uid, gid)
    }

    /// 分配失败撤回未交给打开实例的协议拥有，成功后才赋予末 OFD 关闭责任。
    fn allocate(
        net: NetStackRef,
        endpoint: Endpoint,
        flags: u32,
        uid: u32,
        gid: u32,
    ) -> Result<Arc<Self>, FsError> {
        let watches = match Arc::try_new(WatchHead::new()) {
            Ok(watches) => watches,
            Err(_) => {
                endpoint.abandon();
                return Err(FsError::NoMemory);
            }
        };

        let result = Arc::try_new(Self {
            endpoint: endpoint.clone(),
            net,
            uid,
            gid,
            flags: AtomicU32::new(O_RDWR | (flags & O_NONBLOCK)),
            watches,
            connect_unclaimed: AtomicBool::new(false),
            generic_keepalive: AtomicBool::new(false),
            opened: false,
        });
        match result {
            Ok(mut file) => {
                Arc::get_mut(&mut file)
                    .expect("unpublished socket file is shared")
                    .opened = true;
                Ok(file)
            }
            Err(_) => {
                endpoint.abandon();
                Err(FsError::NoMemory)
            }
        }
    }

    /// 仅向原 TCP 内核消费者借用稳定端点，其他协议不能被误当 TCP。
    pub fn tcp_endpoint(&self) -> Result<&Arc<TcpEndpoint>, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => Ok(endpoint),
            _ => Err(FsError::Unsupported),
        }
    }

    /// 返回本打开实例的真实流或报文类型。
    pub fn socket_type(&self) -> u32 {
        match &self.endpoint {
            Endpoint::Udp(_) => SOCK_DGRAM,
            Endpoint::Tcp(_) => SOCK_STREAM,
            Endpoint::Unix(endpoint) => endpoint.socket_type() as u32,
        }
    }

    /// UDP 与有序 UNIX 记录共享逐消息复制和截断语义。
    fn is_message_oriented(&self) -> bool {
        match &self.endpoint {
            Endpoint::Udp(_) => true,
            Endpoint::Unix(endpoint) => endpoint.socket_type() == UnixSocketType::SeqPacket,
            Endpoint::Tcp(_) => false,
        }
    }

    /// 返回实际地址域，匿名 pair 保持 AF_UNIX。
    pub fn domain(&self) -> u16 {
        match &self.endpoint {
            Endpoint::Unix(_) => AF_UNIX,
            Endpoint::Tcp(endpoint) => endpoint.family().number(),
            Endpoint::Udp(endpoint) => endpoint.family().number(),
        }
    }

    /// 查询同一端点的双面绑定策略。
    pub fn v6_only(&self) -> Result<bool, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => Ok(endpoint.v6_only()),
            Endpoint::Udp(endpoint) => Ok(endpoint.v6_only()),
            Endpoint::Unix(_) => Err(FsError::Unsupported),
        }
    }

    /// 操作锁中的实际绑定事实决定策略能否修改。
    pub fn set_v6_only(&self, enabled: bool) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_v6_only(enabled),
            Endpoint::Udp(endpoint) => endpoint.set_v6_only(enabled),
            Endpoint::Unix(_) => Err(FsError::Unsupported),
        }
    }

    /// 返回实际协议编号，匿名 UNIX stream 使用默认协议零。
    pub fn protocol(&self) -> i32 {
        match &self.endpoint {
            Endpoint::Tcp(_) => IPPROTO_TCP,
            Endpoint::Udp(_) => IPPROTO_UDP,
            Endpoint::Unix(_) => 0,
        }
    }

    /// 每个真实端点拥有独立且不复用的本域身份。
    pub fn id(&self) -> u64 {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.id(),
            Endpoint::Udp(endpoint) => endpoint.id(),
            Endpoint::Unix(endpoint) => endpoint.id(),
        }
    }

    /// 访问模式固定为双向流，非阻塞状态属于真正的 OFD。
    pub fn flags(&self) -> u32 {
        self.flags.load(Ordering::Acquire)
    }

    /// 同一原子读改写提交可变状态位，descriptor 的 CLOEXEC 不进入 OFD。
    pub fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        crate::fs::set_status_flags(&self.flags, raw)
    }

    /// 地址族、mapped 与 scope 在进入真实绑定前集中正规化。
    pub fn bind(&self, local: SocketAddress) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint
                .bind(local.normalize(
                    endpoint.family(),
                    endpoint.v6_only(),
                    AddressOperation::Bind,
                    false,
                )?)
                .map(|_| ()),
            Endpoint::Udp(endpoint) => endpoint
                .bind(local.normalize(
                    endpoint.family(),
                    endpoint.v6_only(),
                    AddressOperation::Bind,
                    true,
                )?)
                .map(|_| ()),
            Endpoint::Unix(_) => Err(FsError::Unsupported),
        }
    }

    /// 只有 TCP 能更新稳定 listener 的 backlog，其他端点明确拒绝监听。
    pub fn listen(&self, backlog: usize) -> Result<(), FsError> {
        self.tcp_endpoint()?.listen(backlog)
    }

    /// UDP 更新默认 peer 或断开，TCP 阻塞重启只领取既有握手的实际结果。
    /// 无进度才消费本次发送期限，中断或到期保留原握手的未领取事实。
    pub fn connect(&self, peer: SocketAddress, caller: FileIoContext<'_>) -> Result<(), FsError> {
        let endpoint = match &self.endpoint {
            Endpoint::Udp(endpoint) => {
                let peer = if peer == SocketAddress::Unspecified {
                    None
                } else {
                    Some(peer.normalize(
                        endpoint.family(),
                        endpoint.v6_only(),
                        AddressOperation::Connect,
                        true,
                    )?)
                };
                return endpoint.connect(peer);
            }
            Endpoint::Tcp(endpoint) => endpoint,
            Endpoint::Unix(_) => return Err(FsError::Unsupported),
        };

        let deadline = self.deadline(false);
        let peer = peer.normalize(
            endpoint.family(),
            endpoint.v6_only(),
            AddressOperation::Connect,
            false,
        )?;
        let nonblock = self.flags() & O_NONBLOCK != 0;
        match endpoint.connect(peer) {
            Ok(()) if nonblock => return Err(FsError::InProgress),
            Ok(()) => self.connect_unclaimed.store(true, Ordering::Release),
            Err(FsError::AlreadyInProgress) if !nonblock => {
                self.connect_unclaimed.store(true, Ordering::Release);
            }
            Err(FsError::AlreadyConnected)
                if !nonblock && self.connect_unclaimed.load(Ordering::Acquire) => {}
            Err(error) => return Err(error),
        }

        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            match endpoint.connection_result() {
                Ok(Some(())) => {
                    self.connect_unclaimed.store(false, Ordering::Release);
                    return Ok(());
                }
                Err(error) => {
                    self.connect_unclaimed.store(false, Ordering::Release);
                    return Err(error);
                }
                Ok(None) => {}
            }

            let deadline = Self::check_wait(deadline, interrupted, FsError::InProgress)?;
            if let Some((context, _)) = &waiting {
                interrupted = context
                    .park_until(matches!(caller, FileIoContext::User(_)), deadline)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(match self.prepare_wait() {
                    Ok(waiting) => waiting,
                    Err(error) => {
                        return Err(error);
                    }
                });
            }
        }
    }

    /// 从稳定 listener 领取成熟 child，为它准备新 OFD；新状态位只来自 accept4。
    /// 同一次接收期限覆盖所有通知后的重查，成熟连接优先于到期或信号。
    pub fn accept(
        &self,
        flags: u32,
        uid: u32,
        gid: u32,
        caller: FileIoContext<'_>,
    ) -> Result<Arc<SocketFile>, FsError> {
        if flags & !(O_NONBLOCK | O_CLOEXEC) != 0 {
            return Err(FsError::InvalidOperation);
        }

        let endpoint = self.tcp_endpoint()?;
        let deadline = self.deadline(true);
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            if let Some(child) = endpoint.accept()? {
                return Self::from_endpoint(self.net.clone(), child, flags, uid, gid);
            }
            if self.flags() & O_NONBLOCK != 0 {
                return Err(FsError::WouldBlock);
            }

            let deadline = Self::check_wait(deadline, interrupted, FsError::WouldBlock)?;
            if let Some((context, _)) = &waiting {
                interrupted = context
                    .park_until(matches!(caller, FileIoContext::User(_)), deadline)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 流交付连续前缀，消息端点领取同条记录及元数据，支持 PEEK 和 TRUNC。
    /// 每次入口固定接收期限，实际交付和非阻塞结果先于等待控制结果。
    pub fn receive(
        &self,
        target: &mut ReadBuffer<'_>,
        flags: u32,
        caller: FileIoContext<'_>,
    ) -> Result<MessageResult, FsError> {
        let allowed = MSG_DONTWAIT
            | MSG_PEEK
            | if self.is_message_oriented() {
                MSG_TRUNC
            } else {
                0
            };
        if flags & !allowed != 0 {
            return Err(FsError::Unsupported);
        }
        if target.copied() == target.len() && !self.is_message_oriented() {
            return Ok(Self::stream_result(0));
        }

        let deadline = self.deadline(true);
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            if let Some(result) = self.receive_once(target, flags & MSG_PEEK != 0)? {
                return Ok(result);
            }
            if self.nonblocking(flags) {
                return Err(FsError::WouldBlock);
            }

            let deadline = Self::check_wait(deadline, interrupted, FsError::WouldBlock)?;
            if let Some((context, _)) = &waiting {
                interrupted = context
                    .park_until(matches!(caller, FileIoContext::User(_)), deadline)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 单次数据面查询保留流的正前缀，消息复制 fault 不转换成部分记录成功。
    fn receive_once(
        &self,
        target: &mut ReadBuffer<'_>,
        peek: bool,
    ) -> Result<Option<MessageResult>, FsError> {
        let start = target.copied();
        let result = match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.receive(target, peek),
            Endpoint::Unix(endpoint) if endpoint.socket_type() == UnixSocketType::SeqPacket => {
                return endpoint.receive_record(target, peek);
            }
            Endpoint::Unix(endpoint) => endpoint.receive(target, peek),
            Endpoint::Udp(endpoint) => return endpoint.receive(target, peek),
        };
        match result {
            Ok(IoProgress::Ready(count)) => Ok(Some(Self::stream_result(count))),
            Ok(IoProgress::Eof) => Ok(Some(Self::stream_result(0))),
            Ok(IoProgress::Blocked) => Ok(None),
            Err(error) => Self::progress(target.copied() - start, error)
                .map(Self::stream_result)
                .map(Some),
        }
    }

    /// 锁外准备同一流前缀或完整消息，返回协议实际接受量。
    /// 两种发送共用入口时的期限快照，消息准备和就绪唤醒都不重新计时。
    pub fn send(
        &self,
        source: &mut WriteBuffer<'_>,
        destination: Option<SocketAddress>,
        flags: u32,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if flags & !(MSG_DONTWAIT | MSG_NOSIGNAL) != 0 {
            return Err(FsError::Unsupported);
        }
        let deadline = self.deadline(false);
        match &self.endpoint {
            Endpoint::Udp(endpoint) => {
                let destination = destination
                    .map(|peer| {
                        peer.normalize(
                            endpoint.family(),
                            endpoint.v6_only(),
                            AddressOperation::Send,
                            true,
                        )
                    })
                    .transpose()?;
                return self.send_datagram(endpoint, source, destination, flags, caller, deadline);
            }
            Endpoint::Unix(_) if destination.is_some() => return Err(FsError::AlreadyConnected),
            Endpoint::Unix(endpoint) if endpoint.socket_type() == UnixSocketType::SeqPacket => {
                return self.send_record(endpoint, source, flags, caller, deadline);
            }
            Endpoint::Tcp(_) | Endpoint::Unix(_) => {}
        }

        let mut prefix = [0u8; STREAM_PREFIX_LEN];
        let prepared = source.pull(&mut prefix)?;
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;

        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            // 空前缀仍交给端点核对真实发送关闭，不能在 OFD 层吞掉 EPIPE。
            let result = match &self.endpoint {
                Endpoint::Tcp(endpoint) => {
                    endpoint.send(&mut WriteBuffer::kernel(&prefix[..prepared]))
                }
                Endpoint::Unix(endpoint) => endpoint.send(&prefix[..prepared]),
                Endpoint::Udp(_) => unreachable!("datagram send uses complete packets"),
            };
            match result {
                Ok(IoProgress::Ready(count)) => return Ok(count),
                Ok(IoProgress::Blocked) => {}
                Ok(IoProgress::Eof) => unreachable!("stream send returned receive EOF"),
                Err(FsError::BrokenPipe) => {
                    if flags & MSG_NOSIGNAL == 0 {
                        if let FileIoContext::User(user) = caller {
                            user.send_kernel_signal(SIGPIPE);
                        }
                    }
                    return Err(FsError::BrokenPipe);
                }
                Err(error) => return Err(error),
            }
            if self.nonblocking(flags) {
                return Err(FsError::WouldBlock);
            }

            let deadline = Self::check_wait(deadline, interrupted, FsError::WouldBlock)?;
            if let Some((context, _)) = &waiting {
                interrupted = context
                    .park_until(matches!(caller, FileIoContext::User(_)), deadline)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 完整 copyin 后保留同份消息和入口期限，等待期间只重试真实 raw 接纳。
    fn send_datagram(
        &self,
        endpoint: &Arc<UdpEndpoint>,
        source: &mut WriteBuffer<'_>,
        destination: Option<InetEndpoint>,
        flags: u32,
        caller: FileIoContext<'_>,
        deadline: Result<Option<MonotonicInstant>, FsError>,
    ) -> Result<usize, FsError> {
        let length = source.len() - source.copied();
        if MAX_IPV4_UDP_PAYLOAD_LEN < length {
            return Err(FsError::MessageTooLarge);
        }

        let mut payload = zeroed_bytes(length)?;
        let prepared = source.pull(&mut payload)?;
        if prepared < length {
            // WriteBuffer 保留首个 copyin 错误；UDP 不提交已经取得的部分输入。
            source.pull(&mut [])?;
            unreachable!("incomplete datagram copy has no stored error");
        }

        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            match endpoint.send(&payload, destination)? {
                IoProgress::Ready(count) => return Ok(count),
                IoProgress::Blocked => {}
                IoProgress::Eof => unreachable!("datagram send returned receive EOF"),
            }
            if self.nonblocking(flags) {
                return Err(FsError::WouldBlock);
            }

            let deadline = Self::check_wait(deadline, interrupted, FsError::WouldBlock)?;
            if let Some((context, _)) = &waiting {
                interrupted = context
                    .park_until(matches!(caller, FileIoContext::User(_)), deadline)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 完整 copyin 一次，等待期间保留同份 Vec，只有方向真正接纳才移交其拥有。
    fn send_record(
        &self,
        endpoint: &Arc<UnixEndpoint>,
        source: &mut WriteBuffer<'_>,
        flags: u32,
        caller: FileIoContext<'_>,
        deadline: Result<Option<MonotonicInstant>, FsError>,
    ) -> Result<usize, FsError> {
        let length = source.len() - source.copied();
        if endpoint.send_capacity() < length {
            return Err(FsError::MessageTooLarge);
        }

        let mut payload = zeroed_bytes(length)?;
        if source.pull(&mut payload)? < length {
            // 部分 copyin 的首个 fault 保存在 WriteBuffer，整条记录不得提交。
            source.pull(&mut [])?;
            unreachable!("incomplete UNIX record copy has no stored error");
        }

        let mut payload = Some(payload);
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            match endpoint.send_record(&mut payload) {
                Ok(IoProgress::Ready(count)) => return Ok(count),
                Ok(IoProgress::Blocked) => {}
                Ok(IoProgress::Eof) => unreachable!("record send returned receive EOF"),
                Err(FsError::BrokenPipe) => {
                    if flags & MSG_NOSIGNAL == 0 {
                        if let FileIoContext::User(user) = caller {
                            user.send_kernel_signal(SIGPIPE);
                        }
                    }
                    return Err(FsError::BrokenPipe);
                }
                Err(error) => return Err(error),
            }
            if self.nonblocking(flags) {
                return Err(FsError::WouldBlock);
            }

            let deadline = Self::check_wait(deadline, interrupted, FsError::WouldBlock)?;
            if let Some((context, _)) = &waiting {
                interrupted = context
                    .park_until(matches!(caller, FileIoContext::User(_)), deadline)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 关闭协议实际方向，流发送关闭不妨碍反向接收。
    pub fn shutdown(&self, how: u32) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.shutdown(how),
            Endpoint::Udp(endpoint) => endpoint.shutdown(how),
            Endpoint::Unix(endpoint) => endpoint.shutdown(how),
        }
    }

    /// 查询实际绑定或路由选源，匿名 UNIX 返回无路径名称。
    pub fn local_address(&self) -> Result<SocketAddress, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint
                .local_address()
                .map(|local| SocketAddress::from_endpoint(local, endpoint.family())),
            Endpoint::Udp(endpoint) => endpoint
                .local_address()
                .map(|local| SocketAddress::from_endpoint(local, endpoint.family())),
            Endpoint::Unix(endpoint) => endpoint.local_address(),
        }
    }

    /// 返回真实连接的 peer 或匿名名称，未连接错误由协议对象保留。
    pub fn peer_address(&self) -> Result<SocketAddress, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint
                .peer_address()
                .map(|peer| SocketAddress::from_endpoint(peer, endpoint.family())),
            Endpoint::Udp(endpoint) => endpoint
                .peer_address()
                .map(|peer| SocketAddress::from_endpoint(peer, endpoint.family())),
            Endpoint::Unix(endpoint) => endpoint.peer_address(),
        }
    }

    /// 一次消费协议记录的实际错误，查询不主动推进收发。
    pub fn take_error(&self) -> Option<FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.take_error(),
            Endpoint::Udp(endpoint) => endpoint.take_error(),
            Endpoint::Unix(endpoint) => endpoint.take_error(),
        }
    }

    /// 只有真实 TCP listener 才提供接受连接能力。
    pub fn is_listener(&self) -> bool {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.is_listener(),
            Endpoint::Udp(_) | Endpoint::Unix(_) => false,
        }
    }

    /// IP 返回原 raw 的有效接收预算，UNIX 返回共同 FIFO 的真实容量。
    pub fn receive_budget(&self) -> usize {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.receive_budget(),
            Endpoint::Udp(endpoint) => endpoint.receive_budget(),
            Endpoint::Unix(endpoint) => endpoint.receive_capacity(),
        }
    }

    /// IP 返回原 raw 的有效发送预算，UNIX 不展示超过后备的配置。
    pub fn send_budget(&self) -> usize {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.send_budget(),
            Endpoint::Udp(endpoint) => endpoint.send_budget(),
            Endpoint::Unix(endpoint) => endpoint.send_capacity(),
        }
    }

    /// signed int 按 unsigned 提示限幅倍增，实际预算不超过原物理后备。
    fn buffer_budget(hint: i32, capacity: usize) -> usize {
        ((hint as u32).min(BUFFER_HINT_MAX) as usize * BUFFER_BUDGET_MULTIPLIER)
            .max(BUFFER_BUDGET_MIN)
            .min(capacity)
    }

    /// 设置同一接收队列的预算，不重新分配或丢弃已接纳数据。
    pub fn set_receive_budget(&self, hint: i32) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => {
                endpoint.set_receive_budget(Self::buffer_budget(hint, endpoint.receive_capacity()))
            }
            Endpoint::Udp(endpoint) => {
                endpoint.set_receive_budget(Self::buffer_budget(hint, endpoint.receive_capacity()))
            }
            Endpoint::Unix(_) => Err(FsError::Unsupported),
        }
    }

    /// 设置同一发送队列的预算，恢复通知由真实端点在锁外处理。
    pub fn set_send_budget(&self, hint: i32) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => {
                endpoint.set_send_budget(Self::buffer_budget(hint, endpoint.send_capacity()))
            }
            Endpoint::Udp(endpoint) => {
                endpoint.set_send_budget(Self::buffer_budget(hint, endpoint.send_capacity()))
            }
            Endpoint::Unix(_) => Err(FsError::Unsupported),
        }
    }

    /// TCP 查询实际探测定时器开关，其他协议只查询同 OFD 的通用布尔值。
    pub fn keepalive(&self) -> bool {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.keepalive(),
            Endpoint::Udp(_) | Endpoint::Unix(_) => self.generic_keepalive.load(Ordering::Acquire),
        }
    }

    /// TCP 修改真实 7200 秒探测，UDP 和 UNIX 不创建探测工作。
    pub fn set_keepalive(&self, enabled: bool) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_keepalive(enabled),
            Endpoint::Udp(_) | Endpoint::Unix(_) => {
                self.generic_keepalive.store(enabled, Ordering::Release);
                Ok(())
            }
        }
    }

    /// 只复制实际端点的控制属性，不取得协议操作权或推动后台进度。
    pub(crate) fn timeout(&self, receive: bool) -> Option<u64> {
        let timeouts = match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.timeouts(),
            Endpoint::Udp(endpoint) => endpoint.timeouts(),
            Endpoint::Unix(endpoint) => endpoint.timeouts(),
        };
        if receive {
            timeouts.receive
        } else {
            timeouts.send
        }
    }

    /// 锁外完成 ABI 转换后，将期限写入与 dup、fork 共享的真实端点。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_timeout(receive, timeout),
            Endpoint::Udp(endpoint) => endpoint.set_timeout(receive, timeout),
            Endpoint::Unix(endpoint) => endpoint.set_timeout(receive, timeout),
        }
    }

    /// 每次请求只计算一次绝对期限；有限加法溢出保留真实参数错误。
    fn deadline(&self, receive: bool) -> Result<Option<MonotonicInstant>, FsError> {
        self.timeout(receive)
            .map(|nanos| {
                time::now()
                    .checked_add(Duration::from_nanos(nanos))
                    .ok_or(FsError::InvalidOperation)
            })
            .transpose()
    }

    /// 无进度且已排除 nonblock 后检查期限与中断，业务结果不被加法溢出覆盖。
    fn check_wait(
        deadline: Result<Option<MonotonicInstant>, FsError>,
        interrupted: bool,
        expired: FsError,
    ) -> Result<Option<MonotonicInstant>, FsError> {
        let deadline = deadline?;
        if deadline.is_some_and(|limit| limit <= time::now()) {
            return Err(expired);
        }
        if interrupted {
            return Err(if deadline.is_some() {
                FsError::InterruptedNoRestart
            } else {
                FsError::Interrupted
            });
        }
        Ok(deadline)
    }

    /// MSS 查询沿 raw 当前 tuple 与路径决策，不在打开对象重复计算。
    pub fn effective_mss(&self) -> Result<usize, FsError> {
        Ok(self.tcp_endpoint()?.effective_mss())
    }

    /// 返回真实 raw 拥塞控制器，打开对象不保存另一份算法名称。
    pub fn congestion_control(&self) -> Result<CongestionControl, FsError> {
        Ok(self.tcp_endpoint()?.congestion_control())
    }

    /// 所选算法仅为已启用的 Reno，其他名称保留真实查找失败。
    pub fn set_congestion_control(&self, name: &[u8]) -> Result<(), FsError> {
        let controller = self.congestion_control()?;
        if name != b"reno" {
            return Err(FsError::NotFound);
        }
        match controller {
            CongestionControl::Reno => Ok(()),
            CongestionControl::None => Err(FsError::Unsupported),
        }
    }

    /// 查询协议保存的地址复用事实，不改变绑定索引或产生通知。
    pub fn reuse_address(&self) -> bool {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.reuse_address(),
            Endpoint::Udp(endpoint) => endpoint.reuse_address(),
            Endpoint::Unix(endpoint) => endpoint.reuse_address(),
        }
    }

    /// 沿协议端点更新复用策略，绑定表和 listener 约束由同域控制处理。
    pub fn set_reuse_address(&self, reuse: bool) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_reuse_address(reuse),
            Endpoint::Udp(endpoint) => endpoint.set_reuse_address(reuse),
            Endpoint::Unix(endpoint) => endpoint.set_reuse_address(reuse),
        }
    }

    /// TCP_NODELAY 只查询 TCP 的实际 Nagle 状态，其他协议明确不支持。
    pub fn nodelay(&self) -> Result<bool, FsError> {
        self.tcp_endpoint()?.nodelay()
    }

    /// 修改 TCP 的实际 Nagle 状态，由原端点安排协议 Work。
    pub fn set_nodelay(&self, enabled: bool) -> Result<(), FsError> {
        self.tcp_endpoint()?.set_nodelay(enabled)
    }

    /// 真实 Blocked 后才准备原等待上下文与一源表，登记结束后由调用者重新扫描。
    fn prepare_wait(&self) -> Result<(Arc<PollContext>, PollTable), FsError> {
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let mut table = PollTable::new(sink, 1)?;
        self.poll(Some(&mut table))?;
        Ok((context, table))
    }

    /// 每次扫描共同消费 OFD 状态和本次消息的非阻塞要求。
    fn nonblocking(&self, flags: u32) -> bool {
        self.flags() & O_NONBLOCK != 0 || flags & MSG_DONTWAIT != 0
    }

    /// 流消息元数据只描述本次交付前缀，不构造报文来源。
    fn stream_result(count: usize) -> MessageResult {
        MessageResult {
            copied: count,
            original_len: count,
            source: None,
            flags: 0,
        }
    }

    /// 已交付正前缀优先于后续 fault 或协议错误，零进度保留原错误。
    fn progress(count: usize, error: FsError) -> Result<usize, FsError> {
        if count == 0 {
            Err(error)
        } else {
            Ok(count)
        }
    }

    /// 匿名 socket 属性使用本域稳定身份和创建者凭据，不伪装 regular 或磁盘路径。
    pub fn stat(&self) -> FileStat {
        let zero = InodeTime {
            seconds: 0,
            nanoseconds: 0,
        };
        FileStat {
            inode: self.id(),
            kind: InodeKind::Socket,
            mode: S_IFSOCK | S_IRWXUGO,
            size: 0,
            uid: self.uid,
            gid: self.gid,
            links: 1,
            blocks: 0,
            device: 0,
            rdev: 0,
            block_size: self.net.loopback_mtu() as u32,
            atime: zero,
            mtime: zero,
            ctime: zero,
        }
    }
}

impl Pollable for SocketFile {
    /// 首轮登记端点唯一持续源，其后只查询原协议状态，不收包或 dispatch。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.poll(table),
            Endpoint::Udp(endpoint) => endpoint.poll(table),
            Endpoint::Unix(endpoint) => endpoint.poll(table),
        }
    }
}

impl FileLike for SocketFile {
    /// 普通零长 read 不消费报文，非空请求沿同一协议交付和等待路径。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if target.copied() == target.len() {
            return Ok(0);
        }
        self.receive(target, 0, caller).map(|result| result.copied)
    }

    /// TCP 流没有可定位位置，pread 不访问用户数据即返回 ESPIPE。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 匿名 socket 不提供目录路径。
    fn path(&self) -> Option<&Path> {
        None
    }

    /// 流数据不提供文件映射 inode。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 事件扫描器直接观察本 OFD 对应的端点源。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    /// 网络控制只取得此真实 SocketFile，不从 stat 类型猜协议对象。
    fn socket(&self) -> Option<&SocketFile> {
        Some(self)
    }

    /// 复制描述符共享同一反向 watch 和末 OFD 注销责任。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        Some(&self.watches)
    }

    /// 接口与默认路由控制使用本 OFD 的同一网络域和本次调用者，不读取创建者权限。
    fn ioctl(
        &self,
        request: u32,
        argument: usize,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        crate::net::control::ioctl(&self.net, request, argument, caller)
    }

    /// 读取共享 OFD 状态，CLOEXEC 始终归描述符槽。
    fn flags(&self) -> u32 {
        SocketFile::flags(self)
    }

    /// F_SETFL 修改原 OFD 状态，不改变双向访问模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        SocketFile::set_status_flags(self, raw)
    }

    /// 只原子修改 socket OFD 的非阻塞位，不改变访问模式和追加位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        crate::fs::set_nonblocking(&self.flags, enabled)
    }

    /// 网络流没有文件长度可截断。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 网络流没有文件存储同步范围。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 流没有共享字节位置，不创建虚假游标。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// fstat 使用真实创建者、本域稳定身份及 S_IFSOCK0777。
    fn stat(&self) -> Result<FileStat, FsError> {
        Ok(SocketFile::stat(self))
    }

    /// 匿名 socket 不能读取目录项。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 匿名 socket 不授予目录枚举能力。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 普通 write/writev 使用默认 peer，UDP 空输入仍提交真实零长消息。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.send(source, None, 0, caller)
    }

    /// TCP 流不能执行 pwrite。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
}

impl Drop for SocketFile {
    /// 末打开实例先同步撤反向 watch，再请求正常协议关闭，不等待 peer 或后台 Work。
    fn drop(&mut self) {
        self.watches.release();
        if self.opened {
            self.endpoint.close();
        }
    }
}
