//! 成熟 TCP 算法拥有连接自己的 ring 与环境；本层只处理包拥有、复制进度和时间域转换。

use ::core::convert::TryFrom;
use alloc::vec::Vec;
use smoltcp::socket::tcp::TcpContext;
use smoltcp::socket::{tcp, PollAt};
use smoltcp::time::{Duration, Instant};
use smoltcp::wire::IpVersion;

use crate::fs::{FsError, ReadBuffer, WriteBuffer};
use crate::net::address::{AddressFamily, InetEndpoint};
use crate::net::packet::{zeroed_bytes, OwnedPacket, MAX_IP_PACKET_LEN};
use crate::net::socket::STREAM_PREFIX_LEN;
use crate::net::IoProgress;
use crate::time::{MonotonicInstant, NSEC_PER_USEC};

const TCP_MAX_RECEIVE_CAPACITY: usize = 1 << 30;
const IPV4_MIN_MTU: usize = 68;
const TCP_KEEPALIVE_IDLE: Duration = Duration::from_secs(7200);

/// 立即工作、绝对定时和仅等待输入保持不同责任，egress 阻塞由真实容量源解除。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProtocolDeadline {
    Now,
    At(MonotonicInstant),
    Ingress,
}

/// 单连接算法和缓冲均由本对象拥有，不借全局 Interface 或 SocketSet。
pub struct TcpCore {
    socket: tcp::Socket<'static>,
    context: tcp::TcpContextState,
    buffers_ready: bool,
    receive_capacity: usize,
    send_capacity: usize,
    receive_budget: usize,
    send_budget: usize,
}

impl TcpCore {
    /// 创建零容量冷核心并记录后备配置，首次监听或连接才准备两向 ring。
    pub fn try_new(
        rx_len: usize,
        tx_len: usize,
        ip_mtu: usize,
        seed: u64,
    ) -> Result<Self, FsError> {
        if rx_len == 0 || tx_len == 0 || TCP_MAX_RECEIVE_CAPACITY < rx_len {
            return Err(FsError::InvalidOperation);
        }
        if !(IPV4_MIN_MTU..=MAX_IP_PACKET_LEN).contains(&ip_mtu) {
            return Err(FsError::InvalidOperation);
        }

        let mut socket = tcp::Socket::new(
            tcp::SocketBuffer::new(Vec::new()),
            tcp::SocketBuffer::new(Vec::new()),
        );
        socket.set_congestion_control(tcp::CongestionControl::Reno);
        Ok(Self {
            socket,
            context: tcp::TcpContextState::new(Instant::ZERO, ip_mtu, seed),
            buffers_ready: false,
            receive_capacity: rx_len,
            send_capacity: tx_len,
            receive_budget: rx_len,
            send_budget: tx_len,
        })
    }

    /// 操作拥有内一次安装两向后备；任一准备失败保留冷 raw、选项和环境。
    pub(crate) fn prepare_buffers(&mut self) -> Result<(), FsError> {
        if self.buffers_ready {
            return Ok(());
        }

        let rx = tcp::SocketBuffer::new(zeroed_bytes(self.receive_capacity)?);
        let tx = tcp::SocketBuffer::new(zeroed_bytes(self.send_capacity)?);
        let mut socket = tcp::Socket::new(rx, tx);
        socket.set_congestion_control(self.socket.congestion_control());
        socket.set_nagle_enabled(self.socket.nagle_enabled());
        socket.set_keep_alive(self.socket.keep_alive());
        socket.set_recv_budget(self.receive_budget);
        socket.set_send_budget(self.send_budget);

        self.socket = socket;
        // 后备安装后永不再替换 raw，协议回到 Closed 也保留原队列和算法拥有。
        self.buffers_ready = true;
        Ok(())
    }

    /// 源地址和端口已由网络域选择；本核心只开始这个明确四元组的握手。
    pub fn connect(
        &mut self,
        local: InetEndpoint,
        peer: InetEndpoint,
        now: MonotonicInstant,
    ) -> Result<(), FsError> {
        self.prepare_buffers()?;
        self.context.set_now(raw_time(now));
        self.socket
            .connect(&mut self.context, peer.raw(), local.raw())
            .map_err(|error| match error {
                tcp::ConnectError::InvalidState => FsError::InvalidOperation,
                tcp::ConnectError::Unaddressable => FsError::AddressNotAvailable,
            })
    }

    /// 监听保留通配地址，不分配临时端口；绑定冲突属于同一网络域的表。
    pub fn listen(&mut self, local: InetEndpoint) -> Result<(), FsError> {
        self.prepare_buffers()?;
        self.socket
            .listen(local.raw_listen())
            .map_err(|error| match error {
                tcp::ListenError::InvalidState => FsError::InvalidOperation,
                tcp::ListenError::Unaddressable => FsError::AddressNotAvailable,
            })
    }

    /// 在连接保护内消费 owned TCP repr；立即回应结束当前借用前取得独立 payload 拥有。
    pub fn process(
        &mut self,
        packet: &OwnedPacket,
        now: MonotonicInstant,
    ) -> Result<Option<OwnedPacket>, FsError> {
        let (ip, tcp) = packet.tcp()?;
        self.context.set_now(raw_time(now));
        if !self.socket.accepts(&ip, &tcp) {
            return Err(FsError::InvalidOperation);
        }

        let ip_mtu = self.context.ip_mtu();
        self.socket
            .process(&mut self.context, &ip, &tcp, tcp.payload)
            .map(|(ip, tcp)| OwnedPacket::try_from_tcp(&ip, &tcp, ip_mtu))
            .transpose()
    }

    /// 同步 emitter 先接受真正拥有的包，成功后 raw 才提交正常发送进度。
    /// raw 的 emit 前重传和计时副作用遵循算法原有次序，错误不伪造全状态回滚。
    pub fn dispatch<F>(&mut self, now: MonotonicInstant, emit: F) -> Result<(), FsError>
    where
        F: FnOnce(OwnedPacket) -> Result<(), FsError>,
    {
        self.context.set_now(raw_time(now));
        let ip_mtu = self.context.ip_mtu();
        self.socket.dispatch(&mut self.context, |_, (ip, tcp)| {
            emit(OwnedPacket::try_from_tcp(&ip, &tcp, ip_mtu)?)
        })
    }

    /// 将算法三态期限映射到同一个 monotonic 时间域，不将 Now 当成周期重排许可。
    pub fn deadline(&mut self, now: MonotonicInstant) -> ProtocolDeadline {
        self.context.set_now(raw_time(now));
        match self.socket.poll_at(&mut self.context) {
            PollAt::Now => ProtocolDeadline::Now,
            PollAt::Ingress => ProtocolDeadline::Ingress,
            PollAt::Time(at) => {
                let micros = u64::try_from(at.total_micros()).expect("negative TCP deadline");
                let nanos = micros
                    .checked_mul(NSEC_PER_USEC)
                    .expect("TCP deadline overflow");
                ProtocolDeadline::At(MonotonicInstant::from_nanos(nanos))
            }
        }
    }

    /// 持本连接操作拥有暂存至多 4096 字节；raw 借用结束后复制，按实际交付差量消费。
    /// PEEK 使用同一连续前缀，但不改变接收位置。
    pub fn receive(
        &mut self,
        target: &mut ReadBuffer<'_>,
        peek: bool,
    ) -> Result<IoProgress, FsError> {
        if target.copied() == target.len() {
            return Ok(IoProgress::Ready(0));
        }

        let before = target.copied();
        let mut prefix = [0u8; STREAM_PREFIX_LEN];
        let length = {
            let bytes = match self.socket.peek((target.len() - before).min(prefix.len())) {
                Ok(bytes) => bytes,
                Err(error) => return self.receive_error(error),
            };
            prefix[..bytes.len()].copy_from_slice(bytes);
            bytes.len()
        };

        let result = target.push(&prefix[..length]);
        let copied = target.copied() - before;
        if !peek && 0 < copied {
            self.socket
                .recv(|bytes| {
                    assert!(
                        copied <= bytes.len(),
                        "TCP receive prefix changed under ownership"
                    );
                    (copied, ())
                })
                .expect("TCP receive state changed under ownership");
        }
        copy_progress(copied, result)
    }

    /// 仅可发送状态接受零请求；非零输入的真实 TX 提交量来自 raw 闭包差量。
    pub fn send(&mut self, source: &mut WriteBuffer<'_>) -> Result<IoProgress, FsError> {
        if source.copied() == source.len() && self.socket.may_send() {
            return Ok(IoProgress::Ready(0));
        }

        let before = source.copied();
        match self.socket.send(|bytes| {
            let copied = source.pull(bytes);
            let accepted = source.copied() - before;
            (accepted, copy_progress(accepted, copied))
        }) {
            Ok(progress) => progress,
            Err(tcp::SendError::InvalidState) => match self.socket.state() {
                tcp::State::SynSent | tcp::State::SynReceived => Ok(IoProgress::Blocked),
                tcp::State::FinWait1
                | tcp::State::FinWait2
                | tcp::State::Closing
                | tcp::State::LastAck
                | tcp::State::TimeWait => Err(FsError::BrokenPipe),
                _ => Err(FsError::NotConnected),
            },
        }
    }

    /// 将 raw 的握手等待、FIN 后完整 EOF 和未连接错误保留为不同结果。
    fn receive_error(&self, error: tcp::RecvError) -> Result<IoProgress, FsError> {
        match error {
            tcp::RecvError::Finished => Ok(IoProgress::Eof),
            tcp::RecvError::InvalidState => match self.socket.state() {
                tcp::State::SynSent | tcp::State::SynReceived => Ok(IoProgress::Blocked),
                _ => Err(FsError::NotConnected),
            },
        }
    }

    /// 请求发送 FIN，已接受的发送字节仍沿原算法排出。
    pub fn close(&mut self) {
        self.socket.close();
    }

    /// 请求立即终止，后续 dispatch 按原算法发送 RST 并结束 tuple。
    pub fn abort(&mut self) {
        self.socket.abort();
    }

    /// 状态只有成熟协议核心的一份，不在适配层建立第二份状态机。
    pub fn state(&self) -> tcp::State {
        self.socket.state()
    }

    /// 连接建立后返回明确的本地端点值，监听地址由 listen_endpoint 查询。
    pub fn local_endpoint(&self) -> Option<InetEndpoint> {
        self.socket.local_endpoint().map(InetEndpoint::from_raw)
    }

    /// 返回这个核心真正拥有的远端地址快照。
    pub fn remote_endpoint(&self) -> Option<InetEndpoint> {
        self.socket.remote_endpoint().map(InetEndpoint::from_raw)
    }

    /// raw 通配监听不保存地址族，由端点的不可变族恢复对应未指定地址。
    #[cfg(feature = "ktest")]
    pub fn listen_endpoint(&self, family: AddressFamily) -> InetEndpoint {
        let endpoint = self.socket.listen_endpoint();
        InetEndpoint {
            address: endpoint.addr.unwrap_or(family.unspecified()),
            port: endpoint.port,
        }
    }

    /// 返回配置的接收后备容量，冷 listener child 仍继承同容量。
    pub fn receive_capacity(&self) -> usize {
        self.receive_capacity
    }

    /// 返回配置的发送后备容量，冷对象不以零 ring 限制早期选项。
    pub fn send_capacity(&self) -> usize {
        self.send_capacity
    }

    /// 返回配置的接收预算，激活后同值交给 raw 管理数据和通告信用。
    pub fn receive_budget(&self) -> usize {
        self.receive_budget
    }

    /// 返回配置的发送预算，激活后未 ACK 字节占用同一预算。
    pub fn send_budget(&self) -> usize {
        self.send_budget
    }

    /// 预算须处于配置的接收后备内，热核心只调整原 ring 的接纳预算。
    pub fn set_receive_budget(&mut self, budget: usize) -> Result<(), FsError> {
        if budget == 0 || self.receive_capacity() < budget {
            return Err(FsError::InvalidOperation);
        }
        if self.buffers_ready {
            self.socket.set_recv_budget(budget);
        }
        self.receive_budget = budget;
        Ok(())
    }

    /// 预算须处于配置的发送后备内，热核心保留已有未 ACK 字节。
    pub fn set_send_budget(&mut self, budget: usize) -> Result<(), FsError> {
        if budget == 0 || self.send_capacity() < budget {
            return Err(FsError::InvalidOperation);
        }
        if self.buffers_ready {
            self.socket.set_send_budget(budget);
        }
        self.send_budget = budget;
        Ok(())
    }

    /// 读取原空闲探测间隔是否启用，不缓存第二份 socket 开关。
    pub fn keepalive(&self) -> bool {
        self.socket.keep_alive().is_some()
    }

    /// 原 TCP 探测使用 7200 秒，关闭直接撤销原探测期限。
    pub fn set_keepalive(&mut self, enabled: bool) {
        self.socket
            .set_keep_alive(enabled.then_some(TCP_KEEPALIVE_IDLE));
    }

    /// 路由已选择实际源和介质后更新唯一发送环境。
    pub fn set_ip_mtu(&mut self, mtu: usize) {
        assert!(
            (IPV4_MIN_MTU..=MAX_IP_PACKET_LEN).contains(&mtu),
            "invalid TCP route MTU"
        );
        self.context.set_ip_mtu(mtu);
    }

    /// 未连接使用 OFD 族，已连接实际 tuple 的协议族由 raw 优先选择。
    pub fn effective_mss(&self, family: AddressFamily) -> usize {
        let version = match family {
            AddressFamily::Ipv4 => IpVersion::Ipv4,
            AddressFamily::Ipv6 => IpVersion::Ipv6,
        };
        self.socket.effective_mss(&self.context, version)
    }

    /// 返回真正处理发送拥塞窗口的原控制器。
    pub fn congestion_control(&self) -> tcp::CongestionControl {
        self.socket.congestion_control()
    }

    /// 查询 raw 的实际 Nagle 策略，不从发送队列或状态推算 TCP_NODELAY。
    pub fn nagle_enabled(&self) -> bool {
        self.socket.nagle_enabled()
    }

    /// 调用者持本连接修改保护；新策略由原 dispatch 和 Work 安排生效。
    pub fn set_nagle_enabled(&mut self, enabled: bool) {
        self.socket.set_nagle_enabled(enabled);
    }

    /// 返回当前可交付字节数，不能以它替代 FIN 或错误查询。
    pub fn receive_queue(&self) -> usize {
        self.socket.recv_queue()
    }

    /// 返回已被协议接受但尚未 ACK 释放的发送 ring 字节数。
    pub fn send_queue(&self) -> usize {
        self.socket.send_queue()
    }

    /// 就绪判断直接取原算法的接收能力，不推测用户是否已经读取。
    pub fn can_receive(&self) -> bool {
        self.socket.can_recv()
    }

    /// 就绪判断直接取原算法的发送能力和真实 ring 容量。
    pub fn can_send(&self) -> bool {
        self.socket.can_send()
    }

    /// 接收 FIN 的状态由原协议表达；本地关闭发送方向仍不结束接收。
    pub fn receive_finished(&self) -> bool {
        matches!(
            self.socket.state(),
            tcp::State::CloseWait
                | tcp::State::LastAck
                | tcp::State::Closing
                | tcp::State::TimeWait
        )
    }
}

/// 零进度的非空请求等待状态变化；故障后的正前缀保持成功提交。
fn copy_progress(copied: usize, result: Result<usize, FsError>) -> Result<IoProgress, FsError> {
    if 0 < copied {
        return Ok(IoProgress::Ready(copied));
    }
    result.map(|_| IoProgress::Blocked)
}

/// 单调纳秒转 raw 微秒；u64 纳秒除以一千后始终可表示为 i64。
fn raw_time(now: MonotonicInstant) -> Instant {
    Instant::from_micros((now.as_nanos() / NSEC_PER_USEC) as i64)
}
