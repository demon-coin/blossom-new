//! 单连接 TCP 协议核心封装模块（`TcpCore`）。
//!
//! ## 模块职责与架构定位
//! 本模块将 `smoltcp` 的单连接 TCP 协议状态机（`tcp::Socket`）与环境上下文（`TcpContextState`）封装为
//! 独立拥有的 [`TcpCore`] 对象，作为上层连接管理层（[`crate::net::tcp::TcpEndpoint`]）的底层引擎：
//! - **向上（`TcpEndpoint`）**：提供建立连接（`connect`）、监听（`listen`）、入站报文推进（`process`）、
//!   出站报文构造（`dispatch`）、单调时钟期限计算（`deadline`）以及用户态缓冲区流式读写（`receive` / `send`）。
//! - **向下（`smoltcp` 与 `OwnedPacket`）**：负责内核 `MonotonicInstant`（纳秒）到 `smoltcp::time::Instant`（微秒）的时间换算，
//!   并将协议栈输出的借用 `IpRepr` / `TcpRepr` 转换为自包含的内核 [`OwnedPacket`]。
//!
//! ## 核心机制：冷启动延迟分配（Lazy Ring Allocation）与分段流式复制
//! ```text
//!   try_new()                     prepare_buffers() (首次 connect / listen / accept child)
//! +-----------------------+       +------------------------------------------------------+
//! | 冷核心 (Cold Core)     |       | 热核心 (Active Core)                                  |
//! | buffers_ready = false | ----> | buffers_ready = true                                 |
//! | RX/TX Vec 容量 = 0     |       | 一次性分配 receive_capacity + send_capacity 环形缓冲区  |
//! | 仅记录容量/预算与选项   |       | 迁移 Nagle / KeepAlive / 拥塞控制 / 收发预算配置       |
//! +-----------------------+       +------------------------------------------------------+
//! ```
//! - **延迟分配**：刚调用 `socket(AF_INET, SOCK_STREAM, 0)` 创建但尚未连接或监听的套接字处于冷状态，
//!   不提前占用数百 KB 的收发环形缓冲区；在冷状态下调用 `setsockopt` 修改 `SO_RCVBUF` / `SO_SNDBUF` / `TCP_NODELAY` 等选项仍可正常保存。
//!   一旦 `buffers_ready` 变为 `true`，底层收发缓冲区终身固定，后续调整 `SO_RCVBUF`/`SO_SNDBUF` 仅更新接纳预算（Budget）。
//! - **两阶段接收复制（`receive`）**：读取时先从接收环形缓冲区 `peek` 至多 `STREAM_PREFIX_LEN`（4096 字节）到栈上临时数组，
//!   释放对 `socket` 的借用后再向用户态 `ReadBuffer` 拷贝（可能触发缺页处理），最后仅按实际成功写入用户空间的字节数推进消费指针。
//!   若拷贝中途发生缺页错误（`EFAULT`），已拷贝的前缀字节仍作为部分成功进度（Partial Progress）提交并返回，避免数据丢失或重复。
//!
//! ## 锁与并发契约
//! `TcpCore` 自身不包含内部锁，也不挂入全局共享的 `SocketSet`。调用方（`TcpEndpoint`）必须通过连接级互斥锁（`SleepMutex`）
//! 独占持有 `&mut TcpCore`，从而使不同 TCP 连接的协议推进、窗口计算与用户态拷贝完全并行执行。

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

/// TCP 接收缓冲区配置容量的上限（1 GiB），防止异常容量请求导致算术溢出或离谱分配。
const TCP_MAX_RECEIVE_CAPACITY: usize = 1 << 30;
/// RFC 791 规定的 IPv4 最小链路路径 MTU（68 字节：60 字节最大 IPv4 首部 + 8 字节最小分片载荷）。
const IPV4_MIN_MTU: usize = 68;
/// 开启 `SO_KEEPALIVE` 后默认的连接空闲探测间隔（7200 秒，即 2 小时，遵循 Linux 默认值）。
const TCP_KEEPALIVE_IDLE: Duration = Duration::from_secs(7200);

/// TCP 协议核心计算出的下一次推进期限。
///
/// 上层 `TcpEndpoint` 根据该期限决定是立即继续调度连接 Work、挂载高精度定时器，还是静默等待外部网络包到达。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProtocolDeadline {
    /// 协议栈有待立即发送的控制段或数据段，需要立即推进 `dispatch`（若受限于出站队列容量，则转入容量等待队列）。
    Now,
    /// 协议栈需要在指定的单调时钟绝对时刻触发超时处理（如重传 RTO、TIME_WAIT 结束、延时 ACK 或 Keep-Alive 探测）。
    At(MonotonicInstant),
    /// 当前无定时任务也无待发数据，仅需等待对端入站报文或本地用户态新操作唤醒。
    Ingress,
}

/// 单条 TCP 连接独立拥有的协议状态机与收发环形缓冲区核心。
pub struct TcpCore {
    /// `smoltcp` 单连接 TCP 状态机与收发环形缓冲区实例。
    socket: tcp::Socket<'static>,
    /// 单连接独立的 TCP 协议上下文（保存当前单调时间、路径 IP MTU 与初始序列号随机种子）。
    context: tcp::TcpContextState,
    /// 标记底层收发环形缓冲区是否已完成实际堆内存分配并安装。
    buffers_ready: bool,
    /// 接收环形缓冲区的物理分配总容量（字节）。
    receive_capacity: usize,
    /// 发送环形缓冲区的物理分配总容量（字节）。
    send_capacity: usize,
    /// 当前生效的接收窗口接纳预算（字节，对应 `SO_RCVBUF`，不超过 `receive_capacity`）。
    receive_budget: usize,
    /// 当前生效的发送缓冲区接纳预算（字节，对应 `SO_SNDBUF`，不超过 `send_capacity`）。
    send_budget: usize,
}

impl TcpCore {
    /// 创建一个尚未分配收发环形缓冲区的冷态 [`TcpCore`]。
    ///
    /// 仅校验收发容量与初始 `ip_mtu` 合法性，并初始化默认使用 Reno 拥塞控制的零容量 `tcp::Socket`；
    /// 实际的收发环形缓冲区推迟到首次调用 [`Self::connect`]、[`Self::listen`] 或被动建连时由 [`Self::prepare_buffers`] 分配。
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

    /// 为冷态核心一次性分配并安装收发双向环形缓冲区，并将已配置的套接字选项迁移到新实例上。
    ///
    /// 调用方须持有所属 `TcpEndpoint` 的连接互斥锁。若任一方向内存分配失败，原冷态 `socket` 的选项与状态保持不变并返回 `NoMemory`。
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
        // 环形缓冲区一旦分配安装便终生固定；即使后续连接回到 Closed 状态也直接复用，不再重复分配或替换底层存储。
        self.buffers_ready = true;
        Ok(())
    }

    /// 在指定的明确四元组（`local` -> `peer`）上发起主动 TCP 三次握手（进入 `SynSent` 状态）。
    ///
    /// 调用前本地源 IP 地址与源端口必须已由路由查询和端口表绑定选定；普通可睡眠任务持连接锁调用。
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

    /// 将核心切换为被动监听状态（`Listen`），准备接收发往 `local` 端点的入站 `SYN` 握手请求。
    ///
    /// 若 `local` 为通配地址（`0.0.0.0` 或 `::`），底层监听端点不限制具体本地 IP，实际收包范围由上层端口绑定表控制。
    pub fn listen(&mut self, local: InetEndpoint) -> Result<(), FsError> {
        self.prepare_buffers()?;
        self.socket
            .listen(local.raw_listen())
            .map_err(|error| match error {
                tcp::ListenError::InvalidState => FsError::InvalidOperation,
                tcp::ListenError::Unaddressable => FsError::AddressNotAvailable,
            })
    }

    /// 向 TCP 状态机喂入一个入站 [`OwnedPacket`]，推进确认号、窗口、状态迁移并接收有效载荷。
    ///
    /// 若协议栈需要立即回复控制段（如挑战 ACK、SYN-ACK 或 RST），则在释放借用前将其深拷贝构造为新的 [`OwnedPacket`] 返回给调用方发送。
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

    /// 驱动 TCP 状态机尝试生成一帧待发送的出站报文，并通过同步闭包 `emit` 提交到网络栈出站队列。
    ///
    /// 只有当 [`OwnedPacket`] 构造成功且 `emit` 返回 `Ok(())` 时，`smoltcp` 才会正式推进正常发送窗口指针；
    /// 若因出站队列满（`WouldBlock`）或内存不足返回错误，未发送的数据段仍保留在发送缓冲区中等待下次调度。
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

    /// 查询协议栈下一次需要驱动的时间点，并将 `smoltcp` 微秒时间戳转换为内核单调时钟 [`ProtocolDeadline`]。
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

    /// 从 TCP 接收缓冲区向用户态（或内核）`target` 读取流式字节数据。
    ///
    /// 若 `peek` 为 `true`（对应 `MSG_PEEK`），仅拷贝可读前缀而不推进接收消费指针；
    /// 正常读取时采用“先 `peek` 到栈缓冲、再写入 `target`、最后按实际写入差量 `recv` 消费”的两阶段策略，
    /// 确保向用户空间拷贝触发缺页异常时不会破坏协议缓冲状态，也不会丢失已成功写入的前缀字节。
    pub fn receive(
        &mut self,
        target: &mut ReadBuffer<'_>,
        peek: bool,
    ) -> Result<IoProgress, FsError> {
        // Step 1: 零长度读取请求无需访问底层协议状态，直接返回 Ready(0)。
        if target.copied() == target.len() {
            return Ok(IoProgress::Ready(0));
        }

        // Step 2: 从接收环形缓冲区预览最多 4096 字节到栈上固定数组，提前结束对 self.socket 的借用。
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

        // Step 3: 将栈上数据写入目标 ReadBuffer，并根据实际成功写入的字节数推进协议接收窗口。
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

    /// 从 `source` 拉取待发送字节并写入 TCP 发送环形缓冲区。
    ///
    /// - 若连接处于握手未完成状态（`SynSent` / `SynReceived`），返回 `IoProgress::Blocked` 让上层等待连接建立；
    /// - 若连接已关闭本地发送方向（`FinWait1` / `FinWait2` / `Closing` / `LastAck` / `TimeWait`），返回 `BrokenPipe`（触发 `EPIPE` / `SIGPIPE`）；
    /// - 若尚未发起连接或已彻底关闭，返回 `NotConnected`。
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

    /// 将 `smoltcp` 底层接收错误映射为内核流式 I/O 进度：对端已发 FIN 且数据读尽映射为 `Eof`，握手中映射为 `Blocked`，其余为 `NotConnected`。
    fn receive_error(&self, error: tcp::RecvError) -> Result<IoProgress, FsError> {
        match error {
            tcp::RecvError::Finished => Ok(IoProgress::Eof),
            tcp::RecvError::InvalidState => match self.socket.state() {
                tcp::State::SynSent | tcp::State::SynReceived => Ok(IoProgress::Blocked),
                _ => Err(FsError::NotConnected),
            },
        }
    }

    /// 优雅关闭本地发送方向：请求协议栈在排空已有发送缓冲区数据后发送 `FIN` 段。
    pub fn close(&mut self) {
        self.socket.close();
    }

    /// 强制中止连接：丢弃未确认数据，使协议栈在后续 `dispatch` 时发送 `RST` 并直接转入 `Closed` 状态。
    pub fn abort(&mut self) {
        self.socket.abort();
    }

    /// 返回当前 TCP 连接所处的 RFC 793 状态机状态（如 `Listen`、`SynSent`、`Established`、`CloseWait` 等）。
    pub fn state(&self) -> tcp::State {
        self.socket.state()
    }

    /// 返回已关联连接的明确本地端点（IP + 端口）；未连接或纯监听状态下返回 `None`。
    pub fn local_endpoint(&self) -> Option<InetEndpoint> {
        self.socket.local_endpoint().map(InetEndpoint::from_raw)
    }

    /// 返回已关联连接的远端对等体端点（IP + 端口）；未连接或纯监听状态下返回 `None`。
    pub fn remote_endpoint(&self) -> Option<InetEndpoint> {
        self.socket.remote_endpoint().map(InetEndpoint::from_raw)
    }

    /// 查询监听状态下的本地监听端点；若底层为通配地址（`None`），则按套接字所属 `family` 还原为 `0.0.0.0` 或 `::`。
    #[cfg(feature = "ktest")]
    pub fn listen_endpoint(&self, family: AddressFamily) -> InetEndpoint {
        let endpoint = self.socket.listen_endpoint();
        InetEndpoint {
            address: endpoint.addr.unwrap_or(family.unspecified()),
            port: endpoint.port,
        }
    }

    /// 返回接收环形缓冲区配置的物理总容量（字节）；被动建连产生的子连接会继承监听套接字的该容量。
    pub fn receive_capacity(&self) -> usize {
        self.receive_capacity
    }

    /// 返回发送环形缓冲区配置的物理总容量（字节）。
    pub fn send_capacity(&self) -> usize {
        self.send_capacity
    }

    /// 返回当前生效的接收窗口接纳预算（字节，对应 `SO_RCVBUF`）。
    pub fn receive_budget(&self) -> usize {
        self.receive_budget
    }

    /// 返回当前生效的发送缓冲区接纳预算（字节，对应 `SO_SNDBUF`）。
    pub fn send_budget(&self) -> usize {
        self.send_budget
    }

    /// 动态更新接收接纳预算（`1..=receive_capacity`）；若环形缓冲区已激活则同步通知底层协议栈调整通告窗口上限。
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

    /// 动态更新发送接纳预算（`1..=send_capacity`）；若环形缓冲区已激活则同步更新底层发送配额（不影响已入队的未确认字节）。
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

    /// 查询当前是否已启用 TCP Keep-Alive 空闲保活探测（对应 `SO_KEEPALIVE`）。
    pub fn keepalive(&self) -> bool {
        self.socket.keep_alive().is_some()
    }

    /// 启用或关闭 TCP Keep-Alive 保活探测；启用时空闲探测周期固定设为 [`TCP_KEEPALIVE_IDLE`]（7200 秒）。
    pub fn set_keepalive(&mut self, enabled: bool) {
        self.socket
            .set_keep_alive(enabled.then_some(TCP_KEEPALIVE_IDLE));
    }

    /// 在路由确定出站接口后更新该连接的路径 IP MTU（必须在 `68..=65535` 字节范围内），供协议栈计算有效 MSS。
    pub fn set_ip_mtu(&mut self, mtu: usize) {
        assert!(
            (IPV4_MIN_MTU..=MAX_IP_PACKET_LEN).contains(&mtu),
            "invalid TCP route MTU"
        );
        self.context.set_ip_mtu(mtu);
    }

    /// 查询当前连接的有效最大分段大小（Effective MSS，字节）；未建立连接时根据传入的套接字 `family` 扣除对应 IP 首部开销估算。
    pub fn effective_mss(&self, family: AddressFamily) -> usize {
        let version = match family {
            AddressFamily::Ipv4 => IpVersion::Ipv4,
            AddressFamily::Ipv6 => IpVersion::Ipv6,
        };
        self.socket.effective_mss(&self.context, version)
    }

    /// 返回当前连接正在使用的发送拥塞控制算法（如 `Reno`）。
    pub fn congestion_control(&self) -> tcp::CongestionControl {
        self.socket.congestion_control()
    }

    /// 查询是否启用了 Nagle 小包合并算法（与用户态 `TCP_NODELAY` 语义相反：`nagle_enabled == !tcp_nodelay`）。
    pub fn nagle_enabled(&self) -> bool {
        self.socket.nagle_enabled()
    }

    /// 启用或禁用 Nagle 算法；调用方需持有连接锁，状态变更将在后续 `dispatch` 发送时生效。
    pub fn set_nagle_enabled(&mut self, enabled: bool) {
        self.socket.set_nagle_enabled(enabled);
    }

    /// 返回接收环形缓冲区中已按序重组完毕、可供用户态立即读取的字节数（对应 `FIONREAD`）。
    pub fn receive_queue(&self) -> usize {
        self.socket.recv_queue()
    }

    /// 返回发送环形缓冲区中已写入但尚未收到对端 `ACK` 确认释放的字节数。
    pub fn send_queue(&self) -> usize {
        self.socket.send_queue()
    }

    /// 判断接收缓冲区中当前是否存在可供用户态立即读取的数据字节（不含对端 `FIN` 产生的 EOF 条件）。
    pub fn can_receive(&self) -> bool {
        self.socket.can_recv()
    }

    /// 判断当前连接状态是否允许发送，且发送缓冲区在当前 `send_budget` 配额下仍有剩余可写空间。
    pub fn can_send(&self) -> bool {
        self.socket.can_send()
    }

    /// 判断是否已收到对端发来的 `FIN` 段（即处于 `CloseWait`、`LastAck`、`Closing` 或 `TimeWait` 状态，读方向到达流末尾）。
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

/// 将缓冲区拷贝结果归约为统一的 [`IoProgress`]：
/// 若已成功传输了至少 1 个字节（Partial Progress），优先向调用方报告已完成字节数 `Ready(copied)`；
/// 若一字节未传且无错误则报告 `Blocked`，若一字节未传且发生错误（如 `EFAULT`）则向上返回该错误。
fn copy_progress(copied: usize, result: Result<usize, FsError>) -> Result<IoProgress, FsError> {
    if 0 < copied {
        return Ok(IoProgress::Ready(copied));
    }
    result.map(|_| IoProgress::Blocked)
}

/// 将内核单调时钟纳秒时间点折算为 `smoltcp` 使用的微秒级 `Instant`（`u64::MAX / 1000` 远小于 `i64::MAX`，转换不会溢出）。
fn raw_time(now: MonotonicInstant) -> Instant {
    Instant::from_micros((now.as_nanos() / NSEC_PER_USEC) as i64)
}
