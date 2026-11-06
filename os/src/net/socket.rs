//! 面向 VFS 与系统调用层的统一套接字打开文件描述（Open File Description，[`SocketFile`]）。
//!
//! ## 模块职责与架构定位
//! 本模块是文件系统抽象（[`FileLike`]、[`Pollable`]）与底层传输协议实现（[`TcpEndpoint`]、[`UdpEndpoint`]、[`UnixEndpoint`]）之间的桥接层：
//! - 上游面向系统调用层（`socket`、`socketpair`、`bind`、`listen`、`accept4`、`connect`、`sendto`、`recvfrom`、`setsockopt`、`getsockopt`）与通用文件 I/O（`read`、`write`、`poll`、`epoll`、`ioctl`、`fstat`）。
//! - 下游直接持有某一种具体协议端点（`Endpoint`）及所属网络域引用（[`NetStackRef`]），在无协议锁状态下完成用户空间缓冲区预读、超时截止时间计算与阻塞等待循环，再将就绪的数据块或控制指令交给协议端点处理。
//!
//! ## 核心设计与阻塞等待机制
//! ### 1. 打开文件描述与协议端点的分层结构
//! ```text
//! +-------------------------------------------------------------------+
//! |                    进程文件描述符表 (FdTable)                     |
//! |          fd 3 (CLOEXEC) ----+        +---- fd 4 (dup/fork)        |
//! +-----------------------------|--------|----------------------------+
//!                               v        v
//! +-------------------------------------------------------------------+
//! |                      SocketFile (共享 OFD)                        |
//! |  - flags: AtomicU32 (O_RDWR | O_NONBLOCK)                         |
//! |  - watches: Arc<WatchHead> (epoll 反向监听链表)                   |
//! |  - connect_unclaimed: AtomicBool (阻塞 connect 信号重入追踪)      |
//! |  - endpoint: Endpoint                                             |
//! +-----------------------------|-------------------------------------+
//!                               v
//!        +----------------------+----------------------+
//!        |                      |                      |
//!        v                      v                      v
//!  Arc<TcpEndpoint>       Arc<UdpEndpoint>      Arc<UnixEndpoint>
//! ```
//!
//! ### 2. 锁外用户态拷贝与两阶段提交
//! 为了避免在持有底层协议锁时触发用户态缺页异常（Page Fault）导致死锁或长时间阻塞：
//! - **流式发送（TCP / UNIX `SOCK_STREAM`）**：先在锁外从 `WriteBuffer` 提取最多 `STREAM_PREFIX_LEN`（4096 字节）到栈缓冲区，随后在阻塞等待循环中反复尝试向端点写入这批已拷贝的字节。
//! - **报文/记录发送（UDP / UNIX `SOCK_SEQPACKET`）**：先在锁外一次性分配并完整拷入整条消息负载（若中途遇到缺页错误则整包放弃，不向协议层提交残缺报文），在等待循环中复用同一份内核缓冲区直到协议层完整接纳。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. **OFD 层自身无互斥锁**：`SocketFile` 的自身状态（`flags`、`connect_unclaimed`、`generic_keepalive`）全部使用原子变量维护，不引入覆盖协议端点的外层睡眠锁或自旋锁。
//! 2. **等待队列注册与睡眠在协议锁外进行**：当协议端点返回 `IoProgress::Blocked` 时，调用线程在协议锁外通过 `prepare_wait` 将 `PollContext` 注册到端点的 `PollWaitHead`，并在每次唤醒后先清除触发标志再重新查询协议状态，防止唤醒丢失（Lost Wakeup）。
//! 3. **最后一个 `SocketFile` 释放时的清理顺序**：`Drop` 先调用 `watches.release()` 同步注销所有挂接的 `epoll` 监听项，再调用底层端点的 `close()` 发起正常协议关闭，不阻塞等待远端挥手或后台异步任务完成。

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

/// 新建 TCP 套接字默认的接收与发送环形缓冲区容量（单位：字节，128 KiB）。
pub const TCP_BUFFER_LEN: usize = 128 * 1024;

/// 面向连接的字节流套接字类型常量（`SOCK_STREAM`）。
pub(crate) const SOCK_STREAM: u32 = 1;
/// 无连接的数据报套接字类型常量（`SOCK_DGRAM`）。
pub(crate) const SOCK_DGRAM: u32 = 2;
/// 面向连接、保留消息边界的有序分组套接字类型常量（`SOCK_SEQPACKET`）。
pub(crate) const SOCK_SEQPACKET: u32 = 5;
/// 从 `socket()` 的 `type` 参数中提取基础套接字类型的低位掩码。
pub(crate) const SOCK_TYPE_MASK: u32 = 0xf;
/// 系统调用允许的最大套接字类型编号上限。
pub(crate) const SOCK_MAX: u32 = 11;
/// 创建套接字时直接设置非阻塞模式的标志位（与 `O_NONBLOCK` 相同）。
pub(crate) const SOCK_NONBLOCK: u32 = O_NONBLOCK;
/// 创建套接字时设置执行时关闭（Close-on-Exec）的标志位（与 `O_CLOEXEC` 相同）。
pub(crate) const SOCK_CLOEXEC: u32 = O_CLOEXEC;

/// `shutdown` 操作常量：关闭读方向（不再接收数据）。
pub(crate) const SHUT_RD: u32 = 0;
/// `shutdown` 操作常量：关闭写方向（不再发送数据）。
pub(crate) const SHUT_WR: u32 = 1;
/// `shutdown` 操作常量：同时关闭读方向与写方向。
pub(crate) const SHUT_RDWR: u32 = 2;

/// IP 协议号：传输控制协议（TCP）。
pub(crate) const IPPROTO_TCP: i32 = 6;
/// IP 协议号：用户数据报协议（UDP）。
pub(crate) const IPPROTO_UDP: i32 = 17;
/// IP 协议号及套接字选项级别：IPv6 协议（`IPPROTO_IPV6`）。
pub(crate) const IPPROTO_IPV6: i32 = 41;
/// 系统调用接受的协议号数值上限。
pub(crate) const IPPROTO_MAX: i32 = 263;

/// 通用套接字选项级别（`SOL_SOCKET`）。
pub(crate) const SOL_SOCKET: i32 = 1;
/// 套接字选项：允许本地地址与端口复用（`SO_REUSEADDR`）。
pub(crate) const SO_REUSEADDR: i32 = 2;
/// 套接字选项：只读查询套接字类型（`SO_TYPE`）。
pub(crate) const SO_TYPE: i32 = 3;
/// 套接字选项：读取并清除挂起的异步错误（`SO_ERROR`）。
pub(crate) const SO_ERROR: i32 = 4;
/// 套接字选项：启用或查询周期性保活探测（`SO_KEEPALIVE`）。
pub(crate) const SO_KEEPALIVE: i32 = 9;
/// 套接字选项：获取或设置发送缓冲区大小预算（`SO_SNDBUF`）。
pub(crate) const SO_SNDBUF: i32 = 7;
/// 套接字选项：获取或设置接收缓冲区大小预算（`SO_RCVBUF`）。
pub(crate) const SO_RCVBUF: i32 = 8;
/// 套接字选项：获取或设置接收操作超时时间（`SO_RCVTIMEO`）。
pub(crate) const SO_RCVTIMEO: i32 = 20;
/// 套接字选项：获取或设置发送/连接操作超时时间（`SO_SNDTIMEO`）。
pub(crate) const SO_SNDTIMEO: i32 = 21;
/// 套接字选项：只读查询套接字是否处于监听状态（`SO_ACCEPTCONN`）。
pub(crate) const SO_ACCEPTCONN: i32 = 30;
/// 套接字选项：只读查询套接字使用的具体协议号（`SO_PROTOCOL`）。
pub(crate) const SO_PROTOCOL: i32 = 38;
/// 套接字选项：只读查询套接字所属的地址族（`SO_DOMAIN`）。
pub(crate) const SO_DOMAIN: i32 = 39;

/// 收发标志：窥视接收队列中的数据而不将其从缓冲区消费移除（`MSG_PEEK`）。
pub(crate) const MSG_PEEK: u32 = 0x2;
/// 收发标志：当数据包被截断时报告或返回报文截断标志（`MSG_TRUNC`）。
pub(crate) const MSG_TRUNC: u32 = 0x20;
/// 收发标志：仅将本次收发操作作为非阻塞执行（`MSG_DONTWAIT`）。
pub(crate) const MSG_DONTWAIT: u32 = 0x40;
/// 收发标志：向已关闭写方向的连接发送数据时不向调用进程发送 `SIGPIPE` 信号（`MSG_NOSIGNAL`）。
pub(crate) const MSG_NOSIGNAL: u32 = 0x4000;

/// 流式套接字单次在协议锁外从用户态预提取的最大字节块长度（4096 字节）。
pub(crate) const STREAM_PREFIX_LEN: usize = 4096;
/// 用户通过 `SO_SNDBUF` / `SO_RCVBUF` 传入的提示值限幅上限（65536 字节）。
const BUFFER_HINT_MAX: u32 = 65536;
/// 经倍增与限幅计算后的收发缓冲区最小有效字节预算（1024 字节）。
const BUFFER_BUDGET_MIN: usize = 1024;
/// 按 Linux ABI 惯例将用户设置的 `SO_SNDBUF` / `SO_RCVBUF` 提示值乘以 2 的倍增系数。
const BUFFER_BUDGET_MULTIPLIER: usize = 2;

/// 套接字端点接收与发送两个方向的阻塞等待超时配置（单位：纳秒）。
///
/// `None` 表示无限期等待直到事件就绪或被信号打断；`Some(0)` 表示超时时间已立即到期。
#[derive(Clone, Copy, Default)]
pub(crate) struct SocketTimeouts {
    /// 接收方向操作（`receive`、`accept`）的超时时间（单位：纳秒）。
    pub(crate) receive: Option<u64>,
    /// 发送方向操作（`send`、`connect`）的超时时间（单位：纳秒）。
    pub(crate) send: Option<u64>,
}

impl SocketTimeouts {
    /// 更新指定方向（`receive == true` 为接收，否则为发送）的超时配置，另一方向保持原值不变。
    pub(crate) fn set(&mut self, receive: bool, timeout: Option<u64>) {
        if receive {
            self.receive = timeout;
        } else {
            self.send = timeout;
        }
    }
}

/// [`SocketFile`] 内部持有的具体传输层协议端点枚举。
#[derive(Clone)]
enum Endpoint {
    /// 面向连接的 TCP 协议端点。
    Tcp(Arc<TcpEndpoint>),
    /// 无连接的 UDP 数据报协议端点。
    Udp(Arc<UdpEndpoint>),
    /// 匿名 UNIX 域套接字对的单侧端点。
    Unix(Arc<UnixEndpoint>),
}

impl Endpoint {
    /// 在 [`SocketFile`] 构造中途因内存不足失败时，立刻撤销并回滚尚未对外发布的底层协议端点。
    fn abandon(&self) {
        match self {
            Self::Tcp(endpoint) => endpoint.abandon(),
            Self::Udp(endpoint) => endpoint.close(),
            Self::Unix(endpoint) => endpoint.close(),
        }
    }

    /// 当持有该端点的最后一个 [`SocketFile`] 被释放时，通知底层协议端点发起正常关闭流程。
    fn close(&self) {
        match self {
            Self::Tcp(endpoint) => endpoint.close(),
            Self::Udp(endpoint) => endpoint.close(),
            Self::Unix(endpoint) => endpoint.close(),
        }
    }
}

/// 套接字的打开文件描述（Open File Description），实现 [`FileLike`] 与 [`Pollable`] 接口。
///
/// 多个进程文件描述符（通过 `dup`、`dup2` 或 `fork` 复制）可共享同一个 `Arc<SocketFile>`。
pub struct SocketFile {
    /// 底层具体的传输协议端点实例（TCP、UDP 或 UNIX）。
    endpoint: Endpoint,
    /// 该套接字所属的网络命名空间/协议栈实例引用。
    net: NetStackRef,
    /// 创建该套接字时的有效用户 ID（用于 `fstat` 报告 `st_uid`）。
    uid: u32,
    /// 创建该套接字时的有效组 ID（用于 `fstat` 报告 `st_gid`）。
    gid: u32,
    /// 打开文件状态标志（固定包含 `O_RDWR`，可变位为 `O_NONBLOCK`）。
    flags: AtomicU32,
    /// 挂接在此打开文件描述上的 `epoll` 反向监听链表头。
    watches: Arc<WatchHead>,
    /// 标记是否存在已发起但因信号中断或超时而尚未被调用者领取结果的阻塞式 TCP `connect`。
    connect_unclaimed: AtomicBool,
    /// 针对 UDP 与 UNIX 等无保活定时器协议保存的通用 `SO_KEEPALIVE` 开关状态。
    generic_keepalive: AtomicBool,
    /// 标记 `SocketFile` 是否已完成初始化并正式接管底层端点的关闭责任。
    opened: bool,
}

impl SocketFile {
    /// 在指定网络域中创建默认缓冲区大小（`TCP_BUFFER_LEN`）的 TCP 端点，并封装为新的 [`SocketFile`]（`O_CLOEXEC` 由调用方写入描述符表槽）。
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

    /// 在指定网络域中创建 UDP 端点，并封装为新的 [`SocketFile`]。
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

    /// 在指定网络域中创建一对相互连接的匿名 UNIX 套接字，并返回两侧各自的 [`SocketFile`]；若任一侧分配失败则自动关闭另一侧。
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

    /// 将已存在的 [`TcpEndpoint`]（例如 `accept` 吐出的已建立连接子端点）封装为新的 [`SocketFile`]。
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

    /// 分配 `WatchHead` 与 [`SocketFile`] 实例；若内存分配失败则调用 `endpoint.abandon()` 清理未发布的协议端点。
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

    /// 若当前套接字为 TCP 类型则返回底层 [`TcpEndpoint`] 引用，否则返回 `FsError::Unsupported`。
    pub fn tcp_endpoint(&self) -> Result<&Arc<TcpEndpoint>, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => Ok(endpoint),
            _ => Err(FsError::Unsupported),
        }
    }

    /// 返回当前套接字的传输类型常量（`SOCK_STREAM`、`SOCK_DGRAM` 或 `SOCK_SEQPACKET`）。
    pub fn socket_type(&self) -> u32 {
        match &self.endpoint {
            Endpoint::Udp(_) => SOCK_DGRAM,
            Endpoint::Tcp(_) => SOCK_STREAM,
            Endpoint::Unix(endpoint) => endpoint.socket_type() as u32,
        }
    }

    /// 判断当前套接字是否为保留消息边界的报文/记录类型（UDP 或 UNIX `SOCK_SEQPACKET`）。
    fn is_message_oriented(&self) -> bool {
        match &self.endpoint {
            Endpoint::Udp(_) => true,
            Endpoint::Unix(endpoint) => endpoint.socket_type() == UnixSocketType::SeqPacket,
            Endpoint::Tcp(_) => false,
        }
    }

    /// 返回当前套接字所属的地址族编号（`AF_INET`、`AF_INET6` 或 `AF_UNIX`）。
    pub fn domain(&self) -> u16 {
        match &self.endpoint {
            Endpoint::Unix(_) => AF_UNIX,
            Endpoint::Tcp(endpoint) => endpoint.family().number(),
            Endpoint::Udp(endpoint) => endpoint.family().number(),
        }
    }

    /// 查询 IPv6 套接字的 `IPV6_V6ONLY` 策略状态；UNIX 套接字返回 `FsError::Unsupported`。
    pub fn v6_only(&self) -> Result<bool, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => Ok(endpoint.v6_only()),
            Endpoint::Udp(endpoint) => Ok(endpoint.v6_only()),
            Endpoint::Unix(_) => Err(FsError::Unsupported),
        }
    }

    /// 设置 IPv6 套接字的 `IPV6_V6ONLY` 策略（仅在尚未绑定端口前允许修改）。
    pub fn set_v6_only(&self, enabled: bool) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_v6_only(enabled),
            Endpoint::Udp(endpoint) => endpoint.set_v6_only(enabled),
            Endpoint::Unix(_) => Err(FsError::Unsupported),
        }
    }

    /// 返回当前套接字的协议编号（`IPPROTO_TCP`、`IPPROTO_UDP`，或 UNIX 域默认的 `0`）。
    pub fn protocol(&self) -> i32 {
        match &self.endpoint {
            Endpoint::Tcp(_) => IPPROTO_TCP,
            Endpoint::Udp(_) => IPPROTO_UDP,
            Endpoint::Unix(_) => 0,
        }
    }

    /// 返回底层协议端点在所属网络域内的全局唯一 ID。
    pub fn id(&self) -> u64 {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.id(),
            Endpoint::Udp(endpoint) => endpoint.id(),
            Endpoint::Unix(endpoint) => endpoint.id(),
        }
    }

    /// 读取当前打开文件描述的状态标志（包含 `O_RDWR` 及可选的 `O_NONBLOCK`）。
    pub fn flags(&self) -> u32 {
        self.flags.load(Ordering::Acquire)
    }

    /// 原子更新打开文件描述的可变状态标志（如 `fcntl(F_SETFL)` 修改 `O_NONBLOCK`）。
    pub fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        crate::fs::set_status_flags(&self.flags, raw)
    }

    /// 将用户传入的套接字地址正规化（校验地址族、转换 IPv4-mapped IPv6 地址及作用域）后绑定到底层 TCP 或 UDP 端点。
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

    /// 将当前 TCP 套接字转换为监听状态或更新其全连接队列上限（`backlog`）；非 TCP 套接字返回 `FsError::Unsupported`。
    pub fn listen(&self, backlog: usize) -> Result<(), FsError> {
        self.tcp_endpoint()?.listen(backlog)
    }

    /// 发起套接字连接或更新 UDP 默认对端地址。
    ///
    /// - 对于 UDP：设置或清除默认发送与接收过滤的对端地址并立即返回。
    /// - 对于 TCP：非阻塞模式下发起握手后返回 `FsError::InProgress`；阻塞模式下等待三次握手完成，若中途被信号打断，后续重入 `connect` 时通过 `connect_unclaimed` 标志继续等待并领取原握手结果。
    pub fn connect(&self, peer: SocketAddress, caller: FileIoContext<'_>) -> Result<(), FsError> {
        // Step 1: 区分协议类型；UDP 直接正规化地址并更新默认对端。
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

        // Step 2: 计算发送方向的超时截止时间，并发起或重入 TCP 三次握手。
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

        // Step 3: 循环检查握手完成状态，未完成时注册等待队列并按截止时间挂起当前任务。
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

    /// 从监听状态的 TCP 套接字取出一项已完成三次握手的子连接，并为其创建新的 [`SocketFile`]。
    ///
    /// 若当前无就绪子连接，非阻塞套接字立即返回 `FsError::WouldBlock`，阻塞套接字则在接收超时截止时间前循环等待唤醒。
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

    /// 从套接字接收数据（支持 `MSG_PEEK`、`MSG_DONTWAIT` 以及面向消息套接字的 `MSG_TRUNC`）。
    ///
    /// 入口处一次性计算接收超时截止时间；在等待循环中优先尝试从底层端点读取数据，无数据且非阻塞时返回 `FsError::WouldBlock`，否则注册等待队列挂起任务。
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

    /// 执行单次非阻塞底层读取尝试；对于字节流套接字，若在已拷入部分字节后遇到错误，优先返回已成功拷贝的字节数。
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

    /// 向套接字发送数据（支持可选目的地址 `destination`、`MSG_DONTWAIT` 与 `MSG_NOSIGNAL`）。
    ///
    /// 在协议锁外完成用户态数据的预提取，并在阻塞等待循环中复用同一份内核缓冲与入口截止时间；若流或有序记录连接已断开且未指定 `MSG_NOSIGNAL`，则向调用进程发送 `SIGPIPE` 信号。
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
        // Step 1: 采样发送超时截止时间，并将 UDP 数据报与 UNIX 有序记录分流到各自的整包发送路径。
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

        // Step 2: 针对字节流（TCP / UNIX Stream），在锁外从用户缓冲区预拉取最多 4096 字节前缀。
        let mut prefix = [0u8; STREAM_PREFIX_LEN];
        let prepared = source.pull(&mut prefix)?;
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;

        // Step 3: 循环尝试将预提取的前缀推入底层端点，空间不足时注册等待队列并睡眠。
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            // 即使预提取长度为 0，也必须调用底层端点以检查写方向是否已关闭并触发 EPIPE。
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

    /// 发送单条完整的 UDP 数据报：在锁外一次性分配并拷入全部负载，若发送队列暂满则在等待循环中持同一份负载重试。
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
            // WriteBuffer 内部保存了首次用户态拷贝错误；再次拉取空切片以取出并返回该错误，避免提交残缺数据报。
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

    /// 发送单条完整的 UNIX `SOCK_SEQPACKET` 记录：在锁外一次性拷入 `Vec<u8>`，仅当对端接收队列有足够配额时才移交所有权。
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
            // WriteBuffer 内部保存了首次用户态拷贝错误；再次拉取空切片以返回该错误，避免提交不完整的记录。
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

    /// 按指定方式（`SHUT_RD`、`SHUT_WR` 或 `SHUT_RDWR`）关闭底层协议端点的读方向和/或写方向。
    pub fn shutdown(&self, how: u32) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.shutdown(how),
            Endpoint::Udp(endpoint) => endpoint.shutdown(how),
            Endpoint::Unix(endpoint) => endpoint.shutdown(how),
        }
    }

    /// 查询套接字当前绑定的本地地址（`getsockname`）；匿名 UNIX 套接字返回 `SocketAddress::UnixUnnamed`。
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

    /// 查询套接字当前连接的远端对端地址（`getpeername`）；若尚未建立连接则由底层端点返回 `FsError::NotConnected`。
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

    /// 取出并清除底层协议端点记录的异步错误（`getsockopt(SO_ERROR)`）。
    pub fn take_error(&self) -> Option<FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.take_error(),
            Endpoint::Udp(endpoint) => endpoint.take_error(),
            Endpoint::Unix(endpoint) => endpoint.take_error(),
        }
    }

    /// 查询当前套接字是否处于监听连接状态（`getsockopt(SO_ACCEPTCONN)`）。
    pub fn is_listener(&self) -> bool {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.is_listener(),
            Endpoint::Udp(_) | Endpoint::Unix(_) => false,
        }
    }

    /// 查询当前生效的接收缓冲区字节预算（`getsockopt(SO_RCVBUF)`）。
    pub fn receive_budget(&self) -> usize {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.receive_budget(),
            Endpoint::Udp(endpoint) => endpoint.receive_budget(),
            Endpoint::Unix(endpoint) => endpoint.receive_capacity(),
        }
    }

    /// 查询当前生效的发送缓冲区字节预算（`getsockopt(SO_SNDBUF)`）。
    pub fn send_budget(&self) -> usize {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.send_budget(),
            Endpoint::Udp(endpoint) => endpoint.send_budget(),
            Endpoint::Unix(endpoint) => endpoint.send_capacity(),
        }
    }

    /// 将用户传入的缓冲区大小提示值按无符号上限截断并乘以 2，再夹紧在 `[BUFFER_BUDGET_MIN, capacity]` 范围内。
    fn buffer_budget(hint: i32, capacity: usize) -> usize {
        ((hint as u32).min(BUFFER_HINT_MAX) as usize * BUFFER_BUDGET_MULTIPLIER)
            .max(BUFFER_BUDGET_MIN)
            .min(capacity)
    }

    /// 设置 TCP 或 UDP 端点的接收缓冲区有效预算（`setsockopt(SO_RCVBUF)`），不丢弃已入队的数据。
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

    /// 设置 TCP 或 UDP 端点的发送缓冲区有效预算（`setsockopt(SO_SNDBUF)`），容量扩大时的写就绪唤醒由端点在锁外完成。
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

    /// 查询 `SO_KEEPALIVE` 选项状态；TCP 返回实际保活定时器开关，UDP 和 UNIX 返回保存在 `SocketFile` 上的布尔标志。
    pub fn keepalive(&self) -> bool {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.keepalive(),
            Endpoint::Udp(_) | Endpoint::Unix(_) => self.generic_keepalive.load(Ordering::Acquire),
        }
    }

    /// 设置 `SO_KEEPALIVE` 选项状态；TCP 启动或停止 7200 秒保活探测定时器，UDP 和 UNIX 仅记录该标志。
    pub fn set_keepalive(&self, enabled: bool) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_keepalive(enabled),
            Endpoint::Udp(_) | Endpoint::Unix(_) => {
                self.generic_keepalive.store(enabled, Ordering::Release);
                Ok(())
            }
        }
    }

    /// 读取底层端点保存的接收（`receive == true`）或发送（`receive == false`）超时时间（单位：纳秒）。
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

    /// 更新底层端点保存的接收或发送超时时间（单位：纳秒），供同一端点的所有打开描述符共享。
    pub(crate) fn set_timeout(&self, receive: bool, timeout: Option<u64>) {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_timeout(receive, timeout),
            Endpoint::Udp(endpoint) => endpoint.set_timeout(receive, timeout),
            Endpoint::Unix(endpoint) => endpoint.set_timeout(receive, timeout),
        }
    }

    /// 根据当前单调时钟与端点的相对超时配置计算本次操作的绝对截止时间（`MonotonicInstant`）。
    fn deadline(&self, receive: bool) -> Result<Option<MonotonicInstant>, FsError> {
        self.timeout(receive)
            .map(|nanos| {
                time::now()
                    .checked_add(Duration::from_nanos(nanos))
                    .ok_or(FsError::InvalidOperation)
            })
            .transpose()
    }

    /// 在操作无进展且需要进入睡眠前，依次检查绝对截止时间是否已到期以及上一轮睡眠是否被信号中断。
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

    /// 查询 TCP 端点当前生效的最大报文段长度（`TCP_MAXSEG`）。
    pub fn effective_mss(&self) -> Result<usize, FsError> {
        Ok(self.tcp_endpoint()?.effective_mss())
    }

    /// 查询 TCP 端点当前使用的拥塞控制算法（`TCP_CONGESTION`）。
    pub fn congestion_control(&self) -> Result<CongestionControl, FsError> {
        Ok(self.tcp_endpoint()?.congestion_control())
    }

    /// 设置 TCP 拥塞控制算法名称（当前仅支持已启用的 `"reno"`，其他名称返回 `FsError::NotFound`）。
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

    /// 查询底层协议端点保存的 `SO_REUSEADDR` 地址复用标志。
    pub fn reuse_address(&self) -> bool {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.reuse_address(),
            Endpoint::Udp(endpoint) => endpoint.reuse_address(),
            Endpoint::Unix(endpoint) => endpoint.reuse_address(),
        }
    }

    /// 更新底层协议端点的 `SO_REUSEADDR` 地址复用策略，并同步到所属网络域的端口绑定表。
    pub fn set_reuse_address(&self, reuse: bool) -> Result<(), FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.set_reuse_address(reuse),
            Endpoint::Udp(endpoint) => endpoint.set_reuse_address(reuse),
            Endpoint::Unix(endpoint) => endpoint.set_reuse_address(reuse),
        }
    }

    /// 查询 TCP 端点的 `TCP_NODELAY`（禁用 Nagle 算法）开关状态。
    pub fn nodelay(&self) -> Result<bool, FsError> {
        self.tcp_endpoint()?.nodelay()
    }

    /// 设置 TCP 端点的 `TCP_NODELAY`（禁用 Nagle 算法）开关状态。
    pub fn set_nodelay(&self, enabled: bool) -> Result<(), FsError> {
        self.tcp_endpoint()?.set_nodelay(enabled)
    }

    /// 创建单次阻塞操作使用的 `PollContext` 与容量为 1 的 `PollTable`，并将当前套接字的等待队列头注册进去。
    fn prepare_wait(&self) -> Result<(Arc<PollContext>, PollTable), FsError> {
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let mut table = PollTable::new(sink, 1)?;
        self.poll(Some(&mut table))?;
        Ok((context, table))
    }

    /// 判断本次 I/O 操作是否应以非阻塞方式执行（综合检查文件状态标志 `O_NONBLOCK` 与单次调用标志 `MSG_DONTWAIT`）。
    fn nonblocking(&self, flags: u32) -> bool {
        self.flags() & O_NONBLOCK != 0 || flags & MSG_DONTWAIT != 0
    }

    /// 构造面向字节流读取操作的返回结果（已拷贝字节数等于原始长度，无源地址与特殊标志）。
    fn stream_result(count: usize) -> MessageResult {
        MessageResult {
            copied: count,
            original_len: count,
            source: None,
            flags: 0,
        }
    }

    /// 处理流式读取中途遇到的错误：若此前已成功向用户缓冲区拷入至少 1 个字节（`count > 0`），则优先返回已拷贝字节数，否则返回原错误。
    fn progress(count: usize, error: FsError) -> Result<usize, FsError> {
        if count == 0 {
            Err(error)
        } else {
            Ok(count)
        }
    }

    /// 构造匿名套接字的 `FileStat` 元数据（使用端点唯一 ID 作为 `inode`，文件类型为 `S_IFSOCK | 0777`，块大小取回环接口 MTU）。
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
    /// 向 `PollTable` 注册底层端点的事件等待队列头，并查询当前就绪的 `PollMask`。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        match &self.endpoint {
            Endpoint::Tcp(endpoint) => endpoint.poll(table),
            Endpoint::Udp(endpoint) => endpoint.poll(table),
            Endpoint::Unix(endpoint) => endpoint.poll(table),
        }
    }
}

impl FileLike for SocketFile {
    /// 通过通用文件 `read` 接口从套接字读取数据；当目标缓冲区长度为 0 时直接返回 `Ok(0)` 而不消费零长数据报。
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

    /// 套接字不支持按偏移量随机读取（`pread`），固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 匿名套接字不关联文件系统目录树路径，返回 `None`。
    fn path(&self) -> Option<&Path> {
        None
    }

    /// 套接字不支持页缓存内存映射（`mmap`），返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 返回自身的 [`Pollable`] 特征对象引用，供 `poll` / `select` / `epoll` 监听就绪事件。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    /// 返回 [`SocketFile`] 自身引用，供网络系统调用识别并提取底层套接字对象。
    fn socket(&self) -> Option<&SocketFile> {
        Some(self)
    }

    /// 返回此打开文件描述的 `epoll` 反向监听链表头引用。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        Some(&self.watches)
    }

    /// 将套接字上的 `ioctl` 请求转发给所属网络域的控制面（用于管理网络接口地址、启停标志与默认路由）。
    fn ioctl(
        &self,
        request: u32,
        argument: usize,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        crate::net::control::ioctl(&self.net, request, argument, caller)
    }

    /// 读取当前打开文件描述的状态标志（`O_RDWR` 及可选的 `O_NONBLOCK`）。
    fn flags(&self) -> u32 {
        SocketFile::flags(self)
    }

    /// 更新当前打开文件描述的可变状态标志（对应 `fcntl(F_SETFL)`）。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        SocketFile::set_status_flags(self, raw)
    }

    /// 原子设置或清除当前打开文件描述的 `O_NONBLOCK` 非阻塞标志（对应 `ioctl(FIONBIO)`）。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        crate::fs::set_nonblocking(&self.flags, enabled)
    }

    /// 套接字不支持文件长度截断（`ftruncate`），固定返回 `FsError::InvalidOperation`。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 套接字不支持磁盘数据同步（`fsync` / `fdatasync`），固定返回 `FsError::InvalidOperation`。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 套接字不支持文件偏移量定位（`lseek`），固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 返回当前套接字的 `FileStat` 属性信息（对应 `fstat`）。
    fn stat(&self) -> Result<FileStat, FsError> {
        Ok(SocketFile::stat(self))
    }

    /// 套接字不是目录，调用 `readdir` 固定返回 `FsError::NotDirectory`。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 套接字不是目录，调用 `getdents64` 遍历目录项固定返回 `FsError::NotDirectory`。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 通过通用文件 `write` 接口向套接字发送数据（使用已连接的默认对端地址，无额外消息标志）。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.send(source, None, 0, caller)
    }

    /// 套接字不支持按偏移量随机写入（`pwrite`），固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
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
    /// 当最后一个指向本 `SocketFile` 的 `Arc` 释放时，先注销全部 `epoll` 监听项，若已完成打开初始化则通知底层协议端点关闭。
    fn drop(&mut self) {
        self.watches.release();
        if self.opened {
            self.endpoint.close();
        }
    }
}
