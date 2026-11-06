//! 套接字（Socket）生命周期管理、连接建立、消息收发及选项配置系统调用实现。
//!
//! ## 模块职责与架构定位
//! - **上游调用者**：`crate::syscall` 主分派器，处理系统调用号 198..=212 及 242（`socket`、`socketpair`、`bind`、`listen`、`accept`、`accept4`、`connect`、`getsockname`、`getpeername`、`sendto`、`recvfrom`、`setsockopt`、`getsockopt`、`shutdown`、`sendmsg`、`recvmsg`）。
//! - **下游依赖**：
//!   - `crate::net`：提供底层 TCP/UDP/UNIX 套接字文件对象（`SocketFile`）、套接字地址编解码（`SocketAddress`）以及网络命名空间运行时。
//!   - `crate::fs`：提供进程文件描述符表（`reserve` / `publish` / `lookup`）与用户态 I/O 向量缓冲封装（`ReadBuffer` / `WriteBuffer`）。
//!   - `crate::syscall::error`：提供用户态内存拷贝（`copyin` / `copyout`）。
//!
//! ## 核心设计与关键机制
//!
//! ### 1. 文件描述符两阶段预留与发布（Reserve-then-Publish）
//! 在 `socket`、`socketpair` 与 `accept` 中，内核始终遵循“先预留槽位、再执行创建或拷贝、最后发布文件对象”的事务顺序：
//! ```text
//! runtime.files.reserve(nofile) ──► 构造 SocketFile / accept 子连接 ──► copyout (如 socketpair fd 数组 / accept 对端地址)
//!       │                                    │                                    │
//!       │                                    ▼ (若失败：自动释放槽位与未发布文件)   ▼ (全部成功)
//!       └────────────────────────────────────┴──────────────────────────► reservation.publish(file, flags)
//! ```
//! 这样可以保证：一旦向用户空间写回地址或描述符数组发生缺页错误（`EFAULT`），未发布的套接字与预留槽位会在作用域结束时自动析构归还，不会在进程文件表中留下半成品描述符。
//!
//! ### 2. LP64 `struct msghdr` 内存布局（共 56 字节）
//! ```text
//! 偏移(B)  大小(B)  字段名称        说明
//! 0..8     8        msg_name        可选对端地址缓冲区指针 (u64)
//! 8..12    4        msg_namelen     地址缓冲区字节长度 (i32 / socklen_t)
//! 12..16   4        (padding)       LP64 对齐填充
//! 16..24   8        msg_iov         scatter/gather I/O 向量数组指针 (u64)
//! 24..32   8        msg_iovlen      I/O 向量元素个数 (u64)
//! 32..40   8        msg_control     辅助控制数据指针 (u64)
//! 40..48   8        msg_controllen  辅助数据字节长度 (u64，当前不支持非零输入，recvmsg 写回 0)
//! 48..52   4        msg_flags       接收消息标志输出位 (u32，输入时忽略，recvmsg 写回偏移 48)
//! 52..56   4        (padding)       尾部对齐填充
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 进程文件描述符表自旋锁仅在 `lookup`、`reserve` 和 `publish` 内部短暂持有，用于克隆 `FileRef`（`Arc`）强引用或修改槽位状态；绝不在持有文件表锁期间执行网络协议操作或用户内存拷贝。
//! - 所有用户态参数（如 `sockaddr`、`msghdr`、`iovec`、`timeval` 选项）均先在锁外通过 `copyin` 复制到栈上缓冲区并完成校验，随后才调用 `SocketFile` 的协议方法，避免在持有套接字端点锁时触发用户页缺页睡眠。

use core::convert::{TryFrom, TryInto};

use crate::fs::{DescriptorFlags, FileIoContext, FileRef, ReadBuffer, UserIoVector, WriteBuffer};
use crate::mm::space::Mm;
use crate::net::address::{
    AddressFamily, SocketAddress, ADDRESS_FAMILY_LEN, AF_INET, AF_INET6, AF_UNIX, MAX_ADDRESS_LEN,
};
use crate::net::socket::{
    SocketFile, IPPROTO_IPV6, IPPROTO_MAX, IPPROTO_TCP, IPPROTO_UDP, MSG_TRUNC, SOCK_CLOEXEC,
    SOCK_DGRAM, SOCK_MAX, SOCK_NONBLOCK, SOCK_SEQPACKET, SOCK_STREAM, SOCK_TYPE_MASK, SOL_SOCKET,
    SO_ACCEPTCONN, SO_DOMAIN, SO_ERROR, SO_KEEPALIVE, SO_PROTOCOL, SO_RCVBUF, SO_RCVTIMEO,
    SO_REUSEADDR, SO_SNDBUF, SO_SNDTIMEO, SO_TYPE,
};
use crate::net::UnixSocketType;
use crate::syscall::error::{copyin, copyout, Errno, SysResult};
use crate::syscall::file::{import_vectors, nofile, MAX_IOV_COUNT};
use crate::syscall::{Outcome, Request, Restart};
use crate::task::UserThread;
use crate::time::{NSEC_PER_SEC, NSEC_PER_USEC, USEC_PER_SEC};
use crate::timer::TICK_NS;
use smoltcp::socket::tcp::CongestionControl;

const SOCKET_FLAGS: u32 = SOCK_NONBLOCK | SOCK_CLOEXEC;

const TCP_NODELAY: i32 = 1;
const TCP_MAXSEG: i32 = 2;
const TCP_CONGESTION: i32 = 13;
const IPV6_V6ONLY: i32 = 26;

const CONGESTION_NAME_LEN: usize = 16;
const ADDRESS_STORAGE_LEN: usize = 128;
const TIMEVAL_LEN: usize = 16;

/// 网络套接字相关系统调用的统一分派入口。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。
/// - **信号中断与重启策略**：当底层无超时（无限等待）的阻塞套接字操作被信号中断且允许自动重启时，下层返回内部错误码 `Errno::RestartSyscall`，此处将其转换为 `Outcome::Restart(Restart::Syscall { request })`；若套接字设置了接收/发送超时（有限等待）并被信号打断，则直接向用户态返回 `EINTR`。
pub(super) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    let result = match request.number {
        198 => socket(user, raw),
        199 => socketpair(user, raw),
        200 => bind(user, raw),
        201 => listen(user, raw),
        202 => accept(user, raw, 0),
        203 => connect(user, raw),
        204 => name(user, raw, false),
        205 => name(user, raw, true),
        206 => sendto(user, raw),
        207 => recvfrom(user, raw),
        208 => setsockopt(user, raw),
        209 => getsockopt(user, raw),
        210 => shutdown(user, raw),
        211 => sendmsg(user, raw),
        212 => recvmsg(user, raw),
        242 => accept(user, raw, raw[3] as u32),
        _ => Err(Errno::NoSyscall),
    };

    match result {
        Err(Errno::RestartSyscall) => Outcome::Restart(Restart::Syscall { request }),
        other => Outcome::Return(other.unwrap_or_else(Errno::result)),
    }
}

/// 将系统调用原始寄存器值按 32 位有符号整数（ABI `int`）解析为文件描述符索引；若为负数则返回 `Err(Errno::BadFd)`。
fn descriptor(raw: usize) -> Result<usize, Errno> {
    let fd = raw as i32;
    if fd < 0 {
        Err(Errno::BadFd)
    } else {
        Ok(fd as usize)
    }
}

/// 从当前进程的文件描述符表中查找 `raw` 对应的打开文件对象，并校验其是否为套接字。
///
/// 文件描述符表自旋锁仅在 `lookup` 克隆 `FileRef` 强引用期间短暂持有；套接字能力检查（`file.socket()`）在锁外完成，非套接字文件返回 `Err(Errno::NotSocket)`。
fn socket_file(user: &UserThread, raw: usize) -> Result<FileRef, Errno> {
    let runtime = user.runtime().expect("socket syscall has no runtime");
    let file = runtime.files.lookup(descriptor(raw)?)?;
    if file.socket().is_none() {
        return Err(Errno::NotSocket);
    }
    Ok(file)
}

/// `socket` 系统调用：校验地址族、套接字类型、协议号及创建标志，创建 TCP 或 UDP 套接字并安装到当前进程的文件描述符表。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。
/// - **参数**：`raw[0]` 为地址族（`AF_INET` / `AF_INET6`），`raw[1]` 为类型与标志位或组合（`SOCK_STREAM` / `SOCK_DGRAM` | `SOCK_NONBLOCK` | `SOCK_CLOEXEC`），`raw[2]` 为协议号。
/// - **返回值**：成功返回新分配的文件描述符编号；参数非法或资源不足时返回对应 `Errno`。`SOCK_NONBLOCK` 保存于底层打开文件对象（OFD），而 `SOCK_CLOEXEC` 保存于文件描述符表槽位。
fn socket(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 拆分套接字类型与附加标志位（SOCK_NONBLOCK / SOCK_CLOEXEC），校验地址族、类型与协议组合。
    let family = raw[0] as i32;
    let kind = raw[1] as u32;
    let protocol = raw[2] as i32;
    let flags = kind & !SOCK_TYPE_MASK;
    let kind = kind & SOCK_TYPE_MASK;
    if flags & !SOCKET_FLAGS != 0 {
        return Err(Errno::InvalidArgument);
    }
    let family = match family {
        value if value == i32::from(AF_INET) => AddressFamily::Ipv4,
        value if value == i32::from(AF_INET6) => AddressFamily::Ipv6,
        _ => return Err(Errno::AddressFamilyNotSupported),
    };
    if SOCK_MAX <= kind {
        return Err(Errno::InvalidArgument);
    }
    if !(0..IPPROTO_MAX).contains(&protocol) {
        return Err(Errno::InvalidArgument);
    }
    match kind {
        SOCK_STREAM if protocol == 0 || protocol == IPPROTO_TCP => {}
        SOCK_DGRAM if protocol == 0 || protocol == IPPROTO_UDP => {}
        SOCK_STREAM | SOCK_DGRAM => return Err(Errno::ProtocolNotSupported),
        _ => return Err(Errno::SocketTypeNotSupported),
    }

    // Step 2: 获取网络命名空间并预先在文件描述符表中保留空闲槽位，随后创建底层 TCP/UDP 套接字对象并发布。
    let runtime = user.runtime().expect("socket has no runtime");
    let net = runtime.net.ok_or(Errno::NetworkUnreachable)?;
    let reservation = runtime.files.reserve(nofile(user))?;
    let file: FileRef = if kind == SOCK_STREAM {
        SocketFile::try_new_tcp(
            net,
            family,
            flags,
            runtime.credentials.uid,
            runtime.credentials.gid,
        )?
    } else {
        SocketFile::try_new_udp(
            net,
            family,
            flags,
            runtime.credentials.uid,
            runtime.credentials.gid,
        )?
    };

    Ok(reservation.publish(
        file,
        DescriptorFlags {
            close_on_exec: flags & SOCK_CLOEXEC != 0,
        },
    ) as isize)
}

/// `socketpair` 系统调用：创建一对已相互连接的匿名 UNIX 域套接字（支持 `SOCK_STREAM` 与 `SOCK_SEQPACKET`），并将两个文件描述符写入用户空间数组。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。
/// - **参数**：`raw[0]` 为地址族（仅 `AF_UNIX` 实际支持配对，`AF_INET`/`AF_INET6` 在参数校验通过后返回 `EOPNOTSUPP`），`raw[1]` 为类型与标志，`raw[2]` 为协议号（必须为 0），`raw[3]` 为用户空间 `int sv[2]` 数组地址。
/// - **返回值**：成功返回 `Ok(0)`。两个描述符槽位预先保留，只有在套接字对创建成功且 8 字节 fd 数组成功拷贝至用户空间后，才正式发布到描述符表。
fn socketpair(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 校验附加标志位、协议范围以及地址族与套接字类型的合法组合。
    let family = raw[0] as i32;
    let kind = raw[1] as u32;
    let protocol = raw[2] as i32;
    let flags = kind & !SOCK_TYPE_MASK;
    let kind = kind & SOCK_TYPE_MASK;
    if flags & !SOCKET_FLAGS != 0 || SOCK_MAX <= kind {
        return Err(Errno::InvalidArgument);
    }
    if !(0..IPPROTO_MAX).contains(&protocol) {
        return Err(Errno::InvalidArgument);
    }
    if family == i32::from(AF_INET) || family == i32::from(AF_INET6) {
        match kind {
            SOCK_STREAM if protocol == 0 || protocol == IPPROTO_TCP => {}
            SOCK_DGRAM if protocol == 0 || protocol == IPPROTO_UDP => {}
            SOCK_STREAM | SOCK_DGRAM => return Err(Errno::ProtocolNotSupported),
            _ => return Err(Errno::SocketTypeNotSupported),
        }
        return Err(Errno::Unsupported);
    }
    if family != i32::from(AF_UNIX) {
        return Err(Errno::AddressFamilyNotSupported);
    }
    let kind = match kind {
        SOCK_STREAM => UnixSocketType::Stream,
        SOCK_SEQPACKET => UnixSocketType::SeqPacket,
        _ => return Err(Errno::SocketTypeNotSupported),
    };
    if protocol != 0 {
        return Err(Errno::ProtocolNotSupported);
    }

    // Step 2: 预留两个文件描述符槽位并构造一对相连的 UNIX 套接字文件对象。
    let runtime = user.runtime().expect("socketpair has no runtime");
    let net = runtime.net.ok_or(Errno::NetworkUnreachable)?;
    let first_slot = runtime.files.reserve(nofile(user))?;
    let second_slot = runtime.files.reserve(nofile(user))?;
    let [first, second] = SocketFile::try_new_unix_pair(
        net,
        kind,
        flags,
        runtime.credentials.uid,
        runtime.credentials.gid,
    )?;

    // Step 3: 先将两个 fd 编号拷贝至用户空间，成功后再正式发布到文件描述符表，确保 copyout 失败时无描述符泄漏。
    let mut bytes = [0u8; 8];
    bytes[..4].copy_from_slice(&(first_slot.fd() as u32).to_ne_bytes());
    bytes[4..].copy_from_slice(&(second_slot.fd() as u32).to_ne_bytes());
    let mm = user.mm().expect("socketpair has no MM");
    copyout(&mm, raw[3], &bytes)?;

    let descriptor_flags = DescriptorFlags {
        close_on_exec: flags & SOCK_CLOEXEC != 0,
    };
    first_slot.publish(first, descriptor_flags);
    second_slot.publish(second, descriptor_flags);
    Ok(0)
}

/// 从用户空间读取 `sockaddr` 字节并解码为内核 `SocketAddress`。
///
/// 要求长度 `raw_length` 介于 `ADDRESS_FAMILY_LEN`（2 字节）与 `ADDRESS_STORAGE_LEN`（128 字节）之间；支持 IPv4 与 IPv6 地址格式及作用域 ID（Scope ID）校验。
fn input_address(mm: &Mm, address: usize, raw_length: usize) -> Result<SocketAddress, Errno> {
    let length = raw_length as i32;
    if length < ADDRESS_FAMILY_LEN as i32 || (ADDRESS_STORAGE_LEN as i32) < length {
        return Err(Errno::InvalidArgument);
    }

    let mut bytes = [0u8; ADDRESS_STORAGE_LEN];
    copyin(mm, &mut bytes[..length as usize], address)?;
    Ok(SocketAddress::decode(&bytes[..length as usize])?)
}

/// 从用户空间读取 4 字节有符号 `socklen_t` 缓冲区容量。
///
/// 若长度值为负数则返回 `Err(Errno::InvalidArgument)`；即使容量为 0，也要求 `address` 指向可读的 4 字节用户内存。
fn capacity(mm: &Mm, address: usize) -> Result<usize, Errno> {
    let mut bytes = [0u8; 4];
    copyin(mm, &mut bytes, address)?;
    let length = i32::from_ne_bytes(bytes);
    if length < 0 {
        Err(Errno::InvalidArgument)
    } else {
        Ok(length as usize)
    }
}

/// 将可选的套接字地址 `name` 按用户提供的缓冲区容量截断写入 `address`，并将完整地址字节长度写回 `length_address`。
///
/// 若 `name` 为 `None`（例如无源地址的流式接收），则不写入地址缓冲区，仅向 `length_address` 写回长度 `0`。
fn output_address(
    mm: &Mm,
    address: usize,
    length_address: usize,
    name: Option<SocketAddress>,
) -> Result<(), Errno> {
    let capacity = capacity(mm, length_address)?;
    let length = output_name(mm, address, capacity, name)?;
    copyout(mm, length_address, &(length as u32).to_ne_bytes())
}

/// `bind` 系统调用：将用户空间传入的本地套接字地址复制并解码到栈上后，绑定到指定套接字。
fn bind(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let mm = user.mm().expect("bind has no MM");
    let endpoint = input_address(&mm, raw[1], raw[2])?;
    file.socket()
        .expect("checked socket capability")
        .bind(endpoint)?;
    Ok(0)
}

/// `listen` 系统调用：将套接字转换为被动监听状态。
///
/// 用户传入的有符号 `backlog`（`raw[1] as i32`）转换为无符号整数后上限截断为 `SOMAXCONN`（负数转换后会夹紧到 `SOMAXCONN`），`0` 保持原值交给监听队列处理。
fn listen(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let backlog = (raw[1] as i32 as u32).min(crate::net::SOMAXCONN as u32) as usize;
    file.socket()
        .expect("checked socket capability")
        .listen(backlog)?;
    Ok(0)
}

/// `accept`（`flags = 0`）与 `accept4` 系统调用：从监听套接字接受一个已完成握手的子连接，可选地向用户空间写回对端地址，并分配新的文件描述符。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。在调用底层 `accept` 前先预留文件描述符槽位；若写出对端地址（`raw[1] != 0`）时发生缺页错误，未发布的子套接字与预留槽位会自动释放。
fn accept(user: &UserThread, raw: [usize; 6], flags: u32) -> SysResult {
    if flags & !SOCKET_FLAGS != 0 {
        return Err(Errno::InvalidArgument);
    }

    let file = socket_file(user, raw[0])?;
    let runtime = user.runtime().expect("accept has no runtime");
    let reservation = runtime.files.reserve(nofile(user))?;
    let child = file.socket().expect("checked socket capability").accept(
        flags,
        runtime.credentials.uid,
        runtime.credentials.gid,
        FileIoContext::User(user),
    )?;

    if raw[1] != 0 {
        let mm = user.mm().expect("accept has no MM");
        output_address(&mm, raw[1], raw[2], Some(child.peer_address()?))?;
    }

    let child: FileRef = child;
    Ok(reservation.publish(
        child,
        DescriptorFlags {
            close_on_exec: flags & SOCK_CLOEXEC != 0,
        },
    ) as isize)
}

/// `connect` 系统调用：读取目标端点地址并发起连接或设置数据报默认对端。
///
/// 阻塞握手等待以及被信号打断后的状态保持（如返回 `EINTR` 后再次调用返回 `EALREADY`）均由底层 `SocketFile` 维护。
fn connect(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let mm = user.mm().expect("connect has no MM");
    let endpoint = input_address(&mm, raw[1], raw[2])?;
    file.socket()
        .expect("checked socket capability")
        .connect(endpoint, FileIoContext::User(user))?;
    Ok(0)
}

/// `getsockname`（`peer = false`）与 `getpeername`（`peer = true`）系统调用：查询套接字的本地绑定地址或已连接的对端地址并写回用户空间。
///
/// 若套接字尚未连接，查询对端地址会直接返回底层 `ENOTCONN` 错误。
fn name(user: &UserThread, raw: [usize; 6], peer: bool) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let socket = file.socket().expect("checked socket capability");
    let address = if peer {
        socket.peer_address()?
    } else {
        socket.local_address()?
    };

    let mm = user.mm().expect("socket name has no MM");
    output_address(&mm, raw[1], raw[2], Some(address))?;
    Ok(0)
}

/// `sendto` 系统调用：从单个用户缓冲区向套接字发送数据，并支持为本次发送指定单次目标地址。
///
/// 对于面向连接的 TCP 套接字，目标地址参数会被忽略；对于 UDP 套接字，单次目标地址仅作用于本次发送，不会修改 `connect` 设置的默认对端。
fn sendto(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let mm = user.mm().expect("sendto has no MM");
    let mut source = WriteBuffer::user(&mm, raw[1], raw[2])?;
    let file = socket_file(user, raw[0])?;
    let socket = file.socket().expect("checked socket capability");
    let destination = if raw[4] == 0 {
        None
    } else {
        let length = raw[5] as i32;
        if length < 0 || (ADDRESS_STORAGE_LEN as i32) < length {
            return Err(Errno::InvalidArgument);
        }
        send_address(&mm, socket, raw[4], length as usize)?
    };

    let count = socket.send(
        &mut source,
        destination,
        raw[3] as u32,
        FileIoContext::User(user),
    )?;
    Ok(count as isize)
}

/// 为 `sendto` / `sendmsg` 从用户空间拷贝单次目标地址字节并按套接字协议族解析：
/// - TCP 套接字忽略目标地址，但在拷贝验证用户内存可读后直接返回 `Ok(None)`；
/// - 匿名 UNIX 套接字对不支持指定目标路径，返回 `Ok(Some(SocketAddress::UnixUnnamed))` 交由下层协议返回对应错误；
/// - IPv4 / IPv6 数据报套接字解码为具体的 `SocketAddress`。
fn send_address(
    mm: &Mm,
    socket: &SocketFile,
    address: usize,
    length: usize,
) -> Result<Option<SocketAddress>, Errno> {
    let mut bytes = [0u8; ADDRESS_STORAGE_LEN];
    copyin(mm, &mut bytes[..length], address)?;
    if socket.protocol() == IPPROTO_TCP {
        Ok(None)
    } else if socket.domain() == AF_UNIX {
        Ok(Some(SocketAddress::UnixUnnamed))
    } else {
        Ok(Some(SocketAddress::decode(&bytes[..length])?))
    }
}

/// 从用户空间 LP64 `struct msghdr` 解析出的内核侧消息头参数快照。
struct MessageHeader {
    /// 用户空间可选地址缓冲区的虚拟地址（`msg_name`，`0` 表示空指针）。
    name: usize,
    /// 地址缓冲区的有效字节上限（由 `msg_namelen` 校验并限制在 `ADDRESS_STORAGE_LEN` 以内；当 `name == 0` 时固定为 `0`）。
    name_capacity: usize,
    /// 用户空间 `struct iovec` 数组的起始虚拟地址（`msg_iov`）。
    vectors: usize,
    /// `msg_iov` 数组包含的元素个数（`msg_iovlen`，已校验不超过 `MAX_IOV_COUNT`）。
    vector_count: usize,
}

/// 从用户空间一次性读取 56 字节的 LP64 `struct msghdr`，按固定字节偏移解析各字段并校验向量数量与辅助数据长度。
///
/// 输入时的对齐填充字节与 `msg_flags` 字段均被忽略；当前不支持非零长度的辅助控制信息（`msg_controllen != 0` 时返回 `Err(Errno::Unsupported)`）。
fn message_header(mm: &Mm, address: usize) -> Result<MessageHeader, Errno> {
    let mut bytes = [0u8; 56];
    copyin(mm, &mut bytes, address)?;
    let name = message_word(&bytes, 0);
    let name_length = i32::from_ne_bytes(bytes[8..12].try_into().expect("fixed namelen field"));
    let vectors = message_word(&bytes, 16);
    let vector_count = message_word(&bytes, 24);
    let _control = message_word(&bytes, 32);
    let control_length = message_word(&bytes, 40);
    let _flags = u32::from_ne_bytes(bytes[48..52].try_into().expect("fixed flags field"));
    if MAX_IOV_COUNT < vector_count {
        return Err(Errno::MessageTooLarge);
    }
    let name_capacity = if name == 0 {
        0
    } else {
        if name_length < 0 {
            return Err(Errno::InvalidArgument);
        }
        (name_length as usize).min(ADDRESS_STORAGE_LEN)
    };
    if control_length != 0 {
        return Err(Errno::Unsupported);
    }

    Ok(MessageHeader {
        name,
        name_capacity,
        vectors,
        vector_count,
    })
}

/// 从 56 字节 `msghdr` 缓冲区指定偏移 `offset` 处读取一个 8 字节 LP64 无符号长整型字（`u64`）并转换为 `usize`。
fn message_word(bytes: &[u8; 56], offset: usize) -> usize {
    u64::from_ne_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("fixed LP64 field"),
    ) as usize
}

/// `sendmsg` 系统调用：解析用户态 `struct msghdr` 与 `iovec` 分散/聚集向量数组，将其组装为统一写入缓冲后交由套接字发送。
///
/// 消息头中附带的单次目标地址不会改变 UDP 套接字已连接的默认对端。
fn sendmsg(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let socket = file.socket().expect("checked socket capability");
    let mm = user.mm().expect("sendmsg has no MM");
    let header = message_header(&mm, raw[1])?;
    let destination = if header.name_capacity == 0 {
        None
    } else {
        send_address(&mm, socket, header.name, header.name_capacity)?
    };

    let mut fast = [UserIoVector {
        address: 0,
        length: 0,
    }; 8];
    let vectors = import_vectors(&mm, header.vectors, header.vector_count, &mut fast)?;
    let mut source = WriteBuffer::user_vector(&mm, &vectors)?;

    Ok(socket.send(
        &mut source,
        destination,
        raw[2] as u32,
        FileIoContext::User(user),
    )? as isize)
}

/// `recvmsg` 系统调用：从套接字接收数据填入用户态 `iovec` 向量缓冲区，并更新用户空间 `struct msghdr` 中的源地址长度、输出标志与控制数据长度。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。消息一旦从接收队列消费，若后续写回源地址或 `msghdr` 字段时触发 `EFAULT`，已消费的报文不会退回接收队列。
/// - **返回值**：默认返回实际拷入用户缓冲区的字节数；若对 `SOCK_DGRAM` 或 `SOCK_SEQPACKET` 传入了 `MSG_TRUNC` 标志，则返回报文截断前的原始总长度。
fn recvmsg(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 解析套接字、读取 56 字节 msghdr 并导入 iovec 数组，调用底层 receive 接收数据。
    let file = socket_file(user, raw[0])?;
    let socket = file.socket().expect("checked socket capability");
    let mm = user.mm().expect("recvmsg has no MM");
    let header = message_header(&mm, raw[1])?;
    let mut fast = [UserIoVector {
        address: 0,
        length: 0,
    }; 8];
    let vectors = import_vectors(&mm, header.vectors, header.vector_count, &mut fast)?;
    let mut target = ReadBuffer::user_vector(&mm, &vectors)?;
    let flags = raw[2] as u32;
    let message = socket.receive(&mut target, flags, FileIoContext::User(user))?;

    // Step 2: 若调用方提供了 msg_name 缓冲区，写出源地址并更新 msghdr 偏移 8 处的 msg_namelen。
    if header.name != 0 {
        let length = output_name(&mm, header.name, header.name_capacity, message.source)?;
        output_message_field(&mm, raw[1], 8, &(length as u32).to_ne_bytes())?;
    }
    // Step 3: 更新 msghdr 偏移 48 处的 msg_flags 与偏移 40 处的 msg_controllen (0)，并根据 MSG_TRUNC 计算返回长度。
    output_message_field(&mm, raw[1], 48, &message.flags.to_ne_bytes())?;
    output_message_field(&mm, raw[1], 40, &0u64.to_ne_bytes())?;

    let count =
        if matches!(socket.socket_type(), SOCK_DGRAM | SOCK_SEQPACKET) && flags & MSG_TRUNC != 0 {
            message.original_len
        } else {
            message.copied
        };
    Ok(count as isize)
}

/// 向用户空间 `struct msghdr`（基址为 `header`）的指定字节偏移 `offset` 处单独写回一个字段，保持其余指针、计数与填充字节不变。
fn output_message_field(mm: &Mm, header: usize, offset: usize, bytes: &[u8]) -> Result<(), Errno> {
    let address = header.checked_add(offset).ok_or(Errno::BadAddress)?;
    copyout(mm, address, bytes)
}

/// 将可选的套接字地址 `name` 编码后，按用户缓冲区容量上限 `capacity` 截断写入 `address`，并返回编码后的未截断完整字节长度。
fn output_name(
    mm: &Mm,
    address: usize,
    capacity: usize,
    name: Option<SocketAddress>,
) -> Result<usize, Errno> {
    let mut bytes = [0u8; MAX_ADDRESS_LEN];
    let length = name.map_or(0, |name| name.encode(&mut bytes));
    let copied = capacity.min(length);
    if copied != 0 {
        copyout(mm, address, &bytes[..copied])?;
    }
    Ok(length)
}

/// `recvfrom` 系统调用：从套接字接收数据到单个用户缓冲区，可选地写回发送方源地址，并根据 `MSG_TRUNC` 标志返回实际拷贝字节数或原始报文长度。
fn recvfrom(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let mm = user.mm().expect("recvfrom has no MM");
    let mut target = ReadBuffer::user(&mm, raw[1], raw[2])?;
    let file = socket_file(user, raw[0])?;
    let message = file.socket().expect("checked socket capability").receive(
        &mut target,
        raw[3] as u32,
        FileIoContext::User(user),
    )?;

    if raw[4] != 0 {
        output_address(&mm, raw[4], raw[5], message.source)?;
    }

    let socket = file.socket().expect("checked socket capability");
    let count = if matches!(socket.socket_type(), SOCK_DGRAM | SOCK_SEQPACKET)
        && raw[3] as u32 & MSG_TRUNC != 0
    {
        message.original_len
    } else {
        message.copied
    };
    Ok(count as isize)
}

/// 在读取或写入用户态选项缓冲区之前，预先校验 `(level, option)` 组合是否受当前套接字的协议族/类型支持，以及只读选项是否被用于 `setsockopt`（`setting == true`）。
fn check_option(socket: &SocketFile, level: i32, option: i32, setting: bool) -> Result<(), Errno> {
    match (level, option) {
        (SOL_SOCKET, SO_REUSEADDR | SO_KEEPALIVE | SO_RCVTIMEO | SO_SNDTIMEO) => Ok(()),
        (SOL_SOCKET, SO_SNDBUF | SO_RCVBUF) if !setting || socket.domain() != AF_UNIX => Ok(()),
        (SOL_SOCKET, SO_TYPE | SO_ERROR | SO_ACCEPTCONN | SO_PROTOCOL | SO_DOMAIN) if !setting => {
            Ok(())
        }
        (IPPROTO_TCP, TCP_NODELAY | TCP_CONGESTION) if socket.protocol() == IPPROTO_TCP => Ok(()),
        (IPPROTO_TCP, TCP_MAXSEG) if !setting && socket.protocol() == IPPROTO_TCP => Ok(()),
        (IPPROTO_IPV6, IPV6_V6ONLY) if socket.domain() == AF_INET6 => Ok(()),
        _ => Err(Errno::ProtocolOptionNotSupported),
    }
}

/// 从用户空间逐字节读取有界拥塞控制算法名称（最多读取 `CONGESTION_NAME_LEN - 1` 即 15 字节），遇到首个 NUL (`\0`) 字节立即停止读取，避免越过终止符后访问未映射的用户内存。
fn input_congestion(
    mm: &Mm,
    address: usize,
    length: usize,
) -> Result<[u8; CONGESTION_NAME_LEN], Errno> {
    let mut name = [0u8; CONGESTION_NAME_LEN];
    for offset in 0..length.min(CONGESTION_NAME_LEN - 1) {
        let source = address.checked_add(offset).ok_or(Errno::BadAddress)?;
        copyin(mm, &mut name[offset..offset + 1], source)?;
        if name[offset] == 0 {
            break;
        }
    }
    Ok(name)
}

/// 从用户空间读取 16 字节 LP64 `struct timeval` 超时配置并换算为按内核时钟节拍（`TICK_NS`）向上取整的纳秒数。
///
/// - 若 `tv_usec` 不在 `0..1_000_000` 范围内，返回 `Err(Errno::Domain)`（即 `EDOM`）；
/// - 若 `tv_sec < 0`，返回 `Ok(Some(0))` 表示立即超时（非阻塞超时）；
/// - 若 `tv_sec == 0 && tv_usec == 0`，返回 `Ok(None)` 表示无限等待（禁用超时）；
/// - 若换算并按节拍向上取整后的纳秒数超出 `u64::MAX`，同样退化为 `Ok(None)`（无限等待）。
fn input_timeout(mm: &Mm, address: usize, length: usize) -> Result<Option<u64>, Errno> {
    if length < TIMEVAL_LEN {
        return Err(Errno::InvalidArgument);
    }

    let mut bytes = [0u8; TIMEVAL_LEN];
    copyin(mm, &mut bytes, address)?;
    let seconds = i64::from_ne_bytes(bytes[..8].try_into().expect("fixed timeval seconds"));
    let microseconds =
        i64::from_ne_bytes(bytes[8..].try_into().expect("fixed timeval microseconds"));
    if microseconds < 0 || USEC_PER_SEC as i64 <= microseconds {
        return Err(Errno::Domain);
    }
    if seconds < 0 {
        return Ok(Some(0));
    }
    if seconds == 0 && microseconds == 0 {
        return Ok(None);
    }

    // 在 u128 宽整数域完成纳秒换算与 TICK_NS 向上取整，若超出 u64 表示范围则映射为 None（无限等待）。
    let nanos = seconds as u128 * u128::from(NSEC_PER_SEC)
        + microseconds as u128 * u128::from(NSEC_PER_USEC);
    let tick = u128::from(TICK_NS);
    let quantized = nanos.div_ceil(tick) * tick;
    Ok(u64::try_from(quantized).ok())
}

/// 将套接字保存的超时纳秒数导出为 16 字节 LP64 `struct timeval`；`None`（无限等待）与 `Some(0)`（立即超时）均编码为全零的 `timeval`。
fn timeout_bytes(timeout: Option<u64>) -> [u8; TIMEVAL_LEN] {
    let nanos = timeout.unwrap_or(0);
    let mut bytes = [0u8; TIMEVAL_LEN];
    bytes[..8].copy_from_slice(&((nanos / NSEC_PER_SEC) as i64).to_ne_bytes());
    bytes[8..].copy_from_slice(&((nanos % NSEC_PER_SEC / NSEC_PER_USEC) as i64).to_ne_bytes());
    bytes
}

/// `setsockopt` 系统调用：在套接字端点锁外从用户空间读取选项值，经校验后更新套接字的超时时间、TCP 拥塞控制算法、缓冲区配额或布尔开关。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。所有 `copyin` 均在加锁修改底层套接字前完成。
fn setsockopt(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 校验文件描述符、选项长度非负以及 (level, option) 是否支持写入。
    let file = socket_file(user, raw[0])?;
    let socket = file.socket().expect("checked socket capability");
    let length = raw[4] as i32;
    if length < 0 {
        return Err(Errno::InvalidArgument);
    }
    let level = raw[1] as i32;
    let option = raw[2] as i32;
    check_option(socket, level, option, true)?;

    // Step 2: 特殊处理变长或非 4 字节整型选项（SO_RCVTIMEO / SO_SNDTIMEO 的 timeval 与 TCP_CONGESTION 的算法名称）。
    let mm = user.mm().expect("setsockopt has no MM");
    if level == SOL_SOCKET && matches!(option, SO_RCVTIMEO | SO_SNDTIMEO) {
        let timeout = input_timeout(&mm, raw[3], length as usize)?;
        socket.set_timeout(option == SO_RCVTIMEO, timeout);
        return Ok(0);
    }
    if level == IPPROTO_TCP && option == TCP_CONGESTION {
        if length == 0 {
            return Err(Errno::InvalidArgument);
        }
        let name = input_congestion(&mm, raw[3], length as usize)?;
        let end = name
            .iter()
            .position(|byte| *byte == 0)
            .expect("bounded congestion name has no terminator");
        socket.set_congestion_control(&name[..end])?;
        return Ok(0);
    }
    // Step 3: 读取 4 字节 i32 选项值并分发给对应的套接字配置方法。
    if length < 4 {
        return Err(Errno::InvalidArgument);
    }

    let mut bytes = [0u8; 4];
    copyin(&mm, &mut bytes, raw[3])?;
    let value = i32::from_ne_bytes(bytes);
    match (level, option) {
        (SOL_SOCKET, SO_REUSEADDR) => socket.set_reuse_address(value != 0)?,
        (SOL_SOCKET, SO_KEEPALIVE) => socket.set_keepalive(value != 0)?,
        (SOL_SOCKET, SO_SNDBUF) => socket.set_send_budget(value)?,
        (SOL_SOCKET, SO_RCVBUF) => socket.set_receive_budget(value)?,
        (IPPROTO_TCP, TCP_NODELAY) => socket.set_nodelay(value != 0)?,
        (IPPROTO_IPV6, IPV6_V6ONLY) => socket.set_v6_only(value != 0)?,
        _ => unreachable!("checked socket setter has no implementation"),
    }
    Ok(0)
}

/// 查询 TCP 套接字当前生效的拥塞控制算法，并将其名称（`"reno"` 或 `"none"`）填入 16 字节 NUL 填充的固定数组中返回。
fn congestion_name(socket: &SocketFile) -> Result<[u8; CONGESTION_NAME_LEN], Errno> {
    let name: &[u8] = match socket.congestion_control()? {
        CongestionControl::Reno => b"reno",
        CongestionControl::None => b"none",
    };
    let mut bytes = [0u8; CONGESTION_NAME_LEN];
    bytes[..name.len()].copy_from_slice(name);
    Ok(bytes)
}

/// 将 `getsockopt` 查询到的选项字节按用户提供的容量上限截断写入 `raw[3]`（`optval`），并将实际拷贝的字节数写回 `raw[4]`（`optlen`）。
fn output_option(mm: &Mm, raw: [usize; 6], capacity: usize, bytes: &[u8]) -> SysResult {
    let copied = capacity.min(bytes.len());
    if copied != 0 {
        copyout(mm, raw[3], &bytes[..copied])?;
    }
    copyout(mm, raw[4], &(copied as u32).to_ne_bytes())?;
    Ok(0)
}

/// `getsockopt` 系统调用：查询套接字的超时配置、拥塞控制名称、协议参数或挂起错误，并写回用户空间。
///
/// 查询 `SO_ERROR` 时会原子取走并清除套接字上挂起的异步错误（`take_error`），将其转换为正数 ABI `errno` 值写回；若后续 `copyout` 失败，已取走的错误状态不会恢复。
fn getsockopt(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let socket = file.socket().expect("checked socket capability");
    let level = raw[1] as i32;
    let option = raw[2] as i32;
    check_option(socket, level, option, false)?;
    let mm = user.mm().expect("getsockopt has no MM");
    let capacity = capacity(&mm, raw[4])?;
    if level == SOL_SOCKET && matches!(option, SO_RCVTIMEO | SO_SNDTIMEO) {
        let bytes = timeout_bytes(socket.timeout(option == SO_RCVTIMEO));
        return output_option(&mm, raw, capacity, &bytes);
    }
    if level == IPPROTO_TCP && option == TCP_CONGESTION {
        return output_option(&mm, raw, capacity, &congestion_name(socket)?);
    }

    let value = match (level, option) {
        (IPPROTO_TCP, TCP_NODELAY) => i32::from(socket.nodelay()?),
        (IPPROTO_TCP, TCP_MAXSEG) => {
            i32::try_from(socket.effective_mss()?).map_err(|_| Errno::Overflow)?
        }
        (IPPROTO_IPV6, IPV6_V6ONLY) => i32::from(socket.v6_only()?),
        (SOL_SOCKET, SO_REUSEADDR) => i32::from(socket.reuse_address()),
        (SOL_SOCKET, SO_KEEPALIVE) => i32::from(socket.keepalive()),
        (SOL_SOCKET, SO_TYPE) => socket.socket_type() as i32,
        (SOL_SOCKET, SO_ERROR) => socket
            .take_error()
            .map_or(0, |error| (-Errno::from(error).result()) as i32),
        (SOL_SOCKET, SO_SNDBUF) => {
            i32::try_from(socket.send_budget()).map_err(|_| Errno::Overflow)?
        }
        (SOL_SOCKET, SO_RCVBUF) => {
            i32::try_from(socket.receive_budget()).map_err(|_| Errno::Overflow)?
        }
        (SOL_SOCKET, SO_ACCEPTCONN) => i32::from(socket.is_listener()),
        (SOL_SOCKET, SO_PROTOCOL) => socket.protocol(),
        (SOL_SOCKET, SO_DOMAIN) => i32::from(socket.domain()),
        _ => unreachable!("checked socket getter has no implementation"),
    };
    output_option(&mm, raw, capacity, &value.to_ne_bytes())
}

/// `shutdown` 系统调用：按指定方向（`SHUT_RD` / `SHUT_WR` / `SHUT_RDWR`）关闭套接字连接上的读取端、写入端或双向传输，同时保持文件描述符本身处于打开状态。
fn shutdown(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    file.socket()
        .expect("checked socket capability")
        .shutdown(raw[1] as u32)?;
    Ok(0)
}
