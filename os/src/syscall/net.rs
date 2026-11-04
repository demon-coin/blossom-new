//! socket 原始入口只复制目标 ABI 字节，描述符与协议拥有沿同一文件表发表和归还。

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

/// 仅无限等待的内部中断标记进入信号重启出口，有限等待交付实际 EINTR。
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

/// 按目标 int 截取描述符；负值不转成有效文件表索引。
fn descriptor(raw: usize) -> Result<usize, Errno> {
    let fd = raw as i32;
    if fd < 0 {
        Err(Errno::BadFd)
    } else {
        Ok(fd as usize)
    }
}

/// 文件表短锁只取得同一 OFD 的强引用，真实 socket 能力在锁外检查。
fn socket_file(user: &UserThread, raw: usize) -> Result<FileRef, Errno> {
    let runtime = user.runtime().expect("socket syscall has no runtime");
    let file = runtime.files.lookup(descriptor(raw)?)?;
    if file.socket().is_none() {
        return Err(Errno::NotSocket);
    }
    Ok(file)
}

/// 只将创建附加位传给 OFD，CLOEXEC 的真正状态由发表时的描述符槽保存。
fn socket(user: &UserThread, raw: [usize; 6]) -> SysResult {
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

/// 两个描述符先预留，完整 pair 和 fd 数组交付成功后才分别发表 OFD。
fn socketpair(user: &UserThread, raw: [usize; 6]) -> SysResult {
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

/// 地址先复制为内核值，两种 IP 族的完整结构与 scope 由共同解码边界检查。
fn input_address(mm: &Mm, address: usize, raw_length: usize) -> Result<SocketAddress, Errno> {
    let length = raw_length as i32;
    if length < ADDRESS_FAMILY_LEN as i32 || (ADDRESS_STORAGE_LEN as i32) < length {
        return Err(Errno::InvalidArgument);
    }
    let mut bytes = [0u8; ADDRESS_STORAGE_LEN];
    copyin(mm, &mut bytes[..length as usize], address)?;
    Ok(SocketAddress::decode(&bytes[..length as usize])?)
}

/// socklen 按目标四字节读取；负容量拒绝，零容量仍要求长度指针可读。
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

/// 只交付可容纳的名称前缀，再写完整名称长度；无名称接收只写真实长度零。
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

/// 绑定只消费复制后的内核端点，不跨协议操作借用用户地址。
fn bind(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let mm = user.mm().expect("bind has no MM");
    let endpoint = input_address(&mm, raw[1], raw[2])?;
    file.socket()
        .expect("checked socket capability")
        .bind(endpoint)?;
    Ok(0)
}

/// signed backlog 转为无符号后按独立 somaxconn 限幅，零交给真实 listener。
fn listen(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let backlog = (raw[1] as i32 as u32).min(crate::net::SOMAXCONN as u32) as usize;
    file.socket()
        .expect("checked socket capability")
        .listen(backlog)?;
    Ok(0)
}

/// 领取 child 前预留原 fd，名称复制失败只结束本次未发表 OFD 和预留槽。
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

/// 连接等待和已有握手的中断重启由真实 OFD 处理，入口不重建协议状态。
fn connect(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let mm = user.mm().expect("connect has no MM");
    let endpoint = input_address(&mm, raw[1], raw[2])?;
    file.socket()
        .expect("checked socket capability")
        .connect(endpoint, FileIoContext::User(user))?;
    Ok(0)
}

/// 名称来自同一协议端点，未连接 peer 的错误不合成为空地址。
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

/// 单次目的属于本次发送；TCP 保留忽略名称的语义，UDP 不修改默认 peer。
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

/// 名称字节只复制一次；TCP 不解释名称，匿名 UNIX 将实际名称请求交给协议拒绝。
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

/// LP64 消息头只保留本次操作需要的内核值，不借用户对象或复制第二份向量。
struct MessageHeader {
    name: usize,
    name_capacity: usize,
    vectors: usize,
    vector_count: usize,
}

/// 一次取得目标 56 字节头，逐字段解释固定偏移，padding 和输入 flags 不参与操作。
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

/// 固定头内的 unsigned long 使用目标 LP64 宽度，不能由宿主结构布局决定。
fn message_word(bytes: &[u8; 56], offset: usize) -> usize {
    u64::from_ne_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("fixed LP64 field"),
    ) as usize
}

/// 一次 import 的原向量直接交同一 OFD，单次名称不改变 UDP 的默认 peer。
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

/// 领取消息后只写真实名称、flags 和零 control 长度，后续 fault 不恢复已消费字节。
fn recvmsg(user: &UserThread, raw: [usize; 6]) -> SysResult {
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
    if header.name != 0 {
        let length = output_name(&mm, header.name, header.name_capacity, message.source)?;
        output_message_field(&mm, raw[1], 8, &(length as u32).to_ne_bytes())?;
    }
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

/// 消息头输出只覆盖一个规定字段，不改写指针、计数或 padding。
fn output_message_field(mm: &Mm, header: usize, offset: usize, bytes: &[u8]) -> Result<(), Errno> {
    let address = header.checked_add(offset).ok_or(Errno::BadAddress)?;
    copyout(mm, address, bytes)
}

/// 名称容量来自本次快照，短容量只复制地址前缀并返回真实完整长度。
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
/// 接收消费真实消息；输入 TRUNC 选择报文原长，名称仍来自同条记录。
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

/// 所选选项的协议和只读边界在 copyin 前检查，不把不支持的设置当作成功。
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

/// 有界字符串在首个 NUL 停止读取，后续不可读字节不改变已经结束的名称。
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

/// LP64 timeval 只导入完整 16 字节；微秒先校验，负秒随后表示立即返回。
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
    // 输入和上取整在宽整数域完成，超出内部纳秒范围沿无限 sentinel 返回。
    let nanos = seconds as u128 * u128::from(NSEC_PER_SEC)
        + microseconds as u128 * u128::from(NSEC_PER_USEC);
    let tick = u128::from(TICK_NS);
    let quantized = nanos.div_ceil(tick) * tick;
    Ok(u64::try_from(quantized).ok())
}

/// getter 导出端点实际量化值；无限与立即值均按 timeval 零值表示。
fn timeout_bytes(timeout: Option<u64>) -> [u8; TIMEVAL_LEN] {
    let nanos = timeout.unwrap_or(0);
    let mut bytes = [0u8; TIMEVAL_LEN];
    bytes[..8].copy_from_slice(&((nanos / NSEC_PER_SEC) as i64).to_ne_bytes());
    bytes[8..].copy_from_slice(&((nanos % NSEC_PER_SEC / NSEC_PER_USEC) as i64).to_ne_bytes());
    bytes
}

/// 选项 copyin 全在端点锁外，完整期限、缓冲提示和开关只修改原对象。
fn setsockopt(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    let socket = file.socket().expect("checked socket capability");
    let length = raw[4] as i32;
    if length < 0 {
        return Err(Errno::InvalidArgument);
    }
    let level = raw[1] as i32;
    let option = raw[2] as i32;
    check_option(socket, level, option, true)?;
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

/// 名称来自 raw 实际控制器，固定名字槽只存本次查询的 ABI 字节。
fn congestion_name(socket: &SocketFile) -> Result<[u8; CONGESTION_NAME_LEN], Errno> {
    let name: &[u8] = match socket.congestion_control()? {
        CongestionControl::Reno => b"reno",
        CongestionControl::None => b"none",
    };
    let mut bytes = [0u8; CONGESTION_NAME_LEN];
    bytes[..name.len()].copy_from_slice(name);
    Ok(bytes)
}

/// 短输出只修改有效前缀，optlen 返回实际复制量；copyout 不回滚协议消费。
fn output_option(mm: &Mm, raw: [usize; 6], capacity: usize, bytes: &[u8]) -> SysResult {
    let copied = capacity.min(bytes.len());
    if copied != 0 {
        copyout(mm, raw[3], &bytes[..copied])?;
    }
    copyout(mm, raw[4], &(copied as u32).to_ne_bytes())?;
    Ok(0)
}

/// 查询真实期限和协议配置；pending error 一次消费后使用最终 ABI errno 交付。
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

/// 收发方向关闭直接作用于真实端点，不用关闭描述符冒充 shutdown。
fn shutdown(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let file = socket_file(user, raw[0])?;
    file.socket()
        .expect("checked socket capability")
        .shutdown(raw[1] as u32)?;
    Ok(0)
}
