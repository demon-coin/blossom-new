//! 内核自拥有的 IP/传输层结构化报文（[`OwnedPacket`]）与入站捕获缓冲区复用机制。
//!
//! # 模块职责与架构定位
//! 在 `smoltcp` 协议栈接口层（`Interface`）与内核 TCP/UDP 端点（`TcpEndpoint` / `UdpEndpoint`）之间，
//! 报文需要脱离底层网卡接收缓冲区或重组缓冲区的生命周期，跨锁传递并排队。
//! 本模块定义了自包含的 [`OwnedPacket`]，将已由 `smoltcp` 解析校验完毕的 IP 首部表示（[`IpRepr`]）、
//! 传输层首部标量（[`TcpRepr`] / [`UdpRepr`]）以及独立的载荷字节（`Vec<u8>`）打包保存，
//! 从而不在内核端点中重复实现网络线格式（Wire Format）的序列化与反序列化。
//!
//! # 核心设计与缓冲区借用回收流
//!
//! ```text
//! OwnedPacket 结构与入站缓冲区复用 (CaptureStorage) 生命周期:
//!
//!   [InterfaceState::capture: Arc<SpinLock<Vec<u8>>>]
//!            |
//!            | 1. 入站前 take 预分配 Vec<u8> (若空则在锁外分配 64KiB)
//!            v
//!   +-------------------------------------------------------+
//!   | OwnedPacket                                           |
//!   |  - ip: IpRepr (源/目的 IP、协议号、载荷长度、HopLimit)|
//!   |  - header: TransportHeader                            |
//!   |      +-- Tcp(TcpRepr<'static>, payload 置空切片 &[])   |
//!   |      +-- Udp(UdpRepr)                                 |
//!   |  - payload: Vec<u8> (独立保存实际传输层载荷字节)      |
//!   |  - binding: Option<Arc<BindLease>> (出站端口保活租约) |
//!   |  - capture_storage: Option<CaptureStorage>            |
//!   +-------------------------------------------------------+
//!            |
//!            | 2. OwnedPacket Drop 时调用 return_capture_storage()
//!            v
//!   清空 Vec<u8> 并归还至 CaptureStorage 槽位（若槽位已有缓冲区则在自旋锁外释放多余 Vec）
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`CaptureStorage`] 内部仅包含一把保护单个 `Vec<u8>` 槽位的叶子级自旋锁（[`SpinLock`]）。
//! - 在 [`return_capture_storage`] 中，持锁期间仅执行 `capacity()` 检查与 `mem::swap` 指针交换，
//!   任何堆内存分配或 `Vec<u8>` 释放（`drop`）均在自旋锁临界区之外执行。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;
use core::mem;
use smoltcp::iface::{IpPayload, Packet};
use smoltcp::wire::{
    IpAddress, IpProtocol, IpRepr, TcpRepr, UdpRepr, IPV4_HEADER_LEN, IPV6_HEADER_LEN,
    UDP_HEADER_LEN,
};

use crate::fs::FsError;
use crate::net::address::InetEndpoint;
use crate::net::table::BindLease;
use crate::sync::SpinLock;

/// 单个完整 IP 报文的最大字节长度（65535 字节，受限于 16 位总长度字段；IPv6 同样遵循此上限，不启用超大帧 Jumbogram）。
pub(crate) const MAX_IP_PACKET_LEN: usize = u16::MAX as usize;
/// IPv4 下单个 UDP 报文的最大应用层载荷字节数（65535 - 20 字节 IPv4 首部 - 8 字节 UDP 首部 = 65507 字节）。
pub(crate) const MAX_IPV4_UDP_PAYLOAD_LEN: usize =
    MAX_IP_PACKET_LEN - IPV4_HEADER_LEN - UDP_HEADER_LEN;
/// IPv6 固定首部（40 字节）与 UDP 首部（8 字节）的总字节长度（48 字节）。
pub(crate) const IPV6_UDP_HEADER_LEN: usize = IPV6_HEADER_LEN + UDP_HEADER_LEN;
/// IPv6 下单个 UDP 报文的最大应用层载荷字节数（65535 - 48 = 65487 字节）。
pub(crate) const MAX_IPV6_UDP_PAYLOAD_LEN: usize = MAX_IP_PACKET_LEN - IPV6_UDP_HEADER_LEN;

/// 内核测试构造 UDP 报文时使用的默认 IP 跳数限制（TTL / Hop Limit = 64）。
#[cfg(feature = "ktest")]
const DEFAULT_HOP_LIMIT: u8 = 64;
/// 每个网络接口（`lo` / `eth0`）独立持有的入站载荷捕获缓冲区槽位。
///
/// 仅包裹单个 `Vec<u8>`，不反向引用网络栈或任何协议端点，从而支持在任意上下文安全归还。
pub(crate) type CaptureStorage = Arc<SpinLock<Vec<u8>>>;

/// 传输层首部标量表示。
#[derive(Debug)]
enum TransportHeader {
    /// TCP 首部标量（序列号、确认号、窗口大小、控制标志等）。
    ///
    /// 内部 `payload` 字段固定置为空静态切片 `&[]`，实际 TCP 载荷字节由外层 [`OwnedPacket::payload`] 独立持有，以消除生命周期传染。
    Tcp(TcpRepr<'static>),
    /// UDP 首部标量（源端口与目的端口）。
    Udp(UdpRepr),
}

/// 内核自拥有的结构化 IP 与传输层报文。
///
/// 入站报文不携带端口绑定租约（`binding` 为 `None`），但可携带 [`CaptureStorage`] 以便在消费完毕后归还预分配缓冲区；
/// 出站报文可通过 `binding` 持有本地端口租约（[`BindLease`]），确保报文在出站队列排队期间源端口不会被提前释放重用。
pub struct OwnedPacket {
    /// 已解析或待发送的 IP 层首部描述（源/目的地址、传输层协议号、载荷长度、跳数限制）。
    ip: IpRepr,
    /// 传输层（TCP 或 UDP）首部标量信息。
    header: TransportHeader,
    /// 独立持有的传输层有效载荷字节缓冲区。
    payload: Vec<u8>,
    /// 出站报文可选携带的本地端口绑定租约，使本地端口生命周期覆盖报文在途排队全过程。
    binding: Option<Arc<BindLease>>,
    /// 入站非空报文可选携带的捕获缓冲区归还句柄。
    capture_storage: Option<CaptureStorage>,
}

impl fmt::Debug for OwnedPacket {
    /// 格式化报文的 IP 首部、传输层首部、载荷字节长度及端口租约编号（不打印载荷具体字节内容）。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("OwnedPacket")
            .field("ip", &self.ip)
            .field("header", &self.header)
            .field("payload_len", &self.payload.len())
            .field("binding_id", &self.binding.as_ref().map(|lease| lease.id()))
            .finish()
    }
}

impl Drop for OwnedPacket {
    /// 当 [`OwnedPacket`] 被销毁时，若携带有入站捕获缓冲区归还句柄，则将内部 `payload` 容量清空并归还至对应接口槽位。
    fn drop(&mut self) {
        if let Some(storage) = self.capture_storage.take() {
            return_capture_storage(&storage, mem::take(&mut self.payload));
        }
    }
}

impl OwnedPacket {
    /// 根据 `smoltcp` 生成的 IP 首部与 TCP 报文段描述构造自拥有的 [`OwnedPacket`]。
    ///
    /// 在拷贝载荷前校验 IP 报文总长度不超过 `ip_mtu` 与 [`MAX_IP_PACKET_LEN`]，并按需精确分配载荷内存。
    pub fn try_from_tcp(ip: &IpRepr, tcp: &TcpRepr<'_>, ip_mtu: usize) -> Result<Self, FsError> {
        let length = ip
            .header_len()
            .checked_add(tcp.buffer_len())
            .ok_or(FsError::MessageTooLarge)?;
        if ip_mtu < length || MAX_IP_PACKET_LEN < length {
            return Err(FsError::MessageTooLarge);
        }

        let mut storage = Vec::new();
        storage
            .try_reserve_exact(tcp.payload.len())
            .map_err(|_| FsError::NoMemory)?;
        Ok(Self::capture_tcp(ip, tcp, storage))
    }

    /// 根据源/目的端点与载荷切片构造自拥有的 UDP [`OwnedPacket`]（供 `ktest` 测试路径使用）。
    ///
    /// 校验目的端口非零且源/目的 IP 版本一致；IPv4 允许超出链路 MTU 的报文进入接口分片器（Fragmenter），IPv6 则要求完整报文不超过 `mtu`。
    #[cfg(feature = "ktest")]
    pub fn try_from_udp(
        source: InetEndpoint,
        destination: InetEndpoint,
        payload: &[u8],
        mtu: usize,
    ) -> Result<Self, FsError> {
        if destination.port == 0 || source.address.version() != destination.address.version() {
            return Err(FsError::InvalidOperation);
        }
        let udp = UdpRepr {
            src_port: source.port,
            dst_port: destination.port,
        };
        let payload_len = udp
            .header_len()
            .checked_add(payload.len())
            .ok_or(FsError::MessageTooLarge)?;
        let ip = IpRepr::new(
            source.address,
            destination.address,
            IpProtocol::Udp,
            payload_len,
            DEFAULT_HOP_LIMIT,
        );
        let length = ip
            .header_len()
            .checked_add(payload_len)
            .ok_or(FsError::MessageTooLarge)?;
        // IPv4 大报文可由网络接口的分片器（Fragmenter）拆分发送；IPv6 不在发送端分片，要求完整报文长度不超过路径 MTU。
        if MAX_IP_PACKET_LEN < length || (matches!(ip, IpRepr::Ipv6(_)) && mtu < length) {
            return Err(FsError::MessageTooLarge);
        }

        let mut storage = Vec::new();
        storage
            .try_reserve_exact(payload.len())
            .map_err(|_| FsError::NoMemory)?;
        Ok(Self::capture_udp(&ip, &udp, payload, storage))
    }

    /// 使用调用方在接口锁外预先分配好容量的 `storage` 捕获 TCP 报文段。
    ///
    /// 调用方必须保证 `storage.capacity() >= tcp.payload.len()` 且 `ip.next_header() == IpProtocol::Tcp`。
    pub fn capture_tcp(ip: &IpRepr, tcp: &TcpRepr<'_>, mut storage: Vec<u8>) -> Self {
        assert!(
            tcp.payload.len() <= storage.capacity(),
            "TCP capture storage too small"
        );
        assert_eq!(ip.next_header(), IpProtocol::Tcp);

        storage.clear();
        storage.extend_from_slice(tcp.payload);
        Self {
            ip: ip.clone(),
            header: TransportHeader::Tcp(TcpRepr {
                payload: &[],
                ..*tcp
            }),
            payload: storage,
            binding: None,
            capture_storage: None,
        }
    }

    /// 使用调用方在接口锁外预先分配好容量的 `storage` 捕获 UDP 数据报。
    ///
    /// 调用方必须保证 `storage.capacity() >= payload.len()` 且 `ip.next_header() == IpProtocol::Udp`；零长度 UDP 数据报同样保留完整的 UDP 首部。
    pub fn capture_udp(ip: &IpRepr, udp: &UdpRepr, payload: &[u8], mut storage: Vec<u8>) -> Self {
        assert!(
            payload.len() <= storage.capacity(),
            "UDP capture storage too small"
        );
        assert_eq!(ip.next_header(), IpProtocol::Udp);

        storage.clear();
        storage.extend_from_slice(payload);
        Self {
            ip: ip.clone(),
            header: TransportHeader::Udp(*udp),
            payload: storage,
            binding: None,
            capture_storage: None,
        }
    }

    /// 为已获准入队的出站报文附加本地端口绑定租约（[`BindLease`]），保证报文发送完成前源端口不被回收。
    pub fn with_binding(mut self, binding: Arc<BindLease>) -> Self {
        assert!(self.binding.is_none(), "packet binding replaced");
        self.binding = Some(binding);
        self
    }

    /// 为非空入站报文关联所属接口的 [`CaptureStorage`]，以便在报文处理完毕析构时自动归还 `payload` 的堆内存容量。
    pub(crate) fn with_capture_storage(mut self, storage: CaptureStorage) -> Self {
        assert!(
            !self.payload.is_empty(),
            "empty packet cannot borrow capture storage"
        );
        assert!(
            self.capture_storage.is_none(),
            "packet capture storage replaced"
        );
        self.capture_storage = Some(storage);
        self
    }

    /// 获取该出站报文携带的本地端口绑定租约引用。
    pub fn binding(&self) -> Option<&Arc<BindLease>> {
        self.binding.as_ref()
    }

    /// 将当前拥有的首部与载荷重新借用拼装为 `smoltcp` 的 [`Packet<'_>`]，供接口层同步调用 `dispatch_ip` 编码发送。
    pub fn as_packet(&self) -> Packet<'_> {
        let payload = match &self.header {
            TransportHeader::Tcp(header) => IpPayload::Tcp(TcpRepr {
                payload: &self.payload,
                ..*header
            }),
            TransportHeader::Udp(header) => IpPayload::Udp(*header, &self.payload),
        };
        Packet::new(self.ip.clone(), payload)
    }

    /// 返回 IP 首部中记录的传输层协议号（如 `IpProtocol::Tcp` 或 `IpProtocol::Udp`）。
    #[cfg(feature = "ktest")]
    pub fn protocol(&self) -> IpProtocol {
        self.ip.next_header()
    }

    /// 提取 UDP 报文的（源端点，目的端点）四元组；若当前报文不是 UDP 则返回 [`FsError::Unsupported`]。
    pub fn udp_endpoints(&self) -> Result<(InetEndpoint, InetEndpoint), FsError> {
        let TransportHeader::Udp(header) = &self.header else {
            return Err(FsError::Unsupported);
        };
        Ok((
            InetEndpoint {
                address: self.source(),
                port: header.src_port,
            },
            InetEndpoint {
                address: self.destination(),
                port: header.dst_port,
            },
        ))
    }

    /// 借用 UDP 报文的有效载荷字节切片（零长度数据报返回 `&[]`）；若当前报文不是 UDP 则返回 [`FsError::Unsupported`]。
    pub fn udp_payload(&self) -> Result<&[u8], FsError> {
        if !matches!(self.header, TransportHeader::Udp(_)) {
            return Err(FsError::Unsupported);
        }
        Ok(&self.payload)
    }

    /// 返回完整 IP 报文的字节总长度（含 IP 首部与传输层报文，不含以太网帧头等链路层头部）。
    pub fn len(&self) -> usize {
        self.ip.buffer_len()
    }

    /// 返回 IP 首部中的源 IP 地址。
    pub fn source(&self) -> IpAddress {
        self.ip.src_addr()
    }

    /// 返回 IP 首部中的目的 IP 地址。
    pub fn destination(&self) -> IpAddress {
        self.ip.dst_addr()
    }

    /// 提取 TCP 报文的（源端点，目的端点）四元组；若当前报文不是 TCP 则返回 [`FsError::Unsupported`]。
    pub fn tcp_endpoints(&self) -> Result<(InetEndpoint, InetEndpoint), FsError> {
        let TransportHeader::Tcp(header) = &self.header else {
            return Err(FsError::Unsupported);
        };
        Ok((
            InetEndpoint {
                address: self.source(),
                port: header.src_port,
            },
            InetEndpoint {
                address: self.destination(),
                port: header.dst_port,
            },
        ))
    }

    /// 提取 TCP 报文的 [`IpRepr`] 副本及借用当前 `payload` 切片的 [`TcpRepr<'_>`]；若当前报文不是 TCP 则返回 [`FsError::Unsupported`]。
    pub fn tcp(&self) -> Result<(IpRepr, TcpRepr<'_>), FsError> {
        let TransportHeader::Tcp(header) = &self.header else {
            return Err(FsError::Unsupported);
        };
        Ok((
            self.ip.clone(),
            TcpRepr {
                payload: &self.payload,
                ..*header
            },
        ))
    }
}

/// 将使用完毕的入站载荷缓冲区 `bytes` 清空并尝试归还给接口的 [`CaptureStorage`] 槽位。
///
/// 若槽位当前为空，则通过 `mem::swap` 留存该 `Vec<u8>` 供下次入站复用；
/// 若槽位中已有其他缓冲区，则在释放自旋锁之后直接析构多余的 `bytes`。
pub(crate) fn return_capture_storage(owner: &CaptureStorage, mut bytes: Vec<u8>) {
    bytes.clear();
    {
        let mut storage = owner.lock();
        if storage.capacity() == 0 {
            mem::swap(&mut *storage, &mut bytes);
        }
    }

    drop(bytes);
}

/// 精确分配并初始化长度为 `length` 的全零字节向量，若堆内存不足则返回 [`FsError::NoMemory`]。
pub(crate) fn zeroed_bytes(length: usize) -> Result<Vec<u8>, FsError> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| FsError::NoMemory)?;
    bytes.resize(length, 0);
    Ok(bytes)
}
