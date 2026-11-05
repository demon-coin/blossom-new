//! 软件包拥有库已验证的 IP 小值、传输标量头和独立 payload，线格式只由库处理。

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

// 当前网络域把完整 IP 报文限制为十六位总长，IPv6 同样扣除固定头，不启用 jumbogram。
pub(crate) const MAX_IP_PACKET_LEN: usize = u16::MAX as usize;
pub(crate) const MAX_IPV4_UDP_PAYLOAD_LEN: usize =
    MAX_IP_PACKET_LEN - IPV4_HEADER_LEN - UDP_HEADER_LEN;
pub(crate) const IPV6_UDP_HEADER_LEN: usize = IPV6_HEADER_LEN + UDP_HEADER_LEN;
pub(crate) const MAX_IPV6_UDP_PAYLOAD_LEN: usize = MAX_IP_PACKET_LEN - IPV6_UDP_HEADER_LEN;

#[cfg(feature = "ktest")]
const DEFAULT_HOP_LIMIT: u8 = 64;

/// 每接口的一份捕获后备独立保活，loan 不引用网络域或端点。
pub(crate) type CaptureStorage = Arc<SpinLock<Vec<u8>>>;

#[derive(Debug)]
enum TransportHeader {
    /// 空 payload 为真实静态空切片，全部数据另由 OwnedPacket 保存。
    Tcp(TcpRepr<'static>),
    Udp(UdpRepr),
}

/// 输入不携绑定租约；已接纳发送可用同一租约覆盖在途端口寿命。
pub struct OwnedPacket {
    ip: IpRepr,
    header: TransportHeader,
    payload: Vec<u8>,
    binding: Option<Arc<BindLease>>,
    capture_storage: Option<CaptureStorage>,
}

impl fmt::Debug for OwnedPacket {
    /// 诊断只展示协议小值与容量，不遍历租约归属或用户数据。
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
    /// 最后包拥有结束时归还真实 loan，holder 和多余后备均在短锁外析构。
    fn drop(&mut self) {
        if let Some(storage) = self.capture_storage.take() {
            return_capture_storage(&storage, mem::take(&mut self.payload));
        }
    }
}

impl OwnedPacket {
    /// 为 raw 产生的合法 TCP repr 准备 payload 拥有，不序列化线格式。
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

    /// typed 消费者可构造合法 UDP repr，实际 UDP 发送由独立 raw Socket dispatch。
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
        // IPv4 大报文由接口原 fragmenter 处理；当前 IPv6 只支持路径内完整报文。
        if MAX_IP_PACKET_LEN < length || (matches!(ip, IpRepr::Ipv6(_)) && mtu < length) {
            return Err(FsError::MessageTooLarge);
        }

        let mut storage = Vec::new();
        storage
            .try_reserve_exact(payload.len())
            .map_err(|_| FsError::NoMemory)?;
        Ok(Self::capture_udp(&ip, &udp, payload, storage))
    }

    /// 调用者已在接口锁外预留足量 storage；捕获只复制本次库已验证的 payload。
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

    /// 调用者已在接口锁外预留足量 storage；零长消息仍保留独立 UDP 头。
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

    /// 给已接纳发送附加端口寿命，payload 与源地址不另复制。
    pub fn with_binding(mut self, binding: Arc<BindLease>) -> Self {
        assert!(self.binding.is_none(), "packet binding replaced");
        self.binding = Some(binding);
        self
    }

    /// 非空入站 payload 携同一接口后备的归还责任，不增加常驻槽位。
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

    /// 借用发送责任，Root 队列继续持完整包直至实际交付结束。
    pub fn binding(&self) -> Option<&Arc<BindLease>> {
        self.binding.as_ref()
    }

    /// 在同步库 dispatch 内重借本包 payload，借用不能跨拥有转移。
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

    /// 协议号来自捕获的库 IP repr，不重复解析传输头。
    #[cfg(feature = "ktest")]
    pub fn protocol(&self) -> IpProtocol {
        self.ip.next_header()
    }

    /// UDP 四元组来自同一 IP repr 和标量头。
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

    /// 借用同一 UDP 记录的真实字节，零长消息返回空切片。
    pub fn udp_payload(&self) -> Result<&[u8], FsError> {
        if !matches!(self.header, TransportHeader::Udp(_)) {
            return Err(FsError::Unsupported);
        }
        Ok(&self.payload)
    }

    /// 返回完整 IP 长度，不计介质头或设备头。
    pub fn len(&self) -> usize {
        self.ip.buffer_len()
    }

    /// 返回同一库 IP repr 的源地址。
    pub fn source(&self) -> IpAddress {
        self.ip.src_addr()
    }

    /// 返回同一库 IP repr 的目的地址。
    pub fn destination(&self) -> IpAddress {
        self.ip.dst_addr()
    }

    /// TCP 四元组来自同一 IP repr 和标量头。
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

    /// raw TCP 只借当前包的 payload，头标量不借 DMA 或 assembler 存储。
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

/// 归还一份已结束的捕获拥有；已有后备时，多余 Vec 在解短锁后释放。
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

/// 所有可失败存储在发表前准备，resize 只初始化已预留容量。
pub(crate) fn zeroed_bytes(length: usize) -> Result<Vec<u8>, FsError> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| FsError::NoMemory)?;
    bytes.resize(length, 0);
    Ok(bytes)
}
