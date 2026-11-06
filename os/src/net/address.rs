//! 套接字地址（Socket Address）ABI 编解码与协议族正规化模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于系统调用 ABI 层（`syscall` / `net::socket`）与内核传输协议层（`tcp` / `udp` / `unix`）之间：
//! - **向上（系统调用边界）**：负责解码来自用户态缓冲区的 `struct sockaddr` 原始字节流（`SocketAddress::decode`），
//!   以及将内核端点地址编码回用户态输出缓冲区（`SocketAddress::encode`），集中处理字节序（网络大端与主机本机字节序）转换与截断规则。
//! - **向下（协议栈与端口索引）**：提供紧凑的值类型 `InetEndpoint`（IP 地址 + 主机字节序端口），并在进入路由与协议栈前
//!   统一校验地址族兼容性、`IPV6_V6ONLY` 双栈策略、IPv4-mapped IPv6 地址（`::ffff:a.b.c.d`）降级以及未指定地址（`0.0.0.0` / `::`）正规化。
//!
//! ## 核心内存布局（Linux `sockaddr` ABI）
//! ```text
//! sockaddr_in (16 字节, AF_INET = 2):
//! +--------+--------+-----------------+---------------------------------+
//! | 0..2   | 2..4   | 4..8            | 8..16                           |
//! | family | port   | sin_addr (IPv4) | sin_zero (填充 0)               |
//! | (ne)   | (be)   | 4 bytes         | 8 bytes                         |
//! +--------+--------+-----------------+---------------------------------+
//!
//! sockaddr_in6 (28 字节, AF_INET6 = 10):
//! +--------+--------+-----------------+---------------------------------+-----------------+
//! | 0..2   | 2..4   | 4..8            | 8..24                           | 24..28          |
//! | family | port   | sin6_flowinfo   | sin6_addr (IPv6 / v4-mapped)    | sin6_scope_id   |
//! | (ne)   | (be)   | (be, u32)       | 16 bytes                        | (ne, u32)       |
//! +--------+--------+-----------------+---------------------------------+-----------------+
//!
//! sockaddr_un (匿名 UNIX 套接字, 2 字节, AF_UNIX = 1):
//! +--------+
//! | 0..2   |
//! | family |
//! | (ne)   |
//! +--------+
//! ```
//! 注：`ne` 表示本机字节序（Native-Endian），`be` 表示网络字节序（Big-Endian）。
//!
//! ## 并发契约
//! 本模块所有类型均为无锁的纯值类型（`Copy`），不包含共享可变状态，可在任意任务或中断/Work 上下文中安全复制与转换。

use core::convert::{TryFrom, TryInto};
use smoltcp::wire::{IpAddress, IpEndpoint, IpListenEndpoint, Ipv4Address, Ipv6Address};

use crate::fs::FsError;

/// 未指定地址族（Linux `AF_UNSPEC`），常用于 UDP 断开远端关联。
pub const AF_UNSPEC: u16 = 0;
/// 本地进程间通信 UNIX 域地址族（Linux `AF_UNIX` / `AF_LOCAL`）。
pub const AF_UNIX: u16 = 1;
/// IPv4 互联网协议地址族（Linux `AF_INET`）。
pub const AF_INET: u16 = 2;
/// IPv6 互联网协议地址族（Linux `AF_INET6`）。
pub const AF_INET6: u16 = 10;

/// Linux `struct sockaddr_in` 结构体字节长度（16 字节）。
pub const INET_ADDRESS_LEN: usize = 16;
/// Linux `struct sockaddr_in6` 结构体字节长度（28 字节）。
pub const INET6_ADDRESS_LEN: usize = 28;
/// 当前支持的最大 `sockaddr` ABI 缓冲区字节长度。
pub const MAX_ADDRESS_LEN: usize = INET6_ADDRESS_LEN;
/// `sockaddr` 头部地址族字段（`sa_family_t` / `u16`）的字节长度（2 字节）。
pub(crate) const ADDRESS_FAMILY_LEN: usize = core::mem::size_of::<u16>();

/// 套接字创建时绑定的网络层协议族。
///
/// 该属性在套接字生命周期内保持不可变；即使双栈 IPv6 套接字实际收发 IPv4-mapped 报文，
/// 其打开文件对象的地址族仍保持为 `Ipv6`，以保证 `getsockname` / `getpeername` 返回正确的 `sockaddr_in6` 格式。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AddressFamily {
    /// IPv4 协议族（对应 `AF_INET`）。
    Ipv4,
    /// IPv6 协议族（对应 `AF_INET6`，未开启 `IPV6_V6ONLY` 时兼容 IPv4-mapped 地址）。
    Ipv6,
}

/// 发起地址正规化的系统调用操作类别。
///
/// Linux ABI 在地址族不匹配或违反 `IPV6_V6ONLY` 约束时，针对不同系统调用返回不同的错误码
/// （例如 `EINVAL`、`EAFNOSUPPORT` 或 `ENETUNREACH`），因此正规化阶段需要区分具体操作。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AddressOperation {
    /// 本地端口与地址绑定（`bind`）。
    Bind,
    /// 建立连接或关联远端默认地址（`connect`）。
    Connect,
    /// 指定单次报文目的地址发送（`sendto` / `sendmsg`）。
    Send,
}

impl AddressFamily {
    /// 返回对应的 Linux `sa_family_t` 地址族常量编号（`AF_INET` 或 `AF_INET6`）。
    pub fn number(self) -> u16 {
        match self {
            Self::Ipv4 => AF_INET,
            Self::Ipv6 => AF_INET6,
        }
    }

    /// 返回该协议族对应的通配（未指定）IP 地址（`0.0.0.0` 或 `::`），用于自动绑定未显式 `bind` 的套接字。
    pub fn unspecified(self) -> IpAddress {
        match self {
            Self::Ipv4 => IpAddress::Ipv4(Ipv4Address::UNSPECIFIED),
            Self::Ipv6 => IpAddress::Ipv6(Ipv6Address::UNSPECIFIED),
        }
    }
}

/// 内核传输层统一的 IP 端点标识（IP 地址 + 端口号）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InetEndpoint {
    /// 网络层 IP 地址（IPv4 或原生 IPv6；IPv4-mapped 地址在正规化后已降级为 `IpAddress::Ipv4`）。
    pub address: IpAddress,
    /// 传输层端口号，采用主机本机字节序（Host Byte Order）。
    pub port: u16,
}

impl InetEndpoint {
    /// 判断端点 IP 是否为通配地址（`0.0.0.0` 或 `::`）。
    pub fn is_unspecified(self) -> bool {
        self.address.is_unspecified()
    }

    /// 转换为 `smoltcp` 协议栈所需的明确 `IpEndpoint`（直接保留当前地址与端口）。
    pub fn raw(self) -> IpEndpoint {
        IpEndpoint::new(self.address, self.port)
    }

    /// 转换为 `smoltcp` 监听端点 `IpListenEndpoint`；若地址为通配地址则映射为 `addr: None`，允许接收任意本地地址上的连接。
    pub fn raw_listen(self) -> IpListenEndpoint {
        IpListenEndpoint {
            addr: if self.is_unspecified() {
                None
            } else {
                Some(self.address)
            },
            port: self.port,
        }
    }

    /// 从 `smoltcp` 的 `IpEndpoint` 构造内核 `InetEndpoint` 值。
    pub fn from_raw(endpoint: IpEndpoint) -> Self {
        Self {
            address: endpoint.addr,
            port: endpoint.port,
        }
    }
}

/// 用户态 `struct sockaddr` 解码后的通用套接字地址表示。
///
/// 在刚完成 ABI 解码时保留完整的原始字段（如 IPv6 的 mapped 形式、`flowinfo` 和 `scope_id`），
/// 随后通过 [`SocketAddress::normalize`] 在进入传输层前集中完成合法性校验与双栈转换。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SocketAddress {
    /// 未指定地址（`AF_UNSPEC`），用于清除已关联的远端地址。
    Unspecified,
    /// IPv4 套接字地址（对应 `struct sockaddr_in`）。
    Inet(InetEndpoint),
    /// IPv6 套接字地址（对应 `struct sockaddr_in6`）。
    Inet6 {
        /// IPv6 地址与主机字节序端口号。
        endpoint: InetEndpoint,
        /// IPv6 流标签与流量类别信息（主机字节序 `u32`）。
        flowinfo: u32,
        /// 链路本地范围标识（Scope ID，本机字节序 `u32`；当前仅支持全局/回环范围 `0`）。
        scope_id: u32,
    },
    /// 匿名 UNIX 域套接字地址（仅含 2 字节 `AF_UNIX` 族编号，不含文件系统路径）。
    UnixUnnamed,
}

impl SocketAddress {
    /// 将从用户空间复制进来的 `sockaddr` 原始字节切片解码为 [`SocketAddress`]。
    ///
    /// 长度不足对应地址族结构体最小要求时返回 `InvalidOperation`（对应 `EINVAL`）；
    /// 带路径名的 `AF_UNIX` 地址当前返回 `Unsupported`，未知地址族返回 `AddressFamilyNotSupported`。
    pub fn decode(bytes: &[u8]) -> Result<Self, FsError> {
        if bytes.len() < ADDRESS_FAMILY_LEN {
            return Err(FsError::InvalidOperation);
        }

        match u16::from_ne_bytes([bytes[0], bytes[1]]) {
            AF_UNSPEC => Ok(Self::Unspecified),
            AF_INET => {
                if bytes.len() < INET_ADDRESS_LEN {
                    return Err(FsError::InvalidOperation);
                }
                Ok(Self::Inet(InetEndpoint {
                    address: IpAddress::Ipv4(Ipv4Address::new(
                        bytes[4], bytes[5], bytes[6], bytes[7],
                    )),
                    port: u16::from_be_bytes([bytes[2], bytes[3]]),
                }))
            }
            AF_INET6 => {
                if bytes.len() < INET6_ADDRESS_LEN {
                    return Err(FsError::InvalidOperation);
                }
                Ok(Self::Inet6 {
                    endpoint: InetEndpoint {
                        address: IpAddress::Ipv6(Ipv6Address::from(
                            <[u8; 16]>::try_from(&bytes[8..24]).unwrap(),
                        )),
                        port: u16::from_be_bytes([bytes[2], bytes[3]]),
                    },
                    flowinfo: u32::from_be_bytes(bytes[4..8].try_into().unwrap()),
                    scope_id: u32::from_ne_bytes(bytes[24..28].try_into().unwrap()),
                })
            }
            AF_UNIX if bytes.len() == ADDRESS_FAMILY_LEN => Ok(Self::UnixUnnamed),
            AF_UNIX => Err(FsError::Unsupported),
            _ => Err(FsError::AddressFamilyNotSupported),
        }
    }

    /// 根据套接字所属的 `family` 将内核 `InetEndpoint` 包装为用户态可见的 [`SocketAddress`]。
    ///
    /// 若套接字为 `AF_INET6` 双栈套接字且底层连接实际使用的是 IPv4 地址，
    /// 则自动将其提升为 `::ffff:a.b.c.d`（IPv4-mapped IPv6 地址），保证返回符合 `sockaddr_in6` ABI 的地址。
    pub fn from_endpoint(endpoint: InetEndpoint, family: AddressFamily) -> Self {
        match family {
            AddressFamily::Ipv4 => {
                assert!(
                    matches!(endpoint.address, IpAddress::Ipv4(_)),
                    "IPv4 socket has IPv6 endpoint"
                );
                Self::Inet(endpoint)
            }
            AddressFamily::Ipv6 => Self::Inet6 {
                endpoint: InetEndpoint {
                    address: match endpoint.address {
                        IpAddress::Ipv4(address) => IpAddress::Ipv6(address.to_ipv6_mapped()),
                        address @ IpAddress::Ipv6(_) => address,
                    },
                    port: endpoint.port,
                },
                flowinfo: 0,
                scope_id: 0,
            },
        }
    }

    /// 根据套接字的协议族、`IPV6_V6ONLY` 开关及操作类型校验并正规化输入地址，输出传输层统一使用的 [`InetEndpoint`]。
    ///
    /// 正规化规则：
    /// 1. 校验输入地址族与套接字协议族是否匹配（双栈 UDP 在非 `Bind` 操作下额外允许直接传入 `AF_INET` 地址）。
    /// 2. 拒绝非零 `scope_id`，并将 `::ffff:a.b.c.d`（IPv4-mapped IPv6）统一还原为 `IpAddress::Ipv4` 交由 IPv4 路由与协议栈处理。
    /// 3. 若开启了 `v6_only` 但正规化结果为 IPv4 地址，则按 Linux 规范区分 `Bind`（`InvalidOperation`）与 `Connect`/`Send`（`NetworkUnreachable`）报错。
    /// 4. 在 `Connect` 或 `Send` 操作中，若目标地址为 IPv6 通配地址 `::`，按惯例重写为本地回环地址 `::1`。
    pub fn normalize(
        self,
        family: AddressFamily,
        v6_only: bool,
        operation: AddressOperation,
        udp: bool,
    ) -> Result<InetEndpoint, FsError> {
        // Step 1: 校验套接字族与输入地址变体的组合，并提取初步的 InetEndpoint（同时将 v4-mapped IPv6 转为 IPv4）。
        let mut endpoint = match (family, self) {
            (AddressFamily::Ipv4, Self::Inet(endpoint)) => {
                if !matches!(endpoint.address, IpAddress::Ipv4(_)) {
                    return Err(FsError::AddressFamilyNotSupported);
                }
                endpoint
            }
            (AddressFamily::Ipv6, Self::Inet(endpoint))
                if udp && operation != AddressOperation::Bind =>
            {
                if !matches!(endpoint.address, IpAddress::Ipv4(_)) {
                    return Err(FsError::AddressFamilyNotSupported);
                }
                if v6_only {
                    return Err(match operation {
                        AddressOperation::Connect => FsError::AddressFamilyNotSupported,
                        AddressOperation::Send => FsError::NetworkUnreachable,
                        AddressOperation::Bind => unreachable!(),
                    });
                }
                endpoint
            }
            (
                AddressFamily::Ipv6,
                Self::Inet6 {
                    endpoint, scope_id, ..
                },
            ) => {
                if scope_id != 0 {
                    return Err(FsError::AddressNotAvailable);
                }
                let IpAddress::Ipv6(address) = endpoint.address else {
                    return Err(FsError::AddressFamilyNotSupported);
                };
                InetEndpoint {
                    address: match address.to_ipv4_mapped() {
                        Some(address) => IpAddress::Ipv4(address),
                        None => IpAddress::Ipv6(address),
                    },
                    port: endpoint.port,
                }
            }
            _ => return Err(FsError::AddressFamilyNotSupported),
        };

        // Step 2: 若 IPv6 套接字启用了 IPV6_V6ONLY，拒绝一切转换后落入 IPv4 的目标或绑定地址。
        if family == AddressFamily::Ipv6
            && v6_only
            && matches!(endpoint.address, IpAddress::Ipv4(_))
        {
            return Err(match operation {
                AddressOperation::Bind => FsError::InvalidOperation,
                AddressOperation::Connect | AddressOperation::Send => FsError::NetworkUnreachable,
            });
        }

        // Step 3: 非 Bind 操作（connect/sendto）将未指定的 IPv6 全零地址 `::` 定向到本机回环 `::1`。
        if operation != AddressOperation::Bind
            && endpoint.address == IpAddress::Ipv6(Ipv6Address::UNSPECIFIED)
        {
            endpoint.address = IpAddress::Ipv6(Ipv6Address::LOCALHOST);
        }
        Ok(endpoint)
    }

    /// 将 [`SocketAddress`] 按 Linux `sockaddr` ABI 二进制格式编码写入 `target`。
    ///
    /// 若 `target` 容量小于完整地址结构长度，则只写入可容纳的前缀字节（安全截断），
    /// 并始终返回未截断前的完整结构体字节长度，供 `getsockname` / `accept` / `recvfrom` 回写用户的 `socklen_t`。
    pub fn encode(self, target: &mut [u8]) -> usize {
        let mut bytes = [0u8; MAX_ADDRESS_LEN];
        let (family, length) = match self {
            Self::Unspecified => (AF_UNSPEC, ADDRESS_FAMILY_LEN),
            Self::UnixUnnamed => (AF_UNIX, ADDRESS_FAMILY_LEN),
            Self::Inet(endpoint) => {
                let IpAddress::Ipv4(address) = endpoint.address else {
                    panic!("IPv4 ABI has IPv6 endpoint");
                };
                bytes[2..4].copy_from_slice(&endpoint.port.to_be_bytes());
                bytes[4..8].copy_from_slice(&address.octets());
                (AF_INET, INET_ADDRESS_LEN)
            }
            Self::Inet6 {
                endpoint,
                flowinfo,
                scope_id,
            } => {
                let IpAddress::Ipv6(address) = endpoint.address else {
                    panic!("IPv6 ABI has IPv4 endpoint");
                };
                bytes[2..4].copy_from_slice(&endpoint.port.to_be_bytes());
                bytes[4..8].copy_from_slice(&flowinfo.to_be_bytes());
                bytes[8..24].copy_from_slice(&address.octets());
                bytes[24..28].copy_from_slice(&scope_id.to_ne_bytes());
                (AF_INET6, INET6_ADDRESS_LEN)
            }
        };

        bytes[..2].copy_from_slice(&family.to_ne_bytes());
        let copied = target.len().min(length);
        target[..copied].copy_from_slice(&bytes[..copied]);
        length
    }
}
