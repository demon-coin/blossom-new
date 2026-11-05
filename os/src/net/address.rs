//! 地址保存库 IP 小值，用户 sockaddr 的族、长度和字节序只在 ABI 边界转换。

use core::convert::{TryFrom, TryInto};
use smoltcp::wire::{IpAddress, IpEndpoint, IpListenEndpoint, Ipv4Address, Ipv6Address};

use crate::fs::FsError;

pub const AF_UNSPEC: u16 = 0;
pub const AF_UNIX: u16 = 1;
pub const AF_INET: u16 = 2;
pub const AF_INET6: u16 = 10;

pub const INET_ADDRESS_LEN: usize = 16;
pub const INET6_ADDRESS_LEN: usize = 28;
pub const MAX_ADDRESS_LEN: usize = INET6_ADDRESS_LEN;
pub(crate) const ADDRESS_FAMILY_LEN: usize = core::mem::size_of::<u16>();

/// 打开对象的地址族不随 mapped 报文使用的实际 IP 协议改变。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AddressFamily {
    Ipv4,
    Ipv6,
}

/// 地址输入按实际操作区分族和 V6ONLY 的 Linux 错误返回。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AddressOperation {
    Bind,
    Connect,
    Send,
}

impl AddressFamily {
    /// 返回 Linux socket ABI 的地址族编号。
    pub fn number(self) -> u16 {
        match self {
            Self::Ipv4 => AF_INET,
            Self::Ipv6 => AF_INET6,
        }
    }

    /// 自动绑定保持打开对象的原生通配地址。
    pub fn unspecified(self) -> IpAddress {
        match self {
            Self::Ipv4 => IpAddress::Ipv4(Ipv4Address::UNSPECIFIED),
            Self::Ipv6 => IpAddress::Ipv6(Ipv6Address::UNSPECIFIED),
        }
    }
}

/// IP 地址直接使用库类型，端口在内核中使用主机顺序。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InetEndpoint {
    pub address: IpAddress,
    pub port: u16,
}

impl InetEndpoint {
    /// 判断本域通配绑定；源路由选择不能改变这个地址值。
    pub fn is_unspecified(self) -> bool {
        self.address.is_unspecified()
    }

    /// 将明确的端点交给协议核心，不在转换时选择地址或端口。
    pub fn raw(self) -> IpEndpoint {
        IpEndpoint::new(self.address, self.port)
    }

    /// 监听通配地址不限制 raw 收包族，接收面由共同绑定索引约束。
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

    /// 将 raw 端点恢复为不借用协议存储的内核小值。
    pub fn from_raw(endpoint: IpEndpoint) -> Self {
        Self {
            address: endpoint.addr,
            port: endpoint.port,
        }
    }
}

/// IPv6 ABI 在解码阶段保留 mapped、flowinfo 和 scope，进入协议前集中正规化。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SocketAddress {
    Unspecified,
    Inet(InetEndpoint),
    Inet6 {
        endpoint: InetEndpoint,
        flowinfo: u32,
        scope_id: u32,
    },
    UnixUnnamed,
}

impl SocketAddress {
    /// 解码已经复制的完整 sockaddr；UNIX 仅接受匿名两字节名称。
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

    /// 按打开对象地址族生成名称，mapped 核心仍输出完整 IPv6 ABI。
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

    /// 集中校验输入族和 scope，再将 mapped 地址交给唯一 IPv4 核心。
    /// Bind 保留原生 :: 通配值，Connect 与 Send 选择本地 loopback 目的。
    pub fn normalize(
        self,
        family: AddressFamily,
        v6_only: bool,
        operation: AddressOperation,
        udp: bool,
    ) -> Result<InetEndpoint, FsError> {
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

        if family == AddressFamily::Ipv6
            && v6_only
            && matches!(endpoint.address, IpAddress::Ipv4(_))
        {
            return Err(match operation {
                AddressOperation::Bind => FsError::InvalidOperation,
                AddressOperation::Connect | AddressOperation::Send => FsError::NetworkUnreachable,
            });
        }

        if operation != AddressOperation::Bind
            && endpoint.address == IpAddress::Ipv6(Ipv6Address::UNSPECIFIED)
        {
            endpoint.address = IpAddress::Ipv6(Ipv6Address::LOCALHOST);
        }
        Ok(endpoint)
    }

    /// 只复制输出容量可容纳的前缀，返回完整结构长度供 socklen 回写。
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
