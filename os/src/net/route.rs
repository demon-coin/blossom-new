//! 路由只携带库接口当次选择的小值，地址、Routes 与邻居状态留在原 Interface。

use smoltcp::wire::{IpAddress, Ipv4Address, Ipv4Cidr};

use crate::fs::FsError;

pub(crate) const IPV4_ADDRESS_BITS: u8 = u32::BITS as u8;
pub(crate) const RTF_UP: u16 = 0x1;
pub(crate) const RTF_GATEWAY: u16 = 0x2;
const IPV4_LOOPBACK_NETWORK: u8 = 127;
const IPV4_MULTICAST_FIRST_OCTET: u8 = 224;

/// 同一协议域的两种实际交付介质。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InterfaceId {
    Loopback,
    Ethernet,
}

impl InterfaceId {
    /// 返回控制 ABI 和 proc 快照共同使用的固定接口名。
    pub fn name(self) -> &'static str {
        match self {
            Self::Loopback => "lo",
            Self::Ethernet => "eth0",
        }
    }

    /// 解析已在 ABI 边界去除 NUL 的接口名字；设备是否存在由域查询。
    pub fn from_name(name: &[u8]) -> Result<Self, FsError> {
        match name {
            b"lo" => Ok(Self::Loopback),
            b"eth0" => Ok(Self::Ethernet),
            _ => Err(FsError::NoDevice),
        }
    }
}

/// 当前库 IPv4 地址与接口管理状态的无拥有副本。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InterfaceSnapshot {
    pub id: InterfaceId,
    pub address: Option<[u8; 4]>,
    pub prefix_len: u8,
    pub up: bool,
}

/// 直连 CIDR 或库默认项的一次只读副本。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RouteSnapshot {
    pub interface: InterfaceId,
    pub destination: [u8; 4],
    pub prefix_len: u8,
    pub gateway: Option<[u8; 4]>,
    pub up: bool,
}

/// 默认路由控制只携带已复制的接口身份和 IPv4 网关。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DefaultRouteRequest {
    pub interface: Option<InterfaceId>,
    pub gateway: [u8; 4],
}

/// 控制入口的一次 IPv4 配置值，提交后由库接口保存真实配置。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EthernetConfig {
    pub address: [u8; 4],
    pub prefix_len: u8,
    pub gateway: Option<[u8; 4]>,
}

impl EthernetConfig {
    /// 在写入库接口前核静态配置边界，不创建第二个路由或地址集合。
    pub fn validate(self) -> Result<(), FsError> {
        if IPV4_ADDRESS_BITS < self.prefix_len || !unicast(self.address) {
            return Err(FsError::InvalidOperation);
        }
        if let Some(gateway) = self.gateway {
            if !unicast(gateway)
                || gateway == self.address
                || !Ipv4Cidr::new(Ipv4Address::from(self.address), self.prefix_len)
                    .contains_addr(&Ipv4Address::from(gateway))
            {
                return Err(FsError::InvalidOperation);
            }
        }
        Ok(())
    }
}

/// 一次库路由选择的源和 MTU，下一跳及邻居解析继续由同一库接口负责。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Route {
    pub interface: InterfaceId,
    pub source: IpAddress,
    pub mtu: usize,
}

/// 当前静态 eth0 控制入口只接受非回环 IPv4 单播值。
pub(crate) fn unicast(address: [u8; 4]) -> bool {
    address[0] != 0
        && address[0] != IPV4_LOOPBACK_NETWORK
        && address[0] < IPV4_MULTICAST_FIRST_OCTET
}
