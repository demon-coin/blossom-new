//! 网络接口标识、路由决策结果与接口/路由状态快照数据类型。
//!
//! # 模块职责与架构定位
//! 本模块定义网络协议栈（`crate::net`）与控制入口（`crate::net::control`）、`/proc/net/*` 伪文件系统快照以及传输层端点之间交互的纯值类型（Value Types）。
//! 实际的 IP 地址、CIDR 前缀、默认网关路由表（`Routes`）以及 ARP/NDP 邻居缓存全部保存在 `smoltcp::iface::Interface` 内部；
//! 本模块仅在锁内提取轻量级的只读快照或传递当次路由查询结果，不在内核中维护第二套冗余的路由表状态。
//!
//! # 核心设计与路由快照模型
//!
//! ```text
//! 网络域双接口与路由表快照模型:
//! +-----------------------+---------------------------------------------------+
//! | InterfaceId::Loopback | "lo"   -> 127.0.0.1/8 直连路由                     |
//! | InterfaceId::Ethernet | "eth0" -> 局域网直连 CIDR 路由 + 可选默认网关路由   |
//! +-----------------------+---------------------------------------------------+
//!            |
//!            +--> InterfaceSnapshot (锁内拷贝接口 IPv4 地址、前缀长度与 UP 状态)
//!            +--> RouteSnapshot     (锁内拷贝直连网段或 0.0.0.0/0 默认路由条目)
//!            +--> Route             (单次发包路由选路结果：出站接口、源 IP、路径 MTU)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块仅包含按值拷贝（`Copy`）的数据结构与纯函数校验逻辑，不持有任何锁。
//! - 调用方在持有网络接口睡眠锁（`SleepMutex`）期间构造 [`InterfaceSnapshot`]、[`RouteSnapshot`] 或 [`Route`]，随后即可释放锁并在锁外安全使用。

use smoltcp::wire::IpAddress;
#[cfg(feature = "ktest")]
use smoltcp::wire::{Ipv4Address, Ipv4Cidr};

use crate::fs::FsError;

/// IPv4 地址的二进制位宽（32 位）。
pub(crate) const IPV4_ADDRESS_BITS: u8 = u32::BITS as u8;

/// 路由条目标志位：路由处于启用状态（`RTF_UP`）。
pub(crate) const RTF_UP: u16 = 0x1;
/// 路由条目标志位：目的地址需经由网关转发（`RTF_GATEWAY`）。
pub(crate) const RTF_GATEWAY: u16 = 0x2;

/// IPv4 环回网段（`127.0.0.0/8`）的首字节（127）。
const IPV4_LOOPBACK_NETWORK: u8 = 127;
/// IPv4 组播地址段（`224.0.0.0/4`）的起始首字节（224）。
const IPV4_MULTICAST_FIRST_OCTET: u8 = 224;

/// 网络协议栈支持的网络接口标识。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InterfaceId {
    /// 本地环回接口（`"lo"`）。
    Loopback,
    /// 以太网物理/虚拟网卡接口（`"eth0"`）。
    Ethernet,
}
impl InterfaceId {
    /// 返回 `ioctl` 控制 ABI 与 `/proc/net` 伪文件快照使用的固定接口名称字符串（`"lo"` 或 `"eth0"`）。
    pub fn name(self) -> &'static str {
        match self {
            Self::Loopback => "lo",
            Self::Ethernet => "eth0",
        }
    }

    /// 将已去除末尾 NUL 字节的接口名称切片解析为 [`InterfaceId`]；未知名称返回 [`FsError::NoDevice`]（`ENODEV`）。
    pub fn from_name(name: &[u8]) -> Result<Self, FsError> {
        match name {
            b"lo" => Ok(Self::Loopback),
            b"eth0" => Ok(Self::Ethernet),
            _ => Err(FsError::NoDevice),
        }
    }
}

/// 网络接口 IPv4 地址与管理状态的只读快照。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InterfaceSnapshot {
    /// 接口标识（`lo` 或 `eth0`）。
    pub id: InterfaceId,
    /// 当前配置的 IPv4 地址（网络字节序 4 字节）；若接口尚未配置 IPv4 地址则为 `None`。
    pub address: Option<[u8; 4]>,
    /// IPv4 子网掩码对应的 CIDR 前缀长度（范围 `0..=32`，单位：比特）。
    pub prefix_len: u8,
    /// 接口管理启用状态（`true` 表示 `IFF_UP` 已启用）。
    pub up: bool,
}

/// 单条 IPv4 路由表项（直连网段路由或默认网关路由）的只读快照。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RouteSnapshot {
    /// 该路由绑定的出站网络接口。
    pub interface: InterfaceId,
    /// 目的网段 IPv4 地址（网络字节序 4 字节；默认路由为 `[0, 0, 0, 0]`）。
    pub destination: [u8; 4],
    /// 目的网段的 CIDR 前缀长度（范围 `0..=32`，默认路由为 `0`）。
    pub prefix_len: u8,
    /// 下一跳网关的 IPv4 地址（网络字节序 4 字节；直连路由为 `None`）。
    pub gateway: Option<[u8; 4]>,
    /// 路由所属接口当前是否处于启用（`UP`）状态。
    pub up: bool,
}

/// 添加或删除 IPv4 默认路由（`0.0.0.0/0`）的控制请求参数。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DefaultRouteRequest {
    /// 用户显式指定的出站网络接口（若 `rtentry.rt_dev` 为空指针则为 `None`，默认作用于 `eth0`）。
    pub interface: Option<InterfaceId>,
    /// 默认网关的 IPv4 地址（网络字节序 4 字节）。
    pub gateway: [u8; 4],
}

/// 以太网接口（`eth0`）的静态 IPv4 配置参数（供内核测试初始化或重配接口使用）。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EthernetConfig {
    /// 待配置的接口 IPv4 单播地址（网络字节序 4 字节）。
    pub address: [u8; 4],
    /// 子网掩码 CIDR 前缀长度（范围 `0..=32`，单位：比特）。
    pub prefix_len: u8,
    /// 可选的同网段默认网关 IPv4 地址（网络字节序 4 字节）。
    pub gateway: Option<[u8; 4]>,
}

#[cfg(feature = "ktest")]
impl EthernetConfig {
    /// 校验静态 IPv4 配置的合法性：要求前缀长度不超过 32、接口地址为非环回单播地址，
    /// 且可选网关必须为同 CIDR 子网内不同于本机地址的合法单播地址。
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

/// 单次出站报文的路由选路结果。
///
/// 仅携带出站接口、选定的本地源 IP 地址以及该接口的 IP 层 MTU；下一跳解析与 ARP/NDP 邻居查找由对应接口在发包时完成。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Route {
    /// 选定的出站网络接口。
    pub interface: InterfaceId,
    /// 选定的本地源 IP 地址。
    pub source: IpAddress,
    /// 出站接口支持的 IP 层最大传输单元（单位：字节）。
    pub mtu: usize,
}

/// 判断给定的 4 字节网络字节序 IPv4 地址是否为合法的非环回单播地址（排除 `0.0.0.0/8`、`127.0.0.0/8` 以及 `224.0.0.0/4` 及以上的组播和保留地址）。
pub(crate) fn unicast(address: [u8; 4]) -> bool {
    address[0] != 0
        && address[0] != IPV4_LOOPBACK_NETWORK
        && address[0] < IPV4_MULTICAST_FIRST_OCTET
}
