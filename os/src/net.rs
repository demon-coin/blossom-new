//! 内核网络协议栈根服务（`NetStack`）与网络接口调度模块。
//!
//! ## 模块职责与架构定位
//! 本模块是内核网络子系统的顶层协调者，连接上层系统调用/VFS 套接字层与底层网卡驱动：
//! - **向上（`syscall` / `net::socket` / `procfs`）**：提供创建 TCP/UDP/UNIX 端点、查询与配置网络接口（`lo` / `eth0`）IPv4 地址及默认路由、
//!   读取接口流量统计等统一入口。整个内核运行期共享同一个 `NetStack` 实例（通过 [`NetStackRef`] 克隆引用，由唯一的 [`NetStackOwner`] 负责同步停机）。
//! - **向下（`drivers::net_device::NetDevice` 与 `smoltcp::iface::Interface`）**：驱动本地回环接口（`lo`，IP 介质）与物理/虚拟以太网接口（`eth0`，Ethernet 介质），
//!   完成 ARP/NDP 邻居解析、IP 分片与重组、ICMP 控制报文自动应答，以及物理网卡 DMA 收发帧的提交与接收。
//!
//! ## 核心数据流与解耦架构
//! 本子系统采用**“接口层线格式解析”与“连接级协议状态机”解耦**的设计，不把所有 TCP/UDP 套接字塞进同一个全局 `SocketSet` 大锁：
//! ```text
//!   [用户态 send/connect]                                      [网卡 RX Work / 回环交换]
//!            |                                                            |
//!            v (连接级 SleepMutex)                                         v (接口级 SleepMutex)
//!   TcpEndpoint / UdpEndpoint                                  NetStack::receive_ethernet / lo
//!            | 构造 OwnedPacket                                           | smoltcp::Interface 解析线格式
//!            v                                                            | ARP/ICMP 应答写入 ingress_reply 槽
//!   NetStack::try_emit()                                                  v
//!            | 挂入 Egress 队列 (最多 32 包)                     IngressCollector (实现 TransportIngress)
//!            v                                                            | 查 NetTables 匹配端点 Arc + 捕获载荷
//!   ingress Work (receive_egress, 预算 16)                                 v (释放接口锁后投递)
//!     +------+-----------------------+                         endpoint.enqueue(OwnedPacket)
//!     | lo: 内存交换 frame <-> rx     | eth0: dispatch_ip 编码帧           |
//!     v                              v                                    v
//!   回环直接进入解析链          NetDevice::try_transmit()      唤醒目标端点专属 Work / 等待队列
//! ```
//! - **双帧槽隔离（`eth0`）**：以太网接口同时配备普通出站帧槽（`frame`）和专用于入站控制应答（如 ARP Reply、ICMP Echo Reply、无匹配端口的 TCP RST）
//!   的应答帧槽（`ingress_reply`）。即使普通出站方向正在等待 IP 分片后续帧，也不会阻塞网卡接收和即时控制应答。
//! - **安全停机生命周期**：[`NetStackOwner::drop`] 在可睡眠任务上下文中调用 `stop_sync()`，按“关闭新请求准入 -> 同步取消并等待根定时器与根 Work 退出 ->
//!   逐个同步停止所有活跃与退休端点 -> 停止网卡 DMA 与中断 -> 排空残留缓冲并关闭工作队列”的严格顺序回收资源，确保异步回调彻底执行完毕并退出后才释放内存。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 多把锁嵌套获取时必须严格遵循以下层级（从外到内，严禁逆序）：
//! 1. `NetStack::admission`（`SleepMutex`）：串行化接口配置变更、端点创建与网络栈停机，**绝不在协议收发路径上持有**。
//! 2. 连接级操作锁（如 `TcpEndpoint` 的 `SleepMutex`）：单连接协议推进与用户读写互斥。
//! 3. 接口状态锁 `NetStack::loopback` -> `NetStack::ethernet`（`SleepMutex`）：保护各自的 `InterfaceState`；路径解析需跨接口时先锁 `loopback`、释放后再锁 `ethernet`。
//! 4. 叶子自旋锁（`SpinLock`，持锁时间极短、不可睡眠、彼此之间不互相嵌套调用外部回调）：
//!    - `NetTables` 内部表锁（仅在 `IngressCollector` 查表或增删端点/端口租约时短暂获取）；
//!    - `NetStack::egress`（保护出站报文队列 `packets` 与容量等待链表 `waiting`）；
//!    - `NetStack::retired`（保护待异步回收的退休端点链表）；
//!    - `NetStack::timer_control`（串行化接口定时器重装填与停机取消）；
//!    - `NetStack::loopback_stats` 与各接口 `CaptureStorage` 自旋锁。

pub mod address;
pub(crate) mod control;
pub mod core;
mod lifecycle;
pub mod packet;
pub mod route;
pub mod socket;
pub mod stats;
pub mod table;
pub mod tcp;
pub mod udp;
pub mod unix;

pub use crate::net::unix::UnixSocketType;

use ::core::convert::TryFrom;
use ::core::mem::ManuallyDrop;
use ::core::pin::Pin;
use ::core::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use alloc::collections::VecDeque;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use smoltcp::iface::{
    BufferError, Config, DispatchError, Interface, InterfaceBuffers, InterfaceInner,
    TransportIngress, TransportResponse,
};
use smoltcp::phy::{Device, DeviceCapabilities, Medium, PacketMeta, RxToken, TxToken};
use smoltcp::time::Instant;
use smoltcp::wire::{
    EthernetAddress, HardwareAddress, IpAddress, IpCidr, IpRepr, Ipv4Address, Ipv4Cidr,
    Ipv6Address, Ipv6Cidr, TcpRepr, UdpRepr, ETHERNET_HEADER_LEN,
};
#[cfg(feature = "ktest")]
use smoltcp::wire::{IpProtocol, TcpControl};
use spin::Once;

use crate::cpu;
use crate::drivers::net_device::{NetDevice, NetDeviceError};
use crate::drivers::DeviceInventory;
use crate::fs::FsError;
use crate::net::address::{AddressFamily, InetEndpoint, SocketAddress};
use crate::net::core::TcpCore;
pub(crate) use crate::net::lifecycle::EgressWait;
use crate::net::lifecycle::{EgressWaiting, RetiredEndpoint, RetiredEndpoints};
use crate::net::packet::{
    return_capture_storage, zeroed_bytes, CaptureStorage, OwnedPacket, MAX_IP_PACKET_LEN,
};
#[cfg(feature = "ktest")]
use crate::net::route::EthernetConfig;
use crate::net::route::{
    unicast, DefaultRouteRequest, InterfaceId, InterfaceSnapshot, Route, RouteSnapshot,
    IPV4_ADDRESS_BITS,
};
use crate::net::stats::InterfaceStats;
use crate::net::table::{BindLease, NetTables, TcpTuple};
use crate::net::tcp::TcpEndpoint;
use crate::net::udp::UdpEndpoint;
use crate::net::unix::{UnixEndpoint, UnixPair};
use crate::sync::{SleepMutex, SleepMutexGuard, SpinLock};
use crate::time::{self, NSEC_PER_USEC};
use crate::timer::{Timer, TimerKind};
use crate::workqueue::{Work, WorkQueue, WorkQueueConfig};

/// 全局软件出站队列（`Egress::packets`）可容纳的最大待发送 [`OwnedPacket`] 数量。
pub const LOOPBACK_CAPACITY: usize = 32;
/// 本地回环接口（`lo`）及内核软件 IP 报文的最大传输单元（65535 字节）。
pub const LOOPBACK_MTU: usize = MAX_IP_PACKET_LEN;
/// `listen` 系统调用允许的最大连接积压队列上限（对应 Linux `SOMAXCONN`）。
pub(crate) const SOMAXCONN: usize = 4096;
/// 单次后台 `Work` 调度循环最多处理的报文或退休端点数量预算，防止高负载下饿死同队列其他任务。
pub(crate) const PROTOCOL_BUDGET: usize = 16;

/// 标准以太网接口（`eth0`）的三层 IP 最大传输单元（1500 字节）。
const ETHERNET_IP_MTU: usize = 1500;
/// 标准以太网二层帧缓冲容量（1500 字节 IP MTU + 14 字节以太网帧头 = 1514 字节，不含驱动私有的 VirtIO 头部）。
pub(crate) const ETHERNET_FRAME_CAPACITY: usize = ETHERNET_IP_MTU + ETHERNET_HEADER_LEN;
/// 每个网络接口分配给 `smoltcp` IP 分片器与分片重组器的工作区字节容量（64 KiB）。
const INTERFACE_WORKSPACE_CAPACITY: usize = 64 * 1024;

/// 回环接口默认 IPv4 地址 `127.0.0.1/8` 的前缀长度（8 位）。
const LOOPBACK_IPV4_PREFIX_LEN: u8 = 8;
/// 点对点 IPv4 链路前缀长度阈值（`/31`）；前缀小于 31 位的子网需排除全 0 网络地址与全 1 广播地址作为网关。
const IPV4_POINT_TO_POINT_PREFIX_LEN: u8 = IPV4_ADDRESS_BITS - 1;

/// `NetStack` 自身常驻的根后台任务数量（`ingress` 收发推进任务 + `reaper` 退休端点回收任务）。
const ROOT_WORK_ITEMS: usize = 2;
/// 网络栈管理的网络接口槽位总数（`lo` 与 `eth0` 共 2 个）。
pub(crate) const INTERFACE_COUNT: usize = 2;
/// `/proc/net/route` 快照数组容量（2 条接口直连子网路由 + 1 条 `eth0` 默认网关路由）。
const ROUTE_SNAPSHOT_CAPACITY: usize = INTERFACE_COUNT + 1;

/// 接口层单帧预分配编码缓冲槽。
///
/// 在接口初始化时一次性分配底层字节数组，使协议栈线格式序列化过程中无需动态分配堆内存；
/// 当缓冲槽中已有待发数据（`len > 0`）或正在锁外提交给网卡驱动（`in_flight`）时，不再向 `smoltcp` 发放新的 `TxToken`。
struct FrameSlot {
    /// 预分配的帧字节存储（回环接口为 `LOOPBACK_MTU` 字节，以太网接口为 `ETHERNET_FRAME_CAPACITY` 字节）。
    bytes: Vec<u8>,
    /// 当前槽中已编码待发送的有效帧字节长度；`0` 表示当前无待发帧。
    len: usize,
    /// 标记 `bytes` 是否已被临时移出接口锁、正在向底层网卡驱动执行 `try_transmit` 提交。
    in_flight: bool,
    /// 回环介质下记录该帧的三层目的 IP 地址，用于在回环接收前校验目标接口状态。
    destination: Option<IpAddress>,
}

impl FrameSlot {
    /// 预分配指定字节容量的零初始化帧缓冲槽；内存不足时返回 `FsError::NoMemory`。
    fn try_new(capacity: usize) -> Result<Self, FsError> {
        Ok(Self {
            bytes: zeroed_bytes(capacity)?,
            len: 0,
            in_flight: false,
            destination: None,
        })
    }

    /// 判断当前帧槽是否为空闲可用状态（既无积压待发数据，也未处于锁外网卡提交途中）。
    fn available(&self) -> bool {
        self.len == 0 && !self.in_flight
    }
}

/// 标识以太网待发送帧来自哪个缓冲槽，以便在锁外提交给网卡后精确更新对应槽位状态。
#[derive(Clone, Copy, Eq, PartialEq)]
enum EthernetFrameSource {
    /// 入站报文解析期间由协议栈自动生成的即时控制应答帧（如 ARP Reply、ICMP Echo Reply、TCP RST）。
    IngressReply,
    /// 来自 `Egress` 队列的常规出站业务报文（或 IP 分片帧），发送完成后需释放关联的端口绑定租约并归还队列槽位。
    Egress,
}

/// 适配 `smoltcp::phy::Device` 特性的轻量级内存帧设备视图。
///
/// 每次在接口锁内推进协议栈收发时临时在栈上构造，将最多一帧待解析输入切片（`input`）与一个预分配输出槽（`frame`）配对交给 `smoltcp`。
struct FrameDevice<'a> {
    /// 供 `smoltcp` 输出一帧编码数据（正常出站帧或入站控制应答帧）的目标缓冲槽。
    frame: &'a mut FrameSlot,
    /// 待 `smoltcp` 解析的单帧输入字节切片（一旦被 `receive` 取走即变为 `None`）。
    input: Option<&'a [u8]>,
    /// 接口的链路介质类型（回环接口为 `Medium::Ip`，以太网接口为 `Medium::Ethernet`）。
    medium: Medium,
}

/// 借用单帧输入字节切片的 `smoltcp` 接收令牌。
struct FrameRx<'a>(&'a [u8]);
/// 独占借用空闲 [`FrameSlot`] 的 `smoltcp` 发送令牌。
struct FrameTx<'a>(&'a mut FrameSlot);

impl RxToken for FrameRx<'_> {
    /// 将输入帧切片同步交由 `smoltcp` 闭包解析；借用仅在接口锁保护的当次调用内有效。
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(self.0)
    }
}

impl TxToken for FrameTx<'_> {
    /// 在预分配的空闲 [`FrameSlot`] 前缀切片上直接编码长度为 `len` 的输出帧，并记录有效帧长。
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        assert!(
            self.0.available() && len <= self.0.bytes.len(),
            "invalid interface TX reservation"
        );
        let result = f(&mut self.0.bytes[..len]);
        self.0.len = len;
        result
    }
}

impl Device for FrameDevice<'_> {
    type RxToken<'a>
        = FrameRx<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = FrameTx<'a>
    where
        Self: 'a;

    /// 仅当配对输出槽空闲且存在待处理输入帧时，向 `smoltcp` 交出收发令牌对，确保解析过程中产生的 ARP/ICMP/RST 应答必定有槽位承载。
    fn receive(&mut self, _: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if !self.frame.available() {
            return None;
        }
        Some((FrameRx(self.input.take()?), FrameTx(self.frame)))
    }

    /// 当输出槽空闲时交出发送令牌；若槽位已满则返回 `None`，使 `smoltcp` IP 分片器继续保留尚未发出的后续分片。
    fn transmit(&mut self, _: Instant) -> Option<Self::TxToken<'_>> {
        self.frame.available().then_some(FrameTx(self.frame))
    }

    /// 报告虚拟帧设备的介质类型、单次突发上限（1 帧）及链路层最大传输单元（以太网含 14 字节二层头，驱动私有头由底层网卡驱动自行处理）。
    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = self.medium;
        caps.max_transmission_unit = match self.medium {
            Medium::Ip => LOOPBACK_MTU,
            Medium::Ethernet => ETHERNET_FRAME_CAPACITY,
        };
        caps.max_burst_size = Some(1);
        caps
    }
}

/// 单个网络接口（`lo` 或 `eth0`）的协议栈实例、缓冲工作区与运行状态。
///
/// 由 `NetStack` 中对应接口的 `SleepMutex` 独占保护。
struct InterfaceState {
    /// `smoltcp` 网络层接口实例（管理 IP 地址列表、路由表、ARP/NDP 邻居缓存及 IP 分片/重组状态）。
    iface: Interface,
    /// 常规出站报文（及回环控制应答）使用的主编码帧槽。
    frame: FrameSlot,
    /// 以太网接口专用的入站控制应答帧槽（回环接口无需此槽，为 `None`），避免常规发送阻塞入站应答。
    ingress_reply: Option<FrameSlot>,
    /// 回环接口专用的接收缓冲数组（长度 `LOOPBACK_MTU`），通过与 `frame.bytes` 零拷贝交换指针完成本地回环收发；以太网接口为空 `Vec`。
    loopback_rx: Vec<u8>,
    /// 接口链路层介质类型（`Medium::Ip` 或 `Medium::Ethernet`）。
    medium: Medium,
    /// 接口管理启用状态（对应 `IFF_UP`）；`lo` 默认启用，`eth0` 默认关闭直至显式配置启用。
    up: bool,
    /// 当前正在 `frame`（或后续 IP 分片）中发送的出站报文所携带的源端口绑定租约，防止报文彻底发出前源端口被提前回收复用。
    binding: Option<Arc<BindLease>>,
    /// 入站有效载荷捕获缓冲池句柄（单槽复用 `Vec<u8>`），减少收包路径上的频繁堆内存分配。
    capture: CaptureStorage,
}

impl InterfaceState {
    /// 预分配接口所需的全部帧缓冲、分片/重组工作区与捕获缓冲，并初始化 `smoltcp::Interface`。
    ///
    /// 若 `medium` 为 `Medium::Ip`（回环接口），会自动配置 `127.0.0.1/8` 与 `::1/128` 地址并将管理状态设为 `up = true`。
    fn try_new(medium: Medium, hardware: HardwareAddress, seed: u64) -> Result<Self, FsError> {
        let capacity = if medium == Medium::Ip {
            LOOPBACK_MTU
        } else {
            ETHERNET_FRAME_CAPACITY
        };
        let mut frame = FrameSlot::try_new(capacity)?;
        let ingress_reply = if medium == Medium::Ethernet {
            Some(FrameSlot::try_new(ETHERNET_FRAME_CAPACITY)?)
        } else {
            None
        };
        let buffers =
            InterfaceBuffers::try_new(INTERFACE_WORKSPACE_CAPACITY, INTERFACE_WORKSPACE_CAPACITY)
                .map_err(|error| match error {
                BufferError::NoMemory => FsError::NoMemory,
                BufferError::InvalidCapacity => FsError::InvalidOperation,
            })?;
        let loopback_rx = if medium == Medium::Ip {
            zeroed_bytes(LOOPBACK_MTU)?
        } else {
            Vec::new()
        };
        let capture = Arc::try_new(SpinLock::new(prepare_ingress_storage()?))
            .map_err(|_| FsError::NoMemory)?;

        let mut config = Config::new(hardware);
        config.random_seed = seed;
        let mut device = FrameDevice {
            frame: &mut frame,
            input: None,
            medium,
        };
        let mut iface = Interface::new(config, &mut device, interface_now(), buffers);
        if medium == Medium::Ip {
            iface.update_ip_addrs(|addresses| {
                addresses
                    .push(IpCidr::Ipv4(Ipv4Cidr::new(
                        Ipv4Address::LOCALHOST,
                        LOOPBACK_IPV4_PREFIX_LEN,
                    )))
                    .expect("loopback IPv4 capacity exhausted");
                addresses
                    .push(IpCidr::Ipv6(Ipv6Cidr::new(
                        Ipv6Address::LOCALHOST,
                        u128::BITS as u8,
                    )))
                    .expect("loopback IPv6 capacity exhausted");
            });
        }

        Ok(Self {
            iface,
            frame,
            ingress_reply,
            loopback_rx,
            medium,
            up: medium == Medium::Ip,
            binding: None,
            capture,
        })
    }

    /// 判断接口当前是否可以接纳下一份新的出站 IP 报文（要求接口处于 `up` 状态、主帧槽空闲，且没有尚未发完的 IP 分片）。
    fn dispatch_ready(&self) -> bool {
        self.up && self.frame.available() && !self.iface.fragmentation_pending()
    }

    /// 判断以太网接口是否具备接收下一帧入站报文的条件（要求接口 `up` 且专用 `ingress_reply` 应答帧槽空闲）。
    fn ethernet_ingress_ready(&self) -> bool {
        self.up
            && self
                .ingress_reply
                .as_ref()
                .is_some_and(FrameSlot::available)
    }

    /// 从 `smoltcp::Interface` 当前地址列表中查询首个已配置的 IPv4 地址前缀（CIDR）。
    fn ipv4(&self) -> Option<Ipv4Cidr> {
        self.iface.ip_addrs().iter().find_map(|cidr| match cidr {
            IpCidr::Ipv4(cidr) => Some(*cidr),
            _ => None,
        })
    }

    /// 提取当前接口的只读状态快照（IPv4 地址、前缀长度与 `up` 标志），供 `ioctl` 与 `/proc` 查询使用。
    fn snapshot(&self, id: InterfaceId) -> InterfaceSnapshot {
        let cidr = self.ipv4();
        InterfaceSnapshot {
            id,
            address: cidr.map(|cidr| cidr.address().octets()),
            prefix_len: cidr.map_or(0, |cidr| cidr.prefix_len()),
            up: self.up,
        }
    }

    /// 替换接口上的 IPv4 地址与前缀长度，同时完整保留原有的 IPv6 地址（如 `::1`）及邻居缓存状态。
    fn set_ipv4(&mut self, address: [u8; 4], prefix_len: u8) {
        self.iface.update_ip_addrs(|addresses| {
            addresses.retain(|cidr| !matches!(cidr, IpCidr::Ipv4(_)));
            addresses
                .push(IpCidr::Ipv4(Ipv4Cidr::new(
                    Ipv4Address::from(address),
                    prefix_len,
                )))
                .expect("interface IPv4 capacity exhausted");
        });
    }

    /// 从接口底层路由表中读取当前配置的 IPv4 默认网关（`0.0.0.0/0` 路由的下一跳地址）。
    fn default_gateway(&self) -> Option<[u8; 4]> {
        self.iface
            .routes()
            .iter()
            .find_map(|route| match route.cidr {
                IpCidr::Ipv4(cidr) if cidr.prefix_len() == 0 => match route.via_router {
                    IpAddress::Ipv4(address) => Some(address.octets()),
                    _ => None,
                },
                _ => None,
            })
    }
}

/// 接口锁内解析入站帧时捕获到的待投递传输层事件。
///
/// 解析阶段仅匹配目标端点引用并构造自包含的 [`OwnedPacket`]，真正的连接入队与等待任务唤醒推迟到释放接口锁之后执行。
enum IngressEvent {
    /// 匹配到已连接或监听中的 TCP 端点及其入站报文。
    Tcp(Arc<TcpEndpoint>, OwnedPacket),
    /// 匹配到绑定的 UDP 端点及其入站数据报。
    Udp(Arc<UdpEndpoint>, OwnedPacket),
}

/// 实现 `smoltcp::iface::TransportIngress` 的入站传输层报文收集器。
struct IngressCollector<'a> {
    /// 网络域全局端点与端口查找表引用。
    tables: &'a NetTables,
    /// 预先在接口锁外准备好的载荷拷贝缓冲区（容量至少为 `LOOPBACK_MTU`）；若当次捕获了非空载荷则被移入 `OwnedPacket`。
    storage: Option<Vec<u8>>,
    /// 载荷缓冲区归属的接口复用池句柄，随 `OwnedPacket` 传递以便在报文消费完毕后自动归还内存。
    capture: CaptureStorage,
    /// 当次单帧解析捕获到的唯一传输层投递事件（单帧最多产生一个传输层报文）。
    event: Option<IngressEvent>,
    /// 回环路径下在接口锁外预先核定的本机目的 IP 地址（以太网路径下为 `None`，由接口自身地址列表判断）。
    local_destination: Option<IpAddress>,
}

impl<'a> IngressCollector<'a> {
    /// 使用接口锁外预先获取的载荷缓冲区构造入站收集器。
    fn new(tables: &'a NetTables, capture: CaptureStorage, storage: Vec<u8>) -> Self {
        Self {
            tables,
            storage: Some(storage),
            capture,
            event: None,
            local_destination: None,
        }
    }

    /// 若当次解析未消耗预备的载荷缓冲区（例如收到空载荷 ACK、ARP/ICMP 报文或无匹配端口），将其归还给接口复用池。
    fn return_storage(&mut self) {
        if let Some(storage) = self.storage.take() {
            return_capture_storage(&self.capture, storage);
        }
    }

    /// 在释放接口锁之后，将捕获到的报文正式推入目标 TCP 或 UDP 端点的接收队列并唤醒等待者。
    fn deliver(self) -> Result<(), FsError> {
        match self.event {
            Some(IngressEvent::Tcp(endpoint, packet)) => endpoint.enqueue(packet),
            Some(IngressEvent::Udp(endpoint, packet)) => endpoint.enqueue(packet),
            None => Ok(()),
        }
    }
}

impl<'s> TransportIngress<'s> for IngressCollector<'_> {
    /// 返回回环接口预先核定的额外本机目的地址（使发往本机 `eth0` IP 但经由 `lo` 回环的报文也能被 `lo` 接口正常接纳）。
    fn local_destination(&self) -> Option<IpAddress> {
        self.local_destination
    }

    /// 处理解析出的入站 TCP 报文段：先按完整四元组查找已建立连接，未命中时再查找本地监听套接字。
    ///
    /// 若命中端点，则将报文头与有效载荷捕获为 [`OwnedPacket`] 暂存于 `self.event`，并返回 `Matched(None)` 告知 `smoltcp` 无需介入；
    /// 若无任何端点匹配，则返回 `Unmatched`，交由 `smoltcp` 自动构造并回复标准 TCP `RST` 段。
    fn tcp<'p>(
        &mut self,
        _: &mut InterfaceInner,
        _: PacketMeta,
        ip: &IpRepr,
        repr: &TcpRepr<'p>,
    ) -> TransportResponse<'p> {
        let peer = InetEndpoint {
            address: ip.src_addr(),
            port: repr.src_port,
        };
        let local = InetEndpoint {
            address: ip.dst_addr(),
            port: repr.dst_port,
        };
        let endpoint = self
            .tables
            .lookup_tcp(TcpTuple { local, peer })
            .or_else(|| self.tables.lookup_listener(local));

        match endpoint {
            Some(endpoint) => {
                assert!(
                    self.event.is_none(),
                    "multiple transports in one ingress frame"
                );
                let storage = if repr.payload.is_empty() {
                    Vec::new()
                } else {
                    self.storage
                        .take()
                        .expect("ingress capture storage missing")
                };
                let packet = OwnedPacket::capture_tcp(ip, repr, storage);
                let packet = if repr.payload.is_empty() {
                    packet
                } else {
                    packet.with_capture_storage(self.capture.clone())
                };
                self.event = Some(IngressEvent::Tcp(endpoint, packet));
                TransportResponse::Matched(None)
            }
            None => TransportResponse::Unmatched,
        }
    }

    /// 处理解析出的入站 UDP 数据报：在端口表中查找匹配目的地址与源过滤条件的 UDP 端点。
    ///
    /// 若命中端点，则捕获载荷构造 [`OwnedPacket`] 并返回 `true`；若未命中则返回 `false`，由 `smoltcp` 自动回复 ICMP 端口不可达报文。
    fn udp(
        &mut self,
        _: &mut InterfaceInner,
        _: PacketMeta,
        ip: &IpRepr,
        repr: &UdpRepr,
        payload: &[u8],
    ) -> bool {
        let peer = InetEndpoint {
            address: ip.src_addr(),
            port: repr.src_port,
        };
        let local = InetEndpoint {
            address: ip.dst_addr(),
            port: repr.dst_port,
        };
        let Some(endpoint) = self.tables.lookup_udp(local, peer) else {
            return false;
        };

        assert!(
            self.event.is_none(),
            "multiple transports in one ingress frame"
        );
        let storage = if payload.is_empty() {
            Vec::new()
        } else {
            self.storage
                .take()
                .expect("ingress capture storage missing")
        };
        let packet = OwnedPacket::capture_udp(ip, repr, payload, storage);
        let packet = if payload.is_empty() {
            packet
        } else {
            packet.with_capture_storage(self.capture.clone())
        };
        self.event = Some(IngressEvent::Udp(endpoint, packet));
        true
    }

    /// 返回 `None`：内核各传输层端点独立持有协议状态机，不使用 `smoltcp` 集中式的 `SocketSet`。
    fn socket_set(&mut self) -> Option<&mut smoltcp::iface::SocketSet<'s>> {
        None
    }
}

/// 全局共享的网络协议栈实例强引用句柄。
///
/// 内核运行时（Runtime）及所有打开的套接字文件对象均克隆并持有同一份 `NetStackRef`；
/// 停机责任不依赖 `Arc` 引用计数归零，而由唯一的 [`NetStackOwner`] 显式执行。
pub type NetStackRef = Arc<NetStack>;

/// 流式套接字单次非阻塞读写推进结果。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IoProgress {
    /// 本次操作已成功传输指定字节数（`Ready(0)` 仅在调用方请求传输长度为 0 时出现）。
    Ready(usize),
    /// 当前无数据可读、发送缓冲区已满或握手尚未完成，需挂起等待状态变更（非阻塞模式下对应 `EAGAIN` / `EWOULDBLOCK`）。
    Blocked,
    /// 对端已优雅关闭连接（收到 TCP `FIN` 或 UNIX 流对端关闭）且接收缓冲区已读尽，流到达末尾。
    Eof,
}

/// 面向报文或记录的套接字（UDP / UNIX `DGRAM` / UNIX `SEQPACKET`）单次接收结果。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MessageResult {
    /// 实际拷贝到用户态接收缓冲区的字节数（不超过用户提供的缓冲区容量）。
    pub copied: usize,
    /// 该数据报在截断前的完整原始字节长度（配合 `MSG_TRUNC` 标志可向用户态报告真实报文大小）。
    pub original_len: usize,
    /// 发送端套接字地址（如 UDP 对端的 IP 与端口；匿名 UNIX 对等体为 `Some(SocketAddress::UnixUnnamed)`）。
    pub source: Option<SocketAddress>,
    /// 随本次接收返回的辅助消息标志位（如报文被截断时置 `MSG_TRUNC`）。
    pub flags: u32,
}

/// 网络协议栈服务的唯一生命周期拥有者。
///
/// 只能由普通可睡眠内核任务创建并持有；在其作用域结束析构时，负责同步关闭整个网络栈、停止网卡硬件并等待所有后台任务彻底退出。
pub struct NetStackOwner {
    /// 被托管的网络栈根引用；使用 `ManuallyDrop` 包装以确保在 `stop_sync()` 彻底执行完毕之后再释放强引用。
    net: ManuallyDrop<NetStackRef>,
}

impl NetStackOwner {
    /// 借用底层 [`NetStackRef`]，供内核运行时或套接字子系统克隆共享引用（克隆方仅获得访问权，不承担停机责任）。
    pub fn stack(&self) -> &NetStackRef {
        &self.net
    }
}

impl Drop for NetStackOwner {
    /// 在可睡眠任务上下文中同步停止网络栈、等待所有后台定时器/Work 与网卡中断退出，随后释放拥有者持有的根引用。
    fn drop(&mut self) {
        self.net.stop_sync().expect("network device stop failed");
        // SAFETY:
        // 1. `self.net` 在 `try_new_base` 中由有效的 `NetStackRef` 构造，且仅在本 `Drop::drop` 中释放一次；
        // 2. 前置调用 `stop_sync()` 已同步等待根定时器、后台 Work 以及网卡 DMA/中断处理彻底退出，不再有裸指针回调依赖该强引用保活。
        unsafe { ManuallyDrop::drop(&mut self.net) };
    }
}

/// 单个传输层端点（TCP / UDP / UNIX）在网络栈中的存活计数槽位守卫。
///
/// 从端点开始构造直到最后一个强引用（`Arc`）被释放（包括表中活跃期、退休等待回收期及文件描述符持有期）全程持有，析构时自动递减活跃端点计数。
pub struct EndpointSlot {
    /// 指向所属网络栈的弱引用，避免端点反向强引用 `NetStack` 造成引用循环。
    net: Weak<NetStack>,
}

impl Drop for EndpointSlot {
    /// 若所属网络栈尚未销毁，原子递减全局存活端点计数并校验不会发生下溢。
    fn drop(&mut self) {
        if let Some(net) = self.net.upgrade() {
            let previous = net.endpoints.fetch_sub(1, Ordering::AcqRel);
            assert!(0 < previous, "network endpoint slot underflow");
        }
    }
}

/// 全局软件出站报文队列与容量流控等待状态。
///
/// 由 `NetStack::egress` 自旋锁统一保护，保证队列准入检查、满队等待登记与停机拒绝原子串行。
struct Egress {
    /// 出站准入开关；网络栈运行期间为 `true`，开始 `stop_sync()` 时置为 `false` 以拒绝新报文入队。
    open: bool,
    /// 待由后台 `ingress` Work 发往 `lo` 或 `eth0` 的有界 FIFO 队列（容量上限为 `LOOPBACK_CAPACITY` = 32）。
    packets: VecDeque<QueuedPacket>,
    /// 因出站队列满（或接口管理性 `down` 暂停）而挂起等待唤醒的端点等待节点侵入式链表。
    waiting: EgressWaiting,
    /// 内核自测（`ktest`）专用的回环报文拦截与丢包注入控制器。
    #[cfg(feature = "ktest")]
    packet_control: Option<TestPacketControl>,
}

/// 出站队列中的单个槽位项。
///
/// 当后台 `receive_egress` 取出报文送入接口编码时，先将 `packet` 置为 `None` 但保留队列槽位，
/// 直到该帧（含所有 IP 分片）真正完成回环交换或被网卡接纳后才移除槽位，防止网卡繁忙期间软件队列额度被新生产者超额占用。
struct QueuedPacket {
    /// 待发送的软件 IP 报文；若正在被对应接口处理或分片发送中则为 `None`（仅占位保额）。
    packet: Option<OwnedPacket>,
    /// 入队前已解析确定的出站路由（目标接口、源 IP 与路径 MTU）。
    route: Route,
}

/// 内核自测（`ktest`）观察到的单次 TCP 数据段丢包与超时重传时间戳快照。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TestLossObservation {
    /// 已主动丢弃的目标数据段计数（0 或 1）。
    pub dropped: usize,
    /// 被首次丢弃的数据段 TCP 起始序列号（`seq`）。
    pub sequence: Option<i32>,
    /// 观察到相同起始序列号数据段后续重传到达的次数。
    pub retransmissions: usize,
    /// 首次拦截并丢弃该数据段时的单调时钟时间戳（纳秒）。
    pub first_ns: u64,
    /// 首次观察到该序列号重传段到达时的单调时钟时间戳（纳秒）。
    pub repeated_ns: u64,
}

/// 内核自测（`ktest`）专用的本地回环（`lo`）TCP 报文故障注入策略。
#[cfg(feature = "ktest")]
enum TestPacketControl {
    /// 拦截并暂存发往指定目的端口的首个 `SYN` 握手段，用于测试握手未完成时的并发关闭、超时或队列行为。
    HoldSyn {
        /// 需要拦截的目标监听端口号（主机字节序）。
        destination_port: u16,
        /// 已被拦截暂存的原始 `SYN` 报文；尚未拦截或已释放后为 `None`。
        held: Option<OwnedPacket>,
        /// 标记暂存的 `SYN` 报文是否已被测试主动放行回环队列。
        released: bool,
    },
    /// 丢弃指定端口对之间的首个非空 TCP 数据段，并记录后续重传到达的时间与次数。
    DropData {
        /// 匹配的数据发送端源端口（主机字节序）。
        source_port: u16,
        /// 匹配的数据接收端目的端口（主机字节序）。
        destination_port: u16,
        /// 丢包与重传观测统计记录。
        observation: TestLossObservation,
    },
    /// 丢弃指定端口对在三次握手阶段发出的首个纯 `ACK` 段（无载荷、无控制位），用于测试被动建连半连接重传与恢复。
    DropHandshakeAck {
        /// 匹配的发送端源端口（主机字节序）。
        source_port: u16,
        /// 匹配的接收端目的端口（主机字节序）。
        destination_port: u16,
        /// 标记首个匹配握手 `ACK` 是否已被丢弃（后续 `ACK` 正常放行）。
        dropped: bool,
    },
}

/// 内核网络协议栈根服务实例。
///
/// 统一管理端口与连接索引表、回环与以太网接口、出站报文队列、专用后台工作队列以及网卡设备生命周期。
pub struct NetStack {
    /// 传输层端口租约与活跃 TCP/UDP/UNIX 端点查找表。
    tables: NetTables,
    /// 本地回环接口（`lo`）的协议栈与帧缓冲状态（停机销毁时置为 `None`）。
    loopback: SleepMutex<Option<InterfaceState>>,
    /// 本地回环接口（`lo`）的收发包数、字节数与丢包统计（自旋锁保护）。
    loopback_stats: SpinLock<InterfaceStats>,
    /// 控制面互斥锁：串行化接口配置修改、新端点创建、输出暂停重查以及 `stop_sync()` 停机流程。
    admission: SleepMutex<()>,
    /// 全局软件出站报文队列与容量流控等待链表（自旋锁保护）。
    egress: SpinLock<Egress>,
    /// 已从 `tables` 摘除、等待后台 `reaper` 任务同步回收定时器与 Work 的退休端点队列（自旋锁保护）。
    retired: SpinLock<RetiredEndpoints>,
    /// 网络子系统专用的多核工作队列（承载根 `ingress`、`reaper` 任务及所有连接的协议推进 `Work`）。
    queue: Once<WorkQueue>,
    /// 推进回环帧交换、以太网帧发送、IP 分片及接口维护的根后台任务。
    ingress: Once<Pin<Arc<Work>>>,
    /// 异步执行退休端点 `stop_sync()` 清理的根后台回收任务。
    reaper: Once<Pin<Arc<Work>>>,
    /// 可选绑定的底层物理或 VirtIO 网卡驱动实例（纯回环模式下未初始化）。
    nic: Once<NetDevice>,
    /// 可选的以太网接口（`eth0`）协议栈与双帧槽状态（仅在绑定了 `nic` 时初始化，停机销毁时置为 `None`）。
    ethernet: Once<SleepMutex<Option<InterfaceState>>>,
    /// 驱动 IP 分片重组超时与 ARP/NDP 邻居重传定时的接口级高精度定时器。
    interface_timer: Once<Pin<Arc<Timer>>>,
    /// 串行化 `interface_timer` 重装填（`arm`）与停机取消（`close_sync`）的自旋锁，防止停机后定时器被重新挂起。
    timer_control: SpinLock<()>,
    /// 标记网络栈是否已进入停机流程（一旦置为 `true`，所有新连接、发送与定时器重排均立即中止）。
    stopping: AtomicBool,
    /// 标记网络栈是否已完成全部后台清理并处于安全可销毁状态（供 `Drop` 断言检查）。
    stopped: AtomicBool,
    /// 当前网络栈内尚未彻底析构的 [`EndpointSlot`] 总数（覆盖构造中、活跃、退休及外部引用阶段）。
    endpoints: AtomicUsize,
    /// 单调递增不回绕的端点唯一身份编号分配器（从 1 开始）。
    next_connection: AtomicU64,
    /// 为新建连接轮询分配默认协议处理 CPU 核的计数器。
    next_protocol_cpu: AtomicUsize,
    /// 网络栈启动时的单调时钟纳秒种子，用于派生接口随机种子与各 TCP 连接的初始序列号（ISN）种子。
    seed: u64,
}

impl NetStack {
    /// 创建仅包含本地回环接口（`lo`）的独立网络协议栈服务，返回其唯一生命周期拥有者 [`NetStackOwner`]。
    pub fn try_new_lo() -> Result<NetStackOwner, FsError> {
        Self::try_new_base()
    }

    /// 创建包含本地回环接口（`lo`）并绑定平台网卡硬件（`eth0`）的完整网络协议栈服务。
    ///
    /// 在普通可睡眠任务上下文中依次初始化基础协议栈、准备网卡 DMA/队列资源、使用硬件 MAC 地址构造 `eth0` 接口并启动网卡收发；
    /// 若任一步骤失败，已创建的基础工作队列与资源会通过 `owner` 的析构安全停机回收。
    ///
    /// # Safety
    /// 1. 调用方必须保证对 `inventory` 中选定的网卡设备硬件寄存器、板级时钟/复位/PHY 配置及中断号拥有独占所有权；
    /// 2. 设备寄存器 MMIO 虚拟映射窗口必须在整个 `NetStackOwner` 存活期间（直至 `Drop` 中的 `stop_sync` 返回）保持有效映射；
    /// 3. `inventory` 描述的中断与 DMA 属性必须与当前运行硬件平台真实拓扑一致。
    pub unsafe fn try_new_nic(inventory: &DeviceInventory) -> Result<NetStackOwner, FsError> {
        let owner = Self::try_new_base()?;
        let net = owner.stack();
        // SAFETY:
        // 1. 调用方已按本函数 `# Safety` 契约保证 `inventory` 中网卡硬件资源独占、平台描述真实且 MMIO 映射持续有效；
        // 2. `net` 来自 `owner` 持有的 `NetStackRef`，若后续启动失败，`owner` 析构会先调用 `stop_sync()` 停止设备再释放 `NetStack`。
        let nic =
            unsafe { NetDevice::prepare(net, inventory) }.map_err(NetDeviceError::into_fs_error)?;
        net.nic.call_once(|| nic);
        let ethernet = InterfaceState::try_new(
            Medium::Ethernet,
            HardwareAddress::Ethernet(EthernetAddress(net.nic.get().unwrap().mac_address())),
            net.seed,
        )?;
        net.ethernet.call_once(|| SleepMutex::new(Some(ethernet)));
        net.nic
            .get()
            .unwrap()
            .start()
            .map_err(NetDeviceError::into_fs_error)?;
        Ok(owner)
    }

    /// 分配并初始化网络栈公共基础设施（端口表、`lo` 接口、出站队列、工作队列及根后台任务/定时器）。
    fn try_new_base() -> Result<NetStackOwner, FsError> {
        let mut packets = VecDeque::new();
        packets
            .try_reserve_exact(LOOPBACK_CAPACITY)
            .map_err(|_| FsError::NoMemory)?;
        let net = Arc::try_new(Self {
            tables: NetTables::try_new()?,
            loopback: SleepMutex::new(Some(InterfaceState::try_new(
                Medium::Ip,
                HardwareAddress::Ip,
                time::now().as_nanos(),
            )?)),
            loopback_stats: SpinLock::new(InterfaceStats::default()),
            admission: SleepMutex::new(()),
            egress: SpinLock::new(Egress {
                open: true,
                packets,
                waiting: EgressWaiting::new(),
                #[cfg(feature = "ktest")]
                packet_control: None,
            }),
            retired: SpinLock::new(RetiredEndpoints::new()),
            queue: Once::new(),
            ingress: Once::new(),
            reaper: Once::new(),
            nic: Once::new(),
            ethernet: Once::new(),
            interface_timer: Once::new(),
            timer_control: SpinLock::new(()),
            stopping: AtomicBool::new(false),
            stopped: AtomicBool::new(true),
            endpoints: AtomicUsize::new(0),
            next_connection: AtomicU64::new(1),
            next_protocol_cpu: AtomicUsize::new(0),
            seed: time::now().as_nanos(),
        })
        .map_err(|_| FsError::NoMemory)?;

        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: cpu::count() + ROOT_WORK_ITEMS,
            ordered: false,
            reclaim: false,
        })
        .map_err(|_| FsError::NoMemory)?;
        net.queue.call_once(|| queue);
        if let Err(error) = net.prepare_workers() {
            net.work_queue().close_sync();
            return Err(error);
        }

        net.stopped.store(false, Ordering::Release);
        Ok(NetStackOwner {
            net: ManuallyDrop::new(net),
        })
    }

    /// 创建绑定到根 `NetStack` 裸指针上下文的 `ingress` 任务、`reaper` 回收任务与接口高精度定时器。
    fn prepare_workers(self: &NetStackRef) -> Result<(), FsError> {
        let context = Arc::as_ptr(self) as usize;
        let ingress = Work::try_new(self.work_queue(), 0, run_ingress, context)
            .map_err(|_| FsError::NoMemory)?;
        let reaper = Work::try_new(self.work_queue(), 0, run_retired, context)
            .map_err(|_| FsError::NoMemory)?;
        let timer = Timer::try_new(0, TimerKind::Precise, interface_expired, context)
            .map_err(|_| FsError::NoMemory)?;
        self.ingress.call_once(|| ingress);
        self.reaper.call_once(|| reaper);
        self.interface_timer.call_once(|| timer);
        Ok(())
    }

    /// 获取控制面串行准入互斥锁，并检查网络栈是否正在停机；仅用于配置修改与对象创建，不在数据包收发路径上持有。
    pub(crate) fn admission(&self) -> Result<SleepMutexGuard<'_, ()>, FsError> {
        let guard = self.admission.lock();
        if self.is_stopping() {
            return Err(FsError::NotConnected);
        }
        Ok(guard)
    }

    /// 返回网络栈内部的端口绑定与活跃端点索引表引用。
    pub(crate) fn tables(&self) -> &NetTables {
        &self.tables
    }

    /// 判断给定 IP 地址是否属于本机任一网络接口（`lo` 或 `eth0`）当前已配置的地址（不区分接口是否处于 `up` 状态）。
    pub fn is_local(&self, address: IpAddress) -> bool {
        if self
            .loopback
            .lock()
            .as_ref()
            .is_some_and(|state| state.iface.is_local(address))
        {
            return true;
        }
        self.ethernet.get().is_some_and(|ethernet| {
            ethernet
                .lock()
                .as_ref()
                .is_some_and(|state| state.iface.is_local(address))
        })
    }

    /// 判断给定本机目的 IP 地址所属的网络接口当前是否处于管理启用（`up`）状态，用于决定回环报文能否最终投递。
    fn local_destination_up(&self, address: IpAddress) -> bool {
        for id in [InterfaceId::Loopback, InterfaceId::Ethernet] {
            let Some(interface) = self.interface(id) else {
                continue;
            };
            let interface = interface.lock();
            if let Some(state) = interface.as_ref() {
                if state.iface.is_local(address) {
                    return state.up;
                }
            }
        }
        false
    }

    /// 为发往 `destination`（可选已绑定本地源地址 `bound_source`）的出站通信解析目标接口、源 IP 与路径 MTU。
    ///
    /// 路由选择规则：
    /// 1. 若显式绑定的 `bound_source` 非通配且不属于本机地址，直接返回 `AddressNotAvailable`；
    /// 2. 发往回环网段（如 `127.0.0.0/8` 或 `::1`）的流量固定选择 `lo` 接口；
    /// 3. 其余流量查询 `eth0` 接口路由表选定源地址；若 `destination` 恰好就是 `eth0` 自身的本机 IP，则保留 `eth0` 源地址但将交付接口重定向到 `lo`（本地自发自收）。
    pub fn resolve_route(
        &self,
        destination: IpAddress,
        bound_source: IpAddress,
    ) -> Result<Route, FsError> {
        // Step 1: 检查网络栈停机标志与绑定源地址的合法性，并将通配源地址对齐到目标地址所属的 IP 版本。
        if self.is_stopping() {
            return Err(FsError::NotConnected);
        }
        if !bound_source.is_unspecified() && !self.is_local(bound_source) {
            return Err(FsError::AddressNotAvailable);
        }

        let bound_source = if bound_source.is_unspecified() {
            match destination {
                IpAddress::Ipv4(_) => IpAddress::Ipv4(Ipv4Address::UNSPECIFIED),
                IpAddress::Ipv6(_) => IpAddress::Ipv6(Ipv6Address::UNSPECIFIED),
            }
        } else {
            bound_source
        };

        // Step 2: 优先匹配本地回环接口（lo）地址范围。
        let loopback = self.loopback.lock();
        let state = loopback.as_ref().ok_or(FsError::NotConnected)?;
        let loopback_up = state.up;
        if state.iface.is_local(destination) {
            if !loopback_up {
                return Err(FsError::NetworkUnreachable);
            }
            let (source, mtu) = state
                .iface
                .resolve_route(destination, bound_source)
                .map_err(dispatch_error)?;
            return Ok(Route {
                interface: InterfaceId::Loopback,
                source,
                mtu,
            });
        }
        drop(loopback);

        // Step 3: 释放 lo 锁后（遵守 lo -> eth0 锁顺序）查询以太网接口（eth0）地址与默认网关路由。
        let ethernet = self
            .ethernet
            .get()
            .ok_or(FsError::NetworkUnreachable)?
            .lock();
        let state = ethernet.as_ref().ok_or(FsError::NotConnected)?;
        if !state.up {
            return Err(FsError::NetworkUnreachable);
        }
        let local_destination = state.iface.is_local(destination);
        if !local_destination
            && !bound_source.is_unspecified()
            && !state.iface.is_local(bound_source)
        {
            return Err(FsError::NetworkUnreachable);
        }
        let (source, mtu) = state
            .iface
            .resolve_route(destination, bound_source)
            .map_err(dispatch_error)?;
        if local_destination {
            if !loopback_up {
                return Err(FsError::NetworkUnreachable);
            }
            Ok(Route {
                interface: InterfaceId::Loopback,
                source,
                mtu: LOOPBACK_MTU,
            })
        } else {
            Ok(Route {
                interface: InterfaceId::Ethernet,
                source,
                mtu,
            })
        }
    }

    /// 内核自测（`ktest`）辅助入口：若底层网卡为 VirtIO-Net 设备，返回其驱动实例引用以供测试检查队列状态。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_virtio_nic(&self) -> Option<&Arc<crate::drivers::virtio_net::VirtioNet>> {
        self.nic.get()?.ktest_virtio()
    }

    /// 内核自测（`ktest`）辅助入口：一键配置 `eth0` 的静态 IPv4 地址、前缀长度与可选默认网关，并将接口置为 `up` 状态。
    #[cfg(feature = "ktest")]
    pub fn configure_ethernet(&self, config: EthernetConfig) -> Result<(), FsError> {
        {
            let _admission = self.admission()?;
            config.validate()?;
            let mut ethernet = self.ethernet.get().ok_or(FsError::NoDevice)?.lock();
            let state = ethernet.as_mut().ok_or(FsError::NotConnected)?;
            state.iface.routes_mut().remove_default_ipv4_route();
            if let Some(gateway) = config.gateway {
                state
                    .iface
                    .routes_mut()
                    .add_default_ipv4_route(Ipv4Address::from(gateway))
                    .map_err(|_| FsError::NoMemory)?;
            }
            state.set_ipv4(config.address, config.prefix_len);
            state.up = true;
        }

        self.configuration_changed();
        Ok(())
    }

    /// 读取指定网络接口（`lo` 或 `eth0`）的当前 IPv4 地址、子网前缀与 `up` 状态快照；若未绑定物理网卡而查询 `eth0` 则返回 `NoDevice`。
    pub fn interface_snapshot(&self, id: InterfaceId) -> Result<InterfaceSnapshot, FsError> {
        let interface = self.interface(id).ok_or(FsError::NoDevice)?.lock();
        let state = interface.as_ref().ok_or(FsError::NotConnected)?;
        Ok(state.snapshot(id))
    }

    /// 依次采集 `[lo, eth0]` 两个接口的状态快照，供接口遍历与 `/proc` 文件格式化在锁外安全读取。
    pub fn interface_snapshots(&self) -> [Option<InterfaceSnapshot>; INTERFACE_COUNT] {
        [
            self.interface_snapshot(InterfaceId::Loopback).ok(),
            self.interface_snapshot(InterfaceId::Ethernet).ok(),
        ]
    }

    /// 生成当前 IPv4 路由表只读快照数组（依次包含 `lo` 直连网段、`eth0` 直连网段及 `eth0` 默认网关路由，最多 3 项）。
    pub fn route_snapshots(&self) -> [Option<RouteSnapshot>; ROUTE_SNAPSHOT_CAPACITY] {
        let mut snapshots = [None; ROUTE_SNAPSHOT_CAPACITY];
        for (index, id) in [InterfaceId::Loopback, InterfaceId::Ethernet]
            .iter()
            .copied()
            .enumerate()
        {
            let Some(interface) = self.interface(id) else {
                continue;
            };
            let interface = interface.lock();
            let Some(state) = interface.as_ref() else {
                continue;
            };
            if let Some(cidr) = state.ipv4() {
                snapshots[index] = Some(RouteSnapshot {
                    interface: id,
                    destination: ipv4_network(cidr.address().octets(), cidr.prefix_len()),
                    prefix_len: cidr.prefix_len(),
                    gateway: None,
                    up: state.up,
                });
            }

            if id == InterfaceId::Ethernet {
                snapshots[INTERFACE_COUNT] = state.default_gateway().map(|gateway| RouteSnapshot {
                    interface: id,
                    destination: [0; 4],
                    prefix_len: 0,
                    gateway: Some(gateway),
                    up: state.up,
                });
            }
        }
        snapshots
    }

    /// 设置指定接口的 IPv4 单播地址（对应 `SIOCSIFADDR`）。
    ///
    /// `lo` 仅允许设置为 `127.0.0.1/8`；`eth0` 若此前尚未配置过 IPv4 地址则按 A/B/C 类地址自动推导初始子网前缀，否则保留已有前缀长度。
    pub fn set_interface_address(&self, id: InterfaceId, address: [u8; 4]) -> Result<(), FsError> {
        {
            let _admission = self.admission()?;
            let mut interface = self.interface(id).ok_or(FsError::NoDevice)?.lock();
            let state = interface.as_mut().ok_or(FsError::NotConnected)?;
            let prefix_len = match id {
                InterfaceId::Loopback => {
                    if address != Ipv4Address::LOCALHOST.octets() {
                        return Err(FsError::InvalidOperation);
                    }
                    LOOPBACK_IPV4_PREFIX_LEN
                }
                InterfaceId::Ethernet => {
                    if !unicast(address) {
                        return Err(FsError::InvalidOperation);
                    }
                    state
                        .ipv4()
                        .map_or_else(|| classful_prefix(address), |cidr| cidr.prefix_len())
                }
            };
            state.set_ipv4(address, prefix_len);
        }

        self.kick_egress();
        Ok(())
    }

    /// 修改指定接口已配置 IPv4 地址的子网掩码前缀长度（对应 `SIOCSIFNETMASK`）；接口尚未配置 IPv4 地址时返回 `AddressNotAvailable`。
    pub fn set_interface_netmask(&self, id: InterfaceId, prefix_len: u8) -> Result<(), FsError> {
        {
            let _admission = self.admission()?;
            let mut interface = self.interface(id).ok_or(FsError::NoDevice)?.lock();
            let state = interface.as_mut().ok_or(FsError::NotConnected)?;
            if IPV4_ADDRESS_BITS < prefix_len
                || (id == InterfaceId::Loopback && prefix_len != LOOPBACK_IPV4_PREFIX_LEN)
            {
                return Err(FsError::InvalidOperation);
            }
            let cidr = state.ipv4().ok_or(FsError::AddressNotAvailable)?;
            state.set_ipv4(cidr.address().octets(), prefix_len);
        }

        self.kick_egress();
        Ok(())
    }

    /// 切换指定接口的管理启用状态（对应 `SIOCSIFFLAGS` 的 `IFF_UP` 位）。
    ///
    /// 将接口置为 `down` 时不会重置网卡硬件或强制断开已有 TCP 连接（在途未确认数据将暂停等待恢复）；重新置为 `up` 时唤醒挂起的等待连接继续发送。
    pub fn set_interface_up(&self, id: InterfaceId, up: bool) -> Result<(), FsError> {
        {
            let _admission = self.admission()?;
            let mut interface = self.interface(id).ok_or(FsError::NoDevice)?.lock();
            let state = interface.as_mut().ok_or(FsError::NotConnected)?;
            state.up = up;
        }

        if up {
            self.configuration_changed();
        } else {
            self.kick_egress();
        }
        Ok(())
    }

    /// 在 `eth0` 接口上添加或删除 IPv4 默认网关路由（对应 `SIOCADDRT` / `SIOCDELRT`）。
    ///
    /// 添加时校验接口处于 `up` 状态、网关为与接口同子网且不等于本机/网络/广播地址的合法单播地址，且当前不存在冲突默认路由；
    /// 删除时校验传入网关与当前生效网关一致，否则返回 `NoSuchRoute`。
    pub fn change_default_route(
        &self,
        request: DefaultRouteRequest,
        add: bool,
    ) -> Result<(), FsError> {
        {
            let _admission = self.admission()?;
            let id = request.interface.unwrap_or(InterfaceId::Ethernet);
            let mut interface = self.interface(id).ok_or(FsError::NoDevice)?.lock();
            let state = interface.as_mut().ok_or(FsError::NotConnected)?;
            if id != InterfaceId::Ethernet {
                return Err(FsError::NetworkUnreachable);
            }
            let existing = state.default_gateway();
            if add {
                if !state.up {
                    return Err(FsError::NetworkUnreachable);
                }
                let cidr = state.ipv4().ok_or(FsError::NetworkUnreachable)?;
                let address = cidr.address().octets();
                let gateway = request.gateway;
                let network = ipv4_network(address, cidr.prefix_len());
                let mask = ipv4_mask(cidr.prefix_len());
                let host = u32::from_be_bytes(gateway) & !mask;
                if !unicast(gateway) || gateway == address {
                    return Err(FsError::InvalidOperation);
                }
                if !cidr.contains_addr(&Ipv4Address::from(gateway)) {
                    return Err(FsError::NetworkUnreachable);
                }
                if cidr.prefix_len() < IPV4_POINT_TO_POINT_PREFIX_LEN
                    && (gateway == network || host == !mask)
                {
                    return Err(FsError::InvalidOperation);
                }
                if existing.is_some() {
                    return Err(FsError::Exists);
                }

                state
                    .iface
                    .routes_mut()
                    .add_default_ipv4_route(Ipv4Address::from(gateway))
                    .map_err(|_| FsError::NoMemory)?;
            } else {
                if existing != Some(request.gateway) {
                    return Err(FsError::NoSuchRoute);
                }
                state.iface.routes_mut().remove_default_ipv4_route();
            }
        }

        self.kick_egress();
        Ok(())
    }

    /// 查询指定网络接口的收发流量与丢包统计快照（`lo` 读取软件回环统计，`eth0` 读取网卡驱动硬件/DMA 完成统计）。
    pub fn statistics(&self, id: InterfaceId) -> Result<InterfaceStats, FsError> {
        self.interface_snapshot(id)?;
        match id {
            InterfaceId::Loopback => Ok(*self.loopback_stats.lock()),
            InterfaceId::Ethernet => Ok(self.nic.get().ok_or(FsError::NoDevice)?.statistics()),
        }
    }

    /// 返回网络栈共享的多核后台工作队列引用。
    pub(crate) fn work_queue(&self) -> &WorkQueue {
        self.queue.get().expect("network queue is not prepared")
    }

    /// 分配全局唯一且永不复用的单调递增端点身份编号（耗尽 `u64` 空间时 panic）。
    pub(crate) fn allocate_identity(&self) -> u64 {
        self.next_connection
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .expect("network connection identity exhausted")
    }

    /// 按在线 CPU 核数轮询分配下一个 TCP 端点的默认协议处理与定时器归属 CPU 编号。
    pub(crate) fn allocate_protocol_cpu(&self) -> usize {
        self.next_protocol_cpu.fetch_add(1, Ordering::Relaxed) % cpu::count()
    }

    /// 检查网络栈是否已开始执行停机关闭流程。
    pub(crate) fn is_stopping(&self) -> bool {
        self.stopping.load(Ordering::Acquire)
    }

    /// 为即将创建的传输层端点申请一个存活计数槽位 [`EndpointSlot`]；停机期间拒绝申请并返回 `NotConnected`。
    pub fn claim_endpoint_slot(self: &NetStackRef) -> Result<EndpointSlot, FsError> {
        if self.is_stopping() {
            return Err(FsError::NotConnected);
        }
        self.endpoints
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .map_err(|_| FsError::NoMemory)?;
        Ok(EndpointSlot {
            net: Arc::downgrade(self),
        })
    }

    /// 创建新的 TCP 端点（含指定容量的冷态 `TcpCore`、连接定时器和专属 `Work`），注册到全局连接表后返回其共享引用。
    pub fn try_new_tcp(
        self: &NetStackRef,
        family: AddressFamily,
        rx_len: usize,
        tx_len: usize,
    ) -> Result<Arc<TcpEndpoint>, FsError> {
        let _admission = self.admission()?;
        let slot = self.claim_endpoint_slot()?;
        let id = self.allocate_identity();
        let endpoint = TcpEndpoint::try_new(self, id, slot, family, rx_len, tx_len)?;
        self.tables.insert_tcp(endpoint.clone())?;
        Ok(endpoint)
    }

    /// 使用黄金分割常数混合网络栈基础种子与连接唯一编号 `id`，派生连接专属随机种子并构造冷态 [`TcpCore`]。
    pub(crate) fn new_core(
        &self,
        id: u64,
        rx_len: usize,
        tx_len: usize,
    ) -> Result<TcpCore, FsError> {
        let seed = self.seed.wrapping_add(id.wrapping_mul(0x9e3779b97f4a7c15));
        TcpCore::try_new(rx_len, tx_len, LOOPBACK_MTU, seed)
    }

    /// 创建新的 UDP 端点并注册到全局 UDP 端点索引表。
    pub fn try_new_udp(
        self: &NetStackRef,
        family: AddressFamily,
    ) -> Result<Arc<UdpEndpoint>, FsError> {
        let _admission = self.admission()?;
        let slot = self.claim_endpoint_slot()?;
        let id = self.allocate_identity();
        let endpoint = UdpEndpoint::try_new(self, id, slot, family)?;
        self.tables.insert_udp(endpoint.clone())?;
        Ok(endpoint)
    }

    /// 创建一对相互连接的匿名 UNIX 域套接字端点（对应 `socketpair`），注册到全局表后返回两端引用。
    pub fn try_new_unix_pair(
        self: &NetStackRef,
        kind: UnixSocketType,
    ) -> Result<[Arc<UnixEndpoint>; 2], FsError> {
        let _admission = self.admission()?;
        let slots = [self.claim_endpoint_slot()?, self.claim_endpoint_slot()?];
        let ids = [self.allocate_identity(), self.allocate_identity()];
        let (pair, endpoints) = UnixPair::try_new(self, kind, ids, slots)?;
        self.tables.insert_unix(pair)?;
        Ok(endpoints)
    }

    /// 返回本地回环接口及软件 IP 报文的最大传输单元（`LOOPBACK_MTU`）。
    pub fn loopback_mtu(&self) -> usize {
        LOOPBACK_MTU
    }

    /// 校验出站路由并将 [`OwnedPacket`] 推入全局软件出站队列 `Egress`，随后异步触发 `ingress` 任务发送。
    ///
    /// 若因停机、报文超长、无路由或软件队列已满（`WouldBlock`）而未能入队，则在错误元组中将原 `packet` 完整归还给调用方以便保留重试。
    pub fn try_emit(&self, packet: OwnedPacket) -> Result<(), (FsError, OwnedPacket)> {
        if self.is_stopping() {
            return Err((FsError::NotConnected, packet));
        }
        if LOOPBACK_MTU < packet.len() {
            return Err((FsError::MessageTooLarge, packet));
        }
        let route = match self.resolve_output(&packet) {
            Ok(route) => route,
            Err(error) => {
                let error = if self.is_stopping() {
                    FsError::NotConnected
                } else {
                    error
                };
                return Err((error, packet));
            }
        };

        {
            let mut egress = self.egress.lock();
            if !egress.open {
                return Err((FsError::NotConnected, packet));
            }
            if egress.packets.len() == LOOPBACK_CAPACITY {
                return Err((FsError::WouldBlock, packet));
            }
            egress.packets.push_back(QueuedPacket {
                packet: Some(packet),
                route,
            });
        }

        self.kick_egress();
        Ok(())
    }

    /// 为待发送报文解析出站路由；若活跃 TCP 连接仅因出口接口被管理性置为 `down` 而暂时返回 `NetworkUnreachable`，
    /// 则将其登记到 `Egress::waiting` 等待队列并转为返回 `WouldBlock`，使连接在接口重新 `up` 前保持存而不发而非报错断开。
    fn resolve_output(&self, packet: &OwnedPacket) -> Result<Route, FsError> {
        let result = self.resolve_route(packet.destination(), packet.source());
        if !matches!(result, Err(FsError::NetworkUnreachable)) {
            return result;
        }
        let Ok((local, peer)) = packet.tcp_endpoints() else {
            return result;
        };
        let Some(endpoint) = self.tables.lookup_tcp(TcpTuple { local, peer }) else {
            return result;
        };

        // 持 admission 锁重新核对路由并登记等待节点，与 set_interface_up(true) 严格串行，防止错过接口恢复唤醒。
        let _admission = self.admission()?;
        let result = self.resolve_route(packet.destination(), packet.source());
        if !matches!(result, Err(FsError::NetworkUnreachable))
            || !self.output_paused(packet.destination(), packet.source())
        {
            return result;
        }
        let mut egress = self.egress.lock();
        if !egress.open {
            return Err(FsError::NotConnected);
        }
        egress.waiting.push(endpoint.egress_wait());
        Err(FsError::WouldBlock)
    }

    /// 判断发往 `destination` 的路由不可达是否单纯由接口处于管理 `down` 状态引起（即底层地址与子网/网关路由配置本身仍然有效）。
    fn output_paused(&self, destination: IpAddress, source: IpAddress) -> bool {
        let loopback = self.loopback.lock();
        let Some(state) = loopback.as_ref() else {
            return false;
        };
        let loopback_up = state.up;
        if state.iface.is_local(destination) {
            return !loopback_up && state.iface.resolve_route(destination, source).is_ok();
        }
        drop(loopback);

        let Some(ethernet) = self.ethernet.get() else {
            return false;
        };
        let ethernet = ethernet.lock();
        ethernet.as_ref().is_some_and(|state| {
            let local = state.iface.is_local(destination);
            let valid_source = local || source.is_unspecified() || state.iface.is_local(source);
            valid_source
                && state.iface.resolve_route(destination, source).is_ok()
                && (!state.up || (local && !loopback_up))
        })
    }

    /// 接口配置或 `up` 状态变更后的统一唤醒钩子：唤醒当前已挂起等待出站容量的全部端点，并触发根 `ingress` 任务推进发送。
    fn configuration_changed(&self) {
        let count = self.egress.lock().waiting.len();
        self.wake_egress_waiters(count);
        self.kick_egress();
    }

    /// 从 `Egress::waiting` 队列头部依次弹出至多 `count` 个等待节点，并在释放 `egress` 自旋锁后逐个通知对应端点恢复发送。
    fn wake_egress_waiters(&self, count: usize) {
        for _ in 0..count {
            let node = self.egress.lock().waiting.pop();
            let Some(node) = node else {
                break;
            };
            self.notify_egress(node.id());
        }
    }

    /// 若网络栈未处于停机状态，向后台工作队列异步提交根 `ingress` 任务以推进回环收发、网卡发送与分片处理。
    pub(crate) fn kick_egress(&self) {
        if !self.is_stopping() {
            let _ = self
                .ingress
                .get()
                .expect("network ingress is not prepared")
                .as_ref()
                .queue();
        }
    }

    /// 发送一份软件 IP 报文（如无连接控制包或单次 UDP 报文）；发送失败时直接丢弃报文并返回对应的 [`FsError`]。
    pub fn send_ip(&self, packet: OwnedPacket) -> Result<(), FsError> {
        self.try_emit(packet).map_err(|(error, _packet)| error)
    }

    /// 检查全局软件出站队列当前是否有空闲槽位；若队列已满且传入的 `node` 尚未入链，则原子地将其挂入 `Egress::waiting` 等待唤醒。
    pub(crate) fn egress_ready(&self, node: &Arc<EgressWait>) -> Result<bool, FsError> {
        let mut egress = self.egress.lock();
        if !egress.open {
            return Err(FsError::NotConnected);
        }
        if node.linked() {
            return Ok(false);
        }
        if egress.packets.len() < LOOPBACK_CAPACITY {
            return Ok(true);
        }
        egress.waiting.push(node);
        Ok(false)
    }

    /// 当端点关闭或进入退休流程时，按身份编号 `id` 从 `Egress::waiting` 等待链表中移除其等待节点（释放自旋锁后再析构节点）。
    pub(crate) fn cancel_egress_wait(&self, id: u64) {
        let node = self.egress.lock().waiting.remove(id);
        drop(node);
    }

    /// 按端点身份编号 `id` 查找活跃的 TCP 或 UDP 端点，并调度其协议任务或唤醒阻塞在发送队列上的任务。
    pub(crate) fn notify_egress(&self, id: u64) {
        if self.is_stopping() {
            return;
        }
        if let Some(endpoint) = self.tables.tcp_by_id(id) {
            endpoint.schedule();
        } else if let Some(endpoint) = self.tables.udp_by_id(id) {
            endpoint.notify_egress();
        }
    }

    /// 将已完成协议生命周期的 TCP 端点从全局活跃表中摘除并转入 `retired` 退休队列，交由后台 `reaper` 任务执行同步停止。
    ///
    /// 由于触发退休的代码通常正在该 TCP 端点自身的 `Work` 回调中运行，不能在自身回调内调用 `stop_sync()` 自等待，因此必须移交独立的 `reaper` 任务回收。
    pub(crate) fn retire_tcp(&self, id: u64) {
        let mut retired = self.retired.lock();
        if self.is_stopping() {
            return;
        }
        if let Some(endpoint) = self.tables.take_tcp(id) {
            retired.push(RetiredEndpoint::Tcp(endpoint));
        }
        drop(retired);

        self.schedule_reaper(id);
    }

    /// 将已关闭的 UDP 端点从全局活跃表中摘除并转入 `retired` 退休队列，交由后台 `reaper` 任务同步等待其回调退出并回收。
    pub(crate) fn retire_udp(&self, id: u64) {
        let mut retired = self.retired.lock();
        if self.is_stopping() {
            return;
        }
        if let Some(endpoint) = self.tables.take_udp(id) {
            retired.push(RetiredEndpoint::Udp(endpoint));
        }
        drop(retired);

        self.schedule_reaper(id);
    }

    /// 撤销退休端点的出站容量等待登记，并异步调度根 `reaper` 回收任务。
    fn schedule_reaper(&self, id: u64) {
        self.cancel_egress_wait(id);
        let _ = self
            .reaper
            .get()
            .expect("network reaper is not prepared")
            .as_ref()
            .queue();
    }

    /// 内核自测（`ktest`）辅助查询：检查身份编号为 `id` 的端点当前是否挂在出站容量等待队列中。
    #[cfg(feature = "ktest")]
    pub(crate) fn egress_waiting(&self, id: u64) -> bool {
        self.egress.lock().waiting.contains(id)
    }

    /// 内核自测（`ktest`）辅助查询：返回当前网络栈中尚未析构的 [`EndpointSlot`] 总数，用于校验连接关闭后无资源泄漏。
    #[cfg(feature = "ktest")]
    pub fn ktest_live_endpoints(&self) -> usize {
        self.endpoints.load(Ordering::Acquire)
    }

    /// 内核自测（`ktest`）故障注入：开启对本地回环目的端口 `destination_port` 首个 `SYN` 握手段的拦截暂存（同一时刻仅允许挂载一种故障规则）。
    #[cfg(feature = "ktest")]
    pub fn ktest_hold_syn(&self, destination_port: u16) -> Result<(), FsError> {
        let mut egress = self.egress.lock();
        if !egress.open {
            return Err(FsError::NotConnected);
        }
        if egress.packet_control.is_some() {
            return Err(FsError::Busy);
        }
        egress.packet_control = Some(TestPacketControl::HoldSyn {
            destination_port,
            held: None,
            released: false,
        });
        Ok(())
    }

    /// 内核自测（`ktest`）辅助查询：判断目标端口的首个回环 `SYN` 报文是否已被成功拦截并暂存在控制器中。
    #[cfg(feature = "ktest")]
    pub fn ktest_has_held_syn(&self) -> bool {
        matches!(
            self.egress.lock().packet_control.as_ref(),
            Some(TestPacketControl::HoldSyn { held: Some(_), .. })
        )
    }

    /// 内核自测（`ktest`）故障恢复：将被拦截暂存的 `SYN` 报文重新放回回环出站队列并触发 `ingress` 任务投递。
    #[cfg(feature = "ktest")]
    pub fn ktest_release_syn(&self) -> Result<(), FsError> {
        {
            let mut egress = self.egress.lock();
            if !egress.open {
                return Err(FsError::NotConnected);
            }
            if egress.packets.len() == LOOPBACK_CAPACITY {
                return Err(FsError::WouldBlock);
            }
            let packet = match egress.packet_control.as_mut() {
                Some(TestPacketControl::HoldSyn { held, released, .. }) => {
                    let packet = held.take().ok_or(FsError::InvalidOperation)?;
                    *released = true;
                    packet
                }
                _ => return Err(FsError::InvalidOperation),
            };
            let route = Route {
                interface: InterfaceId::Loopback,
                source: packet.source(),
                mtu: LOOPBACK_MTU,
            };
            egress.packets.push_back(QueuedPacket {
                packet: Some(packet),
                route,
            });
        }

        let _ = self.ingress.get().unwrap().as_ref().queue();
        Ok(())
    }

    /// 内核自测（`ktest`）故障注入：配置丢弃回环接口上从 `source_port` 发往 `destination_port` 的首个非空 TCP 数据段，并跟踪后续 RTO 重传。
    #[cfg(feature = "ktest")]
    pub fn ktest_drop_next_data(
        &self,
        source_port: u16,
        destination_port: u16,
    ) -> Result<(), FsError> {
        let mut egress = self.egress.lock();
        if !egress.open {
            return Err(FsError::NotConnected);
        }
        if egress.packet_control.is_some() {
            return Err(FsError::Busy);
        }
        egress.packet_control = Some(TestPacketControl::DropData {
            source_port,
            destination_port,
            observation: TestLossObservation::default(),
        });
        Ok(())
    }

    /// 内核自测（`ktest`）故障注入：配置丢弃回环接口上从 `source_port` 发往 `destination_port` 的首个三次握手纯 `ACK` 段。
    #[cfg(feature = "ktest")]
    pub fn ktest_drop_handshake_ack(
        &self,
        source_port: u16,
        destination_port: u16,
    ) -> Result<(), FsError> {
        let mut egress = self.egress.lock();
        if !egress.open {
            return Err(FsError::NotConnected);
        }
        if egress.packet_control.is_some() {
            return Err(FsError::Busy);
        }
        egress.packet_control = Some(TestPacketControl::DropHandshakeAck {
            source_port,
            destination_port,
            dropped: false,
        });
        Ok(())
    }

    /// 内核自测（`ktest`）辅助查询：判断配置的目标握手 `ACK` 段是否已在回环交付阶段被实际拦截丢弃。
    #[cfg(feature = "ktest")]
    pub fn ktest_handshake_ack_dropped(&self) -> bool {
        matches!(
            self.egress.lock().packet_control.as_ref(),
            Some(TestPacketControl::DropHandshakeAck { dropped: true, .. })
        )
    }

    /// 内核自测（`ktest`）辅助查询：读取 `DropData` 规则记录的丢包序列号、重传次数及首包/重传单调时钟纳秒时间戳。
    #[cfg(feature = "ktest")]
    pub fn ktest_loss_observation(&self) -> TestLossObservation {
        match self.egress.lock().packet_control.as_ref() {
            Some(TestPacketControl::DropData { observation, .. }) => *observation,
            _ => TestLossObservation::default(),
        }
    }

    /// 内核自测（`ktest`）清理入口：移除当前挂载的回环报文故障注入规则（若仍有暂存未释放的 `SYN` 包则返回 `Busy`）。
    #[cfg(feature = "ktest")]
    pub fn ktest_clear_packet_control(&self) -> Result<(), FsError> {
        let control = {
            let mut egress = self.egress.lock();
            if matches!(
                egress.packet_control.as_ref(),
                Some(TestPacketControl::HoldSyn { held: Some(_), .. })
            ) {
                return Err(FsError::Busy);
            }
            egress.packet_control.take()
        };

        drop(control);
        Ok(())
    }

    /// 原子递增本地回环接口（`lo`）的发送丢包计数 `tx_dropped`。
    fn record_loopback_tx_drop(&self) {
        let mut stats = self.loopback_stats.lock();
        stats.tx_dropped = stats.tx_dropped.wrapping_add(1);
    }

    /// 内核自测（`ktest`）回环出站过滤钩子：按当前激活的 [`TestPacketControl`] 规则拦截、丢弃或统计匹配的 TCP 报文；返回 `None` 表示报文已被暂存或丢弃。
    #[cfg(feature = "ktest")]
    fn controlled_loopback_packet(&self, packet: OwnedPacket) -> Option<OwnedPacket> {
        if packet.protocol() != IpProtocol::Tcp {
            return Some(packet);
        }
        let (_, repr) = packet.tcp().expect("validated loopback TCP packet");
        let (source, destination, control, payload, sequence, acknowledged) = (
            repr.src_port,
            repr.dst_port,
            repr.control,
            repr.payload.len(),
            repr.seq_number.0,
            repr.ack_number.is_some(),
        );
        let now = time::now().as_nanos();
        let mut egress = self.egress.lock();
        match egress.packet_control.as_mut() {
            Some(TestPacketControl::HoldSyn {
                destination_port,
                held,
                released,
            }) if destination == *destination_port && control == TcpControl::Syn && !*released => {
                if held.is_none() {
                    *held = Some(packet);
                    return None;
                }
                drop(egress);
                drop(packet);
                self.record_loopback_tx_drop();
                return None;
            }
            Some(TestPacketControl::DropHandshakeAck {
                source_port,
                destination_port,
                dropped,
            }) if source == *source_port
                && destination == *destination_port
                && control == TcpControl::None
                && payload == 0
                && acknowledged
                && !*dropped =>
            {
                *dropped = true;
                drop(egress);
                drop(packet);
                self.record_loopback_tx_drop();
                return None;
            }
            Some(TestPacketControl::DropData {
                source_port,
                destination_port,
                observation,
            }) if source == *source_port
                && destination == *destination_port
                && 0 < payload
                && matches!(control, TcpControl::None | TcpControl::Psh) =>
            {
                if observation.dropped == 0 {
                    observation.dropped = 1;
                    observation.sequence = Some(sequence);
                    observation.first_ns = now;
                    drop(egress);
                    drop(packet);
                    self.record_loopback_tx_drop();
                    return None;
                }
                if observation.sequence == Some(sequence) {
                    observation.retransmissions += 1;
                    if observation.retransmissions == 1 {
                        observation.repeated_ns = now;
                    }
                }
            }
            _ => {}
        }
        drop(egress);
        Some(packet)
    }

    /// 从全局软件出站队列中移除已完成发送（或已丢弃）的第 `index` 个槽位，并在锁外析构报文及唤醒等待出站容量的端点。
    fn finish_egress(&self, index: usize) {
        let (entry, count) = {
            let mut egress = self.egress.lock();
            let entry = egress
                .packets
                .remove(index)
                .expect("network egress slot missing");
            let count = egress.waiting.len();
            (entry, count)
        };

        drop(entry);
        self.wake_egress_waiters(count);
    }

    /// 查找并移除指定接口当前正在处理的出站占位槽（即 `packet` 为 `None` 的条目），归还软件出站队列额度并唤醒等待者。
    fn finish_interface_egress(&self, id: InterfaceId) {
        let index = self
            .egress
            .lock()
            .packets
            .iter()
            .position(|entry| entry.route.interface == id && entry.packet.is_none());
        if let Some(index) = index {
            self.finish_egress(index);
        }
    }

    /// 根据接口标识 `id` 返回对应的接口状态睡眠互斥锁引用（纯回环模式下查询 `Ethernet` 返回 `None`）。
    fn interface(&self, id: InterfaceId) -> Option<&SleepMutex<Option<InterfaceState>>> {
        match id {
            InterfaceId::Loopback => Some(&self.loopback),
            InterfaceId::Ethernet => self.ethernet.get(),
        }
    }

    /// 在进入接口解析前校验接口状态并获取一份预分配的入站有效载荷捕获缓冲区（若复用池为空则在接口锁外按 `LOOPBACK_MTU` 容量新分配）。
    fn take_ingress_storage(&self, id: InterfaceId) -> Result<(CaptureStorage, Vec<u8>), FsError> {
        let capture = {
            let interface = self.interface(id).ok_or(FsError::NotConnected)?.lock();
            let state = interface.as_ref().ok_or(FsError::NotConnected)?;
            if self.is_stopping() {
                return Err(FsError::NotConnected);
            }
            if !state.up {
                return Err(FsError::NetworkUnreachable);
            }
            if id == InterfaceId::Ethernet && !state.ethernet_ingress_ready() {
                return Err(FsError::WouldBlock);
            }
            state.capture.clone()
        };

        let storage = ::core::mem::take(&mut *capture.lock());
        let storage = if storage.capacity() == 0 {
            prepare_ingress_storage()?
        } else {
            assert!(
                LOOPBACK_MTU <= storage.capacity(),
                "ingress capture capacity lost"
            );
            storage
        };
        Ok((capture, storage))
    }

    /// 从全局出站队列中挑选第一条所属接口空闲就绪、且目的 IP 未处于 ARP/NDP 邻居解析阻塞（`stalled`）状态的待发报文。
    fn select_egress(
        &self,
        blocked: [bool; INTERFACE_COUNT],
        stalled: &[Option<IpAddress>],
    ) -> Option<(usize, InterfaceId)> {
        let ready = [
            !blocked[0]
                && self
                    .loopback
                    .lock()
                    .as_ref()
                    .is_some_and(InterfaceState::dispatch_ready),
            !blocked[1]
                && self.ethernet.get().is_some_and(|ethernet| {
                    ethernet
                        .lock()
                        .as_ref()
                        .is_some_and(InterfaceState::dispatch_ready)
                }),
        ];

        let egress = self.egress.lock();
        if !egress.open {
            return None;
        }
        egress
            .packets
            .iter()
            .enumerate()
            .find_map(|(index, entry)| {
                let available = match entry.route.interface {
                    InterfaceId::Loopback => ready[0],
                    InterfaceId::Ethernet => ready[1],
                };
                let destination = Some(entry.packet.as_ref()?.destination());
                (available && !stalled.contains(&destination))
                    .then_some((index, entry.route.interface))
            })
    }

    /// 处理本地回环接口（`lo`）上已编码就绪的一帧三层 IP 报文：将输出帧缓冲与 `loopback_rx` 指针零拷贝交换后送入解析链，并投递给目标端点。
    fn receive_loopback_frame(&self) -> Result<bool, FsError> {
        // Step 1: 读取当前待回环帧的目的 IP 地址；若无待发帧或回环接口未启用则直接返回 Ok(false)。
        let destination = {
            let loopback = self.loopback.lock();
            let Some(state) = loopback.as_ref() else {
                return Ok(false);
            };
            if !state.up || state.frame.len == 0 {
                return Ok(false);
            }
            state
                .frame
                .destination
                .expect("loopback output has no typed IP destination")
        };
        // Step 2: 若目的 IP 所属本机接口被管理性置为 down（例如 eth0 已关闭而回环访问 eth0 地址），直接丢弃该帧并计入 tx_dropped。
        if !self.local_destination_up(destination) {
            let binding = {
                let mut loopback = self.loopback.lock();
                let state = loopback.as_mut().ok_or(FsError::NotConnected)?;
                state.frame.len = 0;
                state.frame.destination = None;
                state.binding.take()
            };
            drop(binding);
            self.record_loopback_tx_drop();
            self.finish_interface_egress(InterfaceId::Loopback);
            return Ok(true);
        }

        // Step 3: 准备入站捕获缓冲，在 lo 接口锁内零拷贝交换 frame.bytes 与 loopback_rx 并执行单帧协议解析与流量统计。
        let (capture, storage) = self.take_ingress_storage(InterfaceId::Loopback)?;
        let mut collector = IngressCollector::new(&self.tables, capture, storage);
        collector.local_destination = self.is_local(destination).then_some(destination);
        let (received, binding) = {
            let mut loopback = self.loopback.lock();
            match loopback.as_mut() {
                Some(state) if state.up && !self.is_stopping() && 0 < state.frame.len => {
                    let len = state.frame.len;
                    ::core::mem::swap(&mut state.frame.bytes, &mut state.loopback_rx);
                    state.frame.len = 0;
                    state.frame.destination = None;
                    let mut device = FrameDevice {
                        frame: &mut state.frame,
                        input: Some(&state.loopback_rx[..len]),
                        medium: state.medium,
                    };
                    state
                        .iface
                        .poll_ingress_single(interface_now(), &mut device, &mut collector);
                    let mut stats = self.loopback_stats.lock();
                    stats.tx_bytes = stats.tx_bytes.wrapping_add(len as u64);
                    stats.tx_packets = stats.tx_packets.wrapping_add(1);
                    stats.rx_bytes = stats.rx_bytes.wrapping_add(len as u64);
                    stats.rx_packets = stats.rx_packets.wrapping_add(1);
                    if 0 < state.frame.len {
                        state.frame.destination = state.iface.last_transmit_destination();
                    }
                    (true, state.binding.take())
                }
                _ => (false, None),
            }
        };

        // Step 4: 释放接口锁后归还多余缓冲、释放源端口绑定租约与出站队列槽位，并将捕获到的报文投递给目标 TCP/UDP 端点。
        collector.return_storage();
        if !received {
            return Ok(false);
        }
        drop(binding);
        self.finish_interface_egress(InterfaceId::Loopback);

        let udp = matches!(collector.event.as_ref(), Some(IngressEvent::Udp(_, _)));
        if let Err(error) = collector.deliver() {
            let mut stats = self.loopback_stats.lock();
            stats.rx_dropped = stats.rx_dropped.wrapping_add(1);
            if !udp || error != FsError::WouldBlock {
                return Err(error);
            }
        }
        Ok(true)
    }

    /// 尝试向底层物理/虚拟网卡驱动提交一帧已编码就绪的以太网帧（优先发送 `ingress_reply` 控制应答帧，其次发送 `frame` 常规出站帧）。
    ///
    /// 返回值含义：
    /// - `Ok(None)`：当前无待发以太网帧，或接口未启用/正在停机；
    /// - `Ok(Some(true))`：网卡发送队列成功接纳该帧（若常规出站报文且无后续 IP 分片，同步释放端口租约与软件出站队列槽位）；
    /// - `Ok(Some(false))`：网卡硬件发送队列暂时已满，待发帧数据完整保留在对应槽位中等候网卡发送完成中断唤醒。
    fn transmit_ethernet_frame(&self) -> Result<Option<bool>, FsError> {
        let Some(ethernet) = self.ethernet.get() else {
            return Ok(None);
        };
        // Step 1: 在 eth0 接口锁内按优先级挑选待发帧槽，标记 in_flight 并临时取出底层字节数组以便在锁外拷贝给网卡驱动。
        let (bytes, len, source) = {
            let mut ethernet = ethernet.lock();
            let Some(state) = ethernet.as_mut() else {
                return Ok(None);
            };
            if !state.up || self.is_stopping() {
                return Ok(None);
            }
            let source = if state
                .ingress_reply
                .as_ref()
                .is_some_and(|reply| 0 < reply.len && !reply.in_flight)
            {
                EthernetFrameSource::IngressReply
            } else if 0 < state.frame.len && !state.frame.in_flight {
                EthernetFrameSource::Egress
            } else {
                return Ok(None);
            };
            let frame = match source {
                EthernetFrameSource::IngressReply => state.ingress_reply.as_mut().unwrap(),
                EthernetFrameSource::Egress => &mut state.frame,
            };
            frame.in_flight = true;
            (::core::mem::take(&mut frame.bytes), frame.len, source)
        };

        // Step 2: 在接口锁外调用网卡驱动发送接口，将帧字节写入硬件 TX DMA 描述符缓冲。
        let accepted = self
            .nic
            .get()
            .expect("Ethernet interface without NIC")
            .try_transmit(len, |frame| frame.copy_from_slice(&bytes[..len]));

        // Step 3: 重新获取 eth0 接口锁归还帧缓冲数组；若硬件已接纳则清空帧槽并在最后一个分片发出后取出绑定租约。
        let (binding, finished) = {
            let mut ethernet = ethernet.lock();
            let state = ethernet
                .as_mut()
                .expect("Ethernet workspace released before NIC tail");
            let frame = match source {
                EthernetFrameSource::IngressReply => state.ingress_reply.as_mut().unwrap(),
                EthernetFrameSource::Egress => &mut state.frame,
            };
            frame.bytes = bytes;
            frame.in_flight = false;
            if matches!(accepted, Ok(true)) {
                frame.len = 0;
                frame.destination = None;
                if source == EthernetFrameSource::Egress && !state.iface.fragmentation_pending() {
                    (state.binding.take(), true)
                } else {
                    (None, false)
                }
            } else {
                (None, false)
            }
        };

        drop(binding);
        if finished {
            self.finish_interface_egress(InterfaceId::Ethernet);
        }
        accepted.map(Some).map_err(NetDeviceError::into_fs_error)
    }

    /// 当指定接口存在尚未发完的 IP 分片且主帧槽已腾空时，驱动 `smoltcp` 分片器编码下一片以太网/IP 帧。
    fn advance_fragmentation(&self, id: InterfaceId) -> bool {
        let Some(interface) = self.interface(id) else {
            return false;
        };
        let mut interface = interface.lock();
        let Some(state) = interface.as_mut() else {
            return false;
        };
        if !state.up
            || self.is_stopping()
            || !state.frame.available()
            || !state.iface.fragmentation_pending()
        {
            return false;
        }

        let mut device = FrameDevice {
            frame: &mut state.frame,
            input: None,
            medium: state.medium,
        };
        state.iface.poll_fragmentation(interface_now(), &mut device);
        if 0 < state.frame.len {
            state.frame.destination = state.iface.last_transmit_destination();
        }
        0 < state.frame.len
    }

    /// 根 `ingress` 后台任务主循环：在单次 `PROTOCOL_BUDGET`（16 步）配额内推进回环收发、以太网帧提交、IP 分片编码与新出站报文派发。
    ///
    /// 若遇到网卡硬件 TX 队列满或目标地址正在等待 ARP/NDP 邻居解析，本轮循环会记录阻塞状态并装填接口定时器，避免无意义的忙轮询重排。
    fn receive_egress(&self) {
        if self.is_stopping() {
            return;
        }
        // Step 1: 先驱动各接口的定期维护逻辑（清理超时未完成的 IP 分片重组缓存等）。
        self.maintain_interfaces();
        let mut blocked = [false; INTERFACE_COUNT];
        let mut stalled = [None; LOOPBACK_CAPACITY];
        let mut tx_full = false;
        let mut consumed = 0;
        // Step 2: 在预算配额内按“回环帧消费 -> 以太网帧提交 -> 以太网剩余分片推进 -> 新出站 IP 包编码”的优先级循环推进。
        while consumed < PROTOCOL_BUDGET && !self.is_stopping() {
            match self.receive_loopback_frame() {
                Ok(true) => {
                    consumed += 1;
                    continue;
                }
                Ok(false) => {}
                Err(error) => {
                    crate::klog!(
                        crate::klog::Level::Debug,
                        "Loopback input failed: {:?}",
                        error
                    );
                    blocked[0] = true;
                }
            }

            if !tx_full {
                match self
                    .transmit_ethernet_frame()
                    .expect("Ethernet transmission failed")
                {
                    Some(true) => {
                        consumed += 1;
                        continue;
                    }
                    Some(false) => {
                        tx_full = true;
                        blocked[1] = true;
                    }
                    None => {}
                }
            }
            if !tx_full && self.advance_fragmentation(InterfaceId::Ethernet) {
                consumed += 1;
                continue;
            }

            // Step 3: 从软件出站队列挑选一条可发送报文，在目标接口锁内调用 dispatch_ip 序列化为链路帧。
            let Some((index, id)) = self.select_egress(blocked, &stalled) else {
                break;
            };
            let packet = self.egress.lock().packets[index]
                .packet
                .take()
                .expect("network egress packet already claimed");
            #[cfg(feature = "ktest")]
            let packet = if id == InterfaceId::Loopback {
                match self.controlled_loopback_packet(packet) {
                    Some(packet) => packet,
                    None => {
                        self.finish_egress(index);
                        consumed += 1;
                        continue;
                    }
                }
            } else {
                packet
            };

            let result = {
                let mut interface = self
                    .interface(id)
                    .expect("queued interface disappeared")
                    .lock();
                match interface.as_mut() {
                    Some(state) if state.dispatch_ready() => {
                        let result = state.iface.dispatch_ip(
                            interface_now(),
                            FrameTx(&mut state.frame),
                            PacketMeta::default(),
                            packet.as_packet(),
                        );
                        if 0 < state.frame.len {
                            state.frame.destination = state.iface.last_transmit_destination();
                        }
                        if result.is_ok() {
                            assert!(
                                state.binding.is_none(),
                                "interface retained previous packet lease"
                            );
                            state.binding = packet.binding().cloned();
                        }
                        result
                    }
                    _ => Err(DispatchError::Exhausted),
                }
            };

            // Step 4: 根据接口派发结果处理报文归属：ARP 等待中记入 stalled 退回队列，槽满记入 blocked 退回队列，其余永久错误直接丢弃并归还槽位。
            match result {
                Ok(()) => {
                    drop(packet);
                }
                Err(DispatchError::NeighborPending) => {
                    *stalled
                        .iter_mut()
                        .find(|destination| destination.is_none())
                        .expect("network stalled destination capacity exhausted") =
                        Some(packet.destination());
                    self.egress.lock().packets[index].packet = Some(packet);
                }
                Err(DispatchError::Exhausted) => {
                    self.egress.lock().packets[index].packet = Some(packet);
                    blocked[if id == InterfaceId::Loopback { 0 } else { 1 }] = true;
                }
                Err(error) => {
                    self.finish_egress(index);
                    drop(packet);
                    if id == InterfaceId::Loopback {
                        self.record_loopback_tx_drop();
                    }
                    crate::klog!(crate::klog::Level::Debug, "IP dispatch failed: {:?}", error);
                }
            }
            consumed += 1;
        }

        // Step 5: 装填接口级超时定时器（ARP 重传 / 分片重组超时）；若因耗尽配额退出且仍有可立即推进的工作，则自我重排下一轮任务。
        self.arm_interface_timer();
        if consumed == PROTOCOL_BUDGET && self.actionable_egress(blocked, tx_full, &stalled) {
            self.kick_egress();
        }
    }

    /// 判断预算耗尽时是否仍存在无需等待外部事件即可立即推进的回环帧、以太网待发帧或软件队列报文。
    fn actionable_egress(
        &self,
        blocked: [bool; INTERFACE_COUNT],
        tx_full: bool,
        stalled: &[Option<IpAddress>],
    ) -> bool {
        if !blocked[0]
            && self
                .loopback
                .lock()
                .as_ref()
                .is_some_and(|state| state.up && 0 < state.frame.len)
        {
            return true;
        }
        if !tx_full
            && self.ethernet.get().is_some_and(|ethernet| {
                ethernet.lock().as_ref().is_some_and(|state| {
                    state.up
                        && (state
                            .ingress_reply
                            .as_ref()
                            .is_some_and(|reply| 0 < reply.len)
                            || 0 < state.frame.len
                            || state.iface.fragmentation_pending())
                })
            })
        {
            return true;
        }
        self.select_egress(blocked, stalled).is_some()
    }

    /// 推进 `lo` 与 `eth0` 接口的周期维护（回收过期的 IP 分片重组缓冲与失效邻居项）。
    fn maintain_interfaces(&self) {
        let now = interface_now();
        if let Some(state) = self.loopback.lock().as_mut() {
            state.iface.poll_maintenance(now);
        }
        if let Some(ethernet) = self.ethernet.get() {
            if let Some(state) = ethernet.lock().as_mut() {
                state.iface.poll_maintenance(now);
            }
        }
    }

    /// 汇总所有接口的维护超时与待发目的地址的 ARP/NDP 重传期限，取最早到期时刻装填接口级高精度定时器 `interface_timer`。
    fn arm_interface_timer(&self) {
        let mut destinations = [None; LOOPBACK_CAPACITY];
        {
            let egress = self.egress.lock();
            for (index, entry) in egress.packets.iter().enumerate() {
                destinations[index] = entry
                    .packet
                    .as_ref()
                    .map(|packet| (entry.route.interface, packet.destination()));
            }
        }

        let now = interface_now();
        let mut deadline = None;
        for id in [InterfaceId::Loopback, InterfaceId::Ethernet] {
            let Some(interface) = self.interface(id) else {
                continue;
            };
            let interface = interface.lock();
            let Some(state) = interface.as_ref() else {
                continue;
            };
            deadline = earlier_deadline(deadline, state.iface.maintenance_deadline());
            if !state.up {
                continue;
            }
            for (interface, destination) in destinations.iter().flatten() {
                if *interface == id {
                    deadline =
                        earlier_deadline(deadline, state.iface.egress_deadline(*destination, now));
                }
            }
        }

        let _control = self.timer_control.lock();
        if self.is_stopping() {
            return;
        }
        let timer = self
            .interface_timer
            .get()
            .expect("interface timer is not prepared")
            .as_ref();
        match deadline {
            Some(deadline) => {
                let micros =
                    u64::try_from(deadline.total_micros()).expect("negative interface deadline");
                let nanos = micros
                    .checked_mul(NSEC_PER_USEC)
                    .expect("interface deadline overflow");
                timer
                    .arm(time::MonotonicInstant::from_nanos(nanos))
                    .expect("interface timer arm failed");
            }
            None => {
                timer.cancel_pending();
            }
        }
    }

    /// 由底层网卡驱动接收路径调用：解析一帧刚从网卡 RX DMA 缓冲区收回 CPU 访问权的以太网数据帧，并将匹配到的传输层报文投递给目标端点。
    ///
    /// 解析过程中若产生 ARP Reply、ICMP Echo Reply 或 TCP RST，将直接编码进独立的 `ingress_reply` 帧槽并异步唤醒根 `ingress` 任务发出。
    pub(crate) fn receive_ethernet(&self, frame: &[u8]) -> Result<(), FsError> {
        if self.is_stopping() {
            return Err(FsError::NotConnected);
        }
        let (capture, storage) = self.take_ingress_storage(InterfaceId::Ethernet)?;
        let mut collector = IngressCollector::new(&self.tables, capture, storage);
        let result = {
            let mut ethernet = self
                .ethernet
                .get()
                .expect("Ethernet interface disappeared")
                .lock();
            match ethernet.as_mut() {
                Some(state) if !self.is_stopping() => {
                    if !state.up {
                        Err(FsError::NetworkUnreachable)
                    } else if state.ethernet_ingress_ready() {
                        let reply = state
                            .ingress_reply
                            .as_mut()
                            .expect("Ethernet ingress reply storage missing");
                        let mut device = FrameDevice {
                            frame: reply,
                            input: Some(frame),
                            medium: state.medium,
                        };
                        state.iface.poll_ingress_single(
                            interface_now(),
                            &mut device,
                            &mut collector,
                        );
                        if 0 < reply.len {
                            reply.destination = state.iface.last_transmit_destination();
                        }
                        Ok(())
                    } else {
                        Err(FsError::WouldBlock)
                    }
                }
                _ => Err(FsError::NotConnected),
            }
        };

        collector.return_storage();
        result?;
        self.kick_egress();
        collector.deliver()
    }

    /// 根 `reaper` 后台任务主体：在单次 `PROTOCOL_BUDGET` 配额内从 `retired` 队列弹出已退休端点，在自旋锁外调用 `stop_sync()` 同步停止其定时器与 Work。
    fn reap_retired(&self) {
        for _ in 0..PROTOCOL_BUDGET {
            if self.is_stopping() {
                return;
            }
            let endpoint = self.retired.lock().pop();
            let Some(endpoint) = endpoint else {
                return;
            };
            endpoint.stop_sync();
        }
        let pending = !self.retired.lock().is_empty();
        if pending && !self.is_stopping() {
            let _ = self.reaper.get().unwrap().as_ref().queue();
        }
    }

    /// 同步关闭整个网络协议栈并释放全部运行期资源（仅由 [`NetStackOwner::drop`] 在普通可睡眠任务上下文中调用一次）。
    fn stop_sync(&self) -> Result<(), FsError> {
        // Step 1: 持 admission 与 egress 锁原子设置 stopping 标志并关闭出站队列准入，阻止任何新连接创建或新报文入队。
        {
            let _admission = self.admission.lock();
            let mut egress = self.egress.lock();
            assert!(
                !self.stopping.swap(true, Ordering::AcqRel),
                "network domain stopped twice"
            );
            egress.open = false;
        }

        // Step 2: 与 timer_control 自旋锁同步栅栏后，依次同步取消并等待接口定时器、根 ingress 任务与根 reaper 任务彻底执行完毕退出。
        {
            let _control = self.timer_control.lock();
        }
        if let Some(timer) = self.interface_timer.get() {
            timer.as_ref().close_sync();
        }
        self.ingress.get().unwrap().as_ref().close_sync();
        self.reaper.get().unwrap().as_ref().close_sync();

        // Step 3: 摘取全局表中的全部活跃 TCP、UDP、UNIX 端点以及退休队列中的残留端点，逐一同步停止其专属定时器与 Work。
        let (tcp, udp, unix) = self.tables.take_all();
        for endpoint in tcp {
            endpoint.stop_sync();
        }
        for endpoint in udp {
            endpoint.stop_sync();
        }
        loop {
            let endpoint = self.retired.lock().pop();
            let Some(endpoint) = endpoint else {
                break;
            };
            endpoint.stop_sync();
        }
        for pair in unix {
            pair.stop_sync();
        }

        // Step 4: 同步停止底层物理/虚拟网卡硬件（关闭 DMA 收发引擎、屏蔽中断并等待在途驱动任务退出）。
        if let Some(nic) = self.nic.get() {
            nic.stop_sync().map_err(NetDeviceError::into_fs_error)?;
        }

        // Step 5: 释放各网络接口状态、排空出站等待链表与残留报文队列，最后关闭网络工作队列并标记 stopped = true。
        let loopback = self.loopback.lock().take();
        let ethernet = self
            .ethernet
            .get()
            .and_then(|ethernet| ethernet.lock().take());
        drop((loopback, ethernet));
        #[cfg(feature = "ktest")]
        {
            let control = self.egress.lock().packet_control.take();
            drop(control);
        }

        loop {
            let node = self.egress.lock().waiting.pop();
            let Some(node) = node else {
                break;
            };
            drop(node);
        }
        let packets = {
            let mut egress = self.egress.lock();
            ::core::mem::take(&mut egress.packets)
        };
        drop(packets);

        self.work_queue().close_sync();
        self.stopped.store(true, Ordering::Release);
        Ok(())
    }
}

impl Drop for NetStack {
    /// 校验网络栈在最后一个 `Arc` 强引用释放前已通过 [`NetStackOwner`] 完成了完整的同步停机流程。
    fn drop(&mut self) {
        assert!(
            self.stopped.load(Ordering::Acquire),
            "network domain dropped before stop"
        );
    }
}

/// 根 `ingress` 后台任务的回调函数入口：推进回环帧交换、以太网发送与软件出站队列派发。
fn run_ingress(work: Pin<&Work>) {
    // SAFETY:
    // 1. `work.context()` 在 `NetStack::prepare_workers` 中由 `Arc::as_ptr(self) as usize` 初始化，指向有效的 `NetStack` 实例且满足对齐；
    // 2. 唯一拥有者 `NetStackOwner::drop` 在释放 `NetStack` 强引用前，必先在 `stop_sync()` 中调用 `self.ingress.close_sync()`
    //    同步等待本回调彻底执行完毕并退出，因此回调执行期间 `NetStack` 内存始终存活且只创建共享引用 `&NetStack`。
    let net = unsafe { &*(work.context() as *const NetStack) };
    net.receive_egress();
}

/// 根 `reaper` 后台任务的回调函数入口：同步清理已从全局连接表移入退休队列的 TCP/UDP 端点。
fn run_retired(work: Pin<&Work>) {
    // SAFETY:
    // 1. `work.context()` 在 `NetStack::prepare_workers` 中由 `Arc::as_ptr(self) as usize` 初始化，指向有效的 `NetStack` 实例；
    // 2. `NetStackOwner::drop` 在释放 `NetStack` 前必先调用 `self.reaper.close_sync()` 等待本回调彻底执行完毕退出，保证引用生命周期有效。
    let net = unsafe { &*(work.context() as *const NetStack) };
    net.reap_retired();
}

/// 接口级高精度定时器的超时回调函数入口：当 IP 重组或 ARP/NDP 邻居重传到期时唤醒根 `ingress` 任务。
fn interface_expired(timer: Pin<&Timer>) {
    // SAFETY:
    // 1. `timer.context()` 在 `NetStack::prepare_workers` 中由 `Arc::as_ptr(self) as usize` 初始化，指向有效的 `NetStack` 实例；
    // 2. `NetStackOwner::drop` 在释放 `NetStack` 前必先在 `stop_sync()` 中调用 `interface_timer.close_sync()` 等待正在运行的定时器回调彻底退出。
    let net = unsafe { &*(timer.context() as *const NetStack) };
    net.kick_egress();
}

/// 在接口锁外预分配容量为 `LOOPBACK_MTU`（65535 字节）的空 `Vec<u8>`，用作入站传输层有效载荷的捕获缓冲区。
fn prepare_ingress_storage() -> Result<Vec<u8>, FsError> {
    let mut storage = Vec::new();
    storage
        .try_reserve_exact(LOOPBACK_MTU)
        .map_err(|_| FsError::NoMemory)?;
    Ok(storage)
}

/// 读取当前内核单调时钟并将其纳秒计数折算为 `smoltcp::Interface` 所需的微秒级 `Instant`。
fn interface_now() -> Instant {
    Instant::from_micros(
        i64::try_from(time::now().as_nanos() / NSEC_PER_USEC).expect("interface clock exhausted"),
    )
}

/// 从两个可选的 `Instant` 期限中选出更早到期的那一个。
fn earlier_deadline(left: Option<Instant>, right: Option<Instant>) -> Option<Instant> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left.min(right)),
        (left, right) => left.or(right),
    }
}

/// 根据传统 IPv4 分类编址规则（A 类 `/8`、B 类 `/16`、C 类 `/24`）为首次配置地址的接口推导默认子网前缀长度。
fn classful_prefix(address: [u8; 4]) -> u8 {
    const CLASS_B_FIRST_OCTET: u8 = 128;
    const CLASS_C_FIRST_OCTET: u8 = 192;
    const CLASS_A_PREFIX_LEN: u8 = 0xff00_0000u32.leading_ones() as u8;
    const CLASS_B_PREFIX_LEN: u8 = 0xffff_0000u32.leading_ones() as u8;
    const CLASS_C_PREFIX_LEN: u8 = 0xffff_ff00u32.leading_ones() as u8;

    if address[0] < CLASS_B_FIRST_OCTET {
        CLASS_A_PREFIX_LEN
    } else if address[0] < CLASS_C_FIRST_OCTET {
        CLASS_B_PREFIX_LEN
    } else {
        CLASS_C_PREFIX_LEN
    }
}

/// 将 `0..=32` 位的 IPv4 前缀长度转换为大端序 32 位子网掩码（单独处理 `prefix_len == 0` 以避免 32 位左移溢出）。
fn ipv4_mask(prefix_len: u8) -> u32 {
    if prefix_len == 0 {
        0
    } else {
        u32::MAX << (IPV4_ADDRESS_BITS - prefix_len)
    }
}

/// 将 IPv4 地址与指定前缀长度按位与，计算其所属的子网网络地址（用于生成 `/proc/net/route` 直连路由项）。
fn ipv4_network(address: [u8; 4], prefix_len: u8) -> [u8; 4] {
    (u32::from_be_bytes(address) & ipv4_mask(prefix_len)).to_be_bytes()
}

/// 将 `smoltcp` 接口派发错误映射为内核统一的 [`FsError`] 错误码。
fn dispatch_error(error: DispatchError) -> FsError {
    match error {
        DispatchError::NoRoute => FsError::NetworkUnreachable,
        DispatchError::NeighborPending | DispatchError::Exhausted => FsError::WouldBlock,
        DispatchError::PacketTooLarge => FsError::MessageTooLarge,
    }
}
