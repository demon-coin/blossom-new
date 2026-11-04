//! 网络域拥有本地报文、索引和后台执行；唯一服务拥有者承担同步停止。

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
#[cfg(feature = "ktest")]
use smoltcp::wire::TcpControl;
use smoltcp::wire::{
    EthernetAddress, HardwareAddress, IpAddress, IpCidr, IpProtocol, IpRepr, Ipv4Address, Ipv4Cidr,
    Ipv6Address, Ipv6Cidr, TcpRepr, UdpRepr, ETHERNET_HEADER_LEN,
};
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
use crate::net::route::{
    unicast, DefaultRouteRequest, EthernetConfig, InterfaceId, InterfaceSnapshot, Route,
    RouteSnapshot, IPV4_ADDRESS_BITS,
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

pub const LOOPBACK_CAPACITY: usize = 32;
pub const LOOPBACK_MTU: usize = MAX_IP_PACKET_LEN;
pub(crate) const SOMAXCONN: usize = 4096;
pub(crate) const PROTOCOL_BUDGET: usize = 16;
const ETHERNET_IP_MTU: usize = 1500;
pub(crate) const ETHERNET_FRAME_CAPACITY: usize = ETHERNET_IP_MTU + ETHERNET_HEADER_LEN;
const INTERFACE_WORKSPACE_CAPACITY: usize = 64 * 1024;
const LOOPBACK_IPV4_PREFIX_LEN: u8 = 8;
const IPV4_POINT_TO_POINT_PREFIX_LEN: u8 = IPV4_ADDRESS_BITS - 1;
const ROOT_WORK_ITEMS: usize = 2;
pub(crate) const INTERFACE_COUNT: usize = 2;
const ROUTE_SNAPSHOT_CAPACITY: usize = INTERFACE_COUNT + 1;

/// 每份预备帧独占自己的编码存储，领取期间不得再交出该槽的 TxToken。
struct FrameSlot {
    bytes: Vec<u8>,
    len: usize,
    in_flight: bool,
    destination: Option<IpAddress>,
}

impl FrameSlot {
    /// 在接口发表前准备完整帧存储，后续处理只写已经取得的容量。
    fn try_new(capacity: usize) -> Result<Self, FsError> {
        Ok(Self {
            bytes: zeroed_bytes(capacity)?,
            len: 0,
            in_flight: false,
            destination: None,
        })
    }

    /// 空槽才提供实际可消费的发送存储。
    fn available(&self) -> bool {
        self.len == 0 && !self.in_flight
    }
}

/// 锁外提交期间记录帧的原槽，输入应答不取得普通输出的 binding 或队列完成责任。
#[derive(Clone, Copy, Eq, PartialEq)]
enum EthernetFrameSource {
    IngressReply,
    Egress,
}

/// 软件介质只借当前 RX 前缀，配对 TX 的预备帧负责控制回应的拥有。
struct FrameDevice<'a> {
    frame: &'a mut FrameSlot,
    input: Option<&'a [u8]>,
    medium: Medium,
}

struct FrameRx<'a>(&'a [u8]);
struct FrameTx<'a>(&'a mut FrameSlot);

impl RxToken for FrameRx<'_> {
    /// 输入仅在同步库解析期间可借用，不能跨到端点 Work。
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(self.0)
    }
}

impl TxToken for FrameTx<'_> {
    /// TxToken 已独占空槽，编码完成即正式接纳软件帧，不依赖 NIC 后继容量。
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

    /// 只有配对控制输出可真正接纳时才允许库开始解析一帧。
    fn receive(&mut self, _: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if !self.frame.available() {
            return None;
        }
        Some((FrameRx(self.input.take()?), FrameTx(self.frame)))
    }

    /// 满槽不交 token，fragmenter 因而保留尚未发送的原尾部。
    fn transmit(&mut self, _: Instant) -> Option<Self::TxToken<'_>> {
        self.frame.available().then_some(FrameTx(self.frame))
    }

    /// 介质 MTU 含真实 Ethernet 头，VirtIO 头仍归原 driver。
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

/// 每接口持同一库状态；Ethernet 输入应答与普通输出各有一份预备帧。
struct InterfaceState {
    iface: Interface,
    frame: FrameSlot,
    ingress_reply: Option<FrameSlot>,
    loopback_rx: Vec<u8>,
    medium: Medium,
    up: bool,
    binding: Option<Arc<BindLease>>,
    capture: CaptureStorage,
}

impl InterfaceState {
    /// 发表前 fallible 准备全部存储；仅 Ethernet 另备一帧输入应答，lo 保持原有工作区。
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

    /// 只有管理状态为 up 且原帧和分片器空闲时接纳下一份输出。
    fn dispatch_ready(&self) -> bool {
        self.up && self.frame.available() && !self.iface.fragmentation_pending()
    }

    /// Ethernet 输入只等待自己的配对应答空槽，普通输出和未交完的尾片不阻止接收。
    fn ethernet_ingress_ready(&self) -> bool {
        self.up
            && self
                .ingress_reply
                .as_ref()
                .is_some_and(FrameSlot::available)
    }

    /// IPv4 地址始终从库 CIDR 读取，不在接口元数据保存副本。
    fn ipv4(&self) -> Option<Ipv4Cidr> {
        self.iface.ip_addrs().iter().find_map(|cidr| match cidr {
            IpCidr::Ipv4(cidr) => Some(*cidr),
            _ => None,
        })
    }

    /// 查询只复制 IPv4 和管理状态，不推进协议。
    fn snapshot(&self, id: InterfaceId) -> InterfaceSnapshot {
        let cidr = self.ipv4();
        InterfaceSnapshot {
            id,
            address: cidr.map(|cidr| cidr.address().octets()),
            prefix_len: cidr.map_or(0, |cidr| cidr.prefix_len()),
            up: self.up,
        }
    }

    /// 只替换 IPv4 项，原 IPv6 地址和库邻居刷新协议保持。
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

    /// 读取 Routes 中实际默认 IPv4 项，不保存额外默认配置。
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

/// 输入仅捕获一个端点与 owned repr，协议处理和通知由解接口锁后的调用者承担。
enum IngressEvent {
    Tcp(Arc<TcpEndpoint>, OwnedPacket),
    Udp(Arc<UdpEndpoint>, OwnedPacket),
}

struct IngressCollector<'a> {
    tables: &'a NetTables,
    storage: Option<Vec<u8>>,
    capture: CaptureStorage,
    event: Option<IngressEvent>,
    local_destination: Option<IpAddress>,
}

impl<'a> IngressCollector<'a> {
    /// 消费接口锁外已取得的后备，库回调只初始化实际 payload。
    fn new(tables: &'a NetTables, capture: CaptureStorage, storage: Vec<u8>) -> Self {
        Self {
            tables,
            storage: Some(storage),
            capture,
            event: None,
            local_destination: None,
        }
    }

    /// 未借给 owned 输入的后备归回同一小 holder，不访问 Root 或协议对象。
    fn return_storage(&mut self) {
        if let Some(storage) = self.storage.take() {
            return_capture_storage(&self.capture, storage);
        }
    }

    /// 入队失败保留真实丢弃错误，RX 调用者据实际输入拥有记录 drop。
    fn deliver(self) -> Result<(), FsError> {
        match self.event {
            Some(IngressEvent::Tcp(endpoint, packet)) => endpoint.enqueue(packet),
            Some(IngressEvent::Udp(endpoint, packet)) => endpoint.enqueue(packet),
            None => Ok(()),
        }
    }
}

impl<'s> TransportIngress<'s> for IngressCollector<'_> {
    /// 本次目的仅由 Root 在接口锁外核定，库接纳快照不改变地址配置。
    fn local_destination(&self) -> Option<IpAddress> {
        self.local_destination
    }

    /// 表锁只取得稳定 Arc；没有匹配仍由库生成成熟的正常拒绝回应。
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

    /// UDP 匹配事实不取 raw 锁；RX budget 满的后继丢弃由端点 Work 处理。
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

    /// 内核不拥有用户 SocketSet，普通 IP 控制处理仍由库自身执行。
    fn socket_set(&mut self) -> Option<&mut smoltcp::iface::SocketSet<'s>> {
        None
    }
}

/// Runtime、打开对象和系统拥有者持同一域，不按进程复制网络状态。
pub type NetStackRef = Arc<NetStack>;

/// 流进度、等待和 EOF 分开；Ready(0) 专用于真正的零长操作。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IoProgress {
    Ready(usize),
    Blocked,
    Eof,
}

/// 报文长度和 source 属于同一次接收，截断后的 copied 不冒充完整消息长度。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MessageResult {
    pub copied: usize,
    pub original_len: usize,
    pub source: Option<SocketAddress>,
    pub flags: u32,
}

/// 只能由可睡普通任务持有，最后一次析构等待本域全部回调尾部。
pub struct NetStackOwner {
    net: ManuallyDrop<NetStackRef>,
}

impl NetStackOwner {
    /// 借用服务的同一引用，Runtime 和文件可克隆此引用但不取得停止责任。
    pub fn stack(&self) -> &NetStackRef {
        &self.net
    }
}

impl Drop for NetStackOwner {
    /// 服务拥有者在普通任务作用域结束时关闭一次准入并等待全部后台尾部。
    fn drop(&mut self) {
        self.net.stop_sync().expect("network device stop failed");
        // SAFETY: Root 全部回调和 NIC DMA/IRQ 已真正停止，本 owner 唯一承担这份 Arc 的释放。
        unsafe { ManuallyDrop::drop(&mut self.net) };
    }
}

/// 计数覆盖准备中、表中、退休及外部引用；最后节点释放才归还。
pub struct EndpointSlot {
    net: Weak<NetStack>,
}

impl Drop for EndpointSlot {
    /// 弱域不形成拥有环；域仍存活时减少实际端点计数。
    fn drop(&mut self) {
        if let Some(net) = self.net.upgrade() {
            let previous = net.endpoints.fetch_sub(1, Ordering::AcqRel);
            assert!(0 < previous, "network endpoint slot underflow");
        }
    }
}

/// 队列容量和等待登记使用同一短锁，关闭也在这把锁内拒绝新的包。
struct Egress {
    open: bool,
    packets: VecDeque<QueuedPacket>,
    waiting: EgressWaiting,
    #[cfg(feature = "ktest")]
    packet_control: Option<TestPacketControl>,
}

/// 领取期间保留队列槽，NIC 拒绝容量时不会被并发生产者抢占软件额度。
struct QueuedPacket {
    packet: Option<OwnedPacket>,
    route: Route,
}

/// 只记录所选实际普通段的丢失及同 seq 再到达，时间来自生产 monotonic 域。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TestLossObservation {
    pub dropped: usize,
    pub sequence: Option<i32>,
    pub retransmissions: usize,
    pub first_ns: u64,
    pub repeated_ns: u64,
}

/// 控制只作用于本域 lo 交付窗口，held 保存原包而不合成握手应答。
#[cfg(feature = "ktest")]
enum TestPacketControl {
    HoldSyn {
        destination_port: u16,
        held: Option<OwnedPacket>,
        released: bool,
    },
    DropData {
        source_port: u16,
        destination_port: u16,
        observation: TestLossObservation,
    },
    DropHandshakeAck {
        source_port: u16,
        destination_port: u16,
        dropped: bool,
    },
}

/// 控制发表与停止串行，逐连接协议处理不持这把控制互斥。
pub struct NetStack {
    tables: NetTables,
    loopback: SleepMutex<Option<InterfaceState>>,
    loopback_stats: SpinLock<InterfaceStats>,
    admission: SleepMutex<()>,
    egress: SpinLock<Egress>,
    retired: SpinLock<RetiredEndpoints>,
    queue: Once<WorkQueue>,
    ingress: Once<Pin<Arc<Work>>>,
    reaper: Once<Pin<Arc<Work>>>,
    nic: Once<NetDevice>,
    ethernet: Once<SleepMutex<Option<InterfaceState>>>,
    interface_timer: Once<Pin<Arc<Timer>>>,
    timer_control: SpinLock<()>,
    stopping: AtomicBool,
    stopped: AtomicBool,
    endpoints: AtomicUsize,
    next_connection: AtomicU64,
    next_protocol_cpu: AtomicUsize,
    seed: u64,
}

impl NetStack {
    /// 明确构造只有 lo 的服务，仍使用完整共同协议、表与执行域。
    pub fn try_new_lo() -> Result<NetStackOwner, FsError> {
        Self::try_new_base()
    }

    /// 在普通任务完整准备根、库接口和平台指定的实际 NIC，start 成功才交付服务拥有者。
    ///
    /// # Safety
    /// 调用者须独占所选库存设备及板 MAC、PHY 和设置资源，并保证全部设备窗口永久有效。
    /// 库存须与本次启动资源同源，Root 在发表前保存设备，直到同步停止确认后才释放。
    pub unsafe fn try_new_nic(inventory: &DeviceInventory) -> Result<NetStackOwner, FsError> {
        let owner = Self::try_new_base()?;
        let net = owner.stack();
        // SAFETY: 调用者承担同源库存、独占硬件与永久映射义务，原 Root 从发表前保活设备。
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

    /// 两种介质只在共同协议、索引、队列和稳定 Work 全部准备成功后取得停止责任。
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

    /// 根拥有者覆盖两个 Work 和接口期限，全部准备后才允许外部安排。
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

    /// 控制操作先取得串行准入，再检查关门状态；不跨协议收发持有。
    pub(crate) fn admission(&self) -> Result<SleepMutexGuard<'_, ()>, FsError> {
        let guard = self.admission.lock();
        if self.is_stopping() {
            return Err(FsError::NotConnected);
        }
        Ok(guard)
    }

    /// 返回本域索引，表短锁仅发表或转移真正拥有。
    pub(crate) fn tables(&self) -> &NetTables {
        &self.tables
    }

    /// 本机地址只查询当前库接口，不持有第二份地址表。
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

    /// 交付另外核管理状态，本机地址查询本身仍如实保留 down 接口地址。
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

    /// 本域本机目的统一走 lo，其它目的由实际 eth 地址和 Routes 选源。
    pub fn resolve_route(
        &self,
        destination: IpAddress,
        bound_source: IpAddress,
    ) -> Result<Route, FsError> {
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

    /// 专用测试只借实际 VirtIO 变体的原 Arc，不把板 GMAC 当成 VirtIO 观察对象。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_virtio_nic(&self) -> Option<&Arc<crate::drivers::virtio_net::VirtioNet>> {
        self.nic.get()?.ktest_virtio()
    }

    /// 返回当前 eth0 完整配置副本，不主动推进设备或协议。
    pub fn ethernet_config(&self) -> Option<EthernetConfig> {
        let ethernet = self.ethernet.get()?.lock();
        let state = ethernet.as_ref()?;
        let cidr = state.ipv4()?;
        Some(EthernetConfig {
            address: cidr.address().octets(),
            prefix_len: cidr.prefix_len(),
            gateway: state.default_gateway(),
        })
    }

    /// fixture 提交库 IPv4、默认项和 up 状态，已有 IPv6 与原 TCP 未 ACK 数据保持。
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

    /// 接口查询只在库接口锁内复制小值，未存在的 eth0 返回真实设备错误。
    pub fn interface_snapshot(&self, id: InterfaceId) -> Result<InterfaceSnapshot, FsError> {
        let interface = self.interface(id).ok_or(FsError::NoDevice)?.lock();
        let state = interface.as_ref().ok_or(FsError::NotConnected)?;
        Ok(state.snapshot(id))
    }

    /// 最多两个真实接口的副本，不让格式化者持有库状态。
    pub fn interface_snapshots(&self) -> [Option<InterfaceSnapshot>; INTERFACE_COUNT] {
        [
            self.interface_snapshot(InterfaceId::Loopback).ok(),
            self.interface_snapshot(InterfaceId::Ethernet).ok(),
        ]
    }

    /// 直连从库 CIDR 生成，默认从同一 Routes 生成，最多三条 IPv4 路由。
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

    /// IPv4 重配保留原 IPv6；eth0 首次赋址才采用 classful 前缀。
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

    /// mask 仅改变已经配置的库 IPv4 CIDR，未赋址不能保存隐式第二配置。
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

    /// down 保留原软件包和分片责任；恢复后同一 Work 继续，不 reset 设备或 TCP。
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

    /// 默认路由只修改实际 eth0 的库 Routes，重复和删除缺项保留各自错误。
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

    /// lo 读取真实 IP 交付计数，eth0 读取原 NIC 完成与 RX 收权计数。
    pub fn statistics(&self, id: InterfaceId) -> Result<InterfaceStats, FsError> {
        self.interface_snapshot(id)?;
        match id {
            InterfaceId::Loopback => Ok(*self.loopback_stats.lock()),
            InterfaceId::Ethernet => Ok(self.nic.get().ok_or(FsError::NoDevice)?.statistics()),
        }
    }

    /// 所有连接和域 Work 共用这份执行域及原 per-CPU pools。
    pub(crate) fn work_queue(&self) -> &WorkQueue {
        self.queue.get().expect("network queue is not prepared")
    }

    /// 连接编号不回绕，失败准备也不重新使用已经分配的身份。
    pub(crate) fn allocate_identity(&self) -> u64 {
        self.next_connection
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .expect("network connection identity exhausted")
    }

    /// TCP 工作与期限按节点准备次序轮转，调度位置独立于端点和租约身份。
    pub(crate) fn allocate_protocol_cpu(&self) -> usize {
        self.next_protocol_cpu.fetch_add(1, Ordering::Relaxed) % cpu::count()
    }

    /// 关门后所有外部生产和期限重排都停止。
    pub(crate) fn is_stopping(&self) -> bool {
        self.stopping.load(Ordering::Acquire)
    }

    /// 记录实际端点拥有数，只有计数不可表示或关门才拒绝准备。
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

    /// 从真实容量和本域身份准备核心，节点所有资源齐备后才发表表拥有。
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

    /// child 和普通连接共享同一环境来源，只复制实际查询到的缓冲容量。
    pub(crate) fn new_core(
        &self,
        id: u64,
        rx_len: usize,
        tx_len: usize,
    ) -> Result<TcpCore, FsError> {
        let seed = self.seed.wrapping_add(id.wrapping_mul(0x9e3779b97f4a7c15));
        TcpCore::try_new(rx_len, tx_len, LOOPBACK_MTU, seed)
    }

    /// UDP 与 TCP 取得同一节点额度，但各自拥有独立端口索引。
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

    /// 在同一准入内准备两份节点额度和身份，完整 pair 成功后才交根表拥有。
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

    /// 软件报文和连接使用同一真实 IP MTU。
    pub fn loopback_mtu(&self) -> usize {
        LOOPBACK_MTU
    }

    /// 校验路由后只移动 owned 包，未接受时将完整拥有交回发送者。
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

    /// 已发表 TCP 的管理暂停沿原容量等待，不让普通 down 错误终止未 ACK 数据。
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
        // 管理状态重查和登记与 up 提交串行，唤醒不能先于暂停登记而丢失。
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

    /// 管理暂停要求库中仍存在实际路由，删除默认或错误源地址不能成为容量等待。
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

    /// 配置解锁后只消费入口时已有的节点，通知期间重新登记留待真实恢复。
    fn configuration_changed(&self) {
        let count = self.egress.lock().waiting.len();
        self.wake_egress_waiters(count);
        self.kick_egress();
    }

    /// 每次摘首后解根锁，再调用原通知和释放轻量节点拥有。
    fn wake_egress_waiters(&self, count: usize) {
        for _ in 0..count {
            let node = self.egress.lock().waiting.pop();
            let Some(node) = node else {
                break;
            };
            self.notify_egress(node.id());
        }
    }

    /// 真实 TX 完成、邻居输入或库期限只安排根 Work，不主动轮询连接。
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

    /// 普通 owned 发送失败释放未接受包，保留实际路由或容量错误。
    pub fn send_ip(&self, packet: OwnedPacket) -> Result<(), FsError> {
        self.try_emit(packet).map_err(|(error, _packet)| error)
    }

    /// 容量检查和稳定节点登记由同一根短锁串行，不在协议回调扩容。
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

    /// 末关闭只撤本端点身份，不影响同一共享容量上的其它等待者。
    pub(crate) fn cancel_egress_wait(&self, id: u64) {
        let node = self.egress.lock().waiting.remove(id);
        drop(node);
    }

    /// 表锁只克隆本身份拥有，解锁后定向安排 TCP Work 或通知 UDP 的同一源。
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

    /// 表中成功移出的同一 Arc 接入独立退休投影，当前 callback 不等待自己的 Work。
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

    /// UDP 回调只移交真实拥有，另一个根 Work 才同步结束该端点全尾。
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

    /// 退休和末关闭使用同一容量等待撤销及已有回收票。
    fn schedule_reaper(&self, id: u64) {
        self.cancel_egress_wait(id);
        let _ = self
            .reaper
            .get()
            .expect("network reaper is not prepared")
            .as_ref()
            .queue();
    }

    /// 测试只观察真实容量等待登记，查询不安排或修改连接进度。
    #[cfg(feature = "ktest")]
    pub(crate) fn egress_waiting(&self, id: u64) -> bool {
        self.egress.lock().waiting.contains(id)
    }

    /// 只读实际 EndpointSlot 数，包含准备中、表、退休和外部真实拥有。
    #[cfg(feature = "ktest")]
    pub fn ktest_live_endpoints(&self) -> usize {
        self.endpoints.load(Ordering::Acquire)
    }

    /// 为选定目的端口关闭 SYN 交付窗口；本域同时仅能存在一个具体控制。
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

    /// 只观察原 SYN 包是否已在真正交付窗口移入 held。
    #[cfg(feature = "ktest")]
    pub fn ktest_has_held_syn(&self) -> bool {
        matches!(
            self.egress.lock().packet_control.as_ref(),
            Some(TestPacketControl::HoldSyn { held: Some(_), .. })
        )
    }

    /// 在同一队列短锁内移动原包回 lo；未接受时 held 的原拥有保持不变。
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

    /// 选择一笔真实带 payload 的普通 TCP 段丢失，不修改 raw 时钟或重传算法。
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

    /// 丢失所选端口的首笔真实空 ACK，后续交付不变；关门或已有控制时拒绝配置。
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

    /// 只读取原 ACK 在 lo 交付边界实际丢失的事实，不安排协议进度。
    #[cfg(feature = "ktest")]
    pub fn ktest_handshake_ack_dropped(&self) -> bool {
        matches!(
            self.egress.lock().packet_control.as_ref(),
            Some(TestPacketControl::DropHandshakeAck { dropped: true, .. })
        )
    }

    /// 返回实际交付边界的计数快照，查询本身不推动包或协议。
    #[cfg(feature = "ktest")]
    pub fn ktest_loss_observation(&self) -> TestLossObservation {
        match self.egress.lock().packet_control.as_ref() {
            Some(TestPacketControl::DropData { observation, .. }) => *observation,
            _ => TestLossObservation::default(),
        }
    }

    /// 仅允许结束已经没有 held 包的控制；控制拥有的最终析构在短锁外。
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

    /// 只在原软件包或帧确实丢弃后累计，暂存、持有和容量拒绝不计丢失。
    fn record_loopback_tx_drop(&self) {
        let mut stats = self.loopback_stats.lock();
        stats.tx_dropped = stats.tx_dropped.wrapping_add(1);
    }

    /// typed repr 借用止于短锁前；真实丢弃锁外计数，held 包仍保留拥有。
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

    /// 原队列领取槽与容量通知同锁；设备或协议处理、最后 packet put 全在短锁外。
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

    /// 一个接口同时只 dispatch 一条队列责任，最后帧交出后才归还原32项额度。
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

    /// 取得实际接口保护，控制调用不能在持表锁时进入此入口。
    fn interface(&self, id: InterfaceId) -> Option<&SleepMutex<Option<InterfaceState>>> {
        match id {
            InterfaceId::Loopback => Some(&self.loopback),
            InterfaceId::Ethernet => self.ethernet.get(),
        }
    }

    /// 输入借同一 holder 后备；Ethernet 仅核自己的应答槽，缺后备时在接口锁外准备 Vec。
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

    /// 只选择已有空帧和空 fragmenter 的原槽，不移动等待中的队列项。
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

    /// 完整 IP 输出交换到预备 RX 存储，再经同一库解析和控制链回到端点。
    fn receive_loopback_frame(&self) -> Result<bool, FsError> {
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

    /// 输入应答先于普通输出交 NIC；满槽或错误恢复同一份帧，只有普通输出结束自己的 binding。
    fn transmit_ethernet_frame(&self) -> Result<Option<bool>, FsError> {
        let Some(ethernet) = self.ethernet.get() else {
            return Ok(None);
        };
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
        let accepted = self
            .nic
            .get()
            .expect("Ethernet interface without NIC")
            .try_transmit(len, |frame| frame.copy_from_slice(&bytes[..len]));
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

    /// 只推进库已有余片，TxToken 空槽事实是唯一正常发送提交条件。
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

    /// 根 Work 按预算处理软件帧和 owned 包；NIC 满或邻居等待不产生忙重排。
    fn receive_egress(&self) {
        if self.is_stopping() {
            return;
        }
        self.maintain_interfaces();
        let mut blocked = [false; INTERFACE_COUNT];
        let mut stalled = [None; LOOPBACK_CAPACITY];
        let mut tx_full = false;
        let mut consumed = 0;
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
        self.arm_interface_timer();
        if consumed == PROTOCOL_BUDGET && self.actionable_egress(blocked, tx_full, &stalled) {
            self.kick_egress();
        }
    }

    /// 预算尾包含已编码的输入应答；真实 TX 满仍只等 completion，不因待发帧忙重排。
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

    /// 即使没有下一包也执行成熟库的到期重组回收。
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

    /// 最早重组到期和真实待发目的的邻居期限共享一份根 Timer。
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

    /// RX 借收权 DMA 前缀与独立应答槽，库捕获后解接口锁再交端点，不占用普通输出帧。
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

    /// 具体退休拥有在短锁外覆盖 Timer 和 Work 的完整尾部。
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

    /// 先关生产与全部协议回调，随后确认 NIC 停止；错误保留原根和 IRQ 上下文。
    fn stop_sync(&self) -> Result<(), FsError> {
        {
            let _admission = self.admission.lock();
            let mut egress = self.egress.lock();
            assert!(
                !self.stopping.swap(true, Ordering::AcqRel),
                "network domain stopped twice"
            );
            egress.open = false;
        }
        // Timer 的重提边界与关门串行；等待 callback 全尾时不持这把短锁。
        {
            let _control = self.timer_control.lock();
        }
        if let Some(timer) = self.interface_timer.get() {
            timer.as_ref().close_sync();
        }
        self.ingress.get().unwrap().as_ref().close_sync();
        self.reaper.get().unwrap().as_ref().close_sync();
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
        if let Some(nic) = self.nic.get() {
            nic.stop_sync().map_err(NetDeviceError::into_fs_error)?;
        }
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
    /// Arc 的最后释放不承担 join，服务必须已由唯一普通任务拥有者停稳。
    fn drop(&mut self) {
        assert!(
            self.stopped.load(Ordering::Acquire),
            "network domain dropped before stop"
        );
    }
}

/// 根拥有者从准备开始一直保活 NetStack，直到 ingress Work 同步尾部结束。
fn run_ingress(work: Pin<&Work>) {
    // SAFETY: context 来自稳定 Arc payload；唯一服务拥有者等待本 Work 全尾后才释放域。
    let net = unsafe { &*(work.context() as *const NetStack) };
    net.receive_egress();
}

/// 退休 Work 不同步关闭自身，只等待从具体退休队列移出的连接。
fn run_retired(work: Pin<&Work>) {
    // SAFETY: context 来自稳定 Arc payload；根拥有者在本 Work 同步全尾结束前保活域。
    let net = unsafe { &*(work.context() as *const NetStack) };
    net.reap_retired();
}

/// Timer context 的存活由原 Root owner 覆盖至同步尾部，只移交原 Work。
fn interface_expired(timer: Pin<&Timer>) {
    // SAFETY: context 来自稳定根 Arc，Root owner 在 Timer close_sync 全尾前保留同一强拥有。
    let net = unsafe { &*(timer.context() as *const NetStack) };
    net.kick_egress();
}

/// 预留空 capture 后备；初始化和消费后补存都只能在接口处理锁外调用。
fn prepare_ingress_storage() -> Result<Vec<u8>, FsError> {
    let mut storage = Vec::new();
    storage
        .try_reserve_exact(LOOPBACK_MTU)
        .map_err(|_| FsError::NoMemory)?;
    Ok(storage)
}

/// 库微秒与生产 monotonic 纳秒共享基准，不另建协议时钟。
fn interface_now() -> Instant {
    Instant::from_micros(
        i64::try_from(time::now().as_nanos() / NSEC_PER_USEC).expect("interface clock exhausted"),
    )
}

/// 合并真实可选期限，不把没有责任的接口转换成立即重排。
fn earlier_deadline(left: Option<Instant>, right: Option<Instant>) -> Option<Instant> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left.min(right)),
        (left, right) => left.or(right),
    }
}

/// 正常静态 IPv4 初次赋址采用 A/B/C 类前缀；调用者已核单播边界。
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

/// 连续 IPv4 mask 的零前缀单独处理，避免移位宽度等于整数位数。
fn ipv4_mask(prefix_len: u8) -> u32 {
    if prefix_len == 0 {
        0
    } else {
        u32::MAX << (IPV4_ADDRESS_BITS - prefix_len)
    }
}

/// proc 直连目的由库 CIDR 的地址与前缀计算网络地址。
fn ipv4_network(address: [u8; 4], prefix_len: u8) -> [u8; 4] {
    (u32::from_be_bytes(address) & ipv4_mask(prefix_len)).to_be_bytes()
}

/// 原文件错误直接表达实际路由、容量和合法报文边界。
fn dispatch_error(error: DispatchError) -> FsError {
    match error {
        DispatchError::NoRoute => FsError::NetworkUnreachable,
        DispatchError::NeighborPending | DispatchError::Exhausted => FsError::WouldBlock,
        DispatchError::PacketTooLarge => FsError::MessageTooLarge,
    }
}
