//! VirtIO 传输层公共构建设施与 VirtIO 块设备（`VirtioDevice`）控制内存管理模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于设备枚举层（[`crate::drivers`] / [`crate::drivers::pci`]）与具体设备子系统之间，承担两项核心职责：
//! 1. **公共传输层构建**：向块设备子系统（[`crate::block`]）和 VirtIO 网卡驱动（[`crate::drivers::virtio_net`]）
//!    提供统一的 MMIO 传输（[`prepare_mmio_transport`]）与 Modern PCI 传输（[`prepare_pci_transport`]）构造函数，
//!    以及 QEMU `virt` 平台一致性 DMA 域查询与错误映射工具。
//! 2. **块设备硬件与控制 DMA 封装**：通过 [`VirtioDevice`] 将 `virtio-drivers` 库的 `VirtIOBlk` 驱动实例、
//!    Split VirtQueue 描述符环物理页（`ring`）以及请求头/状态字缓冲页（`pdu`）绑定在同一生命周期内，
//!    并通过 `must_reset` 标志强制要求设备停止确认后才能释放底层物理页。
//!
//! ## 核心布局与生命周期机制
//!
//! ### 1. VirtIO PCI Vendor Capability 到 BAR 窗口的解析布局
//! ```text
//!   PCI 配置空间 (256 Bytes)                      已映射的 PCI Memory BAR 物理窗口
//!  ┌────────────────────────────┐               ┌───────────────────────────────────┐
//!  │ Vendor Capability (0x09)   │               │ BAR Base                          │
//!  │  +0x02 cap_len / cfg_type  │               │   ...                             │
//!  │  +0x04 bar (BAR 索引)      │──────────────►│ + offset ──► ┌─────────────────┐  │
//!  │  +0x08 offset (BAR 内偏移) │               │              │ Common / Notify │  │
//!  │  +0x0c length (窗口字节数) │               │              │ / ISR / Device  │  │
//!  │  +0x10 notify_off_mult     │ (仅 Notify)   │ + offset+len └─────────────────┘  │
//!  └────────────────────────────┘               └───────────────────────────────────┘
//! ```
//!
//! ### 2. `VirtioDevice` 生命周期与 DMA 物理页释放顺序
//! ```text
//!   prepare_mmio / prepare_pci (must_reset = false: 硬件已复位并完成特性协商，尚未写入队列地址)
//!       │
//!       ├──► 若准备后续队列资源失败直接 Drop ──► 安全：硬件未知晓 ring/pdu 地址，直接析构
//!       │
//!       ▼
//!   publish() / driver_mut()   (must_reset = true: 硬件可能持有或访问 ring/pdu 的 DMA 地址)
//!       │
//!       ▼
//!   request_reset() ──► 轮询 reset_complete() 读回 status == 0 并执行全内存屏障
//!       │               (must_reset = false: 确认硬件 DMA 彻底停止)
//!       ▼
//!   Drop::drop():
//!     Step 1: 析构 driver (结束 CPU 侧对队列和 PDU 裸指针的包装视图)
//!     Step 2: 释放 pdu 与 ring 物理页 (归还底层伙伴系统，杜绝硬件后台 DMA 踩内存或 UAF)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `VirtioDevice` 本身不内置锁，上层块设备后端（`VirtioBackend` / `Disk`）通过队列自旋锁串行化对
//!   [`VirtioDevice::driver_mut`]、[`VirtioDevice::ack_interrupt`] 及复位接口的调用。
//! - [`prepare_mmio_transport`] 与 [`prepare_pci_transport`] 必须在可调度的普通任务（`Task`）上下文中调用，
//!   内部通过 `task::current_task()` 校验任务上下文，不可在裸中断上下文中执行初始化。

#[cfg(feature = "ktest")]
pub(crate) mod ktest_prepare;

use crate::{
    boot::{Platform, Region},
    drivers::{
        mmio_region_address,
        pci::{PciDevice, PciInventory},
        MmioBlockCandidate, ProbeError,
    },
    irq::IrqNumber,
    mm::dma::{CoherentDma, DmaDomain, DmaError, KernelHal},
    task,
};
use core::convert::TryFrom;
use core::{mem::ManuallyDrop, ptr::NonNull};
pub(crate) use virtio_drivers::transport::pci::VIRTIO_VENDOR_ID;
use virtio_drivers::{
    device::blk::{BlkInfo, VirtIOBlk, PDU_STRIDE},
    queue::{RingLayout, VirtQueue},
    transport::{
        mmio::{MmioError, MmioTransport, VirtIOHeader},
        pci::{
            bus::{DeviceFunction, PCI_CAP_ID_VNDR},
            PciRegion, PciTransport, SelectedConstruct, VirtioPciError, VIRTIO_PCI_CAP_COMMON_CFG,
            VIRTIO_PCI_CAP_DEVICE_CFG, VIRTIO_PCI_CAP_NOTIFY_CFG,
        },
        DeviceStatus, DeviceType, InterruptStatus, SomeTransport, Transport,
    },
    DmaRegion, Error, Hal,
};

/// Legacy / Transitional VirtIO 块设备的 PCI Device ID（`0x1001`）。
pub(crate) const VIRTIO_PCI_BLOCK_LEGACY: u16 = 0x1001;
/// Modern VirtIO 1.0+ 块设备的 PCI Device ID（`0x1042`）。
pub(crate) const VIRTIO_PCI_BLOCK_MODERN: u16 =
    VIRTIO_PCI_MODERN_DEVICE_BASE + DeviceType::Block as u16;
/// Legacy / Transitional VirtIO 网络设备的 PCI Device ID（`0x1000`）。
pub(crate) const VIRTIO_PCI_NET_LEGACY: u16 = 0x1000;
/// Modern VirtIO 1.0+ 网络设备的 PCI Device ID（`0x1041`）。
pub(crate) const VIRTIO_PCI_NET_MODERN: u16 =
    VIRTIO_PCI_MODERN_DEVICE_BASE + DeviceType::Network as u16;

// 内核支持的 Split VirtQueue 深度范围与块设备控制段开销，必须与底层 `virtio-drivers` 协议实现保持一致。
/// VirtIO 队列允许协商的最小描述符数量（至少容纳单次块请求的头、数据、状态三段及余量）。
pub(crate) const VIRTIO_MIN_QUEUE_DEPTH: u16 = 4;
/// 内核限制的最大 VirtIO 队列描述符深度。
pub(crate) const VIRTIO_MAX_QUEUE_DEPTH: u16 = 128;
/// 单个 VirtIO 块请求除数据段外额外占用的固定描述符数量（1 个 16 字节请求头段 + 1 个 1 字节状态尾段）。
pub(crate) const VIRTIO_CONTROL_SEGMENTS: u16 = 2;
/// VirtIO 块请求末尾设备写回的状态字段长度（1 字节 `u8`）。
pub(crate) const VIRTIO_STATUS_BYTES: usize = core::mem::size_of::<u8>();
/// Split VirtQueue 单个描述符表项的字节大小（16 字节：8B 地址 + 4B 长度 + 2B flags + 2B next）。
#[cfg(feature = "ktest")]
pub(crate) const SPLIT_RING_DESCRIPTOR_BYTES: usize = 16;
/// Modern VirtIO 1.0 PCI Device ID 的起始基值（实际 ID = `0x1040 + VirtIO DeviceType`）。
const VIRTIO_PCI_MODERN_DEVICE_BASE: u16 = 0x1040;

/// 标准 PCI 2.x/3.0 配置空间字节上限（256 字节），用于校验 Capability 结构不越界。
const PCI_CONFIG_SPACE_BYTES: usize = 256;
/// PCI 配置空间起始身份寄存器偏移（`0x00`，低 16 位为 Vendor ID，高 16 位为 Device ID）。
const PCI_IDENTITY: u8 = 0;
/// `virtio_pci_cap` 结构中 `bar` 字段（目标 BAR 索引）相对于 Capability 起始位置的字节偏移。
const VIRTIO_PCI_CAP_BAR: u8 = 4;
/// `virtio_pci_cap` 结构中 `offset` 字段（BAR 内字节偏移）相对于 Capability 起始位置的字节偏移。
const VIRTIO_PCI_CAP_OFFSET: u8 = 8;
/// `virtio_pci_cap` 结构中 `length` 字段（窗口字节长度）相对于 Capability 起始位置的字节偏移。
const VIRTIO_PCI_CAP_LENGTH: u8 = 12;
/// `virtio_pci_notify_cap` 扩展结构中 `notify_off_multiplier` 字段相对于 Capability 起始位置的字节偏移。
const VIRTIO_PCI_NOTIFY_CAP_MULT: u8 = 16;
/// 通用 `virtio_pci_cap` 结构的最小字节长度（16 字节）。
const VIRTIO_PCI_CAP_BYTES: u8 = 16;
/// 通知配置 `virtio_pci_notify_cap` 结构的最小字节长度（通用 16 字节 + 4 字节乘数 = 20 字节）。
const VIRTIO_PCI_NOTIFY_CAP_BYTES: u8 = VIRTIO_PCI_CAP_BYTES + core::mem::size_of::<u32>() as u8;
/// 需要从 PCI Capability 链表中提取的协议配置窗口种类总数（Common=1, Notify=2, ISR=3, Device=4，共 4 类）。
const VIRTIO_PCI_CAP_REGION_COUNT: usize = VIRTIO_PCI_CAP_DEVICE_CFG as usize;

/// VirtIO 设备在传输层构建、特性协商、DMA 队列分配、启动及停机过程中的错误类型。
///
/// 严格区分内存耗尽、固件/总线资源异常、协议协商拒绝与停机复位超时，以便调用方采取对应的恢复或资源保留策略。
#[derive(Debug)]
pub enum VirtioInitError {
    /// 分配控制环、缓冲池或队列影子结构时物理页或内核堆内存不足。
    NoMemory {
        #[cfg_attr(
            not(feature = "ktest"),
            expect(
                dead_code,
                reason = "准备失败日志保留设备状态；ktest 另核 OOM 时的真实协商状态"
            )
        )]
        /// 内存分配失败瞬间从硬件读到的设备状态字（如 `ACKNOWLEDGE | DRIVER | FEATURES_OK`），用于诊断日志与测试断言。
        status: DeviceStatus,
    },
    /// 平台不支持、寄存器窗口映射无效、身份不一致或中断/BAR 资源缺失。
    Resource(
        #[expect(
            dead_code,
            reason = "STORAGE INIT 的 Debug 日志保留原库存或映射失败原因"
        )]
        ProbeError,
    ),
    /// VirtIO MMIO 寄存器魔数、版本号或设备类型校验失败。
    Mmio(
        #[expect(
            dead_code,
            reason = "STORAGE INIT 的 Debug 日志保留 MMIO 身份和版本错误"
        )]
        MmioError,
    ),
    /// VirtIO PCI Capability 缺失必需配置区（如 Common/Notify/ISR）或传输层校验失败。
    Pci(
        #[expect(
            dead_code,
            reason = "STORAGE INIT 的 Debug 日志保留所选 PCI capability 错误"
        )]
        VirtioPciError,
    ),
    /// 控制区或缓冲池在建立 DMA 映射或执行缓存一致性维护时失败（不含由 `NoMemory` 单独表示的内存耗尽）。
    Dma {
        /// 底层 DMA 子系统返回的具体错误原因。
        error: DmaError,
        #[cfg_attr(
            not(feature = "ktest"),
            expect(
                dead_code,
                reason = "DMA 准备失败日志保留设备状态；ktest 另核真实 OOM 状态"
            )
        )]
        /// DMA 操作失败瞬间硬件所处的 `DeviceStatus` 状态字快照。
        status: DeviceStatus,
    },
    /// `virtio-drivers` 协议层返回错误（如特性集不兼容、队列参数非法、硬件复位未完成或设备故障）。
    Protocol(Error),
    /// 已向硬件发起复位请求，但在规定超时时间内设备 `status` 仍未归零，无法确认硬件 DMA 已停止。
    StopUnconfirmed {
        #[expect(
            dead_code,
            reason = "停止失败的 Debug 日志须保留超时瞬间设备状态，不能以复位后状态替代"
        )]
        /// 等待复位超时瞬间读到的硬件 `DeviceStatus` 状态字，用于定位卡死阶段。
        status: DeviceStatus,
    },
}

impl From<ProbeError> for VirtioInitError {
    /// 将设备发现与地址映射阶段的 [`ProbeError`] 包装为 [`VirtioInitError::Resource`]。
    fn from(error: ProbeError) -> Self {
        Self::Resource(error)
    }
}

impl From<MmioError> for VirtioInitError {
    /// 将 MMIO 传输头校验产生的 [`MmioError`] 包装为 [`VirtioInitError::Mmio`]。
    fn from(error: MmioError) -> Self {
        Self::Mmio(error)
    }
}

impl From<VirtioPciError> for VirtioInitError {
    /// 将 PCI Capability 解析或传输构造产生的 [`VirtioPciError`] 包装为 [`VirtioInitError::Pci`]。
    fn from(error: VirtioPciError) -> Self {
        Self::Pci(error)
    }
}

impl VirtioInitError {
    /// 将准备阶段的 [`DmaError`] 转换为初始化错误，并附带当前硬件状态字 `status`（尚未触发额外复位前读取）。
    ///
    /// 若底层错误为 [`DmaError::NoMemory`]，归一为 [`Self::NoMemory`] 以便上层统一识别内存不足。
    pub(crate) fn dma_prepare(error: DmaError, status: DeviceStatus) -> Self {
        if error == DmaError::NoMemory {
            Self::NoMemory { status }
        } else {
            Self::Dma { error, status }
        }
    }

    /// 将 `VirtQueue` 内部影子结构初始化失败的 [`Error`] 转换为初始化错误，并在 `NoMemory` 时附带当前设备状态字 `status`。
    pub(crate) fn queue_prepare(error: Error, status: DeviceStatus) -> Self {
        if error == Error::NoMemory {
            Self::NoMemory { status }
        } else {
            Self::Protocol(error)
        }
    }
}

impl From<Error> for VirtioInitError {
    /// 将不携带设备状态快照的通用 VirtIO 协议错误包装为 [`VirtioInitError::Protocol`]。
    fn from(error: Error) -> Self {
        Self::Protocol(error)
    }
}

/// VirtIO 硬件实例在所属总线上的不可变物理身份标识。
///
/// 将 MMIO 物理基址与 PCI 总线 `bus:device.function` 明确隔离在不同变体中，避免跨总线类型混用地址。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VirtioIdentity {
    /// 挂载于平台内存映射总线（VirtIO-MMIO）上的设备身份。
    Mmio {
        /// 设备寄存器窗口的起始 CPU 物理地址（字节）。
        physical: usize,
        /// 设备发现时读取到的 VirtIO MMIO 协议版本号（1 为 Legacy，2 为 Modern）。
        version: u32,
        /// 设备发现时读取到的 32 位 VirtIO Vendor ID。
        vendor_id: u32,
    },
    /// 挂载于 PCI / PCIe 总线上的设备身份。
    Pci {
        /// PCI 总线号（Bus Number）。
        bus: u8,
        /// PCI 设备/槽位号（Device Number，0..32）。
        device: u8,
        /// PCI 功能号（Function Number，0..8）。
        function: u8,
        /// PCI 配置空间中的 16 位厂商标识（VirtIO 标准值为 `0x1af4`）。
        vendor_id: u16,
        /// PCI 配置空间中的 16 位设备标识（区分 Legacy/Modern 及块/网卡等设备类型）。
        device_id: u16,
    },
}

/// 获取 QEMU `virt` 虚拟平台专用的硬件缓存一致性（Coherent）恒等映射 DMA 域。
///
/// 可在任意上下文调用。当前内核仅在 QEMU `virt` 平台启用 VirtIO 设备；若在非 QEMU 平台上调用，
/// 将返回 [`ProbeError::Resource`] 错误以阻止使用错误的地址转换或缓存假设。
pub(crate) fn qemu_dma_domain() -> Result<DmaDomain, VirtioInitError> {
    if crate::platform::current() != Platform::QemuVirt {
        return Err(ProbeError::Resource("VirtIO DMA is supported only on QEMU virt").into());
    }
    Ok(DmaDomain::qemu_coherent())
}

/// 记录运行期 DMA 错误日志，并将内核 [`DmaError`] 映射为 `virtio-drivers` 协议层可处理的 [`Error`]。
pub(crate) fn dma_protocol(error: DmaError) -> Error {
    crate::klog!(crate::klog::Level::Error, "VirtIO DMA failed: {:?}", error);
    match error {
        DmaError::NoMemory => Error::NoMemory,
        DmaError::InvalidRange
        | DmaError::AddressLimit
        | DmaError::InvalidDomain
        | DmaError::Alignment => Error::InvalidParam,
        DmaError::CacheMaintenance(_) => Error::IoError,
    }
}

/// 基于设备枚举阶段记录的 MMIO 候选信息构造 `MmioTransport`，并二次核对硬件版本与厂商身份。
///
/// 必须在普通任务（`Task`）上下文调用。本函数仅完成寄存器指针包装与身份校验，不会执行设备复位、特性协商或队列发布。
///
/// # Safety
/// 1. 调用方必须保证该硬件实例由当前驱动独占，不存在其他驱动或测试并发访问同一 `registers` 窗口。
/// 2. `registers` 指定的完整物理地址区间必须已完成永久、强序（Uncached Device）内核虚拟地址映射。
/// 3. 硬件及其虚拟地址映射必须在返回传输对象、队列通知（Doorbell）及中断处理函数的整个存活期内保持有效，
///    直至所有并发访问者退出且硬件确认完成 `reset`。
pub(crate) unsafe fn prepare_mmio_transport(
    registers: Region,
    version: u32,
    vendor_id: u32,
    interrupt: IrqNumber,
) -> Result<(SomeTransport<'static>, VirtioIdentity, IrqNumber), VirtioInitError> {
    drop(task::current_task());
    let base = mmio_region_address(registers)?;
    let size = registers
        .end
        .checked_sub(registers.start)
        .ok_or(ProbeError::Resource("invalid MMIO extent"))?;
    let header = NonNull::new(base as *mut VirtIOHeader)
        .ok_or(ProbeError::Resource("null MMIO register address"))?;
    // SAFETY:
    // 1. 上方 `mmio_region_address`、`checked_sub` 与 `NonNull::new` 已确认寄存器虚拟基址非空且区间长度无溢出。
    // 2. 调用方契约保证了该物理窗口属于真实 VirtIO-MMIO 硬件、已完成永久强序映射且由当前驱动独占保活；
    //    `MmioTransport::new` 内部会进一步校验魔数、对齐与寄存器窗口最小长度。
    let transport = unsafe { MmioTransport::new(header, size) }?;
    if u32::from(transport.version()) != version || transport.vendor_id() != vendor_id {
        return Err(ProbeError::Resource("MMIO inventory identity changed").into());
    }
    Ok((
        transport.into(),
        VirtioIdentity::Mmio {
            physical: registers.start,
            version,
            vendor_id,
        },
        interrupt,
    ))
}

/// 基于 PCI 枚举库存中已扫描的设备记录解析 VirtIO Vendor Capabilities 并构造 `PciTransport`。
///
/// 必须在普通任务（`Task`）上下文调用。本函数直接复用 `inventory` 已发现的 Capability 偏移、BAR 物理区间与 INTx 中断号，
/// 不会重新枚举总线或修改 BAR 分配，也不会执行特性协商或队列发布。
///
/// # Safety
/// 1. 调用方必须保证 `device` 来自给定的 `inventory`，且该 PCI 功能（BDF）由当前驱动独占控制。
/// 2. `device` 所引用的全部 Memory BAR 物理地址窗口必须已完成永久、强序（Uncached Device）映射。
/// 3. 设备与 BAR 映射必须在传输层、Doorbell 通知及中断处理函数的整个访问期内持续有效，直到所有消费者退出且确认硬件 `reset`。
pub(crate) unsafe fn prepare_pci_transport(
    inventory: &PciInventory,
    device: &PciDevice,
) -> Result<(SomeTransport<'static>, VirtioIdentity, IrqNumber), VirtioInitError> {
    drop(task::current_task());
    let selected = selected_construct(inventory, device)?;
    let interrupt = device.interrupt.ok_or(ProbeError::Resource(
        "VirtIO PCI device has no INTx resource",
    ))?;
    // SAFETY:
    // 1. `selected_construct` 已核对 PCI Vendor/Device ID 未发生漂移，并验证了 Common/Notify/ISR/Device
    //    四个 Capability 窗口均完整落在已分配的 BAR 边界与已映射的强序虚拟地址空间内。
    // 2. 调用方契约保证了该 PCI 功能独占访问，且相关 BAR 映射在所有消费者退出与硬件复位完成前永久有效。
    let transport = unsafe { PciTransport::new(selected) }?;
    Ok((
        transport.into(),
        VirtioIdentity::Pci {
            bus: device.bus,
            device: device.device,
            function: device.function,
            vendor_id: device.vendor_id,
            device_id: device.device_id,
        },
        interrupt,
    ))
}

/// VirtIO 块设备硬件驱动与控制区物理页的联合拥有者。
///
/// 将 `virtio-drivers` 的块设备状态机与底层 Split Ring 描述符环页（`ring`）、请求头/状态缓冲池页（`pdu`）封装在一起。
/// 所有控制区字段均包裹在 [`ManuallyDrop`] 中，由 [`Drop`] 实现严格保证“先确认硬件停止、先销毁上层驱动视图、最后归还物理页”的顺序。
pub struct VirtioDevice {
    /// `virtio-drivers` 块设备驱动实例，内部持有 `transport` 以及借用 `ring` 和 `pdu` 裸地址的队列视图。
    driver: ManuallyDrop<VirtIOBlk<KernelHal, SomeTransport<'static>>>,
    /// Split VirtQueue 描述符表、Available 环与 Used 环所在的连续一致性 DMA 物理页。
    ring: ManuallyDrop<CoherentDma>,
    /// 每个并发块请求槽位（Tag）对应的 16 字节请求头与 1 字节设备状态字的连续一致性 DMA 物理页。
    pdu: ManuallyDrop<CoherentDma>,
    /// 设备所属总线的不可变物理身份（MMIO 基址或 PCI BDF），用于日志追踪与测试校验。
    identity: VirtioIdentity,
    /// 设备绑定的外部中断号（MMIO SPI 或 PCI INTx 路由后的系统中断号）。
    interrupt: IrqNumber,
    /// 硬件是否可能已知晓队列地址或被可变访问过：一旦置为 `true`，析构前必须通过 [`Self::reset_complete`] 确认硬件状态归零。
    must_reset: bool,
}

/// 内核自测（`ktest`）专用的块设备准备期状态快照，用于在注入失败销毁实例前只读观测硬件与队列状态。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct TestPrepareSnapshot {
    /// 被测块设备的总线物理身份。
    pub identity: VirtioIdentity,
    /// 采样瞬间从硬件寄存器读取的 `DeviceStatus` 状态字。
    pub status: DeviceStatus,
    /// 采样瞬间从 Split VirtQueue 的 `avail.idx` 字段直接读取的可用环索引。
    pub avail_index: u16,
    /// 驱动影子队列当前剩余的空闲描述符数量。
    pub available_descriptors: usize,
    /// 当前 `VirtioDevice` 是否已标记必须执行硬件复位后才能析构（`must_reset`）。
    pub requires_reset: bool,
    /// 该设备中断号上当前已注册的中断处理动作（Action）数量。
    pub irq_actions: usize,
}

impl VirtioDevice {
    /// 从已枚举的 VirtIO-MMIO 块设备候选信息初始化 [`VirtioDevice`]（完成复位、特性协商与控制页分配，暂不发布队列）。
    ///
    /// 必须在普通任务（`Task`）上下文调用。返回时硬件尚未收到队列物理地址，也未置位 `DRIVER_OK`。
    ///
    /// # Safety
    /// 1. 调用方必须独占 `candidate` 对应的真实硬件，且 `candidate` 各字段来自同一次硬件发现。
    /// 2. `candidate.registers` 完整物理区间必须已完成永久强序映射，并在整个驱动生命周期、中断处理及硬件复位完成前保持有效。
    pub unsafe fn prepare_mmio(candidate: &MmioBlockCandidate) -> Result<Self, VirtioInitError> {
        // SAFETY: 调用方契约保证了 `candidate` 硬件独占、寄存器窗口永久强序映射有效且覆盖所有后续消费者生命周期。
        let (transport, identity, interrupt) = unsafe {
            prepare_mmio_transport(
                candidate.registers,
                candidate.version,
                candidate.vendor_id,
                candidate.interrupt,
            )
        }?;
        Self::prepare(transport, identity, interrupt)
    }

    /// 从 PCI 枚举库存中选定的 VirtIO 块设备功能初始化 [`VirtioDevice`]（完成复位、特性协商与控制页分配，暂不发布队列）。
    ///
    /// 必须在普通任务（`Task`）上下文调用。直接复用 `inventory` 已解析的 BDF、Capability 与 BAR 信息。
    ///
    /// # Safety
    /// 1. `device` 必须来自 `inventory`，且调用方独占该真实 PCI 硬件功能。
    /// 2. `device` 关联的所有 Memory BAR 完整物理窗口必须已完成永久强序映射，并在通知、中断处理及确认硬件复位前保持有效。
    pub unsafe fn prepare_pci(
        inventory: &PciInventory,
        device: &PciDevice,
    ) -> Result<Self, VirtioInitError> {
        // SAFETY: 调用方契约保证了 `device` 来自同一 `inventory`、功能独占且全部 BAR 窗口已完成永久强序映射并持续保活。
        let (transport, identity, interrupt) = unsafe { prepare_pci_transport(inventory, device) }?;
        Self::prepare(transport, identity, interrupt)
    }

    /// 执行块设备类型校验、初始复位、特性协商以及 `ring`/`pdu` 控制页分配与绑定。
    ///
    /// 在此阶段尚未向硬件寄存器写入任何队列 DMA 地址；若中途内存分配或协议校验失败，局部变量按逆序自动释放物理页是安全的。
    fn prepare(
        mut transport: SomeTransport<'static>,
        identity: VirtioIdentity,
        interrupt: IrqNumber,
    ) -> Result<Self, VirtioInitError> {
        // Step 1: 校验当前平台支持一致性 VirtIO DMA 且传输层对应的设备类型为块设备（Block）。
        let domain = qemu_dma_domain()?;
        if transport.device_type() != DeviceType::Block {
            return Err(ProbeError::Resource("selected VirtIO device is not block").into());
        }

        // Step 2: 先对硬件执行一次同步复位并插入全内存屏障，清除固件或前序测试遗留的队列状态。
        transport.request_reset();
        if !transport.reset_complete() {
            return Err(Error::ResetPending.into());
        }
        KernelHal::dma_barrier();

        // Step 3: 协商块设备特性与队列参数，并按协商出的队列深度与并发 Tag 数分配一致性 DMA 控制页。
        let info = VirtIOBlk::<KernelHal, SomeTransport<'static>>::negotiate(&mut transport)?;
        let layout = RingLayout::new(info.queue_depth, transport.requires_legacy_layout())?;
        let ring = CoherentDma::allocate(domain, layout.byte_len())
            .map_err(|error| VirtioInitError::dma_prepare(error, transport.get_status()))?;
        let pdu_size = usize::from(info.tag_count)
            .checked_mul(PDU_STRIDE)
            .ok_or(Error::InvalidParam)?;
        let pdu = CoherentDma::allocate(domain, pdu_size)
            .map_err(|error| VirtioInitError::dma_prepare(error, transport.get_status()))?;
        // Step 4: 在已分配的控制页上构建 VirtQueue 与 VirtIOBlk 驱动对象（此步骤仅在内存中初始化结构，不向硬件发布地址）。
        // SAFETY:
        // 1. `ring` 是刚由 `CoherentDma::allocate` 分配并清零的一致性连续物理页，字节长度满足 `layout` 要求。
        // 2. 此时尚未向硬件发布队列地址，无并发设备访问；若后续 `VirtIOBlk::new` 失败，`queue` 先析构、`ring` 后释放。
        let queue = unsafe { VirtQueue::<KernelHal>::new(layout, ring.region()) }
            .map_err(|error| VirtioInitError::queue_prepare(error, transport.get_status()))?;

        // SAFETY:
        // 1. `ring` 与 `pdu` 物理页与 `driver` 一同存入返回的 `VirtioDevice`，且 `Drop` 保证 `driver` 先于物理页析构。
        // 2. 一旦调用 `publish` 或 `driver_mut` 使硬件可能获知 `ring`/`pdu` 地址，`must_reset` 即置为 `true`，
        //    强制在确认硬件复位完成后才允许释放物理页。
        let driver = unsafe { VirtIOBlk::new(transport, info, queue, pdu.region()) }?;
        Ok(Self {
            driver: ManuallyDrop::new(driver),
            ring: ManuallyDrop::new(ring),
            pdu: ManuallyDrop::new(pdu),
            identity,
            interrupt,
            must_reset: false,
        })
    }

    /// 获取底层 `VirtIOBlk` 驱动实例的共享只读引用。
    pub fn driver(&self) -> &VirtIOBlk<KernelHal, SomeTransport<'static>> {
        &self.driver
    }

    /// 获取底层 `VirtIOBlk` 驱动实例的可变引用以提交请求、回收完成项或操作传输层，并自动标记硬件复位义务（`must_reset = true`）。
    ///
    /// 通常在块设备队列自旋锁保护下调用。
    ///
    /// # Safety
    /// 1. 调用方严禁通过 `core::mem::replace` / `swap` 等方式移出或替换返回的 `VirtIOBlk` 实例，它必须始终与本结构体持有的硬件及 `ring`/`pdu` 绑定。
    /// 2. 通过该引用向队列提交的每一次 I/O 请求，其数据缓冲区（Payload DMA 页）必须在设备返回对应 Used 项或确认硬件复位前持续存活。
    /// 3. 调用方必须自行串行化队列操作、Doorbell 通知与复位流程，避免并发重入。
    pub unsafe fn driver_mut(&mut self) -> &mut VirtIOBlk<KernelHal, SomeTransport<'static>> {
        self.must_reset = true;
        &mut self.driver
    }

    /// 返回特性协商确定的块设备参数快照（扇区容量、块大小、队列深度、最大段数及只读标志等）。
    pub fn info(&self) -> BlkInfo {
        self.driver.info()
    }

    /// 返回该块设备绑定的系统外部中断号。
    pub fn interrupt(&self) -> IrqNumber {
        self.interrupt
    }

    /// 返回该块设备在所属总线上的不可变物理身份（MMIO 基址或 PCI BDF）。
    pub fn identity(&self) -> VirtioIdentity {
        self.identity
    }

    /// 返回队列描述符环（`ring`）与请求头/状态区（`pdu`）的底层 `DmaRegion` 地址与长度快照（仅供诊断或测试核对地址范围，不转移所有权）。
    pub fn control_regions(&self) -> (DmaRegion, DmaRegion) {
        (self.ring.region(), self.pdu.region())
    }

    /// 从硬件寄存器读取当前的 `DeviceStatus` 状态字，供初始化错误诊断与停机轮询使用。
    pub fn status(&self) -> DeviceStatus {
        self.driver.status()
    }

    /// 在内核自测（`ktest`）中采集硬件 `status`、内存环 `avail.idx`、空闲描述符数及已注册 IRQ 动作数快照，不改动任何硬件或队列状态。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_prepare_snapshot(&self) -> TestPrepareSnapshot {
        let depth = self.info().queue_depth;
        let ring = self.ring.region();
        // Split VirtQueue 中无论是 Legacy 还是 Modern 布局，`avail` 环均紧跟在 `depth * 16B` 的描述符表之后：
        // 偏移 +0 为 `avail.flags` (u16)，偏移 +2 为 `avail.idx` (u16)；Legacy 布局仅改变后续 `used` 环的页对齐填充。
        let avail_index_offset =
            usize::from(depth) * SPLIT_RING_DESCRIPTOR_BYTES + core::mem::size_of::<u16>();
        assert!(avail_index_offset + core::mem::size_of::<u16>() <= ring.len);
        KernelHal::dma_read_barrier();
        // SAFETY:
        // 1. `&self` 保证了 `self.ring` 物理页当前存活且直接映射虚拟地址 `ring.cpu` 非空。
        // 2. 上方 `assert!` 已验证 `avail_index_offset + 2 <= ring.len`，且偏移量是 16 与 2 的整数倍，满足 `u16` 自然对齐。
        // 3. 使用 `read_volatile` 直接按值读取，避免为可能被硬件访问的 DMA 区域创建长生命周期 Rust 引用。
        let avail_index = u16::from_le(unsafe {
            core::ptr::read_volatile(ring.cpu.as_ptr().add(avail_index_offset).cast::<u16>())
        });
        TestPrepareSnapshot {
            identity: self.identity,
            status: self.status(),
            avail_index,
            available_descriptors: self.driver.available_descriptors(),
            requires_reset: self.must_reset,
            irq_actions: crate::irq::test_action_count(self.interrupt),
        }
    }

    /// 读取并确认本设备的中断状态寄存器（ISR），清除设备侧共享中断挂起原因。
    ///
    /// 在块设备中断处理或停机收尾期间由队列锁串行化调用；仅确认中断不会将已复位的设备重新标记为需要复位。
    pub fn ack_interrupt(&mut self) -> InterruptStatus {
        self.driver.ack_interrupt()
    }

    /// 执行全内存屏障并向硬件写入队列 DMA 地址及 `DRIVER_OK`，正式发布块设备队列。
    ///
    /// 在写入任何队列寄存器之前预先置位 `must_reset = true`：即使发布中途失败，调用方也必须先完成硬件复位才能销毁本实例。
    pub fn publish(&mut self) -> Result<(), VirtioInitError> {
        self.must_reset = true;
        KernelHal::dma_barrier();
        self.driver.publish().map_err(Into::into)
    }

    /// 插入全内存屏障并向硬件状态寄存器写入 `0` 以发起设备复位请求。
    ///
    /// 注意：发起复位并不代表硬件 DMA 已立即停止，`must_reset` 仍保持 `true`，必须配合 [`Self::reset_complete`] 读回确认。
    pub fn request_reset(&mut self) {
        self.must_reset = true;
        KernelHal::dma_barrier();
        self.driver.request_reset();
    }

    /// 检查硬件状态寄存器是否已读回 `0`（复位完成）；若已完成则插入全内存屏障并清除 `must_reset` 标志，允许安全释放控制页。
    pub fn reset_complete(&mut self) -> bool {
        if !self.driver.reset_complete() {
            return false;
        }
        KernelHal::dma_barrier();
        self.must_reset = false;
        true
    }

    /// 查询当前实例在析构前是否仍需执行硬件复位确认（即 `must_reset` 标志状态）。
    pub fn requires_reset(&self) -> bool {
        self.must_reset
    }
}

impl Drop for VirtioDevice {
    /// 销毁块设备驱动并归还 `pdu` 与 `ring` 一致性 DMA 物理页。
    ///
    /// 仅允许在从未发布队列，或已通过 [`Self::reset_complete`] 确认硬件停止后调用；若 `must_reset` 仍为 `true` 则直接 panic 阻止释放仍在被 DMA 访问的物理页。
    fn drop(&mut self) {
        assert!(
            !self.must_reset,
            "VirtIO control DMA dropped before confirmed reset"
        );
        // SAFETY:
        // 1. 上方 `assert!(!self.must_reset)` 保证了硬件从未获得 `ring`/`pdu` 地址，或硬件已完成复位并执行了全内存屏障。
        // 2. 先析构 `driver` 以结束其持有的队列裸指针视图，再依次将 `pdu` 和 `ring` 物理页各释放一次，无重复释放或悬垂访问。
        unsafe {
            ManuallyDrop::drop(&mut self.driver);
            ManuallyDrop::drop(&mut self.pdu);
            ManuallyDrop::drop(&mut self.ring);
        }
    }
}

/// 从 PCI 配置空间安全读取指定 Capability 结构内偏移为 `field` 的 32 位双字，防止 `cap + field` 无符号溢出回绕到配置空间头部。
fn cap_word(
    inventory: &PciInventory,
    device: &PciDevice,
    cap: u8,
    field: u8,
) -> Result<u32, VirtioInitError> {
    let offset = cap
        .checked_add(field)
        .ok_or(ProbeError::Resource("PCI capability offset overflows"))?;
    Ok(inventory.read_config_word(device, offset)?)
}

/// 校验 VirtIO PCI Capability 指向的 BAR 索引、BAR 内偏移与长度是否合法，并将其换算为已映射的强序内核虚拟地址区间 [`PciRegion`]。
fn capability_region(
    device: &PciDevice,
    bar: u8,
    offset: u32,
    length: u32,
) -> Result<PciRegion, VirtioInitError> {
    let bar = device
        .bars
        .get(usize::from(bar))
        .and_then(Option::as_ref)
        .ok_or(ProbeError::Resource(
            "VirtIO capability references unassigned memory BAR",
        ))?;
    let offset = usize::try_from(offset)
        .map_err(|_| ProbeError::Resource("PCI capability offset is not addressable"))?;
    let len = usize::try_from(length)
        .map_err(|_| ProbeError::Resource("PCI capability length is not addressable"))?;
    let size = bar
        .physical
        .end
        .checked_sub(bar.physical.start)
        .ok_or(ProbeError::Resource("invalid PCI BAR physical extent"))?;
    let end = offset
        .checked_add(len)
        .ok_or(ProbeError::Resource("PCI capability extent overflows"))?;
    if len == 0 || size < end {
        return Err(ProbeError::Resource("PCI capability is outside assigned BAR").into());
    }
    let start = bar
        .physical
        .start
        .checked_add(offset)
        .ok_or(ProbeError::Resource("PCI capability address overflows"))?;
    let physical_end = start
        .checked_add(len)
        .ok_or(ProbeError::Resource("PCI capability end overflows"))?;
    let base = mmio_region_address(Region {
        start,
        end: physical_end,
    })?;
    Ok(PciRegion {
        base: NonNull::new(base as *mut u8)
            .ok_or(ProbeError::Resource("null PCI capability VA"))?,
        len,
    })
}

/// 遍历 `device` 在枚举阶段缓存的 PCI Capabilities，提取 VirtIO 协议所需的四类寄存器窗口（Common、Notify、ISR、Device）并组装 [`SelectedConstruct`]。
///
/// 若同一类型的 Capability 出现多次，遵循 VirtIO 规范保留配置链表中的第一个有效项。
fn selected_construct(
    inventory: &PciInventory,
    device: &PciDevice,
) -> Result<SelectedConstruct, VirtioInitError> {
    // Step 1: 重新读取 PCI 配置空间偏移 0x00 的 Vendor/Device ID，确认硬件身份与枚举库存一致且属于 VirtIO 厂商。
    let identity = inventory.read_config_word(device, PCI_IDENTITY)?;
    if identity as u16 != device.vendor_id
        || (identity >> u16::BITS) as u16 != device.device_id
        || device.vendor_id != VIRTIO_VENDOR_ID
    {
        return Err(ProbeError::Resource("PCI inventory identity changed").into());
    }

    // Step 2: 扫描缓存的 Vendor-Specific Capabilities (ID = 0x09)，校验结构长度并将 BAR 内偏移换算为虚拟地址窗口。
    let caps = device
        .capabilities
        .get(..device.capability_count)
        .ok_or(ProbeError::Resource(
            "PCI capability inventory count is invalid",
        ))?;
    let mut regions = [None; VIRTIO_PCI_CAP_REGION_COUNT];
    let mut notify_off_multiplier = 0;
    for cap in caps.iter().flatten() {
        if cap.id != PCI_CAP_ID_VNDR {
            continue;
        }
        let length = cap.private_header as u8;
        let kind = (cap.private_header >> u8::BITS) as u8;
        if !(VIRTIO_PCI_CAP_COMMON_CFG..=VIRTIO_PCI_CAP_DEVICE_CFG).contains(&kind) {
            continue;
        }
        if length < VIRTIO_PCI_CAP_BYTES
            || (kind == VIRTIO_PCI_CAP_NOTIFY_CFG && length < VIRTIO_PCI_NOTIFY_CAP_BYTES)
            || PCI_CONFIG_SPACE_BYTES < usize::from(cap.offset) + usize::from(length)
        {
            return Err(ProbeError::Resource("short VirtIO PCI capability").into());
        }
        let index = usize::from(kind - 1);
        if regions[index].is_some() {
            continue;
        }
        let bar = cap_word(inventory, device, cap.offset, VIRTIO_PCI_CAP_BAR)? as u8;
        let offset = cap_word(inventory, device, cap.offset, VIRTIO_PCI_CAP_OFFSET)?;
        let region_length = cap_word(inventory, device, cap.offset, VIRTIO_PCI_CAP_LENGTH)?;
        regions[index] = Some(capability_region(device, bar, offset, region_length)?);
        if kind == VIRTIO_PCI_CAP_NOTIFY_CFG {
            notify_off_multiplier =
                cap_word(inventory, device, cap.offset, VIRTIO_PCI_NOTIFY_CAP_MULT)?;
        }
    }

    // Step 3: 组装 Modern PCI 传输构造参数（Common、Notify、ISR 为必需窗口，Device 特定配置区为可选窗口）。
    Ok(SelectedConstruct {
        device_function: DeviceFunction {
            bus: device.bus,
            device: device.device,
            function: device.function,
        },
        vendor_id: device.vendor_id,
        device_id: device.device_id,
        common_cfg: regions[0].ok_or(VirtioPciError::MissingCommonConfig)?,
        notify_cfg: regions[1].ok_or(VirtioPciError::MissingNotifyConfig)?,
        notify_off_multiplier,
        isr_cfg: regions[2].ok_or(VirtioPciError::MissingIsrConfig)?,
        device_cfg: regions[3],
    })
}
