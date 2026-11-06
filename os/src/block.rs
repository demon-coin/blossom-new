//! 块设备抽象与通用块 I/O 核心层。
//!
//! # 模块职责与架构定位
//! 本模块位于上层消费者（文件系统、页面/元数据缓存、分区发现、全局设备表 `storage`）与底层块传输后端
//! （VirtIO-Blk、SD/MMC、AHCI、内存盘）之间，定义设备无关的块 I/O 请求协议、容量/对齐限额、错误分类与完成回调模型。
//!
//! # 核心对象分层与请求流转
//! ```text
//!   上层消费者 (FS / Cache / Partition)
//!          │  相对偏移换算为物理扇区
//!          ▼
//!   BlockView (整盘或分区的有界扇区窗口)
//!          │
//!          ▼
//!   Bio (逻辑块请求：持有物理页映射 BioMemory，聚合连续成功前缀)
//!          │  按队列限额拆分为 1..N 个硬件子请求 (RequestPart)
//!          ▼
//!   Disk + SlotStorage (物理块设备实例：管理固定数量请求槽、中断与停机)
//!          │
//!          ▼
//!   Backend (VirtIO / SDMMC / AHCI / RAM 传输层)
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本文件仅定义无锁值类型与纯校验逻辑；子模块的并发锁层级遵循以下顺序（由外向内）：
//! 1. `Disk.control`（`SleepMutex`）：串行化设备启动与停机控制流；
//! 2. `Disk.queue`（`SpinLock`）：保护硬件队列阶段、槽位状态与挂载占用链表；
//! 3. `BioInner.state`（`SpinLock`）：保护单个 `Bio` 的完成计数、物理页容器与首个错误位置。
//!
//! 持自旋锁期间禁止睡眠，也不得反向获取外层锁。

pub(crate) mod backend;
mod bio;
mod disk;
#[cfg(feature = "ktest")]
mod ktest;
#[cfg(feature = "ktest")]
pub(crate) mod ktest_irq;
#[cfg(feature = "ktest")]
mod ktest_queue;
#[cfg(feature = "ktest")]
mod ktest_quota;
#[cfg(feature = "ktest")]
mod ktest_ram;
pub mod partition;
mod reserve;
mod stop;
mod submit;
#[cfg(feature = "ktest")]
pub(crate) mod tests;
mod view;

pub use backend::DiskIdentity;
pub use bio::{Bio, BioCompletion};
pub use disk::{Disk, DiskRef};
pub(crate) use reserve::{
    BioAllocator, BioLane, BioLease, BioMemory, BioReserve, MetadataIoContext,
};
pub use stop::FsClaim;
pub use view::BlockView;

use crate::mm::dma::{DmaError, PayloadRange};
use core::convert::TryFrom;
use virtio_drivers::BufferDirection;

use crate::mm::dma::MappedPages;

/// 块层通用寻址扇区大小（固定为 512 字节），独立于设备硬件逻辑块大小和文件系统块大小。
pub const SECTOR_BYTES: u64 = 512;
/// 单次批量提交或单个硬件请求槽最多聚合的逻辑 `Bio` 数量上限。
pub(super) const MAX_BATCH: usize = 8;

/// 单通道原生控制器（SD/MMC、AHCI）及内存盘预分配的软件请求槽数量，独立于硬件命令深度和 `MAX_BATCH`。
pub(crate) const NATIVE_TAG_COUNT: u16 = 8;

/// 设备无关的块 I/O 操作类型。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlkOperation {
    /// 从块设备读取数据到内存物理页（DMA 方向：`DeviceToDriver`）。
    Read,
    /// 将内存物理页中的数据写入块设备（DMA 方向：`DriverToDevice`）。
    Write,
    /// 刷盘屏障命令，要求设备将易失性写缓存落盘，不携带任何数据负载与扇区范围。
    Flush,
}

/// 文件系统挂载占用（`FsClaim`）对块视图声明的访问权限模式；硬件本身的只读属性由 [`QueueLimits::readonly`] 表示。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AccessMode {
    /// 只读访问模式，禁止通过该挂载占用提交写请求或刷盘请求。
    ReadOnly,
    /// 读写访问模式，允许在视图范围内提交读写与刷盘请求。
    ReadWrite,
}

/// 块设备层向调用方返回的统一错误类型；请求槽或预留池暂时耗尽会在块层内部睡眠等待，不会返回忙重试错误。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlockError {
    /// 内核控制结构或 DMA 映射内存分配失败。
    NoMemory,
    /// 请求的扇区/字节范围越界、算术溢出、长度为零或与操作语义冲突。
    InvalidRange,
    /// 起始字节偏移、传输总长度或物理地址未按设备要求的逻辑块或 DMA 边界对齐。
    Misaligned,
    /// 尝试向只读设备或只读挂载视图提交写操作。
    ReadOnly,
    /// 设备不支持请求的操作（如不支持 `Flush`）、非零属性标志或超出设备枚举上限。
    Unsupported,
    /// 分区表或磁盘元数据结构损坏。
    Corrupt,
    /// 分区表或元数据校验和（CRC）不匹配。
    Checksum,
    /// 设备尚未完成初始化启动，或启动参数指定的根/附加块设备不存在。
    NotReady,
    /// 块设备队列正在排空停机或已停止，拒绝接纳新请求。
    QueueStopped,
    /// 资源正被占用（如仍有活跃挂载时尝试停机、物理页已处于在途 I/O 状态或中断槽位已满）。
    Busy,
    /// 底层存储介质或控制器报告一般 I/O 传输失败。
    Io,
    /// 设备实际完成的字节数少于请求长度（短读或短写）。
    ShortIo,
    /// 控制器发生硬件故障、PCI 命令被拒绝或 DMA 缓存一致性维护失败。
    DeviceFault,
    /// 设备重置超时导致无法确认硬件 DMA 已停止，设备被永久隔离以防止内存踩踏。
    StopUnconfirmed,
    /// VirtIO 传输层返回的其它协议级错误。
    Protocol(virtio_drivers::Error),
}

impl From<virtio_drivers::Error> for BlockError {
    /// 将 VirtIO 驱动错误转换为块层错误，保留内存不足、不支持与介质 I/O 错误的具体分类。
    fn from(error: virtio_drivers::Error) -> Self {
        match error {
            virtio_drivers::Error::NoMemory => Self::NoMemory,
            virtio_drivers::Error::Unsupported => Self::Unsupported,
            virtio_drivers::Error::IoError => Self::Io,
            other => Self::Protocol(other),
        }
    }
}

impl From<DmaError> for BlockError {
    /// 将物理页 DMA 映射与缓存同步错误转换为块层错误，区分内存不足、地址越界、对齐错误与硬件故障。
    fn from(error: DmaError) -> Self {
        match error {
            DmaError::NoMemory => Self::NoMemory,
            DmaError::InvalidRange | DmaError::AddressLimit | DmaError::InvalidDomain => {
                Self::InvalidRange
            }
            DmaError::Alignment => Self::Misaligned,
            DmaError::CacheMaintenance(_) => Self::DeviceFault,
        }
    }
}

/// 逻辑块请求（`Bio`）的最终完成结果。
///
/// 当一个 `Bio` 被拆分为多个硬件子请求并发执行时，只统计从起始位置起连续成功的字节前缀；
/// 若前部子请求失败，即使后部子请求乱序成功，也不会跨过失败点计入 `completed`。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IoResult {
    /// 从请求起始位置起连续成功传输的字节数。
    pub completed: usize,
    /// 首个失败位置对应的块层错误；全部字节均成功完成时为 `None`。
    pub error: Option<BlockError>,
}

/// `Bio` 完成时的异步通知回调类型；回调执行时物理页的 DMA 访问权已从设备收回至 CPU，并以只读切片借出。
#[derive(Clone, Copy)]
pub enum EndIo {
    /// 轻量级内联回调，在收回物理页 DMA 访问权后直接于当前完成上下文（可能处于中断或不可睡眠路径）中执行，严禁睡眠。
    #[expect(
        dead_code,
        reason = "完成协议允许不睡眠轻回调在原页收权后 Inline 执行；当前生产者使用无回调或 Task 回调"
    )]
    Inline(fn(&IoResult, &[MappedPages])),
    /// 任务上下文回调，派发到预分配的工作队列（`Work`）中由独立内核线程执行，允许在回调内睡眠阻塞。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "块请求测试用可阻塞回调验证 slot Work 完成尾部和停止责任"
        )
    )]
    Task(fn(&IoResult, &[MappedPages])),
}

/// 物理块设备或逻辑分区视图的容量与硬件传输限额参数。
#[derive(Clone, Copy, Debug)]
pub struct QueueLimits {
    /// 设备或分区视图的总容量，单位为 512 字节扇区（`SECTOR_BYTES`）。
    pub capacity_sectors: u64,
    /// 单个硬件请求最多可传输的字节数上限。
    pub max_bytes: usize,
    /// 单个硬件请求的散聚表（Scatter-Gather List）最多包含的物理连续段数。
    pub max_segments: usize,
    /// 单个散聚段（SG segment）的最大长度，单位为字节。
    pub max_segment_bytes: u32,
    /// 介质支持的最小可寻址逻辑块大小，单位为字节（通常为 512 或 4096），读写偏移与长度必须对齐到此值。
    pub logical_block_size: u32,
    /// 单个 DMA 段禁止跨越的物理地址边界掩码（例如 `0xffff` 表示禁止跨越 64KiB 边界，全 1 表示无限制）。
    pub dma_boundary_mask: u64,
    /// 设备硬件是否处于只读状态。
    pub readonly: bool,
    /// 设备后端是否支持 `BlkOperation::Flush` 刷盘命令。
    pub flush: bool,
    /// 硬件传输通道可同时在途的命令队列深度。
    pub queue_depth: u16,
    /// 块层为该设备预分配的软件请求槽（`SlotStorage`）总数。
    pub tag_count: u16,
}

impl QueueLimits {
    /// 在普通任务构造 `Bio` 或直接发起请求时校验操作权限、散聚段方向与长度总和、逻辑块对齐及容量边界；
    /// 单个硬件子请求的散聚段数量上限与 DMA 边界掩码不在本函数拒绝，而由后续提交阶段的请求拆分器处理。
    pub fn validate(
        &self,
        operation: BlkOperation,
        sector: u64,
        total_bytes: usize,
        segments: impl IntoIterator<Item = PayloadRange>,
        attributes: u32,
    ) -> Result<(), BlockError> {
        // Step 1: 校验扩展属性位并根据操作类型确定预期的 DMA 传输方向及只读/Flush 能力限制。
        if attributes != 0 {
            return Err(BlockError::Unsupported);
        }

        let direction = match operation {
            BlkOperation::Read => BufferDirection::DeviceToDriver,
            BlkOperation::Write => {
                if self.readonly {
                    return Err(BlockError::ReadOnly);
                }
                BufferDirection::DriverToDevice
            }
            BlkOperation::Flush => {
                if !self.flush {
                    return Err(BlockError::Unsupported);
                }
                if sector != 0 || total_bytes != 0 {
                    return Err(BlockError::InvalidRange);
                }
                BufferDirection::DriverToDevice
            }
        };

        // Step 2: 逐段检查散聚段非空、传输方向一致且物理地址无算术溢出，同时累加核对总字节数。
        let mut sum = 0_usize;
        for segment in segments {
            if operation == BlkOperation::Flush
                || segment.length == 0
                || segment.direction != direction
                || segment
                    .physical
                    .checked_add(segment.length as usize - 1)
                    .is_none()
            {
                return Err(BlockError::InvalidRange);
            }
            sum = sum
                .checked_add(segment.length as usize)
                .ok_or(BlockError::InvalidRange)?;
        }
        if sum != total_bytes {
            return Err(BlockError::InvalidRange);
        }

        // Step 3: Flush 请求已在前面确认零扇区与零长度，可直接放行；数据读写请求继续校验逻辑块对齐与容量上限。
        if operation == BlkOperation::Flush {
            return Ok(());
        }
        if total_bytes == 0 {
            return Err(BlockError::InvalidRange);
        }

        let offset = sector
            .checked_mul(SECTOR_BYTES)
            .ok_or(BlockError::InvalidRange)?;
        let bytes = u64::try_from(total_bytes).map_err(|_| BlockError::InvalidRange)?;
        let logical = u64::from(self.logical_block_size);
        if offset % logical != 0 || bytes % logical != 0 {
            return Err(BlockError::Misaligned);
        }
        let end_sector = sector
            .checked_add(bytes / SECTOR_BYTES)
            .ok_or(BlockError::InvalidRange)?;
        if self.capacity_sectors < end_sector {
            return Err(BlockError::InvalidRange);
        }
        Ok(())
    }
}
