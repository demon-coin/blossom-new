//! 块设备实例、逻辑 Bio 与固定硬件请求各自持有唯一责任。

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

/// 协议 sector 与设备 logical block、文件系统块大小彼此独立。
pub const SECTOR_BYTES: u64 = 512;
pub(super) const MAX_BATCH: usize = 8;

/// 单命令后端的软件完成槽数，独立于硬件通道深度和 Bio 批量大小。
pub(crate) const NATIVE_TAG_COUNT: u16 = 8;

/// 块请求语义不携带具体设备协议的操作码。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlkOperation {
    Read,
    Write,
    Flush,
}

/// 卷能力与挂载视图各自保存访问模式，设备只读能力仍由 QueueLimits 表达。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AccessMode {
    ReadOnly,
    ReadWrite,
}

/// 普通消费者的错误；额度暂满由核心等待，不暴露成忙重试协议。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlockError {
    NoMemory,
    InvalidRange,
    Misaligned,
    ReadOnly,
    Unsupported,
    Corrupt,
    Checksum,
    NotReady,
    QueueStopped,
    Busy,
    Io,
    ShortIo,
    DeviceFault,
    StopUnconfirmed,
    Protocol(virtio_drivers::Error),
}

impl From<virtio_drivers::Error> for BlockError {
    /// 保留实际协议原因，将准备内存失败与临时 descriptor 背压分开。
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
    /// 将原页访问准备错误统一转换为块层语义，保留内存、范围、对齐与设备同步的区别。
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

/// 完成只报告连续成功前缀，后部反序成功不能跨越前面的失败。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IoResult {
    pub completed: usize,
    pub error: Option<BlockError>,
}

/// 完成回调只借已经收权的原 memory；重型回调使用预建 slot Work。
#[derive(Clone, Copy)]
pub enum EndIo {
    Inline(fn(&IoResult, &[MappedPages])),
    Task(fn(&IoResult, &[MappedPages])),
}

/// 物理请求限额区分硬件深度与可睡完成所需的软件槽数。
#[derive(Clone, Copy, Debug)]
pub struct QueueLimits {
    pub capacity_sectors: u64,
    pub max_bytes: usize,
    pub max_segments: usize,
    pub max_segment_bytes: u32,
    pub logical_block_size: u32,
    pub dma_boundary_mask: u64,
    pub readonly: bool,
    pub flush: bool,
    pub queue_depth: u16,
    pub tag_count: u16,
}

impl QueueLimits {
    /// 唯一逻辑入口校验；单 child 段数和边界由最终 SG 拆分负责。
    pub fn validate(
        &self,
        operation: BlkOperation,
        sector: u64,
        total_bytes: usize,
        segments: impl IntoIterator<Item = PayloadRange>,
        attributes: u32,
    ) -> Result<(), BlockError> {
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
