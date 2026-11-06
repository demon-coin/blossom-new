//! 物理块设备的有界扇区窗口视图（整盘或分区）。
//!
//! # 模块职责与架构定位
//! `BlockView` 位于上层消费者（`storage` 分区表发现、文件系统挂载 `FsClaim`、元数据缓存）与底层物理块设备
//! [`DiskRef`] 之间。它在同一个物理 `Disk` 实例之上裁剪出半开扇区区间 `[start_sector, start_sector + sectors)`，
//! 将整盘与分区统一为同一套容量查询、相对偏移换算与边界校验接口，而无需为每个分区重复分配硬件请求槽或工作队列。
//!
//! # 物理磁盘与视图地址映射布局
//! ```text
//!   物理磁盘 Disk (0 .. capacity_sectors)
//!   ┌──────────────┬─────────────────────────────┬──────────────┐
//!   │  前置扇区    │      BlockView 窗口         │  后置扇区    │
//!   └──────────────┴─────────────────────────────┴──────────────┘
//!   0              ▲                             ▲              capacity_sectors
//!                  │                             │
//!             start_sector            start_sector + sectors
//!
//!   视图内相对字节 [offset, offset + len)
//!     ──► 物理绝对字节 = start_sector * 512 + offset
//! ```
//!
//! # 并发与不可变契约
//! `BlockView` 的所有字段在构造时一次性校验并固化，运行期完全只读且不持有独立锁；
//! 克隆 `BlockView` 仅增加底层 `DiskRef`（`Pin<Arc<Disk>>`）的强引用计数，实际队列并发由 `Disk` 内部锁保护。

use crate::block::{Bio, BlkOperation, BlockError, DiskRef, QueueLimits, SECTOR_BYTES};

/// 绑定到特定物理磁盘实例的有界连续扇区窗口（可表示整盘或单个分区）。
#[derive(Clone)]
pub struct BlockView {
    /// 承载该视图的底层物理块设备强引用，所有实际 I/O 提交、刷盘与停机均委托给该实例。
    disk: DiskRef,
    /// 视图起点相对于物理整盘起始位置的扇区偏移量（以 512 字节 `SECTOR_BYTES` 为单位）。
    start_sector: u64,
    /// 视图覆盖的连续扇区总数（以 512 字节 `SECTOR_BYTES` 为单位，非零且不越过物理盘容量）。
    sectors: u64,
}

impl BlockView {
    /// 构造覆盖整块物理磁盘全部容量的视图（起始扇区为 0，扇区数等于设备物理容量）。
    pub fn whole(disk: DiskRef) -> Self {
        let sectors = disk.limits().capacity_sectors;
        Self {
            disk,
            start_sector: 0,
            sectors,
        }
    }

    /// 构造指定扇区范围的子视图（如磁盘分区），校验区间非空、不越出物理介质容量且起点与长度均对齐到设备逻辑块大小。
    pub fn new(disk: DiskRef, start_sector: u64, sectors: u64) -> Result<Self, BlockError> {
        let end = start_sector
            .checked_add(sectors)
            .ok_or(BlockError::InvalidRange)?;
        let logical = u64::from(disk.limits().logical_block_size) / SECTOR_BYTES;
        if sectors == 0
            || disk.limits().capacity_sectors < end
            || start_sector % logical != 0
            || sectors % logical != 0
        {
            return Err(BlockError::InvalidRange);
        }
        Ok(Self {
            disk,
            start_sector,
            sectors,
        })
    }

    /// 返回底层物理块设备引用；刷盘（`Flush`）与设备停机操作直接作用于该物理磁盘。
    pub fn disk(&self) -> &DiskRef {
        &self.disk
    }

    /// 返回本视图在物理整盘上的起始扇区号（单位：512 字节扇区）。
    pub fn start_sector(&self) -> u64 {
        self.start_sector
    }

    /// 返回本视图包含的 512 字节扇区总数。
    pub fn sectors(&self) -> u64 {
        self.sectors
    }

    /// 返回适配到本视图的请求队列限额：将容量替换为视图扇区数，其余硬件分段、对齐与深度限额沿用底层物理磁盘配置。
    pub fn limits(&self) -> QueueLimits {
        QueueLimits {
            capacity_sectors: self.sectors,
            ..self.disk.limits()
        }
    }

    /// 判断两个视图是否指向同一个物理 `Disk` 实例且覆盖完全相同的起始扇区与长度（用于挂载冲突与身份核对）。
    pub fn same_range(&self, other: &Self) -> bool {
        core::ptr::eq(&**self.disk(), &**other.disk())
            && self.start_sector == other.start_sector
            && self.sectors == other.sectors
    }

    /// 校验视图内的相对字节区间 `[offset, offset + len)` 未越过视图容量上限，并将其换算为物理整盘上的绝对起始字节偏移。
    pub fn absolute_byte(&self, offset: u64, len: usize) -> Result<u64, BlockError> {
        let end = offset
            .checked_add(len as u64)
            .ok_or(BlockError::InvalidRange)?;
        let capacity = self
            .sectors
            .checked_mul(SECTOR_BYTES)
            .ok_or(BlockError::InvalidRange)?;
        if capacity < end {
            return Err(BlockError::InvalidRange);
        }
        self.start_sector
            .checked_mul(SECTOR_BYTES)
            .and_then(|base| base.checked_add(offset))
            .ok_or(BlockError::InvalidRange)
    }

    /// 检查物理整盘上的绝对字节区间 `[start, start + len)` 是否完全落在本视图边界内，供文件系统提交与元数据缓存窗口做越界拦截。
    pub(crate) fn contains_bytes(&self, start: u64, len: usize) -> bool {
        let base = self.start_sector * SECTOR_BYTES;
        base <= start
            && start
                .checked_add(len as u64)
                .is_some_and(|end| end <= base + self.sectors * SECTOR_BYTES)
    }

    /// 在普通可睡眠任务上下文中，从视图相对字节偏移 `offset` 处同步读取填满 `dst` 切片的数据。
    pub fn read_exact_at(&self, offset: u64, dst: &mut [u8]) -> Result<(), BlockError> {
        self.disk
            .read_exact_at(self.absolute_byte(offset, dst.len())?, dst)
    }

    /// 在普通可睡眠任务上下文中向底层物理磁盘提交已携带物理绝对扇区地址的 `Bio`；
    /// 由于 `Bio` 在构造时已填入物理 LBA，此处仅校验非 `Flush` 请求的范围是否落在本视图内，越界时直接唤醒积压队列并以 `InvalidRange` 拒绝该请求，不会二次叠加分区偏移。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "RAM 范围测试验证裸 Bio 的分区边界拒绝和原父完成")
    )]
    pub(crate) fn submit_bio(&self, bio: Bio) {
        if bio.inner.operation() != BlkOperation::Flush
            && !self.contains_bytes(bio.inner.sector() * SECTOR_BYTES, bio.inner.len())
        {
            self.disk.unplug();
            crate::block::bio::BioInner::reject(&bio.inner, BlockError::InvalidRange);
            return;
        }
        self.disk.submit_bio(bio);
    }
}
