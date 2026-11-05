//! 同一物理磁盘的有界扇区视图，所有相对地址只在提交边界换算。

use crate::block::{Bio, BlkOperation, BlockError, DiskRef, QueueLimits, SECTOR_BYTES};

/// 分区只保存范围和原物理设备拥有者，不创建另一条硬件队列。
#[derive(Clone)]
pub struct BlockView {
    disk: DiskRef,
    start_sector: u64,
    sectors: u64,
}

impl BlockView {
    /// 建立整盘视图，容量取自已协商的同一物理设备。
    pub fn whole(disk: DiskRef) -> Self {
        let sectors = disk.limits().capacity_sectors;
        Self {
            disk,
            start_sector: 0,
            sectors,
        }
    }

    /// 校验非空半开范围及逻辑块对齐，不允许视图超出物理介质。
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

    /// 借用唯一物理设备，停止和 Flush 仍属于该设备。
    pub fn disk(&self) -> &DiskRef {
        &self.disk
    }

    /// 返回相对物理整盘的起始 sector，范围只由正式构造器建立。
    pub fn start_sector(&self) -> u64 {
        self.start_sector
    }

    /// 返回已经检查的 512B sector 数，调用者不能修改视图边界。
    pub fn sectors(&self) -> u64 {
        self.sectors
    }

    /// 返回视图容量，其他请求限额保持物理设备的真实值。
    pub fn limits(&self) -> QueueLimits {
        QueueLimits {
            capacity_sectors: self.sectors,
            ..self.disk.limits()
        }
    }

    /// 身份同时比较物理对象和准确范围，不以相同设备号替代范围。
    pub fn same_range(&self, other: &Self) -> bool {
        core::ptr::eq(&**self.disk(), &**other.disk())
            && self.start_sector == other.start_sector
            && self.sectors == other.sectors
    }

    /// 将一个已检查的视图内字节范围换算成绝对物理位置。
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

    /// 校验绝对半开字节范围属于本视图，用于 FS 授权与物理缓存窗口。
    pub(crate) fn contains_bytes(&self, start: u64, len: usize) -> bool {
        let base = self.start_sector * SECTOR_BYTES;
        base <= start
            && start
                .checked_add(len as u64)
                .is_some_and(|end| end <= base + self.sectors * SECTOR_BYTES)
    }

    /// 从视图内读取完整字节，原请求仍由同一 Disk 完成。
    pub fn read_exact_at(&self, offset: u64, dst: &mut [u8]) -> Result<(), BlockError> {
        self.disk
            .read_exact_at(self.absolute_byte(offset, dst.len())?, dst)
    }

    /// 裸请求已经保存绝对物理 LBA，提交时只检查本范围，不能再次增加分区偏移。
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
