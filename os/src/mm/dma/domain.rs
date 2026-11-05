//! 设备可达窗口、地址转换与真实 cache 维护；不拥有或分配另一份页。

use crate::arch;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::dma::DmaError;
use virtio_drivers::{BufferDirection, DmaAddr};

/// 同步所覆盖的设备读写方向；双向用于设备和 CPU 交替更新的控制内存。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmaDirection {
    ToDevice,
    FromDevice,
    Bidirectional,
}

impl From<BufferDirection> for DmaDirection {
    /// 将现有请求方向转换为 cache 同步方向，不改变请求自身的访问权限。
    fn from(direction: BufferDirection) -> Self {
        match direction {
            BufferDirection::DriverToDevice => Self::ToDevice,
            BufferDirection::DeviceToDriver => Self::FromDevice,
        }
    }
}

/// 同一设备的一段线性 DMA 窗口；物理地址与总线地址分别保存。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DmaDomain {
    mask: u64,
    physical_start: usize,
    physical_end: usize,
    device_start: u64,
    cache_line: usize,
    coherent: bool,
}

impl DmaDomain {
    /// 建立已由平台资源确认的窗口，拒绝无效 mask、行大小和地址溢出。
    pub fn new(
        mask: u64,
        physical_start: usize,
        physical_end: usize,
        device_start: u64,
        cache_line: usize,
        coherent: bool,
    ) -> Result<Self, DmaError> {
        if mask == 0
            || mask & mask.wrapping_add(1) != 0
            || physical_end <= physical_start
            || !cache_line.is_power_of_two()
            || PAGE_SIZE < cache_line
            || mask < device_start
        {
            return Err(DmaError::InvalidDomain);
        }

        let domain = Self {
            mask,
            physical_start,
            physical_end,
            device_start,
            cache_line,
            coherent,
        };
        device_start
            .checked_add((physical_end - physical_start - 1) as u64)
            .ok_or(DmaError::AddressLimit)?;
        Ok(domain)
    }

    /// 返回 QEMU VirtIO 的真实 coherent 恒等地址域，不执行 cache 维护。
    pub const fn qemu_coherent() -> Self {
        Self {
            mask: u64::MAX,
            physical_start: 0,
            physical_end: usize::MAX,
            device_start: 0,
            cache_line: 1,
            coherent: true,
        }
    }

    /// 验证完整设备范围；noncoherent 流必须独占完整 cacheline，不能扩张到邻居。
    pub fn map(self, physical: usize, len: usize) -> Result<DmaAddr, DmaError> {
        let end = physical.checked_add(len).ok_or(DmaError::AddressLimit)?;
        if len == 0 || physical < self.physical_start || self.physical_end < end {
            return Err(DmaError::AddressLimit);
        }
        if !self.coherent && (physical % self.cache_line != 0 || len % self.cache_line != 0) {
            return Err(DmaError::Alignment);
        }

        let address = self
            .device_start
            .checked_add((physical - self.physical_start) as u64)
            .ok_or(DmaError::AddressLimit)?;
        let last = address
            .checked_add((len - 1) as u64)
            .ok_or(DmaError::AddressLimit)?;
        if self.mask < last {
            return Err(DmaError::AddressLimit);
        }
        Ok(DmaAddr::new(address))
    }

    /// 返回 mask 与转换窗口共同允许的 CPU 物理半开范围，供原 buddy 一次选择。
    pub(crate) fn allocation_window(self) -> (usize, usize) {
        let span = self.physical_end - self.physical_start;
        let reachable_last = self.mask - self.device_start;
        let length = if (span - 1) as u64 <= reachable_last {
            span
        } else {
            reachable_last as usize + 1
        };
        (self.physical_start, self.physical_start + length)
    }

    /// 在发表设备访问之前维护隔离范围并排序；失败时设备尚未获权。
    ///
    /// # Safety
    /// 调用者持有整个范围及永久 RAM 直映，FromDevice/Bidirectional 的冲突 CPU
    /// 访问已结束；ToDevice 已初始化且共享 WB 只允许原始字节 store，不借出切片。
    /// 范围独占完整 cacheline；设备尚未开始访问，不要求 ToDevice 提供内容快照。
    pub unsafe fn sync_for_device(
        self,
        physical: usize,
        len: usize,
        _direction: DmaDirection,
    ) -> Result<(), DmaError> {
        self.map(physical, len)?;
        if !self.coherent {
            arch::dma_cache_flush(physical, len).map_err(DmaError::CacheMaintenance)?;
        }
        arch::dma_write_barrier();
        Ok(())
    }

    /// 确认设备停止后维护设备写入范围；只读设备完成不得 invalidate 已再脏的 WB。
    ///
    /// # Safety
    /// 调用者已确认该范围的全部硬件访问结束，并持有原拥有者；
    /// FromDevice/Bidirectional 期间没有 CPU 写入，范围独占完整 cacheline。
    pub unsafe fn sync_for_cpu(
        self,
        physical: usize,
        len: usize,
        direction: DmaDirection,
    ) -> Result<(), DmaError> {
        self.map(physical, len)?;
        arch::dma_read_barrier();
        if !self.coherent && direction != DmaDirection::ToDevice {
            arch::dma_cache_flush(physical, len).map_err(DmaError::CacheMaintenance)?;
        }
        Ok(())
    }
}
