//! 设备 DMA 寻址窗口（`DmaDomain`）、总线地址线性转换与硬件 Cache 一致性维护。
//!
//! # 模块职责与架构定位
//! 本模块位于 [`super::CoherentDma`] / [`super::MappedPages`] 与底层体系结构 Cache 原语（`arch::dma_cache_flush`）之间：
//! - **上游调用者**：`CoherentDma`（控制区分配与同步）与 `MappedPages`（流式载荷 DMA 映射与同步），以及平台设备树/板级初始化代码。
//! - **下游依赖**：`arch::dma_cache_flush`、`arch::dma_write_barrier` 与 `arch::dma_read_barrier`。
//!
//! # 核心设计与地址转换模型
//! 1. **线性 DMA 窗口映射**：
//!    部分 SoC 外设无法访问全部 CPU 物理地址空间，或者外设看到的设备总线地址（`DmaAddr`）相对于 CPU 物理地址（PA）存在固定偏移与位宽掩码限制：
//!    ```text
//!    CPU 物理地址空间 (PA):
//!    +------------------+------------------------------------+------------------+
//!                       |       [physical_start .. end)      |
//!                       +-----------------+------------------+
//!                                         |
//!                                         |  device_addr = device_start + (pa - physical_start)
//!                                         |  且必须满足: last_byte <= mask
//!                                         v
//!    外设总线地址空间 (DmaAddr):
//!                       +------------------------------------+
//!                       |  [device_start ..<= mask]          |
//!                       +------------------------------------+
//!    ```
//! 2. **非一致性（Non-coherent）DMA 的 Cache 行隔离契约**：
//!    在不支持硬件 Cache 一致性的开发板外设上，DMA 缓冲区必须按 `cache_line` 字节边界对齐且长度为其整数倍。
//!    若与相邻的普通内核变量共享同一条 Cache 行，CPU 对相邻变量的读写会在 DMA 传输期间重新装入陈旧或脏 Cache 行，导致 DMA 数据损坏。
//!
//! # 锁获取顺序与并发契约
//! - `DmaDomain` 是纯值类型（`Copy`），不含任何锁或可变状态。
//! - 调用 `sync_for_device` 与 `sync_for_cpu` 的并发互斥与物理页保活由上层 `CoherentDma` 或 `MappedPages` 拥有者负责。

use crate::arch;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::dma::DmaError;
use virtio_drivers::{BufferDirection, DmaAddr};

/// DMA 传输与 Cache 一致性同步的数据流向。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmaDirection {
    /// 数据从内存流向外设（CPU 写入，外设只读）。
    ToDevice,
    /// 数据从外设流向内存（外设写入，CPU 读取）。
    FromDevice,
    /// CPU 与外设双向读写（常用于共享控制结构）。
    Bidirectional,
}

impl From<BufferDirection> for DmaDirection {
    /// 将 `virtio_drivers` 的缓冲区传输方向转换为内核 `DmaDirection`。
    fn from(direction: BufferDirection) -> Self {
        match direction {
            BufferDirection::DriverToDevice => Self::ToDevice,
            BufferDirection::DeviceToDriver => Self::FromDevice,
        }
    }
}

/// 描述单个外设可达的线性 DMA 地址窗口、寻址掩码及 Cache 一致性属性。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DmaDomain {
    /// 外设支持的最大总线地址位掩码（形如 $2^N - 1$，例如 32 位 DMA 为 `0xffff_ffff`）。
    mask: u64,
    /// 该窗口在 CPU 物理地址空间中的起始字节地址（包含）。
    physical_start: usize,
    /// 该窗口在 CPU 物理地址空间中的结束字节地址（不包含）。
    physical_end: usize,
    /// `physical_start` 映射到外设总线侧时的起始设备地址。
    device_start: u64,
    /// 外设 DMA 要求对齐的硬件 Cache 行字节大小（必须为 2 的幂且不超过 `PAGE_SIZE`）。
    cache_line: usize,
    /// 硬件总线是否自动维护 CPU Cache 与外设 DMA 之间的一致性。
    coherent: bool,
}

impl DmaDomain {
    /// 根据平台硬件资源参数构造并校验一个新的 `DmaDomain`。
    ///
    /// 校验内容包括：`mask` 必须是非零的连续低位掩码（$2^N - 1$）、物理区间非空、`cache_line` 为不超过页大小的 2 的幂，
    /// 以及窗口转换后的设备地址不会发生 64 位无符号溢出。
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

    /// 返回适用于 QEMU VirtIO 设备的全地址空间硬件一致性（Coherent）恒等映射域。
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

    /// 校验 CPU 物理区间 `[physical, physical + len)` 满足窗口范围、寻址掩码与 Cache 行对齐要求，并返回起始设备总线地址。
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

    /// 计算同时满足物理窗口 `[physical_start, physical_end)` 与设备 `mask` 限制的 CPU 物理地址半开区间，供伙伴分配器定向分配。
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

    /// 在启动外设 DMA 传输前，校验区间合法性并执行必要的 Cache 刷新与写内存屏障。
    ///
    /// - 非一致性域（`!self.coherent`）下，会刷回（Clean & Invalidate）`[physical, physical + len)` 覆盖的 Cache 行，确保内存中的内容最新且后续外设写入不会被脏 Cache 行覆盖。
    /// - 最后执行 `arch::dma_write_barrier()`，确保描述符或数据写入在通知外设前完成。
    ///
    /// # Safety
    /// 1. 调用方必须保活 `[physical, physical + len)` 对应的物理内存，且已停止任何冲突的 CPU 访问；
    /// 2. 当方向为 `ToDevice` 或 `Bidirectional` 时，区间内的字节必须已完成初始化；
    /// 3. 非一致性域下，该区间必须独占完整的 Cache 行，不得与并发访问的其他变量共享 Cache 行。
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

    /// 在确认外设 DMA 传输已停止后，执行读内存屏障并在需要时维护 CPU Cache。
    ///
    /// - 当方向为 `FromDevice` 或 `Bidirectional` 且处于非一致性域时，调用 `arch::dma_cache_flush` 清理 CPU 在 DMA 期间可能投机预取到的陈旧 Cache 行。
    /// - 当方向为只读的 `ToDevice` 时，跳过 Cache 维护，避免将回写期间用户态并发重新写脏的页缓存数据意外丢弃（Invalidate）。
    ///
    /// # Safety
    /// 1. 调用方必须已确认外设对该物理区间的硬件 DMA 访问彻底结束，并继续保活底层物理页；
    /// 2. 在 `FromDevice` 或 `Bidirectional` 传输期间没有发生 CPU 写入，且非一致性域下区间独占完整的 Cache 行。
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
