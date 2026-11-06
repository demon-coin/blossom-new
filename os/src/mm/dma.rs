//! DMA 缓冲区管理、一致性控制区（`CoherentDma`）与流式数据载荷（`MappedPages`）生命周期状态机。
//!
//! # 模块职责与架构定位
//! 本模块位于物理页分配器/页缓存（`OwnedPages`、`FolioRead`、`FolioWriteback`）与块设备/网络驱动（VirtIO、SD/MMC、RAM 块设备）之间，负责管理 DMA 内存的所有权、地址转换及 Cache 一致性同步：
//! - **上游调用者**：块层（`Bio` 请求构造与拆分）、文件系统页缓存读写回写路径、VirtIO 与真实板级外设驱动。
//! - **下游依赖**：[`DmaDomain`]（总线地址转换与 Cache 维护）、伙伴分配器（`alloc_pages` / `alloc_pages_in`）及体系结构内存屏障（`arch::dma_*_barrier`）。
//!
//! # 核心设计与 `MappedPages` 访问状态机
//! 1. **物理载荷与设备 DMA 域解耦**：
//!    `MappedPages` 只持有底层物理页（独占页块 `OwnedPages`、页缓存读凭证 `FolioRead` 或脏页回写凭证 `FolioWriteback`）及其页内字节区间，不在构造时绑定具体设备地址或 bounce 缓冲区。
//!    只有当驱动实际提交请求时，才根据后端类型进入硬件 DMA 模式或纯 CPU 拷贝模式。
//! 2. **载荷访问权状态机（`PayloadAccess`）**：
//!    ```text
//!                         begin_device_access(domain)
//!                    +-----------------------------------+
//!                    |  (执行 Cache Flush + 写内存屏障)   |
//!                    |                                   v
//!    +---------------+---------------+           +-------+---------------+
//!    |         Idle (CPU 持有)        |           |   Device(DmaDomain)   |
//!    | - 允许 bytes() / bytes_mut()  |           | - 硬件 DMA 正在访问    |
//!    | - 允许 complete_folio / Drop  |           | - 严禁 CPU 切片或释放  |
//!    +---------------+---------------+           +-------+---------------+
//!                    ^   |                               |
//!                    |   | begin_cpu_access()            | finish_device_access()
//!                    |   v                               | (读屏障 + Cache Invalidate)
//!            +-------+---------------+                   |
//!            |        CpuCopy        |                   |
//!            | - 导出 CpuCopyRange   |                   |
//!            | - finish_cpu_access() +-------------------+
//!            +-----------------------+
//!    ```
//! 3. **异常安全与防 UAF 保护**：
//!    `MappedPages` 使用 `ManuallyDrop<PayloadBacking>` 包装底层物理页凭证，并在 `Drop` 中断言 `access == PayloadAccess::Idle`。
//!    若在外部 DMA 或 CPU 拷贝尚未结束时发生析构，断言会触发 panic 并阻止物理页被归还给分配器，防止外设 DMA 踩踏已重用的内存。
//!
//! # 锁获取顺序与并发契约
//! - `MappedPages` 自身不含内部锁，其状态转换（`begin_*` / `finish_*`）与接受字节账目（`accept_range`、`record_writeback_result`）由外层 `Bio` 或驱动队列锁串行化。
//! - 对于 `FolioWriteback`（脏页回写）载荷，页面可能同时存在用户态只读映射或并发再次标脏的原始写入，因此严禁创建 Rust `&[u8]` 切片，CPU 拷贝路径必须使用 `read_volatile` 逐字节读取。

use crate::arch;
use crate::block::{BlockError, IoResult};
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::buddy::MAX_ORDER;
use crate::mm::{
    alloc_pages, alloc_pages_in, pages as page_map, FolioRead, FolioWriteback, OwnedPages,
};
use core::convert::TryFrom;
use core::mem::{self, ManuallyDrop, MaybeUninit};
use core::ptr::NonNull;
use core::slice;
use virtio_drivers::{BufferDirection, DmaAddr, DmaRegion, Hal, MappedSegment};

mod domain;
pub use domain::{DmaDirection, DmaDomain};
#[cfg(feature = "ktest")]
mod ktest;

/// DMA 内存分配、总线地址映射或 Cache 一致性维护失败的错误类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmaError {
    /// 伙伴系统在目标物理窗口内没有足够大的连续空闲页块。
    NoMemory,
    /// 偏移量、长度或传输方向不合法（如长度为 0、算术溢出或超出底层页块范围）。
    InvalidRange,
    /// 物理地址超出设备的 DMA 寻址窗口（`DmaDomain`）或无有效的内核直接映射。
    AddressLimit,
    /// 构造 [`DmaDomain`] 时传入的掩码、物理窗口或 Cache 行大小配置非法。
    InvalidDomain,
    /// 非一致性（Non-coherent）DMA 缓冲区的起始地址或长度未按设备 Cache 行大小对齐。
    Alignment,
    /// 硬件 Cache 刷新或失效指令执行失败（携带具体静态原因描述）。
    CacheMaintenance(&'static str),
}

/// 面向 `virtio_drivers` 的内核硬件抽象层（`Hal`）实现，提供跨架构 DMA 内存屏障。
pub struct KernelHal;

// SAFETY:
// 1. RISC-V 与 LoongArch 后端均使用硬件设备级内存序指令实现 `dma_*_barrier`；
// 2. 内联汇编保留了编译器 memory 屏障语义（未标记 `nomem` 或 `readonly`），可防止编译器与硬件重排描述符和索引访问。
unsafe impl Hal for KernelHal {
    /// 在更新对设备可见的环索引（如 `avail.idx`）前，确保先前对一致性内存的写入已全局可见。
    fn dma_write_barrier() {
        arch::dma_write_barrier();
    }

    /// 在观测到设备完成更新（如 `used.idx`）后，确保后续对设备写入数据的读取排在完成标志之后。
    fn dma_read_barrier() {
        arch::dma_read_barrier();
    }

    /// 在设备通知、状态寄存器与一致性内存读写之间建立双向全序屏障。
    fn dma_barrier() {
        arch::dma_barrier();
    }
}

/// 设备与 CPU 长期共享的控制区 DMA 内存（如 Virtqueue 描述符环）。
///
/// 内部持有按 $2^{\text{order}}$ 页连续分配并清零的物理页块（`OwnedPages`）以及对应的设备总线地址。
/// 若所在 [`DmaDomain`] 为非一致性（Non-coherent）域，驱动在每次与硬件交换控制数据前后必须显式调用
/// [`Self::sync_for_device`] 和 [`Self::sync_for_cpu`]。
pub struct CoherentDma {
    /// 底层连续物理页块的所有权（析构时按原始阶数 `order` 归还伙伴系统）。
    pages: OwnedPages,
    /// 该物理页块在 `domain` 下的起始设备总线地址。
    device: DmaAddr,
    /// 该控制区所属的设备 DMA 地址域与 Cache 属性。
    domain: DmaDomain,
}

impl CoherentDma {
    /// 在指定的 `domain` 可达窗口内非阻塞分配至少容纳 `bytes` 字节的最小 $2^{\text{order}}$ 连续页块并清零。
    ///
    /// 当 `bytes == 0` 时返回 `InvalidRange`；当所需阶数超过 `MAX_ORDER` 或窗口内无可用连续块时返回 `NoMemory`。
    pub fn allocate(domain: DmaDomain, bytes: usize) -> Result<Self, DmaError> {
        if bytes == 0 {
            return Err(DmaError::InvalidRange);
        }
        let count = (bytes - 1) / PAGE_SIZE + 1;
        let count = count
            .checked_next_power_of_two()
            .ok_or(DmaError::NoMemory)?;
        let order = count.trailing_zeros() as usize;
        if MAX_ORDER < order {
            return Err(DmaError::NoMemory);
        }

        let pages = if domain == DmaDomain::qemu_coherent() {
            alloc_pages(order, true)
        } else {
            let (start, end) = domain.allocation_window();
            alloc_pages_in(order, true, start, end)
        }
        .ok_or(DmaError::NoMemory)?;

        validate_backing(pages.phys_addr(), pages.len(), 0, pages.len())?;
        let device = domain.map(pages.phys_addr(), pages.len())?;
        Ok(Self {
            pages,
            device,
            domain,
        })
    }

    /// 返回整个控制区页块的原始 CPU 指针与设备总线地址视图（[`DmaRegion`]）。
    ///
    /// 返回的裸指针不附带借用生命周期；调用方必须确保在所有基于该视图的 CPU 访问和设备 DMA 彻底停止前，保持 `self` 存活。
    pub fn region(&self) -> DmaRegion {
        DmaRegion {
            cpu: NonNull::new(arch::phys_to_virt(self.pages.phys_addr()) as *mut u8)
                .expect("DMA direct mapping is null"),
            device: self.device,
            len: self.pages.len(),
        }
    }
    /// 返回底层连续页块实际分配并已清零的总字节数（即 $2^{\text{order}} \times \text{PAGE\_SIZE}$，包含向上取整的尾部空间）。
    pub fn byte_len(&self) -> usize {
        self.pages.len()
    }

    /// 返回底层连续页块的伙伴系统分配阶数 `order`（仅供内核测试使用）。
    #[cfg(feature = "ktest")]
    pub fn order(&self) -> usize {
        self.pages.order()
    }

    /// 在通知设备读取或更新 `[offset, offset + len)` 范围前，执行必要的 Cache 刷新与写屏障。
    ///
    /// # Safety
    /// 1. 调用方已完成对该区间的所有前置 CPU 写入，且在设备访问期间不得产生冲突的 CPU 读写；
    /// 2. 非一致性域下，`[offset, offset + len)` 必须独占完整的 Cache 行，不得与并发 CPU 访问的数据共享同一 Cache 行；
    /// 3. 所有使用 [`Self::region`] 裸指针的对象在同步及后续设备访问期间必须保持 `self` 存活。
    pub unsafe fn sync_for_device(
        &self,
        offset: usize,
        len: usize,
        direction: DmaDirection,
    ) -> Result<(), DmaError> {
        validate_range(self.pages.len(), offset, len)?;
        // SAFETY:
        // 1. `validate_range` 已校验 `[offset, offset + len)` 完全落在 `self.pages` 持有的物理页块内；
        // 2. 调用方保证已停止冲突的 CPU 访问且区间满足 Cache 行隔离要求。
        unsafe {
            self.domain
                .sync_for_device(self.pages.phys_addr() + offset, len, direction)
        }
    }

    /// 在确认设备已停止访问 `[offset, offset + len)` 范围后，执行读屏障与必要的 Cache 维护以供 CPU 读取。
    ///
    /// # Safety
    /// 1. 调用方必须通过硬件完成标志或设备复位确认设备对该区间的 DMA 访问已彻底停止（不得仅凭软件超时假定硬件停止）；
    /// 2. 非一致性域下，该区间必须独占完整的 Cache 行，且设备访问期间没有并发 CPU 写入。
    pub unsafe fn sync_for_cpu(
        &self,
        offset: usize,
        len: usize,
        direction: DmaDirection,
    ) -> Result<(), DmaError> {
        validate_range(self.pages.len(), offset, len)?;
        // SAFETY:
        // 1. `validate_range` 已校验 `[offset, offset + len)` 完全落在 `self.pages` 持有的物理页块内；
        // 2. 调用方保证硬件已完成对该区间的全部 DMA 访问。
        unsafe {
            self.domain
                .sync_for_cpu(self.pages.phys_addr() + offset, len, direction)
        }
    }
}

/// I/O 数据载荷（[`MappedPages`]）的底层物理内存所有权凭证。
enum PayloadBacking {
    /// 普通内核独占物理页块（析构时直接归还伙伴分配器）。
    Pages(OwnedPages),
    /// 页缓存（Page Cache）读入填充凭证。
    Folio {
        /// 持有目标页缓存物理页与内容锁的读入句柄。
        read: FolioRead,
        /// 是否在单次传输完整覆盖 `read.len()` 时直接发布整页就绪状态（分段读取子区间时为 `false`）。
        publish_on_success: bool,
    },
    /// 页缓存脏页回写（Writeback）凭证。
    Writeback(FolioWriteback),
}

impl PayloadBacking {
    /// 返回底层物理页块或页缓存页的起始物理字节地址（PA）。
    fn phys_addr(&self) -> usize {
        match self {
            Self::Pages(pages) => pages.phys_addr(),
            Self::Folio { read, .. } => read.phys_addr(),
            Self::Writeback(writeback) => writeback.phys_addr(),
        }
    }
}

/// 数据载荷的 CPU 物理地址区间与传输方向快照（不分配或绑定设备总线地址）。
#[derive(Clone, Copy, Debug)]
pub struct PayloadRange {
    /// 载荷数据在物理内存中的起始字节地址（基底页物理地址 + 页内偏移）。
    pub physical: usize,
    /// 载荷数据的字节长度。
    pub length: u32,
    /// 数据传输方向（`DriverToDevice` 或 `DeviceToDriver`）。
    pub direction: BufferDirection,
}

/// 面向纯 CPU 内存拷贝后端（如 RAM Disk 或 Bounce Buffer）导出的子区间原始拷贝计划。
///
/// 仅保存直接映射裸指针与长度，不借用外层 [`MappedPages`]，从而允许外层在不冲突的情况下更新请求接受账目。
/// 调用方必须确保在执行拷贝期间底层 [`MappedPages`] 保持存活且处于 [`PayloadAccess::CpuCopy`] 状态。
#[derive(Clone, Copy)]
pub struct CpuCopyRange {
    /// 子区间在内核直接映射区的起始裸指针。
    pointer: *mut u8,
    /// 子区间的字节长度。
    len: usize,
    /// 允许的传输方向（`DriverToDevice` 仅允许写出，`DeviceToDriver` 仅允许读入）。
    direction: BufferDirection,
    /// 底层载荷是否为页缓存脏页回写（`FolioWriteback`），若为 `true` 则读取时必须使用 `read_volatile`。
    writeback: bool,
}

impl CpuCopyRange {
    /// 将 `DriverToDevice` 方向的载荷子区间数据拷贝到目标缓冲区 `destination`。
    ///
    /// 若底层为脏页回写（`writeback == true`），则逐字节使用 `read_volatile` 读取，避免与并发用户态写操作产生 Rust 引用别名冲突。
    ///
    /// # Safety
    /// 1. 导出本计划的 [`MappedPages`] 必须保持存活且处于 `CpuCopy` 状态直到本函数返回；
    /// 2. 源区间全部 `self.len` 个字节已完成初始化；
    /// 3. `destination` 必须指向至少 `self.len` 字节的有效可写内存，与源区间不重叠，且没有其他线程并发写入同一目标区间。
    pub unsafe fn copy_to(self, destination: *mut u8) -> Result<(), DmaError> {
        if self.direction != BufferDirection::DriverToDevice {
            return Err(DmaError::InvalidRange);
        }
        if self.writeback {
            for index in 0..self.len {
                // SAFETY:
                // 1. 调用方保证源回写页在 `CpuCopy` 期间存活，且 `destination` 在 `0..self.len` 内有效可写、不与源重叠；
                // 2. 使用 `read_volatile` 逐字节读取原始内存，不构造 `&[u8]` 切片，避免与并发标脏的用户态写入触发 UB。
                unsafe {
                    destination
                        .add(index)
                        .write(self.pointer.add(index).read_volatile())
                };
            }
        } else {
            // SAFETY:
            // 1. 调用方保证独占源物理页在 `CpuCopy` 期间存活、全部字节已初始化且无并发写入；
            // 2. `destination` 指向 `self.len` 字节的有效可写内存且与源区间不重叠。
            unsafe { core::ptr::copy_nonoverlapping(self.pointer, destination, self.len) };
        }
        Ok(())
    }

    /// 从源缓冲区 `source` 将数据拷贝到 `DeviceToDriver` 方向的载荷子区间中。
    ///
    /// # Safety
    /// 1. 导出本计划的 [`MappedPages`] 必须保持存活且处于 `CpuCopy` 状态直到本函数返回；
    /// 2. `source` 必须指向至少 `self.len` 字节的已初始化可读内存，且与目标载荷区间不重叠；
    /// 3. 对目标载荷中重叠子区间的并发拷贝必须由调用方串行化。
    pub unsafe fn copy_from(self, source: *const u8) -> Result<(), DmaError> {
        if self.direction != BufferDirection::DeviceToDriver {
            return Err(DmaError::InvalidRange);
        }
        // SAFETY:
        // 1. `direction == DeviceToDriver` 排除了共享的 `Writeback` 载荷，目标为独占物理页或独占 `FolioRead` 缓冲区；
        // 2. 调用方保证目标页在 `CpuCopy` 期间存活、`source` 具有 `self.len` 字节有效初始化数据且两侧内存不重叠。
        unsafe { core::ptr::copy_nonoverlapping(source, self.pointer, self.len) };
        Ok(())
    }
}

/// [`MappedPages`] 当前所处的外部访问占用状态。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PayloadAccess {
    /// 空闲状态（由 CPU 持有，允许获取切片、启动新的传输或安全析构）。
    Idle,
    /// 纯 CPU 拷贝占用状态（已通过 `begin_cpu_access` 进入，允许导出并使用 `CpuCopyRange`）。
    CpuCopy,
    /// 硬件 DMA 占用状态（已通过 `begin_device_access` 移交指定 `DmaDomain` 的外设访问）。
    Device(DmaDomain),
}

/// 块设备与 I/O 子系统通用的单段物理页数据载荷封装。
///
/// 内部包装了非空的物理页区间（`Pages`、`FolioRead` 或 `FolioWriteback`），并跟踪外设 DMA / CPU 拷贝的访问生命周期
/// 以及父请求拆分提交时的连续已接受字节数（`accepted`）和首个回写错误（`wb_error`）。
pub struct MappedPages {
    /// 底层物理页所有权凭证；使用 `ManuallyDrop` 防止在外部 DMA 尚未停止时因断言 panic 展开而提前释放物理页。
    backing: ManuallyDrop<PayloadBacking>,
    /// 载荷数据相对于底层物理页块起始位置的字节偏移量。
    offset: usize,
    /// 载荷数据的字节长度（不超过 `u32::MAX` 以适配硬件描述符）。
    length: u32,
    /// 数据传输方向（`DriverToDevice` 或 `DeviceToDriver`）。
    direction: BufferDirection,
    /// 当前载荷的外部访问状态（`Idle`、`CpuCopy` 或 `Device`）。
    access: PayloadAccess,
    /// 本轮传输中已被底层驱动成功接纳（Submit）的连续前缀字节数。
    accepted: usize,
    /// 脏页回写过程中记录的首个子请求失败位置及错误（`(失败处的字节偏移, BlockError)`）。
    wb_error: Option<(usize, BlockError)>,
}

impl MappedPages {
    /// 将独占物理页块 `pages` 的 `[offset, offset + len)` 子区间封装为 `MappedPages`。
    ///
    /// 若长度超出 `u32::MAX`、区间越界或物理地址无直接映射，则直接返回 `Err`，`pages` 随错误返回自动按原阶释放。
    pub fn new(
        pages: OwnedPages,
        offset: usize,
        len: usize,
        direction: BufferDirection,
    ) -> Result<Self, DmaError> {
        let length = u32::try_from(len).map_err(|_| DmaError::InvalidRange)?;
        validate_backing(pages.phys_addr(), pages.len(), offset, len)?;
        Ok(Self {
            backing: ManuallyDrop::new(PayloadBacking::Pages(pages)),
            offset,
            length,
            direction,
            access: PayloadAccess::Idle,
            accepted: 0,
            wb_error: None,
        })
    }

    /// 将页缓存整页/全区间读入句柄 `read` 封装为 `MappedPages`（传输成功填满后自动发布页面就绪）。
    ///
    /// 若校验失败则原样退回 `(DmaError, FolioRead)`，不丢失读入锁责任。
    pub(crate) fn from_folio(read: FolioRead) -> Result<Self, (DmaError, FolioRead)> {
        let offset = read.offset();
        let len = read.len();
        Self::map_folio(read, offset, len, true)
    }

    /// 将页缓存读入句柄 `read` 的指定子区间 `[offset, offset + len)` 封装为分段读取载荷。
    ///
    /// 单次子区间传输完成后不会自动发布整页就绪，调用方需通过 [`Self::into_folio`] 取回 `FolioRead` 继续后续分段填充。
    pub(crate) fn from_folio_range(
        read: FolioRead,
        offset: usize,
        len: usize,
    ) -> Result<Self, (DmaError, FolioRead)> {
        Self::map_folio(read, offset, len, false)
    }

    /// 校验 `[offset, offset + len)` 落在 `read` 允许的未完成窗口内，并构造 `PayloadBacking::Folio` 载荷。
    fn map_folio(
        read: FolioRead,
        offset: usize,
        len: usize,
        publish_on_success: bool,
    ) -> Result<Self, (DmaError, FolioRead)> {
        if read.is_completed()
            || offset < read.offset()
            || offset
                .checked_add(len)
                .is_none_or(|end| read.offset() + read.len() < end)
        {
            return Err((DmaError::InvalidRange, read));
        }
        if let Err(error) = validate_backing(read.phys_addr(), PAGE_SIZE, offset, len) {
            return Err((error, read));
        }

        Ok(Self {
            backing: ManuallyDrop::new(PayloadBacking::Folio {
                read,
                publish_on_success,
            }),
            offset,
            length: len as u32,
            direction: BufferDirection::DeviceToDriver,
            access: PayloadAccess::Idle,
            accepted: 0,
            wb_error: None,
        })
    }

    /// 将页缓存脏页回写句柄 `writeback` 封装为 `DriverToDevice` 方向的 `MappedPages`。
    ///
    /// 若校验失败则原样退回 `(DmaError, FolioWriteback)`。
    pub(crate) fn from_writeback(
        writeback: FolioWriteback,
    ) -> Result<Self, (DmaError, FolioWriteback)> {
        if writeback.is_completed() {
            return Err((DmaError::InvalidRange, writeback));
        }
        let offset = writeback.offset();
        let len = writeback.len();
        if let Err(error) = validate_backing(writeback.phys_addr(), PAGE_SIZE, offset, len) {
            return Err((error, writeback));
        }

        Ok(Self {
            backing: ManuallyDrop::new(PayloadBacking::Writeback(writeback)),
            offset,
            length: len as u32,
            direction: BufferDirection::DriverToDevice,
            access: PayloadAccess::Idle,
            accepted: 0,
            wb_error: None,
        })
    }

    /// 在分段读取的当前阶段外部访问结束后，拆出内部的 `FolioRead` 句柄以继续后续子区间读取或最终完成发布。
    pub(crate) fn into_folio(mut self) -> FolioRead {
        assert!(!self.is_device_owned(), "taking an active folio");
        assert!(
            matches!(
                &*self.backing,
                PayloadBacking::Folio {
                    publish_on_success: false,
                    ..
                }
            ),
            "taking a payload without continuing folio ownership"
        );
        // SAFETY:
        // 1. 前置 `assert!` 已确认当前处于 `Idle` 状态（外部 DMA / CPU 拷贝已结束）且为分段 `Folio` 载荷；
        // 2. 取出 `self.backing` 后立即调用 `mem::forget(self)` 跳过 `Drop`，避免重复释放。
        let backing = unsafe { ManuallyDrop::take(&mut self.backing) };
        mem::forget(self);
        match backing {
            PayloadBacking::Folio { read, .. } => read,
            PayloadBacking::Pages(_) | PayloadBacking::Writeback(_) => unreachable!(),
        }
    }

    /// 将当前载荷区间映射到指定 `domain` 的设备总线地址空间，生成供 DMA 描述符使用的 [`MappedSegment`]。
    pub fn segment(&self, domain: DmaDomain) -> Result<MappedSegment, DmaError> {
        Ok(MappedSegment {
            address: domain.map(self.backing.phys_addr() + self.offset, self.len())?,
            length: self.length,
            direction: self.direction,
        })
    }

    /// 返回当前载荷的 CPU 物理地址区间与传输方向快照（不触发任何 DMA 映射或 Cache 操作）。
    pub fn range(&self) -> PayloadRange {
        PayloadRange {
            physical: self.backing.phys_addr() + self.offset,
            length: self.length,
            direction: self.direction,
        }
    }

    /// 返回当前载荷的有效字节长度（不含底层物理页块中位于 `[offset, offset + length)` 之外的部分）。
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.length as usize
    }

    /// 返回当前载荷在底层物理页块内的起始字节偏移量（仅供内核测试使用）。
    #[cfg(feature = "ktest")]
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// 判断当前载荷是否正处于外部 DMA（`Device`）或 CPU 拷贝（`CpuCopy`）占用状态。
    pub fn is_device_owned(&self) -> bool {
        self.access != PayloadAccess::Idle
    }

    /// 判断底层页缓存凭证（`FolioRead` 或 `FolioWriteback`）是否已经完成生命周期结算。
    pub(crate) fn is_folio_complete(&self) -> bool {
        match &*self.backing {
            PayloadBacking::Folio { read, .. } => read.is_completed(),
            PayloadBacking::Writeback(writeback) => writeback.is_completed(),
            PayloadBacking::Pages(_) => false,
        }
    }

    /// 返回底层页缓存页（`FolioRead` / `FolioWriteback`）的物理基址；普通 `OwnedPages` 返回 `None`（仅供测试使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_folio_phys_addr(&self) -> Option<usize> {
        match &*self.backing {
            PayloadBacking::Folio { read, .. } => Some(read.phys_addr()),
            PayloadBacking::Writeback(writeback) => Some(writeback.phys_addr()),
            PayloadBacking::Pages(_) => None,
        }
    }

    /// 若底层为脏页回写凭证（`FolioWriteback`），返回其是否为文件系统元数据页（仅供测试使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_writeback_metadata(&self) -> Option<bool> {
        match &*self.backing {
            PayloadBacking::Writeback(writeback) => Some(writeback.is_metadata()),
            _ => None,
        }
    }

    /// 判断底层是否为关联到具体文件 Inode 的页缓存读入凭证（仅供测试使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_file_read(&self) -> bool {
        matches!(&*self.backing, PayloadBacking::Folio { read, .. } if read.ktest_is_file())
    }

    /// 在子请求成功提交到底层驱动后，累加上报已接纳的连续子区间 `[offset, offset + len)`。
    pub(crate) fn accept_range(&mut self, offset: usize, len: usize) {
        assert_eq!(
            offset, self.accepted,
            "payload acceptance is not contiguous"
        );
        assert!(len <= self.len() - offset);
        self.accepted += len;
    }

    /// 记录脏页回写子请求 `[offset, offset + len)` 的完成结果；若发生错误，仅保留字节偏移最小的首个失败位置。
    pub(crate) fn record_writeback_result(
        &mut self,
        offset: usize,
        len: usize,
        completed: usize,
        error: Option<BlockError>,
    ) {
        if !matches!(&*self.backing, PayloadBacking::Writeback(_)) {
            return;
        }
        let end = offset + len;
        // 尚未被驱动接纳（submit 失败）的子区间不计入设备回写结果。
        if self.accepted <= offset {
            return;
        }
        assert!(end <= self.accepted && completed <= len);
        if let Some(error) = error {
            let failure = offset + completed;
            if self.wb_error.is_none_or(|(first, _)| failure < first) {
                self.wb_error = Some((failure, error));
            }
        }
    }

    /// 在 CPU 持有期间（`Idle`）返回可写字节切片 `&mut [MaybeUninit<u8>]`，用于向独占页或 `FolioRead` 填充初始数据。
    ///
    /// 若当前处于外部访问状态或底层为共享的 `FolioWriteback`，则直接 panic。
    pub fn bytes_mut(&mut self) -> &mut [MaybeUninit<u8>] {
        assert!(
            !self.is_device_owned(),
            "CPU access during external payload ownership"
        );
        let len = self.len();
        match &mut *self.backing {
            PayloadBacking::Pages(pages) => {
                // SAFETY:
                // 1. `MappedPages::new` 已验证 `[offset, offset + len)` 完全位于 `pages` 持有的独占物理页块内；
                // 2. 前置断言确保当前处于 `Idle` 状态，`&mut self` 排除了并发访问，且 `MaybeUninit<u8>` 不要求内存已初始化。
                unsafe { slice::from_raw_parts_mut(pages.as_mut_ptr().add(self.offset), len) }
            }
            PayloadBacking::Folio { read, .. } => {
                let start = self.offset - read.offset();
                &mut read.bytes_mut()[start..start + len]
            }
            PayloadBacking::Writeback(_) => panic!("mutable slice of a writeback folio"),
        }
    }

    /// 在 CPU 持有期间（`Idle`）返回载荷内容的只读字节切片 `&[u8]`。
    ///
    /// # Safety
    /// 1. 调用方必须确保当前载荷的全部 `self.len()` 字节已完成初始化（例如当 DMA 发生短读或错误时，未写入的尾部字节不可作为已初始化数据读取）；
    /// 2. 先前导出的任何裸指针不得在切片借用存续期间并发修改该内存；
    /// 3. 底层不得是 `FolioWriteback`（脏页回写页可能与用户态并发写入共享，此处会触发断言 panic）。
    pub unsafe fn bytes(&self) -> &[u8] {
        assert!(
            !self.is_device_owned(),
            "CPU access during external payload ownership"
        );
        assert!(
            !matches!(&*self.backing, PayloadBacking::Writeback(_)),
            "slice of a writeback folio"
        );
        let pointer = (arch::phys_to_virt(self.backing.phys_addr()) + self.offset) as *const u8;
        // SAFETY:
        // 1. 构造时已验证物理页块边界与直接映射有效性，且前置断言确认当前处于 `Idle` 状态且非 `Writeback` 载荷；
        // 2. 调用方保证全部 `self.len()` 字节已初始化且借用期间无并发写入。
        unsafe { slice::from_raw_parts(pointer, self.len()) }
    }

    /// 在将描述符发布给硬件前，把当前载荷移交到指定 `domain` 的 DMA 设备访问状态（`Device(domain)`）。
    ///
    /// 内部会执行 `domain.sync_for_device`（在非一致性域下刷回脏 Cache 行并执行写内存屏障），并重置本轮接受账目。
    ///
    /// # Safety
    /// 1. 调用方必须已结束对该物理页的所有冲突 CPU 裸指针访问，且当方向为 `DriverToDevice` 时确保全部载荷字节已初始化；
    /// 2. 若底层为 `FolioWriteback`，调用方必须已完成页表写权限撤除（`mkclean`），且在 DMA 期间不得创建指向该页的 Rust 引用切片；
    /// 3. 在所有引用本载荷的硬件描述符真实完成（或设备完成复位）并调用 [`Self::finish_device_access`] 前，不得释放底层物理页或重复移交访问权。
    pub unsafe fn begin_device_access(&mut self, domain: DmaDomain) -> Result<(), DmaError> {
        self.check_begin();
        // SAFETY:
        // 1. `self.backing` 保活整个物理区间，且构造时已通过 `validate_backing` 校验；
        // 2. 调用方保证无冲突 CPU 访问且 `DriverToDevice` 数据已初始化，`domain.sync_for_device` 会校验窗口与 Cache 行对齐。
        unsafe {
            domain.sync_for_device(
                self.backing.phys_addr() + self.offset,
                self.len(),
                self.direction.into(),
            )?
        };
        self.reset_accounting();
        self.access = PayloadAccess::Device(domain);
        Ok(())
    }

    /// 在确认硬件已彻底停止访问本载荷后，执行读内存屏障与必要的 Cache 失效，并将状态恢复为 `Idle`。
    ///
    /// # Safety
    /// 1. 调用方必须已观测到所有引用本载荷的硬件请求真实完成（如 VirtIO `used` 环推进）或设备已完成硬件复位；
    ///    严禁仅凭软件超时、单个子请求完成或错误返回就提前收回访问权；
    /// 2. 此后硬件不得再读写该物理区间。
    pub unsafe fn finish_device_access(&mut self) -> Result<(), DmaError> {
        let PayloadAccess::Device(domain) = self.access else {
            panic!("payload is not DMA device owned");
        };
        // SAFETY:
        // 1. 调用方已确认硬件 DMA 彻底停止，`self.backing` 仍保活底层物理页；
        // 2. `DeviceToDriver` 期间禁止了 CPU 访问，而 `DriverToDevice` 不会对可能被并发重标脏的回写页执行 Cache Invalidate。
        unsafe {
            domain.sync_for_cpu(
                self.backing.phys_addr() + self.offset,
                self.len(),
                self.direction.into(),
            )?
        };
        self.access = PayloadAccess::Idle;
        Ok(())
    }

    /// 校验当前载荷处于 `Idle` 状态且底层页缓存凭证尚未结算完毕，允许开启新一轮外部访问。
    fn check_begin(&self) {
        assert!(
            !self.is_device_owned(),
            "payload already has an external owner"
        );
        assert!(
            !self.is_folio_complete(),
            "completed folio cannot enter I/O"
        );
    }

    /// 在开启新一轮外部访问时清零已接纳字节计数（`accepted`）与回写错误记录（`wb_error`）。
    fn reset_accounting(&mut self) {
        self.accepted = 0;
        self.wb_error = None;
    }

    /// 将当前载荷切换为纯 CPU 拷贝访问状态（`PayloadAccess::CpuCopy`），用于 RAM 块设备或软件拷贝路径。
    ///
    /// # Safety
    /// 1. 调用方必须已结束对底层独占页或 `FolioRead` 的冲突裸指针访问；
    /// 2. 当方向为 `DriverToDevice` 时，载荷范围内的全部字节必须已完成初始化。
    pub unsafe fn begin_cpu_access(&mut self) {
        self.check_begin();
        self.reset_accounting();
        self.access = PayloadAccess::CpuCopy;
    }

    /// 在所有基于 [`CpuCopyRange`] 的 CPU 拷贝操作全部返回后，将载荷状态恢复为 `Idle`。
    ///
    /// # Safety
    /// 调用方必须确保先前由 [`Self::cpu_copy_range`] 导出的所有拷贝计划均已执行完毕，且不再保留任何访问载荷内存的裸指针。
    pub unsafe fn finish_cpu_access(&mut self) {
        assert_eq!(self.access, PayloadAccess::CpuCopy);
        self.access = PayloadAccess::Idle;
    }

    /// 在 `CpuCopy` 状态下导出子区间 `[offset, offset + len)` 的原始 CPU 拷贝计划 [`CpuCopyRange`]。
    ///
    /// # Safety
    /// 1. 调用方必须在执行拷贝期间保持当前 `MappedPages` 存活且维持在 `CpuCopy` 状态（不得在拷贝完成前调用 `finish_cpu_access`）；
    /// 2. 若多个子任务向重叠的目标区间拷贝，调用方必须自行串行化。
    pub unsafe fn cpu_copy_range(
        &self,
        offset: usize,
        len: usize,
    ) -> Result<CpuCopyRange, DmaError> {
        assert_eq!(self.access, PayloadAccess::CpuCopy);
        validate_range(self.len(), offset, len)?;
        Ok(CpuCopyRange {
            pointer: (arch::phys_to_virt(self.backing.phys_addr()) + self.offset + offset)
                as *mut u8,
            len,
            direction: self.direction,
            writeback: matches!(&*self.backing, PayloadBacking::Writeback(_)),
        })
    }

    /// 在父 `Bio` 请求的所有子请求全部完成并收回载荷访问权后，结算底层页缓存凭证（`FolioRead` 或 `FolioWriteback`）。
    ///
    /// - 对于 `FolioRead`：若 `publish_on_success` 为真且成功读取了整页长度，则忽略父请求后续区间的错误并发布页面就绪；否则按实际结果结算。
    /// - 对于 `FolioWriteback`：根据本载荷实际接纳的字节数 `self.accepted` 与首个回写错误 `self.wb_error` 结算脏页状态。
    ///
    /// # Safety
    /// 1. 当前载荷的所有外部硬件 DMA 和 CPU 拷贝必须已彻底结束（`!self.is_device_owned()`），且无残留的裸指针别名；
    /// 2. `result` 必须准确反映父请求落在当前载荷上的连续成功前缀字节数与错误状态。
    pub(crate) unsafe fn complete_folio(&mut self, result: IoResult) {
        assert!(!self.is_device_owned(), "completing active folio");
        match &mut *self.backing {
            PayloadBacking::Folio {
                read,
                publish_on_success,
            } => {
                if *publish_on_success || result.error.is_some() {
                    let result = if *publish_on_success && result.completed == read.len() {
                        IoResult {
                            completed: result.completed,
                            error: None,
                        }
                    } else {
                        result
                    };
                    // SAFETY:
                    // 1. 前置断言已确认外部 DMA / CPU 拷贝全部结束；
                    // 2. `result.completed` 为当前 `FolioRead` 区间内实际已填充完毕的连续前缀字节数。
                    unsafe { read.complete(result) };
                }
            }
            PayloadBacking::Writeback(writeback) => {
                let result = match self.wb_error {
                    Some((completed, error)) => IoResult {
                        completed,
                        error: Some(error),
                    },
                    None => IoResult {
                        completed: self.accepted,
                        error: None,
                    },
                };
                // SAFETY:
                // 1. 前置断言已确认所有子请求与外部访问全部结束；
                // 2. `self.accepted` 仅统计成功提交的子区间，报错偏移来自实际失败的子请求记录。
                unsafe { writeback.complete(self.accepted, result) };
            }
            PayloadBacking::Pages(_) => {}
        }
    }
}

impl Drop for MappedPages {
    /// 析构 `MappedPages` 并释放底层物理页或结算未提交的页缓存凭证。
    ///
    /// 若析构时仍处于外部 DMA 或 CPU 拷贝状态，断言将直接触发 panic，从而保留 `ManuallyDrop` 中的物理页不被释放，防止 DMA 悬垂写。
    fn drop(&mut self) {
        assert!(!self.is_device_owned(), "dropping active payload pages");
        // SAFETY:
        // 1. 前置 `assert!` 已确认外部 DMA 与 CPU 拷贝均已停止，且 `self.backing` 自构造以来未被移出；
        // 2. 此时安全调用 `ManuallyDrop::drop` 释放独占物理页，或触发 `FolioRead` / `FolioWriteback` 的析构清理。
        unsafe { ManuallyDrop::drop(&mut self.backing) };
    }
}

/// 校验非空子区间 `[offset, offset + len)` 是否未发生算术溢出且完全落在总长度 `allocation_len` 之内。
fn validate_range(allocation_len: usize, offset: usize, len: usize) -> Result<(), DmaError> {
    let end = offset.checked_add(len).ok_or(DmaError::InvalidRange)?;
    if len == 0 || allocation_len < end {
        return Err(DmaError::InvalidRange);
    }
    Ok(())
}

/// 校验物理页块 `[physical, physical + allocation_len)` 及其子区间 `[offset, offset + len)` 属于受管 RAM 且具备有效的内核直接映射。
fn validate_backing(
    physical: usize,
    allocation_len: usize,
    offset: usize,
    len: usize,
) -> Result<(), DmaError> {
    validate_range(allocation_len, offset, len)?;
    let block_end = physical
        .checked_add(allocation_len)
        .ok_or(DmaError::AddressLimit)?;
    let head_pfn = physical / PAGE_SIZE;
    let end_pfn = block_end / PAGE_SIZE;
    if !page_map()
        .free_ranges()
        .iter()
        .any(|range| range.start <= head_pfn && end_pfn <= range.end)
    {
        return Err(DmaError::AddressLimit);
    }

    let cpu = arch::phys_to_virt(physical);
    if cpu == 0 || cpu.checked_add(allocation_len).is_none() {
        return Err(DmaError::AddressLimit);
    }
    Ok(())
}
