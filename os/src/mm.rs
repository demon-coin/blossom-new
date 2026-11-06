//! 内存管理子系统（Memory Management）顶层入口与初始化编排。
//!
//! ## 模块职责与架构定位
//! 本模块汇总内核物理内存管理与虚拟地址空间支持，负责在启动早期按依赖顺序建立各层物理分配器，
//! 并向上层（进程地址空间 `space`/`vma`、文件系统页缓存 `folio`、设备 DMA `dma`、全局堆分配器 `heap_allocator`）
//! 导出统一的页帧句柄与内存账务查询接口。
//!
//! - **上游调用者**：内核启动主流程调用 [`initialize`] 完成一次性启动初始化；各业务模块通过
//!   [`alloc_pages`]、[`alloc_pages_in`]、[`FolioRef`]、[`AnonPageRef`] 等申请与管理物理内存；
//!   系统观测与测试框架通过 [`stats`] 获取物理页分布快照。
//! - **下游子模块**：
//!   - `bootmem`：启动期物理内存区间归并、保留区扣除与元数据连续页静态账本。
//!   - `page`：稀疏分段（Section）物理页描述符表（`PageMap` / `PageDesc`）及每页用途状态槽。
//!   - `buddy`：单 Normal Zone 伙伴系统（Buddy System），管理 $2^0 \sim 2^{10}$ 阶连续物理页块。
//!   - `pcp`：每 CPU 单页缓存（Per-CPU Pageset），加速零阶（4 KiB）单页的本地无竞争分配与释放。
//!   - `frame_allocator`：RAII 连续物理页块所有权封装（[`OwnedPages`]）与对外页分配入口。
//!   - `slub` / `heap_allocator`：基于页块构建的固定大小对象缓存与 Rust `GlobalAlloc` 内核堆后端。
//!   - `anon` / `folio` / `reclaim`：匿名页引用计数、文件/元数据页缓存（Folio）与内存回收慢路径。
//!   - `dma` / `space` / `user_table` / `vma` / `uaccess`：DMA 一致性缓冲区、用户虚拟地址空间、页表映射与用户态拷贝。
//!
//! ## 核心初始化流水线与分配器分层
//! ```text
//! BootInfo (RAM & Reserved 物理区间)
//!    │
//!    ▼  Step 1: bootmem::BootMemory::initialize
//! [BootMemory 区间账本] ── 归并 RAM(向内页对齐) / 扣除 Reserved(向外页对齐)
//!    │
//!    ▼  Step 2: page::initialize(&mut memory)
//! [MEMORY: PageMap<'static>] ── 从空闲区切出元数据页，建立 Section 根表与 PageDesc 数组
//!    │
//!    ▼  Step 3: buddy::initialize(map)
//! [Buddy Zone] ── 接管剩余全部 free_ranges，按 PFN 对齐拆分为 0..=MAX_ORDER 空闲链表
//!    │
//!    ▼  Step 4: pcp::initialize()
//! [Per-CPU Pagesets] ── 为每个在线 CPU 建立零阶单页本地缓存控制结构
//!    │
//!    ▼  Step 5: slub::initialize()
//! [SLUB Caches] ── 建立 8B..=4096B 共 12 个大小等级的每 CPU / 节点级部分满链表
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 物理内存分配与页描述符层全部使用关中断或不可睡眠的自旋锁（`SpinLock`），严禁在持锁期间睡眠、触发缺页或执行整页清零：
//! 1. **SLUB 控制锁**（`CpuSlab` 自旋锁 或 `NodePartial` 自旋锁，二者平级互不嵌套）
//!    $\rightarrow$ **页用途锁**（`PageDesc::usage_lock`）。
//! 2. **每 CPU 页缓存锁**（`pcp::Pageset` 自旋锁）
//!    $\rightarrow$ **伙伴系统锁**（`buddy::Zone` 自旋锁）。
//! 3. **页块释放铁律**：从 `PageDesc::usage_lock` 或 SLUB 锁中摘出底层 [`OwnedPages`] 后，
//!    **必须先释放所有上述自旋锁**，随后才能调用 `Drop` 将物理页归还给 `pcp` 或 `buddy`，防止锁顺序反转或持锁时间过长。

mod anon;
mod bootmem;
mod buddy;
pub mod dma;
pub mod folio;
mod frame_allocator;
mod heap_allocator;
#[cfg(feature = "ktest")]
pub(crate) mod ktest;
mod page;
mod pcp;
pub(crate) mod reclaim;
mod slub;
pub(crate) mod space;
pub(crate) mod uaccess;
pub(crate) mod user_table;
pub(crate) mod vma;

pub use anon::AnonPageRef;
pub(crate) use bootmem::{PAGE_SHIFT, PAGE_SIZE};
#[cfg(feature = "ktest")]
pub(crate) use buddy::MAX_ORDER;
pub use folio::{FolioRead, FolioRef, FolioWriteback};
pub use frame_allocator::{alloc_pages, alloc_pages_in, OwnedPages};
#[cfg(feature = "ktest")]
pub(crate) use heap_allocator::allocation_attempts;
#[cfg(feature = "ktest")]
pub use pcp::drain_all_pages;
#[cfg(feature = "ktest")]
pub(crate) use slub::SIZES as SLUB_SIZES;
#[cfg(feature = "ktest")]
pub use slub::{flush_all, shrink_all};

use crate::boot::BootInfo;
use crate::mm::bootmem::BootMemory;
use crate::mm::page::{PageDesc, PageMap};
use core::mem::size_of;
use spin::Once;

/// 全局永久物理页描述符映射表，由启动主核（BSP）在 [`initialize`] 中一次性写入后只读访问。
static MEMORY: Once<PageMap<'static>> = Once::new();

/// 物理内存各分类域的页数统计快照（单位均为 4 KiB 物理页）。
///
/// 其中启动期确定的三项静态分类满足恒等式：
/// `present_pages == reserved_pages + metadata_pages + 初始可分配总页数`；
/// 而伙伴系统、PCP 与 SLUB 三项动态指标仅在系统无并发分配/释放活动时才能精确闭合对账。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MemoryStats {
    /// 纳入内核管理的完整物理 RAM 总页数（不含物理地址空洞与不足一页的边缘碎片）。
    pub present_pages: usize,
    /// 与真实整页 RAM 相交的启动保留页数（含内核镜像、启动栈、早期页表、DTB 与固件保留区）。
    pub reserved_pages: usize,
    /// 启动阶段为存放 `PageMap` 根索引、区间快照及 `PageDesc` 数组而永久划拨的元数据页数。
    pub metadata_pages: usize,
    /// 当前停留在伙伴系统（Buddy System）各阶空闲链表中的物理页总数。
    pub buddy_free_pages: usize,
    /// 当前暂存在所有 CPU 本地单页缓存（Per-CPU Pageset）中的零阶空闲页总数。
    pub pcp_cached_pages: usize,
    /// 当前由 SLUB 分配器向页分配器持有的底层承载页（Slab Backing Pages）总数。
    pub slab_backing_pages: usize,
}

/// 读取当前物理内存各分类域的页数统计快照。
///
/// 调用上下文：可在任意未持有内存管理内部自旋锁（`Zone`、`Pageset`、SLUB 锁）的上下文中调用。
/// 由于各动态子系统独立加锁或通过原子变量读取，并发负载下返回的跨组件快照不具备瞬时原子性，
/// 不能在并发分配期间直接用于减法推导精确的在用页数。
pub fn stats() -> MemoryStats {
    let boot = pages().stats();
    MemoryStats {
        present_pages: boot.present_pages,
        reserved_pages: boot.reserved_pages,
        metadata_pages: boot.metadata_pages,
        buddy_free_pages: buddy::free_pages(),
        pcp_cached_pages: pcp::cached_pages(),
        slab_backing_pages: slub::backing_pages(),
    }
}

/// 获取全局永久物理页描述符表 [`PageMap`] 的共享引用。
///
/// 若在 [`initialize`] 完成之前调用，将直接触发 `panic` 暴露启动时序错误。
pub(crate) fn pages() -> &'static PageMap<'static> {
    MEMORY
        .get()
        .expect("physical memory has not been initialized")
}

/// 根据启动信息初始化物理内存描述符表、伙伴系统、每 CPU 单页缓存与 SLUB 分配器。
///
/// # Safety
/// 1. **单核单次启动时序**：只能由启动主核（BSP）在本地中断关闭、从核（AP）尚未唤醒的启动早期调用一次。
/// 2. **启动区间完整性**：`info` 必须描述当前机器真实的物理内存布局，且 `info.reserved` 已完整覆盖内核代码段/数据段、启动栈、活动页表、设备树（DTB）及固件保留区，防止这些正在使用的内存被当作空闲页分配出去。
/// 3. **直接映射契约**：所有可用物理 RAM 必须已通过内核直接映射窗口（`arch::phys_to_virt`）建立一一对应、永久有效且可写的虚拟地址映射。
/// 4. **独占内存访问**：调用期间不存在任何并发路径读写尚未保留的空闲物理页；由本函数划拨给 `PageMap` 的元数据物理页永久归描述符表独占，后续严禁通过其他裸指针修改或回收。
pub unsafe fn initialize(info: &BootInfo) {
    assert!(
        MEMORY.get().is_none(),
        "physical memory is already initialized"
    );

    // Step 1: 归并物理 RAM 与保留区间，在栈上构造启动早期整页内存账本。
    let mut memory = const { BootMemory::empty() };
    memory
        .initialize(
            &info.memory[..info.memory_count],
            &info.reserved[..info.reserved_count],
        )
        .unwrap_or_else(|error| panic!("memory boot range initialization failed: {}", error));

    // Step 2: 从空闲 RAM 中切出元数据页，构建稀疏分段物理页描述符表并发布到全局 `MEMORY`。
    // SAFETY:
    // 1. 本函数 `# Safety` 契约保证当前为 BSP 关中断单线程执行，AP 未启动，无并发访问。
    // 2. `memory` 由经过校验的 `BootInfo` 初始化，所有可用页均具备永久可写的一一对应直接映射。
    // 3. 切出的元数据页在 `BootMemory` 中立即登记为 `Metadata`，不会进入后续 `free_ranges`，发布后永久只读或受各自内部锁保护；若失败则立即 panic 中止启动。
    let map = unsafe { page::initialize(&mut memory) }
        .unwrap_or_else(|error| panic!("physical page map initialization failed: {}", error));
    MEMORY.call_once(|| map);

    let map = pages();
    let stats = map.stats();
    crate::early_println!("MEMORY present={} reserved={} metadata={} free={} sections={} present_sections={} descriptor_size={}",
    stats.present_pages,
    stats.reserved_pages,
    stats.metadata_pages,
    stats.free_pages,
    map.section_count(),
    map.present_sections(),
    size_of::<PageDesc>());

    // Step 3: 将扣除保留区与元数据页后的最终空闲区间移交给伙伴系统（Buddy System）。
    // SAFETY:
    // 1. `map` 刚刚存入全局 `MEMORY: Once`，具有 `'static` 生命周期且地址固定。
    // 2. 当前仍处于 BSP 关中断单核启动期，`map.free_ranges()` 中的空闲页尚未交给任何分配器或使用者，且 `PageDesc::buddy` 字段无其他 Zone 竞争。
    unsafe { buddy::initialize(map) };
    let free = buddy::free_pages();
    assert_eq!(
        free, stats.free_pages,
        "buddy initial free pages differ from boot memory"
    );
    crate::early_println!("BUDDY free={} max_order={}", free, buddy::MAX_ORDER);

    // Step 4: 基于伙伴系统的可管理总页数计算批量水位，初始化各在线 CPU 的零阶单页缓存（PCP）。
    // SAFETY:
    // 1. `buddy::initialize` 已完成，且 CPU 拓扑在进入 `mm::initialize` 前已发现完毕。
    // 2. 当前处于 BSP 关中断单核启动期，AP 未启动，尚无任何页分配或释放请求访问 `PAGESETS`。
    unsafe { pcp::initialize() };

    // Step 5: 初始化 SLUB 固定大小对象缓存的每 CPU 控制槽与节点级部分满链表，为全局堆分配器就绪做准备。
    // SAFETY:
    // 1. `MEMORY`、`buddy` 与 `pcp` 均已就绪，满足 SLUB 控制表初始化的前置依赖。
    // 2. 当前仍由 BSP 在关中断、AP 未唤醒且无堆对象使用者的状态下唯一调用一次。
    unsafe { slub::initialize() };
}
