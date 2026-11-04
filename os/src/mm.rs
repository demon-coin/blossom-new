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

static MEMORY: Once<PageMap<'static>> = Once::new();

/// 物理内存各域的页数快照；启动分类固定，其他字段只在参与者停稳后构成精确账务。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MemoryStats {
    pub present_pages: usize,
    pub reserved_pages: usize,
    pub metadata_pages: usize,
    pub buddy_free_pages: usize,
    pub pcp_cached_pages: usize,
    pub slab_backing_pages: usize,
}

/// 全部后端初始化后由不持 MM 锁的调用者逐域读取真实页数；跨锁快照不能推导并发 live 数。
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

/// 借用永久发布的物理页描述符表；初始化前调用直接报告时序错误。
pub(crate) fn pages() -> &'static PageMap<'static> {
    MEMORY
        .get()
        .expect("physical memory has not been initialized")
}

/// 从实际启动 RAM 发布描述符，依次建立 buddy、每 CPU 空页缓存与固定 SLUB 控制对象。
///
/// # Safety
/// 只能由 BSP 在 IRQ 关闭、AP 未释放时调用一次。info 必须是已验证并包含完整
/// 内核、启动栈、页表及固件保留区的实际 BootInfo；全部实际 RAM 的直接映射必须
/// 永久可写且一一对应。调用期间不得有其他使用者访问或占用尚未保留的 RAM；
/// 描述符及索引 backing 发布后不得通过其他别名修改、释放或重新分配。
pub unsafe fn initialize(info: &BootInfo) {
    assert!(
        MEMORY.get().is_none(),
        "physical memory is already initialized"
    );

    let mut memory = const { BootMemory::empty() };
    memory
        .initialize(
            &info.memory[..info.memory_count],
            &info.reserved[..info.reserved_count],
        )
        .unwrap_or_else(|error| panic!("memory boot range initialization failed: {}", error));
    // SAFETY: 调用者保证 BSP 独占实际空闲 RAM、IRQ 关闭且 AP 未释放；BootMemory
    // 来自完整实际 BootInfo，backing 使用永久可写且一一对应的 RAM 直接映射，
    // 发布后不再通过其他别名修改或回收；初始化失败立即 panic 终止启动。
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

    // SAFETY: 调用者保证 BSP 唯一初始化、IRQ 关闭且 AP 未释放；map 及全部
    // 描述符已永久发布，最终 free 范围尚未交给其他 owner，也没有并发分配或归还。
    unsafe { buddy::initialize(map) };
    let free = buddy::free_pages();
    assert_eq!(
        free, stats.free_pages,
        "buddy initial free pages differ from boot memory"
    );
    crate::early_println!("BUDDY free={} max_order={}", free, buddy::MAX_ORDER);

    // SAFETY: buddy 已唯一接管全部自由页；调用者保证 BSP 独占初始化、IRQ 关闭
    // 且 AP 未释放，当前尚无页申请或 PCP 使用者。
    unsafe { pcp::initialize() };

    // SAFETY: 永久描述符、buddy 与 PCP 均已建立；BSP 仍独占启动，IRQ 关闭、
    // AP 未释放且没有 SLUB 对象使用者，仅一次构造固定 cache/CPU/node 控制对象。
    unsafe { slub::initialize() };
}
