//! Rust 全局内核堆分配器入口（`#[global_allocator]`）。
//!
//! ## 模块职责与架构定位
//! 本模块实现标准库的 `core::alloc::GlobalAlloc` 特征，将内核中的 `Box`、`Vec`、`Arc` 等
//! 动态内存分配请求按请求布局（`Layout`）分流至两个底层分配器：
//! - **小对象路径（`<= 4096B` 且对齐 `<= 4096B`）**：交由 [`crate::mm::slub`] 按 12 档固定大小类别分配与释放；
//! - **大块/高对齐路径（`> 4096B` 或对齐超出同档步长）**：直接向 [`crate::mm::alloc_pages`] 申请满足大小与对齐的
//!   连续物理页块（Order `0..=MAX_ORDER`），释放时按相同布局推导阶数并通过 [`OwnedPages`] 归还。
//!
//! ## 核心分流流程
//! ```text
//! GlobalAlloc::alloc(layout) / dealloc(ptr, layout)
//!                     |
//!                     +--> slub::class_for(layout) 命中?
//!                            |-- 是 --> slub::allocate(class) / slub::deallocate(ptr)
//!                            |-- 否 --> page_order(layout)
//!                                         |-- 超出 MAX_ORDER --> 返回 null_mut()
//!                                         |-- 合法阶数 --------> alloc_pages(order, false)
//!                                                              / drop(OwnedPages::from_raw)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `HeapAllocator` 本身是无状态零大小结构体（ZST），不持有任何锁；
//! - 并发安全由下游的 `slub`（每 CPU 缓存锁与 Slab 描述符锁）和 `pcp`/`buddy`（每 CPU 页集锁与伙伴系统 Zone 锁）保证；
//! - 分配过程为非阻塞快路径（`alloc_pages(order, false)` 不触发页面回收重试），可在持自旋锁或关中断上下文中调用。

use core::alloc::{GlobalAlloc, Layout};
use core::ptr::{self, NonNull};
#[cfg(feature = "ktest")]
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::buddy::MAX_ORDER;
use crate::mm::{alloc_pages, slub, OwnedPages, PAGE_SHIFT};

/// 内核全局堆分配器零大小类型（ZST），通过 `#[global_allocator]` 注册为 Rust 全局分配器。
struct HeapAllocator;

/// 全局堆分配器静态实例。
#[global_allocator]
static HEAP_ALLOCATOR: HeapAllocator = HeapAllocator;

/// 内核测试专用的全局堆分配尝试次数原子计数器。
#[cfg(feature = "ktest")]
static ALLOCATION_ATTEMPTS: AtomicUsize = AtomicUsize::new(0);

/// 读取全局堆分配器自启动以来的累计 `alloc` 调用次数（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) fn allocation_attempts() -> usize {
    ALLOCATION_ATTEMPTS.load(Ordering::Relaxed)
}

/// 根据请求的 `Layout`（大小与对齐要求）计算所需的最小连续物理页阶数（Order）。
///
/// 若所需字节数超过伙伴系统支持的最大块大小（`PAGE_SIZE << MAX_ORDER`），则返回 `None`。
fn page_order(layout: Layout) -> Option<usize> {
    let required = layout.size().max(layout.align()).max(PAGE_SIZE);
    if PAGE_SIZE << MAX_ORDER < required {
        return None;
    }
    Some(required.next_power_of_two().trailing_zeros() as usize - PAGE_SHIFT)
}

// SAFETY:
// 1. 启动核（BSP）在任何堆分配发生以及从核（AP）启动前，已完成物理页分配器（buddy/PCP）和 SLUB 分配器的初始化；
// 2. `HeapAllocator` 不包含可变静态状态，底层 `slub` 与 `alloc_pages` 均具备完整的多核并发与中断安全同步；
// 3. `alloc` 与 `dealloc` 对同一个 `Layout` 使用完全一致的 `slub::class_for` 与 `page_order` 判定逻辑，保证分配与释放严格配对。
unsafe impl GlobalAlloc for HeapAllocator {
    /// 分配满足 `layout` 大小与对齐要求的内核堆内存；若内存不足或超出最大阶数则返回空指针。
    ///
    /// # Safety
    /// 1. `layout` 必须满足 `GlobalAlloc::alloc` 的契约（大小非零，且对齐为 2 的幂）；
    /// 2. 物理页分配器（`buddy`/`pcp`）与 `slub` 分配器必须已完成初始化。
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        #[cfg(feature = "ktest")]
        ALLOCATION_ATTEMPTS.fetch_add(1, Ordering::Relaxed);
        if let Some(class) = slub::class_for(layout) {
            return slub::allocate(class).map_or(ptr::null_mut(), NonNull::as_ptr);
        }
        let Some(order) = page_order(layout) else {
            return ptr::null_mut();
        };
        alloc_pages(order, false).map_or(ptr::null_mut(), OwnedPages::into_raw)
    }

    /// 按照分配时的原始 `layout` 释放对应的 SLUB 对象或连续物理页块。
    ///
    /// # Safety
    /// 1. `pointer` 必须是由本分配器 `alloc` 返回且尚未释放的有效非空内存首地址；
    /// 2. `layout` 必须与分配该内存块时传入的 `Layout` 完全一致；
    /// 3. 调用方在此调用后交出该内存块的唯一所有权，此前对该内存块的所有读写访问必须已结束。
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if slub::class_for(layout).is_some() {
            // SAFETY:
            // 1. 调用方契约保证 `pointer` 是由 `alloc` 分配的非空有效指针，且 `layout` 与分配时一致；
            // 2. `slub::class_for(layout).is_some()` 证明该对象由 `slub::allocate` 分配，且所有外部访问已结束。
            unsafe { slub::deallocate(NonNull::new_unchecked(pointer)) };
        } else {
            let order = page_order(layout).expect("live heap block exceeds maximum order");
            // SAFETY:
            // 1. `slub::class_for(layout).is_none()` 证明该内存块在 `alloc` 中通过 `alloc_pages(order, false)` 分配并由 `OwnedPages::into_raw` 转出；
            // 2. `order` 由相同的 `page_order(layout)` 推导得出，`pointer` 为该连续页块的直接映射首地址，重建 `OwnedPages` 并立即析构即可按原阶归还物理页。
            drop(unsafe { OwnedPages::from_raw(pointer, order) });
        }
    }
}
