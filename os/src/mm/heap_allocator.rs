//! 无状态 Rust 堆入口，按原 Layout 将唯一归还责任交给 SLUB 或整体页块。

use core::alloc::{GlobalAlloc, Layout};
use core::ptr::{self, NonNull};
#[cfg(feature = "ktest")]
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::buddy::MAX_ORDER;
use crate::mm::{alloc_pages, slub, OwnedPages, PAGE_SHIFT};

struct HeapAllocator;

#[global_allocator]
static HEAP_ALLOCATOR: HeapAllocator = HeapAllocator;

#[cfg(feature = "ktest")]
static ALLOCATION_ATTEMPTS: AtomicUsize = AtomicUsize::new(0);

/// 观察真实全局分配器入口次数；调用者须先停稳其他分配参与者再比较有限区间。
#[cfg(feature = "ktest")]
pub(crate) fn allocation_attempts() -> usize {
    ALLOCATION_ATTEMPTS.load(Ordering::Relaxed)
}

/// 按容量与对齐选择最小整体页阶；先拒绝超过真实上限的请求再取整。
fn page_order(layout: Layout) -> Option<usize> {
    let required = layout.size().max(layout.align()).max(PAGE_SIZE);
    if PAGE_SIZE << MAX_ORDER < required {
        return None;
    }
    Some(required.next_power_of_two().trailing_zeros() as usize - PAGE_SHIFT)
}

// SAFETY: BSP 在任何堆消费者和 AP 运行前发布唯一页与 SLUB 后端，具体静态对象不持有
// 可变状态；后端自行同步。有效非零 Layout 决定类别及原阶，失败返回 null，归还消耗
// 同一分配的唯一责任，不依赖堆构造元数据，也不在持锁或 IRQ guard 内清零、复制。
unsafe impl GlobalAlloc for HeapAllocator {
    /// 非阻塞地申请满足大小与对齐的真实对象；容量不足不触发隐式回收或重试。
    ///
    /// # Safety
    /// layout 必须满足 GlobalAlloc 的有效非零布局合同；页与 SLUB 后端须已初始化。
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

    /// 按原布局恢复真实分配类别与页阶，结束对象拥有并归还原后端。
    ///
    /// # Safety
    /// pointer 必须是本 allocator 返回且仍 live 的原首址，layout 必须等于原分配布局；
    /// 调用者交出唯一归还权，全部访问已结束，无另一份有效凭证且从未归还。
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        if slub::class_for(layout).is_some() {
            // SAFETY: GlobalAlloc 调用者交出本入口返回的非空 live 对象及原布局；
            // 同一 class 判定证明其来自 SLUB，全部访问已结束且归还权唯一。
            unsafe { slub::deallocate(NonNull::new_unchecked(pointer)) };
        } else {
            let order = page_order(layout).expect("live heap block exceeds maximum order");
            // SAFETY: 原布局与 alloc 使用相同选阶，pointer 是 into_raw 转移的原直映块
            // 首址；整块仍 live，访问已结束且无其他 owner，恢复后唯一 Drop 按原阶归还。
            drop(unsafe { OwnedPages::from_raw(pointer, order) });
        }
    }
}
