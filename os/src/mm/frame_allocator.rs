//! 物理页帧分配入口与 RAII 连续页块所有权封装（Frame Allocator）。
//!
//! ## 模块职责与架构定位
//! 本模块位于底层页块池（`pcp` + `buddy`）之上，将内部不自动释放的 `buddy::Block` 封装为具备
//! Rust 移动语义与自动析构归还能力的公开句柄 [`OwnedPages`]，并向内核其余子系统提供统一的物理页分配接口。
//!
//! - **上游调用者**：内核栈分配、用户页表页分配、SLUB 承载页分配、全局堆大页回退分配、匿名页（`anon`）、
//!   页缓存（`folio`）与设备 DMA 缓冲区（`dma`）。
//! - **下游依赖**：`pcp`（零阶 4 KiB 单页快速路径）、`buddy`（高阶连续块与物理窗口受限分配）、`arch::phys_to_virt` / `arch::virt_to_phys`。
//!
//! ## 核心分配与释放分流路径
//! ```text
//! alloc_pages(order, zeroed)                     alloc_pages_in(order, zeroed, start, end)
//!       │                                                       │
//!       ├── order == 0 ──► pcp::allocate()                      └──► buddy::allocate_in(order, start_pfn, end_pfn)
//!       └── order  > 0 ──► buddy::allocate(order)                               │
//!                 │                                                             │
//!                 └──────────────────────┬──────────────────────────────────────┘
//!                                        ▼
//!                         [已退出全部 Zone/PCP 自旋锁与关中断守卫]
//!                         if zeroed { write_bytes(0, PAGE_SIZE << order) }
//!                                        │
//!                                        ▼
//!                                  OwnedPages { block }
//!                                        │
//!                                        ▼  Drop::drop(&mut self)
//!                         ┌──────────────┴──────────────┐
//!                   order == 0                     order > 0
//!                         ▼                             ▼
//!              pcp::release(head_pfn)       buddy::release(head_pfn, order)
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - 本模块自身不定义新锁，但在 `alloc_pages`、`alloc_pages_in` 与 `OwnedPages::drop` 内部会短暂获取 `pcp::Pageset` 锁或 `buddy::ZONE` 锁。
//! - **严禁在持有 `PageDesc::usage_lock` 或 SLUB 控制锁时析构 [`OwnedPages`]**，调用方必须先将 `OwnedPages` 移出内层锁作用域再执行 `drop`。
//! - **锁外清零设计**：当 `zeroed == true` 时，整块内存清零严格安排在 `pcp` / `buddy` 释放自旋锁与恢复中断之后执行，避免大块清零阻塞其他核分配或拉长关中断延迟。

use crate::arch;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::buddy::{self, Block};
use crate::mm::pcp;
use core::mem::{ManuallyDrop, MaybeUninit};

/// 连续物理页块的独占所有权句柄（RAII 封装）。
///
/// 大小固定为 $2^{\text{order}}$ 个连续 4 KiB 物理页；不支持 `Clone`，析构时自动按原始阶数归还给每 CPU 缓存或伙伴系统。
pub struct OwnedPages {
    /// 底层伙伴系统块凭据（记录块首 PFN 与原始分配阶数 `order`）。
    block: Block,
}

impl OwnedPages {
    /// 返回该连续物理页块的起始物理字节地址（按 `PAGE_SIZE << order` 对齐）。
    ///
    /// 注意：返回的纯整数地址仅作硬件页表项或 DMA 描述符填写之用，不延长本 [`OwnedPages`] 的生命周期，
    /// 调用方必须确保在硬件或 DMA 访问彻底结束并收回访问权后才允许析构本句柄。
    pub fn phys_addr(&self) -> usize {
        self.block.head_pfn() * PAGE_SIZE
    }

    /// 返回该连续物理页块的总字节长度（等于 `PAGE_SIZE << self.order()`，至少为 4096 字节）。
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        PAGE_SIZE << self.order()
    }

    /// 返回该连续物理页块分配时的阶数 `order`（块包含 $2^{\text{order}}$ 个 4 KiB 页）。
    pub fn order(&self) -> usize {
        self.block.order()
    }

    /// 返回指向该连续物理页块内核直接映射起始位置的裸可变字节指针。
    ///
    /// 若分配时传入 `zeroed == false`，页内字节可能保留前任使用者的残留数据，读取前必须先完成初始化；
    /// 通过该裸指针衍生的一切读写访问必须在 [`OwnedPages`] 析构或拆解归还之前彻底结束。
    pub fn as_mut_ptr(&mut self) -> *mut MaybeUninit<u8> {
        arch::phys_to_virt(self.phys_addr()) as *mut MaybeUninit<u8>
    }

    /// 消费 `self` 并抑制自动析构，将整块物理页的唯一归还责任转移给返回的内核直接映射虚拟首地址裸指针。
    ///
    /// 主要供内核全局堆分配器（`GlobalAlloc` 大块直通路径）将所有权编码为裸指针交给 `alloc` 库持有；
    /// 接收方必须妥善记住分配时的原始 `order`，并在使用完毕后恰好调用一次 [`Self::from_raw`] 恢复并释放。
    pub(crate) fn into_raw(self) -> *mut u8 {
        let pages = ManuallyDrop::new(self);
        arch::phys_to_virt(pages.phys_addr()) as *mut u8
    }

    /// 从此前由 [`Self::into_raw`] 导出的内核直接映射虚拟首地址 `pointer` 与原始阶数 `order` 重建 [`OwnedPages`]（不触发重分配）。
    ///
    /// # Safety
    /// 1. **原指针与原阶匹配**：`pointer` 必须精确等于此前某次 `OwnedPages::into_raw` 返回的虚拟首地址，`order` 必须等于该块分配时的原始阶数。
    /// 2. **唯一恢复与静止状态**：该块自 `into_raw` 以来从未被恢复或释放过，当前不在任何伙伴系统、PCP 或 SLUB 空闲链表中，且通过 `pointer` 派生的一切读写引用均已结束；调用后严禁再次使用 `pointer` 访问内存或二次调用 `from_raw`。
    pub(crate) unsafe fn from_raw(pointer: *mut u8, order: usize) -> Self {
        let head = arch::virt_to_phys(pointer as usize) / PAGE_SIZE;
        // SAFETY:
        // 1. 调用方契约保证 `pointer` 是 `into_raw` 通过 `arch::phys_to_virt(head * PAGE_SIZE)` 生成的直接映射首地址，经 `virt_to_phys` 逆变换除以 `PAGE_SIZE` 可无损还原原始 `head` PFN，且 `order` 未变。
        // 2. 该块处于独占存活态且全部外部访问已终止，不存在重复归还凭据。
        let block = unsafe { Block::from_raw(head, order) };
        Self { block }
    }
}

impl Drop for OwnedPages {
    /// 析构时自动回收底层物理页：零阶（4 KiB）单页优先归入当前 CPU 本地缓存（PCP），高阶连续块直接按原阶归还伙伴系统。
    fn drop(&mut self) {
        if self.block.order() == 0 {
            // SAFETY:
            // 1. `OwnedPages` 不可克隆，独占持有 `self.block` 对应的零阶物理页唯一归还权，该页当前不在任何空闲链表中。
            // 2. Rust 所有权规则与裸指针使用契约保证进入 `drop` 时对该页的一切访问均已结束。
            unsafe { pcp::release(self.block.head_pfn()) };
        } else {
            // SAFETY:
            // 1. `self.block` 由伙伴系统成功摘出且保留了原始分配阶数 `order > 0`，本实例持有其唯一归还权。
            // 2. 进入 `drop` 时对该连续页块的一切 CPU 与硬件访问均已结束。
            unsafe { buddy::release(self.block.head_pfn(), self.block.order()) };
        }
    }
}

/// 非阻塞地分配一个阶数为 `order`（共 $2^{\text{order}}$ 个连续 4 KiB 页）的物理页块。
///
/// - **快速路径**：`order == 0` 时优先从当前 CPU 的单页缓存（PCP）弹出，未命中或遇锁竞争时回退至伙伴系统；`order > 0` 直接向伙伴系统申请。
/// - **失败语义**：若请求阶数超过 `MAX_ORDER` 或当前空闲连续内存不足，立即返回 `None`（不会睡眠等待，也不会在此层触发页面回收；需要回收重试的调用方应走 `mm::reclaim` 慢路径）。
/// - **锁外清零**：当 `zeroed == true` 时，函数在退出所有分配器自旋锁与关中断守卫之后将整块内存清零（在 `ktest` 并发中断测试开启时，还会在锁外清零窗口武装定时器中断以检验重入安全性）。
pub fn alloc_pages(order: usize, zeroed: bool) -> Option<OwnedPages> {
    #[cfg(feature = "ktest")]
    let mut test_window = crate::mm::ktest::AllocationWindow::enter();
    // Step 1: 按阶数分流向 PCP 或伙伴系统申请独占物理块凭据；锁与关中断守卫在此语句末尾即已全部释放。
    let block = if order == 0 {
        pcp::allocate()
    } else {
        buddy::allocate(order)
    }?;

    // Step 2: 封装为 RAII `OwnedPages`（即便后续发生异常展开也不会泄漏物理页），并在锁外按需清零整块内存。
    let mut pages = OwnedPages { block };
    if zeroed {
        let len = pages.len();
        #[cfg(feature = "ktest")]
        test_window.arm_timer();
        // SAFETY:
        // 1. `block` 刚从空闲分配器摘出并由局部变量 `pages` 唯一独占持有，`[as_mut_ptr(), as_mut_ptr() + len)` 覆盖真实连续 RAM，无任何并发读写别名。
        // 2. 启动阶段已为所有可分配物理 RAM 建立永久可写的一一对应直接映射，且此时未持有任何分配器自旋锁或关中断守卫。
        unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0, len) };
    }
    Some(pages)
}

/// 非阻塞地在半开物理字节地址窗口 `[physical_start, physical_end)` 内分配一个阶数为 `order` 的连续物理页块。
///
/// 窗口起点向上按页取整、终点向下按页取整，确保返回的整块物理页完全落在调用方指定的物理地址边界之内（常用于 32 位寻址受限的设备 DMA）。
/// 本函数直接查询伙伴系统而不排空每 CPU 缓存，且在释放伙伴系统锁之后按 `zeroed` 决定是否将整块清零。
pub fn alloc_pages_in(
    order: usize,
    zeroed: bool,
    physical_start: usize,
    physical_end: usize,
) -> Option<OwnedPages> {
    // Step 1: 将半开物理字节区间向内收缩转换为完整页的 PFN 窗口 `[start, end)`，并向伙伴系统定向申请。
    let start = physical_start.checked_add(PAGE_SIZE - 1)? / PAGE_SIZE;
    let end = physical_end / PAGE_SIZE;
    let block = buddy::allocate_in(order, start, end)?;

    // Step 2: 封装所有权句柄，并在伙伴系统锁释放后按需执行整块清零。
    let mut pages = OwnedPages { block };
    if zeroed {
        let len = pages.len();
        // SAFETY:
        // 1. `buddy::allocate_in` 已将整块连续 RAM 从空闲链表摘除并唯一移交给 `pages`，且 `ZONE` 自旋锁已经释放。
        // 2. `pages` 独占 `len` 字节的永久可写直接映射虚拟内存，写入全零安全且无别名冲突。
        unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0, len) };
    }
    Some(pages)
}
