//! 每 CPU 零阶物理页缓存（Per-CPU Pageset，简称 PCP）。
//!
//! ## 模块职责与架构定位
//! 本模块位于单页分配入口（`crate::mm` 中的零阶页分配与 `OwnedPages` 析构）和伙伴系统
//! （`crate::mm::buddy`）之间，为每个在线 CPU 维护一个零阶（Order-0，即单张 4 KiB 物理页）
//! 空闲页缓存链表：
//! - **上游调用者**：`mm::alloc_pages`（当申请 `order == 0` 时）以及 `OwnedPages::drop`
//!   （当释放 `order == 0` 的物理页时）。
//! - **下游依赖**：`crate::mm::buddy`（批量申请 `allocate_batch` 与批量归还 `release_batch`）。
//!
//! ## 核心设计与链表内存布局
//! 为避免额外分配元数据，PCP 直接复用空闲物理页首部的第 1 个 `usize` 字（8 字节）作为侵入式
//! 单向链表的 `next` 指针，保存下一张空闲页的页帧号（PFN）：
//!
//! ```text
//! Pageset (Per-CPU)
//! +-------------------+
//! | count: N          |
//! | batch / high      |
//! | head: PFN_A       |
//! +---------+---------+
//!           |
//!           v
//!     phys_to_virt(PFN_A * PAGE_SIZE)      phys_to_virt(PFN_B * PAGE_SIZE)
//!     +----------------------------+       +----------------------------+
//!     | word[0]: PFN_B             | ----> | word[0]: NONE (usize::MAX) |
//!     | 其余字节：未定义空闲内容   |       | 其余字节：未定义空闲内容   |
//!     +----------------------------+       +----------------------------+
//! ```
//!
//! 水位与批量移动机制：
//! - 当本核缓存为空（`count == 0`）时，向伙伴系统一次批量补充最多 `batch` 张零阶页；
//! - 当本核缓存数量达到高水位（`count >= high`，其中 `high = 4 * batch`）时，向伙伴系统
//!   一次批量归还 `batch` 张零阶页；
//! - 当 `try_lock_with_irq` 因并发排空（`drain_all_pages`）未命中本核自旋锁时，直接退回
//!   伙伴系统进行单页分配或释放，不阻塞等待。
//!
//! ## 锁获取顺序与并发契约
//! - **锁层级顺序**：`Pageset` 自旋锁（`PAGESETS[cpu]`） ➔ 伙伴系统 Zone 自旋锁（`buddy` 内部锁）。
//! - **严禁反向加锁**：绝对禁止在持有伙伴系统 Zone 锁期间获取任何 CPU 的 `Pageset` 锁。
//! - **中断与抢占**：访问本核 `Pageset` 前必须先通过 `IrqSaveGuard` 关中断以固定当前 CPU，
//!   防止中断重入或核间迁移导致锁竞争与死锁。

use spin::Once;

use crate::arch;
use crate::boot::MAX_CPUS;
use crate::cpu;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::buddy::{self, Block, BATCH_CAPACITY};
use crate::sync::{IrqSaveGuard, SpinLock};

/// 空闲页单向链表的终止哨兵值（表示无下一张页）。
const NONE: usize = usize::MAX;

/// 计算单批移动页数时，按系统可管理总页数的 `1 / 2^10`（约千分之一）作为基准上限。
const MANAGED_BATCH_SHIFT: usize = 10;
/// 推导单批容量时允许参考的最大字节上限（1 MiB）。
const MAX_BATCH_BYTES: usize = 1 << 20;
/// 高水位线 `high` 相对于单批移动量 `batch` 的倍数（`high = 4 * batch`）。
const HIGH_BATCH_RATIO: usize = 4;

/// 单个 CPU 的零阶物理页缓存控制结构。
struct Pageset {
    /// 空闲页单向链表头部的物理页帧号（PFN）；为空时等于 [`NONE`]。
    head: usize,
    /// 当前缓存在链表中的零阶物理页数量（单位：页）。
    count: usize,
    /// 每次向伙伴系统批量补充（refill）或批量排空（drain）的页数步长（单位：页）。
    batch: usize,
    /// 触发自动批量归还的高水位线（单位：页）；当 `count >= high` 时归还 `batch` 页。
    high: usize,
}

impl Pageset {
    /// 创建一个初始为空的每 CPU 页缓存实例，后续只能在对应 CPU 的 `Pageset` 自旋锁保护下访问。
    fn new(batch: usize) -> Self {
        Self {
            head: NONE,
            count: 0,
            batch,
            high: HIGH_BATCH_RATIO * batch,
        }
    }

    /// 将一张已结束所有外部访问的零阶物理页压入本缓存链表头部。
    ///
    /// # Safety
    /// 1. `head` 必须是受伙伴系统管理的有效零阶物理页帧号（PFN），调用方在此调用后交出该页的唯一所有权，且对该页的所有读写访问均已结束；
    /// 2. 该页当前不得位于任何伙伴系统、PCP 或 SLUB 的空闲链表中；
    /// 3. 该物理页对应的内核直接映射虚拟地址必须有效、满足 `usize` 对齐且可写；
    /// 4. 调用方必须已持有当前 `Pageset` 的自旋锁。
    unsafe fn push(&mut self, head: usize) {
        let next = arch::phys_to_virt(head * PAGE_SIZE) as *mut usize;
        // SAFETY:
        // 1. 调用方契约保证 `head` 为独占的零阶物理页，页首直接映射地址对齐到 PAGE_SIZE（满足 usize 对齐）且可写；
        // 2. 当前持有 `&mut self`（受 Pageset 自旋锁保护），无其他执行流并发读写该页首字；
        // 3. 先将原链表头写入页首字，再更新 `self.head` 与 `self.count`，保证链表结构始终一致。
        unsafe { next.write(self.head) };
        self.head = head;
        self.count += 1;
    }

    /// 在持有 `Pageset` 自旋锁的前提下弹出链表头部的零阶页，并重建为独占的 [`Block`] 凭证。
    fn pop(&mut self) -> Option<Block> {
        if self.head == NONE {
            return None;
        }
        let head = self.head;
        let next = arch::phys_to_virt(head * PAGE_SIZE) as *const usize;
        // SAFETY:
        // 1. `self.head != NONE` 说明链表非空，且链表中的页均由 `push` 压入并在页首写入了有效的下一跳 PFN（或 `NONE`）；
        // 2. 当前持有 `&mut self`（受 Pageset 自旋锁保护），排除了并发弹出或修改该页首字的可能。
        self.head = unsafe { next.read() };
        self.count -= 1;
        // SAFETY:
        // 1. `head` 刚从当前 `Pageset` 链表中摘除，其零阶物理页的唯一所有权随之转移给返回的 `Block`；
        // 2. 页首链表字的读取已经完成，该页不再属于任何空闲链表且无其他并发访问者。
        Some(unsafe { Block::from_raw(head, 0) })
    }

    /// 按 `Pageset 锁 ➔ 伙伴系统 Zone 锁` 的顺序从伙伴系统批量申请最多 `batch` 张零阶页并压入本核链表。
    fn refill(&mut self) {
        let mut blocks = core::array::from_fn(|_| None);
        let allocated = buddy::allocate_batch(&mut blocks, self.batch);
        for slot in &mut blocks[..allocated] {
            let block = slot.take().expect("buddy batch count has no block");
            // SAFETY:
            // 1. `buddy::allocate_batch` 返回的每个 `Block` 均代表一张独占的零阶物理页，且尚未被外部读写；
            // 2. 当前持有 `&mut self`（受 Pageset 自旋锁保护），压入链表后 `Block` 值不再有其他副本（`Block` 无自动释放的 Drop）。
            unsafe { self.push(block.head_pfn()) };
        }
    }

    /// 在持有 `Pageset` 自旋锁的前提下从链表弹出最多 `min(count, batch, limit)` 张页并批量归还伙伴系统。
    fn drain_batch(&mut self, limit: usize) -> usize {
        let count = self.count.min(self.batch).min(limit);
        if count == 0 {
            return 0;
        }
        let mut blocks: [Option<Block>; BATCH_CAPACITY] = core::array::from_fn(|_| None);
        for slot in &mut blocks[..count] {
            *slot = Some(self.pop().expect("PCP count has no cached page"));
        }
        // SAFETY:
        // 1. `self.pop()` 已将这批零阶页从当前 PCP 链表中摘除，每张页都恢复为唯一的 `Block` 凭证；
        // 2. 这些页当前无任何并发读写访问，且调用遵守 `Pageset 锁 ➔ 伙伴系统 Zone 锁` 的锁层级顺序。
        unsafe { buddy::release_batch(&mut blocks) };
        count
    }
}

/// 全局每 CPU 零阶页缓存表，按逻辑 CPU 编号索引，在启动期为实际在线 CPU 逐槽初始化。
static PAGESETS: [Once<SpinLock<Pageset>>; MAX_CPUS] = [const { Once::new() }; MAX_CPUS];

/// 根据系统初始可管理总页数计算固定的单批移动页数 `batch`（范围在 `1..=BATCH_CAPACITY` 之间）。
fn batch_size(managed: usize) -> usize {
    let raw = ((managed >> MANAGED_BATCH_SHIFT).min(MAX_BATCH_BYTES / PAGE_SIZE)
        / HIGH_BATCH_RATIO)
        .clamp(1, BATCH_CAPACITY);
    let rounded = 1usize << (usize::BITS - 1 - (raw + raw / 2).leading_zeros());
    (rounded - 1).max(1)
}

/// 在伙伴系统初始化完成后，由启动核（BSP）为所有实际在线 CPU 初始化各自的 `Pageset` 槽位。
///
/// # Safety
/// 1. 伙伴系统（`buddy`）和在线 CPU 表（`cpu`）必须已完成初始化；
/// 2. 仅允许由 BSP 在关中断、从核（AP）尚未启动且尚无并发页分配/释放时调用一次；
/// 3. 所有可管理物理页对应的内核直接映射必须已建立且永久可写。
pub(crate) unsafe fn initialize() {
    assert!(PAGESETS[0].get().is_none(), "PCP initialized twice");
    let batch = batch_size(buddy::free_pages());
    for owner in cpu::actual_set().iter() {
        PAGESETS[owner].call_once(|| SpinLock::new(Pageset::new(batch)));
    }
}

/// 获取指定在线 CPU 的 `Pageset` 自旋锁引用；若未初始化则触发 panic。
fn pageset(owner: usize) -> &'static SpinLock<Pageset> {
    PAGESETS[owner].get().expect("PCP is not initialized")
}

/// 分配一张零阶物理页：优先关中断并尝试从当前 CPU 的 `Pageset` 获取，若锁竞争或缓存补充失败则回退到伙伴系统。
pub(crate) fn allocate() -> Option<Block> {
    let cached = {
        let irq = IrqSaveGuard::new();
        if let Some(mut pageset) = pageset(cpu::current().id).try_lock_with_irq(irq) {
            if pageset.count == 0 {
                pageset.refill();
            }
            pageset.pop()
        } else {
            None
        }
    };

    cached.or_else(|| buddy::allocate(0))
}

/// 释放一张零阶物理页：优先关中断并尝试压入当前 CPU 的 `Pageset`（达高水位时触发单批归还），若锁竞争则直接归还伙伴系统。
///
/// # Safety
/// 1. `head_pfn` 必须是受伙伴系统管理的有效零阶物理页帧号，调用方在此调用后交出该页的唯一所有权；
/// 2. 对该物理页的所有读写及硬件映射访问必须已彻底结束，且该页此前未被重复释放；
/// 3. 该页当前不得位于任何伙伴系统、PCP 或 SLUB 的空闲链表中，且内核直接映射保持有效可写。
pub(crate) unsafe fn release(head_pfn: usize) {
    let cached = {
        let irq = IrqSaveGuard::new();
        if let Some(mut pageset) = pageset(cpu::current().id).try_lock_with_irq(irq) {
            // SAFETY:
            // 1. 调用方契约保证 `head_pfn` 是独占、已结束访问且未在任何空闲链中的有效零阶物理页；
            // 2. `IrqSaveGuard` 保证了当前 CPU 不会被中断重入，`try_lock_with_irq` 成功获取了本核 `Pageset` 自旋锁。
            unsafe { pageset.push(head_pfn) };
            if pageset.high <= pageset.count {
                let batch = pageset.batch;
                pageset.drain_batch(batch);
            }
            true
        } else {
            false
        }
    };

    if !cached {
        // SAFETY:
        // 1. `try_lock_with_irq` 未获取锁，说明 `head_pfn` 未被压入 PCP 链表，唯一所有权仍在当前上下文；
        // 2. 此时已离开 IRQ 与 PCP 锁作用域，直接按零阶归还给伙伴系统符合锁顺序与唯一所有权契约。
        unsafe { buddy::release(head_pfn, 0) };
    }
}

/// 依次遍历所有在线 CPU，按初始快照数量分批将各核 PCP 中缓存的空闲页排空并归还给伙伴系统。
///
/// 每个批次之间会释放 `Pageset` 锁以缩短持锁时间；若此时无并发分配/释放活动，所有 PCP 缓存页将全部回到伙伴系统。
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "内存及 I/O 测试在参与者停稳后排空 PCP，比较原页归还账"
    )
)]
pub fn drain_all_pages() {
    for owner in cpu::actual_set().iter() {
        let pageset = pageset(owner);
        let mut remaining = pageset.lock().count;
        while 0 < remaining {
            let drained = pageset.lock().drain_batch(remaining);
            if drained == 0 {
                break;
            }
            remaining -= drained;
        }
    }
}

/// 依次加锁统计所有在线 CPU 当前缓存在 PCP 中的零阶页总数（并发环境下为近似快照，静默状态下为精确总数）。
pub(crate) fn cached_pages() -> usize {
    cpu::actual_set()
        .iter()
        .map(|owner| pageset(owner).lock().count)
        .sum()
}

/// 加锁读取指定 CPU 的 PCP 当前缓存页数（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) fn cpu_cached_pages(cpu: usize) -> usize {
    assert!(cpu < cpu::count(), "PCP CPU index is out of range");
    pageset(cpu).lock().count
}

/// 读取 0 号 CPU 的 `(batch, high)` 水位配置参数（仅用于内核测试验证水位边界）。
#[cfg(feature = "ktest")]
pub(crate) fn parameters() -> (usize, usize) {
    let pageset = pageset(0).lock();
    (pageset.batch, pageset.high)
}
