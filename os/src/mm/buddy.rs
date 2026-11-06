//! 单 Normal Zone 连续物理页伙伴系统（Buddy System Allocator）。
//!
//! ## 模块职责与架构定位
//! 本模块管理内核全部可动态分配的连续物理页帧，支持阶数 $0 \le \text{order} \le 10$
//!（即单次分配 $2^0 = 1$ 页 4 KiB 至 $2^{10} = 1024$ 页 4 MiB 的 $2^k$ 对齐连续物理块）。
//!
//! - **上游调用者**：
//!   - `mm::initialize` 调用 [`initialize`] 将启动期剩余空闲页一次性纳入伙伴系统；
//!   - `pcp`（每 CPU 单页缓存）调用 [`allocate_batch`] / [`release_batch`] 批量补充与排空零阶单页，并在锁竞争回退时调用 [`allocate`]；
//!   - `frame_allocator` 调用 [`allocate`]、[`allocate_in`] 与 [`release`] 处理多页连续分配、受限物理地址窗口分配（如 32 位 DMA 寻址限制）及释放；
//!   - `mm::stats` 调用 [`free_pages`] 查询伙伴系统当前空闲页总数。
//! - **下游依赖**：`page::PageMap` / `page::PageDesc`（将空闲块双向链表节点直接内嵌在块首 `PageDesc::buddy` 中，零额外内存开销）与 `sync::SpinLock`。
//!
//! ## 核心数据结构与伙伴拆分/合并机制
//! ```text
//! Zone { free: 总空闲页数 }
//!  heads[10] (4 MiB) ──► [PFN 1024 | prev:NONE, next:NONE, order:10]
//!  ...
//!  heads[2]  (16 KiB)──► [PFN 8  | prev:NONE, next:24, order:2] ◄──► [PFN 24 | prev:8, next:NONE, order:2]
//!  heads[1]  (8 KiB) ──► NONE
//!  heads[0]  (4 KiB) ──► [PFN 5  | prev:NONE, next:NONE, order:0]
//!
//! 伙伴地址配对公式: partner_pfn = head_pfn ^ (1 << order)
//! 可合并充要条件:
//!   1. order < MAX_ORDER (最高合并到 10 阶)
//!   2. memory.get(partner_pfn) 存在 (未落入稀疏内存空洞)
//!   3. desc.kind() == PageKind::Available (伙伴不是 Reserved / Metadata / Absent)
//!   4. desc.buddy.order == order (伙伴此刻确实作为完整同阶空闲块头停留在空闲链表中)
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - 全局 [`ZONE`] 由一把关中断自旋锁（`SpinLock<Zone>`）保护；所有对 `Zone::heads`、`Zone::free` 及
//!   `PageDesc::buddy` 字段的读写均严格限制在该锁临界区内。
//! - 锁层级：`pcp::Pageset` 自旋锁 $\rightarrow$ `buddy::ZONE` 自旋锁。持有 `ZONE` 锁期间**严禁**反过来获取任何 `Pageset`、`SLUB` 或 `PageDesc::usage_lock` 锁，也**严禁**执行整页清零等耗时操作。

#[cfg(feature = "ktest")]
use core::mem::MaybeUninit;
use spin::Once;

#[cfg(feature = "ktest")]
use crate::mm::page::PageDesc;
use crate::mm::page::{BuddyLink, PageKind, PageMap};
use crate::sync::SpinLock;

/// 伙伴系统支持的最大分配阶数（$2^{10} = 1024$ 页 = 4 MiB）。
pub(crate) const MAX_ORDER: usize = 10;
/// 每 CPU 单页缓存（PCP）与伙伴系统单次批量搬运零阶页的最大槽位数。
pub(crate) const BATCH_CAPACITY: usize = 8;
/// 空闲链表空指针及“非空闲块头”哨兵值。
const NONE: usize = usize::MAX;

/// 已从伙伴系统摘出的连续物理页块线性所有权凭据。
///
/// 注意：本类型是底层内部凭据，**不实现 `Drop` 自动归还**（以便在 `pcp` 链表压入或 `OwnedPages::into_raw` 转换时拆解为裸 PFN），
/// 上层必须将其封装为 `OwnedPages` 或显式调用 [`release`] / [`release_batch`] 消费，不得静默丢弃导致物理页泄漏。
pub(crate) struct Block {
    /// 连续物理页块的起始物理页帧号（满足 `head % (1 << order) == 0`）。
    head: usize,
    /// 分配时的块阶数（块包含 $2^{\text{order}}$ 个连续 4 KiB 物理页）。
    order: usize,
}

impl Block {
    /// 返回该连续物理页块的起始页帧号（PFN）；仅读取数值，不转移或复制归还所有权。
    pub(crate) fn head_pfn(&self) -> usize {
        self.head
    }

    /// 返回该连续物理页块分配时的阶数 `order`；释放归还时必须原样传回同一阶数。
    pub(crate) fn order(&self) -> usize {
        self.order
    }

    /// 从裸起始页帧号 `head` 与原始阶数 `order` 重建连续页块唯一归还凭据（不触碰伙伴系统链表）。
    ///
    /// # Safety
    /// 1. **来源与对齐真实性**：`head` 与 `order` 必须精确对应此前由伙伴系统分配出的同一个完整连续块。
    /// 2. **唯一所有权转移**：原先持有该块的凭据已被消费（如经 `OwnedPages::into_raw` 拆解或刚从 `pcp` 缓存链表弹出），系统中不存在第二份指向该块或其子页的有效归还凭据。
    /// 3. **脱离空闲链且访问静止**：该块从未被重复归还，当前不在伙伴系统、PCP 或 SLUB 的任何空闲链表中，且先前对该页内存的独占/裸指针读写在归还前均已结束。
    pub(crate) unsafe fn from_raw(head: usize, order: usize) -> Self {
        Self { head, order }
    }
}

/// 单个物理内存管理区（Normal Zone）的伙伴系统空闲链表与页计数状态。
struct Zone<'a> {
    /// 指向页描述符表的引用，用于按 PFN 索引块首 [`BuddyLink`] 及校验伙伴 [`PageKind`]。
    memory: &'a PageMap<'a>,
    /// 各阶（`0..=MAX_ORDER`）空闲块双向链表的首块头 PFN；对应阶无空闲块时为 [`NONE`]。
    heads: [usize; MAX_ORDER + 1],
    /// 当前停留在 `heads` 所有阶空闲链表中的 4 KiB 物理页总数。
    free: usize,
}

impl<'a> Zone<'a> {
    /// 创建尚未装填任何空闲页的空 [`Zone`]；后续所有读写操作均须在包裹它的自旋锁守卫下执行。
    ///
    /// # Safety
    /// `memory` 所管辖的全部 [`PageDesc`] 不得同时被任何其他 `Zone` 实例绑定，确保其 `PageDesc::buddy` 字段唯一归本 `Zone` 的锁域独占访问。
    unsafe fn empty(memory: &'a PageMap<'a>) -> Self {
        Self {
            memory,
            heads: [NONE; MAX_ORDER + 1],
            free: 0,
        }
    }

    /// 在持有本 `Zone` 锁的前提下，读取指定 `pfn` 描述符中的 [`BuddyLink`] 链表状态副本。
    fn link(&self, pfn: usize) -> BuddyLink {
        let desc = self.memory.get(pfn).expect("buddy PFN has no descriptor");
        // SAFETY:
        // 1. `Zone::empty` 的安全契约保证 `memory` 中所有描述符的 `buddy` 字段唯一归本 `Zone` 管辖，且调用方当前持有本 `Zone` 的自旋锁（`&self` 仅在锁守卫内可得）。
        // 2. `PageDesc` 在 `page::initialize`（或 `with_test_zone`）中已完成初始化且地址永久有效，`read()` 按值复制 `Copy` 类型 `BuddyLink`，不产生悬垂引用。
        unsafe { desc.buddy.get().read() }
    }

    /// 在持有本 `Zone` 锁的前提下，更新指定 `pfn` 描述符中的 [`BuddyLink`] 链表状态。
    fn set_link(&mut self, pfn: usize, link: BuddyLink) {
        let desc = self.memory.get(pfn).expect("buddy PFN has no descriptor");
        // SAFETY:
        // 1. `&mut self` 保证当前执行流独占持有本 `Zone` 自旋锁，无任何并发路径读取或写入任何描述符的 `buddy` 字段。
        // 2. `desc.buddy` 为已初始化的 `UnsafeCell<BuddyLink>`，与只读字段 `kind` 及受 `usage_lock` 保护的 `usage` 内存互不重叠。
        unsafe { desc.buddy.get().write(link) };
    }

    /// 将起始于 `head`、阶数为 `order` 的空闲块头插到 `heads[order]` 双向链表头部，并累加空闲页计数。
    fn insert(&mut self, head: usize, order: usize) {
        let next = self.heads[order];
        if next != NONE {
            let mut link = self.link(next);
            link.previous = head;
            self.set_link(next, link);
        }
        self.set_link(
            head,
            BuddyLink {
                order,
                previous: NONE,
                next,
            },
        );
        self.heads[order] = head;
        self.free += 1 << order;
    }

    /// 以 $O(1)$ 时间将起始于 `head`、阶数为 `order` 的空闲块从双向链表中摘除，清除其空闲头阶数标记并扣减空闲页计数。
    fn remove(&mut self, head: usize, order: usize) {
        let link = self.link(head);
        if link.previous == NONE {
            self.heads[order] = link.next;
        } else {
            let mut previous = self.link(link.previous);
            previous.next = link.next;
            self.set_link(link.previous, previous);
        }
        if link.next != NONE {
            let mut next = self.link(link.next);
            next.previous = link.previous;
            self.set_link(link.next, next);
        }
        self.set_link(
            head,
            BuddyLink {
                order: NONE,
                previous: NONE,
                next: NONE,
            },
        );
        self.free -= 1 << order;
    }

    /// 遍历 `PageMap::free_ranges()`，将每段连续空闲 PFN 按自然对齐（`head.trailing_zeros()`）与剩余长度贪心切分为最大合法阶块插入空闲链表。
    fn initialize(&mut self) {
        let memory = self.memory;
        for range in memory.free_ranges() {
            let mut head = range.start;
            while head < range.end {
                let mut order = (head.trailing_zeros() as usize).min(MAX_ORDER);
                while range.end - head < (1 << order) {
                    order -= 1;
                }
                self.insert(head, order);
                head += 1 << order;
            }
        }
    }

    /// 从伙伴系统中分配一个阶数为 `order` 的连续页块；若阶数超限或无足够大的连续块则返回 `None`。
    fn allocate(&mut self, order: usize) -> Option<Block> {
        if MAX_ORDER < order {
            return None;
        }

        // Step 1: 从目标阶 `order` 起向上查找第一个非空的空闲链表阶 `found`。
        let mut found = order;
        while found <= MAX_ORDER && self.heads[found] == NONE {
            found += 1;
        }
        if MAX_ORDER < found {
            return None;
        }

        // Step 2: 摘出高阶块，每次对半拆分并将右半伙伴挂回低一阶空闲链表，保留左半块直至降到目标 `order`。
        let head = self.heads[found];
        self.remove(head, found);
        while order < found {
            found -= 1;
            self.insert(head + (1 << found), found);
        }
        Some(Block { head, order })
    }

    /// 在半开 PFN 窗口 `[start, end)` 内分配一个完全落入该窗口、且满足 $2^{\text{order}}$ 自然对齐的连续页块。
    ///
    /// 用于受物理寻址范围限制的设备 DMA 缓冲区分配；若无满足窗口约束的空闲块则返回 `None`，不会破坏任何已有空闲链表状态。
    fn allocate_in(&mut self, order: usize, start: usize, end: usize) -> Option<Block> {
        if MAX_ORDER < order || end <= start {
            return None;
        }

        let count = 1usize << order;
        // Step 1: 从 `order` 到 `MAX_ORDER` 逐阶扫描空闲块，计算块内首个不小于 `start` 且按 `count` 对齐的候选子块起点 `first`。
        for found in order..=MAX_ORDER {
            let mut head = self.heads[found];
            while head != NONE {
                let first = head.max(start).checked_add(count - 1)? & !(count - 1);
                let block_end = head + (1usize << found);
                // Step 2: 若 `[first, first + count)` 同时落在请求窗口 `[start, end)` 与当前大块 `[head, block_end)` 内，则摘除该大块并定向拆分。
                if first < end && count <= end - first && first + count <= block_end {
                    self.remove(head, found);
                    let mut chosen = head;
                    let mut split = found;
                    // Step 3: 自顶向下按 `first` 落在左半还是右半，将不需要的另一半伙伴放回对应阶链表，精准切出目标子块。
                    while order < split {
                        split -= 1;
                        let right = chosen + (1usize << split);
                        if first < right {
                            self.insert(right, split);
                        } else {
                            self.insert(chosen, split);
                            chosen = right;
                        }
                    }
                    return Some(Block {
                        head: chosen,
                        order,
                    });
                }
                head = self.link(head).next;
            }
        }
        None
    }

    /// 归还起始于 `head`、阶数为 `order` 的连续页块，并循环与同阶空闲伙伴（Buddy）向上合并后插入对应阶链表。
    ///
    /// # Safety
    /// 1. 调用方必须拥有本 `Zone` 此前分配的 `[head, head + (1 << order))` 完整连续块的唯一归还权，且 `head` 满足 `1 << order` 对齐。
    /// 2. 对该物理页块的一切读写与硬件 DMA 访问均已彻底结束，且该块当前不在伙伴系统、PCP 或 SLUB 的任何空闲链表中（严禁双重释放）。
    unsafe fn release(&mut self, mut head: usize, mut order: usize) {
        // 循环检查异或配对伙伴：仅当伙伴存在描述符、属于普通可分配 RAM（`Available`）且正以相同 `order` 停留在空闲链表头时，才摘除伙伴并合并升阶。
        while order < MAX_ORDER {
            let partner = head ^ (1 << order);
            let Some(desc) = self.memory.get(partner) else {
                break;
            };
            if desc.kind() != PageKind::Available || self.link(partner).order != order {
                break;
            }
            self.remove(partner, order);
            head = head.min(partner);
            order += 1;
        }
        self.insert(head, order);
    }
}

/// 全局唯一的 Normal Zone 伙伴系统实例，启动初始化后由自旋锁保护多核并发访问。
static ZONE: Once<SpinLock<Zone<'static>>> = Once::new();

/// `ktest` 单元测试专用的静态页描述符池（避免在内核栈上构造 1024 项大数组导致栈溢出）。
/// 该池受独立自旋锁保护，仅用于验证伙伴拆分/合并纯算法，不映射真实物理页，也不混入全局生产内存池。
#[cfg(feature = "ktest")]
static TEST_DESCRIPTORS: SpinLock<[MaybeUninit<PageDesc>; 1 << MAX_ORDER]> =
    SpinLock::new([const { MaybeUninit::uninit() }; 1 << MAX_ORDER]);

/// 在启动早期初始化全局伙伴系统，一次性接管 `memory.free_ranges()` 中的全部空闲物理页。
///
/// # Safety
/// 1. `memory` 及其引用的全部 [`PageDesc`] 必须已完成永久初始化并具备 `'static` 生命周期。
/// 2. 只能由 BSP 在关中断、AP 未唤醒且无并发页分配/释放的启动阶段调用一次。
/// 3. `memory.free_ranges()` 中的所有页面尚未移交给任何其他拥有者，且描述符的 `buddy` 字段无其他 `Zone` 访问。
pub(crate) unsafe fn initialize(memory: &'static PageMap<'static>) {
    assert!(ZONE.get().is_none(), "buddy initialized twice");
    ZONE.call_once(|| {
        // SAFETY: 调用方契约保证 `memory` 为全局唯一永久发布的 `PageMap`，此前从未创建过绑定该 `memory` 的其他 `Zone`。
        let zone = SpinLock::new(unsafe { Zone::empty(memory) });
        zone.lock().initialize();
        zone
    });
}

/// 非阻塞地从伙伴系统申请一个阶数为 `order` 的连续物理页块。
///
/// 调用上下文：任意不可睡眠或普通内核上下文均可调用（内部 `SpinLock` 自动关中断保护）；
/// 函数在返回前已释放 `ZONE` 锁，不在锁临界区内执行页面清零。
pub(crate) fn allocate(order: usize) -> Option<Block> {
    ZONE.get()
        .expect("buddy is not initialized")
        .lock()
        .allocate(order)
}

/// 非阻塞地在半开 PFN 区间 `[start, end)` 内申请一个阶数为 `order` 的对齐连续物理页块。
///
/// 直接在 `ZONE` 锁内定位满足窗口的空闲块并按需拆分，不会绕道每 CPU 缓存或反复盲目分配再释放。
pub(crate) fn allocate_in(order: usize, start: usize, end: usize) -> Option<Block> {
    ZONE.get()
        .expect("buddy is not initialized")
        .lock()
        .allocate_in(order, start, end)
}

/// 在单次 `ZONE` 加锁临界区内批量申请最多 `count` 个独立的零阶（4 KiB）单页，供每 CPU 缓存（PCP）回填使用。
///
/// 调用前 `count` 不得超过 [`BATCH_CAPACITY`] 且 `output` 数组槽位必须全部为 `None`；
/// 返回实际成功填入 `output[..allocated]` 的单页数量（内存不足时允许少于 `count`，各单页之间不要求物理相邻）。
pub(crate) fn allocate_batch(output: &mut [Option<Block>; BATCH_CAPACITY], count: usize) -> usize {
    assert!(
        count <= BATCH_CAPACITY,
        "buddy batch exceeds fixed capacity"
    );
    assert!(
        output.iter().all(Option::is_none),
        "buddy batch output is occupied"
    );

    let mut zone = ZONE.get().expect("buddy is not initialized").lock();
    let mut allocated = 0;
    for slot in &mut output[..count] {
        let Some(block) = zone.allocate(0) else {
            break;
        };
        *slot = Some(block);
        allocated += 1;
    }
    allocated
}

/// 在单次 `ZONE` 加锁临界区内消费 `blocks` 数组中的所有非空零阶单页凭据，将其批量归还伙伴系统并尝试伙伴合并。
///
/// # Safety
/// 1. `blocks` 中每一个 `Some(block)` 必须是由本伙伴系统分配且互不相同的零阶页（`order == 0`）唯一归还凭据。
/// 2. 这些页面已从所属 PCP 链表中摘除，所有内存读写均已结束，当前不属于任何伙伴系统、PCP 或 SLUB 空闲链表。
pub(crate) unsafe fn release_batch(blocks: &mut [Option<Block>; BATCH_CAPACITY]) {
    assert!(
        blocks.iter().flatten().all(|block| block.order == 0),
        "buddy batch contains a nonzero order"
    );

    let mut zone = ZONE.get().expect("buddy is not initialized").lock();
    for slot in blocks {
        if let Some(block) = slot.take() {
            // SAFETY:
            // 1. 上方 `assert!` 已确认 `block.order == 0`，且调用方契约保证每个槽位持有已脱离 PCP 链表且无任何活跃访问的唯一零阶页归还权。
            // 2. `slot.take()` 就地消费该槽凭据防止重复释放，且当前持有全局唯一的 `ZONE` 锁守卫。
            unsafe { zone.release(block.head, 0) };
        }
    }
}

/// 将起始页帧号为 `head_pfn`、阶数为 `order` 的完整连续物理页块归还给全局伙伴系统。
///
/// # Safety
/// 1. 调用方必须交出此前由 [`allocate`] 或 [`allocate_in`] 返回的同一完整块（`head_pfn` 与原始 `order` 完全一致）的唯一归还所有权。
/// 2. 对该块内所有页面的CPU读写与设备 DMA 访问必须已彻底结束，该块尚未被释放过，且当前不在任何伙伴系统、PCP 或 SLUB 空闲链表中。
pub(crate) unsafe fn release(head_pfn: usize, order: usize) {
    // SAFETY: 调用方契约保证了 `(head_pfn, order)` 对应由全局 `ZONE` 分配且已结束一切访问的唯一完整块，直接获取 `ZONE` 锁执行合并归还安全。
    unsafe {
        ZONE.get()
            .expect("buddy is not initialized")
            .lock()
            .release(head_pfn, order)
    };
}

/// 获取伙伴系统当前各阶空闲链表中的总空闲页数快照（不含暂存在每 CPU 缓存 PCP 中的空闲单页）。
pub(crate) fn free_pages() -> usize {
    ZONE.get().expect("buddy is not initialized").lock().free
}

/// `ktest` 隔离伙伴系统测试夹具：包装受控的局部 [`Zone`] 并跟踪至多 16 个已分配块句柄，析构时自动回收全部未释放块。
#[cfg(feature = "ktest")]
pub(crate) struct TestZone<'a, 'b> {
    /// 被测的隔离局部 `Zone` 可变借用。
    zone: &'a mut Zone<'b>,
    /// 记录测试过程中分配出的存活 [`Block`] 凭据槽位表。
    blocks: [Option<Block>; 16],
}

#[cfg(feature = "ktest")]
impl TestZone<'_, '_> {
    /// 调用 [`Zone::allocate`] 并将结果存入 `blocks[index]`（该槽位此前必须为空），返回是否分配成功。
    pub(crate) fn allocate(&mut self, index: usize, order: usize) -> bool {
        assert!(self.blocks[index].is_none());
        self.blocks[index] = self.zone.allocate(order);
        self.blocks[index].is_some()
    }

    /// 调用 [`Zone::allocate_in`] 在窗口 `[start, end)` 内分配并存入 `blocks[index]`，返回是否分配成功。
    pub(crate) fn allocate_in(
        &mut self,
        index: usize,
        order: usize,
        start: usize,
        end: usize,
    ) -> bool {
        assert!(self.blocks[index].is_none());
        self.blocks[index] = self.zone.allocate_in(order, start, end);
        self.blocks[index].is_some()
    }

    /// 查询 `blocks[index]` 中存活块的 `(head_pfn, order)` 元组，不消费其归还所有权。
    pub(crate) fn block(&self, index: usize) -> (usize, usize) {
        let block = self.blocks[index].as_ref().unwrap();
        (block.head, block.order)
    }

    /// 消费 `blocks[index]` 中的存活块凭据，将其归还至被测 `Zone` 以验证伙伴合并逻辑。
    pub(crate) fn release(&mut self, index: usize) {
        let block = self.blocks[index].take().unwrap();
        // SAFETY: `blocks[index]` 保存的是从同一 `self.zone` 分配且未曾释放的唯一 `Block` 凭据，且单元测试仅操作描述符元数据而不解引用底层物理页。
        unsafe { self.zone.release(block.head, block.order) };
    }

    /// 返回被测 `Zone` 当前的空闲页总数。
    pub(crate) fn free(&self) -> usize {
        self.zone.free
    }
}

#[cfg(feature = "ktest")]
impl Drop for TestZone<'_, '_> {
    /// 析构夹具时自动将被测 `blocks` 表中残留的所有块归还局部 `Zone`（即使测试断言失败发生栈展开也能保持账务干净）。
    fn drop(&mut self) {
        for slot in &mut self.blocks {
            if let Some(block) = slot.take() {
                // SAFETY: 非空槽位持有由 `self.zone` 分配的唯一存活 `Block`，`take()` 确保只归还一次，且测试全程不访问任何物理内存页。
                unsafe { self.zone.release(block.head, block.order) };
            }
        }
    }
}

/// 构造隔离的模拟 [`PageMap`] 与局部 [`Zone`]，在其上执行伙伴算法测试闭包 `body`。
///
/// 当 `fragmented` 为 `false` 时构造一整块完整的 `[0, 1 << MAX_ORDER)` 连续空闲区；
/// 当 `fragmented` 为 `true` 时注入 `Reserved` 页、`Absent` 页与跨空洞高段（PFN 65536..65538），用于测试禁止跨空洞/跨保留页非法合并的边界条件。
#[cfg(feature = "ktest")]
pub(crate) fn with_test_zone(fragmented: bool, body: impl FnOnce(&mut TestZone<'_, '_>)) {
    use crate::mm::bootmem::PageRange;
    // Step 1: 锁定静态测试描述符池，按 `fragmented` 模式逐槽初始化模拟 `PageDesc`。
    let mut storage = TEST_DESCRIPTORS.lock();
    for (pfn, slot) in storage.iter_mut().enumerate() {
        slot.write(PageDesc::new(if fragmented {
            match pfn {
                3 | 11 => PageKind::Reserved,
                7 | 15 => PageKind::Absent,
                _ => PageKind::Available,
            }
        } else {
            PageKind::Available
        }));
    }

    // Step 2: 组装带中间空洞段（`sections[1] == None`）的模拟 `PageMap` 与空闲区间表。
    // SAFETY:
    // 1. 上方循环已完成 `storage` 全部 `1 << MAX_ORDER` 个元素的有效 `PageDesc` 初始化，且 `MaybeUninit<PageDesc>` 与 `PageDesc` 内存布局完全一致。
    // 2. `storage` 自旋锁守卫存活至本函数末尾，局部 `memory`、`zone` 与 `owner` 均在 `storage` 解锁前析构完毕，不存在悬垂借用或并发别名。
    let descriptors =
        unsafe { core::slice::from_raw_parts(storage.as_ptr().cast::<PageDesc>(), storage.len()) };
    let high = [const { PageDesc::new(PageKind::Available) }; 2];
    let sections = [Some(descriptors), None, Some(high.as_slice())];
    let whole = [PageRange {
        start: 0,
        end: 1 << MAX_ORDER,
    }];
    let pieces = [
        PageRange { start: 0, end: 3 },
        PageRange { start: 4, end: 7 },
        PageRange { start: 8, end: 11 },
        PageRange { start: 12, end: 15 },
        PageRange {
            start: 65536,
            end: 65538,
        },
    ];
    let memory = PageMap::test_map(&sections, if fragmented { &pieces } else { &whole });
    // Step 3: 在局部 `PageMap` 上初始化隔离的 `Zone` 并运行测试闭包。
    // SAFETY: `memory` 引用的 `storage` 与栈数组 `high` 均为当前函数独占的隔离测试元数据，未绑定任何其他 `Zone`。
    let lock = SpinLock::new(unsafe { Zone::empty(&memory) });
    let mut zone = lock.lock();
    zone.initialize();
    let mut owner = TestZone {
        zone: &mut zone,
        blocks: [const { None }; 16],
    };
    body(&mut owner);
}
