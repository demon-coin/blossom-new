//! 内核固定大小小对象分配器（SLUB Allocator）。
//!
//! ## 模块职责与架构定位
//! 本模块为内核堆分配器（[`crate::mm::heap_allocator`]）提供 `8B ..= 4096B` 共 12 档固定步长
//! 小对象的高速分配与释放服务：
//! - **上游调用者**：`heap_allocator::HeapAllocator`（处理 `Layout` 命中 `SIZES` 类别的分配与释放），
//!   以及页面回收/测试路径调用的 `flush_all` / `shrink_all`。
//! - **下游依赖**：物理页分配器（`crate::mm::alloc_pages`）、物理页描述符表（`crate::mm::page::PageDesc`）
//!   与跨核同步调用（`crate::cpu::call`）。
//!
//! ## 核心设计与 Slab 状态机
//! 每个 Slab 由连续 `2^order` 张物理页组成，块内按固定步长 `stride` 切分为多个对象槽位。
//! 空闲对象复用其首部第 1 个 `usize` 字作为侵入式单向链表指针（`0` 表示链尾）：
//! - **本核独占空闲链（`CpuSlab::local`）**：仅由当前绑定该 `active` Slab 的 CPU 在本核 `CpuSlab`
//!   锁下直接弹出（`pop`）或压入（本核释放命中 `active` 时），无需获取 Slab 描述符锁。
//! - **跨核共享空闲链（`SlabState::shared`）**：存储在头页 `PageDesc::usage` 的 `SlabState` 中，
//!   接收远端 CPU 或非 `active` 状态下的对象释放，并在 Slab 被重新装载（`install`）为某核 `active`
//!   时整体移交给 `CpuSlab::local`。
//!
//! Slab 在生命周期中的位置状态（`SlabLocation`）流转如下：
//!
//! ```text
//!                        new_slab() (锁外申请物理页并初始化对象链)
//!                                    |
//!                                    v
//!                              [ Detached ]
//!                               /    |     \
//!              install()       /     |      \  put_node()
//!                             v      |       v
//!                    [ Active(cpu) ] |  [ NodePartial ] <--> [ Full ]
//!                        |           |         ^               |
//!        take_active() + |           |         | put_partials  | 满块首次释放对象
//!        deactivate()    +-----------+---------+               v
//!                                                      [ CpuPartial(cpu) ]
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 本模块涉及三类自旋锁：`CpuSlab` 锁（`Cache::cpu(owner)`）、节点部分满链表锁（`Cache::node`）
//! 和物理页描述符用途锁（`PageDesc::usage`）。合法的嵌套加锁顺序严格限定为：
//! 1. `CpuSlab` 自旋锁 ➔ `PageDesc::usage` 自旋锁（如 `install`、`put_cpu_partial`）；
//! 2. `CpuSlab` 自旋锁 ➔ `Cache::node` 自旋锁 ➔ `PageDesc::usage` 自旋锁（如 `allocate` 从节点链表取块）；
//! 3. `Cache::node` 自旋锁 ➔ `PageDesc::usage` 自旋锁（如 `put_node`、`remove_node`、`insert_node`、`deallocate` 慢路径）。
//!
//! **关键并发约束**：
//! - 严禁在持有 `PageDesc::usage` 锁时反向获取 `Cache::node` 或 `CpuSlab` 锁（`deallocate` 观察到
//!   `NodePartial` 时必须先释放 `usage` 锁，再按 `node ➔ usage` 顺序重取）；
//! - 物理页分配（`new_slab`）与物理页释放（`destroy`）必须在释放所有上述自旋锁且退出关中断作用域后执行。

use core::alloc::Layout;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicUsize, Ordering};
use spin::Once;

use crate::arch;
use crate::boot::MAX_CPUS;
use crate::cpu;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::page::{PageDesc, SlabLocation, SlabState};
use crate::mm::{alloc_pages, pages};
use crate::sync::{IrqSaveGuard, SpinLock};

/// SLUB 支持的 12 档固定对象步长（单位：字节）。
pub(crate) const SIZES: [usize; 12] = [8, 16, 32, 64, 96, 128, 192, 256, 512, 1024, 2048, 4096];
/// 表示空 PFN 链节点或无关联 Slab 的哨兵值。
pub(crate) const NONE: usize = usize::MAX;
/// 每个 CPU 在单个大小类别下最多缓存的 `CpuPartial` Slab 数量上限。
pub(crate) const CPU_PARTIAL_LIMIT: usize = 2;
/// 每个大小类别的 `NodePartial` 链表在常规释放路径下保留的最小非满（含全空）Slab 数量。
pub(crate) const MIN_PARTIAL: usize = 2;

/// 单个 Slab 后备物理页块允许使用的最大伙伴阶数（Order-3，即最多 8 张连续 4 KiB 页 = 32 KiB）。
const MAX_SLAB_ORDER: usize = 3;
/// 计算理想 Slab 阶数时，每个 CPU 规模因子对应的最少对象数基数。
const MIN_OBJECTS_PER_CPU_SCALE: usize = 4;
/// 计算理想 Slab 阶数时依次尝试的最大空间浪费分母（分别为 `1/16`、`1/8`、`1/4`、`1/2`）。
const WASTE_FRACTIONS: [usize; 4] = [16, 8, 4, 2];

/// 某一大小类别（`Cache`）的 Slab 物理页阶数与对象布局几何参数。
#[derive(Clone, Copy)]
pub(crate) struct Geometry {
    /// 单个对象槽位的字节步长（单位：字节，取自 [`SIZES`]）。
    pub(crate) stride: usize,
    /// 兼顾对象数量与尾端碎片率的首选分配物理页阶数（Order）。
    pub(crate) preferred_order: usize,
    /// 至少能容纳 1 个对象的最小物理页阶数（当首选阶分配失败时作为回退阶数）。
    pub(crate) minimum_order: usize,
    /// 在 `preferred_order` 下单个 Slab 可容纳的对象总数（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) objects: usize,
}

/// 单个 CPU 在某一大小类别下的本地 Slab 缓存控制结构，受对应 CPU 槽位的自旋锁保护。
struct CpuSlab {
    /// 当前绑定的活动（Active）Slab 头页帧号（PFN）；无活动 Slab 时为 [`NONE`]。
    active: usize,
    /// 当前活动 Slab 在本核独占空闲对象单向链表的首对象虚拟地址；为空时为 `0`。
    local: usize,
    /// 本核独占空闲对象单向链表的尾对象虚拟地址（用于解绑时 O(1) 拼接回 `shared` 链）；为空时为 `0`。
    local_tail: usize,
    /// 本核独占空闲对象链表中的空闲对象数量。
    local_count: usize,
    /// 本核缓存的部分满（`CpuPartial`）Slab 单向链表头页帧号（PFN）；为空时为 [`NONE`]。
    partial: usize,
    /// 本核当前缓存的 `CpuPartial` Slab 块数（上限为 [`CPU_PARTIAL_LIMIT`]）。
    partial_count: usize,
}

impl CpuSlab {
    /// 创建一个初始不持有任何活动 Slab 和部分满 Slab 的每 CPU 控制实例。
    const fn new() -> Self {
        Self {
            active: NONE,
            local: 0,
            local_tail: 0,
            local_count: 0,
            partial: NONE,
            partial_count: 0,
        }
    }

    /// 在持有本核 `CpuSlab` 自旋锁的前提下，从本核独占空闲链 `local` 头部弹出一个空闲对象。
    fn pop(&mut self) -> Option<NonNull<u8>> {
        let pointer = NonNull::new(self.local as *mut u8)?;
        // SAFETY:
        // 1. `self.local != 0` 说明本核空闲链非空，链上每个对象槽位首字均在初始化或释放时写入了下一跳地址（或 0）；
        // 2. 当前持有 `&mut self`（受本核 CpuSlab 自旋锁保护），远端 CPU 只会访问 `SlabState::shared` 链而不会触碰 `local` 链。
        self.local = unsafe { (self.local as *const usize).read() };
        self.local_count -= 1;
        if self.local == 0 {
            self.local_tail = 0;
        }
        Some(pointer)
    }

    /// 摘除当前 `CpuSlab` 的活动 Slab 控制字段并打包为 [`ActiveTicket`]，以便在释放本核锁后执行解绑合并。
    fn take_active(&mut self) -> ActiveTicket {
        let ticket = ActiveTicket {
            head: self.active,
            local: self.local,
            tail: self.local_tail,
            count: self.local_count,
        };
        self.active = NONE;
        self.local = 0;
        self.local_tail = 0;
        self.local_count = 0;
        ticket
    }

    /// 摘除当前 `CpuSlab` 的活动 Slab 与全部 `CpuPartial` 链表控制字段并打包为 [`FlushTicket`]。
    fn take_all(&mut self) -> FlushTicket {
        let ticket = FlushTicket {
            active: self.take_active(),
            partial: self.partial,
        };
        self.partial = NONE;
        self.partial_count = 0;
        ticket
    }
}

/// 从 `CpuSlab` 摘除的活动 Slab 状态转移票据，用于在释放 `CpuSlab` 锁后将剩余 `local` 空闲链归并回 `SlabState::shared`。
struct ActiveTicket {
    /// 被摘除的活动 Slab 头页帧号（PFN）；若原无活动 Slab 则为 [`NONE`]。
    head: usize,
    /// 摘除时剩余的本核独占空闲对象链表头虚拟地址；为空时为 `0`。
    local: usize,
    /// 摘除时剩余的本核独占空闲对象链表尾虚拟地址；为空时为 `0`。
    tail: usize,
    /// 摘除时剩余的本核独占空闲对象数量。
    count: usize,
}

/// 跨核排空（`flush_all`）时从单个 CPU 槽位一次性摘除的活动 Slab 与部分满 Slab 链表票据。
struct FlushTicket {
    /// 被摘除的活动 Slab 票据。
    active: ActiveTicket,
    /// 被摘除的本核 `CpuPartial` Slab 单向链表头页帧号（PFN）；为空时为 [`NONE`]。
    partial: usize,
}

impl FlushTicket {
    /// 创建一个空的排空票据，作为跨核同步调用前栈数组的初始值。
    const fn new() -> Self {
        Self {
            active: ActiveTicket {
                head: NONE,
                local: 0,
                tail: 0,
                count: 0,
            },
            partial: NONE,
        }
    }
}

/// 全局/节点级部分满（`NodePartial`）Slab 双向链表控制结构，受 `Cache::node` 自旋锁保护。
struct NodePartial {
    /// 双向链表头部的 Slab 头页帧号（PFN）；为空时为 [`NONE`]。
    head: usize,
    /// 双向链表尾部的 Slab 头页帧号（PFN）；为空时为 [`NONE`]。
    tail: usize,
    /// 当前位于 `NodePartial` 链表中的 Slab 块数量。
    count: usize,
}

/// 单个固定大小类别的全局 SLUB 缓存描述符。
struct Cache {
    /// 该大小类别的对象步长与物理页阶数几何参数。
    geometry: Geometry,
    /// 指向该大小类别下按 CPU 编号索引的 `CpuSlab` 自旋锁表。
    cpus: &'static [Once<SpinLock<CpuSlab>>; MAX_CPUS],
    /// 受自旋锁保护的节点级部分满（`NodePartial`）Slab 双向链表。
    node: SpinLock<NodePartial>,
    /// 当前该大小类别占用的后备物理页总数（单位：4 KiB 页，用于统计）。
    backing: AtomicUsize,
}

impl Cache {
    /// 获取指定在线 CPU 在当前大小类别下的 `CpuSlab` 自旋锁引用。
    fn cpu(&self, owner: usize) -> &SpinLock<CpuSlab> {
        self.cpus[owner].get().expect("SLUB CPU is not initialized")
    }
}

/// 所有大小类别在各个 CPU 上的 `CpuSlab` 静态存储表。
static CPU_SLABS: [[Once<SpinLock<CpuSlab>>; MAX_CPUS]; SIZES.len()] =
    [const { [const { Once::new() }; MAX_CPUS] }; SIZES.len()];

/// 全局 12 档固定大小 SLUB 缓存静态实例表。
static CACHES: Once<[Cache; SIZES.len()]> = Once::new();

/// 计算能够容纳 `bytes` 字节所需的最小物理页阶数（Order）。
fn order_for(bytes: usize) -> usize {
    let mut order = 0;
    while (PAGE_SIZE << order) < bytes {
        order += 1;
    }
    order
}

/// 根据对象步长 `stride` 和在线 CPU 数量计算单个 Slab 的首选阶数（`preferred_order`）与最小阶数（`minimum_order`）。
fn calculate_geometry(stride: usize) -> Geometry {
    let fls = usize::BITS as usize - cpu::count().leading_zeros() as usize;
    let minimum_order = order_for(stride);
    let min_objects =
        (MIN_OBJECTS_PER_CPU_SCALE * (fls + 1)).min((PAGE_SIZE << MAX_SLAB_ORDER) / stride);
    let min_order = order_for(min_objects * stride);
    let mut preferred_order = minimum_order;
    'fractions: for fraction in WASTE_FRACTIONS {
        for order in min_order..=MAX_SLAB_ORDER {
            let bytes = PAGE_SIZE << order;
            if bytes % stride <= bytes / fraction {
                preferred_order = order;
                break 'fractions;
            }
        }
    }
    Geometry {
        stride,
        preferred_order,
        minimum_order,
        #[cfg(feature = "ktest")]
        objects: (PAGE_SIZE << preferred_order) / stride,
    }
}

/// 初始化全局 SLUB 分配器的所有大小类别与在线 CPU 缓存槽位。
///
/// # Safety
/// 1. 仅允许由启动核（BSP）在关中断、从核（AP）尚未启动且尚无并发堆分配时调用一次；
/// 2. 物理页描述符表（`page`）、在线 CPU 表（`cpu`）及物理页分配器（`buddy`/`pcp`）必须已完成初始化；
/// 3. 所有可用物理内存的内核直接映射必须已建立且永久可写。
pub(crate) unsafe fn initialize() {
    assert!(CACHES.get().is_none(), "SLUB initialized twice");
    CACHES.call_once(|| {
        core::array::from_fn(|class| {
            let cpus = &CPU_SLABS[class];
            for owner in cpu::actual_set().iter() {
                cpus[owner].call_once(|| SpinLock::new(CpuSlab::new()));
            }
            Cache {
                geometry: calculate_geometry(SIZES[class]),
                cpus,
                node: SpinLock::new(NodePartial {
                    head: NONE,
                    tail: NONE,
                    count: 0,
                }),
                backing: AtomicUsize::new(0),
            }
        })
    });
}

/// 查找既能容纳 `layout.size()`、步长又能整除 `layout.align()`（且对齐不超过 `PAGE_SIZE`）的最小 SLUB 类别下标。
pub(crate) fn class_for(layout: Layout) -> Option<usize> {
    if PAGE_SIZE < layout.align() {
        return None;
    }
    SIZES
        .iter()
        .position(|size| layout.size() <= *size && size % layout.align() == 0)
}

/// 获取指定大小类别下标 `class` 对应的全局 [`Cache`] 引用。
fn cache(class: usize) -> &'static Cache {
    &CACHES.get().expect("SLUB is not initialized")[class]
}

/// 根据物理页帧号 `head` 获取对应的全局静态页描述符 [`PageDesc`]。
fn descriptor(head: usize) -> &'static PageDesc {
    pages().get(head).expect("slab PFN has no descriptor")
}

/// 遍历以 `pointer` 为首的侵入式空闲对象单向链表，返回最后一个对象的虚拟地址（若链表为空则返回 `0`）。
///
/// # Safety
/// 1. 链表上的所有空闲对象必须由调用方独占，无并发读写；
/// 2. 每个对象槽位首部第 1 个 `usize` 字已正确初始化并满足 `usize` 对齐，且链表以 `0` 正常终止；
/// 3. 调用方必须同时持有对应的 `CpuSlab` 自旋锁与该 Slab 的 `PageDesc::usage` 自旋锁。
unsafe fn chain_tail(mut pointer: usize) -> usize {
    let mut tail = 0;
    while pointer != 0 {
        tail = pointer;
        // SAFETY:
        // 1. 调用方契约保证 `pointer` 是空闲链表上有效且对齐的对象首地址，且整条链表以 0 结尾；
        // 2. 当前持有 CpuSlab 与 Slab 描述符锁，无并发修改该链表字。
        pointer = unsafe { (pointer as *const usize).read() };
    }
    tail
}

/// 在持有 `CpuSlab` 锁与 `SlabState` 锁的前提下，将一个部分满 Slab 装载为指定 CPU 的活动（`Active`）Slab。
///
/// 将 `state.shared` 空闲链整体移交给 `cpu.local`，并将 `state.inuse` 记为 `state.objects`（即视同本核预占全部剩余空闲对象）。
fn install(cpu: &mut CpuSlab, owner: usize, head: usize, state: &mut SlabState) {
    assert_eq!(cpu.active, NONE, "install overwrites an active slab");
    assert!(state.shared != 0, "partial slab has no free objects");
    cpu.active = head;
    cpu.local = state.shared;
    cpu.local_count = state.objects - state.inuse;
    // SAFETY:
    // 1. 当前同时持有 `cpu`（CpuSlab 锁）和 `state`（PageDesc::usage 锁），排除了对 `local` 和 `shared` 链的任何并发访问；
    // 2. `state.shared != 0` 且链上所有空闲对象首字均由 `new_slab` 或 `free_shared` 正确初始化并以 0 终止。
    cpu.local_tail = unsafe { chain_tail(state.shared) };
    state.shared = 0;
    state.inuse = state.objects;
    state.location = SlabLocation::Active(owner);
    state.next = NONE;
    state.previous = NONE;
}

/// 在持有 `Cache::node` 锁与目标 Slab 的 `PageDesc::usage` 锁的前提下，将 Slab `head` 从 `NodePartial` 双向链表中摘除并置为 `Detached`。
fn remove_node(node: &mut NodePartial, head: usize, state: &mut SlabState) {
    assert!(
        matches!(state.location, SlabLocation::NodePartial),
        "slab is not a node member"
    );
    if state.previous == NONE {
        assert_eq!(node.head, head, "node head differs from member");
        node.head = state.next;
    } else {
        descriptor(state.previous)
            .usage()
            .slab_mut()
            .expect("node predecessor vanished")
            .next = state.next;
    }
    if state.next == NONE {
        assert_eq!(node.tail, head, "node tail differs from member");
        node.tail = state.previous;
    } else {
        descriptor(state.next)
            .usage()
            .slab_mut()
            .expect("node successor vanished")
            .previous = state.previous;
    }
    node.count -= 1;
    state.next = NONE;
    state.previous = NONE;
    state.location = SlabLocation::Detached;
}

/// 在持有 `Cache::node` 锁与目标 Slab 的 `PageDesc::usage` 锁的前提下，将处于 `Detached` 状态的 Slab `head` 插入 `NodePartial` 双向链表尾部。
fn insert_node(node: &mut NodePartial, head: usize, state: &mut SlabState) {
    assert!(
        matches!(state.location, SlabLocation::Detached),
        "slab already has a list owner"
    );
    state.previous = node.tail;
    state.next = NONE;
    if node.tail == NONE {
        node.head = head;
    } else {
        descriptor(node.tail)
            .usage()
            .slab_mut()
            .expect("node tail vanished")
            .next = head;
    }
    node.tail = head;
    node.count += 1;
    state.location = SlabLocation::NodePartial;
}

/// 在不持有任何内存管理自旋锁且开中断的上下文中，清除 Slab 各页的 `slab_head` 标记、扣减统计计数并释放底层连续物理页。
fn destroy(state: SlabState) {
    let head = state.backing.phys_addr() / PAGE_SIZE;
    let count = 1 << state.backing.order();
    for pfn in head..head + count {
        descriptor(pfn).slab_head.store(NONE, Ordering::Release);
    }
    cache(state.cache)
        .backing
        .fetch_sub(count, Ordering::Relaxed);
    drop(state);
}

/// 将处于 `Detached` 状态的 Slab 归入全局 `NodePartial` 链表、标记为 `Full`，或在完全空闲且满足回收条件时直接销毁。
fn put_node(class: usize, head: usize, shrink: bool) {
    let discarded = {
        let mut node = cache(class).node.lock();
        let mut slot = descriptor(head).usage();
        let state = slot.slab_mut().expect("detached slab vanished");
        assert!(
            matches!(state.location, SlabLocation::Detached),
            "detached slab has another owner"
        );
        if state.inuse == 0 && (shrink || MIN_PARTIAL <= node.count) {
            slot.take_slab()
        } else {
            if state.shared == 0 {
                state.location = SlabLocation::Full;
            } else {
                insert_node(&mut node, head, state);
            }
            None
        }
    };

    if let Some(state) = discarded {
        destroy(state);
    }
}

/// 将从 `CpuSlab` 摘除的活动 Slab 票据（[`ActiveTicket`]）解绑：把本核剩余的 `local` 空闲链拼回 `SlabState::shared`，再交由 [`put_node`] 分类处置。
fn deactivate(class: usize, ticket: ActiveTicket) {
    if ticket.head == NONE {
        return;
    }
    {
        let mut slot = descriptor(ticket.head).usage();
        let state = slot.slab_mut().expect("active ticket vanished");
        assert!(
            matches!(state.location, SlabLocation::Active(_)),
            "ticket slab is not active"
        );
        if ticket.local != 0 {
            // SAFETY:
            // 1. `ticket` 独占持有刚从 `CpuSlab` 摘除的 `local` 空闲链，且 `ticket.tail` 指向该链表的最后一个空闲对象；
            // 2. 当前持有 `descriptor(ticket.head).usage()` 自旋锁，排除了对 `state.shared` 的并发读写，将 `state.shared` 接在 `ticket.tail` 之后安全有效。
            unsafe { (ticket.tail as *mut usize).write(state.shared) };
            state.shared = ticket.local;
            state.inuse -= ticket.count;
        }
        state.location = SlabLocation::Detached;
    }

    put_node(class, ticket.head, false);
}

/// 逐一遍历并消费从 `CpuSlab` 摘除的 `CpuPartial` 单向链表，将每个 Slab 状态置为 `Detached` 后交由 [`put_node`] 归入节点链表或销毁。
fn put_partials(class: usize, mut head: usize) {
    while head != NONE {
        let next = {
            let mut slot = descriptor(head).usage();
            let state = slot.slab_mut().expect("CPU partial ticket vanished");
            assert!(
                matches!(state.location, SlabLocation::CpuPartial(_)),
                "partial ticket lost its owner"
            );
            let next = state.next;
            state.next = NONE;
            state.previous = NONE;
            state.location = SlabLocation::Detached;
            next
        };
        put_node(class, head, false);
        head = next;
    }
}

/// 在锁外向伙伴系统申请连续物理页（优先 `preferred_order`，失败则回退 `minimum_order`），初始化对象空闲链表并在头页描述符中注册 [`SlabState`]。
fn new_slab(class: usize) -> Option<usize> {
    // Step 1: 在不持有任何 SLUB 锁的情况下申请后备连续物理页块。
    let cache = cache(class);
    let geometry = cache.geometry;
    let mut backing = alloc_pages(geometry.preferred_order, false).or_else(|| {
        if geometry.minimum_order < geometry.preferred_order {
            alloc_pages(geometry.minimum_order, false)
        } else {
            None
        }
    })?;

    // Step 2: 按固定步长 `stride` 将整块物理内存切分为侵入式空闲对象单向链表。
    let head = backing.phys_addr() / PAGE_SIZE;
    let count = 1 << backing.order();
    let objects = backing.len() / geometry.stride;
    let start = backing.as_mut_ptr().cast::<u8>() as usize;
    for index in 0..objects {
        let pointer = start + index * geometry.stride;
        let next = if index + 1 < objects {
            pointer + geometry.stride
        } else {
            0
        };
        // SAFETY:
        // 1. `backing` 是刚分配且独占的连续物理页块，`stride >= size_of::<usize>()` 且满足 `usize` 对齐；
        // 2. `pointer` 位于 `backing` 有效字节范围内，且该 Slab 尚未对外发布，写入每个对象槽位首字不会产生别名冲突。
        unsafe { (pointer as *mut usize).write(next) };
    }

    // Step 3: 在头页描述符中安装 `SlabState`（初始状态为 `Detached`），并为块内每一页设置 `slab_head` 反向索引。
    {
        let mut slot = descriptor(head).usage();
        slot.install_slab(SlabState {
            backing,
            cache: class,
            objects,
            inuse: 0,
            shared: start,
            location: SlabLocation::Detached,
            next: NONE,
            previous: NONE,
        });
    }

    for pfn in head..head + count {
        descriptor(pfn).slab_head.store(head, Ordering::Release);
    }
    cache.backing.fetch_add(count, Ordering::Relaxed);
    Some(head)
}

/// 从指定大小类别 `class` 中非阻塞地分配一个空闲对象。
///
/// 依次尝试：本核 `local` 空闲链 ➔ 当前活动 Slab 的 `shared` 链 ➔ 本核 `CpuPartial` 链 ➔
/// 全局 `NodePartial` 链 ➔ 锁外分配新 Slab。
pub(crate) fn allocate(class: usize) -> Option<NonNull<u8>> {
    let cache = cache(class);
    loop {
        let exhausted = {
            let _irq = IrqSaveGuard::new();
            let owner = cpu::current().id;
            let mut local = cache.cpu(owner).lock();
            // Step 1: 快路径——直接从本核 `local` 空闲链弹出对象。
            if let Some(pointer) = local.pop() {
                return Some(pointer);
            }
            // Step 2: 本核 `local` 已空，检查当前绑定的 `active` Slab 是否有跨核归还到 `shared` 链的对象。
            if local.active != NONE {
                let mut slot = descriptor(local.active).usage();
                let state = slot.slab_mut().expect("CPU active slab vanished");
                assert!(
                    state.location == SlabLocation::Active(owner),
                    "CPU active owner differs"
                );
                if state.shared != 0 {
                    let head = local.active;
                    local.active = NONE;
                    install(&mut local, owner, head, state);
                    return local.pop();
                }
                Some(local.take_active())
            } else {
                // Step 3: 当前无 `active` Slab，优先从本核 `CpuPartial` 链表中取出一个 Slab 装载为 `active`。
                if local.partial != NONE {
                    let head = local.partial;
                    let mut slot = descriptor(head).usage();
                    let state = slot.slab_mut().expect("CPU partial slab vanished");
                    assert!(
                        state.location == SlabLocation::CpuPartial(owner),
                        "CPU partial owner differs"
                    );
                    local.partial = state.next;
                    local.partial_count -= 1;
                    install(&mut local, owner, head, state);
                    return local.pop();
                }
                // Step 4: 本核 `CpuPartial` 也为空，尝试从全局 `NodePartial` 链表摘取一个 Slab 装载为 `active`。
                let mut node = cache.node.lock();
                if node.head != NONE {
                    let head = node.head;
                    let mut slot = descriptor(head).usage();
                    let state = slot.slab_mut().expect("node partial slab vanished");
                    remove_node(&mut node, head, state);
                    install(&mut local, owner, head, state);
                    return local.pop();
                }
                None
            }
        };

        // 若原 `active` Slab 的 `local` 和 `shared` 均已耗尽，在释放本核锁后将其解绑归入 `Full` / `NodePartial`。
        if let Some(ticket) = exhausted {
            deactivate(class, ticket);
            continue;
        }

        // Step 5: 所有现有缓存均无可用对象，在锁外申请新 Slab 并尝试装载到当前 CPU。
        let head = new_slab(class)?;
        {
            let _irq = IrqSaveGuard::new();
            let owner = cpu::current().id;
            let mut local = cache.cpu(owner).lock();
            if local.active == NONE {
                let mut slot = descriptor(head).usage();
                install(
                    &mut local,
                    owner,
                    head,
                    slot.slab_mut().expect("prepared slab vanished"),
                );
                return local.pop();
            }
        }
        // 锁外申请新 Slab 期间，中断或抢占可能已为本核安装了其他 `active` Slab；此时将新 Slab 交由 `put_node` 放入全局链表。
        put_node(class, head, false);
    }
}

/// 将刚从 `Full` 状态释放出第 1 个空闲对象（已提前标记为 `CpuPartial(owner)`）的 Slab 插入本核 `CpuPartial` 链表。
///
/// 若插入前本核 `CpuPartial` 数量已达到 [`CPU_PARTIAL_LIMIT`]，则先将原有的整条 `CpuPartial` 链表摘除并返回其头 PFN，
/// 由调用方在锁外通过 [`put_partials`] 归入全局 `NodePartial` 链表。调用方必须在关中断期间调用本函数。
fn put_cpu_partial(class: usize, owner: usize, head: usize) -> usize {
    let mut local = cache(class).cpu(owner).lock();
    let drained = if CPU_PARTIAL_LIMIT <= local.partial_count {
        let old = local.partial;
        local.partial = NONE;
        local.partial_count = 0;
        old
    } else {
        NONE
    };
    {
        let mut slot = descriptor(head).usage();
        let state = slot.slab_mut().expect("claimed CPU partial vanished");
        assert!(
            state.location == SlabLocation::CpuPartial(owner),
            "CPU partial claim changed"
        );
        state.next = local.partial;
        state.previous = NONE;
    }
    local.partial = head;
    local.partial_count += 1;
    drained
}

/// 在持有该 Slab 的 `PageDesc::usage` 自旋锁的前提下，将待释放对象 `pointer` 压入 `state.shared` 链表头部并递减 `state.inuse`。
///
/// # Safety
/// 1. `pointer` 必须是属于 `state` 对应 Slab 的有效已分配对象首地址，且调用方在此调用后交出该对象的唯一所有权；
/// 2. 对该对象的所有读写访问均已结束，且对象首部满足 `usize` 对齐并位于可写直接映射内存中；
/// 3. 调用方必须已持有该 Slab 头页的 `PageDesc::usage` 自旋锁。
unsafe fn free_shared(state: &mut SlabState, pointer: usize) {
    assert!(0 < state.inuse, "slab inuse underflow");
    // SAFETY:
    // 1. 调用方契约保证 `pointer` 为独占归还的有效对象首地址且满足 `usize` 对齐；
    // 2. 当前持有 `&mut SlabState`（受 PageDesc::usage 自旋锁保护），无其他执行流并发修改 `state.shared`。
    unsafe { (pointer as *mut usize).write(state.shared) };
    state.shared = pointer;
    state.inuse -= 1;
}

/// 释放一个由 SLUB 分配的对象：通过页描述符反向定位所属 Slab 头页，并根据 Slab 当前状态归入本核 `local` 链或跨核 `shared` 链。
///
/// # Safety
/// 1. `pointer` 必须是由本模块 [`allocate`] 返回且尚未释放的有效对象首地址；
/// 2. 调用方在此调用后交出该对象的唯一所有权，此前对该对象的所有读写访问必须已结束，且后续不得再次解引用或重复释放。
pub(crate) unsafe fn deallocate(pointer: NonNull<u8>) {
    // Step 1: 根据对象虚拟地址计算所在页 PFN，并通过 `slab_head` 找到所属 Slab 头页与大小类别 `class`。
    let address = pointer.as_ptr() as usize;
    let pfn = arch::virt_to_phys(address) / PAGE_SIZE;
    let head = descriptor(pfn).slab_head.load(Ordering::Acquire);
    assert!(head != NONE, "object has no slab head");
    let class = descriptor(head)
        .usage()
        .slab()
        .expect("live object has no slab state")
        .cache;

    let drained = {
        let _irq = IrqSaveGuard::new();
        let owner = cpu::current().id;
        // Step 2: 快路径——若该对象属于当前 CPU 正在使用的 `active` Slab，直接压入本核 `local` 空闲链。
        {
            let mut local = cache(class).cpu(owner).lock();
            if local.active == head {
                // SAFETY:
                // 1. 调用方契约保证 `address` 是已结束外部访问的唯一待释放对象，首字满足 `usize` 对齐且可写；
                // 2. 当前持有本核 `CpuSlab` 自旋锁，排除了对 `local` 链表的并发读写（远端 CPU 只会操作 `SlabState::shared`）。
                unsafe { (address as *mut usize).write(local.local) };
                if local.local == 0 {
                    local.local_tail = address;
                }
                local.local = address;
                local.local_count += 1;
                return;
            }
        }
        // Step 3: 慢路径——对象不属于本核 `active` Slab，获取 Slab 头页描述符锁检查其位置状态。
        let mut slot = descriptor(head).usage();
        let state = slot.slab_mut().expect("live object slab vanished");
        if matches!(state.location, SlabLocation::NodePartial) {
            // 该对象尚未归还，因此放开 `slot` 锁期间该 Slab 的 `inuse >= 1`，不会被销毁；
            // 必须先释放 `slot` 锁，再按 `Cache::node 锁 ➔ PageDesc::usage 锁` 的固定锁顺序重新加锁。
            drop(slot);
            let (discarded, claimed) = {
                let mut node = cache(class).node.lock();
                let mut slot = descriptor(head).usage();
                let state = slot.slab_mut().expect("live node object vanished");
                // SAFETY:
                // 1. 调用方仍持有该待释放对象的唯一所有权（此前仅观察了状态，尚未写入空闲链）；
                // 2. 当前已按 `node ➔ usage` 顺序持有 `state` 锁，独占修改 `state.shared` 与 `state.inuse`。
                unsafe { free_shared(state, address) };
                if matches!(state.location, SlabLocation::Full) {
                    // 在重取锁的窗口期内，该 Slab 可能被某核装载并耗尽成了 `Full`；本次释放使其再次出现空闲对象，需转入 `CpuPartial`。
                    state.location = SlabLocation::CpuPartial(owner);
                    (None, true)
                } else if matches!(state.location, SlabLocation::NodePartial)
                    && state.inuse == 0
                    && MIN_PARTIAL < node.count
                {
                    remove_node(&mut node, head, state);
                    (slot.take_slab(), false)
                } else {
                    (None, false)
                }
            };
            let drained = if claimed {
                put_cpu_partial(class, owner, head)
            } else {
                NONE
            };
            // 必须先退出关中断作用域（`_irq`），才能执行 `destroy` 向伙伴系统归还物理页。
            drop(_irq);
            if let Some(state) = discarded {
                destroy(state);
            }
            put_partials(class, drained);
            return;
        }
        // Step 4: 若 Slab 处于 `Active(other_cpu)`、`CpuPartial` 或 `Full`，直接在当前 Slab 锁下归入 `shared` 链；
        // 若原先为 `Full`，则本次释放是其首个空闲对象，将其认领为本核 `CpuPartial`。
        let was_full = matches!(state.location, SlabLocation::Full);
        // SAFETY:
        // 1. 调用方持有该待释放对象的唯一所有权，且对象尚未归还；
        // 2. 当前持有该 Slab 头页的 `PageDesc::usage` 锁（`slot`），保护对 `state.shared` 和 `state.inuse` 的写入。
        unsafe { free_shared(state, address) };
        if was_full {
            state.location = SlabLocation::CpuPartial(owner);
            drop(slot);
            put_cpu_partial(class, owner, head)
        } else {
            NONE
        }
    };

    put_partials(class, drained);
}

/// 跨核 IPI 回调函数：在目标 CPU 本地关中断上下文中，依次锁住 12 个大小类别的 `CpuSlab` 并摘除全部缓存控制字段写入 `argument`。
///
/// # Safety
/// 1. `argument` 必须指向调用方栈上有效且对齐的 `[FlushTicket; SIZES.len()]` 数组，并在 `cpu::call` 同步返回前保持存活；
/// 2. 在该回调执行期间，该数组由当前回调独占写入，无其他执行流并发读写。
unsafe fn detach_cpu(argument: *mut ()) {
    // SAFETY:
    // 1. `flush_all` 使用同步 `cpu::call` 等待本回调执行完毕，保证栈上 `tickets` 数组在回调期间始终有效；
    // 2. 每个目标 CPU 的回调针对各自独立的 `tickets` 数组实例独占写入，完成后通过 `cpu::call` 的 Release/Acquire 同步发布给调用方。
    let tickets = unsafe { &mut *argument.cast::<[FlushTicket; SIZES.len()]>() };
    let owner = cpu::current().id;
    for (class, ticket) in tickets.iter_mut().enumerate() {
        *ticket = cache(class).cpu(owner).lock().take_all();
    }
}

/// 遍历所有在线 CPU，通过跨核同步调用摘除各核的 `active` 与 `CpuPartial` 缓存，并在当前普通执行流中归并至全局节点链表或释放空 Slab。
///
/// 调用方不得持有任何内存管理自旋锁，且当前上下文必须允许处理跨核 IPI。
pub fn flush_all() {
    for owner in cpu::actual_set().iter() {
        let mut tickets = [const { FlushTicket::new() }; SIZES.len()];
        // SAFETY:
        // 1. `owner` 来自 `cpu::actual_set()`，为已初始化的有效在线 CPU 编号，且当前上下文未持有 MM 自旋锁；
        // 2. `tickets` 为当前栈帧上的有效数组，仅在 `cpu::call` 同步阻塞期间由 `detach_cpu` 独占写入，返回后才在下方读取。
        unsafe {
            cpu::call(
                owner,
                detach_cpu,
                (&mut tickets as *mut [FlushTicket; SIZES.len()]).cast(),
            )
        };
        for (class, ticket) in IntoIterator::into_iter(tickets).enumerate() {
            deactivate(class, ticket.active);
            put_partials(class, ticket.partial);
        }
    }
}

/// 先排空所有 CPU 的本地 SLUB 缓存（[`flush_all`]），再按各大小类别 `NodePartial` 的初始快照数量逐个检查并释放所有完全空闲（`inuse == 0`）的 Slab。
///
/// 仍有存活对象的部分满 Slab 会重新尾插回 `NodePartial` 链表；若此时无并发分配/释放活动，所有全空 Slab 的物理页将全部归还给物理页分配器。
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "内存及 I/O 测试回收空 slab 并比较原页账，内部 flush 和摘离票据随此入口保留"
    )
)]
pub fn shrink_all() {
    flush_all();
    for class in 0..SIZES.len() {
        let budget = cache(class).node.lock().count;
        for _ in 0..budget {
            let head = {
                let mut node = cache(class).node.lock();
                if node.head == NONE {
                    break;
                }
                let head = node.head;
                let mut slot = descriptor(head).usage();
                remove_node(
                    &mut node,
                    head,
                    slot.slab_mut().expect("shrink node slab vanished"),
                );
                head
            };
            put_node(class, head, true);
        }
    }
}

/// 汇总所有 12 个大小类别当前占用的后备物理页总数（单位：4 KiB 页；并发环境下为近似快照，静默状态下为精确总数）。
pub(crate) fn backing_pages() -> usize {
    (0..SIZES.len())
        .map(|class| cache(class).backing.load(Ordering::Relaxed))
        .sum()
}

/// 单个 Slab 头页描述符的状态快照（仅用于内核测试）。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) struct SlabSnapshot {
    /// 该 Slab 的头页帧号（PFN）。
    pub(crate) head_pfn: usize,
    /// 所属的大小类别下标（`0..SIZES.len()`）。
    pub(crate) cache: usize,
    /// 后备连续物理页块的伙伴阶数（Order）。
    pub(crate) order: usize,
    /// 该 Slab 可容纳的对象总数。
    pub(crate) objects: usize,
    /// 当前已分配（或由 `Active` CPU 预占）的对象计数。
    pub(crate) inuse: usize,
    /// 跨核共享空闲对象链表头虚拟地址；为空时为 `0`。
    pub(crate) shared: usize,
    /// 该 Slab 当前所处的位置状态（[`SlabLocation`]）。
    pub(crate) location: SlabLocation,
    /// 在 `CpuPartial` 或 `NodePartial` 链表中的后继 Slab 头页帧号；无后继时为 [`NONE`]。
    pub(crate) next: usize,
    /// 在 `NodePartial` 双向链表中的前驱 Slab 头页帧号；无前驱时为 [`NONE`]。
    pub(crate) previous: usize,
}

/// 单个 CPU 在某一大小类别下的 `CpuSlab` 状态快照（仅用于内核测试）。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) struct CpuSnapshot {
    /// 当前绑定的活动 Slab 头页帧号（PFN）；无活动块时为 [`NONE`]。
    pub(crate) active: usize,
    /// 本核独占空闲链 `local` 中的空闲对象数量。
    pub(crate) local_count: usize,
    /// 本核 `CpuPartial` 链表头部的 Slab 头页帧号（PFN）；为空时为 [`NONE`]。
    pub(crate) partial: usize,
    /// 本核 `CpuPartial` 链表中的 Slab 块数量。
    pub(crate) partial_count: usize,
}

/// 某一大小类别下全局 `NodePartial` 链表的状态快照（仅用于内核测试）。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) struct NodeSnapshot {
    /// `NodePartial` 双向链表头部的 Slab 头页帧号（PFN）；为空时为 [`NONE`]。
    pub(crate) partial: usize,
    /// `NodePartial` 双向链表中的 Slab 块数量。
    pub(crate) count: usize,
}

/// 读取指定大小类别 `class` 的缓存几何参数（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) fn geometry(class: usize) -> Geometry {
    cache(class).geometry
}

/// 根据存活对象指针 `pointer` 定位其所属 Slab 头页并读取状态快照（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) fn slab_snapshot(pointer: NonNull<u8>) -> SlabSnapshot {
    let pfn = arch::virt_to_phys(pointer.as_ptr() as usize) / PAGE_SIZE;
    let head = descriptor(pfn).slab_head.load(Ordering::Acquire);
    slab_head_snapshot(head).expect("snapshot live slab vanished")
}

/// 加锁读取指定头页帧号 `head` 的 Slab 状态快照；若该页当前不是 Slab 头页则返回 `None`（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) fn slab_head_snapshot(head: usize) -> Option<SlabSnapshot> {
    let slot = pages().get(head)?.usage();
    let state = slot.slab()?;
    Some(SlabSnapshot {
        head_pfn: head,
        cache: state.cache,
        order: state.backing.order(),
        objects: state.objects,
        inuse: state.inuse,
        shared: state.shared,
        location: state.location,
        next: state.next,
        previous: state.previous,
    })
}

/// 加锁读取指定大小类别 `class` 与指定 CPU `owner` 的 `CpuSlab` 状态快照（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) fn cpu_snapshot(class: usize, owner: usize) -> CpuSnapshot {
    let local = cache(class).cpu(owner).lock();
    CpuSnapshot {
        active: local.active,
        local_count: local.local_count,
        partial: local.partial,
        partial_count: local.partial_count,
    }
}

/// 加锁读取指定大小类别 `class` 的全局 `NodePartial` 状态快照（仅用于内核测试）。
#[cfg(feature = "ktest")]
pub(crate) fn node_snapshot(class: usize) -> NodeSnapshot {
    let node = cache(class).node.lock();
    NodeSnapshot {
        partial: node.head,
        count: node.count,
    }
}
