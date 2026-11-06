//! 块 I/O 零堆分配预留内存池（Mempool）与元数据缓存基准槽位管理模块。
//!
//! ## 模块职责与架构定位
//! 在系统物理内存耗尽（OOM）执行紧急脏页回收与后台回写时，若块 I/O 请求的 `Arc<BioInner>` 控制块、
//! `Vec<MappedPages>` 载荷容器或元数据索引节点仍向全局堆（`Global`）动态申请内存，就会陷入
//! “回收内存需要发起回写 I/O -> 发起 I/O 需要申请堆内存 -> 堆内存耗尽等待回收”的死锁。
//! 本模块通过实现标准库 `core::alloc::Allocator` 接口的 `BioAllocator`，在读写卷启用前一次性向全局堆预分配
//! 固定数量的原始内存块与元数据物理页。进入就绪态（`ready = true`）后，回写与元数据路径完全在预留池内循环借还，绝不触碰全局堆。
//! - **上游调用者**：块请求核心与提交层（`crate::block::bio`、`crate::block::submit`）、元数据缓存与索引树
//!   （`crate::cache`、`crate::cache::index`）以及文件系统后台回写域（`crate::fs::background`、`crate::fs::writeback`）。
//! - **下游依赖**：Rust 全局分配器（`alloc::alloc::Global`）、物理页缓存句柄（`crate::mm::FolioRef`）、
//!   DMA 映射段（`crate::mm::dma::MappedPages`）以及任务等待队列（`crate::wait::WaitQueue`）。
//!
//! ## 核心设计与槽位布局
//!
//! ### 1. `BioReserve` 内部槽位几何结构
//! 每个 `BioReserve` 实例包含 **9 个 `BioSlot`**（8 个回写槽 + 1 个独立辅助槽）与 **`capacity` 个 `MetadataSlot`**：
//!
//! ```text
//!   BioReserve 预留池布局
//!  ┌─────────────────────────────────────────────────────────────────────────┐
//!  │ bios: [BioSlot; 9] (BIO_SLOTS = MAX_BATCH + 1 = 9)                      │
//!  │  ├─ Slot 0..8 (回写通道 WB):                                            │
//!  │  │    ├─ control: Allocation -> 容纳 ArcInner<BioInner>                 │
//!  │  │    └─ payload: Allocation -> 容纳 [MappedPages; 8] (WB_MAPPINGS = 8) │
//!  │  └─ Slot 8 (AUX_SLOT 独立辅助槽, 供元数据读入与 Flush 专用):            │
//!  │       ├─ control: Allocation -> 容纳 ArcInner<BioInner>                 │
//!  │       └─ payload: Allocation -> 容纳 [MappedPages; 1]                   │
//!  ├─────────────────────────────────────────────────────────────────────────┤
//!  │ metadata: Vec<MetadataSlot> (长度 = capacity)                           │
//!  │  └─ Slot 0..capacity:                                                   │
//!  │       ├─ folio:   FolioRef   -> 预留的 4KiB 元数据缓存基准物理页        │
//!  │       └─ control: Allocation -> 容纳元数据缓存树节点 ArcInner<Node>     │
//!  └─────────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ### 2. `BioSlot` 双层预订/租借与分配器归还生命周期
//! 预留槽位严格区分“业务通道预订（`booked`）”、“单次构造租借（`leased`）”与“底层 `Arc`/`Vec` 实际内存存活（`active`）”。
//! 即使 `Bio` 已发出完成通知，只要外部仍有 `Weak` 弱引用或 `BioCompletion` 尚未释放导致 `Allocator::deallocate` 未被调用，
//! `active` 就保持为 `true`，槽位绝不会被提前重用：
//!
//! ```text
//!   [ 空闲可重用: !booked && !leased && !control.active && !payload.active ]
//!         │
//!         ▼ acquire_wb_lane() 批量预订通道 (仅 WB 槽 0..8; AUX 槽直接进入 leased)
//!   [ 已预订: booked = true ]  (持有 BioLane 期间排斥其他回写波次抢占)
//!         │
//!         ▼ lane.lease_available() / try_lease() 租借单次构造权
//!   [ 构造中: leased = true ]
//!         │  1. lease.memory() 触发 payload.active = true
//!         │  2. Bio::try_new() 触发 control.active = true
//!         │  3. drop(lease)    恢复 leased = false
//!         ▼
//!   [ 在途运行: control.active / payload.active = true ]
//!         │  - Vec<MappedPages> 析构 -> deallocate(Payload) -> payload.active = false
//!         │  - 最后一个 Arc/Weak 释放 -> deallocate(Control) -> control.active = false
//!         │  - drop(BioLane)         -> booked = false
//!         ▼
//!   [ 回到空闲可重用状态，通过 notify_available() 唤醒等待队列 ]
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. **高层业务锁在后，通道预订在前**：回写任务必须在获取文件系统/Inode 业务锁之前调用 `acquire_wb_lane`；
//!    元数据读入任务必须在获取元数据页内容锁（`CONTENT_LOCKED`）之前调用 `acquire_aux`，防止持锁睡眠死锁。
//! 2. **`BioReserve::state` 自旋锁**：叶级短临界区自旋锁，仅保护槽位布尔状态位与指针元数据读写。
//!    准备阶段的 `Global.allocate`、析构阶段的 `Global.deallocate` 以及唤醒等待者的 `available.wake_all()`
//!    均严格在 `state` 自旋锁**外部**执行。

use alloc::alloc::Global;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::{AllocError, Allocator, Layout};
use core::pin::pin;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::block::bio::BioInner;
use crate::block::{BlockError, MAX_BATCH};
use crate::mm::dma::MappedPages;
use crate::mm::FolioRef;
use crate::sched;
use crate::sync::SpinLock;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// 单个回写（Writeback）父 `Bio` 在预留载荷槽中最多容纳的 `MappedPages` 映射段数（8 个）。
pub(crate) const WB_MAPPINGS: usize = MAX_BATCH;
/// `BioReserve` 管理的 `BioSlot` 总槽位数：8 个批量回写槽 + 1 个独立辅助（元数据读 / Flush）槽。
const BIO_SLOTS: usize = MAX_BATCH + 1;
/// 专供元数据单页读入与磁盘 `Flush` 请求使用的独立辅助槽下标（下标 `8`）。
const AUX_SLOT: usize = MAX_BATCH;

/// 承载块请求 DMA 映射段列表的动态数组类型，通过 `BioAllocator` 统一兼容普通堆分配与预留池分配。
pub(crate) type BioMemory = Vec<MappedPages, BioAllocator>;

/// 由全局堆（`Global`）一次性分配并由预留池独占释放权的原始内存块。
///
/// 预留池运行期间通过 `view()` 将裸字节区间借给 `Arc` 或 `Vec` 使用，但绝不转移全局堆的最终释放所有权；
/// 仅当整个 `BioReserve` 销毁时才在 `Drop` 中统一归还给 `Global`。
struct RawBlock {
    /// 全局堆返回的非空内存块起始指针。
    pointer: NonNull<u8>,
    /// 申请该内存块时使用的精确大小与对齐布局（归还给 `Global::deallocate` 时必须原样匹配）。
    layout: Layout,
}

// SAFETY:
// 1. `RawBlock` 的最终释放权由所属 `BioReserve` 唯一持有，不存在跨线程双重释放；
// 2. 内存块的借出与归还状态由 `BioReserve::state` 自旋锁同步，借出期间仅由对应的 `Arc`/`Vec` 按各自并发契约访问，`BioReserve` 自身从不解引用其中的业务数据。
unsafe impl Send for RawBlock {}

impl RawBlock {
    /// 在准备阶段（未持预留池自旋锁时）按指定 `layout` 向全局堆申请一块原始内存。
    fn allocate(layout: Layout) -> Result<Self, AllocError> {
        let memory = Global.allocate(layout)?;
        Ok(Self {
            pointer: memory.cast(),
            layout,
        })
    }

    /// 生成与预分配布局大小一致的裸字节切片指针供 `Allocator::allocate` 返回，不转移底层内存的最终释放权。
    fn view(&self) -> NonNull<[u8]> {
        NonNull::slice_from_raw_parts(self.pointer, self.layout.size())
    }
}

impl Drop for RawBlock {
    /// 在 `BioReserve` 整体销毁时，将预分配的原始内存块按原始布局归还给全局堆。
    fn drop(&mut self) {
        // SAFETY:
        // 1. `BioReserve::drop` 已断言所有槽位的 `booked`、`leased` 与 `active` 均为 `false`，证明借用该内存块的所有 `Arc`/`Vec` 均已彻底析构；
        // 2. `self.pointer` 与 `self.layout` 原样保存自 `RawBlock::allocate` 中的同一次 `Global.allocate` 调用，且无任何其他所有者会对其调用 `Global.deallocate`。
        unsafe { Global.deallocate(self.pointer, self.layout) };
    }
}

/// 单个预留原始内存块的分配状态记录。
///
/// 严格依据 `Allocator::deallocate` 的实际调用来清除 `active` 标志，而不以高层业务请求完成（Ready）推断内存已空闲。
struct Allocation {
    /// 准备阶段向全局堆申请好的原始内存块；尚未分配时为 `None`。
    block: Option<RawBlock>,
    /// 当前是否有尚未调用 `deallocate` 归还的 `Arc`（含 `Weak` 控制块）或 `Vec` 正在占用该内存块。
    active: bool,
    /// 标记该槽位在准备阶段是否已完成至少一次真实的“分配 -> 释放”预热循环（用于在 `mark_ready` 时校验精确 `Layout` 已被捕获）。
    retired_once: bool,
}

impl Allocation {
    /// 创建尚未绑定原始内存块、处于未占用状态的初始分配记录。
    const fn new() -> Self {
        Self {
            block: None,
            active: false,
            retired_once: false,
        }
    }

    /// 判断原始内存块是否已预分配完毕，且当前无任何存活的 `Arc`/`Vec` 占用。
    fn available(&self) -> bool {
        self.block.is_some() && !self.active
    }
}

/// 单个预留 `Bio` 槽位，成对管理 `Arc<BioInner>` 控制块与 `Vec<MappedPages>` 载荷数组的内存状态。
struct BioSlot {
    /// 承载 `Arc<BioInner, BioAllocator>` 引用计数头与 `BioInner` 本体的控制块分配记录。
    control: Allocation,
    /// 承载 `Vec<MappedPages, BioAllocator>` 连续元素缓冲区的载荷分配记录。
    payload: Allocation,
    /// 是否已被某个后台回写通道（`BioLane`）批量预订。
    booked: bool,
    /// 是否正处于单次 `Bio` 构造租借期（持有 `BioLease` 期间为 `true`）。
    leased: bool,
}

impl BioSlot {
    /// 创建无内存块且未预订、未租借的初始 `BioSlot`。
    const fn new() -> Self {
        Self {
            control: Allocation::new(),
            payload: Allocation::new(),
            booked: false,
            leased: false,
        }
    }

    /// 判断槽位的单次构造租约是否已结束，且控制块与载荷块均已归还就绪、可供构造下一个 `Bio`。
    fn reusable(&self) -> bool {
        !self.leased && self.control.available() && self.payload.available()
    }
}

/// 元数据缓存的单个预留槽位，绑定一张常驻基准物理页与一份缓存树节点（`Node`）控制块内存。
struct MetadataSlot {
    /// 预留给元数据缓存使用的 4KiB 物理页基准强引用（确保极端内存压力下元数据页不会被彻底回收耗尽）。
    folio: FolioRef,
    /// 承载元数据缓存索引节点 `Arc<Node, IndexAllocator>` 的控制块分配记录。
    control: Allocation,
    /// 是否正被 `MetadataLease` 租借用于初始化新缓存节点。
    leased: bool,
}

/// 受 `BioReserve::state` 自旋锁保护的预留池内部可变状态。
struct ReserveState {
    /// 固定 9 个 `Bio` 预留槽位（前 8 个用于数据/元数据回写批次，第 9 个用于元数据读入与 Flush）。
    bios: [BioSlot; BIO_SLOTS],
    /// 容量在初始化时固定的元数据页及索引节点控制块槽位数组。
    metadata: Vec<MetadataSlot>,
    /// 预留池是否已完成全部控制块精确布局预热并正式对外开放（置为 `true` 后禁止再向全局堆扩容或补申请）。
    ready: bool,
    /// 预留池自身的单调停止序号，用于在缓存源停用时使旧批次捕获的 `MetadataIoContext` 失效。
    stop_sequence: u64,
}

/// 面向块回写与元数据缓存的零堆分配预留内存池。
///
/// 仅持有底层原始内存块与基准 `FolioRef`，不反向持有上层 `Bio`、缓存节点、卷对象或工作队列引用，避免循环引用。
pub(crate) struct BioReserve {
    /// 保护槽位预订、租借及分配位状态的短临界区自旋锁。
    state: SpinLock<ReserveState>,
    /// 当回写槽或辅助槽暂时耗尽时，供任务睡眠等待槽位归还的等待队列。
    available: WaitQueue,
}

/// 元数据 I/O 操作入口捕获的预留池与所属挂载卷停止边界上下文。
///
/// 采用轻量级借用传递，用于在正式获取辅助槽或发起元数据页读入前校验所属预留池与挂载卷是否已进入停机退役阶段。
#[derive(Clone, Copy)]
pub(crate) struct MetadataIoContext<'a> {
    /// 目标元数据缓存绑定的预留内存池借用。
    reserve: &'a Arc<BioReserve>,
    /// 业务入口处采样的 `ReserveState::stop_sequence` 快照。
    boundary: u64,
    /// 可选的挂载卷级单调停止序列引用及其入口快照 `(sequence_ref, sampled_boundary)`，用于隔离同盘多卷的独立退役。
    view: Option<(&'a AtomicU64, u64)>,
}

impl<'a> MetadataIoContext<'a> {
    /// 使用已提前采样的预留池停止边界 `boundary` 构造上下文（供同一回写波次跨阶段延续初始边界，避免误采后续重激活的新序号）。
    pub(crate) fn from_boundary(reserve: &'a Arc<BioReserve>, boundary: u64) -> Self {
        Self {
            reserve,
            boundary,
            view: None,
        }
    }

    /// 绑定指定挂载卷的独立停止序列 `sequence` 与快照值 `boundary` 构造上下文，使同盘另一卷的退役不会误取消当前卷的元数据操作。
    pub(crate) fn for_view(
        reserve: &'a Arc<BioReserve>,
        sequence: &'a AtomicU64,
        boundary: u64,
    ) -> Self {
        Self {
            reserve,
            boundary: reserve.state.lock().stop_sequence,
            view: Some((sequence, boundary)),
        }
    }

    /// 通过 `Arc::ptr_eq` 校验当前上下文是否指向同一个 `BioReserve` 实例（防止跨缓存实例因序列号数值巧合相同而误判）。
    pub(crate) fn belongs_to(self, reserve: &Arc<BioReserve>) -> bool {
        Arc::ptr_eq(self.reserve, reserve)
    }

    /// 在发起新的元数据内容读入前校验预留池已就绪且停止边界未推进；若已停机则返回 `BlockError::NotReady`（不影响已准入的在途 I/O）。
    pub(crate) fn check(self) -> Result<(), BlockError> {
        let state = self.reserve.state.lock();
        if !state.ready
            || state.stop_sequence != self.boundary
            || self
                .view
                .is_some_and(|(sequence, boundary)| sequence.load(Ordering::Acquire) != boundary)
        {
            Err(BlockError::NotReady)
        } else {
            Ok(())
        }
    }
}

/// 标识单个 `BioSlot` 内部的两类独立预留内存块。
#[derive(Clone, Copy)]
pub(crate) enum BioBlock {
    /// 存放 `Arc<BioInner, BioAllocator>` 控制头与 `BioInner` 对象的内存块。
    Control,
    /// 存放 `Vec<MappedPages, BioAllocator>` 连续元素数组的内存块。
    Payload,
}

/// 块请求与载荷列表统一使用的内存分配器，同时支持常规全局堆与零堆分配预留槽。
#[derive(Clone)]
pub(crate) enum BioAllocator {
    /// 常规路径：直接转发给全局堆分配器 `Global`（允许按需分配与扩缩容）。
    Heap,
    /// 受限预留路径：绑定特定 `BioReserve` 的指定槽位与块类型（大小固定、禁止重分配扩容）。
    Reserved {
        /// 所属预留内存池的强引用（确保所有派生的 `Arc`/`Weak`/`Vec` 彻底析构前预留池不会提前释放底层 `RawBlock`）。
        reserve: Arc<BioReserve>,
        /// 绑定的 `BioSlot` 下标（`0..BIO_SLOTS`）。
        slot: usize,
        /// 本分配器实例负责的块种类（`Control` 或 `Payload`）。
        block: BioBlock,
    },
}

impl BioAllocator {
    /// 从载荷容器的分配器派生出指向同一个 `BioSlot` 的 `Control` 控制块分配器（若原为 `Heap` 则仍保持 `Heap`）。
    pub(super) fn control(&self) -> Self {
        match self {
            Self::Heap => Self::Heap,
            Self::Reserved { reserve, slot, .. } => Self::Reserved {
                reserve: reserve.clone(),
                slot: *slot,
                block: BioBlock::Control,
            },
        }
    }

    /// 返回当前分配器单次允许容纳的最大 `MappedPages` 段数（`Heap` 无上限；预留回写槽为 8，辅助槽为 1）。
    pub(super) fn mapping_limit(&self) -> usize {
        match self {
            Self::Heap => usize::MAX,
            Self::Reserved { slot, .. } => slot_mappings(*slot),
        }
    }
}

// SAFETY:
// 1. `Heap` 分支直接委托给标准库已验证满足 `Allocator` 契约的 `Global`；
// 2. `Reserved` 分支仅在槽位未被占用且请求 `Layout` 与预分配 `RawBlock` 完全一致时返回有效非空切片，并在 `reserve` 强引用保活下直到 `deallocate` 才解除占用。
unsafe impl Allocator for BioAllocator {
    /// 申请匹配 `layout` 的内存块：`Heap` 委托给全局堆，`Reserved` 直接消费当前已租借槽位中的预分配块（就绪态下容量固定，不再向全局堆动态申请）。
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => Global.allocate(layout),
            Self::Reserved {
                reserve,
                slot,
                block,
            } => reserve.allocate_bio(*slot, *block, layout),
        }
    }

    /// 归还此前由同一 `BioAllocator` 分配的内存块。
    ///
    /// # Safety
    /// 1. `pointer` 必须是由当前分配器实例（相同 `Heap` 或相同 `reserve`/`slot`/`block`）此前成功分配且尚未释放的起始地址；
    /// 2. `layout` 必须与分配该内存块时传入的 `Layout` 完全一致，且调用后不得再访问该内存区域。
    unsafe fn deallocate(&self, pointer: NonNull<u8>, layout: Layout) {
        match self {
            Self::Heap => {
                // SAFETY: 调用方遵守 `Allocator::deallocate` 契约，且当前分支为 `Self::Heap`，证明 `pointer` 与 `layout` 来自 `Global::allocate`。
                unsafe { Global.deallocate(pointer, layout) };
            }
            Self::Reserved {
                reserve,
                slot,
                block,
            } => {
                reserve.release_bio(*slot, *block, pointer, layout);
            }
        }
    }

    /// 扩容已分配内存块：`Heap` 转发给全局堆，`Reserved` 预留块容量固定，始终返回 `Err(AllocError)`。
    ///
    /// # Safety
    /// 调用方必须满足标准库 `Allocator::grow` 契约：`pointer` 与 `old_layout` 来自当前分配器的有效分配，且 `new_layout.size() >= old_layout.size()`。
    unsafe fn grow(
        &self,
        pointer: NonNull<u8>,
        old_layout: Layout,
        new_layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => {
                // SAFETY: 调用方已满足 `Allocator::grow` 的指针有效性、旧布局匹配与新尺寸不小于旧尺寸契约，直接委托给 `Global::grow`。
                unsafe { Global.grow(pointer, old_layout, new_layout) }
            }
            Self::Reserved { .. } => Err(AllocError),
        }
    }

    /// 扩容并将新增尾部字节清零：`Heap` 转发给全局堆，`Reserved` 预留块容量固定，始终返回 `Err(AllocError)`。
    ///
    /// # Safety
    /// 调用方必须满足标准库 `Allocator::grow_zeroed` 契约：`pointer` 与 `old_layout` 来自当前分配器的有效分配，且 `new_layout.size() >= old_layout.size()`。
    unsafe fn grow_zeroed(
        &self,
        pointer: NonNull<u8>,
        old_layout: Layout,
        new_layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => {
                // SAFETY: 调用方已满足 `Allocator::grow_zeroed` 的前置契约，且当前分支对应由 `Global` 管理的堆块。
                unsafe { Global.grow_zeroed(pointer, old_layout, new_layout) }
            }
            Self::Reserved { .. } => Err(AllocError),
        }
    }

    /// 缩容已分配内存块：`Heap` 转发给全局堆，`Reserved` 预留块保持原始固定布局不变，返回 `Err(AllocError)`。
    ///
    /// # Safety
    /// 调用方必须满足标准库 `Allocator::shrink` 契约：`pointer` 与 `old_layout` 来自当前分配器的有效分配，且 `new_layout.size() <= old_layout.size()`。
    unsafe fn shrink(
        &self,
        pointer: NonNull<u8>,
        old_layout: Layout,
        new_layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => {
                // SAFETY: 调用方已满足 `Allocator::shrink` 的前置契约，且当前分支对应由 `Global` 管理的堆块。
                unsafe { Global.shrink(pointer, old_layout, new_layout) }
            }
            Self::Reserved { .. } => Err(AllocError),
        }
    }
}

impl BioReserve {
    /// 在普通可睡眠任务上下文中创建预留池：准备 `capacity` 个元数据基准页槽位与 9 个 `BioSlot`（此时尚未标记为 `ready`，元数据节点控制块布局待后续 `Node::prime` 预热填充）。
    pub(crate) fn try_new(
        capacity: usize,
        hot: impl IntoIterator<Item = FolioRef>,
    ) -> Result<Arc<Self>, BlockError> {
        sched::assert_switchable();
        if capacity == 0 {
            return Err(BlockError::InvalidRange);
        }

        // Step 1: 构建容量为 capacity 的元数据槽数组，优先复用已有的热元数据页 hot，不足部分新分配元数据物理页。
        let mut metadata = Vec::new();
        metadata
            .try_reserve_exact(capacity)
            .map_err(|_| BlockError::NoMemory)?;
        let mut hot = hot.into_iter();
        for _ in 0..capacity {
            metadata.push(MetadataSlot {
                folio: match hot.next() {
                    Some(folio) => folio,
                    None => FolioRef::allocate_metadata().ok_or(BlockError::NoMemory)?,
                },
                control: Allocation::new(),
                leased: false,
            });
        }
        if hot.next().is_some() {
            return Err(BlockError::InvalidRange);
        }

        // Step 2: 分配 BioReserve 主体，初始处于未就绪（ready = false）准备阶段。
        let reserve = Arc::try_new(Self {
            state: SpinLock::new(ReserveState {
                bios: core::array::from_fn(|_| BioSlot::new()),
                metadata,
                ready: false,
                stop_sequence: 0,
            }),
            available: WaitQueue::new(),
        })
        .map_err(|_| BlockError::NoMemory)?;

        // Step 3: 逐槽试构造并释放一次 Arc<BioInner> 以捕获精确控制块布局，并预分配对应容量的 MappedPages 数组载荷块。
        for slot in 0..BIO_SLOTS {
            let allocator = BioAllocator::Reserved {
                reserve: reserve.clone(),
                slot,
                block: BioBlock::Control,
            };
            let control = Arc::<BioInner, BioAllocator>::try_new_uninit_in(allocator)
                .map_err(|_| BlockError::NoMemory)?;
            drop(control);
            let layout = Layout::array::<MappedPages>(slot_mappings(slot))
                .map_err(|_| BlockError::NoMemory)?;
            let payload = RawBlock::allocate(layout).map_err(|_| BlockError::NoMemory)?;
            reserve.state.lock().bios[slot].payload.block = Some(payload);
        }
        Ok(reserve)
    }

    /// 返回本预留池配置的元数据基准页槽位总数。
    #[cfg(feature = "ktest")]
    pub(crate) fn capacity(&self) -> usize {
        self.state.lock().metadata.len()
    }

    /// 克隆指定元数据槽位 `slot` 持有的基准物理页 `FolioRef`（仅增加引用计数，不分配新物理页）。
    pub(crate) fn metadata_folio(&self, slot: usize) -> FolioRef {
        self.state.lock().metadata[slot].folio.clone()
    }

    /// 按物理页地址查找 `folio` 在本预留池元数据槽数组中的下标；若不属于本池基准页则返回 `None`。
    pub(crate) fn metadata_slot_for(&self, folio: &FolioRef) -> Option<usize> {
        let physical = folio.phys_addr();
        self.state
            .lock()
            .metadata
            .iter()
            .position(|slot| slot.folio.phys_addr() == physical)
    }

    /// 判断给定 `folio` 是否属于本预留池常驻持有的元数据基准页。
    pub(crate) fn metadata_has_baseline(&self, folio: &FolioRef) -> bool {
        self.metadata_slot_for(folio).is_some()
    }

    /// 在全部 `BioSlot` 与 `MetadataSlot` 控制块完成预热分配且无任何在途租用后，将预留池切换为就绪态（`ready = true`），此后禁止任何堆补分配。
    pub(crate) fn mark_ready(&self) {
        let mut state = self.state.lock();
        assert!(!state.ready, "reserve published twice");
        assert!(state
            .bios
            .iter()
            .all(|slot| slot.reusable() && !slot.booked && slot.control.retired_once));
        assert!(state
            .metadata
            .iter()
            .all(|slot| slot.control.block.is_some() && slot.control.retired_once && !slot.leased));
        state.ready = true;
    }

    /// 为指定元数据槽位 `slot` 分配缓存树节点（`Node`）的控制块内存。
    ///
    /// 准备期（`!ready`）首次调用时在自旋锁外向全局堆申请并缓存精确 `layout`；就绪期（`ready`）仅在持有 `MetadataLease` 且控制块空闲时复用已有块。
    pub(crate) fn allocate_metadata_control(
        &self,
        slot: usize,
        layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        let building = {
            let mut state = self.state.lock();
            let building = !state.ready;
            let metadata = &mut state.metadata[slot];
            if !building && !metadata.leased {
                return Err(AllocError);
            }
            if let Some(result) = use_prepared(&mut metadata.control, layout)? {
                return Ok(result);
            }
            if !building {
                return Err(AllocError);
            }
            metadata.control.active = true;
            building
        };

        assert!(building);
        let allocation = RawBlock::allocate(layout);
        let mut state = self.state.lock();
        match allocation {
            Ok(block) => {
                let view = block.view();
                state.metadata[slot].control.block = Some(block);
                Ok(view)
            }
            Err(error) => {
                state.metadata[slot].control.active = false;
                Err(error)
            }
        }
    }

    /// 在元数据缓存节点（`Arc<Node>`）及其最后一个 `Weak` 弱引用彻底析构时，将控制块标记为空闲并唤醒等待队列。
    ///
    /// # Safety
    /// 1. `pointer` 与 `layout` 必须原样来自本预留池同一 `slot` 此前 `allocate_metadata_control` 返回且尚未归还的分配；
    /// 2. 调用后调用方不得再读写 `pointer` 指向的控制块内存。
    pub(crate) unsafe fn deallocate_metadata_control(
        &self,
        slot: usize,
        pointer: NonNull<u8>,
        layout: Layout,
    ) {
        {
            let mut state = self.state.lock();
            release_allocation(&mut state.metadata[slot].control, pointer, layout);
        }
        self.notify_available();
    }

    /// 非阻塞尝试租借一个空闲的元数据槽位（需同时满足未租借、节点控制块已归还、且基准物理页无外部活跃占用）；若全部忙碌则立即返回 `BlockError::NoMemory`，避免持缓存引用睡眠自锁。
    pub(crate) fn try_metadata_slot(self: &Arc<Self>) -> Result<MetadataLease, BlockError> {
        let mut state = self.state.lock();
        if !state.ready {
            return Err(BlockError::NotReady);
        }
        let Some(slot) = state
            .metadata
            .iter_mut()
            .enumerate()
            .find_map(|(index, slot)| {
                if !slot.leased
                    && slot.control.available()
                    && slot.folio.metadata_inventory_available()
                {
                    slot.leased = true;
                    Some(index)
                } else {
                    None
                }
            })
        else {
            return Err(BlockError::NoMemory);
        };
        Ok(MetadataLease {
            reserve: self.clone(),
            slot,
        })
    }

    /// 采样当前预留池的 `stop_sequence` 构造常规元数据 I/O 上下文。
    pub(crate) fn metadata_io_context(self: &Arc<Self>) -> MetadataIoContext<'_> {
        MetadataIoContext::from_boundary(self, self.state.lock().stop_sequence)
    }

    /// 在自旋锁外唤醒所有等待预留槽位或元数据资源归还的任务（被唤醒方仍须重新加锁复查条件）。
    pub(crate) fn notify_available(&self) {
        self.available.wake_all();
    }

    /// 在获取文件系统业务锁之前阻塞等待并批量预订至少 1 个（最多 `MAX_BATCH = 8` 个）空闲回写 `BioSlot`，返回通道守卫 `BioLane`；若 `stopped()` 为真则返回 `BlockError::QueueStopped`。
    pub(crate) fn acquire_wb_lane(
        self: &Arc<Self>,
        stopped: impl Fn() -> bool,
    ) -> Result<BioLane, BlockError> {
        self.wait_available(|| {
            if stopped() {
                return Err(BlockError::QueueStopped);
            }
            let mut state = self.state.lock();
            if !state.ready {
                return Err(BlockError::NotReady);
            }
            let mut slots = [0; MAX_BATCH];
            let mut count = 0;
            for (index, slot) in state.bios[..MAX_BATCH].iter_mut().enumerate() {
                if !slot.booked && slot.reusable() {
                    slot.booked = true;
                    slots[count] = index;
                    count += 1;
                }
            }
            Ok((count != 0).then(|| BioLane {
                reserve: self.clone(),
                slots,
                count,
            }))
        })
    }

    /// 在获取元数据页内容锁之前阻塞等待并租借专用的辅助槽（`AUX_SLOT`），用于发起元数据读入或 `Flush` 请求；等待期间若 `context` 停止边界失效则返回 `BlockError::NotReady`。
    pub(crate) fn acquire_aux(
        self: &Arc<Self>,
        context: MetadataIoContext<'_>,
    ) -> Result<BioLease, BlockError> {
        assert!(context.belongs_to(self), "foreign metadata IO context");
        self.wait_available(|| {
            let mut state = self.state.lock();
            if state.stop_sequence != context.boundary
                || context.view.is_some_and(|(sequence, boundary)| {
                    sequence.load(Ordering::Acquire) != boundary
                })
            {
                return Err(BlockError::NotReady);
            }
            if !state.ready {
                return Err(BlockError::NotReady);
            }
            let slot = &mut state.bios[AUX_SLOT];
            if slot.booked || !slot.reusable() {
                return Ok(None);
            }
            slot.leased = true;
            Ok(Some(BioLease {
                reserve: self.clone(),
                slot: AUX_SLOT,
            }))
        })
    }

    /// 使用栈上固定的等待节点在 `self.available` 队列上执行无堆分配的“检查 -> 注册等待 -> 二次复查 -> 睡眠挂起”循环，消除唤醒丢失窗口。
    fn wait_available<T>(
        &self,
        mut check: impl FnMut() -> Result<Option<T>, BlockError>,
    ) -> Result<T, BlockError> {
        sched::assert_switchable();
        loop {
            if let Some(value) = check()? {
                return Ok(value);
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `&self` 在整个循环期间保持 `BioReserve` 及内嵌 `self.available` 等待队列有效；
            // 2. `node` 已通过 `pin!` 固定在当前栈帧上不会移动，且每次循环迭代结束或提前 `return` 时 `_registration` 析构都会先将节点从队列安全摘除。
            let _registration = unsafe { self.available.register(node.as_mut()) };
            if let Some(value) = check()? {
                return Ok(value);
            }
            waiter.park();
        }
    }

    /// 从指定 `BioSlot` 的 `Control` 或 `Payload` 槽位分配匹配 `layout` 的内存块；仅在未就绪准备期（`!ready`）首次缺失时才在自旋锁外向全局堆申请。
    fn allocate_bio(
        &self,
        slot: usize,
        block: BioBlock,
        layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        {
            let mut state = self.state.lock();
            let building = !state.ready;
            let bio = &mut state.bios[slot];
            if !building && !bio.leased {
                return Err(AllocError);
            }
            let allocation = bio_allocation(bio, block);
            if let Some(result) = use_prepared(allocation, layout)? {
                return Ok(result);
            }
            if !building {
                return Err(AllocError);
            }
            allocation.active = true;
        }

        let allocation = RawBlock::allocate(layout);
        let mut state = self.state.lock();
        let record = bio_allocation(&mut state.bios[slot], block);
        match allocation {
            Ok(block) => {
                let view = block.view();
                record.block = Some(block);
                Ok(view)
            }
            Err(error) => {
                record.active = false;
                Err(error)
            }
        }
    }

    /// 在 `Arc<BioInner>`（含最后一个 `Weak`）或 `Vec<MappedPages>` 析构时归还对应槽位的控制块或载荷块占用，并唤醒等待者。
    fn release_bio(&self, slot: usize, block: BioBlock, pointer: NonNull<u8>, layout: Layout) {
        {
            let mut state = self.state.lock();
            release_allocation(
                bio_allocation(&mut state.bios[slot], block),
                pointer,
                layout,
            );
        }
        self.notify_available();
    }

    /// 获取预留池当前各槽位状态、存活分配计数及内存块精确布局的快照（供内核测试诊断使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn snapshot(&self) -> ReserveSnapshot {
        let state = self.state.lock();
        ReserveSnapshot {
            ready: state.ready,
            metadata_capacity: state.metadata.len(),
            free_wb: state.bios[..MAX_BATCH]
                .iter()
                .filter(|slot| !slot.booked && slot.reusable())
                .count(),
            aux_free: state.bios[AUX_SLOT].reusable(),
            booked: state.bios.iter().filter(|slot| slot.booked).count(),
            leased: state.bios.iter().filter(|slot| slot.leased).count(),
            control_live: state.bios.iter().filter(|slot| slot.control.active).count(),
            payload_live: state.bios.iter().filter(|slot| slot.payload.active).count(),
            bio_control: state.bios[0]
                .control
                .block
                .as_ref()
                .map(|block| block.layout),
            wb_payload: state.bios[0]
                .payload
                .block
                .as_ref()
                .map(|block| block.layout),
            aux_payload: state.bios[AUX_SLOT]
                .payload
                .block
                .as_ref()
                .map(|block| block.layout),
            node_control: state.metadata[0]
                .control
                .block
                .as_ref()
                .map(|block| block.layout),
            metadata_control_live: state
                .metadata
                .iter()
                .filter(|slot| slot.control.active)
                .count(),
        }
    }

    /// 返回当前正在 `available` 等待队列上睡眠等待预留资源的任务数量（供内核测试验证阻塞行为）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_waiting(&self) -> usize {
        self.available.test_members()
    }
}

impl Drop for BioReserve {
    /// 在最后一个 `Arc<BioReserve>` 强引用释放时校验所有 `BioSlot` 与 `MetadataSlot` 的预订、租借及分配器占用均已彻底清零，随后自动析构底层 `RawBlock` 与基准页。
    fn drop(&mut self) {
        let state = self.state.lock();
        assert!(state.bios.iter().all(|slot| !slot.booked
            && !slot.leased
            && !slot.control.active
            && !slot.payload.active));
        assert!(state
            .metadata
            .iter()
            .all(|slot| !slot.leased && !slot.control.active));
    }
}

/// 回写波次在进入文件系统业务锁前批量预订的 `BioSlot` 通道守卫（RAII）。
///
/// 仅持有已预订槽位的下标集合（将对应槽位置为 `booked = true`），本身不持有任何活跃的 `Bio`、`Vec` 或页面借用。
pub(crate) struct BioLane {
    /// 所属预留内存池的强引用。
    reserve: Arc<BioReserve>,
    /// 本次成功预订的回写槽位下标数组，仅前 `count` 项有效（每个下标范围 `0..MAX_BATCH`）。
    slots: [usize; MAX_BATCH],
    /// 本次通道实际预订到的槽位数量（范围 `1..=MAX_BATCH`）。
    count: usize,
}

impl BioLane {
    /// 返回本通道实际成功预订到的回写槽位数量（`1..=MAX_BATCH`）。
    pub(crate) fn len(&self) -> usize {
        self.count
    }

    /// 非阻塞尝试租借本通道内第 `index` 个槽位的单次构造权；当调用方手上已持有本波未提交的父 `Bio` 时，使用非阻塞探测避免为凑满批次而睡眠阻塞。
    pub(crate) fn try_lease(
        &self,
        index: usize,
        stopped: impl Fn() -> bool,
    ) -> Result<Option<BioLease>, BlockError> {
        assert!(index < self.count);
        if stopped() {
            return Err(BlockError::QueueStopped);
        }

        let slot = self.slots[index];
        let mut state = self.reserve.state.lock();
        let bio = &mut state.bios[slot];
        assert!(bio.booked);
        if !bio.reusable() {
            return Ok(None);
        }
        bio.leased = true;
        Ok(Some(BioLease {
            reserve: self.reserve.clone(),
            slot,
        }))
    }

    /// 在当前波次尚未构造任何未提交 `Bio` 时，阻塞等待本通道内任意一个已预订槽位完成底层内存归还并返回其 `(通道内序号, BioLease)`。
    pub(crate) fn lease_available(
        &self,
        stopped: impl Fn() -> bool,
    ) -> Result<(usize, BioLease), BlockError> {
        self.reserve.wait_available(|| {
            if stopped() {
                return Err(BlockError::QueueStopped);
            }
            let mut state = self.reserve.state.lock();
            for (index, slot) in self.slots[..self.count].iter().copied().enumerate() {
                let bio = &mut state.bios[slot];
                assert!(bio.booked);
                if bio.reusable() {
                    bio.leased = true;
                    return Ok(Some((
                        index,
                        BioLease {
                            reserve: self.reserve.clone(),
                            slot,
                        },
                    )));
                }
            }
            Ok(None)
        })
    }
}

impl Drop for BioLane {
    /// 释放通道对所有槽位的预订标记（`booked = false`）并唤醒等待者；若个别槽位的底层 `Arc`/`Vec` 尚未彻底析构，其 `active` 标志仍会阻止该槽被立即重用。
    fn drop(&mut self) {
        {
            let mut state = self.reserve.state.lock();
            for slot in &self.slots[..self.count] {
                assert!(state.bios[*slot].booked);
                state.bios[*slot].booked = false;
            }
        }
        self.reserve.notify_available();
    }
}

/// 单个 `BioSlot` 的单次 `Bio` 构造租约守卫（RAII）。
///
/// 在构造 `BioMemory` 与 `Bio` 的短暂窗口内持有（置 `leased = true`），构造完成后即可提前释放，底层内存继续由 `active` 标志保护至 `deallocate`。
pub(crate) struct BioLease {
    /// 所属预留内存池的强引用。
    reserve: Arc<BioReserve>,
    /// 当前租借的 `BioSlot` 槽位下标（`0..BIO_SLOTS`）。
    slot: usize,
}

impl BioLease {
    /// 使用当前租借槽位的 `Payload` 预留块创建并预分配固定容量的空 `BioMemory` 容器。
    pub(crate) fn memory(&self) -> Result<BioMemory, BlockError> {
        let mut memory = Vec::new_in(BioAllocator::Reserved {
            reserve: self.reserve.clone(),
            slot: self.slot,
            block: BioBlock::Payload,
        });
        memory
            .try_reserve_exact(slot_mappings(self.slot))
            .map_err(|_| BlockError::NoMemory)?;
        Ok(memory)
    }
}

impl Drop for BioLease {
    /// 解除槽位的单次构造租借标记（`leased = false`）并通知等待队列；已分配出的 `Arc<BioInner>` 与 `BioMemory` 在各自析构前仍保持 `active = true` 防止重用。
    fn drop(&mut self) {
        {
            let mut state = self.reserve.state.lock();
            assert!(state.bios[self.slot].leased);
            state.bios[self.slot].leased = false;
        }
        self.reserve.notify_available();
    }
}

/// 元数据缓存替换或新建索引节点（`Node`）前的单槽租约守卫（RAII）。
pub(crate) struct MetadataLease {
    /// 所属预留内存池的强引用。
    reserve: Arc<BioReserve>,
    /// 当前租借的 `MetadataSlot` 槽位下标（`0..capacity`）。
    slot: usize,
}

impl MetadataLease {
    /// 返回租借的元数据槽位下标，供 `IndexAllocator::Metadata` 绑定对应的节点控制块。
    pub(crate) fn slot(&self) -> usize {
        self.slot
    }

    /// 克隆并返回该元数据槽位绑定的基准物理页 `FolioRef`（调用方需在发布新缓存节点前独占重置页面状态）。
    pub(crate) fn folio(&self) -> FolioRef {
        self.reserve.metadata_folio(self.slot)
    }
}

impl Drop for MetadataLease {
    /// 解除元数据槽位的构造租借标记（`leased = false`）并唤醒等待队列；节点控制块与物理页的实际空闲状态仍由各自引用计数独立把关。
    fn drop(&mut self) {
        {
            let mut state = self.reserve.state.lock();
            assert!(state.metadata[self.slot].leased);
            state.metadata[self.slot].leased = false;
        }
        self.reserve.notify_available();
    }
}

/// 内核测试（`ktest`）读取的预留内存池状态与内存块布局诊断快照。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct ReserveSnapshot {
    /// 预留池是否已完成预热并进入就绪态（`ready`）。
    pub ready: bool,
    /// 元数据缓存基准槽位的总容量。
    pub metadata_capacity: usize,
    /// 当前未被预订（`!booked`）且立即可重用的回写 `BioSlot` 数量（范围 `0..=8`）。
    pub free_wb: usize,
    /// 独立辅助槽（`AUX_SLOT`）当前是否处于空闲可重用状态。
    pub aux_free: bool,
    /// 当前处于通道预订状态（`booked == true`）的 `BioSlot` 数量。
    pub booked: usize,
    /// 当前处于单次构造租借状态（`leased == true`）的 `BioSlot` 数量。
    pub leased: usize,
    /// 当前底层 `Arc<BioInner>` 控制块尚未归还（`control.active == true`）的 `BioSlot` 数量。
    pub control_live: usize,
    /// 当前底层 `Vec<MappedPages>` 载荷块尚未归还（`payload.active == true`）的 `BioSlot` 数量。
    pub payload_live: usize,
    /// 0 号槽捕获的 `Arc<BioInner>` 控制块内存布局（用于测试校验退役前后布局恒定）。
    #[expect(
        dead_code,
        reason = "后台退休测试的 layout-held 和 layout-after 整值 Debug 诊断输出 Bio 控制布局"
    )]
    pub bio_control: Option<Layout>,
    /// 0 号回写槽预分配的 8 段 `MappedPages` 载荷数组内存布局。
    #[expect(
        dead_code,
        reason = "后台退休测试的 layout-held 和 layout-after 整值 Debug 诊断输出 WB payload 布局"
    )]
    pub wb_payload: Option<Layout>,
    /// 独立辅助槽（`AUX_SLOT`）预分配的单段 `MappedPages` 载荷数组内存布局。
    #[expect(
        dead_code,
        reason = "后台退休测试的 layout-held 和 layout-after 整值 Debug 诊断输出 aux payload 布局"
    )]
    pub aux_payload: Option<Layout>,
    /// 0 号元数据槽捕获的缓存索引节点（`Arc<Node>`）控制块内存布局。
    #[expect(
        dead_code,
        reason = "后台退休测试的 layout-held 和 layout-after 整值 Debug 诊断输出元数据节点控制布局"
    )]
    pub node_control: Option<Layout>,
    /// 当前底层 `Arc<Node>` 控制块尚未归还（`control.active == true`）的元数据槽数量。
    pub metadata_control_live: usize,
}

/// 返回指定 `BioSlot` 下标对应的载荷数组最大映射段数：辅助槽 `AUX_SLOT` 为 `1`，其余回写槽均为 `WB_MAPPINGS`（`8`）。
fn slot_mappings(slot: usize) -> usize {
    if slot == AUX_SLOT {
        1
    } else {
        WB_MAPPINGS
    }
}

/// 根据 `BioBlock` 类型选取 `BioSlot` 内部对应的控制块或载荷块分配记录可变引用。
fn bio_allocation(slot: &mut BioSlot, block: BioBlock) -> &mut Allocation {
    match block {
        BioBlock::Control => &mut slot.control,
        BioBlock::Payload => &mut slot.payload,
    }
}

/// 尝试占用已预分配的 `RawBlock`：若当前已被占用或请求 `layout` 与预分配布局不一致则返回 `Err(AllocError)`；若尚未分配过内存块则返回 `Ok(None)`。
fn use_prepared(
    allocation: &mut Allocation,
    layout: Layout,
) -> Result<Option<NonNull<[u8]>>, AllocError> {
    if allocation.active {
        return Err(AllocError);
    }
    let Some(block) = &allocation.block else {
        return Ok(None);
    };
    if block.layout != layout {
        return Err(AllocError);
    }
    allocation.active = true;
    Ok(Some(block.view()))
}

/// 校验归还的 `pointer` 与 `layout` 与预分配块严格一致且当前确实处于占用状态，随后清除 `active` 占用位并标记已完成过预热回收（`retired_once = true`）。
fn release_allocation(allocation: &mut Allocation, pointer: NonNull<u8>, layout: Layout) {
    assert!(allocation.active, "reserve allocation ended twice");
    let block = allocation.block.as_ref().expect("reserve block missing");
    assert_eq!(block.pointer, pointer, "reserve allocation source mismatch");
    assert_eq!(block.layout, layout, "reserve allocation layout mismatch");
    allocation.active = false;
    allocation.retired_once = true;
}
