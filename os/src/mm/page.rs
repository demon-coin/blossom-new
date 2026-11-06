//! 稀疏分段物理页描述符表（Sparse Memory Page Map）与每页用途生命周期管理。
//!
//! ## 模块职责与架构定位
//! 本模块模仿经典的稀疏内存模型（Sparse Memory），将物理地址空间按固定 128 MiB（$2^{27}$ 字节 = 32768 页）
//! 划分为若干个段（`Section`）。仅为实际包含物理 RAM 的段分配连续的 [`PageDesc`] 描述符数组，
//! 使物理地址空间中存在巨大 MMIO 空洞的开发板无需为空洞浪费描述符内存，同时保持 $O(1)$ 的 PFN 下标定位。
//!
//! - **上游调用者**：
//!   - `mm::initialize` 调用 [`initialize`] 在启动期一次性构建 [`PageMap`]；
//!   - `buddy` 读写 [`PageDesc::buddy`] 双向链表节点与 [`PageDesc::kind`]；
//!   - `slub` 原子读写 [`PageDesc::slab_head`]，并通过 [`PageDesc::usage`] 管理 [`SlabState`]；
//!   - `folio`（页缓存）与 `anon`（匿名页）通过 [`PageDesc::usage`] 安装、查询与摘除各自的状态头。
//! - **下游依赖**：`bootmem`（启动期元数据页切分）、`arch::phys_to_virt`（直接映射转换）、`sync::SpinLock`。
//!
//! ## 核心内存布局与每页用途状态机
//!
//! ### 1. 稀疏分段描述符布局（`PageMap`）
//! ```text
//! PFN 拆分:  [   section_index = pfn / 32768   |  offset = pfn % 32768 (15 bits) ]
//!                        │                                      │
//!                        ▼                                      │
//! root 承载块: ┌──────────────────┬──────────────────┬───┐      │
//!              │ sections[0]:None │ sections[1]:Some │...│      │
//!              ├──────────────────┴─────────┬────────┴───┤      │
//!              │ free_ranges 只读快照       │ (ktest快照)│      │
//!              └────────────────────────────┴────────────┘      │
//!                                           │                   │
//!                                           ▼                   ▼
//! group 承载块:                    ┌──────────────────────────────────────┐
//!                                  │ PageDesc[0] .. PageDesc[offset] ..   │ (每段 32768 项)
//!                                  └──────────────────────────────────────┘
//! ```
//!
//! ### 2. 每页用途槽状态机（`PageUse`）
//! 每个已分配页块的首帧描述符在 `usage` 槽中记录当前高层用途，同一时刻至多处于一种角色：
//! ```text
//!                         install_slab()
//!            ┌──────────────────────────────────────► Slab(SlabState) ──┐
//!            │                                                          │ take_slab() (inuse==0 且已摘链)
//!            │            install_folio()                               ▼
//! PageUse::Empty ───────────────────────────────────► Folio(FolioState)─┼──► PageUse::Empty ──► 锁外 Drop OwnedPages
//!            │                                                          ▲    (归还 PCP 或 Buddy)
//!            │            install_anon()                                │ take_folio() (refs==0 且无锁/脏/回写)
//!            └──────────────────────────────────────► Anon(AnonState) ──┘
//!                                                       take_anon() (references==0)
//! ```
//! **固定地址（Pinning）关键设计**：`FolioState` 内部包含自引用性质的睡眠等待队列（`WaitQueue`）。
//! 为保证任务在等待队列上睡眠期间，其他核再次调用 `descriptor.usage()` 加锁修改引用计数时不会触发
//! Rust Stacked Borrows / Tree Borrows 别名违规，`usage_lock` 仅保护一个零大小令牌 `SpinLock<()>`，
//! 而非直接包裹 `SpinLock<PageUse>`；[`PageUseGuard`] 绝不在 `Folio` 存活期间对整个 `PageUse` 创建 `&mut` 独占引用。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - `PageDesc::buddy`：完全由全局唯一的 `buddy::Zone` 自旋锁保护，与 `usage_lock` 管理的字段正交。
//! - `PageDesc::usage_lock`：位于 SLUB `CpuSlab` / `NodePartial` 锁、页缓存索引锁、进程页表锁**内层**；
//!   持有 `usage_lock`（即持有 [`PageUseGuard`]）期间**严禁**睡眠，也**严禁**调用 `OwnedPages::drop` 触发 `pcp` 或 `buddy` 加锁。

use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::mem::size_of;
use core::slice;
use core::sync::atomic::AtomicUsize;

use crate::boot::MAX_MEMORY_REGIONS;
use crate::mm::anon::AnonState;
use crate::mm::bootmem::{BootMemory, BootStats, PageRange, PAGE_SHIFT, RANGE_CAPACITY};
use crate::mm::folio::FolioState;
use crate::mm::{OwnedPages, PAGE_SIZE};
use crate::sync::{SpinLock, SpinLockGuard};

/// 单个稀疏内存段（Section）覆盖的物理地址位数（$2^{27} = 128\text{ MiB}$）。
const SECTION_SIZE_BITS: usize = 27;
/// 单个稀疏内存段包含的 4 KiB 物理页帧数量（$2^{27 - 12} = 32768$ 页）。
const PAGES_PER_SECTION: usize = 1 << (SECTION_SIZE_BITS - PAGE_SHIFT);
/// 根索引表表项类型：`Some` 指向该 128 MiB 段对应的 `PAGES_PER_SECTION` 个描述符切片，`None` 表示纯物理空洞段。
type Section<'a> = Option<&'a [PageDesc]>;

/// 物理页帧在启动期确定的静态物理来源分类（初始化发布后终生不变）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PageKind {
    /// 该 PFN 落在某一个已分配描述符数组的 128 MiB 段内，但本身属于段内的非 RAM 空洞或边缘未对齐区域。
    Absent,
    /// 真实 RAM 页，但已被内核代码/数据段、启动栈、早期页表、DTB 或固件保留区占用，不可动态分配。
    Reserved,
    /// 真实 RAM 页，已在启动期划拨给 `PageMap` 根表或 `PageDesc` 数组作为永久元数据页。
    Metadata,
    /// 纳入伙伴系统管理的普通可分配 RAM 页（注意：该分类恒定不变，无论页面当前正被借出使用还是处于空闲链表中均为 `Available`）。
    Available,
}

/// 单个 4 KiB 物理页帧的永久元数据描述符。
///
/// 生产环境中所有实例均驻留在启动期分配的连续直接映射物理页上，地址终生固定不移动（含 `PhantomPinned`）。
pub(crate) struct PageDesc {
    /// 启动期确定的物理页来源类别，发布后只读。
    kind: PageKind,
    /// 伙伴系统空闲块双向链表节点与阶数标记；仅当页面作为空闲块头停留在伙伴系统中时有效，全程由所属 `buddy::Zone` 自旋锁独占保护。
    pub(crate) buddy: UnsafeCell<BuddyLink>,
    /// SLUB 多页复合块（Compound Slab）反向映射头 PFN：块内每一个子页均以 `Release` 原子写入块首 PFN，
    /// 对象释放路径通过 `Acquire` 原子读取即可由任意对象地址定位所属 Slab 头描述符；非 Slab 页时恒为 `usize::MAX`。
    pub(crate) slab_head: AtomicUsize,
    /// 保护 `usage` 用途槽状态转换与内部共享计数的独立自旋锁令牌（不直接包裹 `usage` 以免使 `FolioState` 内部固定地址引用失效）。
    usage_lock: SpinLock<()>,
    /// 当前物理页（或复合页块首帧）承载的高层用途状态（空、SLUB 块、页缓存 Folio 或匿名页）。
    usage: UnsafeCell<PageUse>,
    /// 标记描述符在内存中不可移动，支撑 `Pin<&'static PageDesc>` 及其内部 `FolioState::waiters` 等待队列的地址固定契约。
    _pinned: PhantomPinned,
}

/// 物理页帧（或连续复合页块首帧）的高层互斥用途及底层 [`OwnedPages`] 归还所有权容器。
enum PageUse {
    /// 未绑定高层子系统用途（页面可能处于空闲态，或由 `OwnedPages` 直接被页表/DMA/栈独占持有）。
    Empty,
    /// 作为 SLUB 分配器的一个 Slab 对象池头帧，内部持有整块物理页的归还所有权与空闲对象链表状态。
    Slab(SlabState),
    /// 作为文件系统页缓存或块设备元数据缓存页（Folio），内部持有物理页所有权、扇区状态与睡眠等待队列。
    Folio(FolioState),
    /// 作为用户态可共享/写时复制（CoW）的 4 KiB 匿名物理页，内部持有物理页所有权与映射引用计数。
    Anon(AnonState),
}

/// 持有单个 [`PageDesc`] 用途自旋锁（`usage_lock`）的临界区守卫。
///
/// 该守卫按需提供已核实变体的受控访问接口，避免直接暴露 `&mut PageUse` 导致存活中的固定 `FolioState` 共享引用失效。
pub(crate) struct PageUseGuard<'a> {
    /// 被加锁访问的目标物理页描述符引用。
    descriptor: &'a PageDesc,
    /// 底层 `usage_lock` 自旋锁守卫（持锁期间禁止睡眠，析构时自动解锁）。
    _lock: SpinLockGuard<'a, ()>,
}

impl PageUseGuard<'_> {
    /// 在持有 `usage_lock` 的前提下读取 `usage` 槽的共享引用，用于判别当前枚举变体标签。
    fn value(&self) -> &PageUse {
        // SAFETY:
        // 1. `self._lock` 持有当前 `PageDesc` 唯一的 `usage_lock` 自旋锁，排除了并发的 `install`、`take_*`、`slab_mut` 与 `anon_mut`。
        // 2. 当槽内为存活的 `Folio` 时，所有并发访问者仅修改 `FolioState` 内部的 `UnsafeCell`/`AtomicU8`/`WaitQueue`，绝不会创建外层 `&mut PageUse`。
        unsafe { &*self.descriptor.usage.get() }
    }

    /// 若当前页正作为 SLUB 头帧使用，则返回 [`SlabState`] 的共享引用；否则返回 `None`。
    pub(crate) fn slab(&self) -> Option<&SlabState> {
        match self.value() {
            PageUse::Slab(state) => Some(state),
            PageUse::Empty | PageUse::Folio(_) | PageUse::Anon(_) => None,
        }
    }

    /// 若当前页正作为 SLUB 头帧使用，则返回 [`SlabState`] 的独占可变引用；否则返回 `None`。
    pub(crate) fn slab_mut(&mut self) -> Option<&mut SlabState> {
        self.slab()?;
        // SAFETY:
        // 1. 前一行 `self.slab()?` 已在当前 `usage_lock` 临界区内确认槽内变体为 `PageUse::Slab`，不存在固定地址的 `FolioState` 锁外投影。
        // 2. 方法签名接收 `&mut self`，借用检查器保证了此前通过同一 `PageUseGuard` 获取的任何共享引用均已结束，返回的 `&mut SlabState` 生命周期绑定在当前守卫之内。
        match unsafe { &mut *self.descriptor.usage.get() } {
            PageUse::Slab(state) => Some(state),
            PageUse::Empty | PageUse::Folio(_) | PageUse::Anon(_) => unreachable!(),
        }
    }

    /// 获取当前页上固定的 [`FolioState`] 共享引用；若槽内并非 `Folio` 则直接 `panic` 报告生命周期错误。
    pub(crate) fn folio(&self) -> &FolioState {
        match self.value() {
            PageUse::Folio(state) => state,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Anon(_) => {
                panic!("live folio lost its page use")
            }
        }
    }

    /// 获取当前页上 [`AnonState`] 的共享引用；若槽内并非 `Anon` 则直接 `panic` 报告生命周期错误。
    pub(crate) fn anon(&self) -> &AnonState {
        match self.value() {
            PageUse::Anon(state) => state,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Folio(_) => {
                panic!("live anonymous page lost its page use")
            }
        }
    }

    /// 获取当前页上 [`AnonState`] 的独占可变引用（用于增减匿名页引用计数）；若槽内并非 `Anon` 则直接 `panic`。
    pub(crate) fn anon_mut(&mut self) -> &mut AnonState {
        self.anon();
        // SAFETY:
        // 1. `self.anon()` 已在当前 `usage_lock` 临界区内验证变体为 `PageUse::Anon`，槽内不含任何向锁外暴露引用的 `FolioState`。
        // 2. `AnonState` 从不向 `usage_lock` 临界区之外借出引用，且 `&mut self` 确保守卫上的旧视图已失效。
        match unsafe { &mut *self.descriptor.usage.get() } {
            PageUse::Anon(state) => state,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Folio(_) => unreachable!(),
        }
    }

    /// 将新的高层用途状态写入当前必须处于 `PageUse::Empty` 的槽位。
    fn install(&mut self, value: PageUse) {
        assert!(
            matches!(self.value(), PageUse::Empty),
            "owned page already has a use"
        );
        // SAFETY:
        // 1. 上方 `assert!` 已在 `usage_lock` 保护下确认槽内为无析构动作的 `PageUse::Empty`，直接 `write` 覆盖不会泄漏旧拥有者或破坏固定对象。
        // 2. `&mut self` 与 `usage_lock` 排除了任何并发读写与存活短借用。
        unsafe { self.descriptor.usage.get().write(value) };
    }

    /// 在尚未对外发布任何 Slab 对象前，将新分配的 [`SlabState`] 安装到空闲用途槽中。
    pub(crate) fn install_slab(&mut self, state: SlabState) {
        self.install(PageUse::Slab(state));
    }

    /// 在构造首份 `FolioRef` 之前，将 [`FolioState`] 安装并固定到空闲用途槽中。
    pub(crate) fn install_folio(&mut self, state: FolioState) {
        self.install(PageUse::Folio(state));
    }

    /// 在构造首份 `AnonPageRef` 之前，将零阶匿名页的 [`AnonState`] 安装到空闲用途槽中。
    pub(crate) fn install_anon(&mut self, state: AnonState) {
        self.install(PageUse::Anon(state));
    }

    /// 当 Slab 内在用对象数归零且已从所有 CPU/Node 链表摘除时，移出 [`SlabState`] 并将槽位恢复为 `Empty`。
    ///
    /// 调用方取出返回值后，必须先释放本 `PageUseGuard` 及所有 SLUB 控制锁并清理块内 `slab_head`，最后才能析构底层 `OwnedPages`。
    pub(crate) fn take_slab(&mut self) -> Option<SlabState> {
        let state = self.slab()?;
        assert_eq!(state.inuse, 0, "removed slab retains live objects");
        assert_eq!(
            state.location,
            SlabLocation::Detached,
            "removed slab retains a list owner"
        );
        // SAFETY:
        // 1. `self.slab()?` 及后续两个 `assert_eq!` 已在 `usage_lock` 保护下确认当前为无存活对象、已脱离链表的 `Slab` 状态，不含固定 `FolioState`。
        // 2. `&mut self` 确保此前借出的 `state` 共享引用已结束，`replace` 原子地提取 `SlabState` 并恢复为 `PageUse::Empty`。
        match unsafe { self.descriptor.usage.get().replace(PageUse::Empty) } {
            PageUse::Slab(state) => Some(state),
            PageUse::Empty | PageUse::Folio(_) | PageUse::Anon(_) => unreachable!(),
        }
    }

    /// 当匿名页最后一个强引用计数归零时，移出底层物理页所有权 [`OwnedPages`] 并将槽位恢复为 `Empty`。
    ///
    /// 调用方必须先释放本 `PageUseGuard` 退出自旋锁临界区，随后再通过 `Drop` 将物理页归还给页分配器。
    pub(crate) fn take_anon(&mut self) -> OwnedPages {
        assert_eq!(
            self.anon().references,
            0,
            "removed anonymous page retains references"
        );
        // SAFETY:
        // 1. 上方 `assert_eq!` 已在 `usage_lock` 内核实当前为 `Anon` 且引用计数已降为 0，不存在其他 `AnonPageRef` 并发访问。
        // 2. `AnonState` 无 `Pin` 地址固定约束，用 `replace` 取出 `backing` 并置为 `PageUse::Empty` 完全合法。
        match unsafe { self.descriptor.usage.get().replace(PageUse::Empty) } {
            PageUse::Anon(state) => state.backing,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Folio(_) => unreachable!(),
        }
    }

    /// 当 Folio 最后一个引用释放且所有异步 I/O 与等待队列均静止后，取出底层 [`OwnedPages`] 并在原地址就地析构 [`FolioState`]。
    ///
    /// 调用方取得返回的 [`OwnedPages`] 后，必须在释放本 `PageUseGuard` 之后再将其归还给页分配器。
    ///
    /// # Safety
    /// 1. 该 Folio 的引用计数必须已经归零，不存在任何存活的 `FolioRef`、`FolioRead`、`FolioWriteback` 或 `FolioContent` 借用。
    /// 2. 页面内容锁未被人持有，设备 DMA 读写与回写（Writeback）均已彻底结束，脏页标志（Dirty）已清理或按丢弃语义显式撤销，严禁静默丢弃仍有回写持久化义务的数据页。
    /// 3. 内部 `WaitQueue` 上已无任何正在等待或被唤醒途中的睡眠节点，所有指向该 `FolioState` 的 `Pin` 投影均已终止。
    pub(crate) unsafe fn take_folio(&mut self) -> OwnedPages {
        // Step 1: 先从 `FolioState` 内部提取未固定的 `OwnedPages`，同时由 `take_backing` 二次断言无锁、无脏位、无回写。
        // SAFETY:
        // 1. 调用方 `# Safety` 契约保证所有锁外 `FolioRef` 与 `Pin` 投影均已结束，此时在 `usage_lock` 下创建 `&mut PageUse` 不会与任何存活借用冲突。
        // 2. `take_backing()` 仅移出 `Option<OwnedPages>`，不会将受 `Pin` 约束的 `WaitQueue` 移出原内存地址。
        let backing = match unsafe { &mut *self.descriptor.usage.get() } {
            PageUse::Folio(state) => state.take_backing(),
            PageUse::Empty | PageUse::Slab(_) | PageUse::Anon(_) => {
                panic!("live folio lost its page use")
            }
        };
        // Step 2: 在描述符槽位原地址调用 `drop_in_place` 析构 `FolioState`（遵守 `Pin` 原地析构契约），随后写入 `Empty`。
        // SAFETY:
        // 1. `backing` 已被置为 `None` 并转移至局部变量，原地析构 `PageUse::Folio` 仅清理空的 `WaitQueue` 等字段，不会在持锁状态下提前释放物理页。
        // 2. `drop_in_place` 完成后槽位处于未初始化态，紧接着 `write(PageUse::Empty)` 立即恢复合法初始化状态。
        unsafe {
            self.descriptor.usage.get().drop_in_place();
            self.descriptor.usage.get().write(PageUse::Empty);
        }
        backing
    }
}

/// 伙伴系统嵌入在空闲块头帧 [`PageDesc::buddy`] 中的双向链表节点与块阶数元数据。
#[derive(Clone, Copy)]
pub(crate) struct BuddyLink {
    /// 若该页是伙伴系统某阶空闲链表中的块头，则记录其阶数 `0..=MAX_ORDER`；否则为 `usize::MAX`（表示不在伙伴空闲链头）。
    pub(crate) order: usize,
    /// 同阶空闲双向链表中前驱空闲块的头 PFN；若本块已是链表首元素或不在链表中则为 `usize::MAX`。
    pub(crate) previous: usize,
    /// 同阶空闲双向链表中后继空闲块的头 PFN；若本块已是链表尾元素或不在链表中则为 `usize::MAX`。
    pub(crate) next: usize,
}

/// SLUB Slab 块当前所处的唯一归属位置状态（用于校验链表成员身份与冻结状态）。
///
/// 当 Slab 处于 `Active(cpu)` 时即代表“已冻结（Frozen）”：本地分配直接走该 CPU 的无锁/轻锁 `local` 链表，
/// 跨核释放归入 `SlabState::shared` 时不会将其挂入节点部分满链表；若远端 `flush_all` 摘离了某核的 `active` 字段，
/// 在摘离方通过票据（`ActiveTicket`）完成解冻归并之前，其 `location` 仍保持 `Active(cpu)` 以阻止其他路径重复接管。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SlabLocation {
    /// 已从所有 CPU 与节点链表摘除的游离状态（刚分配尚未挂载，或正准备销毁归还底层页块）。
    Detached,
    /// 当前正作为编号为 `usize` 的 CPU 的主活动（Frozen）Slab，或正由该 CPU 摘离票据持有待解冻。
    Active(usize),
    /// 挂在编号为 `usize` 的 CPU 本地部分满缓存链表（Per-CPU Partial List）中。
    CpuPartial(usize),
    /// 挂在全局节点级部分满双向链表（`NodePartial`）中，可供任意 CPU 获取补充。
    NodePartial,
    /// 所有对象均已分配完毕且不在任何 CPU/Node 链表中；待首个对象释放时重新迁回部分满链表。
    Full,
}

/// SLUB Slab 复合页块头帧描述符中保存的对象分配元数据。
///
/// 锁保护契约：本结构体的读写必须持有头帧的 `PageDesc::usage_lock`；当修改链表指针（`next` / `previous`）或切换
/// `CpuPartial` / `NodePartial` 归属时，必须先按顺序持有外层的 `CpuSlab` 锁或 `NodePartial` 锁，再获取本头帧锁。
/// 销毁 Slab 时，必须先在头帧锁下通过 [`PageUseGuard::take_slab`] 取出本结构体，退出全部 SLUB 锁与 `usage_lock`
/// 并将块内所有子页的 `slab_head` 重置为 `usize::MAX` 后，才能析构 `backing` 归还物理页。
pub(crate) struct SlabState {
    /// 承载该 Slab 的连续物理页块唯一归还句柄（同时保存了分配时的真实阶数 `order`）。
    pub(crate) backing: OwnedPages,
    /// 所属 SLUB 大小等级缓存的下标（对应 `slub::SIZES` 数组的索引 `0..12`）。
    pub(crate) cache: usize,
    /// 该 Slab 块切分出的固定大小对象总槽位数。
    pub(crate) objects: usize,
    /// 已脱离 `shared` 共享空闲链表的对象计数（注意：当处于 `Active` 冻结态时，包含已批给 CPU `local` 空闲链的对象，因此不能直接当作用户存活对象数）。
    pub(crate) inuse: usize,
    /// 共享空闲对象单向链表的首对象虚拟地址（以空闲对象首 `usize` 字存放下一节点地址，`0` 表示链表末尾；正在被用户使用的对象不在此链中）。
    pub(crate) shared: usize,
    /// 该 Slab 当前所在的链表归属位置。
    pub(crate) location: SlabLocation,
    /// 在 `CpuPartial` 单向链表或 `NodePartial` 双向链表中的后继 Slab 头 PFN（无后继时为 `usize::MAX`）。
    pub(crate) next: usize,
    /// 在 `NodePartial` 双向链表中的前驱 Slab 头 PFN（不处于 `NodePartial` 或无前驱时为 `usize::MAX`）。
    pub(crate) previous: usize,
}

// SAFETY:
// 1. `kind` 在启动期单核写入后终生只读。
// 2. `buddy: UnsafeCell<BuddyLink>` 仅在全局唯一的 `buddy::Zone` 自旋锁临界区内读写。
// 3. `slab_head` 为 `AtomicUsize`，跨核通过 Release/Acquire 原子序同步。
// 4. `usage: UnsafeCell<PageUse>` 的变体切换以及 `SlabState`、`AnonState`、`FolioShared` 的访问均由本描述符的 `usage_lock` 自旋锁串行化；
//    `FolioState` 的 `flags` 为原子变量、`waiters` 自带内部自旋锁，且整个 `PageDesc` 发布后地址永久固定，直到最后一个引用归零才在锁下原地析构。
unsafe impl Sync for PageDesc {}

impl PageDesc {
    /// 构造指定静态物理分类 [`PageKind`]、不带任何动态链表身份与高层用途的初始页描述符。
    pub(crate) const fn new(kind: PageKind) -> Self {
        Self {
            kind,
            buddy: UnsafeCell::new(BuddyLink {
                order: usize::MAX,
                previous: usize::MAX,
                next: usize::MAX,
            }),
            slab_head: AtomicUsize::new(usize::MAX),
            usage_lock: SpinLock::new(()),
            usage: UnsafeCell::new(PageUse::Empty),
            _pinned: PhantomPinned,
        }
    }

    /// 获取当前页描述符的用途自旋锁（`usage_lock`），返回受控访问守卫 [`PageUseGuard`]。
    ///
    /// 调用上下文：不可睡眠的短临界区；调用前需遵守 `CpuSlab`/`NodePartial` $\rightarrow$ `usage_lock` 锁顺序。
    pub(crate) fn usage(&self) -> PageUseGuard<'_> {
        PageUseGuard {
            descriptor: self,
            _lock: self.usage_lock.lock(),
        }
    }

    /// 返回该物理页在启动期确定的静态分类 [`PageKind`]。
    ///
    /// 注意：返回 `PageKind::Available` 仅代表该页属于伙伴系统管辖的普通 RAM，并不代表它此刻正空闲。
    pub(crate) fn kind(&self) -> PageKind {
        self.kind
    }
}

/// 全局稀疏分段物理页描述符映射表。
pub(crate) struct PageMap<'a> {
    /// 按 128 MiB 段索引的根表切片，覆盖从 PFN 0 到最高可用 RAM 段上界的全部段槽位。
    sections: &'a [Section<'a>],
    /// 扣除启动保留区与描述符自身元数据页之后剩余的空闲 PFN 区间快照，供伙伴系统初始化时一次性接管。
    free: &'a [PageRange],
    /// 启动期定格的物理页四域（`present` / `reserved` / `metadata` / `free`）页数账务。
    stats: BootStats,
    /// 根表中实际分配了 [`PageDesc`] 数组的非空段（`Some`）数量。
    present_sections: usize,
    /// 内核自测（`ktest`）专用的元数据 PFN 区间只读快照，用于校验描述符页自身的 `PageKind::Metadata` 标记。
    #[cfg(feature = "ktest")]
    metadata: &'a [PageRange],
}

/// 启动构建 [`PageMap`] 期间用于合并连续非空段并记录其描述符承载页区间的辅助结构。
#[derive(Clone, Copy)]
struct SectionGroup {
    /// 连续非空段组的起始段下标（含）。
    start: usize,
    /// 连续非空段组的结束段下标（不含）。
    end: usize,
    /// 从 [`BootMemory`] 一次性为该组 `(end - start)` 个段分配的连续物理承载页区间。
    backing: PageRange,
}

impl PageMap<'_> {
    /// 根据物理页帧号 `pfn` 以 $O(1)$ 时间查找对应的 [`PageDesc`]；若 `pfn` 超界或落在纯空洞段则返回 `None`。
    pub(crate) fn get(&self, pfn: usize) -> Option<&PageDesc> {
        self.sections
            .get(pfn / PAGES_PER_SECTION)?
            .as_ref()?
            .get(pfn % PAGES_PER_SECTION)
    }

    /// 返回扣除全部启动保留页与元数据页之后的启动期物理页分类统计计数。
    pub(crate) fn stats(&self) -> BootStats {
        self.stats
    }

    /// 返回根索引表覆盖的 128 MiB 段总槽位数（含中间空洞段）。
    pub(crate) fn section_count(&self) -> usize {
        self.sections.len()
    }

    /// 返回实际包含物理 RAM 并已分配 [`PageDesc`] 数组的非空段数量。
    pub(crate) fn present_sections(&self) -> usize {
        self.present_sections
    }

    /// 返回最终应交由伙伴系统接管的初始空闲 PFN 区间切片（仅提供区间信息，不附带释放凭据）。
    pub(crate) fn free_ranges(&self) -> &[PageRange] {
        self.free
    }
}

#[cfg(feature = "ktest")]
impl<'a> PageMap<'a> {
    /// 返回稀疏段根表与元数据页区间切片，供 `ktest` 用例逐页核对物理内存分类不变量。
    pub(crate) fn test_layout_parts(&self) -> (&[Section<'a>], &[PageRange]) {
        (self.sections, self.metadata)
    }

    /// 使用测试自定义的局部描述符切片与空闲区间构造隔离的 [`PageMap`]，不触碰全局生产内存池。
    pub(crate) fn test_map(sections: &'a [Section<'a>], free: &'a [PageRange]) -> Self {
        Self {
            sections,
            free,
            stats: BootStats {
                present_pages: 0,
                reserved_pages: 0,
                metadata_pages: 0,
                free_pages: free.iter().map(|range| range.end - range.start).sum(),
            },
            present_sections: sections.iter().filter(|section| section.is_some()).count(),
            metadata: &[],
        }
    }
}

/// 将所需字节数向上取整为 4 KiB 承载页数量，同时校验字节跨度非零且不超过 Rust 切片最大合法字节上界 `isize::MAX`。
fn backing_pages(bytes: usize) -> Result<usize, &'static str> {
    if bytes == 0 || (isize::MAX as usize) < bytes {
        return Err("page metadata layout is invalid");
    }
    Ok(bytes.div_ceil(PAGE_SIZE))
}

/// 利用单调递增游标 `cursor` 判断有序不重叠区间列表 `ranges` 是否包含递增扫描中的目标 `pfn`。
///
/// 调用方保证对同一个 `cursor` 传入的 `pfn` 单调递增，从而使整轮初始化对每个区间表仅做 $O(N)$ 线性扫描。
fn cursor_contains(ranges: &[PageRange], cursor: &mut usize, pfn: usize) -> bool {
    while *cursor < ranges.len() && ranges[*cursor].end <= pfn {
        *cursor += 1;
    }
    *cursor < ranges.len() && ranges[*cursor].start <= pfn
}

/// 将 `ranges` 中的区间逐项复制到直接映射虚拟地址 `address` 处，并构造永久 `'static` 只读切片。
///
/// # Safety
/// 1. **非空与对齐**：即使 `ranges` 为空，`address` 也必须非零且满足 `PageRange` 的对齐要求。
/// 2. **独占有效承载区**：`address` 起始的内存必须位于单次连续分配的永久可写直接映射 RAM 内，至少容纳 `ranges.len()` 个 `PageRange` 槽位，总字节跨度不超过 `isize::MAX` 且指针加法不回绕。
/// 3. **无别名与不可变发布**：目标内存区间不得与源切片 `ranges` 或任何其他存活引用重叠；本函数返回后，严禁再次修改、释放或重用这段承载内存。
unsafe fn write_ranges(address: usize, ranges: &[PageRange]) -> &'static [PageRange] {
    let destination = address as *mut PageRange;
    for (index, range) in ranges.iter().enumerate() {
        // SAFETY:
        // 1. 调用方契约保证 `destination` 指向至少容纳 `ranges.len()` 项的独占、对齐且可写的连续直接映射内存，`index < ranges.len()` 在合法边界内。
        // 2. 目标区域与源切片 `ranges` 不重叠，逐槽首次初始化安全。
        unsafe { destination.add(index).write(*range) };
    }
    // SAFETY:
    // 1. 上方循环已完成全部 `ranges.len()` 个元素的有效初始化，且即使长度为 0，`destination` 仍满足非空与对齐要求。
    // 2. 调用方保证该直接映射内存永久保留且后续不再写入，满足 `'static` 只读切片的不变量。
    unsafe { slice::from_raw_parts(destination, ranges.len()) }
}

/// 从启动账本 `memory` 中划拨全部描述符元数据页，按递增 PFN 初始化所有非空段的 [`PageDesc`] 并构建永久 [`PageMap`]。
///
/// 关键顺序设计：必须**先完成全部元数据页分配**（使 `memory.metadata()` 与 `memory.free()` 最终定型），
/// **再开始遍历填充 [`PageDesc`]**，这样元数据页自身对应的 `PageDesc` 才能被准确标记为 [`PageKind::Metadata`]。
///
/// # Safety
/// 1. **单核关中断启动**：仅允许由 BSP 在本地中断关闭、AP 尚未唤醒时调用一次。
/// 2. **账本与直接映射真实性**：`memory` 必须由当前机器真实的 RAM 与完整保留区初始化；其中所有空闲页必须已建立一一对应、永久有效且可写的直接映射（`arch::phys_to_virt`），每次 `memory.allocate` 返回的连续物理页在虚拟地址上同样连续、非空、按页对齐且不发生地址回绕。
/// 3. **元数据独占与生命周期**：划拨出的承载页不得与 `memory` 栈对象或其他存活引用重叠，且永久不得释放或重分配；发布后 `PageMap` 的根索引、区间快照与 `PageDesc::kind` 终生只读，`PageDesc` 永不移动，内部可变字段严格遵循各自锁与原子协议。
/// 4. **失败停机**：若中途返回 `Err`，部分承载页可能已从 `memory` 扣除，调用方必须立即 `panic` 终止启动而不得继续使用半成品账本。
pub(crate) unsafe fn initialize(memory: &mut BootMemory) -> Result<PageMap<'static>, &'static str> {
    // Step 1: 将所有物理 RAM 区间映射到 128 MiB 段号，合并相邻或同段的区间以统计非空段组 `groups`。
    let mut groups = [SectionGroup {
        start: 0,
        end: 0,
        backing: PageRange { start: 0, end: 0 },
    }; MAX_MEMORY_REGIONS];
    let mut group_count = 0;
    for range in memory.ram() {
        let start = range.start / PAGES_PER_SECTION;
        let end = range.end.div_ceil(PAGES_PER_SECTION);
        if group_count != 0 && start <= groups[group_count - 1].end {
            groups[group_count - 1].end = groups[group_count - 1].end.max(end);
        } else {
            groups[group_count].start = start;
            groups[group_count].end = end;
            group_count += 1;
        }
    }
    if group_count == 0 {
        return Err("page map has no RAM sections");
    }

    // Step 2: 预先从 `BootMemory` 切出根表页（含根索引数组 + `free` 快照区 + 可选 `ktest` `metadata` 快照区）。
    let section_count = groups[group_count - 1].end;
    let section_bytes = section_count
        .checked_mul(size_of::<Section<'static>>())
        .ok_or("page root size overflows")?;
    // `Section<'static>` 与 `PageRange` 均按 `usize` 对齐，因此在 `sections` 数组末尾可直接紧接存放 `PageRange` 快照。
    let range_bytes = RANGE_CAPACITY
        .checked_mul(size_of::<PageRange>())
        .ok_or("page range storage overflows")?;
    let root_bytes = section_bytes
        .checked_add(range_bytes)
        .ok_or("page root size overflows")?;
    #[cfg(feature = "ktest")]
    let root_bytes = root_bytes
        .checked_add(range_bytes)
        .ok_or("page root size overflows")?;
    let root = memory.allocate(backing_pages(root_bytes)?, 1)?;

    // Step 3: 预先为每个连续非空段组一次性分配连续的 `PageDesc` 承载页；至此全部 `Metadata` 页分配完毕，账本定型。
    let section_descriptor_bytes = PAGES_PER_SECTION
        .checked_mul(size_of::<PageDesc>())
        .ok_or("section descriptor size overflows")?;
    let mut present_sections = 0;
    for group in &mut groups[..group_count] {
        let count = group.end - group.start;
        let bytes = count
            .checked_mul(section_descriptor_bytes)
            .ok_or("page descriptor size overflows")?;
        group.backing = memory.allocate(backing_pages(bytes)?, 1)?;
        present_sections += count;
    }

    // Step 4: 按段号递增扫描 `0..section_count`，利用三个单调游标线性判定每个 PFN 的 `PageKind`，填充 `PageDesc` 数组与根索引表。
    let root_address = crate::arch::phys_to_virt(root.start * PAGE_SIZE);
    let root_pointer = root_address as *mut Section<'static>;
    let mut group_index = 0;
    let mut ram_cursor = 0;
    let mut reserved_cursor = 0;
    let mut metadata_cursor = 0;
    for section_index in 0..section_count {
        while group_index < group_count && groups[group_index].end <= section_index {
            group_index += 1;
        }
        let descriptors = if group_index < group_count && groups[group_index].start <= section_index
        {
            let group = groups[group_index];
            let offset = (section_index - group.start) * section_descriptor_bytes;
            let address = crate::arch::phys_to_virt(group.backing.start * PAGE_SIZE) + offset;
            let destination = address as *mut PageDesc;
            for index in 0..PAGES_PER_SECTION {
                let pfn = section_index * PAGES_PER_SECTION + index;
                let ram = cursor_contains(memory.ram(), &mut ram_cursor, pfn);
                let reserved = cursor_contains(memory.reserved(), &mut reserved_cursor, pfn);
                let metadata = cursor_contains(memory.metadata(), &mut metadata_cursor, pfn);
                let kind = if !ram {
                    PageKind::Absent
                } else if reserved {
                    PageKind::Reserved
                } else if metadata {
                    PageKind::Metadata
                } else {
                    PageKind::Available
                };
                // SAFETY:
                // 1. `group.backing` 是由 `memory.allocate` 独占划拨的页对齐永久直接映射内存，`offset` 将每个 `section_index` 映射到互不重叠的 `section_descriptor_bytes` 子块。
                // 2. 页起始地址（4096 对齐）加上 `section_descriptor_bytes`（32768 * `size_of::<PageDesc>()`）自然满足 `PageDesc` 对齐，且 `index < PAGES_PER_SECTION` 在分配容量之内，每个槽位仅在此首次写入。
                unsafe { destination.add(index).write(PageDesc::new(kind)) };
            }
            // SAFETY:
            // 1. 当前段的全部 `PAGES_PER_SECTION` 个 `PageDesc` 槽位均已由上方循环完成初始化，字节跨度已通过 `backing_pages` 校验不超过 `isize::MAX`。
            // 2. 该段内存位于永久保留的 `group.backing` 内，后续段的 `offset` 与本切片互不重叠，且发布后永不移动或释放。
            Some(unsafe { slice::from_raw_parts(destination, PAGES_PER_SECTION) })
        } else {
            None
        };
        // SAFETY:
        // 1. `root` 承载块已通过 `backing_pages(root_bytes)` 分配了足够容纳 `section_count` 个 `Section` 的页对齐独占直接映射内存。
        // 2. `section_index < section_count`，每个根表槽位仅在此循环中写入一次合法的 `Option<&'static [PageDesc]>`。
        unsafe { root_pointer.add(section_index).write(descriptors) };
    }

    // Step 5: 将填充完毕的根索引数组与尾随其后的 `free`（及 `ktest` 下的 `metadata`）区间快照封装为 `'static` 切片返回。
    // SAFETY:
    // 1. `0..section_count` 全部根表项均已在 Step 4 中完成初始化，总跨度 `section_bytes <= root_bytes <= isize::MAX`。
    // 2. `sections` 仅覆盖 `[root_address, root_address + section_bytes)`，后续 `write_ranges` 仅写入 `root_address + section_bytes` 之后的尾部区域，二者互不重叠且发布后永久只读。
    let sections = unsafe { slice::from_raw_parts(root_pointer, section_count) };
    // SAFETY:
    // 1. Step 2 计算 `root_bytes` 时已在 `section_bytes` 之后预留了 `RANGE_CAPACITY` 个 `PageRange` 槽位，且 `Section` 字节大小为 `usize` 倍数，故 `root_address + section_bytes` 满足 `PageRange` 对齐且非空。
    // 2. `memory.free().len() <= RANGE_CAPACITY`，与栈上的 `memory` 不重叠，写入后永久只读。
    let free = unsafe { write_ranges(root_address + section_bytes, memory.free()) };
    #[cfg(feature = "ktest")]
    // SAFETY:
    // 1. 在启用 `ktest` 时，Step 2 额外累加了第二个 `range_bytes`，故 `root_address + section_bytes + range_bytes` 拥有独立的 `RANGE_CAPACITY` 个对齐槽位，与 `sections` 及 `free` 均不重叠。
    // 2. `memory.metadata().len() <= RANGE_CAPACITY`，写入后永久只读。
    let metadata = unsafe {
        write_ranges(
            root_address + section_bytes + range_bytes,
            memory.metadata(),
        )
    };

    Ok(PageMap {
        sections,
        free,
        stats: memory.stats(),
        present_sections,
        #[cfg(feature = "ktest")]
        metadata,
    })
}
