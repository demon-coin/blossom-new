//! 页缓存稀疏索引树（AVL 树）与全局文件页 LRU 队列。
//!
//! ## 模块职责与架构定位
//! 本模块位于页缓存层（`crate::cache`）底层，为上层元数据页索引（`MetadataPages`）和
//! 文件页映射（`FileMapping`）提供按偏移键检索的平衡二叉搜索树（`Tree`），同时维护
//! 全局唯一的磁盘文件页双队列 LRU（`FILE_LRU`），供内存回收器（`crate::mm::reclaim`）
//! 按访问热度扫描和隔离可回收的文件页。
//!
//! ## 核心设计与关键状态机
//!
//! ### 1. 节点单实例与双投影拆分（`Node`）
//! 每个缓存页节点（`Node`）在创建后其键（`key`）、物理页引用（`folio`）和所属 Inode
//! 身份（`role`）均保持不可变。同一个 `Node` 实例通过两个互不重叠的 `UnsafeCell` 分别嵌入：
//! - **所属索引的 AVL 树**：通过 `index: UnsafeCell<IndexLinks>` 维护左右子树指针、树高和
//!   本轮回写选中标记（`to_write`），仅在持有所属 `FolioIndex::entries` 自旋锁时访问。
//! - **全局文件页 LRU 链表**：通过 `lru: UnsafeCell<LruLinks>` 维护双向链表裸指针、LRU 状态
//!   与保活凭证（`NodeRawTicket`），仅在持有全局 `FILE_LRU` 自旋锁时访问。
//!
//! ```text
//!                      Arc<Node, IndexAllocator>
//!            +-------------------------------------------+
//!            | Immutable: key (u64), folio, role         |
//!            +---------------------+---------------------+
//!            | UnsafeCell          | UnsafeCell          |
//!            | IndexLinks (AVL)    | LruLinks (Global)   |
//!            | - to_write: bool    | - kind: LruKind     |
//!            | - height: u16       | - previous: *const  |
//!            | - left: Link        | - next: *const      |
//!            | - right: Link       | - ticket: Option    |
//!            +----------+----------+----------+----------+
//!                       |                     |
//!         Protected by per-index       Protected by global
//!        FolioIndex::entries lock         FILE_LRU lock
//! ```
//!
//! ### 2. 全局文件页 LRU 状态机（`LruKind`）
//! 只有后端为磁盘文件（`FileRole::Disk`）的节点才会进入 `FILE_LRU`。节点在链表中的
//! 生命周期状态转换如下：
//!
//! ```text
//!   Tree::insert
//!        |
//!        v
//!  +------------+   2nd access on scan    +------------+
//!  |  Inactive  | ----------------------> |   Active   |
//!  +------------+ <---------------------- +------------+
//!    |        ^     age_active (unreferenced)
//!    |        |
//!    | isolate_inactive                   Tree::remove /
//!    | (unreferenced)                     remove_isolated /
//!    v        | putback                   Tree::drop
//!  +------------+ ----------------------> +------------+
//!  |  Isolated  |                         |  Detached  |
//!  +------------+                         +------------+
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 1. **锁层级顺序**：`FolioIndex::entries`（单个索引的 AVL 树自旋锁） ➔ `FILE_LRU`（全局
//!    LRU 自旋锁） ➔ `Folio` 内部引用计数锁（在 `with_lru` 回调内执行 `retire_clean`）。
//!    严禁在持有 `FILE_LRU` 时反向获取任何 `FolioIndex::entries` 锁。
//! 2. **自旋锁临界区无析构契约**：从索引或 LRU 摘除节点时，节点的 `Arc` 强引用会打包进
//!    `RetiredNode` 或返回给调用者，必须在释放 `entries` 和 `FILE_LRU` 自旋锁之后再调用
//!    `release()` 释放物理页引用，防止在最后一个强引用释放时重入块设备或页框分配器锁。

use alloc::alloc::Global;
use alloc::sync::Arc;
use core::alloc::{AllocError, Allocator, Layout};
use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::ops::Deref;
use core::ptr::{self, NonNull};

use crate::block::{BioReserve, BlockError};
use crate::fs::InodeWeak;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::FolioRef;
use crate::sync::SpinLock;

/// `Node` 所使用的底层内存分配器，支持普通内核堆与读写元数据专用预留池。
#[derive(Clone)]
pub(crate) enum IndexAllocator {
    /// 从全局内核堆（`Global`）分配 `Arc<Node>` 控制块（用于文件页、裸盘及只读元数据）。
    Heap,
    /// 从块层预留池（`BioReserve`）的指定槽位分配 `Arc<Node>` 控制块（用于可读写元数据，保证回写路径不因堆耗尽而阻塞）。
    Metadata {
        /// 所属磁盘的元数据预留资源池。
        reserve: Arc<BioReserve>,
        /// 该节点在预留池中占用的固定槽位下标。
        slot: usize,
    },
}

// SAFETY:
// 1. `Heap` 分支直接委托给标准库 `Global` 分配器。
// 2. `Metadata` 分支从 `BioReserve` 的指定 `slot` 分配满足 `layout` 对齐与大小要求的固定内存块；
//    `Arc<BioReserve>` 保证在所有强弱引用销毁前底层存储持续有效，`deallocate` 会将原指针与布局成对归还给同一槽位。
unsafe impl Allocator for IndexAllocator {
    /// 为 `Arc<Node, IndexAllocator>` 分配控制块与节点存储。
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => Global.allocate(layout),
            Self::Metadata { reserve, slot } => reserve.allocate_metadata_control(*slot, layout),
        }
    }

    /// 在 `Arc` 的所有强引用和弱引用全部释放后，归还底层控制块内存。
    ///
    /// # Safety
    /// 1. `pointer` 必须是由同一个 `IndexAllocator` 实例通过 `allocate` 分配的有效指针。
    /// 2. `layout` 必须与分配该指针时传入的布局完全一致，且该控制块此前未被释放过。
    unsafe fn deallocate(&self, pointer: NonNull<u8>, layout: Layout) {
        match self {
            Self::Heap => {
                // SAFETY: 调用方（`Arc` 析构逻辑）保证 `pointer` 与 `layout` 来自此前 `Global.allocate` 的成功返回，
                // 且所有强弱引用计数已归零，不会再次访问或重复释放该堆块。
                unsafe { Global.deallocate(pointer, layout) };
            }
            Self::Metadata { reserve, slot } => {
                // SAFETY: 调用方保证 `pointer` 与 `layout` 来自同一 `reserve` 与 `slot` 的 `allocate_metadata_control`，
                // 且 `Arc` 的强弱引用均已结束，此次调用唯一一次归还该槽位的控制块存储。
                unsafe { reserve.deallocate_metadata_control(*slot, pointer, layout) };
            }
        }
    }
}

/// 带自定义分配器的缓存索引节点强引用。
pub(crate) type NodeRef = Arc<Node, IndexAllocator>;
/// AVL 树中的可选子节点链接。
pub(super) type Link = Option<NodeRef>;

/// 嵌入在 `Node` 内部的 AVL 树链接与回写标记，仅在所属 `Tree` 锁保护下访问。
struct IndexLinks {
    /// 标记该节点是否在当前同步/回写批次中被选中待写回。
    to_write: bool,
    /// 以当前节点为根的 AVL 子树高度（空树为 0，叶子节点为 1）。
    height: u16,
    /// 左子树根节点（所有键严格小于当前节点的 `key`）。
    left: Link,
    /// 右子树根节点（所有键严格大于当前节点的 `key`）。
    right: Link,
}

/// 证明当前上下文持有所属 `Tree` 独占修改权限的零大小凭证，用于约束 `Node::links_mut` 的可变借用周期。
struct IndexWrite;

/// 节点在全局文件页 LRU（`FILE_LRU`）中的生命周期状态。
#[derive(Clone, Copy, PartialEq, Eq)]
enum LruKind {
    /// 未加入任何 LRU 队列（非磁盘文件页，或已从索引摘除完毕）。
    Detached,
    /// 位于非活跃队列（`inactive`），是内存回收扫描的首选候选。
    Inactive,
    /// 位于活跃队列（`active`），表示近期被多次访问的热页。
    Active,
    /// 已被回收扫描线程从 `inactive` 队列临时摘出（正在锁外检查映射与脏状态）。
    Isolated,
}

/// 嵌入在 `Node` 内部的全局 LRU 双向链表指针与保活凭证，仅在 `FILE_LRU` 自旋锁保护下访问。
struct LruLinks {
    /// 当前节点的 LRU 队列归属状态。
    kind: LruKind,
    /// 所在 LRU 队列中的前驱节点裸指针（队首为 `null`）。
    previous: *const Node,
    /// 所在 LRU 队列中的后继节点裸指针（队尾为 `null`）。
    next: *const Node,
    /// 当节点挂在 `Inactive` 或 `Active` 队列上时，用于保活该节点的 `Arc` 裸凭证。
    ticket: Option<NodeRawTicket>,
}

/// 文件页节点所属后端文件系统的角色分类。
pub(crate) enum FileRole {
    /// 磁盘文件系统（如 ext4）中的文件页，支持加入全局 `FILE_LRU` 进行页面回收与回写。
    Disk(InodeWeak),
    /// 内存文件系统（tmpfs/ramfs）中的文件页，常驻内存，不进入磁盘 LRU 或回写队列。
    Memory(InodeWeak),
}

impl FileRole {
    /// 判断该文件页是否属于可被 LRU 回收的磁盘文件。
    pub(crate) fn is_disk(&self) -> bool {
        matches!(self, Self::Disk(_))
    }

    /// 获取该节点所属文件 `Inode` 的弱引用。
    pub(crate) fn owner(&self) -> &InodeWeak {
        match self {
            Self::Disk(owner) | Self::Memory(owner) => owner,
        }
    }
}

/// 页缓存稀疏索引与全局 LRU 共用的不可变页身份节点。
pub(crate) struct Node {
    /// 页在索引中的键（对于文件缓存为页序号 `index`，对于元数据缓存为磁盘物理字节偏移）。
    pub key: u64,
    /// 该节点管理的底层 4KiB 物理页（`Folio`）强引用。
    pub folio: FolioRef,
    /// 若为文件页则记录所属 Inode 角色；若为元数据页则为 `None`。
    pub role: Option<FileRole>,
    /// 供所属 `Tree`（AVL 树）独占维护的树结构链接与待写标记。
    index: UnsafeCell<IndexLinks>,
    /// 供全局 `FILE_LRU` 独占维护的双向链表链接与状态。
    lru: UnsafeCell<LruLinks>,
}

// SAFETY:
// 1. `Node` 的公开字段（`key`、`folio`、`role`）在初始化后只读，且它们本身满足 `Send + Sync`。
// 2. `index` 仅在所属 `FolioIndex::entries` 自旋锁保护下访问，`lru` 仅在全局 `FILE_LRU` 自旋锁保护下访问，
//    两个 `UnsafeCell` 内存区域完全独立，跨线程转移或共享 `Arc<Node>` 不会产生数据竞争。
unsafe impl Send for Node {}
// SAFETY: 同上，两个内部可变区域分别受各自的互斥锁保护，且从不构造指向整个 `Node` 的 `&mut Node`。
unsafe impl Sync for Node {}

/// 从索引树摘除后等待在自旋锁外释放的节点句柄。
pub(super) struct RetiredNode {
    /// 原先由 AVL 索引树持有的节点强引用。
    node: NodeRef,
    /// 若该节点同时从 `FILE_LRU` 的 `Inactive`/`Active` 队列上摘下，则保存对应的第二份强引用。
    member: Option<NodeRef>,
}

impl Deref for RetiredNode {
    type Target = Node;
    /// 解引用访问已摘除节点的只读身份字段（如 `key` 和 `folio`）。
    fn deref(&self) -> &Node {
        &self.node
    }
}

impl RetiredNode {
    /// 在所有索引与 LRU 自旋锁之外释放该节点持有的强引用，返回实际归还给伙伴系统的物理页数。
    pub(super) fn release(self) -> usize {
        let mut freed = self.member.map_or(0, release_node);
        freed += release_node(self.node);
        freed
    }
}

impl Node {
    /// 在不持有任何索引锁或 LRU 锁的上下文中分配并初始化一个新的 `Node`。
    pub(super) fn prepare(
        key: u64,
        folio: FolioRef,
        role: Option<FileRole>,
        allocator: IndexAllocator,
    ) -> Result<NodeRef, BlockError> {
        let mut prepared =
            Arc::<Self, _>::try_new_uninit_in(allocator).map_err(|_| BlockError::NoMemory)?;
        Arc::get_mut(&mut prepared)
            .expect("unpublished node has an observer")
            .write(Self {
                key,
                folio,
                role,
                index: UnsafeCell::new(IndexLinks {
                    to_write: false,
                    height: 1,
                    left: None,
                    right: None,
                }),
                lru: UnsafeCell::new(LruLinks {
                    kind: LruKind::Detached,
                    previous: ptr::null(),
                    next: ptr::null(),
                    ticket: None,
                }),
            });
        // SAFETY: `prepared` 是刚刚分配且尚未向外部发布的私有 `Arc<MaybeUninit<Self>, _>`，
        // 上方的 `write` 已完整初始化 `Node` 的所有字段，因此可安全转换为已初始化的 `Arc<Self, _>`。
        Ok(unsafe { prepared.assume_init() })
    }

    /// 在元数据预留池（`BioReserve`）初始化阶段，预先触发一次指定槽位的 `Arc<Node>` 分配并立即释放，
    /// 使预留池在挂载完成前锁定真实的 `Arc` 控制块内存布局。
    pub(super) fn prime(reserve: &Arc<BioReserve>, slot: usize) -> Result<(), BlockError> {
        let prepared: Arc<MaybeUninit<Self>, _> =
            Arc::<Self, _>::try_new_uninit_in(IndexAllocator::Metadata {
                reserve: Arc::clone(reserve),
                slot,
            })
            .map_err(|_| BlockError::NoMemory)?;
        drop(prepared);
        Ok(())
    }

    /// 在持有所属 `Tree` 访问权限的上下文中只读借用 AVL 树链接字段。
    fn links(&self) -> &IndexLinks {
        // SAFETY: 该方法仅在 `Tree` 及其 AVL 辅助函数中被调用，调用方已持有所属 `FolioIndex::entries` 自旋锁，
        // 且同一时刻不存在仍在存活的 `links_mut` 可变借用；全局 LRU 操作绝不触碰 `self.index`。
        unsafe { &*self.index.get() }
    }

    /// 在持有所属 `Tree` 独占写凭证（`IndexWrite`）的上下文中可变借用 AVL 树链接字段。
    fn links_mut<'a>(&'a self, _write: &'a mut IndexWrite) -> &'a mut IndexLinks {
        // SAFETY: 调用方必须传入 `&'a mut IndexWrite`，该凭证仅在 `Tree` 的可变方法（持有 `entries` 自旋锁）中构造；
        // 借用生命周期绑定在 `_write` 上，保证同一时刻最多只存在一个 `IndexLinks` 可变引用，且不与 `lru` 字段重叠。
        unsafe { &mut *self.index.get() }
    }

    /// 查询该节点是否已被标记为本轮回写目标（调用方需持有所属索引锁）。
    pub(super) fn to_write(&self) -> bool {
        self.links().to_write
    }
}

/// 将一份 `Arc<Node, IndexAllocator>` 强引用拆解为裸指针与分配器后存放在 `LruLinks` 中的保活凭证。
///
/// 这样可以在链表节点内部自持强引用而不产生自引用类型，并在从链表摘除时精确还原原始 `Arc`。
struct NodeRawTicket {
    /// 由 `Arc::into_raw_with_allocator` 拆出的 `Node` 裸指针。
    pointer: *const Node,
    /// 原始 `Arc` 的分配器实例（在还原 `Arc` 时通过 `take()` 取出）。
    allocator: Option<IndexAllocator>,
}

impl NodeRawTicket {
    /// 消费一份 `NodeRef` 强引用，将其转换为可存入 LRU 链表节点的裸凭证。
    fn new(node: NodeRef) -> Self {
        let (pointer, allocator) = Arc::into_raw_with_allocator(node);
        Self {
            pointer,
            allocator: Some(allocator),
        }
    }

    /// 消费该裸凭证并还原出原始的 `NodeRef` 强引用。
    fn into_node(mut self) -> NodeRef {
        let allocator = self
            .allocator
            .take()
            .expect("node raw ticket consumed twice");
        // SAFETY: `self.pointer` 与 `allocator` 来自 `Self::new` 中对同一 `Arc` 调用的 `into_raw_with_allocator`，
        // 且 `self.allocator.take()` 保证该强引用凭证只被还原一次。
        unsafe { Arc::from_raw_in(self.pointer, allocator) }
    }
}

impl Drop for NodeRawTicket {
    /// 若凭证未被 `into_node` 显式取走便被丢弃，则还原并释放对应的 `Arc` 强引用。
    fn drop(&mut self) {
        if let Some(allocator) = self.allocator.take() {
            // SAFETY: `self.allocator` 为 `Some` 说明该凭证尚未被 `into_node` 消费；
            // `self.pointer` 与 `allocator` 配对有效，重建 `Arc` 并立即 drop 可正确递减强引用计数。
            drop(unsafe { Arc::from_raw_in(self.pointer, allocator) });
        }
    }
}

// SAFETY: `NodeRawTicket` 逻辑上等价于拥有一份 `Arc<Node, IndexAllocator>`，而 `Node` 与 `IndexAllocator` 均为 `Send`。
unsafe impl Send for NodeRawTicket {}

/// 全局文件页 LRU 中的单条侵入式双向链表队列。
struct Queue {
    /// 队首节点指针（最旧的节点，回收/老化扫描从队首开始；空队列时为 `null`）。
    first: *const Node,
    /// 队尾节点指针（最新加入或刚被提拔的节点；空队列时为 `null`）。
    last: *const Node,
    /// 队列中当前的节点数量。
    len: usize,
}

impl Queue {
    /// 创建一个空的侵入式双向链表队列。
    const fn new() -> Self {
        Self {
            first: ptr::null(),
            last: ptr::null(),
            len: 0,
        }
    }
}

/// 全局磁盘文件页的双队列 LRU 容器。
struct FileLru {
    /// 非活跃队列（新插入页或从活跃队列降级的页，回收扫描优先从此处隔离）。
    inactive: Queue,
    /// 活跃队列（在非活跃队列中被再次访问而提拔的热页）。
    active: Queue,
}

// SAFETY: `FileLru` 中的所有裸指针均指向由各自 `LruLinks::ticket` 保活的 `Node`，且只在 `FILE_LRU` 自旋锁下访问。
unsafe impl Send for FileLru {}

static FILE_LRU: SpinLock<FileLru> = SpinLock::new(FileLru {
    inactive: Queue::new(),
    active: Queue::new(),
});

impl FileLru {
    /// 在持有 `FILE_LRU` 自旋锁的上下文中，获取指定节点指针的 `LruLinks` 可变引用。
    fn links(&mut self, pointer: *const Node) -> &mut LruLinks {
        assert!(!pointer.is_null(), "null LRU node");
        // SAFETY:
        // 1. 调用方持有 `&mut FileLru`（即持有全局 `FILE_LRU` 自旋锁），排除了对任何节点 `lru` 字段的并发访问。
        // 2. `pointer` 指向的 `Node` 由链表内部的 `ticket` 或调用方持有的 `NodeRef` 保证存活且非空。
        // 3. 仅访问 `lru` 字段所在的 `UnsafeCell`，不触碰 `index` 字段。
        unsafe { &mut *(*pointer).lru.get() }
    }

    /// 根据队列类型枚举返回对应的 `inactive` 或 `active` 队列可变引用。
    fn queue(&mut self, kind: LruKind) -> &mut Queue {
        match kind {
            LruKind::Inactive => &mut self.inactive,
            LruKind::Active => &mut self.active,
            _ => panic!("unlinked LRU kind"),
        }
    }

    /// 将 `pointer` 指向的节点从其当前所在的 LRU 队列中摘除，将其状态更新为 `next_kind`，并返回其保活凭证。
    fn unlink(&mut self, pointer: *const Node, next_kind: LruKind) -> NodeRawTicket {
        let links = self.links(pointer);
        let kind = links.kind;
        let previous = links.previous;
        let next = links.next;
        let ticket = links
            .ticket
            .take()
            .expect("linked LRU node has no raw ticket");
        links.kind = next_kind;
        links.previous = ptr::null();
        links.next = ptr::null();
        if !previous.is_null() {
            self.links(previous).next = next;
        }
        if !next.is_null() {
            self.links(next).previous = previous;
        }
        let queue = self.queue(kind);
        if previous.is_null() {
            queue.first = next;
        }
        if next.is_null() {
            queue.last = previous;
        }
        queue.len = queue.len.checked_sub(1).expect("LRU length underflow");
        ticket
    }

    /// 将持有 `ticket` 保活凭证的节点挂入指定 `kind`（`Inactive` 或 `Active`）队列的队尾。
    fn push(&mut self, ticket: NodeRawTicket, kind: LruKind) {
        let pointer = ticket.pointer;
        let last = self.queue(kind).last;
        let links = self.links(pointer);
        assert!(
            links.ticket.is_none(),
            "LRU insertion retains a member ticket"
        );
        assert!(
            matches!(links.kind, LruKind::Detached | LruKind::Isolated),
            "LRU node already linked"
        );
        links.kind = kind;
        links.previous = last;
        links.next = ptr::null();
        links.ticket = Some(ticket);
        if !last.is_null() {
            self.links(last).next = pointer;
        }
        let queue = self.queue(kind);
        if queue.first.is_null() {
            queue.first = pointer;
        }
        queue.last = pointer;
        queue.len += 1;
    }

    /// 当节点从索引树中彻底移除时，将其从全局 LRU 中分离：
    /// - 若节点仍在 `Inactive` 或 `Active` 队列中，直接摘除并返回其 LRU 强引用供锁外释放；
    /// - 若节点正处于被回收线程 `Isolated` 的状态，则仅将其状态标记为 `Detached`，由回收线程稍后在 `putback` 中释放。
    fn detach(&mut self, node: &NodeRef) -> Option<NodeRef> {
        let pointer = Arc::as_ptr(node);
        match self.links(pointer).kind {
            LruKind::Inactive | LruKind::Active => {
                Some(self.unlink(pointer, LruKind::Detached).into_node())
            }
            LruKind::Isolated => {
                self.links(pointer).kind = LruKind::Detached;
                None
            }
            LruKind::Detached => None,
        }
    }
}

/// 从全局 `inactive` 队列头部扫描并尝试隔离一个未被访问的候选文件页节点。
///
/// 若队首节点近期被访问过（`take_accessed` 返回 `referenced == true`），则将其移至 `inactive` 队尾
/// 或提拔至 `active` 队尾并返回 `None`；若未被访问且回收预算允许，则将其状态设为 `Isolated` 并返回其 `NodeRef`。
pub(super) fn isolate_inactive(attempt: &mut ReclaimAttempt) -> Option<NodeRef> {
    let mut lru = FILE_LRU.lock();
    let pointer = lru.inactive.first;
    if pointer.is_null() || !attempt.scan() {
        return None;
    }
    // SAFETY: `pointer` 是 `inactive.first` 且非空，该节点由其自身的 `LruLinks::ticket` 强引用保活，
    // 且当前持有 `FILE_LRU` 自旋锁防止其被并发摘除；此处仅读取不可变的 `folio` 字段及其原子访问标志位。
    let (referenced, activate) = unsafe { (*pointer).folio.take_accessed() };
    if referenced {
        let ticket = lru.unlink(pointer, LruKind::Isolated);
        lru.push(
            ticket,
            if activate {
                LruKind::Active
            } else {
                LruKind::Inactive
            },
        );
        return None;
    }
    if !attempt.isolate() {
        return None;
    }
    Some(lru.unlink(pointer, LruKind::Isolated).into_node())
}

/// 从全局 `active` 队列头部扫描最多 `count` 个节点进行老化检查：
/// 若节点近期被访问过则放回 `active` 队尾，否则降级至 `inactive` 队尾。
pub(super) fn age_active(attempt: &mut ReclaimAttempt, count: usize) {
    for _ in 0..count {
        let mut lru = FILE_LRU.lock();
        let pointer = lru.active.first;
        if pointer.is_null() || !attempt.scan() {
            break;
        }
        // SAFETY: `pointer` 是 `active.first` 且非空，由其队列内的 `LruLinks::ticket` 强引用保活，
        // 当前持有 `FILE_LRU` 自旋锁防止并发摘除；此处仅访问不可变的 `folio` 字段及其原子访问位。
        let (referenced, _) = unsafe { (*pointer).folio.take_accessed() };
        let ticket = lru.unlink(pointer, LruKind::Isolated);
        lru.push(
            ticket,
            if referenced {
                LruKind::Active
            } else {
                LruKind::Inactive
            },
        );
    }
}

/// 获取当前全局 `inactive` 和 `active` 队列的节点数量快照，用于限定单轮回收扫描的最大步数。
pub(super) fn lru_lengths() -> (usize, usize) {
    let lru = FILE_LRU.lock();
    (lru.inactive.len, lru.active.len)
}

/// 将此前被 `isolate_inactive` 隔离出的节点归还给 LRU 或在锁外彻底释放，返回实际释放的物理页数。
///
/// - 若节点状态仍为 `Isolated`（说明索引树未在隔离期间移除它），则将其重新挂回 `Inactive` 队尾；
/// - 若节点状态已变为 `Detached`（说明在隔离期间文件被截断或关闭，已从索引树摘除），则在释放 `FILE_LRU` 锁后释放该节点。
pub(super) fn putback(node: NodeRef) -> usize {
    let mut lru = FILE_LRU.lock();
    let pointer = Arc::as_ptr(&node);
    if lru.links(pointer).kind == LruKind::Isolated {
        lru.push(NodeRawTicket::new(node), LruKind::Inactive);
        0
    } else {
        assert!(
            lru.links(pointer).kind == LruKind::Detached,
            "putback of a linked node"
        );
        drop(lru);
        release_node(node)
    }
}

/// 在持有全局 `FILE_LRU` 自旋锁的上下文中执行最终的干净页引用核验与索引摘除回调 `retire`。
///
/// 调用方在调用本函数前通常已持有 `FolioIndex::entries` 锁（遵循 `entries` ➔ `FILE_LRU` 的锁顺序），
/// `retire` 闭包内严禁睡眠或执行物理页释放。
pub(super) fn with_lru<T>(retire: impl FnOnce() -> T) -> T {
    let _lru = FILE_LRU.lock();
    retire()
}

/// 释放一份 `NodeRef` 强引用，并返回本次操作实际归还给物理页分配器的页数（0 或 1）。
///
/// 注意：当使用自定义分配器 `IndexAllocator::Metadata` 时，若 `Arc::into_inner` 遇到非最后一个强引用，
/// 标准库实现不会析构克隆出的分配器副本；因此对 `Metadata` 节点直接 `drop(node)`（元数据页常驻预留池，不归还伙伴系统），
/// 仅对 `Heap` 节点使用 `Arc::into_inner` 提取 `Folio` 并统计释放的物理页数。
pub(super) fn release_node(node: NodeRef) -> usize {
    if matches!(Arc::allocator(&node), IndexAllocator::Metadata { .. }) {
        drop(node);
        return 0;
    }
    Arc::into_inner(node).map_or(0, |node| node.folio.release())
}
/// 基于 AVL 自平衡二叉搜索树的页缓存稀疏索引。
pub(super) struct Tree {
    /// AVL 树的根节点。
    root: Link,
    /// 树中当前存储的节点总数。
    pub len: usize,
    /// 标记该索引是否已关闭（关闭后拒绝插入新节点）。
    pub closed: bool,
}

impl Tree {
    /// 创建一个空的 AVL 稀疏索引树。
    pub const fn new() -> Self {
        Self {
            root: None,
            len: 0,
            closed: false,
        }
    }

    /// 按键精确查找节点，命中时返回节点的只读引用。
    pub fn get(&self, key: u64) -> Option<&Node> {
        let mut cursor = self.root.as_deref();
        while let Some(node) = cursor {
            if key == node.key {
                return Some(node);
            }
            cursor = if key < node.key {
                node.links().left.as_deref()
            } else {
                node.links().right.as_deref()
            };
        }
        None
    }

    /// 按键精确查找节点并返回其 `Weak` 弱引用（仅用于内核测试观察节点控制块生命周期，不增加强引用计数）。
    #[cfg(feature = "ktest")]
    pub fn weak_node(&self, key: u64) -> Option<alloc::sync::Weak<Node, IndexAllocator>> {
        let mut cursor = self.root.as_ref();
        while let Some(node) = cursor {
            if key == node.key {
                return Some(Arc::downgrade(node));
            }
            cursor = if key < node.key {
                node.links().left.as_ref()
            } else {
                node.links().right.as_ref()
            };
        }
        None
    }

    /// 查找树中键值大于或等于 `key` 的最小节点（用于按序范围扫描与分段回写）。
    pub fn lower_bound(&self, key: u64) -> Option<&Node> {
        let mut cursor = self.root.as_deref();
        let mut found = None;
        while let Some(node) = cursor {
            if node.key < key {
                cursor = node.links().right.as_deref();
            } else {
                found = Some(node);
                cursor = node.links().left.as_deref();
            }
        }
        found
    }

    /// 设置或清除指定键对应节点的 `to_write` 回写选中标记。
    pub fn set_to_write(&mut self, key: u64, selected: bool) {
        let mut write = IndexWrite;
        if let Some(node) = self.get(key) {
            node.links_mut(&mut write).to_write = selected;
        }
    }

    /// 将预先构造好的新节点插入 AVL 树；若该节点属于磁盘文件（`FileRole::Disk`），则同时将其挂入全局 `FILE_LRU` 的 `Inactive` 队列。
    pub fn insert(&mut self, node: NodeRef) {
        if node.role.as_ref().is_some_and(FileRole::is_disk) {
            FILE_LRU
                .lock()
                .push(NodeRawTicket::new(Arc::clone(&node)), LruKind::Inactive);
        }
        self.root = Some(insert(self.root.take(), node, &mut IndexWrite));
        self.len += 1;
    }

    /// 从 AVL 树中移除指定键的节点，并同步将其从全局 `FILE_LRU` 中分离，返回打包后的 `RetiredNode` 供锁外释放。
    pub fn remove(&mut self, key: u64) -> Option<RetiredNode> {
        let (root, removed) = remove(self.root.take(), key, &mut IndexWrite);
        self.root = root;
        let node = removed?;
        self.len -= 1;
        let member = FILE_LRU.lock().detach(&node);
        Some(RetiredNode { node, member })
    }

    /// 在已持有 `FolioIndex::entries` 和全局 `FILE_LRU` 双锁（即 `with_lru` 内部）的上下文中，
    /// 从 AVL 树移除一个已被回收线程隔离（`Isolated`）的节点，并将其 LRU 状态更新为 `Detached`。
    pub fn remove_isolated(&mut self, key: u64) -> Option<RetiredNode> {
        let (root, removed) = remove(self.root.take(), key, &mut IndexWrite);
        self.root = root;
        let node = removed?;
        self.len -= 1;
        // SAFETY: 本方法仅在 `with_lru` 闭包内被调用，调用方已持有全局 `FILE_LRU` 自旋锁，
        // 且 `node` 由刚从树上摘下的强引用与回收线程的隔离强引用共同保活；此处仅修改 `lru` 投影。
        let links = unsafe { &mut *node.lru.get() };
        assert!(
            links.kind == LruKind::Isolated,
            "final reclaim has no isolated ticket"
        );
        links.kind = LruKind::Detached;
        Some(RetiredNode { node, member: None })
    }
}

impl Drop for Tree {
    /// 析构整棵索引树：逐个弹出最左节点并从全局 `FILE_LRU` 分离，防止残留的树链接导致长链保活。
    fn drop(&mut self) {
        while let Some(node) = self.root.take() {
            let (root, node) = pop_first(node, &mut IndexWrite);
            self.root = root;
            let member = FILE_LRU.lock().detach(&node);
            RetiredNode { node, member }.release();
        }
        self.len = 0;
    }
}
/// 获取可选子树根节点的高度（空子树返回 0）。
fn height(link: &Link) -> u16 {
    link.as_ref().map_or(0, |node| node.links().height)
}

/// 根据左右子树的当前高度重新计算并更新 `node` 的树高。
fn refresh(node: &Node, write: &mut IndexWrite) {
    let links = node.links_mut(write);
    links.height = 1 + height(&links.left).max(height(&links.right));
}

/// 对以 `root` 为根的子树执行 AVL 右旋（将左子节点提升为新的子树根）。
fn rotate_right(root: NodeRef, write: &mut IndexWrite) -> NodeRef {
    let next = root
        .links_mut(write)
        .left
        .take()
        .expect("AVL right rotation has no left child");
    root.links_mut(write).left = next.links_mut(write).right.take();
    refresh(&root, write);
    next.links_mut(write).right = Some(root);
    refresh(&next, write);
    next
}

/// 对以 `root` 为根的子树执行 AVL 左旋（将右子节点提升为新的子树根）。
fn rotate_left(root: NodeRef, write: &mut IndexWrite) -> NodeRef {
    let next = root
        .links_mut(write)
        .right
        .take()
        .expect("AVL left rotation has no right child");
    root.links_mut(write).right = next.links_mut(write).left.take();
    refresh(&root, write);
    next.links_mut(write).left = Some(root);
    refresh(&next, write);
    next
}

/// 在插入或删除节点后重算 `root` 的高度，若左右子树高度差超过 1 则通过单旋或双旋恢复 AVL 平衡。
fn balance(root: NodeRef, write: &mut IndexWrite) -> NodeRef {
    refresh(&root, write);
    if height(&root.links().right) + 1 < height(&root.links().left) {
        let left = root.links().left.as_ref().unwrap();
        if height(&left.links().left) < height(&left.links().right) {
            let left = root.links_mut(write).left.take().unwrap();
            root.links_mut(write).left = Some(rotate_left(left, write));
        }
        return rotate_right(root, write);
    }
    if height(&root.links().left) + 1 < height(&root.links().right) {
        let right = root.links().right.as_ref().unwrap();
        if height(&right.links().right) < height(&right.links().left) {
            let right = root.links_mut(write).right.take().unwrap();
            root.links_mut(write).right = Some(rotate_right(right, write));
        }
        return rotate_left(root, write);
    }
    root
}

/// 递归将新节点 `node` 按键序插入以 `root` 为根的子树，并在回溯路径上恢复 AVL 平衡。
fn insert(root: Link, node: NodeRef, write: &mut IndexWrite) -> NodeRef {
    let Some(root) = root else {
        return node;
    };
    assert_ne!(root.key, node.key, "duplicate folio index key");
    if node.key < root.key {
        let child = root.links_mut(write).left.take();
        root.links_mut(write).left = Some(insert(child, node, write));
    } else {
        let child = root.links_mut(write).right.take();
        root.links_mut(write).right = Some(insert(child, node, write));
    }
    balance(root, write)
}

/// 从非空子树 `root` 中摘除键值最小的（最左）节点，返回平衡后的剩余子树与被摘下的最左节点。
fn pop_first(root: NodeRef, write: &mut IndexWrite) -> (Link, NodeRef) {
    let Some(left) = root.links_mut(write).left.take() else {
        return (root.links_mut(write).right.take(), root);
    };
    let (left, removed) = pop_first(left, write);
    root.links_mut(write).left = left;
    (Some(balance(root, write)), removed)
}

/// 从子树 `root` 中查找并移除键为 `key` 的节点，返回平衡后的新子树根与被移除的节点。
///
/// 若目标节点同时拥有左右子树，则通过 `pop_first` 取出右子树的最小后继节点来接替根位置，
/// 直接复用原节点的 `Arc<Node>` 而不修改其不可变的页身份。
fn remove(root: Link, key: u64, write: &mut IndexWrite) -> (Link, Link) {
    let Some(root) = root else {
        return (None, None);
    };
    if key < root.key {
        let (left, removed) = remove(root.links_mut(write).left.take(), key, write);
        root.links_mut(write).left = left;
        return (Some(balance(root, write)), removed);
    }
    if root.key < key {
        let (right, removed) = remove(root.links_mut(write).right.take(), key, write);
        root.links_mut(write).right = right;
        return (Some(balance(root, write)), removed);
    }
    let left = root.links_mut(write).left.take();
    let Some(right) = root.links_mut(write).right.take() else {
        return (left, Some(root));
    };
    let (right, next) = pop_first(right, write);
    next.links_mut(write).left = left;
    next.links_mut(write).right = right;
    (Some(balance(next, write)), Some(root))
}
