//! 同一 AVL 节点保存不可变页身份，索引链与全局 file LRU 分别固定投影。

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

/// 同一真实 Arc 控制来源；Heap 不拥有资源，元数据保存具体源及原槽。
#[derive(Clone)]
pub(crate) enum IndexAllocator {
    Heap,
    Metadata {
        reserve: Arc<BioReserve>,
        slot: usize,
    },
}

// SAFETY: Heap 委托原 Global；Metadata 仅返回指定槽的真实原布局存储，
// 源在全部 allocator 和 Weak 尾结束前保活，deallocate 按原槽及布局归还一次。
unsafe impl Allocator for IndexAllocator {
    /// 真正 Arc 分配提供实际控制布局，元数据 ready 后不再进入标准堆。
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => Global.allocate(layout),
            Self::Metadata { reserve, slot } => reserve.allocate_metadata_control(*slot, layout),
        }
    }

    /// 仅 allocator 的真 deallocate 归还原控制块，不把 Node payload Drop 当作末尾。
    unsafe fn deallocate(&self, pointer: NonNull<u8>, layout: Layout) {
        match self {
            Self::Heap => {
                // SAFETY: Arc 只归还本 allocator 原先接受的 live 指针和完全相同布局，
                // 全部强弱引用已结束，Global 接收唯一堆归还权。
                unsafe { Global.deallocate(pointer, layout) };
            }
            Self::Metadata { reserve, slot } => {
                // SAFETY: pointer/layout 来自同 source/slot 的 allocate，Arc 全强弱尾
                // 已结束，未重复归还；源 allocator 拥有保持原 raw 存储有效。
                unsafe { reserve.deallocate_metadata_control(*slot, pointer, layout) };
            }
        }
    }
}

pub(crate) type NodeRef = Arc<Node, IndexAllocator>;
pub(super) type Link = Option<NodeRef>;

struct IndexLinks {
    to_write: bool,
    height: u16,
    left: Link,
    right: Link,
}

/// 所属 Tree 独占操作的短借用凭证；任何链接可变投影都借用同一凭证。
struct IndexWrite;

#[derive(Clone, Copy, PartialEq, Eq)]
enum LruKind {
    Detached,
    Inactive,
    Active,
    Isolated,
}

struct LruLinks {
    kind: LruKind,
    previous: *const Node,
    next: *const Node,
    ticket: Option<NodeRawTicket>,
}

/// 文件页节点的后端身份；RAM 节点不进入磁盘 LRU 或写回队列。
pub(crate) enum FileRole {
    Disk(InodeWeak),
    Memory(InodeWeak),
}

impl FileRole {
    /// 返回节点是否拥有磁盘文件回收责任。
    pub(crate) fn is_disk(&self) -> bool {
        matches!(self, Self::Disk(_))
    }

    /// 借用节点的 canonical inode 弱身份。
    pub(crate) fn owner(&self) -> &InodeWeak {
        match self {
            Self::Disk(owner) | Self::Memory(owner) => owner,
        }
    }
}

/// key、原 folio 和真实 file 弱身份在本控制生命周期内不可更换。
pub(crate) struct Node {
    pub key: u64,
    pub folio: FolioRef,
    pub role: Option<FileRole>,
    index: UnsafeCell<IndexLinks>,
    lru: UnsafeCell<LruLinks>,
}

// SAFETY: 只移动 Arc 拥有，固定 Node 的不可变身份不改；index 投影由所属 Tree
// 独占访问，lru 投影只由全局 LRU 锁访问；两个 UnsafeCell 不相互重叠。
unsafe impl Send for Node {}
// SAFETY: Folio/弱身份可共享；所有可变链接遵守分别的锁域，从不产生 whole &mut Node。
unsafe impl Sync for Node {}

/// 原索引及可能的 LRU 成员拥有一起锁外退休，隔离者持自己的同一原票。
pub(super) struct RetiredNode {
    node: NodeRef,
    member: Option<NodeRef>,
}

impl Deref for RetiredNode {
    type Target = Node;
    /// 只借用不可变身份，索引及 LRU 链在返回退休对象前均已正式撤离。
    fn deref(&self) -> &Node {
        &self.node
    }
}

impl RetiredNode {
    /// 两份真实 Node 拥有均结束后才可能归页，实际页计数由原 folio 最后 put 返回。
    pub(super) fn release(self) -> usize {
        let mut freed = self.member.map_or(0, release_node);
        freed += release_node(self.node);
        freed
    }
}

impl Node {
    /// 在全部索引及 LRU 锁外按真实 allocator 准备不可变节点，不猜控制布局。
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
        // SAFETY: genuine uninit Arc 仍私有且无 Weak，write 已初始化 Node 全字段，
        // 从此只共享不可变 Node，两个链接仅用各自固定投影。
        Ok(unsafe { prepared.assume_init() })
    }

    /// bootstrap 真正请求 Arc 的控制布局并立即结束私有空控制，ready 后复用原块。
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

    /// 只在所属 Tree 的共享访问期读取索引链接，LRU 不接触此投影。
    fn links(&self) -> &IndexLinks {
        // SAFETY: 本私有投影只被 Tree/AVL 的共享借用访问；Tree 的可变操作
        // 必须独占其访问域，所有对外 Node 引用都不提供 index 投影。
        unsafe { &*self.index.get() }
    }

    /// 只在所属 Tree 独占操作期间改链接，已有 Arc 或 LRU 借用不覆盖该 UnsafeCell。
    fn links_mut<'a>(&'a self, _write: &'a mut IndexWrite) -> &'a mut IndexLinks {
        // SAFETY: 全部调用来自 Tree 的独占 AVL 操作，旋转只移动同 Tree 子链；
        // LRU 与页读者不访问 index，不创建整个 Node 的可变借用。
        unsafe { &mut *self.index.get() }
    }

    /// 本轮待写只在索引共享锁域读取，页脏与 WB 状态仍来自原 folio。
    pub(super) fn to_write(&self) -> bool {
        self.links().to_write
    }
}

/// 完整移动原 pointer 和 allocator 值，裸定位指针不构成第二份拥有。
struct NodeRawTicket {
    pointer: *const Node,
    allocator: Option<IndexAllocator>,
}

impl NodeRawTicket {
    /// 把一份真实 Node Arc 交 LRU，Heap 与 Metadata 都保持原 allocator 来源。
    fn new(node: NodeRef) -> Self {
        let (pointer, allocator) = Arc::into_raw_with_allocator(node);
        Self {
            pointer,
            allocator: Some(allocator),
        }
    }

    /// 消费同一原票恢复拥有，不 clone allocator、不用默认 from_raw 猜来源。
    fn into_node(mut self) -> NodeRef {
        let allocator = self
            .allocator
            .take()
            .expect("node raw ticket consumed twice");
        // SAFETY: pointer 来自此票的 into_raw_with_allocator，尚未恢复或释放，
        // 原 allocator 完整移动，活强拥有保 Node 和控制，恢复只消费一次。
        unsafe { Arc::from_raw_in(self.pointer, allocator) }
    }
}

impl Drop for NodeRawTicket {
    /// 未交出的原票也只消费一次原 Arc；调用者须在 LRU 短锁外结束票。
    fn drop(&mut self) {
        if let Some(allocator) = self.allocator.take() {
            // SAFETY: Some 表示此票仍唯一持有原 raw 强引用和 allocator，
            // 此路径没有先前恢复；所有 Node 字段访问已结束，普通 Drop 结束原拥有。
            drop(unsafe { Arc::from_raw_in(self.pointer, allocator) });
        }
    }
}

// SAFETY: pointer 保有一份实际 Send Node 强引用，allocator 同票移动；链接只在 LRU 锁域访问。
unsafe impl Send for NodeRawTicket {}

struct Queue {
    first: *const Node,
    last: *const Node,
    len: usize,
}

impl Queue {
    /// 空队列不申请存储，每个成员用自己的固定 LRU 投影保存原票。
    const fn new() -> Self {
        Self {
            first: ptr::null(),
            last: ptr::null(),
            len: 0,
        }
    }
}

struct FileLru {
    inactive: Queue,
    active: Queue,
}

// SAFETY: 所有裸链接和成员原票只在 FILE_LRU 唯一锁域访问，每个链上地址均由原票保活。
unsafe impl Send for FileLru {}

static FILE_LRU: SpinLock<FileLru> = SpinLock::new(FileLru {
    inactive: Queue::new(),
    active: Queue::new(),
});

impl FileLru {
    /// 原 LRU 锁下固定投影单个成员，不访问 index 或创建整个 Node 的可变借用。
    fn links(&mut self, pointer: *const Node) -> &mut LruLinks {
        assert!(!pointer.is_null(), "null LRU node");
        // SAFETY: 调用者持 FILE_LRU 独占 guard，pointer 由成员/隔离/调用者的真实
        // Node Arc 保活；其他写者遵守同锁，仅投影独立 lru UnsafeCell。
        unsafe { &mut *(*pointer).lru.get() }
    }

    /// 按已核验成员状态定位原队列，隔离与 Detached 不属于任何队列。
    fn queue(&mut self, kind: LruKind) -> &mut Queue {
        match kind {
            LruKind::Inactive => &mut self.inactive,
            LruKind::Active => &mut self.active,
            _ => panic!("unlinked LRU kind"),
        }
    }

    /// 撤原成员并转移它的同一票，链上邻居都由各自原票保活。
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

    /// 原票移入队尾，不产生新页引用或新的 canonical inode 使用者。
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

    /// 索引最后关闭也正式 detach，隔离者只看到 Detached，不能重新复活旧 canonical。
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

/// 隔离只移 Node 原票，第一次观察不增加 folio refs，也不阻止 inode 最后使用。
pub(super) fn isolate_inactive(attempt: &mut ReclaimAttempt) -> Option<NodeRef> {
    let mut lru = FILE_LRU.lock();
    let pointer = lru.inactive.first;
    if pointer.is_null() || !attempt.scan() {
        return None;
    }
    // SAFETY: inactive.first 由该节点的原成员票保活，当前 LRU 锁排除摘链；
    // 这里只读不可变 folio 身份及原子访问位，不访问 index 投影。
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

/// 有限老化 active 入口快照，未访问成员降至 inactive，当前轮不再次扫描降级者。
pub(super) fn age_active(attempt: &mut ReclaimAttempt, count: usize) {
    for _ in 0..count {
        let mut lru = FILE_LRU.lock();
        let pointer = lru.active.first;
        if pointer.is_null() || !attempt.scan() {
            break;
        }
        // SAFETY: 原 active 成员票保活固定 Node，LRU 锁排除链变化，访问位是独立原子。
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

/// 当前队列长度只限制本轮入口前缀，之后新插入不会延长该轮工作。
pub(super) fn lru_lengths() -> (usize, usize) {
    let lru = FILE_LRU.lock();
    (lru.inactive.len, lru.active.len)
}

/// 只有仍 Isolated 的同票可 putback；被索引退休的票在 LRU 锁外结束。
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

/// 持原 LRU 锁串行最终 PageUse 核验和索引摘离，retire 不可析构或睡眠。
pub(super) fn with_lru<T>(retire: impl FnOnce() -> T) -> T {
    let _lru = FILE_LRU.lock();
    retire()
}

/// 非末 into_inner 在固定 nightly 不析构 owning allocator；仅 Heap(unit) 允许该消费。
pub(super) fn release_node(node: NodeRef) -> usize {
    if matches!(Arc::allocator(&node), IndexAllocator::Metadata { .. }) {
        drop(node);
        return 0;
    }
    Arc::into_inner(node).map_or(0, |node| node.folio.release())
}

pub(super) struct Tree {
    root: Link,
    pub len: usize,
    pub closed: bool,
}

impl Tree {
    /// 建立空稀疏树，不按文件长度预留槽或节点。
    pub const fn new() -> Self {
        Self {
            root: None,
            len: 0,
            closed: false,
        }
    }

    /// 命中只借用原节点，调用者持原索引共享访问域。
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

    /// 观察原 Node 的真实 Weak 控制尾；索引锁持有期间只增加弱拥有，不 clone 页引用。
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

    /// 返回不小于起点的首节点，范围扫描无需复制整张索引。
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

    /// 独占索引投影修改本轮待写，不建立第二张脏页表。
    pub fn set_to_write(&mut self, key: u64, selected: bool) {
        let mut write = IndexWrite;
        if let Some(node) = self.get(key) {
            node.links_mut(&mut write).to_write = selected;
        }
    }

    /// 接入锁外准备的唯一叶，真实 file 节点同一 index→LRU 顺序登记原票。
    pub fn insert(&mut self, node: NodeRef) {
        if node.role.as_ref().is_some_and(FileRole::is_disk) {
            FILE_LRU
                .lock()
                .push(NodeRawTicket::new(Arc::clone(&node)), LruKind::Inactive);
        }
        self.root = Some(insert(self.root.take(), node, &mut IndexWrite));
        self.len += 1;
    }

    /// 摘原 canonical 并正式 detach，全部拥有交锁外返回，不在索引内归页。
    pub fn remove(&mut self, key: u64) -> Option<RetiredNode> {
        let (root, removed) = remove(self.root.take(), key, &mut IndexWrite);
        self.root = root;
        let node = removed?;
        self.len -= 1;
        let member = FILE_LRU.lock().detach(&node);
        Some(RetiredNode { node, member })
    }

    /// 已持 index→LRU 时只改索引，隔离票由扫描者持有，不再次获取 LRU 锁。
    pub fn remove_isolated(&mut self, key: u64) -> Option<RetiredNode> {
        let (root, removed) = remove(self.root.take(), key, &mut IndexWrite);
        self.root = root;
        let node = removed?;
        self.len -= 1;
        // SAFETY: 本入口只被 with_lru 内最终核验调用，持全局 LRU 锁，node
        // 由原索引和隔离票保活；只修改独立 lru 投影，隔离成员已无链票。
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
    /// 逐个移走最左叶并 detach，不让 LRU 保活包含仍链接子树的关闭节点。
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

/// 空子树高度为零，只读所属索引固定投影。
fn height(link: &Link) -> u16 {
    link.as_ref().map_or(0, |node| node.links().height)
}

/// 子链变更后只重算本节点，不遍历其他子树。
fn refresh(node: &Node, write: &mut IndexWrite) {
    let links = node.links_mut(write);
    links.height = 1 + height(&links.left).max(height(&links.right));
}

/// 提升左子节点，只转移原 Arc 链，页身份与 LRU 投影保持固定。
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

/// 提升右子节点，旋转不分配，也不取得整个 Node 的可变借用。
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

/// 单次插入或删除恢复 AVL 高度差，原 file LRU 成员不随旋转换身份。
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

/// 准备叶沿键序移入，返回路径只修改独立 index 投影。
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

/// 摘最左节点，返回节点已经没有子链，不会由隔离 Arc 保活另一棵树。
fn pop_first(root: NodeRef, write: &mut IndexWrite) -> (Link, NodeRef) {
    let Some(left) = root.links_mut(write).left.take() else {
        return (root.links_mut(write).right.take(), root);
    };
    let (left, removed) = pop_first(left, write);
    root.links_mut(write).left = left;
    (Some(balance(root, write)), removed)
}

/// 双子删除只移植后继原节点，不复制 immutable key/folio 或 whole Node。
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
