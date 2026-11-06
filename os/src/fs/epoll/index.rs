//! `epoll` 监听项（[`Entry`]）的侵入式 AVL 平衡二叉搜索树索引。
//!
//! # 模块职责与架构定位
//! 本模块作为 `os/src/fs/epoll.rs` 的内部子模块，负责维护单个 [`super::EpollFile`]
//! 中所有已注册监听项的查找索引（Interest Set），供 `EPOLL_CTL_ADD`、`EPOLL_CTL_MOD`、
//! `EPOLL_CTL_DEL`、反向注销（`release_key`）以及实例销毁时按 [`Key`]（底层文件对象地址 + 注册 `fd`）
//! 进行 $O(\log N)$ 查找、插入与摘除。
//!
//! # 核心设计与内存布局
//!
//! ## 1. 零额外分配的侵入式 `Arc` 树
//! 每个 [`Entry`] 内嵌 [`Links`] 字段，树边直接使用 `Option<Arc<Entry>>` 表示父节点对子节点的强引用所有权。
//! 新节点在插入树之前已在外部完成分配，因此 [`Tree::insert`]、[`Tree::remove`] 以及所有 AVL 旋转操作
//! 仅交换既有的 `Arc<Entry>` 指针，全程不触发任何堆内存分配。
//!
//! ```text
//! Tree (受 EpollFile::control 睡眠互斥锁保护)
//!   └── root: Option<Arc<Entry>>
//!                      ├── key: (object_addr, fd)
//!                      └── index: Links(UnsafeCell<IndexLinks>)
//!                                   ├── height: u16
//!                                   ├── left:  Option<Arc<Entry>>
//!                                   └── right: Option<Arc<Entry>>
//! ```
//!
//! ## 2. 双儿子节点删除的指针嫁接而非值拷贝
//! 传统二叉搜索树删除双儿子节点时常将中序后继的值覆盖到当前节点，但在 `epoll` 中，底层事件源回调
//! （`EntrySink`）和反向监控链（`WatchNode`）均直接持有特定 [`Entry`] 实例的指针。因此 [`remove`]
//! 采用 [`pop_first`] 将右子树的最左节点整颗摘出并嫁接到被删节点的位置，被删除节点的 `Arc<Entry>`
//! 地址保持不变并原样返回给调用方在锁外注销。
//!
//! # 锁获取顺序与并发契约
//! - 本模块的所有方法均要求调用方已持有所属 `EpollFile::control` 睡眠互斥锁（通过 `&Tree` 或 `&mut Tree` 借用体现）。
//! - [`Entry`] 同时还包含就绪队列字段（由 `ReadyCore::state` 自旋锁保护）和扫描链字段（由 `EpollFile::scan` 锁保护）；
//!   各锁保护各自独立的 `UnsafeCell` 字段，互不重叠。本模块通过零大小类型 [`IndexWrite`] 令牌静态限制只有
//!   `&mut Tree` 独占操作才能获取 [`IndexLinks`] 的可变引用。

use alloc::sync::Arc;
use core::cell::UnsafeCell;

use crate::fs::epoll::{Entry, Key};

/// 树节点子指针类型，持有子监听项的一份强引用所有权。
type Link = Option<Arc<Entry>>;

/// 单个监听项在 AVL 树中的高度与左右子树指针。
struct IndexLinks {
    /// 以当前节点为根的子树高度（空子树高度为 0，单个叶节点高度为 1）。
    height: u16,
    /// 左子树根节点（所有节点的 `key` 严格小于当前节点）。
    left: Link,
    /// 右子树根节点（所有节点的 `key` 严格大于当前节点）。
    right: Link,
}

/// 嵌入在 [`Entry`] 内部的 AVL 树链接字段包装器。
///
/// 仅在持有所属 `EpollFile::control` 睡眠锁期间通过 [`Tree`] 访问（元组字段 `0` 为受 `Tree` 与 `control` 互斥锁保护的 `IndexLinks`），与就绪候选队列存储完全隔离。
pub(super) struct Links(UnsafeCell<IndexLinks>);

impl Links {
    /// 创建一个尚未挂入树的孤立叶节点链接（初始高度为 1，无左右子树）。
    pub(super) const fn new() -> Self {
        Self(UnsafeCell::new(IndexLinks {
            height: 1,
            left: None,
            right: None,
        }))
    }

    /// 在持有 [`Tree`] 共享或独占借用期间读取节点的树链接状态。
    fn get(&self) -> &IndexLinks {
        // SAFETY:
        // 1. 本方法为模块私有，所有调用路径（`Tree::get`、`Tree::first`、`detached`、`height`、`balance`）
        //    均在调用方持有 `EpollFile::control` 睡眠互斥锁期间执行，不会与 `get_mut` 并发冲突；
        // 2. 异步通知回调与扫描路径从不访问 `Entry::index`，也不会构造整个 `Entry` 的 `&mut Entry` 引用。
        unsafe { &*self.0.get() }
    }

    /// 凭 [`IndexWrite`] 独占写令牌获取节点树链接状态的可变引用。
    fn get_mut<'a>(&'a self, _write: &'a mut IndexWrite) -> &'a mut IndexLinks {
        // SAFETY:
        // 1. `IndexWrite` 仅在 `Tree::insert` 与 `Tree::remove`（均持有 `&mut Tree`，即独占 `EpollFile::control` 锁）中构造；
        // 2. 同一时刻至多存在一个活跃的 `&mut IndexLinks` 借用（受 `'a` 生命周期约束），不与 `get()` 或其他可变借用重叠；
        // 3. 树外的其他 `Arc<Entry>` 持有者仅访问不可变身份字段或由其他锁保护的独立 `UnsafeCell` 字段。
        unsafe { &mut *self.0.get() }
    }

    /// 检查已摘除的节点是否已清空左右子树指针，确保析构时不再连带持有其他活跃监听项。
    pub(super) fn detached(&self) -> bool {
        let links = self.get();
        links.left.is_none() && links.right.is_none()
    }
}

/// 仅由 `&mut Tree` 内部操作构造的零大小写权限令牌，用于在编译期约束 [`Links::get_mut`] 的可变借用生命周期。
struct IndexWrite;

/// 管理单个 `epoll` 实例全部已注册监听项的 AVL 平衡二叉搜索树。
pub(super) struct Tree {
    /// AVL 树的根节点指针；空树时为 `None`。
    root: Link,
    /// 当前树中已注册监听项的总数量。
    pub len: usize,
}

impl Tree {
    /// 创建一棵不包含任何监听项的空 AVL 树。
    pub const fn new() -> Self {
        Self { root: None, len: 0 }
    }

    /// 在控制锁保护下按复合键 `key` 查找并返回对应监听项的共享引用。
    pub fn get(&self, key: Key) -> Option<&Arc<Entry>> {
        let mut cursor = self.root.as_ref();
        while let Some(node) = cursor {
            if key == node.key {
                return Some(node);
            }
            cursor = if key < node.key {
                node.index.get().left.as_ref()
            } else {
                node.index.get().right.as_ref()
            };
        }
        None
    }

    /// 返回树中键值最小（最左）的监听项强引用克隆，供 `EpollFile` 析构时分批取出并注销。
    pub fn first(&self) -> Option<Arc<Entry>> {
        let mut cursor = self.root.as_ref()?;
        while let Some(left) = cursor.index.get().left.as_ref() {
            cursor = left;
        }
        Some(Arc::clone(cursor))
    }

    /// 将已在锁外分配好的新监听项节点插入 AVL 树并重新平衡（全程零堆内存分配）。
    pub fn insert(&mut self, node: Arc<Entry>) {
        self.root = Some(insert(self.root.take(), node, &mut IndexWrite));
        self.len += 1;
    }

    /// 按 `key` 从树中摘除目标监听项并恢复 AVL 平衡，返回被摘除且已清空子指针的节点供调用方在锁外释放。
    pub fn remove(&mut self, key: Key) -> Link {
        let (root, removed) = remove(self.root.take(), key, &mut IndexWrite);
        self.root = root;
        if removed.is_some() {
            self.len -= 1;
        }
        removed
    }
}

impl Drop for Tree {
    /// 断言树在销毁前已被上层显式清空（所有监听项必须先走完撤销事件源与反向监控链的正式流程）。
    fn drop(&mut self) {
        assert!(
            self.root.is_none() && self.len == 0,
            "epoll interest tree not drained"
        );
    }
}

/// 读取子树根节点的高度；空子树返回 0。
fn height(link: &Link) -> u16 {
    link.as_ref().map_or(0, |node| node.index.get().height)
}

/// 根据左右子树的当前高度重新计算并更新 `node` 的高度。
fn refresh(node: &Entry, write: &mut IndexWrite) {
    let links = node.index.get_mut(write);
    links.height = 1 + height(&links.left).max(height(&links.right));
}

/// 对以 `root` 为根的子树执行单向右旋（提升左子节点为新根），更新旋转涉及节点的高度并返回新子树根。
fn rotate_right(root: Arc<Entry>, write: &mut IndexWrite) -> Arc<Entry> {
    let next = root
        .index
        .get_mut(write)
        .left
        .take()
        .expect("AVL right rotation has no child");
    root.index.get_mut(write).left = next.index.get_mut(write).right.take();
    refresh(&root, write);
    next.index.get_mut(write).right = Some(root);
    refresh(&next, write);
    next
}

/// 对以 `root` 为根的子树执行单向左旋（提升右子节点为新根），更新旋转涉及节点的高度并返回新子树根。
fn rotate_left(root: Arc<Entry>, write: &mut IndexWrite) -> Arc<Entry> {
    let next = root
        .index
        .get_mut(write)
        .right
        .take()
        .expect("AVL left rotation has no child");
    root.index.get_mut(write).right = next.index.get_mut(write).left.take();
    refresh(&root, write);
    next.index.get_mut(write).left = Some(root);
    refresh(&next, write);
    next
}

/// 在插入或删除回溯路径上更新 `root` 高度，若左右子树高度差达到 2 则通过单旋或双旋恢复 AVL 平衡性质。
fn balance(root: Arc<Entry>, write: &mut IndexWrite) -> Arc<Entry> {
    refresh(&root, write);
    if height(&root.index.get().right) + 1 < height(&root.index.get().left) {
        let left = root.index.get().left.as_ref().unwrap();
        if height(&left.index.get().left) < height(&left.index.get().right) {
            let left = root.index.get_mut(write).left.take().unwrap();
            root.index.get_mut(write).left = Some(rotate_left(left, write));
        }
        return rotate_right(root, write);
    }
    if height(&root.index.get().left) + 1 < height(&root.index.get().right) {
        let right = root.index.get().right.as_ref().unwrap();
        if height(&right.index.get().right) < height(&right.index.get().left) {
            let right = root.index.get_mut(write).right.take().unwrap();
            root.index.get_mut(write).right = Some(rotate_right(right, write));
        }
        return rotate_left(root, write);
    }
    root
}

/// 递归按 `node.key` 将新叶节点插入以 `root` 为根的子树，并在回溯时平衡各祖先节点（调用前须已排除重复键）。
fn insert(root: Link, node: Arc<Entry>, write: &mut IndexWrite) -> Arc<Entry> {
    let Some(root) = root else {
        return node;
    };
    assert!(root.key != node.key, "duplicate epoll interest key");
    if node.key < root.key {
        let child = root.index.get_mut(write).left.take();
        root.index.get_mut(write).left = Some(insert(child, node, write));
    } else {
        let child = root.index.get_mut(write).right.take();
        root.index.get_mut(write).right = Some(insert(child, node, write));
    }
    balance(root, write)
}

/// 从非空子树 `root` 中摘除键值最小（最左）的节点，返回 `(平衡后的剩余子树根, 被摘除且左右子指针均已清空的最左节点)`。
fn pop_first(root: Arc<Entry>, write: &mut IndexWrite) -> (Link, Arc<Entry>) {
    let Some(left) = root.index.get_mut(write).left.take() else {
        return (root.index.get_mut(write).right.take(), root);
    };
    let (left, removed) = pop_first(left, write);
    root.index.get_mut(write).left = left;
    (Some(balance(root, write)), removed)
}

/// 从以 `root` 为根的子树中查找并摘除键为 `key` 的节点，返回 `(平衡后的新子树根, 被摘除的目标节点)`。
fn remove(root: Link, key: Key, write: &mut IndexWrite) -> (Link, Link) {
    let Some(root) = root else {
        return (None, None);
    };
    // Step 1: 键值不相等时向对应左/右子树递归删除，并在回溯时重平衡当前节点。
    if key < root.key {
        let (left, removed) = remove(root.index.get_mut(write).left.take(), key, write);
        root.index.get_mut(write).left = left;
        return (Some(balance(root, write)), removed);
    }
    if root.key < key {
        let (right, removed) = remove(root.index.get_mut(write).right.take(), key, write);
        root.index.get_mut(write).right = right;
        return (Some(balance(root, write)), removed);
    }

    // Step 2: 命中目标节点；先移走其左右子树，使被摘除节点的左右子指针归零（满足 `detached` 断言）。
    let left = root.index.get_mut(write).left.take();
    let Some(right) = root.index.get_mut(write).right.take() else {
        return (left, Some(root));
    };

    // Step 3: 若右子树非空，摘出右子树中的最小节点（中序后继）顶替根位置，保留原节点的 Arc 地址不变。
    let (right, next) = pop_first(right, write);
    next.index.get_mut(write).left = left;
    next.index.get_mut(write).right = right;
    (Some(balance(next, write)), Some(root))
}
