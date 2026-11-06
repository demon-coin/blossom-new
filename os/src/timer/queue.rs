//! 高精度定时器（[`super::TimerKind::Precise`]）的侵入式左缓存红黑树实现。
//!
//! # 模块职责与架构定位
//! 本模块作为每核定时器调度基座（`TimerBase::state`）中的高精度队列后端，向上层 `timer` 模块提供：
//! - $O(1)$ 查询最早到期时间（[`Queue::first`]）与摘取最早到期节点（[`Queue::pop_due`]）；
//! - $O(\log N)$ 按单调时钟绝对截止时间（[`MonotonicInstant`]）插入节点（[`Queue::insert`]）；
//! - $O(\log N)$ 按已知节点指针直接摘除节点（[`Queue::remove`]），无需按时间重新搜索。
//!
//! # 核心设计与结构布局
//! `Queue` 采用侵入式设计，所有树边指针与颜色位直接存放在 `Timer` 内部 `Node` 的 [`Links`] 字段中，
//! 插入与删除过程零堆内存分配。同时维护 `first` 指针缓存整棵树的中序最左节点（最早到期节点）：
//! ```text
//! Queue
//! +-------+-------+
//! | first | root  |
//! +---+---+---+---+
//!     |       |
//!     |       v
//!     |     [Timer B (Black, expires = 200ns)]
//!     |      /                              \
//!     v     v                                v
//!   [Timer A (Red, expires = 100ns)]       [Timer C (Red, expires = 200ns)]
//! ```
//! - **重复截止时间（Duplicate Keys）约定**：当新插入节点的 `expires` 与树中已有节点相同时，
//!   统一沿右子树方向向下搜索插入，保证相同截止时间的节点按先入先出（FIFO）顺序排在原有节点右侧；
//!   红黑树旋转重平衡后，左右两侧子树均允许出现与父节点相等的 `expires`（即满足非严格闭区间
//!   `left.expires <= node.expires <= right.expires`）。
//! - **纯指针移植（No Payload Copy）**：删除拥有两个子节点的内部节点时，仅交换/移植其与右子树最小后继节点
//!   之间的 [`Links`] 指针与颜色，绝不按值复制 `Timer` 结构体或修改 `expires`、引用计数标志。
//!
//! # 锁与并发契约
//! - 本模块自身不包含锁；对 `Queue` 及其关联 [`Links`] 的一切读写操作，调用方必须已持有该队列所属 CPU
//!   的 `TimerBase::state` 自旋锁。
//! - 挂入树中的所有 `Timer` 节点在通过 `remove` 或 `pop_due` 摘除前，其内存地址必须保持固定且有效，
//!   且 `node.expires` 字段在入树期间严禁原地修改（更新截止时间必须先 `remove` 再修改并重新 `insert`）。

use core::ptr::{addr_of_mut, null};

use super::Timer;
use crate::time::MonotonicInstant;

/// 内嵌在每个 `Timer` 节点 `Node` 中的红黑树连接字段。
///
/// 仅记录树拓扑关系与节点颜色，不持有 `Timer` 的所有权，也不记录 `queued`/`running` 生命周期标志。
pub(super) struct Links {
    /// 父节点指针；若当前节点为树根或未入树，则为空指针（`null`）。
    parent: *const Timer,
    /// 左子节点指针（子树中所有节点的 `expires <= 当前节点.expires`）。
    left: *const Timer,
    /// 右子节点指针（子树中所有节点的 `expires >= 当前节点.expires`）。
    right: *const Timer,
    /// 节点颜色标志：`true` 表示红色节点，`false` 表示黑色节点（未入树时默认为 `false`）。
    red: bool,
}

impl Links {
    /// 构造尚未挂入红黑树的初始空链接（父子指针均为空，颜色为黑）。
    pub(super) const fn new() -> Self {
        Self {
            parent: null(),
            left: null(),
            right: null(),
            red: false,
        }
    }
}

/// 按绝对截止时间排序并缓存最左（最早到期）节点的侵入式红黑树。
pub(super) struct Queue {
    /// 红黑树的根节点指针；空树时为 `null`。
    root: *const Timer,
    /// 树中截止时间最早（最左侧）的节点指针缓存；空树时为 `null`，用于 $O(1)$ 查询最近超时。
    first: *const Timer,
}

impl Queue {
    /// 构造一棵空的红黑树队列。
    pub(super) const fn new() -> Self {
        Self {
            root: null(),
            first: null(),
        }
    }

    /// 返回树中最早到期节点（`self.first`）的绝对截止时间；若树为空则返回 `None`。
    ///
    /// 调用方必须已持有所属 CPU 的 `TimerBase::state` 自旋锁。
    pub(super) fn first(&self) -> Option<MonotonicInstant> {
        if self.first.is_null() {
            None
        } else {
            // SAFETY:
            // 1. 上方分支已确认 `self.first` 非空；
            // 2. `self.first` 仅由 `insert`/`remove` 维护为树中有效成员，在摘除前由 `Arc` 强引用或 `arm_borrowed` 契约保活；
            // 3. 调用方已持有 `base.state` 自旋锁串行化访问。
            Some(unsafe { expires(self.first) })
        }
    }

    /// 将未入树的定时器节点 `timer` 按其 `expires` 插入红黑树，并在必要时更新最左节点缓存 `self.first`。
    ///
    /// 若遇到相同截止时间的节点，新节点将向右侧子树插入（保持同期限 FIFO 顺序且不抢占 `self.first`）。
    ///
    /// # Safety
    /// 1. 调用方必须已持有本队列所属 CPU 的 `TimerBase::state` 自旋锁；
    /// 2. `timer` 必须非空、内存地址固定且当前未挂在任何红黑树中；
    /// 3. `timer` 及其 `expires` 字段在被 [`Queue::remove`] 或 [`Queue::pop_due`] 摘除前必须保持有效且不可被外部修改。
    pub(super) unsafe fn insert(&mut self, timer: *const Timer) {
        // SAFETY: 调用方已持有 `base.state` 自旋锁并保证 `timer` 与树中所有现有节点指针有效且地址稳定。
        unsafe {
            // Step 1: 从根节点出发向下搜索插入位置，并跟踪新节点是否全程向左走（即是否成为新的最左节点）。
            let expiry = expires(timer);
            let mut parent = null();
            let mut cursor = self.root;
            let mut left = true;
            let mut leftmost = true;
            while !cursor.is_null() {
                parent = cursor;
                left = expiry < expires(cursor);
                if !left {
                    leftmost = false;
                }
                cursor = child(cursor, left);
            }
            // Step 2: 初始化新节点的树链为红色叶节点，挂载到父节点（或作为空树的新根），并更新最左缓存 first。
            *links(timer) = Links {
                parent,
                left: null(),
                right: null(),
                red: true,
            };
            if parent.is_null() {
                self.root = timer;
            } else {
                set_child(parent, left, timer);
            }
            if leftmost {
                self.first = timer;
            }

            // Step 3: 自底向上修复红黑树因插入红节点可能引发的“红父-红子”冲突。
            self.fix_insert(timer);
        }
    }

    /// 将已知在树中的节点 `timer` 从红黑树中摘除，并重置其内部树链为空。
    ///
    /// # Safety
    /// 1. 调用方必须已持有本队列所属 CPU 的 `TimerBase::state` 自旋锁；
    /// 2. `timer` 必须非空且确实是当前红黑树中的有效成员。
    pub(super) unsafe fn remove(&mut self, timer: *const Timer) {
        // SAFETY: 调用方已持有 `base.state` 自旋锁并保证 `timer` 是本树有效成员，树中所有可达父子指针均有效。
        unsafe {
            // Step 1: 若被删除节点恰好是缓存的最左节点 first，将其推进为其紧邻的中序后继节点。
            if self.first == timer {
                self.first = successor(timer);
            }

            // Step 2: 执行二叉搜索树节点摘除与指针移植，记录顶替节点 replacement、其父节点及实际被摘除位置的颜色 removed_red。
            let left = child(timer, true);
            let right = child(timer, false);
            let replacement;
            let replacement_parent;
            let removed_red;
            if left.is_null() || right.is_null() {
                // 单子节点或叶节点：直接用唯一的非空子节点（或空指针）替换 timer。
                replacement = if left.is_null() { right } else { left };
                replacement_parent = (*links(timer)).parent;
                removed_red = is_red(timer);
                self.replace(timer, replacement);
            } else {
                // 双子节点：寻找右子树的最左后继节点 next，仅通过修改树链指针将 next 移植到 timer 的位置并继承 timer 的颜色。
                let next = minimum(right);
                removed_red = is_red(next);
                replacement = child(next, false);
                if (*links(next)).parent == timer {
                    replacement_parent = next;
                    if !replacement.is_null() {
                        (*links(replacement)).parent = next;
                    }
                } else {
                    replacement_parent = (*links(next)).parent;
                    self.replace(next, replacement);
                    set_child(next, false, right);
                    (*links(right)).parent = next;
                }
                self.replace(timer, next);
                set_child(next, true, left);
                (*links(left)).parent = next;
                (*links(next)).red = is_red(timer);
            }

            // Step 3: 若实际被移除颜色的位置原本是黑色，修复该路径缺失的一层黑高；最后清空被摘除节点的树链。
            if !removed_red {
                self.fix_remove(replacement, replacement_parent);
            }
            *links(timer) = Links::new();
        }
    }

    /// 若最左节点 `self.first` 的截止时间不晚于 `now`（`expires <= now`），将其从树中摘除并返回；否则返回 `None`。
    ///
    /// # Safety
    /// 1. 调用方必须已持有本队列所属 CPU 的 `TimerBase::state` 自旋锁；
    /// 2. 树中所有已登记节点均保持有效且地址稳定。
    pub(super) unsafe fn pop_due(&mut self, now: MonotonicInstant) -> Option<*const Timer> {
        let timer = self.first;
        // SAFETY: `self.first` 为空或指向树中最左有效成员；调用方已持有 `base.state` 自旋锁满足 `expires` 与 `remove` 的前置条件。
        unsafe {
            if timer.is_null() || now < expires(timer) {
                None
            } else {
                self.remove(timer);
                Some(timer)
            }
        }
    }

    /// 用子节点（或空指针）`new` 替换 `old` 在其父节点（或树根 `self.root`）中的挂接位置，并更新 `new` 的父指针。
    ///
    /// # Safety
    /// 1. 调用方必须已持有 `base.state` 自旋锁独占访问本树；
    /// 2. `old` 必须是树中的有效成员，`new` 必须为空指针或有效的 `Timer` 节点。
    unsafe fn replace(&mut self, old: *const Timer, new: *const Timer) {
        // SAFETY: 调用方保证 `old` 及其父指针来自有效树链，且 `new` 非空时指向有效可写的节点树链。
        unsafe {
            let parent = (*links(old)).parent;
            if parent.is_null() {
                self.root = new;
            } else {
                set_child(parent, child(parent, true) == old, new);
            }
            if !new.is_null() {
                (*links(new)).parent = parent;
            }
        }
    }

    /// 围绕节点 `old` 执行局部旋转，将其一侧非空子节点提升为新的子树根：
    /// `promote_left == true` 表示提升左子节点（右旋），`promote_left == false` 表示提升右子节点（左旋）。
    ///
    /// # Safety
    /// 1. 调用方必须已持有 `base.state` 自旋锁独占访问本树；
    /// 2. `old` 及其待提升一侧的子节点 `child(old, promote_left)` 均必须是非空的有效树成员。
    unsafe fn rotate(&mut self, old: *const Timer, promote_left: bool) {
        // SAFETY: 调用方保证 `old` 与待提升子节点 `new` 均非空有效，且持有 `base.state` 自旋锁排他修改父子指针。
        unsafe {
            let new = child(old, promote_left);
            let middle = child(new, !promote_left);
            set_child(old, promote_left, middle);
            if !middle.is_null() {
                (*links(middle)).parent = old;
            }
            self.replace(old, new);
            set_child(new, !promote_left, old);
            (*links(old)).parent = new;
        }
    }

    /// 在插入红色叶节点 `node` 后自底向上通过变色与旋转修复“红父-红子”冲突，最后确保树根为黑色。
    ///
    /// # Safety
    /// 1. 调用方必须已持有 `base.state` 自旋锁独占访问本树；
    /// 2. `node` 是刚刚挂入树底部的红色节点，且插入前原树满足全部红黑树不变量。
    unsafe fn fix_insert(&mut self, mut node: *const Timer) {
        // SAFETY: 调用方持有 `base.state` 锁；当 `parent` 为红色时，因树根恒为黑色，其祖父节点 `grandparent` 必然非空有效。
        unsafe {
            let mut parent = (*links(node)).parent;
            while is_red(parent) {
                let grandparent = (*links(parent)).parent;
                let parent_left = child(grandparent, true) == parent;
                let uncle = child(grandparent, !parent_left);
                if is_red(uncle) {
                    set_red(parent, false);
                    set_red(uncle, false);
                    set_red(grandparent, true);
                    node = grandparent;
                    parent = (*links(node)).parent;
                    continue;
                }
                if child(parent, !parent_left) == node {
                    self.rotate(parent, !parent_left);
                    node = parent;
                    parent = (*links(node)).parent;
                }
                set_red(parent, false);
                set_red(grandparent, true);
                self.rotate(grandparent, parent_left);
                break;
            }
            set_red(self.root, false);
        }
    }

    /// 在删除黑色节点导致子树 `node`（父节点为 `parent`）黑高减 1 后，通过兄弟节点借调、变色与旋转恢复全局黑高平衡。
    ///
    /// 由于空叶节点（`null`）不携带结构体，因此显式传入其父节点指针 `parent` 以定位兄弟子树。
    ///
    /// # Safety
    /// 1. 调用方必须已持有 `base.state` 自旋锁独占访问本树；
    /// 2. 除经过 `node` 的路径比同层路径少一层黑高外，其余树结构有效，且 `parent` 是 `node` 的真实父节点。
    unsafe fn fix_remove(&mut self, mut node: *const Timer, mut parent: *const Timer) {
        // SAFETY: 调用方持有 `base.state` 锁；当 `node != self.root` 且为黑色时，由于原树黑高平衡，
        // 缺失黑高的 `node` 一侧必然对应一个非空的兄弟节点 `sibling`（其子树黑高至少为 1）。
        unsafe {
            while node != self.root && !is_red(node) {
                let node_left = child(parent, true) == node;
                let mut sibling = child(parent, !node_left);
                debug_assert!(!sibling.is_null());
                if is_red(sibling) {
                    set_red(sibling, false);
                    set_red(parent, true);
                    self.rotate(parent, !node_left);
                    sibling = child(parent, !node_left);
                }
                let near = child(sibling, node_left);
                let far = child(sibling, !node_left);
                if !is_red(near) && !is_red(far) {
                    set_red(sibling, true);
                    node = parent;
                    parent = (*links(node)).parent;
                    continue;
                }
                if !is_red(far) {
                    set_red(near, false);
                    set_red(sibling, true);
                    self.rotate(sibling, node_left);
                    sibling = child(parent, !node_left);
                }
                set_red(sibling, is_red(parent));
                set_red(parent, false);
                set_red(child(sibling, !node_left), false);
                self.rotate(parent, !node_left);
                node = self.root;
            }
            set_red(node, false);
        }
    }

    /// 遍历整棵红黑树并断言校验：父子指针双向一致性、截止时间单调有序、无红红相邻、各路径黑高相等以及 `self.first` 等于最左节点。
    ///
    /// 返回 `(节点总数, 根节点黑高)`。
    ///
    /// # Safety
    /// 1. 调用方必须已持有本队列所属 CPU 的 `TimerBase::state` 自旋锁；
    /// 2. 树中所有已入队节点均保持有效且地址稳定。
    #[cfg(feature = "ktest")]
    pub(super) unsafe fn validate(&self) -> (usize, usize) {
        // SAFETY: 调用方持有 `base.state` 锁且保证树中节点存活；`validate_subtree` 先核对父指针以防止环形链表死循环。
        unsafe {
            if self.root.is_null() {
                assert!(self.first.is_null());
                return (0, 1);
            }
            assert!(!self.first.is_null());
            assert!(!is_red(self.root));
            let result = validate_subtree(self.root, null(), None, None);
            assert!(self.first == minimum(self.root));
            result
        }
    }
}

/// 获取 `timer` 内部红黑树连接字段 `Node::rb` 的原始可变指针，避免创建覆盖整个 `Timer` 或 `Node` 的 Rust 引用。
///
/// # Safety
/// `timer` 必须非空且指向有效存活的 `Timer`，调用方必须已持有其所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn links(timer: *const Timer) -> *mut Links {
    // SAFETY: 调用方保证 `timer` 非空有效且持有 `base.state` 锁；通过 `UnsafeCell::get()` 与 `addr_of_mut!` 投影字段指针不产生未定义别名。
    unsafe { addr_of_mut!((*(*timer).node.get()).rb) }
}

/// 读取 `timer` 内部记录的绝对截止时间 `Node::expires`。
///
/// # Safety
/// `timer` 必须非空且指向有效存活的 `Timer`，调用方必须已持有其所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn expires(timer: *const Timer) -> MonotonicInstant {
    // SAFETY: 调用方保证 `timer` 非空有效且持有 `base.state` 锁排他保护 `Node`，按值复制 `Copy` 类型的 `expires`。
    unsafe { (*(*timer).node.get()).expires }
}

/// 读取 `timer` 的左子节点（`left == true`）或右子节点（`left == false`）指针。
///
/// # Safety
/// `timer` 必须非空且指向有效存活的 `Timer`，调用方必须已持有其所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn child(timer: *const Timer, left: bool) -> *const Timer {
    // SAFETY: 调用方保证 `timer` 非空有效且持有 `base.state` 锁。
    unsafe {
        if left {
            (*links(timer)).left
        } else {
            (*links(timer)).right
        }
    }
}

/// 设置 `timer` 的左子节点（`left == true`）或右子节点（`left == false`）指针为 `value`（不修改 `value` 的 `parent`）。
///
/// # Safety
/// `timer` 必须非空且指向有效存活的 `Timer`，调用方必须已持有其所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn set_child(timer: *const Timer, left: bool, value: *const Timer) {
    // SAFETY: 调用方保证 `timer` 非空有效且持有 `base.state` 锁独占访问树链字段。
    unsafe {
        if left {
            (*links(timer)).left = value;
        } else {
            (*links(timer)).right = value;
        }
    }
}

/// 判断节点是否为红色；空指针（虚拟黑色外部叶节点 `NIL`）固定返回 `false`。
///
/// # Safety
/// 若 `timer` 非空，必须指向有效存活的 `Timer`，且调用方已持有其所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn is_red(timer: *const Timer) -> bool {
    // SAFETY: `!timer.is_null()` 短路排除了空指针；非空时调用方保证 `timer` 有效且已持有 `base.state` 锁。
    unsafe { !timer.is_null() && (*links(timer)).red }
}

/// 将非空节点 `timer` 的颜色设置为 `red`；若 `timer` 为空指针则忽略。
///
/// # Safety
/// 若 `timer` 非空，必须指向有效存活的 `Timer`，且调用方已持有其所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn set_red(timer: *const Timer, red: bool) {
    if !timer.is_null() {
        // SAFETY: 上方已确认 `timer` 非空，调用方保证其有效且已持有 `base.state` 锁独占写入颜色字段。
        unsafe { (*links(timer)).red = red };
    }
}

/// 沿左子链一路向下查找非空子树 `node` 中的最左（最小/最早到期）节点。
///
/// # Safety
/// `node` 必须非空且所在子树结构有效无环，调用方必须已持有所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn minimum(mut node: *const Timer) -> *const Timer {
    // SAFETY: 调用方保证 `node` 非空且子树所有左链节点有效、已持有 `base.state` 锁。
    unsafe {
        while !child(node, true).is_null() {
            node = child(node, true);
        }
        node
    }
}

/// 查找树中节点 `node` 的中序后继节点；若 `node` 已是全树最右（最大）节点则返回 `null`。
///
/// # Safety
/// `node` 必须非空且是当前红黑树中的有效成员，调用方必须已持有所属 CPU 的 `TimerBase::state` 自旋锁。
unsafe fn successor(mut node: *const Timer) -> *const Timer {
    // SAFETY: 调用方保证 `node` 及树中所有父子链指针有效且已持有 `base.state` 锁。
    unsafe {
        let right = child(node, false);
        if !right.is_null() {
            return minimum(right);
        }
        let mut parent = (*links(node)).parent;
        while !parent.is_null() && child(parent, false) == node {
            node = parent;
            parent = (*links(node)).parent;
        }
        parent
    }
}

/// 递归校验以 `node` 为根的子树是否满足二叉搜索树闭区间 `[lower, upper]` 约束、父子双向指针一致性、
/// 红节点无红子约束以及左右子树黑高相等，返回 `(子树节点数, 子树黑高)`。
///
/// # Safety
/// 从 `node` 可达的所有非空指针必须指向有效存活的 `Timer`，且调用方已持有所属 CPU 的 `TimerBase::state` 自旋锁。
#[cfg(feature = "ktest")]
unsafe fn validate_subtree(
    node: *const Timer,
    parent: *const Timer,
    lower: Option<MonotonicInstant>,
    upper: Option<MonotonicInstant>,
) -> (usize, usize) {
    if node.is_null() {
        return (0, 1);
    }
    // SAFETY: 上方已确认 `node` 非空，调用方持有 `base.state` 锁且保证所有可达节点有效；
    // 首行断言 `parent` 指针一致并拒绝左右子指针相等，排除了有向环导致的无限递归。
    unsafe {
        assert!((*links(node)).parent == parent);
        let expiry = expires(node);
        if let Some(lower) = lower {
            assert!(lower <= expiry);
        }
        if let Some(upper) = upper {
            assert!(expiry <= upper);
        }
        let left = child(node, true);
        let right = child(node, false);
        assert!(left.is_null() || left != right);
        if is_red(node) {
            assert!(!is_red(left) && !is_red(right));
        }
        let (left_count, left_black) = validate_subtree(left, node, lower, Some(expiry));
        let (right_count, right_black) = validate_subtree(right, node, Some(expiry), upper);
        assert!(left_black == right_black);
        (
            left_count + right_count + 1,
            left_black + usize::from(!is_red(node)),
        )
    }
}
