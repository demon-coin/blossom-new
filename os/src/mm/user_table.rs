//! 用户态硬件页表（`UserPageTable`）与页表目录页生命周期管理。
//!
//! # 模块职责与架构定位
//! 本模块位于虚拟内存管理器（`Mm`）与体系结构页表原语（`crate::arch`）之间，负责管理单个用户地址空间的硬件页表树：
//! - **上游调用者**：`Mm`（处理缺页、建立/解除 VMA 映射、写时复制与脏页回写保护）。
//! - **下游依赖**：伙伴系统页分配器（`alloc_pages` / `ReclaimAttempt`）与体系结构页表项编解码接口（`arch::user_*`）。
//!
//! # 核心设计与所有权模型
//! 1. **表页所有权与数据页解耦**：
//!    - `UserPageTable` 仅拥有用户根页表页（`root`）及动态分配的用户态中间层目录页（保存在 `tables` 链表中）。
//!    - 根页表中的内核高半区共享槽位或常驻空目录仅借用全局静态页表，不纳入 `tables` 所有权链表。
//!    - 叶子页表项（Level 0）映射的用户物理数据页（Page / Folio）由上层 `Mm` 管理生命周期；页表查询或解映射仅返回非拥有的物理地址与权限快照（[`Mapping`]）。
//! 2. **三级/多级页表虚拟地址划分与目录树结构**：
//!    ```text
//!    虚拟地址 (VA):
//!    +-------------------+-------------------+-------------------+----------------+
//!    |  Level 2 Index    |  Level 1 Index    |  Level 0 Index    |  Page Offset   |
//!    |   (INDEX_BITS)    |   (INDEX_BITS)    |   (INDEX_BITS)    |  (PAGE_SHIFT)  |
//!    +---------+---------+---------+---------+---------+---------+----------------+
//!              |                   |                   |
//!              v                   v                   v
//!        [ Root Table ] ---> [ Mid Table ] ---> [ Leaf Table ] ---> 用户物理数据页
//!         (root: L2)          (tables: L1)       (tables: L0)       (由 Mm 持有所有权)
//!    ```
//! 3. **锁外预分配与延迟释放（Lock-Outside Allocation & TLB Shootdown）**：
//!    - `map` 所需的中间目录页必须在获取 PTE 自旋锁前通过 [`PreparedTables`] 预先分配并初始化为空表；持锁期间仅移动 `Box<TableNode>` 指针并原子写入页表项，避免在自旋锁临界区内执行内存分配或堆释放。
//!    - `unmap` 摘除的空中间目录页不会在持锁期间直接释放，而是装入 [`RetiredTables`] 返回给调用方；调用方释放 PTE 自旋锁并完成跨 CPU 同步 TLB 刷新（TLB Shootdown）后，才调用 [`RetiredTables::release`] 归还物理页，防止硬件页表遍历器（Page Table Walker）读取已重用的物理页导致 UAF。
//!
//! # 锁获取顺序与并发契约
//! - 对同一 `UserPageTable` 的软件修改操作（`map`、`unmap`、`clear_leaf`、`mkclean`、`protect_leaf`）必须由上层 `Mm` 的 PTE 自旋锁串行化保护。
//! - 页表槽位读写全部通过 `AtomicUsize`（Acquire/Release/AcqRel）执行，确保软件写入的新目录页初始化内容对硬件页表遍历器可见，且不会撕裂硬件并发更新的访问位（Accessed）/脏位（Dirty）。

use crate::arch;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::{alloc_pages, OwnedPages, PAGE_SHIFT, PAGE_SIZE};
use alloc::boxed::Box;
use core::mem::size_of;
use core::sync::atomic::{AtomicUsize, Ordering};

/// 单个 4KiB 硬件页表页包含的页表项（PTE）槽位数量（64 位下为 512 项）。
const ENTRIES: usize = PAGE_SIZE / size_of::<usize>();
/// 每一级页表索引占用的虚拟地址比特数（512 项对应 9 位）。
const INDEX_BITS: usize = ENTRIES.trailing_zeros() as usize;

/// 用户态叶子页表项的访问权限集合。
///
/// 具体哪些权限组合可由硬件页表表示（例如是否支持只写或只执行）由体系结构后端决定。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PagePermissions {
    /// 是否允许用户态读取该页。
    pub read: bool,
    /// 是否允许用户态写入该页。
    pub write: bool,
    /// 是否允许用户态取指执行该页。
    pub execute: bool,
}

/// 用户态叶子页表项的非拥有型映射快照。
///
/// 仅记录当前槽位解码出的物理页基址与权限，不持有底层物理页的所有权，
/// 调用方不能仅凭 `physical` 整数直接释放物理页。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mapping {
    /// 叶子页表项指向的页对齐物理字节地址（PA）。
    pub physical: usize,
    /// 叶子页表项当前生效的用户态访问权限。
    pub permissions: PagePermissions,
}

/// 页表目录预分配或硬件映射操作失败的原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PageTableError {
    /// 物理页分配器或内核堆内存不足。
    NoMemory,
    /// 虚拟地址未页对齐、超出用户地址空间上限，或物理地址/权限组合非法。
    InvalidRange,
    /// 中间层页表槽位已被非目录项（如大页映射）占用，无法继续向下挂载子表。
    Occupied,
    /// 预分配的 [`PreparedTables`] 中缺少建立映射所需的中间层目录页。
    MissingTables,
}

/// 单个私有中间层页表页的所有权链表节点。
///
/// 堆节点外壳（`Box<TableNode>`）与底层 order-0 物理页均在 PTE 自旋锁外分配并初始化，
/// 持锁临界区内只做链表指针转移，不触发内存分配或释放。
struct TableNode {
    /// 该中间层目录页对应的 order-0 物理页所有权。
    pages: OwnedPages,
    /// 指向链表中下一个私有目录页节点的拥有指针。
    next: Option<Box<TableNode>>,
}

impl TableNode {
    /// 在自旋锁外分配并初始化指定层级（`level`）的页表目录页。
    ///
    /// 若堆外壳分配失败，已申请的 `OwnedPages` 会在函数返回时自动释放。
    fn try_new(level: usize) -> Result<Box<Self>, PageTableError> {
        let pages = alloc_pages(0, false).ok_or(PageTableError::NoMemory)?;
        let mut node =
            Box::try_new(Self { pages, next: None }).map_err(|_| PageTableError::NoMemory)?;
        node.initialize(level);
        Ok(node)
    }

    /// 在缺页异常路径中，复用带页面回收能力的 `ReclaimAttempt` 分配并初始化指定层级的目录页。
    ///
    /// 底层页表页分配可触发内存回收，而 `Box` 节点外壳若遇堆耗尽则直接返回 `NoMemory`。
    fn try_new_reclaim(
        level: usize,
        attempt: &mut ReclaimAttempt,
    ) -> Result<Box<Self>, PageTableError> {
        let pages = attempt
            .allocate_pages(0, false)
            .ok_or(PageTableError::NoMemory)?;
        let mut node =
            Box::try_new(Self { pages, next: None }).map_err(|_| PageTableError::NoMemory)?;
        node.initialize(level);
        Ok(node)
    }

    /// 将新分配且尚未挂载到硬件页表树的目录页全部填充为对应层级的空表项。
    fn initialize(&mut self, level: usize) {
        let pointer = self.pages.as_mut_ptr().cast::<usize>();
        let empty = arch::user_empty_entry(level);
        for index in 0..ENTRIES {
            // SAFETY:
            // 1. 当前节点刚刚分配且尚未发布到任何硬件页表槽位，由 `self.pages` 独占持有；
            // 2. `self.pages` 是完整的 order-0（4KiB）物理页，`index` 在 `0..ENTRIES` 范围内不会越界；
            // 3. 在将目录页挂载到父页表前完成全部槽位初始化，可防止硬件页表遍历器读到未初始化垃圾值。
            unsafe { pointer.add(index).write(empty) };
        }
    }
}

/// 私有中间层页表页的单向所有权链表。
///
/// 支持在 PTE 自旋锁临界区内以 $O(1)$ 开销压入/弹出节点，不触发任何堆分配或析构。
#[derive(Default)]
struct TableOwners {
    /// 链表头节点。
    head: Option<Box<TableNode>>,
}

impl TableOwners {
    /// 将已初始化的目录页节点压入链表头部（纯指针操作，不分配或释放内存）。
    fn push(&mut self, mut node: Box<TableNode>) {
        assert!(node.next.is_none(), "table node is already linked");
        node.next = self.head.take();
        self.head = Some(node);
    }

    /// 弹出链表头部的目录页节点并断开其后继链接，供调用方转移所有权或释放。
    fn pop(&mut self) -> Option<Box<TableNode>> {
        let mut node = self.head.take()?;
        self.head = node.next.take();
        Some(node)
    }

    /// 根据目录页的物理基址从所有权链表中摘除并返回对应节点。
    ///
    /// 调用方必须先将该目录页从硬件父页表槽位中解绑。若链表中不存在该物理地址则直接 panic。
    fn remove(&mut self, physical: usize) -> Box<TableNode> {
        // ponytail: 归还时线性查找私有表页；表页规模使查找成为瓶颈时再增加索引。
        let mut link = &mut self.head;
        while link.is_some() {
            if link.as_ref().unwrap().pages.phys_addr() == physical {
                let mut node = link.take().unwrap();
                *link = node.next.take();
                return node;
            }
            link = &mut link.as_mut().unwrap().next;
        }
        panic!("detached user table has no owner");
    }
}

impl Drop for TableOwners {
    /// 在自旋锁外循环弹出并释放全部目录页节点，避免长链表默认递归析构导致内核栈溢出。
    fn drop(&mut self) {
        while let Some(node) = self.pop() {
            drop(node);
        }
    }
}

/// 建立映射前在 PTE 自旋锁外预分配的中间层目录页集合。
///
/// 若持锁后发现部分层级已由其他并发路径建立，未被消费的目录页节点会保留在本结构中，
/// 待调用方释放 PTE 自旋锁后随本结构析构安全归还给页分配器。
#[derive(Default)]
pub struct PreparedTables {
    /// 按页表层级下标（`0 .. USER_LEVELS - 1`）存放的预分配目录页节点。
    tables: [Option<Box<TableNode>>; arch::USER_LEVELS - 1],
}

impl PreparedTables {
    /// 在 PTE 自旋锁外从第 0 层（叶子表）向上连续预分配 `count` 个中间层目录页。
    ///
    /// `count` 通常由锁外调用 [`UserPageTable::needed_tables`] 探测得出，且不能超过 `USER_LEVELS - 1`。
    pub fn try_new(count: usize) -> Result<Self, PageTableError> {
        if arch::USER_LEVELS - 1 < count {
            return Err(PageTableError::InvalidRange);
        }

        let mut prepared = Self::default();
        for level in 0..count {
            prepared.tables[level] = Some(TableNode::try_new(level)?);
        }
        Ok(prepared)
    }

    /// 在缺页异常路径中，使用 `ReclaimAttempt` 在自旋锁外预分配 `count` 个中间层目录页。
    pub(crate) fn try_new_reclaim(
        count: usize,
        attempt: &mut ReclaimAttempt,
    ) -> Result<Self, PageTableError> {
        if arch::USER_LEVELS - 1 < count {
            return Err(PageTableError::InvalidRange);
        }

        let mut prepared = Self::default();
        for level in 0..count {
            prepared.tables[level] = Some(TableNode::try_new_reclaim(level, attempt)?);
        }
        Ok(prepared)
    }
}

/// 已从硬件页表树摘除、等待跨 CPU TLB 同步刷新后释放的中间层目录页集合。
///
/// 在 `unmap` 持锁期间，变为空表的中间目录页会被摘入此容器；
/// 调用方释放 PTE 自旋锁并执行完覆盖所有活动 CPU 的 TLB Shootdown 后，再析构或调用 `release` 归还物理页。
#[derive(Default)]
pub struct RetiredTables {
    /// 待释放的私有目录页所有权链表。
    owners: TableOwners,
}

impl RetiredTables {
    /// 将另一批已摘除的目录页合并到当前待释放链表中（仅移动链表节点，可在自旋锁临界区内调用）。
    pub(crate) fn append(&mut self, mut other: Self) {
        while let Some(node) = other.owners.pop() {
            self.owners.push(node);
        }
    }

    /// 逐个释放全部已摘除的目录页，并返回本次归还给页分配器的物理页数量。
    ///
    /// 调用方必须确保所有活动 CPU 已完成同步 TLB 刷新且不再持有 PTE 自旋锁。
    pub(crate) fn release(mut self) -> usize {
        let mut freed = 0;
        while let Some(node) = self.owners.pop() {
            drop(node);
            freed += 1;
        }
        freed
    }

    /// 统计当前待释放链表中持有的目录页数量（仅供内核测试使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn len(&self) -> usize {
        let mut count = 0;
        let mut cursor = self.owners.head.as_deref();
        while let Some(node) = cursor {
            count += 1;
            cursor = node.next.as_deref();
        }
        count
    }
}

/// 用户地址空间的硬件页表树。
///
/// 负责管理用户根页表页（`root`）与所有动态分配的用户态中间层目录页（`tables`）。
/// 共享的内核高半区页表及常驻空表由体系结构后端静态维护，不纳入本结构体的所有权链表。
/// 析构前调用方必须确保没有任何活动 CPU 正在使用该页表根，且已完成相应的 TLB 刷新。
pub struct UserPageTable {
    /// 用户根页表页（最高层级目录页）的独占物理页所有权。
    root: OwnedPages,
    /// 挂载在根页表下的所有私有中间层及叶子层目录页所有权链表。
    tables: TableOwners,
}

impl UserPageTable {
    /// 在自旋锁外分配并初始化一个新的用户根页表。
    ///
    /// 用户空间槽位初始化为空项，内核空间共享槽位由体系结构后端借用全局永久内核页表填充。
    pub fn try_new() -> Result<Self, PageTableError> {
        let mut root = alloc_pages(0, false).ok_or(PageTableError::NoMemory)?;
        // SAFETY:
        // 1. `root` 是刚刚分配、尚未对任何 CPU 发布且具备永久直接映射的独占 order-0 物理页；
        // 2. `arch::initialize_user_root` 会初始化整页的全部槽位，内核槽位仅借用全局静态页表物理地址。
        unsafe { arch::initialize_user_root(root.as_mut_ptr().cast::<usize>()) };
        Ok(Self {
            root,
            tables: TableOwners::default(),
        })
    }

    /// 返回用户根页表的物理字节地址（用于装入硬件页表基址寄存器如 `satp` / `pgdl`）。
    pub fn root_phys(&self) -> usize {
        self.root.phys_addr()
    }

    /// 查询指定用户虚拟地址 `va` 所在的叶子页映射快照。
    ///
    /// 只读遍历硬件页表树，不分配内存、不阻塞，也不转移目标物理数据页的所有权。
    pub fn lookup(&self, va: usize) -> Option<Mapping> {
        if arch::USER_ADDRESS_END <= va {
            return None;
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY:
            // 1. `va < USER_ADDRESS_END` 保证只访问用户空间槽位；
            // 2. `table` 来自 `self.root` 或已由 `self.tables` 保活的私有中间目录页，且调用方持有 PTE 锁防止并发拆表。
            let entry = unsafe { read_slot(table, slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return None;
            }
            table = arch::user_entry_physical(entry, level);
        }
        // SAFETY:
        // 上述循环逐级验证了目录项有效性，到达的 `table` 是由 `self.tables` 保活并完整初始化的叶子页表页，
        // 且 `slot_index(va, 0)` 位于 `0..ENTRIES` 范围内。
        mapping(unsafe { read_slot(table, slot_index(va, 0)) })
    }

    /// 在 PTE 自旋锁保护下检查虚拟地址 `va` 是否仍映射到物理页 `physical`，并原子清除其可写位（Write）与硬件脏位（Dirty）。
    ///
    /// 使用 CAS 循环更新叶子页表项，避免丢失硬件并发设置的访问位（Accessed）或脏位。
    /// 若映射存在且物理地址匹配，返回 `Some((was_dirty, pte_changed))`；调用方需在释放锁后对所有活动 CPU 同步刷新 TLB。
    ///
    /// # Safety
    /// 1. 调用方必须持有外层 `Mm` 的 PTE 自旋锁以串行化软件页表修改；
    /// 2. `physical` 对应的物理数据页必须保持存活，直到跨 CPU 同步 TLB 刷新完成。
    pub(crate) unsafe fn mkclean(&mut self, va: usize, physical: usize) -> Option<(bool, bool)> {
        let leaf = self.leaf_table(va)?;
        let pointer = arch::phys_to_virt(leaf) as *mut AtomicUsize;
        // SAFETY:
        // 1. `leaf_table(va)` 已验证 `leaf` 是本页表拥有且完整初始化的叶子目录页；
        // 2. `slot_index(va, 0) < ENTRIES`，且所有软件访问均通过 `AtomicUsize` 配合外层 PTE 锁进行。
        let slot = unsafe { &*pointer.add(slot_index(va, 0)) };
        let mut old = slot.load(Ordering::Acquire);
        loop {
            let current = mapping(old)?;
            if current.physical != physical {
                return None;
            }
            let clean = arch::user_leaf_mkclean(old);
            match slot.compare_exchange_weak(old, clean, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => return Some((arch::user_leaf_dirty(old), clean != old)),
                Err(actual) => old = actual,
            }
        }
    }

    /// 在不更换底层物理页（PFN 保持为 `physical`）的前提下，修改虚拟地址 `va` 处叶子页表项的访问权限。
    ///
    /// 若目标虚拟地址未映射或当前映射的物理页不是 `physical`，则返回 `Ok(false)`。
    ///
    /// # Safety
    /// 1. 调用方必须持有外层 `Mm` 的 PTE 自旋锁以串行化软件页表修改；
    /// 2. `physical` 对应的物理数据页必须保持存活，直到锁外完成跨 CPU 同步 TLB 刷新。
    pub(crate) unsafe fn protect_leaf(
        &mut self,
        va: usize,
        physical: usize,
        permissions: PagePermissions,
    ) -> Result<bool, PageTableError> {
        let next = checked_leaf(physical, permissions)?;
        let Some(leaf) = self.leaf_table(va) else {
            return Ok(false);
        };
        // SAFETY:
        // `leaf_table(va)` 已验证 `leaf` 是本页表拥有且完整初始化的叶子目录页，调用方持有 PTE 锁串行化修改。
        let old = unsafe { read_slot(leaf, slot_index(va, 0)) };
        if mapping(old).is_none_or(|current| current.physical != physical) {
            return Ok(false);
        }
        // SAFETY:
        // 仅在相同物理页上修改访问权限，不释放底层物理页；旧权限对应的 TLB 项由调用方在锁外同步刷新。
        unsafe { replace_slot(leaf, slot_index(va, 0), next) };
        Ok(true)
    }

    /// 沿页表树向下查找虚拟地址 `va` 所在的叶子目录页（Level 0）物理基址。
    ///
    /// 若 `va` 非合法页对齐用户地址，或沿途某级目录尚未分配，则返回 `None`。
    fn leaf_table(&self, va: usize) -> Option<usize> {
        if !valid_page(va) {
            return None;
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY:
            // `self.root` 与沿途非空目录页均由本页表对象保活，且调用方持有 PTE 自旋锁防止并发拆表。
            let entry = unsafe { read_slot(table, slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return None;
            }
            table = arch::user_entry_physical(entry, level);
        }
        Some(table)
    }

    /// 仅清空虚拟地址 `va` 对应的叶子页表项，保留沿途所有中间层目录页。
    ///
    /// 常用于写时复制（COW）或页面迁移时的 Break-Before-Make 流程：先清除旧映射并同步刷新 TLB，再安装新映射。
    ///
    /// # Safety
    /// 1. 调用方必须持有外层 `Mm` 的 PTE 自旋锁以串行化软件页表修改；
    /// 2. 旧叶子项指向的物理数据页必须保持存活，直到所有活动 CPU 完成同步 TLB 刷新；
    /// 3. 在完成跨 CPU TLB 刷新前，不得向该槽位写入指向新物理页或更高权限的页表项。
    pub unsafe fn clear_leaf(&mut self, va: usize) -> Option<Mapping> {
        if !valid_page(va) {
            return None;
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY:
            // `self.root` 及沿途非空目录页均由本页表对象保活，调用方持有 PTE 锁串行化修改，索引位于页内。
            let entry = unsafe { read_slot(table, slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return None;
            }
            table = arch::user_entry_physical(entry, level);
        }
        // SAFETY:
        // `table` 是由本页表对象保活的叶子目录页，调用方负责将旧物理数据页保活至后续跨 CPU TLB 刷新完成；
        // 此处仅原子写入空叶子项，不摘除任何中间层目录页。
        mapping(unsafe { replace_slot(table, slot_index(va, 0), arch::user_empty_entry(0)) })
    }

    /// 探测映射用户虚拟地址 `va` 还需要新分配多少层中间目录页。
    ///
    /// 若 `va` 非法或路径被非目录项占用则返回 0。锁外探测结果仅用于指导 [`PreparedTables::try_new`]，
    /// 真正调用 [`Self::map`] 时会在 PTE 锁内通过 `probe_needed` 重新校验。
    pub fn needed_tables(&self, va: usize) -> usize {
        self.probe_needed(va).unwrap_or(0)
    }

    /// 在虚拟地址 `va` 处安装或替换指向 `physical` 的用户叶子页表项，并返回被替换的旧映射快照。
    ///
    /// 整个过程在 PTE 自旋锁临界区内完成，仅从 `prepared` 中移取所需的预分配目录页，不在锁内分配或释放内存。
    /// 若参数校验失败或 `prepared` 中的目录页数量不足，不会修改任何硬件页表槽位，也不会消耗 `prepared` 中的节点。
    ///
    /// # Safety
    /// 1. 调用方必须持有外层 `Mm` 的 PTE 自旋锁以串行化软件页表修改；
    /// 2. `physical` 必须是由 `Mm` 保活且已完成内容初始化的用户数据页，不得是内核页表页、内核私有对象或存在可变 Rust 引用的内存；
    /// 3. 新映射的数据页必须保持存活直到解除映射并完成跨 CPU TLB 刷新；若替换了旧映射，旧数据页同样必须保持存活直到完成跨 CPU TLB 刷新；
    /// 4. 调用方必须在释放 PTE 自旋锁后执行必要的 TLB / 指令缓存同步，且 `prepared` 只能在释放 PTE 自旋锁后析构。
    pub unsafe fn map(
        &mut self,
        va: usize,
        physical: usize,
        permissions: PagePermissions,
        prepared: &mut PreparedTables,
    ) -> Result<Option<Mapping>, PageTableError> {
        // Step 1: 校验物理地址、权限编码以及路径上缺失的目录层级，确保预分配的目录页充足。
        let leaf = checked_leaf(physical, permissions)?;
        let needed = self.probe_needed(va)?;
        if prepared.tables[..needed].iter().any(Option::is_none) {
            return Err(PageTableError::MissingTables);
        }

        // Step 2: 从根页表自顶向下遍历，若遇到空目录槽位则挂载 `prepared` 中对应层级的目录页。
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            let index = slot_index(va, level);
            // SAFETY:
            // 前置 `probe_needed` 已确认路径上仅包含空槽位或本对象拥有的目录页，
            // 且外层 PTE 锁阻止了并发软件修改，`table` 指向的表页处于存活状态。
            let mut entry = unsafe { read_slot(table, index) };
            if entry == arch::user_empty_entry(level) {
                let node = prepared.tables[level - 1]
                    .take()
                    .expect("prepared table level changed");
                let physical = node.pages.phys_addr();
                entry = arch::user_table_entry(physical);
                self.tables.push(node);
                // SAFETY:
                // 新目录页已在锁外完整初始化为空项，且所有权已转入 `self.tables`；
                // `replace_slot` 使用 AcqRel 原子操作确保目录页初始化内容对硬件页表遍历器可见。
                unsafe { replace_slot(table, index, entry) };
            }
            table = arch::user_entry_physical(entry, level);
        }

        // Step 3: 在叶子目录页（Level 0）原子写入新的叶子页表项，并返回旧槽位解码快照。
        // SAFETY:
        // `table` 是已由 `self.tables` 保活并完整初始化的叶子目录页，调用方负责新旧数据页保活与锁外 TLB 刷新。
        let old = unsafe { replace_slot(table, slot_index(va, 0), leaf) };
        Ok(mapping(old))
    }

    /// 解除虚拟地址 `va` 的叶子映射，并自底向上摘除沿途变为空表的私有中间层目录页。
    ///
    /// 即使叶子槽位先前已被 [`Self::clear_leaf`] 清空，本函数仍会检查并回收沿途已变空的私有目录页。
    /// 摘除的目录页保存在返回的 [`RetiredTables`] 中，不在持锁期间释放。
    ///
    /// # Safety
    /// 1. 调用方必须持有外层 `Mm` 的 PTE 自旋锁以串行化软件页表修改；
    /// 2. 返回的旧数据页（若存在）及 [`RetiredTables`] 中的目录页不得在 PTE 自旋锁临界区内释放；
    /// 3. 调用方必须在锁外完成覆盖所有活动 CPU 的同步 TLB 刷新（同时终止硬件对已摘除目录页的遍历）后，才能释放旧数据页和 `RetiredTables`。
    pub unsafe fn unmap(&mut self, va: usize) -> (Option<Mapping>, RetiredTables) {
        let mut retired = RetiredTables::default();
        if !valid_page(va) {
            return (None, retired);
        }
        // Step 1: 自顶向下记录每一级目录页的物理基址；若中途某级目录不存在则直接返回。
        let mut path = [0usize; arch::USER_LEVELS];
        path[arch::USER_LEVELS - 1] = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY:
            // `path[level]` 来自 `self.root` 或上一轮已验证存活的私有目录页，外层 PTE 锁串行化修改，索引位于页内。
            let entry = unsafe { read_slot(path[level], slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return (None, retired);
            }
            path[level - 1] = arch::user_entry_physical(entry, level);
        }

        // Step 2: 原子清空叶子槽位并解码旧映射快照。
        // SAFETY:
        // `path[0]` 是由本页表对象保活的叶子目录页，调用方负责将旧物理数据页保活至后续跨 CPU TLB 刷新完成。
        let old = unsafe { replace_slot(path[0], slot_index(va, 0), arch::user_empty_entry(0)) };
        let old = mapping(old);

        // Step 3: 自底向上检查私有中间目录页是否已全部变空；若变空则从父目录解绑并移入 `retired`。
        for level in 0..arch::USER_LEVELS - 1 {
            // SAFETY:
            // `path[level]` 尚未被摘除且由 `self.tables` 保活，外层 PTE 锁阻止了并发新增表项。
            if !unsafe { table_is_empty(path[level], level) } {
                break;
            }
            // SAFETY:
            // `path[level]` 的全部槽位已确认置空，父目录 `path[level + 1]` 仍指向该私有子表；
            // 先清空父目录槽位再将子表所有权转移至 `retired`，由调用方在跨 CPU TLB 刷新后统一释放。
            unsafe {
                replace_slot(
                    path[level + 1],
                    slot_index(va, level + 1),
                    arch::user_empty_entry(level + 1),
                )
            };
            retired.owners.push(self.tables.remove(path[level]));
        }
        (old, retired)
    }

    /// 只读探测建立虚拟地址 `va` 的叶子映射需要新建多少层中间目录页。
    ///
    /// 在修改任何硬件页表槽位前完成地址合法性与路径冲突检查。
    fn probe_needed(&self, va: usize) -> Result<usize, PageTableError> {
        if !valid_page(va) {
            return Err(PageTableError::InvalidRange);
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY:
            // `valid_page(va)` 限制只访问用户空间槽位，沿途非空目录页均由本对象保活，
            // 且 `read_slot` 使用原子加载，索引位于页内。
            let entry = unsafe { read_slot(table, slot_index(va, level)) };
            if entry == arch::user_empty_entry(level) {
                return Ok(level);
            }
            if !arch::user_entry_is_table(entry, level) {
                return Err(PageTableError::Occupied);
            }
            table = arch::user_entry_physical(entry, level);
        }
        Ok(0)
    }
}

/// 检查虚拟地址 `va` 是否为页对齐且位于合法用户地址空间（`0 .. USER_ADDRESS_END`）内。
fn valid_page(va: usize) -> bool {
    va < arch::USER_ADDRESS_END && va & (PAGE_SIZE - 1) == 0
}

/// 提取虚拟地址 `va` 在第 `level` 级页表中的槽位下标（`0..ENTRIES`）。
fn slot_index(va: usize, level: usize) -> usize {
    (va >> (PAGE_SHIFT + INDEX_BITS * level)) & (ENTRIES - 1)
}

/// 构造并校验叶子页表项，确保物理地址页对齐且编码后可无损还原物理地址与访问权限。
fn checked_leaf(physical: usize, permissions: PagePermissions) -> Result<usize, PageTableError> {
    if physical & (PAGE_SIZE - 1) != 0 {
        return Err(PageTableError::InvalidRange);
    }
    let entry = arch::user_leaf_entry(physical, permissions);
    if entry == arch::user_empty_entry(0)
        || arch::user_entry_physical(entry, 0) != physical
        || arch::user_leaf_permissions(entry) != Some(permissions)
    {
        return Err(PageTableError::InvalidRange);
    }
    Ok(entry)
}

/// 将硬件叶子页表项解码为非拥有型 [`Mapping`] 快照；若为空项或非法权限项则返回 `None`。
fn mapping(entry: usize) -> Option<Mapping> {
    if entry == arch::user_empty_entry(0) {
        None
    } else {
        Some(Mapping {
            physical: arch::user_entry_physical(entry, 0),
            permissions: arch::user_leaf_permissions(entry)?,
        })
    }
}

/// 使用 Acquire 原子语义读取指定页表页 `physical` 中下标为 `index` 的硬件页表项。
///
/// # Safety
/// 1. `physical` 必须是当前 `UserPageTable` 存活、已完整初始化且具有永久直接映射的页表页物理基址；
/// 2. `index` 必须严格小于 `ENTRIES`；
/// 3. 并发软件修改必须遵守外层 PTE 自旋锁协议。
unsafe fn read_slot(physical: usize, index: usize) -> usize {
    let pointer = arch::phys_to_virt(physical) as *mut AtomicUsize;
    // SAFETY:
    // 1. 调用方保证 `physical` 是存活且已初始化的页表页，页对齐满足 `AtomicUsize` 对齐要求；
    // 2. `index < ENTRIES` 保证指针偏移落在同一 4KiB 页内，原子读取不会与硬件更新产生数据竞争。
    unsafe { (*pointer.add(index)).load(Ordering::Acquire) }
}

/// 使用 AcqRel 原子交换更新指定页表页 `physical` 中下标为 `index` 的硬件页表项，并返回旧值。
///
/// Release 语义保证新子表或新数据页的初始化写入在页表项对硬件可见之前完成。
///
/// # Safety
/// 1. `physical` 与 `index` 必须满足 [`read_slot`] 的全部安全前提；
/// 2. `entry` 必须是当前层级合法的体系结构页表项编码；
/// 3. 调用方必须持有外层 PTE 自旋锁，并将被替换的旧目标页保活至跨 CPU TLB 刷新完成、将新目标页保活至后续解映射并完成 TLB 刷新。
unsafe fn replace_slot(physical: usize, index: usize, entry: usize) -> usize {
    let pointer = arch::phys_to_virt(physical) as *mut AtomicUsize;
    // SAFETY:
    // 1. 调用方保证表页存活、索引在 `0..ENTRIES` 内且软件修改已由 PTE 自旋锁串行化；
    // 2. `AtomicUsize::swap` 以原子方式更新硬件槽位，AcqRel 保证先前的数据/子表初始化不会重排到页表项发布之后。
    unsafe { (*pointer.add(index)).swap(entry, Ordering::AcqRel) }
}

/// 检查第 `level` 层私有目录页 `physical` 的全部 `ENTRIES` 个槽位是否均为空项。
///
/// # Safety
/// 1. `physical` 必须是当前 `UserPageTable` 拥有且尚未摘除的私有目录页，`level` 必须与其真实层级一致；
/// 2. 调用方必须持有外层 PTE 自旋锁并在查询期间保持该目录页存活。
unsafe fn table_is_empty(physical: usize, level: usize) -> bool {
    let empty = arch::user_empty_entry(level);
    for index in 0..ENTRIES {
        // SAFETY:
        // 调用方保证 `physical` 指向存活的私有目录页且 `level` 正确，循环变量 `index` 始终在 `0..ENTRIES` 范围内。
        if unsafe { read_slot(physical, index) } != empty {
            return false;
        }
    }
    true
}
