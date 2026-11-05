//! 用户硬件页表只拥有根和私有中间页，数据页的存活与失效由 Mm 负责。
//! 软件修改由 Mm 的 PTE 短锁串行化；硬件槽始终使用原子访问。

use crate::arch;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::{alloc_pages, OwnedPages, PAGE_SHIFT, PAGE_SIZE};
use alloc::boxed::Box;
use core::mem::size_of;
use core::sync::atomic::{AtomicUsize, Ordering};

const ENTRIES: usize = PAGE_SIZE / size_of::<usize>();
const INDEX_BITS: usize = ENTRIES.trailing_zeros() as usize;

/// 用户叶的访问权限；架构后端决定可表示的权限组合。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PagePermissions {
    pub read: bool,
    pub write: bool,
    pub execute: bool,
}

/// 一个非拥有的页基址映射快照，不能凭物理整数恢复页的归还权。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mapping {
    pub physical: usize,
    pub permissions: PagePermissions,
}

/// 页表准备或硬件映射不能完成的原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PageTableError {
    NoMemory,
    InvalidRange,
    Occupied,
    MissingTables,
}

/// 节点外壳和实际 order-0 页在锁外一次准备，锁内只移动拥有权。
struct TableNode {
    pages: OwnedPages,
    next: Option<Box<TableNode>>,
}

impl TableNode {
    /// 在锁外申请并初始化指定层的表页；失败时正常归还此前取得的拥有。
    fn try_new(level: usize) -> Result<Box<Self>, PageTableError> {
        let pages = alloc_pages(0, false).ok_or(PageTableError::NoMemory)?;
        let mut node =
            Box::try_new(Self { pages, next: None }).map_err(|_| PageTableError::NoMemory)?;
        node.initialize(level);
        Ok(node)
    }

    /// 明确 fault 中间表物理页共用目标页的回收请求，小节点仍真实返回 NoMemory。
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

    /// 将尚未发表的完整页初始化为指定层的空槽，随后才允许链接到硬件目录。
    fn initialize(&mut self, level: usize) {
        let pointer = self.pages.as_mut_ptr().cast::<usize>();
        let empty = arch::user_empty_entry(level);
        for index in 0..ENTRIES {
            // SAFETY: 此节点尚未发表，order-0 页由当前节点独占，索引落在完整页内；
            // 每个机器字都在任何原子读取或硬件访问前写入架构有效的空值。
            unsafe { pointer.add(index).write(empty) };
        }
    }
}

/// 具体拥有链不会在短锁内扩容或析构节点。
#[derive(Default)]
struct TableOwners {
    head: Option<Box<TableNode>>,
}

impl TableOwners {
    /// 把已准备节点移入本链，不分配且不释放任何页或外壳。
    fn push(&mut self, mut node: Box<TableNode>) {
        assert!(node.next.is_none(), "table node is already linked");
        node.next = self.head.take();
        self.head = Some(node);
    }

    /// 摘取链首并断开后继拥有，使接收者可在锁内重新归属该节点。
    fn pop(&mut self) -> Option<Box<TableNode>> {
        let mut node = self.head.take()?;
        self.head = node.next.take();
        Some(node)
    }

    /// 摘取已经从硬件父槽解除的真实拥有，不从物理整数构造新的归还凭证。
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
    /// 在锁外逐个归还拥有，避免大地址空间的链式递归析构消耗内核栈。
    fn drop(&mut self) {
        while let Some(node) = self.pop() {
            drop(node);
        }
    }
}

/// 仅在 PTE 短锁外创建和析构；未消费节点保留到调用者放锁后归还。
#[derive(Default)]
pub struct PreparedTables {
    tables: [Option<Box<TableNode>>; arch::USER_LEVELS - 1],
}

impl PreparedTables {
    /// 锁外从叶层起连续准备 count 层的表页及外壳，count 不能超过私有中间层数。
    /// 缺失目录之后的整条链总是包含 0..count 层；重查减少缺级时保留多余节点。
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

    /// fault 在原短锁外按同一请求准备目录，不改变通用 try_new 的 NOWAIT 行为。
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

/// 已摘离的私有表页；完成覆盖全部活动 CPU 的同步 TLB 失效后才能在锁外析构。
/// 持有此对象不会继续占用原页表的 PTE 锁。
#[derive(Default)]
pub struct RetiredTables {
    owners: TableOwners,
}

impl RetiredTables {
    /// 合并已经摘离的原表页拥有，短锁内只移动节点，统一等锁外失效后归还。
    pub(crate) fn append(&mut self, mut other: Self) {
        while let Some(node) = other.owners.pop() {
            self.owners.push(node);
        }
    }

    /// 调用者已完成所有 CPU 的真实失效，逐个 Drop 原表拥有后才统计归页。
    pub(crate) fn release(mut self) -> usize {
        let mut freed = 0;
        while let Some(node) = self.owners.pop() {
            drop(node);
            freed += 1;
        }
        freed
    }

    /// 观察当前退休链的真实表页数，不从硬件地址重建拥有。
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

/// Mm 保活的用户根及私有中间页；共享内核表和常驻空表从不进入拥有链。
/// 析构前必须结束全部 CPU 对此根的活动使用并完成本地翻译失效。
/// 根物理地址只授予激活所需的标识，不授予外部修改硬件槽的权限。
pub struct UserPageTable {
    root: OwnedPages,
    tables: TableOwners,
}

impl UserPageTable {
    /// 在短锁外申请并完整初始化根；共享内核槽由架构后端借用永久内核表。
    pub fn try_new() -> Result<Self, PageTableError> {
        let mut root = alloc_pages(0, false).ok_or(PageTableError::NoMemory)?;
        // SAFETY: root 是尚未发表、独占且永久直映的真实 order-0 页；后端负责
        // 初始化完整根，并仅借用永久内核表或常驻空表，不向其转移归还权。
        unsafe { arch::initialize_user_root(root.as_mut_ptr().cast::<usize>()) };
        Ok(Self {
            root,
            tables: TableOwners::default(),
        })
    }

    /// 返回由本对象保活的硬件根标识，不构造任何数据页或表页的拥有权。
    pub fn root_phys(&self) -> usize {
        self.root.phys_addr()
    }

    /// 查询包含该用户地址的页映射，不分配、不等待，也不取得数据页拥有。
    pub fn lookup(&self, va: usize) -> Option<Mapping> {
        if arch::USER_ADDRESS_END <= va {
            return None;
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY: 用户范围不会访问共享内核槽；根初始化的空目录不参与追踪，
            // 其余目录只由本对象发表，目标页始终由 tables 保活。短锁阻止软件拆表。
            let entry = unsafe { read_slot(table, slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return None;
            }
            table = arch::user_entry_physical(entry, level);
        }
        // SAFETY: 上述逐级检查只到达本对象拥有且完整初始化的叶表，索引在页内。
        mapping(unsafe { read_slot(table, slot_index(va, 0)) })
    }

    /// 同一 PTE 锁内检查实际 PFN 并降 W/脏位，硬件访问位竞争由 CAS 保留。
    /// 返回脏观察与实际变化；调用者保页并在锁外完成所有 CPU 的翻译失效。
    ///
    /// # Safety
    /// 调用者须串行软件修改，physical 的原页须保活至全 CPU 同步失效完成。
    pub(crate) unsafe fn mkclean(&mut self, va: usize, physical: usize) -> Option<(bool, bool)> {
        let leaf = self.leaf_table(va)?;
        let pointer = arch::phys_to_virt(leaf) as *mut AtomicUsize;
        // SAFETY: leaf_table 验证真实拥有路径和页对齐，软件修改由调用者串行；
        // 槽始终原子访问，数据页由调用者保活，CAS 不产生用户内容引用。
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

    /// 仅对同 PFN 的现有叶改变访问权限，不申请目录，也不改变驻留拥有。
    ///
    /// # Safety
    /// 软件修改须受同一 PTE 锁保护，原页须保活到锁外全部 CPU 同步失效完成。
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
        // SAFETY: leaf_table 验证本表拥有的完整叶表，调用者串行软件修改并保页。
        let old = unsafe { read_slot(leaf, slot_index(va, 0)) };
        if mapping(old).is_none_or(|current| current.physical != physical) {
            return Ok(false);
        }
        // SAFETY: 同 PFN 改权不释放原页，旧翻译由调用者在锁外同步撤销。
        unsafe { replace_slot(leaf, slot_index(va, 0), next) };
        Ok(true)
    }

    /// 定位本对象实际拥有的叶表，不把常驻空目录或非法页地址当作可修改槽。
    fn leaf_table(&self, va: usize) -> Option<usize> {
        if !valid_page(va) {
            return None;
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY: 当前根和检查过的下级表由本对象保活，PTE 锁串行软件拆表。
            let entry = unsafe { read_slot(table, slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return None;
            }
            table = arch::user_entry_physical(entry, level);
        }
        Some(table)
    }

    /// 只解除一个叶而保留中间表，用于同步失效前的 break-before-make 交接。
    ///
    /// # Safety
    /// 调用者必须串行化软件修改，保活旧数据页到全活动 CPU 同步失效完成；
    /// 中间表仍由本对象拥有，失效前不得把新物理页或新权限发表到此叶。
    pub unsafe fn clear_leaf(&mut self, va: usize) -> Option<Mapping> {
        if !valid_page(va) {
            return None;
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY: 根和逐级非空目录均由本表保活；短锁串行软件修改，索引在页内。
            let entry = unsafe { read_slot(table, slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return None;
            }
            table = arch::user_entry_physical(entry, level);
        }
        // SAFETY: 叶表仍归本对象所有，调用者保活旧数据页到后续同步失效完成；
        // 本次只原子清空叶，不摘取或归还任何中间表。
        mapping(unsafe { replace_slot(table, slot_index(va, 0), arch::user_empty_entry(0)) })
    }

    /// 查询映射该页还需多少中间页，非法页基址或被大页占用的路径返回零。
    /// 调用者须在取得 PTE 锁后重新检查，不能把锁外计数视为发表许可。
    pub fn needed_tables(&self, va: usize) -> usize {
        self.probe_needed(va).unwrap_or(0)
    }

    /// 安装或替换一个用户叶，旧叶以非拥有快照交回 Mm；不在锁内分配或归页。
    /// 缺少准备页或输入无效时不修改任何硬件槽，也不消费准备拥有。
    ///
    /// # Safety
    /// Mm 必须持有 physical 指向的完整真实数据页的拥有；该页不得是硬件页表、
    /// 内核私有对象或仍有排他 Rust 引用的存储，权限必须允许用户访问。
    /// 数据初始化须在发表前完成。新页须保活到解除映射及全部活动 CPU 同步失效后，
    /// 替换的旧页同样须保活到同步失效完成。调用者必须串行化软件 PTE 修改，
    /// 在锁外完成 TLB/指令同步，再允许用户执行；prepared 必须在放锁后析构。
    pub unsafe fn map(
        &mut self,
        va: usize,
        physical: usize,
        permissions: PagePermissions,
        prepared: &mut PreparedTables,
    ) -> Result<Option<Mapping>, PageTableError> {
        let leaf = checked_leaf(physical, permissions)?;
        let needed = self.probe_needed(va)?;
        if prepared.tables[..needed].iter().any(Option::is_none) {
            return Err(PageTableError::MissingTables);
        }

        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            let index = slot_index(va, level);
            // SAFETY: preflight 确认路径中只有空槽或本对象拥有的目录，短锁期间
            // 无其他软件修改；root 和此前已发表的子页均保持 live。
            let mut entry = unsafe { read_slot(table, index) };
            if entry == arch::user_empty_entry(level) {
                let node = prepared.tables[level - 1]
                    .take()
                    .expect("prepared table level changed");
                let physical = node.pages.phys_addr();
                entry = arch::user_table_entry(physical);
                self.tables.push(node);
                // SAFETY: 新子页已完整初始化并转入本表唯一拥有链；父槽仍为
                // preflight 所见的空槽。Release 保证初始化先于目录对硬件可见。
                unsafe { replace_slot(table, index, entry) };
            }
            table = arch::user_entry_physical(entry, level);
        }

        // SAFETY: 叶表已由本对象拥有并完整初始化；调用者保活新旧数据页且负责
        // 后续同步失效。原子交换保留硬件可能更新过的旧槽实际值。
        let old = unsafe { replace_slot(table, slot_index(va, 0), leaf) };
        Ok(mapping(old))
    }

    /// 解除一个页并摘取沿途空中间表，任何归还都留给同步失效后的锁外阶段。
    /// 缺失叶仍可摘取已空的私有目录，覆盖换页清叶与文件失效之间的交错。
    ///
    /// # Safety
    /// 调用者必须串行化软件 PTE 修改，保活旧数据页，且不得在 PTE 锁内
    /// 析构返回的退休拥有。全部活动 CPU 的同步 TLB 失效完成前，不得归还
    /// 旧数据页或 RetiredTables；失效须同时结束对被摘离目录的硬件遍历。
    pub unsafe fn unmap(&mut self, va: usize) -> (Option<Mapping>, RetiredTables) {
        let mut retired = RetiredTables::default();
        if !valid_page(va) {
            return (None, retired);
        }
        let mut path = [0usize; arch::USER_LEVELS];
        path[arch::USER_LEVELS - 1] = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY: 当前父页来自本对象拥有的 root 或先前检查的私有目录，
            // 软件修改已串行化；索引在完整且已初始化的页内。
            let entry = unsafe { read_slot(path[level], slot_index(va, level)) };
            if !arch::user_entry_is_table(entry, level) {
                return (None, retired);
            }
            path[level - 1] = arch::user_entry_physical(entry, level);
        }

        // SAFETY: path[0] 是仍由本表拥有的叶表，用户叶和空槽均由后端解释；
        // 调用者保活旧数据页到后续同步失效完成。
        let old = unsafe { replace_slot(path[0], slot_index(va, 0), arch::user_empty_entry(0)) };
        let old = mapping(old);

        for level in 0..arch::USER_LEVELS - 1 {
            // SAFETY: 此子页尚未摘离且由本表保活，短锁阻止软件新增叶或目录。
            if !unsafe { table_is_empty(path[level], level) } {
                break;
            }
            // SAFETY: 子页全部槽已为空，父槽指向该真实私有子页；先解除父槽，
            // 再转移拥有到 retired。调用者必须在同步失效后才析构 retired。
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

    /// 只读检查逐级路径，把所有可能失败的准备检查放在第一处硬件修改之前。
    fn probe_needed(&self, va: usize) -> Result<usize, PageTableError> {
        if !valid_page(va) {
            return Err(PageTableError::InvalidRange);
        }
        let mut table = self.root_phys();
        for level in (1..arch::USER_LEVELS).rev() {
            // SAFETY: 用户根空槽不进入遍历，非空目录由本对象发表并保活，
            // 软件访问受同一 PTE 锁串行化；索引落在完整表页内。
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

/// 把用户页基址限制在后端规定范围内，永远不修改借用的内核根槽。
fn valid_page(va: usize) -> bool {
    va < arch::USER_ADDRESS_END && va & (PAGE_SIZE - 1) == 0
}

/// 从页地址取得该级硬件槽索引，架构共用的三层 4KiB 表每页有 512 槽。
fn slot_index(va: usize, level: usize) -> usize {
    (va >> (PAGE_SHIFT + INDEX_BITS * level)) & (ENTRIES - 1)
}

/// 用后端实际编解码检查权限及物理地址，避免地址截断或保留权限组合被发表。
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

/// 只按架构空叶语义解码非拥有快照，不从地址推导数据页归还权。
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

/// 原子读取仍由拥有者保活的硬件槽，兼容硬件对叶访问位的更新。
///
/// # Safety
/// physical 必须是当前根或拥有链中 live、完整初始化且永久直映的真实表页；
/// index 必须小于 ENTRIES，全部软件修改必须遵守同一 PTE 短锁协议。
unsafe fn read_slot(physical: usize, index: usize) -> usize {
    let pointer = arch::phys_to_virt(physical) as *mut AtomicUsize;
    // SAFETY: 调用者保证 live 且完整初始化的表页，页对齐满足 AtomicUsize，
    // 索引在页内，所有软件槽访问均为原子操作，不建立可变别名引用。
    unsafe { (*pointer.add(index)).load(Ordering::Acquire) }
}

/// 原子发表或摘离一个槽，使子表和数据初始化先于新目录或叶可见。
///
/// # Safety
/// physical/index 须满足 read_slot 协议；entry 必须是同层后端合法槽。
/// 调用者必须保活旧指向到同步失效完成、新指向到解除和同步失效完成。
unsafe fn replace_slot(physical: usize, index: usize, entry: usize) -> usize {
    let pointer = arch::phys_to_virt(physical) as *mut AtomicUsize;
    // SAFETY: 调用者保证表页及索引有效、软件修改串行化且新旧指向保活；
    // 原子交换不会撕裂硬件槽，Release 排序先前的初始化写入。
    unsafe { (*pointer.add(index)).swap(entry, Ordering::AcqRel) }
}

/// 逐槽检查完整表页是否为空，常驻共享空表不作为私有页进入本函数。
///
/// # Safety
/// physical 须是本表拥有且未摘离的私有表页；level 是其实际层级。
/// 调用者必须串行化软件修改，表页须保活到查询完成。
unsafe fn table_is_empty(physical: usize, level: usize) -> bool {
    let empty = arch::user_empty_entry(level);
    for index in 0..ENTRIES {
        // SAFETY: 调用者保证当前表页 live 且层级正确，循环索引始终在完整页内。
        if unsafe { read_slot(physical, index) } != empty {
            return false;
        }
    }
    true
}
