//! 用户进程虚拟地址空间（[`Mm`]）与硬件页表域（[`PteDomain`]）管理。
//!
//! # 模块职责与架构定位
//! 本模块是用户态内存管理的核心，负责将上层的虚拟内存区域（[`Vma`]）与底层的架构硬件页表（[`UserPageTable`]）、
//! 驻留物理页所有权（[`ResidentPages`]）以及文件系统页缓存（Page Cache）反向映射连接起来。
//! - **上游调用者**：系统调用层（`mmap` / `munmap` / `mprotect` / `brk` / `msync` / `futex`）、
//!   异常处理层（用户态缺页异常 `fault`）、进程管理层（`fork`、`execve` 与调度器上下文切换 `enter_current` / `leave_current`）。
//! - **下游依赖**：[`crate::mm::vma`]（VMA 区间索引）、[`crate::mm::user_table`]（硬件多级页表操作）、
//!   [`crate::cache`] 与 [`crate::fs`]（文件页缓存与反向映射注销）、[`crate::cpu`]（跨核 IPI 同步刷新 TLB / I-Cache）。
//!
//! # 核心设计：区间睡眠锁与页表自旋锁职责分离
//! 为支持文件映射的后台脏页回写（Writeback）与文件截断（Truncate）在不获取进程 `mmap` 睡眠锁的情况下直接清理页表项，
//! 每个地址空间被拆分为两个独立的同步域：
//! 1. **虚拟区间域（`mmap: SleepRwLock<MmapState>`）**：可睡眠读写锁，保护有序 VMA 列表与堆边界（`heap_start` / `heap_end`）。
//! 2. **页表与驻留页域（`table: Arc<PteDomain>`）**：自旋锁，同时保护架构硬件页表（`UserPageTable`）与驻留页链表（`ResidentPages`）。
//!    任何时刻，硬件叶子 PTE 的物理页帧号（PFN）必须与 `ResidentPages` 中对应节点持有的物理页引用严格一致。
//!
//! ```text
//!                        +-------------------------------+
//!                        |              Mm               |
//!                        +---------------+---------------+
//!                                        |
//!               +------------------------+------------------------+
//!               v                                                 v
//! +---------------------------+                     +---------------------------+
//! |  mmap: SleepRwLock        |                     |  table: Arc<PteDomain>    |
//! |  (保护有序 VMA 与堆边界)    |                     |  (SpinLock<PteState>)     |
//! +-------------+-------------+                     +-------------+-------------+
//!               |                                                 |
//!               v                                   +-------------+-------------+
//!       +---------------+                           |                           |
//!       |      Vma      |                           v                           v
//!       |  file_range --+--(强引用 Arc)--> +-----------------+         +-----------------+
//!       +---------------+                  |  FileRangeData  |<--(Weak)|  ResidentPages  |
//!                                          |  domain: Arc ---+--(强)-->|  & UserPageTable|
//!                                          +-----------------+         +-----------------+
//! ```
//! 如上图所示，`Vma` 持有 `FileRangeRegistration`（内部强引用 `FileRangeData`），`FileRangeData` 强引用 `Arc<PteDomain>`，
//! 而 `ResidentPage` 仅通过 `Weak<FileRangeData>` 弱引用文件范围来源，从而避免了引用计数循环，并允许页缓存通过
//! `FileRangeData -> PteDomain` 直接定位并清理对应文件的页表项。
//!
//! # 锁获取顺序（Lock Ordering）与 TLB 同步契约
//! 多锁嵌套时必须严格遵守以下从外到内的获取顺序（严禁反向获取）：
//! 1. `Mm::mmap`（`SleepRwLock<MmapState>`，可睡眠读写锁）
//! 2. `Inode::lock_invalidate`（文件截断/失效睡眠锁，仅 `mkwrite` 路径涉及）
//! 3. `Folio::lock_content`（文件页缓存内容睡眠锁）
//! 4. `AddressSpace::lock_ranges`（文件反向映射范围自旋锁）
//! 5. `PteDomain::state`（`SpinLock<PteState>`，页表与驻留页自旋锁，不可睡眠）
//!
//! **TLB 与物理页回收铁律**：从 `PteState` 摘除或降权硬件 PTE 后，旧的 `ResidentPage` 和空中间页表必须暂存在退休链表中，
//! **必须在释放所有自旋锁之后调用 [`sync_all`] 完成全核 TLB 与指令缓存刷新**，最后才能在锁外析构退休节点并归还物理页。

use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::cmp::Ordering as CmpOrdering;
use core::mem::size_of;
use core::ops::Range;
use core::slice;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::arch::{self, UserAccess};
use crate::cache::FileRangeData;
use crate::cpu;
use crate::fs::{ExecAccess, FileRef, FsError, InodeKind, InodeRef, O_ACCMODE, O_RDWR, O_WRONLY};
use crate::mm::anon::SharedAnon;
use crate::mm::reclaim::ReclaimAttempt;
#[cfg(feature = "ktest")]
use crate::mm::user_table::Mapping;
use crate::mm::user_table::{
    PagePermissions, PageTableError, PreparedTables, RetiredTables, UserPageTable,
};
use crate::mm::vma::{self, MmapState, Vma, VmaBacking};
use crate::mm::{AnonPageRef, FolioRef, OwnedPages, PAGE_SIZE};
use crate::sync::{IrqSaveGuard, ReadGuard, SleepRwLock, SpinLock, SpinLockGuard};
use crate::task::{self, process::Resource};

/// 用户虚拟内存操作错误类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VmError {
    /// 物理内存或内核堆分配不足（对应 `ENOMEM`）。
    NoMemory,
    /// 目标文件描述符不支持内存映射，例如管道、套接字或缺少挂载路径（对应 `ENODEV` / `EACCES`）。
    NotMappable,
    /// 虚拟地址区间、长度、对齐方式或文件偏移量不合法（对应 `EINVAL`）。
    InvalidRange,
    /// `MAP_FIXED_NOREPLACE` 或固定插入时目标虚拟地址区间已被占用（对应 `EEXIST`）。
    Occupied,
    /// 访问的地址未建立映射，或操作区间内包含未映射空洞（对应 `EFAULT` / `ENOMEM`）。
    Unmapped,
    /// 访问方式违反 VMA 当前权限，或请求权限超出最大权限上限（对应 `EACCES` / `EPERM`）。
    Permission,
    /// 文件映射缺页访问的位置已达到或超过文件当前字节大小 EOF（对应触发 `SIGBUS`）。
    BeyondEof,
    /// 底层文件系统或块设备 I/O 错误。
    File(FsError),
}

impl From<PageTableError> for VmError {
    /// 将底层硬件页表操作错误转换为虚拟内存错误 [`VmError`]。
    ///
    /// 若出现 [`PageTableError::MissingTables`]，说明调用方违反了“先在锁外预分配中间页表、再持锁提交”的内部协议，直接触发 panic。
    fn from(error: PageTableError) -> Self {
        match error {
            PageTableError::NoMemory => Self::NoMemory,
            PageTableError::InvalidRange => Self::InvalidRange,
            PageTableError::Occupied => Self::Occupied,
            PageTableError::MissingTables => {
                panic!("user mapping committed without prepared tables")
            }
        }
    }
}

/// `mmap` 系统调用的虚拟地址选址策略。
pub(crate) enum MmapAddress {
    /// 启发式选址（未指定 `MAP_FIXED`）：优先尝试给定提示地址，若被占用或低于最小阈值则由内核自动寻找空闲区间。
    Hint(usize),
    /// 强制固定地址并允许覆盖（`MAP_FIXED`）：精确映射到该地址，若与现有 VMA 重叠则自动拆分并解除被覆盖部分的旧映射。
    Fixed(usize),
    /// 强制固定地址但禁止覆盖（`MAP_FIXED_NOREPLACE`）：精确映射到该地址，若与现有 VMA 存在任何重叠则直接返回 [`VmError::Occupied`]。
    FixedNoReplace(usize),
}

/// `mmap` 系统调用的数据源类型。
pub(crate) enum MmapSource {
    /// 私有匿名映射（`MAP_PRIVATE | MAP_ANONYMOUS`）。
    PrivateAnonymous,
    /// 共享匿名映射（`MAP_SHARED | MAP_ANONYMOUS`），每次 `mmap` 调用都会创建一个新的 [`SharedAnon`] 容器。
    SharedAnonymous,
    /// 文件映射（`MAP_SHARED` 或 `MAP_PRIVATE`），直接持有文件打开实例与索引节点引用。
    File {
        /// 被映射的文件打开实例。
        file: FileRef,
        /// 被映射文件的索引节点引用。
        inode: InodeRef,
        /// 映射起始位置在文件中的字节偏移量（需按 [`PAGE_SIZE`] 对齐）。
        offset: u64,
        /// 为 `true` 表示私有写时复制映射（`MAP_PRIVATE`），为 `false` 表示共享回写映射（`MAP_SHARED`）。
        private: bool,
    },
}

/// 经过系统调用入口初步解析后的 `mmap` 请求参数包。
pub(crate) struct MmapRequest {
    /// 目标虚拟地址及选址模式。
    pub(crate) address: MmapAddress,
    /// 请求映射的原始字节长度（尚未向上按页取整）。
    pub(crate) length: usize,
    /// 初始页面访问权限（由 `prot` 参数转换而来）。
    pub(crate) permissions: PagePermissions,
    /// 该映射允许通过 `mprotect` 提升到的最大访问权限（受打开文件描述符的读写模式限制）。
    pub(crate) maximum: PagePermissions,
    /// 映射的数据源类型。
    pub(crate) source: MmapSource,
}

/// 已驻留在进程页表中的单页物理内存后备引用。
///
/// 持有该枚举即保证底层物理页已分配且数据已初始化完毕：
/// - 对共享文件映射的写入必须先走 `mkwrite` 将页缓存标记为脏页；
/// - 对私有映射的写入会触发写时复制（COW），分配新的匿名页替换原引用。
#[derive(Clone)]
enum ResidentBacking {
    /// 匿名物理页引用（用于私有匿名页、共享匿名页以及私有文件映射发生 COW 后的独立副本页）。
    Anonymous(AnonPageRef),
    /// 文件页缓存（Page Cache）物理页引用（用于尚未发生私有写复制的文件映射页）。
    File(FolioRef),
}

impl ResidentBacking {
    /// 返回该驻留页持有的底层物理地址（字节地址，按页对齐）。
    fn physical(&self) -> usize {
        match self {
            Self::Anonymous(page) => page.phys_addr(),
            Self::File(page) => page.phys_addr(),
        }
    }
}

/// 单个虚拟页在页表域（[`PteDomain`]）中的驻留所有权节点。
///
/// 每个已安装的硬件叶子 PTE 都必须有一个对应的 `ResidentPage` 节点来保持底层物理页的引用计数，防止物理页在硬件页表仍指向它时被提前释放。
struct ResidentPage {
    /// 该页对应的用户虚拟页起始地址（按 [`PAGE_SIZE`] 对齐）。
    address: usize,
    /// 该虚拟页当前绑定的底层物理页引用（匿名页或文件缓存页）。
    backing: ResidentBacking,
    /// 软件写时复制（Copy-on-Write）标志：为 `true` 时硬件 PTE 强制保持只读，写入时触发缺页复制。
    cow: bool,
    /// 若该页位于文件映射 VMA 中，弱引用所属 VMA 的 [`FileRangeData`]，供反向映射（rmap）校验该页是否属于指定的文件映射区间。
    source: Weak<FileRangeData>,
    /// 指向链表中下一个驻留页节点的指针。
    next: Option<Box<ResidentPage>>,
}

impl ResidentPage {
    /// 在页表自旋锁之外预分配并初始化一个未入链的 [`ResidentPage`] 节点。
    fn try_new(address: usize, backing: ResidentBacking, cow: bool) -> Result<Box<Self>, VmError> {
        Box::try_new(Self {
            address,
            backing,
            cow,
            source: Weak::new(),
            next: None,
        })
        .map_err(|_| VmError::NoMemory)
    }

    /// 根据所属 VMA 的软件权限 `permissions` 与当前页状态，计算允许写入硬件叶子 PTE 的实际权限。
    ///
    /// 若当前页带有 `cow` 标志，或后备为文件页缓存（[`ResidentBacking::File`]），则默认屏蔽硬件写权限（保持只读）；
    /// 共享文件页的硬件写权限仅能由持有文件页内容锁的 [`Mm::mkwrite`] 路径在完成脏页登记后显式开启。
    fn permissions(&self, permissions: PagePermissions) -> PagePermissions {
        PagePermissions {
            write: permissions.write
                && !self.cow
                && !matches!(&self.backing, ResidentBacking::File(_)),
            ..permissions
        }
    }
}

/// 进程页表域中的驻留物理页单向链表。
///
/// 节点分配与销毁均在页表自旋锁外进行，持锁期间仅做指针摘挂；
/// 析构时采用循环迭代逐个释放节点，防止大量驻留页导致递归 Drop 撑爆内核栈。
#[derive(Default)]
pub(crate) struct ResidentPages {
    /// 链表头节点指针。
    head: Option<Box<ResidentPage>>,
}

impl ResidentPages {
    /// 线性查找虚拟页基址为 `address` 的驻留页节点只读引用。
    fn find(&self, address: usize) -> Option<&ResidentPage> {
        let mut cursor = self.head.as_deref();
        while let Some(page) = cursor {
            if page.address == address {
                return Some(page);
            }
            cursor = page.next.as_deref();
        }
        None
    }

    /// 线性查找虚拟页基址为 `address` 的驻留页节点可变引用（用于在页表自旋锁内更新 `cow` 或 `source` 字段）。
    fn find_mut(&mut self, address: usize) -> Option<&mut ResidentPage> {
        let mut cursor = self.head.as_deref_mut();
        while let Some(page) = cursor {
            if page.address == address {
                return Some(page);
            }
            cursor = page.next.as_deref_mut();
        }
        None
    }

    /// 将已在锁外分配好的独立节点 `page` 头插到链表中（持锁期间不触发内存分配或旧页释放）。
    fn push(&mut self, mut page: Box<ResidentPage>) {
        assert!(page.next.is_none(), "resident page is already linked");
        page.next = self.head.take();
        self.head = Some(page);
    }

    /// 从链表中摘除并返回虚拟页基址为 `address` 的节点；调用方必须在完成全核 TLB 刷新且释放自旋锁后再将其析构。
    fn remove(&mut self, address: usize) -> Option<Box<ResidentPage>> {
        let mut link = &mut self.head;
        while link.is_some() {
            if link.as_ref().unwrap().address == address {
                let mut page = link.take().unwrap();
                *link = page.next.take();
                return Some(page);
            }
            link = &mut link.as_mut().unwrap().next;
        }
        None
    }

    /// 统计虚拟地址区间 `range` 内当前驻留页的数量，供调用方在自旋锁外预分配精确容量的 `Vec`。
    pub(crate) fn count(&self, range: &Range<usize>) -> usize {
        let mut count = 0;
        let mut cursor = self.head.as_deref();
        while let Some(page) = cursor {
            count += usize::from(range.contains(&page.address));
            cursor = page.next.as_deref();
        }
        count
    }

    /// 将区间 `range` 内所有驻留页的虚拟基址收集到已预留足够容量的 `addresses` 向量中（严禁在持页表自旋锁期间触发 `Vec` 扩容分配）。
    fn addresses(&self, range: &Range<usize>, addresses: &mut Vec<usize>) {
        let mut cursor = self.head.as_deref();
        while let Some(page) = cursor {
            if range.contains(&page.address) {
                assert!(addresses.len() < addresses.capacity());
                addresses.push(page.address);
            }
            cursor = page.next.as_deref();
        }
    }
}

impl Drop for ResidentPages {
    /// 迭代式摘除并释放链表中的所有驻留页节点，避免深层递归析构导致内核栈溢出。
    fn drop(&mut self) {
        while let Some(mut page) = self.head.take() {
            self.head = page.next.take();
            drop(page);
        }
    }
}

/// 受 [`PteDomain`] 自旋锁保护的硬件页表与驻留页联合状态。
struct PteState {
    /// 架构相关的用户多级硬件页表。
    table: UserPageTable,
    /// 与硬件叶子 PTE 一一对应的物理页所有权链表。
    pages: ResidentPages,
}

impl PteState {
    /// 检查虚拟页基址 `address` 在当前页表中是否已就绪且允许 `access` 类型的访问。
    ///
    /// 仅当驻留节点存在、硬件叶子 PTE 指向同一个物理页且具备相应硬件权限，并且不是触发写时复制（`cow && Write`）的写入时返回 `true`。
    fn allows(&self, address: usize, access: UserAccess) -> bool {
        self.pages.find(address).is_some_and(|page| {
            self.table.lookup(address).is_some_and(|leaf| {
                leaf.physical == page.backing.physical() && vma::allows(leaf.permissions, access)
            }) && !(page.cow && access == UserAccess::Write)
        })
    }
}

/// 地址空间的硬件页表同步域（PTE Domain）。
///
/// 独立封装为 `Arc<PteDomain>`，使得文件页缓存反向映射（`FileRangeData`）可以持有该域的强引用，
/// 在不获取进程 `Mm::mmap` 睡眠锁的情况下直接加自旋锁清理或撤除文件页映射。
pub(crate) struct PteDomain {
    /// 保护硬件页表与驻留页链表的自旋锁。
    state: SpinLock<PteState>,
}

impl PteDomain {
    /// 获取页表与驻留页状态的自旋锁守卫。
    fn lock(&self) -> SpinLockGuard<'_, PteState> {
        self.state.lock()
    }

    /// 将文件映射区间 `source` 中文件偏移 `offset`、物理地址 `physical` 对应的硬件叶子 PTE 清除写权限与脏位（变为只读干净态）。
    ///
    /// 由页缓存脏页回写（Writeback）路径在持有文件页内容锁与反向映射范围锁时调用，无需获取进程的 `Mm::mmap` 锁。
    /// 返回元组 `(是否观察到硬件脏位, 是否实际修改了页表项的计数 0 或 1)`；调用方负责在释放全部自旋锁后执行全核 TLB 同步。
    pub(crate) fn mkclean_range(
        &self,
        source: &Arc<FileRangeData>,
        offset: u64,
        physical: usize,
    ) -> (bool, usize) {
        let Some(relative) = offset.checked_sub(source.offset) else {
            return (false, 0);
        };
        if (source.range.end - source.range.start) as u64 <= relative {
            return (false, 0);
        }

        let address = source.range.start + relative as usize;
        let mut state = self.lock();
        let Some(page) = state.pages.find(address) else {
            return (false, 0);
        };
        if page.source.as_ptr() != Arc::as_ptr(source)
            || !matches!(&page.backing, ResidentBacking::File(_))
            || page.backing.physical() != physical
        {
            return (false, 0);
        }
        // SAFETY:
        // 1. 当前持有 `self.state` 页表自旋锁，且上方已核实 `page` 的 `source` 指针、文件后备类型及物理地址均与参数 `physical` 完全一致；
        // 2. `page` 节点与调用方持有的文件页内容锁共同保证底层物理页存活；
        // 3. `mkclean` 仅原地移除同一个物理页的写权限，不释放任何数据页或页表页，调用方会在释放所有自旋锁后执行跨核 TLB 刷新。
        unsafe { state.table.mkclean(address, physical) }
            .map_or((false, 0), |(dirty, changed)| (dirty, usize::from(changed)))
    }

    /// 卸载文件映射区间 `source` 在文件字节偏移范围 `range` 内的所有匹配驻留页与硬件 PTE，并将摘下的节点移入退休链表。
    ///
    /// 当 `even_cows` 为 `false` 时，仅卸载仍直接引用文件页缓存（[`ResidentBacking::File`]）的页面；
    /// 当 `even_cows` 为 `true` 时，连同该文件映射区间内已发生写时复制的私有匿名副本页一并卸载。
    ///
    /// # Safety
    /// 1. 调用方必须持有 `source` 所属文件 `AddressSpace` 的范围自旋锁（`lock_ranges`），并在调用期间保证 `source` 与 `self` 存活；
    /// 2. 传入的 `retired_pages` 与 `retired_tables` 在收集被卸载的物理页和空页表后，**必须在释放全部自旋锁并完成全核 TLB 同步（[`sync_all`]）之后**才能析构释放。
    pub(crate) unsafe fn unmap_range(
        &self,
        source: &FileRangeData,
        range: &Range<u64>,
        even_cows: bool,
        retired_pages: &mut ResidentPages,
        retired_tables: &mut RetiredTables,
    ) -> usize {
        let mut state = self.lock();
        let PteState { table, pages } = &mut *state;
        let mut link = &mut pages.head;
        let mut unmapped = 0;
        while let Some(page) = link.as_ref() {
            if page.source.as_ptr() != source as *const FileRangeData
                || !source.range.contains(&page.address)
                || !even_cows && !matches!(&page.backing, ResidentBacking::File(_))
            {
                link = &mut link.as_mut().unwrap().next;
                continue;
            }
            let offset = source
                .offset
                .checked_add((page.address - source.range.start) as u64)
                .expect("registered file offset overflow");
            if range.end <= offset
                || offset.saturating_add(PAGE_SIZE as u64) <= range.start
                || table
                    .lookup(page.address)
                    .is_some_and(|leaf| leaf.physical != page.backing.physical())
            {
                link = &mut link.as_mut().unwrap().next;
                continue;
            }
            // SAFETY:
            // 1. 调用方持有文件范围自旋锁，阻止了并发的 VMA 来源替换与 `fork` 映射提交；当前持有 `self.state` 自旋锁且已核实该页属于 `source` 且硬件叶子物理地址匹配；
            // 2. 被卸载的物理页所有权随 `page` 节点移入 `retired_pages`，可能腾空的中间页表移入 `retired_tables`，两者均在释放全部自旋锁并完成全核 TLB 刷新后才真正释放。
            let (old, tables) = unsafe { table.unmap(page.address) };
            unmapped += usize::from(old.is_some());
            let mut page = link.take().unwrap();
            *link = page.next.take();
            retired_pages.push(page);
            retired_tables.append(tables);
        }
        unmapped
    }

    /// 针对单个文件页缓存（`offset` 与 `physical`），从当前页表域中卸载匹配的共享或未写复制文件映射叶子 PTE。
    ///
    /// 已发生写时复制（COW）的私有匿名页或其他文件偏移的映射不受影响。返回实际卸载的页表项数量（`0` 或 `1`）。
    ///
    /// # Safety
    /// 1. 调用方必须持有 `source` 所属文件的范围自旋锁，并保证 `source` 及其对应的文件页 `Folio` 在调用期间存活；
    /// 2. 移入 `retired_pages` 与 `retired_tables` 的节点必须在释放全部自旋锁并完成全核 TLB 同步（[`sync_all`]）之后才能析构，严禁将摘除页表项视为硬件已立即停止访问该物理页。
    pub(crate) unsafe fn unmap_file_folio(
        &self,
        source: &Arc<FileRangeData>,
        offset: u64,
        physical: usize,
        retired_pages: &mut ResidentPages,
        retired_tables: &mut RetiredTables,
    ) -> usize {
        let Some(relative) = offset.checked_sub(source.offset) else {
            return 0;
        };
        if (source.range.end - source.range.start) as u64 <= relative {
            return 0;
        }

        let address = source.range.start + relative as usize;
        let mut state = self.lock();
        let Some(page) = state.pages.find(address) else {
            return 0;
        };
        if page.source.as_ptr() != Arc::as_ptr(source)
            || !matches!(&page.backing, ResidentBacking::File(_))
            || page.backing.physical() != physical
            || state
                .table
                .lookup(address)
                .is_some_and(|leaf| leaf.physical != physical)
        {
            return 0;
        }
        // SAFETY:
        // 1. 当前持有 `self.state` 页表自旋锁，且已核实驻留节点的 `source`、文件后备类型及硬件叶子 PTE 的物理地址均等于 `physical`；
        // 2. 摘除的 `ResidentPage` 与空中间页表立即移入 `retired_pages` 和 `retired_tables` 保活，调用方在锁外完成全核 TLB 刷新后才会释放。
        let (old, tables) = unsafe { state.table.unmap(address) };
        retired_pages.push(
            state
                .pages
                .remove(address)
                .expect("file resident disappeared under PTE lock"),
        );
        retired_tables.append(tables);
        usize::from(old.is_some())
    }
}

/// 缺页处理在锁外准备物理页时采集的当前映射快照。
///
/// 由 [`Mm::fault_source`] 在持有 `mmap` 读锁与 `PteDomain` 自旋锁时构造；
/// 锁外完成物理页与页表分配后，通过比对 `vma` 指针、`observed` 物理地址和 `needed` 缺表数来检验期间是否发生并发修改。
struct FaultSource {
    /// 缺页地址所属的不可变 `Vma` 实例引用。
    vma: Arc<Vma>,
    /// 采样时该虚拟页已存在的驻留后备页克隆（若当时尚未映射则为 `None`）。
    backing: Option<ResidentBacking>,
    /// 采样时该虚拟页驻留后备的物理地址（用于提交阶段 CAS 式校验驻留页是否被并发替换或卸载）。
    observed: Option<usize>,
    /// 采样时为映射该虚拟地址仍需分配的中间级页表页数量。
    needed: usize,
}

/// 缺页与 `fork` 测试注入点共用的单虚拟地址回调函数类型。
#[cfg(feature = "ktest")]
type AddressHook = Arc<dyn Fn(usize) + Send + Sync>;

/// VMA 区间卸载测试注入点使用的 `(start, end)` 双地址回调函数类型。
#[cfg(feature = "ktest")]
type VmaUnmapHook = Arc<dyn Fn(usize, usize) + Send + Sync>;

/// 用户进程虚拟地址空间（Memory Descriptor）。
///
/// 内部将虚拟内存区域锁（`mmap` 睡眠读写锁）与硬件页表锁（`table` 自旋锁）分离；
/// 当线程在某个 CPU 上运行时，该 CPU 的 `active_mm` 槽位也会持有一份 `Arc<Mm>` 强引用，直到上下文切换回内核根页表为止。
pub struct Mm {
    /// 硬件页表与驻留物理页同步域（与文件反向映射共享强引用）。
    table: Arc<PteDomain>,
    /// 保护有序 VMA 列表与堆边界（`brk`）的可睡眠读写锁。
    mmap: SleepRwLock<MmapState>,
    /// 绑定到该地址空间的活动用户线程数量（由 [`MmUser`] 维护，不含 CPU 活动槽位或 `FutexKey` 持有的 `Arc<Mm>`）。
    users: AtomicUsize,
    /// 主可执行文件引用及其执行期写互斥凭证（`ETXTBSY` 保护），保持到该 `Mm` 的最后一个强引用释放为止。
    executable: Option<(FileRef, ExecAccess)>,
    /// 动态链接器（ELF `PT_INTERP` 解释器）文件引用及其执行期写互斥凭证。
    interpreter: Option<(FileRef, ExecAccess)>,
    /// 测试专用钩子：在 `mkwrite` 完成文件页准备后、重新获取 `mmap` 写锁前触发。
    #[cfg(feature = "ktest")]
    mkwrite_hook: SpinLock<Option<Arc<dyn Fn() + Send + Sync>>>,
    /// 测试专用钩子：在文件缺页持有页内容锁并核实 EOF 后、安装 PTE 前触发。
    #[cfg(feature = "ktest")]
    fault_hook: SpinLock<Option<AddressHook>>,
    /// 测试专用钩子：在 `fork` 采样父进程驻留页后、向子进程页表映射前触发。
    #[cfg(feature = "ktest")]
    fork_hook: SpinLock<Option<AddressHook>>,
    /// 测试专用钩子：在 `replace_range` 发布新 VMA 列表后、逐页摘除旧硬件 PTE 前触发。
    #[cfg(feature = "ktest")]
    vma_unmap_hook: SpinLock<Option<VmaUnmapHook>>,
    /// 测试专用计数器：统计 `mkwrite` 成功将共享文件页授予硬件写权限的次数。
    #[cfg(feature = "ktest")]
    mkwrites: AtomicUsize,
}

/// 更新 VMA 列表后需要在锁外延迟析构的旧 VMA 列表与旧文件反向映射弱引用集合。
type RetiredVmas = (Vec<Arc<Vma>>, Vec<(usize, Weak<FileRangeData>)>);

/// 用户线程对地址空间 [`Mm`] 的绑定凭证（RAII 守卫）。
///
/// 仅代表真实用户线程对地址空间的拥有关系；CPU 活动页表槽位、页表反向映射与 `FutexKey` 持有的 `Arc<Mm>` 不计入 `users`。
pub struct MmUser {
    /// 被绑定的用户地址空间强引用。
    mm: Arc<Mm>,
}

impl MmUser {
    /// 为新用户线程建立地址空间绑定并原子递增 `users` 计数；后续无论正常退出还是中途失败，均由 `Drop` 自动归还计数。
    pub fn new(mm: Arc<Mm>) -> Self {
        mm.users
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .expect("MM user count overflow");
        Self { mm }
    }

    /// 获取当前绑定持有的地址空间 [`Arc<Mm>`] 引用。
    pub fn mm(&self) -> &Arc<Mm> {
        &self.mm
    }
}

impl Drop for MmUser {
    /// 递减地址空间的绑定线程计数；`Arc<Mm>` 自身的引用递减与最终析构在随后自动完成。
    fn drop(&mut self) {
        let previous = self.mm.users.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "MM user count underflow");
    }
}

/// Futex 等待队列的全局唯一标识键（Futex Key）。
///
/// 依据映射类型直接绑定到地址空间 `Mm`、共享匿名对象 `SharedAnon` 或文件索引节点 `InodeRef` 的逻辑字节偏移，
/// 绝不将可能被回收复用的物理页帧号（PFN）作为长期等待键。
#[derive(Clone)]
pub(crate) enum FutexKey {
    /// 基于单个进程地址空间虚拟地址的等待键（用于 `FUTEX_PRIVATE_FLAG` 或匿名私有映射/已写时复制的私有文件页）。
    Mm {
        /// 所属进程的地址空间。
        mm: Arc<Mm>,
        /// 用户态 32 位 futex 字所在的精确虚拟地址（4 字节对齐）。
        address: usize,
        /// 是否携带了 `FUTEX_PRIVATE_FLAG`（私有标志与非私有标志即使在同一 `Mm` 同一地址也互不匹配）。
        private: bool,
    },
    /// 基于共享匿名映射（`MAP_SHARED | MAP_ANONYMOUS`）对象的跨进程等待键。
    Shared {
        /// 共享匿名内存对象。
        owner: Arc<SharedAnon>,
        /// futex 字在共享匿名对象内的精确字节偏移（含页内偏移）。
        offset: u64,
    },
    /// 基于文件映射（`MAP_SHARED` 文件映射或尚未触发 COW 的只读 `MAP_PRIVATE` 文件映射）的跨进程等待键。
    File {
        /// 被映射文件的规范索引节点（Inode）。
        inode: InodeRef,
        /// futex 字在文件中的精确字节偏移（含页内偏移）。
        offset: u64,
    },
}

impl FutexKey {
    /// 判断两个 `FutexKey` 是否指向同一个底层同步对象与同一个精确字节偏移。
    pub(crate) fn matches(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Self::Mm {
                    mm,
                    address,
                    private,
                },
                Self::Mm {
                    mm: other_mm,
                    address: other_address,
                    private: other_private,
                },
            ) => Arc::ptr_eq(mm, other_mm) && address == other_address && private == other_private,
            (
                Self::Shared { owner, offset },
                Self::Shared {
                    owner: other_owner,
                    offset: other_offset,
                },
            ) => Arc::ptr_eq(owner, other_owner) && offset == other_offset,
            (
                Self::File { inode, offset },
                Self::File {
                    inode: other_inode,
                    offset: other_offset,
                },
            ) => inode.ptr_eq(other_inode) && offset == other_offset,
            _ => false,
        }
    }

    /// 计算用于选择全局 Futex 哈希桶的哈希值（桶内冲突仍通过 [`FutexKey::matches`] 精确比对）。
    pub(crate) fn hash(&self) -> usize {
        let (owner, position, kind) = match self {
            Self::Mm {
                mm,
                address,
                private,
            } => (Arc::as_ptr(mm) as usize, *address, usize::from(!*private)),
            Self::Shared { owner, offset } => {
                (Arc::as_ptr(owner) as usize, *offset as usize, 2usize)
            }
            Self::File { inode, offset } => {
                (&**inode as *const _ as usize, *offset as usize, 3usize)
            }
        };
        (owner >> 4).wrapping_mul(0x9e37_79b9) ^ (position >> 2) ^ kind
    }
}

/// 解析 Futex 地址或读取 Futex 字时可能出现的错误。
///
/// 当跨进程文件映射需要通过缺页或写时复制（COW）来确定最终后备对象时，返回 [`FutexAccessError::NeedsFault`]，
/// 调用方必须先释放 `mmap` 读锁并调用 [`Mm::fault`]，随后重试。
pub(crate) enum FutexAccessError {
    /// 虚拟地址越界、未对齐、未映射或权限不足等常规虚拟内存错误。
    Vm(VmError),
    /// 当前页尚未驻留或处于待写时复制状态，需要释放读锁后按指定访问类型执行一次缺页处理。
    NeedsFault(UserAccess),
}

impl From<VmError> for FutexAccessError {
    /// 将虚拟内存错误包装为 [`FutexAccessError::Vm`]。
    fn from(error: VmError) -> Self {
        Self::Vm(error)
    }
}

/// Futex 操作期间持有的地址空间 `mmap` 读锁会话守卫。
///
/// 保证从解析 [`FutexKey`]、读取用户态 32 位字到将等待者挂入哈希桶的整个过程中，目标虚拟地址对应的 VMA 不会被并发卸载或重映射。
pub(crate) struct FutexRead<'a> {
    /// 目标地址空间引用。
    mm: &'a Arc<Mm>,
    /// 已获取的 `MmapState` 睡眠读锁守卫。
    mmap: ReadGuard<'a, MmapState>,
}

impl FutexRead<'_> {
    /// 在持有 `mmap` 读锁的前提下，解析用户虚拟地址 `address` 对应的 [`FutexKey`]。
    pub(crate) fn key(&self, address: usize, private: bool) -> Result<FutexKey, FutexAccessError> {
        if address & (size_of::<u32>() - 1) != 0
            || address
                .checked_add(size_of::<u32>())
                .is_none_or(|end| arch::USER_ADDRESS_END < end)
        {
            return Err(VmError::InvalidRange.into());
        }
        if private {
            return Ok(FutexKey::Mm {
                mm: Arc::clone(self.mm),
                address,
                private: true,
            });
        }
        let vma = self.mmap.find(address).ok_or(VmError::Unmapped)?;
        if !vma.allows(UserAccess::Read) {
            return Err(VmError::Permission.into());
        }
        match &vma.backing {
            VmaBacking::SharedAnonymous { owner, offset } => {
                return Ok(FutexKey::Shared {
                    owner: Arc::clone(owner),
                    offset: offset
                        .checked_add((address - vma.range.start) as u64)
                        .ok_or(VmError::InvalidRange)?,
                });
            }
            VmaBacking::File {
                offset, private, ..
            } => {
                let base = address & !(PAGE_SIZE - 1);
                let state = self.mm.table.lock();
                let Some(page) = state.pages.find(base) else {
                    let access = if *private && vma.allows(UserAccess::Write) {
                        UserAccess::Write
                    } else {
                        UserAccess::Read
                    };
                    return Err(FutexAccessError::NeedsFault(access));
                };
                if *private {
                    if vma.allows(UserAccess::Write) && page.cow {
                        return Err(FutexAccessError::NeedsFault(UserAccess::Write));
                    }
                    if matches!(&page.backing, ResidentBacking::Anonymous(_)) {
                        if !vma.allows(UserAccess::Write) {
                            return Err(VmError::Permission.into());
                        }
                        return Ok(FutexKey::Mm {
                            mm: Arc::clone(self.mm),
                            address,
                            private: false,
                        });
                    }
                }
                return Ok(FutexKey::File {
                    inode: vma
                        .file_range
                        .as_ref()
                        .expect("file VMA has no range")
                        .data()
                        .inode
                        .clone(),
                    offset: offset
                        .checked_add((address - vma.range.start) as u64)
                        .ok_or(VmError::InvalidRange)?,
                });
            }
            _ => {}
        }
        if !vma.allows(UserAccess::Write) {
            return Err(VmError::Permission.into());
        }
        Ok(FutexKey::Mm {
            mm: Arc::clone(self.mm),
            address,
            private: false,
        })
    }

    /// 在持有 `mmap` 读锁与 `PteDomain` 自旋锁的保护下，将用户态 32 位字地址转换为内核直接映射虚拟地址并调用 `visit`。
    ///
    /// 若该页尚未驻留或无读权限则返回 `None`（调用方需释放 `FutexRead` 后触发缺页）。
    /// `visit` 闭包在自旋锁临界区内执行，仅允许执行原子的 32 位字读写与固定哈希桶操作，**严禁睡眠、动态内存分配或将裸地址逃逸到闭包外**。
    pub(crate) fn with_word<R>(&self, address: usize, visit: impl FnOnce(usize) -> R) -> Option<R> {
        if !self.mmap.find(address)?.allows(UserAccess::Read) {
            return None;
        }
        let base = address & !(PAGE_SIZE - 1);
        let state = self.mm.table.lock();
        let page = state.pages.find(base)?;
        let mapping = state.table.lookup(base)?;
        if mapping.physical != page.backing.physical()
            || !vma::allows(mapping.permissions, UserAccess::Read)
        {
            return None;
        }
        Some(visit(
            arch::phys_to_virt(mapping.physical) + (address - base),
        ))
    }
}

impl Mm {
    /// 在普通可睡眠任务上下文中创建一个全新的空用户地址空间（含独立的根页表，尚未绑定到任何线程或 CPU）。
    pub fn try_new() -> Result<Self, PageTableError> {
        Ok(Self {
            table: Arc::try_new(PteDomain {
                state: SpinLock::new(PteState {
                    table: UserPageTable::try_new()?,
                    pages: ResidentPages::default(),
                }),
            })
            .map_err(|_| PageTableError::NoMemory)?,
            mmap: SleepRwLock::new(MmapState::default()),
            users: AtomicUsize::new(0),
            executable: None,
            interpreter: None,
            #[cfg(feature = "ktest")]
            mkwrite_hook: SpinLock::new(None),
            #[cfg(feature = "ktest")]
            fault_hook: SpinLock::new(None),
            #[cfg(feature = "ktest")]
            fork_hook: SpinLock::new(None),
            #[cfg(feature = "ktest")]
            vma_unmap_hook: SpinLock::new(None),
            #[cfg(feature = "ktest")]
            mkwrites: AtomicUsize::new(0),
        })
    }

    /// 为尚未发布的地址空间绑定主可执行文件及其执行期写互斥锁（`ExecAccess`）；当 `Mm` 最终析构时自动释放写互斥。
    pub(crate) fn bind_executable(&mut self, file: FileRef, access: ExecAccess) {
        assert!(self.executable.is_none(), "MM executable already bound");
        self.executable = Some((file, access));
    }

    /// 为尚未发布的地址空间绑定动态链接器（ELF 解释器）文件及其执行期写互斥锁（`ExecAccess`）。
    pub(crate) fn bind_interpreter(&mut self, file: FileRef, access: ExecAccess) {
        assert!(self.interpreter.is_none(), "MM interpreter already bound");
        self.interpreter = Some((file, access));
    }

    /// 返回当前绑定到该地址空间的活动用户线程数量（即存活的 [`MmUser`] 数量）。
    pub fn user_count(&self) -> usize {
        self.users.load(Ordering::Acquire)
    }

    /// 获取 `mmap` 睡眠读锁并构造 [`FutexRead`] 会话，用于在解析 `FutexKey` 与校验用户字期间阻止并发重映射。
    pub(crate) fn futex_read(self: &Arc<Self>) -> FutexRead<'_> {
        FutexRead {
            mm: self,
            mmap: self.mmap.read(),
        }
    }

    /// 测试辅助函数：在同一个共享匿名对象（`SharedAnon`）上为 `[source, source + length)` 建立映射到 `destination` 的额外共享别名 VMA。
    #[cfg(feature = "ktest")]
    pub fn test_shared_alias(
        &self,
        source: usize,
        destination: usize,
        length: usize,
    ) -> Result<(), VmError> {
        let source_range = vma::checked_range(source, length)?;
        let destination_range = vma::checked_range(destination, length)?;
        let mut mmap = self.mmap.write();
        let source_vma = mmap.find(source).ok_or(VmError::Unmapped)?;
        if source_vma.range.end < source_range.end {
            return Err(VmError::InvalidRange);
        }
        let VmaBacking::SharedAnonymous { owner, offset } = &source_vma.backing else {
            return Err(VmError::Permission);
        };
        let alias = Vma::try_new(
            destination_range,
            source_vma.permissions,
            source_vma.maximum,
            VmaBacking::SharedAnonymous {
                owner: Arc::clone(owner),
                offset: offset
                    .checked_add((source - source_vma.range.start) as u64)
                    .ok_or(VmError::InvalidRange)?,
            },
            None,
        )?;
        let retired = self.insert_vma(&mut mmap, alias)?;
        drop(mmap);
        drop(retired);
        Ok(())
    }

    /// 在尚未共享给其他线程或 CPU 的地址空间中直接映射一页已初始化完毕的独占物理页，并将物理页释放责任移交给 `Mm`。
    ///
    /// # Safety
    /// 1. 调用方必须已完整初始化 `pages`（阶数为 0 的单个物理页）的全部字节，且已结束对该页的所有可变 Rust 引用与外部裸指针访问；
    /// 2. 该物理页只能存放用户态代码或数据，不得包含任何内核私有数据结构；调用成功后所有权归 `Mm` 管理，调用方不得再手动释放该物理页。
    pub unsafe fn map_owned(
        &mut self,
        address: usize,
        pages: OwnedPages,
        permissions: PagePermissions,
        maximum: PagePermissions,
    ) -> Result<(), PageTableError> {
        if pages.order() != 0 {
            return Err(PageTableError::InvalidRange);
        }
        let range = vma::checked_range(address, PAGE_SIZE).map_err(table_error)?;
        let vma = Vma::try_new(range, permissions, maximum, VmaBacking::Anonymous, None)
            .map_err(table_error)?;

        // SAFETY:
        // 1. 前置检查已确认 `pages.order() == 0`，且调用方契约保证该物理页已完整初始化、无任何并发映射、DMA 或外部借用；
        // 2. 此处将独占 `OwnedPages` 封装为引用计数的 `AnonPageRef`，后续生命周期完全由本 `Mm` 的 `ResidentPages` 管理。
        let backing = unsafe { AnonPageRef::from_owned(pages) };
        let page = ResidentPage::try_new(address, ResidentBacking::Anonymous(backing), false)
            .map_err(table_error)?;
        let needed = self.table.lock().table.needed_tables(address);
        let mut prepared = PreparedTables::try_new(needed)?;

        let mut mmap = self.mmap.write();
        let retired = self.insert_vma(&mut mmap, vma).map_err(table_error)?;
        {
            let mut state = self.table.lock();
            // SAFETY:
            // 1. `&mut self` 排他借用保证了当前地址空间尚未在任何 CPU 上激活或被并发访问，`page` 已完整初始化并持有物理页强引用；
            // 2. 所需的中间页表 `prepared` 已在锁外预分配完毕，硬件叶子 PTE 与 `ResidentPage` 节点在同一个页表自旋锁临界区内原子安装。
            let previous = unsafe {
                state
                    .table
                    .map(address, page.backing.physical(), permissions, &mut prepared)
            }?;
            assert!(previous.is_none(), "owned page replaced an existing leaf");
            state.pages.push(page);
        }

        drop(mmap);
        drop(retired);
        drop(prepared);
        arch::sync_user_icache();
        Ok(())
    }

    /// 在地址空间中注册一段私有匿名映射区间 `[address, address + length)`（按需分页，首次访问触发缺页时才分配零初始化物理页）。
    pub fn map_anonymous(
        &self,
        address: usize,
        length: usize,
        permissions: PagePermissions,
        maximum: PagePermissions,
    ) -> Result<(), VmError> {
        let range = vma::checked_range(address, length)?;
        let vma = Vma::try_new(range, permissions, maximum, VmaBacking::Anonymous, None)?;
        let mut mmap = self.mmap.write();
        let retired = self.insert_vma(&mut mmap, vma)?;
        drop(mmap);
        drop(retired);
        Ok(())
    }

    /// 在地址空间中注册一段允许向下自动扩展的用户栈区间 `[address, address + length)`。
    ///
    /// 仅注册初始栈区间而非整个 `RLIMIT_STACK` 软限额；`execve` 构造初始栈帧的页面以及运行期压栈触发的缺页均通过常规 `fault` 按需分配物理页。
    pub fn map_stack(
        &self,
        address: usize,
        length: usize,
        permissions: PagePermissions,
        maximum: PagePermissions,
    ) -> Result<(), VmError> {
        let range = vma::checked_range(address, length)?;
        let vma = Vma::try_stack(range, permissions, maximum)?;
        let mut mmap = self.mmap.write();
        let retired = self.insert_vma(&mut mmap, vma)?;
        drop(mmap);
        drop(retired);
        Ok(())
    }

    /// 在地址空间中建立文件映射区间 `[address, address + length)`。
    ///
    /// `Vma` 直接持有克隆的 [`FileRef`] 与 [`InodeRef`]，因此后续关闭文件描述符不会撤销该映射；文件读写模式校验与反向映射注册均在获取页表锁之外完成。
    #[expect(
        clippy::too_many_arguments,
        reason = "文件映射分别携带地址范围、当前和最大权限、打开实例及共享属性，保留 mmap 的原接口和拥有边界"
    )]
    pub fn map_file(
        &self,
        address: usize,
        length: usize,
        permissions: PagePermissions,
        maximum: PagePermissions,
        file: FileRef,
        offset: u64,
        private: bool,
    ) -> Result<(), VmError> {
        let range = vma::checked_range(address, length)?;
        let inode = file.mapping_inode().ok_or(VmError::NotMappable)?.clone();
        check_file(&file, &inode, offset, length, private, permissions, maximum)?;
        let vma = Vma::try_new(
            range,
            permissions,
            maximum,
            VmaBacking::File {
                file,
                inode,
                offset,
                private,
            },
            Some(&self.table),
        )?;
        let mut mmap = self.mmap.write();
        let retired = self.insert_vma(&mut mmap, vma)?;
        drop(mmap);
        drop(retired);
        Ok(())
    }

    /// 将虚拟地址区间 `range` 内所有共享文件映射（`MAP_SHARED`）的脏页同步回写到底层文件（`msync` 系统调用实现）。
    ///
    /// - 逐段获取 `mmap` 读锁定位相交的 `Vma` 并克隆 [`FileRef`]，随后释放 `mmap` 读锁再执行耗时的文件范围同步，避免阻塞同进程缺页；
    /// - 若区间内包含未映射空洞，仍会继续同步后续已映射子区间并最终返回 [`VmError::Unmapped`]（若发生底层文件系统 I/O 错误则优先返回文件错误）；
    /// - 私有映射与匿名映射不回写底层文件；当 `synchronous` 为 `false`（`MS_ASYNC`）时仅校验映射范围存在性。
    pub fn sync_mapping_range(
        &self,
        range: Range<usize>,
        synchronous: bool,
        _invalidate: bool,
    ) -> Result<(), VmError> {
        if range.end < range.start {
            return Err(VmError::InvalidRange);
        }

        let mut cursor = range.start;
        let mut unmapped = false;
        while cursor < range.end {
            let (end, sync) = {
                let mmap = self.mmap.read();
                let position = mmap.vmas.partition_point(|vma| vma.range.end <= cursor);
                let Some(vma) = mmap.vmas.get(position) else {
                    return Err(VmError::Unmapped);
                };
                if cursor < vma.range.start {
                    unmapped = true;
                    cursor = vma.range.start;
                    if range.end <= cursor {
                        return Err(VmError::Unmapped);
                    }
                }
                let end = range.end.min(vma.range.end);
                let sync = match &vma.backing {
                    VmaBacking::File {
                        file,
                        offset,
                        private: false,
                        ..
                    } if synchronous => {
                        let start = offset
                            .checked_add((cursor - vma.range.start) as u64)
                            .ok_or(VmError::InvalidRange)?;
                        let file_end = start
                            .checked_add((end - cursor) as u64)
                            .ok_or(VmError::InvalidRange)?;
                        Some((Arc::clone(file), start..file_end))
                    }
                    _ => None,
                };
                (end, sync)
            };
            if let Some((file, file_range)) = sync {
                file.sync_range(file_range, true).map_err(VmError::File)?;
            }
            cursor = end;
        }
        if unmapped {
            Err(VmError::Unmapped)
        } else {
            Ok(())
        }
    }

    /// 执行 `mmap` 系统调用核心逻辑：向上按页取整请求长度、校验文件后备权限，并在 `mmap` 写锁下完成选址与 VMA 发布。
    ///
    /// 若为 `MAP_FIXED` 覆盖映射，会先在锁外准备好拆分/替换后的完整候选 VMA 列表，再摘除被覆盖区间的旧硬件页表项并刷新全核 TLB。
    pub(crate) fn mmap(&self, request: MmapRequest) -> Result<usize, VmError> {
        if request.length == 0 {
            return Err(VmError::InvalidRange);
        }
        let length = page_end(request.length)?;
        if let MmapSource::File {
            file,
            inode,
            offset,
            private,
        } = &request.source
        {
            check_file(
                file,
                inode,
                *offset,
                length,
                *private,
                request.permissions,
                request.maximum,
            )?;
        }

        let mut mmap = self.mmap.write();
        let (address, replace) = match request.address {
            MmapAddress::Hint(hint) => (mmap.choose_address(hint, length)?, false),
            MmapAddress::Fixed(address) => (address, true),
            MmapAddress::FixedNoReplace(address) => (address, false),
        };
        let range = vma::checked_range(address, length)?;
        if !replace && !mmap.vacant(&range) {
            return Err(VmError::Occupied);
        }

        let backing = match request.source {
            MmapSource::PrivateAnonymous => VmaBacking::Anonymous,
            MmapSource::SharedAnonymous => VmaBacking::SharedAnonymous {
                owner: SharedAnon::try_new()?,
                offset: 0,
            },
            MmapSource::File {
                file,
                inode,
                offset,
                private,
            } => VmaBacking::File {
                file,
                inode,
                offset,
                private,
            },
        };
        let vma = Vma::try_new(
            range.clone(),
            request.permissions,
            request.maximum,
            backing,
            Some(&self.table),
        )?;
        if replace {
            let retired = self.replace_range(&mut mmap, &range, Some(vma))?;
            drop(mmap);
            drop(retired);
        } else {
            let retired = self.insert_vma(&mut mmap, vma)?;
            drop(mmap);
            drop(retired);
        }
        Ok(address)
    }

    /// 执行 `brk` 系统调用：在持有 `mmap` 写锁时调整用户堆顶（Program Break）字节边界，并按需映射新匿名页区间或卸载收缩区间。
    ///
    /// 堆向上增长时必须避开紧邻用户栈下方的防护间隙（[`STACK_GUARD_GAP`]）；任何参数越界、冲突或内存分配失败均保持原 `heap_end` 不变并返回旧值。
    pub(crate) fn brk(&self, request: usize) -> usize {
        let mut mmap = self.mmap.write();
        let old = mmap.heap_end;
        if request == 0 || request < mmap.heap_start || arch::USER_ADDRESS_END < request {
            return old;
        }
        let Ok(end) = page_end(request) else {
            return old;
        };
        let Ok(previous) = page_end(old) else {
            return old;
        };

        let mut retired = None;
        let mut inserted = None;
        match previous.cmp(&end) {
            CmpOrdering::Less => {
                let range = previous..end;
                if !mmap.vacant_with_guard(&range) {
                    return old;
                }
                let permissions = PagePermissions {
                    read: true,
                    write: true,
                    execute: false,
                };
                let maximum = PagePermissions {
                    read: true,
                    write: true,
                    execute: true,
                };
                let candidate =
                    Vma::try_new(range, permissions, maximum, VmaBacking::Anonymous, None);
                match candidate.and_then(|vma| self.insert_vma(&mut mmap, vma)) {
                    Ok(old_vmas) => inserted = Some(old_vmas),
                    Err(_) => return old,
                }
            }
            CmpOrdering::Greater => match self.replace_range(&mut mmap, &(end..previous), None) {
                Ok(discarded) => retired = Some(discarded),
                Err(_) => return old,
            },
            CmpOrdering::Equal => {}
        }

        mmap.heap_end = request;
        drop(mmap);
        drop(inserted);
        drop(retired);
        request
    }

    /// 处理用户态虚拟地址 `address` 上发生的 `access` 类型缺页异常（Page Fault）。
    ///
    /// 涵盖按需分配匿名页、读取文件页缓存、写时复制（COW）、共享文件首次写脏准备（`mkwrite`）以及用户栈自动向下扩展。
    /// 物理页分配、磁盘读取与中间页表预分配均在页表自旋锁之外执行；提交前重新核实 VMA 身份与观测到的物理页是否发生并发变化。
    pub fn fault(&self, address: usize, access: UserAccess) -> Result<(), VmError> {
        let base = address & !(PAGE_SIZE - 1);
        let mut attempt = ReclaimAttempt::new();
        loop {
            // Step 1: 在读锁下采样缺页位置的 VMA 与当前驻留页状态；若地址未映射则尝试作为用户栈向下扩展。
            let source = match self.fault_source(address, access) {
                Ok(Some(source)) => source,
                Ok(None) => return Ok(()),
                Err(VmError::Unmapped) => {
                    let limit = self.current_stack_limit().ok_or(VmError::Unmapped)?;
                    self.grow_stack(address, access, limit)?;
                    continue;
                }
                Err(error) => return Err(error),
            };

            // Step 2: 若是共享文件映射（MAP_SHARED）的写入缺页，交由专门的 mkwrite 路径完成文件脏页准备与写权限授予。
            if access == UserAccess::Write && source.vma.is_shared_file() {
                if self.mkwrite(&source, address, &mut attempt)? {
                    return Ok(());
                }
                continue;
            }

            // Step 3: 在不持有 mmap 锁和页表自旋锁的情况下，分配后备物理页、拷贝 COW 数据并预分配中间级页表。
            let preparation = prepare_fault(&source, base, access, &mut attempt);

            // Step 4: 获取 mmap 写锁并核实当前地址处的 VMA 身份与访问权限在锁外准备期间未被并发修改。
            let mmap = self.mmap.write();
            let current = mmap.find(address).ok_or(VmError::Unmapped)?;
            if !current.allows(access) {
                return Err(VmError::Permission);
            }
            if !Arc::ptr_eq(current, &source.vma) {
                drop(mmap);
                drop(preparation);
                continue;
            }
            let (page, mut prepared, file) = match preparation {
                Ok(prepared) => prepared,
                Err(error) => {
                    {
                        let state = self.table.lock();
                        if state.allows(base, access) {
                            return Ok(());
                        }
                        if state
                            .pages
                            .find(base)
                            .map(|resident| resident.backing.physical())
                            != source.observed
                        {
                            continue;
                        }
                    }
                    if !matches!(&source.backing, Some(ResidentBacking::Anonymous(_))) {
                        if let Some(range) = &source.vma.file_range {
                            let data = range.data();
                            let offset = data
                                .offset
                                .checked_add((base - data.range.start) as u64)
                                .ok_or(VmError::InvalidRange)?;
                            if data.inode.metadata_snapshot().map_err(VmError::File)?.size <= offset
                            {
                                return Err(VmError::BeyondEof);
                            }
                        }
                    }
                    return Err(error);
                }
            };

            // Step 5: 若涉及文件页缓存，锁定文件页内容并校验该页仍在页缓存中且文件偏移未因并发 truncate 越过 EOF。
            let content = file.as_ref().map(|folio| folio.lock_content());
            if let Some(content) = &content {
                let data = source
                    .vma
                    .file_range
                    .as_ref()
                    .expect("file fault lacks source")
                    .data();
                let offset = data
                    .offset
                    .checked_add((base - data.range.start) as u64)
                    .ok_or(VmError::InvalidRange)?;
                if !data
                    .inode
                    .mapping()
                    .is_canonical(offset / PAGE_SIZE as u64, content.folio())
                {
                    continue;
                }
                if data.inode.metadata_snapshot().map_err(VmError::File)?.size <= offset {
                    return Err(VmError::BeyondEof);
                }
                content
                    .range_ready(0, PAGE_SIZE)
                    .map_err(|error| VmError::File(error.into()))?;
                #[cfg(feature = "ktest")]
                {
                    let hook = self.fault_hook.lock().clone();
                    if let Some(hook) = hook {
                        hook(base);
                    }
                }
            }

            // Step 6: 若该虚拟页原本存在旧的只读/COW 叶子 PTE，先在文件范围锁与页表自旋锁下清除旧叶子 PTE，并在锁外完成全核 TLB 刷新。
            {
                let _ranges = source
                    .vma
                    .file_range
                    .as_ref()
                    .map(|range| range.data().inode.mapping().lock_ranges());
                let mut state = self.table.lock();
                let resident = state.pages.find(base);
                if state.allows(base, access) {
                    return Ok(());
                }
                if resident.map(|page| page.backing.physical()) != source.observed
                    || source.needed < state.table.needed_tables(base)
                {
                    continue;
                }
                if source.observed.is_some() {
                    // SAFETY:
                    // 1. 当前同时持有文件范围锁（若有）与 `self.table` 页表自旋锁，且已核实当前驻留页物理地址仍为 `source.observed`；
                    // 2. `clear_leaf` 仅清除硬件叶子 PTE 而不释放中间页表或旧物理页（旧物理页仍由 `state.pages` 和 `source.backing` 保活），随后立即在锁外执行全核 TLB 同步。
                    let old = unsafe { state.table.clear_leaf(base) };
                    assert_eq!(old.map(|leaf| leaf.physical), source.observed);
                }
            }

            sync_all();

            // Step 7: 重新获取文件范围锁与页表自旋锁，确认期间无并发修改后安装新叶子 PTE 并替换 ResidentPages 节点。
            let old = {
                let _ranges = source
                    .vma
                    .file_range
                    .as_ref()
                    .map(|range| range.data().inode.mapping().lock_ranges());
                let mut state = self.table.lock();
                if state
                    .pages
                    .find(base)
                    .map(|resident| resident.backing.physical())
                    != source.observed
                    || source.needed < state.table.needed_tables(base)
                {
                    continue;
                }
                assert!(
                    state.table.lookup(base).is_none(),
                    "fault publication found a live leaf"
                );
                let permissions = page.permissions(current.permissions);
                // SAFETY:
                // 1. 旧叶子 PTE（若存在）已在 Step 6 中清除且完成了全核 TLB 刷新；当前持文件范围锁与页表自旋锁再次确认驻留页与所需中间页表数未变，且硬件叶子槽位为空；
                // 2. 新物理页已完成数据初始化，文件页内容锁阻止了并发截断卸载，预分配的中间页表 `prepared` 充足，且被替换的旧驻留节点 `old` 移出到锁外释放。
                let previous = unsafe {
                    state
                        .table
                        .map(base, page.backing.physical(), permissions, &mut prepared)
                }
                .expect("prepared user fault mapping became invalid");
                assert!(previous.is_none(), "fault publication replaced a live leaf");
                let old = state.pages.remove(base);
                state.pages.push(page);
                old
            };

            sync_all();
            if let Some(folio) = &file {
                folio.mark_accessed();
            }

            drop(content);
            drop(mmap);
            drop(old);
            drop(prepared);
            return Ok(());
        }
    }

    /// 查询当前正在执行的普通用户线程的栈软限额（`RLIMIT_STACK`）。
    ///
    /// 仅当调用方是绑定到本 `Mm` 且当前 CPU 正在使用本 `Mm` 的活动用户线程时返回限额；内核离线操作或跨地址空间访问不会触发栈自动增长。
    fn current_stack_limit(&self) -> Option<u64> {
        task::current_task_id()?;
        let task = task::current_task();
        let user = task.user_thread()?;
        let active = current_mm()?;
        if !core::ptr::eq(Arc::as_ptr(&active), self) {
            return None;
        }
        let bound = user.mm()?;
        if !Arc::ptr_eq(&active, &bound) {
            return None;
        }
        let identity = user.identity()?;
        Some(identity.group().resource_limit(Resource::Stack).current)
    }

    /// 在持有 `mmap` 写锁下重新核实未映射缺页地址 `address`，并将上方紧邻的用户栈 VMA 向下扩展以覆盖该页。
    ///
    /// 本函数仅发布扩展后的新栈 VMA，不直接分配物理页；返回后由 [`Mm::fault`] 下一轮循环按常规匿名页缺页流程分配并映射物理页。
    fn grow_stack(&self, address: usize, access: UserAccess, limit: u64) -> Result<(), VmError> {
        let mut mmap = self.mmap.write();
        if mmap.find(address).is_some() {
            return Ok(());
        }
        let stack = mmap.stack_growth(address, access, limit)?;
        let range = stack.range.clone();
        let vmas = mmap.replacement(&range, Some(stack))?;
        let retired = self.publish_vmas(&mut mmap, vmas, None)?;
        drop(mmap);
        drop(retired);
        Ok(())
    }

    /// 为共享文件映射（`MAP_SHARED`）的写入缺页准备可写页缓存，并将硬件叶子 PTE 升级为可写状态。
    ///
    /// 先在锁外调用文件系统 `prepare_mkwrite` 分配或锁定目标页缓存并等待可能正在进行的回写（Writeback）；
    /// 随后严格按照 `mmap 写锁 -> inode 截断锁 -> 文件页内容锁 -> 页表自旋锁` 的顺序二次核实映射未被卸载或截断，将页缓存标记为脏页（Dirty）并授予硬件写权限。
    /// 若锁外准备期间映射来源或驻留页发生变化，返回 `Ok(false)` 通知外层 `fault` 循环重试。
    fn mkwrite(
        &self,
        source: &FaultSource,
        address: usize,
        attempt: &mut ReclaimAttempt,
    ) -> Result<bool, VmError> {
        let base = address & !(PAGE_SIZE - 1);
        let data = source
            .vma
            .file_range
            .as_ref()
            .expect("shared file has no range")
            .data();
        let offset = data
            .offset
            .checked_add((base - data.range.start) as u64)
            .ok_or(VmError::InvalidRange)?;
        let index = offset / PAGE_SIZE as u64;

        // Step 1: 在不持有地址空间锁的情况下检查 EOF、调用文件系统准备可写页缓存，并预分配驻留节点与中间页表。
        let preparation = (|| {
            if data.inode.metadata_snapshot().map_err(VmError::File)?.size <= offset {
                return Err(VmError::BeyondEof);
            }
            let prepared = data
                .inode
                .prepare_mkwrite(index, attempt)
                .map_err(VmError::File)?;
            #[cfg(feature = "ktest")]
            {
                let hook = self.mkwrite_hook.lock().clone();
                if let Some(hook) = hook {
                    hook();
                }
            }
            let page = if source.observed.is_none() {
                let mut page = ResidentPage::try_new(
                    base,
                    ResidentBacking::File(prepared.folio().clone()),
                    false,
                )?;
                page.source = Arc::downgrade(data);
                Some(page)
            } else {
                None
            };
            let tables = PreparedTables::try_new_reclaim(source.needed, attempt)?;
            Ok((prepared, page, tables))
        })();

        // Step 2: 获取 mmap 写锁与页表自旋锁，核实 VMA 身份未变且当前页未被其他线程抢先完成可写映射。
        let mmap = self.mmap.write();
        let current = mmap.find(address).ok_or(VmError::Unmapped)?;
        if !current.allows(UserAccess::Write) {
            return Err(VmError::Permission);
        }
        if !Arc::ptr_eq(current, &source.vma) {
            drop(mmap);
            drop(preparation);
            return Ok(false);
        }

        {
            let state = self.table.lock();
            let page = state.pages.find(base);
            if page.is_some_and(|page| {
                page.source.as_ptr() == Arc::as_ptr(data)
                    && state.table.lookup(base).is_some_and(|leaf| {
                        leaf.physical == page.backing.physical() && leaf.permissions.write
                    })
            }) {
                return Ok(true);
            }
            if page.map(|page| page.backing.physical()) != source.observed
                || source.needed < state.table.needed_tables(base)
            {
                return Ok(false);
            }
        }

        // Step 3: 依次获取文件截断锁与页内容锁，将页缓存正式标记为脏页，并在页表自旋锁内安装或升级可写叶子 PTE。
        let (prepared, mut page, mut tables) = preparation?;
        let invalidate = data.inode.lock_invalidate().map_err(VmError::File)?;
        let mut content = prepared.folio().lock_content();
        if !data
            .inode
            .finish_mkwrite(&prepared, &mut content)
            .map_err(VmError::File)?
        {
            return Ok(false);
        }
        {
            let mut state = self.table.lock();
            let physical = prepared.folio().phys_addr();
            if let Some(resident) = state.pages.find(base) {
                if resident.source.as_ptr() != Arc::as_ptr(data)
                    || resident.backing.physical() != physical
                    || !matches!(&resident.backing, ResidentBacking::File(_))
                {
                    return Ok(false);
                }
                // SAFETY:
                // 1. 当前持有 `invalidate` 截断锁、文件页 `content` 锁与 `self.table` 页表自旋锁，已核实驻留节点的 `source` 与物理地址 `physical` 完全匹配；
                // 2. `finish_mkwrite` 已将底层文件页标记为脏页，`protect_leaf` 仅原地为同一个物理页开启硬件写权限，不改变物理页所有权，锁外随后执行全核 TLB 同步。
                if !unsafe {
                    state
                        .table
                        .protect_leaf(base, physical, current.permissions)
                }? {
                    return Ok(false);
                }
            } else {
                if state.table.lookup(base).is_some() || source.observed.is_some() {
                    return Ok(false);
                }
                // SAFETY:
                // 1. `prepared` 对应的规范文件页缓存已完整初始化并由 `finish_mkwrite` 标记为脏页；当前持锁确认该虚拟地址尚无叶子 PTE 且中间页表 `tables` 已在锁外备齐；
                // 2. 硬件叶子 PTE 与携带有效 `source` 弱引用的 `ResidentPage` 节点在同一个页表自旋锁临界区内原子发布。
                let previous = unsafe {
                    state
                        .table
                        .map(base, physical, current.permissions, &mut tables)
                }?;
                assert!(previous.is_none());
                state
                    .pages
                    .push(page.take().expect("new mkwrite page lacks prepared owner"));
            }
            #[cfg(feature = "ktest")]
            self.mkwrites.fetch_add(1, Ordering::Relaxed);
        }

        prepared.folio().mark_accessed();
        drop(content);
        drop(invalidate);
        drop(mmap);
        sync_all();
        Ok(true)
    }

    /// 测试辅助函数：设置或清除 `mkwrite` 准备阶段的同步钩子（旧钩子的析构在自旋锁外完成）。
    #[cfg(feature = "ktest")]
    pub fn ktest_set_mkwrite_hook(&self, hook: Option<Arc<dyn Fn() + Send + Sync>>) {
        let previous = core::mem::replace(&mut *self.mkwrite_hook.lock(), hook);
        drop(previous);
    }

    /// 测试辅助函数：设置或清除 VMA 范围卸载阶段（新 VMA 已发布、旧硬件 PTE 待摘除时）的观察钩子。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_vma_unmap_hook(&self, hook: Option<VmaUnmapHook>) {
        let previous = core::mem::replace(&mut *self.vma_unmap_hook.lock(), hook);
        drop(previous);
    }

    /// 测试辅助函数：设置或清除文件缺页在持有页内容锁并核实 EOF 后的暂停钩子。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_fault_hook(&self, hook: Option<AddressHook>) {
        let previous = core::mem::replace(&mut *self.fault_hook.lock(), hook);
        drop(previous);
    }

    /// 测试辅助函数：设置或清除 `fork` 在采样父进程驻留页后的暂停钩子。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_fork_hook(&self, hook: Option<AddressHook>) {
        let previous = core::mem::replace(&mut *self.fork_hook.lock(), hook);
        drop(previous);
    }

    /// 测试辅助函数：读取本地址空间通过 `mkwrite` 成功授予共享文件页写权限的累计次数。
    #[cfg(feature = "ktest")]
    pub fn ktest_mkwrite_count(&self) -> usize {
        self.mkwrites.load(Ordering::Relaxed)
    }

    /// 在持有 `mmap` 读锁与 `PteDomain` 自旋锁下检查缺页地址状态，若硬件页表尚未就绪则采集锁外准备所需的 [`FaultSource`] 快照。
    ///
    /// 若目标页已驻留且硬件 PTE 权限已满足本次 `access`（例如多线程并发缺页已被其他线程修复），则直接更新文件页访问位并返回 `Ok(None)`。
    fn fault_source(
        &self,
        address: usize,
        access: UserAccess,
    ) -> Result<Option<FaultSource>, VmError> {
        let mmap = self.mmap.read();
        let vma = mmap.find(address).ok_or(VmError::Unmapped)?;
        if !vma.allows(access) {
            return Err(VmError::Permission);
        }
        let base = address & !(PAGE_SIZE - 1);
        let state = self.table.lock();
        let resident = state.pages.find(base);
        if let Some(page) = resident {
            if state.table.lookup(base).is_some_and(|mapping| {
                mapping.physical == page.backing.physical()
                    && vma::allows(mapping.permissions, access)
                    && !(page.cow && access == UserAccess::Write)
            }) {
                if let ResidentBacking::File(folio) = &page.backing {
                    folio.mark_accessed();
                }
                return Ok(None);
            }
        } else {
            assert!(
                state.table.lookup(base).is_none(),
                "leaf has no resident owner"
            );
        }
        Ok(Some(FaultSource {
            vma: Arc::clone(vma),
            backing: resident.map(|page| page.backing.clone()),
            observed: resident.map(|page| page.backing.physical()),
            needed: state.table.needed_tables(base),
        }))
    }

    /// 在持有 `mmap` 读锁与 `PteDomain` 自旋锁的保护下，获取虚拟地址 `address` 对应的内核直接映射地址并执行 `visit`。
    ///
    /// 若地址未映射、尚未驻留物理页、权限不满足或写入遇到待写时复制（COW）页，则返回 `Ok(None)`，由调用方释放锁后触发 [`Mm::fault`]。
    /// `visit` 闭包在自旋锁临界区内运行，**严禁在闭包内睡眠、分配内存、触发调度，或将传入的内核虚拟地址保存到闭包外继续使用**。
    pub(crate) fn with_page<R>(
        &self,
        address: usize,
        access: UserAccess,
        visit: impl FnOnce(usize) -> R,
    ) -> Result<Option<R>, VmError> {
        let mmap = self.mmap.read();
        let Some(vma) = mmap.find(address) else {
            return Ok(None);
        };
        if !vma.allows(access) {
            return Err(VmError::Permission);
        }
        let base = address & !(PAGE_SIZE - 1);
        let state = self.table.lock();
        let Some(page) = state.pages.find(base) else {
            return Ok(None);
        };
        if page.cow && access == UserAccess::Write {
            return Ok(None);
        }
        let Some(mapping) = state.table.lookup(base) else {
            return Ok(None);
        };
        if mapping.physical != page.backing.physical() || !vma::allows(mapping.permissions, access)
        {
            return Ok(None);
        }
        let direct = arch::phys_to_virt(mapping.physical) + (address - base);
        let result = visit(direct);
        if let ResidentBacking::File(folio) = &page.backing {
            folio.mark_accessed();
        }
        drop(state);
        drop(mmap);
        Ok(Some(result))
    }

    /// 为 `fork` 系统调用完整复制当前进程的用户地址空间，生成独立的子进程 [`Mm`]。
    ///
    /// 子进程继承主可执行文件与动态链接器的写互斥凭证，并为所有文件 VMA 建立独立的反向映射登记；
    /// 对所有私有映射（`MAP_PRIVATE`）的已驻留物理页，将父子双方的驻留节点均标记为写时复制（`cow = true`）并将硬件叶子 PTE 降权为只读，最后在返回前完成全核 TLB 同步。
    pub fn fork(&self) -> Result<Self, VmError> {
        let mut child = Self::try_new()?;
        child.executable = self.executable.clone();
        child.interpreter = self.interpreter.clone();
        let mmap = self.mmap.write();
        let snapshot = mmap.snapshot(&child.table)?;
        let addresses = self.resident_addresses(&(0..arch::USER_ADDRESS_END))?;
        *child.mmap.write() = snapshot;

        // Step 1: 遍历父进程当前驻留页，向子进程页表复制映射（文件页需持内容锁与范围锁核实未被并发截断）。
        for &address in &addresses {
            let vma = mmap.find(address).expect("resident page has no VMA");
            let (backing, cow, source) = {
                let state = self.table.lock();
                let Some(page) = state.pages.find(address) else {
                    continue;
                };
                (
                    page.backing.clone(),
                    page.cow || vma.is_private(),
                    page.source.as_ptr(),
                )
            };
            #[cfg(feature = "ktest")]
            {
                let hook = self.fork_hook.lock().clone();
                if let Some(hook) = hook {
                    hook(address);
                }
            }
            let mut page = ResidentPage::try_new(address, backing, cow)?;
            {
                let child_mmap = child.mmap.read();
                if let Some(range) = &child_mmap
                    .find(address)
                    .expect("child VMA missing")
                    .file_range
                {
                    page.source = Arc::downgrade(range.data());
                }
            }
            let mut needed = child.table.lock().table.needed_tables(address);
            let mut prepared = PreparedTables::try_new(needed)?;
            let file = match &page.backing {
                ResidentBacking::File(folio) => Some(folio.clone()),
                ResidentBacking::Anonymous(_) => None,
            };
            let content = file.as_ref().map(|folio| folio.lock_content());
            if let Some(content) = &content {
                let data = vma
                    .file_range
                    .as_ref()
                    .expect("fork file page lacks source")
                    .data();
                let offset = data
                    .offset
                    .checked_add((address - data.range.start) as u64)
                    .ok_or(VmError::InvalidRange)?;
                if !data
                    .inode
                    .mapping()
                    .is_canonical(offset / PAGE_SIZE as u64, content.folio())
                    || data.inode.metadata_snapshot().map_err(VmError::File)?.size <= offset
                {
                    continue;
                }
                content
                    .range_ready(0, PAGE_SIZE)
                    .map_err(|error| VmError::File(error.into()))?;
            }

            loop {
                let ranges = vma
                    .file_range
                    .as_ref()
                    .map(|range| range.data().inode.mapping().lock_ranges());
                let parent = self.table.lock();
                let physical = page.backing.physical();
                if !(parent.pages.find(address).is_some_and(|resident| {
                    resident.backing.physical() == physical && resident.source.as_ptr() == source
                }) && parent
                    .table
                    .lookup(address)
                    .is_some_and(|leaf| leaf.physical == physical))
                {
                    break;
                }
                let mut state = child.table.lock();
                let current = state.table.needed_tables(address);
                if needed < current {
                    drop(state);
                    drop(parent);
                    drop(ranges);
                    needed = current;
                    prepared = PreparedTables::try_new(needed)?;
                    continue;
                }
                // SAFETY:
                // 1. 当前同时持有文件范围锁（若有）、父进程页表自旋锁 `parent` 与子进程页表自旋锁 `state`，已核实父进程该页物理地址与来源未变；
                // 2. 子进程 `child` 尚未发布给任何线程或 CPU，物理页由父子 `ResidentPage` 共同持有强引用，私有映射页已通过 `page.permissions` 屏蔽写权限，且中间页表 `prepared` 充足。
                let old = unsafe {
                    state.table.map(
                        address,
                        physical,
                        page.permissions(vma.permissions),
                        &mut prepared,
                    )
                }?;
                assert!(old.is_none());
                state.pages.push(page);
                break;
            }
            drop(content);
            drop(prepared);
        }

        // Step 2: 将父进程所有私有映射（MAP_PRIVATE）驻留页标记为 COW 并将硬件叶子 PTE 降权为只读，随后执行全核 TLB 刷新。
        for &address in &addresses {
            let vma = mmap.find(address).expect("fork resident has no VMA");
            if !vma.is_private() {
                continue;
            }
            let mut state = self.table.lock();
            let Some(page) = state.pages.find_mut(address) else {
                continue;
            };
            page.cow = true;
            let physical = page.backing.physical();
            let permissions = page.permissions(vma.permissions);
            // SAFETY:
            // 1. 当前持有父进程 `self.table` 页表自旋锁，且 `physical` 直接取自当前驻留页节点 `page`，父子引用共同保证底层物理页存活；
            // 2. `protect_leaf` 仅原地移除同一个物理页的硬件写权限，且在子进程可运行及释放 `mmap` 写锁前会调用 `sync_all()` 刷新全核 TLB。
            unsafe { state.table.protect_leaf(address, physical, permissions) }
                .expect("fork protection is not representable");
        }

        sync_all();
        drop(mmap);
        Ok(child)
    }

    /// 修改虚拟地址区间 `[address, address + length)` 的页面访问权限（`mprotect` 系统调用实现）。
    ///
    /// 先校验区间是否全部已映射且不超过各 VMA 的最大权限 `maximum`，再发布拆分/合并后的新 VMA 列表；
    /// 随后逐页更新已驻留页面的硬件叶子 PTE 权限（保留私有页的 `cow` 只读约束与共享文件页的只读约束），并在锁外完成全核 TLB 同步。
    pub fn protect(
        &self,
        address: usize,
        length: usize,
        permissions: PagePermissions,
    ) -> Result<(), VmError> {
        let range = vma::checked_range(address, length)?;
        let mut mmap = self.mmap.write();
        mmap.check_protection(&range, permissions)?;
        let vmas = mmap.rewrite(&range, Some(permissions))?;
        let addresses = self.resident_addresses(&range)?;
        let old_vmas = self.publish_vmas(&mut mmap, vmas, None)?;

        for &address in &addresses {
            let vma = mmap.find(address).expect("protected resident has no VMA");
            let mut state = self.table.lock();
            let Some(page) = state.pages.find(address) else {
                continue;
            };
            let physical = page.backing.physical();
            let permissions = page.permissions(vma.permissions);
            // SAFETY:
            // 1. 当前持有 `self.table` 页表自旋锁，`physical` 取自当前驻留节点 `page`，物理页所有权保持不变；
            // 2. `page.permissions` 保证了 COW 页与文件缓存页不会被直接授予硬件写权限，且循环结束后会在自旋锁外调用 `sync_all()` 刷新全核 TLB。
            unsafe { state.table.protect_leaf(address, physical, permissions) }
                .expect("protection is not representable");
        }

        if !addresses.is_empty() {
            sync_all();
        }
        drop(mmap);
        drop(old_vmas);
        Ok(())
    }

    /// 解除虚拟地址区间 `[address, address + length)` 的内存映射（`munmap` 系统调用实现）。
    ///
    /// 摘除区间内的所有硬件叶子 PTE 与变空的中间页表，在完成全核 TLB 同步并释放 `mmap` 写锁后归还物理页与页表页。
    pub fn unmap(&self, address: usize, length: usize) -> Result<(), VmError> {
        let range = vma::checked_range(address, length)?;
        let mut mmap = self.mmap.write();
        let retired = self.replace_range(&mut mmap, &range, None)?;
        drop(mmap);
        drop(retired);
        Ok(())
    }

    /// 在持有 `mmap` 写锁下向空闲区间插入新 `vma`（若相邻 VMA 可合并则一并更新受影响驻留页的文件范围弱引用）。
    fn insert_vma(&self, mmap: &mut MmapState, vma: Arc<Vma>) -> Result<RetiredVmas, VmError> {
        if !mmap.vacant(&vma.range) {
            return Err(VmError::Occupied);
        }
        let range = vma.range.clone();
        let vmas = mmap.replacement(&range, Some(vma))?;
        self.publish_vmas(mmap, vmas, None)
    }

    /// 发布新的有序 VMA 列表 `vmas`，并同步更新所有非卸载驻留页的文件反向映射弱引用（`source`）。
    ///
    /// 当 VMA 因拆分、合并或权限修改而更换了 [`FileRangeData`] 实例时，必须按照 `文件范围自旋锁 -> 页表自旋锁` 的顺序
    /// 将对应驻留页的硬件 PTE 重置为只读干净态（防止旧的脏页写授权跨 VMA 身份泄漏）并切换 `page.source`；
    /// 处于即将卸载区间 `retiring` 内的驻留页保持原 `source` 不变，留给随后的 `replace_range` 摘除。
    fn publish_vmas(
        &self,
        mmap: &mut MmapState,
        vmas: Vec<Arc<Vma>>,
        retiring: Option<&Range<usize>>,
    ) -> Result<RetiredVmas, VmError> {
        // Step 1: 统计因 VMA 拆分或合并而需要更新文件范围弱引用（source）的驻留页数量，并在锁外预分配缓冲向量。
        let count = {
            let state = self.table.lock();
            let mut cursor = state.pages.head.as_deref();
            let mut count = 0;
            while let Some(page) = cursor {
                if retiring.is_some_and(|range| range.contains(&page.address)) {
                    cursor = page.next.as_deref();
                    continue;
                }
                let after = vmas.partition_point(|vma| vma.range.start <= page.address);
                let next = after
                    .checked_sub(1)
                    .and_then(|index| vmas.get(index))
                    .filter(|vma| page.address < vma.range.end)
                    .and_then(|vma| vma.file_range.as_ref());
                let pointer =
                    next.map_or_else(|| Weak::new().as_ptr(), |range| Arc::as_ptr(range.data()));
                count += usize::from(page.source.as_ptr() != pointer);
                cursor = page.next.as_deref();
            }
            count
        };

        let mut sources = Vec::new();
        sources
            .try_reserve_exact(count)
            .map_err(|_| VmError::NoMemory)?;
        {
            let state = self.table.lock();
            let mut cursor = state.pages.head.as_deref();
            while let Some(page) = cursor {
                if retiring.is_some_and(|range| range.contains(&page.address)) {
                    cursor = page.next.as_deref();
                    continue;
                }
                let after = vmas.partition_point(|vma| vma.range.start <= page.address);
                let next = after
                    .checked_sub(1)
                    .and_then(|index| vmas.get(index))
                    .filter(|vma| page.address < vma.range.end)
                    .and_then(|vma| vma.file_range.as_ref());
                let pointer =
                    next.map_or_else(|| Weak::new().as_ptr(), |range| Arc::as_ptr(range.data()));
                if page.source.as_ptr() != pointer {
                    assert!(sources.len() < sources.capacity());
                    sources.push((page.address, page.source.clone()));
                }
                cursor = page.next.as_deref();
            }
        }

        // Step 2: 替换生效的 VMA 列表，并按锁顺序逐页将更换了文件范围身份的文件页置为只读干净态、更新其 source 弱引用。
        let old = core::mem::replace(&mut mmap.vmas, vmas);
        let mut changed = false;
        for (address, saved) in &mut sources {
            let source = saved.upgrade();
            let previous = {
                let _ranges = source
                    .as_ref()
                    .map(|data| data.inode.mapping().lock_ranges());
                let mut state = self.table.lock();
                let PteState { table, pages } = &mut *state;
                let Some(page) = pages.find_mut(*address) else {
                    continue;
                };
                if page.source.as_ptr() != saved.as_ptr() {
                    continue;
                }
                let next = mmap.find(*address).and_then(|vma| vma.file_range.as_ref());
                if matches!(&page.backing, ResidentBacking::File(_)) {
                    // SAFETY:
                    // 1. 当前持有旧文件范围自旋锁（若有）与 `self.table` 页表自旋锁，已核实驻留页 `page` 的 `source` 未变且后备为文件页；
                    // 2. `mkclean` 仅原地清除硬件写权限以防写授权跨越不同的 `FileRangeData` 身份，底层物理页保持存活，且在锁外统一调用 `sync_all()` 刷新 TLB。
                    changed |= unsafe { table.mkclean(*address, page.backing.physical()) }
                        .is_some_and(|(_, changed)| changed);
                }
                let next = next.map_or_else(Weak::new, |range| Arc::downgrade(range.data()));
                core::mem::replace(&mut page.source, next)
            };
            *saved = previous;
        }
        if changed {
            sync_all();
        }
        Ok((old, sources))
    }

    /// 在持有 `mmap` 写锁下移除区间 `range` 内的旧映射（可选替换为 `replacement`），摘除区间内的硬件 PTE 并刷新全核 TLB。
    ///
    /// 被卸载的旧 VMA、驻留物理页节点与空中间页表全部通过返回值交由调用方在释放 `mmap` 写锁后析构。
    fn replace_range(
        &self,
        mmap: &mut MmapState,
        range: &Range<usize>,
        replacement: Option<Arc<Vma>>,
    ) -> Result<(RetiredVmas, ResidentPages, RetiredTables), VmError> {
        let vmas = mmap.replacement(range, replacement)?;
        let addresses = self.resident_addresses(range)?;
        let mut tables = RetiredTables::default();
        let mut discarded = ResidentPages::default();
        let old_vmas = self.publish_vmas(mmap, vmas, Some(range))?;

        #[cfg(feature = "ktest")]
        {
            let hook = self.vma_unmap_hook.lock().clone();
            if let Some(hook) = hook {
                hook(range.start, range.end);
            }
        }
        for address in addresses.iter().copied() {
            let retired = {
                let mut state = self.table.lock();
                let Some(page) = state.pages.remove(address) else {
                    continue;
                };
                // SAFETY:
                // 1. 外层持有 `mmap` 写锁阻止了新缺页提交，当前持有 `self.table` 页表自旋锁并从 `state.pages` 取出了驻留节点 `page`（已被并发文件截断移除的页已跳过）；
                // 2. `unmap` 摘下的物理页节点移入 `discarded`，变空的中间页表移入 `tables`，两者均在下方 `sync_all()` 完成全核 TLB 与页表遍历缓存刷新后才在锁外释放。
                let (old, retired) = unsafe { state.table.unmap(address) };
                assert_eq!(old.unwrap().physical, page.backing.physical());
                discarded.push(page);
                retired
            };
            tables.append(retired);
        }

        if !addresses.is_empty() {
            sync_all();
        }
        Ok((old_vmas, discarded, tables))
    }

    /// 在持有 `mmap` 写锁的前提下，收集区间 `range` 内所有当前驻留页的虚拟基址列表。
    ///
    /// `mmap` 写锁阻止了新驻留页的插入，因此首次计数的 `count` 是精确上界（期间并发的文件截断仅可能减少驻留页数量）。
    fn resident_addresses(&self, range: &Range<usize>) -> Result<Vec<usize>, VmError> {
        let count = self.table.lock().pages.count(range);
        let mut addresses = Vec::new();
        addresses
            .try_reserve_exact(count)
            .map_err(|_| VmError::NoMemory)?;
        self.table.lock().pages.addresses(range, &mut addresses);
        Ok(addresses)
    }

    /// 测试诊断接口：查询虚拟地址 `address` 当前的硬件叶子 PTE 快照（不转移物理页所有权）。
    #[cfg(feature = "ktest")]
    pub fn mapping(&self, address: usize) -> Option<Mapping> {
        self.table.lock().table.lookup(address)
    }

    /// 设置用户堆区的起始地址 `heap_start` 与初始堆顶 `heap_end`（由 ELF 加载器在初始化进程地址空间时调用）。
    pub(crate) fn set_heap(&self, start: usize, end: usize) -> Result<(), VmError> {
        if end < start || arch::USER_ADDRESS_END < end {
            return Err(VmError::InvalidRange);
        }
        let mut mmap = self.mmap.write();
        mmap.heap_start = start;
        mmap.heap_end = end;
        Ok(())
    }

    /// 返回当前地址空间根页表的物理地址（供激活页表寄存器使用，生命周期由当前 `Mm` 保证）。
    pub(crate) fn root_phys(&self) -> usize {
        self.table.lock().table.root_phys()
    }
}

/// 在锁外为缺页异常准备新的 [`ResidentPage`] 节点、预分配中间级页表 [`PreparedTables`] 以及关联的文件页缓存引用。
///
/// 即使私有文件映射发生了写时复制（转为匿名页后备），仍通过返回值保留源 [`FolioRef`]，以便在提交阶段锁定文件页内容并校验该页仍在页缓存中且未越过文件 EOF。
fn prepare_fault(
    source: &FaultSource,
    base: usize,
    access: UserAccess,
    attempt: &mut ReclaimAttempt,
) -> Result<(Box<ResidentPage>, PreparedTables, Option<FolioRef>), VmError> {
    let (backing, cow, file) = prepare_backing(source, base, access, attempt)?;
    let mut page = ResidentPage::try_new(base, backing, cow)?;
    if let Some(range) = &source.vma.file_range {
        page.source = Arc::downgrade(range.data());
    }
    let tables = PreparedTables::try_new_reclaim(source.needed, attempt)?;
    Ok((page, tables, file))
}

/// 在锁外为缺页异常分配或获取底层物理页后备（[`ResidentBacking`]），并完成写时复制（COW）或文件数据读取。
///
/// 返回元组 `(物理页后备, 是否标记为 COW 只读, 关联的文件页缓存引用)`。
fn prepare_backing(
    source: &FaultSource,
    base: usize,
    access: UserAccess,
    attempt: &mut ReclaimAttempt,
) -> Result<(ResidentBacking, bool, Option<FolioRef>), VmError> {
    if let Some(backing) = &source.backing {
        // ponytail: 私有写时复制（COW）总是分配新页并拷贝；后续若引入页独占引用证明可优化为原地复用。
        let page = AnonPageRef::try_new_reclaim(attempt).ok_or(VmError::NoMemory)?;
        let mut file = None;
        match backing {
            ResidentBacking::Anonymous(source) => {
                // SAFETY:
                // 1. `source` 克隆强引用保证了源匿名物理页已完整初始化且存活，且该页处于 COW 只读状态、无硬件写权限或可变 Rust 引用；
                // 2. `page` 是刚从物理页分配器获得并清零的全新独占物理页，尚未安装到任何页表，因此源页与目标页的物理内存区间绝不重叠且均在直接映射区有效。
                unsafe {
                    arch::copy_user_bytes(
                        page.direct_address() as *mut u8,
                        source.direct_address() as *const u8,
                        PAGE_SIZE,
                    )
                };
            }
            ResidentBacking::File(folio) => {
                // SAFETY:
                // 1. `page` 是刚分配并清零的全新独占匿名页，尚未被映射或共享，当前函数持有其唯一访问权；
                // 2. `page.direct_address()` 指向直接映射区内对齐且长度为 `PAGE_SIZE` 的有效物理页内存，构造的临时可变切片 `dst` 在 `copy_to` 返回后立即销毁，且与源 `folio` 物理页互不重叠。
                let dst = unsafe {
                    slice::from_raw_parts_mut(page.direct_address() as *mut u8, PAGE_SIZE)
                };
                folio
                    .copy_to(0, dst)
                    .map_err(|error| VmError::File(error.into()))?;
                file = Some(folio.clone());
            }
        }
        return Ok((ResidentBacking::Anonymous(page), false, file));
    }
    match &source.vma.backing {
        VmaBacking::Anonymous => Ok((
            ResidentBacking::Anonymous(
                AnonPageRef::try_new_reclaim(attempt).ok_or(VmError::NoMemory)?,
            ),
            false,
            None,
        )),
        VmaBacking::SharedAnonymous { owner, offset } => {
            let offset = offset
                .checked_add((base - source.vma.range.start) as u64)
                .ok_or(VmError::InvalidRange)?;
            Ok((
                ResidentBacking::Anonymous(owner.page(offset / PAGE_SIZE as u64, attempt)?),
                false,
                None,
            ))
        }
        VmaBacking::File {
            inode,
            offset,
            private,
            ..
        } => {
            let offset = offset
                .checked_add((base - source.vma.range.start) as u64)
                .ok_or(VmError::InvalidRange)?;
            let size = inode.metadata_snapshot().map_err(VmError::File)?.size;
            if size <= offset {
                return Err(VmError::BeyondEof);
            }
            let valid = (size - offset).min(PAGE_SIZE as u64) as usize;
            let folio = inode
                .file_page(offset / PAGE_SIZE as u64, attempt)
                .map_err(VmError::File)?;
            if (!*private || valid == PAGE_SIZE) && access != UserAccess::Write {
                folio
                    .lock_content()
                    .range_ready(0, PAGE_SIZE)
                    .map_err(|error| VmError::File(error.into()))?;
                return Ok((ResidentBacking::File(folio.clone()), *private, Some(folio)));
            }
            let page = AnonPageRef::try_new_reclaim(attempt).ok_or(VmError::NoMemory)?;
            // SAFETY:
            // 1. `page` 是刚分配并清零的全新独占匿名页，尚未安装到任何页表或交给外部设备；
            // 2. `valid` 已通过 `(size - offset).min(PAGE_SIZE as u64)` 限制为不超过 `PAGE_SIZE`，构造的临时可变切片 `dst` 仅覆盖目标页前 `valid` 个字节（尾部保持零初始化），且与源 `folio` 物理页不重叠。
            let dst = unsafe { slice::from_raw_parts_mut(page.direct_address() as *mut u8, valid) };
            folio
                .copy_to(0, dst)
                .map_err(|error| VmError::File(error.into()))?;
            Ok((ResidentBacking::Anonymous(page), false, Some(folio)))
        }
    }
}

/// 校验文件映射（`mmap`）的文件描述符访问模式、偏移对齐以及常规文件类型约束。
///
/// - 任何文件映射都要求文件描述符具备读权限（拒绝 `O_WRONLY`）；
/// - 若为共享文件映射（`!private`）且当前或最大权限包含写权限，则要求文件以读写模式（`O_RDWR`）打开且底层挂载路径允许写入；
/// - 仅允许映射常规文件（[`InodeKind::Regular`]）。
fn check_file(
    file: &FileRef,
    inode: &InodeRef,
    offset: u64,
    length: usize,
    private: bool,
    permissions: PagePermissions,
    maximum: PagePermissions,
) -> Result<(), VmError> {
    if file.flags() & O_ACCMODE == O_WRONLY {
        return Err(VmError::File(FsError::BadFd));
    }
    if offset & (PAGE_SIZE as u64 - 1) != 0 || offset.checked_add(length as u64).is_none() {
        return Err(VmError::InvalidRange);
    }
    if !private && (permissions.write || maximum.write) {
        if file.flags() & O_ACCMODE != O_RDWR {
            return Err(VmError::Permission);
        }
        file.path()
            .ok_or(VmError::NotMappable)?
            .check_writable()
            .map_err(VmError::File)?;
    }
    if inode.metadata_snapshot().map_err(VmError::File)?.kind != InodeKind::Regular {
        return Err(VmError::File(FsError::IsDirectory));
    }
    Ok(())
}

/// 将字节地址或长度 `address` 向上按 [`PAGE_SIZE`] 取整；若发生算术溢出则返回 [`VmError::InvalidRange`]。
fn page_end(address: usize) -> Result<usize, VmError> {
    address
        .checked_add(PAGE_SIZE - 1)
        .map(|end| end & !(PAGE_SIZE - 1))
        .ok_or(VmError::InvalidRange)
}

/// 将 [`VmError`] 转换为早期的 [`PageTableError`]（供 [`Mm::map_owned`] 接口使用）。
fn table_error(error: VmError) -> PageTableError {
    match error {
        VmError::NoMemory | VmError::File(FsError::NoMemory) => PageTableError::NoMemory,
        VmError::Occupied => PageTableError::Occupied,
        _ => PageTableError::InvalidRange,
    }
}

/// 跨核同步调用（IPI CSD）的回调函数：刷新当前 CPU 的用户态 TLB 翻译缓存与指令缓存（I-Cache）。
///
/// # Safety
/// 仅能作为 [`cpu::call`] 的跨核同步回调由在线 CPU 执行；不解引用入参指针，且在执行期间绝不获取任何 `Mm`、页表或文件系统锁。
unsafe fn invalidate_cpu(_: *mut ()) {
    arch::flush_user_translations();
    arch::sync_user_icache();
}

/// 向系统中所有在线 CPU 发送同步跨核调用（IPI），刷新全部 CPU 的用户态 TLB 与指令缓存。
///
/// 必须在释放页表自旋锁（`PteDomain`）和文件范围自旋锁之后、释放被卸载物理页或页表页之前调用。
pub(crate) fn sync_all() {
    for target in cpu::actual_set().iter() {
        // SAFETY:
        // 1. 在线 CPU 集合在启动后固定不变，当前处于未持有页表自旋锁的普通任务上下文，可安全发起并等待跨核 IPI 同步；
        // 2. `invalidate_cpu` 仅执行无参数的架构级 TLB / I-Cache 刷新指令，不访问空指针参数且不会重入获取任何锁。
        unsafe { cpu::call(target, invalidate_cpu, core::ptr::null_mut()) };
    }
}

/// 在调度器切换入新任务或 `execve` 激活新地址空间时，将 `mm` 安装为当前 CPU 的活动地址空间并切换硬件页表根寄存器。
///
/// 必须在本地中断关闭（IRQ-off）且前一个任务的活动 `Mm` 已通过 [`leave_current`] 卸载后调用；
/// 激活期间当前 CPU 的 `active_mm` 槽位持有一份 `Arc<Mm>` 强引用，保证硬件正在使用的根页表与驻留页不会被释放。
pub(crate) fn enter_current(mm: Option<Arc<Mm>>) {
    assert!(!arch::irq_enabled(), "MM switch requires disabled IRQ");
    let local = cpu::current();
    // SAFETY:
    // 1. 上方 `assert!` 已确认当前 CPU 本地中断已关闭，不会发生抢占或核间迁移；
    // 2. `local.active_mm` 为当前 CPU 独占的 per-CPU 槽位，仅在关中断路径中访问，不存在并发可变借用。
    let active = unsafe { &mut *local.active_mm.get() };
    assert!(active.is_none(), "previous MM is still active");
    if let Some(mm) = mm {
        // SAFETY:
        // 1. `mm` 强引用保证了用户根页表及所有已映射叶子物理页有效且存活，且根页表中已包含完整的内核高半核共享映射；
        // 2. 当前处于关中断状态且内核栈有效，切换根页表后立即将 `mm` 强引用存入 `active_mm` 槽位保持存活直到下次 `leave_current`。
        unsafe { arch::activate_user_root(mm.root_phys()) };
        arch::sync_user_icache();
        *active = Some(mm);
    }
}

/// 在调度器切出当前任务或进程退出/替换映像时，将当前 CPU 切换回内核根页表并取出原活动地址空间引用。
///
/// 必须在本地中断关闭（IRQ-off）状态下调用；返回的 `Option<Arc<Mm>>` 由调用方在开中断或释放调度锁后安全析构。
pub(crate) fn leave_current() -> Option<Arc<Mm>> {
    assert!(!arch::irq_enabled(), "MM detach requires disabled IRQ");
    let local = cpu::current();
    // SAFETY:
    // 1. 上方 `assert!` 已确认当前 CPU 本地中断已关闭，当前执行流固定在本 CPU 上；
    // 2. `local.active_mm` 仅由本 CPU 在关中断下访问，取出 `Option<Arc<Mm>>` 前已先切回内核根页表并刷新本地 TLB。
    let active = unsafe { &mut *local.active_mm.get() };
    if active.is_some() {
        arch::activate_kernel_root();
    }
    active.take()
}

/// 获取当前 CPU 正在激活使用的用户地址空间 [`Arc<Mm>`] 克隆。
///
/// 内部通过临时关闭本地中断固定当前 CPU 并克隆 `Arc<Mm>`，避免直接借用可能在上下文切换时改变的 `active_mm` 槽位。
pub(crate) fn current_mm() -> Option<Arc<Mm>> {
    let _irq = IrqSaveGuard::new();
    let local = cpu::current();
    // SAFETY:
    // 1. `_irq` 守卫已关闭当前 CPU 本地中断，防止当前任务在读取期间被抢占迁移或触发调度切换修改 `active_mm`；
    // 2. 在关中断保护段内对本 CPU 的 `local.active_mm` 执行只读引用并完成 `Arc::clone`，不会产生数据竞争。
    unsafe { (*local.active_mm.get()).clone() }
}
