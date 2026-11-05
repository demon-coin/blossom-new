//! 有序用户范围与固定 PTE 域分责；mmap 变更和文件 rmap 都在同一短锁核实际叶。

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

/// 用户映射失败保留范围、权限、实际文件和内存不足各自的原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VmError {
    NoMemory,
    NotMappable,
    InvalidRange,
    Occupied,
    Unmapped,
    Permission,
    BeyondEof,
    File(FsError),
}

impl From<PageTableError> for VmError {
    /// 页表准备失败保持真实原因，缺准备表暴露为内部提交协议错误。
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

/// 用户选址将可移动 hint、固定覆盖和固定不覆盖明确区分。
pub(crate) enum MmapAddress {
    Hint(usize),
    Fixed(usize),
    FixedNoReplace(usize),
}

/// 映射源持真实打开实例；共享匿名对象在每次独立 mmap 时创建。
pub(crate) enum MmapSource {
    PrivateAnonymous,
    SharedAnonymous,
    File {
        file: FileRef,
        inode: InodeRef,
        offset: u64,
        private: bool,
    },
}

/// 当前与最大权限由 ABI/OFD 边界决定，Mm 再守页范围和只读后备约束。
pub(crate) struct MmapRequest {
    pub(crate) address: MmapAddress,
    pub(crate) length: usize,
    pub(crate) permissions: PagePermissions,
    pub(crate) maximum: PagePermissions,
    pub(crate) source: MmapSource,
}

/// 已初始化的驻留原页；文件共享写须先完成原页脏准备，私有写复制匿名页。
#[derive(Clone)]
enum ResidentBacking {
    Anonymous(AnonPageRef),
    File(FolioRef),
}

impl ResidentBacking {
    /// 取得本引用保活的物理身份，不授予外部内容访问或再次归还权限。
    fn physical(&self) -> usize {
        match self {
            Self::Anonymous(page) => page.phys_addr(),
            Self::File(page) => page.phys_addr(),
        }
    }
}

struct ResidentPage {
    address: usize,
    backing: ResidentBacking,
    cow: bool,
    source: Weak<FileRangeData>,
    next: Option<Box<ResidentPage>>,
}

impl ResidentPage {
    /// 在 PTE 锁外准备具体驻留拥有节点，失败只释放本次未发表引用。
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

    /// 默认 COW 和文件叶保持只读，文件 W 只能由持内容锁的脏准备另行发表。
    fn permissions(&self, permissions: PagePermissions) -> PagePermissions {
        PagePermissions {
            write: permissions.write
                && !self.cow
                && !matches!(&self.backing, ResidentBacking::File(_)),
            ..permissions
        }
    }
}

/// 具体链只在短锁内移动已准备节点，回收采用迭代析构，避免用户页数转化为内核栈深度。
#[derive(Default)]
pub(crate) struct ResidentPages {
    head: Option<Box<ResidentPage>>,
}

impl ResidentPages {
    /// 查找实际 VA 的驻留责任；初版线性链不在短锁中建立额外分配索引。
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

    /// 在唯一 PTE 修改域定位可修改的 COW 标志，不借用或修改用户内容。
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

    /// 移入已准备且未链接的具体节点，不分配、不释放旧页。
    fn push(&mut self, mut page: Box<ResidentPage>) {
        assert!(page.next.is_none(), "resident page is already linked");
        page.next = self.head.take();
        self.head = Some(page);
    }

    /// 摘取真实节点并断开后继，返回拥有只能在失效完成后锁外释放。
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

    /// 只计算当前范围的实际驻留数量，供写侧在短锁外准备结果容器。
    pub(crate) fn count(&self, range: &Range<usize>) -> usize {
        let mut count = 0;
        let mut cursor = self.head.as_deref();
        while let Some(page) = cursor {
            count += usize::from(range.contains(&page.address));
            cursor = page.next.as_deref();
        }
        count
    }

    /// 向已足额预留的容器写入实际 VA，不允许在 PTE 锁中扩容。
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
    /// 硬件使用结束或同步失效完成后逐个释放原节点，不递归析构整条链。
    fn drop(&mut self) {
        while let Some(mut page) = self.head.take() {
            self.head = page.next.take();
            drop(page);
        }
    }
}

struct PteState {
    table: UserPageTable,
    pages: ResidentPages,
}

impl PteState {
    /// 当前驻留 PFN 与实际叶须同锁吻合，软件 COW 和硬件无许可均要求锁外 fault。
    fn allows(&self, address: usize, access: UserAccess) -> bool {
        self.pages.find(address).is_some_and(|page| {
            self.table.lookup(address).is_some_and(|leaf| {
                leaf.physical == page.backing.physical() && vma::allows(leaf.permissions, access)
            }) && !(page.cow && access == UserAccess::Write)
        })
    }
}

/// 固定地址的唯一 PTE 修改域；范围登记强持域，驻留页弱持范围，不形成强环。
pub(crate) struct PteDomain {
    state: SpinLock<PteState>,
}

impl PteDomain {
    /// 所有驻留拥有和硬件叶修改沿原短锁串行，锁内不分配或最后释放来源。
    fn lock(&self) -> SpinLockGuard<'_, PteState> {
        self.state.lock()
    }

    /// 内容锁持有者只从固定来源检查重叠叶，不取得 Mm 的 mmap 锁。
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
        // SAFETY: 同一 PTE 锁核实当前来源和 PFN，page 与内容 guard 保活原页；
        // 同 PFN 原子降权不释放数据或表，调用者在所有短锁外同步 CPU。
        unsafe { state.table.mkclean(address, physical) }
            .map_or((false, 0), |(dirty, changed)| (dirty, usize::from(changed)))
    }

    /// 原文件范围锁内只摘当前来源和 PFN 对应的叶，退休链交调用者锁外失效后释放。
    ///
    /// # Safety
    /// 调用者须持来源所属的原文件范围锁，并保活来源和域；
    /// 传入的原退休页/表链须在全部短锁外同步全部 CPU 后才析构或复用。
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
            // SAFETY: 原范围锁排除来源换身份和 fork 提交；本 PTE 锁核当前来源/PFN。
            // page 及下方原退休链保活旧页，表页只移入原退休链，调用者锁外 CSD 后才析构。
            let (old, tables) = unsafe { table.unmap(page.address) };
            unmapped += usize::from(old.is_some());
            let mut page = link.take().unwrap();
            *link = page.next.take();
            retired_pages.push(page);
            retired_tables.append(tables);
        }
        unmapped
    }

    /// 只撤当前文件页的同来源同 PFN 叶，私有匿名 COW 和其他文件页均保持。
    ///
    /// # Safety
    /// 调用者须保活 source/folio 并持原文件范围锁；退休链在全部短锁外完成
    /// 所有 CPU 的真实 CSD 后才析构，不能把页表摘离当作硬件已收权。
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
        // SAFETY: 同一 PTE 锁核当前来源/File backing/PFN，原 resident 和退休链保页，
        // 父表摘离只转移原拥有，调用者承诺退出所有短锁后 CSD 再结束退休拥有。
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

struct FaultSource {
    vma: Arc<Vma>,
    backing: Option<ResidentBacking>,
    observed: Option<usize>,
    needed: usize,
}

/// 缺页和 fork 停点共用的单地址测试回调。
#[cfg(feature = "ktest")]
type AddressHook = Arc<dyn Fn(usize) + Send + Sync>;

/// 范围撤叶停点使用的双整数测试回调。
#[cfg(feature = "ktest")]
type VmaUnmapHook = Arc<dyn Fn(usize, usize) + Send + Sync>;

/// 同一 Mm 的范围读写锁与 PTE 短锁分责；活动根由 CPU 强拥有到切回内核根后。
pub struct Mm {
    table: Arc<PteDomain>,
    mmap: SleepRwLock<MmapState>,
    users: AtomicUsize,
    /// 原执行文件与写排斥保留到此 Mm 最后实际拥有结束，普通私有映射不取得责任。
    executable: Option<(FileRef, ExecAccess)>,
    /// 解释器独立保留文件与执行写排斥，不替换主执行文件身份。
    interpreter: Option<(FileRef, ExecAccess)>,
    #[cfg(feature = "ktest")]
    mkwrite_hook: SpinLock<Option<Arc<dyn Fn() + Send + Sync>>>,
    #[cfg(feature = "ktest")]
    fault_hook: SpinLock<Option<AddressHook>>,
    #[cfg(feature = "ktest")]
    fork_hook: SpinLock<Option<AddressHook>>,
    #[cfg(feature = "ktest")]
    vma_unmap_hook: SpinLock<Option<VmaUnmapHook>>,
    #[cfg(feature = "ktest")]
    mkwrites: AtomicUsize,
}

type RetiredVmas = (Vec<Arc<Vma>>, Vec<(usize, Weak<FileRangeData>)>);

/// 一份真实用户线程地址空间绑定；页表、CPU 和 futex key 的 Arc 不参与计数。
pub struct MmUser {
    mm: Arc<Mm>,
}

impl MmUser {
    /// 发表前取得一份绑定责任；失败路径和正常退出均由同一 Drop 归还。
    pub fn new(mm: Arc<Mm>) -> Self {
        mm.users
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .expect("MM user count overflow");
        Self { mm }
    }

    /// 借用本绑定保活的真实地址空间，不额外增加用户绑定计数。
    pub fn mm(&self) -> &Arc<Mm> {
        &self.mm
    }
}

impl Drop for MmUser {
    /// 先归还真实绑定；Arc 的最后析构发生在原调用者持有的锁之外。
    fn drop(&mut self) {
        let previous = self.mm.users.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "MM user count underflow");
    }
}

/// futex 保真实 Mm、共享匿名或 canonical inode 身份，不把可复用 PFN 当作长期 key。
#[derive(Clone)]
pub(crate) enum FutexKey {
    Mm {
        mm: Arc<Mm>,
        address: usize,
        private: bool,
    },
    Shared {
        owner: Arc<SharedAnon>,
        offset: u64,
    },
    File {
        inode: InodeRef,
        offset: u64,
    },
}

impl FutexKey {
    /// 按真实对象及字节位置比较，包括共享对象页内偏移。
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

    /// 哈希只选择固定桶；相等性仍由完整的实际对象身份决定。
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

/// 实际来源辨别需要缺页或私有 COW 时，调用者须放开 mmap 读侧再 fault。
pub(crate) enum FutexAccessError {
    Vm(VmError),
    NeedsFault(UserAccess),
}

impl From<VmError> for FutexAccessError {
    /// 保留实际范围、权限或缺页准备失败的 VM 原因。
    fn from(error: VmError) -> Self {
        Self::Vm(error)
    }
}

/// 先取得可睡眠 mmap 读侧，再建立唯一 Waiter；之后的字访问只取得 PTE 短锁。
pub(crate) struct FutexRead<'a> {
    mm: &'a Arc<Mm>,
    mmap: ReadGuard<'a, MmapState>,
}

impl FutexRead<'_> {
    /// 从同一稳定读侧解析当前 VMA 的具体 futex 身份。
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

    /// 在 mmap→PTE 保护下使用原始地址；缺页返回 None，准备必须放开此读侧后进行。
    /// visit 只允许原始字访问及固定桶操作，不得分配、睡眠或保留可访问地址。
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
    /// 在普通任务上下文准备独立页表，尚未发表给执行者或硬件。
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

    /// 未发表 Mm 接受原执行文件及其写排斥，后续错误由 Mm 正常析构归还。
    pub(crate) fn bind_executable(&mut self, file: FileRef, access: ExecAccess) {
        assert!(self.executable.is_none(), "MM executable already bound");
        self.executable = Some((file, access));
    }

    /// 未发表 Mm 接受唯一 ELF 解释器责任，末拥有析构与主文件遵守同一寿命边界。
    pub(crate) fn bind_interpreter(&mut self, file: FileRef, access: ExecAccess) {
        assert!(self.interpreter.is_none(), "MM interpreter already bound");
        self.interpreter = Some((file, access));
    }

    /// 返回真实用户线程绑定数，不包含 CPU、诊断或同步 key 的拥有引用。
    pub fn user_count(&self) -> usize {
        self.users.load(Ordering::Acquire)
    }

    /// 在唯一 futex Waiter 建立前取得可睡眠范围读侧，排除后续比较登记期间的重映射。
    pub(crate) fn futex_read(self: &Arc<Self>) -> FutexRead<'_> {
        FutexRead {
            mm: self,
            mmap: self.mmap.read(),
        }
    }

    /// 在当前共享匿名对象上安装真实额外别名，保留源 VMA；不接受文件或私有匿名。
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

    /// 在尚未共享的地址空间安装完整初始化单页，保留当前与最大权限并转交匿名页归还责任。
    ///
    /// # Safety
    /// 调用者必须完整初始化 pages 的全部字节，结束全部排他 Rust 引用及未受控原始访问；
    /// 页面只能作为用户数据或代码，不能包含内核私有对象。成功后不得凭旧裸地址释放该页。
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

        // SAFETY: 调用者保证完整初始化的独占 order0 页，无其他映射、设备或内容借用；
        // 此处消费原归还权，后续访问仅沿本 Mm 的原始访问和硬件权限协议。
        let backing = unsafe { AnonPageRef::from_owned(pages) };
        let page = ResidentPage::try_new(address, ResidentBacking::Anonymous(backing), false)
            .map_err(table_error)?;
        let needed = self.table.lock().table.needed_tables(address);
        let mut prepared = PreparedTables::try_new(needed)?;

        let mut mmap = self.mmap.write();
        let retired = self.insert_vma(&mut mmap, vma).map_err(table_error)?;
        {
            let mut state = self.table.lock();
            // SAFETY: self 的唯一借用排除活动共享，page 完整初始化并保有原页；
            // 中间表锁外准备，PTE 和驻留拥有在同一短锁内发表，离开全部 CPU 后才归还。
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

    /// 只发表私有匿名范围，首次实际访问在锁外申请清零基本页。
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

    /// 登记初始栈实际范围并允许当前绑定线程随后向下增长；不预留整个软限额。
    /// 离线 exec 候选须先完整登记构栈所需页，首次触页仍沿普通 fault 准备原页。
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

    /// 映射持完整打开实例，属性检查与取页均不占 mmap/PTE 锁，关闭 fd 不撤销映射。
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

    /// 逐个取得当前 VMA 的真实交集与原 OFD，放 mmap 读侧后同步准确文件范围。
    /// 孔洞保留已完成进度并最终返回 Unmapped，后续真实同步错误优先；私有与匿名不写源。
    /// 当前无锁定 VMA，invalidate 不撤 PTE；非同步请求只检查范围，不启动回写。
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

    /// 页上取整用户长度并在同一写侧选址与发表；固定覆盖先备齐候选再解除旧叶。
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

    /// byte break 与页增删在单次 mmap 写侧完成；增长避开栈 guard，任何失败均保原 break。
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

    /// 按当前 VMA 与实际叶权限处理缺页、COW 或文件写准备；缺口仅允许当前绑定栈增长。
    /// 限额快照、睡眠与原页准备均在短锁外，栈身份与页分别沿原 mmap/PTE 协议发表。
    pub fn fault(&self, address: usize, access: UserAccess) -> Result<(), VmError> {
        let base = address & !(PAGE_SIZE - 1);
        let mut attempt = ReclaimAttempt::new();
        loop {
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

            if access == UserAccess::Write && source.vma.is_shared_file() {
                if self.mkwrite(&source, address, &mut attempt)? {
                    return Ok(());
                }
                continue;
            }

            let preparation = prepare_fault(&source, base, access, &mut attempt);

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
                    // SAFETY: 范围/PTE 短锁核当次来源和驻留，source 保活旧页；
                    // 只清叶保留目录，锁外 CSD 后再次检查，不能假定文件失效未摘表。
                    let old = unsafe { state.table.clear_leaf(base) };
                    assert_eq!(old.map(|leaf| leaf.physical), source.observed);
                }
            }

            sync_all();

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
                // SAFETY: 旧叶已解除且真实 CSD 完成；同范围/PTE 锁重核旧驻留及缺表数。
                // 新页完整初始化，原文件内容锁排除 detach，原叶和退休引用均仍保活。
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

    /// 只从合法普通 Task 的实际用户绑定取当前 STACK soft，离线或无关 Mm 不借用限额。
    /// 所有拥有和值在独立短段取得，返回前释放短锁，调用者随后才取得 mmap 写侧。
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

    /// 在 mmap 写侧重查真实缺口并准备完整替换索引；分配或边界失败不发表部分范围。
    /// 原驻留页及叶不变，新缺页在返回后才沿 fault 的锁外准备和短锁提交处理。
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

    /// 文件写准备退出所有 FS guard 后重新检查范围；最终遵守 mmap→invalidate→folio→PTE。
    /// coherent 原页允许在已有 WB 时再次标脏授 W；来源变化返回 false，仍重核真实准备事实。
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
                // SAFETY: 同一短锁核实当前来源/PFN/叶；invalidate 和内容 guard 保准备事实，
                // finish 已先置真实 Dirty，同 PFN 写授权不改变拥有，锁外再同步 CPU。
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
                // SAFETY: prepared 的 canonical 原页已完整初始化并先标 Dirty；真实源登记
                // 在叶可见前存在，表页锁外准备，同一短锁发表硬件叶和驻留拥有。
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

    /// 受控准备停点只保当前 Mm 的 callback，替换与最后析构发生在短锁外。
    #[cfg(feature = "ktest")]
    pub fn ktest_set_mkwrite_hook(&self, hook: Option<Arc<dyn Fn() + Send + Sync>>) {
        let previous = core::mem::replace(&mut *self.mkwrite_hook.lock(), hook);
        drop(previous);
    }

    /// 在原范围替换已发表新 VMA、旧叶仍待摘取时观察；callback 只暂停真实操作。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_vma_unmap_hook(&self, hook: Option<VmaUnmapHook>) {
        let previous = core::mem::replace(&mut *self.vma_unmap_hook.lock(), hook);
        drop(previous);
    }

    /// 在真实文件候选持源内容锁并重核 EOF 后暂停，只观察原 fault，不构造叶或页。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_fault_hook(&self, hook: Option<AddressHook>) {
        let previous = core::mem::replace(&mut *self.fault_hook.lock(), hook);
        drop(previous);
    }

    /// 在真实 fork 取得原驻留引用后暂停，候选仍须通过当前父叶和来源的最终检查。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_fork_hook(&self, hook: Option<AddressHook>) {
        let previous = core::mem::replace(&mut *self.fork_hook.lock(), hook);
        drop(previous);
    }

    /// 观察真实 Dirty 后的 W 发表次数，不把 VMA 最大权限或诊断读取计入授权。
    #[cfg(feature = "ktest")]
    pub fn ktest_mkwrite_count(&self) -> usize {
        self.mkwrites.load(Ordering::Relaxed)
    }

    /// 取得锁外准备所需的具体身份和原页引用，权限命中不分配或等待文件。
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

    /// 短页命中保持读侧及 PTE 拥有稳定；缺 VMA、缺页或 COW 返回 None，调用者放锁后 fault。
    /// visit 不得分配、睡眠、调度、构造用户内容引用或保留可继续访问的原始地址。
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

    /// 子 Mm 独立登记文件范围并保两映像写排斥，父 COW/TLB 同步完成后才返回。
    pub fn fork(&self) -> Result<Self, VmError> {
        let mut child = Self::try_new()?;
        child.executable = self.executable.clone();
        child.interpreter = self.interpreter.clone();
        let mmap = self.mmap.write();
        let snapshot = mmap.snapshot(&child.table)?;
        let addresses = self.resident_addresses(&(0..arch::USER_ADDRESS_END))?;
        *child.mmap.write() = snapshot;

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
                // SAFETY: 原文件范围锁串行全部范围撤叶与子叶提交；父 PTE 锁重核当前来源/PFN，
                // child 尚不可运行，已初始化原页引用保活，子私有叶只读，表页锁外准备。
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
            // SAFETY: 此短锁核实当次驻留 PFN，父子引用保活原页；同 PFN 只撤 W，
            // 子尚未发表，全部 CPU 的旧父翻译在子可运行前同步失效。
            unsafe { state.table.protect_leaf(address, physical, permissions) }
                .expect("fork protection is not representable");
        }

        sync_all();
        drop(mmap);
        Ok(child)
    }

    /// 按最大权限预检并发表独立范围登记，同 PFN 改权保 COW，shared file 从不直接升 W。
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
            // SAFETY: 当次驻留与实际叶在同一 PTE 锁核 PFN；旧页保持拥有，
            // shared file 许可仍 RO，改权在所有短锁外完成真实全 CPU 失效。
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

    /// 解除实际叶及空中间表，全部在线 CPU 的失效完成后才归还旧数据页和表页。
    pub fn unmap(&self, address: usize, length: usize) -> Result<(), VmError> {
        let range = vma::checked_range(address, length)?;
        let mut mmap = self.mmap.write();
        let retired = self.replace_range(&mut mmap, &range, None)?;
        drop(mmap);
        drop(retired);
        Ok(())
    }

    /// 新映射先准备完整索引，合并可能改变相邻文件来源，发表统一更新所有驻留来源。
    fn insert_vma(&self, mmap: &mut MmapState, vma: Arc<Vma>) -> Result<RetiredVmas, VmError> {
        if !mmap.vacant(&vma.range) {
            return Err(VmError::Occupied);
        }
        let range = vma.range.clone();
        let vmas = mmap.replacement(&range, Some(vma))?;
        self.publish_vmas(mmap, vmas, None)
    }

    /// mmap 写侧预备保留叶的来源清单，再以原范围锁→PTE 换身份；待摘叶保旧来源直到撤叶/CSD。
    fn publish_vmas(
        &self,
        mmap: &mut MmapState,
        vmas: Vec<Arc<Vma>>,
        retiring: Option<&Range<usize>>,
    ) -> Result<RetiredVmas, VmError> {
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
                    // SAFETY: 原范围锁排除全范围失效，PTE 短锁核驻留 PFN；来源换身份不继承 W。
                    // 原页保活，全部改权在短锁外真实 CSD 后完成，Weak 不在此锁析构。
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

    /// 写侧先备齐候选索引及退休容器，再撤叶并同步所有 CPU；旧拥有交回调用者锁外释放。
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
                // SAFETY: mmap 写侧阻止新叶提交，PTE 短锁核真实驻留；并发文件失效已移出的页跳过。
                // 原页和表页只移入各自原退休链，全部 CPU 翻译与目录遍历失效后才归还。
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

    /// mmap 写侧阻止新增驻留，以当前数量预留上界；文件范围失效可并发减少实际页数。
    fn resident_addresses(&self, range: &Range<usize>) -> Result<Vec<usize>, VmError> {
        let count = self.table.lock().pages.count(range);
        let mut addresses = Vec::new();
        addresses
            .try_reserve_exact(count)
            .map_err(|_| VmError::NoMemory)?;
        self.table.lock().pages.addresses(range, &mut addresses);
        Ok(addresses)
    }

    /// 返回非拥有的硬件叶快照，仅用于诊断；不能凭物理整数继续访问或归页。
    #[cfg(feature = "ktest")]
    pub fn mapping(&self, address: usize) -> Option<Mapping> {
        self.table.lock().table.lookup(address)
    }

    /// 记录执行映像确定的堆起点与当前 byte break，不替代实际 VMA 映射操作。
    pub(crate) fn set_heap(&self, start: usize, end: usize) -> Result<(), VmError> {
        if end < start || arch::USER_ADDRESS_END < end {
            return Err(VmError::InvalidRange);
        }
        let mut mmap = self.mmap.write();
        mmap.heap_start = start;
        mmap.heap_end = end;
        Ok(())
    }

    /// 返回由本对象保活的根物理地址，不移交页表或叶页的访问权。
    pub(crate) fn root_phys(&self) -> usize {
        self.table.lock().table.root_phys()
    }
}

/// 锁外准备驻留节点、目录及原文件来源；匿名候选仍保源 folio，提交时同内容锁核 canonical 和 EOF。
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

/// 锁外准备具体驻留拥有；共享文件尾页保 canonical folio，私有 COW 不按引用数猜独占性。
fn prepare_backing(
    source: &FaultSource,
    base: usize,
    access: UserAccess,
    attempt: &mut ReclaimAttempt,
) -> Result<(ResidentBacking, bool, Option<FolioRef>), VmError> {
    if let Some(backing) = &source.backing {
        // ponytail: 私有 COW 总复制一页；有实际映射独占证明后才考虑原页复用。
        let page = AnonPageRef::try_new_reclaim(attempt).ok_or(VmError::NoMemory)?;
        let mut file = None;
        match backing {
            ResidentBacking::Anonymous(source) => {
                // SAFETY: source 克隆保活完整初始化原页，旧 COW 叶无硬件写权限；
                // 原始汇编不建立源引用。新匿名页尚未映射或共享，独占接收复制，
                // 原页仍被 source 持有，分配器返回的新页与其物理范围不重叠。
                unsafe {
                    arch::copy_user_bytes(
                        page.direct_address() as *mut u8,
                        source.direct_address() as *const u8,
                        PAGE_SIZE,
                    )
                };
            }
            ResidentBacking::File(folio) => {
                // SAFETY: 新匿名页已完整清零，只有本地 page 引用且尚未映射或交给设备；
                // folio 保活另一张真实原页，分配器不能返回仍被持有的源页。
                // 短切片只独占匿名目标，copy_to 从已初始化源原始读取，不形成源切片。
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
            // SAFETY: 新匿名页已完整清零，只有本地 page 引用，尚无映射、设备或其他访问；
            // valid 不超过 PAGE_SIZE，folio 保活源原页，分配器返回的目标与其不重叠。
            // 仅匿名目标形成短独占切片，copy_to 原始读取有效前缀，目标尾部保留零初始化。
            let dst = unsafe { slice::from_raw_parts_mut(page.direct_address() as *mut u8, valid) };
            folio
                .copy_to(0, dst)
                .map_err(|error| VmError::File(error.into()))?;
            Ok((ResidentBacking::Anonymous(page), false, Some(folio)))
        }
    }
}

/// 文件映射先核真实 OFD 读取能力；共享最大 W 还要求 RW OFD 与完整路径写能力。
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

/// 将字节边界 checked 上取整到基本页，用户长度和 raw break 共用此纯计算。
fn page_end(address: usize) -> Result<usize, VmError> {
    address
        .checked_add(PAGE_SIZE - 1)
        .map(|end| end & !(PAGE_SIZE - 1))
        .ok_or(VmError::InvalidRange)
}

/// 尚未共享的旧单页入口保留原页表错误类别，不吞掉已经发表范围的错误。
fn table_error(error: VmError) -> PageTableError {
    match error {
        VmError::NoMemory | VmError::File(FsError::NoMemory) => PageTableError::NoMemory,
        VmError::Occupied => PageTableError::Occupied,
        _ => PageTableError::InvalidRange,
    }
}

/// CSD 只执行本核架构失效，不反取 Mm、PTE 或文件锁。
///
/// # Safety
/// 只能由在线 CPU 的同步 call 分派；参数不被读取，调用者保活旧页与表页到返回。
unsafe fn invalidate_cpu(_: *mut ()) {
    arch::flush_user_translations();
    arch::sync_user_icache();
}

/// 叶修改在 PTE/范围短锁外同步全部实际 CPU，固定 ASID0 不维护每个 Mm 的活跃集合。
pub(crate) fn sync_all() {
    for target in cpu::actual_set().iter() {
        // SAFETY: CPU 在线集合启动后固定，普通可切换任务能服务本地 IPI；
        // 未持 PTE 或其他 callback 锁，回调只做架构动作，不读取空参数或递归 call。
        unsafe { cpu::call(target, invalidate_cpu, core::ptr::null_mut()) };
    }
}

/// 在所属CPU的IRQ-off调度交接或exec提交中安装根，硬件使用期间保留CPU拥有。
pub(crate) fn enter_current(mm: Option<Arc<Mm>>) {
    assert!(!arch::irq_enabled(), "MM switch requires disabled IRQ");
    let local = cpu::current();
    // SAFETY: 仅所属 CPU 在 IRQ-off 段访问活动槽；上次任务已切回内核根并移出拥有。
    let active = unsafe { &mut *local.active_mm.get() };
    assert!(active.is_none(), "previous MM is still active");
    if let Some(mm) = mm {
        // SAFETY: mm 拥有完整根和全部已安装页，内核静态映射由架构共享；
        // IRQ 关闭且当前调度栈有效，随后把强拥有放入本 CPU 的活动槽。
        unsafe { arch::activate_user_root(mm.root_phys()) };
        arch::sync_user_icache();
        *active = Some(mm);
    }
}

/// 先安装内核根并完成本地失效，再移出旧拥有供调度器锁外释放。
pub(crate) fn leave_current() -> Option<Arc<Mm>> {
    assert!(!arch::irq_enabled(), "MM detach requires disabled IRQ");
    let local = cpu::current();
    // SAFETY: 只有所属CPU在IRQ-off交接或exec中访问槽，原用户现场已完整保存。
    let active = unsafe { &mut *local.active_mm.get() };
    if active.is_some() {
        arch::activate_kernel_root();
    }
    active.take()
}

/// 在本 CPU 不可迁移的短段取得活动 Mm 拥有，不借用会在交接中变化的槽。
pub(crate) fn current_mm() -> Option<Arc<Mm>> {
    let _irq = IrqSaveGuard::new();
    let local = cpu::current();
    // SAFETY: IRQ guard 固定本 CPU，只有本 CPU 修改槽；克隆在保护段内完成。
    unsafe { (*local.active_mm.get()).clone() }
}
