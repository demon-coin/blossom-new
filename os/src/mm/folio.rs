//! 固定原页引用、范围初始化和内容修改；内容锁与等待源随活 folio 保持原地址。

use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::marker::{PhantomData, PhantomPinned};
use core::mem::{ManuallyDrop, MaybeUninit};
use core::pin::{pin, Pin};
use core::slice;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::arch;
use crate::block::{BlockError, IoResult, SECTOR_BYTES};
use crate::cache::{MetadataCache, FILE_BLOCK_BYTES_1K, FILE_BLOCK_BYTES_2K, FILE_BLOCK_BYTES_4K};
use crate::fs::background::{self, WritebackDomain};
use crate::fs::memfs::MemoryPageCharge;
use crate::fs::InodeRef;
use crate::mm::page::{PageDesc, PageUseGuard};
use crate::mm::{alloc_pages, pages, OwnedPages, PAGE_SIZE};
use crate::sched;
use crate::wait::{WaitNode, WaitQueue, Waiter};

const SECTOR_SIZE: usize = SECTOR_BYTES as usize;
const SECTORS_PER_PAGE: usize = PAGE_SIZE / SECTOR_SIZE;

const CONTENT_LOCKED: u8 = 1;
const REFERENCED: u8 = 2;
const ACTIVATE: u8 = 4;

/// 固定 4KiB 基本页内的 byte 窗口，验证后的起点和长度均可无损保存为 u16。
#[derive(Clone, Copy)]
struct InitializedRange {
    offset: u16,
    len: u16,
}

/// 文件页只初始化一次任意合法范围；RAM 文件额度随内容槽持有到最后原页引用。
enum FolioContents {
    File {
        initialized: Option<Result<InitializedRange, BlockError>>,
        memory_charge: Option<MemoryPageCharge>,
    },
    Metadata {
        valid: u8,
        errors: [Option<BlockError>; SECTORS_PER_PAGE],
    },
}

/// 共享头只经所属 PageUse 锁访问；Dirty 与 WB 各自描述实际 512 字节范围。
struct FolioShared {
    references: usize,
    contents: FolioContents,
    dirty: u8,
    writeback: u8,
    /// 已发表且完成初始化的文件 FS 块；只记准备事实，不保存第二份物理块根。
    file_prepared: u8,
}

/// 页用途内保存原拥有者、独立内容锁和等待源；活引用期间禁止移动整个用途。
pub(crate) struct FolioState {
    backing: Option<OwnedPages>,
    shared: UnsafeCell<FolioShared>,
    flags: AtomicU8,
    waiters: WaitQueue,
    _pinned: PhantomPinned,
}

// SAFETY: backing 在活引用期间不可变；shared 仅通过所属 PageUse guard 访问，
// 内容锁用原子协议转交，等待源内部短锁保护节点。活用途只做内部可变访问，
// refs 归零且读入者、等待者和内容借用均结束后才取走原页，固定状态原地析构。
unsafe impl Sync for FolioState {}

/// 外部页句柄保活实际 canonical 内容域；内部索引和初始读入引用不附此拥有者。
#[derive(Clone)]
enum FolioOwner {
    File(InodeRef),
    Metadata(Arc<MetadataCache>),
}

/// 永久描述符上的一份原页引用；外部取得还保活实际 inode 或元数据缓存。
/// 索引内部的引用不附拥有者，避免 index 到自身内容域的强拥有环。
pub struct FolioRef {
    descriptor: Pin<&'static PageDesc>,
    owner: Option<FolioOwner>,
}

/// 唯一内容锁责任及一份保页引用；未完成时析构以 QueueStopped 结束填充。
/// DMA 层交设备后必须保护整个拥有者，直到实际收权才能完成或析构。
pub struct FolioRead {
    folio: FolioRef,
    offset: usize,
    len: usize,
    completed: bool,
}

/// 一次真实回写范围及原内容域的拥有；设备全尾结束前不得结束本轮 WB。
/// 未交设备的准备者退出时恢复 Dirty，不把取消或失败视为持久化成功。
pub struct FolioWriteback {
    folio: FolioRef,
    offset: u16,
    len: u16,
    completed: bool,
    original_dirty: u8,
    /// 实际提交者附卷统计拥有，元数据完成无需在 IRQ 升级卷的弱引用。
    writeback_domain: Option<Arc<WritebackDomain>>,
    background: bool,
}

/// 普通 Task 借用的内容锁；文件页仅可原始复制，元数据页才可借用初始化范围。
/// 锁不排斥用户共享 store；不得持本 guard 做可能 fault 的用户复制。
pub struct FolioContent<'a> {
    folio: &'a FolioRef,
    locked: bool,
    _task_local: PhantomData<*mut ()>,
}

impl FolioState {
    /// 借用所属用途锁保护的共享头，借用不能超出 guard 的存活期。
    fn shared<'a>(usage: &'a PageUseGuard<'_>) -> &'a FolioShared {
        let state = usage.folio();
        // SAFETY: usage 是本状态唯一所属用途锁的活 guard；所有 shared 写者
        // 都持同锁，返回借用绑定 guard，期间没有可变 shared 借用。
        unsafe { &*state.shared.get() }
    }

    /// 仅修改固定用途内部的共享头，不创建会排斥其他 Pin 借用的 &mut FolioState。
    fn shared_mut<'a>(usage: &'a mut PageUseGuard<'_>) -> &'a mut FolioShared {
        let state = usage.folio();
        // SAFETY: 可变借用 guard 串行本状态全部 shared 访问；只从 UnsafeCell
        // 创建内部可变借用，不改变固定状态、原拥有者、内容锁或等待源的地址。
        unsafe { &mut *state.shared.get() }
    }

    /// 只检查当前范围事实；调用者负责持内容锁或拒绝仍被锁定的状态。
    fn range_ready(shared: &FolioShared, offset: usize, len: usize) -> Result<(), BlockError> {
        match &shared.contents {
            FolioContents::File { initialized, .. } => {
                let range = initialized.ok_or(BlockError::NotReady)??;
                let start = usize::from(range.offset);
                let end = start + usize::from(range.len);
                if offset < start || end < offset + len {
                    return Err(BlockError::InvalidRange);
                }
            }
            FolioContents::Metadata { valid, errors } => {
                let sectors = sector_mask(offset, len);
                for error in &errors[offset / SECTOR_SIZE..=(offset + len - 1) / SECTOR_SIZE] {
                    if let Some(error) = *error {
                        return Err(error);
                    }
                }
                if valid & sectors != sectors {
                    return Err(BlockError::NotReady);
                }
            }
        }
        Ok(())
    }

    /// 最后引用结束后取走原页归还权，固定等待源仍由用途槽原地析构。
    pub(crate) fn take_backing(&mut self) -> OwnedPages {
        assert_eq!(
            self.shared.get_mut().references,
            0,
            "removed folio retains references"
        );
        assert!(
            self.flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0,
            "removed folio retains a content owner"
        );
        let shared = self.shared.get_mut();
        assert_eq!(shared.dirty, 0, "removed folio retains dirty contents");
        assert_eq!(shared.writeback, 0, "removed folio retains writeback");
        self.backing.take().expect("folio backing already removed")
    }
}

impl FolioRef {
    /// 为外部文件页句柄附上实际 inode 引用，内部索引不得保存此句柄。
    /// 重复附同一对象保留原拥有者，不同对象或内容角色暴露为协议错误。
    pub(crate) fn with_file_owner(mut self, owner: InodeRef) -> Self {
        self.attach_file_owner(owner);
        self
    }

    /// 为原引用就地附来源，文件页句柄和读入凭证共用同一角色与身份检查。
    fn attach_file_owner(&mut self, owner: InodeRef) {
        {
            let usage = self.descriptor.usage();
            assert!(
                matches!(
                    &FolioState::shared(&usage).contents,
                    FolioContents::File { .. }
                ),
                "file owner attached to metadata folio"
            );
        }
        match &self.owner {
            None => self.owner = Some(FolioOwner::File(owner)),
            Some(FolioOwner::File(existing)) => {
                assert!(existing.ptr_eq(&owner), "folio file owner changed");
            }
            Some(FolioOwner::Metadata(_)) => panic!("folio owner role changed"),
        }
    }

    /// 为外部元数据页句柄附上真实 Arc 缓存，内部索引不得保存此句柄。
    /// 拥有凭证由 caller 直接移交，不从缓存 payload 地址恢复 Arc。
    pub(crate) fn with_metadata_owner(mut self, owner: Arc<MetadataCache>) -> Self {
        self.attach_metadata_owner(owner);
        self
    }

    /// 原页视图和唯一读入凭证共用真实元数据域身份，内部索引仍无 anchor。
    fn attach_metadata_owner(&mut self, owner: Arc<MetadataCache>) {
        {
            let usage = self.descriptor.usage();
            assert!(
                matches!(
                    &FolioState::shared(&usage).contents,
                    FolioContents::Metadata { .. }
                ),
                "metadata owner attached to file folio"
            );
        }
        match &self.owner {
            None => self.owner = Some(FolioOwner::Metadata(owner)),
            Some(FolioOwner::Metadata(existing)) => {
                assert!(
                    Arc::ptr_eq(existing, &owner),
                    "folio metadata owner changed"
                );
            }
            Some(FolioOwner::File(_)) => panic!("folio owner role changed"),
        }
    }

    /// 借用本外部句柄已有的元数据缓存拥有者，视图无需再保存一份等价 Arc。
    pub(crate) fn metadata_owner(&self) -> Option<&Arc<MetadataCache>> {
        match &self.owner {
            Some(FolioOwner::Metadata(owner)) => Some(owner),
            None | Some(FolioOwner::File(_)) => None,
        }
    }

    /// 消耗真实 order0 拥有者作为文件页；高阶完整返回，不拆分或改变其阶数。
    pub fn from_pages(backing: OwnedPages) -> Result<Self, OwnedPages> {
        Self::install(
            backing,
            FolioContents::File {
                initialized: None,
                memory_charge: None,
            },
        )
    }

    /// 消耗独占 order0 页作为元数据页；此角色终身禁止交给用户可写映射。
    pub fn from_metadata_pages(backing: OwnedPages) -> Result<Self, OwnedPages> {
        Self::install(
            backing,
            FolioContents::Metadata {
                valid: 0,
                errors: [None; SECTORS_PER_PAGE],
            },
        )
    }

    /// 将预留的 RAM 文件页额度移入原页内容槽；节点发表前必须完成此转移。
    pub(crate) fn attach_memory_charge(&mut self, charge: MemoryPageCharge) {
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        let FolioContents::File { memory_charge, .. } = &mut shared.contents else {
            panic!("memory charge attached to metadata folio");
        };
        assert!(
            memory_charge.is_none(),
            "folio already owns a memory charge"
        );
        *memory_charge = Some(charge);
        background::account_file_page(false);
    }

    /// 在未发表的空用途槽安装原拥有者和终身内容角色。
    fn install(backing: OwnedPages, contents: FolioContents) -> Result<Self, OwnedPages> {
        if backing.order() != 0 {
            return Err(backing);
        }

        let file_role = matches!(
            &contents,
            FolioContents::File {
                memory_charge: None,
                ..
            }
        );
        let descriptor = pages()
            .get(backing.phys_addr() / PAGE_SIZE)
            .expect("folio PFN has no descriptor");
        {
            let mut usage = descriptor.usage();
            usage.install_folio(FolioState {
                backing: Some(backing),
                shared: UnsafeCell::new(FolioShared {
                    references: 1,
                    contents,
                    dirty: 0,
                    writeback: 0,
                    file_prepared: 0,
                }),
                flags: AtomicU8::new(0),
                waiters: WaitQueue::new(),
                _pinned: PhantomPinned,
            });
            if file_role {
                background::account_file_page(true);
            }
        }

        // SAFETY: 生产 PageMap 在独占元数据 RAM 中一次发布，描述符永久有效
        // 且不再移动；此 Pin 只借用描述符，活 folio 用途的固定由私有投影协议保证。
        let descriptor = unsafe { Pin::new_unchecked(descriptor) };
        Ok(Self {
            descriptor,
            owner: None,
        })
    }

    /// 从唯一生产页入口取得未清零 order0 页，不申请额外控制对象。
    pub fn allocate() -> Option<Self> {
        let backing = alloc_pages(0, false)?;
        Some(
            Self::from_pages(backing)
                .unwrap_or_else(|_| unreachable!("order0 allocation changed order")),
        )
    }

    /// 从唯一页入口准备未清零元数据页，不建立第二份缓存或额外控制对象。
    pub fn allocate_metadata() -> Option<Self> {
        let backing = alloc_pages(0, false)?;
        Some(
            Self::from_metadata_pages(backing)
                .unwrap_or_else(|_| unreachable!("order0 allocation changed order")),
        )
    }

    /// 返回同一原页地址，仅用于身份与 DMA 映射，不授予内容访问权。
    pub fn phys_addr(&self) -> usize {
        self.state()
            .backing
            .as_ref()
            .expect("live folio lost backing")
            .phys_addr()
    }

    /// 读取用途锁内的真实引用数；并发快照不授予独占访问权。
    #[cfg(feature = "ktest")]
    pub fn ref_count(&self) -> usize {
        FolioState::shared(&self.descriptor.usage()).references
    }

    /// 只识别由 MemoryPageCharge 标记的 RAM 文件原页，不凭文件打开对象猜后端。
    pub(crate) fn is_memory(&self) -> bool {
        let usage = self.descriptor.usage();
        matches!(
            &FolioState::shared(&usage).contents,
            FolioContents::File {
                memory_charge: Some(_),
                ..
            }
        )
    }

    /// 文件的一次填充结束后为真（包括失败）；元数据须整页全部 sector 已终结。
    /// 检查特定视图的可读性必须使用 range_ready，不能据此忽略失败或部分有效范围。
    #[cfg(feature = "ktest")]
    pub fn is_ready(&self) -> bool {
        let usage = self.descriptor.usage();
        if usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED != 0 {
            return false;
        }
        match &FolioState::shared(&usage).contents {
            FolioContents::File { initialized, .. } => initialized.is_some(),
            FolioContents::Metadata { valid, errors } => (0..SECTORS_PER_PAGE)
                .all(|sector| valid & (1 << sector) != 0 || errors[sector].is_some()),
        }
    }

    /// 文件页只认领一次任意非空页内范围；已完成或已在途时返回 Busy。
    /// 在候选发表前调用，内容锁责任可整体移交 DMA 完成域，不携带 Task 睡眠锁。
    pub fn begin_read(&self, offset: usize, len: usize) -> Result<FolioRead, BlockError> {
        validate_range(offset, len)?;
        let state = self.state();
        {
            let mut usage = self.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            match &shared.contents {
                FolioContents::File {
                    initialized: None, ..
                } => {}
                FolioContents::File { .. } => return Err(BlockError::Busy),
                FolioContents::Metadata { .. } => return Err(BlockError::InvalidRange),
            }
            if state.flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0 {
                return Err(BlockError::Busy);
            }
            shared.references = shared
                .references
                .checked_add(1)
                .expect("folio reference count overflow");
            state.flags.fetch_or(CONTENT_LOCKED, Ordering::Acquire);
        }
        Ok(FolioRead {
            folio: Self {
                descriptor: self.descriptor,
                owner: None,
            },
            offset,
            len,
            completed: false,
        })
    }

    /// 等待同页内容锁后重查请求，仅认领其中首段连续未初始化的 sector。
    /// 请求须按 512 字节对齐；已有效返回 None，失败保留所属 sector 的首次具体错误。
    /// owner 的 offset/len 是实际传输范围，不能用原请求覆盖已有效或 Dirty 字节。
    pub fn begin_metadata_read(
        &self,
        offset: usize,
        len: usize,
    ) -> Result<Option<FolioRead>, BlockError> {
        validate_range(offset, len)?;
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }

        let mut content = self.lock_content();
        let range = {
            let mut usage = self.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            let FolioContents::Metadata { valid, errors } = &shared.contents else {
                return Err(BlockError::InvalidRange);
            };
            let end = (offset + len) / SECTOR_SIZE;
            let mut first = offset / SECTOR_SIZE;
            while first < end {
                if let Some(error) = errors[first] {
                    return Err(error);
                }
                if valid & (1 << first) == 0 {
                    break;
                }
                first += 1;
            }
            if first == end {
                return Ok(None);
            }
            let mut last = first + 1;
            while last < end && valid & (1 << last) == 0 && errors[last].is_none() {
                last += 1;
            }
            shared.references = shared
                .references
                .checked_add(1)
                .expect("folio reference count overflow");
            (first * SECTOR_SIZE, (last - first) * SECTOR_SIZE)
        };
        // guard 的锁责任原样交给唯一 owner，不解锁、不携带非 Send 的 Task guard。
        content.locked = false;
        Ok(Some(FolioRead {
            folio: Self {
                descriptor: self.descriptor,
                owner: None,
            },
            offset: range.0,
            len: range.1,
            completed: false,
        }))
    }

    /// 等待当前内容锁结束；文件保留一次范围结果，元数据只认可整页真实有效范围。
    /// 元数据缺失但无 owner 时返回 NotReady，填充精确窗口须使用 begin_metadata_read。
    pub fn wait_ready(&self) -> Result<(), BlockError> {
        let _content = self.lock_content();
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        match &shared.contents {
            FolioContents::File { initialized, .. } => {
                initialized.ok_or(BlockError::NotReady)?.map(|_| ())
            }
            FolioContents::Metadata { .. } => FolioState::range_ready(shared, 0, PAGE_SIZE),
        }
    }

    /// 非阻塞检查本次 byte 窗口；持内容锁、缺失或失败均不授予读取权。
    pub fn range_ready(&self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let usage = self.descriptor.usage();
        if usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED != 0 {
            return Err(BlockError::NotReady);
        }
        FolioState::range_ready(FolioState::shared(&usage), offset, len)
    }

    /// 在普通 Task 中原始复制已初始化内容到独占内核片段，不形成文件页切片。
    /// 等待内容锁但不排斥用户共享 store，因此结果不承诺一致快照；空目标不访问页。
    pub fn copy_to(&self, offset: usize, dst: &mut [u8]) -> Result<(), BlockError> {
        validate_copy_range(offset, dst.len())?;
        if dst.is_empty() {
            return Ok(());
        }
        self.lock_content().copy_to(offset, dst)
    }

    /// 普通 Task 取得页内容锁；沿固定 WaitQueue 先登记再重查，不持用途锁 park。
    /// 内容锁串行内核修改及读入，不排斥用户共享字节访问或替代 DMA 收权证明。
    pub fn lock_content(&self) -> FolioContent<'_> {
        sched::assert_switchable();
        let state = self.state();
        loop {
            if let Some(content) = self.try_lock_content() {
                return content;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 保活固定用途内的源，pin! 固定栈节点；
            // 所有退出均先撤 registration，再析构节点和 waiter，不遗忘登记。
            let _registration = unsafe { state.waiters.register(node.as_mut()) };
            if let Some(content) = self.try_lock_content() {
                return content;
            }
            waiter.park();
        }
    }

    /// 尝试在用途短锁下取得内容 guard，失败不等待；调用者可先提交已准备前缀。
    /// 元数据的全页 WB 仍排斥内容借用，文件 WB 不阻止 coherent 原始写。
    pub(crate) fn try_lock_content(&self) -> Option<FolioContent<'_>> {
        let usage = self.descriptor.usage();
        let state = usage.folio();
        let shared = FolioState::shared(&usage);
        if state.flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0
            || matches!(&shared.contents, FolioContents::Metadata { .. }) && shared.writeback != 0
        {
            return None;
        }
        state.flags.fetch_or(CONTENT_LOCKED, Ordering::Acquire);
        Some(FolioContent {
            folio: self,
            locked: true,
            _task_local: PhantomData,
        })
    }

    /// canonical 索引准入只接受没有外部页引用、填充或 WB 的原页，Dirty 由后续同步处理。
    pub(crate) fn is_cache_only(&self) -> bool {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        shared.references == 1
            && shared.writeback == 0
            && usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0
    }

    /// 真实读入消费、文件 fault 或 uaccess 标记访问；准备、预读及 WB 不调用。
    pub(crate) fn mark_accessed(&self) {
        let _ = self
            .state()
            .flags
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |flags| {
                let access = if flags & REFERENCED == 0 {
                    REFERENCED
                } else {
                    REFERENCED | ACTIVATE
                };
                Some(flags | access)
            });
    }

    /// 老化只消费访问位，内容锁以及并发新访问均保留原原子协议。
    pub(crate) fn take_accessed(&self) -> (bool, bool) {
        let previous = self
            .state()
            .flags
            .fetch_and(!(REFERENCED | ACTIVATE), Ordering::Relaxed);
        (previous & REFERENCED != 0, previous & ACTIVATE != 0)
    }

    /// 消费一份真实引用并在实际归还 file 原页后返回一；拥有者在页用途锁外结束。
    pub(crate) fn release(self) -> usize {
        let mut this = ManuallyDrop::new(self);
        let freed = this.put();
        drop(this.owner.take());
        freed
    }

    /// Drop 与显式回收共用最后 put，归页返回以后才记录 file 页账。
    fn put(&mut self) -> usize {
        let discarded = {
            let mut usage = self.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            shared.references = shared
                .references
                .checked_sub(1)
                .expect("folio reference count underflow");
            if shared.references == 0 {
                assert_eq!(
                    shared.dirty, 0,
                    "last folio reference retains dirty contents"
                );
                assert_eq!(
                    shared.writeback, 0,
                    "last folio reference retains writeback"
                );
                let file = matches!(
                    &shared.contents,
                    FolioContents::File {
                        memory_charge: None,
                        ..
                    }
                );
                let charge = match &mut shared.contents {
                    FolioContents::File { memory_charge, .. } => memory_charge.take(),
                    FolioContents::Metadata { .. } => None,
                };
                assert_eq!(
                    usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED,
                    0,
                    "last folio reference retains a content owner"
                );
                // SAFETY: 引用归零且无内容锁、Dirty、WB；读入和等待者均自持活引用，
                // 全部固定投影已结束，take 只原地析构用途，页在退出用途锁后归还。
                Some((unsafe { usage.take_folio() }, file, charge))
            } else {
                None
            }
        };

        if let Some(FolioOwner::Metadata(cache)) = &self.owner {
            cache.notify_metadata_available();
        }
        if let Some((backing, file, charge)) = discarded {
            drop(backing);
            drop(charge);
            if file {
                background::account_file_page(false);
            }
            usize::from(file)
        } else {
            0
        }
    }

    /// 元数据控制和旧外部凭证已结束后，只有实际源基准才允许重用原页。
    pub(crate) fn metadata_inventory_available(&self) -> bool {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        shared.references == 1
            && matches!(&shared.contents, FolioContents::Metadata { .. })
            && shared.dirty == 0
            && shared.writeback == 0
            && usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0
            && self.owner.is_none()
    }

    /// 原索引基准及可选真实源基准以外没有 Buffer/Read/WB，才准许静止节点摘离。
    pub(crate) fn index_unused(&self, metadata_baseline: bool, allow_dirty: bool) -> bool {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        shared.references == 1 + usize::from(metadata_baseline)
            && (allow_dirty || shared.dirty == 0)
            && shared.writeback == 0
            && usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0
    }

    /// 在 index→LRU→PageUse 同一最终锁域先核外部拥有，再检查真实 File 清洁状态。
    /// retire 只摘拥有，不可分配、睡眠或析构；内容 guard 必须属于传入的扫描原引用。
    pub(crate) fn retire_clean<T>(
        &self,
        scanner: Option<&Self>,
        content: Option<&FolioContent<'_>>,
        retire: impl FnOnce() -> T,
    ) -> Option<T> {
        if scanner.is_some_and(|folio| {
            !core::ptr::eq(self.descriptor.get_ref(), folio.descriptor.get_ref())
        }) {
            return None;
        }
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        if shared.references != 1 + usize::from(scanner.is_some()) {
            return None;
        }
        let our_content = content.is_some_and(|guard| {
            guard.locked
                && core::ptr::eq(self.descriptor.get_ref(), guard.folio.descriptor.get_ref())
        });
        if (usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED != 0) != our_content
            || shared.dirty != 0
            || shared.writeback != 0
            || !matches!(
                &shared.contents,
                FolioContents::File {
                    initialized: Some(Ok(_)),
                    ..
                }
            )
        {
            return None;
        }
        Some(retire())
    }

    /// 源只有基准和本次私有准备者，全部旧责任结束后重设元数据有效与失败范围。
    pub(crate) fn reset_metadata_inventory(&self) {
        let content = self
            .try_lock_content()
            .expect("metadata inventory acquired a busy folio");
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(
            shared.references, 2,
            "metadata inventory retains an external owner"
        );
        assert_eq!(shared.dirty, 0, "metadata inventory retains dirty contents");
        assert_eq!(shared.writeback, 0, "metadata inventory retains writeback");
        match &mut shared.contents {
            FolioContents::Metadata { valid, errors } => {
                *valid = 0;
                *errors = [None; SECTORS_PER_PAGE];
            }
            FolioContents::File { .. } => panic!("metadata inventory has a file folio"),
        }
        drop(usage);
        drop(content);
    }

    /// 最后范围占用结束时仅撤有效与失败状态，可在 IRQ 中持用途锁执行，不借用页字节。
    ///
    /// # Safety
    /// 调用者仍持物理盘的范围准入锁，已证明本窗口没有其他活 claim；
    /// Buffer、读入和 WB 的真实 claim 均已结束，且解除本次 claim 前不会重新准入。
    /// 同页其他窗口可以仍持内容锁或设备拥有，本操作不得修改其状态或原页字节。
    pub(crate) unsafe fn retire_metadata(&self, offset: usize, len: usize) {
        assert!(validate_range(offset, len).is_ok());
        assert_eq!(offset % SECTOR_SIZE, 0);
        assert_eq!(len % SECTOR_SIZE, 0);
        let mask = sector_mask(offset, len);
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(
            shared.dirty & mask,
            0,
            "retired metadata retains dirty contents"
        );
        assert_eq!(
            shared.writeback & mask,
            0,
            "retired metadata retains writeback"
        );
        let FolioContents::Metadata { valid, errors } = &mut shared.contents else {
            panic!("metadata retirement received a file folio");
        };
        *valid &= !mask;
        errors[offset / SECTOR_SIZE..(offset + len) / SECTOR_SIZE].fill(None);
    }

    /// 返回实际 512 字节脏范围位图；该快照不代表已经提交或持久化。
    pub fn dirty_sectors(&self) -> u8 {
        FolioState::shared(&self.descriptor.usage()).dirty
    }

    /// 返回真实回写范围位图；本接口不建立回写、清脏或完成成功的责任。
    pub fn writeback_sectors(&self) -> u8 {
        FolioState::shared(&self.descriptor.usage()).writeback
    }

    /// 在同一次用途锁内取得 Dirty 与 WB，清脏转 WB 不会从两个快照之间漏失。
    pub(crate) fn writeback_state(&self) -> (u8, u8) {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        (shared.dirty, shared.writeback)
    }

    /// 普通 Task 等待当前页的全部真实 WB 结束；Dirty 为零不能替代这个条件。
    pub fn wait_writeback(&self) {
        sched::assert_switchable();
        let state = self.state();
        loop {
            if self.writeback_sectors() == 0 {
                return;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 本引用保活固定等待源，栈节点在全部登记期间固定；
            // 每条返回路径先撤登记，再释放节点和 waiter，真实完成在状态发布后唤醒。
            let _registration = unsafe { state.waiters.register(node.as_mut()) };
            if self.writeback_sectors() == 0 {
                return;
            }
            waiter.park();
        }
    }

    /// 用途锁内只作原子转移，本卷账取真实外部 File owner，不取得调度锁。
    fn account_state(&self, old_dirty: u8, old_wb: u8, shared: &FolioShared) {
        if matches!(
            &shared.contents,
            FolioContents::File {
                memory_charge: Some(_),
                ..
            }
        ) {
            return;
        }
        background::account_folio(old_dirty, shared.dirty, old_wb, shared.writeback);
        if let Some(FolioOwner::File(inode)) = &self.owner {
            inode
                .disk_volume()
                .expect("disk folio accounting has no volume")
                .writeback_domain()
                .account_folio(old_dirty, shared.dirty, old_wb, shared.writeback);
        }
    }

    /// 用途锁外重报可见脏义务；只有可能减轻压力时唤醒 writer，新 Dirty 不制造空唤醒。
    fn notify_dirty_state(&self, relieved: bool) {
        if self.is_memory() {
            return;
        }
        if let Some(FolioOwner::File(inode)) = &self.owner {
            let volume = inode.disk_volume().expect("disk dirty folio has no volume");
            let domain = volume.writeback_domain();
            if self.dirty_sectors() != 0 {
                domain.notify_dirty(volume);
            }
            if relieved {
                domain.wake_writers();
            }
        }
    }

    /// 在用途短锁内定位状态，再投影绑定本活引用的共享 Pin，不将锁带入 IO 或等待。
    fn state(&self) -> Pin<&FolioState> {
        let pointer = {
            let usage = self.descriptor.usage();
            usage.folio() as *const FolioState
        };
        // SAFETY: from_pages 先完成用途写入再发表首引用；此后活引用只通过
        // UnsafeCell 修改共享头，从不创建 &mut FolioState 或替换用途。
        // self 保证 references 至少为一，本借用终止前无法执行最后引用移出；
        // 永久 descriptor 固定该用途槽地址，内容锁、等待源和 backing 均不移动。
        unsafe { Pin::new_unchecked(&*pointer) }
    }
}

impl Clone for FolioRef {
    /// 克隆真实内容域拥有者和原页计数，不复制内容；内部无 anchor 的引用仍保持无 anchor。
    fn clone(&self) -> Self {
        let owner = self.owner.clone();
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        shared.references = shared
            .references
            .checked_add(1)
            .expect("folio reference count overflow");
        Self {
            descriptor: self.descriptor,
            owner,
        }
    }
}

impl Drop for FolioRef {
    /// 最后 put 原页后再结束内容域拥有者；脏、内容锁和 WB 仍须由真实责任者结束。
    fn drop(&mut self) {
        self.put();
    }
}

impl FolioContent<'_> {
    /// 借用当前 guard 实际锁住的原页，用于最终授权与回写的身份核对。
    pub(crate) fn folio(&self) -> &FolioRef {
        self.folio
    }

    /// 在内容锁内读取真实 WB 范围；检查为零之后仍须保持授权与清脏互斥。
    pub(crate) fn writeback_sectors(&self) -> u8 {
        self.folio.writeback_sectors()
    }

    /// 读取本页已发表 FS 块的准备事实，最终授权核对时不取得 inode 块根。
    pub(crate) fn file_prepared_sectors(&self) -> u8 {
        FolioState::shared(&self.folio.descriptor.usage()).file_prepared
    }

    /// 保存已经发表并初始化的整 FS 块事实，调用者持 inode invalidate 和本内容锁。
    /// 块根修改必须先真实提交；此位图不承担分配或块地址查询责任。
    pub(crate) fn mark_file_prepared(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }

        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        if !matches!(&shared.contents, FolioContents::File { .. }) {
            return Err(BlockError::InvalidRange);
        }
        FolioState::range_ready(shared, offset, len)?;
        shared.file_prepared |= sector_mask(offset, len);
        Ok(())
    }

    /// 只重标当前 EOF 内已真实准备的整 FS 块；hole 与 EOF 外块不产生脏义务。
    /// 调用者须保留同一 canonical inode，并按其 invalidate 协议持有本页内容锁；
    /// block_size 必须来自该 inode 所属文件系统的几何。索引内部引用可以没有外部 inode，
    /// 因此文件与元数据角色由固定原页状态区分。
    pub(crate) fn mark_prepared_dirty(
        &mut self,
        valid_bytes: usize,
        block_size: usize,
    ) -> Result<(), BlockError> {
        if PAGE_SIZE < valid_bytes
            || !matches!(
                block_size,
                FILE_BLOCK_BYTES_1K | FILE_BLOCK_BYTES_2K | FILE_BLOCK_BYTES_4K
            )
        {
            return Err(BlockError::InvalidRange);
        }
        if self.folio.is_memory() {
            return Ok(());
        }

        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        if !matches!(&shared.contents, FolioContents::File { .. }) {
            return Err(BlockError::InvalidRange);
        }
        if valid_bytes == 0 {
            return Ok(());
        }
        let end = valid_bytes.div_ceil(block_size) * block_size;
        FolioState::range_ready(shared, 0, end)?;
        let old_dirty = shared.dirty;
        shared.dirty |= shared.file_prepared & sector_mask(0, end);
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(false);
        Ok(())
    }

    /// 初始化已有效的文件缓存 byte 范围，不把未映射 hole 的零内容当作盘块 Dirty。
    /// 调用者先撤该页共享写许可并完成 TLB 同步，再按真实块映射标脏。
    pub(crate) fn zero_file_range(&mut self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_copy_range(offset, len)?;
        if len == 0 {
            return Ok(());
        }
        {
            let usage = self.folio.descriptor.usage();
            let shared = FolioState::shared(&usage);
            if !matches!(&shared.contents, FolioContents::File { .. }) {
                return Err(BlockError::InvalidRange);
            }
            FolioState::range_ready(shared, offset, len)?;
        }

        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        let zeros = [0u8; SECTOR_SIZE];
        for part in (0..len).step_by(zeros.len()) {
            let count = (len - part).min(zeros.len());
            // SAFETY: 内容 guard 保活并串行内核修改，完整窗口已初始化且在原页内；
            // 独占零片段不与目标重叠，文件内容仅作原始访问，不建立共享 RAM 引用。
            unsafe { arch::copy_user_bytes((address + part) as *mut u8, zeros.as_ptr(), count) };
        }
        Ok(())
    }

    /// 缩小后的原文件尾页保留有效前缀，只清已删除整 FS 块的准备和脏义务。
    ///
    /// # Safety
    /// 调用者持同 inode 写序列化、invalidate 独占和 mapping WB 互斥；新 size 已发表。
    /// 本页的共享写许可已撤且全部 CPU 失效完成，唯一填充和真实 WB 已结束；
    /// 删除范围无需落盘，保留的部分 FS 块仍由真实块树负责尾零持久化。
    pub(crate) unsafe fn truncate_file_tail(
        &mut self,
        valid: usize,
        block_size: usize,
    ) -> Result<(), BlockError> {
        if valid == 0
            || PAGE_SIZE <= valid
            || !matches!(
                block_size,
                FILE_BLOCK_BYTES_1K | FILE_BLOCK_BYTES_2K | FILE_BLOCK_BYTES_4K
            )
        {
            return Err(BlockError::InvalidRange);
        }
        assert_eq!(
            self.writeback_sectors(),
            0,
            "file truncate retains writeback"
        );
        self.zero_file_range(valid, PAGE_SIZE - valid)?;
        let retained = valid.div_ceil(block_size) * block_size;
        let keep = sector_mask(0, retained);
        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        let old_dirty = shared.dirty;
        shared.dirty &= keep;
        shared.file_prepared &= keep;
        if valid % block_size != 0 {
            let start = valid / block_size * block_size;
            shared.dirty |= shared.file_prepared & sector_mask(start, block_size);
        }
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(true);
        Ok(())
    }

    /// 在持内容锁期间检查真实 byte 窗口，不把同页其他 sector 的成功借给本范围。
    pub fn range_ready(&self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let usage = self.folio.descriptor.usage();
        FolioState::range_ready(FolioState::shared(&usage), offset, len)
    }

    /// 持锁原始复制到独占片段，不建立共享文件内容的 Rust 引用或一致快照。
    pub fn copy_to(&self, offset: usize, dst: &mut [u8]) -> Result<(), BlockError> {
        validate_copy_range(offset, dst.len())?;
        if dst.is_empty() {
            return Ok(());
        }
        self.range_ready(offset, dst.len())?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY: guard 保活原页且已验证完整初始化范围，原始源不创建共享 RAM 引用；
        // dst 是调用者独占的普通内核片段，不与本页有效内容重叠。
        // metadata 锁排斥写者，file 用户共享 store 可并发，汇编只作有界字节访问。
        unsafe { arch::copy_user_bytes(dst.as_mut_ptr(), address as *const u8, dst.len()) };
        Ok(())
    }

    /// 从普通内核片段原始修改已有效范围，修改前标记覆盖到的实际 sector 为 Dirty。
    /// 不清除已有 WB，不承诺用户共享 store 与此次复制之间的快照或顺序。
    pub fn copy_from(&mut self, offset: usize, src: &[u8]) -> Result<(), BlockError> {
        validate_copy_range(offset, src.len())?;
        if src.is_empty() {
            return Ok(());
        }
        self.mark_dirty(offset, src.len())?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY: guard 保活原页并串行内核修改，mark_dirty 已验证已初始化范围；
        // src 是已初始化内核片段，其借用期间不能并发修改，不与目标原页重叠。
        // 目标没有 Rust 内容引用，用户共享 store 仅经原始访问；汇编不越界。
        unsafe { arch::copy_user_bytes(address as *mut u8, src.as_ptr(), src.len()) };
        Ok(())
    }

    /// 仅对终身元数据角色借用已初始化窗口；返回借用不能超出本内容 guard。
    /// 持该借用时不得做可能 fault 的用户访问，也不得通过物理地址交用户或设备写。
    pub fn metadata_bytes(&self, offset: usize, len: usize) -> Result<&[u8], BlockError> {
        self.metadata_range(offset, len)?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY: 终身 Metadata 角色从未交用户共享写，guard 排斥 CPU 修改和初始化 DMA；
        // 完整窗口已经初始化，folio 引用保活固定原页，切片绑定 guard 且不越界。
        Ok(unsafe { slice::from_raw_parts(address as *const u8, len) })
    }

    /// 在同一元数据锁下借用可修改窗口，并在借用前标记对应 sector 为 Dirty。
    /// 邻记录和校验和应在此真实窗口内同步修改，不得拷回旧整块快照。
    pub fn metadata_bytes_mut(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<&mut [u8], BlockError> {
        self.metadata_range(offset, len)?;
        self.mark_dirty(offset, len)?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY: Metadata 角色无用户共享写，内容 guard 排斥其他内核访问；
        // mark_dirty 已拒绝与在途 WB 重叠的窗口，guard 保持该条件；
        // &mut self 结束此前借用，窗口已初始化且保活，唯一可变切片绑定本 guard。
        Ok(unsafe { slice::from_raw_parts_mut(address as *mut u8, len) })
    }

    /// 在真实删除或失效协议下作废整页有效、失败和 Dirty 状态，不代表回写成功。
    ///
    /// # Safety
    /// 调用者须已从真实索引摘离本页并阻止新取得，结束全部 IO/WB 和用户映射访问，
    /// 完成必要失效及 TLB 同步；其他持页者不得再按旧内容使用或交设备。
    /// 内容须属于未发表候选或实际删除/失效协议，且确实无需落盘；
    /// 独占引用计数不能代替上述证明，已接受的文件系统修改不得仅为归页而丢弃。
    pub unsafe fn discard(&mut self) {
        assert!(self.locked, "discard without a content guard");
        let mut usage = self.folio.descriptor.usage();
        assert!(
            usage.folio().flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0,
            "discard lost content lock"
        );
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(shared.writeback, 0, "discard retains writeback");
        let old_dirty = shared.dirty;
        // 本 guard 独占内容锁，初始化 owner 无法同时存在；设备尾部与外部
        // 映射失效仍由 caller 证明，状态清除不能取消任何已接受的访问责任。
        match &mut shared.contents {
            FolioContents::File { initialized, .. } => *initialized = None,
            FolioContents::Metadata { valid, errors } => {
                *valid = 0;
                errors.fill(None);
            }
        }
        shared.dirty = 0;
        shared.file_prepared = 0;
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(true);
    }

    /// 在真实元数据删除协议下失效准确窗口，保留同页其他块的内容与义务。
    ///
    /// # Safety
    /// 原块已从真实块树摘离并禁止旧映射新取得，全部旧内容消费与 IO/WB 已结束；
    /// 其他持页者不得再按该窗口旧内容使用，窗口确实无需落盘。
    /// 必须先执行本失效，再把物理块归还位图，不能用于取消已接受修改。
    pub(crate) unsafe fn forget_metadata(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }
        let mask = sector_mask(offset, len);
        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(
            shared.writeback & mask,
            0,
            "forget retains metadata writeback"
        );
        let FolioContents::Metadata { valid, errors } = &mut shared.contents else {
            return Err(BlockError::InvalidRange);
        };
        *valid &= !mask;
        errors[offset / SECTOR_SIZE..(offset + len) / SECTOR_SIZE].fill(None);
        let old_dirty = shared.dirty;
        shared.dirty &= !mask;
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        Ok(())
    }

    /// 确认终身元数据角色和完整范围，普通文件页不能借用任何内容切片。
    fn metadata_range(&self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let usage = self.folio.descriptor.usage();
        let shared = FolioState::shared(&usage);
        if !matches!(&shared.contents, FolioContents::Metadata { .. }) {
            return Err(BlockError::InvalidRange);
        }
        if shared.writeback & sector_mask(offset, len) != 0 {
            return Err(BlockError::Busy);
        }
        FolioState::range_ready(shared, offset, len)
    }

    /// 持内容锁把本次已初始化范围的 Dirty 移入 WB，后续标脏保留为新义务。
    ///
    /// # Safety
    /// 文件来源的全部共享可写映射须先撤销写许可并完成真实 TLB 同步；
    /// 本 guard 仍保持对应许可提交与清脏的互斥。元数据来源不得映射给用户。
    /// 返回拥有者只能在其全部设备访问实际结束后完成；准备失败须保留脏义务。
    pub(crate) unsafe fn begin_writeback(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<Option<FolioWriteback>, BlockError> {
        validate_range(offset, len)?;
        if self.folio.is_memory() {
            return Err(BlockError::Unsupported);
        }
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }

        let mask = sector_mask(offset, len);
        let folio = self.folio.clone();
        let original_dirty;
        {
            let mut usage = self.folio.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            if shared.writeback & mask != 0 {
                return Err(BlockError::Busy);
            }
            if shared.dirty & mask == 0 {
                return Ok(None);
            }
            FolioState::range_ready(shared, offset, len)?;
            original_dirty = shared.dirty & mask;
            let old_dirty = shared.dirty;
            let old_wb = shared.writeback;
            shared.writeback |= mask;
            shared.dirty &= !mask;
            self.folio.account_state(old_dirty, old_wb, shared);
        }

        background::note_writeback_start();
        if let Some(FolioOwner::File(inode)) = &self.folio.owner {
            inode
                .disk_volume()
                .expect("disk writeback folio has no volume")
                .writeback_domain()
                .note_writeback_start();
        }
        Ok(Some(FolioWriteback {
            folio,
            offset: offset as u16,
            len: len as u16,
            completed: false,
            original_dirty,
            writeback_domain: None,
            background: false,
        }))
    }

    /// 只在有效 byte 窗口上增加 Dirty；WB 独立保留，失败不会污染其他范围。
    pub(crate) fn mark_dirty(&mut self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        FolioState::range_ready(shared, offset, len)?;
        if matches!(
            &shared.contents,
            FolioContents::File {
                memory_charge: Some(_),
                ..
            }
        ) {
            return Ok(());
        }
        let mask = sector_mask(offset, len);
        if matches!(&shared.contents, FolioContents::Metadata { .. })
            && shared.writeback & mask != 0
        {
            return Err(BlockError::Busy);
        }
        let old_dirty = shared.dirty;
        shared.dirty |= mask;
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(false);
        Ok(())
    }
}

impl Drop for FolioContent<'_> {
    /// 归还实际内容锁并在用途锁外通知等待者；已交读入 owner 的责任不二次解锁。
    fn drop(&mut self) {
        if !self.locked {
            return;
        }
        let state = self.folio.state();
        {
            let usage = self.folio.descriptor.usage();
            assert!(
                usage.folio().flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0,
                "folio content lock lost"
            );
            state.flags.fetch_and(!CONTENT_LOCKED, Ordering::Release);
        }

        state.waiters.wake_all();
    }
}

impl FolioWriteback {
    /// 为卷提交的凭证附真实统计域，后台标志只区分当前有限轮次的完成账。
    pub(crate) fn with_writeback_domain(
        mut self,
        domain: &Arc<WritebackDomain>,
        background: bool,
    ) -> Self {
        if !matches!(&self.folio.owner, Some(FolioOwner::File(_))) {
            domain.note_writeback_start();
        }
        self.writeback_domain = Some(domain.clone());
        self.background = background;
        self
    }

    /// 借用本轮仍保活的原页地址，只用于有界设备映射。
    pub(crate) fn phys_addr(&self) -> usize {
        self.folio.phys_addr()
    }

    /// 返回本轮原页内起点，不授予共享内容的切片访问。
    pub(crate) fn offset(&self) -> usize {
        usize::from(self.offset)
    }

    /// 返回本轮交设备的完整 byte 范围。
    pub(crate) fn len(&self) -> usize {
        usize::from(self.len)
    }

    /// 只分类原页终身角色，不借用任何仍在途的内容。
    #[cfg(feature = "ktest")]
    pub(crate) fn is_metadata(&self) -> bool {
        matches!(
            &FolioState::shared(&self.folio.descriptor.usage()).contents,
            FolioContents::Metadata { .. }
        )
    }

    /// 完成后的凭证不能再次开始设备访问。
    pub(crate) fn is_completed(&self) -> bool {
        self.completed
    }

    /// 真实收权后只结束本轮 WB；未接受原脏范围恢复，已接受失败先记错且不重放。
    ///
    /// # Safety
    /// 整个凭证范围的全部设备访问已由真实 used 或确认 reset 结束；
    /// accepted 是真实提交成功的连续前缀，result 只属于该前缀的设备完成。
    /// 软件取消、超时或其他请求完成不得用来证明收权。
    pub(crate) unsafe fn complete(&mut self, accepted: usize, result: IoResult) {
        assert!(accepted <= self.len() && result.completed <= accepted);
        assert_eq!(accepted % SECTOR_SIZE, 0, "unaligned writeback acceptance");
        if !self.completed {
            let error = result
                .error
                .or_else(|| (result.completed < accepted).then_some(BlockError::ShortIo));
            if 0 < accepted {
                if let Some(error) = error {
                    match &self.folio.owner {
                        Some(FolioOwner::File(inode)) => {
                            inode.mapping().record_wb_error(error.into())
                        }
                        Some(FolioOwner::Metadata(cache)) => cache.record_wb_error(error.into()),
                        None => {}
                    }
                }
            }
            self.finish(accepted);
            if 0 < accepted {
                background::note_completion(accepted, result.completed);
                if let Some(FolioOwner::File(inode)) = &self.folio.owner {
                    inode
                        .disk_volume()
                        .expect("disk writeback completion has no volume")
                        .writeback_domain()
                        .note_completion(accepted, result.completed, error.map(Into::into));
                } else if let Some(domain) = &self.writeback_domain {
                    domain.note_completion(accepted, result.completed, error.map(Into::into));
                }
                if self.background {
                    self.writeback_domain
                        .as_ref()
                        .expect("background writeback has no domain")
                        .note_round_io(accepted, result.completed);
                }
            }
        }
    }

    /// 错误已经发布；用途锁内退役本轮并恢复未接受原义务，锁外才唤醒。
    fn finish(&mut self, accepted: usize) {
        let mask = sector_mask(self.offset(), self.len());
        let unaccepted = if accepted < self.len() {
            sector_mask(self.offset() + accepted, self.len() - accepted)
        } else {
            0
        };
        let state = self.folio.state();
        {
            let mut usage = self.folio.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            assert_eq!(shared.writeback & mask, mask, "folio writeback range lost");
            let old_dirty = shared.dirty;
            let old_wb = shared.writeback;
            shared.dirty |= self.original_dirty & unaccepted;
            shared.writeback &= !mask;
            self.folio.account_state(old_dirty, old_wb, shared);
            self.completed = true;
        }

        state.waiters.wake_all();
        self.folio.notify_dirty_state(accepted == 0);
    }
}

impl Drop for FolioWriteback {
    /// 未提交准备者恢复真实脏义务；设备拥有期由 DMA 外壳禁止提前析构。
    fn drop(&mut self) {
        if !self.completed {
            self.finish(0);
        }
    }
}

impl FolioRead {
    /// 交设备前为唯一文件读入凭证附原 inode，真实设备及隔离拥有保该来源到全尾结束。
    /// 仅修改本凭证中的原 FolioRef，索引内部引用仍不反持 inode。
    pub(crate) fn with_file_owner(mut self, owner: InodeRef) -> Self {
        self.folio.attach_file_owner(owner);
        self
    }

    /// aux 交设备前附原缓存，等待和设备尾部都保活同一 MetadataCache。
    pub(crate) fn with_metadata_owner(mut self, owner: Arc<MetadataCache>) -> Self {
        self.folio.attach_metadata_owner(owner);
        self
    }

    /// 返回仍由本读入者保活的原页地址，DMA 层须维护其自身交权与收权协议。
    pub fn phys_addr(&self) -> usize {
        self.folio.phys_addr()
    }

    /// 返回本次页内读入起点。
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// 返回本次非空读入长度。
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.len
    }

    /// DMA 层拒绝将已经完成其初始化责任的拥有者再次交给设备。
    pub(crate) fn is_completed(&self) -> bool {
        self.completed
    }

    /// 只观察读入原页的终身文件角色，不借用内容或改变初始化责任。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_is_file(&self) -> bool {
        matches!(
            &FolioState::shared(&self.folio.descriptor.usage()).contents,
            FolioContents::File { .. }
        )
    }

    /// 完成前只借用本 owner 的缺失初始化范围；交设备后 DMA 层不得调用此入口。
    pub fn bytes_mut(&mut self) -> &mut [MaybeUninit<u8>] {
        assert!(
            !self.completed,
            "completed folio read cannot write contents"
        );
        let address = arch::phys_to_virt(self.phys_addr()) + self.offset;
        // SAFETY: 本对象唯一持有内容锁，范围尚未有效且不能交用户共享写；
        // metadata 的已有有效范围不在本 owner 内，guard 被内容锁排斥。
        // MaybeUninit 允许未初始化字节，借用绑定 &mut self；DMA 期间整体保护 owner。
        unsafe { slice::from_raw_parts_mut(address as *mut MaybeUninit<u8>, self.len) }
    }

    /// 在真实 DMA 全尾后发表唯一结果、解锁并锁外唤醒；重复完成不改变首次结果。
    ///
    /// # Safety
    /// 本 owner 范围的全部设备、CPU 初始化访问必须已经结束；成功要求其全部字节初始化。
    /// 部分成功与失败不授予本范围内容访问权，完成后原初始化地址不能再被写入。
    /// 元数据其他有效或失败范围保留；完成不约束后续合法 guard 修改或用户共享 store。
    pub unsafe fn complete(&mut self, result: IoResult) {
        if self.completed {
            return;
        }
        let initialized = match result.error {
            Some(error) => Err(error),
            None if result.completed == self.len => Ok(InitializedRange {
                offset: self.offset as u16,
                len: self.len as u16,
            }),
            None => Err(BlockError::ShortIo),
        };
        self.publish(initialized);
    }

    /// 持短用途锁发表终态并结束独立内容锁责任，释放锁后通知实际等待者。
    fn publish(&mut self, initialized: Result<InitializedRange, BlockError>) {
        let state = self.folio.state();
        {
            let mut usage = self.folio.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            assert!(
                state.flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0,
                "folio read lost its content lock"
            );
            match &mut shared.contents {
                FolioContents::File {
                    initialized: result,
                    ..
                } => {
                    assert!(result.is_none(), "folio result already published");
                    *result = Some(initialized);
                }
                FolioContents::Metadata { valid, errors } => {
                    let sectors = sector_mask(self.offset, self.len);
                    assert_eq!(
                        *valid & sectors,
                        0,
                        "metadata read overwrites valid contents"
                    );
                    for stored_error in &mut errors
                        [self.offset / SECTOR_SIZE..(self.offset + self.len) / SECTOR_SIZE]
                    {
                        assert!(
                            stored_error.is_none(),
                            "metadata read overwrites failed contents"
                        );
                        if let Err(error) = initialized {
                            *stored_error = Some(error);
                        }
                    }
                    if initialized.is_ok() {
                        *valid |= sectors;
                    }
                }
            }
            self.completed = true;
            state.flags.fetch_and(!CONTENT_LOCKED, Ordering::Release);
        }

        state.waiters.wake_all();
    }
}

impl Drop for FolioRead {
    /// 未提交读入者结束自身填充责任并唤醒，已完成者只释放所保原页引用。
    fn drop(&mut self) {
        if !self.completed {
            self.publish(Err(BlockError::QueueStopped));
        }
    }
}

/// 拒绝空范围、溢出和跨页访问，所有内容入口共用同一几何边界。
fn validate_range(offset: usize, len: usize) -> Result<(), BlockError> {
    if len == 0 || offset.checked_add(len).is_none_or(|end| PAGE_SIZE < end) {
        return Err(BlockError::InvalidRange);
    }
    Ok(())
}

/// 原始复制允许页尾零长请求，非零请求仍须满足同一 byte 几何边界。
fn validate_copy_range(offset: usize, len: usize) -> Result<(), BlockError> {
    if len == 0 {
        return if offset <= PAGE_SIZE {
            Ok(())
        } else {
            Err(BlockError::InvalidRange)
        };
    }
    validate_range(offset, len)
}

/// 将已验证非空 byte 窗口投影到实际覆盖的 512 字节 sector，不扩大到整页。
fn sector_mask(offset: usize, len: usize) -> u8 {
    let first = offset / SECTOR_SIZE;
    let last = (offset + len - 1) / SECTOR_SIZE;
    (((1_u16 << (last - first + 1)) - 1) << first) as u8
}
