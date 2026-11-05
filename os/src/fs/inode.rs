//! canonical inode 的加载结果、运行属性和块映射分别受实际拥有与同步协议保护。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::marker::PhantomPinned;
use core::ops::Deref;
use core::pin::{pin, Pin};
#[cfg(feature = "ktest")]
use core::sync::atomic::AtomicUsize;
use core::sync::atomic::{AtomicBool, Ordering};
use spin::Once;

use crate::block::{AccessMode, FsClaim, IoResult, MetadataIoContext, SECTOR_BYTES};
use crate::cache::{BufferRef, FileMapping, PreparedFile};
use crate::fs::dentry::{Dentry, DentryCache, DentryRef};
use crate::fs::ext4::{
    codec, finish_error, BlockMapping, Ext4Volume, InodeData, PreparedDirectoryEdit,
    PreparedDirectoryMappings, PreparedInodeMapping, PreparedInodeRelease, UnpublishedInode,
};
use crate::fs::memfs::{MemoryFs, MemoryPageCharge, MemorySpecial};
use crate::fs::readahead::{self, ReadaheadState};
use crate::fs::types::{
    DirectoryCursor, DirectoryEntry, InodeKind, InodeTime, NAME_MAX, S_IALLUGO, S_IFDIR, S_IFLNK,
    S_IFREG, S_IRWXUGO, S_ISGID, S_IXGRP,
};
use crate::fs::writeback;
use crate::fs::{background, FsError, ReadBuffer, WriteBuffer};
use crate::mm::folio::FolioContent;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::{FolioRef, PAGE_SIZE};
use crate::sched;
use crate::sync::{ReadGuard, SleepRwLock, SpinLock, WriteGuard};
use crate::task::ReclaimScope;
use crate::time;
use crate::wait::{WaitNode, WaitQueue, Waiter};
use crate::workqueue::{Work, WorkQueue, WorkQueueConfig};

/// 内存目录的每项记账尺寸，点项与普通名字使用同一单位。
const MEMORY_DIRENT_BYTES: u64 = 20;

#[cfg(feature = "ktest")]
type LoadHook = Arc<dyn Fn(&InodeRef) + Send + Sync>;

/// 独立缓存拥有有限 canonical 域；最后使用和淘汰都在同一索引锁内交给具体回收工作。
/// ponytail: 固定预算内线性查找，预算显著扩大且实测查找成为瓶颈时改为有限散列表。
pub struct InodeCache {
    domain: Arc<InodeDomain>,
    #[cfg(feature = "ktest")]
    load_hook: SpinLock<Option<LoadHook>>,
    #[cfg(feature = "ktest")]
    loads: AtomicUsize,
}

/// canonical inode 槽和名字域弱登记共用短索引锁；外部使用由 InodeRef 在同锁内登记。
pub(crate) struct InodeDomain {
    index: SpinLock<InodeIndex>,
    work: Once<Pin<Arc<Work>>>,
    waiters: WaitQueue,
    retained: AtomicBool,
    retained_next: SpinLock<Option<Arc<InodeDomain>>>,
}

struct InodeIndex {
    /// 活视图唯一共享的有限名字索引，失去最后拥有后可重新选择预算。
    dentry_cache: Weak<DentryCache>,
    slots: Vec<Option<InodeSlot>>,
    /// 实际卷绑定保留至显式卸载或域关闭后的同步停止；容量与 canonical 槽预算相同。
    volumes: Vec<Arc<Ext4Volume>>,
    cache_open: bool,
    evicted: usize,
    /// 同一索引锁内预留的 Work 发表责任，归零前不能宣布整个域已退场。
    dispatching: usize,
}

struct InodeSlot {
    inode: Arc<Inode>,
    users: usize,
    eviction: Eviction,
    /// 若此 inode 是卷根，弱登记身份但不阻止名字对象和 inode 回收。
    root_dentry: Weak<Dentry>,
}

#[derive(Clone, Copy)]
enum Eviction {
    Cached,
    Requested,
    Running,
    Failed(FsError),
}

impl InodeSlot {
    /// 优先复用仍存活的卷根名字；弱登记不延长 dentry 或 canonical inode 的寿命。
    fn bind_root(&mut self, candidate: DentryRef) -> (DentryRef, Option<DentryRef>) {
        if let Some(root) = DentryRef::upgrade(&self.root_dentry) {
            (root, Some(candidate))
        } else {
            self.root_dentry = candidate.downgrade();
            (candidate, None)
        }
    }
}

/// 以稳定 canonical inode 身份定位其有限索引槽。
fn root_slot_position(index: &InodeIndex, inode: &InodeRef) -> usize {
    index
        .slots
        .iter()
        .position(|slot| {
            slot.as_ref()
                .is_some_and(|slot| Arc::ptr_eq(&slot.inode, &inode.0))
        })
        .expect("canonical root inode has no cache slot")
}

/// 常驻域仅供具体 inode 回收使用，执行者仍来自原共享 pool。
static EVICTION_QUEUE: Once<WorkQueue> = Once::new();
/// 失败的删除义务由独立服务强持，链上节点不反向拥有服务或自身。
static FAILED_EVICTIONS: SpinLock<Option<Arc<InodeDomain>>> = SpinLock::new(None);

/// BSP 在原共享 pool 准备后、发布运行许可之前准备 inode 回收和卷回写服务。
pub(crate) fn initialize_inode_eviction() -> Result<(), FsError> {
    assert!(
        EVICTION_QUEUE.get().is_none(),
        "inode eviction initialized twice"
    );
    let queue = WorkQueue::try_new(WorkQueueConfig {
        max_active: 8,
        ordered: false,
        reclaim: true,
    })
    .map_err(|_| FsError::NoMemory)?;
    EVICTION_QUEUE.call_once(|| queue);
    background::initialize()
}

impl InodeDomain {
    /// 缓存发表之前准备同预算的 canonical 槽、卷绑定和单个 Work，提交与最后 put 不再分配。
    fn try_new(capacity: usize) -> Result<Arc<Self>, FsError> {
        let mut slots = Vec::new();
        slots
            .try_reserve_exact(capacity)
            .map_err(|_| FsError::NoMemory)?;
        slots.resize_with(capacity, || None);
        let mut volumes = Vec::new();
        volumes
            .try_reserve_exact(capacity)
            .map_err(|_| FsError::NoMemory)?;
        let domain = Arc::try_new(Self {
            index: SpinLock::new(InodeIndex {
                dentry_cache: Weak::new(),
                slots,
                volumes,
                cache_open: true,
                evicted: 0,
                dispatching: 0,
            }),
            work: Once::new(),
            waiters: WaitQueue::new(),
            retained: AtomicBool::new(false),
            retained_next: SpinLock::new(None),
        })
        .map_err(|_| FsError::NoMemory)?;

        let work = Work::try_new(
            EVICTION_QUEUE
                .get()
                .expect("inode eviction service is not prepared"),
            0,
            evict_inodes,
            Arc::as_ptr(&domain) as usize,
        )
        .map_err(|_| FsError::NoMemory)?;
        domain.work.call_once(|| work);
        Ok(domain)
    }

    /// 调用者持卷绑定锁且已确认旧域退出；预算不足时不取得卷拥有或开放回写服务。
    pub(crate) fn bind_volume(&self, volume: &Arc<Ext4Volume>) -> Result<(), FsError> {
        let mut index = self.index.lock();
        if !index.cache_open {
            return Err(FsError::Busy);
        }
        if index.volumes.len() == index.slots.len() {
            return Err(FsError::NoMemory);
        }
        index.volumes.push(volume.clone());
        Ok(())
    }

    /// 调用者持目标卷绑定锁；仅在全部 canonical 槽均不属于该卷时移交原 Arc，锁外同步停止。
    pub(crate) fn take_unused_volume(&self, volume: &Arc<Ext4Volume>) -> Option<Arc<Ext4Volume>> {
        let mut index = self.index.lock();
        if index
            .slots
            .iter()
            .flatten()
            .any(|slot| Arc::ptr_eq(&slot.inode.disk().filesystem, volume))
        {
            return None;
        }
        let position = index
            .volumes
            .iter()
            .position(|bound| Arc::ptr_eq(bound, volume))?;
        Some(index.volumes.swap_remove(position))
    }

    /// 回收 Work 在 Task 中停止已关闭域的无槽卷，活 inode 和失败槽继续保留各自卷绑定。
    fn retire_volumes(&self) {
        loop {
            let volume = {
                let mut index = self.index.lock();
                if index.cache_open {
                    return;
                }
                let Some(position) = index.volumes.iter().position(|volume| {
                    !index
                        .slots
                        .iter()
                        .flatten()
                        .any(|slot| Arc::ptr_eq(&slot.inode.disk().filesystem, volume))
                }) else {
                    return;
                };
                index.volumes.swap_remove(position)
            };
            // 原 Work 票保住域身份，卷拥有保留到同步停止结束；新域不能越过旧域的 Weak 绑定。
            volume.writeback_domain().stop_sync();
        }
    }

    /// 每个被 Work 接受的票另持一份真实域拥有，重复提交只归还本次未接受的票。
    fn queue(self: &Arc<Self>) {
        let owner = Arc::into_raw(self.clone());
        if !self
            .work
            .get()
            .expect("inode eviction work is not prepared")
            .as_ref()
            .queue()
            .expect("permanent inode eviction queue closed")
        {
            // SAFETY: owner 来自本次 into_raw，未被 Work 接受；这里只有本次票的唯一归还者。
            drop(unsafe { Arc::from_raw(owner) });
        }
        {
            let mut index = self.index.lock();
            assert!(0 < index.dispatching, "inode eviction dispatch lost");
            index.dispatching -= 1;
        }
        self.waiters.wake_all();
    }

    /// 等待已接受回收工作的真实尾部；失败仍占原槽并保留具体原因。
    fn flush(&self) -> Result<(), FsError> {
        self.work
            .get()
            .expect("inode eviction work is not prepared")
            .as_ref()
            .flush();

        let index = self.index.lock();
        for slot in index.slots.iter().flatten() {
            if let Eviction::Failed(error) = slot.eviction {
                return Err(error);
            }
        }
        Ok(())
    }

    /// 原缓存预算固定本轮扫描上界，不通过弱升级重造 inode 或遗失待回收义务。
    pub(crate) fn dirty_inode_limit(&self) -> usize {
        self.index.lock().slots.len()
    }

    /// 索引锁内取得仍可用的原身份；已进入真实回收的对象不再接受新的 WB 使用。
    pub(crate) fn dirty_inode_at(self: &Arc<Self>, position: usize) -> Option<InodeRef> {
        let mut index = self.index.lock();
        let slot = index.slots.get_mut(position)?.as_mut()?;
        if !slot.inode.disk().dirty_obligation.load(Ordering::Acquire)
            || matches!(slot.eviction, Eviction::Running | Eviction::Failed(_))
            || matches!(slot.eviction, Eviction::Requested) && slot.users == 0
        {
            return None;
        }
        Some(use_slot(self, slot, false))
    }

    /// 所有后端和名字查找都复用此 canonical 加载路径，回收中的原身份先等真实结束。
    pub(crate) fn iget(
        self: &Arc<Self>,
        filesystem: &Arc<Ext4Volume>,
        number: u64,
        _cache: Option<&InodeCache>,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<InodeRef, FsError> {
        sched::assert_switchable();
        filesystem.bind_inode_domain(self)?;
        if let Some(inode) = self.find_loaded(filesystem, number)? {
            inode.wait_loaded()?;
            return Ok(inode);
        }
        if self.is_full() {
            // 回收工作不得同步等待自己的 Work；隐藏 EA 准备不足时保原删除义务并返回真 OOM。
            if _cache.is_none() {
                return Err(FsError::NoMemory);
            }
            self.prune_unused();
            if let Some(inode) = self.find_loaded(filesystem, number)? {
                inode.wait_loaded()?;
                return Ok(inode);
            }
            if self.is_full() {
                return Err(FsError::NoMemory);
            }
        }

        let candidate = InodeRef::try_new(filesystem, number, self)?;
        let mut loader = LoadGuard {
            domain: self,
            inode: &candidate,
            active: false,
        };
        let existing = self.publish(&candidate, &mut loader)?;
        if let Some(inode) = existing {
            drop(loader);
            drop(candidate);
            inode.wait_loaded()?;
            return Ok(inode);
        }

        #[cfg(feature = "ktest")]
        if let Some(cache) = _cache {
            cache.loads.fetch_add(1, Ordering::Relaxed);
            let hook = cache.load_hook.lock().clone();
            if let Some(hook) = hook {
                hook(&candidate);
            }
        }
        loader.complete(filesystem.load_inode(number, context));
        drop(loader);
        candidate.wait_loaded()?;
        Ok(candidate)
    }

    /// 在索引锁内判定身份与回收状态，登记后复查避免回收通知早于睡眠。
    fn find_loaded(
        self: &Arc<Self>,
        filesystem: &Arc<Ext4Volume>,
        number: u64,
    ) -> Result<Option<InodeRef>, FsError> {
        loop {
            let result = find(&mut self.index.lock(), self, filesystem, number);
            if result.as_ref().err() != Some(&FsError::Busy) {
                return result;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 强持固定域等待源，pin! 固定原栈节点，登记先于节点与 waiter 析构。
            let registration = unsafe { self.waiters.register(node.as_mut()) };
            let result = find(&mut self.index.lock(), self, filesystem, number);
            if result.as_ref().err() != Some(&FsError::Busy) {
                return result;
            }
            waiter.park();
            drop(registration);
        }
    }

    /// 每轮先在原索引锁内确认卷绑定仍存在，再双查同身份并发表 Loading；槽拥有不计外部使用。
    /// 已摘绑定返回 Busy；返回 None 后调用者必须完成原候选的结果发表。
    fn publish(
        self: &Arc<Self>,
        candidate: &InodeRef,
        loader: &mut LoadGuard<'_>,
    ) -> Result<Option<InodeRef>, FsError> {
        loop {
            let mut index = self.index.lock();
            if !index
                .volumes
                .iter()
                .any(|volume| Arc::ptr_eq(volume, &candidate.disk().filesystem))
            {
                return Err(FsError::Busy);
            }
            match find(
                &mut index,
                self,
                &candidate.disk().filesystem,
                candidate.number,
            ) {
                Ok(Some(inode)) => return Ok(Some(inode)),
                Ok(None) => {}
                Err(FsError::Busy) => {
                    drop(index);
                    if let Some(inode) =
                        self.find_loaded(&candidate.disk().filesystem, candidate.number)?
                    {
                        return Ok(Some(inode));
                    }
                    continue;
                }
                Err(error) => return Err(error),
            }

            let position = index
                .slots
                .iter()
                .position(Option::is_none)
                .ok_or(FsError::NoMemory)?;
            loader.active = true;
            candidate.disk().wb_slot.call_once(|| position);
            index.slots[position] = Some(InodeSlot {
                inode: candidate.0.clone(),
                users: 1,
                eviction: Eviction::Cached,
                root_dentry: Weak::new(),
            });
            return Ok(None);
        }
    }

    /// 已取得临时位图拥有的分配者在初始化新记录前等旧身份结束，不返回不可消费的新 token。
    pub(crate) fn wait_inode_reuse(
        &self,
        filesystem: &Ext4Volume,
        number: u64,
    ) -> Result<(), FsError> {
        sched::assert_switchable();
        if self.inode_reusable(filesystem, number)? {
            return Ok(());
        }
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 调用者的 Arc 保域等待源固定存活，栈节点固定且登记先于节点/waiter 析构。
            let registration = unsafe { self.waiters.register(node.as_mut()) };
            if self.inode_reusable(filesystem, number)? {
                return Ok(());
            }
            waiter.park();
            drop(registration);
        }
    }

    /// 原索引锁内区分正常退场、真实失败和非法的位图/cache 身份冲突，不增加外部使用。
    fn inode_reusable(&self, filesystem: &Ext4Volume, number: u64) -> Result<bool, FsError> {
        let index = self.index.lock();
        let Some(slot) = index.slots.iter().flatten().find(|slot| {
            slot.inode.number == number
                && core::ptr::eq(slot.inode.disk().filesystem.as_ref(), filesystem)
        }) else {
            return Ok(true);
        };
        match slot.eviction {
            Eviction::Requested | Eviction::Running => Ok(false),
            Eviction::Failed(error) => Err(error),
            Eviction::Cached => Err(FsError::Corrupt),
        }
    }

    /// 失败加载没有内容义务，核原对象后摘槽，缓存和等待者的最后析构都在短锁外。
    fn remove_failed(&self, inode: &InodeRef) {
        let victim = {
            let mut index = self.index.lock();
            let position = *inode
                .disk()
                .wb_slot
                .get()
                .expect("loaded inode has no canonical slot");
            if index.slots[position]
                .as_ref()
                .is_some_and(|slot| Arc::ptr_eq(&slot.inode, &inode.0))
            {
                index.slots[position].take()
            } else {
                None
            }
        };
        self.waiters.wake_all();
        drop(victim);
    }

    /// 普通 Task 将纯缓存拥有交给原 Work 并等真实尾部；失败保槽，不能伪报已经回收。
    fn prune_unused(self: &Arc<Self>) -> usize {
        sched::assert_switchable();
        let (before, requested) = {
            let mut index = self.index.lock();
            let before = index.evicted;
            let mut requested = false;
            for slot in index.slots.iter_mut().flatten() {
                if slot.users == 0
                    && slot.inode.data.get().is_some()
                    && slot.inode.mapping.is_cache_only()
                {
                    slot.eviction = Eviction::Requested;
                    requested = true;
                }
            }
            if requested {
                index.dispatching = index
                    .dispatching
                    .checked_add(1)
                    .expect("inode eviction dispatch overflow");
            }
            (before, requested)
        };

        if requested {
            self.queue();
            // 原 usize 接口只报告真正归还的槽；具体失败仍留在 slot、WB errseq 与观察出口。
            let _ = self.flush();
        }
        self.index.lock().evicted - before
    }

    /// Loading 和失败待回收对象均占固定槽，预算不足不能跳过未结束的原拥有。
    fn is_full(&self) -> bool {
        self.index.lock().slots.iter().all(Option::is_some)
    }

    /// 借用仍存活的 canonical 名字索引；不同预算不能改写现有有限容量。
    fn shared_dentry_cache(&self, capacity: usize) -> Result<Option<Arc<DentryCache>>, FsError> {
        let index = self.index.lock();
        let Some(cache) = index.dentry_cache.upgrade() else {
            return Ok(None);
        };
        if cache.capacity() != capacity {
            drop(index);
            drop(cache);
            return Err(FsError::InvalidOperation);
        }
        Ok(Some(cache))
    }

    /// 在 inode 索引短锁内仲裁共享名字索引和卷根，候选准备与输家析构均在锁外。
    fn publish_dentry_namespace(
        &self,
        candidate_cache: Arc<DentryCache>,
        root_inode: &InodeRef,
        candidate_root: DentryRef,
    ) -> Result<(Arc<DentryCache>, DentryRef), FsError> {
        let mut index = self.index.lock();
        let position = root_slot_position(&index, root_inode);
        let (cache, unused_cache) = match index.dentry_cache.upgrade() {
            Some(cache) => {
                if cache.capacity() != candidate_cache.capacity() {
                    drop(index);
                    drop(cache);
                    drop(candidate_cache);
                    drop(candidate_root);
                    return Err(FsError::InvalidOperation);
                }
                (cache, Some(candidate_cache))
            }
            None => {
                index.dentry_cache = Arc::downgrade(&candidate_cache);
                (candidate_cache, None)
            }
        };
        let (root, unused_root) = index.slots[position]
            .as_mut()
            .expect("canonical root inode slot disappeared")
            .bind_root(candidate_root);
        drop(index);
        drop(unused_cache);
        drop(unused_root);
        Ok((cache, root))
    }

    /// 读取卷根的弱身份登记；没有活根时由调用者锁外准备候选。
    fn canonical_root(&self, inode: &InodeRef) -> Option<DentryRef> {
        let index = self.index.lock();
        let slot = index.slots[root_slot_position(&index, inode)]
            .as_ref()
            .expect("canonical root inode slot disappeared");
        DentryRef::upgrade(&slot.root_dentry)
    }

    /// 在 canonical inode 索引锁内发表或复用根 dentry，未发表候选锁外释放。
    fn publish_root(&self, inode: &InodeRef, candidate: DentryRef) -> DentryRef {
        let mut index = self.index.lock();
        let position = root_slot_position(&index, inode);
        let slot = index.slots[position]
            .as_mut()
            .expect("canonical root inode slot disappeared");
        let (root, unused) = slot.bind_root(candidate);
        drop(index);
        drop(unused);
        root
    }
}

/// 外部引用保活实际后端；内存节点内部仅保存弱文件系统身份。
pub struct InodeRef(Arc<Inode>, InodeOwner);

#[derive(Clone)]
enum InodeOwner {
    Disk {
        domain: Arc<InodeDomain>,
        worker: bool,
    },
    Memory(Arc<MemoryFs>),
}

/// 页来源只登记真实后端弱身份，内存来源不得用于磁盘文件回收。
pub(crate) struct InodeWeak {
    inode: Weak<Inode>,
    owner: WeakInodeOwner,
}

enum WeakInodeOwner {
    Disk(Weak<InodeDomain>),
    Memory(Weak<MemoryFs>),
}

impl InodeWeak {
    /// 核对原 inode 及实际后端身份，不增加外部使用。
    pub(crate) fn matches(&self, inode: &InodeRef) -> bool {
        if self.inode.as_ptr() != Arc::as_ptr(&inode.0) {
            return false;
        }
        match (&self.owner, &inode.1) {
            (WeakInodeOwner::Disk(weak), InodeOwner::Disk { domain, .. }) => {
                weak.as_ptr() == Arc::as_ptr(domain)
            }
            (WeakInodeOwner::Memory(weak), InodeOwner::Memory(fs)) => {
                weak.as_ptr() == Arc::as_ptr(fs)
            }
            _ => false,
        }
    }

    /// 内存来源沿原文件系统升级；磁盘来源仍检查 canonical 槽与回收状态。
    pub(crate) fn upgrade(&self) -> Option<InodeRef> {
        let inode = self.inode.upgrade()?;
        match &self.owner {
            WeakInodeOwner::Memory(weak) => {
                let fs = weak.upgrade()?;
                Some(InodeRef(inode, InodeOwner::Memory(fs)))
            }
            WeakInodeOwner::Disk(weak) => {
                let domain = weak.upgrade()?;
                let position = *inode.disk().wb_slot.get()?;
                let mut index = domain.index.lock();
                if !index.cache_open {
                    return None;
                }
                let slot = index.slots.get_mut(position)?.as_mut()?;
                if !Arc::ptr_eq(&slot.inode, &inode)
                    || !matches!(slot.eviction, Eviction::Cached)
                    || !inode.data.get().is_some_and(Result::is_ok)
                {
                    return None;
                }
                let reference = use_slot(&domain, slot, false);
                drop(index);
                Some(reference)
            }
        }
    }
}

/// 先放真实 invalidate 读侧再恢复任务回收范围，借用期不能再次取得 FS 依赖。
pub(crate) struct InvalidateGuard<'a> {
    _guard: ReadGuard<'a, ()>,
    _scope: ReclaimScope,
}

/// 同一 FS 实例中的唯一运行对象；域拥有 inode，inode 不反向强持域。
pub struct Inode {
    backend: InodeBackend,
    number: u64,
    data: Once<Result<InodeState, FsError>>,
    mapping: FileMapping,
    waiters: WaitQueue,
    #[cfg(feature = "ktest")]
    truncate_hook: SpinLock<Option<TruncateHook>>,
    _pinned: PhantomPinned,
}

enum InodeBackend {
    Disk(DiskInode),
    Memory {
        filesystem: Weak<MemoryFs>,
        content: MemoryContent,
    },
}

/// 盘面生命周期义务全部由真实磁盘状态承担。
struct DiskInode {
    filesystem: Arc<Ext4Volume>,
    wb_slot: Once<usize>,
    dirty_obligation: AtomicBool,
    storage_released: AtomicBool,
}

pub(crate) enum MemoryContent {
    Regular,
    Directory(SpinLock<MemoryDirectory>),
    Symlink(Vec<u8>),
    Special(MemorySpecial),
}

pub(crate) struct MemoryDirectory {
    pub(crate) parent: u64,
    pub(crate) next_cookie: u64,
    pub(crate) entries: Vec<MemoryEntry>,
}

pub(crate) struct MemoryEntry {
    pub(crate) name: Arc<Vec<u8>>,
    pub(crate) inode: Arc<Inode>,
    pub(crate) kind: InodeKind,
    pub(crate) cookie: u64,
}

/// 停点只观察真实截断顺序，不改变用户 ABI 或后端结果。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum TruncatePhase {
    AfterFirstUnmap,
    BeforeSecondUnmap,
    BeforeBlockRelease,
    AfterStorageRelease,
}

#[cfg(feature = "ktest")]
type TruncateHook = Arc<dyn Fn(&InodeRef, TruncatePhase) + Send + Sync>;

/// 回收收据只保原域，不增加 inode、文件页或映射使用；正式关闭者可等全域真实退场。
pub(crate) struct InodeCacheWatch(Arc<InodeDomain>);

impl InodeCacheWatch {
    /// 等待调用边界已接受的 Work 全尾，仍有外部使用时不宣称其 inode 已回收。
    #[cfg(feature = "ktest")]
    pub(crate) fn wait(&self) -> Result<(), FsError> {
        self.0.flush()
    }

    /// cache owner 已退出后等待真实使用、卷停止、发表责任和回收 Work；失败继续强保并返回原错误。
    pub(crate) fn wait_retired(&self) -> Result<(), FsError> {
        sched::assert_switchable();
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 收据强持固定域等待源，栈节点固定，registration 先于 node/waiter 析构。
            let registration = unsafe { self.0.waiters.register(node.as_mut()) };
            let retired = {
                let index = self.0.index.lock();
                if index.cache_open {
                    return Err(FsError::InvalidOperation);
                }
                for slot in index.slots.iter().flatten() {
                    if let Eviction::Failed(error) = slot.eviction {
                        return Err(error);
                    }
                }
                index.dispatching == 0
                    && index.volumes.is_empty()
                    && index.slots.iter().all(Option::is_none)
            };
            if retired {
                drop(registration);
                // 摘除队列登记不会结束 Task 等待票，下一次 Work 等待前必须析构 Waiter。
                drop(waiter);
                return self.0.flush();
            }
            waiter.park();
            drop(registration);
        }
    }

    /// 只观察原 canonical 槽，不取得新的使用或改变待回收状态。
    #[cfg(feature = "ktest")]
    pub(crate) fn contains(&self, number: u64) -> bool {
        self.0
            .index
            .lock()
            .slots
            .iter()
            .flatten()
            .any(|slot| slot.inode.number == number)
    }
}

/// 有界属性快照不含块根；size 只在 canonical 属性锁内保存一份运行值。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InodeMetadata {
    pub number: u64,
    pub kind: InodeKind,
    pub mode: u16,
    pub size: u64,
    pub uid: u32,
    pub gid: u32,
    pub links: u32,
    pub blocks: u64,
    pub rdev: u64,
    pub atime: InodeTime,
    pub mtime: InodeTime,
    pub ctime: InodeTime,
    pub generation: u32,
    pub flags: u32,
    pub checksum_seed: u32,
    pub fast_symlink: bool,
    pub file_acl: u64,
    pub extra_isize: usize,
}

struct InodeState {
    write: SleepRwLock<()>,
    invalidate: SleepRwLock<()>,
    metadata: SpinLock<InodeMetadata>,
    storage: InodeStorage,
    writecount: SpinLock<isize>,
}

enum InodeStorage {
    Disk { root: SleepRwLock<[u8; 60]> },
    Memory,
}

/// inode 的唯一写序列化责任，属性和块根仍使用各自的短期保护。
pub(crate) struct InodeWriteGuard<'a> {
    inode: &'a InodeRef,
    state: &'a InodeState,
    _lock: WriteGuard<'a, ()>,
    _scope: ReclaimScope,
}

/// 目录读取 guard 同时排斥名字提交与旧目录加载快照。
pub(crate) struct DirectoryReadGuard<'a> {
    inode: &'a InodeRef,
    state: &'a InodeState,
    _lock: ReadGuard<'a, ()>,
    _scope: ReclaimScope,
}

/// 目录修改 guard 复用 canonical inode 的唯一写侧，不另建目录锁。
pub(crate) struct DirectoryWriteGuard<'a> {
    inode: &'a InodeRef,
    state: &'a InodeState,
    _lock: WriteGuard<'a, ()>,
    _scope: ReclaimScope,
}

/// 一项已经预留 inode-table 窗口且校验完字段的属性变更。
pub(crate) struct PreparedInodeMetadata {
    inode: InodeRef,
    storage: PreparedMetadataStorage,
    old: InodeMetadata,
    next: InodeMetadata,
}

enum PreparedMetadataStorage {
    Disk { record: BufferRef, offset: usize },
    Memory,
}

/// 目录内容和 inode-table 均已预备，Drop 只撤销尚未发表的目录块票。
pub(crate) struct PreparedDiskDirectoryMutation {
    parent: InodeRef,
    volume: Arc<Ext4Volume>,
    edit: Option<PreparedDirectoryEdit>,
    inode_mapping: Option<PreparedInodeMapping>,
    root: [u8; 60],
    next: InodeMetadata,
    changed: bool,
}

pub(crate) enum PreparedDirectoryMutation {
    Disk(PreparedDiskDirectoryMutation),
    Memory {
        parent: InodeRef,
        directory: Option<MemoryDirectory>,
        next: InodeMetadata,
    },
}

/// 新 inode 的三种真实磁盘初始内容。
#[derive(Clone, Copy)]
enum NewInodeContent<'a> {
    Regular,
    Directory,
    Symlink(&'a [u8]),
}

/// 名字出现前拥有 raw inode、canonical inode 与可提交的初始链接数。
pub(crate) struct PreparedDiskNewInode {
    volume: Arc<Ext4Volume>,
    allocation: Option<UnpublishedInode>,
    inode: Option<InodeRef>,
    initial: Option<InodeData>,
    record_initialized: bool,
    directory: Option<PreparedDirectoryEdit>,
    symlink: Option<PreparedSymlink>,
    link_update: Option<PreparedInodeMetadata>,
}

pub(crate) enum PreparedNewInode {
    Disk(PreparedDiskNewInode),
    Memory {
        inode: Option<InodeRef>,
        next: InodeMetadata,
    },
}

/// 长符号链接的真实块映射、目标内容和清理所需 metadata aliases。
struct PreparedSymlink {
    volume: Arc<Ext4Volume>,
    mappings: Option<PreparedDirectoryMappings>,
    buffers: Vec<BufferRef>,
    target: Vec<u8>,
    block_size: usize,
    root: [u8; 60],
    blocks: u64,
    publishing: bool,
}

impl PreparedSymlink {
    /// 预备连续逻辑块、唯一映射计划和干净数据窗口，目标仅在最终发表时写入。
    fn try_new(
        volume: &Arc<Ext4Volume>,
        inode: &InodeData,
        target: &[u8],
    ) -> Result<Self, FsError> {
        if target.len() < codec::INODE_BLOCK_BYTES
            || PAGE_SIZE < target.len()
            || target.is_empty()
            || target.contains(&0)
        {
            return Err(if PAGE_SIZE < target.len() {
                FsError::NameTooLong
            } else {
                FsError::InvalidName
            });
        }

        let block_size = volume.geometry().block_size;
        let count = target.len().div_ceil(block_size);
        let mut owned_target = Vec::new();
        owned_target
            .try_reserve_exact(target.len())
            .map_err(|_| FsError::NoMemory)?;
        owned_target.extend_from_slice(target);
        let mut logical = Vec::new();
        logical
            .try_reserve_exact(count)
            .map_err(|_| FsError::NoMemory)?;
        for block in 0..count {
            logical.push(block as u64);
        }
        let mut buffers = Vec::new();
        buffers
            .try_reserve_exact(count)
            .map_err(|_| FsError::NoMemory)?;

        let mappings = volume.prepare_symlink_mappings(inode, &inode.block, &logical)?;
        let root = *mappings.root();
        let mut prepared = Self {
            volume: Arc::clone(volume),
            mappings: Some(mappings),
            buffers,
            target: owned_target,
            block_size,
            root,
            blocks: inode.blocks,
            publishing: false,
        };
        let sectors = (block_size / SECTOR_BYTES as usize) as u64;
        let added = prepared
            .mappings
            .as_ref()
            .expect("prepared symlink mapping")
            .added_blocks()
            .checked_mul(sectors)
            .ok_or(FsError::FileTooLarge)?;
        prepared.blocks = inode
            .blocks
            .checked_add(added)
            .ok_or(FsError::FileTooLarge)?;

        for &logical in &logical {
            let physical = prepared
                .mappings
                .as_ref()
                .expect("prepared symlink mapping")
                .physical(logical)
                .ok_or(FsError::Corrupt)?;
            prepared.buffers.push(volume.metadata_block(physical)?);
        }
        Ok(prepared)
    }

    /// 把预先拥有的目标写入原 metadata aliases，再发表同一批 extent/indirect 映射。
    fn publish(mut self) {
        self.publishing = true;
        let mut offset = 0usize;
        for buffer in &self.buffers {
            let count = (self.target.len() - offset).min(self.block_size);
            let data = &self.target[offset..offset + count];
            buffer
                .update(self.volume.claim(), |bytes| {
                    bytes.fill(0);
                    bytes[..data.len()].copy_from_slice(data);
                })
                .expect("prepared writable symlink data block");
            offset += count;
        }

        self.mappings
            .take()
            .expect("prepared symlink mapping")
            .publish(&self.volume);
    }
}

impl Drop for PreparedSymlink {
    /// 目标尚未写入时先失效整批数据块的缓存窗口，再归还未发表映射。
    fn drop(&mut self) {
        if !self.publishing {
            let mappings = self
                .mappings
                .as_ref()
                .expect("unpublished symlink mapping absent");
            for logical in 0..self.target.len().div_ceil(self.block_size) {
                let physical = mappings
                    .physical(logical as u64)
                    .expect("unpublished symlink data mapping absent");
                let buffer = self
                    .volume
                    .metadata_cache()
                    .cached_buffer(physical, self.block_size)
                    .expect("unpublished symlink cache range invalid");
                let Some(buffer) = buffer else {
                    continue;
                };
                buffer.wait_writeback();
                // SAFETY: 该物理块仍由未发表 reservation 独占，inode 根尚未连接本批块；
                // metadata_block 返回或失败时读入责任已收回，且 wait_writeback 已结束原页写回。
                unsafe { buffer.forget(self.volume.claim()) }
                    .expect("unpublished symlink data alias invalidation");
            }

            self.mappings
                .take()
                .expect("unpublished symlink mapping absent")
                .cancel(&self.volume);
        }
    }
}

impl PreparedDiskNewInode {
    /// 借用已经完整格式化并进入本缓存域的未链接 canonical inode。
    pub(crate) fn inode(&self) -> &InodeRef {
        self.inode
            .as_ref()
            .expect("prepared inode canonical object absent")
    }

    /// 发表初始内容和 links 后把 bitmap 责任移交给已连接的 canonical inode。
    pub(crate) fn publish(mut self) -> InodeRef {
        self.publish_storage();
        let inode = self
            .inode
            .as_ref()
            .expect("prepared inode canonical object absent");
        let guard = inode
            .inode_write()
            .expect("prepared inode load remains valid");
        guard.publish_metadata(
            self.link_update
                .take()
                .expect("prepared inode link update absent"),
        );
        drop(guard);

        self.allocation
            .take()
            .expect("unpublished inode reservation absent")
            .publish();
        self.inode
            .take()
            .expect("prepared inode canonical object absent")
    }

    /// 先完成实际目录或符号链接块，再允许 inode links 和父名称发表。
    fn publish_storage(&mut self) {
        if let Some(directory) = self.directory.take() {
            let result = directory.publish(&self.volume);
            let (metadata, root) = self
                .inode()
                .with_map_read(|metadata, root| Ok((*metadata, *root)))
                .expect("prepared new directory remains loaded");
            assert_eq!(result.root, root, "new directory root publication mismatch");
            assert_eq!(
                result.size, metadata.size,
                "new directory size publication mismatch"
            );
            assert_eq!(
                result.blocks, metadata.blocks,
                "new directory block count publication mismatch"
            );
            assert_eq!(
                result.flags, metadata.flags,
                "new directory flags publication mismatch"
            );
        }
        if let Some(symlink) = self.symlink.take() {
            symlink.publish();
        }
    }
}

impl Drop for PreparedDiskNewInode {
    /// iget 前清理未发表记录；iget 后复用原 nlink=0 Work 回收真实存储。
    fn drop(&mut self) {
        if self.inode.is_some() {
            self.publish_storage();
            if let Some(allocation) = self.allocation.take() {
                allocation.publish();
            }
            return;
        }

        let Some(allocation) = self.allocation.take() else {
            return;
        };
        if self.record_initialized {
            self.volume.clear_unpublished_inode(
                &allocation,
                self.initial
                    .as_ref()
                    .expect("initialized unpublished inode lacks original record"),
            );
        }
        if let Some(directory) = self.directory.take() {
            directory.cancel(&self.volume);
        }
        drop(self.symlink.take());
        self.volume.release_unpublished_inode(allocation);
    }
}

impl InodeMetadata {
    /// 将盘面加载值拆为运行属性，块根单独进入睡眠读写保护。
    pub(crate) fn from_data(data: &InodeData) -> Self {
        Self {
            number: data.number,
            kind: data.kind,
            mode: data.mode,
            size: data.size,
            uid: data.uid,
            gid: data.gid,
            links: data.links,
            blocks: data.blocks,
            rdev: data.rdev,
            atime: data.atime,
            mtime: data.mtime,
            ctime: data.ctime,
            generation: data.generation,
            flags: data.flags,
            checksum_seed: data.checksum_seed,
            fast_symlink: data.fast_symlink,
            file_acl: data.file_acl,
            extra_isize: data.extra_isize,
        }
    }

    /// 仅为当次真实块映射查询组装 codec 输入，不保存第二份可变状态。
    pub(crate) fn with_block(&self, block: &[u8; 60]) -> InodeData {
        InodeData {
            number: self.number,
            kind: self.kind,
            mode: self.mode,
            size: self.size,
            uid: self.uid,
            gid: self.gid,
            links: self.links,
            blocks: self.blocks,
            rdev: self.rdev,
            atime: self.atime,
            mtime: self.mtime,
            ctime: self.ctime,
            generation: self.generation,
            flags: self.flags,
            checksum_seed: self.checksum_seed,
            fast_symlink: self.fast_symlink,
            file_acl: self.file_acl,
            extra_isize: self.extra_isize,
            block: *block,
        }
    }
}

impl InodeState {
    /// 完成真实加载后一次安装同步容器，初始化不执行额外 IO。
    fn new(data: InodeData) -> Self {
        Self {
            write: SleepRwLock::new(()),
            invalidate: SleepRwLock::new(()),
            metadata: SpinLock::new(InodeMetadata::from_data(&data)),
            storage: InodeStorage::Disk {
                root: SleepRwLock::new(data.block),
            },
            writecount: SpinLock::new(0),
        }
    }

    /// 内存节点只准备共同同步状态，不制造块树根。
    fn memory(metadata: InodeMetadata) -> Self {
        Self {
            write: SleepRwLock::new(()),
            invalidate: SleepRwLock::new(()),
            metadata: SpinLock::new(metadata),
            storage: InodeStorage::Memory,
            writecount: SpinLock::new(0),
        }
    }

    /// 已选择磁盘分支的内部协议借用真实块树锁。
    fn disk_root(&self) -> &SleepRwLock<[u8; 60]> {
        match &self.storage {
            InodeStorage::Disk { root } => root,
            InodeStorage::Memory => panic!("memory inode has no disk root"),
        }
    }
}

/// 锁外准备完成的同源页及当时事实，不保留 inode、块根或内容锁的 guard。
pub(crate) struct PreparedMkwrite {
    folio: FolioRef,
    index: u64,
    size: u64,
    prepared_sectors: u8,
}

impl PreparedMkwrite {
    /// 借用原 canonical 页，最终授权必须重核该页仍属于当前 inode 索引。
    pub(crate) fn folio(&self) -> &FolioRef {
        &self.folio
    }
}

/// 尾零的所有可失败准备在 size 修改前完成，只保原页及真实 Mapped 块位。
struct PreparedTail {
    folio: FolioRef,
    index: u64,
    start: usize,
    end: usize,
    mapped: u8,
}

/// 真正写打开持有的 inode 责任；dup 共享 OFD 不复制本 token。
pub struct WriteAccess {
    inode: InodeRef,
}

/// 每个活执行地址空间保有独立 deny 责任，最后实际映像使用结束才释放。
pub struct ExecAccess {
    inode: InodeRef,
}

impl WriteAccess {
    /// 写能力始终绑定取得时的 canonical inode，不能借给其他文件。
    pub fn write_from(
        &self,
        offset: u64,
        append: bool,
        source: &mut WriteBuffer<'_>,
    ) -> Result<(u64, usize), FsError> {
        self.inode.write_backend(offset, append, source)
    }

    /// 三个外部截断入口共用原写能力和同一实现，不取得或修改 OFD 位置。
    pub fn truncate(&self, new_size: u64) -> Result<(), FsError> {
        self.inode.truncate_backend(new_size)
    }
}

impl Drop for WriteAccess {
    /// 只归还原 inode 的一次写拥有，计数短锁内不分配或等待。
    fn drop(&mut self) {
        let mut count = self.inode.loaded_state().writecount.lock();
        assert!(0 < *count, "write access count lost");
        *count -= 1;
    }
}

impl Clone for ExecAccess {
    /// fork 增加同 inode 的独立映像拥有，活 deny 期间不可能出现正写计数。
    fn clone(&self) -> Self {
        let mut count = self.inode.loaded_state().writecount.lock();
        assert!(*count < 0, "exec access count lost");
        *count = count.checked_sub(1).expect("exec access count overflow");
        Self {
            inode: self.inode.clone(),
        }
    }
}

impl Drop for ExecAccess {
    /// 地址空间最后退出时归还本次 deny，不能替其他映像释放。
    fn drop(&mut self) {
        let mut count = self.inode.loaded_state().writecount.lock();
        assert!(*count < 0, "exec access count lost");
        *count += 1;
    }
}

/// 只有胜出的候选取得此责任；提前退出发布取消结果并解除自己的占位。
struct LoadGuard<'a> {
    domain: &'a Arc<InodeDomain>,
    inode: &'a InodeRef,
    active: bool,
}

impl InodeCache {
    /// 准备独立的有限 canonical 域和具体回收 Work，不预留每文件固定页槽。
    pub fn try_new(inode_capacity: usize) -> Result<Self, FsError> {
        if inode_capacity == 0 {
            return Err(FsError::InvalidOperation);
        }
        Ok(Self {
            domain: InodeDomain::try_new(inode_capacity)?,
            #[cfg(feature = "ktest")]
            load_hook: SpinLock::new(None),
            #[cfg(feature = "ktest")]
            loads: AtomicUsize::new(0),
        })
    }

    /// 名字查找与内部 EA 取得共享同一 canonical 域，真实加载和回收等待均在短锁外。
    pub fn iget(&self, filesystem: &Arc<Ext4Volume>, number: u64) -> Result<InodeRef, FsError> {
        self.domain.iget(filesystem, number, Some(self), None)
    }

    /// 为实际卷取得 canonical 根与共享名字索引；活域保原有限容量，失去最后索引拥有后可重新选预算。
    pub(crate) fn dentry_namespace(
        &self,
        volume: &Arc<Ext4Volume>,
        capacity: usize,
    ) -> Result<(Arc<DentryCache>, DentryRef), FsError> {
        if let Some(cache) = self.domain.shared_dentry_cache(capacity)? {
            let root = self.root_dentry(volume)?;
            return Ok((cache, root));
        }
        let candidate_cache = DentryCache::try_new(capacity)?;
        let root_inode = self.iget(volume, codec::ROOT_INODE)?;
        let candidate_root = DentryRef::try_root(root_inode.clone())?;
        self.domain
            .publish_dentry_namespace(candidate_cache, &root_inode, candidate_root)
    }

    /// 从 canonical inode 槽弱登记中复用卷根，候选在索引锁外准备。
    pub(crate) fn root_dentry(&self, volume: &Arc<Ext4Volume>) -> Result<DentryRef, FsError> {
        let inode = self.iget(volume, codec::ROOT_INODE)?;
        if let Some(root) = self.domain.canonical_root(&inode) {
            return Ok(root);
        }
        let candidate = DentryRef::try_root(inode.clone())?;
        Ok(self.domain.publish_root(&inode, candidate))
    }

    /// 准备真正链接前的普通文件记录和 canonical 对象。
    #[expect(
        clippy::too_many_arguments,
        reason = "父属性、创建模式、umask、身份和时间是同一次普通文件创建的独立快照"
    )]
    pub(crate) fn prepare_regular_inode(
        &self,
        owner: &InodeRef,
        parent: &InodeMetadata,
        mode: u16,
        umask: u16,
        uid: u32,
        gid: u32,
        now_ns: u64,
    ) -> Result<PreparedNewInode, FsError> {
        if owner.is_memory() {
            return owner.prepare_memory_inode(
                parent,
                NewInodeContent::Regular,
                mode,
                umask,
                uid,
                gid,
                now_ns,
            );
        }
        self.prepare_new_inode(
            owner.disk_volume()?,
            parent,
            NewInodeContent::Regular,
            mode,
            umask,
            uid,
            gid,
            now_ns,
        )
        .map(PreparedNewInode::Disk)
    }

    /// 先构造真实点目录与父链接记录，再准备尚未连接的目录 inode。
    #[expect(
        clippy::too_many_arguments,
        reason = "目录创建直接消费父属性、模式、umask、身份和时间并准备点目录与父链接"
    )]
    pub(crate) fn prepare_directory_inode(
        &self,
        owner: &InodeRef,
        parent: &InodeMetadata,
        mode: u16,
        umask: u16,
        uid: u32,
        gid: u32,
        now_ns: u64,
    ) -> Result<PreparedNewInode, FsError> {
        if owner.is_memory() {
            return owner.prepare_memory_inode(
                parent,
                NewInodeContent::Directory,
                mode,
                umask,
                uid,
                gid,
                now_ns,
            );
        }
        self.prepare_new_inode(
            owner.disk_volume()?,
            parent,
            NewInodeContent::Directory,
            mode,
            umask,
            uid,
            gid,
            now_ns,
        )
        .map(PreparedNewInode::Disk)
    }

    /// 以原始目标字节准备 fast 或块存储符号链接，不应用进程 umask。
    pub(crate) fn prepare_symlink_inode(
        &self,
        owner: &InodeRef,
        parent: &InodeMetadata,
        target: &[u8],
        uid: u32,
        gid: u32,
        now_ns: u64,
    ) -> Result<PreparedNewInode, FsError> {
        if owner.is_memory() {
            return owner.prepare_memory_inode(
                parent,
                NewInodeContent::Symlink(target),
                0,
                0,
                uid,
                gid,
                now_ns,
            );
        }
        self.prepare_new_inode(
            owner.disk_volume()?,
            parent,
            NewInodeContent::Symlink(target),
            0,
            0,
            uid,
            gid,
            now_ns,
        )
        .map(PreparedNewInode::Disk)
    }

    /// 分配 inode 后完成块映射或内嵌链接格式及 canonical 加载；返回对象仍为 links=0。
    #[expect(
        clippy::too_many_arguments,
        reason = "三类新 inode 共用一次分配和可失败准备，原始创建快照直接参与记录构造"
    )]
    fn prepare_new_inode(
        &self,
        volume: &Arc<Ext4Volume>,
        parent: &InodeMetadata,
        content: NewInodeContent<'_>,
        mode: u16,
        umask: u16,
        uid: u32,
        gid: u32,
        now_ns: u64,
    ) -> Result<PreparedDiskNewInode, FsError> {
        let (kind, link_count) = match content {
            NewInodeContent::Regular => (InodeKind::Regular, 1),
            NewInodeContent::Directory => (InodeKind::Directory, 2),
            NewInodeContent::Symlink(target) => {
                if target.is_empty() || target.contains(&0) {
                    return Err(FsError::InvalidName);
                }
                if PAGE_SIZE < target.len() {
                    return Err(FsError::NameTooLong);
                }
                (InodeKind::Symlink, 1)
            }
        };

        let allocation = volume.allocate_inode_for_parent(parent, kind, mode, umask, uid, gid)?;
        let number = allocation.number();
        let mut prepared = PreparedDiskNewInode {
            volume: Arc::clone(volume),
            allocation: Some(allocation),
            inode: None,
            initial: None,
            record_initialized: false,
            directory: None,
            symlink: None,
            link_update: None,
        };
        let initial = volume.load_inode(number, None)?;
        prepared.initial = Some(initial);
        let initial = prepared.initial.as_ref().expect("initial inode loaded");
        let mut root = initial.block;
        let mut size = 0u64;
        let mut blocks = initial.blocks;
        let mut directory_flags = None;
        match content {
            NewInodeContent::Regular => {}
            NewInodeContent::Directory => {
                let metadata = InodeMetadata::from_data(initial);
                let edit = volume.prepare_directory_new(&metadata, &root, parent.number)?;
                root = *edit.root();
                size = edit.size();
                blocks = edit.blocks();
                directory_flags = Some(edit.flags());
                prepared.directory = Some(edit);
            }
            NewInodeContent::Symlink(target) if target.len() < codec::INODE_BLOCK_BYTES => {
                root.fill(0);
                root[..target.len()].copy_from_slice(target);
                size = target.len() as u64;
            }
            NewInodeContent::Symlink(target) => {
                let symlink = PreparedSymlink::try_new(volume, initial, target)?;
                root = symlink.root;
                size = target.len() as u64;
                blocks = symlink.blocks;
                prepared.symlink = Some(symlink);
            }
        }

        let mut next = initial_inode_metadata(volume, initial, size, blocks, now_ns)?;
        if next.fast_symlink {
            next.flags &= !codec::EXTENTS;
        }
        if let Some(flags) = directory_flags {
            let permitted = codec::INDEX | codec::HUGE_FILE;
            if flags & !permitted != initial.flags & !permitted {
                return Err(FsError::InvalidOperation);
            }
            let (_, huge) = codec::blocks_encoding(volume.geometry(), next.blocks)?;
            next.flags = (flags & !codec::HUGE_FILE) | if huge { codec::HUGE_FILE } else { 0 };
        }

        volume.initialize_unpublished_inode(
            prepared
                .allocation
                .as_ref()
                .expect("inode reservation absent"),
            initial,
            &next,
            &root,
        )?;
        prepared.record_initialized = true;

        prepared.inode = Some(self.iget(volume, number)?);
        let update = {
            let guard = prepared.inode().inode_write()?;
            guard.prepare_link_delta(link_count, now_ns)?
        };
        prepared.link_update = Some(update);
        Ok(prepared)
    }

    /// 把 cache-only 页和真实脏义务交给原 pool 中的 Task，返回已经结束回收的槽数。
    pub fn prune_unused(&self) -> usize {
        self.domain.prune_unused()
    }

    /// 普通 Task 在显式卸载回收完成后，请原卷同步退休本缓存域的无槽绑定。
    pub(crate) fn retire_volume(&self, volume: &Arc<Ext4Volume>) {
        volume.retire_inode_domain(&self.domain);
    }

    /// 返回 Loading、缓存以及待回收对象共同占用的槽数。
    #[cfg(feature = "ktest")]
    pub fn len(&self) -> usize {
        self.domain.index.lock().slots.iter().flatten().count()
    }

    /// 观察本缓存域是否没有已发表的 inode。
    #[cfg(feature = "ktest")]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 名字索引先释放静止绑定，真实 inode 回收完成后才可重用 canonical 预算。
    pub(crate) fn is_full(&self) -> bool {
        self.domain.is_full()
    }

    /// 在真实 Loading 发表之后、读取 inode 之前运行锁外停点；旧回调锁外析构。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_load_hook(&self, hook: Option<LoadHook>) {
        let old = core::mem::replace(&mut *self.load_hook.lock(), hook);
        drop(old);
    }

    /// 返回真正取得加载责任的次数，不计命中与私有竞争失败候选。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_load_count(&self) -> usize {
        self.loads.load(Ordering::Relaxed)
    }

    /// 只观察原 Loading 的等待源，不增加外部使用或改变回收状态。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_waiting(&self, filesystem: &Arc<Ext4Volume>, number: u64) -> usize {
        let index = self.domain.index.lock();
        index
            .slots
            .iter()
            .flatten()
            .find(|slot| {
                slot.inode.number == number
                    && Arc::ptr_eq(&slot.inode.disk().filesystem, filesystem)
            })
            .map_or(0, |slot| slot.inode.waiters.test_members())
    }

    /// 取得正式关闭收据；收据保所属索引，外部使用和缓存 owner 的退出仍各自结束。
    pub(crate) fn watch(&self) -> InodeCacheWatch {
        InodeCacheWatch(self.domain.clone())
    }
}

impl Drop for InodeCache {
    /// cache owner 退出只交接原槽和卷绑定；空域的回写停止也由预建 Work 在 Task 中完成。
    fn drop(&mut self) {
        let requested = {
            let mut index = self.domain.index.lock();
            index.cache_open = false;
            let requested = !index.volumes.is_empty();
            for slot in index.slots.iter_mut().flatten() {
                if matches!(slot.eviction, Eviction::Cached | Eviction::Requested) {
                    slot.eviction = Eviction::Requested;
                }
            }
            if requested {
                index.dispatching = index
                    .dispatching
                    .checked_add(1)
                    .expect("inode eviction dispatch overflow");
            }
            requested
        };
        if requested {
            self.domain.queue();
        }
    }
}

impl Clone for InodeRef {
    /// 复制真实外部使用，在同一 canonical 锁内与末使用和回收取得互斥。
    fn clone(&self) -> Self {
        let InodeOwner::Disk { domain, .. } = &self.1 else {
            return Self(self.0.clone(), self.1.clone());
        };
        if let Some(position) = self.disk().wb_slot.get().copied() {
            let mut index = domain.index.lock();
            if let Some(slot) = index.slots[position].as_mut() {
                if Arc::ptr_eq(&slot.inode, &self.0) {
                    slot.users = slot.users.checked_add(1).expect("inode use count overflow");
                }
            }
        }
        Self(
            self.0.clone(),
            InodeOwner::Disk {
                domain: domain.clone(),
                worker: false,
            },
        )
    }
}

impl Drop for InodeRef {
    /// 最后使用在原索引锁内转交回收责任，短锁和 IRQ Drop 都不等待设备或释放盘块。
    fn drop(&mut self) {
        let InodeOwner::Disk { domain, worker } = &self.1 else {
            return;
        };
        let Some(position) = self.disk().wb_slot.get().copied() else {
            return;
        };
        let requested = {
            let mut index = domain.index.lock();
            let cache_open = index.cache_open;
            let Some(slot) = index.slots[position].as_mut() else {
                return;
            };
            if !Arc::ptr_eq(&slot.inode, &self.0) {
                return;
            }
            assert!(0 < slot.users, "inode use count lost");
            slot.users -= 1;
            let delete = slot.inode.data.get().is_some_and(|result| {
                result
                    .as_ref()
                    .is_ok_and(|state| state.metadata.lock().links == 0)
            });
            let wanted = !cache_open
                || delete
                || matches!(slot.eviction, Eviction::Requested)
                || !*worker && matches!(slot.eviction, Eviction::Failed(_));
            if wanted
                && slot.users == 0
                && !matches!(slot.eviction, Eviction::Running)
                && (!*worker || !matches!(slot.eviction, Eviction::Failed(_)))
                && slot.inode.mapping.is_cache_only()
            {
                slot.eviction = Eviction::Requested;
                index.dispatching = index
                    .dispatching
                    .checked_add(1)
                    .expect("inode eviction dispatch overflow");
                true
            } else {
                false
            }
        };
        if requested {
            domain.queue();
        }
    }
}

impl InodeRef {
    /// 只 downgrade 当前原 Arc 身份，页索引不强持 inode/domain 或创建新的加载责任。
    pub(crate) fn downgrade_for_reclaim(&self) -> InodeWeak {
        InodeWeak {
            inode: Arc::downgrade(&self.0),
            owner: match &self.1 {
                InodeOwner::Disk { domain, .. } => WeakInodeOwner::Disk(Arc::downgrade(domain)),
                InodeOwner::Memory(fs) => WeakInodeOwner::Memory(Arc::downgrade(fs)),
            },
        }
    }

    /// 普通清洁 file 回收取得独占失效侧；NOFS/NOIO 路径不得调用本入口。
    pub(crate) fn lock_reclaim_invalidate(&self) -> Result<WriteGuard<'_, ()>, FsError> {
        Ok(self.wait_loaded()?.invalidate.write())
    }

    /// 锁外准备尚未发表的稳定对象与唯一 mapping，任何分配失败均无索引占位。
    fn try_new(
        filesystem: &Arc<Ext4Volume>,
        number: u64,
        domain: &Arc<InodeDomain>,
    ) -> Result<Self, FsError> {
        let mapping = FileMapping::new();
        let inode = Arc::try_new(Inode {
            backend: InodeBackend::Disk(DiskInode {
                filesystem: Arc::clone(filesystem),
                wb_slot: Once::new(),
                dirty_obligation: AtomicBool::new(false),
                storage_released: AtomicBool::new(false),
            }),
            #[cfg(feature = "ktest")]
            truncate_hook: SpinLock::new(None),
            number,
            data: Once::new(),
            mapping,
            waiters: WaitQueue::new(),
            _pinned: PhantomPinned,
        })
        .map_err(|_| FsError::NoMemory)?;
        Ok(Self(
            inode,
            InodeOwner::Disk {
                domain: domain.clone(),
                worker: false,
            },
        ))
    }

    /// 比较原对象身份，不以相同卷号、元数据内容或盘上 inode 号代替实例身份。
    pub fn ptr_eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    /// 普通 inode 属性或链接变化取得 canonical 唯一写侧。
    pub(crate) fn inode_write(&self) -> Result<InodeWriteGuard<'_>, FsError> {
        let state = self.wait_loaded()?;
        let scope = ReclaimScope::nofs();
        let lock = state.write.write();
        Ok(InodeWriteGuard {
            inode: self,
            state,
            _lock: lock,
            _scope: scope,
        })
    }

    /// 目录读取 guard 覆盖名字缓存查询、后端加载和等待者读取首次结果。
    pub(crate) fn directory_read(&self) -> Result<DirectoryReadGuard<'_>, FsError> {
        let state = self.wait_loaded()?;
        let scope = ReclaimScope::nofs();
        let lock = state.write.read();
        if state.metadata.lock().kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        Ok(DirectoryReadGuard {
            inode: self,
            state,
            _lock: lock,
            _scope: scope,
        })
    }

    /// 目录修改 guard 复用 inode 写侧并让持锁重查不递归取得读锁。
    pub(crate) fn directory_write(&self) -> Result<DirectoryWriteGuard<'_>, FsError> {
        let state = self.wait_loaded()?;
        let scope = ReclaimScope::nofs();
        let lock = state.write.write();
        if state.metadata.lock().kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        Ok(DirectoryWriteGuard {
            inode: self,
            state,
            _lock: lock,
            _scope: scope,
        })
    }

    /// 普通名字查找通过目录读侧返回首个结果，缺失仍由调用者收到 NotFound。
    #[cfg(feature = "ktest")]
    pub fn lookup(&self, cache: &InodeCache, name: &[u8]) -> Result<InodeRef, FsError> {
        self.directory_read()?
            .lookup(cache, name)?
            .ok_or(FsError::NotFound)
    }

    /// 用户游标按当前目录格式重置碰撞续读状态。
    pub(crate) fn directory_cursor(&self, cookie: u64) -> Result<DirectoryCursor, FsError> {
        self.directory_read()?.cursor_at(cookie)
    }

    /// 返回普通字节目录或 HTree 各自定义的目录结尾 cookie。
    pub(crate) fn directory_end_cookie(&self) -> Result<u64, FsError> {
        self.directory_read()?.end_cookie()
    }

    /// 取下一真实目录项，查询全程持目录读侧。
    pub fn dir_next(&self, cursor: &DirectoryCursor) -> Result<Option<DirectoryEntry>, FsError> {
        self.directory_read()?.dir_next(cursor)
    }

    /// 在 invalidate 与页内容协议内先发表 inode 义务并请求卷进度，再使原页 Dirty 可见。
    pub(crate) fn note_dirty(&self) {
        let disk = self.disk();
        disk.dirty_obligation.store(true, Ordering::Release);
        disk.filesystem
            .writeback_domain()
            .notify_dirty(&disk.filesystem);
    }

    /// 数据回写已放开 WB 互斥，按 invalidate→WB 次序排除新写授权并重核原页义务。
    pub(crate) fn refresh_dirty_obligation(&self) -> Result<(), FsError> {
        let _invalidate = self.wait_loaded()?.invalidate.write();
        let _writeback = self.mapping.lock_writeback();
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// 调用者持 invalidate 独占和本 mapping WB 互斥，范围外 Dirty 和真实 WB 都继续保留。
    fn refresh_dirty_obligation_locked(&self) {
        self.disk().dirty_obligation.store(false, Ordering::Release);
        if self.mapping.has_dirty_or_writeback() {
            self.note_dirty();
        }
    }

    /// 同一短计数锁线性化写打开与 exec deny；隐藏 EA 的 ctime 承载引用计数，禁止普通写修改。
    pub fn acquire_write(&self) -> Result<WriteAccess, FsError> {
        let state = self.wait_loaded()?;
        if state.metadata.lock().flags & codec::EA_INODE != 0 {
            return Err(FsError::InvalidOperation);
        }

        let mut count = state.writecount.lock();
        if *count < 0 {
            return Err(FsError::TextBusy);
        }
        *count = count.checked_add(1).expect("write access count overflow");
        Ok(WriteAccess {
            inode: self.clone(),
        })
    }

    /// 第一次映像读取前取得 deny，任何真正写 owner 活时明确 TextBusy。
    pub fn deny_writes(&self) -> Result<ExecAccess, FsError> {
        let state = self.wait_loaded()?;
        let mut count = state.writecount.lock();
        if 0 < *count {
            return Err(FsError::TextBusy);
        }
        *count = count.checked_sub(1).expect("exec access count overflow");
        Ok(ExecAccess {
            inode: self.clone(),
        })
    }

    /// 类型、权限来源和尺寸均在修改前检查；已取得的写 token 排斥所有执行使用。
    fn truncate(&self, claim: &FsClaim, new_size: u64) -> Result<(), FsError> {
        self.disk_volume()?.require_write(claim)?;
        let state = self.wait_loaded()?;
        let mut attempt = ReclaimAttempt::new();
        let _scope = ReclaimScope::nofs();
        let _write = state.write.write();
        let old = *state.metadata.lock();
        if old.kind != InodeKind::Regular || old.flags & codec::EA_INODE != 0 {
            return Err(FsError::InvalidOperation);
        }
        if write_limit(self.disk_volume()?.geometry(), old.flags) < new_size {
            return Err(FsError::FileTooLarge);
        }
        assert!(0 < *state.writecount.lock(), "truncate has no write access");
        let _invalidate = state.invalidate.write();
        self.truncate_locked(state, claim, new_size, false, &mut attempt, None)
    }

    /// 写序列化和 invalidate 独占来自调用者，先发表尺寸再处理旧页和真实 WB，最后摘树。
    /// 页清理失败仍做第二遍撤叶，保新尺寸、原块根和未消费的删除义务，不回滚已发表事实。
    fn truncate_locked(
        &self,
        state: &InodeState,
        claim: &FsClaim,
        new_size: u64,
        deleting: bool,
        attempt: &mut ReclaimAttempt,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<(), FsError> {
        let volume = self.disk_volume()?;
        let g = volume.geometry();
        let old = *state.metadata.lock();
        let now = time::realtime();
        if deleting && old.kind == InodeKind::Symlink && old.fast_symlink {
            return self.truncate_fast_symlink_locked(state, claim, &old, now, context);
        }

        let mut size_next = write_metadata(g, &old, 0, 0, 0, 0, now)?;
        size_next.size = new_size;
        let tail = if old.size < new_size {
            self.prepare_tail(old.size, new_size, attempt, context)?
        } else if new_size % g.block_size as u64 != 0 {
            self.prepare_tail(
                new_size,
                new_size.div_ceil(g.block_size as u64) * g.block_size as u64,
                attempt,
                context,
            )?
        } else {
            None
        };
        // 块边界上的部分页无需读冷页；已有错误尾页必须在尺寸发表前解释。
        if new_size <= old.size
            && new_size % g.block_size as u64 == 0
            && new_size % PAGE_SIZE as u64 != 0
        {
            if let Some(folio) = self.mapping.cached_folio(new_size / PAGE_SIZE as u64) {
                folio.lock_content().range_ready(0, PAGE_SIZE)?;
            }
        }

        // EA 共享引用与所有元数据 WB 使用同一串行保护，隐藏值准备也在父尺寸修改之前。
        let _metadata_writeback = volume.metadata_cache().lock_writeback();
        let (record, record_offset) = volume.inode_record(old.number, context)?;
        let (prepared, inode_release, old_root) = {
            let root = state.disk_root().write();
            let current = record.with_read(|bytes| {
                codec::decode_inode(
                    g,
                    old.number,
                    &bytes[record_offset..record_offset + g.inode_size],
                )
            })??;
            if InodeMetadata::from_data(&current) != old || current.block != *root {
                return Err(FsError::Corrupt);
            }
            let prepared = volume.prepare_truncate(&old, &root, new_size, context)?;
            let inode_release: Option<PreparedInodeRelease> = if deleting {
                Some(volume.prepare_inode_release(&current, context)?)
            } else {
                None
            };
            (prepared, inode_release, *root)
        };

        let mut next = write_metadata(g, &old, 0, 0, 0, prepared.removed_blocks, now)?;
        next.size = new_size;
        record
            .update(claim, |bytes| {
                codec::patch_inode_mapping(
                    g,
                    &mut bytes[record_offset..record_offset + g.inode_size],
                    &size_next,
                    &old_root,
                )
            })
            .expect("prepared truncate size record");
        *state.metadata.lock() = size_next;

        let first = new_size.div_ceil(PAGE_SIZE as u64) * PAGE_SIZE as u64;
        self.mapping.unmap_range(first..u64::MAX, true);
        #[cfg(feature = "ktest")]
        self.truncate_phase(TruncatePhase::AfterFirstUnmap);

        // 块根已放开；旧数据回写可准备地址和完成全尾，不能带 root 等 WB 互斥。
        let _writeback = self.mapping.lock_writeback();
        if let Some(tail) = &tail {
            tail.folio.wait_writeback();
            self.apply_tail(tail);
        }
        self.note_dirty();
        // SAFETY: 同 inode 写序列化、invalidate 独占和 mapping WB 互斥均仍持有；
        // 新 size 已发表，首遍完整尾范围撤叶/CSD 已结束；普通读和新填充受 invalidate 排斥。
        // 旧唯一填充/WB 由原页等待，root 不跨数据等待；错误后也必须执行下一遍 even_cows。
        let pages = unsafe { self.mapping.truncate_pages(self, new_size, g.block_size) };
        #[cfg(feature = "ktest")]
        self.truncate_phase(TruncatePhase::BeforeSecondUnmap);
        self.mapping.unmap_range(first..u64::MAX, true);
        pages?;

        let mut root = state.disk_root().write();
        assert_eq!(
            *root, old_root,
            "truncate block root changed under write serialization"
        );
        let next_root = prepared.root;
        let retired = prepared.publish(volume, old.checksum_seed);
        record
            .update(claim, |bytes| {
                codec::patch_inode_mapping(
                    g,
                    &mut bytes[record_offset..record_offset + g.inode_size],
                    &next,
                    &next_root,
                )
            })
            .expect("prepared truncate mapping record");
        *root = next_root;
        *state.metadata.lock() = next;
        #[cfg(feature = "ktest")]
        self.truncate_phase(TruncatePhase::BeforeBlockRelease);

        for block in retired {
            // SAFETY: 两遍撤叶/CSD、文件页清理和全部旧数据 WB 已结束；
            // root 写侧已排空旧块查询，元数据 WB 串行排空并阻止旧窗口新提交；
            // 原树已经摘离该块，具体 token 保释放窗口，forget 完成后才允许位图复用。
            unsafe { volume.release_detached_block(block) };
        }
        drop(root);
        drop(tail);

        if let Some(inode_release) = inode_release {
            // SAFETY: 仅真实末使用回收进入 deleting，canonical 槽处于 Running 且没有外部使用；
            // root 写侧已排空后放开，原页和所有旧 data/metadata IO、别名均结束；
            // 元数据 WB 串行仍阻止新提交，EA 原身份和记录由准备窗口保活，inode 位图最后释放。
            unsafe { volume.release_inode(inode_release) };
            self.disk().storage_released.store(true, Ordering::Release);
        }
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// 释放 fast symlink 时只清 inode size/EA 义务，不把 inline target 当块树。
    fn truncate_fast_symlink_locked(
        &self,
        state: &InodeState,
        claim: &FsClaim,
        old: &InodeMetadata,
        now_ns: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<(), FsError> {
        let volume = self.disk_volume()?;
        let g = volume.geometry();
        let mut next = write_metadata(g, old, 0, 0, 0, 0, now_ns)?;
        next.size = 0;
        let _metadata_writeback = volume.metadata_cache().lock_writeback();
        let (record, offset) = volume.inode_record(old.number, context)?;
        let release = {
            let root = state.disk_root().write();
            let current = record.with_read(|bytes| {
                codec::decode_inode(g, old.number, &bytes[offset..offset + g.inode_size])
            })??;
            if InodeMetadata::from_data(&current) != *old || current.block != *root {
                return Err(FsError::Corrupt);
            }
            let release = volume.prepare_inode_release(&current, context)?;
            record
                .update(claim, |bytes| {
                    codec::patch_inode_mapping(
                        g,
                        &mut bytes[offset..offset + g.inode_size],
                        &next,
                        &root,
                    )
                })
                .expect("prepared fast symlink size record");
            *state.metadata.lock() = next;
            release
        };

        self.mapping.unmap_range(0..u64::MAX, true);
        let _writeback = self.mapping.lock_writeback();
        // SAFETY: inode.write 与 invalidate 独占仍由末次使用者持有，首轮完整撤叶/CSD 已结束；
        // symlink target 从不进入 FileMapping，mapping 仅清理意外保留的旧页并等待真实 I/O 尾。
        let pages = unsafe { self.mapping.truncate_pages(self, 0, g.block_size) };
        self.mapping.unmap_range(0..u64::MAX, true);
        pages?;
        if !self.mapping.is_empty() {
            return Err(FsError::Busy);
        }

        // SAFETY: links 为零的原 canonical slot 仍处于 Running 且无外部使用；
        // inline root 未作为块树读取，EA 与 inode 窗口由准备 token 保活，mapping 与 root 均已放开。
        unsafe { volume.release_inode(release) };
        self.disk().storage_released.store(true, Ordering::Release);
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// RW 回收先在业务锁外借本卷真实 scratch/book，嵌套摘树与 metadata 同保旧源边界。
    fn evict(&self) -> Result<(), FsError> {
        sched::assert_switchable();
        let volume = self.disk_volume()?;
        let writable = volume.mode() == AccessMode::ReadWrite;
        let mut storage = if writable {
            Some(volume.writeback_domain().borrow_storage()?)
        } else {
            None
        };
        let context = storage
            .as_ref()
            .and_then(|storage| storage.metadata_io_context(volume.writeback_domain()));

        let mut attempt = ReclaimAttempt::new();
        let state = self.wait_loaded()?;
        let _scope = ReclaimScope::nofs();
        let _write = state.write.write();
        let old = *state.metadata.lock();
        let deleting = old.links == 0 && writable;
        assert_eq!(
            *state.writecount.lock(),
            0,
            "eviction retains write or exec access"
        );

        if deleting {
            if !self.disk().storage_released.load(Ordering::Acquire) {
                {
                    let _invalidate = state.invalidate.write();
                    self.truncate_locked(state, volume.claim(), 0, true, &mut attempt, context)?;
                }
                #[cfg(feature = "ktest")]
                self.truncate_phase(TruncatePhase::AfterStorageRelease);
            }
        } else if let Some(storage) = storage.as_mut() {
            writeback::sync_inode_with_storage(self, 0..u64::MAX, storage)?;
        }

        if let Some(storage) = storage.as_mut() {
            let _metadata_writeback = volume.metadata_cache().lock_writeback();
            writeback::sync_metadata_locked(volume, storage)?;
        }
        if deleting {
            return Ok(());
        }

        let _invalidate = state.invalidate.write();
        let _writeback = self.mapping.lock_writeback();
        self.mapping.prune_unused();
        if !self.mapping.is_empty() {
            return Err(FsError::Busy);
        }
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// 父 EA 准备已保当前记录及原身份，元数据 WB 串行排除其他引用修改；最后引用仅清 nlink。
    pub(crate) fn release_ea_references(
        &self,
        claim: &FsClaim,
        record: &BufferRef,
        offset: usize,
        count: u64,
    ) {
        self.disk()
            .filesystem
            .require_write(claim)
            .expect("prepared EA inode write claim");
        let state = self.loaded_state();
        let _scope = ReclaimScope::nofs();
        let _write = state.write.write();
        let _root = state.disk_root().write();
        let mut next = *state.metadata.lock();
        assert!(
            next.flags & codec::EA_INODE != 0,
            "EA reference targets ordinary inode"
        );
        let g = self.disk().filesystem.geometry();
        record
            .update(claim, |bytes| {
                codec::patch_ea_inode_references(
                    g,
                    &mut bytes[offset..offset + g.inode_size],
                    &mut next,
                    count,
                )
            })
            .expect("prepared EA inode reference record");
        *state.metadata.lock() = next;
    }

    /// callback 借真实阶段事实；调用时没有短锁，持有的睡眠锁与生产截断完全一致。
    #[cfg(feature = "ktest")]
    fn truncate_phase(&self, phase: TruncatePhase) {
        let hook = self.truncate_hook.lock().clone();
        if let Some(hook) = hook {
            hook(self, phase);
        }
    }

    /// 只设置既有截断顺序的观察点，旧 callback 在短锁外释放。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_truncate_hook(&self, hook: Option<TruncateHook>) {
        let previous = core::mem::replace(&mut *self.truncate_hook.lock(), hook);
        drop(previous);
    }

    /// 普通 Task 在 mmap/PTE 和 invalidate 之外准备原候选，再按同源 EOF 事实准备真实 FS 块。
    pub(crate) fn prepare_mkwrite(
        &self,
        index: u64,
        attempt: &mut ReclaimAttempt,
    ) -> Result<PreparedMkwrite, FsError> {
        if self.is_memory() {
            return self.prepare_memory_mkwrite(index, attempt);
        }
        let volume = self.disk_volume()?;
        volume.require_write(volume.claim())?;
        background::balance_dirty_pages(self);
        let _access = self.acquire_write()?;
        loop {
            let mut prepared = Some(self.mapping.prepare_file(self, index, attempt)?);
            let result = (|| {
                let state = self.wait_loaded()?;
                let _scope = ReclaimScope::nofs();
                let _write = state.write.write();
                let _invalidate = state.invalidate.read();
                let metadata = *state.metadata.lock();
                if metadata.kind != InodeKind::Regular {
                    return Err(FsError::InvalidOperation);
                }
                let start = index
                    .checked_mul(PAGE_SIZE as u64)
                    .ok_or(FsError::FileTooLarge)?;
                if metadata.size <= start {
                    return Err(FsError::InvalidOperation);
                }
                let Some(folio) = self.fill_prepared(index, &mut prepared, None)? else {
                    return Ok(None);
                };

                let block_size = volume.geometry().block_size;
                for offset in (0..PAGE_SIZE).step_by(block_size) {
                    let position = start
                        .checked_add(offset as u64)
                        .ok_or(FsError::FileTooLarge)?;
                    if metadata.size <= position {
                        break;
                    }
                    self.prepare_mkwrite_block(state, &folio, position)?;
                }
                let prepared_sectors = folio.lock_content().file_prepared_sectors();
                Ok(Some(PreparedMkwrite {
                    folio,
                    index,
                    size: metadata.size,
                    prepared_sectors,
                }))
            })();
            drop(prepared);
            match result {
                Ok(Some(prepared)) => return Ok(prepared),
                Ok(None) => continue,
                Err(error) => return Err(error),
            }
        }
    }

    /// inode.write 与 invalidate 读侧已经持有；块准备、发表和旧节点归还沿原写后端。
    fn prepare_mkwrite_block(
        &self,
        state: &InodeState,
        folio: &FolioRef,
        position: u64,
    ) -> Result<(), FsError> {
        let volume = self.disk_volume()?;
        let block_size = volume.geometry().block_size;
        let old = *state.metadata.lock();
        let (record, record_offset) = volume.inode_record(old.number, None)?;
        let prepared = {
            let root = state.disk_root().write();
            let current = record.with_read(|bytes| {
                codec::decode_inode(
                    volume.geometry(),
                    old.number,
                    &bytes[record_offset..record_offset + volume.geometry().inode_size],
                )
            })??;
            if InodeMetadata::from_data(&current) != old || current.block != *root {
                return Err(FsError::Corrupt);
            }
            volume.prepare_write(&old, &root, position / block_size as u64)?
        };
        let next = match write_metadata(
            volume.geometry(),
            &old,
            0,
            0,
            prepared.added_blocks,
            prepared.removed_blocks,
            time::realtime(),
        ) {
            Ok(next) => next,
            Err(error) => {
                prepared.cancel(volume);
                return Err(error);
            }
        };

        let mut root = state.disk_root().write();
        let mut content = folio.lock_content();
        let offset = (position % PAGE_SIZE as u64) as usize;
        if prepared.zero {
            content
                .zero_file_range(offset, block_size)
                .expect("prepared mkwrite zero range");
            self.note_dirty();
            content
                .mark_dirty(offset, block_size)
                .expect("prepared mkwrite dirty range");
        }

        let next_root = prepared.root;
        let retired = prepared.publish(volume, old.checksum_seed);
        record
            .update(volume.claim(), |bytes| {
                codec::patch_inode_mapping(
                    volume.geometry(),
                    &mut bytes[record_offset..record_offset + volume.geometry().inode_size],
                    &next,
                    &next_root,
                )
            })
            .expect("prepared writable mkwrite inode record");
        *root = next_root;
        *state.metadata.lock() = next;
        content
            .mark_file_prepared(offset, block_size)
            .expect("published mkwrite block");
        drop(content);

        for block in retired {
            // SAFETY: root 写侧已排空旧读者，发表的新树已摘除原节点，写侧禁止新使用；
            // release 等旧元数据真实 WB 并失效准确窗口，然后才归还物理位图。
            unsafe { volume.release_detached_block(block) };
        }
        Ok(())
    }

    /// 最终授权在 mmap 之后、folio 之前取得读侧，增长与失效使用同一锁的写侧。
    pub(crate) fn lock_invalidate(&self) -> Result<InvalidateGuard<'_>, FsError> {
        let scope = ReclaimScope::nofs();
        let guard = self.wait_loaded()?.invalidate.read();
        Ok(InvalidateGuard {
            _guard: guard,
            _scope: scope,
        })
    }

    /// 持 mmap→invalidate 读侧→本页内容锁重核准备事实，不等待 WB 或取得块根。
    /// 真实 size、页身份或准备事实变化返回 false；coherent 文件原页允许 Dirty 与 WB 并存。
    pub(crate) fn finish_mkwrite(
        &self,
        prepared: &PreparedMkwrite,
        content: &mut FolioContent<'_>,
    ) -> Result<bool, FsError> {
        let metadata = *self.loaded_state().metadata.lock();
        let start = prepared
            .index
            .checked_mul(PAGE_SIZE as u64)
            .ok_or(FsError::FileTooLarge)?;
        if metadata.size != prepared.size
            || metadata.size <= start
            || content.folio().phys_addr() != prepared.folio.phys_addr()
            || !self.mapping.is_canonical(prepared.index, &prepared.folio)
            || !self.is_memory() && content.file_prepared_sectors() != prepared.prepared_sectors
        {
            return Ok(false);
        }
        if self.is_memory() {
            content.range_ready(0, PAGE_SIZE)?;
            return Ok(true);
        }

        let block_size = self.disk_volume()?.geometry().block_size;
        self.note_dirty();
        content.mark_prepared_dirty(
            (metadata.size - start).min(PAGE_SIZE as u64) as usize,
            block_size,
        )?;
        Ok(true)
    }

    /// 唯一普通写循环只由绑定本 inode 的 WriteAccess 调用；输入 fault 不持块根或内容锁。
    fn write_from(
        &self,
        claim: &FsClaim,
        offset: u64,
        append: bool,
        source: &mut WriteBuffer<'_>,
    ) -> Result<(u64, usize), FsError> {
        let volume = self.disk_volume()?;
        volume.require_write(claim)?;
        let state = self.wait_loaded()?;
        let mut attempt = ReclaimAttempt::new();
        let _scope = ReclaimScope::nofs();
        let _write = state.write.write();
        let initial = *state.metadata.lock();
        if initial.kind == InodeKind::Directory {
            return Err(FsError::IsDirectory);
        }
        if initial.kind != InodeKind::Regular {
            return Err(FsError::InvalidOperation);
        }

        let start = if append { initial.size } else { offset };
        let limit = write_limit(volume.geometry(), initial.flags);
        let remaining = source.len() - source.copied();
        let mut accepted = 0usize;
        let mut input = [0u8; 512];
        while accepted < remaining {
            let position = match start.checked_add(accepted as u64) {
                Some(position) if position < limit => position,
                _ => return write_result(start, accepted, FsError::FileTooLarge),
            };
            let size = volume.geometry().block_size;
            let count = (remaining - accepted)
                .min(input.len())
                .min(size - (position % size as u64) as usize)
                .min((limit - position) as usize);
            let pulled = match source.pull(&mut input[..count]) {
                Ok(0) => break,
                Ok(count) => count,
                Err(error) => return write_result(start, accepted, error),
            };

            let old = *state.metadata.lock();
            // 输入已真正取得后才进入失效域，用户源 fault 不携带目的 invalidate guard。
            let end = position + pulled as u64;
            let _invalidate = state.invalidate.write();
            if old.size < end {
                if let Err(error) = self.zero_eof_tail(&old, position, &mut attempt) {
                    return write_result(start, accepted, error);
                }
            }

            let (record, record_offset) = match volume.inode_record(old.number, None) {
                Ok(record) => record,
                Err(error) => return write_result(start, accepted, error),
            };
            let prepared = {
                let root = state.disk_root().write();
                let current = record.with_read(|bytes| {
                    crate::fs::ext4::codec::decode_inode(
                        volume.geometry(),
                        old.number,
                        &bytes[record_offset..record_offset + volume.geometry().inode_size],
                    )
                });
                let valid = match current {
                    Ok(Ok(current))
                        if InodeMetadata::from_data(&current) == old && current.block == *root =>
                    {
                        Ok(())
                    }
                    Ok(Ok(_)) => Err(FsError::Corrupt),
                    Ok(Err(error)) => Err(error),
                    Err(error) => Err(error.into()),
                };
                if let Err(error) = valid {
                    return write_result(start, accepted, error);
                }
                volume.prepare_write(&old, &root, position / size as u64)
            };
            let prepared = match prepared {
                Ok(prepared) => prepared,
                Err(error) => return write_result(start, accepted, error),
            };
            let next = match write_metadata(
                volume.geometry(),
                &old,
                position,
                pulled,
                prepared.added_blocks,
                prepared.removed_blocks,
                time::realtime(),
            ) {
                Ok(next) => next,
                Err(error) => {
                    prepared.cancel(volume);
                    return write_result(start, accepted, error);
                }
            };

            // 尚未发表的映射保持旧根；filefault 可取得读侧完成目的页初始化。
            let folio = match self.read_page(position / PAGE_SIZE as u64, &mut attempt, None) {
                Ok(folio) => folio,
                Err(error) => {
                    prepared.cancel(volume);
                    return write_result(start, accepted, error);
                }
            };

            let mut root = state.disk_root().write();
            let mut content = folio.lock_content();
            let block_offset = ((position / size as u64 * size as u64) % PAGE_SIZE as u64) as usize;
            self.note_dirty();
            if prepared.zero {
                let zeros = [0u8; 512];
                for part in (block_offset..block_offset + size).step_by(zeros.len()) {
                    content
                        .copy_from(part, &zeros)
                        .expect("prepared writable file block");
                }
            }
            content
                .copy_from((position % PAGE_SIZE as u64) as usize, &input[..pulled])
                .expect("prepared writable file bytes");
            content
                .mark_dirty(block_offset, size)
                .expect("prepared file block dirty range");

            let next_root = prepared.root;
            let retired = prepared.publish(volume, old.checksum_seed);
            record
                .update(claim, |bytes| {
                    crate::fs::ext4::codec::patch_inode_mapping(
                        volume.geometry(),
                        &mut bytes[record_offset..record_offset + volume.geometry().inode_size],
                        &next,
                        &next_root,
                    )
                })
                .expect("prepared writable inode record");
            *root = next_root;
            *state.metadata.lock() = next;
            content
                .mark_file_prepared(block_offset, size)
                .expect("published buffered write block");
            drop(content);

            for block in retired {
                // SAFETY: root 写侧已排空旧块树读者，发表后的根和父节点均已去掉旧节点；
                // 写侧仍阻止新读者，释放窗口保原页，forget 等真实元数据 IO/WB 收权后失效，
                // 此后旧窗口不再被映射查询使用，位图最后才允许物理块复用。
                unsafe {
                    volume.release_detached_block(block);
                }
            }
            drop(root);
            accepted += pulled;
        }
        Ok((start, accepted))
    }

    /// 内核按位置读取与用户接收端使用同一页交付循环。
    #[cfg(feature = "ktest")]
    pub fn read_at(&self, offset: u64, dst: &mut [u8]) -> Result<usize, FsError> {
        self.read_into(offset, &mut ReadBuffer::kernel(dst), None)
    }

    /// 每页的私有候选和有限 OOM 请求在 invalidate 外准备；原字节消费与交付仍分开。
    pub(crate) fn read_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        ra: Option<&SpinLock<ReadaheadState>>,
    ) -> Result<usize, FsError> {
        let initial = target.copied();
        let remaining = target.len() - initial;
        if remaining == 0 {
            return Ok(0);
        }
        let state = self.wait_loaded()?;
        let data = *state.metadata.lock();
        if data.kind == InodeKind::Directory {
            return Err(FsError::IsDirectory);
        }
        if data.kind != InodeKind::Regular {
            return Err(FsError::InvalidOperation);
        }
        if data.size <= offset {
            return Ok(0);
        }

        let mut attempt = ReclaimAttempt::new();
        let len = (data.size - offset).min(remaining as u64) as usize;
        let mut copied = 0;
        let mut staging = [0u8; 512];
        while copied < len {
            let position = offset + copied as u64;
            let index = position / PAGE_SIZE as u64;
            let in_page = (position % PAGE_SIZE as u64) as usize;
            let first = copied == 0 || in_page == 0;
            let mut count = (len - copied).min(staging.len()).min(PAGE_SIZE - in_page);
            if first && !self.is_memory() {
                if let Some(ra) = ra {
                    if let Err(error) =
                        readahead::prepare(self, ra, position, len - copied, &mut attempt)
                    {
                        return if copied == 0 { Err(error) } else { Ok(copied) };
                    }
                }
            }
            let mut prepared = match self.mapping.prepare_file(self, index, &mut attempt) {
                Ok(prepared) => Some(prepared),
                Err(error) => return if copied == 0 { Err(error) } else { Ok(copied) },
            };
            let result = {
                let _invalidate = self.lock_invalidate()?;
                let current = *state.metadata.lock();
                if current.size <= position {
                    count = 0;
                    Ok(Some(()))
                } else {
                    count = count.min((current.size - position) as usize);
                    self.fill_prepared(index, &mut prepared, None)
                        .and_then(|folio| {
                            let Some(folio) = folio else {
                                return Ok(None);
                            };
                            folio.copy_to(in_page, &mut staging[..count])?;
                            if first {
                                folio.mark_accessed();
                            }
                            Ok(Some(()))
                        })
                }
            };
            drop(prepared);
            let result = match result {
                Ok(Some(())) => Ok(()),
                Ok(None) => continue,
                Err(error) => Err(error),
            };
            if count == 0 {
                return Ok(copied);
            }

            let result = result.and_then(|()| target.push(&staging[..count]));
            copied = target.copied() - initial;
            if copied != 0 && !self.is_memory() {
                if let Some(ra) = ra {
                    readahead::note_read(ra, offset + copied as u64);
                }
            }
            if let Err(error) = result {
                return if copied == 0 { Err(error) } else { Ok(copied) };
            }
        }
        Ok(copied)
    }

    /// 普通 fault 在失效保护外准备真实候选；仅已退休的命中重查，不重放 canonical IO 首错。
    pub(crate) fn file_page(
        &self,
        index: u64,
        attempt: &mut ReclaimAttempt,
    ) -> Result<FolioRef, FsError> {
        loop {
            let mut prepared = Some(self.mapping.prepare_file(self, index, attempt)?);
            let result = (|| {
                let _invalidate = self.lock_invalidate()?;
                let data = self.metadata_snapshot()?;
                let start = index
                    .checked_mul(PAGE_SIZE as u64)
                    .ok_or(FsError::FileTooLarge)?;
                if data.kind != InodeKind::Regular || data.size <= start {
                    return Err(FsError::InvalidOperation);
                }
                self.fill_prepared(index, &mut prepared, None)
            })();
            drop(prepared);
            match result {
                Ok(Some(folio)) => return Ok(folio),
                Ok(None) => continue,
                Err(error) => return Err(error),
            }
        }
    }

    /// 普通增长写先准备尾页和物理块事实，再撤旧共享 W 并清将变有效的 gap。
    fn zero_eof_tail(
        &self,
        old: &InodeMetadata,
        position: u64,
        attempt: &mut ReclaimAttempt,
    ) -> Result<(), FsError> {
        if let Some(tail) = self.prepare_tail(old.size, position, attempt, None)? {
            self.apply_tail(&tail);
        }
        Ok(())
    }

    /// 写序列化和 invalidate 保护下准备准确尾区间；本阶段不改原字节或脏义务。
    fn prepare_tail(
        &self,
        start: u64,
        end: u64,
        attempt: &mut ReclaimAttempt,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<Option<PreparedTail>, FsError> {
        let offset = (start % PAGE_SIZE as u64) as usize;
        if offset == 0 {
            return Ok(None);
        }
        let index = start / PAGE_SIZE as u64;
        let folio = self.read_page(index, attempt, context)?;
        let size = self.disk_volume()?.geometry().block_size;
        let end = end
            .saturating_sub(index * PAGE_SIZE as u64)
            .min(PAGE_SIZE as u64) as usize;
        let end = end.max(offset);
        let mapped = self.with_map_read(|metadata, root| {
            let mut mapped = 0u8;
            for in_page in (offset / size * size..end).step_by(size) {
                let logical = (index * PAGE_SIZE as u64 + in_page as u64) / size as u64;
                if matches!(
                    self.disk_volume()?
                        .map_block(metadata, root, logical, context)?,
                    BlockMapping::Mapped { .. }
                ) {
                    mapped |= 1 << (in_page / size);
                }
            }
            Ok(mapped)
        })?;
        Ok(Some(PreparedTail {
            folio,
            index,
            start: offset,
            end,
            mapped,
        }))
    }

    /// 撤旧共享 W/CSD 后零化原页尾；仅真实 Mapped 块产生 Dirty，原准备位不扩成新分配。
    fn apply_tail(&self, tail: &PreparedTail) {
        let mut content = tail.folio.lock_content();
        self.mapping.mkclean(tail.index, &tail.folio, &mut content);
        if tail.start == tail.end {
            return;
        }
        content
            .zero_file_range(tail.start, tail.end - tail.start)
            .expect("prepared file tail zero range");
        let size = self.disk().filesystem.geometry().block_size;
        for offset in (tail.start / size * size..tail.end).step_by(size) {
            if tail.mapped & (1 << (offset / size)) != 0 {
                self.note_dirty();
                content
                    .mark_dirty(offset, size)
                    .expect("prepared mapped tail dirty range");
            }
        }
    }

    /// 已持实际 FS 依赖的内部填充沿同请求的 NOFS/NOIO；只认领真实页，不改变 allocator。
    fn read_page(
        &self,
        index: u64,
        attempt: &mut ReclaimAttempt,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<FolioRef, FsError> {
        let mut prepared = Some(self.mapping.prepare_file(self, index, attempt)?);
        let folio = self.fill_prepared(index, &mut prepared, context)?;
        Ok(folio.expect("write/invalidate protected internal file hit retired"))
    }

    /// caller 保 invalidate；相同旧 metadata 边界覆盖根下降，IO 凭证在提交前已保原 inode。
    fn fill_prepared(
        &self,
        index: u64,
        prepared: &mut Option<PreparedFile>,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<Option<FolioRef>, FsError> {
        let Some((folio, read)) = self.mapping.publish_file(self, index, prepared)? else {
            return Ok(None);
        };

        if let Some(mut read) = read {
            if self.is_memory() {
                read.bytes_mut().iter_mut().for_each(|byte| {
                    byte.write(0);
                });
                // SAFETY: 原读入票独占整页初始化，未交设备；全部字节已写零，
                // bytes_mut 借用已结束，完成后不再使用初始化地址。
                unsafe {
                    read.complete(IoResult {
                        completed: PAGE_SIZE,
                        error: None,
                    })
                };
            } else {
                let (size, targets) =
                    match writeback::prepare_file_targets(self, index, &(0..u64::MAX), context) {
                        Ok(prepared) => prepared,
                        Err(error) => return Err(finish_error(read, error)),
                    };
                self.disk_volume()?.fill_page(index, size, &targets, read)?;
            }
        }

        folio.wait_ready()?;
        Ok(Some(folio))
    }
}

impl Deref for InodeRef {
    type Target = Inode;

    /// 只借用原固定对象；没有 DerefMut 或公开 Arc 提供移动固定等待源的入口。
    fn deref(&self) -> &Inode {
        &self.0
    }
}

impl Inode {
    /// 返回该对象的盘上 inode 号；跨卷身份还须包括 filesystem 实例。
    pub fn number(&self) -> u64 {
        self.number
    }

    /// 磁盘调用必须先取得实际磁盘分支，内存节点没有卷或 claim。
    pub fn disk_volume(&self) -> Result<&Arc<Ext4Volume>, FsError> {
        match &self.backend {
            InodeBackend::Disk(disk) => Ok(&disk.filesystem),
            InodeBackend::Memory { .. } => Err(FsError::InvalidOperation),
        }
    }

    /// 内部磁盘协议只在已经选择磁盘分支后借用真实生命周期状态。
    fn disk(&self) -> &DiskInode {
        match &self.backend {
            InodeBackend::Disk(disk) => disk,
            InodeBackend::Memory { .. } => panic!("memory inode entered disk protocol"),
        }
    }

    /// 等待唯一加载结果；属性与块根的访问必须继续取得各自规定的锁。
    fn wait_loaded(&self) -> Result<&InodeState, FsError> {
        sched::assert_switchable();
        let inode = self.pinned().get_ref();
        loop {
            if let Some(result) = inode.data.get() {
                return result.as_ref().map_err(|error| *error);
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 私有 Arc 只给共享借用，inode 和内嵌等待源固定且由调用者引用保活；
            // pin! 固定原栈节点，registration 不被遗忘，退出先撤链再释放节点和 waiter。
            let _registration = unsafe { inode.waiters.register(node.as_mut()) };
            if let Some(result) = inode.data.get() {
                return result.as_ref().map_err(|error| *error);
            }
            waiter.park();
        }
    }

    /// token 只能在加载成功后构造，析构路径不再等待加载或创建新的责任。
    fn loaded_state(&self) -> &InodeState {
        self.data
            .get()
            .expect("access token precedes inode load")
            .as_ref()
            .expect("access token retains a failed inode")
    }

    /// 短属性锁内复制当前运行值，不将 Spin guard 带入 IO、等待或用户复制。
    pub fn metadata_snapshot(&self) -> Result<InodeMetadata, FsError> {
        let mut metadata = *self.wait_loaded()?.metadata.lock();
        if self.is_memory()
            && metadata.kind == InodeKind::Regular
            && self.memory_special().is_none()
        {
            metadata.blocks =
                self.mapping.len() as u64 * (PAGE_SIZE / SECTOR_BYTES as usize) as u64;
        }
        Ok(metadata)
    }

    /// 睡眠 data-sem 读侧保持块映射有效；属性快照的短锁在调用闭包前已经释放。
    pub fn with_map_read<T>(
        &self,
        consume: impl FnOnce(&InodeMetadata, &[u8; 60]) -> Result<T, FsError>,
    ) -> Result<T, FsError> {
        let state = self.wait_loaded()?;
        let _scope = ReclaimScope::nofs();
        self.disk_volume()?;
        let root = state.disk_root().read();
        let metadata = *state.metadata.lock();
        consume(&metadata, &root)
    }

    /// 在块映射读保护下执行真实后端查询，不导出永久块根借用。
    #[cfg(feature = "ktest")]
    pub fn map_block(&self, logical: u64) -> Result<BlockMapping, FsError> {
        self.with_map_read(|metadata, root| {
            self.disk_volume()?.map_block(metadata, root, logical, None)
        })
    }

    /// inode 写序列化与 data-sem 写侧保护受控属性修改，并字段级更新共同盘面记录。
    /// 此入口不分配块、不截断文件；大小和块布局变更须由对应内容协议实现。
    #[cfg(feature = "ktest")]
    pub fn update_metadata<T>(
        &self,
        claim: &FsClaim,
        update: impl FnOnce(&mut InodeMetadata) -> T,
    ) -> Result<T, FsError> {
        let volume = self.disk_volume()?;
        volume.require_write(claim)?;
        let state = self.wait_loaded()?;
        let _scope = ReclaimScope::nofs();
        let _write = state.write.write();
        let _root = state.disk_root().write();
        let old = *state.metadata.lock();
        if old.flags & codec::EA_INODE != 0 {
            return Err(FsError::InvalidOperation);
        }
        let mut next = old;
        let result = update(&mut next);
        volume.update_inode_metadata(claim, &old, &next)?;
        *state.metadata.lock() = next;
        Ok(result)
    }

    /// 内核加载或范围扫描借用唯一页索引，须保对应 InodeRef 到裸页使用与 IO 全尾结束。
    /// 普通文件页消费使用 InodeRef 的出口，交给外部的页引用不能缺少原 inode anchor。
    pub(crate) fn mapping(&self) -> &FileMapping {
        &self.mapping
    }

    /// 由后端按真实 inode 存储规则返回符号链接的原始字节。
    pub fn read_link(&self) -> Result<Vec<u8>, FsError> {
        if let InodeBackend::Memory {
            content: MemoryContent::Symlink(target),
            ..
        } = &self.backend
        {
            return try_bytes(target);
        }
        self.with_map_read(|metadata, root| self.disk_volume()?.read_link(metadata, root))
    }

    /// 私有拥有类型禁止移动内嵌等待源，借用期间最后一个 Arc 不能析构。
    fn pinned(&self) -> Pin<&Self> {
        // SAFETY: Inode 只能由私有构造放入 Arc 后发表，InodeRef 不暴露 Arc 或可变借用；
        // 本模块从不 unwrap、移出或替换对象，Arc 的原分配保持到本共享借用终止之后。
        unsafe { Pin::new_unchecked(self) }
    }
}

impl InodeWriteGuard<'_> {
    /// 短属性锁内复制当前属性；写侧继续保护后续存储提交。
    pub(crate) fn metadata_snapshot(&self) -> InodeMetadata {
        *self.state.metadata.lock()
    }

    /// 预备硬链接数及 ctime；格式或 inode-table 失败发生在目录名提交前。
    pub(crate) fn prepare_link_delta(
        &self,
        delta: i32,
        now_ns: u64,
    ) -> Result<PreparedInodeMetadata, FsError> {
        prepare_link_delta(self.inode, self.state, delta, now_ns)
    }

    /// 预备仅改变 ctime 的 inode-table 记录。
    pub(crate) fn prepare_ctime(&self, now_ns: u64) -> Result<PreparedInodeMetadata, FsError> {
        prepare_inode_ctime(self.inode, self.state, now_ns)
    }

    /// 在 canonical 写侧核所有者并预备权限和 ctime；原类型、内容与执行写占用不改变。
    pub(crate) fn prepare_mode(
        &self,
        mode: u16,
        uid: u32,
        gid: u32,
        now_ns: u64,
    ) -> Result<PreparedInodeMetadata, FsError> {
        let old = self.metadata_snapshot();
        if uid != 0 && uid != old.uid {
            return Err(FsError::NotPermitted);
        }
        let mut permissions = mode & S_IALLUGO;
        if uid != 0 && gid != old.gid {
            permissions &= !S_ISGID;
        }
        let mut prepared = self.prepare_ctime(now_ns)?;
        prepared.next.mode = (old.mode & !S_IALLUGO) | permissions;
        Ok(prepared)
    }

    /// 在唯一写侧预备时间和当前 ctime；省略的字段保留旧值，内容写占用不参与属性修改。
    pub(crate) fn prepare_times(
        &self,
        atime: Option<InodeTime>,
        mtime: Option<InodeTime>,
        now_ns: u64,
    ) -> Result<PreparedInodeMetadata, FsError> {
        let old = self.metadata_snapshot();
        let mut next = old;
        let now = memory_time(now_ns);
        if self.inode.is_memory() {
            self.inode.require_memory_write()?;
            if atime
                .into_iter()
                .chain(mtime)
                .any(|time| (time::NSEC_PER_SEC as u32) <= time.nanoseconds)
            {
                return Err(FsError::InvalidOperation);
            }
            next.atime = atime.unwrap_or(old.atime);
            next.mtime = mtime.unwrap_or(old.mtime);
            next.ctime = now;
        } else {
            if let Some(time) = atime {
                next.atime = codec::prepare_inode_time(
                    time,
                    codec::INODE_ATIME_EXTRA_OFFSET,
                    old.extra_isize,
                )?;
            }
            if let Some(time) = mtime {
                next.mtime = codec::prepare_inode_time(
                    time,
                    codec::INODE_MTIME_EXTRA_OFFSET,
                    old.extra_isize,
                )?;
            }
            next.ctime =
                codec::prepare_inode_time(now, codec::INODE_CTIME_EXTRA_OFFSET, old.extra_isize)?;
        }
        prepare_inode_metadata(self.inode, self.state, old, next)
    }

    /// 在已持有的 canonical 写侧内消费记录准备并发布属性。
    pub(crate) fn publish_metadata(&self, prepared: PreparedInodeMetadata) {
        publish_inode_metadata(self.inode, self.state, prepared);
    }
}

impl DirectoryReadGuard<'_> {
    /// 借用与当前读侧同一份 canonical 属性快照。
    pub(crate) fn metadata_snapshot(&self) -> InodeMetadata {
        *self.state.metadata.lock()
    }

    /// 在既有目录读侧内查找并取得 canonical 子 inode，不再递归读锁。
    pub(crate) fn lookup(
        &self,
        cache: &InodeCache,
        name: &[u8],
    ) -> Result<Option<InodeRef>, FsError> {
        lookup_locked(self.inode, cache, name)
    }

    /// 按目录真实磁盘游标返回下一项，读侧覆盖所有元数据窗口访问。
    pub(crate) fn dir_next(
        &self,
        cursor: &DirectoryCursor,
    ) -> Result<Option<DirectoryEntry>, FsError> {
        if self.inode.is_memory() {
            return self.inode.memory_dir_next(cursor);
        }
        self.inode.with_map_read(|metadata, root| {
            self.inode.disk_volume()?.dir_next(metadata, root, cursor)
        })
    }

    /// 将调用者 cookie 重新解析为普通字节或 HTree 游标。
    pub(crate) fn cursor_at(&self, cookie: u64) -> Result<DirectoryCursor, FsError> {
        if self.inode.is_memory() {
            return Ok(DirectoryCursor::Memory(cookie));
        }
        self.inode.with_map_read(|metadata, root| {
            self.inode
                .disk_volume()?
                .directory_cursor(metadata, root, cookie)
        })
    }

    /// 返回后端规定的 SEEK_END cookie，indexed 目录使用 HTree EOF。
    pub(crate) fn end_cookie(&self) -> Result<u64, FsError> {
        if self.inode.is_memory() {
            return Ok(self.inode.memory_directory()?.lock().next_cookie);
        }
        self.inode.with_map_read(|metadata, root| {
            self.inode
                .disk_volume()?
                .directory_end_cookie(metadata, root)
        })
    }
}

impl DirectoryWriteGuard<'_> {
    /// 借用与当前写侧同一份 canonical 属性快照。
    pub(crate) fn metadata_snapshot(&self) -> InodeMetadata {
        *self.state.metadata.lock()
    }

    /// 持写侧重查目录项，不尝试获取该 inode 的读侧。
    pub(crate) fn lookup(
        &self,
        cache: &InodeCache,
        name: &[u8],
    ) -> Result<Option<InodeRef>, FsError> {
        lookup_locked(self.inode, cache, name)
    }

    /// 目录目标的链接变化复用当前目录写侧，不递归取得 inode 写锁。
    pub(crate) fn prepare_link_delta(
        &self,
        delta: i32,
        now_ns: u64,
    ) -> Result<PreparedInodeMetadata, FsError> {
        prepare_link_delta(self.inode, self.state, delta, now_ns)
    }

    /// 目录本身仅改变 ctime 时继续消费当前写侧。
    pub(crate) fn prepare_ctime(&self, now_ns: u64) -> Result<PreparedInodeMetadata, FsError> {
        prepare_inode_ctime(self.inode, self.state, now_ns)
    }

    /// 在当前目录写侧内消费属性准备，目录查找者尚未取得读侧。
    pub(crate) fn publish_metadata(&self, prepared: PreparedInodeMetadata) {
        publish_inode_metadata(self.inode, self.state, prepared);
    }

    /// 真实扫描点目录与普通项，保留后端 checksum 和 I/O 错误。
    pub(crate) fn is_empty(&self) -> Result<bool, FsError> {
        if self.inode.is_memory() {
            return Ok(self.inode.memory_directory()?.lock().entries.is_empty());
        }
        let metadata = self.metadata_snapshot();
        let root = *self.state.disk_root().read();
        self.inode
            .disk_volume()?
            .directory_is_empty(&metadata, &root)
    }

    /// 预备新目录项及父目录属性，所有可失败操作先于名字提交。
    pub(crate) fn prepare_insert(
        &self,
        name: &[u8],
        number: u64,
        kind: InodeKind,
        parent_links_delta: i32,
        now_ns: u64,
    ) -> Result<PreparedDirectoryMutation, FsError> {
        if self.inode.is_memory() {
            return self.prepare_memory_edit(
                MemoryEdit::Insert(name, number, kind),
                parent_links_delta,
                now_ns,
                false,
            );
        }
        let metadata = self.metadata_snapshot();
        let root = *self.state.disk_root().read();
        let edit = self
            .inode
            .disk_volume()?
            .prepare_directory_add(&metadata, &root, name, number, kind)?;
        self.prepare_edit(edit, parent_links_delta, now_ns)
    }

    /// 预备删除已在同一父写侧重新查到的名称。
    pub(crate) fn prepare_remove(
        &self,
        name: &[u8],
        parent_links_delta: i32,
        now_ns: u64,
    ) -> Result<PreparedDirectoryMutation, FsError> {
        if self.inode.is_memory() {
            return self.prepare_memory_edit(
                MemoryEdit::Remove(name),
                parent_links_delta,
                now_ns,
                false,
            );
        }
        let metadata = self.metadata_snapshot();
        let root = *self.state.disk_root().read();
        let edit = self
            .inode
            .disk_volume()?
            .prepare_directory_remove(&metadata, &root, name)?;
        self.prepare_edit(edit, parent_links_delta, now_ns)
    }

    /// 预备跨父 rename 覆盖目标目录项。
    pub(crate) fn prepare_replace(
        &self,
        name: &[u8],
        number: u64,
        kind: InodeKind,
        parent_links_delta: i32,
        now_ns: u64,
    ) -> Result<PreparedDirectoryMutation, FsError> {
        if self.inode.is_memory() {
            return self.prepare_memory_edit(
                MemoryEdit::Replace(name, number, kind),
                parent_links_delta,
                now_ns,
                false,
            );
        }
        let metadata = self.metadata_snapshot();
        let root = *self.state.disk_root().read();
        let edit = self
            .inode
            .disk_volume()?
            .prepare_directory_replace(&metadata, &root, name, number, kind)?;
        self.prepare_edit(edit, parent_links_delta, now_ns)
    }

    /// 单次准备覆盖目标并移走源项，避免同一旧根上的两份互相覆盖编辑。
    #[expect(
        clippy::too_many_arguments,
        reason = "同父移动将源目标身份、父链接差量与时间纳入同一目录写 guard 的准备"
    )]
    pub(crate) fn prepare_move(
        &self,
        old_name: &[u8],
        new_name: &[u8],
        number: u64,
        kind: InodeKind,
        target: Option<(u64, InodeKind)>,
        parent_links_delta: i32,
        now_ns: u64,
    ) -> Result<PreparedDirectoryMutation, FsError> {
        if self.inode.is_memory() {
            return self.prepare_memory_edit(
                MemoryEdit::Move(old_name, new_name, number, kind, target),
                parent_links_delta,
                now_ns,
                false,
            );
        }
        let metadata = self.metadata_snapshot();
        let root = *self.state.disk_root().read();
        let edit = self
            .inode
            .disk_volume()?
            .prepare_directory_move(&metadata, &root, old_name, new_name, number, kind, target)?;
        self.prepare_edit(edit, parent_links_delta, now_ns)
    }

    /// 预备被移动目录的实际 `..` 记录和修改时间。
    pub(crate) fn prepare_dotdot(
        &self,
        parent_number: u64,
        now_ns: u64,
    ) -> Result<PreparedDirectoryMutation, FsError> {
        if self.inode.is_memory() {
            return self.prepare_memory_edit(MemoryEdit::Dotdot(parent_number), 0, now_ns, false);
        }
        let metadata = self.metadata_snapshot();
        let root = *self.state.disk_root().read();
        let edit =
            self.inode
                .disk_volume()?
                .prepare_directory_dotdot(&metadata, &root, parent_number)?;
        self.prepare_edit(edit, 0, now_ns)
    }

    /// 目录编辑和父 inode 记录完全准备后，先阻止映射读者再执行无失败发表。
    pub(crate) fn publish(&self, prepared: PreparedDirectoryMutation) {
        let mut prepared = match prepared {
            PreparedDirectoryMutation::Disk(prepared) => prepared,
            PreparedDirectoryMutation::Memory {
                parent,
                mut directory,
                next,
            } => {
                assert!(
                    self.inode.ptr_eq(&parent),
                    "memory directory token mismatch"
                );
                if let Some(directory) = directory.take() {
                    let retired = core::mem::replace(
                        &mut *self
                            .inode
                            .memory_directory()
                            .expect("memory directory token")
                            .lock(),
                        directory,
                    );
                    *self.state.metadata.lock() = next;
                    drop(retired);
                }
                return;
            }
        };
        assert!(
            self.inode.ptr_eq(&prepared.parent),
            "directory edit parent mismatch"
        );
        assert!(
            Arc::ptr_eq(
                self.inode.disk_volume().expect("disk directory token"),
                &prepared.volume
            ),
            "directory edit volume mismatch"
        );
        let edit = prepared
            .edit
            .take()
            .expect("directory edit already consumed");
        if !prepared.changed {
            assert!(prepared.inode_mapping.is_none());
            edit.cancel(&prepared.volume);
            return;
        }

        let mut root = self.state.disk_root().write();
        let result = edit.publish(&prepared.volume);
        assert_eq!(result.root, prepared.root, "directory edit root changed");
        assert_eq!(
            result.size, prepared.next.size,
            "directory edit size changed"
        );
        assert_eq!(
            result.blocks, prepared.next.blocks,
            "directory edit blocks changed"
        );
        assert_eq!(
            result.flags, prepared.next.flags,
            "directory edit flags changed"
        );

        let inode_mapping = prepared
            .inode_mapping
            .take()
            .expect("directory inode record already consumed");
        prepared.volume.publish_inode_mapping(inode_mapping);
        *root = prepared.root;
        *self.state.metadata.lock() = prepared.next;
    }

    /// 父 guard 持有期间把目录后端编辑与记录窗口绑定为同一可取消准备。
    fn prepare_edit(
        &self,
        edit: PreparedDirectoryEdit,
        parent_links_delta: i32,
        now_ns: u64,
    ) -> Result<PreparedDirectoryMutation, FsError> {
        let volume = Arc::clone(self.inode.disk_volume()?);
        let old = self.metadata_snapshot();
        let old_root = *self.state.disk_root().read();
        let root = *edit.root();
        let changed = edit.changed();
        if !changed {
            if parent_links_delta != 0 {
                edit.cancel(&volume);
                return Err(FsError::InvalidOperation);
            }
            return Ok(PreparedDirectoryMutation::Disk(
                PreparedDiskDirectoryMutation {
                    parent: self.inode.clone(),
                    volume,
                    edit: Some(edit),
                    inode_mapping: None,
                    root: old_root,
                    next: old,
                    changed,
                },
            ));
        }

        let result = (|| {
            let flags = edit.flags();
            let permitted = codec::INDEX | codec::HUGE_FILE;
            if flags & !permitted != old.flags & !permitted {
                return Err(FsError::InvalidOperation);
            }
            let mut next = old;
            next.size = edit.size();
            next.blocks = edit.blocks();
            next.links = updated_link_count(old.links, parent_links_delta)?;
            let (mtime, ctime) = codec::prepare_inode_write_times(now_ns, old.extra_isize)?;
            next.mtime = mtime;
            next.ctime = ctime;
            let (_, huge) = codec::blocks_encoding(volume.geometry(), next.blocks)?;
            next.flags = (flags & !codec::HUGE_FILE) | if huge { codec::HUGE_FILE } else { 0 };
            let mapping = volume.prepare_inode_mapping(&old, &old_root, &next, &root)?;
            Ok::<_, FsError>((next, mapping))
        })();
        match result {
            Ok((next, inode_mapping)) => Ok(PreparedDirectoryMutation::Disk(
                PreparedDiskDirectoryMutation {
                    parent: self.inode.clone(),
                    volume,
                    edit: Some(edit),
                    inode_mapping: Some(inode_mapping),
                    root,
                    next,
                    changed,
                },
            )),
            Err(error) => {
                edit.cancel(&volume);
                Err(error)
            }
        }
    }
}

impl Drop for PreparedDiskDirectoryMutation {
    /// 尚未提交的目录树只撤销本次预留块，不触碰旧名称和 canonical 状态。
    fn drop(&mut self) {
        if let Some(edit) = self.edit.take() {
            edit.cancel(&self.volume);
        }
    }
}

/// 把目录 link count 的变化限定在 ext4 记录可表示范围内。
fn updated_link_count(current: u32, delta: i32) -> Result<u32, FsError> {
    let links = if delta < 0 {
        current.checked_sub(delta.unsigned_abs())
    } else {
        current.checked_add(delta as u32)
    }
    .ok_or(FsError::InvalidOperation)?;
    if u32::from(u16::MAX) < links {
        return Err(FsError::TooManyLinks);
    }
    Ok(links)
}

/// 构造尚未连接目录名的 canonical 初值，并验证 ext4 sector/时间编码。
fn initial_inode_metadata(
    volume: &Ext4Volume,
    initial: &InodeData,
    size: u64,
    blocks: u64,
    now_ns: u64,
) -> Result<InodeMetadata, FsError> {
    if (i64::MAX as u64) < size {
        return Err(FsError::FileTooLarge);
    }
    let (mtime, ctime) = codec::prepare_inode_write_times(now_ns, initial.extra_isize)?;
    let atime = if codec::INODE_ATIME_EXTRA_OFFSET + core::mem::size_of::<u32>()
        <= codec::GOOD_OLD_INODE_BYTES + initial.extra_isize
    {
        mtime
    } else {
        InodeTime {
            nanoseconds: 0,
            ..mtime
        }
    };
    let (_, huge) = codec::blocks_encoding(volume.geometry(), blocks)?;
    let mut metadata = InodeMetadata::from_data(initial);
    metadata.size = size;
    metadata.blocks = blocks;
    metadata.atime = atime;
    metadata.mtime = mtime;
    metadata.ctime = ctime;
    metadata.flags = (initial.flags & !codec::HUGE_FILE) | if huge { codec::HUGE_FILE } else { 0 };
    metadata.fast_symlink = metadata.kind == InodeKind::Symlink
        && blocks == 0
        && size < codec::INODE_BLOCK_BYTES as u64;
    Ok(metadata)
}

/// 计算链接数与 ctime 的新值，再预留并验证共同 inode-table 记录。
fn prepare_link_delta(
    inode: &InodeRef,
    state: &InodeState,
    delta: i32,
    now_ns: u64,
) -> Result<PreparedInodeMetadata, FsError> {
    let old = *state.metadata.lock();
    let links = if inode.is_memory() {
        memory_link_count(old.links, delta)?
    } else {
        updated_link_count(old.links, delta)?
    };
    let ctime = if inode.is_memory() {
        memory_time(now_ns)
    } else {
        codec::prepare_inode_write_times(now_ns, old.extra_isize)?.1
    };
    let mut next = old;
    next.links = links;
    next.ctime = ctime;
    prepare_inode_metadata(inode, state, old, next)
}

/// 计算 ctime 的可表示盘面值并预备当前 inode 记录。
fn prepare_inode_ctime(
    inode: &InodeRef,
    state: &InodeState,
    now_ns: u64,
) -> Result<PreparedInodeMetadata, FsError> {
    let old = *state.metadata.lock();
    let ctime = if inode.is_memory() {
        memory_time(now_ns)
    } else {
        codec::prepare_inode_write_times(now_ns, old.extra_isize)?.1
    };
    let mut next = old;
    next.ctime = ctime;
    prepare_inode_metadata(inode, state, old, next)
}

/// 记录修改 guard 先核对当前 inode 记录与 canonical 根，再锁外保留准确窗口。
fn prepare_inode_metadata(
    inode: &InodeRef,
    state: &InodeState,
    old: InodeMetadata,
    next: InodeMetadata,
) -> Result<PreparedInodeMetadata, FsError> {
    if inode.is_memory() {
        return Ok(PreparedInodeMetadata {
            inode: inode.clone(),
            storage: PreparedMetadataStorage::Memory,
            old,
            next,
        });
    }

    let volume = inode.disk_volume()?;
    volume.require_write(volume.claim())?;
    if old.flags & codec::EA_INODE != 0 {
        return Err(FsError::InvalidOperation);
    }
    codec::validate_inode_metadata_update(&old, &next)?;

    let (record, offset) = volume.inode_record(old.number, None)?;
    let root = state.disk_root().read();
    let current = record.with_read(|bytes| {
        codec::decode_inode(
            volume.geometry(),
            old.number,
            &bytes[offset..offset + volume.geometry().inode_size],
        )
    })??;
    if InodeMetadata::from_data(&current) != old || current.block != *root {
        return Err(FsError::Corrupt);
    }
    Ok(PreparedInodeMetadata {
        inode: inode.clone(),
        storage: PreparedMetadataStorage::Disk { record, offset },
        old,
        next,
    })
}

/// 写入已校验的当前记录，再发布同一份 canonical 属性快照。
fn publish_inode_metadata(inode: &InodeRef, state: &InodeState, prepared: PreparedInodeMetadata) {
    assert!(
        inode.ptr_eq(&prepared.inode),
        "inode metadata token mismatch"
    );
    assert_eq!(*state.metadata.lock(), prepared.old);

    if let PreparedMetadataStorage::Disk { ref record, offset } = prepared.storage {
        let volume = inode.disk_volume().expect("disk metadata token");
        let g = volume.geometry();
        record
            .update(volume.claim(), |bytes| {
                codec::patch_inode_metadata(
                    g,
                    &mut bytes[offset..offset + g.inode_size],
                    &prepared.old,
                    &prepared.next,
                )
            })
            .expect("prepared writable inode metadata")
            .expect("prepared inode metadata patch");
    }

    *state.metadata.lock() = prepared.next;
}

/// 只由已取得目录读侧或写侧的 guard 调用，加载子 inode 时仍保父锁。
fn lookup_locked(
    inode: &InodeRef,
    cache: &InodeCache,
    name: &[u8],
) -> Result<Option<InodeRef>, FsError> {
    if inode.is_memory() {
        return inode.lookup_memory(name);
    }
    let number =
        inode.with_map_read(|metadata, root| inode.disk_volume()?.lookup(metadata, root, name))?;
    number
        .map(|number| cache.iget(inode.disk_volume()?, number))
        .transpose()
}

impl LoadGuard<'_> {
    /// I/O 已结束后只将结果移入 once，失败解除原占位，再锁外唤醒全部等待者。
    fn complete(&mut self, result: Result<InodeData, FsError>) {
        self.publish(result);
        self.active = false;
    }

    /// 复用首次结果，展开清理不能覆盖已经发布的真实加载原因。
    fn publish(&self, result: Result<InodeData, FsError>) {
        let result = self.inode.data.call_once(|| result.map(InodeState::new));
        if result.is_err() {
            self.domain.remove_failed(self.inode);
        }
        self.inode.waiters.wake_all();
    }
}

impl Drop for LoadGuard<'_> {
    /// 尚未完成的唯一加载者退出时发布取消，等待者仍持原对象直到消费错误。
    fn drop(&mut self) {
        if self.active {
            self.publish(Err(FsError::LoadAborted));
        }
    }
}

/// 只有同一索引锁内取得的使用才能公开；已经运行的回收禁止重取原身份。
fn find(
    index: &mut InodeIndex,
    domain: &Arc<InodeDomain>,
    filesystem: &Arc<Ext4Volume>,
    number: u64,
) -> Result<Option<InodeRef>, FsError> {
    let Some(slot) = index.slots.iter_mut().flatten().find(|slot| {
        slot.inode.number == number && Arc::ptr_eq(&slot.inode.disk().filesystem, filesystem)
    }) else {
        return Ok(None);
    };
    match slot.eviction {
        Eviction::Cached => {}
        Eviction::Requested => {
            // 排队尚未取得回收执行责任；有链接的缓存命中可在同锁取消这次淘汰请求。
            let linked = slot.inode.data.get().is_some_and(|result| {
                result
                    .as_ref()
                    .is_ok_and(|state| 0 < state.metadata.lock().links)
            });
            if !linked {
                return Err(FsError::Busy);
            }
            slot.eviction = Eviction::Cached;
        }
        Eviction::Running => return Err(FsError::Busy),
        Eviction::Failed(error) => return Err(error),
    }
    Ok(Some(use_slot(domain, slot, false)))
}

/// 调用者持 canonical 索引锁，缓存强引用不计作使用；回收者与普通使用共用准确计数。
fn use_slot(domain: &Arc<InodeDomain>, slot: &mut InodeSlot, worker: bool) -> InodeRef {
    slot.users = slot.users.checked_add(1).expect("inode use count overflow");
    InodeRef(
        slot.inode.clone(),
        InodeOwner::Disk {
            domain: domain.clone(),
            worker,
        },
    )
}

/// 失败义务移到具体服务的拥有链，既不弱升级丢责任，也不建立 domain 自保持环。
fn retain_failed(domain: &Arc<InodeDomain>) {
    let mut head = FAILED_EVICTIONS.lock();
    if !domain.retained.swap(true, Ordering::Relaxed) {
        *domain.retained_next.lock() = head.take();
        *head = Some(domain.clone());
    }
}

/// 最后失败槽已经结束后摘原域，链只移动现有 Arc，最后析构在服务锁外执行。
fn release_retained(domain: &Arc<InodeDomain>) {
    let mut head = FAILED_EVICTIONS.lock();
    if !domain.retained.load(Ordering::Relaxed) {
        return;
    }
    let mut kept = None;
    let mut retired = None;
    while let Some(entry) = head.take() {
        *head = entry.retained_next.lock().take();
        if Arc::ptr_eq(&entry, domain) {
            entry.retained.store(false, Ordering::Relaxed);
            retired = Some(entry);
        } else {
            *entry.retained_next.lock() = kept.take();
            kept = Some(entry);
        }
    }
    *head = kept;
    drop(head);
    drop(retired);
}

/// 消费真实域票并回收原 inode，随后在索引锁外同步停止已关闭域的全部无槽卷。
fn evict_inodes(work: Pin<&Work>) {
    // SAFETY: queue 在 Work 接受前用 into_raw 移交一次真实域强引用；
    // callback 的这一票是唯一 from_raw 消费者，域和整数 context 指向的原分配全程存活。
    let domain = unsafe { Arc::from_raw(work.context() as *const InodeDomain) };
    loop {
        let selected = {
            let mut index = domain.index.lock();
            index
                .slots
                .iter_mut()
                .enumerate()
                .find_map(|(position, slot)| {
                    let slot = slot.as_mut()?;
                    if slot.users != 0
                        || !matches!(slot.eviction, Eviction::Requested)
                        || !slot.inode.mapping.is_cache_only()
                    {
                        return None;
                    }
                    slot.eviction = Eviction::Running;
                    Some((position, use_slot(&domain, slot, true)))
                })
        };
        let Some((position, inode)) = selected else {
            break;
        };
        let result = inode.evict();
        let victim = {
            let mut index = domain.index.lock();
            let slot = index.slots[position]
                .as_mut()
                .expect("running inode eviction lost its slot");
            assert!(
                Arc::ptr_eq(&slot.inode, &inode.0),
                "inode eviction identity changed"
            );
            match result {
                Ok(()) => {
                    assert_eq!(slot.users, 1, "evicted inode retains external users");
                    assert!(
                        slot.inode.mapping.is_empty(),
                        "evicted inode retains cached pages"
                    );
                    index.evicted = index
                        .evicted
                        .checked_add(1)
                        .expect("inode eviction count overflow");
                    index.slots[position].take()
                }
                Err(error) => {
                    slot.eviction = Eviction::Failed(error);
                    None
                }
            }
        };
        if result.is_err() {
            retain_failed(&domain);
        }
        domain.waiters.wake_all();
        drop(victim);
        drop(inode);
    }

    let failed = domain
        .index
        .lock()
        .slots
        .iter()
        .flatten()
        .any(|slot| matches!(slot.eviction, Eviction::Failed(_)));
    if !failed {
        release_retained(&domain);
    }
    domain.retire_volumes();
    domain.waiters.wake_all();
}

/// 正前缀已经被文件接受后保留它；零前缀保留具体的首次失败原因。
fn write_result(start: u64, accepted: usize, error: FsError) -> Result<(u64, usize), FsError> {
    if accepted == 0 {
        Err(error)
    } else {
        Ok((start, accepted))
    }
}

/// 有限最大文件位置由实际布局决定，传统树容量与 extent 的 32 位逻辑块号分别计算。
fn write_limit(g: &crate::fs::ext4::Geometry, flags: u32) -> u64 {
    let blocks = if flags & crate::fs::ext4::codec::EXTENTS != 0 {
        codec::EXTENT_LOGICAL_BLOCKS
    } else {
        let n = (g.block_size / 4) as u64;
        codec::DIRECT_BLOCKS as u64 + n + n * n + n * n * n
    };
    (blocks * g.block_size as u64).min(i64::MAX as u64)
}

/// 根据真实接受范围、块净增量与当前时间计算下一属性，未发表失败不修改原状态。
fn write_metadata(
    g: &crate::fs::ext4::Geometry,
    old: &InodeMetadata,
    position: u64,
    count: usize,
    added: u64,
    removed: u64,
    now: u64,
) -> Result<InodeMetadata, FsError> {
    let sectors = (g.block_size / SECTOR_BYTES as usize) as u64;
    let blocks = old
        .blocks
        .checked_add(added * sectors)
        .ok_or(FsError::FileTooLarge)?;
    let blocks = blocks
        .checked_sub(removed * sectors)
        .ok_or(FsError::Corrupt)?;
    let (_, huge) = codec::blocks_encoding(g, blocks)?;
    let (mtime, ctime) = codec::prepare_inode_write_times(now, old.extra_isize)?;

    let mut next = *old;
    next.size = old.size.max(
        position
            .checked_add(count as u64)
            .ok_or(FsError::FileTooLarge)?,
    );
    next.blocks = blocks;
    next.flags = (old.flags & !codec::HUGE_FILE) | if huge { codec::HUGE_FILE } else { 0 };
    next.mtime = mtime;
    next.ctime = ctime;
    Ok(next)
}

impl PreparedNewInode {
    /// 名字准备借用实际 canonical 对象，候选尚未增加链接数。
    pub(crate) fn inode(&self) -> &InodeRef {
        match self {
            Self::Disk(prepared) => prepared.inode(),
            Self::Memory { inode, .. } => inode.as_ref().expect("memory inode token consumed"),
        }
    }

    /// 所有可失败准备结束后发表初始链接数，内存候选取消只释放其实际拥有。
    pub(crate) fn publish(self) -> InodeRef {
        match self {
            Self::Disk(prepared) => prepared.publish(),
            Self::Memory { mut inode, next } => {
                let inode = inode.take().expect("memory inode token consumed");
                let guard = inode.inode_write().expect("prepared memory inode loaded");
                assert_eq!(
                    guard.metadata_snapshot().links,
                    0,
                    "memory inode already linked"
                );
                *guard.state.metadata.lock() = next;
                drop(guard);
                inode
            }
        }
    }
}

impl InodeMetadata {
    /// 内存属性无需盘面编码，初始时间来自原单调时钟。
    pub(crate) fn memory(
        kind: InodeKind,
        mode: u16,
        uid: u32,
        gid: u32,
        links: u32,
        now_ns: u64,
    ) -> Self {
        let now = memory_time(now_ns);
        Self {
            number: 0,
            kind,
            mode,
            size: if kind == InodeKind::Directory {
                2 * MEMORY_DIRENT_BYTES
            } else {
                0
            },
            uid,
            gid,
            links,
            blocks: 0,
            rdev: 0,
            atime: now,
            mtime: now,
            ctime: now,
            generation: 0,
            flags: 0,
            checksum_seed: 0,
            fast_symlink: false,
            file_acl: 0,
            extra_isize: 0,
        }
    }
}

impl Inode {
    /// 返回实际后端身份，不把内存实例伪装成磁盘卷。
    pub fn device(&self) -> u64 {
        match &self.backend {
            InodeBackend::Disk(disk) => disk.filesystem.device(),
            InodeBackend::Memory { filesystem, .. } => filesystem
                .upgrade()
                .expect("memory inode without filesystem owner")
                .device(),
        }
    }

    /// 内存普通文件以原页为存储颗粒，磁盘保持真实卷块大小。
    pub fn block_size(&self) -> u32 {
        match &self.backend {
            InodeBackend::Disk(disk) => disk.filesystem.geometry().block_size as u32,
            InodeBackend::Memory { .. } => PAGE_SIZE as u32,
        }
    }

    /// 内存设备号独立于文件系统设备号；普通文件没有 rdev。
    pub fn rdev(&self) -> u64 {
        self.loaded_state().metadata.lock().rdev
    }

    /// 内存来源没有磁盘回写、预读或可丢弃 file LRU 责任。
    pub fn is_memory(&self) -> bool {
        matches!(self.backend, InodeBackend::Memory { .. })
    }

    /// 特殊 OFD 构造只消费实际节点角色。
    pub fn memory_special(&self) -> Option<MemorySpecial> {
        match &self.backend {
            InodeBackend::Memory {
                content: MemoryContent::Special(special),
                ..
            } => Some(*special),
            _ => None,
        }
    }

    /// 权威目录的短锁只保护项集合，长操作已由共同目录 guard 序列化。
    fn memory_directory(&self) -> Result<&SpinLock<MemoryDirectory>, FsError> {
        match &self.backend {
            InodeBackend::Memory {
                content: MemoryContent::Directory(directory),
                ..
            } => Ok(directory),
            _ => Err(FsError::NotDirectory),
        }
    }
}

impl InodeRef {
    /// 内部根或目录 Arc 转为外部拥有时同时保活其真实文件系统。
    pub(crate) fn from_memory(inode: Arc<Inode>, fs: Arc<MemoryFs>) -> Self {
        match &inode.backend {
            InodeBackend::Memory { filesystem, .. } => assert_eq!(
                filesystem.as_ptr(),
                Arc::as_ptr(&fs),
                "memory filesystem identity changed"
            ),
            InodeBackend::Disk(_) => panic!("disk inode converted to memory reference"),
        }
        Self(inode, InodeOwner::Memory(fs))
    }

    /// 只有内存权威根和目录项可保存此内部 Arc，不复制外部文件系统拥有。
    pub(crate) fn memory_arc(&self) -> Arc<Inode> {
        assert!(self.is_memory(), "disk inode used as memory authority");
        self.0.clone()
    }

    /// fallible 构造内存节点及唯一 mapping，共同状态立即完成且没有磁盘槽。
    pub(crate) fn try_memory(
        fs: &Arc<MemoryFs>,
        metadata: InodeMetadata,
        content: MemoryContent,
    ) -> Result<Self, FsError> {
        let inode = Arc::try_new(Inode {
            backend: InodeBackend::Memory {
                filesystem: Arc::downgrade(fs),
                content,
            },
            number: metadata.number,
            data: Once::new(),
            mapping: FileMapping::new(),
            waiters: WaitQueue::new(),
            #[cfg(feature = "ktest")]
            truncate_hook: SpinLock::new(None),
            _pinned: PhantomPinned,
        })
        .map_err(|_| FsError::NoMemory)?;
        inode.data.call_once(|| Ok(InodeState::memory(metadata)));
        Ok(Self(inode, InodeOwner::Memory(fs.clone())))
    }

    /// 普通内存调用借用其强文件系统拥有，不执行弱升级猜测。
    fn memory_fs(&self) -> Result<&Arc<MemoryFs>, FsError> {
        match &self.1 {
            InodeOwner::Memory(fs) => Ok(fs),
            _ => Err(FsError::InvalidOperation),
        }
    }

    /// cache 在分配真实内存原页之前预留同一文件系统额度。
    pub(crate) fn reserve_memory_page(&self) -> Result<MemoryPageCharge, FsError> {
        self.memory_fs()?.reserve_page()
    }

    /// 普通名字修改遵守唯一后端权限，可信特殊节点构造不经过此入口。
    fn require_memory_write(&self) -> Result<(), FsError> {
        if self.memory_fs()?.mode() != AccessMode::ReadWrite {
            return Err(FsError::ReadOnly);
        }
        Ok(())
    }

    /// 父属性按 setgid/umask 规则形成候选，目标字节在链接发表前独占准备。
    #[expect(
        clippy::too_many_arguments,
        reason = "内存后端使用同一创建快照应用父 setgid、umask、身份、时间及具体内容"
    )]
    fn prepare_memory_inode(
        &self,
        parent: &InodeMetadata,
        content: NewInodeContent<'_>,
        requested_mode: u16,
        umask: u16,
        uid: u32,
        gid: u32,
        now_ns: u64,
    ) -> Result<PreparedNewInode, FsError> {
        self.require_memory_write()?;
        if parent.kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        let (kind, links, content) = match content {
            NewInodeContent::Regular => (InodeKind::Regular, 1, MemoryContent::Regular),
            NewInodeContent::Directory => (
                InodeKind::Directory,
                2,
                MemoryContent::Directory(SpinLock::new(MemoryDirectory {
                    parent: self.number(),
                    next_cookie: 2,
                    entries: Vec::new(),
                })),
            ),
            NewInodeContent::Symlink(target) => {
                if target.is_empty() || target.contains(&0) {
                    return Err(FsError::InvalidName);
                }
                if PAGE_SIZE < target.len() {
                    return Err(FsError::NameTooLong);
                }
                (
                    InodeKind::Symlink,
                    1,
                    MemoryContent::Symlink(try_bytes(target)?),
                )
            }
        };

        let setgid = parent.mode & S_ISGID != 0;
        let kind_mode = match kind {
            InodeKind::Directory => S_IFDIR,
            InodeKind::Symlink => S_IFLNK,
            InodeKind::Regular => S_IFREG,
            InodeKind::Character
            | InodeKind::Block
            | InodeKind::Fifo
            | InodeKind::Socket
            | InodeKind::Other => unreachable!("unsupported kind in a new directory entry"),
        };
        let mut mode = kind_mode
            | if kind == InodeKind::Symlink {
                S_IRWXUGO
            } else {
                requested_mode & S_IALLUGO
            };
        if kind == InodeKind::Directory && setgid {
            mode |= S_ISGID;
        } else if kind != InodeKind::Directory
            && setgid
            && mode & (S_ISGID | S_IXGRP) == (S_ISGID | S_IXGRP)
            && gid != parent.gid
        {
            mode &= !S_ISGID;
        }
        if kind != InodeKind::Symlink {
            mode &= !(umask & S_IRWXUGO);
        }

        let gid = if setgid { parent.gid } else { gid };
        let mut initial = InodeMetadata::memory(kind, mode, uid, gid, 0, now_ns);
        if let MemoryContent::Symlink(target) = &content {
            initial.size = target.len() as u64;
        }
        let inode = self.memory_fs()?.allocate(initial, content, 0)?;
        let mut next = inode.metadata_snapshot()?;
        next.links = links;
        Ok(PreparedNewInode::Memory {
            inode: Some(inode),
            next,
        })
    }

    /// 权威项保留 canonical 子 inode，点项按本目录和实际父编号查找。
    fn lookup_memory(&self, name: &[u8]) -> Result<Option<InodeRef>, FsError> {
        let fs = self.memory_fs()?;
        if name == b"." {
            return Ok(Some(self.clone()));
        }
        let (parent, child) = {
            let directory = self.memory_directory()?.lock();
            (
                directory.parent,
                directory
                    .entries
                    .iter()
                    .find(|entry| entry.name.as_slice() == name)
                    .map(|entry| entry.inode.clone()),
            )
        };
        if name == b".." {
            return fs.find(parent).map(Some);
        }
        Ok(child.map(|child| Self::from_memory(child, fs.clone())))
    }

    /// 未发表内存节点沿共同目录写锁安装，目录增加父链接，特殊叶保持原链接数。
    pub(crate) fn initialize_memory_child(
        &self,
        name: &[u8],
        child: &InodeRef,
    ) -> Result<(), FsError> {
        let guard = self.directory_write()?;
        let kind = child.metadata_snapshot()?.kind;
        let prepared = guard.prepare_memory_edit(
            MemoryEdit::Insert(name, child.number(), kind),
            i32::from(kind == InodeKind::Directory),
            time::realtime(),
            true,
        )?;
        guard.publish(prepared);
        Ok(())
    }

    /// 写能力自动选择真实后端；磁盘 claim 只在磁盘分支借用。
    fn write_backend(
        &self,
        offset: u64,
        append: bool,
        source: &mut WriteBuffer<'_>,
    ) -> Result<(u64, usize), FsError> {
        if self.is_memory() {
            return self.write_memory(offset, append, source);
        }
        self.write_from(self.disk_volume()?.claim(), offset, append, source)
    }

    /// 所有截断调用共享同一能力；内存分支不取得或生成磁盘票。
    fn truncate_backend(&self, new_size: u64) -> Result<(), FsError> {
        if self.is_memory() {
            return self.truncate_memory(new_size);
        }
        self.truncate(self.disk_volume()?.claim(), new_size)
    }

    /// 普通字节写保共同 write/invalidate，输入 fault 不携目的内容锁。
    fn write_memory(
        &self,
        offset: u64,
        append: bool,
        source: &mut WriteBuffer<'_>,
    ) -> Result<(u64, usize), FsError> {
        self.require_memory_write()?;
        let state = self.wait_loaded()?;
        let _scope = ReclaimScope::nofs();
        let _write = state.write.write();
        let old = *state.metadata.lock();
        if old.kind == InodeKind::Directory {
            return Err(FsError::IsDirectory);
        }
        if old.kind != InodeKind::Regular || self.memory_special().is_some() {
            return Err(FsError::InvalidOperation);
        }
        assert!(0 < *state.writecount.lock(), "memory write has no access");

        let start = if append { old.size } else { offset };
        let remaining = source.len() - source.copied();
        let mut accepted = 0usize;
        let mut input = [0u8; 512];
        let mut attempt = ReclaimAttempt::new();
        while accepted < remaining {
            let position = match start.checked_add(accepted as u64) {
                Some(position) if position < i64::MAX as u64 => position,
                _ => return write_result(start, accepted, FsError::FileTooLarge),
            };
            let count = (remaining - accepted)
                .min(input.len())
                .min(PAGE_SIZE - (position % PAGE_SIZE as u64) as usize)
                .min((i64::MAX as u64 - position) as usize);
            let pulled = match source.pull(&mut input[..count]) {
                Ok(0) => break,
                Ok(count) => count,
                Err(error) => return write_result(start, accepted, error),
            };

            let _invalidate = state.invalidate.write();
            let current = *state.metadata.lock();
            if current.size < position {
                if let Err(error) = self.zero_memory_tail(current.size, position) {
                    return write_result(start, accepted, error);
                }
            }
            let folio = match self.read_page(position / PAGE_SIZE as u64, &mut attempt, None) {
                Ok(folio) => folio,
                Err(error) => return write_result(start, accepted, error),
            };
            let mut content = folio.lock_content();
            if let Err(error) =
                content.copy_from((position % PAGE_SIZE as u64) as usize, &input[..pulled])
            {
                return write_result(start, accepted, error.into());
            }

            let mut next = current;
            next.size = current.size.max(position + pulled as u64);
            next.mtime = memory_time(time::realtime());
            next.ctime = next.mtime;
            *state.metadata.lock() = next;
            accepted += pulled;
        }
        Ok((start, accepted))
    }

    /// 只零化已有尾页，缺页本来就是空洞；撤共享写许可后才修改原页字节。
    fn zero_memory_tail(&self, start: u64, end: u64) -> Result<(), FsError> {
        let offset = (start % PAGE_SIZE as u64) as usize;
        if offset == 0 || end <= start {
            return Ok(());
        }
        let index = start / PAGE_SIZE as u64;
        if let Some(folio) = self.mapping.cached_folio(index) {
            let mut content = folio.lock_content();
            content.range_ready(0, PAGE_SIZE)?;
            self.mapping.mkclean(index, &folio, &mut content);
            let count = (end - start).min((PAGE_SIZE - offset) as u64) as usize;
            content.zero_file_range(offset, count)?;
        }
        Ok(())
    }

    /// 尺寸发表后两遍 even_cows 撤叶/CSD 包围原 mapping 清理，原页不登记 Dirty/WB。
    fn truncate_memory(&self, new_size: u64) -> Result<(), FsError> {
        self.require_memory_write()?;
        if (i64::MAX as u64) < new_size {
            return Err(FsError::FileTooLarge);
        }
        let state = self.wait_loaded()?;
        let _scope = ReclaimScope::nofs();
        let _write = state.write.write();
        let old = *state.metadata.lock();
        if old.kind != InodeKind::Regular || self.memory_special().is_some() {
            return Err(FsError::InvalidOperation);
        }
        assert!(
            0 < *state.writecount.lock(),
            "memory truncate has no access"
        );
        let _invalidate = state.invalidate.write();
        if old.size < new_size {
            self.zero_memory_tail(old.size, new_size)?;
        }

        let mut next = old;
        next.size = new_size;
        next.mtime = memory_time(time::realtime());
        next.ctime = next.mtime;
        *state.metadata.lock() = next;

        let first = new_size.div_ceil(PAGE_SIZE as u64) * PAGE_SIZE as u64;
        self.mapping.unmap_range(first..u64::MAX, true);
        #[cfg(feature = "ktest")]
        self.truncate_phase(TruncatePhase::AfterFirstUnmap);
        let _writeback = self.mapping.lock_writeback();
        // SAFETY: 共同 write/invalidate 独占与 mapping WB 互斥仍持有；
        // 新 size 已发表且首遍 even_cows 撤叶/CSD 已结束，原填充由内容锁排空；
        // 内存原页没有设备使用，第二遍撤叶仍在释放失效保护前执行。
        let pages = unsafe { self.mapping.truncate_pages(self, new_size, PAGE_SIZE) };
        #[cfg(feature = "ktest")]
        self.truncate_phase(TruncatePhase::BeforeSecondUnmap);
        self.mapping.unmap_range(first..u64::MAX, true);
        pages
    }

    /// 内存共享写准备只取得同一原页并核尺寸，不准备 sector 或磁盘记录。
    fn prepare_memory_mkwrite(
        &self,
        index: u64,
        attempt: &mut ReclaimAttempt,
    ) -> Result<PreparedMkwrite, FsError> {
        self.require_memory_write()?;
        let _access = self.acquire_write()?;
        loop {
            let mut prepared = Some(self.mapping.prepare_file(self, index, attempt)?);
            let result = (|| {
                let state = self.wait_loaded()?;
                let _scope = ReclaimScope::nofs();
                let _write = state.write.write();
                let _invalidate = state.invalidate.read();
                let metadata = *state.metadata.lock();
                let start = index
                    .checked_mul(PAGE_SIZE as u64)
                    .ok_or(FsError::FileTooLarge)?;
                if metadata.kind != InodeKind::Regular
                    || metadata.size <= start
                    || self.memory_special().is_some()
                {
                    return Err(FsError::InvalidOperation);
                }
                Ok(self
                    .fill_prepared(index, &mut prepared, None)?
                    .map(|folio| PreparedMkwrite {
                        folio,
                        index,
                        size: metadata.size,
                        prepared_sectors: 0,
                    }))
            })();
            drop(prepared);
            match result {
                Ok(Some(prepared)) => return Ok(prepared),
                Ok(None) => continue,
                Err(error) => return Err(error),
            }
        }
    }
}

/// 将内存时间拆成共同秒/纳秒表示，不受 ext4 记录宽度约束。
fn memory_time(now_ns: u64) -> InodeTime {
    InodeTime {
        seconds: (now_ns / time::NSEC_PER_SEC) as i64,
        nanoseconds: (now_ns % time::NSEC_PER_SEC) as u32,
    }
}

/// fallible 拥有原始字节，内存名字和链接不要求 UTF-8。
fn try_bytes(bytes: &[u8]) -> Result<Vec<u8>, FsError> {
    let mut copy = Vec::new();
    copy.try_reserve_exact(bytes.len())
        .map_err(|_| FsError::NoMemory)?;
    copy.extend_from_slice(bytes);
    Ok(copy)
}

/// 目录编辑描述只借调用者字节，所有拥有和分配在发表前完成。
enum MemoryEdit<'a> {
    Insert(&'a [u8], u64, InodeKind),
    Remove(&'a [u8]),
    Replace(&'a [u8], u64, InodeKind),
    Move(&'a [u8], &'a [u8], u64, InodeKind, Option<(u64, InodeKind)>),
    Dotdot(u64),
}

impl MemoryDirectory {
    /// 目录 guard 排斥编辑；先锁外预留，再短锁复制实际项拥有。
    fn try_snapshot(source: &SpinLock<Self>) -> Result<Self, FsError> {
        let len = source.lock().entries.len();
        let mut entries = Vec::new();
        entries
            .try_reserve_exact(len.checked_add(1).ok_or(FsError::NoSpace)?)
            .map_err(|_| FsError::NoMemory)?;

        let source = source.lock();
        entries.extend(source.entries.iter().map(|entry| MemoryEntry {
            name: entry.name.clone(),
            inode: entry.inode.clone(),
            kind: entry.kind,
            cookie: entry.cookie,
        }));
        Ok(Self {
            parent: source.parent,
            next_cookie: source.next_cookie,
            entries,
        })
    }

    /// 真实名称匹配返回其权威位置，cookie 不由此数组下标推导。
    fn position(&self, name: &[u8]) -> Result<usize, FsError> {
        self.entries
            .iter()
            .position(|entry| entry.name.as_slice() == name)
            .ok_or(FsError::NotFound)
    }

    /// 候选项强持真实 canonical 子节点，新名字取得未复用的单调 cookie。
    fn insert(
        &mut self,
        parent: &InodeRef,
        name: &[u8],
        number: u64,
        kind: InodeKind,
    ) -> Result<(), FsError> {
        validate_memory_name(name)?;
        if self
            .entries
            .iter()
            .any(|entry| entry.name.as_slice() == name)
        {
            return Err(FsError::Exists);
        }
        let next_cookie = self.next_cookie.checked_add(1).ok_or(FsError::NoSpace)?;
        let inode = parent.memory_fs()?.find(number)?;
        if inode.metadata_snapshot()?.kind != kind {
            return Err(FsError::InvalidOperation);
        }

        let name = Arc::try_new(try_bytes(name)?).map_err(|_| FsError::NoMemory)?;
        self.entries.push(MemoryEntry {
            name,
            inode: inode.memory_arc(),
            kind,
            cookie: self.next_cookie,
        });
        self.next_cookie = next_cookie;
        Ok(())
    }

    /// 只在私有候选中移走项，现有 fd/VMA 和旧权威目录尚不受影响。
    fn remove(&mut self, name: &[u8]) -> Result<MemoryEntry, FsError> {
        validate_memory_name(name)?;
        let position = self.position(name)?;
        Ok(self.entries.remove(position))
    }

    /// 覆盖保留目标 cookie，取得源真实 Arc 后才替换候选项。
    fn replace(
        &mut self,
        parent: &InodeRef,
        name: &[u8],
        number: u64,
        kind: InodeKind,
    ) -> Result<(), FsError> {
        validate_memory_name(name)?;
        let position = self.position(name)?;
        let inode = parent.memory_fs()?.find(number)?;
        if inode.metadata_snapshot()?.kind != kind {
            return Err(FsError::InvalidOperation);
        }
        self.entries[position].inode = inode.memory_arc();
        self.entries[position].kind = kind;
        Ok(())
    }

    /// 同父 rename 在同一候选中核源和目标，再移项或覆盖，不产生两份旧快照。
    fn move_entry(
        &mut self,
        parent: &InodeRef,
        old: &[u8],
        new: &[u8],
        number: u64,
        kind: InodeKind,
        target: Option<(u64, InodeKind)>,
    ) -> Result<bool, FsError> {
        validate_memory_name(old)?;
        validate_memory_name(new)?;
        let source = self.position(old)?;
        if self.entries[source].inode.number() != number || self.entries[source].kind != kind {
            return Err(FsError::InvalidOperation);
        }
        if old == new {
            return Ok(false);
        }
        match (self.position(new), target) {
            (Ok(position), Some((target_number, target_kind))) => {
                let actual = &self.entries[position];
                if actual.inode.number() != target_number || actual.kind != target_kind {
                    return Err(FsError::InvalidOperation);
                }
                if number == target_number {
                    return Ok(false);
                }
                self.replace(parent, new, number, kind)?;
                self.remove(old)?;
            }
            (Err(FsError::NotFound), None) => {
                self.remove(old)?;
                self.insert(parent, new, number, kind)?;
            }
            _ => return Err(FsError::InvalidOperation),
        }
        Ok(true)
    }
}

impl DirectoryWriteGuard<'_> {
    /// 内存编辑的全部分配、链接计数和时间先准备，发表阶段只交换实际权威内容。
    fn prepare_memory_edit(
        &self,
        edit: MemoryEdit<'_>,
        links_delta: i32,
        now_ns: u64,
        initializing: bool,
    ) -> Result<PreparedDirectoryMutation, FsError> {
        if !initializing {
            self.inode.require_memory_write()?;
        }
        let mut directory = MemoryDirectory::try_snapshot(self.inode.memory_directory()?)?;
        let changed = match edit {
            MemoryEdit::Insert(name, number, kind) => {
                directory.insert(self.inode, name, number, kind)?;
                true
            }
            MemoryEdit::Remove(name) => {
                directory.remove(name)?;
                true
            }
            MemoryEdit::Replace(name, number, kind) => {
                directory.replace(self.inode, name, number, kind)?;
                true
            }
            MemoryEdit::Move(old, new, number, kind, target) => {
                directory.move_entry(self.inode, old, new, number, kind, target)?
            }
            MemoryEdit::Dotdot(parent) => {
                let inode = self.inode.memory_fs()?.find(parent)?;
                if inode.metadata_snapshot()?.kind != InodeKind::Directory {
                    return Err(FsError::NotDirectory);
                }
                let changed = directory.parent != parent;
                directory.parent = parent;
                changed
            }
        };

        let mut next = self.metadata_snapshot();
        if changed {
            next.links = memory_link_count(next.links, links_delta)?;
            next.size = (directory.entries.len() as u64)
                .checked_add(2)
                .and_then(|entries| entries.checked_mul(MEMORY_DIRENT_BYTES))
                .ok_or(FsError::NoSpace)?;
            next.mtime = memory_time(now_ns);
            next.ctime = next.mtime;
        } else if links_delta != 0 {
            return Err(FsError::InvalidOperation);
        }
        Ok(PreparedDirectoryMutation::Memory {
            parent: self.inode.clone(),
            directory: if changed { Some(directory) } else { None },
            next,
        })
    }
}

impl InodeRef {
    /// 内存游标包含点项和单调项 cookie，洞和删除项不导致现存名字重复。
    fn memory_dir_next(&self, cursor: &DirectoryCursor) -> Result<Option<DirectoryEntry>, FsError> {
        let DirectoryCursor::Memory(cookie) = *cursor else {
            return Err(FsError::InvalidOperation);
        };
        let (number, name, kind, next_cookie) = match cookie {
            0 => (self.number(), try_bytes(b".")?, InodeKind::Directory, 1),
            1 => (
                self.memory_directory()?.lock().parent,
                try_bytes(b"..")?,
                InodeKind::Directory,
                2,
            ),
            _ => {
                let entry = {
                    let directory = self.memory_directory()?.lock();
                    directory
                        .entries
                        .iter()
                        .filter(|entry| cookie <= entry.cookie)
                        .min_by_key(|entry| entry.cookie)
                        .map(|entry| {
                            (
                                entry.inode.number(),
                                entry.name.clone(),
                                entry.kind,
                                entry.cookie + 1,
                            )
                        })
                };
                let Some((number, name, kind, next_cookie)) = entry else {
                    return Ok(None);
                };
                (number, try_bytes(name.as_slice())?, kind, next_cookie)
            }
        };
        Ok(Some(DirectoryEntry {
            number,
            name,
            kind: Some(kind),
            next_cookie,
            next_cursor: DirectoryCursor::Memory(next_cookie),
        }))
    }
}

/// 内存硬链接数保共同 u32 属性宽度，减计数不得下溢。
fn memory_link_count(current: u32, delta: i32) -> Result<u32, FsError> {
    if delta < 0 {
        current
            .checked_sub(delta.unsigned_abs())
            .ok_or(FsError::Corrupt)
    } else {
        current
            .checked_add(delta as u32)
            .ok_or(FsError::TooManyLinks)
    }
}

/// 组件长度和字节约束在权威编辑边界检查，点项只能通过专用父更新修改。
fn validate_memory_name(name: &[u8]) -> Result<(), FsError> {
    if name.is_empty() || name.contains(&0) || name.contains(&b'/') || name == b"." || name == b".."
    {
        return Err(FsError::InvalidName);
    }
    if NAME_MAX < name.len() {
        return Err(FsError::NameTooLong);
    }
    Ok(())
}
