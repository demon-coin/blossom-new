//! VFS 权威索引节点（Canonical Inode）管理、单飞加载缓存与双后端（ext4 / 内存文件系统）同步协议。
//!
//! ## 模块职责与架构定位
//! 本模块位于虚拟文件系统（VFS）核心层，为每个文件、目录、符号链接或特殊节点维护在所属挂载域内唯一的
//! 运行态 [`Inode`] 实例，并统一封装磁盘文件系统（`ext4`）与内存文件系统（`MemoryFs`）的读写、截断、
//! `mmap` 写缺页（`mkwrite`）、属性修改与目录项操作。
//!
//! - **上游调用者**：目录项缓存（`dentry`）、打开文件描述（OFD）与系统调用层、虚拟内存缺页/回写/页面回收子系统（`mm` / `cache` / `writeback`）。
//! - **下游依赖**：`ext4` 卷与编解码器（`Ext4Volume`、`codec`）、内存文件系统（`MemoryFs`）、页面缓存映射（`FileMapping`）以及后台工作队列（`WorkQueue`）。
//!
//! ## 核心设计与状态机
//!
//! ### 1. 单飞（Singleflight）加载与缓存槽回收状态机
//! `InodeDomain` 采用预分配固定容量的槽数组管理磁盘 `Inode`。并发 `iget` 查找同一 `(volume, ino)` 时，
//! 只有一个胜出任务在槽内发布未初始化的候选 `Inode` 并执行磁盘读取，其余并发调用者在 `inode.waiters` 上睡眠等待；
//! 当最后一个外部引用（`InodeRef`）释放或缓存域关闭时，槽位在 `index` 自旋锁内推进回收状态机，并交由常驻工作队列异步执行落盘或块释放：
//!
//! ```text
//!                        iget 胜出占位 (data = Once::new(), users = 1)
//!                                         │
//!                      ┌──────────────────┴──────────────────┐
//!               磁盘读取成功                             磁盘读取失败 / 提前中止
//!                      ▼                                     ▼
//!              Eviction::Cached                     remove_failed 立即清空槽位
//!           (data = Ok(InodeState))
//!                      │
//!       prune_unused / 缓存关闭 / links==0 且最后一个 InodeRef 释放
//!                      │ (若新的 iget 命中且 links > 0，可从 Requested 恢复为 Cached)
//!                      ▼
//!             Eviction::Requested ──(提交 WorkQueue 任务)──► Eviction::Running
//!                                                                  │
//!                                                    ┌─────────────┴─────────────┐
//!                                                 evict 成功                  evict 失败
//!                                                    ▼                           ▼
//!                                              清空槽位 (None)          Eviction::Failed(err)
//!                                                                    (挂入 FAILED_EVICTIONS 保活，
//!                                                                     保留脏数据与删除义务防止静默丢失)
//! ```
//!
//! ### 2. 写打开计数与执行互斥（`writecount` 编码）
//! `InodeState::writecount` 使用单个有符号整数在自旋锁内实现 POSIX `ETXTBSY` 互斥语义：
//! - `writecount > 0`：表示当前持有的 [`WriteAccess`] 令牌数（以写模式打开的文件描述数），此时禁止 `execve` 映射执行（`deny_writes` 返回 `TextBusy`）。
//! - `writecount == 0`：空闲状态，既无写打开也无活跃执行映像。
//! - `writecount < 0`：绝对值表示当前活跃的 [`ExecAccess`] 令牌数（正在运行的可执行映像地址空间数，`fork` 时递减、`exit`/`exec` 释放时递增），此时禁止任何写打开（`acquire_write` 返回 `TextBusy`）。
//!
//! ### 3. 两阶段准备与无失败发布（Prepare-then-Publish）
//! 目录编辑、新节点创建、属性更新与块映射扩展均先在睡眠锁保护下完成内存分配、块位图预留、目录块分裂及 `inode-table` 缓冲区读取校验，
//! 构造出带 `Drop` 自动回滚能力的 `Prepared*` 凭据；待全部可能失败的 I/O 与校验通过后，再进入最终发布阶段一气呵成更新磁盘缓冲区与内存快照。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 为避免跨虚拟内存、页面缓存、文件系统与工作队列的死锁，必须严格遵守以下锁层级顺序：
//!
//! 1. **地址空间 / VMA 锁**（由外部 `mm` 调用方在缺页路径持有；但 `prepare_mkwrite` 与用户缓冲区拷贝必须在外部无页表锁时先完成）
//! 2. **`InodeState::write`（`SleepRwLock`）**：单节点内容写、截断、属性变更及目录读写的顶层睡眠锁（持锁前须进入 `ReclaimScope::nofs()` 防止内存回收重入文件系统）。跨多目录操作时由上层 `dentry` 按祖先到后代及固定顺序加锁。
//! 3. **`InodeState::invalidate`（`SleepRwLock`）**：协调文件截断/页淘汰（写锁）与普通读取/缺页填充/`finish_mkwrite`（读锁）的睡眠读写锁。
//! 4. **`Ext4Volume` 元数据回写锁（`metadata_cache().lock_writeback()`）**：截断或释放扩展属性（EA）时串行化元数据回写。
//! 5. **`InodeStorage::Disk::root`（`SleepRwLock<[u8; 60]>`）与 `FileMapping` 回写锁（`mapping.lock_writeback()`）**：
//!    **严禁在持有 `root` 锁期间等待文件数据页回写或调用 `mapping.lock_writeback()`**！因为后台数据回写路径可能需要获取 `root` 读锁查询块映射；截断路径必须先释放 `root` 锁完成页面截断与回写等待，再重新获取 `root` 写锁提交新块树根。
//! 6. **页内容锁（`FolioRef::lock_content`）**：保护单个物理页（Folio）的数据读写与脏/准备状态位，可在持有 `root` 写锁时短暂获取以原子完成新块清零与状态标记。
//! 7. **叶子级自旋锁（`SpinLock`，持锁时间极短、严禁睡眠或触发 I/O）**：
//!    - `FAILED_EVICTIONS` -> `InodeDomain::retained_next`（失败域保活链表锁）
//!    - `InodeDomain::index`（权威缓存槽索引锁；持锁期间仅可读取内层 `InodeState::metadata` 检查 `links`）
//!    - `InodeState::metadata`、`InodeState::writecount`、`MemoryDirectory` 自旋锁。

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

/// 内存文件系统目录中每个目录项的记账字节大小（`.`、`..` 与普通子项均按 20 字节计入目录 `size`）。
const MEMORY_DIRENT_BYTES: u64 = 20;

/// 测试专用的节点加载钩子回调类型，用于在节点占位发布后、实际读取磁盘前注入并发交错。
#[cfg(feature = "ktest")]
type LoadHook = Arc<dyn Fn(&InodeRef) + Send + Sync>;

/// 磁盘文件系统的权威 `Inode` 缓存控制器，管理固定容量的缓存域与异步回收队列。
/// ponytail: 固定预算内线性查找，预算显著扩大且实测查找成为瓶颈时改为有限散列表。
pub struct InodeCache {
    /// 底层共享的缓存域，承载实际的索引槽数组、卷绑定与后台回收任务。
    domain: Arc<InodeDomain>,
    /// 测试钩子：在胜出加载者发布槽位后、执行磁盘读取前触发的回调，由自旋锁保护。
    #[cfg(feature = "ktest")]
    load_hook: SpinLock<Option<LoadHook>>,
    /// 测试统计：实际赢得单飞竞争并向底层卷发起磁盘加载的累计次数。
    #[cfg(feature = "ktest")]
    loads: AtomicUsize,
}

/// 权威 `Inode` 缓存域，统一管理固定容量的节点槽位、关联的 `ext4` 卷及后台异步回收状态。
pub(crate) struct InodeDomain {
    /// 保护缓存槽位、共享目录项缓存弱引用、卷绑定列表与派发计数的自旋锁。
    index: SpinLock<InodeIndex>,
    /// 预先分配并绑定到全局回收工作队列的常驻 `Work` 实例，避免在释放引用时动态分配内存。
    work: Once<Pin<Arc<Work>>>,
    /// 当目标节点正在回收或缓存域正在等待完全退场时，供调用任务睡眠等待的队列。
    waiters: WaitQueue,
    /// 标记当前缓存域是否因存在回收失败的节点而被挂入全局 `FAILED_EVICTIONS` 保活链表。
    retained: AtomicBool,
    /// 全局 `FAILED_EVICTIONS` 单向侵入式链表中的下一个缓存域指针，由 `FAILED_EVICTIONS` 锁顺序保护。
    retained_next: SpinLock<Option<Arc<InodeDomain>>>,
}

/// 受 `InodeDomain::index` 自旋锁保护的内部索引表与生命周期计数器。
struct InodeIndex {
    /// 当前活跃命名空间共享的目录项缓存弱引用；当所有外部强引用释放后允许按新容量重建。
    dentry_cache: Weak<DentryCache>,
    /// 固定容量的权威 `Inode` 槽位数组，槽位总数等于缓存创建时指定的容量预算。
    slots: Vec<Option<InodeSlot>>,
    /// 已绑定到本缓存域的 `Ext4Volume` 列表，容量上限与 `slots` 相同，保留至显式卸载或域关闭后同步停机。
    volumes: Vec<Arc<Ext4Volume>>,
    /// 标记上层 `InodeCache` 所有者是否仍然存活；变为 `false` 后禁止新卷绑定与弱引用升级。
    cache_open: bool,
    /// 累计已成功完成回收并清空槽位的节点次数，供 `prune_unused` 计算本轮实际回收数量。
    evicted: usize,
    /// 已在索引锁内登记但尚未完成 `WorkQueue::queue` 入队动作的派发中任务数；归零前缓存域不可宣告彻底退场。
    dispatching: usize,
}

/// 缓存域中的单个权威 `Inode` 槽位记录。
struct InodeSlot {
    /// 槽位持有的底层权威 `Inode` 强引用（不计入外部活跃引用计数 `users`）。
    inode: Arc<Inode>,
    /// 当前存活的外部 `InodeRef` 句柄数量（包含普通调用者与正在执行回收的后台任务句柄）。
    users: usize,
    /// 该槽位在异步淘汰与落盘生命周期中的当前状态。
    eviction: Eviction,
    /// 若此节点是某个卷的根目录，记录其根 `Dentry` 的弱引用以便复用，且不会阻止根目录项与节点被回收。
    root_dentry: Weak<Dentry>,
}

/// 缓存槽位的异步回收状态机枚举。
#[derive(Clone, Copy)]
enum Eviction {
    /// 正常驻留缓存中（可能正在单飞加载或已完成加载），可被查找命中。
    Cached,
    /// 已标记请求回收，等待后台 `Work` 认领；若 `links > 0` 且在认领前被再次查找命中，可恢复为 `Cached`。
    Requested,
    /// 后台回收任务已认领并正在执行数据同步、页面清理或 `links == 0` 的磁盘块释放，此时排斥任何新查找。
    Running,
    /// 回收过程中发生 I/O 或文件系统错误，保留槽位与具体错误码以防止脏数据或待释放磁盘块丢失。
    Failed(FsError),
}

impl InodeSlot {
    /// 绑定或复用卷根目录的 `Dentry`：若已有存活的根 `Dentry` 则优先升级返回，并将多余的候选对象交给调用者在锁外释放。
    fn bind_root(&mut self, candidate: DentryRef) -> (DentryRef, Option<DentryRef>) {
        if let Some(root) = DentryRef::upgrade(&self.root_dentry) {
            (root, Some(candidate))
        } else {
            self.root_dentry = candidate.downgrade();
            (candidate, None)
        }
    }
}

/// 根据权威 `Inode` 的指针身份在索引表中查找其所在的槽位下标。
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

/// 全局常驻的 `Inode` 异步回收工作队列，运行在具有内存回收（`reclaim`）属性的内核工作线程上。
static EVICTION_QUEUE: Once<WorkQueue> = Once::new();
/// 全局回收失败缓存域保活链表头：当某个缓存域存在 `Eviction::Failed` 槽位时由该静态链表强引用保活，避免形成自引用环。
static FAILED_EVICTIONS: SpinLock<Option<Arc<InodeDomain>>> = SpinLock::new(None);

/// 在内核启动阶段（共享工作线程池就绪后、开放文件系统运行前）初始化全局 `Inode` 异步回收队列与后台回写服务。
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
    /// 创建指定容量预算的缓存域，预先分配全部槽位、卷数组及常驻回收 `Work`，确保后续提交回收与释放引用时零内存分配。
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

    /// 将 `Ext4Volume` 绑定到当前缓存域（调用方须持有卷自身的绑定锁并确认旧域已解绑）；若域已关闭或槽位预算已满则返回错误。
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

    /// 若目标卷在当前缓存域中已无任何驻留 `Inode` 槽位，则从绑定列表中取出其 `Arc` 供调用方在锁外执行同步停机（调用方须持卷绑定锁）。
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

    /// 在后台工作线程上下文中，遍历已关闭的缓存域，将不再拥有任何 `Inode` 槽位的卷逐个移出并同步停止其回写服务。
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
            // 当前后台 Work 持有的域强引用保证了域身份在停机期间存活，防止新缓存域在旧回写服务完全停止前抢先绑定该卷。
            volume.writeback_domain().stop_sync();
        }
    }

    /// 将本缓存域的回收任务排入全局工作队列；每次成功入队会向 `Work` 移交一份独立的 `Arc<InodeDomain>` 强引用所有权。
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
            // SAFETY: `owner` 由上方 `Arc::into_raw(self.clone())` 刚刚构造；`queue()` 返回 `false`
            // 说明该工作项已在队列中排队而未接收本次所有权，当前线程是该裸指针的唯一持有者，立即重建 `Arc` 释放即可平衡引用计数。
            drop(unsafe { Arc::from_raw(owner) });
        }
        {
            let mut index = self.index.lock();
            assert!(0 < index.dispatching, "inode eviction dispatch lost");
            index.dispatching -= 1;
        }
        self.waiters.wake_all();
    }

    /// 阻塞等待当前已入队的回收工作执行完毕；若域内任意槽位处于回收失败状态，则返回首个记录的错误码。
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

    /// 返回缓存槽位数组的总长度，作为后台回写扫描脏节点的固定上界。
    pub(crate) fn dirty_inode_limit(&self) -> usize {
        self.index.lock().slots.len()
    }

    /// 按槽位下标为后台回写获取仍带有脏标记且未进入不可逆回收阶段的 `InodeRef`。
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

    /// 按 `(filesystem, number)` 查找或单飞加载权威 `Inode`（需在可调度任务上下文调用）；若目标节点正在被后台回收则睡眠等待其完成后重试。
    pub(crate) fn iget(
        self: &Arc<Self>,
        filesystem: &Arc<Ext4Volume>,
        number: u64,
        _cache: Option<&InodeCache>,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<InodeRef, FsError> {
        sched::assert_switchable();
        // Step 1: 绑定卷到本缓存域，并优先在现有缓存槽中查找。
        filesystem.bind_inode_domain(self)?;
        if let Some(inode) = self.find_loaded(filesystem, number)? {
            inode.wait_loaded()?;
            return Ok(inode);
        }
        if self.is_full() {
            // Step 2: 槽位已满时，仅普通外部调用者（`_cache.is_some()`）允许触发同步淘汰；
            // 后台回收或内部扩展属性加载路径直接返回内存不足，避免工作线程递归等待自身 WorkQueue 导致死锁。
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

        // Step 3: 在锁外预先分配候选节点，尝试在索引锁内单飞占位发布。
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

        // Step 4: 当前线程赢得单飞加载权，在锁外读取磁盘记录并唤醒所有等待该节点的并发任务。
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

    /// 在索引锁内查找已驻留槽位；若目标节点正处于后台回收状态（返回 `Busy`），则注册到等待队列睡眠直至回收状态推进。
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
            // SAFETY:
            // 1. `self` 是 `&Arc<InodeDomain>`，保证 `self.waiters` 在整个等待与注销期间存活且地址稳定；
            // 2. `node` 通过 `pin!` 固定在当前栈帧上，并在栈帧退出前由 `drop(registration)` 从等待队列摘除。
            let registration = unsafe { self.waiters.register(node.as_mut()) };
            let result = find(&mut self.index.lock(), self, filesystem, number);
            if result.as_ref().err() != Some(&FsError::Busy) {
                return result;
            }
            waiter.park();
            drop(registration);
        }
    }

    /// 将锁外构造的候选节点尝试发布到空闲缓存槽中：
    /// - 若期间其他线程已抢先发布同一节点，返回 `Ok(Some(existing))`；
    /// - 若当前候选成功占槽并激活 `loader`，返回 `Ok(None)`，此后调用方必须负责完成加载结果发布。
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

    /// 供刚分配到磁盘 `Inode` 位图号的创建者调用：在新节点格式化写入前，等待具有相同编号的旧 `Inode` 彻底完成后台回收。
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
            // SAFETY:
            // 1. 调用方持有 `InodeDomain` 的存活引用，保证等待队列 `self.waiters` 在函数返回前有效；
            // 2. `node` 通过 `pin!` 固定在当前栈帧，且 `registration` 在 `node` 与 `waiter` 析构前显式注销。
            let registration = unsafe { self.waiters.register(node.as_mut()) };
            if self.inode_reusable(filesystem, number)? {
                return Ok(());
            }
            waiter.park();
            drop(registration);
        }
    }

    /// 在索引锁内检查指定磁盘节点号是否可安全复用：已不在槽中返回 `Ok(true)`，正在回收返回 `Ok(false)`，
    /// 回收失败返回对应错误，若仍作为活跃缓存驻留则说明位图与缓存状态不一致（返回 `Corrupt`）。
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

    /// 当单飞磁盘读取失败或提前中止时，在索引锁内核对指针身份后移除占位槽，并在锁外唤醒等待者、释放对象。
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

    /// 扫描所有无外部引用且页面缓存仅含干净缓存页的槽位，标记回收并等待后台工作线程执行完毕，返回实际腾出的槽位数。
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
            // 仅统计成功清空的槽位数；若个别节点因回写错误转入 `Failed` 状态，其槽位与错误码会继续保留。
            let _ = self.flush();
        }
        self.index.lock().evicted - before
    }

    /// 检查固定容量的缓存槽是否已全部被占用（包含正在加载、正常缓存及回收失败保留的节点）。
    fn is_full(&self) -> bool {
        self.index.lock().slots.iter().all(Option::is_some)
    }

    /// 尝试升级并返回当前缓存域关联的共享 `DentryCache`；若调用方请求的容量与现有实例不一致则返回 `InvalidOperation`。
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

    /// 在索引自旋锁内原子仲裁并发布共享 `DentryCache` 与卷根 `Dentry`，将竞争落选的候选对象移出锁外安全析构。
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

    /// 尝试从根 `Inode` 所在槽位升级获取仍存活的卷根 `Dentry`。
    fn canonical_root(&self, inode: &InodeRef) -> Option<DentryRef> {
        let index = self.index.lock();
        let slot = index.slots[root_slot_position(&index, inode)]
            .as_ref()
            .expect("canonical root inode slot disappeared");
        DentryRef::upgrade(&slot.root_dentry)
    }

    /// 在索引自旋锁内为根 `Inode` 绑定或复用根 `Dentry`，并在锁外释放多余的候选对象。
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

/// 权威 `Inode` 的外部强引用句柄（元组字段 `0` 为底层唯一运行态 `Arc<Inode>`，字段 `1` 为维护缓存域活跃计数或内存文件系统所有权的 `InodeOwner`）。
pub struct InodeRef(Arc<Inode>, InodeOwner);

/// `InodeRef` 持有的后端所有权标识。
#[derive(Clone)]
enum InodeOwner {
    /// 磁盘 `ext4` 节点句柄，强引用所属 `InodeDomain` 以保证索引槽与回收队列存活。
    Disk {
        /// 节点驻留的权威缓存域。
        domain: Arc<InodeDomain>,
        /// 标记该句柄是否由后台回收线程（`evict_inodes`）持有，防止回收失败的节点在后台线程释放句柄时触发无限自重排循环。
        worker: bool,
    },
    /// 内存文件系统节点句柄，强引用所属 `MemoryFs` 实例以保活底层文件系统。
    Memory(Arc<MemoryFs>),
}

/// 供物理页（Folio）反向追踪所属文件的弱引用标识，不增加 `Inode` 的外部使用计数，也不会阻止节点回收。
pub(crate) struct InodeWeak {
    /// 指向底层 `Inode` 的弱引用。
    inode: Weak<Inode>,
    /// 指向所属缓存域或内存文件系统的弱引用。
    owner: WeakInodeOwner,
}

/// `InodeWeak` 内部记录的后端弱所有权类型。
enum WeakInodeOwner {
    /// 磁盘节点所属缓存域的弱引用。
    Disk(Weak<InodeDomain>),
    /// 内存节点所属内存文件系统的弱引用。
    Memory(Weak<MemoryFs>),
}

impl InodeWeak {
    /// 通过裸指针比较判断该弱引用是否指向给定的 `InodeRef` 及其同一后端实例，不修改任何引用计数。
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

    /// 尝试将页面持有的弱引用升级为有效的 `InodeRef`：内存节点需文件系统仍存活，磁盘节点需缓存域未关闭且槽位处于已加载的 `Cached` 状态。
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

/// 文件失效锁（`invalidate`）读侧守卫，同时维持 `NOFS` 内存分配作用域；字段按声明顺序析构，确保先释放读锁再恢复任务原有的回收上下文。
pub(crate) struct InvalidateGuard<'a> {
    /// `InodeState::invalidate` 的睡眠读锁守卫，排斥并发截断与页缓存整页淘汰。
    _guard: ReadGuard<'a, ()>,
    /// 防止持锁期间内存分配触发文件系统重入的 `NOFS` 作用域守卫。
    _scope: ReclaimScope,
}

/// 文件系统中的唯一运行态索引节点对象；由 `InodeDomain` 或 `MemoryFs` 托管，节点内部不反向强引用缓存域以避免循环引用。
pub struct Inode {
    /// 区分磁盘 `ext4` 与内存文件系统的后端专有状态。
    backend: InodeBackend,
    /// 文件系统内部分配的 64 位 `Inode` 编号（跨卷唯一性需结合 `device()` 或后端实例指针判定）。
    number: u64,
    /// 单次初始化的节点运行态同步容器与元数据结果；未完成初始化前表示节点正处于单飞加载过程中。
    data: Once<Result<InodeState, FsError>>,
    /// 管理该文件驻留物理页（Folio）的页面缓存地址空间映射。
    mapping: FileMapping,
    /// 供等待单飞加载完成的并发任务睡眠的等待队列。
    waiters: WaitQueue,
    /// 测试钩子：用于在截断各关键阶段注入观察回调。
    #[cfg(feature = "ktest")]
    truncate_hook: SpinLock<Option<TruncateHook>>,
    /// 标记结构体不可在内存中移动，确保内嵌的侵入式 `WaitQueue` 地址始终稳定。
    _pinned: PhantomPinned,
}

/// `Inode` 的底层存储后端分支。
enum InodeBackend {
    /// 驻留在块设备 `ext4` 卷上的持久化磁盘节点。
    Disk(DiskInode),
    /// 驻留在内存文件系统（如 `tmpfs` / `devtmpfs`）中的纯内存节点。
    Memory {
        /// 所属 `MemoryFs` 的弱引用（外部通过 `InodeRef` 强引用保活文件系统，避免目录树与文件系统根循环强引用）。
        filesystem: Weak<MemoryFs>,
        /// 内存节点的类型化内容载荷。
        content: MemoryContent,
    },
}

/// 磁盘 `ext4` 节点的生命周期与后台回写追踪状态。
struct DiskInode {
    /// 节点所属的 `Ext4Volume` 强引用。
    filesystem: Arc<Ext4Volume>,
    /// 节点在 `InodeDomain::index.slots` 中固定的槽位下标，首次占槽发布时一次性写入。
    wb_slot: Once<usize>,
    /// 标记该节点是否存在尚未落盘的脏页或正在回写的页面，供后台回写线程快速筛选。
    dirty_obligation: AtomicBool,
    /// 标记当 `links == 0` 淘汰删除时，其磁盘数据块、扩展属性与位图是否已完成释放（防止重复释放）。
    storage_released: AtomicBool,
}

/// 内存文件系统节点的具体内容载荷。
pub(crate) enum MemoryContent {
    /// 普通文件，实际文件字节保存在 `Inode::mapping` 管理的物理页中。
    Regular,
    /// 目录节点，包含由自旋锁保护的内存目录项列表。
    Directory(SpinLock<MemoryDirectory>),
    /// 符号链接节点，直接保存原始目标路径字节序列（不含末尾 NUL）。
    Symlink(Vec<u8>),
    /// 字符设备、块设备、FIFO 或 Socket 等特殊节点描述。
    Special(MemorySpecial),
}

/// 内存目录的权威子项表与游标状态。
pub(crate) struct MemoryDirectory {
    /// 父目录的 `Inode` 编号（根目录指向自身编号），用于解析 `..` 查找与遍历。
    pub(crate) parent: u64,
    /// 下一个新建目录项将分配的单调递增 `cookie`（`0` 和 `1` 固定保留给 `.` 和 `..`）。
    pub(crate) next_cookie: u64,
    /// 当前目录下除 `.` 与 `..` 之外的所有活动子目录项数组。
    pub(crate) entries: Vec<MemoryEntry>,
}

/// 内存目录中的单个子目录项记录。
pub(crate) struct MemoryEntry {
    /// 目录项名称的原始字节切片（不含 `/` 与 `\0`）。
    pub(crate) name: Arc<Vec<u8>>,
    /// 指向子节点的内部强引用，保证目录项存在期间子节点不被销毁。
    pub(crate) inode: Arc<Inode>,
    /// 子节点的文件类型快照，用于 `getdents` 直接填充 `d_type`。
    pub(crate) kind: InodeKind,
    /// 分配给该目录项的唯一单调 `cookie`，保证并发增删目录项时游标续读不会重复或遗漏现存项。
    pub(crate) cookie: u64,
}

/// 测试专用的文件截断阶段枚举，用于在页表卸载、页面清理与磁盘块释放之间观测并发不变量。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum TruncatePhase {
    /// 已更新文件 `size` 并完成第一轮尾部页表映射卸载（`unmap_range`）之后。
    AfterFirstUnmap,
    /// 已完成页面缓存截断（`truncate_pages`）、准备执行第二轮兜底页表卸载之前。
    BeforeSecondUnmap,
    /// 已发布新块树根与元数据记录、准备逐个释放已摘除的旧磁盘块之前。
    BeforeBlockRelease,
    /// 在 `links == 0` 删除回收路径中，已完成磁盘块与 `Inode` 位图释放之后。
    AfterStorageRelease,
}

/// 测试专用的截断阶段观察回调类型。
#[cfg(feature = "ktest")]
type TruncateHook = Arc<dyn Fn(&InodeRef, TruncatePhase) + Send + Sync>;

/// 缓存域生命周期观察凭据：仅强引用 `InodeDomain` 本身（元组字段 `0`）而不占用任何 `Inode` 使用计数，供卸载流程等待整个缓存域彻底退场。
pub(crate) struct InodeCacheWatch(Arc<InodeDomain>);

impl InodeCacheWatch {
    /// 等待当前已排入队列的后台回收任务执行完毕（若仍有外部持有 `InodeRef`，对应节点会继续保留在槽中）。
    #[cfg(feature = "ktest")]
    pub(crate) fn wait(&self) -> Result<(), FsError> {
        self.0.flush()
    }

    /// 在上层 `InodeCache` 已销毁（`cache_open == false`）后，阻塞等待所有派发计数归零、所有槽位清空且所有绑定卷完成停机；若存在回收失败节点则立即返回其错误。
    pub(crate) fn wait_retired(&self) -> Result<(), FsError> {
        sched::assert_switchable();
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self.0` 强引用保活 `InodeDomain`，确保 `self.0.waiters` 队列地址固定有效；
            // 2. `node` 经 `pin!` 固定在当前栈帧，且在任何分支退出或析构 `node`/`waiter` 前都会先释放 `registration`。
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
                // 注销等待队列节点不会自动消耗任务级 `Waiter` 状态；在调用 `flush()` 进入下一轮睡眠等待前必须先析构当前 `waiter`。
                drop(waiter);
                return self.0.flush();
            }
            waiter.park();
            drop(registration);
        }
    }

    /// 检查指定 `Inode` 编号是否仍占用当前缓存域中的某个槽位，不增加引用计数也不改变淘汰状态。
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

/// `Inode` 的定长运行态属性快照（不包含 60 字节的 `ext4` 块映射根数组）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InodeMetadata {
    /// 文件系统内的 64 位 `Inode` 编号。
    pub number: u64,
    /// 节点类型（普通文件、目录、符号链接、字符/块设备、FIFO、Socket 等）。
    pub kind: InodeKind,
    /// POSIX 文件类型与权限位掩码（包含 `S_IFMT`、`S_ISUID`、`S_ISGID`、`S_ISVTX` 及读写执行位）。
    pub mode: u16,
    /// 文件逻辑字节大小（普通文件为有效字节长度，符号链接为目标路径字节数，目录为目录块或记账总字节数）。
    pub size: u64,
    /// 所有者用户 ID。
    pub uid: u32,
    /// 所有者组 ID。
    pub gid: u32,
    /// 硬链接计数（`nlink`）；降为 `0` 且所有打开句柄关闭后将触发存储释放。
    pub links: u32,
    /// 已分配的 512 字节扇区总数（对应 `stat.st_blocks` 单位）。
    pub blocks: u64,
    /// 设备文件对应的主次设备号编码（非设备节点为 `0`）。
    pub rdev: u64,
    /// 最后访问时间戳（`atime`）。
    pub atime: InodeTime,
    /// 最后内容修改时间戳（`mtime`）。
    pub mtime: InodeTime,
    /// 最后状态/属性变更时间戳（`ctime`）。
    pub ctime: InodeTime,
    /// 磁盘 `Inode` 代数（`i_generation`），用于区分复用同一编号的新旧节点。
    pub generation: u32,
    /// 底层文件系统标志位（如 `EXTENTS`、`INDEX`、`HUGE_FILE`、`EA_INODE` 等）。
    pub flags: u32,
    /// `ext4` 元数据 CRC32C 校验和种子（由文件系统 UUID 与节点号/代数派生）。
    pub checksum_seed: u32,
    /// 标记是否为目标路径直接内嵌在 60 字节 `i_block` 数组中的快速符号链接（Fast Symlink）。
    pub fast_symlink: bool,
    /// 外部扩展属性块（`i_file_acl`）的物理块号，无外部块时为 `0`。
    pub file_acl: u64,
    /// 大 `Inode` 记录中超出传统 128 字节部分已占用的扩展字段字节数（`i_extra_isize`）。
    pub extra_isize: usize,
}

/// `Inode` 完成加载后安装的内部同步原语与运行状态容器。
struct InodeState {
    /// 串行化文件内容写入、截断、属性修改以及目录读写的顶层睡眠读写锁。
    write: SleepRwLock<()>,
    /// 协调截断/回收淘汰（写锁）与并发页面读取/缺页填充/`mkwrite` 终检（读锁）的睡眠读写锁。
    invalidate: SleepRwLock<()>,
    /// 保护运行态 `InodeMetadata` 快照的短期自旋锁。
    metadata: SpinLock<InodeMetadata>,
    /// 与后端存储耦合的块树根锁状态。
    storage: InodeStorage,
    /// 写打开与可执行映像互斥计数器（`> 0` 为写句柄数，`< 0` 为活跃执行映像数的相反数）。
    writecount: SpinLock<isize>,
}

/// `InodeState` 中按后端区分的块映射根同步存储。
enum InodeStorage {
    /// 磁盘 `ext4` 节点：使用睡眠读写锁 `root` 保护 60 字节的 `i_block` 根区域（Extent 树头或直接/间接块指针数组）。
    Disk { root: SleepRwLock<[u8; 60]> },
    /// 内存文件系统节点：无磁盘块映射树。
    Memory,
}

/// 单节点独占写锁守卫，用于串行化普通属性变更与硬链接计数调整，并维持 `NOFS` 内存回收作用域。
pub(crate) struct InodeWriteGuard<'a> {
    /// 被加锁的目标 `InodeRef`。
    inode: &'a InodeRef,
    /// 已完成加载的节点内部状态引用。
    state: &'a InodeState,
    /// `InodeState::write` 的独占写锁守卫。
    _lock: WriteGuard<'a, ()>,
    /// 防止持锁分配内存重入文件系统的 `NOFS` 作用域守卫。
    _scope: ReclaimScope,
}

/// 目录共享读锁守卫，保护目录项查找与 `getdents` 遍历免受并发目录编辑干扰。
pub(crate) struct DirectoryReadGuard<'a> {
    /// 被读取的目录 `InodeRef`。
    inode: &'a InodeRef,
    /// 已完成加载的目录节点内部状态引用。
    state: &'a InodeState,
    /// `InodeState::write` 的共享读锁守卫。
    _lock: ReadGuard<'a, ()>,
    /// 防止持锁分配内存重入文件系统的 `NOFS` 作用域守卫。
    _scope: ReclaimScope,
}

/// 目录独占写锁守卫，复用 `InodeState::write` 写侧串行化目录项增删改与目录自身元数据更新。
pub(crate) struct DirectoryWriteGuard<'a> {
    /// 被修改的目录 `InodeRef`。
    inode: &'a InodeRef,
    /// 已完成加载的目录节点内部状态引用。
    state: &'a InodeState,
    /// `InodeState::write` 的独占写锁守卫。
    _lock: WriteGuard<'a, ()>,
    /// 防止持锁分配内存重入文件系统的 `NOFS` 作用域守卫。
    _scope: ReclaimScope,
}

/// 已完成参数校验与磁盘 `inode-table` 缓冲区预留的节点属性变更凭据。
pub(crate) struct PreparedInodeMetadata {
    /// 目标节点的强引用句柄，发布时用于核对身份一致性。
    inode: InodeRef,
    /// 底层磁盘记录缓冲区定位或内存后端标记。
    storage: PreparedMetadataStorage,
    /// 准备阶段读取并校验通过的原属性快照。
    old: InodeMetadata,
    /// 待提交发布的新属性快照。
    next: InodeMetadata,
}

/// `PreparedInodeMetadata` 持有的底层存储写入目标。
enum PreparedMetadataStorage {
    /// 磁盘后端：锁定在元数据缓存中的 `inode-table` 物理块缓冲区 `record` 及该记录在块内的起始字节偏移 `offset`。
    Disk { record: BufferRef, offset: usize },
    /// 内存后端：仅需更新内存中的 `InodeMetadata` 快照。
    Memory,
}

/// 已完成目录数据块编辑与父目录 `inode-table` 映射预留的磁盘目录修改凭据；若未发布而直接析构会自动回滚新分配的目录块。
pub(crate) struct PreparedDiskDirectoryMutation {
    /// 被修改的父目录 `InodeRef`。
    parent: InodeRef,
    /// 父目录所属的 `Ext4Volume`。
    volume: Arc<Ext4Volume>,
    /// 底层 `ext4` 目录块增删改准备凭据，发布或取消时取出消费。
    edit: Option<PreparedDirectoryEdit>,
    /// 父目录自身 `inode-table` 记录与块映射更新凭据（当 `changed == false` 时为 `None`）。
    inode_mapping: Option<PreparedInodeMapping>,
    /// 编辑完成后父目录新的 60 字节块映射根内容。
    root: [u8; 60],
    /// 编辑完成后父目录新的运行态元数据快照。
    next: InodeMetadata,
    /// 标记本次操作是否实际改变了目录内容（如同名原位移动时为 `false`）。
    changed: bool,
}

/// 统一封装磁盘或内存文件系统的目录修改两阶段准备凭据。
pub(crate) enum PreparedDirectoryMutation {
    /// 磁盘 `ext4` 目录修改凭据。
    Disk(PreparedDiskDirectoryMutation),
    /// 内存目录修改凭据，包含预先克隆并完成编辑的新目录快照。
    Memory {
        /// 被修改的父目录 `InodeRef`。
        parent: InodeRef,
        /// 若目录内容发生实际变化，保存已完成增删改的候选 `MemoryDirectory` 副本。
        directory: Option<MemoryDirectory>,
        /// 编辑完成后父目录的新元数据快照。
        next: InodeMetadata,
    },
}

/// 创建新节点时的初始内容类型描述。
#[derive(Clone, Copy)]
enum NewInodeContent<'a> {
    /// 空普通文件。
    Regular,
    /// 包含初始 `.` 与 `..` 目录项的新子目录。
    Directory,
    /// 指向给定目标路径字节序列的符号链接。
    Symlink(&'a [u8]),
}

/// 磁盘新 `Inode` 在正式挂载到父目录名下之前的两阶段准备凭据：
/// 持有位图分配、已格式化为 `links = 0` 的权威 `InodeRef` 以及待发布的初始硬链接更新。
pub(crate) struct PreparedDiskNewInode {
    /// 新节点所属的 `Ext4Volume`。
    volume: Arc<Ext4Volume>,
    /// 未正式发布的磁盘 `Inode` 位图预留凭据。
    allocation: Option<UnpublishedInode>,
    /// 已加载进入 `InodeCache` 的新节点句柄（此时磁盘与内存中 `links == 0`）。
    inode: Option<InodeRef>,
    /// 分配后从磁盘读出的原始 `InodeData` 快照，用于在加载进缓存前失败时恢复原始记录。
    initial: Option<InodeData>,
    /// 标记是否已向磁盘 `inode-table` 写入新节点的初始格式化记录。
    record_initialized: bool,
    /// 若新节点为目录，保存其初始 `.` / `..` 目录块的准备凭据。
    directory: Option<PreparedDirectoryEdit>,
    /// 若新节点为超出 60 字节的长符号链接，保存其目标数据块与映射的准备凭据。
    symlink: Option<PreparedSymlink>,
    /// 将 `links` 从 `0` 提升至初始链接数（普通文件/符号链接为 `1`，目录为 `2`）的元数据更新凭据。
    link_update: Option<PreparedInodeMetadata>,
}

/// 统一封装磁盘或内存文件系统的新建 `Inode` 两阶段准备凭据。
pub(crate) enum PreparedNewInode {
    /// 磁盘 `ext4` 新节点准备凭据。
    Disk(PreparedDiskNewInode),
    /// 内存文件系统新节点准备凭据。
    Memory {
        /// 已在 `MemoryFs` 中分配（初始 `links == 0`）的节点句柄。
        inode: Option<InodeRef>,
        /// 正式发布时应写入的包含初始正数 `links` 的元数据快照。
        next: InodeMetadata,
    },
}

/// 超出 60 字节内嵌上限（Fast Symlink）的磁盘长符号链接准备凭据。
struct PreparedSymlink {
    /// 符号链接所属的 `Ext4Volume`。
    volume: Arc<Ext4Volume>,
    /// 为符号链接数据块预留的 Extent / 间接块映射计划。
    mappings: Option<PreparedDirectoryMappings>,
    /// 目标路径各逻辑块对应的元数据缓存物理块缓冲区列表。
    buffers: Vec<BufferRef>,
    /// 预先拷贝到内核堆上的符号链接目标路径字节序列。
    target: Vec<u8>,
    /// 卷的块字节大小。
    block_size: usize,
    /// 预留映射生成的新 60 字节块映射根数据。
    root: [u8; 60],
    /// 计入符号链接数据块与索引块后的 512 字节扇区总数。
    blocks: u64,
    /// 标记是否已进入不可逆的 `publish` 阶段；若为 `false` 则析构时自动作废缓存窗口并归还物理块。
    publishing: bool,
}

impl PreparedSymlink {
    /// 校验长符号链接目标路径合法性，分配连续逻辑块映射并获取对应的元数据块缓冲区（目标字节延迟到 `publish` 时才真正写入）。
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

    /// 将目标路径字节写入预先获取的数据块缓冲区（尾部补零），随后正式提交块映射分配。
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
    /// 若长符号链接在正式发布前因上层失败而中止，等待相关块回写结束后作废其元数据缓存别名，并归还未发布的物理块预留。
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
                // SAFETY:
                // 1. `self.publishing` 为 `false`，这批物理块仍由当前未发布的 `mappings` 预留独占，尚未挂载到任何已公开的 `Inode` 块树上；
                // 2. 前置 `buffer.wait_writeback()` 已确保该缓冲区无在途块设备回写，且 `metadata_block` 获取完成后无在途读取，可安全直接丢弃缓存页。
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
    /// 借用已格式化为 `links = 0` 并载入缓存域的新 `InodeRef`。
    pub(crate) fn inode(&self) -> &InodeRef {
        self.inode
            .as_ref()
            .expect("prepared inode canonical object absent")
    }

    /// 依次发布底层目录/符号链接初始数据块、将节点 `links` 更新为正数，并确认消费 `Inode` 位图分配，返回最终就绪的 `InodeRef`。
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

    /// 先提交新目录或长符号链接的初始数据块与映射，确保在节点提升链接数与挂载目录名之前底层内容已经就绪。
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
    /// 若新节点在正式发布前中止：
    /// - 已载入缓存域（`self.inode.is_some()`）：提交底层块映射与位图所有权，随后释放 `InodeRef` 交由常规 `links == 0` 异步回收流程清理磁盘存储；
    /// - 尚未载入缓存域：直接还原 `inode-table` 初始记录、撤销未发布的目录/符号链接块预留并归还位图。
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
    /// 从磁盘解码出的 `InodeData` 提取定长运行态属性快照（60 字节块映射根单独交由 `InodeStorage::Disk` 的睡眠读写锁管理）。
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

    /// 将当前属性快照与给定的 60 字节块映射根临时拼装为 `InodeData`，供底层 `ext4` 编解码与映射查询使用。
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
    /// 使用磁盘读取成功的 `InodeData` 初始化节点的运行态锁与元数据容器。
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

    /// 为内存文件系统节点初始化运行态锁与元数据容器。
    fn memory(metadata: InodeMetadata) -> Self {
        Self {
            write: SleepRwLock::new(()),
            invalidate: SleepRwLock::new(()),
            metadata: SpinLock::new(metadata),
            storage: InodeStorage::Memory,
            writecount: SpinLock::new(0),
        }
    }

    /// 获取磁盘节点的 60 字节块映射根读写锁引用；若在内存节点上调用则触发 panic。
    fn disk_root(&self) -> &SleepRwLock<[u8; 60]> {
        match &self.storage {
            InodeStorage::Disk { root } => root,
            InodeStorage::Memory => panic!("memory inode has no disk root"),
        }
    }
}

/// `mmap` 共享可写缺页（`mkwrite`）在无页表锁阶段预先完成的物理页与块分配凭据。
pub(crate) struct PreparedMkwrite {
    /// 已填充就绪并完成底层磁盘块预留的目标物理页（Folio）。
    folio: FolioRef,
    /// 该页在文件中的页索引（`offset / PAGE_SIZE`）。
    index: u64,
    /// 准备阶段观测到的文件字节大小快照，用于在页表上锁后核对文件未被并发截断或扩缩。
    size: u64,
    /// 磁盘文件页内已完成块分配准备的扇区位掩码（内存文件恒为 `0`）。
    prepared_sectors: u8,
}

impl PreparedMkwrite {
    /// 借用准备好的目标物理页引用（最终映射可写 PTE 前仍需通过 `finish_mkwrite` 复核该页未脱离当前文件）。
    pub(crate) fn folio(&self) -> &FolioRef {
        &self.folio
    }
}

/// 文件截断或跨 EOF 扩容写之前预先准备的尾页补零计划。
struct PreparedTail {
    /// 需要执行部分区间清零的尾部物理页。
    folio: FolioRef,
    /// 尾页在文件中的页索引。
    index: u64,
    /// 页内清零起始字节偏移（含）。
    start: usize,
    /// 页内清零结束字节偏移（不含）。
    end: usize,
    /// 页内各文件系统块是否已映射实际磁盘物理块的位掩码（仅已映射块在清零后需标记为脏页回写，稀疏空洞块无需触发回写）。
    mapped: u8,
}

/// 文件写打开访问凭据：每个以可写模式创建的打开文件描述（OFD）持有一份（`dup` 共享同一 OFD 时不重复计数），用于与 `execve` 互斥。
pub struct WriteAccess {
    /// 绑定写权限的目标 `InodeRef`。
    inode: InodeRef,
}

/// 可执行映像占用凭据：每个映射了该可执行文件的活跃进程地址空间独立持有一份，存在期间禁止任何写打开或截断。
pub struct ExecAccess {
    /// 被作为可执行映像映射的目标 `InodeRef`。
    inode: InodeRef,
}

impl WriteAccess {
    /// 凭当前写访问令牌向绑定节点执行定位写或追加写，返回实际写入起始偏移与已写入字节数。
    pub fn write_from(
        &self,
        offset: u64,
        append: bool,
        source: &mut WriteBuffer<'_>,
    ) -> Result<(u64, usize), FsError> {
        self.inode.write_backend(offset, append, source)
    }

    /// 凭当前写访问令牌将绑定节点截断或扩展至 `new_size` 字节（不修改调用方 OFD 的文件偏移量）。
    pub fn truncate(&self, new_size: u64) -> Result<(), FsError> {
        self.inode.truncate_backend(new_size)
    }
}

impl Drop for WriteAccess {
    /// 关闭可写打开文件描述时递减 `writecount` 正计数。
    fn drop(&mut self) {
        let mut count = self.inode.loaded_state().writecount.lock();
        assert!(0 < *count, "write access count lost");
        *count -= 1;
    }
}

impl Clone for ExecAccess {
    /// `fork` 复制进程地址空间时克隆可执行占用凭据，在自旋锁内将负数 `writecount` 继续减一。
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
    /// 进程退出、`execve` 替换映像或卸载可执行映射时递增负数 `writecount`，归还本次拒绝写入占用。
    fn drop(&mut self) {
        let mut count = self.inode.loaded_state().writecount.lock();
        assert!(*count < 0, "exec access count lost");
        *count += 1;
    }
}

/// 单飞加载守卫：仅由赢得槽位发布权的线程激活；若加载中途因取消或 panic 提前退出，析构函数会自动发布中止错误并清理占位槽。
struct LoadGuard<'a> {
    /// 目标节点所属的缓存域。
    domain: &'a Arc<InodeDomain>,
    /// 正在执行磁盘加载的候选节点。
    inode: &'a InodeRef,
    /// 标记当前守卫是否已激活接管槽位加载责任；调用 `complete` 正常发布后置为 `false`。
    active: bool,
}

impl InodeCache {
    /// 创建拥有独立固定槽位容量预算与后台异步回收队列的 `InodeCache`（容量为 `0` 时返回 `InvalidOperation`）。
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

    /// 按卷实例与磁盘 `Inode` 编号查找或加载权威 `InodeRef`。
    pub fn iget(&self, filesystem: &Arc<Ext4Volume>, number: u64) -> Result<InodeRef, FsError> {
        self.domain.iget(filesystem, number, Some(self), None)
    }

    /// 获取或初始化与本缓存域关联的共享 `DentryCache` 及指定卷的根 `Dentry`；若已有存活的 `DentryCache` 则要求请求容量必须一致。
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

    /// 获取指定 `Ext4Volume` 的根 `Dentry`：优先升级根槽位中已登记的存活弱引用，缺失时在锁外构造新候选并原子发布。
    pub(crate) fn root_dentry(&self, volume: &Arc<Ext4Volume>) -> Result<DentryRef, FsError> {
        let inode = self.iget(volume, codec::ROOT_INODE)?;
        if let Some(root) = self.domain.canonical_root(&inode) {
            return Ok(root);
        }
        let candidate = DentryRef::try_root(inode.clone())?;
        Ok(self.domain.publish_root(&inode, candidate))
    }

    /// 两阶段准备创建一个尚未挂载目录项的普通文件 `Inode`（自动分派至内存或磁盘后端）。
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

    /// 两阶段准备创建一个包含初始 `.` 与 `..` 记录但尚未挂载到父目录的新子目录 `Inode`。
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

    /// 两阶段准备创建一个指向 `target` 原始字节路径的新符号链接 `Inode`（符号链接权限固定为 `0777`，不受进程 `umask` 影响）。
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

    /// 磁盘新节点的统一准备流程：分配位图、准备初始目录块或符号链接块、写入 `links = 0` 的初始磁盘记录并载入 `InodeCache`。
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

        // Step 1: 在父目录所在块组附近分配未公开的磁盘 Inode 位图号，并读取其原始槽位记录。
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
        // Step 2: 按节点类型准备初始内容（目录分配 `.`/`..` 块，短符号链接内嵌至 60 字节块根，长符号链接分配数据块）。
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

        // Step 3: 计算初始时间戳与标志位，将 `links = 0` 的完整记录写入磁盘 inode-table。
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

        // Step 4: 通过 `iget` 将新节点载入权威缓存，并预先准备好将 `links` 提升为正数的元数据变更凭据。
        prepared.inode = Some(self.iget(volume, number)?);
        let update = {
            let guard = prepared.inode().inode_write()?;
            guard.prepare_link_delta(link_count, now_ns)?
        };
        prepared.link_update = Some(update);
        Ok(prepared)
    }

    /// 触发淘汰当前无外部句柄引用且无共享内存映射的缓存节点，阻塞等待本轮回收完成后返回实际释放的槽位数。
    pub fn prune_unused(&self) -> usize {
        self.domain.prune_unused()
    }

    /// 在卷显式卸载且关联节点清空后，通知底层卷解绑当前缓存域并同步停止其回写服务。
    pub(crate) fn retire_volume(&self, volume: &Arc<Ext4Volume>) {
        volume.retire_inode_domain(&self.domain);
    }

    /// 返回当前缓存域中已被占用的槽位总数（包含正在加载、正常缓存及待回收节点）。
    #[cfg(feature = "ktest")]
    pub fn len(&self) -> usize {
        self.domain.index.lock().slots.iter().flatten().count()
    }

    /// 判断当前缓存域是否没有任何已占用的节点槽位。
    #[cfg(feature = "ktest")]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 判断当前缓存域的固定槽位预算是否已满。
    pub(crate) fn is_full(&self) -> bool {
        self.domain.is_full()
    }

    /// 设置或清除测试专用的单飞加载停点回调，旧回调移出锁外释放。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_load_hook(&self, hook: Option<LoadHook>) {
        let old = core::mem::replace(&mut *self.load_hook.lock(), hook);
        drop(old);
    }

    /// 返回自缓存创建以来实际向底层卷发起磁盘 `Inode` 读取的累计次数。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_load_count(&self) -> usize {
        self.loads.load(Ordering::Relaxed)
    }

    /// 返回当前正在指定节点的单飞加载等待队列上睡眠的任务数量。
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

    /// 获取当前缓存域的关闭观察凭据，供卸载流程等待所有后台回收与卷停机彻底结束。
    pub(crate) fn watch(&self) -> InodeCacheWatch {
        InodeCacheWatch(self.domain.clone())
    }
}

impl Drop for InodeCache {
    /// 销毁 `InodeCache` 时将域标记为已关闭，把所有可淘汰槽位标记为 `Requested`，并在存在绑定卷时唤醒后台工作队列执行清理与卷停机。
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
    /// 克隆外部 `InodeRef` 句柄：磁盘节点在索引自旋锁内递增槽位的 `users` 活跃计数，且克隆出的句柄始终标记为非后台工作线程身份。
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
    /// 释放外部 `InodeRef` 句柄：磁盘节点在索引锁内递减 `users`；当最后一个引用释放且满足淘汰或删除条件（如 `links == 0` 或缓存已关闭）时，自动将槽位排入后台工作队列异步回收，绝不在 `Drop`（含中断或自旋锁上下文）中阻塞等待磁盘 I/O。
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
    /// 将强引用降级为供物理页（Folio）持有的 `InodeWeak`，使页面缓存可以在不延长节点生命周期的前提下反向定位所属文件。
    pub(crate) fn downgrade_for_reclaim(&self) -> InodeWeak {
        InodeWeak {
            inode: Arc::downgrade(&self.0),
            owner: match &self.1 {
                InodeOwner::Disk { domain, .. } => WeakInodeOwner::Disk(Arc::downgrade(domain)),
                InodeOwner::Memory(fs) => WeakInodeOwner::Memory(Arc::downgrade(fs)),
            },
        }
    }

    /// 为常规内存回收路径获取文件 `invalidate` 独占写锁（调用方不得处于 `NOFS`/`NOIO` 作用域）。
    pub(crate) fn lock_reclaim_invalidate(&self) -> Result<WriteGuard<'_, ()>, FsError> {
        Ok(self.wait_loaded()?.invalidate.write())
    }

    /// 在索引锁外分配尚未初始化的磁盘 `Inode` 实例及其页面映射容器；若内存不足直接失败且不会污染缓存索引。
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

    /// 判断两个 `InodeRef` 是否指向内存中同一个运行态 `Inode` 实例。
    pub fn ptr_eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    /// 等待节点完成加载，进入 `NOFS` 作用域并获取节点独占写锁守卫 `InodeWriteGuard`。
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

    /// 等待节点完成加载，校验其为目录类型后获取目录共享读锁守卫 `DirectoryReadGuard`（同时进入 `NOFS` 作用域）。
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

    /// 等待节点完成加载，校验其为目录类型后获取目录独占写锁守卫 `DirectoryWriteGuard`（复用 `InodeState::write` 写锁，允许持锁直接重查子项而无需递归加读锁）。
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

    /// 在目录共享读锁保护下按名称查找子项并返回其权威 `InodeRef`；若名称不存在则返回 `NotFound`。
    #[cfg(feature = "ktest")]
    pub fn lookup(&self, cache: &InodeCache, name: &[u8]) -> Result<InodeRef, FsError> {
        self.directory_read()?
            .lookup(cache, name)?
            .ok_or(FsError::NotFound)
    }

    /// 在目录共享读锁保护下将用户态传入的偏移 `cookie` 解析为底层目录遍历游标（普通线性偏移或 HTree 哈希碰撞续读状态）。
    pub(crate) fn directory_cursor(&self, cookie: u64) -> Result<DirectoryCursor, FsError> {
        self.directory_read()?.cursor_at(cookie)
    }

    /// 返回目录末尾的 `SEEK_END` 游标 `cookie` 值。
    pub(crate) fn directory_end_cookie(&self) -> Result<u64, FsError> {
        self.directory_read()?.end_cookie()
    }

    /// 在目录共享读锁保护下读取游标位置的下一个有效目录项。
    pub fn dir_next(&self, cursor: &DirectoryCursor) -> Result<Option<DirectoryEntry>, FsError> {
        self.directory_read()?.dir_next(cursor)
    }

    /// 标记当前磁盘节点存在未落盘脏数据，并唤醒所属卷的后台回写服务（必须在页面实际标脏前调用，防止后台扫描漏掉刚变脏的节点）。
    pub(crate) fn note_dirty(&self) {
        let disk = self.disk();
        disk.dirty_obligation.store(true, Ordering::Release);
        disk.filesystem
            .writeback_domain()
            .notify_dirty(&disk.filesystem);
    }

    /// 在数据回写释放回写锁后，按 `invalidate` 写锁 -> `mapping` 回写锁的顺序排斥新写操作并重新核定节点脏标记。
    pub(crate) fn refresh_dirty_obligation(&self) -> Result<(), FsError> {
        let _invalidate = self.wait_loaded()?.invalidate.write();
        let _writeback = self.mapping.lock_writeback();
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// 在调用方已持有 `invalidate` 独占锁与 `mapping` 回写锁的前提下，根据页面缓存中是否仍存在脏页或在途回写页刷新节点的 `dirty_obligation` 标记。
    fn refresh_dirty_obligation_locked(&self) {
        self.disk().dirty_obligation.store(false, Ordering::Release);
        if self.mapping.has_dirty_or_writeback() {
            self.note_dirty();
        }
    }

    /// 申请文件写打开访问令牌 [`WriteAccess`]：
    /// - 拒绝直接写入承载扩展属性引用计数的内部隐藏 `EA_INODE`；
    /// - 若节点当前正作为可执行映像运行（`writecount < 0`），返回 `TextBusy`（`ETXTBSY`）。
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

    /// 在加载可执行映像前申请拒绝写入令牌 [`ExecAccess`]：若当前存在任何可写打开句柄（`writecount > 0`），返回 `TextBusy`（`ETXTBSY`）。
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

    /// 校验卷可写权限、普通文件类型与最大文件尺寸上限，在获取 `write` 与 `invalidate` 双重独占锁后执行磁盘文件截断或扩容。
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

    /// 在调用方已持有 `write` 与 `invalidate` 独占写锁的前提下，执行磁盘文件截断、扩容或 `links == 0` 删除释放：
    /// 先更新文件 `size` 并卸载超出范围的用户态页表映射，再等待在途回写并截断页面缓存，最后摘除磁盘块树、释放物理块及（删除模式下的）`Inode` 位图。
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

        // Step 1: 预先准备非块对齐或扩容场景下的尾页补零计划，并检查块边界部分尾页是否存在历史 I/O 错误。
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
        // 若新尺寸恰好落在磁盘块边界但未满一页，无需读取未缓存的冷页；但若该页已在缓存中，必须在修改尺寸前确认其无挂起错误。
        if new_size <= old.size
            && new_size % g.block_size as u64 == 0
            && new_size % PAGE_SIZE as u64 != 0
        {
            if let Some(folio) = self.mapping.cached_folio(new_size / PAGE_SIZE as u64) {
                folio.lock_content().range_ready(0, PAGE_SIZE)?;
            }
        }

        // Step 2: 获取元数据回写锁与块树根写锁，校验磁盘记录一致性并准备块树截断方案及（删除时）Inode/EA 释放方案。
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

        // Step 3: 先在磁盘记录与内存快照中发布新文件 `size`（保留旧块根），并执行第一轮页表卸载以阻止用户态继续通过旧 PTE 写入尾部。
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

        // Step 4: 此时 `root` 锁已释放（允许在途数据回写获取 `root` 读锁完成块映射查询）；
        // 获取 `mapping` 回写锁，应用尾页补零，清理超出 `new_size` 的缓存页，并执行第二轮兜底页表卸载。
        let _writeback = self.mapping.lock_writeback();
        if let Some(tail) = &tail {
            tail.folio.wait_writeback();
            self.apply_tail(tail);
        }
        self.note_dirty();
        // SAFETY:
        // 1. 当前任务同时持有 `state.write` 独占锁、`state.invalidate` 独占锁与 `self.mapping` 回写互斥锁，排斥一切并发读写、新缺页填充与新回写提交；
        // 2. 新 `size` 已提前发布且第一轮 `unmap_range` 已撤销尾部页表项与跨核 TLB；`root` 锁未持有，不会与在途页面 I/O 死锁；
        // 3. 即使 `truncate_pages` 返回错误，紧随其后的第二轮 `unmap_range(..., true)` 仍会在传播错误前兜底清理由并发 COW 残留的映射。
        let pages = unsafe { self.mapping.truncate_pages(self, new_size, g.block_size) };
        #[cfg(feature = "ktest")]
        self.truncate_phase(TruncatePhase::BeforeSecondUnmap);
        self.mapping.unmap_range(first..u64::MAX, true);
        pages?;

        // Step 5: 重新获取 `root` 写锁，提交裁剪后的新块树根与块计数，逐个作废并释放已从树上摘除的磁盘物理块。
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
            // SAFETY:
            // 1. 两轮 `unmap_range`、`truncate_pages` 与页面回写等待均已完成，没有任何页表或页面缓存仍引用被截断范围；
            // 2. 当前持有 `root` 写锁与元数据回写锁，新块树已正式发布，旧树节点已不可达且无并发元数据回写；
            // 3. `release_detached_block` 内部会先等待对应元数据缓冲区在途回写完成并作废缓存别名，最后才归还物理块位图。
            unsafe { volume.release_detached_block(block) };
        }
        drop(root);
        drop(tail);

        // Step 6: 若处于 `links == 0` 最终删除路径，释放扩展属性引用与磁盘 Inode 位图，并刷新脏节点标记。
        if let Some(inode_release) = inode_release {
            // SAFETY:
            // 1. `inode_release` 仅在 `deleting == true`（即最后一个 `InodeRef` 已释放、槽位处于 `Eviction::Running` 且无任何外部使用者）时构造；
            // 2. `root` 写锁与所有数据页/元数据块释放均已完成，且当前仍持有元数据回写锁阻止并发提交；
            // 3. 关联的隐藏 EA 节点与 `inode-table` 记录由 `inode_release` 凭据保活，此时可安全清零磁盘记录并归还 `Inode` 位图。
            unsafe { volume.release_inode(inode_release) };
            self.disk().storage_released.store(true, Ordering::Release);
        }
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// 专用于删除回收快速符号链接（Fast Symlink）的截断路径：将其 `size` 清零并释放关联扩展属性与 `Inode` 位图，不将内嵌在 60 字节 `i_block` 中的路径字节当作块树解析。
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
        // SAFETY:
        // 1. 调用方持有 `state.write`、`state.invalidate` 独占写锁与 `mapping` 回写锁，且已完成第一轮全范围页表卸载；
        // 2. 快速符号链接从不向 `FileMapping` 插入数据页，此处调用 `truncate_pages` 仅用于排空并确认映射为空，紧接着执行第二轮页表卸载。
        let pages = unsafe { self.mapping.truncate_pages(self, 0, g.block_size) };
        self.mapping.unmap_range(0..u64::MAX, true);
        pages?;
        if !self.mapping.is_empty() {
            return Err(FsError::Busy);
        }

        // SAFETY:
        // 1. 当前节点 `links == 0` 且槽位处于独占的 `Eviction::Running` 回收状态，无任何外部句柄或残留缓存页（上方 `is_empty()` 已校验）；
        // 2. 内嵌符号链接路径未作为磁盘块分配，`release` 凭据在元数据回写锁保护下保活了 `inode-table` 与 EA 记录，可安全归还 `Inode` 位图。
        unsafe { volume.release_inode(release) };
        self.disk().storage_released.store(true, Ordering::Release);
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// 后台工作队列调用的节点回收主入口：
    /// - 对可写卷上 `links == 0` 的已删除节点，截断释放全部数据块、扩展属性与 `Inode` 位图并同步元数据；
    /// - 对普通淘汰节点，将脏数据页与脏元数据同步落盘后清空未使用的干净缓存页。
    fn evict(&self) -> Result<(), FsError> {
        sched::assert_switchable();
        let volume = self.disk_volume()?;
        // Step 1: 在获取节点业务锁之前预先借用卷的回写工作缓冲区（scratch storage），避免锁顺序反转。
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

        // Step 2: 区分删除回收（`links == 0`）与普通淘汰，分别执行磁盘块释放或文件脏页同步回写，随后同步卷元数据。
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

        // Step 3: 普通淘汰路径在 `invalidate` 独占锁与回写锁保护下清空干净缓存页，若仍有活跃页则拒绝淘汰（返回 `Busy`）。
        let _invalidate = state.invalidate.write();
        let _writeback = self.mapping.lock_writeback();
        self.mapping.prune_unused();
        if !self.mapping.is_empty() {
            return Err(FsError::Busy);
        }
        self.refresh_dirty_obligation_locked();
        Ok(())
    }

    /// 在父节点释放外部扩展属性（EA）时，递减共享隐藏 `EA_INODE` 的引用计数（保存在其磁盘记录特定字段中；当计数归零时清零 `links` 以便后续触发后台块释放）。
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

    /// 触发测试专用的截断阶段观察回调（在自旋锁外调用闭包，保持与生产环境一致的睡眠锁持有状态）。
    #[cfg(feature = "ktest")]
    fn truncate_phase(&self, phase: TruncatePhase) {
        let hook = self.truncate_hook.lock().clone();
        if let Some(hook) = hook {
            hook(self, phase);
        }
    }

    /// 设置或清除测试专用的截断阶段观察回调，旧回调移出锁外析构。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_truncate_hook(&self, hook: Option<TruncateHook>) {
        let previous = core::mem::replace(&mut *self.truncate_hook.lock(), hook);
        drop(previous);
    }

    /// 为磁盘或内存文件的 `mmap` 共享可写缺页（`mkwrite`）预先准备物理页并分配底层文件块（必须在未持有地址空间/页表锁时调用）。
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
        // 循环尝试：先在 `invalidate` 锁外分配页面缓存槽位候选，再持 `write` 写锁与 `invalidate` 读锁填充页面并为页内覆盖文件有效范围的每个逻辑块分配物理块。
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

    /// 在持有 `state.write` 写锁与 `state.invalidate` 读锁的前提下，为 `mkwrite` 目标页内单个逻辑偏移所在的块分配（或复用）磁盘物理块并更新块树映射。
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
            // SAFETY:
            // 1. 当前持有 `root` 写锁，已排空所有并发块映射读者，且新块树根已发布、旧 Extent/间接块已从树上摘除；
            // 2. `release_detached_block` 会等待该旧元数据块在途回写结束并作废缓存别名后，才归还物理块位图。
            unsafe { volume.release_detached_block(block) };
        }
        Ok(())
    }

    /// 进入 `NOFS` 作用域并获取文件 `invalidate` 共享读锁守卫（在缺页终检路径中须按 `mmap` 锁 -> `invalidate` 读锁 -> 页内容锁的顺序获取）。
    pub(crate) fn lock_invalidate(&self) -> Result<InvalidateGuard<'_>, FsError> {
        let scope = ReclaimScope::nofs();
        let guard = self.wait_loaded()?.invalidate.read();
        Ok(InvalidateGuard {
            _guard: guard,
            _scope: scope,
        })
    }

    /// 在持有地址空间锁、`invalidate` 读锁与目标页内容锁的前提下，复核 `prepare_mkwrite` 阶段记录的文件大小、物理页身份及块准备位掩码是否仍然有效：
    /// - 若期间发生截断、页面淘汰替换或块准备状态变化，返回 `Ok(false)` 通知调用方重试；
    /// - 校验通过则将页内有效范围标记为脏页并返回 `Ok(true)`，允许安装可写 PTE。
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

    /// 磁盘普通文件的缓冲写入主循环（仅通过绑定写权限的 [`WriteAccess`] 调用）：
    /// 每次循环先在文件锁外从用户缓冲区拉取最多一个块的数据（避免缺页重入死锁），再在 `invalidate` 写锁保护下补零跨 EOF 空洞、预留磁盘块、写入页面缓存并原子发布新块树与文件尺寸。
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
            // Step 1: 计算本次块内可写字节数，并在获取 `invalidate` 锁之前从源缓冲区拉取数据（允许安全处理用户态源地址缺页）。
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
            // Step 2: 进入 `invalidate` 写锁保护区；若写入起点跨过原 EOF，先将原尾页中暴露出的空洞区间清零。
            let end = position + pulled as u64;
            let _invalidate = state.invalidate.write();
            if old.size < end {
                if let Err(error) = self.zero_eof_tail(&old, position, &mut attempt) {
                    return write_result(start, accepted, error);
                }
            }

            // Step 3: 校验磁盘 Inode 记录一致性并为当前写入块预留物理块映射与新元数据快照。
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

            // Step 4: 在新块映射尚未发布前（允许 `read_page` 内部按旧块树读取已有数据），获取并填充目标缓存页。
            let folio = match self.read_page(position / PAGE_SIZE as u64, &mut attempt, None) {
                Ok(folio) => folio,
                Err(error) => {
                    prepared.cancel(volume);
                    return write_result(start, accepted, error);
                }
            };

            // Step 5: 持 `root` 写锁与页内容锁，完成新分配块清零、写入用户数据、标脏页面，并原子提交新块树根与 `inode-table` 记录。
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
                // SAFETY:
                // 1. 当前持有 `root` 写锁，旧块树读者已全部排空，且合并/替换后淘汰的旧索引块已从已发布的新树中彻底移除；
                // 2. `release_detached_block` 内部先等待对应元数据块在途回写完成并作废缓存窗口，最后才释放块位图供后续复用。
                unsafe {
                    volume.release_detached_block(block);
                }
            }
            drop(root);
            accepted += pulled;
        }
        Ok((start, accepted))
    }

    /// 从文件指定字节偏移读取数据到内核切片（测试辅助入口，内部复用 `read_into`）。
    #[cfg(feature = "ktest")]
    pub fn read_at(&self, offset: u64, dst: &mut [u8]) -> Result<usize, FsError> {
        self.read_into(offset, &mut ReadBuffer::kernel(dst), None)
    }

    /// 从普通文件指定偏移读取数据并写入目标 `ReadBuffer`：
    /// 每轮先在 `invalidate` 锁外执行预读与缓存页槽位预备，再在 `invalidate` 读锁内填充页面并拷贝到栈上中转缓冲区，最后锁外推送给用户态接收端（防止用户缓冲区缺页与文件锁死锁）。
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

    /// 为只读内存映射缺页（`file fault`）获取并填充指定页索引的物理页：先在 `invalidate` 锁外准备槽位，再持 `invalidate` 读锁校验 EOF 边界并触发读取。
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

    /// 当写入偏移跨越当前文件末尾（EOF）时，将原尾页中从旧 `size` 到新写入起点（或页末）之间的空洞字节清零。
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

    /// 在持有 `write` 与 `invalidate` 锁的前提下，读取尾部非页对齐位置所在的物理页，并查询页内待清零区间中哪些文件系统块已实际映射了磁盘块（此阶段不修改页内容）。
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

    /// 执行尾页补零计划：先撤销用户态共享可写页表项（`mkclean`）防止并发改写，再将指定区间清零，并仅对已映射磁盘块的范围标记脏位。
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

    /// 供已持有 `write` 与 `invalidate` 锁的内部写入/截断路径同步获取并填充指定文件页。
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

    /// 将锁外预备的页面候选发布到 `FileMapping`（调用方须持有 `invalidate` 读锁或写锁）；若当前线程赢得首次填充权，则按内存清零或磁盘块映射发起读取，并等待页面就绪。
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
                // SAFETY:
                // 1. `read` 是当前新建内存页的唯一初始化读取凭据，未提交给任何块设备 DMA；
                // 2. 上方循环已将整页 `PAGE_SIZE` 字节全部写零初始化，`bytes_mut()` 可变借用已结束，调用 `complete` 标记整页就绪完全合法。
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

    /// 解引用借用底层固定的 `Inode`（不提供 `DerefMut` 或裸 `Arc` 访问，保证内嵌等待队列不会被移动）。
    fn deref(&self) -> &Inode {
        &self.0
    }
}

impl Inode {
    /// 返回节点在所属文件系统内的 64 位 `Inode` 编号（跨文件系统判等须结合 `device()` 或 `InodeRef::ptr_eq`）。
    pub fn number(&self) -> u64 {
        self.number
    }

    /// 获取磁盘节点所属的 `Ext4Volume` 引用；若当前为内存文件系统节点则返回 `InvalidOperation`。
    pub fn disk_volume(&self) -> Result<&Arc<Ext4Volume>, FsError> {
        match &self.backend {
            InodeBackend::Disk(disk) => Ok(&disk.filesystem),
            InodeBackend::Memory { .. } => Err(FsError::InvalidOperation),
        }
    }

    /// 供已确认处于磁盘分支的内部方法直接借用 `DiskInode` 状态；若误在内存节点上调用则触发 panic。
    fn disk(&self) -> &DiskInode {
        match &self.backend {
            InodeBackend::Disk(disk) => disk,
            InodeBackend::Memory { .. } => panic!("memory inode entered disk protocol"),
        }
    }

    /// 等待节点的单飞加载结果就绪并返回内部 `InodeState` 引用；若加载失败则直接返回缓存的错误码。
    fn wait_loaded(&self) -> Result<&InodeState, FsError> {
        sched::assert_switchable();
        let inode = self.pinned().get_ref();
        loop {
            if let Some(result) = inode.data.get() {
                return result.as_ref().map_err(|error| *error);
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `Inode` 始终封装在私有 `Arc` 中且含有 `PhantomPinned`，由调用方持有的引用保活，保证 `inode.waiters` 地址稳定有效；
            // 2. `node` 经 `pin!` 固定在当前栈帧，`_registration` 局部变量在每次循环迭代末尾或提前返回时先于 `node` 与 `waiter` 析构摘链。
            let _registration = unsafe { inode.waiters.register(node.as_mut()) };
            if let Some(result) = inode.data.get() {
                return result.as_ref().map_err(|error| *error);
            }
            waiter.park();
        }
    }

    /// 直接借用已完成加载的 `InodeState`（仅限已持有访问令牌、保证节点必定加载成功的内部或 `Drop` 路径调用）。
    fn loaded_state(&self) -> &InodeState {
        self.data
            .get()
            .expect("access token precedes inode load")
            .as_ref()
            .expect("access token retains a failed inode")
    }

    /// 等待节点加载完成后，在自旋锁内拷贝返回当前运行态元数据快照（内存普通文件动态按驻留页数折算 512 字节扇区数 `blocks`）。
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

    /// 在进入 `NOFS` 作用域并持有磁盘块树根共享读锁期间，复制一份元数据快照一并交给闭包消费（调用闭包前已释放元数据自旋锁）。
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

    /// 查询指定逻辑块号在磁盘上的物理映射状态（测试辅助接口）。
    #[cfg(feature = "ktest")]
    pub fn map_block(&self, logical: u64) -> Result<BlockMapping, FsError> {
        self.with_map_read(|metadata, root| {
            self.disk_volume()?.map_block(metadata, root, logical, None)
        })
    }

    /// 在 `write` 与 `root` 双重独占写锁保护下执行受控的节点属性修改并同步更新磁盘 `inode-table` 记录（测试辅助接口，不用于修改文件尺寸或块布局）。
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

    /// 借用节点的页面缓存映射 `FileMapping`（调用方必须在所有裸页访问与在途 I/O 完成前持续保活对应的 `InodeRef`）。
    pub(crate) fn mapping(&self) -> &FileMapping {
        &self.mapping
    }

    /// 读取符号链接的目标原始字节序列（内存后端直接克隆字节数组，磁盘后端根据快速内嵌或数据块映射读取）。
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

    /// 将 `&Inode` 包装为 `Pin<&Inode>`，供内嵌侵入式等待队列注册使用。
    fn pinned(&self) -> Pin<&Self> {
        // SAFETY:
        // 1. `Inode` 仅通过本模块的私有构造函数在 `Arc` 堆内存中创建，且包含 `PhantomPinned`，外部无法获取 `Arc<Inode>` 或 `&mut Inode`；
        // 2. 本模块绝不会将 `Inode` 移出堆内存或原地覆盖替换，在 `Arc` 强引用彻底归零释放前其内存地址恒定不变。
        unsafe { Pin::new_unchecked(self) }
    }
}

impl InodeWriteGuard<'_> {
    /// 在持有节点独占写锁期间获取当前运行态元数据快照。
    pub(crate) fn metadata_snapshot(&self) -> InodeMetadata {
        *self.state.metadata.lock()
    }

    /// 在节点独占写锁保护下，预先校验并准备硬链接计数增减（`delta`）及 `ctime` 更新凭据。
    pub(crate) fn prepare_link_delta(
        &self,
        delta: i32,
        now_ns: u64,
    ) -> Result<PreparedInodeMetadata, FsError> {
        prepare_link_delta(self.inode, self.state, delta, now_ns)
    }

    /// 在节点独占写锁保护下，预先准备仅刷新 `ctime` 时间戳的元数据更新凭据。
    pub(crate) fn prepare_ctime(&self, now_ns: u64) -> Result<PreparedInodeMetadata, FsError> {
        prepare_inode_ctime(self.inode, self.state, now_ns)
    }

    /// 在节点独占写锁保护下校验调用者 `uid`/`gid` 权限（非 root 且非所属组时自动剥离 `S_ISGID` 位），并准备 `chmod` 权限与 `ctime` 更新凭据。
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

    /// 在节点独占写锁保护下校验并准备 `utimens` 访问/修改时间（`atime`/`mtime`）及当前 `ctime` 更新凭据（传入 `None` 的时间字段保持原值不变）。
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

    /// 在已持有节点独占写锁的前提下，消费准备好的凭据并将新属性写入磁盘缓冲区与内存快照。
    pub(crate) fn publish_metadata(&self, prepared: PreparedInodeMetadata) {
        publish_inode_metadata(self.inode, self.state, prepared);
    }
}

impl DirectoryReadGuard<'_> {
    /// 在持有目录共享读锁期间获取当前目录的元数据快照。
    pub(crate) fn metadata_snapshot(&self) -> InodeMetadata {
        *self.state.metadata.lock()
    }

    /// 在已持有的目录共享读锁内按名称查找子项并加载其权威 `InodeRef`。
    pub(crate) fn lookup(
        &self,
        cache: &InodeCache,
        name: &[u8],
    ) -> Result<Option<InodeRef>, FsError> {
        lookup_locked(self.inode, cache, name)
    }

    /// 在目录共享读锁保护下按游标读取下一个有效目录项。
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

    /// 在目录共享读锁保护下将外部 64 位偏移 `cookie` 转换为内部目录遍历游标。
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

    /// 在目录共享读锁保护下获取目录末尾的 `SEEK_END` 游标 `cookie`。
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
    /// 在持有目录独占写锁期间获取当前目录的元数据快照。
    pub(crate) fn metadata_snapshot(&self) -> InodeMetadata {
        *self.state.metadata.lock()
    }

    /// 在已持有的目录独占写锁内按名称重新查找子项（不重复加读锁）。
    pub(crate) fn lookup(
        &self,
        cache: &InodeCache,
        name: &[u8],
    ) -> Result<Option<InodeRef>, FsError> {
        lookup_locked(self.inode, cache, name)
    }

    /// 复用当前目录独占写锁准备目录自身的硬链接计数增减与 `ctime` 更新凭据。
    pub(crate) fn prepare_link_delta(
        &self,
        delta: i32,
        now_ns: u64,
    ) -> Result<PreparedInodeMetadata, FsError> {
        prepare_link_delta(self.inode, self.state, delta, now_ns)
    }

    /// 复用当前目录独占写锁准备仅刷新目录 `ctime` 的元数据更新凭据。
    pub(crate) fn prepare_ctime(&self, now_ns: u64) -> Result<PreparedInodeMetadata, FsError> {
        prepare_inode_ctime(self.inode, self.state, now_ns)
    }

    /// 在当前目录独占写锁内提交并发布目录自身的元数据更新凭据。
    pub(crate) fn publish_metadata(&self, prepared: PreparedInodeMetadata) {
        publish_inode_metadata(self.inode, self.state, prepared);
    }

    /// 检查当前目录是否为空（仅包含 `.` 和 `..`，不含任何其他有效子项；磁盘后端同时保留块校验和与 I/O 错误检查）。
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

    /// 两阶段准备在当前目录中插入新目录项 `(name, number, kind)`，并同步计算父目录链接计数变化与修改时间。
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

    /// 两阶段准备从当前目录中移除已确认存在的子目录项 `name`，并计算父目录链接计数变化。
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

    /// 两阶段准备在跨父目录 `rename` 操作中覆盖目标父目录下已有的同名目录项 `name`。
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

    /// 两阶段准备同一父目录内的 `rename` 移动或覆盖：在单次目录编辑计划中原子处理移除源项与写入/替换目标项。
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

    /// 两阶段准备在子目录跨父移动时更新该子目录内部的 `..` 父目录编号记录与时间戳。
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

    /// 在持有目录独占写锁的前提下，获取块树根写锁并一气呵成提交已准备好的目录块编辑与父目录 `inode-table` 元数据记录。
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

    /// 将底层 `ext4` 目录编辑计划与父目录 `inode-table` 映射更新绑定为统一的 `PreparedDirectoryMutation`；若元数据准备失败则立即回滚目录块预留。
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
    /// 若磁盘目录修改凭据未经 `publish` 即被丢弃，自动回滚本次预留的目录块，保持原目录树与运行态元数据不变。
    fn drop(&mut self) {
        if let Some(edit) = self.edit.take() {
            edit.cancel(&self.volume);
        }
    }
}

/// 计算增减 `delta` 后的 `ext4` 硬链接计数，校验无下溢且不超过磁盘 16 位字段上限（`u16::MAX`）。
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

/// 为尚未挂载目录项的新建磁盘 `Inode` 构造初始运行态元数据，校验文件大小、扇区编码与扩展时间戳宽度。
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

/// 计算硬链接计数变化与最新 `ctime`，并预留校验对应的底层节点元数据更新凭据。
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

/// 计算当前 `ctime` 的可编码值，并预留校验对应的底层节点元数据更新凭据。
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

/// 校验属性变更合法性，读取磁盘 `inode-table` 缓冲区并在块树根读锁下核对磁盘记录与当前内存状态完全一致，返回可提交凭据。
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

/// 将已准备校验通过的节点属性补丁写入磁盘 `inode-table` 缓冲区，并同步更新内存中的元数据快照。
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

/// 在调用方已持有父目录读锁或写锁的前提下，查找子目录项名称并通过 `iget` 加载对应的子 `InodeRef`。
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
    /// 在磁盘 `Inode` 读取 I/O 结束后向 `Once` 写入唯一加载结果，若失败则移除缓存槽占位，并唤醒所有等待该节点的线程。
    fn complete(&mut self, result: Result<InodeData, FsError>) {
        self.publish(result);
        self.active = false;
    }

    /// 初始化 `inode.data` 的单次结果槽（若此前已设置则保持首次结果不变），并在加载失败时清理缓存槽、广播唤醒等待队列。
    fn publish(&self, result: Result<InodeData, FsError>) {
        let result = self.inode.data.call_once(|| result.map(InodeState::new));
        if result.is_err() {
            self.domain.remove_failed(self.inode);
        }
        self.inode.waiters.wake_all();
    }
}

impl Drop for LoadGuard<'_> {
    /// 若单飞加载线程在调用 `complete` 前异常退出，自动发布 `LoadAborted` 错误并清理槽位，防止并发等待线程永久挂起。
    fn drop(&mut self) {
        if self.active {
            self.publish(Err(FsError::LoadAborted));
        }
    }
}

/// 在已持有索引自旋锁的前提下按 `(filesystem, number)` 查找缓存槽：
/// - 命中 `Cached` 槽位直接递增活跃计数返回；
/// - 命中尚未启动且仍有硬链接（`links > 0`）的 `Requested` 槽位时取消淘汰恢复为 `Cached`；
/// - 槽位正处于 `Running` 回收或 `links == 0` 待删除时返回 `Busy`，处于 `Failed` 时返回保留的错误码。
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
            // 后台工作线程尚未认领该槽位；若节点仍有目录项硬链接，可直接取消本次淘汰请求原地复用。
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

/// 在已持有索引自旋锁的前提下递增槽位的外部活跃引用计数 `users`，并构造对应的 `InodeRef`。
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

/// 将存在回收失败槽位的缓存域挂入全局 `FAILED_EVICTIONS` 单向保活链表，确保未落盘的脏数据与错误信息不会随外部引用消失而静默丢失。
fn retain_failed(domain: &Arc<InodeDomain>) {
    let mut head = FAILED_EVICTIONS.lock();
    if !domain.retained.swap(true, Ordering::Relaxed) {
        *domain.retained_next.lock() = head.take();
        *head = Some(domain.clone());
    }
}

/// 当缓存域内所有回收失败槽位均已清理完毕后，将其从全局 `FAILED_EVICTIONS` 链表摘除，并在全局锁外释放保活强引用。
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

/// 全局 `EVICTION_QUEUE` 工作线程执行的异步回收回调：消费入队时移交的缓存域强引用，逐个认领并回收无外部引用的 `Requested` 槽位，最后对已关闭域中的空闲卷执行同步停机。
fn evict_inodes(work: Pin<&Work>) {
    // SAFETY:
    // 1. `InodeDomain::queue` 在每次工作项成功入队时，都会预先通过 `Arc::into_raw` 泄露一份独立的 `Arc<InodeDomain>` 强引用；
    // 2. 工作队列保证每次成功入队恰好触发一次本回调执行，`work.context()` 保存的即为该 `InodeDomain` 堆地址，在此通过 `Arc::from_raw` 唯一接管并在函数末尾释放。
    let domain = unsafe { Arc::from_raw(work.context() as *const InodeDomain) };
    loop {
        // Step 1: 在索引自旋锁内挑选一个无外部引用且页面缓存仅含干净缓存页的待回收槽位，切换为 `Running` 状态并在锁外执行 `evict()`。
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

    // Step 2: 若所有槽位均无残留失败状态则解除全局保活，并在缓存域已关闭时停机清理不再被槽位引用的卷。
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

/// 处理分块循环写入的中途错误：若此前已成功写入部分字节（`accepted > 0`）则按 POSIX 短写语义返回已写入字节数，否则返回首次错误。
fn write_result(start: u64, accepted: usize, error: FsError) -> Result<(u64, usize), FsError> {
    if accepted == 0 {
        Err(error)
    } else {
        Ok((start, accepted))
    }
}

/// 根据 `ext4` 节点采用 Extent 树还是传统直接/三级间接块布局，计算其支持的最大文件字节偏移上限（不超过 `i64::MAX`）。
fn write_limit(g: &crate::fs::ext4::Geometry, flags: u32) -> u64 {
    let blocks = if flags & crate::fs::ext4::codec::EXTENTS != 0 {
        codec::EXTENT_LOGICAL_BLOCKS
    } else {
        let n = (g.block_size / 4) as u64;
        codec::DIRECT_BLOCKS as u64 + n + n * n + n * n * n
    };
    (blocks * g.block_size as u64).min(i64::MAX as u64)
}

/// 根据本次写入偏移、写入长度以及新增/移除的块数量，计算更新后的文件 `size`、512 字节扇区数 `blocks`、`HUGE_FILE` 标志及时间戳。
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
    /// 借用两阶段准备中尚未提升硬链接计数的新建 `InodeRef`。
    pub(crate) fn inode(&self) -> &InodeRef {
        match self {
            Self::Disk(prepared) => prepared.inode(),
            Self::Memory { inode, .. } => inode.as_ref().expect("memory inode token consumed"),
        }
    }

    /// 在父目录编辑就绪后正式发布新节点的初始硬链接计数并返回 `InodeRef`。
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
    /// 构造内存文件系统节点的初始运行态元数据（目录初始大小计入 `.` 与 `..` 两项）。
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
    /// 返回节点所属文件系统的设备号标识（`st_dev`）。
    pub fn device(&self) -> u64 {
        match &self.backend {
            InodeBackend::Disk(disk) => disk.filesystem.device(),
            InodeBackend::Memory { filesystem, .. } => filesystem
                .upgrade()
                .expect("memory inode without filesystem owner")
                .device(),
        }
    }

    /// 返回节点的最佳 I/O 块字节大小（`st_blksize`：磁盘节点返回卷块大小，内存节点返回物理页大小 `PAGE_SIZE`）。
    pub fn block_size(&self) -> u32 {
        match &self.backend {
            InodeBackend::Disk(disk) => disk.filesystem.geometry().block_size as u32,
            InodeBackend::Memory { .. } => PAGE_SIZE as u32,
        }
    }

    /// 返回特殊设备节点关联的设备号（`st_rdev`，普通文件与目录为 `0`）。
    pub fn rdev(&self) -> u64 {
        self.loaded_state().metadata.lock().rdev
    }

    /// 判断当前节点是否属于内存文件系统后端。
    pub fn is_memory(&self) -> bool {
        matches!(self.backend, InodeBackend::Memory { .. })
    }

    /// 若当前节点为内存文件系统中的特殊节点（设备、FIFO、Socket 等），返回其具体角色描述。
    pub fn memory_special(&self) -> Option<MemorySpecial> {
        match &self.backend {
            InodeBackend::Memory {
                content: MemoryContent::Special(special),
                ..
            } => Some(*special),
            _ => None,
        }
    }

    /// 获取内存目录节点内部的目录项表自旋锁引用；非内存目录节点返回 `NotDirectory`。
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
    /// 将内存目录树内部持有的 `Arc<Inode>` 包装为同时保活所属 `MemoryFs` 的外部 `InodeRef` 句柄。
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

    /// 提取内存节点的内部 `Arc<Inode>` 供 `MemoryFs` 根或父目录项持有（不克隆外层 `Arc<MemoryFs>` 以免形成引用环）。
    pub(crate) fn memory_arc(&self) -> Arc<Inode> {
        assert!(self.is_memory(), "disk inode used as memory authority");
        self.0.clone()
    }

    /// 分配并立即完成初始化一个内存文件系统节点及其页面缓存映射。
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

    /// 获取内存节点句柄强引用的 `MemoryFs` 实例；若当前为磁盘节点则返回 `InvalidOperation`。
    fn memory_fs(&self) -> Result<&Arc<MemoryFs>, FsError> {
        match &self.1 {
            InodeOwner::Memory(fs) => Ok(fs),
            _ => Err(FsError::InvalidOperation),
        }
    }

    /// 在为内存普通文件分配新物理页前，向所属 `MemoryFs` 预留一页配额记账凭据。
    pub(crate) fn reserve_memory_page(&self) -> Result<MemoryPageCharge, FsError> {
        self.memory_fs()?.reserve_page()
    }

    /// 检查所属内存文件系统是否以读写模式挂载，只读挂载时返回 `ReadOnly`。
    fn require_memory_write(&self) -> Result<(), FsError> {
        if self.memory_fs()?.mode() != AccessMode::ReadWrite {
            return Err(FsError::ReadOnly);
        }
        Ok(())
    }

    /// 按照父目录 `S_ISGID` 继承规则与进程 `umask` 计算新内存节点的权限与属组，并在 `MemoryFs` 中预分配 `links = 0` 的节点。
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

    /// 在内存目录中按名称查找子项（特殊处理 `.` 与 `..`），返回包装好文件系统强引用的 `InodeRef`。
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

    /// 在内存文件系统初始化阶段（如挂载创建设备节点时）将子节点直接插入父目录，跳过常规只读挂载权限检查。
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

    /// 根据节点后端类型将写入请求分派至内存文件写入或磁盘卷缓冲写入路径。
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

    /// 根据节点后端类型将截断请求分派至内存文件截断或磁盘卷截断路径。
    fn truncate_backend(&self, new_size: u64) -> Result<(), FsError> {
        if self.is_memory() {
            return self.truncate_memory(new_size);
        }
        self.truncate(self.disk_volume()?.claim(), new_size)
    }

    /// 内存普通文件的写入主循环：每次先在 `invalidate` 锁外从用户源缓冲区拉取数据，再持 `invalidate` 写锁清零跨 EOF 尾页空洞、拷贝数据进物理页并更新文件大小与时间戳。
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

    /// 若内存文件原末尾所在页已驻留在缓存中，先撤销其用户态共享可写映射（`mkclean`），再将 `[start, end)` 暴露出的页内区间清零（未分配的稀疏空洞页无需处理）。
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

    /// 截断或扩展内存普通文件：更新文件 `size` 后，以两轮用户态页表卸载（`unmap_range`）包围页面缓存截断清理。
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
        // SAFETY:
        // 1. 当前任务同时持有 `state.write` 独占锁、`state.invalidate` 独占锁与 `mapping` 回写互斥锁；
        // 2. 新 `size` 已发布且已完成第一轮尾部页表卸载与跨核 TLB 同步，内存页从不提交块设备 DMA，清理后紧接着执行第二轮页表卸载。
        let pages = unsafe { self.mapping.truncate_pages(self, new_size, PAGE_SIZE) };
        #[cfg(feature = "ktest")]
        self.truncate_phase(TruncatePhase::BeforeSecondUnmap);
        self.mapping.unmap_range(first..u64::MAX, true);
        pages
    }

    /// 为内存普通文件的共享可写缺页（`mkwrite`）预先获取或分配零初始化物理页，并记录当前文件 `size` 快照。
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

/// 将单调或实时纳秒时间戳直接拆分为秒与纳秒表示的 `InodeTime`。
fn memory_time(now_ns: u64) -> InodeTime {
    InodeTime {
        seconds: (now_ns / time::NSEC_PER_SEC) as i64,
        nanoseconds: (now_ns % time::NSEC_PER_SEC) as u32,
    }
}

/// 以可失败内存分配方式精确克隆原始字节切片（文件名与符号链接目标允许任意非空字节序列，不要求合法 UTF-8）。
fn try_bytes(bytes: &[u8]) -> Result<Vec<u8>, FsError> {
    let mut copy = Vec::new();
    copy.try_reserve_exact(bytes.len())
        .map_err(|_| FsError::NoMemory)?;
    copy.extend_from_slice(bytes);
    Ok(copy)
}

/// 内存目录修改操作的类型化描述，在准备阶段借用调用方传入的名称切片。
enum MemoryEdit<'a> {
    /// 插入新子目录项 `(名称, 子 Inode 编号, 节点类型)`。
    Insert(&'a [u8], u64, InodeKind),
    /// 按名称移除现有子目录项。
    Remove(&'a [u8]),
    /// 跨父目录重命名覆盖已有同名项 `(名称, 新子 Inode 编号, 新节点类型)`。
    Replace(&'a [u8], u64, InodeKind),
    /// 同父目录内重命名移动 `(旧名称, 新名称, 源编号, 源类型, 可选的被覆盖目标编号与类型)`。
    Move(&'a [u8], &'a [u8], u64, InodeKind, Option<(u64, InodeKind)>),
    /// 更新当前目录记录的父目录编号（`..`）。
    Dotdot(u64),
}

impl MemoryDirectory {
    /// 在外层目录写锁串行化编辑的前提下，先在自旋锁外预留堆内存容量，再持短期自旋锁克隆出一份候选 `MemoryDirectory` 副本。
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

    /// 在候选目录项数组中线性查找精确匹配 `name` 的下标，未找到时返回 `NotFound`。
    fn position(&self, name: &[u8]) -> Result<usize, FsError> {
        self.entries
            .iter()
            .position(|entry| entry.name.as_slice() == name)
            .ok_or(FsError::NotFound)
    }

    /// 校验名称合法性与唯一性后，向候选目录追加新目录项并分配下一个单调递增的游标 `cookie`。
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

    /// 校验名称后从候选目录副本中移除并返回对应的子目录项记录。
    fn remove(&mut self, name: &[u8]) -> Result<MemoryEntry, FsError> {
        validate_memory_name(name)?;
        let position = self.position(name)?;
        Ok(self.entries.remove(position))
    }

    /// 在候选目录副本中原地替换同名目录项指向的子 `Inode` 与类型，保留原目录项的游标 `cookie` 不变。
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

    /// 在同一份候选目录副本中校验源与目标目录项身份，完成同目录内的重命名移动或覆盖，返回目录内容是否发生实际变化。
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
    /// 为内存目录准备增删改事务：在克隆出的私有目录副本上完成全部分配、校验与链接计数计算，使后续 `publish` 只需在自旋锁内做无失败指针交换。
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
    /// 按内存目录游标 `cookie` 读取下一个目录项：`0` 返回 `.`，`1` 返回 `..`，`>= 2` 返回 `cookie` 大于等于当前值的最小单调子项。
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

/// 计算内存节点的硬链接计数增减结果，防止减法下溢（返回 `Corrupt`）或加法溢出（返回 `TooManyLinks`）。
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

/// 校验内存目录子项名称：禁止空名、含 `\0` 或 `/`、`.` 与 `..`，且字节长度不得超过 `NAME_MAX`。
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
