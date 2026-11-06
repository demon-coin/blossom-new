//! 目录项缓存（Dentry Cache / dcache）与名字绑定生命周期管理。
//!
//! # 模块职责与架构定位
//! 本模块维护虚拟文件系统（VFS）在内存中的目录树拓扑与名字查找缓存：
//! - **上游调用者**：路径解析器与命名空间修改操作（`fs::path` 中的 `lookup`、`create`、`mkdir`、`unlink`、`rename` 等）。
//! - **下游依赖**：`InodeCache` 与 `InodeRef`（在缓存未命中时发起底层目录项查找）。
//!
//! # 核心设计与 Dentry 状态机
//! ## 1. 单飞（Singleflight）并发加载与绑定状态机
//! 每个 `Dentry` 内部包含一个 `Once<Result<Binding, FsError>>` 一次性状态槽与等待队列 `WaitQueue`：
//! ```text
//!                         [ 候选 Dentry 创建 (InLookup: Once 未初始化) ]
//!                                              |
//!                      publish() 抢占索引槽位，或 try_prepare_child() 预留槽位
//!                                              |
//!               +------------------------------+------------------------------+
//!               |                              |                              |
//!        目录查找到 Inode               目录返回 NotFound               I/O 错误或 LoadGuard
//!       或 bind_positive()                     |                         未完成即异常析构
//!               |                              v                              |
//!               v                   [ Ok(Binding::Negative) ]                 v
//!   [ Ok(Binding::Positive(inode)) ]   保留在缓存索引中加速负查找        [ Err(FsError) ]
//!      保留在缓存索引中，后续可被       后续 create/mkdir 可将其替换    从索引摘除 (remove_failed)
//!       unlink/rename 修改拓扑                                         并唤醒等待者返回该错误
//! ```
//! - 当多个任务并发查找同一 `(parent, name)` 时，首个在 `publish` 中胜出的任务获得 `LoadGuard` 并在自旋锁外执行底层目录 I/O；其余任务拿到同一 `DentryRef` 并在 `DentryRef::inode()` 中睡眠等待 `Once` 初始化完成。
//!
//! ## 2. 预留槽位机制（`PreparedDentry` / `PreparedPositiveDentry`）
//! - 在创建文件、目录或硬链接前，先通过 `try_prepare_child` 在锁外分配 `Dentry` 并在 `DentryIndex` 中预留一个槽位计数（`reserved += 1`），或锁定一个待替换的负绑定（`Binding::Negative`）条目。
//! - 底层文件系统成功写入目录项后，调用 `PreparedPositiveDentry::publish()` 将新条目推入索引（零内存分配、不会因 `NoMemory` 失败回滚）；若底层操作失败，`PreparedDentry::drop` 自动归还预留配额。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **父目录 Inode 读写睡眠锁**（`inode.directory_read()` 或写锁）：必须在调用 `lookup`、`try_prepare_child`、`unhash`、`move_entry` 前由调用方持有，也可在 `lookup` 内部获取读锁。
//! 2. **缓存索引自旋锁**（`DentryCache::entries`）：保护 `entries` 向量与 `reserved` 计数；持锁期间严禁执行内存分配、块 I/O 或睡眠等待，且所有可能触发级联析构的旧 `DentryRef` 必须移出锁外再 `drop`。
//! 3. **单条目拓扑自旋锁**（`Dentry::topology`）：保护 `parent`、`name` 与 `hashed` 标志。若需同时持有索引锁与拓扑锁，**必须先锁 `DentryCache::entries`，再锁 `Dentry::topology`**。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::marker::PhantomPinned;
use core::mem;
use core::pin::{pin, Pin};
#[cfg(feature = "ktest")]
use core::sync::atomic::{AtomicUsize, Ordering};
use spin::Once;

use crate::fs::{FsError, InodeCache, InodeKind, InodeRef};
use crate::sched;
use crate::sync::SpinLock;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// 内核测试专用的查找同步钩子类型，用于在 `InLookup` 状态发布后、底层目录查询前注入并发停点。
#[cfg(feature = "ktest")]
type LookupHook = Arc<dyn Fn(&DentryRef) + Send + Sync>;

/// 固定容量上限的目录项缓存（dcache）。
///
/// 缓存索引强引用已发布的 `DentryRef`；而每个 `Dentry` 仅强引用其父 `DentryRef` 与正绑定的 `InodeRef`，
/// 不反向持有 `DentryCache`，从而避免循环引用。
/// ponytail: 固定预算内线性查找，预算增大且查找成为瓶颈时改为有限散列表。
pub struct DentryCache {
    /// 受自旋锁保护的活动目录项列表与预留槽位计数。
    entries: SpinLock<DentryIndex>,
    /// 缓存允许容纳的最大目录项槽位数（含已发布项与预留项之和，单位：个）。
    capacity: usize,
    /// 内核测试钩子：在单飞加载胜出者发布 `InLookup` 占位后、发起目录查找前触发。
    #[cfg(feature = "ktest")]
    lookup_hook: SpinLock<Option<LookupHook>>,
    /// 内核测试计数器：统计实际发起底层目录查找（单飞胜出）的累计次数。
    #[cfg(feature = "ktest")]
    loads: AtomicUsize,
}

/// `DentryCache` 内部受自旋锁保护的索引容器。
struct DentryIndex {
    /// 当前处于已哈希（`hashed == true`）状态的所有缓存目录项列表。
    entries: Vec<DentryRef>,
    /// 已被 `PreparedDentry` 预留但尚未正式发布的槽位数量（单位：个）。
    reserved: usize,
}

/// 目录项（`Dentry`）的引用计数共享句柄。
///
/// 克隆 `DentryRef` 仅增加底层 `Arc<Dentry>` 的强引用计数，不会复制名字字节或改变对象指针身份。
#[derive(Clone)]
pub struct DentryRef(Arc<Dentry>);

/// 单个目录项节点，记录目录树中的父子连接关系、组件名字以及与底层 `Inode` 的绑定状态。
///
/// 由于内嵌的侵入式等待队列 `waiters` 在任务等待期间被栈上 `WaitNode` 引用，该结构体通过 `PhantomPinned` 标记为不可移动，
/// 且始终封装在私有 `Arc` 分配内。
pub(crate) struct Dentry {
    /// 受自旋锁保护的父目录引用、组件名字与索引连接状态。
    topology: SpinLock<DentryTopology>,
    /// 一次性初始化的名字解析结果：未初始化时表示正在加载（`InLookup`）；
    /// 初始化后永久固定为正绑定（`Positive`）、负绑定（`Negative`）或首次加载错误。
    binding: Once<Result<Binding, FsError>>,
    /// 并发查找同一目录项的任务在 `binding` 未完成前挂起的等待队列。
    waiters: WaitQueue,
    /// 标记结构体不可在内存中移动，确保侵入式等待队列 `waiters` 的地址稳定。
    _pinned: PhantomPinned,
}

/// `Dentry` 内部受 `topology` 自旋锁保护的命名空间拓扑字段。
struct DentryTopology {
    /// 父目录项引用；对于文件系统挂载根或匿名未哈希目录项为 `None`。
    parent: Option<DentryRef>,
    /// 当前目录项的原始字节名字快照，使用 `Arc` 共享以避免路径拼接与查询时重复分配。
    name: Arc<Vec<u8>>,
    /// 标识该目录项当前是否登记在 `DentryCache` 的活动索引列表中。
    hashed: bool,
}

/// 目录项名字查找完成后的绑定状态。
enum Binding {
    /// 正绑定：名字对应磁盘或内存中实际存在的 `InodeRef`。
    Positive(InodeRef),
    /// 负绑定：名字在父目录中不存在（缓存 `ENOENT` 结果以加速后续重复查找）。
    Negative,
}

/// 目录修改操作（如 `create`、`mkdir`）在写入底层目录前预留的目录项槽位凭证。
///
/// 若操作中途失败且未调用 `publish`，析构函数会自动归还所占用的缓存配额。
pub(crate) struct PreparedDentry<'a> {
    /// 所属的目录项缓存引用。
    cache: &'a DentryCache,
    /// 已在锁外分配好的新目录项候选实例；发布时被 `take()` 取出。
    dentry: Option<DentryRef>,
    /// 若目标名字在缓存中已存在负绑定（`Negative`），记录该待替换的旧负绑定条目。
    replaced: Option<DentryRef>,
    /// 标识是否在 `DentryIndex::reserved` 中占用了新的配额计数（复用旧负绑定槽位时为 `false`）。
    reserved: bool,
}

/// 已完成正绑定（`Binding::Positive`）的预留目录项凭证。
///
/// 通过类型状态保证只有已绑定有效 `InodeRef` 的候选目录项才能提交发布到 `DentryCache` 索引中。
pub(crate) struct PreparedPositiveDentry<'a>(PreparedDentry<'a>);

/// 单飞（Singleflight）目录项加载任务的守卫。
///
/// 当任务在 `publish` 中胜出并将处于 `InLookup` 状态的候选目录项放入索引后，`active` 置为 `true`。
/// 若任务在调用 `complete` 前因 panic 或提前返回而退出，`Drop` 实现会将绑定结果设为 `FsError::LoadAborted`，
/// 从索引中摘除该失败条目并唤醒所有等待者，防止等待者永久挂起。
struct LoadGuard<'a> {
    /// 所属的目录项缓存引用。
    cache: &'a DentryCache,
    /// 当前正在执行底层目录查找的目标候选目录项。
    dentry: &'a DentryRef,
    /// 标识当前守卫是否仍背负未完成的 `InLookup` 发布责任。
    active: bool,
}

impl DentryCache {
    /// 创建指定容量上限的 `DentryCache`，并一次性预分配底层索引向量空间，保证后续在索引自旋锁内插入条目时不发生内存重分配。
    pub(crate) fn try_new(capacity: usize) -> Result<Arc<Self>, FsError> {
        if capacity == 0 {
            return Err(FsError::InvalidOperation);
        }

        let mut entries = Vec::new();
        entries
            .try_reserve_exact(capacity)
            .map_err(|_| FsError::NoMemory)?;
        Arc::try_new(Self {
            entries: SpinLock::new(DentryIndex {
                entries,
                reserved: 0,
            }),
            capacity,
            #[cfg(feature = "ktest")]
            lookup_hook: SpinLock::new(None),
            #[cfg(feature = "ktest")]
            loads: AtomicUsize::new(0),
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 返回目录项缓存配置的最大槽位容量（单位：个）。
    pub(crate) fn capacity(&self) -> usize {
        self.capacity
    }

    /// 在父目录 `parent` 下查找单级路径组件 `name` 对应的 `DentryRef`。
    ///
    /// # 调用上下文
    /// 必须在普通可睡眠任务上下文中调用。函数内部会获取父目录 Inode 的目录读锁（`directory_read`），
    /// 并覆盖缓存查询、单飞（Singleflight）首次加载以及等待并发加载完成的全过程。
    pub fn lookup(
        &self,
        inodes: &InodeCache,
        parent: &DentryRef,
        name: &[u8],
    ) -> Result<DentryRef, FsError> {
        // Step 1: 校验当前处于可睡眠上下文，获取父目录 Inode 与目录读锁，并校验组件名字合法性。
        sched::assert_switchable();
        let inode = parent.inode()?;
        let directory = inode.directory_read()?;
        validate_name(name)?;

        // Step 2: 在缓存索引自旋锁临界区内查找是否已命中缓存；若命中则等待其绑定完成并返回。
        let (existing, full) = {
            let index = self.entries.lock();
            (
                find(&index.entries, parent, name),
                self.capacity <= index.entries.len() + index.reserved,
            )
        };
        if let Some(dentry) = existing {
            dentry.inode()?;
            return Ok(dentry);
        }

        // Step 3: 若 Dentry 缓存已满，或父目录属于磁盘文件系统且 Inode 缓存已满（闲置的正绑定 Dentry 会强引用磁盘 Inode），
        // 先触发一次闲置目录项回收，随后重新检查目标名字是否已被并发插入或是否有空闲槽位。
        if full || (!inode.is_memory() && inodes.is_full()) {
            self.prune_unused();
            // 回收锁外释放期间同一 (parent, name) 可能已被其他任务发布，需再次查询索引。
            let index = self.entries.lock();
            let existing = find(&index.entries, parent, name);
            let full = self.capacity <= index.entries.len() + index.reserved;
            drop(index);
            if let Some(dentry) = existing {
                dentry.inode()?;
                return Ok(dentry);
            }
            if full {
                return Err(FsError::NoMemory);
            }
        }

        // Step 4: 在锁外分配处于 InLookup 状态的候选 Dentry，并尝试将其发布到索引中抢占单飞加载权。
        let candidate = DentryRef::try_child(parent, name)?;
        let mut loader = LoadGuard {
            cache: self,
            dentry: &candidate,
            active: false,
        };
        let existing = match self.publish(&candidate, &mut loader) {
            Err(FsError::NoMemory) => {
                // 候选分配期间其他并发查找可能占满槽位；执行一次闲置回收后再次尝试发布。
                self.prune_unused();
                self.publish(&candidate, &mut loader)?
            }
            result => result?,
        };
        // 若并发任务已抢先发布了同一名字的 Dentry，则放弃本地候选并等待对方加载完成。
        if let Some(dentry) = existing {
            drop(loader);
            drop(candidate);
            dentry.inode()?;
            return Ok(dentry);
        }

        #[cfg(feature = "ktest")]
        {
            self.loads.fetch_add(1, Ordering::Relaxed);
            let hook = self.lookup_hook.lock().clone();
            if let Some(hook) = hook {
                hook(&candidate);
            }
        }

        // Step 5: 当前任务成为单飞加载胜出者，在索引自旋锁外执行底层目录查找，并将结果写入 Once 唤醒等待队列。
        let result = directory
            .lookup(inodes, name)
            .and_then(|inode| inode.ok_or(FsError::NotFound));
        loader.complete(result);
        drop(loader);
        candidate.inode()?;
        Ok(candidate)
    }

    /// 扫描并摘除仅被缓存索引自身强引用（`Arc::strong_count == 1`）且已完成绑定的闲置目录项（包含正绑定与负绑定）。
    ///
    /// 在自旋锁外释放被摘除的子目录项后，其父目录项的强引用计数可能随之降为 1，因此每次成功摘除后会重新扫描，
    /// 实现从闲置叶子节点向父节点的逐层回收；单次调用最多摘除入口时刻的缓存条目数，防止并发插入导致无限循环。
    pub fn prune_unused(&self) -> usize {
        let budget = self.len();
        let mut removed = 0;
        let mut position = budget;
        // 摘除条目会重排向量尾部元素，且子项在锁外析构后可能使其父项也变为仅被缓存引用；
        // 因此每次成功摘除后重置扫描位置，但总摘除次数严格受限于初始长度 `budget`。
        while 0 < position && removed < budget {
            position -= 1;
            let victim = {
                let mut index = self.entries.lock();
                let unused = index.entries.get(position).is_some_and(|dentry| {
                    Arc::strong_count(&dentry.0) == 1
                        && matches!(dentry.0.binding.get(), Some(Ok(_)))
                });
                if unused {
                    let victim = index.entries.swap_remove(position);
                    victim.set_hashed(false);
                    Some(victim)
                } else {
                    None
                }
            };
            if victim.is_some() {
                removed += 1;
                position = self.len();
            }
            drop(victim);
        }
        removed
    }

    /// 返回当前索引中已正式发布的目录项数量（不含 `PreparedDentry` 预留但尚未发布的槽位）。
    pub fn len(&self) -> usize {
        self.entries.lock().entries.len()
    }

    /// 判断当前目录项缓存中是否没有任何已发布的目录项。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "ktest 在静止名字回收后核对缓存已空并重新加载")
    )]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 在调用方已持有父目录写锁的前提下，直接查询 `(parent, name)` 在缓存中的当前条目，避免重复获取父目录读锁导致死锁。
    pub(crate) fn cached_locked(&self, parent: &DentryRef, name: &[u8]) -> Option<DentryRef> {
        let index = self.entries.lock();
        find(&index.entries, parent, name)
    }

    /// 在缓存索引自旋锁临界区内二次检查 `(parent, name)` 是否已存在及剩余容量，
    /// 若未命中且容量充足，则激活 `loader` 守卫并将处于 `InLookup` 状态的 `candidate` 发布到索引中。
    fn publish(
        &self,
        candidate: &DentryRef,
        loader: &mut LoadGuard<'_>,
    ) -> Result<Option<DentryRef>, FsError> {
        let mut index = self.entries.lock();
        let parent = candidate.parent().expect("named dentry has no parent");
        if let Some(dentry) = find(&index.entries, &parent, candidate.name().as_slice()) {
            return Ok(Some(dentry));
        }
        if self.capacity <= index.entries.len() + index.reserved {
            return Err(FsError::NoMemory);
        }

        loader.active = true;
        candidate.set_hashed(true);
        index.entries.push(candidate.clone());
        Ok(None)
    }

    /// 当单飞加载因底层 I/O 或格式错误（非 `NotFound`）失败时，按指针身份从缓存索引中摘除该失败的 `Dentry` 实例，
    /// 并在锁外释放引用，使后续对同一名字的查找能够重新发起加载，同时不影响已在旧实例上等待的任务读取错误结果。
    fn remove_failed(&self, dentry: &DentryRef) {
        let victim = {
            let mut index = self.entries.lock();
            index
                .entries
                .iter()
                .position(|entry| entry.ptr_eq(dentry))
                .map(|position| {
                    let victim = index.entries.swap_remove(position);
                    victim.set_hashed(false);
                    victim
                })
        };
        drop(victim);
    }

    /// 在自旋锁外构造子目录项候选，并在缓存索引中为其预留一个槽位（或标记待替换的旧负绑定条目）；
    /// 若缓存已满则执行一次闲置回收后重试。
    pub(crate) fn try_prepare_child(
        &self,
        parent: &DentryRef,
        name: Arc<Vec<u8>>,
    ) -> Result<PreparedDentry<'_>, FsError> {
        validate_name(name.as_slice())?;
        let candidate = DentryRef::try_named_child(parent, name)?;
        let (replaced, reserved) = match self.reserve_candidate(&candidate) {
            Err(FsError::NoMemory) => {
                self.prune_unused();
                self.reserve_candidate(&candidate)?
            }
            result => result?,
        };
        Ok(PreparedDentry {
            cache: self,
            dentry: Some(candidate),
            replaced,
            reserved,
        })
    }

    /// 在自旋锁外构造子目录项、预留缓存槽位并立即绑定给定的 `inode`，供硬链接（`link`）或已创建好 Inode 的目录操作使用。
    pub(crate) fn try_prepare_positive(
        &self,
        parent: &DentryRef,
        name: Arc<Vec<u8>>,
        inode: InodeRef,
    ) -> Result<PreparedPositiveDentry<'_>, FsError> {
        Ok(self.try_prepare_child(parent, name)?.bind_positive(inode))
    }

    /// 在缓存索引自旋锁内检查目标 `(parent, name)` 是否可用并预留槽位：
    /// 若缓存中已存在负绑定（`Binding::Negative`），则记录该负绑定条目以便在正式发布时原位替换（无需额外消耗 `reserved` 配额）；
    /// 若已存在正绑定或正在加载的条目则返回 `InvalidOperation`。
    fn reserve_candidate(
        &self,
        candidate: &DentryRef,
    ) -> Result<(Option<DentryRef>, bool), FsError> {
        let mut index = self.entries.lock();
        let parent = candidate.parent().expect("named dentry has no parent");
        if let Some(existing) = find(&index.entries, &parent, candidate.name().as_slice()) {
            if matches!(existing.0.binding.get(), Some(Ok(Binding::Negative))) {
                return Ok((Some(existing), false));
            }
            return Err(FsError::InvalidOperation);
        }
        if self.capacity <= index.entries.len() + index.reserved {
            return Err(FsError::NoMemory);
        }
        index.reserved += 1;
        Ok((None, true))
    }

    /// 在底层目录项删除（`unlink` / `rmdir`）成功后，将目标 `dentry` 从缓存索引中摘除并标记为未哈希（`hashed = false`）。
    /// 已打开该路径的 `Path` 句柄仍可继续持有原 `DentryRef` 及其绑定的 `Inode`。
    pub(crate) fn unhash(&self, dentry: &DentryRef) {
        let victim = {
            let mut index = self.entries.lock();
            index
                .entries
                .iter()
                .position(|entry| entry.ptr_eq(dentry))
                .map(|position| {
                    let victim = index.entries.swap_remove(position);
                    victim.set_hashed(false);
                    victim
                })
        };
        drop(victim);
    }

    /// 在底层重命名（`rename`）成功后，原子更新缓存索引中的源条目与被覆盖的目标条目：
    /// 若存在被覆盖的 `target`，将其从索引中摘除；同时更新 `source` 的父目录引用与名字快照，并将旧引用移出索引自旋锁外释放。
    pub(crate) fn move_entry(
        &self,
        source: &DentryRef,
        target: Option<&DentryRef>,
        parent: &DentryRef,
        name: Arc<Vec<u8>>,
    ) {
        if target.is_some_and(|entry| entry.ptr_eq(source)) {
            return;
        }
        let (old_source, old_target, old_topology) = {
            // Step 1: 定位并摘除被覆盖的目标目录项（若存在且与源条目不是同一实例）。
            let mut index = self.entries.lock();
            let target_position = target
                .and_then(|target| index.entries.iter().position(|entry| entry.ptr_eq(target)))
                .or_else(|| {
                    find(&index.entries, parent, name.as_slice()).and_then(|target| {
                        index.entries.iter().position(|entry| entry.ptr_eq(&target))
                    })
                })
                .filter(|position| !index.entries[*position].ptr_eq(source));
            let old_target = target_position.map(|position| {
                let victim = index.entries.swap_remove(position);
                victim.set_hashed(false);
                victim
            });
            // Step 2: 更新源目录项在索引中的槽位及其内部拓扑（新父节点与新名字），将旧引用返回至锁外析构。
            let source_position = index
                .entries
                .iter()
                .position(|entry| entry.ptr_eq(source))
                .expect("renamed source is absent from its dentry index");
            let moved = DentryRef(source.0.clone());
            let old_source = mem::replace(&mut index.entries[source_position], moved);
            let old_topology = source.replace_topology(Some(parent.clone()), name, true);
            (old_source, old_target, old_topology)
        };

        drop(old_source);
        drop(old_target);
        drop(old_topology);
    }

    /// 设置内核测试专用的查找同步钩子，旧钩子移出锁外释放。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_lookup_hook(&self, hook: Option<LookupHook>) {
        let old = mem::replace(&mut *self.lookup_hook.lock(), hook);
        drop(old);
    }

    /// 返回自缓存创建以来单飞加载胜出并实际执行底层目录查找的累计次数（不含缓存命中与竞争失败的候选）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_load_count(&self) -> usize {
        self.loads.load(Ordering::Relaxed)
    }

    /// 查询指定 `(parent, name)` 缓存条目上当前挂起等待的测试任务数量。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_waiting(&self, parent: &DentryRef, name: &[u8]) -> usize {
        let index = self.entries.lock();
        let dentry = find(&index.entries, parent, name);
        dentry.map_or(0, |dentry| dentry.0.waiters.test_members())
    }
}

impl DentryRef {
    /// 基于目录类型的 `inode` 构造无父节点、空名字且已完成正绑定的挂载点根 `DentryRef`（不加入 `DentryCache` 索引）。
    pub(crate) fn try_root(inode: InodeRef) -> Result<Self, FsError> {
        if inode.metadata_snapshot()?.kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        let name = try_name(&[])?;
        let dentry = Arc::try_new(Dentry {
            topology: SpinLock::new(DentryTopology {
                parent: None,
                name: name.clone(),
                hashed: false,
            }),
            binding: Once::new(),
            waiters: WaitQueue::new(),
            _pinned: PhantomPinned,
        })
        .map_err(|_| FsError::NoMemory)?;
        dentry.binding.call_once(|| Ok(Binding::Positive(inode)));
        Ok(Self(dentry))
    }

    /// 为硬链接计数为 0 的匿名 `inode` 构造无父节点、空名字且未加入缓存索引的正绑定 `DentryRef`（仅供内核测试使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn try_unhashed(inode: InodeRef) -> Result<Self, FsError> {
        if inode.metadata_snapshot()?.links != 0 {
            return Err(FsError::InvalidOperation);
        }
        let name = try_name(&[])?;
        let dentry = Arc::try_new(Dentry {
            topology: SpinLock::new(DentryTopology {
                parent: None,
                name: name.clone(),
                hashed: false,
            }),
            binding: Once::new(),
            waiters: WaitQueue::new(),
            _pinned: PhantomPinned,
        })
        .map_err(|_| FsError::NoMemory)?;
        dentry.binding.call_once(|| Ok(Binding::Positive(inode)));
        Ok(Self(dentry))
    }

    /// 在自旋锁外分配名字字节快照并构造处于未完成加载（`InLookup`）状态的子目录项。
    fn try_child(parent: &Self, name: &[u8]) -> Result<Self, FsError> {
        Self::try_named_child(parent, try_name(name)?)
    }

    /// 使用已分配好的名字 `Arc<Vec<u8>>` 构造处于未完成加载（`InLookup`）状态且尚未加入索引的子目录项。
    fn try_named_child(parent: &Self, name: Arc<Vec<u8>>) -> Result<Self, FsError> {
        let dentry = Arc::try_new(Dentry {
            topology: SpinLock::new(DentryTopology {
                parent: Some(parent.clone()),
                name: name.clone(),
                hashed: false,
            }),
            binding: Once::new(),
            waiters: WaitQueue::new(),
            _pinned: PhantomPinned,
        })
        .map_err(|_| FsError::NoMemory)?;
        Ok(Self(dentry))
    }

    /// 获取当前目录项的父目录项引用（挂载根或未关联父节点的目录项返回 `None`）。
    pub fn parent(&self) -> Option<DentryRef> {
        self.0.topology.lock().parent.clone()
    }

    /// 获取当前目录项原始字节名字的 `Arc` 共享快照（不复制底层字节）。
    pub fn name(&self) -> Arc<Vec<u8>> {
        self.0.topology.lock().name.clone()
    }

    /// 在单次 `topology` 自旋锁临界区内原子获取 `(parent, name)` 快照，确保路径反向拼接不会读到并发 `rename` 前后的撕裂状态。
    pub(crate) fn topology_snapshot(&self) -> (Option<DentryRef>, Arc<Vec<u8>>) {
        let topology = self.0.topology.lock();
        (topology.parent.clone(), topology.name.clone())
    }

    /// 按底层 `Arc<Dentry>` 指针地址判断两个 `DentryRef` 是否指向同一个目录项实例（指向同一 Inode 的不同硬链接返回 `false`）。
    pub fn ptr_eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    /// 将强引用降级为 `Weak<Dentry>` 弱引用，供文件系统根 Inode 弱登记其唯一根目录项而不产生循环强引用。
    pub(crate) fn downgrade(&self) -> Weak<Dentry> {
        Arc::downgrade(&self.0)
    }

    /// 尝试将 `Weak<Dentry>` 弱引用升级为强引用 `DentryRef`（若底层目录项已释放则返回 `None`）。
    pub(crate) fn upgrade(dentry: &Weak<Dentry>) -> Option<Self> {
        dentry.upgrade().map(Self)
    }

    /// 判断当前目录项是否仍连接在 `DentryCache` 活动索引中（未被 `unhash` 或淘汰）。
    pub(crate) fn is_hashed(&self) -> bool {
        self.0.topology.lock().hashed
    }

    /// 更新目录项的索引连接标志 `hashed`（调用方已持有 `DentryCache::entries` 索引自旋锁，满足锁顺序契约）。
    fn set_hashed(&self, hashed: bool) {
        self.0.topology.lock().hashed = hashed;
    }

    /// 在 `topology` 自旋锁临界区内一次性替换父节点、名字与 `hashed` 状态，并将被替换的旧拓扑字段返回至锁外析构。
    fn replace_topology(
        &self,
        parent: Option<DentryRef>,
        name: Arc<Vec<u8>>,
        hashed: bool,
    ) -> DentryTopology {
        let mut topology = self.0.topology.lock();
        DentryTopology {
            parent: mem::replace(&mut topology.parent, parent),
            name: mem::replace(&mut topology.name, name),
            hashed: mem::replace(&mut topology.hashed, hashed),
        }
    }

    /// 等待当前目录项的首次单飞加载完成，并返回其正绑定的 `&InodeRef`；
    /// 若结果为负绑定则返回 `FsError::NotFound`，若加载失败则返回对应的 `FsError`。
    ///
    /// # 调用上下文
    /// 必须在普通可睡眠任务上下文中调用。若目录项尚处于 `InLookup` 状态，当前任务将挂起在 `self.0.waiters` 等待队列上。
    pub fn inode(&self) -> Result<&InodeRef, FsError> {
        sched::assert_switchable();
        let dentry = self.0.pinned().get_ref();
        loop {
            if let Some(result) = dentry.binding.get() {
                return binding_inode(result);
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `&self` 在整个函数调用期间持有 `Arc<Dentry>`，结合 `Dentry::pinned` 保证内嵌等待队列 `dentry.waiters` 在堆上地址稳定且不会被释放；
            // 2. `node` 通过 `pin!` 宏固定在当前函数栈帧上，不会在挂起期间移动；
            // 3. `_registration` 守卫在每次循环迭代结束前（栈帧弹出前）正常析构，自动将 `node` 从 `dentry.waiters` 链表中摘除。
            let _registration = unsafe { dentry.waiters.register(node.as_mut()) };
            if let Some(result) = dentry.binding.get() {
                return binding_inode(result);
            }
            waiter.park();
        }
    }
}

impl<'cache> PreparedDentry<'cache> {
    /// 为尚未加入缓存索引的预留候选目录项填入正绑定 `inode`，并转换为可提交发布的 `PreparedPositiveDentry`。
    pub(crate) fn bind_positive(self, inode: InodeRef) -> PreparedPositiveDentry<'cache> {
        self.dentry
            .as_ref()
            .expect("prepared dentry was consumed")
            .0
            .binding
            .call_once(|| Ok(Binding::Positive(inode)));
        PreparedPositiveDentry(self)
    }
}

impl PreparedPositiveDentry<'_> {
    /// 借用已完成正绑定但尚未发布到索引的 `DentryRef`，供调用方在最终发布前构造 `Path`。
    pub(crate) fn dentry(&self) -> &DentryRef {
        self.0
            .dentry
            .as_ref()
            .expect("prepared dentry was consumed")
    }

    /// 消费预留槽位并将已完成正绑定的目录项正式发布到 `DentryCache` 索引中（全程零内存分配）。
    pub(crate) fn publish(self) {
        self.0.publish_inner();
    }
}

impl PreparedDentry<'_> {
    /// 执行预留目录项的索引发布：在索引自旋锁内摘除被覆盖的旧负绑定条目（若存在）、扣减 `reserved` 计数并将新条目推入索引，最后在锁外释放旧条目。
    fn publish_inner(mut self) {
        let dentry = self.dentry.take().expect("prepared dentry was consumed");
        let replaced = {
            let mut index = self.cache.entries.lock();
            let replaced = self.replaced.take().map(|entry| {
                let position = index
                    .entries
                    .iter()
                    .position(|cached| cached.ptr_eq(&entry))
                    .expect("reserved negative dentry disappeared");
                let victim = index.entries.swap_remove(position);
                victim.set_hashed(false);
                victim
            });
            if self.reserved {
                index.reserved -= 1;
                self.reserved = false;
            }
            dentry.set_hashed(true);
            index.entries.push(dentry);
            replaced
        };
        drop(replaced);
    }
}

impl Drop for PreparedDentry<'_> {
    /// 当预留目录项未调用 `publish` 即退出作用域时，在索引自旋锁内归还所占用的 `reserved` 槽位配额。
    fn drop(&mut self) {
        if self.reserved {
            let mut index = self.cache.entries.lock();
            index.reserved -= 1;
            self.reserved = false;
        }
    }
}

/// 在自旋锁外按组件精确长度分配字节向量并封装为 `Arc<Vec<u8>>`，分配失败时返回 `FsError::NoMemory`。
fn try_name(name: &[u8]) -> Result<Arc<Vec<u8>>, FsError> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(name.len())
        .map_err(|_| FsError::NoMemory)?;
    bytes.extend_from_slice(name);
    Arc::try_new(bytes).map_err(|_| FsError::NoMemory)
}

impl Dentry {
    /// 获取固定在 `Arc` 堆分配中的 `Pin<&Dentry>` 共享借用，供侵入式等待队列注册使用。
    fn pinned(&self) -> Pin<&Self> {
        // SAFETY:
        // 1. `Dentry` 标记了 `PhantomPinned`，且仅在模块内部通过 `Arc::try_new` 在堆上构造；
        // 2. 本模块从不调用 `Arc::get_mut`、`Arc::try_unwrap` 或 `mem::replace` 移动 `Dentry` 本体，`DentryRef` 亦不向外暴露可变引用，因此 `Dentry` 在整个生命周期内内存地址恒定不变。
        unsafe { Pin::new_unchecked(self) }
    }
}

impl LoadGuard<'_> {
    /// 将底层目录查找结果转换为 `Binding` 状态并发布：目录返回 `NotFound` 时转为负绑定（`Binding::Negative`）保留在缓存中，
    /// 其余错误将从缓存索引中摘除占位条目并唤醒所有等待任务。
    fn complete(&mut self, result: Result<InodeRef, FsError>) {
        let binding = match result {
            Ok(inode) => Ok(Binding::Positive(inode)),
            Err(FsError::NotFound) => Ok(Binding::Negative),
            Err(error) => Err(error),
        };
        self.publish(binding);
        self.active = false;
    }

    /// 通过 `Once::call_once` 写入单飞加载的最终结果；若结果为错误则先从 `DentryCache` 索引中摘除自身，随后唤醒 `waiters` 上的全部等待者。
    fn publish(&self, binding: Result<Binding, FsError>) {
        let result = self.dentry.0.binding.call_once(|| binding);
        if result.is_err() {
            self.cache.remove_failed(self.dentry);
        }
        self.dentry.0.waiters.wake_all();
    }
}

impl Drop for LoadGuard<'_> {
    /// 若单飞加载任务在调用 `complete` 前异常退出，将绑定状态设为 `FsError::LoadAborted`、摘除索引占位并唤醒等待队列，防止等待者永久死锁。
    fn drop(&mut self) {
        if self.active {
            self.publish(Err(FsError::LoadAborted));
        }
    }
}

/// 在调用方已持有 `DentryCache::entries` 索引自旋锁的前提下，线性扫描查找父节点指针与名字字节均匹配的已哈希目录项。
fn find(entries: &[DentryRef], parent: &DentryRef, name: &[u8]) -> Option<DentryRef> {
    entries
        .iter()
        .find(|dentry| {
            let topology = dentry.0.topology.lock();
            topology.hashed
                && topology.name.as_slice() == name
                && topology
                    .parent
                    .as_ref()
                    .is_some_and(|entry| entry.ptr_eq(parent))
        })
        .cloned()
}

/// 校验单个路径组件名字是否符合文件系统规范（非空、长度不超过 `NAME_MAX`、且不包含 `\0` 或路径分隔符 `/`）。
fn validate_name(name: &[u8]) -> Result<(), FsError> {
    if crate::fs::types::NAME_MAX < name.len() {
        return Err(FsError::NameTooLong);
    }
    if name.is_empty() || name.contains(&0) || name.contains(&b'/') {
        return Err(FsError::InvalidName);
    }
    Ok(())
}

/// 将 `Once` 中保存的绑定结果转换为 `&InodeRef`：正绑定返回 Inode 引用，负绑定映射为 `FsError::NotFound`，加载错误原样返回。
fn binding_inode(binding: &Result<Binding, FsError>) -> Result<&InodeRef, FsError> {
    match binding {
        Ok(Binding::Positive(inode)) => Ok(inode),
        Ok(Binding::Negative) => Err(FsError::NotFound),
        Err(error) => Err(*error),
    }
}
