//! 原始组件按父 dentry 身份缓存；名字加载只发表一次，目录 I/O 不持索引锁。

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

#[cfg(feature = "ktest")]
type LookupHook = Arc<dyn Fn(&DentryRef) + Send + Sync>;

/// 有限索引强持名字对象，对象只强持父与 inode，不反持缓存域。
/// ponytail: 固定预算内线性查找，预算增大且查找成为瓶颈时改为有限散列表。
pub struct DentryCache {
    entries: SpinLock<DentryIndex>,
    capacity: usize,
    #[cfg(feature = "ktest")]
    lookup_hook: SpinLock<Option<LookupHook>>,
    #[cfg(feature = "ktest")]
    loads: AtomicUsize,
}

struct DentryIndex {
    entries: Vec<DentryRef>,
    reserved: usize,
}

/// 共享名字对象身份，当前原始名字由核心拓扑持有Arc快照。
#[derive(Clone)]
pub struct DentryRef(Arc<Dentry>);

/// 固定名字对象可由 canonical inode 槽弱登记为卷根。
pub(crate) struct Dentry {
    topology: SpinLock<DentryTopology>,
    // 尚无结果时处于 InLookup；完成后只保留正绑定、负绑定或首次具体错误。
    binding: Once<Result<Binding, FsError>>,
    waiters: WaitQueue,
    _pinned: PhantomPinned,
}

struct DentryTopology {
    parent: Option<DentryRef>,
    name: Arc<Vec<u8>>,
    hashed: bool,
}

enum Binding {
    Positive(InodeRef),
    Negative,
}

/// 预留一个不可见的名字索引槽，未发表拥有在析构时自动归还。
pub(crate) struct PreparedDentry<'a> {
    cache: &'a DentryCache,
    dentry: Option<DentryRef>,
    replaced: Option<DentryRef>,
    reserved: bool,
}
/// 正绑定已完成；只有此状态允许向名字索引发表候选。
pub(crate) struct PreparedPositiveDentry<'a>(PreparedDentry<'a>);

/// 胜出者在发表 InLookup 前承担责任，退出必须结束原对象并撤自己的槽。
struct LoadGuard<'a> {
    cache: &'a DentryCache,
    dentry: &'a DentryRef,
    active: bool,
}

impl DentryCache {
    /// 锁外预留全部有限槽位并构造共享候选，后续发表不在索引锁内分配。
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

    /// 返回共享名字索引实际保留的固定预算。
    pub(crate) fn capacity(&self) -> usize {
        self.capacity
    }

    /// 普通可切换 Task 查单组件；读 guard 覆盖缓存命中、首次加载和等待。
    pub fn lookup(
        &self,
        inodes: &InodeCache,
        parent: &DentryRef,
        name: &[u8],
    ) -> Result<DentryRef, FsError> {
        sched::assert_switchable();
        let inode = parent.inode()?;
        let directory = inode.directory_read()?;
        validate_name(name)?;
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
        // 静止磁盘正绑定仍强持 inode；RAM 权威 inode 不占磁盘缓存预算。
        if full || (!inode.is_memory() && inodes.is_full()) {
            self.prune_unused();
            // 回收期间同 key 可能已发表；预算满也必须先消费现有对象。
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
        let candidate = DentryRef::try_child(parent, name)?;
        let mut loader = LoadGuard {
            cache: self,
            dentry: &candidate,
            active: false,
        };
        let existing = match self.publish(&candidate, &mut loader) {
            Err(FsError::NoMemory) => {
                // 候选准备期间其他 key 可以占满预算；只有限回收一次并再次双查。
                self.prune_unused();
                self.publish(&candidate, &mut loader)?
            }
            result => result?,
        };
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
        let result = directory
            .lookup(inodes, name)
            .and_then(|inode| inode.ok_or(FsError::NotFound));
        loader.complete(result);
        drop(loader);
        candidate.inode()?;
        Ok(candidate)
    }

    /// 有限摘除 cache-only 的静止正负绑定，锁外释放后重扫使静止子到父逐层收尽。
    pub fn prune_unused(&self) -> usize {
        let budget = self.len();
        let mut removed = 0;
        let mut position = budget;
        // 摘除会重排索引，且子析构可让父变为 cache-only；每次成功后重扫。
        // 摘除数不超过入口槽数，并发新发表不会把本次回收变成无界循环。
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

    /// 返回有限索引中已发表名字的数量，不包含尚未提交的槽位准备。
    pub fn len(&self) -> usize {
        self.entries.lock().entries.len()
    }

    /// 观察缓存域当前是否没有已发表名字。
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// 父目录写 guard 已持有时读取缓存项，避免在最终组件重查中递归取读锁。
    pub(crate) fn cached_locked(&self, parent: &DentryRef, name: &[u8]) -> Option<DentryRef> {
        let index = self.entries.lock();
        find(&index.entries, parent, name)
    }

    /// 同短锁内双查身份与预算，加载责任先于 InLookup 的索引引用发表。
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

    /// 只摘失败的原实例，旧等待者仍持原结果，后续同 key 的新实例不受影响。
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

    /// 锁外准备正名字对象并占用一个不可见索引槽，失败只做一次有限回收。
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

    /// 以已知 inode 完成正绑定，供无需新 inode 的名字操作复用预留候选。
    pub(crate) fn try_prepare_positive(
        &self,
        parent: &DentryRef,
        name: Arc<Vec<u8>>,
        inode: InodeRef,
    ) -> Result<PreparedPositiveDentry<'_>, FsError> {
        Ok(self.try_prepare_child(parent, name)?.bind_positive(inode))
    }

    /// 在索引锁内给新名字保留槽；旧负项只在真实发表时替换。
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

    /// 后端删除成功后解除缓存索引；旧Path继续持有原绑定与名字拓扑。
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

    /// 后端rename成功后移动原对象并解除覆盖项；名字与父拥有在索引锁外释放。
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

    /// 在 InLookup 发表后、真实目录查找前执行锁外停点，替换后的旧回调锁外析构。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_lookup_hook(&self, hook: Option<LookupHook>) {
        let old = mem::replace(&mut *self.lookup_hook.lock(), hook);
        drop(old);
    }

    /// 返回胜出并取得名字加载责任的次数，不计命中与未发表的竞争失败候选。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_load_count(&self) -> usize {
        self.loads.load(Ordering::Relaxed)
    }

    /// 借用真实索引对象，读取它自己的等待队列成员数。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_waiting(&self, parent: &DentryRef, name: &[u8]) -> usize {
        let index = self.entries.lock();
        let dentry = find(&index.entries, parent, name);
        dentry.map_or(0, |dentry| dentry.0.waiters.test_members())
    }
}

impl DentryRef {
    /// 以真实正目录建立无父、空名字且已完成的挂载根，不参与名字索引。
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

    /// 为零链接 inode 建立无父、空名字的正绑定；不登记名字或修改目录。
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

    /// 锁外准备候选名字和稳定 Arc；失败不会占槽或发表等待源。
    fn try_child(parent: &Self, name: &[u8]) -> Result<Self, FsError> {
        Self::try_named_child(parent, try_name(name)?)
    }

    /// 以已准备的名字建立未完成加载对象，不在索引中直接发表。
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

    /// 读取当前父关系；Arc克隆不改变名字对象身份或复制原始字节。
    pub fn parent(&self) -> Option<DentryRef> {
        self.0.topology.lock().parent.clone()
    }

    /// 返回当前不可变原始名字的Arc快照，不为每次查询分配或复制字节。
    pub fn name(&self) -> Arc<Vec<u8>> {
        self.0.topology.lock().name.clone()
    }

    /// 同一拓扑锁内取得父和原始名字，路径导出不能拼接rename前后的两半状态。
    pub(crate) fn topology_snapshot(&self) -> (Option<DentryRef>, Arc<Vec<u8>>) {
        let topology = self.0.topology.lock();
        (topology.parent.clone(), topology.name.clone())
    }

    /// 比较名字对象实例身份，硬链接的同 inode 不代表同 dentry。
    pub fn ptr_eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
    /// 弱登记稳定名字对象，不延长根 dentry 或其 inode 的生命期。
    pub(crate) fn downgrade(&self) -> Weak<Dentry> {
        Arc::downgrade(&self.0)
    }

    /// 将仍存活的 canonical 根升级为拥有引用。
    pub(crate) fn upgrade(dentry: &Weak<Dentry>) -> Option<Self> {
        dentry.upgrade().map(Self)
    }

    /// 读取当前拓扑连接状态，缓存项仅在名字空间真实连接时为真。
    pub(crate) fn is_hashed(&self) -> bool {
        self.0.topology.lock().hashed
    }

    /// 锁内修改哈希状态，调用者已经持有 dcache 索引短锁。
    fn set_hashed(&self, hashed: bool) {
        self.0.topology.lock().hashed = hashed;
    }

    /// 锁内一次替换父、名字和连接状态，旧拥有返回给锁外析构。
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

    /// 锁外等待首次结果并借用唯一 inode；负绑定与真实失败保留各自错误。
    pub fn inode(&self) -> Result<&InodeRef, FsError> {
        sched::assert_switchable();
        let dentry = self.0.pinned().get_ref();
        loop {
            if let Some(result) = dentry.binding.get() {
                return binding_inode(result);
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 私有 Arc 在本借用期间保活固定 dentry 和内嵌等待源；pin! 固定栈节点，
            // registration 不被遗忘，离开本轮先撤链，再释放节点、waiter 和源借用。
            let _registration = unsafe { dentry.waiters.register(node.as_mut()) };
            if let Some(result) = dentry.binding.get() {
                return binding_inode(result);
            }
            waiter.park();
        }
    }
}
impl<'cache> PreparedDentry<'cache> {
    /// 完成未入索引候选的正绑定，并转换为仅能成功发表的状态。
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
    /// 借用已绑定但未入索引的名字对象，供后端发表前构造 Path。
    pub(crate) fn dentry(&self) -> &DentryRef {
        self.0
            .dentry
            .as_ref()
            .expect("prepared dentry was consumed")
    }

    /// 消费预留槽并发表正绑定，提交只移动已准备的引用且不分配。
    pub(crate) fn publish(self) {
        self.0.publish_inner();
    }
}

impl PreparedDentry<'_> {
    /// 消费完整正绑定并发表，索引槽已预留且无需分配。
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
    /// 未发表候选退出时只归还索引预算，复杂引用在短锁外释放。
    fn drop(&mut self) {
        if self.reserved {
            let mut index = self.cache.entries.lock();
            index.reserved -= 1;
            self.reserved = false;
        }
    }
}

/// 锁外一次分配原始字节及其共享快照。
fn try_name(name: &[u8]) -> Result<Arc<Vec<u8>>, FsError> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(name.len())
        .map_err(|_| FsError::NoMemory)?;
    bytes.extend_from_slice(name);
    Arc::try_new(bytes).map_err(|_| FsError::NoMemory)
}

impl Dentry {
    /// 私有稳定拥有类型只给共享借用，固定等待源不能从原 Arc 分配中移出。
    fn pinned(&self) -> Pin<&Self> {
        // SAFETY: 对象仅经私有构造置于 Arc 后发表；本模块不 unwrap、替换或移动对象，
        // DentryRef 不暴露 Arc 或可变借用，本共享借用保证原分配存活。
        unsafe { Pin::new_unchecked(self) }
    }
}

impl LoadGuard<'_> {
    /// 真实查找结束后只将 NotFound 变为负绑定，其他错误解除占位并唤醒等待者。
    fn complete(&mut self, result: Result<InodeRef, FsError>) {
        let binding = match result {
            Ok(inode) => Ok(Binding::Positive(inode)),
            Err(FsError::NotFound) => Ok(Binding::Negative),
            Err(error) => Err(error),
        };
        self.publish(binding);
        self.active = false;
    }

    /// 结果只发表一次，失败摘原槽后锁外通知，旧等待者不改查新的加载实例。
    fn publish(&self, binding: Result<Binding, FsError>) {
        let result = self.dentry.0.binding.call_once(|| binding);
        if result.is_err() {
            self.cache.remove_failed(self.dentry);
        }
        self.dentry.0.waiters.wake_all();
    }
}

impl Drop for LoadGuard<'_> {
    /// 已发表但未完成的加载者退出时结束原对象，避免遗留永久 InLookup。
    fn drop(&mut self) {
        if self.active {
            self.publish(Err(FsError::LoadAborted));
        }
    }
}

/// 索引锁内读取当前连接键；拓扑只按索引锁到单 dentry 锁的方向变更。
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

/// 单组件沿盘上名字长度约束验证，点组件的路径含义由逐组件遍历处理。
fn validate_name(name: &[u8]) -> Result<(), FsError> {
    if crate::fs::types::NAME_MAX < name.len() {
        return Err(FsError::NameTooLong);
    }
    if name.is_empty() || name.contains(&0) || name.contains(&b'/') {
        return Err(FsError::InvalidName);
    }
    Ok(())
}

/// 从已完成的一次绑定借用 inode，不复制对象或将失败误当负绑定。
fn binding_inode(binding: &Result<Binding, FsError>) -> Result<&InodeRef, FsError> {
    match binding {
        Ok(Binding::Positive(inode)) => Ok(inode),
        Ok(Binding::Negative) => Err(FsError::NotFound),
        Err(error) => Err(*error),
    }
}
