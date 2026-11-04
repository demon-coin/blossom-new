//! 具体事件 OFD 使用弱目标、稳定 interest 和独立 ready/overflow 候选链。

use alloc::sync::{Arc, Weak};
use core::cell::UnsafeCell;
use core::ops::Range;
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use crate::fs::file::{
    set_nonblocking, set_status_flags, FileIoContext, FileLike, FileRef, FileStat, ReadBuffer,
    SeekFrom, WriteBuffer,
};
use crate::fs::poll::{PollMask, PollNotify, PollTable, PollWaitHead, Pollable};
use crate::fs::{DirectoryEntry, FsError, InodeKind, InodeRef, InodeTime, Path, O_CLOEXEC, O_RDWR};
use crate::sync::{SleepMutex, SleepMutexGuard, SpinLock};

mod index;

pub const EPOLL_CTL_ADD: u32 = 1;
pub const EPOLL_CTL_DEL: u32 = 2;
pub const EPOLL_CTL_MOD: u32 = 3;
pub const EPOLLONESHOT: u32 = 1 << 30;
pub const EPOLLET: u32 = 1 << 31;
const RETIRE_BATCH: usize = 16;
const PROVIDER_SOURCES: usize = 2;
const ORDINARY_EVENTS: u32 = PollMask::POLLIN.0
    | PollMask::POLLPRI.0
    | PollMask::POLLOUT.0
    | PollMask::POLLERR.0
    | PollMask::POLLHUP.0
    | PollMask::POLLRDNORM.0
    | PollMask::POLLRDBAND.0
    | PollMask::POLLWRNORM.0
    | PollMask::POLLWRBAND.0
    | PollMask::POLLRDHUP.0;
const ALWAYS_EVENTS: u32 = PollMask::POLLERR.0 | PollMask::POLLHUP.0;
static NEXT_INODE: AtomicU64 = AtomicU64::new(1);

/// 两个目标架构使用自然 C 布局；data 是原样保存的位模式，不是内核指针。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EpollEvent {
    pub events: u32,
    pub data: u64,
}

const _: () = assert!(core::mem::size_of::<EpollEvent>() == 16);
const _: () = assert!(core::mem::offset_of!(EpollEvent, data) == 8);

#[derive(Clone, Copy, Eq, Ord, PartialEq, PartialOrd)]
struct Key {
    object: usize,
    fd: i32,
}

impl Key {
    /// 实际 Arc payload 身份与登记 fd 共同组成键；节点的 Weak 保住地址控制尾。
    fn new(target: &FileRef, fd: i32) -> Self {
        Self {
            object: Arc::as_ptr(target) as *const () as usize,
            fd,
        }
    }
}

struct WatchNode {
    key: Key,
    owner: Weak<dyn FileLike>,
    next: SpinLock<Option<Arc<WatchNode>>>,
}

struct WatchState {
    chain: Option<Arc<WatchNode>>,
    released: bool,
}

/// 每个可持久监测 OFD 的反向链；链内只弱持事件 OFD，不延长目标业务寿命。
pub struct WatchHead {
    state: SpinLock<WatchState>,
}

impl WatchHead {
    /// 建立未释放的空反链，具体 OFD 在构造期间 fallible 准备其 Arc。
    pub const fn new() -> Self {
        Self {
            state: SpinLock::new(WatchState {
                chain: None,
                released: false,
            }),
        }
    }

    /// 目标末释放先移走反链，再在短锁外逐项取得控制锁同步撤源。
    pub fn release(&self) {
        let mut chain = {
            let mut state = self.state.lock();
            state.released = true;
            state.chain.take()
        };
        while let Some(node) = chain {
            chain = node.next.lock().take();
            // 弱身份升级与其最后 put 都在反链短锁外；控制入口返回前已结束 guard。
            if let Some(owner) = node.owner.upgrade() {
                owner
                    .epoll()
                    .expect("reverse watch owner is not epoll")
                    .release_key(node.key);
                drop(owner);
            }
        }
    }

    /// 目标由 ADD 调用者强持，末释放不能与本次无分配发表并行。
    fn attach(&self, node: &Arc<WatchNode>) {
        let mut state = self.state.lock();
        assert!(!state.released, "registering a released OFD watch head");
        *node.next.lock() = state.chain.take();
        state.chain = Some(Arc::clone(node));
    }

    /// 已同步撤源后摘除原反向票，被摘下的拥有留到短锁外结束。
    fn unlink(&self, target: &Arc<WatchNode>) {
        let detached = {
            let mut state = self.state.lock();
            if state
                .chain
                .as_ref()
                .is_some_and(|node| Arc::ptr_eq(node, target))
            {
                let removed = state.chain.take();
                state.chain = removed.as_ref().unwrap().next.lock().take();
                removed
            } else {
                let mut cursor = state.chain.as_ref().cloned();
                let mut removed = None;
                while let Some(node) = cursor {
                    let mut next = node.next.lock();
                    if next.as_ref().is_some_and(|node| Arc::ptr_eq(node, target)) {
                        removed = next.take();
                        *next = removed.as_ref().unwrap().next.lock().take();
                        break;
                    }
                    cursor = next.as_ref().cloned();
                }
                removed
            }
        };
        drop(detached);
    }
}

type EntryRef = Arc<Entry>;
type Link = Option<EntryRef>;

#[derive(Clone, Copy)]
enum ChainKind {
    Ready,
    Overflow,
}

struct Projection {
    linked: bool,
    previous: Option<Weak<Entry>>,
    next: Link,
}

struct CandidateState {
    event: EpollEvent,
    armed: bool,
    ready: Projection,
    overflow: Projection,
}

struct QueueWrite;

struct ScanProjection {
    file: Option<FileRef>,
    next: Link,
}

struct Entry {
    key: Key,
    target: Weak<dyn FileLike>,
    watches: Arc<WatchHead>,
    reverse: Arc<WatchNode>,
    core: Arc<ReadyCore>,
    index: index::Links,
    candidate: UnsafeCell<CandidateState>,
    sources: UnsafeCell<Option<PollTable>>,
    scan: UnsafeCell<ScanProjection>,
}

// SAFETY: 只移动稳定 Arc 拥有；index/sources 由所属控制互斥域独占，candidate 由
// ReadyCore 短锁访问，scan 由实例扫描互斥域独占。各投影不重叠，不取得整个 Entry 的可变引用。
unsafe impl Send for Entry {}
// SAFETY: 不可变身份、Weak 与 Arc 可共享；回调仅使用 candidate，控制操作串行
// 访问 index/sources，扫描互斥域覆盖 scan 投影的安装、查询和锁外迭代释放。
unsafe impl Sync for Entry {}

impl Entry {
    /// 源登记前准备全部稳定身份和反向票，尚未 armed 时 callback 不发表候选。
    fn prepare(
        owner: &FileRef,
        target: &FileRef,
        fd: i32,
        core: &Arc<ReadyCore>,
        watches: &Arc<WatchHead>,
    ) -> Result<EntryRef, FsError> {
        let key = Key::new(target, fd);
        let reverse = Arc::try_new(WatchNode {
            key,
            owner: Arc::downgrade(owner),
            next: SpinLock::new(None),
        })
        .map_err(|_| FsError::NoMemory)?;
        Arc::try_new(Self {
            key,
            target: Arc::downgrade(target),
            watches: Arc::clone(watches),
            reverse,
            core: Arc::clone(core),
            index: index::Links::new(),
            candidate: UnsafeCell::new(CandidateState {
                event: EpollEvent::default(),
                armed: false,
                ready: Projection {
                    linked: false,
                    previous: None,
                    next: None,
                },
                overflow: Projection {
                    linked: false,
                    previous: None,
                    next: None,
                },
            }),
            sources: UnsafeCell::new(None),
            scan: UnsafeCell::new(ScanProjection {
                file: None,
                next: None,
            }),
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 唯一候选锁域的凭证约束可变投影，不与控制索引的借用重叠。
    fn candidate_mut<'a>(&'a self, _write: &'a mut QueueWrite) -> &'a mut CandidateState {
        // SAFETY: QueueWrite 仅在 ReadyCore.state 已持有时建立，源、扫描与控制发布
        // 使用同一短锁；整个 Entry 仍由真实 Arc 保活，不创建 whole &mut Entry。
        unsafe { &mut *self.candidate.get() }
    }

    /// 两条链各用自己的前后投影，扫描期间同一节点可以同时属于两个投影。
    fn projection<'a>(&'a self, kind: ChainKind, write: &'a mut QueueWrite) -> &'a mut Projection {
        let state = self.candidate_mut(write);
        match kind {
            ChainKind::Ready => &mut state.ready,
            ChainKind::Overflow => &mut state.overflow,
        }
    }

    /// 扫描拥有链只在实例扫描互斥域访问，不与候选或索引投影重叠。
    fn scan_mut<'a>(&'a self, _write: &'a mut DeferredFiles<'_>) -> &'a mut ScanProjection {
        // SAFETY: DeferredFiles 从唯一扫描 guard 建立，直到其 Drop 迭代清链后才结束
        // guard；ctl/callback 不访问此投影，节点始终由扫描链或当前局部 Arc 保活。
        unsafe { &mut *self.scan.get() }
    }

    /// 控制域独占安装完整登记集合；集合的 sink 原票允许 callback 安全 clone 节点。
    fn install_sources(&self, sources: PollTable, _tree: &mut index::Tree) {
        // SAFETY: 调用者持所属控制 SleepMutex，节点尚未发表，callback 不访问 sources；
        // 槽只安装一次，同一控制域的 detach 必须先 take 后同步撤源。
        let slot = unsafe { &mut *self.sources.get() };
        assert!(slot.is_none(), "epoll sources installed twice");
        *slot = Some(sources);
    }

    /// 控制域取走集合；退休拥有在控制 guard 结束后同步注销全部 source。
    fn take_sources(&self, _tree: &mut index::Tree) -> Option<PollTable> {
        // SAFETY: 控制互斥域串行 ADD/DEL/末释放，回调不读 sources；返回值独占
        // 原集合，之后槽为空，所有源节点仍由返回的集合和源链保活至注销完成。
        unsafe { &mut *self.sources.get() }.take()
    }
}

impl Drop for Entry {
    /// 末节点拥有必须已撤全 source、树、候选和扫描投影，禁止遗留拥有链。
    fn drop(&mut self) {
        assert!(
            self.sources.get_mut().is_none() && self.index.detached(),
            "epoll entry not detached"
        );
        let candidate = self.candidate.get_mut();
        assert!(
            !candidate.ready.linked
                && candidate.ready.previous.is_none()
                && candidate.ready.next.is_none()
                && !candidate.overflow.linked
                && candidate.overflow.previous.is_none()
                && candidate.overflow.next.is_none(),
            "epoll entry still queued"
        );
        let scan = self.scan.get_mut();
        assert!(
            scan.file.is_none() && scan.next.is_none(),
            "epoll entry still holds scan ownership"
        );
    }
}

struct EntrySink {
    entry: EntryRef,
}

impl PollNotify for EntrySink {
    /// 源短锁内只按已发布兴趣记候选及有限通知，不升级事件或目标 OFD。
    fn notify(&self, mask: PollMask) {
        self.entry.core.notify(&self.entry, mask);
    }
}

struct Chain {
    kind: ChainKind,
    first: Link,
    last: Link,
}

impl Chain {
    /// 每条具体候选链持自己的真实首尾 Arc 票。
    const fn new(kind: ChainKind) -> Self {
        Self {
            kind,
            first: None,
            last: None,
        }
    }

    /// 同一投影只挂一次，尾插建立弱回链，不形成强引用环。
    fn push_back(&mut self, node: EntryRef, write: &mut QueueWrite) {
        let projection = node.projection(self.kind, write);
        assert!(
            !projection.linked && projection.previous.is_none() && projection.next.is_none(),
            "epoll candidate already linked"
        );
        projection.linked = true;
        projection.previous = self.last.as_ref().map(Arc::downgrade);
        if let Some(last) = &self.last {
            last.projection(self.kind, write).next = Some(Arc::clone(&node));
        } else {
            self.first = Some(Arc::clone(&node));
        }
        self.last = Some(node);
    }

    /// 未交付项重新放回扫描首部，旧首部的回链改指新节点。
    fn push_front(&mut self, node: EntryRef, write: &mut QueueWrite) {
        let projection = node.projection(self.kind, write);
        assert!(
            !projection.linked && projection.previous.is_none() && projection.next.is_none(),
            "epoll candidate already linked"
        );
        projection.linked = true;
        projection.next = self.first.take();
        let next = projection.next.as_ref().cloned();
        if let Some(next) = next {
            next.projection(self.kind, write).previous = Some(Arc::downgrade(&node));
        } else {
            self.last = Some(Arc::clone(&node));
        }
        self.first = Some(node);
    }

    /// 摘首部同时清前后投影，原 interest 或 sink 保节点不会在短锁下末释放。
    fn pop_front(&mut self, write: &mut QueueWrite) -> Link {
        let node = self.first.as_ref()?.clone();
        self.remove(&node, write)
    }

    /// 整链拼接只移动首尾原票，接缝的前后投影在同一短锁内更新。
    fn append(&mut self, other: &mut Self, write: &mut QueueWrite) {
        assert!(matches!(
            (self.kind, other.kind),
            (ChainKind::Ready, ChainKind::Ready) | (ChainKind::Overflow, ChainKind::Overflow)
        ));
        let Some(first) = other.first.take() else {
            return;
        };
        first.projection(self.kind, write).previous = self.last.as_ref().map(Arc::downgrade);
        if let Some(last) = &self.last {
            last.projection(self.kind, write).next = Some(first);
        } else {
            self.first = Some(first);
        }
        self.last = other.last.take();
    }

    /// 弱前驱与强后继允许 O(1) 摘除；同一 ready 锁保证回链升级必定成功。
    fn remove(&mut self, target: &EntryRef, write: &mut QueueWrite) -> Link {
        let projection = target.projection(self.kind, write);
        if !projection.linked {
            return None;
        }
        projection.linked = false;
        let previous = projection.previous.take().map(|previous| {
            previous
                .upgrade()
                .expect("epoll candidate predecessor lost")
        });
        let next = projection.next.take();
        if let Some(next) = &next {
            next.projection(self.kind, write).previous = previous.as_ref().map(Arc::downgrade);
        } else {
            self.last = previous.as_ref().cloned();
        }
        let removed = if let Some(previous) = previous {
            core::mem::replace(&mut previous.projection(self.kind, write).next, next)
        } else {
            core::mem::replace(&mut self.first, next)
        };
        assert!(
            removed
                .as_ref()
                .is_some_and(|node| Arc::ptr_eq(node, target)),
            "epoll candidate belongs to another chain"
        );
        removed
    }
}

struct ReadyQueue {
    ready: Chain,
    overflow: Chain,
    scanning: bool,
}

struct ReadyCore {
    state: SpinLock<ReadyQueue>,
    head: Arc<PollWaitHead>,
}

impl ReadyCore {
    /// 提前准备自身持续源，callback 强持此小对象而不强持事件 OFD。
    fn prepare() -> Result<Arc<Self>, FsError> {
        let head = Arc::try_new(PollWaitHead::new()).map_err(|_| FsError::NoMemory)?;
        Arc::try_new(Self {
            state: SpinLock::new(ReadyQueue {
                ready: Chain::new(ChainKind::Ready),
                overflow: Chain::new(ChainKind::Overflow),
                scanning: false,
            }),
            head,
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 发布完整新兴趣并重新 armed，随后控制者必须 query-only 观察当前状态。
    fn publish(&self, entry: &Entry, event: EpollEvent) {
        let _state = self.state.lock();
        let mut write = QueueWrite;
        let candidate = entry.candidate_mut(&mut write);
        candidate.event = event;
        candidate.armed = true;
    }

    /// 取得同一短锁下的事件快照，禁用项没有 query 或交付资格。
    fn event(&self, entry: &Entry) -> Option<EpollEvent> {
        let _state = self.state.lock();
        let mut write = QueueWrite;
        let candidate = entry.candidate_mut(&mut write);
        candidate.armed.then_some(candidate.event)
    }

    /// 新通知使用独立 overflow 投影；即使 ready 投影仍在扫描链，也不能剪掉它。
    fn notify(&self, entry: &EntryRef, mask: PollMask) {
        let accepted = {
            let mut state = self.state.lock();
            let mut write = QueueWrite;
            let candidate = entry.candidate_mut(&mut write);
            if !candidate.armed || (!mask.is_empty() && mask.0 & candidate.event.events == 0) {
                false
            } else {
                if state.scanning {
                    if !candidate.overflow.linked {
                        state.overflow.push_back(Arc::clone(entry), &mut write);
                    }
                } else if !candidate.ready.linked {
                    state.ready.push_back(Arc::clone(entry), &mut write);
                }
                true
            }
        };
        if accepted {
            self.head.notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        }
    }

    /// 控制域内移走 ready，并开启 callback 的独立 overflow 接收窗口。
    fn begin(&self) -> Chain {
        let mut state = self.state.lock();
        assert!(
            !state.scanning && state.overflow.first.is_none(),
            "epoll scan already active"
        );
        state.scanning = true;
        core::mem::replace(&mut state.ready, Chain::new(ChainKind::Ready))
    }

    /// 只在短锁下变更扫描链投影，query 和用户交付在本方法返回后执行。
    fn pop(&self, local: &mut Chain) -> Link {
        let _state = self.state.lock();
        local.pop_front(&mut QueueWrite)
    }

    /// 首项或正前缀后的错误都把当前未交付项保回原扫描尾之前。
    fn preserve(&self, local: &mut Chain, entry: EntryRef) {
        let _state = self.state.lock();
        local.push_front(entry, &mut QueueWrite);
    }

    /// 完整交付后才禁 ONESHOT；LT 保下一轮复核，ET 只等待新 source 通知。
    fn delivered(&self, retained: &mut Chain, entry: EntryRef, event: EpollEvent) {
        let _state = self.state.lock();
        let mut write = QueueWrite;
        if event.events & EPOLLONESHOT != 0 {
            entry.candidate_mut(&mut write).armed = false;
        } else if event.events & EPOLLET == 0 {
            retained.push_back(entry, &mut write);
        }
    }

    /// 合并 overflow、未交付尾和 LT 资格；关闭扫描窗口与重新通知属于同一次状态边界。
    fn finish(&self, mut local: Chain, mut retained: Chain) {
        let ready = {
            let mut state = self.state.lock();
            let mut write = QueueWrite;
            while let Some(entry) = state.overflow.pop_front(&mut write) {
                let candidate = entry.candidate_mut(&mut write);
                if candidate.armed && !candidate.ready.linked {
                    state.ready.push_back(entry, &mut write);
                }
            }
            local.append(&mut state.ready, &mut write);
            local.append(&mut retained, &mut write);
            state.ready = local;
            state.scanning = false;
            state.ready.first.is_some()
        };
        if ready {
            self.head.notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        }
    }

    /// 控制域先禁用并清两个候选投影，返回票在 ready 短锁外结束。
    fn forget(&self, entry: &EntryRef) {
        let detached = {
            let mut state = self.state.lock();
            assert!(!state.scanning, "removing interest during epoll scan");
            let mut write = QueueWrite;
            entry.candidate_mut(&mut write).armed = false;
            (
                state.ready.remove(entry, &mut write),
                state.overflow.remove(entry, &mut write),
            )
        };
        drop(detached);
    }
}

struct DeferredFiles<'a> {
    first: Link,
    _scan: SleepMutexGuard<'a, ()>,
}

impl<'a> DeferredFiles<'a> {
    /// 独占扫描 guard 覆盖所有临时目标拥有和控制锁外 put，不按兴趣总量准备容器。
    fn new(scan: SleepMutexGuard<'a, ()>) -> Self {
        Self {
            first: None,
            _scan: scan,
        }
    }

    /// 先把真实 OFD 与节点放入扫描拥有链，再调用可能睡眠的 provider。
    fn query(&mut self, entry: &EntryRef, event: EpollEvent) -> Result<PollMask, FsError> {
        let Some(file) = entry.target.upgrade() else {
            return Ok(PollMask::default());
        };
        let next = self.first.take();
        let projection = entry.scan_mut(self);
        assert!(
            projection.file.is_none() && projection.next.is_none(),
            "epoll entry queried twice in one scan"
        );
        projection.file = Some(file);
        projection.next = next;
        self.first = Some(Arc::clone(entry));
        entry
            .scan_mut(self)
            .file
            .as_ref()
            .unwrap()
            .pollable()
            .expect("watched OFD lost poll capability")
            .poll(None)
            .map(|mask| PollMask(mask.0 & event.events & ORDINARY_EVENTS))
    }
}

impl Drop for DeferredFiles<'_> {
    /// 控制 guard 已结束后逐项断开扫描链，末目标 put 可重入反向 release。
    fn drop(&mut self) {
        while let Some(entry) = self.first.take() {
            let projection = entry.scan_mut(self);
            let next = projection.next.take();
            let file = projection.file.take();
            self.first = next;
            drop(file);
            drop(entry);
        }
    }
}

struct RetiredEntry {
    // 字段按声明顺序析构；先同步撤全源，具体节点拥有覆盖每个 callback 的完整尾部。
    sources: PollTable,
    entry: EntryRef,
}

/// 事件 OFD 本身就是 FileRef payload；控制互斥与短候选锁分别承担睡眠和通知责任。
pub struct EpollFile {
    control: SleepMutex<index::Tree>,
    scan: SleepMutex<()>,
    core: Arc<ReadyCore>,
    flags: AtomicU32,
    inode: u64,
}

impl EpollFile {
    /// fallible 准备完整匿名事件对象；CLOEXEC 由调用者装入原描述符槽。
    pub fn try_new(flags: u32) -> Result<Arc<Self>, FsError> {
        if flags & !O_CLOEXEC != 0 {
            return Err(FsError::InvalidOperation);
        }
        let core = ReadyCore::prepare()?;
        let inode = NEXT_INODE
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
            .expect("epoll inode identity exhausted");
        Arc::try_new(Self {
            control: SleepMutex::new(index::Tree::new()),
            scan: SleepMutex::new(()),
            core,
            flags: AtomicU32::new(O_RDWR),
            inode,
        })
        .map_err(|_| FsError::NoMemory)
    }

    /// 当前实际目标与登记 fd 联合定位兴趣；调用者在表锁外强持 owner 和 target 至返回。
    pub fn control(
        &self,
        owner: &FileRef,
        operation: u32,
        target: &FileRef,
        fd: i32,
        event: Option<EpollEvent>,
    ) -> Result<(), FsError> {
        let provider = target.pollable().ok_or(FsError::NotPermitted)?;
        if !owner
            .epoll()
            .is_some_and(|owner| core::ptr::eq(owner, self))
            || Arc::ptr_eq(owner, target)
            || target.epoll().is_some()
        {
            return Err(FsError::InvalidOperation);
        }
        let watches = target.watch_head().ok_or(FsError::NotPermitted)?;
        if fd < 0 {
            return Err(FsError::BadFd);
        }
        let event = match operation {
            EPOLL_CTL_ADD | EPOLL_CTL_MOD => {
                Some(checked_event(event.ok_or(FsError::InvalidOperation)?)?)
            }
            EPOLL_CTL_DEL => None,
            _ => return Err(FsError::InvalidOperation),
        };
        let mut retired = None;
        let result = {
            let mut tree = self.control.lock();
            match operation {
                EPOLL_CTL_ADD => self.add(
                    &mut tree,
                    owner,
                    target,
                    fd,
                    watches,
                    event.unwrap(),
                    &mut retired,
                ),
                EPOLL_CTL_DEL => {
                    retired = self.detach(&mut tree, Key::new(target, fd));
                    if retired.is_some() {
                        Ok(())
                    } else {
                        Err(FsError::NotFound)
                    }
                }
                EPOLL_CTL_MOD => match tree.get(Key::new(target, fd)) {
                    None => Err(FsError::NotFound),
                    Some(entry) => {
                        let event = event.unwrap();
                        self.core.publish(entry, event);
                        match provider.poll(None) {
                            Ok(mask) => {
                                if mask.0 & event.events & ORDINARY_EVENTS != 0 {
                                    self.core.notify(entry, mask);
                                }
                                Ok(())
                            }
                            Err(error) => Err(error),
                        }
                    }
                },
                _ => unreachable!(),
            }
        };
        drop(retired);
        result
    }

    /// 准备期可失败；全部源登记完成后无分配发表，再按已发布 mask query 当前状态。
    fn add(
        &self,
        tree: &mut index::Tree,
        owner: &FileRef,
        target: &FileRef,
        fd: i32,
        watches: &Arc<WatchHead>,
        event: EpollEvent,
        retired: &mut Option<RetiredEntry>,
    ) -> Result<(), FsError> {
        let key = Key::new(target, fd);
        if tree.get(key).is_some() {
            return Err(FsError::Exists);
        }
        let entry = Entry::prepare(owner, target, fd, &self.core, watches)?;
        let sink: Arc<dyn PollNotify> = Arc::try_new(EntrySink {
            entry: Arc::clone(&entry),
        })
        .map_err(|_| FsError::NoMemory)?;
        let mut sources = PollTable::new(sink, PROVIDER_SOURCES)?;
        let provider = target.pollable().unwrap();
        // 准备期 callback 尚未 armed；部分登记失败也交退休拥有在控制锁外同步撤源。
        if let Err(error) = provider.poll(Some(&mut sources)) {
            *retired = Some(RetiredEntry { sources, entry });
            return Err(error);
        }
        entry.install_sources(sources, tree);
        tree.insert(Arc::clone(&entry));
        watches.attach(&entry.reverse);
        self.core.publish(&entry, event);
        match provider.poll(None) {
            Ok(mask) => {
                if mask.0 & event.events & ORDINARY_EVENTS != 0 {
                    self.core.notify(&entry, mask);
                }
                Ok(())
            }
            Err(error) => {
                *retired = self.detach(tree, key);
                Err(error)
            }
        }
    }

    /// 控制域只禁用并摘离原节点，具体退休拥有在 guard 结束后同步撤源再 put。
    fn detach(&self, tree: &mut index::Tree, key: Key) -> Option<RetiredEntry> {
        let entry = Arc::clone(tree.get(key)?);
        let sources = entry
            .take_sources(tree)
            .expect("epoll interest has no source table");
        self.core.forget(&entry);
        entry.watches.unlink(&entry.reverse);
        let removed = tree
            .remove(key)
            .expect("epoll interest disappeared under control lock");
        Some(RetiredEntry {
            sources,
            entry: removed,
        })
    }

    /// 反向 release 不升级目标，只在自己的控制 guard 结束后放退休节点。
    fn release_key(&self, key: Key) {
        let retired = {
            let mut tree = self.control.lock();
            self.detach(&mut tree, key)
        };
        drop(retired);
    }

    /// query 和 emit 都可睡眠；首个错误保项返回错误，正前缀后的错误保未交付尾及计数。
    pub fn scan(
        &self,
        maxevents: usize,
        emit: &mut dyn FnMut(EpollEvent) -> Result<(), FsError>,
    ) -> Result<usize, FsError> {
        if maxevents == 0 {
            return Err(FsError::InvalidOperation);
        }
        let mut deferred = DeferredFiles::new(self.scan.lock());
        let result = {
            let _control = self.control.lock();
            let mut local = self.core.begin();
            let mut retained = Chain::new(ChainKind::Ready);
            let mut count = 0;
            let mut error = None;
            while count < maxevents {
                let Some(entry) = self.core.pop(&mut local) else {
                    break;
                };
                let Some(event) = self.core.event(&entry) else {
                    continue;
                };
                let mask = match deferred.query(&entry, event) {
                    Ok(mask) => mask,
                    Err(cause) => {
                        self.core.preserve(&mut local, entry);
                        error = Some(cause);
                        break;
                    }
                };
                if mask.is_empty() {
                    continue;
                }
                if let Err(cause) = emit(EpollEvent {
                    events: mask.0,
                    data: event.data,
                }) {
                    self.core.preserve(&mut local, entry);
                    error = Some(cause);
                    break;
                }
                count += 1;
                self.core.delivered(&mut retained, entry, event);
            }
            self.core.finish(local, retained);
            if count == 0 {
                error.map_or(Ok(0), Err)
            } else {
                Ok(count)
            }
        };
        // 末 OFD put 可能回到 WatchHead.release 再取此控制锁，必须在 guard 已结束后执行。
        drop(deferred);
        result
    }

    /// 自身 poll 也复核候选，移走陈旧项而不消费 ET 或提前禁用 ONESHOT。
    fn query_ready(&self) -> Result<bool, FsError> {
        let mut deferred = DeferredFiles::new(self.scan.lock());
        let result = {
            let _control = self.control.lock();
            let mut local = self.core.begin();
            let mut result = Ok(false);
            while let Some(entry) = self.core.pop(&mut local) {
                let Some(event) = self.core.event(&entry) else {
                    continue;
                };
                match deferred.query(&entry, event) {
                    Ok(mask) if mask.is_empty() => {}
                    Ok(_) => {
                        self.core.preserve(&mut local, entry);
                        result = Ok(true);
                        break;
                    }
                    Err(error) => {
                        self.core.preserve(&mut local, entry);
                        result = Err(error);
                        break;
                    }
                }
            }
            self.core.finish(local, Chain::new(ChainKind::Ready));
            result
        };
        drop(deferred);
        result
    }

    /// 原生消费者观察真实节点拥有尾，不新增另一份兴趣或就绪状态。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_watch_lifetime(
        &self,
        target: &FileRef,
        fd: i32,
    ) -> Option<Weak<dyn Send + Sync>> {
        let tree = self.control.lock();
        let node = Arc::clone(tree.get(Key::new(target, fd))?);
        let ticket: Arc<dyn Send + Sync> = node;
        Some(Arc::downgrade(&ticket))
    }
}

/// 外部兴趣只接受当前所选 poll 位及 ET/ONESHOT，ERR/HUP 永远参加复核。
fn checked_event(mut event: EpollEvent) -> Result<EpollEvent, FsError> {
    if event.events & !(ORDINARY_EVENTS | EPOLLET | EPOLLONESHOT) != 0 {
        return Err(FsError::InvalidOperation);
    }
    event.events |= ALWAYS_EVENTS;
    Ok(event)
}

impl Pollable for EpollFile {
    /// 首轮先登记自身持续源，再复核候选；不建立 Waiter，也不允许嵌套监测。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&self.core.head)?;
        }
        self.query_ready().map(|ready| {
            if ready {
                PollMask::POLLIN | PollMask::POLLRDNORM
            } else {
                PollMask::default()
            }
        })
    }
}

impl FileLike for EpollFile {
    /// 事件只能经 scan 交付，普通字节 read 没有语义。
    fn read_into(
        &self,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::InvalidOperation)
    }
    /// 显式位置读取沿默认 positioned_io=false 返回 ESPIPE。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
    /// 匿名事件对象没有可作 dirfd 的路径。
    fn path(&self) -> Option<&Path> {
        None
    }
    /// 事件对象没有文件映射来源。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }
    /// 事件 OFD 提供自己的实际持续源，供临时 poll/select 使用。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }
    /// raw 事件入口借用本具体对象，不检查虚构类型标签。
    fn epoll(&self) -> Option<&EpollFile> {
        Some(self)
    }
    /// 初始 O_RDWR 与可变状态属于同 OFD，CLOEXEC 不进入对象。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }
    /// 沿共同 fcntl 状态位边界更新，不改访问模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }
    /// 只原子修改事件 OFD 的非阻塞位，复制描述符观察同一状态。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }
    /// 事件对象没有可截断内容。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }
    /// 事件对象没有文件或设备同步义务。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }
    /// noop seek 返回固定零位置，位移参数不改变事件状态。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Ok(0)
    }
    /// 无磁盘匿名身份使用 Other 和 0600，不补普通文件类型位。
    fn stat(&self) -> Result<FileStat, FsError> {
        let zero = InodeTime {
            seconds: 0,
            nanoseconds: 0,
        };
        Ok(FileStat {
            inode: self.inode,
            kind: InodeKind::Other,
            mode: 0o600,
            size: 0,
            uid: 0,
            gid: 0,
            links: 1,
            blocks: 0,
            device: 0,
            rdev: 0,
            block_size: crate::mm::PAGE_SIZE as u32,
            atime: zero,
            mtime: zero,
            ctime: zero,
        })
    }
    /// 事件对象不提供目录枚举。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }
    /// 事件对象不提供目录交付能力。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }
    /// 普通字节 write 不会修改事件对象。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::InvalidOperation)
    }
    /// 显式位置写入沿默认 positioned_io=false 返回 ESPIPE。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
}

impl Drop for EpollFile {
    /// 每轮至多摘十六项；控制锁外同步撤源及末 put，反复直到稳定树和候选链为空。
    fn drop(&mut self) {
        loop {
            let mut retired: [Option<RetiredEntry>; RETIRE_BATCH] = core::array::from_fn(|_| None);
            let empty = {
                let mut tree = self.control.lock();
                for slot in &mut retired {
                    let Some(entry) = tree.first() else {
                        break;
                    };
                    *slot = self.detach(&mut tree, entry.key);
                }
                tree.len == 0
            };
            drop(retired);
            if empty {
                break;
            }
        }
        let state = self.core.state.lock();
        assert!(
            !state.scanning && state.ready.first.is_none() && state.overflow.first.is_none(),
            "epoll candidate chains not drained"
        );
    }
}
