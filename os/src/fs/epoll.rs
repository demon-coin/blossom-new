//! `epoll` I/O 事件多路复用实例、监听集合管理与双队列就绪扫描引擎。
//!
//! # 模块职责与架构定位
//! 本模块实现匿名 `epoll` 打开文件描述（[`EpollFile`]）以及嵌入在各可监听打开文件描述（OFD，如管道、
//! 终端、套接字、`kmsg`）中的反向自动注销链表头（[`WatchHead`]）。上游由 `sys_epoll_create1`、
//! `sys_epoll_ctl`、`sys_epoll_pwait` 以及监听 `epoll` 描述符本身的 `ppoll` / `pselect` 调用；
//! 下游依赖 [`crate::fs::poll`] 的持续事件源（[`PollWaitHead`] / [`PollTable`]）接收异步状态通知。
//!
//! # 核心设计与关键状态机
//!
//! ## 1. 弱引用双向解耦与反向自动注销
//! `epoll` 监听项（[`Entry`]）仅持有被监听文件对象的弱引用（`Weak<dyn FileLike>`），不会延长目标文件
//! 的生命周期；同时被监听文件的 [`WatchHead`] 仅持有 `epoll` 实例的弱引用（`WatchNode::owner`）。
//! 当被监听的打开文件描述（OFD）最后一个强引用被释放（Drop）时，它会调用 [`WatchHead::release`]，
//! 自动通知所有监听它的 `epoll` 实例将该监听项从 AVL 索引树和就绪队列中摘除并撤销底层 `poll` 注册。
//!
//! ## 2. 双候选队列（`ready` 与 `overflow`）扫描协议
//! 在执行 [`EpollFile::scan`] 查询底层状态并向用户态拷贝事件期间，线程可能发生缺页或获取睡眠锁，
//! 因此不能一直持有保护就绪队列的自旋锁；与此同时，硬中断或其他线程仍可能并发触发新的事件通知。
//! [`ReadyCore`] 采用 `ready` + `overflow` 双双向链表解决并发冲突：
//!
//! ```text
//! 正常监听期 (scanning = false)
//!   底层事件源通知 ──> Entry 挂入 ReadyQueue.ready 链
//!
//! 进入 scan() / query_ready() 窗口 (scanning = true)
//!   1. begin(): 整条移走 ReadyQueue.ready 到栈局部变量 local，并将 scanning 置为 true
//!   2. 循环无自旋锁扫描 local 中各 Entry：
//!        - 期间新到达的异步通知 ──> Entry 挂入独立的 ReadyQueue.overflow 链（互不干扰）
//!        - 调用 provider.poll(None) 复查即时状态并调用 emit() 交付用户态
//!        - EPOLLONESHOT 交付成功后置 armed = false（直至下次 EPOLL_CTL_MOD 重新激活）
//!        - 水平触发（LT，未设 EPOLLET）交付成功后放入 retained 链，留待下一轮继续复查
//!        - 边缘触发（ET，设 EPOLLET）交付成功后不保留，仅靠后续新通知再次入队
//!        - 若中途出错或达到 maxevents 上限，未交付项通过 preserve() 退回 local 首部
//!   3. finish(): 将 overflow 新通知并入队列，按 [未扫描完的 local -> overflow 新项 -> LT retained]
//!      拼接回 ReadyQueue.ready，关闭 scanning = false；若队列非空则通知监听本 epoll 自身的等待者
//! ```
//!
//! ## 3. 单个监听项 [`Entry`] 的多锁分区投影布局
//! 同一个 `Arc<Entry>` 同时参与四套数据结构，内部通过独立的 `UnsafeCell` 分区并由各自的锁与零大小令牌保护：
//!
//! ```text
//! Arc<Entry>
//!  ├── 不可变身份字段: key (目标 OFD 地址 + fd), target (Weak), watches, reverse, core
//!  ├── index:     Links(UnsafeCell)            <── 由 EpollFile::control (SleepMutex) + IndexWrite 保护
//!  ├── sources:   UnsafeCell<Option<PollTable>> <── 由 EpollFile::control (SleepMutex) 保护
//!  ├── candidate: UnsafeCell<CandidateState>   <── 由 ReadyCore::state (SpinLock) + QueueWrite 保护
//!  └── scan:      UnsafeCell<ScanProjection>   <── 由 EpollFile::scan (SleepMutex) + DeferredFiles 保护
//! ```
//!
//! # 锁获取顺序与并发契约
//! 1. **主锁层级（严禁逆序）**：
//!    `EpollFile::scan`（`SleepMutex`） -> `EpollFile::control`（`SleepMutex`） ->
//!    底层文件对象的 `poll` 状态锁或 `WatchHead::state`（`SpinLock`） -> `ReadyCore::state`（`SpinLock`）。
//! 2. **延迟释放目标文件强引用（避免重入死锁）**：
//!    在 `scan` 期间，`entry.target.upgrade()` 会临时持有目标文件的一份强引用 `FileRef`。若在持有
//!    `EpollFile::control` 锁期间最后一次 `drop(file)`，会触发目标文件析构 -> [`WatchHead::release`] ->
//!    [`EpollFile::release_key`] 再次请求同一把 `EpollFile::control` 锁而死锁！因此 [`DeferredFiles`]
//!    先获取外层的 `EpollFile::scan` 锁，把所有升级出的 `FileRef` 串入 `Entry::scan` 链中保活，等到
//!    `EpollFile::control` 锁释放之后，再在 `DeferredFiles::drop` 中安全释放这些 `FileRef`。
//! 3. **锁外撤销事件源与析构**：
//!    无论是 `EPOLL_CTL_DEL`、`release_key` 还是 `EpollFile::drop`，在 `EpollFile::control` 锁内仅执行
//!    [`EpollFile::detach`] 摘链并打包成 [`RetiredEntry`]，必须在释放 `control` 锁之后再 `drop(retired)`，
//!    利用字段声明顺序先撤销全部底层 `PollTable` 订阅，最后释放 `Entry` 本体。

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

/// `epoll_ctl` 操作码：向 `epoll` 实例新增一个监听项。
pub const EPOLL_CTL_ADD: u32 = 1;
/// `epoll_ctl` 操作码：从 `epoll` 实例移除一个已注册的监听项。
pub const EPOLL_CTL_DEL: u32 = 2;
/// `epoll_ctl` 操作码：修改已注册监听项的事件掩码与用户数据。
pub const EPOLL_CTL_MOD: u32 = 3;

/// 单次触发标志：事件成功交付一次后自动将该监听项置为未激活（`armed = false`），需通过 `EPOLL_CTL_MOD` 重新激活。
pub const EPOLLONESHOT: u32 = 1 << 30;
/// 边缘触发（Edge-Triggered）标志：事件交付后不自动保留在就绪队列中，仅当底层事件源再次发出新通知时才重新入队。
pub const EPOLLET: u32 = 1 << 31;

/// `EpollFile::drop` 每轮在控制锁内批量摘除并移交锁外销毁的最大监听项数量（避免在栈上分配过大数组或长时间持锁）。
const RETIRE_BATCH: usize = 16;
/// 单个被监听文件对象最多向 `PollTable` 注册的底层等待队列头数量上限（例如读写双通道各一个）。
const PROVIDER_SOURCES: usize = 2;

/// 允许用户注册并向用户态报告的全部常规 I/O 事件位掩码集合。
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
/// 无论用户是否在 `events` 中显式指定，始终强制参与监听与就绪报告的错误及挂断事件位。
const ALWAYS_EVENTS: u32 = PollMask::POLLERR.0 | PollMask::POLLHUP.0;

/// 为匿名 `epoll` 实例分配进程内唯一伪 inode 编号的全局单调递增计数器。
static NEXT_INODE: AtomicU64 = AtomicU64::new(1);

/// 与 RISC-V 64 和 LoongArch 64 用户态 ABI 一致的 `struct epoll_event` 事件描述结构体（自然对齐共 16 字节）。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EpollEvent {
    /// 监听或实际触发的事件标志位掩码（`POLL*` 位以及控制输入时的 `EPOLLET` / `EPOLLONESHOT`）。
    pub events: u32,
    /// 用户态传入并由内核原样保存、原样回传的 64 位不透明关联数据（`epoll_data_t` 位模式，内核绝不将其当指针解引用）。
    pub data: u64,
}

const _: () = assert!(core::mem::size_of::<EpollEvent>() == 16);
const _: () = assert!(core::mem::offset_of!(EpollEvent, data) == 8);

/// 在 `epoll` AVL 索引树中唯一标识一条监听记录的复合键（由底层打开文件对象地址与描述符编号组成的二元组）。
#[derive(Clone, Copy, Eq, Ord, PartialEq, PartialOrd)]
struct Key {
    /// 被监听底层打开文件对象（`FileRef` 数据负载）的起始虚拟地址数值。
    object: usize,
    /// 注册该监听项时所使用的用户态文件描述符编号。
    fd: i32,
}

impl Key {
    /// 根据目标打开文件对象的 `Arc` 数据地址与文件描述符编号 `fd` 构造复合查找键。
    ///
    /// 监听项同时持有目标对象的 `Weak` 引用；只要该监听项尚未注销，底层 `Arc` 控制块就不会被回收，
    /// 从而保证目标对象即使已析构，其堆地址也不会被新分配的其他 `FileRef` 复用（避免 ABA 键冲突）。
    fn new(target: &FileRef, fd: i32) -> Self {
        Self {
            object: Arc::as_ptr(target) as *const () as usize,
            fd,
        }
    }
}

/// 挂载在被监听文件 [`WatchHead`] 反向链表上的单个监控记录节点。
struct WatchNode {
    /// 该监听项在所属 `epoll` 实例 AVL 树中的复合查找键。
    key: Key,
    /// 指向所属 `epoll` 打开文件对象的弱引用（避免反向监控链反向延长 `epoll` 实例的生命周期）。
    owner: Weak<dyn FileLike>,
    /// 反向单链表中指向下一个监控节点的指针，受自旋锁保护。
    next: SpinLock<Option<Arc<WatchNode>>>,
}

/// [`WatchHead`] 受自旋锁保护的内部链表状态。
struct WatchState {
    /// 反向监控单链表的首节点指针；无 `epoll` 监听该文件时为 `None`。
    chain: Option<Arc<WatchNode>>,
    /// 标记所属文件对象是否已进入最终析构并执行过 [`WatchHead::release`]。
    released: bool,
}

/// 嵌入在每个可被 `epoll` 持久监听的打开文件描述（OFD）中的反向监控链表头。
///
/// 当文件对象的最后一个强引用被释放时，通过遍历本链表通知所有关联的 `epoll` 实例自动清理对应监听项。
pub struct WatchHead {
    /// 保护反向监控单链表及已释放标志的自旋锁。
    state: SpinLock<WatchState>,
}

impl WatchHead {
    /// 创建一个尚未处于释放状态且无任何监控节点的空反向监控链表头。
    pub const fn new() -> Self {
        Self {
            state: SpinLock::new(WatchState {
                chain: None,
                released: false,
            }),
        }
    }

    /// 在所属文件对象析构时调用：标记反向链已关闭，并在自旋锁外逐个通知存活的 `epoll` 实例移除对应监听项。
    pub fn release(&self) {
        // Step 1: 持自旋锁置位 `released` 并一次性取走整条反向监控链，避免持自旋锁调用睡眠锁。
        let mut chain = {
            let mut state = self.state.lock();
            state.released = true;
            state.chain.take()
        };

        // Step 2: 在自旋锁外逐个拆解节点，尝试升级所属 `epoll` 实例弱引用并调用 `release_key` 同步注销。
        while let Some(node) = chain {
            chain = node.next.lock().take();
            if let Some(owner) = node.owner.upgrade() {
                owner
                    .epoll()
                    .expect("reverse watch owner is not epoll")
                    .release_key(node.key);
                drop(owner);
            }
        }
    }

    /// 将新的反向监控节点头插法挂入当前链表（调用方此时持有目标文件的强引用，因此目标文件不可能并发处于 `released` 状态）。
    fn attach(&self, node: &Arc<WatchNode>) {
        let mut state = self.state.lock();
        assert!(!state.released, "registering a released OFD watch head");
        *node.next.lock() = state.chain.take();
        state.chain = Some(Arc::clone(node));
    }

    /// 从反向监控单链表中查找并摘除指定节点 `target`，并将摘下的强引用放到自旋锁外释放。
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

/// 监听项节点的共享强引用类型别名。
type EntryRef = Arc<Entry>;
/// 侵入式链表与队列的可选后继强引用类型别名。
type Link = Option<EntryRef>;

/// 标识监听项内部使用的两条独立就绪候选双向链表槽位。
#[derive(Clone, Copy)]
enum ChainKind {
    /// 主就绪候选链表（正常监听期间接收通知，以及扫描时移交到局部扫描队列使用的槽位）。
    Ready,
    /// 溢出候选链表（仅在 `scanning == true` 扫描窗口开启期间接收并发新通知的独立槽位）。
    Overflow,
}

/// 监听项在单条侵入式双向候选链表（`ready` 或 `overflow`）中的前后驱指针与入队标志。
struct Projection {
    /// 当前监听项是否正挂载在对应的候选链表中（防止重复入队）。
    linked: bool,
    /// 指向链表前驱节点的弱引用（避免与 `next` 强引用构成循环引用，同时支持 $O(1)$ 任意位置摘除）。
    previous: Option<Weak<Entry>>,
    /// 指向链表后继节点的强引用。
    next: Link,
}

/// [`Entry`] 中受 `ReadyCore::state` 自旋锁保护的就绪候选状态与事件配置。
struct CandidateState {
    /// 当前生效的监听事件掩码（含强制附加的 `POLLERR | POLLHUP`）与用户态 `data` 原样载荷。
    event: EpollEvent,
    /// 该监听项当前是否已激活：未完成发布的准备期或 `EPOLLONESHOT` 触发交付后为 `false`，此时忽略一切通知与扫描。
    armed: bool,
    /// 在 [`ChainKind::Ready`] 链表（或扫描期间的局部 `local` / `retained` 链表）中的链接状态。
    ready: Projection,
    /// 在 [`ChainKind::Overflow`] 链表中的链接状态（允许节点在被 `local` 扫描的同时记录并发到达的新通知）。
    overflow: Projection,
}

/// 仅在持有 `ReadyCore::state` 自旋锁期间构造的零大小写权限令牌，用于在编译期约束 [`Entry::candidate_mut`] 的可变借用。
struct QueueWrite;

/// [`Entry`] 中受 `EpollFile::scan` 睡眠锁保护的当轮扫描临时持有状态。
struct ScanProjection {
    /// 本轮扫描从 `target` 弱引用升级得到的目标文件强引用（保活至释放 `control` 锁后再统一释放，防止重入死锁）。
    file: Option<FileRef>,
    /// 指向 [`DeferredFiles`] 延迟释放单链表中下一个监听项节点的强引用。
    next: Link,
}

/// 表示注册在 `epoll` 实例中的单个监听项核心节点，集中承载索引树、事件源句柄、候选队列与扫描链四套侵入式状态。
struct Entry {
    /// 在所属 `epoll` 实例 AVL 树中的唯一复合键 `(目标文件地址, fd)`。
    key: Key,
    /// 指向被监听打开文件对象的弱引用。
    target: Weak<dyn FileLike>,
    /// 被监听文件对象的反向监控链表头强引用，供主动删除时直接摘除 `reverse` 节点。
    watches: Arc<WatchHead>,
    /// 挂载在 `watches` 链表中的反向监控节点。
    reverse: Arc<WatchNode>,
    /// 所属 `epoll` 实例的就绪队列核心强引用，供异步通知回调直接入队而不必强持有整个 `EpollFile`。
    core: Arc<ReadyCore>,
    /// 嵌入的 AVL 平衡树链接字段（由 `EpollFile::control` 睡眠锁独占保护）。
    index: index::Links,
    /// 就绪候选配置与双链表指针（由 `ReadyCore::state` 自旋锁独占保护）。
    candidate: UnsafeCell<CandidateState>,
    /// 向被监听文件底层事件源注册的持续订阅句柄表（由 `EpollFile::control` 睡眠锁独占保护）。
    sources: UnsafeCell<Option<PollTable>>,
    /// 扫描期间延迟释放目标文件强引用的单链表槽位（由 `EpollFile::scan` 睡眠锁独占保护）。
    scan: UnsafeCell<ScanProjection>,
}

// SAFETY:
// 1. `Entry` 中的不可变字段（`key`、`target`、`watches`、`reverse`、`core`）所含类型全部满足 `Send`；
// 2. 四个内部可变字段分属互不重叠的同步域：`index` 与 `sources` 仅在 `EpollFile::control` 互斥锁下访问，
//    `candidate` 仅在 `ReadyCore::state` 自旋锁下访问，`scan` 仅在 `EpollFile::scan` 互斥锁下访问，跨线程转移所有权安全。
unsafe impl Send for Entry {}
// SAFETY:
// 1. 共享引用 `&Entry` 上的所有不可变字段均可安全跨线程并发读取；
// 2. 各 `UnsafeCell` 字段（`index`、`candidate`、`sources`、`scan`）分别受各自独立的互斥锁或自旋锁严格串行化访问，
//    且绝不会构造覆盖整个结构体的 `&mut Entry`（除独占析构 `Drop::drop` 外），不会产生并发数据竞争。
unsafe impl Sync for Entry {}

impl Entry {
    /// 在向底层事件源注册前分配并初始化监听项及其反向监控节点（初始 `armed = false`，确保注册中途到达的通知不会提前入队）。
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

    /// 凭 [`QueueWrite`] 写令牌获取候选状态与队列指针的可变引用。
    fn candidate_mut<'a>(&'a self, _write: &'a mut QueueWrite) -> &'a mut CandidateState {
        // SAFETY:
        // 1. 调用方必须持有 `&mut QueueWrite`，而 `QueueWrite` 仅在已获取 `ReadyCore::state` 自旋锁的作用域内构造；
        // 2. 生命周期 `'a` 将返回的可变引用绑定在 `&'a mut QueueWrite` 上，保证同一时刻至多存在一个活跃的 `&mut CandidateState`；
        // 3. `candidate` 与 `index`、`sources`、`scan` 存储完全分离，互不产生别名冲突。
        unsafe { &mut *self.candidate.get() }
    }

    /// 根据指定的候选链类型（`Ready` 或 `Overflow`）获取对应双向链表节点投影的可变引用。
    fn projection<'a>(&'a self, kind: ChainKind, write: &'a mut QueueWrite) -> &'a mut Projection {
        let state = self.candidate_mut(write);
        match kind {
            ChainKind::Ready => &mut state.ready,
            ChainKind::Overflow => &mut state.overflow,
        }
    }

    /// 凭 [`DeferredFiles`] 的独占借用获取当轮扫描延迟释放链槽位的可变引用。
    fn scan_mut<'a>(&'a self, _write: &'a mut DeferredFiles<'_>) -> &'a mut ScanProjection {
        // SAFETY:
        // 1. `DeferredFiles` 内部独占持有 `EpollFile::scan` 睡眠锁守卫，直到 `DeferredFiles::drop` 遍历清空整条 `scan` 链后才释放该锁；
        // 2. 其他线程或通知回调从不访问 `Entry::scan`，且返回引用的生命周期 `'a` 绑定在 `&'a mut DeferredFiles` 上，无别名冲突。
        unsafe { &mut *self.scan.get() }
    }

    /// 在持有控制锁（`&mut index::Tree`）期间将已完成底层注册的 [`PollTable`] 存入监听项槽位。
    fn install_sources(&self, sources: PollTable, _tree: &mut index::Tree) {
        // SAFETY:
        // 1. `_tree: &mut index::Tree` 证明调用方独占持有 `EpollFile::control` 睡眠锁；
        // 2. `Entry::sources` 仅在持有该控制锁的 `install_sources`（安装一次）与 `take_sources`（移除一次）中访问，异步通知与扫描路径从不触碰该字段。
        let slot = unsafe { &mut *self.sources.get() };
        assert!(slot.is_none(), "epoll sources installed twice");
        *slot = Some(sources);
    }

    /// 在持有控制锁（`&mut index::Tree`）期间取走监听项保存的 [`PollTable`]，供释放控制锁后同步撤销底层事件源注册。
    fn take_sources(&self, _tree: &mut index::Tree) -> Option<PollTable> {
        // SAFETY:
        // 1. `_tree: &mut index::Tree` 证明调用方独占持有 `EpollFile::control` 睡眠锁，串行化所有添加、删除与反向注销操作；
        // 2. 异步回调与扫描路径从不访问 `Entry::sources`，在此处独占取出 `Option<PollTable>` 不会与任何读者或写者冲突。
        unsafe { &mut *self.sources.get() }.take()
    }
}

impl Drop for Entry {
    /// 断言监听项在销毁最后一个强引用前已彻底从底层事件源、AVL 索引树、双就绪队列及延迟扫描链中全部摘除。
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

/// 绑定到特定 [`Entry`] 的底层 `poll` 通知回调适配器。
struct EntrySink {
    /// 接收通知后需要入队的所属监听项强引用。
    entry: EntryRef,
}

impl PollNotify for EntrySink {
    /// 在底层事件源的通知回调中，将状态变化转发给 `ReadyCore` 按当前激活掩码判断是否加入候选队列。
    fn notify(&self, mask: PollMask) {
        self.entry.core.notify(&self.entry, mask);
    }
}

/// 由 [`Entry`] 内部指定投影字段（`Ready` 或 `Overflow`）串联而成的侵入式双向队列。
struct Chain {
    /// 本链表使用的节点内部投影槽位类别（`Ready` 或 `Overflow`）。
    kind: ChainKind,
    /// 双向链表的队首节点强引用；空队列时为 `None`。
    first: Link,
    /// 双向链表的队尾节点强引用；空队列时为 `None`。
    last: Link,
}

impl Chain {
    /// 创建一条使用指定投影槽位的空双向候选链表。
    const fn new(kind: ChainKind) -> Self {
        Self {
            kind,
            first: None,
            last: None,
        }
    }

    /// 将尚未入队的节点插入队尾（前驱使用 `Weak` 引用以避免相邻节点形成强引用环）。
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

    /// 将尚未入队的节点插回队首（用于在扫描中途遇到错误或仅探测就绪性时保留未交付项的原始顺序）。
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

    /// 弹出并返回队首节点，同时清空其前后驱指针与入队标志。
    fn pop_front(&mut self, write: &mut QueueWrite) -> Link {
        let node = self.first.as_ref()?.clone();
        self.remove(&node, write)
    }

    /// 在 $O(1)$ 时间内将同类型链表 `other` 的全部节点整体拼接到当前链表尾部，并清空 `other`。
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

    /// 利用弱前驱与强后继指针在 $O(1)$ 时间内将指定节点 `target` 从当前链表中摘除并返回；若未在链中则返回 `None`。
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

/// [`ReadyCore`] 受自旋锁保护的双队列状态。
struct ReadyQueue {
    /// 主就绪候选队列（非扫描期直接接收通知；扫描结束后汇总所有待复查项）。
    ready: Chain,
    /// 扫描窗口期专用的溢出候选队列（在 `scanning == true` 期间暂存并发到达的新通知）。
    overflow: Chain,
    /// 当前是否正处于 [`EpollFile::scan`] 或 [`EpollFile::query_ready`] 的无自旋锁扫描窗口内。
    scanning: bool,
}

/// `epoll` 实例的就绪候选队列核心与自身可读事件等待头。
///
/// 独立封装为小对象供异步通知回调（[`EntrySink`]）强持有，避免底层回调强持有整个 [`EpollFile`]。
struct ReadyCore {
    /// 保护双候选队列与各节点 `CandidateState` 的自旋锁。
    state: SpinLock<ReadyQueue>,
    /// 供外部 `epoll_pwait` / `ppoll` / `pselect` 订阅本 `epoll` 实例自身可读事件的持续等待队列头。
    head: Arc<PollWaitHead>,
}

impl ReadyCore {
    /// 分配并初始化空的双候选队列核心及其自身的 `PollWaitHead`。
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

    /// 在自旋锁内更新监听项的感兴趣事件配置并将其置为激活状态（`armed = true`）。
    fn publish(&self, entry: &Entry, event: EpollEvent) {
        let _state = self.state.lock();
        let mut write = QueueWrite;
        let candidate = entry.candidate_mut(&mut write);
        candidate.event = event;
        candidate.armed = true;
    }

    /// 在自旋锁内读取监听项当前的事件配置快照；若该项处于未激活状态（`armed == false`）则返回 `None`。
    fn event(&self, entry: &Entry) -> Option<EpollEvent> {
        let _state = self.state.lock();
        let mut write = QueueWrite;
        let candidate = entry.candidate_mut(&mut write);
        candidate.armed.then_some(candidate.event)
    }

    /// 处理底层事件源发来的状态变化通知：若监听项已激活且事件掩码匹配，将其挂入 `ready`（或扫描期的 `overflow`）队列并唤醒等待本 `epoll` 的线程。
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

    /// 开启扫描窗口：在自旋锁内将 `scanning` 置为 `true`，并整条取出当前 `ready` 链表供调用方在锁外逐项复查。
    fn begin(&self) -> Chain {
        let mut state = self.state.lock();
        assert!(
            !state.scanning && state.overflow.first.is_none(),
            "epoll scan already active"
        );
        state.scanning = true;
        core::mem::replace(&mut state.ready, Chain::new(ChainKind::Ready))
    }

    /// 在自旋锁保护下从局部扫描链表 `local` 队首弹出下一个待检查的监听项。
    fn pop(&self, local: &mut Chain) -> Link {
        let _state = self.state.lock();
        local.pop_front(&mut QueueWrite)
    }

    /// 在自旋锁保护下将尚未完成交付的监听项重新插回局部扫描链表 `local` 队首，避免丢失事件。
    fn preserve(&self, local: &mut Chain, entry: EntryRef) {
        let _state = self.state.lock();
        local.push_front(entry, &mut QueueWrite);
    }

    /// 在单个事件成功交付用户态后更新状态：`EPOLLONESHOT` 置为未激活，水平触发（非 `EPOLLET`）挂入 `retained` 留待下轮继续复查。
    fn delivered(&self, retained: &mut Chain, entry: EntryRef, event: EpollEvent) {
        let _state = self.state.lock();
        let mut write = QueueWrite;
        if event.events & EPOLLONESHOT != 0 {
            entry.candidate_mut(&mut write).armed = false;
        } else if event.events & EPOLLET == 0 {
            retained.push_back(entry, &mut write);
        }
    }

    /// 结束扫描窗口：合并扫描期间积攒的 `overflow` 新通知、未扫描完的 `local` 余项与水平触发保留项 `retained`，并在队列非空时发出可读通知。
    fn finish(&self, mut local: Chain, mut retained: Chain) {
        // Step 1: 持自旋锁将扫描期间落入 overflow 链的激活项转移至 state.ready，再按 [local剩余 -> 新通知 -> LT保留项] 顺序拼回主队列并关闭扫描标志。
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
        // Step 2: 若合并后的主就绪队列非空，在自旋锁外通知正在等待本 epoll 实例可读的观察者。
        if ready {
            self.head.notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        }
    }

    /// 在控制锁保护下禁用目标监听项（`armed = false`）并从 `ready` 与 `overflow` 双队列中摘除，将摘下的强引用放到自旋锁外释放。
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

/// 扫描期间临时保活目标打开文件强引用的守卫容器，确保在释放 `EpollFile::control` 锁之后再销毁目标文件引用。
struct DeferredFiles<'a> {
    /// 由 `Entry::scan` 串联起来的本轮已查询监听项单链表头（每个节点内暂存升级出的 `FileRef`）。
    first: Link,
    /// 独占持有的 `EpollFile::scan` 睡眠锁守卫，保证整个扫描与延迟释放期间 `Entry::scan` 投影的独占访问权。
    _scan: SleepMutexGuard<'a, ()>,
}

impl<'a> DeferredFiles<'a> {
    /// 基于已获取的 `scan` 睡眠锁守卫创建空的延迟文件释放链表。
    fn new(scan: SleepMutexGuard<'a, ()>) -> Self {
        Self {
            first: None,
            _scan: scan,
        }
    }

    /// 升级监听项的目标文件弱引用并挂入延迟释放链，随后调用底层 `poll(None)` 采样即时就绪掩码并与监听掩码取交集。
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
    /// 在 `EpollFile::control` 锁释放之后逐项拆解延迟链表并释放目标文件 `FileRef`（此时即使触发目标文件最后一个引用析构并回调 `WatchHead::release` 获取 `control` 锁也不会死锁）。
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

/// 从 `epoll` 控制树摘除后等待在控制锁外销毁的退役监听项打包结构。
///
/// 依赖 Rust 结构体按字段声明顺序析构的规则：先销毁 `sources` 同步从各底层 `PollWaitHead` 摘链，
/// 确保后续不会再有回调触发，最后才释放 `entry` 本体。
struct RetiredEntry {
    /// 该监听项向底层事件源注册的全部持续订阅句柄表（析构时首先同步摘链注销）。
    #[expect(dead_code, reason = "析构先同步撤销全部 poll 登记，等待回调尾部结束")]
    sources: PollTable,
    /// 监听项节点的强引用（在 `sources` 完成注销后析构并校验所有投影均已清空）。
    #[expect(
        dead_code,
        reason = "强引用保活兴趣节点，直到前一字段撤源并等待全部回调结束"
    )]
    entry: EntryRef,
}

/// 匿名 `epoll` 实例的打开文件对象（实现 [`FileLike`] 与 [`Pollable`]）。
pub struct EpollFile {
    /// 保护已注册监听项 AVL 索引树及串行化增删改、扫描与反向注销的睡眠互斥锁。
    control: SleepMutex<index::Tree>,
    /// 外层扫描睡眠互斥锁（严格先于 `control` 获取，保护 [`DeferredFiles`] 延迟释放链直至 `control` 释放之后）。
    scan: SleepMutex<()>,
    /// 双就绪候选队列与本实例自身 `PollWaitHead` 核心。
    core: Arc<ReadyCore>,
    /// 打开文件状态标志（初始为 `O_RDWR`，支持通过 `fcntl` 原子修改 `O_NONBLOCK` 等状态位）。
    flags: AtomicU32,
    /// `fstat` 报告的进程内唯一匿名伪 inode 编号。
    inode: u64,
}

impl EpollFile {
    /// 创建并初始化一个新的匿名 `epoll` 打开文件对象（仅接受 `0` 或 `O_CLOEXEC` 标志，其中 `O_CLOEXEC` 由上层写入描述符表槽位）。
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

    /// 执行 `epoll_ctl` 控制操作（`EPOLL_CTL_ADD`、`EPOLL_CTL_DEL` 或 `EPOLL_CTL_MOD`）。
    ///
    /// 调用方须在文件描述符表锁之外持有 `owner`（当前 `epoll` 文件对象）与 `target`（目标文件对象）的强引用直至本函数返回。
    pub fn control(
        &self,
        owner: &FileRef,
        operation: u32,
        target: &FileRef,
        fd: i32,
        event: Option<EpollEvent>,
    ) -> Result<(), FsError> {
        // Step 1: 校验目标对象具备 `Pollable` 与 `WatchHead` 能力、排除自监听或嵌套监听 `epoll` 实例，并校验操作码与事件掩码合法性。
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

        // Step 2: 获取控制睡眠锁，在 AVL 索引树上执行增、删、改；若产生需销毁的监听项，暂存入 `retired` 留待锁外清理。
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

        // Step 3: 在释放控制锁之后销毁 `retired`，同步撤销底层 `PollTable` 注册并释放节点强引用。
        drop(retired);
        result
    }

    /// 在持有 `control` 锁期间执行 `EPOLL_CTL_ADD`：先完成所有可能失败的堆分配与底层事件源注册，再无分配地挂入索引树与反向链并查询初始就绪状态。
    #[expect(
        clippy::too_many_arguments,
        reason = "兴趣键、源登记与锁外退休拥有共同完成一次可失败准备和无分配发表"
    )]
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
        // Step 1: 检查复合键是否已存在，防止重复添加同一 `(target, fd)`。
        let key = Key::new(target, fd);
        if tree.get(key).is_some() {
            return Err(FsError::Exists);
        }

        // Step 2: 分配未激活（`armed = false`）的监听项与通知回调，并向目标文件的底层 `PollWaitHead` 注册订阅。
        let entry = Entry::prepare(owner, target, fd, &self.core, watches)?;
        let sink: Arc<dyn PollNotify> = Arc::try_new(EntrySink {
            entry: Arc::clone(&entry),
        })
        .map_err(|_| FsError::NoMemory)?;
        let mut sources = PollTable::new(sink, PROVIDER_SOURCES)?;
        let provider = target.pollable().unwrap();
        // 若注册中途失败，将已部分注册的句柄打包进 `retired` 交由控制锁外统一注销。
        if let Err(error) = provider.poll(Some(&mut sources)) {
            *retired = Some(RetiredEntry { sources, entry });
            return Err(error);
        }

        // Step 3: 零内存分配提交阶段：安装事件源表、插入 AVL 树、挂入目标反向监控链并激活事件配置。
        entry.install_sources(sources, tree);
        tree.insert(Arc::clone(&entry));
        watches.attach(&entry.reverse);
        self.core.publish(&entry, event);

        // Step 4: 采样目标文件当前的即时就绪状态；若已就绪则立即入队通知，若采样报错则回滚摘除该监听项。
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

    /// 在持有 `control` 锁期间将键为 `key` 的监听项禁用，并从候选队列、目标反向监控链与 AVL 索引树中全部摘除，返回供锁外注销的 [`RetiredEntry`]。
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

    /// 供被监听文件的 [`WatchHead::release`] 在文件析构时回调：获取本实例控制锁摘除指定监听项，并在锁外完成底层事件源注销。
    fn release_key(&self, key: Key) {
        let retired = {
            let mut tree = self.control.lock();
            self.detach(&mut tree, key)
        };
        drop(retired);
    }

    /// 扫描当前就绪候选队列，复查各监听项的即时就绪状态并通过回调 `emit` 交付最多 `maxevents` 个事件。
    ///
    /// 若首个事件查询或拷贝即失败则返回对应错误；若已成功交付至少一个事件（Partial Progress）后遇到错误，
    /// 则将当前未交付项退回队列首部并返回已成功交付的事件个数。
    pub fn scan(
        &self,
        maxevents: usize,
        emit: &mut dyn FnMut(EpollEvent) -> Result<(), FsError>,
    ) -> Result<usize, FsError> {
        if maxevents == 0 {
            return Err(FsError::InvalidOperation);
        }

        // Step 1: 先获取外层 `scan` 锁创建延迟释放容器，再获取 `control` 锁并开启双队列扫描窗口。
        let mut deferred = DeferredFiles::new(self.scan.lock());
        let result = {
            let _control = self.control.lock();
            let mut local = self.core.begin();
            let mut retained = Chain::new(ChainKind::Ready);
            let mut count = 0;
            let mut error = None;

            // Step 2: 逐个弹出候选节点，复查底层即时就绪掩码并调用 `emit` 向调用方交付事件。
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

            // Step 3: 合并 `overflow` 新通知、剩余未扫描项与水平触发保留项，关闭扫描窗口。
            self.core.finish(local, retained);
            if count == 0 {
                error.map_or(Ok(0), Err)
            } else {
                Ok(count)
            }
        };

        // Step 4: 在释放 `control` 锁之后再销毁 `deferred` 中保活的目标文件强引用，避免触发目标文件析构重入 `WatchHead::release` 导致死锁。
        drop(deferred);
        result
    }

    /// 仅供外部 `poll`/`select` 查询本 `epoll` 实例自身是否可读：遍历复查候选队列并清理已失效的陈旧项，一旦发现首个真正就绪的项立即保留原项并返回 `true`（不消费边缘触发状态，也不触发 `EPOLLONESHOT` 禁用）。
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

    /// 返回指定监听项内部节点的类型擦除弱引用，供内核测试直接观察该监听项何时彻底释放。
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

/// 校验用户态传入的 `epoll` 事件掩码仅包含受支持的事件位与触发模式标志，并强制并入始终监听的 `POLLERR | POLLHUP`。
fn checked_event(mut event: EpollEvent) -> Result<EpollEvent, FsError> {
    if event.events & !(ORDINARY_EVENTS | EPOLLET | EPOLLONESHOT) != 0 {
        return Err(FsError::InvalidOperation);
    }
    event.events |= ALWAYS_EVENTS;
    Ok(event)
}

impl Pollable for EpollFile {
    /// 实现 `epoll` 实例自身的 `poll` 查询：首轮将本实例的 `ReadyCore::head` 注册到等待表，随后调用 `query_ready` 复查是否存在真实就绪事件。
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
    /// `epoll` 实例仅通过 `epoll_pwait`（[`Self::scan`]）交付结构化事件，不支持普通字节流 `read`。
    fn read_into(
        &self,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::InvalidOperation)
    }

    /// `epoll` 实例不支持带偏移量的随机读取，固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 匿名 `epoll` 实例不存在关联的文件系统目录路径。
    fn path(&self) -> Option<&Path> {
        None
    }

    /// 匿名 `epoll` 实例不支持 `mmap` 内存映射，无底层磁盘或内存 inode。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 暴露 `Pollable` 接口，允许 `ppoll` / `pselect` 监听本 `epoll` 文件描述符的可读事件。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    /// 返回对当前具体 [`EpollFile`] 实例的引用，供 `epoll` 系列系统调用与反向注销入口直接下转型访问。
    fn epoll(&self) -> Option<&EpollFile> {
        Some(self)
    }

    /// 读取当前打开文件描述的状态标志（初始为 `O_RDWR`，不含描述符表级别的 `O_CLOEXEC`）。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 按 `fcntl(F_SETFL)` 规则原子更新可变文件状态标志，保持读写访问模式不变。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除该打开文件描述的 `O_NONBLOCK` 标志（通过 `dup` / `fork` 共享同一描述的所有文件描述符同步生效）。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 匿名 `epoll` 实例无文件大小概念，不支持 `ftruncate`。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 匿名 `epoll` 实例无持久化后备存储，不支持 `fsync` / `fdatasync`。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 空操作 `lseek`，始终返回偏移量 `0` 且不改变任何事件状态。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Ok(0)
    }

    /// 返回匿名 `epoll` 实例的 `fstat` 元数据（类型为 `InodeKind::Other`，权限 `0600`）。
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

    /// `epoll` 实例不是目录，不支持 `readdir`。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// `epoll` 实例不是目录，不支持目录项遍历。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// `epoll` 实例不支持普通字节流 `write`。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::InvalidOperation)
    }

    /// `epoll` 实例不支持带偏移量的随机写入，固定返回 `FsError::IllegalSeek`（`ESPIPE`）。
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
    /// 销毁 `epoll` 实例时循环按批（每批最多 `RETIRE_BATCH` 项）在控制锁内摘除监听项，并在锁外同步撤销底层事件源注册，直至索引树与双候选队列彻底清空。
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
