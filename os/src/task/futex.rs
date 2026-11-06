//! 用户态快速互斥锁（Futex）内核等待队列与唤醒机制。
//!
//! ## 模块职责与架构定位
//! 本模块实现非优先级继承（非 PI）的 Futex 核心操作，负责当用户态原子同步发生竞争时在内核中阻塞等待、按位掩码（Bitset）唤醒以及跨键重排（Requeue）。
//! - **上游调用者**：系统调用层（处理 `futex` 的 `FUTEX_WAIT`、`FUTEX_WAKE`、`FUTEX_REQUEUE` / `FUTEX_CMP_REQUEUE` 等子命令）与线程退出清理路径（`os/src/task/user.rs` 中的 `clear_tid`）。
//! - **下游依赖**：`crate::mm::space`（解析虚拟地址背后的物理/文件后备身份 `FutexKey` 并在页表锁保护下读取用户 32 位字）、`crate::wait`（可被信号打断的任务挂起与唤醒器）、`crate::timer`（处理带超时的精确唤醒）。
//!
//! ## 核心设计与关键状态机
//! 1. **地址无关的后备身份（`FutexKey`）**：私有 Futex 以 `(Mm, 虚拟页对齐偏移)` 或匿名页身份标识，共享 Futex 以底层共享物理页或文件页偏移标识，支持跨进程共享内存同步。
//! 2. **侵入式栈上等待节点与分桶哈希表**：全局哈希表 `TABLE` 包含 32 个独立的自旋锁桶（`Bucket`）。`wait()` 将 `Node` 固定（`Pin`）在当前任务的内核栈上挂入对应桶的双向链表，零堆分配完成常见阻塞。
//! 3. **摘链与锁外唤醒分离**：唤醒方（`wake` / `requeue`）或取消方（`cancel`）通过 `Bucket::take` 将节点从链表摘除，移出内部的 `FutexKey` 与 `Waker` 构造值拥有的 `Detached`，随后以 `Release` 内存序将 `linked` 置为 `false`（摘除者对该栈上节点的最后一次访问）。等待方通过 `Acquire` 观察到 `linked == false` 后即可安全释放栈帧，而实际的调度唤醒与 `FutexKey` 引用递减均在所有锁外执行。
//!
//! ```text
//!   wait() 栈帧固定 Node            wake() / requeue() 摘除          cancel() 仲裁
//!  [linked = false]               [持 Bucket 锁 unlink]           [park_signal 返回]
//!         |                                |                              |
//!   MM -> PTE -> Bucket 锁         移出 key/waker -> Detached      Acquire 读 linked:
//!   校验 *uaddr == expected       linked.store(false, Release)    - false -> WaitOutcome::Woken
//!   挂入双向链表, linked = true            |                      - true  -> 持桶锁自摘除 ->
//!         |                        释放 MM / PTE / Bucket 锁                 TimedOut / Interrupted
//!   释放 MM / PTE / Bucket 锁      锁外调用 waker.notify()                |
//!   启动可选 Timer -> park_signal  锁外 drop(key)                 drop(waiter) -> timer.cancel_sync()
//!                                                                 栈上 Node 安全析构
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - **层级顺序**：`Mm` 映射读锁（`FutexRead`） ➔ 页表锁（`with_word` 内部持有） ➔ `TABLE[i]` 哈希桶自旋锁 ➔ `Waiter` / 调度锁或定时器基座锁（`Timer`）。
//! - **双桶加锁规则**：`requeue` 需要同时操作源桶与目的桶时，必须通过 `with_buckets` 严格按照**桶索引从小到大**的顺序加锁；若源桶与目的桶索引相同，则仅加锁一次并通过 `None` 标记同桶原地更新，防止死锁。
//! - **禁止持锁分配或睡眠**：缺页异常（`mm.fault`）与超出内联容量的 `Notifications` 堆扩容（`spill.try_reserve_exact`）必须在释放全部 `Mm`、页表和哈希桶锁之后执行，随后重新解析最新的 `FutexKey` 重试。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::cell::UnsafeCell;
use core::cmp::Ordering as CmpOrdering;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::arch::{self, UserAccess};
use crate::mm::space::{FutexAccessError, FutexKey, FutexRead, Mm, VmError};
use crate::sync::SpinLock;
use crate::time::{self, MonotonicInstant};
use crate::timer::{Timer, TimerKind};
use crate::wait::{WaitId, Waiter, Waker};

/// 全局 Futex 哈希表的分桶数量（必须为 2 的幂，以便用位与掩码快速取模）。
const BUCKETS: usize = 32;
/// 单次唤醒操作在栈上内联暂存的最大 `Detached` 通知数；超出此数量时在锁外向堆 `Vec` 扩容。
const INLINE_NOTIFICATIONS: usize = 64;

/// Futex 核心操作错误类型；保留语义化的失败原因，由系统调用 ABI 层统一映射为 POSIX errno。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FutexError {
    /// 用户空间 32 位字的当前值与期望值 `expected` 不相等（对应 `EAGAIN`）。
    Mismatch,
    /// 用户地址未按 4 字节自然对齐，或传入的位掩码 `mask` 为 0（对应 `EINVAL`）。
    InvalidArgument,
    /// 访问用户虚拟内存映射或处理缺页时发生的底层虚拟内存错误（如 `EFAULT`、`ENOMEM`）。
    Vm(VmError),
}

impl From<VmError> for FutexError {
    /// 将底层虚拟内存错误包装为 `FutexError::Vm`，保留具体的访存失败原因。
    fn from(error: VmError) -> Self {
        Self::Vm(error)
    }
}

/// `wait` 阻塞等待的最终返回结果。
///
/// 若等待节点已被 `wake` 从哈希桶中实际选中并摘除，即使同一时刻定时器到期或收到信号，也优先返回 `Woken`。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitOutcome {
    /// 节点被 `wake` 或 `requeue` 匹配选中并摘除唤醒。
    Woken,
    /// 等待超过指定单调时钟截止时间且未被唤醒方选中，由等待者自行摘除。
    TimedOut,
    /// 等待期间收到可打断当前睡眠的待处理信号且尚未超时，由等待者自行摘除。
    Interrupted,
}

/// 侵入式双向链表节点受当前所属哈希桶自旋锁保护的可变内部数据。
struct NodeData {
    /// 桶内双向链表的前驱节点裸指针；若为链表首节点则为 null。
    previous: *const Node,
    /// 桶内双向链表的后继节点裸指针；若为链表尾节点则为 null。
    next: *const Node,
    /// 当前等待绑定的底层 Futex 身份键；节点被摘除（`take`）时移出以便在桶锁外释放。
    key: Option<FutexKey>,
    /// 唤醒等待任务的句柄；节点被摘除（`take`）时移出以便在桶锁外触发调度通知。
    waker: Option<Waker>,
}

/// 固定在等待任务内核栈上的侵入式 Futex 等待队列节点。
///
/// `data` 仅在持有节点当前所属桶（`bucket`）的自旋锁时访问；`bucket` 原子记录当前所在桶索引，供等待方在取消时跟随 `requeue` 桶间迁移重试，无需遍历全部哈希桶。
struct Node {
    /// 受当前所属哈希桶自旋锁保护的链表指针、Futex 键与唤醒句柄。
    data: UnsafeCell<NodeData>,
    /// 节点当前所在的 `TABLE` 哈希桶索引（`0..BUCKETS`）；`requeue` 迁移节点时以 `Release` 更新。
    bucket: AtomicUsize,
    /// 节点是否仍挂在哈希桶链表中；摘除者完成最后一次节点访问后以 `Release` 置为 `false`，等待者以 `Acquire` 确认后方可退栈。
    linked: AtomicBool,
    /// 等待者订阅的 32 位匹配位掩码（Bitset），仅当与唤醒掩码按位与非零时才匹配。
    mask: u32,
    /// 内核测试（`ktest`）用于核对队列首节点身份的等待唯一标识。
    #[cfg(feature = "ktest")]
    id: WaitId,
    /// 可选的单调时钟超时截止时间；取消时用于区分超时（`TimedOut`）与信号打断（`Interrupted`）。
    deadline: Option<MonotonicInstant>,
    /// 标记该类型不可在内存中移动（`!Unpin`），保证链表裸指针在节点入链期间始终有效。
    _pinned: PhantomPinned,
}

impl Node {
    /// 在栈上初始化一个尚未入链的等待节点，并根据初始 `key` 的哈希值计算所属桶索引。
    fn new(
        key: FutexKey,
        waker: Waker,
        mask: u32,
        _id: WaitId,
        deadline: Option<MonotonicInstant>,
    ) -> Self {
        Self {
            bucket: AtomicUsize::new(key.hash() & (BUCKETS - 1)),
            data: UnsafeCell::new(NodeData {
                previous: ptr::null(),
                next: ptr::null(),
                key: Some(key),
                waker: Some(waker),
            }),
            linked: AtomicBool::new(false),
            mask,
            #[cfg(feature = "ktest")]
            id: _id,
            deadline,
            _pinned: PhantomPinned,
        }
    }
}

impl Drop for Node {
    /// 断言节点在销毁退栈前已彻底从哈希桶链表摘除，防止链表悬垂指针导致 UAF。
    fn drop(&mut self) {
        assert!(
            !self.linked.load(Ordering::Acquire),
            "dropping a registered futex node"
        );
    }
}

/// 从哈希桶摘除节点后移出的拥有权载荷，不再借用等待者栈上的 `Node` 地址。
///
/// 保证 `FutexKey` 最后一个强引用（`Arc`）的释放以及 `Waker` 内部 `Weak` 引用的析构都在桶自旋锁外完成。
struct Detached {
    /// 被摘除等待者原先持有的 Futex 后备身份键。
    key: FutexKey,
    /// 被摘除等待者的任务唤醒器句柄。
    waker: Waker,
}

/// 单个 Futex 哈希桶的侵入式双向链表头尾与计数状态，按 FIFO 顺序排队。
struct Bucket {
    /// 链表头节点指针（最早入队的等待者）；空桶时为 null。
    head: *const Node,
    /// 链表尾节点指针（最晚入队的等待者）；空桶时为 null。
    tail: *const Node,
    /// 当前挂在本桶链表中的活跃等待节点数量。
    members: usize,
}

// SAFETY:
// 1. `Bucket` 仅存放在全局静态表 `TABLE` 的 `SpinLock` 内，所有链表指针与 `Node::data` 访问均由桶自旋锁串行化；
// 2. `wait` 将 `Node` 固定（`Pin`）在当前任务内核栈上，在 `cancel` 完成唯一摘除仲裁前绝不退栈；
// 3. 外部唤醒者通过 `linked.store(false, Ordering::Release)` 结束对节点的最后一次访问，等待者通过 `Ordering::Acquire` 观察到 `false` 后才释放栈帧。
unsafe impl Send for Bucket {}

impl Bucket {
    /// 编译期构造空的哈希桶链表。
    const fn new() -> Self {
        Self {
            head: ptr::null(),
            tail: ptr::null(),
            members: 0,
        }
    }

    /// 将已固定在栈上的新等待节点挂入本桶链表尾部，并标记为已入链状态。
    fn insert(&mut self, node: Pin<&Node>) {
        assert!(!node.linked.load(Ordering::Relaxed));
        self.push(node.get_ref());
        node.linked.store(true, Ordering::Relaxed);
    }

    /// 将未入链或刚从源桶摘下的节点追加到本桶双向链表尾部。
    fn push(&mut self, node: &Node) {
        let members = self
            .members
            .checked_add(1)
            .expect("futex member count overflow");
        // SAFETY:
        // 1. 调用方持有当前桶的自旋锁，独占访问本桶的 `head`/`tail` 及桶内已入链尾节点的 `data`；
        // 2. 新入队节点由 `wait` 栈帧 `Pin` 保活，或由 `requeue` 在同时持有源桶与目的桶双锁期间从源桶移入（尚未将 `linked` 置为 `false`，等待者不会退栈）。
        unsafe {
            let data = &mut *node.data.get();
            data.previous = self.tail;
            data.next = ptr::null();
            if self.tail.is_null() {
                self.head = node;
            } else {
                (*(*self.tail).data.get()).next = node;
            }
            self.tail = node;
        }
        self.members = members;
    }

    /// 将本桶中仍处于入链状态的节点从双向链表中解链，并更新桶成员计数。
    fn unlink(&mut self, node: &Node) {
        // SAFETY: 调用方持有本桶自旋锁且保证 `node` 当前挂在本桶链表中；`node` 及其前后邻居均处于 `linked == true` 保活期，无并发访问。
        unsafe {
            let data = &mut *node.data.get();
            if data.previous.is_null() {
                self.head = data.next;
            } else {
                (*(*data.previous).data.get()).next = data.next;
            }
            if data.next.is_null() {
                self.tail = data.previous;
            } else {
                (*(*data.next).data.get()).previous = data.previous;
            }
            data.previous = ptr::null();
            data.next = ptr::null();
        }
        self.members = self
            .members
            .checked_sub(1)
            .expect("futex member count underflow");
    }

    /// 判断桶中节点是否与目标 `FutexKey` 指向同一后备存储，且位掩码存在交集。
    fn matches(node: &Node, key: &FutexKey, mask: u32) -> bool {
        // SAFETY: 所有调用点均持有 `node` 当前所属桶的自旋锁；并发取消或迁移在获取同一桶锁前无法修改或移出 `node.data`。
        let data = unsafe { &*node.data.get() };
        node.mask & mask != 0
            && data
                .key
                .as_ref()
                .expect("futex node has no key")
                .matches(key)
    }

    /// 统计本桶中匹配指定 `key` 与 `mask` 的等待节点数，最多统计到 `limit` 即提前停止。
    fn matching(&self, key: &FutexKey, mask: u32, limit: usize) -> usize {
        let mut pointer = self.head;
        let mut matching = 0;
        for _ in 0..self.members {
            if matching == limit {
                break;
            }
            // SAFETY: 调用方持有本桶自旋锁且循环次数严格受 `self.members` 限制，`pointer` 指向桶内仍入链保活的有效 `Node`。
            let node = unsafe { &*pointer };
            if Self::matches(node, key, mask) {
                matching += 1;
            }
            // SAFETY: 调用方持有本桶自旋锁，`node` 仍入链，读取其 `next` 指针不存在数据竞争且不会跨越解锁边界使用。
            pointer = unsafe { (*node.data.get()).next };
        }
        matching
    }

    /// 将节点从本桶链表摘除，移出其 `key` 与 `waker`，并以 `Release` 内存序解除节点的入链生命周期绑定。
    fn take(&mut self, node: &Node) -> Detached {
        self.unlink(node);
        // SAFETY: 本桶自旋锁保证对 `node.data` 的独占访问；`unlink` 后移出全部拥有值，此后不再解引用该节点。
        let detached = unsafe {
            let data = &mut *node.data.get();
            Detached {
                key: data.key.take().expect("futex node has no key"),
                waker: data.waker.take().expect("futex node has no waker"),
            }
        };
        // Release 写入必须是摘除者对栈上 `node` 的最后一次内存访问；此后等待者随时可能观察到 `false` 并销毁栈帧。
        node.linked.store(false, Ordering::Release);
        detached
    }

    /// 在 `requeue` 操作中更新节点的 `FutexKey`，并在跨桶时将节点从源桶移入目的桶尾部。
    fn move_node(
        &mut self,
        node: &Node,
        destination: Option<&mut Bucket>,
        key: &FutexKey,
        bucket: usize,
    ) {
        // SAFETY:
        // 1. 调用方通过 `with_buckets` 同时持有源桶与目的桶自旋锁，排除了并发 `cancel`、唤醒和其他迁移；
        // 2. 调用方栈上仍持有 `source_key` 的强引用副本，因此此处替换下来的 `old_key` 不会在桶自旋锁内触发底层对象（如 `Mm` 或 Inode）的最后一次引用释放；
        // 3. 节点的 `linked` 始终保持 `true`，保留原 `WaitId`、`Waker` 与超时设置不变。
        let old_key = unsafe { (*node.data.get()).key.replace(key.clone()) };
        drop(old_key);
        if let Some(destination) = destination {
            self.unlink(node);
            destination.push(node);
        }
        node.bucket.store(bucket, Ordering::Release);
    }
}

// ponytail: 固定 32 桶并在桶内线性遍历活跃等待节点；实测出现桶锁争用热点后再扩大分桶数。
static TABLE: [SpinLock<Bucket>; BUCKETS] = [const { SpinLock::new(Bucket::new()) }; BUCKETS];

/// 唤醒阶段收集已摘除节点的临时容器。
///
/// 常见小批量唤醒（≤ 64 个）直接存放在栈上内联数组以避免堆分配；超出容量时先释放全部锁，预分配精确容量的 `spill` 向量后重试。
struct Notifications {
    /// 栈上内联存储槽位，容纳前 `INLINE_NOTIFICATIONS` 个已摘除节点。
    inline: [Option<Detached>; INLINE_NOTIFICATIONS],
    /// 当前已推入容器的待通知项总数（含内联槽位与 `spill` 溢出项）。
    count: usize,
    /// 当唤醒数量超过 `INLINE_NOTIFICATIONS` 时，在锁外预留容量并存放超出部分的堆向量。
    spill: Vec<Detached>,
}

impl Notifications {
    /// 创建空的唤醒通知收集容器，初始不进行任何堆内存分配。
    fn new() -> Self {
        Self {
            inline: [const { None }; INLINE_NOTIFICATIONS],
            count: 0,
            spill: Vec::new(),
        }
    }

    /// 返回当前容器在不重新分配内存的前提下最多可容纳的通知总数。
    fn capacity(&self) -> usize {
        INLINE_NOTIFICATIONS + self.spill.capacity()
    }

    /// 在所有锁外为超出内联容量的部分精确预留堆内存；分配失败返回 `VmError::NoMemory`。
    fn reserve(&mut self, needed: usize) -> Result<(), FutexError> {
        self.spill
            .try_reserve_exact(needed.saturating_sub(INLINE_NOTIFICATIONS))
            .map_err(|_| FutexError::Vm(VmError::NoMemory))
    }

    /// 将已摘除的节点载荷推入容器（调用前必须保证容量充足，绝不在桶锁内触发堆分配）。
    fn push(&mut self, detached: Detached) {
        assert!(
            self.count < self.capacity(),
            "futex notification preparation too small"
        );
        if self.count < INLINE_NOTIFICATIONS {
            self.inline[self.count] = Some(detached);
        } else {
            self.spill.push(detached);
        }
        self.count += 1;
    }

    /// 在释放全部 MM、页表和哈希桶锁后逐一唤醒被选中的等待任务，并在锁外释放其 `FutexKey`。
    fn notify(&mut self) {
        for detached in self
            .inline
            .iter_mut()
            .filter_map(Option::take)
            .chain(self.spill.drain(..))
        {
            // 节点被选中并从桶中摘链即计入唤醒成功数，无论目标任务此前是否已收到其他唤醒信号。
            detached.waker.notify();
            drop(detached.key);
        }
    }
}

/// 校验 Futex 用户虚拟地址是否按 4 字节自然对齐、完整落在用户地址空间内，且位掩码非零。
fn validate(address: usize, mask: u32) -> Result<(), FutexError> {
    if address & (core::mem::size_of::<u32>() - 1) != 0 || mask == 0 {
        return Err(FutexError::InvalidArgument);
    }
    if address
        .checked_add(core::mem::size_of::<u32>())
        .is_none_or(|end| arch::USER_ADDRESS_END < end)
    {
        return Err(FutexError::Vm(VmError::InvalidRange));
    }
    Ok(())
}

/// `wait` 超时定时器的中断上下文到期回调函数：唤醒阻塞中的等待任务。
fn timeout_notify(timer: Pin<&Timer>) {
    // SAFETY:
    // 1. `wait` 将 `Waker` 固定（`pin!`）在栈上，并在退栈前先销毁 `Waiter` 再调用 `timer.cancel_sync()` 等待正在执行的中断回调彻底退出；
    // 2. 回调仅共享读取 `Waker` 发出唤醒通知，不访问 Futex `Node` 或桶链表，也不持久化该裸指针。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// 在 `Mm` 映射读锁下解析用户虚拟地址对应的 `FutexKey`；若页未驻留或需处理私有文件写时复制（COW），先释放读锁执行 `mm.fault` 再重试。
fn read_key(
    mm: &Arc<Mm>,
    address: usize,
    private: bool,
) -> Result<(FutexRead<'_>, FutexKey), FutexError> {
    loop {
        let mapping = mm.futex_read();
        match mapping.key(address, private) {
            Ok(key) => return Ok((mapping, key)),
            Err(FutexAccessError::Vm(error)) => return Err(error.into()),
            Err(FutexAccessError::NeedsFault(access)) => {
                drop(mapping);
                mm.fault(address, access)?;
            }
        }
    }
}

/// 在同一把 `Mm` 映射读锁快照下同时解析 `requeue` 源地址与目的地址的 `FutexKey`；若目的页触发缺页，释放读锁与已取源键处理缺页后重新解析两端。
fn read_keys(
    mm: &Arc<Mm>,
    source: usize,
    destination: usize,
    private: bool,
) -> Result<(FutexRead<'_>, FutexKey, FutexKey), FutexError> {
    loop {
        let (mapping, source_key) = read_key(mm, source, private)?;
        match mapping.key(destination, private) {
            Ok(destination_key) => return Ok((mapping, source_key, destination_key)),
            Err(FutexAccessError::Vm(error)) => return Err(error.into()),
            Err(FutexAccessError::NeedsFault(access)) => {
                drop(mapping);
                drop(source_key);
                mm.fault(destination, access)?;
            }
        }
    }
}

/// 若用户地址处的 32 位值等于 `expected`，将当前任务挂入对应的 Futex 哈希桶阻塞等待，直至被唤醒、超时或被信号打断。
///
/// 用户字的读取比较与节点入链在 `Mm` 读锁 ➔ 页表锁 ➔ 哈希桶锁的原子临界区内完成，彻底消除“检查通过后、入队睡眠前错失 `wake`”的唤醒丢失竞态。
pub fn wait(
    mm: &Arc<Mm>,
    address: usize,
    private: bool,
    expected: u32,
    mask: u32,
    deadline: Option<MonotonicInstant>,
) -> Result<WaitOutcome, FutexError> {
    validate(address, mask)?;
    loop {
        // Step 1: 解析 FutexKey 并确保目标页已驻留（read_key 内部缺页路径可能使用其自身的临时 Waiter，故必须在创建本轮 futex Waiter 前完成）。
        let (mapping, key) = read_key(mm, address, private)?;
        if mapping.with_word(address, |_| ()).is_none() {
            drop(mapping);
            drop(key);
            mm.fault(address, UserAccess::Read)?;
            continue;
        }

        // Step 2: 在当前内核栈上固定可中断 Waiter、超时 Timer 以及侵入式等待节点 Node。
        let waiter = Waiter::new_interruptible();
        let waker = core::pin::pin!(waiter.waker());
        let timer = core::pin::pin!(Timer::new(
            crate::cpu::current().id,
            TimerKind::Precise,
            timeout_notify,
            waker.as_ref().get_ref() as *const Waker as usize,
        ));
        let node = core::pin::pin!(Node::new(
            key.clone(),
            waiter.waker(),
            mask,
            waiter.id(),
            deadline
        ));

        // Step 3: 按 MM 读锁 -> PTE 锁 -> Bucket 锁顺序原子校验用户 32 位字并将节点插入桶尾部。
        let index = key.hash() & (BUCKETS - 1);
        let registration = mapping
            .with_word(address, |direct| {
                let mut bucket = TABLE[index].lock();
                // SAFETY:
                // 1. 入口 `validate` 已校验 `address` 4 字节自然对齐且不越出用户空间（不会跨页）；
                // 2. `mapping.with_word` 持有映射读锁与叶页表锁，保证 `direct` 指向已驻留、可读的物理 RAM 直映地址；
                // 3. 使用架构级 `load_user_u32` 执行原子 Acquire 读取，不构造 Rust 引用，避免与用户态并发原子写产生未定义行为。
                let value = unsafe { arch::load_user_u32(direct as *const u32) };
                if value != expected {
                    return Err(FutexError::Mismatch);
                }
                bucket.insert(node.as_ref());
                Ok(())
            })
            .expect("stable futex read lost its resident page");
        drop(mapping);
        drop(key);
        registration?;

        // Step 4: 若指定了超时截止时间，启动借用栈上节点的精确硬件定时器，然后进入可被信号打断的睡眠。
        if let Some(deadline) = deadline {
            // SAFETY:
            // 1. `timer` 与其上下文指向的 `waker` 均已通过 `pin!` 固定在当前栈帧；
            // 2. 函数任意退出路径在 `timer` 和 `waker` 退栈前必定执行 `timer.cancel_sync()`，等待任何并发定时器回调彻底执行完毕并退出。
            unsafe { timer.as_ref().arm_borrowed(deadline) }
                .expect("fresh precise futex timer rejected");
        }

        waiter.park_signal();
        let outcome = cancel(node.as_ref());
        // Step 5: 被唤醒、超时或收到信号后返回，执行取消仲裁摘除节点；`cancel_sync` 内部等待回调退出时可能使用等待原语，故必须先 drop 本轮唯一活跃的 `waiter`。
        drop(waiter);
        timer.as_ref().cancel_sync();
        return Ok(outcome);
    }
}

/// 在 `wait` 从睡眠返回后仲裁等待结果：若节点已被唤醒方摘除则返回 `Woken`，否则跟随当前桶索引自行摘除节点并判定超时或信号打断。
fn cancel(node: Pin<&Node>) -> WaitOutcome {
    loop {
        if !node.linked.load(Ordering::Acquire) {
            return WaitOutcome::Woken;
        }
        let index = node.bucket.load(Ordering::Acquire);
        let mut bucket = TABLE[index].lock();
        if !node.linked.load(Ordering::Acquire) {
            return WaitOutcome::Woken;
        }
        if node.bucket.load(Ordering::Relaxed) != index {
            // 节点在等待加锁期间被并发 `requeue` 迁移到了新桶，重读原子桶索引重试，无需扫描全部哈希桶。
            continue;
        }

        let outcome = if node
            .deadline
            .is_some_and(|deadline| deadline <= time::now())
        {
            WaitOutcome::TimedOut
        } else {
            WaitOutcome::Interrupted
        };
        let detached = bucket.take(node.get_ref());
        drop(bucket);
        drop(detached);
        return outcome;
    }
}

/// 唤醒在指定用户地址上等待且位掩码与 `mask` 相交的最多 `count` 个任务（兼容旧 ABI：`count == 0` 时仍至少尝试唤醒 1 个匹配项），返回实际选中的任务数。
pub fn wake(
    mm: &Arc<Mm>,
    address: usize,
    private: bool,
    count: usize,
    mask: u32,
) -> Result<usize, FutexError> {
    validate(address, mask)?;
    let mut notifications = Notifications::new();
    loop {
        // Step 1: 解析当前地址的 FutexKey，随后立即释放映射读锁，仅持哈希桶自旋锁扫描匹配节点。
        let (mapping, key) = read_key(mm, address, private)?;
        drop(mapping);
        let needed = {
            let mut bucket = TABLE[key.hash() & (BUCKETS - 1)].lock();
            let quota = count.max(1);
            let needed = if quota > notifications.capacity() {
                bucket.matching(&key, mask, quota)
            } else {
                0
            };
            if needed > notifications.capacity() {
                Some(needed)
            } else {
                let mut pointer = bucket.head;
                for _ in 0..bucket.members {
                    if notifications.count == quota {
                        break;
                    }
                    // SAFETY: 当前持有本桶自旋锁且循环受初始 `bucket.members` 上界约束，`pointer` 指向仍入链保活的有效 `Node`。
                    let node = unsafe { &*pointer };
                    // SAFETY: 在调用 `bucket.take(node)` 清空链表指针并释放节点所有权前先读取 `next`，邻居节点同样受本桶锁保护。
                    pointer = unsafe { (*node.data.get()).next };
                    if Bucket::matches(node, &key, mask) {
                        notifications.push(bucket.take(node));
                    }
                }
                None
            }
        };
        drop(key);
        if let Some(needed) = needed {
            // Step 2: 若匹配数量超出内联通知槽位容量，在所有锁外精确分配堆空间后重新解析最新 key 重试。
            notifications.reserve(needed)?;
            continue;
        }

        // Step 3: 在全部锁外统一触发任务调度唤醒并释放摘出的 FutexKey。
        let selected = notifications.count;
        notifications.notify();
        return Ok(selected);
    }
}

/// 按桶索引升序获取源桶与目的桶的自旋锁，防止并发反向 `requeue` 死锁；若源与目的落在同一桶则只加锁一次，`destination` 传 `None`。
fn with_buckets<R>(
    source: usize,
    destination: usize,
    visit: impl FnOnce(&mut Bucket, Option<&mut Bucket>) -> R,
) -> R {
    match source.cmp(&destination) {
        CmpOrdering::Equal => {
            let mut bucket = TABLE[source].lock();
            visit(&mut bucket, None)
        }
        CmpOrdering::Less => {
            let mut source_bucket = TABLE[source].lock();
            let mut destination_bucket = TABLE[destination].lock();
            visit(&mut source_bucket, Some(&mut destination_bucket))
        }
        CmpOrdering::Greater => {
            let mut destination_bucket = TABLE[destination].lock();
            let mut source_bucket = TABLE[source].lock();
            visit(&mut source_bucket, Some(&mut destination_bucket))
        }
    }
}

/// `requeue` 单轮持锁尝试的执行结果。
enum RequeueAttempt {
    /// 已完成唤醒节点摘除与剩余节点重排，记录总共选中的节点数（唤醒数 + 迁移数）。
    Complete(usize),
    /// 待唤醒节点数超出当前 `Notifications` 容器容量，返回需要在锁外预留的目标容量。
    Reserve(usize),
}

/// 唤醒源地址 `address` 上最多 `wake_count` 个等待者，并将后续最多 `move_count` 个等待者迁移到目的地址 `destination` 对应的哈希桶中。
///
/// 若提供了 `compare`（对应 `FUTEX_CMP_REQUEUE`），则在 `Mm` 读锁、页表锁与有序双桶自旋锁共同保护下校验源地址的当前 32 位值；不匹配或锁外内存预留失败时不会修改任何队列状态。返回实际唤醒与迁移的任务总数。
pub fn requeue(
    mm: &Arc<Mm>,
    address: usize,
    destination: usize,
    private: bool,
    wake_count: usize,
    move_count: usize,
    compare: Option<u32>,
) -> Result<usize, FutexError> {
    validate(address, u32::MAX)?;
    validate(destination, u32::MAX)?;
    let mut notifications = Notifications::new();
    loop {
        // Step 1: 在同一映射读锁下解析源与目的 FutexKey；若需要比较源字且源页尚未驻留，先在锁外触发缺页调入。
        let (mapping, source_key, destination_key) = read_keys(mm, address, destination, private)?;
        if compare.is_some() && mapping.with_word(address, |_| ()).is_none() {
            drop(mapping);
            drop(source_key);
            drop(destination_key);
            mm.fault(address, UserAccess::Read)?;
            continue;
        }

        // Step 2: 按固定桶序加双桶锁（如需比较源字则外层还持有源页表锁），原子执行字比较、唤醒摘除与节点重排。
        let source = source_key.hash() & (BUCKETS - 1);
        let destination_bucket = destination_key.hash() & (BUCKETS - 1);
        let mut operation = |direct: Option<usize>| {
            with_buckets(source, destination_bucket, |source, mut destination| {
                if let Some(expected) = compare {
                    // SAFETY:
                    // 1. 入口 `validate` 已校验 `address` 4 字节对齐且不跨页；
                    // 2. 外层 `mapping.with_word` 持有映射读锁与叶页表锁，保证 `direct` 为可读物理页直映地址；
                    // 3. 使用 `arch::load_user_u32` 原子读取用户字，不构造引用且指针不逃逸。
                    let value = unsafe {
                        arch::load_user_u32(direct.expect("comparison has no word") as *const u32)
                    };
                    if value != expected {
                        return Err(FutexError::Mismatch);
                    }
                }
                if wake_count > notifications.capacity() {
                    let needed = source.matching(&source_key, u32::MAX, wake_count);
                    if needed > notifications.capacity() {
                        return Ok(RequeueAttempt::Reserve(needed));
                    }
                }

                let mut selected = 0;
                let mut pointer = source.head;
                for _ in 0..source.members {
                    if selected == wake_count.saturating_add(move_count) {
                        break;
                    }
                    // SAFETY: `with_buckets` 持有源桶与目的桶自旋锁，保证源桶原链表中的节点在遍历期间均处于保活状态。
                    let node = unsafe { &*pointer };
                    // SAFETY: 在 `take` 或 `move_node` 改写 `node` 的前后向指针之前先保存原源链的 `next`，保证后续遍历沿原源桶链表推进。
                    pointer = unsafe { (*node.data.get()).next };
                    if !Bucket::matches(node, &source_key, u32::MAX) {
                        continue;
                    }
                    if selected < wake_count {
                        notifications.push(source.take(node));
                    } else {
                        source.move_node(
                            node,
                            destination.as_deref_mut(),
                            &destination_key,
                            destination_bucket,
                        );
                    }
                    selected += 1;
                }
                Ok(RequeueAttempt::Complete(selected))
            })
        };

        let result = if compare.is_some() {
            mapping
                .with_word(address, |direct| operation(Some(direct)))
                .expect("stable requeue read lost its resident page")
        } else {
            operation(None)
        };
        drop(mapping);
        drop(source_key);
        drop(destination_key);

        match result? {
            RequeueAttempt::Reserve(needed) => {
                // Step 3a: 内联槽位不足，在释放全部 MM、页表和桶锁后预留精确堆容量并重试整轮解析与比较。
                notifications.reserve(needed)?;
            }
            RequeueAttempt::Complete(selected) => {
                // Step 3b: 在所有锁外通知被摘除的唤醒任务并释放其旧 FutexKey。
                notifications.notify();
                return Ok(selected);
            }
        }
    }
}

/// 内核测试（`ktest`）只读观察指定 Futex 键下当前排队的等待队列摘要快照。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FutexSnapshot {
    /// 当前桶内与目标 `FutexKey` 和位掩码匹配的排队节点总数。
    pub count: usize,
    /// 首个匹配等待节点的 `WaitId`（若存在）。
    pub first_wait_id: Option<WaitId>,
    /// 首个匹配等待节点的超时截止时间（若存在）。
    pub first_deadline: Option<MonotonicInstant>,
}

/// 使用与正式 `wake` 完全一致的 `FutexKey` 解析和位掩码规则，在桶自旋锁下只读采集等待队列快照（供 `ktest` 阻塞检查点断言）。
#[cfg(feature = "ktest")]
pub fn test_snapshot(
    mm: &Arc<Mm>,
    address: usize,
    private: bool,
    mask: u32,
) -> Result<FutexSnapshot, FutexError> {
    validate(address, mask)?;
    let (mapping, key) = read_key(mm, address, private)?;
    drop(mapping);
    let bucket = TABLE[key.hash() & (BUCKETS - 1)].lock();
    let mut snapshot = FutexSnapshot {
        count: 0,
        first_wait_id: None,
        first_deadline: None,
    };
    let mut pointer = bucket.head;
    for _ in 0..bucket.members {
        // SAFETY: 当前持有本桶自旋锁，排除了并发摘除与迁移，`pointer` 指向桶内仍固定在等待任务栈上的有效 `Node`。
        let node = unsafe { &*pointer };
        // SAFETY: 本桶自旋锁保护链表结构，`next` 仅在本次持锁的有限只读循环内使用。
        pointer = unsafe { (*node.data.get()).next };
        if Bucket::matches(node, &key, mask) {
            if snapshot.count == 0 {
                snapshot.first_wait_id = Some(node.id);
                snapshot.first_deadline = node.deadline;
            }
            snapshot.count += 1;
        }
    }
    Ok(snapshot)
}
