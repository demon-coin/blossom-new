//! 非 PI 用户同步按真实 MM、匿名或文件后备登记；固定桶移出通知值后才执行唤醒。

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

const BUCKETS: usize = 32;
const INLINE_NOTIFICATIONS: usize = 64;

/// 核心保留比较、参数与真实用户映射错误，ABI 层决定 errno。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FutexError {
    Mismatch,
    InvalidArgument,
    Vm(VmError),
}

impl From<VmError> for FutexError {
    /// 不丢失实际用户访问失败原因。
    fn from(error: VmError) -> Self {
        Self::Vm(error)
    }
}

/// 被桶实际选中优先于同时到期或信号；提示通知本身不是业务完成。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitOutcome {
    Woken,
    TimedOut,
    Interrupted,
}

struct NodeData {
    previous: *const Node,
    next: *const Node,
    key: Option<FutexKey>,
    waker: Option<Waker>,
}

/// 节点只在当前桶锁下访问；当前位置供唯一取消者重试迁移竞争，不扫描其他桶。
struct Node {
    data: UnsafeCell<NodeData>,
    bucket: AtomicUsize,
    linked: AtomicBool,
    mask: u32,
    #[cfg(feature = "ktest")]
    id: WaitId,
    deadline: Option<MonotonicInstant>,
    _pinned: PhantomPinned,
}

impl Node {
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
    fn drop(&mut self) {
        assert!(
            !self.linked.load(Ordering::Acquire),
            "dropping a registered futex node"
        );
    }
}

/// 值拥有的摘离结果不携带栈节点地址；key 的最后 put 与 Weak 析构均在桶锁外。
struct Detached {
    key: FutexKey,
    waker: Waker,
}

struct Bucket {
    head: *const Node,
    tail: *const Node,
    members: usize,
}

// SAFETY: 静态桶锁串行链和节点数据访问；wait 的固定栈存储在唯一取消仲裁结束前保活，
// 唤醒者以 Release 解除链接作为最后节点访问，等待者 Acquire 后才可退栈。
unsafe impl Send for Bucket {}

impl Bucket {
    const fn new() -> Self {
        Self {
            head: ptr::null(),
            tail: ptr::null(),
            members: 0,
        }
    }

    fn insert(&mut self, node: Pin<&Node>) {
        assert!(!node.linked.load(Ordering::Relaxed));
        self.push(node.get_ref());
        node.linked.store(true, Ordering::Relaxed);
    }

    fn push(&mut self, node: &Node) {
        let members = self
            .members
            .checked_add(1)
            .expect("futex member count overflow");
        // SAFETY: 当前桶锁保护链接，wait 固定并保活新节点，已有 tail 在同一桶仍链接；
        // 重排调用点同时持源和目的锁，源链已摘离但未解除节点存活责任。
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

    fn unlink(&mut self, node: &Node) {
        // SAFETY: 调用点只传本桶仍链接的节点，本桶锁保护节点和仍登记的前后邻居。
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

    fn matches(node: &Node, key: &FutexKey, mask: u32) -> bool {
        // SAFETY: 所有调用点持当前桶锁，取消和迁移不能改变仍链接节点的 key。
        let data = unsafe { &*node.data.get() };
        node.mask & mask != 0
            && data
                .key
                .as_ref()
                .expect("futex node has no key")
                .matches(key)
    }

    fn matching(&self, key: &FutexKey, mask: u32, limit: usize) -> usize {
        let mut pointer = self.head;
        let mut matching = 0;
        for _ in 0..self.members {
            if matching == limit {
                break;
            }
            // SAFETY: 当前桶锁保住链中固定节点和 next；计数遍历不修改任何节点。
            let node = unsafe { &*pointer };
            if Self::matches(node, key, mask) {
                matching += 1;
            }
            // SAFETY: 当前桶锁保护节点链接，next 不越过解锁边界。
            pointer = unsafe { (*node.data.get()).next };
        }
        matching
    }

    fn take(&mut self, node: &Node) -> Detached {
        self.unlink(node);
        // SAFETY: 本桶锁独占修改 data，移出全部拥有值后不再读取节点内容。
        let detached = unsafe {
            let data = &mut *node.data.get();
            Detached {
                key: data.key.take().expect("futex node has no key"),
                waker: data.waker.take().expect("futex node has no waker"),
            }
        };
        // 此 Release 必须是摘离者对节点的最后一次访问；后续只使用移出的拥有值。
        node.linked.store(false, Ordering::Release);
        detached
    }

    fn move_node(
        &mut self,
        node: &Node,
        destination: Option<&mut Bucket>,
        key: &FutexKey,
        bucket: usize,
    ) {
        // SAFETY: 双桶锁排除取消和迁移；调用者的 source key 保活旧 owner，
        // 替换的旧 key 因此不可能在桶锁内成为最后 put，WaitId、Waker 和期限不变。
        let old_key = unsafe { (*node.data.get()).key.replace(key.clone()) };
        drop(old_key);
        if let Some(destination) = destination {
            self.unlink(node);
            destination.push(node);
        }
        node.bucket.store(bucket, Ordering::Release);
    }
}

// ponytail: 固定桶内线性遍历真实活登记；出现桶争用热点后再增加桶数，不增加等待者上限。
static TABLE: [SpinLock<Bucket>; BUCKETS] = [const { SpinLock::new(Bucket::new()) }; BUCKETS];

/// 常见唤醒批量不分配；超出内联值容量时，先在所有 MM/PTE/桶锁外准备实际所需空间。
struct Notifications {
    inline: [Option<Detached>; INLINE_NOTIFICATIONS],
    count: usize,
    spill: Vec<Detached>,
}

impl Notifications {
    fn new() -> Self {
        Self {
            inline: [const { None }; INLINE_NOTIFICATIONS],
            count: 0,
            spill: Vec::new(),
        }
    }

    fn capacity(&self) -> usize {
        INLINE_NOTIFICATIONS + self.spill.capacity()
    }

    fn reserve(&mut self, needed: usize) -> Result<(), FutexError> {
        self.spill
            .try_reserve_exact(needed.saturating_sub(INLINE_NOTIFICATIONS))
            .map_err(|_| FutexError::Vm(VmError::NoMemory))
    }

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

    fn notify(&mut self) {
        for detached in self
            .inline
            .iter_mut()
            .filter_map(Option::take)
            .chain(self.spill.drain(..))
        {
            // 选择并摘链已经消耗配额，不以控制通知是否已置位来改返回数量。
            detached.waker.notify();
            drop(detached.key);
        }
    }
}

/// 验证自然对齐的 futex 字宽及完整用户范围，零 bitset 不登记等待。
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

fn timeout_notify(timer: Pin<&Timer>) {
    // SAFETY: wait 固定 Waker 存储，退出时先结束唯一 Waiter 再 cancel_sync 确认回调尾部；
    // IRQ 回调只共享读取 Waker，不读取 futex 节点或桶，也不保存该指针。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// 在原 mmap 读侧取得实际后备身份，缺页或私有文件 COW 先退出读侧再准备。
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

/// 两个重排身份来自同一 mmap 读侧，目的页准备结束后重新解析两端。
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

/// 与当前字的比较和登记在 MM→PTE→bucket 内完成；被选择即成功，不要求用户字改变。
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
        // mmap 读侧可能用自己的 Waiter 等待，必须先取得它，再建立本轮唯一 futex Waiter。
        let (mapping, key) = read_key(mm, address, private)?;
        if mapping.with_word(address, |_| ()).is_none() {
            drop(mapping);
            drop(key);
            mm.fault(address, UserAccess::Read)?;
            continue;
        }

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

        let index = key.hash() & (BUCKETS - 1);
        let registration = mapping
            .with_word(address, |direct| {
                let mut bucket = TABLE[index].lock();
                // SAFETY: 入口检查四字节自然对齐和用户范围，mmap/PTE 保活可读正常 RAM；
                // 字位于单页且只经原始 Acquire 指令访问，不构造用户内容引用。
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

        if let Some(deadline) = deadline {
            // SAFETY: timer、Waker 与节点地址固定；唯一取消先撤桶登记，结束 Waiter 后
            // 同步停止确认正在执行的真实回调尾部，之前不允许任一存储退栈。
            unsafe { timer.as_ref().arm_borrowed(deadline) }
                .expect("fresh precise futex timer rejected");
        }

        waiter.park_signal();
        let outcome = cancel(node.as_ref());
        // 停 timer 必要时使用原等待系统，故先结束本轮唯一 active wait。
        drop(waiter);
        timer.as_ref().cancel_sync();
        return Ok(outcome);
    }
}

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
            // 只在真正重排竞争时重读当前位置；不扫描其他桶、不建立新等待身份。
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

/// 从真实当前 key 中摘离 bitset 相交的登记；旧 ABI 零配额仍可选择首个匹配项。
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
                    // SAFETY: 当前桶锁保住仍链接节点；先取 next，再摘离，Release 后不再访问节点。
                    let node = unsafe { &*pointer };
                    // SAFETY: next 只在本桶锁内使用，邻居仍由各自的 wait 存储保活。
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
            // 空通知容器在所有锁外准备；下一轮重新解析真实 key 和实际所需空间。
            notifications.reserve(needed)?;
            continue;
        }

        let selected = notifications.count;
        notifications.notify();
        return Ok(selected);
    }
}

/// 按桶索引固定加锁顺序；同桶只取一次，回调返回前保持相关 guard。
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

enum RequeueAttempt {
    Complete(usize),
    Reserve(usize),
}

/// 固定索引顺序取得双桶；可比较源字，移动原节点并保留原 WaitId、期限和取消责任。
/// 非 PI 同 key 重排合法；返回实际选中唤醒与迁移的总数，通知空间准备失败不提交变化。
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
        let (mapping, source_key, destination_key) = read_keys(mm, address, destination, private)?;
        if compare.is_some() && mapping.with_word(address, |_| ()).is_none() {
            drop(mapping);
            drop(source_key);
            drop(destination_key);
            mm.fault(address, UserAccess::Read)?;
            continue;
        }

        let source = source_key.hash() & (BUCKETS - 1);
        let destination_bucket = destination_key.hash() & (BUCKETS - 1);
        let mut operation = |direct: Option<usize>| {
            with_buckets(source, destination_bucket, |source, mut destination| {
                if let Some(expected) = compare {
                    // SAFETY: 入口检查自然对齐，当前 mmap/PTE 保活正常 RAM；
                    // 固定顺序双桶内一次原始 Acquire 读取，不构造用户引用或逃逸地址。
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
                    // SAFETY: 固定序双桶锁保住源链；next 在摘离前读取，不携节点跨解锁。
                    let node = unsafe { &*pointer };
                    // SAFETY: 当前源锁保护 next，迁移后也只沿原源链继续本次有限遍历。
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
                // spill 只含拥有值，锁外 fallible 准备后重查 key、源字和实际登记。
                notifications.reserve(needed)?;
            }
            RequeueAttempt::Complete(selected) => {
                // 最后 key put、Weak 释放和调度通知均在 MM/PTE/桶锁外。
                notifications.notify();
                return Ok(selected);
            }
        }
    }
}

/// 只读当前实际桶内匹配登记的有限快照，不另存状态、分配容器或执行唤醒。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FutexSnapshot {
    pub count: usize,
    pub first_wait_id: Option<WaitId>,
    pub first_deadline: Option<MonotonicInstant>,
}

/// 用与生产唤醒相同的 key 和 bitset 筛选真实登记，供用户阻塞检查点读取。
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
        // SAFETY: 当前桶锁保住实际固定节点，迁移和取消不能在本次只读观察期间摘离。
        let node = unsafe { &*pointer };
        // SAFETY: next 只在当前桶锁内用于有限只读遍历，不越过解锁边界。
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
