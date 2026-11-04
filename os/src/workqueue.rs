//! 固定身份的可睡工作、全域准入和两颜色有限排空。

pub(crate) mod delayed;
mod pool;
#[cfg(feature = "ktest")]
pub use delayed::DelayedError;
pub use delayed::DelayedWork;
pub(crate) use pool::{Worker, WorkerPool};

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::AllocError;
use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::pin::Pin;
use core::ptr;
#[cfg(feature = "ktest")]
use core::sync::atomic::AtomicUsize;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::cpu;
use crate::sync::mutex::{SleepMutex, SleepMutexGuard};
use crate::sync::SpinLock;
use crate::task::{JoinHandle, PreparedTask};
use crate::wait::{WaitNode, WaitQueue, Waiter};

// ponytail: 全域短元数据锁限制并行提交吞吐；仅在实际测得竞争后拆分域与池锁。
pub(super) static STATE_LOCK: SpinLock<()> = SpinLock::new(());
static INITIALIZED: AtomicBool = AtomicBool::new(false);
// flush 串行排空旧色后复用另一色，颜色只占一个选择位。
const FLUSH_COLOR_BITS: usize = 1;
const FLUSH_COLORS: usize = 1 << FLUSH_COLOR_BITS;
const FLUSH_COLOR_MASK: usize = FLUSH_COLORS - 1;
#[cfg(feature = "ktest")]
static LIVE_DOMAINS: AtomicUsize = AtomicUsize::new(0);

/// 普通域采用全域并发上限；严格有序域固定 CPU0 且只准入一个回调。
#[derive(Clone, Copy)]
pub struct WorkQueueConfig {
    pub max_active: usize,
    pub ordered: bool,
    pub reclaim: bool,
}

/// 永久关闭拒绝提交，已有 pending 或同步取消期间则返回未新增票。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueueError {
    Closed,
}

pub(super) struct DomainState {
    closed: bool,
    published: bool,
    color: usize,
    counts: [usize; FLUSH_COLORS],
    active: usize,
    reserved_active: usize,
    reserved_workers: usize,
    rescue_stop: bool,
}

pub(super) struct Rescue {
    worker: Arc<Worker>,
    join: UnsafeCell<Option<JoinHandle>>,
}

pub(super) struct Domain {
    config: WorkQueueConfig,
    state: UnsafeCell<DomainState>,
    flush_serial: SleepMutex<()>,
    close_serial: SleepMutex<()>,
    finished: WaitQueue,
    rescuers: UnsafeCell<Vec<Rescue>>,
}

// SAFETY: 所有可变元数据经 STATE_LOCK；rescuer 表仅在发表前构造，关闭者串行取出 join。
unsafe impl Send for Domain {}
// SAFETY: 状态和表的访问遵循同一锁，等待源内部自行同步。
unsafe impl Sync for Domain {}

/// 域的唯一显式停止 owner；工作票和预备救援任务另持稳定域引用。
pub struct WorkQueue {
    domain: Arc<Domain>,
}

impl WorkQueue {
    /// 先准备全部救援执行栈，再发表域；任何准备失败只析构未发表资源。
    pub fn try_new(config: WorkQueueConfig) -> Result<Self, AllocError> {
        assert!(
            0 < config.max_active,
            "work domain needs positive admission limit"
        );
        assert!(
            INITIALIZED.load(Ordering::Acquire),
            "work services not initialized"
        );
        let value = Domain {
            config,
            state: UnsafeCell::new(DomainState {
                closed: false,
                published: false,
                color: 0,
                counts: [0; FLUSH_COLORS],
                active: 0,
                reserved_active: 0,
                reserved_workers: 0,
                rescue_stop: false,
            }),
            flush_serial: SleepMutex::new(()),
            close_serial: SleepMutex::new(()),
            finished: WaitQueue::new(),
            rescuers: UnsafeCell::new(Vec::new()),
        };
        // 在 fallible Arc 分配前计入完整 value；分配失败的普通 Drop 同样归还。
        #[cfg(feature = "ktest")]
        LIVE_DOMAINS.fetch_add(1, Ordering::Relaxed);
        let domain = Arc::try_new(value)?;
        let count = if config.ordered { 1 } else { cpu::count() };
        let mut prepared: Vec<(Arc<Worker>, PreparedTask)> = Vec::new();
        if config.reclaim {
            prepared.try_reserve_exact(count).map_err(|_| AllocError)?;
            // SAFETY: 域尚未发表，此处是表的唯一构造者。
            unsafe {
                let rescuers = &mut *domain.rescuers.get();
                rescuers.try_reserve_exact(count).map_err(|_| AllocError)?;
            }
            for cpu in 0..count {
                prepared.push(pool::prepare_rescuer(cpu, &domain)?);
            }
        }
        // SAFETY: 所有 Task 尚未发表；表容量已准备，插入不分配。
        unsafe {
            let rescues = &mut *domain.rescuers.get();
            for (worker, task) in prepared {
                rescues.push(Rescue {
                    worker,
                    join: UnsafeCell::new(Some(task.publish())),
                });
            }
        }
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 短状态锁保护发表位。
            unsafe {
                (*domain.state.get()).published = true;
            }
        }
        Ok(Self { domain })
    }

    /// 串行切换颜色，只等待入口颜色中的每一份真实票，不追随未来提交。
    pub fn flush(&self) {
        pool::assert_wait_allowed(&self.domain, ptr::null());
        self.domain.flush();
    }

    /// 永久拒绝新票，排空后归还预留执行栈，再停止并真实 join 本域救援任务。
    pub fn close_sync(&self) {
        pool::assert_wait_allowed(&self.domain, ptr::null());
        let _serial = self.domain.close_serial.lock();
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 所有域状态变更由 STATE_LOCK 串行。
            let state = unsafe { &mut *self.domain.state.get() };
            assert!(!state.closed, "work domain closed twice");
            state.closed = true;
        }
        self.domain.flush();
        pool::release_domain_reservations(&self.domain);
        wait_until(&self.domain.finished, || {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 原 manager 只在真实 join 返回后于同锁扣除此计数。
            unsafe { (*self.domain.state.get()).reserved_workers == 0 }
        });
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 短锁保护 rescuer 的退出谓词。
            unsafe {
                (*self.domain.state.get()).rescue_stop = true;
            }
        }
        // SAFETY: 表发表后不可变；join 自身有 UnsafeCell，由唯一 close_serial owner 取出。
        let rescues = unsafe { &*self.domain.rescuers.get() };
        for rescue in rescues.iter() {
            rescue.worker.wake();
        }
        for rescue in rescues.iter() {
            // SAFETY: close_serial 串行唯一关闭者，其他路径只借用不变 worker，不访问 join。
            let join = unsafe {
                let slot = &mut *rescue.join.get();
                slot.take().expect("missing rescuer join")
            };
            assert_eq!(join.join(), 0);
        }
    }

    /// 读取原域尚未经 manager 真实 join 的预留任务数，不把无票误当离栈。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_reserved_workers(&self) -> usize {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 原域的登记和 manager 真实 join 归账均由同一状态锁串行。
        unsafe { (*self.domain.state.get()).reserved_workers }
    }
}

impl Drop for WorkQueue {
    /// owner 不隐式等待，调用者必须先停止外部生产源并显式关闭。
    fn drop(&mut self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 锁下核对 owner 的停止责任。
        assert!(
            unsafe { (*self.domain.state.get()).closed },
            "work domain owner dropped before close"
        );
    }
}

impl Domain {
    /// 普通 running 睡眠仍占 active；预留 Work 已有独占执行栈，不借普通额度。
    pub(super) unsafe fn eligible(&self, work: &Work) -> bool {
        // SAFETY: 调用者持 STATE_LOCK。
        let state = unsafe { &*self.state.get() };
        work.reserved_worker.is_some()
            || state.active
                < if self.config.ordered {
                    1
                } else {
                    self.config.max_active
                }
    }

    /// 单一 flush owner 等颜色完全归零后才允许复用它。
    fn flush(&self) {
        let _serial = self.flush_serial.lock();
        let old = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 颜色与票数始终在同一锁下变更。
            let state = unsafe { &mut *self.state.get() };
            let old = state.color;
            assert_eq!(
                state.counts[old ^ FLUSH_COLOR_MASK],
                0,
                "flush reused a live color"
            );
            state.color ^= FLUSH_COLOR_MASK;
            old
        };
        wait_until(&self.finished, || {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 锁下检查颜色票计数。
            unsafe { (*self.state.get()).counts[old] == 0 }
        });
    }
}

impl Drop for Domain {
    /// 未发表失败对象可以自然释放；已发表域必须无活票且全部救援者已 join。
    fn drop(&mut self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 最后引用持有者独占对象，仍采用统一锁核对状态。
        let state = unsafe { &*self.state.get() };
        assert_eq!(state.counts, [0; FLUSH_COLORS]);
        assert_eq!(state.active, 0);
        assert_eq!(state.reserved_active, 0);
        assert_eq!(state.reserved_workers, 0);
        assert!(!state.published || state.closed);
        #[cfg(feature = "ktest")]
        LIVE_DOMAINS.fetch_sub(1, Ordering::Relaxed);
    }
}

pub(super) struct WorkState {
    next: *const Work,
    previous: *const Work,
    queued: u64,
    running: u64,
    next_ticket: u64,
    order: u64,
    color: usize,
    running_color: usize,
    worker: *const Worker,
    assigned: *const Worker,
    closed: bool,
    stopping: bool,
    delayed_owner: *const DelayedWork,
}

/// 地址稳定、固定域与 CPU 的 owned 工作；queued 和 running 各自拥有一份真实强引用。
pub struct Work {
    domain: Arc<Domain>,
    cpu: usize,
    callback: fn(Pin<&Work>),
    context: usize,
    owned_raw: *const Work,
    reserved_worker: Option<Arc<Worker>>,
    one_shot: bool,
    state: UnsafeCell<WorkState>,
    stop_serial: SleepMutex<()>,
    finished: WaitQueue,
    _pinned: PhantomPinned,
}

// SAFETY: 地址不移动，所有可变状态经 STATE_LOCK；回调只由唯一 claim 执行者调用。
unsafe impl Send for Work {}
// SAFETY: 共享元数据受统一锁保护，函数和上下文值固定。
unsafe impl Sync for Work {}

/// 固定当前两份票的有限等待边界，零表示入口没有该份票。
#[derive(Clone, Copy)]
pub(crate) struct WorkBoundary {
    running: u64,
    queued: u64,
}

pub(super) struct RemovedTicket {
    work: *const Work,
    delayed: *const DelayedWork,
    worker: Option<Arc<Worker>>,
}

impl Drop for RemovedTicket {
    /// 取消与完成先撤状态，真实拥有票在所有元数据锁外释放。
    fn drop(&mut self) {
        // SAFETY: 此拥有票保活 work 直到下面恢复并释放 raw；所有通知在元数据锁外。
        let work = unsafe { &*self.work };
        work.finished.wake_all();
        work.domain.finished.wake_all();
        for cpu in 0..cpu::count() {
            pool::notify(cpu);
        }
        if let Some(worker) = &self.worker {
            worker.wake();
        }
        // SAFETY: work 来自构造期保存的 into_raw 指针且对应一份增加过的异步强引用。
        unsafe {
            drop(Arc::from_raw(self.work));
        }
        if !self.delayed.is_null() {
            // SAFETY: 专用 D claim/cancel 移交唯一真实 into_raw 拥有票。
            unsafe {
                drop(Arc::from_raw(self.delayed));
            }
        }
    }
}

impl Work {
    /// 构造稳定拥有节点，严格有序域规范所有提交至 CPU0。
    pub fn try_new(
        queue: &WorkQueue,
        cpu: usize,
        callback: fn(Pin<&Work>),
        context: usize,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        assert!(cpu < cpu::count(), "work CPU is out of range");
        let cpu = if queue.domain.config.ordered { 0 } else { cpu };
        Self::prepare(
            Arc::clone(&queue.domain),
            cpu,
            callback,
            context,
            false,
            false,
        )
    }

    /// 块控制或重型完成在构造期预留一份真实执行栈，不与未知可睡回调竞争额度。
    /// 每个 Work 仅由自己的执行者领取，普通域/ordered/reclaim 接口保持原语义。
    pub(crate) fn try_new_reserved(
        queue: &WorkQueue,
        cpu: usize,
        callback: fn(Pin<&Work>),
        context: usize,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        assert!(cpu < cpu::count(), "work CPU is out of range");
        Self::prepare(
            Arc::clone(&queue.domain),
            cpu,
            callback,
            context,
            true,
            false,
        )
    }

    /// 一次性 Bio 完成克隆原域和 CPU，另备独占栈；回调返回即归还预留，不新建域 owner。
    pub(crate) fn try_new_related_reserved(
        &self,
        callback: fn(Pin<&Work>),
        context: usize,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        Self::prepare(
            Arc::clone(&self.domain),
            self.cpu,
            callback,
            context,
            true,
            true,
        )
    }

    /// 克隆该 Work 原预留执行者的弱任务观察；尚未进入服务或已析构时为空。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_reserved_task(&self) -> Option<Arc<crate::task::Task>> {
        self.reserved_worker
            .as_ref()
            .and_then(|worker| worker.ktest_task())
    }

    /// 原预留执行者只在其真实 JoinHandle 返回后标记 joined。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_reserved_joined(&self) -> bool {
        self.reserved_worker
            .as_ref()
            .is_some_and(|worker| worker.ktest_joined())
    }

    /// 锁外完成全部 fallible 构造；预留执行者只在 Work 地址和关联写定后发表。
    fn prepare(
        domain: Arc<Domain>,
        cpu: usize,
        callback: fn(Pin<&Work>),
        context: usize,
        reserved: bool,
        one_shot: bool,
    ) -> Result<Pin<Arc<Self>>, AllocError> {
        assert!(
            !reserved || !domain.config.ordered,
            "ordered work cannot reserve parallel execution"
        );
        let raw = Arc::into_raw(Arc::try_new(Self {
            domain,
            cpu,
            callback,
            context,
            owned_raw: ptr::null(),
            reserved_worker: None,
            one_shot,
            state: UnsafeCell::new(WorkState {
                next: ptr::null(),
                previous: ptr::null(),
                queued: 0,
                running: 0,
                next_ticket: 0,
                order: 0,
                color: 0,
                running_color: 0,
                worker: ptr::null(),
                assigned: ptr::null(),
                closed: false,
                stopping: false,
                delayed_owner: ptr::null(),
            }),
            stop_serial: SleepMutex::new(()),
            finished: WaitQueue::new(),
            _pinned: PhantomPinned,
        })?);
        // SAFETY: 未发表且唯一的 into_raw 强引用允许补入原控制块指针，并立即恢复固定拥有。
        let owner = unsafe {
            (*raw.cast_mut()).owned_raw = raw;
            Pin::new_unchecked(Arc::from_raw(raw))
        };
        if reserved {
            // Task 原生准备包含 order4/16页栈；失败尚未发表执行者，原 Work 普通析构。
            let (worker, prepared) = pool::prepare_reserved(cpu, &owner.domain, raw)?;
            // SAFETY: 无票且尚无异步执行者；原固定分配内只初始化一次关联，不移动 Work。
            unsafe {
                (*raw.cast_mut()).reserved_worker = Some(Arc::clone(&worker));
            }
            pool::publish_worker(worker, prepared);
        }
        Ok(owner)
    }

    /// delayed 在捕获边界期间与同步停止串行；先拒自等，再取得可睡锁。
    pub(super) fn lock_stop_serial(self: Pin<&Self>) -> SleepMutexGuard<'_, ()> {
        pool::assert_wait_allowed(&self.domain, self.owned_raw);
        self.get_ref().stop_serial.lock()
    }

    /// 返回构造时的标量上下文。
    pub fn context(&self) -> usize {
        self.context
    }
    /// 返回实际执行和 delayed timer 都必须使用的固定 CPU。
    pub fn cpu(&self) -> usize {
        self.cpu
    }

    /// 接受一个新 pending 票；重复提交和同步取消期间不新增票。
    pub fn queue(self: Pin<&Self>) -> Result<bool, QueueError> {
        let added = {
            let _lock = STATE_LOCK.lock();
            self.queue_locked()
        }?;
        if added {
            self.notify();
        }
        Ok(added)
    }

    /// 锁下只检查永久关闭，不把已有 pending 误当同步停止。
    pub(super) fn is_closed_locked(self: Pin<&Self>) -> bool {
        // SAFETY: 调用者持 STATE_LOCK，域与 Work 使用同一锁。
        unsafe { (*self.state.get()).closed || (*self.domain.state.get()).closed }
    }

    /// 锁下判断提交门是否开放，不含单 pending 重复判断。
    pub(super) fn accepts_locked(self: Pin<&Self>) -> bool {
        // SAFETY: 调用者持 STATE_LOCK。
        !self.is_closed_locked() && !unsafe { (*self.state.get()).stopping }
    }

    /// 锁下返回普通 queue 的完整准入结论，供 delayed 共用唯一停止事实。
    pub(super) fn can_queue_locked(self: Pin<&Self>) -> Result<bool, QueueError> {
        if self.is_closed_locked() {
            return Err(QueueError::Closed);
        }
        // SAFETY: 调用者持 STATE_LOCK。
        Ok(unsafe { !(*self.state.get()).stopping && (*self.state.get()).queued == 0 })
    }

    /// 调用者持 STATE_LOCK；增加真实拥有票、颜色计数和永久池的 intrusive 链，不分配或通知。
    pub(super) fn queue_locked(self: Pin<&Self>) -> Result<bool, QueueError> {
        if !self.can_queue_locked()? {
            return Ok(false);
        }
        // SAFETY: self 来自原 into_raw 稳定存储；统一状态锁保护所有链与票字段。
        unsafe {
            let state = &mut *self.state.get();
            let domain = &mut *self.domain.state.get();
            state.next_ticket = state
                .next_ticket
                .checked_add(1)
                .expect("work ticket exhausted");
            state.queued = state.next_ticket;
            state.color = domain.color;
            state.assigned = if state.worker.is_null() {
                self.reserved_worker
                    .as_ref()
                    .map_or(ptr::null(), Arc::as_ptr)
            } else {
                state.worker
            };
            domain.counts[state.color] = domain.counts[state.color]
                .checked_add(1)
                .expect("work color count exhausted");
            Arc::increment_strong_count(self.owned_raw);
            pool::enqueue(self.cpu, self.owned_raw);
        }
        Ok(true)
    }

    /// 只在构造期未发表时建立 DelayedWork 专用拥有票关联。
    pub(super) fn set_delayed_owner(self: Pin<&Self>, owner: *const DelayedWork) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 调用者保证 owner 未发表，关联一旦写入不再改变。
        unsafe {
            let state = &mut *self.state.get();
            assert!(state.delayed_owner.is_null());
            assert_eq!(state.queued, 0);
            assert_eq!(state.running, 0);
            state.delayed_owner = owner;
        }
    }

    /// 锁外触发实际 idle、manager 或 OOM 救援者，不通过周期扫描补进度。
    pub(super) fn notify(self: Pin<&Self>) {
        pool::notify(self.cpu);
    }

    /// 锁下撤销唯一 pending 票，移出的真实拥有票必须在锁外析构。
    pub(super) fn cancel_queued_locked(self: Pin<&Self>) -> Option<RemovedTicket> {
        // SAFETY: 调用者持 STATE_LOCK；链摘除与颜色扣数不可被 claim 分割。
        unsafe {
            let queued = (*self.state.get()).queued;
            if queued == 0 {
                return None;
            }
            let worker =
                pool::assigned_worker(self.cpu, (*self.state.get()).assigned, &self.domain);
            pool::unlink(self.cpu, self.owned_raw);
            // unlink 已结束对同一 WorkState 的独占借用，此处才重新借用。
            let state = &mut *self.state.get();
            state.queued = 0;
            state.assigned = ptr::null();
            (*self.domain.state.get()).counts[state.color] -= 1;
            let delayed = if state.delayed_owner.is_null() {
                ptr::null()
            } else {
                Pin::new_unchecked(&*state.delayed_owner).cancel_locked()
            };
            Some(RemovedTicket {
                work: self.owned_raw,
                delayed,
                worker,
            })
        }
    }

    /// 锁下固定当前 running 和 queued 两份票，不捕获边界后的再次提交。
    pub(super) fn capture_locked(self: Pin<&Self>) -> WorkBoundary {
        // SAFETY: 调用者持 STATE_LOCK。
        unsafe {
            let state = &*self.state.get();
            WorkBoundary {
                running: state.running,
                queued: state.queued,
            }
        }
    }

    /// 在业务锁内固定当前两份票，不等待回调；等待前须释放回调依赖的锁。
    pub(crate) fn capture_boundary(self: Pin<&Self>) -> WorkBoundary {
        let _lock = STATE_LOCK.lock();
        self.capture_locked()
    }

    /// 等待同一 Work 已捕获的票消失；须保持对象存活并释放回调依赖的锁，不追随后来提交。
    pub(crate) fn flush_boundary(self: Pin<&Self>, boundary: WorkBoundary) -> bool {
        pool::assert_wait_allowed(&self.domain, self.owned_raw);
        let affected = boundary.running != 0 || boundary.queued != 0;
        wait_until(&self.finished, || {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 锁内观察票身份，不以最大完成序号推断前缀。
            let state = unsafe { &*self.state.get() };
            let matches = |ticket: u64| {
                ticket != 0 && (ticket == boundary.running || ticket == boundary.queued)
            };
            !matches(state.running) && !matches(state.queued)
        });
        affected
    }

    /// 等待调用边界内存在的票，不追逐 callback 的未来 self-requeue。
    pub fn flush(self: Pin<&Self>) -> bool {
        let boundary = self.capture_boundary();
        self.flush_boundary(boundary)
    }

    /// 串行暂拒提交或永久关闭，并撤 pending；delayed 可以在 finish 前真实停止 timer。
    pub(super) fn begin_stop(self: Pin<&Self>, close: bool) -> StopGuard<'_> {
        pool::assert_wait_allowed(&self.domain, self.owned_raw);
        let serial = self.get_ref().stop_serial.lock();
        let (affected, removed) = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 锁下设置唯一提交门并固定此次 stop 的真实业务事实。
            let state = unsafe { &mut *self.state.get() };
            if close {
                assert!(!state.closed, "work closed twice");
                state.closed = true;
            }
            state.stopping = true;
            // SAFETY: 构造期关联固定，调用此 inner Work 的 delayed owner 或运行票保活 D；
            // 同一状态锁内捕获 Delay 事实，不能被 transfer 在拒提交后清票而漏报。
            let delayed = !state.delayed_owner.is_null()
                && unsafe { Pin::new_unchecked(&*state.delayed_owner).has_pending_locked() };
            let affected = state.queued != 0 || state.running != 0 || delayed;
            (affected, self.cancel_queued_locked())
        };
        drop(removed);
        self.finished.wake_all();
        self.domain.finished.wake_all();
        self.notify();
        StopGuard {
            work: self,
            serial: Some(serial),
            affected,
            finished: false,
        }
    }

    /// 取消当前 pending、等待真实 running 尾部，返回后允许同一 Work 重新使用。
    pub fn cancel_sync(self: Pin<&Self>) -> bool {
        self.begin_stop(false).finish()
    }
    /// 唯一 owner 永久关闭 Work；预留执行者交原 manager 真实 join 后才返回。
    pub fn close_sync(self: Pin<&Self>) -> bool {
        let affected = self.begin_stop(true).finish();
        if let Some(worker) = &self.reserved_worker {
            pool::close_reservation(worker);
        }
        affected
    }
}

pub(super) struct StopGuard<'a> {
    work: Pin<&'a Work>,
    serial: Option<SleepMutexGuard<'a, ()>>,
    affected: bool,
    finished: bool,
}

impl StopGuard<'_> {
    /// 外部 delayed timer 已同步停止后，等待唯一 callback 真实尾部并恢复非关闭的提交门。
    pub(super) fn finish(mut self) -> bool {
        wait_until(&self.work.finished, || {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 停止门已阻止所有新票，检查真实执行尾部。
            unsafe { (*self.work.state.get()).running == 0 }
        });
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 唯一 stop_serial owner 结束暂拒状态，closed 字段保持不变。
            unsafe {
                (*self.work.state.get()).stopping = false;
            }
        }
        self.finished = true;
        drop(self.serial.take());
        self.affected
    }
}

impl Drop for StopGuard<'_> {
    /// 分离式停止必须完成真实尾部，不以 guard 析构做隐式同步等待。
    fn drop(&mut self) {
        assert!(self.finished, "unfinished work stop guard");
    }
}

impl Drop for Work {
    /// 最后拥有者只核无票并锁外释放预留，不隐式等待或重复同步停止。
    fn drop(&mut self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 最后引用不能与任何拥有票并存；仍核对链状态。
        let state = unsafe { &*self.state.get() };
        assert_eq!(state.queued, 0, "dropping pending work");
        assert_eq!(state.running, 0, "dropping running work");
        drop(_lock);
        if let Some(worker) = &self.reserved_worker {
            pool::release_reservation(worker);
        }
    }
}

/// 永久源登记后复查业务谓词；每一轮只保留一个 active wait，通知不能代替条件。
pub(super) fn wait_until(source: &WaitQueue, mut done: impl FnMut() -> bool) {
    if done() {
        return;
    }
    loop {
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: source 借用和固定栈 node 均存活到 registration 先析构，不遗忘登记。
        let registration = unsafe { source.register(node.as_mut()) };
        if done() {
            return;
        }
        waiter.park();
        drop(registration);
        drop(waiter);
        if done() {
            return;
        }
    }
}

/// BSP 在 MM 就绪而 IRQ/AP 未启动时，先准备全部 pool 基础执行者和 manager 再发表。
pub fn initialize_services() -> Result<(), AllocError> {
    assert_eq!(cpu::current().id, 0);
    assert!(
        !INITIALIZED.load(Ordering::Acquire),
        "work services initialized twice"
    );
    pool::initialize()?;
    INITIALIZED.store(true, Ordering::Release);
    Ok(())
}

#[cfg(feature = "ktest")]
pub(crate) mod testing {
    pub(crate) use super::pool::testing::{
        arm_pause, cancel_pause, is_paused, is_rescuer, manager_blocked, pool_stats, release_pause,
        wait_services_blocked, PausePoint,
    };
    use super::*;
    /// 当前仍存活的真实 payload 数；不代表 Weak 控制块或 allocator backing 已归还。
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct MetadataStats {
        pub tasks: usize,
        pub workers: usize,
        pub domains: usize,
    }
    /// 在任务与服务已停稳后读取构造/最后强拥有析构计数，clone 不增加计数。
    pub(crate) fn metadata_stats() -> MetadataStats {
        MetadataStats {
            tasks: crate::task::test_live_tasks(),
            workers: pool::LIVE_WORKERS.load(Ordering::Relaxed),
            domains: LIVE_DOMAINS.load(Ordering::Relaxed),
        }
    }
    /// 在真实工作元数据锁内记录标量消息，验证延期通知不会反向取得调度锁。
    pub(crate) fn test_klog() -> u64 {
        let _lock = STATE_LOCK.lock();
        crate::klog::write(crate::klog::Level::Debug, format_args!("X25 pool locked"))
    }
    /// 观察真实工作票与当前实际 TaskId。
    pub(crate) struct WorkSnapshot {
        pub queued: bool,
        pub running: bool,
        pub closed: bool,
        pub worker: usize,
    }
    /// 在生产状态锁下读取票身份，不构造模型状态。
    pub(crate) fn snapshot(work: Pin<&Work>) -> WorkSnapshot {
        let _lock = STATE_LOCK.lock();
        snapshot_locked(work)
    }
    /// 调用者持 STATE_LOCK，供 delayed 同锁连贯观察其两阶段票。
    pub(super) fn snapshot_locked(work: Pin<&Work>) -> WorkSnapshot {
        // SAFETY: 调用者持统一状态锁，Worker 指针由 running Task 身份保活。
        unsafe {
            let state = &*work.state.get();
            WorkSnapshot {
                queued: state.queued != 0,
                running: state.running != 0,
                closed: work.is_closed_locked(),
                worker: if state.worker.is_null() {
                    0
                } else {
                    (*state.worker).task_id_locked()
                },
            }
        }
    }
}
