//! 永久 perCPU 池、事件驱动管理者与预备域救援者；执行者只在真实阻塞时退出进度计数。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::alloc::AllocError;
use core::cell::UnsafeCell;
use core::pin::Pin;
use core::ptr;
#[cfg(feature = "ktest")]
use core::sync::atomic::AtomicUsize;
use core::sync::atomic::Ordering;

use super::{Domain, RemovedTicket, Work, STATE_LOCK};
use crate::cpu;
use crate::task::{self, JoinHandle, PreparedTask};
use crate::time::Duration;
use crate::timer;
use crate::wait::{WaitNode, WaitQueue, Waiter};

const CREATE_COOLDOWN: Duration = Duration::from_secs(1);
#[cfg(feature = "ktest")]
pub(super) static LIVE_WORKERS: AtomicUsize = AtomicUsize::new(0);

struct PoolState {
    head: *const Work,
    tail: *const Work,
    next_order: u64,
    workers_head: *const Worker,
    retired_head: *const Worker,
    retiring: usize,
    workers: usize,
    idle: usize,
    nr_running: usize,
    manager_requested: bool,
    manager_blocked: bool,
    create_failed: bool,
    #[cfg(feature = "ktest")]
    stats: Counters,
    #[cfg(feature = "ktest")]
    manager_task: Option<Weak<task::Task>>,
}

#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
struct Counters {
    created: usize,
    retired: usize,
    create_failures: usize,
    rescued: usize,
    sleeping: usize,
    running: usize,
    early_wake: usize,
}

/// CpuLocal 内唯一永久池；基础 worker 和独立 manager 在 AP 启动前准备。
pub(crate) struct WorkerPool {
    state: UnsafeCell<PoolState>,
    manager: WaitQueue,
}

// SAFETY: 池地址永久固定，链与计数只在 STATE_LOCK 内修改。
unsafe impl Send for WorkerPool {}
// SAFETY: 所有可变元数据采用统一状态锁；永久等待源自身同步。
unsafe impl Sync for WorkerPool {}

impl WorkerPool {
    /// 静态准备无资源分配的永久池，真实 Task 由 BSP 初始化过程另行准备。
    pub(crate) const fn new() -> Self {
        Self {
            state: UnsafeCell::new(PoolState {
                head: ptr::null(),
                tail: ptr::null(),
                next_order: 0,
                workers_head: ptr::null(),
                retired_head: ptr::null(),
                retiring: 0,
                workers: 0,
                idle: 0,
                nr_running: 0,
                manager_requested: false,
                manager_blocked: false,
                create_failed: false,
                #[cfg(feature = "ktest")]
                stats: Counters {
                    created: 0,
                    retired: 0,
                    create_failures: 0,
                    rescued: 0,
                    sleeping: 0,
                    running: 0,
                    early_wake: 0,
                },
                #[cfg(feature = "ktest")]
                manager_task: None,
            }),
            manager: WaitQueue::new(),
        }
    }
}

struct WorkerState {
    next: *const Worker,
    task_id: usize,
    current: *const Work,
    callback: bool,
    contributing: bool,
    idle: bool,
    /// 已归还完成票并准备让出，恢复后的下一次 claim 清除此状态。
    ready_to_claim: bool,
    join: Option<JoinHandle>,
    reserved_work: *const Work,
    reservation_open: bool,
    joined: bool,
    #[cfg(feature = "ktest")]
    task: Option<Weak<task::Task>>,
}

/// Task 的不可变身份保留此 metadata；metadata 不反向强拥有 Task，正常退出可归还整个栈。
pub(crate) struct Worker {
    cpu: usize,
    base: bool,
    domain: Option<Weak<Domain>>,
    reserved_domain: Option<Arc<Domain>>,
    state: UnsafeCell<WorkerState>,
    ready: WaitQueue,
}

// SAFETY: 字段构造后固定，状态仅经 STATE_LOCK，所属 Task 串行执行服务入口。
unsafe impl Send for Worker {}
// SAFETY: 元数据锁保护工作关联、idle 和贡献位，等待源独立同步。
unsafe impl Sync for Worker {}

impl Worker {
    /// 准备未发表身份；救援者弱保域，预留者强保域但只比较 Work 地址，不反向拥有 Work。
    fn try_new(
        cpu: usize,
        base: bool,
        domain: Option<Weak<Domain>>,
        reservation: Option<(&Arc<Domain>, *const Work)>,
    ) -> Result<Arc<Self>, AllocError> {
        let value = Self {
            cpu,
            base,
            domain,
            reserved_domain: reservation.map(|(domain, _)| Arc::clone(domain)),
            state: UnsafeCell::new(WorkerState {
                next: ptr::null(),
                task_id: 0,
                current: ptr::null(),
                callback: false,
                contributing: false,
                idle: false,
                ready_to_claim: false,
                join: None,
                reserved_work: reservation.map_or(ptr::null(), |(_, work)| work),
                reservation_open: reservation.is_some(),
                joined: false,
                #[cfg(feature = "ktest")]
                task: None,
            }),
            ready: WaitQueue::new(),
        };
        // 完整 value 先计入，Arc 分配失败由相同 Drop 归还；克隆不经过这里。
        #[cfg(feature = "ktest")]
        LIVE_WORKERS.fetch_add(1, Ordering::Relaxed);
        Arc::try_new(value)
    }

    /// scheduler 在 SCHED 锁外、callback 真正准备阻塞前撤销一次进度贡献。
    pub(crate) fn sleeping(&self) -> bool {
        let changed = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 身份与池计数均由 STATE_LOCK 保护，贡献位只在真 callback 中设置。
            unsafe {
                let worker = &mut *self.state.get();
                if worker.callback && worker.contributing {
                    worker.contributing = false;
                    let pool = &mut *pool(self.cpu).state.get();
                    pool.nr_running -= 1;
                    #[cfg(feature = "ktest")]
                    {
                        pool.stats.sleeping += 1;
                    }
                    true
                } else {
                    false
                }
            }
        };
        if changed {
            notify(self.cpu);
        }
        changed
    }

    /// 恢复或早通知未切出的同一 callback 只恢复本次实际撤销过的贡献。
    pub(crate) fn running(&self, was_running: bool) {
        if !was_running {
            return;
        }
        let _lock = STATE_LOCK.lock();
        // SAFETY: scheduler 与 claim/finish 由同一状态锁串行；callback 返回之前才会结束此身份。
        unsafe {
            let worker = &mut *self.state.get();
            assert!(worker.callback && !worker.contributing);
            worker.contributing = true;
            let pool = &mut *pool(self.cpu).state.get();
            pool.nr_running += 1;
            #[cfg(feature = "ktest")]
            {
                pool.stats.running += 1;
            }
        }
    }

    /// 只记录实际 sleeping 后 prepare_switch 未提交的早唤醒路径。
    #[cfg(feature = "ktest")]
    pub(crate) fn early_wake(&self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 池测试计数同样受生产状态锁保护。
        unsafe {
            (*pool(self.cpu).state.get()).stats.early_wake += 1;
        }
    }

    /// 通知在所有状态和 scheduler 锁外发出。
    pub(super) fn wake(&self) {
        self.ready.wake_one();
    }

    /// 调用者持 STATE_LOCK，返回准备期保存的真实 TaskId。
    pub(super) unsafe fn task_id_locked(&self) -> usize {
        // SAFETY: 调用者持统一元数据锁。
        unsafe { (*self.state.get()).task_id }
    }

    /// 在原状态锁内升级唯一服务入口保存的弱任务观察，不延长执行栈责任。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_task(&self) -> Option<Arc<task::Task>> {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 服务入口登记和退休由同一状态锁串行，弱升级只保控制对象。
        unsafe { (*self.state.get()).task.as_ref().and_then(Weak::upgrade) }
    }

    /// joined 位只由原 manager 在真实任务离栈并 join 返回后更新。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_joined(&self) -> bool {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 原退休链和 joined 状态均在统一状态锁下读写。
        unsafe { (*self.state.get()).joined }
    }
}

#[cfg(feature = "ktest")]
impl Drop for Worker {
    /// Arc 的最后强拥有或未发表分配失败才归还 payload，弱观察不影响计数。
    fn drop(&mut self) {
        LIVE_WORKERS.fetch_sub(1, Ordering::Relaxed);
    }
}

/// 读取 CpuLocal 内嵌的唯一真实池，不维护第二份 CPU 表。
fn pool(cpu: usize) -> &'static WorkerPool {
    &cpu::get(cpu).worker_pool
}

/// 锁下从池保存的原 into_raw 指针克隆身份，返回值只能在锁外最终析构。
unsafe fn clone_worker(raw: *const Worker) -> Arc<Worker> {
    // SAFETY: 调用者持锁，raw 来自 install 保存的真实池强引用；增加后恢复恰好新增的强引用。
    unsafe {
        Arc::increment_strong_count(raw);
        Arc::from_raw(raw)
    }
}

/// 锁下保活撤票的关联身份；payload 只用于比较，强引用来自真实登记或救援表。
///
/// # Safety
/// 调用者必须持 STATE_LOCK；assigned 来自仍 queued 的同 CPU、同域关联票。
pub(super) unsafe fn assigned_worker(
    cpu: usize,
    assigned: *const Worker,
    domain: &Domain,
) -> Option<Arc<Worker>> {
    if assigned.is_null() {
        return None;
    }
    // SAFETY: 调用者持 STATE_LOCK，关联票阻止原普通 worker 退休，救援表由活域保活。
    unsafe {
        let mut cursor = (*pool(cpu).state.get()).workers_head;
        while !cursor.is_null() {
            if cursor == assigned {
                return Some(clone_worker(cursor));
            }
            cursor = (*(*cursor).state.get()).next;
        }
        let rescuers = &*domain.rescuers.get();
        let rescue = rescuers
            .get(cpu)
            .expect("associated worker missing from pool and rescue table");
        assert_eq!(Arc::as_ptr(&rescue.worker), assigned);
        Some(Arc::clone(&rescue.worker))
    }
}

/// 锁下追加唯一 pending 节点，链顺序就是 CPU0 ordered 域的实际 FIFO。
pub(super) unsafe fn enqueue(cpu: usize, work: *const Work) {
    // SAFETY: 调用者持 STATE_LOCK，work 已持一份新增真实拥有票。
    unsafe {
        let state = &mut *pool(cpu).state.get();
        let node = &mut *(*work).state.get();
        state.next_order = state
            .next_order
            .checked_add(1)
            .expect("pool enqueue order exhausted");
        node.order = state.next_order;
        node.previous = state.tail;
        node.next = ptr::null();
        if state.tail.is_null() {
            state.head = work;
        } else {
            (*(*state.tail).state.get()).next = work;
        }
        state.tail = work;
        state.create_failed = false;
    }
}

/// 锁下从永久池撤链，不消耗或释放拥有票。
pub(super) unsafe fn unlink(cpu: usize, work: *const Work) {
    // SAFETY: 调用者持 STATE_LOCK 且节点在该池唯一链内。
    unsafe {
        let state = &mut *pool(cpu).state.get();
        let node = &mut *(*work).state.get();
        if node.previous.is_null() {
            state.head = node.next;
        } else {
            (*(*node.previous).state.get()).next = node.next;
        }
        if node.next.is_null() {
            state.tail = node.previous;
        } else {
            (*(*node.next).state.get()).previous = node.previous;
        }
        node.next = ptr::null();
        node.previous = ptr::null();
    }
}

/// 锁下复用 ordered 最早本域票谓词；已领取前票由 active 限制阻止并行。
unsafe fn fifo_ready(head: *const Work, work: *const Work) -> bool {
    // SAFETY: 调用者持 STATE_LOCK，work 位于 head 链中，所有 pending 节点强拥有各自域。
    unsafe {
        let domain = &(*work).domain;
        if !domain.config.ordered {
            return true;
        }
        let mut earlier = head;
        while earlier != work {
            if Arc::ptr_eq(&(*earlier).domain, domain) {
                return false;
            }
            earlier = (*(*earlier).state.get()).next;
        }
        true
    }
}

/// 锁下挑选准入票；同 Work 的后续票保留原执行者，有序域不绕过前票。
unsafe fn candidate(
    head: *const Work,
    worker: *const Worker,
    rescue: Option<&Domain>,
    unassigned: bool,
) -> *const Work {
    // SAFETY: 调用者持 STATE_LOCK，队列节点由其 pending 强引用保活。
    unsafe {
        let mut deferred: *const Domain = ptr::null();
        let reserved = if worker.is_null() {
            ptr::null()
        } else {
            (*(*worker).state.get()).reserved_work
        };
        if !worker.is_null() {
            let mut cursor = head;
            while !cursor.is_null() {
                let work = &*cursor;
                let node = &*work.state.get();
                if node.assigned == worker
                    && (reserved.is_null() || reserved == cursor)
                    && rescue.is_none_or(|domain| ptr::eq(domain, Arc::as_ptr(&work.domain)))
                {
                    if node.running == 0 && work.domain.eligible(work) && fifo_ready(head, cursor) {
                        return cursor;
                    }
                    deferred = Arc::as_ptr(&work.domain);
                }
                cursor = node.next;
            }
        }
        let mut cursor = head;
        while !cursor.is_null() {
            let work = &*cursor;
            let node = &*work.state.get();
            let correct_domain =
                rescue.is_none_or(|domain| ptr::eq(domain, Arc::as_ptr(&work.domain)));
            // 关联票保留执行者；ordered 只允许推进本域合法前票，不能承接无关域回调。
            let correct_worker = unassigned
                && reserved.is_null()
                && node.assigned.is_null()
                && (deferred.is_null()
                    || (work.domain.config.ordered
                        && ptr::eq(deferred, Arc::as_ptr(&work.domain))));
            if correct_domain
                && correct_worker
                && node.running == 0
                && work.domain.eligible(work)
                && fifo_ready(head, cursor)
            {
                return cursor;
            }
            cursor = node.next;
        }
        ptr::null()
    }
}

/// 真实需求事件只唤醒一个可执行 idle 或独立 manager；不会循环创建或定时扫描。
pub(super) fn notify(cpu: usize) {
    let (idle, manager, failed) = {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 永久池链和 worker 身份始终在统一锁内检查。
        unsafe {
            let state = &mut *pool(cpu).state.get();
            let mut selected = None;
            let mut cursor = state.workers_head;
            while !cursor.is_null() {
                let worker = &*cursor;
                if (*worker.state.get()).idle
                    && !candidate(state.head, cursor, None, true).is_null()
                {
                    selected = Some(clone_worker(cursor));
                    break;
                }
                cursor = (*worker.state.get()).next;
            }
            let manager =
                selected.is_none() && !state.create_failed && need_to_create_worker(state);
            if manager {
                state.manager_requested = true;
            }
            let mut queued = state.head;
            let mut associated_rescue = false;
            while !queued.is_null() {
                let node = &*(*queued).state.get();
                if !node.assigned.is_null() && (*node.assigned).domain.is_some() {
                    associated_rescue = true;
                    break;
                }
                queued = node.next;
            }
            (selected, manager, state.create_failed || associated_rescue)
        }
    };
    if let Some(worker) = idle {
        worker.wake();
    }
    if manager {
        pool(cpu).manager.wake_one();
    }
    if failed {
        wake_rescuers(cpu);
    }
}

/// 创建真实失败后逐个已接受节点通知对应预备域救援者；遍历只发生在需求事件。
fn wake_rescuers(cpu: usize) {
    let last = {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 有限事件边界固定当前队列提交序，不追随之后新入队节点。
        unsafe { (*pool(cpu).state.get()).next_order }
    };
    let mut visited = 0;
    loop {
        let (order, rescue) = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 逐份票在锁下读取；worker clone 保活通知目标，锁外没有队列裸指针。
            unsafe {
                let mut cursor = (*pool(cpu).state.get()).head;
                while !cursor.is_null() && (*(*cursor).state.get()).order <= visited {
                    cursor = (*(*cursor).state.get()).next;
                }
                if cursor.is_null() || last < (*(*cursor).state.get()).order {
                    (last, None)
                } else {
                    let order = (*(*cursor).state.get()).order;
                    let domain = &(*cursor).domain;
                    let rescue = if domain.config.reclaim {
                        let state = &*pool(cpu).state.get();
                        let rescuers = &*domain.rescuers.get();
                        rescuers.get(cpu).and_then(|r| {
                            if candidate(
                                state.head,
                                Arc::as_ptr(&r.worker),
                                Some(domain),
                                state.create_failed,
                            )
                            .is_null()
                            {
                                None
                            } else {
                                Some(Arc::clone(&r.worker))
                            }
                        })
                    } else {
                        None
                    };
                    (order, rescue)
                }
            }
        };
        visited = order;
        if let Some(worker) = rescue {
            worker.wake();
        }
        if last <= visited {
            break;
        }
    }
}

/// 准备 worker 身份和真实 Task，任务未发表时记录其真实标识。
fn prepare_worker(
    cpu: usize,
    base: bool,
    domain: Option<&Arc<Domain>>,
    reservation: Option<(&Arc<Domain>, *const Work)>,
) -> Result<(Arc<Worker>, PreparedTask), AllocError> {
    let worker = Worker::try_new(cpu, base, domain.map(Arc::downgrade), reservation)?;
    let role = Arc::clone(&worker);
    let keep_domain = domain.cloned();
    let prepared =
        task::prepare_worker_on(cpu, Arc::clone(&worker), move || service(role, keep_domain))?;
    {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 元数据尚未发表且仍沿统一锁写入真实 TaskId。
        unsafe {
            (*worker.state.get()).task_id = prepared.task_id().as_usize();
        }
    }
    Ok((worker, prepared))
}

/// 域构造者准备固定 CPU 的独占救援身份与 Task，尚不发表。
pub(super) fn prepare_rescuer(
    cpu: usize,
    domain: &Arc<Domain>,
) -> Result<(Arc<Worker>, PreparedTask), AllocError> {
    prepare_worker(cpu, false, Some(domain), None)
}

/// Work 构造期准备自己的原生 Task/栈；未发表失败对象仍按原拥有关系归还。
pub(super) fn prepare_reserved(
    cpu: usize,
    domain: &Arc<Domain>,
    work: *const Work,
) -> Result<(Arc<Worker>, PreparedTask), AllocError> {
    prepare_worker(cpu, false, None, Some((domain, work)))
}

/// 只释放固定预留，不睡眠；所有状态锁外唤醒，原服务与 manager 承担退出和 join。
pub(super) fn release_reservation(worker: &Worker) {
    {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 固定身份强拥有域，预留位与 retire 谓词只由此统一锁串行。
        unsafe {
            (*worker.state.get()).reservation_open = false;
        }
    }
    // close 的等待者也登记在 ready，必须同时唤醒服务，不能让 wake_one 只命中等待者。
    worker.ready.wake_all();
}

/// 外部 close 在真实 Work 票尾部之后等待原 manager 完成执行栈 join。
pub(super) fn close_reservation(worker: &Worker) {
    release_reservation(worker);
    super::wait_until(&worker.ready, || {
        let _lock = STATE_LOCK.lock();
        // SAFETY: join 位只在 manager 确认原 Task 离栈后写入。
        unsafe { (*worker.state.get()).joined }
    });
}

/// 域已拒新票且颜色归零；从原 pool 链释放未提交 Work 的预留，不增注册表或分配。
pub(super) fn release_domain_reservations(domain: &Arc<Domain>) {
    for cpu in 0..cpu::count() {
        loop {
            let selected = {
                let _lock = STATE_LOCK.lock();
                // SAFETY: 原池链持每份身份强引用，锁内只比较固定域并克隆现存拥有。
                unsafe {
                    let mut cursor = (*pool(cpu).state.get()).workers_head;
                    let mut selected = None;
                    while !cursor.is_null() {
                        let worker = &*cursor;
                        let role = &mut *worker.state.get();
                        if role.reservation_open
                            && worker
                                .reserved_domain
                                .as_ref()
                                .is_some_and(|own| Arc::ptr_eq(own, domain))
                        {
                            role.reservation_open = false;
                            selected = Some(clone_worker(cursor));
                            break;
                        }
                        cursor = role.next;
                    }
                    selected
                }
            };
            let Some(worker) = selected else {
                break;
            };
            worker.ready.wake_all();
        }
    }
}

/// 锁内发表 pool 拥有的真实 worker 强引用；Task 准备和调度发表都在锁外。
fn install(worker: Arc<Worker>) {
    let raw = Arc::into_raw(worker);
    let _lock = STATE_LOCK.lock();
    // SAFETY: raw 是本次 into_raw 对应的池强引用，metadata 节点稳定且只有这一份池登记。
    unsafe {
        let state = &mut *pool((*raw).cpu).state.get();
        (*(*raw).state.get()).next = state.workers_head;
        state.workers_head = raw;
        state.workers += 1;
        if let Some(domain) = &(*raw).reserved_domain {
            assert!(
                !(*domain.state.get()).closed,
                "reserved work constructed after domain close"
            );
            (*domain.state.get()).reserved_workers += 1;
        }
        #[cfg(feature = "ktest")]
        {
            state.stats.created += 1;
        }
    }
}

/// pool 先登记身份，再无分配发表；动态执行者保小完成句柄供 manager 在其正常退出后 join。
pub(super) fn publish_worker(worker: Arc<Worker>, prepared: PreparedTask) {
    install(Arc::clone(&worker));
    let join = prepared.publish();
    if worker.base {
        drop(join);
    } else {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 普通动态 worker 同 manager CPU 发表；预留者尚无票且 reservation_open
        // 阻止跨 CPU 抢先退休，构造入口返回前就会在此锁下保存唯一 join。
        unsafe {
            (*worker.state.get()).join = Some(join);
        }
    }
}

/// 有限摘退休链；join 与最后 metadata Drop 都在状态锁外，栈真实释放后才计已退休。
fn reap_retired(cpu: usize) {
    loop {
        let retired = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 此链拥有从活池移来的同一份 raw 强引用；唯一 manager 摘离并取小观察句柄。
            unsafe {
                let state = &mut *pool(cpu).state.get();
                let raw = state.retired_head;
                if raw.is_null() {
                    None
                } else {
                    let role = &mut *(*raw).state.get();
                    state.retired_head = role.next;
                    role.next = ptr::null();
                    Some((
                        Arc::from_raw(raw),
                        role.join.take().expect("retiring worker without join"),
                    ))
                }
            }
        };
        let Some((worker, join)) = retired else {
            return;
        };
        assert_eq!(join.join(), 0);
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 小 Join 的完成已证明执行栈离开并释放；计数与停稳谓词同锁。
            unsafe {
                let state = &mut *pool(cpu).state.get();
                state.retiring -= 1;
                (*worker.state.get()).joined = true;
                if let Some(domain) = &worker.reserved_domain {
                    (*domain.state.get()).reserved_workers -= 1;
                }
                #[cfg(feature = "ktest")]
                {
                    state.stats.retired += 1;
                }
            }
        }
        worker.ready.wake_all();
        if let Some(domain) = &worker.reserved_domain {
            domain.finished.wake_all();
        }
        drop(worker);
    }
}

/// 全部基础 Task 和 manager 准备成功后才发表任意一个服务。
pub(super) fn initialize() -> Result<(), AllocError> {
    let mut workers = Vec::new();
    let mut managers = Vec::new();
    workers
        .try_reserve_exact(cpu::count())
        .map_err(|_| AllocError)?;
    managers
        .try_reserve_exact(cpu::count())
        .map_err(|_| AllocError)?;
    for cpu in 0..cpu::count() {
        workers.push(prepare_worker(cpu, true, None, None)?);
        managers.push(task::prepare_kernel_on(cpu, move || manager(cpu))?);
    }
    for (worker, prepared) in workers {
        publish_worker(worker, prepared);
    }
    for prepared in managers {
        drop(prepared.publish());
    }
    Ok(())
}

/// 只在无贡献进度且没有现有执行者能承接合法未关联票时承担创建责任。
///
/// # Safety
/// 调用者必须持 STATE_LOCK，state 必须来自本 CPU 永久池，链上节点由现存拥有票与池引用保活。
unsafe fn need_to_create_worker(state: &PoolState) -> bool {
    // SAFETY: 调用者持统一状态锁，队列、关联身份和 worker 可用状态在整个判断期间保持有效。
    unsafe {
        if state.nr_running != 0 || candidate(state.head, ptr::null(), None, true).is_null() {
            return false;
        }
        let mut cursor = state.workers_head;
        while !cursor.is_null() {
            let role = &*(*cursor).state.get();
            if role.idle || role.ready_to_claim {
                let work = candidate(state.head, cursor, None, true);
                if !work.is_null() && (role.idle || (*(*work).state.get()).assigned.is_null()) {
                    return false;
                }
            }
            cursor = role.next;
        }
        true
    }
}

/// 管理者接受真实需求后保留创建责任；失败在锁外冷却，需求消失或执行者可用才结束重试。
fn manager(cpu: usize) -> i32 {
    #[cfg(feature = "ktest")]
    {
        let observation = Arc::downgrade(&task::current_task());
        let _lock = STATE_LOCK.lock();
        // SAFETY: 管理者唯一入口只发表不保执行栈的弱观察，状态锁保护一次初始化。
        unsafe {
            let state = &mut *pool(cpu).state.get();
            assert!(state.manager_task.is_none());
            state.manager_task = Some(observation);
        }
    }
    loop {
        reap_retired(cpu);
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: 永久源与固定栈节点存活至本轮撤销 registration。
        let registration = unsafe { pool(cpu).manager.register(node.as_mut()) };
        let requested = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 管理请求与阻塞观察使用同一短状态锁。
            unsafe {
                let state = &mut *pool(cpu).state.get();
                state.manager_blocked = !state.manager_requested;
                state.manager_requested
            }
        };
        if !requested {
            waiter.park();
        }
        drop(registration);
        drop(waiter);
        let requested = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 消费本次事件；后续冷却责任由此 Task 保留，不能依赖再次提交来重发请求。
            unsafe {
                let state = &mut *pool(cpu).state.get();
                state.manager_blocked = false;
                let requested = state.manager_requested;
                state.manager_requested = false;
                requested
            }
        };
        if !requested {
            continue;
        }
        loop {
            // 其它域持续 OOM 也不能阻止已退出预留者的真实 join 和域 close。
            reap_retired(cpu);
            let needed = {
                let _lock = STATE_LOCK.lock();
                // SAFETY: 每次创建前在同锁内复查准入、实际进度和可接票执行者；队列拥有票始终保留。
                unsafe {
                    let state = &mut *pool(cpu).state.get();
                    let needed = need_to_create_worker(state);
                    if !needed {
                        // 结束本轮失败资格，使 callback 下次真实阻塞可以发起新的创建需求。
                        state.create_failed = false;
                    }
                    needed
                }
            };
            if !needed {
                break;
            }
            match prepare_worker(cpu, false, None, None) {
                Ok((worker, prepared)) => {
                    {
                        let _lock = STATE_LOCK.lock();
                        // SAFETY: 唯一 manager 已成功准备普通执行者，本轮真实创建失败资格在发表前结束。
                        unsafe {
                            (*pool(cpu).state.get()).create_failed = false;
                        }
                    }
                    publish_worker(worker, prepared);
                    break;
                }
                Err(_) => {
                    {
                        let _lock = STATE_LOCK.lock();
                        // SAFETY: 只有真实 fallible 创建失败增加此计数并启用救援资格。
                        unsafe {
                            let state = &mut *pool(cpu).state.get();
                            #[cfg(feature = "ktest")]
                            {
                                state.stats.create_failures += 1;
                            }
                            state.create_failed = true;
                        }
                    }
                    wake_rescuers(cpu);
                    // 管理等待已撤销，固定栈 timer 独占下一代 Waiter；冷却期间不持 STATE_LOCK。
                    timer::sleep(CREATE_COOLDOWN);
                }
            }
        }
    }
}

/// 锁下 pending→running 转移引用和颜色，不增加第二份颜色票或复制节点。
unsafe fn claim(worker: &Worker, domain: Option<&Domain>) -> Option<RemovedTicket> {
    // SAFETY: 调用者持 STATE_LOCK，返回拥有票保活 Work 和可选 DelayedWork 直到锁外尾部。
    unsafe {
        (*worker.state.get()).ready_to_claim = false;
        let state = &*pool(worker.cpu).state.get();
        let raw = candidate(
            state.head,
            worker as *const Worker,
            domain,
            domain.is_none() || state.create_failed,
        );
        if raw.is_null() {
            return None;
        }
        unlink(worker.cpu, raw);
        let work = &*raw;
        let node = &mut *work.state.get();
        assert_eq!(node.running, 0);
        node.running = node.queued;
        node.queued = 0;
        node.running_color = node.color;
        node.worker = worker;
        node.assigned = ptr::null();
        let admission = &mut *work.domain.state.get();
        if work.reserved_worker.is_some() {
            assert_eq!(
                node.worker,
                Arc::as_ptr(work.reserved_worker.as_ref().unwrap())
            );
            admission.reserved_active += 1;
        } else {
            admission.active += 1;
        }
        let role = &mut *worker.state.get();
        if role.idle {
            role.idle = false;
            if domain.is_none() {
                (*pool(worker.cpu).state.get()).idle -= 1;
            }
        }
        role.callback = true;
        role.contributing = true;
        role.current = raw;
        (*pool(worker.cpu).state.get()).nr_running += 1;
        #[cfg(feature = "ktest")]
        if domain.is_some() {
            (*pool(worker.cpu).state.get()).stats.rescued += 1;
        }
        let delayed = if node.delayed_owner.is_null() {
            ptr::null()
        } else {
            Pin::new_unchecked(&*node.delayed_owner).claim_locked()
        };
        Some(RemovedTicket {
            work: raw,
            delayed,
            worker: None,
        })
    }
}

/// 锁下复用真实退休谓词，登记后再检查以封闭撤票通知早于 ready 登记的窗口。
unsafe fn can_retire(worker: &Worker) -> bool {
    // SAFETY: 调用者持 STATE_LOCK，所有 pending 票与 worker 登记串行变化。
    unsafe {
        if worker.base || worker.domain.is_some() || (*worker.state.get()).reservation_open {
            return false;
        }
        let mut work = (*pool(worker.cpu).state.get()).head;
        while !work.is_null() {
            if (*(*work).state.get()).assigned == worker as *const Worker {
                return false;
            }
            work = (*(*work).state.get()).next;
        }
        true
    }
}

/// 从池移出多余执行者的拥有引用；关联后续票仍在队时不得退休其原执行者。
unsafe fn retire(worker: &Worker) -> bool {
    // SAFETY: 调用者持 STATE_LOCK，永久池登记保活 worker 链。
    unsafe {
        if !can_retire(worker) {
            return false;
        }
        let state = &mut *pool(worker.cpu).state.get();
        let payload = worker as *const Worker;
        let mut cursor = state.workers_head;
        let mut previous: *const Worker = ptr::null();
        while cursor != payload {
            assert!(!cursor.is_null());
            previous = cursor;
            cursor = (*(*cursor).state.get()).next;
        }
        // cursor 保留 install 的原 into_raw 来源；payload 仅作地址比较。
        let role = &mut *(*cursor).state.get();
        let next = role.next;
        if previous.is_null() {
            state.workers_head = next;
        } else {
            (*(*previous).state.get()).next = next;
        }
        if role.idle {
            state.idle -= 1;
            role.idle = false;
        }
        state.workers -= 1;
        state.retiring += 1;
        role.next = state.retired_head;
        state.retired_head = cursor;
        state.manager_requested = true;
        true
    }
}

/// callback 在锁外执行，归还真实票后响应调度请求；空闲者复查登记，动态余量正常退休。
fn service(worker: Arc<Worker>, domain: Option<Arc<Domain>>) -> i32 {
    #[cfg(feature = "ktest")]
    {
        let observation = Arc::downgrade(&task::current_task());
        let _lock = STATE_LOCK.lock();
        // SAFETY: worker 唯一入口只保弱观察，不形成 metadata→Task 强引用环。
        unsafe {
            let role = &mut *worker.state.get();
            assert!(role.task.is_none());
            role.task = Some(observation);
        }
    }
    loop {
        let ticket = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: claim 只改变短元数据，票随后在锁外执行与释放。
            unsafe { claim(&worker, domain.as_deref()) }
        };
        if let Some(ticket) = ticket {
            #[cfg(feature = "ktest")]
            testing::pause(worker.cpu, testing::PausePoint::Started);
            // SAFETY: running 拥有票保活原 into_raw 的固定 Work 存储。
            let work = unsafe { Pin::new_unchecked(&*ticket.work) };
            (work.callback)(work);
            {
                let _lock = STATE_LOCK.lock();
                // SAFETY: 只有此 running 执行者能完成票；callback 已正常返回且反馈成对恢复。
                unsafe {
                    let node = &mut *work.state.get();
                    let role = &mut *worker.state.get();
                    assert!(role.callback && role.contributing);
                    assert_eq!(node.worker, Arc::as_ptr(&worker));
                    node.running = 0;
                    node.worker = ptr::null();
                    let state = &mut *work.domain.state.get();
                    if work.reserved_worker.is_some() {
                        state.reserved_active -= 1;
                    } else {
                        state.active -= 1;
                    }
                    if work.one_shot {
                        assert_eq!(node.queued, 0, "one-shot work requeued");
                        node.closed = true;
                        role.reservation_open = false;
                    }
                    state.counts[node.running_color] -= 1;
                    role.callback = false;
                    role.contributing = false;
                    role.current = ptr::null();
                    let pool = &mut *pool(worker.cpu).state.get();
                    pool.nr_running -= 1;
                    pool.create_failed = false;
                }
            }
            drop(ticket);
            #[cfg(feature = "ktest")]
            testing::pause(worker.cpu, testing::PausePoint::Finished);
            // 连续后继票不能独占 CPU；交接期间仅真实可领取的未关联票免于额外创建执行者。
            if cpu::current().need_resched.load(Ordering::Relaxed) {
                {
                    let _lock = STATE_LOCK.lock();
                    // SAFETY: 本服务独占 worker 执行，回调与运行票均已结束，测试停点已返回；
                    // 可用状态只由此处与下一次锁下 claim 修改，不能把停点阻塞当成可接票。
                    unsafe { (*worker.state.get()).ready_to_claim = true };
                }
                crate::sched::cond_resched();
            }
            continue;
        }
        let (retired, stop) = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 退休与救援停止谓词同 claim 共用状态锁。
            unsafe {
                let stop = domain
                    .as_ref()
                    .is_some_and(|d| (*d.state.get()).rescue_stop);
                (retire(&worker), stop)
            }
        };
        if retired {
            pool(worker.cpu).manager.wake_one();
            return 0;
        }
        if stop {
            return 0;
        }
        #[cfg(feature = "ktest")]
        if !worker.base && domain.is_none() {
            // 首次 retire=false 已在锁下确认此动态普通 worker 仍有关联票；此处尚未登记 ready。
            testing::pause(worker.cpu, testing::PausePoint::BeforeWait);
        }
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: worker 的本地 Arc 保住等待源，固定栈 node 到本轮 registration 撤销。
        let registration = unsafe { worker.ready.register(node.as_mut()) };
        let sleep = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 先登记再复查队列与停止条件，通知不能落在最终检查缝隙。
            unsafe {
                {
                    let role = &mut *worker.state.get();
                    if !role.idle {
                        role.idle = true;
                        if domain.is_none() {
                            (*pool(worker.cpu).state.get()).idle += 1;
                        }
                    }
                }
                let stop = domain
                    .as_ref()
                    .is_some_and(|d| (*d.state.get()).rescue_stop);
                let pool = &*pool(worker.cpu).state.get();
                candidate(
                    pool.head,
                    Arc::as_ptr(&worker),
                    domain.as_deref(),
                    domain.is_none() || pool.create_failed,
                )
                .is_null()
                    && !stop
                    && !can_retire(&worker)
            }
        };
        if sleep {
            waiter.park();
        }
        drop(registration);
        drop(waiter);
    }
}

/// 所有 work 同步入口拒绝自等和同 ordered 域后票依赖，不让 rescuer 掩盖逻辑死锁。
pub(super) fn assert_wait_allowed(domain: &Arc<Domain>, work: *const Work) {
    let Some(worker) = task::current_worker() else {
        return;
    };
    let _lock = STATE_LOCK.lock();
    // SAFETY: current 由此 Task 身份保活且仅在 STATE_LOCK 中修改，running 票保活其域。
    unsafe {
        let current = (*worker.state.get()).current;
        if current.is_null() {
            return;
        }
        assert!(
            current != work,
            "work callback cannot synchronously wait for itself"
        );
        let same = Arc::ptr_eq(&(*current).domain, domain);
        assert!(
            !(same && (domain.config.ordered || work.is_null())),
            "work callback cannot synchronously wait for its own domain"
        );
    }
}

#[cfg(feature = "ktest")]
pub(super) mod testing {
    use super::*;
    use core::sync::atomic::{AtomicUsize, Ordering};
    /// 生产 claim、完成后及首次退休检查后的独立锁外停点。
    #[derive(Clone, Copy)]
    pub(crate) enum PausePoint {
        Started,
        Finished,
        BeforeWait,
    }
    const PAUSE_POINTS: usize = PausePoint::BeforeWait as usize + 1;
    const ARMED: usize = 1;
    const PAUSED: usize = 2;
    const RELEASED: usize = 3;
    const PAUSE_TIMEOUT_SECONDS: u64 = 10;
    const SERVICE_TIMEOUT_SECONDS: u64 = 10;
    static PAUSE: [[AtomicUsize; PAUSE_POINTS]; crate::boot::MAX_CPUS] =
        [const { [const { AtomicUsize::new(0) }; PAUSE_POINTS] }; crate::boot::MAX_CPUS];
    /// 真实当前与累积池计数，sleeping/running 是成对 scheduler 反馈而非模型值。
    pub(crate) struct PoolStats {
        pub workers: usize,
        pub idle: usize,
        pub nr_running: usize,
        pub created: usize,
        pub retired: usize,
        pub create_failures: usize,
        pub rescued: usize,
        pub sleeping: usize,
        pub running: usize,
        pub early_wake: usize,
    }
    /// 锁下读取生产池计数。
    pub(crate) fn pool_stats(cpu: usize) -> PoolStats {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 永久池状态由统一锁保护。
        unsafe {
            let s = &*pool(cpu).state.get();
            PoolStats {
                workers: s.workers,
                idle: s.idle,
                nr_running: s.nr_running,
                created: s.stats.created,
                retired: s.stats.retired,
                create_failures: s.stats.create_failures,
                rescued: s.stats.rescued,
                sleeping: s.stats.sleeping,
                running: s.stats.running,
                early_wake: s.stats.early_wake,
            }
        }
    }
    /// 只观察管理 Task 的真实阻塞状态；弱引用升级、scheduler 查询与强引用析构均在状态锁外。
    pub(crate) fn manager_blocked(cpu: usize) -> bool {
        let observation = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 永久池中的弱观察由 STATE_LOCK 保护，只复制弱引用而不进入 scheduler。
            unsafe { (*pool(cpu).state.get()).manager_task.clone() }
        };
        observation
            .and_then(|manager| manager.upgrade())
            .is_some_and(|manager| crate::sched::test_state(&manager) == task::State::Blocked)
    }
    /// 观察真实角色，受控回收必须由预备域救援 Task 执行。
    pub(crate) fn is_rescuer() -> bool {
        task::current_worker().is_some_and(|worker| worker.domain.is_some())
    }
    /// 所有基础执行者和 manager 必须真实到达无工作阻塞稳态。
    pub(crate) fn wait_services_blocked() {
        let start = crate::arch::read_counter();
        let limit = crate::arch::counter_frequency()
            .checked_mul(SERVICE_TIMEOUT_SECONDS)
            .expect("work service deadline overflow");
        loop {
            let mut ready = true;
            for cpu in 0..cpu::count() {
                let observations = {
                    let _lock = STATE_LOCK.lock();
                    // SAFETY: 只在锁下复制弱观察；绝不在状态锁内取得 SCHED 或升级后释放 Task。
                    unsafe {
                        let state = &*pool(cpu).state.get();
                        if state.workers != 1
                            || state.retiring != 0
                            || state.idle != 1
                            || state.nr_running != 0
                            || !state.manager_blocked
                        {
                            None
                        } else {
                            let worker = &*state.workers_head;
                            (*worker.state.get())
                                .task
                                .clone()
                                .zip(state.manager_task.clone())
                        }
                    }
                };
                let blocked = observations.is_some_and(|(worker, manager)| {
                    worker
                        .upgrade()
                        .zip(manager.upgrade())
                        .is_some_and(|(worker, manager)| {
                            crate::sched::test_state(&worker) == task::State::Blocked
                                && crate::sched::test_state(&manager) == task::State::Blocked
                        })
                });
                ready &= blocked;
            }
            if ready {
                return;
            }
            assert!(
                crate::arch::read_counter().wrapping_sub(start) < limit,
                "work services did not block"
            );
            crate::sched::yield_now();
        }
    }
    /// 为实际 CPU 与指定生产窗口独立建立一次停点。
    /// 同一 fixture 的 Thread 控制者串行武装和清理，真实执行者退出旧停点后才可复武装。
    pub(crate) fn arm_pause(cpu: usize, point: PausePoint) {
        assert!(cpu < cpu::count());
        assert_eq!(
            PAUSE[cpu][point as usize].compare_exchange(
                0,
                ARMED,
                Ordering::AcqRel,
                Ordering::Acquire
            ),
            Ok(0)
        );
    }
    /// 检查真实执行者是否已到达指定窗口，不混淆同 CPU 的不同停点。
    pub(crate) fn is_paused(cpu: usize, point: PausePoint) -> bool {
        PAUSE[cpu][point as usize].load(Ordering::Acquire) == PAUSED
    }
    /// 唯一 Thread 控制者放行指定窗口的真实执行者。
    /// 不得持状态锁、调度锁或执行者退出依赖锁；与取消串行，返回不代表已经完成 close/join。
    pub(crate) fn release_pause(cpu: usize, point: PausePoint) {
        assert_eq!(
            PAUSE[cpu][point as usize].compare_exchange(
                PAUSED,
                RELEASED,
                Ordering::AcqRel,
                Ordering::Acquire
            ),
            Ok(PAUSED)
        );
    }
    /// 取消尚未到达的武装，或按原 2→3 握手放行已经暂停的真实 worker。
    ///
    /// 仅 Thread 控制者在未持状态锁、调度锁或执行者退出所需锁时调用；同一停点
    /// 的武装与清理由同一 fixture 串行负责。0 和已放行的 3 幂等返回，不等待离栈。
    /// CAS 失败重读生产者的到达状态；不修改 queued/running、关联或 worker 额度。
    /// 放行后仍须真实 close/join；上一执行者退出停点并完成清理前不得复武装。
    pub(crate) fn cancel_pause(cpu: usize, point: PausePoint) {
        assert!(cpu < cpu::count());
        let pause = &PAUSE[cpu][point as usize];
        loop {
            let state = pause.load(Ordering::Acquire);
            let next = match state {
                0 | RELEASED => return,
                ARMED => 0,
                PAUSED => RELEASED,
                _ => unreachable!("invalid work pause state"),
            };
            if pause
                .compare_exchange(state, next, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return;
            }
        }
    }
    /// 锁外使用有限硬件 counter 握手，绝不持状态锁等待另一 CPU。
    pub(super) fn pause(cpu: usize, point: PausePoint) {
        let pause = &PAUSE[cpu][point as usize];
        if pause
            .compare_exchange(ARMED, PAUSED, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return;
        }
        let start = crate::arch::read_counter();
        let limit = crate::arch::counter_frequency()
            .checked_mul(PAUSE_TIMEOUT_SECONDS)
            .expect("work pause deadline overflow");
        while pause.load(Ordering::Acquire) == PAUSED {
            assert!(
                crate::arch::read_counter().wrapping_sub(start) < limit,
                "work pause deadline expired"
            );
            core::hint::spin_loop();
        }
        assert_eq!(pause.swap(0, Ordering::AcqRel), RELEASED);
    }
}
