//! Per-CPU 工作线程池（`WorkerPool`）、事件驱动管理者线程（Manager）与救援线程（Rescuer）实现。
//!
//! # 模块职责与架构定位
//! 本模块负责管理每个 CPU 核心上的永久工作线程池与内核线程调度交互：
//! - **上游调用者**：`workqueue` 父模块（入队、摘除、排空检查、创建救援/预留线程）与调度器 `sched`
//!   （在 Worker 线程即将睡眠阻塞与恢复运行时回调 `Worker::sleeping` / `Worker::running`）。
//! - **下游依赖**：`task` 与 `sched`（创建内核线程、等待退出 `JoinHandle`、让出 CPU）、
//!   `timer`（OOM 创建失败后的冷却睡眠）以及 `wait`（等待队列）。
//!
//! # 核心设计与关键状态机
//!
//! ## 1. 并发进度感知与动态线程扩缩容（Concurrency Management）
//! 每个 CPU 拥有一个嵌入在 `CpuLocal` 中的永久 `WorkerPool`，包含：
//! - **1 个常驻基础线程（Base Worker）**：系统启动时预分配，永不退休；
//! - **按需创建的动态普通线程（Dynamic Workers）**：当正在运行的所有回调均发生睡眠阻塞导致池的并发进度计数
//!   `nr_running == 0`，且队列中仍有可执行的未关联工作项时，唤醒该 CPU 的独立 **管理者线程（Manager）** 动态创建新 Worker；
//! - **专属预留线程（Reserved Workers）** 与 **工作域救援线程（Rescuers）**：分别保障关键 I/O 完成与 OOM 内存回收路径不发生死锁。
//!
//! ```text
//!   Worker 领取工作项 claim():
//!     callback = true, contributing = true, pool.nr_running += 1
//!          │
//!          ├── [回调执行中即将阻塞] 调度器调用 Worker::sleeping():
//!          │     contributing = false, pool.nr_running -= 1
//!          │     若此时 pool.nr_running == 0 且有待处理工作:
//!          │       notify() ──► 优先唤醒池内空闲 Worker；若无则唤醒 Manager 动态创建新 Worker
//!          │                    （若 Manager 因 OOM 分配失败，置 create_failed = true 并唤醒域 Rescuer）
//!          │
//!          ├── [阻塞回调被唤醒恢复] 调度器调用 Worker::running():
//!          │     contributing = true, pool.nr_running += 1
//!          ▼
//!   回调返回:
//!     callback = false, contributing = false, pool.nr_running -= 1
//!     若队列为空且该线程为无关联票据的动态 Worker ──► retire() 移入 retired_head，由 Manager 线程 join 回收内核栈
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! - **严禁在持有调度器锁（`SCHED`）期间获取 `STATE_LOCK`**：调度器在切换上下文前必须先在锁外调用 `Worker::sleeping`；
//! - **严禁在持有 `STATE_LOCK` 期间触发线程唤醒、内存分配或睡眠**：所有 `WaitQueue::wake_*`、`Task` 准备与 `JoinHandle::join`
//!   均必须在释放 `STATE_LOCK` 之后执行。

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

/// 单个 CPU 工作池（`WorkerPool`）的内部可变状态，所有字段均由全局自旋锁 `STATE_LOCK` 保护。
struct PoolState {
    /// 待执行工作项侵入式双向链表的头指针（按入队先后顺序排列）。
    head: *const Work,
    /// 待执行工作项侵入式双向链表的尾指针。
    tail: *const Work,
    /// 单调递增的入队次序计数器，每次 `enqueue` 时自增 1 并赋给 `WorkState::order`。
    next_order: u64,
    /// 当前池中活跃（含基础、动态及预留）`Worker` 单向链表的头指针；池持有链表上每个节点的 `Arc<Worker>` 强引用。
    workers_head: *const Worker,
    /// 已退出服务循环、等待本 CPU 管理者线程（Manager）执行 `join` 回收内核栈的退休 `Worker` 链表头指针。
    retired_head: *const Worker,
    /// 当前已进入退休状态但尚未完成 `join` 回收的 `Worker` 线程数量。
    retiring: usize,
    /// 当前活跃链表（`workers_head`）中的 `Worker` 总数。
    workers: usize,
    /// 当前处于空闲睡眠等待状态（`idle == true`）的非救援 `Worker` 数量。
    idle: usize,
    /// 当前正在执行回调且未发生睡眠阻塞（`contributing == true`）的 `Worker` 数量。
    nr_running: usize,
    /// 是否已向本 CPU 的管理者线程发起创建新 Worker 或回收退休 Worker 的请求。
    manager_requested: bool,
    /// 管理者线程当前是否正阻塞在等待队列上（供内核测试观察稳态使用）。
    manager_blocked: bool,
    /// 最近一次由管理者线程尝试动态创建 Worker 是否因内存不足（OOM）而失败；置位期间允许域救援线程接管未绑定工作项。
    create_failed: bool,
    /// 内核测试专用的累计事件统计计数器。
    #[cfg(feature = "ktest")]
    stats: Counters,
    /// 本 CPU 管理者线程的弱引用（仅用于内核测试观察其调度阻塞状态）。
    #[cfg(feature = "ktest")]
    manager_task: Option<Weak<task::Task>>,
}

/// 工作池运行期间的累计事件统计计数器（仅用于内核测试）。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
struct Counters {
    /// 累计已安装到本池中的 `Worker` 总数。
    created: usize,
    /// 累计已由管理者线程完成 `join` 回收的动态/预留 `Worker` 总数。
    retired: usize,
    /// 管理者线程动态创建 `Worker` 失败（OOM）的累计次数。
    create_failures: usize,
    /// 由工作域专属救援线程（Rescuer）成功领取工作项的累计次数。
    rescued: usize,
    /// 正在执行回调的 `Worker` 进入睡眠阻塞时撤销并发进度贡献的累计次数。
    sleeping: usize,
    /// 睡眠阻塞中的回调恢复运行时重新计入并发进度贡献的累计次数。
    running: usize,
    /// 回调在调用 `sleeping` 后、实际让出 CPU 前即被提前唤醒（Early Wake）的累计次数。
    early_wake: usize,
}

/// 每个 CPU 核心独享的永久工作线程池，内嵌于 `CpuLocal` 结构中。
///
/// 每个池的基础 Worker 线程与独立管理者线程（Manager）均在系统启动阶段（从核启动前）预先分配完毕。
pub(crate) struct WorkerPool {
    /// 受 `STATE_LOCK` 保护的工作项队列、Worker 链表与并发计数状态。
    state: UnsafeCell<PoolState>,
    /// 用于唤醒本 CPU 管理者线程（Manager）的等待队列。
    manager: WaitQueue,
}

// SAFETY: WorkerPool 静态内嵌于 CpuLocal 中，内存地址永久固定；内部可变状态 PoolState 完全由全局自旋锁 STATE_LOCK 保护。
unsafe impl Send for WorkerPool {}
// SAFETY: 共享引用下对 state 的所有访问均由 STATE_LOCK 串行化，WaitQueue 自身具备多线程同步安全性。
unsafe impl Sync for WorkerPool {}

impl WorkerPool {
    /// 在编译期构造空的初始工作池（不执行任何动态内存分配，基础线程由引导处理器在初始化阶段另行挂载）。
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

/// 单个 `Worker` 线程的运行时可变状态，所有字段均由全局自旋锁 `STATE_LOCK` 保护。
struct WorkerState {
    /// 指向同一 CPU 工作池活跃链表（`workers_head`）或退休链表（`retired_head`）中下一个节点的裸指针。
    next: *const Worker,
    /// 本 Worker 对应的底层内核线程 `TaskId` 标量值。
    task_id: usize,
    /// 当前正在执行的 `Work` 节点裸指针；未在执行回调时为 `null`。
    current: *const Work,
    /// 当前是否正处于执行 `Work` 业务回调函数的过程中。
    callback: bool,
    /// 当前是否正在为所属工作池提供活跃并发进度贡献（执行回调且未发生睡眠阻塞时为 `true`）。
    contributing: bool,
    /// 当前是否已注册到等待队列并处于空闲待命状态。
    idle: bool,
    /// 标记当前 Worker 刚完成上一个回调并因 `need_resched` 主动让出 CPU，但恢复运行后可立即领取下一个工作项（免于触发多余的动态线程创建）。
    ready_to_claim: bool,
    /// 动态创建的普通 Worker 或预留 Worker 对应的内核线程 `JoinHandle`，在线程退休退出后由管理者线程取出并调用 `join` 回收内核栈。
    join: Option<JoinHandle>,
    /// 若本线程是某个 `Work` 的专属预留 Worker，记录该目标 `Work` 的地址（仅用于指针相等性比对，不持有 `Work` 的强引用）。
    reserved_work: *const Work,
    /// 专属预留状态是否仍然有效；当关联的预留 `Work` 销毁、关闭或一次性回调执行完毕时置为 `false`，允许本线程走正常退休回收流程。
    reservation_open: bool,
    /// 本线程退出后是否已由管理者线程完成 `JoinHandle::join` 回收（供 `close_reservation` 等待确认）。
    joined: bool,
    /// 指向本 Worker 底层内核线程 `Task` 的弱引用（仅用于内核测试观察状态，避免形成强引用循环）。
    #[cfg(feature = "ktest")]
    task: Option<Weak<task::Task>>,
}

/// 工作队列执行者线程（Worker）的身份与控制元数据。
///
/// 由底层内核线程 `Task` 持有其 `Arc<Worker>` 强引用；`Worker` 自身不反向持有 `Task` 的强引用，
/// 从而保证线程函数正常返回并经管理者 `join` 后能够彻底释放整个内核栈。
pub(crate) struct Worker {
    /// 本 Worker 固定绑定的 CPU 编号。
    cpu: usize,
    /// 是否为常驻基础线程（Base Worker）；基础线程永不退休。
    base: bool,
    /// 若本线程是某个工作域的专属救援线程（Rescuer），保存指向该工作域的弱引用。
    domain: Option<Weak<Domain>>,
    /// 若本线程是某个工作项的专属预留线程（Reserved Worker），保存所属工作域的强引用以维持域存活直至本线程完成 `join`。
    reserved_domain: Option<Arc<Domain>>,
    /// 受 `STATE_LOCK` 保护的运行时状态。
    state: UnsafeCell<WorkerState>,
    /// 用于唤醒本 Worker 线程领取工作项、退出或通知预留关闭完成的等待队列。
    ready: WaitQueue,
}

// SAFETY: Worker 的不可变字段在构造后固定不变，可变状态 WorkerState 完全受全局自旋锁 STATE_LOCK 保护，
// 且主服务循环仅由唯一绑定的内核线程执行，因此可安全跨线程转移所有权。
unsafe impl Send for Worker {}
// SAFETY: 共享引用下对 WorkerState 的访问均通过 STATE_LOCK 同步，内部的 WaitQueue 自身具备并发安全性。
unsafe impl Sync for Worker {}

impl Worker {
    /// 分配并初始化一个新的 `Worker` 身份实例（此时尚未挂载到工作池或发布到调度器）。
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
        // 在可失败的 Arc 分配前递增存活计数；若分配失败，局部变量 value 的 Drop 会自动递减归还。
        #[cfg(feature = "ktest")]
        LIVE_WORKERS.fetch_add(1, Ordering::Relaxed);
        Arc::try_new(value)
    }

    /// 当正在执行工作项回调的 Worker 线程即将睡眠阻塞时，由调度器在 `SCHED` 锁外调用，扣减一次工作池并发进度计数。
    ///
    /// 若本次扣减使池的活跃进度计数降为 0，会在锁外调用 `notify` 唤醒空闲 Worker 或通知管理者线程补充新线程。
    /// 返回 `true` 表示本次调用实际扣减了并发贡献（后续恢复运行时需配对恢复）。
    pub(crate) fn sleeping(&self) -> bool {
        let changed = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有全局自旋锁 STATE_LOCK，WorkerState 与所属 CPU 的 PoolState 均受该锁保护。
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

    /// 当先前因睡眠阻塞而扣减了并发贡献的 Worker 线程恢复运行（或在切换前被提前唤醒）时，由调度器调用以恢复其并发进度计数。
    pub(crate) fn running(&self, was_running: bool) {
        if !was_running {
            return;
        }

        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK；仅当此前的 sleeping() 返回 true（was_running == true）且仍在同一次回调执行期内时才恢复计数。
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

    /// 记录一次 Worker 在调用 `sleeping` 后、实际完成上下文切换前即被提前唤醒的事件计数（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) fn early_wake(&self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，安全递增本 CPU 工作池的测试统计计数。
        unsafe {
            (*pool(self.cpu).state.get()).stats.early_wake += 1;
        }
    }

    /// 在所有自旋锁与调度器锁之外，唤醒本 Worker 线程的等待队列。
    pub(super) fn wake(&self) {
        self.ready.wake_one();
    }

    /// 在已持有 `STATE_LOCK` 的前提下，读取本 Worker 绑定的内核线程 `TaskId`。
    ///
    /// # Safety
    /// 调用者必须已持有全局自旋锁 `STATE_LOCK`。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "工作票测试在原状态锁内核对实际执行 worker 的 TaskId"
        )
    )]
    pub(super) unsafe fn task_id_locked(&self) -> usize {
        // SAFETY: 调用者契约保证已持有 STATE_LOCK，安全读取 task_id。
        unsafe { (*self.state.get()).task_id }
    }

    /// 获取并升级本 Worker 在进入服务循环时登记的底层 `Task` 弱引用（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_task(&self) -> Option<Arc<task::Task>> {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，安全访问 WorkerState 中的弱引用并尝试升级。
        unsafe { (*self.state.get()).task.as_ref().and_then(Weak::upgrade) }
    }

    /// 检查本 Worker 的内核线程是否已由管理者线程完成 `JoinHandle::join` 回收（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_joined(&self) -> bool {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，安全读取 joined 状态位。
        unsafe { (*self.state.get()).joined }
    }
}

#[cfg(feature = "ktest")]
impl Drop for Worker {
    /// 当 `Worker` 的最后一个强引用释放（或未发布分配失败析构）时，递减全局存活 `Worker` 计数（仅用于内核测试）。
    fn drop(&mut self) {
        LIVE_WORKERS.fetch_sub(1, Ordering::Relaxed);
    }
}

/// 获取指定 CPU 的 `CpuLocal` 中内嵌的永久 `WorkerPool` 静态引用。
fn pool(cpu: usize) -> &'static WorkerPool {
    &cpu::get(cpu).worker_pool
}

/// 在持有 `STATE_LOCK` 的前提下，从工作池链表保存的 `Worker` 裸指针克隆出一份新的 `Arc<Worker>` 强引用。
///
/// # Safety
/// 调用者必须已持有 `STATE_LOCK`，且 `raw` 必须来自 `install` 挂载到工作池链表中的有效 `Arc::into_raw` 指针；
/// 返回的 `Arc<Worker>` 若发生析构，必须在释放 `STATE_LOCK` 之后进行。
unsafe fn clone_worker(raw: *const Worker) -> Arc<Worker> {
    // SAFETY: 调用者保证已持有 STATE_LOCK 且 raw 对应池链表中存活的强引用；先递增引用计数再通过 from_raw 构造新句柄，保持原池引用不变。
    unsafe {
        Arc::increment_strong_count(raw);
        Arc::from_raw(raw)
    }
}

/// 在持有 `STATE_LOCK` 的前提下，查找并克隆排队工作项所关联的目标 `Worker` 强引用（用于取消票据后在锁外唤醒该线程）。
///
/// # Safety
/// 调用者必须已持有全局自旋锁 `STATE_LOCK`，且 `assigned` 必须来自当前仍在 `cpu` 队列中排队的同域工作项。
pub(super) unsafe fn assigned_worker(
    cpu: usize,
    assigned: *const Worker,
    domain: &Domain,
) -> Option<Arc<Worker>> {
    if assigned.is_null() {
        return None;
    }
    // SAFETY: 调用者已持有 STATE_LOCK；只要队列中仍有关联到该 Worker 的排队票据，普通动态 Worker 就不会被退休摘除，
    // 而救援线程则由存活的 domain.rescuers 列表保持有效。
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

/// 在持有 `STATE_LOCK` 的前提下，将工作项节点追加到指定 CPU 工作池双向链表的尾部。
///
/// # Safety
/// 调用者必须已持有 `STATE_LOCK`，且 `work` 必须持有已递增的排队强引用且当前未在任何链表中。
pub(super) unsafe fn enqueue(cpu: usize, work: *const Work) {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK 且 work 指向有效的独立排队工作项节点。
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

/// 在持有 `STATE_LOCK` 的前提下，将工作项节点从指定 CPU 工作池的双向链表中摘除（不消耗或释放其强引用票据）。
///
/// # Safety
/// 调用者必须已持有 `STATE_LOCK`，且 `work` 当前必须位于 `cpu` 工作池的待执行链表中。
pub(super) unsafe fn unlink(cpu: usize, work: *const Work) {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK 且 work 位于该 CPU 工作池链表内，前驱与后继指针均有效。
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

/// 在持有 `STATE_LOCK` 的前提下，检查工作项是否满足严格有序域（`ordered`）的 FIFO 先后顺序约束。
///
/// 若工作项所属域为非有序域，直接返回 `true`；若为有序域，则仅当链表头部到 `work` 之间不存在任何同域的前序排队项时才返回 `true`。
///
/// # Safety
/// 调用者必须已持有 `STATE_LOCK`，且 `work` 必须位于以 `head` 为首的工作池待执行链表中。
unsafe fn fifo_ready(head: *const Work, work: *const Work) -> bool {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK，链表上所有排队节点均持有各自 Work 与所属 Domain 的强引用。
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

/// 在持有 `STATE_LOCK` 的前提下，从待执行队列中为指定 `Worker` 挑选下一个满足准入与顺序约束的工作项。
///
/// 挑选规则：
/// 1. 第一遍优先扫描已明确绑定（`assigned == worker`）给当前线程的工作项；若某工作项虽绑定给当前线程但暂不满足域准入条件，
///    记录其所属域 `deferred`，防止在有序域中绕过该前序工作项去执行同域后续工作项；
/// 2. 第二遍在允许承接未绑定工作项（`unassigned == true`）且当前线程非专属预留线程时，扫描未绑定（`assigned.is_null()`）的可执行工作项。
///
/// # Safety
/// 调用者必须已持有全局自旋锁 `STATE_LOCK`，且 `head` 链表中的所有节点均由各自的排队强引用保持有效。
unsafe fn candidate(
    head: *const Work,
    worker: *const Worker,
    rescue: Option<&Domain>,
    unassigned: bool,
) -> *const Work {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK，队列节点与关联的 Domain 在遍历期间均有效且不会并发修改。
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
            // 已绑定给特定执行者的工作项必须保留给原执行者；若第一遍扫描遇到暂不可执行的绑定工作项（deferred 非空），
            // 仅当该域为严格有序域时才允许当前线程先行推进排在它前面的同域合法工作项，绝不跨域承接无关回调。
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

/// 当有新工作项入队、旧工作项完成或活跃 Worker 进入睡眠阻塞时，按需唤醒目标 CPU 的空闲 Worker、管理者线程或救援线程。
pub(super) fn notify(cpu: usize) {
    let (idle, manager, failed) = {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有全局自旋锁 STATE_LOCK，安全遍历本 CPU 工作池的 Worker 链表与待执行队列。
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

/// 当动态创建 Worker 因 OOM 失败或队列中存在绑定到救援线程的工作项时，扫描当前队列并唤醒对应工作域的救援线程（Rescuer）。
fn wake_rescuers(cpu: usize) {
    let last = {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，读取当前最新的入队序号作为本次扫描的有限上界，避免被后续新入队的工作项无限拉长循环。
        unsafe { (*pool(cpu).state.get()).next_order }
    };
    let mut visited = 0;
    loop {
        let (order, rescue) = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 每次迭代重新获取 STATE_LOCK 并按单调递增的 order 查找下一个节点；在锁内克隆救援 Worker 的 Arc 强引用，不在锁外保留队列裸指针。
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

/// 分配并准备一个 `Worker` 身份及其对应的内核线程 `PreparedTask`（此时尚未挂载到工作池或发布到调度器）。
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
        // SAFETY: worker 刚刚创建且尚未发布，在持有 STATE_LOCK 的情况下写入底层内核线程的真实 TaskId。
        unsafe {
            (*worker.state.get()).task_id = prepared.task_id().as_usize();
        }
    }
    Ok((worker, prepared))
}

/// 为启用了内存回收（`reclaim`）的工作域在指定 CPU 上预分配专属救援线程（Rescuer）及其内核栈。
pub(super) fn prepare_rescuer(
    cpu: usize,
    domain: &Arc<Domain>,
) -> Result<(Arc<Worker>, PreparedTask), AllocError> {
    prepare_worker(cpu, false, Some(domain), None)
}

/// 为带有专属执行栈的工作项（`Work`）在指定 CPU 上预分配专属预留 Worker 线程及其内核栈。
pub(super) fn prepare_reserved(
    cpu: usize,
    domain: &Arc<Domain>,
    work: *const Work,
) -> Result<(Arc<Worker>, PreparedTask), AllocError> {
    prepare_worker(cpu, false, None, Some((domain, work)))
}

/// 解除指定预留 `Worker` 的专属预留状态（非阻塞），并在锁外唤醒该线程使其自行走退休退出流程。
pub(super) fn release_reservation(worker: &Worker) {
    {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，安全将 reservation_open 置为 false 以允许该预留线程退休。
        unsafe {
            (*worker.state.get()).reservation_open = false;
        }
    }
    // close_reservation 的等待者也可能注册在 worker.ready 上，因此必须使用 wake_all 同时唤醒服务线程与外部等待者。
    worker.ready.wake_all();
}

/// 解除专属预留 Worker 的预留状态，并阻塞等待本 CPU 的管理者线程完成对该线程的 `JoinHandle::join` 内核栈回收。
pub(super) fn close_reservation(worker: &Worker) {
    release_reservation(worker);
    super::wait_until(&worker.ready, || {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK；joined 标志仅由 Manager 线程在 JoinHandle::join 返回后于同锁下置为 true。
        unsafe { (*worker.state.get()).joined }
    });
}

/// 在工作域关闭且已排空所有工作项后，遍历所有 CPU 工作池，解除属于该域的所有未关闭预留 Worker 的预留状态并唤醒它们退休。
pub(super) fn release_domain_reservations(domain: &Arc<Domain>) {
    for cpu in 0..cpu::count() {
        loop {
            let selected = {
                let _lock = STATE_LOCK.lock();
                // SAFETY: 已持有 STATE_LOCK，工作池活跃链表持有每个 Worker 的强引用；在锁内匹配所属域并克隆强引用以便在锁外唤醒。
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

/// 将已准备好的 `Worker` 挂载到所属 CPU 工作池的活跃链表头部（由池持有一份 `Arc<Worker>` 强引用）。
fn install(worker: Arc<Worker>) {
    let raw = Arc::into_raw(worker);
    let _lock = STATE_LOCK.lock();
    // SAFETY: raw 刚由 Arc::into_raw 转出，代表移交给工作池活跃链表持有的唯一一份强引用；当前持有 STATE_LOCK，安全修改池链表与计数。
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

/// 将已分配的 `Worker` 挂载到工作池，并将其底层内核线程发布到调度器就绪队列。
///
/// 常驻基础线程（`base == true`）永不退休，直接丢弃 `JoinHandle`；
/// 动态普通线程与预留线程则将 `JoinHandle` 存入 `WorkerState::join`，供退休后由管理者线程执行 `join` 回收内核栈。
pub(super) fn publish_worker(worker: Arc<Worker>, prepared: PreparedTask) {
    install(Arc::clone(&worker));
    let join = prepared.publish();
    if worker.base {
        drop(join);
    } else {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK；对于动态普通 Worker，当前正运行在同一 CPU 的 Manager 线程上；
        // 对于预留 Worker，此时尚无工作项入队且 reservation_open 为 true 阻止其提前退休，因此在任何 retire 发生前必定已完成 join 句柄写入。
        unsafe {
            (*worker.state.get()).join = Some(join);
        }
    }
}

/// 摘除指定 CPU 工作池退休链表中的所有 `Worker`，并在 `STATE_LOCK` 锁外逐一调用 `JoinHandle::join` 回收其内核栈。
fn reap_retired(cpu: usize) {
    loop {
        let retired = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK；retired_head 链表中的每个节点均持有最初由 install 转入的 Arc::into_raw 强引用，
            // 本 CPU 唯一的 Manager 线程在此将其摘除并恢复为局部 Arc 所有权，同时取出对应的 JoinHandle。
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
            // SAFETY: 上方 join.join() 返回已确认目标内核线程彻底离开执行栈并释放了栈内存；持 STATE_LOCK 更新退休计数与 joined 标志。
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

/// 为全系统所有 CPU 预分配常驻基础 Worker 线程与独立管理者线程（Manager），全部成功后统一发布到调度器。
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

/// 在持有 `STATE_LOCK` 的前提下，判断当前 CPU 工作池是否需要由管理者线程动态创建新的普通 `Worker`。
///
/// 仅当池内活跃进度计数 `nr_running == 0`、队列中存在可由未绑定普通线程执行的工作项，
/// 且当前没有任何空闲（`idle`）或即将再次领取工作项（`ready_to_claim`）的现有线程能够承接该工作项时，返回 `true`。
///
/// # Safety
/// 调用者必须已持有全局自旋锁 `STATE_LOCK`，且 `state` 必须属于当前 CPU 的永久 `WorkerPool`。
unsafe fn need_to_create_worker(state: &PoolState) -> bool {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK，待执行队列节点与活跃 Worker 链表在检查期间均保持有效。
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

/// 每个 CPU 独立的管理者内核线程（Manager）主循环：负责回收已退休 Worker 的内核栈，并在并发进度归零时动态创建新 Worker。
///
/// 若动态创建因内存不足（OOM）失败，会置位 `create_failed` 唤醒相关工作域的救援线程，并在锁外睡眠冷却 `CREATE_COOLDOWN` 后重试，
/// 直至创建成功或不再需要新线程为止。
fn manager(cpu: usize) -> i32 {
    #[cfg(feature = "ktest")]
    {
        let observation = Arc::downgrade(&task::current_task());
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，在管理者线程唯一入口处登记一次指向自身 Task 的弱引用供测试观察。
        unsafe {
            let state = &mut *pool(cpu).state.get();
            assert!(state.manager_task.is_none());
            state.manager_task = Some(observation);
        }
    }

    loop {
        // Step 1: 清理并 join 所有已退休的 Worker，随后注册等待节点并在无管理请求时睡眠待命。
        reap_retired(cpu);
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: pool(cpu).manager 为静态生命周期等待队列，栈上固定的 node 在下方 drop(registration) 前始终有效。
        let registration = unsafe { pool(cpu).manager.register(node.as_mut()) };
        let requested = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，同步读取 manager_requested 并更新测试观察位 manager_blocked。
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
            // SAFETY: 已持有 STATE_LOCK，消费本次管理请求标志；后续若发生 OOM 冷却重试由本循环自行保持，无需外部再次触发。
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

        // Step 2: 循环回收退休线程并按需动态创建新 Worker；若遇 OOM 则唤醒救援线程并在锁外冷却重试。
        loop {
            // 即使某些工作域持续处于 OOM 状态，也必须先及时 join 回收已退出的预留线程，避免阻塞其他域的 close_sync。
            reap_retired(cpu);
            let needed = {
                let _lock = STATE_LOCK.lock();
                // SAFETY: 已持有 STATE_LOCK，在每次尝试分配前重新核实是否仍需创建新 Worker。
                unsafe {
                    let state = &mut *pool(cpu).state.get();
                    let needed = need_to_create_worker(state);
                    if !needed {
                        // 当前已不再需要创建新线程，清除 create_failed 标志，以便未来再次阻塞时能正常唤醒 Manager。
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
                        // SAFETY: 已持有 STATE_LOCK，新 Worker 已分配成功，在正式发布前清除 create_failed 失败状态。
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
                        // SAFETY: 已持有 STATE_LOCK，记录动态创建失败计数并置位 create_failed 以允许救援线程接管未绑定工作项。
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
                    // 上方已注销 manager 等待队列上的注册节点，此处在 STATE_LOCK 锁外安全调用定时器睡眠冷却。
                    timer::sleep(CREATE_COOLDOWN);
                }
            }
        }
    }
}

/// 在持有 `STATE_LOCK` 的前提下，为指定 `Worker` 从队列中领取一个可执行工作项，并将其状态从 `queued` 切换为 `running`。
///
/// 直接将排队阶段持有的 `Arc<Work>` 强引用与排空颜色批次转移给运行阶段，并在返回 [`RemovedTicket`] 中携带该强引用以便在回调执行完毕后于锁外释放。
///
/// # Safety
/// 调用者必须已持有全局自旋锁 `STATE_LOCK`。
unsafe fn claim(worker: &Worker, domain: Option<&Domain>) -> Option<RemovedTicket> {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK；返回的 RemovedTicket 持有原排队阶段递增的强引用，保证回调运行期间 Work 与 DelayedWork 存活。
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

/// 在持有 `STATE_LOCK` 的前提下，判断当前 `Worker` 是否满足作为多余动态线程退休退出的条件。
///
/// 常驻基础线程（`base`）、工作域救援线程（`domain.is_some()`）、仍处于有效预留状态的专属线程（`reservation_open`），
/// 以及队列中仍有绑定（`assigned`）到自身的排队工作项的线程，均不可退休。
///
/// # Safety
/// 调用者必须已持有全局自旋锁 `STATE_LOCK`。
unsafe fn can_retire(worker: &Worker) -> bool {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK，可安全读取 WorkerState 并遍历本 CPU 待执行队列。
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

/// 在持有 `STATE_LOCK` 的前提下，若当前动态 `Worker` 满足退休条件，则将其从活跃链表移入退休链表（`retired_head`）并请求管理者线程回收。
///
/// # Safety
/// 调用者必须已持有全局自旋锁 `STATE_LOCK`，且 `worker` 必须位于本 CPU 工作池的活跃链表中。
unsafe fn retire(worker: &Worker) -> bool {
    // SAFETY: 调用者契约保证已持有 STATE_LOCK，且池活跃链表持有各节点的强引用。
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
        // cursor 指向最初由 install 挂载进链表的 Arc::into_raw 指针，直接将其移入 retired_head 供 Manager 线程恢复释放。
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

/// 工作队列执行者线程（含常驻基础 Worker、动态普通 Worker、专属预留 Worker 与域救援线程）的主服务循环。
fn service(worker: Arc<Worker>, domain: Option<Arc<Domain>>) -> i32 {
    #[cfg(feature = "ktest")]
    {
        let observation = Arc::downgrade(&task::current_task());
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，在 Worker 线程唯一入口处保存指向自身 Task 的弱引用，避免形成循环强引用。
        unsafe {
            let role = &mut *worker.state.get();
            assert!(role.task.is_none());
            role.task = Some(observation);
        }
    }
    loop {
        // Step 1: 尝试从本 CPU 工作池队列中领取一个满足条件的工作项。
        let ticket = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，在锁内完成工作项摘除与状态切换，返回的 ticket 在锁外保活目标对象。
            unsafe { claim(&worker, domain.as_deref()) }
        };
        if let Some(ticket) = ticket {
            // Step 2: 若成功领取工作项，在锁外执行其业务回调函数，随后重新持锁完成运行票据归账。
            #[cfg(feature = "ktest")]
            testing::pause(worker.cpu, testing::PausePoint::Started);
            // SAFETY: ticket.work 持有当前运行票据的 Arc<Work> 强引用，且 Work 内存位置固定不移动。
            let work = unsafe { Pin::new_unchecked(&*ticket.work) };
            (work.callback)(work);

            {
                let _lock = STATE_LOCK.lock();
                // SAFETY: 已持有 STATE_LOCK；当前线程是唯一持有该工作项运行权的 Worker，且回调中的睡眠/唤醒计数已在返回前成对恢复。
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

            // Step 3: 在锁外释放 RemovedTicket（触发等待者唤醒与引用递减）；若当前 CPU 存在调度抢占请求，标记 ready_to_claim 并主动让出 CPU。
            if cpu::current().need_resched.load(Ordering::Relaxed) {
                {
                    let _lock = STATE_LOCK.lock();
                    // SAFETY: 已持有 STATE_LOCK；此时回调与运行票据均已结束且测试停点已返回，置位 ready_to_claim 表示本线程唤醒后可立即领取下一项。
                    unsafe { (*worker.state.get()).ready_to_claim = true };
                }
                crate::sched::cond_resched();
            }
            continue;
        }

        // Step 4: 当前无立即可执行的工作项；检查是否应当作为多余动态线程退休，或作为救援线程响应域关闭退出。
        let (retired, stop) = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，在同一临界区内检查救援停止标志并尝试执行动态线程退休。
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
            // 上方 retire 返回 false 说明此动态普通 Worker 在队列中仍有绑定的工作项，在注册等待队列前触发 BeforeWait 测试停点。
            testing::pause(worker.cpu, testing::PausePoint::BeforeWait);
        }

        // Step 5: 注册等待队列并在 STATE_LOCK 下复查队列、停止标志与退休条件；若仍无事可做则进入睡眠。
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: 本地持有的 Arc<Worker> 保证 worker.ready 等待队列有效，栈上固定的 node 在下方 drop(registration) 前始终存活。
        let registration = unsafe { worker.ready.register(node.as_mut()) };
        let sleep = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK；在完成等待队列注册后复查所有唤醒条件，防止唤醒信号在检查与睡眠之间丢失。
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

/// 校验当前执行上下文是否允许对目标工作域 `domain` 或特定工作项 `work` 发起同步等待（防止自等待或同有序域死锁）。
///
/// 若当前线程正作为 Worker 执行某项回调：
/// - 严禁同步等待当前正在执行的同一个 `Work`；
/// - 严禁对当前所属的同一个工作域执行全域级等待（`work.is_null()`），或在同一个严格有序域（`ordered`）内等待其他工作项。
pub(super) fn assert_wait_allowed(domain: &Arc<Domain>, work: *const Work) {
    let Some(worker) = task::current_worker() else {
        return;
    };
    let _lock = STATE_LOCK.lock();
    // SAFETY: 已持有 STATE_LOCK；worker.state.current 仅由当前线程在同锁下设置与清除，且正在运行的票据强引用保证 current 及其所属 domain 有效。
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

    /// Worker 主服务循环中的测试停点位置（仅用于内核测试）。
    #[derive(Clone, Copy)]
    pub(crate) enum PausePoint {
        /// 已成功领取（`claim`）工作项、即将调用业务回调函数之前。
        Started,
        /// 业务回调已返回且已释放运行票据、即将进入下一轮循环之前。
        Finished,
        /// 动态普通 Worker 首次尝试退休未果（因仍有关联工作项）、即将注册等待队列睡眠之前。
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

    /// 单个 CPU 工作池的当前实时状态与累计事件统计快照（仅用于内核测试）。
    pub(crate) struct PoolStats {
        /// 当前活跃链表中的 `Worker` 总数。
        pub workers: usize,
        /// 当前处于空闲等待状态的非救援 `Worker` 数量。
        pub idle: usize,
        /// 当前正在运行回调且未阻塞的 `Worker` 数量。
        pub nr_running: usize,
        /// 累计已创建的 `Worker` 数量。
        pub created: usize,
        /// 累计已退休并完成 `join` 回收的 `Worker` 数量。
        pub retired: usize,
        /// 累计动态创建 `Worker` 失败（OOM）的次数。
        pub create_failures: usize,
        /// 累计由救援线程（Rescuer）领取执行工作项的次数。
        pub rescued: usize,
        /// 累计回调进入睡眠阻塞扣减并发贡献的次数。
        pub sleeping: usize,
        /// 累计回调从睡眠恢复重新计入并发贡献的次数。
        pub running: usize,
        /// 累计回调在切换前被提前唤醒的次数。
        pub early_wake: usize,
    }

    /// 获取 `STATE_LOCK` 并读取指定 CPU 工作池的统计快照（仅用于内核测试）。
    pub(crate) fn pool_stats(cpu: usize) -> PoolStats {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有全局自旋锁 STATE_LOCK，安全读取指定 CPU 的 PoolState。
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

    /// 检查指定 CPU 的管理者线程（Manager）当前是否正处于调度器阻塞（`Blocked`）状态（仅用于内核测试）。
    pub(crate) fn manager_blocked(cpu: usize) -> bool {
        let observation = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，仅在锁内克隆 Weak 引用，将升级与调度器状态查询放在锁外以避免锁顺序反转。
            unsafe { (*pool(cpu).state.get()).manager_task.clone() }
        };
        observation
            .and_then(|manager| manager.upgrade())
            .is_some_and(|manager| crate::sched::test_state(&manager) == task::State::Blocked)
    }

    /// 判断当前正在运行的内核线程是否为某个工作域的专属救援线程（Rescuer，仅用于内核测试）。
    pub(crate) fn is_rescuer() -> bool {
        task::current_worker().is_some_and(|worker| worker.domain.is_some())
    }

    /// 循环等待直至所有 CPU 的工作池均回归仅剩 1 个空闲基础 Worker 且管理者线程均进入阻塞稳态（仅用于内核测试）。
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
                    // SAFETY: 已持有 STATE_LOCK；仅在满足池稳态计数时克隆基础 Worker 与 Manager 的 Weak 引用，不在自旋锁内访问调度器或析构 Arc<Task>。
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

    /// 为指定 CPU 上的指定 `PausePoint` 武装一次测试停点（仅用于内核测试）。
    ///
    /// 同一测试夹具的控制线程须串行管理武装与放行，且必须等待上一轮暂停的 Worker 退出停点并清理完毕后方可再次武装。
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

    /// 检查指定 CPU 上的 Worker 线程是否已到达并暂停在目标测试停点处（仅用于内核测试）。
    pub(crate) fn is_paused(cpu: usize, point: PausePoint) -> bool {
        PAUSE[cpu][point as usize].load(Ordering::Acquire) == PAUSED
    }

    /// 放行指定 CPU 上正暂停在目标停点处的 Worker 线程（仅用于内核测试）。
    ///
    /// 调用者在调用时不得持有 `STATE_LOCK`、调度器锁或阻碍目标线程继续执行的锁。
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

    /// 取消指定 CPU 停点上尚未触发的武装，或放行已经到达停点暂停的 Worker 线程（仅用于内核测试）。
    ///
    /// 供测试控制线程在未持有自旋锁时幂等调用；放行后仍需通过正常的同步等待或关闭接口确认线程完成后续执行。
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

    /// 若当前 CPU 的指定停点已被武装，则在 `STATE_LOCK` 锁外自旋等待测试控制线程放行（仅用于内核测试）。
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
