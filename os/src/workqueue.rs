//! 内核工作队列（Workqueue）核心模块。
//!
//! # 模块职责与架构定位
//! 本模块提供可在内核线程上下文中睡眠的异步工作执行基础设施，是块设备请求完成、
//! 定时延迟任务、文件系统后台回写等子系统的核心异步调度支撑。
//! - **上游调用者**：块设备层（`bio`）、网络/套接字层、系统调用或测试套件，通过创建 `WorkQueue`、
//!   `Work` 或 `DelayedWork` 提交异步任务。
//! - **下游依赖**：`pool` 子模块（管理 per-CPU 工作线程池与救援线程）、`delayed` 子模块
//!   （结合粗粒度定时器实现延迟提交）、`task` 与 `sched`（内核线程创建与调度）、`wait` 与 `sync`
//!   （等待队列、睡眠互斥锁与自旋锁）。
//!
//! # 核心设计与关键状态机
//!
//! ## 1. 单工作项的双槽位生命周期（Queued + Running）
//! 每个 `Work` 实例具有固定的内存地址、所属工作域（`Domain`）和目标 CPU。同一时刻，一个 `Work`
//! 最多保有一份排队中（`queued`）票据和一份执行中（`running`）票据；每份活跃票据都对应一次
//! 真实的 `Arc<Work>` 强引用计数递增，确保异步执行期间内存绝对安全：
//!
//! ```text
//!   [Idle: queued=0, running=0]
//!        │
//!        │ queue() -> 分配单调递增 ticket N, 计入当前域颜色 counts[color], Arc 强引用 +1
//!        ▼
//!   [Queued: queued=N, running=0] ──(cancel_sync / cancel_queued_locked)──► 摘除链表, 扣减颜色计数,
//!        │                                                                  锁外释放 RemovedTicket
//!        │ Worker 线程 claim() -> 摘除链表, running=N, queued=0 (复用同一份 Arc 强引用与颜色计数)
//!        ▼
//!   [Running: queued=0, running=N] ──(运行期间再次调用 queue())──► [Queued+Running: queued=N+1, running=N]
//!        │                                                          (新票据记录 assigned=当前 Worker)
//!        │ 回调函数返回 -> running=0, 扣减 counts[running_color], 锁外释放 RemovedTicket (Arc -1)
//!        ▼
//!   [Idle 或 Queued(N+1)]
//! ```
//!
//! ## 2. 双色翻转有限排空（Two-Color Flush）
//! `WorkQueue::flush` 采用红黑双色（`color` 0 和 1）机制实现无饥饿的有限排空，确保 `flush()`
//! 只等待调用时刻之前已入队的工作项，而不会被后续源源不断提交的新工作无限阻塞：
//!
//! ```text
//!   提交阶段 (domain.color = 0):
//!     新入队的 Work 标记 color = 0，累加 domain.counts[0]
//!
//!   调用 domain.flush() (由 flush_serial 睡眠锁串行化):
//!     1. 持 STATE_LOCK 断言另一颜色计数 counts[1] == 0
//!     2. 将当前入队颜色翻转: domain.color = 0 ^ 1 = 1 (后续新提交全部计入 counts[1])
//!     3. 在 domain.finished 等待队列上睡眠，直到旧颜色计数 counts[0] 归零
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 为防止死锁，本模块及子模块严格遵守以下锁层级顺序（从外到内）：
//! 1. **操作串行化睡眠锁**（可睡眠，不可在中断或自旋锁内获取）：
//!    - `Domain::close_serial` -> `Domain::flush_serial`
//!    - `Work::stop_serial`
//! 2. **全局元数据自旋锁 `STATE_LOCK`**（关中断自旋锁，临界区极短，严禁在持锁期间睡眠、分配内存或唤醒任务）：
//!    - 统一保护 `DomainState`、`WorkState`、`DelayedWork::metadata`、`PoolState` 与 `WorkerState`。
//! 3. **调度器与等待队列锁 / 定时器锁**（如 `WaitQueue` 内部锁、`Timer` 锁）：
//!    - 所有 `wake_one` / `wake_all`、`pool::notify` 以及 `RemovedTicket` 的 `Arc` 析构**必须**在释放 `STATE_LOCK` 之后执行。

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

// ponytail: 全局元数据自旋锁限制高并发提交吞吐；仅在实际测得锁竞争瓶颈后再拆分为域锁与 per-CPU 池锁。
pub(super) static STATE_LOCK: SpinLock<()> = SpinLock::new(());
static INITIALIZED: AtomicBool = AtomicBool::new(false);
// 排空操作（flush）通过串行翻转新旧两种颜色来界定等待批次，颜色索引仅需 1 个比特位。
const FLUSH_COLOR_BITS: usize = 1;
const FLUSH_COLORS: usize = 1 << FLUSH_COLOR_BITS;
const FLUSH_COLOR_MASK: usize = FLUSH_COLORS - 1;
#[cfg(feature = "ktest")]
static LIVE_DOMAINS: AtomicUsize = AtomicUsize::new(0);

/// 工作队列（工作域）的创建配置参数。
#[derive(Clone, Copy)]
pub struct WorkQueueConfig {
    /// 普通并发域在全系统范围内允许同时处于活跃执行状态（未睡眠阻塞或睡眠中）的最大工作项数量。
    /// 若 `ordered` 为 `true`，实际并发上限会被强制限制为 1。
    pub max_active: usize,
    /// 是否为严格有序（Ordered）工作域。
    /// 若为 `true`，所有提交至该域的工作项都会固定绑定到 CPU0，并严格按 FIFO 顺序串行执行（同一时刻最多准入 1 个回调）。
    pub ordered: bool,
    /// 是否开启内存回收救援（Reclaim Rescue）保障。
    /// 若为 `true`，在创建工作域时会预先为相关 CPU 分配专用的救援内核线程（Rescuer）；
    /// 当工作线程池因内存耗尽（OOM）无法动态创建新 Worker 线程时，由救援线程接管该域的工作项以推进内存回收。
    pub reclaim: bool,
}

/// 向工作队列提交工作项时可能返回的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueueError {
    /// 目标工作项或所属工作域已永久关闭，拒绝接受任何新的提交。
    Closed,
}

/// 工作域的运行时可变状态，所有字段均由全局自旋锁 `STATE_LOCK` 保护。
pub(super) struct DomainState {
    /// 工作域是否已调用 `close_sync` 进入永久关闭状态；置位后拒绝新工作入队。
    closed: bool,
    /// 工作域是否已完成初始化并正式对外发布。
    published: bool,
    /// 当前新入队工作项所使用的颜色批次索引（0 或 1）。
    color: usize,
    /// 各颜色批次下尚未执行完毕（含排队中和执行中）的工作票据计数。
    counts: [usize; FLUSH_COLORS],
    /// 当前域内正在占用普通并发配额的执行中工作项数量（不含自带预留 Worker 的工作项）。
    active: usize,
    /// 当前域内正在使用专属预留 Worker 执行的工作项数量。
    reserved_active: usize,
    /// 绑定到本域且尚未由管理者线程（Manager）完成 `join` 回收的预留 Worker 线程总数。
    reserved_workers: usize,
    /// 指示本域的救援线程（Rescuer）是否应当退出主服务循环。
    rescue_stop: bool,
}

/// 工作域专属救援线程（Rescuer）的控制槽位。
pub(super) struct Rescue {
    /// 救援线程对应的 Worker 身份元数据。
    worker: Arc<Worker>,
    /// 救援内核线程的 Join 句柄；在 `close_sync` 期间由唯一持有 `close_serial` 的调用者取出并等待线程退出。
    join: UnsafeCell<Option<JoinHandle>>,
}

/// 工作域内部核心结构，由 `WorkQueue` 句柄、所属 `Work` 节点以及救援线程共同通过 `Arc` 持有。
pub(super) struct Domain {
    /// 不可变的工作域配置参数。
    config: WorkQueueConfig,
    /// 受 `STATE_LOCK` 保护的运行时状态（颜色计数、准入配额、关闭标志等）。
    state: UnsafeCell<DomainState>,
    /// 串行化 `flush` 操作的睡眠互斥锁，保证同一时刻只有一个调用者翻转颜色并等待旧颜色排空。
    flush_serial: SleepMutex<()>,
    /// 串行化 `close_sync` 操作的睡眠互斥锁，保证关闭与救援线程回收流程的独占执行。
    close_serial: SleepMutex<()>,
    /// 当域内工作项完成、取消或预留 Worker 回收完成时触发唤醒的等待队列。
    finished: WaitQueue,
    /// 预分配的救援线程列表；仅在 `WorkQueue::try_new` 发布前写入，发布后切片结构只读。
    rescuers: UnsafeCell<Vec<Rescue>>,
}

// SAFETY: Domain 的所有可变元数据均通过全局自旋锁 STATE_LOCK 同步访问；
// rescuers 向量仅在对象发布前单线程构造，内部 join 句柄由 close_serial 睡眠锁串行取出，因此可安全跨线程转移所有权。
unsafe impl Send for Domain {}
// SAFETY: 共享引用下对 state 的修改均受 STATE_LOCK 保护，rescuers 列表在发布后只读且 join 槽位由 close_serial 独占消费，
// 内部 SleepMutex 与 WaitQueue 均自身具备并发安全性。
unsafe impl Sync for Domain {}

/// 工作队列的外部拥有者句柄，负责管理工作域的生命周期与显式同步关闭。
///
/// 已创建的 `Work` 节点与预分配的救援线程各自持有底层 `Domain` 的 `Arc` 强引用，
/// 但只有本句柄的持有者负责在销毁前调用 [`WorkQueue::close_sync`] 停止工作域。
pub struct WorkQueue {
    /// 指向底层工作域核心结构的强引用。
    domain: Arc<Domain>,
}

impl WorkQueue {
    /// 根据给定配置创建一个新的工作队列。
    ///
    /// 若开启了 `reclaim` 选项，会在正式发布工作域前预先为每个目标 CPU 分配救援线程及其内核栈；
    /// 任何一步内存分配失败都会干净地释放已分配的临时资源并返回 `AllocError`。
    pub fn try_new(config: WorkQueueConfig) -> Result<Self, AllocError> {
        assert!(
            0 < config.max_active,
            "work domain needs positive admission limit"
        );
        assert!(
            INITIALIZED.load(Ordering::Acquire),
            "work services not initialized"
        );

        // Step 1: 构造处于未发布（published = false）初始状态的 Domain 实例。
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
        // 在可失败的 Arc 分配前递增存活域计数；若 Arc::try_new 失败，局部变量 value 的 Drop 会自动递减归还。
        #[cfg(feature = "ktest")]
        LIVE_DOMAINS.fetch_add(1, Ordering::Relaxed);
        let domain = Arc::try_new(value)?;
        // Step 2: 若启用了内存回收救援（reclaim），预先分配救援线程列表与底层内核线程，但暂不发布到调度器。
        let count = if config.ordered { 1 } else { cpu::count() };
        let mut prepared: Vec<(Arc<Worker>, PreparedTask)> = Vec::new();
        if config.reclaim {
            prepared.try_reserve_exact(count).map_err(|_| AllocError)?;
            // SAFETY: domain 刚刚创建且尚未对外发布，当前线程是其唯一持有者，不存在任何并发访问。
            unsafe {
                let rescuers = &mut *domain.rescuers.get();
                rescuers.try_reserve_exact(count).map_err(|_| AllocError)?;
            }
            for cpu in 0..count {
                prepared.push(pool::prepare_rescuer(cpu, &domain)?);
            }
        }

        // Step 3: 所有救援线程均已分配成功，将其发布到调度器就绪队列并保存 JoinHandle，此过程不再触发内存分配。
        // SAFETY: domain 仍未对外发布，当前线程独占访问 rescuers 向量；前方已调用 try_reserve_exact 预留足够容量。
        unsafe {
            let rescues = &mut *domain.rescuers.get();
            for (worker, task) in prepared {
                rescues.push(Rescue {
                    worker,
                    join: UnsafeCell::new(Some(task.publish())),
                });
            }
        }
        // Step 4: 持全局状态锁将工作域标记为已发布（published = true）。
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有全局自旋锁 STATE_LOCK，独占修改 domain.state 中的 published 标志。
            unsafe {
                (*domain.state.get()).published = true;
            }
        }
        Ok(Self { domain })
    }

    /// 同步排空当前工作队列中在本次调用进入前已提交的所有工作项。
    ///
    /// 采用双色翻转机制，仅等待翻转前颜色批次中的工作项全部执行完毕，不会等待 `flush` 开始后新提交的工作项。
    /// 必须在可睡眠上下文中调用，且严禁在同域有序工作项或同域回调中自等待。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "工作队列测试核对域颜色边界只等待入口前的真实工作票"
        )
    )]
    pub fn flush(&self) {
        pool::assert_wait_allowed(&self.domain, ptr::null());
        self.domain.flush();
    }

    /// 永久关闭工作队列，排空所有已入队工作项，并回收全部预留 Worker 与救援线程。
    ///
    /// 调用者必须在可睡眠上下文中执行本方法。关闭流程依次完成：
    /// 1. 将工作域标记为 `closed`，拒绝任何新的工作项入队；
    /// 2. 调用 `flush` 排空所有已入队或正在执行的工作项；
    /// 3. 释放关联在本域上的所有预留 Worker，并等待管理者线程完成内核栈回收（`join`）；
    /// 4. 通知本域所有救援线程退出并逐个执行 `join`，确保所有内核线程彻底离开执行栈。
    pub fn close_sync(&self) {
        pool::assert_wait_allowed(&self.domain, ptr::null());
        let _serial = self.domain.close_serial.lock();
        // Step 1: 持锁置位 closed 标志，永久关闭入队大门。
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，所有对 DomainState 的读写均由该锁串行化。
            let state = unsafe { &mut *self.domain.state.get() };
            assert!(!state.closed, "work domain closed twice");
            state.closed = true;
        }
        // Step 2: 排空关闭前已入队的全部工作项，并解除未提交预留 Work 的专属线程预留。
        self.domain.flush();
        pool::release_domain_reservations(&self.domain);
        // Step 3: 等待本域所有预留 Worker 线程被所属 CPU 的 Manager 线程完全 join 回收。
        wait_until(&self.domain.finished, || {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK；reserved_workers 仅在同锁保护下由 Manager 线程在 join 返回后递减。
            unsafe { (*self.domain.state.get()).reserved_workers == 0 }
        });

        // Step 4: 设置 rescue_stop 退出标志并唤醒所有救援线程，最后逐一 join 回收其内核栈。
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，安全修改救援线程退出标志 rescue_stop。
            unsafe {
                (*self.domain.state.get()).rescue_stop = true;
            }
        }
        // SAFETY: domain.rescuers 在 try_new 发布后切片内容即保持不可变，此处仅获取共享引用遍历。
        let rescues = unsafe { &*self.domain.rescuers.get() };
        for rescue in rescues.iter() {
            rescue.worker.wake();
        }

        for rescue in rescues.iter() {
            // SAFETY: 当前线程持有 close_serial 睡眠锁，且 closed 状态禁止二次关闭，保证此处是唯一访问并取出
            // rescue.join 的执行流；其他路径仅读取不可变的 rescue.worker。
            let join = unsafe {
                let slot = &mut *rescue.join.get();
                slot.take().expect("missing rescuer join")
            };
            assert_eq!(join.join(), 0);
        }
    }

    /// 获取当前工作域中尚未被管理者线程完成 `join` 回收的预留 Worker 数量（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_reserved_workers(&self) -> usize {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，reserved_workers 的递增与 Manager join 后的递减均受该锁保护。
        unsafe { (*self.domain.state.get()).reserved_workers }
    }
}

impl Drop for WorkQueue {
    /// 析构工作队列句柄前断言已显式调用 `close_sync` 完成关闭。
    ///
    /// `Drop` 不会隐式阻塞等待后台任务结束；调用者必须先停止外部提交源并调用 `close_sync`。
    fn drop(&mut self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，安全读取 domain.state.closed 标志以校验生命周期契约。
        assert!(
            unsafe { (*self.domain.state.get()).closed },
            "work domain owner dropped before close"
        );
    }
}

impl Domain {
    /// 检查给定工作项在当前工作域并发配额下是否允许被领取执行。
    ///
    /// 普通工作项即使在回调中睡眠阻塞也依然占用域的 `active` 配额；
    /// 而带有专属预留 Worker（`reserved_worker`）的工作项拥有独立的内核栈，不受普通 `max_active` 配额限制。
    ///
    /// # Safety
    /// 调用者必须已持有全局自旋锁 `STATE_LOCK`。
    pub(super) unsafe fn eligible(&self, work: &Work) -> bool {
        // SAFETY: 调用者契约保证已持有 STATE_LOCK，可安全读取 DomainState。
        let state = unsafe { &*self.state.get() };
        work.reserved_worker.is_some()
            || state.active
                < if self.config.ordered {
                    1
                } else {
                    self.config.max_active
                }
    }

    /// 翻转当前工作域的入队颜色，并阻塞等待旧颜色批次中的所有工作项全部执行完毕。
    ///
    /// 由 `flush_serial` 睡眠互斥锁串行化，保证只有在前一轮颜色计数彻底归零后才会复用该颜色槽位。
    fn flush(&self) {
        let _serial = self.flush_serial.lock();
        let old = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，颜色索引与各色票据计数始终在同一把锁下同步读写。
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
            // SAFETY: 已持有 STATE_LOCK，安全读取旧颜色批次的剩余工作项计数。
            unsafe { (*self.state.get()).counts[old] == 0 }
        });
    }
}

impl Drop for Domain {
    /// 释放工作域底层结构时校验所有活跃工作项、预留配额及预留线程均已清零。
    ///
    /// 若在 `try_new` 期间因内存分配失败而提前析构（`published == false`），允许直接释放；
    /// 若已正式发布，则必定已经过 `close_sync` 完整回收。
    fn drop(&mut self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: Drop 期间当前线程独占该对象，且已持有 STATE_LOCK 以遵守统一状态访问规范。
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

/// 单个工作项（`Work`）的内部可变状态，所有字段均受全局自旋锁 `STATE_LOCK` 保护。
pub(super) struct WorkState {
    /// 侵入式双向链表后继指针，指向同一 CPU `WorkerPool` 待执行队列中的下一个 `Work`。
    next: *const Work,
    /// 侵入式双向链表前驱指针，指向同一 CPU `WorkerPool` 待执行队列中的前一个 `Work`。
    previous: *const Work,
    /// 当前处于排队状态（Queued）的票据序号；为 `0` 表示当前未在队列中排队。
    queued: u64,
    /// 当前正在执行回调（Running）的票据序号；为 `0` 表示当前没有回调正在运行。
    running: u64,
    /// 单调递增的票据序号生成器，每次成功入队时自增 1 并赋给 `queued`。
    next_ticket: u64,
    /// 在目标 CPU 工作池中的入队先后次序编号，用于救援线程按序扫描。
    order: u64,
    /// 当前排队票据（`queued`）所属的排空颜色批次（0 或 1）。
    color: usize,
    /// 当前执行中票据（`running`）所属的排空颜色批次（0 或 1），在回调结束时用于扣减对应颜色的计数。
    running_color: usize,
    /// 当前正在执行本工作项回调的 `Worker` 裸指针；未在执行时为 `null`。
    worker: *const Worker,
    /// 下一轮排队票据指定绑定的目标 `Worker` 裸指针。
    /// 若在回调运行期间再次入队（self-requeue），或者使用了预留 Worker，则固定绑定给该 Worker 执行。
    assigned: *const Worker,
    /// 工作项是否已被永久关闭（`close_sync` 或一次性工作项执行完毕后置位）。
    closed: bool,
    /// 工作项是否正处于同步取消或关闭流程中（暂拒新提交入队，直至正在运行的回调彻底退出）。
    stopping: bool,
    /// 若本工作项隶属于某个 `DelayedWork`，指向该外层 `DelayedWork` 的稳定裸指针；否则为 `null`。
    delayed_owner: *const DelayedWork,
}

/// 具有稳定内存地址、绑定固定工作域与目标 CPU 的可睡眠工作项。
///
/// 每个处于排队中（`queued`）和执行中（`running`）的状态各自持有一份独立的 `Arc<Work>` 强引用，
/// 确保异步执行与取消竞态下的生命周期安全。
pub struct Work {
    /// 所属的工作域。
    domain: Arc<Domain>,
    /// 固定绑定的目标 CPU 编号（严格有序域固定为 CPU0）。
    cpu: usize,
    /// 工作项被 Worker 领取后执行的业务回调函数。
    callback: fn(Pin<&Work>),
    /// 构造时传入的调用方自定义上下文标量（如对象指针或索引）。
    context: usize,
    /// 指向当前 `Work` 自身 `Arc` 负载的裸指针，用于在持锁入队时通过 `Arc::increment_strong_count` 增加强引用。
    owned_raw: *const Work,
    /// 可选的专属预留 Worker 线程；若存在，本工作项仅由该专属线程领取执行，不占用普通并发配额。
    reserved_worker: Option<Arc<Worker>>,
    /// 是否为一次性工作项；若为 `true`，在首次回调执行完毕后自动标记为永久关闭并释放预留 Worker。
    one_shot: bool,
    /// 受 `STATE_LOCK` 保护的链表指针与票据状态。
    state: UnsafeCell<WorkState>,
    /// 串行化 `cancel_sync`、`close_sync` 及 `DelayedWork::flush` 边界捕获的睡眠互斥锁。
    stop_serial: SleepMutex<()>,
    /// 当本工作项的排队或运行票据结束时用于唤醒等待者的等待队列。
    finished: WaitQueue,
    /// 标记为 `!Unpin`，因为 `Work` 处于侵入式双向链表中且包含自引用裸指针 `owned_raw`，地址严禁移动。
    _pinned: PhantomPinned,
}

// SAFETY: Work 通过 Pin<Arc<Work>> 保证内存地址稳定不移动，内部可变状态 WorkState 完全由全局自旋锁 STATE_LOCK 保护，
// 业务回调函数仅由唯一成功 claim 的 Worker 线程串行调用，因此可安全跨线程传递。
unsafe impl Send for Work {}
// SAFETY: 所有共享引用下的可变元数据访问均受 STATE_LOCK 保护，不可变字段在构造后固定不变，
// SleepMutex 与 WaitQueue 自身具备多线程同步安全性。
unsafe impl Sync for Work {}

/// 捕获特定时刻工作项的排队与运行票据序号快照，用于界定 `flush` 的有限等待范围。
///
/// 字段值为 `0` 表示在捕获快照的时刻对应槽位没有活跃票据。
#[derive(Clone, Copy)]
pub(crate) struct WorkBoundary {
    /// 捕获时刻正在运行的票据序号（0 表示无运行中票据）。
    running: u64,
    /// 捕获时刻正在排队的票据序号（0 表示无排队中票据）。
    queued: u64,
}

/// 从受锁保护的排队或运行状态中摘除出来的待释放票据守卫。
///
/// 为避免在持有自旋锁 `STATE_LOCK` 期间触发唤醒操作或执行对象析构，状态摘除函数会返回此结构，
/// 待调用者离开 `STATE_LOCK` 临界区后，由其 `Drop` 实现安全地完成唤醒与 `Arc` 强引用释放。
pub(super) struct RemovedTicket {
    /// 待释放一份 `Arc<Work>` 强引用的工作项裸指针（来自 `Work::owned_raw`）。
    work: *const Work,
    /// 若该工作项关联了 `DelayedWork`，则为同步摘除的一份 `Arc<DelayedWork>` 强引用裸指针；否则为 `null`。
    delayed: *const DelayedWork,
    /// 若取消排队票据时该票据已关联了特定 `Worker`，则持有该 `Worker` 的强引用以便在锁外唤醒它使其检查退休条件。
    worker: Option<Arc<Worker>>,
}

impl Drop for RemovedTicket {
    /// 在 `STATE_LOCK` 锁外执行等待队列唤醒、工作池通知，并归还 `Work` 与 `DelayedWork` 的 `Arc` 强引用。
    fn drop(&mut self) {
        // SAFETY: self.work 持有一份尚未释放的 Arc<Work> 强引用，保证在下方 Arc::from_raw 析构前对象始终有效；
        // 且当前处于 STATE_LOCK 锁外，可以安全唤醒等待队列与工作池。
        let work = unsafe { &*self.work };
        work.finished.wake_all();
        work.domain.finished.wake_all();
        for cpu in 0..cpu::count() {
            pool::notify(cpu);
        }
        if let Some(worker) = &self.worker {
            worker.wake();
        }
        // SAFETY: self.work 来自入队时通过 Arc::increment_strong_count(owned_raw) 增加的一份真实强引用，
        // 且每个 RemovedTicket 实例仅在此处恢复并释放一次。
        unsafe {
            drop(Arc::from_raw(self.work));
        }
        if !self.delayed.is_null() {
            // SAFETY: 非空的 self.delayed 来自 DelayedWork::claim_locked 或 cancel_locked 移交的唯一一份
            // Arc<DelayedWork> 强引用（由 schedule 时 increment_strong_count 创建），在此处精确释放一次。
            unsafe {
                drop(Arc::from_raw(self.delayed));
            }
        }
    }
}

impl Work {
    /// 创建一个绑定到指定工作队列与目标 CPU 的工作项（`Pin<Arc<Work>>`）。
    ///
    /// 若目标工作队列配置为严格有序域（`ordered == true`），无论传入的 `cpu` 为何值，都会自动归一化绑定到 CPU0。
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

    /// 创建一个自带专属预留 Worker 内核线程的工作项。
    ///
    /// 适用于块设备控制或重型 I/O 完成路径：在构造期即预先分配独立的 Worker 线程与内核栈，
    /// 运行时仅由该专属线程领取执行，不占用工作域的普通 `max_active` 并发配额，从而避免与其他可能长时间睡眠的回调争抢额度。
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

    /// 基于现有工作项所属的工作域与目标 CPU，创建一个带有专属预留 Worker 的一次性（One-Shot）关联工作项。
    ///
    /// 常用于一次性 `Bio` 完成回调：回调函数首次执行返回后，该工作项即自动关闭并释放预留的 Worker 线程，无需单独创建新的工作域。
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

    /// 获取当前工作项专属预留 Worker 对应的底层 `Task` 强引用（若尚未进入服务循环或已析构则返回 `None`，仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_reserved_task(&self) -> Option<Arc<crate::task::Task>> {
        self.reserved_worker
            .as_ref()
            .and_then(|worker| worker.ktest_task())
    }

    /// 检查当前工作项专属预留 Worker 的内核线程是否已由管理者线程完成 `join` 回收（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_reserved_joined(&self) -> bool {
        self.reserved_worker
            .as_ref()
            .is_some_and(|worker| worker.ktest_joined())
    }

    /// 在锁外完成 `Work` 实例及可选专属预留 Worker 的可失败内存分配与初始化。
    ///
    /// 只有在 `Work` 的固定内存地址与自引用指针写入完成后，才会将预留 Worker 发布到工作池与调度器。
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
        // SAFETY: raw 刚刚由 Arc::try_new 分配且尚未对外发布，当前线程持有唯一所有权；
        // 在此写入自引用指针 owned_raw 并封装为 Pin<Arc<Self>>，后续永不移动其内存位置。
        let owner = unsafe {
            (*raw.cast_mut()).owned_raw = raw;
            Pin::new_unchecked(Arc::from_raw(raw))
        };

        if reserved {
            // 预先分配专属 Worker 及底层 Task（含 16 页内核栈）；若分配失败，尚未发布的 owner 会走普通 Drop 自动释放。
            let (worker, prepared) = pool::prepare_reserved(cpu, &owner.domain, raw)?;
            // SAFETY: 当前 Work 尚未返回给调用者、无任何排队或运行票据，且预留 Worker 尚未发布到调度器；
            // 在固定内存位置写入一次 reserved_worker 不会移动 Work 本体，也无并发访问冲突。
            unsafe {
                (*raw.cast_mut()).reserved_worker = Some(Arc::clone(&worker));
            }
            pool::publish_worker(worker, prepared);
        }
        Ok(owner)
    }

    /// 先校验当前调用上下文未发生自等待死锁，再获取用于串行化停止与排空边界捕获的 `stop_serial` 睡眠互斥锁。
    pub(super) fn lock_stop_serial(self: Pin<&Self>) -> SleepMutexGuard<'_, ()> {
        pool::assert_wait_allowed(&self.domain, self.owned_raw);
        self.get_ref().stop_serial.lock()
    }

    /// 返回构造时传入的调用方上下文标量值。
    pub fn context(&self) -> usize {
        self.context
    }

    /// 返回本工作项固定绑定的目标 CPU 编号（内部关联的延迟定时器也绑定在同一 CPU）。
    pub fn cpu(&self) -> usize {
        self.cpu
    }

    /// 将工作项提交到目标 CPU 的工作池中排队等待执行。
    ///
    /// - 若工作项或所属工作域已永久关闭，返回 `Err(QueueError::Closed)`；
    /// - 若工作项当前已有排队票据（`queued != 0`）或正处于同步取消（`stopping`）过程中，返回 `Ok(false)` 且不重复入队；
    /// - 若成功新增排队票据，唤醒目标 CPU 工作池并返回 `Ok(true)`。
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

    /// 在持有 `STATE_LOCK` 的前提下，检查工作项本身或所属工作域是否已永久关闭。
    pub(super) fn is_closed_locked(self: Pin<&Self>) -> bool {
        // SAFETY: 调用者已持有全局自旋锁 STATE_LOCK，WorkState 与 DomainState 均受该锁保护。
        unsafe { (*self.state.get()).closed || (*self.domain.state.get()).closed }
    }

    /// 在持有 `STATE_LOCK` 的前提下，检查工作项当前是否允许接受提交（未永久关闭且未处于 `stopping` 同步停止期间）。
    pub(super) fn accepts_locked(self: Pin<&Self>) -> bool {
        // SAFETY: 调用者已持有 STATE_LOCK，安全读取 stopping 状态位。
        !self.is_closed_locked() && !unsafe { (*self.state.get()).stopping }
    }

    /// 在持有 `STATE_LOCK` 的前提下，综合判断当前工作项是否可以新增排队票据。
    pub(super) fn can_queue_locked(self: Pin<&Self>) -> Result<bool, QueueError> {
        if self.is_closed_locked() {
            return Err(QueueError::Closed);
        }
        // SAFETY: 调用者已持有 STATE_LOCK，安全读取 stopping 与 queued 字段。
        Ok(unsafe { !(*self.state.get()).stopping && (*self.state.get()).queued == 0 })
    }

    /// 在持有 `STATE_LOCK` 的前提下执行入队记账：分配新票据序号、递增域颜色计数、增加 `Arc<Work>` 强引用并挂入 CPU 工作池链表。
    ///
    /// 此函数仅修改受锁保护的内存状态，不执行内存分配或线程唤醒（唤醒由调用者在锁外调用 `notify` 完成）。
    pub(super) fn queue_locked(self: Pin<&Self>) -> Result<bool, QueueError> {
        if !self.can_queue_locked()? {
            return Ok(false);
        }
        // SAFETY: 调用者已持有 STATE_LOCK；self.owned_raw 指向当前 Pin<Arc<Work>> 管理的有效负载，
        // 入队时递增一次强引用计数，该引用将由后续 Worker 执行完成或取消操作在锁外通过 RemovedTicket 释放。
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

    /// 在 `DelayedWork::try_new` 构造期间（尚未对外发布前），将内层 `Work` 关联到外层 `DelayedWork` 指针。
    pub(super) fn set_delayed_owner(self: Pin<&Self>, owner: *const DelayedWork) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 已持有 STATE_LOCK，且调用者保证此时 DelayedWork 与 Work 均未对外发布，该字段仅初始化一次。
        unsafe {
            let state = &mut *self.state.get();
            assert!(state.delayed_owner.is_null());
            assert_eq!(state.queued, 0);
            assert_eq!(state.running, 0);
            state.delayed_owner = owner;
        }
    }

    /// 在 `STATE_LOCK` 锁外通知目标 CPU 的工作池唤醒空闲 Worker、管理者线程或内存回收救援线程。
    pub(super) fn notify(self: Pin<&Self>) {
        pool::notify(self.cpu);
    }

    /// 在持有 `STATE_LOCK` 的前提下撤销当前排队中的票据（`queued`），并返回待在锁外析构的 [`RemovedTicket`]。
    pub(super) fn cancel_queued_locked(self: Pin<&Self>) -> Option<RemovedTicket> {
        // SAFETY: 调用者已持有 STATE_LOCK，从工作池链表摘除节点与扣减域颜色计数在同一临界区内原子完成，不会与 Worker claim 交错。
        unsafe {
            let queued = (*self.state.get()).queued;
            if queued == 0 {
                return None;
            }
            let worker =
                pool::assigned_worker(self.cpu, (*self.state.get()).assigned, &self.domain);
            pool::unlink(self.cpu, self.owned_raw);
            // pool::unlink 已结束对 self.state 的内部可变借用，此处重新获取可变引用以清理票据状态。
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

    /// 在持有 `STATE_LOCK` 的前提下捕获当前 `running` 和 `queued` 两个槽位的票据序号快照。
    pub(super) fn capture_locked(self: Pin<&Self>) -> WorkBoundary {
        // SAFETY: 调用者已持有 STATE_LOCK，安全读取当前运行与排队票据序号。
        unsafe {
            let state = &*self.state.get();
            WorkBoundary {
                running: state.running,
                queued: state.queued,
            }
        }
    }

    /// 获取自旋锁并捕获当前工作项的 `running` 与 `queued` 票据边界快照，供后续 `flush_boundary` 等待使用。
    ///
    /// 调用者可在持有自身业务锁时先调用本函数固定等待边界，释放业务锁后再调用 `flush_boundary`，以避免与回调函数产生死锁。
    pub(crate) fn capture_boundary(self: Pin<&Self>) -> WorkBoundary {
        let _lock = STATE_LOCK.lock();
        self.capture_locked()
    }

    /// 阻塞等待 `boundary` 快照中记录的特定 `running` 和 `queued` 票据全部执行完毕或被取消。
    ///
    /// 只精确比对快照捕获时的票据序号，不会等待捕获边界之后新提交（含回调自我重入队）的票据。
    /// 返回 `true` 表示快照中至少包含一份活跃票据并完成了等待；返回 `false` 表示捕获时刻无活跃票据。
    pub(crate) fn flush_boundary(self: Pin<&Self>, boundary: WorkBoundary) -> bool {
        pool::assert_wait_allowed(&self.domain, self.owned_raw);
        let affected = boundary.running != 0 || boundary.queued != 0;
        wait_until(&self.finished, || {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，精确比对当前槽位中的票据序号是否仍等于快照中的序号。
            let state = unsafe { &*self.state.get() };
            let matches = |ticket: u64| {
                ticket != 0 && (ticket == boundary.running || ticket == boundary.queued)
            };
            !matches(state.running) && !matches(state.queued)
        });
        affected
    }

    /// 同步排空当前工作项在本次调用时刻已存在的排队与运行票据，不会等待后续新提交或回调自重入队的票据。
    pub fn flush(self: Pin<&Self>) -> bool {
        let boundary = self.capture_boundary();
        self.flush_boundary(boundary)
    }

    /// 启动同步取消或永久关闭流程的第一阶段：获取串行锁、关闭提交入口并立即撤销排队中的票据。
    ///
    /// 返回 [`StopGuard`] 供调用者（如 `DelayedWork::stop`）在等待正在运行的回调结束前，先同步停止外层定时器。
    pub(super) fn begin_stop(self: Pin<&Self>, close: bool) -> StopGuard<'_> {
        pool::assert_wait_allowed(&self.domain, self.owned_raw);
        let serial = self.get_ref().stop_serial.lock();
        let (affected, removed) = {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK，独占修改 closed 与 stopping 状态位并查询关联的 DelayedWork。
            let state = unsafe { &mut *self.state.get() };
            if close {
                assert!(!state.closed, "work closed twice");
                state.closed = true;
            }
            state.stopping = true;
            // SAFETY: delayed_owner 在构造期写入后不可变，且外层 DelayedWork 的调用者或异步票据保证其内存存活；
            // 在同一把 STATE_LOCK 下检查 DelayedWork 是否有挂起状态，避免定时器到期回调看到 stopping 后清票导致漏记。
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

    /// 同步取消当前排队中的工作票据，并阻塞等待正在执行的回调彻底返回；取消完成后该 `Work` 仍可继续提交复用。
    ///
    /// 返回 `true` 表示本次取消撤销了排队票据或等待了正在运行的回调。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "工作队列测试验证同步取消只撤本轮 pending 并保留对象复用能力"
        )
    )]
    pub fn cancel_sync(self: Pin<&Self>) -> bool {
        self.begin_stop(false).finish()
    }

    /// 永久关闭当前工作项，撤销排队票据并等待正在运行的回调与专属预留 Worker 线程彻底退出回收。
    pub fn close_sync(self: Pin<&Self>) -> bool {
        let affected = self.begin_stop(true).finish();
        if let Some(worker) = &self.reserved_worker {
            pool::close_reservation(worker);
        }
        affected
    }
}

/// 分阶段同步停止流程的守卫对象，由 [`Work::begin_stop`] 创建并必须通过 [`StopGuard::finish`] 显式消费。
pub(super) struct StopGuard<'a> {
    /// 目标工作项的固定引用。
    work: Pin<&'a Work>,
    /// 持有的 `Work::stop_serial` 睡眠互斥锁守卫，确保整个停止流程串行执行。
    serial: Option<SleepMutexGuard<'a, ()>>,
    /// 本次停止操作在入口处是否观测到了排队中、运行中或延迟定时器挂起的工作票据。
    affected: bool,
    /// 标记是否已调用 `finish()` 完成等待，防止守卫被中途直接丢弃。
    finished: bool,
}

impl StopGuard<'_> {
    /// 完成同步停止的第二阶段：阻塞等待正在执行的回调函数彻底返回，随后清除 `stopping` 标志并释放串行锁。
    pub(super) fn finish(mut self) -> bool {
        wait_until(&self.work.finished, || {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 已持有 STATE_LOCK；stopping 已阻止新票据入队，此处检查 running 槽位是否已归零。
            unsafe { (*self.work.state.get()).running == 0 }
        });
        {
            let _lock = STATE_LOCK.lock();
            // SAFETY: 当前线程持有唯一的 stop_serial 锁且已持有 STATE_LOCK，安全恢复 stopping 标志（若为 close 则 closed 保持为 true）。
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
    /// 断言调用者已显式调用 `finish()` 完成了同步等待，禁止通过丢弃守卫跳过等待。
    fn drop(&mut self) {
        assert!(self.finished, "unfinished work stop guard");
    }
}

impl Drop for Work {
    /// 析构工作项时断言无任何排队或运行中的票据残留，并在锁外通知专属预留 Worker 线程解除预留并退休。
    fn drop(&mut self) {
        let _lock = STATE_LOCK.lock();
        // SAFETY: 每份活跃票据均持有独立的 Arc<Work> 强引用，能够进入 Drop 说明引用计数已归零，持 STATE_LOCK 断言状态为空。
        let state = unsafe { &*self.state.get() };
        assert_eq!(state.queued, 0, "dropping pending work");
        assert_eq!(state.running, 0, "dropping running work");
        drop(_lock);
        if let Some(worker) = &self.reserved_worker {
            pool::release_reservation(worker);
        }
    }
}

/// 在指定等待队列 `source` 上循环注册并睡眠，直到条件闭包 `done` 返回 `true`。
///
/// 采用标准的“先注册等待节点、再复查条件、不满足才 `park`”模式，防止唤醒信号在条件检查与睡眠之间丢失；
/// 每轮循环结束后立即注销栈上等待节点，保证同一时刻最多只有一个活跃等待登记。
pub(super) fn wait_until(source: &WaitQueue, mut done: impl FnMut() -> bool) {
    if done() {
        return;
    }

    loop {
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: source 引用与栈上固定的 node 在本轮循环内始终有效，且在 node 离开作用域前通过 drop(registration) 显式注销。
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

/// 在引导处理器（BSP）完成内存管理初始化后、启动中断与从核（AP）之前，预先创建并发布所有 CPU 的基础 Worker 与 Manager 线程。
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

    /// 工作队列子系统当前存活的核心对象数量统计（仅统计未析构的实例负载，不含 `Weak` 控制块，仅用于内核测试）。
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct MetadataStats {
        /// 当前系统中存活的 `Task` 实例总数。
        pub tasks: usize,
        /// 当前存活的 `Worker` 身份结构总数。
        pub workers: usize,
        /// 当前存活的 `Domain` 工作域实例总数。
        pub domains: usize,
    }

    /// 读取当前存活的任务、Worker 与工作域实例计数（仅用于内核测试）。
    pub(crate) fn metadata_stats() -> MetadataStats {
        MetadataStats {
            tasks: crate::task::test_live_tasks(),
            workers: pool::LIVE_WORKERS.load(Ordering::Relaxed),
            domains: LIVE_DOMAINS.load(Ordering::Relaxed),
        }
    }

    /// 在持有 `STATE_LOCK` 期间写入一条内核日志，用于测试日志子系统在自旋锁内的无死锁行为（仅用于内核测试）。
    pub(crate) fn test_klog() -> u64 {
        let _lock = STATE_LOCK.lock();
        crate::klog::write(crate::klog::Level::Debug, format_args!("X25 pool locked"))
    }

    /// 单个 `Work` 实例在某一时刻的运行状态快照（仅用于内核测试）。
    pub(crate) struct WorkSnapshot {
        /// 是否有票据正在队列中等待执行。
        pub queued: bool,
        /// 是否有回调正在 Worker 上执行。
        pub running: bool,
        /// 工作项或所属工作域是否已关闭。
        pub closed: bool,
        /// 当前正在执行该工作项的 Worker 所属内核线程 `TaskId`（未在执行时为 `0`）。
        pub worker: usize,
    }

    /// 获取 `STATE_LOCK` 并读取指定工作项的状态快照（仅用于内核测试）。
    pub(crate) fn snapshot(work: Pin<&Work>) -> WorkSnapshot {
        let _lock = STATE_LOCK.lock();
        snapshot_locked(work)
    }

    /// 在调用者已持有 `STATE_LOCK` 的前提下读取工作项状态快照，供 `delayed` 测试模块在同一把锁下联合观察（仅用于内核测试）。
    pub(super) fn snapshot_locked(work: Pin<&Work>) -> WorkSnapshot {
        // SAFETY: 调用者已持有 STATE_LOCK；当 state.worker 非空时，正在执行回调的内核线程持有该 Worker 的强引用使其保持有效。
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
