//! 内核任务（Task）控制块、生命周期管理与完成同步。
//!
//! ## 模块职责与架构定位
//! 本模块定义内核可调度执行单元 `Task` 及其关联的生命周期、内存回收重入限制（`ReclaimScope`）、创建预备句柄（`PreparedTask`）与退出等待句柄（`JoinHandle` / `JoinObserver`）。
//! - **上游调用者**：内核各子系统（通过 `spawn_kernel` / `spawn_kernel_on` 创建后台任务）、工作队列 `crate::workqueue`（通过 `prepare_worker_on` 创建绑定 Worker 的工作线程）、以及进程/线程管理子模块（`clone`、`exec`、`process`、`signal`、`futex`、`user`）。
//! - **下游依赖**：`crate::mm`（分配任务独占的连续物理页内核栈 `OwnedPages`）、`crate::sched`（将准备就绪的任务发布到全局调度器并管理状态切换）、`crate::wait`（在 `JoinState` 中唤醒等待任务退出的观察者）。
//!
//! ## 核心设计与生命周期状态机
//! 1. **两阶段创建（Prepare & Publish）**：在 `prepare()` 阶段完成闭包装箱、`JoinState` 分配与内核栈页分配；若中途内存不足，所有已分配资源在锁外直接通过 RAII 释放。只有全部资源就绪后，才调用 `PreparedTask::publish()` 将 `Arc<Task>` 移交调度器。
//! 2. **控制块与完成状态分离**：`Task` 自身包含较大的内核栈与执行上下文，而 `JoinHandle` / `JoinObserver` 仅持有轻量的 `Arc<JoinState>`。当任务退出并切回调度栈后，`Task` 的内核栈和闭包立即被释放，而退出码永久保存在 `JoinState` 中供后续 `join()` 读取。
//!
//! ```text
//!   prepare()                   publish()                 sched::pick()
//! [PreparedTask] -----------> [State::Ready] <---------> [State::Running(cpu)]
//! (分配栈/入口/JoinState)     (挂入 live 与 ready 链)            |
//!                                   ^                           | prepare_switch(intent)
//!                                   |                           v
//!                            [State::Blocked] <------ [State::SwitchingOut(cpu, intent)]
//!                            (Park 且尚未收到通知)               |
//!                                                               | SwitchIntent::Exit(code)
//!                                                               v
//!                                                       [State::Exited]
//!                                                  (摘离 live 链 -> 释放栈与入口 -> JoinState::complete)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `Task::scheduling`（`UnsafeCell<Scheduling>`）：**仅允许在持有全局调度锁 `SCHED_LOCK` 时访问**。
//! - `Task::ctx`（`UnsafeCell<TaskContext>`）：**仅允许当前拥有该任务 `Running` / `SwitchingOut` 执行权的唯一 CPU 及切换回的本地调度栈访问**。
//! - `Task::resources`（`UnsafeCell<Resources>`）：入口闭包 `entry` 在首次进入 `task_trampoline` 时取走；内核栈 `stack` 在任务彻底切回本 CPU 静态调度栈（`finish` 摘除活跃登记）后才被移出并在锁外释放。
//! - `Task::user`（`SpinLock<Option<Arc<UserThread>>>`）：自旋锁临界区极短，仅用于克隆或取走 `Arc<UserThread>`，严禁在持有 `task.user` 锁时获取 `SCHED_LOCK` 或睡眠锁。

pub(crate) mod clone;
pub(crate) mod exec;
pub(crate) mod futex;
pub(crate) mod process;
pub(crate) mod signal;
pub(crate) mod time;
mod user;
pub(crate) use user::{prepare_program, Credentials, ProgramError, UserRuntime, UserThread};

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::alloc::AllocError;
use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::pin::pin;
use core::ptr;
#[cfg(feature = "ktest")]
use core::sync::atomic::AtomicPtr;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, AtomicU8, AtomicUsize, Ordering};

use crate::mm::{alloc_pages, OwnedPages};
use crate::sched::context::TaskContext;
use crate::sync::{IrqSaveGuard, SpinLock};
use crate::time::Duration;
use crate::wait::{WaitId, WaitNode, WaitQueue, Waiter};

/// 全局递增的任务 ID 分配器（从 1 开始，0 保留表示当前 CPU 无运行任务）。
static NEXT_ID: AtomicUsize = AtomicUsize::new(1);
/// 测试模式下统计当前存活（已构造且尚未执行 `Task::drop`）的 `Task` 实例数量。
#[cfg(feature = "ktest")]
static LIVE_TASKS: AtomicUsize = AtomicUsize::new(0);

/// 内核任务的全局唯一标识符。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TaskId(usize);
impl TaskId {
    /// 返回任务 ID 对应的非零整数值（`0` 保留给未运行普通任务的调度栈上下文）。
    pub fn as_usize(self) -> usize {
        self.0
    }

    /// 从每 CPU 缓存的原始整数恢复 `TaskId`；若值为 `0`（代表当前处于调度栈或空闲态）则返回 `None`。
    pub(crate) fn from_raw(value: usize) -> Option<Self> {
        if value == 0 {
            None
        } else {
            Some(Self(value))
        }
    }
}

/// 直接内存回收限制标志位：禁止在内存回收路径中重入文件系统（防止递归获取文件系统锁或等待日志写回导致死锁）。
const RECLAIM_NOFS: u8 = 1;
/// 直接内存回收限制标志位：禁止在内存回收路径中发起任何块设备 I/O（防止在块层/分配路径中递归触发 I/O 死锁）。
const RECLAIM_NOIO: u8 = 2;

/// 每个内核任务独占内核栈的伙伴系统分配阶数（`2^4 = 16` 个物理页，即 64 KiB）。
const KERNEL_STACK_ORDER: usize = 4;

/// 当前任务允许直接内存回收（Direct Reclaim）进入的子系统范围。
///
/// 该模式仅限制内存分配失败触发的同步页面回收路径，不影响任务自身显式发起的正常文件系统或块设备 I/O。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ReclaimMode {
    /// 普通模式：允许直接内存回收执行文件系统元数据清理与块设备写回。
    Normal,
    /// 禁止文件系统重入（对应 `GFP_NOFS`）：允许回收干净页，但不得调用文件系统回调。
    NoFs,
    /// 禁止任何 I/O 重入（对应 `GFP_NOIO`）：禁止在回收路径中进入文件系统或提交新的块层 I/O。
    NoIo,
}

impl ReclaimMode {
    /// 读取当前可睡眠任务的直接内存回收模式。
    ///
    /// 只能在普通任务的可调度上下文中调用（内部会调用 `assert_switchable` 校验）；任务跨核迁移不会丢失该状态。
    pub(crate) fn current() -> Self {
        crate::sched::assert_switchable();
        let flags = current_task().reclaim.load(Ordering::Acquire);
        if flags & RECLAIM_NOIO != 0 {
            Self::NoIo
        } else if flags & RECLAIM_NOFS != 0 {
            Self::NoFs
        } else {
            Self::Normal
        }
    }
}

/// 基于 RAII 的任务直接内存回收限制作用域守卫。
///
/// 仅记录进入本层作用域时**新置位**的标志位（`added`），支持嵌套使用；退出作用域时只清除本层新增的限制位，不会破坏外层已有的限制。
/// 该守卫绑定创建时的当前任务，不可跨任务传递（`!Send`）。
pub(crate) struct ReclaimScope {
    /// 绑定该回收限制作用域的当前任务强引用。
    task: Arc<Task>,
    /// 本次进入作用域时实际从 `0` 变为 `1` 的标志位掩码，用于在 `Drop` 时精确还原。
    added: u8,
    /// 标记本守卫为任务本地对象（`!Send` / `!Sync`），防止跨线程转移。
    _task_local: PhantomData<*mut ()>,
}

impl ReclaimScope {
    /// 进入 `NoFs` 作用域：在持有文件系统锁或事务期间调用，防止后续内存分配触发直接回收时再次重入文件系统造成死锁。
    pub(crate) fn nofs() -> Self {
        Self::enter(RECLAIM_NOFS)
    }

    /// 进入 `NoIo` 作用域：在块设备驱动或 I/O 提交路径中调用，同时设置 `RECLAIM_NOFS | RECLAIM_NOIO`，禁止直接回收进入文件系统或发起新 I/O。
    pub(crate) fn noio() -> Self {
        Self::enter(RECLAIM_NOFS | RECLAIM_NOIO)
    }

    /// 为当前任务原子开启 `requested` 回收限制位，并记录本次新增的位掩码。
    fn enter(requested: u8) -> Self {
        crate::sched::assert_switchable();
        let task = current_task();
        let previous = task.reclaim.fetch_or(requested, Ordering::AcqRel);
        Self {
            task,
            added: requested & !previous,
            _task_local: PhantomData,
        }
    }
}

impl Drop for ReclaimScope {
    /// 退出作用域时校验当前仍在原任务中运行，并原子清除本层守卫新增的回收限制位。
    fn drop(&mut self) {
        assert_eq!(
            current_task().id(),
            self.task.id(),
            "reclaim scope changed task"
        );
        self.task.reclaim.fetch_and(!self.added, Ordering::Release);
    }
}

/// 新任务的初始 CPU 放置策略。
pub(crate) enum Placement {
    /// 允许在系统中任意在线 CPU 上调度运行。
    Any,
    /// 固定绑定在指定的单个物理 CPU（`usize` 为逻辑 CPU 编号）上运行。
    Cpu(usize),
}

/// 任务主动切出 CPU（进入 `State::SwitchingOut`）时的调度意图。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SwitchIntent {
    /// 主动让出当前 CPU（`yield_now`），切回调度栈后立即重新标记为 `State::Ready` 并放回就绪队列。
    Yield,
    /// 阻塞等待指定序号（`WaitId`）的事件通知；切回调度栈后若期间未收到通知则进入 `State::Blocked`，否则直接回到 `State::Ready`。
    Park(WaitId),
    /// 任务执行完毕退出并携带返回值（`i32`）；切回调度栈后摘除活跃登记并释放内核栈。
    Exit(i32),
}

/// 任务在调度器中的生命周期状态。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum State {
    /// 已就绪，正挂在全局就绪队列（或即将挂入）等待某个允许的 CPU 选取执行。
    Ready,
    /// 正在编号为 `usize` 的 CPU 上独占执行。
    Running(usize),
    /// 正在编号为 `usize` 的 CPU 上保存寄存器上下文并切回调度栈，携带本次切出的意图 `SwitchIntent`。
    SwitchingOut(usize, SwitchIntent),
    /// 正在等待事件唤醒，不在就绪队列中，也不占用任何 CPU。
    Blocked,
    /// 已彻底切离内核栈并退出，不再参与任何调度。
    Exited,
}

/// 受全局调度锁 `SCHED_LOCK` 保护的任务调度元数据与侵入式双向链表指针。
///
/// 一个 `Task` 同时包含两条独立的侵入式链表节点：
/// - `live_*`：所有已发布且尚未退出的活跃任务链表（链表中每个节点代表调度器持有的 1 个 `Arc<Task>` 强引用）；
/// - `ready_*`：当前处于 `State::Ready` 等待被 CPU 选中的就绪任务 FIFO 队列。
pub(crate) struct Scheduling {
    /// 当前任务的调度状态。
    pub state: State,
    /// 允许执行该任务的 CPU 亲和性掩码集合。
    pub allowed: crate::cpu::CpuSet,
    /// 标记是否有外部调用者（`set_affinity`）正在等待该任务离开不再允许的旧 CPU。
    pub affinity_pending: bool,
    /// 该任务最近一次分配的等待轮次编号（`WaitId`），用于防止过期的延迟唤醒通知误唤醒新一轮等待。
    pub last_wait_id: WaitId,
    /// 当前正在进行的等待轮次状态（仅在 `begin_wait` 与 `end_wait` 之间为 `Some`）。
    pub active_wait: Option<ActiveWait>,
    /// 活跃任务双向链表（`Scheduler::live_head`）中的前驱节点指针。
    pub live_previous: *const Task,
    /// 活跃任务双向链表（`Scheduler::live_head`）中的后继节点指针。
    pub live_next: *const Task,
    /// 就绪任务双向链表（`Scheduler::ready_head`）中的前驱节点指针。
    pub ready_previous: *const Task,
    /// 就绪任务双向链表（`Scheduler::ready_head`）中的后继节点指针。
    pub ready_next: *const Task,
    /// 测试模式下强制指定下一次 `pick` 必须选中的目标 CPU 编号（选中一次后自动清零）。
    #[cfg(feature = "ktest")]
    pub next_cpu: Option<usize>,
}

/// 任务单次阻塞等待（`Waiter`）的活跃轮次记录。
#[derive(Clone, Copy)]
pub(crate) struct ActiveWait {
    /// 本轮等待的唯一递增编号。
    pub id: WaitId,
    /// 标记本轮等待期间是否已经收到过有效的 `sched::notify` 唤醒通知。
    pub notified: bool,
}

/// 内核任务入口闭包的装箱类型别名，返回 `i32` 退出码。
pub(crate) type Entry = Box<dyn FnOnce() -> i32 + Send + 'static>;

/// 任务运行所需的独占执行资源（内核栈与一次性入口闭包）。
///
/// - `entry` 在任务首次进入 `task_trampoline` 时被 `take()` 取出并执行；
/// - `stack` 在任务整个运行与阻塞期间保持有效，直到任务以 `SwitchIntent::Exit` 切回 CPU 静态调度栈后，才在 `Scheduler::finish` 中移出并在锁外释放。
pub(crate) struct Resources {
    /// 任务独占的连续物理页内核栈（64 KiB）。
    pub stack: Option<OwnedPages>,
    /// 任务首次启动时执行的入口闭包。
    pub entry: Option<Entry>,
}

/// 独立于 `Task` 控制块的退出状态同步块，供 `JoinHandle` 和 `JoinObserver` 共享观察。
pub(crate) struct JoinState {
    /// 任务入口闭包返回的退出码（在 `completed` 置为 `true` 前写入）。
    result: AtomicI32,
    /// 标记任务是否已彻底切离内核栈、释放运行资源并完成退出（以 `Release` 语义发布）。
    completed: AtomicBool,
    /// 等待该任务退出的阻塞任务队列。
    waiters: WaitQueue,
}

impl JoinState {
    /// 在任务的内核栈与运行资源已于调度锁外彻底释放后，发布退出码 `result` 并唤醒所有等待 `join` 的任务。
    ///
    /// 完成状态一旦发布即永久保持，支持多个 `JoinObserver` 反复读取。
    pub(crate) fn complete(&self, result: i32) {
        self.result.store(result, Ordering::Relaxed);
        self.completed.store(true, Ordering::Release);
        self.waiters.wake_all();
    }

    /// 非阻塞地检查任务是否已完成退出；若已完成（`Acquire` 观察到 `completed == true`），则返回其退出码。
    fn try_join(&self) -> Option<i32> {
        self.completed
            .load(Ordering::Acquire)
            .then(|| self.result.load(Ordering::Relaxed))
    }

    /// 阻塞当前任务，直到目标任务完成退出并返回退出码。
    ///
    /// 采用标准的“先注册等待队列节点、再复查 `try_join` 条件”两阶段等待模式，消除检查与睡眠之间的唤醒丢失窗口。
    fn wait_result(&self) -> i32 {
        crate::sched::assert_switchable();
        loop {
            if let Some(result) = self.try_join() {
                return result;
            }

            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `&self` 借用保证了 `self.waiters` 等待队列在当前循环迭代内始终存活；
            // 2. `node` 通过 `pin!` 固定在当前任务栈帧上，`_registration` 守卫在栈帧局部变量的逆序析构中必然先于 `node` 和 `waiter` 执行，确保节点在出栈销毁前已从 `waiters` 链表中摘除。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if let Some(result) = self.try_join() {
                return result;
            }
            waiter.park();
        }
    }
}

/// 内核可调度任务控制块（Task Control Block）。
pub struct Task {
    /// 全局唯一且不可变的任务标识符。
    id: TaskId,
    /// 该任务在各 CPU 上累计已结算的实际执行时间（单位：纳秒），由当前执行 CPU 在关中断时更新。
    execution_ns: AtomicU64,
    /// 当前任务的直接内存回收重入限制位掩码（`RECLAIM_NOFS` / `RECLAIM_NOIO`）。
    reclaim: AtomicU8,
    /// 任务切出时保存的被调用者保存寄存器上下文，仅由拥有当前执行权的 CPU 访问。
    pub(crate) ctx: UnsafeCell<TaskContext>,
    /// 调度器状态与侵入式链表指针，仅在持有全局 `SCHED_LOCK` 时访问。
    pub(crate) scheduling: UnsafeCell<Scheduling>,
    /// 任务的独占内核栈与入口闭包资源，仅由当前执行 CPU 及退出切回的调度栈访问。
    pub(crate) resources: UnsafeCell<Resources>,
    /// 任务退出码与完成等待队列状态，与 `JoinHandle` / `JoinObserver` 共享。
    pub(crate) join: Arc<JoinState>,
    /// 若本任务是工作队列（Workqueue）的专属工作线程，则保存其对应的 `Worker` 描述符。
    pub(crate) worker: Option<Arc<crate::workqueue::Worker>>,
    /// 若本任务承载用户态线程，则保存关联的 `UserThread` 引用（由自旋锁保护，任务退出时取走释放）。
    pub(crate) user: SpinLock<Option<Arc<UserThread>>>,
    /// 测试框架（ktest）在当前任务上挂载的断言或异常边界上下文指针。
    #[cfg(feature = "ktest")]
    pub(crate) ktest_boundary: AtomicPtr<()>,
    /// 测试框架（ktest）在当前任务上挂载的测试作用域状态指针。
    #[cfg(feature = "ktest")]
    pub(crate) ktest_scope: AtomicPtr<()>,
}

// SAFETY:
// 1. `scheduling` 仅在持有全局自旋锁 `SCHED_LOCK` 时访问；
// 2. `ctx` 仅由持有该任务唯一 `Running` / `SwitchingOut` 执行权的 CPU 在切换路径中访问；
// 3. `resources` 中的入口闭包满足 `Send`，且内核栈物理页所有权仅在任务退出并切回调度栈后移出一次；
// 4. `user` 由 `SpinLock` 保护，其余字段均为原子类型或不可变对象，因此 `Task` 可安全跨线程转移所有权。
unsafe impl Send for Task {}
// SAFETY:
// 对外通过共享引用 `&Task` 仅能直接读取不可变 `id`、原子字段（`execution_ns`、`reclaim`）或经由 `SpinLock` 保护的 `user`；
// 内部 `UnsafeCell` 字段（`ctx`、`scheduling`、`resources`）的并发访问均由 `SCHED_LOCK` 或单 CPU 独占执行权协议严格串行化。
unsafe impl Sync for Task {}

impl Task {
    /// 获取当前任务绑定的用户态线程对象（`UserThread`）的强引用克隆；纯内核任务或已退出的任务返回 `None`。
    pub(crate) fn user_thread(&self) -> Option<Arc<UserThread>> {
        self.user.lock().clone()
    }

    /// 返回该任务的全局唯一标识符 `TaskId`。
    pub fn id(&self) -> TaskId {
        self.id
    }

    /// 创建一个只读完成状态观察者 `JoinObserver`（仅引用轻量的 `JoinState`，不持有 `Task` 或内核栈的强引用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn join_observer(&self) -> JoinObserver {
        JoinObserver {
            id: self.id,
            join: Arc::clone(&self.join),
        }
    }

    /// 读取该任务在所有 CPU 上已结算的累计执行时长（不包含阻塞睡眠或就绪排队等待的时间）。
    pub fn execution_time(&self) -> Duration {
        Duration::from_nanos(self.execution_ns.load(Ordering::Relaxed))
    }

    /// 将本次执行片段耗时 `delta_ns`（单位：纳秒）累加到任务总执行时间中。
    ///
    /// 仅由当前独占执行该任务的 CPU 在关中断的统计结算边界（`accounting::CpuAccount::settle`）调用。
    pub(crate) fn account_execution(&self, delta_ns: u64) {
        let total = self.execution_ns.load(Ordering::Relaxed);
        self.execution_ns.store(
            total
                .checked_add(delta_ns)
                .expect("task execution time overflow"),
            Ordering::Relaxed,
        );
    }
}

#[cfg(feature = "ktest")]
impl Drop for Task {
    /// 当指向 `Task` 的最后一个 `Arc` 强引用被释放（或 `prepare` 中 `Arc::try_new` 分配失败回滚）时，递减存活任务计数 `LIVE_TASKS`。
    fn drop(&mut self) {
        LIVE_TASKS.fetch_sub(1, Ordering::Relaxed);
    }
}

/// 测试辅助函数：读取当前内存中尚未析构的 `Task` 控制块实例总数。
#[cfg(feature = "ktest")]
pub(crate) fn test_live_tasks() -> usize {
    LIVE_TASKS.load(Ordering::Relaxed)
}

/// 测试辅助函数：读取系统启动以来累计分配过的任务 ID 总数（包含创建后未发布即丢弃的任务）。
#[cfg(feature = "ktest")]
pub(crate) fn test_created_tasks() -> usize {
    NEXT_ID.load(Ordering::Relaxed) - 1
}

/// 任务创建者持有的完成等待句柄，用于等待任务退出并获取其 `i32` 返回值。
pub struct JoinHandle {
    /// 目标任务的唯一标识符。
    id: TaskId,
    /// 与目标任务共享的退出状态与等待队列。
    join: Arc<JoinState>,
}

impl JoinHandle {
    /// 非阻塞地轮询任务是否已退出；若返回 `Some(result)`，则保证该任务已彻底离开内核栈并释放了栈内存。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行测试非阻塞观察离栈完成，确认结果发布时执行栈已经归还"
        )
    )]
    pub fn try_join(&self) -> Option<i32> {
        self.join.try_join()
    }

    /// 消费当前句柄并阻塞等待目标任务彻底退出，返回其入口闭包的退出码。
    pub fn join(self) -> i32 {
        self.join.wait_result()
    }

    /// 返回该句柄对应的目标任务 ID（即使目标任务已退出依然有效）。
    pub fn task_id(&self) -> TaskId {
        self.id
    }

    /// 从当前句柄派生一个共享同一 `JoinState` 的轻量观察者 `JoinObserver`，不会延长 `Task` 控制块或内核栈的生命周期。
    pub(crate) fn observer(&self) -> JoinObserver {
        JoinObserver {
            id: self.id,
            join: Arc::clone(&self.join),
        }
    }
}

/// 轻量的任务完成状态观察者，仅持有 `TaskId` 与 `Arc<JoinState>`，允许多个观察者并发等待同一任务退出而不延长 `Task` 本体的生命周期。
pub(crate) struct JoinObserver {
    /// 被观察任务的唯一标识符。
    id: TaskId,
    /// 被观察任务的退出状态同步块。
    join: Arc<JoinState>,
}

impl JoinObserver {
    /// 非阻塞地检查被观察任务是否已完成退出并释放内核栈；若完成则返回 `Some(退出码)`。
    pub(crate) fn try_join(&self) -> Option<i32> {
        self.join.try_join()
    }

    /// 阻塞等待被观察任务彻底退出并返回其退出码（按引用调用，支持多处重复等待）。
    pub(crate) fn join(&self) -> i32 {
        self.join.wait_result()
    }

    /// 返回被观察任务的唯一标识符 `TaskId`。
    pub(crate) fn task_id(&self) -> TaskId {
        self.id
    }
}

/// 已完成全部内存分配（控制块、内核栈、入口闭包、`JoinState`）但尚未挂入全局调度器的预备任务。
///
/// 若在调用 `publish()` 之前直接丢弃该对象，其持有的 `Arc<Task>`、内核栈与入口闭包会通过常规 `Drop` 直接释放，无需经过调度器清理。
pub(crate) struct PreparedTask {
    /// 尚未发布到调度器的唯一 `Task` 强引用。
    task: Arc<Task>,
    /// 与该任务配对的完成等待句柄。
    handle: JoinHandle,
}

impl PreparedTask {
    /// 在任务发布到调度器之前，为其绑定关联的 `UserThread`，并在 `UserThread` 中登记指向本 `Task` 的弱引用（`Weak<Task>`）。
    pub(crate) fn bind_user(&self, user: Arc<UserThread>) {
        user.bind_executor(Arc::downgrade(&self.task));
        let mut slot = self.task.user.lock();
        assert!(slot.is_none(), "prepared task already has a user binding");
        *slot = Some(user);
    }

    /// 在任务发布前获取其已分配的 `TaskId`。
    pub(crate) fn task_id(&self) -> TaskId {
        self.task.id()
    }

    /// 在任务发布前派生一个 `JoinObserver`，供进程/线程组协调者跟踪该线程何时彻底离开内核栈。
    pub(crate) fn observer(&self) -> JoinObserver {
        self.handle.observer()
    }

    /// 将预备任务的 `Arc<Task>` 所有权移交全局调度器（挂入活跃链表与就绪队列），并返回 `JoinHandle`。
    ///
    /// 此阶段为纯指针链接操作，不再进行任何可能失败的内存分配。
    pub(crate) fn publish(self) -> JoinHandle {
        crate::sched::publish(self.task);
        self.handle
    }
}

/// 创建并立即发布一个可在任意在线 CPU 上运行的普通内核任务。
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "具名测试 runner 在生产调度器上发表不固定 CPU 的 suite Task"
    )
)]
pub fn spawn_kernel(
    entry: impl FnOnce() -> i32 + Send + 'static,
) -> Result<JoinHandle, AllocError> {
    prepare(Placement::Any, None, entry).map(PreparedTask::publish)
}

/// 分配并初始化一个可在任意在线 CPU 上运行的预备内核任务（暂不发布到调度器）。
#[cfg(feature = "ktest")]
pub(crate) fn prepare_kernel(
    entry: impl FnOnce() -> i32 + Send + 'static,
) -> Result<PreparedTask, AllocError> {
    prepare(Placement::Any, None, entry)
}

/// 创建并立即发布一个固定在指定 `cpu` 上运行的内核任务（允许在目标从核正式上线前预先创建）。
pub fn spawn_kernel_on(
    cpu: usize,
    entry: impl FnOnce() -> i32 + Send + 'static,
) -> Result<JoinHandle, AllocError> {
    prepare_kernel_on(cpu, entry).map(PreparedTask::publish)
}

/// 分配并初始化一个固定在指定 `cpu` 上运行的预备内核任务；若后续步骤失败可直接丢弃以回收资源。
pub(crate) fn prepare_kernel_on(
    cpu: usize,
    entry: impl FnOnce() -> i32 + Send + 'static,
) -> Result<PreparedTask, AllocError> {
    assert!(cpu < crate::cpu::count(), "task CPU is out of range");
    prepare(Placement::Cpu(cpu), None, entry)
}

/// 分配并初始化一个固定在指定 `cpu` 上运行、且绑定了工作队列 `worker` 身份的预备内核任务。
pub(crate) fn prepare_worker_on(
    cpu: usize,
    worker: Arc<crate::workqueue::Worker>,
    entry: impl FnOnce() -> i32 + Send + 'static,
) -> Result<PreparedTask, AllocError> {
    assert!(cpu < crate::cpu::count(), "worker CPU is out of range");
    prepare(Placement::Cpu(cpu), Some(worker), entry)
}

/// 内部通用构造函数：分配入口闭包、`JoinState`、64 KiB 连续内核栈与 `Task` 控制块，组装为 `PreparedTask`。
///
/// 若任一步骤发生内存分配失败（`AllocError`），已分配的局部对象按常规所有权在无锁状态下自动析构。
fn prepare(
    placement: Placement,
    worker: Option<Arc<crate::workqueue::Worker>>,
    entry: impl FnOnce() -> i32 + Send + 'static,
) -> Result<PreparedTask, AllocError> {
    // Step 1: 在堆上分配入口闭包与退出状态同步块 JoinState。
    let entry: Entry = Box::try_new(entry)?;
    let join = Arc::try_new(JoinState {
        result: AtomicI32::new(0),
        completed: AtomicBool::new(false),
        waiters: WaitQueue::new(),
    })?;

    // Step 2: 从伙伴系统分配独占内核栈（16 页连续物理内存）并分配全局唯一 TaskId。
    let mut stack = alloc_pages(KERNEL_STACK_ORDER, false).ok_or(AllocError)?;
    let stack_top = stack.as_mut_ptr() as usize + stack.len();
    let id = NEXT_ID
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |id| id.checked_add(1))
        .expect("task ID space exhausted");
    let id = TaskId(id);

    // Step 3: 构造初始状态为 State::Ready、上下文指向栈顶与 task_trampoline 的 Task 实例。
    let value = Task {
        id,
        execution_ns: AtomicU64::new(0),
        reclaim: AtomicU8::new(0),
        ctx: UnsafeCell::new(TaskContext::initial(stack_top)),
        scheduling: UnsafeCell::new(Scheduling {
            state: State::Ready,
            allowed: match &placement {
                Placement::Any => crate::cpu::actual_set(),
                Placement::Cpu(owner) => crate::cpu::CpuSet::single(*owner),
            },
            affinity_pending: false,
            last_wait_id: WaitId::initial(),
            active_wait: None,
            live_previous: ptr::null(),
            live_next: ptr::null(),
            ready_previous: ptr::null(),
            ready_next: ptr::null(),
            #[cfg(feature = "ktest")]
            next_cpu: None,
        }),
        resources: UnsafeCell::new(Resources {
            stack: Some(stack),
            entry: Some(entry),
        }),
        join: Arc::clone(&join),
        worker,
        user: SpinLock::new(None),
        #[cfg(feature = "ktest")]
        ktest_boundary: AtomicPtr::new(ptr::null_mut()),
        #[cfg(feature = "ktest")]
        ktest_scope: AtomicPtr::new(ptr::null_mut()),
    };

    // Step 4: 在调用可能失败的 Arc::try_new 之前先递增 LIVE_TASKS；若分配失败，value 在栈上直接触发 Task::drop 减回计数。
    #[cfg(feature = "ktest")]
    LIVE_TASKS.fetch_add(1, Ordering::Relaxed);
    let task = Arc::try_new(value)?;
    Ok(PreparedTask {
        task,
        handle: JoinHandle { id, join },
    })
}

/// 为已准备好的用户态线程 `user` 分配底层内核 `Task` 并完成双向绑定，返回尚未发布到调度器的 `PreparedTask`。
pub(crate) fn prepare_user(
    user: Arc<UserThread>,
    placement: Placement,
    entry: impl FnOnce(Arc<UserThread>) -> i32 + Send + 'static,
) -> Result<PreparedTask, AllocError> {
    let binding = Arc::clone(&user);
    let prepared = prepare(placement, None, move || entry(user))?;
    prepared.bind_user(binding);
    Ok(prepared)
}

/// 获取当前 CPU 正在运行的普通内核任务的 `Arc<Task>` 强引用。
///
/// 只能在普通任务上下文（或禁用抢占区间）中调用；若在硬中断、软中断下半部或调度器栈中调用则会触发断言 panic。
pub fn current_task() -> Arc<Task> {
    let _irq = IrqSaveGuard::new();
    let local = crate::cpu::current();
    assert_eq!(
        local.hardirq_depth.load(Ordering::Relaxed),
        0,
        "current_task in hard IRQ"
    );
    assert_eq!(
        local.bh_depth.load(Ordering::Relaxed),
        0,
        "current_task in bottom half"
    );
    // SAFETY:
    // 1. `_irq` 守卫已关闭本地中断，本函数内不发生调度切换；
    // 2. `local.current_task` 仅由当前 CPU 在调度切换时修改，当前非空指针指向由调度器活跃任务链表（`live`）持有强引用的运行中任务。
    let task = unsafe { *local.current_task.get() };
    assert!(!task.is_null(), "current_task outside a task");
    // SAFETY:
    // `task` 来自 `sched::publish` 中 `Arc::into_raw` 转换出的有效 `Arc<Task>` 指针，且当前处于 `Running` 状态不会被释放；
    // 先调用 `increment_strong_count` 增加引用计数，再通过 `Arc::from_raw` 构造返回的新 `Arc<Task>`，从而保持调度器原有的强引用不变。
    unsafe {
        Arc::increment_strong_count(task);
        Arc::from_raw(task)
    }
}

/// 获取当前 CPU 正在运行任务所绑定的工作队列 `Worker` 引用（若当前无运行任务或非 Worker 线程则返回 `None`）。
///
/// 全程仅关中断克隆已有的 `Arc<Worker>`，不获取调度锁也不进行内存分配。
pub(crate) fn current_worker() -> Option<Arc<crate::workqueue::Worker>> {
    let _irq = IrqSaveGuard::new();
    let local = crate::cpu::current();
    // SAFETY:
    // 1. `_irq` 守卫已关闭本地中断，当前 CPU 独占访问 `local.current_task`；
    // 2. 若指针非空，该 `Task` 由调度器活跃任务链表保活，且 `task.worker` 自构造后不可变，在此处解引用并克隆 `Arc` 安全有效。
    unsafe { (*local.current_task.get()).as_ref() }
        .and_then(|task| task.worker.as_ref().map(Arc::clone))
}

/// 无锁读取当前 CPU 正在运行的任务 ID；若当前运行在调度器栈或空闲路径上则返回 `None`（可在中断处理函数中调用以诊断被打断的任务）。
pub fn current_task_id() -> Option<TaskId> {
    TaskId::from_raw(
        crate::cpu::current()
            .current_task_id
            .load(Ordering::Relaxed),
    )
}

/// 测试辅助函数：在普通任务上下文中读取当前任务独占内核栈的起始物理地址与字节长度 `(phys_addr, len)`。
#[cfg(feature = "ktest")]
pub fn test_current_stack_range() -> (usize, usize) {
    let _irq = IrqSaveGuard::new();
    let task = current_task();
    // SAFETY:
    // 1. `_irq` 守卫已关闭本地中断，`task` 为当前 CPU 正在独占运行的 `Task`；
    // 2. `task.resources` 中的 `stack` 在任务彻底退出并切回调度栈前始终存在且不会被修改，此处仅读取其起始地址和长度标量。
    let stack = unsafe { &*task.resources.get() }
        .stack
        .as_ref()
        .expect("running task has no stack");
    (stack.phys_addr(), stack.len())
}
