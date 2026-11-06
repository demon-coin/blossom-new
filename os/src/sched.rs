//! 多核任务调度器与两阶段上下文切换协议（Scheduler）。
//!
//! ## 模块职责与架构定位
//! 本模块负责管理全系统所有存活任务（`live` 链表）与就绪队列（`ready` FIFO 队列），驱动每个在线 CPU 的调度主循环（`run`），并协调任务的让出（`yield_now`）、阻塞/唤醒（`park`/`notify`）、CPU 亲和性迁移（`set_affinity`）以及任务退出资源回收。
//! - **上游调用者**：内核启动入口（各 CPU 调用 `sched::run()` 进入调度主循环）、`crate::task`（发布新任务 `publish`）、`crate::wait`（调用 `begin_wait` / `park` / `notify` / `end_wait` 实现睡眠队列）、系统调用与中断返回路径（调用 `cond_resched`、`set_affinity`、`request_user_work`）。
//! - **下游依赖**：`crate::arch::switch_context`（底层寄存器切换汇编）、`crate::cpu`（读取每 CPU 状态与发送核间中断 `kick`）、`crate::mm::space`（切换用户地址空间页表）、`crate::accounting` 与 `crate::time`（执行时间统计与时间片更新）。
//!
//! ## 核心设计与两阶段切换协议
//! 为了在多核 SMP 环境下彻底消除“任务 A 尚未保存完自身寄存器栈帧，就被另一个 CPU 从就绪队列中抢先选中并切入同一个内核栈”的竞态，本调度器采用**两阶段调度栈中转协议**：
//! 1. **阶段一（Task -> CPU 静态调度栈）**：正在运行的任务调用 `prepare_switch(intent)`，在关中断且持有 `SCHED_LOCK` 的情况下将状态从 `Running(cpu)` 改为 `SwitchingOut(cpu, intent)`（**此时任务绝不进入 `ready` 队列**），随后释放 `SCHED_LOCK` 并调用 `switch_context(task_ctx, scheduler_ctx)` 切回本 CPU 永久独占的启动/调度栈。
//! 2. **阶段二（CPU 静态调度栈 -> Finish -> 下一个 Task）**：CPU 回到 `sched::run` 循环后调用 `finish_current()`，再次获取 `SCHED_LOCK` 调用 `Scheduler::finish()`。此时原任务的 `TaskContext` 已完整保存在内存中且本 CPU 已不再使用该任务的内核栈，才安全地将其转为 `State::Ready`（挂入 `ready` 队列供任意合法 CPU 抢占）、`State::Blocked` 或 `State::Exited`（在锁外释放内核栈与地址空间引用）。
//!
//! ```text
//!   [Task 内核栈 (CPU X)]                          [CPU X 静态调度栈 (sched::run)]
//!            |                                                    |
//!   1. prepare_switch(intent)                                     |
//!      - state = SwitchingOut(X, intent)                          |
//!      - accounting::task_leave()                                 |
//!            |                                                    |
//!   2. switch_context(task_ctx, scheduler_ctx) -----------------> |
//!      (保存任务寄存器，切回 CPU X 静态调度栈)                      |
//!                                                        3. leave_current() & finish_current()
//!                                                           - 清理 current_task = null
//!                                                           - SCHED_LOCK.lock().finish(task, X):
//!                                                             * Yield -> State::Ready + push_ready()
//!                                                             * Park  -> Notified ? Ready : Blocked
//!                                                             * Exit  -> State::Exited (摘离 live 链)
//!                                                           - 锁外开中断释放旧 Mm / 退出任务的内核栈
//!                                                                 |
//!                                                        4. SCHED_LOCK.lock().pick(X)
//!                                                           - 选中下一个 Ready 任务 -> State::Running(X)
//!                                                           - enter_current(next_mm)
//!            |                                                    |
//!   5. resume_task() <---------------------------------- switch_context(scheduler_ctx, next_ctx)
//!      - accounting::task_enter()
//!      - enable_interrupts()
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - **全局调度锁 `SCHED_LOCK`**：`SpinLock<Scheduler>`（自动保存并关闭本地中断），保护 `Scheduler` 的 `live`/`ready` 双向链表以及所有 `Task::scheduling` 字段。
//! - **禁止跨切换持锁**：在调用 `switch_context` 前必须释放包括 `SCHED_LOCK` 在内的所有自旋锁，且所有切换点前均通过 `assert_switchable()` 校验本地中断开启、硬中断/软中断深度为 0、抢占禁用深度为 0。
//! - **锁顺序**：若需同时操作 `WaitQueue` 与 `SCHED_LOCK`（如 `WaitQueue::wake_all` 内部调用 `sched::notify`），顺序固定为 **先 `WaitQueue` 内部锁（或锁外唤醒） -> 后 `SCHED_LOCK`**；在持有 `SCHED_LOCK` 期间严禁反向调用任何 `WaitQueue` 方法或触发内存分配/释放。

pub mod context;

use alloc::sync::Arc;
use core::pin::pin;
use core::ptr;
use core::sync::atomic::Ordering;

use crate::sync::{IrqSaveGuard, SpinLock};
use crate::task::{ActiveWait, Resources, Scheduling, State, SwitchIntent, Task};
use crate::wait::queue::{WaitNode, WaitQueue};
use crate::wait::{WaitId, Waiter};
use crate::{accounting, cpu, time, trace};

/// 全局调度器核心数据结构，维护所有已发布任务的侵入式双向链表。
struct Scheduler {
    /// 活跃任务双向链表表头（包含所有已通过 `publish` 发布且尚未通过 `Exit` 退出的任务，每个节点持有一份 `Arc<Task>` 强引用）。
    live_head: *const Task,
    /// 就绪任务 FIFO 双向队列队头（仅包含当前处于 `State::Ready` 的任务节点）。
    ready_head: *const Task,
    /// 就绪任务 FIFO 双向队列队尾。
    ready_tail: *const Task,
}

// SAFETY:
// 1. `Scheduler` 内部的所有裸指针链表仅在持有全局自旋锁 `SCHED_LOCK` 时访问；
// 2. `live` 链表中的每个 `*const Task` 均代表由 `Arc::into_raw` 移交的唯一强引用，`Task` 本身满足 `Send + Sync`。
unsafe impl Send for Scheduler {}

/// 保护全局 `Scheduler` 状态与所有任务 `Scheduling` 字段的自旋锁。
static SCHED_LOCK: SpinLock<Scheduler> = SpinLock::new(Scheduler::new());
/// 当任务完成 CPU 亲和性迁移（离开原受限 CPU）时用于唤醒 `set_affinity` 等待者的全局等待队列。
static AFFINITY_CHANGED: WaitQueue = WaitQueue::new();

/// 统计当前全局调度器中已发布且尚未退出（仍在 `live` 链表中）的活跃任务总数。
pub(crate) fn live_task_count() -> usize {
    let scheduler = SCHED_LOCK.lock();
    let mut task = scheduler.live_head;
    let mut count = 0;
    while !task.is_null() {
        count += 1;
        // SAFETY:
        // 1. 已持有 `SCHED_LOCK`，期间不会有任务并发插入或从 `live` 链表摘除；
        // 2. `live` 链表中的每个节点均由调度器持有一份 `Arc<Task>` 强引用保活，此处仅只读遍历 `live_next` 指针。
        task = unsafe { (*(*task).scheduling.get()).live_next };
    }
    count
}

/// 判断给定任务（`data`）当前是否允许在目标 CPU（`owner`）上被选中运行。
///
/// 同时检查任务的 CPU 亲和性掩码 `allowed` 以及测试模式下的单次定向迁移目标 `next_cpu`。
fn allowed_on(data: &Scheduling, owner: usize) -> bool {
    let allowed = data.allowed.contains(owner);
    #[cfg(feature = "ktest")]
    let allowed = allowed && data.next_cpu.is_none_or(|cpu| cpu == owner);
    allowed
}

impl Scheduler {
    /// 创建一个活跃链表与就绪队列均为空的初始调度器实例。
    const fn new() -> Self {
        Self {
            live_head: ptr::null(),
            ready_head: ptr::null(),
            ready_tail: ptr::null(),
        }
    }

    /// 接收一份由 `Arc::into_raw` 转交的 `Task` 强引用，将其头插挂入 `live` 活跃链表，并尾插放入 `ready` 就绪队列。
    fn publish(&mut self, task: *const Task) {
        // SAFETY:
        // 1. 调用方 `sched::publish` 刚刚将新任务的唯一调度强引用转为 `task` 裸指针，此前该节点未挂入任何链表；
        // 2. `&mut self` 保证当前已持有 `SCHED_LOCK`，可独占修改 `live_head` 与相邻节点的 `live_*` 链接。
        unsafe {
            let data = &mut *(*task).scheduling.get();
            data.live_next = self.live_head;
            if !self.live_head.is_null() {
                (*(*self.live_head).scheduling.get()).live_previous = task;
            }
            self.live_head = task;
        }
        self.push_ready(task, true);
    }

    /// 将处于 `State::Ready` 的任务节点追加到 `ready` 队列尾部，并向允许运行该任务的在线 CPU 发送唤醒或重调度核间中断（IPI）。
    ///
    /// 若目标 CPU 正处于 `idle` 状态，发送 `KICK_SCHED_WAKE`；若目标 CPU 正在运行其他任务且 `preempt_running` 为 `true`，
    /// 则发送 `KICK_RESCHEDULE` 请求重调度（若目标就是本 CPU 且本 CPU 当前已在调度栈上准备直接 `pick`，则跳过自发 IPI）。
    fn push_ready(&mut self, task: *const Task, preempt_running: bool) {
        // SAFETY:
        // 1. `&mut self` 保证已持有 `SCHED_LOCK`，且调用方保证 `task` 仍在 `live` 链表中存活、状态为 `State::Ready` 且当前不在 `ready` 队列中；
        // 2. 在 `SCHED_LOCK` 保护下更新 `ready_head`/`ready_tail` 及前驱后继指针无并发冲突。
        unsafe {
            let data = &mut *(*task).scheduling.get();
            assert_eq!(data.state, State::Ready, "enqueue task is not ready");
            data.ready_previous = self.ready_tail;
            data.ready_next = ptr::null();
            if self.ready_tail.is_null() {
                self.ready_head = task;
            } else {
                (*(*self.ready_tail).scheduling.get()).ready_next = task;
            }
            self.ready_tail = task;

            // Step 2: 在同一把 SCHED_LOCK 保护下检查各允许 CPU 的在线与 Idle 状态，发出对应的核间通知。
            let local = cpu::current();
            for owner in data.allowed.iter() {
                let target = cpu::get(owner);
                if allowed_on(data, owner) && target.online.load(Ordering::Acquire) {
                    if target.idle.load(Ordering::Acquire) {
                        cpu::kick(owner, cpu::KICK_SCHED_WAKE);
                    } else if preempt_running
                        && (owner != local.id || local.current_task_id.load(Ordering::Relaxed) != 0)
                    {
                        cpu::kick(owner, cpu::KICK_RESCHEDULE);
                    }
                }
            }
        }
    }

    /// 将指定的就绪任务节点从 `ready` 双向队列中摘除（不释放其 `live` 链表强引用）。
    fn remove_ready(&mut self, task: *const Task) {
        // SAFETY:
        // 1. `&mut self` 保证已持有 `SCHED_LOCK`，且仅由 `pick` 传入当前仍在 `ready` 队列中的有效任务节点；
        // 2. 锁内独占修改 `ready_head`、`ready_tail` 及相邻节点的 `ready_previous`/`ready_next` 指针。
        unsafe {
            let data = &mut *(*task).scheduling.get();
            if data.ready_previous.is_null() {
                self.ready_head = data.ready_next;
            } else {
                (*(*data.ready_previous).scheduling.get()).ready_next = data.ready_next;
            }
            if data.ready_next.is_null() {
                self.ready_tail = data.ready_previous;
            } else {
                (*(*data.ready_next).scheduling.get()).ready_previous = data.ready_previous;
            }
            data.ready_previous = ptr::null();
            data.ready_next = ptr::null();
        }
    }

    /// 按 FIFO 顺序从 `ready` 队列中查找第一个允许在 `owner` CPU 上执行的任务，将其摘出就绪队列并标记为 `State::Running(owner)`。
    fn pick(&mut self, owner: usize) -> Option<*const Task> {
        let mut task = self.ready_head;
        while !task.is_null() {
            // SAFETY:
            // `ready` 队列中的所有节点均属于 `live` 活跃链表，且当前持有 `SCHED_LOCK` 独占访问各节点的 `scheduling` 元数据。
            let data = unsafe { &mut *(*task).scheduling.get() };
            if allowed_on(data, owner) {
                self.remove_ready(task);
                // SAFETY:
                // `task` 仍由 `live` 链表持有强引用且 `SCHED_LOCK` 尚未释放；将其从 `ready` 队列摘除后赋予 `owner` CPU 独占的 `Running` 状态。
                let data = unsafe { &mut *(*task).scheduling.get() };
                data.state = State::Running(owner);
                #[cfg(feature = "ktest")]
                {
                    data.next_cpu = None;
                }
                return Some(task);
            }
            task = data.ready_next;
        }
        None
    }

    /// 只读检查 `ready` 队列中是否存在允许在 `owner` CPU 上执行的就绪任务（用于在进入硬件休眠前做最终复查，不摘除节点）。
    fn has_ready(&self, owner: usize) -> bool {
        let mut task = self.ready_head;
        while !task.is_null() {
            // SAFETY:
            // `&self` 保证已持有 `SCHED_LOCK`，`ready` 队列节点均由 `live` 链表保活，此处仅只读检查 `scheduling` 元数据。
            let data = unsafe { &*(*task).scheduling.get() };
            if allowed_on(data, owner) {
                return true;
            }
            task = data.ready_next;
        }
        false
    }

    /// 在 CPU 已切回静态调度栈后，完成切出任务 `task` 的状态转换（结束 `State::SwitchingOut`）。
    ///
    /// - `SwitchIntent::Yield`：将状态设为 `State::Ready` 并重新挂入就绪队列（仅当本 CPU 已不再允许运行该任务时才向其他忙核发送抢占提示）。
    /// - `SwitchIntent::Park(id)`：若在切换期间已收到针对 `id` 的唤醒通知（`active.notified == true`），则直接转为 `State::Ready` 重新入队；否则转为 `State::Blocked`。
    /// - `SwitchIntent::Exit(result)`：将状态设为 `State::Exited`，从 `live` 活跃链表中摘除该任务，并取出其内核栈与入口闭包资源返回给调用者在锁外释放。
    fn finish(&mut self, task: *const Task, owner: usize) -> Option<(Resources, i32)> {
        // SAFETY:
        // 1. 当前 CPU 已完成 `switch_context` 回到静态调度栈，`task` 此时不再在任何内核栈上执行，且仍由 `live` 链表持有强引用保活；
        // 2. `&mut self` 保证已持有 `SCHED_LOCK`，独占访问 `task.scheduling`。
        let data = unsafe { &mut *(*task).scheduling.get() };
        match data.state {
            State::SwitchingOut(cpu, SwitchIntent::Yield) if cpu == owner => {
                data.state = State::Ready;
                let preempt_running = !allowed_on(data, owner);
                self.push_ready(task, preempt_running);
                None
            }
            State::SwitchingOut(cpu, SwitchIntent::Park(id)) if cpu == owner => {
                let active = data.active_wait.expect("park has no active wait");
                assert_eq!(active.id, id, "park wait identity changed");
                data.state = if active.notified {
                    State::Ready
                } else {
                    State::Blocked
                };
                if active.notified {
                    self.push_ready(task, true);
                }
                None
            }
            State::SwitchingOut(cpu, SwitchIntent::Exit(result)) if cpu == owner => {
                assert!(
                    data.active_wait.is_none(),
                    "exiting task has an active wait"
                );
                data.state = State::Exited;
                // SAFETY:
                // 1. 当前持有 `SCHED_LOCK`，且 `task` 已经永久退出并切回本 CPU 静态调度栈，不会再被任何 CPU 执行；
                // 2. 在锁内安全将其从 `live` 双向链表中摘除，并从 `task.resources` 中移出独占的内核栈与入口闭包，以便稍后在锁外析构。
                let resources = unsafe {
                    if data.live_previous.is_null() {
                        self.live_head = data.live_next;
                    } else {
                        (*(*data.live_previous).scheduling.get()).live_next = data.live_next;
                    }
                    if !data.live_next.is_null() {
                        (*(*data.live_next).scheduling.get()).live_previous = data.live_previous;
                    }
                    data.live_previous = ptr::null();
                    data.live_next = ptr::null();
                    let resources = &mut *(*task).resources.get();
                    Resources {
                        stack: resources.stack.take(),
                        entry: resources.entry.take(),
                    }
                };
                Some((resources, result))
            }
            _ => panic!("task switch completion has invalid state"),
        }
    }
}

/// 将已完成全部分配的 `Arc<Task>` 发布到全局调度器中（由 `live` 活跃链表接管其强引用所有权）。
pub(crate) fn publish(task: Arc<Task>) {
    let task = Arc::into_raw(task);
    SCHED_LOCK.lock().publish(task);
}

/// 校验当前上下文是否允许执行主动任务切换或阻塞睡眠。
///
/// 要求当前必须满足：
/// 1. 本地中断处于开启状态（从而保证未持有任何 `IrqSaveGuard` 或关中断自旋锁）；
/// 2. 不处于硬件中断（`hardirq_depth == 0`）或软中断下半部（`bh_depth == 0`）上下文；
/// 3. 未处于禁用抢占区间（`preempt_depth == 0`）；
/// 4. 当前 CPU 正在执行普通任务（`current_task_id().is_some()`，而非运行在调度器栈上）。
pub(crate) fn assert_switchable() {
    #[cfg(feature = "ktest")]
    if crate::ktest::panic::is_unwinding() {
        crate::ktest::fatal_event("panic", format_args!("task switch during panic unwind"));
    }
    let local = cpu::current();
    #[cfg(feature = "ktest")]
    if local.hardirq_depth.load(Ordering::Relaxed) != 0
        || local.bh_depth.load(Ordering::Relaxed) != 0
        || !crate::arch::irq_enabled()
    {
        crate::ktest::fatal_event(
            "irq-schedule",
            format_args!("task switch in IRQ or bottom half"),
        );
    }
    #[cfg(feature = "ktest")]
    if local.preempt_depth.load(Ordering::Relaxed) != 0 {
        crate::ktest::fatal_event(
            "preempt-schedule",
            format_args!("task switch with preemption disabled"),
        );
    }
    assert!(crate::arch::irq_enabled(), "task switch with IRQ disabled");
    assert_eq!(
        local.hardirq_depth.load(Ordering::Relaxed),
        0,
        "task switch in hard IRQ"
    );
    assert_eq!(
        local.bh_depth.load(Ordering::Relaxed),
        0,
        "task switch in bottom half"
    );
    assert_eq!(
        local.preempt_depth.load(Ordering::Relaxed),
        0,
        "task switch with preemption disabled"
    );
    assert!(
        crate::task::current_task_id().is_some(),
        "task switch outside a task"
    );
}

/// 当前普通任务主动让出 CPU 执行权，放入就绪队列尾部等待下一次被调度。
///
/// 任务恢复执行时可能已被迁移到另一个物理 CPU，因此调用前不得持有任何绑定当前 CPU 的借用或自旋锁守卫。
pub fn yield_now() {
    switch_out(SwitchIntent::Yield);
    resume_task();
}

/// 在内核长循环或安全点检查当前 CPU 是否存在待处理的重调度请求（`need_resched`）；若有且当前上下文允许切换，则主动让出 CPU。
///
/// 若当前处于关中断、中断处理或禁用抢占上下文，则不消费 `need_resched` 标志，留待后续恢复到可调度安全点时再处理。返回是否实际执行了让出切换。
pub fn cond_resched() -> bool {
    if !crate::arch::irq_enabled() {
        return false;
    }

    let requested = {
        let _irq = IrqSaveGuard::new();
        let local = cpu::current();
        let legal = local.hardirq_depth.load(Ordering::Relaxed) == 0
            && local.bh_depth.load(Ordering::Relaxed) == 0
            && local.preempt_depth.load(Ordering::Relaxed) == 0
            && crate::task::current_task_id().is_some();
        legal && local.need_resched.swap(false, Ordering::Relaxed)
    };
    if requested {
        yield_now();
    }
    requested
}

/// 向目标用户任务 `task` 所在的 CPU（或允许运行它的在线 CPU）发送 `KICK_RESCHEDULE` 核间中断，促使其在返回用户态前检查待处理信号或退出请求。
///
/// 若目标任务正处于 `State::Blocked` 睡眠状态，本函数不会强行将其从不可中断等待中唤醒（可中断睡眠由信号路径通过专属 `Waker` 唤醒）。
pub(crate) fn request_user_work(task: &Task) {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY:
    // 1. 已持有 `SCHED_LOCK`，可安全只读访问 `task.scheduling`；
    // 2. `&Task` 借用保证目标控制块在调用期间存活。
    let data = unsafe { &*task.scheduling.get() };
    match data.state {
        State::Running(owner) | State::SwitchingOut(owner, _) => {
            // 在持有 SCHED_LOCK 期间向当前所属 CPU 发送重调度 IPI，确保不会与迁核或入队操作产生漏通知窗口。
            cpu::kick(owner, cpu::KICK_RESCHEDULE);
        }
        State::Ready => {
            for owner in data.allowed.iter() {
                if allowed_on(data, owner) && cpu::get(owner).online.load(Ordering::Acquire) {
                    cpu::kick(owner, cpu::KICK_RESCHEDULE);
                }
            }
        }
        // 处于 Blocked 的可中断等待由信号唤醒队列精确通知；不可中断等待（如底层块设备 I/O）保持睡眠直到操作完成。
        State::Blocked | State::Exited => {}
    }
}

/// 在全局调度锁保护下读取目标任务 `task` 当前允许运行的 CPU 亲和性掩码集合。
pub(crate) fn affinity(task: &Task) -> cpu::CpuSet {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY:
    // 已持有 `SCHED_LOCK`，且调用方传入的 `&Task` 借用保证 `task` 存活，按值复制 `allowed` 掩码安全无竞态。
    unsafe { (*task.scheduling.get()).allowed }
}

/// 在创建子任务（尚未发布到调度器）时，将父任务 `parent` 的 CPU 亲和性掩码原样复制到子任务 `child` 中。
pub(crate) fn inherit_affinity(parent: &Task, child: &Task) {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY:
    // 1. 已持有 `SCHED_LOCK`，独占访问 `parent` 与 `child` 的 `scheduling` 字段；
    // 2. `parent` 与 `child` 均由调用方引用保活，且 `child` 尚未发布（处于初始 `State::Ready`）。
    unsafe {
        let source = (*parent.scheduling.get()).allowed;
        let target = &mut *child.scheduling.get();
        assert_eq!(target.state, State::Ready);
        target.allowed = source;
    }
}

/// 检查目标任务 `task` 是否已满足最新的 CPU 亲和性约束（即不再运行于被排除的 CPU 上，且未处于正在切出的 `SwitchingOut` 中间态）。
fn affinity_complete(task: &Task) -> bool {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY:
    // 已持有 `SCHED_LOCK` 保护 `state` 与 `allowed`，且调用方持有 `task` 引用使其保持存活。
    let data = unsafe { &*task.scheduling.get() };
    match data.state {
        State::Running(owner) => data.allowed.contains(owner),
        State::SwitchingOut(_, _) => false,
        State::Ready | State::Blocked | State::Exited => true,
    }
}

/// 更新目标任务 `task` 的 CPU 亲和性掩码为 `allowed`，并同步等待该任务彻底离开任何不在 `allowed` 集合内的 CPU。
pub(crate) fn set_affinity(task: &Task, allowed: cpu::CpuSet) {
    assert!(!allowed.is_empty());
    // Step 1: 在调度锁内更新 allowed 掩码；若目标任务正在不再允许的 CPU 上运行或切出，设置 affinity_pending 并发送重调度 IPI。
    {
        let _scheduler = SCHED_LOCK.lock();
        // SAFETY:
        // 已持有 `SCHED_LOCK` 独占访问 `task.scheduling`，且调用方借用保证 `task` 存活。
        let data = unsafe { &mut *task.scheduling.get() };
        data.allowed = allowed;
        match data.state {
            State::Running(owner) if !allowed.contains(owner) => {
                data.affinity_pending = true;
                cpu::kick(owner, cpu::KICK_RESCHEDULE);
            }
            State::SwitchingOut(owner, _) => {
                data.affinity_pending = true;
                cpu::kick(owner, cpu::KICK_RESCHEDULE);
            }
            State::Ready => {
                for owner in allowed.iter() {
                    if cpu::get(owner).online.load(Ordering::Acquire) {
                        cpu::kick(owner, cpu::KICK_RESCHEDULE);
                    }
                }
            }
            State::Running(_) | State::Blocked | State::Exited => {}
        }
    }

    // Step 2: 若修改的是当前任务自身且当前 CPU 已不在允许集合内，主动调用 yield_now 切出以完成迁核。
    if task.id() == crate::task::current_task().id() {
        if !affinity_complete(task) {
            yield_now();
        }
        return;
    }

    // Step 3: 若修改的是其他任务，在全局 AFFINITY_CHANGED 等待队列上睡眠，直到目标 CPU 在 finish_current 中完成切换并唤醒。
    loop {
        if affinity_complete(task) {
            return;
        }
        let waiter = Waiter::new();
        let mut node = pin!(WaitNode::new(waiter.waker()));
        // SAFETY:
        // 1. `AFFINITY_CHANGED` 为 `'static` 全局等待队列，生命周期覆盖整个内核运行期；
        // 2. `node` 通过 `pin!` 固定在当前栈帧上，`_registration` 在每次循环迭代结束时先于 `node` 析构并将节点从队列摘除。
        let _registration = unsafe { AFFINITY_CHANGED.register(node.as_mut()) };
        if affinity_complete(task) {
            return;
        }
        waiter.park();
    }
}

/// 准备将当前运行任务切回本 CPU 的静态调度栈：关闭本地中断，并在 `SCHED_LOCK` 下提交切换意图 `intent`。
///
/// - 若 `intent` 为 `SwitchIntent::Park(id)` 且本轮等待已提前收到唤醒通知（`active.notified == true`），
///   则取消本次切换、重新开启本地中断并返回 `None`，当前任务继续留在 CPU 上运行；
/// - 若成功提交为 `State::SwitchingOut(local.id, intent)`，则结算当前任务执行时间并保持关中断状态，
///   返回 `(当前任务上下文指针, 本 CPU 调度器上下文指针)` 供随后的 `switch_context` 汇编使用。
fn prepare_switch(
    intent: SwitchIntent,
) -> Option<(*mut context::TaskContext, *const context::TaskContext)> {
    assert_switchable();
    crate::arch::irq_save();
    let contexts = {
        let local = cpu::current();
        // SAFETY:
        // 本地中断已由上方 `irq_save()` 关闭，`local.current_task` 仅由当前 CPU 访问，且运行中任务由 `live` 链表保活。
        let task = unsafe { *local.current_task.get() };
        assert!(!task.is_null(), "switching task is missing");
        let committed = {
            let _scheduler = SCHED_LOCK.lock();
            // SAFETY:
            // 已持有 `SCHED_LOCK`，可独占检查并修改当前任务的 `scheduling` 状态。
            let data = unsafe { &mut *(*task).scheduling.get() };
            assert_eq!(
                data.state,
                State::Running(local.id),
                "switching task has no execution ownership"
            );
            let notified = if let SwitchIntent::Park(id) = intent {
                let active = data.active_wait.expect("park has no active wait");
                assert_eq!(active.id, id, "park wait identity changed");
                active.notified
            } else {
                false
            };
            if !notified {
                data.state = State::SwitchingOut(local.id, intent);
            }
            !notified
        };

        if committed {
            accounting::task_leave();
            time::task_switch_out();
            // SAFETY:
            // `task` 刚转为 `SwitchingOut` 状态，在回到调度栈执行 `finish` 之前不会被其他 CPU 选中或销毁；
            // 此处仅取出 `task.ctx` 和本 CPU `scheduler_ctx` 的裸指针。
            Some((
                unsafe { (*task).ctx.get() },
                local.scheduler_ctx.get().cast_const(),
            ))
        } else {
            None
        }
    };
    if contexts.is_none() {
        crate::arch::enable_interrupts();
    }
    contexts
}

/// 提交非阻塞切换意图（`Yield` 或 `Exit`）并执行汇编上下文切换，将 CPU 控制权交回本核静态调度栈。
fn switch_out(intent: SwitchIntent) {
    let (task_ctx, scheduler_ctx) =
        prepare_switch(intent).expect("non-park switch was not committed");
    // SAFETY:
    // 1. `prepare_switch` 已关闭本地中断并释放了 `SCHED_LOCK`，调用方也不持有任何跨切换的自旋锁或 CPU 局部借用；
    // 2. 当前任务处于 `SwitchingOut` 独占阶段，`task_ctx` 与本核 `scheduler_ctx` 均有效且无并发访问。
    unsafe { crate::arch::switch_context(task_ctx, scheduler_ctx) };
}

/// 阻塞当前任务以等待编号为 `id` 的等待轮次被唤醒。
///
/// 若在进入 `prepare_switch` 前该轮次已被 `notify` 标记为已通知，则不会发生上下文切换；
/// 若当前任务是工作队列的 `Worker` 线程，在进入睡眠前后会同步通知工作队列池以按需唤醒替补工作线程。
pub(crate) fn park(id: WaitId) {
    assert_switchable();
    let worker = crate::task::current_worker();
    // Step 1: 若当前任务是工作队列 Worker，在关闭中断和获取 SCHED_LOCK 之前先通知线程池本线程即将睡眠。
    let was_running = worker.as_ref().is_some_and(|worker| worker.sleeping());
    // Step 2: 尝试提交 Park(id) 切换意图；若在此之前已收到唤醒通知，prepare_switch 返回 None。
    let contexts = prepare_switch(SwitchIntent::Park(id));
    #[cfg(feature = "ktest")]
    if contexts.is_none() && was_running {
        worker
            .as_ref()
            .expect("sleeping worker is absent")
            .early_wake();
    }
    // Step 3: 若未收到通知，切换到本 CPU 静态调度栈；待被唤醒并重新被某个 CPU 选中后，调用 resume_task 恢复执行。
    if let Some((task_ctx, scheduler_ctx)) = contexts {
        // SAFETY:
        // `prepare_switch` 已将任务标记为 `SwitchingOut`、释放了 `SCHED_LOCK` 并保持关中断状态，`task_ctx` 与 `scheduler_ctx` 均由当前 CPU 独占。
        unsafe { crate::arch::switch_context(task_ctx, scheduler_ctx) };
        resume_task();
        let task = crate::task::current_task_id().expect("resumed wait has no current task");
        trace::record(
            trace::Kind::WaitResume,
            task.as_usize() as u64,
            id.as_usize() as u64,
        );
    }
    // Step 4: 无论实际发生了睡眠还是因提前通知而跳过切换，都将 Worker 状态恢复为运行中。
    if let Some(worker) = worker {
        worker.running(was_running);
    }
}

/// 为当前正在运行的任务 `task` 开启一轮新的阻塞等待，分配并返回唯一的递增 `WaitId`。
///
/// 调用时当前任务不得已有未结束的活跃等待（`active_wait` 必须为 `None`）。
pub(crate) fn begin_wait(task: &Task) -> WaitId {
    let _scheduler = SCHED_LOCK.lock();
    assert_eq!(
        crate::task::current_task_id(),
        Some(task.id()),
        "wait belongs to another task"
    );
    // SAFETY:
    // 1. 已持有 `SCHED_LOCK`，可独占修改 `task.scheduling`；
    // 2. 调用方持有 `task` 引用，且上方断言已确认 `task` 即为当前 CPU 正在运行的任务。
    let data = unsafe { &mut *task.scheduling.get() };
    assert_eq!(data.state, State::Running(cpu::current().id));
    assert!(
        data.active_wait.is_none(),
        "task already has an active wait"
    );
    let id = data.last_wait_id.next();
    data.last_wait_id = id;
    data.active_wait = Some(ActiveWait {
        id,
        notified: false,
    });
    id
}

/// 在 `Waiter` 析构时结束当前任务编号为 `id` 的等待轮次，清除 `active_wait` 状态。
pub(crate) fn end_wait(task: &Task, id: WaitId) {
    let _scheduler = SCHED_LOCK.lock();
    assert_eq!(crate::task::current_task_id(), Some(task.id()));
    // SAFETY:
    // 1. 已持有 `SCHED_LOCK`，可独占修改 `task.scheduling`；
    // 2. `Waiter` 内部持有 `Arc<Task>` 且 `!Send`，保证析构必发生在同一运行任务上下文中。
    let data = unsafe { &mut *task.scheduling.get() };
    assert_eq!(data.state, State::Running(cpu::current().id));
    assert_eq!(data.active_wait.expect("ending absent wait").id, id);
    data.active_wait = None;
}

/// 尝试向目标任务 `task` 的指定等待轮次 `id` 发送唤醒通知，返回本次调用是否成功触发了首次有效唤醒。
///
/// - 若目标任务当前不处于等待状态、等待轮次已变更为其他 `WaitId`、或本轮此前已被唤醒过，则直接返回 `false`；
/// - 若目标任务尚处于 `Running` 或 `SwitchingOut`（尚未在 `finish` 中变为 `Blocked`），仅将 `active.notified` 置为 `true`（`prepare_switch` 或 `finish` 会据此取消阻塞直接恢复就绪）；
/// - 若目标任务已处于 `State::Blocked`，则将其状态改为 `State::Ready` 并挂入就绪队列尾部。
pub(crate) fn notify(task: &Task, id: WaitId) -> bool {
    let mut scheduler = SCHED_LOCK.lock();
    // SAFETY:
    // 1. 已持有 `SCHED_LOCK`，独占访问 `task.scheduling` 与调度器就绪队列；
    // 2. 调用方（`Waker::notify`）在加锁前已将 `Weak<Task>` 升级为 `Arc<Task>`，保证 `task` 在整个调用期间有效且若需入队仍在 `live` 链表中。
    let data = unsafe { &mut *task.scheduling.get() };
    let Some(active) = data.active_wait.as_mut() else {
        return false;
    };
    if active.id != id || active.notified {
        return false;
    }
    assert!(
        matches!(
            data.state,
            State::Ready | State::Running(_) | State::SwitchingOut(_, _) | State::Blocked
        ),
        "unnotified wait has invalid state"
    );
    active.notified = true;
    if data.state == State::Blocked {
        data.state = State::Ready;
        scheduler.push_ready(task, true);
    }
    true
}

/// 测试辅助函数：模拟任务在提交 `SwitchingOut(Park(id))` 并释放 `SCHED_LOCK` 后、执行 `switch_context` 汇编保存寄存器之前收到并发 `waker.notify()` 的交错窗口。
#[cfg(feature = "ktest")]
pub(crate) fn test_park_notify(task: &Task, id: WaitId, waker: &crate::wait::Waker) {
    assert_switchable();
    let worker = crate::task::current_worker();
    let was_running = worker.as_ref().is_some_and(|worker| worker.sleeping());
    let contexts = prepare_switch(SwitchIntent::Park(id));
    // 普通任务必须成功进入 SwitchingOut 测试窗口；仅工作队列 Worker 允许在 sleeping() 钩子期间提前被唤醒。
    assert!(
        worker.is_some() || contexts.is_some(),
        "test park was already notified"
    );
    if let Some((task_ctx, scheduler_ctx)) = contexts {
        assert!(waker.notify(), "switching-out notification was rejected");
        assert!(matches!(
            test_state(task),
            State::SwitchingOut(_, SwitchIntent::Park(active)) if active == id
        ));
        assert_eq!(test_ready_occurrences(task), 0);
        // SAFETY:
        // `prepare_switch` 已将任务置为 `SwitchingOut`、释放了 `SCHED_LOCK` 且关闭了本地中断；
        // `waker.notify()` 仅置位 `notified` 而未将任务放入 `ready` 队列，因此 `task_ctx` 与 `scheduler_ctx` 仍由本 CPU 独占。
        unsafe { crate::arch::switch_context(task_ctx, scheduler_ctx) };
        resume_task();
    } else if was_running {
        worker
            .as_ref()
            .expect("sleeping worker is absent")
            .early_wake();
    }
    if let Some(worker) = worker {
        worker.running(was_running);
    }
}

/// 测试辅助函数：在全局调度锁保护下读取目标任务当前的 `State` 副本。
#[cfg(feature = "ktest")]
pub(crate) fn test_state(task: &Task) -> State {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY:
    // 已持有 `SCHED_LOCK`，且调用方持有 `&Task` 借用，按值读取 `state` 安全有效。
    unsafe { (*task.scheduling.get()).state }
}

/// 测试辅助函数：验证在持有 `SCHED_LOCK` 自旋锁的临界区内调用 `klog!` 记录日志不会引发死锁或递归调度。
#[cfg(feature = "ktest")]
pub(crate) fn test_klog() -> u64 {
    let _scheduler = SCHED_LOCK.lock();
    crate::klog!(crate::klog::Level::Debug, "X25 scheduler locked")
}

/// 测试辅助函数：遍历全局 `ready` 队列，统计目标任务 `task` 在就绪队列中出现的次数（正常情况下只可能为 0 或 1）。
#[cfg(feature = "ktest")]
pub(crate) fn test_ready_occurrences(task: &Task) -> usize {
    let scheduler = SCHED_LOCK.lock();
    let mut member = scheduler.ready_head;
    let mut count = 0;
    while !member.is_null() {
        if ptr::eq(member, task) {
            count += 1;
        }
        // SAFETY:
        // 已持有 `SCHED_LOCK`，`ready` 队列中的所有节点均由 `live` 链表保活且遍历期间链表结构不变。
        member = unsafe { (*(*member).scheduling.get()).ready_next };
    }
    count
}

/// 任务被调度器切入（首次进入或从上次 `switch_context` 恢复）后的统一收尾入口。
///
/// 在当前目的 CPU 上重置时间片、将 CPU 统计模式切入 `Mode::Task`，最后开启当前 CPU 的本地中断。
fn resume_task() {
    assert!(!crate::arch::irq_enabled(), "task resume has IRQ enabled");
    assert!(
        crate::task::current_task_id().is_some(),
        "task resume has no current task"
    );
    time::task_switch_in();
    accounting::task_enter();
    crate::arch::enable_interrupts();
}

/// 新创建任务首次被 `switch_context` 切入时的汇编返回跳板函数（永不返回）。
///
/// 负责完成首次切入记账与开中断（`resume_task`），从当前任务资源槽中取出一次性入口闭包 `entry` 并执行；
/// 待闭包返回退出码后，调用 `switch_out(SwitchIntent::Exit(result))` 永久切回本 CPU 静态调度栈。
pub(crate) extern "C" fn task_trampoline() -> ! {
    resume_task();
    let entry = {
        let _irq = IrqSaveGuard::new();
        let local = cpu::current();
        // SAFETY:
        // 1. `_irq` 守卫已关闭本地中断，当前任务处于本 CPU 独占的 `Running` 状态且由 `live` 链表保活；
        // 2. `resources.entry` 仅在新任务首次进入 `task_trampoline` 时在此处被 `take()` 取出一次，无其他并发访问。
        unsafe {
            (*(*local.current_task.get()))
                .resources
                .get()
                .as_mut()
                .unwrap()
                .entry
                .take()
        }
        .expect("task entry has already been consumed")
    };
    let result = entry();
    switch_out(SwitchIntent::Exit(result));
    panic!("exited task resumed");
}

/// 在 CPU 静态调度栈上完成对刚刚切出任务的收尾处理（两阶段切换协议的第二阶段）。
///
/// 1. 清除本 CPU 的 `current_task` 与 `current_task_id`；
/// 2. 获取 `SCHED_LOCK` 调用 `Scheduler::finish` 推进任务状态（转为 `Ready`、`Blocked` 或 `Exited`）；
/// 3. 若有 `set_affinity` 等待者正在等待该任务离开本 CPU，唤醒 `AFFINITY_CHANGED`；
/// 4. 若任务已退出（或刚卸下了用户地址空间 `previous_mm`），在释放 `SCHED_LOCK` 并临时开启本地中断后，
///    安全释放用户地址空间、内核栈与 `Task` 控制块，并调用 `join.complete(result)` 唤醒等待 `join` 的观察者。
fn finish_current(previous_mm: Option<Arc<crate::mm::space::Mm>>) {
    // Step 1: 在关中断的静态调度栈上清空当前 CPU 的运行任务指针与任务 ID。
    let (task, owner) = {
        let local = cpu::current();
        // SAFETY:
        // 当前已切回本 CPU 静态调度栈且本地中断关闭，`local.current_task` 仅由所属 CPU 访问。
        let task = unsafe { local.current_task.get().replace(ptr::null()) };
        local.current_task_id.store(0, Ordering::Relaxed);
        (task, local.id)
    };
    assert!(!task.is_null(), "scheduler returned without a task");

    // Step 2: 在 SCHED_LOCK 保护下执行 finish 状态转换，并取走 affinity_pending 标志。
    let (retired, affinity_changed) = {
        let mut scheduler = SCHED_LOCK.lock();
        // SAFETY:
        // 已持有 `SCHED_LOCK`，且 `task` 在 `scheduler.finish` 执行前仍由 `live` 链表持有强引用保活。
        let changed = unsafe {
            let data = &mut *(*task).scheduling.get();
            core::mem::replace(&mut data.affinity_pending, false)
        };
        (scheduler.finish(task, owner), changed)
    };

    // Step 3: 若该任务此前有待处理的亲和性迁移等待，在调度锁外唤醒 AFFINITY_CHANGED 等待队列。
    if affinity_changed {
        AFFINITY_CHANGED.wake_all();
    }

    // Step 4: 在调度锁外开中断释放旧地址空间、退出任务的内核栈与控制块，并发布退出码。
    if let Some((resources, result)) = retired {
        // SAFETY:
        // `scheduler.finish` 刚刚将该退出任务从 `live` 链表中摘除，此处的 `Arc::from_raw(task)` 精确收回当初在 `sched::publish` 中通过 `Arc::into_raw` 移交的那一份强引用所有权。
        let task = unsafe { Arc::from_raw(task) };
        let join = Arc::clone(&task.join);
        let user = task.user.lock().take();
        crate::arch::enable_interrupts();
        drop(previous_mm);
        if let Some(user) = user {
            user.release_mm();
            drop(user);
        }
        drop(resources);
        drop(task);

        join.complete(result);
        drop(join);
        crate::arch::irq_save();
    } else if previous_mm.is_some() {
        crate::arch::enable_interrupts();
        drop(previous_mm);
        crate::arch::irq_save();
    }
}

/// 每个在线 CPU 的调度器主循环入口（运行在各 CPU 的永久启动/调度栈上，永不返回）。
///
/// 循环执行“持锁选任务（`pick`） -> 切换地址空间与寄存器上下文（`switch_context`） -> 切回后收尾（`finish_current`）”；
/// 当就绪队列中没有本 CPU 可运行的任务时，进入 `idle_once` 低功耗等待。
pub fn run() -> ! {
    assert!(
        cpu::current().online.load(Ordering::Acquire),
        "scheduler CPU is offline"
    );
    crate::arch::irq_save();
    accounting::start_scheduler();
    time::activate_current_cpu();

    loop {
        // Step 1: 在 SCHED_LOCK 下尝试选取下一个可执行任务，并同步更新本 CPU 的 need_resched 与 idle 状态。
        let next = {
            let local = cpu::current();
            let mut scheduler = SCHED_LOCK.lock();
            let next = scheduler.pick(local.id);
            // 已经开始新一轮任务分派（或已确认无就绪任务），清除上一任务遗留的重调度请求标志。
            local.need_resched.store(false, Ordering::Relaxed);
            local.idle.store(next.is_none(), Ordering::Release);
            next
        };
        if let Some(task) = next {
            // Step 2: 登记本 CPU 当前运行任务身份，切换用户页表并执行汇编上下文切换进入目标任务。
            let local = cpu::current();
            // SAFETY:
            // `scheduler.pick` 已将 `task` 设为本 CPU 独占的 `Running` 状态，且本地中断处于关闭状态，仅本 CPU 访问 `local.current_task`。
            unsafe { *local.current_task.get() = task };
            // SAFETY:
            // `task` 由 `live` 链表保活且处于本 CPU `Running` 状态，读取不可变的 `id()` 安全无竞态。
            let id = unsafe { (*task).id().as_usize() };
            local.current_task_id.store(id, Ordering::Relaxed);
            trace::record(trace::Kind::TaskDispatch, id as u64, 0);
            // SAFETY:
            // `task` 处于本 CPU 独占 `Running` 状态且 `SCHED_LOCK` 已释放，调用 `user_thread()` 内部通过短自旋锁获取 `UserThread` 引用。
            let mm = unsafe { (*task).user_thread() }.and_then(|user| user.mm());
            crate::mm::space::enter_current(mm);
            // SAFETY:
            // 本地中断关闭且未持有任何自旋锁；`local.scheduler_ctx` 归本 CPU 独占，`(*task).ctx` 归刚被 `pick` 选中的唯一 `Running` 任务独占。
            unsafe {
                crate::arch::switch_context(
                    local.scheduler_ctx.get(),
                    (*task).ctx.get().cast_const(),
                )
            };
            // Step 3: 任务主动让出、阻塞或退出后切回此处，离开用户地址空间并执行 finish_current 收尾。
            let previous_mm = crate::mm::space::leave_current();
            finish_current(previous_mm);
        } else {
            // Step 4: 当前无本核可运行任务，执行一轮空闲等待协议。
            idle_once();
        }
    }
}

/// 在 `run` 主循环未选中就绪任务且已将本 CPU 的 `idle` 标志置为 `true` 后，执行单次空闲等待协议。
///
/// 重新获取 `SCHED_LOCK` 复查就绪队列、核间中断（`kick_pending`）与重调度标志（`need_resched`）：
/// - 若确无任何待处理工作，则在关中断状态下调用架构相关的 `local.idle_once()` 进入硬件低功耗等待；
/// - 若复查发现已有新工作或待处理中断，则短暂开启再关闭中断以响应挂起的硬件中断，随后清除 `idle` 标志返回主循环重新选核。
fn idle_once() {
    #[cfg(feature = "ktest")]
    cpu::test_idle_boundary();
    let local = cpu::current();
    let should_wait = {
        let scheduler = SCHED_LOCK.lock();
        !scheduler.has_ready(local.id)
            && !local.kick_pending()
            && !local.need_resched.load(Ordering::Relaxed)
    };
    if should_wait {
        #[cfg(feature = "ktest")]
        accounting::test_idle_checked();
        accounting::idle_enter();
        // SAFETY:
        // 1. 本 CPU 已在 `SCHED_LOCK` 下发布 `idle = true` 并完成了最终无工作复查；
        // 2. 当前运行在 CPU 静态调度栈上，本地中断关闭且未持有任何自旋锁，满足 `CpuLocal::idle_once` 的调用前提。
        unsafe { local.idle_once() };
        accounting::idle_leave();
    } else {
        // 存在待处理的唤醒或重调度提示，跳过硬件睡眠，短暂开关中断以消费挂起的硬件 IPI/定时器中断。
        crate::arch::enable_interrupts();
        crate::arch::irq_save();
    }

    let _scheduler = SCHED_LOCK.lock();
    local.idle.store(false, Ordering::Release);
}

/// 测试辅助函数：将当前正在运行的任务的下一次调度目标强制指定为 `target` CPU（必须在 `allowed` 集合内且在线），并立即调用 `yield_now` 触发跨核迁移。
#[cfg(feature = "ktest")]
pub fn test_migrate_to(target: usize) {
    assert_switchable();
    assert!(target < cpu::count(), "migration CPU is out of range");
    assert!(
        cpu::get(target).online.load(Ordering::Acquire),
        "migration CPU is offline"
    );
    {
        let _irq = IrqSaveGuard::new();
        let local = cpu::current();
        let _scheduler = SCHED_LOCK.lock();
        // SAFETY:
        // `_irq` 已关闭本地中断且已持有 `SCHED_LOCK`，`local.current_task` 指向当前 CPU 独占运行的任务，可安全修改其 `next_cpu`。
        let data = unsafe { &mut *(*(*local.current_task.get())).scheduling.get() };
        assert!(
            data.allowed.contains(target),
            "migration CPU is not allowed"
        );
        assert!(
            data.next_cpu.is_none(),
            "task already has a migration target"
        );
        data.next_cpu = Some(target);
    }
    yield_now();
}

/// 测试辅助函数：在持有调度锁的情况下，将一个正处于 `State::Blocked` 阻塞状态的任务 `task` 的下一次唤醒调度目标指定为 `target` CPU。
#[cfg(feature = "ktest")]
pub(crate) fn test_request_migration(task: &Task, target: usize) {
    assert!(target < cpu::count(), "migration CPU is out of range");
    assert!(
        cpu::get(target).online.load(Ordering::Acquire),
        "migration CPU is offline"
    );
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY:
    // 已持有 `SCHED_LOCK` 独占访问 `task.scheduling`，且调用方持有 `&Task` 引用保活目标任务。
    let data = unsafe { &mut *task.scheduling.get() };
    assert_eq!(
        data.state,
        State::Blocked,
        "migration request requires Blocked task"
    );
    assert!(
        data.allowed.contains(target),
        "migration CPU is not allowed"
    );
    assert!(
        data.next_cpu.is_none(),
        "task already has a migration target"
    );
    data.next_cpu = Some(target);
}
