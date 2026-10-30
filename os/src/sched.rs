//! Task 先保存到本 CPU 静态调度栈，完成交接以后才重新进入独立 ready 队列。

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

struct Scheduler {
    live_head: *const Task,
    ready_head: *const Task,
    ready_tail: *const Task,
}

// SAFETY: 全部链接与状态只在 SCHED_LOCK 中访问；live 中每个指针各代表一个 Arc 强拥有。
unsafe impl Send for Scheduler {}

static SCHED_LOCK: SpinLock<Scheduler> = SpinLock::new(Scheduler::new());
static AFFINITY_CHANGED: WaitQueue = WaitQueue::new();

/// 在原活登记锁内统计已发表且尚未离栈退出的执行者，不计私有准备对象或旧观察引用。
pub(crate) fn live_task_count() -> usize {
    let scheduler = SCHED_LOCK.lock();
    let mut task = scheduler.live_head;
    let mut count = 0;
    while !task.is_null() {
        count += 1;
        // SAFETY: 活链的每个节点由调度器持有强引用；同一调度锁排除发表、摘离和归还，
        // 此处只读取 live_next，不访问正在执行的寄存器现场或资源。
        task = unsafe { (*(*task).scheduling.get()).live_next };
    }
    count
}

/// 归属与一次测试迁核目标使用同一准入判断，队列选择和 idle 重检不能分歧。
fn allowed_on(data: &Scheduling, owner: usize) -> bool {
    let allowed = data.allowed.contains(owner);
    #[cfg(feature = "ktest")]
    let allowed = allowed && data.next_cpu.is_none_or(|cpu| cpu == owner);
    allowed
}

impl Scheduler {
    /// 两组 intrusive 链表无需引导时或锁内分配。
    const fn new() -> Self {
        Self {
            live_head: ptr::null(),
            ready_head: ptr::null(),
            ready_tail: ptr::null(),
        }
    }

    /// 接收一次原始强拥有并登记活任务，加入 ready 尾部后允许向忙核请求调度。
    fn publish(&mut self, task: *const Task) {
        // SAFETY: task 是新 Arc::into_raw 的唯一活登记拥有，尚未发表；锁独占全部链接。
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

    /// 锁内发表独立 ready 节点并唤醒合法 idle 核；preempt_running 允许通知忙核，
    /// 但本核已离栈且即将直接 pick 时仍不自发重调度 IPI。
    fn push_ready(&mut self, task: *const Task, preempt_running: bool) {
        // SAFETY: 私有调用点保证 task 活着且为 Ready、不在 ready；锁独占独立 ready 链接。
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
            // idle 与 ready 在同一调度锁下发表，纯唤醒即使迟到也不能抢占下一 Running Task。
            // 本核离栈后会直接 pick，额外的自重调度会在下一 Task 开 IRQ 时重新制造提示。
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

    /// 在锁内摘离现有 ready 成员，节点不承担强拥有的释放。
    fn remove_ready(&mut self, task: *const Task) {
        // SAFETY: pick 只传仍在本 ready 链的活节点，锁独占头尾和相邻节点。
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

    /// 按 FIFO 寻找本在线 CPU 可执行的节点，原子取得唯一 Running 执行权。
    fn pick(&mut self, owner: usize) -> Option<*const Task> {
        let mut task = self.ready_head;
        while !task.is_null() {
            // SAFETY: ready 只包含活登记节点，锁保护状态和链接，不访问 ctx。
            let data = unsafe { &mut *(*task).scheduling.get() };
            if allowed_on(data, owner) {
                self.remove_ready(task);
                // SAFETY: task 仍由 live 强拥有且锁尚未释放；摘离 ready 后只给本 CPU 执行权。
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

    /// 在同一发表锁下只读本 CPU 可执行的 ready，最终 idle 检查不取得任务执行权。
    fn has_ready(&self, owner: usize) -> bool {
        let mut task = self.ready_head;
        while !task.is_null() {
            // SAFETY: 同一调度锁保护活登记与 ready 链，本函数只读调度数据。
            let data = unsafe { &*(*task).scheduling.get() };
            if allowed_on(data, owner) {
                return true;
            }
            task = data.ready_next;
        }
        false
    }

    /// ctx 保存后结束 SwitchingOut；普通 Yield 只通知 idle，原核被许可排除时才请求忙核调度，
    /// Park 的真实通知保留抢占提示，退出资源仅移出而不在锁内析构。
    fn finish(&mut self, task: *const Task, owner: usize) -> Option<(Resources, i32)> {
        // SAFETY: 本 CPU 刚回调度栈，活登记保留 task，旧 CPU 已结束 ctx 保存；锁独占状态。
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
                // SAFETY: 活节点尚未释放且退出任务不会再执行；同锁覆盖活链接和资源摘离。
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

/// 发表前已完成全部 fallible 分配；活链接管恰好一个 Arc 强拥有。
pub(crate) fn publish(task: Arc<Task>) {
    let task = Arc::into_raw(task);
    SCHED_LOCK.lock().publish(task);
}

/// 集中检查显式安全点，普通 IRQ-save 锁通过 IRQ 关闭自然被拒绝。
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

/// 普通任务显式让出执行权，恢复可能发生在另一 CPU，普通 guard 不跨交接。
pub fn yield_now() {
    switch_out(SwitchIntent::Yield);
    resume_task();
}

/// 仅普通可服务 IRQ 的安全点消耗提示；非法上下文保留提示给后续安全点。
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

/// 请求用户执行者在所属 CPU 的合法点检查工作，不改现场或唤醒不可中断等待。
pub(crate) fn request_user_work(task: &Task) {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY: 调度锁保护状态与归属；调用者借用保持 Task 有效，只读数据，不访问 ctx 或运行资源。
    let data = unsafe { &*task.scheduling.get() };
    match data.state {
        State::Running(owner) | State::SwitchingOut(owner, _) => {
            // 在归属锁内发表提示，迁核前的旧 owner 与后续 ready 发表不会留下无人接收的窗口。
            cpu::kick(owner, cpu::KICK_RESCHEDULE);
        }
        State::Ready => {
            for owner in data.allowed.iter() {
                if allowed_on(data, owner) && cpu::get(owner).online.load(Ordering::Acquire) {
                    cpu::kick(owner, cpu::KICK_RESCHEDULE);
                }
            }
        }
        // Blocked 的可中断轮次只由其精确 WaitId 通知；内核 DMA 等待保留原资源责任。
        State::Blocked | State::Exited => {}
    }
}

/// 在唯一调度锁下读取许可；用户观察和继承不读取另一份 placement 快照。
pub(crate) fn affinity(task: &Task) -> cpu::CpuSet {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY: 调度锁保护许可，借用保持 Task 控制块存活。
    unsafe { (*task.scheduling.get()).allowed }
}

/// 未发表子执行者与父许可在同一锁下快照，子任务首次运行即遵守继承集合。
pub(crate) fn inherit_affinity(parent: &Task, child: &Task) {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY: 两个 Task 由调用者保活，调度锁独占许可；child 尚未发表。
    unsafe {
        let source = (*parent.scheduling.get()).allowed;
        let target = &mut *child.scheduling.get();
        assert_eq!(target.state, State::Ready);
        target.allowed = source;
    }
}

/// 完成条件在与 pick/finish 相同的锁域判定，不能用 IPI 已发送替代离栈。
fn affinity_complete(task: &Task) -> bool {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY: 调度锁保护状态及许可，Task 由等待者的稳定拥有保活。
    let data = unsafe { &*task.scheduling.get() };
    match data.state {
        State::Running(owner) => data.allowed.contains(owner),
        State::SwitchingOut(_, _) => false,
        State::Ready | State::Blocked | State::Exited => true,
    }
}

/// 提交真实调度许可，等待被排除的运行者保存现场并由原 finish 交出运行权。
pub(crate) fn set_affinity(task: &Task, allowed: cpu::CpuSet) {
    assert!(!allowed.is_empty());
    {
        let _scheduler = SCHED_LOCK.lock();
        // SAFETY: 调度锁独占状态与许可，调用者保活原目标，不访问远端现场。
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
    if task.id() == crate::task::current_task().id() {
        if !affinity_complete(task) {
            yield_now();
        }
        return;
    }
    loop {
        if affinity_complete(task) {
            return;
        }
        let waiter = Waiter::new();
        let mut node = pin!(WaitNode::new(waiter.waker()));
        // SAFETY: 全局等待源永久存在；本轮固定节点先撤登记后离栈，登记后复查关闭通知窗口。
        let _registration = unsafe { AFFINITY_CHANGED.register(node.as_mut()) };
        if affinity_complete(task) {
            return;
        }
        waiter.park();
    }
}

/// 在专用 IRQ-off 段提交意图，通知已到达的 Park 不交出执行权。
fn prepare_switch(
    intent: SwitchIntent,
) -> Option<(*mut context::TaskContext, *const context::TaskContext)> {
    assert_switchable();
    crate::arch::irq_save();
    let contexts = {
        let local = cpu::current();
        // SAFETY: IRQ 关闭，唯一所属 CPU 读取 current；Running 对象由活登记保持有效。
        let task = unsafe { *local.current_task.get() };
        assert!(!task.is_null(), "switching task is missing");
        let committed = {
            let _scheduler = SCHED_LOCK.lock();
            // SAFETY: 调度锁独占 state；当前 Task 仍有唯一 Running 执行权，不访问其他 CPU 的 ctx。
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
            // SAFETY: 活登记保护 task，SwitchingOut 不可分派；只取独立 ctx 裸指针，不构造 &mut Task。
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

/// 提交成功后保存 ctx，普通锁和本 CPU 引用都不跨越汇编切换。
fn switch_out(intent: SwitchIntent) {
    let (task_ctx, scheduler_ctx) =
        prepare_switch(intent).expect("non-park switch was not committed");
    // SAFETY: IRQ 关闭、普通锁已释放；SwitchingOut 执行者与本 CPU 独占两份有效 ctx，
    // scheduler ctx 来自同核 run 的保存现场；调用者恢复后重新获取目的 CPU。
    unsafe { crate::arch::switch_context(task_ctx, scheduler_ctx) };
}

/// 同一等待轮次只有真正缺少通知时才阻塞，目的 CPU 恢复后记录本轮等待身份。
pub(crate) fn park(id: WaitId) {
    assert_switchable();
    let worker = crate::task::current_worker();
    // 池反馈和其中的通知先于最终 IRQ-off/SCHED 段，任何普通 guard 已在调用返回时析构。
    let was_running = worker.as_ref().is_some_and(|worker| worker.sleeping());
    let contexts = prepare_switch(SwitchIntent::Park(id));
    #[cfg(feature = "ktest")]
    if contexts.is_none() && was_running {
        worker
            .as_ref()
            .expect("sleeping worker is absent")
            .early_wake();
    }
    if let Some((task_ctx, scheduler_ctx)) = contexts {
        // SAFETY: prepare_switch 已提交 SwitchingOut 且放锁，IRQ 关闭，两份 ctx 的执行权唯一。
        unsafe { crate::arch::switch_context(task_ctx, scheduler_ctx) };
        resume_task();
        let task = crate::task::current_task_id().expect("resumed wait has no current task");
        trace::record(
            trace::Kind::WaitResume,
            task.as_usize() as u64,
            id.as_usize() as u64,
        );
    }
    // 未提交的提前通知与实际恢复都归还同一进度贡献；yield 不经过此反馈。
    if let Some(worker) = worker {
        worker.running(was_running);
    }
}

/// 为当前 Running Task 开始唯一等待，序号耗尽不允许回绕。
pub(crate) fn begin_wait(task: &Task) -> WaitId {
    let _scheduler = SCHED_LOCK.lock();
    assert_eq!(
        crate::task::current_task_id(),
        Some(task.id()),
        "wait belongs to another task"
    );
    // SAFETY: 调度锁独占 scheduling；调用者强引用保护 task，当前身份核对其 Running 执行权。
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

/// 唯一 Waiter 析构核对本轮身份并清除等待，不改变业务源链接。
pub(crate) fn end_wait(task: &Task, id: WaitId) {
    let _scheduler = SCHED_LOCK.lock();
    assert_eq!(crate::task::current_task_id(), Some(task.id()));
    // SAFETY: 调度锁独占 scheduling；Waiter 的强引用保护 task，析构只能发生在同一任务。
    let data = unsafe { &mut *task.scheduling.get() };
    assert_eq!(data.state, State::Running(cpu::current().id));
    assert_eq!(data.active_wait.expect("ending absent wait").id, id);
    data.active_wait = None;
}

/// 接受本轮首次通知；切换保存前仅记通知，Blocked 入队后允许请求忙核调度，Ready 不重复入队。
pub(crate) fn notify(task: &Task, id: WaitId) -> bool {
    let mut scheduler = SCHED_LOCK.lock();
    // SAFETY: 调度锁独占状态和等待；调用者在锁外升级的强引用覆盖整个访问。
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

/// 在真实 SwitchingOut 放锁后、ctx 保存前通知，并继续同一汇编与 finish 路径。
#[cfg(feature = "ktest")]
pub(crate) fn test_park_notify(task: &Task, id: WaitId, waker: &crate::wait::Waker) {
    assert_switchable();
    let worker = crate::task::current_worker();
    let was_running = worker.as_ref().is_some_and(|worker| worker.sleeping());
    let contexts = prepare_switch(SwitchIntent::Park(id));
    // 普通任务仍必须进入既有 SwitchingOut 测试窗口；仅工作者允许池反馈后的提前通知。
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
        // SAFETY: prepare_switch 已提交 SwitchingOut 且放锁，通知没有转移执行权；IRQ 关闭，
        // 两份 ctx 独占，sleeping 的全部池锁与普通 guard 都已在进入最终 IRQ 段前释放。
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

/// 在唯一调度锁内读取真实状态，不向测试导出写权限。
#[cfg(feature = "ktest")]
pub(crate) fn test_state(task: &Task) -> State {
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY: 调度锁保护 scheduling，调用者借用保持 task 有效，只复制状态值。
    unsafe { (*task.scheduling.get()).state }
}

/// 在真实调度短锁内仅记录标量，不重取调度状态或直接唤醒服务任务。
#[cfg(feature = "ktest")]
pub(crate) fn test_klog() -> u64 {
    let _scheduler = SCHED_LOCK.lock();
    crate::klog!(crate::klog::Level::Debug, "X25 scheduler locked")
}

/// 遍历实际 ready 链并计算同一 Task 的成员次数。
#[cfg(feature = "ktest")]
pub(crate) fn test_ready_occurrences(task: &Task) -> usize {
    let scheduler = SCHED_LOCK.lock();
    let mut member = scheduler.ready_head;
    let mut count = 0;
    while !member.is_null() {
        if ptr::eq(member, task) {
            count += 1;
        }
        // SAFETY: ready 节点由同锁活登记保活，遍历期间链接不会变化。
        member = unsafe { (*(*member).scheduling.get()).ready_next };
    }
    count
}

/// 首次入口与普通恢复都重新识别目的 CPU，再开启本核 IRQ，不恢复旧核 saved 状态。
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

/// 以正常 Rust 调用消耗 FnOnce，入口局部析构结束后才提交退出意图。
pub(crate) extern "C" fn task_trampoline() -> ! {
    resume_task();
    let entry = {
        let _irq = IrqSaveGuard::new();
        let local = cpu::current();
        // SAFETY: 首次 Running 任务由活登记保持，IRQ 关闭且本核独占资源；入口只取走一次。
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

/// 在调度栈清除身份并结束旧根使用，再于锁外释放运行资源和发布离栈结果。
fn finish_current(previous_mm: Option<Arc<crate::mm::space::Mm>>) {
    let (task, owner) = {
        let local = cpu::current();
        // SAFETY: 从 Task 回到本 CPU 调度栈且 IRQ 关闭，只有所属 CPU 访问 current_task。
        let task = unsafe { local.current_task.get().replace(ptr::null()) };
        local.current_task_id.store(0, Ordering::Relaxed);
        (task, local.id)
    };
    assert!(!task.is_null(), "scheduler returned without a task");
    let (retired, affinity_changed) = {
        let mut scheduler = SCHED_LOCK.lock();
        // SAFETY: 当前任务仍属本核离栈交接，锁独占 pending；finish 后不再借用原任务。
        let changed = unsafe {
            let data = &mut *(*task).scheduling.get();
            core::mem::replace(&mut data.affinity_pending, false)
        };
        (scheduler.finish(task, owner), changed)
    };
    if affinity_changed {
        AFFINITY_CHANGED.wake_all();
    }
    if let Some((resources, result)) = retired {
        // SAFETY: finish 已摘离活登记；task 对应此前 into_raw 的那一份强拥有，仅在此恢复一次。
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

/// 每 CPU 从永久 boot 栈进入调度，ready 与 idle 的发表由同一调度锁线性化。
pub fn run() -> ! {
    assert!(
        cpu::current().online.load(Ordering::Acquire),
        "scheduler CPU is offline"
    );
    crate::arch::irq_save();
    accounting::start_scheduler();
    time::activate_current_cpu();
    loop {
        let next = {
            let local = cpu::current();
            let mut scheduler = SCHED_LOCK.lock();
            let next = scheduler.pick(local.id);
            // 已开始一次新分派，或已经证明无本核工作；旧任务的提示不留给下一执行者。
            local.need_resched.store(false, Ordering::Relaxed);
            local.idle.store(next.is_none(), Ordering::Release);
            next
        };
        if let Some(task) = next {
            let local = cpu::current();
            // SAFETY: pick 将唯一 Running 执行权交给本核，活登记保护指针；本 CPU 在 IRQ 关闭时发表身份。
            unsafe { *local.current_task.get() = task };
            // SAFETY: 活 Task 不可被其他 CPU 摘离，身份为不可变字段。
            let id = unsafe { (*task).id().as_usize() };
            local.current_task_id.store(id, Ordering::Relaxed);
            trace::record(trace::Kind::TaskDispatch, id as u64, 0);
            // SAFETY: 目标 Task 具有唯一 Running 执行权且由活登记保活，调度锁已释放。
            let mm = unsafe { (*task).user_thread() }.and_then(|user| user.mm());
            crate::mm::space::enter_current(mm);
            // SAFETY: 本核独占 scheduler ctx；pick 独占目标 Task ctx，二者有效且 IRQ 关闭、调度锁已释放。
            unsafe {
                crate::arch::switch_context(
                    local.scheduler_ctx.get(),
                    (*task).ctx.get().cast_const(),
                )
            };
            let previous_mm = crate::mm::space::leave_current();
            finish_current(previous_mm);
        } else {
            idle_once();
        }
    }
}

/// idle 已在队列锁下发表；放锁后最终重检，等待入口保持 IRQ-off 且不携带 guard。
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
        // SAFETY: 本核已在锁下发表 idle 且最终重检无工作；IRQ 关闭、没有普通 Task 或锁，
        // 架构入口在睡前再次检查原子提示，并保证返回时 IRQ 关闭。
        unsafe { local.idle_once() };
        accounting::idle_leave();
    } else {
        // 待处理提示阻止睡眠，开放 IRQ 消费硬件 pending，队列工作随后重新选择。
        crate::arch::enable_interrupts();
        crate::arch::irq_save();
    }
    let _scheduler = SCHED_LOCK.lock();
    local.idle.store(false, Ordering::Release);
}

/// 在允许集合内指定下一次真实分派 CPU，pick 后恢复集合内的普通选择。
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
        // SAFETY: 本核 IRQ 关闭且调度锁保护许可/next_cpu；活登记保护当前 Running 节点。
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

/// 在调度锁内为已阻塞 Task 指定允许集合内的下一次真实分派 CPU。
#[cfg(feature = "ktest")]
pub(crate) fn test_request_migration(task: &Task, target: usize) {
    assert!(target < cpu::count(), "migration CPU is out of range");
    assert!(
        cpu::get(target).online.load(Ordering::Acquire),
        "migration CPU is offline"
    );
    let _scheduler = SCHED_LOCK.lock();
    // SAFETY: 调度锁保护状态、许可和 next_cpu；调用者强引用保持 Task 活着。
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
