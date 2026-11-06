//! 等待队列（`WaitQueue`）、睡眠互斥锁（`SleepMutex`）、完成量（`Completion`）与 OOM 零分配阻塞路径测试。
//!
//! # 模块职责与架构定位
//! 本模块归属 `ktest::execution_core`，验证内核阻塞唤醒原语在多核并发、状态机切换窗口以及物理内存与 SLUB 堆完全耗尽条件下的正确性：
//! - 上游入口：由 `execution_core::anchors` 注册为 `queue` 与 `mutex` 两个测试套件。
//! - 下游被测模块：`crate::wait`（`Waiter`、`Waker` 代次校验、`WaitQueue` 栈上侵入式节点与 `wake_all` 预算上限）、
//!   `crate::sync`（`SleepMutex` 独占与跨核持有、`Completion` 计数令牌消费）以及 `crate::mm`（物理页与 12 级 SLUB 堆分配器）。
//!
//! # 核心机制与栈上等待节点生命周期
//! ```text
//!   工作线程栈帧                                   共享等待源 (Control::queue / payload / tokens)
//!  +------------------------------------+         +---------------------------------------------+
//!  | let waiter = Waiter::new();        |         | WaitQueue::register(node.as_mut())          |
//!  | pin!(node = WaitNode::new(waker)); | ------> |   链表挂载栈上固定节点 (零堆分配，OOM 可用) |
//!  | waiter.park();                     | <------ | Waker::notify() / wake_one() / wake_all()   |
//!  | drop(registration); // 先脱链      | ------> |   确保节点移出双向链表                      |
//!  | drop(waiter);       // 后弹出栈帧  |         +---------------------------------------------+
//!  +------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试观察槽位 `Slot::task`、`Slot::waker` 与 `Control::old_waker` 各自使用独立的 `SpinLock` 保护。
//! - 锁获取顺序：先获取测试观察 `SpinLock` 克隆出 `Arc<Task>` 或 `Waker`，**立即释放测试 `SpinLock` 后**，
//!   再调用 `Waker::notify()`、`sched::test_state()` 或执行 `drop` 释放引用，严禁在持有测试自旋锁期间重入调度器锁或等待队列锁。
//! - 在 `registration_under_oom` 用例中，所有子线程栈与内存压力容器容量均在 `Waiting::set_up`（耗尽内存之前）预先分配，
//!   且压力块直接存放在捕获边界外的夹具 `Waiting` 中，确保测试中途即使断言失败也能在 `tear_down` 中完整归还物理页与堆对象。

use crate::ktest::execution_core::{baseline, reclaimed, Baseline};
use crate::ktest::execution_services::SuiteBaseline;
use crate::ktest::{spawn_result, spawn_result_on, ChildHandle, ChildToken};
use crate::mm::SLUB_SIZES as CLASSES;
use crate::sync::{Completion, SleepMutex, SpinLock};
use crate::task::{self, State, Task};
use crate::wait::queue::{WaitNode, WaitQueue};
use crate::wait::{Waiter, Waker};
use crate::{cpu, mm, sched};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::alloc::{alloc, dealloc};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::Layout;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// 多线程竞争与批量唤醒用例默认派生的并发工作线程数量。
const TASKS: usize = 4;
/// 每个工作线程在循环竞争或阻塞唤醒用例中执行的迭代轮次数。
const ROUNDS: usize = 4;
/// OOM 压力用例为保存耗尽的物理页块与 SLUB 堆对象而预留的 `Vec` 槽位容量。
const PRESSURE_CAPACITY: usize = 65536;

/// 带超时上限地轮询等待谓词成立；若超时则触发 panic 由当前用例捕获边界拦截，并交由 `Waiting::quiesce` 统一唤醒清理。
fn until(predicate: impl FnMut() -> bool, reason: &'static str) {
    crate::ktest::execution_core::until(predicate, reason).expect(reason);
}

/// 先带超时等待子线程退出内核栈，再通过 `ChildHandle::join` 读取其返回值，确保子线程挂死时能返回错误并进入 `quiesce` 阶段。
fn join(handle: &ChildHandle) -> Result<i32, TestError> {
    crate::ktest::execution_core::until(
        || handle.try_join().is_some(),
        "wait child did not reach original off-stack completion",
    )?;
    handle.join()
}

/// 远端唤醒线程栈上的退出守卫：无论子线程正常返回还是中途断言失败展开，都在退出时唤醒父线程以避免父线程永久阻塞在 `park()`。
struct WakeOnReturn(Arc<Control>);
impl Drop for WakeOnReturn {
    /// 若槽位 0 已发布 `Waker`，在锁外克隆并触发一次唤醒通知。
    fn drop(&mut self) {
        let waker = self.0.slots[0].waker.lock().as_ref().cloned();
        if let Some(waker) = waker {
            waker.notify();
        }
    }
}

/// 单个测试工作线程的身份发布、唤醒句柄与调度统计槽位。
struct Slot {
    /// 工作线程启动后发布的自身 `Arc<Task>` 强引用（由自旋锁保护）。
    task: SpinLock<Option<Arc<Task>>>,
    /// 工作线程当前等待轮次发布的 `Waker` 句柄（由自旋锁保护）。
    waker: SpinLock<Option<Waker>>,
    /// 工作线程当前推进到的测试阶段或轮次序号（`Release` 更新，`Acquire` 观察）。
    phase: AtomicUsize,
    /// 当前正在执行该槽位循环体的活跃线程计数（用于校验同一任务不会并发重入运行）。
    active: AtomicUsize,
    /// 运行期间观测到的 `active` 历史峰值。
    maximum: AtomicUsize,
    /// 工作线程累计经历的主动 `yield_now` 与阻塞 `park` 切换次数。
    switches: AtomicUsize,
}

impl Slot {
    /// 构造空的线程观察槽位（常量初始化，不分配堆内存）。
    const fn new() -> Self {
        Self {
            task: SpinLock::new(None),
            waker: SpinLock::new(None),
            phase: AtomicUsize::new(0),
            active: AtomicUsize::new(0),
            maximum: AtomicUsize::new(0),
            switches: AtomicUsize::new(0),
        }
    }

    /// 将当前正在运行的内核线程强引用发布到本槽位，供协调线程后续查询其调度状态。
    fn publish_task(&self) {
        let current = task::current_task();
        let previous = self.task.lock().replace(current);
        assert!(previous.is_none());
    }

    /// 克隆并返回本槽位已发布的 `Arc<Task>`，调用方在自旋锁外读取其调度状态。
    fn task(&self) -> Arc<Task> {
        Arc::clone(
            self.task
                .lock()
                .as_ref()
                .expect("worker task not published"),
        )
    }

    /// 在自旋锁外克隆本槽位当前发布的 `Waker` 并向其发送唤醒通知，返回本次通知是否首次生效。
    fn notify(&self) -> bool {
        let waker = self
            .waker
            .lock()
            .as_ref()
            .expect("worker waker not published")
            .clone();
        waker.notify()
    }

    /// 移出并释放本槽位持有的 `Arc<Task>` 与 `Waker`，同时清零全部统计原子计数。
    fn clear(&self) {
        let task = self.task.lock().take();
        let waker = self.waker.lock().take();
        drop((task, waker));
        self.phase.store(0, Ordering::Relaxed);
        self.active.store(0, Ordering::Relaxed);
        self.maximum.store(0, Ordering::Relaxed);
        self.switches.store(0, Ordering::Relaxed);
    }
}

/// 等待队列、互斥锁与完成量测试套件跨线程共享的控制块。
struct Control {
    /// 固定 `TASKS` 个工作线程各自的身份与唤醒观察槽位。
    slots: [Slot; TASKS],
    /// 常规等待与唤醒测试使用的共享等待队列。
    queue: WaitQueue,
    /// 验证 `wake_all` 唤醒上限与重入队隔离性使用的专用等待队列。
    drain_queue: WaitQueue,
    /// 用例静止化（`quiesce`）时通知所有工作线程退出循环的协作停止标志。
    stop: AtomicBool,
    /// 控制 `drain_worker` 是否继续下一轮重入队阻塞的运行标志。
    drain_running: AtomicBool,
    /// 已完成首轮入队登记的 `drain_worker` 线程计数。
    drain_initial: AtomicUsize,
    /// 被 `wake_all` 唤醒后再次重新入队登记的累计次数。
    drain_rearms: AtomicUsize,
    /// 保存上一轮已销毁 `Waiter` 遗留的过期 `Waker`（由自旋锁保护），用于验证跨代次唤醒隔离。
    old_waker: SpinLock<Option<Waker>>,
    /// 被测睡眠互斥锁及其保护的非原子共享计数载荷。
    payload: SleepMutex<usize>,
    /// OOM 用例中通知预备子线程在内存耗尽后开始争用 `SleepMutex` 的完成量。
    start_oom: Completion,
    /// OOM 用例中在内存压力释放后放行子线程退出的完成量。
    finish_oom: Completion,
    /// OOM 期间子线程成功获取互斥锁并更新载荷的次数。
    oom_progress: AtomicUsize,
    /// 锁被占用期间调用 `SleepMutex::try_lock` 返回 `None` 的累计次数。
    failed_try: AtomicUsize,
    /// 持有 `SleepMutex` 守卫期间主动调用 `Waiter::park` 阻塞并成功恢复的次数。
    parked: AtomicUsize,
    /// 持有 `SleepMutex` 守卫期间跨核迁移的累计跳数。
    migrations: AtomicUsize,
    /// 验证多消费者令牌语义与令牌窃取竞争的完成量实例。
    tokens: Completion,
    /// 工作线程从 `tokens` 完成量中实际消费成功的令牌总数。
    consumed: AtomicUsize,
}

/// OOM 压力测试中持有一块由全局堆分配器分配的原始内存块的独占守卫。
struct HeapOwner {
    /// `alloc` 成功返回的非空堆内存起始地址。
    pointer: *mut u8,
    /// 分配该内存块时使用的原始大小与对齐布局（`Layout`）。
    layout: Layout,
}

impl Drop for HeapOwner {
    /// 按分配时的原始指针与内存布局将堆对象归还给全局分配器。
    fn drop(&mut self) {
        // SAFETY:
        // 1. `self.pointer` 仅在 `alloc(self.layout)` 返回非空指针时构造存入，指向当前存活的有效堆内存块；
        // 2. `self.layout` 与分配该内存块时传入的 `Layout` 完全一致；
        // 3. `HeapOwner` 未实现 `Copy`/`Clone`，每个分配块由唯一实例独占并在 `drop` 中恰好释放一次。
        unsafe { dealloc(self.pointer, self.layout) };
    }
}

/// 等待与同步原语测试套件的每用例夹具（Fixture）。
struct Waiting {
    /// 用例启动前采集的内核内存与工作队列元数据静止基线。
    initial: Baseline,
    /// 当前用例独占的跨线程共享控制块。
    state: Arc<Control>,
    /// 当前用例已派生的所有子线程观察句柄列表（容量在 `set_up` 中预留）。
    handles: Vec<ChildHandle>,
    /// OOM 测试期间持有的全部物理页块列表，保存在捕获边界外以便统一回收。
    pages: Vec<mm::OwnedPages>,
    /// OOM 测试期间持有的全部 SLUB 堆对象列表，保存在捕获边界外以便统一回收。
    objects: Vec<HeapOwner>,
    /// OOM 用例在施加内存压力前记录的可分配物理页总量基线（页数）。
    oom_pages: Option<usize>,
}

impl Fixture<SuiteBaseline> for Waiting {
    /// 采集用例初始资源基线并创建独立控制块；若当前为 OOM 用例，提前预留物理页与堆对象压力容器的完整容量。
    fn set_up(context: &mut TestContext<'_>, _: &SuiteBaseline) -> Result<Self, TestError> {
        // Step 1: 采集全局静止资源基线并初始化控制块。
        let initial = baseline()?;
        let state = Arc::try_new(Control {
            slots: core::array::from_fn(|_| Slot::new()),
            queue: WaitQueue::new(),
            drain_queue: WaitQueue::new(),
            stop: AtomicBool::new(false),
            drain_running: AtomicBool::new(true),
            drain_initial: AtomicUsize::new(0),
            drain_rearms: AtomicUsize::new(0),
            old_waker: SpinLock::new(None),
            payload: SleepMutex::new(0),
            start_oom: Completion::new(),
            finish_oom: Completion::new(),
            oom_progress: AtomicUsize::new(0),
            failed_try: AtomicUsize::new(0),
            parked: AtomicUsize::new(0),
            tokens: Completion::new(),
            migrations: AtomicUsize::new(0),
            consumed: AtomicUsize::new(0),
        })
        .map_err(|_| TestError("NoMemory: wait controls"))?;

        // Step 2: 预分配子线程句柄槽位；仅针对 `registration_under_oom` 用例在内存充足时预留压力存储槽位。
        let mut handles = Vec::new();
        handles
            .try_reserve_exact(8)
            .map_err(|_| TestError("NoMemory: child observation slots"))?;
        let mut pages = Vec::new();
        let mut objects = Vec::new();
        if context.case.name.ends_with("::registration_under_oom") {
            pages
                .try_reserve_exact(PRESSURE_CAPACITY)
                .map_err(|_| TestError("NoMemory: page pressure slots"))?;
            objects
                .try_reserve_exact(PRESSURE_CAPACITY)
                .map_err(|_| TestError("NoMemory: heap pressure slots"))?;
        }
        Ok(Self {
            initial,
            state,
            handles,
            pages,
            objects,
            oom_pages: None,
        })
    }

    /// 置位所有停止标志并唤醒等待队列、`Slot` 唤醒器及完成量上的全部阻塞线程，确认所有子线程退出内核栈且无残留栈节点。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        // Step 1: 打开全部停止标志，向等待队列、各槽位当前 `Waker` 及所有 `Completion` 发送唤醒信号。
        self.state.stop.store(true, Ordering::Release);
        self.state.drain_running.store(false, Ordering::Release);
        self.state.queue.wake_all();
        self.state.drain_queue.wake_all();
        for slot in &self.state.slots {
            let waker = slot.waker.lock().as_ref().cloned();
            if let Some(waker) = waker {
                waker.notify();
            }
        }
        for _ in 0..TASKS {
            self.state.start_oom.complete();
            self.state.finish_oom.complete();
            self.state.tokens.complete();
        }
        // Step 2: 等待所有子线程离栈结束，并确认所有等待源链表均已清空、不再持有指向任何线程栈帧的节点。
        crate::ktest::execution_core::until(
            || {
                self.handles
                    .iter()
                    .all(|handle| handle.try_join().is_some())
            },
            "wait stop did not reach original off-stack completion",
        )?;
        if self.state.queue.test_members() != 0
            || self.state.drain_queue.test_members() != 0
            || [
                &self.state.start_oom,
                &self.state.finish_oom,
                &self.state.tokens,
            ]
            .iter()
            .any(|source| source.test_waiting() != 0)
        {
            return Err(TestError("wait stop retains original stack registrations"));
        }
        Ok(())
    }

    /// 在子线程全部回收后清空观察引用、释放 OOM 内存压力容器，并校验物理页账务与全局基线完全复原。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        let Self {
            initial,
            state,
            handles,
            mut pages,
            mut objects,
            oom_pages,
        } = self;
        // Step 1: 确认所有子线程均已离栈且等待队列无残留成员，清理各槽位及完成量中的观察引用与剩余令牌。
        if handles.iter().any(|handle| handle.try_join().is_none()) {
            return Err(TestError("waiting Task still on execution stack"));
        }
        if state.queue.test_members() != 0 || state.drain_queue.test_members() != 0 {
            return Err(TestError("wait source still owns stack nodes"));
        }
        for slot in &state.slots {
            slot.clear();
        }
        let old = state.old_waker.lock().take();
        drop(old);
        for source in [&state.start_oom, &state.finish_oom, &state.tokens] {
            if source.test_waiting() != 0 {
                return Err(TestError("completion retains live wait nodes"));
            }
            while source.try_wait() {}
            assert!(!source.is_ready());
        }
        assert!(state
            .slots
            .iter()
            .all(|slot| slot.task.lock().is_none() && slot.waker.lock().is_none()));

        // Step 2: 释放子线程句柄及 OOM 期间占用的堆对象与物理页块，核对物理页总量恢复至施压前水平。
        drop(handles);
        objects.clear();
        pages.clear();
        if let Some(expected) = oom_pages {
            let actual = baseline()?.0;
            if actual.buddy_free_pages + actual.pcp_cached_pages + actual.slab_backing_pages
                != expected
            {
                return Err(TestError(
                    "OOM page accounting did not recover after original join",
                ));
            }
        }
        // Step 3: 释放容器与控制块本身，验证六项内存指标与工作队列元数据回归用例初始基线。
        drop(objects);
        drop(pages);
        drop(state);
        reclaimed(initial)
    }
}

impl Waiting {
    /// 在指定逻辑 CPU 上派生子线程，将返回的 `ChildHandle` 推入夹具预分配槽位并返回其下标。
    fn spawn(
        &mut self,
        cpu: usize,
        body: impl FnOnce(Arc<Control>, ChildToken) -> Result<i32, TestError> + Send + 'static,
    ) -> Result<usize, TestError> {
        let state = Arc::clone(&self.state);
        let handle = spawn_result_on(cpu, move |token| body(state, token))?;
        assert!(self.handles.len() < self.handles.capacity());
        let index = self.handles.len();
        self.handles.push(handle);
        Ok(index)
    }

    /// 派生任意核亲和性（Any）子线程，并将其 `ChildHandle` 登记到夹具预分配槽位中。
    fn spawn_any(
        &mut self,
        body: impl FnOnce(Arc<Control>, ChildToken) -> Result<i32, TestError> + Send + 'static,
    ) -> TestResult {
        let state = Arc::clone(&self.state);
        let handle = spawn_result(move |token| body(state, token))?;
        assert!(self.handles.len() < self.handles.capacity());
        self.handles.push(handle);
        Ok(())
    }
}

/// 等待指定槽位到达给定 `phase` 且对应内核线程在调度器中真正进入 `State::Blocked`、就绪队列出现次数为 0。
fn blocked(slot: &Slot, phase: usize) {
    until(
        || {
            slot.phase.load(Ordering::Acquire) == phase
                && slot
                    .task
                    .lock()
                    .as_ref()
                    .is_some_and(|task| sched::test_state(task) == State::Blocked)
        },
        "wait-worker-blocked",
    );
    assert_eq!(sched::test_ready_occurrences(&slot.task()), 0);
}

/// 执行 `ROUNDS` 轮“主动 `yield_now` + 注册等待队列并 `park` 阻塞”混合切换的工作线程，验证同一线程绝不并发重入或重复入就绪队列。
fn mixed_worker(state: Arc<Control>, token: ChildToken, index: usize) -> Result<i32, TestError> {
    let slot = &state.slots[index];
    slot.publish_task();
    // 每轮依次执行一次主动让出与一次栈上节点入队阻塞，并在唤醒恢复后校验运行态与独占活跃度。
    for round in 1..=ROUNDS {
        if state.stop.load(Ordering::Acquire) || token.stop_requested() {
            break;
        }
        let active = slot.active.fetch_add(1, Ordering::AcqRel) + 1;
        slot.maximum.fetch_max(active, Ordering::AcqRel);
        assert_eq!(active, 1);
        sched::yield_now();
        slot.switches.fetch_add(1, Ordering::Relaxed);
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY:
        // 1. `state` 由当前线程持有的 `Arc<Control>` 保活，`node` 通过 `pin!` 固定在当前循环体栈帧上；
        // 2. 返回的 `registration` 未被 `mem::forget`，并在本轮循环末尾先于 `waiter` 与 `node` 退栈显式执行 `drop(registration)` 脱链。
        let registration = unsafe { state.queue.register(node.as_mut()) };
        let previous = slot.waker.lock().replace(waiter.waker());
        drop(previous);
        slot.phase.store(round, Ordering::Release);
        if !state.stop.load(Ordering::Acquire) && !token.stop_requested() {
            waiter.park();
        }
        slot.switches.fetch_add(1, Ordering::Relaxed);
        assert_eq!(
            sched::test_state(&slot.task()),
            State::Running(cpu::current().id)
        );
        assert_eq!(sched::test_ready_occurrences(&slot.task()), 0);
        assert_eq!(slot.active.fetch_sub(1, Ordering::AcqRel), 1);
        drop(registration);
        drop(waiter);
    }
    Ok((slot.switches.load(Ordering::Acquire) / 2) as i32)
}

/// 测试等待窗口与代次隔离的工作线程：支持在入队后先 `yield_now` 构造 `Ready` 通知窗口，或先销毁首轮 `Waiter` 保留旧 `Waker` 再进入新代次阻塞。
fn window_worker(
    state: Arc<Control>,
    token: ChildToken,
    ready: bool,
    generation: bool,
) -> Result<i32, TestError> {
    let slot = &state.slots[0];
    slot.publish_task();
    // 若启用 `generation` 模式，先完成一轮入队与脱链并将旧代次 `Waker` 存入 `old_waker`，随后使用全新 `Waiter` 进入阻塞。
    if generation {
        let old_id = {
            let waiter = Waiter::new();
            let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `state.queue` 由 `Arc<Control>` 保活，`node` 固定在当前内部作用域栈帧上；
            // 2. `registration` 在本内部块末尾显式执行 `drop(registration)` 完成链表摘除后，内部作用域才销毁 `waiter` 与 `node`。
            let registration = unsafe { state.queue.register(node.as_mut()) };
            let previous = state.old_waker.lock().replace(waiter.waker());
            drop(previous);
            let id = waiter.id();
            drop(registration);
            id
        };
        let waiter = Waiter::new();
        assert_ne!(old_id, waiter.id());
        park_window(&state, &token, slot, &waiter, false);
    } else {
        let waiter = Waiter::new();
        park_window(&state, &token, slot, &waiter, ready);
    }
    slot.phase.store(2, Ordering::Release);
    Ok(if generation { 2 } else { 1 })
}

/// 将栈上固定节点注册到 `state.queue` 并发布 `Waker`；若 `ready` 为真则先调用 `yield_now()` 让线程处于就绪态等待通知，否则直接调用 `park()` 阻塞。
fn park_window(state: &Control, token: &ChildToken, slot: &Slot, waiter: &Waiter, ready: bool) {
    let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
    // SAFETY:
    // 1. 调用方持有 `Arc<Control>` 保证 `state.queue` 存活，`node` 通过 `pin!` 固定在本函数栈帧上；
    // 2. `registration` 覆盖中间的 `yield_now` 与 `park` 全过程，并在函数末尾先于 `node` 退栈显式析构脱链。
    let registration = unsafe { state.queue.register(node.as_mut()) };
    let previous = slot.waker.lock().replace(waiter.waker());
    drop(previous);
    slot.phase.store(1, Ordering::Release);
    if ready {
        sched::yield_now();
    }
    if !state.stop.load(Ordering::Acquire) && !token.stop_requested() {
        waiter.park();
    }
    assert_eq!(
        sched::test_state(&slot.task()),
        State::Running(cpu::current().id)
    );
    drop(registration);
}

/// 循环入队阻塞并在每次被弹出唤醒后立即重新注册下一轮等待的工作线程，用于检验 `WaitQueue::wake_all` 的单轮唤醒预算边界。
fn drain_worker(state: Arc<Control>, token: ChildToken, index: usize) -> Result<i32, TestError> {
    let slot = &state.slots[index];
    slot.publish_task();
    let mut round = 0;
    while state.drain_running.load(Ordering::Acquire) && !token.stop_requested() {
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY:
        // 1. `state.drain_queue` 由 `Arc<Control>` 保活，`node` 固定在当前循环迭代的栈帧上；
        // 2. 每轮迭代末尾均先显式 `drop(registration)` 摘除链表节点，再销毁 `waiter` 与 `node` 进入下一轮或返回。
        let registration = unsafe { state.drain_queue.register(node.as_mut()) };
        round += 1;
        if round == 1 {
            state.drain_initial.fetch_add(1, Ordering::Release);
        } else {
            state.drain_rearms.fetch_add(1, Ordering::Release);
        }
        slot.phase.store(round, Ordering::Release);
        if state.drain_running.load(Ordering::Acquire) && !token.stop_requested() {
            waiter.park();
        }
        drop(registration);
        drop(waiter);
    }
    Ok(round as i32)
}

/// 排空每核页缓存并从高阶到 0 阶循环分配物理页块存入夹具，直至 Buddy 分配器对单页请求也返回 `None`，返回累计占用的 4 KiB 物理页数。
fn exhaust_pages(fixture: &mut Waiting) -> usize {
    let mut held_pages = 0;
    mm::drain_all_pages();
    for order in (0..=mm::MAX_ORDER).rev() {
        loop {
            assert!(
                fixture.pages.len() < fixture.pages.capacity(),
                "execution page pressure storage is full"
            );
            let Some(pages) = mm::alloc_pages(order, false) else {
                break;
            };
            held_pages += pages.len() / mm::PAGE_SIZE;
            fixture.pages.push(pages);
        }
    }
    assert!(0 < held_pages);
    assert!(mm::alloc_pages(0, false).is_none());
    held_pages
}

/// 遍历全部 12 个 SLUB 大小分级循环调用 `alloc`，耗尽各分级现有空闲槽位与后备页直至全部返回空指针，返回触发空指针的分级数量。
fn exhaust_objects(fixture: &mut Waiting) -> usize {
    let mut nulls = 0;
    for size in CLASSES.iter().copied() {
        let layout = Layout::from_size_align(size, 8).unwrap();
        loop {
            assert!(
                fixture.objects.len() < fixture.objects.capacity(),
                "execution object pressure storage is full"
            );
            // SAFETY:
            // 1. 内核内存管理子系统已初始化，且 `CLASSES` 中的每个尺寸均大于 0、对齐为 8，构造出的 `layout` 合法非零；
            // 2. 若返回非空指针，立即封装进唯一的 `HeapOwner` 推入夹具预留槽位，在清理阶段按完全相同的 `layout` 释放。
            let pointer = unsafe { alloc(layout) };
            if pointer.is_null() {
                nulls += 1;
                break;
            }
            fixture.objects.push(HeapOwner { pointer, layout });
        }
    }
    nulls
}

/// 在保持内存耗尽压力的条件下再次尝试分配单页及全部 12 个 SLUB 分级对象，确认分配器确实处于 OOM 状态且未提前恢复；返回累计失败次数。
fn verify_oom() -> usize {
    assert!(mm::alloc_pages(0, false).is_none());
    let mut nulls = 1;
    for size in CLASSES.iter().copied() {
        let layout = Layout::from_size_align(size, 8).unwrap();
        // SAFETY:
        // 1. `layout` 由非零的 `CLASSES` 大小与 8 字节对齐构造，满足全局分配器 `alloc` 的非零布局前置契约；
        // 2. 若意外分配成功（指针非空），立即交由临时 `HeapOwner` 按相同 `layout` 析构归还，避免触发断言展开时泄漏内存。
        let pointer = unsafe { alloc(layout) };
        let recovered = !pointer.is_null();
        if recovered {
            drop(HeapOwner { pointer, layout });
        }
        assert!(
            !recovered,
            "execution allocator recovered before pressure release"
        );
        nulls += 1;
    }
    nulls
}

/// OOM 阻塞测试工作线程：在内存充足时预先启动并阻塞在 `start_oom` 上；待全局物理页与堆耗尽后被唤醒，争用已被占用的 `SleepMutex` 并在获锁更新后阻塞于 `finish_oom`。
fn oom_worker(state: Arc<Control>, _: ChildToken) -> Result<i32, TestError> {
    let slot = &state.slots[0];
    slot.publish_task();
    slot.phase.store(1, Ordering::Release);
    state.start_oom.wait();
    if state.stop.load(Ordering::Acquire) {
        return Ok(0);
    }
    slot.phase.store(2, Ordering::Release);
    let mut guard = state.payload.lock();
    *guard += 1;
    state.oom_progress.fetch_add(1, Ordering::Release);
    drop(guard);
    state.finish_oom.wait();
    Ok(1)
}

/// `SleepMutex` 互斥与内存可见性测试工作线程：循环获取睡眠锁更新非原子计数器，其中 0 号线程在首轮持锁期间额外执行跨核往返迁移与主动 `park` 阻塞。
fn payload_worker(state: Arc<Control>, token: ChildToken, index: usize) -> Result<i32, TestError> {
    sched::test_migrate_to(0);
    let slot = &state.slots[index];
    slot.publish_task();
    assert!(state.payload.try_lock().is_none());
    state.failed_try.fetch_add(1, Ordering::Relaxed);
    slot.phase.store(1, Ordering::Release);
    let mut updates = 0;
    for round in 0..ROUNDS {
        if state.stop.load(Ordering::Acquire) || token.stop_requested() {
            break;
        }
        // 获取睡眠锁；0 号线程在首轮持锁状态下跨核迁移并挂起自身，验证持锁睡眠与迁核期间其他竞争者无法提前窃取互斥锁。
        let mut guard = state.payload.lock();
        let before = *guard;
        if index == 0 && round == 0 {
            if 1 < cpu::count() {
                sched::test_migrate_to(1);
                assert_eq!(cpu::current().id, 1);
                state.migrations.fetch_add(1, Ordering::Relaxed);
                assert_eq!(*guard, before);
                sched::test_migrate_to(0);
                assert_eq!(cpu::current().id, 0);
                state.migrations.fetch_add(1, Ordering::Relaxed);
            }
            let waiter = Waiter::new();
            let previous = slot.waker.lock().replace(waiter.waker());
            drop(previous);
            slot.phase.store(2, Ordering::Release);
            if !state.stop.load(Ordering::Acquire) && !token.stop_requested() {
                waiter.park();
            }
            assert_eq!(*guard, before);
            state.parked.fetch_add(1, Ordering::Release);
        }
        assert_eq!(*guard, before);
        *guard = before + 1;
        updates += 1;
        drop(guard);
        sched::yield_now();
    }
    Ok(updates)
}

/// `Completion` 令牌消费工作线程：阻塞等待 `state.tokens` 完成量，成功消费一个令牌后递增 `consumed` 计数并返回 `1`。
fn token_worker(state: Arc<Control>, _: ChildToken, index: usize) -> Result<i32, TestError> {
    let slot = &state.slots[index];
    slot.publish_task();
    slot.phase.store(1, Ordering::Release);
    state.tokens.wait();
    state.consumed.fetch_add(1, Ordering::Release);
    slot.phase.store(2, Ordering::Release);
    Ok(1)
}

/// 计算 Buddy 空闲页、每核 PCP 缓存页与 SLUB 后备页三者总和，用于校验 OOM 前后可分配物理页总量完全守恒。
fn page_total() -> usize {
    let stats = mm::stats();
    stats.buddy_free_pages + stats.pcp_cached_pages + stats.slab_backing_pages
}

#[::ktest::suite(fixture = Waiting, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "wait"), timeout_ms = 20000)]
pub(crate) mod queue {
    use super::*;

    /// 验证多个工作线程混合执行主动让出与等待队列阻塞时，同一线程绝不并发重入且在就绪队列中最多出现一次。
    ///
    /// - **测试目标**：检验 `Waiter::park` 与 `Waker::notify` 在多轮阻塞唤醒中的幂等性，确保重复调用 `notify()` 不会导致同一任务被重复推入调度器就绪队列。
    /// - **场景步骤**：派生 `TASKS`（4）个工作线程循环 `ROUNDS`（4）轮；每轮主线程等待各工作线程进入 `State::Blocked`，随后对其 `Waker` 连续调用两次 `notify()` 并检查该任务在就绪队列中的出现次数 `test_ready_occurrences`。
    /// - **预期结果**：每次首次 `notify()` 返回 `true`、第二次返回 `false`，任务在就绪队列中的出现次数始终不超过 1；全部 4 个线程完成 32 次切换后正常退出，历史最大并发活跃度 `maximum` 恒为 1。
    #[test]
    fn exclusive_ready_membership(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        // Step 1: 派生 4 个混合切换工作线程，在 4 轮循环中逐个等待其阻塞并测试重复唤醒幂等性。
        sched::test_migrate_to(0);
        for index in 0..TASKS {
            fixture.spawn_any(move |state, token| mixed_worker(state, token, index))?;
        }
        let state = &fixture.state;
        let mut ready_max = 0;
        for round in 1..=ROUNDS {
            for slot in &state.slots {
                blocked(slot, round);
                assert_eq!(slot.active.load(Ordering::Acquire), 1);
                let waker = slot
                    .waker
                    .lock()
                    .as_ref()
                    .expect("mixed worker waker not published")
                    .clone();
                assert!(waker.notify());
                assert!(!waker.notify());
                let ready = sched::test_ready_occurrences(&slot.task());
                assert!(ready <= 1);
                ready_max = ready_max.max(ready);
            }
        }

        // Step 2: 回收所有子线程并核对每个槽位的峰值活跃度、总切换次数及队列清空状态。
        for handle in &fixture.handles {
            assert_eq!(join(handle)?, ROUNDS as i32);
        }
        assert_eq!(
            state
                .slots
                .iter()
                .map(|slot| slot.maximum.load(Ordering::Acquire))
                .max(),
            Some(1)
        );
        assert_eq!(
            state
                .slots
                .iter()
                .map(|slot| slot.switches.load(Ordering::Acquire))
                .sum::<usize>(),
            TASKS * ROUNDS * 2
        );
        if cpu::count() == 1 {
            assert_eq!(ready_max, 1);
        } else {
            assert!(ready_max <= 1);
        }
        for slot in &state.slots {
            assert_eq!(slot.active.load(Ordering::Acquire), 0);
        }
        assert_eq!(state.queue.wake_all(), 0);
        Ok(())
    }

    /// 验证在调用 `Waiter::park()` 之前（无论是在注册 `WaitQueue` 之前还是之后）提前到达的唤醒通知均能被正确锁存，使后续 `park()` 立即返回而不阻塞。
    ///
    /// - **测试目标**：检验 `Waiter` / `Waker` 的预唤醒（Early Notification）状态机转换，防止“检查条件 -> 注册队列 -> 调用 `park()`”窗口期内到达的唤醒丢失。
    /// - **场景步骤**：构造 `Waiter` 与栈上固定的 `WaitNode`；在 `before_source_registration` 用例中先调用 `waker.notify()` 再注册队列，在 `after_source_registration` 用例中先注册队列再调用 `queue.wake_one()`；随后再次调用 `waker.notify()` 并执行 `waiter.park()`。
    /// - **预期结果**：首次唤醒返回 `true`，重复唤醒返回 `false`；`waiter.park()` 不发生阻塞直接返回，当前线程仍处于 `State::Running`，注销后队列成员数为 0。
    #[test]
    #[case("before_source_registration", false)]
    #[case("after_source_registration", true)]
    fn early_notification(_: &mut TestContext<'_>, fixture: &mut Waiting, after: bool) {
        let waiter = Waiter::new();
        let waker = waiter.waker();
        let mut node = core::pin::pin!(WaitNode::new(waker.clone()));
        let accepted = if !after { waker.notify() } else { false };
        // SAFETY:
        // 1. `fixture.state.queue` 在整个用例期间存活，`node` 通过 `pin!` 固定在当前栈帧上；
        // 2. `registration` 在函数末尾显式执行 `drop(registration)` 完成链表脱链后，`node` 与 `waiter` 才随函数返回出栈。
        let registration = unsafe { fixture.state.queue.register(node.as_mut()) };
        let accepted = if after {
            fixture.state.queue.wake_one()
        } else {
            accepted
        };
        assert!(accepted);
        assert!(!waker.notify());
        waiter.park();
        assert_eq!(
            sched::test_state(&task::current_task()),
            State::Running(cpu::current().id)
        );
        drop(registration);
        assert_eq!(fixture.state.queue.test_members(), 0);
    }

    /// 验证当线程已提交阻塞状态（`SwitchingOut`）但尚未完成汇编上下文保存时收到唤醒通知，不会提前入就绪队列，而是在切换完成后安全恢复。
    ///
    /// - **测试目标**：检验 `Waiter::test_park_notify` 模拟的“提交睡眠状态与保存寄存器之间被并发唤醒”窗口期的状态转换安全性。
    /// - **场景步骤**：创建 `Waiter` 并调用 `waiter.test_park_notify()` 触发切换中唤醒路径，返回后再次调用 `waiter.waker().notify()`。
    /// - **预期结果**：线程安全完成切换并恢复运行，且恢复后对同一轮 `Waker` 的重复通知返回 `false`。
    #[test]
    fn switching_out_notification(_: &mut TestContext<'_>, _: &mut Waiting) {
        let waiter = Waiter::new();
        waiter.test_park_notify();
        assert!(!waiter.waker().notify());
    }

    /// 验证工作线程在已阻塞（`Blocked`）或已入队但处于就绪态（`Ready`）两种窗口下收到唤醒通知时，状态机与就绪队列计数均保持正确。
    ///
    /// - **测试目标**：检验当线程已注册等待队列但因主动让出仍处于 `State::Ready` 时收到 `Waker::notify()`，调度器能够锁存该唤醒而不将其重复推入就绪队列。
    /// - **场景步骤**：在 CPU 0 上派生协调线程与工作线程 `window_worker`；在 `blocked` 分支等待工作线程进入 `State::Blocked`，在 `ready` 分支等待工作线程入队后调用 `yield_now()` 停留在 `State::Ready`；随后连续调用两次 `slot.notify()` 并等待工作线程完成 `park()` 返回。
    /// - **预期结果**：首次 `notify()` 返回 `true`、第二次返回 `false`，工作线程在就绪队列中的出现次数恒为 1，最终顺利从 `park()` 返回 `1`。
    #[test]
    #[case("blocked", false)]
    #[case("ready", true)]
    fn queued_notification(
        _: &mut TestContext<'_>,
        fixture: &mut Waiting,
        ready: bool,
    ) -> TestResult {
        let coordinator = fixture.spawn(0, move |state, _| {
            let source = Arc::clone(&state);
            let worker =
                spawn_result_on(0, move |token| window_worker(source, token, ready, false))?;
            let slot = &state.slots[0];
            if ready {
                until(
                    || {
                        slot.phase.load(Ordering::Acquire) == 1
                            && slot
                                .task
                                .lock()
                                .as_ref()
                                .is_some_and(|task| sched::test_state(task) == State::Ready)
                    },
                    "wait-worker-ready",
                );
                assert_eq!(sched::test_ready_occurrences(&slot.task()), 1);
            } else {
                blocked(slot, 1);
            }
            assert!(slot.notify());
            assert!(!slot.notify());
            assert_eq!(sched::test_state(&slot.task()), State::Ready);
            assert_eq!(sched::test_ready_occurrences(&slot.task()), 1);
            assert_eq!(join(&worker)?, 1);
            assert_eq!(slot.phase.load(Ordering::Acquire), 2);
            Ok(0)
        })?;
        assert_eq!(join(&fixture.handles[coordinator])?, 0);
        Ok(())
    }

    /// 验证运行在远端 CPU 1 上的子线程能够跨核唤醒阻塞在 CPU 0 上的父线程。
    ///
    /// - **测试目标**：检验 `Waker::notify()` 的跨核唤醒路径在真实 SMP 核间中断与运行队列锁同步下的正确性。
    /// - **场景步骤**：将主测试线程迁移至 CPU 0 并发布 `Waiter`，随后在 CPU 1 上派生子线程并调用 `waiter.park()` 阻塞主线程；CPU 1 子线程确认主线程在调度器中进入 `State::Blocked` 后调用 `slot.notify()`。
    /// - **预期结果**：主线程从 `park()` 唤醒恢复，CPU 1 子线程返回 `1`（表示 `notify()` 成功生效）。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_notification(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        let slot = &fixture.state.slots[0];
        slot.publish_task();
        let waiter = Waiter::new();
        let previous = slot.waker.lock().replace(waiter.waker());
        drop(previous);
        slot.phase.store(1, Ordering::Release);
        let index = fixture.spawn(1, |state, token| {
            let _wake = WakeOnReturn(Arc::clone(&state));
            until(
                || {
                    state.stop.load(Ordering::Acquire)
                        || token.stop_requested()
                        || sched::test_state(&state.slots[0].task()) == State::Blocked
                },
                "remote parent blocked",
            );
            if state.stop.load(Ordering::Acquire) || token.stop_requested() {
                return Ok(0);
            }
            blocked(&state.slots[0], 1);
            assert_eq!(cpu::current().id, 1);
            Ok(state.slots[0].notify() as i32)
        })?;
        waiter.park();
        drop(waiter);
        assert_eq!(join(&fixture.handles[index])?, 1);
        Ok(())
    }

    /// 验证当被唤醒的线程在 `WaitQueue::wake_all` 执行期间立即重新入队阻塞时，单次 `wake_all` 严格只消费进入时已存在的队列成员，不会陷入无限循环。
    ///
    /// - **测试目标**：检验 `WaitQueue::wake_all` 的快照成员预算上限（Initial Member Budget）机制，防止高频重入队线程饿死唤醒方。
    /// - **场景步骤**：在 CPU 1 上派生 2 个 `drain_worker` 并等待二者在 `drain_queue` 上进入首轮阻塞（`initial == 2`）；在 CPU 0 上调用 `test_wake_all_after_notify`，每次弹出并唤醒一个节点后等待该线程在 CPU 1 上重新注册并进入第 2 轮阻塞。
    /// - **预期结果**：本次 `wake_all` 恰好弹出并唤醒 2 个初始节点后返回，不会继续消费刚重入队的第 2 轮节点；随后置位停止标志并再次调用 `wake_all` 可唤醒这 2 个重入队节点使其正常退出。
    #[test]
    #[ktest(min_cpus = 2)]
    fn finite_wake_all_rearming(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        // Step 1: 在 CPU 1 上启动两个循环重入队线程，等待二者全部完成首轮入队阻塞。
        sched::test_migrate_to(0);
        for index in 0..2 {
            fixture.spawn(1, move |state, token| drain_worker(state, token, index))?;
            blocked(&fixture.state.slots[index], 1);
        }

        // Step 2: 触发带钩子的 `wake_all`，每次唤醒后等待对应线程在 CPU 1 完成第二轮重入队阻塞，断言单次 `wake_all` 严格停在初始成员数 2。
        let state = &fixture.state;
        let initial = state.drain_initial.load(Ordering::Acquire);
        assert_eq!(initial, 2);
        let mut pops = 0;
        let accepted = state.drain_queue.test_wake_all_after_notify(|| {
            pops += 1;
            if pops <= initial {
                blocked(&state.slots[pops - 1], 2);
            } else {
                state.drain_running.store(false, Ordering::Release);
            }
        });
        assert_eq!(
            accepted, initial,
            "wake_all consumed newly registered waits"
        );
        assert_eq!(pops, initial, "wake_all exceeded initial member budget");
        assert_eq!(state.drain_rearms.load(Ordering::Acquire), initial);

        // Step 3: 关闭循环标志并执行最终 `wake_all`，回收两个子线程并确认队列排空。
        state.drain_running.store(false, Ordering::Release);
        assert_eq!(state.drain_queue.wake_all(), initial);
        for handle in &fixture.handles {
            assert_eq!(join(handle)?, 2);
        }
        assert_eq!(state.drain_queue.wake_all(), 0);
        Ok(())
    }

    /// 验证前一轮已销毁 `Waiter` 遗留的旧 `Waker` 无法错误唤醒同一线程在新一轮等待中创建的 `Waiter`。
    ///
    /// - **测试目标**：检验 `Waiter` / `Waker` 基于单调递增 `WaitId` 代次号（Generation）的防ABA误唤醒机制。
    /// - **场景步骤**：派生 `window_worker` 先创建首轮 `Waiter` 将其 `Waker` 存入 `old_waker` 后销毁，再创建新代次 `Waiter` 进入 `State::Blocked`；主线程先取出 `old_waker` 调用 `notify()`，确认无效后再用当前槽位的新 `Waker` 调用 `notify()`。
    /// - **预期结果**：旧 `Waker::notify()` 返回 `false` 且工作线程保持 `State::Blocked`；新 `Waker::notify()` 返回 `true` 并唤醒工作线程正常退出。
    #[test]
    fn stale_waker_new_generation(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        let index = fixture.spawn(0, |state, token| window_worker(state, token, false, true))?;
        let slot = &fixture.state.slots[0];
        blocked(slot, 1);
        let old = fixture.state.old_waker.lock().as_ref().unwrap().clone();
        assert!(!old.notify());
        assert_eq!(slot.phase.load(Ordering::Acquire), 1);
        assert_eq!(sched::test_state(&slot.task()), State::Blocked);
        assert_eq!(sched::test_ready_occurrences(&slot.task()), 0);
        assert!(slot.notify());
        assert_eq!(join(&fixture.handles[index])?, 2);
        assert_eq!(slot.phase.load(Ordering::Acquire), 2);
        drop(old);
        assert_eq!(fixture.state.queue.wake_all(), 0);
        Ok(())
    }

    /// 验证在 Buddy 物理页分配器与全部 12 级 SLUB 堆分配器完全耗尽（OOM）期间，内核等待队列、睡眠互斥锁与完成量仍能零堆分配地正常完成阻塞与唤醒。
    ///
    /// - **测试目标**：确认内核栈上侵入式等待节点（`WaitNode`）与同步原语在运行期不依赖任何动态内存分配，且压力解除后物理页账务精确复原。
    /// - **场景步骤**：记录初始页总量并先锁住 `payload` 睡眠锁，派生 `oom_worker` 停在 `start_oom` 完成量上；随后耗尽所有阶物理页与 12 个 SLUB 分级对象；在持续 OOM 状态下唤醒 `oom_worker` 令其争用 `payload` 进入阻塞，接着释放 `payload` 锁让 `oom_worker` 获锁递增计数并再次阻塞于 `finish_oom`；再次校验系统仍处于 OOM 状态后清空压力容器并唤醒子线程退出。
    /// - **预期结果**：子线程在 OOM 期间顺利完成两次阻塞唤醒并将 `payload` 更新为 `1`，两次 OOM 探测均确认无任何可用页或 SLUB 槽位，压力释放后物理页与堆账务完全恢复。
    #[test]
    fn registration_under_oom(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        // Step 1: 记录初始可分配页总量，预先持锁并启动子线程阻塞在 `start_oom` 门前。
        sched::test_migrate_to(0);
        mm::shrink_all();
        mm::drain_all_pages();
        fixture.oom_pages = Some(page_total());
        let state = Arc::clone(&fixture.state);
        let guard = state.payload.lock();
        let index = fixture.spawn(0, oom_worker)?;
        blocked(&state.slots[0], 1);

        // Step 2: 收缩缓存并彻底耗尽全局 Buddy 物理页与全部 12 个 SLUB 对象分级。
        mm::shrink_all();
        let pages = exhaust_pages(fixture);
        let class_nulls = exhaust_objects(fixture);
        assert_eq!(class_nulls, CLASSES.len());
        let mut nulls = 1 + class_nulls;

        // Step 3: 在零可用内存状态下唤醒子线程争用 `SleepMutex` 阻塞，随后释放锁验证子线程成功获锁更新载荷并再次阻塞于 `finish_oom`。
        state.start_oom.complete();
        blocked(&state.slots[0], 2);
        assert_eq!(sched::test_state(&state.slots[0].task()), State::Blocked);
        drop(guard);
        until(
            || {
                state.oom_progress.load(Ordering::Acquire) == 1
                    && sched::test_state(&state.slots[0].task()) == State::Blocked
            },
            "OOM-worker-progress",
        );
        nulls += verify_oom();
        assert_eq!(*state.payload.lock(), 1);
        assert!(0 < pages);
        assert_eq!(nulls, 2 * (CLASSES.len() + 1));

        // Step 4: 释放全部内存压力块，唤醒子线程退出并核对最终进度。
        fixture.objects.clear();
        fixture.pages.clear();
        state.finish_oom.complete();
        assert_eq!(
            join(&fixture.handles[index])?,
            state.oom_progress.load(Ordering::Acquire) as i32
        );
        state.slots[0].clear();
        Ok(())
    }
}

#[::ktest::suite(fixture = Waiting, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "sync"), timeout_ms = 20000)]
pub(crate) mod mutex {
    use super::*;

    /// 验证四个竞争线程通过 `SleepMutex` 串行更新非原子共享变量时的互斥独占性与跨核内存可见性。
    ///
    /// - **测试目标**：检验 `SleepMutex` 的 `Acquire`/`Release` 内存序保证，以及持锁线程发生跨核迁移或主动阻塞（`park`）期间锁所有权不被其他线程窃取。
    /// - **场景步骤**：主线程先持锁，验证 `try_lock()` 失败后派生 4 个 `payload_worker` 并在锁等待队列上阻塞；释放主线程锁守卫后，0 号线程获锁并在持锁期间跨核往返迁移且挂起在内部 `Waiter` 上，主线程此时再次验证 `try_lock()` 失败后唤醒 0 号线程，等待全部 4 个线程完成各 4 轮累加。
    /// - **预期结果**：最终共享载荷值精确等于 `TASKS * ROUNDS`（16），所有 `try_lock()` 探测均返回 `None`（累计失败 `2 * TASKS + 1` 次）。
    #[test]
    fn ownership_visibility(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        // Step 1: 主线程先持有互斥锁，派生 4 个竞争线程并等待其全部阻塞在锁等待队列中。
        sched::test_migrate_to(0);
        let state = Arc::clone(&fixture.state);
        let guard = state.payload.lock();
        for _ in 0..TASKS {
            assert!(state.payload.try_lock().is_none());
            state.failed_try.fetch_add(1, Ordering::Relaxed);
        }
        for index in 0..TASKS {
            fixture.spawn_any(move |state, token| payload_worker(state, token, index))?;
        }
        for slot in &state.slots {
            blocked(slot, 1);
        }

        // Step 2: 释放主线程锁，等待 0 号线程持锁迁核并进入 park 阻塞，验证持锁睡眠期间 `try_lock` 依然失败后唤醒 0 号线程。
        drop(guard);
        blocked(&state.slots[0], 2);
        assert!(state.payload.try_lock().is_none());
        state.failed_try.fetch_add(1, Ordering::Relaxed);
        assert!(state.slots[0].notify());
        let mut updates = 0;
        for handle in &fixture.handles {
            updates += join(handle)? as usize;
        }

        // Step 3: 汇总所有竞争线程的更新次数，验证最终共享变量值、失败 `try_lock` 计数与跨核迁移次数。
        let visibility = *state.payload.lock();
        assert_eq!(visibility, TASKS * ROUNDS);
        assert_eq!(updates, visibility);
        assert_eq!(state.failed_try.load(Ordering::Acquire), 2 * TASKS + 1);
        assert_eq!(state.parked.load(Ordering::Acquire), 1);
        assert_eq!(
            state.migrations.load(Ordering::Acquire),
            if cpu::count() == 1 { 0 } else { 2 }
        );
        Ok(())
    }

    /// 验证同一线程在持有 `SleepMutex` 守卫期间再次调用非递归 `try_lock()` 会失败返回 `None`，且不会提前破坏或释放既有锁守卫。
    ///
    /// - **测试目标**：确认 `SleepMutex::try_lock` 在锁已被占用时（含同线程重入尝试）严格返回 `None`，且仅在原守卫析构后才允许重新获取。
    /// - **场景步骤**：获取 `payload` 锁守卫并写入 `37`，在持锁状态下调用 `try_lock()`，断言原守卫仍可正常读写；随后 `drop(guard)` 释放锁并再次调用 `try_lock()`。
    /// - **预期结果**：持锁期间 `try_lock()` 返回 `None`，释放守卫后 `try_lock()` 成功返回包含值 `37` 的新守卫。
    #[test]
    fn recursive_try_lock_preserves_owner(_: &mut TestContext<'_>, fixture: &mut Waiting) {
        let mut guard = fixture.state.payload.lock();
        *guard = 37;
        assert!(fixture.state.payload.try_lock().is_none());
        assert_eq!(*guard, 37);
        drop(guard);
        assert_eq!(
            *fixture
                .state
                .payload
                .try_lock()
                .expect("released mutex remains occupied"),
            37
        );
    }

    /// 验证持有 `!Send` 睡眠互斥锁守卫的内核线程在多个 CPU 之间迁移时，锁的独占所有权与受保护数据的可变借用始终有效。
    ///
    /// - **测试目标**：确认虽然 `SleepMutexGuard` 标记为 `!Send`（禁止跨线程传递），但持有该守卫的同一个内核线程在跨 CPU 调度迁移后仍能安全持有并释放该守卫。
    /// - **场景步骤**：将当前线程迁移到 CPU 0 并获取 `payload` 锁写入 `0x7307`，随后携带该栈上守卫依次迁移到 CPU 1 再迁回 CPU 0，在每个核上核对载荷值与 `try_lock()` 互斥状态。
    /// - **预期结果**：两次跨核迁移后当前 CPU 与任务 ID 符合预期，守卫解引用值恒为 `0x7307` 且 `try_lock()` 始终返回 `None`。
    #[test]
    #[ktest(min_cpus = 2)]
    fn guard_migration(_: &mut TestContext<'_>, fixture: &mut Waiting) {
        sched::test_migrate_to(0);
        let id = task::current_task_id();
        let mut guard = fixture.state.payload.lock();
        *guard = 0x7307;
        for target in [1, 0].iter().copied() {
            sched::test_migrate_to(target);
            assert_eq!(cpu::current().id, target);
            assert_eq!(task::current_task_id(), id);
            assert_eq!(*guard, 0x7307);
            assert!(fixture.state.payload.try_lock().is_none());
        }
    }

    /// 验证 `Completion::complete` 按调用次数精确发放令牌，唤醒对应数量的阻塞消费者且不产生多余令牌。
    ///
    /// - **测试目标**：检验 `Completion` 计数信号量语义下的多等待者逐个唤醒与令牌一对一消费守恒。
    /// - **场景步骤**：在 CPU 0 上派生 `TASKS`（4）个 `token_worker` 并等待其全部阻塞在 `tokens.wait()` 上；随后连续调用 `TASKS` 次 `tokens.complete()` 并等待所有子线程返回。
    /// - **预期结果**：4 个消费者线程各消费 1 个令牌并正常返回，累计消费数为 4，且事后调用 `tokens.try_wait()` 返回 `false`（无剩余令牌）。
    #[test]
    fn completion_tokens(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        for index in 0..TASKS {
            fixture.spawn(0, move |state, token| token_worker(state, token, index))?;
        }
        for slot in &fixture.state.slots {
            blocked(slot, 1);
        }

        for _ in 0..TASKS {
            fixture.state.tokens.complete();
        }

        let mut consumers = 0;
        for handle in &fixture.handles {
            consumers += join(handle)? as usize;
        }
        assert_eq!(consumers, TASKS);
        assert_eq!(fixture.state.consumed.load(Ordering::Acquire), consumers);
        assert!(!fixture.state.tokens.try_wait());
        Ok(())
    }

    /// 验证当 `Completion::complete` 唤醒等待线程使其进入 `State::Ready`、但该线程尚未上核执行时，同核其他线程可通过 `try_wait()` 抢先消费该令牌，原等待线程上核后会重新阻塞直到新令牌到达。
    ///
    /// - **测试目标**：检验 `Completion::wait` 被唤醒恢复后的循环重检（Re-check）逻辑，防止令牌被并发 `try_wait()` 窃取后等待者发生虚假返回（Spurious Return）。
    /// - **场景步骤**：在 CPU 0 上启动协调线程与工作线程 `token_worker`；等待工作线程阻塞后调用一次 `tokens.complete()` 使其变为 `State::Ready`，协调线程立即调用 `tokens.try_wait()` 窃取该令牌，观察工作线程重新回到 `State::Blocked` 且 `consumed == 0`；随后再次调用 `tokens.complete()` 发放第二个令牌。
    /// - **预期结果**：第一次令牌被 `try_wait()` 窃取后工作线程重新阻塞而未提前完成；第二次发放令牌后工作线程成功消费并退出。
    #[test]
    fn stolen_completion_token(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        let coordinator = fixture.spawn(0, |state, _| {
            let source = Arc::clone(&state);
            let worker = spawn_result_on(0, move |token| token_worker(source, token, 0))?;
            blocked(&state.slots[0], 1);
            state.tokens.complete();
            assert_eq!(sched::test_state(&state.slots[0].task()), State::Ready);
            assert!(state.tokens.try_wait());
            blocked(&state.slots[0], 1);
            assert_eq!(state.consumed.load(Ordering::Acquire), 0);
            state.tokens.complete();
            assert_eq!(join(&worker)?, 1);
            assert_eq!(state.consumed.load(Ordering::Acquire), 1);
            assert!(!state.tokens.try_wait());
            Ok(0)
        })?;
        assert_eq!(join(&fixture.handles[coordinator])?, 0);
        Ok(())
    }
}
