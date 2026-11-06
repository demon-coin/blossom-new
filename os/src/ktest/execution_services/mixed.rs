//! 多核高精度定时器（`Timer`）、共享工作队列（`WorkQueue`）、普通任务（`Task`）与内核日志（`klog`）联合收敛测试。
//!
//! ## 模块职责与架构定位
//! 本模块属于执行服务测试套件（`execution_services`），负责在多核环境下验证高精度定时器中断回调、工作队列共享池工作者线程、普通睡眠任务与内核日志环之间的协同推进与资源守恒。
//! - **上游调用者**：`ktest` 框架通过 `cases` 测试套件入口驱动本模块。
//! - **下游依赖**：定时器子系统（`crate::timer`）、工作队列子系统（`crate::workqueue`）、任务与调度器（`crate::task`、`crate::sched`）、同步原语（`Completion`、`SpinLock`）以及内核日志环（`crate::klog`）。
//!
//! ## 核心设计与每核协同状态机
//! 系统为每个在线 CPU（`0..cpus`）分配一个独立的 `Slot`、一个绑定该核的高精度定时器 `Timer`、一个绑定该核的 `Work` 节点以及一个普通参与者任务 `participant`。每个 CPU 上的 8 轮（`ROUNDS = 8`）推进流程如下：
//!
//! ```text
//! [协调者 (CPU 0)]
//!    │ 1. 预分配并发布所有核的 participant 任务，等待全部阻塞在 timer_done.wait()
//!    │ 2. 统一首次 arm 各核的高精度 Timer (1ms)
//!    ▼
//! ┌─────────────────────────────────────── 每核循环 (round = 0..8) ───────────────────────────────────────┐
//! │  [高精度 Timer 硬中断回调 (timer_callback, hardirq_depth > 0)]                                        │
//! │     ├── 写入内核日志: "X28 timer slot=<i> round=<r>"                                                  │
//! │     ├── timer_callbacks 计数 +1                                                                       │
//! │     └── timer_done.complete() ──> 唤醒同核的普通任务 participant                                      │
//! │                                                                                                       │
//! │  [普通任务 (participant, 普通线程上下文)]                                                             │
//! │     ├── 从 timer_done.wait() 醒来，校验 timer_callbacks == round + 1                                  │
//! │     ├── 调用 work.queue() 将同核 Work 排入共享工作队列                                                │
//! │     └── 阻塞在 work_done.wait()                                                                       │
//! │                                                                                                       │
//! │  [工作队列 Worker 线程 (work_callback, hardirq_depth == 0)]                                           │
//! │     ├── 写入内核日志: "X28 work slot=<i> round=<r>"                                                   │
//! │     ├── work_callbacks 计数 +1                                                                        │
//! │     └── work_done.complete() ──> 再次唤醒同核的普通任务 participant                                   │
//! │                                                                                                       │
//! │  [普通任务 (participant)]                                                                             │
//! │     ├── 从 work_done.wait() 醒来，校验 work_callbacks == round + 1                                    │
//! │     └── 若 round + 1 < 8，调用 arm(timer) 启动下一轮；否则置 finished = true 并退出栈帧 (LocalDrop)   │
//! └───────────────────────────────────────────────────────────────────────────────────────────────────────┘
//!    │
//!    ▼
//! [协调者收尾与核账]
//!    ├── close_sync 所有 Timer / Work / WorkQueue，等待全部任务 join 与 Arc 强引用归还
//!    └── 校验回调次数、唤醒次数、析构次数及 2 * cpus * 8 条内核日志序号与内容严格守恒
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 每个 `Slot` 持有一把自旋锁 `Slot::task`（`SpinLock<Option<Arc<Task>>>`），仅用于在任务启动、初始阻塞检查及拆卸阶段存取任务句柄。
//! - `Slot::task` 与 `Nodes` 内部的容器自旋锁互不嵌套；持锁期间仅拷贝或取出 `Arc<Task>`，不执行任何阻塞或调度操作。

use alloc::sync::Arc;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::boot::MAX_CPUS;
use crate::klog::{self as log, Level};
use crate::ktest::{ChildHandle, PreparedChild};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, State, Task};
use crate::time::Duration;
use crate::timer::{self, Timer, TimerKind};
use crate::workqueue::{self, Work, WorkQueueConfig};
use crate::{cpu, sched};

/// 带硬件计数器超时的条件轮询辅助函数：在超时上限内不断让出 CPU 直到 `predicate` 成立。
fn until(predicate: impl FnMut() -> bool, name: &'static str) {
    super::wait(predicate, name).unwrap();
}

/// 每个 CPU 参与者任务循环消费定时器与工作项事件的轮数。
const ROUNDS: usize = 8;

/// 单个 CPU 槽位的同步原语与事件统计集合。
struct Slot {
    /// 绑定在该 CPU 上的普通参与者任务句柄，由自旋锁保护。
    task: SpinLock<Option<Arc<Task>>>,
    /// 高精度定时器回调完成后唤醒参与者任务的完成量。
    timer_done: Completion,
    /// 工作队列回调完成后唤醒参与者任务的完成量。
    work_done: Completion,
    /// 高精度定时器回调在该槽位上已执行的次数。
    timer_callbacks: AtomicUsize,
    /// 工作队列回调在该槽位上已执行的次数。
    work_callbacks: AtomicUsize,
    /// 参与者任务成功从 `timer_done` 或 `work_done` 唤醒的累计次数（每轮 2 次）。
    waited: AtomicUsize,
    /// 参与者任务是否已完成全部 `ROUNDS` 轮循环。
    finished: AtomicBool,
    /// 参与者任务栈帧上的 `LocalDrop` 哨兵被析构的次数（正常退出时应为 1）。
    drops: AtomicUsize,
}

impl Slot {
    /// 构造初始清零的编译期常量槽位。
    const fn new() -> Self {
        Self {
            task: SpinLock::new(None),
            timer_done: Completion::new(),
            work_done: Completion::new(),
            timer_callbacks: AtomicUsize::new(0),
            work_callbacks: AtomicUsize::new(0),
            waited: AtomicUsize::new(0),
            finished: AtomicBool::new(false),
            drops: AtomicUsize::new(0),
        }
    }
}

/// 放置在参与者任务栈帧上的 RAII 析构哨兵（元组字段 `0` 为当前参与者任务所属的 CPU 槽位下标），用于验证任务函数正常返回并完成栈展开。
struct LocalDrop(usize);

impl Drop for LocalDrop {
    /// 当参与者任务退出栈帧时将对应槽位的 `drops` 计数加 1，并断言此前尚未析构过。
    fn drop(&mut self) {
        assert_eq!(
            scenario().slots[self.0]
                .drops
                .fetch_add(1, Ordering::AcqRel),
            0
        );
    }
}

/// 高精度定时器硬中断回调函数：校验中断上下文与所属 CPU，记录内核日志并唤醒等待 `timer_done` 的参与者任务。
fn timer_callback(timer: Pin<&Timer>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    let index = timer.context();
    let slot = &scenario().slots[index];
    assert_eq!(timer.cpu(), index);
    assert_eq!(cpu::current().id, index);
    assert!(0 < cpu::current().hardirq_depth.load(Ordering::Relaxed));
    let round = slot.timer_callbacks.load(Ordering::Acquire);
    assert!(round < ROUNDS);
    crate::klog!(Level::Info, "X28 timer slot={} round={}", index, round);
    assert_eq!(slot.timer_callbacks.fetch_add(1, Ordering::AcqRel), round);
    slot.timer_done.complete();
}

/// 工作队列回调函数：在共享工作者线程上下文中校验运行状态与所属 CPU，记录内核日志并唤醒等待 `work_done` 的参与者任务。
fn work_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    let index = work.context();
    let slot = &scenario().slots[index];
    assert_eq!(work.cpu(), index);
    assert_eq!(cpu::current().id, index);
    assert_eq!(cpu::current().hardirq_depth.load(Ordering::Relaxed), 0);
    let worker = task::current_task_id().expect("mixed work outside ordinary task");
    let state = workqueue::testing::snapshot(work);
    assert!(state.running);
    assert_eq!(state.worker, worker.as_usize());
    let round = slot.work_callbacks.load(Ordering::Acquire);
    assert!(round < ROUNDS);
    crate::klog!(Level::Info, "X28 work slot={} round={}", index, round);
    assert_eq!(slot.work_callbacks.fetch_add(1, Ordering::AcqRel), round);
    slot.work_done.complete();
}

/// 为指定的高精度定时器注册 1 毫秒后的到期时间，启动下一轮事件循环。
fn arm(timer: Pin<&Timer>) {
    assert!(timer
        .arm(timer::after(Duration::from_millis(1)).expect("mixed timer deadline overflow"))
        .expect("mixed timer closed during production"));
}

/// 绑定在指定 CPU 上的普通参与者任务入口：依次经历 8 轮“等待定时器完成 -> 提交工作项 -> 等待工作项完成 -> 重新激活定时器”循环。
fn participant(index: usize, timer: Pin<Arc<Timer>>, work: Pin<Arc<Work>>) -> i32 {
    // Step 1: 在栈上放置析构哨兵，校验当前 CPU 与普通任务身份，并将任务句柄登记到槽位中。
    let _local = LocalDrop(index);
    let slot = &scenario().slots[index];
    assert_eq!(cpu::current().id, index);
    assert!(task::current_worker().is_none());
    let previous = slot.task.lock().replace(task::current_task());
    assert!(previous.is_none());
    drop(previous);

    // Step 2: 循环消费 8 轮高精度定时器与工作队列完成事件。
    for round in 0..ROUNDS {
        slot.timer_done.wait();
        if scenario().stop.load(Ordering::Acquire) {
            return round as i32;
        }
        slot.waited.fetch_add(1, Ordering::Relaxed);
        assert_eq!(slot.timer_callbacks.load(Ordering::Acquire), round + 1);
        assert_eq!(cpu::current().id, index);

        assert!(work
            .as_ref()
            .queue()
            .expect("mixed work closed during production"));
        slot.work_done.wait();
        if scenario().stop.load(Ordering::Acquire) {
            return round as i32;
        }
        slot.waited.fetch_add(1, Ordering::Relaxed);
        assert_eq!(slot.work_callbacks.load(Ordering::Acquire), round + 1);
        assert_eq!(cpu::current().id, index);
        if round + 1 < ROUNDS {
            arm(timer.as_ref());
        }
    }
    // Step 3: 标记本槽位已完成全部轮次，不再重新提交定时器或工作项。
    slot.finished.store(true, Ordering::Release);
    ROUNDS as i32
}

/// 检查指定槽位的参与者任务是否已完成登记并阻塞在首轮的 `timer_done` 完成量上。
fn initially_blocked(index: usize, handle: &ChildHandle) -> bool {
    let slot = &scenario().slots[index];
    let observed = slot.task.lock().as_ref().map(Arc::clone);
    observed.is_some_and(|task| {
        assert_eq!(task.id(), handle.task_id());
        sched::test_state(&task) == State::Blocked && slot.timer_done.test_waiting() == 1
    })
}

/// 等待异步对象（`Timer` 或 `Work`）在底层子系统中的所有临时强引用全部释放，只保留测试框架持有的固定强引用计数（3 个）。
fn wait_owned_tail<T>(object: &Pin<Arc<T>>) {
    // SAFETY:
    // 1. 此处克隆 `Pin<Arc<T>>` 并调用 `into_inner_unchecked` 仅用于通过 `Arc::strong_count` 观察引用计数；
    // 2. 绝不会通过 `observed` 移动或修改底层 `T` 的内存，且在观察结束后立即 `drop(observed)`，不破坏 `Pin` 不变量。
    let observed = unsafe { Pin::into_inner_unchecked(object.clone()) };
    until(
        || Arc::strong_count(&observed) == 3,
        "mixed asynchronous ownership returned",
    );
    drop(observed);
}

/// 断言所有在线 CPU 槽位的 `timer_done` 与 `work_done` 完成量均无残留等待任务且无未消费的就绪令牌。
fn empty_sources(cpus: usize) -> usize {
    let mut waiters = 0;
    for slot in &scenario().slots[..cpus] {
        waiters += slot.timer_done.test_waiting() + slot.work_done.test_waiting();
        assert!(!slot.timer_done.is_ready());
        assert!(!slot.work_done.is_ready());
    }
    assert_eq!(waiters, 0);
    waiters
}

/// 执行多核高精度定时器、工作队列、普通任务与内核日志环的完整集成测试。
fn test_mixed() {
    // Step 1: 创建共享工作队列，并为每个在线 CPU 预分配高精度定时器、工作项和参与者子任务。
    let cpus = cpu::count();
    let queue = scenario().nodes.queue(WorkQueueConfig {
        max_active: cpus,
        ordered: false,
        reclaim: false,
    });
    let timers: [Option<Pin<Arc<Timer>>>; MAX_CPUS] = core::array::from_fn(|index| {
        (index < cpus).then(|| {
            scenario()
                .nodes
                .timer(index, TimerKind::Precise, timer_callback, index)
        })
    });
    let works: [Option<Pin<Arc<Work>>>; MAX_CPUS] = core::array::from_fn(|index| {
        (index < cpus).then(|| scenario().nodes.work(&queue, index, work_callback, index))
    });
    let prepared: [Option<PreparedChild>; MAX_CPUS] = core::array::from_fn(|index| {
        (index < cpus).then(|| {
            let timer = timers[index].as_ref().unwrap().clone();
            let work = works[index].as_ref().unwrap().clone();
            crate::ktest::prepare_result_on(index, move |_| Ok(participant(index, timer, work)))
                .expect("mixed participant allocation")
        })
    });

    // Step 2: 发布所有参与者任务，并等待它们全部在各自的 CPU 上进入首轮阻塞状态。
    let mut handles: [Option<ChildHandle>; MAX_CPUS] = [const { None }; MAX_CPUS];
    let mut tasks = 0;
    for (index, prepared) in IntoIterator::into_iter(prepared).enumerate() {
        if let Some(prepared) = prepared {
            handles[index] = Some(prepared.publish().unwrap());
            tasks += 1;
        }
    }
    until(
        || (0..cpus).all(|index| initially_blocked(index, handles[index].as_ref().unwrap())),
        "mixed initial source registration and blocked tasks",
    );
    let blocked = (0..cpus)
        .filter(|&index| initially_blocked(index, handles[index].as_ref().unwrap()))
        .count();
    assert_eq!(blocked, cpus);

    // Step 3: 等待后台日志打印服务静默后记录起始日志序号，随后统一激活所有 CPU 的首轮定时器，并等待全部槽位完成 8 轮循环。
    log::testing::wait_service_blocked();
    let seq_first = log::next_sequence();
    for timer in timers[..cpus].iter().flatten() {
        arm(timer.as_ref());
    }
    until(
        || {
            scenario().slots[..cpus]
                .iter()
                .all(|slot| slot.finished.load(Ordering::Acquire))
        },
        "mixed participants stopped production",
    );

    // Step 4: 同步关闭所有定时器与工作项，冲刷并关闭工作队列，确认所有节点均已关闭且无排队或运行中的残留回调。
    for timer in timers[..cpus].iter().flatten() {
        timer.as_ref().close_sync();
    }
    for work in works[..cpus].iter().flatten() {
        work.as_ref().close_sync();
    }
    queue.flush();
    let mut stopped = 0;
    let mut pending = 0;
    // 此时工作队列尚未整体关闭，Work 的 `closed` 状态仅来自前面针对每个节点调用的 `close_sync()`。
    for index in 0..cpus {
        let timer = timer::testing::snapshot(timers[index].as_ref().unwrap().as_ref());
        let work = workqueue::testing::snapshot(works[index].as_ref().unwrap().as_ref());
        stopped += usize::from(timer.closed) + usize::from(work.closed);
        pending += usize::from(timer.queued)
            + usize::from(timer.running)
            + usize::from(work.queued)
            + usize::from(work.running);
    }
    assert_eq!(stopped, 2 * cpus);
    assert_eq!(pending, 0);

    // Step 5: 关闭工作队列，回收所有参与者任务与异步对象的强引用，并核对回调次数、等待唤醒次数及栈帧析构次数。
    queue.close_sync();
    let _waiters = empty_sources(cpus);
    for handle in handles[..cpus].iter_mut() {
        assert_eq!(handle.take().unwrap().join().unwrap(), ROUNDS as i32);
    }
    for slot in &scenario().slots[..cpus] {
        let observed = slot.task.lock().take();
        drop(observed);
    }
    for timer in timers[..cpus].iter().flatten() {
        wait_owned_tail(timer);
    }
    for work in works[..cpus].iter().flatten() {
        wait_owned_tail(work);
    }
    drop((timers, works, queue, handles));

    let timer_callbacks = scenario().slots[..cpus]
        .iter()
        .map(|slot| slot.timer_callbacks.load(Ordering::Acquire))
        .sum::<usize>();
    let work_callbacks = scenario().slots[..cpus]
        .iter()
        .map(|slot| slot.work_callbacks.load(Ordering::Acquire))
        .sum::<usize>();
    let waited = scenario().slots[..cpus]
        .iter()
        .map(|slot| slot.waited.load(Ordering::Acquire))
        .sum::<usize>();
    let drops = scenario().slots[..cpus]
        .iter()
        .map(|slot| slot.drops.load(Ordering::Acquire))
        .sum::<usize>();
    assert_eq!(tasks, cpus);
    assert_eq!(timer_callbacks, cpus * ROUNDS);
    assert_eq!(work_callbacks, cpus * ROUNDS);
    assert_eq!(waited, 2 * cpus * ROUNDS);
    assert_eq!(drops, cpus);

    // Step 6: 推进后台日志服务并遍历日志游标，校验测试期间产生的 `2 * cpus * ROUNDS` 条日志序号连续、无丢失且每个 `(source, slot, round)` 组合恰好出现一次。
    let catchup = log::next_sequence();
    let log_records = catchup.checked_sub(seq_first).unwrap();
    assert_eq!(log_records, (timer_callbacks + work_callbacks) as u64);
    log::testing::catch_up(catchup);
    log::testing::wait_service_blocked();
    let mut cursor = log::Cursor::from_sequence(seq_first);
    let mut seen = [[[false; ROUNDS]; MAX_CPUS]; 2];
    for sequence in seq_first..catchup {
        let read = cursor.read().expect("mixed callback record missing");
        let record = read.record;
        assert_eq!(read.lost, 0, "bounded mixed records cannot be overwritten");
        assert_eq!(record.seq, sequence);
        assert_eq!(record.level, Level::Info);
        assert!(!record.truncated);
        assert!(0 < record.monotonic_ns);
        let text = record
            .text()
            .strip_prefix("X28 ")
            .expect("mixed callback source prefix mismatch");
        let (kind, fields) = text
            .split_once(" slot=")
            .expect("mixed callback slot field missing");
        let source = match kind {
            "timer" => 0,
            "work" => 1,
            _ => panic!("mixed callback kind mismatch"),
        };
        let (slot, round) = fields
            .split_once(" round=")
            .expect("mixed callback round field missing");
        let slot = slot
            .parse::<usize>()
            .expect("mixed callback slot malformed");
        let round = round
            .parse::<usize>()
            .expect("mixed callback round malformed");
        assert!(slot < cpus && round < ROUNDS);
        let mut expected = log::StackText::<64>::new();
        core::fmt::Write::write_fmt(
            &mut expected,
            format_args!("X28 {} slot={} round={}", kind, slot, round),
        )
        .unwrap();
        assert_eq!(record.text().as_bytes(), expected.as_bytes());
        assert_eq!(record.cpu, slot);
        if source == 1 {
            assert!(record.task.is_some());
        }
        assert!(!seen[source][slot][round], "mixed callback duplicate");
        seen[source][slot][round] = true;
        crate::ktest::report::diagnostic(format_args!(
            "execution-log record seq={} ns={} cpu={} task={} source={} slot={} round={}",
            record.seq,
            record.monotonic_ns,
            record.cpu,
            record.task.map_or(0, |id| id.as_usize()),
            kind,
            slot,
            round
        ));
    }
    assert!(seen.iter().all(|source| source[..cpus]
        .iter()
        .all(|slot| slot.iter().all(|present| *present))));
    assert!(cursor.read().is_none());
    assert_eq!(log::testing::snapshot().next_seq, catchup);
}

/// 混合执行测试场景的全局共享状态。
struct Scenario {
    /// 测试期间创建的定时器、工作项及工作队列节点管理器。
    nodes: super::Nodes,
    /// 通知所有回调与参与者任务提前终止的原子停止标志。
    stop: AtomicBool,
    /// 按 CPU 索引组织的每核同步与统计槽位数组。
    slots: [Slot; MAX_CPUS],
}

/// 指向当前活动测试场景 `Scenario` 的全局原子裸指针，供无捕获参数的函数指针回调（`timer_callback` / `work_callback`）访问。
static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 从全局 `CURRENT` 原子指针恢复当前测试场景的 `Arc<Scenario>` 强引用。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY:
    // 1. `ServiceFixture::set_up` 通过 `Arc::into_raw` 将有效指针存入 `CURRENT`，且前面 `assert!(!pointer.is_null())` 保证了指针非空；
    // 2. `ServiceFixture::tear_down` 仅在所有定时器/工作队列回调同步停止且所有子任务 `join` 完毕后才清空并释放该指针，因此调用期间底层 `Scenario` 始终有效；
    // 3. 先调用 `Arc::increment_strong_count` 增加强引用计数，再调用 `Arc::from_raw` 构造临时 `Arc`，确保返回的 `Arc` 析构时不会提前释放 `CURRENT` 持有的所有权。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}

/// 混合执行测试套件的夹具（Fixture），负责管理单次测试的 `Scenario` 状态与系统资源基线。
struct ServiceFixture {
    /// 当前测试用例的共享场景状态。
    state: Arc<Scenario>,
    /// 测试开始前记录的内存与工作队列元数据资源基线。
    initial: super::Baseline,
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 初始化测试夹具：记录系统资源基线，分配 `Scenario` 并将其原始指针注册到全局 `CURRENT` 中。
    fn set_up(
        _: &mut ::ktest::TestContext<'_>,
        _: &super::SuiteBaseline,
    ) -> Result<Self, ::ktest::TestError> {
        let initial = super::baseline();
        let state = Arc::try_new(Scenario {
            nodes: super::Nodes::new(MAX_CPUS, MAX_CPUS, 0, 1)?,
            stop: AtomicBool::new(false),
            slots: [const { Slot::new() }; MAX_CPUS],
        })
        .map_err(|_| ::ktest::TestError("NoMemory: execution state"))?;
        let pointer = Arc::into_raw(state.clone()).cast_mut();
        assert!(CURRENT
            .compare_exchange(
                core::ptr::null_mut(),
                pointer,
                Ordering::AcqRel,
                Ordering::Acquire
            )
            .is_ok());
        Ok(Self { state, initial })
    }

    /// 静默阶段（Quiesce）：通知所有参与者任务停止等待并退出，同时同步关闭所有定时器、工作项与工作队列节点。
    fn quiesce(&mut self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        self.state.stop_sources();
        self.state.nodes.stop()?;
        Ok(())
    }

    /// 拆卸阶段（Tear Down）：等待异步节点尾部引用归还，校验无残留等待任务，注销全局 `CURRENT` 指针并校验系统内存及元数据回归初始基线。
    fn tear_down(self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        let mut owner = core::mem::ManuallyDrop::new(self);
        owner.state.nodes.tails()?;
        for slot in &owner.state.slots[..cpu::count()] {
            let observed = slot.task.lock().take();
            drop(observed);
            if slot.timer_done.test_waiting() + slot.work_done.test_waiting() != 0 {
                return Err(::ktest::TestError("mixed wait registrations remain"));
            }
        }

        let initial = owner.initial;
        let pointer = CURRENT.swap(core::ptr::null_mut(), Ordering::AcqRel);
        assert_eq!(pointer, Arc::as_ptr(&owner.state).cast_mut());
        // SAFETY:
        // 1. `pointer` 是由 `set_up` 中唯一一次 `Arc::into_raw` 生成并存入 `CURRENT` 的裸指针，上方 `assert_eq!` 已核对其地址与 `owner.state` 一致；
        // 2. 此时所有异步回调与子任务均已停止并回收，没有任何并发访问者再通过 `scenario()` 获取新引用；
        // 3. `Arc::from_raw(pointer)` 回收 `CURRENT` 持有的那份强引用，紧接着 `ManuallyDrop::drop(&mut owner)` 释放 `ServiceFixture` 自身持有的最后一个强引用。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl Scenario {
    /// 设置全局停止标志，并唤醒所有尚未完成的槽位上的 `timer_done` 和 `work_done`，使阻塞中的参与者任务能够立即退出。
    fn stop_sources(&self) {
        if self.stop.swap(true, Ordering::AcqRel) {
            return;
        }
        for slot in &self.slots[..cpu::count()] {
            if !slot.finished.load(Ordering::Acquire) {
                slot.timer_done.complete();
                slot.work_done.complete();
            }
        }
    }
}

impl ServiceFixture {
    /// 在固定绑定于 CPU 0 的协调者子任务中执行测试闭包 `action`，若发生异常则自动调用 `Scenario::stop_sources` 解除参与者阻塞。
    fn fixed(
        &self,
        action: impl FnOnce() -> ::ktest::TestResult + Send + 'static,
    ) -> ::ktest::TestResult {
        super::on_zero(self.state.clone(), Scenario::stop_sources, action)
    }
}

#[::ktest::suite(fixture = ServiceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "integration", "klog"), timeout_ms = 45000)]
pub(crate) mod cases {
    use super::*;

    /// 测试目标：验证多核环境下高精度定时器（硬中断）、共享工作队列（工作者线程）、普通任务阻塞唤醒与无锁内核日志环在多轮并发交错下的完成守恒与资源零泄漏。
    /// 场景步骤：
    /// 1. Given: 为每个在线 CPU 预分配并发布一个普通任务 `participant`、一个高精度 `Timer` 和一个绑定在共享 `WorkQueue` 上的 `Work`，等待所有任务阻塞在首轮 `timer_done` 上；
    /// 2. When: 统一激活所有 CPU 的首轮定时器，驱动每个 CPU 连续完成 8 轮“定时器硬中断日志记录并唤醒任务 -> 任务提交 Work 并阻塞 -> Worker 线程日志记录并唤醒任务 -> 任务激活下一轮定时器”闭环；
    /// 3. Then: 所有节点同步关闭且引用归还后，总定时器回调数与工作回调数均为 `cpus * 8`，总等待唤醒数为 `2 * cpus * 8`，任务栈析构数为 `cpus`；同时内核日志环恰好新增 `2 * cpus * 8` 条未丢失记录，覆盖所有核、所有轮次的定时器与工作项事件。
    #[test]
    fn timer_work_task_log_convergence(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_mixed();
            Ok(())
        })
    }
}
