//! 普通任务逐轮消费精确 timer 与共享 worker 的完成事件，显式停止后核对完整资源账。

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

/// 协调观察由原始硬件counter限制，不承担生产唤醒。
fn until(predicate: impl FnMut() -> bool, name: &'static str) {
    super::wait(predicate, name).unwrap();
}

const ROUNDS: usize = 8;

struct Slot {
    task: SpinLock<Option<Arc<Task>>>,
    timer_done: Completion,
    work_done: Completion,
    timer_callbacks: AtomicUsize,
    work_callbacks: AtomicUsize,
    waited: AtomicUsize,
    finished: AtomicBool,
    drops: AtomicUsize,
}

impl Slot {
    /// 固定容量观察和完成源在异步发表前已有稳定存储，回调上下文只保存槽号。
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

struct LocalDrop(usize);

impl Drop for LocalDrop {
    /// 正常入口返回析构一次，计数与完成消费分别证明栈局部和业务已经结束。
    fn drop(&mut self) {
        assert_eq!(
            scenario().slots[self.0]
                .drops
                .fetch_add(1, Ordering::AcqRel),
            0
        );
    }
}

/// 精确硬回调在所属 CPU 记录有限标量，然后发布本轮业务完成 token。
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

/// 共享池的真实 worker 在固定 CPU 记录本轮，再通知普通任务消费业务完成。
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

/// 只有已经消费上一轮两个事件的普通任务才登记下一轮精确期限。
fn arm(timer: Pin<&Timer>) {
    assert!(timer
        .arm(timer::after(Duration::from_millis(1)).expect("mixed timer deadline overflow"))
        .expect("mixed timer closed during production"));
}

/// 每个预备任务消费八轮真实完成；入口拥有 Timer/Work 克隆在正常离栈时归还。
fn participant(index: usize, timer: Pin<Arc<Timer>>, work: Pin<Arc<Work>>) -> i32 {
    let _local = LocalDrop(index);
    let slot = &scenario().slots[index];
    assert_eq!(cpu::current().id, index);
    assert!(task::current_worker().is_none());
    let previous = slot.task.lock().replace(task::current_task());
    assert!(previous.is_none());
    drop(previous);

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
    // Release 发布最后一次源使用；此后只正常返回，不再 arm 或 queue。
    slot.finished.store(true, Ordering::Release);
    ROUNDS as i32
}

/// 首轮尚无任何生产者，真实 Blocked 与完成源成员登记可保持到协调者统一 arm。
fn initially_blocked(index: usize, handle: &ChildHandle) -> bool {
    let slot = &scenario().slots[index];
    let observed = slot.task.lock().as_ref().map(Arc::clone);
    observed.is_some_and(|task| {
        assert_eq!(task.id(), handle.task_id());
        sched::test_state(&task) == State::Blocked && slot.timer_done.test_waiting() == 1
    })
}

/// 真实 close 清状态之后，仍等待锁外归还的异步 Arc 票，不将清位误作拥有已释放。
fn wait_owned_tail<T>(object: &Pin<Arc<T>>) {
    // SAFETY: 仅克隆并观察 Arc 控制块，绝不移出 T；原 Pin 和本地 observer 全程保活同一固定分配。
    let observed = unsafe { Pin::into_inner_unchecked(object.clone()) };
    until(
        || Arc::strong_count(&observed) == 3,
        "mixed asynchronous ownership returned",
    );
    drop(observed);
}

/// 在生产者结束源使用且所有回调已同步关闭后读取真实成员与 token；不消费残留或代替清理。
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

/// 完整预备每核一个任务和两个源，首次真实登记后开始，停止、join、归还拥有再比较两基线。
fn test_mixed() {
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

    for timer in timers[..cpus].iter().flatten() {
        timer.as_ref().close_sync();
    }
    for work in works[..cpus].iter().flatten() {
        work.as_ref().close_sync();
    }
    queue.flush();
    let mut stopped = 0;
    let mut pending = 0;
    // 域尚未关闭，Work 的 closed 快照只能来自对象自身的永久准入关闭。
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

struct Scenario {
    nodes: super::Nodes,
    stop: AtomicBool,
    slots: [Slot; MAX_CPUS],
}

static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 只克隆fixture已保存的真实into_raw拥有；所有回调关闭且child原join后才撤登记。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY: 同VM顺序case，fixture在catch外持Arc与登记票；quiesce先停所有源，原scope join后才撤指针。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}

struct ServiceFixture {
    state: Arc<Scenario>,
    initial: super::Baseline,
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 每个case从真实常驻基线重新准备独立计数、等待源及原节点owner表。
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

    /// 先合作停止真实生产者并释放停点/gate，再同步停止原节点；不在失败时收栈。
    fn quiesce(&mut self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        self.state.stop_sources();
        self.state.nodes.stop()?;
        Ok(())
    }

    /// runner已join每个原Task；尾票归还以后才释放context/owner并比较完整两账。
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
        // SAFETY: 此指针对应setup保存的唯一登记票；所有异步源停止、原scope已join且无回调借用。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl Scenario {
    /// fixed动作已退出后一次停止下一轮生产，并让真正等待的参与者沿原完成源离栈。
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
    /// 混合协调者是原fixed-0 Task，参与者仍是每CPU真实PreparedChild，并独立拥有原节点。
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

    /// 全部PreparedChild和原节点先准备后发表；每核八轮timer/work/Task完成与所有容量内日志逐条守恒。
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
