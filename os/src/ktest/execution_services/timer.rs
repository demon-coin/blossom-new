//! 高精度定时器（Precise Timer）、时间轮粗粒度定时器（Coarse Timer）与任务睡眠（Sleep）内核测试套件。
//!
//! # 模块职责与架构定位
//! 本模块属于 `ktest::execution_services` 测试层，负责对内核定时器子系统（`crate::timer`）进行全面验证。
//! 上游由 `ktest` 框架调度运行，下游直接调用 `crate::timer`（高精度红黑树定时器、多级时间轮粗定时器及任务睡眠接口）、
//! `crate::wait`（等待者 `Waiter` 与唤醒器 `Waker`）、`crate::time`（单调时钟与硬件时钟事件重编程）以及 `crate::sched`。
//!
//! # 核心机制与定时器生命周期状态机
//! 1. **固定截止期睡眠与提前唤醒重装**：
//!    - 零延迟或已过期的截止时间直接返回，不分配新的等待标识（`WaitId`）。
//!    - 若睡眠任务在截止时间到达前被 `Waker::notify` 虚假或提前唤醒，会重新注册新的 `Waiter`，
//!      但复用同一个栈上定时器与最初设定的绝对截止时间，直到单调时钟真正跨过截止期才返回。
//! 2. **定时器节点状态流转与同步取消/关闭协议**：
//!
//! ```text
//!   [Idle] ──arm()──► [Queued] ──到期摘离 (PausePoint::Dequeued)──► [Running]
//!     ▲                  │                                           │
//!     └──Cancelled───────┴─try_cancel()                              ├─ try_cancel() ──► Busy
//!     ▲                                                              ├─ arm() ─────────► [Running + Queued]
//!     └────回调执行完毕并归还运行引用 (PausePoint::Tail 之后)──────────┘
//! ```
//!
//!    - `cancel_sync()` / `close_sync()`：若目标定时器正处于 `Running` 状态（无论处于回调执行前 `Dequeued` 还是执行后 `Tail` 窗口），
//!      调用方任务都会阻塞等待，直到远端硬中断或下半部（BH）回调彻底执行完毕并归还内部 `Arc` 强引用后才返回；
//!      `close_sync()` 还会永久置位 `closed` 标志，拒绝一切后续 `arm()` 请求。
//! 3. **跨核更早截止期抢占与红黑树/时间轮不变量**：
//!    - 当远端 CPU 正在将较晚的截止期写入硬件比较器时，若并发插入更早的高精度定时器，重编程暂停恢复后硬件必须保留更早的截止时间。
//!    - 高精度红黑树在高频插入、更新与随机取消下保持节点计数、红黑性质与黑高一致；粗粒度时间轮在跨桶跳跃时通过下半部服务线程按预算（Budget）分批回调。
//!
//! # 锁获取顺序与并发契约
//! - 测试状态中的自旋锁（`Scenario::sleep_task`、`sleep_waker`、`cancel_task`）仅用于存取 `Arc<Task>` 或 `Waker` 克隆，
//!   必须在释放测试自旋锁之后再调用 `sched::test_state`、`waker.notify()` 或定时器接口，严禁与定时器底座锁（Base Lock）或调度器锁嵌套。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::hint::spin_loop;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

use crate::sync::{Completion, IrqSaveGuard, SpinLock};
use crate::task::{self, State, Task};
use crate::time::{self as clock, Duration, MonotonicInstant};
use crate::timer::{self as timers, testing, ArmError, CancelResult, Timer, TimerKind};
use crate::wait::{Waiter, Waker};
use crate::{arch, cpu, sched};

/// 基于底层硬件计数器计算当前时刻起 5 秒后的原始计数值截止上限，作为不可重置的超时观察边界。
fn raw_deadline() -> u64 {
    arch::read_counter()
        .checked_add(
            arch::counter_frequency()
                .checked_mul(5)
                .expect("timer raw interval"),
        )
        .expect("timer raw deadline")
}

/// 在硬件计数器到达 `end` 之前等待 `predicate` 条件成立；根据 `yielding` 选择主动让出 CPU（`yield_now`）或忙等自旋（`spin_loop`）。
fn wait_before(mut predicate: impl FnMut() -> bool, end: u64, yielding: bool, name: &str) {
    while !predicate() {
        assert!(
            arch::read_counter() < end,
            "timer execution timeout: {}",
            name
        );
        if yielding {
            sched::yield_now();
        } else {
            spin_loop();
        }
    }
}

/// 计算当前单调时钟之后 `delay` 时长的绝对截止时间点。
fn deadline(delay: Duration) -> MonotonicInstant {
    timers::after(delay).expect("timer deadline overflow")
}

/// 等待所有 CPU 上的粗粒度定时器下半部（BH）服务线程进入阻塞空闲状态（在锁外读取调度状态以避免与定时器底座锁发生锁序反转）。
pub(super) fn wait_services_blocked() {
    let end = raw_deadline();
    for id in 0..cpu::count() {
        wait_before(
            || {
                testing::service_task(id)
                    .is_some_and(|service| sched::test_state(&service) == State::Blocked)
            },
            end,
            true,
            "timer service initial block",
        );
    }
}

/// 克隆已固定（Pinned）的定时器强引用句柄。
fn clone_timer(timer: &Pin<Arc<Timer>>) -> Pin<Arc<Timer>> {
    timer.clone()
}

/// 等待并断言目标定时器在所有队列与正在执行的回调中均已归还强引用，仅剩余测试侧持有的固定引用计数。
fn assert_unique(timer: &Pin<Arc<Timer>>) {
    // SAFETY:
    // 1. 仅对克隆出的 `Pin<Arc<Timer>>` 解包为 `Arc<Timer>` 以读取强引用计数 `strong_count`；
    // 2. 整个过程中不会移动底层 `Timer` 实例或暴露 `&mut Timer`，破坏不了 `Pin` 的内存位置固定不变量。
    let observer = unsafe { Pin::into_inner_unchecked(timer.clone()) };
    wait_before(
        || Arc::strong_count(&observer) == 3,
        raw_deadline(),
        true,
        "timer final running/queued reference returned",
    );
    assert_eq!(Arc::strong_count(&observer), 3);
    drop(observer);
}

/// 在当前测试用例的节点池中分配并注册一个绑定到指定 `cpu` 的定时器实例。
fn new_timer(
    cpu: usize,
    kind: TimerKind,
    callback: fn(Pin<&Timer>),
    context: usize,
) -> Pin<Arc<Timer>> {
    scenario().nodes.timer(cpu, kind, callback, context)
}

/// 睡眠测试钩子：在任务每次注册睡眠等待时保存最新的 `Waker` 句柄，并递增睡眠注册计数 `sleep_arms`。
fn sleep_armed(waker: Waker) {
    let old = scenario().sleep_waker.lock().replace(waker);
    drop(old);
    scenario().sleep_arms.fetch_add(1, Ordering::Release);
}

/// 验证零延迟（`None`）或已过期截止时间（`Some(expiry)`）的睡眠调用立即返回，且期间不会进入等待队列消耗额外的 `WaitId`。
fn immediate_sleep(delay: Option<MonotonicInstant>) -> usize {
    let before = {
        let waiter = Waiter::new();
        waiter.id()
    };
    match delay {
        Some(expiry) => timers::sleep_until(expiry),
        None => timers::sleep(Duration::ZERO),
    }
    let after = {
        let waiter = Waiter::new();
        waiter.id()
    };
    assert_eq!(
        after,
        before.next(),
        "immediate sleep opened an unnecessary wait"
    );
    usize::from(after == before.next())
}

/// 睡眠工作任务：注册当前任务句柄后调用带钩子的 `test_sleep_until` 睡眠至 `expiry`，并在返回后检查是否早于截止期唤醒。
fn sleep_worker(expiry: MonotonicInstant) -> i32 {
    let old = scenario().sleep_task.lock().replace(task::current_task());
    assert!(old.is_none());
    if scenario().stop.load(Ordering::Acquire) {
        return 113;
    }
    timers::test_sleep_until(expiry, sleep_armed);
    scenario()
        .sleep_early
        .fetch_add(usize::from(clock::now() < expiry), Ordering::Relaxed);
    113
}

/// 验证周期性截止时间计算的相位对齐、零周期与上溢拒绝，以及 `timers::after` 相对延迟转换的时间区间单调性。
fn arithmetic() -> (usize, usize) {
    let previous = MonotonicInstant::from_nanos(7);
    let period = Duration::from_nanos(5);
    let late = MonotonicInstant::from_nanos(28);
    let next = timers::periodic_deadline(previous, period, late).expect("periodic phase");
    assert_eq!(next, MonotonicInstant::from_nanos(32));
    assert!(late < next);
    assert_eq!(
        timers::periodic_deadline(previous, period, MonotonicInstant::from_nanos(12)),
        Some(MonotonicInstant::from_nanos(17))
    );
    assert_eq!(
        timers::periodic_deadline(previous, Duration::ZERO, late),
        None
    );
    assert_eq!(
        timers::periodic_deadline(
            MonotonicInstant::from_nanos(u64::MAX - 1),
            period,
            MonotonicInstant::from_nanos(u64::MAX)
        ),
        None
    );
    let phase = usize::from(next.as_nanos() == 32);

    let delay = Duration::from_millis(1);
    let before = clock::now();
    let after = timers::after(delay).expect("checked after");
    let observed = clock::now();
    assert!(before.checked_add(delay).unwrap() <= after);
    assert!(after <= observed.checked_add(delay).unwrap());
    assert_eq!(timers::after(Duration::MAX), None);
    (phase, usize::from(before < after))
}

/// 验证栈上睡眠定时器在连续三次被外部 `Waker` 提前唤醒后仍保持初始绝对截止时间重新阻塞，第 4 次由到期硬中断唤醒返回。
fn test_sleep() {
    let expiry = deadline(Duration::from_millis(250));
    let end = raw_deadline();
    let handle = crate::ktest::spawn_result_on(0, move |_| Ok(sleep_worker(expiry)))
        .expect("sleep test task");
    let mut blocked = 0;
    let mut wakes = 0;
    let mut kept = 0;
    // Step 1: 循环观察 4 轮阻塞状态；前 3 轮在任务阻塞后主动触发 `waker.notify()` 模拟提前唤醒，校验每次重装仍保持原截止时间 `expiry`。
    for round in 1..=4 {
        wait_before(
            || {
                let observed = scenario().sleep_task.lock().as_ref().map(Arc::clone);
                scenario().sleep_arms.load(Ordering::Acquire) == round
                    && observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
            },
            end,
            true,
            "fixed deadline sleeper block",
        );
        blocked += 1;
        let actual_expiry = testing::sleep_deadline();
        assert_eq!(actual_expiry, expiry);
        kept += usize::from(actual_expiry == expiry);
        assert!(
            clock::now() < expiry,
            "early notifications missed original deadline"
        );
        if round <= 3 {
            let waker = scenario().sleep_waker.lock().as_ref().unwrap().clone();
            assert!(waker.notify());
            wakes += 1;
        }
    }
    // Step 2: 第 4 轮不主动通知，等待硬件定时器到期中断唤醒任务并校验返回时刻不早于 `expiry`。
    wait_before(
        || handle.try_join().is_some(),
        end,
        true,
        "sleep final IRQ wake",
    );
    assert_eq!(handle.join().unwrap(), 113);
    let early = scenario().sleep_early.load(Ordering::Acquire);
    let future = usize::from(expiry <= clock::now());
    let deadline_kept = usize::from(
        kept == blocked && scenario().sleep_arms.load(Ordering::Acquire) == blocked && future == 1,
    );
    assert_eq!((future, deadline_kept, early), (1, 1, 0));
    assert_eq!(wakes, 3);

    // Step 3: 清理测试保存的任务与唤醒器句柄，确认子任务已彻底退出且无残留强引用。
    let old_task = scenario().sleep_task.lock().take().unwrap();
    assert_eq!(sched::test_state(&old_task), State::Exited);
    assert_eq!(Arc::strong_count(&old_task), 1);
    let old_waker = scenario().sleep_waker.lock().take();
    drop((old_task, old_waker));
}

/// 高精度定时器生命周期测试回调：在硬中断上下文中校验不可重入性、`try_cancel` 返回 `Busy` 以及回调内自重装（Self-rearm）。
fn lifecycle_callback(timer: Pin<&Timer>) {
    assert!(0 < cpu::current().hardirq_depth.load(Ordering::Acquire));
    assert_eq!(cpu::current().id, timer.cpu());
    let active = scenario().active.fetch_add(1, Ordering::AcqRel) + 1;
    scenario().max_active.fetch_max(active, Ordering::AcqRel);
    assert_eq!(active, 1);
    assert_eq!(timer.try_cancel(), CancelResult::Busy);
    scenario().busy.fetch_add(1, Ordering::Relaxed);
    scenario().callbacks.fetch_add(1, Ordering::Release);
    if scenario().self_rearm.swap(false, Ordering::AcqRel) {
        assert!(timer.arm(deadline(Duration::from_millis(10))).unwrap());
        scenario().rearmed.fetch_add(1, Ordering::Release);
    }
    assert_eq!(scenario().active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// 同步取消/关闭工作任务：等待 `cancel_gate` 发令后对目标定时器执行 `cancel_sync` 或 `close_sync`，并在返回后校验 `running` 标志已清除。
fn cancel_worker(timer: Pin<Arc<Timer>>, close: bool) -> i32 {
    let state = scenario();
    let mut publication = state.cancel_task.lock();
    if state.stop.load(Ordering::Acquire) {
        return 114;
    }
    let old = publication.replace(task::current_task());
    assert!(old.is_none());
    drop(publication);
    state.cancel_gate.wait();
    state.cancel_started.store(true, Ordering::Release);
    if close {
        timer.as_ref().close_sync();
    } else {
        timer.as_ref().cancel_sync();
    }
    assert!(!testing::snapshot(timer.as_ref()).running);
    state.cancel_done.store(true, Ordering::Release);
    114
}

/// 在定时器回调的指定暂停窗口（`Dequeued` 或 `Tail`）注入跨核同步取消或关闭请求，验证调用方任务阻塞等待直到回调彻底退出后才返回。
fn paused_stop(
    timer: &Pin<Arc<Timer>>,
    point: testing::PausePoint,
    close: bool,
    overlap: bool,
) -> (usize, usize) {
    assert!(1 < cpu::count());
    scenario().cancel_started.store(false, Ordering::Relaxed);
    scenario().cancel_done.store(false, Ordering::Relaxed);
    scenario().cancel_gate_sent.store(false, Ordering::Relaxed);
    // Step 1: 在 CPU 0 上启动同步取消工作任务，并等待其阻塞在 `cancel_gate` 起跑门上。
    let worker_timer = clone_timer(timer);
    let handle = crate::ktest::spawn_result_on(0, move |_| Ok(cancel_worker(worker_timer, close)))
        .expect("sync cancel task");
    let end = raw_deadline();
    wait_before(
        || {
            let observed = scenario().cancel_task.lock().as_ref().map(Arc::clone);
            observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
        },
        end,
        true,
        "sync canceller prepared",
    );

    // Step 2: 在目标定时器上设置回调暂停点 `point` 并启动 10ms 定时，等待远端 CPU 硬中断进入暂停窗口（此时 `running && !queued`）。
    testing::arm_pause(timer.as_ref(), point);
    assert!(timer
        .as_ref()
        .arm(deadline(Duration::from_millis(10)))
        .unwrap());
    wait_before(testing::is_paused, end, false, "callback pause");
    let running = testing::snapshot(timer.as_ref());
    assert!(running.running && !running.queued);
    let inflight = usize::from(running.running);
    // Step 3: 若测试重叠场景（`overlap`），在回调暂停期间再次 `arm` 该定时器，使其同时处于 `running && queued` 状态。
    if overlap {
        assert!(timer
            .as_ref()
            .arm(deadline(Duration::from_secs(1)))
            .unwrap());
        let state = testing::snapshot(timer.as_ref());
        assert!(state.running && state.queued);
    }
    assert_eq!(timer.as_ref().try_cancel(), CancelResult::Busy);

    // Step 4: 放行同步取消工作任务，等待其因定时器处于 `running` 状态而进入 `Blocked` 阻塞等待；若为 `close_sync` 则同时验证后续 `arm` 立即返回 `Err(ArmError::Closed)`。
    scenario().release_cancel();
    wait_before(
        || {
            if !scenario().cancel_started.load(Ordering::Acquire) {
                return false;
            }
            let observed = Arc::clone(scenario().cancel_task.lock().as_ref().unwrap());
            sched::test_state(&observed) == State::Blocked
        },
        end,
        true,
        "sync canceller waits callback tail",
    );
    let sync_wait = {
        let observed = Arc::clone(scenario().cancel_task.lock().as_ref().unwrap());
        usize::from(sched::test_state(&observed) == State::Blocked)
    };
    assert!(!scenario().cancel_done.load(Ordering::Acquire));
    assert!(testing::snapshot(timer.as_ref()).running);
    if close {
        assert!(testing::snapshot(timer.as_ref()).closed);
        assert_eq!(
            timer.as_ref().arm(deadline(Duration::from_secs(1))),
            Err(ArmError::Closed)
        );
    }

    assert!(
        !scenario().fail_after_cancel.load(Ordering::Acquire),
        "controlled timer coordinator failure"
    );
    // Step 5: 解除远端回调暂停点，验证同步取消任务在回调尾部执行完毕后唤醒返回，且定时器收敛为空闲状态。
    testing::release_pause();
    wait_before(
        || handle.try_join().is_some(),
        end,
        true,
        "sync cancel final return",
    );
    assert_eq!(handle.join().unwrap(), 114);
    assert!(scenario().cancel_done.load(Ordering::Acquire));
    let state = testing::snapshot(timer.as_ref());
    assert!(!state.running && !state.queued);
    let observed = scenario().cancel_task.lock().take().unwrap();
    assert_eq!(sched::test_state(&observed), State::Exited);
    assert_eq!(Arc::strong_count(&observed), 1);
    drop(observed);
    assert_eq!((inflight, sync_wait), (1, 1));
    (inflight, sync_wait)
}

/// 验证定时器在队列中直接取消（`Cancelled`）、回调执行期间取消（`Busy`）以及跨核暂停窗口下的同步取消等待（`cancel_sync`）。
fn test_cancel(point: Option<testing::PausePoint>) {
    let owner = if cpu::count() == 1 { 0 } else { 1 };
    let timer = new_timer(owner, TimerKind::Precise, lifecycle_callback, 0);
    assert!(timer
        .as_ref()
        .arm(deadline(Duration::from_secs(1)))
        .unwrap());
    assert!(testing::snapshot(timer.as_ref()).queued);
    let queued = usize::from(timer.as_ref().try_cancel() == CancelResult::Cancelled);
    assert_eq!(queued, 1);

    let before = scenario().callbacks.load(Ordering::Acquire);
    let busy_before = scenario().busy.load(Ordering::Acquire);
    let mut inflight = 0;
    let mut sync_wait = 0;
    if let Some(point) = point {
        let (in_flight, waiting) = paused_stop(&timer, point, false, false);
        inflight += in_flight;
        sync_wait += waiting;
    } else {
        assert!(timer
            .as_ref()
            .arm(deadline(Duration::from_millis(10)))
            .unwrap());
        wait_before(
            || before < scenario().callbacks.load(Ordering::Acquire),
            raw_deadline(),
            true,
            "single busy callback",
        );
        timer.as_ref().cancel_sync();
    }
    let state = testing::snapshot(timer.as_ref());
    assert!(!state.running && !state.queued);
    let after = usize::from(timer.as_ref().try_cancel() == CancelResult::Idle);
    let tail = usize::from(!state.running);
    let busy = scenario().busy.load(Ordering::Acquire) - busy_before;
    assert!(1 <= busy);
    assert_eq!((after, tail), (1, 1));
    assert_eq!(inflight, sync_wait);
    assert_eq!(inflight, usize::from(point.is_some()));
    timer.as_ref().close_sync();
    assert_unique(&timer);
    drop(timer);
}

/// 验证同一个定时器节点的回调内自重装（Self-rearm）、取消后复用，以及在回调运行与队列等待重叠（`running && queued`）时的同步关闭（`close_sync`）语义。
fn test_rearm_close() {
    let owner = if cpu::count() == 1 { 0 } else { 1 };
    let timer = new_timer(owner, TimerKind::Precise, lifecycle_callback, 0);
    let callbacks_before = scenario().callbacks.load(Ordering::Acquire);
    let rearmed_before = scenario().rearmed.load(Ordering::Acquire);
    // Step 1: 启用 `self_rearm` 标志并启动定时器，等待首次回调及其内部自重装的第二次回调均执行完毕。
    scenario().self_rearm.store(true, Ordering::Release);
    assert!(timer
        .as_ref()
        .arm(deadline(Duration::from_millis(10)))
        .unwrap());
    let end = raw_deadline();
    wait_before(
        || callbacks_before + 2 <= scenario().callbacks.load(Ordering::Acquire),
        end,
        true,
        "self rearm second callback",
    );
    timer.as_ref().cancel_sync();
    assert_eq!(timer.as_ref().try_cancel(), CancelResult::Idle);

    // Step 2: 对已取消的定时器再次执行 `arm` -> `try_cancel` -> `arm`，验证取消后的节点可正常复用并触发第三次回调。
    assert!(timer
        .as_ref()
        .arm(deadline(Duration::from_secs(1)))
        .unwrap());
    assert_eq!(timer.as_ref().try_cancel(), CancelResult::Cancelled);
    assert!(timer
        .as_ref()
        .arm(deadline(Duration::from_millis(10)))
        .unwrap());
    wait_before(
        || callbacks_before + 3 <= scenario().callbacks.load(Ordering::Acquire),
        end,
        true,
        "cancelled timer reused",
    );
    timer.as_ref().cancel_sync();
    let reused = usize::from(scenario().callbacks.load(Ordering::Acquire) == callbacks_before + 3);

    // Step 3: 在多核环境下构造 `running && queued` 重叠窗口并执行 `close_sync`，验证关闭后拒绝一切新 `arm` 请求且引用计数收敛。
    let _overlap = if 1 < cpu::count() {
        let (running, blocked) = paused_stop(&timer, testing::PausePoint::Tail, true, true);
        assert_eq!(running, blocked);
        running
    } else {
        timer.as_ref().close_sync();
        0
    };
    let state = testing::snapshot(timer.as_ref());
    assert!(state.closed && !state.running && !state.queued);
    let closed = usize::from(state.closed);
    let rejected = usize::from(
        timer.as_ref().arm(deadline(Duration::from_millis(1))) == Err(ArmError::Closed),
    );
    let rearmed = scenario().rearmed.load(Ordering::Acquire) - rearmed_before;
    let callbacks = scenario().callbacks.load(Ordering::Acquire) - callbacks_before;
    let max_active = scenario().max_active.load(Ordering::Acquire);
    assert!(1 <= rearmed && 2 <= callbacks);
    assert_eq!((max_active, reused, closed, rejected), (1, 1, 1, 1));
    assert_unique(&timer);
    drop(timer);
}

/// 跨核高精度定时器测试回调：在目标 CPU 硬中断上下文中校验到期时间不早于设定截止期，并记录触发的 CPU 集合。
fn remote_callback(timer: Pin<&Timer>) {
    assert_eq!(cpu::current().id, timer.cpu());
    assert!(0 < cpu::current().hardirq_depth.load(Ordering::Acquire));
    if timer.context() == 0 {
        assert_eq!(scenario().remote_fired.fetch_add(1, Ordering::AcqRel), 0);
        scenario().remote_early.fetch_add(
            usize::from(
                clock::now().as_nanos() < scenario().remote_deadline.load(Ordering::Acquire),
            ),
            Ordering::Release,
        );
        scenario()
            .remote_cpus
            .insert(cpu::current().id, Ordering::Release);
    } else {
        panic!("old long timer fired before explicit stop");
    }
}

/// 验证各 CPU 高精度定时器的到期顺序，以及在远端 CPU 重编程硬件比较器的暂停窗口中并发插入更早截止期节点时的重编程抢占与硬件 Idle 返回。
fn test_remote() {
    let mut prepared = Vec::with_capacity(cpu::count());
    for target in 0..cpu::count() {
        prepared.push((
            new_timer(target, TimerKind::Precise, remote_callback, 0),
            new_timer(target, TimerKind::Precise, remote_callback, 1),
        ));
    }
    let mut remote = 0;
    let mut kicks = 0;
    let mut idle = 0;
    let mut first = 0;
    let mut stale = 0;
    // Step 1: 遍历每个目标 CPU，先注册一个 10s 后的长周期定时器 `long_timer`，再并发插入更早到期的 `early_timer`。
    for (target, (early_timer, long_timer)) in prepared.iter().enumerate() {
        scenario().remote_fired.store(0, Ordering::Relaxed);
        scenario().remote_early.store(0, Ordering::Relaxed);
        let end = raw_deadline();
        let long = deadline(Duration::from_secs(10));
        assert!(long_timer.as_ref().arm(long).unwrap());
        if 0 < target {
            let before_kicks = cpu::test_timer_reprograms(target);
            loop {
                assert!(arch::read_counter() < end, "remote ISA idle retry timeout");
                wait_before(
                    || {
                        cpu::get(target).idle.load(Ordering::Acquire)
                            && !cpu::get(target).kick_pending()
                            && cpu::get(target).hardirq_depth.load(Ordering::Acquire) == 0
                            && clock::testing::last_programmed_deadline(target) == long
                    },
                    end,
                    false,
                    "remote long timer installed in idle",
                );
                // 预留 1ms 静默期让目标 CPU 有机会实际进入底层 WFI/IDLE 指令等待。
                let quiet_end = arch::read_counter()
                    .checked_add(arch::counter_frequency().div_ceil(1000))
                    .expect("remote idle quiet interval overflow");
                wait_before(
                    || quiet_end <= arch::read_counter(),
                    end,
                    false,
                    "remote idle quiet preparation",
                );
                if !cpu::get(target).idle.load(Ordering::Acquire)
                    || cpu::get(target).kick_pending()
                    || cpu::get(target).hardirq_depth.load(Ordering::Acquire) != 0
                    || clock::testing::last_programmed_deadline(target) != long
                {
                    continue;
                }
                assert!(0 < cpu::test_idle_waits(target));
                assert!(cpu::get(target).idle.load(Ordering::Acquire));
                let round_kicks = cpu::test_timer_reprograms(target);
                let completed = cpu::test_wait_completions(target);
                clock::testing::arm_program_pause(target);
                cpu::kick(target, cpu::KICK_TIMER_REPROGRAM);
                wait_before(
                    || clock::testing::program_paused(target),
                    end,
                    false,
                    "remote protected program pause",
                );

                // 在重编程暂停点内目标 CPU 的 `test_idle_waits` 计数不会增长；记录当前值并插入 100ms 后的更早定时器 `early_timer`。
                let entered = cpu::test_idle_waits(target);
                let early = deadline(Duration::from_millis(100));
                scenario()
                    .remote_deadline
                    .store(early.as_nanos(), Ordering::Release);
                assert!(early_timer.as_ref().arm(early).unwrap());
                assert_eq!(testing::snapshot(early_timer.as_ref()).expiry, early);
                clock::testing::release_program_pause(target);
                wait_before(
                    || clock::testing::last_programmed_deadline(target) == early,
                    end,
                    false,
                    "earlier deadline survives old program",
                );
                wait_before(
                    || {
                        round_kicks < cpu::test_timer_reprograms(target)
                            && !cpu::get(target).kick_pending()
                            && cpu::get(target).hardirq_depth.load(Ordering::Acquire) == 0
                            && cpu::get(target).idle.load(Ordering::Acquire)
                            && entered < cpu::test_idle_waits(target)
                    },
                    end,
                    false,
                    "remote reprogram kicks and idle call completed",
                );
                // RISC-V 在开中断前递增等待完成计数，LoongArch64 在中断返回后的 IDLE 尾部递增；两者均反映底层硬件等待指令的实际唤醒次数。
                let waits = cpu::test_wait_completions(target) - completed;
                // 若释放暂停后目标 CPU 尚未触发 `early_timer` 且当前时间仍在 `early` 之前，说明本次唤醒确实由重编程 IPI 触发而非定时器到期。
                if 0 < waits
                    && scenario().remote_fired.load(Ordering::Acquire) == 0
                    && clock::now() < early
                {
                    let final_program = clock::testing::last_programmed_deadline(target);
                    stale += usize::from(final_program == long);
                    assert_eq!(
                        final_program, early,
                        "old long program overwrote earlier node"
                    );
                    idle += waits;
                    break;
                }

                // 若因时序竞争错过了观察窗口，则同步取消 `early_timer` 并清理本轮状态，在同一节点上重试下一轮。
                early_timer.as_ref().cancel_sync();
                assert_eq!(scenario().remote_early.load(Ordering::Acquire), 0);
                let state = testing::snapshot(early_timer.as_ref());
                assert!(!state.queued && !state.running && !state.closed);
                scenario().remote_fired.store(0, Ordering::Relaxed);
                scenario().remote_early.store(0, Ordering::Relaxed);
            }
            kicks += cpu::test_timer_reprograms(target) - before_kicks;
            remote += 1;
        } else {
            let early = deadline(Duration::from_millis(10));
            scenario()
                .remote_deadline
                .store(early.as_nanos(), Ordering::Release);
            assert!(early_timer.as_ref().arm(early).unwrap());
        }
        // Step 2: 等待 `early_timer` 在目标 CPU 上触发硬中断回调，校验 `long_timer` 仍留在队列中未误触发，随后关闭两个定时器。
        wait_before(
            || scenario().remote_fired.load(Ordering::Acquire) == 1,
            end,
            true,
            "earlier timer real IRQ",
        );
        early_timer.as_ref().cancel_sync();
        let early_count = scenario().remote_early.load(Ordering::Acquire);
        assert_eq!(early_count, 0);
        assert!(testing::snapshot(long_timer.as_ref()).queued);
        first +=
            usize::from(scenario().remote_fired.load(Ordering::Acquire) == 1 && early_count == 0);
        early_timer.as_ref().close_sync();
        long_timer.as_ref().close_sync();
    }
    assert_eq!(remote, cpu::count() - 1);
    assert!(remote <= kicks && remote <= idle);
    assert_eq!(stale, 0);
    assert_eq!(first, cpu::count());
    assert_eq!(
        scenario().remote_cpus.load(Ordering::Acquire),
        cpu::actual_set()
    );
    for (early, long) in prepared {
        early.as_ref().close_sync();
        long.as_ref().close_sync();
        assert_unique(&early);
        assert_unique(&long);
    }
}

/// 在高精度定时器红黑树上混合执行伪随机、逆序、顺序及相同截止期的插入、原地更新与删除，逐步校验节点总数、红黑树性质与黑高不变量。
fn tree_cases() {
    let mut nodes = Vec::with_capacity(48);
    for _ in 0..48 {
        nodes.push(new_timer(0, TimerKind::Precise, remote_callback, 1));
    }
    let base = deadline(Duration::from_secs(10));
    for pattern in 0..4 {
        // Step 1: 按当前模式（伪随机步长、逆序、顺序或全部相同截止期）依次插入 48 个定时器节点，每步校验红黑树节点数与黑高。
        for step in 0..48 {
            let index = match pattern {
                0 => (step * 17) % 48,
                1 => 47 - step,
                _ => step,
            };
            let offset = if pattern == 3 { 0 } else { index % 13 };
            let expiry = base
                .checked_add(Duration::from_millis(offset as u64))
                .unwrap();
            assert!(nodes[index].as_ref().arm(expiry).unwrap());
            let (members, black_height) = testing::validate_tree(0);
            assert_eq!(members, step + 1);
            assert!(0 < black_height);
        }

        // Step 2: 对其中 24 个已在树中的节点执行相同截止期 `base` 的更新（`arm` 返回 `Ok(false)`），校验节点总数保持 48 不变。
        for step in 0..24 {
            let index = (step * 17) % 48;
            assert!(!nodes[index].as_ref().arm(base).unwrap());
            assert_eq!(testing::validate_tree(0).0, 48);
        }

        // Step 3: 按不同顺序逐一取消全部 48 个节点，每步校验红黑树剩余节点数精确递减。
        for step in 0..48 {
            let index = match pattern {
                1 => step,
                2 => 47 - step,
                _ => (step * 29) % 48,
            };
            assert_eq!(nodes[index].as_ref().try_cancel(), CancelResult::Cancelled);
            assert_eq!(testing::validate_tree(0).0, 47 - step);
        }
    }
    for node in nodes {
        node.as_ref().close_sync();
        assert_unique(&node);
    }
    assert_eq!(testing::validate_tree(0).0, 0);
}

/// 粗粒度定时器测试回调：验证回调运行在 CPU 0 的下半部（BH）上下文而非硬中断中，且每个未取消节点仅在到达截止期后触发一次。
fn coarse_callback(timer: Pin<&Timer>) {
    assert_eq!(cpu::current().id, 0);
    assert_eq!(cpu::current().hardirq_depth.load(Ordering::Acquire), 0);
    assert!(0 < cpu::current().bh_depth.load(Ordering::Acquire));
    let index = timer.context();
    assert_eq!(
        scenario().coarse_hits[index].fetch_add(1, Ordering::AcqRel),
        0
    );
    scenario().coarse_early.fetch_add(
        usize::from(
            clock::now().as_nanos() < scenario().coarse_deadlines[index].load(Ordering::Acquire),
        ),
        Ordering::Relaxed,
    );
    scenario().coarse_fired.fetch_add(1, Ordering::Release);
}

/// 验证粗粒度时间轮（Timer Wheel）的范围检查、跨桶到期积压、批量取消以及下半部（BH）服务线程按预算（Budget）分批续跑的统计守恒。
fn test_coarse() {
    // Step 1: 执行时间轮内部自检，分配 256 个粗粒度定时器并验证超出最大允许跨度（`OutOfRange`）的请求被直接拒绝。
    let wheel_cases = testing::self_check_wheel();
    assert!(12 <= wheel_cases);
    let mut nodes = Vec::with_capacity(256);
    for index in 0..256 {
        nodes.push(new_timer(0, TimerKind::Coarse, coarse_callback, index));
    }
    let before = testing::stats(0);
    let mut armed = 0;
    let mut cancelled = 0;
    let tick = testing::TICK_NS;
    let beyond_range = testing::MAX_TIMEOUT_TICKS
        .checked_add(2)
        .and_then(|ticks| ticks.checked_mul(tick))
        .expect("coarse range boundary overflow");
    assert_eq!(
        nodes[0]
            .as_ref()
            .arm(deadline(Duration::from_nanos(beyond_range))),
        Err(ArmError::OutOfRange),
    );
    let rejected = testing::snapshot(nodes[0].as_ref());
    assert!(!rejected.queued && !rejected.running);

    // Step 2: 在关中断区间内将 256 个节点分布在未来连续 3 个时间轮桶中，立即取消前 160 个节点，随后自旋跨过这 3 个桶的截止时间。
    let jump_start;
    let jump_end;
    {
        let _irq = IrqSaveGuard::new();
        let base = deadline(Duration::from_nanos(tick * 10));
        for (index, node) in nodes.iter().enumerate() {
            let expiry = base
                .checked_add(Duration::from_nanos(tick * (index % 3) as u64))
                .unwrap();
            scenario().coarse_deadlines[index].store(expiry.as_nanos(), Ordering::Release);
            assert!(node.as_ref().arm(expiry).unwrap());
            armed += 1;
        }
        for node in nodes.iter().take(160) {
            assert_eq!(node.as_ref().try_cancel(), CancelResult::Cancelled);
            cancelled += 1;
        }
        // 确认所有注册与取消操作完成时最早的截止时间 `base` 仍在未来，保证 3 个桶不会因过期而坍缩进同一个当前桶。
        assert!(
            clock::now() < base,
            "coarse registration crossed earliest deadline"
        );
        jump_start = clock::now().as_nanos() / tick;
        let end = raw_deadline();
        let advance = arch::counter_frequency().checked_mul(15).unwrap() / 100;
        let raw_end = arch::read_counter().checked_add(advance).unwrap();
        wait_before(
            || raw_end <= arch::read_counter(),
            end,
            false,
            "real counter coarse bucket jump",
        );
        jump_end = clock::now().as_nanos() / tick;
        assert!(
            base.checked_add(Duration::from_nanos(tick * 3)).unwrap() < clock::now(),
            "coarse jump did not cross all requested buckets"
        );
        assert_eq!(scenario().coarse_fired.load(Ordering::Acquire), 0);
    }
    let jump_ticks = jump_end.checked_sub(jump_start).unwrap();
    assert!(2 <= jump_ticks);

    // Step 3: 开启中断后等待下半部服务线程按预算分批执行剩余 96 个回调，逐一关闭节点并核对批次（Batches）与续跑（Continuations）统计。
    let expected = armed - cancelled;
    wait_before(
        || scenario().coarse_fired.load(Ordering::Acquire) == expected,
        raw_deadline(),
        true,
        "coarse BH budget continuation",
    );
    for (index, node) in nodes.iter().enumerate() {
        node.as_ref().close_sync();
        let state = testing::snapshot(node.as_ref());
        assert!(state.closed && !state.running && !state.queued);
        assert_eq!(
            scenario().coarse_hits[index].load(Ordering::Acquire),
            usize::from(160 <= index)
        );
        assert_unique(node);
    }

    let fired = scenario().coarse_fired.load(Ordering::Acquire);
    assert_eq!(scenario().coarse_early.load(Ordering::Acquire), 0);
    wait_services_blocked();
    let after = testing::stats(0);
    let batches = after.bh_batches - before.bh_batches;
    let continuations = after.bh_continuations - before.bh_continuations;
    let max_batch = after.max_bh_callbacks;
    let budget = testing::BH_BUDGET;
    assert_eq!(after.bh_callbacks - before.bh_callbacks, fired);
    assert!(128 <= armed && armed / 2 < cancelled);
    assert_eq!(fired, armed - cancelled);
    assert!(2 <= batches && 1 <= continuations && 1 <= max_batch && max_batch <= budget);
    drop(nodes);
    wait_services_blocked();
    assert_eq!(testing::validate_tree(0).0, 0);
    assert!(scenario().sleep_task.lock().is_none());
    assert!(scenario().sleep_waker.lock().is_none());
    assert!(scenario().cancel_task.lock().is_none());
    assert_eq!(scenario().cancel_gate.test_waiting(), 0);
    assert!(!scenario().cancel_gate.is_ready());
}

/// 定时器测试套件的单例共享状态，保存跨任务与中断回调协调所需的原子计数、锁与完成量。
struct Scenario {
    /// 测试用例分配的定时器节点所有权集合。
    nodes: super::Nodes,
    /// 全局停止标志，通知所有辅助工作任务尽快退出。
    stop: AtomicBool,
    /// 固定截止期睡眠测试的工作任务句柄（由自旋锁保护）。
    sleep_task: SpinLock<Option<Arc<Task>>>,
    /// 睡眠工作任务当前注册的唤醒器（`Waker`）句柄（由自旋锁保护）。
    sleep_waker: SpinLock<Option<Waker>>,
    /// 睡眠工作任务累计注册睡眠等待的次数。
    sleep_arms: AtomicUsize,
    /// 睡眠工作任务早于设定截止时间返回的异常次数（正常应为 0）。
    sleep_early: AtomicUsize,
    /// 生命周期测试回调（`lifecycle_callback`）累计执行次数。
    callbacks: AtomicUsize,
    /// 回调内部调用 `try_cancel` 返回 `CancelResult::Busy` 的累计次数。
    busy: AtomicUsize,
    /// 当前正在执行 `lifecycle_callback` 的并发活跃数。
    active: AtomicUsize,
    /// 观测到的 `lifecycle_callback` 历史最大并发活跃数（用于验证同节点回调不可重入，应为 1）。
    max_active: AtomicUsize,
    /// 是否指示下一次 `lifecycle_callback` 在回调内部执行自重装（Self-rearm）。
    self_rearm: AtomicBool,
    /// 回调内部成功完成自重装的累计次数。
    rearmed: AtomicUsize,
    /// 同步取消/关闭测试的工作任务句柄（由自旋锁保护）。
    cancel_task: SpinLock<Option<Arc<Task>>>,
    /// 控制同步取消工作任务开始执行 `cancel_sync`/`close_sync` 的起跑完成量。
    cancel_gate: Completion,
    /// 记录是否已向 `cancel_gate` 发送过起跑信号，确保主流程与异常清理不会重复触发。
    cancel_gate_sent: AtomicBool,
    /// 同步取消工作任务是否已跨过起跑门并准备调用取消接口。
    cancel_started: AtomicBool,
    /// 同步取消工作任务是否已从 `cancel_sync`/`close_sync` 返回。
    cancel_done: AtomicBool,
    /// 是否在同步取消任务进入阻塞后主动触发受控协调失败（用于验证异常清理路径）。
    fail_after_cancel: AtomicBool,
    /// 跨核高精度定时器测试中更早截止期回调（`context == 0`）在当前轮触发的次数。
    remote_fired: AtomicUsize,
    /// 已成功触发跨核高精度定时器回调的 CPU 位图集合。
    remote_cpus: cpu::AtomicCpuSet,
    /// 跨核高精度定时器早于设定截止期触发的异常次数（正常应为 0）。
    remote_early: AtomicUsize,
    /// 跨核高精度定时器测试当前设定的早期截止时间戳（单位：纳秒）。
    remote_deadline: AtomicU64,
    /// 粗粒度定时器测试回调（`coarse_callback`）累计触发的总次数。
    coarse_fired: AtomicUsize,
    /// 粗粒度定时器早于设定截止期触发的异常次数（正常应为 0）。
    coarse_early: AtomicUsize,
    /// 256 个粗粒度定时器各自设定的绝对截止时间戳（单位：纳秒）。
    coarse_deadlines: [AtomicU64; 256],
    /// 256 个粗粒度定时器各自触发回调的次数统计。
    coarse_hits: [AtomicUsize; 256],
}

/// 指向当前正在运行的测试用例 `Scenario` 实例的全局原子裸指针。
static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 获取当前活动测试用例的 `Arc<Scenario>` 共享引用。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY:
    // 1. `ServiceFixture::set_up` 通过 `Arc::into_raw` 导出了一个强引用计数并将其存入 `CURRENT`，在 `tear_down` 回收所有子任务与定时器回调前该指针始终有效；
    // 2. 先调用 `increment_strong_count` 增加一次强引用，再通过 `Arc::from_raw` 构造临时 `Arc`，保证返回后 `CURRENT` 所持有的原始强引用不会被消耗。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}

/// 定时器测试夹具（Fixture），负责在每个用例前后管理 `Scenario` 生命周期、停止定时器节点并校验全局资源基线。
struct ServiceFixture {
    /// 当前测试用例的共享状态实例。
    state: Arc<Scenario>,
    /// 用例启动前记录的物理内存与工作队列元数据基线快照。
    initial: super::Baseline,
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 初始化单个测试用例：迁移至 CPU 0，采样资源基线，预分配定时器节点容量并注册全局 `Scenario` 指针。
    fn set_up(
        _: &mut ::ktest::TestContext<'_>,
        _: &super::SuiteBaseline,
    ) -> Result<Self, ::ktest::TestError> {
        sched::test_migrate_to(0);
        let initial = super::baseline();
        let state = Arc::try_new(Scenario {
            nodes: super::Nodes::new(256usize.max(2 * cpu::count()), 0, 0, 0)?,
            stop: AtomicBool::new(false),
            sleep_task: SpinLock::new(None),
            sleep_waker: SpinLock::new(None),
            sleep_arms: AtomicUsize::new(0),
            sleep_early: AtomicUsize::new(0),
            callbacks: AtomicUsize::new(0),
            busy: AtomicUsize::new(0),
            active: AtomicUsize::new(0),
            max_active: AtomicUsize::new(0),
            self_rearm: AtomicBool::new(false),
            rearmed: AtomicUsize::new(0),
            cancel_task: SpinLock::new(None),
            cancel_gate: Completion::new(),
            cancel_gate_sent: AtomicBool::new(false),
            cancel_started: AtomicBool::new(false),
            cancel_done: AtomicBool::new(false),
            fail_after_cancel: AtomicBool::new(false),
            remote_fired: AtomicUsize::new(0),
            remote_cpus: cpu::AtomicCpuSet::new(),
            remote_early: AtomicUsize::new(0),
            remote_deadline: AtomicU64::new(0),
            coarse_fired: AtomicUsize::new(0),
            coarse_early: AtomicUsize::new(0),
            coarse_deadlines: [const { AtomicU64::new(0) }; 256],
            coarse_hits: [const { AtomicUsize::new(0) }; 256],
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

    /// 静默阶段（Quiesce）：解除所有回调暂停点并唤醒阻塞的辅助任务，随后同步关闭本用例注册的全部定时器节点。
    fn quiesce(&mut self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        self.state.stop_sources();
        self.state.nodes.stop()?;
        Ok(())
    }

    /// 拆卸阶段（Tear Down）：确认所有定时器回调已归还尾引用、完成量与红黑树均已清空，注销 `CURRENT` 指针并核对内存基线。
    fn tear_down(self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        let mut owner = core::mem::ManuallyDrop::new(self);
        owner.state.nodes.tails()?;
        let task = owner.state.sleep_task.lock().take();
        let waker = owner.state.sleep_waker.lock().take();
        let canceller = owner.state.cancel_task.lock().take();
        drop((task, waker, canceller));
        if owner.state.cancel_gate.test_waiting() != 0 || owner.state.cancel_gate.is_ready() {
            return Err(::ktest::TestError("timer cancel source did not converge"));
        }
        if testing::validate_tree(0).0 != 0 {
            return Err(::ktest::TestError("precise timer tree did not converge"));
        }

        let initial = owner.initial;
        let pointer = CURRENT.swap(core::ptr::null_mut(), Ordering::AcqRel);
        assert_eq!(pointer, Arc::as_ptr(&owner.state).cast_mut());
        // SAFETY:
        // 1. `pointer` 是 `set_up` 中通过 `Arc::into_raw` 存入 `CURRENT` 的唯一裸指针，且已通过原子 `swap` 清空；
        // 2. 此时所有定时器节点已同步关闭且回调尾引用已归还、所有子任务已退出，没有任何并发上下文再访问该指针，因此可安全恢复 `Arc` 并销毁 `owner`。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl Scenario {
    /// 向 `cancel_gate` 发送一次起跑完成信号（通过 `cancel_gate_sent` 保证主流程与异常清理合起来最多只触发一次）。
    fn release_cancel(&self) {
        if !self.cancel_gate_sent.swap(true, Ordering::AcqRel) {
            self.cancel_gate.complete();
        }
    }

    /// 停止当前测试用例的所有等待源与暂停点（幂等操作，仅首次调用生效）。
    fn stop_sources(&self) {
        if self.stop.swap(true, Ordering::AcqRel) {
            return;
        }
        timers::testing::cancel_pause();
        let cancel = self.cancel_task.lock().as_ref().map(Arc::clone);
        if cancel
            .as_ref()
            .is_some_and(|task| sched::test_state(task) != State::Exited)
        {
            self.release_cancel();
        }
        let waker = self.sleep_waker.lock().as_ref().cloned();
        if let Some(waker) = waker {
            waker.notify();
        }
        for id in 0..cpu::count() {
            clock::testing::cancel_program_pause(id);
        }
    }
}

impl ServiceFixture {
    /// 在绑定于 CPU 0 的独立协调任务中执行测试闭包 `action`，并在闭包异常退出时自动触发 `Scenario::stop_sources` 清理。
    fn fixed(
        &self,
        action: impl FnOnce() -> ::ktest::TestResult + Send + 'static,
    ) -> ::ktest::TestResult {
        super::on_zero(self.state.clone(), Scenario::stop_sources, action)
    }
}

#[::ktest::suite(fixture = ServiceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "timer"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// **测试目标**：验证零延迟与已过期截止时间的睡眠调用直接返回，不分配多余的等待队列标识（`WaitId`）。
    /// - **Given**：分别传入 `None`（代表 `Duration::ZERO`）与 `Some(MonotonicInstant::from_nanos(0))`（已过期时刻）；
    /// - **When**：在前后两次构造临时 `Waiter` 之间调用 `sleep` 或 `sleep_until`；
    /// - **Then**：睡眠调用立即返回，且前后两个 `Waiter` 的 `WaitId` 连续递增 1，证明睡眠过程未创建额外的等待轮次。
    #[test]
    #[case("zero", None)]
    #[case("past", Some(MonotonicInstant::from_nanos(0)))]
    fn immediate_deadline(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        deadline: Option<MonotonicInstant>,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || {
            assert_eq!(immediate_sleep(deadline), 1);
            Ok(())
        })
    }

    /// **测试目标**：验证周期性定时器截止时间计算的相位推进、边界条件及 `timers::after` 的单调区间。
    /// - **Given**：给定初始相位、周期、落后当前时刻的参数组合，以及零周期、接近 `u64::MAX` 溢出的边界输入；
    /// - **When**：调用 `timers::periodic_deadline` 与 `timers::after`；
    /// - **Then**：正常输入精确对齐到下一个未来周期点，零周期与上溢输入返回 `None`，且 `after` 落在采样前后的单调时钟加上延迟的闭区间内。
    #[test]
    fn periodic_arithmetic(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            assert_eq!(arithmetic(), (1, 1));
            Ok(())
        })
    }

    /// **测试目标**：验证栈上睡眠定时器在多次被提前通知唤醒后，仍能沿用初始绝对截止时间继续阻塞直到硬件中断到期。
    /// - **Given**：启动睡眠工作任务等待 250ms 后的绝对截止时间 `expiry`；
    /// - **When**：在前 3 轮阻塞时通过 `Waker::notify` 主动提前唤醒任务，第 4 轮等待硬件定时器自然到期；
    /// - **Then**：任务共经历 4 次阻塞，每次重装的截止时间均等于初始 `expiry`，最终在单调时钟跨过 `expiry` 后正常返回。
    #[test]
    fn stack_sleep_original_deadline(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_sleep();
            Ok(())
        })
    }

    /// **测试目标**：验证高精度定时器在队列等待、硬中断回调执行中与回调结束后的取消状态转换。
    /// - **Given**：创建一个高精度定时器，先后对其执行长延迟入队取消与 10ms 短延迟到期回调；
    /// - **When**：在入队期间、硬中断回调内部以及 `cancel_sync` 完成后分别调用 `try_cancel`；
    /// - **Then**：入队期间返回 `Cancelled`，回调执行内部返回 `Busy`，回调彻底结束后返回 `Idle` 且强引用计数完全归还。
    #[test]
    fn callback_busy(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_cancel(None);
            Ok(())
        })
    }

    /// **测试目标**：验证跨核同步取消（`cancel_sync`）在回调刚摘离队列（`Dequeued`）与回调末尾（`Tail`）两个窗口均会阻塞等待回调彻底退出。
    /// - **Given**：在 CPU 1 上触发高精度定时器回调并使其暂停在 `PausePoint::Dequeued` 或 `PausePoint::Tail`；
    /// - **When**：CPU 0 上的工作任务并发调用 `cancel_sync()`，随后主协调任务解除回调暂停点；
    /// - **Then**：回调暂停期间工作任务处于 `Blocked` 状态且未提前返回，解除暂停后工作任务唤醒退出，定时器收敛为空闲状态。
    #[test]
    #[ktest(min_cpus = 2)]
    #[case("dequeued", testing::PausePoint::Dequeued)]
    #[case("tail", testing::PausePoint::Tail)]
    fn cancel_real_tail(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        point: testing::PausePoint,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || {
            test_cancel(Some(point));
            Ok(())
        })
    }

    /// **测试目标**：验证当同步取消任务正阻塞等待远端回调尾部时，若主协调任务发生受控失败，清理路径不会重复发送完成量信号且能安全回收资源。
    /// - **Given**：置位 `fail_after_cancel`，使同步取消任务消费 `cancel_gate` 并阻塞在暂停于 `Tail` 的远端定时器上；
    /// - **When**：主协调任务触发受控断言失败并进入 `ServiceFixture::quiesce` / `tear_down` 清理流程；
    /// - **Then**：清理流程自动解除回调暂停并回收取消任务，`cancel_gate` 无残留信号，框架报告预期的受控失败。
    #[test]
    #[ktest(
        min_cpus = 2,
        ignore = "controlled timer coordinator failure must remain Fail after complete cleanup"
    )]
    fn cancel_coordinator_failure(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            scenario().fail_after_cancel.store(true, Ordering::Release);
            test_cancel(Some(testing::PausePoint::Tail));
            Ok(())
        })
    }

    /// **测试目标**：验证同一个定时器节点的回调内自重装（Self-rearm）、取消后复用以及运行与排队重叠时的同步关闭（`close_sync`）。
    /// - **Given**：创建高精度定时器并启用 `self_rearm`，依次执行自重装触发、取消后再次入队触发，最后构造 `running && queued` 重叠状态；
    /// - **When**：对处于重叠状态的定时器调用 `close_sync()` 并尝试再次 `arm`；
    /// - **Then**：回调最大并发活跃数为 1（无重入），取消后可正常复用，`close_sync` 等待回调结束后置位 `closed` 并以 `Err(ArmError::Closed)` 拒绝后续注册。
    #[test]
    fn rearm_reuse_close(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_rearm_close();
            Ok(())
        })
    }

    /// **测试目标**：验证各 CPU 高精度定时器回调触发，以及远端 CPU 在硬件比较器重编程暂停窗口中并发插入更早截止期时的抢占与 Idle 返回。
    /// - **Given**：在每个 CPU 上注册 10s 长定时器；对远端 CPU 暂停其重编程流程并并发插入 100ms 的更早定时器；
    /// - **When**：解除重编程暂停并等待更早定时器到期触发硬中断；
    /// - **Then**：更早的截止时间未被旧的 10s 重编程覆盖，远端 CPU 实际完成底层硬件 Idle 唤醒，且所有 CPU 均按期触发早期回调。
    #[test]
    fn earlier_remote_deadline(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_remote();
            Ok(())
        })
    }

    /// **测试目标**：验证高精度定时器红黑树在多种插入顺序、相同截止期冲突、原地更新与删除下的结构不变量。
    /// - **Given**：预分配 48 个高精度定时器节点，依次采用伪随机、逆序、顺序与全相同截止期 4 种模式；
    /// - **When**：每轮执行 48 次插入、24 次原地更新与 48 次取消，并在每步调用 `testing::validate_tree(0)`；
    /// - **Then**：每一步的树成员数量与预期完全一致，红黑树颜色规则、黑高与最左最小缓存节点始终有效，最终树收敛为空。
    #[test]
    fn precise_tree_invariants(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            tree_cases();
            Ok(())
        })
    }

    /// **测试目标**：验证粗粒度时间轮在硬件计数器跨过多个非空桶时的批量取消、到期触发与下半部（BH）预算分批续跑守恒。
    /// - **Given**：在关中断下将 256 个粗粒度定时器分布在未来 3 个时间轮桶内，立即取消前 160 个，并自旋等待硬件时钟跨过全部 3 个桶；
    /// - **When**：开中断唤醒下半部服务线程处理到期回调，待全部 96 个剩余节点触发后读取统计信息；
    /// - **Then**：前 160 个已取消节点零触发，后 96 个节点各触发 1 次且无提前触发；下半部执行批次不少于 2、续跑次数不少于 1，且单批回调数不超过预算上限 `BH_BUDGET`。
    #[test]
    fn real_wheel_bucket_jump(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_coarse();
            Ok(())
        })
    }
}
