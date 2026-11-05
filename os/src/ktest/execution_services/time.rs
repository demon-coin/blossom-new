//! 在真实任务、硬中断、单调时钟与 idle 入口上验证时间和执行账。

use super::wait;
use alloc::sync::Arc;
use core::convert::TryFrom;
use core::hint::{black_box, spin_loop};
use core::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

/// 观察窗口失败由主体捕获，真实停止仍由catch外fixture完成。
fn until(predicate: impl FnMut() -> bool, name: &'static str) {
    wait(predicate, name).unwrap();
}
use crate::sync::{Completion, IrqSaveGuard, SpinLock};
use crate::task::{self, State, Task};
use crate::time::{self as clock, Duration, MonotonicInstant};
use crate::{accounting, arch, cpu, sched};

const READ_ROUNDS: usize = 128;
const KICK_ROUNDS: usize = 4;

struct KickSlot {
    task: SpinLock<Option<Arc<Task>>>,
    wake: Completion,
    rounds: AtomicUsize,
    timer_before: AtomicUsize,
    stop: AtomicBool,
    pause_after_round: AtomicBool,
    round_paused: AtomicBool,
}

impl KickSlot {
    /// 为每个绑定执行者准备通知和观察对象，不在提交提醒时分配。
    const fn new() -> Self {
        Self {
            task: SpinLock::new(None),
            wake: Completion::new(),
            rounds: AtomicUsize::new(0),
            timer_before: AtomicUsize::new(0),
            stop: AtomicBool::new(false),
            pause_after_round: AtomicBool::new(false),
            round_paused: AtomicBool::new(false),
        }
    }

    /// 克隆现存真实任务控制块，调度状态观察不保留测试锁。
    fn task(&self) -> Arc<Task> {
        Arc::clone(self.task.lock().as_ref().expect("kick task not published"))
    }

    /// 同一停止请求只发表一张票；未登记执行者由登记锁内的停止检查直接退出。
    fn request_stop(&self) {
        if self.stop.swap(true, Ordering::AcqRel) {
            return;
        }
        let task = self.task.lock().as_ref().map(Arc::clone);
        if task
            .as_ref()
            .is_some_and(|task| sched::test_state(task) != State::Exited)
        {
            self.wake.complete();
        }
    }
}

/// 以硬件 counter 限制不可调度观察段，不用时钟快照、yield 或同步 call 补进度。
fn spin_until(predicate: impl FnMut() -> bool, name: &str) {
    spin_until_before(predicate, raw_deadline(), name);
}

/// 从唯一单调时钟构造有限绝对期限。
fn deadline(duration: Duration) -> MonotonicInstant {
    clock::now()
        .checked_add(duration)
        .expect("time test deadline overflow")
}

/// 以硬件 counter 给跨 CPU 停点建立期限，等待期间不进入被暂停的序列快照。
fn raw_deadline() -> u64 {
    arch::read_counter()
        .checked_add(
            arch::counter_frequency()
                .checked_mul(5)
                .expect("raw timeout interval overflow"),
        )
        .expect("raw timeout deadline overflow")
}

/// 复用整轮绝对截止，合法跳过硬件等待的重试不能重置期限。
fn spin_until_before(mut predicate: impl FnMut() -> bool, end: u64, name: &str) {
    while !predicate() {
        assert!(
            arch::read_counter() < end,
            "time execution timeout: {}",
            name
        );
        spin_loop();
    }
}

/// 在普通任务保持运行权期间制造真实忙段，允许硬 IRQ 服务但不调用安全点。
fn busy(duration: Duration) {
    let end = deadline(duration);
    while clock::now() < end {
        black_box(arch::read_counter());
        spin_loop();
    }
}

/// 新任务的量子 IRQ 只能置请求，同核竞争者到普通安全点才取得运行权。
fn deferred_worker() -> i32 {
    assert_eq!(cpu::current().id, 0);
    busy(Duration::from_millis(20));
    let local = cpu::current();
    let peer = crate::ktest::spawn_result_on(0, |_| {
        Ok({
            assert_eq!(cpu::current().id, 0);
            assert!(!scenario().peer_ran.swap(true, Ordering::AcqRel));
            101
        })
    })
    .expect("reschedule peer allocation failed");
    assert!(!scenario().peer_ran.load(Ordering::Acquire));
    let before = {
        let _irq = IrqSaveGuard::new();
        // SAFETY: 当前 CPU0 普通任务持有本核 IRQ-save，未切换且未持有事件锁；
        // 唯一真实 clockevent 重武装以后在同一 IRQ-off 区间采 timer 基线。
        unsafe { clock::testing::rearm_current_quantum() };
        let before = local.timer_interrupts.load(Ordering::Acquire);
        local.need_resched.store(false, Ordering::Release);
        before
    };
    spin_until(
        || before < local.timer_interrupts.load(Ordering::Acquire),
        "task-quantum-irq",
    );
    assert!(!scenario().peer_ran.load(Ordering::Acquire));
    assert!(local.need_resched.load(Ordering::Acquire));
    assert_eq!(task::current_task_id(), Some(task::current_task().id()));
    let safe_points = usize::from(sched::cond_resched());
    assert_eq!(safe_points, 1);
    assert!(scenario().peer_ran.load(Ordering::Acquire));
    until(|| peer.try_join().is_some(), "reschedule-peer-return");
    assert_eq!(peer.join().unwrap(), 101);
    // SAFETY: 当前绑定 CPU0 的普通任务不持锁；异常入口、永久 CpuLocal 和真实 timer
    // 已安装，受控整数测试不跨调度，退出恢复 IRQ 和当前 clockevent。
    let registers = unsafe { arch::test_integer_register_preservation() };
    assert!(if cfg!(target_arch = "riscv64") {
        2 <= registers
    } else {
        1 <= registers
    });
    let irq = local.timer_interrupts.load(Ordering::Acquire) - before;
    assert!(1 <= irq);
    110
}

/// 每个具名输入只执行一个同生产算术边界；独立timekeeper不访问CPU或第二时钟后端。
fn arithmetic_case(frequency: u64, mode: u8) {
    if mode < 3 {
        let (mask, initial) = match mode {
            0 => (u64::MAX, u64::MAX - 40),
            1 => (0xffff, 0xfff0),
            _ => (u64::MAX, 1u64 << 63),
        };
        let keeper = clock::testing::Timekeeper::new(frequency, mask, initial);
        let scale = keeper.scale();
        let delta = 100u64;
        let raw = initial.wrapping_add(delta) & mask;
        let exact = (u128::from(delta) * u128::from(scale.mult)) >> scale.shift;
        assert_eq!(keeper.read(|| initial).as_nanos(), 0);
        assert_eq!(keeper.read(|| raw).as_nanos(), exact as u64);
        assert_eq!(keeper.update(|| raw).as_nanos(), exact as u64);
        assert_eq!(keeper.read(|| raw).as_nanos(), exact as u64);
        let ideal = u128::from(delta) * 1_000_000_000 / u128::from(frequency);
        assert!(exact.abs_diff(ideal) <= (u128::from(delta) >> scale.shift) + 1);
        return;
    }
    let keeper = clock::testing::Timekeeper::new(frequency, u64::MAX, 1u64 << 63);
    let scale = keeper.scale();
    if mode == 3 {
        let mut previous = 0u64;
        let mut fraction_seen = false;
        for step in 1..=4096u64 {
            let fixed = u128::from(step) * u128::from(scale.mult);
            let expected = (fixed >> scale.shift) as u64;
            let raw = (1u64 << 63) + step;
            let read = keeper.read(|| raw).as_nanos();
            assert_eq!(read, expected);
            assert!(previous <= read);
            assert_eq!(keeper.update(|| raw).as_nanos(), read);
            assert_eq!(keeper.read(|| raw).as_nanos(), read);
            let fraction = (fixed & ((1u128 << scale.shift) - 1)) as u64;
            assert_eq!(keeper.fraction(), fraction);
            fraction_seen |= fraction != 0;
            previous = read;
        }
        if frequency == 1_000_003 || frequency == 24_000_000 {
            assert!(fraction_seen);
        }
        return;
    }
    let fixed = u128::from(scale.max_delta) * u128::from(scale.mult);
    let (ns, fraction) = scale
        .convert(scale.max_delta, 0)
        .expect("max delta rejected");
    assert_eq!(u128::from(ns), fixed >> scale.shift);
    assert_eq!(u128::from(fraction), fixed & ((1u128 << scale.shift) - 1));
    let residue = (1u64 << scale.shift) - 1;
    let fixed = fixed + u128::from(residue);
    let (ns, fraction) = scale
        .convert(scale.max_delta, residue)
        .expect("max delta with residue rejected");
    assert_eq!(u128::from(ns), fixed >> scale.shift);
    assert_eq!(u128::from(fraction), fixed & u128::from(residue));
    assert!(scale
        .convert(
            scale.max_delta.checked_add(1).expect("max delta overflow"),
            0
        )
        .is_none());
}

/// 每个真实 reader 核对同一 production now，SMP 时与另一 CPU 的维护写者重叠。
fn time_reader() -> i32 {
    scenario().time_arrived.fetch_add(1, Ordering::Release);
    until(
        || scenario().time_go.load(Ordering::Acquire),
        "time-reader-start",
    );
    let mut previous = clock::now();
    for round in 1..=READ_ROUNDS {
        if scenario().stop.load(Ordering::Acquire) {
            return 111;
        }
        until(
            || {
                scenario().stop.load(Ordering::Acquire)
                    || round <= scenario().write_phase.load(Ordering::Acquire)
            },
            "time-update-published",
        );
        let now = clock::now();
        assert!(previous <= now);
        scenario().read_progress.fetch_add(1, Ordering::Release);
        previous = now;
        sched::yield_now();
    }
    scenario().readers_passed.fetch_add(1, Ordering::Release);
    111
}

/// 真维护写者只推进唯一 production timekeeper，不另外累加每 CPU 时间。
fn time_writer() -> i32 {
    scenario().time_arrived.fetch_add(1, Ordering::Release);
    until(
        || scenario().time_go.load(Ordering::Acquire),
        "time-writer-start",
    );
    let mut previous = clock::now();
    let readers = if cpu::count() == 1 { 1 } else { 2 };
    for round in 1..=READ_ROUNDS {
        if scenario().stop.load(Ordering::Acquire) {
            return 112;
        }
        let now = clock::testing::maintain();
        assert!(previous <= now);
        previous = now;
        scenario().write_phase.store(round, Ordering::Release);
        until(
            || {
                let now = clock::testing::maintain();
                assert!(previous <= now);
                previous = now;
                scenario().stop.load(Ordering::Acquire)
                    || round * readers <= scenario().read_progress.load(Ordering::Acquire)
            },
            "time-concurrent-readers",
        );
    }
    112
}

/// CPU0 真实读者暂停同一次 sample，由 CPU1 普通写者发布新一代并触发序号拒绝重试。
fn snapshot_overlap() -> (usize, usize) {
    if cpu::count() == 1 {
        return (0, 0);
    }
    assert_eq!(cpu::current().id, 0);
    let writer = crate::ktest::spawn_result_on(1, |_| {
        Ok({
            assert_eq!(cpu::current().id, 1);
            spin_until(clock::testing::reader_paused, "snapshot-reader-paused");
            clock::testing::maintain();
            115
        })
    })
    .expect("snapshot writer allocation failed");
    let result = {
        let _irq = IrqSaveGuard::new();
        let previous = clock::now();
        // SAFETY: 指定读者是本核 CPU0 普通任务且 IRQ 已关闭；已预备的 CPU1 写者
        // 可独立前进，读者不持写者所需锁，并在本区间完成 sample 和关闭握手。
        unsafe {
            clock::testing::arm_snapshot_overlap(
                task::current_task_id().expect("snapshot reader missing"),
                writer.task_id(),
            )
        };
        let now = clock::now();
        assert!(previous <= now);
        let result = clock::testing::snapshot_overlap_result();
        assert!(1 <= result.0 && 1 <= result.1);
        clock::testing::finish_snapshot_overlap();
        result
    };
    until(|| writer.try_join().is_some(), "snapshot-writer-return");
    assert_eq!(writer.join().unwrap(), 115);
    result
}

/// 先核算同实现边界，再用真实 reader/writer Task 验证跨 CPU 序列快照。
fn test_timekeeper() {
    let before = clock::testing::updates();
    let readers = if cpu::count() == 1 { 1 } else { 2 };
    let writer = crate::ktest::spawn_result_on(0, |_| Ok(time_writer()))
        .expect("time writer allocation failed");
    let first =
        crate::ktest::spawn_result_on(if cpu::count() == 1 { 0 } else { 1 }, |_| Ok(time_reader()))
            .expect("time reader allocation failed");
    let second = if readers == 2 {
        Some(
            crate::ktest::spawn_result_on(cpu::count() - 1, |_| Ok(time_reader()))
                .expect("second time reader allocation failed"),
        )
    } else {
        None
    };
    until(
        || scenario().time_arrived.load(Ordering::Acquire) == readers + 1,
        "time-task-arrival",
    );
    scenario().time_go.store(true, Ordering::Release);
    until(
        || {
            writer.try_join().is_some()
                && first.try_join().is_some()
                && second
                    .as_ref()
                    .is_none_or(|handle| handle.try_join().is_some())
        },
        "time-readers-return",
    );
    assert_eq!(writer.join().unwrap(), 112);
    assert_eq!(first.join().unwrap(), 111);
    if let Some(second) = second {
        assert_eq!(second.join().unwrap(), 111);
    }
    assert_eq!(scenario().readers_passed.load(Ordering::Acquire), readers);
    let updates = clock::testing::updates() - before;
    assert!(READ_ROUNDS as u64 <= updates);
}

/// 在同一 Task 的普通忙段、真 park 与迁核后分别结算累计执行时间。
fn accounting_worker() -> i32 {
    sched::test_migrate_to(0);
    let current = task::current_task();
    let id = current.id();
    let local = 0x7312usize;
    let stack = black_box(&local) as *const _ as usize;
    let old = scenario().account_task.lock().replace(Arc::clone(&current));
    assert!(old.is_none());
    let start = clock::now();
    let initial = accounting::current_execution_time();
    busy(Duration::from_millis(20));
    scenario().account_phase.store(1, Ordering::Release);
    if !scenario().stop.load(Ordering::Acquire) {
        scenario().account_wake.wait();
    }
    assert_eq!(
        current.id(),
        task::current_task_id().expect("account task missing")
    );
    let mut previous = accounting::current_execution_time();
    assert!(initial < previous);
    if 1 < cpu::count() {
        for target in [1usize, 0] {
            let hop_start = clock::now();
            let hop_execution = accounting::current_execution_time();
            sched::test_migrate_to(target);
            assert_eq!(cpu::current().id, target);
            assert_eq!(arch::cpu_local(), cpu::get(target) as *const _);
            assert_eq!(task::current_task_id(), Some(id));
            assert_eq!(black_box(&local) as *const _ as usize, stack);
            assert_eq!(local, 0x7312);
            busy(Duration::from_millis(2));
            let execution = accounting::current_execution_time();
            let hop_end = clock::now();
            assert!(
                execution
                    .checked_sub(hop_execution)
                    .expect("migration accounting regressed")
                    <= hop_end
                        .checked_duration_since(hop_start)
                        .expect("migration wall regressed")
            );
            assert!(previous < execution);
            previous = execution;
            scenario().migrations.fetch_add(1, Ordering::Relaxed);
        }
    }
    let execution = accounting::current_execution_time()
        .checked_sub(initial)
        .expect("task accounting regressed");
    let end = clock::now();
    let wall = end
        .checked_duration_since(start)
        .expect("wall time regressed");
    assert!(Duration::ZERO < execution && execution < wall);
    scenario().task_ns.store(
        u64::try_from(execution.as_nanos()).expect("task time overflow"),
        Ordering::Release,
    );
    scenario().wall_ns.store(
        u64::try_from(wall.as_nanos()).expect("wall time overflow"),
        Ordering::Release,
    );
    113
}

/// 返回已经发布的计账 Task，不保留测试锁跨观察或通知。
fn account_task() -> Arc<Task> {
    Arc::clone(
        scenario()
            .account_task
            .lock()
            .as_ref()
            .expect("account task not published"),
    )
}

/// 绑定 Task 真等待后由远端 ready 提醒执行，每轮核对目标 timer IRQ 未提供进度。
fn kick_worker(target: usize) -> i32 {
    let slot = &scenario().kicks[target];
    let mut publication = slot.task.lock();
    if slot.stop.load(Ordering::Acquire) {
        return 114;
    }
    let old = publication.replace(task::current_task());
    assert!(old.is_none());
    drop(publication);
    loop {
        slot.wake.wait();
        if slot.stop.load(Ordering::Acquire) {
            break;
        }
        assert_eq!(cpu::current().id, target);
        assert_eq!(task::current_task_id(), Some(slot.task().id()));
        assert_eq!(
            cpu::current().timer_interrupts.load(Ordering::Acquire),
            slot.timer_before.load(Ordering::Acquire)
        );
        slot.rounds.fetch_add(1, Ordering::Release);
        if slot.pause_after_round.load(Ordering::Acquire) {
            slot.round_paused.store(true, Ordering::Release);
            spin_until(
                || !slot.pause_after_round.load(Ordering::Acquire),
                "idle round checkpoint",
            );
            slot.round_paused.store(false, Ordering::Release);
        }
    }
    114
}

/// 入口计数只证明 idle 准备，真实 ISA 等待必须在本轮唤醒后核对完成数。
fn remote_idle(target: usize, after: usize, end: u64) {
    let slot = &scenario().kicks[target];
    spin_until_before(
        || {
            let published = slot.task.lock().is_some();
            published
                && sched::test_state(&slot.task()) == State::Blocked
                && cpu::get(target).idle.load(Ordering::Acquire)
                && after < cpu::test_idle_waits(target)
        },
        end,
        "remote-task-idle",
    );
}

/// CSD 回调只在真实目标硬 IRQ 写入动作计数，完成依旧由原 release 协议返回。
///
/// # Safety
/// 参数必须为空；调用者必须通过同步 cpu::call 等待动作完成。
unsafe fn csd_action(argument: *mut ()) {
    assert!(argument.is_null());
    assert!(0 < cpu::current().hardirq_depth.load(Ordering::Relaxed));
    scenario().csd_actions.fetch_add(1, Ordering::Release);
}

/// 提交一轮真实 Completion ready 提醒，唤醒后返回本轮实际硬等待完成增量。
fn kick_attempt(target: usize, attempt: usize, after: usize, end: u64) -> (usize, usize) {
    remote_idle(target, after, end);
    let local = cpu::get(target);
    let before = local.timer_interrupts.load(Ordering::Acquire);
    spin_until_before(
        || !local.kick_pending() && !local.need_resched.load(Ordering::Acquire),
        end,
        "idle-pending-drained",
    );
    busy(Duration::from_millis(1));
    assert!(local.idle.load(Ordering::Acquire));
    assert_eq!(local.timer_interrupts.load(Ordering::Acquire), before);
    let idle = cpu::test_idle_waits(target);
    if attempt == 2 {
        cpu::test_arm_pause(target, cpu::TestPause::Idle);
        cpu::kick(target, cpu::KICK_RESCHEDULE);
        spin_until_before(
            || cpu::test_is_paused(target, cpu::TestPause::Idle),
            end,
            "pending-before-idle",
        );
    }
    if attempt == 3 {
        let reprograms = cpu::test_timer_reprograms(target);
        cpu::test_arm_pause(target, cpu::TestPause::Ipi);
        cpu::kick(target, cpu::KICK_RESCHEDULE);
        spin_until_before(
            || cpu::test_is_paused(target, cpu::TestPause::Ipi),
            end,
            "ipi-processing-pause",
        );
        cpu::kick(target, cpu::KICK_TIMER_REPROGRAM);
        assert_eq!(
            cpu::test_pending_kicks(target) & cpu::KICK_TIMER_REPROGRAM,
            cpu::KICK_TIMER_REPROGRAM
        );
        cpu::test_release_pause(target, cpu::TestPause::Ipi);
        spin_until_before(
            || reprograms < cpu::test_timer_reprograms(target),
            end,
            "ipi-new-reason",
        );
        spin_until_before(
            || local.idle.load(Ordering::Acquire),
            end,
            "ipi-return-idle",
        );
    }
    let waits_before = cpu::test_wait_completions(target);
    scenario().kicks[target]
        .timer_before
        .store(before, Ordering::Release);
    scenario().kicks[target].wake.complete();
    if attempt == 2 {
        cpu::test_release_pause(target, cpu::TestPause::Idle);
    }
    spin_until_before(
        || scenario().kicks[target].rounds.load(Ordering::Acquire) == attempt,
        end,
        "remote-ready-kick",
    );
    assert_eq!(local.timer_interrupts.load(Ordering::Acquire), before);
    let waits = cpu::test_wait_completions(target)
        .checked_sub(waits_before)
        .expect("idle completion count regressed");
    (idle, waits)
}

/// 先备所有绑定执行者，每个远端在有限截止内核验四轮真硬等待，再停止并 join。
fn test_idle_kicks() -> (usize, usize) {
    if cpu::count() == 1 {
        return (0, 0);
    }
    assert_eq!(cpu::current().id, 0);
    let before_idle: [usize; 4] = core::array::from_fn(|id| {
        if id < cpu::count() {
            cpu::test_idle_waits(id)
        } else {
            0
        }
    });
    let mut handles: [Option<crate::ktest::ChildHandle>; 4] = core::array::from_fn(|id| {
        if 0 < id && id < cpu::count() {
            Some(
                crate::ktest::spawn_result_on(id, move |_| Ok(kick_worker(id)))
                    .expect("idle kick task allocation failed"),
            )
        } else {
            None
        }
    });
    let mut rounds = 0usize;
    let mut waits = 0usize;
    for target in 1..cpu::count() {
        let end = raw_deadline();
        let mut idle = before_idle[target];
        remote_idle(target, idle, end);
        let timer_before = cpu::get(target).timer_interrupts.load(Ordering::Acquire);
        let mut attempt = 0usize;
        let mut verified = 0usize;
        let mut completed_waits = 0usize;
        while verified < KICK_ROUNDS || attempt < 3 {
            assert!(
                arch::read_counter() < end,
                "hard idle wait timeout: cpu={}",
                target
            );
            attempt = attempt.checked_add(1).expect("idle attempt count overflow");
            let (entered, completed) = kick_attempt(target, attempt, idle, end);
            idle = entered;
            completed_waits += completed;
            // 两个停点轮次验证通知协议；其中新增原因可能先唤醒 ISA，不计入 ready 硬等待。
            if 0 < completed && attempt != 2 && attempt != 3 && verified < KICK_ROUNDS {
                verified += 1;
            }
        }
        assert_eq!(verified, KICK_ROUNDS);
        assert!(KICK_ROUNDS <= completed_waits);
        assert_eq!(
            cpu::get(target).timer_interrupts.load(Ordering::Acquire),
            timer_before
        );
        rounds += verified;
        waits += completed_waits;
        scenario().kicks[target].request_stop();
        let handle = handles[target].take().expect("idle kick handle missing");
        until(|| handle.try_join().is_some(), "kick-task-return");
        assert_eq!(handle.join().unwrap(), 114);
        let actions = scenario().csd_actions.load(Ordering::Acquire);
        // SAFETY: 动作不解引用参数且不睡眠；当前 CPU0 Task IRQ 可服务且不持锁，
        // 原同步 CSD 协议等待远端动作的 release 完成后才返回。
        unsafe { cpu::call(target, csd_action, core::ptr::null_mut()) };
        assert_eq!(scenario().csd_actions.load(Ordering::Acquire), actions + 1);
        let observer = scenario().kicks[target].task.lock().take();
        drop(observer);
    }
    (rounds, waits)
}

/// 用真实 CPU 四账和同 Task 的受控 offCPU 区间验证执行时间排除阻塞和迁核间隙。
#[expect(
    clippy::needless_range_loop,
    reason = "逐个真实 CPU 核对固定快照，核数超出快照时保持原索引失败，不用迭代截断掩盖覆盖不足"
)]
fn test_accounting() {
    let before: [accounting::CpuSnapshot; 4] =
        core::array::from_fn(|id| accounting::snapshot(if id < cpu::count() { id } else { 0 }));
    let handle = crate::ktest::spawn_result(|_| Ok(accounting_worker()))
        .expect("account task allocation failed");
    until(
        || {
            scenario().account_phase.load(Ordering::Acquire) == 1
                && sched::test_state(&account_task()) == State::Blocked
        },
        "account-task-parked",
    );
    let observer = account_task();
    let execution = observer.execution_time();
    let blocked_start = clock::now();
    busy(Duration::from_millis(15));
    assert_eq!(observer.execution_time(), execution);
    let offcpu_ns = clock::now()
        .checked_duration_since(blocked_start)
        .expect("offCPU time regressed")
        .as_nanos() as u64;
    assert!(0 < offcpu_ns);
    scenario().account_wake.complete();
    until(|| handle.try_join().is_some(), "account-task-return");
    assert_eq!(handle.join().unwrap(), 113);
    let task_ns = scenario().task_ns.load(Ordering::Acquire);
    let wall_ns = scenario().wall_ns.load(Ordering::Acquire);
    assert!(0 < task_ns && task_ns < wall_ns);
    assert!(task_ns.checked_add(offcpu_ns).expect("interval overflow") <= wall_ns);
    let mut irq_ns = 0u64;
    let mut scheduler_ns = 0u64;
    let mut idle_ns = 0u64;
    for id in 0..cpu::count() {
        let after = accounting::snapshot(id);
        irq_ns += after
            .irq_ns
            .checked_sub(before[id].irq_ns)
            .expect("IRQ accounting regressed");
        scheduler_ns += after
            .scheduler_ns
            .checked_sub(before[id].scheduler_ns)
            .expect("scheduler accounting regressed");
        idle_ns += after
            .idle_ns
            .checked_sub(before[id].idle_ns)
            .expect("idle accounting regressed");
    }
    assert!(0 < irq_ns && 0 < scheduler_ns);
    let migrations = scenario().migrations.load(Ordering::Acquire);
    if cpu::count() == 1 {
        assert_eq!(migrations, 0);
    } else {
        assert!(2 <= migrations && 0 < idle_ns);
    }
    let published = scenario().account_task.lock().take();
    drop((published, observer));
}

struct Scenario {
    nodes: super::Nodes,
    stop: AtomicBool,
    peer_ran: AtomicBool,
    time_go: AtomicBool,
    time_arrived: AtomicUsize,
    write_phase: AtomicUsize,
    read_progress: AtomicUsize,
    readers_passed: AtomicUsize,
    account_task: SpinLock<Option<Arc<Task>>>,
    account_phase: AtomicUsize,
    account_wake: Completion,
    task_ns: AtomicU64,
    wall_ns: AtomicU64,
    migrations: AtomicUsize,
    csd_actions: AtomicUsize,
    kicks: [KickSlot; 4],
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
        sched::test_migrate_to(0);
        let initial = super::baseline();
        let state = Arc::try_new(Scenario {
            nodes: super::Nodes::new(0, 0, 0, 0)?,
            stop: AtomicBool::new(false),
            peer_ran: AtomicBool::new(false),
            time_go: AtomicBool::new(false),
            time_arrived: AtomicUsize::new(0),
            write_phase: AtomicUsize::new(0),
            read_progress: AtomicUsize::new(0),
            readers_passed: AtomicUsize::new(0),
            account_task: SpinLock::new(None),
            account_phase: AtomicUsize::new(0),
            account_wake: Completion::new(),
            task_ns: AtomicU64::new(0),
            wall_ns: AtomicU64::new(0),
            migrations: AtomicUsize::new(0),
            csd_actions: AtomicUsize::new(0),
            kicks: [const { KickSlot::new() }; 4],
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
        let observed = owner.state.account_task.lock().take();
        drop(observed);
        for slot in &owner.state.kicks {
            let observed = slot.task.lock().take();
            drop(observed);
        }
        if owner.state.account_wake.test_waiting() != 0 || owner.state.account_wake.is_ready() {
            return Err(::ktest::TestError("accounting source did not converge"));
        }
        for slot in &owner.state.kicks {
            if slot.wake.test_waiting() != 0 || slot.wake.is_ready() {
                return Err(::ktest::TestError("idle kick source did not converge"));
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
    /// fixed协调动作已经退栈才调用；只一次解除真实停点和业务gate，保留所有原异步owner。
    fn stop_sources(&self) {
        if self.stop.swap(true, Ordering::AcqRel) {
            return;
        }
        self.time_go.store(true, Ordering::Release);
        let account = self.account_task.lock().as_ref().map(Arc::clone);
        if account
            .as_ref()
            .is_some_and(|task| sched::test_state(task) != State::Exited)
        {
            self.account_wake.complete();
        }
        for slot in &self.kicks {
            slot.pause_after_round.store(false, Ordering::Release);
            slot.request_stop();
        }
        for id in 0..cpu::count() {
            clock::testing::cancel_program_pause(id);
            cpu::test_cancel_pause(id, cpu::TestPause::Idle);
            cpu::test_cancel_pause(id, cpu::TestPause::Ipi);
        }
        // SAFETY: 唯一reader在fixed协调动作内；该动作返回或展开完成后才执行本函数，原sample已退出。
        unsafe {
            clock::testing::cancel_snapshot_overlap();
        }
    }
}
impl ServiceFixture {
    /// 固定协调者独立拥有同一Scenario，子入口不捕获fixture或TestContext借用。
    fn fixed(
        &self,
        action: impl FnOnce() -> ::ktest::TestResult + Send + 'static,
    ) -> ::ktest::TestResult {
        super::on_zero(self.state.clone(), Scenario::stop_sources, action)
    }
}

#[::ktest::suite(fixture = ServiceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "time"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// 可选RTC授时失败只留下未校时起点，有效的Unix零点与缺席样本保持可区分。
    #[test]
    #[case("valid_epoch", Ok(Some([1970, 1, 1, 0, 0, 0])), 0u64, false)]
    #[case("valid_leap_day", Ok(Some([2000, 2, 29, 0, 0, 0])), 951_782_400_000_000_000u64, false)]
    #[case("unavailable", Ok(None), 0u64, true)]
    #[case("invalid_calendar", Ok(Some([2000, 0, 0, 0, 0, 0])), 0u64, true)]
    #[case("invalid_leap_day", Ok(Some([2100, 2, 29, 0, 0, 0])), 0u64, true)]
    #[case("read_failure", Err("RTC calendar did not stabilize"), 0u64, true)]
    fn rtc_boot_base(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        sample: Result<Option<[u32; 6]>, &'static str>,
        expected: u64,
        unset: bool,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || {
            let (actual, unavailable) = clock::testing::rtc_realtime_base(sample);
            assert_eq!(actual, expected);
            assert_eq!(unavailable.is_some(), unset);
            Ok(())
        })
    }

    /// 生产日历换算覆盖 Unix 起点、闰年、世纪、有效时分秒及纳秒容量边界。
    #[test]
    fn rtc_calendar_boundaries(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            let fields = |month: u32, day: u32, hour: u32, minute: u32, second: u32| {
                (month << 26) | (day << 21) | (hour << 16) | (minute << 10) | (second << 4)
            };
            assert_eq!(
                clock::testing::rtc_calendar(70, fields(1, 1, 0, 0, 0)),
                Some(0)
            );
            assert_eq!(
                clock::testing::rtc_calendar(100, fields(2, 29, 0, 0, 0)),
                Some(951_782_400_000_000_000)
            );
            assert_eq!(
                clock::testing::rtc_calendar(138, fields(1, 19, 3, 14, 7)),
                Some(2_147_483_647_000_000_000)
            );
            assert_eq!(
                clock::testing::rtc_calendar(200, fields(2, 29, 0, 0, 0)),
                None
            );
            assert_eq!(
                clock::testing::rtc_calendar(123, fields(2, 29, 0, 0, 0)),
                None
            );
            assert_eq!(clock::testing::rtc_calendar(0, fields(1, 1, 0, 0, 0)), None);
            assert_eq!(
                clock::testing::rtc_calendar(8099, fields(1, 1, 0, 0, 0)),
                None
            );
            assert_eq!(
                clock::testing::rtc_calendar(u32::MAX, fields(1, 1, 0, 0, 0)),
                None
            );
            for low in [
                fields(0, 1, 0, 0, 0),
                fields(13, 1, 0, 0, 0),
                fields(1, 0, 0, 0, 0),
                fields(4, 31, 0, 0, 0),
                fields(1, 1, 24, 0, 0),
                fields(1, 1, 0, 60, 0),
                fields(1, 1, 0, 0, 60),
            ] {
                assert_eq!(clock::testing::rtc_calendar(126, low), None);
            }
            Ok(())
        })
    }

    /// RTC 日历投影保持同一单调采样区间，过去 UTC 立即返回且未来睡眠沿原精确 timer。
    #[test]
    fn realtime_and_monotonic_deadlines(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            let before = clock::now();
            let utc = clock::realtime();
            let after = clock::now();
            let projected = clock::realtime_deadline(utc);
            assert!(before <= projected && projected <= after);
            assert!(after.as_nanos() < utc, "RTC epoch is absent from realtime");
            assert_eq!(clock::realtime_deadline(0), MonotonicInstant::from_nanos(0));
            crate::timer::sleep_until(clock::realtime_deadline(0));
            let utc_deadline = clock::realtime().checked_add(20_000_000).unwrap();
            let deadline = clock::realtime_deadline(utc_deadline);
            crate::timer::sleep_until(deadline);
            assert!(deadline <= clock::now());
            assert!(utc_deadline <= clock::realtime());
            Ok(())
        })
    }
    /// 重新武装真实quantum后验证硬IRQ仅置请求、合法安全点切换与整数寄存器保存。
    #[test]
    fn deferred_quantum(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            let child = crate::ktest::spawn_result_on(0, |_| Ok(deferred_worker())).unwrap();
            assert_eq!(child.join().unwrap(), 110);
            Ok(())
        })
    }
    /// 四频率的回绕、长counter、分段余数和最大差值各自成为具名参数，并各自准备fixture。
    #[test]
    #[case("1000003_full_wrap", 1_000_003, 0)]
    #[case("1000003_mask_wrap", 1_000_003, 1)]
    #[case("1000003_long", 1_000_003, 2)]
    #[case("1000003_fraction", 1_000_003, 3)]
    #[case("1000003_max_delta", 1_000_003, 4)]
    #[case("10mhz_full_wrap", 10_000_000, 0)]
    #[case("10mhz_mask_wrap", 10_000_000, 1)]
    #[case("10mhz_long", 10_000_000, 2)]
    #[case("10mhz_fraction", 10_000_000, 3)]
    #[case("10mhz_max_delta", 10_000_000, 4)]
    #[case("24mhz_full_wrap", 24_000_000, 0)]
    #[case("24mhz_mask_wrap", 24_000_000, 1)]
    #[case("24mhz_long", 24_000_000, 2)]
    #[case("24mhz_fraction", 24_000_000, 3)]
    #[case("24mhz_max_delta", 24_000_000, 4)]
    #[case("1ghz_full_wrap", 1_000_000_000, 0)]
    #[case("1ghz_mask_wrap", 1_000_000_000, 1)]
    #[case("1ghz_long", 1_000_000_000, 2)]
    #[case("1ghz_fraction", 1_000_000_000, 3)]
    #[case("1ghz_max_delta", 1_000_000_000, 4)]
    fn fixed_point_boundaries(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        frequency: u64,
        mode: u8,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || {
            arithmetic_case(frequency, mode);
            Ok(())
        })
    }
    /// 指定远核writer推进正在采样的同次generation，reader必须真实拒绝旧序号。
    #[test]
    #[ktest(min_cpus = 2)]
    fn same_snapshot_retry(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            snapshot_overlap();
            Ok(())
        })
    }
    /// 真reader/writer Task竞争唯一timekeeper并逐轮核对单调性与维护次数。
    #[test]
    fn concurrent_monotonic(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_timekeeper();
            Ok(())
        })
    }
    /// 同Task忙段、真正Blocked与迁核保持栈和身份，同时区分Task/IRQ/scheduler账。
    #[test]
    fn execution_accounting(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_accounting();
            Ok(())
        })
    }
    /// 每个远核真实ISA等待完成四轮，timer IRQ基线不增长，IPI新原因不得丢失。
    #[test]
    #[ktest(min_cpus = 2)]
    fn idle_return_without_tick(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            let (rounds, waits) = test_idle_kicks();
            assert_eq!(rounds, 4 * (cpu::count() - 1));
            assert!(rounds <= waits);
            Ok(())
        })
    }
    /// 末轮执行者仍在原栈时发停止票；控制者失败后也必须真join且耗尽完成源。
    #[test]
    #[ktest(
        min_cpus = 2,
        ignore = "controlled idle coordinator failure must remain Fail after complete cleanup"
    )]
    fn idle_stop_during_round(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            let state = scenario();
            let slot = &state.kicks[1];
            slot.pause_after_round.store(true, Ordering::Release);
            let worker = crate::ktest::spawn_result_on(1, |_| Ok(kick_worker(1)))
                .expect("idle stop worker allocation failed");
            until(
                || {
                    let observed = slot.task.lock().as_ref().map(Arc::clone);
                    observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
                },
                "idle stop worker prepared",
            );
            slot.timer_before.store(
                cpu::get(1).timer_interrupts.load(Ordering::Acquire),
                Ordering::Release,
            );
            slot.wake.complete();
            spin_until(
                || slot.round_paused.load(Ordering::Acquire),
                "idle stop round paused",
            );
            slot.request_stop();
            slot.pause_after_round.store(false, Ordering::Release);
            until(|| worker.try_join().is_some(), "idle stop worker returned");
            assert_eq!(worker.join().unwrap(), 114);
            Err(::ktest::TestError("controlled idle coordinator failure"))
        })
    }
}
