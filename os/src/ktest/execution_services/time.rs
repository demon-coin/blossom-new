//! 时间子系统、单调时钟与执行时间记账（Execution Accounting）内核测试套件。
//!
//! # 模块职责与架构定位
//! 本模块属于 `ktest::execution_services` 测试层，负责对内核的时间基础设施进行集成验证。
//! 上游由 `ktest` 框架调度运行，下游直接调用 `crate::time`（单调时钟与 RTC 日历换算）、
//! `crate::accounting`（CPU 四分类时间记账与任务执行时间统计）、`crate::sched`（调度器抢占与安全点）
//! 以及 `crate::cpu`（跨核 IPI 唤醒、同步调用与底层 WFI/IDLE 等待）。
//!
//! # 核心验证机制与并发交互模型
//! 1. **RTC 日历与实时时钟投影**：验证 RTC 硬件采样在合法闰年、世纪边界与无效输入下的纳秒时间戳换算，
//!    以及 UTC 实时截止时间向单调时钟（`MonotonicInstant`）的区间映射。
//! 2. **定点数频率换算与顺序锁（SeqLock）并发快照**：
//!    - 验证不同硬件计数器频率下的 `(mult, shift)` 定点算术、掩码回绕（Wrap-around）与余数（Fraction）累积。
//!    - 通过跨核读写并发与受控快照暂停（Snapshot Overlap），验证顺序锁在写者推进世代号（Generation）时，
//!      读者能够检测到序号变化并重试采样。
//! 3. **时间片硬中断延迟抢占与上下文记账**：
//!    - 验证时间片到期硬中断仅设置 `need_resched` 标志，直到任务主动到达调度安全点（`cond_resched`）才发生上下文切换。
//!    - 验证任务在运行（On-CPU）、阻塞睡眠（Blocked）及跨核迁移（Migration）过程中的执行时间记账守恒。
//! 4. **跨核 Idle 唤醒与 IPI 合并**：
//!
//! ```text
//!   CPU 0 (Coordinator)                        CPU N (Target Worker / Idle)
//!  ─────────────────────                      ──────────────────────────────
//!   等待目标核进入 Idle 等待 ───────────────►  kick_worker 阻塞于 wake.wait()
//!   记录 timer_interrupts 基线                 进入底层 WFI / IDLE 指令等待
//!   (可选) 注入 Idle / IPI 暂停点                   │
//!   触发 wake.complete() ──[IPI_RESCHEDULE]──► 硬件唤醒并退出 Idle（无新时钟中断）
//!   等待 rounds 计数递增 ◄───────────────────  校验 timer_interrupts 未增长并递增 rounds
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试状态中的自旋锁（`Scenario::account_task`、`KickSlot::task`）仅用于发布或克隆 `Arc<Task>` 句柄，
//!   严禁在持有这些自旋锁的临界区内调用 `sched::test_state`、`Completion::complete` 或任何可能阻塞的操作。
//! - 调用 `rearm_current_quantum` 或 `arm_snapshot_overlap` 时，必须在当前 CPU 关中断（`IrqSaveGuard`）保护下执行。

use super::wait;
use alloc::sync::Arc;
use core::convert::TryFrom;
use core::hint::{black_box, spin_loop};
use core::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};

/// 轮询等待 `predicate` 条件成立；若超时则触发 panic 由测试框架捕获，后续清理交由 `ServiceFixture` 执行。
fn until(predicate: impl FnMut() -> bool, name: &'static str) {
    wait(predicate, name).unwrap();
}

use crate::sync::{Completion, IrqSaveGuard, SpinLock};
use crate::task::{self, State, Task};
use crate::time::{self as clock, Duration, MonotonicInstant};
use crate::{accounting, arch, cpu, sched};

/// 单调时钟读写并发测试的总轮数。
const READ_ROUNDS: usize = 128;
/// 跨核 Idle 唤醒测试在每个目标 CPU 上验证的有效唤醒轮数。
const KICK_ROUNDS: usize = 4;

/// 单个目标 CPU 的跨核 Idle 唤醒测试槽位，用于协调工作任务阻塞、唤醒与轮次统计。
struct KickSlot {
    /// 绑定在目标 CPU 上运行的工作任务句柄（由自旋锁保护，仅在注册和克隆句柄时短暂持锁）。
    task: SpinLock<Option<Arc<Task>>>,
    /// 用于唤醒阻塞工作任务的完成量（Completion）。
    wake: Completion,
    /// 工作任务成功从 Idle 唤醒并完成校验的累计轮数。
    rounds: AtomicUsize,
    /// 本轮唤醒前目标 CPU 的时钟中断计数基线，用于验证唤醒不依赖时钟中断。
    timer_before: AtomicUsize,
    /// 通知工作任务退出主循环的停止标志。
    stop: AtomicBool,
    /// 是否要求工作任务在完成当前轮次后进入自旋暂停点（用于测试运行中停止协议）。
    pause_after_round: AtomicBool,
    /// 工作任务是否已到达轮次末尾的自旋暂停点。
    round_paused: AtomicBool,
}

impl KickSlot {
    /// 创建初始为空闲状态的跨核唤醒测试槽位。
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

    /// 克隆并返回已注册的工作任务 `Arc<Task>` 句柄，立即释放内部自旋锁以避免持锁查询调度状态。
    fn task(&self) -> Arc<Task> {
        Arc::clone(self.task.lock().as_ref().expect("kick task not published"))
    }

    /// 请求工作任务停止运行：首次调用时设置 `stop` 标志，若任务尚未退出则触发 `wake` 将其从阻塞中唤醒。
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

/// 在默认硬件超时（5 秒）内自旋等待 `predicate` 条件成立，适用于关中断或不可调度的观察窗口。
fn spin_until(predicate: impl FnMut() -> bool, name: &str) {
    spin_until_before(predicate, raw_deadline(), name);
}

/// 基于当前单调时钟计算延迟 `duration` 后的绝对截止时间点。
fn deadline(duration: Duration) -> MonotonicInstant {
    clock::now()
        .checked_add(duration)
        .expect("time test deadline overflow")
}

/// 基于底层硬件计数器计算当前时刻起 5 秒后的原始计数值截止上限，避免等待期间依赖可能被测试暂停的软件时钟。
fn raw_deadline() -> u64 {
    arch::read_counter()
        .checked_add(
            arch::counter_frequency()
                .checked_mul(5)
                .expect("raw timeout interval overflow"),
        )
        .expect("raw timeout deadline overflow")
}

/// 在指定的硬件计数器截止上限 `end` 之前自旋等待 `predicate` 成立，超时则触发断言失败。
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

/// 在普通任务保持运行状态期间持续忙等指定时长 `duration`，期间允许响应硬件中断但不主动调用调度安全点。
fn busy(duration: Duration) {
    let end = deadline(duration);
    while clock::now() < end {
        black_box(arch::read_counter());
        spin_loop();
    }
}

/// 验证时间片到期硬中断仅设置重调度标志，直到显式调用安全点 `cond_resched` 才切换至同核就绪任务，并验证中断上下文整数寄存器保存。
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
    // Step 1: 关中断并重装当前任务时间片，记录时钟中断计数基线并清除重调度标志。
    let before = {
        let _irq = IrqSaveGuard::new();
        // SAFETY:
        // 1. 当前任务已绑定在 CPU 0 上运行，且通过 `_irq` 持有了本核关中断守卫（`IrqSaveGuard`）；
        // 2. 当前上下文未持有任何调度器锁或时钟事件锁，在关中断区间内重装本核时钟事件不会发生并发竞争。
        unsafe { clock::testing::rearm_current_quantum() };
        let before = local.timer_interrupts.load(Ordering::Acquire);
        local.need_resched.store(false, Ordering::Release);
        before
    };
    // Step 2: 自旋等待至少一次时钟硬中断发生，验证硬中断仅置位 `need_resched` 而未直接切换到 `peer` 任务。
    spin_until(
        || before < local.timer_interrupts.load(Ordering::Acquire),
        "task-quantum-irq",
    );
    assert!(!scenario().peer_ran.load(Ordering::Acquire));
    assert!(local.need_resched.load(Ordering::Acquire));
    assert_eq!(task::current_task_id(), Some(task::current_task().id()));
    // Step 3: 主动进入调度安全点 `cond_resched`，验证 `peer` 任务获得运行并顺利退出。
    let safe_points = usize::from(sched::cond_resched());
    assert_eq!(safe_points, 1);
    assert!(scenario().peer_ran.load(Ordering::Acquire));
    until(|| peer.try_join().is_some(), "reschedule-peer-return");
    assert_eq!(peer.join().unwrap(), 101);

    // Step 4: 验证时钟中断抢占路径下通用整数寄存器的保存与恢复完整性。
    // SAFETY:
    // 1. 当前为绑定在 CPU 0 上的普通线程上下文，未持有任何自旋锁；
    // 2. 架构异常入口、每 CPU 永久结构（`CpuLocal`）及硬件定时器均已就绪，测试内联汇编在退出前会完整恢复中断状态与当前时钟事件。
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

/// 验证定点数时间维护器（`Timekeeper`）在指定频率 `frequency` 与测试模式 `mode` 下的换算精度、掩码回绕、余数累积与最大跨度边界。
fn arithmetic_case(frequency: u64, mode: u8) {
    // Mode 0..=2: 验证 64 位全掩码回绕（Mode 0）、16 位窄掩码回绕（Mode 1）与高半区长计数值（Mode 2）下的单步换算误差。
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
    // Mode 3: 连续单步推进 4096 次，验证定点数亚纳秒余数（Fraction）的逐轮累积与单调不减性。
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

    // Mode 4: 验证最大允许计数器差值 `max_delta` 在零余数、最大余数下的精确换算，以及超出 `max_delta + 1` 时的溢出拒绝。
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

/// 单调时钟并发读者任务：按轮次等待写者推进世代，并在每轮读取全局单调时钟以验证时间戳的单调不减性。
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

/// 单调时钟并发写者任务：循环调用 `clock::testing::maintain()` 推进全局时间维护器，并与并发读者按轮次同步。
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

/// 在 CPU 0 上暂停读者的单次顺序锁采样，同时由 CPU 1 上的写者发布新世代，验证读者检测到序号变化并完成重试。
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
    // 在 CPU 0 关中断区间内启动快照重叠测试，使当前读者采样在中途暂停并等待 CPU 1 写者更新世代。
    let result = {
        let _irq = IrqSaveGuard::new();
        let previous = clock::now();
        // SAFETY:
        // 1. 当前任务绑定在 CPU 0 上且已通过 `_irq` 关闭本地中断，未持有写者所需的维护锁；
        // 2. CPU 1 上的写者任务已创建并可独立推进世代，当前读者会在同一个关中断区间内完成重试采样并调用 `finish_snapshot_overlap` 清理测试状态。
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

/// 启动并发写者与一至两个读者任务，验证跨 CPU 顺序锁快照的并发单调性与时间维护更新次数。
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

/// 执行时间记账工作任务：在 CPU 0 忙等运行、阻塞睡眠及跨 CPU 迁移三个阶段分别采样任务累计执行时间与栈状态。
fn accounting_worker() -> i32 {
    sched::test_migrate_to(0);
    let current = task::current_task();
    let id = current.id();
    let local = 0x7312usize;
    let stack = black_box(&local) as *const _ as usize;
    let old = scenario().account_task.lock().replace(Arc::clone(&current));
    assert!(old.is_none());

    // Step 1: 在 CPU 0 上忙等 20ms 后进入完成量阻塞等待，供主协调任务验证阻塞期间任务执行时间不增长。
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

    // Step 2: 在多核环境下依次迁移至 CPU 1 再回到 CPU 0，验证跨核迁移后栈帧与任务身份保持不变，且单段执行时间增量不超过墙上时钟跨度。
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

    // Step 3: 汇总任务总执行时间与墙上时钟总跨度，供主协调任务校验守恒关系。
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

/// 克隆并返回已注册的执行记账测试任务 `Arc<Task>` 句柄，立即释放内部自旋锁以避免持锁等待或查询状态。
fn account_task() -> Arc<Task> {
    Arc::clone(
        scenario()
            .account_task
            .lock()
            .as_ref()
            .expect("account task not published"),
    )
}

/// 绑定在目标 CPU `target` 上的跨核唤醒工作任务：循环在完成量上阻塞以使目标核进入 Idle，被远端唤醒后校验本核时钟中断计数未增加。
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

/// 自旋等待目标 CPU `target` 上的工作任务进入阻塞状态，且目标 CPU 已进入底层硬件 Idle 等待（`test_idle_waits` 大于 `after`）。
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

/// 跨核同步调用（CSD）测试回调：在目标 CPU 硬中断上下文中递增动作计数。
///
/// # Safety
/// 1. `argument` 必须为空指针（`null_mut`）；
/// 2. 必须由 `cpu::call` 在目标 CPU 硬中断上下文中调用，且调用方同步等待该回调执行完毕后才返回。
unsafe fn csd_action(argument: *mut ()) {
    assert!(argument.is_null());
    assert!(0 < cpu::current().hardirq_depth.load(Ordering::Relaxed));
    scenario().csd_actions.fetch_add(1, Ordering::Release);
}

/// 对目标 CPU `target` 执行单轮跨核唤醒测试，涵盖常规 Idle 唤醒、进入 Idle 前抢占请求拦截（第 2 轮）与 IPI 处理中途新原因注入（第 3 轮）。
fn kick_attempt(target: usize, attempt: usize, after: usize, end: u64) -> (usize, usize) {
    // Step 1: 等待目标 CPU 进入稳定的硬件 Idle 状态，并记录时钟中断计数基线。
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
    // Step 2 (第 2 轮): 在目标 CPU 进入底层 Idle 指令前注入暂停点并发送重调度 IPI，验证进入等待前的待处理请求检查。
    if attempt == 2 {
        cpu::test_arm_pause(target, cpu::TestPause::Idle);
        cpu::kick(target, cpu::KICK_RESCHEDULE);
        spin_until_before(
            || cpu::test_is_paused(target, cpu::TestPause::Idle),
            end,
            "pending-before-idle",
        );
    }
    // Step 3 (第 3 轮): 在目标 CPU 处理 IPI 期间暂停，追加注入 `KICK_TIMER_REPROGRAM` 请求，验证新 IPI 原因不会丢失。
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

    // Step 4: 触发完成量唤醒目标 CPU 上的工作任务，验证其在无新增时钟中断的情况下完成本轮唤醒并返回硬件等待完成增量。
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

/// 在所有非零 CPU 上启动工作任务，逐核验证 `KICK_ROUNDS` 轮无时钟中断介入的跨核 Idle 唤醒与同步 CSD 调用，最后回收所有任务。
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
            // 第 2、3 轮注入了测试暂停点与额外 IPI 原因，可能提前唤醒底层等待指令，因此不计入常规完成量直接唤醒轮次。
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
        // SAFETY:
        // 1. `csd_action` 不解引用传入的空指针参数，且回调内部仅执行原子加法、不会睡眠；
        // 2. 当前 CPU 0 任务处于开中断状态且未持有任何自旋锁，`cpu::call` 会同步等待远端硬中断执行完毕并释放完成标志后才返回。
        unsafe { cpu::call(target, csd_action, core::ptr::null_mut()) };
        assert_eq!(scenario().csd_actions.load(Ordering::Acquire), actions + 1);
        let observer = scenario().kicks[target].task.lock().take();
        drop(observer);
    }
    (rounds, waits)
}

/// 验证任务执行时间记账（剔除阻塞与迁移间隙）与每 CPU 四分类时间统计（任务、硬中断、调度器、Idle）的单调性及守恒关系。
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

/// 时间测试套件的单例共享状态，保存跨任务协调所需的原子标志、完成量与观测数据。
struct Scenario {
    /// 测试用例创建的定时器与工作队列节点所有权集合。
    nodes: super::Nodes,
    /// 全局停止标志，通知所有辅助工作任务尽快退出。
    stop: AtomicBool,
    /// 记录时间片延迟抢占测试中的同核竞争任务是否已运行。
    peer_ran: AtomicBool,
    /// 控制单调时钟并发读写任务同时起跑的发令标志。
    time_go: AtomicBool,
    /// 已到达起跑屏障的单调时钟读写任务数量。
    time_arrived: AtomicUsize,
    /// 单调时钟写者当前已发布的时间维护轮次。
    write_phase: AtomicUsize,
    /// 所有单调时钟读者累计完成的采样次数。
    read_progress: AtomicUsize,
    /// 完成全部 `READ_ROUNDS` 轮采样的读者任务数量。
    readers_passed: AtomicUsize,
    /// 执行时间记账测试的工作任务句柄（由自旋锁保护）。
    account_task: SpinLock<Option<Arc<Task>>>,
    /// 执行时间记账工作任务当前的阶段（`1` 表示已完成首段忙等并准备进入阻塞）。
    account_phase: AtomicUsize,
    /// 用于唤醒执行时间记账工作任务的完成量。
    account_wake: Completion,
    /// 执行时间记账工作任务记录的累计 On-CPU 执行时长（单位：纳秒）。
    task_ns: AtomicU64,
    /// 执行时间记账工作任务记录的起止墙上时钟总跨度（单位：纳秒）。
    wall_ns: AtomicU64,
    /// 执行时间记账工作任务完成的跨 CPU 迁移次数。
    migrations: AtomicUsize,
    /// 跨核同步调用（`cpu::call`）回调累计执行次数。
    csd_actions: AtomicUsize,
    /// 各 CPU 对应的跨核 Idle 唤醒测试槽位（按 CPU ID 索引，最多支持 4 核）。
    kicks: [KickSlot; 4],
}

/// 指向当前正在运行的测试用例 `Scenario` 实例的全局原子裸指针。
static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 获取当前活动测试用例的 `Arc<Scenario>` 共享引用。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY:
    // 1. `ServiceFixture::set_up` 通过 `Arc::into_raw` 导出了一个强引用计数并将其存入 `CURRENT`，在 `tear_down` 回收所有子任务与异步回调前该指针始终有效；
    // 2. 先调用 `increment_strong_count` 增加一次强引用，再通过 `Arc::from_raw` 构造临时 `Arc`，保证返回后 `CURRENT` 所持有的原始强引用不会被消耗。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}

/// 时间与执行记账测试夹具（Fixture），负责在每个用例前后管理 `Scenario` 生命周期并核对全局资源基线。
struct ServiceFixture {
    /// 当前测试用例的共享状态实例。
    state: Arc<Scenario>,
    /// 用例启动前记录的物理内存与工作队列元数据基线快照。
    initial: super::Baseline,
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 初始化单个测试用例：将当前任务迁移至 CPU 0，采样资源基线，构造 `Scenario` 并注册到全局指针 `CURRENT`。
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

    /// 静默阶段（Quiesce）：通知所有测试任务停止并解除底层暂停点，随后同步关闭已注册的节点。
    fn quiesce(&mut self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        self.state.stop_sources();
        self.state.nodes.stop()?;
        Ok(())
    }

    /// 拆卸阶段（Tear Down）：在所有子任务退出且异步回调全部归还引用后，注销全局指针 `CURRENT` 并校验内存与元数据无泄漏。
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
        // SAFETY:
        // 1. `pointer` 是 `set_up` 中通过 `Arc::into_raw` 存入 `CURRENT` 的唯一裸指针，且已通过原子 `swap` 清空；
        // 2. 此时 `quiesce` 已停止所有异步源、测试框架已回收全部子任务，没有任何并发上下文再通过 `CURRENT` 访问该状态，因此可安全恢复 `Arc` 并销毁 `owner`。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl Scenario {
    /// 停止当前测试用例启动的所有任务与硬件暂停点（幂等操作，仅首次调用生效）。
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
        // SAFETY:
        // 1. 顺序锁快照重叠测试中的唯一读者运行在 `fixed` 协调任务内，本函数仅在该协调任务正常返回或栈展开（Unwind）后才被调用；
        // 2. 此时没有任何任务停留在顺序锁采样暂停窗口中，因此可以安全清除全局快照重叠测试状态。
        unsafe {
            clock::testing::cancel_snapshot_overlap();
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

#[::ktest::suite(fixture = ServiceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "time"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// **测试目标**：验证启动阶段 RTC 实时时钟基准解析在合法时间、缺失设备与非法日历输入下的处理。
    /// - **Given**：提供有效 Unix 纪元、闰年日期、无 RTC 设备、非法月日/平年 2 月 29 日或读取未稳定错误样本；
    /// - **When**：调用 `clock::testing::rtc_realtime_base(sample)` 解析实时时钟基准；
    /// - **Then**：有效样本返回精确的纳秒时间戳且未置位不可用原因，无效或缺失样本返回 `0` 并携带明确的不可用标识。
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

    /// **测试目标**：验证 RTC 日历字段向 Unix 纳秒时间戳换算的各项合法边界与越界拒绝。
    /// - **Given**：构造覆盖 1970 纪元起点、2000 世纪闰年、2038 32 位有符号边界，以及平年 2 月 29 日、年份溢出与非法月/日/时/分/秒的位域参数；
    /// - **When**：调用 `clock::testing::rtc_calendar` 执行日历换算；
    /// - **Then**：合法时间返回对应的纳秒时间戳，所有越界或非法日历组合均返回 `None`。
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

    /// **测试目标**：验证实时时钟（UTC）向单调时钟截止时间的投影精度及睡眠唤醒行为。
    /// - **Given**：采样当前单调时钟与实时时钟，并分别构造已过期（0）与未来（+20ms）的 UTC 截止时间；
    /// - **When**：调用 `clock::realtime_deadline` 投影为单调时钟截止时间并调用 `sleep_until`；
    /// - **Then**：当前 UTC 投影落在前后两次单调时钟采样区间内；过去截止时间立即返回，未来截止时间在单调时钟与实时时钟均跨过目标时刻后唤醒。
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

    /// **测试目标**：验证时间片硬中断的延迟抢占（Deferred Preemption）语义与中断上下文整数寄存器保存完整性。
    /// - **Given**：在 CPU 0 上运行工作任务并创建同核就绪竞争任务，关中断重装当前时间片；
    /// - **When**：自旋等待时钟硬中断触发，随后显式调用 `sched::cond_resched()` 并执行寄存器保存测试；
    /// - **Then**：硬中断仅置位 `need_resched` 而不直接切换任务，到达 `cond_resched` 安全点时才切换至竞争任务，且中断前后通用整数寄存器完整保留。
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

    /// **测试目标**：验证定点数时间维护器（`Timekeeper`）在多种计数器频率下的掩码回绕、长计数、亚纳秒余数累积与最大差值边界。
    /// - **Given**：分别指定 1.000003MHz、10MHz、24MHz 与 1GHz 四种频率，以及对应的五种测试模式（0..=4）；
    /// - **When**：调用 `arithmetic_case(frequency, mode)` 执行定点数换算与更新；
    /// - **Then**：换算结果与 128 位精确参考值一致，余数在连续单步更新中精确累积，超出 `max_delta` 的差值被安全拒绝。
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

    /// **测试目标**：验证单调时钟顺序锁（SeqLock）在读写跨核重叠时能够识别世代变化并重试采样。
    /// - **Given**：CPU 0 读者进入采样暂停点，CPU 1 写者并发调用 `maintain()` 推进时间维护器世代；
    /// - **When**：CPU 0 读者恢复采样并完成 `clock::now()` 读取；
    /// - **Then**：读者检测到世代号变更并至少执行一次重试，最终返回的时间戳保持单调不减。
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

    /// **测试目标**：验证多任务跨 CPU 并发读写单调时钟时的全局单调性与维护次数。
    /// - **Given**：在 CPU 0 启动时间维护写者任务，并在其余 CPU 上启动一至两个并发读者任务；
    /// - **When**：写者与读者交错推进并连续采样 `READ_ROUNDS`（128）轮；
    /// - **Then**：所有读者观测到的时间戳严格单调不减，且全局时间维护器的更新次数不少于总轮数。
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

    /// **测试目标**：验证任务执行时间记账（排除阻塞睡眠与跨核迁移间隙）及每 CPU 四分类时间统计的准确性。
    /// - **Given**：启动记账工作任务，依次经历 CPU 0 忙等、完成量阻塞睡眠（15ms）以及在 CPU 1 与 CPU 0 之间的跨核迁移；
    /// - **When**：在任务阻塞期间及退出后分别采样任务执行时间、墙上时钟跨度与各核 `CpuSnapshot`；
    /// - **Then**：阻塞期间任务执行时间保持不变，总执行时间加上阻塞时长不超过墙上时钟总跨度，且中断、调度器与 Idle 记账均正向增长。
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

    /// **测试目标**：验证远端 CPU 在无时钟中断介入时可通过跨核 IPI 从底层硬件 Idle 状态唤醒，且 IPI 合并原因不丢失。
    /// - **Given**：在所有非零 CPU 上启动绑定工作任务并等待其进入底层 WFI/IDLE 等待；
    /// - **When**：主核通过完成量唤醒及暂停点注入连续执行多轮唤醒测试；
    /// - **Then**：每个远端 CPU 均完成 4 轮有效硬件等待唤醒，期间目标核时钟中断计数零增长，且并发注入的定时器重编程请求得到完整处理。
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

    /// **测试目标**：验证工作任务停留在轮次末尾暂停点时，若主协调任务发生受控失败，清理流程仍能唤醒并完整回收子任务。
    /// - **Given**：CPU 1 上的工作任务完成一轮唤醒后停留在 `round_paused` 自旋点；
    /// - **When**：主协调任务发送停止请求、解除轮次暂停并主动返回 `Err` 模拟协调失败；
    /// - **Then**：工作任务正常退出并被 `join` 回收，所有完成量等待队列收敛为空，框架最终报告预期的受控失败。
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
