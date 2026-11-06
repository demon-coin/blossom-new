//! 子任务准入（Task Admission）在物理内存耗尽（OOM）时的失败回滚与析构上下文测试。
//!
//! # 模块职责与架构定位
//! 本模块属于 `ktest` 内核自检套件，用于验证 `crate::ktest::spawn_on` 在物理页耗尽导致子任务
//! 创建失败时，是否正确地将未发布的闭包带回调用方任务的普通可睡眠上下文中析构。
//! - **上游调用者**：`ktest` 线程级（`Thread` 上下文）测试运行器。
//! - **下游被测模块**：`ktest::spawn_on` 任务准入接口、物理页分配器（`crate::mm`）、任务调度器（`crate::sched`）与定时器睡眠接口（`crate::timer`）。
//!
//! # 核心机制与执行流
//! ```text
//! 调用方 Task (普通可睡眠线程上下文)
//!   │
//!   ├─ 1. 预先创建 1 个后台驻留子任务，验证正常准入与框架回收路径
//!   ├─ 2. 持续分配 order-0 物理页直至返回 None，构造确定性的 OOM 压力环境
//!   ├─ 3. 调用 ktest::spawn_on(0, move |_| { drop(capture); ... })
//!   │       ├─ 内部尝试分配子 Task 栈/控制块失败 -> 准备返回 Err("NoMemory: child Task")
//!   │       └─ 闭合锁临界区后丢弃未运行的闭包 -> 触发 SleepingCapture::drop()
//!   └─ 4. 执行 SleepingCapture::drop()
//!           ├─ 检查当前 CPU：中断开启、抢占计数 = 0、硬中断/下半部深度 = 0
//!           ├─ 调用 timer::sleep(1ms)：验证析构中可安全睡眠且唤醒后仍处于原 Task
//!           └─ 重入调用 ktest::spawn_on：验证准入锁已释放（无死锁）且继续返回 OOM
//! ```
//!
//! # 锁获取顺序与并发契约
//! - **失败路径锁外析构契约**：`spawn_on` 内部若持有任务表自旋锁或关中断抢占守卫，必须先释放所有自旋锁并恢复中断与抢占状态，最后才能销毁调用方传入的闭包捕获对象。若在持锁或关中断期间调用 `Drop`，闭包析构中的 `timer::sleep` 会触发关中断睡眠违规，重入 `spawn_on` 则会导致自旋锁死锁。
//! - **内存水位与账务守恒**：测试夹具在 `quiesce` 阶段优先归还被占用的物理页，在 `tear_down` 阶段等待所有后台任务彻底退出并刷空缓存/每 CPU 页缓存后，校验前后内存统计（`mm::MemoryStats` 六项核心指标）完全一致。

use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::{cpu, mm, sched, task, time, timer};

/// 子任务准入失败与闭包析构行为的跨线程原子观测状态。
struct Observation {
    /// `SleepingCapture::drop` 被调用的累计次数（预期为 1 次）。
    drops: AtomicUsize,
    /// OOM 失败的那次 `spawn_on` 闭包主体被执行的次数（预期为 0 次，因任务未创建成功）。
    body_runs: AtomicUsize,
    /// 在 `SleepingCapture::drop` 内部重入调用 `spawn_on` 时，内层闭包主体的执行次数（预期为 0 次）。
    reentrant_body_runs: AtomicUsize,
    /// `SleepingCapture::drop` 中的 `timer::sleep` 是否已顺利返回。
    slept: AtomicBool,
    /// 进入 `SleepingCapture::drop` 时，当前 CPU 是否处于开中断且无抢占/中断嵌套的可睡眠上下文。
    sleep_allowed: AtomicBool,
    /// `SleepingCapture::drop` 睡眠前后是否始终运行在发起调用的同一个父 `Task` 上。
    same_task: AtomicBool,
    /// `SleepingCapture::drop` 内部重入调用 `spawn_on` 是否如期观察到 `NoMemory: child Task` 错误。
    reentrant_oom: AtomicBool,
    /// 压力开始前创建的正常驻留子任务是否已进入运行态。
    child_entered: AtomicBool,
    /// 正常驻留子任务在收到停止信号后是否已完整执行到函数末尾并退出。
    child_exited: AtomicBool,
}

/// 物理页耗尽（OOM）准入测试夹具。
struct PressureFixture {
    /// 测试开始前系统空闲资源停稳后的基线内存统计快照（单位：物理页数）。
    baseline: mm::MemoryStats,
    /// 测试期间持有的所有 order-0 物理页凭证容器，用于主动耗尽空闲物理内存并在清理阶段统一归还。
    held: Vec<mm::OwnedPages>,
    /// 供父任务、驻留子任务及析构函数共享的原子观测账本。
    state: Arc<Observation>,
    /// 标记本轮测试是否已成功发布过驻留子任务。
    published: bool,
}

/// 等待除当前核外的所有 CPU 进入空闲（idle）状态，并收缩内核缓存与每 CPU 页缓存，
/// 返回系统空闲资源停稳后的内存统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("admission settle deadline overflow"))?;
    // Step 1: 轮询等待其余 CPU 均处于 idle 状态，避免后台异步回收干扰内存基线。
    while !(0..cpu::count())
        .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    {
        if time::now() >= deadline {
            return Err(TestError("admission resources did not settle"));
        }
        sched::yield_now();
    }

    // Step 2: 收缩可回收的内核缓存并将每 CPU 页列表（PCP）归还到全局伙伴系统。
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

impl Fixture for PressureFixture {
    /// 在注入内存压力前预先分配页凭证容器容量与原子观测对象，确保压力期间夹具自身不再触发堆分配。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        // Step 1: 获取停稳后的基线内存统计，并按当前总物理页数预留 Vec 容量。
        let baseline = settled()?;
        let mut held = Vec::new();
        held.try_reserve_exact(baseline.present_pages)
            .map_err(|_| TestError("NoMemory: admission page owners"))?;
        // Step 2: 初始化共享原子观测状态。
        let state = Arc::try_new(Observation {
            drops: AtomicUsize::new(0),
            body_runs: AtomicUsize::new(0),
            reentrant_body_runs: AtomicUsize::new(0),
            slept: AtomicBool::new(false),
            sleep_allowed: AtomicBool::new(false),
            same_task: AtomicBool::new(false),
            reentrant_oom: AtomicBool::new(false),
            child_entered: AtomicBool::new(false),
            child_exited: AtomicBool::new(false),
        })
        .map_err(|_| TestError("NoMemory: admission observation"))?;

        Ok(Self {
            baseline,
            held,
            state,
            published: false,
        })
    }

    /// 无论测试主体成功或失败，先清空并归还本用例占用的全部物理页，使后续子任务停止与回收得以正常推进。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.held.clear();
        Ok(())
    }

    /// 在测试框架完成所有已发布子任务的线程回收（join）后，校验闭包析构行为、子任务退出状态以及内存六项统计守恒。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 校验 OOM 时未发布闭包的析构次数、睡眠合法性、所属任务一致性及重入结果。
        context.expect_eq(self.state.drops.load(Ordering::Acquire), 1);
        context.expect_eq(self.state.body_runs.load(Ordering::Acquire), 0);
        context.expect_eq(self.state.reentrant_body_runs.load(Ordering::Acquire), 0);
        context.expect_eq(self.state.slept.load(Ordering::Acquire), true);
        context.expect_eq(self.state.sleep_allowed.load(Ordering::Acquire), true);
        context.expect_eq(self.state.same_task.load(Ordering::Acquire), true);
        context.expect_eq(self.state.reentrant_oom.load(Ordering::Acquire), true);
        context.expect_eq(
            self.state.child_exited.load(Ordering::Acquire),
            self.published,
        );
        context.diagnostic(format_args!(
            "admission body-runs={} capture-drops={} sleep={} same-task={} sleep-allowed={} reentrant-oom={} reentrant-body-runs={} child-exited={}",
            self.state.body_runs.load(Ordering::Acquire),
            self.state.drops.load(Ordering::Acquire),
            self.state.slept.load(Ordering::Acquire),
            self.state.same_task.load(Ordering::Acquire),
            self.state.sleep_allowed.load(Ordering::Acquire),
            self.state.reentrant_oom.load(Ordering::Acquire),
            self.state.reentrant_body_runs.load(Ordering::Acquire),
            self.state.child_exited.load(Ordering::Acquire),
        ));

        // Step 2: 释放夹具自身持有的堆内存，重新等待系统停稳并校验六个内存统计域完全回到基线。
        let baseline = self.baseline;
        drop(self);
        let after = settled()?;
        context.expect_eq(after, baseline);
        context.diagnostic(format_args!(
            "admission six-domains-before={:?} six-domains-after={:?}",
            baseline, after
        ));

        Ok(())
    }
}

/// 被待创建子任务闭包按值捕获的测试守卫，用于在准入失败销毁闭包时验证析构上下文是否允许睡眠与重入。
struct SleepingCapture {
    /// 共享测试观测状态，用于在 `Drop` 中记录执行轨迹。
    state: Arc<Observation>,
    /// 构造该捕获对象时的父任务 ID，用于校验析构函数是否在原发起任务上执行。
    owner: task::TaskId,
}

impl Drop for SleepingCapture {
    /// 在闭包被销毁时执行的析构逻辑：记录上下文状态、执行一次定时器睡眠，并重入调用 `spawn_on`。
    fn drop(&mut self) {
        // Step 1: 记录析构发生，并检查当前 CPU 是否处于开中断且无抢占禁用、无硬中断与下半部嵌套的普通线程上下文。
        self.state.drops.fetch_add(1, Ordering::AcqRel);
        let local = cpu::current();
        self.state.sleep_allowed.store(
            crate::arch::irq_enabled()
                && local.preempt_depth.load(Ordering::Relaxed) == 0
                && local.hardirq_depth.load(Ordering::Relaxed) == 0
                && local.bh_depth.load(Ordering::Relaxed) == 0,
            Ordering::Release,
        );
        // Step 2: 记录睡眠前任务 ID 并睡眠 1 毫秒，验证析构路径不会在持自旋锁或关中断状态下调用调度器。
        let before = task::current_task_id();
        timer::sleep(time::Duration::from_millis(1));
        self.state.slept.store(true, Ordering::Release);
        self.state.same_task.store(
            before == Some(self.owner) && task::current_task_id() == Some(self.owner),
            Ordering::Release,
        );

        // Step 3: 在析构函数内再次调用 spawn_on（不携带递归 SleepingCapture），验证准入路径已释放内部锁且继续返回 OOM。
        let state = Arc::clone(&self.state);
        let result = crate::ktest::spawn_on(0, move |_| {
            state.reentrant_body_runs.fetch_add(1, Ordering::AcqRel);
            Ok(())
        });
        self.state.reentrant_oom.store(
            result.err() == Some(TestError("NoMemory: child Task")),
            Ordering::Release,
        );
    }
}

#[::ktest::suite(context = "Thread", fixture = PressureFixture, min_cpus = 1, tags("admission", "oom", "thread"), timeout_ms = 15000)]
mod children {
    use super::*;

    /// 测试目标：
    /// 验证在物理页耗尽（OOM）导致 `ktest::spawn_on` 创建子任务失败时，未被调度的闭包不会执行主体，
    /// 且其捕获的 `SleepingCapture` 会在原任务的锁外可睡眠上下文中完成析构、睡眠与重入调用。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：内存充足时先通过 `spawn_on` 启动一个驻留子任务并等待其进入运行循环，随后刷空缓存并循环分配完所有 order-0 物理页，构造 OOM 状态；
    /// 2. When：构造携带 `SleepingCapture` 的闭包并调用 `spawn_on` 尝试创建第二个子任务；
    /// 3. Then：`spawn_on` 返回 `Err(TestError("NoMemory: child Task"))`，闭包主体不执行，`SleepingCapture::drop` 在原任务上下文中完成 1ms 睡眠与重入 OOM 检查，清理后内存统计完全恢复基线。
    #[test]
    fn prepare_oom_sleeping_capture(
        context: &mut TestContext<'_>,
        fixture: &mut PressureFixture,
    ) -> TestResult {
        // Step 1: 在物理内存耗尽前先启动一个正常的后台子任务，并等待其进入运行状态。
        let state = Arc::clone(&fixture.state);
        let handle = crate::ktest::spawn_on(0, move |token| {
            state.child_entered.store(true, Ordering::Release);
            while !token.stop_requested() {
                sched::yield_now();
            }
            state.child_exited.store(true, Ordering::Release);
            Ok(())
        })?;
        fixture.published = true;
        drop(handle);
        let deadline = time::now()
            .checked_add(time::Duration::from_secs(5))
            .ok_or(TestError("admission child start deadline overflow"))?;
        while !fixture.state.child_entered.load(Ordering::Acquire) {
            if time::now() >= deadline {
                return Err(TestError("admission child did not enter"));
            }
            sched::yield_now();
        }

        // Step 2: 构造由当前任务拥有的 SleepingCapture，并抽干系统内全部可分配的 order-0 物理页。
        let capture = SleepingCapture {
            state: Arc::clone(&fixture.state),
            owner: task::current_task_id().ok_or(TestError("admission requires ordinary Task"))?,
        };
        mm::shrink_all();
        mm::drain_all_pages();
        while let Some(pages) = mm::alloc_pages(0, false) {
            assert!(fixture.held.len() < fixture.held.capacity());
            fixture.held.push(pages);
        }
        mm::drain_all_pages();
        assert!(
            mm::alloc_pages(0, false).is_none(),
            "admission pressure was not real OOM"
        );

        // Step 3: 在 OOM 状态下调用 spawn_on，触发子任务准备失败与 capture 的即时锁外析构。
        let result = crate::ktest::spawn_on(0, move |_| {
            capture.state.body_runs.fetch_add(1, Ordering::AcqRel);
            drop(capture);
            Ok(())
        });
        context.expect_eq(result.err(), Some(TestError("NoMemory: child Task")));
        Ok(())
    }
}
