//! 真实页耗尽时，未发表子入口的合法可睡析构必须在原 Task 的锁外完成。

use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::{cpu, mm, sched, task, time, timer};

struct Observation {
    drops: AtomicUsize,
    body_runs: AtomicUsize,
    reentrant_body_runs: AtomicUsize,
    slept: AtomicBool,
    sleep_allowed: AtomicBool,
    same_task: AtomicBool,
    reentrant_oom: AtomicBool,
    child_entered: AtomicBool,
    child_exited: AtomicBool,
}

struct PressureFixture {
    baseline: mm::MemoryStats,
    held: Vec<mm::OwnedPages>,
    state: Arc<Observation>,
    published: bool,
}

/// 沿用存储压力验收的停稳规范；只整理真实空闲资源，不回滚快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("admission settle deadline overflow"))?;
    while !(0..cpu::count())
        .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    {
        if time::now() >= deadline {
            return Err(TestError("admission resources did not settle"));
        }
        sched::yield_now();
    }
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

impl Fixture for PressureFixture {
    /// 页凭证容器与全部观察原子在压力开始前 fallible 预备，每例独立拥有。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled()?;
        let mut held = Vec::new();
        held.try_reserve_exact(baseline.present_pages)
            .map_err(|_| TestError("NoMemory: admission page owners"))?;
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

    /// 无论主体成功或失败，先归还本例真实压力页；父框架仍负责全部已发表子任务 join。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.held.clear();
        Ok(())
    }

    /// 只有真实离栈 join 后才检查析构和子尾部，再释放观察空间核完整六域账务。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
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

struct SleepingCapture {
    state: Arc<Observation>,
    owner: task::TaskId,
}

impl Drop for SleepingCapture {
    /// 普通 Send 捕获的真实析构：先睡眠，再重入无递归析构捕获的原准入 API。
    fn drop(&mut self) {
        self.state.drops.fetch_add(1, Ordering::AcqRel);
        let local = cpu::current();
        self.state.sleep_allowed.store(
            crate::arch::irq_enabled()
                && local.preempt_depth.load(Ordering::Relaxed) == 0
                && local.hardirq_depth.load(Ordering::Relaxed) == 0
                && local.bh_depth.load(Ordering::Relaxed) == 0,
            Ordering::Release,
        );
        let before = task::current_task_id();
        // 不提前断言 IRQ 状态，修前必须由真实 sleep 的调度检查报告 irq-schedule。
        timer::sleep(time::Duration::from_millis(1));
        self.state.slept.store(true, Ordering::Release);
        self.state.same_task.store(
            before == Some(self.owner) && task::current_task_id() == Some(self.owner),
            Ordering::Release,
        );
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

    /// 耗尽真实 order0 页后 fallible 准备失败，不执行 body，合法捕获仍睡眠并重入完成一次。
    #[test]
    fn prepare_oom_sleeping_capture(
        context: &mut TestContext<'_>,
        fixture: &mut PressureFixture,
    ) -> TestResult {
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
        let result = crate::ktest::spawn_on(0, move |_| {
            capture.state.body_runs.fetch_add(1, Ordering::AcqRel);
            drop(capture);
            Ok(())
        });
        context.expect_eq(result.err(), Some(TestError("NoMemory: child Task")));
        Ok(())
    }
}
