//! 预期致命事件未发生时，以真实子任务收敛和资源析构证明普通失败路径。

use alloc::sync::Arc;
use core::sync::atomic::{AtomicUsize, Ordering};

use ::ktest::{Fixture, TestContext, TestError, TestResult};

struct State {
    entered: AtomicUsize,
    child_drops: AtomicUsize,
    fixture_drops: AtomicUsize,
}

struct ChildDrop(Arc<State>);
impl Drop for ChildDrop {
    /// 子任务必须真实退出 Rust 主体，不能由父用手动计数代替析构。
    fn drop(&mut self) {
        self.0.child_drops.fetch_add(1, Ordering::Release);
    }
}

struct ReturnFixture {
    state: Arc<State>,
    child: Option<crate::ktest::context::ChildHandle>,
    quiesced: bool,
}
impl Fixture for ReturnFixture {
    /// 建立本例独立拥有型资源；不访问磁盘，也不故意制造 OOM。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let state = Arc::try_new(State {
            entered: AtomicUsize::new(0),
            child_drops: AtomicUsize::new(0),
            fixture_drops: AtomicUsize::new(0),
        })
        .map_err(|_| TestError("NoMemory: fatal-return fixture"))?;
        context.diagnostic(format_args!("fatal-return setup once"));
        Ok(Self {
            state,
            child: None,
            quiesced: false,
        })
    }
    /// 框架已请求合作停止；保留资源直到真实 join 完成，不提前释放子任务可见状态。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        assert!(!self.quiesced);
        assert!(self.child.is_some());
        self.quiesced = true;
        context.diagnostic(format_args!("fatal-return quiesce once"));
        Ok(())
    }
    /// join 后检查真实离栈及独占资源所有权，再执行并观察一次完整析构。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert!(self.quiesced);
        assert_eq!(self.child.as_ref().unwrap().try_join(), Some(0));
        assert_eq!(self.state.child_drops.load(Ordering::Acquire), 1);
        assert_eq!(Arc::strong_count(&self.state), 1);
        context.diagnostic(format_args!("fatal-return join observed child Drop once"));
        let state = self.state.clone();
        drop(self);
        assert_eq!(state.fixture_drops.load(Ordering::Acquire), 1);
        assert_eq!(Arc::strong_count(&state), 1);
        context.diagnostic(format_args!("fatal-return cleanup once fixture Drop once"));
        Ok(())
    }
}
impl Drop for ReturnFixture {
    /// 真实对象释放只在框架安全清理时发生，主体和静止化不会代行此责任。
    fn drop(&mut self) {
        self.state.fixture_drops.fetch_add(1, Ordering::Release);
    }
}

#[::ktest::suite(
    context = "Thread",
    isolation = "Case",
    fixture = ReturnFixture,
    tags("selftest", "selftest-failure", "fatal-return"),
    ignore = "intentional missing expected fatal; select explicitly"
)]
mod expected {
    use super::*;

    /// 真实子任务等待合作停止；主体正常返回，不调用 panic、terminate 或主动致命 API。
    #[test]
    #[ktest(expected_fatal = "oom")]
    fn normal_return(context: &mut TestContext<'_>, fixture: &mut ReturnFixture) -> TestResult {
        let state = fixture.state.clone();
        fixture.child = Some(crate::ktest::spawn_on(0, move |token| {
            let local = ChildDrop(state);
            local.0.entered.store(1, Ordering::Release);
            while !token.stop_requested() {
                crate::sched::yield_now();
            }
            Ok(())
        })?);
        while fixture.state.entered.load(Ordering::Acquire) == 0 {
            crate::sched::yield_now();
        }
        assert_eq!(fixture.state.child_drops.load(Ordering::Acquire), 0);
        assert_eq!(fixture.state.fixture_drops.load(Ordering::Acquire), 0);
        context.diagnostic(format_args!("fatal-return body returning Ok without fatal"));
        Ok(())
    }
}
