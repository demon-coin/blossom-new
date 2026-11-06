//! 声明了 `expected_fatal` 但测试主体正常返回时的生命周期收敛与资源清理自测套件。
//!
//! # 模块职责与架构定位
//! 本模块验证当一个测试用例声明了预期致命事件（如 `#[ktest(expected_fatal = "oom")]`），
//! 但测试主体实际上并未触发该致命事件而是正常返回 `Ok(())` 时，测试框架不会直接中止或跳过清理，
//! 而是严格按照标准生命周期依次执行：
//! 1. `Quiesce`：调用 `Fixture::quiesce` 并发出子任务协作停止信号；
//! 2. `Join`：阻塞等待子任务完整执行完栈上局部变量的 `Drop` 并彻底退出内核栈；
//! 3. `Cleanup`：调用 `Fixture::tear_down` 释放夹具资源并执行夹具自身的 `Drop`；
//! 4. 最终将该用例判定为“未触发预期致命事件”的普通测试失败上报给宿主。
//!
//! # 核心状态流转与生命周期校验
//! ```text
//! set_up()         : 创建共享计数器 State (entered=0, child_drops=0, fixture_drops=0)
//!    |
//!    v
//! normal_return()  : 派生子任务持有 ChildDrop 守卫并轮询 stop_requested()；
//!                    主线程等待 entered==1 后正常返回 Ok(())（未触发预期的 OOM）
//!    |
//!    v
//! quiesce()        : 标记 quiesced=true；随后框架调用 stop_children() 唤醒子任务退出循环
//!    |
//!    v
//! join_children()  : 子任务退出作用域触发 ChildDrop::drop (child_drops=1) 并释放 Arc<State>
//!    |
//!    v
//! tear_down()      : 断言 try_join()==Some(0)、child_drops==1 且 State 强引用计数归 1；
//!                    显式 drop(self) 触发 ReturnFixture::drop (fixture_drops=1)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 父线程与子线程之间仅通过无锁原子计数器（[`State`] 中的 `AtomicUsize`）及 `ChildToken::stop_requested`
//!   进行同步，不持有额外锁。

use alloc::sync::Arc;
use core::sync::atomic::{AtomicUsize, Ordering};

use ::ktest::{Fixture, TestContext, TestError, TestResult};

/// 记录子任务进入、子任务局部变量析构以及测试夹具析构次数的共享原子状态。
struct State {
    /// 子任务已进入执行主体的次数（0 表示未进入，1 表示已进入并构造好栈上守卫）。
    entered: AtomicUsize,
    /// 子任务栈上 [`ChildDrop`] 守卫执行 `Drop::drop` 的次数。
    child_drops: AtomicUsize,
    /// 测试夹具 [`ReturnFixture`] 执行 `Drop::drop` 的次数。
    fixture_drops: AtomicUsize,
}

/// 绑定在子任务栈帧上的 RAII 析构探针守卫（元组字段 `0` 为指向父夹具共享计数状态 `State` 的强引用）。
struct ChildDrop(Arc<State>);

impl Drop for ChildDrop {
    /// 当子任务正常退出其闭包栈帧时递增 `child_drops` 计数，用于证明子任务已完整执行完栈上析构逻辑。
    fn drop(&mut self) {
        self.0.child_drops.fetch_add(1, Ordering::Release);
    }
}

/// 验证预期致命事件未发生时仍能完整走完停止、回收与清理流程的测试夹具。
struct ReturnFixture {
    /// 父线程与子任务共享的生命周期计数器。
    state: Arc<State>,
    /// 测试主体派生的后台子任务观察句柄。
    child: Option<crate::ktest::context::ChildHandle>,
    /// 标记是否已执行过 `Fixture::quiesce` 阶段。
    quiesced: bool,
}

impl Fixture for ReturnFixture {
    /// 初始化夹具状态计数器，不执行磁盘 I/O 或主动触发内存耗尽。
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

    /// 记录用例进入 `Quiesce` 静止化阶段；此时子任务句柄已就绪，保持共享资源有效直至后续 `Join` 完成。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        assert!(!self.quiesced);
        assert!(self.child.is_some());
        self.quiesced = true;
        context.diagnostic(format_args!("fatal-return quiesce once"));
        Ok(())
    }

    /// 在框架完成 `Join` 后，校验子任务已彻底退出内核栈并释放了其持有的 `Arc<State>`，随后验证夹具自身的析构。
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
    /// 当夹具在 `Cleanup` 阶段被销毁时递增 `fixture_drops` 计数。
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

    /// 验证声明了 `expected_fatal = "oom"` 的用例在未触发 OOM 而正常返回时，子任务仍能被协作停止、回收并完整执行 `tear_down`。
    ///
    /// - **测试目标**：确认当预期致命错误未发生时，框架能严格按 `Quiesce -> Join -> Cleanup` 顺序回收后台子任务并清理夹具，最后将用例报告为失败。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：用例标注 `#[ktest(expected_fatal = "oom")]` 并完成 [`ReturnFixture::set_up`] 初始化；
    ///   2. When：在 CPU 0 上派生一个持有 [`ChildDrop`] 并循环等待 `stop_requested()` 的子任务，待其置位 `entered = 1` 后，测试主体不触发 OOM 直接返回 `Ok(())`；
    ///   3. Then：框架依次执行 `quiesce`、通知子任务停止并 `join`、调用 `tear_down` 断言 `child_drops == 1` 与 `fixture_drops == 1`，最终上报未命中预期致命错误的失败结果。
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
