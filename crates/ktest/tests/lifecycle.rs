//! `ktest` 用例生命周期状态机、异常隔离与资源清理回归测试。
//!
//! # 模块职责与架构定位
//! 本模块在宿主环境下直接驱动 `ktest::run_case` 与 `PanicMessage`，验证：
//! 1. 五阶段生命周期（`Setup` -> `Body` -> `Quiesce` -> `Join` -> `Cleanup`）的调用顺序；
//! 2. `Setup` 失败或 Panic、`Body` 断言失败、子任务失败与清理错误之间的隔离与优先级；
//! 3. `#[should_panic]`、`expected_fatal`、局部 `expect_panic` 与非致命 `expect!` 软断言的判定语义；
//! 4. `PanicMessage` 固定缓冲区的 UTF-8 字符边界安全截断。

use std::cell::RefCell;
use std::fmt;
use std::panic::{catch_unwind, panic_any, AssertUnwindSafe};
use std::sync::atomic::{AtomicUsize, Ordering};

use ktest::{
    CaseDesc, CaseOutcome, Caught, Context, Defaults, Failure, Fixture, Isolation, JoinOutcome,
    PanicMessage, Phase, Services, Status, TestContext, TestError, TestResult,
};

/// 普通测试用例的静态描述符基准模板。
static CASE: CaseDesc = CaseDesc {
    name: "host::lifecycle",
    parameter: None,
    source: file!(),
    line: line!(),
    attributes: Defaults {
        context: Context::Boot,
        isolation: Isolation::Suite,
        tags: &[],
        min_cpus: 1,
        timeout_ms: 1000,
        resources: &[],
    },
    ignore: None,
    should_panic: None,
    expected_fatal: None,
    fatal_message: None,
};
/// 声明了 `should_panic = Some("controlled")` 的测试用例静态描述符。
static EXPECTED: CaseDesc = CaseDesc {
    name: "host::expected",
    should_panic: Some("controlled"),
    ..CASE
};
/// 声明了 `expected_fatal = Some("oom")` 的测试用例静态描述符。
static EXPECTED_FATAL: CaseDesc = CaseDesc {
    name: "host::expected_fatal",
    expected_fatal: Some("oom"),
    ..CASE
};

/// 宿主环境下用于记录生命周期阶段轨迹并模拟子任务结果的测试运行时服务。
struct Runtime {
    /// 按发生顺序记录 `Services::phase` 收到的所有生命周期阶段。
    phases: RefCell<Vec<Phase>>,
    /// 按输出顺序保存 `Services::diagnostic` 收到的诊断消息文本。
    messages: RefCell<Vec<String>>,
    /// 预设在 `Services::join_children` 阶段返回的后台子任务失败结果。
    child: Option<Failure>,
}
impl Runtime {
    /// 创建空的宿主测试运行时服务实例。
    fn new() -> Self {
        Self {
            phases: RefCell::new(Vec::new()),
            messages: RefCell::new(Vec::new()),
            child: None,
        }
    }
}
impl Services for Runtime {
    /// 使用 `std::panic::catch_unwind` 捕获回调中的 Panic，支持直接解包 `PanicMessage` 载荷以测试截断标志。
    fn catch(&self, callback: &mut dyn FnMut() -> TestResult) -> Caught {
        match catch_unwind(AssertUnwindSafe(callback)) {
            Ok(value) => Caught::Returned(value),
            Err(payload) => {
                // 若测试显式通过 `panic_any(PanicMessage)` 抛出预置了 `truncated` 标志的消息，直接原样保留。
                if let Some(message) = payload.downcast_ref::<PanicMessage>() {
                    return Caught::Panicked(*message);
                }
                let message = payload
                    .downcast_ref::<&str>()
                    .copied()
                    .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
                    .unwrap_or("non-string host panic");
                Caught::Panicked(PanicMessage::new(message, false))
            }
        }
    }
    /// 将诊断文本推入 `messages` 向量以便测试断言检查。
    fn diagnostic(&self, message: fmt::Arguments<'_>) {
        self.messages.borrow_mut().push(message.to_string());
    }
    /// 将当前进入的 `Phase` 追加到 `phases` 轨迹列表中。
    fn phase(&self, phase: Phase) {
        self.phases.borrow_mut().push(phase);
    }
    /// 本宿主测试未启动异步线程，停止操作为空。
    fn stop_children(&self) {}
    /// 返回预设的后台子任务结果 `self.child`，用于验证主线程结果与子任务结果的组合判定。
    fn join_children(&self) -> Result<JoinOutcome, TestError> {
        Ok(JoinOutcome {
            failure: self.child,
        })
    }
    /// 当清理阶段失败触发终止时，通过 `panic_any(*outcome)` 将 `CaseOutcome` 抛给外层测试断言捕获检查。
    fn terminate(&self, outcome: &CaseOutcome) -> ! {
        panic_any(*outcome)
    }
}

/// 统计夹具 `quiesce` 调用次数与 `Drop` 析构次数的共享原子计数器。
struct Counters {
    /// `Fixture::quiesce` 被调用的累计次数。
    quiesces: AtomicUsize,
    /// 夹具实例执行 `Drop::drop` 析构的累计次数。
    drops: AtomicUsize,
}
/// 持有 `Arc<Counters>` 的测试夹具，用于追踪夹具对象的构造、静止化与 RAII 析构时机。
struct FixtureOwner(std::sync::Arc<Counters>);
impl Fixture<std::sync::Arc<Counters>> for FixtureOwner {
    /// 克隆传入的 `Arc<Counters>` 句柄以构造夹具实例。
    fn set_up(
        _context: &mut TestContext<'_>,
        counters: &std::sync::Arc<Counters>,
    ) -> Result<Self, TestError> {
        Ok(Self(counters.clone()))
    }
    /// 在 `Quiesce` 阶段将 `quiesces` 计数加 1。
    fn quiesce(&mut self, _context: &mut TestContext<'_>) -> TestResult {
        self.0.quiesces.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}
impl Drop for FixtureOwner {
    /// 在夹具对象被 Rust 自动析构时将 `drops` 计数加 1。
    fn drop(&mut self) {
        self.0.drops.fetch_add(1, Ordering::SeqCst);
    }
}

/// 测试目标：验证声明了 `expected_fatal` 的用例在正常返回（未触发致命异常）或提前返回 `Err` 时，仍会完整执行夹具的 `quiesce` 与 `Drop` 清理，并保留首要错误与软断言计数。
/// 场景步骤：针对 `(Ok(()), 无软断言失败)`、`(Ok(()), 有软断言失败)` 与 `(Err, 有软断言失败)` 三种组合运行 `run_case`。
/// 预期结果：每次运行均历经完整的五阶段（`Setup` -> `Body` -> `Quiesce` -> `Join` -> `Cleanup`），`quiesces` 与 `drops` 各为 `1`，`Arc` 强引用计数恢复为 `1`，状态为 `Status::Fail` 且 `primary` 与 `expectations` 准确反映各自故障。
#[test]
fn expected_fatal_return_fails_after_real_fixture_cleanup() {
    for (result, expectation, primary) in [
        (
            Ok(()),
            false,
            Failure::Returned(TestError("expected fatal did not occur")),
        ),
        (
            Ok(()),
            true,
            Failure::Returned(TestError("expected fatal did not occur")),
        ),
        (
            Err(TestError("body failed before fatal")),
            true,
            Failure::Returned(TestError("body failed before fatal")),
        ),
    ] {
        let runtime = Runtime::new();
        let counters = std::sync::Arc::new(Counters {
            quiesces: AtomicUsize::new(0),
            drops: AtomicUsize::new(0),
        });
        let mut context = TestContext::new(&runtime, &EXPECTED_FATAL, 0, 0);
        let outcome =
            ktest::run_case::<FixtureOwner, _>(&mut context, &counters, |context, owner| {
                assert!(std::sync::Arc::ptr_eq(&owner.0, &counters));
                assert_eq!(counters.drops.load(Ordering::SeqCst), 0);
                if expectation {
                    ktest::expect!(context, false, "independent fatal-return expectation");
                }
                result
            });
        assert_eq!(counters.quiesces.load(Ordering::SeqCst), 1);
        assert_eq!(counters.drops.load(Ordering::SeqCst), 1);
        assert_eq!(std::sync::Arc::strong_count(&counters), 1);
        assert_eq!(outcome.status, Status::Fail);
        assert_eq!(outcome.primary, Some(primary));
        assert_eq!(outcome.child, None);
        assert_eq!(outcome.cleanup, None);
        assert_eq!(outcome.expectations, u32::from(expectation));
        assert_eq!(
            *runtime.phases.borrow(),
            [
                Phase::Setup,
                Phase::Body,
                Phase::Quiesce,
                Phase::Join,
                Phase::Cleanup,
            ]
        );
    }
}

/// 测试目标：验证当 `expected_fatal` 用例未触发致命异常且随后在 `tear_down` 清理阶段也返回错误时，终止报告仍保留主体的首要失败原因与软断言计数。
/// 场景步骤：定义在 `tear_down` 中先释放 `FixtureOwner` 再返回 `Err` 的 `CleanupError` 夹具，分别在主体返回 `Ok(())` 和 `Err` 时运行 `run_case` 并捕获 `terminate` 抛出的 `CaseOutcome`。
/// 预期结果：`outcome.status` 为 `Status::Error`，`outcome.primary` 保留缺失致命异常或主体显式错误，`outcome.cleanup` 记录清理错误，且 `outcome.expectations` 为 `1`。
#[test]
fn expected_fatal_return_preserves_primary_through_cleanup_error() {
    /// 包装 `FixtureOwner` 并在 `tear_down` 阶段返回清理错误的测试夹具。
    struct CleanupError(FixtureOwner);
    impl Fixture<std::sync::Arc<Counters>> for CleanupError {
        /// 委托内部 `FixtureOwner` 完成初始化。
        fn set_up(
            context: &mut TestContext<'_>,
            counters: &std::sync::Arc<Counters>,
        ) -> Result<Self, TestError> {
            FixtureOwner::set_up(context, counters).map(Self)
        }
        /// 委托内部 `FixtureOwner` 执行静止化计数。
        fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
            self.0.quiesce(context)
        }
        /// 先显式释放夹具（触发 `FixtureOwner::drop`），随后返回清理错误以触发 `terminate`。
        fn tear_down(self, _context: &mut TestContext<'_>) -> TestResult {
            drop(self);
            Err(TestError("cleanup failed after fatal return"))
        }
    }
    for (result, primary) in [
        (
            Ok(()),
            Failure::Returned(TestError("expected fatal did not occur")),
        ),
        (
            Err(TestError("body failed before fatal")),
            Failure::Returned(TestError("body failed before fatal")),
        ),
    ] {
        let runtime = Runtime::new();
        let counters = std::sync::Arc::new(Counters {
            quiesces: AtomicUsize::new(0),
            drops: AtomicUsize::new(0),
        });
        let mut context = TestContext::new(&runtime, &EXPECTED_FATAL, 0, 0);
        let terminated = catch_unwind(AssertUnwindSafe(|| {
            ktest::run_case::<CleanupError, _>(&mut context, &counters, |context, _| {
                ktest::expect!(context, false, "independent fatal-return expectation");
                result
            })
        }));
        let outcome = terminated.unwrap_err().downcast::<CaseOutcome>().unwrap();
        assert_eq!(counters.quiesces.load(Ordering::SeqCst), 1);
        assert_eq!(counters.drops.load(Ordering::SeqCst), 1);
        assert_eq!(std::sync::Arc::strong_count(&counters), 1);
        assert_eq!(outcome.status, Status::Error);
        assert_eq!(outcome.primary, Some(primary));
        assert_eq!(
            outcome.cleanup,
            Some(Failure::Returned(TestError(
                "cleanup failed after fatal return"
            )))
        );
        assert_eq!(outcome.expectations, 1);
    }
}

/// 测试目标：验证测试主体内 `assert!` 触发 Panic 时，主体栈上的局部 RAII 变量立即展开析构，而用例夹具保留至 `Quiesce` 和 `Join` 完成后才在 `Cleanup` 阶段析构，且不影响后续用例继续执行。
/// 场景步骤：在测试主体内创建带 `Drop` 计数的局部对象 `Local` 后触发 `assert!(false)`，检查计数器与阶段顺序，随后在同一 `Runtime` 上继续运行下一个空用例。
/// 预期结果：`local_drops`、`quiesces`、`drops` 均为 `1`，五阶段顺序完整，当前用例为 `Status::Fail`（`Failure::Panic`），且紧接着的下一用例正常返回 `Status::Pass`。
#[test]
fn assertion_unwinds_locals_and_preserves_fixture_until_cleanup() {
    let runtime = Runtime::new();
    let counters = std::sync::Arc::new(Counters {
        quiesces: AtomicUsize::new(0),
        drops: AtomicUsize::new(0),
    });
    let local_drops = AtomicUsize::new(0);
    let mut context = TestContext::new(&runtime, &CASE, 7, 0);
    let outcome = ktest::run_case::<FixtureOwner, _>(&mut context, &counters, |_, _| {
        /// 主体栈帧内的局部 RAII 哨兵对象，用于验证栈展开时的析构行为。
        struct Local<'a>(&'a AtomicUsize);
        impl Drop for Local<'_> {
            /// 在主体栈帧展开时递增局部析构计数。
            fn drop(&mut self) {
                self.0.fetch_add(1, Ordering::SeqCst);
            }
        }
        let _local = Local(&local_drops);
        assert!(false, "controlled assertion");
        Ok(())
    });
    assert_eq!(outcome.status, Status::Fail);
    assert!(matches!(outcome.primary, Some(Failure::Panic(_))));
    assert_eq!(local_drops.load(Ordering::SeqCst), 1);
    assert_eq!(counters.quiesces.load(Ordering::SeqCst), 1);
    assert_eq!(counters.drops.load(Ordering::SeqCst), 1);
    assert_eq!(
        *runtime.phases.borrow(),
        [
            Phase::Setup,
            Phase::Body,
            Phase::Quiesce,
            Phase::Join,
            Phase::Cleanup
        ]
    );
    let mut next = TestContext::new(&runtime, &CASE, 8, 0);
    assert_eq!(
        ktest::run_case::<(), _>(&mut next, &(), |_, _| Ok(())).status,
        Status::Pass
    );
}

/// 测试目标：验证当 `Fixture::set_up` 返回 `Err` 时，半构造过程中的局部对象会被正常析构，且框架不会调用测试主体、`quiesce` 或 `tear_down`。
/// 场景步骤：定义 `set_up` 内部创建局部 RAII 对象 `Partial` 后返回 `Err` 的夹具 `Never`，并在 `quiesce`、`tear_down` 和测试主体中设置一旦被调用即崩溃的 `panic!`。
/// 预期结果：`Partial` 析构计数为 `1`，用例状态为 `Status::Error`，阶段轨迹仅包含 `[Phase::Setup, Phase::Join]`，主体与夹具清理方法均未被调用。
#[test]
fn failed_setup_does_not_call_body_or_cleanup() {
    /// `set_up` 总是返回错误的测试夹具。
    struct Never;
    impl Fixture<AtomicUsize> for Never {
        /// 创建局部对象 `_partial` 后返回初始化错误，验证局部资源在 `set_up` 返回前即完成析构。
        fn set_up(_context: &mut TestContext<'_>, drops: &AtomicUsize) -> Result<Self, TestError> {
            /// 记录 `set_up` 内部局部资源释放次数的 RAII 辅助结构。
            struct Partial<'a>(&'a AtomicUsize);
            impl Drop for Partial<'_> {
                /// 在 `set_up` 提前返回错误时递增析构计数。
                fn drop(&mut self) {
                    self.0.fetch_add(1, Ordering::SeqCst);
                }
            }
            let _partial = Partial(drops);
            Err(TestError("setup failed"))
        }
        /// `set_up` 失败后不应被调用的静止化方法。
        fn quiesce(&mut self, _context: &mut TestContext<'_>) -> TestResult {
            panic!("nonexistent fixture quiesced")
        }
        /// `set_up` 失败后不应被调用的销毁方法。
        fn tear_down(self, _context: &mut TestContext<'_>) -> TestResult {
            panic!("nonexistent fixture destroyed")
        }
    }
    let runtime = Runtime::new();
    let mut context = TestContext::new(&runtime, &EXPECTED, 0, 0);
    let drops = AtomicUsize::new(0);
    let outcome = ktest::run_case::<Never, _>(&mut context, &drops, |_, _| {
        panic!("body ran after setup failure")
    });
    assert_eq!(drops.load(Ordering::SeqCst), 1);
    assert_eq!(outcome.status, Status::Error);
    assert_eq!(
        outcome.primary,
        Some(Failure::Returned(TestError("setup failed")))
    );
    assert_eq!(*runtime.phases.borrow(), [Phase::Setup, Phase::Join]);
}

/// 测试目标：验证 `Setup` 阶段发生的 Panic 即使消息匹配用例的 `should_panic` 子串，也会被判定为 `Status::Error`，而不会当作测试主体的预期 Panic 通过。
/// 场景步骤：使用声明了 `should_panic = Some("controlled")` 的 `EXPECTED` 描述符运行在 `set_up` 中触发 `panic!("controlled setup")` 的夹具 `Panicking`。
/// 预期结果：返回 `Status::Error` 且 `primary` 为 `Failure::Panic("controlled setup")`，阶段轨迹仅为 `[Phase::Setup, Phase::Join]`。
#[test]
fn setup_panic_cannot_satisfy_body_expectation() {
    /// 在 `set_up` 阶段直接触发 Panic 的测试夹具。
    struct Panicking;
    impl Fixture<()> for Panicking {
        /// 触发包含 `"controlled"` 前缀的构造期 Panic。
        fn set_up(_context: &mut TestContext<'_>, _resources: &()) -> Result<Self, TestError> {
            panic!("controlled setup");
        }
    }
    let runtime = Runtime::new();
    let mut context = TestContext::new(&runtime, &EXPECTED, 0, 0);
    let outcome =
        ktest::run_case::<Panicking, _>(&mut context, &(), |_, _| panic!("body must not run"));
    assert_eq!(outcome.status, Status::Error);
    assert!(
        matches!(outcome.primary, Some(Failure::Panic(message)) if message.as_str() == "controlled setup")
    );
    assert_eq!(*runtime.phases.borrow(), [Phase::Setup, Phase::Join]);
}

/// 测试目标：验证即使测试主体按预期触发了 `should_panic`，若后台子任务在 `Join` 阶段报告了失败，整个用例仍判定为 `Status::Fail`。
/// 场景步骤：将 `runtime.child` 设为 `Some(Failure::Returned(TestError("child failed")))`，并在主体中触发匹配 `EXPECTED` 的 `panic!("controlled body")`。
/// 预期结果：`outcome.primary` 为 `None`（主体预期满足），`outcome.child` 保留子任务错误，最终 `outcome.status` 为 `Status::Fail`。
#[test]
fn expected_body_panic_does_not_cover_child_failure() {
    let mut runtime = Runtime::new();
    runtime.child = Some(Failure::Returned(TestError("child failed")));
    let mut context = TestContext::new(&runtime, &EXPECTED, 0, 0);
    let outcome = ktest::run_case::<(), _>(&mut context, &(), |_, _| panic!("controlled body"));
    assert_eq!(outcome.status, Status::Fail);
    assert_eq!(outcome.primary, None);
    assert_eq!(outcome.child, runtime.child);
}

/// 测试目标：验证 `context.expect_panic` 可在局部捕获预期 Panic 而不中断主体执行，且后续 `expect!` / `expect_eq` 软断言失败会累加计数并在最终使测试判定为 `Status::Fail`。
/// 场景步骤：在测试主体中依次调用匹配的 `expect_panic`、失败的 `expect!`、失败的 `expect_eq(1, 2)`，并在末尾置位 `reached` 标志。
/// 预期结果：主体完整执行到底（`reached == 1`），`outcome.expectations` 为 `2`，`outcome.primary` 为 `Some(Failure::Expectation)`，最终状态为 `Status::Fail`。
#[test]
fn expectations_continue_and_local_panic_does_not_escape() {
    let runtime = Runtime::new();
    let mut context = TestContext::new(&runtime, &CASE, 0, 0);
    let reached = AtomicUsize::new(0);
    let outcome = ktest::run_case::<(), _>(&mut context, &(), |context, _| {
        context.expect_panic("controlled", || panic!("controlled local"));
        ktest::expect!(context, false, "independent field");
        context.expect_eq(1, 2);
        reached.store(1, Ordering::SeqCst);
        Ok(())
    });
    assert_eq!(reached.load(Ordering::SeqCst), 1);
    assert_eq!(outcome.status, Status::Fail);
    assert_eq!(outcome.primary, Some(Failure::Expectation));
    assert_eq!(outcome.expectations, 2);
}

/// 测试目标：验证 `context.expect_panic` 要求捕获的 Panic 消息未被截断；若消息带有 `truncated == true` 标志，即使已保存的前缀包含期望子串也必须记为软断言失败。
/// 场景步骤：分别以 `truncated = false` 和 `truncated = true` 抛出内容为 `"controlled prefix"` 的 `PanicMessage`，在 `expect_panic("controlled", ...)` 中检查结果。
/// 预期结果：`truncated = false` 时用例通过（`expectations == 0`）；`truncated = true` 时用例失败（`expectations == 1`，`primary == Some(Failure::Expectation)`）。
#[test]
fn local_panic_requires_complete_message() {
    for (truncated, status, primary, expectations) in [
        (false, Status::Pass, None, 0),
        (true, Status::Fail, Some(Failure::Expectation), 1),
    ] {
        let runtime = Runtime::new();
        let mut context = TestContext::new(&runtime, &CASE, 0, 0);
        let mut reached = false;
        let outcome = ktest::run_case::<(), _>(&mut context, &(), |context, _| {
            context.expect_panic("controlled", || {
                panic_any(PanicMessage::new("controlled prefix", truncated))
            });
            reached = true;
            Ok(())
        });
        assert!(reached);
        assert_eq!(outcome.status, status);
        assert_eq!(outcome.primary, primary);
        assert_eq!(outcome.expectations, expectations);
    }
}

/// 测试目标：验证测试主体级别的 `should_panic` 同样要求 Panic 消息完整未截断；被截断的匹配前缀会判定为 `ExpectedPanicMismatch`。
/// 场景步骤：在 `EXPECTED`（期望 `"controlled"`）下分别抛出 `truncated = false` 与 `truncated = true` 的 `"controlled prefix"` 消息。
/// 预期结果：未截断时为 `Status::Pass`；截断时为 `Status::Fail` 且 `primary` 为保留了截断标志的 `Failure::ExpectedPanicMismatch`。
#[test]
fn body_panic_requires_complete_message() {
    for (truncated, status, primary) in [
        (false, Status::Pass, None),
        (
            true,
            Status::Fail,
            Some(Failure::ExpectedPanicMismatch(PanicMessage::new(
                "controlled prefix",
                true,
            ))),
        ),
    ] {
        let runtime = Runtime::new();
        let mut context = TestContext::new(&runtime, &EXPECTED, 0, 0);
        let outcome = ktest::run_case::<(), _>(&mut context, &(), |_, _| {
            panic_any(PanicMessage::new("controlled prefix", truncated))
        });
        assert_eq!(outcome.status, status);
        assert_eq!(outcome.primary, primary);
        assert_eq!(outcome.expectations, 0);
    }
}

/// 测试目标：验证 `should_panic` 用例在“完全未触发 Panic”与“触发了消息不匹配的 Panic”两种情形下分别产生可区分的 `Failure` 变体。
/// 场景步骤：在 `EXPECTED` 描述符下分别运行返回 `Ok(())` 的主体与触发 `panic!("different")` 的主体。
/// 预期结果：前者 `primary` 为 `Some(Failure::ExpectedPanicMissing)`，后者为 `Some(Failure::ExpectedPanicMismatch(_))`。
#[test]
fn expected_panic_missing_and_mismatch_are_distinct() {
    let runtime = Runtime::new();
    let mut missing = TestContext::new(&runtime, &EXPECTED, 0, 0);
    assert_eq!(
        ktest::run_case::<(), _>(&mut missing, &(), |_, _| Ok(())).primary,
        Some(Failure::ExpectedPanicMissing)
    );
    let mut mismatch = TestContext::new(&runtime, &EXPECTED, 0, 0);
    assert!(matches!(
        ktest::run_case::<(), _>(&mut mismatch, &(), |_, _| panic!("different")).primary,
        Some(Failure::ExpectedPanicMismatch(_))
    ));
}

/// 测试目标：验证当测试主体、后台子任务与夹具 `tear_down` 清理阶段同时发生错误时，`terminate` 捕获的 `CaseOutcome` 同时完整保留三处失败原因。
/// 场景步骤：配置 `runtime.child` 预设子任务错误，主体返回 `Err("body failed")`，夹具 `CleanupError::tear_down` 返回 `Err("cleanup failed")`。
/// 预期结果：`terminate` 抛出的 `CaseOutcome` 状态为 `Status::Error`，且 `primary`、`child`、`cleanup` 三个字段分别保留对应的 `TestError`。
#[test]
fn cleanup_error_preserves_original_failures() {
    /// 在 `tear_down` 阶段返回错误的测试夹具。
    struct CleanupError;
    impl Fixture<()> for CleanupError {
        /// 构造无状态夹具实例。
        fn set_up(_context: &mut TestContext<'_>, _resources: &()) -> Result<Self, TestError> {
            Ok(Self)
        }
        /// 消费夹具后返回清理失败错误。
        fn tear_down(self, _context: &mut TestContext<'_>) -> TestResult {
            Err(TestError("cleanup failed"))
        }
    }
    let mut runtime = Runtime::new();
    runtime.child = Some(Failure::Returned(TestError("child failed")));
    let mut context = TestContext::new(&runtime, &CASE, 0, 0);
    let result = catch_unwind(AssertUnwindSafe(|| {
        ktest::run_case::<CleanupError, _>(&mut context, &(), |_, _| Err(TestError("body failed")))
    }));
    let outcome = result.unwrap_err().downcast::<CaseOutcome>().unwrap();
    assert_eq!(outcome.status, Status::Error);
    assert_eq!(
        outcome.primary,
        Some(Failure::Returned(TestError("body failed")))
    );
    assert_eq!(outcome.child, runtime.child);
    assert_eq!(
        outcome.cleanup,
        Some(Failure::Returned(TestError("cleanup failed")))
    );
}

/// 测试目标：验证 `PanicMessage` 在遇到超出 256 字节容量的多字节 UTF-8 字符串时，只会在合法的 UTF-8 字符边界处截断，并且一旦封口后不再追加后续单字节字符。
/// 场景步骤：
/// 1. 构造由 100 个 3 字节汉字 `"界"`（共 300 字节）组成的字符串写入 `PanicMessage`；
/// 2. 构造已有 255 个 `"a"` 的 `PanicMessage`，先尝试追加 3 字节的 `"界"`（空间不足），再尝试追加 1 字节的 `"x"`。
/// 预期结果：
/// 1. 第一个消息保留前 85 个 `"界"`（`85 * 3 = 255` 字节）且 `truncated == true`；
/// 2. 第二个消息在追加 `"界"` 失败后触发封口，后续 `"x"` 不会被写入，内容保持 255 个 `"a"`。
#[test]
fn panic_message_preserves_utf8_boundary() {
    let text = "界".repeat(100);
    let message = PanicMessage::new(&text, false);
    assert_eq!(message.as_str(), "界".repeat(85));
    assert!(message.truncated);
    let mut message = PanicMessage::new(&"a".repeat(255), false);
    core::fmt::Write::write_str(&mut message, "界").unwrap();
    core::fmt::Write::write_str(&mut message, "x").unwrap();
    assert_eq!(message.as_str(), "a".repeat(255));
}
