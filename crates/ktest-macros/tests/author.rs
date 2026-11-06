//! `#[ktest::suite]` 过程宏端到端代码生成与生命周期集成测试。
//!
//! # 模块职责与架构定位
//! 本文件在宿主环境下实际展开 `#[ktest_macros::suite]` 模块，验证生成的静态套件描述符（`__KTEST_SUITE`）、
//! 套件级与用例级元数据合并规则、两级夹具（`suite_resources` 与 `fixture`）的构造与销毁顺序，以及多种合法测试函数签名的运行行为。

use std::cell::RefCell;
use std::fmt;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::rc::Rc;

use ktest::{
    CaseDesc, CaseOutcome, CaseRunner, Caught, Context, Failure, Fixture, Isolation, JoinOutcome,
    PanicMessage, Phase, Resource, Services, Status, TestContext, TestError, TestResult,
};

/// 包含套件级共享资源（`Base`）、用例级独立夹具（`Device`）及参数化测试实例的示例测试套件。
#[ktest_macros::suite(context = "Boot", fixture = Device, suite_resources = Base, tags("block"), min_cpus = 2, timeout_ms = 500, resources("scratch"))]
mod typed {
    use super::*;

    /// 套件级共享资源：内部持有引用计数的事件记录数组，用于收集各用例实例在 `quiesce` 阶段写入的值。
    pub struct Base(Rc<RefCell<Vec<u32>>>);
    impl Fixture<()> for Base {
        /// 在套件开始执行时创建空的共享结果记录容器。
        fn set_up(_context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
            Ok(Self(Rc::new(RefCell::new(Vec::new()))))
        }
        /// 在套件内所有选中的参数实例执行并清理完毕后，断言两个实例均已按序完成了各自的 `quiesce` 回调。
        fn tear_down(self, _context: &mut TestContext<'_>) -> TestResult {
            assert_eq!(*self.0.borrow(), [1, 2]);
            Ok(())
        }
    }
    /// 用例级独立夹具：每个参数实例各自持有一份独立的 `Device` 实例。
    pub struct Device {
        /// 指向套件级共享记录容器的引用计数句柄。
        owner: Rc<RefCell<Vec<u32>>>,
        /// 当前用例实例写入的数值状态（初始为 0，验证实例间互不干扰）。
        value: u32,
    }
    impl Fixture<Base> for Device {
        /// 为当前用例实例克隆共享容器句柄，并将实例局部状态 `value` 初始化为 0。
        fn set_up(_context: &mut TestContext<'_>, base: &Base) -> Result<Self, TestError> {
            Ok(Self {
                owner: base.0.clone(),
                value: 0,
            })
        }
        /// 在用例主体执行结束后，将本实例写入的 `value` 追加到套件共享记录容器中。
        fn quiesce(&mut self, _context: &mut TestContext<'_>) -> TestResult {
            self.owner.borrow_mut().push(self.value);
            Ok(())
        }
    }

    /// 测试目标：验证参数化测试的每个 `#[case]` 实例均拥有全新初始化的 `Device` 夹具与独立的随机种子。
    /// 场景步骤：断言 `device.value` 初始为 `0` 且 `context.seed` 与实例序号对应，随后将 `value` 写入 `device.value` 供 `quiesce` 阶段记录。
    /// 预期结果：两个参数实例（`"one"` 与 `"two"`）各自独立通过，互不污染状态。
    #[test]
    #[case("one", 1)]
    #[case("two", 2)]
    #[ktest(tags("smp"), timeout_ms = 70, resources("uart"))]
    fn parameter(context: &mut TestContext<'_>, device: &mut Device, value: u32) -> TestResult {
        assert_eq!(device.value, 0);
        assert_eq!(context.seed, 100 + value as u64);
        device.value = value;
        Ok(())
    }
}

/// 覆盖无参函数、独立静态参数、局部上下文捕获、`#[should_panic]` 与 `#[ignore]` 元数据的基础测试套件。
#[ktest_macros::suite]
mod simple {
    /// 测试目标：验证无参数且默认返回 `()` 的基础测试函数可正常展开并执行标准 `assert_eq!`。
    #[test]
    fn plain() {
        assert_eq!(2 + 2, 4);
    }

    /// 测试目标：验证不声明 `TestContext` 或 `Fixture` 时，测试函数仍可单独接收 `#[case]` 静态参数。
    #[test]
    #[case("zero", 0)]
    fn static_parameter(value: usize) {
        assert_eq!(value, 0);
    }

    /// 测试目标：验证仅接收 `&mut TestContext` 且显式标注 `-> ()` 返回类型的函数签名可正常调用 `expect_panic`。
    #[test]
    fn contextual(context: &mut ktest::TestContext<'_>) -> () {
        context.expect_panic("local", || panic!("local"));
    }

    /// 测试目标：验证标注 `#[should_panic(expected = "...")]` 的测试函数在触发匹配 Panic 时被判定为通过。
    #[test]
    #[should_panic(expected = "controlled")]
    fn expected() {
        panic!("controlled");
    }

    /// 测试目标：验证同时标注 `#[ignore]` 与 `expected_fatal` 的测试函数在未执行时仍完整保留静态元数据并将隔离级别提升为 `Case`。
    #[test]
    #[ignore = "requires destructive VM"]
    #[ktest(expected_fatal = "panic", fatal_message = "fatal diagnostic")]
    fn fatal_metadata() {
        panic!("fatal diagnostic");
    }
}

/// 宿主测试环境下用于驱动 `ktest` 生命周期的最小 `Services` 实现。
struct Runtime {
    /// 收集测试运行期间通过 `Services::diagnostic` 输出的诊断日志列表。
    diagnostics: RefCell<Vec<String>>,
}
impl Services for Runtime {
    /// 使用宿主标准库的 `std::panic::catch_unwind` 捕获测试回调中的 Panic 并提取消息字符串。
    fn catch(&self, callback: &mut dyn FnMut() -> TestResult) -> Caught {
        match catch_unwind(AssertUnwindSafe(callback)) {
            Ok(result) => Caught::Returned(result),
            Err(payload) => {
                let message = payload
                    .downcast_ref::<&str>()
                    .copied()
                    .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
                    .unwrap_or("non-string panic");
                Caught::Panicked(PanicMessage::new(message, false))
            }
        }
    }
    /// 将格式化诊断消息追加到内存日志列表中。
    fn diagnostic(&self, message: fmt::Arguments<'_>) {
        self.diagnostics.borrow_mut().push(message.to_string());
    }
    /// 忽略生命周期阶段切换通知。
    fn phase(&self, _phase: Phase) {}
    /// 本测试无后台子任务，无需发出停止信号。
    fn stop_children(&self) {}
    /// 本测试无后台子任务，直接返回无子任务失败的 `JoinOutcome`。
    fn join_children(&self) -> Result<JoinOutcome, TestError> {
        Ok(JoinOutcome { failure: None })
    }
    /// 清理阶段若意外触发终止，直接通过 Panic 使外层宿主测试失败。
    fn terminate(&self, outcome: &CaseOutcome) -> ! {
        panic!("unexpected termination: {:?}", outcome)
    }
}
/// 按指定索引列表顺序执行套件内用例的宿主 `CaseRunner` 实现。
struct Runner<'a> {
    /// 底层宿主运行时服务引用。
    runtime: &'a Runtime,
    /// 待执行的用例在套件 `cases` 表中的下标列表。
    selected: &'a [usize],
    /// 收集每个已执行用例的返回结果。
    outcomes: Vec<CaseOutcome>,
}
impl CaseRunner for Runner<'_> {
    /// 遍历 `selected` 中的用例索引，为每个实例创建带有确定性种子的 `TestContext` 并调用 `dispatch` 执行。
    fn run(
        &mut self,
        cases: &'static [CaseDesc],
        dispatch: &mut dyn FnMut(&mut TestContext<'_>, usize) -> CaseOutcome,
    ) -> TestResult {
        for &index in self.selected {
            let mut context = TestContext::new(self.runtime, &cases[index], 101 + index as u64, 0);
            self.outcomes.push(dispatch(&mut context, index));
        }
        Ok(())
    }
}

/// 测试目标：验证带 `suite_resources`、`fixture` 与 `#[case]` 的套件在宏展开后能正确合并静态属性，并为每个参数实例独立构造和清理夹具。
/// 场景步骤：检查 `typed::__KTEST_SUITE.cases` 的合并属性（`context`、`tags`、`resources`、`min_cpus`、`timeout_ms`、全名与源文件名），随后调用 `run` 依次执行索引 `0` 和 `1`。
/// 预期结果：套件初始化、执行与清理均无错误，两个参数实例状态均为 `Status::Pass`，且 `Base::tear_down` 验证了两个 `Device` 实例的 `quiesce` 均按序执行。
#[test]
fn typed_parameter_instances_have_fresh_fixture_and_effective_metadata() {
    let cases = typed::__KTEST_SUITE.cases;
    assert_eq!(
        cases.iter().map(|case| case.parameter).collect::<Vec<_>>(),
        [Some("one"), Some("two")]
    );
    for case in cases {
        assert_eq!(case.attributes.context, Context::Boot);
        assert_eq!(case.attributes.tags, ["block", "smp"]);
        assert_eq!(
            case.attributes.resources,
            [Resource::Scratch, Resource::Uart]
        );
        assert_eq!(case.attributes.min_cpus, 2);
        assert_eq!(case.attributes.timeout_ms, 70);
        assert!(case
            .name
            .starts_with(concat!(module_path!(), "::typed::parameter[")));
        assert!(case.source.ends_with("author.rs"));
    }
    let runtime = Runtime {
        diagnostics: RefCell::new(Vec::new()),
    };
    let mut suite_context = TestContext::new(&runtime, &cases[0], 0, 0);
    let mut runner = Runner {
        runtime: &runtime,
        selected: &[0, 1],
        outcomes: Vec::new(),
    };
    let result = (typed::__KTEST_SUITE.run)(&mut suite_context, &mut runner);
    assert_eq!(result.setup, None);
    assert_eq!(result.execution, None);
    assert_eq!(result.cleanup, None);
    assert_eq!(
        runner
            .outcomes
            .iter()
            .map(|value| value.status)
            .collect::<Vec<_>>(),
        [Status::Pass, Status::Pass]
    );
}

/// 测试目标：验证无参测试、单参数测试、`TestContext` 测试、`#[should_panic]` 测试以及 `#[ignore]`/`expected_fatal` 元数据在展开后行为正确。
/// 场景步骤：通过 `Runner` 执行 `simple` 套件的前 4 个用例（索引 `0..=3`），并检查第 5 个用例（`fatal_metadata`）的静态描述符字段。
/// 预期结果：前 4 个用例全部返回 `Status::Pass`（其中 `expected` 用例的受控 Panic 被正确消化），第 5 个用例的隔离级别自动提升为 `Isolation::Case` 且完整保留 `expected_fatal`、`fatal_message` 与 `ignore` 信息。
#[test]
fn simple_author_signatures_and_expected_panic_execute() {
    let cases = simple::__KTEST_SUITE.cases;
    let runtime = Runtime {
        diagnostics: RefCell::new(Vec::new()),
    };
    let mut context = TestContext::new(&runtime, &cases[0], 0, 0);
    let mut runner = Runner {
        runtime: &runtime,
        selected: &[0, 1, 2, 3],
        outcomes: Vec::new(),
    };
    let result = (simple::__KTEST_SUITE.run)(&mut context, &mut runner);
    assert_eq!(result.execution, None);
    assert_eq!(
        runner
            .outcomes
            .iter()
            .map(|value| value.status)
            .collect::<Vec<_>>(),
        [Status::Pass; 4]
    );
    let fatal = &cases[4];
    assert_eq!(fatal.attributes.isolation, Isolation::Case);
    assert_eq!(fatal.expected_fatal, Some("panic"));
    assert_eq!(fatal.fatal_message, Some("fatal diagnostic"));
    assert_eq!(fatal.ignore, Some("requires destructive VM"));
    assert_eq!(cases[0].attributes.context, Context::Thread);
    assert!(cases[3].should_panic.is_some());
    assert!(!runner
        .outcomes
        .iter()
        .any(|result| matches!(result.primary, Some(Failure::Panic(_)))));
}
