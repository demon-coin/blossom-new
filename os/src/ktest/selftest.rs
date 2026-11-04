//! 两种真实执行栈的展开纵线；负例单独选择，不把框架失败伪装成通过。

use core::cell::Cell;
use core::fmt;

use ::ktest::{Fixture, TestContext, TestError, TestResult};

use crate::ktest::panic::{catch, is_unwinding};
use crate::sync::{IrqSaveGuard, PreemptGuard, SpinLock};

/// 显式根只引用真实 suite 注册槽，不维护第二份 case 清单。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 8] {
    [
        boot::__ktest_anchor(),
        thread::__ktest_anchor(),
        children::__ktest_anchor(),
        child_failures::__ktest_anchor(),
        failures::__ktest_anchor(),
        setup_failure::__ktest_anchor(),
        cleanup_failure::__ktest_anchor(),
        fatal::__ktest_anchor(),
    ]
}

/// 每例重新创建的局部锁和析构账本，不依赖上一个实例的状态。
struct LocalFixture {
    lock: SpinLock<usize>,
    drops: [Cell<usize>; 3],
    unwinding_drops: Cell<usize>,
}

impl Fixture for LocalFixture {
    /// Boot 阶段仅建立栈内状态，不取得页、堆、磁盘或任务。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            lock: SpinLock::new(0),
            drops: [Cell::new(0), Cell::new(0), Cell::new(0)],
            unwinding_drops: Cell::new(0),
        })
    }
}

/// 正常 Boot suite 的明确一次性基础对象，只使用栈内状态。
struct NoHeapBase {
    token: usize,
}
impl Fixture for NoHeapBase {
    /// suite setup 自己也经真实窄捕获执行，诊断由父核对每 VM 恰出现一次。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        context.diagnostic(format_args!("vertical Boot suite setup once"));
        Ok(Self {
            token: catch(|| 13).unwrap(),
        })
    }
    /// 最后一例收敛后消费基础对象，不重置任何内核全局状态。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert_eq!(self.token, 13);
        context.diagnostic(format_args!("vertical Boot suite cleanup once"));
        Ok(())
    }
}
impl Fixture<NoHeapBase> for LocalFixture {
    /// 每例验证基础对象可用，再独立创建局部锁与析构账本。
    fn set_up(context: &mut TestContext<'_>, base: &NoHeapBase) -> Result<Self, TestError> {
        assert_eq!(base.token, 13);
        <Self as Fixture<()>>::set_up(context, &())
    }
}

struct DropMark<'a> {
    count: &'a Cell<usize>,
    unwinding: &'a Cell<usize>,
}

impl Drop for DropMark<'_> {
    /// 只记录真实析构和真实展开状态，绝不在展开期间调度或分配。
    fn drop(&mut self) {
        self.count.set(self.count.get() + 1);
        if is_unwinding() {
            self.unwinding.set(self.unwinding.get() + 1);
        }
    }
}

/// 保留三个独立 Rust 栈帧，让 landing pad 实际执行每层局部析构。
#[inline(never)]
fn drop_level(fixture: &LocalFixture, level: usize) {
    let _local = DropMark {
        count: &fixture.drops[level],
        unwinding: &fixture.unwinding_drops,
    };
    if level == 0 {
        panic!("vertical multilayer drop");
    }
    drop_level(fixture, level - 1);
}

/// 在同一实例内验证嵌套与连续异常均归还各自边界。
fn nested_and_sequential(context: &mut TestContext<'_>) {
    context.expect_panic("vertical outer", || {
        let inner = catch(|| panic!("vertical inner")).unwrap_err();
        assert!(inner.matches_message("vertical inner"));
        assert!(!is_unwinding());
        panic!("vertical outer");
    });
    for message in ["vertical first", "vertical second"] {
        let result = catch(|| panic!("{}", message)).unwrap_err();
        assert!(result.matches_message(message));
        assert!(!is_unwinding());
    }
    assert_eq!(catch(|| 37).unwrap(), 37);
}

/// 观察关 IRQ 锁的真实释放，再次取得同一把锁；不回滚主体已经发生的写入。
fn lock_and_irq(context: &mut TestContext<'_>, fixture: &LocalFixture, initially_off: bool) {
    let _preempt = PreemptGuard::new();
    let _outer_irq = initially_off.then(IrqSaveGuard::new);
    let before = crate::arch::irq_enabled();
    context.expect_panic("vertical locked panic", || {
        let mut guard = fixture.lock.lock();
        assert!(!crate::arch::irq_enabled());
        *guard = 17;
        panic!("vertical locked panic");
    });
    assert_eq!(crate::arch::irq_enabled(), before);
    let mut guard = fixture
        .lock
        .try_lock()
        .expect("unwind leaked original lock");
    assert_eq!(*guard, 17);
    *guard = 29;
    drop(guard);
    assert_eq!(crate::arch::irq_enabled(), before);
    assert_eq!(
        *fixture
            .lock
            .try_lock()
            .expect("second lock acquisition failed"),
        29
    );
}

/// 仅读取实际 FP 开关，不为测量而开启 FP 或改动任务的控制状态。
fn fp_control() -> usize {
    let value: usize;
    #[cfg(target_arch = "riscv64")]
    // SAFETY: 当前为内核 S-mode；读 sstatus 不访问 FP，不修改 IRQ 或任何寄存器状态。
    unsafe {
        core::arch::asm!("csrr {value}, sstatus", value = out(reg) value, options(nomem, nostack));
        value & crate::arch::SR_FS
    }
    #[cfg(target_arch = "loongarch64")]
    // SAFETY: 当前为 PLV0；仅读 EUEN.FPE，不访问 FCSR/FP，不修改控制状态。
    unsafe {
        core::arch::asm!("csrrd {value}, {euen}", value = out(reg) value,
            euen = const crate::arch::CSR_EUEN, options(nomem, nostack));
        value & crate::arch::EUEN_FP
    }
}

/// FP Off 主体经过真实 panic 后仍保持 Off，控制读取期间不得迁核。
fn fp_off_restore() {
    let _preempt = PreemptGuard::new();
    let before = fp_control();
    assert_eq!(before, 0, "ordinary kernel body must enter with FP Off");
    let record = catch(|| panic!("vertical FP Off")).unwrap_err();
    assert!(record.matches_message("vertical FP Off"));
    assert_eq!(fp_control(), before);
}

struct LongMessage;
impl fmt::Display for LongMessage {
    /// 用多段多字节 UTF-8 触发固定消息缓冲边界，不在格式化器中分配。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("vertical-long:")?;
        for _ in 0..200 {
            formatter.write_str("界")?;
        }
        formatter.write_str("must-not-replace-prefix")
    }
}

/// 截断保留原 UTF-8 前缀，不能把它当成完整预期消息。
fn truncated_message() {
    let record = catch(|| panic!("{}", LongMessage)).unwrap_err();
    assert!(record.message_truncated());
    assert!(record.message().starts_with("vertical-long:"));
    assert!(record.message().ends_with(" [truncated]"));
    assert!(!record.message().contains("must-not-replace-prefix"));
    assert!(!record.matches_message(record.message()));
    assert!(record.file().ends_with("ktest/selftest.rs"));
    assert!(record.line != 0);
}

#[::ktest::suite(context = "Boot", fixture = LocalFixture, suite_resources = NoHeapBase, tags("selftest", "selftest-normal", "noheap"))]
mod boot {
    use super::*;

    /// 具名参数覆盖零、普通数与 usize 顶界，表达式在主体捕获内执行。
    #[test]
    #[case("zero", 0usize, Some(1usize))]
    #[case("one", 1usize, Some(2usize))]
    #[case("max", usize::MAX, None)]
    fn boundary(_: &mut TestContext<'_>, _: &mut LocalFixture, value: usize, next: Option<usize>) {
        assert_eq!(value.checked_add(1), next);
    }

    /// 宿主 hashlib 给出的独立期望覆盖 SHA 填充界限、多字节和最大十进制输入。
    #[test]
    #[case("empty", 0u64, 0u32, "", 0x390feabc786e369eu64)]
    #[case(
        "pad56",
        0u64,
        0u32,
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        0x69f7d75320a9179au64
    )]
    #[case(
        "multiblock_unicode",
        u64::MAX,
        u32::MAX,
        "界界界界界界界界界界界界界界界界界界界界界界界界界",
        0x975cb9042ebd696bu64
    )]
    #[case(
        "full_name",
        1234u64,
        0u32,
        "os::ktest::selftest::thread::migrate_then_panic",
        0xbd4aaff5028002c5u64
    )]
    fn seed_hash(
        _: &mut TestContext<'_>,
        _: &mut LocalFixture,
        seed: u64,
        repeat: u32,
        name: &str,
        expected: u64,
    ) {
        assert_eq!(
            crate::ktest::runner::case_seed(seed, repeat, name),
            expected
        );
    }

    /// 普通主体 panic 由正式 should_panic 消息合同判断。
    #[test]
    #[should_panic(expected = "vertical expected Boot")]
    fn expected(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        panic!("vertical expected Boot");
    }

    /// setup 后的嵌套与连续捕获仍完全不使用堆。
    #[test]
    fn nested(context: &mut TestContext<'_>, _: &mut LocalFixture) {
        nested_and_sequential(context);
    }

    /// 三层局部对象必须在展开中恰释放一次。
    #[test]
    fn drops(context: &mut TestContext<'_>, fixture: &mut LocalFixture) {
        context.expect_panic("vertical multilayer drop", || drop_level(fixture, 2));
        assert_eq!(fixture.drops.each_ref().map(Cell::get), [1, 1, 1]);
        assert_eq!(fixture.unwinding_drops.get(), 3);
    }

    /// 原 IRQ 状态和原锁分别观察，不手工解锁或恢复异常前的数据。
    #[test]
    #[case("entry", false)]
    #[case("off", true)]
    fn lock_irq(context: &mut TestContext<'_>, fixture: &mut LocalFixture, off: bool) {
        lock_and_irq(context, fixture, off);
    }

    /// MM 之前也能恢复 FP Off 控制位。
    #[test]
    fn fp_off(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        fp_off_restore();
    }

    /// 全部 callee-saved GPR/FP、SP、per-CPU 基址和 FCSR 经真实 catch 恢复。
    #[test]
    fn registers(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        assert!(crate::ktest::selftest_register_roundtrip());
    }

    /// 分段 UTF-8 消息的截断不依赖 alloc。
    #[test]
    fn truncation(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        truncated_message();
    }
}

#[::ktest::suite(context = "Thread", fixture = LocalFixture, tags("selftest", "selftest-normal", "thread"))]
mod thread {
    use super::*;

    /// 同短名字在另一个 suite 具有不同的真实 module_path 全名。
    #[test]
    #[case("zero", 0usize, Some(1usize))]
    #[case("max", usize::MAX, None)]
    fn boundary(_: &mut TestContext<'_>, _: &mut LocalFixture, value: usize, next: Option<usize>) {
        assert_eq!(value.checked_add(1), next);
    }

    /// Task 栈上的主体预期 panic 不传播到 suite 调度入口。
    #[test]
    #[should_panic(expected = "vertical expected Thread")]
    fn expected(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        panic!("vertical expected Thread");
    }

    /// 当前真实 Task 同步执行所有嵌套和连续边界。
    #[test]
    fn nested(context: &mut TestContext<'_>, _: &mut LocalFixture) {
        nested_and_sequential(context);
    }

    /// Task 上的多层 landing pad 与 Boot 相同地逐层析构。
    #[test]
    fn drops(context: &mut TestContext<'_>, fixture: &mut LocalFixture) {
        context.expect_panic("vertical multilayer drop", || drop_level(fixture, 2));
        assert_eq!(fixture.drops.each_ref().map(Cell::get), [1, 1, 1]);
        assert_eq!(fixture.unwinding_drops.get(), 3);
    }

    /// 真实 Any Task 定向迁核后仍按原边界展开，不能恢复旧 CPU 的基址。
    #[test]
    #[ktest(min_cpus = 2, tags("smp"))]
    fn migrate_then_panic(context: &mut TestContext<'_>, fixture: &mut LocalFixture) {
        let task = crate::task::current_task_id().expect("migration requires a Task");
        let origin = crate::cpu::current().id;
        let target = (origin + 1) % crate::cpu::count();
        let outer = crate::ktest::current_boundary();
        context.expect_panic("vertical multilayer drop", || {
            let boundary = crate::ktest::current_boundary();
            crate::sched::test_migrate_to(target);
            assert_eq!(crate::cpu::current().id, target);
            assert_eq!(crate::task::current_task_id(), Some(task));
            assert_eq!(crate::ktest::current_boundary(), boundary);
            assert!(!is_unwinding());
            drop_level(fixture, 2);
        });
        assert_eq!(crate::task::current_task_id(), Some(task));
        assert_eq!(crate::ktest::current_boundary(), outer);
        assert!(!is_unwinding());
        assert_eq!(fixture.drops.each_ref().map(Cell::get), [1, 1, 1]);
        assert_eq!(fixture.unwinding_drops.get(), 3);
        let record = catch(|| panic!("vertical after migration")).unwrap_err();
        assert!(record.matches_message("vertical after migration"));
        assert!(!is_unwinding());
    }

    /// 可抢占 Task 在临时不可迁核段内观察真实锁和 IRQ 复原。
    #[test]
    #[case("entry", false)]
    #[case("off", true)]
    fn lock_irq(context: &mut TestContext<'_>, fixture: &mut LocalFixture, off: bool) {
        lock_and_irq(context, fixture, off);
    }

    /// 展开临时启用 FP 后必须归还当前任务的 Off 状态。
    #[test]
    fn fp_off(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        fp_off_restore();
    }

    /// 真实 Task 栈上的 ABI 哨兵不依赖 timer trap 或纸面寄存器值。
    #[test]
    fn registers(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        assert!(crate::ktest::selftest_register_roundtrip());
    }

    /// 低层记录与 Task 捕获边界各自保留固定消息。
    #[test]
    fn truncation(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        truncated_message();
    }
}

#[::ktest::suite(
    context = "Thread",
    tags("selftest", "selftest-failure"),
    ignore = "intentional framework failures; select explicitly"
)]
mod failures {
    use super::*;

    /// 失败期望不得提前返回；后续断言和诊断必须真实执行。
    #[test]
    fn expect_continues(context: &mut TestContext<'_>) {
        let fixture = <LocalFixture as Fixture<()>>::set_up(context, &()).unwrap();
        ::ktest::expect!(context, false, "vertical intentional expectation");
        let record = catch(|| drop_level(&fixture, 2)).unwrap_err();
        assert!(record.matches_message("vertical multilayer drop"));
        assert_eq!(fixture.drops.each_ref().map(Cell::get), [1, 1, 1]);
        context.diagnostic(format_args!("vertical after failed expectation reached"));
    }

    /// 显式 Result Err 应作为主体失败保留原消息。
    #[test]
    fn result_error() -> TestResult {
        Err(TestError("vertical explicit result error"))
    }

    /// 普通 assert 失败必须归属本例，不能污染后续独立 Pass 实例。
    #[test]
    fn assertion_failure() {
        assert!(false, "vertical intentional assertion");
    }

    /// 匹配主体 panic 也不能抹去此前独立的 expect 失败。
    #[test]
    #[should_panic(expected = "vertical matching after expectation")]
    fn expect_then_expected(context: &mut TestContext<'_>) {
        ::ktest::expect!(context, false, "vertical independent expectation");
        panic!("vertical matching after expectation");
    }

    /// 没有发生 panic 时 should_panic 不得返回 Pass。
    #[test]
    #[should_panic(expected = "vertical missing")]
    fn missing_panic() {}

    /// 错误消息不能满足 should_panic。
    #[test]
    #[should_panic(expected = "vertical wanted")]
    fn mismatched_panic() {
        panic!("vertical different");
    }

    /// 即使实际消息包含期望前缀，截断记录也不能满足完整预期。
    #[test]
    #[should_panic(expected = "vertical-long")]
    fn truncated_prefix() {
        panic!("{}", LongMessage);
    }

    /// 局部 expect_panic 的缺失或消息不符仍继续主体并记 Fail。
    #[test]
    #[case("missing", false)]
    #[case("mismatch", true)]
    fn local_expectation(context: &mut TestContext<'_>, panic_now: bool) {
        context.expect_panic("vertical wanted", || {
            if panic_now {
                panic!("vertical different");
            }
        });
        assert_eq!(catch(|| 41).unwrap(), 41);
        context.diagnostic(format_args!(
            "vertical after failed local panic expectation reached"
        ));
    }

    /// 与负例同批选择时仍须独立得到真实 Pass，不读负例的状态。
    #[test]
    fn after_failure() {
        let value = catch(|| 53).unwrap();
        assert_eq!(value, 53);
        assert!(!is_unwinding());
    }
}

/// 每次 suite 执行都创建新的部分构造账本，只汇总本次被选择的尝试。
struct SetupEvidence {
    attempts: Cell<usize>,
    drops: Cell<usize>,
}
impl Fixture for SetupEvidence {
    /// 基础账本仍是无堆栈对象。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            attempts: Cell::new(0),
            drops: Cell::new(0),
        })
    }
    /// 每次尝试必须释放其局部对象；选择任意子集都不依赖其它实例。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert_eq!(self.drops.get(), self.attempts.get());
        context.diagnostic(format_args!(
            "vertical partial setup drops={} attempts={}",
            self.drops.get(),
            self.attempts.get()
        ));
        Ok(())
    }
}
struct PartialSetup<'a>(&'a Cell<usize>);
impl Drop for PartialSetup<'_> {
    /// Err 返回与真实展开都由正常 Rust 析构释放局部部分对象。
    fn drop(&mut self) {
        self.0.set(self.0.get() + 1);
    }
}
struct RejectedFixture;
impl Fixture<SetupEvidence> for RejectedFixture {
    /// 只返回真实 Err 或真实 panic，从不发表半构造 fixture。
    fn set_up(context: &mut TestContext<'_>, base: &SetupEvidence) -> Result<Self, TestError> {
        base.attempts.set(base.attempts.get() + 1);
        let _partial = PartialSetup(&base.drops);
        if context.case.parameter == Some("panic") {
            panic!("vertical partial setup panic");
        }
        Err(TestError("vertical partial setup error"))
    }
}

#[::ktest::suite(context = "Boot", fixture = RejectedFixture, suite_resources = SetupEvidence, tags("selftest", "selftest-failure", "noheap"), ignore = "intentional setup Error; body must not run")]
mod setup_failure {
    use super::*;

    /// 未完成 fixture 构造必须报告 Error、释放局部对象且不进入 body 阶段。
    #[test]
    #[case("error", ())]
    #[case("panic", ())]
    #[should_panic(expected = "vertical partial setup panic")]
    fn partial(_: &mut TestContext<'_>, _: &mut RejectedFixture, _: ()) {
        panic!("vertical setup failure body unexpectedly ran");
    }
}

struct CleanupError {
    in_quiesce: bool,
    panics: bool,
}
impl Fixture for CleanupError {
    /// 无异步资源的具体对象按参数选择真实清理阶段和失败形式。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            in_quiesce: matches!(context.case.parameter, Some("quiesce" | "quiesce_panic")),
            panics: matches!(
                context.case.parameter,
                Some("quiesce_panic" | "tear_down_panic")
            ),
        })
    }
    /// 清理失败由框架 terminate 结束 VM，不在用例中伪造 fatal。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        if self.in_quiesce {
            if self.panics {
                panic!("vertical cleanup panic");
            }
            Err(TestError("vertical quiesce error"))
        } else {
            Ok(())
        }
    }
    /// 消费函数同样保留主体错误并终止，不把清理错误压成普通 Pass。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        if self.panics {
            panic!("vertical cleanup panic");
        }
        Err(TestError("vertical tear_down error"))
    }
}

#[::ktest::suite(context = "Thread", fixture = CleanupError, isolation = "Case", tags("selftest", "selftest-failure"), ignore = "intentional cleanup Error and VM termination; select one parameter")]
mod cleanup_failure {
    use super::*;

    /// 主体原始 Err 与后续清理错误必须同时出现在框架真实错误账本。
    #[test]
    #[case("quiesce", ())]
    #[case("tear_down", ())]
    fn primary_and_cleanup(_: &mut TestContext<'_>, _: &mut CleanupError, _: ()) -> TestResult {
        Err(TestError("vertical primary before cleanup error"))
    }

    /// 子任务先真实离栈，再由框架 join 保留主体、子失败与消费清理错误三份账本。
    #[test]
    #[case("tear_down", ())]
    fn primary_child_cleanup(_: &mut TestContext<'_>, _: &mut CleanupError, _: ()) -> TestResult {
        let handle =
            crate::ktest::spawn_on(0, |_| Err(TestError("vertical child before cleanup error")))?;
        while handle.try_join().is_none() {
            crate::sched::yield_now();
        }
        Err(TestError("vertical primary before child cleanup error"))
    }

    /// 主体缺失 panic，不能由清理阶段的相同预期消息补成成功。
    #[test]
    #[case("quiesce_panic", ())]
    #[case("tear_down_panic", ())]
    #[should_panic(expected = "vertical cleanup panic")]
    fn cleanup_panic_is_not_body_panic(context: &mut TestContext<'_>, _: &mut CleanupError, _: ()) {
        context.diagnostic(format_args!("vertical body returned before cleanup panic"));
    }
}

struct PanicOnDrop;
impl Drop for PanicOnDrop {
    /// 真实 landing pad 中的第二次 panic 必须进入底层 double-panic 控制者。
    fn drop(&mut self) {
        panic!("vertical second panic in Drop");
    }
}

/// 普通 C ABI 不允许 Rust 异常越过此边界，PanicInfo 必须报告不可展开。
#[inline(never)]
extern "C" fn panic_across_abi() {
    panic!("vertical non-unwind C ABI");
}

#[::ktest::suite(
    context = "Thread",
    isolation = "Case",
    tags("selftest", "selftest-fatal"),
    ignore = "destructive isolated VM selftest; select explicitly"
)]
mod fatal {
    use super::*;

    /// 正常 arm 后，真实第二次 panic 由 low-level panic 控制者确认类型。
    #[test]
    #[ktest(
        expected_fatal = "double-panic",
        fatal_message = "double panic during unwind"
    )]
    fn double_panic() {
        let _local = PanicOnDrop;
        panic!("vertical first panic before Drop");
    }

    /// 调用真实非 unwind ABI，而不是用例自行宣称发生 ABI fatal。
    #[test]
    #[ktest(
        expected_fatal = "abi",
        fatal_message = "panic cannot unwind across ABI boundary"
    )]
    fn abi() {
        panic_across_abi();
    }

    /// 同核同步动作也必须守真实不可展开边界，不能借原Task捕获槽返回主体。
    #[test]
    #[ktest(
        expected_fatal = "panic",
        fatal_message = "vertical local synchronous call panic"
    )]
    fn local_sync_call() {
        /// 回调不访问参数或调度，唯一故障是原同步分派中的真实panic。
        unsafe fn fail(_: *mut ()) {
            panic!("vertical local synchronous call panic");
        }
        // SAFETY: 当前在线CPU有效；回调不使用空参数、不睡眠或递归调用，未持callback锁。
        unsafe { crate::cpu::call(crate::cpu::current().id, fail, core::ptr::null_mut()) };
    }

    /// 只在显式 abort 策略 VM 选择；unwind 策略中这是普通被捕获的失败。
    #[test]
    #[ktest(expected_fatal = "panic", fatal_message = "vertical real abort panic")]
    fn abort_panic() {
        panic!("vertical real abort panic");
    }
}

/// 每例只分配一个拥有型状态，分别记录实际启动 CPU 和局部对象释放。
struct ChildState {
    drops: core::sync::atomic::AtomicUsize,
    entered: crate::cpu::AtomicCpuSet,
    finished: crate::cpu::AtomicCpuSet,
}
struct ChildFixture {
    state: alloc::sync::Arc<ChildState>,
    started: usize,
    cpu_mask: crate::cpu::CpuSet,
}
impl Fixture for ChildFixture {
    /// 只在 Thread 例申请拥有型状态，分配失败返回真实 TestError。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            state: alloc::sync::Arc::try_new(ChildState {
                drops: core::sync::atomic::AtomicUsize::new(0),
                entered: crate::cpu::AtomicCpuSet::new(),
                finished: crate::cpu::AtomicCpuSet::new(),
            })
            .map_err(|_| TestError("NoMemory: owned child state"))?,
            started: 0,
            cpu_mask: crate::cpu::CpuSet::empty(),
        })
    }
    /// 父框架完成合作停止和真实离栈 join 后，才观察子局部对象的实际析构。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        use core::sync::atomic::Ordering;
        assert_eq!(self.state.drops.load(Ordering::Acquire), self.started);
        assert_eq!(self.state.finished.load(Ordering::Acquire), self.cpu_mask);
        Ok(())
    }
}
struct OwnedChildDrop {
    state: alloc::sync::Arc<ChildState>,
    cpu: usize,
}
impl Drop for OwnedChildDrop {
    /// 普通返回或展开都恰记录一次局部对象释放，绝不在 Drop 中等待或调度。
    fn drop(&mut self) {
        use core::sync::atomic::Ordering;
        self.state.drops.fetch_add(1, Ordering::AcqRel);
        self.state.finished.insert(self.cpu, Ordering::AcqRel);
    }
}

/// 拥有的 Arc 移入真实 Send+'static 子入口，发表成功后才登记本例期待的完成数量。
fn publish_child(
    fixture: &mut ChildFixture,
    cpu: usize,
    failure: u8,
) -> Result<crate::ktest::ChildHandle, TestError> {
    let state = alloc::sync::Arc::clone(&fixture.state);
    let handle = crate::ktest::spawn_on(cpu, move |token| {
        let local = OwnedChildDrop { state, cpu };
        local
            .state
            .entered
            .insert(cpu, core::sync::atomic::Ordering::AcqRel);
        assert_eq!(crate::cpu::current().id, cpu);
        match failure {
            1 => Err(TestError("vertical real child error")),
            2 => panic!("vertical real child panic"),
            _ => {
                while !token.stop_requested() {
                    crate::sched::yield_now();
                }
                Ok(())
            }
        }
    })?;
    fixture.started += 1;
    fixture.cpu_mask.insert(cpu);
    Ok(handle)
}

#[::ktest::suite(context = "Thread", fixture = ChildFixture, min_cpus = 2, tags("selftest", "selftest-normal", "thread", "smp"))]
mod children {
    use super::*;

    /// 丢弃全部观察句柄仍由父 scope 负责停止/join，最后才消费 fixture。
    #[test]
    #[case("cpu0", 0usize)]
    #[case("cpu1", 1usize)]
    #[case("both", 2usize)]
    #[case("all", 3usize)]
    fn ownership(_: &mut TestContext<'_>, fixture: &mut ChildFixture, cpu: usize) -> TestResult {
        if cpu == 3 {
            for target in 0..crate::cpu::count() {
                drop(publish_child(fixture, target, 0)?);
            }
        } else if cpu == 2 {
            drop(publish_child(fixture, 0, 0)?);
            drop(publish_child(fixture, 1, 0)?);
        } else {
            drop(publish_child(fixture, cpu, 0)?);
        }
        while fixture
            .state
            .entered
            .load(core::sync::atomic::Ordering::Acquire)
            != fixture.cpu_mask
        {
            crate::sched::yield_now();
        }
        Ok(())
    }
}

#[::ktest::suite(context = "Thread", fixture = ChildFixture, tags("selftest", "selftest-failure"), ignore = "intentional child failures; select explicitly")]
mod child_failures {
    use super::*;

    /// 子显式 Err 必须独立归父 Fail；主体自身正常返回。
    #[test]
    fn child_error(_: &mut TestContext<'_>, fixture: &mut ChildFixture) -> TestResult {
        drop(publish_child(fixture, 0, 1)?);
        Ok(())
    }

    /// 子 panic 的匹配消息不能满足父主体 should_panic。
    #[test]
    #[should_panic(expected = "vertical real child panic")]
    fn child_panic_not_body(_: &mut TestContext<'_>, fixture: &mut ChildFixture) -> TestResult {
        drop(publish_child(fixture, 0, 2)?);
        Ok(())
    }

    /// 主体和子任务各自的原始错误都必须保留，不把子失败覆盖进主失败槽。
    #[test]
    fn primary_and_child(_: &mut TestContext<'_>, fixture: &mut ChildFixture) -> TestResult {
        drop(publish_child(fixture, 0, 1)?);
        Err(TestError("vertical primary plus real child error"))
    }
}
