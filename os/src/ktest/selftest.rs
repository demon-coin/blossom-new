//! 内核测试框架（`ktest`）核心运行机制、栈展开恢复与故障分类的自检测试套件模块。
//!
//! ## 模块职责与架构定位
//! 本模块归属 `ktest` 子系统自身的功能验证层，通过多个正向与负向测试套件，全面检验测试框架在两类运行上下文（`Boot` 启动栈与 `Thread` 内核线程栈）下的行为正确性。
//! - **上游调用者**：由宿主机测试运行器通过 `ktest::runner` 按套件与用例序号选择性调度执行（其中特意构造失败或致命崩溃的负向套件默认带有 `ignore` 标记，仅在框架自检专项验证时显式指定运行）。
//! - **下游依赖**：`ktest::panic`（异常捕获与展开状态查询）、`ktest::registers`（被调用者保存寄存器往返校验）、`ktest::runner::case_seed`（SHA-256 种子派生）以及 `sync` / `sched` / `cpu` 同步与多核调度原语。
//!
//! ## 核心设计与自检套件矩阵
//! ```text
//! +--------------------+---------+-----------+--------------------------------------------------------+
//! | 套件名称           | Context | Isolation | 核心验证目标                                           |
//! +--------------------+---------+-----------+--------------------------------------------------------+
//! | boot               | Boot    | Suite     | 无堆环境下的参数化用例、SHA-256 种子、多层 Drop 展开、 |
//! |                    |         |           | 自旋锁/IRQ 恢复、FP Off 保持、ABI 寄存器往返与消息截断 |
//! | thread             | Thread  | Suite     | 内核线程栈上的同等功能 + 跨核迁移后 (SMP) 异常展开边界 |
//! | failures           | Thread  | Suite     | 负向验证：expect! 继续执行、Err 返回、assert! 失败、   |
//! |                    |         |           | should_panic 缺失/不匹配/截断，以及后续用例独立恢复    |
//! | setup_failure      | Boot    | Suite     | 夹具 set_up 失败或 panic 时局部对象析构且跳过 body     |
//! | cleanup_failure    | Thread  | Case      | quiesce / tear_down 失败或 panic 时多源错误记录与关机  |
//! | fatal              | Thread  | Case      | double-panic、跨 C ABI 展开、IPI 同步回调 panic 与     |
//! |                    |         |           | panic=abort 模式下的底层致命故障拦截                   |
//! | children           | Thread  | Suite     | 多核子线程协作停止 (stop_requested)、汇合与资源析构    |
//! | child_failures     | Thread  | Suite     | 子线程返回 Err 或触发 panic 时的独立错误归属与汇总     |
//! +--------------------+---------+-----------+--------------------------------------------------------+
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `lock_and_irq` 测试辅助函数先获取 `PreemptGuard` 固定当前 CPU，再根据参数可选获取外层 `IrqSaveGuard`，最后获取用例局部的 `LocalFixture::lock` 自旋锁，触发栈展开后验证锁守卫被自动析构解锁且中断使能位恢复原状。
//! - `ChildState` 使用无锁原子计数器（`AtomicUsize`）与原子 CPU 位图（`AtomicCpuSet`）跨核记录子线程进入与析构完成状态，在 `Drop` 实现中绝不睡眠或触发调度。

use core::cell::Cell;
use core::fmt;

use ::ktest::{Fixture, TestContext, TestError, TestResult};

use crate::ktest::panic::{catch, is_unwinding};
use crate::sync::{IrqSaveGuard, PreemptGuard, SpinLock};

/// 每个测试用例独立创建的局部测试夹具：包含一把自旋锁和用于验证栈展开期间局部对象析构次数的计数器。
struct LocalFixture {
    /// 用于测试异常栈展开时守卫（Guard）自动解锁与中断状态恢复的局部自旋锁。
    lock: SpinLock<usize>,
    /// 记录三层嵌套调用栈帧（`level = 0, 1, 2`）中局部 `DropMark` 对象被析构次数的计数器数组。
    drops: [Cell<usize>; 3],
    /// 记录在栈展开进行中（`is_unwinding() == true`）触发 `DropMark::drop` 的累计次数。
    unwinding_drops: Cell<usize>,
}

impl Fixture for LocalFixture {
    /// 初始化仅包含栈内状态的 `LocalFixture`（不依赖堆内存分配、物理页、磁盘或内核线程）。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            lock: SpinLock::new(0),
            drops: [Cell::new(0), Cell::new(0), Cell::new(0)],
            unwinding_drops: Cell::new(0),
        })
    }
}

/// `boot` 测试套件的套件级共享基础资源，用于验证无堆环境下套件级 `set_up` 与 `tear_down` 各执行一次。
struct NoHeapBase {
    /// 在套件初始化时写入、并在每个用例 `set_up` 及套件 `tear_down` 时校验的固定特征值（`13`）。
    token: usize,
}

impl Fixture for NoHeapBase {
    /// 执行 `boot` 套件的一次性初始化：输出套件级 Setup 诊断行，并通过 `catch` 验证在套件 Setup 阶段异常捕获器可用。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        context.diagnostic(format_args!("vertical Boot suite setup once"));
        Ok(Self {
            token: catch(|| 13).unwrap(),
        })
    }

    /// 在 `boot` 套件所有选中用例执行完毕后校验 `token` 并输出套件级 Cleanup 诊断行。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert_eq!(self.token, 13);
        context.diagnostic(format_args!("vertical Boot suite cleanup once"));
        Ok(())
    }
}

impl Fixture<NoHeapBase> for LocalFixture {
    /// 在依赖 `NoHeapBase` 的套件中，先校验套件级共享资源的 `token` 值，再创建当前用例独立的 `LocalFixture`。
    fn set_up(context: &mut TestContext<'_>, base: &NoHeapBase) -> Result<Self, TestError> {
        assert_eq!(base.token, 13);
        <Self as Fixture<()>>::set_up(context, &())
    }
}

/// 用于观测栈帧析构行为的局部守卫对象：在离开作用域或栈展开时递增对应的计数器。
struct DropMark<'a> {
    /// 指向当前栈帧层级对应的析构次数计数器。
    count: &'a Cell<usize>,
    /// 指向栈展开期间触发析构的总次数计数器。
    unwinding: &'a Cell<usize>,
}

impl Drop for DropMark<'_> {
    /// 递增当前栈帧的析构计数；若当前正处于 `panic!` 栈展开过程中，则同时递增展开析构计数。
    fn drop(&mut self) {
        self.count.set(self.count.get() + 1);
        if is_unwinding() {
            self.unwinding.set(self.unwinding.get() + 1);
        }
    }
}

/// 构造禁止内联的递归调用链（共 `level + 1` 层独立栈帧），在每层栈上放置一个 `DropMark` 并在最深层（`level == 0`）触发 `panic!`。
///
/// 用于验证 DWARF 栈展开器能够依次穿过每一层栈帧的着陆垫（Landing Pad）并完整执行所有局部变量的 `Drop` 析构函数。
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

/// 验证在同一个测试用例内嵌套使用 `expect_panic` 与 `catch`、以及连续多次捕获 `panic!` 时，每次展开都能精确停在最内层捕获边界并清理展开状态。
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

/// 验证在持有自旋锁（自动关闭本地中断）期间发生 `panic!` 并完成栈展开后：
/// 1. 自旋锁守卫被正常析构释放，后续可再次通过 `try_lock` 获取该锁；
/// 2. 本地 CPU 的中断使能状态（`irq_enabled()`）恢复到进入测试块之前的状态 `before`；
/// 3. 异常发生前已写入锁保护数据的修改（`*guard = 17`）保持原样。
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

/// 读取当前 CPU 的硬件浮点单元（FPU）使能控制位（RISC-V 为 `sstatus.FS`，LoongArch64 为 `CSR_EUEN.FPE`）。
///
/// 仅读取控制寄存器位，不执行任何浮点指令，也不改动当前任务的浮点控制状态。
fn fp_control() -> usize {
    let value: usize;
    #[cfg(target_arch = "riscv64")]
    // SAFETY: 当前运行在内核态（S-mode），通过 `csrr` 读取 `sstatus` 寄存器不会触发浮点异常，也不会修改内存、栈或中断状态。
    unsafe {
        core::arch::asm!("csrr {value}, sstatus", value = out(reg) value, options(nomem, nostack));
        value & crate::arch::SR_FS
    }
    #[cfg(target_arch = "loongarch64")]
    // SAFETY: 当前运行在内核最高特权级（PLV0），通过 `csrrd` 读取 `CSR_EUEN` 寄存器仅读取 FPU 使能位，不访问 FCSR/FPR，也不修改内存、栈或控制状态。
    unsafe {
        core::arch::asm!("csrrd {value}, {euen}", value = out(reg) value,
            euen = const crate::arch::CSR_EUEN, options(nomem, nostack));
        value & crate::arch::EUEN_FP
    }
}

/// 验证内核常规执行流在进入时处于 FPU 关闭（FP Off）状态，且经历一次 `panic!` 栈展开捕获后 FPU 依然保持关闭状态。
fn fp_off_restore() {
    let _preempt = PreemptGuard::new();
    let before = fp_control();
    assert_eq!(before, 0, "ordinary kernel body must enter with FP Off");
    let record = catch(|| panic!("vertical FP Off")).unwrap_err();
    assert!(record.matches_message("vertical FP Off"));
    assert_eq!(fp_control(), before);
}

/// 用于构造超长多字节 UTF-8 异常消息的格式化辅助类型，验证定长异常消息缓冲区的字符边界截断逻辑。
struct LongMessage;
impl fmt::Display for LongMessage {
    /// 流式输出前缀、200 个三字节中文字符 `"界"` 以及尾部标记（全程不分配堆内存），使其总长度超出固定 panic 消息缓冲区容量。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("vertical-long:")?;
        for _ in 0..200 {
            formatter.write_str("界")?;
        }
        formatter.write_str("must-not-replace-prefix")
    }
}

/// 验证当 `panic!` 消息超出固定缓冲区容量被截断时：
/// 1. `message_truncated()` 标志被置为 `true`；
/// 2. 已保存的消息保留合法的 UTF-8 前缀并以 `" [truncated]"` 结尾；
/// 3. 被截断的不完整消息不会通过 `matches_message` 的期望消息匹配校验。
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

    /// 测试目标：验证 `Boot` 上下文中的参数化测试用例（`#[case]`）能够正确传入边界值（`0`、`1`、`usize::MAX`）并执行断言。
    #[test]
    #[case("zero", 0usize, Some(1usize))]
    #[case("one", 1usize, Some(2usize))]
    #[case("max", usize::MAX, None)]
    fn boundary(_: &mut TestContext<'_>, _: &mut LocalFixture, value: usize, next: Option<usize>) {
        assert_eq!(value.checked_add(1), next);
    }

    /// 测试目标：验证内核流式 `case_seed` 函数针对空名称、56 字节 SHA-256 填充边界、多分组多字节 UTF-8 名称及常规完整路径名称计算出的 64 位种子，与宿主机标准 SHA-256 计算结果完全一致。
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

    /// 测试目标：验证 `Boot` 栈上声明了 `#[should_panic(expected = ...)]` 的用例在触发匹配消息的 `panic!` 时被判定为通过。
    #[test]
    #[should_panic(expected = "vertical expected Boot")]
    fn expected(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        panic!("vertical expected Boot");
    }

    /// 测试目标：验证 `Boot` 栈（无堆分配环境）下嵌套与连续多次异常捕获均能正常展开并恢复状态。
    #[test]
    fn nested(context: &mut TestContext<'_>, _: &mut LocalFixture) {
        nested_and_sequential(context);
    }

    /// 测试目标：验证 `Boot` 栈上三层栈帧在经历异常展开时，每一层的局部对象 `Drop` 析构函数均恰好在展开状态下执行一次。
    #[test]
    fn drops(context: &mut TestContext<'_>, fixture: &mut LocalFixture) {
        context.expect_panic("vertical multilayer drop", || drop_level(fixture, 2));
        assert_eq!(fixture.drops.each_ref().map(Cell::get), [1, 1, 1]);
        assert_eq!(fixture.unwinding_drops.get(), 3);
    }

    /// 测试目标：验证 `Boot` 栈上无论进入前中断处于开启还是关闭状态，持锁期间发生 `panic!` 展开都会自动释放自旋锁并恢复原有中断状态。
    #[test]
    #[case("entry", false)]
    #[case("off", true)]
    fn lock_irq(context: &mut TestContext<'_>, fixture: &mut LocalFixture, off: bool) {
        lock_and_irq(context, fixture, off);
    }

    /// 测试目标：验证 `Boot` 栈在内存管理初始化之前经历异常展开后，硬件浮点控制位仍保持关闭（FP Off）。
    #[test]
    fn fp_off(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        fp_off_restore();
    }

    /// 测试目标：验证 `Boot` 栈上经历异常展开后，目标架构 ABI 规定的所有被调用者保存通用/浮点寄存器、SP、Per-CPU 基址及 FCSR 均完整恢复。
    #[test]
    fn registers(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        assert!(crate::ktest::selftest_register_roundtrip());
    }

    /// 测试目标：验证 `Boot` 栈上超长多字节 UTF-8 异常消息在无堆环境下能够按字符边界安全截断并标记。
    #[test]
    fn truncation(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        truncated_message();
    }
}

#[::ktest::suite(context = "Thread", fixture = LocalFixture, tags("selftest", "selftest-normal", "thread"))]
mod thread {
    use super::*;

    /// 测试目标：验证 `Thread` 套件内与 `boot` 套件同名的参数化用例具有独立的模块全路径名称并正确执行。
    #[test]
    #[case("zero", 0usize, Some(1usize))]
    #[case("max", usize::MAX, None)]
    fn boundary(_: &mut TestContext<'_>, _: &mut LocalFixture, value: usize, next: Option<usize>) {
        assert_eq!(value.checked_add(1), next);
    }

    /// 测试目标：验证内核线程（`Task`）栈上的 `#[should_panic]` 用例能够被用例捕获边界拦截，不会向上击穿测试套件调度线程。
    #[test]
    #[should_panic(expected = "vertical expected Thread")]
    fn expected(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        panic!("vertical expected Thread");
    }

    /// 测试目标：验证内核线程栈上嵌套与连续多次异常捕获边界的正确性。
    #[test]
    fn nested(context: &mut TestContext<'_>, _: &mut LocalFixture) {
        nested_and_sequential(context);
    }

    /// 测试目标：验证内核线程栈上的多层调用帧在栈展开期间同样逐层执行局部变量的 `Drop` 析构。
    #[test]
    fn drops(context: &mut TestContext<'_>, fixture: &mut LocalFixture) {
        context.expect_panic("vertical multilayer drop", || drop_level(fixture, 2));
        assert_eq!(fixture.drops.each_ref().map(Cell::get), [1, 1, 1]);
        assert_eq!(fixture.unwinding_drops.get(), 3);
    }

    /// 测试目标：验证内核线程在 `expect_panic` 捕获块内部主动跨核迁移（`test_migrate_to`）到另一颗 CPU 后触发 `panic!`，栈展开器仍能沿当前任务栈正确回退并保留新 CPU 的 Per-CPU 基址。
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

    /// 测试目标：验证可抢占内核线程在关闭抢占段内触发持锁 `panic!` 后，自旋锁与本地中断使能状态均被正确恢复。
    #[test]
    #[case("entry", false)]
    #[case("off", true)]
    fn lock_irq(context: &mut TestContext<'_>, fixture: &mut LocalFixture, off: bool) {
        lock_and_irq(context, fixture, off);
    }

    /// 测试目标：验证内核线程在经历栈展开后（即使展开器内部临时访问过浮点寄存器），任务自身的 FPU 控制位仍恢复为关闭状态（FP Off）。
    #[test]
    fn fp_off(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        fp_off_restore();
    }

    /// 测试目标：验证内核线程栈上经历异常展开后，全部被调用者保存通用/浮点寄存器、SP、Per-CPU 基址及 FCSR 均完整恢复。
    #[test]
    fn registers(_: &mut TestContext<'_>, _: &mut LocalFixture) {
        assert!(crate::ktest::selftest_register_roundtrip());
    }

    /// 测试目标：验证内核线程栈上的超长 UTF-8 异常消息截断与不匹配保护行为。
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

    /// 测试目标：验证非致命期望宏 `expect!` 失败时只累加期望失败计数而不提前中止用例，后续栈展开断言与诊断输出仍继续执行。
    #[test]
    fn expect_continues(context: &mut TestContext<'_>) {
        let fixture = <LocalFixture as Fixture<()>>::set_up(context, &()).unwrap();
        ::ktest::expect!(context, false, "vertical intentional expectation");
        let record = catch(|| drop_level(&fixture, 2)).unwrap_err();
        assert!(record.matches_message("vertical multilayer drop"));
        assert_eq!(fixture.drops.each_ref().map(Cell::get), [1, 1, 1]);
        context.diagnostic(format_args!("vertical after failed expectation reached"));
    }

    /// 测试目标：验证返回 `Err(TestError(...))` 的测试用例会被记录为主线程错误（`Failure::Returned`）并标记用例失败。
    #[test]
    fn result_error() -> TestResult {
        Err(TestError("vertical explicit result error"))
    }

    /// 测试目标：验证普通 `assert!` 失败能被正确归属为当前用例失败，且不会污染同一批次后续运行的正常用例。
    #[expect(
        clippy::assertions_on_constants,
        reason = "自测必须触发真实 assert 失败并核对归属，不能替换为另一种 panic 来源"
    )]
    #[test]
    fn assertion_failure() {
        assert!(false, "vertical intentional assertion");
    }

    /// 测试目标：验证即使主体最终触发了满足 `#[should_panic]` 预期的异常，此前已经失败的 `expect!` 断言仍会使该用例最终判定为失败。
    #[test]
    #[should_panic(expected = "vertical matching after expectation")]
    fn expect_then_expected(context: &mut TestContext<'_>) {
        ::ktest::expect!(context, false, "vertical independent expectation");
        panic!("vertical matching after expectation");
    }

    /// 测试目标：验证声明了 `#[should_panic]` 但主体正常返回未触发异常的用例会被判定为失败。
    #[test]
    #[should_panic(expected = "vertical missing")]
    fn missing_panic() {}

    /// 测试目标：验证触发了 `panic!` 但实际消息不包含 `#[should_panic(expected = ...)]` 指定子串的用例会被判定为失败。
    #[test]
    #[should_panic(expected = "vertical wanted")]
    fn mismatched_panic() {
        panic!("vertical different");
    }

    /// 测试目标：验证当异常消息因过长而被截断时，即使保留的前缀包含期望子串，也不会被误判为满足 `#[should_panic]`。
    #[test]
    #[should_panic(expected = "vertical-long")]
    fn truncated_prefix() {
        panic!("{}", LongMessage);
    }

    /// 测试目标：验证局部 `context.expect_panic(...)` 在未发生异常或异常消息不匹配时，会记录期望失败并继续执行用例后续语句。
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

    /// 测试目标：验证排在多个故意失败用例之后的正常用例仍能独立执行并获得 `Pass` 结果。
    #[test]
    fn after_failure() {
        let value = catch(|| 53).unwrap();
        assert_eq!(value, 53);
        assert!(!is_unwinding());
    }
}

/// `setup_failure` 套件的共享统计夹具：记录用例级夹具尝试构造的次数与半成品局部对象被析构的次数。
struct SetupEvidence {
    /// 用例级夹具 `RejectedFixture::set_up` 被调用的总次数。
    attempts: Cell<usize>,
    /// `RejectedFixture::set_up` 内部已构造的局部对象 `PartialSetup` 被成功析构的次数。
    drops: Cell<usize>,
}

impl Fixture for SetupEvidence {
    /// 创建初始计数均为 0 的栈内统计对象。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            attempts: Cell::new(0),
            drops: Cell::new(0),
        })
    }

    /// 在套件结束时断言每次失败的用例 `set_up` 尝试都已完整析构其内部创建的局部对象，并输出统计诊断行。
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

/// 在 `RejectedFixture::set_up` 内部创建的局部守卫（元组字段 `0` 为指向套件级 `SetupEvidence::drops` 计数器的借用），用于验证 `set_up` 中途返回 `Err` 或触发 `panic!` 时局部变量均被正常析构。
struct PartialSetup<'a>(&'a Cell<usize>);

impl Drop for PartialSetup<'_> {
    /// 无论因 `Err` 提前返回还是因栈展开退出作用域，均将 `drops` 计数加 1。
    fn drop(&mut self) {
        self.0.set(self.0.get() + 1);
    }
}

/// 故意在 `set_up` 阶段返回错误或触发异常的测试用例夹具。
struct RejectedFixture;

impl Fixture<SetupEvidence> for RejectedFixture {
    /// 递增尝试计数并创建 `PartialSetup` 局部守卫，随后按用例参数选择触发 `panic!` 或返回 `Err`。
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

    /// 测试目标：验证当夹具 `set_up` 返回 `Err` 或发生 `panic!` 时，框架会将用例标记为 `# ERROR`、正常析构 `set_up` 内的局部对象，且绝不执行测试用例主体（即使声明了匹配的 `#[should_panic]`）。
    #[test]
    #[case("error", ())]
    #[case("panic", ())]
    #[should_panic(expected = "vertical partial setup panic")]
    fn partial(_: &mut TestContext<'_>, _: &mut RejectedFixture, _: ()) {
        panic!("vertical setup failure body unexpectedly ran");
    }
}

/// 故意在清理阶段（`quiesce` 或 `tear_down`）返回错误或触发异常的测试夹具，用于验证清理失败时的中止与错误报告机制。
struct CleanupError {
    /// 为 `true` 时在 `quiesce` 阶段触发故障；为 `false` 时在 `tear_down` 阶段触发故障。
    in_quiesce: bool,
    /// 为 `true` 时以 `panic!` 形式触发清理故障；为 `false` 时以返回 `Err(TestError)` 形式触发清理故障。
    panics: bool,
}

impl Fixture for CleanupError {
    /// 根据当前用例参数（`"quiesce"` / `"tear_down"` / `"quiesce_panic"` / `"tear_down_panic"`）配置清理故障发生阶段与形式。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            in_quiesce: matches!(context.case.parameter, Some("quiesce" | "quiesce_panic")),
            panics: matches!(
                context.case.parameter,
                Some("quiesce_panic" | "tear_down_panic")
            ),
        })
    }

    /// 若配置为在 `quiesce` 阶段失败，则在此处触发 `panic!` 或返回 `Err`。
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

    /// 在 `tear_down` 阶段触发 `panic!` 或返回 `Err`，验证框架通过 `case-abort` 报告清理错误并关机。
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

    /// 测试目标：验证当测试主体返回 `Err` 且随后的 `quiesce` 或 `tear_down` 清理阶段也返回 `Err` 时，两份错误会同时保存在 `case-abort` 的 `primary` 与 `cleanup` 字段中。
    #[test]
    #[case("quiesce", ())]
    #[case("tear_down", ())]
    fn primary_and_cleanup(_: &mut TestContext<'_>, _: &mut CleanupError, _: ()) -> TestResult {
        Err(TestError("vertical primary before cleanup error"))
    }

    /// 测试目标：验证当主线程、子线程以及 `tear_down` 清理阶段均发生错误时，三方错误分别独立保存在 `primary`、`child` 与 `cleanup` 字段中，互不覆盖。
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

    /// 测试目标：验证当 `#[should_panic]` 用例在主体阶段未发生 `panic!`、而在清理阶段发生了同消息的 `panic!` 时，不能将其算作满足主体 `should_panic` 预期。
    #[test]
    #[case("quiesce_panic", ())]
    #[case("tear_down_panic", ())]
    #[should_panic(expected = "vertical cleanup panic")]
    fn cleanup_panic_is_not_body_panic(context: &mut TestContext<'_>, _: &mut CleanupError, _: ()) {
        context.diagnostic(format_args!("vertical body returned before cleanup panic"));
    }
}

/// 在 `Drop::drop` 析构期间再次触发 `panic!` 的辅助类型，用于构造栈展开期间的双重异常（Double Panic）。
struct PanicOnDrop;
impl Drop for PanicOnDrop {
    /// 在第一次 `panic!` 栈展开执行着陆垫析构时抛出第二次 `panic!`，触发底层 `double-panic` 致命事件。
    fn drop(&mut self) {
        panic!("vertical second panic in Drop");
    }
}

/// 使用非展开 C ABI（`extern "C"`）声明的测试函数：内部直接触发 `panic!`，用于验证编译器生成的跨 ABI 展开拦截检查。
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

    /// 测试目标：验证在栈展开析构局部变量 `PanicOnDrop` 时再次发生 `panic!`，会被底层异常处理器识别并上报为 `double-panic` 致命事件。
    #[test]
    #[ktest(
        expected_fatal = "double-panic",
        fatal_message = "double panic during unwind"
    )]
    fn double_panic() {
        let _local = PanicOnDrop;
        panic!("vertical first panic before Drop");
    }

    /// 测试目标：验证在 `extern "C"` 函数内触发无法跨越 ABI 边界展开的 `panic!`（`can_unwind() == false`）时，底层异常处理器会立即识别并上报 `abi` 致命事件。
    #[test]
    #[ktest(
        expected_fatal = "abi",
        fatal_message = "panic cannot unwind across ABI boundary"
    )]
    fn abi() {
        panic_across_abi();
    }

    /// 测试目标：验证通过 `cpu::call` 执行同核同步跨核调用回调时，回调内部发生的 `panic!` 处于不可展开的同步调用屏障内，会直接触发底层 `panic` 致命事件而不会错误复用外层线程的捕获槽。
    #[test]
    #[ktest(
        expected_fatal = "panic",
        fatal_message = "vertical local synchronous call panic"
    )]
    fn local_sync_call() {
        /// 供 `cpu::call` 调用的测试回调：直接触发一次 `panic!`。
        ///
        /// # Safety
        /// 回调函数不解引用传入的空指针参数，也不在关中断上下文中睡眠或递归调用 `cpu::call`。
        unsafe fn fail(_: *mut ()) {
            panic!("vertical local synchronous call panic");
        }
        // SAFETY:
        // 1. `crate::cpu::current().id` 是当前正在运行的在线 CPU 编号。
        // 2. `fail` 回调忽略传入的 `null_mut()` 参数，且当前上下文未持有与 `cpu::call` 冲突的锁。
        unsafe { crate::cpu::call(crate::cpu::current().id, fail, core::ptr::null_mut()) };
    }

    /// 测试目标：在以 `panic = "abort"` 编译的内核镜像中，验证普通 `panic!` 直接触发底层 `panic` 致命事件并关机。
    #[test]
    #[ktest(expected_fatal = "panic", fatal_message = "vertical real abort panic")]
    fn abort_panic() {
        panic!("vertical real abort panic");
    }
}

/// 父测试用例与派生的测试子线程之间通过 `Arc` 共享的原子状态记录表。
struct ChildState {
    /// 已完成局部守卫 `OwnedChildDrop` 析构的子线程总数量。
    drops: core::sync::atomic::AtomicUsize,
    /// 已成功进入线程入口函数并开始运行的子线程所在 CPU 编号集合。
    entered: crate::cpu::AtomicCpuSet,
    /// 已完成 `OwnedChildDrop` 析构退出的子线程所在 CPU 编号集合。
    finished: crate::cpu::AtomicCpuSet,
}

/// 用于测试多核子线程派生、协作停止与汇合清理的用例夹具。
struct ChildFixture {
    /// 与所有派生子线程共享的原子观测状态。
    state: alloc::sync::Arc<ChildState>,
    /// 当前用例已成功启动的子线程数量。
    started: usize,
    /// 当前用例已启动子线程的目标 CPU 编号集合。
    cpu_mask: crate::cpu::CpuSet,
}

impl Fixture for ChildFixture {
    /// 在堆上分配 `Arc<ChildState>` 并初始化子线程计数与 CPU 掩码。
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

    /// 在框架完成所有子线程的协作停止（Quiesce）与栈回收汇合（Join）后，验证所有已启动子线程的栈上局部对象均已完整析构。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        use core::sync::atomic::Ordering;
        assert_eq!(self.state.drops.load(Ordering::Acquire), self.started);
        assert_eq!(self.state.finished.load(Ordering::Acquire), self.cpu_mask);
        Ok(())
    }
}

/// 放置在子线程栈帧上的局部守卫：当子线程正常退出或因 `panic!` 栈展开退出时，在共享 `ChildState` 中登记析构完成。
struct OwnedChildDrop {
    /// 指向父子共享 `ChildState` 的强引用。
    state: alloc::sync::Arc<ChildState>,
    /// 该子线程绑定运行的 CPU 编号。
    cpu: usize,
}

impl Drop for OwnedChildDrop {
    /// 以原子操作递增子线程析构计数并标记对应 CPU 上的子线程已完成局部对象清理。
    fn drop(&mut self) {
        use core::sync::atomic::Ordering;
        self.state.drops.fetch_add(1, Ordering::AcqRel);
        self.state.finished.insert(self.cpu, Ordering::AcqRel);
    }
}

/// 在目标 `cpu` 上派生一个受测试框架管理的子线程，并在创建成功后更新 `fixture` 中的已启动计数与 CPU 集合。
///
/// 参数 `failure` 控制子线程行为：`0` 表示循环等待 `token.stop_requested()` 后正常返回 `Ok(())`；`1` 表示立即返回 `Err`；`2` 表示立即触发 `panic!`。
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

    /// 测试目标：验证即使测试主体直接丢弃（Drop）了子线程的 `ChildHandle`，测试框架的 `ArcScope` 仍会在用例主体结束后自动向所有子线程发送停止信号、等待其完全退出栈帧，最后才执行 `ChildFixture::tear_down` 断言所有子线程资源已释放。
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

    /// 测试目标：验证当测试主体正常返回 `Ok(())`、但派生的子线程返回 `Err` 时，子线程错误会被记录在 `child` 错误槽中并将整个用例判定为失败。
    #[test]
    fn child_error(_: &mut TestContext<'_>, fixture: &mut ChildFixture) -> TestResult {
        drop(publish_child(fixture, 0, 1)?);
        Ok(())
    }

    /// 测试目标：验证子线程发生的 `panic!` 会被归属为子线程错误（`child`），而不能用来满足父线程主体声明的 `#[should_panic]` 预期。
    #[test]
    #[should_panic(expected = "vertical real child panic")]
    fn child_panic_not_body(_: &mut TestContext<'_>, fixture: &mut ChildFixture) -> TestResult {
        drop(publish_child(fixture, 0, 2)?);
        Ok(())
    }

    /// 测试目标：验证当父线程主体与子线程同时返回 `Err` 时，两份错误分别独立保存在 `primary` 与 `child` 字段中，互不覆盖。
    #[test]
    fn primary_and_child(_: &mut TestContext<'_>, fixture: &mut ChildFixture) -> TestResult {
        drop(publish_child(fixture, 0, 1)?);
        Err(TestError("vertical primary plus real child error"))
    }
}
