//! `ktest` 内核与宿主通用测试协议及生命周期驱动库。
//!
//! # 模块职责与架构定位
//! 本模块定义了 `no_std` 环境下的测试元数据描述符、测试上下文（`TestContext`）、
//! 资源夹具特征（`Fixture`）以及套件与用例的执行状态机（`run_suite`、`run_case`）。
//! - **上游调用者**：由 `ktest_macros::suite` 过程宏生成的静态套件注册表与用例包装函数，以及测试编写者调用的断言与上下文接口。
//! - **下游依赖方**：由内核测试框架（如 `os::ktest`）或宿主测试桩实现 `Services` 与 `CaseRunner` 特征，提供具体的栈展开捕获（Unwind）、诊断输出、后台子任务同步与异常终止能力。
//!
//! # 核心设计与生命周期状态机
//!
//! ## 1. 两级夹具（Fixture）模型
//! 测试资源分为两个层级：
//! - **套件级共享资源（`suite_resources`）**：由 `run_suite<R>` 在执行整个套件前构造一次，以只读引用 `&R` 共享给套件内被选中的每个测试用例，最后在套件结束时统一静止化并销毁。
//! - **用例级独立夹具（`fixture`）**：由 `run_case<F, R>` 为每个测试用例实例独立构造，以可变引用 `&mut F` 传入测试主体，确保各测试用例及参数化实例之间的可变状态彼此隔离。
//!
//! ## 2. 单用例五阶段执行状态机（`Phase`）
//! ```text
//! +------------------+
//! |   Phase::Setup   |  调用 F::set_up(&mut ctx, &resources) 构造用例夹具
//! +--------+---------+
//!          |
//!          |---------------------------+ (Setup 返回 Err 或 Panic：跳过 Body 与 Quiesce)
//!          v                           |
//! +------------------+                 |
//! |   Phase::Body    |  执行测试主体与非致命软断言 (expect / expect_eq / expect_panic)
//! +--------+---------+                 |
//!          |                           |
//!          v                           v
//! (请求停止后台子任务: services.stop_children())
//!          |                           |
//!          v                           |
//! +------------------+                 |
//! |  Phase::Quiesce  |  调用 fixture.quiesce(&mut ctx) 停止异步生产源（如中断、Workqueue、DMA）
//! +--------+---------+                 |
//!          |                           |
//!          +---------------------------+
//!          v
//! +------------------+
//! |   Phase::Join    |  调用 services.join_children() 等待全部后台子任务退出执行栈
//! +--------+---------+
//!          |
//!          v
//! +------------------+
//! |  Phase::Cleanup  |  调用 fixture.tear_down(&mut ctx) 消费并释放夹具对象
//! +------------------+
//! ```
//!
//! ## 3. 链接段注册表布局（`.ktest.suites`）
//! ```text
//! .ktest.suites 只读数据段（按指针步长紧密排列）:
//! +--------------------+--------------------+-----+
//! | SuiteSlot(&SUITE0) | SuiteSlot(&SUITE1) | ... |
//! +--------------------+--------------------+-----+
//!           |
//!           v
//!      SuiteDesc {
//!          name: "module::path",
//!          defaults: Defaults { ... },
//!          cases: &[CaseDesc { ... }],
//!          run: __ktest_run,
//!      }
//! ```
//!
//! # 并发与生命周期安全契约
//! - **栈展开隔离**：夹具对象 `F` 与套件资源 `R` 保存在 `services.catch` 捕获边界之外的局部变量槽中。当测试主体发生 Panic 时，局部栈帧展开不会提前析构夹具，保证夹具存活至 `Quiesce` 与 `Join` 阶段完成之后。
//! - **清理顺序契约**：销毁夹具前必须依次完成 `stop_children` -> `quiesce` -> `join_children`。只有在确认所有后台子任务和异步回调已彻底退出执行栈后，才会进入 `Cleanup` 阶段调用 `tear_down` 释放内存。
//! - **清理失败降级终止**：若 `Quiesce`、`Join` 或 `Cleanup` 任一阶段返回错误或发生 Panic，说明后台任务可能仍在并发访问夹具内存；此时框架通过 `services.terminate` 立即终止运行环境，不再继续 Drop 夹具，从而防止悬垂指针访问（Use-After-Free）。

#![no_std]

use core::fmt::{self, Write};

#[cfg(feature = "macros")]
/// 重新导出 `#[ktest::suite]` 过程宏属性，用于声明测试套件模块。
pub use ktest_macros::suite;

/// 测试用例或夹具显式返回的错误类型，内部持有静态生命周期的错误描述字符串，避免依赖动态内存分配器。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TestError(pub &'static str);
/// 测试函数与夹具生命周期方法的标准返回结果类型。
pub type TestResult = Result<(), TestError>;

/// 测试用例运行所处的内核执行上下文类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Context {
    /// 在内核启动早期（调度器或线程环境尚未完备前）的引导栈上下文中直接执行。
    Boot,
    /// 在可调度的独立内核线程上下文中执行。
    Thread,
}
/// 测试用例运行时的环境隔离级别。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Isolation {
    /// 同一测试套件内的用例可在同一次虚拟机运行会话中连续执行。
    Suite,
    /// 单个测试用例需要独占一次独立的虚拟机运行会话（例如包含破坏性操作或预期致命异常的测试）。
    Case,
}
/// 测试套件或用例声明依赖的外部硬件设备或内核独占子系统资源。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Resource {
    /// 串口控制台（UART）输入/输出设备。
    Uart,
    /// 供块设备或文件系统测试读写的临时擦除盘（Scratch Disk）。
    Scratch,
    /// 根文件系统块设备（Root Disk）。
    Root,
    /// 附加的扩展测试块设备（Extra Disk）。
    Extra,
    /// 从核（Application Processor）冻结与跨核同步控制权。
    ApFreeze,
    /// VirtIO 传输层平台访问钩子（用于注入或拦截 DMA/MMIO 访问）。
    VirtioAccessPlatform,
    /// 用户态网络协议栈与回环/虚拟网卡测试环境。
    NetUser,
}
impl Resource {
    /// 返回该资源在命令行过滤与宿主测试调度器契约中对应的固定标识符字符串。
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Uart => "uart",
            Self::Scratch => "scratch",
            Self::Root => "root",
            Self::Extra => "extra",
            Self::ApFreeze => "ap-freeze",
            Self::VirtioAccessPlatform => "virtio-access-platform",
            Self::NetUser => "net-user",
        }
    }
}

/// 测试套件或单个测试用例的默认执行配置属性。
#[derive(Clone, Copy, Debug)]
pub struct Defaults {
    /// 执行所在的上下文环境（`Boot` 或 `Thread`）。
    pub context: Context,
    /// 要求的环境隔离级别（`Suite` 或 `Case`）。
    pub isolation: Isolation,
    /// 用于分类筛选的静态标签列表。
    pub tags: &'static [&'static str],
    /// 运行该测试所需的最少在线 CPU 核心数。
    pub min_cpus: u32,
    /// 单次执行的最大超时时间配额（单位：毫秒）。
    pub timeout_ms: u64,
    /// 运行该测试所依赖的独占硬件或平台资源列表。
    pub resources: &'static [Resource],
}
/// 单个测试用例实例的编译期静态元数据描述符。
#[derive(Clone, Copy, Debug)]
pub struct CaseDesc {
    /// 用例的全局唯一全限定名称（格式为 `suite_path::fn_name` 或 `suite_path::fn_name[param]`）。
    pub name: &'static str,
    /// 参数化测试实例的名称标签（由 `#[case("label", ...)]` 指定；非参数化测试为 `None`）。
    pub parameter: Option<&'static str>,
    /// 定义该测试函数的源文件路径（由 `file!()` 捕获）。
    pub source: &'static str,
    /// 定义该测试函数的源文件行号（由 `line!()` 捕获）。
    pub line: u32,
    /// 合并套件默认值与用例级覆盖项后的生效执行属性。
    pub attributes: Defaults,
    /// 若被标记为忽略（`#[ignore = "reason"]`），保存跳过原因字符串。
    pub ignore: Option<&'static str>,
    /// 若预期测试主体发生 Panic（`#[should_panic]`），保存期望匹配的子串（空串表示匹配任意非截断 Panic）。
    pub should_panic: Option<&'static str>,
    /// 若预期触发内核致命异常（如 `"oom"`、`"illegal"`、`"panic"`），保存异常类型标识。
    pub expected_fatal: Option<&'static str>,
    /// 预期致命异常发生时要求诊断输出必须包含的子串。
    pub fatal_message: Option<&'static str>,
}
/// 测试套件的编译期静态描述符。
pub struct SuiteDesc {
    /// 套件的完整模块路径名称。
    pub name: &'static str,
    /// 套件级别声明的默认执行属性。
    pub defaults: Defaults,
    /// 套件内注册的全部测试用例实例描述符切片。
    pub cases: &'static [CaseDesc],
    /// 套件执行入口函数指针，负责初始化套件资源并驱动用例运行器。
    pub run: fn(&mut TestContext<'_>, &mut dyn CaseRunner) -> SuiteOutcome,
}

/// 放置在 `.ktest.suites` 专用链接段中的只读套件注册槽。
///
/// 链接脚本将该段按指针大小对齐排列，内核启动后按指针步长线性扫描即可发现全部测试套件；
/// 静态库归档抽取则通过过程宏生成的导出锚点函数强制保留该槽位。
#[repr(C)]
pub struct SuiteSlot(pub &'static SuiteDesc);

/// 测试用例或套件在生命周期状态机中当前所处的执行阶段。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// 正在构造套件资源或用例夹具（`Fixture::set_up`）。
    Setup,
    /// 正在执行测试主体逻辑（Test Body）。
    Body,
    /// 正在停止夹具管理的异步任务或硬件事件源（`Fixture::quiesce`）。
    Quiesce,
    /// 正在等待当前测试派生的全部后台子任务退出执行栈（`Services::join_children`）。
    Join,
    /// 正在消费并销毁夹具对象（`Fixture::tear_down`）。
    Cleanup,
}
/// 单个测试用例执行完毕后的最终判定状态。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// 测试主体、子任务及所有软断言均通过，且清理正常完成。
    Pass,
    /// 测试主体失败、未满足预期的 Panic/Fatal 条件、软断言失败或后台子任务失败。
    Fail,
    /// 测试用例因过滤条件、忽略标记或资源不足而被跳过。
    Skip,
    /// 夹具初始化（`Setup`）失败或清理阶段（`Quiesce` / `Join` / `Cleanup`）发生错误。
    Error,
}

/// 固定容量的栈上 UTF-8 Panic 消息缓冲区。
///
/// 容量固定为 256 字节，不依赖堆内存分配器（以便在测试内存分配器故障或 OOM 场景下安全记录消息）。
/// 当消息超出 256 字节时，仅在合法的 UTF-8 字符边界处截断并置位 `truncated` 标志。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanicMessage {
    /// 存放 UTF-8 字节序列的固定大小缓冲区（256 字节）。
    bytes: [u8; 256],
    /// 当前已写入的有效 UTF-8 字节长度（范围 `0..=256`）。
    len: u16,
    /// 消息是否因超出缓冲区容量或在外部捕获时已被截断。
    pub truncated: bool,
    /// 缓冲区是否已满并封口（一旦发生截断，后续追加写入将直接忽略）。
    sealed: bool,
}
impl PanicMessage {
    /// 从给定的字符串切片构造 `PanicMessage`，并合并外部传入的初始截断标志。
    ///
    /// 若 `message` 长度超过 256 字节，则仅保留最长的完整 UTF-8 字符前缀，并将 `truncated` 设为 `true`。
    pub fn new(message: &str, truncated: bool) -> Self {
        let mut value = Self {
            bytes: [0; 256],
            len: 0,
            truncated,
            sealed: false,
        };
        let _ = value.write_str(message);
        value
    }
    /// 借用缓冲区中已写入的有效 UTF-8 字符串切片。
    pub fn as_str(&self) -> &str {
        // SAFETY:
        // 1. `self.len` 初始为 0，唯一修改它的 `write_str` 保证 `start + count <= self.bytes.len()`（256 字节），因此 `&self.bytes[..self.len as usize]` 切片必定在数组边界内。
        // 2. `write_str` 在每次复制字节前均通过 `value.is_char_boundary(count)` 向下对齐到 UTF-8 字符边界，保证 `self.bytes[..self.len as usize]` 始终是合法的 UTF-8 字节序列。
        unsafe { core::str::from_utf8_unchecked(&self.bytes[..self.len as usize]) }
    }
}
impl Write for PanicMessage {
    /// 向缓冲区追加字符串片段；若空间不足则在最后一个完整 UTF-8 字符边界处截断并封口。
    fn write_str(&mut self, value: &str) -> fmt::Result {
        // Step 1: 若此前写入已触发容量截断封口，则忽略后续追加内容。
        if self.sealed {
            return Ok(());
        }
        // Step 2: 计算剩余可用字节数，并向前回退到合法的 UTF-8 字符边界，避免劈开多字节字符。
        let available = self.bytes.len() - self.len as usize;
        let mut count = value.len().min(available);
        while !value.is_char_boundary(count) {
            count -= 1;
        }
        // Step 3: 复制完整字符前缀并更新长度与截断标志。
        let start = self.len as usize;
        self.bytes[start..start + count].copy_from_slice(&value.as_bytes()[..count]);
        self.len += count as u16;
        self.sealed = count < value.len();
        self.truncated |= self.sealed;
        Ok(())
    }
}

/// 受控异常捕获边界（`Services::catch`）的执行结果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Caught {
    /// 被调用闭包正常返回（包含 `Ok(())` 或显式 `Err(TestError)`）。
    Returned(TestResult),
    /// 被调用闭包在执行期间触发了 Panic，并已被栈展开器捕获为有界消息。
    Panicked(PanicMessage),
}
/// 测试执行、后台子任务或清理阶段记录的具体失败原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    /// 函数通过 `Result::Err` 显式返回了 `TestError`。
    Returned(TestError),
    /// 执行期间触发了未预期的 Panic。
    Panic(PanicMessage),
    /// 用例声明了 `#[should_panic]`，但测试主体正常返回而未触发 Panic。
    ExpectedPanicMissing,
    /// 用例声明了 `#[should_panic(expected = "...")]`，但捕获到的 Panic 消息被截断或不包含期望子串。
    ExpectedPanicMismatch(PanicMessage),
    /// 测试主体通过 `expect!` / `expect_eq!` / `expect_panic` 记录了至少一次非致命软断言失败。
    Expectation,
}
/// 单个测试用例实例运行完成后的完整结果汇总。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CaseOutcome {
    /// 用例的最终状态判定（`Pass` / `Fail` / `Skip` / `Error`）。
    pub status: Status,
    /// 主执行通路（`Setup` 或 `Body` 阶段，或软断言汇总）记录的首要失败原因。
    pub primary: Option<Failure>,
    /// 后台子任务在 `Join` 阶段汇总报告的失败原因。
    pub child: Option<Failure>,
    /// 清理阶段（`Quiesce` / `Join` / `Cleanup`）发生的错误。
    pub cleanup: Option<Failure>,
    /// 本次用例执行期间累计失败的非致命软断言（`expect`）次数。
    pub expectations: u32,
}
/// 整个测试套件运行完成后的结果汇总。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SuiteOutcome {
    /// 套件级共享资源初始化（`Setup`）阶段的失败原因。
    pub setup: Option<Failure>,
    /// 套件内用例分派器（`CaseRunner::run`）执行阶段的失败原因。
    pub execution: Option<Failure>,
    /// 套件级后台子任务在 `Join` 阶段汇总报告的失败原因。
    pub child: Option<Failure>,
    /// 套件级共享资源清理阶段的失败原因。
    pub cleanup: Option<Failure>,
    /// 套件上下文自身累计失败的非致命软断言次数。
    pub expectations: u32,
}
/// 等待当前测试作用域内所有后台子任务退出执行栈后的汇总结果。
///
/// 当 `Services::join_children` 返回 `Ok(JoinOutcome)` 时，代表所有子任务均已安全退出各自栈帧；
/// 若子任务在运行期间曾发生断言失败或返回错误，则记录在 `failure` 字段中。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JoinOutcome {
    /// 已安全退出的后台子任务所报告的失败原因（若全部子任务均成功则为 `None`）。
    pub failure: Option<Failure>,
}

/// 测试运行器向 `ktest` 生命周期状态机提供的底层运行时服务接口。
///
/// 实现方必须满足以下安全与生命周期契约：
/// 1. `catch` 必须在当前受控的 Rust 执行栈上同步调用 `callback` 恰好一次，并在同一边界捕获 Panic 展开，不得跨越 C 栈帧、陷入（Trap）边界或未受控的异步线程展开。
/// 2. `stop_children` 与 `join_children` 负责管理当前测试作用域登记的全部后台子任务；`join_children` 返回 `Ok` 时，所有子任务必须已彻底退出各自任务栈，确保后续销毁夹具内存不会引发悬垂引用。
pub trait Services {
    /// 在当前执行栈的异常捕获边界内同步执行一次 `callback`，将正常返回值或 Panic 转换为 `Caught`。
    fn catch(&self, callback: &mut dyn FnMut() -> TestResult) -> Caught;
    /// 将诊断文本同步输出到测试运行器的可靠诊断通道（如直接串口输出）。
    fn diagnostic(&self, message: fmt::Arguments<'_>);
    /// 在进入对应生命周期阶段前通知运行器更新当前阶段标记（用于超时监控与故障定位）。
    fn phase(&self, phase: Phase);
    /// 向当前测试作用域派生的所有后台子任务发出协作式停止请求。
    fn stop_children(&self);
    /// 阻塞等待当前测试作用域登记的全部后台子任务执行完毕并退出栈帧；若无法安全回收则返回 `Err`。
    fn join_children(&self) -> Result<JoinOutcome, TestError>;
    /// 在清理阶段失败时保存完整的错误现场并立即终止虚拟机/运行环境，避免析构仍被后台异步访问的资源。
    fn terminate(&self, outcome: &CaseOutcome) -> !;
}

/// 单个测试用例或套件运行期间持有的执行上下文。
///
/// 封装了底层运行时服务接口、当前用例静态描述符、确定性随机种子以及软断言失败计数。
pub struct TestContext<'a> {
    /// 指向底层运行器服务实现的引用。
    services: &'a dyn Services,
    /// 当前正在执行的测试用例静态元数据描述符。
    pub case: &'static CaseDesc,
    /// 为当前测试用例实例分配的确定性伪随机种子。
    pub seed: u64,
    /// 当前用例在重复压力测试模式下的迭代序号（从 0 开始）。
    pub repeat: u32,
    /// 当前用例已累计触发的非致命软断言失败次数。
    expectations: u32,
}
impl<'a> TestContext<'a> {
    /// 为单次测试运行创建新的 `TestContext`，并将软断言失败计数初始化为 0。
    pub fn new(
        services: &'a dyn Services,
        case: &'static CaseDesc,
        seed: u64,
        repeat: u32,
    ) -> Self {
        Self {
            services,
            case,
            seed,
            repeat,
            expectations: 0,
        }
    }
    /// 向运行器的诊断通道同步输出格式化诊断消息。
    pub fn diagnostic(&self, message: fmt::Arguments<'_>) {
        self.services.diagnostic(message);
    }
    /// 检查布尔条件 `condition`；若为 `false`，则递增软断言失败计数并输出带源码位置的诊断信息，随后继续执行测试主体。
    pub fn expect(
        &mut self,
        condition: bool,
        source: &str,
        line: u32,
        message: fmt::Arguments<'_>,
    ) {
        if !condition {
            self.expectations = self.expectations.saturating_add(1);
            self.diagnostic(format_args!(
                "{}:{}: expectation failed: {}",
                source, line, message
            ));
        }
    }
    /// 比较两个值是否相等；若不相等则记录软断言失败并输出双方的 `Debug` 表示，随后继续执行。
    #[track_caller]
    pub fn expect_eq<T: PartialEq + fmt::Debug>(&mut self, actual: T, expected: T) {
        let location = core::panic::Location::caller();
        self.expect(
            actual == expected,
            location.file(),
            location.line(),
            format_args!("actual={:?}, expected={:?}", actual, expected),
        );
    }
    /// 在局部异常捕获边界内执行闭包 `body`，检查其是否触发了包含 `expected` 子串且未被截断的 Panic。
    ///
    /// 若闭包未发生 Panic、Panic 消息被截断或不包含 `expected` 子串，则记录一次软断言失败并继续后续测试逻辑。
    #[track_caller]
    pub fn expect_panic<T>(&mut self, expected: &str, body: impl FnOnce() -> T) {
        let services = self.services;
        let result = capture(services, || {
            body();
            Ok(())
        });
        let matched = matches!(&result, Err(Failure::Panic(message)) if !message.truncated && message.as_str().contains(expected));
        let location = core::panic::Location::caller();
        self.expect(
            matched,
            location.file(),
            location.line(),
            format_args!("expected panic containing {:?}, got {:?}", expected, result),
        );
    }
}

/// 测试套件共享资源或用例独立夹具的生命周期管理特征。
///
/// 夹具实例保存在测试主体的 Panic 捕获边界之外：
/// - 若 `set_up` 返回 `Err` 或发生 Panic，已分配的局部资源由 `set_up` 内部自行通过 RAII 释放，后续不会调用 `quiesce` 与 `tear_down`。
/// - 若 `set_up` 成功，即使测试主体发生 Panic，框架也会依次调用 `quiesce`（停止异步活动）并在后台子任务全部退出后调用 `tear_down`（安全销毁对象）。
pub trait Fixture<R = ()>: Sized {
    /// 根据测试上下文与上游只读共享资源 `resources` 构造完整的夹具实例。
    fn set_up(context: &mut TestContext<'_>, resources: &R) -> Result<Self, TestError>;
    /// 在测试主体结束后停止该夹具启动的异步生产源（如中断回调、工作队列任务或 DMA 传输）。
    ///
    /// 无异步活动的夹具可直接使用默认的空实现。
    fn quiesce(&mut self, _context: &mut TestContext<'_>) -> TestResult {
        Ok(())
    }
    /// 在所有后台子任务已退出执行栈后，按值消费并销毁夹具对象。
    fn tear_down(self, _context: &mut TestContext<'_>) -> TestResult {
        drop(self);
        Ok(())
    }
}
impl<R> Fixture<R> for () {
    /// 默认空夹具不分配任何资源，直接返回单元值 `()`。
    fn set_up(_context: &mut TestContext<'_>, _resources: &R) -> Result<Self, TestError> {
        Ok(())
    }
}

/// 套件内部测试用例的分派与遍历运行器接口。
///
/// 实现方在 `run_suite` 已构造好套件共享资源后被调用，负责遍历选中的用例索引、
/// 为每个实例创建独立的 `TestContext`，并在当前受控栈上同步调用 `dispatch` 闭包。
pub trait CaseRunner {
    /// 按运行器选定的用例索引列表依次调用 `dispatch` 执行测试实例。
    fn run(
        &mut self,
        cases: &'static [CaseDesc],
        dispatch: &mut dyn FnMut(&mut TestContext<'_>, usize) -> CaseOutcome,
    ) -> TestResult;
}

/// 将测试函数允许的返回类型（`()` 或 `TestResult`）统一转换为 `TestResult` 的辅助特征。
pub trait IntoTestResult {
    /// 将当前返回值转换为标准的 `TestResult`。
    fn into_test_result(self) -> TestResult;
}
impl IntoTestResult for () {
    /// 单元类型 `()` 映射为 `Ok(())`（是否通过仍会结合 `TestContext` 中的软断言账本共同判定）。
    fn into_test_result(self) -> TestResult {
        Ok(())
    }
}
impl IntoTestResult for TestResult {
    /// 原样返回测试函数显式给出的 `TestResult`。
    fn into_test_result(self) -> TestResult {
        self
    }
}

/// 在 `services.catch` 异常捕获边界内执行一次 `body`，并通过外部 `Option` 槽安全传出成功构造的具体返回值 `T`。
fn capture<T>(
    services: &dyn Services,
    body: impl FnOnce() -> Result<T, TestError>,
) -> Result<T, Failure> {
    let mut body = Some(body);
    let mut value = None;
    let result = services.catch(&mut || {
        value = Some(body.take().expect("catch callback invoked twice")()?);
        Ok(())
    });
    match result {
        Caught::Returned(Ok(())) => Ok(value.expect("catch callback was not invoked")),
        Caught::Returned(Err(error)) => Err(Failure::Returned(error)),
        Caught::Panicked(message) => Err(Failure::Panic(message)),
    }
}

/// 处理清理阶段（`Quiesce` / `Join` / `Cleanup`）发生的错误：记录诊断日志并调用 `services.terminate` 终止运行环境，避免在后台仍可能访问资源时强行 Drop 夹具。
fn cleanup_failure(
    context: &TestContext<'_>,
    primary: Option<Failure>,
    child: Option<Failure>,
    error: Failure,
) -> ! {
    let outcome = CaseOutcome {
        status: Status::Error,
        primary,
        child,
        cleanup: Some(error),
        expectations: context.expectations,
    };
    context.diagnostic(format_args!(
        "cleanup failed: {:?}; primary: {:?}; child: {:?}",
        error, primary, child
    ));
    context.services.terminate(&outcome)
}

/// 执行单个测试用例实例的完整五阶段生命周期（`Setup` -> `Body` -> `Quiesce` -> `Join` -> `Cleanup`）。
///
/// # 执行语义
/// - 每个阶段均运行在独立的 `capture` 异常捕获边界内，夹具对象 `F` 保存在主体捕获边界之外。
/// - `#[should_panic]` 仅对 `Body` 阶段未截断的 Panic 消息生效；`Setup`、后台子任务及清理阶段的异常均独立报告。
/// - 声明了 `expected_fatal` 的用例若从 `Body` 正常返回，则视为未触发预期致命异常并记为失败，但仍会走完完整的清理流程。
pub fn run_case<F: Fixture<R>, R>(
    context: &mut TestContext<'_>,
    resources: &R,
    body: impl FnOnce(&mut TestContext<'_>, &mut F) -> TestResult,
) -> CaseOutcome {
    let services = context.services;
    // Step 1: 进入 Setup 阶段，构造用例级独立夹具 F。
    services.phase(Phase::Setup);
    let fixture = capture(services, || F::set_up(context, resources));
    let mut primary = None;
    let mut setup_failed = false;
    let mut fixture = match fixture {
        Ok(value) => Some(value),
        Err(error) => {
            primary = Some(error);
            setup_failed = true;
            None
        }
    };
    // Step 2: 若夹具构造成功，进入 Body 阶段执行测试主体，并核对 should_panic 与 expected_fatal 预期。
    if let Some(value) = fixture.as_mut() {
        services.phase(Phase::Body);
        let result = capture(services, || body(context, value));
        primary = match (context.case.should_panic, result) {
            (Some(expected), Err(Failure::Panic(message)))
                if !message.truncated && message.as_str().contains(expected) =>
            {
                None
            }
            (Some(_), Err(Failure::Panic(message))) => {
                Some(Failure::ExpectedPanicMismatch(message))
            }
            (Some(_), Ok(())) => Some(Failure::ExpectedPanicMissing),
            (_, Err(error)) => Some(error),
            (None, Ok(())) if context.case.expected_fatal.is_some() => {
                Some(Failure::Returned(TestError("expected fatal did not occur")))
            }
            (None, Ok(())) => None,
        };
    }
    // Step 3: 通知当前测试作用域派生的后台子任务停止运行，并对已构造的夹具执行 Quiesce 静止化。
    services.stop_children();
    if let Some(value) = fixture.as_mut() {
        services.phase(Phase::Quiesce);
        if let Err(error) = capture(services, || value.quiesce(context)) {
            cleanup_failure(context, primary, None, error);
        }
    }
    // Step 4: 进入 Join 阶段，等待所有后台子任务彻底退出各自执行栈。
    services.phase(Phase::Join);
    let child = match capture(services, || services.join_children()) {
        Ok(joined) => joined.failure,
        Err(error) => cleanup_failure(context, primary, None, error),
    };
    // Step 5: 所有后台子任务均已退出后，进入 Cleanup 阶段安全消费并释放夹具。
    if let Some(value) = fixture {
        services.phase(Phase::Cleanup);
        if let Err(error) = capture(services, || value.tear_down(context)) {
            cleanup_failure(context, primary, child, error);
        }
    }
    // Step 6: 汇总软断言（expect）失败计数与各阶段结果，生成最终的 CaseOutcome。
    if primary.is_none() && context.expectations != 0 {
        primary = Some(Failure::Expectation);
    }
    CaseOutcome {
        status: if setup_failed {
            Status::Error
        } else if primary.is_some() || child.is_some() {
            Status::Fail
        } else {
            Status::Pass
        },
        primary,
        child,
        cleanup: None,
        expectations: context.expectations,
    }
}

/// 执行整个测试套件的生命周期：构造套件级共享资源 `R`，将只读引用 `&R` 分派给选中的各个测试用例，最后执行套件资源的静止化、子任务回收与销毁。
pub fn run_suite<R: Fixture<()>>(
    context: &mut TestContext<'_>,
    runner: &mut dyn CaseRunner,
    cases: &'static [CaseDesc],
    dispatch: fn(&mut TestContext<'_>, &R, usize) -> CaseOutcome,
) -> SuiteOutcome {
    let services = context.services;
    // Step 1: 构造套件级共享资源 R；若初始化失败，回收可能已派生的子任务后直接返回。
    services.phase(Phase::Setup);
    let mut resources = match capture(services, || R::set_up(context, &())) {
        Ok(resources) => resources,
        Err(error) => {
            services.stop_children();
            services.phase(Phase::Join);
            let child = match capture(services, || services.join_children()) {
                Ok(joined) => joined.failure,
                Err(cleanup) => cleanup_failure(context, Some(error), None, cleanup),
            };
            return SuiteOutcome {
                setup: Some(error),
                execution: None,
                child,
                cleanup: None,
                expectations: context.expectations,
            };
        }
    };
    // Step 2: 套件资源就绪后，委托 CaseRunner 依次运行选中的测试用例实例，每个用例共享只读引用 &resources。
    let execution = capture(services, || {
        runner.run(cases, &mut |case_context, index| {
            dispatch(case_context, &resources, index)
        })
    });
    let primary = execution.err();
    // Step 3: 用例执行完毕后，停止套件级后台子任务并静止化套件共享资源。
    services.stop_children();
    services.phase(Phase::Quiesce);
    if let Err(error) = capture(services, || resources.quiesce(context)) {
        cleanup_failure(context, primary, None, error);
    }
    // Step 4: 等待套件级后台子任务全部退出执行栈，随后安全销毁套件共享资源。
    services.phase(Phase::Join);
    let child = match capture(services, || services.join_children()) {
        Ok(joined) => joined.failure,
        Err(error) => cleanup_failure(context, primary, None, error),
    };
    services.phase(Phase::Cleanup);
    if let Err(error) = capture(services, || resources.tear_down(context)) {
        cleanup_failure(context, primary, child, error);
    }
    let primary = primary.or(if context.expectations != 0 {
        Some(Failure::Expectation)
    } else {
        None
    });
    SuiteOutcome {
        setup: None,
        execution: primary,
        child,
        cleanup: None,
        expectations: context.expectations,
    }
}

/// 非致命软断言宏：检查布尔表达式 `$condition`，若为 `false` 则在 `$context` 中记录失败位置与诊断消息，但不会中断当前测试函数执行。
#[macro_export]
macro_rules! expect {
    ($context:expr, $condition:expr $(,)?) => { $context.expect($condition, file!(), line!(), core::format_args!("{}", core::stringify!($condition))) };
    ($context:expr, $condition:expr, $($message:tt)+) => { $context.expect($condition, file!(), line!(), core::format_args!($($message)+)) };
}
/// 非致命相等性软断言宏：对 `$actual` 与 `$expected` 分别求值一次并按引用比较；若不相等则输出双方的 `Debug` 值并记录软断言失败。
#[macro_export]
macro_rules! expect_eq {
    ($context:expr, $actual:expr, $expected:expr $(,)?) => {{
        match (&$actual, &$expected) {
            (actual, expected) => $context.expect(
                actual == expected,
                file!(),
                line!(),
                core::format_args!("actual={:?}, expected={:?}", actual, expected),
            ),
        }
    }};
}
