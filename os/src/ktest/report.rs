//! 内核测试框架（`ktest`）的 KTAP / JSON 控制协议同步串口报告模块。
//!
//! ## 模块职责与架构定位
//! 本模块负责将测试套件与测试用例的执行生命周期事件、诊断日志、失败详情以及致命异常（Fatal Event），
//! 按照宿主机测试运行器约定的 KTAP（Kernel Test Anything Protocol）与扩展 `# KTEST ...` 控制行格式，
//! 直接同步输出到唯一的硬件控制串口（UART），不经过可能丢弃消息的异步日志环形缓冲区。
//! - **上游调用者**：`ktest::runner`、`ktest::registry`、`ktest::context` 及内核致命异常入口 `ktest::fatal_event`。
//! - **下游依赖**：`ktest::control_write`（常规同步串口写入）、`ktest::emergency_write`（无锁应急串口写入）、`arch::test_shutdown` 及 `arch::emergency_shutdown`。
//!
//! ## 核心设计与输出协议结构
//! 一次完整的测试套件运行在串口上产生的控制协议流如下所示：
//! ```text
//! KTAP version 1
//! 1..1
//!   # Subtest: <suite_name>
//!   KTAP version 1
//!   1..<case_count>
//!   # KTEST suite-begin setup
//!   # KTEST case-begin <case_id>
//!   # KTEST phase <case_id> setup | body | quiesce | join | cleanup
//!   # KTEST armed <case_id> <expected_fatal_kind>          (可选：仅当声明了 expected_fatal 时)
//!   # diagnostic: <line_1>                                 (可选：多行诊断自动按行添加前缀，防止注入伪造控制行)
//!   ok | not ok <ordinal> <case_name> [# SKIP ... | # ERROR]
//! ok | not ok 1 <suite_name>
//! # KTEST complete
//! ```
//! 若清理阶段（`Quiesce` / `Join` / `Cleanup`）未能正常收敛，或触发了底层致命故障，则输出 `# KTEST case-abort`、`# KTEST suite-error` 或 `# KTEST fatal` 并立即调用底层应急关机。
//!
//! ## 锁获取顺序与并发契约
//! - `OUTPUT` 自旋锁：保护常规测试协议行（`write`、`diagnostic`、`result`、`case_abort`、`suite_abort`）的串口输出，避免多核并发打印的诊断行与控制行交错。持锁期间仅执行同步 UART 字符输出，不可睡眠，也不获取其他业务锁。
//! - `fatal_event` 处于不可恢复的异常或崩溃上下文，不获取 `OUTPUT` 锁，直接通过 `super::emergency_write` 执行无锁应急输出并关机，以防在当前 CPU 已持有 `OUTPUT` 锁时触发死锁。

use ::ktest::{CaseDesc, CaseOutcome, Failure, Phase, Status};
use core::fmt::{self, Write};

/// 串行化常规测试控制行与诊断输出的全局自旋锁，防止多核输出交错破坏协议行格式。
static OUTPUT: crate::sync::SpinLock<()> = crate::sync::SpinLock::new(());

/// 在持有 `OUTPUT` 自旋锁的前提下，将格式化文本片段同步写入控制串口。
///
/// 调用上下文：任意普通测试上下文或关中断上下文（非重入致命异常路径）。
pub(crate) fn write(args: fmt::Arguments<'_>) {
    let _output = OUTPUT.lock();
    super::control_write(args);
}

/// 将字符串 `text` 按 JSON 字符串规范逐字符流式转义并写入 `output`（无需在堆上分配临时缓冲区）。
fn json_escape(output: &mut impl Write, text: &str) -> fmt::Result {
    for ch in text.chars() {
        match ch {
            '"' => output.write_str("\\\"")?,
            '\\' => output.write_str("\\\\")?,
            '\n' => output.write_str("\\n")?,
            '\r' => output.write_str("\\r")?,
            '\t' => output.write_str("\\t")?,
            c if c <= '\u{1f}' => write!(output, "\\u{:04x}", c as u32)?,
            c => output.write_char(c)?,
        }
    }
    Ok(())
}

/// 将静态字符串切片（元组字段 `0`）包装为带双引号的流式 JSON 字符串输出适配器。
pub(crate) struct Json(pub(crate) &'static str);

impl fmt::Display for Json {
    /// 输出包裹在双引号内的 JSON 转义字符串。
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_char('"')?;
        json_escape(f, self.0)?;
        f.write_char('"')
    }
}

/// 将可选静态字符串切片（元组字段 `0`）包装为 JSON 可空字符串（`"..."` 或 `null`）的输出适配器。
pub(crate) struct Nullable(pub(crate) Option<&'static str>);

impl fmt::Display for Nullable {
    /// 当内部为 `Some` 时委派给 `Json` 输出，为 `None` 时输出 `"null"`。
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(s) => fmt::Display::fmt(&Json(s), f),
            None => f.write_str("null"),
        }
    }
}

/// 将写入 `fmt::Formatter` 的任意格式化分片实时进行 JSON 字符转义的 `fmt::Write` 适配器。
///
/// 用于将 `Failure` 的 `Debug` 格式化输出安全嵌入单行 JSON 中（元组字段 `0` 为底层目标格式化器的可变借用），避免异常消息中的换行或引号破坏控制协议行。
struct JsonEscape<'a, 'b>(&'a mut fmt::Formatter<'b>);
impl Write for JsonEscape<'_, '_> {
    /// 对写入的每个文本片段执行 JSON 字符转义后转发给底层格式化器。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        json_escape(self.0, text)
    }
}

/// 将可选的测试失败原因 `Option<Failure>`（元组字段 `0`）格式化为 JSON 字符串或 `null` 的包装器。
struct JsonFailure(Option<Failure>);

impl fmt::Display for JsonFailure {
    /// 若存在失败原因，则将其 `Debug` 文本经 JSON 转义后包裹在双引号中输出；否则输出 `"null"`。
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            None => f.write_str("null"),
            Some(failure) => {
                f.write_char('"')?;
                write!(JsonEscape(f), "{:?}", failure)?;
                f.write_char('"')
            }
        }
    }
}

/// 当测试用例或套件在清理阶段无法安全收敛而中止时，用于输出完整失败记录（元组字段 `0` 为包含各阶段失败原因与期望计数的 `CaseOutcome` 引用）的 JSON 格式化包装器。
struct AbortLedger<'a>(&'a CaseOutcome);

impl fmt::Display for AbortLedger<'_> {
    /// 将主线程错误、子线程错误、清理错误、未满足期望计数及中止原因一并格式化为单行 JSON 对象。
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let outcome = self.0;
        write!(
            f,
            "{{\"primary\":{},\"child\":{},\"cleanup\":{},\"expectations\":{},\"reason\":{}}}",
            JsonFailure(outcome.primary),
            JsonFailure(outcome.child),
            JsonFailure(outcome.cleanup),
            outcome.expectations,
            Json("cleanup cannot safely converge; terminating without dropping live resources")
        )
    }
}

/// 输出 KTAP 顶层头部以及当前测试套件（Subtest）的起始声明行与计划用例总数。
pub(crate) fn suite_begin(name: &str, count: usize) {
    write(format_args!(
        "KTAP version 1\n1..1\n  # Subtest: {}\n  KTAP version 1\n  1..{}\n",
        name, count
    ));
}

/// 在测试用例开始执行任何准备（Setup）动作前，向控制串口输出 `# KTEST case-begin <id>` 控制行。
pub(crate) fn begin(id: u32) {
    write(format_args!("  # KTEST case-begin {}\n", id));
}

/// 向控制串口报告测试用例 `id` 当前进入的生命周期阶段（`setup` / `body` / `quiesce` / `join` / `cleanup`）。
///
/// 调用方负责保证阶段按生命周期顺序单调推进，不重复或倒退。
pub(crate) fn phase(id: u32, phase: Phase) {
    let name = match phase {
        Phase::Setup => "setup",
        Phase::Body => "body",
        Phase::Quiesce => "quiesce",
        Phase::Join => "join",
        Phase::Cleanup => "cleanup",
    };
    write(format_args!("  # KTEST phase {} {}\n", id, name));
}

/// 向控制串口输出 `# KTEST armed <id> <kind>`，通知宿主机当前用例已进入预期致命故障（`expected_fatal`）的监测窗口。
pub(crate) fn armed(id: u32, kind: &str) {
    write(format_args!("  # KTEST armed {} {}\n", id, kind));
}

/// KTAP 诊断行的标准前缀，用于将诊断消息隔离在非控制命令命名空间内。
const DIAGNOSTIC_PREFIX: &str = "  # diagnostic: ";

/// 为多行格式化参数 `fmt::Arguments`（元组字段 `0`）自动在首行及每个换行符后冠以 `DIAGNOSTIC_PREFIX` 前缀的显示适配器。
struct Diagnostic<'a>(fmt::Arguments<'a>);
impl fmt::Display for Diagnostic<'_> {
    /// 先输出首行诊断前缀，再通过 `DiagnosticWriter` 流式转发正文并在每次换行后补齐前缀。
    fn fmt(&self, output: &mut fmt::Formatter<'_>) -> fmt::Result {
        output.write_str(DIAGNOSTIC_PREFIX)?;
        DiagnosticWriter(output).write_fmt(self.0)
    }
}

/// 在每个换行符（`\n`）之后自动插入 `DIAGNOSTIC_PREFIX` 的流式写入包装器（元组字段 `0` 为底层目标格式化器的可变借用）。
struct DiagnosticWriter<'a, 'b>(&'a mut fmt::Formatter<'b>);
impl Write for DiagnosticWriter<'_, '_> {
    /// 逐段扫描换行符并在每个 `\n` 后追加诊断前缀，防止包含换行的诊断或异常消息伪造出新的 KTAP 控制行。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for part in text.split_inclusive('\n') {
            self.0.write_str(part)?;
            if part.ends_with('\n') {
                self.0.write_str(DIAGNOSTIC_PREFIX)?;
            }
        }
        Ok(())
    }
}

/// 向控制串口同步输出一条测试诊断消息（自动按行添加 `# diagnostic: ` 前缀，无需堆内存分配）。
pub(crate) fn diagnostic(args: fmt::Arguments<'_>) {
    let _output = OUTPUT.lock();
    super::control_write(format_args!("{}\n", Diagnostic(args)));
}

/// 将测试用例结果（`CaseOutcome`）中记录的主线程错误、子线程错误、清理阶段错误及未满足的 `expect!` 计数分别作为独立诊断行输出。
pub(crate) fn failures(outcome: &CaseOutcome) {
    for (name, failure) in [
        ("primary", outcome.primary),
        ("child", outcome.child),
        ("cleanup", outcome.cleanup),
    ] {
        if let Some(error) = failure {
            diagnostic(format_args!("{}: {:?}", name, error));
        }
    }
    if outcome.expectations != 0 {
        diagnostic(format_args!("expectations: {}", outcome.expectations));
    }
}

/// 输出单个测试用例的最终 KTAP 结果行（`ok` / `not ok`），并在此前先输出所有关联的失败诊断详情。
///
/// 调用上下文：调用方已完成该用例适用的协作停止、子任务汇合（Join）与夹具清理（或因静态跳过/Setup 失败而在 Setup 阶段提前结束）。
pub(crate) fn result(ordinal: usize, case: &CaseDesc, outcome: &CaseOutcome, skip: Option<&str>) {
    failures(outcome);
    let ok = matches!(outcome.status, Status::Pass | Status::Skip);
    let _output = OUTPUT.lock();
    super::control_write(format_args!(
        "  {} {} {}",
        if ok { "ok" } else { "not ok" },
        ordinal,
        case.name
    ));
    match outcome.status {
        Status::Skip => {
            super::control_write(format_args!(" # SKIP"));
            if let Some(reason) = skip {
                super::control_write(format_args!(" {}", reason));
            }
        }
        Status::Error => super::control_write(format_args!(" # ERROR")),
        _ => {}
    }
    super::control_write(format_args!("\n"));
}

/// 输出测试套件的顶层 KTAP 汇总结果与 `# KTEST complete` 结束标志，随后通过架构测试关机接口正常退出虚拟机。
pub(crate) fn complete(name: &str, passed: bool) -> ! {
    write(format_args!(
        "{} 1 {}\n# KTEST complete\n",
        if passed { "ok" } else { "not ok" },
        name
    ));
    crate::arch::test_shutdown()
}

/// 报告测试套件级别的阶段性错误（`# KTEST suite-error`），随后将该套件标记为失败并关机退出。
///
/// 尚未开始执行的其余测试用例由宿主机统一标记为未运行（`NotRun`）。
pub(crate) fn suite_error(name: &str, stage: &str, failure: Failure) -> ! {
    write(format_args!(
        "  # KTEST suite-error {} {:?}\n",
        stage, failure
    ));
    complete(name, false)
}

/// 当单个测试用例在清理阶段（`Quiesce` / `Join` / `Cleanup`）发生失败或无法收敛时，在单次锁临界区内输出 `# KTEST case-abort` 与完整失败记录并立即应急关机。
///
/// 此时不再继续析构可能仍有后台任务引用的存活资源，以避免释放后使用（UAF）。
pub(crate) fn case_abort(name: &str, id: u32, phase: Phase, outcome: &CaseOutcome) -> ! {
    let phase = match phase {
        Phase::Quiesce => "quiesce",
        Phase::Join => "join",
        Phase::Cleanup => "cleanup",
        _ => super::context::fatal("case abort outside cleanup phase"),
    };
    let _output = OUTPUT.lock();
    super::control_write(format_args!(
        "  # KTEST case-abort {} {} {}\nnot ok 1 {}\n# KTEST complete\n",
        id,
        phase,
        AbortLedger(outcome),
        name
    ));
    crate::arch::emergency_shutdown()
}

/// 当测试套件级共享夹具在清理阶段无法安全收敛时，输出包含完整失败记录的 `# KTEST suite-error` 并立即应急关机。
pub(crate) fn suite_abort(name: &str, stage: &str, outcome: &CaseOutcome) -> ! {
    let _output = OUTPUT.lock();
    super::control_write(format_args!(
        "  # KTEST suite-error {} {}\nnot ok 1 {}\n# KTEST complete\n",
        stage,
        AbortLedger(outcome),
        name
    ));
    crate::arch::emergency_shutdown()
}

/// 在发生致命内核故障（如未捕获异常、非法指令、中断上下文违规调度、双重 Panic 等）时输出 `# KTEST fatal` 事件并立即执行应急关机。
///
/// 调用上下文：由底层异常处理程序或致命错误入口调用。函数内部不获取 `OUTPUT` 锁，直接通过 `super::emergency_write` 执行单次无锁应急串口输出。
pub(crate) fn fatal_event(kind: &'static str, message: fmt::Arguments<'_>) -> ! {
    if let Some(id) = super::context::active_case() {
        super::emergency_write(format_args!(
            "\n  # KTEST fatal {} {}\n{}\n",
            id,
            kind,
            Diagnostic(message)
        ));
    } else {
        super::emergency_write(format_args!(
            "\n{}\n",
            Diagnostic(format_args!("fatal outside case: {}: {}", kind, message))
        ));
    }
    crate::arch::emergency_shutdown()
}
