//! 唯一 UART 的同步测试协议；不经过可丢弃日志队列。

use ::ktest::{CaseDesc, CaseOutcome, Failure, Phase, Status};
use core::fmt::{self, Write};

static OUTPUT: crate::sync::SpinLock<()> = crate::sync::SpinLock::new(());

/// 将同步格式化片段交给父提供的真实控制 UART。
pub(crate) fn write(args: fmt::Arguments<'_>) {
    let _output = OUTPUT.lock();
    super::control_write(args);
}

/// 流式转义任意格式化片段；不为名称或失败账本申请缓冲。
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

/// 流式 JSON 字符串转义，不为长名称申请缓冲。
pub(crate) struct Json(pub(crate) &'static str);

impl fmt::Display for Json {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_char('"')?;
        json_escape(f, self.0)?;
        f.write_char('"')
    }
}

/// 输出 JSON 可空字符串。
pub(crate) struct Nullable(pub(crate) Option<&'static str>);

impl fmt::Display for Nullable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(s) => fmt::Display::fmt(&Json(s), f),
            None => f.write_str("null"),
        }
    }
}

/// Debug 的每个片段经过 JSON 转义，panic 正文不会破坏控制行。
struct JsonEscape<'a, 'b>(&'a mut fmt::Formatter<'b>);
impl Write for JsonEscape<'_, '_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        json_escape(self.0, text)
    }
}
struct JsonFailure(Option<Failure>);

impl fmt::Display for JsonFailure {
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

/// 同一事件保存独立失败和 expect 计数，不用任一失败覆盖其它账本。
struct AbortLedger<'a>(&'a CaseOutcome);

impl fmt::Display for AbortLedger<'_> {
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

/// 按 KTAP 层级开始唯一 suite。
pub(crate) fn suite_begin(name: &str, count: usize) {
    write(format_args!(
        "KTAP version 1\n1..1\n  # Subtest: {}\n  KTAP version 1\n  1..{}\n",
        name, count
    ));
}

/// 在任何 suite 或 case setup 之前登记开始。
pub(crate) fn begin(id: u32) {
    write(format_args!("  # KTEST case-begin {}\n", id));
}

/// 公布真实阶段，caller 负责不重复或倒退。
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

/// 只声明期望，不把元数据声明当作已经发生的故障。
pub(crate) fn armed(id: u32, kind: &str) {
    write(format_args!("  # KTEST armed {} {}\n", id, kind));
}

const DIAGNOSTIC_PREFIX: &str = "  # diagnostic: ";

/// 正文直接流入调用者的 formatter，普通与应急输出共享同一逐行隔离规则。
struct Diagnostic<'a>(fmt::Arguments<'a>);
impl fmt::Display for Diagnostic<'_> {
    fn fmt(&self, output: &mut fmt::Formatter<'_>) -> fmt::Result {
        output.write_str(DIAGNOSTIC_PREFIX)?;
        DiagnosticWriter(output).write_fmt(self.0)
    }
}

struct DiagnosticWriter<'a, 'b>(&'a mut fmt::Formatter<'b>);
impl Write for DiagnosticWriter<'_, '_> {
    /// 每个换行之后恢复诊断前缀，避免作者消息伪造控制行。
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

/// 同步诊断保持全文；每行进入非控制命名空间，不缓冲或重新分配作者消息。
pub(crate) fn diagnostic(args: fmt::Arguments<'_>) {
    let _output = OUTPUT.lock();
    super::control_write(format_args!("{}\n", Diagnostic(args)));
}

/// 保存独立主、子、清理错误；仅一份实际结果。
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

/// 发布唯一 KTAP 终态；调用方先完成适用的停止、join 和 fixture 清理。
/// 静态 Skip 或准备失败可在 setup 结束，独立失败正文不进入结果单行。
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

/// 输出完整 Suite 终态后通过 QEMU 测试设备结束本次运行。
pub(crate) fn complete(name: &str, passed: bool) -> ! {
    write(format_args!(
        "{} 1 {}\n# KTEST complete\n",
        if passed { "ok" } else { "not ok" },
        name
    ));
    crate::arch::test_shutdown()
}

/// 明确保存独立 SuiteError；未开始项由宿主记 NotRun，不伪造 case 结果。
pub(crate) fn suite_error(name: &str, stage: &str, failure: Failure) -> ! {
    write(format_args!(
        "  # KTEST suite-error {} {:?}\n",
        stage, failure
    ));
    complete(name, false)
}

/// 未收敛实例只有 abort 终结；同一次锁内报告外层失败，再真实关机且不展开。
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

/// suite 哨兵保留独立 SuiteError；原因正文包含完整账本，不占用 case 身份。
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

/// 真实错误控制者调用；类型来自实际故障而非 armed 期望。
/// 正文流式适配仍在一次无锁应急写入内完成，整条消息共享原 UART 有界预算。
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
