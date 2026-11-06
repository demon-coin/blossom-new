//! `kselftest` TAP v13（Test Anything Protocol）标准输出有界流式解析器。
//!
//! # 模块职责与架构定位
//! 本模块供 [`crate::process`] 在运行 `Protocol::Tap` 协议的测试程序时实时消费子进程的标准输出（`stdout`），并将其与子进程退出状态（[`ExitOutcome`]）结合，判定最终的测试结论（[`Verdict`]）与子项统计（[`TapSummary`]）。
//!
//! # 核心机制与流式解析状态机
//!
//! ## 1. 固定容量行前缀缓冲（4096 字节上限）
//! 为防止测试程序输出超长日志行耗尽运行器内存，[`TapObserver`] 预分配一个 4096 字节的单行缓冲区：
//! - 当单行字节数不超过 4096 时，完整缓存在 `line` 中并在遇到 `\n`（或进程退出）时解析；
//! - 当单行超过 4096 字节时，置位 `overflow = true` 并丢弃后续字节直至行尾 `\n`。若被截断的行恰好是 TAP 控制行（`TAP version`、`1..N`、`ok`/`not ok`），则立即记录格式损坏错误（`error`）。
//!
//! ## 2. TAP 控制行与结果判定状态机
//! ```text
//! stdout 字节流 ──► feed() 按 '\n' 切行 ──► consume_line()
//!                                               │
//!        ┌──────────────────────────────────────┼──────────────────────────────────────┐
//!        ▼                                      ▼                                      ▼
//!  "TAP version 13"                      "1..N [# SKIP ...]"                  "ok / not ok <num> [# <DIR>]"
//!  - 必须在 plan 与结果之前出现          - 只能出现一次                       - 序号必须严格递增 (count + 1)
//!  - 仅接受版本 13                       - 若出现在结果之后，记 trailing_plan   - 特例：ksft_exit_skip() 会产生
//!                                        - 若为 "1..0 # SKIP"，记 zero_skip     序号为 count + 2 的终末 SKIP
//!                                                                               （记录 terminal_skip，要求退出码为 4）
//!
//! 进程结束 ──► finish(outcome)
//!                ├─ Signaled           ──► Verdict::Signaled
//!                ├─ TimedOut           ──► Verdict::TimedOut
//!                ├─ SpawnError/IoError ──► Verdict::InfrastructureError
//!                └─ Exited(code)       ──► exited(code)
//!                     ├─ bailed_out / failed > 0 / xpassed > 0  ──► Verdict::Failed
//!                     ├─ 格式错误 / 计划未完成 / 非法终末缺号   ──► Verdict::Broken
//!                     ├─ code == 0 且全部跳过 (或 zero_skip)    ──► Verdict::Skip
//!                     ├─ code == 0                              ──► Verdict::Pass
//!                     ├─ code == 4 (KSFT_SKIP) 且存在跳过项     ──► Verdict::Skip
//!                     └─ 其他非零退出码                         ──► Verdict::Failed
//! ```
//!
//! # 锁获取顺序与并发契约
//! [`TapObserver`] 实例由单个子进程监控循环独占持有，按顺序接收读取到的 `stdout` 字节块，不涉及跨线程共享或锁同步。

use crate::outcome::{ExitOutcome, ProtocolResult, TapSummary, Verdict};

/// TAP v13 标准输出流式观察器。
pub(crate) struct TapObserver {
    /// 累计解析出的 TAP 计划数、各类子项计数、中止标志与首个协议错误信息。
    summary: TapSummary,
    /// 当前正在拼装的单行字节缓冲区（容量上限为 4096 字节）。
    line: Vec<u8>,
    /// 当前行长度是否已超过 4096 字节上限而发生截断。
    overflow: bool,
    /// 是否已经解析过 `TAP version 13` 头部行。
    version: bool,
    /// 已消费的 `ok` / `not ok` 测试结果行总数。
    count: u64,
    /// 是否遇到了 `ksft_exit_skip()` 特有的跳号终末跳过行（序号为 `count + 2`）。
    terminal_skip: bool,
    /// 是否遇到了空跳过计划行（`1..0 # SKIP ...`）。
    zero_skip: bool,
    /// 测试计划行（`1..N`）是否出现在至少一条测试结果行之后（后置计划后不允许再出现新结果）。
    trailing_plan: bool,
}

impl TapObserver {
    /// 创建一个具有 4096 字节预分配行缓冲区的 [`TapObserver`]。
    pub(crate) fn new() -> Self {
        Self {
            summary: TapSummary::default(),
            line: Vec::with_capacity(4096),
            overflow: false,
            version: false,
            count: 0,
            terminal_skip: false,
            zero_skip: false,
            trailing_plan: false,
        }
    }

    /// 记录首个遇到的 TAP 协议格式错误信息（若已有错误则保留最早的一条）。
    fn invalid(&mut self, message: &str) {
        if self.summary.error.is_none() {
            self.summary.error = Some(message.into());
        }
    }

    /// 将新读取到的 `stdout` 字节流推入观察器，按换行符 `\n` 切分并触发单行解析。
    pub(crate) fn feed(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if byte == b'\n' {
                self.consume();
            } else if self.line.len() < 4096 {
                self.line.push(byte);
            } else {
                self.overflow = true;
            }
        }
    }

    /// 取出当前行缓冲区，剥离可选的 `\r` 后调用 [`Self::consume_line`] 解析，随后清空并复用原缓冲区容量。
    fn consume(&mut self) {
        let mut buffer = std::mem::take(&mut self.line);
        let line = buffer.strip_suffix(b"\r").unwrap_or(&buffer);
        self.consume_line(line);
        buffer.clear();
        self.line = buffer;
        self.overflow = false;
    }

    /// 解析单行文本，识别 `Bail out!`、`TAP version`、`1..N` 计划行及 `ok`/`not ok` 结果行。
    fn consume_line(&mut self, line: &[u8]) {
        // Step 1: 检查是否为紧急中止行 `Bail out!`，并校验控制行的 UTF-8 编码合法性。
        if line.starts_with(b"Bail out!") {
            self.summary.bailed_out = true;
            return;
        }
        let Ok(line) = std::str::from_utf8(line) else {
            if line.starts_with(b"ok ")
                || line.starts_with(b"not ok ")
                || line.starts_with(b"1..")
                || line.starts_with(b"TAP version")
            {
                self.invalid("non-UTF-8 TAP control line");
            }
            return;
        };
        // Step 2: 解析版本声明行 `TAP version 13`（必须位于计划与所有结果之前，且不可重复或截断）。
        if line.starts_with("TAP version") {
            if self.version
                || self.summary.plan.is_some()
                || self.count != 0
                || line != "TAP version 13"
                || self.overflow
            {
                self.invalid("invalid or repeated TAP version");
            }
            self.version = true;
            return;
        }
        // Step 3: 解析测试计划行 `1..N`（支持前置或后置计划，以及 `1..0 # SKIP` 全跳过声明）。
        if let Some(plan) = line.strip_prefix("1..") {
            let (number, comment) = plan.split_once('#').unwrap_or((plan, ""));
            let count = number.trim().parse::<u64>();
            match count {
                Ok(count) if self.summary.plan.is_none() && !self.overflow => {
                    self.summary.plan = Some(count);
                    self.zero_skip = count == 0 && directive(comment, "SKIP");
                    self.trailing_plan = self.count != 0;
                    if count == 0 && (!self.zero_skip || self.count != 0) {
                        self.invalid("zero plan requires an empty SKIP");
                    }
                }
                _ => self.invalid("invalid or repeated TAP plan"),
            }
            return;
        }
        // Step 4: 解析 `ok` / `not ok` 测试结果行，校验序号连续性并按 `# SKIP` / `# XFAIL` / `# XPASS` 指令更新计数。
        let (ok, result) = if let Some(result) = line.strip_prefix("not ok ") {
            (false, result)
        } else if let Some(result) = line.strip_prefix("ok ") {
            (true, result)
        } else {
            return;
        };
        if self.overflow || self.terminal_skip || self.trailing_plan || self.zero_skip {
            self.invalid("truncated or late TAP result");
            return;
        }
        let (number, rest) = result
            .split_once(char::is_whitespace)
            .unwrap_or((result, ""));
        let Ok(number) = number.parse::<u64>() else {
            self.invalid("TAP result requires a number");
            return;
        };
        let comment = rest.split_once('#').map_or("", |(_, comment)| comment);
        let skip = ok && directive(comment, "SKIP");
        // kselftest 的 `ksft_exit_skip()` 宏会先递增跳过计数再打印 `ok <1 + test_num> # SKIP`，导致行号比预期多 1；仅在进程以退出码 4 结束且无后续结果时允许该跳号。
        if skip && number == self.count + 2 {
            self.terminal_skip = true;
        } else if number != self.count + 1 {
            self.invalid("duplicate or out-of-order TAP result");
        }
        self.count += 1;
        if !ok {
            self.summary.failed += 1;
        } else if skip {
            self.summary.skipped += 1;
        } else if directive(comment, "XFAIL") {
            self.summary.xfailed += 1;
        } else if directive(comment, "XPASS") {
            self.summary.xpassed += 1;
        } else {
            self.summary.passed += 1;
        }
    }

    /// 在子进程结束时刷新末尾未换行的残留输出，并结合进程退出状态生成最终 [`ProtocolResult`]。
    pub(crate) fn finish(mut self, outcome: &ExitOutcome) -> ProtocolResult {
        if !self.line.is_empty() || self.overflow {
            self.consume();
        }
        let verdict = match outcome {
            ExitOutcome::Signaled { .. } => Verdict::Signaled,
            ExitOutcome::TimedOut { .. } => Verdict::TimedOut,
            ExitOutcome::SpawnError { .. } | ExitOutcome::IoError { .. } => {
                Verdict::InfrastructureError
            }
            ExitOutcome::Exited { code, .. } => self.exited(*code),
        };
        ProtocolResult {
            verdict,
            tap: Some(self.summary),
        }
    }

    /// 根据正常退出的进程状态码 `code` 与已收集的 TAP 统计数据判定最终 [`Verdict`]。
    fn exited(&mut self, code: i32) -> Verdict {
        // Step 1: 只要出现 `Bail out!`、失败子项或意外通过（XPASS）子项，直接判定为 Failed。
        if self.summary.bailed_out || self.summary.failed != 0 || self.summary.xpassed != 0 {
            return Verdict::Failed;
        }
        // Step 2: 校验终末跳号跳过（terminal_skip）与计划总数（plan == count）的完整性；任何协议违规均判定为 Broken。
        if self.terminal_skip && code != 4 {
            self.invalid("terminal skip numbering requires exit 4");
        }
        let complete = self.summary.plan == Some(self.count);
        if !self.zero_skip && !complete && !(code == 4 && self.terminal_skip) {
            self.invalid("missing TAP plan or incomplete results");
        }
        if self.summary.error.is_some() {
            return Verdict::Broken;
        }
        // Step 3: 在协议完整的前提下，结合退出码 0（正常结束）或 4（KSFT_SKIP）给出 Pass、Skip 或 Failed 结论。
        match code {
            0 if self.zero_skip || (self.count != 0 && self.count == self.summary.skipped) => {
                Verdict::Skip
            }
            0 => Verdict::Pass,
            4 if self.zero_skip || self.summary.skipped != 0 => Verdict::Skip,
            _ => Verdict::Failed,
        }
    }
}

/// 检查 `#` 注释文本的首个空白分隔词是否与目标指令名 `expected`（忽略 ASCII 大小写，如 `SKIP`、`XFAIL`、`XPASS`）完全匹配。
fn directive(comment: &str, expected: &str) -> bool {
    comment
        .split_whitespace()
        .next()
        .is_some_and(|word| word.eq_ignore_ascii_case(expected))
}
