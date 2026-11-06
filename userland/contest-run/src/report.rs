//! 子进程退出码与比赛输出协议（`Plain`、`Ltp`、`Tap`、`Basic`、`Pre2025Libctest`）的流式解析与结果判定模块。
//!
//! ## 模块职责与架构定位
//! 本模块在测试子进程运行期间接收来自 `process` 模块的 `stdout`/`stderr` 原始字节流（`ProtocolObserver::feed`），
//! 并在子进程结束时结合 `ExitOutcome` 产出最终的 `ProtocolResult`（`ProtocolObserver::finish`）。
//! 上游由 `process::run_command` 驱动，下游把 TAP 协议的具体行解析委托给 `tap::TapObserver`。
//!
//! ## 核心协议判定机制与状态机
//!
//! 1. **LTP 退出码位图（`ltp_verdict`）**：
//!    Linux Test Project (LTP) 将测试结果编码在进程退出码的低位比特中：
//!    ```text
//!      Bit 0 (0x01, 1)  : TFAIL (测试断言失败)
//!      Bit 1 (0x02, 2)  : TBROK (测试准备或环境损坏)
//!      Bit 2 (0x04, 4)  : TWARN (运行附带警告)
//!      Bit 5 (0x20, 32) : TCONF (内核配置或特性不支持)
//!    ```
//!    判定优先级：`负数或未知位或 TBROK` (`Broken`) > `TFAIL` (`Failed`) > `TWARN` (`Warning`) > `TCONF` (`Configuration`) > `0` (`Pass`)。
//!
//! 2. **带起止标记的流式状态机（`Basic` 与 `Pre2025Libctest`）**：
//!    `LineState` 为 `stdout` 和 `stderr` 各自独立维护一个有界行前缀状态机（只缓存不超过标记长度上限的前缀，防止海量输出耗尽内存）：
//!    ```text
//!      [初始: started=false]
//!           │
//!           ├── 匹配当前用例 START 标记行 ──► [活动区间: started=true, active=true]
//!           │                                      │
//!           │                                      ├─ 出现 "Pass!" ──────────► passed=true
//!           │                                      ├─ 出现已知错误/FAIL前缀 ─► failed=true
//!           │                                      ├─ 出现其他用例 START/END ► failed=true, active=false, foreign=true
//!           │                                      └─ 匹配当前用例 END 标记 ─► ended=true, active=false
//!           │
//!           └── 最终判定 (marked_verdict):
//!               1. 退出码不等于预期值 或 任一流 failed=true ──► Failed
//!               2. 两个流均未出现 started=true             ──► NotRun
//!               3. 同一流内满足 ended=true (且 Libctest 需 passed=true) ──► Completed / Pass
//!               4. 其他情况                                ──► Failed
//!    ```
//!
//! ## 并发与锁契约
//! `ProtocolObserver` 实例由单次 `run_command` 栈帧独占持有，按顺序消费 `poll` 循环读取的字节块，无跨线程共享或锁操作。

use crate::model::Protocol;
use crate::outcome::{ExitOutcome, ProtocolResult, Verdict};
use crate::tap::TapObserver;

/// 子进程输出流通道类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamKind {
    /// 标准输出流（文件描述符 1）。
    Stdout,
    /// 标准错误流（文件描述符 2）。
    Stderr,
}

/// 针对单个测试用例的流式协议观察器。
pub struct ProtocolObserver<'a> {
    /// 当前测试用例所使用的判定协议引用。
    protocol: &'a Protocol,
    /// 预期匹配的测试起始标记行文本（如 `"========== START test_open =========="`）。
    start: String,
    /// 预期匹配的测试结束标记行文本（如 `"========== END test_open =========="`）。
    end: String,
    /// `Pre2025Libctest` 协议下的失败行匹配前缀（如 `"FAIL <selector> "`）。
    failure: String,
    /// 分别对应 `[Stdout, Stderr]` 的双通道行级状态机。
    streams: [LineState; 2],
    /// 当协议为 `Protocol::Tap` 时启用的专用 TAP 解析器。
    tap: Option<TapObserver>,
}

impl<'a> ProtocolObserver<'a> {
    /// 根据指定的测试协议初始化观察器，并计算每行需要缓存的最大字节上界。
    pub fn new(protocol: &'a Protocol) -> Self {
        let (start, end, failure) = match protocol {
            Protocol::Basic { name } => {
                let marker = if name == "mkdir_" { "mkdir" } else { name };
                (
                    format!("========== START test_{marker} =========="),
                    format!("========== END test_{marker} =========="),
                    String::new(),
                )
            }
            Protocol::Pre2025Libctest { variant, selector } => (
                format!("========== START entry-{variant}.exe {selector} =========="),
                format!("========== END entry-{variant}.exe {selector} =========="),
                format!("FAIL {selector} "),
            ),
            Protocol::Tap => (String::new(), String::new(), String::new()),
            Protocol::Plain | Protocol::Ltp => (String::new(), String::new(), String::new()),
        };
        let limit = if start.is_empty() {
            0
        } else {
            128.max(start.len() + 1)
                .max(end.len() + 1)
                .max(failure.len() + 1)
        };
        Self {
            protocol,
            start,
            end,
            failure,
            streams: [LineState::new(limit), LineState::new(limit)],
            tap: matches!(protocol, Protocol::Tap).then(TapObserver::new),
        }
    }

    /// 向观察器喂入从指定输出流读取到的原始字节块。
    ///
    /// `Stdout` 与 `Stderr` 各自独立按 `\n` 切分；单行长度超过有界上限时仅保留前缀并置 `overflow = true`，不无限增长内存。
    pub fn feed(&mut self, stream: StreamKind, bytes: &[u8]) {
        if let Some(tap) = &mut self.tap {
            if stream == StreamKind::Stdout {
                tap.feed(bytes);
            }
            return;
        }
        if matches!(self.protocol, Protocol::Plain | Protocol::Ltp) {
            return;
        }
        let state = &mut self.streams[match stream {
            StreamKind::Stdout => 0,
            StreamKind::Stderr => 1,
        }];
        for &byte in bytes {
            if byte == b'\n' {
                state.consume(self.protocol, &self.start, &self.end, &self.failure);
            } else if state.line.len() < state.limit {
                state.line.push(byte);
            } else {
                state.overflow = true;
            }
        }
    }

    /// 在子进程完全退出且管道排空后闭合观察器，结合进程退出结果生成最终的 `ProtocolResult`。
    pub fn finish(mut self, outcome: &ExitOutcome) -> ProtocolResult {
        if let Some(tap) = self.tap.take() {
            return tap.finish(outcome);
        }
        ProtocolResult {
            verdict: self.finish_marked(outcome),
            tap: None,
        }
    }

    /// 刷入两个输出流末尾未以换行符结束的残留行，并按进程退出状态与协议规则计算最终 `Verdict`。
    fn finish_marked(mut self, outcome: &ExitOutcome) -> Verdict {
        // Step 1: 消费 stdout/stderr 末尾可能没有换行符的最后一行内容。
        for state in &mut self.streams {
            if !state.line.is_empty() || state.overflow {
                state.consume(self.protocol, &self.start, &self.end, &self.failure);
            }
        }
        // Step 2: 优先处理信号终止、超时与启动/I/O 故障，正常退出时再交由各协议分支判定。
        let code = match outcome {
            ExitOutcome::Exited { code, .. } => *code,
            ExitOutcome::Signaled { .. } => return Verdict::Signaled,
            ExitOutcome::TimedOut { .. } => return Verdict::TimedOut,
            ExitOutcome::SpawnError { .. } | ExitOutcome::IoError { .. } => {
                return Verdict::InfrastructureError
            }
        };
        match self.protocol {
            Protocol::Plain => {
                if code == 0 {
                    Verdict::Pass
                } else {
                    Verdict::Failed
                }
            }
            Protocol::Ltp => ltp_verdict(code),
            Protocol::Tap => unreachable!("TAP has its own streaming state"),
            Protocol::Basic { .. } => self.marked_verdict(code, 0, false),
            Protocol::Pre2025Libctest { .. } => self.marked_verdict(code, 1, true),
        }
    }

    /// 根据双流的标记状态机汇总结论（用于 `Basic` 与 `Pre2025Libctest` 协议）。
    ///
    /// 要求预期退出码匹配、任一流均未观测到失败证据，且至少有一个流完整经历了当前用例的起止区间（`Pre2025Libctest` 还需在该区间内看到 `Pass!`）。
    fn marked_verdict(&self, code: i32, expected: i32, needs_pass: bool) -> Verdict {
        if code != expected || self.streams.iter().any(|state| state.failed) {
            return Verdict::Failed;
        }
        if !self.streams.iter().any(|state| state.started) {
            return Verdict::NotRun;
        }
        if self
            .streams
            .iter()
            .any(|state| state.ended && (!needs_pass || state.passed))
        {
            if needs_pass {
                Verdict::Pass
            } else {
                Verdict::Completed
            }
        } else {
            Verdict::Failed
        }
    }
}

/// 单个输出流（`Stdout` 或 `Stderr`）的有界行缓冲与标记匹配状态机。
struct LineState {
    /// 当前正在累积的单行有界字节前缀缓冲区。
    line: Vec<u8>,
    /// 单行缓冲区允许保存的最大字节数上界。
    limit: usize,
    /// 当前行字节数是否已超过 `limit`（超长行不能匹配精确的起止或 `Pass!` 标记，但仍可凭前缀匹配失败信息）。
    overflow: bool,
    /// 该流中是否曾完整匹配到当前用例的 `START` 标记行。
    started: bool,
    /// 当前是否正处于目标用例的 `START` 与 `END` 标记区间内部。
    active: bool,
    /// 当前是否进入了其他非目标用例的 `START`/`END` 区域（用于抑制外部区域触发误报）。
    foreign: bool,
    /// 是否在 `active` 状态下完整匹配到了当前用例的 `END` 标记行。
    ended: bool,
    /// 是否在 `active` 状态下完整匹配到了 `Pass!` 行。
    passed: bool,
    /// 是否观测到了任何协议违规（如重复 `START`、未闭合即遇到外部标记）或显式错误输出行。
    failed: bool,
}

impl LineState {
    /// 创建预分配固定容量 `limit` 的单流行状态机。
    fn new(limit: usize) -> Self {
        Self {
            line: Vec::with_capacity(limit),
            limit,
            overflow: false,
            started: false,
            active: false,
            foreign: false,
            ended: false,
            passed: false,
            failed: false,
        }
    }

    /// 消费并判定当前累积的一行文本，更新标记状态机后清空行缓冲区。
    ///
    /// 仅未发生截断（`!overflow`）的完整行才能匹配起止边界或 `Pass!`；发生截断的超长行仍会检查是否以失败前缀开头。
    fn consume(&mut self, protocol: &Protocol, start: &str, end: &str, failure: &str) {
        // Step 1: 剥离行尾可选的 '\r'，并检查是否命中 Libctest 或 Basic 协议的失败文本。
        let mut line = self.line.as_slice();
        if !self.overflow {
            line = line.strip_suffix(b"\r").unwrap_or(line);
        }
        if matches!(protocol, Protocol::Pre2025Libctest { .. })
            && line.starts_with(failure.as_bytes())
        {
            self.failed = true;
        }
        if let Protocol::Basic { name } = protocol {
            if !self.foreign && basic_error(name, line, self.overflow) {
                self.failed = true;
            }
        }
        // Step 2: 若当前行未超长截断，匹配起止边界标记与 "Pass!" 状态行。
        if !self.overflow {
            if line == start.as_bytes() {
                if self.started {
                    self.failed = true;
                }
                self.started = true;
                self.active = true;
                self.foreign = false;
                self.ended = false;
                self.passed = false;
            } else if line == end.as_bytes()
                || (matches!(protocol, Protocol::Basic { name } if name == "execve")
                    && line == b"========== END main ==========")
            {
                if self.active {
                    self.ended = true;
                }
                self.active = false;
            } else if line.starts_with(b"========== START ") || line.starts_with(b"========== END ")
            {
                if self.active {
                    self.failed = true;
                }
                self.active = false;
                self.foreign = true;
            } else if self.active && line == b"Pass!" {
                self.passed = true;
            }
        }
        // Step 3: 重置行缓冲与溢出标志，准备接收下一行。
        self.line.clear();
        self.overflow = false;
    }
}

/// 检查单行输出是否命中初赛 `basic` 测试套件对应子用例的已知错误提示或非零返回值。
fn basic_error(name: &str, line: &[u8], overflow: bool) -> bool {
    let literal = match name {
        "clone" => "clone process error.",
        "close" => "  close error.",
        "execve" => "  execve error.",
        "exit" => "exit ERR.",
        "getcwd" => "getcwd ERROR.",
        "getppid" => "  getppid error.",
        "gettimeofday" => "gettimeofday error.",
        "mkdir_" => "  mkdir error.",
        "mmap" | "munmap" => "mmap error.",
        "sleep" => "sleep error.",
        "unlink" => "  unlink error!",
        "wait" => "wait child error.",
        "waitpid" => "waitpid error.",
        _ => "",
    };
    if !overflow
        && ((!literal.is_empty() && line == literal.as_bytes())
            || line == b" --- Assert Fatal ! ---")
    {
        return true;
    }
    let fields: &[&[u8]] = match name {
        "mount" => &[b"mount return: ", b"umount return: "],
        "umount" => &[b"mount return: ", b"return: "],
        "munmap" => &[b"munmap return: "],
        _ => &[],
    };
    fields.iter().any(|prefix| {
        line.strip_prefix(*prefix)
            .is_some_and(|value| overflow || value != b"0")
    })
}

/// 将 LTP 测试程序的进程退出码按 `TFAIL`/`TBROK`/`TWARN`/`TCONF` 位掩码映射为 `Verdict`。
fn ltp_verdict(code: i32) -> Verdict {
    const TFAIL: i32 = 1;
    const TBROK: i32 = 2;
    const TWARN: i32 = 4;
    const TCONF: i32 = 32;
    if code < 0 || (code & !(TFAIL | TBROK | TWARN | TCONF)) != 0 || (code & TBROK) != 0 {
        Verdict::Broken
    } else if (code & TFAIL) != 0 {
        Verdict::Failed
    } else if (code & TWARN) != 0 {
        Verdict::Warning
    } else if (code & TCONF) != 0 {
        Verdict::Configuration
    } else {
        Verdict::Pass
    }
}
