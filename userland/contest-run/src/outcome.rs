//! 子进程退出状态与协议判定结果的公共数据类型模块。
//!
//! ## 模块职责与架构定位
//! 本模块定义 `process`（进程管理）、`report`/`tap`（协议解析）与 `execute`（用例执行编排）之间共享的结果数据模型。
//! 本模块不依赖具体的执行或解析实现，作为底层进程控制与上层测试报告之间的契约边界。
//!
//! ## 结果判定分层关系
//!
//! ```text
//!   操作系统进程退出与管道状态
//!       │
//!       ├─► ExitOutcome (Exited / Signaled / SpawnError / TimedOut / IoError)
//!       ├─► io_errors: Vec<String> (管道转发、进程组信号、辅助进程回收等非致命或附加 I/O 错误)
//!       └─► ProtocolObserver / TapObserver 流式输出解析
//!               │
//!               ▼
//!         ProtocolResult { verdict: Verdict, tap: Option<TapSummary> }
//!               │
//!               ▼
//!         Execution { outcome, io_errors, elapsed, protocol }
//! ```
//!
//! ## 并发与锁契约
//! 本模块仅包含纯数据结构体与枚举，无锁或跨线程共享状态。

use std::time::Duration;

/// 子进程生命周期结束时的底层退出分类结果。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExitOutcome {
    /// 子进程正常调用 `exit` / `_exit` 退出。
    Exited {
        /// 提取出的低 8 位进程退出码（`0..=255`）。
        code: i32,
        /// `waitpid` 返回的原始 32 位状态字（`wstatus`）。
        raw: i32,
    },
    /// 子进程在未超时的情况下被未捕获的信号终止。
    Signaled {
        /// 导致进程终止的信号编号（如 `SIGSEGV = 11`）。
        signal: i32,
        /// `waitpid` 返回的原始 32 位状态字（`wstatus`）。
        raw: i32,
    },
    /// 子进程在 `fork`/`exec` 启动阶段（含 `pre_exec` 中的 `chroot`/`chdir`/`setpgid`）失败。
    SpawnError {
        /// 操作系统返回的错误码（`errno`）。
        errno: Option<i32>,
        /// 可读的错误描述信息。
        message: String,
    },
    /// 子进程运行或输出排空超过单次用例期限（`deadline`），由执行器发送信号终止。
    TimedOut {
        /// 回收子进程时取得的原始 `waitpid` 状态字（若 `wait` 失败则为 `None`）。
        raw: Option<i32>,
        /// 若子进程最终因信号退出，记录该信号编号（通常为 `SIGTERM` 或 `SIGKILL`）。
        signal: Option<i32>,
    },
    /// 子进程启动后在管道设置、输出转发建立或 `waitpid` 过程中发生系统 I/O 错误。
    IoError {
        /// 操作系统返回的错误码（`errno`）。
        errno: Option<i32>,
        /// 可读的错误描述信息。
        message: String,
        /// 回收子进程时取得的原始 `waitpid` 状态字（若尚未取得则为 `None`）。
        raw: Option<i32>,
    },
}

/// 单次命令运行（主测试命令或前后置脚本命令）的完整执行记录。
#[derive(Debug)]
pub struct Execution {
    /// 子进程底层的退出分类状态。
    pub outcome: ExitOutcome,
    /// 运行期间在标准输出/错误管道读取、转发、辅助进程回收或发送进程组信号时收集到的独立 I/O 错误列表。
    pub io_errors: Vec<String>,
    /// 命令从启动到完全结束（含输出排空）所耗费的单调时钟时长。
    pub elapsed: Duration,
    /// 结合退出状态与流式输出解析得出的协议判定结果。
    pub protocol: ProtocolResult,
}

/// 测试用例单次迭代的统一高层协议判定结论。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// 测试通过（退出码及协议要求的通过标记全部满足）。
    Pass,
    /// 测试被主动跳过（如 TAP 协议中全部子项均带 `# SKIP` 指令）。
    Skip,
    /// 测试完整运行结束（用于仅检查 `START`/`END` 边界与无报错、但不输出显式 `Pass!` 标记的 `basic` 协议）。
    Completed,
    /// 测试断言失败、退出码非预期或缺少必要的完成标记。
    Failed,
    /// 测试程序自身执行异常或协议格式损坏（如 LTP `TBROK` 或 TAP 语法错误）。
    Broken,
    /// 测试运行完成但附带警告（如 LTP `TWARN`）。
    Warning,
    /// 当前内核或运行环境不满足测试所需的配置或特性支持（如 LTP `TCONF`）。
    Configuration,
    /// 测试未实际运行（如前序基础设施故障中止，或输出中从未出现协议起始标记）。
    NotRun,
    /// 测试运行超时并被强制终止。
    TimedOut,
    /// 测试进程因未捕获信号崩溃或被杀死。
    Signaled,
    /// 执行器自身在挂载、钩子、管道或进程控制环节发生基础设施错误。
    InfrastructureError,
}

impl Verdict {
    /// 返回报告日志中使用的固定英文状态名称字符串。
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Pass => "Pass",
            Self::Skip => "Skip",
            Self::Completed => "Completed",
            Self::Failed => "Failed",
            Self::Broken => "Broken",
            Self::Warning => "Warning",
            Self::Configuration => "Configuration",
            Self::NotRun => "NotRun",
            Self::TimedOut => "TimedOut",
            Self::Signaled => "Signaled",
            Self::InfrastructureError => "InfrastructureError",
        }
    }

    /// 判断该结论在执行器汇总退出码时是否视为成功（仅 `Pass` 与 `Completed` 视为成功）。
    pub fn is_success(self) -> bool {
        matches!(self, Self::Pass | Self::Completed)
    }
}

/// 协议观察器在命令结束时生成的最终解析结果。
#[derive(Debug, PartialEq, Eq)]
pub struct ProtocolResult {
    /// 综合进程退出状态与流式输出内容得出的高层判定结论。
    pub verdict: Verdict,
    /// 若该用例采用 TAP 协议，附带解析出的子测试统计摘要。
    pub tap: Option<TapSummary>,
}

/// Test Anything Protocol (TAP) 协议单次运行的子测试统计摘要。
#[derive(Debug, Default, PartialEq, Eq)]
pub struct TapSummary {
    /// `1..N` 计划行声明的预期子测试总数（若未出现计划行则为 `None`）。
    pub plan: Option<u64>,
    /// 状态为 `ok` 且无 `TODO`/`SKIP` 指令的通过子测试数量。
    pub passed: u64,
    /// 状态为 `not ok` 且无 `TODO`/`SKIP` 指令的失败子测试数量。
    pub failed: u64,
    /// 带有 `# SKIP` 指令的子测试数量（或 `1..0 # SKIP` 整集跳过）。
    pub skipped: u64,
    /// 状态为 `not ok` 且带有 `# TODO` 指令的预期失败子测试数量。
    pub xfailed: u64,
    /// 状态为 `ok` 但带有 `# TODO` 指令的意外通过子测试数量。
    pub xpassed: u64,
    /// 输出中是否出现了 `Bail out!` 紧急中止行。
    pub bailed_out: bool,
    /// 流式解析过程中遇到的首个 TAP 协议格式或计数不匹配错误描述。
    pub error: Option<String>,
}
