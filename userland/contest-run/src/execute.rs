//! 单个测试用例的生命周期编排：前置准备（`setup`）、内核钩子配对（`before`/`after`）、主命令执行与后置清理（`cleanup`）。
//!
//! ## 模块职责与架构定位
//! 本模块位于测例选择层（`crate::selection`）与底层子进程执行器（`crate::process::run_command`）之间。
//! 上游主运行器（`contest-run`）针对每个解析后的 [`ResolvedCase`] 按轮次调用 [`execute_case`]，也可以在初始化
//! 根环境时直接调用 [`run_steps`] 执行一系列准备命令。本模块负责维护五阶段生命周期顺序与错误传播规则，不直接管理底层管道或信号。
//!
//! ## 单轮测例执行状态机
//! ```text
//! [1. setup 命令序列] (stop_on_error = true)
//!       │
//!       ├─> 失败 ──> 记录 setup_error ──────────────────────────────────┐
//!       ▼ 成功                                                          │
//! [2. before 钩子] (仅当 case.hooks == true)                            │
//!       │                                                               │
//!       ├─> 失败 ──> 记录 before_error (before_succeeded = false) ──────┤
//!       ▼ 成功 (或未启用 hooks)                                         │
//! [3. 主测试命令 run_command]                                           │
//!       │                                                               │
//!       ▼ 无论主命令通过、失败、超时或被信号终止                        │
//! [4. after 钩子] (仅当 before_succeeded == true 时配对执行)            │
//!       │                                                               │
//!       ▼                                                               │
//! [5. cleanup 命令序列] (stop_on_error = false，始终执行以回收环境资源) ◀┘
//! ```
//!
//! ## 并发与锁契约
//! 本模块在单线程控制流中顺序推进各阶段命令，所有状态保存在栈上的 [`CaseRecord`] 中，不涉及共享锁。

use crate::model::{CommandSpec, Protocol, ResolvedCase};
use crate::outcome::{Execution, ExitOutcome};
use crate::process::run_command;
use std::collections::BTreeMap;
use std::io;
use std::os::fd::RawFd;
use std::path::Path;
use std::time::Duration;

/// 单个测例在单次迭代（Iteration）中的完整执行记录。
#[derive(Debug)]
pub struct CaseRecord {
    /// 主测试命令的子进程执行结果与协议判定；若在 `setup` 或 `before` 阶段提前失败则为 `None`。
    pub execution: Option<Execution>,
    /// 前置准备（`setup`）阶段遇到的首个命令执行错误描述。
    pub setup_error: Option<String>,
    /// 主命令启动前调用内核 `before` 钩子时发生的错误描述。
    pub before_error: Option<String>,
    /// 主命令结束后调用内核 `after` 钩子时发生的错误描述。
    pub after_error: Option<String>,
    /// 后置清理（`cleanup`）阶段所有失败命令的错误描述列表。
    pub cleanup_errors: Vec<String>,
    /// 当前执行所属的轮次编号（从 1 开始计数）。
    pub iteration: u32,
}

impl CaseRecord {
    /// 判断本次执行是否发生了环境或基础设施层面的致命错误，需要立即中止后续测例或轮次。
    ///
    /// 当 `setup`、`before`、`after`、`cleanup` 任一环节报错，或主命令在执行期间遭遇底层管道/进程 I/O 错误时返回 `true`，
    /// 防止在被污染或未知的系统状态下继续运行后续测试。
    pub fn must_stop(&self) -> bool {
        self.setup_error.is_some()
            || self.before_error.is_some()
            || self.after_error.is_some()
            || !self.cleanup_errors.is_empty()
            || self.execution.as_ref().is_some_and(|run| {
                !run.io_errors.is_empty() || matches!(run.outcome, ExitOutcome::IoError { .. })
            })
    }

    /// 判断本轮测例是否完全通过。
    ///
    /// 仅当未发生任何基础设施错误（`!self.must_stop()`），且主命令已实际运行并被对应输出协议判定为成功时返回 `true`。
    pub fn is_success(&self) -> bool {
        !self.must_stop()
            && self
                .execution
                .as_ref()
                .is_some_and(|run| run.protocol.verdict.is_success())
    }
}

/// 顺序执行一组环境准备或清理命令（`argv` 数组列表），并收集所有非零退出、超时或 I/O 错误。
///
/// 调用上下文：在根环境初始化（`prepare_root`）、测例前置准备（`setup`）或后置清理（`cleanup`）阶段同步调用。
/// 参数：
/// - `root`：可选的 `chroot` 目标根目录；为 `None` 时在当前系统根下执行。
/// - `cwd`：子进程的工作目录（相对于生效的根目录）。
/// - `commands`：待执行的命令数组列表，每项为 `[program, arg1, arg2, ...]`。
/// - `env`：附加或覆盖的环境变量映射。
/// - `timeout`：单条命令的执行超时上限。
/// - `outputs`：子进程标准输出和标准错误绑定的文件描述符对 `[stdout_fd, stderr_fd]`。
/// - `stop_on_error`：若为 `true`（如 `setup` 阶段），遇到第一条失败命令即停止后续命令；若为 `false`（如 `cleanup` 阶段），继续尝试执行剩余清理命令。
/// 返回值：所有执行失败的命令诊断信息列表；全部以退出码 0 正常结束且无 I/O 错误时返回空列表。
pub fn run_steps(
    root: Option<&Path>,
    cwd: &Path,
    commands: &[Vec<String>],
    env: &BTreeMap<String, String>,
    timeout: Duration,
    outputs: [RawFd; 2],
    stop_on_error: bool,
) -> Vec<String> {
    let mut errors = Vec::new();
    for argv in commands {
        let Some((program, args)) = argv.split_first() else {
            errors.push("empty preparation command".into());
            break;
        };
        let spec = CommandSpec::Exec {
            program: program.clone(),
            args: args.to_vec(),
        };
        let run = run_command(root, cwd, &spec, env, timeout, &Protocol::Plain, outputs);
        if !matches!(run.outcome, ExitOutcome::Exited { code: 0, .. }) || !run.io_errors.is_empty()
        {
            errors.push(format!(
                "command {argv:?}: {:?}; I/O errors: {:?}",
                run.outcome, run.io_errors
            ));
            if stop_on_error {
                break;
            }
        }
    }
    errors
}

/// 执行单个测例的一轮完整生命周期（`setup` -> `before` 钩子 -> 主命令 -> `after` 钩子 -> `cleanup`）。
///
/// 调用上下文：由测试运行器在单轮测试循环中调用。
/// 参数 `root` 为可选的目标根目录路径，`case` 为已完成参数合并的测例规格，`iteration` 为从 1 开始的轮次号，
/// `outputs` 为输出文件描述符对，`hook` 为触发内核测试钩子的回调函数。
/// 返回值：包含主命令结果及各阶段基础设施错误的 [`CaseRecord`]。无论中间阶段是否失败，都会保证已成功的 `before` 钩子得到 `after` 配对，且 `cleanup` 列表始终被执行。
pub fn execute_case(
    root: Option<&Path>,
    case: &ResolvedCase,
    iteration: u32,
    outputs: [RawFd; 2],
    mut hook: impl FnMut(bool, &str) -> io::Result<()>,
) -> CaseRecord {
    let timeout = Duration::from_secs(case.timeout_sec);
    // Step 1: 执行 setup 准备命令序列（遇到首个错误即中止后续 setup 命令）。
    let setup = run_steps(
        root,
        &case.cwd,
        &case.setup,
        &case.env,
        timeout,
        outputs,
        true,
    );
    let mut record = CaseRecord {
        execution: None,
        setup_error: setup.into_iter().next(),
        before_error: None,
        after_error: None,
        cleanup_errors: Vec::new(),
        iteration,
    };
    // Step 2: 若 setup 成功且测例启用了内核钩子，触发 before 钩子并记录是否成功，以便后续严格配对 after 钩子。
    let mut before_succeeded = false;
    if record.setup_error.is_none() && case.hooks {
        match hook(true, &case.id) {
            Ok(()) => before_succeeded = true,
            Err(error) => record.before_error = Some(error.to_string()),
        }
    }
    // Step 3: 仅当 setup 和 before 钩子均无错误时，启动并监控主测试命令。
    if record.setup_error.is_none() && record.before_error.is_none() {
        record.execution = Some(run_command(
            root,
            &case.cwd,
            &case.command,
            &case.env,
            timeout,
            &case.protocol,
            outputs,
        ));
    }
    // Step 4: 若先前 before 钩子已成功执行，无论主命令结果如何，必须配对执行 after 钩子以恢复内核状态。
    if before_succeeded {
        if let Err(error) = hook(false, &case.id) {
            record.after_error = Some(error.to_string());
        }
    }
    // Step 5: 始终执行全部 cleanup 清理命令（不因单条失败而跳过其余清理命令），汇总并返回完整记录。
    record.cleanup_errors = run_steps(
        root,
        &case.cwd,
        &case.cleanup,
        &case.env,
        timeout,
        outputs,
        false,
    );
    record
}
