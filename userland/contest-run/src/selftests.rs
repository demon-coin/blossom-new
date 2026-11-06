//! Linux 内核自检测试（kselftest）清单解析、用例筛选与执行汇总模块。
//!
//! # 模块职责与架构定位
//! 本模块实现 `contest-run selftests` 子命令，用于在当前系统根环境下直接运行已安装的 `kselftest` 测试集：
//! - **上游**：由 `main.rs` 在识别到 `selftests` 子命令时调用 [`Options::parse`] 与 [`run`]。
//! - **下游**：复用 [`crate::process::run_command`] 启动子进程并指定 [`Protocol::Tap`] 解析 TAP（Test Anything Protocol）输出，最终将结构化的 `SELFTEST` 进度与汇总行写入报告流。
//!
//! # 核心机制与退出码状态机
//!
//! ## 1. 清单目录布局与身份标识（`collection:program`）
//! ```text
//! /test/selftests/                      <-- manifest 所在父目录 (directory)
//! ├── kselftest-list.txt                <-- 每行格式: "<collection>:<program>"
//! ├── timers/                           <-- 集合子目录 (cwd = directory.join(collection))
//! │   └── posix_timers                  <-- 可执行文件 (executable = cwd.join(program))
//! └── vm/
//!     └── mlock2-tests
//! ```
//!
//! ## 2. 汇总退出码优先级（[`Summary::status`]）
//! ```text
//!                        ┌─ errors > 0  ──► 退出码 2 (基础设施/协议损坏错误，最高优先级)
//!                        ├─ failed > 0  ──► 退出码 1 (至少一个测试失败)
//! Summary 统计结果 ──────┼─ passed > 0  ──► 退出码 0 (无错误无失败，且至少一个通过)
//!                        ├─ skipped > 0 ──► 退出码 4 (全部已运行测试均为跳过，对应 KSFT_SKIP = 4)
//!                        └─ 全为 0      ──► 退出码 2 (异常空汇总兜底)
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本模块按清单顺序串行启动每个自检测试程序，不持有任何进程内互斥锁或自旋锁。

use crate::model::{CommandSpec, Protocol};
use crate::outcome::{Execution, ExitOutcome, Verdict};
use crate::process::run_command;
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io::{self, Write};
use std::os::fd::RawFd;
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// 单个 `kselftest` 测试程序的默认执行超时上限（60 秒）。
const DEFAULT_PROGRAM_TIMEOUT: Duration = Duration::from_secs(60);

/// `selftests` 子命令的命令行参数配置。
pub struct Options {
    /// `kselftest-list.txt` 清单文件的路径（默认为 `/test/selftests/kselftest-list.txt`）。
    pub manifest: PathBuf,
    /// 通过 `--collection` 显式筛选的测试集合名称集合（为空时若 `programs` 也为空则选择全部）。
    pub collections: BTreeSet<String>,
    /// 通过 `--program` 显式筛选的 `<collection>:<program>` 完整标识集合。
    pub programs: BTreeSet<String>,
    /// 是否仅列出匹配的测试程序 ID 而不实际执行（`--list`）。
    pub list: bool,
    /// 单个测试程序从启动时刻算起的墙钟超时预算（单位：秒，子进程输出不会重置该计时器）。
    pub program_timeout: Duration,
}

impl Options {
    /// 解析 `selftests` 子命令参数列表，校验超时时间范围与筛选条件格式。
    ///
    /// # 参数
    /// - `args`：去除子命令名 `selftests` 后的命令行参数切片。
    ///
    /// # 返回值
    /// 解析成功返回 [`Options`]；遇到未知参数、重复单值选项、非法路径组件或超出单调时钟范围的超时值时返回错误描述。
    pub fn parse(args: &[String]) -> Result<Self, String> {
        let mut options = Self {
            manifest: "/test/selftests/kselftest-list.txt".into(),
            collections: BTreeSet::new(),
            programs: BTreeSet::new(),
            list: false,
            program_timeout: DEFAULT_PROGRAM_TIMEOUT,
        };
        let mut manifest_seen = false;
        let mut program_timeout_seen = false;
        let mut args = args.iter();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--list" if !options.list => options.list = true,
                "--manifest" if !manifest_seen => {
                    options.manifest = args.next().ok_or("missing value for --manifest")?.into();
                    manifest_seen = true;
                }
                "--program-timeout-seconds" if !program_timeout_seen => {
                    let value = args
                        .next()
                        .ok_or("missing value for --program-timeout-seconds")?;
                    if value.is_empty() || !value.bytes().all(|byte| byte.is_ascii_digit()) {
                        return Err("program timeout must be a positive integer in seconds".into());
                    }
                    let seconds: u64 = value
                        .parse()
                        .map_err(|_| "program timeout exceeds the supported range")?;
                    let timeout = Duration::from_secs(seconds);
                    if seconds == 0 || Instant::now().checked_add(timeout).is_none() {
                        return Err(
                            "program timeout is zero or exceeds the monotonic clock range".into(),
                        );
                    }
                    options.program_timeout = timeout;
                    program_timeout_seen = true;
                }
                "--collection" | "--program" => {
                    let value = args
                        .next()
                        .ok_or_else(|| format!("missing value for {arg}"))?;
                    let set = if arg == "--collection" {
                        component(value)?;
                        &mut options.collections
                    } else {
                        identity(value)?;
                        &mut options.programs
                    };
                    if !set.insert(value.clone()) {
                        return Err(format!("duplicate selection: {value}"));
                    }
                }
                _ => return Err(format!("unknown or repeated argument: {arg}")),
            }
        }
        Ok(options)
    }
}

/// 从清单中解析并选中的单个 `kselftest` 待执行程序描述。
#[derive(Debug, PartialEq, Eq)]
pub struct Program {
    /// 清单中的标准条目标识（格式为 `<collection>:<program>`）。
    pub id: String,
    /// 子进程启动时的工作目录（即该集合所在的子目录 `directory/<collection>`）。
    pub cwd: PathBuf,
    /// 待执行程序在文件系统中的完整路径（`directory/<collection>/<program>`）。
    pub executable: PathBuf,
}

/// 校验单个路径组件（集合名或程序文件名）是否合法。
///
/// 拒绝空字符串、`.`、`..`、空白字符、路径分隔符（`/`、`\`）、清单分隔符（`:`）及 NUL 字节，确保路径严格位于相应的集合子目录内。
fn component(value: &str) -> Result<(), String> {
    if value.is_empty()
        || value == "."
        || value == ".."
        || value
            .bytes()
            .any(|byte| byte.is_ascii_whitespace() || matches!(byte, b'/' | b'\\' | b':' | 0))
    {
        return Err(format!("invalid selftest path component: {value:?}"));
    }
    Ok(())
}

/// 将 `<collection>:<program>` 格式的条目的标识拆分为集合名与程序名，并分别校验两者的路径组件合法性。
fn identity(value: &str) -> Result<(&str, &str), String> {
    let (collection, program) = value
        .split_once(':')
        .ok_or_else(|| format!("expected collection:program: {value:?}"))?;
    component(collection)?;
    component(program)?;
    Ok((collection, program))
}

/// 读取并规范化 `kselftest` 清单文件，按 [`Options`] 中的集合与程序过滤条件返回待执行的 [`Program`] 列表。
///
/// # 参数
/// - `options`：命令行解析得到的清单路径与筛选集合。
///
/// # 返回值
/// 按清单原始行顺序返回匹配的 [`Program`] 列表；若清单包含重复条目、请求了不存在的集合/程序，或筛选结果为空，则返回错误字符串。
pub fn select(options: &Options) -> Result<Vec<Program>, String> {
    // Step 1: 规范化清单文件路径并读取全部文本内容，确定测试安装的根父目录。
    let manifest = fs::canonicalize(&options.manifest)
        .map_err(|error| format!("manifest {}: {error}", options.manifest.display()))?;
    let text = fs::read_to_string(&manifest)
        .map_err(|error| format!("manifest {}: {error}", manifest.display()))?;
    let directory = manifest
        .parent()
        .ok_or("manifest has no parent directory")?;
    let mut programs = Vec::new();
    let mut ids = BTreeSet::new();
    let mut collections = BTreeSet::new();
    // Step 2: 逐行解析清单中的 `<collection>:<program>` 条目，检测重复项并构造候选程序列表。
    for (index, line) in text.lines().enumerate() {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (collection, program) = identity(line)
            .map_err(|error| format!("{}:{}: {error}", manifest.display(), index + 1))?;
        if !ids.insert(line.to_string()) {
            return Err(format!("duplicate installed selftest: {line}"));
        }
        collections.insert(collection.to_string());
        let cwd = directory.join(collection);
        programs.push(Program {
            id: line.to_string(),
            executable: cwd.join(program),
            cwd,
        });
    }
    // Step 3: 核对用户通过 `--collection` 和 `--program` 指定的目标是否均在清单中存在，随后过滤出最终执行集。
    for requested in &options.collections {
        if !collections.contains(requested) {
            return Err(format!("unknown collection: {requested}"));
        }
    }
    for requested in &options.programs {
        if !ids.contains(requested) {
            return Err(format!("unknown program: {requested}"));
        }
    }
    programs.retain(|program| {
        let (collection, _) = program.id.split_once(':').unwrap();
        (options.collections.is_empty() && options.programs.is_empty())
            || options.collections.contains(collection)
            || options.programs.contains(&program.id)
    });
    if programs.is_empty() {
        return Err("no installed selftests selected".into());
    }
    Ok(programs)
}

/// `kselftest` 批量执行的分类统计计数器。
#[derive(Debug, Default, PartialEq, Eq)]
pub struct Summary {
    /// 判定为通过（[`Verdict::Pass`]）的测试程序数量。
    pub passed: usize,
    /// 判定为失败（[`Verdict::Failed`]、信号终止、超时等）的测试程序数量。
    pub failed: usize,
    /// 判定为跳过（[`Verdict::Skip`]）的测试程序数量。
    pub skipped: usize,
    /// 发生进程启动/I/O 错误或 TAP 协议损坏（[`Verdict::Broken`] / [`Verdict::InfrastructureError`]）的程序数量。
    pub errors: usize,
}

impl Summary {
    /// 根据单个测试程序的执行结果（[`Execution`]）更新分类统计计数。
    pub fn record(&mut self, run: &Execution) {
        if !run.io_errors.is_empty()
            || matches!(
                run.outcome,
                ExitOutcome::SpawnError { .. } | ExitOutcome::IoError { .. }
            )
        {
            self.errors += 1;
            return;
        }
        match run.protocol.verdict {
            Verdict::Pass => self.passed += 1,
            Verdict::Skip => self.skipped += 1,
            Verdict::Broken | Verdict::InfrastructureError => self.errors += 1,
            _ => self.failed += 1,
        }
    }

    /// 根据汇总计数计算整个 `selftests` 子命令的进程退出码（`0` 通过、`1` 失败、`2` 错误、`4` 全部跳过）。
    pub fn status(&self) -> i32 {
        if self.errors != 0 {
            2
        } else if self.failed != 0 {
            1
        } else if self.passed != 0 {
            0
        } else if self.skipped != 0 {
            4
        } else {
            2
        }
    }
}

/// 按顺序运行选中的 `kselftest` 程序（或在 `--list` 模式下仅输出 ID），并将结构化执行日志写入 `output`。
///
/// # 参数
/// - `options`：命令行参数配置。
/// - `output`：结构化报告输出流（写入 `SELFTEST start`、`SELFTEST result` 及 `SELFTEST summary`）。
/// - `outputs`：传递给子进程输出转发器的宿主标准输出与标准错误文件描述符。
pub fn run(options: &Options, output: &mut impl Write, outputs: [RawFd; 2]) -> Result<i32, String> {
    // Step 1: 解析清单并筛选目标程序；若处于 `--list` 模式则逐行打印程序 ID 后直接返回 0。
    let programs = select(options)?;
    if options.list {
        for program in programs {
            writeln!(output, "{}", program.id).map_err(output_error)?;
        }
        output.flush().map_err(output_error)?;
        return Ok(0);
    }
    // Step 2: 依次启动每个测试程序，使用 TAP 协议解析器收集结果并实时刷新单条测试报告。
    let mut summary = Summary::default();
    for program in programs {
        writeln!(
            output,
            "SELFTEST start id={:?} timeout_seconds={}",
            program.id,
            options.program_timeout.as_secs()
        )
        .map_err(output_error)?;
        output.flush().map_err(output_error)?;
        let spec = CommandSpec::Exec {
            program: program
                .executable
                .to_str()
                .ok_or("selftest executable path is not UTF-8")?
                .into(),
            args: Vec::new(),
        };
        let run = run_command(
            None,
            &program.cwd,
            &spec,
            &BTreeMap::new(),
            options.program_timeout,
            &Protocol::Tap,
            outputs,
        );
        summary.record(&run);
        writeln!(
            output,
            "SELFTEST result id={:?} timeout_seconds={} outcome={} status={:?} elapsed_ms={} tap={:?} io_errors={:?}",
            program.id,
            options.program_timeout.as_secs(),
            run.protocol.verdict.as_str(),
            run.outcome,
            run.elapsed.as_millis(),
            run.protocol.tap,
            run.io_errors
        )
        .map_err(output_error)?;
        output.flush().map_err(output_error)?;
    }
    // Step 3: 输出最终统计汇总行并返回整体退出状态码。
    let status = summary.status();
    writeln!(
        output,
        "SELFTEST summary pass={} fail={} skip={} error={} status={status}",
        summary.passed, summary.failed, summary.skipped, summary.errors
    )
    .map_err(output_error)?;
    output.flush().map_err(output_error)?;
    Ok(status)
}

/// 将报告输出流的写入或刷新 `io::Error` 格式化为统一的错误字符串。
fn output_error(error: io::Error) -> String {
    format!("selftest report output: {error}")
}
