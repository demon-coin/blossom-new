//! 子进程执行器（`process::run_command`）与用例生命周期编排器（`execute::execute_case`）集成测试。
//!
//! # 模块职责与定位
//! 本文件是 `contest-run` 工具箱中进程执行与单用例生命周期控制的集成测试套件。
//! 上游构造测试命令规格（`CommandSpec`）、目标根目录规格（`RootSpec`）与已解析用例（`ResolvedCase`），
//! 下游直接调用 `process::run_command`、`execute::execute_case`、`custom::hook_device` 及 `runtime::diagnose`，
//! 验证子进程派生、工作目录与环境变量传递、标准输出/标准错误双流捕获转发、总超时与 `SIGKILL` 兜底回收、
//! `setup` / `before` Hook / 主命令 / `after` Hook / `cleanup` 完整生命周期配对，以及共享底层打开文件描述（OFD）时的非阻塞标志隔离。
//!
//! # 单用例执行生命周期与双流转发状态机
//!
//! ```text
//! execute_case(root, case, iteration, outputs, hook)
//!   │
//!   ├─► 1. setup 命令序列 ────(任一失败)──────────────────────────────────────┐
//!   │         │ (全部成功)                                                    │
//!   │         ▼                                                               │
//!   ├─► 2. hook(true, id) [before Hook] ──(失败)──────────────────────────────┤
//!   │         │ (成功或未启用 hooks)                                          │
//!   │         ▼                                                               │
//!   ├─► 3. run_command (主测试进程)                                           │
//!   │      ├─ Plain 协议：子进程直接继承 outputs 描述符                       │
//!   │      └─ Basic / Tap 等需观测协议：                                      │
//!   │           子进程写入捕获管道 ──► 宿主非阻塞轮询读取 ──► ProtocolObserver│
//!   │                                      │                                  │
//!   │                                      └──► 转发至 outputs 描述符         │
//!   │      (若达到总截止时间 deadline：先发 SIGTERM，超时未退则发 SIGKILL 回收)│
//!   │         │ (无论主进程正常退出、超时还是 SpawnError)                     │
//!   │         ▼                                                               │
//!   ├─► 4. hook(false, id) [after Hook] (若 before 已成功则必须配对执行)      │
//!   │         │                                                               │
//!   └─► 5. cleanup 命令序列 (始终执行；cleanup 错误单独记录，不覆盖主进程结果)◄┘
//! ```
//!
//! # 锁获取顺序与并发契约
//! Rust 默认测试框架会多线程并行运行本文件中的 `#[test]` 用例。由于 `run_command` 与 `execute_case`
//! 内部会调用 `fork`/`exec` 创建子进程，若多个测试线程并发打开临时文件或管道，并发 `fork` 出的子进程可能意外继承
//! 其他测试线程尚未关闭的文件描述符（导致管道写端无法观察到 EOF 或文件繁忙）。
//! 因此，本文件使用全局互斥锁 [`EXECUTION`] 将所有调用子进程执行器的测试串行化：
//! - 测试线程在调用 `execute_real_command` 或 `execute_real_case` 前先获取 `EXECUTION` 锁；
//! - 仅存在这一把全局锁，无嵌套锁顺序问题；若前序测试断言失败触发 panic，后续测试通过 `into_inner()` 忽略毒化（Poison）继续独立执行。

use contest_run::custom::hook_device;
use contest_run::execute::{execute_case as execute_real_case, CaseRecord};
use contest_run::model::{CommandSpec, Protocol, ResolvedCase, SourceLocation};
use contest_run::outcome::{Execution, ExitOutcome};
use contest_run::process::run_command as execute_real_command;
use std::collections::BTreeMap;
use std::fs::{self, File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// 为每个测试实例分配唯一临时目录编号的单调递增计数器。
static SERIAL: AtomicU64 = AtomicU64::new(0);
/// 串行化本文件中各测试函数的子进程派生操作，防止多线程并发 `fork` 导致控制文件描述符跨测试泄漏。
static EXECUTION: Mutex<()> = Mutex::new(());

/// 获取子进程执行互斥锁守卫；若前一个测试因断言 panic 导致锁中毒（Poisoned），则提取内部守卫继续执行后续测试。
fn exclusive_execution() -> std::sync::MutexGuard<'static, ()> {
    match EXECUTION.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// 串行化包装 `contest_run::process::run_command`：在持有 `EXECUTION` 锁的条件下运行单条命令，避免并行测试间发生描述符继承干扰。
fn run_command(
    root: Option<&Path>,
    cwd: &Path,
    spec: &CommandSpec,
    env: &BTreeMap<String, String>,
    timeout: Duration,
    protocol: &Protocol,
    outputs: [std::os::fd::RawFd; 2],
) -> Execution {
    let _exclusive = exclusive_execution();
    execute_real_command(root, cwd, spec, env, timeout, protocol, outputs)
}

/// 串行化包装 `contest_run::execute::execute_case`：在持有 `EXECUTION` 锁的条件下运行完整测试用例生命周期。
fn execute_case(
    root: Option<&Path>,
    case: &ResolvedCase,
    iteration: u32,
    outputs: [std::os::fd::RawFd; 2],
    hook: impl FnMut(bool, &str) -> io::Result<()>,
) -> CaseRecord {
    let _exclusive = exclusive_execution();
    execute_real_case(root, case, iteration, outputs, hook)
}

/// 测试专用的临时工作目录守卫（RAII Fixture）。
///
/// 为单个测试创建独立的临时目录（元组字段 `0` 为路径 `<temp_dir>/contest-execute-<pid>-<serial>`），并在离开作用域时自动清理。
struct Scratch(PathBuf);

impl Scratch {
    /// 创建并返回一个全新的测试专属临时目录。
    fn new() -> Self {
        let path = std::env::temp_dir().join(format!(
            "contest-execute-{}-{}",
            std::process::id(),
            SERIAL.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir(&path).unwrap();
        Self(path)
    }

    /// 在临时目录下以读写排他模式创建名为 `name` 的输出捕获文件。
    fn output(&self, name: &str) -> File {
        OpenOptions::new()
            .create_new(true)
            .read(true)
            .write(true)
            .open(self.0.join(name))
            .unwrap()
    }

    /// 以 `Protocol::Plain` 协议在当前临时目录中执行 `spec`，并返回执行结果及捕获到的 `stdout`、`stderr` 字节内容。
    fn run(&self, spec: CommandSpec, timeout: Duration) -> (Execution, Vec<u8>, Vec<u8>) {
        self.run_protocol(spec, timeout, &Protocol::Plain)
    }

    /// 以指定的 `protocol` 在当前临时目录中执行 `spec`，并返回执行结果及捕获到的 `stdout`、`stderr` 字节内容。
    fn run_protocol(
        &self,
        spec: CommandSpec,
        timeout: Duration,
        protocol: &Protocol,
    ) -> (Execution, Vec<u8>, Vec<u8>) {
        let stdout = self.output("stdout");
        let stderr = self.output("stderr");
        let run = run_command(
            None,
            &self.0,
            &spec,
            &BTreeMap::new(),
            timeout,
            protocol,
            [stdout.as_raw_fd(), stderr.as_raw_fd()],
        );
        (
            run,
            fs::read(self.0.join("stdout")).unwrap(),
            fs::read(self.0.join("stderr")).unwrap(),
        )
    }
}

impl Drop for Scratch {
    /// 测试结束时递归删除本实例创建的临时目录及其内部文件。
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

/// 辅助函数：构造通过 `/bin/sh -c` 执行指定 `command` 字符串的 `CommandSpec::Shell` 配置。
fn shell(command: &str) -> CommandSpec {
    CommandSpec::Shell {
        shell: "/bin/sh".into(),
        command: command.into(),
    }
}

/// 验证子进程正常退出（含退出码 0、非零码 7、127）、启动失败（`ENOENT`）与信号终止（`SIGTERM`）各自保持独立的 `ExitOutcome` 编码。
///
/// - **测试目标**：确保进程执行器能够精确区分 `exec` 阶段的 `ENOENT` 启动失败与 shell 返回的退出码 127，并保留原始 `wait` 状态字及信号编号。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 通过 shell 依次执行 `exit 0`、`exit 7`、`exit 127`；When 运行结束；Then 返回 `ExitOutcome::Exited`，且 `raw` 字段等于 `code << 8`。
///   2. Given 直接 `Exec` 一个不存在的可执行文件路径；When 运行结束；Then 返回 `ExitOutcome::SpawnError { errno: Some(ENOENT), .. }`。
///   3. Given 通过 shell 执行 `kill -TERM $$` 向自身发送终止信号；When 运行结束；Then 返回 `ExitOutcome::Signaled { signal: SIGTERM, raw: SIGTERM }`。
#[test]
fn real_exit_and_spawn_states_are_distinct() {
    for code in [0, 7, 127] {
        let scratch = Scratch::new();
        let (run, _, _) = scratch.run(shell(&format!("exit {code}")), Duration::from_secs(2));
        assert_eq!(
            run.outcome,
            ExitOutcome::Exited {
                code,
                raw: code << 8
            }
        );
        assert!(run.io_errors.is_empty());
    }
    let scratch = Scratch::new();
    let (run, _, _) = scratch.run(
        CommandSpec::Exec {
            program: scratch.0.join("absent").to_str().unwrap().into(),
            args: Vec::new(),
        },
        Duration::from_secs(2),
    );
    assert!(matches!(
        run.outcome,
        ExitOutcome::SpawnError {
            errno: Some(libc::ENOENT),
            ..
        }
    ));
    let scratch = Scratch::new();
    let (run, _, _) = scratch.run(shell("kill -TERM $$"), Duration::from_secs(2));
    assert!(matches!(
        run.outcome,
        ExitOutcome::Signaled {
            signal: libc::SIGTERM,
            raw: libc::SIGTERM
        }
    ));
}

/// 验证 `CommandSpec::Exec` 在传递工作目录（`cwd`）、含空格的环境变量、空字符串参数、含空格参数及 UTF-8 中文参数时的完整性。
///
/// - **测试目标**：确保参数数组与环境变量直接经由系统调用原样传递给被测程序，不发生二次 shell 展开、分词截断或编码损坏。
/// - **场景步骤（Given-When-Then）**：
///   - Given 构造 `CommandSpec::Exec` 调用 `/bin/sh -c`，设置工作目录为临时目录、环境变量 `CHECK="env with spaces"`，并传入空串 `""`、`"value with spaces"` 与 `"中文"` 三个位置参数；
///   - When 子进程使用 `printf '%s\000'` 将 `$PWD`、`$CHECK` 及三个参数以 NUL 分隔输出到 `stdout`；
///   - Then 捕获到的字节序列与预期完全一致。
#[test]
fn real_cwd_environment_and_argument_boundaries() {
    let scratch = Scratch::new();
    let stdout = scratch.output("stdout");
    let stderr = scratch.output("stderr");
    let spec = CommandSpec::Exec {
        program: "/bin/sh".into(),
        args: vec![
            "-c".into(),
            "printf '%s\\000' \"$PWD\" \"$CHECK\" \"$1\" \"$2\" \"$3\"".into(),
            "argv0".into(),
            "".into(),
            "value with spaces".into(),
            "中文".into(),
        ],
    };
    let env = BTreeMap::from([("CHECK".into(), "env with spaces".into())]);
    let run = run_command(
        None,
        &scratch.0,
        &spec,
        &env,
        Duration::from_secs(2),
        &Protocol::Plain,
        [stdout.as_raw_fd(), stderr.as_raw_fd()],
    );
    assert!(matches!(run.outcome, ExitOutcome::Exited { code: 0, .. }));
    let expected = format!(
        "{}\0env with spaces\0\0value with spaces\0中文\0",
        scratch.0.display()
    );
    assert_eq!(
        fs::read(scratch.0.join("stdout")).unwrap(),
        expected.as_bytes()
    );
}

/// 验证命令行参数或脚本中包含内嵌 NUL 字节（`\0`），以及指定的目标根目录（`root`）或工作目录（`cwd`）不存在时，均归类为 `ExitOutcome::SpawnError`。
///
/// - **测试目标**：确保 C 字符串转换错误与启动前目录切换/根目录切换失败均被安全捕获并报告为启动错误。
/// - **场景步骤（Given-When-Then）**：
///   1. Given `Exec` 参数含 `"bad\0arg"` 或 `Shell` 命令含 `"bad\0command"`；When 调用 `run_command`；Then 返回 `ExitOutcome::SpawnError`。
///   2. Given `cwd` 指向不存在的子目录，或 `root` 指向不存在的根目录；When 调用 `run_command`；Then 返回 `ExitOutcome::SpawnError`。
#[test]
fn nul_and_invalid_root_or_cwd_remain_spawn_errors() {
    let scratch = Scratch::new();
    let stdout = scratch.output("stdout");
    let stderr = scratch.output("stderr");
    for spec in [
        CommandSpec::Exec {
            program: "/bin/true".into(),
            args: vec!["bad\0arg".into()],
        },
        shell("bad\0command"),
    ] {
        let run = run_command(
            None,
            &scratch.0,
            &spec,
            &BTreeMap::new(),
            Duration::from_secs(1),
            &Protocol::Plain,
            [stdout.as_raw_fd(), stderr.as_raw_fd()],
        );
        assert!(matches!(run.outcome, ExitOutcome::SpawnError { .. }));
    }
    for (root, cwd) in [
        (None, scratch.0.join("missing")),
        (Some(scratch.0.join("missing")), PathBuf::from("/")),
    ] {
        let run = run_command(
            root.as_deref(),
            &cwd,
            &shell("exit 0"),
            &BTreeMap::new(),
            Duration::from_secs(1),
            &Protocol::Plain,
            [stdout.as_raw_fd(), stderr.as_raw_fd()],
        );
        assert!(matches!(run.outcome, ExitOutcome::SpawnError { .. }));
    }
}

/// 验证在需要捕获并转发输出的协议（如 `Protocol::Basic`）下，子进程并发向 `stdout` 和 `stderr` 写入大量数据（含 NUL 与 `0xff` 非文本字节）时不会发生管道缓冲区死锁，且数据分毫不差地转发至目标文件。
///
/// - **测试目标**：检验双流并发轮询转发器的吞吐能力与二进制透明性。
/// - **场景步骤（Given-When-Then）**：
///   - Given 子进程在 `START test_bulk` 与 `END test_bulk` 之间循环 8192 次向 `stdout` 和 `stderr` 各写入 32 字节（合计各 256 KiB，远超默认的 64 KiB 管道容量），并在末尾输出 `\x00\xff`；
///   - When 以 `Protocol::Basic` 运行该命令；
///   - Then 进程正常以 0 退出，协议判定成功，无 I/O 错误，且落盘的 `stdout` 与 `stderr` 文件内容与预期字节完全相等。
#[test]
fn large_stdout_and_stderr_are_forwarded_without_deadlock() {
    let scratch = Scratch::new();
    let command = "printf '========== START test_bulk ==========\\n'; i=0; while [ $i -lt 8192 ]; do printf '0123456789abcdef0123456789abcdef'; printf 'fedcba9876543210fedcba9876543210' >&2; i=$((i+1)); done; printf '\\000\\377\\n========== END test_bulk ==========\\n'";
    let (run, stdout, stderr) = scratch.run_protocol(
        shell(command),
        Duration::from_secs(10),
        &Protocol::Basic {
            name: "bulk".into(),
        },
    );
    assert_eq!(run.outcome, ExitOutcome::Exited { code: 0, raw: 0 });
    assert!(run.protocol.verdict.is_success(), "{:?}", run.protocol);
    assert!(run.io_errors.is_empty(), "{:?}", run.io_errors);
    let mut expected = b"========== START test_bulk ==========\n".to_vec();
    expected.extend(b"0123456789abcdef0123456789abcdef".repeat(8192));
    expected.extend([0, 255]);
    expected.extend(b"\n========== END test_bulk ==========\n");
    assert_eq!(stdout, expected);
    assert_eq!(stderr, b"fedcba9876543210fedcba9876543210".repeat(8192));
}

/// 验证即使被测子进程持续不断地产生输出并忽略 `SIGTERM` 信号，单次执行的总超时截止时间（Deadline）依然严格生效，最终通过 `SIGKILL` 强制终止并回收进程组。
///
/// - **测试目标**：确保超时计时器基于绝对墙钟截止时间而非空闲读超时（持续输出不能续期），且对屏蔽 `SIGTERM` 的顽固进程能升级使用 `SIGKILL` 清理。
/// - **场景步骤（Given-When-Then）**：
///   - Given 子进程通过 `trap '' TERM` 忽略 `SIGTERM` 并在死循环中不断打印 `"still running\n"`，设定超时时间为 100ms；
///   - When 以 `Protocol::Basic` 执行；
///   - Then 返回 `ExitOutcome::TimedOut { signal: Some(SIGKILL), .. }`，总耗时远小于 3 秒，且超时前已产生的部分 `stdout` 输出仍被成功捕获。
#[test]
fn continuous_output_obeys_one_total_deadline() {
    let scratch = Scratch::new();
    let started = Instant::now();
    let (run, stdout, _) = scratch.run_protocol(
        shell("printf '========== START test_deadline ==========\\n'; trap '' TERM; while :; do printf 'still running\\n'; done"),
        Duration::from_millis(100),
        &Protocol::Basic { name: "deadline".into() },
    );
    assert!(matches!(
        run.outcome,
        ExitOutcome::TimedOut {
            raw: Some(_),
            signal: Some(libc::SIGKILL)
        }
    ));
    assert!(started.elapsed() < Duration::from_secs(3));
    assert!(!stdout.is_empty());
}

/// 辅助函数：构造一个带有 `setup`、`cleanup` 命令及 `hooks: true` 的基准 `ResolvedCase`（默认主程序指向不存在的二进制文件以便测试启动失败路径）。
fn case(scratch: &Scratch) -> ResolvedCase {
    ResolvedCase {
        id: "runner:trace".into(),
        source: SourceLocation {
            file: PathBuf::from("host"),
            entry: 1,
            catalog: None,
            line: None,
        },
        root: "host".into(),
        cwd: scratch.0.clone(),
        command: CommandSpec::Exec {
            program: scratch.0.join("absent").to_str().unwrap().into(),
            args: Vec::new(),
        },
        env: BTreeMap::new(),
        timeout_sec: 2,
        repeat: 1,
        hooks: true,
        setup: vec![vec![
            "/bin/sh".into(),
            "-c".into(),
            "printf setup > setup".into(),
        ]],
        cleanup: vec![vec![
            "/bin/sh".into(),
            "-c".into(),
            "printf cleanup > cleanup".into(),
        ]],
        protocol: Protocol::Plain,
    }
}

/// 验证当 `setup` 与 `before` Hook 已成功执行后，即使主程序在启动阶段失败（`SpawnError`），执行器仍会成对调用 `after` Hook 并运行全部 `cleanup` 命令，且 `cleanup` 的非零退出不会覆盖主程序的错误状态。
///
/// - **测试目标**：确保 `before`/`after` Hook 的配对不变量与 `cleanup` 的无条件清理保障。
/// - **场景步骤（Given-When-Then）**：
///   - Given 用例启用了 `hooks`，`setup` 创建标记文件，主程序为不存在的路径 `absent`，`cleanup` 包含创建标记文件与 `exit 9` 两条命令；
///   - When 调用 `execute_case`；
///   - Then Hook 回调按 `[true, false]` 顺序被调用两次，主执行结果保持为 `SpawnError(ENOENT)`，`cleanup` 文件成功生成，且 `exit 9` 被单独记录在 `record.cleanup_errors` 中。
#[test]
fn spawn_failure_still_runs_paired_after_and_cleanup() {
    let scratch = Scratch::new();
    let output = scratch.output("output");
    let device = scratch.0.join("hook");
    File::create(&device).unwrap();
    let mut case = case(&scratch);
    case.cleanup
        .push(vec!["/bin/sh".into(), "-c".into(), "exit 9".into()]);
    let mut calls = Vec::new();
    let record = execute_case(
        None,
        &case,
        2,
        [output.as_raw_fd(), output.as_raw_fd()],
        |before, id| {
            assert!(scratch.0.join("setup").exists());
            if !before {
                assert_eq!(fs::read(&device).unwrap(), b"before runner:trace\n");
            }
            calls.push(before);
            hook_device(&device, before, id)
        },
    );
    assert_eq!(calls, vec![true, false]);
    assert!(matches!(
        record.execution.unwrap().outcome,
        ExitOutcome::SpawnError {
            errno: Some(libc::ENOENT),
            ..
        }
    ));
    assert_eq!(record.iteration, 2);
    assert_eq!(fs::read(scratch.0.join("cleanup")).unwrap(), b"cleanup");
    assert_eq!(record.cleanup_errors.len(), 1);
}

/// 验证当主程序已执行完毕并取得退出状态后，若 `after` Hook 返回错误，该错误会标记用例必须停止后续迭代（`must_stop`），但不会覆盖已获得的主进程退出码，且 `cleanup` 依然照常运行。
///
/// - **测试目标**：确保后置 Hook 错误与主程序退出结果各自独立记录，互不掩盖。
/// - **场景步骤（Given-When-Then）**：
///   - Given 主命令为 `exit 7`，`before` Hook 返回 `Ok(())`，`after` Hook 返回 `Err("after failed")`；
///   - When 调用 `execute_case`；
///   - Then `record.must_stop()` 为 `true`，主结果仍为 `Exited { code: 7, .. }`，`record.after_error` 记录 `"after failed"`，且 `cleanup` 标记文件成功创建。
#[test]
fn after_error_does_not_replace_primary_outcome() {
    let scratch = Scratch::new();
    let output = scratch.output("output");
    let mut case = case(&scratch);
    case.command = shell("exit 7");
    let record = execute_case(
        None,
        &case,
        1,
        [output.as_raw_fd(), output.as_raw_fd()],
        |before, _| {
            if before {
                Ok(())
            } else {
                Err(io::Error::new(io::ErrorKind::Other, "after failed"))
            }
        },
    );
    assert!(record.must_stop());
    assert_eq!(
        record.execution.as_ref().unwrap().outcome,
        ExitOutcome::Exited {
            code: 7,
            raw: 7 << 8
        }
    );
    assert_eq!(record.after_error.as_deref(), Some("after failed"));
    assert!(scratch.0.join("cleanup").exists());
}

/// 验证当 `hooks` 设为 `false` 时绝不调用 Hook 回调，且当 `setup` 命令执行失败时跳过主程序启动但依然执行 `cleanup` 清理序列。
///
/// - **测试目标**：确保前置准备失败时的短路保护与资源清理契约。
/// - **场景步骤（Given-When-Then）**：
///   - Given `case.hooks = false`，`case.setup` 为 `exit 3`，Hook 闭包内设为一旦调用即 `panic!`；
///   - When 调用 `execute_case`；
///   - Then Hook 未被触发，`record.execution` 为 `None`，`record.setup_error` 为 `Some`，`record.must_stop()` 为 `true`，且 `cleanup` 标记文件仍被创建。
#[test]
fn setup_failure_and_disabled_hooks_have_explicit_tails() {
    let scratch = Scratch::new();
    let output = scratch.output("output");
    let mut case = case(&scratch);
    case.hooks = false;
    case.setup = vec![vec!["/bin/sh".into(), "-c".into(), "exit 3".into()]];
    let record = execute_case(
        None,
        &case,
        1,
        [output.as_raw_fd(), output.as_raw_fd()],
        |_, _| panic!("disabled hook called"),
    );
    assert!(record.execution.is_none());
    assert!(record.setup_error.is_some());
    assert!(record.must_stop());
    assert!(scratch.0.join("cleanup").exists());
}

/// 验证当输出转发目标（如 `/dev/full`）写入失败时，I/O 错误被独立记录，既不会篡改子进程真实的退出码（`exit 7`），也不会污染外部传入文件描述符的打开状态标志（如 `O_NONBLOCK`）。
///
/// - **测试目标**：确保输出转发失败与进程退出状态解耦，且执行器不会修改调用方提供的共享输出描述符的底层文件状态标志。
/// - **场景步骤（Given-When-Then）**：
///   - Given 打开 `/dev/full` 作为 `outputs` 描述符，记录其初始 `F_GETFL` 标志，运行输出文本并以 `exit 7` 退出的命令；
///   - When 调用 `run_command`；
///   - Then `run.outcome` 保持为 `Exited { code: 7, .. }`，`run.io_errors` 非空，且运行后 `/dev/full` 的 `F_GETFL` 标志与初始值一致。
#[test]
fn output_error_preserves_wait_and_shared_output_flags() {
    let scratch = Scratch::new();
    let full = OpenOptions::new().write(true).open("/dev/full").unwrap();
    // SAFETY: `full` 在当前测试函数作用域内始终保持打开有效，`F_GETFL` 仅读取文件状态标志，不解引用任何内存指针。
    let initial = unsafe { libc::fcntl(full.as_raw_fd(), libc::F_GETFL) };
    let run = run_command(
        None,
        &scratch.0,
        &shell("printf output; exit 7"),
        &BTreeMap::new(),
        Duration::from_secs(2),
        &Protocol::Basic {
            name: "output_error".into(),
        },
        [full.as_raw_fd(), full.as_raw_fd()],
    );
    assert_eq!(
        run.outcome,
        ExitOutcome::Exited {
            code: 7,
            raw: 7 << 8
        }
    );
    assert!(!run.io_errors.is_empty());
    // SAFETY: `full` 由当前栈帧持有且尚未关闭，再次调用 `F_GETFL` 仅读取其底层打开文件描述（OFD）的当前状态标志。
    assert_eq!(
        unsafe { libc::fcntl(full.as_raw_fd(), libc::F_GETFL) },
        initial
    );
}

/// 验证 `hook_device` 在向控制设备写入前会严格校验用例 ID 格式，拒绝空串、含空格/NUL/换行符或超长（512 字节）的输入，且校验失败时不触碰目标文件。
///
/// - **测试目标**：防止格式非法的用例 ID 破坏内核或宿主 Hook 控制设备的行协议解析。
/// - **场景步骤（Given-When-Then）**：
///   - Given 预先写入 `"original"` 内容的模拟设备文件，以及 5 种非法 ID（空串、含空格、含 `\0`、含 `\n`、512 个 `'a'`）；
///   - When 依次调用 `hook_device`；
///   - Then 均返回 `Err`，且设备文件内容始终保持为 `"original"`。
#[test]
fn hook_validation_happens_before_device_write() {
    let scratch = Scratch::new();
    let device = scratch.0.join("hook");
    fs::write(&device, b"original").unwrap();
    for id in ["", "bad id", "bad\0id", "bad\nid", &"a".repeat(512)] {
        assert!(hook_device(Path::new(&device), true, id).is_err());
        assert_eq!(fs::read(&device).unwrap(), b"original");
    }
}

/// 验证当外部输出接收端（如无人读取的 Unix 套接字对）缓冲区写满阻塞时，执行器的总超时截止时间依然准时生效，并将未能转发的残余字节记录为独立的 `io_errors`。
///
/// - **测试目标**：防止慢速或停滞的外部输出消费者卡死执行器的超时回收循环。
/// - **场景步骤（Given-When-Then）**：
///   - Given 创建一对 `UnixStream` 且不读取对端，子进程忽略 `SIGTERM` 并死循环高速输出文本，设定超时为 100ms；
///   - When 以 `Protocol::Basic` 运行；
///   - Then 准时触发 `ExitOutcome::TimedOut`，未转发完的数据产生非空 `run.io_errors`，且总运行时间远小于 3 秒。
#[test]
fn blocked_output_sink_does_not_suspend_deadline() {
    let scratch = Scratch::new();
    let (sink, _unread) = std::os::unix::net::UnixStream::pair().unwrap();
    let run = run_command(
        None,
        &scratch.0,
        &shell("trap '' TERM; while :; do printf '0123456789abcdef0123456789abcdef'; done"),
        &BTreeMap::new(),
        Duration::from_millis(100),
        &Protocol::Basic {
            name: "blocked".into(),
        },
        [sink.as_raw_fd(), sink.as_raw_fd()],
    );
    assert!(matches!(
        run.outcome,
        ExitOutcome::TimedOut { raw: Some(_), .. }
    ));
    assert!(!run.io_errors.is_empty());
    assert!(run.elapsed < Duration::from_secs(3));
}

/// 验证 `runtime::diagnose` 在诊断 ELF 动态链接器（`PT_INTERP`）路径时，会将绝对符号链接限制在目标根目录（`root.path`）内部进行解析，而不会逃逸到宿主机的根目录 `/`。
///
/// - **测试目标**：确保跨根环境（chroot/目标根）下的启动失败诊断能准确检查目标根内的可执行文件与动态加载器。
/// - **场景步骤（Given-When-Then）**：
///   - Given 在临时目录 `scratch` 下构造 `/application -> /bin/app` 绝对符号链接，`/bin/app` 为含 `PT_INTERP = "/lib/alias"` 的 64 位小端 ELF 头，且 `/lib/alias -> /lib/loader` 指向临时目录内的真实文件；
///   - When 以 `scratch` 作为 `RootSpec` 调用 `runtime::diagnose` 诊断 `/application`；
///   - Then 诊断消息确认动态链接器在目标根目录内存在（`"present in target root"`），且未读取或篡改宿主机 `/lib` 下的文件。
#[test]
fn interpreter_diagnostic_resolves_absolute_links_inside_target_root() {
    // Step 1: 在临时目标根目录内创建模拟加载器、绝对路径符号链接以及带 PT_INTERP 段头的最小 64 位 ELF 文件。
    let scratch = Scratch::new();
    fs::create_dir(scratch.0.join("bin")).unwrap();
    fs::create_dir(scratch.0.join("lib")).unwrap();
    fs::write(scratch.0.join("lib/loader"), b"supplied loader bytes").unwrap();
    std::os::unix::fs::symlink("/lib/loader", scratch.0.join("lib/alias")).unwrap();
    let interp = b"/lib/alias\0";
    let mut elf = vec![0u8; 120];
    elf[..6].copy_from_slice(b"\x7fELF\x02\x01");
    elf[32..40].copy_from_slice(&64u64.to_le_bytes());
    elf[54..56].copy_from_slice(&56u16.to_le_bytes());
    elf[56..58].copy_from_slice(&1u16.to_le_bytes());
    elf[64..68].copy_from_slice(&3u32.to_le_bytes());
    elf[72..80].copy_from_slice(&120u64.to_le_bytes());
    elf[96..104].copy_from_slice(&(interp.len() as u64).to_le_bytes());
    elf.extend(interp);
    fs::write(scratch.0.join("bin/app"), elf).unwrap();
    std::os::unix::fs::symlink("/bin/app", scratch.0.join("application")).unwrap();
    // Step 2: 构造指向该临时目录的 RootSpec 并调用 diagnose，验证绝对符号链接在目标根内部被正确解析。
    let root = contest_run::model::RootSpec {
        path: scratch.0.clone(),
        cwd: "/".into(),
        env: BTreeMap::new(),
        mounts: Vec::new(),
        prepare: Vec::new(),
    };
    let diagnostic = contest_run::runtime::diagnose(
        &root,
        Path::new("/"),
        &CommandSpec::Exec {
            program: "/application".into(),
            args: Vec::new(),
        },
    );
    assert!(
        diagnostic.contains("present in target root"),
        "{diagnostic}"
    );
    assert_eq!(
        fs::read(scratch.0.join("lib/loader")).unwrap(),
        b"supplied loader bytes"
    );
}

/// 验证当测试运行器（worker）与被测子进程（consumer）的 `stdin` 和 `stdout` 共享同一个底层打开文件描述（Open File Description，常见于串口控制台或套接字终端）时，无论使用 `Plain` 直接输出还是 `Basic` 管道捕获转发，都不会把共享 OFD 污染为非阻塞（`O_NONBLOCK`）模式，从而保证被测子进程的阻塞式 `stdin` 读取正常工作。
///
/// - **测试目标**：防止输出转发器在设置非阻塞 I/O 时误改外部共享控制台的 `O_NONBLOCK` 标志，导致子进程读 `stdin` 立即遭遇 `EAGAIN`。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 主测试进程创建一对 `UnixStream`（`console` 与 `peer`），将 `console` 同时复制为 `worker` 子进程的 `stdin` 与 `stdout`（二者共享同一内核 OFD），分别测试 `"plain"` 与 `"basic"` 两种模式；
///   2. When `worker` 通过 `run_command` 启动 `consumer` 子进程，`consumer` 先打印就绪握手行 `READY` 并等待父进程确认收到转发输出；
///   3. Then 父进程从 `peer` 读到 `READY` 后创建 `gate` 同步文件，延迟 100ms 再向 `peer` 写入 `"delayed stdin data\n"`；
///   4. Then `consumer` 通过 `fcntl(STDIN_FILENO, F_GETFL)` 断言 `O_NONBLOCK` 位为 0，随后阻塞调用 `read_line` 成功读到延迟到达的 `"delayed stdin data\n"` 并正常退出。
#[test]
fn shared_console_ofd_preserves_blocking_stdin() {
    use std::io::{BufRead, Read, Write};
    use std::os::fd::OwnedFd;
    use std::os::unix::net::UnixStream;
    use std::process::{Command, Stdio};

    const TEST: &str = "shared_console_ofd_preserves_blocking_stdin";
    const ROLE: &str = "CONTEST_F4_STDIN_ROLE";
    const MODE: &str = "CONTEST_F4_STDIN_MODE";
    const GATE: &str = "CONTEST_F4_STDIN_GATE";
    const READY: &str = "F4 stdin probe ready";

    // 根据环境变量 ROLE 区分当前进程在三层自举测试中的角色：consumer（被测孙进程）、worker（运行器子进程）或主测试协调进程。
    match std::env::var(ROLE).as_deref() {
        Ok("consumer") => {
            // 角色 A (consumer)：打印就绪信号，等待父进程确认输出泵已工作后，检查 stdin 未被设为非阻塞并执行阻塞读取。
            println!("\n========== START test_stdin ==========");
            println!("{READY}");
            io::stdout().flush().unwrap();
            let gate = PathBuf::from(std::env::var_os(GATE).unwrap());
            let deadline = Instant::now() + Duration::from_secs(2);
            while !gate.exists() {
                assert!(
                    Instant::now() < deadline,
                    "parent did not observe pump handshake"
                );
                std::thread::sleep(Duration::from_millis(5));
            }
            // SAFETY: `STDIN_FILENO` (0) 是当前进程继承的有效标准输入描述符，`F_GETFL` 仅读取文件状态标志，不涉及指针访问。
            let flags = unsafe { libc::fcntl(libc::STDIN_FILENO, libc::F_GETFL) };
            assert!(
                flags != -1,
                "child stdin is invalid: {}",
                io::Error::last_os_error()
            );
            assert_eq!(
                flags & libc::O_NONBLOCK,
                0,
                "runner changed inherited child stdin to nonblocking while pumping stdout"
            );
            let mut input = String::new();
            io::stdin().read_line(&mut input).unwrap();
            assert_eq!(input, "delayed stdin data\n");
            println!("========== END test_stdin ==========");
        }
        Ok("worker") => {
            // 角色 B (worker)：调用被测的 `run_command` 启动 consumer，并将输出转发至自身的 stdout/stderr（与 stdin 共享同一 OFD）。
            let scratch = Scratch::new();
            let spec = CommandSpec::Exec {
                program: std::env::current_exe()
                    .unwrap()
                    .into_os_string()
                    .into_string()
                    .unwrap(),
                args: vec!["--exact".into(), TEST.into(), "--nocapture".into()],
            };
            let env = BTreeMap::from([(ROLE.into(), "consumer".into())]);
            let protocol = if std::env::var(MODE).unwrap() == "basic" {
                Protocol::Basic {
                    name: "stdin".into(),
                }
            } else {
                Protocol::Plain
            };
            let run = run_command(
                None,
                &scratch.0,
                &spec,
                &env,
                Duration::from_secs(2),
                &protocol,
                [libc::STDOUT_FILENO, libc::STDERR_FILENO],
            );
            assert!(run.io_errors.is_empty(), "{:?}", run.io_errors);
            assert_eq!(run.outcome, ExitOutcome::Exited { code: 0, raw: 0 });
            assert!(run.protocol.verdict.is_success(), "{:?}", run.protocol);
        }
        _ => {
            // 角色 C (主测试进程)：创建共享同一底层 OFD 的 stdin/stdout 套接字，验证 plain 与 basic 模式下延迟写入的 stdin 均能被阻塞读取。
            let _exclusive = exclusive_execution();
            for mode in ["plain", "basic"] {
                let scratch = Scratch::new();
                let gate = scratch.0.join("pump-forwarded-handshake");
                let (console, peer) = UnixStream::pair().unwrap();
                peer.set_read_timeout(Some(Duration::from_secs(4))).unwrap();
                let mut sender = peer.try_clone().unwrap();
                let stdin: OwnedFd = console.try_clone().unwrap().into();
                let stdout: OwnedFd = console.try_clone().unwrap().into();
                let mut worker = Command::new(std::env::current_exe().unwrap())
                    .args(["--exact", TEST, "--nocapture"])
                    .env(ROLE, "worker")
                    .env(MODE, mode)
                    .env(GATE, &gate)
                    .stdin(Stdio::from(stdin))
                    .stdout(Stdio::from(stdout))
                    .stderr(Stdio::inherit())
                    .spawn()
                    .unwrap();
                drop(console);
                let mut reader = io::BufReader::new(peer);
                let mut transcript = String::new();
                let observed = loop {
                    let mut line = String::new();
                    match reader.read_line(&mut line) {
                        Ok(0) => break false,
                        Ok(_) => {
                            let ready = line.contains(READY);
                            transcript.push_str(&line);
                            if ready {
                                break true;
                            }
                        }
                        Err(error) => {
                            transcript.push_str(&format!("console read: {error}"));
                            break false;
                        }
                    }
                };
                let sent = if observed {
                    fs::write(&gate, b"stdout forwarded by pump").unwrap();
                    std::thread::sleep(Duration::from_millis(100));
                    sender.write_all(b"delayed stdin data\n")
                } else {
                    Ok(())
                };
                let remaining = reader.read_to_string(&mut transcript);
                let status = worker.wait().unwrap();
                assert!(
                    observed,
                    "{mode} consumer handshake was not forwarded: {transcript}"
                );
                assert!(
                    sent.is_ok(),
                    "{mode} delayed stdin write failed: {sent:?}; {transcript}"
                );
                assert!(
                    remaining.is_ok(),
                    "{mode} console tail read failed: {remaining:?}; {transcript}"
                );
                assert!(
                    status.success(),
                    "{mode} shared-OFD worker failed: {status}; {transcript}"
                );
            }
        }
    }
}

/// 验证当被测子进程已经打印完全部输出并以退出码 7 退出，但外部输出接收端（`outputs`）因缓冲区满而无法在截止时间内排空转发缓冲时，执行器仍保留子进程真实的 `Exited { code: 7 }` 状态，仅将排空超时附加到 `io_errors` 中。
///
/// - **测试目标**：确保子进程自身的退出状态与宿主机输出转发管道的排空超时严格分离。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 创建一对 `UnixStream`，预先将写端 `sink` 塞满直至返回 `WouldBlock`，随后切回阻塞模式且不读取对端；
///   2. When 子进程输出简短的 `START`/`END` 标记（写入内部捕获管道即刻完成）并立即执行 `exit 7`，设定超时为 300ms；
///   3. Then 子进程正常退出被回收，`run.outcome` 为 `Exited { code: 7, .. }`，而转发器因无法向已满的 `sink` 写出数据，在到期后向 `run.io_errors` 记录 `"output relay did not drain before deadline"`。
#[test]
fn relay_drain_deadline_preserves_completed_child_status() {
    use std::io::Write;

    // Step 1: 构造一个发送缓冲区已被彻底填满且无人消费的 UnixStream 作为输出接收端。
    let scratch = Scratch::new();
    let (mut sink, _unread) = std::os::unix::net::UnixStream::pair().unwrap();
    sink.set_nonblocking(true).unwrap();
    let fill = [0u8; 8192];
    loop {
        match sink.write(&fill) {
            Ok(0) => panic!("socket stopped making progress without EAGAIN"),
            Ok(_) => {}
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => break,
            Err(error) => panic!("fill output socket: {error}"),
        }
    }
    // Step 2: 运行快速退出的子进程，验证子进程退出码 7 被完整保留，同时排空超时被记录进 io_errors。
    sink.set_nonblocking(false).unwrap();
    let run = run_command(
        None,
        &scratch.0,
        &shell("printf '========== START test_tail ==========\\n========== END test_tail ==========\\n'; exit 7"),
        &BTreeMap::new(),
        Duration::from_millis(300),
        &Protocol::Basic { name: "tail".into() },
        [sink.as_raw_fd(), sink.as_raw_fd()],
    );
    assert_eq!(
        run.outcome,
        ExitOutcome::Exited {
            code: 7,
            raw: 7 << 8
        }
    );
    assert!(
        run.io_errors
            .iter()
            .any(|error| error.contains("output relay did not drain before deadline")),
        "{:?}",
        run.io_errors
    );
    assert!(run.elapsed < Duration::from_secs(3));
}

/// 验证在 `Protocol::Tap` 协议下，若子进程输出完整的成功 TAP 文本并以 0 退出，但外部输出目标（`/dev/full`）写入失败，则进程退出状态仍保留为 `Exited(0)`，而协议最终判定被降级为 `Verdict::InfrastructureError`。
///
/// - **测试目标**：防止输出日志落盘失败时丢失基础设施告警而误报测试通过。
/// - **场景步骤（Given-When-Then）**：
///   - Given 输出目标为 `/dev/full`，子进程打印合法的 `TAP version 13\n1..1\nok 1 completed\n` 并以 0 退出；
///   - When 调用 `run_command`；
///   - Then `run.outcome` 保持 `Exited { code: 0, raw: 0 }`，`run.io_errors` 非空，且 `run.protocol.verdict` 为 `Verdict::InfrastructureError`。
#[test]
fn tap_output_error_preserves_wait_and_fails_result() {
    let scratch = Scratch::new();
    let full = OpenOptions::new().write(true).open("/dev/full").unwrap();
    let run = run_command(
        None,
        &scratch.0,
        &shell("printf 'TAP version 13\\n1..1\\nok 1 completed\\n'"),
        &BTreeMap::new(),
        Duration::from_secs(2),
        &Protocol::Tap,
        [full.as_raw_fd(), full.as_raw_fd()],
    );
    assert_eq!(run.outcome, ExitOutcome::Exited { code: 0, raw: 0 });
    assert!(!run.io_errors.is_empty());
    assert_eq!(
        run.protocol.verdict,
        contest_run::outcome::Verdict::InfrastructureError
    );
}
