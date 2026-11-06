//! `contest-run` 二进制主入口模块。
//!
//! ## 模块职责与架构定位
//! 本模块是比赛用户态测试执行器（`contest-run`）的命令行入口与顶层编排器。
//! 上游接收内核初始进程或用户传入的命令行参数（如块设备路径、测试计划文件、或单条显式命令），
//! 下游调用 `config`、`selection`、`runtime`、`execute` 与 `process` 等子模块，
//! 完成工作盘挂载、测试用例解析与筛选、测试根目录（chroot 环境）生命周期管理、逐例执行以及最终的 `sync` 与关机控制。
//!
//! ## 核心执行流程与状态机
//! 正式测试计划模式与单条显式命令模式共用同一套挂载根、执行函数与资源释放路径：
//!
//! ```text
//!   parse_cli()
//!       │
//!       ├── 显式命令模式 (--root ... --cwd ... -- CMD) ──► explicit_config()
//!       └── 计划文件模式 (--plan / --file / --list)   ──► config::load()
//!                                                             │
//!                                                             ▼
//!                                            挂载比赛外置盘至 CONTEST_ROOT (/mnt/contest)
//!                                                             │
//!                                                             ▼
//!                                           ┌─────────── mounted() ───────────┐
//!                                           │ --list: 仅解析并打印选中用例    │
//!                                           │ 正常执行: run_cases()           │
//!                                           │   ├─ 按 root 分组准备与释放挂载 │
//!                                           │   ├─ 打印协议分组起止边界标记   │
//!                                           │   └─ 循环 repeat 次调用 execute │
//!                                           └─────────────────┬───────────────┘
//!                                                             │
//!                                                             ▼
//!                                                卸载 CONTEST_ROOT 工作盘
//!                                                             │
//!                                                             ▼
//!                                       非 --list 模式执行 finish() (sync 及可选 poweroff)
//! ```
//!
//! ## 并发与锁契约
//! 本模块在单线程顺序控制流中运行，不持有跨线程共享锁；测试子进程与输出转发辅助进程的同步由 `process` 模块通过进程组信号与管道 EOF 集中管理。

use contest_run::config;
use contest_run::custom;
use contest_run::execute::{execute_case, CaseRecord};
use contest_run::model::{
    CommandSpec, Config, Defaults, Finish, Protocol, ResolvedCase, RootSpec, SourceLocation,
};
use contest_run::outcome::ExitOutcome;
use contest_run::process::run_command;
use contest_run::runtime::{diagnose, Mounts, CONTEST_ROOT};
use contest_run::selection;
use std::collections::BTreeMap;
use std::env;
use std::path::{Component, Path, PathBuf};
use std::time::Duration;

/// 命令行参数解析结果。
struct Cli {
    /// 承载比赛测试套件的块设备路径（必须位于 `/dev` 目录下，如 `/dev/vda`）。
    device: PathBuf,
    /// 主测试计划 TOML 配置文件路径（默认为 `/etc/contest/plan.toml`）。
    plan: PathBuf,
    /// 附加的覆盖或补充测试计划文件路径列表（通过 `--file` 传入）。
    files: Vec<PathBuf>,
    /// 是否处于仅列出选中测试用例的干跑模式（`--list`）。
    list: bool,
    /// 显式单命令执行模式参数：`Some((root目录, 工作目录cwd, 命令及参数数组))`。
    command: Option<(PathBuf, PathBuf, Vec<String>)>,
}

/// 解析命令行参数并校验基础格式约束。
///
/// 在普通运行上下文中调用。参数格式错误会在挂载任何文件系统前直接返回错误说明；
/// 块设备路径需位于 `/dev` 下，显式命令模式的命令参数数组直接透传给 `exec`，不经过 shell 二次拆分。
fn parse_cli() -> Result<Cli, String> {
    // Step 1: 收集全部命令行参数并校验 UTF-8 编码与必填的块设备参数。
    let args = env::args_os()
        .skip(1)
        .map(|arg| {
            arg.into_string()
                .map_err(|_| "CLI requires UTF-8 arguments".to_string())
        })
        .collect::<Result<Vec<_>, _>>()?;
    let device = args.first().ok_or_else(|| "Usage: contest-run DEVICE [--plan FILE] [--file FILE ...] [--list] | DEVICE --root ROOT --cwd CWD -- COMMAND [ARG...]".to_string())?;
    if Path::new(device).parent() != Some(Path::new("/dev")) {
        return Err(format!("invalid contest device: {device}"));
    }
    let mut cli = Cli {
        device: device.into(),
        plan: "/etc/contest/plan.toml".into(),
        files: Vec::new(),
        list: false,
        command: None,
    };
    // Step 2: 顺序扫描可选标志或显式单命令模式（`--root ROOT --cwd CWD -- COMMAND...`）。
    let mut index = 1;
    while index < args.len() {
        match args[index].as_str() {
            "--plan" | "--file" => {
                let value = args
                    .get(index + 1)
                    .ok_or_else(|| format!("missing value for {}", args[index]))?;
                if args[index] == "--plan" {
                    cli.plan = value.into();
                } else {
                    cli.files.push(value.into());
                }
                index += 2;
            }
            "--list" => {
                cli.list = true;
                index += 1;
            }
            "--root" if index == 1 => {
                if args.get(3).map(String::as_str) != Some("--cwd")
                    || args.get(5).map(String::as_str) != Some("--")
                    || args.len() < 7
                {
                    return Err("expected DEVICE --root ROOT --cwd CWD -- COMMAND [ARG...]".into());
                }
                let root = PathBuf::from(&args[2]);
                let cwd = PathBuf::from(&args[4]);
                if !root.is_absolute()
                    || root
                        .components()
                        .any(|part| matches!(part, Component::ParentDir))
                    || !cwd.is_absolute()
                    || args[6].is_empty()
                {
                    return Err("explicit root/cwd must be absolute, root must not contain '..', and command must not be empty".into());
                }
                cli.command = Some((root, cwd, args[6..].to_vec()));
                return Ok(cli);
            }
            argument => return Err(format!("unknown CLI argument: {argument}")),
        }
    }
    Ok(cli)
}

/// 将命令行显式指定的单条命令包装为标准的 `Config` 与单个 `ResolvedCase`。
///
/// 使单命令调试模式与正式 TOML 计划文件模式复用相同的挂载准备与进程执行逻辑；
/// 执行完毕后默认采用 `Finish::Return` 返回调用方，不触发系统关机。
fn explicit_config(
    cli: &Cli,
    root: &Path,
    cwd: &Path,
    argv: &[String],
) -> (Config, Vec<ResolvedCase>) {
    let root = RootSpec {
        path: Path::new(CONTEST_ROOT).join(root.strip_prefix("/").unwrap()),
        cwd: cwd.to_path_buf(),
        env: BTreeMap::new(),
        mounts: vec!["dev".into(), "proc".into(), "tmp".into()],
        prepare: Vec::new(),
    };
    let source = SourceLocation {
        file: PathBuf::from("<command-line>"),
        entry: 1,
        catalog: None,
        line: None,
    };
    let case = ResolvedCase {
        id: "command".into(),
        source,
        root: "command".into(),
        cwd: cwd.into(),
        command: CommandSpec::Exec {
            program: argv[0].clone(),
            args: argv[1..].to_vec(),
        },
        env: BTreeMap::new(),
        timeout_sec: 60,
        repeat: 1,
        hooks: false,
        setup: Vec::new(),
        cleanup: Vec::new(),
        protocol: Protocol::Plain,
    };
    let config = Config {
        roots: BTreeMap::from([("command".into(), root)]),
        defaults: Defaults::default(),
        steps: Vec::new(),
        overrides: Vec::new(),
        finish: Finish::Return,
        plan: cli.plan.clone(),
    };
    (config, vec![case])
}

/// 提取测试用例所属的旧版比赛分组标识 `(协议名, 根名称)`。
///
/// 仅 `Basic` 与 `Pre2025Libctest` 协议需要输出分组边界标记；其余协议返回 `None`。
fn group(case: &ResolvedCase) -> Option<(String, String)> {
    let name = match case.protocol {
        Protocol::Basic { .. } => "basic",
        Protocol::Pre2025Libctest { .. } => "libctest",
        _ => return None,
    };
    Some((name.into(), case.root.clone()))
}

/// 向标准输出打印旧版比赛协议的测试分组起始（`START`）或结束（`END`）标记行。
///
/// 该标记仅供宿主判题脚本识别分组边界，在单个测试用例启动前或结束后由主进程直接打印，不经过单测例的流式输出观察器。
fn group_marker(group: &(String, String), start: bool) {
    println!(
        "#### OS COMP TEST GROUP {} {}-{} ####",
        if start { "START" } else { "END" },
        group.0,
        group.1
    );
}

/// 打印单次测试用例迭代的执行结果摘要与各类基础设施或 I/O 错误日志。
///
/// 每次重复执行（`iteration`）都会独立输出一行 `CONTEST result` 记录，后续迭代不会覆盖先前的记录。
fn print_record(case: &ResolvedCase, record: &CaseRecord) {
    if let Some(run) = &record.execution {
        println!(
            "CONTEST result id={:?} iteration={} outcome={} status={:?} elapsed_ms={}",
            case.id,
            record.iteration,
            run.protocol.verdict.as_str(),
            run.outcome,
            run.elapsed.as_millis()
        );
        for error in &run.io_errors {
            eprintln!("CONTEST I/O error id={:?}: {error}", case.id);
        }
    } else {
        println!(
            "CONTEST result id={:?} iteration={} outcome=NotRun",
            case.id, record.iteration
        );
    }
    for (phase, error) in [
        ("setup", &record.setup_error),
        ("before", &record.before_error),
        ("after", &record.after_error),
    ] {
        if let Some(error) = error {
            eprintln!(
                "CONTEST infrastructure error id={:?} phase={phase}: {error}",
                case.id
            );
        }
    }
    for error in &record.cleanup_errors {
        eprintln!(
            "CONTEST infrastructure error id={:?} phase=cleanup: {error}",
            case.id
        );
    }
}

/// 将子进程退出结果转换为符合 Unix shell 惯例的进程退出码。
///
/// 正常退出返回原状态码；信号终止返回 `128 + signal`；超时返回 `124`；启动或管道 I/O 错误返回 `125`。
fn command_status(outcome: &ExitOutcome) -> i32 {
    match outcome {
        ExitOutcome::Exited { code, .. } => *code,
        ExitOutcome::Signaled { signal, .. } => 128 + signal,
        ExitOutcome::TimedOut { .. } => 124,
        ExitOutcome::SpawnError { .. } | ExitOutcome::IoError { .. } => 125,
    }
}

/// 按顺序执行展开后的全部测试用例列表，管理测试根环境的切换与分组边界输出。
///
/// 当相邻测试用例切换 `root` 时，会先卸载并清理上一个测试根挂载的伪文件系统（`dev`/`proc`/`tmp`），
/// 再准备下一个测试根。若遇到致命基础设施错误（`must_stop()`），后续未执行的用例及剩余轮次均记录为 `NotRun`。
fn run_cases(config: &Config, cases: &[ResolvedCase], explicit: bool) -> Result<i32, String> {
    let mut mounts = Mounts::default();
    let mut active_root: Option<&str> = None;
    let mut active_group = None;
    let mut result = 0;
    let mut failure = None;
    let mut stopped = false;
    for case in cases {
        // Step 1: 若先前用例触发了不可恢复的基础设施故障，跳过实际执行并直接登记 NotRun。
        if stopped {
            for iteration in 1..=case.repeat {
                println!(
                    "CONTEST result id={:?} iteration={iteration} outcome=NotRun",
                    case.id
                );
            }
            continue;
        }
        // Step 2: 当协议分组或所属测试根发生变化时，关闭旧分组标记并切换测试根挂载环境。
        if active_group != group(case) {
            if let Some(previous) = active_group.take() {
                group_marker(&previous, false);
            }
        }
        if active_root != Some(case.root.as_str()) {
            if let Err(error) = mounts.release() {
                failure = Some(error);
                stopped = true;
            }
            if !stopped {
                let root = &config.roots[&case.root];
                if let Err(error) = mounts.prepare_root(
                    root,
                    Duration::from_secs(config.defaults.timeout_sec),
                    [1, 2],
                ) {
                    failure = Some(error);
                    stopped = true;
                }
                active_root = Some(&case.root);
            }
        }
        if stopped {
            for iteration in 1..=case.repeat {
                println!(
                    "CONTEST result id={:?} iteration={iteration} outcome=NotRun",
                    case.id
                );
            }
            continue;
        }
        // Step 3: 若进入新的旧版协议分组，输出分组起始标记。
        if active_group.is_none() {
            active_group = group(case);
            if let Some(current) = &active_group {
                group_marker(current, true);
            }
        }
        // Step 4: 按用例配置的 repeat 次数循环执行，收集判定结果、诊断信息与停止条件。
        let root = &config.roots[&case.root];
        for iteration in 1..=case.repeat {
            if stopped {
                println!(
                    "CONTEST result id={:?} iteration={iteration} outcome=NotRun",
                    case.id
                );
                continue;
            }
            if let Protocol::Basic { name } = &case.protocol {
                println!("Testing {name} :");
            }
            let record = execute_case(Some(&root.path), case, iteration, [1, 2], custom::hook);
            print_record(case, &record);
            if let Some(run) = &record.execution {
                if matches!(run.outcome, ExitOutcome::SpawnError { .. }) {
                    eprintln!(
                        "CONTEST diagnostic id={:?}: {}",
                        case.id,
                        diagnose(root, &case.cwd, &case.command)
                    );
                }
                if explicit {
                    result = command_status(&run.outcome);
                }
            }
            if !record.is_success() && (!explicit || result == 0) {
                result = 1;
            }
            if record.must_stop() {
                stopped = true;
                result = if result == 0 { 1 } else { result };
            }
        }
    }
    // Step 5: 闭合末尾活动的分组标记，并释放最后一个测试根持有的挂载点。
    if !stopped {
        if let Some(current) = &active_group {
            group_marker(current, false);
        }
    }
    if let Err(error) = mounts.release() {
        if let Some(primary) = &failure {
            eprintln!("CONTEST primary infrastructure error: {primary}");
        }
        failure = Some(error);
    }
    if let Some(error) = failure {
        Err(error)
    } else {
        Ok(result)
    }
}

/// 在比赛块设备已挂载的前提下，完成用例筛选并根据模式执行列表展示或实际测试。
///
/// `--list` 模式只解析并输出选中用例的元数据与可执行文件诊断信息，不执行测试根准备脚本、自定义钩子或关机动作。
fn mounted(config: &Config, cli: &Cli, explicit: Option<Vec<ResolvedCase>>) -> Result<i32, String> {
    let cases = match explicit {
        Some(cases) => cases,
        None => selection::resolve(config, env::consts::ARCH)?,
    };
    if cli.list {
        for case in &cases {
            let root = &config.roots[&case.root];
            println!("CONTEST selected id={:?} source={} root={:?} path={:?} cwd={:?} command={:?} hooks={} repeat={} diagnostic={:?}", case.id, case.source, case.root, root.path, case.cwd, case.command, case.hooks, case.repeat, diagnose(root, &case.cwd, &case.command));
        }
        return Ok(0);
    }
    run_cases(config, &cases, cli.command.is_some())
}

/// 在全部测试执行并卸载工作盘后，执行 `/bin/busybox sync` 落盘并根据配置决定是否关机。
///
/// 若 `mode` 为 `Finish::Poweroff`，在同步完成后继续调用 `/bin/busybox poweroff`；
/// 若 `mode` 为 `Finish::Return`，仅输出完成状态码并返回。
fn finish(mode: Finish, status: i32) -> i32 {
    let env = BTreeMap::new();
    // Step 1: 调用 `/bin/busybox sync` 将页缓存中的脏数据刷回存储设备。
    let sync = CommandSpec::Exec {
        program: "/bin/busybox".into(),
        args: vec!["sync".into()],
    };
    let run = run_command(
        None,
        Path::new("/"),
        &sync,
        &env,
        Duration::from_secs(60),
        &Protocol::Plain,
        [1, 2],
    );
    let mut status = status;
    if !matches!(run.outcome, ExitOutcome::Exited { code: 0, .. }) || !run.io_errors.is_empty() {
        eprintln!("CONTEST sync error: {:?} {:?}", run.outcome, run.io_errors);
        if status == 0 {
            status = 1;
        }
    }
    println!("CONTEST finished status={status}");
    // Step 2: 若配置要求运行结束后关机，则调用 `/bin/busybox poweroff`。
    if mode == Finish::Poweroff {
        let poweroff = CommandSpec::Exec {
            program: "/bin/busybox".into(),
            args: vec!["poweroff".into()],
        };
        let run = run_command(
            None,
            Path::new("/"),
            &poweroff,
            &env,
            Duration::from_secs(60),
            &Protocol::Plain,
            [1, 2],
        );
        if !matches!(run.outcome, ExitOutcome::Exited { code: 0, .. }) || !run.io_errors.is_empty()
        {
            eprintln!(
                "CONTEST poweroff error: {:?} {:?}",
                run.outcome, run.io_errors
            );
            if status == 0 {
                status = 1;
            }
        }
    }
    status
}

/// 执行 `contest-run` 的完整生命周期：解析参数、加载配置、挂载比赛盘、运行或列出用例、卸载清理与收尾。
///
/// 挂载比赛块设备后的所有退出路径都会先调用 `contest.release()` 卸载挂载点，再根据是否为 `--list` 模式决定是否进入 `finish()`。
fn run() -> Result<i32, String> {
    let cli = parse_cli()?;
    let (config, explicit) = match &cli.command {
        Some((root, cwd, argv)) => {
            let (config, cases) = explicit_config(&cli, root, cwd, argv);
            (config, Some(cases))
        }
        None => (config::load(&cli.plan, &cli.files)?, None),
    };
    // Step 1: 挂载比赛块设备到 CONTEST_ROOT，并在挂载期间执行用例列表或测试主循环。
    let mut contest = Mounts::default();
    contest
        .mount(&cli.device, Path::new(CONTEST_ROOT), "ext4", false)
        .map_err(|error| format!("mount contest {}: {error}", cli.device.display()))?;
    let result = mounted(&config, &cli, explicit);
    let released = contest.release();
    // Step 2: 汇总测试执行状态与挂载卸载结果，非 --list 模式下执行最终的 sync 与 finish 动作。
    let mut status = match result {
        Ok(status) => status,
        Err(error) => {
            eprintln!("CONTEST infrastructure error: {error}");
            1
        }
    };
    if let Err(error) = released {
        eprintln!("CONTEST cleanup error: {error}");
        if status == 0 {
            status = 1;
        }
    }
    if cli.list {
        Ok(status)
    } else {
        Ok(finish(config.finish, status))
    }
}

/// 进程主入口：运行顶层流程并将结果或参数/配置错误映射为进程退出码（错误码 `2` 表示顶层配置或启动失败）。
fn main() {
    let status = match run() {
        Ok(status) => status,
        Err(error) => {
            eprintln!("CONTEST error: {error}");
            2
        }
    };
    std::process::exit(status);
}
