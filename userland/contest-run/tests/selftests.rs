//! `user-selftests` 命令行解析、测试清单筛选与汇总退出码集成测试。
//!
//! # 模块职责与定位
//! 本文件是 `contest-run` 工具箱中 `selftests` 子模块的集成测试套件。
//! 上游对接命令行入口（`user-selftests` 二进制程序及 `selftests::Options` 解析器），
//! 下游调用清单选择器 `selftests::select`、执行入口 `selftests::run`、TAP 协议流式观察器
//! `ProtocolObserver` 以及运行汇总器 `Summary`，验证参数校验、只读清单列举及程序级退出码优先级。
//!
//! # 核心流程与退出码优先级状态机
//!
//! ```text
//! CLI 参数 ──► Options::parse (拒绝重复/越界/非法超时)
//!                  │
//!                  ▼
//!           selftests::select (读取 kselftest-list.txt，校验 collection:program 格式与路径穿越)
//!                  │
//!                  ├──► --list 模式：仅向输出流打印清单条目，不访问或要求测试程序文件存在
//!                  │
//!                  └──► 执行模式：运行选中程序 ──► ProtocolObserver (解析 TAP 输出)
//!                                                      │
//!                                                      ▼
//!                                               Summary::record
//!                                                      │
//!                        ┌─────────────────────────────┼─────────────────────────────┐
//!                        ▼                             ▼                             ▼
//!              存在 I/O 转发错误或            存在测试用例失败 / XPASS         全部用例跳过 (SKIP)
//!              TAP 计划未完成 (Broken)        或 XFAIL 退出码非 0                   │
//!                        │                             │                             ▼
//!                        ▼                             ▼                         退出码 4
//!                    退出码 2                      退出码 1                 (若全部通过则为 0)
//! ```
//!
//! # 并发与锁契约
//! 本测试文件中的各个测试函数互相独立，通过进程 ID 与测试专属临时目录（`Scratch`）隔离文件系统副作用，不涉及跨线程共享锁。

use contest_run::model::Protocol;
use contest_run::outcome::{Execution, ExitOutcome};
use contest_run::report::{ProtocolObserver, StreamKind};
use contest_run::selftests::{self, Options, Summary};
use std::fs;
use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

/// 测试专用的临时工作目录守卫（RAII Fixture）。
///
/// 在测试开始时绑定创建的临时目录（元组字段 `0` 为临时目录路径），当测试函数结束离开作用域时，通过 `Drop` 自动递归清理该目录及其中的清单文件。
struct Scratch(PathBuf);

impl Drop for Scratch {
    /// 测试结束时自动删除本测试创建的临时目录及内部所有文件。
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

/// 验证 `--list` 模式仅读取清单文件而不检查可执行文件是否存在，且实际执行缺失程序或遇到非法清单/参数时能准确报错。
///
/// - **测试目标**：确保清单列举（`--list`）与程序执行阶段职责分离，同时拦截路径穿越、重复条目、缺失程序及输出写入失败。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 清单行缺少 `:` 分隔符（`fs/file_shared`）；When 调用 `selftests::select`；Then 返回格式解析错误。
///   2. Given 清单包含合法的 `fs:file_shared` 与 `net:tcp_udp` 且对应子目录与程序文件并未创建；When 执行 `user-selftests --list`；Then 退出码为 0 且原样输出两行条目。
///   3. Given 输出目标为 `/dev/full`；When 调用 `selftests::run` 打印列表；Then 因写入空间不足返回 I/O 错误。
///   4. Given 通过 `--collection` 与 `--program` 组合选择条目，但磁盘上不存在目标可执行文件；When 运行 `user-selftests`；Then 因启动程序失败（`ENOENT`）返回退出码 2。
///   5. Given 选择器或清单中包含不存在的条目、相对路径穿越（`..`）、重复条目、空清单或重复的 `--collection` 参数；When 解析或筛选；Then 均返回错误。
#[test]
fn list_reads_only_manifest_and_execution_reports_missing_program() {
    // Step 1: 准备独立临时目录，验证非法清单格式（使用 `/` 而非 `:`）会被拒绝。
    let directory = std::env::temp_dir().join(format!("selftest-list-{}", std::process::id()));
    fs::create_dir(&directory).unwrap();
    let scratch = Scratch(directory);
    let manifest = scratch.0.join("kselftest-list.txt");
    fs::write(&manifest, "fs/file_shared\n").unwrap();
    let args = vec![
        "--manifest".into(),
        manifest.to_str().unwrap().into(),
        "--list".into(),
    ];
    assert!(selftests::select(&Options::parse(&args).unwrap()).is_err());

    // Step 2: 写入合法清单并在不存在实际测试二进制文件的情况下运行 `--list`，验证只读列举成功。
    fs::write(&manifest, "fs:file_shared\nnet:tcp_udp\n").unwrap();
    let list = Command::new(env!("CARGO_BIN_EXE_user-selftests"))
        .args(&args)
        .output()
        .unwrap();
    assert_eq!(list.status.code(), Some(0));
    assert_eq!(list.stdout, b"fs:file_shared\nnet:tcp_udp\n");
    assert!(!scratch.0.join("fs").exists());
    // Step 3: 验证当输出流（`/dev/full`）写入失败时，`selftests::run` 会向上传播 I/O 错误。
    let mut full = fs::OpenOptions::new()
        .write(true)
        .open("/dev/full")
        .unwrap();
    assert!(selftests::run(&Options::parse(&args).unwrap(), &mut full, [1, 2]).is_err());

    // Step 4: 验证按集合与按单程序组合筛选的正确性，以及实际运行缺失二进制文件时返回退出码 2。
    let options = Options::parse(&[
        "--manifest".into(),
        manifest.to_str().unwrap().into(),
        "--collection".into(),
        "fs".into(),
        "--program".into(),
        "net:tcp_udp".into(),
    ])
    .unwrap();
    assert_eq!(selftests::select(&options).unwrap().len(), 2);
    let missing = Command::new(env!("CARGO_BIN_EXE_user-selftests"))
        .args([
            "--manifest",
            manifest.to_str().unwrap(),
            "--program",
            "fs:file_shared",
        ])
        .output()
        .unwrap();
    assert_eq!(missing.status.code(), Some(2));
    // Step 5: 验证未登记的程序名、含 `..` 的路径穿越选择器、重复/空白清单以及重复 CLI 选项均被拒绝。
    for selection in ["fs:missing", "../bad:test", "fs:../bad"] {
        let result = Options::parse(&[
            "--manifest".into(),
            manifest.to_str().unwrap().into(),
            "--program".into(),
            selection.into(),
        ])
        .and_then(|options| selftests::select(&options));
        assert!(result.is_err(), "{selection}");
    }
    for contents in ["fs:a\nfs:a\n", "fs:../a\n", ""] {
        fs::write(&manifest, contents).unwrap();
        assert!(selftests::select(&options).is_err());
    }
    assert!(Options::parse(&[
        "--collection".into(),
        "fs".into(),
        "--collection".into(),
        "fs".into()
    ])
    .is_err());
}

/// 验证 `--program-timeout-seconds` 参数对缺失值、非正数、浮点数、整数溢出及重复指定等非法输入的拦截行为。
///
/// - **测试目标**：确保超时时间解析器严格只接受正整数且不允许重复传入，防止非法超时预算进入底层进程执行器。
/// - **场景步骤（Given-When-Then）**：
///   - Given 命令行参数包含 `--program-timeout-seconds` 且后续跟随空缺、0、负数、带正号、小数、空串、`u64::MAX`、超出 `u64` 的大整数或第二次重复指定该选项；
///   - When 调用 `Options::parse`；
///   - Then 解析结果全部为 `Err`。
#[test]
fn program_timeout_rejects_invalid_cli_values() {
    for values in [
        vec![],
        vec!["0"],
        vec!["-1"],
        vec!["+1"],
        vec!["1.5"],
        vec![""],
        vec!["18446744073709551615"],
        vec!["18446744073709551616"],
        vec!["1", "--program-timeout-seconds", "2"],
    ] {
        let args = std::iter::once("--program-timeout-seconds")
            .chain(values)
            .map(String::from)
            .collect::<Vec<_>>();
        assert!(Options::parse(&args).is_err(), "{args:?}");
    }
}

/// 验证 `Summary` 汇总器根据 TAP 协议解析结果、进程退出码与输出 I/O 错误计算最终退出码的优先级规则。
///
/// - **测试目标**：确保成功（0）、全部跳过（4）、测试失败（1）与协议损坏/基础设施 I/O 错误（2）按严重程度正确汇总。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 不同类型的 TAP 标准输出与进程退出码组合（正常通过、单步跳过、0 计划跳过、单步失败、计划数未跑满、XFAIL 但进程以非零码退出）；
///   2. When 经 `ProtocolObserver` 解析并调用 `Summary::record` 记录首次运行；Then `summary.status()` 分别返回预期的退出码（0、4、1、2）。
///   3. When 向同一 `Execution` 追加一条输出转发 I/O 错误并再次记录到 `Summary`；Then 无论此前结果如何，最终汇总退出码均升级为最高优先级的错误码 2。
#[test]
fn summary_exit_codes_preserve_failure_and_skip_priorities() {
    // Step 1: 遍历典型的 TAP 输出与退出码组合，依次校验首次记录时的汇总退出码。
    for (text, code, expected) in [
        ("1..1\nok 1 passed\n", 0, 0),
        ("1..1\nok 1 # SKIP unavailable\n", 0, 4),
        ("1..0 # SKIP unavailable\n", 4, 4),
        ("1..1\nnot ok 1 failed\n", 0, 1),
        ("1..2\nok 1 incomplete\n", 0, 2),
        ("1..1\nok 1 # XFAIL expected\n", 2, 1),
    ] {
        let outcome = ExitOutcome::Exited {
            code,
            raw: code << 8,
        };
        let mut observer = ProtocolObserver::new(&Protocol::Tap);
        observer.feed(StreamKind::Stdout, text.as_bytes());
        let mut run = Execution {
            protocol: observer.finish(&outcome),
            outcome,
            io_errors: Vec::new(),
            elapsed: Duration::ZERO,
        };
        let mut summary = Summary::default();
        summary.record(&run);
        assert_eq!(summary.status(), expected, "{text}");
        // Step 2: 注入输出转发 I/O 错误后再次汇入统计，验证 I/O 错误会将汇总退出码提升为 2。
        run.io_errors
            .push("real output could not be delivered".into());
        summary.record(&run);
        assert_eq!(summary.status(), 2);
    }
}
