//! `kselftest` TAP（Test Anything Protocol）输出协议解析与超时控制的集成测试。
//!
//! # 模块职责与定位
//! 本文件是 `contest-run` 工具箱针对 Linux `kselftest.h` 标准头文件的回归测试套件。
//! 上游通过宿主 C 编译器（`cc`）将包含真实 `kselftest.h` 的样本程序编译为可执行文件，
//! 下游交由 `process::run_command`、`report::ProtocolObserver` 以及 `user-selftests` 命令行入口执行与解析，
//! 验证各类 TAP 计划行、跳过（SKIP）、预期失败（XFAIL）、意外通过（XPASS）、紧急中止（Bail out!）、任意字节切分流式输入以及程序绝对超时限制的判定语义。
//!
//! # TAP 协议状态机与判定规则
//!
//! ```text
//!                        ┌──────────────────────────────────────────────┐
//!                        │  kselftest.h C 测试程序 / 原始字节流输入      │
//!                        └──────────────────────┬───────────────────────┘
//!                                               │ (仅解析 Stdout；Stderr 仅作诊断输出，不参与协议匹配)
//!                                               ▼
//!                        ┌──────────────────────────────────────────────┐
//!                        │         ProtocolObserver (Protocol::Tap)      │
//!                        │  - 跟踪 TAP version 13 头与 1..N 计划行       │
//!                        │  - 校验序号递增、无重复、无缺项、无二次计划   │
//!                        │  - 识别 # SKIP / # XFAIL / # XPASS / Bail out│
//!                        └──────────────────────┬───────────────────────┘
//!                                               │ 结合进程 ExitOutcome
//!                 ┌─────────────────────────────┼─────────────────────────────┐
//!                 ▼                             ▼                             ▼
//!      进程超时 / 信号终止             存在 not ok / XPASS /          计划完整且全部 ok / XFAIL
//!      (TimedOut / Signaled)           Bail out! / 退出码不匹配        或全部标记为 # SKIP
//!                 │                    或序号缺失/重复 (Broken)               │
//!                 ▼                             │                             ▼
//!       Verdict::TimedOut /                     ▼                   Verdict::Pass (退出码 0)
//!       Verdict::Signaled            Verdict::Failed / Broken       Verdict::Skip (退出码 0 或 4)
//! ```
//!
//! # 并发与锁契约
//! 每个测试使用带当前进程 ID 的独立临时目录（`Scratch`）存放编译产物与输出捕获文件，不共享全局可变状态。

use contest_run::model::{CommandSpec, Protocol};
use contest_run::outcome::{ExitOutcome, Verdict};
use contest_run::process::run_command;
use contest_run::report::{ProtocolObserver, StreamKind};
use std::collections::BTreeMap;
use std::fs::{self, File};
use std::os::fd::AsRawFd;
use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

/// 内联的 C 语言测试样本源码，直接引用仓库中的 `kselftest.h` 头文件以生成与真实内核自测完全一致的 TAP 输出与退出码。
const SAMPLE: &str = r#"
#include "kselftest.h"
#include <time.h>
int main(int argc, char **argv)
{
    if (argc != 1 && argc != 2) return 1;
    const char *mode = argc == 2 ? argv[1] : strrchr(argv[0], '/');
    if (argc == 1) mode = mode ? mode + 1 : argv[0];
    ksft_print_header();
    if (!strcmp(mode, "zero-skip")) ksft_exit_skip("no support\n");
    if (!strcmp(mode, "unplanned-skip")) {
        ksft_test_result_pass("first\n");
        ksft_exit_skip("remaining\n");
    }
    if (!strcmp(mode, "plan-skip") || !strcmp(mode, "partial-skip")) {
        ksft_set_plan(3);
        if (!strcmp(mode, "partial-skip")) ksft_test_result_pass("first\n");
        ksft_exit_skip("remaining\n");
    }
    ksft_set_plan(1);
    if (!strcmp(mode, "slow") || !strcmp(mode, "noisy")) {
        struct timespec tick = { .tv_nsec = 20000000 };
        for (int i = 0; i < 100; ++i) {
            if (!strcmp(mode, "noisy")) ksft_print_msg("progress %d\n", i);
            if (nanosleep(&tick, NULL)) ksft_exit_fail_msg("nanosleep failed\n");
        }
        ksft_test_result_pass("completed within program budget\n");
    } else if (!strcmp(mode, "pass")) ksft_test_result_pass("ordinary\n");
    else if (!strcmp(mode, "all-skip")) ksft_test_result_skip("no support\n");
    else if (!strcmp(mode, "fail-zero")) {
        ksft_test_result_fail("failed\n");
        ksft_exit_pass();
    } else if (!strcmp(mode, "fail-skip")) {
        ksft_test_result_fail("failed\n");
        ksft_exit_skip("remaining\n");
    } else if (!strcmp(mode, "xfail")) ksft_test_result_xfail("expected\n");
    else if (!strcmp(mode, "xfail-exit")) {
        ksft_test_result_xfail("expected\n");
        ksft_exit_xfail();
    } else if (!strcmp(mode, "xpass")) {
        ksft_test_result_code(KSFT_XPASS, "unexpected", NULL);
        ksft_exit_pass();
    } else ksft_exit_fail_msg("aborted\n");
    ksft_finished();
}
"#;

/// 验证基于 `kselftest.h` 编译的测试程序在不同执行模式下的 TAP 判定结果、流式分块一致性以及程序总超时预算。
///
/// - **测试目标**：
///   1. 确保 `ProtocolObserver` 对 `kselftest.h` 产生的各类真实输出（通过、全跳过、0 计划跳过、部分跳过、失败后仍调 `ksft_exit_pass`、`XFAIL`、`XPASS`、中止退出）及对应退出码（0、1、2、4）给出准确的 `Verdict`。
///   2. 确保无论标准输出按何种块大小（1 字节至全长）流式喂入 `ProtocolObserver`，最终解析结论完全一致。
///   3. 确保 `user-selftests --program-timeout-seconds` 对慢速程序（`slow`）及持续打印日志的慢速程序（`noisy`）实施严格的墙钟总超时限制（持续输出不能重置超时计时器）。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 在临时目录中使用 `cc` 编译 `SAMPLE` 源码生成可执行文件；
///   2. When 依次传入 12 种测试模式参数调用 `run_command`，并对捕获到的标准输出按任意字节块长重放给 `ProtocolObserver`；Then 进程退出码、协议判定结果及所有分块重放结果均与预期一致。
///   3. Given 将可执行文件硬链接为 `timing/slow` 与 `timing/noisy` 并写入清单文件；When 分别以 1 秒和 5 秒超时预算调用 `user-selftests`；Then 预算不足（1 秒）时均因超时返回退出码 1，预算充足（5 秒）时正常完成并返回退出码 0。
#[test]
fn fixed_header_outputs_keep_skip_and_failure_exit_semantics() {
    // Step 1: 创建临时目录并调用宿主 C 编译器编译内联的 kselftest 样本程序。
    let directory = std::env::temp_dir().join(format!("selftest-tap-{}", std::process::id()));
    fs::create_dir(&directory).unwrap();
    let scratch = Scratch(directory);
    let source = scratch.0.join("sample.c");
    let executable = scratch.0.join("sample");
    fs::write(&source, SAMPLE).unwrap();
    let include = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/user/include");
    let compiled = Command::new("cc")
        .arg("-std=gnu11")
        .arg("-I")
        .arg(include)
        .arg(&source)
        .arg("-o")
        .arg(&executable)
        .output()
        .unwrap();
    assert!(
        compiled.status.success(),
        "{}",
        String::from_utf8_lossy(&compiled.stderr)
    );
    // Step 2: 遍历 12 种典型 kselftest 运行模式，校验进程退出状态、TAP 协议判定以及任意分块大小下的流式解析一致性。
    for (mode, code, expected) in [
        ("pass", 0, Verdict::Pass),
        ("all-skip", 0, Verdict::Skip),
        ("zero-skip", 4, Verdict::Skip),
        ("plan-skip", 4, Verdict::Skip),
        ("partial-skip", 4, Verdict::Skip),
        ("unplanned-skip", 4, Verdict::Skip),
        ("fail-zero", 0, Verdict::Failed),
        ("fail-skip", 4, Verdict::Failed),
        ("xfail", 0, Verdict::Pass),
        ("xfail-exit", 2, Verdict::Failed),
        ("xpass", 0, Verdict::Failed),
        ("bail", 1, Verdict::Failed),
    ] {
        let stdout = File::create(scratch.0.join(format!("{mode}.stdout"))).unwrap();
        let stderr = File::create(scratch.0.join(format!("{mode}.stderr"))).unwrap();
        let spec = CommandSpec::Exec {
            program: executable.to_str().unwrap().into(),
            args: vec![mode.into()],
        };
        let run = run_command(
            None,
            &scratch.0,
            &spec,
            &BTreeMap::new(),
            Duration::from_secs(3),
            &Protocol::Tap,
            [stdout.as_raw_fd(), stderr.as_raw_fd()],
        );
        assert_eq!(
            run.outcome,
            ExitOutcome::Exited {
                code,
                raw: code << 8
            },
            "{mode}"
        );
        assert!(run.io_errors.is_empty(), "{mode}: {:?}", run.io_errors);
        assert_eq!(run.protocol.verdict, expected, "{mode}: {:?}", run.protocol);
        // 将捕获到的完整 stdout 按 1..=len 的所有块长切分重放，验证流式行缓冲无边界漏洞。
        let bytes = fs::read(scratch.0.join(format!("{mode}.stdout"))).unwrap();
        for size in 1..=bytes.len() {
            let mut observer = ProtocolObserver::new(&Protocol::Tap);
            for chunk in bytes.chunks(size) {
                observer.feed(StreamKind::Stdout, chunk);
            }
            assert_eq!(
                observer.finish(&run.outcome),
                run.protocol,
                "{mode}: chunk {size}"
            );
        }
    }
    // Step 3: 构造 timing 集合清单，通过 user-selftests 二进制验证慢速静默程序与持续输出程序均受总超时预算约束。
    let collection = scratch.0.join("timing");
    fs::create_dir(&collection).unwrap();
    for mode in ["slow", "noisy"] {
        fs::hard_link(&executable, collection.join(mode)).unwrap();
    }
    let manifest = scratch.0.join("kselftest-list.txt");
    fs::write(&manifest, "timing:slow\ntiming:noisy\n").unwrap();
    for (program, seconds, expected) in [
        ("timing:slow", "1", 1),
        ("timing:slow", "5", 0),
        ("timing:noisy", "1", 1),
    ] {
        let run = Command::new(env!("CARGO_BIN_EXE_user-selftests"))
            .args([
                "--manifest",
                manifest.to_str().unwrap(),
                "--program",
                program,
                "--program-timeout-seconds",
                seconds,
            ])
            .output()
            .unwrap();
        assert_eq!(
            run.status.code(),
            Some(expected),
            "{program} budget={seconds}: stdout={} stderr={}",
            String::from_utf8_lossy(&run.stdout),
            String::from_utf8_lossy(&run.stderr)
        );
    }
}

/// 测试专用的临时目录守卫（RAII Fixture）。
///
/// 负责在测试结束或 panic 栈展开时自动清理编译生成的 C 源码、可执行文件与捕获输出（元组字段 `0` 为临时目录路径）。
struct Scratch(PathBuf);

impl Drop for Scratch {
    /// 测试退出时递归删除临时目录及其全部测试产物。
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

/// 验证 TAP 协议要求标准输出具备完整且合法的计划与测试点序列，且标准错误输出或异常终止状态不会误导判定。
///
/// - **测试目标**：确保缺失测试项、重复序号、越界序号、重复计划行、`Bail out!` 中止、跨流拼接、进程超时/被信号杀死等异常情况均被准确识别，同时忽略 `stderr` 上的同名干扰文本。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 进程以退出码 0 正常退出，但 `stdout` 存在未跑满计划数（`1..2` 仅 1 项）、重复序号（两个 `ok 1`）、序号跳号（`1..1` 却输出 `ok 2`）、重复计划行或中途出现 `Bail out!`；When 完成解析；Then 判定结果均为非成功状态。
///   2. Given 一行 `ok 1 success` 被拆分到 `stdout` 与 `stderr` 两个流中；When 完成解析；Then 两个流不会跨流拼合行，因缺少合法测试项而被判定为 `Verdict::Broken`。
///   3. Given `stdout` 输出了完整的 `1..1\nok 1 success\n`，但进程退出状态为 `TimedOut` 或 `Signaled`；When 完成解析；Then 最终判定保留为 `Verdict::TimedOut` 或 `Verdict::Signaled`。
///   4. Given `stdout` 输出完整合法的通过序列，而 `stderr` 包含 `not ok 1` 与 `Bail out!` 诊断文本；When 完成解析；Then `stderr` 文本不影响 `stdout` 的 TAP 协议状态，最终判定仍为 `Verdict::Pass`。
#[test]
fn tap_completion_requires_one_intact_stdout_protocol() {
    // Step 1: 验证各类结构残缺或违背 TAP 约束的 stdout 序列即便配合退出码 0 也不能判定为成功。
    let exited = ExitOutcome::Exited { code: 0, raw: 0 };
    for bytes in [
        b"1..2\nok 1 first\n".as_slice(),
        b"1..1\nok 1 first\nok 1 duplicate\n",
        b"1..1\nok 2 # SKIP terminal\n",
        b"1..1\nok 1 first\n1..1\n",
        b"1..1\nBail out! failed\nok 1 last\n",
    ] {
        let mut observer = ProtocolObserver::new(&Protocol::Tap);
        observer.feed(StreamKind::Stdout, bytes);
        assert!(!observer.finish(&exited).verdict.is_success());
    }
    // Step 2: 验证 stdout 与 stderr 的半行数据绝不会跨流拼接成一条合法的 ok 记录。
    let mut observer = ProtocolObserver::new(&Protocol::Tap);
    observer.feed(StreamKind::Stdout, b"1..1\nok ");
    observer.feed(StreamKind::Stderr, b"1 success\n");
    assert_eq!(observer.finish(&exited).verdict, Verdict::Broken);

    // Step 3: 验证即使 stdout 输出了完整的通过文本，进程超时或信号终止依然优先决定最终 Verdict。
    for (outcome, verdict) in [
        (
            ExitOutcome::TimedOut {
                raw: None,
                signal: None,
            },
            Verdict::TimedOut,
        ),
        (
            ExitOutcome::Signaled { signal: 9, raw: 9 },
            Verdict::Signaled,
        ),
    ] {
        let mut observer = ProtocolObserver::new(&Protocol::Tap);
        observer.feed(StreamKind::Stdout, b"1..1\nok 1 success\n");
        assert_eq!(observer.finish(&outcome).verdict, verdict);
    }
    // Step 4: 验证 stderr 中的 "not ok" 或 "Bail out!" 仅被视作普通诊断输出，不会污染合法的 stdout TAP 结果。
    let mut observer = ProtocolObserver::new(&Protocol::Tap);
    observer.feed(StreamKind::Stdout, b"1..1\nok 1 success\n");
    observer.feed(
        StreamKind::Stderr,
        b"not ok 1 diagnostic only\nBail out! diagnostic only\n",
    );
    assert_eq!(observer.finish(&exited).verdict, Verdict::Pass);
}
