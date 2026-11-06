//! 测试输出协议观察器（`ProtocolObserver`）的流式解析与结果判定单元测试。
//!
//! # 模块职责与定位
//! 本文件是 `contest-run` 工具箱中 `report::ProtocolObserver` 的核心协议解析回归测试。
//! 上游接收被测子进程产生的 `Stdout` / `Stderr` 字节流分片与最终进程退出状态（`ExitOutcome`），
//! 下游产出统一的测试判定结果（`Verdict`）。本模块覆盖四类竞赛与内核测试协议：
//! 1. `Protocol::Plain`：仅依据进程退出码判定（0 为 `Pass`，非 0 为 `Failed`），忽略输出文本中的评分字样。
//! 2. `Protocol::Basic`：匹配 `========== START test_<name> ==========` 与对应 `END` 标记，同时扫描已知系统调用错误关键字。
//! 3. `Protocol::Pre2025Libctest`：匹配指定 `variant`（`dynamic`/`static`）与 `selector` 的起止标记、`Pass!` 行、`FAIL <selector> [...]` 错误行以及 `runtest.c` 特有的退出码 1。
//! 4. `Protocol::Ltp`：按 Linux Test Project 位掩码（`TFAIL=1`、`TBROK=2`、`TWARN=4`、`TCONF=32`）解析退出码，忽略外层包装脚本输出的提示文本。
//!
//! # 流式解析与状态判定架构
//!
//! ```text
//!   Stdout 字节块 ──► [Stdout 行缓冲] ──┐
//!                                       ├──► 协议状态机 (Basic / Libctest / Plain / Ltp)
//!   Stderr 字节块 ──► [Stderr 行缓冲] ──┘         │ (两流各自独立成行，绝不跨流拼接半行或跨流拼凑起止区间)
//!                                                 ▼
//!                                      observer.finish(&ExitOutcome)
//!                                                 │
//!           ┌─────────────────────────────────────┼─────────────────────────────────────┐
//!           ▼                                     ▼                                     ▼
//!  SpawnError / IoError                  TimedOut / Signaled                   Exited { code, raw }
//!           │                                     │                                     │
//!           ▼                                     ▼                                     ▼
//! Verdict::InfrastructureError      Verdict::TimedOut / Signaled       结合协议标记与退出码产出最终 Verdict
//!                                                                      (Pass / Completed / Failed / NotRun /
//!                                                                       Broken / Warning / Configuration)
//! ```
//!
//! # 并发与锁契约
//! 所有测试均为纯内存流式状态机验证，每个测试用例在栈上创建独立的 `ProtocolObserver` 实例，无共享状态或锁竞争。

use contest_run::model::Protocol;
use contest_run::outcome::{ExitOutcome, Verdict};
use contest_run::report::{ProtocolObserver, StreamKind};

/// 根据给定的退出码 `code` 构造正常的 `ExitOutcome::Exited` 状态，并将原始 `wait` 状态字编码为 `code << 8`。
fn exited(code: i32) -> ExitOutcome {
    ExitOutcome::Exited {
        code,
        raw: code << 8,
    }
}

/// 辅助函数：创建针对 `protocol` 的 `ProtocolObserver`，依次喂入 `stdout` 与 `stderr` 字节切片，并返回结合 `outcome` 后的最终 `Verdict`。
fn observe(protocol: &Protocol, stdout: &[u8], stderr: &[u8], outcome: &ExitOutcome) -> Verdict {
    let mut observer = ProtocolObserver::new(protocol);
    observer.feed(StreamKind::Stdout, stdout);
    observer.feed(StreamKind::Stderr, stderr);
    observer.finish(outcome).verdict
}

/// 辅助函数：构造 `Protocol::Pre2025Libctest` 协议配置，指定链接变体 `variant`（如 `"dynamic"` 或 `"static"`）与子测试选择器 `selector`。
fn libctest(variant: &str, selector: &str) -> Protocol {
    Protocol::Pre2025Libctest {
        variant: variant.into(),
        selector: selector.into(),
    }
}

/// 辅助函数：生成符合 `libctest` `runtest.c` 格式的标准成功输出文本（包含 `START`、`Pass!` 与 `END` 三行）。
fn libc_output(variant: &str, selector: &str) -> String {
    format!("========== START entry-{variant}.exe {selector} ==========\nPass!\n========== END entry-{variant}.exe {selector} ==========\n")
}

/// 辅助函数：生成符合 `basic` 测试套件 `TEST_START` / `TEST_END` 宏格式的标准输出文本（其中测试名 `mkdir_` 映射为标记名 `test_mkdir`）。
fn basic_output(name: &str, body: &str) -> String {
    let marker = if name == "mkdir_" { "mkdir" } else { name };
    format!("========== START test_{marker} ==========\n{body}========== END test_{marker} ==========\n")
}

#[test]
/// 验证 `Protocol::Plain` 仅依据进程退出码判定通过或失败，不受标准输出或标准错误中 `FAIL`、`PASS` 等文本的影响。
///
/// - **测试目标**：确保通用 `Plain` 协议严格遵循进程退出状态码语义。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 输出流包含 `"FAIL LTP CASE x : 0\n"` 与 `"FAIL\n"`，但进程以退出码 0 退出；When 调用 `observe`；Then 判定为 `Verdict::Pass`。
///   2. Given 输出流包含 `"PASS\nPass!\n"`，但进程以非零退出码（1、2、32、127、255）退出；When 调用 `observe`；Then 均判定为 `Verdict::Failed`。
fn plain_keeps_exit_status_semantics() {
    assert_eq!(
        observe(
            &Protocol::Plain,
            b"FAIL LTP CASE x : 0\n",
            b"FAIL\n",
            &exited(0)
        ),
        Verdict::Pass
    );
    for code in [1, 2, 32, 127, 255] {
        assert_eq!(
            observe(&Protocol::Plain, b"PASS\nPass!\n", b"", &exited(code)),
            Verdict::Failed
        );
    }
}

#[test]
/// 验证 `Protocol::Basic` 必须观察到与自身测试名严格匹配的起止标记且退出码为 0 才能判定为 `Verdict::Completed`。
///
/// - **测试目标**：确保 `Basic` 协议不会仅凭退出码 0 或其他测试项的起止标记误判完成，且缺少结束标记或退出码非零时均判为失败。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 协议目标为 `mmap`，输出包含完整的 `test_mmap` 起止行且退出码为 0；When 解析完成；Then 返回 `Verdict::Completed`。
///   2. Given 输出仅为 `"PASS\n"` 或为另一测试 `test_pipe` 的完整起止行；When 解析完成；Then 因未观察到 `test_mmap` 的启动标记而返回 `Verdict::NotRun`。
///   3. Given 输出仅有 `START test_mmap` 而无 `END`、或有完整起止行但退出码为 1、或 `START test_mmap` 搭配了不匹配的 `END test_pipe`；When 解析完成；Then 均返回 `Verdict::Failed`。
fn basic_requires_its_own_markers() {
    let protocol = Protocol::Basic {
        name: "mmap".into(),
    };
    let output = basic_output("mmap", "mmap content: data\n");
    assert_eq!(
        observe(&protocol, output.as_bytes(), b"", &exited(0)),
        Verdict::Completed
    );
    assert_eq!(
        observe(&protocol, b"PASS\n", b"", &exited(0)),
        Verdict::NotRun
    );
    assert_eq!(
        observe(
            &protocol,
            basic_output("pipe", "").as_bytes(),
            b"",
            &exited(0)
        ),
        Verdict::NotRun
    );
    assert_eq!(
        observe(
            &protocol,
            b"========== START test_mmap ==========\n",
            b"",
            &exited(0)
        ),
        Verdict::Failed
    );
    assert_eq!(
        observe(&protocol, output.as_bytes(), b"", &exited(1)),
        Verdict::Failed
    );
    let mismatched =
        b"========== START test_mmap ==========\n========== END test_pipe ==========\n";
    assert_eq!(
        observe(&protocol, mismatched, b"", &exited(0)),
        Verdict::Failed
    );
}

#[test]
/// 验证 `Basic` 协议对 `execve`（映像替换后输出 `END main`）和 `mkdir_`（输出 `test_mkdir`）两个特殊命名规则的精确支持。
///
/// - **测试目标**：确保只有 `execve` 测试允许以 `========== END main ==========` 闭合区间，且 `mkdir_` 配置能正确匹配 `test_mkdir` 标记。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 协议为 `Basic { name: "execve" }`，输出以 `START test_execve` 开始、以 `END main` 结束且退出码为 0；When 解析完成；Then 返回 `Verdict::Completed`。
///   2. Given 协议为 `Basic { name: "pipe" }`，输出却以 `END main` 闭合；When 解析完成；Then 因非 `execve` 用例不允许使用 `END main` 而返回 `Verdict::Failed`。
///   3. Given 协议为 `Basic { name: "mkdir_" }`，输出包含 `test_mkdir` 起止标记且退出码为 0；When 解析完成；Then 返回 `Verdict::Completed`。
fn basic_execve_and_mkdir_use_original_names() {
    let output = b"========== START test_execve ==========\n  I am test_echo.\nexecve success.\n========== END main ==========";
    assert_eq!(
        observe(
            &Protocol::Basic {
                name: "execve".into()
            },
            output,
            b"",
            &exited(0)
        ),
        Verdict::Completed
    );
    let not_execve = b"========== START test_pipe ==========\n========== END main ==========\n";
    assert_eq!(
        observe(
            &Protocol::Basic {
                name: "pipe".into()
            },
            not_execve,
            b"",
            &exited(0)
        ),
        Verdict::Failed
    );
    assert_eq!(
        observe(
            &Protocol::Basic {
                name: "mkdir_".into()
            },
            basic_output("mkdir_", "").as_bytes(),
            b"",
            &exited(0)
        ),
        Verdict::Completed
    );
}

#[test]
/// 验证 `Basic` 协议在正文中检测到已知系统调用错误特征行时，即使存在完整的起止标记、`Pass!` 字样和 0 退出码，依然优先判定为失败。
///
/// - **测试目标**：防止被测程序在系统调用失败后仍打印结束标记并以 0 退出导致漏报错误；同时确认正常返回值（如 `mount return: 0`）及无关文本不会误触发失败。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 20 组 `Basic` 测试名及其对应的错误输出行（如 `mmap error.`、`mount return: -1`、`--- Assert Fatal ! ---`），分别置于 `stdout` 正文或 `stderr` 流中，且伴随 `Pass!` 与退出码 0；
///   2. When 调用 `observe`；Then 全部判定为 `Verdict::Failed`。
///   3. Given `mount` 测试输出 `mount return: 0`、`umount return: 0` 与普通负载行 `FAIL is payload`；When 调用 `observe`；Then 判定为 `Verdict::Completed`。
fn basic_known_errors_take_priority() {
    let cases = [
        ("clone", "clone process error."),
        ("close", "  close error."),
        ("execve", "  execve error."),
        ("exit", "exit ERR."),
        ("getcwd", "getcwd ERROR."),
        ("getppid", "  getppid error."),
        ("gettimeofday", "gettimeofday error."),
        ("mkdir_", "  mkdir error."),
        ("mmap", "mmap error."),
        ("munmap", "mmap error."),
        ("sleep", "sleep error."),
        ("unlink", "  unlink error!"),
        ("wait", "wait child error."),
        ("waitpid", "waitpid error."),
        ("mount", "mount return: -1"),
        ("mount", "umount return: -1"),
        ("umount", "mount return: -1"),
        ("umount", "return: -1"),
        ("munmap", "munmap return: -1"),
        ("write", " --- Assert Fatal ! ---"),
    ];
    for (name, error) in cases {
        let protocol = Protocol::Basic { name: name.into() };
        let output = basic_output(name, &format!("{error}\nPass!\n"));
        assert_eq!(
            observe(&protocol, output.as_bytes(), b"", &exited(0)),
            Verdict::Failed,
            "{name}: {error}"
        );
        assert_eq!(
            observe(
                &protocol,
                basic_output(name, "").as_bytes(),
                format!("{error}\n").as_bytes(),
                &exited(0)
            ),
            Verdict::Failed
        );
    }
    let protocol = Protocol::Basic {
        name: "mount".into(),
    };
    assert_eq!(
        observe(
            &protocol,
            basic_output(
                "mount",
                "mount return: 0\numount return: 0\nFAIL is payload\n"
            )
            .as_bytes(),
            b"",
            &exited(0)
        ),
        Verdict::Completed
    );
}

#[test]
/// 验证 `Pre2025Libctest` 协议要求退出码必须为 1 且同时具备对应变体与选择器的完整成功输出区间才能判定为 `Verdict::Pass`。
///
/// - **测试目标**：适配历史 `libctest` `runtest.c` 的特殊约定（子测试结束后宿主包装返回退出码 1），确保单独的退出码 1 或单独的成功文本均不能误判通过。
/// - **场景步骤（Given-When-Then）**：
///   1. Given `dynamic` 与 `static` 两种变体的 `argv` 用例，当输出包含完整 `START`、`Pass!`、`END` 且退出码为 1 时；When 解析完成；Then 返回 `Verdict::Pass`。
///   2. Given 同样完整的输出，但退出码为 0、2、127 或 255；When 解析完成；Then 全部返回 `Verdict::Failed`。
///   3. Given 退出码为 1，但输出为空或仅有 `Pass!`（无 `START` 标记）；When 解析完成；Then 返回 `Verdict::NotRun`。
///   4. Given 输出仅有 `dynamic` 变体的 `START` 与 `Pass!` 而缺少 `END`；When 解析完成；Then 对 `dynamic` 协议返回 `Verdict::Failed`，对 `static` 协议返回 `Verdict::NotRun`。
fn libctest_return_one_is_not_plain_failure_or_automatic_pass() {
    for variant in ["dynamic", "static"] {
        let protocol = libctest(variant, "argv");
        let output = libc_output(variant, "argv");
        assert_eq!(
            observe(&protocol, output.as_bytes(), b"", &exited(1)),
            Verdict::Pass
        );
        for code in [0, 2, 127, 255] {
            assert_eq!(
                observe(&protocol, output.as_bytes(), b"", &exited(code)),
                Verdict::Failed
            );
        }
        assert_eq!(observe(&protocol, b"", b"", &exited(1)), Verdict::NotRun);
        assert_eq!(
            observe(&protocol, b"Pass!\n", b"", &exited(1)),
            Verdict::NotRun
        );
        assert_eq!(
            observe(
                &protocol,
                b"========== START entry-dynamic.exe argv ==========\nPass!\n",
                b"",
                &exited(1)
            ),
            if variant == "dynamic" {
                Verdict::Failed
            } else {
                Verdict::NotRun
            }
        );
    }
}

#[test]
/// 验证 `Pre2025Libctest` 严格按配置的 `variant` 与 `selector` 隔离匹配区间，其他变体或兄弟用例的成功输出不能替代目标用例。
///
/// - **测试目标**：防止多用例同批输出或变体不一致时发生交叉误判。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 目标为 `dynamic` + `argv`，而输出为 `static` + `argv` 或 `dynamic` + `env` 的完整成功块；When 解析完成；Then 返回 `Verdict::NotRun`。
///   2. Given 输出中 `dynamic` + `argv` 区间内缺少 `Pass!`，紧接着出现了 `dynamic` + `env` 的完整成功块；When 解析完成；Then 目标 `argv` 仍判定为 `Verdict::Failed`。
///   3. Given `START entry-dynamic.exe argv` 搭配了 `END entry-static.exe argv`（变体不匹配的结束行），或起止行之间没有 `Pass!`；When 解析完成；Then 均判定为 `Verdict::Failed`。
fn libctest_scopes_success_to_variant_and_selector() {
    let protocol = libctest("dynamic", "argv");
    for output in [libc_output("static", "argv"), libc_output("dynamic", "env")] {
        assert_eq!(
            observe(&protocol, output.as_bytes(), b"", &exited(1)),
            Verdict::NotRun
        );
    }
    let output = b"========== START entry-dynamic.exe argv ==========\n========== END entry-dynamic.exe argv ==========\n========== START entry-dynamic.exe env ==========\nPass!\n========== END entry-dynamic.exe env ==========\n";
    assert_eq!(observe(&protocol, output, b"", &exited(1)), Verdict::Failed);
    let no_end = b"========== START entry-dynamic.exe argv ==========\nPass!\n========== END entry-static.exe argv ==========\n";
    assert_eq!(observe(&protocol, no_end, b"", &exited(1)), Verdict::Failed);
    let no_pass = b"========== START entry-dynamic.exe argv ==========\n========== END entry-dynamic.exe argv ==========\n";
    assert_eq!(
        observe(&protocol, no_pass, b"", &exited(1)),
        Verdict::Failed
    );
}

#[test]
/// 验证 `Pre2025Libctest` 中针对当前 `selector` 的 `FAIL <selector> [...]` 错误行具有永久一票否决权，且不会误伤前缀相似的其他选择器。
///
/// - **测试目标**：确保无论 `FAIL argv [...]` 出现在成功块之前、之后还是 `stderr` 流中，都会将结果锁定为 `Verdict::Failed`；同时 `FAIL argv_extra` 或 `FAIL env` 不会影响 `argv` 的通过判定。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 完整的 `dynamic` + `argv` 成功输出，并在其前置、后置或 `stderr` 中注入 5 种不同原因的 `FAIL argv [...]` 行；When 解析完成；Then 均返回 `Verdict::Failed`。
///   2. Given `stdout` 为完整的 `argv` 成功输出，`stderr` 为 `FAIL argv_extra [status 1]` 与 `FAIL env [status 1]`；When 解析完成；Then 选择器精确按词边界匹配，返回 `Verdict::Pass`。
fn libctest_failure_is_sticky_and_selector_exact() {
    let protocol = libctest("dynamic", "argv");
    let valid = libc_output("dynamic", "argv");
    for failure in [
        "FAIL argv [internal]\n",
        "FAIL argv [status 1]\n",
        "FAIL argv [timed out]\n",
        "FAIL argv [signal Killed]\n",
        "FAIL argv [unknown]\n",
    ] {
        for output in [format!("{failure}{valid}"), format!("{valid}{failure}")] {
            assert_eq!(
                observe(&protocol, output.as_bytes(), b"", &exited(1)),
                Verdict::Failed
            );
        }
        assert_eq!(
            observe(&protocol, valid.as_bytes(), failure.as_bytes(), &exited(1)),
            Verdict::Failed
        );
    }
    assert_eq!(
        observe(
            &protocol,
            valid.as_bytes(),
            b"FAIL argv_extra [status 1]\nFAIL env [status 1]\n",
            &exited(1)
        ),
        Verdict::Pass
    );
}

#[test]
/// 验证 `ProtocolObserver` 在任意单点切分（0..=len）与任意固定块大小（1..=len）的流式输入下，均能产生与整块输入完全一致的协议判定。
///
/// - **测试目标**：确保内部行缓冲器在处理跨 `feed` 调用被截断的标记行或错误行时没有任何分块边界 bug。
/// - **场景步骤（Given-When-Then）**：
///   - Given 四组典型协议场景（`libctest` 通过、`Basic` 完成、`Basic` 错误失败、`libctest` 带 `FAIL` 行失败）；
///   - When 分别通过 `Stdout` 和 `Stderr` 流，枚举所有二切分点 `split` 以及所有分块长度 `size` 喂入 `ProtocolObserver`；
///   - Then 所有分块方式产出的 `Verdict` 均与预期值严格相等。
fn arbitrary_chunk_boundaries_preserve_markers() {
    let cases = [
        (
            libctest("dynamic", "argv"),
            libc_output("dynamic", "argv"),
            exited(1),
            Verdict::Pass,
        ),
        (
            Protocol::Basic {
                name: "mmap".into(),
            },
            basic_output("mmap", "mmap content: ok\n"),
            exited(0),
            Verdict::Completed,
        ),
        (
            Protocol::Basic {
                name: "mmap".into(),
            },
            basic_output("mmap", "mmap error.\n"),
            exited(0),
            Verdict::Failed,
        ),
        (
            libctest("dynamic", "argv"),
            format!("{}FAIL argv [status 1]\n", libc_output("dynamic", "argv")),
            exited(1),
            Verdict::Failed,
        ),
    ];
    for (protocol, text, outcome, expected) in cases {
        for stream in [StreamKind::Stdout, StreamKind::Stderr] {
            for split in 0..=text.len() {
                let mut observer = ProtocolObserver::new(&protocol);
                observer.feed(stream, &text.as_bytes()[..split]);
                observer.feed(stream, &text.as_bytes()[split..]);
                assert_eq!(observer.finish(&outcome).verdict, expected, "split {split}");
            }
            for size in 1..=text.len() {
                let mut observer = ProtocolObserver::new(&protocol);
                for chunk in text.as_bytes().chunks(size) {
                    observer.feed(stream, chunk);
                }
                assert_eq!(observer.finish(&outcome).verdict, expected, "chunk {size}");
            }
        }
    }
}

#[test]
/// 验证 `Stdout` 与 `Stderr` 拥有各自独立的行缓冲区与协议区间状态，绝不会将跨流的半行文字或跨流的起止标记拼凑成成功结果。
///
/// - **测试目标**：防止并发交错的标准输出与标准错误字节流相互串扰合成伪造的协议标记行。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 将完整的 `libctest` 成功输出从第 20 字节处切开，前半段喂给 `Stdout`，后半段喂给 `Stderr`；When 解析完成；Then 因两流均无完整 `START` 行而返回 `Verdict::NotRun`。
///   2. Given `Stdout` 仅有 `START` 与 `END` 行，而 `Pass!` 被输出在 `Stderr` 中；When 解析完成；Then 返回 `Verdict::Failed`。
///   3. Given `Stdout` 输出 `START` 与 `Pass!`，而 `END` 行输出在 `Stderr` 中；When 解析完成；Then 返回 `Verdict::Failed`。
///   4. Given `Stdout` 输出完整成功块，而 `Stderr` 输出不带末尾换行符的 `"FAIL argv [status 1]"`；When 解析完成；Then `finish` 刷新末尾未换行缓冲并识别该失败行，返回 `Verdict::Failed`。
fn streams_never_synthesize_lines_or_success() {
    let protocol = libctest("dynamic", "argv");
    let text = libc_output("dynamic", "argv");
    let split = 20;
    let mut observer = ProtocolObserver::new(&protocol);
    observer.feed(StreamKind::Stdout, &text.as_bytes()[..split]);
    observer.feed(StreamKind::Stderr, &text.as_bytes()[split..]);
    assert_eq!(observer.finish(&exited(1)).verdict, Verdict::NotRun);
    let framed = b"========== START entry-dynamic.exe argv ==========\n========== END entry-dynamic.exe argv ==========\n";
    assert_eq!(
        observe(&protocol, framed, b"Pass!\n", &exited(1)),
        Verdict::Failed
    );
    let start = b"========== START entry-dynamic.exe argv ==========\nPass!\n";
    let end = b"========== END entry-dynamic.exe argv ==========\n";
    assert_eq!(observe(&protocol, start, end, &exited(1)), Verdict::Failed);
    let valid = libc_output("dynamic", "argv");
    assert_eq!(
        observe(
            &protocol,
            valid.as_bytes(),
            b"FAIL argv [status 1]",
            &exited(1)
        ),
        Verdict::Failed
    );
}

#[test]
/// 验证 `observer.finish()` 会消费末尾不含 `\n` 的残留行，并且对 CRLF（`\r\n`）行尾与 LF（`\n`）行尾具有完全一致的解析语义。
///
/// - **测试目标**：兼容被测程序最后一行未打印换行符或使用 `\r\n` 换行的输出格式。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 去掉末尾 `\n` 的 `libctest` 成功输出，或将全部 `\n` 替换为 `\r\n` 的成功输出；When 解析完成；Then 均判定为 `Verdict::Pass`。
///   2. Given `Stdout` 为完整的 `Basic` `mmap` 起止块，而 `Stderr` 为不带末尾换行符的 `"mmap error."`；When 解析完成；Then 末尾错误行被正确消费，判定为 `Verdict::Failed`。
fn finish_consumes_unterminated_lines() {
    let protocol = libctest("static", "env");
    let output = libc_output("static", "env");
    assert_eq!(
        observe(
            &protocol,
            output.trim_end_matches('\n').as_bytes(),
            b"",
            &exited(1)
        ),
        Verdict::Pass
    );
    assert_eq!(
        observe(
            &protocol,
            output.replace('\n', "\r\n").as_bytes(),
            b"",
            &exited(1)
        ),
        Verdict::Pass
    );
    let basic = Protocol::Basic {
        name: "mmap".into(),
    };
    assert_eq!(
        observe(
            &basic,
            basic_output("mmap", "").as_bytes(),
            b"mmap error.",
            &exited(0)
        ),
        Verdict::Failed
    );
}

#[test]
/// 验证超长无关输出行或超长诊断后缀不会撑爆状态机或掩盖已匹配的失败前缀，也不会将带额外后缀的伪造 `START` 行误认为合法标记。
///
/// - **测试目标**：检验流式行缓冲在面对超长单行（数万字节无换行）时的有界内存行为与前缀匹配精确性。
/// - **场景步骤（Given-When-Then）**：
///   1. Given 先连续喂入 4096 次无换行的无关文本，换行后再喂入合法的 `libctest` 成功块；When 解析完成；Then 判定为 `Verdict::Pass`。
///   2. Given `Stdout` 已完成合法成功块，`Stderr` 先输出 `"FAIL argv "` 前缀再追加 4096 次 `"long diagnostic "` 且无换行；When 解析完成；Then 依然准确识别失败前缀，返回 `Verdict::Failed`。
///   3. Given `Stdout` 输出 `========== START entry-dynamic.exe argv ==========` 后紧跟 4096 个 `'x'` 再换行；When 解析完成；Then 该行因非精确匹配而不触发启动状态，返回 `Verdict::NotRun`。
fn long_output_does_not_hide_failure_or_invent_marker() {
    let protocol = libctest("dynamic", "argv");
    let mut observer = ProtocolObserver::new(&protocol);
    for _ in 0..4096 {
        observer.feed(StreamKind::Stdout, b"unrelated output without a newline");
    }
    observer.feed(StreamKind::Stdout, b"\n");
    observer.feed(
        StreamKind::Stdout,
        libc_output("dynamic", "argv").as_bytes(),
    );
    assert_eq!(observer.finish(&exited(1)).verdict, Verdict::Pass);
    let mut observer = ProtocolObserver::new(&protocol);
    observer.feed(
        StreamKind::Stdout,
        libc_output("dynamic", "argv").as_bytes(),
    );
    observer.feed(StreamKind::Stderr, b"FAIL argv ");
    for _ in 0..4096 {
        observer.feed(StreamKind::Stderr, b"long diagnostic ");
    }
    assert_eq!(observer.finish(&exited(1)).verdict, Verdict::Failed);
    let mut observer = ProtocolObserver::new(&protocol);
    observer.feed(
        StreamKind::Stdout,
        b"========== START entry-dynamic.exe argv ==========",
    );
    for _ in 0..4096 {
        observer.feed(StreamKind::Stdout, b"x");
    }
    observer.feed(
        StreamKind::Stdout,
        b"\nPass!\n========== END entry-dynamic.exe argv ==========\n",
    );
    assert_eq!(observer.finish(&exited(1)).verdict, Verdict::NotRun);
}

#[test]
/// 验证 `Protocol::Ltp` 严格按照 Linux Test Project 退出码位掩码规则推导判定结果，忽略外层脚本输出的 `"FAIL LTP CASE ..."` 固定文本，且不修改传入的 `ExitOutcome`。
///
/// - **测试目标**：确保 LTP 协议准确解码单一状态位、混合 `TCONF(32)` 的组合位以及未知位掩码（归类为 `Broken`）。
/// - **场景步骤（Given-When-Then）**：
///   - Given 15 种典型 LTP 退出码（0、1、2、4、32、各组合位及未定义高位 8/16/64/127/255），且 `Stdout` 包含干扰文字 `"FAIL LTP CASE fork01 : 0\nTPASS: payload\n"`；
///   - When 调用 `observe`；
///   - Then 返回的 `Verdict` 严格遵循 `Broken > Failed > Warning > Configuration > Pass` 的位掩码优先级，且原 `outcome` 结构体的 `code` 与 `raw` 字段保持原样。
fn ltp_preserves_result_bits_and_mixed_configuration() {
    let cases = [
        (0, Verdict::Pass),
        (1, Verdict::Failed),
        (2, Verdict::Broken),
        (4, Verdict::Warning),
        (32, Verdict::Configuration),
        (32 | 1, Verdict::Failed),
        (32 | 2, Verdict::Broken),
        (32 | 4, Verdict::Warning),
        (32 | 1 | 4, Verdict::Failed),
        (32 | 1 | 2 | 4, Verdict::Broken),
        (8, Verdict::Broken),
        (16, Verdict::Broken),
        (64, Verdict::Broken),
        (127, Verdict::Broken),
        (255, Verdict::Broken),
    ];
    for (code, expected) in cases {
        let outcome = exited(code);
        let original = outcome.clone();
        assert_eq!(
            observe(
                &Protocol::Ltp,
                b"FAIL LTP CASE fork01 : 0\nTPASS: payload\n",
                b"",
                &outcome
            ),
            expected
        );
        assert_eq!(outcome, original);
        assert_eq!(
            outcome,
            ExitOutcome::Exited {
                code,
                raw: code << 8
            }
        );
    }
}

#[test]
/// 验证当进程处于启动失败（`SpawnError`）、信号终止（`Signaled`）、执行超时（`TimedOut`）或底层 I/O 错误（`IoError`）时，任何协议的成功输出文本都不能覆盖基础设施层面的异常结论。
///
/// - **测试目标**：确保进程级非正常终止与 I/O 故障始终高于输出流中的协议文本判定。
/// - **场景步骤（Given-When-Then）**：
///   - Given 6 种非正常 `ExitOutcome`（`SpawnError`、两种 `Signaled`、两种 `TimedOut`、`IoError`）与全部 4 种协议（`Plain`、`Ltp`、`Basic`、`Pre2025Libctest`）；
///   - When 同时喂入包含完整 `Basic` 与 `Libctest` 成功标记的 `Stdout` 及 `"Pass!\n"` 的 `Stderr`；
///   - Then 所有组合均返回对应的异常判定（`InfrastructureError`、`Signaled`、`TimedOut`），不会被正常输出文本覆盖为通过。
fn abnormal_outcomes_keep_infrastructure_boundaries() {
    let cases = [
        (
            ExitOutcome::SpawnError {
                errno: Some(2),
                message: "No such file or directory".into(),
            },
            Verdict::InfrastructureError,
        ),
        (
            ExitOutcome::Signaled { signal: 9, raw: 9 },
            Verdict::Signaled,
        ),
        (
            ExitOutcome::Signaled {
                signal: 11,
                raw: 139,
            },
            Verdict::Signaled,
        ),
        (
            ExitOutcome::TimedOut {
                raw: None,
                signal: None,
            },
            Verdict::TimedOut,
        ),
        (
            ExitOutcome::TimedOut {
                raw: Some(9),
                signal: Some(9),
            },
            Verdict::TimedOut,
        ),
        (
            ExitOutcome::IoError {
                errno: Some(5),
                message: "Input/output error".into(),
                raw: Some(0),
            },
            Verdict::InfrastructureError,
        ),
    ];
    let protocols = [
        Protocol::Plain,
        Protocol::Ltp,
        Protocol::Basic {
            name: "mmap".into(),
        },
        libctest("dynamic", "argv"),
    ];
    for protocol in protocols {
        for (outcome, expected) in &cases {
            let original = outcome.clone();
            let text = format!(
                "{}{}",
                basic_output("mmap", ""),
                libc_output("dynamic", "argv")
            );
            assert_eq!(
                observe(&protocol, text.as_bytes(), b"Pass!\n", outcome),
                *expected
            );
            assert_eq!(*outcome, original);
        }
    }
}
