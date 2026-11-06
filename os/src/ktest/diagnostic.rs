//! 测试诊断输出与串口控制协议的命名空间隔离自测套件。
//!
//! # 模块职责与架构定位
//! 本模块验证当测试用例通过 `TestContext::diagnostic` 或多行 `panic!` 消息输出包含 `KTEST` / `KTAP`
//! 协议保留关键字、跨 `write_str` 片段拆分的伪造控制行、空行以及多字节 UTF-8 字符时，`ktest::report`
//! 层能否正确为每一行诊断文本加上 `# ` 注释前缀转义，从而确保宿主解析器只消费真正的测试生命周期控制帧。
//!
//! # 核心机制与转义流
//! ```text
//! 测试作者调用 context.diagnostic(...) 或触发多行 panic!(...)
//!        |
//!        | 分段 write_str: "KTE" + "ST phase ...\n\nKTAP version 1\n...α" + "β\n"
//!        v
//! +-----------------------------------------------------------------------+
//! | ktest::report 流式行缓冲与前缀转义器                                   |
//! |  - 跨 fmt::Formatter::write_str 片段拼接完整行                        |
//! |  - 每行行首强制冠以 "  # " 诊断前缀，隔离 "KTEST " / "KTAP " 保留词   |
//! +-----------------------------------------------------------------------+
//!        |
//!        v
//! 串口输出（宿主 Runner 将带有 `#` 前缀的行识别为普通诊断文本而非协议指令）
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本自测套件运行于 `Boot` 上下文，单核顺序执行，仅通过 `TestContext::diagnostic` 与 `panic!`
//!   触发 `report` 层的串口输出锁，不涉及额外同步原语。

use core::fmt;

use ::ktest::TestContext;

/// 跨多次 `write_str` 调用分段输出协议保留词与多字节 UTF-8 字符的格式化器夹具。
struct FragmentedMessage;

impl fmt::Display for FragmentedMessage {
    /// 将形似 `KTEST` 控制帧、`KTAP` 结果行、空行以及双字节 UTF-8 字符（`αβ`）拆分为多个片段写入，
    /// 用于检验流式诊断输出器在任意片段边界下均能保持行完整性与前缀转义。
    fn fmt(&self, output: &mut fmt::Formatter<'_>) -> fmt::Result {
        output.write_str("KTE")?;
        output.write_str("ST phase ")?;
        write!(output, "{}", u32::MAX)?;
        output.write_str(" body\n")?;
        output.write_str("\n")?;
        output.write_str("KTEST complete\nKTEST status ")?;
        write!(output, "{}", u32::MAX)?;
        output.write_str(" Pass\n")?;
        output.write_str(
            "KTAP version 1\n1..1\nok 1 diagnostic sample\nnot ok 1 diagnostic sample\n",
        )?;
        output.write_str("  # KTEST phase 4294967295 body\n")?;
        output.write_str("fragment-boundary: α")?;
        output.write_str("β\n")?;
        output.write_str("KTEST")?;
        output.write_str(" READY 1 arch=riscv64 cpus=1 mode=debug panic=unwind\n")
    }
}

#[::ktest::suite(context = "Boot", fixture = (), tags("selftest", "selftest-normal", "diagnostic"))]
mod boot {
    use super::*;

    /// 验证 `TestContext::diagnostic` 输出包含协议保留词的多行分段文本时不会干扰宿主状态机。
    ///
    /// - **测试目标**：确认用户态或内核态测试代码通过 `context.diagnostic` 打印伪造的 `KTEST` / `KTAP`
    ///   控制行、空字符串及前导换行符时，报告层能将其安全转义为注释行。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：处于 `Boot` 上下文的普通测试用例；
    ///   2. When：依次调用 `context.diagnostic` 输出 [`FragmentedMessage`]、空串 `""` 以及带前导换行的多行字符串；
    ///   3. Then：测试主体正常返回 `Pass`，宿主不会将诊断正文中的 `KTEST complete` 或 `not ok` 误判为协议控制指令。
    #[test]
    fn author_namespace(context: &mut TestContext<'_>, _: &mut ()) {
        context.diagnostic(format_args!("{}", FragmentedMessage));
        context.diagnostic(format_args!(""));
        context.diagnostic(format_args!(
            "\n{}\n{}",
            "newline-boundary", "diagnostic body returned"
        ));
    }
}

#[::ktest::suite(
    context = "Boot",
    fixture = (),
    isolation = "Case",
    tags("selftest", "selftest-fatal", "diagnostic"),
    ignore = "destructive abort-only diagnostic selftest; select explicitly"
)]
mod fatal {
    use super::*;

    /// 验证在 `panic = "abort"` 模式下触发含协议保留词的多行 `panic!` 时，致命错误报告仍能完整保留并转义正文。
    ///
    /// - **测试目标**：确认应急致命输出路径（abort panic）在打印多行 panic 消息时，既不会因内嵌的
    ///   `KTAP` / `KTEST` 文本破坏协议帧结构，也不会丢失跨行末尾的多字节 UTF-8 消息。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：显式选中该破坏性用例，且框架在进入主体时已武装 `expected_fatal = "panic"`；
    ///   2. When：调用 `panic!` 抛出包含换行符、伪造 `KTEST` / `KTAP` 行及末尾 `"fatal body preserved αβ"` 的消息；
    ///   3. Then：宿主捕获预期致命 panic，且完整匹配尾部的 `fatal_message`。
    #[test]
    #[ktest(expected_fatal = "panic", fatal_message = "fatal body preserved αβ")]
    fn multiline_abort(_: &mut TestContext<'_>, _: &mut ()) {
        panic!(
            "diagnostic actual abort panic\n  # KTEST phase 4294967295 body\nKTAP version 1\n1..1\nok 1 diagnostic fatal sample\n# KTEST complete\nfatal body preserved αβ"
        );
    }
}
