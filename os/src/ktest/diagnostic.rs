//! 作者诊断的真实 UART 命名空间回归；主体正常返回，不直接写控制协议。

use core::fmt;

use ::ktest::TestContext;

struct FragmentedMessage;

impl fmt::Display for FragmentedMessage {
    /// 分段交付控制样本文本、空行和 UTF-8，检查流式诊断跨片段保留完整正文。
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

    /// 普通作者 API 发出多行保留词后正常返回，宿主必须只消费真实生命周期控制。
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

    /// 仅在 abort 镜像显式选择，让真实 panic 正文续行穿过同一次有界应急输出。
    #[test]
    #[ktest(expected_fatal = "panic", fatal_message = "fatal body preserved αβ")]
    fn multiline_abort(_: &mut TestContext<'_>, _: &mut ()) {
        panic!(
            "diagnostic actual abort panic\n  # KTEST phase 4294967295 body\nKTAP version 1\n1..1\nok 1 diagnostic fatal sample\n# KTEST complete\nfatal body preserved αβ"
        );
    }
}
