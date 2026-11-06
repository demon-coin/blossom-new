//! 用户态内置自测集（Selftests）独立命令行入口。
//!
//! ## 模块职责与架构定位
//! 本模块编译为 `/bin/user-selftests` 可执行程序，供交互式 Shell、自动化脚本或系统启动流程直接调用。
//! 其上游接收命令行参数，下游委托 `contest_run::selftests` 模块完成已安装自测清单（Manifest）的解析、
//! 筛选与子进程顺序执行。与比赛主入口 `contest-run` 不同，本入口不挂载外部比赛磁盘，执行结束后直接将
//! 退出状态码返回给父进程，不触发系统关机。
//!
//! ## 退出状态码契约
//! ```text
//! CLI 参数解析 ──┬─> 失败 (非 UTF-8 / 非法选项) ──────────────────> stderr 打印错误，exit(2)
//!                └─> 成功
//!                     │
//!                     ▼
//!            selftests::run(...) ──┬─> 清单读取 / I/O 错误 ───────> stderr 打印错误，exit(2)
//!                                  ├─> 全部选中程序通过 / 仅列出 ──> exit(0)
//!                                  └─> 存在失败或超时的测试程序 ──> exit(1)
//! ```
//!
//! ## 并发与锁契约
//! 本入口在单线程顺序上下文中运行，直接借用进程的标准输出（FD 1）和标准错误（FD 2）供子进程继承，不涉及多线程锁。

use contest_run::selftests::{self, Options};
use std::io;

/// 解析命令行参数并运行已安装的用户态自测清单。
///
/// 调用上下文：作为独立用户态进程的主入口运行。
/// 返回行为：通过 `std::process::exit` 直接终止进程；测试全部通过返回 `0`，存在测例失败返回 `1`，参数或运行期基础设施错误统一返回 `2`。
fn main() {
    // Step 1: 收集并校验命令行参数为 UTF-8 字符串，解析选项后交由 selftests::run 执行（子进程输出直接写向 FD 1 和 FD 2）。
    let result = std::env::args_os()
        .skip(1)
        .map(|arg| {
            arg.into_string()
                .map_err(|_| "CLI requires UTF-8 arguments".to_string())
        })
        .collect::<Result<Vec<_>, _>>()
        .and_then(|args| Options::parse(&args))
        .and_then(|options| selftests::run(&options, &mut io::stdout(), [1, 2]));
    // Step 2: 汇总执行结果；若解析或清单加载出错则向标准错误输出诊断信息并映射为退出码 2。
    let status = match result {
        Ok(status) => status,
        Err(error) => {
            let _ = std::io::Write::write_fmt(
                &mut io::stderr(),
                format_args!("SELFTEST error: {error}\n"),
            );
            2
        }
    };
    std::process::exit(status);
}
