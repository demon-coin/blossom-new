//! 内核测试钩子设备（`/dev/testhook`）的用户态写入接口。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `contest-run` 执行层与内核 `/dev/testhook` 字符设备之间。当测例开启 `hooks` 开关时，
//! 上游执行引擎（`crate::execute::execute_case`）在主命令启动前调用 `hook(true, id)`，在主命令结束后调用
//! `hook(false, id)`，通知内核执行针对特定测例的前置准备或后置清理/追踪操作（例如 `runner:trace`）。
//!
//! ## 消息协议与写入契约
//! ```text
//! +-----------------------+-----+--------------------+------+
//! | "before" 或 "after"   | ' ' | 测例 ID (无空白/0) | '\n' |  (总长度 <= 512 字节)
//! +-----------------------+-----+--------------------+------+
//!             │
//!             ▼  单次 write() 系统调用提交到 /dev/testhook (O_WRONLY | O_CLOEXEC)
//! +---------------------------------------------------------+
//! | written == message.len()  ──> Ok(())                    |
//! | written != message.len()  ──> Err(WriteZero) 拒绝重试   |
//! +---------------------------------------------------------+
//! ```
//! 内核钩子操作可能具有非幂等的副作用，因此必须通过单次 `write` 系统调用完整提交；一旦出现短写（Short Write）
//! 或中断错误，直接向调用方返回错误并中止后续测例，避免重复发送导致内核状态错乱。
//!
//! ## 并发与锁契约
//! 每次调用独立打开设备文件描述符并写入单条消息，不维护跨调用的用户态全局状态或锁。

use std::fs::OpenOptions;
use std::io::{self, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;

/// 向指定的内核测试钩子字符设备写入单条前置（`before`）或后置（`after`）控制消息。
///
/// 调用上下文：在测例主进程启动前或退出后由用户态主线程同步调用。
/// 参数 `path` 为目标钩子设备路径，`before` 为 `true` 表示前置钩子、为 `false` 表示后置钩子，`id` 为不含空白与 NUL 的测例标识符。
/// 返回值：单次完整写入成功返回 `Ok(())`；若 `id` 非法、消息超过 512 字节、打开设备失败或发生部分写入，均返回 `io::Error` 且不会自动重发。
pub fn hook_device(path: &Path, before: bool, id: &str) -> io::Result<()> {
    // Step 1: 校验测例 ID 非空且不包含 NUL 字节或 ASCII 空白字符，防止破坏单行空格分隔协议。
    if id.is_empty()
        || id
            .bytes()
            .any(|byte| byte == 0 || byte.is_ascii_whitespace())
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "invalid hook case ID",
        ));
    }
    // Step 2: 组装单行控制消息并检查 512 字节长度上限。
    let operation = if before { "before" } else { "after" };
    let message = format!("{operation} {id}\n");
    if 512 < message.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "hook message exceeds 512 bytes",
        ));
    }
    // Step 3: 以只写和 O_CLOEXEC 标志打开设备并执行单次写入；若字节数不完整则报错退出，避免非幂等钩子被重复执行。
    let mut device = OpenOptions::new()
        .write(true)
        .custom_flags(libc::O_CLOEXEC)
        .open(path)?;
    let written = device.write(message.as_bytes())?;
    if written != message.len() {
        return Err(io::Error::new(
            io::ErrorKind::WriteZero,
            format!(
                "hook short write: {written}/{}; message not replayed",
                message.len()
            ),
        ));
    }
    Ok(())
}

/// 向系统默认的 `/dev/testhook` 内核测试钩子设备发送前置或后置控制消息。
///
/// 调用上下文：供 `contest-run` 主程序在执行启用了 `hooks` 的测例时作为默认钩子回调传入。
/// 参数与返回值语义同 [`hook_device`]。
pub fn hook(before: bool, id: &str) -> io::Result<()> {
    hook_device(Path::new("/dev/testhook"), before, id)
}
