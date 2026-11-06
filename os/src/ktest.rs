//! 内核内置测试框架（`ktest`）顶层模块。
//!
//! # 模块职责与架构定位
//! 本模块将独立的 `::ktest` 协议原语桥接到内核运行时环境，向上游的启动流程（`early`、`run_boot`、
//! `spawn_thread`）和全局异常/故障入口（panic 处理器、OOM 分配失败、架构陷阱）提供统一的测试生命周期管理，
//! 向下游依赖串口控制台（`crate::console`）、任务调度器（`crate::task`）以及栈展开器（`unwinding`）。
//!
//! # 核心架构与执行流
//! ```text
//! +-----------------------------------------------------------------------+
//! |                      宿主测试执行器 (Host Runner)                      |
//! +-----------------------------------------------------------------------+
//!             | UART 控制指令                            ^ KTAP / KTEST 帧
//!             v                                          |
//! +-----------------------------------------------------------------------+
//! |  runner / admission / registry : 命令解析、用例筛选与套件生命周期驱动  |
//! +-----------------------------------------------------------------------+
//!             |                                          |
//!    +--------+---------+                       +--------+---------+
//!    | Boot 执行上下文  |                       | Thread 执行上下文|
//!    | (BSP 单核、无堆) |                       | (Task 线程、SMP) |
//!    +--------+---------+                       +--------+---------+
//!             \                                          /
//!              v                                        v
//! +-----------------------------------------------------------------------+
//! | context : 测试作用域绑定 (BootGuard / ArcScope)、子任务派生与协同回收  |
//! | panic   : 栈上固定容量 DWARF 展开捕获边界 (Boundary) 与浮点状态恢复   |
//! | report  : 生产级单串口协议输出、多行诊断转义与致命故障终止上报        |
//! +-----------------------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 控制协议与诊断输出统一复用内核唯一的生产级串口控制台，避免引入测试专用 I/O 通道。
//! - `fatal_event` 允许在任意执行阶段（包括 Per-CPU 指针尚未安装的早期启动阶段、关中断临界区或异常处理器）调用；
//!   在 Per-CPU 就绪前严禁读取当前 CPU 或 `Task` 测试槽位，直接走无锁应急串口输出并停机。

mod admission;
#[cfg(target_arch = "loongarch64")]
mod ale;
mod boot_cases;
mod context;
mod diagnostic;
mod execution_core;
pub(crate) mod execution_services;
mod fatal_return;
mod network;
pub(crate) mod panic;
mod registers;
mod registry;
pub(crate) mod report;
mod runner;
pub(crate) mod selftest;
mod user_execution;

pub(crate) use crate::console::{
    early_write as control_write, emergency_write, ktest_read_byte as control_read_byte,
};
pub(crate) use context::{
    active_case, child_failed, current_boundary, fatal, prepare_result, prepare_result_on,
    replace_boundary, spawn_on, spawn_result, spawn_result_on, ChildHandle, ChildToken,
    PreparedChild,
};
pub(crate) use registers::selftest_register_roundtrip;
pub(crate) use runner::{early, requested_context, run_boot, spawn_thread, RunCommand};

/// 上报不可恢复的致命测试事件并终止系统运行。
///
/// # 调用上下文
/// 可在内核任意阶段调用（包括 Per-CPU 尚未初始化的早期启动阶段、普通任务上下文或异常/中断处理上下文）。
///
/// # 参数
/// - `kind`：致命事件类别标识（例如 `"panic"`、`"double-panic"`、`"abi"`、`"oom"`）。
/// - `message`：描述故障原因的格式化消息。
pub(crate) fn fatal_event(kind: &'static str, message: core::fmt::Arguments<'_>) -> ! {
    // Step 1: 若当前 CPU 的 Per-CPU 结构体尚未安装，无法安全读取当前 Task 或测试上下文槽位，
    // 直接通过底层应急串口输出早期致命帧并关机。
    if crate::arch::cpu_local().is_null() {
        emergency_write(format_args!("KTEST early fatal {}: {}\n", kind, message));
        crate::arch::emergency_shutdown();
    }
    // Step 2: Per-CPU 已就绪时，委托给报告子模块关联当前活跃用例 ID 并输出标准协议帧。
    report::fatal_event(kind, message)
}
