//! 测试专用入口与捕获桥；控制协议只使用生产唯一 UART。

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

/// 实际错误控制者传入真实故障类型；CPU 尚未安装时不能读取测试 Task 槽。
pub(crate) fn fatal_event(kind: &'static str, message: core::fmt::Arguments<'_>) -> ! {
    if crate::arch::cpu_local().is_null() {
        emergency_write(format_args!("KTEST early fatal {}: {}\n", kind, message));
        crate::arch::emergency_shutdown();
    }
    report::fatal_event(kind, message)
}
