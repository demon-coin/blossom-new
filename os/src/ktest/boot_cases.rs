//! 启动期与底层硬件自检测试集（`boot_cases`）的顶层模块汇总与链接锚点入口。
//!
//! # 模块职责与架构定位
//! 本模块组织了内核启动早期（`Boot` 上下文）及多核启动后（`Thread` 上下文）针对固件信息、
//! 早期地址映射、CPU 位图、多核在线状态、栈布局、浮点/条件标志寄存器、跨核同步调用（IPI Call）、
//! 串口控制台以及致命陷阱恢复的自检子模块。
//! - **上游调用者**：`crate::ktest` 测试框架注册表与链接保活机制。
//! - **下游子模块**：`firmware`、`cpu_set`、`hardware`、`calls`、`uart`、`fatal`。
//!
//! # 核心机制与测试套件分层
//! ```text
//! boot_cases
//!  ├─ Boot 上下文（早期自检，不依赖动态页分配器 MM 或任务调度器 Task）
//!  │   ├─ firmware  : 固件传入的启动参数、FDT/内存区间合法性检查
//!  │   ├─ cpu_set   : CpuSet / AtomicCpuSet 位运算与字节编解码
//!  │   ├─ hardware  : 早期内核虚拟地址映射、每 CPU 结构、栈边界、FPU/FCC 寄存器
//!  │   ├─ uart      : 早期串口输出状态与格式化锁行为
//!  │   └─ fatal     : 启动期异常陷入与致命错误路径测试
//!  └─ Thread 上下文（多核调度就绪后运行）
//!      └─ calls     : 跨核同步函数调用（cpu::call）与 IPI 队列并发压力测试
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本文件仅聚合各子套件的静态注册槽引用，不持有任何运行时状态或锁。

mod calls;
mod cpu_set;
mod fatal;
mod firmware;
mod hardware;
mod uart;

/// 返回所有启动自检测试套件的静态注册槽引用数组，防止链接器在编译期将未直接引用的测试节（Section）裁剪丢弃。
pub(crate) fn anchors(
) -> [&'static ::ktest::SuiteSlot; 10 + 2 * cfg!(target_arch = "riscv64") as usize] {
    [
        firmware::cases::__ktest_anchor(),
        #[cfg(target_arch = "riscv64")]
        firmware::no_map::__ktest_anchor(),
        cpu_set::cases::__ktest_anchor(),
        hardware::mapping::__ktest_anchor(),
        #[cfg(target_arch = "riscv64")]
        hardware::rv_mapping::__ktest_anchor(),
        hardware::cpus::__ktest_anchor(),
        hardware::stacks::cases::__ktest_anchor(),
        #[cfg(target_arch = "riscv64")]
        hardware::rv_fp::__ktest_anchor(),
        #[cfg(target_arch = "loongarch64")]
        hardware::fcc::cases::__ktest_anchor(),
        calls::cases::__ktest_anchor(),
        uart::cases::__ktest_anchor(),
        fatal::traps::__ktest_anchor(),
        fatal::startup::__ktest_anchor(),
    ]
}
