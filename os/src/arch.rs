//! 架构抽象公共门面与早期启动内存辅助。
//!
//! # 模块职责与架构定位
//! 本模块作为内核跨架构（`riscv64` 与 `loongarch64`）的统一入口：
//! - 上游调用者：启动入口（`boot`）、中断与陷入分发（`irq`/`task`）、内存管理（`mm`）、驱动（DMA 屏障）以及每核状态管理（`cpu`）。
//! - 下游依赖：根据编译期 `target_arch` 条件编译，将具体的特权级指令、页表格式、中断控制器与上下文切换重导出自 `riscv64` 或 `loongarch64` 子模块。
//!
//! # 核心设计与用户态陷入事件模型
//! 1. **早期无栈字清零（`boot_clear_words`）**：
//!    在分页或完整高地址映射尚未建立的最早启动阶段，不能依赖编译器可能生成的 `memset` 调用或带溢出检查的算术辅助函数。本模块直接用内联汇编逐机器字（64 位）清零 `.bss` 等早期 RAM 区间。
//! 2. **用户态陷入分类（`UserTrap`）**：
//!    各架构底层汇编保存完用户态寄存器并切回内核任务栈后，会将硬件异常码归一化为四类事件交给通用任务层处理：
//!    ```text
//!    用户态运行 (U-mode / PLV3)
//!       │ 硬件陷入 (ecall / syscall / 外部中断 / 缺页 / 非法指令等)
//!       ▼
//!    架构汇编入口保存 TrapFrame 并切换至内核栈
//!       │
//!       ▼
//!    归一化为 UserTrap 枚举:
//!      ├─ Syscall                      -> 系统调用分发
//!      ├─ Interrupt                    -> 硬中断处理与抢占检查
//!      ├─ PageFault { address, access }-> VMA 缺页处理 (Read / Write / Execute)
//!      └─ Exception { code, address }  -> 致命异常信号或终止任务
//!    ```
//! 3. **一致性 DMA 内存屏障**：
//!    为网卡、块设备等驱动提供跨架构的读写屏障封装（`dma_write_barrier`、`dma_read_barrier`、`dma_barrier`），确保 CPU 对描述符环与数据缓冲区的读写顺序对外部 DMA 设备可见。

use core::{arch::asm, mem::size_of};

/// 受支持架构（RISC-V 64 与 LoongArch 64）C ABI 要求的常规函数调用栈字节对齐粒度（16 字节）。
pub(crate) const STACK_ALIGNMENT: usize = 16;

/// 在早期启动阶段向指定地址直接写入一个机器字（`usize`），避免生成依赖高地址映射的函数调用或栈操作。
///
/// # Safety
/// 1. `address` 必须在当前地址翻译阶段可写、按 `usize`（8 字节）对齐，且由当前执行的 CPU 独占访问。
/// 2. 写入该机器字不得覆盖当前正在使用的栈帧、内核代码段或其他活跃对象。
#[inline(always)]
unsafe fn boot_store_word(address: usize, value: usize) {
    #[cfg(target_arch = "riscv64")]
    // SAFETY:
    // 1. 调用方契约保证 `address` 按 8 字节对齐且在当前地址阶段可写、无并发访问。
    // 2. 使用 `options(nostack)` 直接发射单条 `sd` 指令，避免接触栈空间或调用尚未建立高地址映射的外部符号。
    unsafe {
        asm!("sd {value}, 0({address})",
            address = in(reg) address, value = in(reg) value, options(nostack));
    }
    #[cfg(target_arch = "loongarch64")]
    // SAFETY:
    // 1. 调用方契约保证 `address` 按 8 字节对齐且在当前地址阶段可写、无并发访问。
    // 2. 使用 `options(nostack)` 直接发射单条 `st.d` 指令，避免接触栈空间或调用尚未建立高地址映射的外部符号。
    unsafe {
        asm!("st.d {value}, {address}, 0",
            address = in(reg) address, value = in(reg) value, options(nostack));
    }
}

/// 按机器字（`usize`）步长清零启动期 RAM 的半开区间 `[start, end)`；若 `start == end` 则不执行任何写入。
///
/// # Safety
/// 1. 必须满足 `start <= end`，且 `start` 与 `end` 均按 `size_of::<usize>()`（8 字节）对齐。
/// 2. 整个 `[start, end)` 区间在当前地址翻译阶段必须可写且由调用方独占。
/// 3. 该区间不得与当前正在执行的栈帧、代码段或任何活跃数据重叠。
#[inline(always)]
pub(crate) unsafe fn boot_clear_words(mut start: usize, end: usize) {
    while start < end {
        // SAFETY: 循环条件 `start < end` 与两端点按机器字对齐的前提保证了当前 `[start, start + 8)` 完整位于调用方独占的可写区间内。
        unsafe { boot_store_word(start, 0) };
        // 游标按机器字步长递增直至等于 `end`，不会发生越界回绕；
        // 使用 `wrapping_add` 防止在 Debug 构建下插入依赖高地址映射的整数溢出 panic 处理函数。
        start = start.wrapping_add(size_of::<usize>());
    }
}

#[cfg(target_arch = "riscv64")]
mod riscv64;
#[cfg(target_arch = "riscv64")]
pub use riscv64::*;

#[cfg(target_arch = "loongarch64")]
mod loongarch64;
#[cfg(target_arch = "loongarch64")]
pub use loongarch64::*;

#[cfg(target_arch = "loongarch64")]
use loongarch64 as dma_backend;
#[cfg(target_arch = "riscv64")]
use riscv64 as dma_backend;

/// 用户态触发缺页异常（Page Fault）时硬件报告的访问方向。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UserAccess {
    /// 用户态读取内存触发的缺页（Load Page Fault）。
    Read,
    /// 用户态写入内存触发的缺页（Store/AMO Page Fault）。
    Write,
    /// 用户态取指执行触发的缺页（Instruction/Fetch Page Fault）。
    Execute,
}

/// 用户态陷入内核并完整保存寄存器现场后，向任务主循环报告的归一化事件。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UserTrap {
    /// 系统调用请求（RISC-V `ecall` 或 LoongArch `syscall`）。
    Syscall,
    /// 硬件中断（定时器中断、软件 IPI 或外部设备中断）。
    Interrupt,
    /// 用户空间虚拟地址缺页或权限异常（`address` 为触发缺页的故障虚拟字节地址，`access` 为触发缺页时的读/写/执行访问类型）。
    PageFault { address: usize, access: UserAccess },
    /// 无法通过缺页恢复的其他同步硬件异常（`code` 为架构原生异常编码如 RISC-V `scause` 或 LoongArch `Estat.Ecode`，`address` 为故障地址或指令字节地址）。
    Exception { code: usize, address: usize },
}

/// 一致性 DMA 写屏障：在向设备发布描述符环索引或触发 MMIO 门铃前，确保此前对描述符与数据缓冲区的写入全部完成并对设备可见。
pub fn dma_write_barrier() {
    dma_backend::dma_write_barrier();
}

/// 一致性 DMA 读屏障：在观察到设备完成标志或中断后，确保后续对 DMA 接收缓冲区数据的读取排在状态读取之后。
pub fn dma_read_barrier() {
    dma_backend::dma_read_barrier();
}

/// 一致性 DMA 全屏障：对 DMA 内存读写与外设 MMIO 寄存器访问施加双向顺序约束。
pub fn dma_barrier() {
    dma_backend::dma_barrier();
}

#[cfg(not(any(target_arch = "riscv64", target_arch = "loongarch64")))]
compile_error!("kernel supports only riscv64 and loongarch64 bare-metal targets");
