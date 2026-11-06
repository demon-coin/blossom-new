//! LoongArch64 稳定计数器补偿寄存器（`CNTC`）零点对齐模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 架构层，负责在系统引导阶段将所有 CPU 核心的硬件定时器读数对齐到统一的零点基准。
//! - **上游调用者**：内核主引导流程（BSP 启动阶段调用 [`establish_counter_epoch`]，AP 启动阶段在进入时间子系统前调用 [`apply_counter_epoch`]）。
//! - **下游依赖**：通过 `super::{csr_read, csr_write, read_counter}` 访问 LoongArch64 架构的 `CNTC` 控制状态寄存器（CSR 0x42）和 `rdtime.d` 指令。
//!
//! ## 核心设计与计数补偿原理
//! LoongArch64 处理器内部包含一个随恒定频率递增的 64 位硬件原始计数器（Raw Counter）。
//! 当软件执行 `rdtime.d` 指令读取当前计数值时，硬件并非直接返回原始计数值，而是加上本核 `CSR_CNTC` 补偿寄存器中的有符号/无符号 64 位偏移量：
//!
//! ```text
//! +-----------------------+      +-----------------------+      +-----------------------+
//! |  64-bit Raw Counter   | ---> |  + CSR_CNTC (Offset)  | ---> |  rdtime.d Read Value  |
//! +-----------------------+      +-----------------------+      +-----------------------+
//!
//! 对齐计算流程（以 wraps mod 2^64 运算）：
//!   1. raw    = rdtime.d - old_CNTC            (还原出硬件原始计数值)
//!   2. offset = 0 - raw = (!raw).wrapping_add(1) (计算将当前时刻归零所需的补偿值)
//!   3. new_CNTC = offset                       (写入后此刻的 rdtime.d 读数从 0 开始递增)
//! ```
//!
//! 为避免启动前固件或模拟器在 `CNTC` 中残留不同初值，或者由于原始计数器从上电起已累积较大数值导致多核间基准不一致，
//! 主核（BSP）在启动早期计算出将当前原始计数值抵消为 `0` 所需的全局补偿量 `offset`，写入自身 `CSR_CNTC` 并发布到全局变量 [`COUNTER_EPOCH`]；
//! 随后启动的从核（AP）在进行任何时间采样前读取 [`COUNTER_EPOCH`] 并写入各自的 `CSR_CNTC`，从而保证全系统所有核心共享同一个单调递增的时间零点。
//!
//! ## 同步与并发契约
//! - 本模块不涉及自旋锁或睡眠锁，仅通过原子变量 [`COUNTER_EPOCH`] 和 [`COUNTER_EPOCH_READY`] 的 Release/Acquire 内存序完成主从核之间的单次发布-订阅同步。
//! - [`establish_counter_epoch`] 只能由主核在释放从核之前调用一次；[`apply_counter_epoch`] 必须在从核首次调用 `read_counter` 或初始化本地定时器之前完成。

use crate::arch::loongarch64::register;

use super::{csr_read, csr_write, read_counter};
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// 主核（BSP）计算并发布的全局计数器补偿值（单位：硬件计数器 tick），供所有从核（AP）在启动时复制到各自的 `CSR_CNTC`。
static COUNTER_EPOCH: AtomicU64 = AtomicU64::new(0);
/// 标识全局计数器零点补偿值 [`COUNTER_EPOCH`] 是否已由主核计算完毕并完成 Release 发布。
static COUNTER_EPOCH_READY: AtomicBool = AtomicBool::new(false);

/// 读取当前 CPU 核心的计数器补偿寄存器 `CSR_CNTC`（CSR 0x42）。
///
/// 硬件执行 `rdtime.d` 指令时的返回值等于底层原始计数值加上该寄存器的值（按 64 位无符号环绕加法计算）。
///
/// - **调用上下文**：可在内核特权级（PLV0）任意上下文调用。
fn counter_compensation() -> u64 {
    csr_read::<{ register::CSR_CNTC }>() as u64
}

/// 将指定的补偿值写入当前 CPU 核心的 `CSR_CNTC` 寄存器，使本核后续 `rdtime.d` 读数对齐到统一的时间零点。
///
/// - **调用上下文**：仅限内核启动早期（PLV0、本地中断关闭）调用。
/// - **参数 `value`**：待写入 `CSR_CNTC` 的 64 位补偿偏移量（单位：硬件计数器 tick）。
///
/// # Safety
/// 1. 调用时当前 CPU 必须处于最高特权级（PLV0），否则执行 `csrwr` 指令会触发特权指令异常。
/// 2. 当前 CPU 核心在此调用前不得有任何时间子系统或调度器的时间戳采样依赖旧的 `CSR_CNTC` 补偿值，否则会导致单调时钟读数发生回退或跳变。
unsafe fn write_counter_compensation(value: u64) {
    // SAFETY:
    // 1. 调用方契约保证了当前执行流处于 PLV0 特权级，且 CSR_CNTC (0x42) 是合法可写的本核定时器补偿寄存器。
    // 2. 调用方契约保证了本核尚未产生依赖旧补偿值的时间戳采样，修改 CNTC 不会破坏单调时间不变量。
    unsafe { csr_write::<{ register::CSR_CNTC }>(value as usize) };
}

/// 在主核（BSP）启动阶段建立全系统统一的硬件计数器零点，并将补偿值发布给后续启动的从核（AP）。
///
/// - **调用上下文**：由主核在本地中断关闭、从核尚未唤醒、且全局时间服务尚未初始化的单线程启动阶段调用一次。
/// - **副作用**：改写主核自身的 `CSR_CNTC` 寄存器，并更新 [`COUNTER_EPOCH`] 与 [`COUNTER_EPOCH_READY`] 全局原子状态。
pub fn establish_counter_epoch() {
    assert!(
        !COUNTER_EPOCH_READY.load(Ordering::Acquire),
        "counter epoch is already established"
    );

    // Step 1: 从当前 rdtime 读数中减去旧的 CNTC 补偿值，还原出底层硬件原始计数值 raw，
    // 再通过模 2^64 取反计算出使当前读数归零的新补偿量 offset。
    let raw = read_counter().wrapping_sub(counter_compensation());
    let offset = 0u64.wrapping_sub(raw);
    // SAFETY:
    // 1. 主核启动早期运行于 PLV0 特权级且本地中断已关闭。
    // 2. 上方 assert! 确认零点尚未建立，此时主核还未启动时间子系统，没有任何时间戳采样依赖旧的 CNTC 值。
    unsafe { write_counter_compensation(offset) };

    // Step 2: 先写入全局补偿值，再以 Release 内存序将就绪标志置为 true，确保从核 Acquire 读取就绪标志后一定能看到正确的 offset。
    COUNTER_EPOCH.store(offset, Ordering::Release);
    COUNTER_EPOCH_READY.store(true, Ordering::Release);
}

/// 在从核（AP）启动阶段将主核已建立的全局计数器补偿值写入本核 `CSR_CNTC` 寄存器。
///
/// - **调用上下文**：在从核完成基础 CSR 初始化后、首次读取时间戳或启动本地定时器之前（PLV0、本地中断关闭）调用。
///
/// # Safety
/// 1. 调用时当前从核必须处于 PLV0 特权级。
/// 2. 必须在本核进行任何时间采样或定时器编程之前调用，以防本核观察到未对齐的硬件计数值。
pub unsafe fn apply_counter_epoch() {
    assert!(
        COUNTER_EPOCH_READY.load(Ordering::Acquire),
        "counter epoch is not established"
    );

    // Step 1: 在上方的 Acquire 断言确认主核已发布全局零点后，读取全局补偿值并写入本核 CSR_CNTC。
    let offset = COUNTER_EPOCH.load(Ordering::Acquire);
    // SAFETY:
    // 1. 调用方契约保证了当前从核运行于 PLV0 特权级。
    // 2. 调用方契约保证了本核尚无任何组件采样过时间戳，此时写入与主核一致的 offset 不会破坏时间单调性。
    unsafe { write_counter_compensation(offset) };
}
