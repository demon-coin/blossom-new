//! 各核 `rdtime` 补偿寄存器 `CNTC` 的统一零点。

use crate::arch::loongarch64::register;

use super::{csr_read, csr_write, read_counter};
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// BSP 建立的统一计数器零点；各核在首次采样前继承同一补偿。
static COUNTER_EPOCH: AtomicU64 = AtomicU64::new(0);
static COUNTER_EPOCH_READY: AtomicBool = AtomicBool::new(false);

/// 读取计数器补偿寄存器；`rdtime` 的读数等于原始计数加上该补偿。
fn counter_compensation() -> u64 {
    csr_read::<{ register::CSR_CNTC }>() as u64
}

/// 写入本核计数器补偿，使补偿后的读数与统一零点对齐。
///
/// # Safety
/// 仅 PLV0 启动段可调用，且该核不得已有时间采样依赖旧补偿。
unsafe fn write_counter_compensation(value: u64) {
    // SAFETY: 调用者保证 PLV0 且本核尚无依赖旧补偿的时间采样。
    unsafe { csr_write::<{ register::CSR_CNTC }>(value as usize) };
}

/// BSP 在开放 AP 与时间服务前建立统一计数器零点。
pub fn establish_counter_epoch() {
    assert!(
        !COUNTER_EPOCH_READY.load(Ordering::Acquire),
        "counter epoch is already established"
    );
    let raw = read_counter().wrapping_sub(counter_compensation());
    let offset = 0u64.wrapping_sub(raw);
    // SAFETY: BSP 启动段处于 PLV0、IRQ 关闭，尚无本核时间采样。
    unsafe { write_counter_compensation(offset) };
    COUNTER_EPOCH.store(offset, Ordering::Release);
    COUNTER_EPOCH_READY.store(true, Ordering::Release);
}

/// 在本核首次时间采样前写入 BSP 建立的统一补偿。
///
/// # Safety
/// 仅启动段在 PLV0、本核尚无时间采样时调用。
pub unsafe fn apply_counter_epoch() {
    assert!(
        COUNTER_EPOCH_READY.load(Ordering::Acquire),
        "counter epoch is not established"
    );
    let offset = COUNTER_EPOCH.load(Ordering::Acquire);
    // SAFETY: 调用者保证本核尚未依赖旧补偿。
    unsafe { write_counter_compensation(offset) };
}

/// 读取最近一次异常保存的指令字；仅用于同步异常的现场诊断。
pub fn bad_instruction() -> usize {
    csr_read::<{ register::CSR_BADI }>()
}
