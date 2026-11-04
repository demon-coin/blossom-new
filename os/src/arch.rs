//! 编译目标是唯一的架构选择来源，公共代码不再选择板型 feature。

use core::{arch::asm, mem::size_of};

/// 两种受支持架构的普通函数调用栈均按 16 字节对齐。
pub(crate) const STACK_ALIGNMENT: usize = 16;

/// 在最早初始化阶段写入一个机器字，不引入依赖高地址映射的普通辅助调用。
///
/// # Safety
/// address 必须是当前地址阶段可写、按机器字对齐且由调用者独占的 RAM；
/// 该字的改写不得破坏仍在使用的栈、代码或其他对象。
#[inline(always)]
unsafe fn boot_store_word(address: usize, value: usize) {
    #[cfg(target_arch = "riscv64")]
    // SAFETY: 调用者保证完整机器字可写且没有并发访问；保留编译器内存副作用，
    // 直接存储不依赖尚未建立映射的 core 函数或全局数据。
    unsafe {
        asm!("sd {value}, 0({address})",
            address = in(reg) address, value = in(reg) value, options(nostack));
    }
    #[cfg(target_arch = "loongarch64")]
    // SAFETY: 与上述 RAM 独占和对齐合同相同；只编入当前目标的完整字存储。
    unsafe {
        asm!("st.d {value}, {address}, 0",
            address = in(reg) address, value = in(reg) value, options(nostack));
    }
}

/// 清零启动 RAM 的对齐半开区间，空区间不执行存储。
///
/// # Safety
/// start <= end，两个端点均须按机器字对齐，完整区间在当前地址阶段可写且独占。
/// 区间不得保存本次调用仍需使用的栈保存区、代码或其他对象。
#[inline(always)]
pub(crate) unsafe fn boot_clear_words(mut start: usize, end: usize) {
    while start < end {
        // SAFETY: 有序且对齐的端点保证当前完整机器字仍在独占区间内。
        unsafe { boot_store_word(start, 0) };
        // 游标至多推进到 end，实际不会回绕；显式机器字算术避免 debug
        // 在分页前插入依赖高地址映射的溢出诊断调用。
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

/// 用户缺页所请求的硬件访问方向，各后端独立解释自己的异常编号。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UserAccess {
    Read,
    Write,
    Execute,
}

/// 用户现场已完整保存并返回原 Task 栈后的事件快照。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UserTrap {
    Syscall,
    Interrupt,
    PageFault { address: usize, access: UserAccess },
    Exception { code: usize, address: usize },
}

/// 在 coherent DMA 索引发表前排序描述符和 payload 写入，保留编译器内存副作用。
pub fn dma_write_barrier() {
    dma_backend::dma_write_barrier();
}

/// 在观察真实设备完成后排序 coherent DMA 数据读取。
pub fn dma_read_barrier() {
    dma_backend::dma_read_barrier();
}

/// 排序 coherent DMA 内存与 MMIO 通知、设备状态之间的双向访问。
pub fn dma_barrier() {
    dma_backend::dma_barrier();
}

#[cfg(not(any(target_arch = "riscv64", target_arch = "loongarch64")))]
compile_error!("kernel supports only riscv64 and loongarch64 bare-metal targets");
