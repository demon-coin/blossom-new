//! 两组 LIOINTC 共享原 IRQ domain；源号按 bank×32 规范化，路由固定到实际 BSP。

use crate::arch::loongarch64::register::{CPU_HWI0, CPU_HWI1};
use crate::arch::loongarch64::{barrier, mmio_address, EXTERNAL_CONTROLLER};
use crate::boot::{BootInfo, LioIntcResources};

const LIOINTC_SOURCES_PER_BANK: u32 = u32::BITS;
const BANK_COUNT: u32 = 2;
const SOURCE_COUNT: u32 = BANK_COUNT * LIOINTC_SOURCES_PER_BANK;
const BOARD_CPU_COUNT: usize = 2;
const ROUTE_PARENT_SHIFT: u32 = 4;
const ENABLE_STATUS: usize = 0x24;
const ENABLE: usize = 0x28;
const DISABLE: usize = 0x2c;
const POLARITY: usize = 0x30;
const EDGE: usize = 0x34;

/// 从已验证的 main 窗口读取对齐寄存器，不建立设备内存引用。
fn read(resource: LioIntcResources, offset: usize) -> u32 {
    assert!(
        offset + core::mem::size_of::<u32>() <= resource.registers.end - resource.registers.start
    );
    // SAFETY: BootInfo 已验证窗口与四字节对齐，DMW 保留实际设备映射。
    unsafe { ((mmio_address(resource.registers.start) + offset) as *const u32).read_volatile() }
}

/// 写 enable/disable 等硬件寄存器；调用者持唯一控制器短锁。
fn write(resource: LioIntcResources, offset: usize, value: u32) {
    assert!(
        offset + core::mem::size_of::<u32>() <= resource.registers.end - resource.registers.start
    );
    // SAFETY: main 窗口已验证且永久映射，offset 是硬件定义的对齐寄存器。
    unsafe {
        ((mmio_address(resource.registers.start) + offset) as *mut u32).write_volatile(value)
    };
}

/// BSP 在 AP 释放前屏蔽全部源并发布 level-high 与固定物理核路由。
pub(super) fn initialize(info: &BootInfo) {
    assert_eq!(info.interrupts.source_count, SOURCE_COUNT);
    assert!(info.bsp_hardware_id < BOARD_CPU_COUNT);
    let _controller = EXTERNAL_CONTROLLER.lock();
    for resource in info.board.liointc {
        assert!((CPU_HWI0..=CPU_HWI1).contains(&resource.parent_interrupt));
        write(resource, DISABLE, u32::MAX);
        write(resource, POLARITY, 0);
        write(resource, EDGE, 0);
        for source in 0..LIOINTC_SOURCES_PER_BANK as usize {
            let route = (1u8 << (resource.parent_interrupt - CPU_HWI0 + ROUTE_PARENT_SHIFT))
                | (1u8 << info.bsp_hardware_id);
            // SAFETY: 32 个独立字节路由完整位于 main 的前 32 字节，初始化尚无并发访问。
            unsafe {
                ((mmio_address(resource.registers.start) + source) as *mut u8)
                    .write_volatile(route);
            }
        }
    }
    barrier();
}

/// 按规范化 source 定位实际 bank 与单个位；不修改相邻设备。
pub(super) fn set_enabled(source: u32, enabled: bool) {
    assert!(source < SOURCE_COUNT);
    let _controller = EXTERNAL_CONTROLLER.lock();
    let resource =
        crate::boot::info().board.liointc[source as usize / LIOINTC_SOURCES_PER_BANK as usize];
    write(
        resource,
        if enabled { ENABLE } else { DISABLE },
        1u32 << (source % LIOINTC_SOURCES_PER_BANK),
    );
    barrier();
}

/// 只读取本核 ISR 与真实 enable 状态，保留另一组尚未处理的 level 源。
pub(super) fn claim() -> Option<u32> {
    let info = crate::boot::info();
    let hardware_id = crate::cpu::current().hardware_id;
    assert!(hardware_id < BOARD_CPU_COUNT);
    let _controller = EXTERNAL_CONTROLLER.lock();
    for (bank, resource) in info.board.liointc.iter().copied().enumerate() {
        let region = resource.isr[hardware_id];
        // SAFETY: 解析边界验证本核 ISR 四字节窗口，永久 DMW 映射且读取无写副作用。
        let pending = unsafe { (mmio_address(region.start) as *const u32).read_volatile() }
            & read(resource, ENABLE_STATUS);
        if pending != 0 {
            return Some(bank as u32 * LIOINTC_SOURCES_PER_BANK + pending.trailing_zeros());
        }
    }
    None
}

/// 设备 ISR 已应答且源保持屏蔽时完成设备访问排序；level 源没有独立 W1C 完成寄存器。
pub(super) fn complete(source: u32) {
    assert!(source < SOURCE_COUNT);
    barrier();
}
