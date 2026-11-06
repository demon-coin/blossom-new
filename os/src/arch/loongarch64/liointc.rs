//! 龙芯 2K1000LA（`Ls2k1000la`）传统 I/O 中断控制器（LIOINTC）驱动模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 架构层，负责在龙芯 2K1000LA 真实开发板平台上管理两组 32 位传统 I/O 中断控制器（LIOINTC Bank 0 与 Bank 1），将其统一抽象为包含 64 个中断源（`source` 0..64）的扁平中断域。
//! - **上游调用者**：同级父模块 `arch::loongarch64` 中的外部中断分发接口（`external_init`、`external_mask`、`external_unmask`、`external_claim`、`external_complete`）。
//! - **下游依赖**：通过 `arch::loongarch64::mmio_address` 将设备物理地址转换为非缓存强序直接映射窗口（`DMW0`）虚拟地址，通过 `barrier()` 执行 `dbar 0` 设备访存屏障，并读取 `boot::BootInfo` 中由设备树解析出的 `LioIntcResources` 硬件资源描述。
//!
//! ## 核心设计与寄存器布局
//! 龙芯 2K1000LA 板载两组独立的 LIOINTC 控制器（Bank 0 管辖规范化中断号 `0..32`，Bank 1 管辖 `32..64`）。每组控制器包含一个主控制寄存器窗口（`registers`）以及每个物理 CPU 核心独立的中断状态寄存器窗口（`isr[core_id]`）：
//!
//! ```text
//! 规范化中断号 (source: 0..64) 映射关系：
//!   bank = source / 32 (0 或 1)
//!   bit  = source % 32 (0..32)
//!
//! 每组 LIOINTC 主寄存器窗口 (resource.registers.start + offset)：
//! +------------------+-----------------------------------------------------------+
//! | 偏移量 (Offset)  | 寄存器名称与功能说明                                      |
//! +------------------+-----------------------------------------------------------+
//! | 0x00 .. 0x1F     | ENTRY[0..31] (32 个单字节路由寄存器，每个源占 1 字节)      |
//! |                  |   - bit[7:4]: 路由到的 CPU 硬件中断引脚 (1 << (HWI - 2))  |
//! |                  |   - bit[3:0]: 路由到的目标物理核位图 (1 << core_id)       |
//! | 0x24             | ENABLE_STATUS (32-bit, 当前已使能的中断源位图)            |
//! | 0x28             | ENABLE        (32-bit, 写 1 使能对应位的中断源，W1S)      |
//! | 0x2C             | DISABLE       (32-bit, 写 1 屏蔽对应位的中断源，W1C)      |
//! | 0x30             | POLARITY      (32-bit, 触发极性：0 表示高电平/上升沿有效) |
//! | 0x34             | EDGE          (32-bit, 触发方式：0 表示电平触发)          |
//! +------------------+-----------------------------------------------------------+
//!
//! 每核中断状态窗口 (resource.isr[hardware_id].start)：
//! +------------------+-----------------------------------------------------------+
//! | 0x00             | CORE_ISR      (32-bit, 只读，当前物理核待处理的中断位图)  |
//! +------------------+-----------------------------------------------------------+
//! ```
//!
//! ### 电平触发中断处理协议
//! LIOINTC 的电平触发（Level-High）中断源没有独立的写 1 清除（W1C）完成寄存器：
//! 1. 中断到来时，[`claim`] 读取当前物理核的 `CORE_ISR` 并与 `ENABLE_STATUS` 按位与，返回第一个待处理且已使能的规范化中断号。
//! 2. 上层中断框架随即调用 [`set_enabled`]`(source, false)` 向 `DISABLE` 寄存器写入对应位以屏蔽该中断源，随后调用具体外设驱动应答硬件中断（拉低中断线）。
//! 3. 处理完毕后，上层调用 [`complete`] 执行内存屏障确保外设应答操作落地，最后重新调用 [`set_enabled`]`(source, true)` 开放该中断源。
//!
//! ## 锁获取顺序与并发契约
//! - 本模块所有寄存器读写均由父模块定义的全局自旋锁 `EXTERNAL_CONTROLLER`（`SpinLock<()>`）串行化保护。
//! - **锁顺序契约**：上层中断子系统若需同时持有特定中断源的描述符锁（Descriptor Lock）与控制器锁，必须遵循 **先获取上层中断描述符锁，再获取 `EXTERNAL_CONTROLLER` 自旋锁** 的顺序；`EXTERNAL_CONTROLLER` 持锁期间严禁调用任何可能睡眠或反向获取上层锁的函数。

use crate::arch::loongarch64::register::{CPU_HWI0, CPU_HWI1};
use crate::arch::loongarch64::{barrier, mmio_address, EXTERNAL_CONTROLLER};
use crate::boot::{BootInfo, LioIntcResources};

/// 每组 LIOINTC 控制器管理的中断源数量（32 个，对应 32 位寄存器的每一位）。
const LIOINTC_SOURCES_PER_BANK: u32 = u32::BITS;
/// 板载 LIOINTC 控制器的组数（Bank 0 与 Bank 1 共 2 组）。
const BANK_COUNT: u32 = 2;
/// 系统支持的规范化外部中断源总数（2 × 32 = 64 个）。
const SOURCE_COUNT: u32 = BANK_COUNT * LIOINTC_SOURCES_PER_BANK;
/// 龙芯 2K1000LA 开发板的物理 CPU 核心数量上限（双核：物理核 ID 为 0 和 1）。
const BOARD_CPU_COUNT: usize = 2;
/// 单字节路由寄存器（`ENTRY[i]`）中父级 CPU 硬件中断引脚掩码的高半字节左移位数（bit[7:4]）。
const ROUTE_PARENT_SHIFT: u32 = 4;

/// 主寄存器窗口内“中断使能状态寄存器”（`ENABLE_STATUS`）的字节偏移量。
const ENABLE_STATUS: usize = 0x24;
/// 主寄存器窗口内“写 1 使能寄存器”（`ENABLE`）的字节偏移量。
const ENABLE: usize = 0x28;
/// 主寄存器窗口内“写 1 屏蔽寄存器”（`DISABLE`）的字节偏移量。
const DISABLE: usize = 0x2c;
/// 主寄存器窗口内“中断极性配置寄存器”（`POLARITY`）的字节偏移量。
const POLARITY: usize = 0x30;
/// 主寄存器窗口内“边沿/电平触发方式寄存器”（`EDGE`）的字节偏移量。
const EDGE: usize = 0x34;

/// 从指定 LIOINTC 控制器的主寄存器窗口读取一个 32 位寄存器。
///
/// - **调用上下文**：调用方应已持有 `EXTERNAL_CONTROLLER` 自旋锁或在单核启动阶段调用。
/// - **参数 `resource`**：目标 LIOINTC Bank 的硬件资源区间描述。
/// - **参数 `offset`**：相对于主寄存器窗口起始地址的字节偏移量（必须 4 字节对齐且不越界）。
/// - **返回值**：读取到的 32 位无符号寄存器值。
fn read(resource: LioIntcResources, offset: usize) -> u32 {
    assert!(
        offset + core::mem::size_of::<u32>() <= resource.registers.end - resource.registers.start
    );
    // SAFETY:
    // 1. 上方 assert! 已检查 offset + 4 不超过 resource.registers 的窗口长度，且设备树解析阶段已验证起始地址按 4 字节对齐。
    // 2. mmio_address 将物理地址映射到永久有效的非缓存强序 DMW0 窗口，只读寄存器访问不会产生未定义行为。
    unsafe { ((mmio_address(resource.registers.start) + offset) as *const u32).read_volatile() }
}

/// 向指定 LIOINTC 控制器的主寄存器窗口写入一个 32 位寄存器（如 `ENABLE`、`DISABLE`、`POLARITY`、`EDGE`）。
///
/// - **调用上下文**：调用方必须已持有 `EXTERNAL_CONTROLLER` 自旋锁，以串行化对控制器寄存器的访问。
/// - **参数 `resource`**：目标 LIOINTC Bank 的硬件资源区间描述。
/// - **参数 `offset`**：相对于主寄存器窗口起始地址的字节偏移量（必须 4 字节对齐且不越界）。
/// - **参数 `value`**：待写入的 32 位寄存器值。
fn write(resource: LioIntcResources, offset: usize, value: u32) {
    assert!(
        offset + core::mem::size_of::<u32>() <= resource.registers.end - resource.registers.start
    );
    // SAFETY:
    // 1. 上方 assert! 保证了写入范围完整落在已验证且按 4 字节对齐的 resource.registers MMIO 窗口内。
    // 2. 调用方持有 EXTERNAL_CONTROLLER 自旋锁保证了多核访问互斥，通过强序 DMW0 窗口执行 volatile 写入符合硬件规范。
    unsafe {
        ((mmio_address(resource.registers.start) + offset) as *mut u32).write_volatile(value)
    };
}

/// 初始化板载两组 LIOINTC 控制器：屏蔽全部 64 个中断源，配置为高电平触发，并将所有中断源固定路由到主核（BSP）对应的 CPU 硬件中断线。
///
/// - **调用上下文**：由主核（BSP）在本地中断关闭、从核（AP）尚未唤醒的启动阶段通过 `arch::loongarch64::external_init` 调用一次。
/// - **参数 `info`**：全局启动信息快照，提供两组 LIOINTC 的寄存器地址、父中断号及主核物理 ID。
pub(super) fn initialize(info: &BootInfo) {
    assert_eq!(info.interrupts.source_count, SOURCE_COUNT);
    assert!(info.bsp_hardware_id < BOARD_CPU_COUNT);

    let _controller = EXTERNAL_CONTROLLER.lock();
    // Step 1: 遍历两组 LIOINTC Bank，先全部屏蔽所有中断源，并将触发模式统一设置为高电平触发（POLARITY = 0, EDGE = 0）。
    for resource in info.board.liointc {
        assert!((CPU_HWI0..=CPU_HWI1).contains(&resource.parent_interrupt));
        write(resource, DISABLE, u32::MAX);
        write(resource, POLARITY, 0);
        write(resource, EDGE, 0);
        // Step 2: 逐个配置当前 Bank 内 32 个中断源的单字节路由寄存器（偏移 0x00..0x1F），
        // 低 4 位选择主核物理 ID（1 << bsp_hardware_id），高 4 位选择该 Bank 绑定的 CPU 硬件中断引脚（HWI0 或 HWI1）。
        for source in 0..LIOINTC_SOURCES_PER_BANK as usize {
            let route = (1u8 << (resource.parent_interrupt - CPU_HWI0 + ROUTE_PARENT_SHIFT))
                | (1u8 << info.bsp_hardware_id);
            // SAFETY:
            // 1. 设备树解析阶段已验证 resource.registers 长度至少覆盖 0x38 字节，因此前 32 字节（0..32）路由表完整位于有效 MMIO 窗口内。
            // 2. 当前已持有 EXTERNAL_CONTROLLER 自旋锁且处于主核单线程初始化阶段，单字节 volatile 写入无并发冲突。
            unsafe {
                ((mmio_address(resource.registers.start) + source) as *mut u8)
                    .write_volatile(route);
            }
        }
    }
    // Step 3: 执行完整数据屏障，确保所有屏蔽与路由配置在函数返回前到达硬件控制器。
    barrier();
}

/// 开启或屏蔽指定的规范化外部中断源（`0..64`）。
///
/// 由于 LIOINTC 采用独立的写 1 使能（`ENABLE`）和写 1 屏蔽（`DISABLE`）寄存器，写入单比特掩码只会改变目标中断源的状态，不会影响同组的其他 31 个中断源。
///
/// - **调用上下文**：可在任意内核上下文（含中断处理流程）中调用；函数内部会自动获取 `EXTERNAL_CONTROLLER` 自旋锁。
/// - **参数 `source`**：规范化中断源编号（`0..32` 对应 Bank 0，`32..64` 对应 Bank 1）。
/// - **参数 `enabled`**：`true` 表示向 `ENABLE` 寄存器写 1 开放该中断源，`false` 表示向 `DISABLE` 寄存器写 1 屏蔽该中断源。
pub(super) fn set_enabled(source: u32, enabled: bool) {
    assert!(source < SOURCE_COUNT);

    let _controller = EXTERNAL_CONTROLLER.lock();
    // Step 1: 根据 source / 32 定位所属的 LIOINTC Bank，并向 ENABLE 或 DISABLE 寄存器写入仅包含 (source % 32) 位的掩码。
    let resource =
        crate::boot::info().board.liointc[source as usize / LIOINTC_SOURCES_PER_BANK as usize];
    write(
        resource,
        if enabled { ENABLE } else { DISABLE },
        1u32 << (source % LIOINTC_SOURCES_PER_BANK),
    );
    // Step 2: 执行访存屏障，确保使能或屏蔽操作对后续中断裁决立即生效。
    barrier();
}

/// 读取当前物理 CPU 核心在两组 LIOINTC 上的待处理中断状态，返回第一个已使能且挂起的规范化中断源编号。
///
/// - **调用上下文**：在硬中断处理上下文（本地中断已关闭）中由 `arch::loongarch64::external_claim` 调用。
/// - **返回值**：若存在待处理的有效中断，返回 `Some(source)`（`0..64`）；若两组控制器均无当前核的待处理中断，返回 `None`。
pub(super) fn claim() -> Option<u32> {
    let info = crate::boot::info();
    let hardware_id = crate::cpu::current().hardware_id;
    assert!(hardware_id < BOARD_CPU_COUNT);

    let _controller = EXTERNAL_CONTROLLER.lock();
    // Step 1: 依次扫描 Bank 0 和 Bank 1，读取当前物理核专属的 32 位 ISR 寄存器，并与 ENABLE_STATUS 按位与过滤掉已屏蔽的中断源。
    for (bank, resource) in info.board.liointc.iter().copied().enumerate() {
        let region = resource.isr[hardware_id];
        // SAFETY:
        // 1. 上方 assert! 保证了 hardware_id < 2，且启动时设备树校验已确认每个核的 isr[hardware_id] 窗口至少包含 4 字节且按 4 字节对齐。
        // 2. 通过永久有效的强序 DMW0 映射读取只读状态寄存器，且已持有 EXTERNAL_CONTROLLER 锁，不会产生未定义行为。
        let pending = unsafe { (mmio_address(region.start) as *const u32).read_volatile() }
            & read(resource, ENABLE_STATUS);
        // Step 2: 若当前 Bank 存在挂起中断，利用 trailing_zeros 取最低位的中断源并转换为全局规范化编号返回；
        // 同一 Bank 或另一 Bank 中剩余的挂起电平中断会继续保持置位，留待后续 claim 调用处理。
        if pending != 0 {
            return Some(bank as u32 * LIOINTC_SOURCES_PER_BANK + pending.trailing_zeros());
        }
    }
    None
}

/// 完成指定外部中断源的处理周期。
///
/// LIOINTC 的电平触发中断源没有独立的写 1 清除（W1C）完成寄存器；当具体外设驱动清除设备内部中断状态后，中断线电平自然恢复。
/// 本函数通过执行 `dbar 0` 屏障，保证外设的中断应答写入在调用方重新调用 [`set_enabled`]`(source, true)` 开放中断前已经彻底完成。
///
/// - **调用上下文**：在外设中断处理函数执行完毕、且该中断源仍处于屏蔽状态时，由 `arch::loongarch64::external_complete` 调用。
/// - **参数 `source`**：已完成处理的规范化中断源编号（`0..64`）。
pub(super) fn complete(source: u32) {
    assert!(source < SOURCE_COUNT);
    barrier();
}
