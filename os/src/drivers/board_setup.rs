//! StarFive JH7110 板级外设（SDIO / GMAC）的时钟树、复位控制与引脚复用（IOMUX）配置模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于具体板级外设驱动（`sdmmc`、`dwmac`）与底层 SoC 系统控制寄存器之间，
//! 负责在设备初始化阶段根据 [`boot::DeviceSetup`] 中解析出的设备树（FDT）绑定信息，
//! 对 JH7110 的系统时钟复位控制器（`SYS_CRG` / `AON_CRG`）、锁相环状态（`PLL_SYSCON`）
//! 以及引脚复用控制器（`SYS_PIN` / `AON_PIN`）执行寄存器级的读-改-写配置。
//!
//! ## 核心设计与硬件寄存器布局
//! 本模块仅按需修改目标外设叶子节点所需的时钟门控（Gate）、分频系数（Divider）、
//! 多路复用父源（Mux）与复位线（Reset），保持共享 PLL 与 CPU/总线根时钟的固件配置不变。
//!
//! 1. **CRG 时钟与复位寄存器布局**：
//!    - 每个时钟节点对应一个独立的 32 位寄存器（偏移 = `clock_id * 4`）：
//!
//! ```text
//!  31      28 27      24 23                                      0
//! +----------+----------+-----------------------------------------+
//! |  ENABLE  |   MUX    |                 DIVIDER                 |
//! | (Bit 31) | [27:24]  |                 [23:0]                  |
//! +----------+----------+-----------------------------------------+
//! ```
//!
//!    - 复位控制器按每 32 个复位源共享一个 32 位控制字（`ASSERT`）与一个状态字（`STATUS`）；
//!      写入 `ASSERT` 置 1 触发复位（此时 `STATUS` 对应位变为 0），清 0 解除复位（等待 `STATUS` 变为 1）。
//!
//! 2. **SYS_PIN 引脚复用与电气配置布局**：
//!    - 64 个通用 GPIO 引脚在 `SYS_DOEN`（输出使能）、`SYS_DOUT`（输出信号路由）和 `SYS_GPI`（输入信号路由）
//!      中按每字 4 个字节槽（8 bit/slot）排列；
//!    - `PinResource::mux` 的 32 位编码格式如下：
//!
//! ```text
//!  31          24 23          16 15        10 9      8 7          0
//! +--------------+--------------+------------+--------+------------+
//! |  DIN (GPI)   | DOUT (GPOUT) | DOEN(GPOEN)|  FUNC  | PIN NUMBER |
//! |   [31:24]    |   [23:16]    |  [15:10]   | [9:8]  |   [7:0]    |
//! +--------------+--------------+------------+--------+------------+
//! ```
//!
//! 3. **设备初始化时序（[`apply_setup`]）**：
//!
//! ```text
//! validate_setup (在首次写寄存器前校验所有 Clock/Reset/Pin 范围)
//!    ├─► 1. 配置并使能 assigned-clocks 与设备 clocks (自顶向下开启父门控)
//!    ├─► 2. 解除 IOMUX 控制器自身的时钟门控与复位 (prepare_pins)
//!    ├─► 3. 拉低目标外设的所有复位线 (assert resets)
//!    ├─► 4. 配置引脚复用路由与 Pad 电气属性，并保持复位至少 reset_hold 时长
//!    └─► 5. 释放目标外设的所有复位线并等待硬件状态就绪 (deassert resets)
//! ```
//!
//! ## 锁顺序与并发契约
//! - 全局睡眠互斥锁 [`SETUP_LOCK`] 串行化所有对共享 CRG 与 IOMUX 寄存器的读-改-写操作。
//! - 本模块所有公开入口均必须在时钟源已就绪、允许调度的普通任务上下文中调用；
//!   中断处理路径或早期启动阶段严禁并发访问这些共享控制器寄存器。

use crate::arch::{dma_barrier, mmio_address};
use crate::boot::{
    self, ClockResource, DeviceSetup, GpioInputResource, PinResource, Region, ResourceReference,
};
use crate::sync::SleepMutex;
use crate::time::{self, Duration};
use core::convert::TryFrom;

const SYS_CRG: usize = 0x1302_0000;
const PLL_SYSCON: usize = 0x1303_0000;
const SYS_PIN: usize = 0x1304_0000;
const AON_CRG: usize = 0x1700_0000;
const AON_PIN: usize = 0x1702_0000;

pub(crate) const ENABLE: u32 = 1 << 31;
pub(crate) const DIV_MASK: u32 = 0x00ff_ffff;
pub(crate) const MUX_MASK: u32 = 0x0f00_0000;
const TRANSITION_TIMEOUT: Duration = Duration::from_millis(1);

// CRG 中每个时钟占用一个 32 位字；复位控制器则按每 32 个复位源共享一个 32 位字。
const REGISTER_BYTES: usize = core::mem::size_of::<u32>();
pub(crate) const CLOCK_MUX_SHIFT: u32 = 24;
const MAX_CLOCK_DEPTH: usize = 16;
const SYS_RESET_ASSERT: usize = 0x2f8;
const SYS_RESET_STATUS: usize = 0x308;
pub(crate) const AON_RESET_ASSERT: usize = 0x38;
pub(crate) const AON_RESET_STATUS: usize = 0x3c;
const SYS_CRG_MIN_BYTES: usize = SYS_RESET_STATUS + 3 * REGISTER_BYTES;
const AON_CRG_MIN_BYTES: usize = AON_RESET_STATUS + REGISTER_BYTES;

// 时钟与复位 ID 均归属于各自的控制器（SYS_CRG 或 AON_CRG），不可跨控制器混用。
const SYSCLK_BUS_ROOT: u32 = 5;
const SYSCLK_AXI_CFG0: u32 = 7;
const SYSCLK_STG_AXIAHB: u32 = 8;
const SYSCLK_AHB0: u32 = 9;
const SYSCLK_AHB1: u32 = 10;
const SYSCLK_APB_BUS: u32 = 11;
const SYSCLK_APB0: u32 = 12;
const SYSCLK_SDIO0_AHB: u32 = 91;
const SYSCLK_SDIO1_AHB: u32 = 92;
const SYSCLK_SDIO0_SDCARD: u32 = 93;
const SYSCLK_SDIO1_SDCARD: u32 = 94;
const SYSCLK_GMAC1_AHB: u32 = 97;
const SYSCLK_GMAC1_AXI: u32 = 98;
const SYSCLK_GMAC_SRC: u32 = 99;
const SYSCLK_GMAC1_GTXCLK: u32 = 100;
const SYSCLK_GMAC1_RMII_RTX: u32 = 101;
const SYSCLK_GMAC1_PTP: u32 = 102;
const SYSCLK_GMAC1_RX: u32 = 103;
const SYSCLK_GMAC1_RX_INV: u32 = 104;
const SYSCLK_GMAC1_TX: u32 = 105;
const SYSCLK_GMAC1_TX_INV: u32 = 106;
const SYSCLK_GMAC1_GTXC: u32 = 107;
const SYSCLK_GMAC0_GTXCLK: u32 = 108;
const SYSCLK_GMAC0_PTP: u32 = 109;
const SYSCLK_GMAC_PHY: u32 = 110;
const SYSCLK_GMAC0_GTXC: u32 = 111;
const SYSCLK_IOMUX_APB: u32 = 112;

const AONCLK_GMAC0_AHB: u32 = 2;
const AONCLK_GMAC0_AXI: u32 = 3;
const AONCLK_GMAC0_RMII_RTX: u32 = 4;
const AONCLK_GMAC0_TX: u32 = 5;
const AONCLK_GMAC0_TX_INV: u32 = 6;
const AONCLK_GMAC0_RX: u32 = 7;
const AONCLK_GMAC0_RX_INV: u32 = 8;

const SYSRST_IOMUX_APB: u32 = 2;
const SYSRST_SDIO0_AHB: u32 = 64;
const SYSRST_GMAC1_AHB: u32 = 67;
const AONRST_GMAC0_AXI: u32 = 0;
const AONRST_IOMUX: u32 = 2;

const PLL_MODE_MASK: u32 = 3;
const PLL_MODE_INTEGER: u32 = 3;
const PLL_FBDIV_MASK: u32 = 0xfff;
const PLL_FBDIV_MIN: u32 = 8;
const PLL_PREDIV_MASK: u32 = 0x3f;
const PLL_FRAC_BITS: u32 = 24;
const PLL_FRAC_MASK: u32 = (1 << PLL_FRAC_BITS) - 1;
const PLL_POSTDIV_SHIFT: u32 = 28;
const PLL_POSTDIV_MASK: u32 = 3;

const PLL0_PD: usize = 0x18;
const PLL0_PD_SHIFT: u32 = 24;
const PLL0_FBDIV: usize = 0x1c;
const PLL0_FRAC: usize = 0x20;
const PLL0_PREDIV: usize = 0x24;
const PLL2_PD: usize = 0x2c;
const PLL2_PD_SHIFT: u32 = 15;
const PLL2_FBDIV_SHIFT: u32 = 17;
const PLL2_FRAC: usize = 0x30;
const PLL2_PREDIV: usize = 0x34;
const PLL_MIN_BYTES: usize = PLL2_PREDIV + REGISTER_BYTES;

pub(crate) const SYS_GPIO_COUNT: u32 = 64;
const SYS_DOEN: usize = 0;
const SYS_DOUT: usize = 0x40;
const SYS_GPI: usize = 0x80;
const SYS_GPIOIN: usize = 0x118;
const SYS_PADCFG: usize = 0x120;
const SYS_FUNC_SEL: usize = 0x29c;
const SYS_PIN_MIN_BYTES: usize = SYS_FUNC_SEL + 6 * REGISTER_BYTES;
const AON_PIN_MIN_BYTES: usize = 0x48;
const GPIO_SLOTS_PER_WORD: u32 = 4;
const GPIO_SLOT_BITS: u32 = 8;
const GPIO_DOUT_MASK: u32 = 0x7f;
const GPIO_DOEN_MASK: u32 = 0x3f;
const GPIO_GPI_MASK: u32 = 0x7f;
const GPIO_GPI_PIN_BIAS: u32 = 2;
const GPIO_OUTPUT_DISABLE: u32 = 1;

const PINMUX_PIN_MASK: u32 = 0xff;
const PINMUX_FUNCTION_SHIFT: u32 = 8;
const PINMUX_FUNCTION_MASK: u32 = 3;
const PINMUX_DOEN_SHIFT: u32 = 10;
const PINMUX_DOEN_MASK: u32 = 0x3f;
const PINMUX_DOUT_SHIFT: u32 = 16;
const PINMUX_DOUT_MASK: u32 = 0xff;
const PINMUX_DIN_SHIFT: u32 = 24;
const GPI_NONE: u32 = 0xff;

const PAD_GMAC1_MDC: u32 = 75;
const PAD_GMAC1_RXC: u32 = 82;
const PAD_GMAC1_TXC: u32 = 88;
const PAD_GMAC0_MDC: u32 = 6;
const PAD_GMAC0_TXC: u32 = 19;

const PADCFG_INPUT_ENABLE: u32 = 1;
const PADCFG_DRIVE_SHIFT: u32 = 1;
const PADCFG_DRIVE_MASK: u32 = 3 << PADCFG_DRIVE_SHIFT;
const PADCFG_PULL_UP: u32 = 1 << 3;
const PADCFG_PULL_DOWN: u32 = 1 << 4;
const PADCFG_SLEW_SHIFT: u32 = 5;
const PADCFG_SLEW_MASK: u32 = 1 << PADCFG_SLEW_SHIFT;
const PADCFG_SCHMITT_ENABLE: u32 = 1 << 6;

const GPOUT_SDIO0_RST: u32 = 19;
const GPOUT_SDIO1_CLK: u32 = 55;
const GPOUT_SDIO1_CMD: u32 = 57;
const GPOUT_SDIO1_RST: u32 = 66;
const GPOEN_SDIO1_CMD: u32 = 19;
const GPOEN_SDIO1_DATA7: u32 = 27;
const GPI_SDIO0_CD: u32 = 11;
const GPI_SDIO0_WP: u32 = 13;
const GPI_SDIO1_CD: u32 = 41;
const GPI_SDIO1_STRB: u32 = 53;

// 使用可睡眠互斥锁串行化共享 CRG/IOMUX 寄存器的读-改-写操作；中断上下文不得并发调用本模块。
static SETUP_LOCK: SleepMutex<()> = SleepMutex::new(());

/// 根据规范化的板级设备配置请求，依次完成目标外设的时钟配置、引脚复用及复位释放。
///
/// 若请求中不包含任何时钟、复位或引脚条目，则直接返回而不获取锁。
/// 必须在系统单调时钟已建立、允许调度的普通任务上下文中调用。所有目标复位线拉低（assert）后，
/// 会至少维持 `reset_hold` 指定的单调时间长度再释放（deassert）；若中途出错，此前已生效的共享时钟配置保持现状。
///
/// # Safety
/// 1. `setup` 中引用的所有控制器区间（Provider）、父时钟以及 [`boot::info`] 中的 PLL/CRG 资源必须对应真实板级硬件，
///    且其完整寄存器范围已在内核虚拟地址空间中建立永久强序 MMIO 映射；
/// 2. 调用方必须独占目标外设，且在调用期间已停止该外设的一切 DMA 活动；
/// 3. 系统中所有对 CRG 与 pinctrl 寄存器的修改必须通过 [`SETUP_LOCK`] 串行化，且被修改频率或父源的时钟节点不得有其他并发活动外设正在使用。
pub(crate) unsafe fn apply_setup(
    setup: &DeviceSetup,
    reset_hold: Duration,
) -> Result<(), &'static str> {
    validate_setup(setup)?;
    if setup.clocks.count == 0
        && setup.clocks.assigned_count == 0
        && setup.reset_count == 0
        && setup.pin_count == 0
    {
        return Ok(());
    }

    // Step 1: 获取全局板级配置锁，配置并使能设备所需的时钟与指定时钟（assigned-clocks）。
    let _guard = SETUP_LOCK.lock();
    for clock in &setup.clocks.assigned[..setup.clocks.assigned_count] {
        configure_clock(clock)?;
    }
    for clock in &setup.clocks.entries[..setup.clocks.count] {
        configure_clock(clock)?;
        enable_clock(clock.reference, 0)?;
    }
    // assigned-clocks 中的时钟节点未必重复出现在 clocks 列表中，同样需要递归开启其父源与自身门控。
    for clock in &setup.clocks.assigned[..setup.clocks.assigned_count] {
        enable_clock(clock.reference, 0)?;
    }

    // Step 2: 预先使能 IOMUX 控制器时钟并解除其复位，随后将目标外设的所有复位线置为有效（assert）。
    prepare_pins(setup)?;
    for reset in &setup.resets[..setup.reset_count] {
        set_reset(*reset, true)?;
    }
    let reset_deadline = if setup.reset_count == 0 || reset_hold.is_zero() {
        None
    } else {
        Some(
            time::now()
                .checked_add(reset_hold)
                .ok_or("JH7110 reset hold deadline overflow")?,
        )
    };

    // Step 3: 在外设处于复位保持状态期间配置引脚复用与电气参数，并等待复位保持时间到期。
    for pin in &setup.pins[..setup.pin_count] {
        apply_pin(pin)?;
    }
    if let Some(until) = reset_deadline {
        while time::now() < until {
            core::hint::spin_loop();
        }
    }

    // Step 4: 解除目标外设的所有复位线（deassert），等待硬件复位状态位确认释放完成。
    for reset in &setup.resets[..setup.reset_count] {
        set_reset(*reset, false)?;
    }
    Ok(())
}

/// 从硬件时钟树寄存器实时计算并返回指定时钟节点的当前输出频率（单位：Hz）。
///
/// 对于外部固定频率时钟（`fixed-clock`），直接返回设备树给定的频率；对于 CRG 内部时钟，
/// 沿硬件实际选中的多路复用父源与分频寄存器逐级向上回溯计算。
///
/// # Safety
/// 1. 调用方必须满足与 [`apply_setup`] 相同的永久强序 MMIO 映射与 [`SETUP_LOCK`] 串行化要求；
/// 2. 只能在普通可睡眠任务上下文中调用，且 [`boot::info`] 中的晶振频率与 PLL 寄存器区间必须已由启动阶段正确初始化。
pub(crate) unsafe fn clock_rate(clock: &ClockResource) -> Result<u64, &'static str> {
    validate_clock(clock)?;
    if fixed(clock.reference) {
        return clock.rate.ok_or("fixed clock-frequency is missing");
    }
    let _guard = SETUP_LOCK.lock();
    rate(clock.reference, 0)
}

/// 在普通可睡眠任务中将指定的 `SYS_PIN` 通用引脚配置为施密特触发输入模式，并读取其原始引脚电平。
///
/// 本函数仅返回 GPIO 输入寄存器的原始比特状态（`true` 为高电平，`false` 为低电平），不反转卡检测（CD）等信号的低有效极性。
///
/// # Safety
/// 1. `input.provider` 与系统 `SYS_CRG` 必须来自全局启动信息快照，且已完成完整的永久强序 MMIO 映射；
/// 2. 调用方必须独占 `input.pin` 对应引脚的复用配置权，所有共享 CRG 与 IOMUX 寄存器访问均受 [`SETUP_LOCK`] 保护。
pub(crate) unsafe fn read_gpio_input(input: &GpioInputResource) -> Result<bool, &'static str> {
    check_region(input.provider, SYS_PIN, SYS_PIN_MIN_BYTES)?;
    if SYS_GPIO_COUNT <= input.pin {
        return Err("JH7110 SYS GPIO input pin is outside the supported range");
    }
    // Step 1: 获取全局配置锁，确保 SYS_PIN 依赖的 IOMUX APB 时钟已开启且复位已解除。
    let _guard = SETUP_LOCK.lock();
    let provider = crg(SYS_CRG)?;
    enable_clock(
        ResourceReference {
            provider,
            id: SYSCLK_IOMUX_APB,
        },
        0,
    )?;
    set_reset(
        ResourceReference {
            provider,
            id: SYSRST_IOMUX_APB,
        },
        false,
    )?;

    // Step 2: 计算引脚在每字 4 槽寄存器中的字节偏移与位偏移，启用输入与施密特触发并关闭输出驱动。
    let pin = input.pin;
    let offset = pin as usize / GPIO_SLOTS_PER_WORD as usize * REGISTER_BYTES;
    let shift = pin % GPIO_SLOTS_PER_WORD * GPIO_SLOT_BITS;
    update(
        input.provider,
        SYS_PADCFG + pin as usize * REGISTER_BYTES,
        PADCFG_INPUT_ENABLE | PADCFG_SCHMITT_ENABLE,
        PADCFG_INPUT_ENABLE | PADCFG_SCHMITT_ENABLE,
    )?;
    update(
        input.provider,
        SYS_DOUT + offset,
        GPIO_DOUT_MASK << shift,
        0,
    )?;
    update(
        input.provider,
        SYS_DOEN + offset,
        GPIO_DOEN_MASK << shift,
        GPIO_OUTPUT_DISABLE << shift,
    )?;
    if let Some((offset, shift)) = function_field(pin) {
        update(input.provider, offset, PINMUX_FUNCTION_MASK << shift, 0)?;
    }

    // Step 3: 从 SYS_GPIOIN 寄存器读取该引脚对应的原始输入电平位。
    Ok(read(
        input.provider,
        SYS_GPIOIN + pin as usize / u32::BITS as usize * REGISTER_BYTES,
    ) & (1 << (pin % u32::BITS))
        != 0)
}

/// 校验控制器物理地址区间 `region` 是否精确起始于 `base` 且长度至少包含 `bytes` 字节。
fn check_region(region: Region, base: usize, bytes: usize) -> Result<(), &'static str> {
    if region.start != base || region.end < region.start || region.end - region.start < bytes {
        return Err("JH7110 provider address or register range is unsupported");
    }
    Ok(())
}

/// 从已验证的设备寄存器区间读取 `offset` 处的 32 位寄存器值，并在前后插入 DMA 内存屏障。
fn read(region: Region, offset: usize) -> u32 {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    dma_barrier();
    // SAFETY:
    // 1. 外层公开 `unsafe` 入口契约保证 `region` 具有永久有效的强序 MMIO 映射，且调用方已通过 `check_region` 验证基址与最小长度；
    // 2. 上方 `assert!` 已确认 `offset` 按 4 字节对齐且 `[offset, offset + 4)` 完全落在 `region` 范围内。
    let value = unsafe { ((mmio_address(region.start) + offset) as *const u32).read_volatile() };
    dma_barrier();
    value
}

/// 向已验证的设备寄存器区间 `offset` 处写入 32 位值，并在前后插入架构设备屏障以保证硬件可见顺序。
fn write(region: Region, offset: usize, value: u32) {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    dma_barrier();
    // SAFETY:
    // 1. 外层公开 `unsafe` 入口保证 `region` 已建立永久强序 MMIO 映射，且所有调用路径均持有 `SETUP_LOCK` 串行化寄存器修改；
    // 2. 上方 `assert!` 已确认 `offset` 按 4 字节对齐且目标 32 位字完全处于已校验的 `region` 边界内。
    unsafe { ((mmio_address(region.start) + offset) as *mut u32).write_volatile(value) };
    dma_barrier();
}

/// 轮询等待寄存器 `offset` 在掩码 `mask` 下的位域变为 `value`，超时上限为 [`TRANSITION_TIMEOUT`]（1 毫秒）。
fn wait_bits(region: Region, offset: usize, mask: u32, value: u32) -> Result<(), &'static str> {
    let deadline = time::now()
        .checked_add(TRANSITION_TIMEOUT)
        .ok_or("JH7110 transition deadline overflow")?;
    loop {
        if read(region, offset) & mask == value {
            return Ok(());
        }
        if deadline <= time::now() {
            return Err("JH7110 register transition timed out");
        }
        core::hint::spin_loop();
    }
}

/// 对寄存器 `offset` 执行掩码读-改-写（清除 `mask` 位并填入 `value`），并轮询确认硬件回读值已生效。
fn update(region: Region, offset: usize, mask: u32, value: u32) -> Result<(), &'static str> {
    write(region, offset, (read(region, offset) & !mask) | value);
    wait_bits(region, offset, mask, value)
}

/// 判断资源引用是否表示无 MMIO 寄存器的外部固定频率时钟（`provider` 起止地址与 `id` 全为 0）。
fn fixed(reference: ResourceReference) -> bool {
    reference.provider.start == 0 && reference.provider.end == 0 && reference.id == 0
}

/// 在执行任何硬件寄存器写入前，全面校验 `setup` 中所有时钟、复位及引脚资源的计数与硬件绑定合法性。
fn validate_setup(setup: &DeviceSetup) -> Result<(), &'static str> {
    if setup.clocks.entries.len() < setup.clocks.count
        || setup.clocks.assigned.len() < setup.clocks.assigned_count
        || setup.resets.len() < setup.reset_count
        || setup.pins.len() < setup.pin_count
    {
        return Err("board setup resource count exceeds normalized capacity");
    }
    for clock in setup.clocks.entries[..setup.clocks.count]
        .iter()
        .chain(setup.clocks.assigned[..setup.clocks.assigned_count].iter())
    {
        validate_clock(clock)?;
    }
    for reset in &setup.resets[..setup.reset_count] {
        reset_registers(*reset)?;
    }
    for pin in &setup.pins[..setup.pin_count] {
        validate_pin(pin)?;
    }
    Ok(())
}

/// 校验单个时钟资源是否属于 JH7110 支持的 SDIO、GMAC 或 IOMUX 时钟绑定范围。
fn validate_clock(clock: &ClockResource) -> Result<(), &'static str> {
    if fixed(clock.reference) {
        if clock.parent.is_some() || clock.rate.is_none() || clock.rate == Some(0) {
            return Err("fixed clock requires clock-frequency and cannot select a parent");
        }
        return Ok(());
    }
    match clock.reference.provider.start {
        SYS_CRG => {
            check_region(clock.reference.provider, SYS_CRG, SYS_CRG_MIN_BYTES)?;
            if !matches!(clock.reference.id, SYSCLK_SDIO0_AHB..=SYSCLK_SDIO1_SDCARD | SYSCLK_GMAC1_AHB..=SYSCLK_IOMUX_APB)
            {
                return Err("clock id is outside supported JH7110 SYS SDIO/GMAC/IOMUX bindings");
            }
        }
        AON_CRG => {
            check_region(clock.reference.provider, AON_CRG, AON_CRG_MIN_BYTES)?;
            if !matches!(clock.reference.id, AONCLK_GMAC0_AHB..=AONCLK_GMAC0_RX_INV) {
                return Err("clock id is outside supported JH7110 AON GMAC bindings");
            }
        }
        _ => return Err("clock provider is not a supported JH7110 CRG"),
    }
    if clock.rate == Some(0) {
        return Err("assigned clock rate is zero");
    }
    if let Some(parent) = clock.parent {
        parent_index(clock.reference, parent)?;
    }
    Ok(())
}

/// 从启动阶段选定的 MMC 或 GMAC 设备资源中查找并返回基址为 `base` 的 CRG 控制器物理区间。
fn crg(base: usize) -> Result<Region, &'static str> {
    let board = &boot::info().board;
    for setup in [
        board.mmc.as_ref().map(|r| &r.setup),
        board.gmac.as_ref().map(|r| &r.setup),
    ]
    .iter()
    .flatten()
    {
        for reference in setup.clocks.entries[..setup.clocks.count]
            .iter()
            .chain(setup.clocks.assigned[..setup.clocks.assigned_count].iter())
            .flat_map(|clock| [Some(clock.reference), clock.parent])
            .flatten()
            .chain(setup.resets[..setup.reset_count].iter().copied())
        {
            if reference.provider.start == base {
                check_region(
                    reference.provider,
                    base,
                    if base == SYS_CRG {
                        SYS_CRG_MIN_BYTES
                    } else {
                        AON_CRG_MIN_BYTES
                    },
                )?;
                return Ok(reference.provider);
            }
        }
    }
    Err("required JH7110 CRG provider is missing from selected device resources")
}

/// 将设备树指定的时钟父节点引用 `parent` 转换为目标多路复用时钟 `clock` 硬件寄存器的选择索引（Mux Index）。
fn parent_index(clock: ResourceReference, parent: ResourceReference) -> Result<u32, &'static str> {
    if clock.provider.start == AON_CRG
        && clock.id == AONCLK_GMAC0_TX
        && parent.provider.start == SYS_CRG
        && parent.id == SYSCLK_GMAC0_GTXCLK
    {
        check_region(parent.provider, SYS_CRG, SYS_CRG_MIN_BYTES)?;
        return Ok(0);
    }
    if parent.provider != clock.provider {
        return Err("assigned clock parent provider is not connected to this mux");
    }
    let parents = match (clock.provider.start, clock.id) {
        (SYS_CRG, SYSCLK_GMAC1_RX) => {
            return Err("GMAC RX external parent identity is not normalized")
        }
        (SYS_CRG, SYSCLK_GMAC1_TX) => [SYSCLK_GMAC1_GTXCLK, SYSCLK_GMAC1_RMII_RTX],
        (AON_CRG, AONCLK_GMAC0_TX) => {
            // AON GMAC0_TX 的 0 号父源位于 SYS_CRG（已在函数开头处理），1 号父源为同属 AON_CRG 的 RMII_RTX。
            if parent.id == AONCLK_GMAC0_RMII_RTX {
                return Ok(1);
            }
            return Err("AON GMAC TX parent must name RMII_RTX or normalized SYS GTX clock");
        }
        (AON_CRG, AONCLK_GMAC0_RX) => {
            return Err("GMAC RX external parent identity is not normalized")
        }
        _ => return Err("assigned parent targets a clock without a supported mux"),
    };
    parents
        .iter()
        .position(|id| *id == parent.id)
        .map(|index| index as u32)
        .ok_or("assigned clock parent id is not connected to this mux")
}

/// 将单个时钟资源的父源选择（Mux）与目标输出频率（Rate）写入硬件寄存器。
fn configure_clock(clock: &ClockResource) -> Result<(), &'static str> {
    if fixed(clock.reference) {
        return Ok(());
    }
    if let Some(parent) = clock.parent {
        let index = parent_index(clock.reference, parent)?;
        update(
            clock.reference.provider,
            clock.reference.id as usize * REGISTER_BYTES,
            MUX_MASK,
            index << CLOCK_MUX_SHIFT,
        )?;
    }
    if let Some(requested) = clock.rate {
        set_rate(clock.reference, requested, 0)?;
    }
    Ok(())
}

/// JH7110 局部时钟树中某个时钟节点的直接上游父节点类型。
///
/// 根时钟输入（晶振、PLL 或外部引脚时钟）没有独立的 CRG 门控寄存器。
#[derive(Clone, Copy)]
enum Parent {
    /// 位于 `SYS_CRG` 或 `AON_CRG` 内的上游内部时钟节点。
    Clock(ResourceReference),
    /// 板载外部主晶振输入（通常为 24 MHz，由设备树提供）。
    Oscillator,
    /// `PLL_SYSCON` 中的硬件锁相环输出（参数为 PLL 编号，如 `0` 表示 PLL0、`2` 表示 PLL2）。
    Pll(u32),
    /// GMAC 外部 RMII 参考时钟引脚输入。
    Rmii,
    /// GMAC 外部 RGMII 接收时钟引脚输入。
    Rgmii,
}

/// 在与 `reference` 相同的 CRG 控制器内构造时钟 ID 为 `id` 的上游父节点 [`Parent::Clock`]。
fn local(reference: ResourceReference, id: u32) -> Parent {
    Parent::Clock(ResourceReference {
        provider: reference.provider,
        id,
    })
}

/// 从硬件时钟寄存器读取多路复用选择位（`MUX_MASK`），并校验其处于双父源索引范围（`0..=1`）内。
fn selection(reference: ResourceReference) -> Result<u32, &'static str> {
    let selection = (read(reference.provider, reference.id as usize * REGISTER_BYTES) & MUX_MASK)
        >> CLOCK_MUX_SHIFT;
    if 1 < selection {
        return Err("JH7110 clock mux selection is outside its two parents");
    }
    Ok(selection)
}

/// 查询指定 SDIO、GMAC 或 IOMUX 时钟节点在硬件时钟树中的直接上游父节点。
fn parent(reference: ResourceReference) -> Result<Parent, &'static str> {
    let result = match (reference.provider.start, reference.id) {
        (SYS_CRG, SYSCLK_BUS_ROOT) => {
            if selection(reference)? == 0 {
                Parent::Oscillator
            } else {
                Parent::Pll(2)
            }
        }
        (SYS_CRG, SYSCLK_AXI_CFG0) => local(reference, SYSCLK_BUS_ROOT),
        (SYS_CRG, SYSCLK_STG_AXIAHB) => local(reference, SYSCLK_AXI_CFG0),
        (SYS_CRG, SYSCLK_AHB0 | SYSCLK_AHB1 | SYSCLK_APB_BUS) => {
            local(reference, SYSCLK_STG_AXIAHB)
        }
        (SYS_CRG, SYSCLK_APB0) => local(reference, SYSCLK_APB_BUS),
        (SYS_CRG, SYSCLK_SDIO0_AHB | SYSCLK_SDIO1_AHB | SYSCLK_GMAC1_AHB) => {
            local(reference, SYSCLK_AHB0)
        }
        (SYS_CRG, SYSCLK_SDIO0_SDCARD | SYSCLK_SDIO1_SDCARD) => local(reference, SYSCLK_AXI_CFG0),
        (SYS_CRG, SYSCLK_GMAC1_AXI) => local(reference, SYSCLK_STG_AXIAHB),
        (SYS_CRG, SYSCLK_GMAC_SRC | SYSCLK_GMAC1_GTXCLK | SYSCLK_GMAC0_GTXCLK) => Parent::Pll(0),
        (SYS_CRG, SYSCLK_GMAC1_RMII_RTX) => Parent::Rmii,
        (SYS_CRG, SYSCLK_GMAC1_PTP | SYSCLK_GMAC0_PTP | SYSCLK_GMAC_PHY) => {
            local(reference, SYSCLK_GMAC_SRC)
        }
        (SYS_CRG, SYSCLK_GMAC1_RX) => {
            if selection(reference)? == 0 {
                Parent::Rgmii
            } else {
                local(reference, SYSCLK_GMAC1_RMII_RTX)
            }
        }
        (SYS_CRG, SYSCLK_GMAC1_RX_INV) => local(reference, SYSCLK_GMAC1_RX),
        (SYS_CRG, SYSCLK_GMAC1_TX) => local(
            reference,
            if selection(reference)? == 0 {
                SYSCLK_GMAC1_GTXCLK
            } else {
                SYSCLK_GMAC1_RMII_RTX
            },
        ),
        (SYS_CRG, SYSCLK_GMAC1_TX_INV) => local(reference, SYSCLK_GMAC1_TX),
        (SYS_CRG, SYSCLK_GMAC1_GTXC) => local(reference, SYSCLK_GMAC1_GTXCLK),
        (SYS_CRG, SYSCLK_GMAC0_GTXC) => local(reference, SYSCLK_GMAC0_GTXCLK),
        (SYS_CRG, SYSCLK_IOMUX_APB) => local(reference, SYSCLK_APB_BUS),
        (AON_CRG, AONCLK_GMAC0_AHB | AONCLK_GMAC0_AXI) => Parent::Clock(ResourceReference {
            provider: crg(SYS_CRG)?,
            id: SYSCLK_STG_AXIAHB,
        }),
        (AON_CRG, AONCLK_GMAC0_RMII_RTX) => Parent::Rmii,
        (AON_CRG, AONCLK_GMAC0_TX) => {
            if selection(reference)? == 0 {
                Parent::Clock(ResourceReference {
                    provider: crg(SYS_CRG)?,
                    id: SYSCLK_GMAC0_GTXCLK,
                })
            } else {
                local(reference, AONCLK_GMAC0_RMII_RTX)
            }
        }
        (AON_CRG, AONCLK_GMAC0_TX_INV) => local(reference, AONCLK_GMAC0_TX),
        (AON_CRG, AONCLK_GMAC0_RX) => {
            if selection(reference)? == 0 {
                Parent::Rgmii
            } else {
                local(reference, AONCLK_GMAC0_RMII_RTX)
            }
        }
        (AON_CRG, AONCLK_GMAC0_RX_INV) => local(reference, AONCLK_GMAC0_RX),
        _ => return Err("JH7110 clock tree edge is unsupported"),
    };
    Ok(result)
}

/// 返回指定时钟节点硬件分频寄存器的最大允许整数分频值；返回 `0` 表示该节点不包含分频字段。
fn divider_max(reference: ResourceReference) -> u32 {
    match (reference.provider.start, reference.id) {
        (SYS_CRG, SYSCLK_AXI_CFG0) => 3,
        (SYS_CRG, SYSCLK_STG_AXIAHB) => 2,
        (SYS_CRG, SYSCLK_APB_BUS) => 8,
        (
            SYS_CRG,
            SYSCLK_SDIO0_SDCARD | SYSCLK_SDIO1_SDCARD | SYSCLK_GMAC1_GTXCLK | SYSCLK_GMAC0_GTXCLK,
        ) => 15,
        (SYS_CRG, SYSCLK_GMAC_SRC) => 7,
        (SYS_CRG, SYSCLK_GMAC1_RMII_RTX) | (AON_CRG, AONCLK_GMAC0_RMII_RTX) => 30,
        (SYS_CRG, SYSCLK_GMAC1_PTP | SYSCLK_GMAC0_PTP | SYSCLK_GMAC_PHY) => 31,
        _ => 0,
    }
}

/// 判断指定时钟节点的硬件寄存器是否包含独立的门控使能位（Bit 31 `ENABLE`）。
fn gated(reference: ResourceReference) -> bool {
    matches!((reference.provider.start, reference.id),
        (SYS_CRG, SYSCLK_AHB0 | SYSCLK_AHB1 | SYSCLK_APB0 | SYSCLK_SDIO0_AHB..=SYSCLK_SDIO1_SDCARD | SYSCLK_GMAC1_AHB | SYSCLK_GMAC1_AXI | SYSCLK_GMAC1_PTP | SYSCLK_GMAC1_TX | SYSCLK_GMAC1_GTXC..=SYSCLK_IOMUX_APB)
        | (AON_CRG, AONCLK_GMAC0_AHB | AONCLK_GMAC0_AXI | AONCLK_GMAC0_TX))
}

/// 读取并校验指定时钟节点的当前硬件整数分频系数（无分频器的节点固定返回 `1`）。
fn divisor(reference: ResourceReference) -> Result<u64, &'static str> {
    let maximum = divider_max(reference);
    if maximum == 0 {
        return Ok(1);
    }
    let divisor = read(reference.provider, reference.id as usize * REGISTER_BYTES) & DIV_MASK;
    if divisor == 0 || maximum < divisor {
        return Err("JH7110 clock divider is zero or outside the hardware range");
    }
    Ok(u64::from(divisor))
}

/// 按时钟树拓扑自顶向下递归开启上游父时钟门控，再置位当前时钟节点的使能位。
fn enable_clock(reference: ResourceReference, depth: usize) -> Result<(), &'static str> {
    if fixed(reference) {
        return Ok(());
    }
    if MAX_CLOCK_DEPTH <= depth {
        return Err("JH7110 selected clock path exceeds its bounded depth");
    }
    if let Parent::Clock(upstream) = parent(reference)? {
        enable_clock(upstream, depth + 1)?;
    }
    divisor(reference)?;
    if gated(reference) {
        update(
            reference.provider,
            reference.id as usize * REGISTER_BYTES,
            ENABLE,
            ENABLE,
        )?;
    }
    Ok(())
}

/// 从全局启动快照中读取板载主晶振频率（单位：Hz），若缺失或为 0 则返回错误。
fn oscillator_rate() -> Result<u64, &'static str> {
    boot::info()
        .board
        .clock_inputs
        .oscillator_hz
        .filter(|rate| *rate != 0)
        .ok_or("JH7110 oscillator clock-frequency is missing from normalized board inputs")
}

/// 读取 `PLL_SYSCON` 中指定锁相环（PLL0 或 PLL2）的整数/小数倍频与预分频/后分频寄存器，计算其实时输出频率（单位：Hz）。
fn pll_rate(id: u32) -> Result<u64, &'static str> {
    let registers = boot::info().board.clock_inputs.pll_registers;
    check_region(registers, PLL_SYSCON, PLL_MIN_BYTES)?;
    let (pd, shift, fbdiv_offset, fbdiv_shift, frac_offset, prediv_offset) = match id {
        0 => (
            PLL0_PD,
            PLL0_PD_SHIFT,
            PLL0_FBDIV,
            0,
            PLL0_FRAC,
            PLL0_PREDIV,
        ),
        2 => (
            PLL2_PD,
            PLL2_PD_SHIFT,
            PLL2_PD,
            PLL2_FBDIV_SHIFT,
            PLL2_FRAC,
            PLL2_PREDIV,
        ),
        _ => return Err("PLL id is outside the selected SDIO/GMAC clock paths"),
    };
    let mode = (read(registers, pd) >> shift) & PLL_MODE_MASK;
    let fbdiv = (read(registers, fbdiv_offset) >> fbdiv_shift) & PLL_FBDIV_MASK;
    let fraction = read(registers, frac_offset);
    let prediv = read(registers, prediv_offset) & PLL_PREDIV_MASK;
    if prediv == 0 || fbdiv < PLL_FBDIV_MIN || !matches!(mode, 0 | PLL_MODE_INTEGER) {
        return Err("JH7110 PLL mode or divider is invalid");
    }
    let fractional = if mode == 0 {
        fraction & PLL_FRAC_MASK
    } else {
        0
    };
    let postdiv = (fraction >> PLL_POSTDIV_SHIFT) & PLL_POSTDIV_MASK;
    let oscillator = u128::from(oscillator_rate()?);
    let multiplied = oscillator * u128::from(fbdiv)
        + oscillator * u128::from(fractional) / (1u128 << PLL_FRAC_BITS);
    let output = multiplied / (u128::from(prediv) << postdiv);
    u64::try_from(output)
        .ok()
        .filter(|rate| *rate != 0)
        .ok_or("JH7110 PLL output rate is invalid")
}

/// 计算上游父节点 `upstream` 的输出频率（单位：Hz）；未在启动信息中提供频率的外部 RMII/RGMII 引脚时钟返回错误。
fn parent_rate(upstream: Parent, depth: usize) -> Result<u64, &'static str> {
    match upstream {
        Parent::Clock(reference) => rate(reference, depth),
        Parent::Oscillator => oscillator_rate(),
        Parent::Pll(id) => pll_rate(id),
        Parent::Rmii => Err("GMAC RMII external input clock-frequency is not normalized"),
        Parent::Rgmii => Err("GMAC RGMII external input clock-frequency is not normalized"),
    }
}

/// 沿硬件当前选中的父节点路径递归计算 `reference` 时钟节点的输出频率（单位：Hz）。
fn rate(reference: ResourceReference, depth: usize) -> Result<u64, &'static str> {
    if MAX_CLOCK_DEPTH <= depth {
        return Err("JH7110 selected clock path exceeds its bounded depth");
    }
    let output = parent_rate(parent(reference)?, depth + 1)? / divisor(reference)?;
    if output == 0 {
        return Err("JH7110 clock output rate is zero");
    }
    Ok(output)
}

/// 计算 SDIO 卡时钟的整数分频系数：采用向上取整（`ceil(parent / requested)`）以保证实际输出频率不超过卡协议上限。
fn sd_divider(parent: u64, requested: u64, maximum: u32) -> Result<u32, &'static str> {
    if requested == 0 {
        return Err("requested SDIO clock rate is zero");
    }
    let divider = parent / requested + u64::from(parent % requested != 0);
    if divider == 0 || u64::from(maximum) < divider {
        return Err("requested SDIO rate exceeds the hardware divider range");
    }
    Ok(divider as u32)
}

/// 计算 GMAC 等严格链路时钟的精确整数分频系数：要求父时钟频率必须能被目标频率整除。
fn exact_divider(parent: u64, requested: u64, maximum: u32) -> Result<u32, &'static str> {
    if requested == 0 || parent % requested != 0 {
        return Err("requested clock rate has no exact integer divider");
    }
    let divider = parent / requested;
    if divider == 0 || u64::from(maximum) < divider {
        return Err("requested clock rate exceeds the hardware divider range");
    }
    Ok(divider as u32)
}

/// 设置目标时钟节点的输出频率：对含分频器的 SDIO/GMAC 节点直接更新分频字段，对门控/反相派生节点则向上游父节点传播。
fn set_rate(
    reference: ResourceReference,
    requested: u64,
    depth: usize,
) -> Result<(), &'static str> {
    if MAX_CLOCK_DEPTH <= depth {
        return Err("JH7110 selected clock path exceeds its bounded depth");
    }
    if matches!((reference.provider.start, reference.id),
        (SYS_CRG, SYSCLK_SDIO0_SDCARD | SYSCLK_SDIO1_SDCARD | SYSCLK_GMAC_SRC..=SYSCLK_GMAC1_PTP | SYSCLK_GMAC0_GTXCLK..=SYSCLK_GMAC_PHY) | (AON_CRG, AONCLK_GMAC0_RMII_RTX))
    {
        let source = parent_rate(parent(reference)?, depth + 1)?;
        let maximum = divider_max(reference);
        let divider = if matches!(
            (reference.provider.start, reference.id),
            (SYS_CRG, SYSCLK_SDIO0_SDCARD | SYSCLK_SDIO1_SDCARD)
        ) {
            sd_divider(source, requested, maximum)?
        } else {
            exact_divider(source, requested, maximum)?
        };
        return update(
            reference.provider,
            reference.id as usize * REGISTER_BYTES,
            DIV_MASK,
            divider,
        );
    }
    if matches!(
        (reference.provider.start, reference.id),
        (
            SYS_CRG,
            SYSCLK_GMAC1_RX_INV..=SYSCLK_GMAC1_GTXC | SYSCLK_GMAC0_GTXC
        ) | (
            AON_CRG,
            AONCLK_GMAC0_TX | AONCLK_GMAC0_TX_INV | AONCLK_GMAC0_RX_INV
        )
    ) {
        if let Parent::Clock(upstream) = parent(reference)? {
            return set_rate(upstream, requested, depth + 1);
        }
    }
    if rate(reference, depth)? == requested {
        Ok(())
    } else {
        Err("requested rate would require changing a shared root or an unsupported input")
    }
}

/// 计算指定复位源在对应 CRG 中的控制字偏移（`ASSERT`）、状态字偏移（`STATUS`）与单比特掩码。
fn reset_registers(reference: ResourceReference) -> Result<(usize, usize, u32), &'static str> {
    let (assert, status) = match (reference.provider.start, reference.id) {
        (SYS_CRG, SYSRST_IOMUX_APB | SYSRST_SDIO0_AHB..=SYSRST_GMAC1_AHB) => {
            check_region(reference.provider, SYS_CRG, SYS_CRG_MIN_BYTES)?;
            (SYS_RESET_ASSERT, SYS_RESET_STATUS)
        }
        (AON_CRG, AONRST_GMAC0_AXI..=AONRST_IOMUX) => {
            check_region(reference.provider, AON_CRG, AON_CRG_MIN_BYTES)?;
            (AON_RESET_ASSERT, AON_RESET_STATUS)
        }
        _ => return Err("reset provider/id is outside the supported SDIO/GMAC/IOMUX bindings"),
    };
    let word = reference.id as usize / u32::BITS as usize * REGISTER_BYTES;
    Ok((
        assert + word,
        status + word,
        1 << (reference.id % u32::BITS),
    ))
}

/// 置位或清除指定复位源的控制位，并轮询独立的状态寄存器直至硬件复位状态切换完成（JH7110 状态位：`0` 为复位有效，`1` 为复位解除）。
fn set_reset(reference: ResourceReference, asserted: bool) -> Result<(), &'static str> {
    let (control, status, mask) = reset_registers(reference)?;
    write(
        reference.provider,
        control,
        (read(reference.provider, control) & !mask) | if asserted { mask } else { 0 },
    );
    wait_bits(
        reference.provider,
        status,
        mask,
        if asserted { 0 } else { mask },
    )
}

/// 将规范化的引脚电气配置（上下拉、驱动电流 mA、输入使能、施密特触发、摆率）编码为 `SYS_PADCFG` 寄存器的掩码与目标位值。
fn pad_config(pin: &PinResource) -> Result<(u32, u32), &'static str> {
    let mut mask = 0;
    let mut value = 0;
    if pin.pull_up && pin.bias_disable {
        return Err("pin pull-up and bias-disable conflict");
    }
    if pin.pull_up || pin.bias_disable {
        mask |= PADCFG_PULL_DOWN | PADCFG_PULL_UP;
        if pin.pull_up {
            value |= PADCFG_PULL_UP;
        }
    }
    if let Some(strength) = pin.drive_strength {
        let encoded = match strength {
            2 => 0,
            4 => 1,
            8 => 2,
            12 => 3,
            _ => return Err("JH7110 pin drive-strength must be 2/4/8/12 mA"),
        };
        mask |= PADCFG_DRIVE_MASK;
        value |= encoded << PADCFG_DRIVE_SHIFT;
    }
    for (setting, bit) in [
        (pin.input_enable, PADCFG_INPUT_ENABLE),
        (pin.schmitt_enable, PADCFG_SCHMITT_ENABLE),
    ] {
        if let Some(enabled) = setting {
            mask |= bit;
            if enabled {
                value |= bit;
            }
        }
    }
    if let Some(slew) = pin.slew_rate {
        if 1 < slew {
            return Err("JH7110 pin slew-rate must be zero or one");
        }
        mask |= PADCFG_SLEW_MASK;
        value |= slew << PADCFG_SLEW_SHIFT;
    }
    Ok((mask, value))
}

/// 查询需要显式写入 `SYS_FUNC_SEL` 寄存器以切换至 Function 0 的引脚对应的寄存器字节偏移与位偏移。
fn function_field(pin: u32) -> Option<(usize, u32)> {
    match pin {
        6 => Some((SYS_FUNC_SEL + 5 * REGISTER_BYTES, 0)),
        7..=9 => Some((SYS_FUNC_SEL + 5 * REGISTER_BYTES, 2 + 3 * (pin - 7))),
        10..=19 => Some((SYS_FUNC_SEL, 2 + 3 * (pin - 10))),
        20..=29 => Some((SYS_FUNC_SEL + REGISTER_BYTES, 3 * (pin - 20))),
        30..=35 => Some((SYS_FUNC_SEL + 2 * REGISTER_BYTES, 3 * (pin - 30))),
        36..=40 => Some((SYS_FUNC_SEL + 2 * REGISTER_BYTES, 17 + 3 * (pin - 36))),
        41..=51 => Some((SYS_FUNC_SEL + 3 * REGISTER_BYTES, 3 * (pin - 41))),
        52..=54 => Some((SYS_FUNC_SEL + 4 * REGISTER_BYTES, 2 * (pin - 52))),
        55..=63 => Some((SYS_FUNC_SEL + 4 * REGISTER_BYTES, 6 + 3 * (pin - 55))),
        PAD_GMAC1_RXC => Some((SYS_FUNC_SEL, 0)),
        _ => None,
    }
}

/// 校验单个引脚复用资源是否属于受支持的 `SYS_PIN`/`AON_PIN` 控制器及合法的 SDIO/GMAC 信号路由。
fn validate_pin(pin: &PinResource) -> Result<(), &'static str> {
    let number = pin.mux & PINMUX_PIN_MASK;
    let function = (pin.mux >> PINMUX_FUNCTION_SHIFT) & PINMUX_FUNCTION_MASK;
    let (mask, _) = pad_config(pin)?;
    match pin.provider.start {
        SYS_PIN => {
            check_region(pin.provider, SYS_PIN, SYS_PIN_MIN_BYTES)?;
            if PAD_GMAC1_TXC < number || function != 0 {
                return Err("SYS pin mux is outside supported SDIO/GMAC function zero");
            }
            if number < SYS_GPIO_COUNT {
                let output = (pin.mux >> PINMUX_DOUT_SHIFT) & PINMUX_DOUT_MASK;
                let enable = (pin.mux >> PINMUX_DOEN_SHIFT) & PINMUX_DOEN_MASK;
                let input = pin.mux >> PINMUX_DIN_SHIFT;
                if !matches!(
                    output,
                    0 | 1 | GPOUT_SDIO0_RST | GPOUT_SDIO1_CLK | GPOUT_SDIO1_CMD..=GPOUT_SDIO1_RST
                ) || !matches!(enable, 0 | 1 | GPOEN_SDIO1_CMD..=GPOEN_SDIO1_DATA7)
                    || !matches!(input, GPI_SDIO0_CD..=GPI_SDIO0_WP | GPI_SDIO1_CD..=GPI_SDIO1_STRB | GPI_NONE)
                    || GPIO_DOUT_MASK < output
                {
                    return Err("SYS GPIO mux does not name a supported SDIO signal");
                }
            } else if pin.mux != (number | (GPIO_OUTPUT_DISABLE << PINMUX_DOEN_SHIFT)) {
                return Err("dedicated SDIO/GMAC pad has invalid GPIO routing fields");
            }
            if PAD_GMAC1_MDC <= number && mask != 0 {
                return Err("dedicated GMAC pads have no supported pad electrical registers");
            }
        }
        AON_PIN => {
            check_region(pin.provider, AON_PIN, AON_PIN_MIN_BYTES)?;
            if !(PAD_GMAC0_MDC..=PAD_GMAC0_TXC).contains(&number)
                || pin.mux != (number | (GPIO_OUTPUT_DISABLE << PINMUX_DOEN_SHIFT))
            {
                return Err("AON pin mux is outside supported dedicated GMAC pads");
            }
            if mask != 0 {
                return Err("dedicated AON GMAC pads have no supported pad electrical registers");
            }
        }
        _ => return Err("pin provider is not a supported JH7110 IOMUX"),
    }
    Ok(())
}

/// 在访问引脚复用寄存器前，按需开启 `SYS_PIN`/`AON_PIN` 依赖的 IOMUX 时钟并解除其复位（仅解除不重新拉低复位，以免破坏已初始化外设的引脚状态）。
fn prepare_pins(setup: &DeviceSetup) -> Result<(), &'static str> {
    if setup.pins[..setup.pin_count]
        .iter()
        .any(|pin| pin.provider.start == SYS_PIN)
    {
        let provider = crg(SYS_CRG)?;
        enable_clock(
            ResourceReference {
                provider,
                id: SYSCLK_IOMUX_APB,
            },
            0,
        )?;
        set_reset(
            ResourceReference {
                provider,
                id: SYSRST_IOMUX_APB,
            },
            false,
        )?;
    }
    if setup.pins[..setup.pin_count]
        .iter()
        .any(|pin| pin.provider.start == AON_PIN)
    {
        set_reset(
            ResourceReference {
                provider: crg(AON_CRG)?,
                id: AONRST_IOMUX,
            },
            false,
        )?;
    }
    Ok(())
}

/// 将单个引脚的输出信号路由（`DOUT`）、输出使能路由（`DOEN`）、输入信号路由（`GPI`）、功能选择（`FUNC_SEL`）与电气配置（`PADCFG`）写入硬件寄存器。
fn apply_pin(pin: &PinResource) -> Result<(), &'static str> {
    let number = pin.mux & PINMUX_PIN_MASK;
    if pin.provider.start == SYS_PIN {
        if number < SYS_GPIO_COUNT {
            let offset = number as usize / GPIO_SLOTS_PER_WORD as usize * REGISTER_BYTES;
            let shift = number % GPIO_SLOTS_PER_WORD * GPIO_SLOT_BITS;
            let output = (pin.mux >> PINMUX_DOUT_SHIFT) & PINMUX_DOUT_MASK;
            let enable = (pin.mux >> PINMUX_DOEN_SHIFT) & PINMUX_DOEN_MASK;
            let input = pin.mux >> PINMUX_DIN_SHIFT;
            update(
                pin.provider,
                SYS_DOUT + offset,
                GPIO_DOUT_MASK << shift,
                output << shift,
            )?;
            update(
                pin.provider,
                SYS_DOEN + offset,
                GPIO_DOEN_MASK << shift,
                enable << shift,
            )?;
            if input != GPI_NONE {
                let input_offset = input as usize / GPIO_SLOTS_PER_WORD as usize * REGISTER_BYTES;
                let input_shift = input % GPIO_SLOTS_PER_WORD * GPIO_SLOT_BITS;
                update(
                    pin.provider,
                    SYS_GPI + input_offset,
                    GPIO_GPI_MASK << input_shift,
                    (number + GPIO_GPI_PIN_BIAS) << input_shift,
                )?;
            }
        }

        if let Some((offset, shift)) = function_field(number) {
            update(pin.provider, offset, PINMUX_FUNCTION_MASK << shift, 0)?;
        }

        let (mask, value) = pad_config(pin)?;
        if mask != 0 {
            update(
                pin.provider,
                SYS_PADCFG + number as usize * REGISTER_BYTES,
                mask,
                value,
            )?;
        }
    }
    Ok(())
}

#[cfg(feature = "ktest")]
#[::ktest::suite(context = "Boot", tags("boot", "board-setup"), timeout_ms = 1000)]
mod checks {
    use super::*;

    /// 测试目标：验证纯软件计算函数（分频系数、功能选择位偏移、Pad 电气位掩码及复位寄存器定位）的编码准确性与非法参数拒绝行为。
    /// 场景步骤：依次传入合法与越界的 GMAC/SDIO 分频请求、引脚编号、上下拉冲突配置、非法驱动电流及复位 ID。
    /// 预期结果：合法输入返回精确匹配的寄存器偏移/掩码/分频值，越界或冲突输入均返回 `Err`。
    #[test]
    fn clock_and_pin_encoding() {
        assert_eq!(exact_divider(200_000_000, 50_000_000, 15), Ok(4));
        assert!(exact_divider(200_000_000, 48_000_000, 15).is_err());
        assert!(exact_divider(1_500_000_000, 25_000_000, 15).is_err());
        assert_eq!(sd_divider(396_000_000, 50_000_000, 15), Ok(8));
        assert!(sd_divider(396_000_000, 0, 15).is_err());
        assert!(sd_divider(396_000_000, 1_000_000, 15).is_err());

        assert_eq!(function_field(9), Some((0x2b0, 8)));
        assert_eq!(function_field(10), Some((0x29c, 2)));

        let mut pin = PinResource {
            provider: Region {
                start: SYS_PIN,
                end: SYS_PIN + 0x10000,
            },
            mux: (255 << 24) | (55 << 16) | 10,
            pull_up: true,
            drive_strength: Some(12),
            input_enable: Some(false),
            schmitt_enable: Some(false),
            slew_rate: Some(0),
            ..PinResource::default()
        };
        assert_eq!(pad_config(&pin), Ok((0x7f, 0x0e)));
        assert!(validate_pin(&pin).is_ok());
        pin.bias_disable = true;
        assert!(validate_pin(&pin).is_err());
        pin.bias_disable = false;
        pin.drive_strength = Some(6);
        assert!(validate_pin(&pin).is_err());

        let reset = ResourceReference {
            provider: Region {
                start: SYS_CRG,
                end: SYS_CRG + 0x10000,
            },
            id: 66,
        };
        assert_eq!(reset_registers(reset), Ok((0x300, 0x310, 4)));
        assert!(validate_setup(&DeviceSetup::default()).is_ok());
    }
}
