//! 所选 MMC/GMAC 的 JH7110 时钟、复位和引脚动作；不建立设备注册表。
//!
//! 归档身份见 `build/stage11/linux-reference-exports.json`；以下是实际 7.3.0-rc4
//! 归档中 `linux/` 下的成员，不要求它们已全部导出到普通源码目录：
//! `drivers/clk/starfive/clk-starfive-jh7110-{sys,aon,pll}.c`、
//! `clk-starfive-jh71x0.{c,h}`、`drivers/reset/starfive/reset-starfive-jh71x0.c`、
//! `reset-starfive-jh7110.c`、`drivers/pinctrl/starfive/pinctrl-starfive-jh7110{,-sys}.c`，
//! 以及 `arch/riscv/boot/dts/starfive/jh7110{,-common}.dtsi` 的 MMC/GMAC 绑定。
//! 门控交叉依据为固定 `ref/stage11/moss-ba14c9c06eea/` 中的
//! `vendor/axdriver/src/jh7110.rs:20-29,65-95`；其它同级板资料不提供可复用的时钟状态机。
//! 只改叶设备需要的门、分频、父源和复位；共享 PLL、CPU/总线根的配置保留。

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
const ENABLE: u32 = 1 << 31;
const DIV_MASK: u32 = 0x00ff_ffff;
const MUX_MASK: u32 = 0x0f00_0000;
const TRANSITION_TIMEOUT: Duration = Duration::from_millis(1);

// 可睡锁串行共享寄存器的 RMW；IRQ/早期 RTC 不得与这些初始化动作并发。
static SETUP_LOCK: SleepMutex<()> = SleepMutex::new(());

/// 对一次规范化请求执行实际板设备动作；空请求不取锁、不读取启动快照。
///
/// 只能在时间源已建立、可切换的普通任务中调用。所选复位全部 assert 后，
/// 至少保持 reset_hold 的单调时间；失败前已完成的动作保留，不回滚共享时钟。
///
/// # Safety
/// 所有 provider、父时钟和启动快照中的 PLL/syscrg 必须来自当前实际板资源，
/// 其完整范围必须已有永久设备 MMIO 映射；调用者须独占目标外设且已阻止其 DMA。
/// 全部 CRG/pinctrl 使用者须通过本锁串行，或在外部保证不并发修改同一寄存器；
/// 调用者还须保证被改变的父时钟没有本请求以外的活动使用者。此处不取得资源拥有权。
/// 叶驱动必须先检验自己必需的 clock/reset/pin，不能把缺资源传成空请求。
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
    let _guard = SETUP_LOCK.lock();
    for clock in &setup.clocks.assigned[..setup.clocks.assigned_count] {
        configure_clock(clock)?;
    }
    for clock in &setup.clocks.entries[..setup.clocks.count] {
        configure_clock(clock)?;
        enable_clock(clock.reference, 0)?;
    }
    // assigned-clocks 也可能不在 clocks 中；它自己的父源和门同样必须生效。
    for clock in &setup.clocks.assigned[..setup.clocks.assigned_count] {
        enable_clock(clock.reference, 0)?;
    }
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
    for pin in &setup.pins[..setup.pin_count] {
        apply_pin(pin)?;
    }
    if let Some(until) = reset_deadline {
        while time::now() < until {
            core::hint::spin_loop();
        }
    }
    for reset in &setup.resets[..setup.reset_count] {
        set_reset(*reset, false)?;
    }
    Ok(())
}

/// 读取所选实际时钟树的当前输出率；请求中的 rate 不代替硬件读数。
///
/// # Safety
/// 与 apply_setup 相同的永久映射和共享寄存器串行义务适用；只能在可切换任务中调用。
/// JH7110 的晶振频率、PLL 寄存器资源必须已经由唯一启动发现发布，不能由调用者猜测。
pub(crate) unsafe fn clock_rate(clock: &ClockResource) -> Result<u64, &'static str> {
    validate_clock(clock)?;
    if fixed(clock.reference) {
        return clock.rate.ok_or("fixed clock-frequency is missing");
    }
    let _guard = SETUP_LOCK.lock();
    rate(clock.reference, 0)
}

/// 在普通可睡任务中配置独立 SYS GPIO 输入并读取原始电平，不解释卡检测极性。
///
/// # Safety
/// GPIO provider 与所选设备的 SYS CRG 必须来自唯一启动快照，并已有完整永久 MMIO
/// 映射；调用者独占该 pin 的功能选择，全部共享 CRG/pinctrl 更新遵守 SETUP_LOCK。
pub(crate) unsafe fn read_gpio_input(input: &GpioInputResource) -> Result<bool, &'static str> {
    check_region(input.provider, SYS_PIN, 0x2b4)?;
    if 64 <= input.pin {
        return Err("JH7110 SYS GPIO input pin is outside the supported range");
    }
    let _guard = SETUP_LOCK.lock();
    let provider = crg(SYS_CRG)?;
    enable_clock(ResourceReference { provider, id: 112 }, 0)?;
    set_reset(ResourceReference { provider, id: 2 }, false)?;
    let pin = input.pin;
    let offset = pin as usize / 4 * 4;
    let shift = pin % 4 * 8;
    update(
        input.provider,
        0x120 + pin as usize * 4,
        (1 << 0) | (1 << 6),
        (1 << 0) | (1 << 6),
    )?;
    update(input.provider, 0x40 + offset, 0x7f << shift, 0)?;
    update(input.provider, offset, 0x3f << shift, 1 << shift)?;
    if let Some((offset, shift)) = function_field(pin) {
        update(input.provider, offset, 3 << shift, 0)?;
    }
    Ok(read(input.provider, 0x118 + pin as usize / 32 * 4) & (1 << (pin % 32)) != 0)
}

/// 检查完整寄存器范围，防止偏移或地址加法溢出后再进行 MMIO。
fn check_region(region: Region, base: usize, bytes: usize) -> Result<(), &'static str> {
    if region.start != base || region.end < region.start || region.end - region.start < bytes {
        return Err("JH7110 provider address or register range is unsupported");
    }
    Ok(())
}

/// 私有 MMIO 读取只由已验证资源和外层 unsafe 调用合同覆盖。
fn read(region: Region, offset: usize) -> u32 {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    dma_barrier();
    // SAFETY: 对外入口要求 provider 永久映射，所有调用前已校验该硬件范围；偏移覆盖完整 u32。
    let value = unsafe { ((mmio_address(region.start) + offset) as *const u32).read_volatile() };
    dma_barrier();
    value
}

/// 写入实际设备寄存器并以架构设备屏障发布，不以编译器屏障替代设备顺序。
fn write(region: Region, offset: usize, value: u32) {
    assert!(offset % 4 == 0 && offset + 4 <= region.end - region.start);
    dma_barrier();
    // SAFETY: 外层调用者提供永久映射和独占，SETUP_LOCK 串行 RMW；本访问在已验证的四字节范围内。
    unsafe { ((mmio_address(region.start) + offset) as *mut u32).write_volatile(value) };
    dma_barrier();
}

/// 等待真实寄存器字段达到期望值，期限属于原单调时间域。
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

/// 只更新请求字段，保留同一字内其它设备状态，并确认实际回读。
fn update(region: Region, offset: usize, mask: u32, value: u32) -> Result<(), &'static str> {
    write(region, offset, (read(region, offset) & !mask) | value);
    wait_bits(region, offset, mask, value)
}

/// 零范围只代表规范化的 fixed-clock，非零但空的 provider 不作固定输入处理。
fn fixed(reference: ResourceReference) -> bool {
    reference.provider.start == 0 && reference.provider.end == 0 && reference.id == 0
}

/// 校验计数和所有请求的绑定形状，在第一次 MMIO 写前拒绝不支持的输入。
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

/// 只接受本轮 SDIO、GMAC 以及其所需 IOMUX 的实际 provider/id。
fn validate_clock(clock: &ClockResource) -> Result<(), &'static str> {
    if fixed(clock.reference) {
        if clock.parent.is_some() || clock.rate.is_none() || clock.rate == Some(0) {
            return Err("fixed clock requires clock-frequency and cannot select a parent");
        }
        return Ok(());
    }
    match clock.reference.provider.start {
        SYS_CRG => {
            check_region(clock.reference.provider, SYS_CRG, 0x314)?;
            if !matches!(clock.reference.id, 91..=94 | 97..=112) {
                return Err("clock id is outside supported JH7110 SYS SDIO/GMAC/IOMUX bindings");
            }
        }
        AON_CRG => {
            check_region(clock.reference.provider, AON_CRG, 0x40)?;
            if !matches!(clock.reference.id, 2..=8) {
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

/// 从所选两个设备的真实 provider 引用取得 CRG，缺映射时不从板地址制造资源。
fn crg(base: usize) -> Result<Region, &'static str> {
    let board = &boot::info().board;
    for setup in [
        board.mmc.as_ref().map(|r| &r.setup),
        board.gmac.as_ref().map(|r| &r.setup),
    ]
    .into_iter()
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
                    if base == SYS_CRG { 0x314 } else { 0x40 },
                )?;
                return Ok(reference.provider);
            }
        }
    }
    Err("required JH7110 CRG provider is missing from selected device resources")
}

/// 只把实际硬件支持的 GMAC mux 父引用转换成选择位；固定输入 phandle 丢失身份时拒绝猜测。
fn parent_index(clock: ResourceReference, parent: ResourceReference) -> Result<u32, &'static str> {
    if clock.provider.start == AON_CRG
        && clock.id == 5
        && parent.provider.start == SYS_CRG
        && parent.id == 108
    {
        check_region(parent.provider, SYS_CRG, 0x314)?;
        return Ok(0);
    }
    if parent.provider != clock.provider {
        return Err("assigned clock parent provider is not connected to this mux");
    }
    let parents = match (clock.provider.start, clock.id) {
        (SYS_CRG, 103) => return Err("GMAC RX external parent identity is not normalized"),
        (SYS_CRG, 105) => [100, 101],
        (AON_CRG, 5) => {
            // AON 的 GTX 父源来自 SYS；本轮实际 DT 只指定同 AON 的 RMII_RTX。
            if parent.id == 4 {
                return Ok(1);
            }
            return Err("AON GMAC TX parent must name RMII_RTX or normalized SYS GTX clock");
        }
        (AON_CRG, 7) => return Err("GMAC RX external parent identity is not normalized"),
        _ => return Err("assigned parent targets a clock without a supported mux"),
    };
    parents
        .iter()
        .position(|id| *id == parent.id)
        .map(|index| index as u32)
        .ok_or("assigned clock parent id is not connected to this mux")
}

/// 父源和输出率分别落实到硬件，不因已有 gate 或请求重复而跳过配置。
fn configure_clock(clock: &ClockResource) -> Result<(), &'static str> {
    if fixed(clock.reference) {
        return Ok(());
    }
    if let Some(parent) = clock.parent {
        let index = parent_index(clock.reference, parent)?;
        update(
            clock.reference.provider,
            clock.reference.id as usize * 4,
            MUX_MASK,
            index << 24,
        )?;
    }
    if let Some(requested) = clock.rate {
        set_rate(clock.reference, requested, 0)?;
    }
    Ok(())
}

/// 有限的所选硬件树边；根输入没有独立 CRG 门寄存器。
#[derive(Clone, Copy)]
enum Parent {
    Clock(ResourceReference),
    Oscillator,
    Pll(u32),
    Rmii,
    Rgmii,
}

/// 在同一已验证 CRG 内构造硬件父节点，不生成新物理资源。
fn local(reference: ResourceReference, id: u32) -> Parent {
    Parent::Clock(ResourceReference {
        provider: reference.provider,
        id,
    })
}

/// 读取所选 mux 的实际值，保留不合法硬件选择的错误。
fn selection(reference: ResourceReference) -> Result<u32, &'static str> {
    let selection = (read(reference.provider, reference.id as usize * 4) & MUX_MASK) >> 24;
    if 1 < selection {
        return Err("JH7110 clock mux selection is outside its two parents");
    }
    Ok(selection)
}

/// 描述实际 SDIO/GMAC 到已有总线根的窄树，不支持其它外设的任意时钟遍历。
fn parent(reference: ResourceReference) -> Result<Parent, &'static str> {
    let result = match (reference.provider.start, reference.id) {
        (SYS_CRG, 5) => {
            if selection(reference)? == 0 {
                Parent::Oscillator
            } else {
                Parent::Pll(2)
            }
        }
        (SYS_CRG, 7) => local(reference, 5),
        (SYS_CRG, 8) => local(reference, 7),
        (SYS_CRG, 9 | 10 | 11) => local(reference, 8),
        (SYS_CRG, 12) => local(reference, 11),
        (SYS_CRG, 91 | 92 | 97) => local(reference, 9),
        (SYS_CRG, 93 | 94) => local(reference, 7),
        (SYS_CRG, 98) => local(reference, 8),
        (SYS_CRG, 99 | 100 | 108) => Parent::Pll(0),
        (SYS_CRG, 101) => Parent::Rmii,
        (SYS_CRG, 102 | 109 | 110) => local(reference, 99),
        (SYS_CRG, 103) => {
            if selection(reference)? == 0 {
                Parent::Rgmii
            } else {
                local(reference, 101)
            }
        }
        (SYS_CRG, 104) => local(reference, 103),
        (SYS_CRG, 105) => local(
            reference,
            if selection(reference)? == 0 { 100 } else { 101 },
        ),
        (SYS_CRG, 106) => local(reference, 105),
        (SYS_CRG, 107) => local(reference, 100),
        (SYS_CRG, 111) => local(reference, 108),
        (SYS_CRG, 112) => local(reference, 11),
        (AON_CRG, 2 | 3) => Parent::Clock(ResourceReference {
            provider: crg(SYS_CRG)?,
            id: 8,
        }),
        (AON_CRG, 4) => Parent::Rmii,
        (AON_CRG, 5) => {
            if selection(reference)? == 0 {
                Parent::Clock(ResourceReference {
                    provider: crg(SYS_CRG)?,
                    id: 108,
                })
            } else {
                local(reference, 4)
            }
        }
        (AON_CRG, 6) => local(reference, 5),
        (AON_CRG, 7) => {
            if selection(reference)? == 0 {
                Parent::Rgmii
            } else {
                local(reference, 4)
            }
        }
        (AON_CRG, 8) => local(reference, 7),
        _ => return Err("JH7110 clock tree edge is unsupported"),
    };
    Ok(result)
}

/// 返回真实整数分频的范围；零表示本节点不含分频字段。
fn divider_max(reference: ResourceReference) -> u32 {
    match (reference.provider.start, reference.id) {
        (SYS_CRG, 7) => 3,
        (SYS_CRG, 8) => 2,
        (SYS_CRG, 11) => 8,
        (SYS_CRG, 93 | 94 | 100 | 108) => 15,
        (SYS_CRG, 99) => 7,
        (SYS_CRG, 101) | (AON_CRG, 4) => 30,
        (SYS_CRG, 102 | 109 | 110) => 31,
        _ => 0,
    }
}

/// 只对有 gate 的节点写 bit31；反相节点递归到真实 gated parent，绝不伪造门位。
fn gated(reference: ResourceReference) -> bool {
    matches!((reference.provider.start, reference.id),
        (SYS_CRG, 9 | 10 | 12 | 91..=94 | 97 | 98 | 102 | 105 | 107..=112)
        | (AON_CRG, 2 | 3 | 5))
}

/// 检查实际分频不为零且位于该寄存器的硬件范围内。
fn divisor(reference: ResourceReference) -> Result<u64, &'static str> {
    let maximum = divider_max(reference);
    if maximum == 0 {
        return Ok(1);
    }
    let divisor = read(reference.provider, reference.id as usize * 4) & DIV_MASK;
    if divisor == 0 || maximum < divisor {
        return Err("JH7110 clock divider is zero or outside the hardware range");
    }
    Ok(u64::from(divisor))
}

/// 先启用实际父门再启用叶门；不重编程 PLL 或未请求的根频率。
fn enable_clock(reference: ResourceReference, depth: usize) -> Result<(), &'static str> {
    if fixed(reference) {
        return Ok(());
    }
    if 16 <= depth {
        return Err("JH7110 selected clock path exceeds its bounded depth");
    }
    if let Parent::Clock(upstream) = parent(reference)? {
        enable_clock(upstream, depth + 1)?;
    }
    divisor(reference)?;
    if gated(reference) {
        update(
            reference.provider,
            reference.id as usize * 4,
            ENABLE,
            ENABLE,
        )?;
    }
    Ok(())
}

/// 读取启动快照发布的真实晶振频率，缺失时不能把参考板频率当作输入。
fn oscillator_rate() -> Result<u64, &'static str> {
    boot::info()
        .board
        .clock_inputs
        .oscillator_hz
        .filter(|rate| *rate != 0)
        .ok_or("JH7110 oscillator clock-frequency is missing from normalized board inputs")
}

/// 读取实际 PLL 的整数或小数配置，计算原有 PLL 输出；不修改共享 PLL。
fn pll_rate(id: u32) -> Result<u64, &'static str> {
    let registers = boot::info().board.clock_inputs.pll_registers;
    check_region(registers, PLL_SYSCON, 0x38)?;
    let (pd, shift, fbdiv_offset, fbdiv_shift, frac_offset, prediv_offset) = match id {
        0 => (0x18, 24, 0x1c, 0, 0x20, 0x24),
        2 => (0x2c, 15, 0x2c, 17, 0x30, 0x34),
        _ => return Err("PLL id is outside the selected SDIO/GMAC clock paths"),
    };
    let mode = (read(registers, pd) >> shift) & 3;
    let fbdiv = (read(registers, fbdiv_offset) >> fbdiv_shift) & 0xfff;
    let fraction = read(registers, frac_offset);
    let prediv = read(registers, prediv_offset) & 0x3f;
    if prediv == 0 || fbdiv < 8 || !matches!(mode, 0 | 3) {
        return Err("JH7110 PLL mode or divider is invalid");
    }
    let fractional = if mode == 0 { fraction & 0x00ff_ffff } else { 0 };
    let postdiv = (fraction >> 28) & 3;
    let oscillator = u128::from(oscillator_rate()?);
    let multiplied =
        oscillator * u128::from(fbdiv) + oscillator * u128::from(fractional) / (1u128 << 24);
    let output = multiplied / (u128::from(prediv) << postdiv);
    u64::try_from(output)
        .ok()
        .filter(|rate| *rate != 0)
        .ok_or("JH7110 PLL output rate is invalid")
}

/// 区分板内部可计算输入与尚未规范化身份的外部 GMAC 引脚时钟。
fn parent_rate(upstream: Parent, depth: usize) -> Result<u64, &'static str> {
    match upstream {
        Parent::Clock(reference) => rate(reference, depth),
        Parent::Oscillator => oscillator_rate(),
        Parent::Pll(id) => pll_rate(id),
        Parent::Rmii => Err("GMAC RMII external input clock-frequency is not normalized"),
        Parent::Rgmii => Err("GMAC RGMII external input clock-frequency is not normalized"),
    }
}

/// 沿已选择的真实父边计算频率，gate 关闭不改变源频率但零分频保留错误。
fn rate(reference: ResourceReference, depth: usize) -> Result<u64, &'static str> {
    if 16 <= depth {
        return Err("JH7110 selected clock path exceeds its bounded depth");
    }
    let output = parent_rate(parent(reference)?, depth + 1)? / divisor(reference)?;
    if output == 0 {
        return Err("JH7110 clock output rate is zero");
    }
    Ok(output)
}

/// SDIO 请求允许整数分频量化，向上取整保证输出不超过上限；叶须读取实际输出率。
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

/// GMAC 链路时钟必须精确表达请求，不能量化后继续按原协商速率驱动 PHY。
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

/// 频率请求只改所选 SDIO/GMAC 叶分频，gate/反相节点沿同一真实父源传播。
fn set_rate(
    reference: ResourceReference,
    requested: u64,
    depth: usize,
) -> Result<(), &'static str> {
    if 16 <= depth {
        return Err("JH7110 selected clock path exceeds its bounded depth");
    }
    if matches!((reference.provider.start, reference.id),
        (SYS_CRG, 93 | 94 | 99..=102 | 108..=110) | (AON_CRG, 4))
    {
        let source = parent_rate(parent(reference)?, depth + 1)?;
        let maximum = divider_max(reference);
        let divider = if matches!((reference.provider.start, reference.id), (SYS_CRG, 93 | 94)) {
            sd_divider(source, requested, maximum)?
        } else {
            exact_divider(source, requested, maximum)?
        };
        return update(
            reference.provider,
            reference.id as usize * 4,
            DIV_MASK,
            divider,
        );
    }
    if matches!(
        (reference.provider.start, reference.id),
        (SYS_CRG, 104..=107 | 111) | (AON_CRG, 5 | 6 | 8)
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

/// 取得真实复位字和状态字；JH7110 asserted 为零、deasserted 为一。
fn reset_registers(reference: ResourceReference) -> Result<(usize, usize, u32), &'static str> {
    let (assert, status) = match (reference.provider.start, reference.id) {
        (SYS_CRG, 2 | 64..=67) => {
            check_region(reference.provider, SYS_CRG, 0x314)?;
            (0x2f8, 0x308)
        }
        (AON_CRG, 0..=2) => {
            check_region(reference.provider, AON_CRG, 0x40)?;
            (0x38, 0x3c)
        }
        _ => return Err("reset provider/id is outside the supported SDIO/GMAC/IOMUX bindings"),
    };
    let word = reference.id as usize / 32 * 4;
    Ok((assert + word, status + word, 1 << (reference.id % 32)))
}

/// 更新 assert 位后等待独立 status 位，不把写入完成当作复位完成。
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

/// 计算每个规范化电气字段的真实 pad 位；未请求的电气位保持原值。
fn pad_config(pin: &PinResource) -> Result<(u32, u32), &'static str> {
    let mut mask = 0;
    let mut value = 0;
    if pin.pull_up && pin.bias_disable {
        return Err("pin pull-up and bias-disable conflict");
    }
    if pin.pull_up || pin.bias_disable {
        mask |= (1 << 4) | (1 << 3);
        if pin.pull_up {
            value |= 1 << 3;
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
        mask |= 3 << 1;
        value |= encoded << 1;
    }
    for (setting, bit) in [(pin.input_enable, 0), (pin.schmitt_enable, 6)] {
        if let Some(enabled) = setting {
            mask |= 1 << bit;
            if enabled {
                value |= 1 << bit;
            }
        }
    }
    if let Some(slew) = pin.slew_rate {
        if 1 < slew {
            return Err("JH7110 pin slew-rate must be zero or one");
        }
        mask |= 1 << 5;
        value |= slew << 5;
    }
    Ok((mask, value))
}

/// GPIO function0 的选择位来自实际 SYS func_sel；专用 SD/GMAC pad 不制造虚构 mux 字段。
fn function_field(pin: u32) -> Option<(usize, u32)> {
    match pin {
        6 => Some((0x2b0, 0)),
        7..=9 => Some((0x2b0, 2 + 3 * (pin - 7))),
        10..=19 => Some((0x29c, 2 + 3 * (pin - 10))),
        20..=29 => Some((0x2a0, 3 * (pin - 20))),
        30..=35 => Some((0x2a4, 3 * (pin - 30))),
        36..=40 => Some((0x2a4, 17 + 3 * (pin - 36))),
        41..=51 => Some((0x2a8, 3 * (pin - 41))),
        52..=54 => Some((0x2ac, 2 * (pin - 52))),
        55..=63 => Some((0x2ac, 6 + 3 * (pin - 55))),
        82 => Some((0x29c, 0)),
        _ => None,
    }
}

/// 只接受 SDIO GPIO 信号及 SD/GMAC 固定功能；电气无寄存器的 GMAC pad 明确拒绝配置。
fn validate_pin(pin: &PinResource) -> Result<(), &'static str> {
    let number = pin.mux & 255;
    let function = (pin.mux >> 8) & 3;
    let (mask, _) = pad_config(pin)?;
    match pin.provider.start {
        SYS_PIN => {
            check_region(pin.provider, SYS_PIN, 0x2b4)?;
            if 88 < number || function != 0 {
                return Err("SYS pin mux is outside supported SDIO/GMAC function zero");
            }
            if number < 64 {
                let output = (pin.mux >> 16) & 255;
                let enable = (pin.mux >> 10) & 63;
                let input = pin.mux >> 24;
                if !matches!(output, 0 | 1 | 19 | 55 | 57..=66)
                    || !matches!(enable, 0 | 1 | 19..=27)
                    || !matches!(input, 11..=13 | 41..=53 | 255)
                    || 127 < output
                {
                    return Err("SYS GPIO mux does not name a supported SDIO signal");
                }
            } else if pin.mux != (number | (1 << 10)) {
                return Err("dedicated SDIO/GMAC pad has invalid GPIO routing fields");
            }
            if 75 <= number && mask != 0 {
                return Err("dedicated GMAC pads have no supported pad electrical registers");
            }
        }
        AON_PIN => {
            check_region(pin.provider, AON_PIN, 0x48)?;
            if !(6..=19).contains(&number) || pin.mux != (number | (1 << 10)) {
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

/// IOMUX 的共享复位只解除、不重新 assert，避免破坏此前已初始化设备的引脚状态。
fn prepare_pins(setup: &DeviceSetup) -> Result<(), &'static str> {
    if setup.pins[..setup.pin_count]
        .iter()
        .any(|pin| pin.provider.start == SYS_PIN)
    {
        let provider = crg(SYS_CRG)?;
        enable_clock(ResourceReference { provider, id: 112 }, 0)?;
        set_reset(ResourceReference { provider, id: 2 }, false)?;
    }
    if setup.pins[..setup.pin_count]
        .iter()
        .any(|pin| pin.provider.start == AON_PIN)
    {
        set_reset(
            ResourceReference {
                provider: crg(AON_CRG)?,
                id: 2,
            },
            false,
        )?;
    }
    Ok(())
}

/// 按一个 pin 的字节槽修改 DOUT/DOEN/GPI；专用 pad 仅有已验证的固定功能。
fn apply_pin(pin: &PinResource) -> Result<(), &'static str> {
    let number = pin.mux & 255;
    if pin.provider.start == SYS_PIN {
        if number < 64 {
            let offset = number as usize / 4 * 4;
            let shift = number % 4 * 8;
            let output = (pin.mux >> 16) & 255;
            let enable = (pin.mux >> 10) & 63;
            let input = pin.mux >> 24;
            update(pin.provider, 0x40 + offset, 0x7f << shift, output << shift)?;
            update(pin.provider, offset, 0x3f << shift, enable << shift)?;
            if input != 255 {
                let input_offset = input as usize / 4 * 4;
                let input_shift = input % 4 * 8;
                update(
                    pin.provider,
                    0x80 + input_offset,
                    0x7f << input_shift,
                    (number + 2) << input_shift,
                )?;
            }
        }
        if let Some((offset, shift)) = function_field(number) {
            update(pin.provider, offset, 3u32 << shift, 0)?;
        }
        let (mask, value) = pad_config(pin)?;
        if mask != 0 {
            update(pin.provider, 0x120 + number as usize * 4, mask, value)?;
        }
    }
    Ok(())
}

#[cfg(feature = "ktest")]
#[::ktest::suite(context = "Boot", tags("boot", "board-setup"), timeout_ms = 1000)]
mod checks {
    use super::*;

    /// 纯转换检验覆盖整数分频、pinmux 位域和所有电气开关，不访问任何板寄存器。
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
