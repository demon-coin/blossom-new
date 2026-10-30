//! JH7110 所选 SDIO1、GMAC 与缓存资源绑定，不解释另一份设备树。

use super::resources;
use super::{
    compatible, device_region, device_source, enabled, interrupt_parent, number, property,
    single_region, word, Fdt, Node,
};
use crate::boot::{BootInfo, GmacResources, GpioInputResource, MmcResources};

/// 从当前 OS DT 的真实绑定构造 VF2 设备快照。
pub(super) fn discover<'a>(
    tree: &'a Fdt<'a>,
    cpus: Node<'a>,
    uart: Node<'a>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    #[cfg(not(target_arch = "riscv64"))]
    {
        let _ = (tree, cpus, uart, info);
        Err("VisionFive2 requires RISC-V")
    }
    #[cfg(target_arch = "riscv64")]
    {
        let controller = interrupt_parent(tree, uart)?;
        info.interrupts.controller = single_region(tree, controller)?;
        info.interrupts.uart_source = resources::first_irq(tree, uart)?;
        super::discover_plic(tree, cpus, controller, info)?;
        if info.hardware_ids[..info.cpu_count]
            .iter()
            .any(|id| !(1..=4).contains(id))
        {
            return Err("FDT VF2 CPU is not an application U74 hart");
        }
        let ccache = unique(tree, "starfive,jh7110-ccache")?;
        let registers = single_region(tree, ccache)?;
        device_region(registers, info)?;
        info.board.ccache = registers;
        info.board.cache_line_size = number(ccache, "cache-block-size")? as usize;
        if info.board.cache_line_size != 64 || registers.end - registers.start < 0x208 {
            return Err("FDT JH7110 ccache geometry is unsupported");
        }
        info.board.clock_inputs = resources::jh7110_clock_inputs(tree, uart)?;
        info.board.mmc = discover_mmc(tree, controller, info)?;
        info.board.gmac = discover_gmac(tree, controller, info)?;
        let mut rtcs = tree
            .all_nodes()
            .filter(|node| enabled(*node) && compatible(*node, "starfive,jh7110-rtc"));
        if let Some(rtc) = rtcs.next() {
            if rtcs.next().is_some() {
                return Err("FDT JH7110 RTC is duplicated");
            }
            info.rtc = single_region(tree, rtc)?;
            resources::require_provider(
                tree,
                rtc,
                "clocks",
                "#clock-cells",
                "starfive,jh7110-aoncrg",
            )?;
            resources::require_provider(
                tree,
                rtc,
                "resets",
                "#reset-cells",
                "starfive,jh7110-aoncrg",
            )?;
            info.board.rtc_setup = resources::setup(tree, rtc)?;
        }
        Ok(())
    }
}

/// 只为明确选择且启用的 SDIO1 解析并登记资源，绑定错误保留原失败。
fn discover_mmc<'a>(
    tree: &'a Fdt<'a>,
    controller: Node<'a>,
    info: &mut BootInfo,
) -> Result<Option<MmcResources>, &'static str> {
    let Some(mmc) = resources::optional_alias(tree, "mmc1")? else {
        return Ok(None);
    };
    if !compatible(mmc, "starfive,jh7110-mmc") {
        return Err("FDT mmc1 is not a JH7110 DW-MMC");
    }
    let registers = single_region(tree, mmc)?;
    device_region(registers, info)?;
    let interrupt_source = resources::first_irq(tree, mmc)?;
    require_plic(tree, mmc, controller)?;
    device_source(interrupt_source, info)?;
    let setup = resources::setup(tree, mmc)?;
    resources::require_names(mmc, "clock-names", &["biu", "ciu"])?;
    resources::require_names(mmc, "reset-names", &["reset"])?;
    let (_, syscrg) = resources::named_clock(tree, mmc, "biu")?;
    if !compatible(syscrg, "starfive,jh7110-syscrg") {
        return Err("FDT selected SDIO bus clock does not use the system CRG");
    }
    if setup.clocks.count != 2 || setup.reset_count != 1 || setup.pin_count != 6 {
        return Err("FDT SDIO1 clock, reset or pin resources are incomplete");
    }
    let sysreg = property(mmc, "starfive,sysreg")?;
    if sysreg.len() != 16 {
        return Err("FDT MMC sysreg binding is invalid");
    }
    let syscon = tree
        .find_phandle(word(sysreg, 0)?)
        .ok_or("FDT MMC syscon is missing")?;
    let resource = MmcResources {
        registers,
        interrupt_source,
        setup,
        syscon: single_region(tree, syscon)?,
        sysreg: [word(sysreg, 4)?, word(sysreg, 8)?, word(sysreg, 12)?],
        bus_width: number(mmc, "bus-width")?,
        max_frequency: number(mmc, "max-frequency")?,
        fifo_depth: resources::optional_number(mmc, "fifo-depth")?,
        data_offset: resources::optional_number(mmc, "data-addr")?.unwrap_or(0),
        card_detect: card_detect_gpio(tree, mmc, info)?,
        cd_inverted: mmc.property("cd-inverted").is_some(),
        disable_wp: mmc.property("disable-wp").is_some(),
        post_power_on_delay_ms: resources::optional_number(mmc, "post-power-on-delay-ms")?
            .unwrap_or(10),
        dma_coherent: resources::coherent(tree, mmc, true)?,
        dma_mask: u32::MAX as u64,
    };
    if resource.bus_width != 4
        || resource.max_frequency == 0
        || resource
            .fifo_depth
            .is_some_and(|depth| depth < 2 || 4096 < depth)
    {
        return Err("FDT SDIO1 card interface is unsupported");
    }
    Ok(Some(resource))
}

/// 保存独立卡检测输入；已声明的 GPIO 损坏时失败，不改走控制器的卡检测位。
fn card_detect_gpio<'a>(
    tree: &'a Fdt<'a>,
    mmc: Node<'a>,
    info: &BootInfo,
) -> Result<Option<GpioInputResource>, &'static str> {
    let Some(raw) = mmc.property("cd-gpios") else {
        return Ok(None);
    };
    if raw.value.len() != 12 {
        return Err("FDT SDIO card-detect GPIO must contain one two-cell reference");
    }
    let gpio = tree
        .find_phandle(word(raw.value, 0)?)
        .ok_or("FDT SDIO card-detect GPIO provider is missing")?;
    if !enabled(gpio)
        || !compatible(gpio, "starfive,jh7110-sys-pinctrl")
        || gpio.property("gpio-controller").is_none()
        || number(gpio, "#gpio-cells")? != 2
    {
        return Err("FDT SDIO card-detect provider is not a supported SYS GPIO controller");
    }
    let pin = word(raw.value, 4)?;
    let flags = word(raw.value, 8)?;
    if 64 <= pin || flags & !1 != 0 {
        return Err("FDT SDIO card-detect GPIO pin or flags are unsupported");
    }
    let provider = single_region(tree, gpio)?;
    device_region(provider, info)?;
    Ok(Some(GpioInputResource {
        provider,
        pin,
        active_low: flags & 1 != 0,
    }))
}

/// 只为两个真实 alias 中唯一启用的 GMAC 登记资源，未选网口不取得运行期拥有。
fn discover_gmac<'a>(
    tree: &'a Fdt<'a>,
    controller: Node<'a>,
    info: &mut BootInfo,
) -> Result<Option<GmacResources>, &'static str> {
    let mac = match (
        resources::optional_alias(tree, "ethernet0")?,
        resources::optional_alias(tree, "ethernet1")?,
    ) {
        (None, None) => return Ok(None),
        (Some(mac), None) | (None, Some(mac)) => mac,
        (Some(_), Some(_)) => return Err("FDT VF2 requires at most one enabled GMAC"),
    };
    if !compatible(mac, "starfive,jh7110-dwmac") {
        return Err("FDT selected Ethernet device is not a JH7110 GMAC");
    }
    require_plic(tree, mac, controller)?;
    let mut resource = resources::gmac(tree, mac, true)?;
    resource.registers = single_region(tree, mac)?;
    device_region(resource.registers, info)?;
    resource.interrupt_source = resources::first_irq(tree, mac)?;
    device_source(resource.interrupt_source, info)?;
    let sysreg = property(mac, "starfive,syscon")?;
    if sysreg.len() != 12 {
        return Err("FDT GMAC syscon binding is invalid");
    }
    let syscon = tree
        .find_phandle(word(sysreg, 0)?)
        .ok_or("FDT GMAC syscon is missing")?;
    resource.syscon = single_region(tree, syscon)?;
    resource.sysreg = [word(sysreg, 4)?, word(sysreg, 8)?];
    resources::require_names(
        mac,
        "clock-names",
        &["stmmaceth", "pclk", "ptp_ref", "tx", "gtx"],
    )?;
    resources::require_names(mac, "reset-names", &["stmmaceth", "ahb"])?;
    if resource.setup.clocks.count != 5 || resource.setup.reset_count != 2 {
        return Err("FDT GMAC clocks or resets are incomplete");
    }
    Ok(Some(resource))
}

/// 取得必须唯一的启用绑定，重复硬件不得静默择一。
fn unique<'a>(tree: &'a Fdt<'a>, binding: &str) -> Result<Node<'a>, &'static str> {
    let mut nodes = tree
        .all_nodes()
        .filter(|node| enabled(*node) && compatible(*node, binding));
    let node = nodes
        .next()
        .ok_or("FDT required VF2 controller is missing")?;
    if nodes.next().is_some() {
        return Err("FDT required VF2 controller is duplicated");
    }
    Ok(node)
}

/// 所选设备必须连接同一实际 PLIC，不能套另一个域的 source。
fn require_plic<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    controller: Node<'a>,
) -> Result<(), &'static str> {
    let actual = interrupt_parent(tree, node)?;
    if actual.name.as_ptr() != controller.name.as_ptr() {
        return Err("FDT VF2 device uses a different interrupt controller");
    }
    Ok(())
}
