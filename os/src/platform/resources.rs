//! 所选设备的标准属性在唯一 FDT 发现期间转成有界值。

use super::{
    cells, compatible, cstr, enabled, interrupt_parent, number, parent, property, single_region,
    word, Fdt, Node,
};
use crate::boot::{
    BoardClockInputs, BootInfo, ClockResource, DeviceClocks, DeviceSetup, GmacResources, PhyMode,
    PinResource, Platform, Region, ResourceReference,
};
use core::convert::{TryFrom, TryInto};

/// 取得唯一启用的具名路径或 alias，不猜另一设备。
pub(super) fn alias<'a>(tree: &'a Fdt<'a>, name: &str) -> Result<Node<'a>, &'static str> {
    let aliases = tree
        .find_node("/aliases")
        .ok_or("FDT aliases node is missing")?;
    let path = cstr(property(aliases, name)?)?;
    let node = tree.find_node(path).ok_or("FDT alias target is missing")?;
    if !enabled(node) {
        return Err("FDT selected alias target is disabled");
    }
    Ok(node)
}

/// 只选择已声明且启用的 alias；明确禁用不取资源，损坏声明仍返回错误。
pub(super) fn optional_alias<'a>(
    tree: &'a Fdt<'a>,
    name: &str,
) -> Result<Option<Node<'a>>, &'static str> {
    let Some(aliases) = tree.find_node("/aliases") else {
        return Ok(None);
    };
    let Some(value) = aliases.property(name) else {
        return Ok(None);
    };
    let path = cstr(value.value)?;
    let node = tree.find_node(path).ok_or("FDT alias target is missing")?;
    if node.property("status").and_then(|status| status.as_str()) == Some("disabled") {
        return Ok(None);
    }
    if !enabled(node) {
        return Err("FDT selected alias target is disabled");
    }
    Ok(Some(node))
}

/// 按 chosen 的 stdout-path 解析控制台，包括标准 alias。
pub(super) fn stdout_uart<'a>(
    tree: &'a Fdt<'a>,
    chosen: Node<'a>,
) -> Result<Node<'a>, &'static str> {
    let stdout = cstr(property(chosen, "stdout-path")?)?;
    let path = stdout.split(':').next().ok_or("FDT stdout-path is empty")?;
    let uart = if path.starts_with('/') {
        tree.find_node(path).ok_or("FDT stdout UART is missing")?
    } else {
        alias(tree, path)?
    };
    if !enabled(uart) || !(compatible(uart, "ns16550a") || compatible(uart, "snps,dw-apb-uart")) {
        return Err("FDT stdout UART binding is unsupported");
    }
    Ok(uart)
}

/// 读取存在时必须为单 cell 的可选属性。
pub(super) fn optional_number(node: Node<'_>, name: &str) -> Result<Option<u32>, &'static str> {
    node.property(name).map(|_| number(node, name)).transpose()
}

/// 按最近的显式属性判断 DMA 一致性，无声明时沿 DMA 父链使用调用方的架构默认值。
pub(super) fn coherent<'a>(
    tree: &'a Fdt<'a>,
    mut node: Node<'a>,
    default_coherent: bool,
) -> Result<bool, &'static str> {
    // DMA phandle 可以跨越结构祖先；超过真实节点总数仍未结束说明父链存在环。
    for _ in tree.all_nodes() {
        if node.property("dma-coherent").is_some() {
            return Ok(true);
        }
        if node.property("dma-noncoherent").is_some() {
            return Ok(false);
        }
        let Some(bus) = dma_parent(tree, node)? else {
            return Ok(default_coherent);
        };
        node = bus;
    }
    Err("FDT DMA parent chain contains a cycle")
}

/// 只解析 dma-mem 选中的 interconnect 父节点，不把 provider 当作 clock/reset 或 MMIO。
fn dma_parent<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<Option<Node<'a>>, &'static str> {
    let Some(names) = node.property("interconnect-names") else {
        return Ok(parent(tree, node));
    };
    let mut names = names.value;
    let mut index = 0usize;
    loop {
        if names.is_empty() {
            return Ok(parent(tree, node));
        }
        let name = cstr(names)?;
        if name == "dma-mem" {
            break;
        }
        names = &names[name.len() + 1..];
        index += 1;
    }

    let raw = property(node, "interconnects")?;
    let mut at = 0;
    loop {
        let handle = word(raw, at)?;
        at += 4;
        // 空 phandle 是零参数占位；只有未选项可以跳过，选中的 DMA 父必须存在。
        if handle == 0 {
            if index == 0 {
                return Err("FDT selected DMA interconnect phandle is missing");
            }
            index -= 1;
            continue;
        }
        let provider = tree
            .find_phandle(handle)
            .ok_or("FDT DMA interconnect provider is missing")?;
        let count = usize::try_from(number(provider, "#interconnect-cells")?)
            .map_err(|_| "FDT DMA interconnect cell count is not representable")?;
        let end = count
            .checked_mul(4)
            .and_then(|size| at.checked_add(size))
            .ok_or("FDT DMA interconnect argument length overflows")?;
        if raw.len() < end {
            return Err("FDT DMA interconnect arguments are truncated");
        }
        if index == 0 {
            return Ok(Some(provider));
        }
        at = end;
        index -= 1;
    }
}

/// 读取 clock/reset 的零或单参数 provider，固定时钟不伪造 MMIO。
fn reference<'a>(
    tree: &'a Fdt<'a>,
    bytes: &[u8],
    at: &mut usize,
    count_name: &str,
) -> Result<(ResourceReference, Node<'a>), &'static str> {
    let handle = word(bytes, *at)?;
    *at += 4;
    let provider = tree
        .find_phandle(handle)
        .ok_or("FDT resource provider phandle is missing")?;
    if !enabled(provider) {
        return Err("FDT resource provider is disabled");
    }
    let count = number(provider, count_name)?;
    if 1 < count {
        return Err("FDT resource provider has unsupported argument cells");
    }
    let id = if count == 1 {
        let value = word(bytes, *at)?;
        *at += 4;
        value
    } else {
        0
    };
    let registers = provider_region(tree, provider)?;
    Ok((
        ResourceReference {
            provider: registers,
            id,
        },
        provider,
    ))
}

/// PLL 与父 syscon 共用寄存器窗口；无 reg 的绑定不得生成空窗口。
fn provider_region<'a>(tree: &'a Fdt<'a>, provider: Node<'a>) -> Result<Region, &'static str> {
    if compatible(provider, "fixed-clock") {
        return Ok(Region::default());
    }
    if compatible(provider, "starfive,jh7110-pll") && provider.property("reg").is_none() {
        let syscon = parent(tree, provider).ok_or("FDT JH7110 PLL syscon parent is missing")?;
        if !enabled(syscon)
            || !compatible(syscon, "starfive,jh7110-sys-syscon")
            || !compatible(syscon, "syscon")
        {
            return Err("FDT JH7110 PLL parent is not its system syscon");
        }
        return single_region(tree, syscon);
    }
    single_region(tree, provider)
}

/// 按 clock-names 与 clocks 的相同索引取得实际输入，禁止以遍历首项代替具名父源。
pub(super) fn named_clock<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    wanted: &str,
) -> Result<(ResourceReference, Node<'a>), &'static str> {
    let clocks = property(node, "clocks")?;
    let mut names = property(node, "clock-names")?;
    let mut at = 0;
    while !names.is_empty() {
        let name = cstr(names)?;
        names = &names[name.len() + 1..];
        let input = reference(tree, clocks, &mut at, "#clock-cells")?;
        if name == wanted {
            return Ok(input);
        }
    }
    Err("FDT required named clock input is missing")
}

/// 沿所选 UART 的实际 SYS CRG 父链保存共享 PLL 输入，频率仍由设备动作读取。
pub(super) fn jh7110_clock_inputs<'a>(
    tree: &'a Fdt<'a>,
    uart: Node<'a>,
) -> Result<BoardClockInputs, &'static str> {
    let (_, syscrg) = named_clock(tree, uart, "baudclk")?;
    if !compatible(syscrg, "starfive,jh7110-syscrg") {
        return Err("FDT selected UART clock does not use the system CRG");
    }
    let (pll_reference, pll) = named_clock(tree, syscrg, "pll0_out")?;
    if !compatible(pll, "starfive,jh7110-pll") || pll_reference.id != 0 {
        return Err("FDT system PLL0 input binding is invalid");
    }
    for (name, id) in [("pll1_out", 1), ("pll2_out", 2)] {
        let (output, provider) = named_clock(tree, syscrg, name)?;
        if provider.name.as_ptr() != pll.name.as_ptr() || output.id != id {
            return Err("FDT system PLL outputs do not share the selected PLL provider");
        }
    }
    let raw = property(pll, "clocks")?;
    let mut at = 0;
    let (_, oscillator) = reference(tree, raw, &mut at, "#clock-cells")?;
    if at != raw.len()
        || !compatible(oscillator, "fixed-clock")
        || number(oscillator, "#clock-cells")? != 0
    {
        return Err("FDT JH7110 PLL requires one fixed oscillator input");
    }
    let (_, system_oscillator) = named_clock(tree, syscrg, "osc")?;
    if system_oscillator.name.as_ptr() != oscillator.name.as_ptr() {
        return Err("FDT system CRG and PLL oscillator inputs differ");
    }
    let oscillator_hz = number(oscillator, "clock-frequency")? as u64;
    if oscillator_hz == 0 {
        return Err("FDT JH7110 oscillator frequency is zero");
    }
    Ok(BoardClockInputs {
        oscillator_hz: Some(oscillator_hz),
        pll_registers: pll_reference.provider,
    })
}

/// 解析设备实际时钟与该设备的 assigned-clocks，不创建通用时钟解释器。
pub(super) fn clocks<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<DeviceClocks, &'static str> {
    let mut result = DeviceClocks::default();
    let Some(raw) = node.property("clocks") else {
        return Ok(result);
    };
    let mut at = 0;
    while at < raw.value.len() {
        if result.count == result.entries.len() {
            return Err("FDT device clock capacity exceeded");
        }
        let (reference, provider) = reference(tree, raw.value, &mut at, "#clock-cells")?;
        let rate = optional_number(provider, "clock-frequency")?.map(u64::from);
        result.entries[result.count] = ClockResource {
            reference,
            parent: None,
            rate,
        };
        result.count += 1;
    }
    if let Some(assigned) = node.property("assigned-clocks") {
        let rates = node.property("assigned-clock-rates");
        let parents = node.property("assigned-clock-parents");
        let mut at = 0;
        let mut parent_at = 0;
        let mut index = 0;
        while at < assigned.value.len() {
            let (assigned_ref, _) = reference(tree, assigned.value, &mut at, "#clock-cells")?;
            let parent = if let Some(parents) = parents {
                if parent_at < parents.value.len() {
                    if word(parents.value, parent_at)? == 0 {
                        parent_at += 4;
                        None
                    } else {
                        Some(reference(tree, parents.value, &mut parent_at, "#clock-cells")?.0)
                    }
                } else {
                    None
                }
            } else {
                None
            };
            let rate = rates
                .map(|r| word(r.value, index * 4))
                .transpose()?
                .filter(|rate| *rate != 0)
                .map(u64::from);
            if result.assigned_count == result.assigned.len() {
                return Err("FDT assigned clock capacity exceeded");
            }
            result.assigned[result.assigned_count] = ClockResource {
                reference: assigned_ref,
                parent,
                rate,
            };
            result.assigned_count += 1;
            if let Some(clock) = result.entries[..result.count].iter_mut().find(|clock| {
                clock.reference.provider == assigned_ref.provider
                    && clock.reference.id == assigned_ref.id
            }) {
                clock.parent = parent;
                if rate.is_some() {
                    clock.rate = rate;
                }
            }
            index += 1;
        }
    }
    Ok(result)
}

/// 保存设备需要的复位及 default 引脚组，电气设置保持每个实际 pin 的归属。
pub(super) fn setup<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<DeviceSetup, &'static str> {
    let mut result = DeviceSetup {
        clocks: clocks(tree, node)?,
        ..DeviceSetup::default()
    };
    if let Some(raw) = node.property("resets") {
        let mut at = 0;
        while at < raw.value.len() {
            if result.reset_count == result.resets.len() {
                return Err("FDT reset capacity exceeded");
            }
            result.resets[result.reset_count] =
                reference(tree, raw.value, &mut at, "#reset-cells")?.0;
            result.reset_count += 1;
        }
    }
    if let Some(raw) = node.property("pinctrl-0") {
        if raw.value.len() % 4 != 0 {
            return Err("FDT pinctrl phandles are truncated");
        }
        for handle in raw.value.chunks_exact(4) {
            let group = tree
                .find_phandle(word(handle, 0)?)
                .ok_or("FDT pinctrl group is missing")?;
            let provider = parent(tree, group).ok_or("FDT pinctrl provider is missing")?;
            let registers = single_region(tree, provider)?;
            for pins in group.children() {
                let Some(muxes) = pins.property("pinmux") else {
                    continue;
                };
                if muxes.value.len() % 4 != 0 {
                    return Err("FDT pinmux values are truncated");
                }
                for mux in muxes.value.chunks_exact(4) {
                    if result.pin_count == result.pins.len() {
                        return Err("FDT pin capacity exceeded");
                    }
                    result.pins[result.pin_count] = PinResource {
                        provider: registers,
                        mux: word(mux, 0)?,
                        pull_up: pins.property("bias-pull-up").is_some(),
                        bias_disable: pins.property("bias-disable").is_some(),
                        drive_strength: optional_number(pins, "drive-strength")?,
                        input_enable: boolean_setting(pins, "input-enable", "input-disable")?,
                        schmitt_enable: boolean_setting(
                            pins,
                            "input-schmitt-enable",
                            "input-schmitt-disable",
                        )?,
                        slew_rate: optional_number(pins, "slew-rate")?,
                    };
                    result.pin_count += 1;
                }
            }
        }
    }
    Ok(result)
}

/// 成对布尔电气属性不得同时出现。
fn boolean_setting(node: Node<'_>, yes: &str, no: &str) -> Result<Option<bool>, &'static str> {
    match (node.property(yes).is_some(), node.property(no).is_some()) {
        (true, true) => Err("FDT pin settings conflict"),
        (true, false) => Ok(Some(true)),
        (false, true) => Ok(Some(false)),
        (false, false) => Ok(None),
    }
}

/// 规范 UART 访问、波特率和真实输入时钟；QEMU 保留其模拟串口 baudbase 语义。
pub(super) fn discover_uart<'a>(
    tree: &'a Fdt<'a>,
    chosen: Node<'a>,
    uart: Node<'a>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    let width = optional_number(uart, "reg-io-width")?.unwrap_or(1);
    let shift = optional_number(uart, "reg-shift")?.unwrap_or(0);
    if !matches!(width, 1 | 4) || 2 < shift {
        return Err("FDT UART register access geometry is unsupported");
    }
    let mut uart_clocks = clocks(tree, uart)?;
    let mut frequency = optional_number(uart, "clock-frequency")?.map(u64::from);
    if info.platform == Platform::VisionFive2 {
        if width != 4
            || shift != 2
            || uart_clocks.count != 2
            || uart_clocks.entries[0].reference.id != 146
        {
            return Err("FDT VF2 UART0 clocks or register geometry differ from its binding");
        }
        require_names(uart, "clock-names", &["baudclk", "apb_pclk"])?;
        let clock_provider = tree
            .find_phandle(word(property(uart, "clocks")?, 0)?)
            .ok_or("FDT UART clock provider is missing")?;
        let raw = property(clock_provider, "clocks")?;
        let mut at = 0;
        if cstr(property(clock_provider, "clock-names")?)? != "osc" {
            return Err("FDT UART0 clock provider has no first oscillator input");
        }
        let (osc, oscillator) = reference(tree, raw, &mut at, "#clock-cells")?;
        if !compatible(oscillator, "fixed-clock") {
            return Err("FDT UART oscillator is not a fixed clock");
        }
        frequency = Some(number(oscillator, "clock-frequency")? as u64);
        uart_clocks.entries[0].parent = Some(osc);
        uart_clocks.entries[0].rate = frequency;
    }
    let stdout = cstr(property(chosen, "stdout-path")?)?;
    let baud = if let Some(options) = stdout.split_once(':').map(|(_, options)| options) {
        let mut baud = 0u32;
        for byte in options.bytes().take_while(u8::is_ascii_digit) {
            baud = baud
                .checked_mul(10)
                .and_then(|value| value.checked_add((byte - b'0') as u32))
                .ok_or("FDT UART baud overflows")?;
        }
        if baud == 0 {
            return Err("FDT UART baud is absent from stdout options");
        }
        baud
    } else if let Some(baud) = optional_number(uart, "current-speed")? {
        baud
    } else if info.platform == Platform::QemuVirt {
        115200
    } else {
        return Err("FDT board UART baud rate is missing");
    };
    if baud == 0 {
        return Err("FDT UART baud rate is zero");
    }
    if frequency == Some(0) || (info.platform != Platform::QemuVirt && frequency.is_none()) {
        return Err("FDT UART input clock is missing or zero");
    }
    info.uart = crate::boot::UartResources {
        reg_io_width: width,
        reg_shift: shift,
        clock_frequency: frequency,
        clocks: uart_clocks,
        baud,
        dw_apb: compatible(uart, "snps,dw-apb-uart"),
    };
    Ok(())
}

/// 保存 PHY 和 MAC 的共同真实属性，并按调用方的架构默认值解析 DMA 一致性。
pub(super) fn gmac<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    default_coherent: bool,
) -> Result<GmacResources, &'static str> {
    let phy = tree
        .find_phandle(number(node, "phy-handle")?)
        .ok_or("FDT GMAC PHY is missing")?;
    let address = number(phy, "reg")?;
    if 32 <= address {
        return Err("FDT GMAC PHY address exceeds Clause 22");
    }
    let mode = match cstr(property(node, "phy-mode")?)? {
        "rgmii" => PhyMode::Rgmii,
        "rgmii-id" => PhyMode::RgmiiId,
        "rgmii-rxid" => PhyMode::RgmiiRxid,
        "rgmii-txid" => PhyMode::RgmiiTxid,
        "rmii" => PhyMode::Rmii,
        _ => return Err("FDT GMAC PHY mode is unsupported"),
    };
    let mac = node
        .property("mac-address")
        .or_else(|| node.property("local-mac-address"));
    let mac_address = mac
        .map(|p| {
            let value: [u8; 6] = p
                .value
                .try_into()
                .map_err(|_| "FDT MAC address length is invalid")?;
            if value[0] & 1 != 0 || value == [0; 6] {
                return Err("FDT MAC address is not unicast");
            }
            Ok(value)
        })
        .transpose()?;
    let dma_mask = node
        .property("dma-mask")
        .map(|p| cells(p.value).map(|value| value as u64))
        .transpose()?
        .unwrap_or(u32::MAX as u64);
    if dma_mask == 0 || dma_mask & dma_mask.wrapping_add(1) != 0 {
        return Err("FDT GMAC DMA mask is not a contiguous address range");
    }
    Ok(GmacResources {
        setup: setup(tree, node)?,
        mac_address,
        phy_address: address,
        phy_mode: mode,
        rx_delay_ps: optional_number(phy, "rx-internal-delay-ps")?,
        tx_delay_ps: optional_number(phy, "tx-internal-delay-ps")?,
        phy_tx_clock_adjust: phy.property("motorcomm,tx-clk-adj-enabled").is_some(),
        phy_tx_inverted: [
            phy.property("motorcomm,tx-clk-10-inverted").is_some(),
            phy.property("motorcomm,tx-clk-100-inverted").is_some(),
            phy.property("motorcomm,tx-clk-1000-inverted").is_some(),
        ],
        phy_clock_drive_ua: optional_number(phy, "motorcomm,rx-clk-drv-microamp")?,
        phy_data_drive_ua: optional_number(phy, "motorcomm,rx-data-drv-microamp")?,
        tx_use_rgmii_clock: node.property("starfive,tx-use-rgmii-clk").is_some(),
        dma_coherent: coherent(tree, node, default_coherent)?,
        dma_mask: dma_mask.min(u32::MAX as u64),
        ..GmacResources::default()
    })
}

/// 从一个所选节点取得控制器绑定的首个中断，附加事件保留在设备硬件内处理。
pub(super) fn first_irq<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<u32, &'static str> {
    let controller = interrupt_parent(tree, node)?;
    let raw = property(node, "interrupts")?;
    let width = number(controller, "#interrupt-cells")? as usize * 4;
    if width == 0 || raw.len() % width != 0 {
        return Err("FDT device interrupt cells are truncated");
    }
    word(raw, 0)
}

/// 验证具名资源的固定绑定顺序，驱动无需重新解释字符串属性。
pub(super) fn require_names(
    node: Node<'_>,
    property_name: &str,
    names: &[&str],
) -> Result<(), &'static str> {
    let mut raw = property(node, property_name)?;
    for expected in names {
        let name = cstr(raw)?;
        if name != *expected {
            return Err("FDT selected device resource names differ from the binding");
        }
        raw = &raw[name.len() + 1..];
    }
    if !raw.is_empty() {
        return Err("FDT selected device resource names contain extra entries");
    }
    Ok(())
}

/// 对已选择设备的时钟或复位引用验证真实控制器类型，数值 ID 不能跨 provider 使用。
pub(super) fn require_provider<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    property_name: &str,
    cell_name: &str,
    binding: &str,
) -> Result<(), &'static str> {
    let raw = property(node, property_name)?;
    if raw.is_empty() {
        return Err("FDT required provider resource list is empty");
    }
    let mut at = 0;
    while at < raw.len() {
        let (_, provider) = reference(tree, raw, &mut at, cell_name)?;
        if !compatible(provider, binding) {
            return Err("FDT device resource provider has an incompatible binding");
        }
    }
    Ok(())
}
