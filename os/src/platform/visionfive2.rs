//! StarFive VisionFive 2（JH7110 SoC）开发板的设备树（FDT）资源发现与校验模块。
//!
//! # 模块职责与架构定位
//! 本模块位于平台抽象层（`crate::platform`）下，由上层引导资源发现入口（`platform::discover`）
//! 在识别出当前平台为 `Platform::VisionFive2` 时调用。它负责从当前传给操作系统的扁平设备树（FDT）
//! 中提取 JH7110 SoC 运行所需的核心控制器与板载外设资源，并将校验后的物理地址区间、中断号、
//! 时钟/复位拓扑写入 [`BootInfo`]，供后续架构初始化与设备驱动（PLIC、SiFive CCache、DW-MMC、
//! DWMAC、RTC、SYSCRG/AONCRG）直接使用。
//!
//! # 核心设计与硬件拓扑
//! VisionFive 2 开发板搭载的 JH7110 SoC 包含 1 个 S7 监控核（Hart 0，无 MMU）与 4 个 U74 应用核
//! （Hart 1..=4）。本模块在设备树解析阶段执行严格的硬件绑定校验，核心外设拓扑与设备树属性布局如下：
//!
//! ```text
//!                        +---------------------------------------+
//!                        |      CPU Cores (U74 Hart 1..=4)       |
//!                        +---------------------------------------+
//!                                            ^
//!                                            | 外部中断目标 (S-Mode Context)
//!                        +---------------------------------------+
//!                        |       PLIC (riscv,plic0 / plic)       |
//!                        +---------------------------------------+
//!                           ^                ^                ^
//!              UART IRQ     |    SDIO1 IRQ   |    GMAC1 IRQ   |
//!           +---------------+   +------------+   +------------+
//!           |                   |                |
//!   +---------------+   +---------------+   +---------------+   +---------------+
//!   |  DW-APB UART  |   | SDIO1 (mmc1)  |   | GMAC1 (eth1)  |   |  JH7110 RTC   |
//!   +---------------+   +---------------+   +---------------+   +---------------+
//!           |                   |                |                      |
//!           +-------------------+----------------+                      |
//!                               | 时钟 / 复位 / Syscon / 引脚复用        | 时钟 / 复位
//!                               v                                       v
//!               +-------------------------------+       +-------------------------------+
//!               | SYSCRG / SYS-SYSCON / PINCTRL |       |            AONCRG             |
//!               +-------------------------------+       +-------------------------------+
//!
//!   starfive,sysreg (MMC, 4 cells):  [ <syscon_phandle>, <offset>, <shift>, <mask> ]
//!   starfive,syscon (GMAC, 3 cells): [ <syscon_phandle>, <offset>, <shift> ]
//!   cd-gpios        (GPIO, 3 cells): [ <gpio_phandle>,   <pin>,    <flags> ]
//! ```
//!
//! 关键资源绑定约束：
//! 1. **处理器核过滤**：仅允许 Hart ID 位于 `1..=4` 的 U74 应用核参与 SMP 引导，排斥 Hart 0（S7 监控核）。
//! 2. **一致性缓存控制器（SiFive CCache）**：校验缓存行大小必须为 64 字节，且寄存器窗口覆盖 `FLUSH64`
//!    寄存器，用于非一致性 DMA 缓冲区的物理地址刷回与失效。
//! 3. **按别名认领板载外设**：仅解析设备树 `/aliases` 中显式选定且处于启用状态的 `mmc1`（SDIO1，
//!    对应板载 microSD 卡槽）与 `ethernet1`（GMAC1），同一 SoC 上未选用的另一路控制器不会混入运行期资源。
//!
//! # 锁获取顺序与并发契约
//! 本模块仅在引导早期单核（BSP）串行初始化阶段执行，只读写栈上或尚未公开的 [`BootInfo`] 结构，
//! 不涉及任何运行期自旋锁或睡眠锁。

#[cfg(target_arch = "riscv64")]
use super::region_at;
#[cfg(target_arch = "riscv64")]
use super::resources;
#[cfg(target_arch = "riscv64")]
use super::{
    compatible, device_region, device_source, enabled, interrupt_parent, number, property,
    single_region, word, FDT_CELL_BYTES,
};
use super::{Fdt, Node};
use crate::boot::BootInfo;
#[cfg(target_arch = "riscv64")]
use crate::boot::{GmacResources, GpioInputResource, MmcResources};
#[cfg(target_arch = "riscv64")]
use crate::drivers::sdmmc::{FIFO_MAX_DEPTH, FIFO_MIN_DEPTH};

#[cfg(target_arch = "riscv64")]
const GPIO_ACTIVE_LOW: u32 = 1;
#[cfg(target_arch = "riscv64")]
const MMC_POST_POWER_ON_DELAY_MS: u32 = 10;

/// 从设备树中发现并校验 VisionFive 2 开发板的板级核心控制器与外设资源。
///
/// 调用上下文：引导早期单核（BSP）环境，由 `platform::discover` 在完成基础 CPU 与串口节点定位后调用。
///
/// 参数：
/// - `tree`：当前操作系统的扁平设备树（FDT）只读视图。
/// - `cpus`：设备树中的 `/cpus` 节点，用于解析 PLIC 中断目标上下文。
/// - `uart`：已选定的控制台串口节点，用于定位其所属的根中断控制器（PLIC）及推算时钟源频率。
/// - `info`：待填充的全局引导信息结构，成功时写入中断控制器、CCache、时钟输入及板载外设资源。
///
/// 返回值：
/// - `Ok(())`：所有必需的板级资源与选定外设均通过绑定校验并已记录到 `info`。
/// - `Err(&'static str)`：架构不匹配、关键控制器缺失或重复、Hart ID 越界或寄存器几何参数不符合硬件要求。
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
        // Step 1: 解析串口的父中断控制器（PLIC），校验所有在线 CPU 均为 U74 应用核（Hart 1..=4）。
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

        // Step 2: 定位唯一的 SiFive CCache 控制器，校验其缓存行大小（64 字节）与 FLUSH64 寄存器范围。
        let ccache = unique(tree, "starfive,jh7110-ccache")?;
        let registers = region_at(tree, ccache, 0)?;
        device_region(registers, info)?;
        info.board.ccache = registers;
        info.board.cache_line_size = number(ccache, "cache-block-size")? as usize;
        if info.board.cache_line_size != crate::arch::SIFIVE_CCACHE_LINE_SIZE
            || registers.end - registers.start
                < crate::arch::SIFIVE_CCACHE_FLUSH64 + core::mem::size_of::<u64>()
        {
            return Err("FDT JH7110 ccache geometry is unsupported");
        }

        // Step 3: 发现根时钟输入及选定的 SDIO1（mmc1）和千兆网卡（ethernet1）外设资源。
        info.board.clock_inputs = resources::jh7110_clock_inputs(tree, uart)?;
        info.board.mmc = discover_mmc(tree, controller, info)?;
        info.board.gmac = discover_gmac(tree, controller, info)?;

        // Step 4: 若设备树中启用了 JH7110 RTC，则校验其唯一性及其在 AONCRG 域下的时钟与复位依赖。
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

/// 解析并校验设备树别名 `mmc1` 指向的 JH7110 SDIO1（DesignWare Mobile Storage Host Controller）资源。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`discover`] 调用。
///
/// 参数：
/// - `tree`：扁平设备树只读视图。
/// - `controller`：系统主 PLIC 节点，用于验证 SDIO1 的中断线连接在同一个 PLIC 上。
/// - `info`：引导信息结构，用于登记 MMIO 地址空间与中断源上限。
///
/// 返回值：
/// - `Ok(Some(MmcResources))`：`mmc1` 存在、已启用且全部寄存器、时钟、复位、引脚、Syscon 与卡接口参数合法。
/// - `Ok(None)`：设备树未定义 `mmc1` 别名或对应节点处于禁用状态。
/// - `Err(&'static str)`：`mmc1` 节点已启用但兼容性、时钟/复位/引脚数量、Syscon 格式或总线宽度不符合要求。
#[cfg(target_arch = "riscv64")]
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

    // Step 1: 提取 MMIO 寄存器窗口与中断源编号，确认其挂载于主 PLIC 控制器下。
    let registers = single_region(tree, mmc)?;
    device_region(registers, info)?;
    let interrupt_source = resources::first_irq(tree, mmc)?;
    require_plic(tree, mmc, controller)?;
    device_source(interrupt_source, info)?;

    // Step 2: 解析并校验 SYSCRG 时钟（biu/ciu）、复位线与 6 路 SDIO 引脚复用配置。
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

    // Step 3: 解析 starfive,sysreg 四元组（phandle、寄存器偏移、位移、掩码）及卡槽接口参数。
    let sysreg = property(mmc, "starfive,sysreg")?;
    if sysreg.len() != 4 * FDT_CELL_BYTES {
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
        sysreg: [
            word(sysreg, FDT_CELL_BYTES)?,
            word(sysreg, 2 * FDT_CELL_BYTES)?,
            word(sysreg, 3 * FDT_CELL_BYTES)?,
        ],
        bus_width: number(mmc, "bus-width")?,
        max_frequency: number(mmc, "max-frequency")?,
        fifo_depth: resources::optional_number(mmc, "fifo-depth")?,
        data_offset: resources::optional_number(mmc, "data-addr")?.unwrap_or(0),
        card_detect: card_detect_gpio(tree, mmc, info)?,
        cd_inverted: mmc.property("cd-inverted").is_some(),
        disable_wp: mmc.property("disable-wp").is_some(),
        post_power_on_delay_ms: resources::optional_number(mmc, "post-power-on-delay-ms")?
            .unwrap_or(MMC_POST_POWER_ON_DELAY_MS),
        dma_coherent: resources::coherent(tree, mmc, true)?,
        dma_mask: u32::MAX as u64,
    };

    // Step 4: 校验 4-bit 数据线宽、非零最大时钟频率与合法的硬件 FIFO 深度范围。
    if resource.bus_width != 4
        || resource.max_frequency == 0
        || resource
            .fifo_depth
            .is_some_and(|depth| !(FIFO_MIN_DEPTH..=FIFO_MAX_DEPTH).contains(&depth))
    {
        return Err("FDT SDIO1 card interface is unsupported");
    }

    Ok(Some(resource))
}

/// 解析 SDIO 控制器节点上可选的 `cd-gpios`（外部 GPIO 卡检测引脚）属性。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`discover_mmc`] 调用。
///
/// 参数：
/// - `tree`：扁平设备树只读视图。
/// - `mmc`：正在解析的 SDIO 设备树节点。
/// - `info`：引导信息结构，用于校验 GPIO 控制器寄存器区间与内存区域不重叠。
///
/// 返回值：
/// - `Ok(Some(GpioInputResource))`：存在 `cd-gpios` 属性且指向合法的 `starfive,jh7110-sys-pinctrl` 引脚。
/// - `Ok(None)`：节点未声明 `cd-gpios` 属性，驱动将使用 DW-MMC 控制器自带的 `CDETECT` 寄存器检测插卡。
/// - `Err(&'static str)`：声明了 `cd-gpios`，但属性长度、GPIO 控制器兼容性、引脚编号或有效电平标志非法。
#[cfg(target_arch = "riscv64")]
fn card_detect_gpio<'a>(
    tree: &'a Fdt<'a>,
    mmc: Node<'a>,
    info: &BootInfo,
) -> Result<Option<GpioInputResource>, &'static str> {
    let Some(raw) = mmc.property("cd-gpios") else {
        return Ok(None);
    };
    if raw.value.len() != 3 * FDT_CELL_BYTES {
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

    let pin = word(raw.value, FDT_CELL_BYTES)?;
    let flags = word(raw.value, 2 * FDT_CELL_BYTES)?;
    if crate::drivers::board_setup::SYS_GPIO_COUNT <= pin || flags & !GPIO_ACTIVE_LOW != 0 {
        return Err("FDT SDIO card-detect GPIO pin or flags are unsupported");
    }
    let provider = single_region(tree, gpio)?;
    device_region(provider, info)?;
    Ok(Some(GpioInputResource {
        provider,
        pin,
        active_low: flags & GPIO_ACTIVE_LOW != 0,
    }))
}

/// 解析并校验设备树别名 `ethernet1` 指向的 JH7110 千兆以太网控制器（DesignWare QoS / DWMAC）资源。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`discover`] 调用。
///
/// 参数：
/// - `tree`：扁平设备树只读视图。
/// - `controller`：系统主 PLIC 节点，用于确认网卡中断线连接在同一 PLIC 上。
/// - `info`：引导信息结构，用于登记网卡 MMIO 区域与中断源上限。
///
/// 返回值：
/// - `Ok(Some(GmacResources))`：`ethernet1` 存在、已启用且寄存器、中断、Syscon 及 5 路时钟 / 2 路复位全部合法。
/// - `Ok(None)`：设备树未定义 `ethernet1` 别名或对应节点处于禁用状态。
/// - `Err(&'static str)`：`ethernet1` 节点已启用但设备兼容性、Syscon 绑定或时钟/复位资源不完整。
#[cfg(target_arch = "riscv64")]
fn discover_gmac<'a>(
    tree: &'a Fdt<'a>,
    controller: Node<'a>,
    info: &mut BootInfo,
) -> Result<Option<GmacResources>, &'static str> {
    let Some(mac) = resources::optional_alias(tree, "ethernet1")? else {
        return Ok(None);
    };
    if !compatible(mac, "starfive,jh7110-dwmac") {
        return Err("FDT selected Ethernet device is not a JH7110 GMAC");
    }

    // Step 1: 校验网卡中断连接到主 PLIC，提取通用 GMAC 参数并登记 MMIO 窗口与中断号。
    require_plic(tree, mac, controller)?;
    let mut resource = resources::gmac(tree, mac, true)?;
    resource.registers = single_region(tree, mac)?;
    device_region(resource.registers, info)?;
    resource.interrupt_source = resources::first_irq(tree, mac)?;
    device_source(resource.interrupt_source, info)?;

    // Step 2: 解析 starfive,syscon 三元组（phandle、寄存器偏移、位移），用于配置 PHY 接口模式。
    let sysreg = property(mac, "starfive,syscon")?;
    if sysreg.len() != 3 * FDT_CELL_BYTES {
        return Err("FDT GMAC syscon binding is invalid");
    }
    let syscon = tree
        .find_phandle(word(sysreg, 0)?)
        .ok_or("FDT GMAC syscon is missing")?;
    resource.syscon = single_region(tree, syscon)?;
    resource.sysreg = [
        word(sysreg, FDT_CELL_BYTES)?,
        word(sysreg, 2 * FDT_CELL_BYTES)?,
    ];

    // Step 3: 校验 JH7110 GMAC 必需的 5 路命名时钟与 2 路命名复位线。
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

/// 在设备树中查找唯一启用且匹配指定 `compatible` 字符串的控制器节点。
///
/// 调用上下文：引导早期单核（BSP）环境。若未找到对应节点，或存在多个同时启用的同名控制器节点，
/// 则返回错误，防止内核在多实例硬件上绑定到错误的控制器。
#[cfg(target_arch = "riscv64")]
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

/// 校验指定外设节点的父中断控制器是否与系统主 PLIC 为同一设备树节点。
///
/// 调用上下文：引导早期单核（BSP）环境。由于外设的 `interrupts` 属性只给出相对于其父中断控制器的
/// 局部中断源编号，必须先确认该外设确实挂载在系统选定的主 PLIC 下，才能把该中断号直接交给 PLIC 驱动管理。
#[cfg(target_arch = "riscv64")]
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
