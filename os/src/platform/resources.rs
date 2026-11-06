//! 设备树（FDT）标准外设属性与板级硬件资源的提取、绑定校验与有界转换。
//!
//! # 模块职责与架构定位
//! 本模块是平台资源发现（`crate::platform`）的子模块，负责在单次 FDT 扫描阶段将设备树节点上的原始字节属性转换为定长、强类型的启动资源结构（`crate::boot`）：
//! - **上游调用者**：`crate::platform`（通用控制台发现）、`crate::platform::visionfive2`（JH7110 板级外设发现）以及 `crate::platform::ls2k1000la`（LS2K1000LA 板级外设发现）。
//! - **下游消费者**：转换后的 `UartResources`、`DeviceClocks`、`DeviceSetup`、`GmacResources`、`BoardClockInputs` 等结构存入全局 `BootInfo` 快照，供早期控制台、时钟复位控制器（CRG）、引脚复用（Pinctrl）、DW-MMC 以及 DWMAC 网卡驱动在初始化时直接读取。
//!
//! # 核心设计：设备树资源引用与硬件拓扑解析
//! ```text
//! 设备节点 (UART / GMAC / MMC)
//!  ├── "clocks" / "clock-names" / "assigned-clocks*"
//!  │     └──> [phandle + #clock-cells] ──> 时钟提供者 (SYSCRG / PLL / fixed-clock)
//!  │                                         └── 解析寄存器物理区间 (或父 syscon 窗口) 与频率
//!  ├── "resets"
//!  │     └──> [phandle + #reset-cells] ──> 复位控制器寄存器区间 + 复位线编号
//!  ├── "pinctrl-0"
//!  │     └──> [phandle] ──> 引脚分组节点 ──> 子节点 "pinmux" + 上下拉/驱动强度/施密特触发器属性
//!  └── "dma-coherent" / "dma-noncoherent"
//!        └──> 沿 "interconnects" ("dma-mem") 或结构父链向上回溯，直至根节点或检出环路
//! ```
//!
//! # 锁顺序与并发契约
//! 本模块所有函数均为无状态的纯解析函数或仅修改调用方独占传入的 `&mut BootInfo`，在单核（BSP）早期启动阶段或 `ktest` 测试上下文中执行，不涉及任何锁或动态内存分配。

#[cfg(target_arch = "riscv64")]
use super::interrupt_parent;
use super::{
    cells, compatible, cstr, enabled, number, parent, property, single_region, word, Fdt, Node,
    FDT_CELL_BYTES,
};
#[cfg(target_arch = "riscv64")]
use crate::boot::BoardClockInputs;
use crate::boot::{
    BootInfo, ClockResource, DeviceClocks, DeviceSetup, GmacResources, PhyMode, PinResource,
    Platform, Region, ResourceReference,
};
use core::convert::{TryFrom, TryInto};

/// StarFive JH7110 系统时钟控制器（SYSCRG）中 `UART0_CORE` 时钟门控的硬件编号。
const JH7110_SYSCLK_UART0_CORE: u32 = 146;
/// QEMU `virt` 平台模拟串口在设备树未显式声明波特率时使用的默认波特率（115200 bps）。
const QEMU_CONSOLE_BAUD: u32 = 115_200;

/// 通过 `/aliases` 节点解析具名设备别名，返回对应且处于启用状态的设备树节点。
///
/// - **调用上下文**：早期启动单核资源发现阶段。
/// - **参数**：`tree` 为已通过结构校验的设备树；`name` 为 `/aliases` 下的属性名（如 `"serial0"`、`"ethernet0"`）。
/// - **返回值**：成功时返回目标 `Node`；若 `/aliases` 缺失、目标路径不存在或节点被禁用则返回静态错误描述。
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

/// 解析可选的设备树别名：若别名未声明或目标节点显式标为 `"disabled"` 则返回 `Ok(None)`，若声明损坏或状态非法则返回错误。
///
/// - **调用上下文**：板级可选外设（如 VisionFive 2 的双网口、SD/eMMC 槽位）发现阶段。
/// - **参数**：`tree` 为已校验的设备树；`name` 为 `/aliases` 下的别名属性名。
/// - **返回值**：找到且启用时返回 `Ok(Some(node))`，未声明或明确禁用时返回 `Ok(None)`，路径失效或状态异常时返回 `Err`。
#[cfg(target_arch = "riscv64")]
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

/// 根据 `/chosen` 节点的 `stdout-path` 属性解析主控制台 UART 节点（支持绝对路径或 `/aliases` 别名，并校验 `ns16550a` 或 `snps,dw-apb-uart` 兼容性）。
///
/// - **调用上下文**：早期启动控制台资源发现阶段。
/// - **参数**：`tree` 为已校验的设备树；`chosen` 为 `/chosen` 节点。
/// - **返回值**：成功时返回启用且兼容的 UART 节点；属性缺失、节点未启用或驱动绑定不支持时返回 `Err`。
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

/// 读取节点上的可选单 cell（32 位大端整数）属性；属性不存在时返回 `Ok(None)`，存在但长度不为 4 字节时返回错误。
///
/// - **调用上下文**：设备树属性解析。
/// - **参数**：`node` 为目标设备节点；`name` 为属性名称。
/// - **返回值**：成功时返回 `Ok(Some(u32))` 或 `Ok(None)`。
pub(super) fn optional_number(node: Node<'_>, name: &str) -> Result<Option<u32>, &'static str> {
    node.property(name).map(|_| number(node, name)).transpose()
}

/// 沿设备的 DMA 互连父链（优先查找名为 `"dma-mem"` 的 `interconnects` 引用，否则沿设备树结构父节点向上）解析设备的缓存一致性（DMA Coherency）。
///
/// - **调用上下文**：外设（如 GMAC、MMC、AHCI）DMA 属性发现阶段，或 `ktest` 单元测试。
/// - **参数**：`tree` 为已校验的设备树；`node` 为起始设备节点；`default_coherent` 为整条父链均未显式声明 `dma-coherent` / `dma-noncoherent` 时采用的架构默认值。
/// - **返回值**：返回 `Ok(true)` 表示硬件保证 DMA 缓存一致，`Ok(false)` 表示非一致（需软件维护缓存同步）；若父链步数超过节点总数（存在环路）或属性格式损坏则返回 `Err`。
pub(super) fn coherent<'a>(
    tree: &'a Fdt<'a>,
    mut node: Node<'a>,
    default_coherent: bool,
) -> Result<bool, &'static str> {
    // Step 1: 逐层向上回溯 DMA 父节点。由于 phandle 引用可能跨越树层级甚至在损坏的 DTB 中形成环路，
    // 循环次数以设备树总节点数为上限，超过上限仍未到达根节点即判定为父链环路。
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

/// 查找节点在 DMA 内存访问路径上的直接父节点：若节点声明了包含 `"dma-mem"` 的 `interconnect-names`，则解析对应的 `interconnects` 提供者节点；否则回退到设备树结构上的直接父节点。
///
/// - **调用上下文**：由 [`coherent`] 在向上回溯 DMA 父链时内部调用。
/// - **参数**：`tree` 为已校验的设备树；`node` 为当前检查的节点。
/// - **返回值**：返回 `Ok(Some(parent))` 表示上一级总线/互连节点，`Ok(None)` 表示已到达根节点之上；若互连属性截断或目标 phandle 无效则返回 `Err`。
fn dma_parent<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<Option<Node<'a>>, &'static str> {
    // Step 1: 在 interconnect-names 字符串列表中定位 "dma-mem" 条目的索引；若无该属性或未包含 "dma-mem"，则直接回退到树结构父节点。
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

    // Step 2: 按 "#interconnect-cells" 规定的参数宽度顺序扫描 interconnects 属性，跳过前序条目直至抵达目标索引。
    let raw = property(node, "interconnects")?;
    let mut at = 0;
    loop {
        let handle = word(raw, at)?;
        at += FDT_CELL_BYTES;
        // phandle 为 0 表示零参数的空占位条目；仅当它不是当前选中的 "dma-mem" 条目时才允许跳过。
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
            .checked_mul(FDT_CELL_BYTES)
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

/// 从属性字节流的当前游标位置读取一项时钟或复位控制器引用（phandle + 0 或 1 个参数 cell），并解析该提供者的寄存器物理区间。
///
/// - **调用上下文**：解析设备节点的 `clocks`、`assigned-clocks`、`assigned-clock-parents` 或 `resets` 属性列表。
/// - **参数**：`tree` 为设备树；`bytes` 为属性原始字节切片；`at` 为当前读取字节偏移游标（调用后自动前移）；`count_name` 为提供者节点上声明参数个数的属性名（如 `"#clock-cells"` 或 `"#reset-cells"`）。
/// - **返回值**：成功时返回 `(ResourceReference, provider_node)` 二元组；若 phandle 不存在、提供者未启用或参数 cell 数量超过 1 则返回 `Err`。
fn reference<'a>(
    tree: &'a Fdt<'a>,
    bytes: &[u8],
    at: &mut usize,
    count_name: &str,
) -> Result<(ResourceReference, Node<'a>), &'static str> {
    let handle = word(bytes, *at)?;
    *at += FDT_CELL_BYTES;
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
        *at += FDT_CELL_BYTES;
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

/// 获取时钟或复位提供者节点的寄存器物理地址区间：
/// - `fixed-clock` 无硬件寄存器，返回空的默认 `Region`；
/// - StarFive JH7110 的 PLL 节点若未单独声明 `reg`，则共享其父级 `starfive,jh7110-sys-syscon` 系统控制器节点的寄存器窗口；
/// - 其余控制器直接读取其唯一的 `reg` 物理区间。
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

/// 在节点的 `clock-names` 与 `clocks` 属性中按同名索引查找指定名称的时钟输入，返回其资源引用及提供者节点。
///
/// - **调用上下文**：RISC-V 板级时钟拓扑与外设具名时钟解析阶段。
/// - **参数**：`tree` 为设备树；`node` 为消费该时钟的设备节点；`wanted` 为目标时钟名称（如 `"baudclk"`、`"osc"`、`"pll0_out"`）。
/// - **返回值**：找到时返回 `(ResourceReference, provider_node)`；未找到或属性格式非法时返回 `Err`。
#[cfg(target_arch = "riscv64")]
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

/// 从所选控制台 UART 的 `"baudclk"` 时钟出发，验证并提取 StarFive JH7110 系统时钟控制器（SYSCRG）、共享 PLL（PLL0/PLL1/PLL2）及外部固定晶振（`osc`）的硬件拓扑参数。
///
/// - **调用上下文**：VisionFive 2 平台资源发现阶段。
/// - **参数**：`tree` 为设备树；`uart` 为主控制台 UART 节点。
/// - **返回值**：成功时返回包含晶振频率（Hz）与 PLL 寄存器物理窗口的 [`BoardClockInputs`]；若时钟树拓扑与 JH7110 绑定不符则返回 `Err`。
#[cfg(target_arch = "riscv64")]
pub(super) fn jh7110_clock_inputs<'a>(
    tree: &'a Fdt<'a>,
    uart: Node<'a>,
) -> Result<BoardClockInputs, &'static str> {
    // Step 1: 确认 UART 的 baudclk 来自 JH7110 系统时钟控制器（SYSCRG）。
    let (_, syscrg) = named_clock(tree, uart, "baudclk")?;
    if !compatible(syscrg, "starfive,jh7110-syscrg") {
        return Err("FDT selected UART clock does not use the system CRG");
    }

    // Step 2: 校验 SYSCRG 的 pll0_out、pll1_out、pll2_out 三个主 PLL 输入均指向同一个 JH7110 PLL 提供者且输出编号分别为 0、1、2。
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

    // Step 3: 校验 PLL 与 SYSCRG 的参考晶振（osc）为同一个非零频率的 fixed-clock 节点。
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

/// 解析设备节点的 `clocks` 输入列表以及 `assigned-clocks`、`assigned-clock-parents`、`assigned-clock-rates` 预设配置，汇总到定长的 [`DeviceClocks`] 结构中。
///
/// - **调用上下文**：外设（UART、GMAC、MMC 等）时钟资源提取阶段。
/// - **参数**：`tree` 为设备树；`node` 为目标外设节点。
/// - **返回值**：成功时返回填充完毕的 [`DeviceClocks`]；若超出定长数组容量或引用非法则返回 `Err`。
pub(super) fn clocks<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<DeviceClocks, &'static str> {
    // Step 1: 顺序解析设备的 "clocks" 属性，记录每个时钟源的提供者寄存器窗口、编号及固定频率（若有）。
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

    // Step 2: 若存在 "assigned-clocks"，同步解析其对应的父时钟源（assigned-clock-parents）与目标频率（assigned-clock-rates），
    // 并回填更新 entries 中匹配的时钟项。
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
                        parent_at += FDT_CELL_BYTES;
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
                .map(|r| word(r.value, index * FDT_CELL_BYTES))
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

/// 提取外设完整的前置硬件配置（包含时钟 `clocks`、复位线 `resets` 以及默认引脚复用组 `pinctrl-0` 的电气参数）。
///
/// - **调用上下文**：板级外设（如 GMAC、SDIO/eMMC）发现阶段。
/// - **参数**：`tree` 为设备树；`node` 为目标外设节点。
/// - **返回值**：成功时返回 [`DeviceSetup`]；若复位或引脚数量超出定长数组容量或属性截断则返回 `Err`。
pub(super) fn setup<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<DeviceSetup, &'static str> {
    // Step 1: 解析设备的时钟输入与预设时钟配置。
    let mut result = DeviceSetup {
        clocks: clocks(tree, node)?,
        ..DeviceSetup::default()
    };
    // Step 2: 解析设备的 "resets" 属性列表，记录每根复位线所属的控制器寄存器区间与复位编号。
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

    // Step 3: 解析默认引脚配置 "pinctrl-0"，遍历每个引脚组子节点的 "pinmux" 列表，并将上下拉、驱动强度、输入使能、施密特触发器及压摆率属性绑定到每个具体引脚。
    if let Some(raw) = node.property("pinctrl-0") {
        if raw.value.len() % FDT_CELL_BYTES != 0 {
            return Err("FDT pinctrl phandles are truncated");
        }
        for handle in raw.value.chunks_exact(FDT_CELL_BYTES) {
            let group = tree
                .find_phandle(word(handle, 0)?)
                .ok_or("FDT pinctrl group is missing")?;
            let provider = parent(tree, group).ok_or("FDT pinctrl provider is missing")?;
            let registers = single_region(tree, provider)?;
            for pins in group.children() {
                let Some(muxes) = pins.property("pinmux") else {
                    continue;
                };
                if muxes.value.len() % FDT_CELL_BYTES != 0 {
                    return Err("FDT pinmux values are truncated");
                }
                for mux in muxes.value.chunks_exact(FDT_CELL_BYTES) {
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

/// 解析成对互斥的布尔型引脚电气属性（如 `input-enable` 与 `input-disable`）；若两者同时出现则判定为配置冲突并返回错误。
fn boolean_setting(node: Node<'_>, yes: &str, no: &str) -> Result<Option<bool>, &'static str> {
    match (node.property(yes).is_some(), node.property(no).is_some()) {
        (true, true) => Err("FDT pin settings conflict"),
        (true, false) => Ok(Some(true)),
        (false, true) => Ok(Some(false)),
        (false, false) => Ok(None),
    }
}

/// 解析并校验主控制台 UART 的寄存器访问位宽、地址移位、输入时钟源及目标波特率，结果写入 `info.uart`。
///
/// - **调用上下文**：通用平台资源发现阶段（[`super::discover_tree`]）。
/// - **参数**：`tree` 为设备树；`chosen` 为 `/chosen` 节点；`uart` 为已选定的控制台节点；`info` 为待填充的启动信息快照。
/// - **返回值**：成功时更新 `info.uart` 并返回 `Ok(())`；若寄存器几何参数、时钟拓扑或波特率无效则返回 `Err`。
pub(super) fn discover_uart<'a>(
    tree: &'a Fdt<'a>,
    chosen: Node<'a>,
    uart: Node<'a>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    // Step 1: 读取并校验寄存器访问位宽（reg-io-width，支持 1 或 4 字节）与偏移移位（reg-shift，不超过 2）。
    let width = optional_number(uart, "reg-io-width")?.unwrap_or(1);
    let shift = optional_number(uart, "reg-shift")?.unwrap_or(0);
    if !matches!(width, 1 | 4) || 2 < shift {
        return Err("FDT UART register access geometry is unsupported");
    }

    // Step 2: 解析 UART 时钟；在 VisionFive 2 平台上额外校验 UART0 的双时钟绑定（baudclk 与 apb_pclk）并追溯上游固定晶振频率。
    let mut uart_clocks = clocks(tree, uart)?;
    let mut frequency = optional_number(uart, "clock-frequency")?.map(u64::from);
    if info.platform == Platform::VisionFive2 {
        if width != 4
            || shift != 2
            || uart_clocks.count != 2
            || uart_clocks.entries[0].reference.id != JH7110_SYSCLK_UART0_CORE
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

    // Step 3: 按优先级解析控制台波特率：优先提取 /chosen stdout-path 冒号后的参数，其次读取节点 current-speed 属性，最后在 QEMU 平台上回退至默认 115200 波特率。
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
        QEMU_CONSOLE_BAUD
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

/// 解析千兆以太网控制器（GMAC / DWMAC）及其关联外部 PHY 的设备树属性，生成 [`GmacResources`] 描述符。
///
/// - **调用上下文**：VisionFive 2 与龙芯 2K1000LA 板级网卡资源发现阶段。
/// - **参数**：`tree` 为设备树；`node` 为 GMAC 控制器节点；`default_coherent` 为当前架构的默认 DMA 缓存一致性配置。
/// - **返回值**：成功时返回包含时钟/复位/引脚配置、PHY 地址与接口模式、信号延迟微调参数、单播 MAC 地址及 DMA 掩码的 [`GmacResources`]。
pub(super) fn gmac<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    default_coherent: bool,
) -> Result<GmacResources, &'static str> {
    // Step 1: 通过 phy-handle 定位外部 PHY 节点，校验 Clause 22 PHY 地址范围（0..32）以及 RGMII/RMII 接口模式。
    let phy = tree
        .find_phandle(number(node, "phy-handle")?)
        .ok_or("FDT GMAC PHY is missing")?;
    let address = number(phy, "reg")?;
    if crate::drivers::dwmac::PHY_ADDRESS_COUNT <= address {
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

    // Step 2: 读取可选的 mac-address 或 local-mac-address 属性，若存在则校验其必须是 6 字节的非零单播（Unicast）以太网地址。
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

    // Step 3: 解析并校验连续的 DMA 地址掩码（dma-mask）与缓存一致性属性，汇总 PHY 延迟与时钟反相等硬件微调参数。
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

/// 读取设备节点在其所属中断控制器下的第一个中断源编号（忽略设备可能声明的后续附加中断项）。
///
/// - **调用上下文**：RISC-V 板级外设（如 GMAC、MMC）中断源发现阶段。
/// - **参数**：`tree` 为设备树；`node` 为目标外设节点。
/// - **返回值**：成功时返回首个中断源编号（`u32`）；若中断属性长度与父控制器的 `#interrupt-cells` 不匹配则返回 `Err`。
#[cfg(target_arch = "riscv64")]
pub(super) fn first_irq<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<u32, &'static str> {
    let controller = interrupt_parent(tree, node)?;
    let raw = property(node, "interrupts")?;
    let width = number(controller, "#interrupt-cells")? as usize * FDT_CELL_BYTES;
    if width == 0 || raw.len() % width != 0 {
        return Err("FDT device interrupt cells are truncated");
    }
    word(raw, 0)
}

/// 校验设备节点的具名资源属性（如 `reg-names`、`clock-names`、`reset-names`）是否与驱动期望的名称序列严格一致（顺序相同且无多余项）。
///
/// - **调用上下文**：外设资源发现阶段的前置绑定检查，使后续驱动代码可直接按固定索引访问资源而无需在运行时重复解析字符串。
/// - **参数**：`node` 为设备节点；`property_name` 为名称列表属性名；`names` 为期望的字符串切片序列。
/// - **返回值**：完全匹配时返回 `Ok(())`，否则返回 `Err`。
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

/// 校验设备节点上指定资源列表属性（如 `clocks` 或 `resets`）引用的所有提供者节点均兼容预期的控制器 `compatible` 绑定。
///
/// - **调用上下文**：RISC-V 板级外设发现阶段，防止将不同控制器的时钟/复位 ID 混用。
/// - **参数**：`tree` 为设备树；`node` 为外设节点；`property_name` 为引用列表属性名；`cell_name` 为提供者参数宽度属性名；`binding` 为期望的提供者 `compatible` 字符串。
/// - **返回值**：全部提供者均兼容时返回 `Ok(())`，列表为空或存在不兼容提供者时返回 `Err`。
#[cfg(target_arch = "riscv64")]
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
