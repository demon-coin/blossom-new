//! 引导期固件资源与内核保留区快照（Boot Resource Snapshot）。
//!
//! # 模块职责与架构定位
//! 本模块定义了内核在最早期引导阶段从设备树（DTB）与 UEFI 配置表中解析出的硬件拓扑、
//! 物理内存分布、保留区域、中断控制器、串口、PCI 桥与板级外设（MMC / AHCI / GMAC / RTC / 时钟复位）描述。
//! - **上游写入者**：`platform::discover` 在主核（BSP）单核关中断阶段调用 [`prepare`] 获取静态存储槽的唯一可变引用，
//!   填充各硬件资源字段，并由 `main.rs` 登记内核镜像保留区后调用 [`publish`] 冻结发布。
//! - **下游读取者**：`mm`、`arch`、`cpu`、`irq`、`time`、`drivers` 等全部子系统通过 [`info`] 获取 `'static` 只读引用。
//!
//! # 核心设计与生命周期状态机
//! ```text
//! BOOT_STORAGE (MaybeUninit<BootInfo>) + BOOT_INFO (spin::Once<&'static BootInfo>)
//!
//!   [未初始化 Uninit]
//!          │
//!          │ boot::prepare(raw_args, bsp_hardware_id)
//!          │ (仅 BSP 关中断调用一次，通过 addr_of_mut! 原地逐字段初始化，避免 64KiB 启动栈溢出)
//!          ▼
//!   [主核独占可变期 &'static mut BootInfo]
//!          │  1. platform::discover 解析 RAM / 保留区 / CPU / 中断 / PCI / 板级外设
//!          │  2. reserve(kernel_region()) 登记内核自身物理区间
//!          │
//!          │ boot::publish(info)
//!          ▼
//!   [全局冻结只读期 &'static BootInfo] ──► boot::info() 供全内核无锁并发读取
//! ```
//!
//! # PCI OpenFirmware 中断路由（`phys.hi`）位域格式
//! ```text
//! 31      26 25  24 23      16 15     11 10       8 7            0
//! +---------+------+----------+---------+----------+-------------+
//! |  Flags  |Space |   Bus    | Device  | Function | Register... |
//! +---------+------+----------+---------+----------+-------------+
//!            ^^^^^^ ^^^^^^^^^^ ^^^^^^^^^ ^^^^^^^^^^
//!            01:IO   bits23:16 bits15:11  bits10:8
//!            10:M32
//!            11:M64
//! ```
//!
//! # 同步与并发契约
//! - [`BOOT_STORAGE`] 的可变访问严格限制在主核唤醒从核（AP）且开启中断之前的单执行流阶段；
//! - [`BOOT_INFO`] 通过 [`spin::Once`] 保证发布操作的单次性与跨核可见性，发布后全生命周期只读，无需加锁。

use core::mem::MaybeUninit;
use core::ptr;
use spin::Once;

include!(concat!(env!("OUT_DIR"), "/cpu-capacity.rs"));
/// 内核启动命令行（`bootargs`）的最大字节容量（4 KiB）。
pub const MAX_BOOTARGS_BYTES: usize = 4096;
/// 固件可申报的可用物理内存（RAM）区间最大数量。
pub const MAX_MEMORY_REGIONS: usize = 32;
/// 固件保留区（`reserved-memory` / `/memreserve/` / 内核镜像 / DTB / initrd）的最大登记数量。
pub const MAX_RESERVED_REGIONS: usize = 128;
/// 固件设备树可声明的启用态 VirtIO-MMIO 设备槽位上限。
pub const MAX_VIRTIO_MMIO: usize = 8;
/// 单个外设可绑定的最大时钟条目数。
pub(crate) const MAX_DEVICE_CLOCKS: usize = 8;
/// 单个外设可绑定的最大复位控制器信号数。
pub(crate) const MAX_DEVICE_RESETS: usize = 3;
/// 单个外设可绑定的最大引脚复用（pinctrl）配置项数。
pub(crate) const MAX_DEVICE_PINS: usize = 16;
/// PCI 主桥可声明的地址转换窗口（`ranges`）最大数量。
pub(crate) const MAX_PCI_RANGES: usize = 4;

/// OpenFirmware PCI 高 32 位地址单元（`phys.hi`）中的地址空间类型掩码（bits 25:24）。
pub(crate) const PCI_SPACE_CODE_MASK: u32 = 0x0300_0000;
/// OpenFirmware PCI I/O 端口空间编码（`0b01 << 24`）。
pub(crate) const PCI_SPACE_IO: u32 = 0x0100_0000;
/// OpenFirmware PCI 32 位内存空间编码（`0b10 << 24`）。
pub(crate) const PCI_SPACE_MEM32: u32 = 0x0200_0000;
/// OpenFirmware PCI 64 位内存空间编码（`0b11 << 24`）。
pub(crate) const PCI_SPACE_MEM64: u32 = 0x0300_0000;
/// OpenFirmware PCI `phys.hi` 中总线号（Bus）字段的起始位移（bits 23:16）。
pub(crate) const PCI_BUS_SHIFT: u32 = 16;
/// OpenFirmware PCI `phys.hi` 中设备号（Device）字段的起始位移（bits 15:11）。
pub(crate) const PCI_DEVICE_SHIFT: u32 = 11;
/// OpenFirmware PCI `phys.hi` 中功能号（Function）字段的起始位移（bits 10:8）。
pub(crate) const PCI_FUNCTION_SHIFT: u32 = 8;
/// 单条 PCI 总线上的最大设备号数量（0..32）。
pub(crate) const PCI_DEVICE_COUNT: u8 = 32;
/// 单个 PCI 设备上的最大功能号数量（0..8）。
pub(crate) const PCI_FUNCTION_COUNT: u8 = 8;
/// PCI 传统中断引脚（INTA#..INTD#）的数量（合法编号为 1..=4）。
pub(crate) const PCI_INTX_PIN_COUNT: u8 = 4;

/// 当前目标平台外部中断控制器的中断源表容量（RISC-V PLIC 包含保留的 0 号源）。
pub const MAX_INTERRUPT_SOURCES: usize = if cfg!(blossom_platform = "visionfive2") {
    136 + 1
} else if cfg!(target_arch = "riscv64") {
    95 + 1
} else {
    64
};

/// 编译期选定的目标硬件平台类型。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Platform {
    /// QEMU `virt` 虚拟机平台（支持 RISC-V 64 与 LoongArch 64）。
    #[default]
    QemuVirt,
    /// StarFive VisionFive 2（JH7110 SoC，RISC-V 64）开发板。
    VisionFive2,
    /// 龙芯 LS2K1000LA（LoongArch 64）开发板。
    Ls2k1000la,
}

/// 左闭右开的物理地址区间 `[start, end)`（单位：字节）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Region {
    /// 区间起始物理地址（包含，单位：字节）。
    pub start: usize,
    /// 区间结束物理地址（不包含，单位：字节）。
    pub end: usize,
}

impl Region {
    /// 由起始物理地址 `start` 与字节长度 `size` 构造半开区间，若发生整数加法溢出或长度为 0 则返回错误。
    pub fn from_size(start: usize, size: usize) -> Result<Self, &'static str> {
        let end = start.checked_add(size).ok_or("physical region overflows")?;
        if end <= start {
            return Err("physical region is empty");
        }
        Ok(Self { start, end })
    }

    /// 判断当前半开区间 `[self.start, self.end)` 与 `other` 是否存在交集。
    pub fn overlaps(self, other: Self) -> bool {
        self.start < other.end && other.start < self.end
    }
}

/// 串口与片上中断控制器的物理资源及中断号描述。
#[derive(Clone, Copy, Debug)]
pub struct InterruptResources {
    /// 控制台 UART 寄存器的物理地址区间。
    pub uart: Region,
    /// 控制台 UART 在外部中断控制器上的中断源编号（IRQ Source ID）。
    pub uart_source: u32,
    /// 外部中断控制器合法中断源编号的开区间上界（RISC-V PLIC 的 0 号源保留不用）。
    pub source_count: u32,
    /// 主外部中断控制器（RISC-V PLIC 或 LoongArch PCH-PIC）的 MMIO 物理地址区间。
    pub controller: Region,
    /// 父级中断控制器（如 LoongArch EIOINTC）的 MMIO 物理地址区间；无独立父控制器时为空区间。
    pub parent_controller: Region,
    /// 外部中断控制器汇入 CPU 核内中断控制器的父中断号。
    pub root_interrupt: u32,
    /// RISC-V PLIC 中每个逻辑 CPU 对应的 S 态（Supervisor）上下文索引；LoongArch 不使用此数组。
    #[allow(
        dead_code,
        reason = "RISC-V PLIC 按每核 context 访问 claim 和 enable，LoongArch 使用独立中断控制器"
    )]
    pub plic_contexts: [usize; MAX_CPUS],
}

impl Default for InterruptResources {
    /// 创建全零初始化的空白中断资源快照，供平台解析阶段逐项填入。
    fn default() -> Self {
        Self {
            uart: Region::default(),
            uart_source: 0,
            source_count: 0,
            controller: Region::default(),
            parent_controller: Region::default(),
            root_interrupt: 0,
            plic_contexts: [0; MAX_CPUS],
        }
    }
}

/// 控制台 UART（16550 / Synopsys DW APB UART）的寄存器访问宽度、时钟与波特率资源。
#[derive(Clone, Copy, Debug, Default)]
pub struct UartResources {
    /// 寄存器 MMIO 访问字节宽度（如 1 字节或 4 字节）。
    pub reg_io_width: u32,
    /// 相邻寄存器偏移量的左移位数（例如 `reg-shift = 2` 表示按 4 字节对齐寻址）。
    pub reg_shift: u32,
    /// 固件直接给定的固定输入时钟频率（单位：Hz）；若由可编程时钟控制器提供则在 `clocks` 中描述。
    pub clock_frequency: Option<u64>,
    /// 串口依赖的时钟控制器绑定列表（用于 VisionFive 2 等开发板的时钟使能与频率查询）。
    #[allow(
        dead_code,
        reason = "RISC-V 映射和板 UART 准备消费时钟 provider，其他平台保留同一固件资源快照"
    )]
    pub clocks: DeviceClocks,
    /// 串口目标通信波特率（例如 `115200`）。
    pub baud: u32,
    /// 是否为 Synopsys DesignWare APB UART（需处理忙状态中断 USR 寄存器）。
    pub dw_apb: bool,
}

/// 指向时钟控制器或复位控制器中特定通道的二元引用。
#[derive(Clone, Copy, Debug, Default)]
pub struct ResourceReference {
    /// 提供该时钟或复位信号的控制器寄存器物理区间。
    pub provider: Region,
    /// 控制器内部的时钟门控编号或复位线编号。
    pub id: u32,
}

/// 单个设备时钟通道及其可选的父时钟源与目标频率配置。
#[derive(Clone, Copy, Debug, Default)]
pub struct ClockResource {
    /// 目标时钟控制器及通道编号。
    pub reference: ResourceReference,
    /// 设备树 `assigned-clock-parents` 指定的父时钟源引用。
    pub parent: Option<ResourceReference>,
    /// 设备树 `assigned-clock-rates` 指定的目标频率（单位：Hz）。
    pub rate: Option<u64>,
}

/// 单个外设关联的有界时钟配置表，保持与设备树 `clocks` / `assigned-clocks` 相同的顺序。
#[derive(Clone, Copy, Debug, Default)]
pub struct DeviceClocks {
    /// 外设运行所需的时钟门控与工作时钟列表（来自 `clocks` 属性）。
    pub entries: [ClockResource; MAX_DEVICE_CLOCKS],
    /// `entries` 中的有效时钟条目数量。
    pub count: usize,
    /// 外设初始化时需配置的父时钟与频率列表（来自 `assigned-clocks` 属性）。
    pub assigned: [ClockResource; MAX_DEVICE_CLOCKS],
    /// `assigned` 中的有效配置条目数量。
    pub assigned_count: usize,
}

/// 单个引脚复用（Pinmux）与电气属性配置项。
#[derive(Clone, Copy, Debug, Default)]
pub struct PinResource {
    /// 所属引脚控制器（pinctrl / iomux）的寄存器物理区间。
    pub provider: Region,
    /// 编码后的引脚编号、功能选择（mux）及寄存器偏移字。
    pub mux: u32,
    /// 是否启用内部上拉电阻。
    pub pull_up: bool,
    /// 是否禁用上下拉偏置。
    pub bias_disable: bool,
    /// 可选的驱动电流强度编码值（单位依具体 SoC pad 手册而定）。
    pub drive_strength: Option<u32>,
    /// 可选的输入缓冲使能开关。
    pub input_enable: Option<bool>,
    /// 可选的施密特触发器（Schmitt Trigger）使能开关。
    pub schmitt_enable: Option<bool>,
    /// 可选的信号压摆率（Slew Rate）配置值。
    pub slew_rate: Option<u32>,
}

/// 通用 GPIO 输入引脚描述（如 microSD 卡检测引脚）。
#[derive(Clone, Copy, Debug, Default)]
pub struct GpioInputResource {
    /// 所属 GPIO 控制器的寄存器物理地址区间。
    #[allow(
        dead_code,
        reason = "VF2 的卡检测 GPIO 准备及映射消费 provider，其他平台没有同一 GPIO 控制器"
    )]
    pub provider: Region,
    /// 控制器内的 GPIO 引脚编号。
    #[allow(
        dead_code,
        reason = "VF2 卡检测按实际 GPIO pin 读取电平，其他平台使用控制器原生卡检测"
    )]
    pub pin: u32,
    /// 信号是否为低电平有效（`true` 表示低电平代表有卡/有效）。
    #[allow(
        dead_code,
        reason = "SD 卡检测按固件 GPIO 极性解释实际电平，具体平台可没有外部 CD GPIO"
    )]
    pub active_low: bool,
}

/// 片上原生外设上线前所需的时钟、复位与引脚复用资源集合。
#[derive(Clone, Copy, Debug, Default)]
pub struct DeviceSetup {
    /// 外设关联的时钟列表与初始频率/父源设置。
    pub clocks: DeviceClocks,
    /// 外设关联的复位信号列表。
    pub resets: [ResourceReference; MAX_DEVICE_RESETS],
    /// `resets` 中的有效复位信号数量。
    pub reset_count: usize,
    /// 外设关联的引脚复用与电气配置列表。
    pub pins: [PinResource; MAX_DEVICE_PINS],
    /// `pins` 中的有效引脚配置数量。
    pub pin_count: usize,
}

/// 龙芯 LIOINTC（本地 I/O 中断控制器）单组硬件资源描述。
#[derive(Clone, Copy, Debug, Default)]
pub struct LioIntcResources {
    /// LIOINTC 主控制寄存器窗口的物理地址区间。
    pub registers: Region,
    #[allow(
        dead_code,
        reason = "LoongArch LIOINTC 用本核 ISR 窗口分发，RISC-V 使用 PLIC"
    )]
    /// 每个核心对应的中断状态寄存器（ISR）物理地址窗口。
    pub isr: [Region; 2],
    #[allow(
        dead_code,
        reason = "LoongArch LIOINTC 按固件父 cause 路由硬中断，RISC-V 使用 PLIC"
    )]
    /// 该组 LIOINTC 连接到 CPU 核内中断控制器的父硬中断线编号。
    pub parent_interrupt: u32,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "固件测试逐字段比较 LIOINTC 父路由表，生产解析时已经核定支持的路由"
        )
    )]
    /// 4 个中断分组各自对应的父中断路由映射掩码。
    pub parent_map: [u32; 4],
}

/// 通过系统控制器（syscon）寄存器位操作触发的板级关机或重启描述。
#[derive(Clone, Copy, Debug)]
pub struct SysconAction {
    /// syscon 控制器的寄存器物理地址区间。
    #[allow(
        dead_code,
        reason = "LoongArch 最终关机或重启访问固件 syscon 窗口，RISC-V 使用 SBI 动作"
    )]
    pub registers: Region,
    #[allow(
        dead_code,
        reason = "LoongArch 最终电源动作使用固件指定 syscon 偏移，RISC-V 使用 SBI 动作"
    )]
    /// 电源控制寄存器在 `registers` 窗口内的字节偏移量。
    pub offset: u32,
    #[allow(
        dead_code,
        reason = "LoongArch syscon 电源动作只改固件 mask 指定的位，RISC-V 使用 SBI 动作"
    )]
    /// 写入时需要修改的寄存器位掩码。
    pub mask: u32,
    #[allow(
        dead_code,
        reason = "LoongArch syscon 电源动作写入固件指定值，RISC-V 使用 SBI 动作"
    )]
    /// 触发电源动作需写入寄存器的目标数值。
    pub value: u32,
}

/// SD/MMC 主机控制器（如 DesignWare Mobile Storage Host）的硬件资源与卡槽参数。
#[derive(Clone, Copy, Debug, Default)]
pub struct MmcResources {
    /// MMC 控制器寄存器的物理地址区间。
    pub registers: Region,
    /// MMC 控制器在外部中断控制器上的中断源编号。
    pub interrupt_source: u32,
    /// 控制器上线所需的时钟、复位与引脚复用资源。
    pub setup: DeviceSetup,
    /// SoC 系统控制器（syscon）寄存器物理区间（用于调谐或采样相位配置）。
    #[allow(
        dead_code,
        reason = "RISC-V 映射 VF2 MMC 的 syscon 资源，其他架构不访问该板窗口"
    )]
    pub syscon: Region,
    #[expect(
        dead_code,
        reason = "资源快照保存 DT 的 MMC syscon 选择字段；当前 SD 驱动不写该选择，不能把它解释为已执行的硬件设置"
    )]
    /// 设备树中声明的 syscon 寄存器偏移与掩码参数三元组。
    pub sysreg: [u32; 3],
    /// 支持的数据总线位宽（例如 4 线模式为 `4`）。
    pub bus_width: u32,
    /// 控制器允许的最大总线时钟频率（单位：Hz）。
    pub max_frequency: u32,
    /// 控制器内置 FIFO 深度（单位：字）。
    pub fifo_depth: Option<u32>,
    /// FIFO 数据寄存器在控制器 MMIO 窗口内的字节偏移量。
    pub data_offset: u32,
    /// 可选的外部 GPIO 卡检测引脚资源。
    pub card_detect: Option<GpioInputResource>,
    /// 卡检测信号极性是否反转。
    pub cd_inverted: bool,
    /// 是否忽略物理写保护引脚（始终视为可写）。
    pub disable_wp: bool,
    /// 卡槽上电后的稳定等待时间（单位：毫秒）。
    pub post_power_on_delay_ms: u32,
    /// 该设备 DMA 访问是否与 CPU 缓存保持硬件一致性（Cache Coherent）。
    pub dma_coherent: bool,
    /// 该设备 DMA 引擎可寻址的最高物理地址位掩码。
    pub dma_mask: u64,
}

/// PCI 总线上的设备功能（BDF）定位标识及其中断/DMA 属性。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciFunctionResource {
    /// PCI 总线号（Bus，0..=255）。
    pub bus: u8,
    /// PCI 设备号（Device，0..32）。
    pub device: u8,
    /// PCI 功能号（Function，0..8）。
    pub function: u8,
    /// 该功能解析后的外部中断控制器中断源编号。
    pub interrupt_source: u32,
    /// 该 PCI 功能的 DMA 访问是否具备硬件缓存一致性。
    pub dma_coherent: bool,
}

/// SATA AHCI 控制器的硬件资源描述（支持片上直接 MMIO 或 PCI 挂载两种形态）。
#[derive(Clone, Copy, Debug, Default)]
pub struct AhciResources {
    /// 片上直连 AHCI 的 MMIO 物理地址区间；若为 PCI 设备则在总线枚举 BAR 后补全。
    pub registers: Region,
    /// 当 AHCI 挂载在 PCI 总线上时的 BDF 标识。
    pub pci: Option<PciFunctionResource>,
    /// AHCI 控制器的中断源编号。
    pub interrupt_source: u32,
    /// AHCI DMA 访问是否具备硬件缓存一致性。
    pub dma_coherent: bool,
    /// 可选的设备专属 DMA 寻址掩码。
    pub dma_mask: Option<u64>,
    /// 固件指定的可用 SATA 端口位图（覆盖 HBA `PI` 寄存器默认值）。
    pub ports_implemented: Option<u32>,
}

/// 千兆以太网控制器（Synopsys DesignWare GMAC）的硬件资源、PHY 接口与时钟调谐描述。
#[derive(Clone, Copy, Debug, Default)]
pub struct GmacResources {
    /// 片上直连 GMAC 的 MMIO 物理地址区间；若为 PCI 设备则由 BAR 枚举确定。
    pub registers: Region,
    /// 当 GMAC 挂载在 PCI 总线上时的 BDF 标识。
    pub pci: Option<PciFunctionResource>,
    /// GMAC 控制器的中断源编号。
    pub interrupt_source: u32,
    /// GMAC 所需的时钟、复位与引脚配置资源。
    pub setup: DeviceSetup,
    /// 用于配置 PHY 接口模式与时钟选择的 syscon 物理地址区间。
    pub syscon: Region,
    /// syscon 中 GMAC 控制寄存器的字节偏移量与位域掩码。
    pub sysreg: [u32; 2],
    /// 固件传入的 6 字节以太网 MAC 地址。
    pub mac_address: Option<[u8; 6]>,
    /// MDIO 总线上的外部 PHY 芯片地址（0..32）。
    pub phy_address: u32,
    /// MAC 与 PHY 之间的硬件接口模式（如 RGMII / RGMII-ID）。
    pub phy_mode: PhyMode,
    /// 可选的 RX 时钟线延迟时间（单位：皮秒 ps）。
    pub rx_delay_ps: Option<u32>,
    /// 可选的 TX 时钟线延迟时间（单位：皮秒 ps）。
    pub tx_delay_ps: Option<u32>,
    /// 是否启用 PHY 侧 TX 时钟相位调整。
    pub phy_tx_clock_adjust: bool,
    /// 10M / 100M / 1000M 三种速率下是否反转 PHY TX 时钟极性。
    pub phy_tx_inverted: [bool; 3],
    /// PHY 时钟引脚驱动电流强度（单位：微安 uA）。
    pub phy_clock_drive_ua: Option<u32>,
    /// PHY 数据引脚驱动电流强度（单位：微安 uA）。
    pub phy_data_drive_ua: Option<u32>,
    /// 发送路径是否直接使用外部 RGMII 参考时钟。
    pub tx_use_rgmii_clock: bool,
    /// GMAC DMA 访问是否具备硬件缓存一致性。
    pub dma_coherent: bool,
    /// GMAC DMA 引擎可寻址的最高物理地址位掩码。
    pub dma_mask: u64,
}

/// 以太网 MAC 与外部 PHY 芯片之间的物理接口连接模式。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PhyMode {
    /// 标准 RGMII 模式（芯片内部不附加时钟延迟）。
    #[default]
    Rgmii,
    /// RGMII-ID 模式（RX 与 TX 均启用 PHY 内部时钟延迟）。
    RgmiiId,
    /// RGMII-RXID 模式（仅 RX 启用 PHY 内部时钟延迟）。
    RgmiiRxid,
    /// RGMII-TXID 模式（仅 TX 启用 PHY 内部时钟延迟）。
    RgmiiTxid,
    /// 精简介质独立接口（RMII）模式。
    Rmii,
}

/// 板级晶振与锁相环（PLL）输入资源（用于 JH7110 等平台动态计算时钟树频率）。
#[derive(Clone, Copy, Debug, Default)]
pub struct BoardClockInputs {
    /// 板载外部主晶振频率（单位：Hz）。
    pub oscillator_hz: Option<u64>,
    /// 片上 PLL 控制寄存器的物理地址区间。
    pub pll_registers: Region,
}

/// 真实开发板（VisionFive 2 / LS2K1000LA）特有的片上控制器与板载外设资源集合。
#[derive(Clone, Copy, Debug, Default)]
pub struct BoardResources {
    /// 板级主晶振与 PLL 寄存器资源。
    pub clock_inputs: BoardClockInputs,
    /// 片上非一致性缓存控制器（如 SiFive Composable Cache）的物理地址区间。
    pub ccache: Region,
    /// 非一致性 DMA 刷缓存操作所需的缓存行字节大小（Cache Line Size，单位：字节）。
    pub cache_line_size: usize,
    /// 龙芯平台的两组本地 I/O 中断控制器（LIOINTC）资源。
    pub liointc: [LioIntcResources; 2],
    /// 电源管理控制器（PMC / ACPI PM）的物理地址区间。
    pub pmc: Region,
    /// 板载实时时钟（RTC）上线所需的时钟与复位配置资源。
    #[allow(
        dead_code,
        reason = "RISC-V RTC 准备消费真实时钟和复位资源，LoongArch RTC 不使用此板设置"
    )]
    pub rtc_setup: DeviceSetup,
    /// 基于 syscon 寄存器的系统重启操作描述。
    pub reboot: Option<SysconAction>,
    /// 基于 syscon 寄存器的系统关机断电操作描述。
    pub poweroff: Option<SysconAction>,
    /// 板载 SD/MMC 控制器资源（若存在且已启用）。
    pub mmc: Option<MmcResources>,
    /// 板载 SATA AHCI 控制器资源（若存在且已启用）。
    pub ahci: Option<AhciResources>,
    /// 板载千兆以太网（GMAC）控制器资源（若存在且已启用）。
    pub gmac: Option<GmacResources>,
}

/// PCI 主桥配置空间（Configuration Space）的硬件访问协议类型。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PciConfiguration {
    /// 当前平台未提供可用 PCI 主桥。
    #[default]
    Absent,
    /// 标准 PCIe ECAM（Enhanced Configuration Access Mechanism）线性窗口。
    Ecam,
    /// 龙芯 LS2K 标准配置窗口与扩展配置窗口分离访问协议。
    Loongson,
}

/// PCI 主桥声明的单条总线地址到 CPU 物理地址映射窗口（来自设备树 `ranges`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciRange {
    /// OpenFirmware PCI 地址空间属性标志字（`phys.hi`，区分 I/O、MEM32、MEM64 及预取属性）。
    pub flags: u32,
    /// 该窗口在 PCI 总线侧的起始地址（单位：字节）。
    pub bus_start: usize,
    /// 该窗口对应到 CPU 侧的物理地址区间 `[start, end)`。
    pub cpu: Region,
}

/// 单个启用态 VirtIO-MMIO 传输槽位的物理寄存器与中断描述。
#[derive(Clone, Copy, Debug, Default)]
pub struct VirtioMmioResource {
    /// VirtIO-MMIO 寄存器的物理地址区间。
    pub registers: Region,
    /// 该槽位在外部中断控制器上的中断源编号。
    pub interrupt_source: u32,
}

/// PCI 主桥 `interrupt-map` 中的单条传统中断（INTx）路由规则。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciIntxRoute {
    /// 经 `interrupt-map-mask` 掩码后的 4 单元子设备地址与引脚号（`[phys.hi, phys.mid, phys.lo, pin]`）。
    pub child: [u32; 4],
    /// 匹配成功后路由到的外部中断控制器中断源编号。
    pub source: u32,
}

/// PCI 主桥的配置空间窗口、BAR 映射窗口以及 INTx 中断路由表。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciResources {
    /// 主配置空间（ECAM 或龙芯 Type 0/1 低位配置窗口）的物理地址区间。
    pub configuration: Region,
    /// 配置空间访问协议类型。
    pub kind: PciConfiguration,
    /// 龙芯平台专用的扩展配置空间物理地址区间（标准 ECAM 下为空区间）。
    pub configuration_extended: Region,
    /// PCI 总线地址到 CPU 物理地址的转换窗口数组。
    pub ranges: [PciRange; MAX_PCI_RANGES],
    /// `ranges` 中的有效窗口数量。
    pub range_count: usize,
    /// 主桥管理的起始 PCI 总线号（通常为 0）。
    pub bus_start: u32,
    /// 主桥管理的结束 PCI 总线号（包含）。
    pub bus_end: u32,
    /// 设备树 `interrupt-map-mask` 规定的 4 单元匹配掩码。
    pub intx_mask: [u32; 4],
    /// 设备树 `interrupt-map` 解析出的 INTx 路由条目数组。
    pub intx_routes: [PciIntxRoute; 16],
    /// `intx_routes` 中的有效路由条目数量。
    pub intx_route_count: usize,
}

impl PciResources {
    /// 根据 PCI 设备的总线号（`bus`）、设备号（`device`）、功能号（`function`）和中断引脚（`pin`，1..=4）
    /// 查询主桥 `interrupt-map` 路由表，返回对应的外部中断控制器中断源编号。
    pub fn interrupt_source(
        &self,
        bus: u8,
        device: u8,
        function: u8,
        pin: u8,
    ) -> Result<u32, &'static str> {
        // Step 1: 校验总线号、设备号、功能号与 INTx 引脚号（1..=4）是否在主桥合法范围内。
        if (bus as u32) < self.bus_start
            || self.bus_end < bus as u32
            || PCI_DEVICE_COUNT <= device
            || PCI_FUNCTION_COUNT <= function
            || !(1..=PCI_INTX_PIN_COUNT).contains(&pin)
        {
            return Err("PCI INTx BDF or pin is out of range");
        }

        // Step 2: 按 OpenFirmware PCI phys.hi 格式拼装子地址，与 interrupt-map_mask 掩码匹配查找目标中断源。
        let child = [
            ((bus as u32) << PCI_BUS_SHIFT)
                | ((device as u32) << PCI_DEVICE_SHIFT)
                | ((function as u32) << PCI_FUNCTION_SHIFT),
            0,
            0,
            pin as u32,
        ];
        self.intx_routes[..self.intx_route_count]
            .iter()
            .find(|route| {
                (0..child.len()).all(|i| ((route.child[i] ^ child[i]) & self.intx_mask[i]) == 0)
            })
            .map(|route| route.source)
            .ok_or("PCI INTx has no host interrupt-map route")
    }
}

/// 由主核（BSP）在引导早期一次性解析并冻结发布的全局硬件与固件资源快照。
#[derive(Clone, Copy, Debug)]
pub struct BootInfo {
    /// 当前运行的硬件平台标识。
    pub platform: Platform,
    /// 固件进入内核入口时传入的原始 4 个寄存器参数。
    pub raw_args: [usize; 4],
    /// 主核（BSP）的硬件核编号（RISC-V hartid 或 LoongArch cpuid）。
    pub bsp_hardware_id: usize,
    /// 固件申报并启用的逻辑 CPU 核心总数。
    pub cpu_count: usize,
    /// 按逻辑 CPU 编号（`0..cpu_count`）排列的各核硬件 ID 表（0 号槽固定为 BSP）。
    pub hardware_ids: [usize; MAX_CPUS],
    /// `memory` 数组中的有效可用物理 RAM 区间数量。
    pub memory_count: usize,
    /// 固件声明的可用物理 RAM 区间列表。
    pub memory: [Region; MAX_MEMORY_REGIONS],
    /// `reserved` 数组中的已登记物理保留区数量。
    pub reserved_count: usize,
    /// 禁止物理页分配器分配的保留物理地址区间列表（含固件保留区、内核镜像、DTB、initrd）。
    pub reserved: [Region; MAX_RESERVED_REGIONS],
    /// 与 `reserved` 一一对应的映射属性标志：`true` 表示固件声明了 `no-map`，禁止纳入内核 RAM 线性直映窗口。
    pub reserved_no_map: [bool; MAX_RESERVED_REGIONS],
    /// 扁平设备树（DTB）在物理内存中的字节区间。
    pub dtb: Region,
    /// 可选的初始内存盘（initrd / initramfs）物理地址区间。
    pub initrd: Option<Region>,
    /// 控制台串口的寄存器宽度、时钟与波特率参数。
    pub uart: UartResources,
    /// 开发板专有外设与片上系统控制器资源。
    pub board: BoardResources,
    /// 串口与外部中断控制器物理窗口及中断源参数。
    pub interrupts: InterruptResources,
    /// 实时时钟（RTC）寄存器的物理地址区间（用于在引导阶段读取初始墙钟日历时间）。
    pub rtc: Region,
    /// PCI 主桥配置窗口、地址范围与中断路由资源。
    pub pci: PciResources,
    /// 固件声明的启用态 VirtIO-MMIO 槽位数组。
    pub virtio_mmio: [VirtioMmioResource; MAX_VIRTIO_MMIO],
    /// `virtio_mmio` 中的有效槽位数量。
    pub virtio_mmio_count: usize,
    /// 内核启动命令行（`bootargs`）的原始 UTF-8/ASCII 字节缓冲区。
    pub bootargs: [u8; MAX_BOOTARGS_BYTES],
    /// `bootargs` 中的实际有效字节长度（不含结尾 NUL 填充）。
    pub bootargs_len: usize,
    /// CPU 硬件计时器（RISC-V `time` / LoongArch 恒定频率计时器）的每秒计数频率（单位：Hz）。
    pub counter_frequency: u64,
}

impl BootInfo {
    /// 在 `pointer` 指向的未初始化静态槽中原地逐字段写入初始默认值。
    ///
    /// 直接通过原始指针向目标静态内存写入，避免在 64 KiB 早期启动栈上先构造大体积 `BootInfo` 临时变量再按值拷贝。
    ///
    /// # Safety
    /// - `pointer` 必须指向对齐良好、可写且由当前主核唯一独占的 `BootInfo` 大小内存块；
    /// - 写入完成前不存在任何并发读取者，且目标内存块中无需要调用 `Drop` 的旧值。
    pub(crate) unsafe fn initialize_at(
        pointer: *mut Self,
        raw_args: [usize; 4],
        bsp_hardware_id: usize,
    ) {
        // SAFETY:
        // 1. 调用方保证了 `pointer` 指向合法对齐且独占的 `BOOT_STORAGE` 静态内存；
        // 2. 使用 `ptr::addr_of_mut!` 直接定位各字段地址并调用 `write`，全程不会对未初始化内存创建 Rust 引用。
        unsafe {
            ptr::addr_of_mut!((*pointer).platform).write(crate::platform::current());
            ptr::addr_of_mut!((*pointer).raw_args).write(raw_args);
            ptr::addr_of_mut!((*pointer).bsp_hardware_id).write(bsp_hardware_id);
            ptr::addr_of_mut!((*pointer).cpu_count).write(0);
            ptr::addr_of_mut!((*pointer).hardware_ids).write([0; MAX_CPUS]);
            ptr::addr_of_mut!((*pointer).memory_count).write(0);
            ptr::addr_of_mut!((*pointer).memory).write([Region::default(); MAX_MEMORY_REGIONS]);
            ptr::addr_of_mut!((*pointer).reserved_count).write(0);
            ptr::addr_of_mut!((*pointer).reserved).write([Region::default(); MAX_RESERVED_REGIONS]);
            ptr::addr_of_mut!((*pointer).reserved_no_map).write([false; MAX_RESERVED_REGIONS]);
            ptr::addr_of_mut!((*pointer).dtb).write(Region::default());
            ptr::addr_of_mut!((*pointer).initrd).write(None);
            ptr::addr_of_mut!((*pointer).uart).write(UartResources::default());
            ptr::addr_of_mut!((*pointer).board).write(BoardResources::default());
            ptr::addr_of_mut!((*pointer).interrupts).write(InterruptResources::default());
            ptr::addr_of_mut!((*pointer).rtc).write(Region::default());
            ptr::addr_of_mut!((*pointer).pci).write(PciResources::default());
            ptr::addr_of_mut!((*pointer).virtio_mmio)
                .write([VirtioMmioResource::default(); MAX_VIRTIO_MMIO]);
            ptr::addr_of_mut!((*pointer).virtio_mmio_count).write(0);
            ptr::addr_of_mut!((*pointer).bootargs).write([0; MAX_BOOTARGS_BYTES]);
            ptr::addr_of_mut!((*pointer).bootargs_len).write(0);
            ptr::addr_of_mut!((*pointer).counter_frequency).write(0);
        }
    }

    /// 返回内核启动命令行（`bootargs`）的有效字节切片（不含末尾未使用的填充字节）。
    pub fn bootargs(&self) -> &[u8] {
        &self.bootargs[..self.bootargs_len]
    }

    /// 登记一段普通物理保留区（禁止页分配器分配，但仍保留在内核 RAM 线性直映窗口中）。
    pub fn reserve(&mut self, region: Region) -> Result<(), &'static str> {
        self.reserve_with_attribute(region, false)
    }

    /// 登记一段 `no-map` 固件保留区（同时禁止页分配器分配以及建立内核 RAM 线性直映）。
    pub fn reserve_no_map(&mut self, region: Region) -> Result<(), &'static str> {
        self.reserve_with_attribute(region, true)
    }

    /// 将物理保留区间及其 `no_map` 属性追加到保留表中；若超过 [`MAX_RESERVED_REGIONS`] 容量上限则返回错误且不修改原表。
    fn reserve_with_attribute(&mut self, region: Region, no_map: bool) -> Result<(), &'static str> {
        if MAX_RESERVED_REGIONS <= self.reserved_count {
            return Err("boot reservation capacity exceeded");
        }

        self.reserved[self.reserved_count] = region;
        self.reserved_no_map[self.reserved_count] = no_map;
        self.reserved_count += 1;
        Ok(())
    }
}

/// 存放全局唯一 [`BootInfo`] 实例的静态未初始化内存槽。
static mut BOOT_STORAGE: MaybeUninit<BootInfo> = MaybeUninit::uninit();
/// 保存已冻结发布的 [`BootInfo`] 静态只读引用（由 `spin::Once` 保证只发布一次）。
static BOOT_INFO: Once<&'static BootInfo> = Once::new();

/// 在静态槽 [`BOOT_STORAGE`] 中原地初始化 `BootInfo`，并返回供早期平台发现阶段填充的唯一可变引用。
///
/// # Safety
/// - 必须仅由主核（BSP）在中断关闭、从核（AP）未唤醒的早期引导阶段调用一次；
/// - 调用时 `BOOT_STORAGE` 尚无任何活跃引用，返回的 `&'static mut BootInfo` 在传入 [`publish`] 前不得产生别名。
pub(crate) unsafe fn prepare(
    raw_args: [usize; 4],
    bsp_hardware_id: usize,
) -> &'static mut BootInfo {
    assert!(BOOT_INFO.get().is_none(), "BootInfo is already published");
    let pointer = ptr::addr_of_mut!(BOOT_STORAGE).cast::<BootInfo>();
    // SAFETY:
    // 1. 上方 `assert!` 与主核单核关中断契约保证了 `BOOT_STORAGE` 尚未发布且由当前执行流独占；
    // 2. `BootInfo::initialize_at` 已将全部字段完整写入合法初值，随后将 `pointer` 转为唯一可变引用是安全的。
    unsafe {
        BootInfo::initialize_at(pointer, raw_args, bsp_hardware_id);
        &mut *pointer
    }
}

/// 根据链接脚本导出的 `__kernel_virt_start` 与 `__kernel_virt_end` 符号，
/// 计算整个内核镜像（含代码、数据、BSS、启动栈与早期页表）占用的物理地址区间。
pub fn kernel_region() -> Region {
    extern "C" {
        static __kernel_virt_start: u8;
        static __kernel_virt_end: u8;
    }
    Region {
        start: crate::arch::virt_to_phys(core::ptr::addr_of!(__kernel_virt_start) as usize),
        end: crate::arch::virt_to_phys(core::ptr::addr_of!(__kernel_virt_end) as usize),
    }
}

/// 消费早期可变引用，将填充完毕的 [`BootInfo`] 冻结并发布为全局只读快照。
///
/// # Safety
/// - 必须仅由主核（BSP）在从核未唤醒、本地中断关闭时调用一次；
/// - `info` 必须是由 [`prepare`] 返回的唯一可变引用，且已完成全部固件资源校验与内核保留区登记；
/// - 本函数调用后，严禁再通过任何指针修改 [`BOOT_STORAGE`]。
pub unsafe fn publish(info: &'static mut BootInfo) {
    assert!(BOOT_INFO.get().is_none(), "BootInfo is already published");
    let info: &'static BootInfo = info;
    BOOT_INFO.call_once(|| info);
}

/// 获取已发布的全局 [`BootInfo`] 只读引用；若在 [`publish`] 之前调用则立即触发 panic。
pub fn info() -> &'static BootInfo {
    BOOT_INFO
        .get()
        .copied()
        .expect("BootInfo has not been published")
}
