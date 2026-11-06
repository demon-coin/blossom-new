//! PCI / PCIe 根总线（Root Bus）单次枚举、BAR 地址空间分配与配置空间访问管理。
//!
//! ## 模块职责与架构定位
//! 本模块由顶层 [`crate::drivers::discover`] 在普通初始化任务中调用，负责根据 [`boot::BootInfo`]
//! 提供的 PCI 主桥资源描述（标准 ECAM 或龙芯 LS2K/LS7A 扩展配置空间 CFG0/CFG1），
//! 对根总线（`bus = 0`）执行单次全功能扫描、BAR 尺寸探测与物理地址分配，并产出不可变的
//! [`PciInventory`] 供后续 VirtIO-PCI、AHCI 与 GMAC 驱动查询使用。
//!
//! ## 核心设计与地址/配置空间布局
//! 1. **双配置访问机制（[`HostCam`]）**：
//!    - `Ecam`：标准 PCIe ECAM 线性内存映射配置空间。
//!    - `Loongson`：龙芯桥片专用配置空间（CFG0/CFG1），其扩展配置偏移（`extended = true`）位域编码如下：
//!
//! ```text
//!  28         24 27       16 15     11 10     8 7               0
//! +-------------+-----------+---------+--------+-----------------+
//! | Type1(1<<28)| ExtOffset |  Device |  Func  |  BaseRegister   |
//! | (bus != 0)  |  [11:8]   |  [4:0]  | [2:0]  |      [7:0]      |
//! +-------------+-----------+---------+--------+-----------------+
//!               (注: bus != 0 时 bits[23:16] 还会或上 bus << 16)
//! ```
//!
//! 2. **BAR 探测、分配与回滚状态机（[`PciInventory::discover`]）**：
//!
//! ```text
//! collect_devices (扫描 Bus 0，读取身份、Capability 链、中断路由与原始 BAR/Command)
//!    │
//!    ▼
//! configure_resources
//!    ├─► Step 1: 关闭所有候选功能的 IO_SPACE / MEMORY_SPACE / BUS_MASTER 解码并回读确认
//!    ├─► Step 2: 探测各功能 BAR 的类型与尺寸 (root.bars)
//!    ├─► Step 3: 校验并登记固件已分配且落在主桥窗口内的非零 BAR 区间 (reserve)
//!    ├─► Step 4: 为未分配或越界的内存 BAR 在 MEM32 窗口内按 2^n 对齐寻找空闲区间 (allocate)
//!    ├─► Step 5: 预先校验板级原生 AHCI / GMAC 的 BAR 与中断路由一致性 (resolve_registers)
//!    ├─► Step 6: 将最终 BAR 总线地址写回硬件寄存器并回读核对，随后开启 MEMORY_SPACE | BUS_MASTER
//!    │
//!    └─► 若中途任何一步返回 Err ──► restore_configuration (恢复原始 BAR 与 Command)
//! ```
//!
//! ## 锁顺序与并发契约
//! - 全局原子标志 [`DISCOVERED`]（`AcqRel`）保证 [`PciInventory::discover`] 在整个内核生命周期内至多成功执行一次。
//! - 所有对 PCI 配置空间的可写操作（BAR 尺寸探测、地址编程、Command 启停）均在 [`PciInventory::discover`]
//!   独占持有 `&mut PciRoot<HostCam>` 期间完成；枚举结束后，`PciInventory` 仅对外暴露只读切片与受限的只读配置字查询。

use alloc::vec::Vec;
use core::convert::TryFrom;
use core::sync::atomic::{AtomicBool, Ordering};
use virtio_drivers::transport::pci::bus::{
    BarInfo, Cam, Command, ConfigurationAccess, DeviceFunction, DeviceFunctionInfo, HeaderType,
    MemoryBarType, MmioCam, PciRoot,
};

use crate::{
    boot::{
        self, AhciResources, BootInfo, GmacResources, PciConfiguration, PciFunctionResource,
        PciRange, PciResources, Region, PCI_BUS_SHIFT, PCI_DEVICE_COUNT, PCI_DEVICE_SHIFT,
        PCI_FUNCTION_COUNT, PCI_FUNCTION_SHIFT, PCI_INTX_PIN_COUNT, PCI_SPACE_CODE_MASK,
        PCI_SPACE_IO, PCI_SPACE_MEM32, PCI_SPACE_MEM64,
    },
    drivers::{mmio_region_address, ProbeError},
    irq::{self, IrqNumber},
    task,
};

static DISCOVERED: AtomicBool = AtomicBool::new(false);

const PCI_REGISTER_BYTES: usize = core::mem::size_of::<u32>();
const PCI_CFG_SPACE_BYTES: u16 = 256;
const PCI_CFG_EXP_SPACE_BYTES: u16 = 4096;
const PCI_HEADER_BYTES: u8 = 0x40;
const PCI_BAR_COUNT: usize = 6;
const PCI_CAPABILITY_CAPACITY: usize =
    (PCI_CFG_SPACE_BYTES as usize - PCI_HEADER_BYTES as usize) / PCI_REGISTER_BYTES;

const PCI_VENDOR_WORD: u8 = 0x00;
const PCI_CLASS_REVISION_WORD: u8 = 0x08;
const PCI_CACHE_LINE_WORD: u8 = 0x0c;
const PCI_SUBSYSTEM_WORD: u8 = 0x2c;
const PCI_CAPABILITY_LIST_WORD: u8 = 0x34;
const PCI_INTERRUPT_WORD: u8 = 0x3c;
const PCI_BAR0: u8 = 0x10;
const PCI_BAR1: u8 = PCI_BAR0 + PCI_REGISTER_BYTES as u8;
const PCI_BAR_IO_SPACE: u32 = 1;
const PCI_BAR_MEMORY_TYPE_MASK: u32 = 6;
const PCI_BAR_MEMORY_TYPE_32: u32 = 0;
const PCI_BAR_MEMORY_TYPE_64: u32 = 4;
const PCI_BAR_MEMORY_ADDRESS_MASK: u32 = !15;

const PCI_VENDOR_LOONGSON: u16 = 0x0014;
const PCI_DEVICE_LOONGSON_AHCI: u16 = 0x7a08;
const PCI_DEVICE_LOONGSON_GMAC: u16 = 0x7a03;
const PCI_DEVICE_LOONGSON_GMAC2: u16 = 0x7a23;
const PCI_CLASS_AHCI: u32 = 0x010601;

const LOONGSON_ROOT_PORT_FIRST: u8 = 9;
const LOONGSON_ROOT_PORT_LAST: u8 = 20;
const LOONGSON_CFG_REGISTER_MASK: usize = 0xff;
const LOONGSON_CFG_EXT_REGISTER_MASK: usize = 0xf00;
const LOONGSON_CFG_EXT_REGISTER_SHIFT: u32 = 16;
const LOONGSON_CFG0_TYPE1: usize = 1 << 24;
const LOONGSON_CFG1_TYPE1: usize = 1 << 28;

const IO_DOMAIN: u8 = 1;
const MEMORY_DOMAIN: u8 = 2;

/// 已完成地址分配与 CPU 物理地址换算的 PCI 内存基址寄存器（Memory BAR）描述。
///
/// 若某 64 位内存 BAR 占用相邻两个 32 位寄存器槽位（`BAR[i]` 与 `BAR[i+1]`），
/// 则完整信息保存在低槽 `bars[i]` 中，高槽 `bars[i+1]` 保持为 `None`。
#[derive(Clone, Copy, Debug)]
pub struct PciBar {
    /// 写入 PCI BAR 寄存器的 PCI 总线域起始地址（字节单位）。
    pub bus_address: u64,
    /// 经主桥 `ranges` 窗口换算后的 CPU 物理地址区间（字节单位）。
    pub physical: Region,
    #[cfg_attr(
        feature = "ktest",
        expect(
            dead_code,
            reason = "普通 init 的 PCI BAR 资源日志读取预取属性，ktest 不运行该初始化入口"
        )
    )]
    /// 该内存 BAR 是否支持预取（Prefetchable）。
    pub prefetchable: bool,
}

/// PCI 配置空间 Capability 链表中的单个能力项头部快照。
#[derive(Clone, Copy, Debug)]
pub struct PciCapability {
    /// 该 Capability 在 256 字节标准配置空间内的字节偏移量。
    pub offset: u8,
    /// PCI SIG 定义的 Capability ID（如 `0x09` 为 Vendor-Specific / VirtIO）。
    pub id: u8,
    /// 紧随 `id` 与 `next` 指针之后的 16 位能力私有头部字段。
    pub private_header: u16,
}

/// PCI 根总线上单个已枚举并完成 BAR 配置的功能（Function）不可变资源快照。
#[derive(Debug)]
pub struct PciDevice {
    /// PCI 总线号（根总线固定为 `0`）。
    pub bus: u8,
    /// PCI 设备号（Slot / Device，`0..32`）。
    pub device: u8,
    /// PCI 功能号（Function，`0..8`）。
    pub function: u8,
    /// 硬件厂商标识（Vendor ID）。
    pub vendor_id: u16,
    /// 硬件设备标识（Device ID）。
    pub device_id: u16,
    #[cfg_attr(
        feature = "ktest",
        expect(
            dead_code,
            reason = "普通 init 的 PCI 功能资源日志读取原始头类型，ktest 不运行该初始化入口"
        )
    )]
    /// 配置空间 `0x0e` 字节处的原始 Header Type 字段（含多功能标志位 Bit 7）。
    pub header_type: u8,
    /// 24 位类别码（`(class << 16) | (subclass << 8) | prog_if`），用于按标准类别匹配 AHCI 等控制器。
    class_code: u32,
    /// 最多 6 个标准 32 位 BAR 槽位的内存映射信息（I/O BAR、未实现槽位或 64 位 BAR 高半槽为 `None`）。
    pub bars: [Option<PciBar>; PCI_BAR_COUNT],
    /// 从配置空间 Capability 链表按序提取的能力项定长数组。
    pub capabilities: [Option<PciCapability>; PCI_CAPABILITY_CAPACITY],
    /// `capabilities` 数组中有效能力项的数量。
    pub capability_count: usize,
    /// 已映射至内核外部中断控制器域的 PCI INTx 中断号（不使用中断的功能为 `None`）。
    pub interrupt: Option<IrqNumber>,
}

impl PciDevice {
    /// 构造 `virtio_drivers` 使用的 `(bus, device, function)` 配置空间地址标识。
    fn bdf(&self) -> DeviceFunction {
        DeviceFunction {
            bus: self.bus,
            device: self.device,
            function: self.function,
        }
    }
}

/// PCI 根总线唯一配置空间所有者与已枚举设备资源清单。
pub struct PciInventory {
    /// PCI 根总线配置空间访问句柄；当平台无 PCI 主桥（[`PciConfiguration::Absent`]）时为 `None`。
    root: Option<PciRoot<HostCam>>,
    /// 按 `(bus, device, function)` 升序排列的已配置根总线设备列表。
    devices: Vec<PciDevice>,
}

/// BAR 探测与分配阶段暂存单个候选功能及其原始硬件寄存器状态的内部工作项。
struct PendingDevice {
    /// 正在构建的设备资源描述（其 `bars` 数组在 `configure_resources` 中填充）。
    device: PciDevice,
    /// 枚举开始前读取的原始 PCI `Command` 寄存器值，用于配置失败时无损回滚。
    command: Command,
    /// 枚举开始前读取的 6 个原始 32 位 BAR 寄存器字，用于配置失败时恢复固件地址。
    original_bars: [u32; PCI_BAR_COUNT],
    /// 关闭地址解码后通过 `virtio_drivers` 探测到的各槽位 BAR 类型与尺寸信息。
    bar_info: [Option<BarInfo>; PCI_BAR_COUNT],
}

/// 记录 PCI I/O 域或内存域中已被现有或新分配 BAR 占用的总线地址半开区间 `[start, end)`。
#[derive(Clone, Copy)]
struct Occupied {
    /// 地址空间域标识（[`IO_DOMAIN`] 表示 I/O 空间，[`MEMORY_DOMAIN`] 表示 32/64 位内存空间）。
    domain: u8,
    /// 已占用区间在 PCI 总线地址空间中的起始地址（字节单位，含）。
    start: u64,
    /// 已占用区间在 PCI 总线地址空间中的结束地址（字节单位，不含）。
    end: u64,
}

/// 底层 PCI 主桥配置空间访问机制封装。
enum HostCam {
    /// 标准 PCIe ECAM 内存映射配置空间访问器。
    Ecam(MmioCam<'static>),
    /// 龙芯 LS2K/LS7A 桥片专用扩展配置空间（CFG1）访问器。
    Loongson {
        /// 龙芯扩展配置窗口在内核强序 MMIO 区域中的虚拟基地址。
        base: usize,
        /// 龙芯扩展配置窗口的总字节长度。
        length: usize,
        /// 主桥支持的最大有效总线号（含）。
        bus_end: u8,
    },
}

/// 计算龙芯 CFG0（`extended = false`）或 CFG1（`extended = true`）配置空间中指定 `(bdf, offset)` 的字节偏移量。
///
/// 若设备/功能号越界、`offset` 未按 4 字节对齐、超出配置空间上限，或命中了龙芯根总线上硬件禁止访问的
/// Root Port 非零功能（`bus == 0 && device in 9..=20 && function != 0`），则返回 `None`。
fn loongson_offset(bdf: DeviceFunction, offset: u16, extended: bool) -> Option<usize> {
    if PCI_DEVICE_COUNT <= bdf.device
        || PCI_FUNCTION_COUNT <= bdf.function
        || offset & (PCI_REGISTER_BYTES as u16 - 1) != 0
        || (!extended && PCI_CFG_SPACE_BYTES <= offset)
        || PCI_CFG_EXP_SPACE_BYTES <= offset
        || (bdf.bus == 0
            && (LOONGSON_ROOT_PORT_FIRST..=LOONGSON_ROOT_PORT_LAST).contains(&bdf.device)
            && bdf.function != 0)
    {
        return None;
    }

    let devfn =
        ((bdf.device as usize) << (PCI_DEVICE_SHIFT - PCI_FUNCTION_SHIFT)) | bdf.function as usize;
    let mut address =
        (devfn << PCI_FUNCTION_SHIFT) | (offset as usize & LOONGSON_CFG_REGISTER_MASK);
    if extended {
        address |=
            (offset as usize & LOONGSON_CFG_EXT_REGISTER_MASK) << LOONGSON_CFG_EXT_REGISTER_SHIFT;
    }
    if bdf.bus != 0 {
        address |= (bdf.bus as usize) << PCI_BUS_SHIFT;
        address |= if extended {
            LOONGSON_CFG1_TYPE1
        } else {
            LOONGSON_CFG0_TYPE1
        };
    }
    Some(address)
}

impl ConfigurationAccess for HostCam {
    /// 从配置空间读取 `bdf` 功能在 `offset` 处的 32 位寄存器字；若超出主桥窗口或命中隐藏功能则按 PCI 空槽语义返回 `u32::MAX`。
    fn read_word(&self, bdf: DeviceFunction, offset: u8) -> u32 {
        match self {
            Self::Ecam(cam) => cam.read_word(bdf, offset),
            Self::Loongson {
                base,
                length,
                bus_end,
            } => {
                if *bus_end < bdf.bus {
                    return u32::MAX;
                }
                let Some(at) = loongson_offset(bdf, offset as u16, true) else {
                    return u32::MAX;
                };
                if *length < at + PCI_REGISTER_BYTES {
                    return u32::MAX;
                }
                // SAFETY:
                // 1. `PciInventory::discover` 已通过 `mmio_region_address` 验证 `[base, base + length)` 拥有永久强序 MMIO 映射；
                // 2. `loongson_offset` 已校验 `offset` 按 4 字节对齐且 `at + 4 <= length` 已在上方确认，目标 32 位字完全处于映射窗口内。
                unsafe { ((*base + at) as *const u32).read_volatile() }
            }
        }
    }

    /// 向配置空间中 `bdf` 功能的 `offset` 处写入 32 位寄存器字（仅在持有 `&mut self` 期间调用）。
    fn write_word(&mut self, bdf: DeviceFunction, offset: u8, data: u32) {
        match self {
            Self::Ecam(cam) => cam.write_word(bdf, offset, data),
            Self::Loongson {
                base,
                length,
                bus_end,
            } => {
                assert!(bdf.bus <= *bus_end, "PCI bus exceeds Loongson host");
                let at = loongson_offset(bdf, offset as u16, true)
                    .expect("invalid Loongson configuration write");
                assert!(
                    at + PCI_REGISTER_BYTES <= *length,
                    "Loongson configuration write exceeds CFG1"
                );
                // SAFETY:
                // 1. 调用方独占持有 `&mut self`，保证当前没有并发配置空间写入；
                // 2. `loongson_offset` 与上方 `assert!` 已确认 `at` 按 4 字节对齐且 `[at, at + 4)` 完全位于永久强序映射 `[base, base + length)` 内。
                unsafe { ((*base + at) as *mut u32).write_volatile(data) };
            }
        }
    }

    /// 为 `PciRoot` 内部只读枚举器创建指向同一配置空间映射的轻量级副本。
    ///
    /// # Safety
    /// 1. 调用方只能通过克隆出的副本执行无副作用的只读配置空间访问；
    /// 2. 在副本存活期间，不得与原 [`HostCam`] 所有者的配置空间写入操作并发执行。
    unsafe fn unsafe_clone(&self) -> Self {
        match self {
            // SAFETY: 外层调用方已承诺遵守 `ConfigurationAccess::unsafe_clone` 的只读且无并发写契约，且底层 ECAM 映射在内核生命周期内永久有效。
            Self::Ecam(cam) => Self::Ecam(unsafe { cam.unsafe_clone() }),
            Self::Loongson {
                base,
                length,
                bus_end,
            } => Self::Loongson {
                base: *base,
                length: *length,
                bus_end: *bus_end,
            },
        }
    }
}

impl PciInventory {
    /// 在普通初始化任务中单次枚举全局启动信息发布的 PCI 主桥，完成 BAR 分配并返回设备库存。
    ///
    /// 若配置过程中任何设备的 BAR、中断或原生窗口校验失败，会自动回滚所有已扫描设备的原始 BAR 与 `Command` 寄存器。
    pub fn discover(info: &'static BootInfo) -> Result<Self, ProbeError> {
        // Step 1: 确认处于普通任务上下文，校验全局唯一的 BootInfo 引用并通过 DISCOVERED 原子标志防止重复枚举。
        drop(task::current_task());
        if !core::ptr::eq(info, boot::info()) {
            return Err(ProbeError::Resource(
                "PCI requires the published boot snapshot",
            ));
        }
        if DISCOVERED.swap(true, Ordering::AcqRel) {
            return Err(ProbeError::AlreadyDiscovered);
        }
        if info.pci.kind == PciConfiguration::Absent {
            return Ok(Self {
                root: None,
                devices: Vec::new(),
            });
        }
        if info.pci.bus_start != 0 {
            return Err(ProbeError::Resource("PCI root bus must start at zero"));
        }

        // Step 2: 根据主桥类型（标准 ECAM 或龙芯扩展 CFG1）转换配置空间虚拟基址并构造 HostCam。
        let configuration = match info.pci.kind {
            PciConfiguration::Ecam => info.pci.configuration,
            PciConfiguration::Loongson => info.pci.configuration_extended,
            PciConfiguration::Absent => unreachable!(),
        };
        let virtual_address = mmio_region_address(configuration)?;
        let cam = match info.pci.kind {
            PciConfiguration::Ecam => {
                // SAFETY:
                // 1. `mmio_region_address` 已验证 `configuration` 非空且映射到内核永久强序 MMIO 虚拟地址 `virtual_address`；
                // 2. `DISCOVERED` 原子标志保证全局仅构造唯一一个 `PciInventory`，独占管理该 ECAM 窗口直至关机。
                HostCam::Ecam(unsafe {
                    MmioCam::new(
                        virtual_address as *mut u8,
                        configuration.end - configuration.start,
                        Cam::Ecam,
                    )?
                })
            }
            PciConfiguration::Loongson => HostCam::Loongson {
                base: virtual_address,
                length: configuration.end - configuration.start,
                bus_end: info.pci.bus_end as u8,
            },
            PciConfiguration::Absent => unreachable!(),
        };

        // Step 3: 扫描根总线收集候选功能，并在修改硬件 BAR 前预先分配好结果向量与地址占用表的堆内存。
        let mut root = PciRoot::new(cam);
        let mut pending = collect_devices(&root, info)?;
        let mut devices = Vec::new();
        devices
            .try_reserve_exact(pending.len())
            .map_err(|_| ProbeError::NoMemory)?;
        let mut occupied = Vec::new();
        let capacity = pending
            .len()
            .checked_mul(PCI_BAR_COUNT)
            .ok_or(ProbeError::NoMemory)?;
        occupied
            .try_reserve_exact(capacity)
            .map_err(|_| ProbeError::NoMemory)?;

        // Step 4: 执行全量 BAR 探测、地址分配与 Command 使能；若中途出错则完整恢复原始配置状态。
        if let Err(error) = configure_resources(&mut root, &mut pending, &mut occupied, info) {
            restore_configuration(&mut root, &pending);
            return Err(error);
        }

        for entry in pending {
            devices.push(entry.device);
        }
        Ok(Self {
            root: Some(root),
            devices,
        })
    }

    /// 返回按 `(bus, device, function)` 升序排列的根总线已配置设备只读切片。
    pub fn devices(&self) -> &[PciDevice] {
        &self.devices
    }

    /// 在已配置的 PCI 库存中核对并解析板级 AHCI 控制器的寄存器物理区间与中断路由。
    pub fn resolve_ahci(&self, resources: &AhciResources) -> Result<AhciResources, ProbeError> {
        let mut resolved = *resources;
        resolved.registers = resolve_registers(
            self.devices.iter(),
            resources.pci,
            resources.registers,
            resources.interrupt_source,
            resources.dma_coherent,
            ahci_bar,
        )?;
        Ok(resolved)
    }

    /// 在已配置的 PCI 库存中核对并解析板级 GMAC 网卡的寄存器物理区间与中断路由。
    pub fn resolve_gmac(&self, resources: &GmacResources) -> Result<GmacResources, ProbeError> {
        let mut resolved = *resources;
        resolved.registers = resolve_registers(
            self.devices.iter(),
            resources.pci,
            resources.registers,
            resources.interrupt_source,
            resources.dma_coherent,
            gmac_bar,
        )?;
        Ok(resolved)
    }

    /// 读取归属于本库存的指定 `device` 在 `offset` 处的 32 位只读配置空间字。
    ///
    /// 仅允许访问 4 字节对齐的身份、Capability 链表指针、中断字段及 `0x40` 之后的 Capability 数据区，
    /// 拒绝访问外来 `PciDevice` 引用或可能带有读清零（Read-to-Clear）副作用的控制/状态字。
    pub fn read_config_word(&self, device: &PciDevice, offset: u8) -> Result<u32, ProbeError> {
        if offset & (PCI_REGISTER_BYTES as u8 - 1) != 0
            || !self
                .devices
                .iter()
                .any(|entry| core::ptr::eq(entry, device))
        {
            return Err(ProbeError::Resource(
                "invalid selected PCI configuration access",
            ));
        }
        // 限制标准配置头（0x00..0x40）内仅可读取无副作用的身份与元数据字，防止外部误触状态或控制寄存器。
        if !matches!(
            offset,
            PCI_VENDOR_WORD
                | PCI_CLASS_REVISION_WORD
                | PCI_CACHE_LINE_WORD
                | PCI_SUBSYSTEM_WORD
                | PCI_CAPABILITY_LIST_WORD
                | PCI_INTERRUPT_WORD
        ) && offset < PCI_HEADER_BYTES
        {
            return Err(ProbeError::Resource(
                "PCI configuration field is not read-only inventory data",
            ));
        }
        Ok(self
            .root
            .as_ref()
            .ok_or(ProbeError::Resource("PCI host is absent"))?
            .read_config_word(device.bdf(), offset))
    }
}

/// 根据 PCI 厂商、设备 ID 与类别码确定 AHCI 控制器使用的内存 BAR 槽位索引（龙芯集成 AHCI 使用 BAR0，标准 AHCI 1.0 使用 BAR5）。
fn ahci_bar(vendor: u16, device: u16, class_code: u32) -> Result<usize, ProbeError> {
    if vendor == PCI_VENDOR_LOONGSON && device == PCI_DEVICE_LOONGSON_AHCI {
        return Ok(0);
    }
    if class_code == PCI_CLASS_AHCI {
        return Ok(5);
    }
    Err(ProbeError::Resource(
        "selected PCI function is not a supported AHCI device",
    ))
}

/// 根据 PCI 厂商与设备 ID 确定龙芯单通道 GMAC 控制器使用的内存 BAR 槽位索引（固定使用 BAR0）。
fn gmac_bar(vendor: u16, device: u16, _class_code: u32) -> Result<usize, ProbeError> {
    if vendor == PCI_VENDOR_LOONGSON
        && matches!(device, PCI_DEVICE_LOONGSON_GMAC | PCI_DEVICE_LOONGSON_GMAC2)
    {
        return Ok(0);
    }
    Err(ProbeError::Resource(
        "selected PCI function is not a supported GMAC device",
    ))
}

/// 校验固件直接描述的寄存器窗口 `described` 与 PCI `bar` 物理窗口的一致性，并返回完整的 `bar` 物理区间。
fn normalize_window(bar: Region, described: Region, pci: bool) -> Result<Region, ProbeError> {
    if bar.end <= bar.start
        || (!(pci && described == Region::default())
            && (described.start != bar.start
                || described.end <= described.start
                || bar.end < described.end))
    {
        return Err(ProbeError::Resource(
            "native register window differs from selected PCI BAR",
        ));
    }
    Ok(bar)
}

/// 在已枚举的 PCI 设备列表中按显式 BDF 或寄存器物理窗口匹配唯一个原生外设（AHCI/GMAC），并校验其中断路由与 MMIO 可访问性；纯非 PCI 的直连 MMIO 外设则原样返回其寄存器区间。
fn resolve_registers<'a>(
    devices: impl Iterator<Item = &'a PciDevice>,
    selected: Option<PciFunctionResource>,
    registers: Region,
    source: u32,
    coherent: bool,
    select_bar: fn(u16, u16, u32) -> Result<usize, ProbeError>,
) -> Result<Region, ProbeError> {
    let interrupt = irq::map_source(source)?;
    if let Some(identity) = selected {
        if identity.interrupt_source != source || identity.dma_coherent != coherent {
            return Err(ProbeError::Resource(
                "native PCI identity and device resources differ",
            ));
        }
    } else if registers.end <= registers.start {
        return Err(ProbeError::Resource(
            "native MMIO register window is missing",
        ));
    }

    let mut found = None;
    for device in devices {
        let index = if let Some(identity) = selected {
            if device.bus != identity.bus
                || device.device != identity.device
                || device.function != identity.function
            {
                continue;
            }
            select_bar(device.vendor_id, device.device_id, device.class_code)?
        } else {
            let Ok(index) = select_bar(device.vendor_id, device.device_id, device.class_code)
            else {
                continue;
            };
            if !device.bars[index].is_some_and(|bar| bar.physical.overlaps(registers)) {
                continue;
            }
            index
        };
        if found.is_some() {
            return Err(ProbeError::Resource("native PCI resource is not unique"));
        }
        let bar = device.bars[index].ok_or(ProbeError::Resource(
            "selected native PCI memory BAR is absent",
        ))?;
        let physical = normalize_window(bar.physical, registers, selected.is_some())?;
        if device.interrupt != Some(interrupt) {
            return Err(ProbeError::Resource(
                "native PCI interrupt differs from device resources",
            ));
        }
        found = Some(physical);
    }

    let physical = match found {
        Some(physical) => physical,
        None if selected.is_some() => {
            return Err(ProbeError::Resource(
                "selected native PCI function is absent",
            ));
        }
        None => registers,
    };
    mmio_region_address(physical)?;
    Ok(physical)
}

/// 扫描根总线（`bus = 0`）上的所有功能，在修改硬件配置前收集设备身份、Capability 链表、中断映射以及原始 BAR/`Command` 寄存器快照。
fn collect_devices(
    root: &PciRoot<HostCam>,
    info: &BootInfo,
) -> Result<Vec<PendingDevice>, ProbeError> {
    let mut pending = Vec::new();
    for (bdf, identity) in root.enumerate_bus(0) {
        let native_source = if info.pci.kind == PciConfiguration::Loongson {
            let source = selected_native(root, info, bdf, &identity)?;
            if source.is_none() {
                continue;
            }
            source
        } else {
            None
        };
        if identity.header_type != HeaderType::Standard {
            return Err(ProbeError::Resource(
                "PCI bridge or unsupported header is outside root-bus discovery",
            ));
        }

        pending.try_reserve(1).map_err(|_| ProbeError::NoMemory)?;
        let header_type = (root.read_config_word(bdf, PCI_CACHE_LINE_WORD) >> u16::BITS) as u8;
        let pin = (root.read_config_word(bdf, PCI_INTERRUPT_WORD) >> u8::BITS) as u8;
        let interrupt = if let Some(source) = native_source {
            Some(irq::map_source(source)?)
        } else if pin == 0 {
            None
        } else {
            let source = info
                .pci
                .interrupt_source(bdf.bus, bdf.device, bdf.function, pin)
                .map_err(ProbeError::Resource)?;
            Some(irq::map_source(source)?)
        };

        let mut capabilities = [None; PCI_CAPABILITY_CAPACITY];
        let mut capability_count = 0;
        for capability in root.capabilities(bdf) {
            let capability = capability?;
            if capability_count == capabilities.len() {
                return Err(ProbeError::Resource("PCI capability capacity exceeded"));
            }
            capabilities[capability_count] = Some(PciCapability {
                offset: capability.offset,
                id: capability.id,
                private_header: capability.private_header,
            });
            capability_count += 1;
        }

        let original_bars = core::array::from_fn(|index| {
            root.read_config_word(bdf, PCI_BAR0 + index as u8 * PCI_REGISTER_BYTES as u8)
        });
        pending.push(PendingDevice {
            device: PciDevice {
                bus: bdf.bus,
                device: bdf.device,
                function: bdf.function,
                vendor_id: identity.vendor_id,
                device_id: identity.device_id,
                header_type,
                class_code: (u32::from(identity.class) << 16)
                    | (u32::from(identity.subclass) << 8)
                    | u32::from(identity.prog_if),
                bars: [None; PCI_BAR_COUNT],
                capabilities,
                capability_count,
                interrupt,
            },
            command: root.get_status_command(bdf).1,
            original_bars,
            bar_info: core::array::from_fn(|_| None),
        });
    }

    if info.pci.kind == PciConfiguration::Loongson {
        for selected in info
            .board
            .ahci
            .and_then(|ahci| ahci.pci)
            .into_iter()
            .chain(info.board.gmac.and_then(|gmac| gmac.pci))
        {
            if !pending.iter().any(|entry| {
                entry.device.bus == selected.bus
                    && entry.device.device == selected.device
                    && entry.device.function == selected.function
            }) {
                return Err(ProbeError::Resource(
                    "selected native PCI function is absent",
                ));
            }
        }
    }
    // 枚举迭代器天然按 BDF 升序前进，保留此顺序以保证设备发现编号的确定性。
    Ok(pending)
}

/// 在龙芯平台上判断根总线功能 `bdf` 是否属于启动信息选定的板载 AHCI 或 GMAC 外设，并在匹配时返回其中断源编号。
fn selected_native(
    root: &PciRoot<HostCam>,
    info: &BootInfo,
    bdf: DeviceFunction,
    identity: &DeviceFunctionInfo,
) -> Result<Option<u32>, ProbeError> {
    let class_code = (u32::from(identity.class) << 16)
        | (u32::from(identity.subclass) << 8)
        | u32::from(identity.prog_if);
    let ahci = info.board.ahci.map(|resources| {
        (
            resources.pci,
            resources.registers,
            resources.interrupt_source,
            ahci_bar(identity.vendor_id, identity.device_id, class_code),
        )
    });
    let gmac = info.board.gmac.map(|resources| {
        (
            resources.pci,
            resources.registers,
            resources.interrupt_source,
            gmac_bar(identity.vendor_id, identity.device_id, class_code),
        )
    });
    let mut source = None;
    for (selected, registers, interrupt, bar) in IntoIterator::into_iter([ahci, gmac]).flatten() {
        if let Some(selected) = selected {
            if bdf.bus != selected.bus
                || bdf.device != selected.device
                || bdf.function != selected.function
            {
                continue;
            }
            bar?;
            // 龙芯集成 AHCI/GMAC 配置空间的 Interrupt Pin 寄存器可能未按多功能规范报告，此处按 `1 + (function % 4)` 计算实际路由引脚。
            let pin = if identity.vendor_id == PCI_VENDOR_LOONGSON
                && matches!(
                    identity.device_id,
                    PCI_DEVICE_LOONGSON_AHCI | PCI_DEVICE_LOONGSON_GMAC
                ) {
                1 + (bdf.function & (PCI_INTX_PIN_COUNT - 1))
            } else {
                (root.read_config_word(bdf, PCI_INTERRUPT_WORD) >> u8::BITS) as u8
            };
            let routed = info
                .pci
                .interrupt_source(bdf.bus, bdf.device, bdf.function, pin)
                .map_err(ProbeError::Resource)?;
            if routed != interrupt || selected.interrupt_source != interrupt {
                return Err(ProbeError::Resource(
                    "selected native PCI interrupt route differs",
                ));
            }
        } else {
            let Ok(index) = bar else { continue };
            let low = root.read_config_word(bdf, PCI_BAR0 + index as u8 * PCI_REGISTER_BYTES as u8);
            if low & PCI_BAR_IO_SPACE != 0
                || !matches!(
                    low & PCI_BAR_MEMORY_TYPE_MASK,
                    PCI_BAR_MEMORY_TYPE_32 | PCI_BAR_MEMORY_TYPE_64
                )
            {
                continue;
            }
            let high = if low & PCI_BAR_MEMORY_TYPE_MASK == PCI_BAR_MEMORY_TYPE_64 {
                root.read_config_word(bdf, PCI_BAR1 + index as u8 * PCI_REGISTER_BYTES as u8)
            } else {
                0
            };
            let address =
                u64::from(low & PCI_BAR_MEMORY_ADDRESS_MASK) | (u64::from(high) << u32::BITS);
            if address == 0 {
                continue;
            }
            let physical = translate(&info.pci, MEMORY_DOMAIN, address, 1)?;
            if physical.start != registers.start {
                continue;
            }
        }
        if source.replace(interrupt).is_some() {
            return Err(ProbeError::Resource(
                "native PCI function has multiple selected resources",
            ));
        }
    }
    Ok(source)
}

/// 对所有候选功能统一执行 BAR 尺寸探测、总线地址分配与原生外设窗口核对，全部验证通过后才将 BAR 地址写回硬件并开启地址解码与总线主控（DMA）。
fn configure_resources(
    root: &mut PciRoot<HostCam>,
    pending: &mut [PendingDevice],
    occupied: &mut Vec<Occupied>,
    info: &BootInfo,
) -> Result<(), ProbeError> {
    let host = &info.pci;
    let decode = Command::IO_SPACE | Command::MEMORY_SPACE | Command::BUS_MASTER;
    // Step 1: 暂时关闭所有候选功能的 I/O、Memory 地址解码与 Bus Master DMA，并回读确认硬件已关闭解码，以便安全地写全 1 探测 BAR 尺寸。
    for entry in pending.iter() {
        root.set_command(entry.device.bdf(), entry.command & !decode);
        if root
            .get_status_command(entry.device.bdf())
            .1
            .intersects(decode)
        {
            return Err(ProbeError::Resource(
                "PCI device did not disable address decoding",
            ));
        }
    }

    // Step 2: 探测每个候选功能的 6 个 BAR 槽位类型与所需空间大小。
    for entry in pending.iter_mut() {
        entry.bar_info = root.bars(entry.device.bdf())?;
    }

    // Step 3: 扫描固件已预设的非零 BAR 地址，若合法且落在主桥窗口内则率先登记到地址占用表 `occupied` 中，避免后续新分配与之冲突。
    for entry in pending.iter() {
        for bar in entry.bar_info.iter().flatten() {
            let (domain, address, size) = bar_address(bar);
            if address != 0 {
                validate_bar_address(bar, address)?;
                if matches!(bar, BarInfo::IO { .. }) {
                    translate(host, domain, address, size)?;
                    reserve(occupied, domain, address, size)?;
                } else if translate(host, domain, address, size).is_ok() {
                    reserve(occupied, domain, address, size)?;
                }
                // 若内存 BAR 的固件预设地址不在主桥允许窗口内，此处暂不登记，留待 Step 4 清零并重新分配。
            }
        }
    }

    // Step 4: 为每个内存 BAR 确定最终总线地址（保留合法的固件地址，或清零寄存器后从主桥 MEM32 窗口分配新区间）并换算 CPU 物理地址。
    for entry in pending.iter_mut() {
        for (index, bar) in entry.bar_info.iter().enumerate() {
            let Some(bar) = bar else { continue };
            let BarInfo::Memory {
                address_type,
                prefetchable,
                address,
                size,
            } = bar
            else {
                continue;
            };
            let address_type = *address_type;
            let size = *size;
            let existing = *address;
            let address = if existing != 0 && translate(host, MEMORY_DOMAIN, existing, size).is_ok()
            {
                existing
            } else {
                // 当固件未分配地址（0）或原地址超出主桥窗口时，先将硬件 BAR 清零，再在 MEM32 窗口内分配对齐的空闲区间。
                match address_type {
                    MemoryBarType::Width64 => root.set_bar_64(entry.device.bdf(), index as u8, 0),
                    _ => root.set_bar_32(entry.device.bdf(), index as u8, 0),
                }
                allocate(host, occupied, address_type, size)?
            };
            let physical = translate(host, MEMORY_DOMAIN, address, size)?;
            mmio_region_address(physical)?;
            entry.device.bars[index] = Some(PciBar {
                bus_address: address,
                physical,
                prefetchable: *prefetchable,
            });
        }
    }

    // Step 5: 在真正向硬件写入新 BAR 地址前，校验板级 AHCI 与 GMAC 原生外设的 BAR 映射及中断号是否完全匹配。
    if let Some(resources) = info.board.ahci {
        resolve_registers(
            pending.iter().map(|entry| &entry.device),
            resources.pci,
            resources.registers,
            resources.interrupt_source,
            resources.dma_coherent,
            ahci_bar,
        )?;
    }
    if let Some(resources) = info.board.gmac {
        resolve_registers(
            pending.iter().map(|entry| &entry.device),
            resources.pci,
            resources.registers,
            resources.interrupt_source,
            resources.dma_coherent,
            gmac_bar,
        )?;
    }

    // Step 6: 所有地址与占用校验均已通过，将最终 BAR 总线地址写入硬件配置空间并回读核对，随后开启 MEMORY_SPACE 与 BUS_MASTER。
    for entry in pending.iter() {
        for (index, selected) in entry.device.bars.iter().enumerate() {
            let Some(selected) = selected else { continue };
            let bdf = entry.device.bdf();
            let wide = matches!(
                entry.bar_info[index],
                Some(BarInfo::Memory {
                    address_type: MemoryBarType::Width64,
                    ..
                })
            );
            if wide {
                root.set_bar_64(bdf, index as u8, selected.bus_address);
            } else {
                root.set_bar_32(bdf, index as u8, selected.bus_address as u32);
            }
            let low = root.read_config_word(bdf, PCI_BAR0 + index as u8 * PCI_REGISTER_BYTES as u8)
                & PCI_BAR_MEMORY_ADDRESS_MASK;
            let high = if wide {
                root.read_config_word(bdf, PCI_BAR1 + index as u8 * PCI_REGISTER_BYTES as u8)
            } else {
                0
            };
            if (u64::from(low) | (u64::from(high) << u32::BITS)) != selected.bus_address {
                return Err(ProbeError::Resource(
                    "PCI BAR configuration readback differs",
                ));
            }
        }
    }

    for entry in pending.iter() {
        let mut command = entry.command & !decode;
        if entry.device.bars.iter().any(Option::is_some) {
            command |= Command::MEMORY_SPACE | Command::BUS_MASTER;
        }
        root.set_command(entry.device.bdf(), command);
        if root.get_status_command(entry.device.bdf()).1 & decode != command & decode {
            return Err(ProbeError::Resource(
                "PCI command configuration readback differs",
            ));
        }
    }
    Ok(())
}

/// 当 BAR 分配或校验失败时，先关闭所有候选功能的地址解码，再将全部 BAR 与 `Command` 寄存器恢复为枚举前的原始固件值。
fn restore_configuration(root: &mut PciRoot<HostCam>, pending: &[PendingDevice]) {
    for entry in pending {
        root.set_command(
            entry.device.bdf(),
            entry.command & !(Command::IO_SPACE | Command::MEMORY_SPACE | Command::BUS_MASTER),
        );
    }

    for entry in pending {
        let mut index = 0;
        while index < PCI_BAR_COUNT {
            let original = entry.original_bars[index];
            if original & (PCI_BAR_IO_SPACE | PCI_BAR_MEMORY_TYPE_MASK) == PCI_BAR_MEMORY_TYPE_64
                && index < PCI_BAR_COUNT - 1
            {
                let address =
                    u64::from(original) | (u64::from(entry.original_bars[index + 1]) << u32::BITS);
                root.set_bar_64(entry.device.bdf(), index as u8, address);
                index += 2;
            } else {
                root.set_bar_32(entry.device.bdf(), index as u8, original);
                index += 1;
            }
        }
    }

    for entry in pending {
        root.set_command(entry.device.bdf(), entry.command);
    }
}

/// 提取 `BarInfo` 所属的地址空间域（[`IO_DOMAIN`] 或 [`MEMORY_DOMAIN`]）、当前总线地址及字节大小。
fn bar_address(bar: &BarInfo) -> (u8, u64, u64) {
    match bar {
        BarInfo::IO { address, size } => (IO_DOMAIN, u64::from(*address), u64::from(*size)),
        BarInfo::Memory { address, size, .. } => (MEMORY_DOMAIN, *address, *size),
    }
}

/// 校验给定 BAR 的尺寸是否为 2 的幂、总线地址是否按尺寸自然对齐，以及区间末尾是否未超出该 BAR 位宽类型的可寻址上限。
fn validate_bar_address(bar: &BarInfo, address: u64) -> Result<(), ProbeError> {
    let (_, _, size) = bar_address(bar);
    let end = address
        .checked_add(size)
        .ok_or(ProbeError::Resource("PCI BAR address overflows"))?;
    let limit = match bar {
        BarInfo::Memory {
            address_type: MemoryBarType::Width64,
            ..
        } => u64::MAX,
        BarInfo::Memory {
            address_type: MemoryBarType::Below1MiB,
            ..
        } => 1 << 20,
        _ => 1 << u32::BITS,
    };
    if !size.is_power_of_two() || address & (size - 1) != 0 || limit < end {
        return Err(ProbeError::Resource(
            "PCI BAR size, alignment or address width is invalid",
        ));
    }
    Ok(())
}

/// 根据主桥地址转换窗口（`PciRange`）的标志位返回其所属的地址空间域（I/O 域或内存域）。
fn range_domain(range: &PciRange) -> u8 {
    match range.flags & PCI_SPACE_CODE_MASK {
        PCI_SPACE_IO => IO_DOMAIN,
        PCI_SPACE_MEM32 | PCI_SPACE_MEM64 => MEMORY_DOMAIN,
        _ => 0,
    }
}

/// 将 PCI 总线域 `domain` 内的区间 `[address, address + size)` 通过主桥 `ranges` 表换算为 CPU 物理地址区间 [`Region`]。
fn translate(
    host: &PciResources,
    domain: u8,
    address: u64,
    size: u64,
) -> Result<Region, ProbeError> {
    for range in &host.ranges[..host.range_count] {
        if range_domain(range) != domain {
            continue;
        }
        let start = range.bus_start as u64;
        let Some(offset) = address.checked_sub(start) else {
            continue;
        };
        let length = (range.cpu.end - range.cpu.start) as u64;
        let end = offset
            .checked_add(size)
            .ok_or(ProbeError::Resource("PCI range offset overflows"))?;
        if length < end {
            continue;
        }
        let offset = usize::try_from(offset)
            .map_err(|_| ProbeError::Resource("PCI offset exceeds CPU address width"))?;
        let size = usize::try_from(size)
            .map_err(|_| ProbeError::Resource("PCI size exceeds CPU address width"))?;
        let physical = range
            .cpu
            .start
            .checked_add(offset)
            .ok_or(ProbeError::Resource("PCI CPU address overflows"))?;
        return Region::from_size(physical, size).map_err(ProbeError::Resource);
    }
    Err(ProbeError::Resource(
        "PCI BAR is outside allowed host ranges",
    ))
}

/// 在已占用地址表 `occupied` 中登记区间 `[start, start + size)`，若与同一地址域内的既有区间重叠则返回 [`ProbeError::Resource`]。
fn reserve(
    occupied: &mut Vec<Occupied>,
    domain: u8,
    start: u64,
    size: u64,
) -> Result<(), ProbeError> {
    let end = start
        .checked_add(size)
        .ok_or(ProbeError::Resource("PCI BAR occupation overflows"))?;
    if occupied
        .iter()
        .any(|old| old.domain == domain && start < old.end && old.start < end)
    {
        return Err(ProbeError::Resource("PCI BAR occupations overlap"));
    }
    occupied.push(Occupied { domain, start, end });
    Ok(())
}

/// 在主桥 `MEM32` 地址窗口中按 `size` 自然对齐要求搜寻第一块未占用的空闲总线地址区间，并在登记占用后返回其起始总线地址。
fn allocate(
    host: &PciResources,
    occupied: &mut Vec<Occupied>,
    width: MemoryBarType,
    size: u64,
) -> Result<u64, ProbeError> {
    if !size.is_power_of_two() {
        return Err(ProbeError::Resource("PCI allocation size is invalid"));
    }
    let width_limit = match width {
        MemoryBarType::Width64 => u64::MAX,
        MemoryBarType::Width32 => 1 << u32::BITS,
        MemoryBarType::Below1MiB => 1 << 20,
    };
    for range in &host.ranges[..host.range_count] {
        if range.flags & PCI_SPACE_CODE_MASK != PCI_SPACE_MEM32 {
            continue;
        }
        let limit = (range.bus_start as u64)
            .checked_add((range.cpu.end - range.cpu.start) as u64)
            .ok_or(ProbeError::Resource("PCI bus range overflows"))?
            .min(width_limit);
        let mut candidate = (range.bus_start as u64).max(size);
        loop {
            candidate = candidate
                .checked_add(size - 1)
                .ok_or(ProbeError::Resource("PCI BAR alignment overflows"))?
                & !(size - 1);
            let end = candidate
                .checked_add(size)
                .ok_or(ProbeError::Resource("PCI allocation overflows"))?;
            if limit < end {
                break;
            }
            let collision = occupied
                .iter()
                .filter(|old| old.domain == MEMORY_DOMAIN && candidate < old.end && old.start < end)
                .map(|old| old.end)
                .max();
            if let Some(next) = collision {
                candidate = next;
                continue;
            }
            reserve(occupied, MEMORY_DOMAIN, candidate, size)?;
            return Ok(candidate);
        }
    }
    Err(ProbeError::Resource(
        "PCI memory windows have no free aligned BAR extent",
    ))
}

#[cfg(feature = "ktest")]
#[::ktest::suite(context = "Boot", tags("boot", "pci"), timeout_ms = 1000)]
mod configuration_checks {
    use super::*;

    /// 测试目标：验证龙芯 CFG0（标准 256B）与 CFG1（扩展 4096B）配置空间偏移计算公式，以及根总线 Root Port 非零隐藏功能的拦截逻辑。
    /// 场景步骤：分别计算 Bus 0 SATA 设备、Bus 2 下游设备在标准/扩展偏移处的地址，并查询 Bus 0 Device 9 Function 1 隐藏功能。
    /// 预期结果：标准与扩展偏移均精确匹配龙芯硬件位域公式（含 Type 1 标志位），越界偏移或隐藏功能返回 `None`。
    #[test]
    fn loongson_configuration_offsets() {
        let sata = DeviceFunction {
            bus: 0,
            device: 8,
            function: 0,
        };
        assert_eq!(loongson_offset(sata, 0x10, false), Some(0x4010));
        assert_eq!(loongson_offset(sata, 0x310, true), Some(0x0300_4010));
        assert_eq!(loongson_offset(sata, 0x310, false), None);

        let secondary = DeviceFunction {
            bus: 2,
            device: 0,
            function: 0,
        };
        assert_eq!(loongson_offset(secondary, 0x10, false), Some(0x0102_0010));
        assert_eq!(loongson_offset(secondary, 0x310, true), Some(0x1302_0010));

        let hidden = DeviceFunction {
            bus: 0,
            device: 9,
            function: 1,
        };
        assert_eq!(loongson_offset(hidden, 0, false), None);
    }

    /// 测试目标：验证原生 AHCI 与 GMAC 控制器对应 BAR 槽位索引的选择规则。
    /// 场景步骤：分别传入龙芯集成 AHCI、标准 PCI 类别 AHCI、龙芯两代单通道 GMAC 以及不匹配的厂商/设备 ID。
    /// 预期结果：龙芯 AHCI/GMAC 返回 BAR0，标准 AHCI 返回 BAR5，其余不支持的设备组合均返回 `Err`。
    #[test]
    fn native_device_bar_rules() {
        assert_eq!(ahci_bar(0x0014, 0x7a08, 0).unwrap(), 0);
        assert_eq!(ahci_bar(0x8086, 0x2922, 0x010601).unwrap(), 5);
        assert!(ahci_bar(0x0014, 0x7a03, 0x020000).is_err());
        assert!(ahci_bar(0x8086, 0x2922, 0x010600).is_err());

        assert_eq!(gmac_bar(0x0014, 0x7a03, 0x020000).unwrap(), 0);
        assert_eq!(gmac_bar(0x0014, 0x7a23, 0x020000).unwrap(), 0);
        assert!(gmac_bar(0x0014, 0x7a13, 0x020000).is_err());
        assert!(gmac_bar(0x1234, 0x7a03, 0x020000).is_err());
    }

    /// 测试目标：验证原生外设直接描述的寄存器区间与 PCI BAR 物理区间的对齐与包含关系校验（`normalize_window`）。
    /// 场景步骤：传入缺省区间、完整匹配的 BAR 区间、同起点的子区间，以及起点偏移、超出 BAR 长度或空的异常区间。
    /// 预期结果：合法配置返回完整的 PCI BAR 物理区间，起点不一致、越界或非 PCI 空区间均返回 `Err`。
    #[test]
    fn native_complete_register_windows() {
        let bar = Region::from_size(0x400e_0000, 0x10000).unwrap();
        assert_eq!(normalize_window(bar, Region::default(), true).unwrap(), bar);
        assert_eq!(normalize_window(bar, bar, true).unwrap(), bar);
        let direct = Region::from_size(bar.start, 0x1100).unwrap();
        assert_eq!(normalize_window(bar, direct, false).unwrap(), bar);
        assert!(normalize_window(bar, Region::default(), false).is_err());
        let configuration_identity = Region::from_size(0x4000, 0x1100).unwrap();
        assert!(normalize_window(bar, configuration_identity, true).is_err());
        let offset = Region::from_size(bar.start + 0x100, 0x1000).unwrap();
        assert!(normalize_window(bar, offset, false).is_err());
        let oversized = Region::from_size(bar.start, 0x20000).unwrap();
        assert!(normalize_window(bar, oversized, false).is_err());
        assert!(normalize_window(Region::default(), direct, false).is_err());
    }
}
