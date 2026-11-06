//! 外设驱动子系统与只读硬件资源发现入口。
//!
//! ## 模块职责与架构定位
//! 本模块作为内核设备驱动层（`drivers`）的顶层枢纽，向上供内核初始化流程调用以盘点
//! 系统中可用的块设备、网络设备及 PCI 功能；向下组织并导出 VirtIO（MMIO/PCI）、AHCI、
//! DWMAC、SDMMC、PCI 主桥及板级时钟/引脚配置（`board_setup`）等具体驱动子模块。
//!
//! ## 核心设计与发现流程
//! 资源发现（[`discover`]）在普通可调度初始化任务中单次执行，仅对硬件身份寄存器执行只读探测，
//! 不分配或初始化 Virtqueue，也不置位 `DRIVER_OK`，将实际的设备激活留给后续具体驱动：
//!
//! ```text
//! BootInfo (固件/FDT 发现的原始区间与中断源)
//!    │
//!    ├─► VirtIO-MMIO 扫描 (读取前 16 字节只读身份头)
//!    │     ├─ Offset 0x00: MagicValue (固定为 0x7472_6976 "virt")
//!    │     ├─ Offset 0x04: Version    (1 = Legacy, 2 = Modern)
//!    │     ├─ Offset 0x08: DeviceID   (1 = Network, 2 = Block)
//!    │     └─ Offset 0x0c: VendorID   (设备厂商标识)
//!    │     │
//!    │     ├─► DeviceID == 2 ──► 按物理基址升序收集到 mmio_blocks
//!    │     └─► DeviceID == 1 ──► 按物理基址升序收集到 mmio_networks
//!    │
//!    └─► PciInventory::discover(info) ──► 枚举并配置 PCI Root Bus 设备与 BAR
//! ```
//!
//! ## 锁顺序与并发契约
//! - [`discover`] 必须在普通任务上下文（`current_task` 存在）中调用，在无锁状态下预分配动态数组容量，
//!   避免在中断处理路径中触发堆内存扩容。
//! - 本模块自身不持有锁；内部调用的 [`irq::map_source`] 与 [`PciInventory::discover`] 各自维护其同步契约。

use alloc::vec::Vec;
use core::ptr;
use virtio_drivers::transport::pci::bus::PciError;
use virtio_drivers::transport::{mmio::MmioVersion, DeviceType};

const VIRTIO_MMIO_MAGIC: u32 = 0x7472_6976;
const VIRTIO_MMIO_VERSION: usize = 0x04;
const VIRTIO_MMIO_DEVICE_ID: usize = 0x08;
const VIRTIO_MMIO_VENDOR_ID: usize = 0x0c;
const VIRTIO_MMIO_IDENTITY_SIZE: usize = VIRTIO_MMIO_VENDOR_ID + core::mem::size_of::<u32>();

use crate::{
    arch,
    boot::{self, Region},
    irq::{self, IrqError, IrqNumber},
    task,
};

pub(crate) mod ahci;
pub(crate) mod board_setup;
pub(crate) mod dwmac;
pub(crate) mod net_device;
pub mod pci;
pub(crate) mod sdmmc;
pub mod virtio;
pub mod virtio_net;

use crate::drivers::pci::PciInventory;

/// 设备资源发现与早期探测阶段的统一错误类型。
#[derive(Debug)]
pub enum ProbeError {
    /// 内核堆内存不足，无法预留设备候选列表或资源表的容量。
    NoMemory,
    /// 全局单次发现流程（如 PCI 主桥枚举）已被执行过，拒绝重复探测。
    AlreadyDiscovered,
    /// 固件描述的寄存器窗口、设备身份头或总线资源不符合硬件规范。
    Resource(
        #[expect(
            dead_code,
            reason = "资源发现失败通过 Debug 日志保留原始固件错误，errno 映射只选择错误类别"
        )]
        &'static str,
    ),
    /// 底层 PCI 配置空间或 BAR 探测过程中发生的传输层错误。
    Pci(PciError),
    /// 设备中断源向内核全局中断控制器域映射时发生的错误。
    Irq(IrqError),
}

impl From<PciError> for ProbeError {
    /// 将 `virtio_drivers` 的 PCI 总线错误转换为统一的 [`ProbeError::Pci`]。
    fn from(error: PciError) -> Self {
        Self::Pci(error)
    }
}

impl From<IrqError> for ProbeError {
    /// 将中断子系统的映射错误转换为统一的 [`ProbeError::Irq`]。
    fn from(error: IrqError) -> Self {
        Self::Irq(error)
    }
}

/// 已通过身份校验的 VirtIO-MMIO 块设备（`DeviceID = 2`）候选描述。
///
/// 仅记录硬件位置、协议版本与中断号，尚未映射 Virtqueue 或修改设备状态寄存器。
#[derive(Debug)]
pub struct MmioBlockCandidate {
    /// 控制器 MMIO 寄存器的 CPU 物理地址区间（字节单位）。
    pub registers: Region,
    /// VirtIO-MMIO 协议版本号（`1` 代表 Legacy，`2` 代表 Modern）。
    pub version: u32,
    /// 硬件寄存器报告的 VirtIO 厂商标识（Vendor ID）。
    pub vendor_id: u32,
    /// 已映射到内核外部中断控制器域的设备中断号。
    pub interrupt: IrqNumber,
}

/// 已通过身份校验的 VirtIO-MMIO 网络设备（`DeviceID = 1`）候选描述。
///
/// 独立于块设备列表保存，供后续网络驱动初始化阶段按需绑定。
#[derive(Debug)]
pub struct MmioNetCandidate {
    /// 网卡 MMIO 寄存器的 CPU 物理地址区间（字节单位）。
    pub registers: Region,
    #[allow(
        dead_code,
        reason = "RISC-V MMIO NIC 准备核对发现时的版本，LoongArch 网络使用 PCI 库存"
    )]
    /// VirtIO-MMIO 协议版本号（`1` 代表 Legacy，`2` 代表 Modern）。
    pub version: u32,
    #[allow(
        dead_code,
        reason = "RISC-V MMIO NIC 准备核对发现时的 vendor 身份，LoongArch 网络使用 PCI 库存"
    )]
    /// 硬件寄存器报告的 VirtIO 厂商标识（Vendor ID）。
    pub vendor_id: u32,
    #[allow(
        dead_code,
        reason = "RISC-V MMIO NIC 以发现的 IRQ source 注册，LoongArch 网络使用 PCI INTx"
    )]
    /// 已映射到内核外部中断控制器域的网卡中断号。
    pub interrupt: IrqNumber,
}

/// 系统启动阶段盘点出的全部可用外设资源清单。
///
/// 分别保存 VirtIO-MMIO 块设备候选、VirtIO-MMIO 网络设备候选以及 PCI 主桥设备库存，
/// 仅提供硬件拓扑与资源快照，不在此阶段实例化具体设备驱动。
pub struct DeviceInventory {
    /// 按寄存器物理起始地址升序排列的 VirtIO-MMIO 块设备候选列表。
    pub mmio_blocks: Vec<MmioBlockCandidate>,
    /// 按寄存器物理起始地址升序排列的 VirtIO-MMIO 网络设备候选列表。
    pub mmio_networks: Vec<MmioNetCandidate>,
    /// PCI 根总线枚举得到的配置空间访问器与已分配 BAR 的设备列表。
    pub pci: PciInventory,
}

/// 在普通初始化任务中扫描并汇总 VirtIO-MMIO 块/网络候选设备与 PCI 总线资源。
///
/// # 调用上下文与副作用
/// - 必须在普通可调度任务上下文中调用（内部通过 `task::current_task()` 校验）。
/// - 仅对 VirtIO-MMIO 头部做只读校验并对 PCI 根总线执行单次枚举与 BAR 分配，
///   不初始化任何具体外设的传输队列或将设备置为 `DRIVER_OK`。
pub fn discover() -> Result<DeviceInventory, ProbeError> {
    // Step 1: 确认当前处于普通任务上下文，并为 MMIO 块/网络候选列表预分配精确容量。
    drop(task::current_task());
    let info = boot::info();
    let mut mmio_blocks = Vec::new();
    mmio_blocks
        .try_reserve_exact(info.virtio_mmio_count)
        .map_err(|_| ProbeError::NoMemory)?;
    let mut mmio_networks = Vec::new();
    mmio_networks
        .try_reserve_exact(info.virtio_mmio_count)
        .map_err(|_| ProbeError::NoMemory)?;

    // Step 2: 遍历启动信息中的 VirtIO-MMIO 区间，校验窗口对齐并读取只读身份寄存器。
    for resource in &info.virtio_mmio[..info.virtio_mmio_count] {
        let base = mmio_region_address(resource.registers)?;
        if resource.registers.end - resource.registers.start < VIRTIO_MMIO_IDENTITY_SIZE
            || base & 3 != 0
        {
            return Err(ProbeError::Resource("VirtIO MMIO header window is invalid"));
        }
        // SAFETY:
        // 1. `mmio_region_address` 已校验 `resource.registers` 非空且完全处于架构强序 MMIO 映射窗口内；
        // 2. 上方条件判断已保证窗口长度至少为 `VIRTIO_MMIO_IDENTITY_SIZE`（16 字节）且基地址 `base` 按 4 字节对齐；
        // 3. 偏移 `0x00`、`0x04`、`0x08`、`0x0c` 均为 VirtIO-MMIO 规范定义的 32 位只读身份寄存器，读取无硬件副作用。
        let (magic, version, device_id, vendor_id) = unsafe {
            (
                ptr::read_volatile(base as *const u32),
                ptr::read_volatile((base + VIRTIO_MMIO_VERSION) as *const u32),
                ptr::read_volatile((base + VIRTIO_MMIO_DEVICE_ID) as *const u32),
                ptr::read_volatile((base + VIRTIO_MMIO_VENDOR_ID) as *const u32),
            )
        };
        if magic != VIRTIO_MMIO_MAGIC
            || (version != MmioVersion::Legacy as u32 && version != MmioVersion::Modern as u32)
        {
            return Err(ProbeError::Resource(
                "VirtIO MMIO identity or version is invalid",
            ));
        }

        // Step 3: 根据 DeviceID 分类收集块设备与网络设备候选，并映射各自的硬件中断号。
        if device_id == DeviceType::Block as u32 {
            mmio_blocks.push(MmioBlockCandidate {
                registers: resource.registers,
                version,
                vendor_id,
                interrupt: irq::map_source(resource.interrupt_source)?,
            });
        } else if device_id == DeviceType::Network as u32 {
            mmio_networks.push(MmioNetCandidate {
                registers: resource.registers,
                version,
                vendor_id,
                interrupt: irq::map_source(resource.interrupt_source)?,
            });
        }
    }

    // Step 4: 按寄存器物理基址排序以固定设备编号顺序，随后执行 PCI 主桥枚举。
    mmio_blocks.sort_unstable_by_key(|candidate| candidate.registers.start);
    mmio_networks.sort_unstable_by_key(|candidate| candidate.registers.start);
    let pci = PciInventory::discover(info)?;

    Ok(DeviceInventory {
        mmio_blocks,
        mmio_networks,
        pci,
    })
}

/// 将设备 MMIO 的 CPU 物理地址区间转换为内核强序设备虚拟起始地址，并执行溢出与窗口边界检查。
///
/// # 参数与返回值
/// - `region`: 设备寄存器的 CPU 物理地址半开区间 `[start, end)`（字节单位）。
/// - 成功时返回 `region.start` 对应的内核 MMIO 虚拟地址；若区间为空、发生算术溢出或超出架构固定的 MMIO 映射窗口则返回 [`ProbeError::Resource`]。
pub(crate) fn mmio_region_address(region: Region) -> Result<usize, ProbeError> {
    if region.end <= region.start {
        return Err(ProbeError::Resource("MMIO region is empty"));
    }
    let base = arch::mmio_address(0);
    let start = base
        .checked_add(region.start)
        .ok_or(ProbeError::Resource("MMIO virtual address overflows"))?;
    let end = base
        .checked_add(region.end - 1)
        .ok_or(ProbeError::Resource("MMIO virtual end overflows"))?;
    if start != arch::mmio_address(region.start) || end != arch::mmio_address(region.end - 1) {
        return Err(ProbeError::Resource(
            "MMIO address exceeds architecture window",
        ));
    }
    Ok(start)
}
