//! 当前启用设备的只读资源发现入口；普通 Task 负责准备，IRQ 不扩容。

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

/// 资源发现的唯一错误类型，保留固件、PCI 和 IRQ 边界的实际失败。
#[derive(Debug)]
pub enum ProbeError {
    NoMemory,
    AlreadyDiscovered,
    Resource(
        #[expect(
            dead_code,
            reason = "资源发现失败通过 Debug 日志保留原始固件错误，errno 映射只选择错误类别"
        )]
        &'static str,
    ),
    Pci(PciError),
    Irq(IrqError),
}

impl From<PciError> for ProbeError {
    /// PCI 配置错误保持原始原因，不自动选择另一配置访问机制。
    fn from(error: PciError) -> Self {
        Self::Pci(error)
    }
}

impl From<IrqError> for ProbeError {
    /// IRQ 资源必须来自已初始化的唯一外部 domain。
    fn from(error: IrqError) -> Self {
        Self::Irq(error)
    }
}

/// 实际 DeviceID=2 的 MMIO block 候选，未操作队列或设备状态。
#[derive(Debug)]
pub struct MmioBlockCandidate {
    pub registers: Region,
    pub version: u32,
    pub vendor_id: u32,
    pub interrupt: IrqNumber,
}

/// 实际 DeviceID=1 的 MMIO network 候选，独立于磁盘编号与选择。
#[derive(Debug)]
pub struct MmioNetCandidate {
    pub registers: Region,
    #[allow(
        dead_code,
        reason = "RISC-V MMIO NIC 准备核对发现时的版本，LoongArch 网络使用 PCI 库存"
    )]
    pub version: u32,
    #[allow(
        dead_code,
        reason = "RISC-V MMIO NIC 准备核对发现时的 vendor 身份，LoongArch 网络使用 PCI 库存"
    )]
    pub vendor_id: u32,
    #[allow(
        dead_code,
        reason = "RISC-V MMIO NIC 以发现的 IRQ source 注册，LoongArch 网络使用 PCI INTx"
    )]
    pub interrupt: IrqNumber,
}

/// 分开保存 MMIO 块、网络候选与所有 PCI 功能，不预先创建设备实例。
pub struct DeviceInventory {
    pub mmio_blocks: Vec<MmioBlockCandidate>,
    pub mmio_networks: Vec<MmioNetCandidate>,
    pub pci: PciInventory,
}

/// 普通 init Task 一次读取块、网络及 PCI 启用资源，不设置 queue 或 DRIVER_OK。
pub fn discover() -> Result<DeviceInventory, ProbeError> {
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

    for resource in &info.virtio_mmio[..info.virtio_mmio_count] {
        let base = mmio_region_address(resource.registers)?;
        if resource.registers.end - resource.registers.start < VIRTIO_MMIO_IDENTITY_SIZE
            || base & 3 != 0
        {
            return Err(ProbeError::Resource("VirtIO MMIO header window is invalid"));
        }
        // SAFETY: BootInfo 仅发布已验证的启用 VirtIO-MMIO 区间，AP 前已安装 PTE
        // 或强序 DMW；四个对齐 u32 均位于窗口内且是协议只读身份字段。
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

    mmio_blocks.sort_unstable_by_key(|candidate| candidate.registers.start);
    mmio_networks.sort_unstable_by_key(|candidate| candidate.registers.start);
    let pci = PciInventory::discover(info)?;

    Ok(DeviceInventory {
        mmio_blocks,
        mmio_networks,
        pci,
    })
}

/// checked CPU PA→强序 VA 换算；可访问映射由 AP 前架构初始化负责。
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
