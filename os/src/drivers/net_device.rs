//! 网络根只保存真实设备叶，共同协议、帧工作区和停止责任仍由同一 NetStack 拥有。

use alloc::sync::Arc;

use crate::boot::{self, Platform};
use crate::drivers::dwmac::{Dwmac, GmacError};
use crate::drivers::virtio::{
    VirtioInitError, VIRTIO_PCI_NET_LEGACY, VIRTIO_PCI_NET_MODERN, VIRTIO_VENDOR_ID,
};
use crate::drivers::virtio_net::VirtioNet;
use crate::drivers::{DeviceInventory, ProbeError};
use crate::fs::FsError;
use crate::net::stats::InterfaceStats;
use crate::net::NetStackRef;

/// 每个变体直接持有原设备 Arc，不增加另一层拥有者或设备运行时。
pub(crate) enum NetDevice {
    Virtio(Arc<VirtioNet>),
    Dwmac(Arc<Dwmac>),
}

/// 设备边界保留原叶错误，只有进入文件能力层时才转换为 FsError。
#[derive(Debug)]
pub(crate) enum NetDeviceError {
    Virtio(VirtioInitError),
    Dwmac(GmacError),
    Resource(ProbeError),
}

impl NetDevice {
    /// 只查询同次库存或启动快照中的真实候选，不探测硬件或按接口名字创建设备。
    #[cfg_attr(
        feature = "ktest",
        expect(
            dead_code,
            reason = "普通 init 按实际库存选择 NIC 或 lo，ktest 自行建立具名网络 fixture"
        )
    )]
    pub(crate) fn has_candidate(inventory: &DeviceInventory) -> bool {
        match boot::info().platform {
            Platform::QemuVirt => {
                !inventory.mmio_networks.is_empty()
                    || inventory.pci.devices().iter().any(|device| {
                        device.vendor_id == VIRTIO_VENDOR_ID
                            && matches!(
                                device.device_id,
                                VIRTIO_PCI_NET_LEGACY | VIRTIO_PCI_NET_MODERN
                            )
                    })
            }
            Platform::VisionFive2 | Platform::Ls2k1000la => boot::info().board.gmac.is_some(),
        }
    }

    /// 从平台指定的实际候选准备设备；板 PCI 资源必须先由同一库存解析完整 BAR 和 IRQ。
    ///
    /// # Safety
    /// 调用者须独占所选 MAC、PHY 和板设置，保证库存、启动资源及全部设备窗口永久有效。
    /// 返回的设备须在 start 前交给网络根强持，直到 producer、IRQ、Work 和 DMA 停稳后释放。
    pub(crate) unsafe fn prepare(
        net: &NetStackRef,
        inventory: &DeviceInventory,
    ) -> Result<Self, NetDeviceError> {
        match boot::info().platform {
            Platform::QemuVirt => {
                // SAFETY: 调用者保证同次库存、完整永久映射和唯一硬件拥有；叶仍验证唯一候选。
                unsafe { VirtioNet::prepare(net, inventory, 0) }
                    .map(Self::Virtio)
                    .map_err(NetDeviceError::Virtio)
            }
            Platform::VisionFive2 | Platform::Ls2k1000la => {
                let resources = boot::info()
                    .board
                    .gmac
                    .as_ref()
                    .ok_or(NetDeviceError::Dwmac(GmacError::Resource(
                        "board has no selected GMAC",
                    )))?;
                let resolved = inventory
                    .pci
                    .resolve_gmac(resources)
                    .map_err(NetDeviceError::Resource)?;
                // SAFETY: 同一库存已将 PCI 身份规范化为完整设备窗口；调用者承担独占、永久映射
                // 和板设置义务，返回 Arc 在发表前由原网络根保存，覆盖全部停止尾部。
                unsafe { Dwmac::prepare(net, &resolved, 0) }
                    .map(Self::Dwmac)
                    .map_err(NetDeviceError::Dwmac)
            }
        }
    }

    /// 返回叶准备时验证的实际 MAC，不为缺失的板资源生成地址。
    pub(crate) fn mac_address(&self) -> [u8; 6] {
        match self {
            Self::Virtio(nic) => nic.info().mac,
            Self::Dwmac(nic) => nic.mac_address(),
        }
    }

    /// 复制原叶的真实收发计数，不从协议请求量推算设备完成。
    pub(crate) fn statistics(&self) -> InterfaceStats {
        match self {
            Self::Virtio(nic) => nic.statistics(),
            Self::Dwmac(nic) => nic.statistics(),
        }
    }

    /// 根已保存本设备时发表真实队列和 IRQ；任何失败仍由同一根执行停止。
    pub(crate) fn start(&self) -> Result<(), NetDeviceError> {
        match self {
            Self::Virtio(nic) => nic.start().map_err(NetDeviceError::Virtio),
            Self::Dwmac(nic) => nic.start().map_err(NetDeviceError::Dwmac),
        }
    }

    /// 锁外 writer 只编码本帧，不得睡眠、分配、uaccess 或反调网络；槽满不调用 writer。
    pub(crate) fn try_transmit(
        &self,
        frame_len: usize,
        writer: impl FnOnce(&mut [u8]),
    ) -> Result<bool, NetDeviceError> {
        match self {
            Self::Virtio(nic) => nic
                .try_transmit(frame_len, writer)
                .map_err(NetDeviceError::Virtio),
            Self::Dwmac(nic) => nic
                .try_transmit(frame_len, writer)
                .map_err(NetDeviceError::Dwmac),
        }
    }

    /// 原根结束所有外部生产者后同步等待叶停止，未确认时不得释放根或原设备 Arc。
    pub(crate) fn stop_sync(&self) -> Result<(), NetDeviceError> {
        match self {
            Self::Virtio(nic) => nic.stop_sync().map_err(NetDeviceError::Virtio),
            Self::Dwmac(nic) => nic.stop_sync().map_err(NetDeviceError::Dwmac),
        }
    }

    /// 专用 fixture 只借真实 VirtIO 变体的原 Arc，原生 GMAC 不具有 VirtIO 观察身份。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_virtio(&self) -> Option<&Arc<VirtioNet>> {
        match self {
            Self::Virtio(nic) => Some(nic),
            Self::Dwmac(_) => None,
        }
    }
}

impl NetDeviceError {
    /// 文件层保留内存不足类别，并在转换前记录真实叶及原始原因，不构造另一设备的状态。
    pub(crate) fn into_fs_error(self) -> FsError {
        crate::klog!(
            crate::klog::Level::Error,
            "Network device failed: {:?}",
            self
        );
        match self {
            Self::Virtio(VirtioInitError::NoMemory { .. })
            | Self::Dwmac(GmacError::NoMemory)
            | Self::Resource(ProbeError::NoMemory) => FsError::NoMemory,
            _ => FsError::DeviceIo,
        }
    }
}
