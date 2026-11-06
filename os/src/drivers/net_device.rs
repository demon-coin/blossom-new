//! 网络硬件驱动的统一分发层（`NetDevice`）。
//!
//! ## 模块职责与架构定位
//! 本模块位于协议栈核心（[`crate::net::NetStack`]）与具体网卡驱动（QEMU 虚拟平台的 [`VirtioNet`]、
//! 实体开发板的 [`Dwmac`]）之间，提供静态枚举分发接口。协议栈本身统一管理网络协议状态、帧编码工作区
//! 与服务生命周期；本模块仅将准备（`prepare`）、启动（`start`）、非阻塞发送（`try_transmit`）、
//! 统计读取（`statistics`）与同步停机（`stop_sync`）委托给当前平台选定的底层网卡实例。
//!
//! ## 架构关系与所有权模型
//! ```text
//!   NetStack (协议栈根对象，持有强引用 Arc)
//!       │
//!       ▼
//!   NetDevice (静态枚举分发，无额外锁或中间缓冲)
//!    ├── Virtio(Arc<VirtioNet>)  ──► QEMU virt (MMIO / PCI VirtIO-Net)
//!    └── Dwmac(Arc<Dwmac>)       ──► VisionFive2 / LS2K1000LA (DesignWare GMAC)
//!          │
//!          └── Weak<NetStack>    (中断 / Work 接收路径反向弱引用回推以太网帧，避免循环引用)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - **无附加锁**：`NetDevice` 本身不持有任何锁，并发同步完全由具体驱动内部的控制睡眠锁与队列自旋锁负责。
//! - **发送回调禁令**：传入 [`NetDevice::try_transmit`] 的帧写入闭包虽然在驱动自旋锁之外执行，
//!   但仍处于协议栈发送路径中，严禁睡眠、动态分配内存、访问用户态内存或重入协议栈，以防死锁或破坏状态。
//! - **生命周期契约**：调用 [`NetDevice::start`] 前，上层协议栈必须已将 `NetDevice` 存入稳定强引用；
//!   销毁前必须先停止所有外部发送生产者并调用 [`NetDevice::stop_sync`] 等待中断、后台 `Work` 与硬件 DMA 彻底停稳，
//!   否则提前释放最后一个 `Arc` 将触发内部防御断言或导致悬垂指针访问（UAF）。

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

/// 统一封装不同平台的底层以太网硬件驱动实例。
///
/// 每个变体直接持有具体驱动的强引用 [`Arc`]，不引入动态分发（`dyn Trait`）或额外的中间状态层。
pub(crate) enum NetDevice {
    /// QEMU `virt` 平台使用的 VirtIO 网络设备驱动实例。
    Virtio(Arc<VirtioNet>),
    /// VisionFive 2 与 LS2K1000LA 开发板使用的 DesignWare GMAC 以太网控制器驱动实例。
    Dwmac(Arc<Dwmac>),
}

/// 网络设备初始化、启动、发送与停机过程中的底层错误集合。
///
/// 在驱动与协议栈交互边界保留底层驱动的具体错误原因，仅在跨越系统调用或文件抽象层时通过
/// [`NetDeviceError::into_fs_error`] 统一映射为 [`FsError`]。
#[derive(Debug)]
pub(crate) enum NetDeviceError {
    /// VirtIO 网络设备在资源校验、DMA 内存分配、协议协商或停机复位时发生的错误。
    Virtio(VirtioInitError),
    /// DesignWare GMAC 控制器在寄存器访问、PHY 协商、DMA 描述符环准备或停机时发生的错误。
    Dwmac(GmacError),
    /// 设备枚举库存或 PCI/中断资源解析阶段发生的错误。
    Resource(ProbeError),
}

impl NetDevice {
    /// 检查当前运行平台的设备枚举库存或板级启动信息中是否存在可用的网卡候选设备。
    ///
    /// 可在任意上下文调用。本函数仅执行纯内存查询（QEMU 检查 MMIO 网络槽位与 VirtIO-Net PCI 设备，
    /// 实体开发板检查板级 GMAC 配置），不会访问硬件寄存器或改变设备状态。
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

    /// 根据当前平台与设备枚举库存初始化底层网卡驱动，分配 DMA 队列与缓冲池，但暂不向硬件发布队列。
    ///
    /// 必须在可睡眠的普通任务上下文中调用（内部会分配物理页和后台 `Work`）。对于使用 PCI 总线的开发板
    /// （如 LS2K1000LA），会先通过 `inventory.pci` 将板级配置解析为完整的 BAR 映射窗口与中断号。
    ///
    /// # Safety
    /// 1. 调用方必须保证当前平台选定的网卡控制器、PHY 及板级时钟/引脚资源由本次初始化独占，不存在并发驱动实例。
    /// 2. `inventory` 与启动资源描述的所有 MMIO / PCI BAR 寄存器窗口必须已完成永久、强序（Uncached Device）映射，
    ///    且在整个驱动运行及停机复位完成前保持有效。
    /// 3. 返回的 `NetDevice` 在调用 [`Self::start`] 之前必须由 `net` 对应的 [`crate::net::NetStack`] 持久持有，
    ///    直到所有外部发送者退出且 [`Self::stop_sync`] 成功返回后才能释放。
    pub(crate) unsafe fn prepare(
        net: &NetStackRef,
        inventory: &DeviceInventory,
    ) -> Result<Self, NetDeviceError> {
        match boot::info().platform {
            Platform::QemuVirt => {
                // SAFETY:
                // 1. 调用方契约保证了 `inventory` 来自同一次真实硬件枚举，相关寄存器窗口已完成永久强序映射且硬件独占。
                // 2. 返回的 `Arc<VirtioNet>` 会由 `NetStack` 保存并在 `stop_sync` 确认复位完成后才释放。
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
                // SAFETY:
                // 1. 上方 `resolve_gmac` 已验证并补全了 PCI BAR 物理窗口与中断资源。
                // 2. 调用方契约保证了 MAC/PHY 独占访问、寄存器永久映射有效，以及 `NetStack` 在整个运行与停机期间持有返回的 `Arc`。
                unsafe { Dwmac::prepare(net, &resolved, 0) }
                    .map(Self::Dwmac)
                    .map_err(NetDeviceError::Dwmac)
            }
        }
    }

    /// 返回网卡在准备阶段从硬件读取并校验通过的 6 字节以太网 MAC 地址。
    ///
    /// 可在任意上下文无锁调用，返回值在设备生命周期内恒定不变。
    pub(crate) fn mac_address(&self) -> [u8; 6] {
        match self {
            Self::Virtio(nic) => nic.info().mac,
            Self::Dwmac(nic) => nic.mac_address(),
        }
    }

    /// 读取底层网卡硬件队列统计的收发包数、字节数及错误/丢包计数快照。
    ///
    /// 可在普通任务上下文调用（内部会短暂获取驱动自旋锁复制统计结构体）。统计值反映驱动在 DMA 队列边界实际完成的收发事件。
    pub(crate) fn statistics(&self) -> InterfaceStats {
        match self {
            Self::Virtio(nic) => nic.statistics(),
            Self::Dwmac(nic) => nic.statistics(),
        }
    }

    /// 注册设备中断处理函数，向硬件发布收发 DMA 队列并使能网卡中断，使设备进入正式收发状态。
    ///
    /// 必须在普通可睡眠任务上下文调用，且调用前 [`crate::net::NetStack`] 必须已持有本 `NetDevice`。
    /// 若启动过程中在向硬件写入队列地址后发生错误，调用方仍须执行 [`Self::stop_sync`] 以复位硬件并安全回收 DMA 页面。
    pub(crate) fn start(&self) -> Result<(), NetDeviceError> {
        match self {
            Self::Virtio(nic) => nic.start().map_err(NetDeviceError::Virtio),
            Self::Dwmac(nic) => nic.start().map_err(NetDeviceError::Dwmac),
        }
    }

    /// 尝试认领一个空闲发送槽位，调用 `writer` 编码以太网帧并提交给硬件发送队列。
    ///
    /// 在普通任务或后台网络工作任务上下文调用。
    /// - `frame_len`：待发送以太网帧的字节长度（不含硬件特定的前导描述符头，范围通常为 14..=1514 字节）。
    /// - `writer`：接收长度恰好为 `frame_len` 的可变字节切片并填充以太网帧内容；该函数在驱动队列锁之外执行，
    ///   但严禁睡眠、分配内存、访问用户空间或回调网络栈。
    /// - 返回值：`Ok(true)` 表示已成功提交发送队列并触发硬件通知；`Ok(false)` 表示硬件发送槽位已满（此时不会调用 `writer`）；
    ///   `Err(...)` 表示参数非法或设备已处于故障/非运行状态。
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

    /// 在所有外部发送生产者均已停止后，同步关闭后台收发 `Work`、复位硬件 DMA 控制器并注销中断。
    ///
    /// 必须在普通可睡眠任务上下文调用（等待 `Work` 退出、轮询硬件复位完成及同步注销 IRQ 均可能睡眠）。
    /// 若返回 `Err`（例如硬件复位超时未确认），底层驱动会继续保留 DMA 物理页所有权与中断上下文，防止硬件仍在后台踩内存；
    /// 此时上层绝不能提前销毁 `NetStack` 或释放本设备引用。
    pub(crate) fn stop_sync(&self) -> Result<(), NetDeviceError> {
        match self {
            Self::Virtio(nic) => nic.stop_sync().map_err(NetDeviceError::Virtio),
            Self::Dwmac(nic) => nic.stop_sync().map_err(NetDeviceError::Dwmac),
        }
    }

    /// 在内核自测（`ktest`）模式下获取底层 [`VirtioNet`] 实例引用，供测试用例检查队列与中断内部状态；
    /// 若当前网卡为原生 `Dwmac` 则返回 `None`。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_virtio(&self) -> Option<&Arc<VirtioNet>> {
        match self {
            Self::Virtio(nic) => Some(nic),
            Self::Dwmac(_) => None,
        }
    }
}

impl NetDeviceError {
    /// 记录错误日志并将底层网卡驱动错误转换为文件系统/套接字层通用的 [`FsError`]。
    ///
    /// 内存分配不足统一映射为 [`FsError::NoMemory`]，其余硬件探测、协议或复位错误映射为 [`FsError::DeviceIo`]。
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
