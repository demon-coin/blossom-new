//! 共用的选定设备传输与具体块设备的 ring/PDU 页和停止责任。

#[cfg(feature = "ktest")]
pub(crate) mod ktest_prepare;

use crate::{
    boot::{Platform, Region},
    drivers::{
        mmio_region_address,
        pci::{PciDevice, PciInventory},
        MmioBlockCandidate, ProbeError,
    },
    irq::IrqNumber,
    mm::dma::{CoherentDma, DmaDomain, DmaError, KernelHal},
    task,
};
use core::convert::TryFrom;
use core::{mem::ManuallyDrop, ptr::NonNull};
pub(crate) use virtio_drivers::transport::pci::VIRTIO_VENDOR_ID;
use virtio_drivers::{
    device::blk::{BlkInfo, VirtIOBlk, PDU_STRIDE},
    queue::{RingLayout, VirtQueue},
    transport::{
        mmio::{MmioError, MmioTransport, VirtIOHeader},
        pci::{
            bus::{DeviceFunction, PCI_CAP_ID_VNDR},
            PciRegion, PciTransport, SelectedConstruct, VirtioPciError, VIRTIO_PCI_CAP_COMMON_CFG,
            VIRTIO_PCI_CAP_DEVICE_CFG, VIRTIO_PCI_CAP_NOTIFY_CFG,
        },
        DeviceStatus, DeviceType, InterruptStatus, SomeTransport, Transport,
    },
    DmaRegion, Error, Hal,
};

pub(crate) const VIRTIO_PCI_BLOCK_LEGACY: u16 = 0x1001;
pub(crate) const VIRTIO_PCI_BLOCK_MODERN: u16 =
    VIRTIO_PCI_MODERN_DEVICE_BASE + DeviceType::Block as u16;
pub(crate) const VIRTIO_PCI_NET_LEGACY: u16 = 0x1000;
pub(crate) const VIRTIO_PCI_NET_MODERN: u16 =
    VIRTIO_PCI_MODERN_DEVICE_BASE + DeviceType::Network as u16;
// 内核所选 split-ring 深度和块控制布局，须与所绑定后端的实际协议保持一致。
pub(crate) const VIRTIO_MIN_QUEUE_DEPTH: u16 = 4;
pub(crate) const VIRTIO_MAX_QUEUE_DEPTH: u16 = 128;
pub(crate) const VIRTIO_CONTROL_SEGMENTS: u16 = 2;
pub(crate) const VIRTIO_STATUS_BYTES: usize = core::mem::size_of::<u8>();
#[cfg(feature = "ktest")]
pub(crate) const SPLIT_RING_DESCRIPTOR_BYTES: usize = 16;
const VIRTIO_PCI_MODERN_DEVICE_BASE: u16 = 0x1040;
const PCI_CONFIG_SPACE_BYTES: usize = 256;
const PCI_IDENTITY: u8 = 0;
const VIRTIO_PCI_CAP_BAR: u8 = 4;
const VIRTIO_PCI_CAP_OFFSET: u8 = 8;
const VIRTIO_PCI_CAP_LENGTH: u8 = 12;
const VIRTIO_PCI_NOTIFY_CAP_MULT: u8 = 16;
const VIRTIO_PCI_CAP_BYTES: u8 = 16;
const VIRTIO_PCI_NOTIFY_CAP_BYTES: u8 = VIRTIO_PCI_CAP_BYTES + core::mem::size_of::<u32>() as u8;
const VIRTIO_PCI_CAP_REGION_COUNT: usize = VIRTIO_PCI_CAP_DEVICE_CFG as usize;

/// 保留真实资源、传输与 DMA 准备失败，不把资源不足当无设备。
#[derive(Debug)]
pub enum VirtioInitError {
    NoMemory {
        #[cfg_attr(
            not(feature = "ktest"),
            expect(
                dead_code,
                reason = "准备失败日志保留设备状态；ktest 另核 OOM 时的真实协商状态"
            )
        )]
        status: DeviceStatus,
    },
    Resource(
        #[expect(
            dead_code,
            reason = "STORAGE INIT 的 Debug 日志保留原库存或映射失败原因"
        )]
        ProbeError,
    ),
    Mmio(
        #[expect(
            dead_code,
            reason = "STORAGE INIT 的 Debug 日志保留 MMIO 身份和版本错误"
        )]
        MmioError,
    ),
    Pci(
        #[expect(
            dead_code,
            reason = "STORAGE INIT 的 Debug 日志保留所选 PCI capability 错误"
        )]
        VirtioPciError,
    ),
    Dma {
        error: DmaError,
        #[cfg_attr(
            not(feature = "ktest"),
            expect(
                dead_code,
                reason = "DMA 准备失败日志保留设备状态；ktest 另核真实 OOM 状态"
            )
        )]
        status: DeviceStatus,
    },
    Protocol(Error),
    StopUnconfirmed {
        #[expect(
            dead_code,
            reason = "停止失败的 Debug 日志须保留超时瞬间设备状态，不能以复位后状态替代"
        )]
        status: DeviceStatus,
    },
}

impl From<ProbeError> for VirtioInitError {
    /// 保存库存与映射边界的实际失败。
    fn from(error: ProbeError) -> Self {
        Self::Resource(error)
    }
}
impl From<MmioError> for VirtioInitError {
    /// 保留 MMIO 身份和版本错误。
    fn from(error: MmioError) -> Self {
        Self::Mmio(error)
    }
}
impl From<VirtioPciError> for VirtioInitError {
    /// 保留选定 PCI capability 的实际错误。
    fn from(error: VirtioPciError) -> Self {
        Self::Pci(error)
    }
}
impl VirtioInitError {
    /// 页分配失败在 transport 仍存活时保留真实状态，不以重新 reset 后的值代替。
    pub(crate) fn dma_prepare(error: DmaError, status: DeviceStatus) -> Self {
        if error == DmaError::NoMemory {
            Self::NoMemory { status }
        } else {
            Self::Dma { error, status }
        }
    }
    /// shadow 分配失败同样保留未发表时的设备状态。
    pub(crate) fn queue_prepare(error: Error, status: DeviceStatus) -> Self {
        if error == Error::NoMemory {
            Self::NoMemory { status }
        } else {
            Self::Protocol(error)
        }
    }
}
impl From<Error> for VirtioInitError {
    /// 无设备状态上下文的协议错误保持原值，不伪造状态快照。
    fn from(error: Error) -> Self {
        Self::Protocol(error)
    }
}

/// 完整硬件身份；MMIO 物理窗口与 PCI BDF 不混用地址域。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VirtioIdentity {
    Mmio {
        physical: usize,
        version: u32,
        vendor_id: u32,
    },
    Pci {
        bus: u8,
        device: u8,
        function: u8,
        vendor_id: u16,
        device_id: u16,
    },
}

/// 仅为已经确认 coherent 恒等 DMA 的 QEMU virt 选择设备域；实体板组合须另行实现。
pub(crate) fn qemu_dma_domain() -> Result<DmaDomain, VirtioInitError> {
    if crate::platform::current() != Platform::QemuVirt {
        return Err(ProbeError::Resource("VirtIO DMA is supported only on QEMU virt").into());
    }
    Ok(DmaDomain::qemu_coherent())
}

/// 运行期协议错误保留原 DMA 诊断，再转换为库能表达的对应失败类别。
pub(crate) fn dma_protocol(error: DmaError) -> Error {
    crate::klog!(crate::klog::Level::Error, "VirtIO DMA failed: {:?}", error);
    match error {
        DmaError::NoMemory => Error::NoMemory,
        DmaError::InvalidRange
        | DmaError::AddressLimit
        | DmaError::InvalidDomain
        | DmaError::Alignment => Error::InvalidParam,
        DmaError::CacheMaintenance(_) => Error::IoError,
    }
}

/// 在普通 Task 中构造同一次库存的 MMIO 传输，不协商特性或发表队列。
///
/// 调用者须独占该真实硬件，完整 registers 窗口须永久强序映射，version、vendor_id
/// 与 interrupt 须来自该设备的真实库存；设备和映射必须覆盖所有传输、通知及 IRQ
/// 消费者的访问期，直到它们全部退出且设备已确认 reset。构造失败不选择其它设备。
pub(crate) unsafe fn prepare_mmio_transport(
    registers: Region,
    version: u32,
    vendor_id: u32,
    interrupt: IrqNumber,
) -> Result<(SomeTransport<'static>, VirtioIdentity, IrqNumber), VirtioInitError> {
    drop(task::current_task());
    let base = mmio_region_address(registers)?;
    let size = registers
        .end
        .checked_sub(registers.start)
        .ok_or(ProbeError::Resource("invalid MMIO extent"))?;
    let header = NonNull::new(base as *mut VirtIOHeader)
        .ok_or(ProbeError::Resource("null MMIO register address"))?;
    // SAFETY: 调用者承担真实硬件独占和完整永久强序映射；checked 地址与长度来自
    // 同一窗口，库构造进一步核对对齐、窗口范围与身份，所有消费者寿命仍由调用者保证。
    let transport = unsafe { MmioTransport::new(header, size) }?;
    if u32::from(transport.version()) != version || transport.vendor_id() != vendor_id {
        return Err(ProbeError::Resource("MMIO inventory identity changed").into());
    }
    Ok((
        transport.into(),
        VirtioIdentity::Mmio {
            physical: registers.start,
            version,
            vendor_id,
        },
        interrupt,
    ))
}

/// 在普通 Task 中构造同一 PCI 库存的选定传输，不重扫 capability、BAR 或 INTx。
///
/// 调用者须独占此真实功能，device 须来自 inventory，库存 BAR 的完整 CPU 物理
/// 窗口须永久强序映射；设备和映射必须覆盖所有传输、通知及 IRQ 消费者的访问期，
/// 直到它们全部退出且设备已确认 reset。构造不协商特性或发表队列。
pub(crate) unsafe fn prepare_pci_transport(
    inventory: &PciInventory,
    device: &PciDevice,
) -> Result<(SomeTransport<'static>, VirtioIdentity, IrqNumber), VirtioInitError> {
    drop(task::current_task());
    let selected = selected_construct(inventory, device)?;
    let interrupt = device.interrupt.ok_or(ProbeError::Resource(
        "VirtIO PCI device has no INTx resource",
    ))?;
    // SAFETY: selected 的身份、capability 与完整 BAR 内范围由同库存验证；调用者
    // 承担实际功能独占、永久强序映射和全部消费者访问期内的设备保活义务。
    let transport = unsafe { PciTransport::new(selected) }?;
    Ok((
        transport.into(),
        VirtioIdentity::Pci {
            bus: device.bus,
            device: device.device,
            function: device.function,
            vendor_id: device.vendor_id,
            device_id: device.device_id,
        },
        interrupt,
    ))
}

/// 原页拥有者和库 driver 共存；未确认停止时禁止析构任何控制区。
pub struct VirtioDevice {
    driver: ManuallyDrop<VirtIOBlk<KernelHal, SomeTransport<'static>>>,
    ring: ManuallyDrop<CoherentDma>,
    pdu: ManuallyDrop<CoherentDma>,
    identity: VirtioIdentity,
    interrupt: IrqNumber,
    must_reset: bool,
}

/// 仅验收镜像读取仍由本实例拥有的 transport 与 ring，不更改设备状态。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct TestPrepareSnapshot {
    pub identity: VirtioIdentity,
    pub status: DeviceStatus,
    pub avail_index: u16,
    pub available_descriptors: usize,
    pub requires_reset: bool,
    pub irq_actions: usize,
}

impl VirtioDevice {
    /// 在普通 Task 中准备已发现 MMIO 块设备，不发表队列或 DRIVER_OK。
    ///
    /// 调用者须独占 candidate 的真实硬件，完整物理窗口须永久强序映射，身份来自
    /// 同一次发现；设备和映射须覆盖全部传输、通知及 IRQ 访问，直到尾部结束并确认 reset。
    pub unsafe fn prepare_mmio(candidate: &MmioBlockCandidate) -> Result<Self, VirtioInitError> {
        // SAFETY: 调用者保证同库存设备独占、完整永久强序映射及全部消费者寿命；
        // 候选字段直接传递，不构造第二份硬件 owner。
        let (transport, identity, interrupt) = unsafe {
            prepare_mmio_transport(
                candidate.registers,
                candidate.version,
                candidate.vendor_id,
                candidate.interrupt,
            )
        }?;
        Self::prepare(transport, identity, interrupt)
    }

    /// 只消费库存已选 BDF、cap 元数据与 BAR，不重新扫描或创建配置 owner。
    ///
    /// 调用者须独占此真实硬件，device 来自 inventory，库存 BAR 的完整 CPU 物理
    /// 窗口须永久强序映射；设备及映射须覆盖全部通知、IRQ 尾部和确认 reset。
    pub unsafe fn prepare_pci(
        inventory: &PciInventory,
        device: &PciDevice,
    ) -> Result<Self, VirtioInitError> {
        // SAFETY: 调用者承担同库存唯一功能、完整永久映射及全部消费者寿命；
        // 共用构造只产生该功能的传输，块类型、协商和控制页仍由本 owner 准备。
        let (transport, identity, interrupt) = unsafe { prepare_pci_transport(inventory, device) }?;
        Self::prepare(transport, identity, interrupt)
    }

    /// reset 确认后锁外预备所有 fallible 资源，尚未发表时错误自动归还原页。
    fn prepare(
        mut transport: SomeTransport<'static>,
        identity: VirtioIdentity,
        interrupt: IrqNumber,
    ) -> Result<Self, VirtioInitError> {
        let domain = qemu_dma_domain()?;
        if transport.device_type() != DeviceType::Block {
            return Err(ProbeError::Resource("selected VirtIO device is not block").into());
        }
        transport.request_reset();
        if !transport.reset_complete() {
            return Err(Error::ResetPending.into());
        }
        KernelHal::dma_barrier();
        let info = VirtIOBlk::<KernelHal, SomeTransport<'static>>::negotiate(&mut transport)?;
        let layout = RingLayout::new(info.queue_depth, transport.requires_legacy_layout())?;
        let ring = CoherentDma::allocate(domain, layout.byte_len())
            .map_err(|error| VirtioInitError::dma_prepare(error, transport.get_status()))?;
        let pdu_size = usize::from(info.tag_count)
            .checked_mul(PDU_STRIDE)
            .ok_or(Error::InvalidParam)?;
        let pdu = CoherentDma::allocate(domain, pdu_size)
            .map_err(|error| VirtioInitError::dma_prepare(error, transport.get_status()))?;
        // SAFETY: 原 ring owner 存活，真实 coherent CPU/DMA 同一页，尚未交设备访问。
        let queue = unsafe { VirtQueue::<KernelHal>::new(layout, ring.region()) }
            .map_err(|error| VirtioInitError::queue_prepare(error, transport.get_status()))?;
        // SAFETY: ring/PDU 原页与 driver 同寿命，绑定不发表；拥有者在发表后强制真实 reset。
        let driver = unsafe { VirtIOBlk::new(transport, info, queue, pdu.region()) }?;
        Ok(Self {
            driver: ManuallyDrop::new(driver),
            ring: ManuallyDrop::new(ring),
            pdu: ManuallyDrop::new(pdu),
            identity,
            interrupt,
            must_reset: false,
        })
    }

    /// 只读访问实际 driver；不返回第二份拥有者。
    pub fn driver(&self) -> &VirtIOBlk<KernelHal, SomeTransport<'static>> {
        &self.driver
    }
    /// 在实际 owner 保活期间借用后端并保留停止责任，调用者负责队列和通知的串行化。
    ///
    /// # Safety
    /// 不得移出、替换或交换后端；它必须始终关联本 owner 的硬件及 ring/PDU。
    /// 借用期间须保证对应请求的全部 DMA 拥有者存活，且不与 reset 或另一个访问者并行。
    pub unsafe fn driver_mut(&mut self) -> &mut VirtIOBlk<KernelHal, SomeTransport<'static>> {
        self.must_reset = true;
        &mut self.driver
    }
    /// 返回真实协商所得块设备事实。
    pub fn info(&self) -> BlkInfo {
        self.driver.info()
    }
    /// 返回第一批 IRQ domain 的真实身份。
    pub fn interrupt(&self) -> IrqNumber {
        self.interrupt
    }
    /// 返回不可变完整硬件身份。
    pub fn identity(&self) -> VirtioIdentity {
        self.identity
    }
    /// 返回原 ring/PDU 分配的裸视图；不提供安全 CPU 解引用或另一份页拥有权。
    pub fn control_regions(&self) -> (DmaRegion, DmaRegion) {
        (self.ring.region(), self.pdu.region())
    }
    /// 返回真实硬件状态，供初始化诊断和停止流程使用。
    pub fn status(&self) -> DeviceStatus {
        self.driver.status()
    }

    /// 在真实准备失败点、实例销毁前读取硬件 status 与本实例原 ring 的 avail.idx。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_prepare_snapshot(&self) -> TestPrepareSnapshot {
        let depth = self.info().queue_depth;
        let ring = self.ring.region();
        // 两种 split 布局的 avail 都紧接 descriptor 表，legacy 仅改变 used 对齐。
        let avail_index_offset =
            usize::from(depth) * SPLIT_RING_DESCRIPTOR_BYTES + core::mem::size_of::<u16>();
        assert!(avail_index_offset + core::mem::size_of::<u16>() <= ring.len);
        KernelHal::dma_read_barrier();
        // SAFETY: 本方法借用唯一实例，ring 原页仍 live；offset 是已验证 split
        // 布局内对齐的 u16，volatile 读取不创建 DMA 内存引用，不消费或修改队列。
        let avail_index = u16::from_le(unsafe {
            core::ptr::read_volatile(ring.cpu.as_ptr().add(avail_index_offset).cast::<u16>())
        });
        TestPrepareSnapshot {
            identity: self.identity,
            status: self.status(),
            avail_index,
            available_descriptors: self.driver.available_descriptors(),
            requires_reset: self.must_reset,
            irq_actions: crate::irq::test_action_count(self.interrupt),
        }
    }
    /// 确认本设备 ISR，不重新引入 DMA 停止责任；退休期仍须清本设备共享 IRQ 原因。
    pub fn ack_interrupt(&mut self) -> InterruptStatus {
        self.driver.ack_interrupt()
    }
    /// 在任何 queue_set 写入前标记停止责任；失败返回时仍保完整原页和 driver。
    pub fn publish(&mut self) -> Result<(), VirtioInitError> {
        self.must_reset = true;
        KernelHal::dma_barrier();
        self.driver.publish().map_err(Into::into)
    }
    /// 请求设备停止，写零不清除停止责任。
    pub fn request_reset(&mut self) {
        self.must_reset = true;
        KernelHal::dma_barrier();
        self.driver.request_reset();
    }
    /// 一次真实状态读回为零后执行完整屏障并清停止责任；未完成时保持全部 owner。
    pub fn reset_complete(&mut self) -> bool {
        if !self.driver.reset_complete() {
            return false;
        }
        KernelHal::dma_barrier();
        self.must_reset = false;
        true
    }
    /// 查询当前是否必须先确认设备停止；不代替真实状态读回。
    pub fn requires_reset(&self) -> bool {
        self.must_reset
    }
}

impl Drop for VirtioDevice {
    /// 仅确认停止或从未发表时销毁原 driver 和控制页；无 reset 写入或忙等。
    fn drop(&mut self) {
        assert!(
            !self.must_reset,
            "VirtIO control DMA dropped before confirmed reset"
        );
        // SAFETY: 从未发表或 reset 已真实确认并完整屏障；driver 先销毁，原页随后各归还一次。
        unsafe {
            ManuallyDrop::drop(&mut self.driver);
            ManuallyDrop::drop(&mut self.pdu);
            ManuallyDrop::drop(&mut self.ring);
        }
    }
}

/// 读取同一次库存的选定 capability 字段，checked 偏移不能回绕到身份配置字。
fn cap_word(
    inventory: &PciInventory,
    device: &PciDevice,
    cap: u8,
    field: u8,
) -> Result<u32, VirtioInitError> {
    let offset = cap
        .checked_add(field)
        .ok_or(ProbeError::Resource("PCI capability offset overflows"))?;
    Ok(inventory.read_config_word(device, offset)?)
}

/// 将 capability 的 BAR 内范围换算为已经映射的强序 CPU 窗口。
fn capability_region(
    device: &PciDevice,
    bar: u8,
    offset: u32,
    length: u32,
) -> Result<PciRegion, VirtioInitError> {
    let bar = device
        .bars
        .get(usize::from(bar))
        .and_then(Option::as_ref)
        .ok_or(ProbeError::Resource(
            "VirtIO capability references unassigned memory BAR",
        ))?;
    let offset = usize::try_from(offset)
        .map_err(|_| ProbeError::Resource("PCI capability offset is not addressable"))?;
    let len = usize::try_from(length)
        .map_err(|_| ProbeError::Resource("PCI capability length is not addressable"))?;
    let size = bar
        .physical
        .end
        .checked_sub(bar.physical.start)
        .ok_or(ProbeError::Resource("invalid PCI BAR physical extent"))?;
    let end = offset
        .checked_add(len)
        .ok_or(ProbeError::Resource("PCI capability extent overflows"))?;
    if len == 0 || size < end {
        return Err(ProbeError::Resource("PCI capability is outside assigned BAR").into());
    }
    let start = bar
        .physical
        .start
        .checked_add(offset)
        .ok_or(ProbeError::Resource("PCI capability address overflows"))?;
    let physical_end = start
        .checked_add(len)
        .ok_or(ProbeError::Resource("PCI capability end overflows"))?;
    let base = mmio_region_address(Region {
        start,
        end: physical_end,
    })?;
    Ok(PciRegion {
        base: NonNull::new(base as *mut u8)
            .ok_or(ProbeError::Resource("null PCI capability VA"))?,
        len,
    })
}

/// 使用已有 cap 数组选择四种协议窗口，不遍历硬件链或探测 BAR。
fn selected_construct(
    inventory: &PciInventory,
    device: &PciDevice,
) -> Result<SelectedConstruct, VirtioInitError> {
    let identity = inventory.read_config_word(device, PCI_IDENTITY)?;
    if identity as u16 != device.vendor_id
        || (identity >> u16::BITS) as u16 != device.device_id
        || device.vendor_id != VIRTIO_VENDOR_ID
    {
        return Err(ProbeError::Resource("PCI inventory identity changed").into());
    }
    let caps = device
        .capabilities
        .get(..device.capability_count)
        .ok_or(ProbeError::Resource(
            "PCI capability inventory count is invalid",
        ))?;
    let mut regions = [None; VIRTIO_PCI_CAP_REGION_COUNT];
    let mut notify_off_multiplier = 0;
    for cap in caps.iter().flatten() {
        if cap.id != PCI_CAP_ID_VNDR {
            continue;
        }
        let length = cap.private_header as u8;
        let kind = (cap.private_header >> u8::BITS) as u8;
        if !(VIRTIO_PCI_CAP_COMMON_CFG..=VIRTIO_PCI_CAP_DEVICE_CFG).contains(&kind) {
            continue;
        }
        if length < VIRTIO_PCI_CAP_BYTES
            || (kind == VIRTIO_PCI_CAP_NOTIFY_CFG && length < VIRTIO_PCI_NOTIFY_CAP_BYTES)
            || PCI_CONFIG_SPACE_BYTES < usize::from(cap.offset) + usize::from(length)
        {
            return Err(ProbeError::Resource("short VirtIO PCI capability").into());
        }
        let index = usize::from(kind - 1);
        if regions[index].is_some() {
            continue;
        }
        let bar = cap_word(inventory, device, cap.offset, VIRTIO_PCI_CAP_BAR)? as u8;
        let offset = cap_word(inventory, device, cap.offset, VIRTIO_PCI_CAP_OFFSET)?;
        let region_length = cap_word(inventory, device, cap.offset, VIRTIO_PCI_CAP_LENGTH)?;
        regions[index] = Some(capability_region(device, bar, offset, region_length)?);
        if kind == VIRTIO_PCI_CAP_NOTIFY_CFG {
            notify_off_multiplier =
                cap_word(inventory, device, cap.offset, VIRTIO_PCI_NOTIFY_CAP_MULT)?;
        }
    }
    Ok(SelectedConstruct {
        device_function: DeviceFunction {
            bus: device.bus,
            device: device.device,
            function: device.function,
        },
        vendor_id: device.vendor_id,
        device_id: device.device_id,
        common_cfg: regions[0].ok_or(VirtioPciError::MissingCommonConfig)?,
        notify_cfg: regions[1].ok_or(VirtioPciError::MissingNotifyConfig)?,
        notify_off_multiplier,
        isr_cfg: regions[2].ok_or(VirtioPciError::MissingIsrConfig)?,
        device_cfg: regions[3],
    })
}
