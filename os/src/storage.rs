//! 一次库存建立物理设备拥有表和精确名字视图；未确认停止的设备保留原槽。

use alloc::vec::Vec;
use core::convert::TryFrom;
use core::sync::atomic::{AtomicBool, Ordering};
use spin::Once;

use crate::block::partition::{self, DeviceName, DeviceSelector, LETTERED_DISK_COUNT};
use crate::block::{BlockError, BlockView, Disk, DiskRef, NATIVE_TAG_COUNT};
use crate::boot;
use crate::drivers::virtio::{
    VirtioDevice, VirtioInitError, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::{DeviceInventory, ProbeError};
use crate::mm::dma::DmaError;
use crate::sync::{SleepMutex, SpinLock};
use crate::workqueue::{WorkQueue, WorkQueueConfig};

static STORAGE: Once<Storage> = Once::new();

/// 物理 owner 数由一次库存固定，分区仅增加范围视图，不增加硬件实例。
pub(crate) struct Storage {
    disks: SpinLock<Vec<Option<DiskRef>>>,
    names: Vec<DeviceName>,
    views: SpinLock<Vec<(DeviceSelector, BlockView)>>,
    queue: WorkQueue,
    closed: SleepMutex<bool>,
    ready: AtomicBool,
}

impl Storage {
    /// 借用唯一永久表；初始化成功以前 lookup 和 disk 都不授予数据访问。
    pub(crate) fn get() -> Option<&'static Self> {
        STORAGE.get()
    }

    /// 在唯一普通初始化 Task 中准备全部实际介质；所需物理设备缺失时失败，未选空槽不发表。
    ///
    /// # Safety
    /// inventory 须来自本次唯一真实发现，调用者独占全部块设备并保持完整强序映射。
    /// BootInfo 选定的 MMC/AHCI 及其 setup、卡检测 GPIO、实际 ABAR/IRQ 也须独占并永久映射，固件已退出；
    /// AHCI 的 DMA 恒等窗口与各设备 coherency/mask 必须反映真实平台。
    /// 固件 initrd 须由 BootInfo 保留到内核结束且永久直映；固件、用户映射、其他设备和
    /// 另一 RAM disk owner 均不得访问该范围，直到本表 Disk 的 stop 全尾结束。
    #[cfg_attr(
        feature = "ktest",
        expect(
            dead_code,
            reason = "普通 init 准备唯一永久 Storage；ktest 自行拥有磁盘 fixture，仍编译同一真实准备和停止调用链"
        )
    )]
    pub(crate) unsafe fn initialize(
        inventory: &DeviceInventory,
        root: DeviceName,
        extra: Option<DeviceName>,
    ) -> Result<&'static Self, BlockError> {
        if STORAGE.get().is_some() {
            return Err(BlockError::Busy);
        }

        // SAFETY: 调用者传入唯一库存、设备独占权及永久映射，准备尚不发表任何数据请求。
        let (prepared, devices) = unsafe { Self::prepare(inventory) }?;
        let storage = STORAGE.call_once(|| prepared);
        let result = (|| {
            let mut index = 0;
            for device in devices {
                let disk = Disk::prepare(device, &storage.queue)?;
                storage.disks.lock()[index] = Some(disk);
                index += 1;
            }
            if let Some(resources) = boot::info().board.mmc.as_ref() {
                // SAFETY: 同一启动快照已映射选定控制器、setup 及独立卡检测 GPIO，调用者独占此设备；
                // 准备成功后立即进入永久 owning 槽，发表前由 Disk 登记稳定 IRQ action。
                let disk = unsafe { Disk::prepare_sdmmc(resources, &storage.queue) }?;
                storage.disks.lock()[index] = disk;
                index += 1;
            }
            if let Some(selected) = boot::info().board.ahci.as_ref() {
                let resources = inventory.pci.resolve_ahci(selected).map_err(probe_error)?;
                // SAFETY: 资源由本次唯一库存解析完整 ABAR/IRQ；永久映射和独占功能由调用者保证，
                // 原 Disk 进入 owning 槽后才 start，错误仍由本表保持到确认停止。
                let disk = unsafe { Disk::prepare_ahci(&resources, &storage.queue) }?;
                storage.disks.lock()[index] = disk;
                index += 1;
            }
            if let Some(region) = boot::info().initrd {
                // SAFETY: BootInfo 在 MM 接管前保留完整 initrd，入口唯一且无冲突映射或访问；
                // 永久表保原 Disk 到所有 CPU 复制、请求、IRQ/Work 和 stop 真尾结束。
                let disk = unsafe { Disk::prepare_ram(region, &storage.queue) }?;
                storage.disks.lock()[index] = Some(disk);
                index += 1;
            }
            assert_eq!(
                index,
                storage.names.len(),
                "storage inventory changed during preparation"
            );

            for device in core::iter::once(root).chain(extra) {
                let present =
                    storage.names.iter().enumerate().any(|(index, name)| {
                        *name == device && storage.owned_disk(index).is_some()
                    });
                if !present {
                    return Err(BlockError::NotReady);
                }
            }

            for index in 0..storage.names.len() {
                if let Some(disk) = storage.owned_disk(index) {
                    disk.start()?;
                }
            }
            let views = storage.discover_views()?;
            *storage.views.lock() = views;
            Ok(())
        })();

        if let Err(error) = result {
            // 所有已经准备的 owner 均已在永久表内；未确认停止时不摘原槽和完成域。
            return Err(storage.stop_all().err().unwrap_or(error));
        }
        storage.ready.store(true, Ordering::Release);
        Ok(storage)
    }

    /// 一次预备库存 VirtIO 传输、全部物理拥有槽与原完成域，原生控制器在永久表发表后准备。
    ///
    /// # Safety
    /// 调用者独占库存全部块设备及 BootInfo 选定原生资源，保持其永久映射，与 initialize 义务一致。
    unsafe fn prepare(
        inventory: &DeviceInventory,
    ) -> Result<(Self, Vec<VirtioDevice>), BlockError> {
        let pci_blocks = inventory.pci.devices().iter().filter(|device| {
            device.vendor_id == VIRTIO_VENDOR_ID
                && matches!(
                    device.device_id,
                    VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
                )
        });
        let virtio_count = inventory
            .mmio_blocks
            .len()
            .checked_add(pci_blocks.clone().count())
            .ok_or(BlockError::InvalidRange)?;
        if usize::from(LETTERED_DISK_COUNT) < virtio_count {
            return Err(BlockError::Unsupported);
        }
        let ram_count = usize::from(boot::info().initrd.is_some());
        let mmc_count = usize::from(boot::info().board.mmc.is_some());
        let ahci_count = usize::from(boot::info().board.ahci.is_some());
        let count = virtio_count
            .checked_add(mmc_count + ahci_count + ram_count)
            .ok_or(BlockError::InvalidRange)?;
        if count == 0 {
            return Err(BlockError::NotReady);
        }

        let mut devices = Vec::new();
        devices
            .try_reserve_exact(virtio_count)
            .map_err(|_| BlockError::NoMemory)?;
        for candidate in &inventory.mmio_blocks {
            // SAFETY: 候选来自同次真实库存，调用者独占该候选且完整强序映射永久有效。
            devices.push(unsafe { VirtioDevice::prepare_mmio(candidate) }.map_err(init_error)?);
        }
        for device in pci_blocks {
            // SAFETY: BDF 来自本库存，调用者独占真实功能，完整 BAR 映射覆盖 owner 寿命。
            devices.push(
                unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }.map_err(init_error)?,
            );
        }

        // 原生控制器和 RAM 各有八软件槽及一个原 progress，硬件单通道独立于回调容量。
        let max_active = devices.iter().try_fold(
            (mmc_count + ahci_count + ram_count) * (usize::from(NATIVE_TAG_COUNT) + 1),
            |count, device| {
                count
                    .checked_add(usize::from(device.info().tag_count) + 1)
                    .ok_or(BlockError::InvalidRange)
            },
        )?;
        let mut disks = Vec::new();
        disks
            .try_reserve_exact(count)
            .map_err(|_| BlockError::NoMemory)?;
        disks.resize_with(count, || None);
        let mut names = Vec::new();
        names
            .try_reserve_exact(count)
            .map_err(|_| BlockError::NoMemory)?;
        for index in 0..virtio_count {
            names.push(DeviceName::Virtio(
                u8::try_from(index).map_err(|_| BlockError::InvalidRange)?,
            ));
        }
        if mmc_count == 1 {
            names.push(DeviceName::Mmc(0));
        }
        if ahci_count == 1 {
            names.push(DeviceName::Scsi(0));
        }
        if ram_count == 1 {
            names.push(DeviceName::Ram);
        }

        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| BlockError::NoMemory)?;
        Ok((
            Self {
                disks: SpinLock::new(disks),
                names,
                views: SpinLock::new(Vec::new()),
                queue,
                closed: SleepMutex::new(false),
                ready: AtomicBool::new(false),
            },
            devices,
        ))
    }

    /// 全部实际物理盘 start 成功后在锁外读表，跳过无介质槽；已识别格式错误使初始化失败。
    fn discover_views(&self) -> Result<Vec<(DeviceSelector, BlockView)>, BlockError> {
        let mut views = Vec::new();
        views
            .try_reserve_exact(self.count())
            .map_err(|_| BlockError::NoMemory)?;
        for (index, device) in self.names.iter().copied().enumerate() {
            let Some(disk) = self.owned_disk(index) else {
                continue;
            };
            let whole = BlockView::whole(disk);
            let partitions = partition::discover(&whole)?;
            views
                .try_reserve(partitions.len() + 1)
                .map_err(|_| BlockError::NoMemory)?;
            for partition in partitions {
                let view = BlockView::new(
                    whole.disk().clone(),
                    partition.start_sector,
                    partition.sectors,
                )?;
                let selector = DeviceSelector {
                    device,
                    partition: Some(partition.number),
                };
                views.push((selector, view));
            }
            views.push((
                DeviceSelector {
                    device,
                    partition: None,
                },
                whole,
            ));
        }
        Ok(views)
    }

    /// 返回实际持有的物理盘数；无介质槽和分区视图均不计入。
    pub(crate) fn count(&self) -> usize {
        self.disks.lock().iter().flatten().count()
    }

    /// 只返回已发现的完全相同设备与分区，不把缺失分区替换为整盘。
    pub(crate) fn lookup(&self, selector: DeviceSelector) -> Option<BlockView> {
        if !self.ready.load(Ordering::Acquire) {
            return None;
        }
        self.views
            .lock()
            .iter()
            .find(|(name, _)| *name == selector)
            .map(|(_, view)| view.clone())
    }

    /// 初始化失败时仍可取原 owner，ready 不控制停止责任。
    fn owned_disk(&self, index: usize) -> Option<DiskRef> {
        self.disks.lock().get(index).and_then(Clone::clone)
    }

    /// 遍历固定槽停止每个实际物理盘至全尾，未确认的 owner 与完成域保持原位。
    pub(crate) fn stop_all(&self) -> Result<(), BlockError> {
        let mut closed = self.closed.lock();
        if *closed {
            return Ok(());
        }
        self.ready.store(false, Ordering::Release);
        let views = core::mem::take(&mut *self.views.lock());
        drop(views);

        let mut first_error = None;
        for index in 0..self.names.len() {
            let Some(disk) = self.owned_disk(index) else {
                continue;
            };
            let result = disk.stop();
            if result == Ok(()) || result == Err(BlockError::DeviceFault) {
                let removed = self.disks.lock()[index].take();
                drop(removed);
            }
            if let Err(error) = result {
                first_error.get_or_insert(error);
            }
            drop(disk);
        }

        if self.disks.lock().iter().all(Option::is_none) {
            self.queue.close_sync();
            *closed = true;
        }
        first_error.map_or(Ok(()), Err)
    }
}

/// 保存准备时真实错误日志，不以之后的 reset 覆盖原协商失败原因。
fn init_error(error: VirtioInitError) -> BlockError {
    crate::early_println!("STORAGE INIT error={:?}", error);
    match error {
        VirtioInitError::NoMemory { .. }
        | VirtioInitError::Dma {
            error: DmaError::NoMemory,
            ..
        } => BlockError::NoMemory,
        VirtioInitError::Protocol(error) => error.into(),
        _ => BlockError::InvalidRange,
    }
}

/// 记录完整资源错误后按其真实类别返回，不把解析失败改成驱动不支持或换盘重试。
fn probe_error(error: ProbeError) -> BlockError {
    use crate::irq::IrqError;
    use virtio_drivers::transport::pci::bus::PciError;
    crate::early_println!("STORAGE RESOURCE error={:?}", error);
    match error {
        ProbeError::NoMemory => BlockError::NoMemory,
        ProbeError::AlreadyDiscovered => BlockError::Busy,
        ProbeError::Resource(_) => BlockError::InvalidRange,
        ProbeError::Pci(PciError::CommandRejected) => BlockError::DeviceFault,
        ProbeError::Pci(_) => BlockError::InvalidRange,
        ProbeError::Irq(IrqError::NotInitialized) => BlockError::NotReady,
        ProbeError::Irq(IrqError::InvalidSource) => BlockError::InvalidRange,
        ProbeError::Irq(IrqError::ActionsFull) => BlockError::Busy,
    }
}
