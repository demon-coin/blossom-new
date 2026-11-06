//! 全局物理块设备表、分区视图目录与统一停机协调器。
//!
//! # 模块职责与架构定位
//! 本模块在内核单次设备探测完成后，汇总 VirtIO-Blk（MMIO/PCI）、板载 SD/MMC、PCI AHCI 与 `initrd` 内存盘，
//! 构建全局唯一的永久单例 [`Storage`]。它的上游调用者是内核初始化入口（`initialize`）、根/附加文件系统挂载逻辑
//! （`lookup`）以及系统关机/重启路径（`stop_all`）；下游依赖 `drivers` 的硬件枚举结果、`block::Disk` 的请求队列管理与
//! `block::partition` 的 GPT/MBR 分区表解析。
//!
//! # 物理槽位与命名视图布局
//! ```text
//!   Storage.names / Storage.disks (固定长度一一对应)
//!   ┌──────────────┬──────────────┬──────────────┬──────────────┐
//!   │ Virtio(0..k) │    Mmc(0)    │   Scsi(0)    │     Ram      │
//!   ├──────────────┼──────────────┼──────────────┼──────────────┤
//!   │ Some(DiskRef)│ Option<Disk> │ Option<Disk> │ Some(DiskRef)│
//!   └──────┬───────┴──────┬───────┴──────┬───────┴──────┬───────┘
//!          │              │ (无卡/无盘为 None)          │
//!          ▼              ▼                             ▼
//!   Storage.views: Vec<(DeviceSelector, BlockView)>
//!     - (/dev/vda1, 分区视图), (/dev/vda, 整盘视图), (/dev/mmcblk0p1, ...), (/dev/ram0, ...)
//! ```
//! 若某个设备在停机时无法确认硬件重置完成（如返回 `StopUnconfirmed`）或仍有活跃挂载（`Busy`），
//! 对应的 `DiskRef` 与共享完成工作队列将永久保留在原槽位中，防止后台残余 DMA 踩踏已回收的内核内存。
//!
//! # 锁获取顺序与并发契约
//! 1. `Storage.closed`（`SleepMutex<bool>`）：串行化全局 `stop_all` 停机流程，持锁期间可调用 `Disk::stop` 睡眠等待；
//! 2. `Disk` 内部锁（如 `Disk.control`、`Disk.queue`）：在未持有 `disks` / `views` 自旋锁的前提下调用；
//! 3. `Storage.disks` / `Storage.views`（`SpinLock`）：叶级自旋锁，仅用于短暂克隆 `DiskRef`、替换或清空视图列表，严禁相互嵌套或持锁睡眠。

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

/// 全局唯一的块存储管理表实例，仅在内核普通启动阶段初始化一次。
static STORAGE: Once<Storage> = Once::new();

/// 全局块存储设备与分区视图注册表。
pub(crate) struct Storage {
    /// 与 `names` 下标一一对应的物理磁盘强引用槽位（由自旋锁保护）；控制器的卡槽/端口无介质或已确认安全停机后为 `None`。
    disks: SpinLock<Vec<Option<DiskRef>>>,
    /// 启动阶段一次性固定的候选物理设备名列表（按 VirtIO、MMC、SCSI、RAM 顺序排列，初始化后只读）。
    names: Vec<DeviceName>,
    /// 已发现的整盘及分区视图映射表（由自旋锁保护），供上层按 `DeviceSelector`（如 `/dev/vda1`）精确查找。
    views: SpinLock<Vec<(DeviceSelector, BlockView)>>,
    /// 所有物理磁盘共用的块层异步推进与完成回调工作队列（开启内存回收保障 `reclaim: true`）。
    queue: WorkQueue,
    /// 串行化 `stop_all` 停机流程的睡眠互斥锁；内部布尔值为 `true` 表示所有物理盘与工作队列均已彻底关闭。
    closed: SleepMutex<bool>,
    /// 标记初始化及分区扫描是否全部成功完成的原子标志；为 `false` 时 `lookup` 拒绝向外暴露任何视图。
    ready: AtomicBool,
}

impl Storage {
    /// 获取全局 `Storage` 单例引用；若尚未调用 `initialize` 则返回 `None`。
    pub(crate) fn get() -> Option<&'static Self> {
        STORAGE.get()
    }

    /// 在普通可睡眠初始化任务中一次性构建全部块设备、启动硬件队列并扫描分区视图；
    /// 若启动参数要求的 `root` 或 `extra` 物理设备不存在（如空卡槽），或任一设备启动/分区解析失败，则回滚停止所有已准备设备并返回错误。
    ///
    /// # Safety
    /// 1. `inventory` 必须来自当前内核启动过程中的唯一真实设备枚举结果，调用方必须移交其中全部块设备的独占控制权，且对应的 MMIO/PCI BAR 强序映射在内核生命周期内永久有效；
    /// 2. `BootInfo` 选定的板载 SD/MMC 与 AHCI 控制器寄存器、时钟/复位配置区、卡检测 GPIO 及中断线必须由内核独占且永久映射，启动固件已退出且不再访问这些硬件，设备 DMA 窗口与缓存一致性掩码必须与真实硬件拓扑一致；
    /// 3. 若存在 `initrd` 内存盘，其物理内存区间必须由 `BootInfo` 永久保留并建立恒等/线性直映；在该内存盘实例的 `Disk::stop` 彻底执行完毕前，启动固件、用户态地址空间、其它外设或其它内存盘实例均不得并发读写该区间。
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

        // Step 1: 预检设备数量、初始化 VirtIO 传输层并分配全局 Storage 骨架与共享工作队列。
        // SAFETY: 调用方已按函数 `# Safety` 契约保证 `inventory` 与平台资源的独占权及永久映射；`prepare` 仅协商传输层，尚未开启数据请求。
        let (prepared, devices) = unsafe { Self::prepare(inventory) }?;
        let storage = STORAGE.call_once(|| prepared);
        let result = (|| {
            // Step 2: 按固定顺序将 VirtIO、SD/MMC、AHCI 与内存盘构造为 Disk 实例，并立即存入永久槽位纳入停机管理。
            let mut index = 0;
            for device in devices {
                let disk = Disk::prepare(device, &storage.queue)?;
                storage.disks.lock()[index] = Some(disk);
                index += 1;
            }
            if let Some(resources) = boot::info().board.mmc.as_ref() {
                // SAFETY:
                // 1. 调用方契约保证 `BootInfo` 中的 MMC 控制器、配置区与卡检测 GPIO 已被永久映射且由内核独占；
                // 2. 返回的 `DiskRef` 立即存入全局 `storage.disks` 槽位保活，保证在中断、工作队列与 `stop` 全部结束前不会释放。
                let disk = unsafe { Disk::prepare_sdmmc(resources, &storage.queue) }?;
                storage.disks.lock()[index] = disk;
                index += 1;
            }
            if let Some(selected) = boot::info().board.ahci.as_ref() {
                let resources = inventory.pci.resolve_ahci(selected).map_err(probe_error)?;
                // SAFETY:
                // 1. `resolve_ahci` 已从本次唯一 PCI 库存中解析出完整的 ABAR 映射与中断号，调用方契约保证该 PCI 功能独占且映射永久有效；
                // 2. 返回的 `DiskRef` 立即存入全局 `storage.disks` 槽位保活，若后续启动失败也由 `stop_all` 统一等待硬件停稳后再释放。
                let disk = unsafe { Disk::prepare_ahci(&resources, &storage.queue) }?;
                storage.disks.lock()[index] = disk;
                index += 1;
            }
            if let Some(region) = boot::info().initrd {
                // SAFETY:
                // 1. 调用方契约保证 `initrd` 物理区间在内存管理接管前已完整保留、建立永久直映，且无任何外部并发访问；
                // 2. 返回的 `DiskRef` 立即存入全局 `storage.disks` 槽位保活，直至全部 CPU 拷贝、工作队列回调及 `stop` 停机流程彻底结束。
                let disk = unsafe { Disk::prepare_ram(region, &storage.queue) }?;
                storage.disks.lock()[index] = Some(disk);
                index += 1;
            }
            assert_eq!(
                index,
                storage.names.len(),
                "storage inventory changed during preparation"
            );

            // Step 3: 核实启动参数指定的根设备与可选附加设备均已实际就绪（排除无卡/无盘的空槽位）。
            for device in core::iter::once(root).chain(extra) {
                let present =
                    storage.names.iter().enumerate().any(|(index, name)| {
                        *name == device && storage.owned_disk(index).is_some()
                    });
                if !present {
                    return Err(BlockError::NotReady);
                }
            }

            // Step 4: 逐个注册中断并激活所有物理磁盘队列，随后在锁外读取各盘分区表并填充视图目录。
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
            // Step 5 (失败回滚): 所有已构造的 Disk 均已登记在永久槽位中；调用 stop_all 排空并关闭已确认停稳的设备，无法确认重置的设备留在原槽位隔离。
            return Err(storage.stop_all().err().unwrap_or(error));
        }
        storage.ready.store(true, Ordering::Release);
        Ok(storage)
    }

    /// 统计候选块设备总数，完成所有 VirtIO-Blk（MMIO 与 PCI）传输层握手，并预分配 `Storage` 槽位数组与共享工作队列；
    /// 原生 SD/MMC、AHCI 控制器与内存盘需要在 `Storage` 单例落位并持有共享 `WorkQueue` 后再逐一构造。
    ///
    /// # Safety
    /// 调用方必须满足与 [`Self::initialize`] 相同的硬件独占权与 MMIO/PCI BAR 永久强序映射前置条件。
    unsafe fn prepare(
        inventory: &DeviceInventory,
    ) -> Result<(Self, Vec<VirtioDevice>), BlockError> {
        // Step 1: 统计 MMIO/PCI VirtIO-Blk、板载 MMC/AHCI 及 initrd 内存盘数量，校验字母盘上限与非零总数。
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

        // Step 2: 初始化所有 VirtIO-Blk 传输层（MMIO 先于 PCI），握手失败时输出早期诊断日志并中止。
        let mut devices = Vec::new();
        devices
            .try_reserve_exact(virtio_count)
            .map_err(|_| BlockError::NoMemory)?;
        for candidate in &inventory.mmio_blocks {
            // SAFETY: `candidate` 来自本次唯一设备枚举，调用方契约保证该 MMIO 块设备由内核独占且寄存器强序映射永久有效。
            devices.push(unsafe { VirtioDevice::prepare_mmio(candidate) }.map_err(init_error)?);
        }
        for device in pci_blocks {
            // SAFETY: `device` 来自本次唯一 PCI 枚举，调用方契约保证该 PCI 功能由内核独占且完整 BAR 映射在设备生命周期内永久有效。
            devices.push(
                unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }.map_err(init_error)?,
            );
        }

        // Step 3: 计算共享工作队列所需的最大并发活跃度（每个设备需容纳全部请求槽完成 Work 外加 1 个进度推进 Work），并分配槽位表与设备名表。
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

    /// 在所有物理磁盘启动成功后，于自旋锁外依次读取每块盘的 MBR/GPT 分区表，生成分区子视图与整盘视图列表；
    /// 无介质的空槽位直接跳过，而存在分区表签名但内容损坏或越界时会使整个初始化失败。
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

    /// 返回当前实际持有的物理磁盘数量（不含无介质的空槽位，也不按分区重复计数）。
    pub(crate) fn count(&self) -> usize {
        self.disks.lock().iter().flatten().count()
    }

    /// 按设备与分区选择器精确查找对应的块视图；仅在 `Storage` 就绪后返回克隆视图，分区不存在时返回 `None` 而不会回退为整盘。
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

    /// 按物理槽位下标克隆对应的 `DiskRef`（不受 `ready` 标志限制，供初始化未完成时的回滚停机路径使用）。
    fn owned_disk(&self, index: usize) -> Option<DiskRef> {
        self.disks.lock().get(index).and_then(Clone::clone)
    }

    /// 在可睡眠任务上下文中串行停止全部物理磁盘并关闭共享工作队列。
    ///
    /// 仅当某块磁盘返回 `Ok(())` 或已确认重置完成的 `Err(BlockError::DeviceFault)` 时才将其从槽位移除释放；
    /// 若仍有挂载（`Busy`）或重置未确认（`StopUnconfirmed`），则保留该 `DiskRef` 与共享工作队列不释放，避免残留 DMA 访问悬垂内存。
    pub(crate) fn stop_all(&self) -> Result<(), BlockError> {
        let mut closed = self.closed.lock();
        if *closed {
            return Ok(());
        }
        // Step 1: 立即关闭对外视图查询入口并清空分区视图缓存，释放视图层持有的 DiskRef 引用。
        self.ready.store(false, Ordering::Release);
        let views = core::mem::take(&mut *self.views.lock());
        drop(views);

        // Step 2: 逐槽调用 Disk::stop 排空请求、停止 DMA 并同步注销中断与工作项；仅移除已确认硬件停稳的磁盘。
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

        // Step 3: 仅当所有物理磁盘均已安全停机并移出槽位后，才同步关闭共享工作队列并标记全局关闭完成。
        if self.disks.lock().iter().all(Option::is_none) {
            self.queue.close_sync();
            *closed = true;
        }
        first_error.map_or(Ok(()), Err)
    }
}

/// 输出 VirtIO 传输层初始化失败的早期串口日志，并将错误映射为对应的 `BlockError`。
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

/// 输出 PCI/IRQ 设备资源解析失败的早期串口日志，并按具体失败原因转换为对应的 `BlockError`。
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
