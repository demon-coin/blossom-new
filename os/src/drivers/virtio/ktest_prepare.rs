//! VirtIO 块设备准备阶段（Preparation）的内存压力、控制区布局与发布边界回归测试。
//!
//! # 模块职责与架构定位
//! 本模块位于 `drivers::virtio` 下，负责验证 VirtIO 块设备（`VirtioDevice`）与通用块设备对象（`Disk`）
//! 在两阶段初始化（`prepare` 分配资源与配置队列 ➔ `publish` / `start` 置位 `DRIVER_OK` 并注册中断）过程中的异常安全性与资源回收契约：
//! 1. **物理页与堆内存耗尽回滚**：当 Virtqueue 描述符环（Ring）页块、请求头尾控制区（PDU）页块、或请求标签散聚表（SG Table）预留容量遭遇内存不足（OOM）时，设备不得置位 `DRIVER_OK`，不得注册中断处理函数，已分配的 DMA 页与工作队列元数据必须完整归还，且释放内存压力后同一硬件设备必须能正常恢复初始化与发布。
//! 2. **控制区 DMA 内存布局与发布前状态**：验证 Ring 与 PDU 区域的页面对齐、容量下界、物理地址区间不重叠，以及发布前 `avail.idx == 0`、空闲描述符数等于队列深度、`used` 环为空等硬件契约。
//! 3. **现代传输层特性协商拒绝**：由子模块 [`features`] 验证遗漏 `VIRTIO_F_ACCESS_PLATFORM` 时的硬件握手拒绝与恢复。
//!
//! # 核心机制：VirtIO 块设备两阶段准备与 OOM 注入点
//! ```text
//!   VirtioDevice::prepare_mmio / prepare_pci
//!     ├── 1. 传输层特性协商 (ACKNOWLEDGE | DRIVER | FEATURES_OK)
//!     ├── 2. 分配 Virtqueue Ring 连续 DMA 页 ◄── [注入点 A: ring_page_oom_recovery]
//!     └── 3. 分配 Request PDU 连续 DMA 页    ◄── [注入点 B: pdu_page_oom_recovery]
//!            (此时尚未置位 DRIVER_OK，requires_reset = false)
//!
//!   Disk::prepare(device, workqueue)
//!     ├── 4. 分配固定大小槽表 (tags / completions / waiter_lists)
//!     └── 5. 为每个 tag 预留 SG Vec 容量     ◄── [注入点 C: sg_heap_oom_recovery]
//!            (若中途失败：销毁未发布 VirtioDevice，不置位 DRIVER_OK，不注册 IRQ)
//!
//!   VirtioDevice::publish() / Disk::start()
//!     └── 6. 置位 DRIVER_OK（并按需注册 IRQ） ◄── [验证点 D: controls_before_driver_ok]
//! ```
//!
//! # 并发与硬件生命周期契约
//! - 所有测试套件均采用 `isolation = "Case"` 串行运行，每次仅操作同一块独占的测试设备（`scratch` 或 `root`）。
//! - **严禁新旧实例并存**：对同一硬件重新调用 `prepare` 之前，上一个 `VirtioDevice` 或 `Disk` 实例必须已完成硬件复位（`status == 0`）或确认从未发布（未置位 `DRIVER_OK`），且其 Rust 实例已析构。
//! - 若硬件复位超时未确认（`reset` 返回错误），夹具 `quiesce` 会向上层测试框架报错并中止清理，保留 DMA 控制页不释放，防止设备仍在后台 DMA 访问已回收的物理页。

mod features;

use ::ktest::{Fixture, SuiteSlot, TestContext, TestError, TestResult};
use alloc::vec::Vec;
use core::ptr::NonNull;
use core::sync::atomic::Ordering;
use virtio_drivers::device::blk::{BlkInfo, PDU_STRIDE};
use virtio_drivers::queue::RingLayout;
use virtio_drivers::transport::{
    mmio::{MmioTransport, VirtIOHeader},
    pci::PciTransport,
    DeviceStatus, SomeTransport,
};

use super::{
    VirtioDevice, VirtioIdentity, VirtioInitError, VIRTIO_PCI_BLOCK_LEGACY,
    VIRTIO_PCI_BLOCK_MODERN, VIRTIO_VENDOR_ID,
};
use crate::block::{BlockError, Disk, DiskRef};
use crate::drivers::{self, DeviceInventory};
use crate::mm::dma::DmaError;
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, irq, mm, sched, time, timer};

/// 等待所有工作队列服务线程进入阻塞态且其余 CPU 核心全部空闲后，冲刷 SLUB 与每 CPU 页面缓存（PCP），返回当前物理内存六维统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("preparation resource deadline overflow"))?;
    crate::workqueue::testing::wait_services_blocked();
    while !(0..cpu::count())
        .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    {
        if deadline <= time::now() {
            return Err(TestError("preparation service tail did not settle"));
        }
        sched::yield_now();
    }
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 扫描设备清单并校验系统中恰好挂载了一个目标 VirtIO 块设备，返回该设备清单。
fn inventory() -> Result<DeviceInventory, TestError> {
    let inventory =
        drivers::discover().map_err(|_| TestError("preparation inventory discovery failed"))?;
    let count = inventory.mmio_blocks.len()
        + inventory
            .pci
            .devices()
            .iter()
            .filter(|device| {
                device.vendor_id == VIRTIO_VENDOR_ID
                    && matches!(
                        device.device_id,
                        VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
                    )
            })
            .count();
    if count != 1 {
        return Err(TestError(
            "preparation requires exactly one exclusive attached device",
        ));
    }
    Ok(inventory)
}

/// 从设备清单中取出唯一的 VirtIO 块设备候选（优先 MMIO，其次 PCI），执行第一阶段准备（分配 Ring/PDU DMA 控制页但暂不置位 `DRIVER_OK`）。
fn prepare(inventory: &DeviceInventory) -> Result<VirtioDevice, VirtioInitError> {
    if let Some(candidate) = inventory.mmio_blocks.first() {
        // SAFETY:
        // 1. `candidate` 来自启动阶段探测并建立永久强序 MMIO 映射的 `DeviceInventory`。
        // 2. 本模块各测试夹具按 `Case` 隔离级别串行运行，且每次调用 `prepare` 前上一个设备实例均已完成硬件复位或确认从未置位 `DRIVER_OK` 并已销毁，当前无其他实例并发访问该 MMIO 窗口。
        unsafe { VirtioDevice::prepare_mmio(candidate) }
    } else {
        let device = inventory
            .pci
            .devices()
            .iter()
            .find(|device| {
                device.vendor_id == VIRTIO_VENDOR_ID
                    && matches!(
                        device.device_id,
                        VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
                    )
            })
            .ok_or(crate::drivers::ProbeError::Resource(
                "scratch PCI candidate missing",
            ))?;
        // SAFETY:
        // 1. `device` 来自 `inventory.pci`，其配置空间与 BAR 区域已在内核启动时完成永久映射。
        // 2. 夹具串行独占该 PCI BDF 设备，且前序实例已停止并释放，不存在并发驱动实例。
        unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
    }
}

/// 为特性协商测试构造底层 VirtIO 传输层句柄（`SomeTransport`），不分配 Virtqueue 环或创建上层块设备对象。
fn transport(inventory: &DeviceInventory) -> Result<SomeTransport<'static>, TestError> {
    if let Some(candidate) = inventory.mmio_blocks.first() {
        let base = drivers::mmio_region_address(candidate.registers)
            .map_err(|_| TestError("feature MMIO mapping invalid"))?;
        let header =
            NonNull::new(base as *mut VirtIOHeader).ok_or(TestError("feature MMIO header null"))?;
        // SAFETY:
        // 1. `header` 由合法的 `mmio_region_address` 转换而来，指向永久映射的 VirtIO MMIO 寄存器窗口，且窗口长度 `candidate.registers.end - candidate.registers.start` 与硬件描述一致。
        // 2. 当前测试夹具独占该设备，未创建任何 `VirtioDevice` 或注册中断，无并发访问。
        let transport = unsafe {
            MmioTransport::new(header, candidate.registers.end - candidate.registers.start)
        }
        .map_err(|_| TestError("feature MMIO transport failed"))?;
        Ok(transport.into())
    } else {
        let device = inventory
            .pci
            .devices()
            .iter()
            .find(|device| {
                device.vendor_id == VIRTIO_VENDOR_ID
                    && matches!(
                        device.device_id,
                        VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
                    )
            })
            .ok_or(TestError("feature PCI candidate missing"))?;
        let selected = super::selected_construct(&inventory.pci, device)
            .map_err(|_| TestError("feature PCI capabilities invalid"))?;
        // SAFETY:
        // 1. `selected` 由 `selected_construct` 校验通过，对应唯一的 VirtIO 块设备 PCI BDF，其配置空间与 BAR 映射在内核生命周期内持续有效。
        // 2. 当前夹具独占该 PCI 功能，不存在其他传输层或驱动实例别名。
        Ok(unsafe { PciTransport::new(selected) }
            .map_err(|_| TestError("feature PCI transport failed"))?
            .into())
    }
}

/// 回收 SLUB 与 PCP 缓存后持续分配阶数为 0 的物理页并存入预分配容量的 `held` 向量，直至伙伴分配器彻底耗尽并返回 `None`。
fn exhaust(held: &mut Vec<mm::OwnedPages>) {
    assert!(held.is_empty());
    mm::shrink_all();
    mm::drain_all_pages();
    while let Some(pages) = mm::alloc_pages(0, false) {
        assert!(
            held.len() < held.capacity(),
            "preparation pressure capacity exhausted"
        );
        held.push(pages);
    }
    mm::drain_all_pages();
    assert!(
        mm::alloc_pages(0, false).is_none(),
        "preparation pressure was not real OOM"
    );
}

/// 向 `VirtioDevice` 发起硬件复位请求并轮询等待状态寄存器归零；若超时未归零则返回错误，调用方需保留设备实例以维持 DMA 控制页存活。
fn reset(device: &mut VirtioDevice) -> TestResult {
    device.request_reset();
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(2))
        .ok_or(TestError("preparation reset deadline overflow"))?;
    while !device.reset_complete() {
        if deadline <= time::now() {
            return Err(TestError(
                "preparation reset unconfirmed; retaining original DMA",
            ));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    if !device.status().is_empty() {
        return Err(TestError("preparation reset status was not zero"));
    }
    Ok(())
}

/// VirtIO 块设备准备阶段与 OOM 恢复测试的共享夹具状态持有者。
struct Owner {
    /// 测试开始前采集的系统物理内存六维统计基线，用于拆卸阶段核对内存零泄漏。
    baseline: mm::MemoryStats,
    /// 测试开始前采集的工作队列元数据统计基线，用于校验后台工作项与池资源归还。
    metadata: crate::workqueue::testing::MetadataStats,
    /// 硬件探测得到的设备清单，含唯一的测试用 VirtIO 块设备。
    inventory: DeviceInventory,
    /// 内存压力注入期间暂存已分配物理页的容器（在加压前已按系统总页数预留容量，加压期间不触发堆扩容）。
    held: Vec<mm::OwnedPages>,
    /// 当前处于已准备或已发布状态的 `VirtioDevice` 实例；若已移交 `Disk` 或已销毁则为 `None`。
    device: Option<VirtioDevice>,
    /// 当前由 `Disk::prepare` 构造的通用块设备引用；未构造或已销毁时为 `None`。
    disk: Option<DiskRef>,
    /// 仅供 `Sg` 测试套件使用的块完成工作队列；其余套件不需要构造 `Disk`，故为 `None`。
    queue: Option<WorkQueue>,
    /// 首次探测记录的块设备几何与队列参数（含 `queue_depth` 与 `tag_count`）。
    info: BlkInfo,
    /// 首次探测记录的设备硬件身份标识（MMIO 基址/版本或 PCI BDF）。
    identity: VirtioIdentity,
    /// 设备绑定的硬件中断号，用于断言未发布失败路径不会误注册中断处理函数。
    interrupt: irq::IrqNumber,
    /// 标识当前设备是否为 VirtIO Legacy MMIO（version 1），决定是否校验 `FEATURES_OK` 位及传统环布局。
    legacy: bool,
}

impl Owner {
    /// 创建测试状态持有者：采集内存与工作队列基线，通过一次探测性 `prepare` 获取设备参数（`BlkInfo`、身份与中断号）后立即释放，并预分配内存压力槽位与可选的完成工作队列。
    fn new(sg: bool) -> Result<Self, TestError> {
        let baseline = settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let inventory = inventory()?;
        let device =
            prepare(&inventory).map_err(|_| TestError("initial scratch preparation failed"))?;
        let info = device.info();
        let identity = device.identity();
        let interrupt = device.interrupt();
        let legacy = matches!(identity, VirtioIdentity::Mmio { version: 1, .. });
        drop(device);

        let mut held = Vec::new();
        held.try_reserve_exact(mm::stats().present_pages)
            .map_err(|_| TestError("pressure owner storage allocation failed"))?;
        let queue = if sg {
            Some(
                WorkQueue::try_new(WorkQueueConfig {
                    max_active: usize::from(info.tag_count) + 1,
                    ordered: false,
                    reclaim: true,
                })
                .map_err(|_| TestError("SG completion domain preparation failed"))?,
            )
        } else {
            None
        };
        Ok(Self {
            baseline,
            metadata,
            inventory,
            held,
            device: None,
            disk: None,
            queue,
            info,
            identity,
            interrupt,
            legacy,
        })
    }

    /// 在释放内存压力后对同一候选设备重新执行 `prepare`，校验未发布快照状态，随后调用 `publish()` 验证 `DRIVER_OK` 握手并完成硬件复位销毁。
    fn recover(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.device = Some(
            prepare(&self.inventory)
                .map_err(|_| TestError("same candidate failed after releasing pressure"))?,
        );
        let device = self.device.as_mut().unwrap();
        context.expect_eq(device.identity(), self.identity);
        let snapshot = device.test_prepare_snapshot();
        context.expect_eq(snapshot.avail_index, 0);
        context.expect_eq(
            snapshot.available_descriptors,
            usize::from(self.info.queue_depth),
        );
        context.expect_eq(snapshot.requires_reset, false);
        context.expect_eq(snapshot.status.contains(DeviceStatus::DRIVER_OK), false);

        device
            .publish()
            .map_err(|_| TestError("same candidate publication after OOM failed"))?;
        context.expect_eq(device.status().contains(DeviceStatus::DRIVER_OK), true);
        context.expect_eq(
            device.status().contains(DeviceStatus::FEATURES_OK),
            !self.legacy,
        );
        reset(device)?;
        drop(self.device.take());
        Ok(())
    }

    /// 校验 Ring 或 PDU 阶段因 OOM 准备失败时错误枚举中携带的实时设备状态字：确认 `DRIVER_OK` 未置位，且 Modern 设备已置位 `FEATURES_OK`。
    fn oom_status(&self, context: &mut TestContext<'_>, error: VirtioInitError) -> TestResult {
        let status = match error {
            VirtioInitError::NoMemory { status }
            | VirtioInitError::Dma {
                error: DmaError::NoMemory,
                status,
            } => status,
            _ => return Err(TestError("unexpected ring/PDU preparation error")),
        };
        context.expect_eq(status.contains(DeviceStatus::DRIVER_OK), false);
        context.expect_eq(status.contains(DeviceStatus::FEATURES_OK), !self.legacy);
        context.diagnostic(format_args!(
            "prepare-oom identity={:?} depth={} tags={} live-status={:#x} driver-ok=0",
            self.identity,
            self.info.queue_depth,
            self.info.tag_count,
            status.bits()
        ));
        Ok(())
    }

    /// 验证 Virtqueue 描述符环（`pdu == false`）或请求头尾控制区 PDU（`pdu == true`）在物理页耗尽时的分配失败回滚及放压后的恢复。
    fn pages(&mut self, context: &mut TestContext<'_>, pdu: bool) -> TestResult {
        // Step 1: 采样当前常驻内存、工作队列元数据及中断注册计数；若测试目标为 PDU OOM，则预先分配并保有一块恰好等于 Ring 大小的同阶连续物理页。
        let resident = settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let actions = irq::test_action_count(self.interrupt);
        let reserved = if pdu {
            let bytes = RingLayout::new(self.info.queue_depth, self.legacy)
                .map_err(|_| TestError("ring layout invalid"))?
                .byte_len();
            let order = bytes
                .div_ceil(mm::PAGE_SIZE)
                .next_power_of_two()
                .trailing_zeros() as usize;
            Some(
                mm::alloc_pages(order, false)
                    .ok_or(TestError("original ring budget reservation failed"))?,
            )
        } else {
            None
        };

        // Step 2: 耗尽系统剩余全部物理页；若为 PDU 测试则释放刚才预留的 Ring 页块并冲刷 PCP 回伙伴系统，使得后续 prepare 恰好够分配 Ring 但在分配 PDU 时触发 OOM。
        exhaust(&mut self.held);
        drop(reserved);
        mm::drain_all_pages();
        let result = prepare(&self.inventory);
        let error = match result {
            Ok(device) => {
                self.device = Some(device);
                None
            }
            Err(error) => Some(error),
        };

        // Step 3: 释放内存压力，断言 prepare 因 OOM 失败、设备未置位 DRIVER_OK、未注册中断且已分配的 Ring 页（若有）已完整回滚归还。
        self.held.clear();
        if self.device.is_some() {
            return Err(TestError(
                "ring/PDU unexpectedly prepared under real page pressure",
            ));
        }
        self.oom_status(
            context,
            error.ok_or(TestError("ring/PDU OOM error missing"))?,
        )?;
        context.expect_eq(irq::test_action_count(self.interrupt), actions);
        context.expect_eq(settled()?, resident);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);

        // Step 4: 验证释放压力后同一硬件候选能够重新 prepare、publish 并正常复位回收。
        self.recover(context)?;
        context.expect_eq(irq::test_action_count(self.interrupt), actions);
        context.expect_eq(settled()?, resident);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }

    /// 验证 `Disk::prepare` 在固定大小请求表分配成功后、为某个 tag 预留散聚表（SG `Vec`）容量遭遇堆内存耗尽时的回滚与恢复行为。
    fn sg(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 采样常驻内存、工作队列元数据及中断计数基线，依次以 order 0..=5（1 至 32 页）作为堆内存预算窗口进行探测。
        let resident = settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let actions = irq::test_action_count(self.interrupt);
        let mut failure = None;
        for order in 0..=5 {
            let reserved = mm::alloc_pages(order, false)
                .ok_or(TestError("SG original budget block allocation failed"))?;
            self.device = Some(
                prepare(&self.inventory)
                    .map_err(|_| TestError("SG unique candidate preparation failed"))?,
            );
            context.expect_eq(self.device.as_ref().unwrap().identity(), self.identity);
            context.expect_eq(
                self.device.as_ref().unwrap().info().tag_count,
                self.info.tag_count,
            );
            context.expect_eq(
                self.device.as_ref().unwrap().info().queue_depth,
                self.info.queue_depth,
            );

            // Step 2: 在 VirtioDevice 准备完成后耗尽剩余物理页，再释放 order 阶预留块回伙伴系统，令 Disk::test_prepare_observed 在有限页面预算下尝试分配内部表与 SG 缓冲区。
            let mut observation = None;
            exhaust(&mut self.held);
            drop(reserved);
            mm::drain_all_pages();
            let result = Disk::test_prepare_observed(
                self.device.take().unwrap(),
                self.queue.as_ref().unwrap(),
                &mut observation,
            );
            let error = match result {
                Ok(disk) => {
                    self.disk = Some(disk);
                    None
                }
                Err(error) => Some(error),
            };

            // Step 3: 释放压力并清理可能构造出的 Disk，校验每次尝试后中断计数与物理内存均完整归还；若捕获到 SG 预留失败快照（前三张固定表已分配成功），则校验未发布状态并跳出搜索循环。
            self.held.clear();
            if let Some(disk) = self.disk.as_ref() {
                disk.stop()
                    .map_err(|_| TestError("unpublished pressure Disk stop unconfirmed"))?;
                drop(self.disk.take());
            }
            context.expect_eq(irq::test_action_count(self.interrupt), actions);
            context.expect_eq(settled()?, resident);
            context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
            if let Some((tag, snapshot)) = observation {
                context.expect_eq(error, Some(BlockError::NoMemory));
                context.expect_eq(snapshot.identity, self.identity);
                context.expect_eq(tag < self.info.tag_count, true);
                context.expect_eq(snapshot.irq_actions, actions);
                context.expect_eq(snapshot.status.contains(DeviceStatus::DRIVER_OK), false);
                context.expect_eq(snapshot.requires_reset, false);
                context.expect_eq(snapshot.avail_index, 0);
                context.expect_eq(
                    snapshot.available_descriptors,
                    usize::from(self.info.queue_depth),
                );
                context.expect_eq(
                    snapshot.status.contains(DeviceStatus::FEATURES_OK),
                    !self.legacy,
                );
                failure = Some((order, tag, snapshot.status));
                break;
            }
            if error != Some(BlockError::NoMemory) {
                return Err(TestError(
                    "bounded SG budget reached success before actual SG reserve failure",
                ));
            }
        }
        let (order, tag, status) = failure.ok_or(TestError(
            "no actual SG reserve failure in original six page budgets",
        ))?;

        // Step 4: 在无内存压力下重新执行 VirtioDevice::prepare 与 Disk::prepare，验证完整块设备准备路径与 VirtioDevice::publish 均可正常恢复且无内存泄漏。
        self.device = Some(
            prepare(&self.inventory)
                .map_err(|_| TestError("SG candidate failed after releasing pressure"))?,
        );
        let snapshot = self.device.as_ref().unwrap().test_prepare_snapshot();
        context.expect_eq(snapshot.identity, self.identity);
        context.expect_eq(snapshot.status.contains(DeviceStatus::DRIVER_OK), false);
        context.expect_eq(snapshot.avail_index, 0);
        self.disk = Some(
            Disk::prepare(self.device.take().unwrap(), self.queue.as_ref().unwrap())
                .map_err(|_| TestError("fixed request/SG preparation did not recover"))?,
        );
        context.expect_eq(irq::test_action_count(self.interrupt), actions);
        self.disk
            .as_ref()
            .unwrap()
            .stop()
            .map_err(|_| TestError("recovered unpublished Disk stop failed"))?;
        drop(self.disk.take());
        context.expect_eq(settled()?, resident);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);

        self.recover(context)?;
        context.expect_eq(settled()?, resident);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        context.diagnostic(format_args!("request-sg-oom identity={:?} budget-order={} budget-pages={} failed-tag={} live-status={:#x} depth={} tags={} original-six-budgets=1 unpublished=1 irq-unregistered=1 avail-empty=1", self.identity, order, 1usize << order, tag, status.bits(), self.info.queue_depth, self.info.tag_count));
        Ok(())
    }

    /// 验证 `VirtioDevice` 的 Ring 与 PDU 控制区 DMA 布局约束（容量充足、4KiB 页对齐、互不重叠）以及 `publish()` 前后的状态位与空队列不变量。
    fn geometry(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 准备 VirtioDevice 并校验 Ring 与 PDU 控制区的长度下界、4096 字节对齐以及 DMA 区间无重叠。
        self.device = Some(
            prepare(&self.inventory)
                .map_err(|_| TestError("backend geometry preparation failed"))?,
        );
        let device = self.device.as_mut().unwrap();
        let info = device.info();
        let (ring, pdu) = device.control_regions();
        let layout = RingLayout::new(info.queue_depth, self.legacy)
            .map_err(|_| TestError("backend ring layout invalid"))?;
        context.expect_eq(
            layout.byte_len() <= ring.len && usize::from(info.tag_count) * PDU_STRIDE <= pdu.len,
            true,
        );
        context.expect_eq(ring.device.value() & 4095, 0);
        context.expect_eq(pdu.device.value() & 4095, 0);
        context.expect_eq(
            ring.device.value() + ring.len as u64 <= pdu.device.value()
                || pdu.device.value() + pdu.len as u64 <= ring.device.value(),
            true,
        );

        // Step 2: 校验 publish() 之前未置位 DRIVER_OK、无需硬件复位且 avail_index 为 0。
        let snapshot = device.test_prepare_snapshot();
        context.expect_eq(snapshot.status.contains(DeviceStatus::DRIVER_OK), false);
        context.expect_eq(snapshot.requires_reset, false);
        context.expect_eq(snapshot.avail_index, 0);

        // Step 3: 调用 publish() 激活设备，校验状态位（DRIVER_OK / FEATURES_OK）、可用描述符数量等于 queue_depth 且 used 环为空。
        device
            .publish()
            .map_err(|_| TestError("backend geometry publication failed"))?;
        context.expect_eq(device.status().contains(DeviceStatus::DRIVER_OK), true);
        context.expect_eq(
            device.status().contains(DeviceStatus::FEATURES_OK),
            !self.legacy,
        );
        context.expect_eq(
            device.driver().available_descriptors(),
            usize::from(info.queue_depth),
        );
        // SAFETY:
        // 1. 当前测试夹具独占 `device`，且未注册任何中断处理函数或启动后台工作任务，无并发访问。
        // 2. 队列自 `publish()` 以来从未提交过任何 I/O 请求，Ring 与 PDU 控制区物理页由 `self.device` 持续保活。
        context.expect_eq(
            unsafe { device.driver_mut() }
                .pop_used()
                .map_err(|_| TestError("empty used ring invalid"))?,
            None,
        );
        Ok(())
    }

    /// 静止化夹具：释放暂存的压力物理页，复位或停止在测 `VirtioDevice`/`Disk`，并同步关闭测试工作队列；若硬件无法安全停止则返回错误以阻止后续销毁 DMA 页。
    fn quiesce(&mut self) -> TestResult {
        self.held.clear();
        if let Some(device) = self.device.as_mut() {
            reset(device)?;
        }
        if let Some(disk) = self.disk.as_ref() {
            disk.stop()
                .map_err(|_| TestError("preparation Disk cannot safely stop"))?;
        }
        if let Some(queue) = self.queue.as_ref() {
            queue.close_sync();
        }
        Ok(())
    }

    /// 拆卸夹具：在硬件已静止、工作队列已关闭后析构全部字段，并校验系统物理内存六维统计与工作队列元数据完整恢复至初始基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        let metadata = self.metadata;
        drop(self);
        context.expect_eq(settled()?, baseline);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }
}

/// Virtqueue 描述符环（Ring）页面分配 OOM 与恢复测试夹具（内部封装 [`Owner`]，第 0 字段持有测试状态）。
struct Ring(Owner);
/// 请求头尾控制区（PDU）页面分配 OOM 与恢复测试夹具（内部封装 [`Owner`]，第 0 字段持有测试状态）。
struct Pdu(Owner);
/// 块设备请求散聚表（SG Table）堆预留 OOM 与恢复测试夹具（内部封装 [`Owner`]，第 0 字段持有测试状态）。
struct Sg(Owner);
/// VirtIO 后端控制区几何布局与 `DRIVER_OK` 发布边界测试夹具（内部封装 [`Owner`]，第 0 字段持有测试状态）。
struct Backend(Owner);

/// 为封装了 [`Owner`] 的各测试套件夹具统一实现 `ktest::Fixture` 生命周期方法。
macro_rules! fixture {
    ($name:ident, $sg:expr) => {
        impl Fixture for $name {
            /// 为当前独立测试用例初始化候选设备信息、内存基线与压力容器。
            fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
                Owner::new($sg).map(Self)
            }

            /// 静止化夹具：确保硬件设备复位停止且工作队列线程全部退出后再进入拆卸阶段。
            fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce()
            }

            /// 拆卸夹具：释放测试资源并断言物理内存六维统计与工作队列元数据零泄漏。
            fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
                self.0.tear_down(context)
            }
        }
    };
}

fixture!(Ring, false);
fixture!(Pdu, false);
fixture!(Sg, true);
fixture!(Backend, false);

#[::ktest::suite(fixture = Ring, isolation = "Case", tags("block", "prepare", "B03"), resources("scratch"), timeout_ms = 20000)]
mod ring {
    use super::*;
    /// 验证物理页彻底耗尽时 Virtqueue 描述符环（Ring）分配失败回滚，以及释放压力后同一硬件设备的正常准备与发布恢复。
    ///
    /// # 测试目标
    /// 确认当伙伴系统无任何可用物理页时，`VirtioDevice::prepare_mmio/prepare_pci` 在分配 Ring DMA 页时返回 `NoMemory`，
    /// 设备状态不置位 `DRIVER_OK`、不注册中断；释放内存压力后同一设备可成功完成准备、发布与复位。
    ///
    /// # 场景步骤（Given-When-Then）
    /// 1. **Given**：采样内存与中断基线，通过 `exhaust` 耗尽系统全部空闲物理页。
    /// 2. **When**：对候选设备调用 `prepare`。
    /// 3. **Then**：返回携带实时状态的 `NoMemory` 错误，状态字不含 `DRIVER_OK`，中断注册数不变，释放压力后内存统计完全回到基线。
    /// 4. **When / Then**：调用 `recover` 重新准备并发布同一设备，确认 `DRIVER_OK` 置位成功并完成复位回收。
    #[test]
    fn ring_page_oom_recovery(context: &mut TestContext<'_>, fixture: &mut Ring) -> TestResult {
        fixture.0.pages(context, false)
    }
}

#[::ktest::suite(fixture = Pdu, isolation = "Case", tags("block", "prepare", "B03"), resources("scratch"), timeout_ms = 20000)]
mod pdu {
    use super::*;
    /// 验证物理页仅够分配 Virtqueue Ring 但不足以分配请求控制区（PDU）时的回滚行为，以及释放压力后同一设备的恢复。
    ///
    /// # 测试目标
    /// 确认当可用连续物理页恰好只够分配 Ring 而不够分配 PDU 时，`prepare` 在第二处 DMA 分配失败后能完整释放先前已分配的 Ring 物理页，
    /// 不置位 `DRIVER_OK`、不注册中断，并在释放内存压力后可正常恢复。
    ///
    /// # 场景步骤（Given-When-Then）
    /// 1. **Given**：根据设备 `queue_depth` 计算 Ring 所需页阶并预先分配一块同阶物理页，随后耗尽系统其余全部物理页，再释放该预留块回伙伴系统。
    /// 2. **When**：对候选设备调用 `prepare`（此时 Ring 分配成功，紧随其后的 PDU 分配触发 OOM）。
    /// 3. **Then**：返回 `NoMemory` 错误，状态字不含 `DRIVER_OK`，已分配的 Ring 页面随错误回滚释放，释放压力后内存六维统计与中断计数完全恢复基线。
    /// 4. **When / Then**：调用 `recover` 验证同一设备可正常完成准备、发布与复位。
    #[test]
    fn pdu_page_oom_recovery(context: &mut TestContext<'_>, fixture: &mut Pdu) -> TestResult {
        fixture.0.pages(context, true)
    }
}

#[::ktest::suite(fixture = Sg, isolation = "Case", tags("block", "prepare", "B03"), resources("scratch"), timeout_ms = 30000)]
mod sg {
    use super::*;
    /// 验证 `Disk::prepare` 在固定请求表分配成功后、为某个 tag 预留散聚表（SG Table）堆内存失败时的回滚与恢复。
    ///
    /// # 测试目标
    /// 确认在有限页面预算（1 至 32 页）下，当 `Disk::prepare` 完成了 `tags`、`completions`、`waiter_lists` 三张固定表分配，
    /// 但在为第 `tag` 个槽位预留 `sg_tables` 容量遭遇 `NoMemory` 时，底层 `VirtioDevice` 保持未发布状态（无 `DRIVER_OK`、无中断注册、`avail.idx == 0`），
    /// 且已分配的堆表与 DMA 页在回滚后零泄漏；释放压力后 `Disk::prepare` 与 `VirtioDevice::publish` 均可正常成功。
    ///
    /// # 场景步骤（Given-When-Then）
    /// 1. **Given**：在 `order = 0..=5` 循环中先完成 `VirtioDevice` 准备，耗尽剩余物理页后仅释放 `2^order` 页预算供堆分配器使用。
    /// 2. **When**：调用 `Disk::test_prepare_observed` 尝试构造块设备并记录 SG 预留失败时的快照。
    /// 3. **Then**：在预算范围内捕获到真实的 SG 预留 `BlockError::NoMemory`，快照显示 `DRIVER_OK == false`、`avail_index == 0`、中断未注册，且回滚后内存完全恢复基线。
    /// 4. **When / Then**：释放压力后重新执行 `Disk::prepare` 与 `recover`，确认设备可正常准备、停止、发布与复位。
    #[test]
    fn sg_heap_oom_recovery(context: &mut TestContext<'_>, fixture: &mut Sg) -> TestResult {
        fixture.0.sg(context)
    }
}

#[::ktest::suite(fixture = Backend, isolation = "Case", tags("block", "prepare", "B03"), resources("scratch"), timeout_ms = 15000)]
mod backend {
    use super::*;
    /// 验证 `VirtioDevice` 在置位 `DRIVER_OK` 前后的控制区 DMA 几何布局、特性/状态位握手以及空队列描述符状态。
    ///
    /// # 测试目标
    /// 确认 `prepare` 分配的 Ring 与 PDU DMA 区域满足长度下界、4KiB 页对齐且物理地址不重叠；
    /// 确认 `publish()` 前未置位 `DRIVER_OK`，`publish()` 后置位 `DRIVER_OK`（Modern 设备同时置位 `FEATURES_OK`），可用描述符数等于队列深度且 `used` 环为空。
    ///
    /// # 场景步骤（Given-When-Then）
    /// 1. **Given / When**：调用 `prepare` 构造未发布的 `VirtioDevice`，读取其 `control_regions` 与准备快照。
    /// 2. **Then**：Ring 与 PDU 长度不小于理论布局、设备地址按 4096 字节对齐且两段区间不重叠；快照中 `DRIVER_OK == false`、`avail_index == 0`。
    /// 3. **When**：调用 `device.publish()` 激活设备。
    /// 4. **Then**：设备状态包含 `DRIVER_OK`（非 Legacy 设备包含 `FEATURES_OK`），可用描述符数等于 `queue_depth`，`pop_used()` 返回 `Ok(None)`。
    #[test]
    fn controls_before_driver_ok(
        context: &mut TestContext<'_>,
        fixture: &mut Backend,
    ) -> TestResult {
        fixture.0.geometry(context)
    }
}

/// 返回本模块全部 5 个准备阶段测试套件的注册槽数组，供父级 `drivers::virtio` 模块统一挂载。
pub(crate) fn anchors() -> [&'static SuiteSlot; 5] {
    [
        ring::__ktest_anchor(),
        pdu::__ktest_anchor(),
        sg::__ktest_anchor(),
        backend::__ktest_anchor(),
        features::anchor(),
    ]
}
