//! 真实候选的原页压力与发表边界；失败实例不与恢复实例并存。

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

/// 只等真实服务尾部，再整理空闲缓存；完整 MemoryStats 保留六域。
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

/// 只接收本例声明的一份独占原盘候选，不猜选多盘中的业务根设备。
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

/// 同一库存的候选只在上一实例已停止并释放后重新构造。
fn prepare(inventory: &DeviceInventory) -> Result<VirtioDevice, VirtioInitError> {
    if let Some(candidate) = inventory.mmio_blocks.first() {
        // SAFETY: fixture 串行独占唯一 scratch，BootInfo 的完整永久强序映射有效；
        // 每次调用前上一实例已确认 reset 或从未发表，并已销毁。
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
        // SAFETY: 同一次库存的唯一 BDF 与完整 BAR 永久映射，fixture 串行持唯一硬件责任。
        unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
    }
}

/// 仅特性拒绝例借真正 transport；没有 ring、queue_set 或第二设备 owner。
fn transport(inventory: &DeviceInventory) -> Result<SomeTransport<'static>, TestError> {
    if let Some(candidate) = inventory.mmio_blocks.first() {
        let base = drivers::mmio_region_address(candidate.registers)
            .map_err(|_| TestError("feature MMIO mapping invalid"))?;
        let header =
            NonNull::new(base as *mut VirtIOHeader).ok_or(TestError("feature MMIO header null"))?;
        // SAFETY: 本例声明的唯一原盘候选完整窗口已永久映射，fixture 尚无 driver/IRQ owner。
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
        // SAFETY: selected 属于本例声明的唯一原盘 BDF，完整配置/BAR 映射永久有效，无其它 owner。
        Ok(unsafe { PciTransport::new(selected) }
            .map_err(|_| TestError("feature PCI transport failed"))?
            .into())
    }
}

/// 页耗尽只消费预备凭证容量，最后一次申请在真实分配器上失败。
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

/// 真实 status 零确认前仍由 fixture 保住原控制页；超时返回错误并保活。
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

struct Owner {
    baseline: mm::MemoryStats,
    metadata: crate::workqueue::testing::MetadataStats,
    inventory: DeviceInventory,
    held: Vec<mm::OwnedPages>,
    device: Option<VirtioDevice>,
    disk: Option<DiskRef>,
    queue: Option<WorkQueue>,
    info: BlkInfo,
    identity: VirtioIdentity,
    interrupt: irq::IrqNumber,
    legacy: bool,
}

impl Owner {
    /// 当前真实 Info 决定 ring、PDU、tag 与完成域预算，不固定历史失败 tag/order。
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

    /// 放压后在同一候选上准备并真实发表；原 snapshot 检查发生在发表之前。
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

    /// 没有原设备实例被发表时的 OOM 保存 live status，不用复构后的寄存器代替。
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

    /// 独立 ring/PDU 失败例按当前真实布局只释放恰好 ring 原阶块。
    fn pages(&mut self, context: &mut TestContext<'_>, pdu: bool) -> TestResult {
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

        self.recover(context)?;
        context.expect_eq(irq::test_action_count(self.interrupt), actions);
        context.expect_eq(settled()?, resident);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }

    /// 原固定六个整阶预算查找真正 SG reserve 失败，前三张表已成功才有 live snapshot。
    fn sg(&mut self, context: &mut TestContext<'_>) -> TestResult {
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

    /// 原 ring/PDU 不重叠且空队列具完整 descriptor；全部资源成功后才 DRIVER_OK。
    fn geometry(&mut self, context: &mut TestContext<'_>) -> TestResult {
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

        let snapshot = device.test_prepare_snapshot();
        context.expect_eq(snapshot.status.contains(DeviceStatus::DRIVER_OK), false);
        context.expect_eq(snapshot.requires_reset, false);
        context.expect_eq(snapshot.avail_index, 0);

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
        // SAFETY: fixture 是唯一 owner，空队列无 payload，控制页保活且没有 IRQ/其它访问者。
        context.expect_eq(
            unsafe { device.driver_mut() }
                .pop_used()
                .map_err(|_| TestError("empty used ring invalid"))?,
            None,
        );
        Ok(())
    }

    /// 先放归真实压力，再停设备、同步关闭预留 Work 的原域；错误保原 owner 给 case-abort。
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

    /// 真实停止与原 join 完成后才析构候选/页/域，资源账必须回到本例 setup 前。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        let metadata = self.metadata;
        drop(self);
        context.expect_eq(settled()?, baseline);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }
}

struct Ring(Owner);
struct Pdu(Owner);
struct Sg(Owner);
struct Backend(Owner);

macro_rules! fixture {
    ($name:ident, $sg:expr) => {
        impl Fixture for $name {
            /// 每个独立 case 准备当前候选和原压力存储，不共享上例状态。
            fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
                Owner::new($sg).map(Self)
            }

            /// typed fixture 保活到原设备、IRQ、Work 与 manager join 都完成。
            fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce()
            }

            /// 普通 final 之前验证原六域归还。
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
    /// 完全耗尽真实原页令 ring 准备失败，并在同候选放压恢复。
    #[test]
    fn ring_page_oom_recovery(context: &mut TestContext<'_>, fixture: &mut Ring) -> TestResult {
        fixture.0.pages(context, false)
    }
}

#[::ktest::suite(fixture = Pdu, isolation = "Case", tags("block", "prepare", "B03"), resources("scratch"), timeout_ms = 20000)]
mod pdu {
    use super::*;
    /// 只还真实 ring 原阶预算令 PDU 页准备失败，并恢复同候选。
    #[test]
    fn pdu_page_oom_recovery(context: &mut TestContext<'_>, fixture: &mut Pdu) -> TestResult {
        fixture.0.pages(context, true)
    }
}

#[::ktest::suite(fixture = Sg, isolation = "Case", tags("block", "prepare", "B03"), resources("scratch"), timeout_ms = 30000)]
mod sg {
    use super::*;
    /// 固定表成功后的真正 SG heap reserve 失败保未发表状态和原六域。
    #[test]
    fn sg_heap_oom_recovery(context: &mut TestContext<'_>, fixture: &mut Sg) -> TestResult {
        fixture.0.sg(context)
    }
}

#[::ktest::suite(fixture = Backend, isolation = "Case", tags("block", "prepare", "B03"), resources("scratch"), timeout_ms = 15000)]
mod backend {
    use super::*;
    /// 验证真实控制页布局、现代/旧状态位和空 used，不包含 queue/quota 业务。
    #[test]
    fn controls_before_driver_ok(
        context: &mut TestContext<'_>,
        fixture: &mut Backend,
    ) -> TestResult {
        fixture.0.geometry(context)
    }
}

/// 指向真实 suite 槽，父驱动模块仅保留这些注册锚点。
pub(crate) fn anchors() -> [&'static SuiteSlot; 5] {
    [
        ring::__ktest_anchor(),
        pdu::__ktest_anchor(),
        sg::__ktest_anchor(),
        backend::__ktest_anchor(),
        features::anchor(),
    ]
}
