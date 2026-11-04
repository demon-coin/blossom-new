//! 唯一 UART 与两台真实块设备的外部 IRQ；LA 必须来自同一实际 INTx。

mod uart;

use super::{Bio, BioAllocator, BioCompletion, BlkOperation, Disk, DiskRef, IoResult};
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::{self, DeviceInventory};
use crate::mm::dma::MappedPages;
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, irq, mm, sched, time, timer};
use ::ktest::{Fixture, SuiteSlot, TestContext, TestError, TestResult};
use alloc::vec::Vec;
use core::sync::atomic::Ordering;
use virtio_drivers::{BufferDirection, MappedSegment};

/// 单调真实谓词等待，睡眠不算 ISR、Blocked 或完成证据。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("block IRQ deadline overflow"))?;
    while !ready() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 所有参与者安全停止后才读取完整六域，不能用定长延时替代原 join。
fn settled() -> Result<mm::MemoryStats, TestError> {
    crate::workqueue::testing::wait_services_blocked();
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("block IRQ settle deadline overflow"))?;
    while !(0..cpu::count())
        .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    {
        if deadline <= time::now() {
            return Err(TestError("block IRQ services did not settle"));
        }
        sched::yield_now();
    }
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 只从同一库存按稳定槽/BDF选择两台实例，不读取根盘业务身份或写其内容。
fn devices(inventory: &DeviceInventory) -> Result<[VirtioDevice; 2], TestError> {
    let mut pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    let pci_count = pci.clone().count();
    if inventory.mmio_blocks.len() + pci_count != 2 {
        return Err(TestError(
            "multi-device IRQ requires exactly two attached root/extra devices",
        ));
    }
    if inventory.mmio_blocks.len() == 2 {
        // SAFETY: host 只给此 Case 两个独占测试盘，库存和完整永久 MMIO 映射有效；
        // 尚无其它 driver/IRQ owner，每个候选只构造一份实例。
        let first = unsafe { VirtioDevice::prepare_mmio(&inventory.mmio_blocks[0]) }
            .map_err(|_| TestError("IRQ first MMIO preparation failed"))?;
        // SAFETY: 第二个不同物理窗口属于同库存，不与 first 共享设备或控制页。
        let second = unsafe { VirtioDevice::prepare_mmio(&inventory.mmio_blocks[1]) }
            .map_err(|_| TestError("IRQ second MMIO preparation failed"))?;
        Ok([first, second])
    } else if pci_count == 2 {
        // SAFETY: 两个 BDF 属于本库存且永久 BAR 映射有效，每个仅有本 fixture 的唯一 owner。
        let first = unsafe { VirtioDevice::prepare_pci(&inventory.pci, pci.next().unwrap()) }
            .map_err(|_| TestError("IRQ first PCI preparation failed"))?;
        // SAFETY: 第二个实际 BDF 与 first 不同，全部 CPU/device 窗口已由库存验证。
        let second = unsafe { VirtioDevice::prepare_pci(&inventory.pci, pci.next().unwrap()) }
            .map_err(|_| TestError("IRQ second PCI preparation failed"))?;
        Ok([first, second])
    } else {
        Err(TestError(
            "mixed MMIO/PCI IRQ topology is not the selected hardware profile",
        ))
    }
}

struct Request {
    bio: Option<Bio>,
    completion: BioCompletion,
    original: MappedSegment,
    unit: usize,
}
impl Request {
    /// 仅 Read 一个 logical block，原页、Bio 与完成存储都在 start 之前预备。
    fn new(disk: &Disk) -> Result<Self, TestError> {
        let unit = disk.limits().logical_block_size as usize;
        if mm::PAGE_SIZE < unit {
            return Err(TestError("IRQ logical block exceeds original page"));
        }
        let pages = mm::alloc_pages(0, true)
            .ok_or(TestError("IRQ original Read page allocation failed"))?;
        let mapped = MappedPages::new(pages, 0, unit, BufferDirection::DeviceToDriver)
            .map_err(|_| TestError("IRQ original Read page mapping failed"))?;
        let original = mapped
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable");
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(1)
            .map_err(|_| TestError("IRQ Read owner storage allocation failed"))?;
        memory.push(mapped);
        // SAFETY: 原页已清零且只有此 Bio 拥有，无 CPU 裸别名；只读范围通过真实 Disk 准入，
        // 无 callback，fixture 到真实完成或 reset 后收权并持续拥有原完成对象。
        let bio = unsafe { Bio::try_new(disk, BlkOperation::Read, 0, memory, 0, None) }
            .map_err(|_| TestError("IRQ Read Bio preparation failed"))?;
        let completion = bio.completion();
        Ok(Self {
            bio: Some(bio),
            completion,
            original,
            unit,
        })
    }

    /// 同一个 Read 的真实 used/status 完成后收回原页，核地址/方向及只可领取一次。
    fn reclaim(&self, context: &mut TestContext<'_>) -> TestResult {
        until("IRQ Read did not complete", || {
            self.completion.is_complete()
        })?;
        context.expect_eq(
            self.completion.result(),
            Some(IoResult {
                completed: self.unit,
                error: None,
            }),
        );
        let memory = self
            .completion
            .take_memory()
            .map_err(|_| TestError("IRQ completed original page not reclaimable"))?;
        if memory.len() != 1 {
            return Err(TestError("IRQ original page owner count changed"));
        }
        let mapped = &memory[0];
        let segment = mapped
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable");
        context.expect_eq(
            (segment.address, segment.length, segment.direction),
            (
                self.original.address,
                self.original.length,
                self.original.direction,
            ),
        );
        context.expect_eq(mapped.is_device_owned(), false);
        context.expect_eq(
            self.completion.take_memory().err(),
            Some(super::BlockError::Busy),
        );
        drop(memory);
        Ok(())
    }
}

struct MultiIrq {
    baseline: mm::MemoryStats,
    metadata: crate::workqueue::testing::MetadataStats,
    _inventory: DeviceInventory,
    disks: [DiskRef; 2],
    queues: [WorkQueue; 2],
    sources: [irq::IrqNumber; 2],
    identities: [VirtioIdentity; 2],
    legacy: [bool; 2],
    actions: [usize; 2],
    requests: [Request; 4],
    uart: uart::Uart,
    activation_error: bool,
}
impl Fixture for MultiIrq {
    /// 两台设备与唯一 UART 的全部稳定存储先准备；start 失败仍返回强 owner 清理。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let inventory =
            drivers::discover().map_err(|_| TestError("IRQ inventory discovery failed"))?;
        let devices = devices(&inventory)?;
        let sources = [devices[0].interrupt(), devices[1].interrupt()];
        let identities = [devices[0].identity(), devices[1].identity()];
        let legacy =
            identities.map(|identity| matches!(identity, VirtioIdentity::Mmio { version: 1, .. }));
        let actions = [
            irq::test_action_count(sources[0]),
            irq::test_action_count(sources[1]),
        ];
        if cfg!(target_arch = "loongarch64") && sources[0] != sources[1] {
            return Err(TestError(
                "LA two real PCI devices do not share INTx; require slots 0x5/0x9",
            ));
        }
        if cfg!(target_arch = "riscv64") && sources[0] == sources[1] {
            return Err(TestError(
                "RV two real MMIO devices must have distinct PLIC sources",
            ));
        }
        let queue0 = WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(devices[0].info().tag_count) + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("IRQ first completion domain preparation failed"))?;
        let queue1 = match WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(devices[1].info().tag_count) + 1,
            ordered: false,
            reclaim: true,
        }) {
            Ok(queue) => queue,
            Err(_) => {
                queue0.close_sync();
                return Err(TestError("IRQ second completion domain preparation failed"));
            }
        };
        let queues = [queue0, queue1];
        let mut prepared = core::iter::IntoIterator::into_iter(devices);
        let disk0 = match Disk::prepare(prepared.next().unwrap(), &queues[0]) {
            Ok(disk) => disk,
            Err(_) => {
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(TestError("IRQ first Disk preparation failed"));
            }
        };
        let disk1 = match Disk::prepare(prepared.next().unwrap(), &queues[1]) {
            Ok(disk) => disk,
            Err(_) => {
                drop(disk0);
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(TestError("IRQ second Disk preparation failed"));
            }
        };
        let disks = [disk0, disk1];
        let requests = (|| {
            Ok([
                Request::new(&disks[0])?,
                Request::new(&disks[1])?,
                Request::new(&disks[1])?,
                Request::new(&disks[1])?,
            ])
        })();
        let requests = match requests {
            Ok(requests) => requests,
            Err(error) => {
                drop(disks);
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(error);
            }
        };
        let uart = match uart::Uart::prepare() {
            Ok(uart) => uart,
            Err(error) => {
                drop(disks);
                for queue in &queues {
                    queue.close_sync();
                }
                return Err(error);
            }
        };
        let mut fixture = Self {
            baseline,
            metadata,
            _inventory: inventory,
            disks,
            queues,
            sources,
            identities,
            legacy,
            actions,
            requests,
            uart,
            activation_error: false,
        };
        for disk in &fixture.disks {
            if let Err(error) = disk.start() {
                fixture.activation_error = true;
                context.diagnostic(format_args!(
                    "multi-device IRQ activation failed: {:?}; retaining original owner",
                    error
                ));
                break;
            }
        }
        Ok(fixture)
    }

    /// 释放真实 IRQ gates 后合作停 DMA，同步注销设备和 UART，再关原域并原 manager join。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.enable_disks()?;
        for disk in &self.disks {
            disk.stop().map_err(|_| {
                TestError("multi-device IRQ stop unconfirmed; retaining original DMA/domain")
            })?;
        }
        self.uart.quiesce()?;
        for queue in &self.queues {
            queue.close_sync();
        }
        Ok(())
    }

    /// 原 action/Work/Task 已离栈后释放所有拥有者，恢复默认 UART 并核六域和 action 账。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        let metadata = self.metadata;
        let sources = self.sources;
        let actions = self.actions;
        drop(self);
        for index in 0..2 {
            context.expect_eq(irq::test_action_count(sources[index]), actions[index]);
        }
        context.expect_eq(settled()?, baseline);
        context.expect_eq(crate::workqueue::testing::metadata_stats(), metadata);
        Ok(())
    }
}

/// gate 只改变原 token 的准入；queue 锁与 handler 不依赖此 token 存储锁。
fn disable(disk: &Disk) {
    if let Some(token) = disk.irq.lock().as_ref() {
        token.disable_sync();
    }
}
/// 开放同一原 token；Stopped Disk 已无 token，不能重装被注销的 action。
fn enable(disk: &Disk) {
    if let Some(token) = disk.irq.lock().as_ref() {
        token.enable();
    }
}

impl MultiIrq {
    /// 共享 INTx 的两个原 action 同时开放，再一次解屏蔽，不能留下只有假邻居可处理的窗口。
    fn enable_disks(&self) -> TestResult {
        if self.sources[0] == self.sources[1] {
            let first = self.disks[0].irq.lock();
            let second = self.disks[1].irq.lock();
            match (first.as_ref(), second.as_ref()) {
                (Some(first), Some(second)) => {
                    if !irq::testing::enable_pair(first, second) {
                        return Err(TestError(
                            "shared INTx original actions cannot be enabled together",
                        ));
                    }
                }
                (Some(token), None) | (None, Some(token)) => token.enable(),
                (None, None) => {}
            }
        } else {
            for disk in &self.disks {
                enable(disk);
            }
        }
        Ok(())
    }
    /// 仅消费预建 Bio，测试没有软件 handler 调用或额外 worker 进度旁路。
    fn submit(&mut self, request: usize, disk: usize) {
        self.disks[disk].submit_bio(
            self.requests[request]
                .bio
                .take()
                .expect("IRQ request submitted twice"),
        );
    }

    /// gate 尚闭时读原 used.idx，证明两个设备完成与 UART FIFO 已同时待处理。
    fn pending(&self, context: &mut TestContext<'_>) -> TestResult {
        until(
            "two hardware used rings and UART FIFO never became pending together",
            || {
                super::tests::used_index(&self.disks[0], self.legacy[0]) == 1
                    && super::tests::used_index(&self.disks[1], self.legacy[1]) == 1
                    && crate::console::receive_pending()
            },
        )?;
        for disk in &self.disks {
            context.expect_eq(disk.test_snapshot().irq_queue, 0);
            context.expect_eq(disk.test_snapshot().completed, 0);
        }
        Ok(())
    }

    /// 三个真实 producer 先一起 pending，再经唯一外部 domain 和原 Disk ISR 路径完成。
    fn exercise(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error {
            return Err(TestError("multi-device IRQ activation failed"));
        }
        let uart_source = self.uart.source();
        context.expect_eq(
            uart_source != self.sources[0] && uart_source != self.sources[1],
            true,
        );
        let dispatch = [
            irq::testing::dispatch_count(self.sources[0]),
            irq::testing::dispatch_count(self.sources[1]),
            irq::testing::dispatch_count(uart_source),
        ];
        let added = if self.sources[0] == self.sources[1] {
            2
        } else {
            1
        };
        for index in 0..2 {
            context.expect_eq(
                irq::test_action_count(self.sources[index]),
                self.actions[index] + added,
            );
        }
        disable(&self.disks[0]);
        disable(&self.disks[1]);
        self.uart.begin(b'I')?;
        self.submit(0, 0);
        self.submit(1, 1);
        self.pending(context)?;
        self.uart.enable();
        self.enable_disks()?;
        self.requests[0].reclaim(context)?;
        self.requests[1].reclaim(context)?;
        self.uart.receive(context, b'I')?;
        for (index, previous) in dispatch[..2].iter().enumerate() {
            context.expect_eq(0 < self.disks[index].test_snapshot().irq_queue, true);
            context.expect_eq(
                *previous < irq::testing::dispatch_count(self.sources[index]),
                true,
            );
        }
        context.expect_eq(
            dispatch[2] < irq::testing::dispatch_count(uart_source),
            true,
        );

        // 只有邻设备重新产生 IRQ：共享 action 的另一个真实 ISR 必须返回 empty，仍不截断派发。
        {
            let queue = self.disks[0].queue.lock();
            context.expect_eq(queue.phase, super::disk::QueuePhase::Live);
            context.expect_eq(queue.failure, None);
        }
        let empty = self.disks[0].test_snapshot().irq_empty;
        let queue_irqs = self.disks[1].test_snapshot().irq_queue;
        self.submit(2, 1);
        self.requests[2].reclaim(context)?;
        context.expect_eq(queue_irqs < self.disks[1].test_snapshot().irq_queue, true);
        if self.sources[0] == self.sources[1] {
            context.expect_eq(empty < self.disks[0].test_snapshot().irq_empty, true);
            let queue = self.disks[0].queue.lock();
            context.expect_eq(queue.phase, super::disk::QueuePhase::Live);
            context.expect_eq(queue.failure, None);
        }
        self.disks[0]
            .stop()
            .map_err(|_| TestError("first shared action removal cannot safely stop"))?;
        let stopped_calls = self.disks[0].test_snapshot().irq_calls;
        context.expect_eq(self.disks[0].irq.lock().is_none(), true);
        context.expect_eq(
            irq::test_action_count(self.sources[0]),
            self.actions[0]
                + if self.sources[0] == self.sources[1] {
                    1
                } else {
                    0
                },
        );
        let queue_irqs = self.disks[1].test_snapshot().irq_queue;
        let neighbor_dispatch = irq::testing::dispatch_count(self.sources[1]);
        disable(&self.disks[1]);
        self.uart.begin(b'J')?;
        self.submit(3, 1);
        until(
            "remaining hardware used ring and UART FIFO did not become pending",
            || {
                super::tests::used_index(&self.disks[1], self.legacy[1]) == 3
                    && crate::console::receive_pending()
            },
        )?;
        self.uart.enable();
        enable(&self.disks[1]);
        self.requests[3].reclaim(context)?;
        self.uart.receive(context, b'J')?;
        context.expect_eq(queue_irqs < self.disks[1].test_snapshot().irq_queue, true);
        context.expect_eq(
            neighbor_dispatch < irq::testing::dispatch_count(self.sources[1]),
            true,
        );
        context.expect_eq(self.disks[0].test_snapshot().irq_calls, stopped_calls);
        context.expect_eq(self.disks[1].test_snapshot().completed, 3);
        context.diagnostic(format_args!("hardware-irq identities={:?} sources={}/{} uart-source={} shared-intx={} shared-empty-isr-checked={} actual-disk-snapshots={:?}/{:?} read-only=1 root-business-identity=not-covered", self.identities, self.sources[0].source(), self.sources[1].source(), uart_source.source(), (self.sources[0] == self.sources[1]) as usize, (self.sources[0] == self.sources[1]) as usize, self.disks[0].test_snapshot(), self.disks[1].test_snapshot()));
        Ok(())
    }
}

#[::ktest::suite(fixture = MultiIrq, isolation = "Case", tags("block", "irq", "B02"), resources("root", "extra", "uart"), timeout_ms = 30000)]
mod hardware {
    use super::*;
    /// RV 多 PLIC source、LA 共享真实 INTx；摘一块设备 action 后另盘及 UART 继续真实 IRQ。
    #[test]
    fn uart_two_devices_unregister_neighbor(
        context: &mut TestContext<'_>,
        fixture: &mut MultiIrq,
    ) -> TestResult {
        fixture.exercise(context)
    }
}

/// 返回实际硬件 suite 的注册槽，供父 block 模块保留。
pub(crate) fn anchors() -> [&'static SuiteSlot; 1] {
    [hardware::__ktest_anchor()]
}
