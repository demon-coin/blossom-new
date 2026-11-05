//! 每例重新准备真实块队列；受控父账本明确不充当硬件反序或真实 I/O 成功。

use ::ktest::{self, Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use virtio_drivers::{BufferDirection, MappedSegment};

use super::bio::BioInner;
use super::disk::{QueuePhase, SlotState};
use super::{
    AccessMode, Bio, BioAllocator, BioCompletion, BlkOperation, BlockError, Disk, DiskRef, EndIo,
    FsClaim, IoResult,
};
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::DeviceInventory;
use crate::mm::dma::MappedPages;
use crate::sync::{Completion, SpinLock};
use crate::task::{self, JoinHandle, PreparedTask, State, Task};
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, mm, sched, time, timer};

// EndIo 只有函数指针；唯一有界定位槽强拥有当前 fixture 的观察者，不保存跨例 gate。
static CALLBACK_OWNER: SpinLock<Option<Arc<Observation>>> = SpinLock::new(None);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    Multi,
    Tag,
    Descriptor,
    Rearm,
    Split,
    Merge,
    Notify,
    Tail,
    Callback,
    Scratch,
    FsScratch,
    ReadonlyDevice,
    Entry,
    Identity,
    Exact,
    Model,
    Progress,
    ProgressBatch,
    Generation,
    Stopped,
}

struct Observation {
    gate: Completion,
    tasks: SpinLock<Vec<Option<Arc<Task>>>>,
    callback_task: SpinLock<Option<Arc<Task>>>,
    callback_result: SpinLock<Option<IoResult>>,
    callback_valid: AtomicBool,
    callback_calls: AtomicUsize,
    callback_tail: AtomicBool,
    peer: SpinLock<Option<BioCompletion>>,
}

impl Observation {
    /// 观察槽、gate 与 Task 强拥有全部在设备发表以前 fallible 准备。
    fn prepare(task_count: usize) -> Result<Arc<Self>, TestError> {
        let mut tasks = Vec::new();
        tasks
            .try_reserve_exact(task_count)
            .map_err(|_| TestError("queue Task observer allocation failed"))?;
        tasks.resize_with(task_count, || None);
        Arc::try_new(Self {
            gate: Completion::new(),
            tasks: SpinLock::new(tasks),
            callback_task: SpinLock::new(None),
            callback_result: SpinLock::new(None),
            callback_valid: AtomicBool::new(false),
            callback_calls: AtomicUsize::new(0),
            callback_tail: AtomicBool::new(false),
            peer: SpinLock::new(None),
        })
        .map_err(|_| TestError("queue observation allocation failed"))
    }

    /// 锁外向生产调度器读取真实 Blocked，不以 entered 或 sleep 推断阻塞。
    fn blocked(&self, ordinal: usize) -> bool {
        let observed = self.tasks.lock()[ordinal].clone();
        observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 回调的真实执行者必须是预建 Work 的 Task，且此刻仍停在合法等待源。
    fn callback_blocked(&self) -> bool {
        let observed = self.callback_task.lock().clone();
        observed
            .is_some_and(|task| task.worker.is_some() && sched::test_state(&task) == State::Blocked)
    }
}

/// 回调借已收权原页，合法等待外部 gate 或独立父；不等待自身完成或本盘 stop。
fn callback(result: &IoResult, memory: &[MappedPages]) {
    let owner = CALLBACK_OWNER
        .lock()
        .as_ref()
        .expect("queue callback fixture owner")
        .clone();
    owner.callback_calls.fetch_add(1, Ordering::AcqRel);
    owner.callback_valid.store(
        result.error.is_none()
            && result.completed == memory.iter().map(MappedPages::len).sum::<usize>()
            && memory.iter().all(|mapped| !mapped.is_device_owned()),
        Ordering::Release,
    );
    *owner.callback_result.lock() = Some(*result);
    *owner.callback_task.lock() = Some(task::current_task());
    let peer = owner.peer.lock().clone();
    if let Some(peer) = peer {
        let peer_result = peer.wait();
        if peer_result != *result {
            owner.callback_valid.store(false, Ordering::Release);
        }
    } else {
        owner.gate.wait();
    }
    owner.callback_tail.store(true, Ordering::Release);
}

/// 有界单调期限只等待实际谓词；不用于证明生产等待实现。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("queue observation deadline overflow"))?;
    while !ready() {
        if time::now() >= deadline {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 所有参与者离栈后整理真实缓存，保存完整六域资源账本。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("queue resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 每例借 suite 一次发现库存取得独占设备，上一例已真实 stop/join。
fn devices(inventory: &DeviceInventory, count: usize) -> Result<Vec<VirtioDevice>, TestError> {
    let pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    if inventory.mmio_blocks.len() + pci.clone().count() != count {
        return Err(TestError(
            "queue fixture attached device count differs from declared resources",
        ));
    }

    let mut prepared = Vec::new();
    prepared
        .try_reserve_exact(count)
        .map_err(|_| TestError("queue inventory owner allocation failed"))?;
    for candidate in &inventory.mmio_blocks {
        // SAFETY: host 只挂本例声明的工作副本；本次真实库存和永久映射有效，fixture 唯一拥有。
        prepared.push(
            unsafe { VirtioDevice::prepare_mmio(candidate) }
                .map_err(|_| TestError("queue MMIO device preparation failed"))?,
        );
    }
    for device in pci {
        // SAFETY: BDF 来自同一真实库存，唯一 fixture 保活至 IRQ/Work 尾部和真 reset。
        prepared.push(
            unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
                .map_err(|_| TestError("queue PCI device preparation failed"))?,
        );
    }
    Ok(prepared)
}

struct Request {
    pending: Option<Bio>,
    completion: BioCompletion,
    originals: Vec<MappedSegment>,
    sector: u64,
    bytes: usize,
}

impl Request {
    /// 每片使用独立 order0 原页；保存 DMA 身份但不复制 payload，也不留 CPU 裸别名。
    fn prepare(
        disk: &Disk,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
        first_offset: usize,
        end_io: Option<EndIo>,
    ) -> Result<Self, TestError> {
        let count = (bytes + first_offset).div_ceil(mm::PAGE_SIZE);
        let mut memory = Vec::new_in(BioAllocator::Heap);
        let mut originals = Vec::new();
        memory
            .try_reserve_exact(count)
            .map_err(|_| TestError("queue original owners allocation failed"))?;
        originals
            .try_reserve_exact(count)
            .map_err(|_| TestError("queue original identities allocation failed"))?;
        let mut remaining = bytes;
        let mut offset = first_offset;
        let mut logical = 0;
        while remaining != 0 {
            let length = remaining.min(mm::PAGE_SIZE - offset);
            let page = mm::alloc_pages(0, true)
                .ok_or(TestError("queue original page allocation failed"))?;
            let direction = if operation == BlkOperation::Write {
                BufferDirection::DriverToDevice
            } else {
                BufferDirection::DeviceToDriver
            };
            let mut mapped = MappedPages::new(page, offset, length, direction)
                .map_err(|_| TestError("queue original mapping failed"))?;
            if operation == BlkOperation::Write {
                for byte in mapped.bytes_mut() {
                    byte.write((logical as u8).wrapping_mul(13).wrapping_add(71));
                    logical += 1;
                }
            }
            originals.push(
                mapped
                    .segment(crate::mm::dma::DmaDomain::qemu_coherent())
                    .expect("QEMU fixture mapping must be reachable"),
            );
            memory.push(mapped);
            remaining -= length;
            offset = 0;
        }

        // SAFETY: 原页独占、Write 完整初始化、构造后没有 CPU 裸别名；Task 回调只借已收权页，
        // 等独立父或外部 gate，不等待自身 stop。模型用例从不将其地址发表到设备。
        let bio = unsafe { Bio::try_new(disk, operation, sector, memory, 0, end_io) }
            .map_err(|_| TestError("queue Bio preparation failed"))?;
        let completion = bio.completion();
        Ok(Self {
            pending: Some(bio),
            completion,
            originals,
            sector,
            bytes,
        })
    }

    /// 所有原页按原顺序且只领取一次；成功读逐字节核完整非零固定图样。
    fn reclaim(
        &self,
        context: &mut TestContext<'_>,
        expected: IoResult,
        tag: Option<u8>,
    ) -> TestResult {
        until("queue parent completion did not progress", || {
            self.completion.is_complete()
        })?;
        context.expect_eq(self.completion.wait(), expected);
        let memory = self
            .completion
            .take_memory()
            .map_err(|_| TestError("queue original memory not reclaimable"))?;
        context.expect_eq(memory.len(), self.originals.len());
        let mut logical = 0;
        for (mapped, original) in memory.iter().zip(&self.originals) {
            let actual = mapped
                .segment(crate::mm::dma::DmaDomain::qemu_coherent())
                .expect("QEMU fixture mapping must be reachable");
            context.expect_eq(
                (actual.address, actual.length, actual.direction),
                (original.address, original.length, original.direction),
            );
            context.expect_eq(mapped.is_device_owned(), false);
            if let Some(tag) = tag {
                // SAFETY: 原页预先零填，完整成功 Read 已结束全部 child 并收权；只借唯一 CPU owner。
                for byte in unsafe { mapped.bytes() } {
                    let at = self.sector * 512 + logical as u64;
                    let wanted = tag
                        .wrapping_mul(29)
                        .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                        .wrapping_add((at % 512) as u8);
                    if *byte != wanted {
                        return Err(TestError("queue wrong disk/sector/original SG byte"));
                    }
                    logical += 1;
                }
            }
        }
        if tag.is_some() {
            context.expect_eq(logical, self.bytes);
        }
        context.expect_eq(self.completion.take_memory().err(), Some(BlockError::Busy));
        drop(memory);
        Ok(())
    }

    /// 此父完整成功后的常用结果，模型错误不能通过此助手伪装成功读。
    fn success(&self, context: &mut TestContext<'_>, tag: Option<u8>) -> TestResult {
        self.reclaim(
            context,
            IoResult {
                completed: self.bytes,
                error: None,
            },
            tag,
        )
    }
}

struct Owner {
    scenario: Scenario,
    baseline: mm::MemoryStats,
    disk: DiskRef,
    extra: Option<DiskRef>,
    queue: WorkQueue,
    requests: Vec<Request>,
    batch: Vec<Bio>,
    prepared: Vec<PreparedTask>,
    joins: Vec<JoinHandle>,
    observation: Arc<Observation>,
    activation_error: Option<BlockError>,
    legacy: bool,
    claim: Option<FsClaim>,
    fill: usize,
    model_live: Option<usize>,
    wb_folios: Vec<mm::FolioRef>,
    wb_active: Option<Bio>,
}

impl Owner {
    /// 借 suite 库存固定真实 limits；原页、gate 和 producer 栈在 start 前 fallible 准备。
    fn prepare(
        scenario: Scenario,
        context: &mut TestContext<'_>,
        resources: &DeviceInventory,
    ) -> Result<Self, TestError> {
        let baseline = settled()?;
        let mut inventory = devices(
            resources,
            if scenario == Scenario::Identity { 2 } else { 1 },
        )?;
        let device = inventory.remove(0);
        let info = device.info();
        let unit = info.logical_block_size as usize;
        if unit > mm::PAGE_SIZE || !matches!(unit, 512 | 1024 | 2048 | 4096) {
            return Err(TestError(
                "queue logical block exceeds legacy real fixture support",
            ));
        }
        if scenario == Scenario::Scratch && (info.readonly || !info.flush) {
            return Err(TestError(
                "scratch case requires writable device with negotiated FLUSH",
            ));
        }
        if scenario == Scenario::FsScratch && info.readonly {
            return Err(TestError(
                "FS write case requires a writable scratch device",
            ));
        }
        if scenario == Scenario::ReadonlyDevice && !info.readonly {
            return Err(TestError(
                "readonly device case requires --read-only scratch",
            ));
        }

        let legacy = matches!(device.identity(), VirtioIdentity::Mmio { version: 1, .. });
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(info.tag_count) + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("queue completion domain preparation failed"))?;
        let prepared_disk = match scenario {
            Scenario::Descriptor => Disk::test_prepare_with_limits(
                device,
                &queue,
                4 * mm::PAGE_SIZE,
                usize::from(info.max_segments),
                mm::PAGE_SIZE as u32,
                u64::MAX,
            ),
            Scenario::Split => Disk::test_prepare_with_limits(
                device,
                &queue,
                mm::PAGE_SIZE,
                2,
                (unit * 3 / 2) as u32,
                (unit * 4 - 1) as u64,
            ),
            Scenario::Tail => Disk::test_prepare_with_limits(
                device,
                &queue,
                unit * 2,
                1,
                unit as u32,
                (unit - 1) as u64,
            ),
            _ => Disk::prepare(device, &queue),
        };
        let disk = match prepared_disk {
            Ok(disk) => disk,
            Err(_) => {
                queue.close_sync();
                return Err(TestError("queue fixed request preparation failed"));
            }
        };

        let resources = (|| {
            let extra = if let Some(device) = inventory.pop() {
                Some(
                    Disk::prepare(device, &queue)
                        .map_err(|_| TestError("extra identity Disk preparation failed"))?,
                )
            } else {
                None
            };
            let result = (|| {
                let task_count = match scenario {
                    Scenario::Multi => 8,
                    Scenario::Tag | Scenario::Descriptor | Scenario::Generation => 1,
                    _ => 0,
                };
                let observation = Observation::prepare(task_count)?;
                let mut requests = Vec::new();
                let mut batch = Vec::new();
                let mut prepared = Vec::new();
                let mut joins = Vec::new();
                let fill = match scenario {
                    Scenario::Tag => usize::from(info.tag_count),
                    Scenario::Descriptor => usize::from(info.queue_depth) / 6,
                    Scenario::Rearm => usize::from(info.tag_count),
                    _ => 0,
                };
                if scenario == Scenario::Descriptor
                    && (info.max_segments < 4
                        || info.max_segment_bytes < mm::PAGE_SIZE as u32
                        || fill >= usize::from(info.tag_count))
                {
                    return Err(TestError(
                        "descriptor fixture cannot exhaust descriptors before tags",
                    ));
                }
                if scenario == Scenario::Rearm && fill <= super::disk::IRQ_BUDGET {
                    return Err(TestError(
                        "available real tags cannot exceed the actual production used budget",
                    ));
                }

                let count = match scenario {
                    Scenario::Multi => 8,
                    Scenario::Tag | Scenario::Descriptor => fill + 1,
                    Scenario::Rearm => fill,
                    Scenario::Merge | Scenario::Notify => 3,
                    Scenario::Tail
                    | Scenario::Progress
                    | Scenario::ProgressBatch
                    | Scenario::Generation => 2,
                    Scenario::Scratch => 4,
                    Scenario::FsScratch => {
                        if info.flush {
                            6
                        } else {
                            5
                        }
                    }
                    Scenario::Exact => 0,
                    _ => 1,
                };
                requests
                    .try_reserve_exact(count)
                    .map_err(|_| TestError("queue request observer allocation failed"))?;
                if matches!(
                    scenario,
                    Scenario::Merge | Scenario::Notify | Scenario::Tail | Scenario::ProgressBatch
                ) {
                    batch
                        .try_reserve_exact(count)
                        .map_err(|_| TestError("queue explicit batch allocation failed"))?;
                }
                prepared
                    .try_reserve_exact(task_count)
                    .map_err(|_| TestError("queue prepared producer allocation failed"))?;
                joins
                    .try_reserve_exact(task_count)
                    .map_err(|_| TestError("queue producer join allocation failed"))?;

                for ordinal in 0..count {
                    let (operation, sector, bytes, offset) = match scenario {
                        Scenario::Multi => (
                            BlkOperation::Read,
                            64 + ordinal as u64 * 32,
                            2 * mm::PAGE_SIZE,
                            0,
                        ),
                        Scenario::Tag | Scenario::Descriptor | Scenario::Rearm => (
                            BlkOperation::Read,
                            if ordinal == fill {
                                1536
                            } else {
                                512 + ordinal as u64 * 40
                            },
                            if scenario == Scenario::Descriptor {
                                4 * mm::PAGE_SIZE
                            } else {
                                unit
                            },
                            0,
                        ),
                        Scenario::Split => (BlkOperation::Read, 320, 3 * mm::PAGE_SIZE, 0),
                        Scenario::Merge => (
                            BlkOperation::Read,
                            256 + ordinal as u64 * unit as u64 / 512,
                            unit,
                            0,
                        ),
                        Scenario::Notify => {
                            (BlkOperation::Read, 256 + ordinal as u64 * 32, unit, 0)
                        }
                        Scenario::Tail => (
                            BlkOperation::Read,
                            448 + ordinal as u64 * unit as u64 / 512,
                            unit,
                            if ordinal == 1 { unit / 2 } else { 0 },
                        ),
                        Scenario::Scratch => (
                            match ordinal {
                                0..=1 => BlkOperation::Write,
                                2 => BlkOperation::Read,
                                _ => BlkOperation::Flush,
                            },
                            if ordinal == 3 { 0 } else { 2048 },
                            if ordinal == 3 { 0 } else { unit },
                            0,
                        ),
                        Scenario::FsScratch => (
                            match ordinal {
                                0..=3 => BlkOperation::Write,
                                4 => BlkOperation::Read,
                                _ => BlkOperation::Flush,
                            },
                            if ordinal == 5 { 0 } else { 2048 },
                            if ordinal == 5 { 0 } else { unit },
                            0,
                        ),
                        Scenario::Model => (
                            BlkOperation::Read,
                            32,
                            unit * match context.case.parameter {
                                Some("reverse_short_prefix" | "minimum_error_prefix") => 3,
                                Some("generation_failure_prefix") => 2,
                                Some("synchronous_child_producer_hold") => 1,
                                Some("wb_inline" | "wb_reverse" | "wb_rejected_tail") => 1,
                                _ => return Err(TestError("unknown controlled ledger parameter")),
                            },
                            0,
                        ),
                        Scenario::Progress | Scenario::ProgressBatch | Scenario::Generation => (
                            BlkOperation::Read,
                            if ordinal == 0 {
                                400
                            } else if scenario == Scenario::ProgressBatch {
                                400 + unit as u64 / 512
                            } else {
                                416
                            },
                            unit,
                            0,
                        ),
                        _ => (BlkOperation::Read, 384, unit, 0),
                    };
                    let end_io = if (ordinal == 0
                        && matches!(
                            scenario,
                            Scenario::Callback
                                | Scenario::Progress
                                | Scenario::ProgressBatch
                                | Scenario::Generation
                        ))
                        || (scenario == Scenario::FsScratch && ordinal == 3)
                    {
                        Some(EndIo::Task(callback))
                    } else {
                        None
                    };
                    requests.push(Request::prepare(
                        &disk, operation, sector, bytes, offset, end_io,
                    )?);
                }

                if matches!(
                    scenario,
                    Scenario::Progress | Scenario::ProgressBatch | Scenario::Generation
                ) {
                    *observation.peer.lock() = Some(requests[1].completion.clone());
                }
                if matches!(
                    scenario,
                    Scenario::Multi | Scenario::Tag | Scenario::Descriptor | Scenario::Generation
                ) {
                    let producers = if scenario == Scenario::Multi { 8 } else { 1 };
                    for ordinal in 0..producers {
                        let index = if scenario == Scenario::Multi {
                            ordinal
                        } else if scenario == Scenario::Generation {
                            0
                        } else {
                            fill
                        };
                        let first = requests[index]
                            .pending
                            .take()
                            .expect("prepared producer Bio");
                        let second = if scenario == Scenario::Generation {
                            requests[1].pending.take()
                        } else {
                            None
                        };
                        let completion = requests[index].completion.clone();
                        let bytes = requests[index].bytes;
                        let producer_disk = disk.clone();
                        let observer = observation.clone();
                        prepared.push(
                            task::prepare_kernel_on(ordinal % cpu::count(), move || {
                                observer.tasks.lock()[ordinal] = Some(task::current_task());
                                if let Some(second) = second {
                                    super::tests::generation_pair(&producer_disk, first, second);
                                } else {
                                    producer_disk.submit_bio(first);
                                }
                                if completion.wait()
                                    == (IoResult {
                                        completed: bytes,
                                        error: None,
                                    })
                                {
                                    0
                                } else {
                                    1
                                }
                            })
                            .map_err(|_| {
                                TestError("queue actual producer Task preparation failed")
                            })?,
                        );
                    }
                }
                Ok((observation, requests, batch, prepared, joins, fill))
            })();
            match result {
                Ok(resources) => Ok((extra, resources)),
                Err(error) => {
                    if let Some(extra) = extra {
                        extra
                            .stop()
                            .map_err(|_| TestError("unpublished extra stop failed"))?;
                    }
                    Err(error)
                }
            }
        })();
        let (extra, (observation, requests, batch, prepared, joins, fill)) = match resources {
            Ok(resources) => resources,
            Err(error) => {
                disk.stop()
                    .map_err(|_| TestError("unpublished queue stop failed"))?;
                drop(disk);
                queue.close_sync();
                return Err(error);
            }
        };

        if matches!(
            scenario,
            Scenario::Callback
                | Scenario::Progress
                | Scenario::ProgressBatch
                | Scenario::Generation
                | Scenario::FsScratch
        ) {
            let mut slot = CALLBACK_OWNER.lock();
            assert!(slot.is_none(), "previous queue callback owner remains");
            *slot = Some(observation.clone());
        }

        let mut activation_error = disk.start().err();
        if let Some(extra) = &extra {
            if let Err(error) = extra.start() {
                activation_error.get_or_insert(error);
            }
        }
        if let Some(error) = activation_error {
            context.diagnostic(format_args!(
                "queue activation failed: {:?}; retaining typed owner",
                error
            ));
        }
        Ok(Self {
            scenario,
            baseline,
            disk,
            extra,
            queue,
            requests,
            batch,
            prepared,
            joins,
            observation,
            activation_error,
            legacy,
            fill,
            claim: None,
            model_live: None,
            wb_folios: Vec::new(),
            wb_active: None,
        })
    }

    /// 激活错误仍由成功构造的 fixture 保活到真实 quiesce，不 Drop 活设备。
    fn ready(&self) -> TestResult {
        if self.activation_error.is_some() {
            Err(TestError("queue activation failed"))
        } else {
            Ok(())
        }
    }

    /// 每个新 fixture 自己读固定盘身份，不继承上一例 pattern tag。
    fn tag(&self) -> Result<u8, TestError> {
        let unit = self.disk.limits().logical_block_size as usize;
        let mut bytes = [0_u8; mm::PAGE_SIZE];
        self.disk
            .read_exact_at(0, &mut bytes[..unit])
            .map_err(|_| TestError("queue real fixture identity read failed"))?;
        match bytes[0] {
            29 => Ok(1),
            58 => Ok(2),
            _ => Err(TestError("queue root requires fixed raw pattern, not ext4")),
        }
    }

    /// 从真实 ring 锁外观察到设备 used.idx 已写目标；不消费或合成 used。
    fn used(&self, target: usize) -> TestResult {
        until("queue real used index did not progress", || {
            super::tests::used_index(&self.disk, self.legacy) == target as u16
        })
    }

    /// 锁内仅 volatile 读真实 avail.flags，fixture 保有控制页且不与 stop 竞争。
    fn interrupt_flags(&self) -> u16 {
        let q = self.disk.queue.lock();
        let device = q.device.as_ref().expect("live queue ring owner");
        let ring = device.test_virtio().control_regions().0;
        let flags = usize::from(self.disk.limits().queue_depth) * 16;
        crate::arch::dma_read_barrier();
        // SAFETY: 同一 queue 锁保控制区；avail.flags 位于 descriptor 表后对齐 u16，仅 volatile 读取。
        u16::from_le(unsafe {
            core::ptr::read_volatile(ring.cpu.as_ptr().add(flags).cast::<u16>())
        })
    }

    /// 发表全部预建 producer，闭包没有 borrowed context 或 fixture 引用。
    fn publish_tasks(&mut self) {
        for producer in self.prepared.drain(..) {
            self.joins.push(producer.publish());
        }
    }

    /// 等每个生产者真的离栈后消费原 JoinHandle；不以完成 flag 代替 join。
    fn join_tasks(&mut self) -> TestResult {
        for join in &self.joins {
            until("queue producer did not leave its stack", || {
                join.try_join().is_some()
            })?;
        }
        let mut failed = false;
        for join in self.joins.drain(..) {
            failed |= join.join() != 0;
        }
        if failed {
            Err(TestError("queue producer returned failure"))
        } else {
            Ok(())
        }
    }

    /// 只消耗本例预建 Bio，不创建另一个 I/O 引擎或调用旧 run。
    fn submit(&mut self, index: usize) {
        self.disk.submit_bio(
            self.requests[index]
                .pending
                .take()
                .expect("queue Bio submitted twice"),
        );
    }

    /// 显式 batch 的原 Vec 在 setup reserve，一次消费并保持每父独立原页。
    fn submit_batch(&mut self) {
        for request in &mut self.requests {
            self.batch.push(
                request
                    .pending
                    .take()
                    .expect("queue batch Bio submitted twice"),
            );
        }
        self.disk.submit_batch(core::mem::take(&mut self.batch));
    }

    /// 真设备页和原回调尾部全结束，再 flush 原 Work 票。
    fn drained(&self) -> TestResult {
        until("queue slots did not return", || {
            self.disk.test_snapshot().inflight == 0
        })?;
        self.disk.test_flush_completions();
        Ok(())
    }

    /// 八个不同 Task 同时保留两页真实在途请求，逐父核完整 sector 图样及原页 once。
    fn multi(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = self.tag()?;
        let before = self.disk.test_snapshot();
        self.disk.test_suspend_completions();
        self.publish_tasks();
        until("eight real producers did not publish", || {
            self.disk.test_snapshot().submitted == before.submitted + 8
        })?;
        context.expect_eq(self.disk.test_snapshot().inflight, 8);
        until("eight producer Task identities unavailable", || {
            self.observation.tasks.lock().iter().all(Option::is_some)
        })?;
        let tasks = self.observation.tasks.lock();
        for a in 0..8 {
            for b in 0..a {
                context.expect_eq(
                    Arc::ptr_eq(tasks[a].as_ref().unwrap(), tasks[b].as_ref().unwrap()),
                    false,
                );
            }
        }
        drop(tasks);
        context.expect_eq(
            self.joins.iter().all(|join| join.try_join().is_none()),
            true,
        );
        for request in &self.requests {
            context.expect_eq(request.completion.is_complete(), false);
        }

        self.disk.test_resume_completions();
        for request in &self.requests {
            request.success(context, Some(tag))?;
        }
        self.join_tasks()?;
        self.drained()?;
        context.expect_eq(self.disk.test_snapshot().peak_inflight > 1, true);
        context.expect_eq(self.disk.test_snapshot().completed - before.completed, 8);
        Ok(())
    }

    /// 真 tag 或 descriptor 额度耗尽后 kick，额外 submitter 必须在生产 State::Blocked 并真实唤醒。
    fn pressure(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = self.tag()?;
        let before = self.disk.test_snapshot();
        self.disk.test_suspend_completions();
        for index in 0..self.fill {
            self.submit(index);
        }
        context.expect_eq(
            self.disk.test_snapshot().submitted - before.submitted,
            self.fill,
        );
        context.expect_eq(self.disk.test_snapshot().inflight, self.fill);

        self.publish_tasks();
        until("extra producer never entered real resource Blocked", || {
            self.observation.blocked(0)
                && self.disk.test_snapshot().resource_parks > before.resource_parks
        })?;
        let q = self.disk.queue.lock();
        let free = q
            .slot_states
            .iter()
            .filter(|state| **state == SlotState::Free)
            .count();
        let preparing = q
            .slot_states
            .iter()
            .filter(|state| **state == SlotState::Preparing)
            .count();
        let descriptors = q
            .device
            .as_ref()
            .expect("blocked live device")
            .test_virtio()
            .driver()
            .available_descriptors();
        if self.scenario == Scenario::Tag {
            context.expect_eq((free, preparing, q.active_slots), (0, 0, self.fill));
            context.expect_eq(descriptors >= 3, true);
        } else {
            context.expect_eq(preparing, 1);
            context.expect_eq(q.active_slots, self.fill + 1);
            context.expect_eq(free + preparing > 0, true);
            context.expect_eq(descriptors < 6, true);
        }
        drop(q);
        context.expect_eq(self.joins[0].try_join().is_none(), true);
        context.expect_eq(
            self.disk.test_snapshot().notifications > before.notifications,
            true,
        );
        self.used(before.submitted + self.fill)?;

        self.disk.test_resume_completions();
        for request in &self.requests {
            request.success(context, Some(tag))?;
        }
        self.join_tasks()?;
        self.drained()?;
        let after = self.disk.test_snapshot();
        context.expect_eq(after.submitted - before.submitted, self.fill + 1);
        context.expect_eq(after.completed - before.completed, self.fill + 1);
        if self.scenario == Scenario::Tag {
            context.expect_eq(
                after.budget_continuations > before.budget_continuations,
                true,
            );
        }
        Ok(())
    }

    /// NO_INTERRUPT 下先取得超过生产实际预算的真实 used，再重装复查和续取，不等待新 IRQ。
    fn rearm(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = self.tag()?;
        let before = self.disk.test_snapshot();
        self.disk.test_suspend_completions();
        context.expect_eq(self.interrupt_flags() & 1, 1);
        for index in 0..self.requests.len() {
            self.submit(index);
        }
        self.used(before.submitted + self.fill)?;
        context.expect_eq(self.disk.test_snapshot().completed, before.completed);

        self.disk.test_resume_completions();
        for request in &self.requests {
            request.success(context, Some(tag))?;
        }
        self.drained()?;
        context.expect_eq(self.interrupt_flags() & 1, 0);
        let after = self.disk.test_snapshot();
        context.expect_eq(after.completed - before.completed, self.fill);
        context.expect_eq(
            after.budget_continuations > before.budget_continuations,
            true,
        );
        context.diagnostic(format_args!(
            "flags-rearm requests={} used-budget={} budget-continuations={} all-originals-reclaimed=1",
            after.submitted - before.submitted, super::disk::IRQ_BUDGET,
            after.budget_continuations - before.budget_continuations,
        ));
        Ok(())
    }

    /// 多原页 SG 按一次实际限额拆分，真实 child 全结束以前父不能 ready 或归页。
    fn split(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = self.tag()?;
        let before = self.disk.test_snapshot();
        self.disk.test_suspend_completions();
        self.submit(0);
        let generated = self.disk.test_snapshot();
        context.expect_eq(generated.submitted - before.submitted > 1, true);
        context.expect_eq(generated.splits > before.splits, true);
        context.expect_eq(self.requests[0].completion.is_complete(), false);
        context.expect_eq(
            self.requests[0].completion.take_memory().err(),
            Some(BlockError::Busy),
        );
        self.used(generated.submitted)?;

        self.disk.test_resume_completions();
        self.requests[0].success(context, Some(tag))?;
        self.drained()
    }

    /// 三父 merge/非相邻 notify/非法尾部各有独立 fixture，不能靠上一例累积事件。
    fn batch(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = self.tag()?;
        let before = self.disk.test_snapshot();
        self.submit_batch();
        for (index, request) in self.requests.iter().enumerate() {
            if self.scenario == Scenario::Tail && index == 1 {
                request.reclaim(
                    context,
                    IoResult {
                        completed: 0,
                        error: Some(BlockError::Unsupported),
                    },
                    None,
                )?;
            } else {
                request.success(context, Some(tag))?;
            }
        }
        self.drained()?;
        let after = self.disk.test_snapshot();
        context.expect_eq(
            after.submitted - before.submitted,
            if self.scenario == Scenario::Notify {
                3
            } else {
                1
            },
        );
        if self.scenario == Scenario::Merge {
            context.expect_eq(after.merges - before.merges, 2);
        }
        context.expect_eq(after.notifications - before.notifications, 1);
        Ok(())
    }

    /// 回调真 Blocked 时槽、父和 memory 都不可提前归还；外部门释放后一次原尾部完成。
    fn callback_tail(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = self.tag()?;
        let before = self.disk.test_snapshot();
        self.disk.test_suspend_completions();
        self.submit(0);
        context.expect_eq(self.requests[0].completion.is_complete(), false);
        self.disk.test_resume_completions();
        until("Task callback never entered real Blocked", || {
            self.observation.callback_blocked()
        })?;
        context.expect_eq(self.requests[0].completion.is_complete(), false);
        context.expect_eq(
            self.requests[0].completion.take_memory().err(),
            Some(BlockError::Busy),
        );
        context.expect_eq(self.disk.test_snapshot().inflight, 1);
        context.expect_eq(self.disk.test_snapshot().completed, before.completed);

        self.observation.gate.complete();
        self.requests[0].success(context, Some(tag))?;
        self.drained()?;
        self.check_callback(context);
        Ok(())
    }

    /// 两个真实 used 的独立/显式 batch peer 回调不阻断第二父；generation 只测未发表账本。
    fn peer(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = if self.scenario == Scenario::Generation {
            None
        } else {
            Some(self.tag()?)
        };
        let before = self.disk.test_snapshot();
        if self.scenario == Scenario::Generation {
            self.publish_tasks();
        } else {
            self.disk.test_suspend_completions();
            if self.scenario == Scenario::ProgressBatch {
                self.submit_batch();
            } else {
                self.submit(0);
                self.submit(1);
            }
            context.expect_eq(self.disk.test_snapshot().submitted - before.submitted, 2);
            self.used(before.submitted + 2)?;
            self.disk.test_resume_via_progress();
        }

        for request in &self.requests {
            request.success(context, tag)?;
        }
        self.join_tasks()?;
        self.drained()?;
        if self.scenario == Scenario::Generation {
            context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);
            context.diagnostic(format_args!(
                "generation-peer scope=controlled-ledger addresses-not-published=1"
            ));
        }
        self.check_callback(context);
        Ok(())
    }

    /// 真实原 Work Task 与合法借页回调各一次，尾部可见后才允许清理定位槽。
    fn check_callback(&self, context: &mut TestContext<'_>) {
        context.expect_eq(self.observation.callback_calls.load(Ordering::Acquire), 1);
        context.expect_eq(
            self.observation.callback_valid.load(Ordering::Acquire),
            true,
        );
        context.expect_eq(self.observation.callback_tail.load(Ordering::Acquire), true);
        context.expect_eq(
            self.observation
                .callback_task
                .lock()
                .as_ref()
                .is_some_and(|task| task.worker.is_some()),
            true,
        );
        context.expect_eq(
            *self.observation.callback_result.lock(),
            Some(IoResult {
                completed: self.requests[0].bytes,
                error: None,
            }),
        );
    }

    /// 独占 scratch 上 claim 拒写和 Busy stop，释放后实际 Write/Read-back/FLUSH，不写挂载盘。
    fn scratch(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let claim = Disk::claim_fs(
            &crate::block::BlockView::whole(self.disk.clone()),
            AccessMode::ReadOnly,
        )
        .map_err(|_| TestError("scratch RO FS claim failed"))?;
        let before = self.disk.test_snapshot();
        context.expect_eq(self.disk.stop(), Err(BlockError::Busy));
        self.submit(0);
        self.requests[0].reclaim(
            context,
            IoResult {
                completed: 0,
                error: Some(BlockError::ReadOnly),
            },
            None,
        )?;
        context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);
        drop(claim);

        self.submit(1);
        self.requests[1].success(context, None)?;
        self.submit(2);
        self.readback(2, context)?;
        self.submit(3);
        self.requests[3].success(context, None)?;
        self.drained()
    }

    /// 已提交 Read 的全长字节及原页身份逐一验证，拒绝只检查首字节的假读回。
    fn readback(&self, index: usize, context: &mut TestContext<'_>) -> TestResult {
        let request = &self.requests[index];
        until("scratch read-back did not complete", || {
            request.completion.is_complete()
        })?;
        context.expect_eq(
            request.completion.wait(),
            IoResult {
                completed: request.bytes,
                error: None,
            },
        );
        let memory = request
            .completion
            .take_memory()
            .map_err(|_| TestError("scratch read-back original memory unavailable"))?;
        let mut logical = 0;
        for (mapped, original) in memory.iter().zip(&request.originals) {
            let segment = mapped
                .segment(crate::mm::dma::DmaDomain::qemu_coherent())
                .expect("QEMU fixture mapping must be reachable");
            context.expect_eq(
                (segment.address, segment.length, segment.direction),
                (original.address, original.length, original.direction),
            );
            context.expect_eq(mapped.is_device_owned(), false);
            // SAFETY: 完整真实 Read-back 全部 child 已结束且原零页已收权，没有并发 CPU/设备写者。
            for byte in unsafe { mapped.bytes() } {
                if *byte != (logical as u8).wrapping_mul(13).wrapping_add(71) {
                    return Err(TestError("scratch Write/Read-back byte mismatch"));
                }
                logical += 1;
            }
        }
        context.expect_eq(logical, request.bytes);
        context.expect_eq(
            request.completion.take_memory().err(),
            Some(BlockError::Busy),
        );
        drop(memory);
        Ok(())
    }

    /// 合法 FS 写和 raw/RO 拒绝同用生产队列，克隆占用与真实回调尾部分开核验。
    fn fs_scratch(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let before = self.disk.test_snapshot();
        let ro = Disk::claim_fs(
            &crate::block::BlockView::whole(self.disk.clone()),
            AccessMode::ReadOnly,
        )
        .map_err(|_| TestError("scratch RO FS claim failed"))?;
        self.submit(0);
        ro.submit_bio(self.requests[1].pending.take().unwrap());
        for request in &self.requests[..2] {
            request.reclaim(
                context,
                IoResult {
                    completed: 0,
                    error: Some(BlockError::ReadOnly),
                },
                None,
            )?;
        }
        context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);
        drop(ro);

        self.claim = Some(
            Disk::claim_fs(
                &crate::block::BlockView::whole(self.disk.clone()),
                AccessMode::ReadWrite,
            )
            .map_err(|_| TestError("scratch RW FS claim failed"))?,
        );
        let clone = self.claim.as_ref().unwrap().clone();
        context.expect_eq(self.disk.queue.lock().mounts, 1);
        drop(clone);
        context.expect_eq(self.disk.queue.lock().mounts, 1);
        self.submit(2);
        self.requests[2].reclaim(
            context,
            IoResult {
                completed: 0,
                error: Some(BlockError::ReadOnly),
            },
            None,
        )?;
        context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);

        self.disk.test_suspend_completions();
        let used = super::tests::used_index(&self.disk, self.legacy);
        self.claim
            .as_ref()
            .unwrap()
            .submit_bio(self.requests[3].pending.take().unwrap());
        drop(self.claim.take());
        self.used(usize::from(used.wrapping_add(1)))?;
        context.expect_eq(self.disk.queue.lock().mounts, 1);
        context.expect_eq(self.disk.stop(), Err(BlockError::Busy));
        self.disk.test_resume_completions();
        until("FS write callback did not block on the real gate", || {
            self.observation.callback_blocked()
        })?;
        context.expect_eq(self.disk.queue.lock().mounts, 1);
        context.expect_eq(self.disk.test_snapshot().inflight, 1);
        context.expect_eq(self.requests[3].completion.is_complete(), false);
        context.expect_eq(
            self.requests[3].completion.take_memory().err(),
            Some(BlockError::Busy),
        );
        context.expect_eq(self.disk.stop(), Err(BlockError::Busy));

        self.observation.gate.complete();
        self.requests[3].success(context, None)?;
        self.drained()?;
        context.expect_eq(self.disk.queue.lock().mounts, 0);
        context.expect_eq(self.observation.callback_calls.load(Ordering::Acquire), 1);
        context.expect_eq(
            self.observation.callback_valid.load(Ordering::Acquire),
            true,
        );
        context.expect_eq(self.observation.callback_tail.load(Ordering::Acquire), true);

        self.claim = Some(
            Disk::claim_fs(
                &crate::block::BlockView::whole(self.disk.clone()),
                AccessMode::ReadWrite,
            )
            .map_err(|_| TestError("scratch RW FS readback claim failed"))?,
        );
        self.claim
            .as_ref()
            .unwrap()
            .submit_bio(self.requests[4].pending.take().unwrap());
        self.readback(4, context)?;
        if self.disk.limits().flush {
            self.claim
                .as_ref()
                .unwrap()
                .submit_bio(self.requests[5].pending.take().unwrap());
            self.requests[5].success(context, None)?;
        }
        self.drained()?;
        context.diagnostic(format_args!(
            "FS-claim scratch-sector=2048 bytes={} authorized-write-readback=1 negotiated-FLUSH={} clone-occupancy=1 used-and-callback-tail-retained=1 mount-view-covered=0 file-write-fsync-covered=0",
            self.requests[4].bytes, self.disk.limits().flush,
        ));
        drop(self.claim.take());
        Ok(())
    }

    /// 真实协商只读设备拒绝 RW 占用，RO 占用仍可完成实际 Read。
    fn readonly_device(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        context.expect_eq(self.disk.limits().readonly, true);
        context.expect_eq(
            Disk::claim_fs(
                &crate::block::BlockView::whole(self.disk.clone()),
                AccessMode::ReadWrite,
            )
            .err(),
            Some(BlockError::ReadOnly),
        );
        context.expect_eq(self.disk.queue.lock().mounts, 0);
        self.claim = Some(
            Disk::claim_fs(
                &crate::block::BlockView::whole(self.disk.clone()),
                AccessMode::ReadOnly,
            )
            .map_err(|_| TestError("readonly device RO FS claim failed"))?,
        );
        self.claim
            .as_ref()
            .unwrap()
            .submit_bio(self.requests[0].pending.take().unwrap());
        self.requests[0].success(context, None)?;
        self.drained()?;
        drop(self.claim.take());
        context.diagnostic(format_args!(
            "FS-claim negotiated-device-readonly=1 RW-rejected=1 RO-real-read=1"
        ));
        Ok(())
    }

    /// 原入参所有范围、方向、映射长度、logical、属性和 FLUSH 反例不得发表且不改 dst。
    fn entry(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let limits = self.disk.limits();
        let unit = limits.logical_block_size as usize;
        let original = self.requests[0].originals[0];
        let segment = crate::mm::dma::PayloadRange {
            physical: original.address.value() as usize,
            length: original.length,
            direction: original.direction,
        };
        let before = self.disk.test_snapshot();
        context.expect_eq(
            limits.validate(BlkOperation::Read, u64::MAX, unit, [segment], 0),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(
            limits.validate(
                BlkOperation::Read,
                limits.capacity_sectors,
                unit,
                [segment],
                0,
            ),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(
            limits.validate(BlkOperation::Read, 0, unit + 512, [segment], 0),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(
            limits.validate(BlkOperation::Read, 0, 0, [], 0),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(
            limits.validate(BlkOperation::Read, 0, unit, [segment], 1),
            Err(BlockError::Unsupported),
        );

        let wrong = crate::mm::dma::PayloadRange {
            direction: BufferDirection::DriverToDevice,
            ..segment
        };
        context.expect_eq(
            limits.validate(BlkOperation::Read, 0, unit, [wrong], 0),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(
            limits.validate(
                BlkOperation::Read,
                0,
                511,
                [crate::mm::dma::PayloadRange {
                    length: 511,
                    ..segment
                }],
                0,
            ),
            Err(BlockError::Misaligned),
        );

        let mut logical = limits;
        logical.logical_block_size = 4096;
        context.expect_eq(
            logical.validate(BlkOperation::Read, 1, unit, [segment], 0),
            Err(BlockError::Misaligned),
        );
        let mut readonly = limits;
        readonly.readonly = true;
        context.expect_eq(
            readonly.validate(BlkOperation::Write, 0, unit, [wrong], 0),
            Err(BlockError::ReadOnly),
        );

        let mut flush = limits;
        flush.flush = false;
        context.expect_eq(
            flush.validate(BlkOperation::Flush, 0, 0, [], 0),
            Err(BlockError::Unsupported),
        );
        flush.flush = true;
        context.expect_eq(
            flush.validate(BlkOperation::Flush, 1, 0, [], 0),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(
            flush.validate(BlkOperation::Flush, 0, unit, [segment], 0),
            Err(BlockError::InvalidRange),
        );

        let mut destination = [0x73_u8; 512];
        context.expect_eq(
            self.disk.read_exact_at(1, &mut destination),
            Err(BlockError::Misaligned),
        );
        context.expect_eq(destination, [0x73_u8; 512]);
        context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);
        Ok(())
    }

    /// 真实不同设备 identity 错投在 admission 拒绝，不发表任何 descriptor，原页仍 once 归还。
    fn identity(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let extra = self
            .extra
            .as_ref()
            .ok_or(TestError("identity fixture missing extra disk"))?;
        context.expect_eq(self.disk.identity() == extra.identity(), false);
        let before = extra.test_snapshot();
        extra.submit_bio(
            self.requests[0]
                .pending
                .take()
                .expect("identity wrong target Bio"),
        );
        self.requests[0].reclaim(
            context,
            IoResult {
                completed: 0,
                error: Some(BlockError::InvalidRange),
            },
            None,
        )?;
        context.expect_eq(extra.test_snapshot().submitted, before.submitted);
        context.expect_eq(self.disk.test_snapshot().submitted, 0);
        Ok(())
    }

    /// 4K logical 也实际读取两个不同 sector 的完整非零图样，不把零页当有效夹具。
    fn exact(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let tag = self.tag()?;
        let unit = self.disk.limits().logical_block_size as usize;
        let mut first = [0_u8; mm::PAGE_SIZE];
        let mut second = [0_u8; mm::PAGE_SIZE];
        self.disk
            .read_exact_at(64 * 512, &mut first[..unit])
            .map_err(|_| TestError("first exact read failed"))?;
        self.disk
            .read_exact_at(96 * 512, &mut second[..unit])
            .map_err(|_| TestError("second exact read failed"))?;
        for (sector, bytes) in [(64_u64, &first[..unit]), (96_u64, &second[..unit])] {
            for (offset, byte) in bytes.iter().enumerate() {
                let at = sector * 512 + offset as u64;
                let expected = tag
                    .wrapping_mul(29)
                    .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                    .wrapping_add((at % 512) as u8);
                if *byte != expected {
                    return Err(TestError("exact read full pattern mismatch"));
                }
            }
        }
        context.expect_eq(first[..unit].iter().any(|byte| *byte != 0), true);
        context.expect_eq(first[..unit] != second[..unit], true);
        self.drained()
    }

    /// 独立控制父账本：反序最早短错误、生成失败和同步 child，地址绝不发表为硬件证据。
    fn model(&mut self, context: &mut TestContext<'_>, mode: Ledger) -> TestResult {
        self.ready()?;
        let unit = self.disk.limits().logical_block_size as usize;
        let before = self.disk.test_snapshot();
        let index = 0;
        let bio = self.requests[index]
            .pending
            .take()
            .expect("controlled ledger Bio");
        self.model_live = Some(index);
        let _ = bio.inner.begin();
        let expected = match mode {
            Ledger::Reverse => {
                for index in 0..3 {
                    bio.inner.register_child(index * unit, unit);
                }
                BioInner::finish_child(&bio.inner, 2 * unit, unit, unit, None);
                BioInner::finish_child(&bio.inner, 0, unit, unit, None);
                BioInner::finish_generation(&bio.inner, None, None);
                context.expect_eq(self.requests[index].completion.is_complete(), false);
                context.expect_eq(
                    self.requests[index].completion.take_memory().err(),
                    Some(BlockError::Busy),
                );
                BioInner::finish_child(&bio.inner, unit, unit, unit / 4, Some(BlockError::Io));
                IoResult {
                    completed: unit + unit / 4,
                    error: Some(BlockError::Io),
                }
            }
            Ledger::MinimumError => {
                for offset in [0, unit, 2 * unit] {
                    bio.inner.register_child(offset, unit);
                }
                BioInner::finish_child(
                    &bio.inner,
                    2 * unit,
                    unit,
                    unit / 2,
                    Some(BlockError::ShortIo),
                );
                BioInner::finish_child(&bio.inner, unit, unit, unit / 4, Some(BlockError::Io));
                context.expect_eq(self.requests[index].completion.is_complete(), false);
                BioInner::finish_generation(&bio.inner, None, None);
                context.expect_eq(self.requests[index].completion.is_complete(), false);
                context.expect_eq(
                    self.requests[index].completion.take_memory().err(),
                    Some(BlockError::Busy),
                );
                BioInner::finish_child(&bio.inner, 0, unit, unit, None);
                IoResult {
                    completed: unit + unit / 4,
                    error: Some(BlockError::Io),
                }
            }
            Ledger::GenerationFailure => {
                bio.inner.register_child(0, unit);
                BioInner::finish_generation(&bio.inner, Some((unit, BlockError::NoMemory)), None);
                context.expect_eq(self.requests[index].completion.is_complete(), false);
                context.expect_eq(
                    self.requests[index].completion.take_memory().err(),
                    Some(BlockError::Busy),
                );
                BioInner::finish_child(&bio.inner, 0, unit, unit, None);
                IoResult {
                    completed: unit,
                    error: Some(BlockError::NoMemory),
                }
            }
            Ledger::Synchronous => {
                bio.inner.register_child(0, unit);
                BioInner::finish_child(&bio.inner, 0, unit, unit, None);
                context.expect_eq(self.requests[index].completion.is_complete(), false);
                context.expect_eq(
                    self.requests[index].completion.take_memory().err(),
                    Some(BlockError::Busy),
                );
                BioInner::finish_generation(&bio.inner, None, None);
                IoResult {
                    completed: unit,
                    error: None,
                }
            }
            Ledger::WbInline | Ledger::WbReverse | Ledger::WbRejectedTail => unreachable!(),
        };

        self.requests[index].reclaim(context, expected, None)?;
        self.model_live = None;
        context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);
        context.diagnostic(format_args!(
            "aggregation scope=controlled-ledger addresses-not-published=1"
        ));
        Ok(())
    }

    /// 私有 scratch 原页调用生产 WB、父 child 和真实接受账；反序仅由控制者驱动，不代表硬件乱序。
    fn writeback_ledger(&mut self, context: &mut TestContext<'_>, mode: Ledger) -> TestResult {
        self.ready()?;
        let unit = self.disk.limits().logical_block_size as usize;
        self.wb_folios
            .try_reserve_exact(3)
            .map_err(|_| TestError("WB ledger folio owners OOM"))?;
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(3)
            .map_err(|_| TestError("WB ledger mapped owners OOM"))?;
        for _ in 0..3 {
            let page = mm::alloc_pages(0, true).ok_or(TestError("WB ledger original page OOM"))?;
            let folio = mm::FolioRef::from_pages(page)
                .map_err(|_| TestError("WB ledger folio conversion failed"))?;
            let mut read = folio
                .begin_read(0, mm::PAGE_SIZE)
                .map_err(|_| TestError("WB ledger initialization failed"))?;
            // SAFETY: alloc_pages 已真实零化整原页；页未发表，无设备或外部 CPU 写者，初始化访问已结束。
            unsafe {
                read.complete(IoResult {
                    completed: mm::PAGE_SIZE,
                    error: None,
                })
            };
            self.wb_folios.push(folio);
        }

        let mut identities = [0usize; 3];
        for (index, folio) in self.wb_folios.iter().enumerate() {
            let mut content = folio.lock_content();
            content
                .copy_from(0, &[0x40 + index as u8; 4096][..unit])
                .map_err(|_| TestError("WB ledger original write failed"))?;
            identities[index] = folio.phys_addr();
            // SAFETY: 这些页只在当前 fixture 私有持有，没有 VMA/PTE；内容 guard 排斥内核修改，范围已初始化。
            let wb = unsafe { content.begin_writeback(0, unit) }
                .map_err(|_| TestError("WB ledger begin failed"))?
                .ok_or(TestError("WB ledger lost original Dirty"))?;
            memory.push(
                MappedPages::from_writeback(wb)
                    .map_err(|_| TestError("WB ledger mapping failed"))?,
            );
        }

        // SAFETY: WB 凭证保同一原 folio，全部字节初始化且无 VMA；控制账本从不把地址发表到硬件。
        // begin/child/generation 使用生产对象；每项控制完成代表本模型的唯一访问尾部。
        self.wb_active = Some(
            unsafe { Bio::try_new(&self.disk, BlkOperation::Write, 32, memory, 0, None) }
                .map_err(|_| TestError("WB ledger parent preparation failed"))?,
        );
        let parent = self.wb_active.as_ref().unwrap().inner.clone();
        let completion = self.wb_active.as_ref().unwrap().completion();
        let before = self.disk.test_snapshot();
        let _ = parent.begin();
        self.wb_folios[0]
            .lock_content()
            .copy_from(0, &[0x7e])
            .map_err(|_| TestError("WB ledger new Dirty failed"))?;
        for index in 0..3 {
            parent.register_child(index * unit, unit);
            if mode != Ledger::WbRejectedTail || index < 2 {
                parent.accept_child(index * unit, unit);
            }
            if mode == Ledger::WbInline {
                BioInner::finish_child(&parent, index * unit, unit, unit, None);
                context.expect_eq(completion.is_complete(), false);
                context.expect_eq(completion.take_memory().err(), Some(BlockError::Busy));
                context.expect_eq(
                    self.wb_folios
                        .iter()
                        .all(|folio| folio.writeback_sectors() != 0),
                    true,
                );
            }
        }

        let accepted = if mode == Ledger::WbRejectedTail {
            2 * unit
        } else {
            3 * unit
        };
        context.expect_eq(completion.test_acceptance(), (3, 3 * unit, accepted));
        let expected = match mode {
            Ledger::WbInline => {
                BioInner::finish_generation(&parent, None, None);
                IoResult {
                    completed: 3 * unit,
                    error: None,
                }
            }
            Ledger::WbReverse => {
                BioInner::finish_child(&parent, 2 * unit, unit, 0, Some(BlockError::Io));
                BioInner::finish_child(&parent, unit, unit, unit, None);
                BioInner::finish_generation(&parent, None, None);
                context.expect_eq(completion.is_complete(), false);
                context.expect_eq(self.wb_folios[0].writeback_sectors() != 0, true);
                BioInner::finish_child(&parent, 0, unit, unit, None);
                IoResult {
                    completed: 2 * unit,
                    error: Some(BlockError::Io),
                }
            }
            Ledger::WbRejectedTail => {
                BioInner::finish_child(&parent, 2 * unit, unit, 0, Some(BlockError::QueueStopped));
                BioInner::finish_generation(
                    &parent,
                    Some((2 * unit, BlockError::QueueStopped)),
                    None,
                );
                BioInner::finish_child(&parent, unit, unit, unit, None);
                context.expect_eq(completion.is_complete(), false);
                context.expect_eq(self.wb_folios[2].writeback_sectors() != 0, true);
                BioInner::finish_child(&parent, 0, unit, unit, None);
                IoResult {
                    completed: 2 * unit,
                    error: Some(BlockError::QueueStopped),
                }
            }
            _ => unreachable!(),
        };

        context.expect_eq(completion.wait(), expected);
        let reclaimed = completion
            .take_memory()
            .map_err(|_| TestError("WB ledger original payload not returned"))?;
        for (mapped, identity) in reclaimed.iter().zip(identities) {
            context.expect_eq(
                mapped
                    .segment(crate::mm::dma::DmaDomain::qemu_coherent())
                    .expect("QEMU fixture mapping must be reachable")
                    .address
                    .value(),
                identity as u64,
            );
            context.expect_eq(
                mapped
                    .segment(crate::mm::dma::DmaDomain::qemu_coherent())
                    .expect("QEMU fixture mapping must be reachable")
                    .direction,
                BufferDirection::DriverToDevice,
            );
            context.expect_eq(mapped.is_device_owned(), false);
        }
        context.expect_eq(completion.take_memory().err(), Some(BlockError::Busy));
        context.expect_eq(
            self.wb_folios
                .iter()
                .all(|folio| folio.writeback_sectors() == 0),
            true,
        );
        context.expect_eq(self.wb_folios[0].dirty_sectors() != 0, true);
        context.expect_eq(self.wb_folios[1].dirty_sectors(), 0);
        context.expect_eq(
            self.wb_folios[2].dirty_sectors() != 0,
            mode == Ledger::WbRejectedTail,
        );
        context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);

        drop((reclaimed, completion, parent));
        self.wb_active.take();
        self.sync_model_dirty()?;
        context.expect_eq(
            self.wb_folios
                .iter()
                .all(|folio| folio.dirty_sectors() == 0 && folio.writeback_sectors() == 0),
            true,
        );
        context.diagnostic(format_args!("W10 scope=controlled-parent-ledger original-folio-DMA=1 inline-producer-hold=1 accepted-prefix={} hardware-reverse-order-tested=0 restored-and-new-dirty-real-scratch-sync=1", accepted));
        Ok(())
    }

    /// 控制账本留下的新 Dirty 和未接受尾部随后由真实 scratch I/O 同步，禁止清 Dirty 代替归还。
    fn sync_model_dirty(&mut self) -> TestResult {
        let unit = self.disk.limits().logical_block_size as usize;
        for (index, folio) in self.wb_folios.iter().enumerate() {
            if folio.dirty_sectors() == 0 {
                continue;
            }
            let mut memory = Vec::new_in(BioAllocator::Heap);
            memory
                .try_reserve_exact(1)
                .map_err(|_| TestError("WB scratch sync mapping OOM"))?;
            let mut content = folio.lock_content();
            // SAFETY: 模型父已全尾且从不发表硬件地址；私有原页无 VMA，guard 保对应真实脏范围。
            let wb = unsafe { content.begin_writeback(0, unit) }
                .map_err(|_| TestError("WB scratch sync begin failed"))?
                .ok_or(TestError("WB scratch sync lost Dirty"))?;
            drop(content);
            memory.push(
                MappedPages::from_writeback(wb)
                    .map_err(|_| TestError("WB scratch sync map failed"))?,
            );
            // SAFETY: 原 WB 范围完整初始化，无 CPU 裸别名；Disk 将其保到真正设备和父完成尾部。
            let bio = unsafe {
                Bio::try_new(
                    &self.disk,
                    BlkOperation::Write,
                    32 + (index * unit / 512) as u64,
                    memory,
                    0,
                    None,
                )
            }
            .map_err(|_| TestError("WB scratch sync parent OOM"))?;
            let complete = bio.completion();
            self.disk.submit_bio(bio);
            until("WB scratch sync did not reach real parent tail", || {
                complete.is_complete()
            })?;
            if complete.wait()
                != (IoResult {
                    completed: unit,
                    error: None,
                })
            {
                return Err(TestError("WB scratch real sync failed"));
            }
            drop(
                complete
                    .take_memory()
                    .map_err(|_| TestError("WB scratch original payload missing"))?,
            );

            let mut readback = [0u8; mm::PAGE_SIZE];
            self.disk
                .read_exact_at(32 * 512 + (index * unit) as u64, &mut readback[..unit])
                .map_err(|_| TestError("WB scratch full readback failed"))?;
            if readback[0] != if index == 0 { 0x7e } else { 0x40 + index as u8 }
                || readback[1..unit]
                    .iter()
                    .any(|byte| *byte != 0x40 + index as u8)
            {
                return Err(TestError("WB scratch original full bytes differ"));
            }
        }
        Ok(())
    }

    /// 真 stop 后拒新 Bio 是单独行为，不把旧 finish 全套隐藏到其它 case 末尾。
    fn stopped(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        self.disk
            .stop()
            .map_err(|_| TestError("stopped admission real stop failed"))?;
        let before = self.disk.test_snapshot();
        self.submit(0);
        self.requests[0].reclaim(
            context,
            IoResult {
                completed: 0,
                error: Some(BlockError::QueueStopped),
            },
            None,
        )?;
        context.expect_eq(self.disk.test_snapshot().submitted, before.submitted);
        context.expect_eq(self.disk.stop(), Ok(()));
        Ok(())
    }

    /// 先释放合法等待源/恢复 used、停 producer 和原设备并 join，再关同域；错误保活 case-abort。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self
            .model_live
            .is_some_and(|index| !self.requests[index].completion.is_complete())
        {
            // 主体在控制账本中途 unwind 时不能猜剩余义务或 Drop 设备拥有原页；原完成对象仍强保页。
            return Err(TestError(
                "controlled ledger obligations did not converge; retaining original fixture",
            ));
        }

        self.observation.gate.complete();
        if self.disk.queue.lock().phase == QueuePhase::Live {
            self.disk.test_resume_completions();
        }
        if let Some(bio) = &self.wb_active {
            if !bio.completion().is_complete() {
                return Err(TestError(
                    "WB controlled parent retains unfinished original obligations",
                ));
            }
        }
        drop(self.wb_active.take());
        self.sync_model_dirty()?;
        drop(core::mem::take(&mut self.prepared));
        // peer 尚未提交而主体提前退出时，应先真实拒绝它，让已进入的回调合法返回。
        for index in 0..self.requests.len() {
            if self.requests[index].pending.is_some() {
                // 没有运行 producer 仍能触碰此 Bio；先清其未发表拥有，再由其原账本 reject 唤醒 peer。
                let bio = self.requests[index].pending.take().unwrap();
                BioInner::reject(&bio.inner, BlockError::QueueStopped);
            }
        }

        self.drained()?;
        drop(self.claim.take());
        self.disk
            .stop()
            .map_err(|_| TestError("queue stop cannot safely converge; retaining DMA fixture"))?;
        if let Some(extra) = &self.extra {
            extra
                .stop()
                .map_err(|_| TestError("extra identity stop cannot safely converge"))?;
        }
        self.join_tasks()?;
        self.queue.close_sync();

        if matches!(
            self.scenario,
            Scenario::Callback
                | Scenario::Progress
                | Scenario::ProgressBatch
                | Scenario::Generation
                | Scenario::FsScratch
        ) {
            // 设备 stop 和同域 close 已确认全部 slot/finalizer 原 Work 真尾部，不再查询此槽。
            let locator = CALLBACK_OWNER.lock().take();
            drop(locator);
        }
        let q = self.disk.queue.lock();
        context.expect_eq(q.phase, QueuePhase::Stopped);
        context.expect_eq(
            (
                q.generators,
                q.active_slots,
                q.notify_active,
                q.writing_slots,
            ),
            (0, 0, 0, 0),
        );
        drop(q);
        context.expect_eq(self.disk.irq.lock().is_none(), true);
        Ok(())
    }

    /// consuming tear 在真实 quiesce/join 后 Drop 原 owner，六域基线来自本例准备之前。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        let after = settled()?;
        context.expect_eq(after, baseline);
        context.diagnostic(format_args!("queue-resources six-domains-before={:?} six-domains-after={:?} real-stop-join-cleanup=1", baseline, after));
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Ledger {
    Reverse,
    MinimumError,
    GenerationFailure,
    Synchronous,
    WbInline,
    WbReverse,
    WbRejectedTail,
}

macro_rules! fixture {
    ($name:ident, $scenario:ident) => {
        struct $name(Owner);
        impl Fixture<DeviceInventory> for $name {
            /// 每例借一次库存独立准备真实域、原页与限额，不重复扫描设备。
            fn set_up(
                context: &mut TestContext<'_>,
                resources: &DeviceInventory,
            ) -> Result<Self, TestError> {
                Owner::prepare(Scenario::$scenario, context, resources).map(Self)
            }
            /// 具体 fixture 真实停源并 join，失败由现成 typed case-abort 保住原 owner。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// consuming Drop 安全原 owner，再核本实例完整六域页账。
            fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
                self.0.tear_down(context)
            }
        }
    };
}
fixture!(Multi, Multi);
fixture!(Tag, Tag);
fixture!(Descriptor, Descriptor);
fixture!(Rearm, Rearm);
fixture!(Split, Split);
fixture!(Merge, Merge);
fixture!(Notify, Notify);
fixture!(Tail, Tail);
fixture!(Callback, Callback);
fixture!(Scratch, Scratch);
fixture!(Entry, Entry);
fixture!(Identity, Identity);
fixture!(Exact, Exact);
fixture!(Model, Model);
fixture!(Progress, Progress);
fixture!(ProgressBatch, ProgressBatch);
fixture!(Generation, Generation);
fixture!(Stopped, Stopped);
fixture!(FsScratch, FsScratch);
fixture!(ReadonlyDevice, ReadonlyDevice);

#[ktest::suite(fixture = Multi, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("block", "B04", "T5"), resources("root"), timeout_ms = 15000)]
mod multi {
    use super::*;
    /// 八个真实不同 Task 各读两页不同 sector，保原页并证明实际 peak 多在途。
    #[test]
    fn eight_tasks_original_pages(
        context: &mut TestContext<'_>,
        fixture: &mut Multi,
    ) -> TestResult {
        fixture.0.multi(context)
    }
}
#[ktest::suite(fixture = Tag, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("block", "B07", "T5"), resources("root"), timeout_ms = 15000)]
mod tags {
    use super::*;
    /// 真 tag 耗尽、kick、生产 Blocked 与 used 唤醒。
    #[test]
    fn tag_exhaustion_blocks_and_wakes(
        context: &mut TestContext<'_>,
        fixture: &mut Tag,
    ) -> TestResult {
        fixture.0.pressure(context)
    }
}
#[ktest::suite(fixture = Descriptor, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("block", "B07", "T5"), resources("root"), timeout_ms = 15000)]
mod descriptors {
    use super::*;
    /// 固定四 SG 请求在 tag 未满时耗尽真实 descriptor。
    #[test]
    fn descriptor_exhaustion_blocks_and_wakes(
        context: &mut TestContext<'_>,
        fixture: &mut Descriptor,
    ) -> TestResult {
        fixture.0.pressure(context)
    }
}
#[ktest::suite(fixture = Rearm, suite_resources = DeviceInventory, context = "Thread", tags("block", "B08", "T5"), resources("root"), timeout_ms = 15000)]
mod interrupts {
    use super::*;
    /// flags 禁中断、重装复查和实际超过生产预算的真实 used 续行。
    #[test]
    fn flags_rearm_budget_continuation(
        context: &mut TestContext<'_>,
        fixture: &mut Rearm,
    ) -> TestResult {
        fixture.0.rearm(context)
    }
}
#[ktest::suite(fixture = Split, suite_resources = DeviceInventory, context = "Thread", tags("block", "B05", "B06", "T5"), resources("root"), timeout_ms = 15000)]
mod scatter {
    use super::*;
    /// 三独立原页沿真实限额 split，所有 child 后统一父 ready。
    #[test]
    fn original_sg_limited_children(
        context: &mut TestContext<'_>,
        fixture: &mut Split,
    ) -> TestResult {
        fixture.0.split(context)
    }
}
#[ktest::suite(fixture = Merge, suite_resources = DeviceInventory, context = "Thread", tags("block", "B09", "T5"), resources("root"), timeout_ms = 15000)]
mod merge {
    use super::*;
    /// 同 batch 三父只发一个真实 request，每父原页独立归还。
    #[test]
    fn three_parents_one_request(context: &mut TestContext<'_>, fixture: &mut Merge) -> TestResult {
        fixture.0.batch(context)
    }
}
#[ktest::suite(fixture = Notify, suite_resources = DeviceInventory, context = "Thread", tags("block", "B09", "T5"), resources("root"), timeout_ms = 15000)]
mod notify {
    use super::*;
    /// 非相邻三个 request 的显式 batch 只 unplug 一次。
    #[test]
    fn explicit_batch_single_notify(
        context: &mut TestContext<'_>,
        fixture: &mut Notify,
    ) -> TestResult {
        fixture.0.batch(context)
    }
}
#[ktest::suite(fixture = Tail, suite_resources = DeviceInventory, context = "Thread", tags("block", "B09", "T5"), resources("root"), timeout_ms = 15000)]
mod tail {
    use super::*;
    /// DMA boundary 无合法尾 logical block 时首父仍真实 unplug 完成。
    #[test]
    fn failing_tail_unplugs_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut Tail,
    ) -> TestResult {
        fixture.0.batch(context)
    }
}
#[ktest::suite(fixture = Callback, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("block", "callback", "T5"), resources("root"), timeout_ms = 15000)]
mod callbacks {
    use super::*;
    /// Task 回调真 Blocked 保槽与父，放 gate 后原尾部归权。
    #[test]
    fn blocked_task_callback_holds_slot(
        context: &mut TestContext<'_>,
        fixture: &mut Callback,
    ) -> TestResult {
        fixture.0.callback_tail(context)
    }
}
#[ktest::suite(fixture = Scratch, suite_resources = DeviceInventory, context = "Thread", tags("block", "B10", "T5"), resources("scratch"), timeout_ms = 15000)]
mod scratch {
    use super::*;
    /// 不借根盘写入：独占 scratch 的 claim 拒写、Busy stop、释放后真写读与 FLUSH。
    #[test]
    fn ro_fs_claim_then_unclaimed_raw_write_readback_flush(
        context: &mut TestContext<'_>,
        fixture: &mut Scratch,
    ) -> TestResult {
        fixture.0.scratch(context)
    }
}
#[ktest::suite(fixture = Entry, suite_resources = DeviceInventory, context = "Thread", tags("block", "entry", "T5"), resources("root"), timeout_ms = 15000)]
mod entry {
    use super::*;
    /// 非法范围/映射方向/长度/logical/属性/FLUSH 不发表且 dst 保持原值。
    #[test]
    fn invalid_input_never_publishes(
        context: &mut TestContext<'_>,
        fixture: &mut Entry,
    ) -> TestResult {
        fixture.0.entry(context)
    }
}
#[ktest::suite(fixture = Identity, suite_resources = DeviceInventory, context = "Thread", tags("block", "entry", "T5"), resources("root", "extra"), timeout_ms = 15000)]
mod identity {
    use super::*;
    /// 真双设备的错 identity Bio 拒绝且原页只归还一次。
    #[test]
    fn wrong_disk_identity_never_publishes(
        context: &mut TestContext<'_>,
        fixture: &mut Identity,
    ) -> TestResult {
        fixture.0.identity(context)
    }
}
#[ktest::suite(fixture = Exact, suite_resources = DeviceInventory, context = "Thread", tags("block", "read-exact", "T5"), resources("root"), timeout_ms = 15000)]
mod exact {
    use super::*;
    /// 普通逻辑块含 4K 的不同 sector 必须实际读取非零不同完整图样。
    #[test]
    fn distinct_nonzero_logical_reads(
        context: &mut TestContext<'_>,
        fixture: &mut Exact,
    ) -> TestResult {
        fixture.0.exact(context)
    }
}
#[ktest::suite(fixture = Model, suite_resources = DeviceInventory, context = "Thread", isolation = "Case", tags("block", "controlled-ledger", "T5"), timeout_ms = 15000)]
mod ledger {
    use super::*;
    /// 受控 child 顺序与生成尾部仅证明账本，不宣称硬件反序成功。
    #[test]
    #[case("reverse_short_prefix", Ledger::Reverse)]
    #[case("minimum_error_prefix", Ledger::MinimumError)]
    #[case("generation_failure_prefix", Ledger::GenerationFailure)]
    #[case("synchronous_child_producer_hold", Ledger::Synchronous)]
    #[ktest(resources("root"))]
    fn controlled_aggregation(
        context: &mut TestContext<'_>,
        fixture: &mut Model,
        mode: Ledger,
    ) -> TestResult {
        fixture.0.model(context, mode)
    }

    /// 三个真实原 folio 的 WB 责任随同一父账消费，控制次序和真正 scratch 收尾分开报告。
    #[test]
    #[case("wb_inline", Ledger::WbInline)]
    #[case("wb_reverse", Ledger::WbReverse)]
    #[case("wb_rejected_tail", Ledger::WbRejectedTail)]
    #[ktest(tags("W10", "fs-sync"), resources("scratch"))]
    fn original_folio_writeback_parent(
        context: &mut TestContext<'_>,
        fixture: &mut Model,
        mode: Ledger,
    ) -> TestResult {
        fixture.0.writeback_ledger(context, mode)
    }
}
#[ktest::suite(fixture = Progress, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("block", "callback", "T5"), resources("root"), timeout_ms = 15000)]
mod progress {
    use super::*;
    /// 非 batch 两个真实 used 的 progress Work 必须让 peer 完成而不被 A 回调阻断。
    #[test]
    fn progress_peer_two_real_used(
        context: &mut TestContext<'_>,
        fixture: &mut Progress,
    ) -> TestResult {
        fixture.0.peer(context)
    }
}
#[ktest::suite(fixture = ProgressBatch, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("block", "callback", "T5"), resources("root"), timeout_ms = 15000)]
mod progress_batch {
    use super::*;
    /// 同 batch 的 Task peer 父不能错误 merge 导致槽完成依赖死锁。
    #[test]
    fn batch_peer_two_real_used(
        context: &mut TestContext<'_>,
        fixture: &mut ProgressBatch,
    ) -> TestResult {
        fixture.0.peer(context)
    }
}
#[ktest::suite(fixture = Generation, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("block", "controlled-ledger", "callback", "T5"), resources("root"), timeout_ms = 15000)]
mod generation {
    use super::*;
    /// child 先结束的 generation-peer 只控制未发表账本，真实 reserved finalizer 不阻断 B。
    #[test]
    fn generation_peer_unpublished_ledger(
        context: &mut TestContext<'_>,
        fixture: &mut Generation,
    ) -> TestResult {
        fixture.0.peer(context)
    }
}
#[ktest::suite(fixture = Stopped, suite_resources = DeviceInventory, context = "Thread", tags("block", "stop", "T5"), resources("root"), timeout_ms = 15000)]
mod stopped {
    use super::*;
    /// 实际停止后拒新 admission 且 repeat stop 不重放。
    #[test]
    fn stopped_admission_returns_original(
        context: &mut TestContext<'_>,
        fixture: &mut Stopped,
    ) -> TestResult {
        fixture.0.stopped(context)
    }
}

#[ktest::suite(fixture = FsScratch, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2,
    tags("block", "FS-claim"), resources("scratch"), timeout_ms = 15000)]
mod fs_claim {
    use super::*;
    /// scratch 的真实 FS 授权写、完整读回与协商 FLUSH；活回调持占用至真实尾部。
    #[test]
    fn authorized_write_readback_and_callback_tail(
        context: &mut TestContext<'_>,
        fixture: &mut FsScratch,
    ) -> TestResult {
        fixture.0.fs_scratch(context)
    }

    /// 已接受生成者、实际 slot 和卷范围共同仲裁，非重叠 scratch 保持真实可写。
    #[test]
    fn bounded_claims_keep_nonoverlapping_scratch_writable(
        context: &mut TestContext<'_>,
        fixture: &mut FsScratch,
    ) -> TestResult {
        let owner = &mut fixture.0;
        owner.ready()?;
        let root = crate::block::BlockView::new(owner.disk.clone(), 0, 1024)
            .map_err(|_| TestError("root view construction failed"))?;
        let scratch = crate::block::BlockView::new(owner.disk.clone(), 2048, 1024)
            .map_err(|_| TestError("scratch view construction failed"))?;
        let accepted = owner
            .disk
            .admit(owner.requests[0].pending.take().unwrap())
            .map_err(|_| TestError("raw writer admission failed"))?;
        context.expect_eq(
            Disk::claim_fs(&scratch, AccessMode::ReadOnly).err(),
            Some(BlockError::Busy),
        );
        let root_claim = Disk::claim_fs(&root, AccessMode::ReadWrite)
            .map_err(|_| TestError("nonoverlapping accepted writer blocked root"))?;
        accepted.finish(Some((0, BlockError::QueueStopped)));
        owner.requests[0].reclaim(
            context,
            IoResult {
                completed: 0,
                error: Some(BlockError::QueueStopped),
            },
            None,
        )?;

        let overlap = crate::block::BlockView::new(owner.disk.clone(), 512, 1024)
            .map_err(|_| TestError("overlap view construction failed"))?;
        context.expect_eq(
            Disk::claim_fs(&overlap, AccessMode::ReadOnly).err(),
            Some(BlockError::Busy),
        );
        root_claim.submit_bio(owner.requests[1].pending.take().unwrap());
        owner.requests[1].reclaim(
            context,
            IoResult {
                completed: 0,
                error: Some(BlockError::InvalidRange),
            },
            None,
        )?;

        scratch.submit_bio(owner.requests[2].pending.take().unwrap());
        owner.requests[2].success(context, None)?;
        until("scratch raw slot tail did not finish", || {
            owner.disk.test_snapshot().inflight == 0
        })?;

        let scratch_claim = Disk::claim_fs(&scratch, AccessMode::ReadWrite)
            .map_err(|_| TestError("disjoint scratch claim blocked"))?;
        let mut bytes = [0x7b; 512];
        context.expect_eq(
            scratch.read_exact_at(1024 * 512, &mut bytes),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(bytes, [0x7b; 512]);
        drop(scratch_claim);
        drop(root_claim);
        Ok(())
    }
}

#[ktest::suite(fixture = ReadonlyDevice, suite_resources = DeviceInventory, context = "Thread",
    tags("block", "FS-claim"), resources("scratch"), timeout_ms = 15000)]
mod readonly_device {
    use super::*;
    /// 参数必须使设备真实协商 RO，不替换生产 capability 或伪造 limits。
    #[test]
    fn negotiated_ro_rejects_rw_claim(
        context: &mut TestContext<'_>,
        fixture: &mut ReadonlyDevice,
    ) -> TestResult {
        fixture.0.readonly_device(context)
    }
}
