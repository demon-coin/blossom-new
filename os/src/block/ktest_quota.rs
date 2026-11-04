//! 同一原完成域的非 slot finalizer 配额回归；受控父账本不冒充硬件 used。

use ::ktest::{self, Fixture, TestContext, TestError, TestResult};
use alloc::alloc::{alloc, dealloc};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::Layout;
use core::hint::black_box;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use virtio_drivers::{BufferDirection, MappedSegment};

use super::disk::QueuePhase;
use super::{
    AccessMode, Bio, BioAllocator, BioCompletion, BlkOperation, BlockError, Disk, DiskRef, EndIo,
    FsClaim, IoResult,
};
use crate::drivers::virtio::{
    VirtioDevice, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN, VIRTIO_VENDOR_ID,
};
use crate::drivers::DeviceInventory;
use crate::mm::dma::MappedPages;
use crate::sync::SpinLock;
use crate::task::{self, JoinHandle, PreparedTask, State, Task};
use crate::workqueue::{Work, WorkQueue, WorkQueueConfig};
use crate::{cpu, mm, sched, time, timer};

// EndIo 的函数指针只经此单槽定位当前 fixture；所有 Task/页/gate 均由该例拥有。
static ACTIVE: SpinLock<Option<Arc<Observation>>> = SpinLock::new(None);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Origin {
    Reject,
    GenerationLast,
}

struct Observation {
    peer: BioCompletion,
    expected_a: IoResult,
    originals: Vec<MappedSegment>,
    tasks: SpinLock<Vec<Option<Arc<Task>>>>,
    entered: AtomicUsize,
    returned: AtomicUsize,
    valid_a: AtomicBool,
    peer_calls: AtomicUsize,
    peer_task: SpinLock<Option<Arc<Task>>>,
    valid_b: AtomicBool,
    peer_original: MappedSegment,
    peer_tag: AtomicUsize,
    stop_tasks: SpinLock<Vec<Option<Arc<Task>>>>,
    stop_results: SpinLock<Vec<Option<Result<(), BlockError>>>>,
}

impl Observation {
    /// 短锁仅克隆现存强拥有，真实 Blocked 状态始终向调度器查询。
    fn blocked(&self) -> usize {
        (0..self.originals.len())
            .filter(|&ordinal| {
                let observed = self.tasks.lock()[ordinal].clone();
                observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
            })
            .count()
    }

    /// 独立 stop Task 的观察不能用尚未返回或 entered 标记替代真实 park。
    fn stop_blocked(&self, index: usize) -> bool {
        let task = self.stop_tasks.lock()[index].clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

/// 克隆当前例的有界强拥有后立即放开定位锁，不在回调中分配控制对象。
fn observation() -> Arc<Observation> {
    ACTIVE
        .lock()
        .as_ref()
        .expect("quota fixture absent")
        .clone()
}

/// 固定 raw 图样按整个成功范围逐字节核对，不把首 byte 当作数据完整性证明。
fn patterned(memory: &MappedPages, tag: u8, sector: u64) -> bool {
    if memory.is_device_owned() {
        return false;
    }
    // SAFETY: 调用者只在 Read 全长成功且 CPU 已收权后调用；原页不再有设备写者。
    unsafe { memory.bytes() }
        .iter()
        .enumerate()
        .all(|(offset, actual)| {
            let at = sector * 512 + offset as u64;
            *actual
                == tag
                    .wrapping_mul(29)
                    .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                    .wrapping_add((at % 512) as u8)
        })
}

/// A 只等独立 B Read；拒绝不发表 Write，generation-last 不发表任何地址。
fn waiting_callback(result: &IoResult, memory: &[MappedPages]) {
    let owner = observation();
    let valid = *result == owner.expected_a && memory.len() == 1 && !memory[0].is_device_owned();
    let ordinal = if valid {
        let mapped = &memory[0];
        let segment = mapped
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable");
        // SAFETY: A 的原页预先全部清零；两种 origin 均无设备 descriptor 引用。
        let zero = unsafe { mapped.bytes() }.iter().all(|byte| *byte == 0);
        owner.originals.iter().position(|original| {
            zero && segment.address == original.address
                && segment.length == original.length
                && segment.direction == original.direction
        })
    } else {
        None
    };
    if let Some(ordinal) = ordinal {
        if owner.tasks.lock()[ordinal]
            .replace(task::current_task())
            .is_some()
        {
            owner.valid_a.store(false, Ordering::Release);
        }
    } else {
        owner.valid_a.store(false, Ordering::Release);
    }
    owner.entered.fetch_add(1, Ordering::Release);
    let expected_b = IoResult {
        completed: owner.peer_original.length as usize,
        error: None,
    };
    if owner.peer.wait() != expected_b {
        owner.valid_a.store(false, Ordering::Release);
    }
    owner.returned.fetch_add(1, Ordering::Release);
}

/// B 的真实 slot Task 回调检查完整不同盘图样、原页和恰一次执行，不等待自身。
fn peer_callback(result: &IoResult, memory: &[MappedPages]) {
    let owner = observation();
    sched::assert_switchable();
    let once = owner.peer_calls.fetch_add(1, Ordering::AcqRel) == 0;
    *owner.peer_task.lock() = Some(task::current_task());
    let valid = once
        && *result
            == IoResult {
                completed: owner.peer_original.length as usize,
                error: None,
            }
        && memory.len() == 1
        && memory[0]
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable")
            .address
            == owner.peer_original.address
        && memory[0]
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable")
            .length
            == owner.peer_original.length
        && memory[0]
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable")
            .direction
            == owner.peer_original.direction
        && patterned(
            &memory[0],
            owner.peer_tag.load(Ordering::Acquire) as u8,
            400,
        );
    if !valid {
        owner.valid_b.store(false, Ordering::Release);
    }
}

/// 有限单调期限只检查确切谓词；计时器睡眠本身不是任何完成或阻塞证据。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(10))
        .ok_or(TestError("quota deadline overflow"))?;
    while !ready() {
        if time::now() >= deadline {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 所有参与者离栈后整理真实 allocator backing，保存完整 MM 六域账本。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("quota services did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 每例借 suite 的一次发现库存准备独占工作设备，不重复扫描 PCI。
fn devices(
    inventory: &DeviceInventory,
    count: usize,
) -> Result<Vec<(VirtioDevice, bool)>, TestError> {
    let pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    if inventory.mmio_blocks.len() + pci.clone().count() != count {
        return Err(TestError(
            "quota requires exact declared root/extra raw device count",
        ));
    }
    let mut prepared = Vec::new();
    prepared
        .try_reserve_exact(count)
        .map_err(|_| TestError("quota inventory owner allocation failed"))?;
    for candidate in &inventory.mmio_blocks {
        // SAFETY: 本例唯一库存与永久强序映射；host 只挂该例的 raw 工作副本，无其它 owner。
        let device = unsafe { VirtioDevice::prepare_mmio(candidate) }
            .map_err(|_| TestError("quota MMIO preparation failed"))?;
        prepared.push((device, candidate.version == 1));
    }
    for candidate in pci {
        // SAFETY: 同一真实库存中的独占 BDF；上一例 stop/join 完成后才会创建本例。
        let device = unsafe { VirtioDevice::prepare_pci(&inventory.pci, candidate) }
            .map_err(|_| TestError("quota PCI preparation failed"))?;
        prepared.push((device, false));
    }
    Ok(prepared)
}

/// 原零页在压力前完整预备；Write 只用作 claim 拒绝，不向设备发表。
fn bio(
    disk: &Disk,
    operation: BlkOperation,
    callback: fn(&IoResult, &[MappedPages]),
) -> Result<(Bio, MappedSegment), TestError> {
    let unit = disk.limits().logical_block_size as usize;
    if unit > mm::PAGE_SIZE {
        return Err(TestError("quota logical block exceeds original page"));
    }
    let pages =
        mm::alloc_pages(0, true).ok_or(TestError("quota original zero page unavailable"))?;
    let direction = if operation == BlkOperation::Write {
        BufferDirection::DriverToDevice
    } else {
        BufferDirection::DeviceToDriver
    };
    let mapped = MappedPages::new(pages, 0, unit, direction)
        .map_err(|_| TestError("quota original page mapping failed"))?;
    let original = mapped
        .segment(crate::mm::dma::DmaDomain::qemu_coherent())
        .expect("QEMU fixture mapping must be reachable");
    let mut memory = Vec::new_in(BioAllocator::Heap);
    memory
        .try_reserve_exact(1)
        .map_err(|_| TestError("quota original owner vector failed"))?;
    memory.push(mapped);
    // SAFETY: 全零原页完整初始化且无裸别名；A 仅等待独立 B，不等待自身/stop/复用槽。
    let bio = unsafe { Bio::try_new(disk, operation, 400, memory, 0, Some(EndIo::Task(callback))) }
        .map_err(|_| {
            TestError("quota Bio construction failed; Reject needs writable raw working copy")
        })?;
    Ok((bio, original))
}

struct Pressure {
    pages: Vec<mm::OwnedPages>,
    // 原堆对象的首个 usize 仅在此唯一拥有者内保存下一节点，不需要压力期间扩容。
    heap: [usize; mm::SLUB_SIZES.len()],
}

impl Pressure {
    /// 页凭证容量在任何压力之前准备，堆链的控制槽直接属于 fixture。
    fn prepare() -> Result<Self, TestError> {
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(mm::stats().present_pages)
            .map_err(|_| TestError("quota pressure owner capacity unavailable"))?;
        Ok(Self {
            pages,
            heap: [0; mm::SLUB_SIZES.len()],
        })
    }

    /// 真正耗尽页和当前控制 Task 可用的全部堆类别，返回无分配的观察证据。
    fn exhaust(&mut self) -> (bool, bool) {
        mm::shrink_all();
        mm::drain_all_pages();
        while let Some(page) = mm::alloc_pages(0, false) {
            assert!(
                self.pages.len() < self.pages.capacity(),
                "quota pressure capacity"
            );
            self.pages.push(page);
        }
        for (index, size) in mm::SLUB_SIZES.iter().copied().enumerate() {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("quota heap pressure layout");
            loop {
                // SAFETY: 固定非零合法布局，真实全局堆入口；只消费真实成功分配。
                let pointer = black_box(unsafe { alloc(layout) });
                if pointer.is_null() {
                    break;
                }
                // SAFETY: 每类别至少8字节且8对齐，独占对象保存自身链字，没有外部借用。
                unsafe {
                    pointer.cast::<usize>().write(self.heap[index]);
                }
                self.heap[index] = pointer as usize;
            }
        }
        mm::drain_all_pages();
        let pages_failed = black_box(mm::alloc_pages(0, false)).is_none();
        let mut heap_failed = true;
        for size in mm::SLUB_SIZES
            .iter()
            .copied()
            .chain(core::iter::once(mm::PAGE_SIZE * 2))
        {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("quota heap probe layout");
            // SAFETY: 探测同一真实全局堆；若意外成功立即按原布局归还，不记录假 OOM。
            // 不让分配与立即归还被优化成假成功，保留真实 NULL 探测。
            let pointer = black_box(unsafe { alloc(layout) });
            if !pointer.is_null() {
                heap_failed = false;
                unsafe {
                    dealloc(pointer, layout);
                }
            }
        }
        (pages_failed, heap_failed)
    }

    /// 每个原堆节点按原布局归还一次，再释放原页压力，绝不手工改 allocator 状态。
    fn release(&mut self) {
        for (index, size) in mm::SLUB_SIZES.iter().copied().enumerate() {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("quota heap release layout");
            while self.heap[index] != 0 {
                let pointer = self.heap[index] as *mut u8;
                // SAFETY: 节点来自本链的原 alloc，首字已初始化，布局与分配完全一致。
                unsafe {
                    self.heap[index] = pointer.cast::<usize>().read();
                    dealloc(pointer, layout);
                }
            }
        }
        self.pages.clear();
    }
}

impl Drop for Pressure {
    /// body 的任何错误也不遗失真正页/堆压力凭证。
    fn drop(&mut self) {
        self.release();
    }
}

struct Owner {
    baseline: mm::MemoryStats,
    origin: Origin,
    oom: bool,
    disks: Vec<DiskRef>,
    legacy: Vec<bool>,
    queue: WorkQueue,
    max_active: usize,
    observation: Arc<Observation>,
    completions: Vec<BioCompletion>,
    prepared_a: Option<PreparedTask>,
    prepared_b: Option<PreparedTask>,
    producer: Option<JoinHandle>,
    submitter: Option<JoinHandle>,
    prepared_stops: Vec<Option<PreparedTask>>,
    stops: Vec<Option<JoinHandle>>,
    claim: Option<FsClaim>,
    pressure: Pressure,
    activation_error: bool,
    before_a: (usize, usize),
    before_b: (usize, usize),
    used_target: u16,
    b_slot: Option<usize>,
}

impl Owner {
    /// 借一次库存建立原共享 quota、Bio、观察者及本例 producer/stop 栈。
    fn prepare(
        count: usize,
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        let baseline = settled()?;
        let origin = if context.case.name.contains("reject") {
            Origin::Reject
        } else {
            Origin::GenerationLast
        };
        let oom = context
            .case
            .parameter
            .is_some_and(|name| name.starts_with("oom"));
        let devices = devices(inventory, count)?;
        let max_active = devices
            .iter()
            .map(|(device, _)| usize::from(device.info().tag_count) + 1)
            .sum();
        if devices
            .iter()
            .any(|(device, _)| device.info().tag_count != 32)
        {
            return Err(TestError(
                "quota Tag32 regression needs actual negotiated tag_count=32",
            ));
        }
        if origin == Origin::Reject && devices[0].0.info().readonly {
            return Err(TestError(
                "Reject requires writable raw working copy; no Write issued",
            ));
        }
        let mut disks = Vec::new();
        let mut legacy = Vec::new();
        disks
            .try_reserve_exact(count)
            .map_err(|_| TestError("quota Disk owner capacity failed"))?;
        legacy
            .try_reserve_exact(count)
            .map_err(|_| TestError("quota layout owner capacity failed"))?;
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("quota original completion domain failed"))?;
        for (device, layout) in devices {
            match Disk::prepare(device, &queue) {
                Ok(disk) => {
                    disks.push(disk);
                    legacy.push(layout);
                }
                Err(_) => {
                    for disk in &disks {
                        disk.stop().expect("unpublished quota Disk stop");
                    }
                    drop(disks);
                    queue.close_sync();
                    return Err(TestError("quota Disk preparation failed"));
                }
            }
        }
        let resources = (|| {
            // 原33回归与新增普通quota+1边界分别具名，双盘总是使用实际共享额度+1。
            let a_count = if count == 1
                && context
                    .case
                    .parameter
                    .is_some_and(|name| name.ends_with("33"))
            {
                max_active
            } else {
                max_active + 1
            };
            let (peer, peer_original) = bio(&disks[count - 1], BlkOperation::Read, peer_callback)?;
            let mut pending = Vec::new();
            let mut completions = Vec::new();
            let mut originals = Vec::new();
            let mut tasks = Vec::new();
            pending
                .try_reserve_exact(a_count)
                .map_err(|_| TestError("quota pending capacity"))?;
            completions
                .try_reserve_exact(a_count)
                .map_err(|_| TestError("quota completion capacity"))?;
            originals
                .try_reserve_exact(a_count)
                .map_err(|_| TestError("quota original capacity"))?;
            tasks
                .try_reserve_exact(a_count)
                .map_err(|_| TestError("quota Task observation capacity"))?;
            for _ in 0..a_count {
                let operation = if origin == Origin::Reject {
                    BlkOperation::Write
                } else {
                    BlkOperation::Read
                };
                let (bio, original) = bio(&disks[0], operation, waiting_callback)?;
                completions.push(bio.completion());
                originals.push(original);
                tasks.push(None);
                pending.push(bio);
            }
            let mut stop_tasks = Vec::new();
            let mut stop_results = Vec::new();
            stop_tasks
                .try_reserve_exact(count)
                .map_err(|_| TestError("quota stop observation capacity"))?;
            stop_results
                .try_reserve_exact(count)
                .map_err(|_| TestError("quota stop result capacity"))?;
            for _ in 0..count {
                stop_tasks.push(None);
                stop_results.push(None);
            }
            let expected_a = IoResult {
                completed: if origin == Origin::Reject {
                    0
                } else {
                    disks[0].limits().logical_block_size as usize
                },
                error: if origin == Origin::Reject {
                    Some(BlockError::ReadOnly)
                } else {
                    None
                },
            };
            let observation = Arc::try_new(Observation {
                peer: peer.completion(),
                expected_a,
                originals,
                tasks: SpinLock::new(tasks),
                entered: AtomicUsize::new(0),
                returned: AtomicUsize::new(0),
                valid_a: AtomicBool::new(true),
                peer_calls: AtomicUsize::new(0),
                peer_task: SpinLock::new(None),
                valid_b: AtomicBool::new(true),
                peer_original,
                peer_tag: AtomicUsize::new(0),
                stop_tasks: SpinLock::new(stop_tasks),
                stop_results: SpinLock::new(stop_results),
            })
            .map_err(|_| TestError("quota observation allocation failed"))?;
            let a_disk = disks[0].clone();
            let prepared_a = task::prepare_kernel_on(0, move || {
                for bio in pending {
                    match origin {
                        Origin::Reject => a_disk.submit_bio(bio),
                        Origin::GenerationLast => super::tests::generation_last(&a_disk, bio),
                    }
                }
                0
            })
            .map_err(|_| TestError("quota A producer preparation failed"))?;
            let b_disk = disks[count - 1].clone();
            let prepared_b = task::prepare_kernel_on(1, move || {
                b_disk.submit_bio(peer);
                0
            })
            .map_err(|_| TestError("quota B producer preparation failed"))?;
            let mut prepared_stops = Vec::new();
            let mut stops = Vec::new();
            prepared_stops
                .try_reserve_exact(count)
                .map_err(|_| TestError("quota stop Task capacity"))?;
            stops
                .try_reserve_exact(count)
                .map_err(|_| TestError("quota stop Join capacity"))?;
            for (index, disk) in disks.iter().enumerate() {
                let disk = disk.clone();
                let observed = observation.clone();
                let stop = task::prepare_kernel_on(1, move || {
                    observed.stop_tasks.lock()[index] = Some(task::current_task());
                    observed.stop_results.lock()[index] = Some(disk.stop());
                    0
                })
                .map_err(|_| TestError("quota independent stop preparation failed"))?;
                prepared_stops.push(Some(stop));
                stops.push(None);
            }
            let pressure = Pressure::prepare()?;
            Ok((
                observation,
                completions,
                prepared_a,
                prepared_b,
                prepared_stops,
                stops,
                pressure,
            ))
        })();
        let (observation, completions, prepared_a, prepared_b, prepared_stops, stops, pressure) =
            match resources {
                Ok(resources) => resources,
                Err(error) => {
                    for disk in &disks {
                        disk.stop().expect("unpublished quota resources stop");
                    }
                    drop(disks);
                    queue.close_sync();
                    return Err(error);
                }
            };
        assert!(
            ACTIVE.lock().is_none(),
            "previous quota callbacks still own locator"
        );
        *ACTIVE.lock() = Some(observation.clone());
        let mut activation_error = false;
        for disk in &disks {
            if disk.start().is_err() {
                activation_error = true;
                break;
            }
        }
        Ok(Self {
            baseline,
            origin,
            oom,
            disks,
            legacy,
            queue,
            max_active,
            observation,
            completions,
            prepared_a: Some(prepared_a),
            prepared_b: Some(prepared_b),
            producer: None,
            submitter: None,
            prepared_stops,
            stops,
            claim: None,
            pressure,
            activation_error,
            before_a: (0, 0),
            before_b: (0, 0),
            used_target: 0,
            b_slot: None,
        })
    }

    /// 各 raw 工作副本全长 identity Read 对固定图样，跨盘必须 tag1/tag2 均真实不同。
    fn identities(&self, context: &mut TestContext<'_>) -> TestResult {
        let mut tags = 0_u8;
        for (index, disk) in self.disks.iter().enumerate() {
            let unit = disk.limits().logical_block_size as usize;
            let mut bytes = [0_u8; mm::PAGE_SIZE];
            disk.read_exact_at(0, &mut bytes[..unit])
                .map_err(|_| TestError("quota raw identity Read failed"))?;
            // 真实发现顺序不等于 host root/extra attach 顺序；用全范围图样核实际身份。
            let tag = match bytes[0] {
                29 => 1_u8,
                58 if self.disks.len() == 2 => 2_u8,
                _ => return Err(TestError("quota requires declared raw tag1/tag2 identity")),
            };
            if tags & (1 << tag) != 0 {
                return Err(TestError(
                    "quota cross-disk raw identities are not distinct",
                ));
            }
            tags |= 1 << tag;
            for (at, actual) in bytes[..unit].iter().enumerate() {
                let expected = tag
                    .wrapping_mul(29)
                    .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                    .wrapping_add((at % 512) as u8);
                if *actual != expected {
                    return Err(TestError(
                        "quota root/extra raw full identity pattern mismatch",
                    ));
                }
            }
            if index + 1 == self.disks.len() {
                self.observation
                    .peer_tag
                    .store(tag as usize, Ordering::Release);
            }
            until("quota identity slot tail", || {
                disk.queue.lock().active_slots == 0
            })?;
            if let Some(work) = &disk.progress_work {
                work.as_ref().flush();
            }
            context.expect_eq(disk.limits().tag_count, 32);
        }
        context.expect_eq(tags, if self.disks.len() == 2 { 6 } else { 2 });
        Ok(())
    }

    /// 保原33反例并覆盖普通配额+1边界，核每个 A 自己的原生 Task 真 Blocked。
    fn saturation(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error {
            return Err(TestError(
                "quota activation failed; owners retained for quiesce",
            ));
        }
        self.identities(context)?;
        if self.origin == Origin::Reject {
            self.claim = Some(
                Disk::claim_fs(
                    &crate::block::BlockView::whole(self.disks[0].clone()),
                    AccessMode::ReadOnly,
                )
                .map_err(|_| TestError("quota RO FS claim failed"))?,
            );
        }
        self.before_a = self.counts(0);
        let b = self.disks.len() - 1;
        self.before_b = self.counts(b);
        self.producer = Some(self.prepared_a.take().expect("quota prepared A").publish());
        let count = self.completions.len();
        until("quota A callbacks did not all genuinely Block", || {
            self.observation.entered.load(Ordering::Acquire) == count
                && self.observation.blocked() == count
        })?;
        until("quota A producer did not leave stack", || {
            self.producer.as_ref().unwrap().try_join() == Some(0)
        })?;
        context.expect_eq(self.counts(0), self.before_a);
        context.expect_eq(self.disks[0].queue.lock().active_slots, 0);
        context.expect_eq(self.observation.returned.load(Ordering::Acquire), 0);
        context.expect_eq(
            self.completions
                .iter()
                .all(|completion| !completion.is_complete()),
            true,
        );
        for (index, completion) in self.completions.iter().enumerate() {
            let native = completion
                .ktest_finalizer_task()
                .ok_or(TestError("quota A missing reserved native Task"))?;
            let observed = self.observation.tasks.lock()[index]
                .clone()
                .ok_or(TestError("quota A callback Task missing"))?;
            context.expect_eq(Arc::ptr_eq(&native, &observed), true);
            for prior in 0..index {
                let previous = self.observation.tasks.lock()[prior].clone().unwrap();
                context.expect_eq(Arc::ptr_eq(&observed, &previous), false);
            }
        }
        self.disks[b].ktest_hold_used();
        self.used_target = super::tests::used_index(&self.disks[b], self.legacy[b]).wrapping_add(1);
        self.submitter = Some(self.prepared_b.take().expect("quota prepared B").publish());
        until("quota B publication missing", || {
            self.counts(b).0 == self.before_b.0 + 1
        })?;
        until("quota B real hardware used missing", || {
            super::tests::used_index(&self.disks[b], self.legacy[b]) == self.used_target
        })?;
        until("quota B producer did not leave stack", || {
            self.submitter.as_ref().unwrap().try_join() == Some(0)
        })?;
        {
            let q = self.disks[b].queue.lock();
            context.expect_eq(q.active_slots, 1);
            self.b_slot = q
                .slot_states
                .iter()
                .position(|state| matches!(state, super::disk::SlotState::Submitted));
        }
        if self.b_slot.is_none() {
            return Err(TestError("quota B lost original submitted slot"));
        }
        context.expect_eq(
            self.queue.ktest_reserved_workers(),
            self.max_active + count + 1,
        );
        let slot_work = self.disks[b].slots[self.b_slot.unwrap()]
            .completion_work
            .as_ref()
            .unwrap();
        let progress_work = self.disks[b].progress_work.as_ref().unwrap();
        until("quota B native reserved Tasks not ready", || {
            slot_work.ktest_reserved_task().is_some()
                && progress_work.ktest_reserved_task().is_some()
                && self.observation.peer.ktest_finalizer_task().is_some()
        })?;
        let b_slot_task = slot_work.ktest_reserved_task().unwrap();
        let b_progress_task = progress_work.ktest_reserved_task().unwrap();
        let b_finalizer_task = self.observation.peer.ktest_finalizer_task().unwrap();
        context.expect_eq(Arc::ptr_eq(&b_slot_task, &b_progress_task), false);
        context.expect_eq(Arc::ptr_eq(&b_slot_task, &b_finalizer_task), false);
        context.expect_eq(Arc::ptr_eq(&b_progress_task, &b_finalizer_task), false);
        for ordinal in 0..count {
            let a_task = self.observation.tasks.lock()[ordinal].clone().unwrap();
            context.expect_eq(Arc::ptr_eq(&a_task, &b_slot_task), false);
            context.expect_eq(Arc::ptr_eq(&a_task, &b_progress_task), false);
            context.expect_eq(Arc::ptr_eq(&a_task, &b_finalizer_task), false);
        }
        context.expect_eq(self.observation.peer.is_complete(), false);
        context.expect_eq(self.observation.peer_calls.load(Ordering::Acquire), 0);
        context.expect_eq(self.observation.blocked(), count);
        // 在 B 的真实 SG/descriptor/生成者与全部观察/stop 栈已经存在后才开始 OOM。
        let oom_evidence = if self.oom {
            Some(self.pressure.exhaust())
        } else {
            None
        };
        self.claim.take();
        let stopped_wait = if !self.oom {
            self.start_stop(b);
            until(
                "quota B stop must Draining and really Block before used release",
                || {
                    self.disks[b].queue.lock().phase == QueuePhase::Draining
                        && self.observation.stop_blocked(b)
                },
            )
        } else {
            Ok(())
        };
        self.disks[b].ktest_release_used();
        let progressed = until(
            "quota B and all A did not progress with original execution resources",
            || {
                self.observation.peer.is_complete()
                    && self.observation.returned.load(Ordering::Acquire) == count
                    && self.completions.iter().all(BioCompletion::is_complete)
            },
        );
        // OOM 区间内不打印、构造 SG 或申请控制对象；放压后再报告确切探测结果。
        self.pressure.release();
        stopped_wait?;
        progressed?;
        // OOM 放压之前不发表其它盘 stop，避免先归还原控制栈稀释 B 恢复前的真实压力。
        if self.oom {
            self.start_stop(b);
        }
        if b != 0 {
            self.start_stop(0);
        }
        until("quota real B slot tail missing", || {
            self.disks[b].queue.lock().active_slots == 0
        })?;
        if let Some((pages, heap)) = oom_evidence {
            context.expect_eq(pages, true);
            context.expect_eq(heap, true);
            context.diagnostic(format_args!(
                "quota OOM page-failed={} heap-failed={}",
                pages, heap
            ));
        }
        context.expect_eq(self.observation.entered.load(Ordering::Acquire), count);
        context.expect_eq(self.observation.returned.load(Ordering::Acquire), count);
        context.expect_eq(self.observation.valid_a.load(Ordering::Acquire), true);
        context.expect_eq(self.observation.peer_calls.load(Ordering::Acquire), 1);
        context.expect_eq(self.observation.valid_b.load(Ordering::Acquire), true);
        let b_task = self
            .observation
            .peer_task
            .lock()
            .clone()
            .ok_or(TestError("quota B Task callback missing"))?;
        context.expect_eq(Arc::ptr_eq(&b_task, &b_slot_task), true);
        for task in self.observation.tasks.lock().iter().flatten() {
            context.expect_eq(Arc::ptr_eq(task, &b_task), false);
        }
        context.expect_eq(self.counts(b), (self.before_b.0 + 1, self.before_b.1 + 1));
        if b != 0 {
            context.expect_eq(self.counts(0), self.before_a);
        }
        for (completion, original) in self.completions.iter().zip(&self.observation.originals) {
            reclaim(
                context,
                completion,
                *original,
                self.observation.expected_a,
                None,
            )?;
        }
        reclaim(
            context,
            &self.observation.peer,
            self.observation.peer_original,
            IoResult {
                completed: self.observation.peer_original.length as usize,
                error: None,
            },
            Some(self.observation.peer_tag.load(Ordering::Acquire) as u8),
        )?;
        context.diagnostic(format_args!("quota origin={:?} disks={} normal-max-active={} A={} raw-Read-used={} Task-Blocked={} mode={} A-scope={} B-scope=real-hardware-read-full-pattern",
            self.origin, self.disks.len(), self.max_active, count, self.used_target, count,
            if self.oom { "page-and-heap-OOM-after-real-used" } else { "normal" },
            if self.origin == Origin::Reject { "real-FsClaim-RO-admission-no-write" } else { "controlled-ledger-no-descriptors-not-hardware-used" }));
        Ok(())
    }

    /// 只借生产 queue 短锁观察真 publish/finish，不另造测试计数模型。
    fn counts(&self, index: usize) -> (usize, usize) {
        let q = self.disks[index].queue.lock();
        (q.ktest_stop.published, q.ktest_stop.finished)
    }

    /// stop 控制 Task 和闭包早已准备；发表不需要申请 Task/栈/控制对象。
    fn start_stop(&mut self, index: usize) {
        self.stops[index] = Some(
            self.prepared_stops[index]
                .take()
                .expect("quota stop published twice")
                .publish(),
        );
    }

    /// body 失败仍先结束生成者及释放 B，再按 B/A 责任顺序真 stop/IRQ/Work/native join。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.pressure.release();
        let b = self.disks.len() - 1;
        if !self.activation_error {
            if self.producer.is_some() && self.submitter.is_none() {
                self.submitter = Some(
                    self.prepared_b
                        .take()
                        .expect("quota cleanup B producer")
                        .publish(),
                );
            }
            for disk in &self.disks {
                disk.ktest_release_used();
            }
        }
        for join in [self.producer.as_ref(), self.submitter.as_ref()]
            .iter()
            .flatten()
        {
            until("quota producer cannot safely leave stack", || {
                join.try_join().is_some()
            })?;
        }
        for join in IntoIterator::into_iter([self.producer.take(), self.submitter.take()]).flatten()
        {
            if join.join() != 0 {
                return Err(TestError("quota producer returned failure"));
            }
        }
        drop(self.prepared_a.take());
        drop(self.prepared_b.take());
        self.claim.take();
        for index in (0..=b).rev() {
            if let Some(join) = self.stops[index].as_ref() {
                until("quota stop Task cannot safely leave stack", || {
                    join.try_join().is_some()
                })?;
            }
            if let Some(join) = self.stops[index].take() {
                if join.join() != 0 {
                    return Err(TestError("quota stop Task failure"));
                }
                context.expect_eq(self.observation.stop_results.lock()[index], Some(Ok(())));
            }
            drop(self.prepared_stops[index].take());
            self.disks[index].stop().map_err(|_| {
                TestError("quota Disk stop cannot safely converge; retaining original owner")
            })?;
            let q = self.disks[index].queue.lock();
            context.expect_eq(q.phase, QueuePhase::Stopped);
            context.expect_eq(q.device.is_none(), true);
            context.expect_eq(
                (
                    q.generators,
                    q.writer_generators,
                    q.active_slots,
                    q.writing_slots,
                    q.notify_active,
                ),
                (0, 0, 0, 0, 0),
            );
            if !self.activation_error {
                context.expect_eq(
                    (
                        q.ktest_stop.reset_requests,
                        q.ktest_stop.reset_confirmations,
                    ),
                    (1, 1),
                );
            }
            drop(q);
            context.expect_eq(self.disks[index].irq.lock().is_none(), true);
            if let Some(work) = &self.disks[index].progress_work {
                context.expect_eq(work.ktest_reserved_joined(), true);
            }
            for slot in &self.disks[index].slots {
                if let Some(work) = &slot.completion_work {
                    context.expect_eq(work.ktest_reserved_joined(), true);
                }
            }
        }
        // 所有 producer 已结束，所有盘 slot/progress 已 close/join；最后关闭原共享域。
        self.queue.close_sync();
        context.expect_eq(self.queue.ktest_reserved_workers(), 0);
        for completion in &self.completions {
            completion.test_flush_finalizer();
            context.expect_eq(completion.ktest_finalizer_joined(), true);
        }
        self.observation.peer.test_flush_finalizer();
        context.expect_eq(self.observation.peer.ktest_finalizer_joined(), true);
        // stop 与域真实 native join 之后才摘当前 EndIo 定位槽。
        let active = ACTIVE
            .lock()
            .take()
            .ok_or(TestError("quota locator disappeared before join"))?;
        context.expect_eq(Arc::ptr_eq(&active, &self.observation), true);
        drop(active);
        Ok(())
    }

    /// 最后观察 Arc/Task metadata/页拥有全部普通析构后再核六域，不把 joined 当成全析构。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        let after = settled()?;
        context.expect_eq(after, baseline);
        context.diagnostic(format_args!(
            "quota resources-before={:?} resources-after={:?} stopped-IRQ-Work-native-join=1",
            baseline, after
        ));
        Ok(())
    }
}

/// Bio 完成后只取同一原 DMA 页一次，并再核全范围真实 raw 数据。
fn reclaim(
    context: &mut TestContext<'_>,
    completion: &BioCompletion,
    original: MappedSegment,
    expected: IoResult,
    tag: Option<u8>,
) -> TestResult {
    context.expect_eq(completion.result(), Some(expected));
    let memory = completion
        .take_memory()
        .map_err(|_| TestError("quota original page unavailable"))?;
    context.expect_eq(memory.len(), 1);
    if let Some(mapped) = memory.first() {
        let segment = mapped
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable");
        context.expect_eq(
            (segment.address, segment.length, segment.direction),
            (original.address, original.length, original.direction),
        );
        context.expect_eq(mapped.is_device_owned(), false);
        if let Some(tag) = tag {
            context.expect_eq(patterned(mapped, tag, 400), true);
        }
    }
    context.expect_eq(completion.take_memory().err(), Some(BlockError::Busy));
    drop(memory);
    Ok(())
}

struct Single(Owner);
struct Double(Owner);

macro_rules! fixture {
    ($name:ident, $count:expr) => {
        impl Fixture<DeviceInventory> for $name {
            /// 每例只借一次发现的库存，独立取得真实设备、域、原页与控制责任。
            fn set_up(
                context: &mut TestContext<'_>,
                inventory: &DeviceInventory,
            ) -> Result<Self, TestError> {
                Owner::prepare($count, context, inventory).map(Self)
            }
            /// 不能安全收敛时 Err 交原 typed case-abort 保活终结 VM，不 Drop 活设备。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// 全部 join 以后析构并核完整六域。
            fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
                self.0.tear_down(context)
            }
        }
    };
}
fixture!(Single, 1);
fixture!(Double, 2);

#[ktest::suite(fixture = Single, suite_resources = DeviceInventory, context = "Thread", isolation = "Case", min_cpus = 2,
    tags("block", "quota", "B11", "T5"), resources("root"), timeout_ms = 60000)]
mod single {
    use super::*;
    /// 原 tag32 单盘 A33 反例和原普通 quota33 的 A34 边界分别保留合法 peer 等待。
    #[test]
    #[case("normal33", ())]
    #[case("oom33", ())]
    #[case("normal", ())]
    #[case("oom", ())]
    fn reject(context: &mut TestContext<'_>, fixture: &mut Single, _: ()) -> TestResult {
        fixture.0.saturation(context)
    }

    /// 单盘生成者最后完成只走已有受控账本 seam，独立 B 是真实硬件 Read。
    #[test]
    #[case("normal33", ())]
    #[case("oom33", ())]
    #[case("normal", ())]
    #[case("oom", ())]
    fn generation_last(context: &mut TestContext<'_>, fixture: &mut Single, _: ()) -> TestResult {
        fixture.0.saturation(context)
    }
}

#[ktest::suite(fixture = Double, suite_resources = DeviceInventory, context = "Thread", isolation = "Case", min_cpus = 2,
    tags("block", "quota", "cross-disk", "B11", "T5"), resources("root", "extra"), timeout_ms = 60000)]
mod double {
    use super::*;
    /// 双盘共享原普通 quota66，A67 在盘A原保留栈等待盘B真实 Task Read。
    #[test]
    #[case("normal", ())]
    #[case("oom", ())]
    fn reject(context: &mut TestContext<'_>, fixture: &mut Double, _: ()) -> TestResult {
        fixture.0.saturation(context)
    }

    /// 双盘受控 generation-last A67 不占用或替代另一盘独立 slot/progress/Bio 栈。
    #[test]
    #[case("normal", ())]
    #[case("oom", ())]
    fn generation_last(context: &mut TestContext<'_>, fixture: &mut Double, _: ()) -> TestResult {
        fixture.0.saturation(context)
    }
}

struct Unsubmitted {
    baseline: mm::MemoryStats,
    queue: WorkQueue,
    work: Pin<Arc<Work>>,
    calls: Arc<AtomicUsize>,
    closed: bool,
}

/// 正常路径不会调用：若未提交 Work 意外执行，真实原子证据使 case 失败。
fn unsubmitted_callback(work: Pin<&Work>) {
    // SAFETY: fixture 强保 calls 到原域 close/join 结束；context 指向该固定 Arc payload。
    let calls = unsafe { &*(work.context() as *const AtomicUsize) };
    calls.fetch_add(1, Ordering::Release);
}

impl Fixture<()> for Unsubmitted {
    /// 真构造 reserved Work/native Task，却不提交任何票，并保外部 Work owner 穿过 close。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled()?;
        let calls = Arc::try_new(AtomicUsize::new(0))
            .map_err(|_| TestError("unsubmitted observation failed"))?;
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("unsubmitted domain failed"))?;
        let work = match Work::try_new_reserved(
            &queue,
            0,
            unsubmitted_callback,
            Arc::as_ptr(&calls) as usize,
        ) {
            Ok(work) => work,
            Err(_) => {
                queue.close_sync();
                return Err(TestError("unsubmitted reserved Work preparation failed"));
            }
        };
        Ok(Self {
            baseline,
            queue,
            work,
            calls,
            closed: false,
        })
    }

    /// owner 不丢 Work，不伪 join；真实域 close 会撤未提交预留并等原 manager 离栈 join。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if !self.closed {
            self.queue.close_sync();
            self.closed = true;
        }
        context.expect_eq(self.queue.ktest_reserved_workers(), 0);
        context.expect_eq(self.work.ktest_reserved_joined(), true);
        context.expect_eq(self.calls.load(Ordering::Acquire), 0);
        Ok(())
    }

    /// 所有原控制 Arc drop 后核 MM 六域，native joined 与 Task metadata 分开验证。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        let after = settled()?;
        context.expect_eq(after, baseline);
        Ok(())
    }
}

#[ktest::suite(fixture = Unsubmitted, context = "Thread", min_cpus = 2,
    tags("block", "quota", "reserved", "T5"), timeout_ms = 15000)]
mod reserved {
    use super::*;
    /// 外部仍持成功构造但从未 submit 的 Work 时，domain.close_sync 必须实际退出并 join。
    #[test]
    fn unsubmitted_domain_close(context: &mut TestContext<'_>, fixture: &mut Unsubmitted) {
        context.expect_eq(fixture.queue.ktest_reserved_workers(), 1);
        context.expect_eq(fixture.work.ktest_reserved_joined(), false);
        fixture.queue.close_sync();
        fixture.closed = true;
        context.expect_eq(fixture.queue.ktest_reserved_workers(), 0);
        context.expect_eq(fixture.work.ktest_reserved_joined(), true);
        context.expect_eq(fixture.calls.load(Ordering::Acquire), 0);
        context.expect_eq(fixture.work.as_ref().queue().is_err(), true);
    }
}
