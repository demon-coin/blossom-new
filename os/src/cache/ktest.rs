//! 原 folio、真实块设备读入与共同元数据内容的具名消费回归。

use ::ktest::{self, Fixture, TestContext, TestError, TestResult};
use alloc::alloc::{alloc, dealloc};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::Layout;
use core::hint::black_box;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::block::{
    self, AccessMode, BioAllocator, BlockError, BlockView, Disk, DiskRef, IoResult,
};
use crate::cache::{BufferRef, FolioIndex, MetadataCache};
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::DeviceInventory;
use crate::fs::ext4::{codec, Ext4Volume};
use crate::fs::{FsError, InodeMetadata};
use crate::mm::folio::{FolioRead, FolioRef};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, JoinHandle, PreparedTask, State, Task};
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, mm, sched, time, timer};

const OFFSET: u64 = 64 * mm::PAGE_SIZE as u64;
const PRESSURE_CAPACITY: usize = 65536;

/// 父入口只显式引用宏产生的实际注册槽，不维护第二份用例表。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 12] {
    [
        folio::__ktest_anchor(),
        metadata_views::__ktest_anchor(),
        metadata_singleflight::__ktest_anchor(),
        file_singleflight::__ktest_anchor(),
        metadata_parallel::__ktest_anchor(),
        file_parallel::__ktest_anchor(),
        stopped_failure::__ktest_anchor(),
        bounded::__ktest_anchor(),
        oom::__ktest_anchor(),
        prepare_oom::__ktest_anchor(),
        mutable_metadata::__ktest_anchor(),
        metadata_records::__ktest_anchor(),
    ]
}

/// 有界轮询只观察真实状态，等待正确性由实际 Blocked 与生产唤醒证明。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("cache deadline overflow"))?;
    while !ready() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 参与者离栈后整理真实 SLUB/PCP，六域包括当前框架与服务的常驻页。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("cache resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::flush_all();
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 完整消费拥有者后核六域，不重置计数或回滚物理内存状态。
fn recovered(context: &mut TestContext<'_>, baseline: mm::MemoryStats) -> TestResult {
    let after = settled()?;
    context.expect_eq(after, baseline);
    context.diagnostic(format_args!(
        "cache-resources six-domains-before={:?} six-domains-after={:?} real-stop-join-cleanup=1",
        baseline, after
    ));
    Ok(())
}

/// 逐字节验证宿主原 raw 输入盘的非零图样，零内容或短范围不能充当成功。
fn patterned(bytes: &[u8], tag: u8, offset: u64, len: usize) -> bool {
    bytes.len() == len
        && bytes.iter().any(|byte| *byte != 0)
        && bytes.iter().enumerate().all(|(index, byte)| {
            let at = offset + index as u64;
            *byte
                == tag
                    .wrapping_mul(29)
                    .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                    .wrapping_add((at % 512) as u8)
        })
}

/// 每例借整组保活库存准备 root/extra 工作副本；前例已完成 stop，不重复一次性发现。
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
            "cache attached device count differs from declared resources",
        ));
    }

    let mut prepared = Vec::new();
    prepared
        .try_reserve_exact(count)
        .map_err(|_| TestError("cache device owner allocation failed"))?;
    for candidate in &inventory.mmio_blocks {
        // SAFETY: 库存由 suite 保活；前例 stop 已结束设备访问，本例独占候选直到真实 stop。
        prepared.push(
            unsafe { VirtioDevice::prepare_mmio(candidate) }
                .map_err(|_| TestError("cache MMIO preparation failed"))?,
        );
    }
    for device in pci {
        // SAFETY: BDF 借自整组库存；前例已 reset/同步尾部，本例独占至自身 IRQ/Work 结束。
        prepared.push(
            unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
                .map_err(|_| TestError("cache PCI preparation failed"))?,
        );
    }
    Ok(prepared)
}

struct Pages {
    baseline: mm::MemoryStats,
    folios: [Option<FolioRef>; 2],
    higher: Option<mm::OwnedPages>,
}

impl Fixture for Pages {
    /// 在捕获外保留两种原页的完整拥有，正文失败也由具体 fixture 归还。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled()?;
        let pages = mm::alloc_pages(0, false).ok_or(TestError("folio order0 preparation OOM"))?;
        let folio = FolioRef::from_pages(pages).map_err(|_| TestError("folio rejected order0"))?;
        let higher = mm::alloc_pages(1, false).ok_or(TestError("folio order1 preparation OOM"))?;
        Ok(Self {
            baseline,
            folios: [Some(folio), None],
            higher: Some(higher),
        })
    }

    /// 只有同步原页责任；析构全部拥有后核六域原基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        recovered(context, baseline)
    }
}

impl Pages {
    /// Clone 保同 PFN，非末引用不归页，末引用沿原分配器归还；高阶 Err 保原块与内容。
    fn identity(&mut self, context: &mut TestContext<'_>) -> TestResult {
        let before = settled()?;
        let first = self.folios[0].as_ref().unwrap();
        let address = first.phys_addr();
        self.folios[1] = Some(first.clone());
        context.expect_eq(self.folios[1].as_ref().unwrap().phys_addr(), address);
        context.expect_eq(self.folios[0].as_ref().unwrap().ref_count(), 2);
        drop(self.folios[0].take());
        context.expect_eq(self.folios[1].as_ref().unwrap().ref_count(), 1);
        context.expect_eq(settled()?, before);
        drop(self.folios[1].take());
        let mut expected = before;
        expected.buddy_free_pages += 1;
        context.expect_eq(settled()?, expected);

        let mut higher = self.higher.take().unwrap();
        let address = higher.phys_addr();
        let len = higher.len();
        // SAFETY: higher 唯一拥有整块且尚无设备/folio 别名，全部字节写定后才读取。
        unsafe { higher.as_mut_ptr().cast::<u8>().write_bytes(0xa7, len) };
        match FolioRef::from_pages(higher) {
            Err(mut original) => {
                context.expect_eq(original.phys_addr(), address);
                context.expect_eq(original.order(), 1);
                context.expect_eq(original.len(), len);
                // SAFETY: 上方整块已初始化，返回 Err 仍持原块唯一拥有，短借用结束后才移动。
                let unchanged =
                    unsafe { core::slice::from_raw_parts(original.as_mut_ptr().cast::<u8>(), len) }
                        .iter()
                        .all(|byte| *byte == 0xa7);
                context.expect_eq(unchanged, true);
                self.higher = Some(original);
            }
            Ok(folio) => {
                self.folios[0] = Some(folio);
                return Err(TestError("folio accepted higher-order pages"));
            }
        }
        drop(self.higher.take());
        expected.buddy_free_pages += 2;
        context.expect_eq(settled()?, expected);
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    Views,
    MetadataFlight,
    FileFlight,
    MetadataIndependent,
    Independent,
    Failure,
    Bounded,
    PrepareOom,
    Oom,
    MutableMetadata,
    MetadataRecords,
}

enum Returned {
    Buffer(BufferRef),
    Folio(FolioRef),
}

impl Returned {
    /// 只借本次真实页引用身份，观察不复制内容。
    fn folio(&self) -> &FolioRef {
        match self {
            Self::Buffer(buffer) => buffer.folio(),
            Self::Folio(folio) => folio,
        }
    }

    /// 真实成功初始化范围内核验整个页图样，不越过 buffer 的短视图。
    fn valid(&self, offset: u64) -> Result<bool, BlockError> {
        match self {
            Self::Buffer(buffer) => {
                buffer.with_read(|bytes| patterned(bytes, 1, offset, mm::PAGE_SIZE))
            }
            Self::Folio(folio) => {
                let mut bytes = [0_u8; mm::PAGE_SIZE];
                folio.copy_to(0, &mut bytes)?;
                Ok(patterned(&bytes, 1, offset, mm::PAGE_SIZE))
            }
        }
    }
}

pub(crate) struct HeapPressure {
    pages: Vec<mm::OwnedPages>,
    // 每个独占原堆对象的首字保存本类别下一节点，压力期间不申请链表节点。
    heap: [usize; mm::SLUB_SIZES.len()],
}

impl HeapPressure {
    /// 控制 Task 的真实页凭证容量先于设备 start 预留，堆类别链头常驻本 owner。
    pub(crate) fn prepare() -> Result<Self, TestError> {
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(mm::stats().present_pages)
            .map_err(|_| TestError("cache prepare-OOM pressure slots unavailable"))?;
        Ok(Self {
            pages,
            heap: [0; mm::SLUB_SIZES.len()],
        })
    }

    /// 固定 CPU 控制者耗尽真实页及全部 SLUB 类，再用原 allocator 观察真实 NULL。
    pub(crate) fn exhaust(&mut self) -> (bool, bool) {
        mm::shrink_all();
        mm::drain_all_pages();
        for order in [mm::MAX_ORDER, 0] {
            while let Some(pages) = mm::alloc_pages(order, false) {
                assert!(
                    self.pages.len() < self.pages.capacity(),
                    "cache pressure capacity"
                );
                self.pages.push(pages);
            }
        }

        for (index, size) in mm::SLUB_SIZES.iter().copied().enumerate() {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("cache heap pressure layout");
            loop {
                // SAFETY: 固定有效非零布局，真实全局分配器返回的成功对象由本 owner 独占。
                let pointer = black_box(unsafe { alloc(layout) });
                if pointer.is_null() {
                    break;
                }
                // SAFETY: 所有类别至少八字节且八对齐，首字仅保存本对象的独占链链接。
                unsafe { pointer.cast::<usize>().write(self.heap[index]) };
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
                .expect("cache heap probe layout");
            // SAFETY: 有效原布局探测同一真实 allocator，black_box 保留实际 NULL 观察。
            let pointer = black_box(unsafe { alloc(layout) });
            if !pointer.is_null() {
                heap_failed = false;
                // SAFETY: 意外成功的原对象没有外部引用，立即按同一布局归还一次。
                unsafe { dealloc(pointer, layout) };
            }
        }
        (pages_failed, heap_failed)
    }
}

impl Drop for HeapPressure {
    /// 正文返回或展开都按原布局释放每个节点，最后锁外归还全部物理页。
    fn drop(&mut self) {
        for (index, size) in mm::SLUB_SIZES.iter().copied().enumerate() {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("cache heap release layout");
            while self.heap[index] != 0 {
                let pointer = self.heap[index] as *mut u8;
                // SAFETY: 节点是本链尚未归还的原 alloc，首字已初始化，布局与分配相同。
                unsafe {
                    self.heap[index] = pointer.cast::<usize>().read();
                    dealloc(pointer, layout);
                }
            }
        }
    }
}

/// 预建固定 CPU0 Task 独占压力与唯一 read，正式准备返回后才放压；不制造完成结果。
fn prepare_oom_read(observation: &Observation, disk: &DiskRef) -> Result<(), BlockError> {
    let mut pressure = observation
        .pressure
        .lock()
        .take()
        .expect("cache prepared heap pressure missing");
    let evidence = pressure.exhaust();
    *observation.oom_evidence.lock() = Some(evidence);
    let read = observation
        .read
        .lock()
        .take()
        .expect("cache prepare-OOM original read missing");
    let result = if evidence == (true, true) {
        disk.load_folio(OFFSET, read, Vec::new_in(BioAllocator::Heap))
    } else {
        // 探测未证明耗尽时取消未发表 owner；正文将拒绝该证据，不冒称准备 OOM。
        drop(read);
        Err(BlockError::QueueStopped)
    };
    drop(pressure);
    result
}

struct Observation {
    cancelled: AtomicBool,
    gate: Completion,
    entered: [AtomicBool; 3],
    tasks: SpinLock<[Option<Arc<Task>>; 3]>,
    results: SpinLock<[Option<Result<Returned, BlockError>>; 3]>,
    read: SpinLock<Option<FolioRead>>,
    pressure: SpinLock<Option<HeapPressure>>,
    oom_evidence: SpinLock<Option<(bool, bool)>>,
}

impl Observation {
    /// 所有 gate、结果槽与唯一未提交 read owner 在任何设备发表之前准备。
    fn prepare(
        read: Option<FolioRead>,
        pressure: Option<HeapPressure>,
    ) -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            cancelled: AtomicBool::new(false),
            gate: Completion::new(),
            entered: [const { AtomicBool::new(false) }; 3],
            tasks: SpinLock::new([const { None }; 3]),
            results: SpinLock::new([const { None }; 3]),
            read: SpinLock::new(read),
            pressure: SpinLock::new(pressure),
            oom_evidence: SpinLock::new(None),
        })
        .map_err(|_| TestError("cache observer preparation OOM"))
    }

    /// 登记真实调用后的 Task 才统计 Blocked，gate 自身等待不计入缓存等待证据。
    fn blocked(&self, index: usize) -> bool {
        if !self.entered[index].load(Ordering::Acquire) {
            return false;
        }
        let task = self.tasks.lock()[index].clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

struct Owner {
    baseline: mm::MemoryStats,
    disk: DiskRef,
    extra: Option<DiskRef>,
    queue: WorkQueue,
    metadata: Option<Arc<MetadataCache>>,
    extra_metadata: Option<Arc<MetadataCache>>,
    mapping: Arc<FolioIndex>,
    observation: Arc<Observation>,
    held: [Option<Returned>; 3],
    prepared: Vec<PreparedTask>,
    joins: Vec<JoinHandle>,
    pressure: Vec<mm::OwnedPages>,
    legacy: bool,
    activation_error: Option<BlockError>,
    avail_address: usize,
    private_metadata: bool,
}

impl Owner {
    /// 借本组库存启动真实设备，再建立显式 FS 占用与本例控制拥有。
    fn prepare(scenario: Scenario, resources: &DeviceInventory) -> Result<Self, TestError> {
        let baseline = settled()?;
        let mut inventory = devices(
            resources,
            if matches!(scenario, Scenario::Views | Scenario::MetadataRecords) {
                2
            } else {
                1
            },
        )?;
        let device = inventory.remove(0);
        let info = device.info();
        let unit = info.logical_block_size as usize;
        if !matches!(unit, 512 | 1024 | 2048 | 4096)
            || (scenario == Scenario::Views
                && (1024 < unit
                    || inventory
                        .iter()
                        .any(|device| 1024 < device.info().logical_block_size)))
        {
            return Err(TestError(
                "cache views require logical block <= 1KiB; page cases require <= 4KiB",
            ));
        }

        let legacy = matches!(device.identity(), VirtioIdentity::Mmio { version: 1, .. });
        let ring = device.control_regions().0;
        let layout = virtio_drivers::queue::RingLayout::new(info.queue_depth, legacy)
            .map_err(|_| TestError("cache real ring layout invalid"))?;
        let avail_address =
            ring.cpu.as_ptr() as usize + layout.avail_offset() + core::mem::size_of::<u16>();
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(info.tag_count) + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("cache completion domain preparation OOM"))?;
        let disk = match Disk::prepare(device, &queue) {
            Ok(disk) => disk,
            Err(_) => {
                queue.close_sync();
                return Err(TestError("cache Disk preparation failed"));
            }
        };
        if disk.start().is_err() {
            disk.stop()
                .map_err(|_| TestError("cache failed activation stop failed"))?;
            drop(disk);
            queue.close_sync();
            return Err(TestError("cache Disk activation failed"));
        }

        let owner = Self::prepare_resources(
            scenario,
            baseline,
            disk.clone(),
            queue,
            legacy,
            inventory,
            avail_address,
        )?;
        Ok(owner)
    }

    /// 已启动设备先取得 RO 占用，缓存及任务准备失败按原拥有顺序释放。
    fn prepare_resources(
        scenario: Scenario,
        baseline: mm::MemoryStats,
        mut disk: DiskRef,
        queue: WorkQueue,
        legacy: bool,
        mut inventory: Vec<VirtioDevice>,
        avail_address: usize,
    ) -> Result<Self, TestError> {
        let mut extra = match inventory
            .pop()
            .map(|device| Disk::prepare(device, &queue))
            .transpose()
        {
            Ok(extra) => extra,
            Err(_) => {
                disk.stop()
                    .map_err(|_| TestError("unpublished cache Disk stop failed"))?;
                drop(disk);
                queue.close_sync();
                return Err(TestError("cache extra Disk preparation failed"));
            }
        };
        let resources = (|| {
            if let Some(extra) = &extra {
                extra
                    .start()
                    .map_err(|_| TestError("extra cache activation failed"))?;
            }
            if scenario == Scenario::MetadataRecords {
                let first_root =
                    Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadOnly, 8)
                        .is_ok();
                let second_root = Ext4Volume::open(
                    BlockView::whole(extra.as_ref().unwrap().clone()),
                    AccessMode::ReadOnly,
                    8,
                )
                .is_ok();
                if first_root == second_root {
                    return Err(TestError(
                        "record case requires exactly one real ext4 root and one scratch",
                    ));
                }
                if first_root {
                    core::mem::swap(&mut disk, extra.as_mut().unwrap());
                }
            }

            let capacity = if scenario == Scenario::Bounded { 2 } else { 8 };
            let metadata = if matches!(
                scenario,
                Scenario::Views
                    | Scenario::MetadataFlight
                    | Scenario::MetadataIndependent
                    | Scenario::Bounded
                    | Scenario::Oom
                    | Scenario::MutableMetadata
                    | Scenario::MetadataRecords
            ) {
                let mode = if matches!(
                    scenario,
                    Scenario::MutableMetadata | Scenario::MetadataRecords
                ) {
                    AccessMode::ReadWrite
                } else {
                    AccessMode::ReadOnly
                };
                let claim = Disk::claim_fs(&BlockView::whole(disk.clone()), mode)
                    .map_err(|_| TestError("metadata FS claim failed"))?;
                Some(
                    MetadataCache::shared(&claim, capacity)
                        .map_err(|_| TestError("metadata preparation failed"))?,
                )
            } else {
                None
            };
            let mapping = Arc::try_new(FolioIndex::new(None, false))
                .map_err(|_| TestError("mapping owner preparation OOM"))?;
            let extra_metadata = extra
                .as_ref()
                .map(|disk| {
                    let claim =
                        Disk::claim_fs(&BlockView::whole(disk.clone()), AccessMode::ReadOnly)?;
                    MetadataCache::shared(&claim, 8)
                })
                .transpose()
                .map_err(|_| TestError("extra metadata preparation failed"))?;

            let (failure_folio, read) =
                if matches!(scenario, Scenario::Failure | Scenario::PrepareOom) {
                    let (folio, read) = mapping
                        .get_or_prepare(64, None)
                        .map_err(|_| TestError("failure placeholder preparation failed"))?;
                    (Some(folio), read)
                } else {
                    (None, None)
                };
            let pressure = if scenario == Scenario::PrepareOom {
                Some(HeapPressure::prepare()?)
            } else {
                None
            };
            let observation = Observation::prepare(read, pressure)?;
            let count = match scenario {
                Scenario::MetadataFlight
                | Scenario::FileFlight
                | Scenario::MetadataIndependent
                | Scenario::Independent => 2,
                Scenario::Failure | Scenario::PrepareOom => 3,
                _ => 0,
            };
            let mut prepared = Vec::new();
            let mut joins = Vec::new();
            prepared
                .try_reserve_exact(
                    if matches!(
                        scenario,
                        Scenario::MutableMetadata | Scenario::MetadataRecords
                    ) {
                        2
                    } else {
                        count
                    },
                )
                .map_err(|_| TestError("cache prepared Task slots OOM"))?;
            joins
                .try_reserve_exact(
                    if matches!(
                        scenario,
                        Scenario::MutableMetadata | Scenario::MetadataRecords
                    ) {
                        2
                    } else {
                        count
                    },
                )
                .map_err(|_| TestError("cache join slots OOM"))?;
            let mut pressure = Vec::new();
            if scenario == Scenario::Oom {
                pressure
                    .try_reserve_exact(PRESSURE_CAPACITY)
                    .map_err(|_| TestError("cache pressure handles OOM"))?;
            }

            for index in 0..count {
                let observer = observation.clone();
                let task_disk = disk.clone();
                let cache = if matches!(
                    scenario,
                    Scenario::MetadataFlight | Scenario::MetadataIndependent
                ) {
                    metadata.clone()
                } else {
                    None
                };
                let file = mapping.clone();
                let failure_folio = failure_folio.clone();
                prepared.push(
                    task::prepare_kernel_on(index % cpu::count(), move || {
                        observer.tasks.lock()[index] = Some(task::current_task());
                        if matches!(scenario, Scenario::Failure | Scenario::PrepareOom)
                            && index == 0
                        {
                            observer.gate.wait();
                        }
                        if observer.cancelled.load(Ordering::Acquire) {
                            return 0;
                        }
                        observer.entered[index].store(true, Ordering::Release);
                        let key = if matches!(
                            scenario,
                            Scenario::Independent | Scenario::MetadataIndependent
                        ) {
                            64 + index as u64
                        } else {
                            64
                        };
                        let result = if matches!(
                            scenario,
                            Scenario::MetadataFlight | Scenario::MetadataIndependent
                        ) {
                            cache
                                .as_ref()
                                .unwrap()
                                .get(key, mm::PAGE_SIZE, None)
                                .map(Returned::Buffer)
                        } else if scenario == Scenario::PrepareOom && index == 0 {
                            prepare_oom_read(&observer, &task_disk)
                                .map(|()| Returned::Folio(failure_folio.unwrap()))
                        } else if scenario == Scenario::Failure && index == 0 {
                            let read = observer
                                .read
                                .lock()
                                .take()
                                .expect("failure original read owner missing");
                            task_disk
                                .load_folio(OFFSET, read, Vec::new_in(BioAllocator::Heap))
                                .map(|()| Returned::Folio(failure_folio.unwrap()))
                        } else {
                            match file.get_or_prepare(key, None) {
                                Ok((folio, Some(read))) => task_disk
                                    .load_folio(
                                        key * mm::PAGE_SIZE as u64,
                                        read,
                                        Vec::new_in(BioAllocator::Heap),
                                    )
                                    .map(|()| Returned::Folio(folio)),
                                Ok((folio, None)) => {
                                    folio.wait_ready().map(|()| Returned::Folio(folio))
                                }
                                Err(error) => Err(error),
                            }
                        };
                        observer.results.lock()[index] = Some(result);
                        0
                    })
                    .map_err(|_| TestError("cache native Task preparation failed"))?,
                );
            }
            let mut held = [const { None }; 3];
            held[0] = failure_folio.map(Returned::Folio);
            Ok((
                metadata,
                mapping,
                extra_metadata,
                observation,
                prepared,
                joins,
                pressure,
                held,
            ))
        })();
        match resources {
            Ok((
                metadata,
                mapping,
                extra_metadata,
                observation,
                prepared,
                joins,
                pressure,
                held,
            )) => Ok(Self {
                baseline,
                disk,
                extra,
                queue,
                metadata,
                extra_metadata,
                mapping,
                observation,
                prepared,
                joins,
                pressure,
                held,
                legacy,
                activation_error: None,
                avail_address,
                private_metadata: matches!(
                    scenario,
                    Scenario::MutableMetadata | Scenario::MetadataRecords
                ),
            }),
            Err(error) => {
                if let Some(extra) = &extra {
                    extra
                        .stop()
                        .map_err(|_| TestError("unpublished extra cache Disk stop failed"))?;
                }
                drop(extra);
                disk.stop()
                    .map_err(|_| TestError("unpublished cache Disk stop failed"))?;
                drop(disk);
                queue.close_sync();
                Err(error)
            }
        }
    }

    /// 激活失败仍保完整 fixture，交具体 quiesce 处理真实 stop。
    fn ready(&self) -> TestResult {
        if self.activation_error.is_some() {
            Err(TestError("cache Disk activation failed"))
        } else {
            Ok(())
        }
    }

    /// 消费一个预建原生 Task；固定 join 容量在发表前预留。
    fn publish_one(&mut self) {
        self.joins.push(self.prepared.remove(0).publish());
    }

    /// 全部 Task 真正离栈后才消费 JoinHandle，完成 flag 不能代替 join。
    fn join_tasks(&mut self) -> TestResult {
        for join in &self.joins {
            until("cache producer did not leave stack", || {
                join.try_join().is_some()
            })?;
        }
        let mut failed = false;
        for join in self.joins.drain(..) {
            failed |= join.join() != 0;
        }
        if failed {
            Err(TestError("cache native producer failed"))
        } else {
            Ok(())
        }
    }

    /// 真实硬件 used.idx 写到目标才继续，不修改 ring 或制造完成。
    fn used(&self, target: u16) -> TestResult {
        until("cache real used did not progress", || {
            block::tests::used_index(&self.disk, self.legacy) == target
        })
    }

    /// 结果先从观察锁移到 fixture 原拥有槽，再调用可能取得内容锁的真实消费者。
    fn take_results(&mut self) -> TestResult {
        for index in 0..2 {
            let result = self.observation.results.lock()[index].take();
            self.held[index] = Some(
                result
                    .ok_or(TestError("cache producer result absent"))?
                    .map_err(|_| TestError("cache real read failed"))?,
            );
        }
        Ok(())
    }

    /// 同 key 等待者实际 Blocked，恢复原 used 后全部消费同 PFN 与完整数据。
    fn singleflight(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        self.disk.test_suspend_completions();
        let before = self.disk.test_snapshot();
        let used = block::tests::used_index(&self.disk, self.legacy);
        self.publish_one();
        self.used(used.wrapping_add(1))?;
        until("cache read owner did not block", || {
            self.observation.blocked(0)
        })?;
        self.publish_one();
        until("cache same-key waiter did not block", || {
            self.observation.blocked(1)
        })?;
        context.expect_eq(self.disk.test_snapshot().submitted - before.submitted, 1);
        context.expect_eq(self.disk.test_snapshot().inflight, 1);

        self.disk.test_resume_completions();
        self.join_tasks()?;
        self.disk.test_flush_completions();
        self.take_results()?;
        let mut address = None;
        for returned in self.held[..2].iter().flatten() {
            if let Some(address) = address {
                context.expect_eq(returned.folio().phys_addr(), address);
            }
            address = Some(returned.folio().phys_addr());
            context.expect_eq(returned.valid(OFFSET), Ok(true));
        }
        context.expect_eq(self.disk.test_snapshot().submitted - before.submitted, 1);
        Ok(())
    }

    /// 不同文件页在真实 used 暂停窗口同时拥有两个设备请求。
    fn independent(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        self.disk.test_suspend_completions();
        let before = self.disk.test_snapshot();
        let used = block::tests::used_index(&self.disk, self.legacy);
        self.publish_one();
        self.publish_one();
        self.used(used.wrapping_add(2))?;
        until("independent file reads did not block", || {
            self.observation.blocked(0) && self.observation.blocked(1)
        })?;
        context.expect_eq(self.disk.test_snapshot().submitted - before.submitted, 2);
        context.expect_eq(self.disk.test_snapshot().inflight, 2);

        self.disk.test_resume_completions();
        self.join_tasks()?;
        self.disk.test_flush_completions();
        self.take_results()?;
        let mut first = None;
        for (index, returned) in self.held[..2].iter().flatten().enumerate() {
            if let Some(address) = first {
                context.expect_eq(returned.folio().phys_addr() != address, true);
            }
            first = Some(returned.folio().phys_addr());
            context.expect_eq(
                returned.valid(OFFSET + index as u64 * mm::PAGE_SIZE as u64),
                Ok(true),
            );
        }
        Ok(())
    }

    /// QueueStopped 只证明真实停止后的提交拒绝与 folio 失败唤醒，不外推硬件 EIO。
    fn failure(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        self.publish_one();
        self.publish_one();
        self.publish_one();
        until("failed file waiters did not block", || {
            self.observation.blocked(1) && self.observation.blocked(2)
        })?;

        self.disk
            .stop()
            .map_err(|_| TestError("cache failure real stop failed"))?;
        let before = self.disk.test_snapshot().submitted;
        self.observation.gate.complete();
        self.join_tasks()?;
        context.expect_eq(self.disk.test_snapshot().submitted, before);

        for result in &*self.observation.results.lock() {
            context.expect_eq(matches!(result, Some(Err(BlockError::QueueStopped))), true);
        }
        let folio = self.held[0].as_ref().unwrap().folio();
        context.expect_eq(folio.is_ready(), true);
        context.expect_eq(folio.wait_ready(), Err(BlockError::QueueStopped));
        context.expect_eq(
            folio.copy_to(0, &mut [0_u8; mm::PAGE_SIZE]),
            Err(BlockError::QueueStopped),
        );
        let (hit, owner) = self
            .mapping
            .get_or_prepare(64, None)
            .map_err(|_| TestError("failure hit disappeared"))?;
        context.expect_eq(hit.phys_addr(), folio.phys_addr());
        context.expect_eq(owner.is_none(), true);
        context.expect_eq(hit.wait_ready(), Err(BlockError::QueueStopped));
        context.diagnostic(format_args!(
            "cache-failure real-QueueStopped=1 hardware-EIO-covered=0"
        ));
        Ok(())
    }

    /// root 与 extra 各自验证 1/2/4KiB 短视图，身份不跨 Disk 合并。
    fn views(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let extra = self
            .extra_metadata
            .as_ref()
            .ok_or(TestError("extra metadata absent"))?;
        for size in [1024, 2048, 4096] {
            let offset = 33 * size as u64;
            let root = self
                .metadata
                .as_ref()
                .unwrap()
                .get(33, size, None)
                .map_err(|_| TestError("root metadata view read failed"))?;
            let second = extra
                .get(33, size, None)
                .map_err(|_| TestError("extra metadata view read failed"))?;
            context.expect_eq(root.len(), size);
            context.expect_eq(root.offset(), offset as usize % mm::PAGE_SIZE);
            context.expect_eq(
                root.with_read(|bytes| patterned(bytes, 1, offset, size)),
                Ok(true),
            );
            context.expect_eq(
                second.with_read(|bytes| patterned(bytes, 2, offset, size)),
                Ok(true),
            );
            context.expect_eq(root.folio().phys_addr() != second.folio().phys_addr(), true);
        }
        Ok(())
    }

    /// 裸页索引的唯一填充 owner 由实际 Disk 消费，不伪造 canonical inode 或文件 LRU。
    fn file_page(&self, index: u64) -> Result<FolioRef, BlockError> {
        let (folio, read) = self.mapping.get_or_prepare(index, None)?;
        if let Some(read) = read {
            self.disk.load_folio(
                index * mm::PAGE_SIZE as u64,
                read,
                Vec::new_in(BioAllocator::Heap),
            )?;
        }
        folio.wait_ready()?;
        Ok(folio)
    }

    /// 元数据预算满时拒绝活页 miss；文件稀疏索引保留任意已持页并正常增长。
    fn bounded(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        self.held[0] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(64, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("bounded first metadata read failed"))?,
        ));
        self.held[1] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(65, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("bounded second metadata read failed"))?,
        ));
        self.disk.test_flush_completions();
        context.expect_eq(
            matches!(
                self.metadata.as_ref().unwrap().get(66, mm::PAGE_SIZE, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );
        context.expect_eq(self.metadata.as_ref().unwrap().prune_unused(), 0);

        drop(self.held[0].take());
        self.held[0] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(66, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("bounded clean metadata victim did not retire"))?,
        ));
        context.expect_eq(self.metadata.as_ref().unwrap().len(), 2);
        context.expect_eq(
            self.held[0]
                .as_ref()
                .unwrap()
                .valid(66 * mm::PAGE_SIZE as u64),
            Ok(true),
        );
        drop(self.held[0].take());
        drop(self.held[1].take());
        self.disk.test_flush_completions();
        context.expect_eq(self.metadata.as_ref().unwrap().prune_unused(), 2);
        context.expect_eq(self.metadata.as_ref().unwrap().len(), 0);

        self.held[0] = Some(Returned::Folio(
            self.file_page(64)
                .map_err(|_| TestError("bounded first file read failed"))?,
        ));
        self.held[1] = Some(Returned::Folio(
            self.file_page(65)
                .map_err(|_| TestError("bounded second file read failed"))?,
        ));
        self.disk.test_flush_completions();
        self.held[2] =
            Some(Returned::Folio(self.file_page(66).map_err(|_| {
                TestError("sparse third live file page failed")
            })?));
        context.expect_eq(self.mapping.prune_unused(None), 0);
        for (index, held) in self.held.iter().flatten().enumerate() {
            context.expect_eq(
                held.valid((64 + index as u64) * mm::PAGE_SIZE as u64),
                Ok(true),
            );
        }

        drop(self.held[0].take());
        drop(self.held[1].take());
        drop(self.held[2].take());
        self.disk.test_flush_completions();
        context.expect_eq(self.mapping.prune_unused(None), 3);
        context.expect_eq(self.mapping.len(), 0);
        Ok(())
    }

    /// 预留控制槽后耗尽真实物理页；不在压力期间申请额外测试拥有存储。
    fn exhaust_pages(&mut self, context: &mut TestContext<'_>) -> TestResult {
        mm::drain_all_pages();
        for order in [10, 0] {
            loop {
                if self.pressure.len() == self.pressure.capacity() {
                    return Err(TestError("cache pressure handle capacity exhausted"));
                }
                let Some(pages) = mm::alloc_pages(order, false) else {
                    break;
                };
                self.pressure.push(pages);
            }
        }
        let stats = mm::stats();
        context.expect_eq(stats.buddy_free_pages + stats.pcp_cached_pages, 0);
        context.expect_eq(mm::alloc_pages(0, false).is_none(), true);
        Ok(())
    }

    /// 稀疏索引真实页 OOM 时 miss 保原错误；放压后只建立一个原页填充拥有者。
    fn reclaim_under_oom(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let index = 80;
        self.exhaust_pages(context)?;
        context.expect_eq(
            matches!(
                self.mapping.get_or_prepare(index, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );

        self.pressure.clear();
        let (folio, read) = self
            .mapping
            .get_or_prepare(index, None)
            .map_err(|_| TestError("sparse miss did not recover after releasing pressure"))?;
        let read = read.ok_or(TestError("recovered miss did not own its fill"))?;
        let (hit, duplicate) = self
            .mapping
            .get_or_prepare(index, None)
            .map_err(|_| TestError("recovered placeholder disappeared"))?;
        context.expect_eq(hit.phys_addr(), folio.phys_addr());
        context.expect_eq(duplicate.is_none(), true);
        context.expect_eq(folio.is_ready(), false);
        self.disk
            .load_folio(
                index * mm::PAGE_SIZE as u64,
                read,
                Vec::new_in(BioAllocator::Heap),
            )
            .map_err(|_| TestError("recovered original folio real read failed"))?;
        context.expect_eq(
            Returned::Folio(folio).valid(index * mm::PAGE_SIZE as u64),
            Ok(true),
        );
        Ok(())
    }

    /// 真实页 OOM 中已有 metadata/file hit 仍可读且保原 PFN，不依赖新的申请。
    fn oom(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        self.held[0] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(64, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("OOM warm metadata read failed"))?,
        ));
        self.held[1] = Some(Returned::Folio(
            self.file_page(64)
                .map_err(|_| TestError("OOM warm file read failed"))?,
        ));
        self.disk.test_flush_completions();

        self.exhaust_pages(context)?;
        let buffer = self
            .metadata
            .as_ref()
            .unwrap()
            .get(64, mm::PAGE_SIZE, None)
            .map_err(|_| TestError("metadata hit allocated under OOM"))?;
        context.expect_eq(
            buffer.folio().phys_addr(),
            self.held[0].as_ref().unwrap().folio().phys_addr(),
        );
        context.expect_eq(
            buffer.with_read(|bytes| patterned(bytes, 1, OFFSET, mm::PAGE_SIZE)),
            Ok(true),
        );
        let (folio, read) = self
            .mapping
            .get_or_prepare(64, None)
            .map_err(|_| TestError("file hit allocated under OOM"))?;
        context.expect_eq(read.is_none(), true);
        context.expect_eq(
            folio.phys_addr(),
            self.held[1].as_ref().unwrap().folio().phys_addr(),
        );
        context.expect_eq(Returned::Folio(folio).valid(OFFSET), Ok(true));
        context.expect_eq(
            matches!(
                self.metadata.as_ref().unwrap().get(65, mm::PAGE_SIZE, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );
        context.expect_eq(
            matches!(
                self.mapping.get_or_prepare(65, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );
        self.pressure.clear();
        Ok(())
    }

    /// 本 fixture 独占 producer，Disk 保活原控制区；只观察真实 avail.idx，不写 ring。
    fn avail_index(&self) -> u16 {
        // SAFETY: 地址来自此 Disk 接管前的原控制页及真实 RingLayout，u16 对齐；
        // 调用仅在 ready 成功且尚未 stop 的正文，fixture 无并发 stop，仅读 volatile 标量。
        u16::from_le(unsafe { core::ptr::read_volatile(self.avail_address as *const u16) })
    }

    /// 两个真实 file waiter 先 Blocked，CPU0 原控制 Task 再令正式准备失败且不发表请求。
    fn prepare_oom(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        self.publish_one();
        self.publish_one();
        self.publish_one();
        until("prepare-OOM file waiters did not block", || {
            self.observation.blocked(1) && self.observation.blocked(2)
        })?;
        let before = self.disk.test_snapshot();
        let avail = self.avail_index();
        self.observation.gate.complete();
        self.join_tasks()?;

        let evidence = self
            .observation
            .oom_evidence
            .lock()
            .as_ref()
            .copied()
            .ok_or(TestError("prepare-OOM allocator evidence absent"))?;
        context.expect_eq(evidence, (true, true));
        for result in &*self.observation.results.lock() {
            context.expect_eq(matches!(result, Some(Err(BlockError::NoMemory))), true);
        }
        let folio = self.held[0].as_ref().unwrap().folio();
        context.expect_eq(folio.is_ready(), true);
        context.expect_eq(folio.wait_ready(), Err(BlockError::NoMemory));
        context.expect_eq(
            folio.copy_to(0, &mut [0_u8; mm::PAGE_SIZE]),
            Err(BlockError::NoMemory),
        );
        let (hit, read) = self
            .mapping
            .get_or_prepare(64, None)
            .map_err(|_| TestError("prepare-OOM live placeholder disappeared"))?;
        context.expect_eq(hit.phys_addr(), folio.phys_addr());
        context.expect_eq(read.is_none(), true);
        context.expect_eq(hit.wait_ready(), Err(BlockError::NoMemory));
        let after = self.disk.test_snapshot();
        context.expect_eq(after.submitted, before.submitted);
        context.expect_eq(after.notifications, before.notifications);
        context.expect_eq(after.inflight, before.inflight);
        context.expect_eq(self.avail_index(), avail);
        context.diagnostic(format_args!(
            "cache-prepare-OOM real-page-NULL={} real-heap-NULL={} native-Blocked-waiters=2 original-PFN={} submitted-before={} submitted-after={} avail-before={} avail-after={} hardware-EIO-covered=0",
            evidence.0, evidence.1, folio.phys_addr(), before.submitted, after.submitted, avail, self.avail_index(),
        ));
        Ok(())
    }

    /// 私有 scratch 的 super 和 tiny 视图读取缺失范围，共页更新不能被后续 Read 覆盖。
    fn overlapping_updates(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        if 1024 < self.disk.limits().logical_block_size {
            return Err(TestError("overlap case requires logical block <= 1KiB"));
        }
        let cache = self.metadata.as_ref().unwrap();
        let claim = Disk::claim_fs(&BlockView::whole(self.disk.clone()), AccessMode::ReadWrite)
            .map_err(|_| TestError("private metadata RW claim failed"))?;
        let same = MetadataCache::shared(&claim, 8)
            .map_err(|_| TestError("shared same-Disk metadata cache disappeared"))?;
        context.expect_eq(MetadataCache::shared(&claim, 7).is_err(), true);
        let tiny = cache
            .get(1, 1024, None)
            .map_err(|_| TestError("private 1KiB metadata read failed"))?;
        let pfn = tiny.folio().phys_addr();
        context.expect_eq(tiny.folio().range_ready(1024, 1024), Ok(()));
        context.expect_eq(tiny.folio().range_ready(0, 1024), Err(BlockError::NotReady));
        context.expect_eq(
            tiny.folio().range_ready(2048, 2048),
            Err(BlockError::NotReady),
        );
        context.expect_eq(tiny.folio().is_ready(), false);
        tiny.update(&claim, |bytes| bytes.fill(0xa5))
            .map_err(|_| TestError("private tiny update failed"))?;
        context.expect_eq(tiny.folio().dirty_sectors(), 0b0000_1100);
        let ro = Disk::claim_fs(&BlockView::whole(self.disk.clone()), AccessMode::ReadOnly)
            .map_err(|_| TestError("private metadata RO claim failed"))?;
        context.expect_eq(
            tiny.update(&ro, |bytes| bytes.fill(0xee)),
            Err(BlockError::ReadOnly),
        );

        let two = same
            .get(0, 2048, None)
            .map_err(|_| TestError("private 2KiB metadata read failed"))?;
        context.expect_eq(two.folio().phys_addr(), pfn);
        context.expect_eq(
            two.with_read(|bytes| {
                bytes[..1024].iter().all(|b| *b == 0) && bytes[1024..].iter().all(|b| *b == 0xa5)
            }),
            Ok(true),
        );
        two.update(&claim, |bytes| bytes[..512].fill(0x19))
            .map_err(|_| TestError("private 2KiB metadata update failed"))?;

        let super_view = cache
            .get(0, mm::PAGE_SIZE, None)
            .map_err(|_| TestError("private super metadata read failed"))?;
        context.expect_eq(super_view.folio().phys_addr(), pfn);
        context.expect_eq(
            super_view.with_read(|bytes| {
                bytes[..512].iter().all(|b| *b == 0x19)
                    && bytes[512..1024].iter().all(|b| *b == 0)
                    && bytes[1024..2048].iter().all(|b| *b == 0xa5)
                    && bytes[2048..].iter().all(|b| *b == 0)
            }),
            Ok(true),
        );
        context.expect_eq(
            tiny.with_read(|bytes| bytes.iter().all(|b| *b == 0xa5)),
            Ok(true),
        );
        context.expect_eq(super_view.folio().dirty_sectors(), 0b0000_1111);
        context.expect_eq(super_view.folio().writeback_sectors(), 0);

        drop(super_view);
        drop(two);
        drop(tiny);
        drop(same);
        context.expect_eq(cache.prune_unused(), 0);
        context.expect_eq(cache.len(), 1);
        context.diagnostic(format_args!(
            "metadata-overlap private-scratch=1 physical-page=0 PFN={} windows=1/2/4KiB partial-not-ready=1 dirty-retained-after-read=1 dirty-prune-refused=1 CPU-content-only=1 file-write-fsync-covered=0",
            pfn,
        ));
        Ok(())
    }

    /// 从只读真实卷取两有效记录，在私有 scratch 同页用生产 codec 更新并复核完整 CRC。
    fn concurrent_records(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let volume = Ext4Volume::open(
            BlockView::whole(self.extra.as_ref().unwrap().clone()),
            AccessMode::ReadOnly,
            8,
        )
        .map_err(|_| TestError("record case real root open failed"))?;
        let g = volume.geometry();
        if g.inode_size != 256 || g.features_ro_compat & codec::METADATA_CSUM == 0 {
            return Err(TestError(
                "record case requires real 256-byte checksummed inode records",
            ));
        }
        let root = volume
            .load_inode(2, None)
            .map_err(|_| TestError("record root inode failed"))?;
        let pattern = volume
            .lookup(&InodeMetadata::from_data(&root), &root.block, b"pattern")
            .map_err(|_| TestError("record pattern lookup failed"))?
            .ok_or(TestError(
                "record case requires existing root pattern fixture",
            ))?;
        let source_cache = MetadataCache::shared(volume.claim(), 8)
            .map_err(|_| TestError("record source metadata cache unavailable"))?;
        let numbers = [root.number, pattern];
        let mut original = [[0u8; 256]; 2];
        let mut before = [InodeMetadata::from_data(&root); 2];
        for index in 0..2 {
            let number = numbers[index];
            let group = (number - 1) / g.inodes_per_group;
            let descriptor = group * g.descriptor_size as u64;
            let block = g.first_data_block + 1 + descriptor / g.block_size as u64;
            let offset = (descriptor % g.block_size as u64) as usize;
            // 主描述符属于保留元数据范围，按卷的原缓存读取，不能当作普通数据块校验。
            let table = source_cache
                .get(block, g.block_size, None)
                .map_err(|_| TestError("record group descriptor read failed"))?
                .with_read(|bytes| {
                    codec::decode_group(g, group, codec::bytes(bytes, offset, g.descriptor_size)?)
                })
                .map_err(|_| TestError("record descriptor view failed"))?
                .map_err(|_| TestError("record descriptor decode failed"))?;
            let byte = ((number - 1) % g.inodes_per_group) * g.inode_size as u64;
            let buffer = volume
                .metadata_block(table + byte / g.block_size as u64)
                .map_err(|_| TestError("record inode table read failed"))?;
            buffer
                .with_read(|bytes| {
                    original[index].copy_from_slice(
                        &bytes[(byte % g.block_size as u64) as usize
                            ..(byte % g.block_size as u64) as usize + 256],
                    );
                })
                .map_err(|_| TestError("record source copy failed"))?;
            let decoded = codec::decode_inode(g, number, &original[index])
                .map_err(|_| TestError("record source checksum failed"))?;
            before[index] = InodeMetadata::from_data(&decoded);
        }

        let mut after = before;
        for index in 0..2 {
            after[index].mode ^= 0o020;
            after[index].uid = 0x1234_abcd + index as u32;
            after[index].gid = 0xabcd_1234 + index as u32;
            after[index].atime.seconds += 7 + index as i64;
            codec::validate_inode_metadata_update(&before[index], &after[index])
                .map_err(|_| TestError("record expected metadata not representable"))?;
        }

        let claim = Disk::claim_fs(&BlockView::whole(self.disk.clone()), AccessMode::ReadWrite)
            .map_err(|_| TestError("private record RW claim failed"))?;
        let cache = self.metadata.as_ref().unwrap().clone();
        let buffer = cache
            .get(0, mm::PAGE_SIZE, None)
            .map_err(|_| TestError("private record scratch read failed"))?;
        buffer
            .update(&claim, |bytes| {
                bytes[..256].copy_from_slice(&original[0]);
                bytes[256..512].copy_from_slice(&original[1]);
            })
            .map_err(|_| TestError("private record initialization failed"))?;

        for index in 0..2 {
            let target = buffer.clone();
            let task_claim = claim.clone();
            let task_volume = volume.clone();
            let observer = self.observation.clone();
            let old = before[index];
            let next = after[index];
            self.prepared.push(
                task::prepare_kernel_on(index % cpu::count(), move || {
                    observer.tasks.lock()[index] = Some(task::current_task());
                    observer.entered[index].store(true, Ordering::Release);
                    match target.update(&task_claim, |bytes| {
                        codec::patch_inode_metadata(
                            task_volume.geometry(),
                            &mut bytes[index * 256..(index + 1) * 256],
                            &old,
                            &next,
                        )
                    }) {
                        Ok(Ok(())) => 0,
                        _ => 1,
                    }
                })
                .map_err(|_| TestError("record update Task preparation failed"))?,
            );
        }

        let content = buffer.folio().lock_content();
        self.publish_one();
        self.publish_one();
        let blocked = until(
            "record update tasks did not block on common content",
            || {
                self.observation.entered[0].load(Ordering::Acquire)
                    && self.observation.entered[1].load(Ordering::Acquire)
                    && self.observation.blocked(0)
                    && self.observation.blocked(1)
            },
        );
        drop(content);
        blocked?;
        self.join_tasks()?;

        let (decoded, roots_preserved, corrupt_rejected) = buffer
            .with_read(|bytes| {
                let mut decoded = [Err(FsError::Corrupt); 2];
                let mut roots_preserved = [false; 2];
                let mut corrupt_rejected = [false; 2];
                for index in 0..2 {
                    let record = &bytes[index * 256..(index + 1) * 256];
                    decoded[index] = codec::decode_inode(g, numbers[index], record)
                        .map(|inode| InodeMetadata::from_data(&inode));
                    roots_preserved[index] = record[40..100] == original[index][40..100];
                    let mut corrupt = [0u8; 256];
                    corrupt.copy_from_slice(record);
                    corrupt[255] ^= 1;
                    corrupt_rejected[index] =
                        codec::decode_inode(g, numbers[index], &corrupt).is_err();
                }
                (decoded, roots_preserved, corrupt_rejected)
            })
            .map_err(|_| TestError("record final common view failed"))?;
        for index in 0..2 {
            context.expect_eq(decoded[index], Ok(after[index]));
            context.expect_eq(roots_preserved[index], true);
            context.expect_eq(corrupt_rejected[index], true);
        }

        for index in 0..2 {
            let unchanged = volume
                .load_inode(numbers[index], None)
                .map_err(|_| TestError("record original root reread failed"))?;
            context.expect_eq(InodeMetadata::from_data(&unchanged), before[index]);
        }
        let pfn = buffer.folio().phys_addr();
        drop(buffer);
        context.expect_eq(cache.prune_unused(), 0);
        context.diagnostic(format_args!(
            "metadata-records private-scratch=1 PFN={} native-blocked=2 shared-page-records=2 production-codec=1 full-CRC=1 root-unchanged=1 file-write-fsync-covered=0",
            pfn,
        ));
        Ok(())
    }

    /// 正常和主体失败共用真实静止化：停未发表 producer、放 gate/read、恢复 used、stop/join/Work 尾。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.observation.cancelled.store(true, Ordering::Release);
        self.observation.gate.complete();
        drop(core::mem::take(&mut self.prepared));
        self.pressure.clear();
        let pressure = self.observation.pressure.lock().take();
        drop(pressure);
        self.disk.test_resume_completions();
        if let Some(extra) = &self.extra {
            extra.test_resume_completions();
        }

        if let Some(join) = self.joins.first() {
            until("cache read owner did not leave stack", || {
                join.try_join().is_some()
            })?;
        }
        let read = self.observation.read.lock().take();
        drop(read);
        self.join_tasks()?;
        self.disk.test_flush_completions();
        if let Some(extra) = &self.extra {
            extra.test_flush_completions();
        }

        self.held.fill_with(|| None);
        let results =
            core::mem::replace(&mut *self.observation.results.lock(), [const { None }; 3]);
        drop(results);
        if self.private_metadata {
            // SAFETY: 本例只有私有 scratch metadata，未构造 FS/inode 或发表任何写；
            // 已取消并 join 全部参与者、结束真实 Read/Work，外部 Buffer/Folio 均已释放。
            // discard_private 先关闭新取得并真实 detach；没有用户映射或 WB，CPU 修改无需落盘。
            unsafe { self.metadata.as_ref().unwrap().discard_private() }
                .map_err(|_| TestError("private metadata detach retained a live owner"))?;
        }
        drop(self.metadata.take());
        drop(self.extra_metadata.take());

        self.disk
            .stop()
            .map_err(|_| TestError("cache stop cannot converge; retaining original DMA owner"))?;
        if let Some(extra) = &self.extra {
            extra
                .stop()
                .map_err(|_| TestError("extra cache stop cannot converge"))?;
        }
        self.queue.close_sync();
        context.expect_eq(self.disk.test_snapshot().inflight, 0);
        if let Some(extra) = &self.extra {
            context.expect_eq(extra.test_snapshot().inflight, 0);
        }
        Ok(())
    }

    /// stop、IRQ/Work 同步与真实 join 已成功后锁外销毁原引用，核完整六域基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        recovered(context, baseline)
    }
}

macro_rules! fixture {
    ($name:ident, $scenario:ident) => {
        struct $name(Owner);
        impl Fixture<DeviceInventory> for $name {
            /// 每例只借组库存，独立准备真实设备、控制存储与 Task，再发表硬件。
            fn set_up(
                _: &mut TestContext<'_>,
                inventory: &DeviceInventory,
            ) -> Result<Self, TestError> {
                Owner::prepare(Scenario::$scenario, inventory).map(Self)
            }
            /// 具体 owner 无法收敛时返回错误，typed case-abort 保留原 fixture。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// 仅在所有实际异步责任结束后消费 fixture 并结六域账。
            fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
                self.0.tear_down(context)
            }
        }
    };
}
fixture!(Views, Views);
fixture!(MetadataFlight, MetadataFlight);
fixture!(FileFlight, FileFlight);
fixture!(Independent, Independent);
fixture!(MetadataIndependent, MetadataIndependent);
fixture!(Failure, Failure);
fixture!(Bounded, Bounded);
fixture!(Oom, Oom);
fixture!(PrepareOom, PrepareOom);
fixture!(MutableMetadata, MutableMetadata);
fixture!(MetadataRecords, MetadataRecords);

#[ktest::suite(fixture = Pages, context = "Thread", tags("cache", "folio"), timeout_ms = 15000)]
mod folio {
    use super::*;
    /// 同 PFN 原引用、末引用归页与高阶拒绝保原块。
    #[test]
    fn original_page_lifetime(context: &mut TestContext<'_>, fixture: &mut Pages) -> TestResult {
        fixture.identity(context)
    }

    /// 有序、逆序和大跨度键保留各自原页，真实 OOM 命中与外部 pin 后的两次 prune 共享原拥有。
    #[test]
    fn sparse_index_insert_hit_and_prune(
        context: &mut TestContext<'_>,
        _: &mut Pages,
    ) -> TestResult {
        for pass in 0..3 {
            let mapping = FolioIndex::new(None, false);
            let key = |index: usize| match pass {
                0 => index as u64,
                1 => (95 - index) as u64,
                _ if index == 95 => u64::MAX,
                _ => (1u64 << 40) + index as u64 * 8191,
            };
            let mut addresses = [0usize; 96];
            for (index, address) in addresses.iter_mut().enumerate() {
                let (folio, read) = mapping
                    .get_or_prepare(key(index), None)
                    .map_err(|_| TestError("sparse AVL insertion failed"))?;
                let mut read = read.ok_or(TestError("sparse miss did not own initialization"))?;
                for byte in read.bytes_mut() {
                    byte.write(index as u8);
                }
                // SAFETY: 本 owner 从未交给设备；其整个页范围已由上面循环初始化，
                // 没有任何 DMA/CPU 初始化别名，complete 后不再使用原可写范围。
                unsafe {
                    read.complete(IoResult {
                        completed: mm::PAGE_SIZE,
                        error: None,
                    });
                }
                *address = folio.phys_addr();
            }

            if pass == 0 {
                let mut pressure = HeapPressure::prepare()?;
                let evidence = pressure.exhaust();
                let result = mapping.get_or_prepare(key(48), None);
                drop(pressure);
                context.expect_eq(evidence, (true, true));
                let (hit, read) =
                    result.map_err(|_| TestError("sparse hit allocated under real OOM"))?;
                context.expect_eq(hit.phys_addr(), addresses[48]);
                context.expect_eq(read.is_none(), true);
            }
            for (index, address) in addresses.iter().enumerate() {
                let (hit, read) = mapping
                    .get_or_prepare(key(index), None)
                    .map_err(|_| TestError("sparse AVL hit lost original page"))?;
                context.expect_eq(hit.phys_addr(), *address);
                context.expect_eq(read.is_none(), true);
                let mut byte = [0u8; 1];
                context.expect_eq(hit.copy_to(0, &mut byte), Ok(()));
                context.expect_eq(byte[0], index as u8);
            }

            let first = mapping
                .get_or_prepare(key(0), None)
                .map_err(|_| TestError("sparse first pin failed"))?
                .0;
            let middle = mapping
                .get_or_prepare(key(48), None)
                .map_err(|_| TestError("sparse middle pin failed"))?
                .0;
            let last = mapping
                .get_or_prepare(key(95), None)
                .map_err(|_| TestError("sparse last pin failed"))?
                .0;
            context.expect_eq(mapping.prune_unused(None), 93);
            drop(first);
            drop(middle);
            drop(last);
            context.expect_eq(mapping.prune_unused(None), 3);
        }
        context.diagnostic(format_args!(
            "sparse-index orders=ascending/descending/sparse pages-per-order=96 real-OOM-hit=1 same-PFN-and-content=1 prune-live=93 prune-final=3"
        ));
        Ok(())
    }
}

#[ktest::suite(fixture = Views, suite_resources = DeviceInventory, context = "Thread", tags("cache", "metadata"), resources("root", "extra"), timeout_ms = 15000)]
mod metadata_views {
    use super::*;
    /// 两个真实设备各自提供完整 1/2/4KiB 非零短视图。
    #[test]
    fn complete_pattern_1_2_4k(context: &mut TestContext<'_>, fixture: &mut Views) -> TestResult {
        fixture.0.views(context)
    }
}

#[ktest::suite(fixture = MetadataFlight, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "metadata", "B14"), resources("root"), timeout_ms = 15000)]
mod metadata_singleflight {
    use super::*;
    /// 同 key 唯一真实提交，原生等待者 Blocked 后读同 PFN。
    #[test]
    fn blocked_waiter_one_submit(
        context: &mut TestContext<'_>,
        fixture: &mut MetadataFlight,
    ) -> TestResult {
        fixture.0.singleflight(context)
    }
}

#[ktest::suite(fixture = FileFlight, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "file", "B14"), resources("root"), timeout_ms = 15000)]
mod file_singleflight {
    use super::*;
    /// 唯一 file read owner 由实际 Disk 填充，同 key waiter 复用原页。
    #[test]
    fn blocked_waiter_one_submit(
        context: &mut TestContext<'_>,
        fixture: &mut FileFlight,
    ) -> TestResult {
        fixture.0.singleflight(context)
    }
}

#[ktest::suite(fixture = MetadataIndependent, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "metadata", "B15"), resources("root"), timeout_ms = 15000)]
mod metadata_parallel {
    use super::*;
    /// 元数据不同 key 同时持有真实设备请求，各自消费完整图样。
    #[test]
    fn independent_keys_two_inflight(
        context: &mut TestContext<'_>,
        fixture: &mut MetadataIndependent,
    ) -> TestResult {
        fixture.0.independent(context)
    }
}

#[ktest::suite(fixture = Independent, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "file", "B15"), resources("root"), timeout_ms = 15000)]
mod file_parallel {
    use super::*;
    /// 两个独立 key 真正在途，恢复真实 used 后各读完整图样。
    #[test]
    fn independent_keys_two_inflight(
        context: &mut TestContext<'_>,
        fixture: &mut Independent,
    ) -> TestResult {
        fixture.0.independent(context)
    }
}

#[ktest::suite(fixture = Failure, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "file", "QueueStopped", "B11", "B14"), resources("root"), timeout_ms = 15000)]
mod stopped_failure {
    use super::*;
    /// 真实 stopped Disk 拒绝 read owner，两个原生 waiter 同错误醒来且内容不可读。
    #[test]
    fn stopped_read_wakes_all(context: &mut TestContext<'_>, fixture: &mut Failure) -> TestResult {
        fixture.0.failure(context)
    }
}

#[ktest::suite(fixture = Bounded, suite_resources = DeviceInventory, context = "Thread", tags("cache", "prune", "B23"), resources("root"), timeout_ms = 15000)]
mod bounded {
    use super::*;
    /// 元数据活页拒预算外 miss，文件稀疏索引保留三个活页及完整内容。
    #[test]
    fn clean_prune_and_live_no_memory(
        context: &mut TestContext<'_>,
        fixture: &mut Bounded,
    ) -> TestResult {
        fixture.0.bounded(context)
    }
}

#[ktest::suite(fixture = Oom, suite_resources = DeviceInventory, context = "Thread", tags("cache", "oom", "B23"), resources("root"), timeout_ms = 15000)]
mod oom {
    use super::*;
    /// 真实物理页耗尽时旧 hit 仍读取原数据，新 miss 保 NoMemory。
    #[test]
    fn existing_hits_read_under_real_oom(
        context: &mut TestContext<'_>,
        fixture: &mut Oom,
    ) -> TestResult {
        fixture.0.oom(context)
    }

    /// 真实 OOM 保 miss 错误，放压后只建立一个原页填充拥有者。
    #[test]
    fn sparse_miss_recovers_after_real_oom(
        context: &mut TestContext<'_>,
        fixture: &mut Oom,
    ) -> TestResult {
        fixture.0.reclaim_under_oom(context)
    }
}

#[ktest::suite(fixture = PrepareOom, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2,
    tags("cache", "file", "prepare-oom", "B14"), resources("root"), timeout_ms = 60000)]
mod prepare_oom {
    use super::*;
    /// 正式准备真 OOM 不发表请求，两个同页 waiter 取得原 NoMemory。
    #[test]
    fn unsubmitted_read_wakes_all(
        context: &mut TestContext<'_>,
        fixture: &mut PrepareOom,
    ) -> TestResult {
        fixture.0.prepare_oom(context)
    }
}

#[ktest::suite(fixture = MutableMetadata, suite_resources = DeviceInventory, context = "Thread",
    tags("cache", "metadata"), resources("scratch"), timeout_ms = 15000)]
mod mutable_metadata {
    use super::*;
    /// 实际 scratch Read 补缺失 sector，1/2/4KiB 视图共 PFN 且保既有 CPU 修改。
    #[test]
    fn overlapping_partial_views_preserve_updates(
        context: &mut TestContext<'_>,
        fixture: &mut MutableMetadata,
    ) -> TestResult {
        fixture.0.overlapping_updates(context)
    }
}

#[ktest::suite(fixture = MetadataRecords, suite_resources = DeviceInventory, context = "Thread",
    tags("cache", "metadata"), resources("root", "scratch"), timeout_ms = 15000)]
mod metadata_records {
    use super::*;
    /// 两实际生产 inode 记录复制到私有页，由原生任务互斥更新并以生产解码器核完整 CRC。
    #[test]
    fn common_page_record_updates_keep_both_checksums(
        context: &mut TestContext<'_>,
        fixture: &mut MetadataRecords,
    ) -> TestResult {
        fixture.0.concurrent_records(context)
    }
}
