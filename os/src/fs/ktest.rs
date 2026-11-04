//! 真实挂载、canonical inode、原文件页与持续源的具名消费回归。

use ::ktest::{self, Fixture, Resource, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::block::{self, AccessMode, BlockError, BlockView, Disk, DiskIdentity, DiskRef};
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::DeviceInventory;
use crate::fs::ext4::{codec, Ext4Volume};
use crate::fs::inode::{InodeCache, InodeCacheWatch, InodeRef, InvalidateGuard};
use crate::fs::{
    DentryRef, DescriptorFlags, FileRef, FileTable, FsContext, FsError, InodeKind, OpenOptions,
    Path, ResolveMode, Vfs,
};
use crate::mm::{FolioRead, FolioRef};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, JoinHandle, PreparedTask, State, Task};
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, drivers, mm, sched, time, timer};

mod concurrency;
mod epoll;
mod file;
mod format;
mod mem_mapping;
mod path;
mod pipe;
mod poll_source;
mod pseudo;
mod system;
pub(crate) mod tty;
mod user;

const PATTERN_SIZE: u64 = 65573;
const DIRECTORY_ENTRIES: usize = 12000;
const DEEP_EXTENTS: u64 = 1500;

/// 保留格式、inode、路径、文件、持续源与伪文件系统的实际宏锚点，不维护中央用例列表。
pub(crate) fn anchors() -> (
    [&'static ::ktest::SuiteSlot; 31],
    [&'static ::ktest::SuiteSlot; 7],
    [&'static ::ktest::SuiteSlot; 9],
    [&'static ::ktest::SuiteSlot; 6],
) {
    (
        [
            format::codec::__ktest_anchor(),
            format::volume::__ktest_anchor(),
            format::identity::__ktest_anchor(),
            format::bounded::__ktest_anchor(),
            format::oom::__ktest_anchor(),
            format::bad_super::__ktest_anchor(),
            format::bad_group::__ktest_anchor(),
            format::bad_inode::__ktest_anchor(),
            format::bad_extent::__ktest_anchor(),
            format::bad_directory::__ktest_anchor(),
            format::bad_dx::__ktest_anchor(),
            format::bad_directory_layout::__ktest_anchor(),
            format::bad_dx_layout::__ktest_anchor(),
            format::bad_geometry::__ktest_anchor(),
            format::bad_features::__ktest_anchor(),
            format::bad_mapping::__ktest_anchor(),
            format::prefix::__ktest_anchor(),
            concurrency::inode_loading::__ktest_anchor(),
            concurrency::file_loading::__ktest_anchor(),
            concurrency::failed_loading::__ktest_anchor(),
            user::anchor(),
            user::write_anchor(),
            user::namespace_anchor(),
            user::readahead_anchor(),
            user::background_anchor(),
            poll_source::source::__ktest_anchor(),
            pseudo::anchors()[0],
            mem_mapping::anchors()[0],
            pipe::anchor(),
            tty::anchor(),
            epoll::anchor(),
        ],
        path::anchors(),
        file::anchors(),
        system::anchors(),
    )
}

/// 有限等待仅观察真实状态，sleep 不充当 Blocked、IRQ 或离栈证据。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(10))
        .ok_or(TestError("filesystem observation deadline overflow"))?;
    while !ready() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 无参与者使用资源时整理 SLUB/PCP，保留框架与共享服务的常驻六域。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("filesystem resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::flush_all();
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 销毁原拥有者后逐字段等原六域基线，不重置统计或掩盖余留资源。
fn recovered(
    context: &mut TestContext<'_>,
    label: &'static str,
    before: mm::MemoryStats,
) -> TestResult {
    let after = settled()?;
    context.diagnostic(format_args!(
        "fs-resources boundary={} six-domains-before={:?} six-domains-after={:?}",
        label, before, after
    ));
    if after != before {
        return Err(TestError(label));
    }
    context.expect_eq(after, before);
    Ok(())
}

/// 比较文件逻辑偏移上的完整外部图样，零数据和短读取都不能代替成功。
fn patterned(bytes: &[u8], offset: u64) -> bool {
    bytes.iter().enumerate().all(|(index, byte)| {
        let at = offset + index as u64;
        *byte
            == 37u8
                .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                .wrapping_add((at % 512) as u8)
    })
}

/// 将真实文件系统错误保留在诊断中，同时返回框架静态失败消息。
fn checked<T>(context: &TestContext<'_>, result: Result<T, FsError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("fs-operation error={:?}", error));
        TestError("filesystem operation failed")
    })
}

/// 调用者持同 inode 的 invalidate 共享侧及 NOFS 范围，读入凭证保原 inode 到数据全尾。
fn fill_file_page(
    inode: &InodeRef,
    index: u64,
    read: FolioRead,
    _invalidate: &InvalidateGuard<'_>,
) -> Result<(), FsError> {
    match crate::fs::writeback::prepare_file_targets(inode, index, &(0..u64::MAX), None) {
        Ok((size, targets)) => inode.disk_volume()?.fill_page(index, size, &targets, read),
        Err(error) => Err(crate::fs::ext4::finish_error(read, error)),
    }
}

impl Fixture for DeviceInventory {
    /// 整组用例只发现一次真实库存，并在首例基线前等待常驻工作服务进入阻塞稳态。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let inventory = drivers::discover().map_err(|error| {
            context.diagnostic(format_args!("fs-discovery error={:?}", error));
            TestError("filesystem discovery failed")
        })?;
        crate::workqueue::testing::wait_services_blocked();
        Ok(inventory)
    }
}

/// 每例借用整组保活的真实库存，上一例完成 reset、IRQ 与 Work 后才重新准备独占设备。
fn devices(
    context: &TestContext<'_>,
    inventory: &DeviceInventory,
    count: usize,
) -> Result<Vec<VirtioDevice>, TestError> {
    let pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    if inventory.mmio_blocks.len() + pci.clone().count() != count {
        return Err(TestError(
            "filesystem attached disks differ from declared roles",
        ));
    }
    let mut prepared = Vec::new();
    prepared
        .try_reserve_exact(count)
        .map_err(|_| TestError("filesystem device slots OOM"))?;
    for candidate in &inventory.mmio_blocks {
        prepared.push(
            // SAFETY: 框架串行分派用例，前例已完成 stop/reset/IRQ/Work；本组库存与永久映射
            // 保活到全部用例结束，本例独占该候选的传输和控制页。
            unsafe { VirtioDevice::prepare_mmio(candidate) }.map_err(|error| {
                context.diagnostic(format_args!("fs-MMIO-prepare error={:?}", error));
                TestError("filesystem MMIO preparation failed")
            })?,
        );
    }
    for device in pci {
        prepared.push(
            // SAFETY: BDF 借自本组真实库存；前例已归还设备访问权，本例独占传输并保活
            // 到 reset、IRQ 和完成 Work 全尾，库存由外层 suite 继续拥有。
            unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }.map_err(|error| {
                context.diagnostic(format_args!("fs-PCI-prepare error={:?}", error));
                TestError("filesystem PCI preparation failed")
            })?,
        );
    }
    Ok(prepared)
}

enum Job {
    Get {
        cache: Arc<InodeCache>,
        volume: Arc<Ext4Volume>,
        number: u64,
    },
    Read {
        inode: InodeRef,
        page: u64,
    },
    Fill {
        inode: InodeRef,
        page: u64,
    },
    Oom {
        cache: Arc<InodeCache>,
        volume: Arc<Ext4Volume>,
        inode: InodeRef,
        empty_number: u64,
        pressure: crate::cache::ktest::HeapPressure,
    },
    Lookup {
        vfs: Arc<Vfs>,
        parent: DentryRef,
        name: &'static [u8],
    },
    Resolve {
        vfs: Arc<Vfs>,
        context: FsContext,
        pathname: &'static [u8],
        mode: ResolveMode,
    },
    LookupOom {
        vfs: Arc<Vfs>,
        parent: DentryRef,
        cached: DentryRef,
        name: &'static [u8],
        pressure: crate::cache::ktest::HeapPressure,
    },
    FileRead {
        table: Arc<FileTable>,
        fd: usize,
        len: usize,
        expected_offset: u64,
    },
    FileDupExact {
        table: Arc<FileTable>,
        source: usize,
        target: usize,
    },
    FileOom {
        table: Arc<FileTable>,
        vfs: Arc<Vfs>,
        context: FsContext,
        fd: usize,
        cached: FileRef,
        pathname: &'static [u8],
        pressure: crate::cache::ktest::HeapPressure,
    },
    Append {
        file: FileRef,
        gate: Arc<Completion>,
        marker: u8,
        records: usize,
    },
    WriteOom {
        file: FileRef,
        pressure: crate::cache::ktest::HeapPressure,
    },
    User {
        thread: Arc<crate::task::UserThread>,
        access: crate::arch::UserAccess,
    },
}

enum Returned {
    Written(usize),
    WriteOom {
        evidence: (bool, bool),
        warm: Result<usize, FsError>,
        cold: Result<usize, FsError>,
    },
    Inode(InodeRef),
    Dentry(DentryRef),
    Path(Path),
    Page {
        folio: FolioRef,
        bytes: usize,
        valid: bool,
    },
    Oom {
        producer_cpu: usize,
        evidence: (bool, bool),
        same_inode: bool,
        same_mapping: bool,
        bytes: usize,
        valid: bool,
        inode_error: Option<FsError>,
        page_error: Option<FsError>,
        address: usize,
        submitted_before: usize,
        submitted_after: usize,
    },
    LookupOom {
        producer_cpu: usize,
        evidence: (bool, bool),
        same_dentry: bool,
        error: Option<FsError>,
        len_before: usize,
        len_after: usize,
        submitted_before: usize,
        submitted_after: usize,
    },
    FileRead {
        bytes: usize,
        valid: bool,
    },
    FileDescriptor(usize),
    FileOom {
        producer_cpu: usize,
        evidence: (bool, bool),
        same_ofd: bool,
        bytes: usize,
        valid: bool,
        new_error: Option<FsError>,
        reserved_fd: usize,
        reused_fd: usize,
        growth_errors: [Option<FsError>; 2],
        growth_reused_fd: usize,
        growth_initial_slots: usize,
        submitted_before: usize,
        submitted_after: usize,
    },
    User(Result<file::VmAccessResult, TestError>),
}

struct Observation {
    cancelled: AtomicBool,
    gate: Completion,
    hold_loader: AtomicBool,
    loader_entered: AtomicUsize,
    entered: [AtomicBool; 3],
    tasks: SpinLock<[Option<Arc<Task>>; 3]>,
    jobs: SpinLock<[Option<Job>; 3]>,
    results: SpinLock<[Option<Result<Returned, FsError>>; 3]>,
}

impl Observation {
    /// 每例预备自己的 gate、任务和结果槽，异步正文无需再分配控制存储。
    fn prepare() -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            cancelled: AtomicBool::new(false),
            gate: Completion::new(),
            hold_loader: AtomicBool::new(false),
            loader_entered: AtomicUsize::new(0),
            entered: [const { AtomicBool::new(false) }; 3],
            tasks: SpinLock::new([const { None }; 3]),
            jobs: SpinLock::new([const { None }; 3]),
            results: SpinLock::new([const { None }; 3]),
        })
        .map_err(|_| TestError("filesystem observation OOM"))
    }

    /// 正式调用入口已经进入且生产调度器报告 Blocked，才算被测源的等待者。
    fn blocked(&self, index: usize) -> bool {
        if !self.entered[index].load(Ordering::Acquire) {
            return false;
        }
        let task = self.tasks.lock()[index].clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

/// 普通原生任务在 invalidate 外准备候选，保护内发表；FILL 停点和取消保原加载结果。
fn perform(job: Job, observer: &Observation) -> Result<Returned, FsError> {
    let cancelled = &observer.cancelled;
    match job {
        Job::Append {
            file,
            gate,
            marker,
            records,
        } => {
            gate.wait();
            let mut bytes = [marker; 512];
            let mut accepted = 0;
            for index in 0..records {
                if cancelled.load(Ordering::Acquire) {
                    return Err(FsError::LoadAborted);
                }
                bytes[1] = index as u8;
                let count = file.write(&bytes)?;
                if count != bytes.len() {
                    return Err(FsError::Corrupt);
                }
                accepted += count;
            }
            Ok(Returned::Written(accepted))
        }
        Job::WriteOom { file, mut pressure } => {
            let evidence = {
                let _pin = crate::sync::PreemptGuard::new();
                pressure.exhaust()
            };
            let warm = file.write_at(0, &[0x6d; 17]);
            let cold = file.write_at(4096, &[0x9b; 17]);
            drop(pressure);
            Ok(Returned::WriteOom {
                evidence,
                warm,
                cold,
            })
        }
        Job::User { thread, access } => Ok(Returned::User(file::run_vm_access(
            thread, access, cancelled,
        ))),
        Job::FileRead {
            table,
            fd,
            len,
            expected_offset,
        } => {
            if mm::PAGE_SIZE < len {
                return Err(FsError::InvalidOperation);
            }
            let mut data = [0xa7; mm::PAGE_SIZE];
            let bytes = table.read(fd, &mut data[..len])?;
            Ok(Returned::FileRead {
                bytes,
                valid: patterned(&data[..bytes], expected_offset),
            })
        }
        Job::FileDupExact {
            table,
            source,
            target,
        } => table
            .dup_exact(source, target, 1024)
            .map(Returned::FileDescriptor),
        Job::FileOom {
            table,
            vfs,
            context,
            fd,
            cached,
            pathname,
            mut pressure,
        } => {
            let root = vfs.root();
            let disk = root.inode()?.disk_volume()?.disk();
            let submitted_before = disk.test_snapshot().submitted;
            let growth = FileTable::try_new()?;
            let base = growth
                .reserve(4097)?
                .publish(cached.clone(), DescriptorFlags::default());
            let initial_slots = growth.allocated_slots();
            for _ in 1..initial_slots {
                growth.dup(base, 4097)?;
            }
            let evidence = pressure.exhaust();
            let growth_errors = [
                growth.reserve(4097).err(),
                growth.dup_exact(base, 4096, 4097).err(),
            ];
            let same = table.lookup(fd)?;
            let mut data = [0xa7; mm::PAGE_SIZE];
            let bytes = same.read_at(0, &mut data)?;
            let reservation = table.reserve(1024)?;
            let reserved_fd = reservation.publish(cached.clone(), DescriptorFlags::default());
            let published = table.lookup(reserved_fd)?;
            let same_ofd = Arc::ptr_eq(&same, &cached) && Arc::ptr_eq(&published, &cached);
            table.close(reserved_fd)?;
            let new_error = table
                .open(&vfs, &context, pathname, OpenOptions::default(), 1024)
                .err();
            let reservation = table.reserve(1024)?;
            let reused_fd = reservation.fd();
            drop(reservation);
            drop(pressure);
            let growth_reused_fd = growth.reserve(4097)?.fd();
            let returned = Returned::FileOom {
                producer_cpu: cpu::current().id,
                evidence,
                same_ofd,
                bytes,
                valid: patterned(&data[..bytes], 0),
                new_error,
                reserved_fd,
                reused_fd,
                growth_errors,
                growth_reused_fd,
                growth_initial_slots: initial_slots,
                submitted_before,
                submitted_after: disk.test_snapshot().submitted,
            };
            Ok(returned)
        }
        Job::Lookup { vfs, parent, name } => vfs
            .dcache()
            .lookup(vfs.inodes(), &parent, name)
            .map(Returned::Dentry),
        Job::Resolve {
            vfs,
            context,
            pathname,
            mode,
        } => vfs.resolve(&context, pathname, mode).map(Returned::Path),
        Job::LookupOom {
            vfs,
            parent,
            cached,
            name,
            mut pressure,
        } => {
            let disk = parent.inode()?.disk_volume()?.disk();
            let submitted_before = disk.test_snapshot().submitted;
            let len_before = vfs.dcache().len();
            let evidence = pressure.exhaust();
            let same = vfs
                .dcache()
                .lookup(vfs.inodes(), &parent, cached.name().as_slice())?;
            let error = vfs.dcache().lookup(vfs.inodes(), &parent, name).err();
            let returned = Returned::LookupOom {
                producer_cpu: cpu::current().id,
                evidence,
                same_dentry: same.ptr_eq(&cached),
                error,
                len_before,
                len_after: vfs.dcache().len(),
                submitted_before,
                submitted_after: disk.test_snapshot().submitted,
            };
            drop(pressure);
            Ok(returned)
        }
        Job::Get {
            cache,
            volume,
            number,
        } => cache.iget(&volume, number).map(Returned::Inode),
        Job::Read { inode, page } => {
            let offset = page
                .checked_mul(mm::PAGE_SIZE as u64)
                .ok_or(FsError::Corrupt)?;
            let mut bytes = [0xa7; mm::PAGE_SIZE];
            let count = inode.read_at(offset, &mut bytes)?;
            let _invalidate = inode.lock_invalidate()?;
            let folio = inode
                .mapping()
                .cached_folio(page)
                .ok_or(FsError::Corrupt)?
                .with_file_owner(inode.clone());
            folio.wait_ready()?;
            Ok(Returned::Page {
                folio,
                bytes: count,
                valid: patterned(&bytes[..count], offset),
            })
        }
        Job::Fill { inode, page } => {
            let mut attempt = mm::reclaim::ReclaimAttempt::new();
            let mut prepared = Some(inode.mapping().prepare_file(&inode, page, &mut attempt)?);
            let result = (|| {
                let invalidate = inode.lock_invalidate()?;
                if cancelled.load(Ordering::Acquire) {
                    return Err(FsError::LoadAborted);
                }
                let (folio, read) = inode
                    .mapping()
                    .publish_file(&inode, page, &mut prepared)?
                    .ok_or(FsError::Corrupt)?;
                let folio = folio.with_file_owner(inode.clone());
                let read = read.ok_or(FsError::Corrupt)?.with_file_owner(inode.clone());
                if observer.hold_loader.load(Ordering::Acquire) {
                    observer.loader_entered.fetch_add(1, Ordering::AcqRel);
                    observer.gate.wait();
                }
                if cancelled.load(Ordering::Acquire) {
                    return Err(crate::fs::ext4::finish_error(
                        read,
                        FsError::Io(BlockError::QueueStopped),
                    ));
                }
                fill_file_page(&inode, page, read, &invalidate)?;
                folio.wait_ready()?;
                let mut bytes = [0xa7; mm::PAGE_SIZE];
                folio.copy_to(0, &mut bytes)?;
                let valid = patterned(&bytes, page * mm::PAGE_SIZE as u64);
                Ok(Returned::Page {
                    folio,
                    bytes: mm::PAGE_SIZE,
                    valid,
                })
            })();
            drop(prepared);
            result
        }
        Job::Oom {
            cache,
            volume,
            inode,
            empty_number,
            mut pressure,
        } => {
            let submitted_before = volume.disk().test_snapshot().submitted;
            let evidence = pressure.exhaust();
            let same = cache.iget(&volume, inode.number())?;
            let mut data = [0xa7; mm::PAGE_SIZE];
            let bytes = same.read_at(0, &mut data)?;
            let valid = patterned(&data[..bytes], 0);
            let inode_error = cache.iget(&volume, empty_number).err();
            let page_error = inode.read_at(mm::PAGE_SIZE as u64, &mut data).err();
            let folio = inode
                .mapping()
                .cached_folio(0)
                .ok_or(FsError::Corrupt)?
                .with_file_owner(inode.clone());
            folio.wait_ready()?;
            let returned = Returned::Oom {
                producer_cpu: cpu::current().id,
                evidence,
                same_inode: same.ptr_eq(&inode),
                same_mapping: core::ptr::eq(same.mapping(), inode.mapping()),
                bytes,
                valid,
                inode_error,
                page_error,
                address: folio.phys_addr(),
                submitted_before,
                submitted_after: volume.disk().test_snapshot().submitted,
            };
            drop(pressure);
            Ok(returned)
        }
    }
}

struct Owner {
    initial: mm::MemoryStats,
    filesystem_baseline: mm::MemoryStats,
    disks: Vec<DiskRef>,
    queue: WorkQueue,
    cache: Option<Arc<InodeCache>>,
    cache_watch: Option<InodeCacheWatch>,
    observation: Option<Arc<Observation>>,
    pressure: Option<crate::cache::ktest::HeapPressure>,
    prepared: [Option<PreparedTask>; 3],
    joins: Vec<JoinHandle>,
    activation_error: Option<BlockError>,
}

impl Owner {
    /// 库存与常驻服务已由整组预备，每例建立自己的资源与两层完整基线后才发表设备。
    fn prepare(
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        let initial = settled()?;
        let count = [Resource::Root, Resource::Extra, Resource::Scratch]
            .iter()
            .filter(|role| context.case.attributes.resources.contains(*role))
            .count();
        let devices = devices(context, inventory, count)?;
        let max_tags = devices
            .iter()
            .map(|device| usize::from(device.info().tag_count))
            .max()
            .expect("filesystem attached disk count validated before preparation");
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: max_tags + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("filesystem completion domain OOM"))?;
        let mut disks = Vec::new();
        let disk_result = (|| {
            disks
                .try_reserve_exact(count)
                .map_err(|_| TestError("filesystem Disk slots OOM"))?;
            for device in devices {
                disks.push(
                    Disk::prepare(device, &queue)
                        .map_err(|_| TestError("filesystem Disk preparation failed"))?,
                );
            }
            Ok(())
        })();
        if let Err(error) = disk_result {
            for disk in &disks {
                disk.stop()
                    .map_err(|_| TestError("unpublished filesystem Disk stop failed"))?;
            }
            queue.close_sync();
            return Err(error);
        }
        let resources = (|| {
            let filesystem_baseline = settled()?;
            let pressure = if context.case.attributes.tags.contains(&"fs-oom") {
                Some(crate::cache::ktest::HeapPressure::prepare()?)
            } else {
                None
            };
            let inode_capacity = if context
                .case
                .attributes
                .tags
                .contains(&"fs-path-cache-budget")
            {
                2
            } else if context.case.attributes.tags.contains(&"fs-path")
                || context.case.attributes.tags.contains(&"fs-system")
            {
                64
            } else {
                8
            };
            let cache = Arc::try_new(
                InodeCache::try_new(inode_capacity)
                    .map_err(|_| TestError("inode cache preparation failed"))?,
            )
            .map_err(|_| TestError("inode cache owner OOM"))?;
            let observation = Observation::prepare()?;
            let loader = observation.clone();
            let hook: Arc<dyn Fn(&InodeRef) + Send + Sync> = Arc::try_new(move |_: &InodeRef| {
                if loader.hold_loader.load(Ordering::Acquire) {
                    loader.loader_entered.fetch_add(1, Ordering::AcqRel);
                    loader.gate.wait();
                }
            })
            .map_err(|_| TestError("inode load observer OOM"))?;
            cache.ktest_set_load_hook(Some(hook));
            let mut prepared = [const { None }; 3];
            let mut joins = Vec::new();
            joins
                .try_reserve_exact(3)
                .map_err(|_| TestError("filesystem native join slots OOM"))?;
            for index in 0..3 {
                let observer = observation.clone();
                prepared[index] = Some(
                    task::prepare_kernel_on(index % cpu::count(), move || {
                        observer.tasks.lock()[index] = Some(task::current_task());
                        let job = observer.jobs.lock()[index].take();
                        if observer.cancelled.load(Ordering::Acquire) {
                            drop(job);
                            return 0;
                        }
                        observer.entered[index].store(true, Ordering::Release);
                        let Some(job) = job else {
                            return 1;
                        };
                        let result = perform(job, &observer);
                        observer.results.lock()[index] = Some(result);
                        0
                    })
                    .map_err(|_| TestError("filesystem native task preparation failed"))?,
                );
            }
            Ok((
                filesystem_baseline,
                cache,
                observation,
                pressure,
                prepared,
                joins,
            ))
        })();
        let (filesystem_baseline, cache, observation, pressure, prepared, joins) = match resources {
            Ok(resources) => resources,
            Err(error) => {
                for disk in &disks {
                    disk.stop()
                        .map_err(|_| TestError("unpublished filesystem resources stop failed"))?;
                }
                queue.close_sync();
                return Err(error);
            }
        };
        let activation_error = disks.iter().find_map(|disk| disk.start().err());
        Ok(Self {
            initial,
            filesystem_baseline,
            disks,
            queue,
            cache: Some(cache),
            cache_watch: None,
            observation: Some(observation),
            pressure,
            prepared,
            joins,
            activation_error,
        })
    }

    /// 正文只借成功发表的 root；激活失败仍由原 fixture 静止化。
    fn disk(&self) -> Result<&DiskRef, TestError> {
        if self.activation_error.is_some() {
            return Err(TestError("filesystem Disk activation failed"));
        }
        self.disks
            .first()
            .ok_or(TestError("filesystem root Disk missing"))
    }

    /// 正式缓存原拥有者在 fixture 存活，异步克隆只扩展当前例寿命。
    fn cache(&self) -> &Arc<InodeCache> {
        self.cache
            .as_ref()
            .expect("filesystem cache consumed before body")
    }

    /// 观察控制存储在主体捕获外固定存活，不借任何主体栈。
    fn observer(&self) -> &Arc<Observation> {
        self.observation
            .as_ref()
            .expect("filesystem observation consumed before body")
    }

    /// 打开真实只读卷；所有格式检查使用生产 codec 与共享 metadata cache。
    fn open(&self, context: &TestContext<'_>) -> Result<Arc<Ext4Volume>, TestError> {
        checked(
            context,
            Ext4Volume::open(
                BlockView::whole(self.disk()?.clone()),
                AccessMode::ReadOnly,
                64,
            ),
        )
    }

    /// 只用正式 root inode 按原字节组件查找，不维护宿主 inode 编号表。
    fn file(
        &self,
        context: &TestContext<'_>,
        volume: &Arc<Ext4Volume>,
        name: &[u8],
    ) -> Result<InodeRef, TestError> {
        let root = checked(context, self.cache().iget(volume, codec::ROOT_INODE))?;
        checked(context, root.lookup(self.cache(), name))
    }

    /// 给尚未发表的原 Task 转交拥有型工作，push 仅消费预留槽。
    fn publish(&mut self, index: usize, job: Job) {
        self.observer().jobs.lock()[index] = Some(job);
        self.joins.push(
            self.prepared[index]
                .take()
                .expect("filesystem native task already published")
                .publish(),
        );
    }

    /// 全部原生 Task 真正离栈，才能消费 JoinHandle 和释放它们的观察存储。
    fn join_tasks(&mut self) -> TestResult {
        for join in &self.joins {
            until("filesystem producer did not leave original stack", || {
                join.try_join().is_some()
            })?;
        }
        for join in self.joins.drain(..) {
            if join.join() != 0 {
                return Err(TestError("filesystem native producer failed"));
            }
        }
        Ok(())
    }

    /// 停止控制者并等待原生任务离栈，结果引用结束后交出 cache owner；派生路径仍可在设备阶段之前退出。
    fn quiesce_jobs(&mut self) -> TestResult {
        let observer = self.observer().clone();
        observer.cancelled.store(true, Ordering::Release);
        observer.hold_loader.store(false, Ordering::Release);
        observer.gate.complete();
        drop(core::mem::take(&mut self.prepared));
        drop(self.pressure.take());
        for disk in &self.disks {
            disk.test_resume_completions();
        }
        self.join_tasks()?;
        for disk in &self.disks {
            disk.test_flush_completions();
        }
        let jobs = core::mem::replace(&mut *observer.jobs.lock(), [const { None }; 3]);
        let results = core::mem::replace(&mut *observer.results.lock(), [const { None }; 3]);
        let tasks = core::mem::replace(&mut *observer.tasks.lock(), [const { None }; 3]);
        drop(jobs);
        drop(results);
        drop(tasks);
        if let Some(cache) = self.cache.take() {
            cache.ktest_set_load_hook(None);
            assert!(self.cache_watch.is_none());
            self.cache_watch = Some(cache.watch());
            drop(cache);
        }
        drop(self.observation.take());
        drop(observer);
        drop(core::mem::take(&mut self.joins));
        Ok(())
    }

    /// 派生路径和最后 cache owner 已退出，先等原 canonical 域真实退场，再核原六域并停止设备。
    fn quiesce_devices(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if let Some(watch) = self.cache_watch.as_ref() {
            let result = watch.wait_retired();
            if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
                return Err(TestError(
                    "canonical inode stop unconfirmed; retaining original owners",
                ));
            }
            checked(context, result)?;
            drop(self.cache_watch.take());
        }
        recovered(
            context,
            "filesystem-before-disk-stop",
            self.filesystem_baseline,
        )?;
        for disk in &self.disks {
            disk.stop().map_err(|_| {
                TestError("filesystem stop cannot converge; retaining original DMA owner")
            })?;
            context.expect_eq(disk.test_snapshot().inflight, 0);
        }
        self.queue.close_sync();
        Ok(())
    }

    /// 先结束全部异步拥有，再释放设备；派生夹具可在两段之间释放路径和卷。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.quiesce_jobs()?;
        self.quiesce_devices(context)
    }

    /// 原 claim、Task、IRQ 与 Work 均结束后销毁设备，再核完整初始六域。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let initial = self.initial;
        drop(self);
        recovered(context, "filesystem-after-disk-drop", initial)
    }
}

impl Fixture<DeviceInventory> for Owner {
    /// 每例借用外层保活库存，先持有完整停止责任，激活错误仍保留原设备供清理。
    fn set_up(
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        Self::prepare(context, inventory)
    }

    /// typed case-abort 在无法收敛时保留原 fixture，不假 join 或丢 DMA owner。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        Owner::quiesce(self, context)
    }

    /// 两层真实资源比较都属于当前例，不拿全客体 PASS 替代局部判定。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        Owner::tear_down(self, context)
    }
}
