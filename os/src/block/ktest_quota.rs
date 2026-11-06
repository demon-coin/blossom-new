//! 块设备完成工作队列（WorkQueue）配额、预留完成任务（Finalizer Task）隔离与 OOM 压力回归测试。
//!
//! # 模块职责与架构定位
//! 本模块属于块设备子系统的内核单元测试（`ktest`），专门验证当大量非硬件槽位触发的 `Bio` 完成回调
//! （如准入拒绝或生成期最后完成回调）在 `EndIo::Task` 上下文中阻塞等待其他磁盘 I/O 时，
//! 不会耗尽共享 `WorkQueue` 的并发配额（`max_active`），从而保证真实的硬件 `Used` 环完成任务与进度推进任务仍能正常调度。
//!
//! # 核心设计与并发交互拓扑
//! 测试构造两类并发请求流（请求组 A 与独立请求 B），它们共享同一个 `WorkQueue` 完成域：
//!
//! ```text
//! +-----------------------------------------------------------------------+
//! | 共享完成工作队列 WorkQueue (普通并发配额 max_active = N * (32 + 1))    |
//! +-----------------------------------------------------------------------+
//!        |                                             |
//!        | (1) 非槽位完成路径 (A 组: 33/34/67 个 Bio)   | (2) 真实硬件 I/O 路径 (独立请求 B)
//!        v                                             v
//! +-----------------------------------+   +-------------------------------+
//! | Origin::Reject (RO FsClaim 拒绝)  |   | VirtIO 硬件槽位提交 Read (B)  |
//! | 或 Origin::GenerationLast         |   | ktest_hold_used 暂缓完成通知  |
//! +-----------------------------------+   +-------------------------------+
//!        |                                             |
//!        v                                             v
//! +-----------------------------------+   +-------------------------------+
//! | 触发各自预留的 Finalizer Task     |   | 释放 Used 后触发槽位专属的    |
//! | 进入 waiting_callback             |   | slot completion_work 与       |
//! | 调用 owner.peer.wait() 陷入阻塞   |   | peer_callback，唤醒全部 A     |
//! +-----------------------------------+   +-------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 全局定位锁 `ACTIVE`：仅在进入回调或清理夹具时短暂获取以克隆 `Arc<Observation>`，严禁持锁睡眠或嵌套获取其他锁。
//! - 观测状态自旋锁（`Observation::tasks`、`peer_task`、`stop_tasks`、`stop_results`）：叶子自旋锁，仅保护各自槽位的读写，持锁期间不调用调度器查询接口（先克隆 `Arc<Task>` 释放锁后再查询 `sched::test_state`）。
//! - 磁盘队列自旋锁 `Disk::queue`：用于读取队列阶段与槽位状态，严禁与 `Observation` 内部自旋锁交叉嵌套。

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

/// 全局当前活跃测试夹具的观察状态槽位。
/// `EndIo::Task` 仅接收裸函数指针，回调通过此全局槽位获取当前用例的 `Observation` 强引用。
static ACTIVE: SpinLock<Option<Arc<Observation>>> = SpinLock::new(None);

/// 请求组 A 触发非硬件槽位完成回调的来源模式。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Origin {
    /// 磁盘被只读文件系统声明（`FsClaim`）占用时，提交写请求在准入阶段被立即拒绝（返回 `BlockError::ReadOnly`）。
    Reject,
    /// 通过受控账本路径模拟生成期最后一个子分片完成（不占用硬件描述符槽位）。
    GenerationLast,
}

/// 测试用例运行期间的跨任务共享观测状态。
struct Observation {
    /// 独立请求 B 的完成句柄，供 A 组所有回调阻塞等待。
    peer: BioCompletion,
    /// 请求组 A 每个 `Bio` 预期收到的完成结果。
    expected_a: IoResult,
    /// 请求组 A 每个 `Bio` 绑定的原始 DMA 物理段描述（用于校验物理地址、长度与方向未发生篡改）。
    originals: Vec<MappedSegment>,
    /// 记录执行每个 A 组回调的内核任务（`Task`）强引用，由自旋锁保护，按 `originals` 下标索引。
    tasks: SpinLock<Vec<Option<Arc<Task>>>>,
    /// 已进入 `waiting_callback` 回调的 A 组任务计数。
    entered: AtomicUsize,
    /// 已从 `waiting_callback` 唤醒并退出的 A 组任务计数。
    returned: AtomicUsize,
    /// 请求组 A 的回调参数、DMA 缓冲区所有权与等待返回值是否全部符合预期。
    valid_a: AtomicBool,
    /// 独立请求 B 的回调 `peer_callback` 被调用的次数（预期恰好为 1）。
    peer_calls: AtomicUsize,
    /// 执行独立请求 B 回调的内核任务（`Task`）强引用，由自旋锁保护。
    peer_task: SpinLock<Option<Arc<Task>>>,
    /// 独立请求 B 的回调参数、DMA 缓冲区及扇区数据图样是否全部校验通过。
    valid_b: AtomicBool,
    /// 独立请求 B 绑定的原始 DMA 物理段描述。
    peer_original: MappedSegment,
    /// 独立请求 B 所在目标磁盘的扇区图样标识（`1` 或 `2`）。
    peer_tag: AtomicUsize,
    /// 各块设备独立执行 `Disk::stop` 的内核任务强引用，按磁盘下标索引。
    stop_tasks: SpinLock<Vec<Option<Arc<Task>>>>,
    /// 各块设备独立执行 `Disk::stop` 的返回结果，按磁盘下标索引。
    stop_results: SpinLock<Vec<Option<Result<(), BlockError>>>>,
}

impl Observation {
    /// 统计请求组 A 中当前已在调度器中处于 `State::Blocked` 阻塞状态的回调任务数量。
    /// 在自旋锁临界区内仅克隆 `Arc<Task>`，随后在锁外向调度器查询任务状态。
    fn blocked(&self) -> usize {
        (0..self.originals.len())
            .filter(|&ordinal| {
                let observed = self.tasks.lock()[ordinal].clone();
                observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
            })
            .count()
    }

    /// 查询指定磁盘下标对应的 `Disk::stop` 任务是否已在调度器中处于 `State::Blocked` 阻塞状态。
    fn stop_blocked(&self, index: usize) -> bool {
        let task = self.stop_tasks.lock()[index].clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

/// 从全局 `ACTIVE` 槽位克隆当前测试夹具的 `Observation` 强引用，随后立即释放自旋锁。
fn observation() -> Arc<Observation> {
    ACTIVE
        .lock()
        .as_ref()
        .expect("quota fixture absent")
        .clone()
}

/// 校验 `memory` 缓冲区的全部字节是否匹配指定磁盘标识 `tag` 与起始扇区 `sector` 的确定性数据图样。
fn patterned(memory: &MappedPages, tag: u8, sector: u64) -> bool {
    if memory.is_device_owned() {
        return false;
    }
    // SAFETY:
    // 1. 前置条件 `if memory.is_device_owned()` 已确认物理页的 DMA 访问权已交还 CPU；
    // 2. 调用方仅在读取完成且无并发设备写入时调用本函数，在缓冲区的有效长度内创建只读字节切片是安全的。
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

/// 请求组 A 的 `EndIo::Task` 完成回调函数。
/// 校验完成状态与全零物理页后记录当前执行任务，并阻塞等待独立请求 B 完成。
fn waiting_callback(result: &IoResult, memory: &[MappedPages]) {
    let owner = observation();
    let valid = *result == owner.expected_a && memory.len() == 1 && !memory[0].is_device_owned();
    let ordinal = if valid {
        let mapped = &memory[0];
        let segment = mapped
            .segment(crate::mm::dma::DmaDomain::qemu_coherent())
            .expect("QEMU fixture mapping must be reachable");
        // SAFETY:
        // 1. 请求组 A 的物理页在 `bio()` 构造时通过 `mm::alloc_pages(0, true)` 分配并清零；
        // 2. `Reject` 与 `GenerationLast` 两种路径均未向硬件描述符环发布该页，且上方已检查 `!memory[0].is_device_owned()`，CPU 独占访问该页。
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

/// 独立请求 B 的 `EndIo::Task` 完成回调函数。
/// 在可调度上下文中校验回调仅执行一次、DMA 物理段属性一致以及扇区 400 的完整数据图样。
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

/// 在 10 秒单调时钟期限内轮询条件 `ready`，每次未就绪时睡眠 1 毫秒；超时则返回 `TestError(label)`。
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

/// 等待除当前 CPU 外的所有核心进入空闲（idle）状态，随后收缩 SLUB 缓存并排空每 CPU 页面缓存，返回全系统 6 个内存域的统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("quota services did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 从测试套件共享的设备发现清单 `inventory` 中准备指定数量（`count`）的 VirtIO 块设备及其 Legacy 布局标志。
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
        // SAFETY:
        // 1. `candidate` 来自启动阶段枚举的合法 VirtIO MMIO 区域且具备永久强序内核映射；
        // 2. 测试套件声明了 `isolation = "Case"` 且上一用例已在 `quiesce` 中停止并回收设备，当前用例独占该硬件实例。
        let device = unsafe { VirtioDevice::prepare_mmio(candidate) }
            .map_err(|_| TestError("quota MMIO preparation failed"))?;
        prepared.push((device, candidate.version == 1));
    }
    for candidate in pci {
        // SAFETY:
        // 1. `candidate` 来自 PCI 总线枚举到的合法 VirtIO 块设备 BDF；
        // 2. 上一测试用例已完成设备停止与工作线程回收，当前用例独占该 PCI 设备。
        let device = unsafe { VirtioDevice::prepare_pci(&inventory.pci, candidate) }
            .map_err(|_| TestError("quota PCI preparation failed"))?;
        prepared.push((device, false));
    }
    Ok(prepared)
}

/// 为指定磁盘 `disk` 分配一个清零物理页并构造单扇区（扇区号 400）的 `Bio` 请求，绑定 `callback` 完成回调并返回对应的 DMA 段信息。
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

    // SAFETY:
    // 1. `pages` 由 `mm::alloc_pages(0, true)` 分配并清零，且所有权已完全移交 `memory` 向量，不存在其他 CPU 别名引用；
    // 2. 绑定的 `EndIo::Task(callback)` 仅等待独立的请求 B，不会死锁于自身或当前槽位。
    let bio = unsafe { Bio::try_new(disk, operation, 400, memory, 0, Some(EndIo::Task(callback))) }
        .map_err(|_| {
            TestError("quota Bio construction failed; Reject needs writable raw working copy")
        })?;
    Ok((bio, original))
}

/// 内存压力控制器，用于在测试关键阶段耗尽物理页与全部 SLUB 堆大小类别的空闲内存，并在阶段结束后原样归还。
struct Pressure {
    /// 压力期间持有的全部高阶/单页物理页所有权列表（容量在施加压力前预先分配）。
    pages: Vec<mm::OwnedPages>,
    /// 每个 SLUB 大小类别（`mm::SLUB_SIZES`）的侵入式单向空闲链表头指针（以 `usize` 地址保存，0 表示空链表）。
    heap: [usize; mm::SLUB_SIZES.len()],
}

impl Pressure {
    /// 在施加 OOM 压力前预先为物理页保存向量分配足够的容量，避免在耗尽内存时发生向量扩容。
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

    /// 耗尽系统当前可用的全部单页物理内存与所有 SLUB 堆大小类别，并返回 `(物理页是否耗尽, 堆内存是否耗尽)` 的探测结果。
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
                // SAFETY: `size` 来自内核预定义的合法非零 `mm::SLUB_SIZES`，对齐要求为 `usize` 对齐，`Layout` 合法。
                let pointer = black_box(unsafe { alloc(layout) });
                if pointer.is_null() {
                    break;
                }
                // SAFETY:
                // 1. 上方已检查 `!pointer.is_null()`，该内存块由当前调用方独占拥有；
                // 2. `mm::SLUB_SIZES` 中最小类别不小于 `size_of::<usize>()` 且满足 `usize` 对齐，在首字写入下一节点地址安全。
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
            // SAFETY: `layout` 为合法的非零大小与 `usize` 对齐布局；若探测意外分配成功，立即使用完全相同的 `layout` 释放该指针。
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

    /// 遍历所有 SLUB 大小类别的侵入式链表并将堆块按原布局逐一释放，随后清空持有的全部物理页。
    fn release(&mut self) {
        for (index, size) in mm::SLUB_SIZES.iter().copied().enumerate() {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("quota heap release layout");
            while self.heap[index] != 0 {
                let pointer = self.heap[index] as *mut u8;
                // SAFETY:
                // 1. `pointer` 来自 `exhaust()` 中使用相同 `layout` 成功分配的非空块；
                // 2. 首字在入链时已写入前驱节点地址，读取后立即调用 `dealloc` 释放，不会产生二次释放或悬垂访问。
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
    /// 析构时自动释放全部持有的堆块与物理页，防止测试中途出错退出时泄漏内存压力资源。
    fn drop(&mut self) {
        self.release();
    }
}

/// 单盘或双盘配额饱和回归测试的夹具主结构，负责管理设备、工作队列、请求及辅助任务的完整生命周期。
struct Owner {
    /// 用例启动前全系统 6 个内存域的统计基线（用于在 `tear_down` 中校验零内存泄漏）。
    baseline: mm::MemoryStats,
    /// 请求组 A 触发非槽位完成回调的路径类型（`Reject` 或 `GenerationLast`）。
    origin: Origin,
    /// 是否在请求 B 已收到硬件 `Used` 环通知但尚未放行完成时施加物理页与堆 OOM 压力。
    oom: bool,
    /// 当前用例管理的块设备实例列表（单盘为 1 个，双盘为 2 个）。
    disks: Vec<DiskRef>,
    /// 与 `disks` 一一对应的 VirtIO Legacy 寄存器布局标志。
    legacy: Vec<bool>,
    /// 所有磁盘与 `Bio` 完成回调共享的 `WorkQueue` 完成域。
    queue: WorkQueue,
    /// 共享 `WorkQueue` 的普通并发工作任务配额（每块磁盘 `tag_count + 1` 的总和）。
    max_active: usize,
    /// 跨任务与回调共享的观测状态。
    observation: Arc<Observation>,
    /// 请求组 A 所有 `Bio` 的完成句柄列表。
    completions: Vec<BioCompletion>,
    /// 预先分配好内核栈、负责提交请求组 A 的生产者任务（绑定在 CPU 0）。
    prepared_a: Option<PreparedTask>,
    /// 预先分配好内核栈、负责提交独立请求 B 的生产者任务（绑定在 CPU 1）。
    prepared_b: Option<PreparedTask>,
    /// 请求组 A 生产者任务发布调度后的 `JoinHandle`。
    producer: Option<JoinHandle>,
    /// 独立请求 B 生产者任务发布调度后的 `JoinHandle`。
    submitter: Option<JoinHandle>,
    /// 为每块磁盘预先分配好内核栈、用于并发执行 `Disk::stop` 的任务列表。
    prepared_stops: Vec<Option<PreparedTask>>,
    /// 各磁盘 `Disk::stop` 任务发布调度后的 `JoinHandle` 列表。
    stops: Vec<Option<JoinHandle>>,
    /// 在 `Origin::Reject` 模式下持有的整盘只读文件系统声明（用于触发写请求准入拒绝）。
    claim: Option<FsClaim>,
    /// 物理页与 SLUB 堆 OOM 内存压力控制器。
    pressure: Pressure,
    /// 磁盘在 `Disk::start()` 阶段是否发生错误。
    activation_error: bool,
    /// 提交请求组 A 之前，磁盘 0 的 `(已发布请求数, 已完成请求数)` 快照。
    before_a: (usize, usize),
    /// 提交独立请求 B 之前，目标磁盘 B 的 `(已发布请求数, 已完成请求数)` 快照。
    before_b: (usize, usize),
    /// 独立请求 B 预期推进到的 VirtIO `Used` 环索引目标值。
    used_target: u16,
    /// 独立请求 B 在目标磁盘 B 上占用的硬件槽位下标。
    b_slot: Option<usize>,
}

impl Owner {
    /// 根据指定的磁盘数量 `count` 与测试用例参数，预先分配共享 `WorkQueue`、`Bio` 请求、观测状态以及生产者与停止任务。
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
            // Step 1: 计算请求组 A 的并发数量（`normal33`/`oom33` 使用 `max_active`，其余用例使用 `max_active + 1` 超出普通配额）。
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

    /// 读取每块磁盘扇区 0 的全扇区字节图样，校验磁盘身份标识（`tag1` / `tag2`）完整且双盘互不相同。
    fn identities(&self, context: &mut TestContext<'_>) -> TestResult {
        let mut tags = 0_u8;
        for (index, disk) in self.disks.iter().enumerate() {
            let unit = disk.limits().logical_block_size as usize;
            let mut bytes = [0_u8; mm::PAGE_SIZE];
            disk.read_exact_at(0, &mut bytes[..unit])
                .map_err(|_| TestError("quota raw identity Read failed"))?;
            // 设备枚举顺序可能与宿主挂载顺序不同，通过首字节与全扇区校验公式确定磁盘的实际 tag 编号。
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

    /// 执行配额饱和与预留完成任务隔离测试主流程：
    /// 让请求组 A 全部阻塞在 `waiting_callback` 中，验证独立请求 B 仍能通过专属预留任务完成硬件读取并唤醒全部 A 组任务。
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
        // Step 1: 发布请求组 A 的生产者任务，等待全部 A 组回调进入 `waiting_callback` 并在调度器中处于 `Blocked` 状态。
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

        // Step 2: 暂缓磁盘 B 的 `Used` 环完成处理，发布独立请求 B 并等待硬件将其写入 `Used` 环且预留工作任务就绪。
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

        // Step 3: 在请求 B 的描述符与所有预留任务就绪后，按用例配置施加 OOM 压力，或在非 OOM 模式下并发发起 `Disk::stop`。
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

        // Step 4: 释放磁盘 B 的 `Used` 环拦截，验证在配额耗尽（及 OOM）条件下请求 B 仍能完成并唤醒全部请求组 A。
        self.disks[b].ktest_release_used();
        let progressed = until(
            "quota B and all A did not progress with original execution resources",
            || {
                self.observation.peer.is_complete()
                    && self.observation.returned.load(Ordering::Acquire) == count
                    && self.completions.iter().all(BioCompletion::is_complete)
            },
        );
        // Step 5: 解除内存压力后，再执行后续断言与诊断输出。
        self.pressure.release();
        stopped_wait?;
        progressed?;

        // Step 6: 触发剩余磁盘的停止任务，等待硬件槽位归零，并校验回调执行次数、任务隔离性及 DMA 缓冲区数据完整性。
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

    /// 获取指定磁盘当前的 `(已发布到硬件槽位的请求数, 已完成硬件槽位的请求数)` 统计快照。
    fn counts(&self, index: usize) -> (usize, usize) {
        let q = self.disks[index].queue.lock();
        (q.ktest_stop.published, q.ktest_stop.finished)
    }

    /// 将指定磁盘预先准备好的 `Disk::stop` 内核任务发布到调度器执行。
    fn start_stop(&mut self, index: usize) {
        self.stops[index] = Some(
            self.prepared_stops[index]
                .take()
                .expect("quota stop published twice")
                .publish(),
        );
    }

    /// 静止化清理流程：释放内存压力与 `Used` 环拦截，回收所有生产者任务，按逆序停止磁盘并关闭共享工作队列。
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

        // Step 1: 所有生产者与磁盘专属工作项均已停止并回收，同步关闭共享 `WorkQueue` 并刷新所有 `Bio` 预留完成任务。
        self.queue.close_sync();
        context.expect_eq(self.queue.ktest_reserved_workers(), 0);
        for completion in &self.completions {
            completion.test_flush_finalizer();
            context.expect_eq(completion.ktest_finalizer_joined(), true);
        }
        self.observation.peer.test_flush_finalizer();
        context.expect_eq(self.observation.peer.ktest_finalizer_joined(), true);

        // Step 2: 所有异步回调任务均已确认退出，清空全局 `ACTIVE` 定位槽。
        let active = ACTIVE
            .lock()
            .take()
            .ok_or(TestError("quota locator disappeared before join"))?;
        context.expect_eq(Arc::ptr_eq(&active, &self.observation), true);
        drop(active);
        Ok(())
    }

    /// 销毁测试夹具并等待所有 CPU 核心空闲、排空页面缓存，断言全系统 6 个内存域统计与测试前基线完全一致。
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

/// 从已完成的 `BioCompletion` 中取回 DMA 物理页所有权，校验结果、物理段属性、数据图样以及重复取回时的 `BlockError::Busy` 错误。
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

/// 单盘配额回归测试夹具封装（包装 `Owner`）。
struct Single(Owner);
/// 双盘跨盘配额回归测试夹具封装（包装 `Owner`）。
struct Double(Owner);

macro_rules! fixture {
    ($name:ident, $count:expr) => {
        impl Fixture<DeviceInventory> for $name {
            /// 从共享设备清单中初始化指定磁盘数量的 `Owner` 测试夹具。
            fn set_up(
                context: &mut TestContext<'_>,
                inventory: &DeviceInventory,
            ) -> Result<Self, TestError> {
                Owner::prepare($count, context, inventory).map(Self)
            }
            /// 静止化夹具中的所有活跃任务、磁盘中断、工作项与共享工作队列。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// 销毁夹具并校验全系统 6 个内存域无任何泄漏。
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
    /// 测试目标：验证单盘场景下，当 33 或 34 个写请求因只读 `FsClaim` 被准入拒绝并全部阻塞在 `EndIo::Task` 回调中时，
    /// 同一磁盘上的独立硬件读请求 B 仍能通过预留工作任务完成（含正常与 OOM 压力模式）并唤醒所有阻塞回调。
    #[test]
    #[case("normal33", ())]
    #[case("oom33", ())]
    #[case("normal", ())]
    #[case("oom", ())]
    fn reject(context: &mut TestContext<'_>, fixture: &mut Single, _: ()) -> TestResult {
        fixture.0.saturation(context)
    }

    /// 测试目标：验证单盘场景下，当 33 或 34 个请求通过生成期最后完成路径阻塞在 `EndIo::Task` 回调中时，
    /// 同一磁盘上的独立硬件读请求 B 仍能正常完成硬件读取（含正常与 OOM 压力模式）并唤醒所有阻塞回调。
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
    /// 测试目标：验证双盘共享同一个 `WorkQueue`（普通并发配额 66）时，磁盘 A 上 67 个被只读 `FsClaim` 拒绝的请求回调全部阻塞后，
    /// 磁盘 B 上的独立硬件读请求仍能完成（含正常与 OOM 压力模式）并唤醒磁盘 A 的全部阻塞回调。
    #[test]
    #[case("normal", ())]
    #[case("oom", ())]
    fn reject(context: &mut TestContext<'_>, fixture: &mut Double, _: ()) -> TestResult {
        fixture.0.saturation(context)
    }

    /// 测试目标：验证双盘共享同一个 `WorkQueue`（普通并发配额 66）时，磁盘 A 上 67 个生成期最后完成回调全部阻塞后，
    /// 不会占用或阻塞磁盘 B 的独立槽位与进度工作项，磁盘 B 的硬件读取仍能完成并唤醒所有回调。
    #[test]
    #[case("normal", ())]
    #[case("oom", ())]
    fn generation_last(context: &mut TestContext<'_>, fixture: &mut Double, _: ()) -> TestResult {
        fixture.0.saturation(context)
    }
}

/// 验证“已构造但从未提交的预留工作项（Reserved Work）”在工作队列关闭时能正确退出并回收的测试夹具。
struct Unsubmitted {
    /// 用例启动前全系统 6 个内存域的统计基线。
    baseline: mm::MemoryStats,
    /// 被测工作队列实例。
    queue: WorkQueue,
    /// 已通过 `Work::try_new_reserved` 构造但从未入队提交的预留工作项。
    work: Pin<Arc<Work>>,
    /// 记录 `unsubmitted_callback` 被调用次数的原子计数器（预期始终为 0）。
    calls: Arc<AtomicUsize>,
    /// 标记工作队列是否已执行 `close_sync`。
    closed: bool,
}

/// 未提交预留工作项的回调函数（正常情况下永不会被调用；若被误调用则递增计数器使断言失败）。
fn unsubmitted_callback(work: Pin<&Work>) {
    // SAFETY: `Unsubmitted` 夹具持有 `calls: Arc<AtomicUsize>` 直到 `queue.close_sync()` 完成，
    // `work.context()` 保存的是 `Arc::as_ptr(&calls)` 转换而来的合法有效指针。
    let calls = unsafe { &*(work.context() as *const AtomicUsize) };
    calls.fetch_add(1, Ordering::Release);
}

impl Fixture<()> for Unsubmitted {
    /// 创建 `max_active = 1` 的工作队列并构造一个带专属预留任务的 `Work`，但不向队列提交该工作项。
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

    /// 若用例尚未关闭队列则执行同步关闭，并断言预留工作者计数归零、预留任务已退出回收且回调从未执行。
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

    /// 释放夹具持有的全部控制结构与 `Arc` 引用，断言全系统 6 个内存域恢复至基线水平。
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
    /// 测试目标：验证当外部仍持有 `Pin<Arc<Work>>` 强引用且该预留 `Work` 从未调用 `queue()` 提交时，
    /// 调用 `WorkQueue::close_sync()` 能正确撤销未提交的预留任务并等待其内核线程退出回收，且关闭后再次入队返回错误。
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
