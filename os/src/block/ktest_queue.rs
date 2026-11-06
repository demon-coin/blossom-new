//! 块设备请求队列（`Disk` / `Bio`）与 VirtIO 环并发调度的内核自检套件（`ktest`）。
//!
//! ## 模块职责与架构定位
//! 本模块位于块设备层测试平面，上游由 `ktest` 框架按套件与用例调度，下游直接驱动 `Disk`、
//! `Bio`、`BioCompletion`、`FsClaim`、`WorkQueue` 以及 VirtIO MMIO/PCI 块设备驱动，
//! 全面验证块请求队列的并发提交、资源反压、请求拆分与合并、中断抑制与预算续行、
//! 异步回调生命周期、受控父请求账本（Ledger）聚合以及文件系统卷独占挂载（Claim）语义。
//!
//! ## 核心设计与生命周期模型
//! 每个测试用例都通过 `Owner` 夹具独立完成完整的硬件队列与内存生命周期闭环：
//!
//! ```text
//!   set_up (Owner::prepare)
//!     │  1. settled()：等待所有 CPU 空闲，排空页缓存与 Per-CPU 列表，记录 6 域内存基线
//!     │  2. devices()：从 DeviceInventory 接管 VirtIO 块设备（MMIO 或 PCI）
//!     │  3. WorkQueue::try_new + Disk::prepare：分配请求槽位、描述符环与完成工作队列
//!     │  4. Request::prepare：预分配独立 0 阶物理页（MappedPages）、Bio 与生产者内核线程
//!     │  5. disk.start()：注册中断并激活队列至 QueuePhase::Live
//!     ▼
//!   Test Body (multi / pressure / rearm / split / batch / callback_tail / ...)
//!     │  - 提交 Bio（单发、批量或多线程并发）或驱动受控账本（BioInner）
//!     │  - 验证阻塞唤醒、环描述符/槽位占用、DMA 访问权交还（!is_device_owned()）与读回数据
//!     ▼
//!   quiesce (Owner::quiesce)
//!     │  1. 释放外部同步门（gate），恢复被挂起的完成处理，刷回残余脏页
//!     │  2. 排空在途槽位（drained），释放 FsClaim，调用 disk.stop() 停止硬件队列并注销中断
//!     │  3. join_tasks() 等待全部生产者线程退出内核栈，关闭 WorkQueue，清理全局回调槽
//!     ▼
//!   tear_down (Owner::tear_down)
//!        释放 Owner 持有的全部对象，再次执行 settled() 并断言 6 域内存统计与基线完全一致
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 全局回调定位锁 `CALLBACK_OWNER`、观察器内部自旋锁（`Observation::tasks`、
//!   `callback_task`、`callback_result`、`peer`）以及 `Disk::queue` 自旋锁均为叶级短临界区锁，
//!   彼此之间**禁止嵌套获取**。
//! - 严禁在持有上述任何 `SpinLock` 期间调用 `Completion::wait`、`BioCompletion::wait`、
//!   `timer::sleep` 或 `sched::test_state` 等可能阻塞或查询调度器状态的操作。
//! - 异步任务回调（`EndIo::Task(callback)`）在工作队列的 Worker 线程中执行，
//!   只允许等待外部 `Observation::gate` 或无依赖的配对请求（`peer`），绝不等待自身请求完成或当前磁盘停止，从而避免工作队列死锁。

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

/// 全局回调观察者槽位。
///
/// 由于 `EndIo::Task` 仅接收无状态函数指针 `fn(&IoResult, &[MappedPages])`，
/// 需要使用回调的测试夹具会在激活队列前将当前用例的 `Arc<Observation>` 存入此静态槽位，
/// 并在 `quiesce` 确认磁盘停止且工作队列彻底关闭后将其清空。
static CALLBACK_OWNER: SpinLock<Option<Arc<Observation>>> = SpinLock::new(None);

#[derive(Clone, Copy, PartialEq, Eq)]
/// 块队列测试用例的场景分类枚举，决定 `Owner::prepare` 预分配的资源规模、队列限额与请求布局。
enum Scenario {
    /// 8 个内核生产者线程并发提交各自独立的双页读请求。
    Multi,
    /// 耗尽全部请求槽位（Tag）后触发第 `tag_count + 1` 个提交者睡眠阻塞与完成唤醒。
    Tag,
    /// 在槽位未满时先耗尽 VirtIO 环描述符（Descriptor），触发提交者阻塞与完成唤醒。
    Descriptor,
    /// 在 `VRING_AVAIL_F_NO_INTERRUPT` 中断抑制与单次中断处理预算（`IRQ_BUDGET`）下验证重装检查与工作队列续行。
    Rearm,
    /// 将单个跨越硬件段数或段字节上限的多页分散/聚集（SG）请求拆分为多个子请求（Child）下发。
    Split,
    /// 同一批次（Batch）中三个扇区连续的父请求合并为单个硬件请求下发。
    Merge,
    /// 同一批次中三个扇区不连续的父请求各自独立下发，但仅在批次末尾执行一次硬件队列通知（Unplug/Kick）。
    Notify,
    /// 批次中第二个请求因跨越 DMA 段边界且无完整逻辑块而在拆分阶段失败时，首个已入环请求仍能被正常通知并完成。
    Tail,
    /// 验证 `EndIo::Task` 异步回调在阻塞期间保持槽位与父请求未完成状态，直至回调退出后才交还物理页并唤醒等待者。
    Callback,
    /// 在独占 Scratch 磁盘上验证只读挂载占用（`FsClaim`）拒绝写请求与拒绝停盘，以及释放占用后的裸盘写、读回校验与 `Flush`。
    Scratch,
    /// 验证读写模式 `FsClaim` 的授权写入、克隆引用计数、异步回调期间保持挂载占用，以及读回与 `Flush`。
    FsScratch,
    /// 验证硬件特性协商为只读（`VIRTIO_BLK_F_RO`）的块设备拒绝读写 `FsClaim`，但允许只读 `FsClaim` 正常读取。
    ReadonlyDevice,
    /// 验证请求入口参数校验（`DiskLimits::validate` 与 `read_exact_at`），拒绝越界、未对齐、方向错误或非法 `Flush` 请求。
    Entry,
    /// 验证双磁盘环境下将绑定于磁盘 A 的 `Bio` 提交给磁盘 B 时在准入阶段被拒绝，且物理页正常归还。
    Identity,
    /// 验证 `Disk::read_exact_at` 在不同逻辑扇区（兼容 512B 至 4KiB 逻辑块）读取到的固定图样数据完整且互不相同。
    Exact,
    /// 不经过硬件队列，直接在内存中驱动 `BioInner` 受控父请求账本，验证子请求乱序完成、最短错误前缀与回写（Writeback）脏页状态转换。
    Model,
    /// 两个独立提交的请求中，第一个请求的回调等待第二个请求完成，验证完成推进工作项不会因首个回调阻塞而饿死第二个请求。
    Progress,
    /// 同一批次提交的两个请求中，第一个请求带有 `EndIo::Task` 回调并等待第二个请求，验证队列禁止将带异步回调的请求与后续请求合并以免自锁。
    ProgressBatch,
    /// 在受控账本上验证前一个请求的拆分子请求提前完成时，预留的完成收尾流程不会阻塞同批次后续请求的生成与完成。
    Generation,
    /// 验证磁盘调用 `stop()` 进入停止状态后拒绝新提交的 `Bio`（返回 `BlockError::QueueStopped`），且重复调用 `stop()` 保持幂等。
    Stopped,
}

/// 跨线程与异步回调的执行状态观察器。
///
/// 由 `Owner` 与生产者内核线程或 `EndIo::Task` 回调共享，用于同步测试阶段并记录执行上下文。
struct Observation {
    /// 外部同步门：用于让 `callback` 主动阻塞在已知位置，直到主测试线程调用 `complete()` 放行。
    gate: Completion,
    /// 各生产者内核线程的 `Task` 句柄表，按生产者序号索引；由本字段自身的 `SpinLock` 保护。
    tasks: SpinLock<Vec<Option<Arc<Task>>>>,
    /// 实际执行 `callback` 的工作队列 Worker 线程 `Task` 句柄；由本字段自身的 `SpinLock` 保护。
    callback_task: SpinLock<Option<Arc<Task>>>,
    /// `callback` 收到的 `IoResult` 副本；由本字段自身的 `SpinLock` 保护。
    callback_result: SpinLock<Option<IoResult>>,
    /// 记录 `callback` 入口处校验是否通过（无错误、完成字节数匹配、物理页已收回 CPU 访问权且配对请求结果一致）。
    callback_valid: AtomicBool,
    /// `callback` 被调用的总次数计数器。
    callback_calls: AtomicUsize,
    /// 标记 `callback` 是否已跨过阻塞点并执行到函数末尾即将返回的位置。
    callback_tail: AtomicBool,
    /// 可选的配对请求完成句柄；若设置，则 `callback` 会等待该配对请求完成而非等待 `gate`。
    peer: SpinLock<Option<BioCompletion>>,
}

impl Observation {
    /// 预分配指定数量的生产者线程观察槽位并构造 `Observation` 实例。
    ///
    /// 在普通测试准备上下文中调用，所有堆分配在磁盘激活前完成，分配失败时返回 `TestError`。
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

    /// 查询序号为 `ordinal` 的生产者线程当前在调度器中是否已进入 `State::Blocked` 阻塞态。
    ///
    /// 先在短自旋锁内克隆 `Arc<Task>`，释放锁后再向调度器查询任务状态。
    fn blocked(&self, ordinal: usize) -> bool {
        let observed = self.tasks.lock()[ordinal].clone();
        observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 检查执行 `callback` 的任务是否为工作队列的 Worker 线程且当前正处于 `State::Blocked` 阻塞态。
    fn callback_blocked(&self) -> bool {
        let observed = self.callback_task.lock().clone();
        observed
            .is_some_and(|task| task.worker.is_some() && sched::test_state(&task) == State::Blocked)
    }
}

/// 供 `EndIo::Task` 使用的测试回调函数。
///
/// 在工作队列的 Worker 线程上下文中执行：
/// 1. 从全局 `CALLBACK_OWNER` 获取当前用例的 `Observation`；
/// 2. 校验 I/O 结果无误、完成字节数等于全部 `MappedPages` 长度之和，且所有物理页均已从设备收回 DMA 访问权（`!is_device_owned()`）；
/// 3. 记录当前 Worker 线程的 `Task` 句柄与 `IoResult`；
/// 4. 若配置了配对请求（`peer`），则等待配对请求完成并核对结果；否则等待外部同步门（`gate`）放行；
/// 5. 唤醒后置位 `callback_tail` 标志并退出。
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

/// 在 5 秒单调时钟期限内轮询谓词 `ready`，每次未就绪时睡眠 1 毫秒；超时则返回 `Err(TestError(label))`。
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

/// 等待除当前 CPU 外的所有其他 CPU 进入空闲态，随后收缩内核缓存并排空 Per-CPU 页缓存，返回稳定的 6 域物理内存统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("queue resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 从测试套件发现的硬件清单 `inventory` 中提取并初始化指定数量（`count`）的 VirtIO 块设备。
///
/// 先核对 MMIO 与 PCI VirtIO 块设备总数是否与当前用例声明的设备数一致，随后逐一完成设备探测与映射准备。
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
        // SAFETY:
        // 1. `candidate` 来自测试框架启动阶段扫描并建立永久 MMIO 映射的 `DeviceInventory`；
        // 2. 上一个测试用例已在 `quiesce` 与 `tear_down` 中完成了设备停机、中断注销与所有权释放，当前用例独占该 MMIO 块设备。
        prepared.push(
            unsafe { VirtioDevice::prepare_mmio(candidate) }
                .map_err(|_| TestError("queue MMIO device preparation failed"))?,
        );
    }
    for device in pci {
        // SAFETY:
        // 1. `device` 的 PCI BDF 来自同一个有效的 `DeviceInventory`；
        // 2. 当前测试夹具独占该 PCI 设备，并将其保活至 `quiesce` 阶段完成队列停机、中断注销与硬件复位为止。
        prepared.push(
            unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
                .map_err(|_| TestError("queue PCI device preparation failed"))?,
        );
    }
    Ok(prepared)
}

/// 单个测试 I/O 请求的夹具封装，持有待提交的 `Bio`、完成等待句柄以及底层物理页的原始 DMA 段信息。
struct Request {
    /// 待提交给 `Disk` 的 `Bio` 实例；提交或移交生产者线程后被 `take()` 置为 `None`。
    pending: Option<Bio>,
    /// 与 `Bio` 关联的完成通知与物理页回收句柄。
    completion: BioCompletion,
    /// 构造请求时记录的每个 `MappedPages` 分片在 QEMU 一致性 DMA 域下的物理地址、长度与方向快照，用于回收时核对物理页身份未被替换。
    originals: Vec<MappedSegment>,
    /// 请求起始的 512 字节扇区号。
    sector: u64,
    /// 请求传输的总字节数。
    bytes: usize,
}

impl Request {
    /// 为指定磁盘分配独立的 0 阶物理页（`MappedPages`）并构造 `Bio` 请求。
    ///
    /// - 对于写操作（`BlkOperation::Write`），预先向物理页写入按字节递增的确定性校验图样 `(logical * 13 + 71)`；
    /// - 对于读操作（`BlkOperation::Read`），分配零初始化物理页；
    /// - 记录所有物理分片的初始 DMA 段信息后封装为 `Request` 返回。
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

        // SAFETY:
        // 1. `memory` 中的每个 `MappedPages` 均由刚刚分配的独立 0 阶物理页构造，若为写请求已完成全部字节初始化，且构造后未保留任何 CPU 裸指针或切片别名；
        // 2. 若挂载了 `EndIo::Task(callback)`，回调内仅只读借用已收回 CPU 访问权的 `MappedPages`，且只等待外部 `gate` 或独立 `peer` 请求，不等待自身完成或当前磁盘 `stop()`；
        // 3. 在 `Owner::quiesce` 中会确保所有已提交请求或受控账本彻底完成并回收，不会提前释放处于设备 DMA 期间的物理页。
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

    /// 等待父请求完成，取回底层 `MappedPages` 列表，校验物理页身份、DMA 访问权归还状态以及读操作的数据图样。
    ///
    /// - 断言 `completion.wait()` 等于 `expected`；
    /// - 断言 `take_memory()` 成功取回与初始分片数量、物理地址、长度、传输方向完全一致的 `MappedPages`，且 `is_device_owned()` 为 `false`；
    /// - 若提供了磁盘身份标签 `tag`，逐字节校验读回的数据是否符合磁盘预置扇区公式 `tag * 29 + (offset / 512) * 17 + (offset % 512)`；
    /// - 断言再次调用 `take_memory()` 返回 `Err(BlockError::Busy)`，确保物理页只能被领取一次。
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
                // SAFETY:
                // 1. 上方 `self.completion.wait()` 已确认父请求及所有拆分子请求全部完成，且 `assert!(!mapped.is_device_owned())` 确认设备已交还 DMA 访问权；
                // 2. 物理页在分配时已零初始化并由设备 DMA 读入数据，当前测试线程通过 `take_memory()` 独占持有 `memory`，无任何并发写者。
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

    /// 验证请求完整成功（`completed == self.bytes` 且 `error == None`）并执行物理页回收与可选的数据图样校验。
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

/// 块队列测试套件的核心夹具所有者，管理单次测试用例的全部硬件设备、工作队列、请求缓冲与并发线程生命周期。
struct Owner {
    /// 当前测试用例对应的场景枚举。
    scenario: Scenario,
    /// 用例准备开始前（所有 CPU 空闲且缓存排空后）记录的 6 域物理内存统计基线。
    baseline: mm::MemoryStats,
    /// 当前被测的主块设备队列引用。
    disk: DiskRef,
    /// 双设备场景（`Scenario::Identity`）下挂载的第二个块设备队列引用。
    extra: Option<DiskRef>,
    /// 驱动块设备异步完成收尾与 `EndIo::Task` 回调的工作队列。
    queue: WorkQueue,
    /// 预先分配并构造好的测试请求列表。
    requests: Vec<Request>,
    /// 批量提交场景（`Merge`、`Notify`、`Tail`、`ProgressBatch`）下预留容量的 `Bio` 向量。
    batch: Vec<Bio>,
    /// 预先分配好内核栈但尚未发布到调度器的生产者任务列表。
    prepared: Vec<PreparedTask>,
    /// 已发布到调度器的生产者任务 `JoinHandle` 列表，用于在 `quiesce` 阶段等待线程彻底退出内核栈。
    joins: Vec<JoinHandle>,
    /// 当前用例的跨线程与回调状态观察器。
    observation: Arc<Observation>,
    /// `disk.start()` 或 `extra.start()` 激活阶段的错误记录；若非空则延迟到测试主体入口报告，以保证已分配资源仍走完标准 `quiesce`/`tear_down` 清理流程。
    activation_error: Option<BlockError>,
    /// 主块设备是否为 VirtIO Legacy MMIO（v1）设备，用于决定读取环 `used.idx` 时的内存布局偏移。
    legacy: bool,
    /// 文件系统挂载占用场景（`FsScratch`、`ReadonlyDevice`）下持有的 `FsClaim` 凭证。
    claim: Option<FsClaim>,
    /// 资源反压与中断预算场景（`Tag`、`Descriptor`、`Rearm`）中用于填满槽位或环的初始请求数量。
    fill: usize,
    /// 受控父账本场景（`Scenario::Model`）中当前已调用 `begin()` 但尚未收敛完成的请求下标；用于防止中途断言失败时直接丢弃未完成账本。
    model_live: Option<usize>,
    /// 受控回写账本场景（`writeback_ledger`）中分配的私有页缓存 `FolioRef` 列表。
    wb_folios: Vec<mm::FolioRef>,
    /// 受控回写账本场景中正在运行的父 `Bio` 实例。
    wb_active: Option<Bio>,
}

impl Owner {
    /// 根据场景 `scenario` 准备完整的块设备队列夹具。
    ///
    /// 依次完成：记录 6 域内存基线、提取并校验 VirtIO 设备能力、创建完成工作队列、按场景定制队列分段限额、
    /// 预分配请求页与生产者内核线程、绑定全局回调观察槽，最后调用 `start()` 激活磁盘。
    /// 若任何预分配步骤失败，立即回滚停止未发布的磁盘并关闭工作队列。
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

        // Step 1: 创建完成工作队列，并按场景配置 Disk 的最大请求字节数、最大段数、单段字节数与 DMA 边界掩码。
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

        // Step 2: 在闭包内预分配第二块磁盘（Identity 场景）、Observation、Request 列表与生产者线程，便于失败时统一回滚。
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
                // Step 3: 为并发或阻塞场景预构建生产者内核线程（此时仅分配栈与闭包，尚未加入调度队列）。
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

        // Step 4: 若场景使用 EndIo::Task 回调，将当前 Observation 注册到全局静态槽位，随后激活磁盘中断与队列。
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

    /// 检查磁盘在 `prepare` 阶段是否成功激活；若激活失败则返回 `Err(TestError)`，交由框架执行 `quiesce` 与 `tear_down` 回收。
    fn ready(&self) -> TestResult {
        if self.activation_error.is_some() {
            Err(TestError("queue activation failed"))
        } else {
            Ok(())
        }
    }

    /// 读取磁盘扇区 0 的首字节以识别预置的测试镜像身份标签（`1` 或 `2`），供后续扇区数据校验公式使用。
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

    /// 轮询等待硬件 VirtIO 环的 `used.idx` 前进到目标值 `target`。
    fn used(&self, target: usize) -> TestResult {
        until("queue real used index did not progress", || {
            super::tests::used_index(&self.disk, self.legacy) == target as u16
        })
    }

    /// 在持有 `disk.queue` 自旋锁的情况下，通过 `read_volatile` 读取 VirtIO `avail.flags` 字段（用于检查 `VRING_AVAIL_F_NO_INTERRUPT` 位）。
    fn interrupt_flags(&self) -> u16 {
        let q = self.disk.queue.lock();
        let device = q.device.as_ref().expect("live queue ring owner");
        let ring = device.test_virtio().control_regions().0;
        let flags = usize::from(self.disk.limits().queue_depth) * 16;
        crate::arch::dma_read_barrier();
        // SAFETY:
        // 1. 当前持有 `self.disk.queue` 自旋锁，且 `q.device.as_ref().expect(...)` 确认设备环处于存活状态，未与 `stop()` 发生并发释放竞争；
        // 2. 在 VirtIO Split Vring 布局中，`avail.flags` 紧随 `queue_depth` 个 16 字节描述符之后，偏移 `queue_depth * 16` 字节且满足 2 字节对齐，执行只读 `read_volatile` 安全有效。
        u16::from_le(unsafe {
            core::ptr::read_volatile(ring.cpu.as_ptr().add(flags).cast::<u16>())
        })
    }

    /// 将所有预构建的生产者任务发布到调度器开始并发执行，并保存对应的 `JoinHandle`。
    fn publish_tasks(&mut self) {
        for producer in self.prepared.drain(..) {
            self.joins.push(producer.publish());
        }
    }

    /// 等待所有已发布的生产者线程彻底退出内核栈并回收 `JoinHandle`，若任一生产者返回非零状态则报错。
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

    /// 取出下标为 `index` 的预建 `Bio` 并调用 `Disk::submit_bio` 单发提交。
    fn submit(&mut self, index: usize) {
        self.disk.submit_bio(
            self.requests[index]
                .pending
                .take()
                .expect("queue Bio submitted twice"),
        );
    }

    /// 将所有预建 `Bio` 移入预留好容量的 `batch` 向量，并调用 `Disk::submit_batch` 一次性批量提交。
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

    /// 等待磁盘在途槽位数（`inflight`）归零，并刷新完成工作队列中的所有收尾任务。
    fn drained(&self) -> TestResult {
        until("queue slots did not return", || {
            self.disk.test_snapshot().inflight == 0
        })?;
        self.disk.test_flush_completions();
        Ok(())
    }

    /// 验证 8 个独立内核线程并发提交各自的双页读请求：
    /// 先挂起完成处理以确认 8 个请求同时在途且来自 8 个不同的 `Task`，随后恢复完成处理并逐一校验读回的数据图样与物理页归还。
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

    /// 验证请求槽位（`Scenario::Tag`）或 VirtIO 环描述符（`Scenario::Descriptor`）耗尽时的反压阻塞与唤醒：
    /// 先填满 `self.fill` 个请求并挂起完成处理，发布额外生产者线程并确认其在发出硬件通知后进入 `State::Blocked`；
    /// 随后恢复完成处理，验证所有请求顺利完成且被阻塞的生产者线程被唤醒退出。
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

    /// 验证 `VRING_AVAIL_F_NO_INTERRUPT` 中断抑制、重装复查（Rearm）与超出单次中断预算（`IRQ_BUDGET`）时的工作队列续行：
    /// 在挂起完成处理并置位 `NO_INTERRUPT` 期间提交超过 `IRQ_BUDGET` 数量的请求，等待硬件全部写入 `used` 环后恢复完成处理，
    /// 验证通过预算续行机制消费完所有请求并清除 `NO_INTERRUPT` 标志。
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

    /// 验证多页分散/聚集（SG）请求按队列分段与DMA边界限额自动拆分为多个子请求（Child）：
    /// 在子请求尚未处理完毕前，父请求保持未完成且禁止提前取回物理页；恢复完成处理后，父请求聚合成功并完整归还所有物理页。
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

    /// 验证显式批量提交（`submit_batch`）在三种场景下的行为：
    /// - `Scenario::Merge`：三个扇区相邻的请求合并为 1 个硬件请求下发，发生 2 次合并与 1 次硬件通知；
    /// - `Scenario::Notify`：三个扇区不相邻的请求作为 3 个硬件请求下发，但仅在批次末尾执行 1 次硬件通知；
    /// - `Scenario::Tail`：第二个请求因跨越 DMA 边界且不构成完整逻辑块而返回 `BlockError::Unsupported`，首个已入环请求仍被正常通知并完成。
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

    /// 验证 `EndIo::Task(callback)` 异步回调阻塞期间的生命周期不变量：
    /// 当 Worker 线程阻塞在 `Observation::gate` 上时，请求槽位仍在途（`inflight == 1`）、父请求未完成且物理页不可被领取；
    /// 释放 `gate` 后，回调执行完毕退出，父请求随之完成并归还物理页。
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

    /// 验证带 `EndIo::Task` 回调的请求等待同队列配对请求（`peer`）完成时的防死锁机制：
    /// - `Scenario::Progress` / `Scenario::ProgressBatch`：两个请求均通过硬件 `used` 环完成，首个请求的回调阻塞等待第二个请求时，完成推进机制仍能处理第二个请求并唤醒首个回调；
    /// - `Scenario::Generation`：在受控账本（不提交到硬件）上验证首个请求在拆分阶段子请求提前完成时，预留的完成工作项不会阻塞第二个请求的完成。
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

    /// 校验 `EndIo::Task(callback)` 恰好被工作队列的 Worker 线程调用 1 次、入参校验通过且回调已完整执行到末尾退出。
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

    /// 在独占 Scratch 磁盘上验证文件系统挂载占用（`FsClaim`）与裸盘 I/O 的互斥与读写流程：
    /// 1. 持有只读 `FsClaim` 期间，`disk.stop()` 返回 `BlockError::Busy`，且裸盘写请求在准入阶段被拒绝（返回 `BlockError::ReadOnly`）；
    /// 2. 释放 `FsClaim` 后，裸盘写请求、读回校验与 `BlkOperation::Flush` 请求均在硬件上顺利完成。
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

    /// 等待下标为 `index` 的读回请求完成，校验其物理页 DMA 段身份未变、访问权已交还 CPU，且全部字节与预写入图样一致。
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
            // SAFETY:
            // 1. 上方 `request.completion.wait()` 已确认读回请求的所有子请求全部完成，且 `!mapped.is_device_owned()` 确认设备已交还 DMA 访问权；
            // 2. 物理页在分配时已零初始化并由设备 DMA 填入读回字节，当前线程通过 `take_memory()` 独占持有 `memory`，无并发写者。
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

    /// 验证读写模式 `FsClaim` 的授权写入、克隆引用计数、异步回调期间保持挂载占用，以及读回与 `Flush`：
    /// 1. 只读 `FsClaim` 下，裸盘写与通过 `ro.submit_bio` 提交的写请求均被拒绝；
    /// 2. 切换为读写 `FsClaim` 后，克隆并丢弃副本不会提前将 `mounts` 计数清零，且未通过 `FsClaim` 授权的裸盘写仍被拒绝；
    /// 3. 通过读写 `FsClaim` 提交带 `EndIo::Task` 回调的写请求后立即 `drop(claim)`，验证在硬件完成及回调阻塞于 `gate` 期间 `mounts` 仍保持为 1 且拒绝 `disk.stop()`；
    /// 4. 放行 `gate` 待回调彻底退出后，`mounts` 归零，随后重新获取读写 `FsClaim` 完成读回校验与可选的 `Flush`。
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

        // Step 1: 切换为读写 FsClaim，验证克隆引用计数（mounts 保持为 1）以及未授权裸盘写仍被拒绝。
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

        // Step 2: 通过读写 FsClaim 提交带回调的写请求并立即释放句柄，验证在途请求与回调阻塞期间挂载占用（mounts == 1）持续保持。
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

        // Step 3: 释放 gate 使回调退出，确认 mounts 归零后重新申请读写 FsClaim 验证读回数据与 Flush。
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

    /// 验证硬件特性协商为只读（`readonly == true`）的块设备拒绝 `AccessMode::ReadWrite` 挂载占用，但允许 `AccessMode::ReadOnly` 占用并正常读取扇区。
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

    /// 验证 `DiskLimits::validate` 与 `Disk::read_exact_at` 的入口参数校验：
    /// 覆盖扇区越界、总字节数与段长度不匹配、空分段、不支持的请求标志、DMA 传输方向错误、未对齐长度/扇区、只读拒绝写以及非法 `Flush` 参数；
    /// 确认所有非法请求均不向硬件环提交，且 `read_exact_at` 失败时不修改目标缓冲区内容。
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

    /// 验证双磁盘场景下的设备身份（Identity）准入检查：
    /// 将针对主磁盘 `self.disk` 构造的 `Bio` 提交给另一块磁盘 `extra` 时，在准入阶段即以 `BlockError::InvalidRange` 拒绝，不向任何硬件环发布描述符，且物理页正常归还。
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

    /// 验证 `Disk::read_exact_at` 在两个不同扇区（扇区 64 与扇区 96）读取完整逻辑块（支持 512B 至 4KiB），校验读回字节非零、互不相同且完全符合预置扇区图样。
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

    /// 在内存中直接驱动 `BioInner` 受控父请求账本（不向硬件提交），验证四种聚合边界：
    /// - `Ledger::Reverse`：3 个子请求乱序完成，其中偏移 `unit` 的子请求发生短读与 `BlockError::Io`，最终完成字节数截断为首个错误前的连续成功前缀 `unit + unit / 4`；
    /// - `Ledger::MinimumError`：多个子请求同时报错（偏移 `unit` 报 `Io`，偏移 `2 * unit` 报 `ShortIo`），最终保留偏移最小处的错误 `BlockError::Io` 与对应前缀 `unit + unit / 4`；
    /// - `Ledger::GenerationFailure`：第 1 个子请求成功，但父请求在拆分第 2 个子请求时因 `BlockError::NoMemory` 终止生成，最终报告已完成前缀 `unit` 与 `NoMemory`；
    /// - `Ledger::Synchronous`：子请求在父请求调用 `finish_generation` 之前同步完成，验证父请求必须等到 `finish_generation` 释放生产者持有权后才标记完成。
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

    /// 在受控父请求账本上验证页缓存 `Folio` 回写（Writeback）与脏页（Dirty）状态机的联动：
    /// 1. 分配 3 个私有 0 阶 `Folio`，写入初始数据并开启回写（清除 Dirty 位、置位 Writeback 位），构造父 `Bio`；
    /// 2. 在回写期间重新向第 0 个 `Folio` 写入新字节使其再次变脏（并发重脏），随后按 `mode` 驱动子请求完成：
    ///    - `Ledger::WbInline`：子请求在注册循环内同步完成，但在 `finish_generation` 前所有 `Folio` 仍保持 Writeback 状态；
    ///    - `Ledger::WbReverse`：第 2 个子请求报 `BlockError::Io` 但已被队列接受（`accept_child`），回写结束时清除 Writeback 位且不恢复已被接受子请求的旧 Dirty 位；
    ///    - `Ledger::WbRejectedTail`：第 2 个子请求未被队列接受（未调用 `accept_child`）并因 `QueueStopped` 失败，回写结束时自动恢复第 2 个 `Folio` 的 Dirty 位以防丢脏；
    /// 3. 最后调用 `sync_model_dirty` 将所有残余脏 `Folio`（并发重脏的第 0 页，以及 `WbRejectedTail` 下恢复脏位的第 2 页）真正写入 Scratch 磁盘并读回核验。
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
        // Step 1: 分配 3 个私有 0 阶物理页并封装为已完成初始化读取的 FolioRef。
        for _ in 0..3 {
            let page = mm::alloc_pages(0, true).ok_or(TestError("WB ledger original page OOM"))?;
            let folio = mm::FolioRef::from_pages(page)
                .map_err(|_| TestError("WB ledger folio conversion failed"))?;
            let mut read = folio
                .begin_read(0, mm::PAGE_SIZE)
                .map_err(|_| TestError("WB ledger initialization failed"))?;
            // SAFETY:
            // 1. `mm::alloc_pages(0, true)` 已将整页（`PAGE_SIZE` 字节）清零初始化；
            // 2. 该 `folio` 为当前测试函数局部新建，尚未暴露给任何设备或并发线程，`begin_read` 对应的初始化范围已准备就绪。
            unsafe {
                read.complete(IoResult {
                    completed: mm::PAGE_SIZE,
                    error: None,
                })
            };
            self.wb_folios.push(folio);
        }

        // Step 2: 锁定每个 Folio 的内容，写入可区分的测试字节（0x40 + index），并启动回写生成 MappedPages。
        let mut identities = [0usize; 3];
        for (index, folio) in self.wb_folios.iter().enumerate() {
            let mut content = folio.lock_content();
            content
                .copy_from(0, &[0x40 + index as u8; 4096][..unit])
                .map_err(|_| TestError("WB ledger original write failed"))?;
            identities[index] = folio.phys_addr();
            // SAFETY:
            // 1. 当前 `wb_folios` 中的页面仅由本测试夹具私有持有，未映射至任何用户地址空间（无 VMA/PTE）；
            // 2. 当前持有 `content` 锁守卫（排斥其他内核写者），且前一行 `copy_from` 已将 `[0, unit)` 区间完整初始化。
            let wb = unsafe { content.begin_writeback(0, unit) }
                .map_err(|_| TestError("WB ledger begin failed"))?
                .ok_or(TestError("WB ledger lost original Dirty"))?;
            memory.push(
                MappedPages::from_writeback(wb)
                    .map_err(|_| TestError("WB ledger mapping failed"))?,
            );
        }

        // Step 3: 构造受控回写父 Bio，模拟回写期间第 0 个 Folio 发生并发重脏，并按指定模式驱动子请求注册与完成。
        // SAFETY:
        // 1. `memory` 中的每个 `MappedPages` 均由 `MappedPages::from_writeback` 从有效且已初始化的私有 `Folio` 构造，无 VMA 映射或裸指针别名；
        // 2. 该受控父 `Bio` 仅通过 `BioInner` 方法在内存中演练状态机，绝不向硬件环发布 DMA 地址，且在本函数返回前等待其彻底完成并回收 `memory`。
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

        // Step 4: 等待受控父 Bio 完成，回收 MappedPages，核对各 Folio 的 Writeback 与 Dirty 位状态，最后将剩余脏页同步刷入 Scratch 磁盘。
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

    /// 将受控回写账本测试中保留或恢复了 Dirty 位的私有 `Folio` 通过 Scratch 磁盘写请求刷盘，并读回核对完整字节内容。
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
            // SAFETY:
            // 1. 前一个受控父 `Bio` 已在 `writeback_ledger` 中彻底完成并释放了所有 `MappedPages`；
            // 2. 该 `folio` 为当前夹具私有持有（无 VMA/PTE 映射），当前持有 `content` 锁且 `[0, unit)` 范围的数据已完整初始化。
            let wb = unsafe { content.begin_writeback(0, unit) }
                .map_err(|_| TestError("WB scratch sync begin failed"))?
                .ok_or(TestError("WB scratch sync lost Dirty"))?;
            drop(content);
            memory.push(
                MappedPages::from_writeback(wb)
                    .map_err(|_| TestError("WB scratch sync map failed"))?,
            );
            // SAFETY:
            // 1. `memory` 中的回写范围已完整初始化且 `content` 锁已释放，不存在任何 CPU 裸指针或可变引用别名；
            // 2. 提交给 `self.disk` 后，紧接着在下方等待 `complete.wait()` 完成并调用 `take_memory()` 收回物理页访问权，不会提前释放。
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

    /// 验证磁盘调用 `stop()` 进入停止状态后的准入拒绝与幂等性：
    /// 停止后提交新 `Bio` 立即以 `BlockError::QueueStopped` 完成并归还物理页，不向硬件提交任何请求；再次调用 `stop()` 直接返回 `Ok(())`。
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

    /// 用例静默化（Quiesce）流程：
    /// 1. 检查受控账本请求是否已收敛，防止在存在未完成子请求义务时直接销毁夹具；
    /// 2. 放行外部同步门 `gate`，恢复被挂起的完成处理，并将残余脏 `Folio` 同步刷盘；
    /// 3. 丢弃未发布的生产者任务，主动拒绝（`BioInner::reject`）尚未提交的剩余 `Bio` 以唤醒可能正在等待 `peer` 的回调；
    /// 4. 等待所有在途槽位排空，释放 `FsClaim`，停止主/副磁盘队列，等待全部生产者线程退出内核栈并同步关闭工作队列；
    /// 5. 清空全局回调观察槽，并断言队列处于 `QueuePhase::Stopped` 且所有计数器与中断句柄均已归零。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self
            .model_live
            .is_some_and(|index| !self.requests[index].completion.is_complete())
        {
            // 若测试主体在受控账本尚未完成前异常退出，返回错误以保留夹具对象，避免提前释放仍带有未完成引用计数的请求页。
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
        // 若配对请求（peer）尚未提交而测试主体提前退出，主动将其拒绝（QueueStopped），使正在等待 peer 的回调线程能够正常唤醒返回。
        for index in 0..self.requests.len() {
            if self.requests[index].pending.is_some() {
                // 此时尚未发布的生产者已被丢弃，无并发线程访问该 pending Bio；将其取出并通过 BioInner::reject 标记结束以唤醒等待者。
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
            // 此时磁盘已停止且工作队列已同步关闭，所有槽位收尾与回调工作项均已执行完毕退出，可安全清空全局回调槽位。
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

    /// 用例拆卸（Tear Down）流程：
    /// 消费并释放 `Owner` 持有的全部资源，等待所有 CPU 空闲并排空内核缓存与页分配器，断言最终 6 域物理内存统计与用例开始前的 `baseline` 完全一致。
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
/// 受控父请求账本（`Scenario::Model`）的子测试模式枚举。
enum Ledger {
    /// 3 个子请求乱序完成，中间子请求发生短读与 I/O 错误，验证按偏移截断为最短成功前缀。
    Reverse,
    /// 多个子请求在不同偏移处报错，验证保留最小偏移处的错误码与成功前缀。
    MinimumError,
    /// 首个子请求成功，但父请求在继续生成后续子请求时遇到错误，验证生成期错误与子请求结果的合并。
    GenerationFailure,
    /// 子请求在父请求调用 `finish_generation` 之前同步完成，验证生产者持有权对完成时机的保护。
    Synchronous,
    /// 页缓存 `Folio` 回写：子请求在注册循环内同步完成，验证 `finish_generation` 后统一清除 Writeback 位并保留并发重脏位。
    WbInline,
    /// 页缓存 `Folio` 回写：子请求乱序完成且末尾已被队列接受的子请求报 I/O 错误，验证清除 Writeback 位且不恢复已接受子请求的旧脏位。
    WbReverse,
    /// 页缓存 `Folio` 回写：末尾子请求未被队列接受（因 `QueueStopped` 被拒绝），验证回写结束时自动恢复该页的 Dirty 位以防丢失脏数据。
    WbRejectedTail,
}

/// 为指定测试场景生成实现 `ktest::Fixture<DeviceInventory>` 的薄包装结构体。
macro_rules! fixture {
    ($name:ident, $scenario:ident) => {
        /// 绑定到特定 `Scenario` 的测试夹具包装器，内部持有完整的 `Owner` 状态。
        struct $name(Owner);
        impl Fixture<DeviceInventory> for $name {
            /// 根据当前用例所属的 `Scenario` 准备独立的块设备队列、工作队列、物理页与生产者线程。
            fn set_up(
                context: &mut TestContext<'_>,
                resources: &DeviceInventory,
            ) -> Result<Self, TestError> {
                Owner::prepare(Scenario::$scenario, context, resources).map(Self)
            }
            /// 停止设备队列、排空在途请求并等待所有生产者与工作队列线程退出。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// 释放夹具所有权并校验 6 域物理内存统计完全回归基线。
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
    /// 测试目标：验证 8 个独立内核线程并发提交双页读请求时的多请求在途能力与物理页回收。
    /// 场景步骤：挂起完成处理 -> 发布 8 个生产者线程各自提交请求 -> 断言 8 个请求同时在途且线程身份互异 -> 恢复完成处理并等待全部完成。
    /// 预期结果：峰值在途数大于 1，8 个请求的读回数据完全符合扇区图样，且各自的物理页均被准确收回。
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
    /// 测试目标：验证请求槽位（Tag）耗尽时的硬件通知、提交者阻塞（`State::Blocked`）与完成唤醒。
    /// 场景步骤：挂起完成处理并提交 `tag_count` 个请求占满全部槽位 -> 发布第 `tag_count + 1` 个生产者线程 -> 断言该线程触发硬件通知并进入 `State::Blocked` -> 恢复完成处理。
    /// 预期结果：所有 `tag_count + 1` 个请求全部成功完成并校验通过，被阻塞的生产者线程正常唤醒退出。
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
    /// 测试目标：验证在槽位未满但 VirtIO 环描述符（Descriptor）耗尽时的提交者阻塞与完成唤醒。
    /// 场景步骤：使用每个占 4 段 SG 的请求在槽位耗尽前先占满可用描述符 -> 发布额外生产者线程 -> 断言剩余描述符不足 6 个且线程进入 `State::Blocked` -> 恢复完成处理。
    /// 预期结果：所有请求在描述符归还后依次完成，数据校验无误且生产者线程正常退出。
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
    /// 测试目标：验证 `VRING_AVAIL_F_NO_INTERRUPT` 中断抑制、重装复查（Rearm）以及超出单次中断预算（`IRQ_BUDGET`）时的工作队列续行。
    /// 场景步骤：挂起完成处理（此时置位 `NO_INTERRUPT`） -> 提交 `tag_count`（大于 `IRQ_BUDGET`）个请求并等待硬件全部写入 `used` 环 -> 恢复完成处理。
    /// 预期结果：无需新的硬件中断即可通过重装复查与工作队列续行消费完所有请求，最终 `NO_INTERRUPT` 标志被清除。
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
    /// 测试目标：验证跨越多页且超出单请求段数/段字节/DMA边界限额的分散/聚集（SG）请求自动拆分与父请求聚合。
    /// 场景步骤：提交包含 3 个独立物理页的读请求 -> 断言其被拆分为多个硬件子请求且完成前无法取回物理页 -> 恢复完成处理。
    /// 预期结果：所有子请求完成后父请求标记完成，3 个物理页按原顺序收回且扇区数据图样完整匹配。
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
    /// 测试目标：验证同一批次（Batch）中三个扇区连续的父请求自动合并为单个硬件请求下发。
    /// 场景步骤：通过 `submit_batch` 一次性提交 3 个扇区相邻的读请求 -> 等待全部父请求完成并回收各自的物理页。
    /// 预期结果：硬件环仅下发 1 个请求、记录 2 次合并与 1 次队列通知，且 3 个父请求各自独立收回其物理页与正确数据。
    #[test]
    fn three_parents_one_request(context: &mut TestContext<'_>, fixture: &mut Merge) -> TestResult {
        fixture.0.batch(context)
    }
}
#[ktest::suite(fixture = Notify, suite_resources = DeviceInventory, context = "Thread", tags("block", "B09", "T5"), resources("root"), timeout_ms = 15000)]
mod notify {
    use super::*;
    /// 测试目标：验证同一批次中三个扇区不连续的请求仅在批次末尾执行一次硬件队列通知（Unplug）。
    /// 场景步骤：通过 `submit_batch` 提交 3 个扇区间隔较远的读请求 -> 等待全部完成并检查队列统计快照。
    /// 预期结果：硬件环下发 3 个独立请求，但 `notifications` 计数仅增加 1。
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
    /// 测试目标：验证批量提交中后续请求在拆分阶段因对齐/边界限制失败时，前面已入环的请求仍能被正常通知（Unplug）并完成。
    /// 场景步骤：批量提交 2 个请求，其中第 2 个请求页内偏移跨越 DMA 边界且不足一个逻辑块 -> 等待两个请求结束。
    /// 预期结果：第 2 个请求立即以 `BlockError::Unsupported` 失败并归还物理页，第 1 个请求被正常通知硬件并成功读回数据。
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
    /// 测试目标：验证 `EndIo::Task` 异步回调阻塞期间保持槽位与父请求未完成状态，直至回调退出后才释放槽位并完成父请求。
    /// 场景步骤：提交带 `EndIo::Task(callback)` 的请求 -> 等待 Worker 线程在 `gate` 上进入 `State::Blocked` -> 断言此时 `inflight == 1`、父请求未完成且物理页不可取回 -> 放行 `gate`。
    /// 预期结果：回调执行至末尾退出后，父请求完成、物理页收回且槽位归还。
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
    /// 测试目标：在独占 Scratch 磁盘上验证只读 `FsClaim` 的写保护与防停盘保护，以及释放占用后的裸盘写、读回与 `Flush`。
    /// 场景步骤：申请只读 `FsClaim` -> 断言 `disk.stop()` 返回 `Busy` 且裸盘写返回 `ReadOnly` -> 释放 `FsClaim` -> 依次提交裸盘写、读回与 `Flush` 请求。
    /// 预期结果：占用期间未向硬件下发写请求；释放后写入、读回全字节校验与 `Flush` 均成功完成。
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
    /// 测试目标：验证 `DiskLimits::validate` 与 `Disk::read_exact_at` 对各类非法输入参数的同步拦截。
    /// 场景步骤：依次传入越界扇区、长度不匹配、空分段、未知标志位、错误 DMA 方向、未对齐长度/扇区、只读盘写、非法 `Flush` 参数及未对齐 `read_exact_at`。
    /// 预期结果：所有非法调用均返回对应 `BlockError`，硬件提交计数不变，且 `read_exact_at` 失败时不污染目标缓冲区。
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
    /// 测试目标：验证将绑定于主磁盘的 `Bio` 错误提交给另一块磁盘时在准入阶段被拒绝。
    /// 场景步骤：在双磁盘夹具上，将针对 `disk` 构造的 `Bio` 提交给 `extra` 磁盘 -> 回收该请求并检查两块磁盘的提交统计。
    /// 预期结果：请求以 `BlockError::InvalidRange` 结束并正常归还物理页，两块磁盘的硬件提交计数均为 0。
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
    /// 测试目标：验证 `Disk::read_exact_at` 在不同扇区读取完整逻辑块（兼容 512B 至 4KiB）的正确性。
    /// 场景步骤：分别在扇区 64 与扇区 96 调用 `read_exact_at` 读取一个逻辑块 -> 逐字节核对预置扇区公式。
    /// 预期结果：两次读取的数据均非全零、内容互不相同且与扇区图样公式完全一致。
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
    /// 测试目标：在受控父请求账本（不经过硬件环）上验证子请求乱序短读、多错误最小偏移收敛、生成期错误合并以及同步子请求完成。
    /// 场景步骤：按参数 `mode`（`Reverse` / `MinimumError` / `GenerationFailure` / `Synchronous`）手动驱动 `BioInner` 的注册、子请求完成与生成结束 -> 校验最终 `IoResult` 与物理页回收。
    /// 预期结果：父请求在所有子请求与生成阶段均结束后才标记完成，返回的成功前缀字节数与错误码严格符合最小偏移截断规则。
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

    /// 测试目标：在受控父请求账本上验证页缓存 `Folio` 回写（Writeback）与脏页（Dirty）状态转换，并通过 Scratch 磁盘验证残余脏页刷盘。
    /// 场景步骤：构造 3 个私有 `Folio` 启动回写并模拟第 0 页并发重脏 -> 按 `mode`（`WbInline` / `WbReverse` / `WbRejectedTail`）驱动账本结束 -> 校验各页 Dirty/Writeback 位 -> 将残余脏页提交到 Scratch 磁盘同步并读回。
    /// 预期结果：账本结束时所有页的 Writeback 位清零，并发重脏页与未被队列接受的失败页保留 Dirty 位，随后经 Scratch 磁盘写入并读回校验全部字节一致。
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
    /// 测试目标：验证两个独立提交的请求均进入硬件 `used` 环后，第一个请求的 `EndIo::Task` 回调等待第二个请求完成时不会阻塞完成推进。
    /// 场景步骤：挂起完成处理 -> 依次提交请求 0（带回调等待请求 1）与请求 1 -> 等待硬件 `used.idx` 前进 2 -> 触发完成推进工作项。
    /// 预期结果：当请求 0 的回调在 Worker 线程中等待请求 1 时，工作队列继续完成请求 1，使两个请求与回调均顺利完成。
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
    /// 测试目标：验证同一批次（`submit_batch`）提交的两个扇区连续请求中，若首个请求带有 `EndIo::Task` 回调并等待第二个请求，队列不会将二者错误合并而导致死锁。
    /// 场景步骤：挂起完成处理 -> 批量提交扇区相邻的请求 0（带回调等待请求 1）与请求 1 -> 断言下发了 2 个独立硬件请求并等待 `used.idx` 前进 2 -> 触发完成推进。
    /// 预期结果：两个请求作为独立槽位完成，请求 0 的回调顺利等到请求 1 完成并正常退出。
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
    /// 测试目标：在受控账本上验证前一个请求的拆分子请求提前完成时，其预留的完成收尾任务不会阻塞后续配对请求的生成与完成。
    /// 场景步骤：发布生产者线程调用 `generation_pair` 驱动受控账本（请求 0 的回调等待请求 1） -> 等待两个请求与生产者线程完成。
    /// 预期结果：请求 1 顺利完成并唤醒请求 0 的回调，期间不向硬件环提交任何描述符。
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
    /// 测试目标：验证磁盘调用 `stop()` 停止后拒绝新请求准入，且重复调用 `stop()` 保持幂等。
    /// 场景步骤：调用 `disk.stop()` -> 提交新 `Bio` 并验证其完成结果 -> 再次调用 `disk.stop()`。
    /// 预期结果：新请求立即以 `BlockError::QueueStopped` 结束并归还物理页，硬件提交计数不变，第二次 `stop()` 返回 `Ok(())`。
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
    /// 测试目标：验证 Scratch 磁盘上读写 `FsClaim` 的授权写入、克隆引用计数、异步回调阻塞期间保持挂载占用，以及读回与 `Flush`。
    /// 场景步骤：验证只读 `FsClaim` 拒写 -> 申请读写 `FsClaim` 验证克隆计数与未授权裸盘拒写 -> 提交带回调写请求后立即释放 `FsClaim` 句柄，验证回调阻塞在 `gate` 期间 `mounts == 1` 且拒绝 `stop()` -> 放行 `gate` 后重新申请读写 `FsClaim` 执行读回与 `Flush`。
    /// 预期结果：只有在途请求及异步回调彻底退出后 `mounts` 才归零，授权写入的数据经读回逐字节校验完全一致。
    #[test]
    fn authorized_write_readback_and_callback_tail(
        context: &mut TestContext<'_>,
        fixture: &mut FsScratch,
    ) -> TestResult {
        fixture.0.fs_scratch(context)
    }

    /// 测试目标：验证基于扇区间区的 `BlockView` 与 `FsClaim` 范围仲裁：已准入的裸盘写者或已挂载的文件系统视图仅阻塞重叠扇区范围，不阻塞非重叠分区。
    /// 场景步骤：
    /// 1. 构造 `root` `[0, 1024)` 与 `scratch` `[2048, 3072)` 两个不重叠视图；
    /// 2. 在 `scratch` 范围准入一个裸盘写请求（尚未结束），断言此时 `claim_fs(&scratch)` 返回 `Busy`，但 `claim_fs(&root, ReadWrite)` 成功；
    /// 3. 结束该裸盘写者后，验证与 `root` 重叠的视图 `[512, 1536)` 无法被 `claim_fs`，且向 `root_claim` 提交目标在 `scratch` 扇区的请求会因越界被拒绝（`InvalidRange`）；
    /// 4. 向未挂载的 `scratch` 视图直接提交裸盘写请求并等待完成，随后申请 `claim_fs(&scratch, ReadWrite)` 并验证越界 `read_exact_at` 被拦截且不改目标缓冲区。
    /// 预期结果：范围冲突检测精确按扇区区间工作，非重叠分区的裸盘写与文件系统挂载互不干扰。
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
    /// 测试目标：验证硬件协商为只读（`VIRTIO_BLK_F_RO`）的块设备拒绝读写 `FsClaim`，但允许只读 `FsClaim` 执行扇区读取。
    /// 场景步骤：断言 `disk.limits().readonly == true` -> 尝试申请 `ReadWrite` 模式的 `FsClaim` -> 申请 `ReadOnly` 模式的 `FsClaim` 并提交读请求。
    /// 预期结果：读写占用被以 `BlockError::ReadOnly` 拒绝且 `mounts` 保持为 0；只读占用成功完成硬件读取并归还物理页。
    #[test]
    fn negotiated_ro_rejects_rw_claim(
        context: &mut TestContext<'_>,
        fixture: &mut ReadonlyDevice,
    ) -> TestResult {
        fixture.0.readonly_device(context)
    }
}
