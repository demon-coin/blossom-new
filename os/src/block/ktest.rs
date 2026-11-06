//! 块设备停机排空（Draining）、在途请求退休（Retiring）与复位未确认隔离（Quarantined）测试模块。
//!
//! # 模块职责与架构定位
//! 本模块针对独占测试盘（`scratch`）验证 `Disk::stop()` 在三种关键停机场景下的状态机演进、DMA 内存安全以及完成回调生命周期保障：
//! 1. **正常排空停机（[`Scenario::Normal`]）**：当已准入的读请求进入耗时的任务级完成回调（`EndIo::Task`）并阻塞时，并发调用的 `Disk::stop()` 必须将队列转入 `QueuePhase::Draining`、拒绝新提交的请求（返回 `BlockError::QueueStopped`），且 `stop()` 本身必须阻塞等待该任务级回调彻底执行完毕退出后，才执行硬件复位并转入 `QueuePhase::Stopped`。
//! 2. **在途写请求故障退休（[`Scenario::Retirement`]）**：当写请求已提交给硬件但完成环被暂扣时，触发 `Disk::request_retire(BlockError::DeviceFault)` 并调用 `stop()`。验证队列在完成硬件复位（确保设备停止 DMA）后才将该在途写请求以 `DeviceFault` 错误退休并交还物理页访问权，且绝不自动重放结果未知的写操作。
//! 3. **硬件复位未确认隔离（[`Scenario::Withheld`]）**：模拟软件侧未能读到硬件复位确认（`withhold_confirmation = true`），验证 `stop()` 在等待超时后返回 `Err(BlockError::StopUnconfirmed)`，并将队列永久转入 `QueuePhase::Quarantined` 隔离状态，继续保留底层设备、DMA 控制区、在途请求物理页以及中断/工作队列资源的所有权，防止在硬件仍可能进行 DMA 时提前释放内存导致悬垂访问（UAF）。
//!
//! # 停机状态机演进示意图
//! ```text
//!                        +----------------------+
//!                        |   QueuePhase::Live   |
//!                        +----------------------+
//!                           |                |
//!          Disk::stop()     |                |  request_retire(DeviceFault)
//!       (正常排空在途请求)  v                v  (故障中止并请求硬件复位)
//!                +--------------------+    +--------------------+
//!                | QueuePhase::       |    | QueuePhase::       |
//!                | Draining           |    | Retiring           |
//!                +--------------------+    +--------------------+
//!                  | (拒绝新 Bio,            | (发起硬件 reset,
//!                  |  等待在途 slot 与       |  等待 status 归零确认)
//!                  |  EndIo 回调彻底退出)    |
//!                  +------------+------------+
//!                               |
//!            +------------------+------------------+
//!            | 硬件 reset 确认成功                 | 硬件 reset 超时未确认
//!            v                                     v
//!   +----------------------+             +------------------------+
//!   | QueuePhase::Stopped  |             | QueuePhase::Quarantined|
//!   | (收回全部物理页,     |             | (返回 StopUnconfirmed, |
//!   |  注销 IRQ/Work,      |             |  保留全部 DMA/设备资源 |
//!   |  释放底层设备)       |             |  直至隔离虚拟机终止)   |
//!   +----------------------+             +------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 全局回调定位锁 [`CALLBACK_OWNER`]、观察器内部锁（`Observation::callback`、`stop_task`、`stop_result`）与磁盘队列锁（`disk.queue`）均为独立叶级自旋锁，禁止相互嵌套持有。
//! - 查询任务调度状态（`sched::test_state`）必须在释放 `Observation` 自旋锁之后进行，仅持有克隆出的 `Arc<Task>`。

use ::ktest::{self, Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use virtio_drivers::{BufferDirection, MappedSegment};

use super::disk::QueuePhase;
use super::{
    Bio, BioAllocator, BioCompletion, BlkOperation, BlockError, Disk, DiskRef, EndIo, IoResult,
};
use crate::drivers::virtio::{
    VirtioDevice, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN, VIRTIO_VENDOR_ID,
};
use crate::mm::dma::MappedPages;
use crate::sync::{Completion, SpinLock};
use crate::task::{self, JoinHandle, PreparedTask, State, Task};
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, drivers, mm, sched, time, timer};

/// 全局静态槽：用于在 `EndIo::Task(held_callback)` 函数指针回调中定位当前测试用例的 [`Observation`] 实例。
///
/// 由于 `EndIo::Task` 仅接收无捕获的裸函数指针 `fn(&IoResult, &[MappedPages])`，`Normal` 场景在启动前将当前 `Arc<Observation>` 存入此自旋锁槽位，并在停机且工作队列同步关闭后将其取出清空。
static CALLBACK_OWNER: SpinLock<Option<Arc<Observation>>> = SpinLock::new(None);

/// 块设备停机测试的三种子场景枚举。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    /// 正常停机排空：等待在途读请求的任务级 `EndIo` 回调执行完毕后再完成硬件复位与停机。
    Normal,
    /// 在途写请求故障退休：触发 `DeviceFault` 后先完成硬件复位，再报错退休在途写请求并收回物理页。
    Retirement,
    /// 硬件复位未确认隔离：扣留复位确认读取，验证超时后返回 `StopUnconfirmed` 并保留全部 DMA 资源。
    Withheld,
}

/// 停机测试过程中的跨线程同步与状态观测容器。
struct Observation {
    /// 控制 `held_callback` 解除阻塞并继续执行至退出的完成门控。
    release_callback: Completion,
    /// 记录进入 `held_callback` 的任务句柄、回调收到的 `IoResult` 以及回调执行时物理页是否已收回 CPU 访问权。
    callback: SpinLock<Option<(Arc<Task>, IoResult, bool)>>,
    /// `held_callback` 被调用的总次数。
    callback_calls: AtomicUsize,
    /// 标记 `held_callback` 是否已跨过 `release_callback.wait()` 并执行到函数末尾。
    callback_tail: AtomicBool,
    /// 独立停机内核线程（执行 `Disk::stop()`）的任务句柄，用于外部轮询其是否进入 `State::Blocked`。
    stop_task: SpinLock<Option<Arc<Task>>>,
    /// 独立停机内核线程执行 `Disk::stop()` 返回的最终结果。
    stop_result: SpinLock<Option<Result<(), BlockError>>>,
}

impl Observation {
    /// 在测试开始前预先分配并初始化观测容器。
    fn prepare() -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            release_callback: Completion::new(),
            callback: SpinLock::new(None),
            callback_calls: AtomicUsize::new(0),
            callback_tail: AtomicBool::new(false),
            stop_task: SpinLock::new(None),
            stop_result: SpinLock::new(None),
        })
        .map_err(|_| TestError("stop observation allocation failed"))
    }
    /// 检查执行 `Disk::stop()` 的独立内核线程当前是否已在调度器中进入 `State::Blocked` 阻塞态。
    fn stop_blocked(&self) -> bool {
        let task = self.stop_task.lock().clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 检查执行 `held_callback` 的工作队列任务当前是否已在调度器中进入 `State::Blocked` 阻塞态。
    fn callback_blocked(&self) -> bool {
        let task = self.callback.lock().as_ref().map(|entry| entry.0.clone());
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

/// `Normal` 场景使用的任务级完成回调（`EndIo::Task`）：记录回调收到的结果与物理页归属状态，随后在 `release_callback` 上阻塞等待测试主线程放行。
fn held_callback(result: &IoResult, memory: &[MappedPages]) {
    let owner = CALLBACK_OWNER
        .lock()
        .as_ref()
        .expect("normal stop callback owner")
        .clone();
    owner.callback_calls.fetch_add(1, Ordering::AcqRel);
    *owner.callback.lock() = Some((
        task::current_task(),
        *result,
        memory.len() == 1 && !memory[0].is_device_owned(),
    ));
    owner.release_callback.wait();
    owner.callback_tail.store(true, Ordering::Release);
}

/// 在 5 秒单调时钟期限内轮询等待条件 `ready` 成立，每次未就绪时睡眠 1 毫秒。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("stop observation deadline overflow"))?;
    while !ready() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 等待所有其他 CPU 核进入 idle 空闲态，收缩全局内存缓存并排空每 CPU 页池后返回六域物理内存统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("stop resources did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 从设备发现清单中查找并初始化唯一挂载的 `scratch` 测试块设备（MMIO 或 PCI）。
fn scratch() -> Result<VirtioDevice, TestError> {
    let inventory = drivers::discover().map_err(|_| TestError("scratch discovery failed"))?;
    let mut pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    if inventory.mmio_blocks.len() + pci.clone().count() != 1 {
        return Err(TestError(
            "stop fixture requires exactly one attached scratch disk",
        ));
    }

    if let Some(candidate) = inventory.mmio_blocks.first() {
        // SAFETY:
        // 1. 宿主机测试环境为本用例独占挂载唯一一块 scratch 块设备，其 MMIO 映射窗口在内核生命周期内有效。
        // 2. 当前测试夹具是该设备的唯一所有者，不存在并发驱动实例。
        unsafe { VirtioDevice::prepare_mmio(candidate) }
            .map_err(|_| TestError("scratch MMIO preparation failed"))
    } else {
        let device = pci.next().ok_or(TestError("scratch PCI device missing"))?;
        // SAFETY:
        // 1. 该 PCI 设备是枚举清单中唯一的 VirtIO 块设备，其 BAR 映射已完成初始化。
        // 2. 测试夹具独占持有该设备直至完成硬件复位与停机清理。
        unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
            .map_err(|_| TestError("scratch PCI preparation failed"))
    }
}

/// 分配 1 个独立物理页并构造指定操作类型（`Read` 或 `Write`）的单逻辑块 `Bio` 请求，同时返回其 DMA 映射段信息以便后续核对。
fn bio(
    disk: &Disk,
    operation: BlkOperation,
    callback: Option<EndIo>,
) -> Result<(Bio, MappedSegment), TestError> {
    let unit = disk.limits().logical_block_size as usize;
    if unit > mm::PAGE_SIZE {
        return Err(TestError("scratch logical block exceeds original page"));
    }

    let mut pages =
        mm::alloc_pages(0, true).ok_or(TestError("scratch original page allocation failed"))?;
    if operation == BlkOperation::Write {
        // SAFETY:
        // 1. `pages` 刚刚由 `mm::alloc_pages(0, true)` 分配且尚未建立 DMA 映射或提交给设备，当前函数拥有其唯一独占访问权。
        // 2. 前置检查已确保 `unit <= mm::PAGE_SIZE`，因此在 `[0..unit)` 范围内写入 `0x5a` 不会越界。
        unsafe { core::slice::from_raw_parts_mut(pages.as_mut_ptr().cast::<u8>(), unit) }
            .fill(0x5a);
    }
    let direction = if operation == BlkOperation::Write {
        BufferDirection::DriverToDevice
    } else {
        BufferDirection::DeviceToDriver
    };
    let mapped = MappedPages::new(pages, 0, unit, direction)
        .map_err(|_| TestError("scratch original page mapping failed"))?;
    let original = mapped
        .segment(crate::mm::dma::DmaDomain::qemu_coherent())
        .expect("QEMU fixture mapping must be reachable");
    let mut memory = Vec::new_in(BioAllocator::Heap);
    memory
        .try_reserve_exact(1)
        .map_err(|_| TestError("scratch owner vector allocation failed"))?;
    memory.push(mapped);

    // SAFETY:
    // 1. 物理页唯一所有权已封装进 `memory` 向量中的 `MappedPages`，写请求的数据区已完整初始化为 `0x5a`，不存在任何 CPU 裸指针别名。
    // 2. 挂载的 `EndIo::Task(held_callback)` 仅读取已完成请求的结果并等待外部测试线程显式放行，不会在回调内部重入调用 `Disk::stop()`。
    let bio = unsafe { Bio::try_new(disk, operation, 128, memory, 0, callback) }.map_err(|_| {
        TestError("scratch Bio preparation failed (Write requires writable scratch)")
    })?;
    Ok((bio, original))
}

/// 块设备停机测试的通用资源所有者与场景驱动器。
struct Owner {
    /// 当前运行的停机测试子场景（`Normal`、`Retirement` 或 `Withheld`）。
    scenario: Scenario,
    /// 测试开始前的六域物理内存统计基线。
    baseline: mm::MemoryStats,
    /// 底层 VirtIO 设备的环形队列（ring）与请求头尾控制区（pdu）的 DMA 地址及长度快照 `(ring_addr, ring_len, pdu_addr, pdu_len)`。
    controls: (u64, usize, u64, usize),
    /// 被测 scratch 块设备的强引用句柄。
    disk: DiskRef,
    /// 承载块设备完成回调与进度任务的独立工作队列。
    queue: WorkQueue,
    /// 跨线程同步与状态观测容器。
    observation: Arc<Observation>,
    /// 预构造的主测试在途请求（提交后置为 `None`）。
    pending: Option<Bio>,
    /// 主测试在途请求的完成跟踪句柄。
    completion: BioCompletion,
    /// 主测试在途请求物理页的原始 DMA 映射段信息。
    original: MappedSegment,
    /// 预构造的用于在 `Draining` 阶段验证拒绝新请求的第二条只读 `Bio` 请求。
    rejected: Option<Bio>,
    /// 被拒绝请求的完成跟踪句柄。
    rejected_completion: BioCompletion,
    /// 被拒绝请求物理页的原始 DMA 映射段信息。
    rejected_original: MappedSegment,
    /// 预先在 CPU 1 上准备好的独立停机任务（发布后置为 `None`）。
    prepared_stop: Option<PreparedTask>,
    /// 已发布的停机任务连接句柄（`JoinHandle`），用于等待 `Disk::stop()` 线程退出。
    stop_join: Option<JoinHandle>,
    /// 记录 `set_up` 阶段调用 `Disk::start()` 时可能发生的错误。
    activation_error: Option<BlockError>,
}

impl Owner {
    /// 为指定测试场景准备完整的测试夹具：初始化独占 scratch 设备、工作队列、`Disk` 实例、预构造 `Bio` 请求及独立停机线程，并启动设备。
    fn prepare(scenario: Scenario, context: &mut TestContext<'_>) -> Result<Self, TestError> {
        // Step 1: 获取内存基线并发现独占 scratch 设备；若为写退休场景则校验设备可写。
        let baseline = settled()?;
        let device = scratch()?;
        if scenario == Scenario::Retirement && device.info().readonly {
            return Err(TestError(
                "uncertain_write requires writable scratch; no raw write issued",
            ));
        }

        // Step 2: 记录底层 DMA 控制区地址快照，创建完成工作队列并初始化 `Disk` 实例。
        let (ring, pdu) = device.control_regions();
        let controls = (ring.device.value(), ring.len, pdu.device.value(), pdu.len);
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(device.info().tag_count) + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("scratch completion domain preparation failed"))?;
        let disk = match Disk::prepare(device, &queue) {
            Ok(disk) => disk,
            Err(_) => {
                queue.close_sync();
                return Err(TestError("scratch Disk preparation failed"));
            }
        };

        // Step 3: 在启动硬件前预先分配观测容器、主请求、将被拒绝的请求以及绑定在 CPU 1 上的停机内核线程。
        let resources = (|| {
            let observation = Observation::prepare()?;
            let operation = if scenario == Scenario::Retirement {
                BlkOperation::Write
            } else {
                BlkOperation::Read
            };
            let callback = if scenario == Scenario::Normal {
                Some(EndIo::Task(held_callback))
            } else {
                None
            };
            let (pending, original) = bio(&disk, operation, callback)?;
            let (rejected, rejected_original) = bio(&disk, BlkOperation::Read, None)?;
            let stop_disk = disk.clone();
            let stop_observation = observation.clone();
            let prepared_stop = task::prepare_kernel_on(1, move || {
                *stop_observation.stop_task.lock() = Some(task::current_task());
                let result = stop_disk.stop();
                *stop_observation.stop_result.lock() = Some(result);
                0
            })
            .map_err(|_| TestError("independent stop Task preparation failed"))?;
            Ok((
                observation,
                pending,
                original,
                rejected,
                rejected_original,
                prepared_stop,
            ))
        })();
        let (observation, pending, original, rejected, rejected_original, prepared_stop) =
            match resources {
                Ok(resources) => resources,
                Err(error) => {
                    // 若预分配阶段失败，此时设备尚未启动（start），直接调用 stop() 释放预留资源并同步关闭工作队列。
                    disk.stop()
                        .map_err(|_| TestError("unpublished scratch stop failed"))?;
                    drop(disk);
                    queue.close_sync();
                    return Err(error);
                }
            };

        let completion = pending.completion();
        let rejected_completion = rejected.completion();
        if scenario == Scenario::Normal {
            let mut locator = CALLBACK_OWNER.lock();
            assert!(
                locator.is_none(),
                "previous normal stop callback still alive"
            );
            *locator = Some(observation.clone());
        }

        // Step 4: 启动块设备；若启动失败仍返回 `Owner` 实例，交由 `quiesce()` 执行规范的停机清理。
        let activation_error = disk.start().err();
        if let Some(error) = activation_error {
            context.diagnostic(format_args!(
                "scratch activation failed: {:?}; retaining owner for quiesce",
                error
            ));
        }
        Ok(Self {
            scenario,
            baseline,
            controls,
            disk,
            queue,
            observation,
            pending: Some(pending),
            completion,
            original,
            rejected: Some(rejected),
            rejected_completion,
            rejected_original,
            prepared_stop: Some(prepared_stop),
            stop_join: None,
            activation_error,
        })
    }

    /// 发布预先准备好的独立停机内核线程，使其并发调用 `Disk::stop()`。
    fn start_stop(&mut self) {
        self.stop_join = Some(
            self.prepared_stop
                .take()
                .expect("stop Task published twice")
                .publish(),
        );
    }

    /// 校验 `BioCompletion` 的完成结果等于 `expected`，收回物理页 DMA 访问权并核对 DMA 映射地址、长度、方向以及不可二次重复领取。
    fn reclaim(
        context: &mut TestContext<'_>,
        completion: &BioCompletion,
        original: MappedSegment,
        expected: IoResult,
    ) -> TestResult {
        context.expect_eq(completion.result(), Some(expected));
        let memory = completion
            .take_memory()
            .map_err(|_| TestError("original Bio page not reclaimable"))?;
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
        }
        context.expect_eq(completion.take_memory().err(), Some(BlockError::Busy));
        drop(memory);
        Ok(())
    }

    /// 执行正常停机排空测试（`Scenario::Normal`）：验证 `Disk::stop()` 在 `QueuePhase::Draining` 期间拒绝新请求，并阻塞等待已进入任务级回调的在途读请求彻底执行完毕后才完成硬件复位与停机。
    fn normal(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("scratch activation failed"));
        }
        // Step 1: 暂扣完成环处理，提交带 `held_callback` 的读请求后再放行完成环，确保请求生成期已结束且工作队列线程阻塞在 `held_callback` 内部。
        self.disk.ktest_hold_used();
        self.disk
            .submit_bio(self.pending.take().expect("prepared normal Read"));
        self.disk.ktest_release_used();
        until("real Task callback did not Block", || {
            self.observation.callback_blocked()
        })?;

        // Step 2: 启动独立停机线程，等待队列进入 `QueuePhase::Draining` 且停机线程进入 `State::Blocked` 阻塞态，核对此时在途槽位与停机统计计数。
        self.start_stop();
        until("normal stop did not enter Draining and Block", || {
            let draining = self.disk.queue.lock().phase == QueuePhase::Draining;
            draining && self.observation.stop_blocked()
        })?;
        {
            let q = self.disk.queue.lock();
            context.expect_eq(q.active_slots, 1);
            context.expect_eq(q.ktest_stop.published, 1);
            context.expect_eq(q.ktest_stop.finished, 0);
            context.expect_eq(q.ktest_stop.reset_requests, 0);
        }
        context.expect_eq(self.completion.is_complete(), false);
        context.expect_eq(
            self.observation.callback_tail.load(Ordering::Acquire),
            false,
        );
        context.expect_eq(*self.observation.stop_result.lock(), None);

        // Step 3: 在 `Draining` 期间提交第二条读请求，验证其被立即以 `BlockError::QueueStopped` 拒绝并直接归还物理页。
        self.disk
            .submit_bio(self.rejected.take().expect("prebuilt Draining rejection"));
        until("Draining rejection did not complete", || {
            self.rejected_completion.is_complete()
        })?;
        Self::reclaim(
            context,
            &self.rejected_completion,
            self.rejected_original,
            IoResult {
                completed: 0,
                error: Some(BlockError::QueueStopped),
            },
        )?;
        context.expect_eq(self.disk.queue.lock().ktest_stop.published, 1);

        // Step 4: 唤醒阻塞中的 `held_callback` 使其执行至退出，等待在途请求完成与停机线程汇合，最后核对物理页回收与 `QueuePhase::Stopped` 最终状态。
        self.observation.release_callback.complete();
        until("accepted callback and stop tail did not finish", || {
            self.completion.is_complete()
                && self
                    .stop_join
                    .as_ref()
                    .is_some_and(|join| join.try_join() == Some(0))
        })?;
        context.expect_eq(self.observation.callback_calls.load(Ordering::Acquire), 1);
        context.expect_eq(self.observation.callback_tail.load(Ordering::Acquire), true);
        let callback = self.observation.callback.lock();
        context.expect_eq(
            callback.as_ref().map(|entry| (entry.1, entry.2)),
            Some((
                IoResult {
                    completed: self.original.length as usize,
                    error: None,
                },
                true,
            )),
        );
        drop(callback);
        Self::reclaim(
            context,
            &self.completion,
            self.original,
            IoResult {
                completed: self.original.length as usize,
                error: None,
            },
        )?;
        self.check_stopped(context, Ok(()));
        context.diagnostic(format_args!("normal-stop scope=real-Read-heavy-Task-EndIo draining=1 stop-real-Blocked=1 callback-real-Blocked=1 accepted-tail=1 reset-confirmed=1"));
        Ok(())
    }

    /// 执行在途写请求故障退休测试（`Scenario::Retirement`）：验证写请求提交硬件后遭遇 `DeviceFault` 时，系统先完成硬件复位再报错收回物理页，且不重放结果未知的写请求。
    fn retirement(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("scratch activation failed"));
        }
        // Step 1: 暂扣完成环消费并提交写请求，校验请求已发布至硬件且 `writing_slots` 计数为 1。
        self.disk.ktest_hold_used();
        self.disk
            .submit_bio(self.pending.take().expect("prepared uncertain Write"));
        {
            let q = self.disk.queue.lock();
            context.expect_eq(q.ktest_stop.published, 1);
            context.expect_eq(q.active_slots, 1);
            context.expect_eq(q.writing_slots, 1);
        }
        context.expect_eq(self.completion.is_complete(), false);

        // Step 2: 触发 `DeviceFault` 故障退休并启动停机线程，等待硬件复位完成且在途写请求收敛结束。
        self.disk.request_retire(BlockError::DeviceFault);
        self.start_stop();
        until("controlled Write retirement did not converge", || {
            self.completion.is_complete()
                && self
                    .stop_join
                    .as_ref()
                    .is_some_and(|join| join.try_join() == Some(0))
        })?;

        // Step 3: 验证写请求以 `BlockError::DeviceFault`（完成 0 字节）报错返回并交还物理页访问权，且队列最终进入 `QueuePhase::Stopped`。
        Self::reclaim(
            context,
            &self.completion,
            self.original,
            IoResult {
                completed: 0,
                error: Some(BlockError::DeviceFault),
            },
        )?;
        self.check_stopped(context, Err(BlockError::DeviceFault));
        context.diagnostic(format_args!("uncertain-write scope=controlled-completion-retirement actual-device-may-have-written=1 hardware-write-failure=not-claimed published=1 neverReplay=1 error-once=1 accepted-tail=1 real-reset-confirmed=1"));
        Ok(())
    }

    /// 执行硬件复位未确认隔离测试（`Scenario::Withheld`）：模拟软件侧无法读取到硬件复位确认，验证 `Disk::stop()` 超时返回 `BlockError::StopUnconfirmed` 并将队列转入 `QueuePhase::Quarantined` 隔离态以保留所有 DMA 资源。
    fn withheld(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("scratch activation failed"));
        }
        // Step 1: 暂扣完成环并设置 `withhold_confirmation = true` 拦截复位确认，提交读请求后触发故障退休与停机。
        self.disk.ktest_hold_used();
        self.disk.queue.lock().ktest_stop.withhold_confirmation = true;
        self.disk
            .submit_bio(self.pending.take().expect("prepared retained Read"));
        self.disk.request_retire(BlockError::DeviceFault);
        self.start_stop();
        until(
            "withheld confirmation did not return StopUnconfirmed",
            || {
                self.stop_join
                    .as_ref()
                    .is_some_and(|join| join.try_join() == Some(0))
            },
        )?;

        // Step 2: 验证 `Disk::stop()` 返回 `Err(BlockError::StopUnconfirmed)`，且底层设备、控制区、在途物理页及中断资源均被保留在隔离态中未释放。
        context.expect_eq(
            *self.observation.stop_result.lock(),
            Some(Err(BlockError::StopUnconfirmed)),
        );
        self.retained(context);
        context.diagnostic(format_args!("stop-unconfirmed scope=controlled-confirmation-withheld hardware-refusal=not-claimed actual-reset-request=1 confirmation-read=withheld original-bounded-owner=retained case-abort-required=1"));
        Ok(())
    }

    /// 断言块设备已成功完成硬件复位并彻底进入 `QueuePhase::Stopped` 状态，所有在途计数清零且中断已注销。
    fn check_stopped(&self, context: &mut TestContext<'_>, expected: Result<(), BlockError>) {
        context.expect_eq(*self.observation.stop_result.lock(), Some(expected));
        let q = self.disk.queue.lock();
        context.expect_eq(q.phase, QueuePhase::Stopped);
        context.expect_eq(q.device.is_none(), true);
        context.expect_eq(
            (
                q.generators,
                q.active_slots,
                q.notify_active,
                q.writing_slots,
            ),
            (0, 0, 0, 0),
        );
        context.expect_eq((q.ktest_stop.published, q.ktest_stop.finished), (1, 1));
        context.expect_eq(
            (
                q.ktest_stop.reset_requests,
                q.ktest_stop.reset_confirmations,
            ),
            (1, 1),
        );
        drop(q);
        context.expect_eq(self.disk.irq.lock().is_none(), true);
    }

    /// 断言块设备在复位未确认后处于 `QueuePhase::Quarantined` 隔离状态，底层设备、DMA 控制区、在途物理页、中断 token 及工作项均保持存活未释放。
    fn retained(&self, context: &mut TestContext<'_>) {
        let q = self.disk.queue.lock();
        context.expect_eq(q.phase, QueuePhase::Quarantined);
        context.expect_eq(q.failure, Some(BlockError::StopUnconfirmed));
        context.expect_eq((q.ktest_stop.published, q.ktest_stop.finished), (1, 0));
        context.expect_eq(
            (
                q.ktest_stop.reset_requests,
                q.ktest_stop.reset_confirmations,
            ),
            (1, 0),
        );
        context.expect_eq(q.active_slots, 1);
        context.expect_eq(q.device.as_ref().unwrap().test_inflight(), 1);
        context.expect_eq(
            q.device
                .as_ref()
                .map(|backend| backend.test_virtio().requires_reset()),
            Some(true),
        );
        let controls = q.device.as_ref().map(|device| {
            let (ring, pdu) = device.test_virtio().control_regions();
            (ring.device.value(), ring.len, pdu.device.value(), pdu.len)
        });
        context.expect_eq(controls, Some(self.controls));
        drop(q);
        context.expect_eq(self.disk.irq.lock().is_some(), true);
        context.expect_eq(self.disk.progress_work.is_some(), true);
        context.expect_eq(self.completion.is_complete(), false);
        context.expect_eq(self.completion.take_memory().err(), Some(BlockError::Busy));
    }

    /// 停机清理阶段：放行可能阻塞的回调与完成环，等待独立停机线程退出，执行 `Disk::stop()` 并同步关闭工作队列（若处于复位未确认隔离态则返回错误触发用例级虚拟机中止以保护 DMA 内存）。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 放行回调门控与完成环暂扣，丢弃未发布的预备任务并等待已发布的停机线程安全退出。
        self.observation.release_callback.complete();
        if self.scenario != Scenario::Withheld {
            self.disk.ktest_release_used();
        }
        // 若停机任务尚未发布，直接在此处析构以提前释放其持有的 `DiskRef`。
        drop(self.prepared_stop.take());
        if let Some(join) = self.stop_join.as_ref() {
            until("stop Task cannot safely leave its stack", || {
                join.try_join().is_some()
            })?;
        }
        if let Some(join) = self.stop_join.take() {
            if join.join() != 0 {
                return Err(TestError("stop Task returned failure"));
            }
        }

        // Step 2: 确保块设备已完成停机；若为 `StopUnconfirmed` 隔离状态，校验资源保留后返回错误以触发隔离虚拟机终止。
        match self.disk.stop() {
            Ok(()) | Err(BlockError::DeviceFault) => {}
            Err(BlockError::StopUnconfirmed) => {
                self.retained(context);
                return Err(TestError("controlled-confirmation-withheld: StopUnconfirmed; retaining original Disk, DMA and WorkQueue owner until case-abort shutdown"));
            }
            Err(_) => return Err(TestError("scratch stop cannot safely converge")),
        }
        if self.scenario == Scenario::Normal {
            // `Disk::stop()` 已同步等待所有槽位工作项退出，此时可安全清空全局回调定位槽。
            let locator = { CALLBACK_OWNER.lock().take() };
            drop(locator);
        }
        self.queue.close_sync();
        Ok(())
    }

    /// 销毁测试夹具并校验全局六域物理内存统计完全恢复至测试前基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        let mut after = settled()?;
        until("scratch six-domain resources did not return", || {
            mm::shrink_all();
            mm::drain_all_pages();
            after = mm::stats();
            after == baseline
        })?;
        context.expect_eq(after, baseline);
        context.diagnostic(format_args!("stop-resources six-domains-before={:?} six-domains-after={:?} real-stop-join-cleanup=1", baseline, after));
        Ok(())
    }
}

/// 正常停机排空测试场景的夹具包装类型。
struct NormalStop(Owner);
/// 在途写请求故障退休测试场景的夹具包装类型。
struct UncertainWrite(Owner);
/// 硬件复位未确认隔离测试场景的夹具包装类型。
struct StopUnconfirmed(Owner);

macro_rules! fixture {
    ($fixture:ident, $scenario:ident) => {
        impl Fixture<()> for $fixture {
            /// 为当前测试用例构造独立的停机测试夹具实例。
            fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
                Owner::prepare(Scenario::$scenario, context).map(Self)
            }
            /// 停止被测块设备、放行回调并等待独立停机线程退出。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// 析构测试夹具并校验六域物理内存资源无泄漏。
            fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
                self.0.tear_down(context)
            }
        }
    };
}
fixture!(NormalStop, Normal);
fixture!(UncertainWrite, Retirement);
fixture!(StopUnconfirmed, Withheld);

#[ktest::suite(fixture = NormalStop, context = "Thread", min_cpus = 2, tags("block", "stop", "B11", "T5"), resources("scratch"), timeout_ms = 15000)]
mod normal {
    use super::*;
    /// **测试目标**：验证当已准入的请求正在执行耗时的任务级完成回调（`EndIo::Task`）时，`Disk::stop()` 能正确进入 `Draining` 状态拒绝新请求，并阻塞等待回调彻底退出后才完成硬件复位与停机。
    /// **场景步骤**：
    /// 1. 提交挂载了 `held_callback` 的读请求，等待工作队列线程进入 `held_callback` 并阻塞。
    /// 2. 在另一内核线程调用 `Disk::stop()`，验证队列进入 `QueuePhase::Draining` 且停机线程进入 `State::Blocked`。
    /// 3. 在 `Draining` 期间提交新读请求，验证其立即以 `BlockError::QueueStopped` 失败并交还物理页。
    /// 4. 放行 `held_callback`，等待在途请求完成与 `Disk::stop()` 返回 `Ok(())`。
    /// **预期结果**：在途请求与被拒绝请求的物理页 DMA 访问权均被正常收回，队列最终进入 `QueuePhase::Stopped` 且硬件复位确认计数为 1。
    #[test]
    fn normal_stop_accepted(context: &mut TestContext<'_>, fixture: &mut NormalStop) -> TestResult {
        fixture.0.normal(context)
    }
}

#[ktest::suite(fixture = UncertainWrite, context = "Thread", min_cpus = 2, tags("block", "stop", "B11", "T5"), resources("scratch"), timeout_ms = 15000)]
mod retirement {
    use super::*;
    /// **测试目标**：验证在途写请求遭遇 `DeviceFault` 故障退休时，队列必须在完成硬件复位后才报错收回写请求物理页，且绝不自动重放结果未知的写操作。
    /// **场景步骤**：
    /// 1. 暂扣完成环并向可写 scratch 设备提交 1 个逻辑块的写请求，验证请求已发布且 `writing_slots == 1`。
    /// 2. 调用 `Disk::request_retire(BlockError::DeviceFault)` 并启动独立停机线程执行 `Disk::stop()`。
    /// 3. 等待写请求完成通知与停机线程结束。
    /// **预期结果**：写请求仅向硬件发布 1 次（不重放），在硬件复位确认后以 `BlockError::DeviceFault`（完成 0 字节）退休并交还物理页，`Disk::stop()` 返回 `Err(BlockError::DeviceFault)`。
    #[test]
    fn uncertain_write(context: &mut TestContext<'_>, fixture: &mut UncertainWrite) -> TestResult {
        fixture.0.retirement(context)
    }
}

#[ktest::suite(fixture = StopUnconfirmed, context = "Thread", isolation = "Case", min_cpus = 2, tags("block", "stop", "B11", "T5", "negative"), resources("scratch"), timeout_ms = 15000)]
mod confirmation {
    use super::*;
    /// **测试目标**：验证当硬件复位状态确认被扣留（模拟硬件未确认复位）时，`Disk::stop()` 超时后返回 `Err(BlockError::StopUnconfirmed)` 并将设备转入 `QueuePhase::Quarantined` 隔离状态，持续保留所有 DMA 资源以防止悬垂 DMA 破坏内存。
    /// **场景步骤**：
    /// 1. 暂扣完成环并将 `ktest_stop.withhold_confirmation` 设为 `true`，提交读请求后触发 `request_retire` 与 `Disk::stop()`。
    /// 2. 等待 2 秒复位轮询超时后停机线程返回。
    /// **预期结果**：`Disk::stop()` 返回 `Err(BlockError::StopUnconfirmed)`，队列处于 `QueuePhase::Quarantined`，底层设备、DMA 控制区、在途请求物理页及中断注册项均保持存活不被释放，随后由 `quiesce` 返回错误终止隔离测试虚拟机。
    #[test]
    #[ignore = "controlled-confirmation-withheld intentionally cannot quiesce: case-abort Error terminates the isolated VM while retaining original DMA owners"]
    fn stop_unconfirmed(
        context: &mut TestContext<'_>,
        fixture: &mut StopUnconfirmed,
    ) -> TestResult {
        fixture.0.withheld(context)
    }
}
