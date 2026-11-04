//! 真实 scratch 停机用例；受控退休和拒读确认不冒充硬件写故障或硬件拒绝 reset。

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

// EndIo 只接受函数指针；此有界槽仅定位当前 normal 例的强拥有者。
// gate 属于每次新建的 Observation，清理等真实回调尾部后摘槽，不保存跨例 GO 状态。
static CALLBACK_OWNER: SpinLock<Option<Arc<Observation>>> = SpinLock::new(None);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    Normal,
    Retirement,
    Withheld,
}

struct Observation {
    release_callback: Completion,
    callback: SpinLock<Option<(Arc<Task>, IoResult, bool)>>,
    callback_calls: AtomicUsize,
    callback_tail: AtomicBool,
    stop_task: SpinLock<Option<Arc<Task>>>,
    stop_result: SpinLock<Option<Result<(), BlockError>>>,
}

impl Observation {
    /// 所有控制存储在硬件发表前 fallible 预备，异步入口只克隆现存拥有者。
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

    /// 拷贝强拥有后锁外读调度器事实，不以 entered 或 timer 睡眠冒充 Blocked。
    fn stop_blocked(&self) -> bool {
        let task = self.stop_task.lock().clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 此回调自己的原生 Task 确实 park，且尚未返回其完成 Work。
    fn callback_blocked(&self) -> bool {
        let task = self.callback.lock().as_ref().map(|entry| entry.0.clone());
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

/// 真实 Read 已收权后停在 Task EndIo 尾部，不等待自身 Bio、stop 或同槽复用。
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

/// 有限单调期限只等待确切生产谓词，sleep 不构成任何阻塞或完成证据。
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

/// 无参与者再使用资源后整理真实空页缓存和 slab，保存完整六域页账。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("stop resources did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 唯一 attached 块设备必须来自本例声明的独占 scratch，不能猜第一根盘并原始写入。
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
        // SAFETY: host 为本例只挂独占 scratch；一次真实库存和永久映射有效，无其它 owner。
        unsafe { VirtioDevice::prepare_mmio(candidate) }
            .map_err(|_| TestError("scratch MMIO preparation failed"))
    } else {
        let device = pci.next().ok_or(TestError("scratch PCI device missing"))?;
        // SAFETY: 同一真实库存中的唯一 scratch BDF；fixture 保活至真实 reset 和所有尾部。
        unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
            .map_err(|_| TestError("scratch PCI preparation failed"))
    }
}

/// 单个 logical block 的独立原页在发表前完成初始化，保存真实 DMA 地址而不复制数据。
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
        // SAFETY: 原页尚未映射或发表，由此局部独占；初始化完整 Write 范围，无裸别名存续。
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
    // SAFETY: 唯一原页且 Write 已完整初始化；Task callback 只借收权页并等外部 gate，
    // 无回调等待自身 stop，无裸 CPU 别名；Read 结果不授权读未初始化尾部。
    let bio = unsafe { Bio::try_new(disk, operation, 128, memory, 0, callback) }.map_err(|_| {
        TestError("scratch Bio preparation failed (Write requires writable scratch)")
    })?;
    Ok((bio, original))
}

struct Owner {
    scenario: Scenario,
    baseline: mm::MemoryStats,
    controls: (u64, usize, u64, usize),
    disk: DiskRef,
    queue: WorkQueue,
    observation: Arc<Observation>,
    pending: Option<Bio>,
    completion: BioCompletion,
    original: MappedSegment,
    rejected: Option<Bio>,
    rejected_completion: BioCompletion,
    rejected_original: MappedSegment,
    prepared_stop: Option<PreparedTask>,
    stop_join: Option<JoinHandle>,
    activation_error: Option<BlockError>,
}

impl Owner {
    /// 同一 fixture 持唯一 Disk 和完成域；全部 Bio、观察存储及独立 stop 栈先于 start 预备。
    fn prepare(scenario: Scenario, context: &mut TestContext<'_>) -> Result<Self, TestError> {
        let baseline = settled()?;
        let device = scratch()?;
        if scenario == Scenario::Retirement && device.info().readonly {
            return Err(TestError(
                "uncertain_write requires writable scratch; no raw write issued",
            ));
        }
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
                    // 从未 start；撤原预留执行资源后关闭同域并真实 join，不丢已发表 Work owner。
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
        let activation_error = disk.start().err();
        if let Some(error) = activation_error {
            // 仍返回实际 owner 给 typed quiesce；不能 setup Err 后 Drop 可能已发表的 DMA。
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

    /// 只发表预建 Task，闭包独立拥有 DiskRef 和 Arc 存储，没有 fixture/context 借用。
    fn start_stop(&mut self) {
        self.stop_join = Some(
            self.prepared_stop
                .take()
                .expect("stop Task published twice")
                .publish(),
        );
    }

    /// 从原完成对象只领取原页一次，核地址、范围、方向和 CPU 收权；二次领取必须 Busy。
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

    /// 控制器在自然排空和拒新期间验证 stop 确实停在 accepted 重型 callback 的尾部。
    fn normal(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("scratch activation failed"));
        }
        // 先结束真实生成者再放行 used，保证最后义务来自原 slot 的重型 callback。
        self.disk.ktest_hold_used();
        self.disk
            .submit_bio(self.pending.take().expect("prepared normal Read"));
        self.disk.ktest_release_used();
        until("real Task callback did not Block", || {
            self.observation.callback_blocked()
        })?;
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

    /// Write 真发表一次后只控制 used retirement；实际设备可能已写，绝不重放未知 Write。
    fn retirement(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("scratch activation failed"));
        }
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
        self.disk.request_retire(BlockError::DeviceFault);
        self.start_stop();
        until("controlled Write retirement did not converge", || {
            self.completion.is_complete()
                && self
                    .stop_join
                    .as_ref()
                    .is_some_and(|join| join.try_join() == Some(0))
        })?;
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

    /// 只拒绝软件读取/承认 reset 确认；真实写零与生产两秒单调期限仍执行。
    fn withheld(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("scratch activation failed"));
        }
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
        context.expect_eq(
            *self.observation.stop_result.lock(),
            Some(Err(BlockError::StopUnconfirmed)),
        );
        self.retained(context);
        context.diagnostic(format_args!("stop-unconfirmed scope=controlled-confirmation-withheld hardware-refusal=not-claimed actual-reset-request=1 confirmation-read=withheld original-bounded-owner=retained case-abort-required=1"));
        Ok(())
    }

    /// stop 返回只在真实 reset 零、accepted 尾部及同步 IRQ/Work 结束后发生。
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

    /// 原 fixture 一直拥有控制区、请求页面、域及 IRQ/Work；此处绝不读取伪零状态。
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

    /// 主体失败也先释放合法回调并同步 stop/join；不能确认的原 owner 返回真实清理错误。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.observation.release_callback.complete();
        if self.scenario != Scenario::Withheld {
            self.disk.ktest_release_used();
        }
        // 未发表 stop Task 可以普通析构；其独立 DiskRef 不应拖到资源账本核验以后。
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
        match self.disk.stop() {
            Ok(()) | Err(BlockError::DeviceFault) => {}
            Err(BlockError::StopUnconfirmed) => {
                self.retained(context);
                return Err(TestError("controlled-confirmation-withheld: StopUnconfirmed; retaining original Disk, DMA and WorkQueue owner until case-abort shutdown"));
            }
            Err(_) => return Err(TestError("scratch stop cannot safely converge")),
        }
        if self.scenario == Scenario::Normal {
            // stop 已经 close_sync 原 slot Work，回调不会再查此定位槽。
            let locator = { CALLBACK_OWNER.lock().take() };
            drop(locator);
        }
        self.queue.close_sync();
        Ok(())
    }

    /// 安全静止并真实 join 后析构同一 owner，再用六域页数证明原资源确实归还。
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

struct NormalStop(Owner);
struct UncertainWrite(Owner);
struct StopUnconfirmed(Owner);

macro_rules! fixture {
    ($fixture:ident, $scenario:ident) => {
        impl Fixture<()> for $fixture {
            /// 每实例从真实 scratch 构造全新有界 owner，不依赖其它 case 的执行顺序。
            fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
                Owner::prepare(Scenario::$scenario, context).map(Self)
            }
            /// 先停该设备及回调并 join 自己的 stop Task，失败由 typed API case-abort 保活终结。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// 普通 final 之前先析构安全 owner 并核完整原资源账本。
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
    /// accepted 的真实重型 callback 尚未结束时 normal stop 必须 Draining、拒新并真 Blocked。
    #[test]
    fn normal_stop_accepted(context: &mut TestContext<'_>, fixture: &mut NormalStop) -> TestResult {
        fixture.0.normal(context)
    }
}

#[ktest::suite(fixture = UncertainWrite, context = "Thread", min_cpus = 2, tags("block", "stop", "B11", "T5"), resources("scratch"), timeout_ms = 15000)]
mod retirement {
    use super::*;
    /// 一次真实 Write 的受控完成退休只在真 reset 后报错收权，绝不推断设备没有写入。
    #[test]
    fn uncertain_write(context: &mut TestContext<'_>, fixture: &mut UncertainWrite) -> TestResult {
        fixture.0.retirement(context)
    }
}

#[ktest::suite(fixture = StopUnconfirmed, context = "Thread", isolation = "Case", min_cpus = 2, tags("block", "stop", "B11", "T5", "negative"), resources("scratch"), timeout_ms = 15000)]
mod confirmation {
    use super::*;
    /// 软件拒读 reset 确认后 fixture 保原有界 DMA，必须 case-abort Error 而不是普通 Pass。
    #[test]
    #[ignore = "controlled-confirmation-withheld intentionally cannot quiesce: case-abort Error terminates the isolated VM while retaining original DMA owners"]
    fn stop_unconfirmed(
        context: &mut TestContext<'_>,
        fixture: &mut StopUnconfirmed,
    ) -> TestResult {
        fixture.0.withheld(context)
    }
}
