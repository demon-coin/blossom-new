//! 后台回写的共同夹具保留原取消、完成与退休顺序，行为由三个正式叶套件拥有。

use alloc::sync::Arc;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::block::{BioCompletion, BioLane, BioReserve};
use crate::fs::background::{global_snapshot, WritebackDomain, WritebackSnapshot};
use crate::fs::ktest::user::write::{allocation_counts, WriteOwner};
use crate::fs::ktest::user::UserSuite;
use crate::fs::ktest::*;
use crate::fs::FileRef;
use crate::mm::PAGE_SIZE;

mod metadata;
mod progress;
mod retirement;

const INITIAL_BYTE: u8 = 0x31;

/// 门只暂停已结束的原回写轮；生产者的停止位与原 Task 句柄分别拥有取消和离栈责任。
struct WaveControl {
    cancelled: AtomicBool,
    producer_stop: AtomicBool,
    produced: AtomicUsize,
    accepted_bytes: AtomicUsize,
    returned_sequence: AtomicUsize,
    rounds: AtomicUsize,
    last: SpinLock<Option<WritebackSnapshot>>,
    gate: Completion,
}

impl WaveControl {
    /// 正文准备可失败的观察存储，设置期的原资源基线不包含本例控制对象。
    fn try_new() -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            cancelled: AtomicBool::new(false),
            producer_stop: AtomicBool::new(false),
            produced: AtomicUsize::new(0),
            accepted_bytes: AtomicUsize::new(0),
            returned_sequence: AtomicUsize::new(0),
            rounds: AtomicUsize::new(0),
            last: SpinLock::new(None),
            gate: Completion::new(),
        })
        .map_err(|_| TestError("writeback wave observation OOM"))
    }

    /// 快照来自原回调已经完成的数据和元数据；不取得文件锁或更改任何页状态。
    fn completed_round(&self, snapshot: WritebackSnapshot) {
        if self.cancelled.load(Ordering::Acquire) {
            return;
        }
        *self.last.lock() = Some(snapshot);
        self.rounds.fetch_add(1, Ordering::Release);
        self.gate.wait();
    }

    /// 同时观察原轮序号和真实完成等待成员，避免把轮尾短暂状态当作停点。
    fn round(&self, number: usize) -> Result<WritebackSnapshot, TestError> {
        until(
            "background round did not reach its completed boundary",
            || number <= self.rounds.load(Ordering::Acquire) && self.gate.test_waiting() == 1,
        )?;
        if self.rounds.load(Ordering::Acquire) != number {
            return Err(TestError("background round escaped its owned gate"));
        }
        self.last
            .lock()
            .as_ref()
            .copied()
            .ok_or(TestError("completed background round has no snapshot"))
    }

    /// 先禁止后续等待，再唤醒已经登记或即将登记的本轮回调。
    fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
        self.gate.complete();
    }
}

/// 只增加本例门控的清理顺序，卷、设备、消费者句柄与两层资源账仍归原 WriteOwner。
struct BackgroundOwner {
    write: WriteOwner,
    control: Option<Arc<WaveControl>>,
    domain: Option<Arc<WritebackDomain>>,
    retired: Option<InodeCacheWatch>,
    source: Option<Arc<BioReserve>>,
    lane: Option<BioLane>,
    observers: [Option<BioCompletion>; 8],
}

impl BackgroundOwner {
    /// 正常写入分配真实盘块并同步；同 VM 前例内容通过普通 truncate 退出，不假设空文件。
    fn prepare_file(
        &mut self,
        context: &mut TestContext<'_>,
        pages: usize,
    ) -> Result<FileRef, TestError> {
        self.write.prepare_namespace(context)?;
        let file = self.write.open(context, b"/test/write-reserve", 2)?;
        let volume = self.write.volume();
        if volume.geometry().block_size != PAGE_SIZE {
            return Err(TestError(
                "background wave requires the ordinary 4KiB root image",
            ));
        }

        checked(context, file.truncate(0))?;
        let (free_blocks, free_inodes) = allocation_counts(context, &volume)?;
        context.diagnostic(format_args!(
            "background-input block-size={} free-blocks={} free-inodes={} requested-file-pages={} dirty-limit={}",
            volume.geometry().block_size,
            free_blocks,
            free_inodes,
            pages,
            global_snapshot().dirty_limit,
        ));
        if free_blocks < (pages + 64) as u64 {
            return Err(TestError(
                "background wave requires actual free data and mapping blocks",
            ));
        }

        let input = [INITIAL_BYTE; PAGE_SIZE];
        for index in 0..pages {
            if checked(context, file.write_at((index * PAGE_SIZE) as u64, &input))? != PAGE_SIZE {
                return Err(TestError(
                    "background file preparation accepted a short page",
                ));
            }
        }

        checked(context, file.sync_range(0..u64::MAX, false))?;
        drained(&volume)?;
        context.expect_eq(
            checked(context, file.stat())?.size,
            (pages * PAGE_SIZE) as u64,
        );
        Ok(file)
    }

    /// 门只观察武装后选中指定页类别的真实轮，准备期迟到的空轮不占本例门票。
    fn observe_rounds(&mut self, metadata: bool) -> Result<Arc<WaveControl>, TestError> {
        assert!(self.control.is_none() && self.domain.is_none());
        let control = WaveControl::try_new()?;
        let observed = control.clone();
        let domain = self.write.volume().writeback_domain().clone();
        let previous_round = domain.snapshot().rounds;
        let hook: Arc<dyn Fn(WritebackSnapshot) + Send + Sync> =
            Arc::try_new(move |snapshot: WritebackSnapshot| {
                let selected = if metadata {
                    snapshot.round_selected_metadata
                } else {
                    snapshot.round_selected_data
                };
                if previous_round < snapshot.rounds && 0 < selected {
                    observed.completed_round(snapshot);
                }
            })
            .map_err(|_| TestError("writeback round hook OOM"))?;
        domain.ktest_set_round_hook(Some(hook));
        self.domain = Some(domain);
        self.control = Some(control.clone());
        Ok(control)
    }

    /// 撤钩只结束轮尾观察；停止位仍保在夹具，已接受的生产者必须由原 WriteOwner join。
    fn release_rounds(&mut self) {
        if let Some(control) = &self.control {
            control.cancel();
        }
        if let Some(domain) = self.domain.take() {
            domain.ktest_set_round_hook(None);
        }
    }
}

impl Fixture<UserSuite> for BackgroundOwner {
    /// 库存与网络只由suite准备一次，每例继续使用原WriteOwner的真实基线。
    fn set_up(context: &mut TestContext<'_>, suite: &UserSuite) -> Result<Self, TestError> {
        Ok(Self {
            write: <WriteOwner as Fixture<UserSuite>>::set_up(context, suite)?,
            control: None,
            domain: None,
            retired: None,
            source: None,
            lane: None,
            observers: [const { None }; 8],
        })
    }

    /// 先放行完成、轮尾门和 lane，再 join 原消费者并等 canonical 退休；停止只由无槽卷 owner 执行。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if let Some(control) = &self.control {
            control.producer_stop.store(true, Ordering::Release);
        }

        for disk in &self.write.user.path.owner.disks {
            disk.test_resume_completions();
        }
        self.release_rounds();
        if let Some(source) = &self.source {
            source.notify_available();
        }
        for observer in &mut self.observers {
            if let Some(completion) = observer.as_ref() {
                until("reserved observer retains accepted DMA", || {
                    completion.is_complete()
                })?;
                drop(completion.take_memory());
            }
            drop(observer.take());
        }
        drop(self.lane.take());

        self.write.join_consumers()?;
        if let Some(watch) = self.retired.as_ref() {
            checked(context, watch.wait_retired())?;
        }
        drop(self.retired.take());

        self.control.take();
        drop(self.source.take());
        <WriteOwner as Fixture<UserSuite>>::quiesce(&mut self.write, context)
    }

    /// 控制存储已经释放，原 MM 六字段和用户三项元数据逐项比较各自同 VM 基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert!(
            self.control.is_none()
                && self.domain.is_none()
                && self.source.is_none()
                && self.lane.is_none()
        );
        assert!(self.observers.iter().all(Option::is_none));
        assert!(self.retired.is_none());
        <WriteOwner as Fixture<UserSuite>>::tear_down(self.write, context)
    }
}

/// 原域不再声明 pending/running 后等待设备工作票；完整回调尾部仍由卷退休同步结束。
fn drained(volume: &Ext4Volume) -> TestResult {
    until(
        "background work retained a pending or running obligation",
        || {
            let state = volume.writeback_domain().snapshot();
            !state.pending && !state.running
        },
    )?;
    volume.claim().disk().test_flush_completions();
    Ok(())
}

/// 本轮入口槽数和两个页预算各自核对，真实接受和完成字节不能由选页数代替。
fn finite_round(context: &mut TestContext<'_>, snapshot: WritebackSnapshot) {
    context.expect_eq(0 < snapshot.round_limit, true);
    context.expect_eq(0 < snapshot.round_scanned, true);
    context.expect_eq(snapshot.round_scanned <= snapshot.round_limit, true);
    context.expect_eq(snapshot.round_selected_data <= 48, true);
    context.expect_eq(snapshot.round_selected_metadata <= 16, true);
    context.expect_eq(
        snapshot.round_selected_data + snapshot.round_selected_metadata <= 64,
        true,
    );

    context.expect_eq(0 < snapshot.round_accepted_bytes, true);
    context.expect_eq(
        snapshot.round_accepted_bytes <= (64 * PAGE_SIZE) as u64,
        true,
    );
    context.expect_eq(
        snapshot.round_completed_bytes <= snapshot.round_accepted_bytes,
        true,
    );
    context.expect_eq(snapshot.running, false);
}

/// 显式保活三个行为叶的真实注册槽，原文件系统消费者仍只取得一个正式锚点。
pub(crate) fn anchor() -> &'static ::ktest::SuiteSlot {
    let suites = [
        retirement::cases::__ktest_anchor(),
        progress::cases::__ktest_anchor(),
        metadata::cases::__ktest_anchor(),
    ];
    core::hint::black_box(suites);
    suites[0]
}
