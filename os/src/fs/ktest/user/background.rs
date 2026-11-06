//! 后台回写子系统（`WritebackDomain`）的集成测试公共夹具与轮次同步控制器。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统用户态与后台行为测试层（`fs::ktest::user::background`），为下属三个测试子模块提供共享基础设施：
//! - [`metadata`]：验证卷空闲状态下的元数据定时回写，以及脏页超限时的普通写入者阻塞节流（Dirty Throttling）与写回带宽估算。
//! - [`progress`]：验证单轮回写预算上限（数据页 48 页、元数据页 16 页）、扫描游标公平推进、写回期间并发修改产生的后继脏页（Redirty）保留，以及 `sync_range` 显式范围同步的快照边界。
//! - [`retirement`]：验证文件系统卸载、Inode 缓存（`InodeCache`）退役、预留缓冲池（`BioReserve`）与块请求（`Bio`）弱引用（`Weak`）释放的生命周期顺序。
//!
//! # 核心机制：回写轮次拦截与门控状态机（`WaveControl`）
//! 为了在测试中稳定观测每一轮后台回写结束时的瞬态统计快照，测试夹具通过 `WritebackDomain::ktest_set_round_hook` 注册轮次完成回调，并与主测试线程通过 [`WaveControl`] 协同：
//!
//! ```text
//!   [后台回写 Work 线程]                        [主测试线程 / 并发写入线程]
//!           |                                                |
//!   执行一轮脏页扫描与提交                                    |
//!           |                                                |
//!   completed_round(snapshot)                                |
//!     1. 检查 cancelled 标志                                 |
//!     2. 将本轮快照写入 last (SpinLock)                      |
//!     3. rounds 计数 +1 (Release)  ----------------------> round(number) 轮询等待:
//!     4. gate.wait() 阻塞挂起                                rounds >= number 且 gate.test_waiting() == 1
//!           :                                                |
//!           : (停留在轮尾门控，供测试线程检查状态)            读取 last 快照并断言各项预算指标
//!           :                                                |
//!           : <------------------------------------------- gate.complete() 放行下一轮
//!     5. 唤醒并退出本轮回调                                  或 cancel() 置位 cancelled 并放行
//! ```
//!
//! # 锁获取顺序与资源清理契约（Quiesce Order）
//! 1. **自旋锁（`WaveControl::last`）与完成门（`WaveControl::gate`）**：回调线程必须先获取 `last` 自旋锁写入快照并释放锁，随后更新 `rounds` 原子计数，最后才能在 `gate.wait()` 上睡眠；严禁持有 `last` 自旋锁跨睡眠点。
//! 2. **夹具静默回收顺序（`BackgroundOwner::quiesce`）**：
//!    - 先置位 `producer_stop` 通知并发写入线程准备退出；
//!    - 恢复块设备测试挂起的完成回调（`test_resume_completions`），取消轮尾门控并卸载回写钩子（`release_rounds`），唤醒预留池等待队列（`notify_available`）；
//!    - 等待所有被测试持有的 `BioCompletion` 完成并归还预留 DMA 内存，释放预留通道 `BioLane`；
//!    - 回收所有并发测试消费者线程（`join_consumers`），最后等待测试自建的 `InodeCache` 彻底退役（`wait_retired`）并委托底层 `WriteOwner` 完成卷与内存基线核对。

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

/// 准备测试文件时预填充的初始字节值（ASCII `'1'`）。
const INITIAL_BYTE: u8 = 0x31;

/// 后台回写轮次同步与并发写入状态控制器。
///
/// 用于在后台回写 Work 线程与测试主线程之间同步轮次完成边界，并记录并发写入生产者的进度与停止信号。
struct WaveControl {
    /// 标记轮尾门控是否已取消；置位后回写完成回调不再阻塞等待。
    cancelled: AtomicBool,
    /// 通知并发写入生产者线程退出的停止标志。
    producer_stop: AtomicBool,
    /// 并发写入生产者已完成的写入轮次或页计数。
    produced: AtomicUsize,
    /// 并发写入生产者累计成功写入的字节数。
    accepted_bytes: AtomicUsize,
    /// 并发写入生产者最近一次写入返回时观测到的回写域完成序列号（`completion_seq`）。
    returned_sequence: AtomicUsize,
    /// 已到达轮尾门控的合格后台回写轮次数。
    rounds: AtomicUsize,
    /// 最近一次完成回写轮次的统计快照，由自旋锁保护。
    last: SpinLock<Option<WritebackSnapshot>>,
    /// 用于在每轮后台回写结束时暂停回写线程的完成量门控。
    gate: Completion,
}

impl WaveControl {
    /// 分配并初始化回写轮次控制器。
    ///
    /// 在测试正文阶段按需分配，若堆内存不足则返回 `TestError`。
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

    /// 记录一轮后台回写的完成快照并在门控处挂起当前回写线程。
    ///
    /// 由 `WritebackDomain` 的轮尾钩子调用；调用时不持有文件锁且不修改任何缓存页状态。
    /// 若控制器已取消（`cancelled` 为 `true`），则直接返回而不阻塞。
    fn completed_round(&self, snapshot: WritebackSnapshot) {
        if self.cancelled.load(Ordering::Acquire) {
            return;
        }
        *self.last.lock() = Some(snapshot);
        self.rounds.fetch_add(1, Ordering::Release);
        self.gate.wait();
    }

    /// 等待指定的第 `number` 轮后台回写完成并停留在门控处，返回该轮的统计快照。
    ///
    /// 同时检查 `rounds` 计数与 `gate.test_waiting() == 1`，确保回写回调已真正进入等待队列后才读取快照。
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

    /// 取消轮尾门控拦截并唤醒正在等待的回写回调线程。
    ///
    /// 先以 `Release` 内存序将 `cancelled` 置为 `true` 阻止后续轮次进入等待，再调用 `gate.complete()` 放行已阻塞的回调。
    fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
        self.gate.complete();
    }
}

/// 后台回写测试套件的资源与生命周期夹具。
///
/// 在复用 [`WriteOwner`] 的块设备、挂载卷、并发消费者线程槽及内存泄漏基线检查的基础上，
/// 额外管理本测试注册的回写轮次钩子、预留缓冲池借用以及独立 Inode 缓存的退役顺序。
struct BackgroundOwner {
    /// 底层写入测试夹具，负责维护块设备、Ext4 挂载卷、并发任务槽及资源基线统计。
    write: WriteOwner,
    /// 当前测试激活的回写轮次门控控制器。
    control: Option<Arc<WaveControl>>,
    /// 已安装测试轮尾钩子的回写域引用，用于在测试结束时卸载钩子。
    domain: Option<Arc<WritebackDomain>>,
    /// 测试期间新建的独立 `InodeCache` 退役观察句柄，确保夹具清理前缓存已彻底退役。
    retired: Option<InodeCacheWatch>,
    /// 测试持有的块层预留缓冲池（`BioReserve`）引用，用于在清理阶段唤醒等待中的借用者。
    source: Option<Arc<BioReserve>>,
    /// 测试显式占用的回写预留通道（`BioLane`），用于在清理阶段按序归还配额。
    lane: Option<BioLane>,
    /// 测试持有的最多 8 个块请求完成句柄（`BioCompletion`），用于在清理阶段等待 DMA 完成并回收预留内存。
    observers: [Option<BioCompletion>; 8],
}

impl BackgroundOwner {
    /// 准备指定页数的测试文件并同步落盘，使回写域在测试开始前处于干净静止状态。
    ///
    /// 打开 `/test/write-reserve` 并先执行 `truncate(0)` 清空前序测试遗留的数据，
    /// 随后逐页写入 `INITIAL_BYTE` 分配物理数据块与映射块，最后通过 `sync_range` 和 [`drained`] 确保所有脏页落盘。
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

        // Step 1: 截断旧文件内容并校验卷块大小与剩余空闲块数是否满足本次测试需求。
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

        // Step 2: 逐页写入初始字节，触发底层盘块分配。
        let input = [INITIAL_BYTE; PAGE_SIZE];
        for index in 0..pages {
            if checked(context, file.write_at((index * PAGE_SIZE) as u64, &input))? != PAGE_SIZE {
                return Err(TestError(
                    "background file preparation accepted a short page",
                ));
            }
        }

        // Step 3: 同步全文件范围并等待后台回写域与块设备完成队列彻底排空。
        checked(context, file.sync_range(0..u64::MAX, false))?;
        drained(&volume)?;
        context.expect_eq(
            checked(context, file.stat())?.size,
            (pages * PAGE_SIZE) as u64,
        );
        Ok(file)
    }

    /// 在当前卷的回写域上注册轮次完成钩子，并返回新建的 [`WaveControl`] 控制器。
    ///
    /// 参数 `metadata` 指定拦截的目标页类别：`true` 表示仅当本轮选中了脏元数据页（`round_selected_metadata > 0`）时才触发门控，
    /// `false` 表示仅当选中了脏数据页（`round_selected_data > 0`）时才触发门控，从而过滤掉准备阶段迟到的空轮次。
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

    /// 取消轮尾门控并卸载回写域上的测试钩子。
    ///
    /// 仅解除回写轮次拦截；`WaveControl` 实例本身仍保留在夹具中，供并发写入线程读取停止标志并在 `WriteOwner` 中汇合退出。
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
    /// 初始化后台回写测试夹具，复用 [`WriteOwner`] 的套件级卷与资源基线。
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

    /// 按严格依赖顺序停止并发写入、放行阻塞门控、回收预留 DMA 内存并等待缓存退役。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 通知并发生产者停止写入，恢复块设备完成事件并解除回写轮次门控。
        if let Some(control) = &self.control {
            control.producer_stop.store(true, Ordering::Release);
        }

        for disk in &self.write.user.path.owner.disks {
            disk.test_resume_completions();
        }
        self.release_rounds();
        // Step 2: 唤醒预留缓冲池等待者，等待所有已提交的测试 Bio 完成并归还预留内存与通道。
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

        // Step 3: 汇合所有并发消费者线程，并等待测试自建的 InodeCache 彻底退役。
        self.write.join_consumers()?;
        if let Some(watch) = self.retired.as_ref() {
            checked(context, watch.wait_retired())?;
        }
        drop(self.retired.take());

        self.control.take();
        drop(self.source.take());
        <WriteOwner as Fixture<UserSuite>>::quiesce(&mut self.write, context)
    }

    /// 校验所有测试专属控制句柄与观察槽均已清空，随后委托 [`WriteOwner`] 执行内存与资源基线核对。
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

/// 等待指定 Ext4 卷的后台回写域脱离 `pending`/`running` 状态，并冲刷块设备完成队列。
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

/// 校验已完成的一轮后台回写快照是否满足单轮扫描上限与页数预算（数据页 ≤ 48，元数据页 ≤ 16，总页数 ≤ 64）。
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

/// 显式引用并保活三个子测试套件的注册槽位，向父级测试框架导出统一入口锚点。
pub(crate) fn anchor() -> &'static ::ktest::SuiteSlot {
    let suites = [
        retirement::cases::__ktest_anchor(),
        progress::cases::__ktest_anchor(),
        metadata::cases::__ktest_anchor(),
    ];
    core::hint::black_box(suites);
    suites[0]
}
