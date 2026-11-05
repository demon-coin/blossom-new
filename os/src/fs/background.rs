//! 卷级有限回写使用共享 worker；原页状态、真实完成带宽和 writer 等待共用同一责任域。

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::ops::{Deref, DerefMut};
use core::pin::{pin, Pin};
use core::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use spin::Once;

use crate::block::{BioReserve, BlockError, MetadataIoContext};
use crate::fs::{writeback, Ext4Volume, FsError, InodeRef};
use crate::mm::{self, PAGE_SIZE};
use crate::sched;
use crate::sync::{SleepMutex, SpinLock};
use crate::task::ReclaimScope;
use crate::time::{self, Duration, MonotonicInstant, NSEC_PER_SEC};
use crate::timer::{Timer, TimerKind};
use crate::wait::{WaitNode, WaitQueue, Waiter, Waker};
use crate::workqueue::{DelayedWork, WorkQueue, WorkQueueConfig};

const DIRTY_RATIO: usize = 10;
const DATA_BUDGET: usize = 48;
const METADATA_BUDGET: usize = 16;
const DIRTY_EXPIRE: Duration = Duration::from_millis(200);
const MAX_PAUSE_NS: u64 = 200_000_000;
const BANDWIDTH_INTERVAL_NS: u64 = 200_000_000;
const WB_MAX_ACTIVE: usize = 8;
const MIN_DIRTY_PAGES: usize = 32;
const MAX_DIRTY_PAGES: usize = 4096;
const PERCENT: usize = 100;

static WB_QUEUE: Once<WorkQueue> = Once::new();
static FILE_PAGES: AtomicUsize = AtomicUsize::new(0);
static DIRTY_PAGES: AtomicUsize = AtomicUsize::new(0);
static WRITEBACK_PAGES: AtomicUsize = AtomicUsize::new(0);
static ACTIVE_BANDWIDTH: AtomicU64 = AtomicU64::new(0);
static GLOBAL_COMPLETION: SpinLock<Completion> = SpinLock::new(Completion::new());

/// 带宽只由真实完成 byte 差与同一单调时间差产生；零时间不建立样本。
struct Completion {
    accepted_bytes: u64,
    completed_bytes: u64,
    sequence: u64,
    error_sequence: u64,
    sample_time: Option<MonotonicInstant>,
    sample_bytes: u64,
    bandwidth: u64,
    #[cfg(feature = "ktest")]
    last_sample_bytes: u64,
    #[cfg(feature = "ktest")]
    last_sample_elapsed_ns: u64,
    #[cfg(feature = "ktest")]
    last_sample_end_ns: u64,
}

impl Completion {
    /// 未提交任何原页时保持空样本，不能用默认吞吐代替设备完成。
    const fn new() -> Self {
        Self {
            accepted_bytes: 0,
            completed_bytes: 0,
            sequence: 0,
            error_sequence: 0,
            sample_time: None,
            sample_bytes: 0,
            bandwidth: 0,
            #[cfg(feature = "ktest")]
            last_sample_bytes: 0,
            #[cfg(feature = "ktest")]
            last_sample_elapsed_ns: 0,
            #[cfg(feature = "ktest")]
            last_sample_end_ns: 0,
        }
    }

    /// 原范围开始 WB 时记录首个测量起点，不重置仍在累计的实际完成窗口。
    fn start(&mut self, now: MonotonicInstant) {
        if self.sample_time.is_none() {
            self.sample_time = Some(now);
            self.sample_bytes = self.completed_bytes;
        }
    }

    /// 已接受范围完成后累计真实前缀，初样本即时建立，后续只在有限测量窗更新。
    fn complete(&mut self, accepted: usize, completed: usize, now: MonotonicInstant) {
        assert!(0 < accepted && completed <= accepted);
        self.accepted_bytes += accepted as u64;
        self.completed_bytes += completed as u64;
        self.sequence += 1;
        let Some(start) = self.sample_time else {
            return;
        };
        let elapsed = now.as_nanos() - start.as_nanos();
        if elapsed == 0 || (self.bandwidth != 0 && elapsed < BANDWIDTH_INTERVAL_NS) {
            return;
        }
        let bytes = self.completed_bytes - self.sample_bytes;
        if bytes != 0 {
            self.bandwidth =
                ((u128::from(bytes) * u128::from(NSEC_PER_SEC)) / u128::from(elapsed)) as u64;
            #[cfg(feature = "ktest")]
            {
                self.last_sample_bytes = bytes;
                self.last_sample_elapsed_ns = elapsed;
                self.last_sample_end_ns = now.as_nanos();
            }
        }
        if BANDWIDTH_INTERVAL_NS <= elapsed {
            self.sample_time = Some(now);
            self.sample_bytes = self.completed_bytes;
        }
    }
}

/// 本轮只有选中的原节点和已完成 IO 的标量事实，不保存第二个 inode 或页名单。
#[derive(Clone, Copy, Default)]
struct Round {
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "ktest 的有限回写轮观察核对扫描上限与实际扫描数")
    )]
    limit: usize,
    scanned: usize,
    selected_data: usize,
    selected_metadata: usize,
    accepted_bytes: u64,
    completed_bytes: u64,
    later_dirty: bool,
}

struct State {
    active: bool,
    stopping: bool,
    halted: bool,
    /// 唯一尚未进入业务回调的真实 Arc<Ext4Volume>::into_raw 票。
    pending: usize,
    running: bool,
    notified: bool,
    next_slot: usize,
    completion: Completion,
    contribution: u64,
    rounds: u64,
    finished_rounds: u64,
    stop_sequence: u64,
    round: Round,
    writer_waits: u64,
    writer_wakes: u64,
    waiting_writers: usize,
}

/// 有限 WB 轮完成后的同一只读观察合同。
#[cfg(feature = "ktest")]
type RoundHook = Arc<dyn Fn(WritebackSnapshot) + Send + Sync>;

/// 卷强持服务，服务只为已接受的一次 pending/running 保卷票，不永久反持卷。
pub(crate) struct WritebackDomain {
    work: Once<Pin<Arc<DelayedWork>>>,
    state: SpinLock<State>,
    stop: SleepMutex<()>,
    source: Option<Arc<BioReserve>>,
    metadata_sequence: AtomicU64,
    scratch: SpinLock<Option<Box<writeback::WbStorage>>>,
    dirty_pages: AtomicUsize,
    writeback_pages: AtomicUsize,
    submitted_bytes: AtomicU64,
    waiters: WaitQueue,
    #[cfg(feature = "ktest")]
    round_hook: SpinLock<Option<RoundHook>>,
}

/// 值边界只记录真实域事实，不保活后继轮或把开始计数当完成。
#[derive(Clone, Copy, Debug)]
pub(crate) struct WritebackBoundary {
    identity: usize,
    completion_sequence: u64,
    error_sequence: u64,
    finished_rounds: u64,
    stop_sequence: u64,
    submitted_bytes: u64,
    completed_bytes: u64,
    inactive: bool,
    halted: bool,
}

/// 本边界之后的真实增量；停止或零提交轮结束并不冒称释放了文件页。
#[derive(Clone, Copy, Debug)]
pub(crate) struct WritebackProgress {
    pub submitted_bytes: u64,
    pub completed_bytes: u64,
    pub stopped: bool,
    pub error: bool,
}

/// 借出时把唯一预建 Box 移出域，不让 idle scratch 反持活请求或 inode。
pub(crate) struct WbStorageLease {
    domain: Arc<WritebackDomain>,
    storage: Option<Box<writeback::WbStorage>>,
    _scope: ReclaimScope,
}

impl Deref for WbStorageLease {
    type Target = writeback::WbStorage;

    /// 正式借用只暴露仍由本 guard 唯一持有的窗口。
    fn deref(&self) -> &Self::Target {
        self.storage
            .as_deref()
            .expect("writeback storage lease ended")
    }
}

impl DerefMut for WbStorageLease {
    /// 原窗口没有第二份拥有，可在同一 Task 的 guard 下逐窗复用。
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.storage
            .as_deref_mut()
            .expect("writeback storage lease ended")
    }
}

impl Drop for WbStorageLease {
    /// 临时凭证及 book 在域锁外清空，再归还空 Box 并通知所有真实等待者。
    fn drop(&mut self) {
        let storage = self
            .storage
            .take()
            .expect("writeback storage returned twice");
        self.domain.return_storage(storage);
    }
}

/// 同卷真实状态快照；本轮实际完成者发布 accepted 账，提交瞬间与在途数仍由原 Disk/Bio 观察。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct WritebackSnapshot {
    pub dirty_pages: usize,
    pub writeback_pages: usize,
    pub submitted_bytes: u64,
    pub accepted_bytes: u64,
    pub completed_bytes: u64,
    pub completion_seq: u64,
    pub error_seq: u64,
    pub bandwidth_bytes_per_sec: u64,
    #[cfg(feature = "ktest")]
    pub sample_bytes: u64,
    #[cfg(feature = "ktest")]
    pub sample_elapsed_ns: u64,
    #[cfg(feature = "ktest")]
    pub sample_end_ns: u64,
    pub rounds: u64,
    pub finished_rounds: u64,
    pub stop_sequence: u64,
    pub round_limit: usize,
    pub round_scanned: usize,
    pub round_selected_data: usize,
    pub round_selected_metadata: usize,
    pub round_accepted_bytes: u64,
    pub round_completed_bytes: u64,
    pub later_dirty: bool,
    pub writer_waits: u64,
    pub writer_wakes: u64,
    pub waiting_writers: usize,
    pub active: bool,
    pub pending: bool,
    pub running: bool,
}

/// 全局页账与当前文件页近似阈值；metadata 参与脏账但不计文件可回收页。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct DirtySnapshot {
    #[expect(dead_code, reason = "全局回写压力诊断通过 Debug 输出存活文件页账")]
    pub file_pages: usize,
    #[expect(dead_code, reason = "全局回写压力诊断通过 Debug 输出当前脏页账")]
    pub dirty_pages: usize,
    #[expect(dead_code, reason = "全局回写压力诊断通过 Debug 输出当前回写页账")]
    pub writeback_pages: usize,
    #[expect(
        dead_code,
        reason = "全局回写压力诊断通过 Debug 输出计算脏页阈值的 dirtyable 页数"
    )]
    pub dirtyable_pages: usize,
    pub dirty_limit: usize,
    pub background_limit: usize,
    pub completed_bytes: u64,
    pub bandwidth_bytes_per_sec: u64,
}

/// BSP 在共享 pool 准备后、运行许可发布前预建独立准入的 reclaim WB 队列。
pub(crate) fn initialize() -> Result<(), FsError> {
    assert!(
        WB_QUEUE.get().is_none(),
        "writeback service initialized twice"
    );
    let queue = WorkQueue::try_new(WorkQueueConfig {
        max_active: WB_MAX_ACTIVE,
        ordered: false,
        reclaim: true,
    })
    .map_err(|_| FsError::NoMemory)?;
    WB_QUEUE.call_once(|| queue);
    Ok(())
}

impl WritebackDomain {
    /// 卷发表前准备全部控制对象；context 只定位本域，接受异步工作时另交真实卷票。
    pub(crate) fn try_new(volume: &Arc<Ext4Volume>) -> Result<Arc<Self>, FsError> {
        let source = if volume.mode() == crate::block::AccessMode::ReadWrite {
            volume.metadata_cache().reserve()
        } else {
            None
        };
        let scratch = match &source {
            Some(source) => Some(
                Box::try_new(writeback::WbStorage::try_reserved(source.clone())?)
                    .map_err(|_| FsError::NoMemory)?,
            ),
            None => None,
        };
        let domain = Arc::try_new(Self {
            work: Once::new(),
            state: SpinLock::new(State {
                active: false,
                stopping: false,
                halted: false,
                pending: 0,
                running: false,
                notified: false,
                next_slot: 0,
                completion: Completion::new(),
                contribution: 0,
                rounds: 0,
                finished_rounds: 0,
                stop_sequence: 0,
                round: Round::default(),
                writer_waits: 0,
                writer_wakes: 0,
                waiting_writers: 0,
            }),
            stop: SleepMutex::new(()),
            source,
            metadata_sequence: AtomicU64::new(0),
            scratch: SpinLock::new(scratch),
            dirty_pages: AtomicUsize::new(0),
            writeback_pages: AtomicUsize::new(0),
            submitted_bytes: AtomicU64::new(0),
            waiters: WaitQueue::new(),
            #[cfg(feature = "ktest")]
            round_hook: SpinLock::new(None),
        })
        .map_err(|_| FsError::NoMemory)?;
        let work = DelayedWork::try_new(
            WB_QUEUE.get().expect("writeback service is not prepared"),
            0,
            run_writeback,
            Arc::as_ptr(&domain) as usize,
        )
        .map_err(|_| FsError::NoMemory)?;
        domain.work.call_once(|| work);
        Ok(domain)
    }

    /// 捕获本卷停止边界，再在所有 FS 业务锁外等待 scratch 和共用来源中的完整 lane。
    pub(crate) fn borrow_storage(self: &Arc<Self>) -> Result<WbStorageLease, FsError> {
        sched::assert_switchable();
        let source = self.source.as_ref().ok_or(FsError::ReadOnly)?;
        let (boundary, metadata_boundary) = {
            let state = self.state.lock();
            if !state.active || state.stopping || state.halted {
                return Err(FsError::Io(BlockError::QueueStopped));
            }
            (
                state.stop_sequence,
                self.metadata_sequence.load(Ordering::Acquire),
            )
        };
        loop {
            if self.storage_stopped(boundary) {
                return Err(FsError::Io(BlockError::QueueStopped));
            }
            let storage = self.scratch.lock().take();
            if let Some(mut storage) = storage {
                let scope = ReclaimScope::noio();
                match source.acquire_wb_lane(|| self.storage_stopped(boundary)) {
                    Ok(lane) => {
                        storage.bind_lane(lane, boundary, metadata_boundary);
                        return Ok(WbStorageLease {
                            domain: self.clone(),
                            storage: Some(storage),
                            _scope: scope,
                        });
                    }
                    Err(error) => {
                        self.return_storage(storage);
                        return Err(FsError::from(error));
                    }
                }
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 的真实 Arc 保域，pin 栈节点，所有出口先结束登记。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if self.storage_stopped(boundary) {
                return Err(FsError::Io(BlockError::QueueStopped));
            }
            if self.scratch.lock().is_some() {
                continue;
            }
            waiter.park();
        }
    }

    /// 原借用只能核原单调边界，stop 后新 activate 不能复活它。
    pub(crate) fn storage_stopped(&self, boundary: u64) -> bool {
        let state = self.state.lock();
        state.stop_sequence != boundary || !state.active || state.stopping || state.halted
    }

    /// ctx 借本卷的停止序列与物理来源，另一卷的停止不推进此序列。
    pub(crate) fn metadata_io_context(&self, boundary: u64) -> MetadataIoContext<'_> {
        MetadataIoContext::for_view(
            self.source
                .as_ref()
                .expect("read-only domain has no reserved metadata source"),
            &self.metadata_sequence,
            boundary,
        )
    }

    /// 清空和来源通知都在短锁外；锁内只移回唯一空窗口。
    fn return_storage(&self, mut storage: Box<writeback::WbStorage>) {
        storage.clear();
        {
            let mut scratch = self.scratch.lock();
            assert!(scratch.is_none(), "writeback scratch returned twice");
            *scratch = Some(storage);
        }
        self.waiters.wake_all();
    }

    /// 停止等待实际借出 Box 归还，不以 running=false 或来源控制可用代替业务尾。
    fn wait_storage_returned(&self) {
        if self.source.is_none() {
            return;
        }
        loop {
            if self.scratch.lock().is_some() {
                return;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: stop caller 保域，固定栈节点在返回前撤销登记。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if self.scratch.lock().is_some() {
                return;
            }
            waiter.park();
        }
    }

    /// driver 接受前缀时只做原子记账，不能在 Bio/Request 锁下转取本域短锁。
    pub(crate) fn note_submitted(&self, bytes: usize) {
        self.submitted_bytes
            .fetch_add(bytes as u64, Ordering::Relaxed);
    }

    /// 在调度前记录真实尾边界；拒准入明确返回停止/错误事实，不合成完成。
    pub(crate) fn request_progress(
        self: &Arc<Self>,
        volume: &Arc<Ext4Volume>,
    ) -> WritebackBoundary {
        let boundary = {
            let state = self.state.lock();
            WritebackBoundary {
                identity: self.as_ref() as *const Self as usize,
                completion_sequence: state.completion.sequence,
                error_sequence: state.completion.error_sequence,
                finished_rounds: state.finished_rounds,
                stop_sequence: state.stop_sequence,
                submitted_bytes: self.submitted_bytes.load(Ordering::Relaxed),
                completed_bytes: state.completion.completed_bytes,
                inactive: !state.active || state.stopping,
                halted: state.halted,
            }
        };
        if !boundary.inactive && !boundary.halted {
            self.schedule(volume, Duration::ZERO);
        }
        boundary
    }

    /// 最多等待一个真实完成/error/有限轮尾/stop 边界，不轮询 rounds 开始或 WB0。
    pub(crate) fn wait_progress(&self, boundary: WritebackBoundary) -> WritebackProgress {
        assert_eq!(
            boundary.identity, self as *const Self as usize,
            "foreign writeback boundary"
        );
        sched::assert_switchable();
        loop {
            if let Some(progress) = self.progress_after(boundary) {
                return progress;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: caller 保真实域，节点固定且登记先于二次边界检查。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if let Some(progress) = self.progress_after(boundary) {
                return progress;
            }
            waiter.park();
        }
    }

    /// 值读取只报告本边界之后的实际 byte 与停止/错误，不把提交量当完成量。
    fn progress_after(&self, boundary: WritebackBoundary) -> Option<WritebackProgress> {
        let state = self.state.lock();
        let stopped = boundary.inactive || state.stop_sequence != boundary.stop_sequence;
        let error = boundary.halted || state.completion.error_sequence != boundary.error_sequence;
        if !stopped
            && !error
            && state.completion.sequence == boundary.completion_sequence
            && state.finished_rounds == boundary.finished_rounds
        {
            return None;
        }
        Some(WritebackProgress {
            submitted_bytes: self
                .submitted_bytes
                .load(Ordering::Relaxed)
                .saturating_sub(boundary.submitted_bytes),
            completed_bytes: state
                .completion
                .completed_bytes
                .saturating_sub(boundary.completed_bytes),
            stopped,
            error,
        })
    }

    /// 真正新 canonical 域绑定后开放原服务；此前停止必须已经结束全部回调尾部。
    pub(crate) fn activate(&self) {
        let mut state = self.state.lock();
        assert!(!state.active && !state.stopping && !state.running && state.pending == 0);
        assert_eq!(
            state.contribution, 0,
            "inactive writeback retains bandwidth"
        );
        state.active = true;
        state.halted = false;
        state.notified = false;
        state.completion.sample_time = None;
        state.completion.bandwidth = 0;
    }

    /// 标脏通知不分配或睡眠；已有有限 pending 票保持不变，运行中的通知留给轮尾复查。
    pub(crate) fn notify_dirty(self: &Arc<Self>, volume: &Arc<Ext4Volume>) {
        {
            let mut state = self.state.lock();
            if !state.active || state.halted {
                return;
            }
            if state.running {
                state.notified = true;
                return;
            }
            if state.pending != 0 {
                return;
            }
        }
        let (_, background) = self.limits();
        let delay = if background <= self.pressure() {
            Duration::ZERO
        } else {
            DIRTY_EXPIRE
        };
        self.schedule(volume, delay);
    }

    /// 调度锁覆盖卷票取得、queue 接受和 pending 发表，拒绝票仅在锁外恢复归还。
    fn schedule(&self, volume: &Arc<Ext4Volume>, delay: Duration) {
        assert!(core::ptr::eq(volume.writeback_domain().as_ref(), self));
        let rejected = {
            let mut state = self.state.lock();
            if !state.active || state.halted {
                return;
            }
            if state.running {
                state.notified = true;
                return;
            }
            if state.pending != 0 {
                return;
            }
            let ticket = Arc::into_raw(volume.clone());
            if self
                .work()
                .queue(delay)
                .expect("prepared writeback queue rejected")
            {
                state.pending = ticket as usize;
                None
            } else {
                Some(ticket)
            }
        };
        if let Some(ticket) = rejected {
            // SAFETY: ticket 来自本次 into_raw 且 queue 未接受，唯一归还者在调度锁外消费它。
            drop(unsafe { Arc::from_raw(ticket) });
        }
    }

    /// cache 关闭且本卷最后 canonical 槽退休后调用；不能持索引锁或从自身回调等待。
    pub(crate) fn stop_sync(&self) {
        sched::assert_switchable();
        let _stop = self.stop.lock();
        {
            let mut state = self.state.lock();
            state.active = false;
            state.stopping = true;
            state.stop_sequence += 1;
            self.metadata_sequence.fetch_add(1, Ordering::Release);
            ACTIVE_BANDWIDTH.fetch_sub(state.contribution, Ordering::Relaxed);
            state.contribution = 0;
        }
        self.waiters.wake_all();
        if let Some(source) = &self.source {
            source.notify_available();
        }
        self.work().cancel_sync();
        self.wait_storage_returned();
        let ticket = {
            let mut state = self.state.lock();
            assert!(!state.running, "writeback callback survived cancellation");
            state.stopping = false;
            state.notified = false;
            core::mem::take(&mut state.pending)
        };
        if ticket != 0 {
            // SAFETY: cancel_sync 已结束 timer/Work 全尾；未进入 callback 的唯一 pending 票只在此归还。
            drop(unsafe { Arc::from_raw(ticket as *const Ext4Volume) });
        }
    }

    /// 用途锁内仅增减本卷真实 mask 的零与非零转换，不进入调度、分配或等待。
    pub(crate) fn account_folio(&self, old_dirty: u8, new_dirty: u8, old_wb: u8, new_wb: u8) {
        account_mask(&self.dirty_pages, old_dirty, new_dirty);
        account_mask(&self.writeback_pages, old_wb, new_wb);
    }

    /// 内容用途锁外记录真实 WB 起点；没有完成以前不会发表任何带宽。
    pub(crate) fn note_writeback_start(&self) {
        self.state.lock().completion.start(time::now());
    }

    /// 真实完成前缀更新本卷计数、活域带宽贡献和通知，原 errseq 仍由 mapping 保存。
    pub(crate) fn note_completion(
        &self,
        accepted: usize,
        completed: usize,
        error: Option<FsError>,
    ) {
        if accepted == 0 {
            return;
        }
        {
            let mut state = self.state.lock();
            state.completion.complete(accepted, completed, time::now());
            if error.is_some() {
                state.completion.error_sequence += 1;
            }
            let next = if state.active {
                state.completion.bandwidth
            } else {
                0
            };
            if state.contribution <= next {
                ACTIVE_BANDWIDTH.fetch_add(next - state.contribution, Ordering::Relaxed);
            } else {
                ACTIVE_BANDWIDTH.fetch_sub(state.contribution - next, Ordering::Relaxed);
            }
            state.contribution = next;
        }
        self.waiters.wake_all();
    }

    /// 准备或提交返回真实错误后立即结束合理限速等待，错误本身仍留在原 Result/errseq。
    fn note_error(&self) {
        self.state.lock().completion.error_sequence += 1;
        self.waiters.wake_all();
    }

    /// 后台同步在标记 ToWrite 后立即记录实际数量，后续准备错误不会抹去已选事实。
    pub(crate) fn note_selection(&self, pages: usize, metadata: bool) {
        let mut state = self.state.lock();
        assert!(state.running, "background selection outside its round");
        if metadata {
            state.round.selected_metadata += pages;
            assert!(state.round.selected_metadata <= METADATA_BUDGET);
        } else {
            state.round.selected_data += pages;
            assert!(state.round.selected_data <= DATA_BUDGET);
        }
    }

    /// 本轮原凭证在真实完成时分别报告 accepted 与 completed，不将选页数当提交量。
    pub(crate) fn note_round_io(&self, accepted: usize, completed: usize) {
        let mut state = self.state.lock();
        state.round.accepted_bytes += accepted as u64;
        state.round.completed_bytes += completed as u64;
    }

    /// 页失效或脏状态变化后允许 writer 重查本域压力，不要求整个系统清脏。
    pub(crate) fn wake_writers(&self) {
        self.waiters.wake_all();
    }

    /// 只读真实计数及最后一个有限 round 的事实，不驱动 WB 或合成完成。
    #[cfg(feature = "ktest")]
    pub(crate) fn snapshot(&self) -> WritebackSnapshot {
        let state = self.state.lock();
        WritebackSnapshot {
            dirty_pages: self.dirty_pages.load(Ordering::Relaxed),
            writeback_pages: self.writeback_pages.load(Ordering::Relaxed),
            submitted_bytes: self.submitted_bytes.load(Ordering::Relaxed),
            accepted_bytes: state.completion.accepted_bytes,
            completed_bytes: state.completion.completed_bytes,
            completion_seq: state.completion.sequence,
            error_seq: state.completion.error_sequence,
            bandwidth_bytes_per_sec: state.completion.bandwidth,
            #[cfg(feature = "ktest")]
            sample_bytes: state.completion.last_sample_bytes,
            #[cfg(feature = "ktest")]
            sample_elapsed_ns: state.completion.last_sample_elapsed_ns,
            #[cfg(feature = "ktest")]
            sample_end_ns: state.completion.last_sample_end_ns,
            rounds: state.rounds,
            finished_rounds: state.finished_rounds,
            stop_sequence: state.stop_sequence,
            round_limit: state.round.limit,
            round_scanned: state.round.scanned,
            round_selected_data: state.round.selected_data,
            round_selected_metadata: state.round.selected_metadata,
            round_accepted_bytes: state.round.accepted_bytes,
            round_completed_bytes: state.round.completed_bytes,
            later_dirty: state.round.later_dirty,
            writer_waits: state.writer_waits,
            writer_wakes: state.writer_wakes,
            waiting_writers: state.waiting_writers,
            active: state.active,
            pending: state.pending != 0,
            running: state.running,
        }
    }

    /// 观察有限轮次已结束、后继尚未由本回调安排的真实边界，旧回调在短锁外释放。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_round_hook(&self, hook: Option<RoundHook>) {
        let previous = core::mem::replace(&mut *self.round_hook.lock(), hook);
        drop(previous);
    }

    /// 读取当前实际带宽与脏占比推导的本域上下限，不修改阈值或制造测试压力。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_limits(&self) -> (usize, usize) {
        self.limits()
    }

    /// 全局上限按活域实测带宽分配；没有本域样本时只借当前真实脏占比启动 WB。
    fn limits(&self) -> (usize, usize) {
        let (_, global_limit) = dirty_limit();
        let bandwidth = self.state.lock().completion.bandwidth;
        let total_bandwidth = ACTIVE_BANDWIDTH.load(Ordering::Relaxed);
        let limit = if bandwidth != 0 && total_bandwidth != 0 {
            (u128::from(global_limit as u64) * u128::from(bandwidth) / u128::from(total_bandwidth))
                as usize
        } else {
            let own = self.pressure();
            let total =
                DIRTY_PAGES.load(Ordering::Relaxed) + WRITEBACK_PAGES.load(Ordering::Relaxed);
            if total == 0 {
                global_limit
            } else {
                global_limit.saturating_mul(own) / total
            }
        }
        .clamp(1, global_limit);
        (limit, (limit / 2).max(1))
    }

    /// Dirty 与 WB 分别保留同页的两种义务，限速不把在途页当作已完成。
    fn pressure(&self) -> usize {
        self.dirty_pages.load(Ordering::Relaxed) + self.writeback_pages.load(Ordering::Relaxed)
    }

    /// 返回构造时准备的固定 Work，不建立新控制对象或更换队列。
    fn work(&self) -> Pin<&DelayedWork> {
        self.work
            .get()
            .expect("writeback work is not prepared")
            .as_ref()
    }

    /// 一次回调至多走入口槽数和 48+16 页；首错仍保留下轮实际脏义务。
    fn run_round(self: &Arc<Self>, volume: &Arc<Ext4Volume>) {
        let limit = volume.dirty_inode_limit();
        let start = {
            let mut state = self.state.lock();
            state.rounds += 1;
            state.round = Round {
                limit,
                ..Round::default()
            };
            state.notified = false;
            if limit == 0 {
                0
            } else {
                state.next_slot % limit
            }
        };
        let mut first_error = None;
        let mut unconfirmed = false;
        match self.borrow_storage() {
            Ok(mut storage) => {
                for offset in 0..limit {
                    let remaining = DATA_BUDGET - self.state.lock().round.selected_data;
                    if remaining == 0 || storage.stopped(self) {
                        break;
                    }
                    let slot = (start + offset) % limit;
                    {
                        let mut state = self.state.lock();
                        state.round.scanned += 1;
                        state.next_slot = (slot + 1) % limit;
                    }
                    let Some(inode) = volume.dirty_inode_at(slot) else {
                        continue;
                    };
                    if let Err(error) =
                        writeback::sync_inode_budget(&inode, remaining, &mut storage)
                    {
                        first_error.get_or_insert(error);
                        self.note_error();
                        if error == FsError::Io(BlockError::StopUnconfirmed) {
                            unconfirmed = true;
                            break;
                        }
                    }
                }
                if !unconfirmed && !storage.stopped(self) {
                    if let Err(error) =
                        writeback::sync_metadata_budget(volume, METADATA_BUDGET, &mut storage)
                    {
                        first_error.get_or_insert(error);
                        self.note_error();
                        unconfirmed = error == FsError::Io(BlockError::StopUnconfirmed);
                    }
                }
            }
            Err(error) => {
                first_error = Some(error);
                self.note_error();
            }
        }
        let pending = self.pressure() != 0 || volume.metadata_cache().has_dirty_or_writeback();
        let again = {
            let mut state = self.state.lock();
            state.round.later_dirty = pending;
            state.halted = unconfirmed;
            state.running = false;
            state.finished_rounds += 1;
            state.active && !state.halted && (pending || state.notified)
        };
        self.waiters.wake_all();
        #[cfg(feature = "ktest")]
        let hook = self.round_hook.lock().clone();
        #[cfg(feature = "ktest")]
        if let Some(hook) = hook {
            hook(self.snapshot());
        }
        if again {
            let delay = if first_error.is_some() {
                DIRTY_EXPIRE
            } else {
                Duration::ZERO
            };
            self.schedule(volume, delay);
        }
    }
}

impl Drop for WritebackDomain {
    /// 最后 owner 只核对已接受责任和带宽贡献已退休，不在析构中等待 worker。
    fn drop(&mut self) {
        let state = self.state.lock();
        assert_eq!(state.pending, 0, "dropping pending writeback volume ticket");
        assert!(!state.running, "dropping running writeback domain");
        assert_eq!(state.contribution, 0, "dropping active writeback bandwidth");
        assert!(
            self.source.is_none() || self.scratch.lock().is_some(),
            "dropping borrowed writeback scratch"
        );
    }
}

/// 回调先唯一消费 pending 卷票，局部 Arc 通过卷字段保域到最后一次业务访问。
fn run_writeback(work: Pin<&DelayedWork>) {
    let pointer = work.context() as *const WritebackDomain;
    // SAFETY: 接受 queue 前取得的真实卷票强持已发表的域；pending 取走前该票不会被任何其他路径消费。
    let domain = unsafe { &*pointer };
    let ticket = {
        let mut state = domain.state.lock();
        assert!(!state.running, "writeback callback reentered");
        let ticket = core::mem::take(&mut state.pending);
        assert_ne!(ticket, 0, "writeback callback lost volume ticket");
        state.running = true;
        ticket
    };
    // SAFETY: ticket 是当前回调从 pending 唯一取走的 into_raw 强票；cancel_sync 只能归还未被取走的票。
    let volume = unsafe { Arc::from_raw(ticket as *const Ext4Volume) };
    volume.writeback_domain().run_round(&volume);
}

/// file-role 在真实安装和最终 take_backing 时配对，索引数量不参与此计数。
pub(crate) fn account_file_page(added: bool) {
    if added {
        FILE_PAGES.fetch_add(1, Ordering::Relaxed);
    } else {
        assert!(
            0 < FILE_PAGES.fetch_sub(1, Ordering::Relaxed),
            "file page count underflow"
        );
    }
}

/// 每次原页用途状态转换只计算 mask 的零与非零，不按写入 byte 重复计页。
pub(crate) fn account_folio(old_dirty: u8, new_dirty: u8, old_wb: u8, new_wb: u8) {
    account_mask(&DIRTY_PAGES, old_dirty, new_dirty);
    account_mask(&WRITEBACK_PAGES, old_wb, new_wb);
}

/// 保持一个页级标量与原 mask 的零边界配对，不执行任何调度或复杂析构。
fn account_mask(counter: &AtomicUsize, old: u8, new: u8) {
    if old == 0 && new != 0 {
        counter.fetch_add(1, Ordering::Relaxed);
    } else if old != 0 && new == 0 {
        assert!(
            0 < counter.fetch_sub(1, Ordering::Relaxed),
            "folio state count underflow"
        );
    }
}

/// 原页开始真实 WB 时在用途锁外建立全局测量起点。
pub(crate) fn note_writeback_start() {
    GLOBAL_COMPLETION.lock().start(time::now());
}

/// 全局只累计已经结束的真实 accepted 范围及其完成前缀；取消不增加吞吐。
pub(crate) fn note_completion(accepted: usize, completed: usize) {
    if accepted != 0 {
        GLOBAL_COMPLETION
            .lock()
            .complete(accepted, completed, time::now());
    }
}

/// 无匿名 swap/LRU 政策时，以实际空闲加存活 File-role 页近似当前文件 dirtyable。
fn dirty_limit() -> (usize, usize) {
    let memory = mm::stats();
    let dirtyable =
        memory.buddy_free_pages + memory.pcp_cached_pages + FILE_PAGES.load(Ordering::Relaxed);
    (
        dirtyable,
        (dirtyable * DIRTY_RATIO / PERCENT).clamp(MIN_DIRTY_PAGES, MAX_DIRTY_PAGES),
    )
}

/// 只读取本 VM 当前原页账和真实完成样本，跨锁快照不提供停稳证明。
#[cfg(feature = "ktest")]
pub(crate) fn global_snapshot() -> DirtySnapshot {
    let (dirtyable, limit) = dirty_limit();
    let completion = GLOBAL_COMPLETION.lock();
    DirtySnapshot {
        file_pages: FILE_PAGES.load(Ordering::Relaxed),
        dirty_pages: DIRTY_PAGES.load(Ordering::Relaxed),
        writeback_pages: WRITEBACK_PAGES.load(Ordering::Relaxed),
        dirtyable_pages: dirtyable,
        dirty_limit: limit,
        background_limit: limit / 2,
        completed_bytes: completion.completed_bytes,
        bandwidth_bytes_per_sec: completion.bandwidth,
    }
}

/// 磁盘 writer 在释放 inode、cursor、invalidate 和内容 guard 后按本域实测完成有限等待。
/// 本接口不返回写入错误，已接受的正前缀不能被限速改成未写；原错误由 errseq 消费。
pub(crate) fn balance_dirty_pages(inode: &InodeRef) {
    sched::assert_switchable();
    let volume = inode
        .disk_volume()
        .expect("dirty balancing requires a disk inode");
    let domain = volume.writeback_domain();
    let (limit, background) = domain.limits();
    let pressure = domain.pressure();
    if pressure <= background {
        return;
    }
    domain.notify_dirty(volume);
    if pressure <= limit {
        return;
    }
    let (sequence, errors, completed, bandwidth) = {
        let state = domain.state.lock();
        (
            state.completion.sequence,
            state.completion.error_sequence,
            state.completion.completed_bytes,
            state.completion.bandwidth,
        )
    };
    let excess = ((pressure - limit) * PAGE_SIZE) as u64;
    let (target, deadline) = if bandwidth == 0 {
        (completed, None)
    } else {
        let nanos = ((u128::from(excess) * u128::from(NSEC_PER_SEC))
            .div_ceil(u128::from(bandwidth)))
        .min(u128::from(MAX_PAUSE_NS)) as u64;
        let target_bytes =
            ((u128::from(bandwidth) * u128::from(nanos)) / u128::from(NSEC_PER_SEC)) as u64;
        let deadline = time::now()
            .checked_add(Duration::from_nanos(nanos))
            .expect("dirty balance deadline overflow");
        (completed + target_bytes.max(1).min(excess), Some(deadline))
    };
    let mut waiting = false;
    loop {
        let waiter = Waiter::new();
        let mut node = pin!(WaitNode::new(waiter.waker()));
        // SAFETY: inode 强持卷和固定域，栈节点固定；每次退出先撤登记，之后才释放节点与 Waiter。
        let registration = unsafe { domain.waiters.register(node.as_mut()) };
        let mut state = domain.state.lock();
        let finished = !state.active
            || state.halted
            || state.completion.error_sequence != errors
            || domain.pressure() <= limit
            || if deadline.is_none() {
                state.completion.sequence != sequence
            } else {
                target <= state.completion.completed_bytes
                    || deadline.is_some_and(|deadline| deadline <= time::now())
            };
        if finished {
            drop(state);
            drop(registration);
            drop(waiter);
            break;
        }
        if !waiting {
            state.writer_waits += 1;
            state.waiting_writers += 1;
            waiting = true;
        }
        drop(state);
        let waker = pin!(waiter.waker());
        let timer = pin!(Timer::new(
            crate::cpu::current().id,
            TimerKind::Precise,
            wake_writer,
            waker.as_ref().get_ref() as *const Waker as usize
        ));
        if let Some(deadline) = deadline {
            // SAFETY: timer 与 Waker 固定在本栈，取消同步结束 IRQ 真尾部后才退出作用域；回调只通知。
            unsafe { timer.as_ref().arm_borrowed(deadline) }
                .expect("fresh dirty balance timer rejected");
        }
        waiter.park();
        domain.state.lock().writer_wakes += 1;
        drop(registration);
        // Timer 同步取消自身可能等待，必须先结束当前 Task 的唯一 active wait。
        drop(waiter);
        if deadline.is_some() {
            timer.as_ref().cancel_sync();
        }
    }
    if waiting {
        domain.state.lock().waiting_writers -= 1;
    }
}

/// 精确定时器只通知本次等待，不把超时发布为写回完成或清除 Dirty。
fn wake_writer(timer: Pin<&Timer>) {
    // SAFETY: balance 的固定 Waker 保持到 cancel_sync 确认回调尾部，IRQ 不保存指针或访问页内容。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}
