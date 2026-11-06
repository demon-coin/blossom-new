//! 卷级后台回写调度域（`WritebackDomain`）、实测回写带宽采样与前台脏页平衡限速（`balance_dirty_pages`）模块。
//!
//! # 模块职责与架构定位
//! 本模块为每个已挂载的 ext4 卷维护独立的后台脏页回写生命周期、脏页/在途回写页计数、实测完成带宽估算以及写入者流控：
//! - **上游调用者**：文件写入与缺页写路径（调用 [`balance_dirty_pages`] 进行脏页限速）、页缓存状态变更（调用 [`account_folio`] / [`WritebackDomain::notify_dirty`]）、内存回收扫描（调用 `request_progress` / `wait_progress`）以及卷挂载与退休流程（`activate` / `stop_sync`）。
//! - **下游依赖**：专用内存回收级工作队列 `WB_QUEUE`（[`WorkQueue`]）、高精度单调时钟与定时器（[`Timer`]）、统一页面回写引擎（[`crate::fs::writeback`]）以及块层预留内存池（[`BioReserve`]）。
//!
//! # 核心机制与状态机
//!
//! ## 1. `WritebackDomain` 生命周期与异步卷票（Raw `Arc` Ticket）转移
//! `Ext4Volume` 强持有 `Arc<WritebackDomain>`。为避免空闲时循环引用导致卷无法卸载，后台任务 `DelayedWork` 仅保存指向 `WritebackDomain` 的裸指针；
//! 仅当 `schedule` 成功将任务排入工作队列时，才通过 `Arc::into_raw(volume.clone())` 导出唯一一份卷强引用票存入 `State::pending`：
//!
//! ```text
//!   try_new() ──► Inactive (active=false, pending=0, running=false)
//!                    │
//!                    │ activate() (卷绑定至 Inode 域后启用)
//!                    ▼
//!                 Active ◄──────────────────────────────────────────┐
//!                    │                                              │
//!                    │ schedule(): queue(delay) 接受任务             │ run_round 结束，且无剩余脏页
//!                    │ 导出唯一卷票存入 state.pending                │ (或再次 schedule 开启下一轮)
//!                    ▼                                              │
//!                 Queued (pending != 0, running=false)              │
//!                    │                                              │
//!                    ├─► run_writeback 回调启动:                     │
//!                    │   原子取走 pending 还原 Arc<Ext4Volume>，      │
//!                    │   置 running=true 执行最多 48+16 页定量回写 ──┘
//!                    │
//!                    └─► stop_sync() (卷退休停机):
//!                        1. 置 active=false, stopping=true, 递增 stop_sequence
//!                        2. cancel_sync() 取消未触发定时器并等待运行中回调退出
//!                        3. wait_storage_returned() 等待借出的 WbStorage 归还
//!                        4. 若 pending != 0 (回调未启动即被取消)，回收该卷票
//! ```
//!
//! ## 2. 全局与分卷脏页阈值及前台写入限速
//! - **全局可脏页基数（`dirtyable`）**：由伙伴分配器空闲页 + 每核 PCP 缓存页 + 存活文件缓存页（`FILE_PAGES`）之和近似，按 `DIRTY_RATIO`（10%）折算并钳制在 `[32, 4096]` 页之间作为全局上限 `global_limit`。
//! - **分卷配额（`limits`）**：优先按本卷实测带宽占全局活跃卷总带宽（`ACTIVE_BANDWIDTH`）的比例切分 `global_limit`；尚无带宽样本时按当前脏/回写页占比分配。后台唤醒阈值为硬上限的一半（`limit / 2`）。
//! - **前台限速（`balance_dirty_pages`）**：写入者在释放全部文件系统与页面锁后检查本卷压力（`dirty_pages + writeback_pages`）：超过后台阈值时触发异步回写；超过硬上限时按超出字节数与实测带宽计算最多 `200ms` 的暂停窗口睡眠等待。
//!
//! # 锁获取顺序与并发契约
//! 1. **禁止持文件系统锁进入限速或停机**：调用 [`balance_dirty_pages`] 前必须已释放 Inode 锁、目录游标、失效锁与 `FolioContent` 锁；调用 [`WritebackDomain::stop_sync`] 时不得持有缓存索引锁，也不得在后台工作回调自身内部调用。
//! 2. **停机串行化睡眠锁**（`WritebackDomain::stop: SleepMutex`）：包裹整个停机取消与工作缓冲区回收等待流程。
//! 3. **域状态与缓冲区自旋锁**（`WritebackDomain::state`、`WritebackDomain::scratch`、`GLOBAL_COMPLETION`）：均为极短临界区自旋锁，持锁期间严禁睡眠、堆分配或发起块层请求；所有等待队列唤醒（`waiters.wake_all()`）与被拒卷票释放均在释放 `state` 锁之后执行。
//! 4. **页描述符用途锁与块层提交锁隔离**：`account_folio` 仅在 `PageDesc` 用途锁内对原子计数器执行零/非零边沿增减，不获取 `state` 自旋锁；`note_submitted` 仅做原子累加，防止在块层 `Bio`/队列自旋锁下嵌套获取回写域锁。

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

/// 全局脏页硬上限占系统可脏页总数（`dirtyable`）的百分比分子（10%）。
const DIRTY_RATIO: usize = 10;
/// 单轮后台回写回调最多标记并回写的文件数据页预算上限（48 页）。
const DATA_BUDGET: usize = 48;
/// 单轮后台回写回调最多标记并回写的卷元数据页预算上限（16 页）。
const METADATA_BUDGET: usize = 16;
/// 脏页未达后台压力阈值时的默认延迟回写超时期限，以及回写遇错后的重试退避延迟（200 毫秒）。
const DIRTY_EXPIRE: Duration = Duration::from_millis(200);
/// 前台写入者在 `balance_dirty_pages` 中单次阻塞限速等待的最长持续时间（200 毫秒，单位：纳秒）。
const MAX_PAUSE_NS: u64 = 200_000_000;
/// 更新实测回写带宽样本的最小滑动采样时间窗长度（200 毫秒，单位：纳秒）。
const BANDWIDTH_INTERVAL_NS: u64 = 200_000_000;
/// 全局后台回写工作队列 `WB_QUEUE` 允许并发运行的最大活跃工作项数量。
const WB_MAX_ACTIVE: usize = 8;
/// 全局脏页硬上限的下界保护值（单位：4 KiB 页）。
const MIN_DIRTY_PAGES: usize = 32;
/// 全局脏页硬上限的上界保护值（单位：4 KiB 页）。
const MAX_DIRTY_PAGES: usize = 4096;
/// 百分比换算分母常量（100）。
const PERCENT: usize = 100;

/// 全局共享的后台回写工作队列（启用 `reclaim` 救援进展保证，最多 `WB_MAX_ACTIVE` 个并发工作项）。
static WB_QUEUE: Once<WorkQueue> = Once::new();
/// 全系统当前已安装且存活的文件角色（`File`）页缓存物理页总数。
static FILE_PAGES: AtomicUsize = AtomicUsize::new(0);
/// 全系统当前至少包含一个脏扇区（`dirty != 0`）的页缓存物理页总数（含文件页与元数据页）。
static DIRTY_PAGES: AtomicUsize = AtomicUsize::new(0);
/// 全系统当前至少包含一个在途回写扇区（`writeback != 0`）的页缓存物理页总数（含文件页与元数据页）。
static WRITEBACK_PAGES: AtomicUsize = AtomicUsize::new(0);
/// 当前所有处于活跃状态（`active == true`）的回写域实测带宽之和（单位：字节/秒）。
static ACTIVE_BANDWIDTH: AtomicU64 = AtomicU64::new(0);
/// 全局回写完成字节数与吞吐率采样器（受自旋锁保护）。
static GLOBAL_COMPLETION: SpinLock<Completion> = SpinLock::new(Completion::new());

/// 回写完成流量统计与基于单调时钟窗口的实测吞吐率采样器。
///
/// 仅根据块设备真实结束的字节增量与对应的单调时钟时间差计算带宽；时间差为零时不生成无效样本。
struct Completion {
    /// 已提交至设备并结束处理的请求总范围字节数（含部分失败请求的总长度，单位：字节）。
    accepted_bytes: u64,
    /// 设备实际成功落盘的累计完成字节数（单位：字节）。
    completed_bytes: u64,
    /// 回写批次完成事件单调递增序号，供等待者判断是否有新完成进展。
    sequence: u64,
    /// 回写准备或传输错误事件单调递增序号，供限速与内存回收等待者及时感知异常并退出等待。
    error_sequence: u64,
    /// 当前带宽采样窗口的起始单调时间戳；尚未发起任何回写时为 `None`。
    sample_time: Option<MonotonicInstant>,
    /// 当前带宽采样窗口起始时刻对应的 `completed_bytes` 快照值（单位：字节）。
    sample_bytes: u64,
    /// 最近一次有效采样窗口测得的设备回写吞吐率（单位：字节/秒）；建立首样本前为 `0`。
    bandwidth: u64,
    /// 最近一次更新带宽样本时窗口内累计完成的字节数（仅 `ktest` 诊断核对使用）。
    #[cfg(feature = "ktest")]
    last_sample_bytes: u64,
    /// 最近一次更新带宽样本时窗口经历的纳秒时长（仅 `ktest` 诊断核对使用）。
    #[cfg(feature = "ktest")]
    last_sample_elapsed_ns: u64,
    /// 最近一次更新带宽样本时的窗口结束单调时钟纳秒值（仅 `ktest` 诊断核对使用）。
    #[cfg(feature = "ktest")]
    last_sample_end_ns: u64,
}

impl Completion {
    /// 创建初始零状态采样器；在设备实际完成回写前带宽保持为 `0`，不使用预设默认吞吐率。
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

    /// 在物理页首次进入回写状态时记录初始采样时间起点；若当前已有进行中的采样窗口则保持原起点不变。
    fn start(&mut self, now: MonotonicInstant) {
        if self.sample_time.is_none() {
            self.sample_time = Some(now);
            self.sample_bytes = self.completed_bytes;
        }
    }

    /// 在一批已接受的回写请求结束时累加完成字节数，并按采样时间窗口更新实测带宽（首个非零耗时样本立即建立，后续每满 `BANDWIDTH_INTERVAL_NS` 滚动更新）。
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

/// 单轮后台定量回写（`run_round`）的扫描与完成统计记录。
///
/// 仅保存标量统计值，不额外持有任何 `Inode` 或物理页引用。
#[derive(Clone, Copy, Default)]
struct Round {
    /// 本轮启动时脏 Inode 索引表的槽位总数上限。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "ktest 的有限回写轮观察核对扫描上限与实际扫描数")
    )]
    limit: usize,
    /// 本轮实际扫描过的脏 Inode 槽位数量。
    scanned: usize,
    /// 本轮标记进入 `ToWrite` 的文件数据页总数（上限为 `DATA_BUDGET`）。
    selected_data: usize,
    /// 本轮标记进入 `ToWrite` 的卷元数据页总数（上限为 `METADATA_BUDGET`）。
    selected_metadata: usize,
    /// 本轮发起的后台回写凭证在完成时累计报告的已接受字节数（单位：字节）。
    accepted_bytes: u64,
    /// 本轮发起的后台回写凭证在完成时累计报告的实际成功落盘字节数（单位：字节）。
    completed_bytes: u64,
    /// 本轮结束时本卷是否仍存在未清零的脏页或在途回写页，决定是否需要继续调度下一轮。
    later_dirty: bool,
}

/// 受 [`WritebackDomain::state`] 自旋锁保护的卷级回写调度与统计可变状态。
struct State {
    /// 当前回写域是否已激活并允许接纳新的后台回写调度与工作缓冲区借用。
    active: bool,
    /// 当前回写域是否正处于 `stop_sync` 停机排空流程中。
    stopping: bool,
    /// 是否因底层块设备返回 `StopUnconfirmed`（DMA 未确认停止）而永久熔断后续自动调度。
    halted: bool,
    /// 已排入工作队列但尚未被 `run_writeback` 回调取走的唯一 `Arc::into_raw(volume)` 裸指针地址；无排队任务时为 `0`。
    pending: usize,
    /// 后台回写回调 `run_writeback` 当前是否正在执行本卷的回写轮次。
    running: bool,
    /// 在 `running == true` 期间是否收到过新的标脏或推进通知；若为 `true` 则在轮末补调下一轮。
    notified: bool,
    /// 下一轮后台回写扫描脏 Inode 槽位表时的起始轮询下标，避免前排热门文件饥饿后续文件。
    next_slot: usize,
    /// 本卷的回写完成字节计数与实测吞吐率采样器。
    completion: Completion,
    /// 本卷当前计入全局 `ACTIVE_BANDWIDTH` 的带宽份额（单位：字节/秒）；停机时清零。
    contribution: u64,
    /// 累计已启动的后台回写轮次计数。
    rounds: u64,
    /// 累计已执行完毕退出的后台回写轮次计数。
    finished_rounds: u64,
    /// 每次调用 `stop_sync` 时单调递增的停机代次序号，用于使旧借用的工作缓冲区失效。
    stop_sequence: u64,
    /// 当前正在执行或最近一轮后台回写的明细统计。
    round: Round,
    /// 前台写入者在 `balance_dirty_pages` 中因超限而进入睡眠限速的累计次数。
    writer_waits: u64,
    /// 前台限速睡眠写入者从 `park` 恢复唤醒的累计次数。
    writer_wakes: u64,
    /// 当前正处于 `balance_dirty_pages` 限速等待循环中的写入者线程数量。
    waiting_writers: usize,
}

/// `ktest` 专用回调钩子类型：在每轮后台回写完成并唤醒等待者后、调度下一轮前接收只读状态快照。
#[cfg(feature = "ktest")]
type RoundHook = Arc<dyn Fn(WritebackSnapshot) + Send + Sync>;

/// 单个 `Ext4Volume` 专属的后台回写控制、脏页记账与写入限速协调域。
///
/// 由所属 `Ext4Volume` 强持有；自身仅在异步工作项已入队（`pending != 0`）或正在执行（`running`）期间通过临时卷票反向保活 `Ext4Volume`，空闲时不形成循环引用。
pub(crate) struct WritebackDomain {
    /// 构造时一次性初始化并固定在堆上的延迟工作项（绑定至全局 `WB_QUEUE`）。
    work: Once<Pin<Arc<DelayedWork>>>,
    /// 保护回写生命周期标志、排队卷票、轮次游标与带宽统计的自旋锁。
    state: SpinLock<State>,
    /// 串行化 `stop_sync` 停机流程的睡眠互斥锁。
    stop: SleepMutex<()>,
    /// 读写卷关联的块层预留内存池（只读挂载卷为 `None`）。
    source: Option<Arc<BioReserve>>,
    /// 每次卷停机时递增的元数据 I/O 边界序号，供 `MetadataIoContext` 拦截跨停机周期的旧请求。
    metadata_sequence: AtomicU64,
    /// 读写卷预分配的唯一后台回写工作缓冲区槽位；借出期间为 `None`，归还后恢复为 `Some`。
    scratch: SpinLock<Option<Box<writeback::WbStorage>>>,
    /// 本卷当前含脏扇区的缓存物理页数量（含本卷文件页与元数据页）。
    dirty_pages: AtomicUsize,
    /// 本卷当前含在途回写扇区的缓存物理页数量（含本卷文件页与元数据页）。
    writeback_pages: AtomicUsize,
    /// 块设备驱动已接纳进入硬件传输队列的本卷回写累计字节数（单位：字节）。
    submitted_bytes: AtomicU64,
    /// 供前台限速写入者、内存回收等待者及等待 `scratch` 归还的线程共用的等待队列。
    waiters: WaitQueue,
    /// 测试专用的单轮回写完成观察钩子。
    #[cfg(feature = "ktest")]
    round_hook: SpinLock<Option<RoundHook>>,
}

/// 内存回收等调用方在请求推进回写前捕获的回写域状态快照边界。
///
/// 仅保存标量序号与字节计数，供 [`WritebackDomain::wait_progress`] 比对是否有真实完成、轮次结束、错误或停机事件发生。
#[derive(Clone, Copy, Debug)]
pub(crate) struct WritebackBoundary {
    /// 产生该边界的 `WritebackDomain` 实例内存地址标识，用于断言防止跨卷混用边界。
    identity: usize,
    /// 捕获时刻的回写完成事件序号（`Completion::sequence`）。
    completion_sequence: u64,
    /// 捕获时刻的回写错误事件序号（`Completion::error_sequence`）。
    error_sequence: u64,
    /// 捕获时刻已结束的后台回写轮次数（`State::finished_rounds`）。
    finished_rounds: u64,
    /// 捕获时刻的停机代次序号（`State::stop_sequence`）。
    stop_sequence: u64,
    /// 捕获时刻驱动已接纳的累计提交字节数（单位：字节）。
    submitted_bytes: u64,
    /// 捕获时刻设备已完成的累计落盘字节数（单位：字节）。
    completed_bytes: u64,
    /// 捕获时刻该回写域是否未激活或正在停机。
    inactive: bool,
    /// 捕获时刻该回写域是否已因停机未确认而熔断停止。
    halted: bool,
}

/// 相对于某一 [`WritebackBoundary`] 观察到的回写推进结果增量。
#[derive(Clone, Copy, Debug)]
pub(crate) struct WritebackProgress {
    /// 自边界捕获以来新增提交至设备驱动的字节数（单位：字节）。
    pub submitted_bytes: u64,
    /// 自边界捕获以来新增由设备完成落盘的字节数（单位：字节）。
    pub completed_bytes: u64,
    /// 回写域在边界时已处于非活跃状态，或在等待期间发生了停机。
    pub stopped: bool,
    /// 回写域在边界时已熔断，或在等待期间记录到了新的回写错误。
    pub error: bool,
}

/// 从 [`WritebackDomain`] 独占借出的预分配回写工作缓冲区租约守卫（RAII Guard）。
///
/// 借出期间将唯一的 `Box<WbStorage>` 移出回写域并进入 `ReclaimScope::noio()` 保护作用域；
/// 析构时在锁外清空所有临时引用与块层通道，再将空缓冲区放回域槽位并唤醒等待者。
pub(crate) struct WbStorageLease {
    /// 借出该工作缓冲区的所属回写域强引用。
    domain: Arc<WritebackDomain>,
    /// 当前租约独占持有的预分配工作缓冲区；仅在 `Drop::drop` 归还时取走变为 `None`。
    storage: Option<Box<writeback::WbStorage>>,
    /// 租约存活期间禁止当前任务递归进入 I/O 型内存回收的作用域守卫。
    _scope: ReclaimScope,
}

impl Deref for WbStorageLease {
    type Target = writeback::WbStorage;

    /// 返回当前租约独占持有的 [`writeback::WbStorage`] 共享引用。
    fn deref(&self) -> &Self::Target {
        self.storage
            .as_deref()
            .expect("writeback storage lease ended")
    }
}

impl DerefMut for WbStorageLease {
    /// 返回当前租约独占持有的 [`writeback::WbStorage`] 可变引用，供同一回写任务跨多个滑动窗口复用。
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.storage
            .as_deref_mut()
            .expect("writeback storage lease ended")
    }
}

impl Drop for WbStorageLease {
    /// 析构时取走工作缓冲区，在回写域自旋锁外清空临时凭证后归还至 `scratch` 槽位并唤醒等待队列。
    fn drop(&mut self) {
        let storage = self
            .storage
            .take()
            .expect("writeback storage returned twice");
        self.domain.return_storage(storage);
    }
}

/// 单个卷回写域在某一时刻的只读运行状态与统计快照（仅 `ktest` 测试断言与诊断使用）。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct WritebackSnapshot {
    /// 本卷当前脏缓存页数。
    pub dirty_pages: usize,
    /// 本卷当前在途回写缓存页数。
    pub writeback_pages: usize,
    /// 驱动已接纳的累计回写字节数（单位：字节）。
    pub submitted_bytes: u64,
    /// 已完成回写批次的总接受范围字节数（单位：字节）。
    pub accepted_bytes: u64,
    /// 设备实际成功落盘的累计字节数（单位：字节）。
    pub completed_bytes: u64,
    /// 回写批次完成事件单调序号。
    pub completion_seq: u64,
    /// 回写错误事件单调序号。
    pub error_seq: u64,
    /// 本卷最近一次采样测得的回写带宽（单位：字节/秒）。
    pub bandwidth_bytes_per_sec: u64,
    /// 最近一次带宽采样窗口内完成的字节数（单位：字节）。
    #[cfg(feature = "ktest")]
    pub sample_bytes: u64,
    /// 最近一次带宽采样窗口经历的时长（单位：纳秒）。
    #[cfg(feature = "ktest")]
    pub sample_elapsed_ns: u64,
    /// 最近一次带宽采样窗口结束时的单调时钟读数（单位：纳秒）。
    #[cfg(feature = "ktest")]
    pub sample_end_ns: u64,
    /// 累计已启动的后台回写轮次数。
    pub rounds: u64,
    /// 累计已结束的后台回写轮次数。
    pub finished_rounds: u64,
    /// 当前停机代次序号。
    pub stop_sequence: u64,
    /// 最近一轮后台回写启动时的脏 Inode 槽位上限。
    pub round_limit: usize,
    /// 最近一轮后台回写实际扫描的脏 Inode 槽位数。
    pub round_scanned: usize,
    /// 最近一轮后台回写选中的文件数据页数。
    pub round_selected_data: usize,
    /// 最近一轮后台回写选中的卷元数据页数。
    pub round_selected_metadata: usize,
    /// 最近一轮后台回写已接受的请求总字节数（单位：字节）。
    pub round_accepted_bytes: u64,
    /// 最近一轮后台回写实际落盘完成的字节数（单位：字节）。
    pub round_completed_bytes: u64,
    /// 最近一轮结束时本卷是否仍有残留脏页或在途回写页。
    pub later_dirty: bool,
    /// 前台写入者进入限速等待的累计次数。
    pub writer_waits: u64,
    /// 前台写入者从限速等待被唤醒的累计次数。
    pub writer_wakes: u64,
    /// 当前正在限速等待中的前台写入者数量。
    pub waiting_writers: usize,
    /// 回写域当前是否处于已激活状态。
    pub active: bool,
    /// 工作队列中是否挂有尚未启动的本卷回写任务票。
    pub pending: bool,
    /// 后台回写回调当前是否正在运行。
    pub running: bool,
}

/// 全局文件缓存页计数、脏页压力阈值与全局回写带宽快照（仅 `ktest` 诊断与断言使用）。
///
/// 元数据缓存页计入全局 `dirty_pages` / `writeback_pages` 压力账，但不计入可回收文件页 `file_pages`。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct DirtySnapshot {
    /// 全系统当前存活的文件角色缓存页总数。
    #[expect(dead_code, reason = "全局回写压力诊断通过 Debug 输出存活文件页账")]
    pub file_pages: usize,
    /// 全系统当前含脏扇区的缓存页总数。
    #[expect(dead_code, reason = "全局回写压力诊断通过 Debug 输出当前脏页账")]
    pub dirty_pages: usize,
    /// 全系统当前含在途回写扇区的缓存页总数。
    #[expect(dead_code, reason = "全局回写压力诊断通过 Debug 输出当前回写页账")]
    pub writeback_pages: usize,
    /// 用于折算全局脏页上限的可脏页基数（空闲伙伴页 + PCP 缓存页 + 存活文件页）。
    #[expect(
        dead_code,
        reason = "全局回写压力诊断通过 Debug 输出计算脏页阈值的 dirtyable 页数"
    )]
    pub dirtyable_pages: usize,
    /// 全局脏页硬上限（单位：页）。
    pub dirty_limit: usize,
    /// 全局后台异步回写触发阈值（`dirty_limit / 2`，单位：页）。
    pub background_limit: usize,
    /// 全局累计已完成落盘的回写字节数（单位：字节）。
    pub completed_bytes: u64,
    /// 全局实测回写吞吐率（单位：字节/秒）。
    pub bandwidth_bytes_per_sec: u64,
}

/// 在系统启动期共享工作线程池就绪后，初始化全局专用的内存回收级后台回写工作队列 `WB_QUEUE`。
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
    /// 在卷发布前创建并初始化回写域控制结构、预分配工作缓冲区（读写卷）以及绑定至 `WB_QUEUE` 的延迟工作项。
    ///
    /// `DelayedWork` 仅保存 `WritebackDomain` 的裸指针地址作为上下文；实际触发异步回写时由 `schedule` 另行传递 `Arc<Ext4Volume>` 强引用票来保证生命周期。
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

    /// 在不持有任何文件系统业务锁的可调度任务上下文中，捕获当前卷停机边界并阻塞借出预分配的回写工作缓冲区与块层预留通道（`BioLane`）。
    pub(crate) fn borrow_storage(self: &Arc<Self>) -> Result<WbStorageLease, FsError> {
        sched::assert_switchable();
        let source = self.source.as_ref().ok_or(FsError::ReadOnly)?;
        // Step 1: 检查卷是否处于活跃可回写状态，并捕获本次借用绑定的停机代次与元数据代次边界。
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

        // Step 2: 循环尝试取走唯一预分配的 scratch 缓冲区，并在 noio 保护作用域内向 BioReserve 申请回写通道。
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
            // Step 3: 若 scratch 当前已被其他回写任务借走，注册到等待队列后复查条件并睡眠等待归还通知。
            // SAFETY:
            // 1. `self` 由调用方持有的 `&Arc<Self>` 强引用保活，`node` 通过 `pin!` 固定在当前栈帧上；
            // 2. `_registration` 为栈上局部变量（未使用 `mem::forget`），无论从提前返回分支还是循环末尾退出，都会先于 `node` 析构并从 `self.waiters` 链表脱链。
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

    /// 检查给定停机代次 `boundary` 对应的工作缓冲区借用是否已失效（卷发生了停机、熔断或重新激活推进了 `stop_sequence`）。
    pub(crate) fn storage_stopped(&self, boundary: u64) -> bool {
        let state = self.state.lock();
        state.stop_sequence != boundary || !state.active || state.stopping || state.halted
    }

    /// 基于本卷的块层预留池与元数据停机序号构造 [`MetadataIoContext`]，用于隔离不同卷之间的停机状态检查。
    pub(crate) fn metadata_io_context(&self, boundary: u64) -> MetadataIoContext<'_> {
        MetadataIoContext::for_view(
            self.source
                .as_ref()
                .expect("read-only domain has no reserved metadata source"),
            &self.metadata_sequence,
            boundary,
        )
    }

    /// 在自旋锁外清空已用完的工作缓冲区与预留通道，随后将其放回 `scratch` 槽位并唤醒所有等待者。
    fn return_storage(&self, mut storage: Box<writeback::WbStorage>) {
        storage.clear();
        {
            let mut scratch = self.scratch.lock();
            assert!(scratch.is_none(), "writeback scratch returned twice");
            *scratch = Some(storage);
        }
        self.waiters.wake_all();
    }

    /// 在停机流程中阻塞等待已借出的 `WbStorage` 实例真正归还至 `scratch` 槽位，确保所有借用该缓冲区的回写路径已彻底退出。
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
            // SAFETY:
            // 1. 调用方在执行 `stop_sync` 期间持有 `&self` 强保活所属回写域，`node` 通过 `pin!` 固定在当前栈帧上；
            // 2. `_registration` 在每次循环迭代结束或函数提前返回时自动析构脱链，先于栈上 `node` 释放。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if self.scratch.lock().is_some() {
                return;
            }
            waiter.park();
        }
    }

    /// 在块设备驱动接纳回写请求时原子累加已提交字节数（可安全在 `Bio` 或设备队列自旋锁内调用，不获取本域 `state` 自旋锁）。
    pub(crate) fn note_submitted(&self, bytes: usize) {
        self.submitted_bytes
            .fetch_add(bytes as u64, Ordering::Relaxed);
    }

    /// 捕获当前回写域的完成序号与字节计数边界，并在域处于正常活跃状态时立即触发一轮零延迟后台回写（供内存回收路径调用）。
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

    /// 在可调度上下文中阻塞等待回写状态推进越过给定的 `boundary`（出现新的设备完成、后台轮次结束、回写错误或停机事件）。
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
            // SAFETY:
            // 1. 调用方持有的 `&self` 保证 `self.waiters` 在本函数返回前始终存活，`node` 通过 `pin!` 固定在当前栈帧上；
            // 2. `_registration` 在每次迭代末尾或提前 `return` 时正常执行 `Drop` 从队列脱链，早于 `node` 生命周期结束。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if let Some(progress) = self.progress_after(boundary) {
                return progress;
            }
            waiter.park();
        }
    }

    /// 检查当前回写域状态相较于快照 `boundary` 是否已发生完成推进、轮次结束、错误或停机；若尚未越过边界则返回 `None`。
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

    /// 在卷成功绑定至活跃 Inode 域后开放后台回写服务，重置熔断标志与带宽采样起点。
    ///
    /// 调用前上一生命周期的停机流程必须已彻底排空所有回调与借出资源。
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

    /// 在页面或 Inode 首次变脏时通知回写域按需调度后台回写（无堆分配、不睡眠）。
    ///
    /// 若后台回调正在运行则仅置位 `notified` 交由轮末复查；若已有排队工作项则保持不变；否则按当前脏页压力决定立即调度（`Duration::ZERO`）或延迟到期调度（`DIRTY_EXPIRE`）。
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

    /// 在 `state` 自旋锁保护下克隆一份 `Arc<Ext4Volume>` 强引用票并将延迟任务排入 `WB_QUEUE`；若工作项已在队列中则在锁外释放多余的卷引用。
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
            // SAFETY:
            // 1. `ticket` 刚刚由上方 `Arc::into_raw(volume.clone())` 导出；
            // 2. 因 `work().queue(delay)` 返回 `false`（工作项已在队列中），该指针未存入 `state.pending` 也未暴露给其他线程，此处在锁外唯一调用 `Arc::from_raw` 还原并释放该强引用。
            drop(unsafe { Arc::from_raw(ticket) });
        }
    }

    /// 同步停止本卷的后台回写服务并排空所有异步回调与借出资源。
    ///
    /// 在缓存关闭且本卷最后一个活跃 Inode 槽位退休后调用；调用方必须处于可睡眠任务上下文，严禁持有缓存索引锁或在后台工作回调自身内部调用。
    pub(crate) fn stop_sync(&self) {
        sched::assert_switchable();
        let _stop = self.stop.lock();
        // Step 1: 置位停机标志、递增数据与元数据停机代次，并从全局活跃带宽总和中扣除本卷份额。
        {
            let mut state = self.state.lock();
            state.active = false;
            state.stopping = true;
            state.stop_sequence += 1;
            self.metadata_sequence.fetch_add(1, Ordering::Release);
            ACTIVE_BANDWIDTH.fetch_sub(state.contribution, Ordering::Relaxed);
            state.contribution = 0;
        }
        // Step 2: 唤醒本域所有等待者与块层预留池等待者，使阻塞在借用或限速上的线程尽快感知停机退出。
        self.waiters.wake_all();
        if let Some(source) = &self.source {
            source.notify_available();
        }

        // Step 3: 同步取消工作队列中的延迟任务（若回调正在运行则等待其彻底执行完毕退出），并等待借出的 WbStorage 归还。
        self.work().cancel_sync();
        self.wait_storage_returned();

        // Step 4: 清理停机状态并回收尚未进入 run_writeback 回调的排队卷票（若有）。
        let ticket = {
            let mut state = self.state.lock();
            assert!(!state.running, "writeback callback survived cancellation");
            state.stopping = false;
            state.notified = false;
            core::mem::take(&mut state.pending)
        };
        if ticket != 0 {
            // SAFETY:
            // 1. 上方 `self.work().cancel_sync()` 已确保延迟定时器与工作队列回调彻底终止，且 `assert!(!state.running)` 确认无并发回调运行；
            // 2. 非零 `ticket` 是此前 `schedule` 通过 `Arc::into_raw` 存入 `state.pending` 但尚未被 `run_writeback` 取走的唯一强引用票，此处通过 `core::mem::take` 独占取出并还原释放。
            drop(unsafe { Arc::from_raw(ticket as *const Ext4Volume) });
        }
    }

    /// 在 `PageDesc` 用途自旋锁内根据页面 `dirty` 和 `writeback` 位图的零/非零边沿转换，原子增减本卷的脏页与在途回写页计数。
    pub(crate) fn account_folio(&self, old_dirty: u8, new_dirty: u8, old_wb: u8, new_wb: u8) {
        account_mask(&self.dirty_pages, old_dirty, new_dirty);
        account_mask(&self.writeback_pages, old_wb, new_wb);
    }

    /// 在页面开始回写时（已释放 `PageDesc` 用途锁）记录本卷带宽采样的初始时间起点。
    pub(crate) fn note_writeback_start(&self) {
        self.state.lock().completion.start(time::now());
    }

    /// 在一批回写请求完成时更新本卷完成字节计数、错误序号、带宽样本及全局活跃带宽贡献值，并唤醒等待队列上的限速写入者与回收等待者。
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

    /// 在回写准备或块层提交过程遇到错误时递增错误序号并唤醒等待者，避免限速写入者在无法推进回写时长时间阻塞（具体错误仍通过原 `Result` 或映射的 `errseq` 向上传递）。
    fn note_error(&self) {
        self.state.lock().completion.error_sequence += 1;
        self.waiters.wake_all();
    }

    /// 在后台回写轮次标记 `ToWrite` 页面后立即记录本轮选中的文件数据页或元数据页数量，并断言不超过单轮预算上限。
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

    /// 在后台回写凭证完成 DMA 时累加本轮实际接受与成功落盘的字节数。
    pub(crate) fn note_round_io(&self, accepted: usize, completed: usize) {
        let mut state = self.state.lock();
        state.round.accepted_bytes += accepted as u64;
        state.round.completed_bytes += completed as u64;
    }

    /// 在页面失效（invalidate/truncate）或脏位清除后唤醒本域等待队列，使正在限速睡眠的前台写入者及时重新检查本卷压力。
    pub(crate) fn wake_writers(&self) {
        self.waiters.wake_all();
    }

    /// 读取本卷回写域当前的只读统计与轮次状态快照（仅 `ktest` 使用）。
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

    /// 设置或清除每轮后台回写结束时的测试观察钩子，并在释放自旋锁后再析构旧钩子闭包（仅 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_round_hook(&self, hook: Option<RoundHook>) {
        let previous = core::mem::replace(&mut *self.round_hook.lock(), hook);
        drop(previous);
    }

    /// 返回当前按实测带宽或脏页占比计算出的本卷脏页硬上限与后台唤醒阈值（仅 `ktest` 使用）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_limits(&self) -> (usize, usize) {
        self.limits()
    }

    /// 计算本卷当前的脏页硬上限与后台异步回写触发阈值 `(limit, background_limit)`。
    ///
    /// 当本卷与全局均存在非零实测带宽样本时，按本卷带宽占 `ACTIVE_BANDWIDTH` 的比例切分全局上限；尚未建立带宽样本时，按本卷脏/回写页数占全局总压力页数的比例分配。
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

    /// 返回本卷当前的脏页压力页数（脏缓存页数与在途回写页数之和）。
    fn pressure(&self) -> usize {
        self.dirty_pages.load(Ordering::Relaxed) + self.writeback_pages.load(Ordering::Relaxed)
    }

    /// 返回构造时预初始化的固定延迟工作项引用 `Pin<&DelayedWork>`。
    fn work(&self) -> Pin<&DelayedWork> {
        self.work
            .get()
            .expect("writeback work is not prepared")
            .as_ref()
    }

    /// 执行单轮有界后台回写：最多扫描一轮脏 Inode 槽位表，并回写最多 `DATA_BUDGET`（48）张文件数据页与 `METADATA_BUDGET`（16）张元数据页。
    ///
    /// 轮末若卷仍存在脏页/在途回写页或运行期间收到了新通知，则自动调度下一轮（若本轮遇到错误则退避延迟 `DIRTY_EXPIRE`）。
    fn run_round(self: &Arc<Self>, volume: &Arc<Ext4Volume>) {
        // Step 1: 初始化本轮统计，并从上一轮留下的 next_slot 轮询起点开始扫描脏 Inode 表。
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
        // Step 2: 借用本卷预分配的 WbStorage，按槽位轮询回写各脏 Inode 的数据页（累计不超过 DATA_BUDGET）。
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

                // Step 3: 若文件数据回写未发生停机未确认异常，继续按 METADATA_BUDGET 预算回写卷元数据脏页。
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

        // Step 4: 更新轮次结束状态并唤醒本域等待者；若仍有残留脏页或新通知且未熔断停机，则排入下一轮后台回写任务。
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
    /// 在最后一个 `Arc<WritebackDomain>` 引用释放时断言所有排队卷票、运行中回调、活跃带宽份额及借出的工作缓冲区均已彻底归还。
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

/// 后台回写延迟工作项入口回调：从 `state.pending` 取出唯一卷强引用票还原为 `Arc<Ext4Volume>`，驱动该卷执行一轮有界回写。
fn run_writeback(work: Pin<&DelayedWork>) {
    let pointer = work.context() as *const WritebackDomain;
    // SAFETY:
    // 1. `schedule` 在将本工作项排入队列前已克隆一份 `Arc<Ext4Volume>` 存入 `state.pending`，而 `Ext4Volume` 强持有 `Arc<WritebackDomain>`；
    // 2. 在本回调下方获取 `domain.state` 锁并取走 `pending` 之前，`stop_sync` 的 `cancel_sync` 会阻塞等待本回调完成，因此 `pointer` 指向的 `WritebackDomain` 在此处必然存活有效。
    let domain = unsafe { &*pointer };
    let ticket = {
        let mut state = domain.state.lock();
        assert!(!state.running, "writeback callback reentered");
        let ticket = core::mem::take(&mut state.pending);
        assert_ne!(ticket, 0, "writeback callback lost volume ticket");
        state.running = true;
        ticket
    };
    // SAFETY:
    // 1. `ticket` 刚刚在 `domain.state` 自旋锁内通过 `core::mem::take(&mut state.pending)` 唯一取出（上方 `assert_ne!(ticket, 0)` 确认有效）；
    // 2. `stop_sync` 仅在 `cancel_sync` 等待本回调彻底退出后才检查残留的 `pending`，不会与此处重复还原同一份 `Arc::into_raw` 指针。
    let volume = unsafe { Arc::from_raw(ticket as *const Ext4Volume) };
    volume.writeback_domain().run_round(&volume);
}

/// 在物理页被安装为文件缓存页（`added = true`）或最终回收释放底层页（`added = false`）时，配对增减全局存活文件页计数 `FILE_PAGES`。
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

/// 在 `PageDesc` 用途自旋锁内根据物理页 `dirty` 与 `writeback` 扇区位图的零/非零边沿转换，更新全局脏页与回写页计数器。
pub(crate) fn account_folio(old_dirty: u8, new_dirty: u8, old_wb: u8, new_wb: u8) {
    account_mask(&DIRTY_PAGES, old_dirty, new_dirty);
    account_mask(&WRITEBACK_PAGES, old_wb, new_wb);
}

/// 当扇区位图从 `0` 变为非零时将页级计数器加 `1`，从非零清零时减 `1`（下溢时直接触发断言 panic）。
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

/// 在任意物理页开始回写时记录全局完成采样器的初始单调时钟起点。
pub(crate) fn note_writeback_start() {
    GLOBAL_COMPLETION.lock().start(time::now());
}

/// 在已提交给块设备的回写范围结束时，累加全局已完成字节数并按采样窗口更新全局实测回写带宽。
pub(crate) fn note_completion(accepted: usize, completed: usize) {
    if accepted != 0 {
        GLOBAL_COMPLETION
            .lock()
            .complete(accepted, completed, time::now());
    }
}

/// 计算当前系统的可脏页基数与全局脏页硬上限 `(dirtyable_pages, dirty_limit)`。
///
/// 由于当前页面回收仅针对文件缓存页，可脏页基数取伙伴空闲页、每核 PCP 缓存页与存活文件页之和，再按 `DIRTY_RATIO`（10%）折算并钳制到 `[MIN_DIRTY_PAGES, MAX_DIRTY_PAGES]`。
fn dirty_limit() -> (usize, usize) {
    let memory = mm::stats();
    let dirtyable =
        memory.buddy_free_pages + memory.pcp_cached_pages + FILE_PAGES.load(Ordering::Relaxed);
    (
        dirtyable,
        (dirtyable * DIRTY_RATIO / PERCENT).clamp(MIN_DIRTY_PAGES, MAX_DIRTY_PAGES),
    )
}

/// 读取当前全局文件页计数、脏页压力阈值及全局完成带宽快照（仅 `ktest` 诊断使用）。
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

/// 前台磁盘文件写入后的脏页平衡与流控限速入口。
///
/// 调用方必须处于可调度任务上下文，且已释放所有 Inode 锁、目录游标、失效锁及 `FolioContent` 锁。
/// 当本卷脏/回写页压力超过后台阈值时触发异步回写；超过硬上限时按超出量与实测带宽进入有界睡眠等待（单次最长 `200ms`）。
/// 本函数不向写入者返回回写 I/O 错误（已写入页缓存的数据保持有效，异步落盘错误由文件描述符在 `fsync` 时通过 `errseq` 报告）。
pub(crate) fn balance_dirty_pages(inode: &InodeRef) {
    sched::assert_switchable();
    let volume = inode
        .disk_volume()
        .expect("dirty balancing requires a disk inode");
    let domain = volume.writeback_domain();
    // Step 1: 计算本卷脏页阈值与当前压力；未达后台阈值直接返回，超过后台阈值则唤醒后台回写，未超硬上限亦直接返回。
    let (limit, background) = domain.limits();
    let pressure = domain.pressure();
    if pressure <= background {
        return;
    }
    domain.notify_dirty(volume);
    if pressure <= limit {
        return;
    }

    // Step 2: 压力超过硬上限，读取当前完成序号与实测带宽，计算本次限速等待的目标完成字节数或最长暂停截止时间（最多 200ms）。
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

    // Step 3: 注册到本卷等待队列并在锁下复查退出条件；若仍超限则挂载高精度超时定时器进入睡眠等待。
    let mut waiting = false;
    loop {
        let waiter = Waiter::new();
        let mut node = pin!(WaitNode::new(waiter.waker()));
        // SAFETY:
        // 1. 调用方持有的 `inode` 强引用保活所属 `Ext4Volume` 及其 `WritebackDomain`（含 `domain.waiters`），`node` 通过 `pin!` 固定在当前栈帧上；
        // 2. `registration` 在每次循环末尾或提前 `break` 分支中均先于 `node` 和 `waiter` 显式 `drop` 脱链。
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
            // SAFETY:
            // 1. `timer` 与 `waker` 均通过 `pin!` 固定在当前循环迭代栈帧上；
            // 2. 下方从 `waiter.park()` 唤醒后必定先执行 `timer.as_ref().cancel_sync()` 等待正在运行的定时器中断回调彻底退出，随后才退出作用域销毁 `timer` 与 `waker`。
            unsafe { timer.as_ref().arm_borrowed(deadline) }
                .expect("fresh dirty balance timer rejected");
        }
        waiter.park();
        domain.state.lock().writer_wakes += 1;
        drop(registration);
        // `Timer::cancel_sync` 在等待远端核中断回调结束时可能内部自建等待，因此必须先显式 drop 当前任务的 `waiter` 以结束本轮活跃等待状态。
        drop(waiter);
        if deadline.is_some() {
            timer.as_ref().cancel_sync();
        }
    }
    if waiting {
        domain.state.lock().waiting_writers -= 1;
    }
}

/// 前台脏页限速定时器的中断到期回调：通过上下文指针唤醒对应的限速等待者重新检查压力与超时条件。
fn wake_writer(timer: Pin<&Timer>) {
    // SAFETY:
    // 1. `timer.context()` 保存的是 `balance_dirty_pages` 栈上已固定 `Waker` 的裸指针；
    // 2. `balance_dirty_pages` 在 `waker` 离开作用域前必定调用 `timer.as_ref().cancel_sync()` 等待本中断回调彻底执行完毕退出，且 `Waker::notify` 仅需共享引用即可并发安全调用。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}
