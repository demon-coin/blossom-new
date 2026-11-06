//! 块设备实例（[`Disk`]）、请求队列状态机与固定请求槽（[`SlotStorage`]）管理。
//!
//! ## 模块职责与架构定位
//! 本模块是块设备层（`crate::block`）的核心调度与资源管理枢纽：
//! - **上游调用者**：由 `crate::block::submit`（请求拆分与合并）、`crate::block::bio`（`Bio` 生命周期）、
//!   `crate::block::stop`（停机与 `FsClaim` 仲裁）以及系统启动/驱动初始化代码调用。
//! - **下游依赖**：通过 `crate::block::backend::Backend` 统一对接 VirtIO-Blk、RAM 盘、SD/MMC 与 AHCI
//!   控制器；利用 `crate::irq` 处理设备完成中断，并通过 `crate::workqueue` 异步执行后端命令或可睡眠完成回调。
//!
//! ## 核心设计与关键状态机
//!
//! ### 1. 预分配固定请求槽与零热路径内存分配
//! 在 `Disk::prepare*` 阶段，根据设备队列深度（`limits.tag_count`）一次性预分配全部 [`SlotStorage`]、
//! 散聚表（SG `Vec<PayloadRange>`）容量、进度 `Work` 与每槽位完成 `Work`。进入 I/O 提交与中断完成热路径后，
//! 不再进行任何堆内存分配，从而彻底消除写回（Writeback）与中断上下文中的内存分配死锁风险。
//!
//! ### 2. 请求槽状态机（[`SlotState`]）与独占访问权转移
//! 每个请求槽的 `SlotResources` 存放在 `UnsafeCell` 中，其独占访问权由 `QueueState::slot_states[tag]`
//! 的状态转换严格隔离，无需为每个槽位单独加锁：
//! ```text
//!   ┌──────┐  reserve_slot()   ┌───────────┐  try_publish()   ┌───────────┐
//!   │ Free │ ────────────────► │ Preparing │ ───────────────► │ Submitted │
//!   └──────┘                   └───────────┘                  └─────┬─────┘
//!      ▲                             │                              │
//!      │       (未发布即 Drop)        │         ┌────────────────────┼────────────────────┐
//!      ├─────────────────────────────┘         │ (VirtIO IRQ 消费   │ (Work 后端认领)    │ (复位后强制失败)
//!      │                                       │  used ring 认领)   ▼                    │
//!      │                                       │             ┌───────────┐               │
//!      │                                       │             │ Executing │               │
//!      │                                       │             └─────┬─────┘               │
//!      │      finish_slot() 清理资源后归还     ▼                   ▼                     ▼
//!      └──────────────────────────────────── ┌─────────────────────────────────────────────┐
//!                                            │                  Finishing                  │
//!                                            └─────────────────────────────────────────────┘
//! ```
//!
//! ### 3. 队列生命周期状态机（[`QueuePhase`]）
//! ```text
//!   Prepared ──(start)──► Live ──(stop)──► Draining ──(排空后)──► Resetting ──(确认复位)──► DmaStopped ──► Stopped
//!      │                   │                  │                       │
//!      └──(start 失败)─────┴──(硬件故障)──────┴──► Faulting ──────────┴──(2s 超时)──► Quarantined
//! ```
//! - `Live`：正常接纳新的根 `Bio` 请求（`admit_from`）及子请求切片。
//! - `Draining`：拒绝新的根 `Bio` 准入，但允许已准入的请求生成者（`generators > 0`）继续分配槽位并发起子请求，直至排空。
//! - `Faulting` / `Resetting` / `DmaStopped` / `Stopped` / `Quarantined`：拒绝一切新请求或子请求提交。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. `Disk::control`（`SleepMutex<()>`）：最外层睡眠锁，仅在普通任务调用 `start()` 和 `stop()` 时获取。
//! 2. `Disk::queue`（`SpinLock<QueueState>`）：保护队列阶段、槽位状态数组、活跃计数与侵入式链表。
//!    - 持锁期间严禁睡眠、严禁执行大块内存拷贝（如 RAM/SDMMC 数据复制）或耗时的 `Bio` 完成回调。
//!    - 若需同时操作 `BioInner` 内部状态锁，顺序为：`Disk::queue` ➔ `BioInner` 状态锁（如 `register_child` / `accept_child`）。
//! 3. `Disk::irq`（`SpinLock<Option<IrqRegistration>>`）与 `Disk::metadata_cache`（`SpinLock<Weak<MetadataPages>>`）：
//!    均为叶子自旋锁，不在持有它们时再去获取 `Disk::queue` 锁。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::pin::{pin, Pin};
use core::ptr;

use crate::block::backend::{virtio_limits, Backend, SlotControl, WorkTarget};
use virtio_drivers::Hal;

use crate::block::bio::{BioArc, BioInner};
use crate::block::{
    AccessMode, Bio, BlkOperation, BlockError, DiskIdentity, FsClaim, QueueLimits, MAX_BATCH,
    SECTOR_BYTES,
};
use crate::cache::MetadataPages;
use crate::drivers::virtio::VirtioDevice;
use crate::irq::{self, IrqRegistration};
use crate::mm::dma::{DmaDomain, KernelHal, PayloadRange};
use crate::sync::{SleepMutex, SpinLock};
use crate::wait::{WaitNode, WaitQueue, Waiter};
use crate::workqueue::{Work, WorkQueue};

/// 单次中断或单轮进度 `Work` 连续处理的请求槽上限（8 个），超过后让出并调度下一轮 `progress_work`，防止长时间关中断或饿死其他任务。
pub(super) const IRQ_BUDGET: usize = 8;

/// 固定在堆上（Pin）且引用计数的块设备句柄类型。
pub type DiskRef = Pin<Arc<Disk>>;

/// 块设备请求队列的生命周期阶段。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum QueuePhase {
    /// 固定槽位、后端数据结构与 `Work` 已在内存中准备就绪，但尚未向硬件发布队列地址或使能中断。
    Prepared,
    /// 设备已启动并正常工作，允许准入新的根 `Bio` 请求并提交子请求。
    Live,
    /// 正在停机排空：拒绝新的根 `Bio` 准入，但允许已准入的请求生成者继续拆分并提交子请求直至完成。
    Draining,
    /// 检测到设备故障或启动失败：已停止接收任何新子请求，等待认领并执行硬件复位。
    Faulting,
    /// 唯一执行流已认领复位流程：正在等待门铃通知和 `Executing` 槽位退出，并向硬件发出复位命令轮询确认。
    Resetting,
    /// 硬件已确认复位完成且已执行完 DMA 屏障：设备不再访问任何内存，可安全回收在途 `Submitted` 槽位与物理页。
    DmaStopped,
    /// 中断已同步注销、所有 `Work` 已关闭、后端控制器已释放，设备彻底停机。
    Stopped,
    /// 硬件复位在超时时间内未能确认完成：队列进入永久隔离状态，保留设备与在途 DMA 页面所有权以防 DMA 踩内存。
    Quarantined,
}

/// 单个固定请求槽（Tag）的状态机阶段。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum SlotState {
    /// 槽位空闲，可被 `try_reserve_slot` 分配给请求准备者。
    Free,
    /// 已被某个任务通过 `PreparingSlot` 守卫独占认领，正在锁外填充散聚表（SG）与子请求片段（`RequestPart`）。
    Preparing,
    /// 已通过 `try_publish` 提交给后端（写入 VirtIO avail ring 或排入 `Work` 后端队列），硬件或后端工作队列拥有访问权。
    Submitted,
    /// 仅用于 `Work` 驱动的后端（RAM / SDMMC / AHCI）：`progress_work` 已从后端队列取出该槽位，正在锁外执行数据传输。
    Executing,
    /// 硬件或后端已完成传输并交回软件认领者，正在执行或等待 `completion_work` 执行上层 `Bio` 完成回调与资源清理。
    Finishing,
}

/// 由 `Disk::queue` 自旋锁保护的队列运行期状态与统计信息。
pub(super) struct QueueState {
    /// 底层块设备后端实例（VirtIO / RAM / SDMMC / AHCI）；在 `Stopped` 阶段被 `take()` 释放。
    pub(super) device: Option<Backend>,
    /// 当前队列所处的生命周期阶段。
    pub(super) phase: QueuePhase,
    /// 记录导致队列进入故障或隔离状态的首个错误原因。
    pub(super) failure: Option<BlockError>,
    /// 当前已准入但尚未彻底结束父 `Bio` 完成回调的请求生成者总数。
    pub(super) generators: usize,
    /// 当前已准入的写操作请求生成者数量。
    pub(super) writer_generators: usize,
    /// 当前非 `Free` 状态的请求槽总数（涵盖 `Preparing`、`Submitted`、`Executing`、`Finishing`）。
    pub(super) active_slots: usize,
    /// 当前处于 `Submitted` 或 `Finishing` 状态的写操作请求槽数量。
    pub(super) writing_slots: usize,
    /// 当前正在锁外执行硬件门铃通知（Doorbell MMIO 写入）的并发执行流计数；复位前必须等待其归零。
    pub(super) notify_active: usize,
    /// 标记是否有已提交至队列但尚未敲响硬件门铃（或尚未唤醒 `progress_work`）的请求。
    pub(super) pending_notify: bool,
    /// 当前活跃的文件系统扇区占用（`FsClaim`）计数；非零时禁止 `stop()` 停机。
    pub(super) mounts: usize,
    /// 每个请求槽（按 `tag` 索引）的当前状态。
    pub(super) slot_states: Vec<SlotState>,
    /// 指向活跃文件系统占用（`FsClaimInner`）侵入式单向链表头部的指针。
    pub(super) claim_head: *const crate::block::stop::FsClaimInner,
    /// 指向已准入的裸设备写请求（无 `FsClaim` 的 `BioInner`）侵入式单向链表头部的指针。
    pub(super) raw_writers: *const BioInner,
    /// 每个请求槽对应的裸设备写入扇区区间 `Some((start_sector, end_sector))`，用于与 `claim_fs` 进行范围冲突检测。
    pub(super) slot_writes: Vec<Option<(u64, u64)>>,
    /// 内核测试（`ktest`）统计计数器快照。
    #[cfg(feature = "ktest")]
    counters: TestSnapshot,
    /// 内核测试开关：是否暂停消费完成队列（used ring）。
    #[cfg(feature = "ktest")]
    test_paused: bool,
    /// 内核测试中用于观测和控制停机/复位流程的状态。
    #[cfg(feature = "ktest")]
    pub(super) ktest_stop: StopTestState,
    /// 内核测试记录的最近一次单 Folio 请求的首个散聚段 DMA 映射信息：`(物理页基址, 设备 DMA 地址, 页内偏移, 字节长度)`。
    #[cfg(feature = "ktest")]
    ktest_folio_dma: Option<(usize, u64, usize, usize)>,
    /// 内核测试注册的一次性完成错误注入配置：`(匹配请求类型, 注入错误码, 可选的成功读取字节前缀)`。
    #[cfg(feature = "ktest")]
    ktest_completion_error: Option<(TestCompletionKind, BlockError, Option<usize>)>,
    /// 内核测试中指定要拦截并暂缓软件完成回调的目标文件页物理地址（不影响硬件 used ring 的正常消费）。
    #[cfg(feature = "ktest")]
    ktest_hold_file_writeback_page: Option<usize>,
    /// 内核测试中已从硬件完成队列取出、但因匹配 `ktest_hold_file_writeback_page` 而暂缓调用 `finish_slot` 的槽位号列表。
    #[cfg(feature = "ktest")]
    ktest_held_file_writeback_slots: [Option<u16>; MAX_BATCH],
}

impl QueueState {
    /// 判断当前队列阶段是否允许已准入的请求生成者继续分配槽位、提交子请求及处理完成事件（仅 `Live` 与 `Draining` 阶段返回 `true`）。
    fn accepts_children(&self) -> bool {
        matches!(self.phase, QueuePhase::Live | QueuePhase::Draining)
    }

    /// 返回队列在非活动或故障阶段拒绝请求时应报告的错误码（若存在故障则返回故障错误，否则返回 `QueueStopped`）。
    fn stopped_error(&self) -> BlockError {
        self.failure.unwrap_or(BlockError::QueueStopped)
    }
}

/// 单个硬件请求槽中所承载的一个上层 `Bio` 子片段（支持将最多 `MAX_BATCH` 个相邻 `Bio` 片段合并到同一请求槽）。
pub(super) struct RequestPart {
    /// 所属上层 `Bio` 的内部强引用。
    pub(super) bio: BioArc,
    /// 该片段在所属 `Bio` 字节范围中的起始偏移量（单位：字节）。
    pub(super) offset: usize,
    /// 该片段的字节长度（单位：字节）。
    pub(super) length: usize,
    /// 独立持有的文件系统占用凭证副本，确保在槽位资源清理与队列计数归还前，文件系统占用不会被提前释放（即使父 `Bio` 已提前唤醒等待者）。
    _fs_claim: Option<FsClaim>,
}

/// 单个请求槽在一次 I/O 周期内独占使用的可变资源集合。
struct SlotResources {
    /// 预分配容量的散聚表（Scatter-Gather List），记录本次请求的物理/DMA 地址段。
    sg: Vec<PayloadRange>,
    /// 本次请求合并承载的 `Bio` 子片段数组（最多 `MAX_BATCH` 个）。
    parts: [Option<RequestPart>; MAX_BATCH],
    /// `parts` 数组中当前有效的子片段数量。
    part_count: usize,
    /// 本次请求的块操作类型（`Read`、`Write` 或 `Flush`）。
    operation: BlkOperation,
    /// 本次请求在磁盘上的起始扇区号（以 512 字节 `SECTOR_BYTES` 为单位；`Flush` 请求为 0）。
    sector: u64,
    /// 本次请求传输的总字节数。
    bytes: usize,
    /// 由复位清理（`fail_submitted_after_reset`）或测试错误注入强制覆盖的错误码。
    forced_error: Option<BlockError>,
    /// 后端返回的归一化 I/O 完成结果（已完成字节数与可选错误）。
    completion: Option<crate::block::IoResult>,
    /// 内核测试注入的读操作部分成功前缀：`Some((已成功读取字节数, 后续部分注入的错误码))`。
    #[cfg(feature = "ktest")]
    ktest_read_prefix: Option<(usize, BlockError)>,
    /// 标记当前槽位是否已通过 `PreparingSlot::configure` 完成参数校验与协议头写入。
    configured: bool,
}

/// 固定在 `Disk::slots` 数组中的单个请求槽存储单元。
pub(super) struct SlotStorage {
    /// 槽位的可变请求资源，其独占访问权由 `QueueState::slot_states` 状态机在 `Preparing`、`Executing` 与 `Finishing` 阶段之间安全流转。
    resources: UnsafeCell<SlotResources>,
    /// 预分配的槽位专属完成工作任务（当槽位包含需要任务上下文的 `Bio` 完成回调时调度）。
    pub(super) completion_work: Option<Pin<Arc<Work>>>,
    /// 槽位在硬件与队列中的唯一编号（`0..limits.tag_count`）。
    pub(super) tag: u16,
    /// 指回所属 `Disk` 实例的裸指针（在 `Disk::prepare_backend` 固定 `Disk` 地址后写入，供 `completion_work` 回调使用）。
    pub(super) disk: *const Disk,
    /// 后端与槽位绑定的控制块句柄（如 VirtIO 请求头与状态字节的访问入口）。
    control: SlotControl,
}

// SAFETY:
// 1. `resources` 仅在持有独占 `PreparingSlot` 守卫、处于唯一 `Executing` 工作任务、或赢得 `Submitted -> Finishing` 转换的唯一完成执行流中被访问；
// 2. `disk` 裸指针在 `Disk` 固定（Pin）后一次性写入且不再改变，在 `Disk::stop` 同步关闭所有 `Work` 与注销 IRQ 前 `Disk` 始终存活。
unsafe impl Send for SlotStorage {}
// SAFETY:
// 同一槽位在不同执行流之间的可变访问期由 `Disk::queue` 自旋锁内的 `SlotState` 状态转换及释放-获取内存序严格隔开，不会产生并发数据竞争。
unsafe impl Sync for SlotStorage {}

/// 块设备核心对象，统一管理请求队列、固定请求槽数组、中断注册、工作队列任务及 DMA 域。
///
/// 该结构体包含自引用裸指针（如 `SlotStorage::disk` 及注册给 IRQ/`Work` 的上下文指针），因此通过 [`DiskRef`]（`Pin<Arc<Disk>>`）固定在堆内存中，不可移动。
pub struct Disk {
    /// 保护队列状态机、槽位分配、在途计数及占用链表的自旋锁。
    pub(super) queue: SpinLock<QueueState>,
    /// 串行化外部任务调用 `start()` 与 `stop()` 的睡眠互斥锁。
    pub(super) control: SleepMutex<()>,
    /// 保存设备中断注册句柄的自旋锁（在 `start()` 时注册，在 `stop()` 时取出并同步注销）。
    pub(super) irq: SpinLock<Option<IrqRegistration>>,
    /// 预分配的设备级进度工作任务（用于处理 `Work` 驱动后端的 I/O 执行、超出中断预算后的接续消费以及异步故障复位）。
    pub(super) progress_work: Option<Pin<Arc<Work>>>,
    /// 在准备阶段一次性分配且容量固定的请求槽数组（按下标 `tag` 索引）。
    pub(super) slots: Vec<SlotStorage>,
    /// 当请求槽耗尽或硬件描述符环（avail ring）暂满时，供提交者等待空闲资源的等待队列。
    pub(super) available: WaitQueue,
    /// 供停机/复位流程等待在途请求生成者、槽位与门铃通知全部归零（Quiescent）的等待队列。
    pub(super) quiescent: WaitQueue,
    /// 设备在准备阶段协商确定的队列与 DMA 硬件限制参数。
    pub(super) limits: QueueLimits,
    /// 设备的全局唯一身份标识。
    pub(super) identity: DiskIdentity,
    /// 设备所属的硬件 DMA 映射域（RAM 盘等纯 CPU 拷贝后端为 `None`）。
    pub(super) domain: Option<DmaDomain>,
    /// 指向本设备共享元数据页缓存（`MetadataPages`）的弱引用，允许多个相同范围的挂载视图复用同一份元数据缓存。
    metadata_cache: SpinLock<Weak<MetadataPages>>,
    /// 标记 `Disk` 为 `!Unpin`，防止从 `Pin<Arc<Disk>>` 中被移动导致自引用裸指针失效。
    pub(super) _pinned: PhantomPinned,
}

// SAFETY:
// 1. 队列状态与链表指针由 `queue` 自旋锁保护，后端硬件命令由唯一的 `progress_work` 或后端内部锁串行化；
// 2. 注册给中断与 `Work` 的 `Disk` 裸指针在 `Disk::stop` 同步注销 IRQ 并关闭所有 `Work` 之前始终保持有效。
unsafe impl Send for Disk {}
// SAFETY:
// 公开接口不直接暴露内部 `UnsafeCell` 或裸指针；跨 CPU 任务的请求准备与完成回调仅访问各自通过 `queue` 锁独占认领的 `SlotStorage`。
unsafe impl Sync for Disk {}
/// 内核测试（`ktest`）用于观测队列运行行为的事件统计快照。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct TestSnapshot {
    /// 已成功提交到后端设备的请求槽总次数。
    pub submitted: usize,
    /// 已完成并释放归还的请求槽总次数。
    pub completed: usize,
    /// 已提交的文件数据页回写（Data Writeback）请求数。
    pub data_submitted: usize,
    /// 已提交的共享元数据页回写（Metadata Writeback）请求数。
    pub metadata_submitted: usize,
    /// 已提交的缓存刷出（`Flush`）请求数。
    pub flush_submitted: usize,
    /// 已提交的文件数据页读入（File Read）请求数。
    pub file_read_submitted: usize,
    /// 运行期间同时处于在途（Inflight）状态的请求槽峰值数量。
    pub peak_inflight: usize,
    /// 实际敲响硬件门铃（Doorbell Notify）的总次数。
    pub notifications: usize,
    /// 提交者因槽位或硬件描述符不足而在 `available` 等待队列上睡眠（Park）的次数。
    pub resource_parks: usize,
    /// 上层 `Bio` 因硬件限额而被拆分为多个子请求片段的额外拆分次数。
    pub splits: usize,
    /// 相邻 `Bio` 子片段被合并到同一个硬件请求槽的合并次数。
    pub merges: usize,
    /// 因单次消费达到 `IRQ_BUDGET` 上限而交由 `progress_work` 接续处理的次数。
    pub budget_continuations: usize,
    /// 当前正处于在途状态的请求槽数量。
    pub inflight: usize,
    /// 中断处理函数读取并确认本设备中断状态寄存器（ISR）的总次数。
    #[cfg(feature = "ktest")]
    pub irq_calls: usize,
    /// 中断状态寄存器包含队列完成位（QUEUE interrupt）的次数。
    #[cfg(feature = "ktest")]
    pub irq_queue: usize,
    /// 中断状态寄存器为空（共享中断线上非本设备触发）的次数。
    #[cfg(feature = "ktest")]
    pub irq_empty: usize,
}

/// 内核测试中用于匹配单次完成错误注入目标的请求类别。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TestCompletionKind {
    /// 普通文件数据页的回写请求。
    Data,
    /// 文件系统元数据页的回写请求。
    Metadata,
    /// 块设备 `Flush` 同步请求。
    Flush,
    /// 普通文件数据页的读入请求。
    FileRead,
}

#[cfg(feature = "ktest")]
impl TestCompletionKind {
    /// 检查已完成传输的请求槽资源 `resources` 是否匹配当前错误注入目标类别（裸设备直接请求不参与文件数据/元数据匹配）。
    fn matches(self, resources: &SlotResources) -> bool {
        match self {
            Self::Flush => resources.operation == BlkOperation::Flush,
            Self::FileRead => {
                resources.operation == BlkOperation::Read
                    && resources
                        .parts
                        .iter()
                        .flatten()
                        .any(|part| part.bio.ktest_file_read(part.offset) == Some(true))
            }
            Self::Data | Self::Metadata => {
                resources.operation == BlkOperation::Write
                    && resources.parts.iter().flatten().any(|part| {
                        part.bio.ktest_writeback_metadata(part.offset)
                            == Some(self == Self::Metadata)
                    })
            }
        }
    }
}

/// 内核测试中用于控制和观测停机与硬件复位流程的辅助状态。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct StopTestState {
    /// 是否暂停消费硬件完成队列（用于构造复位前存在未消费完成项的并发场景）。
    pub(super) pause_used: bool,
    /// 是否拦截硬件复位完成确认（用于模拟硬件复位超时并验证 `Quarantined` 隔离行为）。
    pub(super) withhold_confirmation: bool,
    /// 已成功发布到后端的请求槽累计数量。
    pub(super) published: usize,
    /// 已执行完 `finish_slot` 的请求槽累计数量。
    pub(super) finished: usize,
    /// 已向后端硬件发起 `request_reset()` 的次数。
    pub(super) reset_requests: usize,
    /// 已确认硬件 `reset_complete()` 并完成 DMA 屏障的次数。
    pub(super) reset_confirmations: usize,
}

impl Disk {
    /// 在普通任务上下文中根据 VirtIO 设备硬件特性预分配所有固定请求槽与 `Work`，构造处于 `Prepared` 阶段的 [`DiskRef`]。
    ///
    /// 此阶段仅完成内存结构准备，尚未向硬件写入队列物理地址或注册中断，需随后调用 [`Disk::start`] 启动。
    pub fn prepare(device: VirtioDevice, workqueue: &WorkQueue) -> Result<DiskRef, BlockError> {
        let limits = virtio_limits(device.info())?;
        Self::prepare_with_limits(
            device,
            workqueue,
            limits,
            #[cfg(feature = "ktest")]
            None,
        )
    }

    /// 内核测试辅助构造函数：在不超出硬件真实能力的前提下，使用收紧的队列限额参数构造 [`DiskRef`]，用于测试请求拆分与边界对齐。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_prepare_with_limits(
        device: VirtioDevice,
        workqueue: &WorkQueue,
        max_bytes: usize,
        max_segments: usize,
        max_segment_bytes: u32,
        dma_boundary_mask: u64,
    ) -> Result<DiskRef, BlockError> {
        let mut limits = virtio_limits(device.info())?;
        if max_bytes < limits.logical_block_size as usize
            || limits.max_bytes < max_bytes
            || max_bytes % limits.logical_block_size as usize != 0
            || max_segments == 0
            || limits.max_segments < max_segments
            || max_segment_bytes == 0
            || limits.max_segment_bytes < max_segment_bytes
            || (dma_boundary_mask != u64::MAX
                && !dma_boundary_mask.wrapping_add(1).is_power_of_two())
        {
            return Err(BlockError::InvalidRange);
        }

        limits.max_bytes = max_bytes;
        limits.max_segments = max_segments;
        limits.max_segment_bytes = max_segment_bytes;
        limits.dma_boundary_mask = dma_boundary_mask;
        Self::prepare_with_limits(
            device,
            workqueue,
            limits,
            #[cfg(feature = "ktest")]
            None,
        )
    }

    /// 内核测试辅助构造函数：构造 VirtIO 磁盘并通过输出参数 `observation` 记录散聚表预分配快照。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_prepare_observed(
        device: VirtioDevice,
        workqueue: &WorkQueue,
        observation: &mut Option<(u16, crate::drivers::virtio::TestPrepareSnapshot)>,
    ) -> Result<DiskRef, BlockError> {
        assert!(observation.is_none(), "stale SG preparation observation");
        let limits = virtio_limits(device.info())?;
        Self::prepare_with_limits(device, workqueue, limits, Some(observation))
    }

    /// 使用指定的队列限额 `limits` 初始化 VirtIO 后端并构造固定请求槽表。
    fn prepare_with_limits(
        device: VirtioDevice,
        workqueue: &WorkQueue,
        limits: QueueLimits,
        #[cfg(feature = "ktest")] observation: Option<
            &mut Option<(u16, crate::drivers::virtio::TestPrepareSnapshot)>,
        >,
    ) -> Result<DiskRef, BlockError> {
        assert!(
            !device.requires_reset(),
            "preparing an already published disk"
        );
        let backend = Backend::virtio(
            device,
            limits,
            #[cfg(feature = "ktest")]
            observation,
        )?;
        Self::prepare_backend(backend, workqueue, limits)
    }

    /// 基于一段独占的连续物理内存区域 `region` 构造内存盘（RAM Disk）实例。
    ///
    /// # Safety
    /// 1. `region` 必须指向完整、有效且在内核恒等/线性映射区可访问的物理 RAM（例如启动期保留的 initrd 或调用方独占持有的物理页）；
    /// 2. 该物理内存区域的所有者必须保证其存活至本 `Disk` 完成 `stop()` 且所有关联的 `Work`/CPU 拷贝任务彻底退出；
    /// 3. 在本 `Disk` 处于活跃状态期间，调用方、用户态进程或其他外设均不得并发修改或映射该内存区域，避免数据竞争。
    pub unsafe fn prepare_ram(
        region: crate::boot::Region,
        workqueue: &WorkQueue,
    ) -> Result<DiskRef, BlockError> {
        let limits = Backend::ram_limits(region)?;
        Self::prepare_backend(Backend::ram(region)?, workqueue, limits)
    }

    /// 使用平台解析出的 SD/MMC 控制器资源初始化 SD 卡块设备实例；若卡槽未检测到介质则返回 `Ok(None)`。
    ///
    /// # Safety
    /// 1. `resources` 必须来自唯一的平台硬件资源描述，且控制器 MMIO 及其关联的时钟/复位/引脚复用寄存器已完成永久有效的内核映射并由本驱动独占；
    /// 2. 调用方必须保持返回的 [`DiskRef`] 存活至所有中断、后端命令、`Work` 任务及 `stop()` 流程彻底执行完毕。
    pub(crate) unsafe fn prepare_sdmmc(
        resources: &crate::boot::MmcResources,
        workqueue: &WorkQueue,
    ) -> Result<Option<DiskRef>, BlockError> {
        // SAFETY: 调用方已满足 `prepare_sdmmc` 的 `# Safety` 前置条件（寄存器映射有效且独占、生命周期受控）。
        let Some((backend, limits)) = unsafe { Backend::sdmmc(resources) }? else {
            return Ok(None);
        };
        Self::prepare_backend(backend, workqueue, limits).map(Some)
    }

    /// 使用平台解析出的 AHCI 控制器与端口资源初始化 SATA 块设备实例；若端口未连接存储介质则返回 `Ok(None)`。
    ///
    /// # Safety
    /// 1. `resources` 中的 ABAR 寄存器、中断号与 DMA 窗口必须来自唯一的平台硬件资源描述，固件已退出接管且 MMIO 强序映射永久有效；
    /// 2. 调用方独占该 AHCI 端口，并保证返回的 [`DiskRef`] 存活至所有 DMA 传输、中断、`Work` 任务及 `stop()` 流程彻底执行完毕。
    pub(crate) unsafe fn prepare_ahci(
        resources: &crate::boot::AhciResources,
        workqueue: &WorkQueue,
    ) -> Result<Option<DiskRef>, BlockError> {
        // SAFETY: 调用方已满足 `prepare_ahci` 的 `# Safety` 前置条件（ABAR/IRQ/DMA 资源有效且独占、生命周期受控）。
        let Some((backend, limits)) = unsafe { Backend::ahci(resources) }? else {
            return Ok(None);
        };
        Self::prepare_backend(backend, workqueue, limits).map(Some)
    }

    /// 为给定的块设备后端 `backend` 预分配全部固定请求槽、散聚表容量与工作队列任务（`Work`），返回固定在堆上的 [`DiskRef`]。
    ///
    /// 所有可能因内存不足而失败的堆分配均在此阶段完成，I/O 运行期不再进行动态扩容。
    fn prepare_backend(
        backend: Backend,
        workqueue: &WorkQueue,
        limits: QueueLimits,
    ) -> Result<DiskRef, BlockError> {
        let identity = backend.identity();
        let domain = backend.domain();
        let mut slots = Vec::new();
        let mut slot_states = Vec::new();
        let mut slot_writes = Vec::new();
        // Step 1: 按照队列深度 `limits.tag_count` 与单请求最大段数 `limits.max_segments` 预分配槽位数组与散聚表容量。
        slots
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        slot_states
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        slot_writes
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        for tag in 0..limits.tag_count {
            let mut sg = Vec::new();
            sg.try_reserve_exact(limits.max_segments)
                .map_err(|_| BlockError::NoMemory)?;
            slots.push(SlotStorage {
                resources: UnsafeCell::new(SlotResources {
                    sg,
                    parts: core::array::from_fn(|_| None),
                    part_count: 0,
                    operation: BlkOperation::Read,
                    sector: 0,
                    bytes: 0,
                    forced_error: None,
                    completion: None,
                    #[cfg(feature = "ktest")]
                    ktest_read_prefix: None,
                    configured: false,
                }),
                completion_work: None,
                tag,
                disk: ptr::null(),
                control: backend.slot_control(tag),
            });
            slot_states.push(SlotState::Free);
            slot_writes.push(None);
        }

        // Step 2: 分配 `Arc<Disk>` 并将队列初始阶段设为 `QueuePhase::Prepared`。
        let disk = Arc::try_new(Self {
            queue: SpinLock::new(QueueState {
                device: Some(backend),
                phase: QueuePhase::Prepared,
                failure: None,
                generators: 0,
                writer_generators: 0,
                active_slots: 0,
                writing_slots: 0,
                notify_active: 0,
                pending_notify: false,
                mounts: 0,
                slot_states,
                claim_head: ptr::null(),
                raw_writers: ptr::null(),
                slot_writes,
                #[cfg(feature = "ktest")]
                counters: TestSnapshot::default(),
                #[cfg(feature = "ktest")]
                test_paused: false,
                #[cfg(feature = "ktest")]
                ktest_stop: StopTestState::default(),
                #[cfg(feature = "ktest")]
                ktest_folio_dma: None,
                #[cfg(feature = "ktest")]
                ktest_completion_error: None,
                #[cfg(feature = "ktest")]
                ktest_hold_file_writeback_page: None,
                #[cfg(feature = "ktest")]
                ktest_held_file_writeback_slots: [None; MAX_BATCH],
            }),
            control: SleepMutex::new(()),
            irq: SpinLock::new(None),
            progress_work: None,
            slots,
            available: WaitQueue::new(),
            quiescent: WaitQueue::new(),
            limits,
            identity,
            domain,
            metadata_cache: SpinLock::new(Weak::new()),
            _pinned: PhantomPinned,
        })
        .map_err(|_| BlockError::NoMemory)?;

        // Step 3: 将 `Arc<Disk>` 转换为 `Pin<Arc<Disk>>`，并在对象地址固定后初始化自引用指针与预留的 `Work` 实例。
        let raw = Arc::into_raw(disk);
        // SAFETY:
        // 1. `raw` 来自上一行刚创建的唯一 `Arc<Disk>`，此时尚未向外部发布任何克隆引用，也未注册 IRQ 或入队 `Work`；
        // 2. 立即通过 `Arc::from_raw(raw)` 恢复所有权并封装为 `Pin`，若后续步骤失败，`disk` 仍会按 `Prepared` 状态正常析构。
        let disk = unsafe { Pin::new_unchecked(Arc::from_raw(raw)) };
        let progress = Work::try_new_reserved(workqueue, 0, progress_callback, raw as usize)
            .map_err(|_| BlockError::NoMemory)?;
        // SAFETY:
        // 1. `disk` 尚未对外发布，当前函数拥有唯一访问权；
        // 2. 修改 `progress_work` 字段不会移动 `Disk` 本身或其内部的 `slots` 数组。
        unsafe { (*raw.cast_mut()).progress_work = Some(progress) };
        for index in 0..usize::from(limits.tag_count) {
            // SAFETY:
            // 1. `slots` 数组已在 Step 1 中完成固定容量填充且绝不再重新分配；
            // 2. `disk` 尚未对外发布，当前构造函数独占访问每个 `SlotStorage` 元素。
            let slot = unsafe { &mut (&mut (*raw.cast_mut()).slots)[index] };
            slot.disk = raw;
            slot.completion_work = Some(
                Work::try_new_reserved(
                    workqueue,
                    0,
                    completion_callback,
                    slot as *const SlotStorage as usize,
                )
                .map_err(|_| BlockError::NoMemory)?,
            );
        }

        Ok(disk)
    }

    /// 启动处于 `Prepared` 阶段的块设备：注册中断处理函数、向硬件发布队列地址并使能设备中断，将队列切换至 `Live` 阶段。
    ///
    /// 若硬件发布失败，队列将转入 `Faulting` 阶段，调用方仍需调用 [`Disk::stop`] 清理已注册的中断与工作任务。
    pub fn start(&self) -> Result<(), BlockError> {
        // Step 1: 获取 `control` 睡眠锁并检查队列是否处于 `Prepared` 阶段，读取后端中断号。
        let _control = self.control.lock();
        let number = {
            let q = self.queue.lock();
            if q.phase != QueuePhase::Prepared {
                return Err(BlockError::NotReady);
            }
            q.device
                .as_ref()
                .expect("prepared disk has no device")
                .interrupt()
        };

        // Step 2: 若设备使用硬件中断，先以禁用（disabled）状态注册中断回调，防止队列未就绪前触发中断。
        if let Some(number) = number {
            // SAFETY:
            // 1. `self` 通过 `DiskRef`（`Pin<Arc<Disk>>`）固定在堆上，地址恒定；
            // 2. 外部所有者在释放 `Disk` 前必须调用 `Disk::stop()`，其中会调用 `registration.unregister_sync()` 等待中断回调彻底退出。
            let registration =
                unsafe { irq::register(number, irq_callback, self as *const Self as *mut ()) }
                    .map_err(|_| BlockError::Busy)?;
            *self.irq.lock() = Some(registration);
        }

        // Step 3: 在 `queue` 自旋锁下向后端硬件发布队列并切换至 `QueuePhase::Live`（失败则转入 `Faulting`）。
        let result = {
            let mut q = self.queue.lock();
            let result = q
                .device
                .as_mut()
                .expect("prepared disk has no device")
                .publish();
            match result {
                Ok(()) => {
                    let backend = q.device.as_mut().expect("published disk has no backend");
                    if backend.uses_work() {
                        backend.set_interrupts(true);
                    }
                    q.phase = QueuePhase::Live;
                    Ok(())
                }
                Err(error) => {
                    q.failure = Some(error);
                    q.phase = QueuePhase::Faulting;
                    Err(error)
                }
            }
        };

        // Step 4: 队列发布成功后，正式使能中断控制器上的该中断线。
        if result.is_ok() {
            if let Some(registration) = self.irq.lock().as_ref() {
                registration.enable();
            }
        }
        result
    }

    /// 返回本块设备的全局唯一身份标识（[`DiskIdentity`]）。
    pub fn identity(&self) -> DiskIdentity {
        self.identity
    }

    /// 返回本块设备在准备阶段协商确定的队列与 DMA 硬件限制参数（[`QueueLimits`]）。
    pub fn limits(&self) -> QueueLimits {
        self.limits
    }

    /// 尝试升级并返回绑定在本磁盘上的共享元数据页缓存（[`MetadataPages`]）；若尚未绑定或已被释放则返回 `None`。
    pub(crate) fn metadata_cache(&self) -> Option<Arc<MetadataPages>> {
        self.metadata_cache.lock().upgrade()
    }

    /// 注册或复用本磁盘的共享元数据页缓存。
    ///
    /// 若当前已有存活的 `MetadataPages` 实例，则直接返回已有实例（忽略调用方传入的 `candidate`）；
    /// 否则将 `candidate` 的弱引用存入 `metadata_cache` 并返回其强引用克隆。旧的失效弱引用在锁外释放。
    pub(crate) fn intern_metadata_cache(
        &self,
        candidate: &Arc<MetadataPages>,
    ) -> Arc<MetadataPages> {
        let retired = {
            let mut slot = self.metadata_cache.lock();
            if let Some(existing) = slot.upgrade() {
                return existing;
            }
            core::mem::replace(&mut *slot, Arc::downgrade(candidate))
        };
        drop(retired);
        Arc::clone(candidate)
    }

    /// 当 `MetadataPages` 析构时调用：仅当槽位中的弱引用仍指向 `identity` 时将其清空并返回，供调用方在锁外释放。
    pub(crate) fn unregister_metadata_cache(
        &self,
        identity: *const MetadataPages,
    ) -> Option<Weak<MetadataPages>> {
        let mut slot = self.metadata_cache.lock();
        if slot.as_ptr() == identity {
            Some(core::mem::take(&mut *slot))
        } else {
            None
        }
    }

    /// 在与 `progress_work` 相同的工作队列上预分配一个一次性收尾 `Work`，供需要在任务上下文执行完成回调的根 `Bio` 使用。
    pub(super) fn prepare_finalizer(
        &self,
        callback: fn(Pin<&Work>),
        context: usize,
    ) -> Result<Pin<Arc<Work>>, BlockError> {
        self.progress_work
            .as_ref()
            .expect("prepared disk has no progress work")
            .try_new_related_reserved(callback, context)
            .map_err(|_| BlockError::NoMemory)
    }

    /// 在根 `Bio` 的拆分生成与父完成回调全部结束后，递减队列的 `generators` 计数（若为裸写请求则同时从 `raw_writers` 链表摘除），并唤醒 `quiescent` 等待队列。
    pub(super) fn finish_generation_tail(&self, bio: &BioInner) {
        {
            let mut q = self.queue.lock();
            assert!(q.generators != 0, "block generator tail ended twice");
            q.generators -= 1;
            if bio.operation() == BlkOperation::Write {
                q.writer_generators -= 1;
                let mut link = &mut q.raw_writers as *mut *const BioInner;
                // SAFETY:
                // 1. 当前持有 `queue` 自旋锁，串行化对 `raw_writers` 链表的所有修改；
                // 2. 链表中每个已准入的 `BioInner` 节点在调用本函数将其摘除前，始终由对应的请求生成者（`ActiveBio` 或收尾 `Work`）保活。
                unsafe {
                    while !(*link).is_null() {
                        if ptr::eq(*link, bio) {
                            *link = *bio.raw_writer_next.get();
                            *bio.raw_writer_next.get() = ptr::null();
                            break;
                        }
                        link = (*(*link)).raw_writer_next.get();
                    }
                }
            }
        }
        self.quiescent.wake_all();
    }

    /// 准入一个不带文件系统占用凭证（裸设备访问）的根 `Bio` 请求（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(super) fn admit<'a>(
        self: &'a DiskRef,
        bio: Bio,
    ) -> Result<ActiveBio<'a>, (Bio, BlockError)> {
        self.admit_from(bio, None)
    }

    /// 校验并准入一个上层根 `Bio` 请求，成功时返回代表活跃请求生成者的 [`ActiveBio`] 守卫。
    ///
    /// # 准入检查（在 `queue` 自旋锁下原子执行）
    /// 1. 队列必须处于 `QueuePhase::Live` 阶段；
    /// 2. `bio` 的目标设备身份必须与本磁盘一致，且若提供了 `claim`（`FsClaim`），其所属磁盘及扇区范围必须完整包含本次请求；
    /// 3. 若为写请求，磁盘与 `claim` 必须具备可写权限；若为无 `claim` 的裸设备写请求，其扇区范围不得与任何已挂载的 `FsClaim` 重叠；
    /// 4. 通过检查后递增 `generators`（及 `writer_generators`），将裸写请求挂入 `raw_writers` 链表，并在锁外完成 DMA 映射同步（`bio.inner.begin()`）。
    pub(super) fn admit_from<'a>(
        self: &'a DiskRef,
        bio: Bio,
        claim: Option<&FsClaim>,
    ) -> Result<ActiveBio<'a>, (Bio, BlockError)> {
        // Step 1: 在 `queue` 自旋锁下校验队列状态、目标范围与写权限，并登记请求生成者计数。
        {
            let mut q = self.queue.lock();
            let error = if !matches!(q.phase, QueuePhase::Live) {
                Some(q.stopped_error())
            } else if bio.inner.target() != self.identity
                || claim.is_some_and(|claim| {
                    !ptr::eq(&**claim.disk(), &**self)
                        || (bio.inner.operation() != BlkOperation::Flush
                            && !claim
                                .view()
                                .contains_bytes(bio.inner.sector() * SECTOR_BYTES, bio.inner.len()))
                })
            {
                Some(BlockError::InvalidRange)
            } else if bio.inner.operation() == BlkOperation::Write
                && (self.limits.readonly
                    || match claim {
                        Some(claim) => claim.mode() != AccessMode::ReadWrite,
                        None => crate::block::stop::mounted_write_conflicts(
                            &q,
                            (
                                bio.inner.sector(),
                                bio.inner.sector()
                                    + (bio.inner.len() / SECTOR_BYTES as usize) as u64,
                            ),
                        ),
                    })
            {
                Some(BlockError::ReadOnly)
            } else {
                None
            };
            if let Some(error) = error {
                return Err((bio, error));
            }

            q.generators += 1;
            if bio.inner.operation() == BlkOperation::Write {
                q.writer_generators += 1;
                if claim.is_none() {
                    // SAFETY:
                    // 1. 当前 `bio` 尚未发布，且当前持有 `queue` 自旋锁；
                    // 2. `bio.inner` 将由后续构造的 `ActiveBio`（或失败回滚时的 `finish_generation_tail`）保活至持同一把锁从 `raw_writers` 摘除。
                    unsafe {
                        *bio.inner.raw_writer_next.get() = q.raw_writers;
                    }
                    q.raw_writers = &*bio.inner;
                }
            }
        }

        // Step 2: 在锁外执行缓冲区状态切换与 DMA 缓存同步；若失败则回滚生成者计数与链表节点。
        if let Err(error) = bio.inner.begin() {
            self.finish_generation_tail(&bio.inner);
            return Err((bio, error));
        }
        if let Some(claim) = claim {
            bio.inner.retain_fs_claim(claim);
        }
        Ok(ActiveBio {
            bio: bio.inner,
            offset: 0,
            disk: self,
            fs_claim: claim.cloned(),
            finished: false,
        })
    }

    /// 为已准入的请求生成者分配一个空闲请求槽（返回 [`PreparingSlot`] 守卫）。
    ///
    /// 若当前无空闲槽位，会先调用 [`Disk::unplug`] 刷出待发送通知以加速在途请求完成，随后在 `available` 等待队列上睡眠等待。
    pub(super) fn reserve_slot(&self) -> Result<PreparingSlot<'_>, BlockError> {
        loop {
            if let Some(slot) = self.try_reserve_slot()? {
                return Ok(slot);
            }
            self.unplug();
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self.available` 在本次方法调用期间始终有效；
            // 2. `node` 固定（Pin）在当前栈帧上，且 `registration` 守卫在本轮循环退出前必定执行析构并从等待队列摘除该节点。
            let registration = unsafe { self.available.register(node.as_mut()) };
            if let Some(slot) = self.try_reserve_slot()? {
                return Ok(slot);
            }
            self.count_park();
            waiter.park();
            drop(registration);
        }
    }

    /// 尝试在 `queue` 自旋锁下查找一个处于 `Free` 状态的槽位并将其切换为 `Preparing` 状态。
    fn try_reserve_slot(&self) -> Result<Option<PreparingSlot<'_>>, BlockError> {
        let mut q = self.queue.lock();
        if !q.accepts_children() {
            return Err(q.stopped_error());
        }
        let Some(index) = q
            .slot_states
            .iter()
            .position(|state| *state == SlotState::Free)
        else {
            return Ok(None);
        };
        q.slot_states[index] = SlotState::Preparing;
        q.active_slots += 1;
        Ok(Some(PreparingSlot {
            disk: self,
            tag: index as u16,
            published: false,
        }))
    }

    /// 刷出待发送的队列通知：若后端使用 `Work` 驱动则调度 `progress_work`；若使用硬件门铃则在 `queue` 锁内递增 `notify_active` 后在锁外执行 MMIO 通知。
    ///
    /// `notify_active` 计数确保在门铃写入及紧随其后的 `dma_barrier()` 彻底完成前，并发的停机/故障复位流程不会提前释放或复位设备。
    pub(super) fn unplug(&self) {
        let notification = {
            let mut q = self.queue.lock();
            if !q.accepts_children() || !q.pending_notify {
                return;
            }
            q.pending_notify = false;
            let backend = q.device.as_ref().expect("live disk has no device");
            if backend.uses_work() {
                drop(q);
                self.queue_progress();
                return;
            }
            let Some(notification) = backend.notification() else {
                return;
            };
            q.notify_active += 1;
            notification
        };
        // SAFETY:
        // 1. 在 `queue` 锁内已递增 `notify_active`，硬件复位流程（`reset_claimed`）必须等待 `notify_active == 0` 后才能复位设备，因此当前设备寄存器映射必定有效；
        // 2. 门铃写入前后均配合 `dma_barrier()`，确保描述符写入对硬件可见且门铃写入在递减 `notify_active` 前完成。
        unsafe { notification.notify::<KernelHal>() };
        KernelHal::dma_barrier();
        {
            let mut q = self.queue.lock();
            q.notify_active -= 1;
            #[cfg(feature = "ktest")]
            {
                q.counters.notifications += 1;
            }
        }
        self.quiescent.wake_all();
    }

    /// 将队列标记为故障状态（`QueuePhase::Faulting`），记录首个故障错误码并唤醒等待队列与 `progress_work` 异步执行硬件复位。
    ///
    /// 若后端并非正在执行同步命令（`Executing`），则立即屏蔽设备中断；否则保留中断使能直至正在执行的后端命令返回或硬件复位完成。
    pub(super) fn mark_fault(&self, error: BlockError) {
        let claimed = {
            let mut q = self.queue.lock();
            if matches!(
                q.phase,
                QueuePhase::Prepared | QueuePhase::Live | QueuePhase::Draining
            ) {
                q.failure = Some(error);
                q.phase = QueuePhase::Faulting;
                q.pending_notify = false;
                let executing = q
                    .slot_states
                    .iter()
                    .any(|state| *state == SlotState::Executing);
                if let Some(device) = q.device.as_mut() {
                    if !device.uses_work() || !executing {
                        device.set_interrupts(false);
                    }
                }
                true
            } else {
                false
            }
        };
        self.available.wake_all();
        self.quiescent.wake_all();
        if claimed {
            self.queue_progress();
        }
    }

    /// 将预分配的设备进度工作任务（`progress_work`）排入工作队列（可在中断或持锁/无锁上下文中安全调用，不进行内存分配）。
    pub(super) fn queue_progress(&self) {
        if let Some(work) = &self.progress_work {
            let _ = work.as_ref().queue();
        }
    }

    /// 设备中断处理核心逻辑：读取并确认硬件中断状态寄存器（ISR），根据中断原因触发故障退役或消费完成队列（`drain_used`）。
    ///
    /// 返回 `true` 表示本次中断确由本设备触发并已处理，返回 `false` 表示共享中断线上非本设备中断。
    fn interrupt(&self) -> bool {
        let (cause, live) = {
            let mut q = self.queue.lock();
            let live = q.accepts_children();
            let Some(device) = q.device.as_mut() else {
                return false;
            };
            let cause = device.acknowledge();
            #[cfg(feature = "ktest")]
            {
                q.counters.irq_calls += 1;
                if cause.completion {
                    q.counters.irq_queue += 1;
                } else if !cause.claimed {
                    q.counters.irq_empty += 1;
                }
            }
            (cause, live)
        };
        if live && cause.fault {
            self.request_retire(BlockError::DeviceFault);
        }
        if live && cause.completion && !cause.fault {
            self.drain_used();
        }
        cause.claimed || cause.fault
    }
    /// 在 `progress_work` 工作任务上下文中，按批次（最多 `IRQ_BUDGET` 个）从 `Work` 驱动后端取出已提交槽位并在锁外执行实际 I/O。
    fn progress_requests(&self) {
        crate::sched::assert_switchable();
        for _ in 0..IRQ_BUDGET {
            // Step 1: 在 `queue` 自旋锁下从后端待处理队列取出一个 `Submitted` 槽位并将其状态切换为 `Executing`。
            let (tag, target) = {
                let mut q = self.queue.lock();
                if !q.accepts_children() {
                    return;
                }
                let backend = q.device.as_mut().expect("live disk has no backend");
                let Some(tag) = backend.next_work_request() else {
                    return;
                };
                let target = backend
                    .work_target()
                    .expect("queued Work request has no execution target");
                assert_eq!(q.slot_states[usize::from(tag)], SlotState::Submitted);
                q.slot_states[usize::from(tag)] = SlotState::Executing;
                (tag, target)
            };
            // Step 2: 在 `queue` 锁外独占访问 `SlotResources`，根据后端类型执行内存拷贝、SD/MMC 弹跳缓冲传输或 AHCI DMA 命令。
            // SAFETY:
            // 1. 上一步在 `queue` 锁下将槽位状态从 `Submitted` 切换为 `Executing`，确保当前唯一的 `progress_work` 独占拥有该槽位；
            // 2. 停机复位流程（`reset_claimed`）会等待所有 `Executing` 槽位退出，中断处理函数仅更新控制器邮箱而不触碰 `SlotResources`；
            // 3. 槽位中的 `parts` 持有上层 `Bio` 引用与 `FsClaim`，保证目标缓冲区与扇区占用在传输期间始终有效。
            let resources = unsafe { &mut *self.slots[usize::from(tag)].resources.get() };
            let native = !matches!(&target, WorkTarget::Ram(_));
            let result = match target {
                WorkTarget::Ram(region) => {
                    let start = region.start + (resources.sector * SECTOR_BYTES) as usize;
                    // SAFETY:
                    // 1. `prepare_ram` 契约保证 `region` 独占且线性映射有效，请求提交前已校验扇区范围不越界且不与 `Bio` 缓冲区重叠；
                    // 2. 单一的 `progress_work` 串行执行所有 RAM 盘拷贝，无并发写冲突。
                    unsafe {
                        copy_cpu_parts(resources, crate::arch::phys_to_virt(start) as *mut u8)
                    }
                }
                WorkTarget::Sdmmc(controller) => {
                    // SAFETY:
                    // 1. `resources.parts` 中的上层 `Bio` 处于 CPU 拷贝访问期且在子请求完成前保持有效；
                    // 2. `bounce` 弹跳缓冲区由 `controller` 内部锁独占保护，闭包仅执行有界内存拷贝，不睡眠、不分配内存也不重入控制器。
                    unsafe {
                        controller.execute(
                            resources.operation,
                            resources.sector,
                            resources.bytes,
                            |bounce| {
                                let result = copy_cpu_parts(resources, bounce);
                                match result.error {
                                    Some(error) => Err(error),
                                    None => Ok(result.completed),
                                }
                            },
                        )
                    }
                }
                WorkTarget::Ahci(controller) => {
                    // SAFETY:
                    // 1. 上层 `Bio` 已按该 AHCI 控制器的 DMA 域完成缓存同步与访问权移交，`resources.sg` 仅包含有效的物理/DMA 地址段；
                    // 2. `Executing` 状态与 `controller` 引用保证在硬件确认命令结束或复位停止前，所有物理页保持存活且 CPU 不并发访问其内容。
                    unsafe {
                        controller.execute(
                            resources.operation,
                            resources.sector,
                            resources.bytes,
                            &resources.sg,
                        )
                    }
                }
            };
            // Step 3: 若后端控制器因硬件复位超时返回 `StopUnconfirmed`，立即将队列转入 `Quarantined` 隔离阶段并保留槽位与物理页不释放。
            if result.error == Some(BlockError::StopUnconfirmed) {
                let mut q = self.queue.lock();
                assert_eq!(q.slot_states[usize::from(tag)], SlotState::Executing);
                q.failure = Some(BlockError::StopUnconfirmed);
                q.phase = QueuePhase::Quarantined;
                q.pending_notify = false;
                drop(q);
                // 硬件未能确认停止 DMA，严禁回收 `parts`、严禁完成上层 `Bio` 或释放物理页。
                self.available.wake_all();
                self.quiescent.wake_all();
                return;
            }
            // Step 4: 记录完成结果，将槽位状态切换为 `Finishing`，并在锁外派发上层完成回调。
            resources.completion = Some(result);
            {
                let mut q = self.queue.lock();
                assert_eq!(q.slot_states[usize::from(tag)], SlotState::Executing);
                q.slot_states[usize::from(tag)] = SlotState::Finishing;
            }
            self.quiescent.wake_all();
            self.available.wake_all();
            if native {
                if let Some(error) = result.error {
                    self.request_retire(error);
                }
            }
            // 此时后端硬件通道已释放，可以安全派发槽位完成处理（即使完成回调中再次发起同盘读请求也不会死锁）。
            self.dispatch_slot(tag);
        }
        self.queue_progress();
    }

    /// 从中断驱动后端（如 VirtIO）的完成环（used ring）中最多消费 `IRQ_BUDGET`（8）个已完成槽位并派发完成回调。
    ///
    /// 消费完毕后重新开启队列中断通知并复查完成环；若仍有未消费项，则调度预分配的 `progress_work` 继续消费。
    fn drain_used(&self) {
        let mut tags = [None; IRQ_BUDGET];
        let mut count = 0;
        let mut fault = None;
        let mut continuation = false;
        // Step 1: 在 `queue` 自旋锁下暂时抑制设备中断，循环弹出最多 `IRQ_BUDGET` 个完成项并将对应槽位切换至 `Finishing` 状态。
        {
            let mut q = self.queue.lock();
            if !q.accepts_children() {
                return;
            }
            if q.device
                .as_ref()
                .expect("live disk has no backend")
                .uses_work()
            {
                return;
            }
            #[cfg(feature = "ktest")]
            if q.test_paused {
                return;
            }
            #[cfg(feature = "ktest")]
            if q.ktest_stop.pause_used {
                return;
            }
            q.device
                .as_mut()
                .expect("live disk has no device")
                .set_interrupts(false);
            while count < IRQ_BUDGET {
                let used = q
                    .device
                    .as_mut()
                    .expect("live disk has no device")
                    .pop_completion();
                match used {
                    Ok(Some((tag, result))) => {
                        if q.slot_states.get(usize::from(tag)) != Some(&SlotState::Submitted) {
                            fault = Some(BlockError::DeviceFault);
                            break;
                        }
                        q.slot_states[usize::from(tag)] = SlotState::Finishing;
                        // SAFETY:
                        // 1. 硬件已将该请求写入 used ring，`pop_completion` 内部已完成 DMA 屏障并回收硬件描述符；
                        // 2. 当前在 `queue` 自旋锁下将状态从 `Submitted` 唯一切换为 `Finishing`，阻止其他执行流复用或并发访问该槽位的 `SlotResources`。
                        let resources =
                            unsafe { &mut *self.slots[usize::from(tag)].resources.get() };
                        resources.completion = Some(result);
                        #[cfg(feature = "ktest")]
                        if let Some((kind, error, prefix)) = q.ktest_completion_error {
                            if kind.matches(resources) {
                                q.ktest_completion_error = None;
                                if let Some(completed) = prefix {
                                    resources.ktest_read_prefix = Some((completed, error));
                                } else {
                                    resources.forced_error = Some(error);
                                }
                            }
                        }
                        #[cfg(feature = "ktest")]
                        {
                            let held_page = q.ktest_hold_file_writeback_page;
                            let hold = held_page.is_some_and(|physical| {
                                resources.operation == BlkOperation::Write
                                    && resources.parts.iter().flatten().any(|part| {
                                        part.bio.ktest_writeback_metadata(part.offset)
                                            == Some(false)
                                            && part
                                                .bio
                                                .ktest_folio_origin(part.offset)
                                                .is_some_and(|(origin, _)| origin == physical)
                                    })
                            });
                            if hold {
                                // 硬件 used ring 已正常消费，仅将该槽位号暂存在 `ktest_held_file_writeback_slots` 中延后调用 `finish_slot`。
                                let slot = q
                                    .ktest_held_file_writeback_slots
                                    .iter_mut()
                                    .find(|slot| slot.is_none())
                                    .expect("held file writeback slots exhausted");
                                *slot = Some(tag);
                            } else {
                                tags[count] = Some(tag);
                            }
                        }
                        #[cfg(not(feature = "ktest"))]
                        {
                            tags[count] = Some(tag);
                        }
                        count += 1;
                    }
                    Ok(None) => break,
                    Err(_) => {
                        fault = Some(BlockError::DeviceFault);
                        break;
                    }
                }
            }
            if fault.is_none() {
                continuation = q
                    .device
                    .as_mut()
                    .expect("live disk has no device")
                    .arm_and_check();
                #[cfg(feature = "ktest")]
                if continuation {
                    q.counters.budget_continuations += 1;
                }
            }
        }

        // Step 2: 在 `queue` 锁外处理设备异常、唤醒等待硬件描述符的提交者，并逐个派发已完成槽位。
        if let Some(error) = fault {
            self.mark_fault(error);
        }
        // 硬件描述符已在 `pop_completion` 中归还，等待描述符空间的 `PreparingSlot::publish` 任务可立即被唤醒。
        if 0 < count {
            self.available.wake_all();
        }
        for tag in tags[..count].iter().flatten() {
            self.dispatch_slot(*tag);
        }
        if continuation {
            self.queue_progress();
        }
    }

    /// 派发处于 `Finishing` 状态的请求槽：若任一子片段需要在可睡眠的任务上下文中完成，则排入该槽位专属的 `completion_work`；否则直接在当前上下文调用 [`Disk::finish_slot`]。
    pub(super) fn dispatch_slot(&self, tag: u16) {
        let slot = &self.slots[usize::from(tag)];
        // SAFETY:
        // 调用方刚刚赢得该槽位向 `SlotState::Finishing` 的唯一状态转换，在 `finish_slot` 将其置回 `Free` 前无任何并发访问。
        let needs_task = unsafe {
            (*slot.resources.get())
                .parts
                .iter()
                .flatten()
                .any(|part| part.bio.needs_task())
        };
        if needs_task {
            slot.completion_work
                .as_ref()
                .expect("slot has no completion work")
                .as_ref()
                .queue()
                .expect("active slot completion work is closed");
        } else {
            self.finish_slot(tag);
        }
    }

    /// 执行 `Finishing` 槽位的最终完成处理：按实际完成的字节前缀逐个通知上层 `Bio` 子片段完成、清空槽位资源、在 `queue` 锁下将槽位状态归还为 `Free`，最后释放 `FsClaim`。
    pub(super) fn finish_slot(&self, tag: u16) {
        // Step 1: 独占读取槽位的完成结果，若检测到设备故障则触发故障状态转换。
        let slot = &self.slots[usize::from(tag)];
        // SAFETY:
        // 本函数仅由赢得 `Finishing` 状态的唯一执行流（或其调度的专属 `completion_work`）调用；在 Step 4 将状态置回 `Free` 前，本槽位不会被重新分配。
        let resources = unsafe { &mut *slot.resources.get() };
        let (completed, error) = completion_result(resources);
        if error == Some(BlockError::DeviceFault) {
            self.mark_fault(BlockError::DeviceFault);
        }

        // Step 2: 取出所有 `RequestPart`，按照已完成字节数 `completed` 依次结算每个子片段的成功进度或错误。
        let writing = resources.operation == BlkOperation::Write;
        let mut remaining = completed;
        let parts = core::mem::replace(&mut resources.parts, [const { None }; MAX_BATCH]);
        for part in parts.iter().flatten() {
            let progress = remaining.min(part.length);
            remaining -= progress;
            let part_error = if progress < part.length || part.length == 0 {
                error
            } else {
                None
            };
            BioInner::finish_child(&part.bio, part.offset, part.length, progress, part_error);
        }

        // Step 3: 重置槽位内的散聚表与元数据，为下一轮复用做准备。
        resources.sg.clear();
        resources.part_count = 0;
        resources.configured = false;
        resources.forced_error = None;
        resources.completion = None;
        #[cfg(feature = "ktest")]
        {
            resources.ktest_read_prefix = None;
        }

        // Step 4: 在 `queue` 自旋锁下将槽位状态置回 `Free` 并递减活跃槽位计数，随后在锁外唤醒等待队列并释放 `parts` 持有的 `FsClaim`。
        {
            let mut q = self.queue.lock();
            assert_eq!(q.slot_states[usize::from(tag)], SlotState::Finishing);
            q.slot_states[usize::from(tag)] = SlotState::Free;
            q.active_slots -= 1;
            if writing {
                q.writing_slots -= 1;
                q.slot_writes[usize::from(tag)] = None;
            }
            #[cfg(feature = "ktest")]
            {
                q.ktest_stop.finished += 1;
            }
            #[cfg(feature = "ktest")]
            {
                q.counters.completed += 1;
                q.counters.inflight -= 1;
            }
        }
        self.available.wake_all();
        self.quiescent.wake_all();
        // 必须在 `active_slots` 归还之后再 drop `parts`，确保 `RequestPart::_fs_claim` 覆盖整个槽位生命周期。
        drop(parts);
    }

    /// 在硬件复位已确认完成且进入 `DmaStopped` 阶段后，将所有仍处于 `Submitted` 状态的在途请求槽强制转入 `Finishing` 并以指定错误 `error` 结束。
    pub(super) fn fail_submitted_after_reset(&self, error: BlockError) {
        for index in 0..self.slots.len() {
            let claimed = {
                let mut q = self.queue.lock();
                assert_eq!(q.phase, QueuePhase::DmaStopped);
                if q.slot_states[index] == SlotState::Submitted {
                    q.slot_states[index] = SlotState::Finishing;
                    // SAFETY:
                    // 1. `q.phase == QueuePhase::DmaStopped` 证明硬件复位已完成且已执行 DMA 屏障，硬件不再访问任何槽位；
                    // 2. 在 `queue` 自旋锁下将状态从 `Submitted` 唯一切换为 `Finishing`，当前执行流取得该槽位 `resources` 的独占访问权。
                    unsafe {
                        (*self.slots[index].resources.get()).forced_error = Some(error);
                    }
                    true
                } else {
                    false
                }
            };
            if claimed {
                self.dispatch_slot(index as u16);
            }
        }
        self.available.wake_all();
        self.quiescent.wake_all();
    }
    /// 返回内核测试统计计数器快照（[`TestSnapshot`]）。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_snapshot(&self) -> TestSnapshot {
        self.queue.lock().counters
    }

    /// 内核测试钩子：为下一次完成的普通文件数据页回写（Data Writeback）请求注入一次软件完成错误 `error`。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_data(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::Data, error, None);
    }

    /// 内核测试钩子：为下一次完成的普通文件数据页读入（File Read）请求注入一次软件完成错误 `error`。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_file_read(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::FileRead, error, None);
    }

    /// 内核测试钩子：在下一次文件数据页读入请求完成时，模拟仅成功传输前 `completed` 字节（须为扇区整数倍），并对剩余部分注入错误 `error`。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_file_read_after(&self, completed: usize, error: BlockError) {
        assert!(0 < completed && completed % SECTOR_BYTES as usize == 0);
        self.test_arm_completion_error(TestCompletionKind::FileRead, error, Some(completed));
    }

    /// 内核测试钩子：为下一次完成的共享元数据页回写（Metadata Writeback）请求注入一次软件完成错误 `error`。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_metadata(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::Metadata, error, None);
    }

    /// 内核测试钩子：为下一次完成的 `Flush` 同步请求注入一次软件完成错误 `error`。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_flush(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::Flush, error, None);
    }

    /// 在 `queue` 自旋锁下配置单次完成错误注入规则 `(kind, error, prefix)`。
    #[cfg(feature = "ktest")]
    fn test_arm_completion_error(
        &self,
        kind: TestCompletionKind,
        error: BlockError,
        prefix: Option<usize>,
    ) {
        assert!(matches!(
            error,
            BlockError::Io | BlockError::ShortIo | BlockError::NoMemory
        ));
        let mut q = self.queue.lock();
        assert!(
            q.accepts_children(),
            "arming completion error on stopped disk"
        );
        assert!(
            q.ktest_completion_error.is_none(),
            "completion error already armed"
        );
        q.ktest_completion_error = Some((kind, error, prefix));
    }

    /// 检查已配置的内核测试完成错误注入是否仍在等待匹配（尚未被任何完成项消费）。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_completion_error_pending(&self) -> bool {
        self.queue.lock().ktest_completion_error.is_some()
    }

    /// 同步等待本磁盘的进度 `Work` 与所有槽位的完成 `Work` 当前排队的任务全部执行完毕。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_flush_completions(&self) {
        if let Some(work) = &self.progress_work {
            work.as_ref().flush();
        }
        for slot in &self.slots {
            if let Some(work) = &slot.completion_work {
                work.as_ref().flush();
            }
        }
    }

    /// 在请求生成者成功提交全部子请求后，累加本次生成的子请求拆分次数 `splits` 与跨 `Bio` 合并次数 `merges`。
    #[cfg(feature = "ktest")]
    pub(super) fn record_generation(&self, splits: usize, merges: usize) {
        let mut q = self.queue.lock();
        q.counters.splits += splits;
        q.counters.merges += merges;
    }

    /// 内核测试钩子：指定当承载物理页 `folio` 的文件数据回写请求从硬件完成环取出后，暂缓执行其 `finish_slot` 软件完成回调（其他无关 I/O 仍正常完成）。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_hold_file_writeback(&self, folio: &crate::mm::FolioRef) {
        let mut q = self.queue.lock();
        assert!(
            q.accepts_children(),
            "holding file writeback on a stopped disk"
        );
        assert!(
            !q.test_paused,
            "file writeback hold cannot combine with queue pause"
        );
        assert!(
            q.ktest_hold_file_writeback_page.is_none()
                && q.ktest_held_file_writeback_slots
                    .iter()
                    .all(Option::is_none),
            "file writeback hold is already active"
        );
        q.ktest_hold_file_writeback_page = Some(folio.phys_addr());
    }

    /// 检查目标物理页 `folio` 的文件数据回写请求是否已被硬件完成并正处于 `Finishing` 暂缓状态。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_file_writeback_is_held(&self, folio: &crate::mm::FolioRef) -> bool {
        let q = self.queue.lock();
        q.ktest_hold_file_writeback_page == Some(folio.phys_addr())
            && q.ktest_held_file_writeback_slots
                .iter()
                .any(Option::is_some)
    }

    /// 内核测试钩子：暂停本磁盘完成环（used ring）的消费并屏蔽队列完成通知中断。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_suspend_completions(&self) {
        let mut q = self.queue.lock();
        assert!(q.accepts_children(), "pausing a stopped disk");
        q.test_paused = true;
        q.device
            .as_mut()
            .expect("live disk has no device")
            .set_interrupts(false);
    }

    /// 内核测试钩子：恢复本磁盘的完成消费，派发所有暂缓的文件回写槽位（`ktest_held_file_writeback_slots`），并在存在待处理完成项时调度 `progress_work`。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_resume_completions(&self) {
        let (pending, held) = {
            let mut q = self.queue.lock();
            q.test_paused = false;
            let held =
                core::mem::replace(&mut q.ktest_held_file_writeback_slots, [None; MAX_BATCH]);
            q.ktest_hold_file_writeback_page = None;
            let pending = if q.accepts_children() {
                q.device
                    .as_mut()
                    .expect("live disk has no device")
                    .arm_and_check()
            } else {
                false
            };
            (pending, held)
        };

        for tag in core::iter::IntoIterator::into_iter(held).flatten() {
            self.dispatch_slot(tag);
        }
        if pending {
            self.queue_progress();
        }
    }

    /// 内核测试钩子：解除 `test_paused` 并直接通过 `progress_work` 驱动完成环消费（由 `drain_used` 在消费结束后重新开启中断通知）。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_resume_via_progress(&self) {
        self.queue.lock().test_paused = false;
        self.queue_progress();
    }

    /// 停机测试钩子：暂停消费完成环（`pause_used = true`）并抑制队列中断，使已完成请求留在硬件 used ring 中以测试停机排空与复位交互。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_hold_used(&self) {
        let mut q = self.queue.lock();
        assert_eq!(q.phase, QueuePhase::Live);
        q.ktest_stop.pause_used = true;
        q.device
            .as_mut()
            .expect("live stop fixture")
            .set_interrupts(false);
    }

    /// 停机测试钩子：解除 `pause_used` 并调度 `progress_work` 继续消费硬件完成环。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_release_used(&self) {
        self.queue.lock().ktest_stop.pause_used = false;
        self.queue_progress();
    }

    /// 返回最近一次成功提交的单 Folio 请求的首个散聚段 DMA 信息：`(物理页基址, 设备 DMA 地址, 页内偏移, 字节长度)`。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_last_folio_dma(&self) -> Option<(usize, u64, usize, usize)> {
        self.queue.lock().ktest_folio_dma
    }

    /// 在任务注册等待节点并复查仍无可用资源、即将睡眠（`park()`）前递增 `resource_parks` 统计计数。
    fn count_park(&self) {
        #[cfg(feature = "ktest")]
        {
            self.queue.lock().counters.resource_parks += 1;
        }
    }
}

impl Drop for Disk {
    /// 析构断言检查：销毁 `Disk` 前必须处于从未启动的 `Prepared` 阶段或已完成显式停机的 `Stopped` 阶段，且中断与所有活跃计数均已清零。
    fn drop(&mut self) {
        let q = self.queue.lock();
        assert!(
            matches!(q.phase, QueuePhase::Prepared | QueuePhase::Stopped),
            "dropping an active disk"
        );
        assert!(self.irq.lock().is_none(), "dropping a registered disk");
        assert_eq!(q.generators, 0);
        assert_eq!(q.active_slots, 0);
        assert_eq!(q.notify_active, 0);
    }
}

/// 代表一个已通过 [`Disk::admit_from`] 准入、正在拆分子请求并提交到硬件槽位的活跃根 `Bio` 守卫。
pub(super) struct ActiveBio<'a> {
    /// 已准入的上层 `Bio` 内部强引用。
    pub(super) bio: BioArc,
    /// 当前已成功拆分并提交的字节偏移进度（单位：字节）。
    pub(super) offset: usize,
    /// 目标块设备引用。
    disk: &'a DiskRef,
    /// 标记是否已调用 `finish_inner` 完成了生成阶段结算。
    finished: bool,
    /// 本次提交携带的文件系统占用凭证副本，保活至生成阶段收尾完成。
    fs_claim: Option<FsClaim>,
}

impl ActiveBio<'_> {
    /// 正常或带错结束当前根 `Bio` 的子请求生成阶段，并在所有子请求及完成回调结束后归还队列 `generators` 计数。
    pub(super) fn finish(mut self, error: Option<(usize, BlockError)>) {
        self.finish_inner(error);
    }

    /// 内部结算逻辑：调用 `BioInner::finish_generation`；若未延迟到异步收尾 `Work`，则立即调用 `finish_generation_tail` 归还生成者计数并释放 `fs_claim`。
    fn finish_inner(&mut self, error: Option<(usize, BlockError)>) {
        if self.finished {
            return;
        }
        let deferred = BioInner::finish_generation(&self.bio, error, Some(self.disk));
        self.finished = true;
        if !deferred {
            self.disk.finish_generation_tail(&self.bio);
        }
        // 若完成回调被推迟到异步 `Work`，`BioInner` 内部已保留一份 `FsClaim`；若已同步完成，则在归还 `generators` 计数后安全释放。
        drop(self.fs_claim.take());
    }
}

impl Drop for ActiveBio<'_> {
    /// 当 `ActiveBio` 未显式调用 `finish` 而提前析构时，刷出已提交的前缀请求并以 `(self.offset, BlockError::QueueStopped)` 结束生成阶段。
    fn drop(&mut self) {
        if !self.finished {
            self.disk.unplug();
        }
        self.finish_inner(Some((self.offset, BlockError::QueueStopped)));
    }
}

/// 代表一个处于 `SlotState::Preparing` 阶段的独占请求槽守卫，用于在 `queue` 锁外填充散聚表、合并 `Bio` 子片段并最终发布到硬件队列。
pub(super) struct PreparingSlot<'a> {
    /// 所属块设备引用。
    disk: &'a Disk,
    /// 当前独占认领的槽位编号（`0..limits.tag_count`）。
    tag: u16,
    /// 标记该槽位是否已通过 `try_publish` 成功切换为 `SlotState::Submitted`（若为 `false`，析构时会自动清理并将槽位归还为 `Free`）。
    published: bool,
}

impl PreparingSlot<'_> {
    /// 返回当前槽位内部 `SlotResources` 的共享引用（生命周期绑定到当前守卫借用）。
    fn resources(&self) -> &SlotResources {
        // SAFETY:
        // `PreparingSlot` 独占拥有处于 `SlotState::Preparing` 的槽位 `self.tag`，在发布前硬件与其他 CPU 均不会访问该槽位的 `resources`。
        unsafe { &*self.disk.slots[usize::from(self.tag)].resources.get() }
    }

    /// 返回当前槽位内部 `SlotResources` 的独占可变引用（生命周期绑定到当前守卫可变借用）。
    fn resources_mut(&mut self) -> &mut SlotResources {
        // SAFETY:
        // `&mut self` 保证了同一任务内无其他借用别名，且 `SlotState::Preparing` 状态排除了硬件与其他任务的并发访问。
        unsafe { &mut *self.disk.slots[usize::from(self.tag)].resources.get() }
    }

    /// 返回当前槽位已填充的散聚段（[`PayloadRange`]）切片。
    pub(super) fn sg(&self) -> &[PayloadRange] {
        &self.resources().sg
    }

    /// 返回当前槽位预分配散聚表 `Vec<PayloadRange>` 的可变引用（容量已在 `Disk::prepare_backend` 中按 `limits.max_segments` 预留，调用方不得触发扩容）。
    pub(super) fn sg_mut(&mut self) -> &mut Vec<PayloadRange> {
        &mut self.resources_mut().sg
    }

    /// 返回当前槽位已添加的 `Bio` 子片段（[`RequestPart`]）数量。
    pub(super) fn part_count(&self) -> usize {
        self.resources().part_count
    }

    /// 向当前请求槽追加一个上层 `Bio` 的子片段 `[offset, offset + length)`（此时尚未向 `BioInner` 登记在途子请求计数，仅在 `try_publish` 时登记）。
    pub(super) fn push_part(
        &mut self,
        bio: BioArc,
        offset: usize,
        length: usize,
    ) -> Result<(), BlockError> {
        let resources = self.resources_mut();
        if resources.part_count == MAX_BATCH
            || bio.len() < offset.checked_add(length).ok_or(BlockError::InvalidRange)?
        {
            return Err(BlockError::InvalidRange);
        }
        let fs_claim = bio.fs_claim();
        resources.parts[resources.part_count] = Some(RequestPart {
            bio,
            offset,
            length,
            _fs_claim: fs_claim,
        });
        resources.part_count += 1;
        Ok(())
    }

    /// 在 `queue` 锁外校验散聚表与子片段总字节数及硬件对齐约束，并初始化槽位的硬件协议请求头（`SlotControl::prepare`）。
    pub(super) fn configure(
        &mut self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
    ) -> Result<(), BlockError> {
        // Step 1: 校验子片段数量、散聚段数量以及散聚段与子片段累加字节数是否严格等于 `bytes` 且符合逻辑块大小对齐。
        let limits = self.disk.limits;
        let resources = self.resources();
        if resources.part_count == 0
            || limits.max_segments < resources.sg.len()
            || limits.max_bytes < bytes
        {
            return Err(BlockError::InvalidRange);
        }
        let sg_bytes = resources
            .sg
            .iter()
            .try_fold(0usize, |total, segment| {
                total.checked_add(segment.length as usize)
            })
            .ok_or(BlockError::InvalidRange)?;
        let part_bytes = resources
            .parts
            .iter()
            .flatten()
            .try_fold(0usize, |total, part| total.checked_add(part.length))
            .ok_or(BlockError::InvalidRange)?;
        if sg_bytes != bytes
            || part_bytes != bytes
            || (operation == BlkOperation::Flush
                && (sector != 0 || bytes != 0 || !resources.sg.is_empty()))
            || (operation != BlkOperation::Flush
                && (bytes == 0 || bytes % limits.logical_block_size as usize != 0))
        {
            return Err(BlockError::InvalidRange);
        }

        // Step 2: 写入后端协议控制头（如 VirtIO 请求头）并标记 `resources.configured = true`。
        let slot = &self.disk.slots[usize::from(self.tag)];
        slot.control.prepare(operation, sector)?;
        let resources = self.resources_mut();
        resources.operation = operation;
        resources.sector = sector;
        resources.bytes = bytes;
        resources.forced_error = None;
        resources.completion = None;
        #[cfg(feature = "ktest")]
        {
            resources.ktest_read_prefix = None;
        }
        resources.configured = true;
        Ok(())
    }

    /// 将已配置就绪的请求槽发布到后端硬件队列。
    ///
    /// 若后端硬件描述符（如 VirtIO indirect/chained descriptors）暂时不足，则保留当前槽位已准备好的全部数据，
    /// 先调用 [`Disk::unplug`] 触发在途请求执行，再在 `available` 等待队列上睡眠等待描述符释放后重试。
    pub(super) fn publish(mut self) -> Result<(), BlockError> {
        if !self.resources().configured {
            return Err(BlockError::InvalidRange);
        }

        loop {
            if self.try_publish()? {
                self.published = true;
                return Ok(());
            }
            self.disk.unplug();
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self.disk.available` 在本次调用期间始终有效；
            // 2. `node` 固定（Pin）在当前栈帧上，且 `registration` 守卫在本轮循环退出前必定执行析构并从等待队列摘除该节点。
            let registration = unsafe { self.disk.available.register(node.as_mut()) };
            if self.try_publish()? {
                self.published = true;
                return Ok(());
            }
            self.disk.count_park();
            waiter.park();
            drop(registration);
        }
    }

    /// 尝试在 `queue` 自旋锁下向后端提交当前槽位：若描述符不足则返回 `Ok(false)`；若成功则将槽位状态切换为 `Submitted` 并置位 `pending_notify`。
    fn try_publish(&mut self) -> Result<bool, BlockError> {
        let disk = self.disk;
        let tag = self.tag;
        // Step 1: 在 `queue` 自旋锁下检查队列阶段与后端描述符余量，向各 `Bio` 登记子请求并提交给后端。
        let result = {
            let mut q = disk.queue.lock();
            let resources = self.resources();
            if !q.accepts_children() {
                return Err(q.stopped_error());
            }
            assert_eq!(q.slot_states[usize::from(tag)], SlotState::Preparing);
            if !q
                .device
                .as_ref()
                .expect("live disk has no device")
                .can_submit(resources.sg.len())
            {
                return Ok(false);
            }
            for part in resources.parts.iter().flatten() {
                part.bio.register_child(part.offset, part.length);
            }
            let result = q.device.as_mut().expect("live disk has no device").submit(
                tag,
                resources.operation,
                resources.bytes,
                &resources.sg,
            );
            if result.is_ok() {
                for part in resources.parts.iter().flatten() {
                    part.bio.accept_child(part.offset, part.length);
                }
                q.slot_states[usize::from(tag)] = SlotState::Submitted;
                q.pending_notify = true;
                if resources.operation == BlkOperation::Write {
                    q.writing_slots += 1;
                    if resources
                        .parts
                        .iter()
                        .flatten()
                        .any(|part| part._fs_claim.is_none())
                    {
                        q.slot_writes[usize::from(tag)] = Some((
                            resources.sector,
                            resources.sector + (resources.bytes / SECTOR_BYTES as usize) as u64,
                        ));
                    }
                }
                #[cfg(feature = "ktest")]
                {
                    q.ktest_stop.published += 1;
                    if let Some((part, segment)) =
                        resources.parts[0].as_ref().zip(resources.sg.first())
                    {
                        if let Some((physical, offset)) = part.bio.ktest_folio_origin(part.offset) {
                            q.ktest_folio_dma = Some((
                                physical,
                                segment.physical as u64,
                                offset,
                                segment.length as usize,
                            ));
                        }
                    }
                }
                #[cfg(feature = "ktest")]
                {
                    q.counters.submitted += 1;
                    if TestCompletionKind::Data.matches(resources) {
                        q.counters.data_submitted += 1;
                    }
                    if TestCompletionKind::Metadata.matches(resources) {
                        q.counters.metadata_submitted += 1;
                    }
                    if TestCompletionKind::Flush.matches(resources) {
                        q.counters.flush_submitted += 1;
                    }
                    if TestCompletionKind::FileRead.matches(resources) {
                        q.counters.file_read_submitted += 1;
                    }
                    q.counters.inflight += 1;
                    q.counters.peak_inflight = q.counters.peak_inflight.max(q.counters.inflight);
                }
            }
            result
        };

        // Step 2: 若后端 `submit` 失败，在锁外回滚所有已登记的 `Bio` 子请求并视需要触发设备故障退役。
        match result {
            Ok(_) => Ok(true),
            Err(error) => {
                let fault = error == BlockError::DeviceFault;
                let resources = self.resources();
                for part in resources.parts.iter().flatten() {
                    BioInner::finish_child(&part.bio, part.offset, part.length, 0, Some(error));
                }
                if fault {
                    disk.request_retire(error);
                }
                disk.unplug();
                Err(error)
            }
        }
    }
}

impl Drop for PreparingSlot<'_> {
    /// 若 `PreparingSlot` 在成功发布前被丢弃（例如准备阶段发生错误退出），清空已填充的散聚表与子片段并将槽位状态归还为 `Free`。
    fn drop(&mut self) {
        if self.published {
            return;
        }
        let disk = self.disk;
        let tag = self.tag;
        let resources = self.resources_mut();
        let parts = core::mem::replace(&mut resources.parts, [const { None }; MAX_BATCH]);
        resources.sg.clear();
        resources.part_count = 0;
        resources.configured = false;

        {
            let mut q = disk.queue.lock();
            assert_eq!(q.slot_states[usize::from(tag)], SlotState::Preparing);
            q.slot_states[usize::from(tag)] = SlotState::Free;
            q.active_slots -= 1;
        }
        disk.available.wake_all();
        disk.quiescent.wake_all();
        drop(parts);
    }
}

/// 从 `Finishing` 槽位的 `resources` 中提取归一化的完成结果 `(已完成字节数, 可选错误)`（优先应用 `forced_error` 与测试注入的短读前缀）。
fn completion_result(resources: &SlotResources) -> (usize, Option<BlockError>) {
    if let Some(error) = resources.forced_error {
        return (0, Some(error));
    }
    let result = resources
        .completion
        .expect("finishing slot has no normalized result");
    #[cfg(feature = "ktest")]
    if result.error.is_none() && resources.operation == BlkOperation::Read {
        if let Some((completed, error)) = resources.ktest_read_prefix {
            assert!(
                completed < resources.bytes,
                "controlled prefix has no failed tail"
            );
            return (completed, Some(error));
        }
    }
    (result.completed, result.error)
}

/// 注册给中断子系统的 C 风格回调入口，将上下文指针还原为 [`Disk`] 引用并调用 [`Disk::interrupt`]。
///
/// # Safety
/// 1. `context` 必须是在 [`Disk::start`] 中通过 `irq::register` 传入的固定 [`Disk`] 实例指针；
/// 2. [`Disk`] 实例在 [`Disk::stop`] 调用 `unregister_sync()` 同步等待本回调彻底返回之前，必须始终保持存活且地址不变。
unsafe fn irq_callback(context: *mut ()) -> bool {
    // SAFETY: 调用方与 `Disk::stop` 中的 `unregister_sync` 契约保证了 `context` 指向的 `Disk` 在本函数返回前始终有效。
    unsafe { &*context.cast::<Disk>() }.interrupt()
}

/// 设备级进度工作任务（`progress_work`）的回调入口：根据当前队列阶段执行异步故障复位、驱动 `Work` 后端请求或接续消费完成环。
fn progress_callback(work: Pin<&Work>) {
    // SAFETY: `work.context()` 在 `Disk::prepare_backend` 中固定为 `Disk` 地址，且 `Disk::stop` 在释放 `Disk` 前必定先调用 `progress_work.close_sync()` 等待本回调退出。
    let disk = unsafe { &*(work.context() as *const Disk) };
    let (phase, uses_work) = {
        let q = disk.queue.lock();
        (q.phase, q.device.as_ref().is_some_and(Backend::uses_work))
    };
    if matches!(
        phase,
        QueuePhase::Faulting | QueuePhase::Resetting | QueuePhase::DmaStopped
    ) {
        let _ = disk.fault_reset();
    } else if matches!(phase, QueuePhase::Live | QueuePhase::Draining) {
        if uses_work {
            disk.progress_requests();
        } else {
            disk.drain_used();
        }
    }
}

/// 槽位专属完成工作任务（`completion_work`）的回调入口：在可睡眠的任务上下文中执行 [`Disk::finish_slot`]。
fn completion_callback(work: Pin<&Work>) {
    // SAFETY: `work.context()` 指向固定容量 `Disk::slots` 数组中的 `SlotStorage` 元素，`Disk::stop` 在释放 `Disk` 前必定先调用每个 `completion_work.close_sync()` 等待本回调退出。
    let slot = unsafe { &*(work.context() as *const SlotStorage) };
    let tag = slot.tag;
    // SAFETY: `slot.disk` 在 `Disk::prepare_backend` 中写定为所属 `Disk` 的固定地址，在 `completion_work` 关闭前始终有效。
    let disk = unsafe { &*slot.disk };
    disk.finish_slot(tag);
}

/// 将处于 `Executing` 状态的请求槽中所有 `RequestPart` 与连续虚拟地址缓冲区 `pointer` 之间按顺序执行 CPU 字节拷贝（用于 RAM 盘或 SD/MMC 弹跳缓冲区）。
///
/// 若某个子片段拷贝失败，则保留此前已成功拷贝的连续字节数 `copied` 并立即返回错误。
///
/// # Safety
/// 1. `pointer` 必须指向长度至少为 `resources.bytes` 的有效、可读写连续内存区域，且与所有 `resources.parts` 中的 `Bio` 缓冲区均不重叠；
/// 2. `resources` 所属槽位必须处于唯一的 `SlotState::Executing` 阶段，所有子片段的 `Bio` 在本函数返回前保持存活且处于合法的 CPU 访问期。
unsafe fn copy_cpu_parts(resources: &SlotResources, pointer: *mut u8) -> crate::block::IoResult {
    let mut copied = 0;
    for part in resources.parts.iter().flatten() {
        if part.length == 0 {
            continue;
        }
        // SAFETY:
        // 1. `PreparingSlot::configure` 已验证所有 `part.length` 之和等于 `resources.bytes`，因此 `copied + part.length <= resources.bytes`，`pointer.add(copied)` 不会越界；
        // 2. 调用方保证 `pointer` 连续区域独占有效且与 `part.bio` 缓冲区不重叠，当前槽位由唯一的 `Executing` 工作任务独占访问。
        if let Err(error) = unsafe {
            part.bio
                .copy_cpu(part.offset, part.length, pointer.add(copied))
        } {
            return crate::block::IoResult {
                completed: copied,
                error: Some(error),
            };
        }
        copied += part.length;
    }
    crate::block::IoResult {
        completed: copied,
        error: None,
    }
}
