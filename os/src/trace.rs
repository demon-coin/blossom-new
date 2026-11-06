//! 内核低开销每 CPU 事件追踪子系统（Per-CPU Event Trace）。
//!
//! ## 模块职责与架构定位
//! 本模块提供默认关闭、按 CPU 隔离的定长环形事件采样机制，用于诊断调度切换、等待恢复、网络批处理与协议状态迁移的纳秒级时序：
//! - **事件生产者**：调度器（`sched`）、VirtIO 网卡驱动（`drivers::virtio_net`）及 TCP 协议栈（`net::tcp`）在关键路径调用 [`record`] 记录纯标量事件。
//! - **控制与快照消费者**：`/dev/ktrace` 设备文件（`fs::ktrace`）、比赛执行钩子（`contest_hooks`）与内核自测通过 [`start`]、[`stop`] 和 [`snapshot`] 串行控制采集生命周期并导出 ASCII 文本快照。
//!
//! ## 核心设计与采集状态机
//! 为消除多核并发记录时的跨核缓存行争用，每个逻辑 CPU 独立拥有一个容量为 `CAPACITY`（128 条）的环形缓冲区；全局采集状态在 `CONTROL` 睡眠锁串行保护下按以下状态机流转：
//!
//! ```text
//!          start() 清空各核旧环并置 ENABLED = true
//!    ┌───────────────────────────────────────────────────┐
//!    │                                                   ▼
//!  [ 已停止 Stopped (ENABLED = false) ]        [ 采集中 Active (ENABLED = true) ]
//!    ▲  - record() 快速路径直接返回               - record() 关中断 -> 复查 ENABLED -> 写入本核 RINGS[cpu]
//!    │  - snapshot() 允许导出文本快照             - start() / snapshot() 返回 FsError::Busy
//!    │  - 重复 stop() 幂等返回 Ok(())             │
//!    │                                           │ stop():
//!    └───────────────────────────────────────────┘   1. ENABLED.store(false, Release)
//!       所有在线核执行完 freeze_cpu 跨核屏障         2. 对每个在线核同步调用 cpu::call(..., freeze_cpu)
//! ```
//!
//! **停稳屏障原理**：`record` 在本地关中断（`IrqSaveGuard`）临界区内二次检查 `ENABLED` 并完成入环；而 `stop` 先将 `ENABLED` 置为 `false`，再向每个在线 CPU 发送同步跨核函数调用（CSD IPI）。目标核只有在退出原先的关中断区域后才能响应 IPI 并执行 `freeze_cpu`，因此当所有 `cpu::call` 返回时，全系统所有在途的 `record` 调用必定已彻底执行完毕，此后 [`snapshot`] 与下一次 [`start`] 可安全访问静止的每核环。
//!
//! ## 锁获取顺序与调用契约
//! - **锁层级顺序**：`CONTROL`（`SleepMutex`，仅普通可睡眠任务获取） ➔ `RINGS[cpu]`（`SpinLock`，内部自动关中断）。
//! - **跨核屏障无锁约束**：`stop` 在持有 `CONTROL` 期间调用 `cpu::call` 等待远端 CPU 响应 IPI 时，**绝不持有任何 `RINGS[cpu]` 自旋锁**，保持本地中断开启以避免跨核互等死锁。
//! - **`record` 禁止调用的上下文**：NMI、`panic` 路径、时间维护（timekeeper）写锁临界区、`trace` 模块自身内部以及 `cpu::call` 的跨核中断回调（如 `freeze_cpu`），防止重入本核 `RINGS[cpu]` 自旋锁或等待被自身打断的时间写者。

use alloc::vec::Vec;
use core::fmt::{self, Write};
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::boot::MAX_CPUS;
use crate::fs::FsError;
use crate::sync::{IrqSaveGuard, SleepMutex, SpinLock};
use crate::task::{self, TaskId};
use crate::{cpu, time};

/// 单个逻辑 CPU 事件追踪环的最大槽位容量（超出后按模循环覆盖本核最老记录）。
const CAPACITY: usize = 128;

/// 单条 `record ...` 文本行（含 7 个十进制最大值字段、关键字与换行符）预留的字节容量上限（单位：字节）。
const RECORD_TEXT_CAPACITY: usize = 192;
/// 单个 CPU 分区头 `cpu id=... next_seq=... overwritten=...\n` 预留的字节容量上限（单位：字节）。
const CPU_TEXT_CAPACITY: usize = 80;
/// 快照首行全局头 `ktrace version=1 unit=ns cpus=... capacity=...\n` 预留的字节容量上限（单位：字节）。
const HEADER_TEXT_CAPACITY: usize = 64;

/// 全局追踪采集开关标志（`true` 表示允许各核提交新事件，写操作由 `CONTROL` 串行化）。
static ENABLED: AtomicBool = AtomicBool::new(false);
/// 串行化 `start`、`stop` 与 `snapshot` 控制路径的睡眠互斥锁。
static CONTROL: SleepMutex<()> = SleepMutex::new(());
/// 按逻辑 CPU 编号索引的每核事件环形缓冲区数组。
static RINGS: [SpinLock<Ring>; MAX_CPUS] = [const { SpinLock::new(Ring::new()) }; MAX_CPUS];

/// 内核追踪事件的固定类型编码。
///
/// 以稳定的 `u8` 编号直接输出到快照文本的 `kind=` 字段，避免运行期依赖动态字符串表。
#[derive(Clone, Copy)]
#[repr(u8)]
pub(crate) enum Kind {
    /// 调度器在当前 CPU 上切换并投入运行目标任务（`object` 为任务 ID）。
    TaskDispatch = 1,
    /// 等待中的任务被唤醒并在调度后恢复执行（`object` 为任务 ID，`value` 为等待轮次 `WaitId`）。
    WaitResume = 2,
    /// 网卡驱动单次批处理接收完成的数据包数量（`object` 为设备标识，`value` 为收包数）。
    NetRxBatch = 3,
    /// 网卡驱动单次批处理回收完成的发送包数量（`object` 为设备标识，`value` 为完成包数）。
    NetTxCompleteBatch = 4,
    /// TCP 端点发生协议状态迁移（`object` 为端点 ID，`value` 为编码后的状态转换值）。
    TcpState = 5,
    /// TCP 协议定时器到期触发处理（`object` 为端点 ID）。
    ProtocolDeadline = 6,
}

/// 单条每 CPU 追踪事件记录。
///
/// 仅保存纯标量快照，不持有任务、网络端点或用户空间内存的引用，生命周期与被观测对象完全解耦。
#[derive(Clone, Copy)]
struct Record {
    /// 该事件在所属 CPU 环内的单调递增序号（从 `1` 开始）。
    seq: u64,
    /// 事件采样时的系统单调时钟时间戳（单位：纳秒）。
    monotonic_ns: u64,
    /// 记录该事件的逻辑 CPU 编号。
    cpu: usize,
    /// 事件发生时当前 CPU 正在运行的任务 ID（无当前任务时为 `None`，导出文本记为 `0`）。
    task: Option<TaskId>,
    /// 事件类别编码。
    kind: Kind,
    /// 事件关联的主体对象标识（如 `TaskId`、网络端点编号或网卡编号）。
    object: u64,
    /// 事件携带的附加数值负载（如批处理包数、`WaitId` 或协议状态编码）。
    value: u64,
}

/// 单个 CPU 私有的定长循环覆盖事件环。
#[derive(Clone, Copy)]
struct Ring {
    /// 定长事件槽位数组，按 `(seq - 1) % CAPACITY` 循环写入。
    records: [Option<Record>; CAPACITY],
    /// 本环下一条待写入事件的序号（初始为 `1`，每次提交加 `1`）。
    next_seq: u64,
}

impl Ring {
    /// 编译期创建空的每 CPU 事件环，起始待分配序号为 `1`。
    const fn new() -> Self {
        Self {
            records: [None; CAPACITY],
            next_seq: 1,
        }
    }

    /// 根据已分配序号计算当前环中实际保留的有效事件条数（上限为 `CAPACITY`）。
    fn retained(&self) -> usize {
        (self.next_seq - 1).min(CAPACITY as u64) as usize
    }

    /// 为事件分配本核序号并写入对应槽位；若 `u64` 序号溢出则直接触发 panic。
    fn push(&mut self, mut record: Record) {
        let seq = self.next_seq;
        let next_seq = seq.checked_add(1).expect("trace sequence exhausted");
        record.seq = seq;
        self.records[((seq - 1) % CAPACITY as u64) as usize] = Some(record);
        self.next_seq = next_seq;
    }
}

/// 预留精确容量上界的快照文本写入器（内部 `Vec<u8>` 初始化时已通过 `try_reserve_exact` 预留足够容量，确保格式化过程中绝不触发二次堆内存重分配）。
struct SnapshotText(Vec<u8>);

impl Write for SnapshotText {
    /// 向预分配缓冲区追加文本片段；若超出预留容量上限则直接断言失败，防止隐式扩容。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        assert!(
            text.len() <= self.0.capacity() - self.0.len(),
            "trace text exceeds reserved capacity"
        );
        self.0.extend_from_slice(text.as_bytes());
        Ok(())
    }
}

/// 查询全局追踪开关当前是否处于开启状态。
pub(crate) fn enabled() -> bool {
    ENABLED.load(Ordering::Acquire)
}

/// 在当前已初始化的逻辑 CPU 事件环中记录一条追踪事件（当追踪未开启时以单次原子读快速返回）。
///
/// ## 调用上下文与约束
/// - 允许在普通任务上下文及常规硬件中断（hardirq）路径中调用；全程无堆内存分配、无日志输出、无睡眠。
/// - 严禁在 NMI、`panic`、时间维护写锁临界区、`trace` 内部或 `cpu::call` 跨核回调中调用。
pub(crate) fn record(kind: Kind, object: u64, value: u64) {
    // Step 1: 无锁快速检查开关；默认关闭时仅消耗一次 Relaxed 原子读取。
    if !ENABLED.load(Ordering::Relaxed) {
        return;
    }

    // Step 2: 关闭本地中断以固定当前逻辑 CPU，并与 `stop()` 发送的跨核屏障 IPI 建立先后顺序。
    let _irq = IrqSaveGuard::new();
    let cpu = cpu::current().id;
    // Step 3: 关中断后以 Acquire 语义复查开关；若 `stop()` 已置 `false`，则保证本核响应屏障后不再写入任何新记录。
    if !ENABLED.load(Ordering::Acquire) {
        return;
    }

    // Step 4: 采样纳秒时间戳与当前任务身份，写入本核专属事件环。
    let record = Record {
        seq: 0,
        monotonic_ns: time::now().as_nanos(),
        cpu,
        task: task::current_task_id(),
        kind,
        object,
        value,
    };
    RINGS[cpu].lock().push(record);
}

/// 清空所有实际 CPU 的历史追踪环并开启新一轮全局事件采集（必须在普通可睡眠任务中调用）。
///
/// 若当前追踪已处于开启状态，返回 `Err(FsError::Busy)`。
pub(crate) fn start() -> Result<(), FsError> {
    let _control = CONTROL.lock();
    if enabled() {
        return Err(FsError::Busy);
    }

    // `CONTROL` 睡眠锁保证上一轮 `stop()` 的跨核停稳屏障已全部完成，此时无任何在途生产者会并发写入各核环。
    for owner in cpu::actual_set().iter() {
        let mut ring = RINGS[owner].lock();
        ring.records.fill(None);
        ring.next_seq = 1;
    }
    ENABLED.store(true, Ordering::Release);
    Ok(())
}

/// 跨核函数调用（CSD）空操作回调，仅利用目标 CPU 响应 IPI 的时机充当关中断临界区排空屏障。
///
/// # Safety
/// 1. 只能作为 `cpu::call` 的回调函数在已上线的目标 CPU 中断上下文执行；
/// 2. 函数体内部严禁睡眠、访问入参裸指针、递归调用 `cpu::call` 或调用 `trace::record`。
unsafe fn freeze_cpu(_: *mut ()) {}

/// 停止全局事件采集，并同步等待所有在线 CPU 上已进入关中断路径的在途提交彻底完成（必须在普通可睡眠任务中调用）。
///
/// 若追踪本就处于停止状态，幂等返回 `Ok(())` 且保留环中已冻结的历史记录供 [`snapshot`] 读取。
pub(crate) fn stop() -> Result<(), FsError> {
    let _control = CONTROL.lock();
    if !enabled() {
        return Ok(());
    }
    // Step 1: 先关闭全局采集标志，阻止各核新进入 `record` 关中断段的执行流继续提交。
    ENABLED.store(false, Ordering::Release);

    // Step 2: 逐核执行同步跨核空回调，确保各核在看到 `ENABLED == false` 之前已进入关中断段的在途 `record` 全部落盘退出。
    for target in cpu::actual_set().iter() {
        assert!(
            cpu::get(target).online.load(Ordering::Acquire),
            "trace stop requires online CPUs"
        );
        // SAFETY:
        // 1. `target` 来自 `cpu::actual_set()`，且上方 `assert!` 已用 `Acquire` 验证目标 CPU 处于在线状态；
        // 2. `CONTROL.lock()` 已校验当前处于可调度、开中断的普通任务上下文，且此处未持有任何 `RINGS` 自旋锁，本核在等待期间可正常响应远端发来的 IPI；
        // 3. `freeze_cpu` 为静态空函数，不解引用 `ptr::null_mut()` 参数，也不睡眠、不访问用户内存、不递归发起跨核调用或写追踪记录。
        unsafe { cpu::call(target, freeze_cpu, ptr::null_mut()) };
    }
    Ok(())
}

/// 在追踪已停止的状态下，将所有 CPU 环中保留的事件格式化为独立的 ASCII 文本字节流（必须在普通可睡眠任务中调用）。
///
/// - 若当前仍处于采集状态，返回 `Err(FsError::Busy)`；
/// - 若预分配输出缓冲区时内存不足，返回 `Err(FsError::NoMemory)`。
pub(crate) fn snapshot() -> Result<Vec<u8>, FsError> {
    let _control = CONTROL.lock();
    if enabled() {
        return Err(FsError::Busy);
    }

    // Step 1: 统计所有 CPU 当前保留的记录总数，按固定单行最大字节上界一次性向堆申请精确缓冲区。
    let cpus = cpu::count();
    let mut capacity = HEADER_TEXT_CAPACITY + cpus * CPU_TEXT_CAPACITY;
    for ring in &RINGS[..cpus] {
        capacity += ring.lock().retained() * RECORD_TEXT_CAPACITY;
    }

    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(capacity)
        .map_err(|_| FsError::NoMemory)?;

    // Step 2: 写入全局协议版本头，再按 CPU 编号顺序复制各核环并在自旋锁外完成文本格式化。
    let mut text = SnapshotText(bytes);
    writeln!(
        text,
        "ktrace version=1 unit=ns cpus={} capacity={}",
        cpus, CAPACITY
    )
    .expect("trace header formatting failed");
    for (cpu, source) in RINGS[..cpus].iter().enumerate() {
        let ring = *source.lock();
        let retained = ring.retained() as u64;
        let overwritten = ring.next_seq - 1 - retained;
        writeln!(
            text,
            "cpu id={} next_seq={} overwritten={}",
            cpu, ring.next_seq, overwritten
        )
        .expect("trace CPU formatting failed");
        for seq in ring.next_seq - retained..ring.next_seq {
            let record = ring.records[((seq - 1) % CAPACITY as u64) as usize]
                .expect("trace retained record is missing");
            assert_eq!(record.seq, seq, "trace slot sequence mismatch");
            write!(
                text,
                "record seq={} ns={} cpu={} task=",
                record.seq, record.monotonic_ns, record.cpu
            )
            .expect("trace identity formatting failed");
            match record.task {
                Some(task) => write!(text, "{}", task.as_usize()),
                None => text.write_str("0"),
            }
            .expect("trace task formatting failed");
            writeln!(
                text,
                " kind={} object={} value={}",
                record.kind as u8, record.object, record.value
            )
            .expect("trace event formatting failed");
        }
    }
    Ok(text.0)
}
