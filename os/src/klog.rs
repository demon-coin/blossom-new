//! 内核结构化日志子系统（Kernel Log）。
//!
//! ## 模块职责与架构定位
//! 本模块为内核各子系统提供低侵入、定长缓冲的结构化日志记录能力：
//! - **上游生产者**：普通内核任务、硬件中断处理路径以及持有调度器/工作池自旋锁的执行流（通过 [`klog!`] 宏或 [`write`] 入口提交日志）。
//! - **下游消费者**：
//!   1. `klog::printer` 后台串口输出任务：将环形缓冲区中的日志逐字节异步输出到控制台 UART；
//!   2. `/dev/kmsg` 字符设备（`fs::kmsg`）与 `syslog` 系统调用：通过独立 [`Cursor`] 读取历史与实时日志，并通过 [`reader_source`] 支持 `poll`/`epoll` 事件多路复用。
//!
//! ## 核心设计与提交流程
//! 为避免日志路径阻塞高优先级中断或引发锁反转，生产者与消费者的职责在时间与空间上彻底解耦：
//!
//! ```text
//!  生产者 (任务 / 硬中断 / 持自旋锁上下文)
//!    │ 1. 栈上格式化 UTF-8 正文 (StackText<512>，超长按字符边界截断)
//!    │ 2. 关中断读取当前 CPU 编号与 TaskId 快照 + 单调时钟纳秒戳
//!    │ 3. 持 RING 自旋锁按值写入定长槽位并分配递增序号 seq
//!    │ 4. 向 CPU 0 发送 KICK_LOG_PRINTER 核间中断 (异步投递，不直接唤醒调度器)
//!    ▼
//!  [ 定长环形缓冲区 Ring (64 槽 × Record) ]
//!    │
//!    ├──► CPU 0 IPI 处理函数调用 wake_printer()
//!    │      ├──► 通知 READER_SOURCE (唤醒 poll/epoll 等待 /dev/kmsg 的读者)
//!    │      └──► 唤醒 RECORD_WAIT 等待队列上的后台打印任务
//!    ▼
//!  后台打印任务 (printer::run) / 用户态读者 (Cursor)
//! ```
//!
//! ## 锁获取顺序与调用上下文契约
//! - **锁层级**：`write` 内部依次独立获取短临界区资源，互不嵌套：
//!   1. 关中断读取本核 `CpuLocal` 标量快照；
//!   2. 读取单调时钟 `time::now()`；
//!   3. 获取 `ring::RING` 自旋锁写入记录；
//!   4. 调用 `cpu::kick` 置位原子标志并触发硬件 IPI。
//!
//!   因此，即使调用方已持有调度器锁（`sched`）或工作队列池锁（`workqueue`），调用 `write` 也不会因为直接唤醒任务而重入调度器死锁。
//! - **禁止调用的上下文**：
//!   1. CPU 局部存储与单调时钟源完成初始化之前；
//!   2. 时间维护（timekeeper）写锁临界区（`write` 会读取时间，若打断写者将导致死锁）；
//!   3. `klog::ring` 自旋锁临界区内部（避免自旋锁递归死锁）；
//!   4. 内核 `panic` 路径（应直接走紧急串口直写路径，避免等待可能崩溃前已持有的锁）。
//! - **格式化参数契约**：在中断或持自旋锁上下文中调用时，只应传入标量或静态短字符串切片；若传入自定义 `Display`/`Debug` 对象，调用方须自行确保其 `fmt` 实现不会睡眠、分配堆内存、获取冲突锁或递归调用日志。

mod printer;
mod ring;

pub use printer::initialize_service;
#[cfg(feature = "ktest")]
pub use printer::testing;
pub(crate) use printer::wake_printer;
pub use ring::{next_sequence, Cursor, ReadRecord, Record};

use alloc::sync::Arc;
use core::fmt;
use core::sync::atomic::Ordering;
use spin::Once;

use crate::fs::poll::{PollMask, PollWaitHead};
use crate::sync::IrqSaveGuard;
use crate::task::TaskId;

/// 内核日志环形缓冲区的最大记录槽位数（超出后循环覆盖最老记录）。
pub const CAPACITY: usize = 64;
/// 单条日志记录允许保存的最大 UTF-8 正文字节数（超出部分按字符边界截断）。
pub const TEXT_CAPACITY: usize = 512;

/// 全局日志读者事件源，由引导核在 `initialize_service` 阶段一次性初始化并常驻内存，供 `/dev/kmsg` 挂载 `poll`/`epoll` 监听。
static READER_SOURCE: Once<Arc<PollWaitHead>> = Once::new();

/// 获取全局日志读者 `poll`/`epoll` 等待队列头引用；必须在 [`initialize_service`] 完成后调用，否则触发 panic。
pub(crate) fn reader_source() -> &'static Arc<PollWaitHead> {
    READER_SOURCE
        .get()
        .expect("klog reader source not initialized")
}

/// 在核间中断（IPI）延迟处理点广播日志可读或环绕覆盖事件；仅唤醒等待者，各读者随后通过各自的 [`Cursor`] 自行检查实际序号与丢失量。
pub(crate) fn wake_readers() {
    reader_source()
        .notify(PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLERR | PollMask::POLLPRI);
}

/// 内核日志严重程度级别。
///
/// 按严重程度从高到低排列，以 `u8` 紧凑表示，直接映射为输出协议头部的固定文本标签。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Level {
    /// 错误级别：子系统发生无法局部静默恢复的故障或违规状态。
    Error,
    /// 警告级别：遇到异常边界或降级处理，但系统仍可继续运行。
    Warn,
    /// 信息级别：关键生命周期节点或重要的状态迁移事件。
    Info,
    /// 调试级别：供开发与诊断参考的细粒度运行轨迹。
    Debug,
}

impl Level {
    /// 返回串口与文本输出协议使用的全大写级别名称字符串。
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Error => "ERROR",
            Self::Warn => "WARN",
            Self::Info => "INFO",
            Self::Debug => "DEBUG",
        }
    }
}

/// 固定容量的栈上 UTF-8 文本格式化缓冲区。
///
/// 用于在获取日志环自旋锁之前，在调用者栈上完成字符串拼接，避免堆内存分配。
pub(super) struct StackText<const N: usize> {
    /// 存放已写入字节的固定长度栈数组，仅前 `len` 个字节为有效 UTF-8 内容。
    bytes: [u8; N],
    /// 当前已成功写入 `bytes` 的有效字节长度（满足 `len <= N` 且位于 UTF-8 字符边界）。
    len: usize,
    /// 标记格式化输出是否因超出容量 `N` 而发生过截断。
    truncated: bool,
}

impl<const N: usize> StackText<N> {
    /// 创建一个空的栈上文本缓冲区，不执行堆分配。
    pub(super) const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            truncated: false,
        }
    }

    /// 返回当前已写入的有效 UTF-8 字节切片。
    pub(super) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    /// 返回写入过程中是否因缓冲区容量不足而截断了后续文本。
    pub(super) fn is_truncated(&self) -> bool {
        self.truncated
    }
}

impl<const N: usize> fmt::Write for StackText<N> {
    /// 向栈缓冲区追加字符串片段，确保结果始终为合法 UTF-8 前缀。
    ///
    /// 当剩余空间不足以容纳整个 `text` 时，回退到最后一个完整的 UTF-8 字符边界写入前缀，
    /// 并将 `truncated` 置为 `true`；此后继续追加的片段将被直接忽略并返回 `Ok(())`，以便区分“容量截断”与格式化器自身的错误。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if self.truncated {
            return Ok(());
        }
        // 计算剩余字节容量上限，并向前回退至合法的 UTF-8 字符边界，防止截断多字节字符产生乱码。
        let mut copied = text.len().min(N - self.len);
        while !text.is_char_boundary(copied) {
            copied -= 1;
        }
        self.bytes[self.len..self.len + copied].copy_from_slice(&text.as_bytes()[..copied]);
        self.len += copied;
        self.truncated = copied < text.len();
        Ok(())
    }
}

/// 产生日志瞬间的本核执行上下文身份快照。
///
/// 仅复制标量编号，不持有 `Task` 引用计数或调度器锁。
struct IdentitySnapshot {
    /// 提交日志时所在逻辑 CPU 的编号。
    cpu: usize,
    /// 提交日志时该 CPU 正在运行的任务 ID（若处于早期启动或纯空闲上下文则为 `None`）。
    task: Option<TaskId>,
}

impl IdentitySnapshot {
    /// 临时关闭本地中断，原子地采样当前逻辑 CPU 编号与正在运行的 `TaskId`。
    fn capture() -> Self {
        let _irq = IrqSaveGuard::new();
        let local = crate::cpu::current();
        Self {
            cpu: local.id,
            task: TaskId::from_raw(local.current_task_id.load(Ordering::Relaxed)),
        }
    }
}

/// 格式化并提交一条结构化内核日志记录，返回分配给该记录的全局单调递增序号（从 1 开始）。
///
/// ## 调用上下文与约束
/// - 可在普通可睡眠任务、硬件中断处理路径或持有调度器/工作池自旋锁的临界区中调用（前提是 CPU 局部存储与单调时钟已完成初始化）。
/// - 严禁在时间维护写锁临界区、`klog::ring` 内部或 `panic` 处理路径中调用，否则会导致自旋锁死锁。
/// - 传入的 `arguments` 在本函数栈上立即完成格式化：若处于不可睡眠或持锁上下文，调用方必须保证参数的 `Display`/`Debug` 实现不会睡眠、分配堆内存或获取冲突锁。
/// - 若格式化器报错或 64 位日志序号耗尽，将在获取日志环锁之外直接触发 panic。
pub fn write(level: Level, arguments: fmt::Arguments<'_>) -> u64 {
    // Step 1: 在栈上完成文本格式化，避免在日志环自旋锁内执行耗时的字符串拼接。
    let mut text = StackText::<TEXT_CAPACITY>::new();
    fmt::write(&mut text, arguments).expect("klog formatter failed");

    // Step 2: 采集当前 CPU、任务 ID 与单调时钟纳秒时间戳，组装定长记录值。
    let identity = IdentitySnapshot::capture();
    let monotonic_ns = crate::time::now().as_nanos();
    let record = Record {
        seq: 0,
        level,
        len: text.len,
        truncated: text.truncated,
        monotonic_ns,
        cpu: identity.cpu,
        task: identity.task,
        bytes: text.bytes,
    };

    // Step 3: 持短自旋锁将记录写入环形缓冲区并获取发布序号。
    let sequence = ring::store(record).expect("klog sequence exhausted");

    // Step 4: 通过异步 IPI 通知引导核（CPU 0）唤醒后台打印任务与 /dev/kmsg 读者，避免在当前上下文直接进入调度器。
    crate::cpu::kick(0, crate::cpu::KICK_LOG_PRINTER);
    sequence
}

/// 按指定日志级别将格式化文本写入内核日志环，并返回该条记录的全局序号。
#[macro_export]
macro_rules! klog {
    ($level:expr, $($argument:tt)*) => {
        $crate::klog::write($level, ::core::format_args!($($argument)*))
    };
}
