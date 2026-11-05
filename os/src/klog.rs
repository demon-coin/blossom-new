//! 正常记录只当场消费标量或静态短文本；保存完整记录后延后通知独立输出任务。
//!
//! CPU 与时间源必须已经初始化。时间维护写侧、记录环内部和 panic 路径禁止调用
//! 普通记录；这些上下文不能等待被自己打断的时间写者，也不能递归取得记录锁。

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

pub const CAPACITY: usize = 64;
pub const TEXT_CAPACITY: usize = 512;

static READER_SOURCE: Once<Arc<PollWaitHead>> = Once::new();

/// 借用 BSP 预备并永久保活的读者通知源；初始化之前调用直接失败。
pub(crate) fn reader_source() -> &'static Arc<PollWaitHead> {
    READER_SOURCE
        .get()
        .expect("klog reader source not initialized")
}

/// 在日志 IPI 的延期点提示可读或覆盖；通知不消费记录，实际状态由各游标复查。
pub(crate) fn wake_readers() {
    reader_source()
        .notify(PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLERR | PollMask::POLLPRI);
}

/// 固定的小值级别，不建立运行期过滤或动态目标配置。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Level {
    Error,
    Warn,
    Info,
    Debug,
}

impl Level {
    /// 返回输出协议使用的固定级别文本，不格式化业务对象。
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Error => "ERROR",
            Self::Warn => "WARN",
            Self::Info => "INFO",
            Self::Debug => "DEBUG",
        }
    }
}

/// 栈内 UTF-8 前缀；容量仅限制保存量，不约束任意 Display 的计算或锁行为。
pub(super) struct StackText<const N: usize> {
    bytes: [u8; N],
    len: usize,
    truncated: bool,
}

impl<const N: usize> StackText<N> {
    /// 创建不借用外部正文、也不申请堆存储的空缓冲。
    pub(super) const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            truncated: false,
        }
    }

    /// 借用本次栈缓冲已经完整保存的字节，不能超过缓冲生命周期。
    pub(super) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    /// 区分容量截断与格式化对象主动返回的错误。
    pub(super) fn is_truncated(&self) -> bool {
        self.truncated
    }
}

impl<const N: usize> fmt::Write for StackText<N> {
    /// 仅保留合法 UTF-8 前缀；截断返回成功，后续片段不再拼入前缀。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if self.truncated {
            return Ok(());
        }
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

/// 本 CPU 的短标量身份，不携带 Task 拥有权或调度锁借用。
struct IdentitySnapshot {
    cpu: usize,
    task: Option<TaskId>,
}

impl IdentitySnapshot {
    /// 在短 IRQ-save 段同时取 CPU 与当前 TaskId，离开后不保留 CpuLocal 借用。
    fn capture() -> Self {
        let _irq = IrqSaveGuard::new();
        let local = crate::cpu::current();
        Self {
            cpu: local.id,
            task: TaskId::from_raw(local.current_task_id.load(Ordering::Relaxed)),
        }
    }
}

/// 在已初始化 CPU/时间上下文保存完整记录，返回实际发表序号。
///
/// 普通任务、IRQ 以及持调度或 pool 短锁的调用者只能传标量或静态短文本；
/// 任意 Display/Debug 可能睡眠、分配、反取业务锁或递归日志，本入口不使其安全。
/// 时间维护写侧、记录环内部和 panic 路径禁止调用；格式化失败与序号耗尽
/// 都在记录锁外报告错误，绝不将失败伪装成已保存的记录。
pub fn write(level: Level, arguments: fmt::Arguments<'_>) -> u64 {
    let mut text = StackText::<TEXT_CAPACITY>::new();
    fmt::write(&mut text, arguments).expect("klog formatter failed");

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

    let sequence = ring::store(record).expect("klog sequence exhausted");
    crate::cpu::kick(0, crate::cpu::KICK_LOG_PRINTER);
    sequence
}

/// 通过同一个正常入口当场格式化，不保存 Arguments 或正文借用。
#[macro_export]
macro_rules! klog {
    ($level:expr, $($argument:tt)*) => {
        $crate::klog::write($level, ::core::format_args!($($argument)*))
    };
}
