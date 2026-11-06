//! 内核日志定长环形缓冲区（Log Ring Buffer）。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `klog` 子系统最底层，维护全局唯一的定长日志槽位数组与单调递增序号：
//! - **上游写入方**：`klog::write` 在栈上完成文本格式化、时间戳与上下文采样后，调用 [`store`] 将完整记录写入环形缓冲区。
//! - **下游读取方**：后台打印服务（`klog::printer`）、`/dev/kmsg`（`fs::kmsg`）与 `syslog` 系统调用各自持有独立的 [`Cursor`]，互不干扰地按序号拉取记录副本。
//!
//! ## 核心设计与序号映射
//! 有效日志序号从 `1` 开始严格单调递增（`0` 保留为空哨兵），通过模运算映射到 `CAPACITY`（64）个槽位：
//!
//! ```text
//!  全局序号空间:  1 ... [ oldest = max(1, next - CAPACITY) ....... next - 1 ]  next (待分配)
//!                               ▲                                       ▲
//!                        当前仍可读取的最老记录                  最新已发布记录
//!
//!  槽位映射关系:  slot_index = ((seq - 1) % CAPACITY) as usize
//!
//!  游标读取判定 (Cursor::next):
//!    - Cursor.next >= Ring.next   ──► 尚无新记录可读，返回 None
//!    - Cursor.next <  Ring.oldest ──► 游标落后已被覆盖，跳至 oldest 读取，报告 lost = oldest - Cursor.next
//!    - Ring.oldest <= Cursor.next < Ring.next ──► 正常读取对应槽位副本，lost = 0，游标推进至 seq + 1
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 本模块仅包含一把全局自旋锁 `RING`（内部自动保存并关闭本地中断），为叶级（Leaf）锁。
//! - **严禁在 `RING` 临界区内调用任何外部子系统**（禁止格式化文本、读取时钟、分配堆内存、唤醒调度器、操作串口或再次调用 `klog`），临界区内只允许做定长内存拷贝与整数运算，以保证微秒级的确定性持锁时间。

use crate::sync::SpinLock;
use crate::task::TaskId;

use super::{Level, CAPACITY, TEXT_CAPACITY};

/// 单条完整的内核日志记录。
///
/// 结构体以定长数组内联持有全部正文字节与标量元数据；消费者通过 `Clone` 取得独立副本后，
/// 即使环形缓冲区的原槽位随后被新日志覆盖，消费者手中的记录依然完整有效。
#[derive(Clone)]
pub struct Record {
    /// 全局单调递增的日志序号（从 `1` 开始，由 `Ring::store` 在入环时分配）。
    pub seq: u64,
    /// 日志严重程度级别。
    pub level: Level,
    /// `bytes` 中有效 UTF-8 正文的前缀字节数（单位：字节，满足 `len <= TEXT_CAPACITY`）。
    pub len: usize,
    /// 标记原始日志正文是否因超出 `TEXT_CAPACITY` 而被截断。
    pub truncated: bool,
    /// 记录产生时的系统单调时钟时间戳（单位：纳秒）。
    pub monotonic_ns: u64,
    /// 产生该条日志的逻辑 CPU 编号。
    pub cpu: usize,
    /// 产生该条日志时正在运行的任务 ID（启动早期或空闲上下文为 `None`）。
    pub task: Option<TaskId>,
    /// 内联存储的 UTF-8 正文字节缓冲区，仅 `bytes[..len]` 为有效内容。
    pub(super) bytes: [u8; TEXT_CAPACITY],
}

impl Record {
    /// 以 `&str` 借用记录中的有效 UTF-8 正文切片；若公开字段 `len` 被篡改导致越界或破坏 UTF-8 边界，直接触发 panic。
    pub fn text(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).expect("invalid klog text length")
    }
}

/// 游标单次读取返回的结果，包含日志记录副本及本次读取前被覆盖丢失的记录条数。
pub struct ReadRecord {
    /// 本次读取到的完整日志记录副本。
    pub record: Record,
    /// 自该游标上次读取以来，因生产者环绕覆盖而未能读到的日志条数（`0` 表示连续无丢失）。
    pub lost: u64,
}

/// 定长循环覆盖的内核日志存储环（由全局自旋锁 `RING` 保护）。
struct Ring {
    /// 固定容量的槽位数组；槽位按 `(seq - 1) % CAPACITY` 循环复用。
    slots: [Option<Record>; CAPACITY],
    /// 下一条待发布日志将分配的序号（初始为 `1`，严格单调递增）。
    next: u64,
}

impl Ring {
    /// 编译期构造空的日志环形缓冲区，起始待分配序号设为 `1`（序号 `0` 不代表任何有效记录）。
    const fn new() -> Self {
        Self {
            slots: [const { None }; CAPACITY],
            next: 1,
        }
    }

    /// 计算当前环形缓冲区中尚未被覆盖的最老可读序号（缓冲区未写满时返回 `1`）。
    fn oldest(&self) -> u64 {
        self.next.saturating_sub(CAPACITY as u64).max(1)
    }

    /// 为记录分配递增序号并写入对应槽位；若 `u64` 序号即将溢出则返回 `None`，由调用方在锁外报错。
    fn store(&mut self, mut record: Record) -> Option<u64> {
        let following = self.next.checked_add(1)?;
        let sequence = self.next;
        record.seq = sequence;
        self.slots[((sequence - 1) % CAPACITY as u64) as usize] = Some(record);
        self.next = following;
        Some(sequence)
    }
}

// ponytail: 全局单把自旋锁串行定长记录拷贝；当实测多核高频日志出现锁争用瓶颈时再改为每 CPU 分片环。
static RING: SpinLock<Ring> = SpinLock::new(Ring::new());

/// 获取 `RING` 自旋锁并将已在锁外组装完毕的日志记录写入缓冲区，返回分配的全局序号。
pub(super) fn store(record: Record) -> Option<u64> {
    RING.lock().store(record)
}

/// 获取下一条尚未发布的日志序号（常作为 `syslog` 或测试同步等待的排他右边界快照）。
pub fn next_sequence() -> u64 {
    RING.lock().next
}

/// 消费者私有的日志读取游标。
///
/// 每个消费者（如后台打印任务、每个打开的 `/dev/kmsg` 文件实例）独立维护各自的 `Cursor`，读取操作不会影响环形缓冲区本身或其他消费者。
pub struct Cursor {
    /// 该游标期望读取的下一条日志序号（满足 `next >= 1`）。
    next: u64,
}

impl Cursor {
    /// 创建一个定位在当前环形缓冲区最老可读记录处的游标；创建前更早被覆盖的历史不计入该游标的 `lost` 计数。
    pub fn oldest() -> Self {
        Self {
            next: RING.lock().oldest(),
        }
    }

    /// 从指定的非零序号创建游标（允许传入尚未生成的未来序号，后续读取时若已被覆盖会自动计算缺口）。
    pub fn from_sequence(sequence: u64) -> Self {
        assert!(1 <= sequence, "klog cursor sequence must be nonzero");
        Self { next: sequence }
    }

    /// 非破坏性地检查下一条可读记录的状态：`None` 表示尚无新日志，`Some(0)` 表示下一条正常可读，`Some(n)`（`n > 0`）表示有新日志但游标已落后并丢失了 `n` 条记录。
    pub fn available(&self) -> Option<u64> {
        let ring = RING.lock();
        if ring.next <= self.next {
            return None;
        }
        Some(self.next.max(ring.oldest()) - self.next)
    }

    /// 从环形缓冲区读取下一条可用记录的独立副本，并将游标推进至所读记录的下一个序号。
    ///
    /// - 若尚无序号大于等于 `self.next` 的记录发布，返回 `None` 且不改变游标位置。
    /// - 若 `self.next` 落后于缓冲区当前保留的最老序号，自动快进至最老可用记录，并通过 `ReadRecord::lost` 报告中间被覆盖的条数（注意：`lost` 表示整条记录被覆盖丢失，与单条记录内部的文本截断标志 `Record::truncated` 相互独立）。
    pub fn read(&mut self) -> Option<ReadRecord> {
        let ring = RING.lock();
        if ring.next <= self.next {
            return None;
        }

        // 若游标已落后于最老保留序号，跳到 ring.oldest() 读取并统计被覆盖的缺口条数。
        let sequence = self.next.max(ring.oldest());
        let record = ring.slots[((sequence - 1) % CAPACITY as u64) as usize]
            .as_ref()
            .expect("published klog record missing")
            .clone();
        let lost = sequence - self.next;
        // `Ring::store` 仅在 `checked_add(1)` 成功时才发布记录，因此已发布记录的 `sequence + 1` 必定不会算术溢出。
        self.next = sequence + 1;
        Some(ReadRecord { record, lost })
    }
}
