//! 一个固定完整记录环；锁内仅搬运已有字节与更新序号，不格式化或通知调度器。

use crate::sync::SpinLock;
use crate::task::TaskId;

use super::{Level, CAPACITY, TEXT_CAPACITY};

/// 独立拥有全部正文与元数据；被覆盖的槽不影响已交给消费者的副本。
#[derive(Clone)]
pub struct Record {
    pub seq: u64,
    pub level: Level,
    pub len: usize,
    pub truncated: bool,
    pub monotonic_ns: u64,
    pub cpu: usize,
    pub task: Option<TaskId>,
    pub(super) bytes: [u8; TEXT_CAPACITY],
}

impl Record {
    /// 借用拥有的合法 UTF-8 正文；调用者自行破坏公开 len 时直接报错而非产生无效引用。
    pub fn text(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).expect("invalid klog text length")
    }
}

/// 一次读取的稳定整记录与该游标在本次读取之前遗漏的精确数量。
pub struct ReadRecord {
    pub record: Record,
    pub lost: u64,
}

struct Ring {
    slots: [Option<Record>; CAPACITY],
    next: u64,
}

impl Ring {
    /// 静态建立全部槽，序号从一开始且零不代表实际记录。
    const fn new() -> Self {
        Self {
            slots: [const { None }; CAPACITY],
            next: 1,
        }
    }

    /// 计算仍可读取的最老序号，未写满时保持一。
    fn oldest(&self) -> u64 {
        self.next.saturating_sub(CAPACITY as u64).max(1)
    }

    /// 锁内只发表完整记录；耗尽返回 None，由调用者在解锁后报告。
    fn store(&mut self, mut record: Record) -> Option<u64> {
        let following = self.next.checked_add(1)?;
        let sequence = self.next;
        record.seq = sequence;
        self.slots[((sequence - 1) % CAPACITY as u64) as usize] = Some(record);
        self.next = following;
        Some(sequence)
    }
}

// ponytail: 一个短锁串行完整记录复制；只有实测生产者争用才考虑分片。
static RING: SpinLock<Ring> = SpinLock::new(Ring::new());

/// 只接收已在锁外完成的拥有记录，不调用时间、格式化、堆、调度器或端口。
pub(super) fn store(record: Record) -> Option<u64> {
    RING.lock().store(record)
}

/// 返回固定追赶边界；不会因消费者等待而追逐后来提交的记录。
pub fn next_sequence() -> u64 {
    RING.lock().next
}

/// 消费者独立的下一序号，不改变环或其他消费者的进度。
pub struct Cursor {
    next: u64,
}

impl Cursor {
    /// 从调用瞬间仍保存的最老记录开始，之前的历史不计入该新游标的丢失量。
    pub fn oldest() -> Self {
        Self {
            next: RING.lock().oldest(),
        }
    }

    /// 从指定的非零序号开始；允许未来序号，读取时按实际覆盖报告缺口。
    pub fn from_sequence(sequence: u64) -> Self {
        assert!(1 <= sequence, "klog cursor sequence must be nonzero");
        Self { next: sequence }
    }

    /// 查询下一条的覆盖量，Some(0) 表示可读且未覆盖，None 表示尚无下一条；不推进或复制正文。
    pub fn available(&self) -> Option<u64> {
        let ring = RING.lock();
        if ring.next <= self.next {
            return None;
        }
        Some(self.next.max(ring.oldest()) - self.next)
    }

    /// 在一个短锁内取得稳定副本并沿实际序号推进；空读不重放也不消费未来记录。
    ///
    /// lost 只描述此游标的槽覆盖，正文 truncated 与它互不替代。环内禁止调用
    /// 普通日志，复制没有 fmt、堆存储、Task 拥有权、调度锁或端口访问。
    pub fn read(&mut self) -> Option<ReadRecord> {
        let ring = RING.lock();
        if ring.next <= self.next {
            return None;
        }

        let sequence = self.next.max(ring.oldest());
        let record = ring.slots[((sequence - 1) % CAPACITY as u64) as usize]
            .as_ref()
            .expect("published klog record missing")
            .clone();
        let lost = sequence - self.next;
        // store 只发表具有可表示后继的序号，所以这里不能回绕。
        self.next = sequence + 1;
        Some(ReadRecord { record, lost })
    }
}
