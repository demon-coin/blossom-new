//! 逻辑 CPU 位图集合（`CpuSet` 与 `AtomicCpuSet`）。
//!
//! # 模块职责与架构定位
//! 本模块在编译期最大 CPU 容量（`MAX_CPUS`）内提供无堆分配的定长逻辑 CPU 位图：
//! - 上游调用者：调度器亲和性（`sched`）、系统调用 `sched_getaffinity`/`sched_setaffinity`、多核启动与停机协调（`cpu`）以及内核自测（`ktest`）。
//! - 下游依赖：仅依赖 `boot::MAX_CPUS` 编译期常量与 `core` 原子操作，不依赖堆分配器或运行时启动信息。
//!
//! # 核心设计与位图内存布局
//! `CpuSet` 使用定长 `usize` 数组 `[usize; WORDS]` 按小端位序（Little-Endian Bit Order）存储逻辑 CPU 编号，与 Linux `cpu_set_t`（`unsigned long` 数组）的二进制布局兼容：
//! ```text
//! words[0] (64 bits on 64-bit targets):
//! ┌─────────┬─────────┬─────┬─────────┬─────────┐
//! │ bit 63  │ bit 62  │ ... │  bit 1  │  bit 0  │
//! │ CPU 63  │ CPU 62  │ ... │  CPU 1  │  CPU 0  │
//! └─────────┴─────────┴─────┴─────────┴─────────┘
//! 映射关系:
//!   word_index = cpu_id / WORD_BITS
//!   bit_offset = cpu_id % WORD_BITS
//! ```

#[cfg(feature = "ktest")]
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::boot::MAX_CPUS;

/// 单个机器字（`usize`）包含的比特位数（64 位架构下为 64）。
const WORD_BITS: usize = usize::BITS as usize;
/// 覆盖 `MAX_CPUS` 个逻辑 CPU 所需的 `usize` 数组长度（向上取整）。
const WORDS: usize = MAX_CPUS.div_ceil(WORD_BITS);

/// 编译期定长的逻辑 CPU 位图集合，按值传递且不涉及堆内存分配。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuSet {
    /// 按机器字组织的位图数组；第 `id` 号逻辑 CPU 对应 `words[id / WORD_BITS]` 的第 `id % WORD_BITS` 位。
    words: [usize; WORDS],
}

impl Default for CpuSet {
    /// 返回不包含任何 CPU 的空集合。
    fn default() -> Self {
        Self::empty()
    }
}

impl CpuSet {
    /// 构造一个不包含任何逻辑 CPU 的空集合。
    pub const fn empty() -> Self {
        Self { words: [0; WORDS] }
    }

    /// 构造包含逻辑 CPU 编号 `[0, count)` 的连续前缀集合。
    ///
    /// 当某个机器字被完整覆盖（`WORD_BITS <= bits`）时直接填充 `usize::MAX`，避免对 `usize` 执行等于字宽的左移导致溢出。
    pub const fn prefix(count: usize) -> Self {
        assert!(count <= MAX_CPUS);
        let mut set = Self::empty();
        let mut word = 0;
        while word < WORDS {
            let start = word * WORD_BITS;
            if start < count {
                let bits = count - start;
                set.words[word] = if WORD_BITS <= bits {
                    usize::MAX
                } else {
                    (1usize << bits) - 1
                };
            }
            word += 1;
        }
        set
    }

    /// 构造仅包含单个逻辑 CPU `id` 的集合；要求 `id < MAX_CPUS`。
    pub const fn single(id: usize) -> Self {
        assert!(id < MAX_CPUS);
        let mut set = Self::empty();
        set.words[id / WORD_BITS] = 1usize << (id % WORD_BITS);
        set
    }

    /// 判断逻辑 CPU `id` 是否在集合中；若 `id >= MAX_CPUS` 则直接返回 `false`。
    pub fn contains(self, id: usize) -> bool {
        id < MAX_CPUS && self.words[id / WORD_BITS] & (1usize << (id % WORD_BITS)) != 0
    }

    /// 将逻辑 CPU `id` 加入集合；要求 `id < MAX_CPUS`。
    pub fn insert(&mut self, id: usize) {
        assert!(id < MAX_CPUS);
        self.words[id / WORD_BITS] |= 1usize << (id % WORD_BITS);
    }

    /// 从集合中移除逻辑 CPU `id`；要求 `id < MAX_CPUS`。
    #[cfg(feature = "ktest")]
    pub fn remove(&mut self, id: usize) {
        assert!(id < MAX_CPUS);
        self.words[id / WORD_BITS] &= !(1usize << (id % WORD_BITS));
    }

    /// 计算并返回当前集合与 `other` 的并集（$A \cup B$）。
    #[cfg(feature = "ktest")]
    pub fn union(mut self, other: Self) -> Self {
        for (word, rhs) in self.words.iter_mut().zip(other.words) {
            *word |= rhs;
        }
        self
    }

    /// 计算并返回当前集合与 `other` 的交集（$A \cap B$）。
    pub fn intersection(mut self, other: Self) -> Self {
        for (word, rhs) in self.words.iter_mut().zip(other.words) {
            *word &= rhs;
        }
        self
    }

    /// 计算并返回当前集合与 `other` 的差集（$A \setminus B$，即属于 `self` 但不属于 `other` 的成员）。
    #[cfg(feature = "ktest")]
    pub fn difference(mut self, other: Self) -> Self {
        for (word, rhs) in self.words.iter_mut().zip(other.words) {
            *word &= !rhs;
        }
        self
    }

    /// 判断集合是否为空（不包含任何逻辑 CPU）。
    pub fn is_empty(self) -> bool {
        self.words.iter().all(|word| *word == 0)
    }

    /// 返回按逻辑 CPU 编号升序遍历集合成员的迭代器。
    pub fn iter(self) -> CpuSetIter {
        CpuSetIter { set: self, word: 0 }
    }

    /// 返回位图底层 `usize` 数组所占的总字节数（对齐 Linux `cpu_set_t` 机器字粒度）。
    pub const fn byte_len() -> usize {
        WORDS * core::mem::size_of::<usize>()
    }

    /// 从小端字节切片反序列化 `CpuSet`。
    ///
    /// 最多读取前 `Self::byte_len()` 个字节（不足部分视为 0，超出部分忽略），并通过与 `Self::prefix(MAX_CPUS)` 求交集屏蔽超出 `MAX_CPUS` 的高位比特。
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut set = Self::empty();
        for (index, byte) in bytes.iter().take(Self::byte_len()).enumerate() {
            set.words[index / core::mem::size_of::<usize>()] |=
                usize::from(*byte) << ((index % core::mem::size_of::<usize>()) * u8::BITS as usize);
        }
        set.intersection(Self::prefix(MAX_CPUS))
    }

    /// 将集合按小端字节序写入目标切片 `bytes`（要求 `bytes.len() <= Self::byte_len()`）。
    ///
    /// 若用户态缓冲区长于 `Self::byte_len()`，超出部分的清零或保留由系统调用 ABI 层负责处理。
    pub fn write_bytes(self, bytes: &mut [u8]) {
        assert!(bytes.len() <= Self::byte_len());
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = (self.words[index / core::mem::size_of::<usize>()]
                >> ((index % core::mem::size_of::<usize>()) * u8::BITS as usize))
                as u8;
        }
    }
}

/// 按逻辑 CPU 编号升序遍历 [`CpuSet`] 成员的迭代器。
pub struct CpuSetIter {
    /// 待消费的剩余位图副本；每次迭代会清除已返回的最低有效位。
    set: CpuSet,
    /// 当前正在扫描的机器字下标（范围为 `0..WORDS`）。
    word: usize,
}

impl Iterator for CpuSetIter {
    type Item = usize;

    /// 提取并清除当前非零机器字中的最低有效位（利用 `trailing_zeros` 与 `*bits &= *bits - 1`），返回对应的逻辑 CPU 编号。
    fn next(&mut self) -> Option<Self::Item> {
        while self.word < WORDS {
            let bits = &mut self.set.words[self.word];
            if *bits != 0 {
                let bit = bits.trailing_zeros() as usize;
                *bits &= *bits - 1;
                return Some(self.word * WORD_BITS + bit);
            }
            self.word += 1;
        }
        None
    }
}

#[cfg(feature = "ktest")]
/// 支持多核并发无锁置位的原子 CPU 位图集合，供内核自测（`ktest`）记录曾经执行过某段逻辑的 CPU 集合。
pub(crate) struct AtomicCpuSet {
    /// 按机器字组织的原子位图数组，每个元素独立支持原子位或与交换操作。
    words: [AtomicUsize; WORDS],
}

#[cfg(feature = "ktest")]
impl AtomicCpuSet {
    /// 编译期构造一个初始为空的原子 CPU 集合。
    pub const fn new() -> Self {
        Self {
            words: [const { AtomicUsize::new(0) }; WORDS],
        }
    }

    /// 原子地将逻辑 CPU `id` 加入集合（使用 `fetch_or`）；要求 `id < MAX_CPUS`。
    pub fn insert(&self, id: usize, ordering: Ordering) {
        assert!(id < MAX_CPUS);
        self.words[id / WORD_BITS].fetch_or(1usize << (id % WORD_BITS), ordering);
    }

    /// 逐个机器字读取当前已记录的 CPU 集合并返回 [`CpuSet`] 快照。
    pub fn load(&self, ordering: Ordering) -> CpuSet {
        CpuSet {
            words: core::array::from_fn(|word| self.words[word].load(ordering)),
        }
    }

    /// 逐个机器字原子地清零并取走当前已记录的 CPU 集合。
    pub fn take(&self, ordering: Ordering) -> CpuSet {
        CpuSet {
            words: core::array::from_fn(|word| self.words[word].swap(0, ordering)),
        }
    }
}
