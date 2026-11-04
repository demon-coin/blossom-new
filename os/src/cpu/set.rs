//! 编译容量内的逻辑 CPU 位图；硬件路由和信号 ABI 不使用此表示。

#[cfg(feature = "ktest")]
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::boot::MAX_CPUS;

const WORD_BITS: usize = usize::BITS as usize;
const WORDS: usize = MAX_CPUS.div_ceil(WORD_BITS);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CpuSet {
    words: [usize; WORDS],
}

impl Default for CpuSet {
    /// 默认值只表达空集合，不读取尚未发布的启动资源。
    fn default() -> Self {
        Self::empty()
    }
}

impl CpuSet {
    /// 创建没有成员的无堆集合。
    pub const fn empty() -> Self {
        Self { words: [0; WORDS] }
    }

    /// 将已验证的逻辑编号前缀转换为集合，不对整字宽执行移位。
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

    /// 创建一个已验证逻辑 CPU 的集合。
    pub const fn single(id: usize) -> Self {
        assert!(id < MAX_CPUS);
        let mut set = Self::empty();
        set.words[id / WORD_BITS] = 1usize << (id % WORD_BITS);
        set
    }

    /// 判断逻辑 CPU 是否为成员；编译范围以外永远没有成员。
    pub fn contains(self, id: usize) -> bool {
        id < MAX_CPUS && self.words[id / WORD_BITS] & (1usize << (id % WORD_BITS)) != 0
    }

    /// 登记编译范围内的逻辑 CPU。
    pub fn insert(&mut self, id: usize) {
        assert!(id < MAX_CPUS);
        self.words[id / WORD_BITS] |= 1usize << (id % WORD_BITS);
    }

    /// 移除编译范围内的逻辑 CPU。
    #[cfg(feature = "ktest")]
    pub fn remove(&mut self, id: usize) {
        assert!(id < MAX_CPUS);
        self.words[id / WORD_BITS] &= !(1usize << (id % WORD_BITS));
    }

    /// 返回两集合的并集。
    #[cfg(feature = "ktest")]
    pub fn union(mut self, other: Self) -> Self {
        for (word, rhs) in self.words.iter_mut().zip(other.words) {
            *word |= rhs;
        }
        self
    }

    /// 返回两集合的共同成员。
    pub fn intersection(mut self, other: Self) -> Self {
        for (word, rhs) in self.words.iter_mut().zip(other.words) {
            *word &= rhs;
        }
        self
    }

    /// 返回只属于当前集合的成员。
    #[cfg(feature = "ktest")]
    pub fn difference(mut self, other: Self) -> Self {
        for (word, rhs) in self.words.iter_mut().zip(other.words) {
            *word &= !rhs;
        }
        self
    }

    /// 判断集合是否没有成员。
    pub fn is_empty(self) -> bool {
        self.words.iter().all(|word| *word == 0)
    }

    /// 按逻辑编号升序迭代，不分配存储。
    pub fn iter(self) -> CpuSetIter {
        CpuSetIter { set: self, word: 0 }
    }

    /// 返回 Linux unsigned-long 位图所占的编译字节数。
    pub const fn byte_len() -> usize {
        WORDS * core::mem::size_of::<usize>()
    }

    /// 导入有限字节，短输入补零，额外尾部不消费。
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut set = Self::empty();
        for (index, byte) in bytes.iter().take(Self::byte_len()).enumerate() {
            set.words[index / core::mem::size_of::<usize>()] |=
                usize::from(*byte) << ((index % core::mem::size_of::<usize>()) * u8::BITS as usize);
        }
        set.intersection(Self::prefix(MAX_CPUS))
    }

    /// 只写调用者提供的内部前缀，用户更长缓冲尾部由 ABI 边界保留。
    pub fn write_bytes(self, bytes: &mut [u8]) {
        assert!(bytes.len() <= Self::byte_len());
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = (self.words[index / core::mem::size_of::<usize>()]
                >> ((index % core::mem::size_of::<usize>()) * u8::BITS as usize))
                as u8;
        }
    }
}

pub struct CpuSetIter {
    set: CpuSet,
    word: usize,
}

impl Iterator for CpuSetIter {
    type Item = usize;

    /// 每次消费最低成员位，空字只跳过一次。
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
/// 单调测试观察的逐字原子集合；不用于事务性的调度许可。
pub(crate) struct AtomicCpuSet {
    words: [AtomicUsize; WORDS],
}

#[cfg(feature = "ktest")]
impl AtomicCpuSet {
    /// 静态构造空观察集合。
    pub const fn new() -> Self {
        Self {
            words: [const { AtomicUsize::new(0) }; WORDS],
        }
    }

    /// 原子登记本次真实执行 CPU，不改变其它字的观察。
    pub fn insert(&self, id: usize, ordering: Ordering) {
        assert!(id < MAX_CPUS);
        self.words[id / WORD_BITS].fetch_or(1usize << (id % WORD_BITS), ordering);
    }

    /// 逐字读取单调观察，精确比较须先停止所有生产者。
    pub fn load(&self, ordering: Ordering) -> CpuSet {
        CpuSet {
            words: core::array::from_fn(|word| self.words[word].load(ordering)),
        }
    }

    /// 逐字取走观察集合；并发写入由各字的原子顺序决定归属。
    pub fn take(&self, ordering: Ordering) -> CpuSet {
        CpuSet {
            words: core::array::from_fn(|word| self.words[word].swap(0, ordering)),
        }
    }
}
