//! CPU 集合位图（`CpuSet` 与 `AtomicCpuSet`）的集合代数、跨字边界、小端字节 ABI 编解码与原子操作自检。
//!
//! # 模块职责与架构定位
//! 本模块属于 `ktest` 启动期（`Boot` 上下文）测试套件，用于验证 `crate::cpu::CpuSet` 和
//! `crate::cpu::AtomicCpuSet` 在不同核心数量边界下的位图运算与用户态 `sched_getaffinity` /
//! `sched_setaffinity` 小端字节序列转换语义。
//! - **上游调用者**：`ktest` 启动期测试运行器。
//! - **下游被测模块**：`crate::cpu::CpuSet`、`crate::cpu::AtomicCpuSet`、`crate::boot::MAX_CPUS`。
//!
//! # 核心机制与位图内存布局
//! ```text
//! CpuSet 内部按 64 位字（usize / u64）数组存储最多 MAX_CPUS 个逻辑核位：
//!
//! Word 0 (CPU 0..63)                    Word 1 (CPU 64..127)
//! +--------+--------+-----+--------+   +--------+--------+-----+---------+
//! | Byte 0 | Byte 1 | ... | Byte 7 |   | Byte 8 | Byte 9 | ... | Byte 15 |
//! |  7..0  | 15..8  |     | 63..56 |   | 71..64 | 79..72 |     | 127..120|
//! +--------+--------+-----+--------+   +--------+--------+-----+---------+
//!
//! 小端字节序 ABI 映射规则：
//! - 逻辑核 `id` 对应字节下标 `id / 8` 中的第 `id % 8` 位（掩码 `1u8 << (id % 8)`）。
//! - `from_bytes` 导入短切片时高位自动补 0；导入超长切片时忽略超出 `MAX_CPUS` 的尾部字节。
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本模块在单核启动期栈上运行，不涉及锁；`AtomicCpuSet` 通过逐字原子指令（`fetch_or` / `load` / `swap`）提供无锁并发位图更新与提取。

use crate::boot::MAX_CPUS;
use crate::cpu::{AtomicCpuSet, CpuSet};
use core::sync::atomic::Ordering;

#[::ktest::suite(context = "Boot", tags("boot", "cpu"))]
pub(crate) mod cases {
    use super::*;

    /// 测试目标：
    /// 验证 `CpuSet::prefix` 在空集、单核、64 位整字边界（63/64/65）、双字边界（127/128）及 `MAX_CPUS` 下的
    /// 升序迭代、逐个插入等价性以及差集（`difference`）、交集（`intersection`）、并集（`union`）代数恒等式。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：遍历候选核心数 `count`（跳过超出编译期 `MAX_CPUS` 的项），构造 `prefix = CpuSet::prefix(count)`；
    /// 2. When：对比 `prefix.iter()` 与 `0..count`、逐位 `insert` 构造的集合，并针对非空集合剥离末位 `count - 1` 计算差集、交集和并集；
    /// 3. Then：迭代器严格按升序产出 `0..count` 且不含越界项 `count`，逐位插入结果与 `prefix` 相等，且集合差、交、并运算结果完全符合集合代数定律。
    #[test]
    fn prefix_and_algebra() {
        for count in [0, 1, 63, 64, 65, 127, 128, MAX_CPUS] {
            if MAX_CPUS < count {
                continue;
            }
            let prefix = CpuSet::prefix(count);
            assert!(prefix.iter().eq(0..count));
            assert!(!prefix.contains(count));

            let mut inserted = CpuSet::empty();
            for id in 0..count {
                inserted.insert(id);
            }
            assert_eq!(prefix, inserted);

            if count == 0 {
                continue;
            }
            let last = CpuSet::single(count - 1);
            inserted.remove(count - 1);
            assert_eq!(prefix.difference(last), inserted);
            assert_eq!(prefix.intersection(last), last);
            assert_eq!(inserted.union(last), prefix);
        }
    }

    /// 测试目标：
    /// 验证稀疏 `CpuSet` 与小端字节切片之间的双向编解码（`write_bytes` / `from_bytes`），包括缓冲区越界保护、短切片零扩展与超长切片尾部截断。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：构造包含跨字节与跨 64 位字边界离散核号（`0, 7, 31, 32, 63, 64, 65, 127, MAX_CPUS - 1`）的 `CpuSet`，准备末尾带 8 字节哨兵值 `0xa5` 的字节数组；
    /// 2. When：调用 `write_bytes` 写出前半段，再用不同截断长度 `0..=byte_len()`、空切片及超长全 `0xff` 切片调用 `from_bytes`；
    /// 3. Then：写出操作不覆写尾部 `0xa5` 哨兵，每个比特位精确对应 `bytes[id / 8] & (1 << (id % 8))`，完整或超长切片往返还原原集合，短切片仅保留前 `length * 8` 位内的成员。
    #[test]
    fn sparse_byte_roundtrip() {
        let mut set = CpuSet::empty();
        for id in [0, 7, 31, 32, 63, 64, 65, 127, MAX_CPUS - 1] {
            if id < MAX_CPUS {
                set.insert(id);
            }
        }

        let mut bytes = [0xa5u8; CpuSet::byte_len() + 8];
        set.write_bytes(&mut bytes[..CpuSet::byte_len()]);
        assert!(bytes[CpuSet::byte_len()..].iter().all(|byte| *byte == 0xa5));
        for id in 0..MAX_CPUS {
            assert_eq!((bytes[id / 8] & (1u8 << (id % 8))) != 0, set.contains(id));
        }
        assert_eq!(CpuSet::from_bytes(&bytes), set);

        for length in 0..=CpuSet::byte_len() {
            let imported = CpuSet::from_bytes(&bytes[..length]);
            for id in 0..MAX_CPUS {
                assert_eq!(imported.contains(id), id < length * 8 && set.contains(id));
            }
        }

        assert_eq!(CpuSet::from_bytes(&[]), CpuSet::empty());
        assert_eq!(
            CpuSet::from_bytes(&[0xff; CpuSet::byte_len() + 8]),
            CpuSet::prefix(MAX_CPUS)
        );
    }

    /// 测试目标：
    /// 验证 `AtomicCpuSet` 在逆序逐位插入全部逻辑核后，`load` 能完整读取全集，且 `take` 能原子取走全集并将内部状态清零。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：新建空的 `AtomicCpuSet`，按 `(0..MAX_CPUS).rev()` 逆序调用 `insert` 置位所有核心；
    /// 2. When：依次调用 `observed.load`、`observed.take` 以及第二次 `observed.load`；
    /// 3. Then：首次 `load` 与 `take` 均返回 `CpuSet::prefix(MAX_CPUS)`，取走后的第二次 `load` 返回 `CpuSet::empty()`。
    #[test]
    fn atomic_words() {
        let observed = AtomicCpuSet::new();
        for id in (0..MAX_CPUS).rev() {
            observed.insert(id, Ordering::Release);
        }
        assert_eq!(observed.load(Ordering::Acquire), CpuSet::prefix(MAX_CPUS));
        assert_eq!(observed.take(Ordering::AcqRel), CpuSet::prefix(MAX_CPUS));
        assert_eq!(observed.load(Ordering::Acquire), CpuSet::empty());
    }
}
