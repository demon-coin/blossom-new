//! 直接消费生产 CPU 集合，覆盖整字边界、第二字与 ABI 字节长度。

use crate::boot::MAX_CPUS;
use crate::cpu::{AtomicCpuSet, CpuSet};
use core::sync::atomic::Ordering;

#[::ktest::suite(context = "Boot", tags("boot", "cpu"))]
pub(crate) mod cases {
    use super::*;

    /// 空前缀、整字及编译末项均由同一集合算法产生，升序迭代不得丢失高字。
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

    /// 稀疏位图按真实小端 ABI 写出，短输入补零且额外尾部不导入。
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

    /// 逐字原子观察在所有写者结束后保持完整集合，取走后全部字清空。
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
