//! 正文展开期间 fixture 保真实资源，清理结束后才公布预期 panic 结果。

use crate::mm::ktest::{HeapFixture, HeapObject, PageFixture, PressureFixture, SlabFixture};
use crate::mm::{self, PAGE_SIZE};
use core::alloc::Layout;
use ktest::TestContext;

#[ktest::suite(fixture = PageFixture, tags("mm", "lifecycle"), timeout_ms = 30000)]
mod pages {
    use super::*;
    /// 原页唯一权由 body 外 fixture 持有，正文 panic 后仍安全按原阶归还。
    #[test]
    #[should_panic(expected = "owned page body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut PageFixture) {
        f.held[0] = Some(mm::alloc_pages(1, true).unwrap());
        assert_eq!(f.held[0].as_ref().unwrap().len(), PAGE_SIZE * 2);
        panic!("owned page body unwind");
    }
}
#[ktest::suite(fixture = SlabFixture, tags("mm", "lifecycle"), timeout_ms = 30000)]
mod objects {
    use super::*;
    /// 原 SLUB 对象在正文展开时保持有效，真实归还及 reclaim 由 fixture 消费。
    #[test]
    #[should_panic(expected = "slab body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut SlabFixture) {
        f.allocate(0);
        f.held[0].as_ref().unwrap().check();
        panic!("slab body unwind");
    }
}
#[ktest::suite(fixture = HeapFixture, tags("mm", "lifecycle"), timeout_ms = 30000)]
mod heap {
    use super::*;
    /// 原布局和指针不会因正文 panic 丢失，真实 GlobalAlloc 归还后核六域。
    #[test]
    #[should_panic(expected = "heap body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut HeapFixture) {
        f.held[0] = Some(HeapObject::allocate(
            Layout::from_size_align(8193, 64).unwrap(),
            false,
        ));
        f.held[0].as_mut().unwrap().fill(0x6d);
        panic!("heap body unwind");
    }
}
#[ktest::suite(fixture = PressureFixture, tags("mm", "lifecycle", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod pressure {
    use super::*;
    /// 真实耗尽后的正文 panic 不依赖堆构造异常，所有压力页由 fixture 保至 cleanup。
    #[test]
    #[should_panic(expected = "pressure body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut PressureFixture) {
        f.exhaust();
        assert!(!f.pressure.is_empty());
        panic!("pressure body unwind");
    }
}
