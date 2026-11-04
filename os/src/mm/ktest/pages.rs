//! 原阶拥有、PCP 边界与真实页压力；每参数一个固定核原 Task。

use crate::mm::ktest::{
    fixed::{FixedPages, FixedPressure},
    free,
};
use crate::mm::{self, buddy, page::PageKind, pcp, PAGE_SIZE};
use core::hint::black_box;
use ktest::{TestContext, TestResult};

#[ktest::suite(fixture = FixedPages, tags("mm", "pages"), timeout_ms = 30000)]
mod owned {
    use super::*;
    /// 每阶独立核原阶、物理对齐与整块可用页分类；zeroed 检查全部字节。
    #[test]
    #[case("order0", 0, false)]
    #[case("order1", 1, false)]
    #[case("order2", 2, false)]
    #[case("order3", 3, false)]
    #[case("order4", 4, false)]
    #[case("order5", 5, false)]
    #[case("order6", 6, false)]
    #[case("order7", 7, false)]
    #[case("order8", 8, false)]
    #[case("order9", 9, false)]
    #[case("order10", 10, false)]
    #[case("zero0", 0, true)]
    #[case("zero1", 1, true)]
    #[case("zero2", 2, true)]
    #[case("zero3", 3, true)]
    #[case("zero4", 4, true)]
    #[case("zero5", 5, true)]
    #[case("zero6", 6, true)]
    #[case("zero7", 7, true)]
    #[case("zero8", 8, true)]
    #[case("zero9", 9, true)]
    #[case("zero10", 10, true)]
    fn original_order(
        _: &mut TestContext<'_>,
        fixture: &mut FixedPages,
        order: usize,
        zeroed: bool,
    ) -> TestResult {
        fixture.run(move |f| {
            mm::drain_all_pages();
            let initial = free();
            f.held[0] =
                Some(mm::alloc_pages(order, zeroed).expect("owned pages allocation failed"));
            let pages = f.held[0].as_mut().unwrap();
            let length = PAGE_SIZE << order;
            let start = pages.phys_addr();
            assert_eq!(pages.order(), order);
            assert_eq!(pages.len(), length);
            assert_eq!(start % length, 0);
            for pfn in start / PAGE_SIZE..start / PAGE_SIZE + (1 << order) {
                assert_eq!(mm::pages().get(pfn).unwrap().kind(), PageKind::Available);
            }
            if zeroed {
                // SAFETY: alloc_pages 已初始化完整 zeroed 块，owning 槽在整个检查期间保活。
                let bytes =
                    unsafe { core::slice::from_raw_parts(pages.as_mut_ptr().cast::<u8>(), length) };
                assert!(bytes.iter().all(|&byte| byte == 0));
            } else {
                // SAFETY: owning 槽独占真实块，length 为原完整长度，尚未发布给其他核。
                unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0xa5, length) };
            }
            assert_eq!(free(), initial - (1 << order));
            drop(f.held[0].take());
            assert_eq!(free(), initial);
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), initial);
        })
    }
    /// 同时 live 的全部阶不重叠，累计页数准确，逆序 Drop 恢复原页池。
    #[test]
    fn mixed_live(_: &mut TestContext<'_>, fixture: &mut FixedPages) -> TestResult {
        fixture.run(|f| {
            mm::drain_all_pages();
            let initial = free();
            let mut allocated = 0;
            for order in 0..=buddy::MAX_ORDER {
                f.held[order] = Some(mm::alloc_pages(order, false).unwrap());
                let pages = f.held[order].as_ref().unwrap();
                for other in f.held[..order].iter().flatten() {
                    assert!(
                        pages.phys_addr() + pages.len() <= other.phys_addr()
                            || other.phys_addr() + other.len() <= pages.phys_addr()
                    );
                }
                allocated += 1 << order;
                assert_eq!(free(), initial - allocated);
            }
            for slot in f.held.iter_mut().rev() {
                drop(slot.take());
            }
            assert_eq!(free(), initial);
        })
    }
    /// 超阶与极大阶分别拒绝，失败不改变页账务。
    #[test]
    #[case("over_max", buddy::MAX_ORDER + 1, false)]
    #[case("usize_max_zeroed", usize::MAX, true)]
    fn over_order(
        _: &mut TestContext<'_>,
        fixture: &mut FixedPages,
        order: usize,
        zeroed: bool,
    ) -> TestResult {
        fixture.run(move |_| {
            let initial = free();
            assert!(black_box(mm::alloc_pages(order, zeroed)).is_none());
            assert_eq!(free(), initial);
        })
    }
    /// high 前不归还，恰到 high 只 drain 一个有限 batch。
    #[test]
    fn high_batch(_: &mut TestContext<'_>, fixture: &mut FixedPages) -> TestResult {
        fixture.run(|f| {
            let (batch, high) = pcp::parameters();
            assert!(batch < high && high <= f.held.len());
            mm::drain_all_pages();
            let initial = free();
            for slot in &mut f.held[..high] {
                *slot = Some(mm::alloc_pages(0, false).unwrap());
            }
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), initial - high);
            for slot in &mut f.held[..high - 1] {
                drop(slot.take());
            }
            assert_eq!(pcp::cpu_cached_pages(0), high - 1);
            assert_eq!(buddy::free_pages(), initial - high);
            drop(f.held[high - 1].take());
            assert_eq!(pcp::cpu_cached_pages(0), high - batch);
            assert_eq!(buddy::free_pages(), initial - high + batch);
        })
    }
    /// 单核有限流每轮都核 live 唯一、high 上界及停止后直接 drain。
    #[test]
    fn local_drain_stream(_: &mut TestContext<'_>, fixture: &mut FixedPages) -> TestResult {
        fixture.run(|f| {
            let (_, high) = pcp::parameters();
            mm::drain_all_pages();
            let initial = free();
            for _ in 0..16 {
                for index in 0..high {
                    f.held[index] = Some(mm::alloc_pages(0, false).unwrap());
                    let pfn = f.held[index].as_ref().unwrap().phys_addr();
                    for other in f.held[..index].iter().flatten() {
                        assert_ne!(other.phys_addr(), pfn);
                    }
                }
                assert_eq!(free(), initial - high);
                for slot in &mut f.held[..high] {
                    drop(slot.take());
                    assert!(pcp::cpu_cached_pages(0) < high);
                }
                mm::drain_all_pages();
                assert_eq!(pcp::cached_pages(), 0);
                assert_eq!(buddy::free_pages(), initial);
            }
        })
    }
}

#[ktest::suite(fixture = FixedPressure, tags("mm", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod pressure {
    use super::*;
    /// 耗尽实际页后真实 None，释放唯一压力句柄后由 fixture 核六域。
    #[test]
    fn global_oom(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            f.exhaust();
            assert!(!f.pressure.is_empty());
            assert!(black_box(mm::alloc_pages(0, false)).is_none());
        })
    }
    /// 两个非相邻实际页不能拼 order1，失败保原自由页数。
    #[test]
    fn fragmented_order1(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            let (a, b) = f.nonadjacent();
            f.exhaust();
            drop(f.samples[a].take());
            drop(f.samples[b].take());
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), 2);
            assert_eq!(pcp::cached_pages(), 0);
            assert!(black_box(mm::alloc_pages(1, false)).is_none());
            assert_eq!(free(), 2);
        })
    }
    /// 真 OOM 后两页不足整批仍补充，原 PFN 逐页返回且无重复唯一权。
    #[test]
    fn partial_refill(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            let (batch, _) = pcp::parameters();
            assert!(2 < batch);
            let (a, b) = f.nonadjacent();
            let expected = [
                f.samples[a].as_ref().unwrap().phys_addr(),
                f.samples[b].as_ref().unwrap().phys_addr(),
            ];
            f.exhaust();
            assert!(black_box(mm::alloc_pages(0, false)).is_none());
            drop(f.samples[a].take());
            drop(f.samples[b].take());
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), 2);
            assert_eq!(pcp::cached_pages(), 0);
            assert!(black_box(mm::alloc_pages(1, false)).is_none());
            assert_eq!(free(), 2);
            f.samples[a] = Some(mm::alloc_pages(0, false).unwrap());
            assert_eq!(1 + pcp::cpu_cached_pages(0), 2);
            assert_eq!(buddy::free_pages(), 0);
            f.samples[b] = Some(mm::alloc_pages(0, false).unwrap());
            let actual = [
                f.samples[a].as_ref().unwrap().phys_addr(),
                f.samples[b].as_ref().unwrap().phys_addr(),
            ];
            assert_ne!(actual[0], actual[1]);
            assert!(actual.iter().all(|p| expected.contains(p)));
            assert_eq!(free(), 0);
            assert!(black_box(mm::alloc_pages(0, false)).is_none());
        })
    }
}

#[ktest::suite(fixture = FixedPages, tags("mm", "irq", "locks"), timeout_ms = 30000)]
mod lock_state {
    use super::*;
    /// 独立开/关 IRQ 参数核 try_lock 成功、竞争失败及每层析构的原 IRQ 状态。
    #[test]
    #[case("enabled", true)]
    #[case("disabled", false)]
    fn try_lock_restore(
        _: &mut TestContext<'_>,
        fixture: &mut FixedPages,
        enabled: bool,
    ) -> TestResult {
        fixture.run(move |_| check_try_lock_restore(enabled))
    }
}

/// 在当前真实 CPU 复用原锁断言；短不可抢占段保住私有锁的 IRQ 状态来源。
pub(super) fn check_try_lock_restore(enabled: bool) {
    use crate::{
        arch,
        sync::{IrqSaveGuard, PreemptGuard, SpinLock},
    };
    let _preempt = PreemptGuard::new();
    let lock = SpinLock::new(0usize);
    assert!(arch::irq_enabled());
    let disabled = if enabled {
        None
    } else {
        Some(IrqSaveGuard::new())
    };
    let guard = lock.try_lock().unwrap();
    assert!(!arch::irq_enabled());
    drop(guard);
    assert_eq!(arch::irq_enabled(), enabled);
    let saved = arch::irq_save();
    // SAFETY: saved 来自当前核，私有锁无 IRQ/异核用户，短 preempt guard 保同核。
    unsafe { arch::irq_restore(saved) };
    let owner = lock.lock();
    // SAFETY: 私有锁仅本 Task 尝试非阻塞取得，短不可抢占段不睡眠，退出 owner 前重新关 IRQ。
    unsafe { arch::irq_restore(saved) };
    assert!(lock.try_lock().is_none());
    assert_eq!(arch::irq_enabled(), enabled);
    arch::irq_save();
    drop(owner);
    assert_eq!(arch::irq_enabled(), enabled);
    drop(disabled);
    assert!(arch::irq_enabled());
}
