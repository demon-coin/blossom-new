//! 实际CPU栈拥有、真实页耗尽回滚及未启用栈的归还。

use crate::cache::ktest::HeapPressure;
use crate::cpu::stack::{prepare_slots, StackSlot};
use crate::ktest::execution_services::{baseline, reclaimed};
use crate::sync::PreemptGuard;
use crate::{cpu, mm};
use ::ktest::{TestContext, TestError, TestResult};
use core::cell::Cell;
use core::sync::atomic::Ordering;

/// 对两个从未交给CPU的局部槽执行真实准备和归还，核对独立页范围与完整资源账。
///
/// # Safety
/// 两槽必须互异、未准备且从未交给任何CPU；调用期间须由当前Task独占，
/// 无其他准备者、归还者或栈访问者，成功拥有的页只由本函数归还。
unsafe fn prepare_and_release(context: &mut TestContext<'_>, slots: &[StackSlot; 2]) -> TestResult {
    let before = baseline();
    let preempt = PreemptGuard::new();
    // SAFETY: 调用者保证两个不同局部槽尚未准备、从未交给CPU且无并发访问；
    // 有限数组迭代不重复，MM已由Thread生产启动路径完成初始化。
    unsafe { prepare_slots(slots.iter()) }
        .map_err(|_| TestError("AP stack retry allocation failed"))?;
    let allocations = [slots[0].allocation(), slots[1].allocation()];
    let tops = [slots[0].top(), slots[1].top()];
    let allocated = mm::stats();

    for slot in slots {
        // SAFETY: 两槽只在本函数准备，从未交给CPU或用于栈访问；
        // 当前Task独占归还权，准备已结束且不存在并发准备或重复归还。
        unsafe { slot.release_unstarted() };
    }
    let empty = slots
        .iter()
        .all(|slot| slot.top() == 0 && slot.allocation().is_none());
    let after = mm::stats();
    drop(preempt);

    context.diagnostic(format_args!(
        "ap-stacks prepared={:?} tops={:?} before={:?} allocated={:?} released={:?}",
        allocations, tops, before.0, allocated, after
    ));
    let [(first, first_bytes), (second, second_bytes)] =
        allocations.map(|allocation| allocation.expect("prepared stack has no page owner"));
    assert_eq!([first_bytes, second_bytes], [64 * 1024; 2]);
    assert_eq!([first % mm::PAGE_SIZE, second % mm::PAGE_SIZE], [0; 2]);
    assert_eq!(tops, [first + first_bytes, second + second_bytes]);
    assert!(first + first_bytes <= second || second + second_bytes <= first);
    let mut expected = before.0;
    expected.buddy_free_pages -= (first_bytes + second_bytes) / mm::PAGE_SIZE;
    context.expect_eq(allocated, expected);
    assert!(empty);
    context.expect_eq(after, before.0);
    reclaimed(before)
}

#[::ktest::suite(
    context = "Thread",
    tags("boot", "cpu", "ap-stacks"),
    timeout_ms = 30000
)]
pub(crate) mod cases {
    use super::*;

    /// 只遍历本次实际CPU，核对BSP静态栈、AP真实页拥有和启动时已记录的栈内探针。
    #[test]
    fn live_allocations(context: &mut TestContext<'_>) {
        assert_eq!(cpu::STACK_SIZE, 64 * 1024);
        let bsp_start = core::ptr::addr_of!(cpu::BOOT_STACK) as usize;
        for id in 0..cpu::count() {
            let local = cpu::get(id);
            assert!(local.online.load(Ordering::Acquire));
            let top = local.stack_top();
            let (start, bytes) = if id == 0 {
                assert_eq!(local.stack_allocation(), None);
                assert_eq!(top, bsp_start + 64 * 1024);
                (bsp_start, 64 * 1024)
            } else {
                let (start, bytes) = local
                    .stack_allocation()
                    .expect("online AP has no stack page owner");
                assert_eq!(bytes, 64 * 1024);
                assert_eq!(start % mm::PAGE_SIZE, 0);
                assert_eq!(top, start + bytes);
                (start, bytes)
            };
            let probe = local.boot_stack_probe.load(Ordering::Acquire);
            assert!(start <= probe && probe < start + bytes);
            for other in 0..id {
                let other_top = cpu::get(other).stack_top();
                let other_start = other_top - 64 * 1024;
                assert!(top <= other_start || other_top <= start);
            }
            context.diagnostic(format_args!(
                "ap-stacks live cpu={} allocation={:?} range={:#x}..{:#x} probe={:#x}",
                id,
                local.stack_allocation(),
                start,
                top,
                probe
            ));
        }
    }

    /// 真实耗尽后只放回一个栈块，观察第一槽已拥有但未发表、第二槽OOM及全量回滚，
    /// 恢复压力后在同一未启动集合重新准备并归还，不用失败注入替代allocator错误。
    #[test]
    fn partial_prepare_oom(context: &mut TestContext<'_>) -> TestResult {
        let before = baseline();
        let slots = [StackSlot::new(0), StackSlot::new(0)];
        let partial = Cell::new(None);
        let mut pressure = HeapPressure::prepare()?;
        let reserved = mm::alloc_pages((cpu::STACK_SIZE / mm::PAGE_SIZE).ilog2() as usize, false)
            .ok_or(TestError("AP stack OOM reserve unavailable"))?;

        let preempt = PreemptGuard::new();
        let evidence = pressure.exhaust();
        drop(reserved);
        let rollback_before = mm::stats();
        let candidates = slots.iter().inspect(|slot| {
            if core::ptr::eq(*slot, &slots[1]) {
                if let Some(allocation) = slots[0].allocation() {
                    partial.set(Some((allocation, slots[0].top(), slots[1].top())));
                }
            }
        });
        // SAFETY: 有限数组只含两个不同的局部未启动槽，从未交给CPU且无外部引用；
        // 当前Task禁止抢占并独占准备，inspect只观察拥有和栈顶，不修改槽或页内容。
        let failed = unsafe { prepare_slots(candidates) }.is_err();
        let rollback_after = mm::stats();
        let empty = slots
            .iter()
            .all(|slot| slot.top() == 0 && slot.allocation().is_none());
        drop(pressure);
        drop(preempt);

        context.diagnostic(format_args!(
            "ap-stacks OOM exhausted={:?} partial={:?} failed={} empty={} before={:?} after={:?}",
            evidence,
            partial.get(),
            failed,
            empty,
            rollback_before,
            rollback_after
        ));
        assert_eq!(evidence, (true, true));
        let ((start, bytes), first_top, second_top) = partial
            .get()
            .expect("first stack allocation was not observed before OOM");
        assert_eq!(bytes, 64 * 1024);
        assert_eq!(start % mm::PAGE_SIZE, 0);
        assert_eq!([first_top, second_top], [0; 2]);
        assert!(failed);
        assert!(empty);
        context.expect_eq(rollback_after, rollback_before);
        reclaimed(before)?;

        // SAFETY: 上述真实失败已清空两个局部槽；从创建至今从未交给CPU，
        // 无并发准备、归还或栈访问，同一当前Task继续独占该有限集合。
        unsafe { prepare_and_release(context, &slots) }
    }

    /// 正常准备两个真实独立栈后，仅在它们从未交给CPU的条件下释放拥有并恢复资源。
    #[test]
    fn unstarted_release(context: &mut TestContext<'_>) -> TestResult {
        let slots = [StackSlot::new(0), StackSlot::new(0)];
        // SAFETY: 两个不同槽刚在当前Task栈上构造，尚未准备且从未交给任何CPU；
        // 没有外部引用、并发准备者、归还者或栈访问者，调用期间由当前Task独占。
        unsafe { prepare_and_release(context, &slots) }
    }
}
