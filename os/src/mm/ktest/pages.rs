//! 物理页分配器阶数契约、每 CPU 缓存（PCP）水位边界、内存压力（OOM/碎片）与自旋锁中断恢复单元测试。
//!
//! ## 模块职责与架构定位
//! 本模块验证物理页分配层（`mm::alloc_pages`、`OwnedPages`、`pcp`、`buddy`）在单核固定任务环境下的核心行为，
//! 同时为多核测试模块（`mm::ktest::smp`）提供可跨核复用的自旋锁中断状态校验函数 [`check_try_lock_restore`]：
//! - **上游驱动者**：`ktest` 测试框架通过 `FixedPages` 与 `FixedPressure` 夹具在固定 CPU0 任务中运行 `owned`、`pressure` 与 `lock_state` 套件。
//! - **被测与下游依赖**：直接调用 `mm::alloc_pages`、`mm::drain_all_pages`、`mm::pages`、`pcp`、`buddy` 以及 `sync::SpinLock`。
//!
//! ## 核心机制与 PCP 水位/补充边界
//! 零阶（Order-0）物理页释放与补充遵循每 CPU 缓存（PCP）的高水位批量排空与非满批回退机制：
//!
//! ```text
//! 单核 PCP 缓存计数 (count) 变化边界：
//!   释放前 count == high - 1 ──► 再释放第 high 张页 ──► 触发批量排空 batch 张页至 Buddy
//!                                                        └─ 结果：PCP 剩余 high - batch，Buddy 增加 batch
//!
//! 内存耗尽后非满批补充 (Partial Refill)：
//!   全局仅存 2 张非相邻零阶页 (< batch)
//!     ├─ 请求 Order-1 (2 连续页) ──► 因物理不相邻无法合并，返回 None
//!     └─ 请求 Order-0 (单页)     ──► PCP 虽不足 batch 仍接管全部 2 张页：1 张返回调用方，1 张留存 PCP
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `owned` 与 `pressure` 套件在普通任务上下文中运行，所有分配与释放均通过标准 API 自动管理内部自旋锁。
//! - [`check_try_lock_restore`] 在 `PreemptGuard` 保护下仅操作栈上私有的 `SpinLock<usize>`（无中断处理函数或其他 CPU 访问该锁），
//!   用于精确观测 `try_lock` 成功、锁竞争失败及守卫析构对本核硬件中断使能位（IRQ Enable）的恢复行为。

use crate::mm::ktest::{
    fixed::{FixedPages, FixedPressure},
    free,
};
use crate::mm::{self, buddy, page::PageKind, pcp, PAGE_SIZE};
use core::hint::black_box;
use ktest::{TestContext, TestResult};

/// 连续物理页块各阶分配对齐、清零语义与每 CPU 零阶页缓存（PCP）高水位边界测试套件。
#[ktest::suite(fixture = FixedPages, tags("mm", "pages"), timeout_ms = 30000)]
mod owned {
    use super::*;
    /// 逐阶验证 `0..=MAX_ORDER` 的物理页块分配、自然对齐、页描述符分类、清零/读写及释放账本守恒。
    ///
    /// - **测试目标**：确认伙伴系统与 PCP 对所有合法阶数返回的物理块满足 $2^{\text{order}} \times \text{PAGE\_SIZE}$ 自然对齐，且 `zeroed` 参数准确控制整块清零。
    /// - **场景步骤**：先排空全核 PCP 记录初始空闲页数；分配指定 `(order, zeroed)` 页块，检查阶数、字节长度、物理地址对齐及每一帧 `PageDesc` 的 `PageKind::Available` 分类；若 `zeroed` 为真则校验全块字节为 `0`，否则写满 `0xa5` 验证可写性；最后释放页块并排空 PCP 复核伙伴系统空闲页数。
    /// - **预期结果**：对齐与页分类全部符合契约，读写/全零断言通过，释放与排空后伙伴系统空闲页数精确回到初始值。
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
                // SAFETY:
                // 1. 上方 `mm::alloc_pages(order, true)` 已将整段 `length` 字节清零初始化；
                // 2. 该物理页块由 `f.held[0]` 独占持有且仅在当前任务中读取，切片生命周期内无任何并发修改。
                let bytes =
                    unsafe { core::slice::from_raw_parts(pages.as_mut_ptr().cast::<u8>(), length) };
                assert!(bytes.iter().all(|&byte| byte == 0));
            } else {
                // SAFETY:
                // 1. `f.held[0]` 唯一拥有刚分配的连续物理页块，`length == PAGE_SIZE << order` 精确等于该块的字节长度；
                // 2. 该内存块尚未共享给任何其他任务或设备，整段直接映射虚拟地址可安全独占写入。
                unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0xa5, length) };
            }
            assert_eq!(free(), initial - (1 << order));

            drop(f.held[0].take());
            assert_eq!(free(), initial);
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), initial);
        })
    }
    /// 验证 `0..=MAX_ORDER` 全部阶数的物理页块同时存活时物理区间互不重叠，且逆序释放后空闲页完全恢复。
    ///
    /// - **测试目标**：检验伙伴系统在连续切分出所有阶数的块时不会产生地址别名（重叠分配）或账本偏差。
    /// - **场景步骤**：排空 PCP 后依次分配 `0..=MAX_ORDER` 各阶页块，每次与此前已持有的所有块比对物理起止区间并核对累计扣减的空闲页数；随后按逆序逐一释放全部页块。
    /// - **预期结果**：任意两个存活页块的物理地址区间均不交叠，每步空闲页减少量精确等于 $\sum 2^{\text{order}}$，全部释放后空闲页总数恢复初始值。
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
    /// 验证超过 `MAX_ORDER` 的非法阶数（含 `usize::MAX` 溢出边界）会被安全拒绝且不影响空闲页账本。
    ///
    /// - **测试目标**：确保 `alloc_pages` 对超界 `order` 参数具备完整的防御边界，不会移位溢出或误扣页数。
    /// - **场景步骤**：记录当前可分配空闲页数，通过 `black_box` 调用 `mm::alloc_pages(order, zeroed)` 传入 `MAX_ORDER + 1` 或 `usize::MAX`。
    /// - **预期结果**：调用立即返回 `None`，全局可分配空闲页数保持不变。
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
    /// 验证零阶页释放未达 PCP 高水位（`high`）时不归还伙伴系统，刚达到 `high` 时精确归还一个批次（`batch`）。
    ///
    /// - **测试目标**：确认每 CPU 页缓存（PCP）的高水位滞回阈值与单批次排空步长符合预期。
    /// - **场景步骤**：在 CPU0 上分配 `high` 张零阶页并排空残留缓存；先释放前 `high - 1` 张页观察 PCP 与伙伴系统页数，再释放第 `high` 张页再次观察两者变化。
    /// - **预期结果**：释放 `high - 1` 张后全部留在 CPU0 的 PCP 中，伙伴系统空闲页不增加；释放第 `high` 张后 PCP 缓存变为 `high - batch`，伙伴系统空闲页精确增加 `batch`。
    #[test]
    fn high_batch(_: &mut TestContext<'_>, fixture: &mut FixedPages) -> TestResult {
        fixture.run(|f| {
            let (batch, high) = pcp::parameters();
            assert!(batch < high && high <= f.held.len());
            mm::drain_all_pages();
            // Step 1: 分配 high 张零阶页并排空因批量预取留存在 PCP 中的多余页，使本核 PCP 归零。
            let initial = free();
            for slot in &mut f.held[..high] {
                *slot = Some(mm::alloc_pages(0, false).unwrap());
            }
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), initial - high);

            // Step 2: 释放 high - 1 张页，验证全部滞留在本地 PCP 中，尚未触发向伙伴系统的归还。
            for slot in &mut f.held[..high - 1] {
                drop(slot.take());
            }
            assert_eq!(pcp::cpu_cached_pages(0), high - 1);
            assert_eq!(buddy::free_pages(), initial - high);

            // Step 3: 释放第 high 张页跨过高水位线，验证恰好向伙伴系统批量归还 batch 张页。
            drop(f.held[high - 1].take());
            assert_eq!(pcp::cpu_cached_pages(0), high - batch);
            assert_eq!(buddy::free_pages(), initial - high + batch);
        })
    }
    /// 验证单核循环多轮批量分配、释放与显式排空过程中，存活页帧始终唯一且 PCP 缓存严格受限于 `high` 水位。
    ///
    /// - **测试目标**：检验 PCP 与伙伴系统在多轮反复吞吐和 `drain_all_pages` 交替作用下的页帧去重与账本稳定性。
    /// - **场景步骤**：连续循环 16 轮，每轮分配 `high` 张零阶页并校验物理地址两两不同，逐张释放时检查本核 PCP 缓存始终小于 `high`，随后调用 `drain_all_pages` 排空。
    /// - **预期结果**：每轮存活页帧无一重复，释放过程不突破水位上限，每轮排空后 PCP 缓存归零且伙伴系统空闲页恢复初始基线。
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

/// 全局物理内存耗尽（OOM）与外部碎片条件下的物理页分配及 PCP 补充边界测试套件。
#[ktest::suite(fixture = FixedPressure, tags("mm", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod pressure {
    use super::*;
    /// 验证耗尽系统全部可分配物理页后，继续请求零阶页返回 `None`，且退出时夹具可完整回收所有压力页。
    ///
    /// - **测试目标**：确认伙伴系统与全核 PCP 同时耗尽时的物理内存不足（OOM）返回语义及大规模页释放恢复能力。
    /// - **场景步骤**：调用 `f.exhaust()` 分配并持有全部高阶与零阶空闲页，随后再次调用 `alloc_pages(0, false)`。
    /// - **预期结果**：压力容器非空，OOM 后的分配请求返回 `None`，用例结束时夹具回收全部压力页并恢复六域统计基线。
    #[test]
    fn global_oom(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            f.exhaust();
            assert!(!f.pressure.is_empty());
            assert!(black_box(mm::alloc_pages(0, false)).is_none());
        })
    }
    /// 验证全局仅剩两张物理不相邻的零阶空闲页时，无法合并满足一阶（Order-1，连续两页）分配请求。
    ///
    /// - **测试目标**：确认伙伴系统仅合并满足伙伴对齐且物理相邻的页块，外部碎片导致高阶分配失败时不会丢失空闲页。
    /// - **场景步骤**：先挑出两张物理间距超过一页的样本页，耗尽其余全部内存，再释放这两张样本页并调用 `drain_all_pages` 将其归还至伙伴系统，随后请求 `alloc_pages(1, false)`。
    /// - **预期结果**：伙伴系统持有 2 张零阶空闲页，但 `alloc_pages(1, false)` 返回 `None`，失败后全局空闲页仍为 `2`。
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
    /// 验证全局剩余零阶页数少于单批步长（`2 < batch`）时，PCP 仍能非满批补充并逐页精确分配完所有可用页帧。
    ///
    /// - **测试目标**：检验伙伴系统不足 `batch` 页时的 PCP 部分补充（Partial Refill）路径不会漏页或重复发放同一页帧。
    /// - **场景步骤**：保留两张非相邻样本页后耗尽全系统内存，归还该两张页至伙伴系统；连续两次调用 `alloc_pages(0, false)`，检查首次分配后 PCP 预取剩余页数、两次返回的物理地址集合以及第三次分配结果。
    /// - **预期结果**：首次零阶分配将伙伴系统的 2 张页一并取入（1 张返回调用方，1 张缓存于 PCP），第二次分配返回另一张样本页，两张页地址互异且与原释放样本完全一致，第三次分配因内存再次耗尽返回 `None`。
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

            // Step 1: 耗尽其余全部物理页后归还两张非相邻页至伙伴系统，确认总空闲页仅为 2 且无法分配 Order-1。
            f.exhaust();
            assert!(black_box(mm::alloc_pages(0, false)).is_none());
            drop(f.samples[a].take());
            drop(f.samples[b].take());
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), 2);
            assert_eq!(pcp::cached_pages(), 0);
            assert!(black_box(mm::alloc_pages(1, false)).is_none());
            assert_eq!(free(), 2);

            // Step 2: 连续两次申请零阶页，验证部分补充机制将仅有的 2 张页不重不漏地交付完毕。
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

/// 自旋锁（`SpinLock`）在不同初始硬件中断状态下的 `try_lock` 与析构恢复测试套件。
#[ktest::suite(fixture = FixedPages, tags("mm", "irq", "locks"), timeout_ms = 30000)]
mod lock_state {
    use super::*;
    /// 分别在初始开中断与初始关中断两种状态下，验证 `SpinLock::try_lock` 成功、竞争失败及守卫析构后的本地中断状态。
    ///
    /// - **测试目标**：确保 `SpinLock` 无论在开中断还是关中断上下文中加锁或尝试加锁失败，均严格保持进入前的本地 IRQ 状态不泄漏。
    /// - **场景步骤**：由参数 `enabled` 指定测试起始时是否开启本地中断，委托 [`check_try_lock_restore`] 在固定 CPU0 上依次验证无竞争 `try_lock` 与有竞争 `try_lock`。
    /// - **预期结果**：每次守卫释放及锁竞争失败返回后，硬件中断开关状态均与进入前的 `enabled` 完全一致。
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

/// 在当前物理 CPU 上校验 `SpinLock::try_lock` 成功获取、锁竞争失败及守卫析构全过程的本地中断状态（IRQ）恢复不变量。
///
/// 调用上下文：在普通任务且初始本地中断开启（`arch::irq_enabled() == true`）的上下文中调用；内部使用 `PreemptGuard` 禁止抢占迁核。
/// 参数 `enabled`：为 `true` 时测试在开中断状态下操作自旋锁；为 `false` 时先套一层 `IrqSaveGuard` 测试在关中断状态下嵌套操作自旋锁。
pub(super) fn check_try_lock_restore(enabled: bool) {
    use crate::{
        arch,
        sync::{IrqSaveGuard, PreemptGuard, SpinLock},
    };
    // Step 1: 禁止抢占以固定当前物理 CPU，并根据 enabled 参数构造初始开中断或关中断环境。
    let _preempt = PreemptGuard::new();
    let lock = SpinLock::new(0usize);
    assert!(arch::irq_enabled());
    let disabled = if enabled {
        None
    } else {
        Some(IrqSaveGuard::new())
    };
    // Step 2: 验证无竞争时 try_lock 成功关闭本地中断，释放守卫后恢复至 enabled 状态。
    let guard = lock.try_lock().unwrap();
    assert!(!arch::irq_enabled());
    drop(guard);
    assert_eq!(arch::irq_enabled(), enabled);
    // Step 3: 构造同核自竞争场景以检验 try_lock 失败分支不会篡改调用前的中断状态。
    let saved = arch::irq_save();
    // SAFETY:
    // 1. `_preempt` 禁止抢占保证了自上方 `arch::irq_save()` 以来当前任务始终运行在同一物理 CPU 上；
    // 2. `saved` 由本核刚读取生成，原样恢复本核中断状态不会破坏任何并发守卫契约。
    unsafe { arch::irq_restore(saved) };
    let owner = lock.lock();
    // SAFETY:
    // 1. `lock` 是当前函数栈帧私有的局部自旋锁，没有任何中断处理程序或其他 CPU 会访问它，因此在持有 `owner` 期间临时恢复 `saved` 中断状态绝对不会引发中断重入死锁；
    // 2. `_preempt` 保证当前任务不会在开中断期间被抢占迁移到其他 CPU，且下方在析构 `owner` 前会重新调用 `arch::irq_save()` 关闭中断，使 `owner` 析构看到一致的关中断前提。
    unsafe { arch::irq_restore(saved) };
    assert!(lock.try_lock().is_none());
    assert_eq!(arch::irq_enabled(), enabled);
    arch::irq_save();
    drop(owner);
    assert_eq!(arch::irq_enabled(), enabled);
    drop(disabled);
    assert!(arch::irq_enabled());
}
