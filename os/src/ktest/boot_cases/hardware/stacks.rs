//! 主从核（BSP / AP）内核栈空间布局、批量准备内存不足（OOM）回滚及未启动栈回收测试。
//!
//! # 模块职责与架构定位
//! 本模块隶属于启动与硬件自检子系统（`boot_cases::hardware`），验证 `cpu::stack` 模块的内核栈槽位（[`StackSlot`]）生命周期与物理内存账务一致性：
//! - **上游调用者**：由内核测试框架（`ktest`）在 `Thread` 阶段调度执行。
//! - **下游依赖**：调用 `cpu::stack::{prepare_slots, StackSlot}` 进行动态栈页分配与回收，使用 `cache::ktest::HeapPressure` 构造受控的物理页耗尽场景，并通过 `execution_services::{baseline, reclaimed}` 与 `mm::stats` 对账伙伴系统及内存统计快照。
//!
//! # 核心设计与栈槽状态转换
//! 1. **主核与辅核栈空间布局**：
//!    - 主核（CPU 0 / BSP）使用链接期静态分配的 `cpu::BOOT_STACK`（64 KiB），不向伙伴系统借用动态页（`stack_allocation() == None`）。
//!    - 辅核（CPU 1..N / AP）各自独占伙伴系统分配的 64 KiB（`order = 4`）连续物理页块，栈顶为直接映射区间的高地址端 `start + 64 KiB`，各核栈地址区间互不重叠。
//!
//! 2. **`prepare_slots` 两阶段提交与 OOM 回滚状态机**：
//! ```text
//!   [初始空槽: slots[0..2] 均 top == 0, allocation == None]
//!          │
//!          │ prepare_slots() Step 1: 为 slots[0] 分配 64 KiB 连续页成功
//!          ▼
//!   [半准备态: slots[0] 持有物理页但 top == 0 (未发布); slots[1] 待分配]
//!          │
//!          ├──(slots[1] 遭遇 OOM 分配失败)──> 自动对全部候选槽调用 release_unstarted()
//!          │                                        │
//!          │                                        ▼
//!          │                         [全量回滚: 两槽清空，伙伴系统空闲页完整归还]
//!          │
//!          └──(slots[1] 分配成功)──> Step 2: 统一将 top 写为 start + 64 KiB (已发布)
//!                                                   │
//!                                                   ▼ (测试中未交给任何 CPU 启动)
//!                                         release_unstarted() 回收并校验内存账闭合
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 执行 `prepare_slots`、内存耗尽探针及 `release_unstarted` 期间持有 `PreemptGuard` 禁止任务抢占，减少跨核切换对内存统计瞬时快照的干扰。
//! - `StackSlot` 内部自旋锁（保护 `Option<OwnedPages>`）仅在读取分配范围或摘出页块所有权时短暂持有，物理页释放始终在 `StackSlot` 锁外完成，遵守 `StackSlot` 自旋锁先释放、再进入伙伴系统锁的顺序。

use crate::cache::ktest::HeapPressure;
use crate::cpu::stack::{prepare_slots, StackSlot};
use crate::ktest::execution_services::{baseline, reclaimed};
use crate::sync::PreemptGuard;
use crate::{cpu, mm};
use ::ktest::{TestContext, TestError, TestResult};
use core::cell::Cell;
use core::sync::atomic::Ordering;

/// 为两个局部未启动的 [`StackSlot`] 批量分配物理页栈并随即回收，验证栈区间独立性与物理页账务闭合。
///
/// 调用上下文：`Thread` 阶段可睡眠内核线程（内部在分配与回收窗口持有 `PreemptGuard`，但在首尾调用 `baseline` / `reclaimed` 时允许调度）。
///
/// # Safety
/// 1. `slots` 中的两个 [`StackSlot`] 必须是互不相同的局部变量，处于未准备的初始空状态（`top == 0` 且未持有物理页）。
/// 2. 两个槽位的栈地址必须从未发布给固件或任何 CPU 使用，调用期间由当前任务独占，无并发准备、回收或栈内存读写。
unsafe fn prepare_and_release(context: &mut TestContext<'_>, slots: &[StackSlot; 2]) -> TestResult {
    // Step 1: 采集系统静止态内存基线，随后关闭本地抢占并批量准备两个 64 KiB 内核栈槽。
    let before = baseline();
    let preempt = PreemptGuard::new();
    // SAFETY:
    // 1. 本函数 `# Safety` 契约保证 `slots` 包含两个互异、初始为空且从未交给任何 CPU 的局部栈槽，且当前任务独占访问。
    // 2. 当前运行在 `Thread` 阶段，物理页分配器与直接映射区均已完成初始化。
    unsafe { prepare_slots(slots.iter()) }
        .map_err(|_| TestError("AP stack retry allocation failed"))?;
    let allocations = [slots[0].allocation(), slots[1].allocation()];
    let tops = [slots[0].top(), slots[1].top()];
    let allocated = mm::stats();

    // Step 2: 逐一回收两个从未交给任何 CPU 启动的栈槽，并在释放抢占守卫前采样回收后的内存统计。
    for slot in slots {
        // SAFETY:
        // 1. 两个栈槽仅在上方 `prepare_slots` 中分配了物理页，其栈顶从未写入启动记录，也从未交给任何 CPU 或固件作为运行栈。
        // 2. 当前任务独占 `slots` 数组且循环按序访问每个元素一次，不存在并发访问或重复释放。
        unsafe { slot.release_unstarted() };
    }
    let empty = slots
        .iter()
        .all(|slot| slot.top() == 0 && slot.allocation().is_none());
    let after = mm::stats();
    drop(preempt);

    // Step 3: 校验两块栈内存的大小（64 KiB）、页对齐、栈顶位置、区间互不重叠以及伙伴系统空闲页的增减与全量归还。
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

    /// 验证当前在线的所有主从核（BSP 与 AP）内核栈空间布局、物理页归属及启动探针地址范围。
    ///
    /// - **测试目标**：确认主核使用静态链接栈且不占用动态页，每个在线辅核均持有独立的 64 KiB 页对齐动态栈，且启动阶段记录的栈指针探针严格落在各自栈区间内。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：系统已完成多核引导并进入 `Thread` 阶段，`0..cpu::count()` 内的所有 CPU 均已上线；
    ///   2. When：遍历每个在线 CPU 的 `CpuLocal` 描述符，读取其 `stack_top()`、`stack_allocation()` 与 `boot_stack_probe`；
    ///   3. Then：CPU 0 无动态页所有权且栈顶等于 `BOOT_STACK + 64 KiB`；其余辅核各持有 64 KiB 页对齐动态栈；所有 CPU 的启动栈探针均位于 `[start, start + 64 KiB)` 内，且任意两核的栈区间互不重叠。
    #[test]
    fn live_allocations(context: &mut TestContext<'_>) {
        assert_eq!(cpu::STACK_SIZE, 64 * 1024);
        let bsp_start = core::ptr::addr_of!(cpu::BOOT_STACK) as usize;
        // 遍历所有在线逻辑 CPU，分别校验主核静态栈与辅核动态栈边界及两两不重叠约束。
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

    /// 验证批量准备多个辅核栈槽时中途遭遇物理页耗尽（OOM）的两阶段未发布隔离、全量回滚与后续重试恢复。
    ///
    /// - **测试目标**：确认 `prepare_slots` 在第一个栈槽分配成功、第二个栈槽因物理内存不足失败时，不会提前发布任何栈顶地址，且能将第一个栈槽已占用的物理页完整归还给分配器，随后在内存压力解除后同一组槽位仍可正常复用。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：构造两个空局部栈槽，预留一块 64 KiB 连续物理页（`reserved`），随后通过 `HeapPressure::exhaust()` 耗尽剩余可分配内存，再释放 `reserved` 使系统中恰好只够分配一个 64 KiB 栈块；
    ///   2. When：调用 `prepare_slots` 依次准备两个槽位，并通过迭代器 `inspect` 在进入第二个槽位前快照第一个槽位的分配状态与两槽 `top` 值；
    ///   3. Then：快照显示第一个槽位已获得 64 KiB 物理页但两槽 `top` 均为 `0`（未发布）；第二个槽位触发 OOM 后 `prepare_slots` 返回错误并将两槽清空，内存统计恢复至调用前状态；解除内存压力后，对同一组 `slots` 执行 [`prepare_and_release`] 成功完成分配与归还。
    #[test]
    fn partial_prepare_oom(context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 记录初始内存基线，预留单块 64 KiB 页后耗尽其余物理内存，再放回预留块以构造“仅够一个栈槽”的精确边界。
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

        // Step 2: 批量准备两个栈槽；利用迭代器钩子在推进到第 2 个槽位时观测第 1 个槽位“已分配物理页但尚未发布 top”的中间态。
        let candidates = slots.iter().inspect(|slot| {
            if core::ptr::eq(*slot, &slots[1]) {
                if let Some(allocation) = slots[0].allocation() {
                    partial.set(Some((allocation, slots[0].top(), slots[1].top())));
                }
            }
        });
        // SAFETY:
        // 1. `slots` 为当前栈帧上新建的两个互异局部空槽，从未交给任何 CPU 或外部线程。
        // 2. `PreemptGuard` 保持当前任务连续执行，`inspect` 闭包仅只读查询 `allocation()` 与 `top()`，不修改槽位状态或读写栈页。
        let failed = unsafe { prepare_slots(candidates) }.is_err();
        let rollback_after = mm::stats();
        let empty = slots
            .iter()
            .all(|slot| slot.top() == 0 && slot.allocation().is_none());
        drop(pressure);
        drop(preempt);

        // Step 3: 释放内存压力守卫，断言半准备中间态、OOM 错误返回及回滚前后的内存统计完全一致。
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

        // Step 4: 在内存压力完全解除后，复用同一组回滚后的空槽验证二次准备与正常回收。
        // SAFETY:
        // 1. 上方断言 `empty` 已确认 `prepare_slots` 失败回滚后两个局部槽均已恢复为空状态（`top == 0` 且无物理页）。
        // 2. 两个槽位自创建以来从未交给任何 CPU，由当前任务全程独占访问。
        unsafe { prepare_and_release(context, &slots) }
    }

    /// 验证在内存充足条件下准备两个独立的辅核栈槽后，对未启动栈槽执行回收能够完整归还物理页。
    ///
    /// - **测试目标**：确认 `prepare_slots` 成功发布栈顶后，若目标辅核尚未启动，`StackSlot::release_unstarted` 能清零栈顶并将两块 64 KiB 连续页准确退回伙伴系统。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：在当前线程栈上构造两个初始为空的局部 [`StackSlot`]；
    ///   2. When：调用 [`prepare_and_release`] 执行批量分配、栈顶发布与未启动回收；
    ///   3. Then：分配阶段伙伴系统空闲页精确减少 $128\text{ KiB} / \text{PAGE\_SIZE}$ 页且两栈区间互不重叠，回收后两槽清空且系统内存账完全恢复至基线。
    #[test]
    fn unstarted_release(context: &mut TestContext<'_>) -> TestResult {
        let slots = [StackSlot::new(0), StackSlot::new(0)];
        // SAFETY:
        // 1. `slots` 中的两个 [`StackSlot`] 刚在当前任务栈上创建，互不相同且均处于初始空状态。
        // 2. 两个槽位没有任何外部引用，也从未交给任何 CPU 使用，调用期间由当前任务独占。
        unsafe { prepare_and_release(context, &slots) }
    }
}
