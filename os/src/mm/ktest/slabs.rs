//! SLUB 分配器 Slab 生命周期状态机、复合页头尾查找、最小阶回退与内存统计对账单元测试。
//!
//! ## 模块职责与架构定位
//! 本模块验证内核固定大小对象分配器（`mm::slub`）的内部链表结构不变量、异常内存压力下的阶数回退策略，
//! 以及全局内存统计快照（`mm::MemoryStats`）与 SLUB/页分配器的闭合关系：
//! - **上游驱动者**：`ktest` 框架通过 `FixedSlabs` 与 `FixedPressure` 夹具在固定 CPU0 任务中运行 `ownership`、`fallback` 与 `accounting` 套件。
//! - **被测与下游依赖**：操作 4096B 档位（`SLUB_CLASS`）的 `SlabObject`，调用 `slub` 状态快照接口、`flush_all`、`shrink_all`、`drain_all_pages` 及 `mm::stats`。
//!
//! ## 核心机制与 Slab 位置状态流转
//! 测试覆盖单个高阶 Slab 在本地缓存、CPU 部分满链与节点部分满链之间的完整迁移闭环，以及物理碎片压力下的阶数降级：
//!
//! ```text
//! 1. 常规 Slab 状态迁移闭环：
//!    [ Active(0) ] ──(槽位耗尽切换)──► [ Full ] ──(首次释放对象)──► [ CpuPartial(0) ]
//!         ▲                                                               │
//!         │ (重新激活装载)                                                │ (超 CPU_PARTIAL_LIMIT 或 flush_all)
//!         └────────────────────── [ NodePartial ] ◄───────────────────────┘
//!                                        │
//!                                        └──(inuse == 0 且 shrink_all)──► 销毁并归还底层物理页块
//!
//! 2. 物理页阶数降级回退 (Fallback)：
//!    首选高阶 (preferred_order > 0) 分配失败 ──► 回退至最小阶 (minimum_order == 0, 单张 4 KiB 页)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 所有用例在绑定于 CPU0 且无其他并发测试参与者的静止窗口内采样快照（`node_snapshot`、`cpu_snapshot`、`slab_head_snapshot`）。
//! - 测试自身不直接持有任何内存子系统内部自旋锁，每次快照读取在 `slub` 内部独立加锁并立即释放。

use crate::mm::ktest::{
    fixed::{FixedPressure, FixedSlabs},
    free, SlabObject, CLASS as SLUB_CLASS, SLOTS as SLUB_SLOTS,
};
use crate::mm::{self, buddy, flush_all, page::SlabLocation, pcp, shrink_all, slub, PAGE_SIZE};
use crate::{arch, cpu};
use core::hint::black_box;
use ktest::{TestContext, TestResult};

/// 校验 `SLUB_CLASS` 档位的节点部分满链表（`NodePartial`）与所有 CPU 的本地部分满链表（`CpuPartial`）的结构完整性及互斥性。
///
/// 调用上下文：在受测档位无并发分配/释放的静止点调用，验证双向/单向链表长度闭合、无环无重复、位置标记一致且 `Active`/`CpuPartial`/`NodePartial` 集合两两不相交。
fn partial_members() {
    // Step 1: 遍历节点级 NodePartial 双向链表，核对前驱指针、未满条件、空块上限并收集所有头页 PFN。
    let node = slub::node_snapshot(SLUB_CLASS);
    let mut node_heads = [usize::MAX; SLUB_SLOTS];
    assert!(
        node.count <= SLUB_SLOTS,
        "memory SLUB node snapshot exceeds storage"
    );
    let mut head = node.partial;
    let mut previous = usize::MAX;
    let mut empty = 0;
    for index in 0..node.count {
        assert!(
            !node_heads[..index].contains(&head),
            "memory SLUB node list contains duplicate member"
        );
        node_heads[index] = head;
        let state =
            slub::slab_head_snapshot(head).expect("memory SLUB node points at freed backing");
        assert_eq!(
            state.location,
            SlabLocation::NodePartial,
            "memory SLUB node member role differs"
        );
        assert_eq!(
            state.previous, previous,
            "memory SLUB node previous link differs"
        );
        assert!(
            state.inuse < state.objects,
            "memory SLUB node member is full"
        );
        assert_eq!(
            state.cache, SLUB_CLASS,
            "memory SLUB node member cache differs"
        );
        if state.inuse == 0 {
            empty += 1;
        }
        previous = head;
        head = state.next;
    }
    assert_eq!(
        head,
        usize::MAX,
        "memory SLUB node count does not cover list"
    );
    assert!(
        empty <= slub::MIN_PARTIAL,
        "memory SLUB node retained too many empty slabs"
    );

    // Step 2: 遍历每个在线 CPU 的 CpuPartial 单向链表与 Active 槽，确保无重复且与节点链表严格互斥。
    for id in 0..cpu::count() {
        let state = slub::cpu_snapshot(SLUB_CLASS, id);
        assert!(
            state.partial_count <= slub::CPU_PARTIAL_LIMIT,
            "memory SLUB CPU partial exceeded bound"
        );
        assert!(
            !node_heads[..node.count].contains(&state.active),
            "memory SLUB active is also node member"
        );
        let mut cpu_heads = [usize::MAX; slub::CPU_PARTIAL_LIMIT];
        let mut head = state.partial;
        for index in 0..state.partial_count {
            assert_ne!(head, state.active, "memory SLUB CPU partial is also active");
            assert!(
                !node_heads[..node.count].contains(&head),
                "memory SLUB partial belongs to node and CPU"
            );
            assert!(
                !cpu_heads[..index].contains(&head),
                "memory SLUB CPU list contains duplicate member"
            );
            cpu_heads[index] = head;
            let slab =
                slub::slab_head_snapshot(head).expect("memory SLUB CPU points at freed backing");
            assert_eq!(
                slab.location,
                SlabLocation::CpuPartial(id),
                "memory SLUB CPU partial was frozen or misplaced"
            );
            assert_eq!(
                slab.cache, SLUB_CLASS,
                "memory SLUB CPU partial cache differs"
            );
            head = slab.next;
        }
        assert_eq!(
            head,
            usize::MAX,
            "memory SLUB CPU partial count does not cover list"
        );
    }
}

/// SLUB 本核空闲链命中、活动 Slab 切换、部分满链表迁移以及高阶尾页寻址测试套件。
#[ktest::suite(fixture = FixedSlabs, tags("mm", "slub"), timeout_ms = 30000)]
mod ownership {
    use super::*;
    /// 验证本核独占空闲链（`local`）快速命中与槽位耗尽后的活动 Slab（`Active`）切换语义。
    ///
    /// - **测试目标**：确认在同一 CPU 上连续分配时优先消耗当前活动 Slab 的 `local` 空闲链，耗尽后旧块转为 `Full` 并启用新活动块。
    /// - **场景步骤**：连续分配超过 3 个整块容量的对象（`objects * 3 + 1`），每次分配前后对比 CPU0 的 `local_count` 与 `active` 头页帧号，并在切换时检查上一个 Slab 的状态。
    /// - **预期结果**：观察到多次 `local` 递减命中与至少两次 `active` 切换；每次切换后原满载 Slab 均处于 `SlabLocation::Full` 且 `inuse == objects`。
    #[test]
    fn local_active(_: &mut TestContext<'_>, fixture: &mut FixedSlabs) -> TestResult {
        fixture.run(|f| {
            let geometry = slub::geometry(SLUB_CLASS);
            assert_eq!(
                geometry.stride, PAGE_SIZE,
                "memory SLUB large class stride differs"
            );
            let count = geometry.objects * 3 + 1;
            assert!(
                count <= SLUB_SLOTS,
                "memory SLUB local storage is too small"
            );

            let mut switches = 0;
            let mut hits = 0;
            for index in 0..count {
                let before = slub::cpu_snapshot(SLUB_CLASS, 0);
                f.allocate(index);
                let after = slub::cpu_snapshot(SLUB_CLASS, 0);
                let slab = slub::slab_snapshot(f.pointer(index));
                assert_eq!(
                    slab.head_pfn, after.active,
                    "memory SLUB allocation is not owner active"
                );
                assert_eq!(
                    slab.location,
                    SlabLocation::Active(0),
                    "memory SLUB active role differs"
                );
                if 0 < before.local_count {
                    assert_eq!(
                        after.active, before.active,
                        "memory SLUB local hit changed active"
                    );
                    assert_eq!(
                        after.local_count + 1,
                        before.local_count,
                        "memory SLUB local hit lost objects"
                    );
                    hits += 1;
                } else if before.active != usize::MAX && before.active != after.active {
                    switches += 1;
                    let previous = slub::slab_snapshot(f.pointer(index - 1));
                    assert_eq!(
                        previous.location,
                        SlabLocation::Full,
                        "memory SLUB exhausted active was not full"
                    );
                    assert_eq!(
                        previous.inuse, previous.objects,
                        "memory SLUB full count differs"
                    );
                }
            }
            assert!(
                1 < switches && 0 < hits,
                "memory SLUB did not exercise local hits and active switches"
            );
            for index in 0..count {
                f.release(index);
            }
        })
    }
    /// 验证 Slab 跨越 `Full`、`CpuPartial`、`NodePartial` 到重新成为 `Active` 的完整所有权转移，以及 `shrink_all` 对存活块的保护。
    ///
    /// - **测试目标**：全面检验 SLUB 各级缓存链表的晋升、溢出下沉、主动刷回（`flush_all`）、重激活与空块收缩（`shrink_all`）逻辑。
    /// - **场景步骤**：先填满 4 个 Slab；释放第 1 个块的首对象使其进入 `CpuPartial(0)`，再释放该块剩余对象验证空块仍暂留 `CpuPartial(0)`；继续释放其余满块对象使 `CpuPartial` 超限溢出至 `NodePartial`；调用 `flush_all()` 后持续分配直至目标 `NodePartial` 块重新被装载为 `Active(0)`；最后调用 `shrink_all()` 并核验所有存活对象。
    /// - **预期结果**：每次状态转换后 `partial_members()` 链表不变量均成立，全空块在 `shrink_all` 中归还，而仍含存活对象的 Slab 保持原头页身份与数据完好。
    #[test]
    fn partial_ownership(_: &mut TestContext<'_>, fixture: &mut FixedSlabs) -> TestResult {
        fixture.run(|f| {
            let objects = slub::geometry(SLUB_CLASS).objects;
            let count = objects * 4 + 1;
            assert!(
                count + objects * 4 <= SLUB_SLOTS,
                "memory SLUB partial storage is too small"
            );

            // Step 1: 连续分配使前 4 个 Slab 完全填满进入 Full 状态。
            for index in 0..count {
                f.allocate(index);
            }
            let first = slub::slab_snapshot(f.pointer(0));
            let target = slub::slab_snapshot(f.pointer(objects + 1)).head_pfn;
            let backing = slub::backing_pages();
            for slab in 0..4 {
                assert_eq!(
                    slub::slab_snapshot(f.pointer(slab * objects)).location,
                    SlabLocation::Full,
                    "memory SLUB full setup failed"
                );
            }

            // Step 2: 释放第 1 个满块的对象，验证首次释放转入 CpuPartial(0)，且块排空后仍暂留本核缓存不提前销毁。
            f.release(0);
            let partial = slub::slab_head_snapshot(first.head_pfn)
                .expect("memory SLUB first free destroyed live slab");
            assert_eq!(
                partial.location,
                SlabLocation::CpuPartial(0),
                "memory SLUB full first free did not enter CPU partial"
            );
            partial_members();
            for index in 1..objects {
                f.release(index);
            }
            let empty = slub::slab_head_snapshot(first.head_pfn)
                .expect("memory SLUB empty CPU partial was freed early");
            assert_eq!(
                empty.location,
                SlabLocation::CpuPartial(0),
                "memory SLUB empty lost CPU responsibility"
            );
            assert_eq!(
                empty.inuse, 0,
                "memory SLUB empty CPU partial is still in use"
            );
            assert_eq!(
                slub::backing_pages(),
                backing,
                "memory SLUB empty CPU partial released backing early"
            );

            // Step 3: 从另外两个满块各释放 1 个对象，触发 CpuPartial 超出上限并将旧块推入 NodePartial。
            f.release(objects);
            f.release(objects * 2);
            partial_members();
            assert!(
                0 < slub::node_snapshot(SLUB_CLASS).count,
                "memory SLUB partial bound did not move any slab to node"
            );

            // Step 4: 刷回全核缓存至节点链，再持续分配直到将目标 NodePartial 块重新激活为 CPU0 的 Active 块。
            flush_all();
            partial_members();
            assert_eq!(
                slub::slab_head_snapshot(target).unwrap().location,
                SlabLocation::NodePartial,
                "memory SLUB flush lost live node partial"
            );
            let mut allocated = count;
            while slub::cpu_snapshot(SLUB_CLASS, 0).active != target {
                assert!(
                    allocated < count + objects * 4,
                    "memory SLUB node partial was never reactivated"
                );
                f.allocate(allocated);
                allocated += 1;
            }
            assert_eq!(
                slub::slab_head_snapshot(target).unwrap().location,
                SlabLocation::Active(0),
                "memory SLUB node acquisition did not freeze active"
            );
            partial_members();

            // Step 5: 执行全局收缩回收空块，确认所有存活对象的签名、内容及所属头页身份不受影响。
            shrink_all();
            partial_members();
            for object in f.held.iter().flatten() {
                object.check();
            }
            assert_eq!(
                slub::slab_snapshot(f.pointer(objects + 1)).head_pfn,
                target,
                "memory SLUB shrink changed live slab identity"
            );
            assert!(
                0 < slub::backing_pages(),
                "memory SLUB shrink freed live backing"
            );
            for index in 0..allocated {
                if f.held[index].is_some() {
                    f.release(index);
                }
            }
        })
    }
    /// 验证高阶复合 Slab 中位于尾页（Tail Page）的对象能够准确定位所属头页（Head Page），且释放尾页对象不影响同块其余存活对象。
    ///
    /// - **测试目标**：确认复合页尾页描述符正确转发至头页元数据，不会误将尾页当作独立页头释放或破坏同块相邻对象。
    /// - **场景步骤**：分配填满一整个高阶 Slab，对比首对象与末对象（位于尾页）的 `slab_snapshot` 快照；随后单独释放尾页上的末对象，检查底层承载页数及剩余对象的快照与内容。
    /// - **预期结果**：尾页对象查询得到与首对象完全一致的 `head_pfn`、`cache` 和 `order`；释放尾页对象后承载页未被提前释放，其余对象内容与元数据保持完好。
    #[test]
    fn tail_head(_: &mut TestContext<'_>, fixture: &mut FixedSlabs) -> TestResult {
        fixture.run(|f| {
            let geometry = slub::geometry(SLUB_CLASS);
            assert!(
                0 < geometry.preferred_order && 1 < geometry.objects,
                "memory SLUB tail requires normal high-order slab"
            );

            for index in 0..geometry.objects {
                f.allocate(index);
            }
            let first = slub::slab_snapshot(f.pointer(0));
            let tail_index = geometry.objects - 1;
            let tail = slub::slab_snapshot(f.pointer(tail_index));
            assert_eq!(
                first.order, geometry.preferred_order,
                "memory SLUB normal allocation did not use preferred order"
            );
            assert_eq!(
                first.objects, geometry.objects,
                "memory SLUB normal capacity differs from actual geometry"
            );
            assert_eq!(first.cache, SLUB_CLASS, "memory SLUB normal cache differs");
            assert_eq!(
                first.head_pfn,
                arch::virt_to_phys(f.pointer(0).as_ptr() as usize) / PAGE_SIZE,
                "memory SLUB first object did not locate its actual backing head"
            );
            assert_eq!(
                tail.head_pfn, first.head_pfn,
                "memory SLUB tail lookup selected another head"
            );
            assert_eq!(tail.cache, SLUB_CLASS, "memory SLUB tail cache differs");
            assert_eq!(
                tail.order, first.order,
                "memory SLUB tail actual order differs"
            );
            let tail_pfn = arch::virt_to_phys(f.pointer(tail_index).as_ptr() as usize) / PAGE_SIZE;
            assert!(
                first.head_pfn < tail_pfn && tail_pfn < first.head_pfn + (1 << first.order),
                "memory SLUB tail object is not on a backing tail page"
            );
            let backing = slub::backing_pages();
            f.release(tail_index);
            assert_eq!(
                slub::backing_pages(),
                backing,
                "memory SLUB tail free released live backing"
            );
            for index in 0..tail_index {
                f.held[index].as_ref().unwrap().check();
                let state = slub::slab_snapshot(f.pointer(index));
                assert_eq!(
                    state.head_pfn, first.head_pfn,
                    "memory SLUB tail free changed surviving head"
                );
                assert_eq!(
                    state.cache, SLUB_CLASS,
                    "memory SLUB tail free changed surviving cache"
                );
                f.release(index);
            }
        })
    }
}

/// 物理内存高度碎片化与近 OOM 压力下 SLUB 首选阶向最小阶（`minimum_order`）降级回退测试套件。
#[ktest::suite(fixture = FixedPressure, tags("mm", "slub", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod fallback {
    use super::*;
    /// 验证伙伴系统无法满足首选高阶（`preferred_order > 0`）且仅剩单张零阶物理页时，SLUB 自动降级以最小阶（Order-0）成功建立承载块。
    ///
    /// - **测试目标**：确认 SLUB 在高阶连续内存缺失时具备回退至单页最小阶继续服务分配的能力，且释放收缩后该单页能完整归还。
    /// - **场景步骤**：先预留 1 张零阶样本页，调用 `exhaust()` 耗尽全局物理页并抽干 `SLUB_CLASS` 现有的全部空闲槽位使 `slub::allocate` 返回 `None`；随后释放预留的那 1 张零阶页并排空 PCP 至伙伴系统（此时仅有 1 页空闲，无法分配高阶块），再次请求分配 1 个 `SlabObject`；检查新 Slab 的阶数与物理地址，最后释放该对象并执行 `shrink_all`。
    /// - **预期结果**：降级分配成功，新 Slab 的 `order == 0`、`objects == 1` 且恰好复用刚归还的单张物理页；释放并收缩后该页退回伙伴系统，头页元数据清除。
    #[test]
    fn preferred_to_minimum(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            let geometry = slub::geometry(SLUB_CLASS);
            assert!(0 < geometry.preferred_order);
            assert_eq!(geometry.minimum_order, 0);
            // Step 1: 预留 1 张零阶页后耗尽全局物理内存，并抽干受测 SLUB 档位已缓存的所有空闲槽位。
            f.samples[0] = Some(mm::alloc_pages(0, false).unwrap());
            let expected = f.samples[0].as_ref().unwrap().phys_addr();
            f.exhaust();
            assert!(black_box(mm::alloc_pages(0, false)).is_none());
            let mut existing = 0;
            loop {
                assert!(
                    existing + 1 < f.objects.len(),
                    "existing class objects exceed owning storage"
                );
                let Some(object) = SlabObject::try_allocate() else {
                    break;
                };
                f.objects[existing] = Some(object);
                existing += 1;
            }
            let baseline_backing = slub::backing_pages();
            assert!(black_box(slub::allocate(SLUB_CLASS)).is_none());
            assert_eq!(slub::backing_pages(), baseline_backing);

            // Step 2: 归还唯一的零阶物理页至伙伴系统，验证首选高阶不可得时 SLUB 成功降级使用该单页分配对象。
            drop(f.samples[0].take());
            mm::drain_all_pages();
            assert_eq!(buddy::free_pages(), 1);
            assert_eq!(pcp::cached_pages(), 0);
            assert!(black_box(mm::alloc_pages(geometry.preferred_order, false)).is_none());
            f.objects[existing] = Some(SlabObject::allocate());
            let actual = slub::slab_snapshot(f.objects[existing].as_ref().unwrap().0);
            assert_eq!(actual.order, 0);
            assert_eq!(actual.objects, 1);
            assert_eq!(actual.cache, SLUB_CLASS);
            assert_eq!(actual.head_pfn * PAGE_SIZE, expected);
            assert_eq!(slub::backing_pages(), baseline_backing + 1);
            assert_eq!(free(), 0);
            f.objects[existing].as_ref().unwrap().check();

            // Step 3: 释放降级分配的对象并执行全局收缩与 PCP 排空，确认该单张承载页完整归还伙伴系统。
            drop(f.objects[existing].take());
            shrink_all();
            mm::drain_all_pages();
            assert_eq!(slub::backing_pages(), baseline_backing);
            assert_eq!(buddy::free_pages(), 1);
            assert!(slub::slab_head_snapshot(actual.head_pfn).is_none());
        })
    }
}

/// 存活物理页块与 SLUB 对象跨越缓存刷回、收缩与排空时的全局内存统计（`MemoryStats`）六域闭合测试套件。
#[ktest::suite(fixture = FixedSlabs, tags("mm", "stats"), timeout_ms = 30000)]
mod accounting {
    use super::*;
    /// 验证同时持有存活 SLUB 对象与直接物理页块时，执行 `flush_all`/`shrink_all`/`drain_all_pages` 不会触碰存活数据，且 `MemoryStats` 六个统计域增减精确闭合。
    ///
    /// - **测试目标**：检验内存回收清理入口对在用内存的安全性，以及 `present`/`reserved`/`metadata`/`buddy_free`/`pcp_cached`/`slab_backing` 六项指标的守恒关系。
    /// - **场景步骤**：记录静止态六域基线并核验启动恒等式；分配 1 个 SLUB 对象与 1 个一阶（Order-1）物理页块并填充 `0x6d`；依次执行全量刷回、收缩与排空后检查对象与页块内容及实时 `mm::stats()` 差值；最后释放全部资源再次比对静止态快照。
    /// - **预期结果**：回收操作后存活对象与页块内容完好，`slab_backing_pages` 精确增加 `1 << slab.order`，`buddy_free_pages` 精确减少直接分配页数与新增 Slab 承载页数之和；全部释放后六域统计完全回到初始基线。
    #[test]
    fn live_across_reclaim(_: &mut TestContext<'_>, fixture: &mut FixedSlabs) -> TestResult {
        fixture.run(|f| {
            let baseline = crate::mm::ktest::settled();
            let boot = mm::pages().stats();
            assert_eq!(
                baseline.present_pages,
                baseline.reserved_pages + baseline.metadata_pages + boot.free_pages
            );
            assert_eq!(baseline.pcp_cached_pages, 0);
            // Step 1: 分配 1 个 SLUB 对象与 1 个 Order-1 物理页块，并将物理页块写满特征字节 0x6d。
            f.allocate(0);
            let pointer = f.pointer(0);
            let head = slub::slab_snapshot(pointer).head_pfn;
            f.pages = Some(mm::alloc_pages(1, false).unwrap());
            let pages = f.pages.as_mut().unwrap();
            let whole = pages.len() / PAGE_SIZE;
            let length = pages.len();
            // SAFETY:
            // 1. `f.pages` 刚由 `mm::alloc_pages(1, false)` 成功分配并由当前夹具独占持有，`length` 等于其完整字节长度；
            // 2. 该连续物理页块无任何其他并发读者或写者，整段直接映射内存可安全写入。
            unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0x6d, length) };

            // Step 2: 执行全局缓存刷回、空块收缩与 PCP 排空，核验存活数据完好且六域统计增量精确匹配。
            flush_all();
            shrink_all();
            mm::drain_all_pages();
            f.held[0].as_ref().unwrap().check();
            let pages = f.pages.as_mut().unwrap();
            // SAFETY:
            // 1. 上方 Step 1 已通过 `write_bytes` 将整段 `pages.len()` 字节初始化为 `0x6d`；
            // 2. `f.pages` 在整个测试期间由当前任务独占保活，回收操作不会触碰已分配页块，构造只读切片无数据竞争。
            let contents = unsafe {
                core::slice::from_raw_parts(pages.as_mut_ptr().cast::<u8>(), pages.len())
            };
            assert!(contents.iter().all(|&byte| byte == 0x6d));
            let slab = slub::slab_snapshot(pointer);
            assert_eq!(slab.head_pfn, head);
            assert_eq!(slab.inuse, 1);
            let live = mm::stats();
            assert_eq!(live.present_pages, baseline.present_pages);
            assert_eq!(live.reserved_pages, baseline.reserved_pages);
            assert_eq!(live.metadata_pages, baseline.metadata_pages);
            assert_eq!(live.pcp_cached_pages, 0);
            assert_eq!(
                live.slab_backing_pages,
                baseline.slab_backing_pages + (1 << slab.order)
            );
            assert_eq!(
                baseline.buddy_free_pages - live.buddy_free_pages,
                whole + (1 << slab.order)
            );

            // Step 3: 释放 SLUB 对象与物理页块，验证收缩排空后 Slab 头页注销且六域统计完全恢复初始基线。
            f.release(0);
            drop(f.pages.take());
            let final_state = crate::mm::ktest::settled();
            assert!(slub::slab_head_snapshot(head).is_none());
            assert_eq!(final_state, baseline);
        })
    }
}
