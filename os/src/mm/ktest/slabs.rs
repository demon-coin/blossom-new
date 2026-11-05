//! SLUB 真实 active/partial/head 状态及 minimum backing 回归。

use crate::mm::ktest::{
    fixed::{FixedPressure, FixedSlabs},
    free, SlabObject, CLASS as SLUB_CLASS, SLOTS as SLUB_SLOTS,
};
use crate::mm::{self, buddy, flush_all, page::SlabLocation, pcp, shrink_all, slub, PAGE_SIZE};
use crate::{arch, cpu};
use core::hint::black_box;
use ktest::{TestContext, TestResult};

/// 停止参与者的真实链数量、角色及 CPU/node 互斥检查。
fn partial_members() {
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

#[ktest::suite(fixture = FixedSlabs, tags("mm", "slub"), timeout_ms = 30000)]
mod ownership {
    use super::*;
    /// 真实本核 local 命中与跨 slab active 切换，每页对象始终由 fixture 持有。
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
    /// 实际 full→CPU partial→node→active 成员转换与 live 跨 shrink。
    #[test]
    fn partial_ownership(_: &mut TestContext<'_>, fixture: &mut FixedSlabs) -> TestResult {
        fixture.run(|f| {
            let objects = slub::geometry(SLUB_CLASS).objects;
            let count = objects * 4 + 1;
            assert!(
                count + objects * 4 <= SLUB_SLOTS,
                "memory SLUB partial storage is too small"
            );

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

            f.release(objects);
            f.release(objects * 2);
            partial_members();
            assert!(
                0 < slub::node_snapshot(SLUB_CLASS).count,
                "memory SLUB partial bound did not move any slab to node"
            );

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
    /// 高阶 tail 页定位原真实 head/cache/order，释放后保其余 live 内容。
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

#[ktest::suite(fixture = FixedPressure, tags("mm", "slub", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod fallback {
    use super::*;
    /// 真页 OOM 并耗尽既有 class free 后，只归还一页由真实 minimum 接管。
    #[test]
    fn preferred_to_minimum(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            let geometry = slub::geometry(SLUB_CLASS);
            assert!(0 < geometry.preferred_order);
            assert_eq!(geometry.minimum_order, 0);
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

            drop(f.objects[existing].take());
            shrink_all();
            mm::drain_all_pages();
            assert_eq!(slub::backing_pages(), baseline_backing);
            assert_eq!(buddy::free_pages(), 1);
            assert!(slub::slab_head_snapshot(actual.head_pfn).is_none());
        })
    }
}

#[ktest::suite(fixture = FixedSlabs, tags("mm", "stats"), timeout_ms = 30000)]
mod accounting {
    use super::*;
    /// 实际 live 页与对象跨 flush/shrink/drain 保身份、内容及六域增量。
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
            f.allocate(0);
            let pointer = f.pointer(0);
            let head = slub::slab_snapshot(pointer).head_pfn;
            f.pages = Some(mm::alloc_pages(1, false).unwrap());
            let pages = f.pages.as_mut().unwrap();
            let whole = pages.len() / PAGE_SIZE;
            let length = pages.len();
            // SAFETY: owning fixture 独占整个 live 页块，完整 length 均可写。
            unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0x6d, length) };

            flush_all();
            shrink_all();
            mm::drain_all_pages();
            f.held[0].as_ref().unwrap().check();
            let pages = f.pages.as_mut().unwrap();
            // SAFETY: 完整范围此前已初始化，fixture 保原页唯一权，无并发写者。
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

            f.release(0);
            drop(f.pages.take());
            let final_state = crate::mm::ktest::settled();
            assert!(slub::slab_head_snapshot(head).is_none());
            assert_eq!(final_state, baseline);
        })
    }
}
