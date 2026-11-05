//! 全局分配器原 Layout、真实 NULL、零页与 realloc 的拥有权回归。

use crate::mm::ktest::{
    fixed::{FixedHeap, FixedPressure},
    free, HeapObject,
};
use crate::{
    arch,
    mm::{buddy, slub, PAGE_SIZE},
};
use core::{alloc::Layout, hint::black_box};
use ktest::{TestContext, TestResult};

const MAX_BLOCK: usize = PAGE_SIZE << buddy::MAX_ORDER;
const LARGE: usize = usize::MAX;

const LAYOUTS: &[(usize, usize, usize)] = &[
    (1, 1, 0),
    (8, 8, 0),
    (9, 1, 1),
    (16, 16, 1),
    (17, 1, 2),
    (32, 32, 2),
    (33, 1, 3),
    (64, 64, 3),
    (65, 8, 4),
    (96, 32, 4),
    (65, 64, 5),
    (128, 128, 5),
    (129, 8, 6),
    (192, 64, 6),
    (193, 1, 7),
    (256, 256, 7),
    (257, 1, 8),
    (512, 512, 8),
    (513, 1, 9),
    (1024, 1024, 9),
    (1025, 1, 10),
    (2048, 2048, 10),
    (2049, PAGE_SIZE, 11),
    (4096, PAGE_SIZE, 11),
    (4097, 8, LARGE),
    (7, PAGE_SIZE * 2, LARGE),
    (8193, PAGE_SIZE * 4, LARGE),
    (MAX_BLOCK, MAX_BLOCK, LARGE),
];

/// 对真实 live 指针检查最小 class、backing 原阶、对象步长与完整请求范围。
fn layout_check(object: &HeapObject, class: usize) -> usize {
    let layout = object.layout;
    let address = object.pointer.as_ptr() as usize;
    assert_eq!(address % layout.align(), 0);
    if class == LARGE {
        let bytes = layout
            .size()
            .max(layout.align())
            .max(PAGE_SIZE)
            .next_power_of_two();
        assert_eq!(arch::virt_to_phys(address) % bytes, 0);
        bytes / PAGE_SIZE
    } else {
        let state = slub::slab_snapshot(object.pointer);
        let geometry = slub::geometry(class);
        assert_eq!(state.cache, class);
        assert_eq!(state.order, geometry.preferred_order);
        let base = arch::phys_to_virt(state.head_pfn * PAGE_SIZE);
        assert_eq!((address - base) % geometry.stride, 0);
        assert!(address + layout.size() <= base + (PAGE_SIZE << state.order));
        0
    }
}

/// 在按地址排序的 live 拥有前缀中寻找当前 CPU active slab 的三个连续槽。
fn active_neighbor_start(held: &[Option<HeapObject>]) -> Option<usize> {
    let cache = slub::slab_snapshot(held.first()?.as_ref()?.pointer).cache;
    let stride = slub::geometry(cache).stride;
    let active = slub::cpu_snapshot(cache, crate::cpu::current().id).active;
    held.windows(3).position(|slots| {
        let [Some(left), Some(middle), Some(right)] = slots else {
            return false;
        };
        let left_address = left.pointer.as_ptr() as usize;
        let middle_address = middle.pointer.as_ptr() as usize;
        let right_address = right.pointer.as_ptr() as usize;
        middle_address - left_address == stride
            && right_address - middle_address == stride
            && slub::slab_snapshot(left.pointer).head_pfn == active
            && slub::slab_snapshot(right.pointer).head_pfn == active
    })
}

#[ktest::suite(fixture = FixedHeap, tags("mm", "heap"), timeout_ms = 30000)]
mod layouts {
    use super::*;
    /// 每个原 Layout 形成独立参数，真实分配、初始化、读取、原布局 Drop。
    #[test]
    #[case("size1_align1", 0)]
    #[case("size8_align8", 1)]
    #[case("size9_align1", 2)]
    #[case("size16_align16", 3)]
    #[case("size17_align1", 4)]
    #[case("size32_align32", 5)]
    #[case("size33_align1", 6)]
    #[case("size64_align64", 7)]
    #[case("size65_align8", 8)]
    #[case("size96_align32", 9)]
    #[case("size65_align64", 10)]
    #[case("size128_align128", 11)]
    #[case("size129_align8", 12)]
    #[case("size192_align64", 13)]
    #[case("size193_align1", 14)]
    #[case("size256_align256", 15)]
    #[case("size257_align1", 16)]
    #[case("size512_align512", 17)]
    #[case("size513_align1", 18)]
    #[case("size1024_align1024", 19)]
    #[case("size1025_align1", 20)]
    #[case("size2048_align2048", 21)]
    #[case("size2049_align4096", 22)]
    #[case("size4096_align4096", 23)]
    #[case("size4097_align8", 24)]
    #[case("size7_align8192", 25)]
    #[case("size8193_align16384", 26)]
    #[case("maximum_block", 27)]
    fn selection(_: &mut TestContext<'_>, fixture: &mut FixedHeap, index: usize) -> TestResult {
        fixture.run(move |f| {
            let (size, align, class) = LAYOUTS[index];
            f.held[0] = Some(HeapObject::allocate(
                Layout::from_size_align(size, align).unwrap(),
                false,
            ));
            let object = f.held[0].as_mut().unwrap();
            layout_check(object, class);
            let byte = (index as u8).wrapping_mul(7).wrapping_add(1);
            object.fill(byte);
            object.check(size, byte);
        })
    }
    /// 原全部 Layout 同时 live 不重叠，slab 与整体页块逐域增量守恒。
    #[test]
    fn simultaneous_ranges(_: &mut TestContext<'_>, fixture: &mut FixedHeap) -> TestResult {
        fixture.run(|f| {
            let initial = free() + slub::backing_pages();
            let mut large_pages = 0;
            for (index, &(size, align, class)) in LAYOUTS.iter().enumerate() {
                f.held[index] = Some(HeapObject::allocate(
                    Layout::from_size_align(size, align).unwrap(),
                    false,
                ));
                let address = f.held[index].as_ref().unwrap().pointer.as_ptr() as usize;
                for previous in f.held[..index].iter().flatten() {
                    let other = previous.pointer.as_ptr() as usize;
                    assert!(other + previous.layout.size() <= address || address + size <= other);
                }
                let object = f.held[index].as_mut().unwrap();
                large_pages += layout_check(object, class);
                object.fill((index as u8).wrapping_mul(7).wrapping_add(1));
            }

            assert_eq!(free() + slub::backing_pages() + large_pages, initial);
            for (index, object) in f.held[..LAYOUTS.len()].iter().flatten().enumerate() {
                object.check(
                    object.layout.size(),
                    (index as u8).wrapping_mul(7).wrapping_add(1),
                );
            }
        })
    }
    /// 超大尺寸与超大对齐各经过实际 allocator，black_box 保真实 NULL 观察。
    #[test]
    #[case("size_over_max", MAX_BLOCK + 1, 1)]
    #[case("align_over_max", 1, MAX_BLOCK * 2)]
    fn rejected_layout(
        _: &mut TestContext<'_>,
        fixture: &mut FixedHeap,
        size: usize,
        align: usize,
    ) -> TestResult {
        fixture.run(move |f| {
            let layout = Layout::from_size_align(size, align).unwrap();
            let before = free();
            // SAFETY: Layout 合法非零；若意外成功仍立即登记 owner，断言失败可安全清理。
            let pointer = black_box(unsafe { alloc::alloc::alloc(black_box(layout)) });
            if let Some(pointer) = core::ptr::NonNull::new(pointer) {
                f.held[0] = Some(HeapObject { pointer, layout });
            }
            assert!(pointer.is_null());
            assert_eq!(free(), before);
        })
    }
    /// 首次取得 active slab 的三邻接槽即停止采样，zeroed 只清中间请求且保留左右 live 字节与几何。
    #[test]
    #[case("size65_align8", 65, 8)]
    #[case("size129_align8", 129, 8)]
    #[case("size4097_align8192", 4097, PAGE_SIZE * 2)]
    fn zeroed_neighbors(
        _: &mut TestContext<'_>,
        fixture: &mut FixedHeap,
        size: usize,
        align: usize,
    ) -> TestResult {
        fixture.run(move |f| {
            let layout = Layout::from_size_align(size, align).unwrap();
            let count = if align <= PAGE_SIZE { f.held.len() } else { 3 };
            let mut start = None;
            for index in 0..count {
                f.held[index] = Some(HeapObject::allocate(layout, false));
                if align <= PAGE_SIZE {
                    let held = &mut f.held[..=index];
                    held.sort_unstable_by_key(|slot| {
                        slot.as_ref()
                            .map_or(usize::MAX, |object| object.pointer.as_ptr() as usize)
                    });
                    start = active_neighbor_start(held);
                    if start.is_some() {
                        break;
                    }
                }
            }
            if align <= PAGE_SIZE {
                let start = start.expect("owned heap sample has no adjacent live neighbors");
                f.held.rotate_left(start);
            }

            for (index, byte) in [0x35, 0xac, 0x79].iter().copied().enumerate() {
                f.held[index].as_mut().unwrap().fill(byte);
            }
            let middle = f.held[1].as_ref().unwrap().pointer;
            drop(f.held[1].take());
            f.held[1] = Some(HeapObject::allocate(layout, true));

            let left = f.held[0].as_ref().unwrap();
            let zero = f.held[1].as_ref().unwrap();
            let right = f.held[2].as_ref().unwrap();
            assert_eq!(zero.pointer.as_ptr() as usize % align, 0);
            if align <= PAGE_SIZE {
                let l = slub::slab_snapshot(left.pointer);
                let r = slub::slab_snapshot(right.pointer);
                let z = slub::slab_snapshot(zero.pointer);
                assert_eq!(zero.pointer, middle);
                assert_eq!(l.head_pfn, z.head_pfn);
                assert_eq!(r.head_pfn, z.head_pfn);
                let stride = slub::geometry(l.cache).stride;
                assert_eq!(
                    (left.pointer.as_ptr() as usize).abs_diff(zero.pointer.as_ptr() as usize),
                    stride
                );
                assert_eq!(
                    (right.pointer.as_ptr() as usize).abs_diff(zero.pointer.as_ptr() as usize),
                    stride
                );
            }
            zero.check(size, 0);
            left.check(size, 0x35);
            right.check(size, 0x79);
        })
    }
    /// 独立增长与缩小参数都保最小旧请求前缀、对齐以及两侧 live 请求。
    #[test]
    #[case("grow", 65, 8193)]
    #[case("shrink", 8193, 33)]
    fn realloc_prefix(
        _: &mut TestContext<'_>,
        fixture: &mut FixedHeap,
        old_size: usize,
        new_size: usize,
    ) -> TestResult {
        fixture.run(move |f| {
            let layout = Layout::from_size_align(old_size, 64).unwrap();
            let prefix = if old_size < new_size { 0x5b } else { 0x91 };
            for (index, byte) in [0x35, prefix, 0x79].iter().copied().enumerate() {
                f.held[index] = Some(HeapObject::allocate(layout, false));
                f.held[index].as_mut().unwrap().fill(byte);
            }

            let middle = f.held[1].as_mut().unwrap();
            assert!(middle.resize(new_size));
            assert_eq!(middle.pointer.as_ptr() as usize % 64, 0);
            middle.check(old_size.min(new_size), prefix);
            middle.fill(0x91);
            middle.check(new_size, 0x91);
            f.held[0].as_ref().unwrap().check(old_size, 0x35);
            f.held[2].as_ref().unwrap().check(old_size, 0x79);
        })
    }
    /// Box 内容真实走最小 cache 并由 fixture 持有，panic 时同样执行 Drop。
    #[test]
    fn boxed_payload(_: &mut TestContext<'_>, fixture: &mut FixedHeap) -> TestResult {
        fixture.run(|f| {
            f.boxed = Some(black_box(alloc::boxed::Box::new(core::array::from_fn(
                |i| i * 19 + 7,
            ))));
            let boxed = f.boxed.as_ref().unwrap();
            let pointer = core::ptr::NonNull::from(black_box(boxed.as_ref())).cast::<u8>();
            let state = slub::slab_snapshot(pointer);
            assert_eq!(
                slub::SIZES[state.cache],
                core::mem::size_of_val(boxed.as_ref())
            );
            for (i, &word) in boxed.iter().enumerate() {
                assert_eq!(word, i * 19 + 7);
            }
        })
    }
    /// Vec 多次真实增长保留 1025 个元素，不只断言容量非零。
    #[test]
    fn vector_growth(_: &mut TestContext<'_>, fixture: &mut FixedHeap) -> TestResult {
        fixture.run(|f| {
            f.vector = Some(alloc::vec::Vec::with_capacity(2));
            let vector = f.vector.as_mut().unwrap();
            let initial = vector.capacity();
            let mut growths = 0;
            for i in 0..1025 {
                let before = vector.capacity();
                vector.push(i * 23 + 3);
                growths += usize::from(before < vector.capacity());
            }
            assert!(initial < vector.capacity() && 1 < growths);
            assert_eq!(vector.len(), 1025);

            for (i, &word) in black_box(vector).iter().enumerate() {
                assert_eq!(word, i * 23 + 3);
            }
        })
    }
}

#[ktest::suite(fixture = FixedPressure, tags("mm", "heap", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod exhaustion {
    use super::*;
    /// 真 OOM 后失败 realloc 不动原指针、布局与初始化前缀，两侧对象仍 live。
    #[test]
    fn realloc_preserves_old(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            let layout = Layout::from_size_align(65, 64).unwrap();
            for (i, byte) in [0x35, 0xc7, 0x79].iter().copied().enumerate() {
                f.heap[i] = Some(HeapObject::allocate(layout, false));
                f.heap[i].as_mut().unwrap().fill(byte);
            }
            let original = f.heap[1].as_ref().unwrap().pointer;
            f.exhaust();
            assert!(!f.pressure.is_empty());

            assert!(!f.heap[1].as_mut().unwrap().resize(8193));
            let survivor = f.heap[1].as_ref().unwrap();
            assert_eq!(survivor.pointer, original);
            assert_eq!(survivor.layout, layout);
            survivor.check(65, 0xc7);
            f.heap[0].as_ref().unwrap().check(65, 0x35);
            f.heap[2].as_ref().unwrap().check(65, 0x79);
        })
    }
    /// 上限内的大对象在实际全局页耗尽后经 GlobalAlloc 返回真实 NULL。
    #[test]
    fn allocation_null(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            f.exhaust();
            let layout = Layout::from_size_align(8193, 64).unwrap();
            // SAFETY: 合法非零请求，若意外成功先保存原唯一权再断言。
            let pointer = black_box(unsafe { alloc::alloc::alloc(black_box(layout)) });
            if let Some(pointer) = core::ptr::NonNull::new(pointer) {
                f.heap[0] = Some(HeapObject { pointer, layout });
            }
            assert!(pointer.is_null());
            assert_eq!(free(), 0);
        })
    }
}
