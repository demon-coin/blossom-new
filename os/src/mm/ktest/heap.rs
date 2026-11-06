//! 全局内核堆分配器（`GlobalAlloc`）布局路由、清零隔离与重分配（`realloc`）单元测试套件。
//!
//! ## 模块职责与架构定位
//! 本模块隶属于内存管理子系统的内核自测套件（`mm::ktest`），负责对全局堆分配器
//! （`crate::mm::heap_allocator`）面向 `core::alloc::GlobalAlloc` 与 `alloc` 标准容器（`Box`、`Vec`）
//! 的行为进行端到端验证：
//! - **上游驱动者**：`ktest` 测试运行器在固定 CPU0 的独立任务中调用 `layouts` 与 `exhaustion` 两个测试套件。
//! - **被测与下游依赖**：通过 `alloc::alloc::{alloc, alloc_zeroed, realloc, dealloc}` 触发全局堆分配，
//!   并结合 `mm::slub` 快照与 `mm::buddy`/`mm::pcp` 空闲页计数校验内部大小分档与物理页账本守恒。
//!
//! ## 核心测试机制与布局路由边界
//! 全局堆分配器根据请求的 `Layout(size, align)` 将分配请求分流至 SLUB 小对象缓存或伙伴系统大块页分配：
//!
//! ```text
//! Layout(size, align)
//!   │
//!   ├─ max(size, align) <= 4096B ──► SLUB 12 档大小类别 (class 0..=11, 步长 8B..=4096B)
//!   │                                 └─ 验证：最小匹配 class、首选阶数、槽位步长对齐与邻槽零越界
//!   ├─ 4096B < max(size, align) <= MAX_BLOCK (4 MiB) ──► 伙伴系统连续物理页块 (LARGE)
//!   │                                                     └─ 验证：2 的幂向上取整物理对齐与页数守恒
//!   └─ max(size, align) > MAX_BLOCK ──► 超出硬件/伙伴系统上限，直接返回空指针 (NULL)
//! ```
//!
//! ## 锁顺序与并发执行契约
//! - 所有测试用例均通过 `FixedHeap` 或 `FixedPressure` 夹具在绑定于 CPU0 的独立内核任务中串行运行，
//!   确保本核 SLUB 活动块（Active Slab）采样确定且无跨核噪声。
//! - 测试代码自身不持有任何自旋锁；堆分配与快照查询按正常调用顺序进入 `slub` 与 `buddy`/`pcp` 内部锁。

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

/// 伙伴系统单次可分配的最大连续物理块字节数（`PAGE_SIZE << MAX_ORDER`）。
const MAX_BLOCK: usize = PAGE_SIZE << buddy::MAX_ORDER;
/// 在 [`LAYOUTS`] 测试表中标记请求应走伙伴系统大块页路径而非 SLUB 小对象缓存的哨兵值。
const LARGE: usize = usize::MAX;

/// 覆盖全部 12 档 SLUB 大小类别边界及大块页分配边界的测试参数表：`(size, align, expected_class)`。
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

/// 校验已分配堆对象 `object` 的地址对齐、所属 SLUB 档位（或大块物理页阶数）及槽位边界。
///
/// 调用上下文：在持有 `object` 独占所有权的普通任务上下文中调用。
/// 返回值：若该对象直接由伙伴系统分配（`class == LARGE`），返回其占用的物理页数；若属于 SLUB 缓存，返回 `0`。
fn layout_check(object: &HeapObject, class: usize) -> usize {
    let layout = object.layout;
    let address = object.pointer.as_ptr() as usize;
    assert_eq!(address % layout.align(), 0);
    // 分支 1：超过 4096B 或高对齐的大块分配，验证物理地址按向上取整后的 2 的幂字节边界对齐。
    if class == LARGE {
        let bytes = layout
            .size()
            .max(layout.align())
            .max(PAGE_SIZE)
            .next_power_of_two();
        assert_eq!(arch::virt_to_phys(address) % bytes, 0);
        bytes / PAGE_SIZE
    } else {
        // 分支 2：SLUB 小对象分配，验证命中预期大小档位、首选阶数、槽位步长偏移且未越出 Slab 尾边界。
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

/// 在按虚拟地址升序排列的已持有对象切片 `held` 中，查找同属当前 CPU 活动 Slab（`active`）的三个相邻槽位起点下标。
///
/// 调用上下文：在固定绑定于目标 CPU 的任务中调用，用于构造“左邻、中间、右邻”紧密相连的三槽位测试样本。
/// 返回值：若找到三个连续步长（`stride`）且位于当前活动 Slab 的对象，返回左侧对象在 `held` 中的起始下标；否则返回 `None`。
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

/// 全局堆分配器的常规布局分流、零初始化邻槽隔离、重分配前缀保留与标准容器集成测试套件。
#[ktest::suite(fixture = FixedHeap, tags("mm", "heap"), timeout_ms = 30000)]
mod layouts {
    use super::*;
    /// 逐项验证 [`LAYOUTS`] 表中每一组 `(size, align)` 的分配档位选择、字节读写与按原布局释放。
    ///
    /// - **测试目标**：确认从 1B 到 `MAX_BLOCK` 的所有边界大小与对齐组合均路由至最小匹配的 SLUB 类别或大块页路径。
    /// - **场景步骤**：按参数下标取出布局并分配非清零堆对象，调用 `layout_check` 核验几何属性，随后写入特征字节并逐字节读回校验。
    /// - **预期结果**：对齐与档位完全匹配预期，整段请求区间可安全读写，用例退出时由夹具按原始 `Layout` 归还内存且无泄漏。
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
    /// 验证 [`LAYOUTS`] 表中全部 28 种布局同时存活时地址区间互不重叠，且物理页账本严格守恒。
    ///
    /// - **测试目标**：检验多档位 SLUB 缓存与大块物理页并发驻留时的地址隔离性与全局页数核算准确性。
    /// - **场景步骤**：依次分配全部 28 种布局的对象，每次分配后与此前所有存活对象比对地址区间并填充特征字节；全部就绪后核对空闲页、SLUB 承载页与大块页之和，最后逐个复核对象内容。
    /// - **预期结果**：任意两个存活对象的虚拟地址区间均不交叠，`free() + slub::backing_pages() + large_pages` 与分配前基线完全相等，所有对象数据完好。
    #[test]
    fn simultaneous_ranges(_: &mut TestContext<'_>, fixture: &mut FixedHeap) -> TestResult {
        fixture.run(|f| {
            let initial = free() + slub::backing_pages();
            let mut large_pages = 0;
            // Step 1: 逐一分配全部测试布局，校验与既有存活对象无地址重叠并写入各自独立的特征值。
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

            // Step 2: 校验空闲页、SLUB 底层承载页与大块直分配页三者之和守恒，并复核所有存活对象内容未被踩踏。
            assert_eq!(free() + slub::backing_pages() + large_pages, initial);
            for (index, object) in f.held[..LAYOUTS.len()].iter().flatten().enumerate() {
                object.check(
                    object.layout.size(),
                    (index as u8).wrapping_mul(7).wrapping_add(1),
                );
            }
        })
    }
    /// 验证超出伙伴系统最大块限制（尺寸超限或对齐超限）的请求会被全局分配器安全拒绝并返回空指针。
    ///
    /// - **测试目标**：确保超大 `size` 或超大 `align` 不会触发整数溢出、异常崩溃或破坏空闲页计数。
    /// - **场景步骤**：构造超出 `MAX_BLOCK` 的合法 `Layout`，通过 `black_box` 屏障直接调用 `alloc::alloc::alloc` 防止编译器常量折叠优化。
    /// - **预期结果**：返回空指针（`NULL`），且调用前后的全局可分配空闲页数保持不变。
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
            // SAFETY:
            // 1. 上方 `Layout::from_size_align(size, align).unwrap()` 已校验 `size > 0` 且 `align` 为合法的 2 的幂，满足 `GlobalAlloc::alloc` 前置契约；
            // 2. 若分配器因缺陷意外返回非空指针，下方 `NonNull::new` 分支会立即将其封装进夹具的 `f.held[0]` 槽位，确保断言失败展开时仍能按原 `layout` 安全释放。
            let pointer = black_box(unsafe { alloc::alloc::alloc(black_box(layout)) });
            if let Some(pointer) = core::ptr::NonNull::new(pointer) {
                f.held[0] = Some(HeapObject { pointer, layout });
            }
            assert!(pointer.is_null());
            assert_eq!(free(), before);
        })
    }
    /// 验证 `alloc_zeroed` 仅清零目标对象自身，绝不越界覆盖同一活动 Slab 内紧邻的左右存活对象。
    ///
    /// - **测试目标**：检验 SLUB 槽位复用与大块页分配在 `alloc_zeroed` 路径下的清零完整性及邻槽边界隔离。
    /// - **场景步骤**：持续分配直至在当前 CPU 的活动 Slab 中捕获三个地址连续的相邻槽位（大块页则直接分配三个对象），分别填充 `0x35`、`0xac`、`0x79`；释放中间槽位后立即以 `zeroed = true` 重新分配，检查复用身份、全零内容与左右邻槽内容。
    /// - **预期结果**：SLUB 路径下重新分配精确复用刚释放的中间槽位且与左右邻槽保持单步长间距；中间对象全部字节为 `0`，左右相邻对象仍完整保持 `0x35` 与 `0x79`。
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
            // Step 1: 对于 SLUB 小对象，持续采样并按地址排序，直到捕获同一活动 Slab 上紧密相邻的三个槽位。
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

            // Step 2: 向左、中、右三个对象填入非零特征字节，随后释放中间对象并以清零模式重新申请。
            for (index, byte) in [0x35, 0xac, 0x79].iter().copied().enumerate() {
                f.held[index].as_mut().unwrap().fill(byte);
            }
            let middle = f.held[1].as_ref().unwrap().pointer;
            drop(f.held[1].take());
            f.held[1] = Some(HeapObject::allocate(layout, true));

            // Step 3: 核验新分配的中间对象对齐、活动 Slab 邻接几何关系，以及中间全零、两侧数据未受污染。
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
    /// 验证 `realloc` 在跨档位扩容（SLUB 到大块页）与缩容（大块页到 SLUB）时均保留公共前缀且不影响相邻对象。
    ///
    /// - **测试目标**：确认 `GlobalAlloc::realloc` 在大小跨越 SLUB 与伙伴系统边界时正确拷贝 `min(old_size, new_size)` 字节并维护对齐。
    /// - **场景步骤**：连续分配三个 64B 对齐的对象并分别填充特征值，对中间对象调用 `resize(new_size)` 执行扩容或缩容，检查新指针的对齐、旧数据前缀保留情况及新空间可写性，最后复核两侧对象。
    /// - **预期结果**：`resize` 成功返回，新地址仍满足 64B 对齐，前 `min(old_size, new_size)` 字节保持原特征值，重写整个新长度后读写正常，左右存活对象内容未变。
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

            // 执行重分配并验证对齐、旧前缀保留、全区间可写性以及相邻存活对象完好性。
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
    /// 验证标准库 `Box<[usize; 12]>` 分配落在精确匹配的 96B SLUB 大小档位中，且元素读写与析构正常。
    ///
    /// - **测试目标**：确认 Rust 标准库智能指针 `Box` 通过全局分配器路由到最紧凑的 SLUB 缓存档位。
    /// - **场景步骤**：在夹具槽位中创建包含 12 个 `usize` 元素的 `Box` 数组，通过 `slub::slab_snapshot` 查询其所属 Slab 缓存档位步长，并逐元素校验内容。
    /// - **预期结果**：`Box` 所处 SLUB 档位的对象步长恰好等于数组字节大小（96B），全部元素值准确无误。
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
    /// 验证标准库 `Vec<usize>` 在连续推入 1025 个元素过程中经历多次动态扩容，且历史元素完整保留。
    ///
    /// - **测试目标**：确认 `Vec` 从小容量 SLUB 槽位逐步扩容至多页大块内存的全链路 `alloc`/`realloc`/`dealloc` 协同正确。
    /// - **场景步骤**：创建初始容量为 2 的 `Vec<usize>`，循环推入 1025 个计算元素并统计容量增长次数，随后遍历校验全部元素。
    /// - **预期结果**：容量发生不止一次增长，最终长度为 1025，且所有下标对应的元素值均与推入时一致。
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

/// 物理内存耗尽（OOM）压力下的全局堆分配失败回退与原对象保持测试套件。
#[ktest::suite(fixture = FixedPressure, tags("mm", "heap", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod exhaustion {
    use super::*;
    /// 验证物理内存耗尽导致 `realloc` 扩容失败时，原对象指针、布局与数据内容均保持完好。
    ///
    /// - **测试目标**：确保 `GlobalAlloc::realloc` 在无法分配新大块内存时遵循不变性契约——返回空指针且绝不释放或篡改旧对象。
    /// - **场景步骤**：先分配三个 65B 堆对象并填充特征字节，调用 `exhaust()` 耗尽全局物理空闲页，再对中间对象尝试扩容至 8193B（需连续多页）。
    /// - **预期结果**：`resize` 返回 `false`，中间对象的指针地址、布局及 65B 内容 `0xc7` 原封不动，两侧相邻对象同样完好，夹具清理时只释放一次旧对象。
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
    /// 验证全局物理空闲页耗尽后，向 `GlobalAlloc` 请求大块堆内存会干净地返回空指针（`NULL`）。
    ///
    /// - **测试目标**：确认大对象分配路径在伙伴系统无可用页时不会 panic，而是向上层调用方返回 `NULL`。
    /// - **场景步骤**：调用 `exhaust()` 抽干伙伴系统与 PCP 缓存中的全部物理页，随后直接调用 `alloc::alloc::alloc` 申请 8193B 的大块布局。
    /// - **预期结果**：分配返回空指针，且全局空闲页计数仍为 `0`。
    #[test]
    fn allocation_null(_: &mut TestContext<'_>, fixture: &mut FixedPressure) -> TestResult {
        fixture.run(|f| {
            f.exhaust();
            let layout = Layout::from_size_align(8193, 64).unwrap();
            // SAFETY:
            // 1. 上方构造的 `layout` 大小非零（8193）且对齐合法（64），满足 `GlobalAlloc::alloc` 调用契约；
            // 2. 若分配意外成功返回非空指针，下方 `NonNull::new` 分支会立即将其存入 `f.heap[0]` 托管生命周期，避免断言失败展开时发生内存泄漏。
            let pointer = black_box(unsafe { alloc::alloc::alloc(black_box(layout)) });
            if let Some(pointer) = core::ptr::NonNull::new(pointer) {
                f.heap[0] = Some(HeapObject { pointer, layout });
            }
            assert!(pointer.is_null());
            assert_eq!(free(), 0);
        })
    }
}
