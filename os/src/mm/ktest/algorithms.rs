//! 启动区间与受控 buddy 池逐行为验收，直接消费生产算法。

use crate::boot::{BootInfo, Region, MAX_MEMORY_REGIONS, MAX_RESERVED_REGIONS};
use crate::mm::{
    bootmem::{BootMemory, BootStats, PageRange, PAGE_SIZE},
    buddy,
    page::{PageDesc, PageKind, PageMap},
};

/// 在调用方的空账本中填写原区间边界输入，不构造完整固件快照或返回账本副本。
fn ranges(memory: &mut BootMemory) {
    let ram = [
        Region {
            start: 20 * PAGE_SIZE,
            end: 24 * PAGE_SIZE,
        },
        Region {
            start: PAGE_SIZE + 1,
            end: 5 * PAGE_SIZE,
        },
        Region {
            start: 4 * PAGE_SIZE,
            end: 8 * PAGE_SIZE,
        },
        Region {
            start: 8 * PAGE_SIZE,
            end: 12 * PAGE_SIZE - 1,
        },
        Region {
            start: 40 * PAGE_SIZE + 1,
            end: 40 * PAGE_SIZE + 2,
        },
    ];
    let reserved = [
        Region {
            start: 3 * PAGE_SIZE + 1,
            end: 4 * PAGE_SIZE + 1,
        },
        Region {
            start: 4 * PAGE_SIZE,
            end: 5 * PAGE_SIZE,
        },
        Region {
            start: 5 * PAGE_SIZE,
            end: 6 * PAGE_SIZE,
        },
        Region {
            start: 10 * PAGE_SIZE + 1,
            end: 21 * PAGE_SIZE + 1,
        },
        Region {
            start: 30 * PAGE_SIZE,
            end: 31 * PAGE_SIZE,
        },
    ];
    memory.initialize(&ram, &reserved).unwrap();
}

/// 执行原两次实际metadata扣除，保留对齐前缀与尾部空洞，全部状态留在调用方账本。
fn deducted(memory: &mut BootMemory) {
    ranges(memory);
    assert_eq!(memory.allocate(2, 4), Ok(PageRange { start: 8, end: 10 }));
    assert_eq!(memory.allocate(2, 2), Ok(PageRange { start: 6, end: 8 }));
}

#[ktest::suite(context = "Boot", tags("mm", "bootmem"), timeout_ms = 10000)]
mod boot_ranges {
    use super::*;
    /// RAM 先合并原字节区间后向内取整，保留真实空洞。
    #[test]
    fn ram_union() {
        let mut memory = const { BootMemory::empty() };
        ranges(&mut memory);
        assert_eq!(
            memory.ram(),
            [
                PageRange { start: 2, end: 11 },
                PageRange { start: 20, end: 24 }
            ]
        );
    }
    /// reserved 向外取整后仅扣除与 RAM 的交集。
    #[test]
    fn reserved_intersection() {
        let mut memory = const { BootMemory::empty() };
        ranges(&mut memory);
        assert_eq!(
            memory.reserved(),
            [
                PageRange { start: 3, end: 6 },
                PageRange { start: 10, end: 11 },
                PageRange { start: 20, end: 22 }
            ]
        );
    }
    /// 原 free 与分类账务互斥且总数守恒。
    #[test]
    fn initial_free() {
        let mut memory = const { BootMemory::empty() };
        ranges(&mut memory);
        assert_eq!(
            memory.free(),
            [
                PageRange { start: 2, end: 3 },
                PageRange { start: 6, end: 10 },
                PageRange { start: 22, end: 24 }
            ]
        );
        assert_eq!(
            memory.stats(),
            BootStats {
                present_pages: 13,
                reserved_pages: 6,
                metadata_pages: 0,
                free_pages: 7
            }
        );
    }
    /// 对齐申请合并 metadata，free 真实扣除四页。
    #[test]
    fn metadata_deduction() {
        let mut memory = const { BootMemory::empty() };
        deducted(&mut memory);
        assert_eq!(memory.metadata(), [PageRange { start: 6, end: 10 }]);
        assert_eq!(
            memory.free(),
            [
                PageRange { start: 2, end: 3 },
                PageRange { start: 22, end: 24 }
            ]
        );
        assert_eq!(
            memory.stats(),
            BootStats {
                present_pages: 13,
                reserved_pages: 6,
                metadata_pages: 4,
                free_pages: 3
            }
        );
    }
    /// 每个非法或不可满足参数独立失败，原 metadata 与账务不变。
    #[test]
    #[case("fragmented", 3, 1)]
    #[case("zero_pages", 0, 1)]
    #[case("zero_alignment", 1, 0)]
    #[case("non_power_two", 1, 3)]
    #[case("size_overflow", usize::MAX, 1)]
    #[case("align_overflow", 1, 1usize << (usize::BITS - 1))]
    fn allocation_rejection(pages: usize, alignment: usize) {
        let mut memory = const { BootMemory::empty() };
        deducted(&mut memory);
        let before = memory.stats();
        assert!(memory.allocate(pages, alignment).is_err());
        assert_eq!(memory.stats(), before);
        assert_eq!(memory.metadata(), [PageRange { start: 6, end: 10 }]);
    }
    /// 两片未对齐且相邻的字节 RAM 共同覆盖一页，metadata 居中切分 free。
    #[test]
    fn adjacent_byte_fragments() {
        let ram = [
            Region {
                start: PAGE_SIZE,
                end: PAGE_SIZE + PAGE_SIZE / 2,
            },
            Region {
                start: PAGE_SIZE + PAGE_SIZE / 2,
                end: 8 * PAGE_SIZE,
            },
        ];
        let mut memory = const { BootMemory::empty() };
        memory.initialize(&ram, &[]).unwrap();
        assert_eq!(memory.ram(), [PageRange { start: 1, end: 8 }]);
        assert_eq!(memory.allocate(2, 4), Ok(PageRange { start: 4, end: 6 }));
        assert_eq!(
            memory.free(),
            [
                PageRange { start: 1, end: 4 },
                PageRange { start: 6, end: 8 }
            ]
        );
        assert_eq!(memory.metadata(), [PageRange { start: 4, end: 6 }]);
        assert_eq!(
            memory.stats(),
            BootStats {
                present_pages: 7,
                reserved_pages: 0,
                metadata_pages: 2,
                free_pages: 5
            }
        );
    }
    /// 原始输入的对齐溢出、片段数量与无效区间分别独立拒绝。
    #[test]
    #[case("ram_overflow", 0)]
    #[case("reserved_overflow", 1)]
    #[case("ram_count", 2)]
    #[case("reserved_count", 3)]
    #[case("empty_region", 4)]
    fn input_rejection(kind: usize) {
        let mut ram = [Region {
            start: 0,
            end: 32 * PAGE_SIZE,
        }; MAX_MEMORY_REGIONS + 1];
        let mut reserved = [Region { start: 0, end: 1 }; MAX_RESERVED_REGIONS + 1];
        let (ram_count, reserved_count) = match kind {
            0 => {
                ram[0] = Region {
                    start: usize::MAX - 1,
                    end: usize::MAX,
                };
                (1, 0)
            }
            1 => {
                reserved[0] = Region {
                    start: usize::MAX - 1,
                    end: usize::MAX,
                };
                (1, 1)
            }
            2 => (MAX_MEMORY_REGIONS + 1, 0),
            3 => (1, MAX_RESERVED_REGIONS + 1),
            _ => {
                ram[0] = Region { start: 10, end: 10 };
                (1, 0)
            }
        };
        let mut memory = const { BootMemory::empty() };
        assert!(memory
            .initialize(&ram[..ram_count], &reserved[..reserved_count])
            .is_err());
    }
    /// 每次成功申请消费一个固定 metadata 额度，超限不改已分配页。
    #[test]
    fn metadata_capacity() {
        let limit = 1 + MAX_MEMORY_REGIONS;
        let mut memory = const { BootMemory::empty() };
        memory
            .initialize(
                &[Region {
                    start: 0,
                    end: (limit + 1) * PAGE_SIZE,
                }],
                &[],
            )
            .unwrap();
        for pfn in 0..limit {
            assert_eq!(
                memory.allocate(1, 1),
                Ok(PageRange {
                    start: pfn,
                    end: pfn + 1
                })
            );
        }
        let before = memory.stats();
        assert!(memory.allocate(1, 1).is_err());
        assert_eq!(memory.stats(), before);
        assert_eq!(
            memory.metadata(),
            [PageRange {
                start: 0,
                end: limit
            }]
        );
    }
}

#[ktest::suite(context = "Boot", tags("mm", "buddy"), timeout_ms = 10000)]
mod controlled_buddy {
    use super::*;
    /// 每阶独立真实拆分和合并，最大块恢复不依赖上一参数。
    #[test]
    #[case("order0", 0)]
    #[case("order1", 1)]
    #[case("order2", 2)]
    #[case("order3", 3)]
    #[case("order4", 4)]
    #[case("order5", 5)]
    #[case("order6", 6)]
    #[case("order7", 7)]
    #[case("order8", 8)]
    #[case("order9", 9)]
    #[case("order10", 10)]
    fn split_merge(order: usize) {
        buddy::with_test_zone(false, |zone| {
            assert!(zone.allocate(0, order));
            let (head, actual) = zone.block(0);
            assert_eq!(actual, order);
            assert_eq!(head % (1 << order), 0);
            assert!(head + (1 << order) <= 1024);
            assert_eq!(zone.free(), 1024 - (1 << order));

            zone.release(0);
            assert!(zone.allocate(0, 10));
            assert_eq!(zone.block(0), (0, 10));
            assert_eq!(zone.free(), 0);
            zone.release(0);
            assert_eq!(zone.free(), 1024);
        });
    }
    /// 混阶填满原池，live 块不重叠，非顺序归还后恢复原最大块。
    #[test]
    fn mixed_orders() {
        buddy::with_test_zone(false, |zone| {
            assert!(!zone.allocate(0, 11));
            assert_eq!(zone.free(), 1024);
            assert!(zone.allocate(0, 10));
            assert_eq!(zone.block(0), (0, 10));
            assert_eq!(zone.free(), 0);
            zone.release(0);

            for index in 0..=10 {
                let order = if index == 10 { 0 } else { index };
                let before = zone.free();
                assert!(zone.allocate(index, order));
                let (head, actual) = zone.block(index);
                assert_eq!(actual, order);
                assert_eq!(head % (1 << order), 0);
                assert!(head + (1 << order) <= 1024);
                assert_eq!(zone.free(), before - (1 << order));
                for other in 0..index {
                    let (b, o) = zone.block(other);
                    assert!(head + (1 << order) <= b || b + (1 << o) <= head);
                }
            }
            assert_eq!(zone.free(), 0);
            assert!(!zone.allocate(11, 0));

            for index in [9, 0, 7, 2, 10, 4, 1, 8, 3, 6, 5] {
                zone.release(index);
            }
            assert_eq!(zone.free(), 1024);
            assert!(zone.allocate(0, 10));
            assert_eq!(zone.block(0), (0, 10));
            zone.release(0);
            assert_eq!(zone.free(), 1024);
        });
    }
    /// 原保留页及 section 空洞切断连续块，耗尽与归还只消费实际可用描述符。
    #[test]
    fn fragmented_exhaustion() {
        buddy::with_test_zone(true, |zone| {
            assert_eq!(zone.free(), 14);
            assert!(!zone.allocate(0, 3));
            assert_eq!(zone.free(), 14);
            for index in 0..14 {
                assert!(zone.allocate(index, 0));
                let (head, order) = zone.block(index);
                assert_eq!(order, 0);
                assert!(
                    (head < 15 && ![3, 7, 11].contains(&head)) || (65536..65538).contains(&head)
                );
                for other in 0..index {
                    assert_ne!(zone.block(other).0, head);
                }
            }
            assert_eq!(zone.free(), 0);
            assert!(!zone.allocate(14, 0));

            for step in 0..14 {
                zone.release((step * 5) % 14);
            }
            assert_eq!(zone.free(), 14);
            assert!(!zone.allocate(0, 2));
            assert_eq!(zone.free(), 14);
            for index in 0..5 {
                assert!(zone.allocate(index, 1));
            }
            assert!(!zone.allocate(5, 1));
            assert_eq!(zone.free(), 4);

            for index in 0..5 {
                zone.release(index);
            }
            assert_eq!(zone.free(), 14);
        });
    }
    /// 可达窗口在原大块内选择右子块，拒绝越界后仍按原阶完整合并。
    #[test]
    fn constrained_split_merge() {
        buddy::with_test_zone(false, |zone| {
            assert!(zone.allocate_in(0, 2, 9, 20));
            assert_eq!(zone.block(0), (12, 2));
            assert_eq!(zone.free(), 1020);
            assert!(!zone.allocate_in(1, 2, 13, 19));
            assert_eq!(zone.free(), 1020);
            assert!(zone.allocate_in(1, 2, 16, 20));
            assert_eq!(zone.block(1), (16, 2));
            zone.release(0);
            zone.release(1);
            assert!(zone.allocate(0, 10));
            assert_eq!(zone.block(0), (0, 10));
            zone.release(0);
            assert_eq!(zone.free(), 1024);
        });
    }

    /// 受限选择跳过保留页和 section 孔洞，不因存在高地址空页越过 mask。
    #[test]
    fn constrained_reserved_and_holes() {
        buddy::with_test_zone(true, |zone| {
            assert!(!zone.allocate_in(0, 2, 0, 16));
            assert_eq!(zone.free(), 14);
            for index in 0..12 {
                assert!(zone.allocate_in(index, 0, 0, 16));
                let (head, order) = zone.block(index);
                assert!(head < 16 && ![3, 7, 11, 15].contains(&head));
                assert_eq!(order, 0);
            }
            assert!(!zone.allocate_in(12, 0, 0, 16));
            assert_eq!(zone.free(), 2);

            for index in 0..12 {
                zone.release(index);
            }
            assert_eq!(zone.free(), 14);
        });
    }
    /// PageMap 逐 section 定位独立描述符，无 backing 的整 section 返回 None。
    #[test]
    fn pfn_holes() {
        let low = [
            PageDesc::new(PageKind::Reserved),
            PageDesc::new(PageKind::Metadata),
        ];
        let high = [PageDesc::new(PageKind::Available)];
        let sections = [Some(low.as_slice()), None, Some(high.as_slice())];
        let map = PageMap::test_map(&sections, &[]);
        assert_eq!(map.get(0).unwrap().kind(), PageKind::Reserved);
        assert_eq!(map.get(1).unwrap().kind(), PageKind::Metadata);
        assert!(map.get(2).is_none());
        assert!(map.get(32768).is_none());
        assert!(map.get(usize::MAX).is_none());
        assert_eq!(map.get(65536).unwrap().kind(), PageKind::Available);
        assert!(map.get(65537).is_none());
    }
}

/// 判断原字节 RAM 是否完整覆盖该页，允许相邻资源联合覆盖。
fn ram_page(info: &BootInfo, pfn: usize) -> bool {
    let mut covered = pfn * PAGE_SIZE;
    let end = covered + PAGE_SIZE;
    while covered < end {
        let next = info.memory[..info.memory_count]
            .iter()
            .filter(|r| r.start <= covered && covered < r.end)
            .map(|r| r.end)
            .max()
            .unwrap_or(covered);
        if next == covered {
            return false;
        }
        covered = next;
    }
    true
}

/// 检查真实不可变启动范围包含 PFN，不赋予物理访问权。
fn contains(ranges: &[PageRange], pfn: usize) -> bool {
    ranges.iter().any(|r| r.start <= pfn && pfn < r.end)
}

#[ktest::suite(context = "Thread", tags("mm", "pagemap"), timeout_ms = 10000)]
mod real_page_map {
    use super::*;
    /// 逐 PFN 原分类、空洞、reserved/metadata/free 互斥与六分类计数。
    #[test]
    fn classifications() {
        let map = crate::mm::pages();
        let info = crate::boot::info();
        let (sections, metadata_ranges) = map.test_layout_parts();
        let mut counts = [0usize; 4];
        let mut present_sections = 0;
        for (section_index, section) in sections.iter().enumerate() {
            let mut has_ram = false;
            for offset in 0..32768 {
                let pfn = section_index * 32768 + offset;
                let ram = ram_page(info, pfn);
                has_ram |= ram;
                let reserved = ram
                    && info.reserved[..info.reserved_count]
                        .iter()
                        .any(|r| r.start / PAGE_SIZE <= pfn && pfn < r.end.div_ceil(PAGE_SIZE));
                let metadata = contains(metadata_ranges, pfn);
                let free = contains(map.free_ranges(), pfn);
                assert!(!((reserved || !ram) && (metadata || free)));
                assert!(!(metadata && free));
                let kind = if !ram {
                    PageKind::Absent
                } else if reserved {
                    counts[1] += 1;
                    PageKind::Reserved
                } else if metadata {
                    counts[2] += 1;
                    PageKind::Metadata
                } else {
                    assert!(free);
                    counts[3] += 1;
                    PageKind::Available
                };
                counts[0] += usize::from(ram);
                match map.get(pfn) {
                    Some(desc) => assert_eq!(desc.kind(), kind),
                    None => assert!(!ram && section.is_none()),
                }
            }
            assert_eq!(has_ram, section.is_some());
            present_sections += usize::from(section.is_some());
        }

        let stats = map.stats();
        assert_eq!(
            counts,
            [
                stats.present_pages,
                stats.reserved_pages,
                stats.metadata_pages,
                stats.free_pages
            ]
        );
        assert_eq!(counts[0], counts[1] + counts[2] + counts[3]);
        assert_eq!(present_sections, map.present_sections());

        let mut previous = 0;
        let mut listed = 0;
        for range in map.free_ranges() {
            assert!(range.start < range.end && previous <= range.start);
            listed += range.end - range.start;
            previous = range.end;
        }
        assert_eq!(listed, counts[3]);
    }
    /// 根索引长度来自实际 RAM 最大完整页，边界与 usize::MAX 不越界。
    #[test]
    fn index_boundaries() {
        let map = crate::mm::pages();
        let info = crate::boot::info();
        let maximum = info.memory[..info.memory_count]
            .iter()
            .filter_map(|r| {
                let end = r.end / PAGE_SIZE;
                end.checked_sub(1)
                    .filter(|&p| ram_page(info, p))
                    .map(|_| end)
            })
            .max()
            .unwrap_or(0);
        assert_eq!(map.section_count(), maximum.div_ceil(32768));
        assert!(map.get(map.section_count() * 32768).is_none());
        assert!(map.get(usize::MAX).is_none());
    }
    /// 永久根/范围/描述符 backing 均属于 metadata，绝不出现在 free。
    #[test]
    fn permanent_metadata() {
        let map = crate::mm::pages();
        let (sections, metadata) = map.test_layout_parts();
        let mut regions = [(0usize, 0usize); 3];
        regions[0] = (sections.as_ptr() as usize, core::mem::size_of_val(sections));
        regions[1] = (
            map.free_ranges().as_ptr() as usize,
            core::mem::size_of_val(map.free_ranges()),
        );
        regions[2] = (metadata.as_ptr() as usize, core::mem::size_of_val(metadata));
        for (address, bytes) in
            regions
                .iter()
                .copied()
                .chain(sections.iter().flatten().map(|descriptors| {
                    assert_eq!(descriptors.len(), 32768);
                    (
                        descriptors.as_ptr() as usize,
                        core::mem::size_of_val(*descriptors),
                    )
                }))
        {
            if bytes == 0 {
                continue;
            }
            let physical = crate::arch::virt_to_phys(address);
            for pfn in physical / PAGE_SIZE..(physical + bytes).div_ceil(PAGE_SIZE) {
                assert_eq!(map.get(pfn).unwrap().kind(), PageKind::Metadata);
                assert!(contains(metadata, pfn));
                assert!(!contains(map.free_ranges(), pfn));
            }
        }
    }
    /// 内核及 DTB 与 RAM 交集均保留，不随 suite 选择顺序改变。
    #[test]
    #[case("kernel", crate::boot::kernel_region())]
    #[case("dtb", crate::boot::info().dtb)]
    fn reserved_resources(region: Region) {
        if region.start == region.end {
            return;
        }
        let map = crate::mm::pages();
        for pfn in region.start / PAGE_SIZE..region.end.div_ceil(PAGE_SIZE) {
            assert!(!contains(map.free_ranges(), pfn));
            assert!(map
                .get(pfn)
                .is_none_or(|d| matches!(d.kind(), PageKind::Absent | PageKind::Reserved)));
        }
    }
}
