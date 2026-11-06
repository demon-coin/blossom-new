//! 启动期区间账本（`BootMemory`）、受控伙伴分配器（`buddy`）与物理页描述符映射（`PageMap`）的算法单元测试。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `mm::ktest` 下，负责在不干扰全局伙伴分配器运行状态的前提下，
//! 直接对物理内存管理的核心数据结构与算法进行白盒与实机双重验证：
//! 1. `boot_ranges`：验证 `BootMemory` 对固件提供的字节级 RAM 区间与保留区（Reserved）的取整、合并、交集扣除与元数据（Metadata）分配。
//! 2. `controlled_buddy`：通过 `buddy::with_test_zone` 在受控内存池上验证伙伴系统的跨阶拆分（Split）、伙伴合并（Merge）、地址窗口受限分配与稀疏段（Section）空洞跳过。
//! 3. `real_page_map`：遍历实机全局 `mm::pages()`（`PageMap`），核对每一个物理页帧（PFN）的四类状态（`Absent` / `Reserved` / `Metadata` / `Available`）互斥性与计数守恒。
//!
//! ## 核心机制与布局图解
//!
//! ### 1. `BootMemory` 字节区间向物理页帧（PFN）取整与扣除规则
//! ```text
//! 原始字节 RAM 区间:     [Page 1 + 1B .......... Page 5) U [Page 4 .......... Page 12 - 1B)
//! 1) 先合并相邻/重叠字节: [Page 1 + 1B ...................................... Page 12 - 1B)
//! 2) RAM 向内页对齐:            [PFN 2 ............................... PFN 11) (丢弃首尾不满一页的残片)
//! 3) Reserved 向外页对齐:             [PFN 3 ........ PFN 6)    [PFN 10 . PFN 11)
//! 4) 扣除交集得初始 Free:       [2..3)                      [6..10)
//! 5) 分配 Metadata (4页):                                   [6..10) -> 转为 Metadata
//! ```
//!
//! ### 2. 稀疏 `PageMap` 二级段表（Section Table）与頁分类
//! ```text
//! PFN 拆解: [ Section 索引 (pfn / 32768) | 段内偏移 (pfn % 32768) ]
//!
//! PageMap::sections
//!   +-- Section 0: Some(&[PageDesc; 32768])  --> 含有效 RAM 页，按页标记 Reserved / Metadata / Available
//!   +-- Section 1: None                      --> 整段 128 MiB 无 RAM，查询直接返回 None (Absent)
//!   +-- Section 2: Some(&[PageDesc; 32768])  --> 稀疏高地址 RAM 段
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `boot_ranges` 与 `controlled_buddy` 均在栈上或函数局部构造独立状态，不获取全局锁。
//! - `real_page_map` 只读访问引导期完成后即冻结的 `boot::info()` 与 `PageMap` 静态元数据布局，无需加锁即可并发安全读取。

use crate::boot::{BootInfo, Region, MAX_MEMORY_REGIONS, MAX_RESERVED_REGIONS};
use crate::mm::{
    bootmem::{BootMemory, BootStats, PageRange, PAGE_SIZE},
    buddy,
    page::{PageDesc, PageKind, PageMap},
};

/// 用精心构造的跨页、重叠、未对齐及含空洞的 RAM 与 Reserved 字节区间初始化调用方传入的空 `BootMemory` 账本。
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

/// 在 `ranges` 初始化的基础上，连续执行两次带对齐要求的元数据（Metadata）页分配，将 PFN `[6, 10)` 从空闲区间转为元数据区间。
fn deducted(memory: &mut BootMemory) {
    ranges(memory);
    assert_eq!(memory.allocate(2, 4), Ok(PageRange { start: 8, end: 10 }));
    assert_eq!(memory.allocate(2, 2), Ok(PageRange { start: 6, end: 8 }));
}

#[ktest::suite(context = "Boot", tags("mm", "bootmem"), timeout_ms = 10000)]
mod boot_ranges {
    use super::*;
    /// 测试目标：验证 `BootMemory` 对多段重叠或未对齐的原始字节 RAM 区间先合并再向内页对齐（向下取整终点、向上取整起点）的规则。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建一个空的 `BootMemory` 账本。
    /// 2. When：调用 `ranges(&mut memory)` 载入包含重叠、跨页未对齐以及不足一页碎片的 5 段 RAM 区间。
    /// 3. Then：重叠区间合并并向内收缩到完整页边界后得到 `[PFN 2, 11)` 与 `[PFN 20, 24)`，第 40 页内仅 1 字节的碎片被丢弃。
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
    /// 测试目标：验证 `BootMemory` 对保留（Reserved）字节区间向外页对齐后，仅保留其与有效 RAM 区间的交集。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建一个空的 `BootMemory` 账本。
    /// 2. When：调用 `ranges(&mut memory)` 载入跨页未对齐、相邻以及落在 RAM 空洞中的保留区间。
    /// 3. Then：保留区间向外扩展到页边界并与 RAM 求交集后，合并为 `[3, 6)`、`[10, 11)` 和 `[20, 22)`，完全位于 RAM 外的 `[30, 31)` 被忽略。
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
    /// 测试目标：验证初始化后的空闲页区间（Free）等于有效 RAM 扣除保留区（Reserved），且四类页统计满足总数守恒。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建一个空的 `BootMemory` 账本并调用 `ranges(&mut memory)` 完成初始化。
    /// 2. When：读取空闲区间列表 `memory.free()` 与统计信息 `memory.stats()`。
    /// 3. Then：空闲区间为 `[2, 3)`、`[6, 10)`、`[22, 24)` 共 7 页；总有效页 `present_pages`（13）等于 `reserved_pages`（6）加 `free_pages`（7），`metadata_pages` 为 0。
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
    /// 测试目标：验证从 `BootMemory` 分配对齐的元数据页后，相邻元数据区间自动合并，且空闲页计数相应扣减。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建一个空的 `BootMemory` 账本。
    /// 2. When：调用 `deducted(&mut memory)` 先分配 4 页对齐的 2 页（落入 `[8, 10)`），再分配 2 页对齐的 2 页（落入 `[6, 8)`）。
    /// 3. Then：两段分配合并为单一元数据区间 `[6, 10)`；原 `[6, 10)` 空闲区间被完全消耗，剩余空闲页减少 4 页变为 3 页。
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
    /// 测试目标：验证 `BootMemory::allocate` 对连续空闲页不足、零页数、零对齐、非 2 的幂次对齐及整数溢出的请求均返回错误且不破坏既有状态。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：初始化并执行 `deducted(&mut memory)`，此时剩余空闲区间为 `[2, 3)`（1 页）和 `[22, 24)`（2 页）。
    /// 2. When：使用参数化用例依次尝试分配 3 页（碎片化无法满足）、0 页、0 对齐、对齐为 3、页数溢出和对齐溢出。
    /// 3. Then：`allocate` 均返回 `Err`，且调用前后的 `stats()` 与 `metadata()` 区间保持不变。
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
    /// 测试目标：验证两段在页内半页边界处首尾相接的字节 RAM 能够先合并再向内取整，避免丢失交界处的完整物理页，并支持从空闲区间中部切分元数据。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：构造两段在 `PAGE_SIZE + PAGE_SIZE / 2` 处无缝拼接的 RAM 字节区间 `[1.0 页, 1.5 页)` 与 `[1.5 页, 8.0 页)`。
    /// 2. When：初始化 `BootMemory` 后，请求分配 4 页对齐的 2 页元数据。
    /// 3. Then：第 1 页完整保留，有效 RAM 为 `[1, 8)` 共 7 页；4 页对齐分配命中 `[4, 6)`，将原空闲区间居中切分为 `[1, 4)` 与 `[6, 8)`。
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
    /// 测试目标：验证 `BootMemory::initialize` 对地址向上取整溢出、区间数量超限以及空区间（`start == end`）等非法输入的拒绝行为。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：构造测试用的 RAM 与 Reserved 区间数组。
    /// 2. When：根据用例类型分别注入接近 `usize::MAX` 的溢出地址、超过 `MAX_MEMORY_REGIONS` / `MAX_RESERVED_REGIONS` 的区间切片，以及 `start == end` 的空区间。
    /// 3. Then：`memory.initialize(...)` 均返回 `Err`。
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
    /// 测试目标：验证当不连续的元数据分配耗尽 `BootMemory` 内部固定容量槽位时，后续无法合并的新分配会失败且不修改已分配账本。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：初始化一段包含 `limit + 1` 页的连续 RAM，并循环逐页分配 `[0, limit)` 共 `limit` 页（合并为单一段 `[0, limit)`）。
    /// 2. When：继续尝试分配最后第 `limit` 页时，验证容量上限边界与账本状态。
    /// 3. Then：超出容量上限的分配返回 `Err`，已有的统计信息与元数据区间 `[0, limit)` 保持不变。
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
    /// 测试目标：验证伙伴分配器在 `0..=10` 各个阶数（Order）下的块拆分、对齐不变量以及释放后的伙伴合并恢复能力。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：通过 `buddy::with_test_zone(false, ...)` 创建一个包含 1024 页（单个 Order 10 块）的连续受控伙伴内存池。
    /// 2. When：先分配一个指定 `order` 的块并检查其起始页帧对齐与剩余空闲页数，随后将其释放，再尝试分配一个完整的 Order 10 最大块。
    /// 3. Then：首次分配得到的块阶数等于 `order`、起始页号按 `1 << order` 对齐且空闲页数精确减少 `1 << order`；释放后伙伴链逐级完整合并回单个 Order 10 块，最终释放后 1024 页全部恢复空闲。
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
    /// 测试目标：验证混合阶数（Order 0..=9 再加一个 Order 0）恰好耗尽整个 1024 页内存池时各块互不重叠，且按乱序释放后仍能完整合并回单个 Order 10 大块。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建一个 1024 页的连续受控伙伴内存池，先验证超阶请求（Order 11）失败且 Order 10 分配释放正常。
    /// 2. When：依次分配 Order `0, 1, 2, ..., 9` 以及最后一个 Order `0` 共 11 个块（总页数 $1 + 2 + 4 + \dots + 512 + 1 = 1024$），验证池耗尽后，按固定乱序 `[9, 0, 7, 2, 10, 4, 1, 8, 3, 6, 5]` 逐一释放全部块。
    /// 3. Then：已分配的 11 个块两两物理区间互不重叠，分配完后空闲页为 0 且新分配失败；乱序释放完成后空闲页恢复为 1024，且可再次分配出完整的 Order 10 块。
    #[test]
    fn mixed_orders() {
        buddy::with_test_zone(false, |zone| {
            // Step 1: 验证超最大阶（Order 11）分配失败，且完整 Order 10 块可正常分配与释放。
            assert!(!zone.allocate(0, 11));
            assert_eq!(zone.free(), 1024);
            assert!(zone.allocate(0, 10));
            assert_eq!(zone.block(0), (0, 10));
            assert_eq!(zone.free(), 0);
            zone.release(0);

            // Step 2: 依次分配 Order 0..=9 及最后一个 Order 0 块以耗尽全部 1024 页，并校验已分配块两两不重叠。
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

            // Step 3: 按固定乱序释放全部 11 个块，验证伙伴系统能够逐级合并恢复回完整的 Order 10 块。
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
    /// 测试目标：验证含保留页（Reserved）与稀疏段（Section）空洞的碎片化伙伴池在初始化、单页耗尽、乱序释放及最大可合并阶数上的正确性。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：通过 `buddy::with_test_zone(true, ...)` 创建含保留页（PFN 3, 7, 11, 15）与跨段高地址页（PFN 65536..65538）的稀疏测试池（共 14 个可用页）。
    /// 2. When：先尝试分配 Order 3（8 页连续块），随后循环分配 14 个单页（Order 0）耗尽内存池；接着按步长 5 乱序释放全部 14 页，再尝试分配 Order 2（4 页块）和 5 个 Order 1（2 页块）。
    /// 3. Then：因每 4 页中都有保留页打断，Order 3 和 Order 2 均无法分配；14 个单页均避开保留页与空洞且互不重复；乱序释放后恰好可合并出 5 个 Order 1 块（共 10 页），剩余 4 个孤立单页。
    #[test]
    fn fragmented_exhaustion() {
        buddy::with_test_zone(true, |zone| {
            // Step 1: 验证保留页阻断了 Order 3 连续块，随后循环分配 14 个单页（Order 0）耗尽稀疏池。
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

            // Step 2: 乱序释放全部 14 页后，验证保留页仍阻断 Order 2，但可分配出 5 个 Order 1 块并最终归还。
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
    /// 测试目标：验证带物理页帧窗口限制的分配接口 `allocate_in` 能在大块拆分时精准挑选落在窗口内的右子块，并在释放后恢复完整大块。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建一个 1024 页的连续受控伙伴内存池。
    /// 2. When：请求在窗口 `[9, 20)` 内分配 Order 2（4 页对齐块），接着请求在不足以容纳 4 页对齐块的窗口 `[13, 19)` 内分配 Order 2，再在 `[16, 20)` 内分配 Order 2，最后释放两个块并尝试分配 Order 10。
    /// 3. Then：第一次分配跳过未对齐的前缀，拆分并选中窗口内的 `[12, 16)`；第二次因 `[13, 19)` 内无 4 页对齐的完整块而失败；第三次成功选中 `[16, 20)`；全部释放后完整合并回 `[0, 1024)` 的 Order 10 大块。
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

    /// 测试目标：验证窗口受限分配 `allocate_in` 在含保留页与稀疏空洞的内存池中严格遵守 `[start, end)` 上下界，不会误分配高地址段中的空闲页。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：创建稀疏测试池（低地址段 `[0, 16)` 内含 12 个可用单页和 4 个保留页，高地址段 `65536..65538` 含 2 个可用页，总计 14 页）。
    /// 2. When：限定在低地址窗口 `[0, 16)` 内尝试分配 Order 2，随后连续分配 12 个单页（Order 0），并尝试分配第 13 个单页。
    /// 3. Then：Order 2 因保留页阻断而失败；12 次单页分配全部落在 `[0, 16)` 内且避开保留页 `[3, 7, 11, 15]`；第 13 次分配因窗口外的高地址 2 页不在 `[0, 16)` 范围内而被拒绝，此时全局仍剩 2 页空闲。
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
    /// 测试目标：验证 `PageMap` 的稀疏二级段表（Section Table）能够按 `PFN / 32768` 独立定位页描述符，并对未分配后台数组的空洞段或段内越界偏移返回 `None`。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：构造包含 3 个 Section 槽位的测试 `PageMap`：Section 0 含 2 页（`Reserved`、`Metadata`），Section 1 为 `None`（空洞），Section 2（起始 PFN 65536）含 1 页（`Available`）。
    /// 2. When：分别查询 PFN `0`、`1`、`2`（Section 0 内越界）、`32768`（Section 1 空洞）、`usize::MAX`、`65536` 和 `65537`（Section 2 内越界）。
    /// 3. Then：有效 PFN 返回对应的 `PageKind`，段内越界、空洞段及 `usize::MAX` 均安全返回 `None`。
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

/// 检查原始引导信息 `BootInfo` 中的字节级 RAM 区间是否完整覆盖了物理页帧 `pfn` 对应的 `[pfn * PAGE_SIZE, (pfn + 1) * PAGE_SIZE)` 范围（支持多段相邻区间拼接覆盖）。
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

/// 判断给定的半开页帧区间列表 `ranges` 中是否包含指定的物理页帧号 `pfn`。
fn contains(ranges: &[PageRange], pfn: usize) -> bool {
    ranges.iter().any(|r| r.start <= pfn && pfn < r.end)
}

#[ktest::suite(context = "Thread", tags("mm", "pagemap"), timeout_ms = 10000)]
mod real_page_map {
    use super::*;
    /// 测试目标：验证实机全局 `PageMap` 中所有 Section 的每个 PFN 状态分类与引导信息严格一致，且 `Reserved`、`Metadata`、`Available` 互斥并与统计汇总完全吻合。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：获取内核全局 `mm::pages()`（`PageMap`）与 `boot::info()`。
    /// 2. When：遍历所有 Section 及其内部的 32768 个 PFN，根据 `boot::info()` 独立计算每个 PFN 是否属于 RAM、Reserved、Metadata 或 Free，并与 `map.get(pfn)` 返回的 `PageDesc::kind()` 逐一比对，同时累加各分类页数。
    /// 3. Then：每个 PFN 的分类与预期完全一致，不存在无 RAM 却分配了 Section 数组的空段；`present_pages == reserved_pages + metadata_pages + free_pages` 恒成立，且 `free_ranges()` 严格递增不重叠、总页数等于 `free_pages`。
    #[test]
    fn classifications() {
        // Step 1: 遍历二级段表中的所有 Section 与段内 32768 个 PFN，逐页核对状态分类互斥性并累加计数。
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

        // Step 2: 校验累加的四类页帧计数与 PageMap::stats() 统计快照完全相等且总数守恒。
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

        // Step 3: 校验 free_ranges() 列表严格递增不重叠，且区间长度之和等于空闲页总数。
        let mut previous = 0;
        let mut listed = 0;
        for range in map.free_ranges() {
            assert!(range.start < range.end && previous <= range.start);
            listed += range.end - range.start;
            previous = range.end;
        }
        assert_eq!(listed, counts[3]);
    }
    /// 测试目标：验证实机 `PageMap` 的根 Section 数组长度恰好覆盖最高有效 RAM 页所在的段，且段表上界与 `usize::MAX` 查询均安全返回 `None`。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：读取全局 `mm::pages()` 与 `boot::info()`。
    /// 2. When：找出 `boot::info()` 中最后一个完整有效 RAM 页帧的上界 `maximum`，计算所需 Section 数量 `maximum.div_ceil(32768)`，并查询越界 PFN `section_count * 32768` 与 `usize::MAX`。
    /// 3. Then：`map.section_count()` 与计算值严格相等，越界 PFN 查询均返回 `None`。
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
    /// 测试目标：验证承载 `PageMap` 自身数据结构（根 Section 切片、`free_ranges` 切片、`metadata` 切片及各段 `PageDesc` 数组）的物理页全部被标记为 `PageKind::Metadata`，绝不会混入空闲页池。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：从 `mm::pages()` 提取根段表、空闲区间表、元数据区间表以及所有非空 Section 的 `PageDesc` 切片虚拟地址与字节长度。
    /// 2. When：将每段切片的虚拟地址转换为物理地址，遍历其跨越的所有物理页帧 `pfn`。
    /// 3. Then：每一个承载 `PageMap` 元数据的物理页在 `map.get(pfn)` 中均标记为 `PageKind::Metadata`，包含在 `metadata` 区间内，且不在 `free_ranges()` 中。
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
    /// 测试目标：验证内核镜像（Kernel）与设备树（DTB）所占用的物理页帧均被排除在可分配空闲区间之外，且在 `PageMap` 中标记为 `Reserved`（或位于 RAM 外的 `Absent`）。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：通过参数化用例分别传入 `boot::kernel_region()` 与 `boot::info().dtb` 物理字节区间。
    /// 2. When：若区间非空，遍历其向外页对齐覆盖的所有物理页帧 `pfn`。
    /// 3. Then：没有任何一个 `pfn` 出现在 `map.free_ranges()` 中，且 `map.get(pfn)` 仅可能为 `None`、`PageKind::Absent` 或 `PageKind::Reserved`。
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
