//! 启动早期无堆物理内存区间账本（Boot-time Memory Allocator）。
//!
//! ## 模块职责与架构定位
//! 在伙伴系统与内核堆尚未就绪的启动最早期，本模块使用栈上固定容量的有序区间表（`Ranges`），
//! 将平台固件上报的原始字节级物理内存区间（`RAM`）与保留区间（`Reserved`）归并、对齐并转化为
//! 4 KiB 物理页帧号（PFN）区间，同时为 `page::initialize` 提供连续物理页的静态切分分配能力。
//!
//! - **上游调用者**：`mm::initialize` 创建并初始化 [`BootMemory`]，随后传递给 `page::initialize`
//!   划拨描述符元数据页，初始化完成后该栈对象即被丢弃。
//! - **下游依赖**：仅依赖 `crate::boot::Region` 与编译期容量常量，不依赖任何动态内存分配或硬件锁。
//!
//! ## 页边界取整与四域互斥划分规则
//! 为防止越界踩踏固件保留内存或未对齐的 RAM 边缘，物理字节区间按保守方向取整为半开 PFN 区间 `[start, end)`：
//!
//! ```text
//! 原始 RAM 字节区间:       [ ram.start ─────────────────────────── ram.end )
//!   向内对齐 (inward=true):    [ ceil(start/4K) ─────────── floor(end/4K) )  ==> present RAM 页集合
//!
//! 原始 Reserved 字节区间:         [ res.start ───────── res.end )
//!   向外对齐 (inward=false):   [ floor(start/4K) ──── ceil(end/4K) )         ==> 保留页掩码集合
//!
//! 交集扣除与元数据切分后，所有 present RAM 页严格划分为三个互不相交的子集：
//! ┌───────────────────┬───────────────────────────────────────┬──────────────────────────┐
//! │   reserved 集合   │             metadata 集合             │        free 集合         │
//! │ (RAM ∩ Reserved)  │ (从 free 切出的 PageMap/PageDesc 页)  │ (最终移交伙伴系统的空闲页)│
//! └───────────────────┴───────────────────────────────────────┴──────────────────────────┘
//! 恒等式: present_pages == reserved_pages + metadata_pages + free_pages
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 本模块纯属启动早期 BSP 关中断单线程使用的局部数据结构，内部不包含任何锁，也不跨核共享。

use crate::boot::{Region, MAX_MEMORY_REGIONS, MAX_RESERVED_REGIONS};

/// 标准基本物理页大小的位移位数（$2^{12} = 4096$ 字节）。
pub(crate) const PAGE_SHIFT: usize = 12;
/// 标准基本物理页的字节大小（4096 字节 / 4 KiB）。
pub(crate) const PAGE_SIZE: usize = 1 << PAGE_SHIFT;

/// 启动期元数据连续页分配的最大次数上界：1 次 `PageMap` 根表分配 + 每段物理 RAM 至多 1 次 `PageDesc` 承载页分配。
const MAX_METADATA_ALLOCATIONS: usize = 1 + MAX_MEMORY_REGIONS;
/// 单个有序区间表 [`Ranges`] 的最大条目容量，足以容纳所有 RAM 段、保留段与元数据切分产生的碎片区间。
pub(crate) const RANGE_CAPACITY: usize =
    MAX_MEMORY_REGIONS + MAX_RESERVED_REGIONS + MAX_METADATA_ALLOCATIONS;

/// 半开区间 `[start, end)`，在归并阶段可暂存字节地址，对齐完成后表示物理页帧号（PFN，单位：4 KiB 页）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct PageRange {
    /// 区间起始位置（含），规范化后表示起始 PFN。
    pub start: usize,
    /// 区间结束位置（不含），满足 `start < end` 时有效，规范化后表示结束 PFN。
    pub end: usize,
}

/// 启动早期物理页四类角色的统计计数（单位均为 4 KiB 物理页）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BootStats {
    /// 向内页对齐后的真实可用物理 RAM 总页数。
    pub present_pages: usize,
    /// 位于 `present_pages` 范围内、因与内核镜像/DTB/固件保留区重叠而不可分配的页数。
    pub reserved_pages: usize,
    /// 从空闲 RAM 中切分出来用于存放 `PageMap` 根表与 `PageDesc` 描述符数组的页数。
    pub metadata_pages: usize,
    /// 扣除 `reserved_pages` 与 `metadata_pages` 后剩余、即将移交给伙伴系统的空闲页数。
    pub free_pages: usize,
}

/// 固定容量的规范化有序区间集合，内部始终维持“按起点升序、互不重叠且互不相邻”的不变量。
struct Ranges {
    /// 存放区间的定长数组，仅前 `count` 项为有效区间。
    entries: [PageRange; RANGE_CAPACITY],
    /// 当前已存入的有效不相邻区间数量，满足 `count <= RANGE_CAPACITY`。
    count: usize,
}

impl Ranges {
    /// 构造空的定长有序区间表，可在编译期或栈上直接初始化而不依赖堆内存。
    const fn new() -> Self {
        Self {
            entries: [PageRange { start: 0, end: 0 }; RANGE_CAPACITY],
            count: 0,
        }
    }

    /// 返回当前已记录的全部有效区间切片；切片内元素按 `start` 严格递增且互不相邻（`entries[i].end < entries[i+1].start`）。
    fn as_slice(&self) -> &[PageRange] {
        &self.entries[..self.count]
    }

    /// 将半开区间 `range` 插入集合，自动合并所有与之重叠或首尾紧邻的已有区间。
    ///
    /// 若 `range` 为空（`end <= start`）则直接忽略；若插入独立新区间会导致超出 `RANGE_CAPACITY`，
    /// 则在修改任何已有元素前返回错误，保证失败时原集合状态完好无损。
    fn insert(&mut self, mut range: PageRange) -> Result<(), &'static str> {
        if range.end <= range.start {
            return Ok(());
        }

        // Step 1: 二分查找第一个 `end >= range.start` 的区间位置，定位可能发生合并的起始下标 `first`。
        let first = self
            .as_slice()
            .partition_point(|entry| entry.end < range.start);
        // Step 2: 向右扫描所有满足 `entry.start <= range.end`（重叠或紧邻）的区间，将其并入 `range`。
        let mut last = first;
        while last < self.count && self.entries[last].start <= range.end {
            range.start = range.start.min(self.entries[last].start);
            range.end = range.end.max(self.entries[last].end);
            last += 1;
        }

        // Step 3: 若未与任何现有区间合并则检查容量并右移腾出空槽；否则收缩被吞并的多余槽位，最后写入合并结果。
        if first == last {
            if self.count == RANGE_CAPACITY {
                return Err("boot range capacity exceeded");
            }
            self.entries.copy_within(first..self.count, first + 1);
            self.count += 1;
        } else {
            self.entries.copy_within(last..self.count, first + 1);
            self.count -= last - first - 1;
        }
        self.entries[first] = range;
        Ok(())
    }

    /// 将一组物理字节区间先按字节合并、再统一对齐转换为 PFN 区间。
    ///
    /// 先按字节合并再做页取整至关重要：若两段相邻的半页 RAM 拼起来恰好凑满一整页，
    /// 先合并能保留这个完整物理页，避免各自向内取整时将其误丢弃。
    /// 参数 `inward` 为 `true` 时向内收缩到完整页边界（用于可用 RAM），为 `false` 时向外扩张覆盖整页（用于保留区）。
    fn initialize_regions(&mut self, regions: &[Region], inward: bool) -> Result<(), &'static str> {
        // Step 1: 校验每段原始物理字节区间非空，并在表内完成字节级重叠/相邻归并。
        for region in regions {
            if region.end <= region.start {
                return Err("invalid boot physical region");
            }
            self.insert(PageRange {
                start: region.start,
                end: region.end,
            })?;
        }

        // Step 2: 原地将合并后的字节区间按指定方向转换为 PFN 区间；向外取整可能令原本分离的两段保留区变为相邻，需顺带二次合并。
        let mut output = 0;
        for index in 0..self.count {
            let region = self.entries[index];
            let range = if inward {
                PageRange {
                    start: round_up(region.start)? / PAGE_SIZE,
                    end: region.end / PAGE_SIZE,
                }
            } else {
                PageRange {
                    start: region.start / PAGE_SIZE,
                    end: round_up(region.end)? / PAGE_SIZE,
                }
            };
            if range.start < range.end {
                if 0 < output && range.start <= self.entries[output - 1].end {
                    self.entries[output - 1].end = self.entries[output - 1].end.max(range.end);
                } else {
                    self.entries[output] = range;
                    output += 1;
                }
            }
        }
        self.count = output;
        Ok(())
    }

    /// 累加集合中所有不重叠区间的总页数。
    fn pages(&self) -> usize {
        self.as_slice()
            .iter()
            .map(|range| range.end - range.start)
            .sum()
    }
}

/// 将物理字节地址向上对齐到 `PAGE_SIZE`（4 KiB）整数倍；若加法溢出 `usize` 则返回错误。
fn round_up(address: usize) -> Result<usize, &'static str> {
    address
        .checked_add(PAGE_SIZE - 1)
        .map(|value| value & !(PAGE_SIZE - 1))
        .ok_or("boot page alignment overflows")
}

/// 启动期物理内存分类账本与连续元数据页分配器。
pub(crate) struct BootMemory {
    /// 向内对齐到整页后的全部可用物理 RAM PFN 区间集合。
    ram: Ranges,
    /// 与 `ram` 存在实际交集的保留区 PFN 区间集合（位于 RAM 之外的 MMIO 保留区不计入此处）。
    reserved: Ranges,
    /// 通过 [`Self::allocate`] 从 `free` 切分出来用于存放页管理元数据的 PFN 区间集合。
    metadata: Ranges,
    /// 当前尚未被保留或划拨给元数据的空闲可分配 PFN 区间集合。
    free: Ranges,
    /// 已执行的元数据区间分配次数，用于校验不超过 [`MAX_METADATA_ALLOCATIONS`]。
    allocation_count: usize,
}

impl BootMemory {
    /// 构造编译期常量空账本，便于调用方直接在原位初始化而无需在栈上拷贝大型临时结构体。
    pub(crate) const fn empty() -> Self {
        Self {
            ram: Ranges::new(),
            reserved: Ranges::new(),
            metadata: Ranges::new(),
            free: Ranges::new(),
            allocation_count: 0,
        }
    }

    /// 根据平台原始物理 RAM 列表与保留区列表初始化空账本，构建初始 `ram`、`reserved` 与 `free` 集合。
    ///
    /// 仅位于真实整页 `ram` 内部的保留页会被计入 `self.reserved` 并从 `self.free` 中扣除；
    /// 落在 RAM 范围外的固件保留区（如设备 MMIO 洞）自动被忽略。若返回错误，调用方必须立即中止内核启动。
    pub(crate) fn initialize(
        &mut self,
        ram: &[Region],
        reserved: &[Region],
    ) -> Result<(), &'static str> {
        assert!(
            self.ram.count == 0
                && self.reserved.count == 0
                && self.metadata.count == 0
                && self.free.count == 0
                && self.allocation_count == 0,
            "boot memory is already initialized"
        );
        if MAX_MEMORY_REGIONS < ram.len() || MAX_RESERVED_REGIONS < reserved.len() {
            return Err("boot region count exceeds capacity");
        }

        // Step 1: 分别将 RAM（向内取整）与保留区（向外取整）规范化为有序不重叠的 PFN 区间列表。
        self.ram.initialize_regions(ram, true)?;
        let mut reservations = const { Ranges::new() };
        reservations.initialize_regions(reserved, false)?;

        // Step 2: 双指针线性扫描 `ram` 与 `reservations`，将每段 RAM 切分为交替的 `free` 间隙与 `reserved` 交集。
        let mut reservation_index = 0;
        for ram in self.ram.as_slice() {
            let mut cursor = ram.start;
            while reservation_index < reservations.count
                && reservations.entries[reservation_index].end <= cursor
            {
                reservation_index += 1;
            }
            let mut index = reservation_index;
            while index < reservations.count && reservations.entries[index].start < ram.end {
                let reservation = reservations.entries[index];
                let start = reservation.start.max(cursor);
                let end = reservation.end.min(ram.end);
                self.free.insert(PageRange {
                    start: cursor,
                    end: start,
                })?;
                self.reserved.insert(PageRange { start, end })?;
                cursor = end;
                if ram.end <= reservation.end {
                    break;
                }
                index += 1;
            }
            self.free.insert(PageRange {
                start: cursor,
                end: ram.end,
            })?;
            reservation_index = index;
        }
        Ok(())
    }

    /// 返回所有真实整页物理 RAM 的有序 PFN 区间切片（不含段间物理地址空洞）。
    pub(crate) fn ram(&self) -> &[PageRange] {
        self.ram.as_slice()
    }

    /// 返回当前剩余空闲物理页的有序 PFN 区间切片。
    pub(crate) fn free(&self) -> &[PageRange] {
        self.free.as_slice()
    }

    /// 返回与真实整页 RAM 相交的启动保留页有序 PFN 区间切片。
    pub(crate) fn reserved(&self) -> &[PageRange] {
        self.reserved.as_slice()
    }

    /// 返回已划拨给页管理元数据的有序 PFN 区间切片。
    pub(crate) fn metadata(&self) -> &[PageRange] {
        self.metadata.as_slice()
    }

    /// 汇总当前四个互斥集合的物理页数账务快照。
    pub(crate) fn stats(&self) -> BootStats {
        BootStats {
            present_pages: self.ram.pages(),
            reserved_pages: self.reserved.pages(),
            metadata_pages: self.metadata.pages(),
            free_pages: self.free.pages(),
        }
    }

    /// 采用首次适应（First-Fit）策略，从空闲区间中切分出 `pages` 个满足 `alignment_pages` 对齐的连续物理页，
    /// 并将其登记到 `metadata` 集合中。
    ///
    /// 参数 `alignment_pages` 必须为 2 的幂（以页为单位）。若参数非法、容量耗尽或无足够大的连续空闲块，
    /// 则返回错误且不改变 `free` 与 `metadata` 的原有状态。
    pub(crate) fn allocate(
        &mut self,
        pages: usize,
        alignment_pages: usize,
    ) -> Result<PageRange, &'static str> {
        // Step 1: 校验请求页数、对齐页数合法性、字节乘法无溢出，以及元数据分配次数未超上限。
        if pages == 0 || !alignment_pages.is_power_of_two() {
            return Err("invalid boot allocation size or alignment");
        }
        pages
            .checked_mul(PAGE_SIZE)
            .ok_or("boot allocation size overflows")?;
        alignment_pages
            .checked_mul(PAGE_SIZE)
            .ok_or("boot allocation alignment overflows")?;
        if self.allocation_count == MAX_METADATA_ALLOCATIONS {
            return Err("boot metadata allocation capacity exceeded");
        }

        // Step 2: 按地址升序扫描 `free` 区间，寻找第一个对齐后仍能完整容纳 `pages` 页的空闲段。
        for index in 0..self.free.count {
            let free = self.free.entries[index];
            let start = free
                .start
                .checked_add(alignment_pages - 1)
                .ok_or("boot allocation alignment overflows")?
                & !(alignment_pages - 1);
            let end = start
                .checked_add(pages)
                .ok_or("boot allocation end overflows")?;
            if free.end < end {
                continue;
            }
            let prefix = free.start < start;
            let suffix = end < free.end;
            if prefix && suffix && self.free.count == RANGE_CAPACITY {
                return Err("boot free range capacity exceeded");
            }

            // Step 3: 先登记至 `metadata`（若超出容量可无损报错返回），再按前后缀剩余情况原地修剪或分裂 `free` 条目。
            let allocated = PageRange { start, end };
            self.metadata.insert(allocated)?;
            match (prefix, suffix) {
                (true, true) => {
                    self.free
                        .entries
                        .copy_within(index + 1..self.free.count, index + 2);
                    self.free.entries[index] = PageRange {
                        start: free.start,
                        end: start,
                    };
                    self.free.entries[index + 1] = PageRange {
                        start: end,
                        end: free.end,
                    };
                    self.free.count += 1;
                }
                (true, false) => self.free.entries[index].end = start,
                (false, true) => self.free.entries[index].start = end,
                (false, false) => {
                    self.free
                        .entries
                        .copy_within(index + 1..self.free.count, index);
                    self.free.count -= 1;
                }
            }
            self.allocation_count += 1;
            return Ok(allocated);
        }
        Err("no contiguous RAM for boot metadata")
    }
}
