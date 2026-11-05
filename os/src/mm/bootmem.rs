//! 启动阶段按真实 RAM 管理整页区间，元数据只从未保留页取得。

use crate::boot::{Region, MAX_MEMORY_REGIONS, MAX_RESERVED_REGIONS};

pub(crate) const PAGE_SHIFT: usize = 12;
pub(crate) const PAGE_SIZE: usize = 1 << PAGE_SHIFT;

// root 索引一次，每段 RAM 至多一次描述符 backing。
const MAX_METADATA_ALLOCATIONS: usize = 1 + MAX_MEMORY_REGIONS;
pub(crate) const RANGE_CAPACITY: usize =
    MAX_MEMORY_REGIONS + MAX_RESERVED_REGIONS + MAX_METADATA_ALLOCATIONS;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct PageRange {
    pub start: usize,
    pub end: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BootStats {
    pub present_pages: usize,
    pub reserved_pages: usize,
    pub metadata_pages: usize,
    pub free_pages: usize,
}

struct Ranges {
    entries: [PageRange; RANGE_CAPACITY],
    count: usize,
}

impl Ranges {
    /// 创建启动期小型固定容量表，不借用堆存储。
    const fn new() -> Self {
        Self {
            entries: [PageRange { start: 0, end: 0 }; RANGE_CAPACITY],
            count: 0,
        }
    }

    /// 仅返回按起点排序、互不相交且不相邻的有效区间。
    fn as_slice(&self) -> &[PageRange] {
        &self.entries[..self.count]
    }

    /// 将区间并入有序集合，容量错误不改变原集合。
    fn insert(&mut self, mut range: PageRange) -> Result<(), &'static str> {
        if range.end <= range.start {
            return Ok(());
        }

        let first = self
            .as_slice()
            .partition_point(|entry| entry.end < range.start);
        let mut last = first;
        while last < self.count && self.entries[last].start <= range.end {
            range.start = range.start.min(self.entries[last].start);
            range.end = range.end.max(self.entries[last].end);
            last += 1;
        }

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

    /// 在已有空表内合并物理字节区间再取整，避免返回固定数组副本和丢失相邻片段的完整页。
    fn initialize_regions(&mut self, regions: &[Region], inward: bool) -> Result<(), &'static str> {
        for region in regions {
            if region.end <= region.start {
                return Err("invalid boot physical region");
            }
            self.insert(PageRange {
                start: region.start,
                end: region.end,
            })?;
        }

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

    /// 累计互不重叠区间的页数。
    fn pages(&self) -> usize {
        self.as_slice()
            .iter()
            .map(|range| range.end - range.start)
            .sum()
    }
}

/// 地址向上取整时拒绝超出物理地址整数范围。
fn round_up(address: usize) -> Result<usize, &'static str> {
    address
        .checked_add(PAGE_SIZE - 1)
        .map(|value| value & !(PAGE_SIZE - 1))
        .ok_or("boot page alignment overflows")
}

pub(crate) struct BootMemory {
    ram: Ranges,
    reserved: Ranges,
    metadata: Ranges,
    free: Ranges,
    allocation_count: usize,
}

impl BootMemory {
    /// 为调用方存储建立合法空账本，常量初始化不经过大对象返回帧。
    pub(crate) const fn empty() -> Self {
        Self {
            ram: Ranges::new(),
            reserved: Ranges::new(),
            metadata: Ranges::new(),
            free: Ranges::new(),
            allocation_count: 0,
        }
    }

    /// 在新空账本内分类真实整页RAM，只扣除保留区与RAM的交集；失败后调用方不得继续启动。
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

        self.ram.initialize_regions(ram, true)?;
        let mut reservations = const { Ranges::new() };
        reservations.initialize_regions(reserved, false)?;

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

    /// 借用真实整页 RAM 并集，空洞不属于该集合。
    pub(crate) fn ram(&self) -> &[PageRange] {
        self.ram.as_slice()
    }

    /// 借用尚未移交给元数据的可用页并集。
    pub(crate) fn free(&self) -> &[PageRange] {
        self.free.as_slice()
    }

    /// 借用原始保留资源与整页 RAM 的交集。
    pub(crate) fn reserved(&self) -> &[PageRange] {
        self.reserved.as_slice()
    }

    /// 借用已实际分配给启动元数据的页并集。
    pub(crate) fn metadata(&self) -> &[PageRange] {
        self.metadata.as_slice()
    }

    /// 从真实区间计算互斥页角色的账务。
    pub(crate) fn stats(&self) -> BootStats {
        BootStats {
            present_pages: self.ram.pages(),
            reserved_pages: self.reserved.pages(),
            metadata_pages: self.metadata.pages(),
            free_pages: self.free.pages(),
        }
    }

    /// 从单个可用区间取得对齐连续页并登记元数据；错误不改变页拥有状态。
    pub(crate) fn allocate(
        &mut self,
        pages: usize,
        alignment_pages: usize,
    ) -> Result<PageRange, &'static str> {
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
