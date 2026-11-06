//! PID 槽位分页表与空闲位图分配器。
//!
//! ## 模块职责与架构定位
//! 本模块作为 [`super::TaskGraph`] 的底层存储容器，负责在 `[1, max_pid]` 范围内按需分配槽位页、维护空闲号码位图并提供 `O(1)` 下标索引。
//! - **上游调用者**：仅由 `process` 父模块及 `lifecycle` 子模块在持有 `TaskGraph::slots` 自旋锁时访问（`prepare_growth` 除外，专门在锁外预分配内存）。
//! - **下游依赖**：仅依赖内核堆分配器（`Box`、`Vec`）构造固定大小的槽位页与页目录。
//!
//! ## 核心设计与内存布局
//! 1. **两级分页结构与地址稳定性**：
//!    每个 [`SlotPage`] 固定包含 32 个 [`Slot`]（对应一个 `u32` 位图字）并独立装箱为 `Box<SlotPage>`。当页目录 `Vec<Box<SlotPage>>` 扩容搬迁时，只移动指向各页的指针，页内 `Slot` 及关联的 `Arc<Pid>` 堆地址保持不变，初始也无需按 `max_pid` 一次性预分配大块内存。
//! 2. **空闲位图编码与分步锁外扩容**：
//!    槽位下标 `index` 对应公开号码 `pid = index + 1`（`index / 32` 为页号，`index % 32` 为页内位偏移）。`SlotPage::free` 中某位为 `1` 表示对应槽位空闲（既未被预留也未绑定任何 `TID`/`TGID`/`PGID`/`SID` 角色），为 `0` 表示已被预留或占用，通过 `trailing_zeros()` 总是优先分配最小空闲号码。当现有页无空闲位时，调用方先释放 `TaskGraph::slots` 自旋锁调用 [`Numbers::prepare_growth`] 分配候选页与新目录，再重新加锁通过 [`Numbers::install`] 原子挂载；若并发竞争导致候选未被采用，多余内存随 `Growth` 在锁外析构。
//!
//! ```text
//! Numbers { max_pid, pages: Vec<Box<SlotPage>> }
//!   │
//!   ├─ pages[0] ──► SlotPage (PID 1..=32)
//!   │                 ├─ slots: [Slot; 32]   (index 0 -> PID 1, ..., index 31 -> PID 32)
//!   │                 └─ free:  u32          (bit k == 1: 空闲; bit k == 0: 预留或已占用)
//!   ├─ pages[1] ──► SlotPage (PID 33..=64)
//!   └─ ... (按需在锁外分配并挂载，总有效槽数上限为 min(pages.len() * 32, max_pid))
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - [`Numbers`] 自身不含内部锁，其所有读写方法都必须在持有外层 `TaskGraph::slots` 自旋锁期间调用。
//! - [`Numbers::prepare_growth`] 会执行可能触发内存分配的堆操作，必须在 `TaskGraph::slots` 自旋锁之外调用；[`Growth`] 结构体中未被 `install` 消费的后备页或旧空目录也必须在释放自旋锁后再 Drop。

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ops::{Index, IndexMut};

use super::{ProcessError, Slot};

/// 单个槽位页容纳的槽位数量（固定为 32，与单字 `u32` 空闲位图的位数一一对应）。
const PAGE_SLOTS: usize = u32::BITS as usize;
/// 系统允许配置的最大 PID 上限（正 32 位有符号整数上限 `2^31 - 1`，兼容 Linux `pid_t` ABI）。
pub(crate) const MAX_PID: usize = i32::MAX as usize;

/// 固定包含 32 个身份槽位及对应空闲位图的单页存储块。
struct SlotPage {
    /// 页内连续存放的 32 个身份槽位；页指针经 `Box` 固定后，目录扩容不会移动这些槽位。
    slots: [Slot; PAGE_SLOTS],
    /// 页内槽位空闲状态位图：第 `k` 位为 `1` 表示 `slots[k]` 空闲可分配，为 `0` 表示已预留或仍有活跃角色引用。
    free: u32,
}

/// 按需分页的 PID 槽位表。
///
/// 由外层 `TaskGraph::slots` 自旋锁保护，负责维护 `[0, max_pid)` 下标范围内的槽位页目录与空闲号码分配状态。
pub(super) struct Numbers {
    /// 按页号顺序排列的槽位页指针目录。
    pages: Vec<Box<SlotPage>>,
    /// 允许分配的最大号码数上限（有效槽位下标严格小于 `max_pid`）。
    max_pid: usize,
}

/// 在 `TaskGraph::slots` 自旋锁外预先分配的扩容候选包。
///
/// 若并发线程已抢先完成扩容，未被 `install` 取走的内存会随本对象在锁外自动释放。
pub(super) struct Growth {
    /// 当原页目录容量已满时预先分配的倍增容量新目录容器；无需替换目录时为空 `Vec`。
    pages: Vec<Box<SlotPage>>,
    /// 预先构造并初始化为全空闲（`free = u32::MAX`）的新槽位页；被 `install` 挂载后置为 `None`。
    page: Option<Box<SlotPage>>,
}

impl Numbers {
    /// 创建指定号码上限 `max_pid` 的槽位表，并预分配首个包含 32 个槽位的页。
    ///
    /// 在构造 `TaskGraph` 时于锁外调用。若 `max_pid == 0` 或超过 [`MAX_PID`] 则返回 [`ProcessError::InvalidCapacity`]，堆内存不足则返回 [`ProcessError::NoMemory`]。
    pub(super) fn try_new(max_pid: usize) -> Result<Self, ProcessError> {
        if max_pid == 0 || MAX_PID < max_pid {
            return Err(ProcessError::InvalidCapacity);
        }

        let mut numbers = Self {
            pages: Vec::new(),
            max_pid,
        };
        let mut growth = Self::prepare_growth(0, true)?;
        numbers.install(&mut growth);
        Ok(numbers)
    }

    /// 在关系锁外预分配一个全空闲的新槽位页，并在 `replace_directory` 为真时按 2 的幂次预留新页目录容量。
    ///
    /// 参数 `pages` 为观察到的当前页目录长度。必须在未持有 `TaskGraph::slots` 自旋锁的可分配上下文中调用。
    pub(super) fn prepare_growth(
        pages: usize,
        replace_directory: bool,
    ) -> Result<Growth, ProcessError> {
        let mut directory = Vec::new();
        if replace_directory {
            directory
                .try_reserve_exact((pages + 1).next_power_of_two())
                .map_err(|_| ProcessError::NoMemory)?;
        }

        let page = Box::try_new(SlotPage {
            slots: core::array::from_fn(|_| Slot::vacant()),
            free: u32::MAX,
        })
        .map_err(|_| ProcessError::NoMemory)?;
        Ok(Growth {
            pages: directory,
            page: Some(page),
        })
    }

    /// 在持有关系锁期间复查目录状态并挂载预分配的槽位页，返回是否成功追加新页。
    ///
    /// 若候选页为空、有效槽数已达 `max_pid`、或并发扩容已耗尽预留目录容量，则放弃挂载并返回 `false`。替换页目录时会将旧空 `Vec` 交换到 `growth.pages` 中，留待调用方释放自旋锁后再销毁。
    pub(super) fn install(&mut self, growth: &mut Growth) -> bool {
        // 复查锁外分配期间是否已被其他并发线程抢先扩满至 max_pid 上限。
        if growth.page.is_none() || self.max_pid <= self.len() {
            return false;
        }
        // 若页目录容量已满，验证候选目录容量足以容纳现有页加 1 个新页后再搬移页指针。
        if self.pages.len() == self.pages.capacity() {
            if growth.pages.capacity() < self.pages.len() + 1 {
                return false;
            }
            growth.pages.append(&mut self.pages);
            core::mem::swap(&mut self.pages, &mut growth.pages);
        }

        self.pages
            .push(growth.page.take().expect("missing PID page"));
        true
    }

    /// 返回下一次调用 [`Self::prepare_growth`] 所需的参数快照 `(当前页数, 页目录是否已满)`。
    pub(super) fn growth_requirement(&self) -> (usize, bool) {
        (self.pages.len(), self.pages.len() == self.pages.capacity())
    }

    /// 返回当前已分配页所覆盖且不超过 `max_pid` 的有效槽位总数（仅表示扫描边界，不代表活跃进程数或配额）。
    pub(super) fn len(&self) -> usize {
        self.max_pid.min(self.pages.len() * PAGE_SLOTS)
    }

    /// 判断已分配槽位数是否已到达 `max_pid` 上限（到达上限后无法再继续追加新页）。
    pub(super) fn exhausted(&self) -> bool {
        self.len() == self.max_pid
    }

    /// 扫描各页空闲位图并返回编号最小的空闲槽位下标；若当前已分配页中无可用空位则返回 `None`。
    pub(super) fn first_free(&self) -> Option<usize> {
        self.pages.iter().enumerate().find_map(|(page, slots)| {
            let index = page * PAGE_SLOTS + slots.free.trailing_zeros() as usize;
            (slots.free != 0 && index < self.len()).then_some(index)
        })
    }

    /// 更新指定槽位下标 `index` 在所属页位图中的空闲标志（`free = true` 置 `1` 标记可分配，`false` 清 `0` 标记占用）。
    pub(super) fn set_free(&mut self, index: usize, free: bool) {
        let bit = 1u32 << (index % PAGE_SLOTS);
        if free {
            self.pages[index / PAGE_SLOTS].free |= bit;
        } else {
            self.pages[index / PAGE_SLOTS].free &= !bit;
        }
    }

    /// 按 0 基下标 `index` 获取已分配且处于 `max_pid` 范围内的槽位只读引用；越界或尚未分配的页返回 `None`。
    pub(super) fn get(&self, index: usize) -> Option<&Slot> {
        (index < self.len()).then(|| &self.pages[index / PAGE_SLOTS].slots[index % PAGE_SLOTS])
    }

    /// 按 0 基下标 `index` 获取已分配且处于 `max_pid` 范围内的槽位可变引用；越界或尚未分配的页返回 `None`。
    pub(super) fn get_mut(&mut self, index: usize) -> Option<&mut Slot> {
        if self.len() <= index {
            return None;
        }
        Some(&mut self.pages[index / PAGE_SLOTS].slots[index % PAGE_SLOTS])
    }

    /// 按号码升序遍历当前已分配且在 `max_pid` 范围内的全部槽位。
    pub(super) fn iter(&self) -> impl Iterator<Item = &Slot> {
        self.pages
            .iter()
            .flat_map(|page| page.slots.iter())
            .take(self.len())
    }

    /// 按号码升序可变遍历当前已分配且在 `max_pid` 范围内的全部槽位。
    pub(super) fn iter_mut(&mut self) -> impl Iterator<Item = &mut Slot> {
        let len = self.len();
        self.pages
            .iter_mut()
            .flat_map(|page| page.slots.iter_mut())
            .take(len)
    }
}

impl Index<usize> for Numbers {
    type Output = Slot;

    /// 按已知有效的 0 基槽位下标直接索引槽位；若下标超出当前已分配范围则直接 panic。
    fn index(&self, index: usize) -> &Slot {
        self.get(index).expect("PID slot outside allocated range")
    }
}

impl IndexMut<usize> for Numbers {
    /// 按已知有效的 0 基槽位下标可变索引槽位；若下标超出当前已分配范围则直接 panic。
    fn index_mut(&mut self, index: usize) -> &mut Slot {
        self.get_mut(index)
            .expect("PID slot outside allocated range")
    }
}
