//! 稳定号码槽页与空闲位图；目录扩容只移动页拥有，身份对象不随目录移动。

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::ops::{Index, IndexMut};

use super::{ProcessError, Slot};

const PAGE_SLOTS: usize = u32::BITS as usize;
pub(crate) const MAX_PID: usize = i32::MAX as usize;

struct SlotPage {
    slots: [Slot; PAGE_SLOTS],
    free: u32,
}

pub(super) struct Numbers {
    pages: Vec<Box<SlotPage>>,
    max_pid: usize,
}

pub(super) struct Growth {
    pages: Vec<Box<SlotPage>>,
    page: Option<Box<SlotPage>>,
}

impl Numbers {
    /// 首次只建立一个小槽页，号码上界不决定初始分配量。
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

    /// 锁外预备稳定页，仅当目录已满时按倍增容量准备替换目录。
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

    /// 重验目录代次后移入候选；旧空目录随候选在关系锁外释放。
    pub(super) fn install(&mut self, growth: &mut Growth) -> bool {
        if growth.page.is_none() || self.max_pid <= self.len() {
            return false;
        }
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

    /// 返回候选页所需目录快照；已有空位时不重复分配目录。
    pub(super) fn growth_requirement(&self) -> (usize, bool) {
        (self.pages.len(), self.pages.len() == self.pages.capacity())
    }

    /// 已分配槽数仅用于扫描，不能用作进程额度。
    pub(super) fn len(&self) -> usize {
        self.max_pid.min(self.pages.len() * PAGE_SLOTS)
    }

    /// 目录已覆盖完整号码域时不能再增长。
    pub(super) fn exhausted(&self) -> bool {
        self.len() == self.max_pid
    }

    /// 每页位图选择最小空号，预留和发表都保持该位清零。
    pub(super) fn first_free(&self) -> Option<usize> {
        self.pages.iter().enumerate().find_map(|(page, slots)| {
            let index = page * PAGE_SLOTS + slots.free.trailing_zeros() as usize;
            (slots.free != 0 && index < self.len()).then_some(index)
        })
    }

    /// 号码预留与最后角色解绑在同一关系事务更新空闲位。
    pub(super) fn set_free(&mut self, index: usize, free: bool) {
        let bit = 1u32 << (index % PAGE_SLOTS);
        if free {
            self.pages[index / PAGE_SLOTS].free |= bit;
        } else {
            self.pages[index / PAGE_SLOTS].free &= !bit;
        }
    }

    /// 查询只接受已分配且在号码域内的槽。
    pub(super) fn get(&self, index: usize) -> Option<&Slot> {
        (index < self.len()).then(|| &self.pages[index / PAGE_SLOTS].slots[index % PAGE_SLOTS])
    }

    /// 可变查询不跨越放锁边界。
    pub(super) fn get_mut(&mut self, index: usize) -> Option<&mut Slot> {
        if self.len() <= index {
            return None;
        }
        Some(&mut self.pages[index / PAGE_SLOTS].slots[index % PAGE_SLOTS])
    }

    /// 全图只扫描已经建立的稳定页。
    pub(super) fn iter(&self) -> impl Iterator<Item = &Slot> {
        self.pages
            .iter()
            .flat_map(|page| page.slots.iter())
            .take(self.len())
    }

    /// 变更扫描不触及尚未分配的号码。
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
    /// 图内已验证的身份可直接按原号码访问。
    fn index(&self, index: usize) -> &Slot {
        self.get(index).expect("PID slot outside allocated range")
    }
}

impl IndexMut<usize> for Numbers {
    /// 可变借用限定于本次关系锁事务。
    fn index_mut(&mut self, index: usize) -> &mut Slot {
        self.get_mut(index)
            .expect("PID slot outside allocated range")
    }
}
