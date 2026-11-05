//! 打开实例只保存有界顺序提示，文件页和异步读入责任仍属于唯一 inode mapping。

use alloc::vec::Vec;
use core::ops::Range;

use crate::block::SECTOR_BYTES;
use crate::cache::PreparedFile;
use crate::fs::ext4::finish_error;
use crate::fs::{writeback, FsError, InodeRef};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::{FolioRead, PAGE_SIZE};
use crate::sync::SpinLock;

const RA_PAGES: usize = 32;

/// 页窗口与真实交付位置属于同一 OFD；短锁只保护这些提示，不串行页内容或 I/O。
pub struct ReadaheadState {
    start: u64,
    size: usize,
    async_size: usize,
    prev_pos: u64,
    ra_pages: usize,
}

/// 只读诊断保存窗口尝试及实际交付位置，不把配置窗口当作已接受或已完成的设备页数。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReadaheadSnapshot {
    pub start: u64,
    pub size: usize,
    pub async_size: usize,
    pub prev_pos: u64,
    pub ra_pages: usize,
}

impl ReadaheadState {
    /// 新打开实例没有读历史；首次随机访问不会被位置零误判为顺序流。
    pub const fn new() -> Self {
        Self {
            start: 0,
            size: 0,
            async_size: 0,
            prev_pos: u64::MAX,
            ra_pages: RA_PAGES,
        }
    }

    /// 只按锁外取得的缓存事实计算下一窗口，随机小请求不改既有顺序窗口。
    fn plan(
        &mut self,
        index: u64,
        requested: usize,
        cached: bool,
        history: usize,
    ) -> Option<Range<u64>> {
        let expected = self.start + (self.size - self.async_size) as u64;
        if cached {
            if self.async_size == 0 || index != expected {
                return None;
            }
            self.start += self.size as u64;
            self.size = next_size(self.size, self.ra_pages);
            self.async_size = self.size;
        } else {
            let sequential = index
                .checked_sub(self.prev_pos / PAGE_SIZE as u64)
                .is_some_and(|distance| distance <= 1);
            if index == 0 || self.ra_pages < requested || sequential {
                self.start = index;
                self.size = initial_size(requested, self.ra_pages);
                self.async_size = if requested < self.size {
                    self.size - requested
                } else {
                    self.size / 2
                };
            } else if history <= requested {
                return Some(index..index + requested.min(self.ra_pages) as u64);
            } else {
                let history = if history as u64 == index {
                    history * 2
                } else {
                    history
                };
                self.start = index;
                self.size = (history + requested).min(self.ra_pages);
                self.async_size = 1;
            }
        }
        Some(self.start..self.start + self.size as u64)
    }

    /// 复制提示供具体消费者观察，不导出修改原 OFD 状态的入口。
    #[cfg(feature = "ktest")]
    pub(crate) fn snapshot(&self) -> ReadaheadSnapshot {
        ReadaheadSnapshot {
            start: self.start,
            size: self.size,
            async_size: self.async_size,
            prev_pos: self.prev_pos,
            ra_pages: self.ra_pages,
        }
    }
}

/// 请求先向上取二次幂，再按最大窗口的两个分界增长；最大三十二页时单页初窗为四页。
fn initial_size(requested: usize, maximum: usize) -> usize {
    let size = requested.min(maximum).next_power_of_two();
    if size <= maximum / 32 {
        size * 4
    } else if size <= maximum / 4 {
        size * 2
    } else {
        maximum
    }
}

/// 触发页命中后沿原窗口向前推进；四页窗口在三十二页上限下增长为八页。
fn next_size(current: usize, maximum: usize) -> usize {
    if current < maximum / 16 {
        current * 4
    } else if current <= maximum / 2 {
        current * 2
    } else {
        maximum
    }
}

/// 在本 mapping 向前只查询最多三十二个已有页，不读取内容或借用其他打开实例的历史。
fn cached_history(inode: &InodeRef, index: u64) -> usize {
    let mut history = 0;
    while history < RA_PAGES && (history as u64) < index {
        if inode
            .mapping()
            .cached_folio(index - history as u64 - 1)
            .is_none()
        {
            break;
        }
        history += 1;
    }
    history
}

/// 每页首次交付前准备有限窗口；状态锁在取得 inode invalidate 及任何内容或 I/O 保护前释放。
/// 共享 invalidate 只由本 Task 持到提交结束，异步凭证保原页和 inode，截断沿内容锁等真实全尾。
pub(crate) fn prepare(
    inode: &InodeRef,
    state: &SpinLock<ReadaheadState>,
    position: u64,
    requested: usize,
    attempt: &mut ReclaimAttempt,
) -> Result<(), FsError> {
    if requested == 0 {
        return Ok(());
    }

    let index = position / PAGE_SIZE as u64;
    let in_page = (position % PAGE_SIZE as u64) as usize;
    let requested = (requested.min(RA_PAGES * PAGE_SIZE) + in_page).div_ceil(PAGE_SIZE);
    let cached = inode.mapping().cached_folio(index).is_some();
    let history = if cached {
        0
    } else {
        cached_history(inode, index)
    };
    let window = state.lock().plan(index, requested, cached, history);
    let Some(mut window) = window else {
        return Ok(());
    };

    let size = inode.metadata_snapshot()?.size;
    window.end = window.end.min(size.div_ceil(PAGE_SIZE as u64));
    if window.end <= window.start {
        return Ok(());
    }
    submit_window(inode, window, index, attempt)
}

/// 真实交付后记录末位置；用户故障的正前缀同样推进，零交付调用者不进入本接口。
pub(crate) fn note_read(state: &SpinLock<ReadaheadState>, actual_end: u64) {
    state.lock().prev_pos = actual_end;
}

/// 页、Node 和 IO 片段先在 invalidate 外 fallible 准备；拒尾仍提交真实前缀，输家锁外结束。
fn submit_window(
    inode: &InodeRef,
    mut window: Range<u64>,
    demand: u64,
    attempt: &mut ReclaimAttempt,
) -> Result<(), FsError> {
    let volume = inode.disk_volume()?;
    let mut ranges: Vec<(u64, FolioRead)> = Vec::new();
    if ranges
        .try_reserve_exact((window.end - window.start) as usize)
        .is_err()
    {
        return if window.start == demand {
            Err(FsError::NoMemory)
        } else {
            Ok(())
        };
    }

    let mut candidates: [Option<PreparedFile>; RA_PAGES] = [const { None }; RA_PAGES];
    let mut demand_error = None;
    for (slot, index) in window.clone().enumerate() {
        match inode.mapping().prepare_file(inode, index, attempt) {
            Ok(prepared) => candidates[slot] = Some(prepared),
            Err(error) => {
                if index == demand {
                    demand_error = Some(error);
                }
                window.end = index;
                break;
            }
        }
    }

    let result = (|| {
        let _invalidate = inode.lock_invalidate()?;
        window.end = window
            .end
            .min(inode.metadata_snapshot()?.size.div_ceil(PAGE_SIZE as u64));
        for (slot, index) in window.clone().enumerate() {
            let read = match inode
                .mapping()
                .publish_file(inode, index, &mut candidates[slot])
            {
                Ok(Some((_, None))) => continue,
                Ok(Some((_, Some(read)))) => read,
                Ok(None) => break,
                Err(error) => {
                    if index == demand {
                        demand_error = Some(error.into());
                    }
                    break;
                }
            };
            let (size, targets) =
                match writeback::prepare_file_targets(inode, index, &(0..u64::MAX), None) {
                    Ok(prepared) => prepared,
                    Err(error) => {
                        let error = finish_error(read, error);
                        if index == demand {
                            demand_error = Some(error);
                        }
                        break;
                    }
                };
            if let Some(byte) = contiguous_page(index, size, volume.geometry().block_size, &targets)
            {
                ranges.push((byte, read));
                continue;
            }
            if let Err(error) = volume.claim().read_folio_batch(&mut ranges) {
                finish_error(read, error.into());
                break;
            }
            if let Err(error) = volume.fill_page(index, size, &targets, read) {
                if index == demand {
                    demand_error = Some(error);
                }
                break;
            }
        }
        // 后继预取失败不能覆盖需求页实际成功，也不能同步重读已经失败的原页。
        let _ = volume.claim().read_folio_batch(&mut ranges);
        demand_error.map_or(Ok(()), Err)
    })();

    drop(candidates);
    result
}

/// 只有 EOF 内整页且每个 FS 块都物理连续时才合并，孔、未写块、碎片和尾页保原填充语义。
fn contiguous_page(
    index: u64,
    file_size: u64,
    block_size: usize,
    targets: &[Option<u64>; PAGE_SIZE / SECTOR_BYTES as usize],
) -> Option<u64> {
    let end = index.checked_add(1)?.checked_mul(PAGE_SIZE as u64)?;
    if file_size < end {
        return None;
    }
    let first = targets[0]?;
    for offset in (block_size..PAGE_SIZE).step_by(block_size) {
        if targets[offset / block_size] != Some(first.checked_add(offset as u64)?) {
            return None;
        }
    }
    Some(first)
}
