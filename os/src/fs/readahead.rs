//! 打开文件描述符级顺序预读（Readahead）状态机与窗口提交调度。
//!
//! # 模块职责与架构定位
//! 本模块为磁盘常规文件（`ext4`）的顺序与局部性读取提供异步预读优化：
//! - **上游调用者**：`InodeRef::read_into` 在每次读取的首页或跨页边界处调用 `prepare` 触发预读窗口，并在向调用方成功拷贝字节后调用 `note_read` 推进上次读取结束偏移；`File`（打开文件描述，OFD）在自身结构体中内嵌 `SpinLock<ReadaheadState>`。
//! - **下游依赖**：页缓存（`AddressSpace::prepare_file` / `publish_file`）、文件块映射查询（`writeback::prepare_file_targets`）以及 `Ext4Volume` 的批量连续读与单页填充接口。
//! - **职责边界**：`ReadaheadState` 仅保存属于单个打开文件实例的窗口预测提示，不持有物理页或块 I/O 状态；实际的文件页缓存索引、单飞（Singleflight）加载权以及异步块读取完成回调均归属于唯一的 `Inode` 地址空间（`mapping`）。
//!
//! # 核心设计与预读窗口状态机
//! ## 1. 预读窗口几何布局
//! 一次预读窗口 `[start, start + size)` 被划分为**同步消费前缀**与**异步预取后缀**两部分，交界处的页称为**触发页（Lookahead / Trigger Page）**：
//! ```text
//! 页索引方向 ----------------------------------------------------------->
//! +----------------------------+----------------------------------------+
//! |  同步消费区 (size - async)  |        异步预取区 (async_size)          |
//! +----------------------------+----------------------------------------+
//! ^                            ^                                        ^
//! start                   expected (触发页)                      start + size
//!                         = start + size - async_size
//! ```
//! - **冷启动 / 顺序未命中**：当读取首字节位于页 `0`、单次请求跨度超过 `ra_pages`、或目标页紧邻上次实际读完的页（距离 `<= 1` 页）时，按 `initial_size` 建立新窗口（`ra_pages = 32` 时单页请求初窗为 4 页），并将 `async_size` 设为剩余预取页数（或窗口的一半），从而提前在触发页埋下扩展点。
//! - **流水线推进（页缓存命中触发页）**：当顺序流消费到已缓存的 `expected` 页时，`plan` 立即将新窗口推进到旧窗口末尾 `start + size`，按 `next_size` 倍增窗口大小（`4 -> 8 -> 16 -> 32` 页），并将整段新窗口全部标为异步预取（`async_size = size`），实现“边读当前窗口、后台并发预取下一窗口”的流水线。
//! - **跨文件描述符 / 交错顺序流恢复**：若目标页未命中且不紧邻本 OFD 的 `prev_pos`，则向前回溯检查 `inode.mapping()` 中连续驻留的已缓存页数 `history`（最多 32 页）。若存在比请求更大的连续历史缓存前缀，则据此恢复交错读取窗口；否则视为纯随机小读，仅按需读取请求页而不污染现有的顺序预读窗口状态。
//!
//! ## 2. 预分配与提交协议
//! 1. **锁外内存预准备**：在获取 `inode.lock_invalidate()` 之前，先为窗口内各页调用 `prepare_file`（可能触发内存回收与物理页分配），避免持睡眠锁期间重入回收路径。
//! 2. **失效锁内发布与连续块合并**：持有共享的 `lock_invalidate` 防止并发截断（`truncate`）更改文件大小或驱逐页；逐页 `publish_file` 抢占单飞读入权（`FolioRead`），将 EOF 以内且文件系统块在磁盘上物理连续的整页收集进 `ranges` 一次批量下发，遇到空洞、未初始化 Extent、碎片或尾页时则先刷出累积批次再走 `fill_page` 精确填充。
//! 3. **容错语义**：只有调用方当前急需的目标页（`demand`）准备或提交失败才会向 `read` 返回错误；后续异步预取页的任何分配或 I/O 失败均被静默吸收，由各页的 `FolioRead` 独立结清状态。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **`SpinLock<ReadaheadState>`（OFD 预读提示自旋锁）**：仅在 `plan` 计算窗口与 `note_read` 更新偏移时短暂持有；必须在调用 `inode.lock_invalidate()`、页缓存操作或任何可能睡眠的分配/块 I/O **之前释放**。
//! 2. **`InodeRef::lock_invalidate`（Inode 截断/失效共享锁）**：在 `submit_window` 发布缓存页与提交块读取期间持有，提交返回后即可释放；底层异步 DMA 期间的物理页与 Inode 生命周期由块层持有的 `FolioRead` 凭证保活，并发截断在页内容锁上等待在途读取彻底执行完毕并退出。

use alloc::vec::Vec;
use core::ops::Range;

use crate::block::SECTOR_BYTES;
use crate::cache::PreparedFile;
use crate::fs::ext4::finish_error;
use crate::fs::{writeback, FsError, InodeRef};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::{FolioRead, PAGE_SIZE};
use crate::sync::SpinLock;

/// 单个打开文件实例默认允许的最大预读窗口页数（32 页，4 KiB 页大小下为 128 KiB）。
const RA_PAGES: usize = 32;

/// 绑定在打开文件描述（OFD）上的顺序预读窗口状态。
///
/// 由外层 `SpinLock<ReadaheadState>` 保护，仅记录窗口预测参数与上次读取位置，不阻塞页缓存读写或块设备 I/O。
pub struct ReadaheadState {
    /// 当前活跃预读窗口的起始文件页索引（单位：页，每页 `PAGE_SIZE` 字节）。
    start: u64,
    /// 当前活跃预读窗口的总长度（单位：页）；为 `0` 表示尚未建立顺序预读窗口。
    size: usize,
    /// 当前窗口尾部属于提前异步预取的页数（单位：页），用于计算触发下一轮扩展的页索引 `start + size - async_size`。
    async_size: usize,
    /// 该打开文件实例上一次成功向调用方交付数据的末尾文件字节偏移（单位：字节）；初始为 `u64::MAX` 以免偏移 `0` 以外的首次随机读被误判为顺序流。
    prev_pos: u64,
    /// 该打开文件实例允许扩展到的最大预读窗口上限（单位：页）。
    ra_pages: usize,
}

/// 供内核测试（`ktest`）观察的预读状态只读快照。
///
/// 记录 `ReadaheadState` 在某一时刻的窗口参数与上次交付偏移；注意窗口大小仅代表预读计划尝试的范围，不等于底层块设备实际已接受或已完成的页数。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReadaheadSnapshot {
    /// 当前预读窗口的起始文件页索引（单位：页）。
    pub start: u64,
    /// 当前预读窗口的总长度（单位：页）。
    pub size: usize,
    /// 当前预读窗口尾部的异步预取页数（单位：页）。
    pub async_size: usize,
    /// 上一次成功交付给调用方的末尾文件字节偏移（单位：字节）。
    pub prev_pos: u64,
    /// 最大预读窗口页数上限（单位：页）。
    pub ra_pages: usize,
}

impl ReadaheadState {
    /// 创建无读取历史的初始预读状态，将 `prev_pos` 置为 `u64::MAX` 以区分文件首字节读取与首次非零偏移随机读取。
    pub const fn new() -> Self {
        Self {
            start: 0,
            size: 0,
            async_size: 0,
            prev_pos: u64::MAX,
            ra_pages: RA_PAGES,
        }
    }

    /// 根据自旋锁外采样的目标页缓存命中情况与历史驻留页数，更新状态机并规划需要提交的预读页区间。
    ///
    /// 在持有 `SpinLock<ReadaheadState>` 时调用：
    /// - `index`：本次读取起始偏移所在的文件页索引；
    /// - `requested`：本次读取请求折算并截断后的页数；
    /// - `cached`：`index` 页当前是否已在页缓存中命中；
    /// - `history`：`cached == false` 时，`index` 之前连续驻留在页缓存中的页数。
    ///
    /// 返回 `Some(start..end)` 表示需要发起准备与预取的页索引范围；返回 `None` 表示当前页已在缓存且未命中扩展触发页，无需下发预读。
    fn plan(
        &mut self,
        index: u64,
        requested: usize,
        cached: bool,
        history: usize,
    ) -> Option<Range<u64>> {
        // Step 1: 计算当前窗口的异步扩展触发页索引 expected。
        let expected = self.start + (self.size - self.async_size) as u64;
        if cached {
            // Step 2a: 目标页已在页缓存命中。仅当它恰好是当前窗口埋下的触发页时，才将窗口向后倍增推进。
            if self.async_size == 0 || index != expected {
                return None;
            }
            self.start += self.size as u64;
            self.size = next_size(self.size, self.ra_pages);
            self.async_size = self.size;
        } else {
            // Step 2b: 目标页未命中页缓存。判断其是否紧接在本 OFD 上次读取结束页之后（同页或下一相邻页）。
            let sequential = index
                .checked_sub(self.prev_pos / PAGE_SIZE as u64)
                .is_some_and(|distance| distance <= 1);
            if index == 0 || self.ra_pages < requested || sequential {
                // 从文件开头读取、大块读取或明确的顺序流：建立全新顺序预读窗口并设置触发页。
                self.start = index;
                self.size = initial_size(requested, self.ra_pages);
                self.async_size = if requested < self.size {
                    self.size - requested
                } else {
                    self.size / 2
                };
            } else if history <= requested {
                // 缺少连续缓存历史且非顺序位置：按纯随机小读处理，仅读取所需页，保留原顺序窗口不被覆盖。
                return Some(index..index + requested.min(self.ra_pages) as u64);
            } else {
                // 虽与上次 prev_pos 不连续，但页缓存中存在显著的前序连续驻留页（如多线程交错顺序读），据此恢复窗口。
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

    /// 导出当前预读窗口状态的只读副本供内核测试断言检查。
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

/// 计算新建顺序预读窗口的初始页数。
///
/// 先将请求页数限制在 `maximum` 以内并向上取整到 2 的幂，再根据占最大窗口的比例放大 4 倍或 2 倍（当 `maximum == 32` 时，单页请求的初始窗口为 4 页）。
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

/// 计算命中触发页后下一轮预读窗口的页数。
///
/// 小窗口扩容 4 倍，中等窗口扩容 2 倍，直至达到上限 `maximum`（当 `maximum == 32` 时，4 页窗口推进为 8 页，随后依次为 16 页、32 页）。
fn next_size(current: usize, maximum: usize) -> usize {
    if current < maximum / 16 {
        current * 4
    } else if current <= maximum / 2 {
        current * 2
    } else {
        maximum
    }
}

/// 从目标页 `index` 向前（低页号方向）探测当前 `Inode` 地址空间中连续驻留的已缓存页数量（最多探测 `RA_PAGES` 页）。
///
/// 仅查询页缓存索引槽位是否存在，不读取页内容，可在普通任务无锁上下文中调用。
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

/// 在常规文件首次进入某页读取前规划并异步提交有界预读窗口。
///
/// 调用上下文：普通可睡眠任务上下文，调用方未持有 `state` 自旋锁与 `inode` 的 `invalidate` 锁。
/// 锁与生命周期契约：`state` 自旋锁在采样后短暂获取并立即释放，绝不跨 `inode.lock_invalidate()` 或块 I/O 持有；
/// `lock_invalidate` 仅在本函数提交块请求期间持有，底层异步读取期间由 `FolioRead` 凭证保活物理页与 `Inode`，并发截断通过等待页读取状态彻底结束来保证内存安全。
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

    // Step 1: 将字节偏移与请求长度折算为页索引与有界请求页数，并在自旋锁外探测页缓存状态。
    let index = position / PAGE_SIZE as u64;
    let in_page = (position % PAGE_SIZE as u64) as usize;
    let requested = (requested.min(RA_PAGES * PAGE_SIZE) + in_page).div_ceil(PAGE_SIZE);
    let cached = inode.mapping().cached_folio(index).is_some();
    let history = if cached {
        0
    } else {
        cached_history(inode, index)
    };

    // Step 2: 在 OFD 预读自旋锁临界区内更新状态机并取得计划窗口，随后立即释放自旋锁。
    let window = state.lock().plan(index, requested, cached, history);
    let Some(mut window) = window else {
        return Ok(());
    };

    // Step 3: 按当前文件大小裁剪掉超出 EOF 的页，若窗口非空则下发异步预读请求。
    let size = inode.metadata_snapshot()?.size;
    window.end = window.end.min(size.div_ceil(PAGE_SIZE as u64));
    if window.end <= window.start {
        return Ok(());
    }
    submit_window(inode, window, index, attempt)
}

/// 在实际向用户缓冲区成功拷贝字节后，更新该打开文件实例的上次读取末尾偏移 `prev_pos`。
///
/// 即使后续块发生用户态缺页错误，只要本次已成功拷贝了部分字节（Partial Progress，即非零前缀），也必须推进 `prev_pos`；零字节拷贝时调用方不应调用本函数。
pub(crate) fn note_read(state: &SpinLock<ReadaheadState>, actual_end: u64) {
    state.lock().prev_pos = actual_end;
}

/// 为计划窗口 `window` 分配缓存页候选并向底层 `Ext4Volume` 批量提交块读取请求。
///
/// 在普通可睡眠任务中调用。`demand` 为调用方当前同步等待的目标页索引：仅当 `demand` 页本身分配、映射查询或提交失败时才返回 `Err`；
/// 窗口内其余预取页若中途遇到资源不足或 I/O 错误，则截断后续预取并正常提交已就绪的前缀页。
fn submit_window(
    inode: &InodeRef,
    mut window: Range<u64>,
    demand: u64,
    attempt: &mut ReclaimAttempt,
) -> Result<(), FsError> {
    // Step 1: 取得底层磁盘卷并预分配连续块批次缓冲；若仅异步预取窗口分配失败则直接放弃预读而不报错。
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

    // Step 2: 在获取 lock_invalidate 之前，为窗口内每一页预先分配物理页与页缓存树节点（可能触发内存回收）。
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

    // Step 3: 持有 Inode 失效共享锁，二次校验 EOF 后将候选页发布进页缓存并合并物理连续块下发异步读。
    let result = (|| {
        let _invalidate = inode.lock_invalidate()?;
        window.end = window
            .end
            .min(inode.metadata_snapshot()?.size.div_ceil(PAGE_SIZE as u64));
        for (slot, index) in window.clone().enumerate() {
            // 将预准备的候选页插入 mapping：若已被其他并发读者填充完毕则跳过，若赢得单飞读入权则拿到 FolioRead。
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
            // 查询该文件页内各个文件系统块对应的磁盘字节偏移（空洞或未初始化 Extent 对应 None）。
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
            // 若整页位于 EOF 内且所有块在磁盘上物理连续，则暂存到批次队列中合并为大块 BIO 提交。
            if let Some(byte) = contiguous_page(index, size, volume.geometry().block_size, &targets)
            {
                ranges.push((byte, read));
                continue;
            }
            // 遇到稀疏空洞、不连续块或文件尾半页：先刷出此前积累的连续批次，再对该页执行逐块填充与零清。
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
        // Step 4: 提交队列中剩余的连续页批次；后续异步预取页的提交失败不应掩盖已成功下发的目标页，也不应导致目标页重复同步读取。
        let _ = volume.claim().read_folio_batch(&mut ranges);
        demand_error.map_or(Ok(()), Err)
    })();

    // Step 5: 在释放 lock_invalidate 之后再统一析构未消费的候选页（例如输掉单飞竞争或因 EOF 裁剪剩余的页）。
    drop(candidates);
    result
}

/// 判断文件页 `index` 是否完全处于 `file_size`（EOF）之内且页内所有文件系统块在磁盘上物理连续。
///
/// 若满足条件则返回首页块在卷上的起始字节偏移 `Some(first)`，可直接合并进整页批量读；
/// 若包含文件空洞、未写入（unwritten）Extent、跨不连续块碎片或跨越 EOF 的尾页，则返回 `None`，交由 `fill_page` 按块精确读取并清零尾端。
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
