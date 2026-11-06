//! 文件数据页与元数据缓存页的统一回写（Writeback）引擎模块。
//!
//! # 模块职责与架构定位
//! 本模块负责将已修改的文件页缓存（`FileMapping`）和卷元数据缓存（`MetadataCache`）中的脏物理页（`FolioRef`）
//! 按批次转化为块设备写请求并落盘：
//! - **上游调用者**：显式文件同步（`fsync` / `fdatasync` / `sync_filesystem`）、Inode 回收淘汰（`inode::evict`）以及后台定时/超限回写线程（`fs::background`）。
//! - **下游依赖**：页缓存标记与内容锁（`crate::cache`、`crate::mm::folio`）、ext4 逻辑块到物理块映射（`Ext4Volume::map_block`）以及块设备批量写与刷盘接口（`FsClaim::writeback_batch`、`FsClaim::flush`）。
//!
//! # 核心设计与多步窗口回写流水线
//! 为避免并发写入者持续置脏导致单次 `fsync` 无限循环（写饥饿），并防止在持有页面内容锁或 Extent 映射锁期间发起数据块 I/O 造成死锁，回写采用“单次快照标记 + 定长滑动窗口（`MAX_BATCH = 8` 页）”流水线：
//!
//! ```text
//!   步骤 0: 入口一次性标记 ToWrite (防饥饿快照)
//!   mapping / metadata_cache 扫描目标范围，将当前 dirty|writeback 页打上 FolioMark::ToWrite
//!          │
//!          ▼  按 MAX_BATCH (8 页) 滑动窗口循环摘取 ToWrite 页
//!   ┌─────────────────────────────────────────────────────────────────────────┐
//!   │ 窗口步骤 A: 等待旧在途回写 & 解析目标物理块偏移 (不持有 FolioContent 锁) │
//!   │  1. folio.wait_writeback() 等待上一轮在途 DMA 结束                      │
//!   │  2. 文件页在 inode.with_map_read 下查询 Extent 树得到 FilePagePlan      │
//!   ├─────────────────────────────────────────────────────────────────────────┤
//!   │ 窗口步骤 B: 逐页加内容锁、清写权限并收集回写凭证 FolioWriteback          │
//!   │  1. try_lock_content 失败时先提交已收集前缀，再阻塞 lock_content 防死锁  │
//!   │  2. 文件页执行 mapping.mkclean 撤销用户态共享可写 PTE 并跨核同步 TLB     │
//!   │  3. content.begin_writeback 将脏扇区原子转入 writeback 位图并生成凭证    │
//!   ├─────────────────────────────────────────────────────────────────────────┤
//!   │ 窗口步骤 C: 块层合并提交与完成等待                                       │
//!   │  claim.writeback_batch 将相邻物理偏移合并进父 Bio，提交设备并等待完成    │
//!   └─────────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # 锁获取顺序与并发契约
//! 1. **禁止嵌套内存回收作用域**：所有回写入口均进入 `ReclaimScope::noio()`，防止回写路径中的内部分配再次触发页面回收死锁。
//! 2. **映射/缓存级回写睡眠互斥锁**（`FileMapping::lock_writeback` 或 `MetadataCache::lock_writeback`）：串行化同一 Inode 或同一卷元数据缓存上的 `ToWrite` 标记扫描与后台游标更新；不同 Inode 的数据回写互不阻塞。
//! 3. **Inode 块映射读锁**（`Inode::with_map_read`）：仅在窗口步骤 A 查询逻辑块对应的物理磁盘位置时短暂持有，必须在获取页面内容锁或提交数据 `Bio` 前释放。
//! 4. **页面内容睡眠锁**（`FolioRef::lock_content`）$\rightarrow$ **页表锁与跨核 TLB 同步**（`mkclean`）$\rightarrow$ **页描述符自旋锁**（`begin_writeback` 内部）。若当前手头已持有尚未提交的 `FolioWriteback` 凭证而下一张页的 `try_lock_content` 失败，必须先调用 `submit_ranges` 将已准备好的前缀发往块层，再阻塞等待内容锁，以免阻塞正在等待前序 I/O 的持锁者。
//! 5. **Inode 失效锁**（`refresh_dirty_obligation` 内部获取）：必须在释放 `FileMapping::lock_writeback` 之后获取，用于重新核对并清理 Inode 的脏链表挂载状态。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::ops::Range;

use crate::block::{
    Bio, BioAllocator, BioCompletion, BioLane, BioReserve, BlockError, FsClaim, MetadataIoContext,
    MAX_BATCH, SECTOR_BYTES,
};
use crate::cache::FolioMark;
use crate::fs::background::WritebackDomain;
use crate::fs::{BlockMapping, Ext4Volume, FsError, InodeRef};
use crate::mm::{FolioRef, FolioWriteback, PAGE_SIZE};
use crate::task::ReclaimScope;

/// 单张 4 KiB 物理页包含的 512 字节标准扇区数量（8 个扇区）。
const SECTORS_PER_PAGE: usize = PAGE_SIZE / SECTOR_BYTES as usize;
/// 单个 `MAX_BATCH`（8 页）回写窗口最多可产生的独立扇区回写区间容量上限（$8 \times 8 = 64$）。
const WINDOW_RANGES: usize = MAX_BATCH * SECTORS_PER_PAGE;

/// 单张文件页在回写窗口步骤 A 生成的物理目标映射计划。
///
/// 仅保存物理页强引用与各文件系统块在卷内的目标字节偏移，不拷贝页面数据内容。
struct FilePagePlan {
    /// 该页在所属文件内的逻辑页索引（以 `PAGE_SIZE` 为单位）。
    index: u64,
    /// 绑定了所属文件 `InodeRef` 所有者的页缓存物理页强引用。
    folio: FolioRef,
    /// 页内各块槽位对应的卷内相对字节偏移（`None` 表示文件空洞、超出文件末尾或不在同步范围内）；
    /// 有效元素个数为 `PAGE_SIZE / block_size`（最多 `SECTORS_PER_PAGE` 个）。
    targets: [Option<u64>; SECTORS_PER_PAGE],
}

/// 单次回写任务复用的定长工作缓冲区与块层预留通道上下文。
///
/// 前台显式同步可在锁外按需在堆上构造独立实例，使不同 Inode 的 `fsync` 并发执行；
/// 后台回写或内存回收路径则从 `WritebackDomain` 借用预分配实例并绑定 `BioReserve` 预留通道（`BioLane`），实现零堆分配回写。
/// 实例归还或析构前会清空全部临时引用与 `Bio` 句柄，确保空闲缓冲区不会延长任何物理页或 `Inode` 的生命周期。
pub(crate) struct WbStorage {
    /// 当前批次从缓存索引中摘取的 `ToWrite` 页面暂存槽（键为文件页号或元数据字节偏移）。
    selected: [Option<(u64, FolioRef)>; MAX_BATCH],
    /// 当前批次已完成逻辑块到物理块映射查询的文件页回写计划表。
    plans: [Option<FilePagePlan>; MAX_BATCH],
    /// 已将脏位转为在途回写状态的待提交区间列表，元素为 `(块视图绝对字节偏移, FolioWriteback)`，预留容量固定为 `WINDOW_RANGES`。
    ranges: Vec<(u64, FolioWriteback)>,
    /// 传递给块层 `writeback_batch` 用于暂存本波待提交父 `Bio` 的工作槽。
    bios: [Option<Bio>; MAX_BATCH],
    /// 传递给块层 `writeback_batch` 用于等待本波父 `Bio` 完成的工作槽，元素为 `(完成句柄, 预期字节数)`。
    completions: [Option<(BioCompletion, usize)>; MAX_BATCH],
    /// 后台预留模式关联的块层预留内存池强引用；普通前台按需构造时为 `None`。
    source: Option<Arc<BioReserve>>,
    /// 本次后台借用期间独占持有的块层预留 `Bio` 通道；前台模式或空闲未借出状态下为 `None`。
    lane: Option<BioLane>,
    /// 借用后台缓冲区时捕获的 `WritebackDomain` 停止代次序号，用于检测卷是否已进入停机流程。
    stop_sequence: Option<u64>,
    /// 借用后台缓冲区时捕获的元数据 I/O 停止边界序号，供映射查询读取Extent块时校验停机状态。
    metadata_boundary: Option<u64>,
}

impl WbStorage {
    /// 为普通前台显式同步分配一套新的回写工作缓冲区（预留 `WINDOW_RANGES` 容量的区间向量，不绑定后台预留通道）。
    pub(crate) fn try_new() -> Result<Self, FsError> {
        Ok(Self {
            selected: [const { None }; MAX_BATCH],
            plans: [const { None }; MAX_BATCH],
            ranges: reserve_ranges()?,
            bios: [const { None }; MAX_BATCH],
            completions: [const { None }; MAX_BATCH],
            source: None,
            lane: None,
            stop_sequence: None,
            metadata_boundary: None,
        })
    }

    /// 在读写卷发布前为后台回写域预分配一套关联指定 `BioReserve` 预留池的工作缓冲区。
    pub(crate) fn try_reserved(source: Arc<BioReserve>) -> Result<Self, FsError> {
        let mut storage = Self::try_new()?;
        storage.source = Some(source);
        Ok(storage)
    }

    /// 在文件系统业务锁外为已预分配的工作缓冲区绑定借入的块层预留通道 `lane` 以及当前卷的数据与元数据停机代次边界。
    pub(crate) fn bind_lane(&mut self, lane: BioLane, stop_sequence: u64, metadata_boundary: u64) {
        assert!(self.source.is_some() && self.lane.is_none());
        assert!(self.stop_sequence.is_none() && self.metadata_boundary.is_none());
        self.lane = Some(lane);
        self.stop_sequence = Some(stop_sequence);
        self.metadata_boundary = Some(metadata_boundary);
    }

    /// 根据借用时捕获的 `metadata_boundary` 构造绑定到外部 `domain` 生命周期的 [`MetadataIoContext`]，供回写期间加载间接 Extent 块使用。
    pub(crate) fn metadata_io_context<'a>(
        &self,
        domain: &'a WritebackDomain,
    ) -> Option<MetadataIoContext<'a>> {
        self.metadata_boundary.map(|boundary| {
            let context = domain.metadata_io_context(boundary);
            assert!(context.belongs_to(self.source.as_ref().expect("reserved source disappeared")));
            context
        })
    }

    /// 检查绑定了后台停机边界的工作缓冲区是否因所属回写域停机或重激活而失效；未绑定后台边界的前台局部缓冲区始终返回 `false`。
    pub(crate) fn stopped(&self, domain: &WritebackDomain) -> bool {
        self.stop_sequence
            .is_some_and(|boundary| domain.storage_stopped(boundary))
    }

    /// 清空工作缓冲区中残留的所有页面引用、回写计划、待提交凭证、`Bio` 句柄并释放预留通道 `lane`，同时保留 `ranges` 向量的已分配容量。
    pub(crate) fn clear(&mut self) {
        for entry in &mut self.selected {
            drop(entry.take());
        }
        for entry in &mut self.plans {
            drop(entry.take());
        }
        self.ranges.clear();
        for entry in &mut self.bios {
            drop(entry.take());
        }
        for entry in &mut self.completions {
            drop(entry.take());
        }
        drop(self.lane.take());
        self.stop_sequence = None;
        self.metadata_boundary = None;
    }
}

impl Drop for WbStorage {
    /// 析构时调用 [`Self::clear`] 释放所有未提交的回写凭证与预留通道。
    fn drop(&mut self) {
        self.clear();
    }
}

/// 同步指定 `inode` 全文件范围（`0..u64::MAX`）内的脏数据页并等待本轮选中的页面落盘完成。
pub(crate) fn sync_inode(inode: &InodeRef) -> Result<(), FsError> {
    sync_inode_range(inode, 0..u64::MAX)
}

/// 同步指定 `inode` 在半开字节区间 `range` 内的全部脏数据页（前台完整同步，不受后台页数预算限制）。
pub(crate) fn sync_inode_range(inode: &InodeRef, range: Range<u64>) -> Result<(), FsError> {
    let mut storage = WbStorage::try_new()?;
    sync_inode_with_storage(inode, range, &mut storage)
}

/// 复用调用方已持有的 `storage` 工作缓冲区同步 `inode` 指定字节区间 `range` 内的脏数据页（供 Inode 淘汰回收等已借有缓冲区的路径调用）。
pub(crate) fn sync_inode_with_storage(
    inode: &InodeRef,
    range: Range<u64>,
    storage: &mut WbStorage,
) -> Result<(), FsError> {
    sync_inode_selection(inode, &range, usize::MAX, false, storage).map(|_| ())
}

/// 从 `inode` 的后台回写游标处起最多选择并同步 `max_pages` 张脏数据页，返回本轮实际标记选中的页面数量。
pub(crate) fn sync_inode_budget(
    inode: &InodeRef,
    max_pages: usize,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    sync_inode_selection(inode, &(0..u64::MAX), max_pages, true, storage)
}

/// 文件数据回写的公共调度入口：进入 `noio` 内存回收保护作用域，在 `mapping` 回写互斥锁下执行选页与提交，并在释放回写锁后刷新 `inode` 的脏状态挂载义务。
fn sync_inode_selection(
    inode: &InodeRef,
    range: &Range<u64>,
    max_pages: usize,
    background: bool,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    let _scope = ReclaimScope::noio();
    if storage.stopped(inode.disk_volume()?.writeback_domain()) {
        return Err(FsError::Io(BlockError::QueueStopped));
    }

    // Step 1: 获取该文件映射的回写睡眠锁；若为后台回写则传入持久游标的可变引用，前台同步则不修改后台游标。
    let result = {
        let mut writeback = inode.mapping().lock_writeback();
        let cursor = background.then_some(&mut *writeback);
        sync_inode_locked(inode, range, max_pages, cursor, storage)
    };
    // Step 2: 若底层设备队列停机未确认（DMA 可能仍在访问物理页），直接原样返回错误，跳过后续脏义务清理。
    if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
        return result;
    }

    // Step 3: 在释放文件数据回写互斥锁之后刷新 Inode 脏链表状态（内部会获取 invalidate 锁，遵守锁顺序契约）。
    let refreshed = inode.refresh_dirty_obligation();
    result.and_then(|selected| refreshed.map(|()| selected))
}

/// 同步指定卷 `volume` 元数据缓存中的全部脏页，并在数据落盘后向块设备发送一次硬件 `FLUSH` 刷盘命令。
pub(crate) fn sync_metadata(volume: &Ext4Volume) -> Result<(), FsError> {
    let mut storage = WbStorage::try_new()?;
    sync_metadata_with_storage(volume, &mut storage)
}

/// 复用已有 `storage` 同步卷 `volume` 的全部脏元数据页并执行硬件 `FLUSH` 刷盘命令。
///
/// 先获取元数据缓存的回写睡眠锁，再向 `BioReserve` 申请独立辅助槽位（`aux`）用于承载 `FLUSH` 请求，避免与持有该回写锁并发执行截断冷读的辅助 I/O 发生死锁。
pub(crate) fn sync_metadata_with_storage(
    volume: &Ext4Volume,
    storage: &mut WbStorage,
) -> Result<(), FsError> {
    let _scope = ReclaimScope::noio();
    let cache = volume.metadata_cache();
    let _writeback = cache.lock_writeback();
    // Step 1: 在元数据回写互斥锁保护下预先申请用于末尾 FLUSH 命令的辅助 Bio 内存容器。
    let source = volume.metadata_cache().reserve();
    let context = storage.metadata_io_context(volume.writeback_domain());
    let aux = match &source {
        Some(source) => {
            Some(source.acquire_aux(context.unwrap_or_else(|| source.metadata_io_context()))?)
        }
        None => None,
    };
    let memory = match &aux {
        Some(aux) => aux.memory()?,
        None => Vec::new_in(BioAllocator::Heap),
    };

    // Step 2: 将元数据缓存中的所有脏页批量提交并等待写 I/O 完成。
    let result = sync_metadata_locked(volume, storage);
    if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
        return result;
    }

    // Step 3: 校验停机边界未发生变化后向底层块视图下发 FLUSH 命令，若失败则将错误记入元数据缓存的 errseq 状态字。
    if let Some(context) = context {
        context.check()?;
    }
    let flush = volume.claim().flush(memory).map_err(FsError::from);
    drop(aux);
    if let Err(error) = flush {
        cache.record_wb_error(error);
    }
    result.and(flush)
}

/// 后台回写入口：从卷元数据缓存的后台游标处起最多选择并回写 `max_pages` 张脏元数据页，仅等待写 I/O 完成而不发送硬件 `FLUSH`。
pub(crate) fn sync_metadata_budget(
    volume: &Ext4Volume,
    max_pages: usize,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    let _scope = ReclaimScope::noio();
    let mut writeback = volume.metadata_cache().lock_writeback();
    sync_metadata_pages_locked(volume, max_pages, Some(&mut *writeback), storage)
}

/// 在调用方已持有卷元数据缓存回写睡眠锁的前提下，将当前全部脏元数据页回写至块设备（不发送末尾 `FLUSH`）。
pub(crate) fn sync_metadata_locked(
    volume: &Ext4Volume,
    storage: &mut WbStorage,
) -> Result<(), FsError> {
    sync_metadata_pages_locked(volume, usize::MAX, None, storage).map(|_| ())
}

/// 元数据缓存页回写的核心滑动窗口循环，供前台全量同步与后台定量扫描共同复用。
///
/// 入口一次性将目标脏页标记为 `FolioMark::ToWrite` 以防并发置脏造成饥饿；仅当本轮所有窗口未发生 `StopUnconfirmed` 时才推进后台游标。
fn sync_metadata_pages_locked(
    volume: &Ext4Volume,
    max_pages: usize,
    background: Option<&mut u64>,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    let cache = volume.metadata_cache();
    let is_background = background.is_some();
    let mut cursor = background.as_deref().copied().unwrap_or(0);
    // Step 1: 一次性在元数据缓存中扫描并标记待回写页面（后台受 max_pages 预算限制并计算下轮续点）。
    let (selected_count, next_index) = if is_background {
        cache.mark_to_write_budget(cursor, u64::MAX, max_pages)
    } else {
        (cache.mark_to_write_range(0, u64::MAX), 0)
    };
    if is_background {
        volume
            .writeback_domain()
            .note_selection(selected_count, true);
    }

    let mut remaining = selected_count;
    let mut first_error = None;
    let mut unconfirmed = false;
    while remaining != 0 {
        let next = cache.select_range(cursor, u64::MAX, FolioMark::ToWrite, &mut storage.selected);
        // Step 2: 摘取当前最多 MAX_BATCH 页窗口，在尚未向本窗口建立任何新在途回写凭证前先等待旧回写完成，避免阻塞已收集前缀的提交。
        for (_, folio) in storage.selected.iter().flatten() {
            folio.wait_writeback();
        }

        // Step 3: 逐页加内容锁收集脏扇区回写凭证并清除缓存索引上的 ToWrite 标记。
        for slot in 0..MAX_BATCH {
            let Some((byte_offset, folio)) = storage.selected[slot].take() else {
                continue;
            };
            remaining -= 1;
            let folio = folio.with_metadata_owner(cache.clone());
            let result = if storage.stopped(volume.writeback_domain()) {
                Err(FsError::Io(BlockError::QueueStopped))
            } else {
                prepare_metadata_page(volume, byte_offset, &folio, storage, is_background)
            };
            unconfirmed |= result == Err(FsError::Io(BlockError::StopUnconfirmed));
            remember_error(&mut first_error, result);
            cache.set_to_write(byte_offset, false);
        }

        // Step 4: 将本窗口收集的所有元数据扇区区间合并提交至块层并等待硬件完成。
        let result = submit_ranges(volume.claim(), storage, volume.writeback_domain());
        unconfirmed |= result == Err(FsError::Io(BlockError::StopUnconfirmed));
        remember_error(&mut first_error, result);
        let Some(next) = next else {
            break;
        };
        cursor = next;
    }

    // Step 5: 若未出现设备停机未确认异常，则更新后台扫描续点游标并返回首个遇到的错误（或选中页数）。
    if !unconfirmed {
        if let Some(cursor) = background {
            *cursor = next_index;
        }
    }
    first_error.map_or(Ok(selected_count), Err)
}

/// 文件数据页回写的核心滑动窗口循环，供前台范围同步与后台定量扫描共同复用。
///
/// 入口先将目标范围内的脏页打上 `FolioMark::ToWrite` 快照标记；每个 `MAX_BATCH` 窗口先查询全部页面的磁盘物理块偏移（避免在持有页面内容锁时发起元数据块读取），再逐页撤销用户态写权限、生成回写凭证并批量提交。
fn sync_inode_locked(
    inode: &InodeRef,
    range: &Range<u64>,
    max_pages: usize,
    background: Option<&mut u64>,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    let (start, end) = page_window(range)?;
    if start == end {
        return Ok(0);
    }

    // Step 1: 按前台全范围或后台预算模式一次性标记 ToWrite 页面，锁定本轮回写工作集。
    let mapping = inode.mapping();
    let start = background.as_deref().copied().unwrap_or(start);
    let (selected_count, next_index) = if background.is_some() {
        mapping.mark_to_write_budget(start, end, max_pages)
    } else {
        (mapping.mark_to_write_range(start, end), 0)
    };
    let volume = inode.disk_volume()?;
    let domain = volume.writeback_domain();
    let is_background = background.is_some();
    if is_background {
        domain.note_selection(selected_count, false);
    }

    let mut remaining = selected_count;
    let mut cursor = start;
    let mut first_error = None;
    let mut unconfirmed = false;
    while remaining != 0 {
        // Step 2 (窗口步骤 A): 摘取最多 MAX_BATCH 张页面，等待旧在途回写结束，并在不持有页面内容锁的情况下解析各块的磁盘物理偏移。
        let next = mapping.select_range(cursor, end, FolioMark::ToWrite, &mut storage.selected);
        for slot in 0..MAX_BATCH {
            let Some((index, folio)) = storage.selected[slot].take() else {
                continue;
            };
            remaining -= 1;
            let folio = folio.with_file_owner(inode.clone());
            folio.wait_writeback();
            if folio.dirty_sectors() == 0 {
                mapping.set_to_write(index, false);
                continue;
            }
            let prepared = if storage.stopped(domain) {
                Err(FsError::Io(BlockError::QueueStopped))
            } else {
                prepare_file_targets(inode, index, range, storage.metadata_io_context(domain))
            };
            match prepared {
                Ok((_, targets)) => {
                    storage.plans[slot] = Some(FilePagePlan {
                        index,
                        folio,
                        targets,
                    })
                }
                Err(error) => {
                    unconfirmed |= error == FsError::Io(BlockError::StopUnconfirmed);
                    mapping.set_to_write(index, false);
                    remember_error(&mut first_error, Err(error));
                }
            }
        }

        // Step 3 (窗口步骤 B): 对已完成块映射的页面获取内容锁、清理用户态可写页表项并生成 FolioWriteback 凭证。
        for slot in 0..MAX_BATCH {
            let Some(plan) = storage.plans[slot].take() else {
                continue;
            };
            let result = prepare_file_page(inode, &plan, storage, is_background);
            unconfirmed |= result == Err(FsError::Io(BlockError::StopUnconfirmed));
            remember_error(&mut first_error, result);
            mapping.set_to_write(plan.index, false);
        }

        // Step 4 (窗口步骤 C): 将本窗口收集的回写区间批量提交至块层并等待 DMA 完成。
        let result = submit_ranges(volume.claim(), storage, domain);
        unconfirmed |= result == Err(FsError::Io(BlockError::StopUnconfirmed));
        remember_error(&mut first_error, result);
        let Some(next) = next else {
            break;
        };
        cursor = next;
    }

    // Step 5: 若设备未发生停机未确认异常，推进后台回写游标并返回本轮首个错误或总选中页数。
    if !unconfirmed {
        if let Some(cursor) = background {
            *cursor = next_index;
        }
    }
    first_error.map_or(Ok(selected_count), Err)
}

/// 在持有 `inode` 块映射读锁（`with_map_read`）期间，解析文件第 `index` 页内与同步字节区间 `range` 重叠的已映射物理块字节偏移。
///
/// 同时返回同一读锁快照下的文件字节大小 `metadata.size`（供文件页填充路径与回写路径共享使用）；映射读锁在此函数返回时即释放，不会跨越后续的数据块 DMA 提交与等待。
pub(crate) fn prepare_file_targets(
    inode: &InodeRef,
    index: u64,
    range: &Range<u64>,
    context: Option<MetadataIoContext<'_>>,
) -> Result<(u64, [Option<u64>; SECTORS_PER_PAGE]), FsError> {
    let volume = inode.disk_volume()?;
    let size = volume.geometry().block_size;
    let page = index
        .checked_mul(PAGE_SIZE as u64)
        .ok_or(FsError::FileTooLarge)?;
    inode.with_map_read(|metadata, root| {
        let mut targets = [None; SECTORS_PER_PAGE];
        for offset in (0..PAGE_SIZE).step_by(size) {
            let position = page
                .checked_add(offset as u64)
                .ok_or(FsError::FileTooLarge)?;
            if metadata.size <= position || range.end <= position {
                break;
            }
            if position.saturating_add(size as u64) <= range.start {
                continue;
            }
            if let BlockMapping::Mapped { physical, .. } =
                volume.map_block(metadata, root, position / size as u64, context)?
            {
                targets[offset / size] =
                    Some(physical.checked_mul(size as u64).ok_or(FsError::Corrupt)?);
            }
        }
        Ok((metadata.size, targets))
    })
}

/// 为已完成物理块地址解析的文件页获取内容锁、撤销用户态共享可写映射，并将脏块转化为待提交的 `FolioWriteback` 凭证。
///
/// 若页面内容锁无法立即非阻塞获取（`try_lock_content` 返回 `None`），会先将 `storage` 中已收集的前序页面凭证提交给块设备驱动，再阻塞等待当前页的内容锁，防止持锁方正在等待前序 I/O 而导致死锁。
fn prepare_file_page(
    inode: &InodeRef,
    plan: &FilePagePlan,
    storage: &mut WbStorage,
    background: bool,
) -> Result<(), FsError> {
    let volume = inode.disk_volume()?;
    let domain = volume.writeback_domain();
    if storage.stopped(domain) {
        return Err(FsError::Io(BlockError::QueueStopped));
    }

    let mut content = match plan.folio.try_lock_content() {
        Some(content) => content,
        None => {
            submit_ranges(volume.claim(), storage, domain)?;
            plan.folio.lock_content()
        }
    };
    inode
        .mapping()
        .mkclean(plan.index, &plan.folio, &mut content);

    let size = volume.geometry().block_size;
    let dirty = plan.folio.dirty_sectors();
    for offset in (0..PAGE_SIZE).step_by(size) {
        if dirty & range_mask(offset, size) == 0 {
            continue;
        }
        let Some(target) = plan.targets[offset / size] else {
            continue;
        };
        // SAFETY:
        // 1. 上方已独占持有当前页面的内容睡眠锁 `content`，且 `inode.mapping().mkclean` 已撤销该页所有用户态共享可写 PTE 并完成全核 TLB 同步，保证回写期间用户态再次写入必定先触发缺页异常重新置脏；
        // 2. 内核对文件页不构造 `&[u8]` 切片，块层通过一致的 `ToDevice` DMA 映射读取物理页；
        // 3. 返回的 `FolioWriteback` 凭证持有物理页与所属 `InodeRef` 强引用，随后移入父 `Bio`，直至硬件 DMA 传输彻底结束后才调用 `complete` 释放。
        if let Some(writeback) = unsafe { content.begin_writeback(offset, size)? } {
            let target = volume.view().absolute_byte(target, size)?;
            storage
                .ranges
                .push((target, writeback.with_writeback_domain(domain, background)));
        }
    }
    Ok(())
}

/// 为单张元数据缓存页获取内容锁，并将页内连续的脏有效扇区合并转化为待提交的 `FolioWriteback` 凭证。
///
/// 若内容锁已被占用，同样先提交 `storage` 中已累积的前序回写区间，再等待在途回写与内容锁。
fn prepare_metadata_page(
    volume: &Ext4Volume,
    byte_offset: u64,
    folio: &FolioRef,
    storage: &mut WbStorage,
    background: bool,
) -> Result<(), FsError> {
    let mut content = match folio.try_lock_content() {
        Some(content) => content,
        None => {
            submit_ranges(volume.claim(), storage, volume.writeback_domain())?;
            folio.wait_writeback();
            folio.lock_content()
        }
    };

    let mut pending = folio.dirty_sectors() & volume.metadata_cache().page_mask(byte_offset);
    while pending != 0 {
        let start = pending.trailing_zeros() as usize;
        let count = (pending >> start).trailing_ones() as usize;
        let offset = start * SECTOR_BYTES as usize;
        let len = count * SECTOR_BYTES as usize;
        let target = byte_offset
            .checked_add(offset as u64)
            .ok_or(BlockError::InvalidRange)?;
        // SAFETY:
        // 1. 元数据缓存页终身不会映射给用户态，且上方已独占持有 `content` 内容锁，排斥任何其他内核线程的可变借用；
        // 2. `begin_writeback` 内部校验 `[offset, offset + len)` 扇区已初始化有效并将脏位原子转入 `writeback` 位图；
        // 3. 返回的 `FolioWriteback` 凭证持有物理页与 `Arc<MetadataCache>` 强引用，保证底层物理页在块设备 DMA 传输彻底完成前持续有效。
        if let Some(writeback) = unsafe { content.begin_writeback(offset, len)? } {
            storage.ranges.push((
                target,
                writeback.with_writeback_domain(volume.writeback_domain(), background),
            ));
        }
        pending &= !range_mask(offset, len);
    }
    Ok(())
}

/// 预分配容量精确为 `WINDOW_RANGES`（64 项）的待提交回写区间向量，确保在持有页面内容锁期间 `push` 不会触发堆重分配。
fn reserve_ranges() -> Result<Vec<(u64, FolioWriteback)>, FsError> {
    let mut ranges = Vec::new();
    ranges
        .try_reserve_exact(WINDOW_RANGES)
        .map_err(|_| FsError::NoMemory)?;
    Ok(ranges)
}

/// 将 `storage.ranges` 中累积的回写区间通过 `claim.writeback_batch` 批量提交至块层，并阻塞等待全部父 `Bio` 完成后返回首个错误。
fn submit_ranges(
    claim: &FsClaim,
    storage: &mut WbStorage,
    domain: &Arc<WritebackDomain>,
) -> Result<(), FsError> {
    if storage.ranges.is_empty() {
        return Ok(());
    }

    let boundary = storage.stop_sequence;
    claim
        .writeback_batch(
            &mut storage.ranges,
            storage.lane.as_ref(),
            &mut storage.bios,
            &mut storage.completions,
            domain,
            || boundary.is_some_and(|boundary| domain.storage_stopped(boundary)),
        )
        .map_err(FsError::from)
}

/// 若 `first` 尚未记录错误且 `result` 为 `Err`，则保存该首次错误，以便后续窗口继续完成剩余页面的回写清理后统一向上汇报。
fn remember_error(first: &mut Option<FsError>, result: Result<(), FsError>) {
    if first.is_none() {
        *first = result.err();
    }
}

/// 将半开文件字节区间 `range` 换算为覆盖该区间的半开文件页号区间 `[start, end)`，正确处理空区间、非页对齐末尾以及 `u64::MAX` 全文件哨兵而不发生算术溢出。
fn page_window(range: &Range<u64>) -> Result<(u64, u64), FsError> {
    if range.end < range.start {
        return Err(FsError::InvalidOperation);
    }
    let start = range.start / PAGE_SIZE as u64;
    if range.start == range.end {
        return Ok((start, start));
    }
    let end = range.end / PAGE_SIZE as u64 + u64::from(range.end % PAGE_SIZE as u64 != 0);
    Ok((start, end))
}

/// 将经过对齐与越界校验的页内字节偏移 `offset` 与长度 `len` 转换为对应的 8 位 512B 扇区位掩码。
fn range_mask(offset: usize, len: usize) -> u8 {
    (((1u16 << (len / SECTOR_BYTES as usize)) - 1) << (offset / SECTOR_BYTES as usize)) as u8
}
