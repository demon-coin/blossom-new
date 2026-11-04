//! 文件与元数据回写共用原页、有限 ToWrite 选择和唯一 FS Bio 批次。

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

const SECTORS_PER_PAGE: usize = PAGE_SIZE / SECTOR_BYTES as usize;
const WINDOW_RANGES: usize = MAX_BATCH * SECTORS_PER_PAGE;

/// 一个有界准备窗口只保存原页引用和真实块地址，不复制页内容。
struct FilePagePlan {
    index: u64,
    folio: FolioRef,
    targets: [Option<u64>; SECTORS_PER_PAGE],
}

/// 一个同步入口拥有固定窗口；储备 book 不保存活 Bio，所有临时拥有在归还前清空。
pub(crate) struct WbStorage {
    selected: [Option<(u64, FolioRef)>; MAX_BATCH],
    plans: [Option<FilePagePlan>; MAX_BATCH],
    ranges: Vec<(u64, FolioWriteback)>,
    bios: [Option<Bio>; MAX_BATCH],
    completions: [Option<(BioCompletion, usize)>; MAX_BATCH],
    source: Option<Arc<BioReserve>>,
    lane: Option<BioLane>,
    stop_sequence: Option<u64>,
    metadata_boundary: Option<u64>,
}

impl WbStorage {
    /// 普通前台在业务锁外准备自己的有限 fallible 窗口，保不同 inode 并发。
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

    /// 卷发表前为原后台域准备同一窗口，不把储备 allocator 容器留在 idle scratch。
    pub(crate) fn try_reserved(source: Arc<BioReserve>) -> Result<Self, FsError> {
        let mut storage = Self::try_new()?;
        storage.source = Some(source);
        Ok(storage)
    }

    /// 业务锁外移入实际 book，两个停止边界均沿本次借用保持。
    pub(crate) fn bind_lane(&mut self, lane: BioLane, stop_sequence: u64, metadata_boundary: u64) {
        assert!(self.source.is_some() && self.lane.is_none());
        assert!(self.stop_sequence.is_none() && self.metadata_boundary.is_none());
        self.lane = Some(lane);
        self.stop_sequence = Some(stop_sequence);
        self.metadata_boundary = Some(metadata_boundary);
    }

    /// ctx 借外部 domain 的真实源，返回生命周期不借可变 scratch 本身。
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

    /// 旧借用不跟随重新 activate，普通局部来源不借后台停止 gate。
    pub(crate) fn stopped(&self, domain: &WritebackDomain) -> bool {
        self.stop_sequence
            .is_some_and(|boundary| domain.storage_stopped(boundary))
    }

    /// 锁外结束临时拥有及 book；实际 Arc/Vec 尾仍由来源 deallocate 继续保住。
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
    /// 普通错误出口也结束未接受的原凭证，不在析构中等待设备或工作。
    fn drop(&mut self) {
        self.clear();
    }
}

/// 同步 inode 的全部文件范围，等待已有 WB 并只结束本轮选择。
pub(crate) fn sync_inode(inode: &InodeRef) -> Result<(), FsError> {
    sync_inode_range(inode, 0..u64::MAX)
}

/// 同步半开文件字节范围；入口完整集合不受后台页预算限制。
pub(crate) fn sync_inode_range(inode: &InodeRef, range: Range<u64>) -> Result<(), FsError> {
    let mut storage = WbStorage::try_new()?;
    sync_inode_with_storage(inode, range, &mut storage)
}

/// 压力及 eviction 沿已经借入的窗口同步，不再次借自己持有的 lane。
pub(crate) fn sync_inode_with_storage(
    inode: &InodeRef,
    range: Range<u64>,
    storage: &mut WbStorage,
) -> Result<(), FsError> {
    sync_inode_selection(inode, &range, usize::MAX, false, storage).map(|_| ())
}

/// 后台只同步本次最多 max_pages 个原页，返回入口实际选中数量。
pub(crate) fn sync_inode_budget(
    inode: &InodeRef,
    max_pages: usize,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    sync_inode_selection(inode, &(0..u64::MAX), max_pages, true, storage)
}

/// 两类同步共用选择、提交和清义务顺序，刷新在数据 WB 互斥结束后取得 invalidate。
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
    let result = {
        let mut writeback = inode.mapping().lock_writeback();
        let cursor = background.then_some(&mut *writeback);
        sync_inode_locked(inode, range, max_pages, cursor, storage)
    };
    if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
        return result;
    }
    let refreshed = inode.refresh_dirty_obligation();
    result.and_then(|selected| refreshed.map(|()| selected))
}

/// 只协调共同元数据阶段及协商的真实 FLUSH，不串行不同 inode 的数据 IO。
pub(crate) fn sync_metadata(volume: &Ext4Volume) -> Result<(), FsError> {
    let mut storage = WbStorage::try_new()?;
    sync_metadata_with_storage(volume, &mut storage)
}

/// 共同 WB 互斥先于独立 FLUSH 单元，允许持该互斥的截断冷读取先结束辅助 IO。
pub(crate) fn sync_metadata_with_storage(
    volume: &Ext4Volume,
    storage: &mut WbStorage,
) -> Result<(), FsError> {
    let _scope = ReclaimScope::noio();
    let cache = volume.metadata_cache();
    let _writeback = cache.lock_writeback();
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
    let result = sync_metadata_locked(volume, storage);
    if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
        return result;
    }
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

/// 后台从共同元数据续点至多选择 max_pages 页，只等实际 IO，不代替 fsync 的 FLUSH。
pub(crate) fn sync_metadata_budget(
    volume: &Ext4Volume,
    max_pages: usize,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    let _scope = ReclaimScope::noio();
    let mut writeback = volume.metadata_cache().lock_writeback();
    sync_metadata_pages_locked(volume, max_pages, Some(&mut *writeback), storage)
}

/// 调用者持共同元数据 WB 互斥；完整同步仍处理入口全部有限集合。
pub(crate) fn sync_metadata_locked(
    volume: &Ext4Volume,
    storage: &mut WbStorage,
) -> Result<(), FsError> {
    sync_metadata_pages_locked(volume, usize::MAX, None, storage).map(|_| ())
}

/// 元数据完整与后台扫描复用预建窗口；仅真实全尾后提交后台续点。
fn sync_metadata_pages_locked(
    volume: &Ext4Volume,
    max_pages: usize,
    background: Option<&mut u64>,
    storage: &mut WbStorage,
) -> Result<usize, FsError> {
    let cache = volume.metadata_cache();
    let is_background = background.is_some();
    let mut cursor = background.as_deref().copied().unwrap_or(0);
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
        // 本窗口还没有新 WB，旧 WB 等待不能阻止已准备前缀获得 kick。
        for (_, folio) in storage.selected.iter().flatten() {
            folio.wait_writeback();
        }
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
        let result = submit_ranges(volume.claim(), storage, volume.writeback_domain());
        unconfirmed |= result == Err(FsError::Io(BlockError::StopUnconfirmed));
        remember_error(&mut first_error, result);
        let Some(next) = next else {
            break;
        };
        cursor = next;
    }
    if !unconfirmed {
        if let Some(cursor) = background {
            *cursor = next_index;
        }
    }
    first_error.map_or(Ok(selected_count), Err)
}

/// 两类文件同步只选一次 ToWrite；先准备整个窗口 targets，后建立任何新 WB。
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
        for slot in 0..MAX_BATCH {
            let Some(plan) = storage.plans[slot].take() else {
                continue;
            };
            let result = prepare_file_page(inode, &plan, storage, is_background);
            unconfirmed |= result == Err(FsError::Io(BlockError::StopUnconfirmed));
            remember_error(&mut first_error, result);
            mapping.set_to_write(plan.index, false);
        }
        let result = submit_ranges(volume.claim(), storage, domain);
        unconfirmed |= result == Err(FsError::Io(BlockError::StopUnconfirmed));
        remember_error(&mut first_error, result);
        let Some(next) = next else {
            break;
        };
        cursor = next;
    }
    if !unconfirmed {
        if let Some(cursor) = background {
            *cursor = next_index;
        }
    }
    first_error.map_or(Ok(selected_count), Err)
}

/// 回写和填充共用真实块地址及同根尺寸；根读保护不跨数据 IO、额度或完成等待。
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

/// 当前内容锁不能立即取得时，先提交并通知已准备的前缀再进入真实等待。
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
        // SAFETY: 本内容 guard 排斥许可提交；mkclean 已撤同源共享 W 并完成全 CPU CSD。
        // 原页使用 coherent ToDevice，之后的原始写另记 Dirty，不导出 Rust 内容切片；
        // 返回凭证移入唯一父 Bio，设备及生成者全尾前始终保活 inode 和原页。
        if let Some(writeback) = unsafe { content.begin_writeback(offset, size)? } {
            let target = volume.view().absolute_byte(target, size)?;
            storage
                .ranges
                .push((target, writeback.with_writeback_domain(domain, background)));
        }
    }
    Ok(())
}

/// 元数据 WB 只覆盖真实有效窗口，并为所有卷级同步保留同一实际完成统计域。
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
        // SAFETY: 元数据终身不授给用户，content guard 排斥可变借用；begin_writeback
        // 校验准确窗口已初始化并清脏，原 cache/claim 由凭证保活到真实设备全尾。
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

/// 为最多八页的准确 sector 范围预留存储，内容锁内 push 不分配。
fn reserve_ranges() -> Result<Vec<(u64, FolioWriteback)>, FsError> {
    let mut ranges = Vec::new();
    ranges
        .try_reserve_exact(WINDOW_RANGES)
        .map_err(|_| FsError::NoMemory)?;
    Ok(ranges)
}

/// 父批次沿原核心统一 kick 并观察全部完成后返回首错。
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

/// 保留本轮首次错误，后续父仍完成自己的全部尾部。
fn remember_error(first: &mut Option<FsError>, result: Result<(), FsError>) {
    if first.is_none() {
        *first = result.err();
    }
}

/// 换算半开 byte 范围，整文件哨兵和非页对齐尾都不溢出。
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

/// 把已经校验的页内准确范围转换为原 PageDesc 的八位 sector mask。
fn range_mask(offset: usize, len: usize) -> u8 {
    (((1u16 << (len / SECTOR_BYTES as usize)) - 1) << (offset / SECTOR_BYTES as usize)) as u8
}
