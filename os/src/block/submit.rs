//! 显式批次只保留固定生成者与准备槽，设备访问始终使用原页的有界 SG。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::convert::TryFrom;

use virtio_drivers::BufferDirection;

use crate::block::bio::BioInner;
use crate::block::disk::{ActiveBio, PreparingSlot};
#[cfg(feature = "ktest")]
use crate::block::DiskIdentity;
use crate::block::{
    Bio, BioAllocator, BioCompletion, BioLane, BioLease, BioMemory, BlkOperation, BlockError, Disk,
    DiskRef, FsClaim, IoResult, QueueLimits, MAX_BATCH, SECTOR_BYTES,
};
use crate::fs::background::WritebackDomain;
use crate::mm::dma::{MappedPages, PayloadRange};
use crate::mm::{alloc_pages, FolioRead, FolioWriteback, PAGE_SIZE};
#[cfg(feature = "ktest")]
use crate::sync::SpinLock;
use crate::task;

/// 当前位置只引用父 Bio 的段编号，跨 child 保留游标避免从首页反复扫描。
#[derive(Clone, Copy, Default)]
struct SegmentCursor {
    index: usize,
    within: usize,
}

impl SegmentCursor {
    /// 按真实原段消耗字节；游标从不复制页拥有权或访问 payload。
    fn advance(&mut self, bio: &BioInner, mut bytes: usize) -> Result<(), BlockError> {
        while 0 < bytes {
            let segment = bio.segment(self.index).ok_or(BlockError::InvalidRange)?;
            let available = (segment.length as usize)
                .checked_sub(self.within)
                .ok_or(BlockError::InvalidRange)?;
            if available == 0 {
                return Err(BlockError::InvalidRange);
            }
            let consumed = available.min(bytes);
            self.within += consumed;
            bytes -= consumed;
            if self.within == segment.length as usize {
                self.index += 1;
                self.within = 0;
            }
        }
        Ok(())
    }
}

/// 尚未发表的计划只保存固定元数据，发表成功后才推进各父的已生成位置。
struct GenerationPlan {
    indices: [usize; MAX_BATCH],
    lengths: [usize; MAX_BATCH],
    cursors: [SegmentCursor; MAX_BATCH],
    count: usize,
    #[cfg(feature = "ktest")]
    splits: usize,
}

#[cfg(feature = "ktest")]
const READ_BATCH_CAPTURE_PAGES: usize = 32;

/// 最近一波原页读的真实接受事实；地址只复制数值，不延长页、inode 或 claim 的寿命。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct ReadBatchSnapshot {
    pub sequence: u64,
    pub disk: DiskIdentity,
    pub bios: usize,
    pub multi_page_bios: usize,
    pub pages: usize,
    pub accepted_bytes: usize,
    pub first_sector: u64,
    /// 只保存本波前三十二个原映射中已完整接受的页，数组有效长度由本字段给出。
    pub captured_pages: usize,
    pub phys_addrs: [usize; READ_BATCH_CAPTURE_PAGES],
    pub dma_addrs: [u64; READ_BATCH_CAPTURE_PAGES],
}

#[cfg(feature = "ktest")]
static READ_BATCH: SpinLock<Option<ReadBatchSnapshot>> = SpinLock::new(None);

/// 提交前保存原映射地址，提交后只读取原父接受账，设备完成者不访问本诊断存储。
#[cfg(feature = "ktest")]
struct ReadBatchObservation {
    snapshot: ReadBatchSnapshot,
    parents: [Option<(BioCompletion, usize)>; MAX_BATCH],
}

#[cfg(feature = "ktest")]
impl ReadBatchObservation {
    /// 原 Bio 尚未交设备时复制地址并保有限父观察者，避免与完成者移出 memory 的窗口竞争。
    fn new(claim: &FsClaim, bios: &[Bio]) -> Self {
        let mut observed = Self {
            snapshot: ReadBatchSnapshot {
                sequence: 0,
                disk: claim.disk().identity(),
                bios: 0,
                multi_page_bios: 0,
                pages: 0,
                accepted_bytes: 0,
                first_sector: bios.first().map_or(0, |bio| bio.inner.sector()),
                captured_pages: 0,
                phys_addrs: [0; READ_BATCH_CAPTURE_PAGES],
                dma_addrs: [0; READ_BATCH_CAPTURE_PAGES],
            },
            parents: [const { None }; MAX_BATCH],
        };
        for (slot, bio) in bios.iter().enumerate() {
            let pages = bio.inner.segment_count();
            observed.parents[slot] = Some((bio.completion(), pages));
            for index in 0..pages {
                let output = observed.snapshot.captured_pages;
                if output == observed.snapshot.phys_addrs.len() {
                    break;
                }
                let (physical, offset) = bio
                    .inner
                    .ktest_folio_origin(index * PAGE_SIZE)
                    .expect("read batch original folio missing");
                assert_eq!(offset, 0, "read batch has a partial folio");
                observed.snapshot.phys_addrs[output] = physical;
                observed.snapshot.dma_addrs[output] = bio
                    .inner
                    .segment(index)
                    .expect("read batch DMA segment missing")
                    .physical as u64;
                observed.snapshot.captured_pages += 1;
            }
        }
        observed
    }

    /// 只统计实际 driver.submit 接受的完整原页；登记、尝试或失败父不冒充已提交。
    fn publish(mut self) {
        let captured = self.snapshot.captured_pages;
        let mut source = 0;
        let mut output = 0;
        for (completion, pages) in IntoIterator::into_iter(self.parents).flatten() {
            let accepted = completion.test_acceptance().2;
            let accepted_pages = accepted / PAGE_SIZE;
            self.snapshot.accepted_bytes += accepted;
            self.snapshot.bios += usize::from(0 < accepted);
            self.snapshot.multi_page_bios += usize::from(1 < accepted_pages);
            self.snapshot.pages += accepted_pages;
            for page in 0..accepted_pages.min(captured.saturating_sub(source)) {
                self.snapshot.phys_addrs[output] = self.snapshot.phys_addrs[source + page];
                self.snapshot.dma_addrs[output] = self.snapshot.dma_addrs[source + page];
                output += 1;
            }
            source += pages;
        }
        self.snapshot.captured_pages = output;
        let mut last = READ_BATCH.lock();
        self.snapshot.sequence = last
            .as_ref()
            .map_or(1, |last| last.sequence.wrapping_add(1));
        *last = Some(self.snapshot);
    }
}

impl FsClaim {
    /// 以本占用提交 FS 请求，Write 仅允许 RW；Read 与设备支持的 Flush 可由 RO 使用。
    /// 接受的请求独立保占用至生成、设备收权、回调及原队列资源的全部尾部结束。
    pub fn submit_bio(&self, bio: Bio) {
        self.disk().submit_bio_from(bio, Some(self));
    }

    /// 显式 FS 批次沿同一原 Bio 准入、有限合并及 split 路径，不另建提交引擎。
    pub fn submit_batch(&self, bios: impl IntoIterator<Item = Bio>) {
        self.disk().submit_batch_from(bios, Some(self));
    }

    /// 同步字节读取保实际范围占用到 slot 真尾，返回前只复制完整成功的数据。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "文件写入和截断测试通过原 FS 占用读取实际设备字节")
    )]
    pub fn read_exact_at(&self, offset: u64, dst: &mut [u8]) -> Result<(), BlockError> {
        let absolute = self.view().absolute_byte(offset, dst.len())?;
        drop(task::current_task());
        let result = self.disk().read_with_pages(absolute, dst, Some(self));
        self.disk().unplug();
        result
    }

    /// 将卷内准确原页范围换算一次，并保本占用至 accepted 与 slot 的全部完成尾。
    pub(crate) fn load_folio(
        &self,
        byte: u64,
        read: FolioRead,
        memory: BioMemory,
    ) -> Result<(), BlockError> {
        let len = read.len();
        let absolute = match self.view().absolute_byte(byte, len) {
            Ok(absolute) => absolute,
            Err(error) => {
                fail_read(read, error);
                return Err(error);
            }
        };
        let (completion, result) =
            self.disk()
                .read_folio_bio(absolute, read, None, memory, Some(self))?;
        drop(completion.take_memory()?);
        if let Some(error) = result.error {
            return Err(error);
        }
        if result.completed != len {
            return Err(BlockError::ShortIo);
        }
        Ok(())
    }

    /// 页内分段读取携带实际卷占用，成功后原读入者继续拥有未完成的页面。
    pub fn fill_folio_range(
        &self,
        byte: u64,
        read: FolioRead,
        offset: usize,
        len: usize,
    ) -> Result<FolioRead, BlockError> {
        let absolute = match self.view().absolute_byte(byte, len) {
            Ok(absolute) => absolute,
            Err(error) => {
                fail_read(read, error);
                return Err(error);
            }
        };
        let (completion, result) = self.disk().read_folio_bio(
            absolute,
            read,
            Some((offset, len)),
            Vec::new_in(BioAllocator::Heap),
            Some(self),
        )?;
        finish_folio_range(completion, result, len)
    }

    /// 连续磁盘范围组合多个整页读入凭证，每波最多八父，提交后不逐页等待真实完成。
    /// 准备拒尾仍提交已有父，所有未交设备的原凭证发布同一个具体准备错误。
    pub(crate) fn read_folio_batch(
        &self,
        ranges: &mut Vec<(u64, FolioRead)>,
    ) -> Result<(), BlockError> {
        let mut pending = ranges.drain(..);
        while pending.len() != 0 {
            let mut bios = Vec::new();
            if bios.try_reserve_exact(MAX_BATCH).is_err() {
                fail_pending_reads(&mut pending, BlockError::NoMemory);
                return Err(BlockError::NoMemory);
            }
            let mut preparation_error = None;
            for _ in 0..MAX_BATCH {
                if pending.len() == 0 {
                    break;
                }
                match self.prepare_read_bio(&mut pending) {
                    Ok((bio, error)) => {
                        bios.push(bio);
                        if error.is_some() {
                            preparation_error = error;
                            break;
                        }
                    }
                    Err(error) => {
                        preparation_error = Some(error);
                        break;
                    }
                }
            }
            #[cfg(feature = "ktest")]
            let observation = ReadBatchObservation::new(self, &bios);
            self.submit_batch(bios);
            #[cfg(feature = "ktest")]
            observation.publish();
            if let Some(error) = preparation_error {
                fail_pending_reads(&mut pending, error);
                return Err(error);
            }
        }
        Ok(())
    }

    /// 只合并整页且物理 byte 相邻的读范围，部分映射准备失败时保留已映射前缀供本波提交。
    fn prepare_read_bio(
        &self,
        pending: &mut alloc::vec::Drain<'_, (u64, FolioRead)>,
    ) -> Result<(Bio, Option<BlockError>), BlockError> {
        let byte_offset = pending.as_slice()[0].0;
        if byte_offset % SECTOR_BYTES != 0 {
            return Err(BlockError::Misaligned);
        }
        let mut end = byte_offset;
        let mut count = 0;
        for (offset, read) in pending.as_slice() {
            if *offset != end {
                break;
            }
            if read.offset() != 0 || read.len() != PAGE_SIZE {
                break;
            }
            end = end
                .checked_add(PAGE_SIZE as u64)
                .ok_or(BlockError::InvalidRange)?;
            count += 1;
        }
        if count == 0 {
            return Err(BlockError::InvalidRange);
        }
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(count)
            .map_err(|_| BlockError::NoMemory)?;
        let mut preparation_error = None;
        for _ in 0..count {
            let (_, read) = pending.next().expect("read batch prefix disappeared");
            match MappedPages::from_folio(read) {
                Ok(mapped) => memory.push(mapped),
                Err((error, read)) => {
                    let error = BlockError::from(error);
                    fail_read(read, error);
                    preparation_error = Some(error);
                    break;
                }
            }
        }
        if memory.is_empty() {
            return Err(preparation_error.expect("empty read batch has no preparation error"));
        }
        // SAFETY: 每份 read 独占一个原页内容锁且保原 inode，CPU 初始化别名均已结束。
        // 连续磁盘范围直接映射这些原页；无回调，Bio 在全部真实 child 和生成者结束后
        // 才收权并发布各页结果，FS claim 由同一正式准入保存到原队列全尾。
        let bio = unsafe {
            Bio::try_new(
                self.disk(),
                BlkOperation::Read,
                self.view()
                    .absolute_byte(byte_offset, memory.iter().map(MappedPages::len).sum())?
                    / SECTOR_BYTES,
                memory,
                0,
                None,
            )
        }?;
        Ok((bio, preparation_error))
    }

    /// 复制最近本盘读批次的真实接受事实，不等待或控制 I/O，也不保存设备拥有者。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_read_batch_snapshot(&self) -> Option<ReadBatchSnapshot> {
        let snapshot = *READ_BATCH.lock();
        snapshot.filter(|snapshot| snapshot.disk == self.disk().identity())
    }

    /// 多原页相邻范围共用一个父，固定父槽和观察者复用同一预建窗口。
    /// 准备拒尾仍先提交已映射前缀，实际全尾后才允许 book 内的后继窗重用存储。
    pub(crate) fn writeback_batch(
        &self,
        ranges: &mut Vec<(u64, FolioWriteback)>,
        lane: Option<&BioLane>,
        bios: &mut [Option<Bio>; MAX_BATCH],
        completions: &mut [Option<(BioCompletion, usize)>; MAX_BATCH],
        domain: &Arc<WritebackDomain>,
        stopped: impl Fn() -> bool,
    ) -> Result<(), BlockError> {
        assert!(bios.iter().all(Option::is_none));
        assert!(completions.iter().all(Option::is_none));
        let mut pending = ranges.drain(..);
        while pending.len() != 0 {
            let width = lane.map_or(MAX_BATCH, BioLane::len);
            let (first_slot, mut first_lease) = match lane {
                Some(lane) => {
                    let (slot, lease) = lane.lease_available(&stopped)?;
                    (slot, Some(lease))
                }
                None => (0, None),
            };
            let mut preparation_error = None;
            for step in 0..width {
                let slot = (first_slot + step) % width;
                if pending.len() == 0 {
                    break;
                }
                let lease = if step == 0 {
                    first_lease.take()
                } else {
                    match lane {
                        Some(lane) => match lane.try_lease(slot, &stopped) {
                            Ok(Some(lease)) => Some(lease),
                            Ok(None) => continue,
                            Err(error) => {
                                preparation_error = Some(error);
                                break;
                            }
                        },
                        None => None,
                    }
                };
                let prepared = self.prepare_writeback_bio(&mut pending, lease);
                match prepared {
                    Ok((bio, len, error)) => {
                        bio.inner.track_writeback(domain);
                        completions[step] = Some((bio.completion(), len));
                        bios[step] = Some(bio);
                        if error.is_some() {
                            preparation_error = error;
                            break;
                        }
                    }
                    Err(error) => {
                        preparation_error = Some(error);
                        break;
                    }
                }
            }
            self.submit_batch(bios.iter_mut().filter_map(Option::take));
            let mut first_error = None;
            for entry in completions.iter_mut() {
                let Some((completion, len)) = entry.take() else {
                    continue;
                };
                let result = completion.wait();
                let error = result
                    .error
                    .or_else(|| (result.completed < len).then_some(BlockError::ShortIo));
                if first_error.is_none() {
                    first_error = error;
                }
                match completion.take_memory() {
                    Ok(memory) => drop(memory),
                    Err(error) => {
                        first_error.get_or_insert(error);
                    }
                }
                drop(completion);
            }
            if let Some(error) = first_error.or(preparation_error) {
                return Err(error);
            }
        }
        Ok(())
    }

    /// 只合并真实连续 byte 范围，储备父最多八映射；失败前的映射仍构造真实父提交。
    fn prepare_writeback_bio(
        &self,
        pending: &mut alloc::vec::Drain<'_, (u64, FolioWriteback)>,
        lease: Option<BioLease>,
    ) -> Result<(Bio, usize, Option<BlockError>), BlockError> {
        let mut memory = match &lease {
            Some(lease) => lease.memory()?,
            None => Vec::new_in(BioAllocator::Heap),
        };
        let byte_offset = pending.as_slice()[0].0;
        if byte_offset % SECTOR_BYTES != 0 {
            return Err(BlockError::Misaligned);
        }
        let mut end = byte_offset;
        let mut count = 0;
        for (offset, writeback) in pending
            .as_slice()
            .iter()
            .take(memory.allocator().mapping_limit())
        {
            if *offset != end {
                break;
            }
            end = end
                .checked_add(writeback.len() as u64)
                .ok_or(BlockError::InvalidRange)?;
            count += 1;
        }
        memory
            .try_reserve_exact(count)
            .map_err(|_| BlockError::NoMemory)?;
        let mut len = 0;
        let mut preparation_error = None;
        for _ in 0..count {
            let (_, writeback) = pending.next().expect("writeback prefix disappeared");
            match MappedPages::from_writeback(writeback) {
                Ok(mapped) => {
                    len += mapped.len();
                    memory.push(mapped);
                }
                Err((error, writeback)) => {
                    drop(writeback);
                    preparation_error = Some(BlockError::from(error));
                    break;
                }
            }
        }
        if memory.is_empty() {
            return Err(preparation_error.expect("empty writeback prefix has no error"));
        }
        // SAFETY: 凭证来自已初始化原页，已完成共享撤权及 CSD，不导出内容切片。
        // 每映射只记录自己的真实接受和未接受恢复；全部 child 与生成者收权前不结束 WB。
        let bio = unsafe {
            Bio::try_new(
                self.disk(),
                BlkOperation::Write,
                byte_offset / SECTOR_BYTES,
                memory,
                0,
                None,
            )
        }?;
        drop(lease);
        Ok((bio, len, preparation_error))
    }

    /// 使用调用者业务锁外准备的同一来源提交真实 FLUSH，内部观察者全部结束后返回。
    pub(crate) fn flush(&self, memory: BioMemory) -> Result<(), BlockError> {
        // SAFETY: Flush 没有数据映射或 CPU 别名，也不含回调；
        // genuine Bio 仍承担生成者、全部 child 和真实完成责任。
        let bio = unsafe { Bio::try_new(self.disk(), BlkOperation::Flush, 0, memory, 0, None) }?;
        let completion = bio.completion();
        self.submit_bio(bio);
        self.disk().unplug();
        let result = completion.wait();
        let memory = completion.take_memory();
        drop(completion);
        drop(memory?);
        result.error.map_or(Ok(()), Err)
    }
}

/// 所有这些读入凭证都尚未交设备，逐份结束准确失败责任，不由析构改成取消错误。
fn fail_pending_reads(pending: &mut alloc::vec::Drain<'_, (u64, FolioRead)>, error: BlockError) {
    for (_, read) in pending {
        fail_read(read, error);
    }
}

/// 准备失败直接发布原页错误；失败不初始化字节、不零填或重读。
fn fail_read(mut read: FolioRead, error: BlockError) {
    // SAFETY: 本凭证从未交设备，CPU 借用已结束；零完成错误不授予任何内容访问。
    unsafe {
        read.complete(IoResult {
            completed: 0,
            error: Some(error),
        });
    }
}

impl Disk {
    /// raw 提交不携 FS 授权，任何活占用期间拒绝 Write；父完成仍由观察者等待。
    pub(crate) fn submit_bio(self: &DiskRef, bio: Bio) {
        self.submit_bio_from(bio, None);
    }

    /// 唯一单父提交先验证来源，再同步生成原 child；失败仍 kick 已发表前缀。
    fn submit_bio_from(self: &DiskRef, bio: Bio, claim: Option<&FsClaim>) {
        drop(task::current_task());
        let mut active: [Option<ActiveBio<'_>>; MAX_BATCH] = [const { None }; MAX_BATCH];
        match self.admit_from(bio, claim) {
            Ok(accepted) => active[0] = Some(accepted),
            Err((bio, error)) => {
                self.unplug();
                BioInner::reject(&bio.inner, error);
                return;
            }
        }
        self.submit_accepted(&mut active);
    }

    /// raw 批次没有 FS 授权，每组最多八个生成者，组内相邻普通范围才允许合并。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "块队列测试验证无 FS 授权的显式批次合并及连续完成前缀"
        )
    )]
    pub(crate) fn submit_batch(self: &DiskRef, bios: impl IntoIterator<Item = Bio>) {
        self.submit_batch_from(bios, None);
    }

    /// 两种来源共用批次准入及 child 生成，拒绝的父只清自身准备资源。
    fn submit_batch_from(
        self: &DiskRef,
        bios: impl IntoIterator<Item = Bio>,
        claim: Option<&FsClaim>,
    ) {
        drop(task::current_task());
        let mut pending = bios.into_iter().peekable();
        loop {
            let mut active: [Option<ActiveBio<'_>>; MAX_BATCH] = [const { None }; MAX_BATCH];
            for entry in &mut active {
                let Some(bio) = pending.next() else {
                    break;
                };
                match self.admit_from(bio, claim) {
                    Ok(accepted) => *entry = Some(accepted),
                    Err((bio, error)) => {
                        self.unplug();
                        BioInner::reject(&bio.inner, error);
                    }
                }
            }
            self.submit_accepted(&mut active);
            if pending.peek().is_none() {
                return;
            }
        }
    }

    /// 每个 accepted root 都在通知已发表前缀后结束生成责任，失败不取消旧 child。
    fn submit_accepted(&self, active: &mut [Option<ActiveBio<'_>>; MAX_BATCH]) {
        let mut generated = [false; MAX_BATCH];
        let error = self.generate_children(active, &mut generated).err();
        self.unplug();
        for (index, entry) in active.iter_mut().enumerate() {
            if let Some(accepted) = entry.take() {
                let failure = if generated[index] {
                    None
                } else {
                    error.map(|error| (accepted.offset, error))
                };
                accepted.finish(failure);
            }
        }
    }

    /// 准备与发表按 child 顺序执行；描述符等待由同一个 PreparingSlot 保有 SG。
    fn generate_children(
        &self,
        active: &mut [Option<ActiveBio<'_>>; MAX_BATCH],
        generated: &mut [bool; MAX_BATCH],
    ) -> Result<(), BlockError> {
        let mut cursors = [SegmentCursor::default(); MAX_BATCH];
        let mut next = next_active(active, 0);
        while next < MAX_BATCH {
            let mut slot = self.reserve_slot()?;
            let plan = self.prepare_child(&mut slot, active, cursors, next)?;
            slot.publish()?;
            #[cfg(feature = "ktest")]
            self.record_generation(plan.splits, plan.count - 1);
            for part in 0..plan.count {
                let index = plan.indices[part];
                let accepted = active[index].as_mut().expect("planned Bio is absent");
                accepted.offset += plan.lengths[part];
                cursors[index] = plan.cursors[index];
                generated[index] = accepted.bio.operation() == BlkOperation::Flush
                    || accepted.offset == accepted.bio.len();
                next = if generated[index] {
                    next_active(active, index + 1)
                } else {
                    index
                };
            }
        }
        Ok(())
    }

    /// 锁外填充一个固定槽，逐父裁剪到逻辑块边界后才保存对应完成义务。
    fn prepare_child(
        &self,
        slot: &mut PreparingSlot<'_>,
        active: &[Option<ActiveBio<'_>>; MAX_BATCH],
        cursors: [SegmentCursor; MAX_BATCH],
        first: usize,
    ) -> Result<GenerationPlan, BlockError> {
        let root = active[first].as_ref().expect("first Bio is absent");
        let operation = root.bio.operation();
        let sector = root
            .bio
            .sector()
            .checked_add((root.offset as u64) / SECTOR_BYTES)
            .ok_or(BlockError::InvalidRange)?;
        let mut plan = GenerationPlan {
            indices: [0; MAX_BATCH],
            lengths: [0; MAX_BATCH],
            cursors,
            count: 0,
            #[cfg(feature = "ktest")]
            splits: 0,
        };
        if operation == BlkOperation::Flush {
            slot.push_part(Arc::clone(&root.bio), 0, 0)?;
            slot.configure(operation, 0, 0)?;
            plan.indices[0] = first;
            plan.count = 1;
            return Ok(plan);
        }
        let limits = self.limits();
        let mut bytes = 0;
        let mut index = first;
        while index < MAX_BATCH && slot.part_count() < MAX_BATCH {
            let accepted = active[index].as_ref().expect("merge Bio is absent");
            let bio = &accepted.bio;
            let current_sector = bio
                .sector()
                .checked_add((accepted.offset as u64) / SECTOR_BYTES)
                .ok_or(BlockError::InvalidRange)?;
            let end_sector = sector
                .checked_add((bytes as u64) / SECTOR_BYTES)
                .ok_or(BlockError::InvalidRange)?;
            if bio.operation() != operation || current_sector != end_sector {
                break;
            }
            // 可睡回调可能等待相邻父；合并到同一槽会让它阻塞后续 part 的完成。
            if plan.count != 0 && (root.bio.needs_task() || bio.needs_task()) {
                break;
            }
            let remaining = bio
                .len()
                .checked_sub(accepted.offset)
                .ok_or(BlockError::InvalidRange)?;
            let budget = remaining.min(limits.max_bytes - bytes);
            let length =
                append_prefix(bio, &mut plan.cursors[index], slot.sg_mut(), limits, budget)?;
            if length == 0 {
                if plan.count == 0 {
                    return Err(BlockError::Unsupported);
                }
                break;
            }
            slot.push_part(Arc::clone(bio), accepted.offset, length)?;
            plan.indices[plan.count] = index;
            plan.lengths[plan.count] = length;
            plan.count += 1;
            #[cfg(feature = "ktest")]
            {
                plan.splits += usize::from(0 < accepted.offset);
            }
            bytes += length;
            if length < remaining || bytes == limits.max_bytes {
                break;
            }
            index = next_active(active, index + 1);
        }
        debug_assert!(0 < bytes && !slot.sg().is_empty());
        slot.configure(operation, sector, bytes)?;
        Ok(plan)
    }

    /// 将原 folio 的完整读入范围交给 Bio，等待真实 DMA 全尾及内容结果发布。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "原页缓存测试用无 FS 占用的真实 Disk 验证读入、OOM 和完成收权"
        )
    )]
    pub(crate) fn load_folio(
        self: &DiskRef,
        byte_offset: u64,
        read: FolioRead,
        memory: BioMemory,
    ) -> Result<(), BlockError> {
        let len = read.len();
        let (completion, result) = self.read_folio_bio(byte_offset, read, None, memory, None)?;
        let memory = completion.take_memory();
        drop(completion);
        drop(memory?);
        if let Some(error) = result.error {
            return Err(error);
        }
        if result.completed != len {
            return Err(BlockError::ShortIo);
        }
        Ok(())
    }

    /// 锁外准备原页 Bio 并同步等待；None 结束完整读，子范围成功后由来源继续填页。
    fn read_folio_bio(
        self: &DiskRef,
        byte_offset: u64,
        mut read: FolioRead,
        range: Option<(usize, usize)>,
        mut memory: BioMemory,
        claim: Option<&FsClaim>,
    ) -> Result<(BioCompletion, IoResult), BlockError> {
        drop(task::current_task());
        let prepared = if byte_offset % SECTOR_BYTES != 0 {
            Err(BlockError::Misaligned)
        } else {
            memory
                .try_reserve_exact(1)
                .map_err(|_| BlockError::NoMemory)
        };
        if let Err(error) = prepared {
            // SAFETY: read 尚未映射或交设备，调用期间 CPU 可变借用已结束；
            // 错误不授予任何初始化范围，所有等待者取得同一真实错误。
            unsafe {
                read.complete(IoResult {
                    completed: 0,
                    error: Some(error),
                })
            };
            return Err(error);
        }
        let mapped = match range {
            Some((offset, len)) => MappedPages::from_folio_range(read, offset, len),
            None => MappedPages::from_folio(read),
        };
        let mapped = match mapped {
            Ok(mapped) => mapped,
            Err((error, mut read)) => {
                let error = BlockError::from(error);
                // SAFETY: 映射准备失败，设备从未取得范围，失败不发布可读内容。
                unsafe {
                    read.complete(IoResult {
                        completed: 0,
                        error: Some(error),
                    })
                };
                return Err(error);
            }
        };
        memory.push(mapped);
        // SAFETY: 唯一 read owner 保活并独占内容锁，全部 CPU 借用已经结束；
        // Read 无回调且不要求初始内容有效，Bio 全尾真实收权后才结束本段责任。
        let bio = unsafe {
            Bio::try_new(
                self,
                BlkOperation::Read,
                byte_offset / SECTOR_BYTES,
                memory,
                0,
                None,
            )
        }?;
        let completion = bio.completion();
        self.submit_bio_from(bio, claim);
        self.unplug();
        let result = completion.wait();
        Ok((completion, result))
    }

    /// 普通 Task 用请求自有零页完成字节范围读取，只有全长成功才修改调用者缓冲区。
    /// 非空范围必须符合设备逻辑块对齐；文件页填充直接使用自己的页提交 Bio。
    pub fn read_exact_at(self: &DiskRef, offset: u64, dst: &mut [u8]) -> Result<(), BlockError> {
        drop(task::current_task());
        let result = self.read_with_pages(offset, dst, None);
        self.unplug();
        result
    }

    /// 所有 fallible 页与 Vec 准备都在队列锁外，错误完成仍通过原拥有者归还原阶。
    fn read_with_pages(
        self: &DiskRef,
        offset: u64,
        dst: &mut [u8],
        claim: Option<&FsClaim>,
    ) -> Result<(), BlockError> {
        let limits = self.limits();
        let logical = u64::from(limits.logical_block_size);
        let capacity = limits
            .capacity_sectors
            .checked_mul(SECTOR_BYTES)
            .ok_or(BlockError::InvalidRange)?;
        let end = offset
            .checked_add(u64::try_from(dst.len()).map_err(|_| BlockError::InvalidRange)?)
            .ok_or(BlockError::InvalidRange)?;
        if capacity < end {
            return Err(BlockError::InvalidRange);
        }
        if dst.is_empty() {
            return Ok(());
        }
        if offset % logical != 0 || (dst.len() as u64) % logical != 0 {
            return Err(BlockError::Misaligned);
        }
        let total = dst.len();
        let page_count = total / PAGE_SIZE + usize::from(total % PAGE_SIZE != 0);
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(page_count)
            .map_err(|_| BlockError::NoMemory)?;
        let mut remaining = total;
        while 0 < remaining {
            let length = remaining.min(PAGE_SIZE);
            let pages = alloc_pages(0, true).ok_or(BlockError::NoMemory)?;
            let mapped = MappedPages::new(pages, 0, length, BufferDirection::DeviceToDriver)
                .map_err(BlockError::from)?;
            memory.push(mapped);
            remaining -= length;
        }
        // SAFETY: 每个映射持有独占原零页，全部 CPU 别名在移入前结束；读请求无回调，
        // 页在 Bio 最终完成前不被 CPU 使用，失败与短读也不将未成功数据复制到 dst。
        let bio = unsafe {
            Bio::try_new(
                self,
                BlkOperation::Read,
                offset / SECTOR_BYTES,
                memory,
                0,
                None,
            )
        }?;
        let completion = bio.completion();
        self.submit_bio_from(bio, claim);
        self.unplug();
        let result = completion.wait();
        let memory = completion.take_memory()?;
        if let Some(error) = result.error {
            return Err(error);
        }
        if result.completed != total {
            return Err(BlockError::ShortIo);
        }
        let mut copied = 0;
        for mapped in &memory {
            // SAFETY: 原页分配时整页清零，整个读取范围已全长成功；wait/take_memory
            // 已结束所有 child 的 DMA 并收回 CPU 拥有权，局部拥有者禁止并发写与提前归还。
            let bytes = unsafe { mapped.bytes() };
            let length = mapped.len();
            dst[copied..copied + length].copy_from_slice(bytes);
            copied += length;
        }
        debug_assert_eq!(copied, dst.len());
        Ok(())
    }
}

/// 完整收权后移回原分段读入者，短结果结束真实失败而不授予尾部访问。
fn finish_folio_range(
    completion: BioCompletion,
    result: IoResult,
    len: usize,
) -> Result<FolioRead, BlockError> {
    if let Some(error) = result.error {
        return Err(error);
    }
    let mut read = completion
        .take_memory()
        .expect("completed folio payload was already taken")
        .pop()
        .expect("single folio payload is missing")
        .into_folio();
    if result.completed != len {
        // SAFETY: completion 已结束全部子请求并收权，短结果不授予未初始化尾部访问权。
        unsafe {
            read.complete(IoResult {
                completed: result.completed,
                error: Some(BlockError::ShortIo),
            })
        };
        return Err(BlockError::ShortIo);
    }
    Ok(read)
}

/// 略过被 admission 拒绝的 root，不为批次建立第二份拥有队列。
fn next_active(active: &[Option<ActiveBio<'_>>; MAX_BATCH], start: usize) -> usize {
    (start..MAX_BATCH)
        .find(|index| active[*index].is_some())
        .unwrap_or(MAX_BATCH)
}

/// 先取有限最小值再加一，完整边界掩码与地址零不能使边界额度溢出。
fn segment_limit(limits: QueueLimits, address: u64) -> usize {
    let before_end = limits.dma_boundary_mask - (address & limits.dma_boundary_mask);
    (before_end.min(u64::from(limits.max_segment_bytes) - 1) + 1) as usize
}

/// 将父原地址写入已预留 SG，最终向下对齐并裁掉尾段，不建立任何 payload bounce。
fn append_prefix(
    bio: &BioInner,
    cursor: &mut SegmentCursor,
    sg: &mut Vec<PayloadRange>,
    limits: QueueLimits,
    budget: usize,
) -> Result<usize, BlockError> {
    debug_assert!(limits.max_segments <= sg.capacity());
    let mut scan = *cursor;
    let mut bytes = 0;
    while bytes < budget {
        let source = bio.segment(scan.index).ok_or(BlockError::InvalidRange)?;
        let available = (source.length as usize)
            .checked_sub(scan.within)
            .ok_or(BlockError::InvalidRange)?;
        if available == 0 {
            return Err(BlockError::InvalidRange);
        }
        let address = source
            .physical
            .checked_add(scan.within)
            .ok_or(BlockError::InvalidRange)?;
        let remaining = available.min(budget - bytes);
        let mut length = 0;
        if let Some(last) = sg.last_mut() {
            let contiguous = last.direction == source.direction
                && last.physical.checked_add(last.length as usize) == Some(address);
            if contiguous {
                let room = segment_limit(limits, last.physical as u64)
                    .checked_sub(last.length as usize)
                    .ok_or(BlockError::InvalidRange)?;
                length = room.min(remaining);
                last.length += length as u32;
            }
        }
        if length == 0 {
            if sg.len() == limits.max_segments {
                break;
            }
            length = segment_limit(limits, address as u64).min(remaining);
            if length == 0 {
                return Err(BlockError::Unsupported);
            }
            sg.push(PayloadRange {
                physical: address,
                length: length as u32,
                direction: source.direction,
            });
        }
        scan.advance(bio, length)?;
        bytes += length;
    }
    let logical = limits.logical_block_size as usize;
    let prefix = bytes / logical * logical;
    trim_tail(sg, bytes - prefix);
    cursor.advance(bio, prefix)?;
    Ok(prefix)
}

/// 裁掉本轮追加范围的尾部，包括与旧尾段合并的部分，保留既有 parts 的完整字节。
fn trim_tail(sg: &mut Vec<PayloadRange>, mut bytes: usize) {
    while 0 < bytes {
        let last = sg.last_mut().expect("SG prefix tail is absent");
        let removed = bytes.min(last.length as usize);
        last.length -= removed as u32;
        bytes -= removed;
        if last.length == 0 {
            sg.pop();
        }
    }
}
