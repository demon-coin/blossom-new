//! 块 I/O 提交引擎、批次合并与散聚段拆分（Bio Submit / Merge / Split）模块。
//!
//! ## 模块职责与架构定位
//! 本模块连接上层逻辑块请求（`Bio` / 页缓存 `FolioRead` / 回写 `FolioWriteback`）与下层物理设备请求槽
//! （`Disk` 的 `PreparingSlot`）。它负责验证挂载占用权限（`FsClaim`）、以固定大小批次（最多 `MAX_BATCH = 8`）
//! 接纳逻辑父请求，并在满足设备硬件限额的前提下执行相邻请求合并（Merge）与超限拆分（Split），
//! 直接利用物理页原始地址构造散聚列表（Scatter-Gather List，简称 SG）提交给设备驱动，全程无需中间回弹缓冲区（Bounce Buffer）。
//! - **上游调用者**：文件系统挂载占用（`FsClaim`）、页缓存与元数据缓存（`crate::cache`）、文件读写与后台回写（`crate::fs`）
//!   以及无挂载视图的裸盘访问接口（`Disk::submit_bio` / `Disk::read_exact_at`）。
//! - **下游依赖**：逻辑块请求状态机（`crate::block::bio`）、物理磁盘请求槽与准入队列（`crate::block::disk`）、
//!   预留内存池（`crate::block::reserve`）以及物理页 DMA 映射原语（`crate::mm::dma`）。
//!
//! ## 核心设计与提交流水线
//!
//! ### 1. 批次准入、合并与拆分的父子映射关系
//! 提交路径按最多 `MAX_BATCH`（8 个）父 `Bio` 为一波进入准入（`admit_from`），生成 `ActiveBio` 生成者守卫；
//! 随后在循环中依次申请物理请求槽（`PreparingSlot`），按硬件队列限额（`QueueLimits`）填充子请求（Child）：
//!
//! ```text
//!   上层提交批次:  [ Parent Bio 0 (16 KiB) ]  [ Parent Bio 1 (4 KiB, 扇区紧邻) ]  [ Parent Bio 2 ... ]
//!                         │        │                       │
//!          硬件单槽上限拆分 │        └───────────┬───────────┘ 扇区连续且同向 -> 合并进同一请求槽
//!                         ▼                    ▼
//!   硬件请求槽:    ┌──────────────┐     ┌─────────────────────────┐
//!   PreparingSlot  │ Child Slot A │     │      Child Slot B       │
//!                  │  parts: [0]  │     │  parts: [0(尾), 1(全)]  │
//!                  │  SG: 原物理页 │     │  SG: 物理连续段自动拼接 │
//!                  └──────┬───────┘     └────────────┬────────────┘
//!                         └─────────────┬────────────┘
//!                                       ▼
//!                          slot.publish() 移交设备驱动队列
//!                          全部生成结束后 self.unplug() 触发硬件通知
//! ```
//!
//! ### 2. 两阶段计划提交与部分进度保留（`GenerationPlan`）
//! 在向 `PreparingSlot` 填充散聚段期间，`prepare_child` 只在栈上局部副本 `GenerationPlan` 中试探推进
//! 各父 `Bio` 的段游标（`SegmentCursor`），并在末尾按设备 `logical_block_size` 向下对齐裁剪未满一个逻辑块的尾段（`trim_tail`）。
//! 只有当 `slot.publish()` 成功将子请求送入硬件队列后，才将 `GenerationPlan` 记录的字节数与游标正式写回 `ActiveBio`。
//! 若中途因内存不足或队列停机出错，已发表的前置子请求继续正常由硬件完成，`ActiveBio::finish` 仅对未生成的剩余后缀报告错误偏移。
//!
//! ### 3. 页缓存批量读与后台回写通道
//! - **批量预读（`FsClaim::read_folio_batch`）**：将磁盘偏移连续的整页 `FolioRead` 合并进同一个多页父 `Bio`，
//!   凑满最多 8 个父 `Bio` 后异步批量提交，无需逐页同步阻塞。
//! - **受限回写批次（`FsClaim::writeback_batch`）**：配合预留池通道（`BioLane` / `BioLease`），在零堆分配条件下
//!   构造最多 8 个回写父 `Bio` 批量提交，随后逐个等待 DMA 彻底结束并回收预留的 `Vec` 与 `Arc` 控制块，供下一波次复用。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. **禁止持有当前任务引用跨睡眠点**：所有可能阻塞的入口均先执行 `drop(task::current_task())`，避免栈上残留任务强引用。
//! 2. **队列锁外完成可失败准备**：物理页分配、`Vec::try_reserve_exact`、DMA 映射校验以及 `PreparingSlot` 的散聚表拼接
//!    全部在 `Disk::queue` 自旋锁外执行，避免在关中断自旋锁内执行耗时计算或内存分配。
//! 3. **锁层级顺序**：`BioReserve::state`（预留池自旋锁，仅在租借/归还槽位瞬间短暂获取）与 `Disk::queue`（磁盘队列自旋锁，
//!    仅在 `admit_from`、`reserve_slot`、`publish`、`unplug` 瞬间获取）彼此独立、互不嵌套；`READ_BATCH` 测试诊断锁仅在
//!    `submit_batch` 返回后单独获取。

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

/// 父 `Bio` 内部 DMA 映射段数组的扫描游标。
///
/// 当一个大 `Bio` 被拆分为多个硬件子请求（`PreparingSlot`）时，跨子请求保留当前段下标与段内字节偏移，
/// 避免每次拆分都从第 0 个映射段重新线性扫描。该游标仅记录位置数值，不持有物理页所有权，也不解引用页内数据。
#[derive(Clone, Copy, Default)]
struct SegmentCursor {
    /// 当前扫描到的父 `Bio` 映射段下标（对应 `BioInner::segment(index)`）。
    index: usize,
    /// 当前映射段内已消费的字节偏移量（范围 `0..segment.length`）。
    within: usize,
}

impl SegmentCursor {
    /// 沿父 `Bio` 的映射段向前推进 `bytes` 字节；若越过段边界则自动切换到下一段，超出总范围则返回 `BlockError::InvalidRange`。
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

/// 单个硬件子请求槽（`PreparingSlot`）在正式发布前的段推进计划。
///
/// 在 `prepare_child` 阶段仅在栈上保存各参与父 `Bio` 的消费长度与更新后的游标；
/// 只有当 `slot.publish()` 成功将子请求提交到设备队列后，调用方才按本计划推进各 `ActiveBio` 的已生成偏移量。
struct GenerationPlan {
    /// 本子请求包含的各个分片（Part）在 `active` 批次数组中的父 `Bio` 下标，有效元素数为 `0..count`。
    indices: [usize; MAX_BATCH],
    /// 与 `indices` 一一对应的各父 `Bio` 在本子请求中实际消费的字节长度（单位：字节）。
    lengths: [usize; MAX_BATCH],
    /// 假设本子请求发布成功后，批次内各父 `Bio` 对应的最新段扫描游标快照。
    cursors: [SegmentCursor; MAX_BATCH],
    /// 本子请求实际合并纳入的父 `Bio` 分片数量（范围 `1..=MAX_BATCH`）。
    count: usize,
    /// 测试统计：本子请求中包含“非零起始偏移（即父 `Bio` 被拆分后的后续分片）”的分片计数。
    #[cfg(feature = "ktest")]
    splits: usize,
}

/// 内核测试（`ktest`）中单波批量读最多捕获的物理页与 DMA 地址样本数。
#[cfg(feature = "ktest")]
const READ_BATCH_CAPTURE_PAGES: usize = 32;

/// 内核测试（`ktest`）用于验证最近一波页缓存批量读（`read_folio_batch`）提交情况的诊断快照。
///
/// 快照仅复制物理地址与 DMA 地址的数值，不持有物理页、Inode 或 `FsClaim` 的引用计数，不影响正常资源回收生命周期。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct ReadBatchSnapshot {
    /// 全局单调递增（回绕）的读批次发布序号，用于区分先后多次批量读操作。
    pub sequence: u64,
    /// 本波批量读所属目标磁盘的唯一标识。
    pub disk: DiskIdentity,
    /// 本波中至少有 1 个字节被设备驱动实际接收的父 `Bio` 数量。
    pub bios: usize,
    /// 本波中被设备驱动实际接收超过 1 张物理页（即成功触发连续页合并）的父 `Bio` 数量。
    pub multi_page_bios: usize,
    /// 本波中被设备驱动实际完整接收的物理页总数。
    pub pages: usize,
    /// 本波中被设备驱动实际接收的总字节数（单位：字节）。
    pub accepted_bytes: usize,
    /// 本波首个父 `Bio` 的起始 512B 扇区号；若批次为空则为 `0`。
    pub first_sector: u64,
    /// `phys_addrs` 与 `dma_addrs` 数组中实际填入的有效页数（最多 `READ_BATCH_CAPTURE_PAGES` 页，且仅保留驱动已接收的完整页）。
    pub captured_pages: usize,
    /// 捕获的前 `captured_pages` 张已接收页面的 CPU 物理起始地址（单位：字节）。
    pub phys_addrs: [usize; READ_BATCH_CAPTURE_PAGES],
    /// 捕获的前 `captured_pages` 张已接收页面的设备侧 DMA 起始地址（单位：字节）。
    pub dma_addrs: [u64; READ_BATCH_CAPTURE_PAGES],
}

/// 全局最近一次页缓存批量读的诊断快照存储（受自旋锁保护，仅在 `ktest` 启用时编译）。
#[cfg(feature = "ktest")]
static READ_BATCH: SpinLock<Option<ReadBatchSnapshot>> = SpinLock::new(None);

/// 单波批量读提交前后的观测辅助收集器。
///
/// 在 `Bio` 提交给设备之前预先提取物理页与 DMA 地址，并持有各父请求的 `BioCompletion` 观察句柄；
/// 提交返回后通过 `BioCompletion` 查询实际被驱动接收的字节数，从而避免与设备中断完成路径移走 `BioMemory` 发生数据竞争。
#[cfg(feature = "ktest")]
struct ReadBatchObservation {
    /// 待筛选并发布的读批次快照草稿（初始保存提交前采集的候选页地址）。
    snapshot: ReadBatchSnapshot,
    /// 本波各槽位父 `Bio` 的完成观察句柄及其提交时的总页数 `(completion, pages)`。
    parents: [Option<(BioCompletion, usize)>; MAX_BATCH],
}

#[cfg(feature = "ktest")]
impl ReadBatchObservation {
    /// 在父 `Bio` 尚未提交给设备驱动前采集最多 32 张候选页的物理地址与 DMA 地址，并克隆各父请求的 `BioCompletion` 句柄。
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

    /// 在 `submit_batch` 返回后查询各父请求实际被设备驱动接收的字节数，剔除未被接收的页面并将最终统计快照写入全局 `READ_BATCH`。
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
    /// 在当前文件系统挂载占用（`FsClaim`）下提交单个块请求 `Bio`。
    ///
    /// 写请求（`Write`）仅在读写挂载（`AccessMode::ReadWrite`）下被准入；读请求（`Read`）与设备支持的刷盘（`Flush`）
    /// 可在只读挂载下执行。通过准入的请求会独立持有一份 `FsClaim` 克隆，直到子请求生成、设备 DMA 访问权收回、
    /// 完成回调以及请求槽队列计数清理全部执行完毕后才释放。
    pub fn submit_bio(&self, bio: Bio) {
        self.disk().submit_bio_from(bio, Some(self));
    }

    /// 在当前挂载占用下批量提交一组 `Bio`，复用与单请求相同的准入校验、相邻扇区合并与超限拆分流水线。
    pub fn submit_batch(&self, bios: impl IntoIterator<Item = Bio>) {
        self.disk().submit_batch_from(bios, Some(self));
    }

    /// 按卷内相对字节偏移 `offset` 同步读取精确长度的数据到 `dst`（调用者位于普通可睡眠任务上下文）。
    ///
    /// 请求在在途期间持续持有 `FsClaim` 直到硬件请求槽彻底结束并收回 DMA 访问权；仅当整段范围完整读取成功时才将数据拷贝至 `dst`。
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

    /// 将卷内相对字节偏移 `byte` 换算为磁盘绝对偏移，使用传入的 `memory` 容器同步将整段 `FolioRead` 读入物理页。
    ///
    /// 若卷边界校验失败则立即在 `read` 上发布错误并返回；提交后等待设备 DMA 彻底完成、收回页面访问权并校验无短读（`ShortIo`）。
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

    /// 按卷内相对字节偏移 `byte` 同步读取数据至物理页的指定子区间 `[offset, offset + len)`。
    ///
    /// 适用于文件块大小小于 4KiB 时的页内分段填充：本次子区间 DMA 彻底结束并收回访问权后，将仍持有页内容锁的 `FolioRead` 返还调用方继续填充其余区间。
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

    /// 批量合并并异步提交一组整页读入请求 `ranges`（元素为 `(卷内字节偏移, FolioRead)`），不在本函数内阻塞等待设备 I/O 完成。
    ///
    /// 每波最多构造 `MAX_BATCH`（8 个）父 `Bio`；每个父 `Bio` 自动合并磁盘字节偏移连续的多个整页 `FolioRead`。
    /// 若中途某页映射或内存准备失败，仍先将当前波次已成功准备的父 `Bio` 提交给设备，随后为剩余未提交的所有 `FolioRead` 发布该准备错误并返回。
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
            // Step 1: 从待处理队列中依次提取连续整页区间，最多构造 MAX_BATCH 个父 Bio。
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

            // Step 2: 批量提交本波已就绪的父 Bio；即使后续页面准备出错，也不丢弃本波已构造的前缀请求。
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

    /// 从 `pending` 头部扫描并消费一段磁盘字节相邻的整页 `FolioRead`，打包构造为单个多页读 `Bio`。
    ///
    /// 若扫描前缀中第 `k`（`k > 0`）张页在建立 `MappedPages` 时失败，会立即结束该失败页的读凭证，
    /// 并用前 `k` 张已成功映射的页面构造合法 `Bio` 连同 `Some(error)` 一并返回，供上层先提交已就绪前缀。
    fn prepare_read_bio(
        &self,
        pending: &mut alloc::vec::Drain<'_, (u64, FolioRead)>,
    ) -> Result<(Bio, Option<BlockError>), BlockError> {
        let byte_offset = pending.as_slice()[0].0;
        if byte_offset % SECTOR_BYTES != 0 {
            return Err(BlockError::Misaligned);
        }
        // Step 1: 预览并统计从队首起磁盘偏移严格连续、且均为完整 4KiB 页面的最大可合并页数。
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

        // Step 2: 预分配映射段容器，并将连续前缀中的 FolioRead 逐个转换为 DMA 映射页。
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

        // Step 3: 将卷内偏移换算为物理扇区号并构造多页读 Bio。
        // SAFETY:
        // 1. `memory` 中的每个 `MappedPages` 均由独占页内容锁（`CONTENT_LOCKED`）且保活所属 Inode 的 `FolioRead` 构造，调用前无任何活跃的 CPU 切片借用；
        // 2. 操作类型为 `BlkOperation::Read` 且 `end_io` 为 `None`，不要求物理页初始数据有效，也不存在回调并发访问；
        // 3. `Bio` 会在所有硬件子请求与生成者全部执行完毕、收回物理页 DMA 访问权后才完成各页的 `FolioRead` 凭证。
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

    /// 读取全局最近一次批量读诊断快照；仅当该快照属于当前 `FsClaim` 所在的同一块物理磁盘时才返回。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_read_batch_snapshot(&self) -> Option<ReadBatchSnapshot> {
        let snapshot = *READ_BATCH.lock();
        snapshot.filter(|snapshot| snapshot.disk == self.disk().identity())
    }

    /// 分波次批量提交并同步等待一组脏页回写凭证 `ranges`（元素为 `(磁盘绝对字节偏移, FolioWriteback)`）。
    ///
    /// 若传入预留池通道 `lane`（`BioLane`），则使用零堆分配的预留槽位构造每波最多 `lane.len()` 个父 `Bio`，
    /// 相邻磁盘字节范围自动合并进同一个父 `Bio`（受单槽最多 8 个映射段限制）。每波调用 `submit_batch` 后，
    /// 同步等待本波全部父 `Bio` 完成 DMA 并取回释放 `BioMemory` 与 `BioCompletion`，确保预留池控制块与载荷块归还后才进入下一波复用。
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
            // Step 1: 获取本波首个可用槽位（允许在尚未持有任何未提交 Bio 时睡眠等待预留槽归还）。
            let width = lane.map_or(MAX_BATCH, BioLane::len);
            let (first_slot, mut first_lease) = match lane {
                Some(lane) => {
                    let (slot, lease) = lane.lease_available(&stopped)?;
                    (slot, Some(lease))
                }
                None => (0, None),
            };
            // Step 2: 轮询通道内其余槽位（后续槽位采用非阻塞 try_lease，避免携已构造的未提交 Bio 睡眠死锁）。
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

            // Step 3: 批量提交本波全部回写父 Bio，随后逐一等待硬件完成并主动回收 memory/completion 以归还预留池块。
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

    /// 从 `pending` 头部消费一段磁盘绝对字节偏移连续的 `FolioWriteback`（不超过分配器的 `mapping_limit` 上限），构造单个回写 `Bio`。
    ///
    /// 若首项之后某张页的 DMA 映射校验失败，该失败凭证会被立即析构回滚，此前已成功放入 `memory` 的连续前缀仍会正常构造为 `Bio` 返回提交。
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

        // SAFETY:
        // 1. 每个 `FolioWriteback` 均源自已完成初始化的缓存页，且在创建回写凭证时已完成页表写权限清理与缓存同步，不存在导出给外部的可变切片；
        // 2. 操作类型为 `BlkOperation::Write` 且无完成回调，`Bio` 保证在生成者及所有拆分子请求的设备 DMA 访问权全部收回后，才逐页结算回写完成状态。
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

    /// 使用调用方在业务锁外预先准备的空 `BioMemory` 容器向磁盘提交同步 `Flush` 请求，等待设备刷盘彻底完成并回收容器内存后返回。
    pub(crate) fn flush(&self, memory: BioMemory) -> Result<(), BlockError> {
        // SAFETY:
        // 1. `BlkOperation::Flush` 不携带任何数据映射段（`memory` 为空），不存在物理页初始化要求或 CPU/DMA 内存别名；
        // 2. `end_io` 为 `None`，无回调并发访问风险。
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

/// 遍历尚未提交给设备的剩余 `FolioRead` 队列，逐一以具体准备错误 `error` 结束读入等待（避免直接 Drop 被统一替换为取消错误）。
fn fail_pending_reads(pending: &mut alloc::vec::Drain<'_, (u64, FolioRead)>, error: BlockError) {
    for (_, read) in pending {
        fail_read(read, error);
    }
}

/// 在尚未提交给设备驱动的 `FolioRead` 上直接发布准备阶段错误 `error`（完成字节数为 `0`）。
fn fail_read(mut read: FolioRead, error: BlockError) {
    // SAFETY:
    // 1. 该 `read` 凭证从未移交块设备或建立活跃 DMA 映射，且调用方未保留任何页内字节借用；
    // 2. 传入 `completed: 0` 与 `Some(error)` 不会标记页面任何字节已初始化，等待该页的任务将收到准确的错误码。
    unsafe {
        read.complete(IoResult {
            completed: 0,
            error: Some(error),
        });
    }
}

impl Disk {
    /// 在无文件系统挂载占用的裸盘模式下提交单个 `Bio`（存在活跃文件系统占用期间拒绝裸盘写请求；完成结果由调用方持有的 `BioCompletion` 观察）。
    pub(crate) fn submit_bio(self: &DiskRef, bio: Bio) {
        self.submit_bio_from(bio, None);
    }

    /// 校验单个 `Bio` 的准入权限并同步将其拆分生成硬件子请求；若准入被拒则先刷出队列中已就绪的前置请求再拒绝当前 `Bio`。
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

    /// 在无文件系统挂载占用的裸盘模式下批量提交一组 `Bio`（按每波最多 `MAX_BATCH` 个分组处理，支持同波次内相邻读写请求合并）。
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

    /// 按每波最多 `MAX_BATCH`（8 个）父请求对迭代器执行准入校验与硬件子请求生成；准入失败的单个请求立即就地拒绝，不影响同波次其他合法请求。
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

    /// 为当前批次所有已通过准入的 `ActiveBio` 生成硬件子请求，调用 `unplug` 通知驱动，并结算每个父请求的生成者生命周期。
    ///
    /// 若子请求生成中途发生错误，此前已成功发布到硬件队列的子请求继续正常执行，`finish` 仅在尚未生成完毕的父请求上记录错误发生的起始偏移。
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

    /// 按顺序逐个申请硬件请求槽（`PreparingSlot`），在锁外填充散聚列表并发布子请求，直到批次内所有父 `Bio` 全部生成完毕。
    fn generate_children(
        &self,
        active: &mut [Option<ActiveBio<'_>>; MAX_BATCH],
        generated: &mut [bool; MAX_BATCH],
    ) -> Result<(), BlockError> {
        let mut cursors = [SegmentCursor::default(); MAX_BATCH];
        let mut next = next_active(active, 0);
        while next < MAX_BATCH {
            // Step 1: 申请空闲硬件请求槽（若槽位暂满则在队列锁外睡眠等待），并在锁外构造子请求散聚表计划。
            let mut slot = self.reserve_slot()?;
            let plan = self.prepare_child(&mut slot, active, cursors, next)?;
            // Step 2: 将填充完毕的硬件请求槽正式发布到设备队列；发布成功后才推进各父请求的已生成字节偏移与段游标。
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

    /// 在队列锁外向单个硬件请求槽 `slot` 填充子请求，尽可能合并批次中扇区紧邻且方向相同的后续父 `Bio`，返回本次填充的 `GenerationPlan`。
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
        // Step 1: Flush 请求不含数据载荷且不可与其他请求合并，单独占用一个硬件请求槽。
        if operation == BlkOperation::Flush {
            slot.push_part(Arc::clone(&root.bio), 0, 0)?;
            slot.configure(operation, 0, 0)?;
            plan.indices[0] = first;
            plan.count = 1;
            return Ok(plan);
        }

        // Step 2: 读写请求从首个活跃父 Bio 开始，在硬件单槽字节上限（max_bytes）、段数上限（max_segments）与分片数上限内顺序扫描可合并的相邻请求。
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
            // 若任一请求带有可睡眠 Task 完成回调，其回调可能等待相邻请求，禁止将其与其他父请求合并进同一请求槽以免互相阻塞完成。
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
        // Step 3: 写入操作码、起始扇区与总字节数，完成硬件请求槽的协议头配置。
        debug_assert!(0 < bytes && !slot.sg().is_empty());
        slot.configure(operation, sector, bytes)?;
        Ok(plan)
    }

    /// 在无文件系统挂载占用的裸盘模式下，按磁盘绝对字节偏移 `byte_offset` 将整张 `FolioRead` 同步读入物理页，等待 DMA 完成并校验无短读。
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

    /// 在队列锁外校验偏移对齐、建立物理页或页内子区间 `range` 的 DMA 映射并提交读 `Bio`，同步阻塞等待硬件完成后返回观察句柄与 I/O 结果。
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
            // SAFETY:
            // 1. `read` 尚未封装为 DMA 映射段，也未提交给设备，调用期间无任何活跃的 CPU 页内切片借用；
            // 2. 传入 `completed: 0` 与准备错误不会暴露未初始化字节，所有等待该页就绪的任务将收到同一错误。
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
                // SAFETY:
                // 1. DMA 映射校验失败并原样退回 `read` 所有权，设备从未接触该物理页；
                // 2. 以 `completed: 0` 结束读凭证不会发布任何未初始化的页内容。
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
        // SAFETY:
        // 1. `mapped` 独占持有页内容锁（`CONTENT_LOCKED`）并保活底层物理页与所属对象，不存在并发 CPU 切片借用；
        // 2. 操作类型为 `BlkOperation::Read` 且无完成回调，`Bio` 保证在所有子请求完成并收回物理页 DMA 访问权后才结算读凭证。
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

    /// 在普通可睡眠任务上下文中，使用临时分配的零初始化物理页从磁盘绝对字节偏移 `offset` 同步读取数据到 `dst`。
    ///
    /// 非空读取范围的起始偏移与长度均须对齐设备 `logical_block_size`；仅在全部字节读取成功后才将页面数据拷贝至调用方缓冲区 `dst`。
    pub fn read_exact_at(self: &DiskRef, offset: u64, dst: &mut [u8]) -> Result<(), BlockError> {
        drop(task::current_task());
        let result = self.read_with_pages(offset, dst, None);
        self.unplug();
        result
    }

    /// 在队列锁外校验设备容量与逻辑块对齐、逐页分配零页提交读 `Bio`，并在收回 DMA 访问权后将完整读取的数据复制到 `dst`。
    fn read_with_pages(
        self: &DiskRef,
        offset: u64,
        dst: &mut [u8],
        claim: Option<&FsClaim>,
    ) -> Result<(), BlockError> {
        // Step 1: 校验磁盘容量上界、空切片快速返回以及设备逻辑块大小（logical_block_size）对齐。
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

        // Step 2: 在队列锁外按 4KiB 粒度申请清零物理页并建立 DeviceToDriver 方向的 DMA 映射列表。
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

        // Step 3: 提交读 Bio 并等待硬件完成，收回临时物理页访问权后校验全长无误再拷贝至 dst。
        // SAFETY:
        // 1. `memory` 中每个映射均独占刚分配且已整页清零的 `OwnedPages`，移入 `Bio` 前无任何 CPU 切片别名；
        // 2. 操作类型为 `BlkOperation::Read` 且无回调，在 `completion.wait()` 与 `take_memory()` 返回前 CPU 不会读取这些页面。
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
            // SAFETY:
            // 1. 底层物理页在分配时已整页清零，且上方已校验无错误且 `result.completed == total`；
            // 2. `completion.wait()` 与 `completion.take_memory()` 已确保所有子请求结束 DMA 并收回 CPU 访问权，`memory` 为当前栈帧独占持有，无并发写入。
            let bytes = unsafe { mapped.bytes() };
            let length = mapped.len();
            dst[copied..copied + length].copy_from_slice(bytes);
            copied += length;
        }
        debug_assert_eq!(copied, dst.len());
        Ok(())
    }
}

/// 在页内子区间读 `Bio` 完成后收回物理页访问权并取回 `FolioRead`；若发生短读（`ShortIo`）则立即在读凭证上登记失败并返回错误。
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
        // SAFETY:
        // 1. 上方 `take_memory()` 与 `into_folio()` 已确认全部硬件子请求执行完毕并收回物理页 DMA 访问权；
        // 2. 传入 `Some(BlockError::ShortIo)` 终止读凭证，不会将未读满的尾部区间标记为已初始化可读。
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

/// 从批次槽位下标 `start` 起向后查找下一个已通过准入的 `ActiveBio` 下标；若后续全为空则返回 `MAX_BATCH`。
fn next_active(active: &[Option<ActiveBio<'_>>; MAX_BATCH], start: usize) -> usize {
    (start..MAX_BATCH)
        .find(|index| active[*index].is_some())
        .unwrap_or(MAX_BATCH)
}

/// 计算从物理地址 `address` 开始的单个散聚段最大允许字节数（同时受限于设备 `dma_boundary_mask` 跨界掩码与 `max_segment_bytes` 上限）。
///
/// 先在减一后的无符号域取最小值再加一，防止 `dma_boundary_mask == u64::MAX` 且 `address == 0` 时发生 `u64` 加法溢出。
fn segment_limit(limits: QueueLimits, address: u64) -> usize {
    let before_end = limits.dma_boundary_mask - (address & limits.dma_boundary_mask);
    (before_end.min(u64::from(limits.max_segment_bytes) - 1) + 1) as usize
}

/// 在不超过 `budget` 字节与硬件段数上限的前提下，将父 `Bio` 从 `cursor` 起的物理段追加（或与尾段合并）到散聚表 `sg` 中，
/// 最后按设备 `logical_block_size` 向下取整裁剪尾部并推进 `cursor`，返回实际纳入的对齐字节数。
fn append_prefix(
    bio: &BioInner,
    cursor: &mut SegmentCursor,
    sg: &mut Vec<PayloadRange>,
    limits: QueueLimits,
    budget: usize,
) -> Result<usize, BlockError> {
    debug_assert!(limits.max_segments <= sg.capacity());
    // Step 1: 沿局部游标扫描父 Bio 的物理段，物理地址紧邻且未跨 DMA 边界时直接原地扩展尾段，否则压入新散聚段。
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

    // Step 2: 将累计字节数向下对齐到设备逻辑块大小，裁掉散聚表末尾不足一个逻辑块的零头，并正式推进父游标。
    let logical = limits.logical_block_size as usize;
    let prefix = bytes / logical * logical;
    trim_tail(sg, bytes - prefix);
    cursor.advance(bio, prefix)?;
    Ok(prefix)
}

/// 从散聚表 `sg` 末尾反向回退并截断指定的 `bytes` 字节（包含刚才合并到既有尾段上的部分），确保先前已纳入分片的字节范围保持完整。
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
