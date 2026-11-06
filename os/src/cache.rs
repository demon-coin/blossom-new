//! 页缓存子系统（Page Cache）：统一管理磁盘元数据缓存（`MetadataCache`）与文件页缓存（`FileMapping`）。
//!
//! ## 模块职责与架构定位
//! 本模块位于虚拟文件系统（`crate::fs`）、内存管理（`crate::mm`）与块设备层（`crate::block`）之间：
//! - **向上（VFS / Ext4 / 进程地址空间）**：为文件读写、`mmap` 内存映射提供以 4KiB 物理页（`FolioRef`）
//!   为单位的稀疏文件缓存（`FileMapping`）；为文件系统超级块、块组描述符、Inode 表和目录/Extent 树
//!   提供支持 1KiB / 2KiB / 4KiB 块粒度的元数据窗口视图（`MetadataCache` 与 `BufferRef`）。
//! - **向下（块设备层与物理内存回收）**：通过 `FsClaim` 提交块设备读写 I/O，协调回写错误序列
//!   （`WritebackError`），并配合 `crate::mm::reclaim` 扫描全局文件页 LRU、卸载用户态页表映射及回收干净页。
//!
//! ## 核心设计与关键布局
//!
//! ### 1. 物理盘共享元数据页域与分区视图（`MetadataPages` vs `MetadataCache`）
//! 同一块物理磁盘（`Disk`）仅维护唯一的一套 `MetadataPages`（含容量受限的 `FolioIndex` 和
//! 预留内存池 `BioReserve`）。磁盘上的不同分区或卷各自持有独立的 `MetadataCache` 门面（Facade）：
//!
//! ```text
//!   MetadataCache (Partition A)          MetadataCache (Partition B)
//!   - claim: FsClaim (Range A)           - claim: FsClaim (Range B)
//!   - selected: SpinLock<Vec<u64>>       - selected: SpinLock<Vec<u64>>
//!   - wb_error: WritebackError           - wb_error: WritebackError
//!                 \                                 /
//!                  +---------------+---------------+
//!                                  | Arc<MetadataPages>
//!                                  v
//!                     MetadataPages (Per Physical Disk)
//!                     - index: FolioIndex (key = disk byte offset)
//!                     - reserve: Option<Arc<BioReserve>>
//! ```
//! 当分区起止扇区未按 4KiB 页对齐时，相邻两个分区的边界可能共享同一个 4KiB 物理页的
//! 不同扇区（通过 `MetadataCache::page_mask` 计算 8 位扇区掩码隔离各自的有效/脏/回写扇区），
//! 且跨页的元数据块通过 `BufferRef { folio, tail: Some(...) }` 透明拼接。
//!
//! ### 2. 回写错误状态字（`WritebackError`，参考经典的 `errseq_t` 32 位位图算法）
//! 为了在无需动态分配错误队列的前提下向多个独立的文件描述符（OFD）或同步调用者准确报告后台回写错误，
//! `WritebackError::value` 将错误码、观察标记与发生计数器编码在同一个 `u32` 中：
//!
//! ```text
//!  31                                   13    12 11               0
//! +---------------------------------------+-----+------------------+
//! |         Error Counter (19 bits)       |SEEN |  Errno (12 bits) |
//! +---------------------------------------+-----+------------------+
//!                                            ^
//!                                   ERRSEQ_SHIFT = 12
//! ```
//! - **Bits `[11:0]`（`ERRSEQ_ERRNO`）**：最近一次回写失败对应的 POSIX `Errno` 值。
//! - **Bit `12`（`ERRSEQ_SEEN`）**：标记当前记录的错误是否已被至少一个观察者读取过。
//!   - 若新观察者打开文件时 `SEEN == 0`（存在尚未被任何人观测到的历史错误），`sample()` 返回 `0`，
//!     确保后续首次 `fsync` 仍能捕获该错误；
//!   - 仅当 `SEEN == 1` 时，新写入的错误才会将高 19 位计数器加 1（`+ ERRSEQ_INCREMENT`），
//!     避免未被观测的连续错误冲掉游标差值。
//!
//! ## 锁获取顺序与并发契约
//! 1. **文件截断与回收锁顺序**：
//!    `Inode` 写互斥锁 ➔ `Inode` 失效睡眠锁（`invalidate`） ➔ `FileMapping::writeback`（睡眠锁）
//!    ➔ `Folio::lock_content`（页内容睡眠锁） ➔ `FileMapping::ranges`（反向映射自旋锁）
//!    ➔ `PteDomain` 页表锁。
//! 2. **索引与 LRU 锁顺序**：
//!    `FolioIndex::entries`（自旋锁） ➔ `index::FILE_LRU`（全局 LRU 自旋锁） ➔ `Folio` 内部状态自旋锁。
//! 3. **元数据预留池初始化锁顺序**：
//!    `MetadataPages::reserve_init`（睡眠读写锁） ➔ `FolioIndex::entries`（自旋锁） / `MetadataPages::reserve`（自旋锁）。
//! 4. **跨页元数据块内容锁顺序**：
//!    `BufferRef` 访问跨越两个相邻物理页的块时，严格按照磁盘字节偏移升序依次获取首页（`self.folio`）
//!    与尾页（`tail`）的 `lock_content()`，杜绝 AB-BA 死锁。

mod index;

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::convert::TryFrom;
use core::ops::{Deref, Range};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::block::{
    AccessMode, BioAllocator, BioMemory, BioReserve, BlockError, DiskRef, FsClaim,
    MetadataIoContext, SECTOR_BYTES,
};
use crate::fs::ext4::Ext4Volume;
use crate::fs::{FsError, InodeRef};
use crate::mm::folio::FolioContent;
use crate::mm::reclaim::{ReclaimAttempt, ReclaimStats};
use crate::mm::space::{sync_all, PteDomain, ResidentPages};
use crate::mm::user_table::RetiredTables;
use crate::mm::{FolioRead, FolioRef, PAGE_SIZE};
use crate::sync::{SleepMutex, SleepMutexGuard, SleepRwLock, SpinLock, SpinLockGuard};
use crate::syscall::Errno;
use crate::task::{ReclaimMode, ReclaimScope};
use index::{FileRole, IndexAllocator, Node, NodeRef, RetiredNode, Tree};

/// 页缓存范围扫描时用于筛选节点的标记条件。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FolioMark {
    /// 筛选包含脏扇区（`dirty_sectors() != 0`）的物理页。
    Dirty,
    /// 筛选正在进行块设备回写（`writeback_sectors() != 0`）的物理页。
    Writeback,
    /// 筛选在当前同步批次中已被 `mark_to_write_range` 标记为待写回的节点。
    ToWrite,
}

/// `WritebackError` 状态字中低位 `Errno` 字段的位宽（12 位）。
const ERRSEQ_SHIFT: u32 = 12;
/// 提取 `WritebackError` 低 12 位 `Errno` 的掩码（`0xfff`）。
const ERRSEQ_ERRNO: u32 = (1 << ERRSEQ_SHIFT) - 1;
/// 标记当前错误已被至少一个调用方观测过的标志位（第 12 位，`0x1000`）。
const ERRSEQ_SEEN: u32 = 1 << ERRSEQ_SHIFT;
/// 当已被观测过的错误之后再次发生新错误时，向高 19 位计数器累加的步进值（第 13 位，`0x2000`）。
const ERRSEQ_INCREMENT: u32 = 1 << (ERRSEQ_SHIFT + 1);

/// 文件系统与元数据缓存支持的 1KiB 逻辑块大小（字节）。
pub(crate) const FILE_BLOCK_BYTES_1K: usize = 1024;
/// 文件系统与元数据缓存支持的 2KiB 逻辑块大小（字节）。
pub(crate) const FILE_BLOCK_BYTES_2K: usize = 2 * FILE_BLOCK_BYTES_1K;
/// 文件系统与元数据缓存支持的 4KiB 逻辑块大小（字节）。
pub(crate) const FILE_BLOCK_BYTES_4K: usize = 4 * FILE_BLOCK_BYTES_1K;

/// 采用 32 位 `errseq_t` 位图算法记录异步回写错误的状态机，同时保留最后一次具体的 `FsError`。
struct WritebackError {
    /// 编码了低 12 位 `Errno`、第 12 位 `SEEN` 标志及高 19 位错误计数器的状态字（0 表示从未出错）。
    value: u32,
    /// 最近一次记录的具体文件系统错误枚举，供 `check_and_advance` 精确返回。
    error: Option<FsError>,
}

impl WritebackError {
    /// 创建一个初始无错误记录的 `WritebackError` 状态机。
    const fn new() -> Self {
        Self {
            value: 0,
            error: None,
        }
    }

    /// 为新创建的观察者（如新打开的文件描述符）采样初始游标值。
    ///
    /// 若当前存在尚未被任何观察者读取过的错误（`ERRSEQ_SEEN == 0`），则返回 `0`，
    /// 使该新观察者在首次调用 `check_and_advance` 时也能观测到这次未见错误。
    fn sample(&self) -> u32 {
        if self.value & ERRSEQ_SEEN == 0 {
            0
        } else {
            self.value
        }
    }

    /// 记录一次新的回写错误：更新低 12 位 `Errno` 并保存具体 `error`；
    /// 仅当上一次错误已被观察过（`ERRSEQ_SEEN != 0`）时才递增高位错误计数器并清除 `SEEN` 位。
    fn record(&mut self, error: FsError) {
        let mut next = (self.value & !(ERRSEQ_ERRNO | ERRSEQ_SEEN)) | writeback_errno(error);
        if self.value & ERRSEQ_SEEN != 0 {
            next = next.wrapping_add(ERRSEQ_INCREMENT);
        }
        self.value = next;
        self.error = Some(error);
    }

    /// 检查自观察者上次游标 `*since` 以来是否发生了新的回写错误：
    /// 若状态字发生变化，则置位 `ERRSEQ_SEEN`、将 `*since` 推进到最新状态字并返回最近一次的 `FsError`。
    fn check_and_advance(&mut self, since: &mut u32) -> Result<(), FsError> {
        if self.value == *since {
            return Ok(());
        }
        self.value |= ERRSEQ_SEEN;
        *since = self.value;
        Err(self.error.expect("writeback errseq has no error"))
    }
}

/// 将内核内部文件系统错误 `FsError` 映射为 12 位以内的标准 POSIX `Errno` 数值。
fn writeback_errno(error: FsError) -> u32 {
    match error {
        FsError::NoMemory | FsError::Io(BlockError::NoMemory) => Errno::NoMemory as u32,
        FsError::NoSpace => Errno::NoSpace as u32,
        FsError::TextBusy => Errno::TextBusy as u32,
        FsError::FileTooLarge => Errno::FileTooLarge as u32,
        FsError::Unsupported | FsError::Io(BlockError::Unsupported) => Errno::Unsupported as u32,
        FsError::NotFound | FsError::MissingRoot => Errno::NotFound as u32,
        FsError::Exists => Errno::Exists as u32,
        FsError::NotEmpty => Errno::NotEmpty as u32,
        FsError::CrossDevice => Errno::CrossDevice as u32,
        FsError::TooManyLinks => Errno::TooManyLinks as u32,
        FsError::NameTooLong => Errno::NameTooLong as u32,
        FsError::NotPermitted => Errno::NotPermitted as u32,
        FsError::NotDirectory => Errno::NotDirectory as u32,
        FsError::InvalidName
        | FsError::InvalidOperation
        | FsError::Io(BlockError::InvalidRange | BlockError::Misaligned) => {
            Errno::InvalidArgument as u32
        }
        FsError::Busy | FsError::Io(BlockError::Busy) => Errno::Busy as u32,
        FsError::TooManySymlinks => Errno::TooManySymlinks as u32,
        FsError::BadFd => Errno::BadFd as u32,
        FsError::TooManyFiles => Errno::TooManyFiles as u32,
        FsError::ReadOnly | FsError::Io(BlockError::ReadOnly) => Errno::ReadOnly as u32,
        FsError::IsDirectory => Errno::IsDirectory as u32,
        FsError::IllegalSeek => Errno::IllegalSeek as u32,
        FsError::NotTty => Errno::NotTty as u32,
        FsError::NoDevice => Errno::NoDevice as u32,
        FsError::NoDeviceAddress => Errno::NoDeviceAddress as u32,
        FsError::NoProcess | FsError::NoSuchRoute => Errno::NoProcess as u32,
        FsError::DeviceIo => Errno::Io as u32,
        FsError::AddressFamilyNotSupported => Errno::AddressFamilyNotSupported as u32,
        FsError::AddressInUse => Errno::AddressInUse as u32,
        FsError::AddressNotAvailable => Errno::AddressNotAvailable as u32,
        FsError::NetworkUnreachable => Errno::NetworkUnreachable as u32,
        FsError::ConnectionReset => Errno::ConnectionReset as u32,
        FsError::AlreadyConnected => Errno::AlreadyConnected as u32,
        FsError::NotConnected => Errno::NotConnected as u32,
        FsError::ConnectionRefused => Errno::ConnectionRefused as u32,
        FsError::AlreadyInProgress => Errno::AlreadyInProgress as u32,
        FsError::InProgress => Errno::InProgress as u32,
        FsError::MessageTooLarge => Errno::MessageTooLarge as u32,
        FsError::NotSocket => Errno::NotSocket as u32,
        FsError::DestinationRequired => Errno::DestinationRequired as u32,
        FsError::WouldBlock => Errno::Again as u32,
        FsError::BrokenPipe => Errno::BrokenPipe as u32,
        FsError::Interrupted | FsError::InterruptedNoRestart => Errno::Interrupted as u32,
        FsError::BadAddress => Errno::BadAddress as u32,
        FsError::Permission => Errno::Permission as u32,
        FsError::Corrupt | FsError::Checksum | FsError::LoadAborted | FsError::Io(_) => {
            Errno::Io as u32
        }
    }
}

/// 测试目标：验证 `WritebackError` 的未观测错误采样、多观察者独立游标推进、错误覆盖以及 19 位计数器回绕逻辑。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_wb_error_protocol() -> bool {
    // Step 1: 初始无错误时采样游标为 0，检查推进应返回 Ok(())。
    let mut error = WritebackError::new();
    let mut first = error.sample();
    if error.check_and_advance(&mut first) != Ok(()) {
        return false;
    }
    // Step 2: 连续发生两次未观测错误时，后一次覆盖前一次的 Errno，且新观察者采样仍从 0 开始以捕获未见错误。
    error.record(FsError::NoMemory);
    error.record(FsError::NoSpace);
    if error.value != 28 || error.sample() != 0 {
        return false;
    }
    let mut second = error.sample();
    if error.check_and_advance(&mut first) != Err(FsError::NoSpace)
        || error.check_and_advance(&mut second) != Err(FsError::NoSpace)
        || error.check_and_advance(&mut first) != Ok(())
        || first != second
        || error.sample() != first
    {
        return false;
    }
    // Step 3: 已被观察过的错误之后再次发生新错误时，高 19 位计数器加 1（ERRSEQ_INCREMENT）。
    error.record(FsError::Io(BlockError::ShortIo));
    if error.value != ERRSEQ_INCREMENT | 5
        || error.sample() != 0
        || error.check_and_advance(&mut first) != Err(FsError::Io(BlockError::ShortIo))
    {
        return false;
    }
    // Step 4: 将高 19 位计数器直接设置为全 1 最大值且置位 SEEN，验证再次调用 `record` 时计数器回绕至 0 仍能正确报告新错误。
    error.value = !(ERRSEQ_ERRNO | ERRSEQ_SEEN) | ERRSEQ_SEEN | 5;
    error.record(FsError::NoMemory);
    error.value == 12
        && error.sample() == 0
        && error.check_and_advance(&mut second) == Err(FsError::NoMemory)
        && error.check_and_advance(&mut second) == Ok(())
}

/// 管理单个文件或物理磁盘元数据页的稀疏索引封装。
pub(crate) struct FolioIndex {
    /// 受自旋锁保护的底层 AVL 平衡二叉搜索树。
    entries: SpinLock<Tree>,
    /// 索引允许驻留的最大物理页数上限（文件索引为 `None` 表示不设固定上限；元数据索引为 `Some(capacity)`）。
    capacity: Option<usize>,
    /// 标记该索引是否用于管理磁盘元数据页（决定页分配方式与是否在未命中时立即启动整页读取）。
    metadata: bool,
}

impl FolioIndex {
    /// 创建一个新的稀疏页索引。
    ///
    /// - `capacity`：可选的驻留节点数量上限（元数据缓存传入固定预算，文件缓存传入 `None`）。
    /// - `metadata`：为 `true` 表示元数据索引，为 `false` 表示文件或裸设备页索引。
    pub(crate) const fn new(capacity: Option<usize>, metadata: bool) -> Self {
        Self {
            entries: SpinLock::new(Tree::new()),
            capacity,
            metadata,
        }
    }

    /// 当索引节点数量达到 `capacity` 上限时，按升序查找并摘除一个无外部引用、无脏数据和回写任务的干净节点。
    ///
    /// 返回被摘除的 `RetiredNode`，由调用方在释放 `entries` 自旋锁之后释放底层物理页。
    fn evict_if_full(
        &self,
        entries: &mut Tree,
        reserve: Option<&Arc<BioReserve>>,
    ) -> Result<Option<RetiredNode>, BlockError> {
        if self.capacity.is_none_or(|capacity| entries.len < capacity) {
            return Ok(None);
        }
        let mut cursor = 0;
        while let Some(node) = entries.lower_bound(cursor) {
            let key = node.key;
            let baseline = reserve.is_some_and(|source| source.metadata_has_baseline(&node.folio));
            if !node.folio.is_memory() && node.folio.index_unused(baseline, false) {
                return Ok(entries.remove(key));
            }
            let Some(next) = key.checked_add(1) else {
                break;
            };
            cursor = next;
        }
        Err(BlockError::NoMemory)
    }

    /// 查找键为 `key` 的缓存页；若未命中则在锁外分配新页与 `Node` 并插入索引。
    ///
    /// - 对于已就绪的可读写元数据缓存（`reserve` 为 `Some`），从 `BioReserve` 预留池租借固定槽位与物理页；
    /// - 对于只读元数据或普通文件缓存（`reserve` 为 `None`），从内核堆与伙伴系统分配；
    /// - 返回 `(FolioRef, Option<FolioRead>)`：若本次调用新建了非元数据页，则同时返回整页读取凭证 `Some(FolioRead)`。
    pub(crate) fn get_or_prepare(
        &self,
        key: u64,
        reserve: Option<&Arc<BioReserve>>,
    ) -> Result<(FolioRef, Option<FolioRead>), BlockError> {
        // Step 1: 持锁检查目标键是否已命中；若未命中且容量已满，先摘除一个可驱逐的空闲节点并在锁外释放。
        let victim = {
            let mut entries = self.entries.lock();
            if entries.closed {
                return Err(BlockError::QueueStopped);
            }
            if let Some(node) = entries.get(key) {
                return Ok((node.folio.clone(), None));
            }
            self.evict_if_full(&mut entries, reserve)?
        };
        if let Some(victim) = victim {
            victim.release();
        }

        // Step 2: 在自旋锁外准备候选物理页（`candidate`）与索引节点（`Node`）。
        let (candidate, allocator, lease) = if let Some(reserve) = reserve {
            assert!(self.metadata, "file/raw index received a metadata source");
            let lease = reserve.try_metadata_slot()?;
            let candidate = lease.folio();
            candidate.reset_metadata_inventory();
            let allocator = IndexAllocator::Metadata {
                reserve: Arc::clone(reserve),
                slot: lease.slot(),
            };
            (candidate, allocator, Some(lease))
        } else {
            let candidate = if self.metadata {
                FolioRef::allocate_metadata()
            } else {
                FolioRef::allocate()
            }
            .ok_or(BlockError::NoMemory)?;
            (candidate, IndexAllocator::Heap, None)
        };
        let read = if self.metadata {
            None
        } else {
            Some(candidate.begin_read(0, PAGE_SIZE)?)
        };
        let mut prepared = Some(Node::prepare(key, candidate.clone(), None, allocator)?);
        drop(lease);

        // Step 3: 再次获取索引锁复查目标键（防止并发插入竞争），若仍未命中则正式插入新节点。
        let (existing, victim) = {
            let mut entries = self.entries.lock();
            if entries.closed {
                return Err(BlockError::QueueStopped);
            }
            if let Some(node) = entries.get(key) {
                (Some(node.folio.clone()), None)
            } else {
                let victim = self.evict_if_full(&mut entries, reserve)?;
                entries.insert(prepared.take().unwrap());
                (None, victim)
            }
        };

        if let Some(victim) = victim {
            victim.release();
        }
        drop(prepared);
        if let Some(folio) = existing {
            drop(read);
            return Ok((folio, None));
        }
        Ok((candidate, read))
    }

    /// 扫描索引并摘除所有当前无外部引用、无脏扇区且无在途回写的空闲节点，返回成功清理的节点数量。
    pub(crate) fn prune_unused(&self, reserve: Option<&Arc<BioReserve>>) -> usize {
        let budget = self.len();
        let mut cursor = 0;
        let mut removed = 0;
        for _ in 0..budget {
            let (victim, next) = {
                let mut entries = self.entries.lock();
                let Some(node) = entries.lower_bound(cursor) else {
                    break;
                };
                let key = node.key;
                let baseline =
                    reserve.is_some_and(|source| source.metadata_has_baseline(&node.folio));
                let victim = if !node.folio.is_memory() && node.folio.index_unused(baseline, false)
                {
                    entries.remove(key)
                } else {
                    None
                };
                (victim, key.checked_add(1))
            };
            removed += usize::from(victim.is_some());
            if let Some(victim) = victim {
                victim.release();
            }
            let Some(next) = next else {
                break;
            };
            cursor = next;
        }
        removed
    }

    /// 返回当前索引树中驻留的节点总数。
    pub(crate) fn len(&self) -> usize {
        self.entries.lock().len
    }

    /// 在半开键区间 `[start, end)` 内按升序扫描，将最多 `max_pages` 个存在脏数据或在途回写的节点标记为 `to_write`。
    ///
    /// 返回 `(本轮标记的节点数, 下一轮续扫的起始键)`；若已扫描至区间末尾，则下一轮起始键归零。
    fn mark_to_write_range(&self, start: u64, end: u64, max_pages: usize) -> (usize, u64) {
        if max_pages == 0 {
            return (0, start);
        }
        let mut entries = self.entries.lock();
        let budget = entries.len;
        let mut cursor = start;
        let mut selected = 0;
        for _ in 0..budget {
            let Some(node) = entries.lower_bound(cursor) else {
                break;
            };
            let key = node.key;
            if end <= key {
                break;
            }
            if selected == max_pages {
                return (selected, cursor);
            }
            let (dirty, writeback) = node.folio.writeback_state();
            let pending = dirty != 0 || writeback != 0;
            entries.set_to_write(key, pending);
            selected += usize::from(pending);
            let Some(next) = key.checked_add(1) else {
                break;
            };
            cursor = next;
        }
        (selected, 0)
    }

    /// 检查索引中是否仍存在任何包含脏扇区或正在回写的物理页。
    fn has_dirty_or_writeback(&self) -> bool {
        let entries = self.entries.lock();
        let mut cursor = 0;
        while let Some(node) = entries.lower_bound(cursor) {
            let (dirty, writeback) = node.folio.writeback_state();
            if dirty != 0 || writeback != 0 {
                return true;
            }
            let Some(next) = node.key.checked_add(1) else {
                break;
            };
            cursor = next;
        }
        false
    }

    /// 在半开键区间 `[start, end)` 内按序筛选匹配 `mark` 状态的节点，将其 `(key, FolioRef)` 填入 `output` 切片。
    ///
    /// 若在填满 `output` 或耗尽扫描预算时仍未到达 `end`，则返回 `Some(next_cursor)` 供调用方继续分段扫描。
    pub(crate) fn select_range(
        &self,
        start: u64,
        end: u64,
        mark: FolioMark,
        output: &mut [Option<(u64, FolioRef)>],
    ) -> Option<u64> {
        let entries = self.entries.lock();
        let mut cursor = start;
        let mut selected = 0;
        let budget = if mark == FolioMark::ToWrite {
            entries.len
        } else {
            output.len()
        };
        for _ in 0..budget {
            if selected == output.len() {
                break;
            }
            let node = entries.lower_bound(cursor)?;
            if end <= node.key {
                return None;
            }
            let matches = match mark {
                FolioMark::Dirty => node.folio.dirty_sectors() != 0,
                FolioMark::Writeback => node.folio.writeback_sectors() != 0,
                FolioMark::ToWrite => node.to_write(),
            };
            if matches {
                assert!(
                    output[selected].is_none(),
                    "folio scan output retains an owner"
                );
                output[selected] = Some((node.key, node.folio.clone()));
                selected += 1;
            }
            cursor = node.key.checked_add(1)?;
        }
        (cursor < end).then_some(cursor)
    }
}

/// 元数据页槽位在索引中发布后、启动扇区读取前触发的测试观察回调类型。
#[cfg(feature = "ktest")]
pub(crate) type MetadataLoadHook = Arc<dyn Fn(&Arc<MetadataCache>, u64, usize) + Send + Sync>;

/// 单个物理块设备（`Disk`）全局共享的元数据物理页域。
pub struct MetadataPages {
    /// 以磁盘绝对字节偏移（4KiB 页对齐）为键的稀疏物理页索引。
    index: FolioIndex,
    /// 保护 `reserve` 预留池首次初始化及热节点迁移过程的睡眠读写锁。
    reserve_init: SleepRwLock<()>,
    /// 可读写挂载时初始化的块层元数据预留池（包含固定数量的物理页与 `Node` 控制块槽位）。
    reserve: SpinLock<Option<Arc<BioReserve>>>,
    /// 当前磁盘上已创建的各分区/卷 `MetadataCache` 门面的弱引用列表。
    facades: SleepMutex<Vec<Weak<MetadataCache>>>,
    /// 底层物理块设备的强引用。
    disk: DiskRef,
}

/// 绑定到具体磁盘分区或卷范围（`FsClaim`）的元数据缓存视图门面。
pub struct MetadataCache {
    /// 指向所属物理磁盘共享的 `MetadataPages` 页域。
    pages: Arc<MetadataPages>,
    /// 关联的 `Ext4Volume` 弱引用，用于在元数据被修改变脏时通知对应卷的回写域。
    pub(crate) filesystem: SleepMutex<Weak<Ext4Volume>>,
    /// 当前视图在块设备上的分区范围与读写权限声明。
    claim: FsClaim,
    /// 串行化当前视图的元数据回写扫描与同步完成，并保存后台增量回写的续扫游标（磁盘字节偏移）。
    writeback: SleepMutex<u64>,
    /// 当前视图在本轮同步/回写中被标记为 `ToWrite` 的物理页键（字节偏移）列表。
    selected: SpinLock<Vec<u64>>,
    /// 记录当前视图元数据异步回写错误的 `errseq_t` 状态机。
    wb_error: SpinLock<WritebackError>,
    /// 标记当前元数据缓存视图是否已关闭（关闭后拒绝新的块获取请求）。
    closed: AtomicBool,
    /// 内核测试专用的元数据加载阶段观察钩子。
    #[cfg(feature = "ktest")]
    load_hook: SpinLock<Option<MetadataLoadHook>>,
}

impl Deref for MetadataCache {
    type Target = MetadataPages;

    /// 解引用到所属物理磁盘共享的 `MetadataPages`。
    fn deref(&self) -> &Self::Target {
        &self.pages
    }
}

impl MetadataPages {
    /// 在分区或卷卸载释放 `FsClaim` 时，使半开扇区区间 `[start, end)` 内已无任何活跃占用的元数据扇区状态失效。
    ///
    /// # Safety
    /// 1. 调用方必须持有所属磁盘的队列锁，且待卸载的最后一个 `FsClaim` 尚未从队列摘除（阻止新访问重新准入该区间）。
    /// 2. 传入的 `[start, end)` 扇区区间必须已剔除同一磁盘上其他仍存活 `FsClaim` 的重叠范围。
    /// 3. 该区间内所有的 `BufferRef` 借用、在途读取、脏扇区与回写 I/O 必须已全部完成。
    pub(crate) unsafe fn retire_range(&self, start: u64, end: u64) {
        let Some(byte) = start.checked_mul(SECTOR_BYTES) else {
            return;
        };
        let mut cursor = byte / PAGE_SIZE as u64 * PAGE_SIZE as u64;
        let entries = self.index.entries.lock();
        while let Some(node) = entries
            .lower_bound(cursor)
            .filter(|node| node.key / SECTOR_BYTES < end)
        {
            let page = node.key / SECTOR_BYTES;
            let first = start.max(page);
            let last = end.min(page + PAGE_SIZE as u64 / SECTOR_BYTES);
            let offset = ((first - page) * SECTOR_BYTES) as usize;
            let len = ((last - first) * SECTOR_BYTES) as usize;
            // SAFETY:
            // 1. 调用方持有磁盘队列锁并已核验 `[start, end)` 内无任何活跃 `BufferRef`、读取或回写任务。
            // 2. 当前持有 `entries` 自旋锁保证 `node.folio` 存活，且 `[offset, offset + len)` 严格限制在
            //    当前物理页与待退休扇区区间的交集内，不会影响同一物理页内属于相邻分区的其他扇区。
            unsafe { node.folio.retire_metadata(offset, len) };
            cursor = node.key + 1;
        }
    }
}

impl MetadataCache {
    /// 获取当前元数据缓存视图的回写睡眠互斥锁，用于串行化显式同步与后台回写扫描，并返回后台续扫游标守卫。
    pub(crate) fn lock_writeback(&self) -> SleepMutexGuard<'_, u64> {
        self.writeback.lock()
    }

    /// 在可读写文件系统挂载发布前，初始化块层元数据预留池（`BioReserve`），并将引导阶段已驻留的热元数据节点
    /// 平滑替换为由预留池固定槽位分配的 `Node`，确保运行期元数据读写与回写不再依赖动态堆分配。
    pub(crate) fn prepare_reserve(self: &Arc<Self>) -> Result<Arc<BioReserve>, BlockError> {
        let _scope = ReclaimScope::noio();
        let _init = self.reserve_init.write();
        if let Some(reserve) = self.reserve() {
            return Ok(reserve);
        }

        // Step 1: 扫描索引收集引导期已加载的所有热元数据页 `(key, FolioRef)`。
        let mut hot = Vec::new();
        hot.try_reserve_exact(self.capacity())
            .map_err(|_| BlockError::NoMemory)?;
        let mut cursor = 0;
        loop {
            let next = {
                let entries = self.index.entries.lock();
                if entries.closed {
                    return Err(BlockError::QueueStopped);
                }
                entries
                    .lower_bound(cursor)
                    .map(|node| (node.key, node.folio.clone()))
            };
            let Some((key, folio)) = next else {
                break;
            };
            hot.push((key, folio));
            let Some(next) = key.checked_add(1) else {
                break;
            };
            cursor = next;
        }
        // Step 2: 用热页集合创建固定容量的 `BioReserve`，并预先触发所有槽位的 `Node::prime` 锁定 `Arc` 控制块布局。
        let reserve =
            BioReserve::try_new(self.capacity(), hot.iter().map(|(_, folio)| folio.clone()))?;
        for slot in 0..self.capacity() {
            Node::prime(&reserve, slot)?;
        }

        // Step 3: 将引导期由 `Heap` 分配的旧索引节点逐一原子替换为由 `BioReserve` 槽位分配的新节点。
        // 在单次 `entries` 自旋锁内完成同物理页（PFN）节点的摘除与插入，避免并发扫描观察到节点空窗。
        for (slot, (key, folio)) in hot.into_iter().enumerate() {
            let replacement = Node::prepare(
                key,
                folio,
                None,
                IndexAllocator::Metadata {
                    reserve: Arc::clone(&reserve),
                    slot,
                },
            )
            .expect("primed metadata control allocation failed");
            let old = {
                let mut entries = self.index.entries.lock();
                let node = entries
                    .get(key)
                    .expect("bootstrap metadata node disappeared");
                assert_eq!(
                    node.folio.phys_addr(),
                    replacement.folio.phys_addr(),
                    "bootstrap metadata identity changed"
                );
                // 在同一次索引自旋锁临界区内用新节点替换旧节点，保持物理页与扇区状态连续可见。
                let old = entries.remove(key).unwrap();
                entries.insert(replacement);
                old
            };
            // 在索引自旋锁外释放旧 `Heap` 节点的强引用。
            old.release();
        }

        // Step 4: 标记预留池就绪并发布到 `self.reserve`。
        reserve.mark_ready();
        *self.reserve.lock() = Some(Arc::clone(&reserve));
        Ok(reserve)
    }

    /// 获取当前元数据页域已就绪的 `BioReserve` 预留池强引用（若处于只读模式或尚未初始化则返回 `None`）。
    pub(crate) fn reserve(&self) -> Option<Arc<BioReserve>> {
        self.reserve.lock().clone()
    }

    /// 当某个元数据页上的 `BufferRef`、读取或回写任务释放最后一个引用后，唤醒在 `BioReserve` 上等待空闲槽位的任务。
    pub(crate) fn notify_metadata_available(&self) {
        let reserve = self.reserve();
        if let Some(reserve) = reserve {
            reserve.notify_available();
        }
    }

    /// 为新的同步观察者采样当前元数据回写错误的初始 `errseq_t` 游标值。
    pub(crate) fn sample_wb_error(&self) -> u32 {
        self.wb_error.lock().sample()
    }

    /// 检查自 `*since` 游标以来当前视图是否发生过新的元数据回写错误，并推进游标。
    pub(crate) fn check_and_advance_wb_error(&self, since: &mut u32) -> Result<(), FsError> {
        self.wb_error.lock().check_and_advance(since)
    }

    /// 在元数据回写 I/O 失败时记录具体的 `FsError` 并更新 `errseq_t` 状态字。
    pub(crate) fn record_wb_error(&self, error: FsError) {
        self.wb_error.lock().record(error);
    }

    /// 获取或创建与给定磁盘占用范围（`claim`）和物理页容量预算（`capacity`）对应的 `MetadataCache` 门面。
    ///
    /// - 同一块物理磁盘共享唯一的底层 `MetadataPages` 及总页数预算；
    /// - 若相同扇区范围与访问模式的门面已存在且未关闭，则直接复用该 `Arc<MetadataCache>`。
    pub fn shared(claim: &FsClaim, capacity: usize) -> Result<Arc<Self>, BlockError> {
        if capacity == 0 {
            return Err(BlockError::InvalidRange);
        }
        // Step 1: 获取或初始化所属物理磁盘唯一的 `MetadataPages` 页域，并校验容量预算一致。
        let pages = match claim.disk().metadata_cache() {
            Some(pages) => pages,
            None => {
                let candidate = Arc::try_new(MetadataPages {
                    index: FolioIndex::new(Some(capacity), true),
                    reserve_init: SleepRwLock::new(()),
                    reserve: SpinLock::new(None),
                    facades: SleepMutex::new(Vec::new()),
                    disk: claim.disk().clone(),
                })
                .map_err(|_| BlockError::NoMemory)?;
                claim.disk().intern_metadata_cache(&candidate)
            }
        };
        if pages.index.capacity != Some(capacity) {
            return Err(BlockError::Busy);
        }
        if let Some(existing) = Self::existing_facade(&pages.facades.lock(), claim)? {
            return Ok(existing);
        }

        // Step 2: 分配新的 `MetadataCache` 门面候选实例，并在 `facades` 互斥锁下执行二次检查与弱引用注册。
        let mut selected = Vec::new();
        selected
            .try_reserve_exact(capacity)
            .map_err(|_| BlockError::NoMemory)?;
        let candidate = Arc::try_new(Self {
            pages: pages.clone(),
            claim: claim.clone(),
            filesystem: SleepMutex::new(Weak::new()),
            writeback: SleepMutex::new(0),
            selected: SpinLock::new(selected),
            wb_error: SpinLock::new(WritebackError::new()),
            closed: AtomicBool::new(false),
            #[cfg(feature = "ktest")]
            load_hook: SpinLock::new(None),
        })
        .map_err(|_| BlockError::NoMemory)?;

        let mut facades = pages.facades.lock();
        facades.retain(|entry| entry.strong_count() != 0);
        if let Some(existing) = Self::existing_facade(&facades, claim)? {
            drop(facades);
            return Ok(existing);
        }
        facades.try_reserve(1).map_err(|_| BlockError::NoMemory)?;
        facades.push(Arc::downgrade(&candidate));
        drop(facades);
        Ok(candidate)
    }

    /// 在已注册的门面弱引用列表中查找与 `claim` 具有相同扇区范围的存活 `MetadataCache`：
    /// 若已关闭则返回 `QueueStopped`，若读写模式冲突则返回 `Busy`。
    fn existing_facade(
        facades: &[Weak<Self>],
        claim: &FsClaim,
    ) -> Result<Option<Arc<Self>>, BlockError> {
        for entry in facades {
            if let Some(existing) = entry.upgrade() {
                if existing.claim.view().same_range(claim.view()) {
                    if existing.closed.load(Ordering::Acquire) {
                        return Err(BlockError::QueueStopped);
                    }
                    if existing.claim.mode() != claim.mode() {
                        return Err(BlockError::Busy);
                    }
                    return Ok(Some(existing));
                }
            }
        }
        Ok(None)
    }

    /// 设置或清除元数据页槽位发布后的测试观察钩子，旧钩子在自旋锁外释放。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_load_hook(&self, hook: Option<MetadataLoadHook>) {
        let retired = core::mem::replace(&mut *self.load_hook.lock(), hook);
        drop(retired);
    }

    /// 读取并锁定逻辑块号为 `block`、块大小为 `block_size`（1KiB / 2KiB / 4KiB）的元数据块，返回其 `BufferRef` 视图。
    ///
    /// 若目标块的扇区尚未加载到物理页中，则自动发起块设备读取并等待就绪；已有效或已变脏的扇区不会被磁盘旧数据覆盖。
    pub(crate) fn get(
        self: &Arc<Self>,
        block: u64,
        block_size: usize,
        io_context: Option<MetadataIoContext<'_>>,
    ) -> Result<BufferRef, BlockError> {
        if self.closed.load(Ordering::Acquire) {
            return Err(BlockError::QueueStopped);
        }
        // Step 1: 校验块几何参数，计算磁盘绝对字节偏移及是否跨越两个相邻 4KiB 物理页。
        let (byte, offset) = self.block_range(block, block_size)?;
        let first_len = block_size.min(PAGE_SIZE - offset);
        let page = byte - offset as u64;
        let initial = self.reserve();
        let initial_context = match io_context {
            Some(context) => Some(context),
            None => initial.as_ref().map(BioReserve::metadata_io_context),
        };
        if let Some(context) = initial_context {
            context.check()?;
        }

        // Step 2: 在 `reserve_init` 读锁保护下，在索引中获取或插入首页（及跨页时的尾页）槽位，并绑定当前 `MetadataCache` 属主。
        let (folio, tail, reserve) = {
            let _init = self.reserve_init.read();
            let reserve = self.reserve();
            if let Some(context) = initial_context {
                assert!(
                    reserve
                        .as_ref()
                        .is_some_and(|source| context.belongs_to(source)),
                    "metadata IO source changed"
                );
                context.check()?;
            }
            let (folio, _) = self.index.get_or_prepare(page, reserve.as_ref())?;
            let folio = folio.with_metadata_owner(Arc::clone(self));
            let tail = if first_len < block_size {
                let (tail, _) = self
                    .index
                    .get_or_prepare(byte + first_len as u64, reserve.as_ref())?;
                Some(tail.with_metadata_owner(Arc::clone(self)))
            } else {
                None
            };
            (folio, tail, reserve)
        };
        let context = match initial_context {
            Some(context) => Some(context),
            None => reserve.as_ref().map(BioReserve::metadata_io_context),
        };
        #[cfg(feature = "ktest")]
        // Step 3 (ktest): 在所有页槽位均已发布到索引后、发起扇区读取前，在锁外调用测试钩子。
        {
            let hook = self.load_hook.lock().clone();
            if let Some(hook) = hook {
                hook(self, block, block_size);
            }
        }

        // Step 4: 对首页（及可选的尾页）补齐尚未就绪的扇区数据。
        // 若此前未捕获到 `reserve`（只读或引导阶段），则持 `reserve_init` 读锁防止与并发的 `prepare_reserve` 冲突。
        let _legacy_scope = reserve.is_none().then(ReclaimScope::noio);
        let transition = reserve.is_none().then(|| self.reserve_init.read());
        let published = if transition.is_some() {
            self.reserve()
        } else {
            None
        };
        let source = reserve.as_ref().or(published.as_ref());
        let context = match context {
            Some(context) => Some(context),
            None => source.map(BioReserve::metadata_io_context),
        };
        let _legacy_io = if source.is_none() {
            transition
        } else {
            drop(transition);
            None
        };
        self.get_segment(&folio, byte, first_len, source, context)?;
        if let Some(tail) = &tail {
            self.get_segment(
                tail,
                byte + first_len as u64,
                block_size - first_len,
                source,
                context,
            )?;
        }
        Ok(BufferRef {
            folio,
            tail,
            offset,
            len: block_size,
        })
    }

    /// 确保单个物理页 `folio` 内 `[byte % PAGE_SIZE, byte % PAGE_SIZE + len)` 字节区间的元数据扇区已加载就绪。
    ///
    /// 若部分扇区尚未就绪，则通过单飞（Singleflight）机制调用 `begin_metadata_read` 认领缺失区间并从磁盘读取。
    fn get_segment(
        self: &Arc<Self>,
        folio: &FolioRef,
        byte: u64,
        len: usize,
        source: Option<&Arc<BioReserve>>,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<(), BlockError> {
        let offset = (byte % PAGE_SIZE as u64) as usize;
        let page = byte - offset as u64;
        loop {
            if let Some(context) = context {
                context.check()?;
            }
            match folio.range_ready(offset, len) {
                Ok(()) => break,
                Err(BlockError::NotReady) => {}
                Err(error) => return Err(error),
            }

            let _scope = source.map(|_| ReclaimScope::noio());
            let lease = match (source, context) {
                (Some(source), Some(context)) => Some(source.acquire_aux(context)?),
                (None, None) => None,
                _ => unreachable!("metadata IO context lost its actual source"),
            };
            let memory = lease.as_ref().map(|lease| lease.memory()).transpose()?;
            if let Some(context) = context {
                context.check()?;
            }
            let Some(read) = folio.begin_metadata_read(offset, len)? else {
                break;
            };
            let read = read.with_metadata_owner(Arc::clone(self));
            let memory = memory.unwrap_or_else(|| BioMemory::new_in(BioAllocator::Heap));
            let relative =
                page + read.offset() as u64 - self.claim.view().start_sector() * SECTOR_BYTES;
            self.claim.load_folio(relative, read, memory)?;
        }
        Ok(())
    }

    /// 仅当目标元数据块涉及的所有物理页均已驻留在索引中时，直接返回对应的 `BufferRef`（不分配新节点，也不发起磁盘读取）。
    pub(crate) fn cached_buffer(
        self: &Arc<Self>,
        block: u64,
        block_size: usize,
    ) -> Result<Option<BufferRef>, BlockError> {
        if self.closed.load(Ordering::Acquire) {
            return Err(BlockError::QueueStopped);
        }
        let (byte, offset) = self.block_range(block, block_size)?;
        let page = byte - offset as u64;
        let first_len = block_size.min(PAGE_SIZE - offset);
        let (folio, tail) = {
            let entries = self.index.entries.lock();
            let Some(node) = entries.get(page) else {
                return Ok(None);
            };
            let tail = if first_len < block_size {
                let Some(node) = entries.get(page + PAGE_SIZE as u64) else {
                    return Ok(None);
                };
                Some(node.folio.clone().with_metadata_owner(self.clone()))
            } else {
                None
            };
            (node.folio.clone(), tail)
        };
        Ok(Some(BufferRef {
            folio: folio.with_metadata_owner(self.clone()),
            tail,
            offset,
            len: block_size,
        }))
    }

    /// 校验逻辑块号 `block` 与块大小 `block_size` 是否合法且满足设备扇区对齐，返回 `(磁盘绝对字节偏移, 页内字节偏移)`。
    fn block_range(&self, block: u64, block_size: usize) -> Result<(u64, usize), BlockError> {
        if !matches!(
            block_size,
            FILE_BLOCK_BYTES_1K | FILE_BLOCK_BYTES_2K | FILE_BLOCK_BYTES_4K
        ) {
            return Err(BlockError::InvalidRange);
        }
        let size = u64::try_from(block_size).map_err(|_| BlockError::InvalidRange)?;
        let byte = block.checked_mul(size).ok_or(BlockError::InvalidRange)?;
        let byte = self.claim.view().absolute_byte(byte, block_size)?;
        let limits = self.claim.view().limits();
        let logical = u64::from(limits.logical_block_size);
        if byte % logical != 0 || size % logical != 0 {
            return Err(BlockError::Misaligned);
        }
        let offset = (byte % PAGE_SIZE as u64) as usize;
        Ok((byte, offset))
    }

    /// 扫描当前分区视图覆盖的物理页区间，摘除并释放所有无外部引用、无脏扇区且无在途回写的空闲节点，返回清理的节点数。
    pub fn prune_unused(&self) -> usize {
        let reserve = self.reserve();
        let (mut cursor, end) = self.page_bounds();
        let mut removed = 0;
        loop {
            let (victim, next) = {
                let mut entries = self.index.entries.lock();
                let Some(node) = entries.lower_bound(cursor).filter(|node| node.key < end) else {
                    break;
                };
                let key = node.key;
                let baseline = reserve
                    .as_ref()
                    .is_some_and(|source| source.metadata_has_baseline(&node.folio));
                let victim = if node.folio.index_unused(baseline, false) {
                    entries.remove(key)
                } else {
                    None
                };
                (victim, key + 1)
            };
            if let Some(victim) = victim {
                removed += 1;
                victim.release();
            }
            cursor = next;
        }
        removed
    }

    /// 统计当前分区视图范围在共享元数据索引中实际驻留的物理页数量。
    #[cfg(feature = "ktest")]
    pub fn len(&self) -> usize {
        let (mut cursor, end) = self.page_bounds();
        let mut count = 0;
        loop {
            let next = {
                let entries = self.index.entries.lock();
                entries
                    .lower_bound(cursor)
                    .filter(|node| node.key < end)
                    .map(|node| node.key + 1)
            };
            let Some(next) = next else { break };
            count += 1;
            cursor = next;
        }
        count
    }

    /// 判断当前分区视图范围在元数据索引中是否没有任何驻留物理页。
    #[cfg(feature = "ktest")]
    #[expect(
        dead_code,
        reason = "元数据物理页索引与 len 共同提供集合空态查询，当前测试只直接读取长度"
    )]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 在当前分区视图的 `selected` 列表中设置或取消指定物理页键（`byte_offset`）的本轮回写选中标记。
    pub(crate) fn set_to_write(&self, byte_offset: u64, selected: bool) {
        let mut keys = self.selected.lock();
        if selected {
            if !keys.contains(&byte_offset) {
                assert!(
                    keys.len() < self.capacity(),
                    "metadata selection exceeds physical budget"
                );
                keys.push(byte_offset);
            }
        } else {
            keys.retain(|key| *key != byte_offset);
        }
    }

    /// 在半开字节区间 `[start, end)` 内将所有属于当前视图的脏页或正在回写的页标记为 `ToWrite`，返回选中的页数。
    pub(crate) fn mark_to_write_range(&self, start: u64, end: u64) -> usize {
        self.mark_to_write_budget(start, end, usize::MAX).0
    }

    /// 在半开字节区间 `[start, end)` 内将最多 `max_pages` 个属于当前视图的脏页或回写页标记为 `ToWrite`，
    /// 返回 `(本轮选中的页数, 下一轮续扫的起始字节偏移)`。
    pub(crate) fn mark_to_write_budget(
        &self,
        start: u64,
        end: u64,
        max_pages: usize,
    ) -> (usize, u64) {
        self.selected.lock().clear();
        let (first, limit) = self.page_bounds();
        let mut cursor = start.max(first);
        let end = end.min(limit);
        let mut count = 0;
        while cursor < end {
            if count == max_pages {
                return (count, cursor);
            }
            let next = {
                let entries = self.index.entries.lock();
                entries
                    .lower_bound(cursor)
                    .filter(|node| node.key < end)
                    .map(|node| (node.key, node.folio.clone()))
            };
            let Some((key, folio)) = next else { break };
            let (dirty, writeback) = folio.writeback_state();
            if (dirty | writeback) & self.page_mask(key) != 0 {
                self.set_to_write(key, true);
                count += 1;
            }
            cursor = key + 1;
        }
        (count, 0)
    }

    /// 检查当前分区视图范围内是否仍存在包含脏扇区或正在回写的元数据页。
    pub(crate) fn has_dirty_or_writeback(&self) -> bool {
        let mut selected = [None];
        self.select_range(0, u64::MAX, FolioMark::Dirty, &mut selected);
        if selected[0].is_some() {
            return true;
        }
        self.select_range(0, u64::MAX, FolioMark::Writeback, &mut selected);
        selected[0].is_some()
    }

    /// 在半开字节区间 `[start, end)` 内按序筛选当前视图中匹配 `mark` 状态的元数据页，填入 `output` 切片并返回下一轮续扫游标。
    pub(crate) fn select_range(
        &self,
        start: u64,
        end: u64,
        mark: FolioMark,
        output: &mut [Option<(u64, FolioRef)>],
    ) -> Option<u64> {
        let (first, limit) = self.page_bounds();
        let mut cursor = start.max(first);
        let end = end.min(limit);
        let mut count = 0;
        while cursor < end && count < output.len() {
            let next = {
                let entries = self.index.entries.lock();
                entries
                    .lower_bound(cursor)
                    .filter(|node| node.key < end)
                    .map(|node| (node.key, node.folio.clone()))
            };
            let (key, folio) = next?;
            let mask = self.page_mask(key);
            let matches = match mark {
                FolioMark::Dirty => folio.dirty_sectors() & mask != 0,
                FolioMark::Writeback => folio.writeback_sectors() & mask != 0,
                FolioMark::ToWrite => self.selected.lock().contains(&key),
            };
            if matches {
                assert!(
                    output[count].is_none(),
                    "metadata scan output retains an owner"
                );
                output[count] = Some((key, folio));
                count += 1;
            }
            cursor = key + 1;
        }
        (cursor < end).then_some(cursor)
    }

    /// 计算当前分区视图在物理磁盘上的字节范围：返回 `(向下按 4KiB 页对齐的起始字节偏移, 结束字节偏移)`。
    fn page_bounds(&self) -> (u64, u64) {
        let start = self.claim.view().start_sector() * SECTOR_BYTES;
        let end = start + self.claim.view().sectors() * SECTOR_BYTES;
        (start / PAGE_SIZE as u64 * PAGE_SIZE as u64, end)
    }

    /// 计算在起始磁盘字节偏移为 `page` 的 4KiB 物理页中，属于当前分区视图的 512B 扇区 8 位掩码（Bit `i` 对应页内第 `i` 个扇区）。
    pub(crate) fn page_mask(&self, page: u64) -> u8 {
        let start = self.claim.view().start_sector() * SECTOR_BYTES;
        let end = start + self.claim.view().sectors() * SECTOR_BYTES;
        let first = start.max(page);
        let last = end.min(page.saturating_add(PAGE_SIZE as u64));
        if last <= first {
            return 0;
        }
        let offset = ((first - page) / SECTOR_BYTES) as usize;
        let sectors = ((last - first) / SECTOR_BYTES) as usize;
        (((1u16 << sectors) - 1) << offset) as u8
    }

    /// 返回所属物理磁盘元数据缓存的总物理页容量预算。
    pub fn capacity(&self) -> usize {
        self.index.capacity.unwrap()
    }

    /// 在预留池已就绪的前提下，获取当前视图范围内指定物理页键对应的 `Node` 弱引用（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_node(&self, byte_offset: u64) -> Option<Weak<Node, IndexAllocator>> {
        let _init = self.reserve_init.read();
        self.reserve()?;
        if self.page_mask(byte_offset) == 0 {
            return None;
        }
        self.index.entries.lock().weak_node(byte_offset)
    }

    /// 关闭当前元数据缓存视图，并丢弃本分区范围内所有未落盘的私有元数据扇区，同时保留共享边界页上属于相邻分区的扇区数据。
    ///
    /// # Safety
    /// 1. 调用方必须已停止针对当前分区视图的一切并发写入、读取、回写任务以及外部 `BufferRef` 借用。
    /// 2. 当前视图中被丢弃的脏扇区仅包含测试或未提交的私有修改，不得包含已向文件系统承诺持久化的元数据变更。
    #[cfg(feature = "ktest")]
    pub unsafe fn discard_private(self: &Arc<Self>) -> Result<usize, BlockError> {
        let reserve = self.reserve();
        let (first, end) = self.page_bounds();
        let mut cursor = first;
        // Step 1: 预检查当前视图覆盖的所有独占物理页均已无外部引用或活跃 I/O。
        loop {
            let next = {
                let entries = self.index.entries.lock();
                entries
                    .lower_bound(cursor)
                    .filter(|node| node.key < end)
                    .map(|node| {
                        let baseline = reserve
                            .as_ref()
                            .is_some_and(|source| source.metadata_has_baseline(&node.folio));
                        (
                            node.key,
                            self.page_mask(node.key) != u8::MAX
                                || node.folio.index_unused(baseline, true),
                        )
                    })
            };
            let Some((key, unused)) = next else { break };
            if !unused {
                return Err(BlockError::Busy);
            }
            cursor = key + 1;
        }

        // Step 2: 标记当前视图已关闭，并逐页清除属于当前视图扇区掩码范围内的有效位与脏位。
        self.closed.store(true, Ordering::Release);
        cursor = first;
        loop {
            let next = {
                let entries = self.index.entries.lock();
                entries
                    .lower_bound(cursor)
                    .filter(|node| node.key < end)
                    .map(|node| (node.key, node.folio.clone()))
            };
            let Some((key, folio)) = next else { break };
            let mask = self.page_mask(key);
            let offset = mask.trailing_zeros() as usize * SECTOR_BYTES as usize;
            let len = mask.count_ones() as usize * SECTOR_BYTES as usize;
            let mut content = folio.lock_content();
            // SAFETY:
            // 1. 调用方保证当前视图已无任何并发生产者或外部 `BufferRef` 访问，且上方 Step 1 已核验无在途 I/O。
            // 2. 当前持有 `folio.lock_content()` 睡眠锁，且 `[offset, offset + len)` 严格由 `self.page_mask(key)` 算出，
            //    仅作废当前分区占有的扇区，不会篡改同一物理页内属于相邻分区的扇区内容与脏标记。
            unsafe { content.forget_metadata(offset, len)? };
            cursor = key + 1;
        }
        self.selected.lock().clear();
        Ok(self.prune_unused())
    }
}

impl Drop for MetadataPages {
    /// 当物理磁盘的 `MetadataPages` 最后一个强引用释放时，从所属 `Disk` 注销弱引用登记。
    fn drop(&mut self) {
        let identity = self as *const Self;
        let retired = self.disk.unregister_metadata_cache(identity);
        drop(retired);
    }
}

/// 指向元数据缓存中一个 1KiB / 2KiB / 4KiB 逻辑块窗口的引用句柄（支持跨相邻两个 4KiB 物理页）。
#[derive(Clone)]
pub struct BufferRef {
    /// 逻辑块起始位置所在的第一个 4KiB 物理页（附带所属 `MetadataCache` 强引用）。
    folio: FolioRef,
    /// 当非页对齐分区的逻辑块跨越 4KiB 页边界时，指向紧邻的下一个 4KiB 物理页。
    tail: Option<FolioRef>,
    /// 逻辑块起始字节在 `folio`（首页）内的字节偏移量（`0..PAGE_SIZE`）。
    offset: usize,
    /// 逻辑块的字节长度（1024、2048 或 4096）。
    len: usize,
}

impl BufferRef {
    /// 在持有底层物理页内容锁（`lock_content`）的保护下，以只读字节切片 `&[u8]` 的形式访问该元数据块内容。
    ///
    /// 注意：`consume` 闭包在页内容锁内执行，不得阻塞等待新 I/O 或执行可能触发缺页异常的用户态内存拷贝。
    pub fn with_read<T>(&self, consume: impl FnOnce(&[u8]) -> T) -> Result<T, BlockError> {
        if let Some(tail) = &self.tail {
            return self.read_split(tail, consume);
        }
        let content = self.folio.lock_content();
        Ok(consume(content.metadata_bytes(self.offset, self.len)?))
    }

    /// 处理跨越两个物理页的元数据块只读访问：按物理页偏移升序依次锁定首页与尾页，将两段字节拼入栈缓冲区后交给 `consume`。
    #[inline(never)]
    fn read_split<T>(
        &self,
        tail: &FolioRef,
        consume: impl FnOnce(&[u8]) -> T,
    ) -> Result<T, BlockError> {
        let first = self.folio.lock_content();
        let second = tail.lock_content();
        let count = PAGE_SIZE - self.offset;
        let mut bytes = [0u8; PAGE_SIZE];
        bytes[..count].copy_from_slice(first.metadata_bytes(self.offset, count)?);
        bytes[count..self.len].copy_from_slice(second.metadata_bytes(0, self.len - count)?);
        Ok(consume(&bytes[..self.len]))
    }

    /// 在验证 `claim` 具备当前分区视图的读写权限后，锁定底层物理页内容并以可变切片 `&mut [u8]` 执行原地修改闭包 `update`。
    ///
    /// 修改前会自动将对应扇区标记为脏（Dirty），并在释放页内容锁之后通知关联的 `Ext4Volume` 回写域。
    pub fn update<T>(
        &self,
        claim: &FsClaim,
        update: impl FnOnce(&mut [u8]) -> T,
    ) -> Result<T, BlockError> {
        if claim.mode() != AccessMode::ReadWrite {
            return Err(BlockError::ReadOnly);
        }
        let cache = self
            .folio
            .metadata_owner()
            .expect("metadata buffer lost its cache owner");
        if !claim.view().same_range(cache.claim.view()) {
            return Err(BlockError::InvalidRange);
        }

        let result = if let Some(tail) = &self.tail {
            self.update_split(tail, update)?
        } else {
            let mut content = self.folio.lock_content();
            update(content.metadata_bytes_mut(self.offset, self.len)?)
        };

        let volume = cache.filesystem.lock().upgrade();
        if let Some(volume) = volume {
            volume.writeback_domain().notify_dirty(&volume);
        }
        Ok(result)
    }

    /// 处理跨越两个物理页的元数据块原地修改：同时持有首页与尾页的内容锁，在栈缓冲区中执行 `update` 后写回两页，防止并发修改丢失。
    #[inline(never)]
    fn update_split<T>(
        &self,
        tail: &FolioRef,
        update: impl FnOnce(&mut [u8]) -> T,
    ) -> Result<T, BlockError> {
        let mut first = self.folio.lock_content();
        let mut second = tail.lock_content();
        let count = PAGE_SIZE - self.offset;
        let first_bytes = first.metadata_bytes_mut(self.offset, count)?;
        let second_bytes = second.metadata_bytes_mut(0, self.len - count)?;
        let mut bytes = [0u8; PAGE_SIZE];
        bytes[..count].copy_from_slice(first_bytes);
        bytes[count..self.len].copy_from_slice(second_bytes);
        let result = update(&mut bytes[..self.len]);
        first_bytes.copy_from_slice(&bytes[..count]);
        second_bytes.copy_from_slice(&bytes[count..self.len]);
        Ok(result)
    }

    /// 等待该元数据块所覆盖的全部物理页（含跨页时的尾页）上正在进行的回写 I/O 完成。
    pub(crate) fn wait_writeback(&self) {
        self.folio.wait_writeback();
        if let Some(tail) = &self.tail {
            tail.wait_writeback();
        }
    }

    /// 当文件系统释放某个元数据块（如缩减目录或释放 Extent 树节点）且调用方已等待其在途回写完成后，直接清除该块窗口的脏标记与有效标记。
    ///
    /// # Safety
    /// 1. 调用方必须已持有文件系统的块映射写锁并将该元数据块从树/目录结构中摘除，确保不会有新的读取者获取该块。
    /// 2. 针对该块的所有旧读取、修改和 I/O 请求必须已结束，且该磁盘块尚未归还给空闲块位图（防止与新分配复用产生冲突）。
    pub(crate) unsafe fn forget(&self, claim: &FsClaim) -> Result<(), BlockError> {
        if claim.mode() != AccessMode::ReadWrite {
            return Err(BlockError::ReadOnly);
        }
        let cache = self
            .folio
            .metadata_owner()
            .expect("metadata buffer lost its cache owner");
        if !claim.view().same_range(cache.claim.view()) {
            return Err(BlockError::InvalidRange);
        }

        let mut content = self.folio.lock_content();
        let count = self.len.min(PAGE_SIZE - self.offset);
        // SAFETY:
        // 1. 调用方已将该元数据块从文件系统结构中摘除且尚未归还位图，当前无其他读取或修改者使用该窗口。
        // 2. 已持有 `self.folio.lock_content()` 独占锁，且 `[self.offset, self.offset + count)` 仅覆盖该块在首页中的片段，不影响同页其他块。
        unsafe { content.forget_metadata(self.offset, count)? };
        if let Some(tail) = &self.tail {
            let mut content = tail.lock_content();
            // SAFETY: 同上，已持有尾页 `tail.lock_content()` 独占锁，且 `[0, self.len - count)` 仅覆盖该跨页块在尾页中的剩余片段。
            unsafe { content.forget_metadata(0, self.len - count)? };
        }
        Ok(())
    }

    /// 返回该元数据块视图的字节长度。
    #[cfg(feature = "ktest")]
    pub fn len(&self) -> usize {
        self.len
    }

    /// 判断该元数据块视图的字节长度是否为 0。
    #[cfg(feature = "ktest")]
    #[expect(
        dead_code,
        reason = "元数据 Buffer 与 len 共同提供字节视图空态查询，当前合法构造长度非零且测试只读取长度"
    )]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// 返回该元数据块起始位置在首个 4KiB 物理页内的字节偏移。
    #[cfg(feature = "ktest")]
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// 返回首个 4KiB 物理页的引用（仅用于内核测试诊断物理页状态）。
    #[cfg(feature = "ktest")]
    pub fn folio(&self) -> &FolioRef {
        &self.folio
    }
}

/// 描述单个文件映射虚拟内存区域（VMA）与后端 `Inode` 关联关系的不可变元数据，用于反向映射（rmap）遍历。
pub(crate) struct FileRangeData {
    /// 被映射的目标文件 `Inode` 强引用。
    pub(crate) inode: InodeRef,
    /// 该 VMA 所属进程地址空间的页表域（`PteDomain`），用于在截断或回收时定位并卸载用户态页表项。
    pub(crate) domain: Arc<PteDomain>,
    /// 该 VMA 在用户地址空间中的虚拟地址半开区间 `[start_va, end_va)`（字节）。
    pub(crate) range: Range<usize>,
    /// `range.start` 对应于文件内部的起始字节偏移量（按 4KiB 页对齐）。
    pub(crate) offset: u64,
    /// 标记该映射是否为私有写时复制映射（`MAP_PRIVATE` 为 `true`，`MAP_SHARED` 为 `false`）。
    pub(crate) private: bool,
}

impl FileRangeData {
    /// 生成用于在 `FileMapping::ranges` 有序数组中唯一定位该映射条目的复合排序键 `(文件起始字节偏移, Arc 实例内存地址)`。
    fn key(this: &Arc<Self>) -> (u64, usize) {
        (this.offset, Arc::as_ptr(this) as usize)
    }
}

/// 由文件映射 VMA 独占持有的反向映射注册守卫（Drop 时自动从 `FileMapping::ranges` 注销）。
pub(crate) struct FileRangeRegistration {
    /// 当前 VMA 对应的文件映射范围描述数据。
    data: Arc<FileRangeData>,
}

impl FileRangeRegistration {
    /// 分配 `FileRangeData` 并将其注册到目标文件 `inode.mapping()` 的反向映射有序表中。
    pub(crate) fn try_new(
        inode: InodeRef,
        domain: Arc<PteDomain>,
        range: Range<usize>,
        offset: u64,
        private: bool,
    ) -> Result<Self, BlockError> {
        let data = Arc::try_new(FileRangeData {
            inode,
            domain,
            range,
            offset,
            private,
        })
        .map_err(|_| BlockError::NoMemory)?;
        data.inode.mapping().register_range(&data)?;
        Ok(Self { data })
    }

    /// 获取底层的 `Arc<FileRangeData>` 只读引用。
    pub(crate) fn data(&self) -> &Arc<FileRangeData> {
        &self.data
    }
}

impl Drop for FileRangeRegistration {
    /// 在 VMA 销毁时，先持 `ranges` 自旋锁将自身的 `Weak` 条目从反向映射表中移除，再在锁外释放 `data`。
    fn drop(&mut self) {
        let detached = {
            let mut ranges = self.data.inode.mapping().ranges.lock();
            let key = FileRangeData::key(&self.data);
            let index = ranges
                .binary_search_by_key(&key, |(key, _)| *key)
                .expect("file range registration disappeared");
            ranges.remove(index)
        };
        drop(detached);
    }
}

/// 文件映射范围卸载（unmap）过程中全 CPU 硬件 TLB 同步前后的测试观察阶段枚举。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FileUnmapPhase {
    /// 页表叶子项已清除、但尚未执行全核 TLB 刷新（`sync_all`）之前的阶段。
    BeforeSync,
    /// 全核 TLB 刷新（`sync_all`）已完成、但尚未释放退休物理页与页表页之前的阶段。
    AfterSync,
}

/// 在 `Inode` 失效锁（`invalidate`）之外预先查找或分配的文件页候选结果。
pub(crate) enum PreparedFile {
    /// 索引中已命中的现有物理页引用（已附带所属 `InodeRef`）。
    Cached(FolioRef),
    /// 索引未命中而在锁外新分配的索引节点与整页读取凭证（`node` 为预分配并初始化的索引树节点，`read` 为覆盖整页 `[0, PAGE_SIZE)` 的单飞读取凭证）。
    New { node: NodeRef, read: FolioRead },
}

/// 按 `(文件起始偏移, Arc 地址)` 升序排列的文件反向映射弱引用集合。
type FileRanges = Vec<((u64, usize), Weak<FileRangeData>)>;

/// 文件页表卸载与全核 TLB 同步边界的测试观察回调类型。
#[cfg(feature = "ktest")]
type FileUnmapHook = Arc<dyn Fn(FileUnmapPhase, usize, usize) + Send + Sync>;

/// 单个文件 `Inode` 的页缓存映射与反向映射（rmap）管理器。
pub struct FileMapping {
    /// 以文件内 4KiB 页序号（`index = byte_offset / PAGE_SIZE`）为键的稀疏物理页索引。
    index: FolioIndex,
    /// 映射了该文件的所有用户态 VMA 范围描述表（按文件内起始偏移升序排列）。
    ranges: SpinLock<FileRanges>,
    /// 串行化同一文件的数据回写扫描与同步完成，并保存后台增量回写的续扫文件页序号。
    writeback: SleepMutex<u64>,
    /// 记录该文件数据页异步回写错误的 `errseq_t` 状态机。
    wb_error: SpinLock<WritebackError>,
    /// 内核测试专用的页表卸载与 TLB 同步边界观察钩子。
    #[cfg(feature = "ktest")]
    unmap_hook: SpinLock<Option<FileUnmapHook>>,
}

impl FileMapping {
    /// 获取当前文件的回写睡眠互斥锁，用于串行化该文件的 `fsync` 与后台回写扫描，并返回后台续扫页号守卫。
    pub(crate) fn lock_writeback(&self) -> SleepMutexGuard<'_, u64> {
        self.writeback.lock()
    }

    /// 为新打开的文件描述符（OFD）采样当前文件数据回写错误的初始 `errseq_t` 游标。
    pub(crate) fn sample_wb_error(&self) -> u32 {
        self.wb_error.lock().sample()
    }

    /// 在调用方持有自身 OFD 游标锁的前提下，检查并推进该文件的数据回写错误游标。
    pub(crate) fn check_and_advance_wb_error(&self, since: &mut u32) -> Result<(), FsError> {
        self.wb_error.lock().check_and_advance(since)
    }

    /// 在文件数据页回写 I/O 失败时记录具体的 `FsError` 并更新 `errseq_t` 状态字。
    pub(crate) fn record_wb_error(&self, error: FsError) {
        self.wb_error.lock().record(error);
    }

    /// 检查当前文件索引中是否仍存在包含脏扇区或正在回写的数据页。
    pub(crate) fn has_dirty_or_writeback(&self) -> bool {
        self.index.has_dirty_or_writeback()
    }

    /// 创建一个空的 `FileMapping` 实例。
    pub const fn new() -> Self {
        Self {
            index: FolioIndex::new(None, false),
            ranges: SpinLock::new(Vec::new()),
            writeback: SleepMutex::new(0),
            wb_error: SpinLock::new(WritebackError::new()),
            #[cfg(feature = "ktest")]
            unmap_hook: SpinLock::new(None),
        }
    }

    /// 在不持有 `Inode` 失效锁（`invalidate`）或页内容锁的上下文中，查找第 `index` 页或预先分配新页与 `Node`。
    ///
    /// 若为内存文件系统（tmpfs/ramfs）且未命中，还会预先向所属 Inode 申请内存页配额并绑定到新物理页上。
    pub(crate) fn prepare_file(
        &self,
        owner: &InodeRef,
        index: u64,
        attempt: &mut ReclaimAttempt,
    ) -> Result<PreparedFile, FsError> {
        assert!(
            core::ptr::eq(owner.mapping(), self),
            "file mapping prepare owner changed"
        );
        let cached = {
            let entries = self.index.entries.lock();
            if entries.closed {
                return Err(FsError::LoadAborted);
            }
            entries.get(index).map(|node| node.folio.clone())
        };
        if let Some(folio) = cached {
            return Ok(PreparedFile::Cached(folio.with_file_owner(owner.clone())));
        }

        let mut charge = owner
            .is_memory()
            .then(|| owner.reserve_memory_page())
            .transpose()?;
        attempt
            .prepare(|| {
                let mut folio = FolioRef::allocate().ok_or(BlockError::NoMemory)?;
                let read = folio
                    .begin_read(0, PAGE_SIZE)?
                    .with_file_owner(owner.clone());
                let role = if owner.is_memory() {
                    FileRole::Memory(owner.downgrade_for_reclaim())
                } else {
                    FileRole::Disk(owner.downgrade_for_reclaim())
                };
                let node = Node::prepare(index, folio.clone(), Some(role), IndexAllocator::Heap)?;
                if let Some(charge) = charge.take() {
                    folio.attach_memory_charge(charge);
                }
                Ok(PreparedFile::New { node, read })
            })
            .map_err(Into::into)
    }

    /// 在调用方已持有所属 `Inode` 锁并核验文件大小（EOF）之后，将 `prepare_file` 准备的候选结果发布到索引树中。
    ///
    /// - 若目标页已存在于索引中，则直接返回索引中的物理页（若此前准备了 `PreparedFile::New`，则留在 `prepared` 中由调用方在锁外释放）；
    /// - 若目标页不在索引中且 `prepared` 为 `Cached`（说明原命中页在并发截断/回收中已被移除），则返回 `Ok(None)` 通知调用方重新调用 `prepare_file`。
    pub(crate) fn publish_file(
        &self,
        owner: &InodeRef,
        index: u64,
        prepared: &mut Option<PreparedFile>,
    ) -> Result<Option<(FolioRef, Option<FolioRead>)>, BlockError> {
        assert!(
            core::ptr::eq(owner.mapping(), self),
            "file mapping publish owner changed"
        );
        let result = {
            let mut entries = self.index.entries.lock();
            if entries.closed {
                return Err(BlockError::QueueStopped);
            }
            if let Some(node) = entries.get(index) {
                match prepared {
                    Some(PreparedFile::Cached(folio))
                        if folio.phys_addr() == node.folio.phys_addr() =>
                    {
                        let Some(PreparedFile::Cached(folio)) = prepared.take() else {
                            unreachable!();
                        };
                        return Ok(Some((folio, None)));
                    }
                    _ => (node.folio.clone(), None),
                }
            } else {
                if !matches!(prepared, Some(PreparedFile::New { .. })) {
                    return Ok(None);
                }
                let PreparedFile::New { node, read } = prepared.take().unwrap() else {
                    unreachable!();
                };
                assert_eq!(node.key, index, "file candidate key changed");
                assert!(
                    node.role
                        .as_ref()
                        .is_some_and(|role| role.owner().matches(owner)),
                    "file candidate owner changed"
                );
                let folio = node.folio.clone();
                entries.insert(node);
                (folio, Some(read))
            }
        };
        Ok(Some((result.0.with_file_owner(owner.clone()), result.1)))
    }

    /// 若第 `index` 页当前驻留在文件索引中，则返回其 `FolioRef` 克隆（不分配新页、不发起读取，也不绑定 `InodeRef` 属主）。
    pub(crate) fn cached_folio(&self, index: u64) -> Option<FolioRef> {
        self.index
            .entries
            .lock()
            .get(index)
            .map(|node| node.folio.clone())
    }

    /// 检查文件索引中第 `index` 页是否仍然指向与 `folio` 相同的物理页（相同物理地址 PFN）。
    pub(crate) fn is_canonical(&self, index: u64, folio: &FolioRef) -> bool {
        self.index
            .entries
            .lock()
            .get(index)
            .is_some_and(|node| node.folio.phys_addr() == folio.phys_addr())
    }

    /// 获取反向映射范围表（`ranges`）的自旋锁守卫，用于串行化页表项安装、写时复制（COW）以及 `fork` 页表复制。
    pub(crate) fn lock_ranges(&self) -> SpinLockGuard<'_, FileRanges> {
        self.ranges.lock()
    }

    /// 卸载所有用户态进程地址空间中映射了该文件字节区间 `range` 的页表叶子项，并在锁外完成全核 TLB 刷新后释放退休页。
    ///
    /// - 当 `even_cows == true`（如文件截断收缩）时，连同基于该文件区间派生的私有写时复制（COW）匿名页映射一并卸载；
    /// - 当 `even_cows == false`（如页面回收或整页失效）时，仅卸载直接指向文件缓存页的页表项，保留私有匿名副本。
    pub fn unmap_range(&self, range: Range<u64>, even_cows: bool) -> usize {
        if range.end <= range.start {
            return 0;
        }

        let mut pages = ResidentPages::default();
        let mut tables = RetiredTables::default();
        let mut unmapped = 0;
        {
            // ponytail: 同一 Inode 的 ranges 自旋锁覆盖全范围卸载；若未来超大映射卸载延迟成为瓶颈，可拆分为分段持锁。
            let ranges = self.ranges.lock();
            for ((offset, _), weak) in ranges.iter() {
                if range.end <= *offset {
                    break;
                }
                // SAFETY:
                // 1. `ranges` 中的每个 `Weak<FileRangeData>` 都对应一个存活的 `FileRangeRegistration`；
                //    `FileRangeRegistration::drop` 必须先获取当前持有的 `self.ranges` 自旋锁才能移除条目并析构 `data`，
                //    因此在持有 `self.ranges` 锁期间，`weak.as_ptr()` 指向的 `FileRangeData` 保证有效且不会被释放。
                let data = unsafe { &*weak.as_ptr() };
                // SAFETY:
                // 1. 当前持有 `self.ranges` 自旋锁，与并发的缺页填表、COW 换页及 `fork` 保持互斥。
                // 2. 被卸载页表项所转移出的物理页引用与空页表页均收集在 `pages` 和 `tables` 中，
                //    直到下方释放所有自旋锁并执行 `sync_retired`（全核 TLB 同步）之后才会真正释放。
                unmapped += unsafe {
                    data.domain
                        .unmap_range(data, &range, even_cows, &mut pages, &mut tables)
                };
            }
        }

        self.sync_retired(&pages, &tables);
        drop(pages);
        drop(tables);
        unmapped
    }

    /// 对刚刚从页表中卸下的物理页集合 `pages` 与退休页表页 `tables` 执行全核 TLB 刷新（`sync_all`），并在同步前后触发测试观察钩子。
    fn sync_retired(&self, pages: &ResidentPages, tables: &RetiredTables) {
        #[cfg(feature = "ktest")]
        let hook = self.unmap_hook.lock().clone();
        #[cfg(feature = "ktest")]
        if let Some(hook) = &hook {
            hook(
                FileUnmapPhase::BeforeSync,
                pages.count(&(0..usize::MAX)),
                tables.len(),
            );
        }
        sync_all();
        #[cfg(feature = "ktest")]
        if let Some(hook) = &hook {
            hook(
                FileUnmapPhase::AfterSync,
                pages.count(&(0..usize::MAX)),
                tables.len(),
            );
        }
        #[cfg(not(feature = "ktest"))]
        let _ = (pages, tables);
    }

    /// 在常规内存回收（`ReclaimMode::Normal`）路径中，尝试清理并卸载指向指定干净文件页 `folio` 的所有用户态页表映射。
    ///
    /// 若遍历过程中发现共享映射（`MAP_SHARED`）的页表项带有硬件脏位，则将其转为页缓存脏扇区并中止对该页的回收（返回 `false`）。
    fn reclaim_mappings(
        &self,
        inode: &InodeRef,
        node: &Node,
        folio: &FolioRef,
        content: &mut FolioContent<'_>,
        attempt: &mut ReclaimAttempt,
    ) -> bool {
        const SOURCES: usize = 32;
        let start = node
            .key
            .checked_mul(PAGE_SIZE as u64)
            .expect("file reclaim offset overflow");
        let end = start
            .checked_add(PAGE_SIZE as u64)
            .expect("file reclaim end overflow");
        let mut sources: [Option<Arc<FileRangeData>>; SOURCES] = [const { None }; SOURCES];
        let mut count = 0;
        // Step 1: 在 `ranges` 自旋锁下收集覆盖当前文件页偏移 `[start, end)` 的前 `SOURCES`（最多 32）个 VMA 反向映射描述。
        {
            let ranges = self.ranges.lock();
            for ((offset, _), weak) in ranges.iter() {
                if end <= *offset {
                    break;
                }
                if !attempt.rmap() {
                    return false;
                }
                let data = weak
                    .upgrade()
                    .expect("registered reclaim source lost owner");
                if data
                    .offset
                    .saturating_add((data.range.end - data.range.start) as u64)
                    <= start
                {
                    continue;
                }
                if count == SOURCES {
                    return false;
                }
                sources[count] = Some(data);
                count += 1;
            }
        }

        // Step 2: 遍历所有共享映射（`!data.private`），清除页表项写权限并汇总硬件脏位；若发现被写脏则标记页缓存扇区为 Dirty。
        for data in sources[..count].iter().flatten() {
            if !data.private {
                let (observed, _) = data.domain.mkclean_range(data, start, folio.phys_addr());
                if observed {
                    let size = inode
                        .metadata_snapshot()
                        .expect("reclaim inode is not loaded")
                        .size;
                    let valid = size.saturating_sub(start).min(PAGE_SIZE as u64) as usize;
                    if valid != 0 {
                        inode.note_dirty();
                        content
                            .mark_prepared_dirty(valid, inode.block_size() as usize)
                            .expect("reclaim prepared dirty range changed");
                    }
                }
            }
        }
        sync_all();
        let (dirty, writeback) = folio.writeback_state();
        if dirty != 0 || writeback != 0 {
            return false;
        }

        // Step 3: 若全核 TLB 同步后该页仍保持干净且无回写，则再次持 `ranges` 锁卸载所有指向该文件页的页表叶子项。
        let mut pages = ResidentPages::default();
        let mut tables = RetiredTables::default();
        let mut unmapped = 0;
        let mut complete = true;
        {
            let _ranges = self.ranges.lock();
            for data in sources[..count].iter().flatten() {
                if !attempt.rmap() {
                    complete = false;
                    break;
                }
                // SAFETY:
                // 1. `data` 由 `sources` 数组中的 `Arc<FileRangeData>` 强引用保活，且当前持有 `self.ranges` 自旋锁。
                // 2. `unmap_file_folio` 仅卸载仍指向 `folio.phys_addr()` 的文件后备页表项（不影响私有匿名 COW 页），
                //    且卸下的引用存入 `pages` 和 `tables`，在下方锁外完成 `sync_retired` 全核 TLB 同步后才释放。
                unmapped += unsafe {
                    data.domain.unmap_file_folio(
                        data,
                        start,
                        folio.phys_addr(),
                        &mut pages,
                        &mut tables,
                    )
                };
            }
        }

        self.sync_retired(&pages, &tables);
        drop(pages);
        let freed_tables = tables.release();
        attempt.account(ReclaimStats {
            unmapped,
            freed_tables,
            ..ReclaimStats::default()
        });
        complete
    }

    /// 设置或清除文件页表卸载与 TLB 同步边界的测试观察钩子，旧钩子在自旋锁外释放。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_unmap_hook(&self, hook: Option<FileUnmapHook>) {
        let previous = core::mem::replace(&mut *self.unmap_hook.lock(), hook);
        drop(previous);
    }

    /// 在文件截断收缩（将文件长度减小至 `new_size`）时，清理并移除所有完全超出 `new_size` 的缓存页，
    /// 并作废跨 `new_size` 边界页内超出新文件末尾的完整文件系统逻辑块。
    ///
    /// # Safety
    /// 1. 调用方必须同时持有该 `Inode` 的写互斥锁、失效独占锁（`invalidate`）以及当前 `FileMapping` 的 `writeback` 睡眠锁。
    /// 2. 调用方必须已更新 `Inode` 的 `new_size`，且已对被截断区间执行过第一轮 `even_cows = true` 的 `unmap_range` 与 TLB 同步。
    /// 3. 本函数返回后，调用方仍需在释放底层磁盘块之前执行第二轮 `unmap_range(..., true)`，并负责将跨边界逻辑块的尾部字节清零。
    pub(crate) unsafe fn truncate_pages(
        &self,
        inode: &InodeRef,
        new_size: u64,
        block_size: usize,
    ) -> Result<(), FsError> {
        assert!(
            core::ptr::eq(inode.mapping(), self),
            "truncate mapping owner changed"
        );
        if !matches!(
            block_size,
            FILE_BLOCK_BYTES_1K | FILE_BLOCK_BYTES_2K | FILE_BLOCK_BYTES_4K
        ) {
            return Err(FsError::InvalidOperation);
        }
        // Step 1: 逐个摘除并作废页起始偏移完全位于 `new_size` 之后的整页节点。
        let first = new_size.div_ceil(PAGE_SIZE as u64);
        loop {
            let next = {
                let entries = self.index.entries.lock();
                entries
                    .lower_bound(first)
                    .map(|node| (node.key, node.folio.clone()))
            };
            let Some((index, folio)) = next else { break };
            let folio = folio.with_file_owner(inode.clone());
            let mut content = folio.lock_content();
            folio.wait_writeback();
            let start = index
                .checked_mul(PAGE_SIZE as u64)
                .expect("file page offset overflow");
            self.unmap_range(start..start.saturating_add(PAGE_SIZE as u64), false);
            let detached = {
                let mut entries = self.index.entries.lock();
                assert!(
                    entries
                        .get(index)
                        .is_some_and(|node| { node.folio.phys_addr() == folio.phys_addr() }),
                    "truncate file page identity changed"
                );
                entries.remove(index).unwrap()
            };
            // SAFETY:
            // 1. 调用方持有 `Inode` 写锁、`invalidate` 锁与 `writeback` 锁，阻止了新的缺页加载与回写启动。
            // 2. 上方已获取 `folio.lock_content()`、等待在途回写完成、卸载了所有关联页表项并将节点从索引树摘除，
            //    因此可安全调用 `content.discard()` 直接清除该页的脏状态并回收内存配额。
            unsafe { content.discard() };
            drop(content);
            drop(detached);
            drop(folio);
        }

        // Step 2: 若 `new_size` 落在某个 4KiB 页内部，则清理该尾部半页内完全超出 `new_size` 的文件系统逻辑块脏/有效状态。
        let valid = (new_size % PAGE_SIZE as u64) as usize;
        if valid == 0 {
            return Ok(());
        }
        let index = new_size / PAGE_SIZE as u64;
        let folio = {
            let entries = self.index.entries.lock();
            entries.get(index).map(|node| node.folio.clone())
        };
        if let Some(folio) = folio {
            let folio = folio.with_file_owner(inode.clone());
            let mut content = folio.lock_content();
            folio.wait_writeback();
            self.mkclean(index, &folio, &mut content);
            // SAFETY:
            // 1. 调用方持有截断所需的三把互斥锁，上方已获取 `folio.lock_content()`、等待回写完成并调用 `mkclean` 收回所有用户态写权限。
            // 2. `truncate_file_tail(valid, block_size)` 仅作废偏移 `valid` 之后完整被删的文件系统逻辑块，保留有效前缀数据。
            unsafe { content.truncate_file_tail(valid, block_size) }?;
            self.set_to_write(index, false);
        }
        Ok(())
    }

    /// 检查当前文件索引中的所有驻留页是否均仅由页缓存自身持有（无用户态页表映射、无在途 I/O 或外部借用）。
    pub(crate) fn is_cache_only(&self) -> bool {
        let entries = self.index.entries.lock();
        let mut cursor = 0;
        while let Some(node) = entries.lower_bound(cursor) {
            if !node.folio.is_cache_only() {
                return false;
            }
            let Some(next) = node.key.checked_add(1) else {
                break;
            };
            cursor = next;
        }
        true
    }

    /// 将新的文件映射范围 `data` 按键序插入 `self.ranges`；若底层 `Vec` 容量不足，则在自旋锁外扩容后再交换插入。
    fn register_range(&self, data: &Arc<FileRangeData>) -> Result<(), BlockError> {
        let key = FileRangeData::key(data);
        let mut spare = Vec::new();
        loop {
            let mut ranges = self.ranges.lock();
            if ranges.len() < ranges.capacity() {
                let index = ranges.partition_point(|(existing, _)| *existing < key);
                ranges.insert(index, (key, Arc::downgrade(data)));
                return Ok(());
            }
            let needed = ranges.len().checked_add(1).ok_or(BlockError::NoMemory)?;
            if needed <= spare.capacity() {
                spare.extend(ranges.drain(..));
                core::mem::swap(&mut *ranges, &mut spare);
                let index = ranges.partition_point(|(existing, _)| *existing < key);
                ranges.insert(index, (key, Arc::downgrade(data)));
                drop(ranges);
                drop(spare);
                return Ok(());
            }
            drop(ranges);
            spare
                .try_reserve_exact(needed)
                .map_err(|_| BlockError::NoMemory)?;
        }
    }

    /// 在持有 `folio` 页内容锁（`content`）的前提下，遍历所有覆盖第 `index` 页的共享文件映射（`MAP_SHARED`），
    /// 将用户态页表项降权为只读、汇总硬件脏位到 `content` 的脏扇区掩码中，并在最后执行一次全核 TLB 同步（`sync_all`）。
    ///
    /// 返回本次操作实际清除可写权限的用户态页表项数量。
    pub(crate) fn mkclean(
        &self,
        index: u64,
        folio: &FolioRef,
        content: &mut FolioContent<'_>,
    ) -> usize {
        assert_eq!(content.folio().phys_addr(), folio.phys_addr());
        let start = index
            .checked_mul(PAGE_SIZE as u64)
            .expect("file page offset overflow");
        let end = start
            .checked_add(PAGE_SIZE as u64)
            .expect("file page end overflow");
        let mut cursor: Option<Arc<FileRangeData>> = None;
        let mut cleaned = 0;
        loop {
            let next = {
                let ranges = self.ranges.lock();
                let after = cursor.as_ref().map(FileRangeData::key);
                let position = after.map_or(0, |key| {
                    ranges.partition_point(|(existing, _)| *existing <= key)
                });
                ranges
                    .get(position)
                    .filter(|(key, _)| key.0 < end)
                    .map(|(_, weak)| weak.upgrade().expect("registered file range lost owner"))
            };
            let Some(data) = next else { break };
            if !data.private {
                let (observed, changed) =
                    data.domain.mkclean_range(&data, start, folio.phys_addr());
                cleaned += changed;
                if observed && !data.inode.is_memory() {
                    let size = data
                        .inode
                        .metadata_snapshot()
                        .expect("registered file inode is not loaded")
                        .size;
                    let valid = size.saturating_sub(start).min(PAGE_SIZE as u64) as usize;
                    if 0 < valid {
                        let block_size = data.inode.block_size() as usize;
                        data.inode.note_dirty();
                        content
                            .mark_prepared_dirty(valid, block_size)
                            .expect("prepared file dirty range is invalid");
                    }
                }
            }
            cursor = Some(data);
        }
        // 即使本轮未扫到可写页表项，此前并发的 COW 换页也可能刚清除了写权限但尚未完成全核刷新，因此统一执行 `sync_all()`。
        sync_all();
        cleaned
    }

    /// 扫描当前文件索引，摘除并释放所有无外部引用、无脏数据且无在途回写的干净空闲页，返回清理的页数。
    pub fn prune_unused(&self) -> usize {
        self.index.prune_unused(None)
    }

    /// 返回当前文件索引中驻留的物理页节点数量。
    pub fn len(&self) -> usize {
        self.index.len()
    }

    /// 判断当前文件索引是否为空（没有任何驻留的缓存页）。
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 设置或清除文件第 `index` 页的 `to_write` 本轮回写选中标记。
    pub fn set_to_write(&self, index: u64, selected: bool) {
        self.index.entries.lock().set_to_write(index, selected);
    }

    /// 在半开页号区间 `[start, end)` 内将所有脏页或正在回写的页标记为 `ToWrite`，返回选中的页数。
    pub(crate) fn mark_to_write_range(&self, start: u64, end: u64) -> usize {
        self.index.mark_to_write_range(start, end, usize::MAX).0
    }

    /// 在半开页号区间 `[start, end)` 内将最多 `max_pages` 个脏页或正在回写的页标记为 `ToWrite`，
    /// 返回 `(本轮选中的页数, 下一轮续扫的起始页号)`。
    pub(crate) fn mark_to_write_budget(
        &self,
        start: u64,
        end: u64,
        max_pages: usize,
    ) -> (usize, u64) {
        self.index.mark_to_write_range(start, end, max_pages)
    }

    /// 在半开页号区间 `[start, end)` 内按序筛选匹配 `mark` 状态的文件页填入 `output`，并返回下一轮续扫的起始页号。
    pub(crate) fn select_range(
        &self,
        start: u64,
        end: u64,
        mark: FolioMark,
        output: &mut [Option<(u64, FolioRef)>],
    ) -> Option<u64> {
        self.index.select_range(start, end, mark, output)
    }
}

impl Default for FileMapping {
    /// 创建一个默认的空 `FileMapping` 实例。
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "ktest")]
pub(crate) mod ktest;

/// 执行一轮针对全局磁盘文件页 LRU 的物理内存回收尝试。
///
/// 先执行一轮干净页扫描与回收（`reclaim_pass`）；若首轮未能释放任何文件页或页表页，
/// 且扫描过程中遇到了可读写卷上的脏页，并在当前回收上下文允许等待回写（`attempt.wait_writeback()`）时，
/// 则触发对应 `Ext4Volume` 的一次定量回写推进，待回写完成后再执行第二轮干净页回收。
pub(crate) fn reclaim_file(attempt: &mut ReclaimAttempt) {
    let before = attempt.stats();
    let mut writeback = None;
    reclaim_pass(attempt, &mut writeback);
    let after = attempt.stats();
    if before.freed_file + before.freed_tables < after.freed_file + after.freed_tables {
        return;
    }
    let Some(volume) = writeback else {
        return;
    };
    if !attempt.wait_writeback() {
        return;
    }

    let domain = volume.writeback_domain();
    let boundary = domain.request_progress(&volume);
    let progress = domain.wait_progress(boundary);
    attempt.account(ReclaimStats {
        submitted_bytes: progress.submitted_bytes as usize,
        completed_bytes: progress.completed_bytes as usize,
        io_error: progress.error,
        ..ReclaimStats::default()
    });
    if progress.stopped || progress.error {
        return;
    }
    reclaim_pass(attempt, &mut None);
}

/// 执行单轮全局文件页 LRU 扫描：从 `inactive` 队列隔离候选页并尝试回收，随后对 `active` 队列头部执行定量老化扫描。
fn reclaim_pass(attempt: &mut ReclaimAttempt, writeback: &mut Option<Arc<Ext4Volume>>) {
    const ACTIVE_AGING_BATCH: usize = 16;
    let (inactive, active) = index::lru_lengths();
    let aging = active.min(ACTIVE_AGING_BATCH).min(attempt.scan_remaining());
    for _ in 0..inactive.min(attempt.scan_remaining() - aging) {
        if !attempt.can_isolate() {
            break;
        }
        let Some(node) = index::isolate_inactive(attempt) else {
            continue;
        };
        reclaim_node(node, attempt, writeback);
    }
    index::age_active(attempt, aging);
}

/// 尝试回收单个已从 `inactive` 队列隔离出的文件页节点 `node`：
///
/// - 若所属 `Inode` 正在析构（弱引用升级失败），则直接通过 `putback` 归还或释放节点；
/// - 在常规回收模式（`ReclaimMode::Normal`）下，先卸载指向该页的用户态页表项并确认页面干净，
///   随后在 `entries` 与 `FILE_LRU` 双锁保护下调用 `retire_clean` 原子核验无外部引用后从索引摘除；
/// - 若页面变脏或仍被外部引用而无法回收，则记录待回写卷并通过 `putback` 将节点放回 `inactive` 队尾。
fn reclaim_node(
    node: NodeRef,
    attempt: &mut ReclaimAttempt,
    writeback: &mut Option<Arc<Ext4Volume>>,
) {
    // Step 1: 尝试将节点的弱 Inode 引用升级为强引用；若 Inode 已在销毁中则直接归还或释放节点。
    let Some(inode) = node.role.as_ref().and_then(|role| role.owner().upgrade()) else {
        let freed_file = index::putback(node);
        attempt.account(ReclaimStats {
            freed_file,
            ..ReclaimStats::default()
        });
        return;
    };
    let mapping = inode.mapping();
    let normal = attempt.mode() == ReclaimMode::Normal;
    let mut scanner = None;
    let mut needs_writeback = false;
    // Step 2: 根据回收模式（常规卸载映射 vs 仅限无映射缓存页）检查脏状态、清理页表映射并从索引树原子摘除。
    let retired = if normal {
        (|| {
            let _scope = ReclaimScope::nofs();
            let _invalidate = inode.lock_reclaim_invalidate().ok()?;
            if !mapping.is_canonical(node.key, &node.folio) {
                return None;
            }
            scanner = Some(node.folio.clone().with_file_owner(inode.clone()));
            let folio = scanner.as_ref().unwrap();
            let (dirty, wb) = folio.writeback_state();
            if dirty != 0 || wb != 0 {
                needs_writeback = true;
                return None;
            }
            let mut content = folio.try_lock_content()?;
            if !mapping.reclaim_mappings(&inode, &node, folio, &mut content, attempt) {
                let (dirty, wb) = folio.writeback_state();
                needs_writeback = dirty != 0 || wb != 0;
                return None;
            }
            let mut entries = mapping.index.entries.lock();
            if entries.closed
                || !entries
                    .get(node.key)
                    .is_some_and(|current| core::ptr::eq(current, &*node))
            {
                return None;
            }
            index::with_lru(|| {
                node.folio.retire_clean(Some(folio), Some(&content), || {
                    entries
                        .remove_isolated(node.key)
                        .expect("reclaim canonical disappeared")
                })
            })
        })()
    } else {
        let mut entries = mapping.index.entries.lock();
        if entries.closed
            || !entries
                .get(node.key)
                .is_some_and(|current| core::ptr::eq(current, &*node))
        {
            None
        } else {
            index::with_lru(|| {
                node.folio.retire_clean(None, None, || {
                    entries
                        .remove_isolated(node.key)
                        .expect("cache-only canonical disappeared")
                })
            })
        }
    };

    // Step 3: 若因页面变脏而未能回收，记录所属可读写卷以便后续触发定量回写；最后在锁外释放已摘除节点或将节点放回 LRU。
    if needs_writeback && writeback.is_none() {
        if let Ok(volume) = inode.disk_volume() {
            if volume.mode() == AccessMode::ReadWrite {
                *writeback = Some(Arc::clone(volume));
            }
        }
    }
    let mut freed_file = if let Some(retired) = retired {
        let freed = retired.release();
        freed + index::release_node(node)
    } else {
        index::putback(node)
    };
    if let Some(folio) = scanner {
        freed_file += folio.release();
    }
    attempt.account(ReclaimStats {
        freed_file,
        ..ReclaimStats::default()
    });
}

/// 返回当前目标架构下 `Node` 结构体的字节大小（仅用于内核测试校验元数据预留槽位大小）。
#[cfg(feature = "ktest")]
pub(crate) const fn ktest_node_bytes() -> usize {
    core::mem::size_of::<Node>()
}
