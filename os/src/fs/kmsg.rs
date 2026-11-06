//! `/dev/kmsg` 内核日志字符设备打开文件描述（OFD）实现。
//!
//! ## 模块职责与架构定位
//! 本模块为 `/dev/kmsg` 节点提供只读记录流消费接口（[`KmsgFile`]）。
//! 上游由虚拟文件系统的打开路径（当打开 `MemorySpecial::Kmsg` 节点时）调用 [`open`] 创建实例，
//! 下游依赖全局内核日志环形缓冲区（[`crate::klog`]）读取结构化日志记录并订阅新增日志唤醒通知，
//! 同时对接 `poll` / `epoll` 事件多路复用子系统（[`crate::fs::poll`]、[`crate::fs::epoll`]）。
//!
//! ## 核心设计与游标消费流程
//! 每个通过 `open("/dev/kmsg", ...)` 创建的 [`KmsgFile`] 独立维护自己的日志序列号游标（[`Cursor`]）；
//! 而通过 `dup` / `fork` 共享同一个 [`KmsgFile`] 的多个文件描述符则共享该游标与读取互斥锁。
//!
//! ```text
//! KmsgFile::read_into 消费状态机:
//!
//!   acquire `read` (SleepMutex)
//!       │
//!       ▼
//!   acquire `cursor` (SpinLock) ──► cursor.read() 从 klog 环形缓冲区拉取下一条记录
//!       │
//!       ├─► Some(next) 且 next.lost > 0 (游标落后的记录已被环形缓冲区覆盖):
//!       │       重置 cursor 到 next.record.seq，释放锁并返回 Err(BrokenPipe / EPIPE)
//!       │
//!       ├─► Some(next) 且 next.lost == 0:
//!       │       释放 cursor 自旋锁 ──► RecordText::encode 在栈上格式化为 "<prio>,<seq>,<usec>,-;<escaped>\n"
//!       │       ──► 在 read 睡眠锁保护下拷贝到用户缓冲区并返回字节数
//!       │
//!       └─► None (当前无未读日志):
//!               ├─ 若设置了 O_NONBLOCK ──► 返回 Err(WouldBlock / EAGAIN)
//!               └─ 阻塞模式 ──► 释放 `read` 锁，向 `klog::reader_source()` 注册等待并休眠，唤醒后重试
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 1. `KmsgFile::read`（[`SleepMutex`]）：最外层可睡眠互斥锁，串行化同一打开实例上的 `read_into` 与 `seek` 操作，允许在持锁期间执行用户态内存拷贝（可能触发缺页睡眠）。
//! 2. `KmsgFile::cursor`（[`SpinLock`]）：内层自旋锁，保护日志游标 [`Cursor`]；在 `cursor.read()` / `cursor.available()` 内部会短暂获取全局日志环自旋锁。**严禁**在持有 `cursor` 自旋锁期间执行用户空间拷贝或任何可能睡眠的操作。

use alloc::sync::Arc;
use core::fmt::{self, Write};
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::fs::epoll::WatchHead;
use crate::fs::file::{require_read, set_nonblocking, set_status_flags, stat_path};
use crate::fs::poll::{PollContext, PollMask, PollNotify, PollTable, Pollable};
use crate::fs::{
    DirectoryEntry, FileIoContext, FileLike, FileRef, FileStat, FsError, InodeRef, Path,
    ReadBuffer, SeekFrom, WriteBuffer, O_ACCMODE, O_NONBLOCK, O_RDONLY, O_TRUNC,
};
use crate::klog::{self, Cursor, Level, Record, TEXT_CAPACITY};
use crate::sync::{SleepMutex, SpinLock};
use crate::wait::WaitResult;

/// 单条格式化后 `/dev/kmsg` 文本记录的最大栈缓冲区字节数。
///
/// 计算依据：正文最大长度 [`TEXT_CAPACITY`] 中每个字节最坏转义为 `\xNN`（4 倍膨胀），
/// 外加固定头部 `<priority>,<seq>,<timestamp_us>,-;` 与结尾换行符预留的 96 字节。
const RECORD_CAPACITY: usize = TEXT_CAPACITY * 4 + 96;

/// Linux syslog / `/dev/kmsg` 错误级别数值（`LOG_ERR = 3`）。
const LOGLEVEL_ERR: i32 = 3;
/// Linux syslog / `/dev/kmsg` 警告级别数值（`LOG_WARNING = 4`）。
const LOGLEVEL_WARNING: i32 = 4;
/// Linux syslog / `/dev/kmsg` 信息级别数值（`LOG_INFO = 6`）。
const LOGLEVEL_INFO: i32 = 6;
/// Linux syslog / `/dev/kmsg` 调试级别数值（`LOG_DEBUG = 7`）。
const LOGLEVEL_DEBUG: i32 = 7;

/// `/dev/kmsg` 字符设备的打开文件描述（Open File Description）。
///
/// 每次独立的 `open` 系统调用都会创建一个拥有独立日志序列号游标的 [`KmsgFile`]；
/// 通过 `dup` 或 `fork` 复制出的文件描述符共享同一个 [`KmsgFile`] 实例、读取串行锁、状态标志以及 `epoll` 监视链表。
pub struct KmsgFile {
    /// 打开该设备时的挂载路径与目录项引用，用于 `fstat` 查询属性及维持挂载点引用计数。
    path: Path,
    /// 打开文件状态标志（如 `O_RDONLY`、`O_NONBLOCK`），使用原子变量支持无锁读取与修改。
    flags: AtomicU32,
    /// 串行化同一打开实例上并发 `read` 与 `seek` 操作的可睡眠互斥锁。
    read: SleepMutex<()>,
    /// 保护当前日志消费序列号游标的自旋锁（持锁时间极短，不可跨用户态内存拷贝）。
    cursor: SpinLock<Cursor>,
    /// 挂载在此打开文件实例上的 `epoll` 反向监视链表头，最后一个强引用释放时自动清理。
    watches: Arc<WatchHead>,
}

/// 单条 `/dev/kmsg` 扩展格式日志的栈上编码缓冲区。
struct RecordText {
    /// 存放格式化后 ASCII 文本的固定大小字节数组。
    bytes: [u8; RECORD_CAPACITY],
    /// 当前已写入 `bytes` 的有效字节长度（单位：字节，满足 `len <= RECORD_CAPACITY`）。
    len: usize,
}

impl RecordText {
    /// 将一条内核日志记录 [`Record`] 编码为 `/dev/kmsg` 扩展文本格式。
    ///
    /// 在日志环形缓冲区自旋锁释放后调用。输出格式为：
    /// `<priority>,<seq>,<monotonic_usec>,-;<escaped_text>\n`，其中不可打印字符与反斜杠编码为 `\xNN`。
    fn encode(record: &Record) -> Self {
        let mut text = Self {
            bytes: [0; RECORD_CAPACITY],
            len: 0,
        };
        let priority = match record.level {
            Level::Error => LOGLEVEL_ERR,
            Level::Warn => LOGLEVEL_WARNING,
            Level::Info => LOGLEVEL_INFO,
            Level::Debug => LOGLEVEL_DEBUG,
        };
        write!(
            text,
            "{},{},{},-;",
            priority,
            record.seq,
            record.monotonic_ns / crate::time::NSEC_PER_USEC
        )
        .expect("kmsg header exceeded fixed capacity");

        const HEX: &[u8; 16] = b"0123456789abcdef";
        for &byte in record.text().as_bytes() {
            if !(32..127).contains(&byte) || byte == b'\\' {
                let escaped = [
                    b'\\',
                    b'x',
                    HEX[(byte >> 4) as usize],
                    HEX[(byte & 15) as usize],
                ];
                text.bytes[text.len..text.len + escaped.len()].copy_from_slice(&escaped);
                text.len += escaped.len();
            } else {
                text.bytes[text.len] = byte;
                text.len += 1;
            }
        }

        text.bytes[text.len] = b'\n';
        text.len += 1;
        text
    }
}

impl Write for RecordText {
    /// 向固定容量的栈缓冲区追加字符串片段；若剩余空间不足则返回 `fmt::Error`，拒绝截断写入。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if RECORD_CAPACITY - self.len < text.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + text.len()].copy_from_slice(text.as_bytes());
        self.len += text.len();
        Ok(())
    }
}

/// 打开 `/dev/kmsg` 设备节点，创建只读的 [`KmsgFile`] 实例。
///
/// - 调用上下文：普通可睡眠任务上下文。
/// - 参数：
///   - `path`：设备节点的解析路径。
///   - `flags`：打开标志位；必须为只读模式（`O_RDONLY`）且不得包含 `O_TRUNC`，否则返回 [`FsError::ReadOnly`]。
/// - 返回值：成功返回封装后的 [`FileRef`]；内存不足返回 [`FsError::NoMemory`]。
pub(crate) fn open(path: Path, flags: u32) -> Result<FileRef, FsError> {
    if flags & O_ACCMODE != O_RDONLY || flags & O_TRUNC != 0 {
        return Err(FsError::ReadOnly);
    }
    let watches = Arc::try_new(WatchHead::new()).map_err(|_| FsError::NoMemory)?;
    Ok(Arc::try_new(KmsgFile {
        path,
        flags: AtomicU32::new(flags),
        read: SleepMutex::new(()),
        cursor: SpinLock::new(Cursor::oldest()),
        watches,
    })
    .map_err(|_| FsError::NoMemory)?)
}

impl KmsgFile {
    /// 为阻塞读取准备一次性轮询等待上下文（[`PollContext`]）与轮询登记表（[`PollTable`]），
    /// 并将其注册到全局内核日志通知源上。调用方在注册后需再次检查游标以消除唤醒丢失竞态。
    fn prepare_wait(&self) -> Result<(Arc<PollContext>, PollTable), FsError> {
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let mut table = PollTable::new(sink, 1)?;
        self.poll(Some(&mut table))?;
        Ok((context, table))
    }
}

impl Pollable for KmsgFile {
    /// 查询当前游标处是否有可读日志，并可选地将等待者注册到全局内核日志通知源。
    ///
    /// - 若游标处尚无新记录，返回空掩码；
    /// - 若有正常未读记录，返回 `POLLIN | POLLRDNORM`；
    /// - 若游标落后的旧记录已被环形缓冲区覆盖（存在丢失记录），额外置位 `POLLERR | POLLPRI`。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(klog::reader_source())?;
        }
        Ok(match self.cursor.lock().available() {
            None => PollMask::default(),
            Some(0) => PollMask::POLLIN | PollMask::POLLRDNORM,
            Some(_) => {
                PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLERR | PollMask::POLLPRI
            }
        })
    }
}

impl FileLike for KmsgFile {
    /// 从当前游标位置读取下一条完整的内核日志记录。
    ///
    /// - 调用上下文：普通可睡眠任务上下文。
    /// - 行为说明：
    ///   1. 每次调用最多消费并格式化一条日志记录；
    ///   2. 若游标指向的旧记录已被环形缓冲区覆盖，将游标快进到当前最老可用记录并返回 [`FsError::BrokenPipe`]（对应 `EPIPE`）；
    ///   3. 若用户接收缓冲区剩余容量不足以容纳整条格式化后的记录，返回 [`FsError::InvalidOperation`]（对应 `EINVAL`）；
    ///   4. 若当前无可读记录，非阻塞模式返回 [`FsError::WouldBlock`]，阻塞模式休眠等待新日志写入。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        let mut interrupted = false;
        loop {
            if let Some((context, _)) = &waiting {
                context.clear_triggered();
            }
            // Step 1: 获取串行读取睡眠锁，并在短自旋锁内尝试从日志环拉取一条记录。
            {
                let _read = self.read.lock();
                let next = {
                    let mut cursor = self.cursor.lock();
                    let next = cursor.read();
                    if let Some(next) = &next {
                        if 0 < next.lost {
                            *cursor = Cursor::from_sequence(next.record.seq);
                        }
                    }
                    next
                };
                if let Some(next) = next {
                    // Step 2: 若存在记录丢失（被环形缓冲区覆盖），游标已在上方快进，本次向调用者报告 EPIPE。
                    if 0 < next.lost {
                        return Err(FsError::BrokenPipe);
                    }
                    // Step 3: 在自旋锁外将记录格式化到栈缓冲区，并校验用户缓冲区容量。
                    let text = RecordText::encode(&next.record);
                    if target.len() - target.copied() < text.len {
                        return Err(FsError::InvalidOperation);
                    }
                    // Step 4: 仅在持有可睡眠 `read` 锁的情况下将数据拷贝至用户空间（此时 `cursor` 与日志环自旋锁均已释放）。
                    let copied = target.push(&text.bytes[..text.len])?;
                    if copied < text.len {
                        return Err(FsError::BadAddress);
                    }
                    return Ok(copied);
                }
                if self.flags() & O_NONBLOCK != 0 {
                    return Err(FsError::WouldBlock);
                }
                if interrupted {
                    return Err(FsError::Interrupted);
                }
            }

            // Step 5: 进入休眠等待前先释放 `read` 睡眠锁；首次为空时先注册通知源再立即复查一次，确认仍无数据才挂起当前线程。
            if let Some((context, _)) = &waiting {
                interrupted = context.park_until(matches!(caller, FileIoContext::User(_)), None)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// `/dev/kmsg` 为顺序记录流，不支持按指定字节偏移读取（`pread`），始终返回 [`FsError::IllegalSeek`]。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 返回打开该设备时的 [`Path`]，用于查询底层字符设备 Inode 属性并維持挂载点生命周期。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// `/dev/kmsg` 为特殊字符设备流，不支持 `mmap` 页缓存映射，返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 返回 `Pollable` trait 对象引用，以支持 `poll` / `select` / `epoll` 事件监听。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    /// 返回该打开文件实例关联的 `epoll` 反向监视链表头。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        Some(&self.watches)
    }

    /// 原子读取当前打开文件实例的状态标志位（不含文件描述符表级别的 `FD_CLOEXEC`）。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新 `fcntl(F_SETFL)` 允许修改的文件状态标志位，不影响只读访问模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除当前打开实例的 `O_NONBLOCK` 非阻塞标志位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// `/dev/kmsg` 为只读字符设备，不支持截断操作，始终返回 [`FsError::ReadOnly`]。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::ReadOnly)
    }

    /// `/dev/kmsg` 为内存日志流，不支持 `fsync` / `fdatasync` 持久化同步，返回 [`FsError::InvalidOperation`]。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 重置日志读取游标位置，与同一打开实例的 `read_into` 操作通过 `read` 锁串行化。
    ///
    /// - `SeekFrom::Start(0)`（`SEEK_SET, 0`）：将游标重置到当前环形缓冲区中最老的一条记录；
    /// - `SeekFrom::End(0)`（`SEEK_END, 0`）：将游标跳到当前最新记录之后（仅读取后续新产生的日志）；
    /// - `SeekFrom::Current(0)` 返回 [`FsError::InvalidOperation`]，其他非零偏移返回 [`FsError::IllegalSeek`]。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError> {
        let _read = self.read.lock();
        let mut cursor = self.cursor.lock();
        *cursor = match from {
            SeekFrom::Start(0) => Cursor::oldest(),
            SeekFrom::End(0) => Cursor::from_sequence(klog::next_sequence()),
            SeekFrom::Current(0) => return Err(FsError::InvalidOperation),
            _ => return Err(FsError::IllegalSeek),
        };
        Ok(0)
    }

    /// 查询底层 `/dev/kmsg` 字符设备节点的元数据（Inode 编号、权限模式及设备号 `rdev`）。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 字符设备不支持目录项读取，始终返回 [`FsError::NotDirectory`]。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 字符设备不支持目录遍历，始终返回 [`FsError::NotDirectory`]。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// `/dev/kmsg` 打开实例固定为只读，任何写入尝试均返回 [`FsError::BadFd`]。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }

    /// `/dev/kmsg` 不支持按偏移位置写入（`pwrite`），始终返回 [`FsError::IllegalSeek`]。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
}

impl Drop for KmsgFile {
    /// 当最后一个指向本打开文件实例的 `Arc` 引用被释放时，通知关联的 `epoll` 监视项解绑；
    /// 全局内核日志环的通知源继续保留。
    fn drop(&mut self) {
        self.watches.release();
    }
}
