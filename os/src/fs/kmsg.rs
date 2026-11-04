//! 内核日志打开实例只保存独立记录游标，正文和通知仍归唯一日志环。

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

const RECORD_CAPACITY: usize = TEXT_CAPACITY * 4 + 96;
const LOGLEVEL_ERR: i32 = 3;
const LOGLEVEL_WARNING: i32 = 4;
const LOGLEVEL_INFO: i32 = 6;
const LOGLEVEL_DEBUG: i32 = 7;

/// 独立 open 拥有独立序号，复制描述符共享消费串行锁、状态位与末 watch 责任。
pub struct KmsgFile {
    path: Path,
    flags: AtomicU32,
    read: SleepMutex<()>,
    cursor: SpinLock<Cursor>,
    watches: Arc<WatchHead>,
}

/// 一条扩展记录的完整栈文本，容量包含最坏字节转义和固定标量头。
struct RecordText {
    bytes: [u8; RECORD_CAPACITY],
    len: usize,
}

impl RecordText {
    /// 在日志短锁外编码固定头和正文，不复制调用者或 CPU 字段。
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
    /// 固定标量格式化须完整放入栈缓冲，容量不足不返回截断成功。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if RECORD_CAPACITY - self.len < text.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + text.len()].copy_from_slice(text.as_bytes());
        self.len += text.len();
        Ok(())
    }
}

/// 普通可睡眠任务准备只读实例，失败不发表任何 reader 或 watch 登记。
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
    /// 确认空读后登记永久通知源，调用者随后重新扫描而不直接 park。
    fn prepare_wait(&self) -> Result<(Arc<PollContext>, PollTable), FsError> {
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let mut table = PollTable::new(sink, 1)?;
        self.poll(Some(&mut table))?;
        Ok((context, table))
    }
}

impl Pollable for KmsgFile {
    /// 先登记源再查询本实例序号；覆盖报告读就绪和缺口，空游标不造默认可读。
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
    /// 每次先消费一个完整记录再交付；小缓冲或用户故障不回退有效记录序号。
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
                    if 0 < next.lost {
                        return Err(FsError::BrokenPipe);
                    }
                    let text = RecordText::encode(&next.record);
                    if target.len() - target.copied() < text.len {
                        return Err(FsError::InvalidOperation);
                    }
                    // 只在可睡眠串行锁内 uaccess，cursor 与 RING 的短锁已结束。
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
            // park 前释放消费锁；醒来结束唯一 Waiter 后再竞争串行锁和复查序号。
            if let Some((context, _)) = &waiting {
                interrupted = context.park_until(matches!(caller, FileIoContext::User(_)), None)
                    == WaitResult::Interrupted;
            } else {
                waiting = Some(self.prepare_wait()?);
            }
        }
    }

    /// 记录流不授予显式字节位置读取能力。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 真实设备路径保活节点及挂载，属性不由正文副本制造。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 记录流没有普通文件页映射来源。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 临时 poll 和持久 epoll 都借用本 OFD 的真实游标判断。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    /// 同一 OFD 的所有复制引用共享原反向 watch 链。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        Some(&self.watches)
    }

    /// 原子状态属于打开实例，不包含描述符 CLOEXEC。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 只改变现行公共状态位，不改变只读访问模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 只原子修改本日志 OFD 的非阻塞位，不改变记录游标和追加状态。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 只读日志节点不允许截断环中记录。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::ReadOnly)
    }

    /// 记录读取没有持久化提交能力。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 只接受零位移 rewind 或跳到当前末尾，序号变化与同 OFD 消费串行。
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

    /// 返回原字符设备节点的身份、权限及实际 rdev。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 日志记录不提供目录枚举。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 日志设备没有目录交付能力。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 只读打开实例不消费任何写入端字节。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }

    /// 记录流按位置写入先报告无位置能力。
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
    /// 末 OFD 同步撤去自己的持久 watch，永久日志源继续归服务拥有。
    fn drop(&mut self) {
        self.watches.release();
    }
}
