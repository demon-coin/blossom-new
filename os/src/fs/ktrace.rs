//! 每个打开实例在停止后首次读取时保存自己的文本，后继捕获不替换旧快照。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::fs::file::{require_read, set_nonblocking, set_status_flags, stat_path};
use crate::fs::{
    DirectoryEntry, FileIoContext, FileLike, FileRef, FileStat, FsError, InodeRef, Path,
    ReadBuffer, SeekFrom, WriteBuffer, O_ACCMODE, O_RDONLY, O_TRUNC,
};
use crate::sync::SleepMutex;
use crate::trace;

const BLOSSOM_TRACE_START: u32 = 0x4201;
const BLOSSOM_TRACE_STOP: u32 = 0x4202;

/// 本 OFD 的快照和字节位置一起串行，未读取与已取得空文本保持可区分。
struct TraceSnapshot {
    bytes: Option<Vec<u8>>,
    cursor: u64,
}

/// 停止后的拥有文本只归本打开实例，复制描述符共享它和读取位置。
pub struct TraceFile {
    path: Path,
    flags: AtomicU32,
    snapshot: SleepMutex<TraceSnapshot>,
}

/// 普通可睡眠任务准备只读实例，打开本身不启动捕获或取得新快照。
pub(crate) fn open(path: Path, flags: u32) -> Result<FileRef, FsError> {
    if flags & O_ACCMODE != O_RDONLY || flags & O_TRUNC != 0 {
        return Err(FsError::ReadOnly);
    }
    Ok(Arc::try_new(TraceFile {
        path,
        flags: AtomicU32::new(flags),
        snapshot: SleepMutex::new(TraceSnapshot {
            bytes: None,
            cursor: 0,
        }),
    })
    .map_err(|_| FsError::NoMemory)?)
}

impl TraceFile {
    /// 采中一律返回 Busy；停稳后首次取得完整文本，用户故障保留实际交付前缀。
    fn read_snapshot(
        snapshot: &mut TraceSnapshot,
        offset: u64,
        target: &mut ReadBuffer<'_>,
    ) -> Result<usize, FsError> {
        if trace::enabled() {
            return Err(FsError::Busy);
        }
        if snapshot.bytes.is_none() {
            snapshot.bytes = Some(trace::snapshot()?);
        }

        let bytes = snapshot
            .bytes
            .as_ref()
            .expect("trace snapshot missing after capture");
        if bytes.len() as u64 <= offset || target.copied() == target.len() {
            return Ok(0);
        }

        let start = target.copied();
        match target.push(&bytes[offset as usize..]) {
            Ok(count) => Ok(count),
            Err(error) if target.copied() == start => Err(error),
            Err(_) => Ok(target.copied() - start),
        }
    }
}

impl FileLike for TraceFile {
    /// 稳定文本允许显式字节位置读取，不授予只读实例写权限。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 读取和复制描述符共享位置，只按实际交付量推进。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        let mut snapshot = self.snapshot.lock();
        let offset = snapshot.cursor;
        let count = Self::read_snapshot(&mut snapshot, offset, target)?;
        snapshot.cursor += count as u64;
        Ok(count)
    }

    /// 显式位置读取同一本 OFD 的文本，但不改变顺序读取位置。
    fn read_at_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        Self::read_snapshot(&mut self.snapshot.lock(), offset, target)
    }

    /// 保留真实设备节点路径及原挂载寿命。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 拥有文本不提供普通文件页映射来源。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 无指针控制只核本次调用者的真实 uid，命令不保存到 OFD 快照状态。
    fn ioctl(
        &self,
        command: u32,
        _argument: usize,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if !matches!(command, BLOSSOM_TRACE_START | BLOSSOM_TRACE_STOP) {
            return Err(FsError::NotTty);
        }
        if let FileIoContext::User(user) = caller {
            let runtime = user.runtime().ok_or(FsError::InvalidOperation)?;
            if runtime.credentials.uid != 0 {
                return Err(FsError::NotPermitted);
            }
        }

        match command {
            BLOSSOM_TRACE_START => trace::start()?,
            BLOSSOM_TRACE_STOP => trace::stop()?,
            _ => unreachable!("validated trace command changed"),
        }
        Ok(0)
    }

    /// 原子状态属于打开实例，不包含描述符 CLOEXEC。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 只改变公共协议允许的状态，不改变只读模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 只原子修改本快照 OFD 的非阻塞位，不改变快照内容和追加状态。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 只读快照不能截断生产记录或本实例文本。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::ReadOnly)
    }

    /// 快照没有可提交的持久化操作。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 已有文本按非负 loff_t 定位；首次读取前只允许 rewind，不查询未知捕获长度。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError> {
        let mut snapshot = self.snapshot.lock();
        let Some(bytes) = &snapshot.bytes else {
            if from == SeekFrom::Start(0) {
                snapshot.cursor = 0;
                return Ok(0);
            }
            return Err(FsError::IllegalSeek);
        };
        let position = match from {
            SeekFrom::Start(position) => Some(position),
            SeekFrom::Current(delta) => snapshot.cursor.checked_add_signed(delta),
            SeekFrom::End(delta) => (bytes.len() as u64).checked_add_signed(delta),
        }
        .ok_or(FsError::InvalidOperation)?;
        if (i64::MAX as u64) < position {
            return Err(FsError::InvalidOperation);
        }

        snapshot.cursor = position;
        Ok(position)
    }

    /// 原节点提供身份及 rdev，长度只反映本实例已经拥有的文本。
    fn stat(&self) -> Result<FileStat, FsError> {
        let mut stat = stat_path(&self.path)?;
        let snapshot = self.snapshot.lock();
        stat.size = snapshot
            .bytes
            .as_ref()
            .map_or(0, |bytes| bytes.len() as u64);
        stat.blocks = 0;
        Ok(stat)
    }

    /// 文本快照不提供目录枚举。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 快照 OFD 没有目录交付能力。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 只读实例即使空写也拒绝描述符写模式，不访问输入字节。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }

    /// 显式位置写同样不消费输入端。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }
}
