//! 内核跟踪快照字符设备（`/dev/ktrace`）的文件操作实现。
//!
//! # 模块职责与架构定位
//! 本模块为内核运行时跟踪子系统（`crate::trace`）提供用户态文件接口。
//! - **上游调用者**：VFS 特殊设备打开入口（`crate::fs`）及系统调用层（`read`、`pread`、`lseek`、`ioctl`、`fstat`）。
//! - **下游依赖**：`crate::trace` 提供的全局跟踪启停控制（`start`、`stop`、`enabled`）与文本导出（`snapshot`）。
//!
//! # 核心设计与快照生命周期
//! 每个通过 `open` 创建的打开文件描述（Open File Description, `TraceFile`）维护独立的快照缓存与读取游标：
//!
//! ```text
//!   open("/dev/ktrace", O_RDONLY)
//!         │
//!         ▼
//! ┌──────────────────────────┐
//! │ bytes: None, cursor: 0   │ ── ioctl(BLOSSOM_TRACE_START / STOP) ──► 控制全局 trace 状态
//! └──────────────────────────┘
//!         │
//!         │ 首次 read / pread（要求 trace::enabled() == false，否则返回 EBUSY）
//!         ▼
//! ┌──────────────────────────┐
//! │ bytes: Some(Vec<u8>)     │ ── 后续 read / pread / lseek / fstat 均访问该固定文本快照，
//! │ cursor: 随顺序读取推进    │    即使全局再次启动新一轮捕获，也不会覆盖本实例已固定的快照
//! └──────────────────────────┘
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `TraceFile::snapshot`（`SleepMutex<TraceSnapshot>`）：保护本打开实例的快照字节数组与顺序读取游标。
//! - 持 `snapshot` 睡眠锁期间仅执行内存拷贝或调用 `trace::snapshot()` 生成文本，不获取其他文件锁或任务图锁。

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

/// 启动全局内核跟踪捕获的 `ioctl` 命令字。
const BLOSSOM_TRACE_START: u32 = 0x4201;
/// 停止全局内核跟踪捕获的 `ioctl` 命令字。
const BLOSSOM_TRACE_STOP: u32 = 0x4202;

/// 单个打开文件描述（OFD）持有的跟踪文本快照与读取游标状态。
///
/// 由 `TraceFile::snapshot` 睡眠锁保护，使延迟快照生成与游标推进保持串行化。
struct TraceSnapshot {
    /// 停止跟踪后首次读取时捕获的完整格式化文本（单位：字节）。
    /// `None` 表示尚未执行过读取，`Some`（含空 `Vec`）表示已固定快照。
    bytes: Option<Vec<u8>>,
    /// 当前顺序读取的字节偏移量（单位：字节）。
    cursor: u64,
}

/// `/dev/ktrace` 跟踪设备的打开文件实例。
///
/// 停止跟踪后首次读取生成的文本归本实例独占；通过 `dup` 或 `fork` 共享同一打开文件描述的描述符共用该快照与游标。
pub struct TraceFile {
    /// 打开时对应的设备节点路径，用于 `fstat` 和路径查询。
    path: Path,
    /// 打开文件状态标志（如 `O_RDONLY`、`O_NONBLOCK`，不含文件描述符级的 `FD_CLOEXEC`）。
    flags: AtomicU32,
    /// 保护快照缓冲区与顺序读取游标的睡眠互斥锁。
    snapshot: SleepMutex<TraceSnapshot>,
}

/// 打开 `/dev/ktrace` 设备并返回只读文件实例。
///
/// 在普通可睡眠任务上下文中调用。仅接受只读且非截断模式（`O_RDONLY` 且未设置 `O_TRUNC`），
/// 打开操作本身不会启动或停止全局跟踪，也不会立即抓取快照。
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
    /// 从指定字节偏移读取跟踪快照内容到目标缓冲区。
    ///
    /// 调用方必须已持有 `snapshot` 睡眠锁。
    /// - 若全局跟踪仍在运行（`trace::enabled()` 为真），返回 `FsError::Busy`。
    /// - 若当前打开实例尚未生成快照，则在首次读取时调用 `trace::snapshot()` 抓取并固定文本。
    /// - 向用户缓冲区拷贝中途若发生缺页错误，只要已成功拷贝了部分字节（Partial Progress），就返回已拷贝的字节数。
    fn read_snapshot(
        snapshot: &mut TraceSnapshot,
        offset: u64,
        target: &mut ReadBuffer<'_>,
    ) -> Result<usize, FsError> {
        // Step 1: 检查全局跟踪是否已停止，运行期间禁止导出快照。
        if trace::enabled() {
            return Err(FsError::Busy);
        }
        // Step 2: 首次读取时抓取全局跟踪缓冲区并冻结为本实例的私有文本。
        if snapshot.bytes.is_none() {
            snapshot.bytes = Some(trace::snapshot()?);
        }

        let bytes = snapshot
            .bytes
            .as_ref()
            .expect("trace snapshot missing after capture");
        // Step 3: 检查读取偏移是否已越过快照末尾，或目标缓冲区已满。
        if bytes.len() as u64 <= offset || target.copied() == target.len() {
            return Ok(0);
        }

        // Step 4: 将快照字节写入目标缓冲区；若发生错误但已有成功写入的前缀字节，则返回已写入长度。
        let start = target.copied();
        match target.push(&bytes[offset as usize..]) {
            Ok(count) => Ok(count),
            Err(error) if target.copied() == start => Err(error),
            Err(_) => Ok(target.copied() - start),
        }
    }
}

impl FileLike for TraceFile {
    /// 标识该文件支持显式偏移量读取（`pread`）与 `lseek` 定位。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 从当前游标位置顺序读取跟踪快照，并按实际交付的字节数推进游标。
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

    /// 从指定字节偏移 `offset` 处读取跟踪快照（`pread`），不修改顺序读取游标。
    fn read_at_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        Self::read_snapshot(&mut self.snapshot.lock(), offset, target)
    }

    /// 返回打开该设备时的挂载路径。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 跟踪快照保存在内核堆缓冲区中，不支持 `mmap` 页缓存映射。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 处理 `/dev/ktrace` 专用的跟踪启停控制命令（`BLOSSOM_TRACE_START` / `BLOSSOM_TRACE_STOP`）。
    ///
    /// 若调用方来自用户态任务，必须具备 root 权限（`uid == 0`），否则返回 `FsError::NotPermitted`。
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

    /// 读取当前打开文件描述的状态标志（原子读取，不含描述符级标志）。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新打开文件描述的可变状态标志（如 `F_SETFL`）。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除本打开文件实例的非阻塞标志（`O_NONBLOCK`）。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 跟踪快照为只读设备，拒绝截断操作并返回 `FsError::ReadOnly`。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::ReadOnly)
    }

    /// 内存跟踪快照无底层持久化存储，不支持 `fsync` / `fdatasync`。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 调整顺序读取游标位置（`lseek`）。
    ///
    /// - 若尚未读取并生成快照，仅允许重置到文件开头（`SeekFrom::Start(0)`），其余定位方式返回 `FsError::IllegalSeek`。
    /// - 若已生成快照，则按快照字节长度计算目标偏移，并校验结果在非负 `loff_t`（`0..=i64::MAX`）范围内。
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

    /// 获取设备状态信息（`fstat`），其中 `size` 字段反映本实例已抓取的快照字节数（未抓取时为 0）。
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

    /// 字符设备非目录，不支持读取目录项。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 字符设备非目录，不支持遍历目录项。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 跟踪快照固定以只读模式打开，写入操作一律按描述符不可写返回 `FsError::BadFd`。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }

    /// 跟踪快照固定以只读模式打开，指定偏移写入（`pwrite`）一律返回 `FsError::BadFd`。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }
}
