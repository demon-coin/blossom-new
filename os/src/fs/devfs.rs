//! `devfs` 设备文件系统初始化与内置虚拟字符设备实现。
//!
//! ## 模块职责与架构定位
//! 本模块负责在内存文件系统（[`MemoryFs`]）之上构建 `/dev` 目录树（[`try_new`]），
//! 并直接实现不依赖外部硬件驱动的三个虚拟字符设备打开文件描述（[`NullFile`]、[`ZeroFile`]、[`TestHookFile`]）。
//! 上游调用者为 VFS 挂载与打开路径（`Vfs` 初始化挂载 `/dev`，打开节点时根据 [`MemorySpecial`] 分派到本模块或对应的终端/日志模块）；
//! 下游依赖 [`MemoryFs`] 管理目录项与设备节点 Inode，以及 [`crate::contest_hooks`] 执行评测阶段钩子操作。
//!
//! ## `/dev` 固定节点与设备号（`rdev`）布局
//! 设备号编码遵循 Linux `(major << 8) | minor` 约定：
//!
//! ```text
//! /dev (MemoryFsKind::Devfs)
//!  ├── null     (1, 3)   mode 0666 ──► MemorySpecial::Null           ──► devfs::NullFile
//!  ├── zero     (1, 5)   mode 0666 ──► MemorySpecial::Zero           ──► devfs::ZeroFile
//!  ├── ttyS0    (4, 64)  mode 0666 ──► MemorySpecial::TtySerial      ──► tty 子系统
//!  ├── console  (5, 1)   mode 0666 ──► MemorySpecial::Console        ──► tty 子系统
//!  ├── tty      (5, 0)   mode 0666 ──► MemorySpecial::ControllingTty ──► tty 子系统
//!  ├── kmsg     (1, 11)  mode 0444 ──► MemorySpecial::Kmsg           ──► fs::kmsg::KmsgFile
//!  ├── ktrace   (240, 1) mode 0444 ──► MemorySpecial::Ktrace         ──► ktrace 子系统
//!  └── testhook (240, 2) mode 0200 ──► MemorySpecial::TestHook       ──► devfs::TestHookFile
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `NullFile`、`ZeroFile` 与 `TestHookFile` 内部仅通过 `AtomicU32` 维护打开文件状态标志，不持有任何内容锁。
//! - 在向用户空间读写全零数据或拉取控制消息时（`zeros` / `discard` / `copy_hook_message`），均使用栈上 512 字节临时缓冲区且不持有自旋锁，允许用户空间缺页异常正常获取内存管理锁。
//! - `TestHookFile` 进行权限校验时短暂获取当前线程运行时凭据，并在校验完毕后立即释放，随后才调用可能睡眠的 `contest_hooks::before` / `after`。

use alloc::sync::Arc;
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::contest_hooks;
use crate::fs::file::{require_read, require_write, set_nonblocking, set_status_flags, stat_path};
use crate::fs::memfs::{MemoryFs, MemoryFsKind, MemorySpecial};
use crate::fs::{
    DirectoryEntry, FileIoContext, FileLike, FileRef, FileStat, FsError, InodeRef, Path,
    ReadBuffer, SeekFrom, WriteBuffer, O_ACCMODE, O_WRONLY,
};

/// `/dev/null` 字符设备号：主设备号 1，次设备号 3。
const NULL_RDEV: u64 = (1 << u8::BITS) | 3;
/// `/dev/zero` 字符设备号：主设备号 1，次设备号 5。
const ZERO_RDEV: u64 = (1 << u8::BITS) | 5;
/// `/dev/ttyS0` 串口终端设备号：主设备号 4，次设备号 64。
const SERIAL_RDEV: u64 = (4 << u8::BITS) | 64;
/// `/dev/console` 系统控制台设备号：主设备号 5，次设备号 1。
const CONSOLE_RDEV: u64 = (5 << u8::BITS) | 1;
/// `/dev/tty` 当前进程控制终端设备号：主设备号 5，次设备号 0。
const CONTROLLING_TTY_RDEV: u64 = 5 << u8::BITS;
/// `/dev/kmsg` 内核日志设备号：主设备号 1，次设备号 11。
const KMSG_RDEV: u64 = (1 << u8::BITS) | 11;
/// `/dev/ktrace` 内核追踪设备号：主设备号 240，次设备号 1。
const KTRACE_RDEV: u64 = (240 << u8::BITS) | 1;
/// `/dev/testhook` 评测控制设备号：主设备号 240，次设备号 2。
const TESTHOOK_RDEV: u64 = (240 << u8::BITS) | 2;

/// `/dev/testhook` 单次写入控制命令的最大字节数上限（单位：字节）。
const HOOK_MESSAGE_BYTES: usize = 512;
/// `/dev/null` 与 `/dev/zero` 分块读写用户空间时的栈缓冲区大小（单位：字节）。
const DEVICE_IO_CHUNK: usize = 512;

/// `/dev/null` 空设备的打开文件描述（Open File Description）。
///
/// 读取始终立即返回 EOF（0 字节），写入消费并丢弃全部输入数据；不维护有意义的字节偏移游标。
pub struct NullFile {
    /// 打开该设备时的挂载路径与目录项引用，用于 `fstat` 查询属性及维持挂载点生命周期。
    path: Path,
    /// 打开文件状态标志（如访问模式、`O_NONBLOCK`），供同一打开实例的多个文件描述符原子共享。
    flags: AtomicU32,
}

/// `/dev/zero` 零设备的打开文件描述（Open File Description）。
///
/// 读取时通过栈上固定大小的分块缓冲区向接收端无限填充零字节，写入时丢弃全部输入数据。
pub struct ZeroFile {
    /// 打开该设备时的挂载路径与目录项引用，用于 `fstat` 查询属性及维持挂载点生命周期。
    path: Path,
    /// 打开文件状态标志（如访问模式、`O_NONBLOCK`），供同一打开实例的多个文件描述符原子共享。
    flags: AtomicU32,
}

/// `/dev/testhook` 评测控制设备的打开文件描述（Open File Description）。
///
/// 仅允许 `uid == 0` 的特权上下文以只写模式（`O_WRONLY`）打开并写入完整的单条控制命令（`before <id>\n` 或 `after <id>\n`）。
pub struct TestHookFile {
    /// 打开该设备时的挂载路径与目录项引用，用于 `fstat` 查询属性及维持挂载点生命周期。
    path: Path,
    /// 打开文件状态标志（如 `O_WRONLY`、`O_NONBLOCK`），供同一打开实例的多个文件描述符原子共享。
    flags: AtomicU32,
}

/// 创建并初始化一个新的 `devfs` 内存文件系统实例，在根目录下安装全部内置字符设备节点。
///
/// - 调用上下文：内核初始化或挂载路径（可睡眠上下文）。
/// - 参数：`root_mode` 为 `/dev` 根目录的权限位（如 `0o755`）。
/// - 返回值：成功返回尚未公开名字缓存的 [`Arc<MemoryFs>`]；内存不足返回 [`FsError::NoMemory`]。
pub fn try_new(root_mode: u16) -> Result<Arc<MemoryFs>, FsError> {
    let fs = MemoryFs::try_new(MemoryFsKind::Devfs, root_mode)?;
    let root = fs.root();
    fs.initialize_special(&root, b"null", MemorySpecial::Null, 0o666, NULL_RDEV)?;
    fs.initialize_special(&root, b"zero", MemorySpecial::Zero, 0o666, ZERO_RDEV)?;
    fs.initialize_special(
        &root,
        b"ttyS0",
        MemorySpecial::TtySerial,
        0o666,
        SERIAL_RDEV,
    )?;
    fs.initialize_special(
        &root,
        b"console",
        MemorySpecial::Console,
        0o666,
        CONSOLE_RDEV,
    )?;
    fs.initialize_special(
        &root,
        b"tty",
        MemorySpecial::ControllingTty,
        0o666,
        CONTROLLING_TTY_RDEV,
    )?;
    fs.initialize_special(&root, b"kmsg", MemorySpecial::Kmsg, 0o444, KMSG_RDEV)?;
    fs.initialize_special(&root, b"ktrace", MemorySpecial::Ktrace, 0o444, KTRACE_RDEV)?;
    fs.initialize_special(
        &root,
        b"testhook",
        MemorySpecial::TestHook,
        0o200,
        TESTHOOK_RDEV,
    )?;
    Ok(fs)
}

/// 打开由 `devfs` 直接托管的虚拟字符设备节点（`null`、`zero`、`testhook`）。
///
/// - 调用上下文：普通可睡眠任务上下文。
/// - 参数：
///   - `path`：目标设备节点的解析路径。
///   - `flags`：打开文件状态标志；对于字符设备，`O_TRUNC` 标志被忽略；`TestHook` 节点强制要求 `O_WRONLY`。
///   - `caller`：发起打开请求的 I/O 上下文，`TestHook` 节点要求用户态调用者 `uid == 0`。
///   - `special`：目标 Inode 标记的特殊节点类型。
/// - 返回值：成功返回对应的 [`FileRef`]；权限不足返回 [`FsError::NotPermitted`]，模式不合法返回 [`FsError::InvalidOperation`]。
pub fn open(
    path: Path,
    flags: u32,
    caller: FileIoContext<'_>,
    special: MemorySpecial,
) -> Result<FileRef, FsError> {
    match special {
        MemorySpecial::Null => Ok(Arc::try_new(NullFile {
            path,
            flags: AtomicU32::new(flags),
        })
        .map_err(|_| FsError::NoMemory)?),
        MemorySpecial::Zero => Ok(Arc::try_new(ZeroFile {
            path,
            flags: AtomicU32::new(flags),
        })
        .map_err(|_| FsError::NoMemory)?),
        MemorySpecial::TestHook => {
            if flags & O_ACCMODE != O_WRONLY {
                return Err(FsError::InvalidOperation);
            }
            require_hook_control(caller)?;
            Ok(Arc::try_new(TestHookFile {
                path,
                flags: AtomicU32::new(flags),
            })
            .map_err(|_| FsError::NoMemory)?)
        }
        _ => Err(FsError::InvalidOperation),
    }
}

/// 校验调用方是否具备操作 `/dev/testhook` 的 root 权限（内核上下文直接放行，用户上下文要求 `uid == 0`）。
fn require_hook_control(caller: FileIoContext<'_>) -> Result<(), FsError> {
    if let FileIoContext::User(user) = caller {
        let runtime = user.runtime().ok_or(FsError::InvalidOperation)?;
        if runtime.credentials.uid != 0 {
            return Err(FsError::NotPermitted);
        }
    }
    Ok(())
}

/// 将单次写入请求完整拉取到栈上缓冲区 `bytes` 中。
///
/// 要求请求长度在 `1..=HOOK_MESSAGE_BYTES`（512 字节）之间且一次性完整拷贝；
/// 空写入、超长消息或拷贝中途提前结束均返回 [`FsError::InvalidOperation`]。
fn copy_hook_message(
    source: &mut WriteBuffer<'_>,
    bytes: &mut [u8; HOOK_MESSAGE_BYTES],
) -> Result<usize, FsError> {
    let length = source.len();
    if length == 0 || bytes.len() < length {
        return Err(FsError::InvalidOperation);
    }

    let mut copied = 0;
    while copied < length {
        let count = source.pull(&mut bytes[copied..length])?;
        if count == 0 {
            return Err(FsError::InvalidOperation);
        }
        copied += count;
    }
    Ok(length)
}

/// 解析并执行一条完整的 `/dev/testhook` 控制命令。
///
/// 命令格式必须为 `"before <case_id>\n"` 或 `"after <case_id>\n"`，其中 `<case_id>` 非空且不含空白字符或 NUL 字节。
/// 语法校验通过后转交 [`contest_hooks::before`] 或 [`contest_hooks::after`] 执行。
fn dispatch_hook_message(message: &[u8]) -> Result<(), FsError> {
    let command = message
        .strip_suffix(b"\n")
        .ok_or(FsError::InvalidOperation)?;
    let (before, case) = if let Some(case) = command.strip_prefix(b"before ") {
        (true, case)
    } else if let Some(case) = command.strip_prefix(b"after ") {
        (false, case)
    } else {
        return Err(FsError::InvalidOperation);
    };
    if case.is_empty()
        || case
            .iter()
            .any(|byte| *byte == 0 || byte.is_ascii_whitespace())
    {
        return Err(FsError::InvalidOperation);
    }

    if before {
        contest_hooks::before(case)
    } else {
        contest_hooks::after(case)
    }
}

/// 分块从写入源拉取并丢弃全部字节（用于 `/dev/null` 与 `/dev/zero` 的写入实现）。
///
/// 即使数据被丢弃，仍会实际读取用户态源缓冲区以验证内存可访问性；
/// 若拷贝过程中发生缺页错误，在已成功消费部分字节时返回已消费的字节数（Partial Progress），在首字节即失败时返回对应错误。
fn discard(source: &mut WriteBuffer<'_>) -> Result<usize, FsError> {
    let start = source.copied();
    let mut bytes = [0; DEVICE_IO_CHUNK];
    while source.copied() < source.len() {
        match source.pull(&mut bytes) {
            Ok(0) => break,
            Ok(_) => {}
            Err(error) if source.copied() == start => return Err(error),
            Err(_) => break,
        }
    }
    Ok(source.copied() - start)
}

/// 分块向读取目标缓冲区填充全零字节（用于 `/dev/zero` 的读取实现）。
///
/// 每次推送 512 字节栈上零数组且不持有任何内核锁，允许缺页异常处理正常获取内存管理锁；
/// 若中途发生缺页错误，在已成功写入部分字节时返回已写入的字节数（Partial Progress），在首字节即失败时返回对应错误。
fn zeros(target: &mut ReadBuffer<'_>) -> Result<usize, FsError> {
    let start = target.copied();
    let bytes = [0; DEVICE_IO_CHUNK];
    while target.copied() < target.len() {
        match target.push(&bytes) {
            Ok(0) => break,
            Ok(_) => {}
            Err(error) if target.copied() == start => return Err(error),
            Err(_) => break,
        }
    }
    Ok(target.copied() - start)
}

impl FileLike for NullFile {
    /// `/dev/null` 支持 `pread` / `pwrite` 按偏移位置调用（偏移量参数不影响读写语义）。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 从 `/dev/null` 读取始终立即返回 `Ok(0)`（EOF），不向接收缓冲区写入任何数据。
    fn read_into(
        &self,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        Ok(0)
    }

    /// 从 `/dev/null` 指定偏移处读取同样立即返回 `Ok(0)`（EOF）。
    fn read_at_into(
        &self,
        _offset: u64,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.read_into(target, caller)
    }

    /// 返回打开该设备时的 [`Path`]，用于查询底层字符设备 Inode 属性并维持挂载点生命周期。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// `/dev/null` 不支持普通文件的页缓存映射，返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 原子读取当前打开文件实例的状态标志位。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新 `fcntl(F_SETFL)` 允许修改的文件状态标志位。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除当前打开实例的 `O_NONBLOCK` 非阻塞标志位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 字符设备不支持截断文件长度，返回 [`FsError::InvalidOperation`]。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 虚拟字符设备无底层持久化存储，不支持 `fsync` / `fdatasync`，返回 [`FsError::InvalidOperation`]。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// `/dev/null` 的 `lseek` 操作对任意受支持的起点和偏移均成功返回新位置 `0`。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Ok(0)
    }

    /// 查询底层 `/dev/null` 字符设备节点的元数据（Inode 编号、权限及设备号 `rdev`）。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 字符设备不是目录，拒绝 `readdir` 并返回 [`FsError::NotDirectory`]。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 字符设备不是目录，拒绝目录遍历并返回 [`FsError::NotDirectory`]。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 向 `/dev/null` 写入时分块拉取并丢弃输入数据；若请求长度为 0 则不访问源地址。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_write(self.flags())?;
        discard(source)
    }

    /// 向 `/dev/null` 指定偏移处写入同样拉取并丢弃输入数据，忽略偏移量。
    fn write_at_from(
        &self,
        _offset: u64,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.write_from(source, caller)
    }
}

impl FileLike for ZeroFile {
    /// `/dev/zero` 支持 `pread` / `pwrite` 按偏移位置调用（偏移量参数不影响全零内容）。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 从 `/dev/zero` 读取时向接收缓冲区填充零字节；若中途遇到用户态缺页错误，返回已成功拷贝的字节数。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        zeros(target)
    }

    /// 从 `/dev/zero` 指定偏移处读取同样向接收缓冲区填充零字节，忽略偏移量。
    fn read_at_into(
        &self,
        _offset: u64,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.read_into(target, caller)
    }

    /// 返回打开该设备时的 [`Path`]，用于查询底层字符设备 Inode 属性并维持挂载点生命周期。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// `/dev/zero` 不提供共享文件页缓存映射，返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 原子读取当前打开文件实例的状态标志位。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新 `fcntl(F_SETFL)` 允许修改的文件状态标志位。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除当前打开实例的 `O_NONBLOCK` 非阻塞标志位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 字符设备不支持截断文件长度，返回 [`FsError::InvalidOperation`]。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 虚拟字符设备不支持 `fsync` / `fdatasync` 持久化同步，返回 [`FsError::InvalidOperation`]。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// `/dev/zero` 的 `lseek` 操作对任意受支持的起点和偏移均成功返回新位置 `0`。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Ok(0)
    }

    /// 查询底层 `/dev/zero` 字符设备节点的元数据（Inode 编号、权限及设备号 `rdev`）。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 字符设备不是目录，拒绝 `readdir` 并返回 [`FsError::NotDirectory`]。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 字符设备不是目录，拒绝目录遍历并返回 [`FsError::NotDirectory`]。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 向 `/dev/zero` 写入时分块拉取并丢弃输入数据。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_write(self.flags())?;
        discard(source)
    }

    /// 向 `/dev/zero` 指定偏移处写入同样拉取并丢弃输入数据，忽略偏移量。
    fn write_at_from(
        &self,
        _offset: u64,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.write_from(source, caller)
    }
}

impl FileLike for TestHookFile {
    /// `/dev/testhook` 为只写控制节点，任何读取尝试均返回 [`FsError::BadFd`]。
    fn read_into(
        &self,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }

    /// `/dev/testhook` 不支持按偏移位置读取（`pread`），始终返回 [`FsError::IllegalSeek`]。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 返回打开该设备时的 [`Path`]，用于查询底层字符设备 Inode 属性并维持挂载点生命周期。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 控制设备不支持文件页缓存映射，返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 原子读取当前打开文件实例的状态标志位。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新 `fcntl(F_SETFL)` 允许修改的文件状态标志位，不影响只写访问模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除当前打开实例的 `O_NONBLOCK` 非阻塞标志位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 控制节点不支持文件截断，返回 [`FsError::InvalidOperation`]。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 控制节点不支持 `fsync` / `fdatasync` 同步操作，返回 [`FsError::InvalidOperation`]。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 控制命令流无字节游标概念，拒绝 `lseek` 并返回 [`FsError::IllegalSeek`]。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 查询底层 `/dev/testhook` 字符设备节点的元数据（Inode 编号、权限及设备号 `rdev`）。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 控制设备不是目录，拒绝 `readdir` 并返回 [`FsError::NotDirectory`]。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 控制设备不是目录，拒绝目录遍历并返回 [`FsError::NotDirectory`]。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 接收并执行一条评测控制命令。
    ///
    /// 先校验写权限与当前调用者的 root 身份，再将完整命令拷贝至内核栈缓冲区并解析执行；
    /// 若拷贝失败或命令语法错误则直接返回错误且不触发任何钩子副作用，成功则返回完整命令字节长度。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_write(self.flags())?;
        require_hook_control(caller)?;

        let mut bytes = [0; HOOK_MESSAGE_BYTES];
        let length = copy_hook_message(source, &mut bytes)?;

        dispatch_hook_message(&bytes[..length])?;
        Ok(length)
    }

    /// 控制设备不支持按偏移位置写入（`pwrite`），始终返回 [`FsError::IllegalSeek`]。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
}
