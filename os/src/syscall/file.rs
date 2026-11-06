//! 文件系统与描述符类系统调用实现（打开、读写、向量 I/O、零拷贝传输、目录遍历、元数据查询与修改、描述符管理）。
//!
//! # 模块职责与架构定位
//! 本模块是用户态文件 ABI 与内核 VFS 子系统（`crate::fs`）之间的适配层：
//! - **上游调用者**：`crate::syscall` 总分发器与 `poll` 等兄弟模块（调用 [`nofile`]、[`import_vectors`]、[`resolve_bytes`]）。
//! - **下游依赖**：进程文件描述符表 `FileTable`、打开文件描述对象（Open File Description, `FileRef`）、虚拟文件系统 [`Vfs`] 与路径上下文 [`FsContext`]、以及进程地址空间 [`Mm`]（逐页安全读写用户态缓冲区）。
//!
//! # 核心机制与关键设计
//! ## 1. 槽位预留与原子发布（Slot Reservation & Publication）
//! 在 `openat`、`pipe2` 等需要分配新文件描述符的系统调用中，采用“先预留槽位 -> 构造底层文件对象并完成用户态拷贝 -> 正式发布到描述符槽位”的顺序：
//! ```text
//! 1. runtime.files.reserve(nofile)  ──► 原子占用空闲 fd 编号（此时并发线程尚不可见）
//! 2. 路径解析 / 创建底层 FileRef / copyout 管道 fd 数组
//!      ├─ 若中途失败 ──► Reservation 自动 Drop 回收槽位，不会在文件表中残留半成品 fd
//!      └─ 若全部成功 ──► 3. reservation.publish(file, flags) 原子公开至文件表
//! ```
//!
//! ## 2. 用户态 ABI 结构体字节布局
//! 本模块不将用户态地址直接强转为 Rust 结构体引用，而是在内核栈缓冲中按目标架构 LP64 小端序字节偏移编码后再通过 `copyout` 写出：
//! ```text
//! struct linux_dirent64 (变长，8 字节对齐):
//! +-----------------------+-----------------------+-------------------+---------------+-------------------+
//! | 0..8: d_ino (u64)     | 8..16: d_off (u64)    | 16..18: reclen    | 18: d_type    | 19..: d_name\0... |
//! +-----------------------+-----------------------+-------------------+---------------+-------------------+
//!
//! struct stat (128 字节, asm-generic LP64 布局):
//! +-----------------------+-----------------------+-------------------+-------------------+
//! | 0..8: st_dev (u64)    | 8..16: st_ino (u64)   | 16..20: mode(u32) | 20..24: nlink(u32)|
//! | 24..28: uid (u32)     | 28..32: gid (u32)     | 32..40: rdev(u64) | 40..48: pad (0)   |
//! | 48..56: size (i64)    | 56..60: blksize(u32)  | 60..64: pad (0)   | 64..72: blocks    |
//! | 72..88: atime (s+ns)  | 88..104: mtime (s+ns) | 104..120: ctime   | 120..128: pad (0) |
//! +-----------------------+-----------------------+-------------------+-------------------+
//!
//! struct statx (256 字节):
//! +-----------------------+-----------------------+-------------------+-------------------+
//! | 0..4: mask (0x7ff)    | 4..8: blksize (u32)   | 8..16: attributes | 16..20: nlink(u32)|
//! | 20..24: uid (u32)     | 24..28: gid (u32)     | 28..30: mode(u16) | 30..32: pad (0)   |
//! | 32..40: ino (u64)     | 40..48: size (u64)    | 48..56: blocks    | 56..64: attr_mask |
//! | 64..80: atime         | 80..96: btime (0)     | 96..112: ctime    | 112..128: mtime   |
//! | 128..136: rdev(maj/m) | 136..144: dev(maj/m) | 144..256: 保留字段 (全部填 0)         |
//! +-----------------------+-----------------------+-------------------+-------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! 1. **文件表锁（`FileTable`）与 I/O 操作解耦**：调用 `runtime.files.lookup(fd)` 时仅在极短临界区内克隆 `FileRef`（`Arc`），所有可能阻塞或触发缺页的读写、同步、截断、`ioctl` 操作必须在文件表锁外执行。
//! 2. **共享文件系统上下文锁（`runtime.fs`）与路径解析解耦**：通过 [`fs_snapshot`] 克隆当前的 `Arc<Vfs>` 与 [`FsContext`]（含 `root`、`cwd`、`umask`）后立即释放 `runtime.fs` 锁，严禁在持有 `runtime.fs` 锁期间执行 VFS 路径解析或 Inode I/O；`chdir`/`chroot`/`fchdir` 仅在路径与权限全部验证完毕后，再次短暂加锁原子替换 `cwd` 或 `root`，并在锁外析构旧 `Path`。
//! 3. **Inode 元数据写锁与内容写锁**：修改权限（`chmod`）与时间戳（`utimensat`）仅获取 Inode 元数据写守卫（`inode_write`），不获取文件内容写锁（`acquire_write`）。

use alloc::borrow::Cow;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::convert::{TryFrom, TryInto};

use crate::arch;
use crate::fs::{
    self, DescriptorFlags, DirectoryEntry, FileIoContext, FileStat, FsContext, InodeKind,
    InodeTime, Path, ReadBuffer, ResolveMode, SeekFrom, UserIoVector, Vfs, WriteBuffer, MAY_EXEC,
    MAY_READ, MAY_WRITE, NAME_MAX, O_ACCMODE, O_APPEND, O_CLOEXEC, O_CREAT, O_DIRECTORY, O_EXCL,
    O_LARGEFILE, O_NOCTTY, O_NOFOLLOW, O_NONBLOCK, O_RDONLY, O_RDWR, O_SYNC, O_TRUNC, O_WRONLY,
    PATH_MAX, S_IFBLK, S_IFCHR, S_IFDIR, S_IFIFO, S_IFLNK, S_IFREG, S_IFSOCK,
};
use crate::mm::space::Mm;
use crate::mm::PAGE_SIZE;
use crate::syscall::error::{copyin, copyout, Errno, SysResult};
use crate::task::process::Resource;
use crate::task::{UserRuntime, UserThread};
use crate::time::NSEC_PER_SEC;

const AT_FDCWD: i32 = -100;
const FIONBIO: u32 = 0x5421;

const AT_SYMLINK_NOFOLLOW: usize = 0x100;
const AT_EMPTY_PATH: usize = 0x1000;
const AT_EACCESS: u32 = 0x200;
const AT_NO_AUTOMOUNT: u32 = 0x800;
const AT_STATX_SYNC_TYPE: u32 = 0x6000;
const STATX_BASIC_STATS: u32 = 0x7ff;
const STATX_RESERVED: u32 = 0x8000_0000;
const AT_REMOVEDIR: u32 = 0x200;
const AT_SYMLINK_FOLLOW: u32 = 0x400;
const RENAME_NOREPLACE: u32 = 1;

const F_DUPFD: u32 = 0;
const F_GETFD: u32 = 1;
const F_SETFD: u32 = 2;
const F_GETFL: u32 = 3;
const F_SETFL: u32 = 4;
const F_LINUX_SPECIFIC_BASE: u32 = 1024;
const F_DUPFD_CLOEXEC: u32 = F_LINUX_SPECIFIC_BASE + 6;
const FD_CLOEXEC: usize = 1;

const SEEK_SET: u32 = 0;
const SEEK_CUR: u32 = 1;
const SEEK_END: u32 = 2;

const SPLICE_F_MOVE: u32 = 0x01;
const SPLICE_F_NONBLOCK: u32 = 0x02;
const SPLICE_F_MORE: u32 = 0x04;
const SPLICE_F_GIFT: u32 = 0x08;
const SPLICE_F_ALL: u32 = SPLICE_F_MOVE | SPLICE_F_NONBLOCK | SPLICE_F_MORE | SPLICE_F_GIFT;

const UTIME_NOW: i64 = (1 << 30) - 1;
const UTIME_OMIT: i64 = UTIME_NOW - 1;
const MAX_NANOSECONDS: i64 = NSEC_PER_SEC as i64 - 1;

const UIO_FASTIOV: usize = 8;

const DIRENT64_NAME_OFFSET: usize = 19;
const DIRENT64_ALIGNMENT: usize = core::mem::size_of::<u64>();
const DIRENT64_MAX_SIZE: usize =
    (DIRENT64_NAME_OFFSET + NAME_MAX + 1 + DIRENT64_ALIGNMENT - 1) & !(DIRENT64_ALIGNMENT - 1);
const STAT_SIZE: usize = 128;
const STATX_SIZE: usize = 256;

const S_DT_SHIFT: u32 = 12;
const DT_UNKNOWN: u8 = 0;
const DT_FIFO: u8 = (S_IFIFO >> S_DT_SHIFT) as u8;
const DT_CHR: u8 = (S_IFCHR >> S_DT_SHIFT) as u8;
const DT_DIR: u8 = (S_IFDIR >> S_DT_SHIFT) as u8;
const DT_BLK: u8 = (S_IFBLK >> S_DT_SHIFT) as u8;
const DT_REG: u8 = (S_IFREG >> S_DT_SHIFT) as u8;
const DT_LNK: u8 = (S_IFLNK >> S_DT_SHIFT) as u8;
const DT_SOCK: u8 = (S_IFSOCK >> S_DT_SHIFT) as u8;

const OPENAT_SUPPORTED: u32 = O_ACCMODE
    | O_CREAT
    | O_EXCL
    | O_TRUNC
    | O_APPEND
    | O_NONBLOCK
    | O_SYNC
    | O_LARGEFILE
    | O_DIRECTORY
    | O_NOFOLLOW
    | O_NOCTTY
    | O_CLOEXEC;

pub(super) const MAX_RW_COUNT: usize = i32::MAX as usize & !(PAGE_SIZE - 1);
pub(super) const MAX_IOV_COUNT: usize = 1024;

/// 查询当前线程所属进程组的文件描述符数量软限制（`RLIMIT_NOFILE`）。
///
/// 每次分配描述符时实时从线程组资源限制中读取，避免将旧上限缓存在可能跨线程共享的文件表中。
pub(super) fn nofile(user: &UserThread) -> usize {
    user.identity()
        .expect("descriptor syscall has no user identity")
        .group()
        .resource_limit(Resource::NoFile)
        .current as usize
}

/// 处理 `pipe2` 系统调用：创建匿名管道并将读端与写端文件描述符写回用户态。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw[0]`: 用户态 `int pipefd[2]` 数组首地址。
/// - `raw[1]`: 标志位（仅允许 `O_CLOEXEC` 与 `O_NONBLOCK`）。
///
/// # 原子发布契约
/// - 预先在文件表中预留两个描述符槽位并创建底层管道读写端点。
/// - 仅当两个描述符编号完整通过 `copyout` 写入用户空间后，才将读写端点正式发布到文件表中；若 `copyout` 失败，预留槽位与管道对象随作用域自动回收，不会产生泄漏。
pub(super) fn pipe2(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let flags = raw[1] as u32;
    if flags & !(O_CLOEXEC | O_NONBLOCK) != 0 {
        return Err(Errno::InvalidArgument);
    }

    // Step 1: 预留两个文件描述符槽位并构造底层管道读写端对象。
    let runtime = user.runtime().expect("pipe2 has no runtime");
    let reader_slot = runtime.files.reserve(nofile(user))?;
    let writer_slot = runtime.files.reserve(nofile(user))?;
    let (reader, writer) =
        fs::pipe::try_new(flags, runtime.credentials.uid, runtime.credentials.gid)?;

    // Step 2: 将分配到的读端、写端 fd 编号编码为 8 字节小端序数组并写回用户空间。
    let mut bytes = [0u8; 8];
    bytes[..4].copy_from_slice(&(reader_slot.fd() as u32).to_le_bytes());
    bytes[4..].copy_from_slice(&(writer_slot.fd() as u32).to_le_bytes());
    let mm = user.mm().expect("pipe2 has no MM");
    copyout(&mm, raw[0], &bytes)?;

    // Step 3: 用户态写回成功后，正式将读写端点发布至文件表槽位。
    let descriptor_flags = DescriptorFlags {
        close_on_exec: flags & O_CLOEXEC != 0,
    };
    reader_slot.publish(reader, descriptor_flags);
    writer_slot.publish(writer, descriptor_flags);
    Ok(0)
}

/// 从用户态可选指针 `address` 完整读取 64 位有符号文件偏移量（`loff_t`）。
///
/// - 若 `address == 0`（空指针），返回 `Ok(None)`，表示使用文件对象的当前共享游标。
/// - 若非空，则完整读取 8 字节并按位重解释为 `u64`，后续由传输层统一检查是否为非法负偏移。
fn transfer_offset(mm: &Mm, address: usize) -> Result<Option<u64>, Errno> {
    if address == 0 {
        return Ok(None);
    }
    let mut bytes = [0u8; 8];
    copyin(mm, &mut bytes, address)?;
    Ok(Some(i64::from_le_bytes(bytes) as u64))
}

/// 将更新后的显式文件偏移量（`loff_t`）写回用户态指针 `address`。
///
/// 若 `offset` 为 `None`（即调用方未提供显式偏移指针），则不执行任何操作。
fn store_transfer_offset(mm: &Mm, address: usize, offset: Option<u64>) -> Result<(), Errno> {
    if let Some(offset) = offset {
        copyout(mm, address, &offset.to_le_bytes())?;
    }
    Ok(())
}

/// 处理 `copy_file_range` 系统调用：在两个打开文件之间直接拷贝数据，无需经过用户态缓冲区中转。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[fd_in, off_in_ptr, fd_out, off_out_ptr, len, flags]`。
///
/// # 偏移量更新契约
/// - 若传入了非空的 `off_in_ptr` 或 `off_out_ptr`，则使用显式偏移且不修改文件对象自身的游标；当成功传输字节数 `accepted > 0` 时，依次尝试将推进后的源偏移和目标偏移写回用户空间。
pub(super) fn copy_file_range(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("copy_file_range has no runtime");
    let source = runtime.files.lookup(descriptor(raw[0])?)?;
    let target = runtime.files.lookup(descriptor(raw[2])?)?;
    let mm = user.mm().expect("copy_file_range has no MM");
    let mut source_offset = transfer_offset(&mm, raw[1])?;
    let mut target_offset = transfer_offset(&mm, raw[3])?;
    if raw[5] as u32 != 0 || (raw[4] as isize) < 0 {
        return Err(Errno::InvalidArgument);
    }

    let accepted = fs::transfer::copy_file_range(
        &source,
        source_offset.as_mut(),
        &target,
        target_offset.as_mut(),
        raw[4].min(MAX_RW_COUNT),
        FileIoContext::User(user),
    )?;

    if 0 < accepted {
        // 分别尝试写回源端与目标端的新偏移量；即使源端写回发生缺页错误，仍继续完成目标端偏移量的写回。
        let input = store_transfer_offset(&mm, raw[1], source_offset);
        let output = store_transfer_offset(&mm, raw[3], target_offset);
        input.and(output)?;
    }
    Ok(accepted as isize)
}

/// 处理 `splice` 系统调用：在管道与文件描述符之间移动或拷贝数据。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[fd_in, off_in_ptr, fd_out, off_out_ptr, len, flags]`。
///
/// # 校验与返回值契约
/// - 当请求传输长度 `len == 0` 时直接返回 `Ok(0)`。
/// - 若管道端传入了非空偏移指针（`off_in_ptr != 0` 或 `off_out_ptr != 0`），在对用户指针解引用之前立即返回 `ESPIPE`（[`Errno::IllegalSeek`]）。
/// - 仅消费目标端实际接收的字节数，并同步推进非管道端的偏移量。
pub(super) fn splice(user: &UserThread, raw: [usize; 6]) -> SysResult {
    // Step 1: 零长度请求直接成功返回，并校验 splice 标志位。
    if raw[4] == 0 {
        return Ok(0);
    }
    let flags = raw[5] as u32;
    if flags & !SPLICE_F_ALL != 0 {
        return Err(Errno::InvalidArgument);
    }

    // Step 2: 查找源与目标文件对象，拒绝为管道端点指定显式偏移指针。
    let runtime = user.runtime().expect("splice has no runtime");
    let source = runtime.files.lookup(descriptor(raw[0])?)?;
    let target = runtime.files.lookup(descriptor(raw[2])?)?;
    if (source.pipe_end().is_some() && raw[1] != 0) || (target.pipe_end().is_some() && raw[3] != 0)
    {
        return Err(Errno::IllegalSeek);
    }

    // Step 3: 从用户态读取显式偏移量，调用底层零拷贝/页传输实现，并将更新后的偏移量写回用户态。
    let mm = user.mm().expect("splice has no MM");
    let mut target_offset = transfer_offset(&mm, raw[3])?;
    let mut source_offset = transfer_offset(&mm, raw[1])?;
    if (raw[4] as isize) < 0 {
        return Err(Errno::InvalidArgument);
    }

    let accepted = fs::transfer::splice(
        &source,
        source_offset.as_mut(),
        &target,
        target_offset.as_mut(),
        raw[4].min(MAX_RW_COUNT),
        flags,
        FileIoContext::User(user),
    )?;

    store_transfer_offset(&mm, raw[3], target_offset)?;
    store_transfer_offset(&mm, raw[1], source_offset)?;
    Ok(accepted as isize)
}

/// 从用户态导入 `struct iovec` 向量数组，并校验每段缓冲区的地址范围与总长度上限。
///
/// # 内存分配与截断策略
/// - `count` 不得超过 [`MAX_IOV_COUNT`]（1024）。
/// - 当 `count <= UIO_FASTIOV`（8）时直接借用调用方栈上的 `fast` 数组，避免堆分配；超出时才在堆上分配 `Vec<UserIoVector>`。
/// - 所有向量段的总可读/可写字节数被截断在 [`MAX_RW_COUNT`] 以内。
pub(super) fn import_vectors<'a>(
    mm: &Mm,
    address: usize,
    count: usize,
    fast: &'a mut [UserIoVector; UIO_FASTIOV],
) -> Result<Cow<'a, [UserIoVector]>, Errno> {
    // Step 1: 校验向量段数量上限；count 按无符号 usize 解释，任何负数编码均会超出 MAX_IOV_COUNT。
    if MAX_IOV_COUNT < count {
        return Err(Errno::InvalidArgument);
    }
    if count == 0 {
        return Ok(Cow::Borrowed(&fast[..0]));
    }

    // Step 2: 选择栈上快速缓冲区或堆分配缓冲区，并一次性从用户空间拷入全部 iovec 结构体。
    let mut heap = Vec::new();
    let vectors = if count <= fast.len() {
        &mut fast[..count]
    } else {
        heap.try_reserve_exact(count).map_err(|_| Errno::NoMemory)?;
        heap.resize(
            count,
            UserIoVector {
                address: 0,
                length: 0,
            },
        );
        heap.as_mut_slice()
    };

    assert_eq!(core::mem::size_of::<UserIoVector>(), 16);
    // SAFETY:
    // 1. `UserIoVector` 为 `#[repr(C)]` 且仅包含两个 `usize` 字段（共 16 字节，已由上方 `assert_eq!` 断言），与目标 LP64 ABI 的 `struct iovec` 内存布局完全一致，且任意位模式均为合法的 `usize` 值。
    // 2. `vectors` 为当前函数独占的可变切片，且已初始化恰好 `count` 个元素，因此从 `vectors.as_mut_ptr()` 起构造长度为 `count * 16` 的可变 `u8` 切片完全落在有效分配内，无越界或别名冲突。
    let bytes = unsafe {
        core::slice::from_raw_parts_mut(
            vectors.as_mut_ptr().cast::<u8>(),
            count * core::mem::size_of::<UserIoVector>(),
        )
    };
    let progress = mm.copyin(bytes, address);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(Errno::BadAddress);
    }
    if vectors.iter().any(|vector| (vector.length as isize) < 0) {
        return Err(Errno::InvalidArgument);
    }

    // Step 3: 逐段检查用户态虚拟地址范围是否越界，并将累计总长度裁剪至 MAX_RW_COUNT 以内。
    let mut total = 0usize;
    for vector in vectors.iter_mut() {
        // 单段向量在按 MAX_RW_COUNT 裁剪后检查地址上界，多段向量检查原始长度地址上界；零长度段不访问基地址。
        let checked_length = if count == 1 {
            vector.length.min(MAX_RW_COUNT)
        } else {
            vector.length
        };
        if checked_length != 0 {
            let end = vector
                .address
                .checked_add(checked_length)
                .ok_or(Errno::BadAddress)?;
            if arch::USER_ADDRESS_END < end {
                return Err(Errno::BadAddress);
            }
        }
        vector.length = vector.length.min(MAX_RW_COUNT - total);
        total += vector.length;
    }

    if count <= fast.len() {
        Ok(Cow::Borrowed(&fast[..count]))
    } else {
        Ok(Cow::Owned(heap))
    }
}

/// 处理 `readv` 系统调用：从文件描述符读取数据并分散写入用户态多个 `iovec` 缓冲区（Scatter Read）。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[fd, iov_ptr, iovcnt, ...]`。
///
/// # 错误与部分进度契约
/// - 依次校验文件描述符有效性、文件非只写模式（`O_WRONLY` 返回 `EBADF`）及 `iovec` 数组合法性。
/// - 若读取过程中后续向量段发生缺页错误，底层读取逻辑会保留此前已成功写入用户缓冲区的字节数（Partial Progress）并推进相应文件游标。
pub(super) fn readv(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("readv has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    if file.flags() & O_ACCMODE == O_WRONLY {
        return Err(Errno::BadFd);
    }

    let mm = user.mm().expect("readv has no MM");
    let mut fast = [UserIoVector {
        address: 0,
        length: 0,
    }; UIO_FASTIOV];
    let vectors = import_vectors(&mm, raw[1], raw[2], &mut fast)?;
    let mut target = ReadBuffer::user_vector(&mm, &vectors)?;
    if target.len() == 0 {
        return Ok(0);
    }

    Ok(file.read_into(&mut target, FileIoContext::User(user))? as isize)
}

/// 处理 `writev` 系统调用：将用户态多个 `iovec` 缓冲区的数据聚集写入文件描述符（Gather Write）。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[fd, iov_ptr, iovcnt, ...]`。
///
/// # 边界契约
/// - 当所有向量段总长度为 0 时，普通文件直接返回 `Ok(0)`；但若目标为套接字（Socket），仍向下层协议栈发起调用以支持零长度数据报发送或关闭状态检测。
pub(super) fn writev(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("writev has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    if file.flags() & O_ACCMODE == O_RDONLY {
        return Err(Errno::BadFd);
    }

    let mm = user.mm().expect("writev has no MM");
    let mut fast = [UserIoVector {
        address: 0,
        length: 0,
    }; UIO_FASTIOV];
    let vectors = import_vectors(&mm, raw[1], raw[2], &mut fast)?;
    let mut source = WriteBuffer::user_vector(&mm, &vectors)?;
    if source.len() == 0 && file.socket().is_none() {
        return Ok(0);
    }

    Ok(file.write_from(&mut source, FileIoContext::User(user))? as isize)
}

/// 处理 `openat` 系统调用：解析路径并打开或创建文件，返回新分配的文件描述符。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[dirfd, pathname_ptr, flags, mode, ...]`。
///
/// # 执行流程与并发契约
/// 1. 校验打开标志位（拒绝非法的读写模式组合、`O_CREAT | O_DIRECTORY` 冲突以及未支持的标志位）。
/// 2. 先在进程文件表中预留描述符槽位，再从用户态读取路径字符串并获取当前 VFS 与 [`FsContext`] 快照。
/// 3. 若指定了 `O_CREAT`，通过 `vfs.open_with` 在目录项创建或查找完成后直接回调 [`prepare_open`] 构造打开文件对象；否则先解析路径再调用 [`prepare_open`]。
/// 4. 全部成功后将打开文件对象正式发布到预留槽位中。
pub(super) fn openat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("openat has no runtime");
    let mm = user.mm().expect("openat has no MM");
    // Step 1: 校验打开标志位组合是否合法。
    let flags = raw[2] as u32;
    if flags & O_ACCMODE == O_ACCMODE {
        return Err(Errno::InvalidArgument);
    }
    if flags & !OPENAT_SUPPORTED != 0 {
        return Err(Errno::Unsupported);
    }
    if flags & (O_CREAT | O_DIRECTORY) == (O_CREAT | O_DIRECTORY) {
        return Err(Errno::InvalidArgument);
    }

    // Step 2: 预留文件描述符槽位，读取用户态路径并获取 VFS 上下文快照与起始解析目录。
    let reservation = runtime.files.reserve(nofile(user))?;
    let pathname = mm.read_c_string(raw[1], PATH_MAX)?;
    let (vfs, context) = fs_snapshot(&runtime);
    let start = resolve_start(&runtime, &context, raw[0], &pathname)?;
    let credentials = (runtime.credentials.uid, runtime.credentials.gid);

    // Step 3: 根据是否包含 O_CREAT 标志执行创建/查找路径并构造底层打开文件对象。
    let file = if flags & O_CREAT != 0 {
        vfs.open_with(
            &context,
            &start,
            &pathname,
            credentials,
            Some((raw[3] as u16, context.umask())),
            flags & O_EXCL != 0,
            flags & O_NOFOLLOW != 0,
            |path, created| {
                prepare_open(
                    &vfs,
                    path,
                    flags,
                    credentials,
                    created,
                    FileIoContext::User(user),
                )
            },
        )?
    } else {
        let mode = if flags & O_NOFOLLOW != 0 {
            ResolveMode::NoFollow
        } else {
            ResolveMode::Follow
        };
        let path = vfs.resolve_from(&context, &start, &pathname, mode, Some(credentials))?;
        prepare_open(
            &vfs,
            path,
            flags,
            credentials,
            false,
            FileIoContext::User(user),
        )?
    };

    // Step 4: 将构造完成的打开文件对象正式发布到预留槽位中。
    Ok(reservation.publish(
        file,
        DescriptorFlags {
            close_on_exec: flags & O_CLOEXEC != 0,
        },
    ) as isize)
}

/// 根据解析或新建得到的目标 [`Path`] 完成自主访问控制（DAC）权限校验并构造打开文件对象 [`fs::FileRef`]。
///
/// # 权限与截断契约
/// - 若文件并非本次新建（`!created`），则根据打开模式（`O_RDONLY` / `O_WRONLY` / `O_RDWR`）及是否带有 `O_TRUNC` 检查当前凭证的读/写权限。
/// - 若文件是本次 `O_CREAT` 新建的空文件（`created == true`），按 POSIX 规范直接授予创建者本次打开权限，不再重复检查新文件的 mode 位，也不需要执行 `O_TRUNC` 截断。
fn prepare_open(
    vfs: &Vfs,
    path: Path,
    flags: u32,
    credentials: (u32, u32),
    created: bool,
    caller: FileIoContext<'_>,
) -> Result<fs::FileRef, fs::FsError> {
    let access = flags & O_ACCMODE;
    if access == O_ACCMODE {
        return Err(fs::FsError::InvalidOperation);
    }
    let kind = path.inode()?.metadata_snapshot()?.kind;
    if flags & O_CREAT != 0 && !created && kind == InodeKind::Directory {
        return Err(fs::FsError::IsDirectory);
    }
    if flags & O_DIRECTORY != 0 && kind != InodeKind::Directory {
        return Err(fs::FsError::NotDirectory);
    }

    if !created {
        let access = match access {
            O_RDONLY => MAY_READ,
            O_WRONLY => MAY_WRITE,
            O_RDWR => MAY_READ | MAY_WRITE,
            _ => unreachable!(),
        };
        let access = if flags & O_TRUNC != 0 {
            access | MAY_WRITE
        } else {
            access
        };
        path.check_access(credentials.0, credentials.1, access)?;
    }

    let flags = flags & !(O_CREAT | O_EXCL | if created { O_TRUNC } else { 0 });
    fs::open_path_with_flags(vfs, path, flags, caller)
}

/// 处理 `mkdirat` 系统调用：根据指定目录文件描述符与路径创建新目录。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[dirfd, pathname_ptr, mode, ...]`。
/// - 最终创建的目录权限会与当前进程 [`FsContext`] 的 `umask` 掩码结合计算。
pub(super) fn mkdirat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("mkdirat has no runtime");
    let mm = user.mm().expect("mkdirat has no MM");
    let pathname = mm.read_c_string(raw[1], PATH_MAX)?;
    let (vfs, context) = fs_snapshot(&runtime);
    let start = resolve_start(&runtime, &context, raw[0], &pathname)?;
    let credentials = (runtime.credentials.uid, runtime.credentials.gid);
    vfs.mkdir_from(
        &context,
        &start,
        &pathname,
        raw[2] as u16,
        credentials,
        context.umask(),
    )?;
    Ok(0)
}

/// 处理 `unlinkat` 系统调用：删除目录项（普通文件/链接）或空目录。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[dirfd, pathname_ptr, flags, ...]`。
/// - `flags` 仅允许 `0`（删除普通目录项）或 `AT_REMOVEDIR`（删除空目录）。
pub(super) fn unlinkat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let flags = raw[2] as u32;
    if flags & !AT_REMOVEDIR != 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("unlinkat has no runtime");
    let mm = user.mm().expect("unlinkat has no MM");
    let pathname = mm.read_c_string(raw[1], PATH_MAX)?;
    let (vfs, context) = fs_snapshot(&runtime);
    let start = resolve_start(&runtime, &context, raw[0], &pathname)?;
    let credentials = (runtime.credentials.uid, runtime.credentials.gid);
    if flags & AT_REMOVEDIR != 0 {
        vfs.rmdir_from(&context, &start, &pathname, credentials)?;
    } else {
        vfs.unlink_from(&context, &start, &pathname, credentials)?;
    }
    Ok(0)
}

/// 处理 `symlinkat` 系统调用：在指定目录下创建指向目标字符串的符号链接。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[target_ptr, newdirfd, linkpath_ptr, ...]`。
/// - `target` 作为原始字节串原样写入符号链接内容，仅对 `linkpath` 执行 VFS 路径解析。
pub(super) fn symlinkat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("symlinkat has no runtime");
    let mm = user.mm().expect("symlinkat has no MM");
    let target = mm.read_c_string(raw[0], PATH_MAX)?;
    let pathname = mm.read_c_string(raw[2], PATH_MAX)?;
    let (vfs, context) = fs_snapshot(&runtime);
    let start = resolve_start(&runtime, &context, raw[1], &pathname)?;
    let credentials = (runtime.credentials.uid, runtime.credentials.gid);
    vfs.symlink_from(&context, &start, &pathname, &target, credentials)?;
    Ok(0)
}

/// 处理 `linkat` 系统调用：为已有文件创建新的硬链接目录项。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[olddirfd, oldpath_ptr, newdirfd, newpath_ptr, flags, ...]`。
/// - 默认不跟随源路径末尾的符号链接（硬链接符号链接本身）；仅当 `flags` 指定 `AT_SYMLINK_FOLLOW` 时才跟随源路径末尾的符号链接。
pub(super) fn linkat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let flags = raw[4] as u32;
    if flags & !AT_SYMLINK_FOLLOW != 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("linkat has no runtime");
    let mm = user.mm().expect("linkat has no MM");
    let old_pathname = mm.read_c_string(raw[1], PATH_MAX)?;
    let new_pathname = mm.read_c_string(raw[3], PATH_MAX)?;
    let (vfs, context) = fs_snapshot(&runtime);
    let old_start = resolve_start(&runtime, &context, raw[0], &old_pathname)?;
    let new_start = resolve_start(&runtime, &context, raw[2], &new_pathname)?;
    let mode = if flags & AT_SYMLINK_FOLLOW != 0 {
        ResolveMode::Follow
    } else {
        ResolveMode::NoFollow
    };
    let credentials = (runtime.credentials.uid, runtime.credentials.gid);
    let source = vfs.resolve_from(&context, &old_start, &old_pathname, mode, Some(credentials))?;

    vfs.link_from(&source, &context, &new_start, &new_pathname, credentials)?;
    Ok(0)
}

/// 处理 `renameat2` 系统调用：重命名或移动目录项。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[olddirfd, oldpath_ptr, newdirfd, newpath_ptr, flags, ...]`。
/// - `flags` 仅支持 `0`（允许原子覆盖同名目标）与 `RENAME_NOREPLACE`（目标已存在时返回 `EEXIST`），传入其他标志位返回 [`Errno::InvalidArgument`]。
pub(super) fn renameat2(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let flags = raw[4] as u32;
    if flags & !RENAME_NOREPLACE != 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("renameat2 has no runtime");
    let mm = user.mm().expect("renameat2 has no MM");
    let old_pathname = mm.read_c_string(raw[1], PATH_MAX)?;
    let new_pathname = mm.read_c_string(raw[3], PATH_MAX)?;
    let (vfs, context) = fs_snapshot(&runtime);
    let old_start = resolve_start(&runtime, &context, raw[0], &old_pathname)?;
    let new_start = resolve_start(&runtime, &context, raw[2], &new_pathname)?;
    let credentials = (runtime.credentials.uid, runtime.credentials.gid);
    vfs.rename_from(
        &context,
        &old_start,
        &old_pathname,
        &new_start,
        &new_pathname,
        flags & RENAME_NOREPLACE != 0,
        credentials,
    )?;
    Ok(0)
}

/// 处理 `umask` 系统调用：原子替换当前共享文件系统上下文的文件创建权限掩码，并返回旧掩码值。
///
/// 通过 `CLONE_FS` 共享文件系统上下文的线程组共享同一个 `umask`。
pub(super) fn umask(user: &UserThread, raw: usize) -> SysResult {
    let runtime = user.runtime().expect("umask has no runtime");
    let previous = runtime.fs.lock().context.exchange_umask(raw as u16);
    Ok(previous as isize)
}

/// 处理 `truncate` 系统调用：按文件路径将普通文件截断或扩展至指定字节长度。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[pathname_ptr, length, ...]`。
///
/// # 校验顺序
/// - 先检查有符号长度 `length` 是否为负数（负数直接返回 `EINVAL`，不访问用户态路径指针）。
/// - 跟随符号链接解析目标 Inode，校验其必须为普通文件且当前进程具备写权限（`MAY_WRITE`），最后获取 Inode 写访问权完成截断。
pub(super) fn truncate(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let length = raw[1] as i64;
    if length < 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("truncate has no runtime");
    let mm = user.mm().expect("truncate has no MM");
    let path = resolve(
        &runtime,
        &mm,
        AT_FDCWD as usize,
        raw[0],
        ResolveMode::Follow,
    )?;
    let inode = path.inode()?;
    match inode.metadata_snapshot()?.kind {
        InodeKind::Regular => {}
        InodeKind::Directory => return Err(Errno::IsDirectory),
        _ => return Err(Errno::InvalidArgument),
    }
    path.check_access(runtime.credentials.uid, runtime.credentials.gid, MAY_WRITE)?;

    let access = inode.acquire_write()?;
    access.truncate(length as u64)?;
    Ok(0)
}

/// 处理 `ftruncate` 系统调用：通过已打开的文件描述符将文件截断或扩展至指定字节长度。
///
/// # 调用上下文与参数
/// - 运行于普通可睡眠的用户任务上下文。
/// - `raw`: `[fd, length, ...]`。
/// - 先拒绝负数长度，再从文件表获取打开文件对象引用并在表锁外执行截断，不修改文件对象的读写游标。
pub(super) fn ftruncate(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let length = raw[1] as i64;
    if length < 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("ftruncate has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    file.truncate(length as u64)?;
    Ok(0)
}

/// 处理 `read` 系统调用：从打开文件描述符的当前共享游标位置读取数据到用户态缓冲区，并按实际读取字节数推进游标。
pub(super) fn read(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("read has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    if file.flags() & O_ACCMODE == O_WRONLY {
        return Err(Errno::BadFd);
    }

    let mm = user.mm().expect("read has no MM");
    let mut target = ReadBuffer::user(&mm, raw[1], raw[2].min(MAX_RW_COUNT))?;
    Ok(file.read_into(&mut target, FileIoContext::User(user))? as isize)
}

/// 处理 `pread64` 系统调用：从文件指定偏移量处读取数据，不修改打开文件对象的共享读写游标。
///
/// # 校验顺序
/// 1. 拒绝负偏移量（`raw[3] as i64 < 0` 返回 `EINVAL`）。
/// 2. 查找打开文件对象，检查其是否支持随机定位 I/O（管道或套接字返回 `ESPIPE`）。
/// 3. 检查打开模式是否允许读取（`O_WRONLY` 返回 `EBADF`），并校验用户缓冲区地址范围。
pub(super) fn pread(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if (raw[3] as i64) < 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("pread has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    if !file.positioned_io() {
        return Err(Errno::IllegalSeek);
    }
    if file.flags() & O_ACCMODE == O_WRONLY {
        return Err(Errno::BadFd);
    }

    let mm = user.mm().expect("pread has no MM");
    let mut target = ReadBuffer::user(&mm, raw[1], raw[2].min(MAX_RW_COUNT))?;
    Ok(file.read_at_into(raw[3] as u64, &mut target, FileIoContext::User(user))? as isize)
}

/// 处理 `write` 系统调用：将用户态缓冲区数据写入打开文件描述符，并按实际写入字节数更新共享游标。
pub(super) fn write(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("write has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    if file.flags() & O_ACCMODE == O_RDONLY {
        return Err(Errno::BadFd);
    }

    let mm = user.mm().expect("write has no MM");
    let mut source = WriteBuffer::user(&mm, raw[1], raw[2].min(MAX_RW_COUNT))?;
    Ok(file.write_from(&mut source, FileIoContext::User(user))? as isize)
}

/// 处理 `pwrite64` 系统调用：向文件指定偏移量处写入数据，不修改打开文件对象的共享读写游标。
///
/// # 校验顺序
/// 1. 拒绝负偏移量（`raw[3] as i64 < 0` 返回 `EINVAL`）。
/// 2. 查找打开文件对象，检查其是否支持随机定位 I/O（不支持则返回 `ESPIPE`）。
/// 3. 检查打开模式是否允许写入（`O_RDONLY` 返回 `EBADF`），随后执行定位写入。
pub(super) fn pwrite(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if (raw[3] as i64) < 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("pwrite has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    if !file.positioned_io() {
        return Err(Errno::IllegalSeek);
    }
    if file.flags() & O_ACCMODE == O_RDONLY {
        return Err(Errno::BadFd);
    }

    let mm = user.mm().expect("pwrite has no MM");
    let mut source = WriteBuffer::user(&mm, raw[1], raw[2].min(MAX_RW_COUNT))?;
    Ok(file.write_at_from(raw[3] as u64, &mut source, FileIoContext::User(user))? as isize)
}

/// 处理 `ioctl` 系统调用：执行通用文件描述符非阻塞状态切换（`FIONBIO`）或转发至底层设备/文件专用控制接口。
pub(super) fn ioctl(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("ioctl has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    let command = raw[1] as u32;
    if command == FIONBIO {
        let mm = user.mm().expect("ioctl has no MM");
        let mut bytes = [0u8; 4];
        copyin(&mm, &mut bytes, raw[2])?;
        file.set_nonblocking(i32::from_le_bytes(bytes) != 0)?;
        return Ok(0);
    }
    Ok(file.ioctl(command, raw[2], FileIoContext::User(user))? as isize)
}

/// 处理 `fsync` 系统调用：将指定文件描述符对应的全部脏数据页及文件元数据同步刷新到底层存储设备。
pub(super) fn fsync(user: &UserThread, raw: usize) -> SysResult {
    sync_file(user, raw, false)
}

/// 处理 `fdatasync` 系统调用：将指定文件描述符对应的脏数据页及寻址必需的元数据同步刷新到底层存储设备。
pub(super) fn fdatasync(user: &UserThread, raw: usize) -> SysResult {
    sync_file(user, raw, true)
}

/// `fsync` 与 `fdatasync` 的共用实现：从文件表克隆打开文件强引用后，在表锁外对全文件范围执行同步刷盘。
fn sync_file(user: &UserThread, raw: usize, data_only: bool) -> SysResult {
    let runtime = user.runtime().expect("file sync has no runtime");
    let file = runtime.files.lookup(descriptor(raw)?)?;
    file.sync_range(0..u64::MAX, data_only)?;
    Ok(0)
}

/// 处理 `lseek` 系统调用：根据基准模式（`SEEK_SET` / `SEEK_CUR` / `SEEK_END`）计算并更新打开文件的共享读写游标。
pub(super) fn seek(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let delta = raw[1] as i64;
    let from = match raw[2] as u32 {
        SEEK_SET => SeekFrom::Start(raw[1] as u64),
        SEEK_CUR => SeekFrom::Current(delta),
        SEEK_END => SeekFrom::End(delta),
        _ => return Err(Errno::InvalidArgument),
    };

    let runtime = user.runtime().expect("lseek has no runtime");
    let position = runtime.files.seek(descriptor(raw[0])?, from)?;
    isize::try_from(position).map_err(|_| Errno::Overflow)
}

/// 处理 `close` 系统调用：从进程文件描述符表中移除指定槽位，并在表锁外释放对应的打开文件对象引用。
///
/// 若其他并发线程已在关闭前通过 `lookup` 克隆了该文件的 `Arc` 引用，其正在进行的 I/O 仍安全作用于原文件对象，直至最后一个强引用被释放。
pub(super) fn close(user: &UserThread, raw: usize) -> SysResult {
    user.runtime()
        .expect("close has no runtime")
        .files
        .close(descriptor(raw)?)?;
    Ok(0)
}

/// 处理 `dup` 系统调用：分配最小可用文件描述符指向同一个打开文件对象，并清除新描述符的 `FD_CLOEXEC` 标志。
pub(super) fn dup(user: &UserThread, raw: usize) -> SysResult {
    Ok(user
        .runtime()
        .expect("dup has no runtime")
        .files
        .dup(descriptor(raw)?, nofile(user))? as isize)
}

/// 处理 `dup3` 系统调用：将原文件描述符复制到指定的目标描述符编号 `newfd`，并按 `flags` 设置 `O_CLOEXEC`。
///
/// 若 `oldfd == newfd` 或 `flags` 含有未知标志位，返回 [`Errno::InvalidArgument`]；若 `newfd` 原先已打开文件，旧文件引用在文件表锁外自动释放。
pub(super) fn dup3(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[2] as u32 & !O_CLOEXEC != 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("dup3 has no runtime");
    Ok(runtime.files.dup_exact_flags(
        descriptor(raw[0])?,
        descriptor(raw[1])?,
        DescriptorFlags {
            close_on_exec: raw[2] as u32 & O_CLOEXEC != 0,
        },
        nofile(user),
    )? as isize)
}

/// 处理 `fcntl` 系统调用：查询或修改文件描述符标志（`FD_CLOEXEC`）、打开文件状态标志（`O_NONBLOCK` / `O_APPEND`）或复制描述符。
pub(super) fn fcntl(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("fcntl has no runtime");
    let fd = descriptor(raw[0])?;
    match raw[1] as u32 {
        F_DUPFD | F_DUPFD_CLOEXEC => {
            let minimum = raw[2] as i32;
            if minimum < 0 {
                return Err(Errno::InvalidArgument);
            }
            Ok(runtime.files.dup_min(
                fd,
                minimum as usize,
                DescriptorFlags {
                    close_on_exec: raw[1] as u32 == F_DUPFD_CLOEXEC,
                },
                nofile(user),
            )? as isize)
        }
        F_GETFD => Ok(runtime.files.flags(fd)?.close_on_exec as isize),
        F_SETFD => {
            runtime.files.set_flags(
                fd,
                DescriptorFlags {
                    close_on_exec: raw[2] & FD_CLOEXEC != 0,
                },
            )?;
            Ok(0)
        }
        F_GETFL => Ok(runtime.files.lookup(fd)?.flags() as isize),
        F_SETFL => {
            runtime.files.lookup(fd)?.set_status_flags(raw[2] as u32)?;
            Ok(0)
        }
        _ => Err(Errno::InvalidArgument),
    }
}

/// 处理 `fstat` 系统调用：查询已打开文件描述符的元数据属性，并编码为 128 字节 `struct stat` 写回用户态。
pub(super) fn fstat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("fstat has no runtime");
    let stat = runtime.files.stat(descriptor(raw[0])?)?;
    let bytes = encode_stat(stat)?;
    copyout(&user.mm().expect("fstat has no MM"), raw[1], &bytes)?;
    Ok(0)
}

/// 处理 `newfstatat` 系统调用：按目录描述符、路径及标志位查询文件元数据，并编码为 128 字节 `struct stat` 写回用户态。
pub(super) fn statat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[3] & !(AT_SYMLINK_NOFOLLOW | AT_EMPTY_PATH | AT_NO_AUTOMOUNT as usize) != 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("statat has no runtime");
    let mm = user.mm().expect("statat has no MM");
    let stat = query_stat(&runtime, &mm, raw[0], raw[1], raw[3])?;
    copyout(&mm, raw[2], &encode_stat(stat)?)?;
    Ok(0)
}

/// 处理 `statx` 系统调用：按扩展标志位与掩码查询文件元数据，并编码为 256 字节 `struct statx` 写回用户态。
pub(super) fn statx(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let flags = raw[2] as u32;
    let mask = raw[3] as u32;
    let supported =
        AT_SYMLINK_NOFOLLOW as u32 | AT_EMPTY_PATH as u32 | AT_NO_AUTOMOUNT | AT_STATX_SYNC_TYPE;
    if flags & !supported != 0
        || flags & AT_STATX_SYNC_TYPE == AT_STATX_SYNC_TYPE
        || mask & STATX_RESERVED != 0
    {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("statx has no runtime");
    let mm = user.mm().expect("statx has no MM");
    let path_flags = (flags & !(AT_NO_AUTOMOUNT | AT_STATX_SYNC_TYPE)) as usize;
    let stat = query_stat(&runtime, &mm, raw[0], raw[1], path_flags)?;
    copyout(&mm, raw[4], &encode_statx(stat))?;
    Ok(0)
}

/// `newfstatat` 与 `statx` 共用的文件属性查询辅助函数。
///
/// # 空指针与 `AT_EMPTY_PATH` 规则
/// - 当用户态路径指针 `address == 0` 时，仅当 `flags == AT_EMPTY_PATH` 且 `dirfd >= 0` 时允许直接查询 `dirfd` 本身属性；否则返回 `EFAULT`（[`Errno::BadAddress`]）。
/// - 当路径为空字符串 `""` 且带有 `AT_EMPTY_PATH` 时，`dirfd == AT_FDCWD` 查询当前工作目录属性，非负 `dirfd` 查询对应打开文件属性。
fn query_stat(
    runtime: &UserRuntime,
    mm: &Mm,
    dirfd: usize,
    address: usize,
    flags: usize,
) -> Result<FileStat, Errno> {
    let pathname = if address == 0 {
        if flags != AT_EMPTY_PATH || (dirfd as i32) < 0 {
            return Err(Errno::BadAddress);
        }
        Vec::new()
    } else {
        mm.read_c_string(address, PATH_MAX)?
    };

    if pathname.is_empty() && flags & AT_EMPTY_PATH != 0 {
        if dirfd as i32 == AT_FDCWD {
            let (_, context) = fs_snapshot(runtime);
            Ok(fs::stat_path(context.cwd())?)
        } else {
            Ok(runtime.files.stat(descriptor(dirfd)?)?)
        }
    } else {
        let mode = if flags & AT_SYMLINK_NOFOLLOW != 0 {
            ResolveMode::NoFollow
        } else {
            ResolveMode::Follow
        };
        Ok(fs::stat_path(&resolve_bytes(
            runtime, dirfd, &pathname, mode,
        )?)?)
    }
}

/// 处理 `getdents64` 系统调用：批量读取目录项并以 8 字节对齐的 `linux_dirent64` 变长记录写入用户态缓冲区。
///
/// # 游标与部分读取契约
/// - 仅当整条 `linux_dirent64` 记录完整写入用户态缓冲区后，回调才返回 `Ok(true)` 推进目录游标（`cookie`）。
/// - 若用户缓冲区剩余空间不足以容纳当前记录：若尚未写入任何记录（`complete == 0`），返回 `EINVAL`；若已成功写入至少一条完整记录，则正常停止遍历并返回已写入的总字节数。
pub(super) fn getdents(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("getdents has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    let mm = user.mm().expect("getdents has no MM");
    let mut target = ReadBuffer::user(&mm, raw[1], raw[2].min(u32::MAX as usize))?;
    let mut complete = 0usize;

    // 逐条遍历目录项，按 8 字节对齐组装 linux_dirent64 并推入用户缓冲区。
    let result = file.for_each_dirent(&mut |entry| {
        let length = (DIRENT64_NAME_OFFSET + entry.name.len() + 1 + DIRENT64_ALIGNMENT - 1)
            & !(DIRENT64_ALIGNMENT - 1);
        if target.len() - complete < length {
            return if complete == 0 {
                Err(fs::FsError::InvalidOperation)
            } else {
                Ok(false)
            };
        }

        let mut bytes = [0u8; DIRENT64_MAX_SIZE];
        bytes[0..8].copy_from_slice(&entry.number.to_le_bytes());
        bytes[8..16].copy_from_slice(&entry.next_cookie.to_le_bytes());
        bytes[16..18].copy_from_slice(&(length as u16).to_le_bytes());
        bytes[18] = dirent_type(entry);
        bytes[19..19 + entry.name.len()].copy_from_slice(&entry.name);
        if target.push(&bytes[..length])? != length {
            return Err(fs::FsError::BadAddress);
        }
        complete += length;
        Ok(true)
    });

    if 0 < complete {
        Ok(complete as isize)
    } else {
        result.map(|()| 0).map_err(Errno::from)
    }
}

/// 处理 `readlinkat` 系统调用：读取符号链接指向的目标路径原始字节（不追加末尾 `\0`）并写入用户态缓冲区。
pub(super) fn readlink(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let count = raw[3] as i32;
    if count <= 0 {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("readlink has no runtime");
    let mm = user.mm().expect("readlink has no MM");
    let path = resolve(&runtime, &mm, raw[0], raw[1], ResolveMode::NoFollow)?;
    if path.inode()?.metadata_snapshot()?.kind != InodeKind::Symlink {
        return Err(Errno::InvalidArgument);
    }

    let target = path.inode()?.read_link()?;
    let count = target.len().min(count as usize);
    copyout(&mm, raw[2], &target[..count])?;
    Ok(count as isize)
}

/// 处理 `chdir` 系统调用：解析目标路径并验证其为具备可执行（搜索，`MAY_EXEC`）权限的目录后，替换当前共享 [`FsContext`] 的工作目录。
pub(super) fn chdir(user: &UserThread, address: usize) -> SysResult {
    let runtime = user.runtime().expect("chdir has no runtime");
    let mm = user.mm().expect("chdir has no MM");
    let path = resolve(
        &runtime,
        &mm,
        AT_FDCWD as usize,
        address,
        ResolveMode::Follow,
    )?;
    if path.inode()?.metadata_snapshot()?.kind != InodeKind::Directory {
        return Err(Errno::NotDirectory);
    }
    path.check_access(runtime.credentials.uid, runtime.credentials.gid, MAY_EXEC)?;

    let retired = runtime.fs.lock().context.exchange_cwd(path);
    drop(retired);
    Ok(0)
}

/// 处理 `chroot` 系统调用：解析目标目录路径，验证搜索权限（`MAY_EXEC`）与超级用户权限（`uid == 0`）后，替换当前共享 [`FsContext`] 的根目录。
///
/// 路径解析与权限检查均在 `runtime.fs` 锁外完成；替换时仅更新 `root`，不影响 `cwd`、`umask` 及已打开文件，旧根目录引用在锁外释放。
pub(super) fn chroot(user: &UserThread, address: usize) -> SysResult {
    let runtime = user.runtime().expect("chroot has no runtime");
    let mm = user.mm().expect("chroot has no MM");
    let path = resolve(
        &runtime,
        &mm,
        AT_FDCWD as usize,
        address,
        ResolveMode::Follow,
    )?;
    if path.inode()?.metadata_snapshot()?.kind != InodeKind::Directory {
        return Err(Errno::NotDirectory);
    }
    path.check_access(runtime.credentials.uid, runtime.credentials.gid, MAY_EXEC)?;
    if runtime.credentials.uid != 0 {
        return Err(Errno::NotPermitted);
    }

    let retired = runtime.fs.lock().context.exchange_root(path);
    drop(retired);
    Ok(0)
}

/// 处理 `getcwd` 系统调用：获取当前工作目录的绝对路径字符串（含末尾 `\0`）并写入用户态缓冲区，成功返回写入的总字节数。
pub(super) fn getcwd(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("getcwd has no runtime");
    let mm = user.mm().expect("getcwd has no MM");
    let (vfs, context) = fs_snapshot(&runtime);
    let mut bytes = vfs.cwd_bytes(&context)?;
    if raw[1] < bytes.len() + 1 {
        return Err(Errno::Range);
    }

    // cwd_bytes 已预留末尾 NUL 字节容量，直接追加 '\0' 并拷贝至用户态。
    bytes.push(0);
    copyout(&mm, raw[0], &bytes)?;
    Ok(bytes.len() as isize)
}

/// 处理 `fchdir` 系统调用：根据已打开目录的文件描述符验证搜索权限（`MAY_EXEC`），并切换当前工作目录。
pub(super) fn fchdir(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("fchdir has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    let path = file.path().ok_or(Errno::NotDirectory)?.clone();
    if path.inode()?.metadata_snapshot()?.kind != InodeKind::Directory {
        return Err(Errno::NotDirectory);
    }
    path.check_access(runtime.credentials.uid, runtime.credentials.gid, MAY_EXEC)?;

    let retired = runtime.fs.lock().context.exchange_cwd(path);
    drop(retired);
    Ok(0)
}

/// 处理 `fchmod` 系统调用：通过已打开的文件描述符修改底层 Inode 的访问权限位。
pub(super) fn fchmod(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("fchmod has no runtime");
    let file = runtime.files.lookup(descriptor(raw[0])?)?;
    let path = file.path().ok_or(Errno::InvalidArgument)?;
    chmod_path(&runtime, path, raw[1] as u16)
}

/// 处理 `fchmodat` 系统调用：解析目标文件路径（跟随末尾符号链接）并修改底层 Inode 的访问权限位。
pub(super) fn fchmodat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("fchmodat has no runtime");
    let mm = user.mm().expect("fchmodat has no MM");
    let path = resolve(&runtime, &mm, raw[0], raw[1], ResolveMode::Follow)?;
    chmod_path(&runtime, &path, raw[2] as u16)
}

/// 处理 `faccessat` / `faccessat2` 系统调用：检查当前进程凭证对目标路径是否具备指定的读/写/执行权限。
pub(super) fn faccessat(user: &UserThread, raw: [usize; 6], flags: u32) -> SysResult {
    let mode = raw[2] as u32;
    if mode & !u32::from(MAY_READ | MAY_WRITE | MAY_EXEC) != 0
        || flags & !(AT_EACCESS | AT_SYMLINK_NOFOLLOW as u32 | AT_EMPTY_PATH as u32) != 0
    {
        return Err(Errno::InvalidArgument);
    }

    let runtime = user.runtime().expect("faccessat has no runtime");
    let mm = user.mm().expect("faccessat has no MM");
    let pathname = mm.read_c_string(raw[1], PATH_MAX)?;
    let path = if pathname.is_empty() && flags & AT_EMPTY_PATH as u32 != 0 {
        if raw[0] as i32 == AT_FDCWD {
            fs_snapshot(&runtime).1.cwd().clone()
        } else {
            let file = runtime.files.lookup(descriptor(raw[0])?)?;
            file.path().cloned().ok_or(Errno::NoSyscall)?
        }
    } else {
        let follow = if flags & AT_SYMLINK_NOFOLLOW as u32 != 0 {
            ResolveMode::NoFollow
        } else {
            ResolveMode::Follow
        };
        resolve_bytes(&runtime, raw[0], &pathname, follow)?
    };

    path.check_access(
        runtime.credentials.uid,
        runtime.credentials.gid,
        mode as u16,
    )?;
    Ok(0)
}

/// 修改指定路径对应 Inode 的权限位（`mode`）并同步更新状态变更时间（`ctime`）。
///
/// 先检查所在挂载点是否可写，随后仅获取 Inode 元数据写守卫（`inode_write`）完成校验与发布，无需获取文件内容写锁。
fn chmod_path(runtime: &UserRuntime, path: &Path, mode: u16) -> SysResult {
    path.check_writable()?;
    let inode = path.inode()?;
    let guard = inode.inode_write()?;
    let prepared = guard.prepare_mode(
        mode,
        runtime.credentials.uid,
        runtime.credentials.gid,
        crate::time::realtime(),
    )?;
    guard.publish_metadata(prepared);
    Ok(0)
}

/// `utimensat` 系统调用中单个时间戳（访问时间 `atime` 或修改时间 `mtime`）的更新策略。
#[derive(Clone, Copy, Eq, PartialEq)]
enum FileTimeUpdate {
    /// `UTIME_NOW`：将对应时间戳更新为当前系统实时时钟。
    Now,
    /// `UTIME_OMIT`：保持对应时间戳不变。
    Omit,
    /// 将对应时间戳设置为用户显式指定的秒与纳秒值。
    Value(InodeTime),
}

impl FileTimeUpdate {
    /// 从 16 字节小端序 `timespec` 缓冲区解码时间更新策略；显式纳秒值必须落在 `0..=999_999_999` 范围内。
    fn decode(bytes: &[u8]) -> Result<Self, Errno> {
        let seconds = i64::from_le_bytes(bytes[..8].try_into().expect("timespec seconds"));
        let nanoseconds =
            i64::from_le_bytes(bytes[8..16].try_into().expect("timespec nanoseconds"));
        match nanoseconds {
            UTIME_NOW => Ok(Self::Now),
            UTIME_OMIT => Ok(Self::Omit),
            0..=MAX_NANOSECONDS => Ok(Self::Value(InodeTime {
                seconds,
                nanoseconds: nanoseconds as u32,
            })),
            _ => Err(Errno::InvalidArgument),
        }
    }

    /// 将时间更新策略解析为最终写入 Inode 的可选 [`InodeTime`]（`Now` 替换为本次统一采样的 `now`，`Omit` 映射为 `None`）。
    fn value(self, now: InodeTime) -> Option<InodeTime> {
        match self {
            Self::Now => Some(now),
            Self::Omit => None,
            Self::Value(time) => Some(time),
        }
    }
}

/// 从用户态指针 `address` 一次性拷入 `[atime, mtime]` 两个 16 字节 `timespec` 结构（若指针为空则默认全为 `UTIME_NOW`）。
fn import_file_times(mm: &Mm, address: usize) -> Result<[FileTimeUpdate; 2], Errno> {
    if address == 0 {
        return Ok([FileTimeUpdate::Now; 2]);
    }

    let mut bytes = [0u8; 32];
    copyin(mm, &mut bytes, address)?;
    Ok([
        FileTimeUpdate::decode(&bytes[..16])?,
        FileTimeUpdate::decode(&bytes[16..])?,
    ])
}

/// 解析 `utimensat` 的目标文件 [`Path`]（支持 `futimens` 空路径指针模式、`AT_EMPTY_PATH` 及 `AT_SYMLINK_NOFOLLOW`）。
fn file_times_path(
    runtime: &UserRuntime,
    mm: &Mm,
    dirfd: usize,
    address: usize,
    flags: u32,
) -> Result<Path, Errno> {
    if address == 0 && dirfd as i32 != AT_FDCWD {
        if flags != 0 {
            return Err(Errno::InvalidArgument);
        }
        let file = runtime.files.lookup(descriptor(dirfd)?)?;
        return file.path().cloned().ok_or(Errno::InvalidArgument);
    }
    if flags & !(AT_SYMLINK_NOFOLLOW as u32 | AT_EMPTY_PATH as u32) != 0 {
        return Err(Errno::InvalidArgument);
    }

    let pathname = mm.read_c_string(address, PATH_MAX)?;
    if pathname.is_empty() && flags & AT_EMPTY_PATH as u32 != 0 {
        if dirfd as i32 == AT_FDCWD {
            return Ok(fs_snapshot(runtime).1.cwd().clone());
        }
        let file = runtime.files.lookup(descriptor(dirfd)?)?;
        return file.path().cloned().ok_or(Errno::InvalidArgument);
    }
    let mode = if flags & AT_SYMLINK_NOFOLLOW as u32 != 0 {
        ResolveMode::NoFollow
    } else {
        ResolveMode::Follow
    };
    resolve_bytes(runtime, dirfd, &pathname, mode)
}

/// 处理 `utimensat` 系统调用：更新文件的访问时间（`atime`）与修改时间（`mtime`）。
///
/// # 权限与边界契约
/// - 若 `atime` 与 `mtime` 均为 `UTIME_OMIT`，无需访问文件描述符或路径即可直接返回 `Ok(0)`。
/// - 非 root 且非文件所有者的用户仅允许在两个时间戳同时为 `UTIME_NOW` 且对文件具备写权限（`MAY_WRITE`）时更新时间。
pub(super) fn utimensat(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let mm = user.mm().expect("utimensat has no MM");
    let times = import_file_times(&mm, raw[2])?;
    // Step 1: 若两个时间戳均为 UTIME_OMIT，无需解析路径或修改 ctime，直接成功返回。
    if times == [FileTimeUpdate::Omit; 2] {
        return Ok(0);
    }

    // Step 2: 解析目标路径、检查挂载点写权限，并验证调用者是否具备修改时间戳的权限。
    let runtime = user.runtime().expect("utimensat has no runtime");
    let path = file_times_path(&runtime, &mm, raw[0], raw[1], raw[3] as u32)?;
    path.check_writable()?;
    let inode = path.inode()?;
    let guard = inode.inode_write()?;
    let metadata = guard.metadata_snapshot();
    let uid = runtime.credentials.uid;
    if uid != 0 && uid != metadata.uid {
        if times != [FileTimeUpdate::Now; 2] {
            return Err(Errno::NotPermitted);
        }
        path.check_access(uid, runtime.credentials.gid, MAY_WRITE)?;
    }

    // Step 3: 采样当前实时时钟，生成并原子发布新的 Inode 时间戳元数据。
    let now_ns = crate::time::realtime();
    let now = InodeTime {
        seconds: (now_ns / NSEC_PER_SEC) as i64,
        nanoseconds: (now_ns % NSEC_PER_SEC) as u32,
    };
    let prepared = guard.prepare_times(times[0].value(now), times[1].value(now), now_ns)?;
    guard.publish_metadata(prepared);
    Ok(0)
}

/// 将寄存器传入的原始文件描述符按有符号 32 位整数（`i32`）解码，拒绝负数以防变成巨大的无符号槽位下标。
fn descriptor(raw: usize) -> Result<usize, Errno> {
    let fd = raw as i32;
    if fd < 0 {
        Err(Errno::BadFd)
    } else {
        Ok(fd as usize)
    }
}

/// 短暂获取进程 `runtime.fs` 自旋锁，克隆当前的 [`Vfs`] 与 [`FsContext`] 快照后立即释放锁，避免在后续路径解析或 I/O 期间持锁。
fn fs_snapshot(runtime: &UserRuntime) -> (Arc<Vfs>, FsContext) {
    let current = runtime.fs.lock();
    (current.vfs.clone(), current.context.clone())
}

/// 确定路径解析的起始 [`Path`]：绝对路径从当前 `root` 出发，`AT_FDCWD` 相对路径从 `cwd` 出发，其余相对路径从 `dirfd` 对应的已打开目录出发。
fn resolve_start(
    runtime: &UserRuntime,
    context: &FsContext,
    dirfd: usize,
    pathname: &[u8],
) -> Result<Path, Errno> {
    if pathname.is_empty() {
        return Err(Errno::NotFound);
    }
    if pathname[0] == b'/' {
        return Ok(context.root().clone());
    }
    if dirfd as i32 == AT_FDCWD {
        return Ok(context.cwd().clone());
    }

    let file = runtime.files.lookup(descriptor(dirfd)?)?;
    if file.stat()?.kind != InodeKind::Directory {
        return Err(Errno::NotDirectory);
    }
    Ok(file.path().ok_or(Errno::NotDirectory)?.clone())
}

/// 从用户地址空间读取以 `\0` 结尾的路径字节串，并在当前进程的 VFS 上下文中解析为 [`Path`]。
fn resolve(
    runtime: &UserRuntime,
    mm: &Mm,
    dirfd: usize,
    address: usize,
    mode: ResolveMode,
) -> Result<Path, Errno> {
    let pathname = mm.read_c_string(address, PATH_MAX)?;
    resolve_bytes(runtime, dirfd, &pathname, mode)
}

/// 根据已拷入内核的路径字节切片 `pathname` 和起始目录描述符 `dirfd` 执行 VFS 逐组件路径解析与目录搜索权限检查。
pub(super) fn resolve_bytes(
    runtime: &UserRuntime,
    dirfd: usize,
    pathname: &[u8],
    mode: ResolveMode,
) -> Result<Path, Errno> {
    let (vfs, context) = fs_snapshot(runtime);
    let start = resolve_start(runtime, &context, dirfd, pathname)?;
    vfs.resolve_from(
        &context,
        &start,
        pathname,
        mode,
        Some((runtime.credentials.uid, runtime.credentials.gid)),
    )
    .map_err(Errno::from)
}

/// 将内核 [`FileStat`] 编码为 RISC-V 64 与 LoongArch 64 共用的 128 字节 `asm-generic` 小端序 `struct stat` 字节数组。
fn encode_stat(stat: FileStat) -> Result<[u8; STAT_SIZE], Errno> {
    let size = i64::try_from(stat.size).map_err(|_| Errno::Overflow)?;
    let blocks = i64::try_from(stat.blocks).map_err(|_| Errno::Overflow)?;
    let mut bytes = [0u8; STAT_SIZE];
    put_u64(&mut bytes, 0, stat.device);
    put_u64(&mut bytes, 8, stat.inode);
    put_u32(&mut bytes, 16, u32::from(stat.mode));
    put_u32(&mut bytes, 20, stat.links);
    put_u32(&mut bytes, 24, stat.uid);
    put_u32(&mut bytes, 28, stat.gid);
    put_u64(&mut bytes, 32, stat.rdev);
    put_u64(&mut bytes, 48, size as u64);
    put_u32(&mut bytes, 56, stat.block_size);
    put_u64(&mut bytes, 64, blocks as u64);
    for (offset, time) in [(72, stat.atime), (88, stat.mtime), (104, stat.ctime)] {
        put_u64(&mut bytes, offset, time.seconds as u64);
        put_u64(&mut bytes, offset + 8, u64::from(time.nanoseconds));
    }
    Ok(bytes)
}

/// 将内核 [`FileStat`] 编码为 256 字节小端序 `struct statx` 字节数组，填充基础统计字段（`STATX_BASIC_STATS`）并将保留区域清零。
fn encode_statx(stat: FileStat) -> [u8; STATX_SIZE] {
    let mut bytes = [0u8; STATX_SIZE];
    put_u32(&mut bytes, 0, STATX_BASIC_STATS);
    put_u32(&mut bytes, 4, stat.block_size);
    put_u32(&mut bytes, 16, stat.links);
    put_u32(&mut bytes, 20, stat.uid);
    put_u32(&mut bytes, 24, stat.gid);
    bytes[28..30].copy_from_slice(&stat.mode.to_le_bytes());
    put_u64(&mut bytes, 32, stat.inode);
    put_u64(&mut bytes, 40, stat.size);
    put_u64(&mut bytes, 48, stat.blocks);
    for (offset, time) in [(64, stat.atime), (96, stat.ctime), (112, stat.mtime)] {
        put_u64(&mut bytes, offset, time.seconds as u64);
        put_u32(&mut bytes, offset + 8, time.nanoseconds);
    }
    for (offset, device) in [(128, stat.rdev), (136, stat.device)] {
        let (major, minor) = device_numbers(device);
        put_u32(&mut bytes, offset, major);
        put_u32(&mut bytes, offset + 4, minor);
    }
    bytes
}

/// 将 Linux 64 位 `dev_t` 设备号拆分为 32 位主设备号（`major`）与 32 位次设备号（`minor`）。
fn device_numbers(device: u64) -> (u32, u32) {
    let major = ((device >> 8) & 0xfff) | ((device >> 32) & 0xffff_f000);
    let minor = (device & 0xff) | ((device >> 12) & 0xffff_ff00);
    (major as u32, minor as u32)
}

/// 将 64 位无符号整数按小端序写入字节缓冲区的指定偏移处。
fn put_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// 将 32 位无符号整数按小端序写入字节缓冲区的指定偏移处。
fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// 将目录项中的可选 [`InodeKind`] 转换为 `linux_dirent64::d_type` 常量；若底层文件系统未提供类型则返回 `DT_UNKNOWN`。
fn dirent_type(entry: &DirectoryEntry) -> u8 {
    match entry.kind {
        Some(InodeKind::Regular) => DT_REG,
        Some(InodeKind::Directory) => DT_DIR,
        Some(InodeKind::Symlink) => DT_LNK,
        Some(InodeKind::Character) => DT_CHR,
        Some(InodeKind::Block) => DT_BLK,
        Some(InodeKind::Fifo) => DT_FIFO,
        Some(InodeKind::Socket) => DT_SOCK,
        _ => DT_UNKNOWN,
    }
}
