//! 虚拟文件系统（VFS）顶层模块。
//!
//! # 模块职责与架构定位
//! 本模块作为内核文件系统的总门面（Facade），组织并重导出下属子模块的核心抽象：
//! - **上游调用者**：系统调用层（`syscall`）、进程管理（`task`，维护进程打开文件表 `FileTable` 与文件系统上下文 `FsContext`）、ELF 加载器以及系统启动挂载流程。
//! - **下游子系统**：页缓存（`mm::folio`）与块设备 I/O 层（`block`），为磁盘文件系统（`ext4`）提供按页缓存的数据读写与脏页回写通道。
//!
//! # 核心设计与分层架构
//! ```text
//! +-----------------------------------------------------------------------+
//! |                     系统调用与内核消费者 (Syscall / Task)               |
//! +-----------------------------------------------------------------------+
//!                                     |
//!                                     v
//! +-----------------------------------------------------------------------+
//! |  打开文件与路径解析层: FileTable / FileRef / Vfs / Path / DentryCache  |
//! +-----------------------------------------------------------------------+
//!                                     |
//!                                     v
//! +-----------------------------------------------------------------------+
//! |  索引节点与并发控制层: InodeCache / InodeRef (读写/目录睡眠锁、引用计数) |
//! +-----------------------------------------------------------------------+
//!          |                          |                          |
//!          v                          v                          v
//! +-----------------+      +--------------------+      +------------------+
//! |   磁盘文件系统   |      |    内存与伪文件系统  |      |  字符/管道/多路复用 |
//! |      (ext4)     |      | (memfs/devfs/proc) |      | (tty/pipe/epoll) |
//! +-----------------+      +--------------------+      +------------------+
//!          |
//!          v
//! +-----------------------------------------------------------------------+
//! |          页缓存与回写引擎 (mm::folio / readahead / writeback)          |
//! +-----------------------------------------------------------------------+
//!          |
//!          v
//! +-----------------------------------------------------------------------+
//! |                       块设备抽象层 (block::BlockView)                  |
//! +-----------------------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 文件系统各层严格遵守自顶向下的加锁顺序，严禁反向获取以免发生死锁：
//! 1. **进程文件描述符表锁**（`FileTable` 内部自旋锁）或 **VFS 挂载表锁**（`Vfs` 挂载自旋锁）。
//! 2. **目录操作睡眠锁**（`Inode` 目录读写锁，跨命名空间修改时遵循父目录在先、特定双目录顺序的规则）。
//! 3. **Dentry 缓存索引自旋锁**（`DentryCache::entries`）$\rightarrow$ **单个 Dentry 拓扑自旋锁**（`Dentry::topology`）。
//! 4. **Inode 缓存索引自旋锁**（`InodeCache`）与 **Inode 元数据/状态自旋锁**。
//! 5. **页缓存（Folio）状态锁与块层提交队列锁**（任何自旋锁临界区内严禁发起睡眠等待或块设备同步 I/O）。

pub(crate) mod background;
mod dentry;
pub(crate) mod devfs;
pub(crate) mod epoll;
pub(crate) mod ext4;
mod file;
mod file_table;
mod inode;
pub(crate) mod kmsg;
#[cfg(feature = "ktest")]
pub(crate) mod ktest;
pub(crate) mod ktrace;
pub(crate) mod memfs;
mod path;
pub(crate) mod pipe;
pub(crate) mod poll;
pub(crate) mod procfs;
pub(crate) mod readahead;
pub(crate) mod system;
pub(crate) mod transfer;
pub(crate) mod tty;
mod types;
pub(crate) mod writeback;

pub use dentry::{DentryCache, DentryRef};
pub use ext4::{BlockMapping, Ext4Volume};
pub use file::{
    open_path, open_path_with_flags, stat_path, FileIoContext, FileLike, FileRef, FileStat,
    SeekFrom,
};
pub(crate) use file::{
    set_nonblocking, set_status_flags, O_ACCMODE, O_APPEND, O_CLOEXEC, O_CREAT, O_DIRECTORY,
    O_EXCL, O_LARGEFILE, O_NOCTTY, O_NOFOLLOW, O_NONBLOCK, O_RDONLY, O_RDWR, O_SYNC, O_TRUNC,
    O_WRONLY,
};
pub use file::{ReadBuffer, UserIoVector, WriteBuffer};
#[cfg(feature = "ktest")]
pub use file_table::OpenOptions;
pub use file_table::{DescriptorFlags, FileTable};
pub(crate) use inode::{initialize_inode_eviction, DirectoryWriteGuard, InodeWeak};
pub use inode::{ExecAccess, InodeCache, InodeMetadata, InodeRef, WriteAccess};
pub use memfs::Filesystem;
pub use path::{FsContext, Path, ResolveMode, Vfs};
#[cfg(feature = "ktest")]
pub use poll::PollMask;
pub use poll::Pollable;
pub(crate) use types::{DirectoryCursor, DirectoryPosition};
pub use types::{DirectoryEntry, InodeKind, InodeTime};
pub(crate) use types::{
    MAY_EXEC, MAY_READ, MAY_WRITE, NAME_MAX, PATH_MAX, S_IALLUGO, S_IFBLK, S_IFCHR, S_IFDIR,
    S_IFIFO, S_IFLNK, S_IFREG, S_IFSOCK, S_IRWXUGO,
};

use crate::block::BlockError;

/// 文件系统、I/O 流与套接字操作统一返回的错误类型。
///
/// 涵盖内存与磁盘空间不足、磁盘元数据损坏、路径与名字解析错误、权限与状态冲突、
/// 套接字连接与路由错误以及底层块设备 I/O 错误，供系统调用边界映射为标准 POSIX `errno`。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FsError {
    /// 内核内存分配失败或固定容量的缓存（如 Dentry/Inode 缓存）已满（对应 `ENOMEM`）。
    NoMemory,
    /// 文件系统磁盘块或 Inode 配额耗尽（对应 `ENOSPC`）。
    NoSpace,
    /// 试图写入正在作为可执行映像运行的文件，或映射正在被写入的可执行文件（对应 `ETXTBSY`）。
    TextBusy,
    /// 文件偏移或大小超出文件系统格式支持的上限（对应 `EFBIG`）。
    FileTooLarge,
    /// 磁盘上的文件系统结构（超级块、Extent 树、目录项等）违反格式不变量（对应 `EUCLEAN` / `EIO`）。
    Corrupt,
    /// 磁盘元数据或数据块的校验和（如 ext4 CRC32C）校验失败。
    Checksum,
    /// 文件系统包含当前内核不支持的特性标志或操作模式（对应 `EOPNOTSUPP` / `ENOSYS`）。
    Unsupported,
    /// 路径组件、文件或目录不存在（对应 `ENOENT`）。
    NotFound,
    /// 创建文件、目录或挂载点时目标名字已存在（对应 `EEXIST`）。
    Exists,
    /// 删除或覆盖目录时目标目录非空（对应 `ENOTEMPTY`）。
    NotEmpty,
    /// 跨不同挂载实例或文件系统执行硬链接或重命名（对应 `EXDEV`）。
    CrossDevice,
    /// Inode 的硬链接计数已达上限（对应 `EMLINK`）。
    TooManyLinks,
    /// 单个路径组件超过 `NAME_MAX` 或总路径超过 `PATH_MAX`（对应 `ENAMETOOLONG`）。
    NameTooLong,
    /// 操作违反系统安全策略或对象固定属性，如对目录执行 `unlink`（对应 `EPERM`）。
    NotPermitted,
    /// 启动参数未指定必需的根设备/根目录，或指定的根设备不存在。
    MissingRoot,
    /// 路径中间组件或要求为目录的目标实际不是目录（对应 `ENOTDIR`）。
    NotDirectory,
    /// 路径组件为空、包含非法字符（如 `/` 或 `\0`）或属于禁止修改的 `.`/`..`（对应 `EINVAL`）。
    InvalidName,
    /// 调用参数、标志位组合或对象当前状态不支持该操作（对应 `EINVAL`）。
    InvalidOperation,
    /// 目标挂载点、目录或资源正被占用，无法卸载或删除（对应 `EBUSY`）。
    Busy,
    /// 符号链接展开层数超过上限，存在循环链接风险（对应 `ELOOP`）。
    TooManySymlinks,
    /// 文件描述符编号无效，或打开模式不允许当前读写操作（对应 `EBADF`）。
    BadFd,
    /// 进程打开的文件描述符数量已达到 `RLIMIT_NOFILE` 限制（对应 `EMFILE`）。
    TooManyFiles,
    /// 试图在只读挂载卷或只读设备上执行修改操作（对应 `EROFS`）。
    ReadOnly,
    /// 试图按普通文件方式打开或写入目录对象（对应 `EISDIR`）。
    IsDirectory,
    /// 在管道、FIFO、套接字或终端等不可随机寻址的流对象上调用 `lseek`（对应 `ESPIPE`）。
    IllegalSeek,
    /// 对非终端设备发起终端专属的 `ioctl` 控制操作（对应 `ENOTTY`）。
    NotTty,
    /// 设备节点对应的主次设备号未注册底层驱动程序（对应 `ENODEV`）。
    NoDevice,
    /// 特殊设备地址无效或开启条件不满足，如非阻塞只写打开无读者的 FIFO（对应 `ENXIO`）。
    NoDeviceAddress,
    /// 终端前台进程组或目标进程不存在（对应 `ESRCH`）。
    NoProcess,
    /// 网络数据包找不到到达目标地址的路由表项（对应 `EHOSTUNREACH` / `ENETUNREACH`）。
    NoSuchRoute,
    /// 字符设备、块设备或外设控制器发生底层硬件 I/O 故障（对应 `EIO`）。
    DeviceIo,
    /// 套接字不支持所请求的地址族（对应 `EAFNOSUPPORT`）。
    AddressFamilyNotSupported,
    /// 尝试绑定的本地网络地址与端口已被占用（对应 `EADDRINUSE`）。
    AddressInUse,
    /// 请求绑定的源地址不属于本机接口或无可用临时端口（对应 `EADDRNOTAVAIL`）。
    AddressNotAvailable,
    /// 目标网络不可达（对应 `ENETUNREACH`）。
    NetworkUnreachable,
    /// 对端重置了网络连接（对应 `ECONNRESET`）。
    ConnectionReset,
    /// 在已建立连接的套接字上重复发起连接（对应 `EISCONN`）。
    AlreadyConnected,
    /// 在未建立连接的套接字上执行需要连接的收发或查询操作（对应 `ENOTCONN`）。
    NotConnected,
    /// 远端主机拒绝网络连接请求（对应 `ECONNREFUSED`）。
    ConnectionRefused,
    /// 非阻塞套接字上已有正在进行的连接操作（对应 `EALREADY`）。
    AlreadyInProgress,
    /// 非阻塞连接操作已启动但尚未完成（对应 `EINPROGRESS`）。
    InProgress,
    /// 数据报长度超出协议或发送缓冲区允许的最大报文尺寸（对应 `EMSGSIZE`）。
    MessageTooLarge,
    /// 文件描述符指向的对象不是套接字（对应 `ENOTSOCK`）。
    #[expect(
        dead_code,
        reason = "文件错误字典保留套接字类型错误并准确映射 ENOTSOCK；当前 fd 类型校验直接在 syscall 边界返回 errno"
    )]
    NotSocket,
    /// 未连接的无连接套接字（如 UDP）在发送时未提供目的地址（对应 `EDestAddrReq`）。
    DestinationRequired,
    /// 非阻塞操作当前无法立即完成，需等待读写就绪（对应 `EAGAIN` / `EWOULDBLOCK`）。
    WouldBlock,
    /// 向已无读端打开的管道或已关闭写方向的套接字写入数据（对应 `EPIPE`）。
    BrokenPipe,
    /// 阻塞等待被待处理的信号中断，可根据 `SA_RESTART` 策略决定是否自动重启（对应 `EINTR`）。
    Interrupted,
    /// 带超时的套接字阻塞等待被信号中断，且已消耗部分超时预算，不可自动重启系统调用（对应 `EINTR`）。
    InterruptedNoRestart,
    /// 用户态传入的缓冲区虚拟地址无效、未映射或越界（对应 `EFAULT`）。
    BadAddress,
    /// 当前进程凭证（UID/GID）不具备目标 Inode 所需的读、写或执行权限（对应 `EACCES`）。
    Permission,
    /// 单飞（Singleflight）并发加载 Dentry 或 Inode 时，负责加载的任务异常中止。
    LoadAborted,
    /// 底层块设备层返回的物理 I/O、越界或只读错误。
    Io(BlockError),
}

impl FsError {
    /// 将页缓存（Folio）填充期间发生的磁盘格式或底层 I/O 错误转换为 `BlockError`，
    /// 以便写入共享的 `Folio` 状态槽并唤醒并发等待同一页的其他读者。
    ///
    /// # 调用上下文与约束
    /// 仅限在页缓存填充（Page Fill）失败路径上调用。名字解析、权限检查、套接字控制等与页内容加载无关的错误严禁写入 `Folio`，否则会触发 panic。
    pub(crate) fn folio_error(self) -> BlockError {
        match self {
            Self::NoMemory => BlockError::NoMemory,
            Self::Corrupt => BlockError::Corrupt,
            Self::Checksum => BlockError::Checksum,
            Self::Unsupported => BlockError::Unsupported,
            Self::Io(error) => error,
            Self::BadFd
            | Self::TooManyFiles
            | Self::NoSpace
            | Self::TextBusy
            | Self::FileTooLarge
            | Self::ReadOnly
            | Self::IsDirectory
            | Self::BadAddress
            | Self::IllegalSeek
            | Self::NotTty
            | Self::NoDevice
            | Self::NoDeviceAddress
            | Self::NoProcess
            | Self::NoSuchRoute
            | Self::DeviceIo
            | Self::AddressFamilyNotSupported
            | Self::AddressInUse
            | Self::AddressNotAvailable
            | Self::NetworkUnreachable
            | Self::ConnectionReset
            | Self::AlreadyConnected
            | Self::NotConnected
            | Self::ConnectionRefused
            | Self::AlreadyInProgress
            | Self::InProgress
            | Self::MessageTooLarge
            | Self::NotSocket
            | Self::DestinationRequired
            | Self::WouldBlock
            | Self::BrokenPipe
            | Self::Interrupted
            | Self::InterruptedNoRestart
            | Self::Permission
            | Self::NotFound
            | Self::Exists
            | Self::NotEmpty
            | Self::CrossDevice
            | Self::TooManyLinks
            | Self::NameTooLong
            | Self::NotPermitted
            | Self::MissingRoot
            | Self::NotDirectory
            | Self::InvalidName
            | Self::InvalidOperation
            | Self::Busy
            | Self::TooManySymlinks
            | Self::LoadAborted => panic!("non-fill error published to a folio: {:?}", self),
        }
    }
}

impl From<BlockError> for FsError {
    /// 将底层块设备或页缓存记录的 `BlockError` 还原为文件系统层的 `FsError`。
    ///
    /// 内存不足、数据损坏、校验和错误与不支持的特性会还原为对应的顶层变体，其余块层错误统一包装在 `FsError::Io` 中。
    fn from(error: BlockError) -> Self {
        match error {
            BlockError::NoMemory => Self::NoMemory,
            BlockError::Corrupt => Self::Corrupt,
            BlockError::Checksum => Self::Checksum,
            BlockError::Unsupported => Self::Unsupported,
            other => Self::Io(other),
        }
    }
}
