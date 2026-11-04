//! 文件内容、唯一 inode 身份与元数据视图共用原页缓存和块 I/O。

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

/// 文件能力、socket地址、格式、名字与块I/O错误保留各自原因，不以零内容代替失败。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FsError {
    NoMemory,
    NoSpace,
    TextBusy,
    FileTooLarge,
    Corrupt,
    Checksum,
    Unsupported,
    NotFound,
    Exists,
    NotEmpty,
    CrossDevice,
    TooManyLinks,
    NameTooLong,
    NotPermitted,
    MissingRoot,
    NotDirectory,
    InvalidName,
    InvalidOperation,
    Busy,
    TooManySymlinks,
    BadFd,
    TooManyFiles,
    ReadOnly,
    IsDirectory,
    IllegalSeek,
    NotTty,
    NoDevice,
    NoDeviceAddress,
    NoProcess,
    NoSuchRoute,
    DeviceIo,
    AddressFamilyNotSupported,
    AddressInUse,
    AddressNotAvailable,
    NetworkUnreachable,
    ConnectionReset,
    AlreadyConnected,
    NotConnected,
    ConnectionRefused,
    AlreadyInProgress,
    InProgress,
    MessageTooLarge,
    NotSocket,
    /// UDP 发送既无本次目的地址，也无默认对端。
    DestinationRequired,
    WouldBlock,
    BrokenPipe,
    Interrupted,
    /// 有限 socket 等待已消耗本次期限，信号返回不能重新开始等待。
    InterruptedNoRestart,
    BadAddress,
    Permission,
    LoadAborted,
    Io(BlockError),
}

impl FsError {
    /// 将页填充的实际格式或 I/O 错误写入共享 folio，等待者读取同一原因。
    /// 根选择、名字查找、文件能力、socket 控制与消息错误和 inode 加载取消不属于页填充结果。
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
    /// 页填充保留的格式错误恢复到文件接口，其余错误保持真实块层原因。
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
