//! 用户ABI错误在分派边界编码，文件和VM内部仍保留各自具体错误。

use crate::block::BlockError;
use crate::fs::FsError;
use crate::mm::space::{Mm, VmError};
use crate::mm::uaccess::CStringError;
use crate::task::futex::FutexError;
use crate::task::process::ProcessError;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(isize)]
pub(crate) enum Errno {
    NotPermitted = 1,
    NotFound = 2,
    NoProcess = 3,
    Interrupted = 4,
    Io = 5,
    NoDeviceAddress = 6,
    ArgumentsTooBig = 7,
    ExecFormat = 8,
    BadFd = 9,
    NoChild = 10,
    Again = 11,
    NoMemory = 12,
    Permission = 13,
    BadAddress = 14,
    Busy = 16,
    Exists = 17,
    CrossDevice = 18,
    NoDevice = 19,
    NotDirectory = 20,
    IsDirectory = 21,
    InvalidArgument = 22,
    TooManyFiles = 24,
    NotTty = 25,
    TextBusy = 26,
    FileTooLarge = 27,
    NoSpace = 28,
    IllegalSeek = 29,
    ReadOnly = 30,
    TooManyLinks = 31,
    BrokenPipe = 32,
    Domain = 33,
    Range = 34,
    NameTooLong = 36,
    NoSyscall = 38,
    NotEmpty = 39,
    TooManySymlinks = 40,
    Overflow = 75,
    NotSocket = 88,
    DestinationRequired = 89,
    MessageTooLarge = 90,
    ProtocolOptionNotSupported = 92,
    ProtocolNotSupported = 93,
    SocketTypeNotSupported = 94,
    Unsupported = 95,
    AddressFamilyNotSupported = 97,
    AddressInUse = 98,
    AddressNotAvailable = 99,
    NetworkUnreachable = 101,
    ConnectionReset = 104,
    AlreadyConnected = 106,
    NotConnected = 107,
    TimedOut = 110,
    ConnectionRefused = 111,
    AlreadyInProgress = 114,
    InProgress = 115,
    /// 仅供内部重启分派使用，任何用户返回都编码为 EINTR。
    RestartSyscall = 512,
}

pub(super) type SysResult = Result<isize, Errno>;

impl Errno {
    /// 普通返回只交付 Linux errno，内部重启标记不得泄露到用户 ABI。
    pub(super) fn result(self) -> isize {
        match self {
            Self::RestartSyscall => -(Self::Interrupted as isize),
            other => -(other as isize),
        }
    }
}

impl From<FsError> for Errno {
    /// 保留文件与 socket 的真实错误；可重启中断仅转换为内部分派标记。
    fn from(error: FsError) -> Self {
        match error {
            FsError::NoMemory | FsError::Io(BlockError::NoMemory) => Self::NoMemory,
            FsError::NoSpace => Self::NoSpace,
            FsError::TextBusy => Self::TextBusy,
            FsError::FileTooLarge => Self::FileTooLarge,
            FsError::BadAddress => Self::BadAddress,
            FsError::Permission => Self::Permission,
            FsError::NotPermitted => Self::NotPermitted,
            FsError::Unsupported => Self::Unsupported,
            FsError::NotFound | FsError::MissingRoot => Self::NotFound,
            FsError::Exists => Self::Exists,
            FsError::NotEmpty => Self::NotEmpty,
            FsError::CrossDevice => Self::CrossDevice,
            FsError::TooManyLinks => Self::TooManyLinks,
            FsError::NameTooLong => Self::NameTooLong,
            FsError::NotDirectory => Self::NotDirectory,
            FsError::InvalidName | FsError::InvalidOperation => Self::InvalidArgument,
            FsError::Busy => Self::Busy,
            FsError::TooManySymlinks => Self::TooManySymlinks,
            FsError::BadFd => Self::BadFd,
            FsError::TooManyFiles => Self::TooManyFiles,
            FsError::ReadOnly => Self::ReadOnly,
            FsError::IsDirectory => Self::IsDirectory,
            FsError::IllegalSeek => Self::IllegalSeek,
            FsError::NotTty => Self::NotTty,
            FsError::NoDevice => Self::NoDevice,
            FsError::NoDeviceAddress => Self::NoDeviceAddress,
            FsError::NoProcess | FsError::NoSuchRoute => Self::NoProcess,
            FsError::DeviceIo => Self::Io,
            FsError::AddressFamilyNotSupported => Self::AddressFamilyNotSupported,
            FsError::AddressInUse => Self::AddressInUse,
            FsError::AddressNotAvailable => Self::AddressNotAvailable,
            FsError::NetworkUnreachable => Self::NetworkUnreachable,
            FsError::ConnectionReset => Self::ConnectionReset,
            FsError::AlreadyConnected => Self::AlreadyConnected,
            FsError::NotConnected => Self::NotConnected,
            FsError::ConnectionRefused => Self::ConnectionRefused,
            FsError::AlreadyInProgress => Self::AlreadyInProgress,
            FsError::InProgress => Self::InProgress,
            FsError::MessageTooLarge => Self::MessageTooLarge,
            FsError::NotSocket => Self::NotSocket,
            FsError::DestinationRequired => Self::DestinationRequired,
            FsError::WouldBlock => Self::Again,
            FsError::BrokenPipe => Self::BrokenPipe,
            FsError::Interrupted => Self::RestartSyscall,
            FsError::InterruptedNoRestart => Self::Interrupted,
            FsError::Corrupt | FsError::Checksum | FsError::LoadAborted | FsError::Io(_) => {
                Self::Io
            }
        }
    }
}

impl From<VmError> for Errno {
    /// 映射操作区分不合法范围、实际空洞和最大权限，uaccess另按BadAddress交付。
    fn from(error: VmError) -> Self {
        match error {
            VmError::NoMemory | VmError::Unmapped => Self::NoMemory,
            VmError::InvalidRange => Self::InvalidArgument,
            VmError::Occupied => Self::Exists,
            VmError::Permission => Self::Permission,
            VmError::NotMappable => Self::NoDevice,
            VmError::BeyondEof => Self::BadAddress,
            VmError::File(error) => error.into(),
        }
    }
}

impl From<FutexError> for Errno {
    /// futex参数错误与真实用户访问错误分开，空洞/权限不按一般mmap错误编码。
    fn from(error: FutexError) -> Self {
        match error {
            FutexError::Mismatch => Self::Again,
            FutexError::InvalidArgument => Self::InvalidArgument,
            FutexError::Vm(VmError::NoMemory) => Self::NoMemory,
            FutexError::Vm(VmError::File(error)) => error.into(),
            FutexError::Vm(_) => Self::BadAddress,
        }
    }
}

impl From<CStringError> for Errno {
    /// 路径保原始字节；无NUL、分配失败和不可访问页各有明确返回。
    fn from(error: CStringError) -> Self {
        match error {
            CStringError::Vm(_) => Self::BadAddress,
            CStringError::NoMemory => Self::NoMemory,
            CStringError::Unterminated => Self::NameTooLong,
        }
    }
}

impl From<ProcessError> for Errno {
    /// 身份准备失败不发表半个child，号码耗尽与对象分配失败分开返回。
    fn from(error: ProcessError) -> Self {
        match error {
            ProcessError::InvalidCapacity => Self::InvalidArgument,
            ProcessError::NoMemory => Self::NoMemory,
            ProcessError::NoPid => Self::Again,
            ProcessError::InvalidParent => Self::NoProcess,
            ProcessError::NoChild => Self::NoChild,
            ProcessError::Busy => Self::Again,
            ProcessError::NoProcess => Self::NoProcess,
            ProcessError::Permission => Self::NotPermitted,
            ProcessError::Access => Self::Permission,
            ProcessError::InvalidArgument => Self::InvalidArgument,
        }
    }
}

/// 完整交付已初始化ABI字节后才算成功，故障不撤回实际写入前缀。
pub(super) fn copyout(mm: &Mm, address: usize, bytes: &[u8]) -> Result<(), Errno> {
    let progress = mm.copyout(address, bytes);
    if progress.copied == bytes.len() && progress.error.is_none() {
        Ok(())
    } else {
        Err(Errno::BadAddress)
    }
}

/// 完整取得本次ABI字节，不以部分初始化内核值解释用户对象。
pub(super) fn copyin(mm: &Mm, bytes: &mut [u8], address: usize) -> Result<(), Errno> {
    let progress = mm.copyin(bytes, address);
    if progress.copied == bytes.len() && progress.error.is_none() {
        Ok(())
    } else {
        Err(Errno::BadAddress)
    }
}
