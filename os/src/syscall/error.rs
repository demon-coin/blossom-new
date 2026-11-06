//! 系统调用 ABI 错误编码（[`Errno`]）、跨子系统错误转换与定长用户空间拷贝辅助。
//!
//! # 模块职责与架构定位
//! 本模块位于内核各子系统（文件系统、虚拟内存、进程管理、Futex 等）与系统调用返回边界之间：
//! - **上游调用者**：`syscall` 顶层分发器及下属各系统调用实现模块（`file`、`memory`、`task`、`net`、`poll`、`signal`、`time` 等），以及块页缓存回写错误记录（`cache`）。
//! - **下游依赖**：接收来自 `fs::FsError`、`mm::space::VmError`、`mm::uaccess::CStringError`、`task::futex::FutexError`、`task::process::ProcessError` 等子系统的强类型领域错误，统一映射为符合 Linux 用户态 ABI 的正数错误码枚举 [`Errno`]。
//!
//! # 核心设计：错误编码流转与内部重启哨兵隔离
//! 内核子系统内部不使用笼统的整数 `errno`，而是各自保留语义明确的 Rust 枚举错误；仅在跨越系统调用边界时才执行单向映射并取反写入返回值寄存器 `a0`：
//!
//! ```text
//!   FsError / VmError / FutexError / CStringError / ProcessError
//!                                │
//!                                ▼  (From<...> 语义转换)
//!                     syscall::error::Errno (正整数判别值)
//!                        │                         │
//!       常规错误 (1..=115)│                         │ 可重启中断哨兵 (RestartSyscall = 512)
//!                        │                         ├──► 若该调用支持 SA_RESTART 分发拦截:
//!                        │                         │    转为 Outcome::Restart 交由信号层裁决
//!                        ▼                         ▼
//!                   Errno::result() 统一取负编码写入寄存器 a0
//!         -(errno as isize)                  兜底降级为 -EINTR (-4)，绝不向用户态泄露 -512
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块中的 `From` 错误转换与 `Errno::result` 均为纯值计算，不涉及任何锁，可在任意上下文调用。
//! - [`copyin`] 与 [`copyout`] 底层通过 [`Mm`] 访问用户页表，遇到未映射或写时复制（CoW）页时会获取地址空间锁并可能触发缺页睡眠等待磁盘 I/O；因此**严禁在持有任何自旋锁（`SpinLock`）或关中断临界区内调用 [`copyin`]/[`copyout`]**。

use crate::block::BlockError;
use crate::fs::FsError;
use crate::mm::space::{Mm, VmError};
use crate::mm::uaccess::CStringError;
use crate::task::futex::FutexError;
use crate::task::process::ProcessError;

/// 系统调用层使用的 Linux 标准错误码枚举（正数编码）与内部重启控制标记。
///
/// 各变体的判别值为正整数（对应 POSIX `E*` 常量绝对值），通过 [`Errno::result`] 转换为系统调用寄存器所需的负数返回值。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(isize)]
pub(crate) enum Errno {
    /// 操作不被允许，通常因缺少特权或所有权不匹配（`EPERM = 1`）。
    NotPermitted = 1,
    /// 目标文件、目录或路径组件不存在（`ENOENT = 2`）。
    NotFound = 2,
    /// 指定的进程、线程或进程组不存在（`ESRCH = 3`）。
    NoProcess = 3,
    /// 阻塞中的系统调用被信号中断且不自动重启（`EINTR = 4`）。
    Interrupted = 4,
    /// 底层块设备、终端或文件系统元数据读写发生物理/逻辑 I/O 错误（`EIO = 5`）。
    Io = 5,
    /// 设备不存在或请求的设备地址/通道无效，如读取无写端且无数据的 FIFO 或越界设备（`ENXIO = 6`）。
    NoDeviceAddress = 6,
    /// `execve` 参数列表与环境变量总字节数超出上限（`E2BIG = 7`）。
    ArgumentsTooBig = 7,
    /// 可执行文件 ELF 魔数、架构或头部格式不合法（`ENOEXEC = 8`）。
    ExecFormat = 8,
    /// 文件描述符无效、未打开，或与请求的操作方向（读/写）不兼容（`EBADF = 9`）。
    BadFd = 9,
    /// 当前进程没有符合等待条件的子进程（`ECHILD = 10`）。
    NoChild = 10,
    /// 资源暂时不可用、非阻塞 I/O 将会阻塞（`EWOULDBLOCK`），或 Futex 期望值不匹配（`EAGAIN = 11`）。
    Again = 11,
    /// 内核堆/物理页分配失败，或进程虚拟地址空间映射数量/大小超限（`ENOMEM = 12`）。
    NoMemory = 12,
    /// 文件或目录访问权限不足，或尝试违反映射最大保护权限（`EACCES = 13`）。
    Permission = 13,
    /// 用户空间指针越界、未映射或缺少所需读写权限（`EFAULT = 14`）。
    BadAddress = 14,
    /// 目标设备、挂载点或文件资源正在使用中无法卸载或独占修改（`EBUSY = 16`）。
    Busy = 16,
    /// 创建文件、目录或固定不覆盖映射（`MAP_FIXED_NOREPLACE`）时目标已存在（`EEXIST = 17`）。
    Exists = 17,
    /// 尝试跨不同挂载点/文件系统创建硬链接或重命名（`EXDEV = 18`）。
    CrossDevice = 18,
    /// 指定的文件系统类型不存在，或底层对象不支持 `mmap` 内存映射（`ENODEV = 19`）。
    NoDevice = 19,
    /// 路径解析中间组件不是目录，或对非目录文件执行目录专属操作（`ENOTDIR = 20`）。
    NotDirectory = 20,
    /// 尝试对目录执行仅限普通文件的写入或截断等操作（`EISDIR = 21`）。
    IsDirectory = 21,
    /// 系统调用标志位、长度、对齐或偏移量等参数不合法（`EINVAL = 22`）。
    InvalidArgument = 22,
    /// 当前进程打开的文件描述符数量已达进程软/硬限制上限（`EMFILE = 24`）。
    TooManyFiles = 24,
    /// 对非终端文件描述符发起了终端专属 `ioctl` 控制命令（`ENOTTY = 25`）。
    NotTty = 25,
    /// 尝试以写方式打开或截断正在作为可执行映像运行的文件，或执行正在被写入的文件（`ETXTBSY = 26`）。
    TextBusy = 26,
    /// 文件写入偏移或截断长度超出文件系统支持的最大单文件字节上限（`EFBIG = 27`）。
    FileTooLarge = 27,
    /// 磁盘数据块、Inode 或内存文件系统容量已耗尽（`ENOSPC = 28`）。
    NoSpace = 28,
    /// 尝试在管道（Pipe）、FIFO 或套接字等流式对象上执行随机定位 `lseek`/`pread`/`pwrite`（`ESPIPE = 29`）。
    IllegalSeek = 29,
    /// 尝试在以只读方式挂载的文件系统上创建、删除或修改文件（`EROFS = 30`）。
    ReadOnly = 30,
    /// 单个 Inode 的硬链接计数已达文件系统上限（`EMLINK = 31`）。
    TooManyLinks = 31,
    /// 向所有读端均已关闭的管道或套接字写入数据（`EPIPE = 32`）。
    BrokenPipe = 32,
    /// 数值参数超出操作允许的定义域（`EDOM = 33`）。
    Domain = 33,
    /// 输出结果或路径字符串超出调用方提供的缓冲区长度（`ERANGE = 34`）。
    Range = 34,
    /// 单个文件名组件或完整路径字节长度超出上限（`ENAMETOOLONG = 36`）。
    NameTooLong = 36,
    /// 系统调用编号未实现，或子功能在当前配置下未提供（`ENOSYS = 38`）。
    NoSyscall = 38,
    /// 尝试删除或覆盖仍包含其他条目的非空目录（`ENOTEMPTY = 39`）。
    NotEmpty = 39,
    /// 路径解析过程中符号链接嵌套跳转层数超限，或 `O_NOFOLLOW` 命中符号链接（`ELOOP = 40`）。
    TooManySymlinks = 40,
    /// 计算出的偏移量、文件大小或统计字段超出目标数据类型可表示范围（`EOVERFLOW = 75`）。
    Overflow = 75,
    /// 对非套接字文件描述符执行了套接字专属系统调用（`ENOTSOCK = 88`）。
    NotSocket = 88,
    /// 在未绑定默认对端的无连接套接字（如 UDP）上发送数据时未提供目标地址（`EDESTADDRREQ = 89`）。
    DestinationRequired = 89,
    /// 数据报长度超出底层协议或发送缓冲区允许的单包最大尺寸（`EMSGSIZE = 90`）。
    MessageTooLarge = 90,
    /// 请求的套接字选项（`getsockopt`/`setsockopt`）在当前协议层不受支持（`ENOPROTOOPT = 92`）。
    ProtocolOptionNotSupported = 92,
    /// 指定的传输层协议在当前地址族下不受支持（`EPROTONOSUPPORT = 93`）。
    ProtocolNotSupported = 93,
    /// 指定的套接字类型（如 `SOCK_RAW`）不受支持（`ESOCKTNOSUPPORT = 94`）。
    SocketTypeNotSupported = 94,
    /// 目标文件、文件系统或套接字传输端点不支持请求的操作（`EOPNOTSUPP` / `ENOTSUP = 95`）。
    Unsupported = 95,
    /// 地址结构体中的地址族（`AF_*`）不被当前内核或套接字协议支持（`EAFNOSUPPORT = 97`）。
    AddressFamilyNotSupported = 97,
    /// 请求绑定的本地 IP 地址与端口组合已被占用且不允许复用（`EADDRINUSE = 98`）。
    AddressInUse = 98,
    /// 请求绑定的本地地址不属于本机接口，或临时端口已耗尽（`EADDRNOTAVAIL = 99`）。
    AddressNotAvailable = 99,
    /// 找不到前往目标网络或主机的可用路由（`ENETUNREACH = 101`）。
    NetworkUnreachable = 101,
    /// 对端发送 RST 复位报文导致现有连接异常断开（`ECONNRESET = 104`）。
    ConnectionReset = 104,
    /// 面向连接的套接字已处于建立连接状态，不能重复发起连接（`EISCONN = 106`）。
    AlreadyConnected = 106,
    /// 套接字尚未建立连接，无法读写数据或查询对端地址（`ENOTCONN = 107`）。
    NotConnected = 107,
    /// 阻塞等待（如 Futex、套接字连接或收包）超过指定时限仍未完成（`ETIMEDOUT = 110`）。
    TimedOut = 110,
    /// 目标主机端口无监听者或主动拒绝连接请求（`ECONNREFUSED = 111`）。
    ConnectionRefused = 111,
    /// 非阻塞套接字上已有前一次连接请求正在进行中（`EALREADY = 114`）。
    AlreadyInProgress = 114,
    /// 非阻塞套接字连接请求已发起但三次握手尚未完成（`EINPROGRESS = 115`）。
    InProgress = 115,
    /// 内核内部专用的系统调用重启哨兵（对应 Linux 内核内部的 `ERESTARTSYS = 512`）。
    /// 供可重启系统调用向分发器传递中断信号；若落入普通返回路径，[`Errno::result`] 会将其统一编码为 `-EINTR`，绝不暴露给用户态。
    RestartSyscall = 512,
}

/// 常规系统调用处理函数的统一返回类型：成功返回非负 `isize`（如字节数、文件描述符或 0），失败返回 [`Errno`]。
pub(super) type SysResult = Result<isize, Errno>;

impl Errno {
    /// 将 [`Errno`] 编码为系统调用寄存器 `a0` 使用的负数 ABI 返回值。
    ///
    /// 内部专用的 [`Self::RestartSyscall`] 会在此处安全折叠为 `-EINTR`（`-4`），确保内部控制码不会泄露到用户空间。
    pub(super) fn result(self) -> isize {
        match self {
            Self::RestartSyscall => -(Self::Interrupted as isize),
            other => -(other as isize),
        }
    }
}

impl From<FsError> for Errno {
    /// 将虚拟文件系统与网络套接字层的 [`FsError`] 映射为系统调用 [`Errno`]。
    ///
    /// 其中零进度下可按 `SA_RESTART` 策略恢复的中断（[`FsError::Interrupted`]）转换为内部哨兵 [`Self::RestartSyscall`]，
    /// 而明确不可自动重启的中断（[`FsError::InterruptedNoRestart`]）直接转换为 [`Self::Interrupted`]。
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
    /// 将虚拟内存管理操作（`mmap`、`munmap`、`mprotect`、`msync`）返回的 [`VmError`] 映射为 [`Errno`]。
    ///
    /// 注意：本转换面向地址空间管理类系统调用（例如未映射区间在 `mprotect`/`msync` 下对应 `ENOMEM`）；
    /// 普通用户空间缓冲区读写（`uaccess`）的缺页错误不走此规则，而是由 [`copyin`]/[`copyout`] 直接返回 [`Self::BadAddress`]。
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
    /// 将 `futex` 子系统的 [`FutexError`] 映射为系统调用 [`Errno`]。
    ///
    /// 区分原子比较不相等（`EAGAIN`）、对齐或参数非法（`EINVAL`）、内核缺内存（`ENOMEM`）以及文件后备 I/O 错误，
    /// 而将其余用户虚拟地址未映射或只读/权限错误统一编码为 `EFAULT`（[`Self::BadAddress`]）。
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
    /// 将从用户空间读取以 NUL 结尾 C 字符串时的 [`CStringError`] 映射为 [`Errno`]。
    ///
    /// 用户指针缺页不可读返回 `EFAULT`，内核分配缓冲失败返回 `ENOMEM`，在最大长度内未找到终止符 `\0` 返回 `ENAMETOOLONG`。
    fn from(error: CStringError) -> Self {
        match error {
            CStringError::Vm(_) => Self::BadAddress,
            CStringError::NoMemory => Self::NoMemory,
            CStringError::Unterminated => Self::NameTooLong,
        }
    }
}

impl From<ProcessError> for Errno {
    /// 将进程/线程关系与生命周期操作返回的 [`ProcessError`] 映射为 [`Errno`]。
    ///
    /// 区分内核内存耗尽（`ENOMEM`）与 PID 槽位耗尽/临时繁忙（`EAGAIN`），并将权限与目标不存在错误分别对应到 `EPERM`、`EACCES`、`ESRCH` 与 `ECHILD`。
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

/// 将内核切片 `bytes` 的全部字节完整拷贝到用户虚拟地址 `address` 处。
///
/// 在普通可睡眠的用户线程上下文中调用（严禁持自旋锁调用）。
/// 仅当全部 `bytes.len()` 字节均无错写入时返回 `Ok(())`；若中途遇到未映射页或不可写页，已写入的前缀字节保留在用户内存中，函数返回 `Err(Errno::BadAddress)`（`-EFAULT`）。
pub(super) fn copyout(mm: &Mm, address: usize, bytes: &[u8]) -> Result<(), Errno> {
    let progress = mm.copyout(address, bytes);
    if progress.copied == bytes.len() && progress.error.is_none() {
        Ok(())
    } else {
        Err(Errno::BadAddress)
    }
}

/// 从用户虚拟地址 `address` 处将定长数据完整读取到内核切片 `bytes` 中。
///
/// 在普通可睡眠的用户线程上下文中调用（严禁持自旋锁调用）。
/// 仅当全部 `bytes.len()` 字节均成功读入后才返回 `Ok(())`；一旦发生部分拷贝失败立即返回 `Err(Errno::BadAddress)`（`-EFAULT`），防止调用方解析只填充了一半的结构体。
pub(super) fn copyin(mm: &Mm, bytes: &mut [u8], address: usize) -> Result<(), Errno> {
    let progress = mm.copyin(bytes, address);
    if progress.copied == bytes.len() && progress.error.is_none() {
        Ok(())
    } else {
        Err(Errno::BadAddress)
    }
}
