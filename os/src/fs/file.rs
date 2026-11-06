//! 打开文件描述（Open File Description, OFD）抽象、用户态/内核态 I/O 缓冲以及普通文件与目录实现。
//!
//! # 模块职责与架构定位
//! 在类 Unix 文件系统中，“进程文件描述符（`fd`）”与“打开文件描述（OFD）”分为两层：每次调用 `open`/`openat` 会新建一个独立的 OFD（拥有独立的读写游标与状态标志）；而 `dup`/`fork` 复制描述符时，多个 `fd` 共享同一个底层 OFD（`Arc<dyn FileLike>`）。
//! - **上游调用者**：系统调用层（`read`/`write`/`pread`/`pwrite`/`readv`/`writev`、`lseek`、`fstat`、`fsync`、`getdents64`、`fcntl` 等）、文件描述符表（`FileTable`）、跨文件传输（`transfer`）与多路复用（`poll`/`epoll`）。
//! - **下游依赖**：路径与挂载视图（`Path`、`Vfs`）、索引节点与写准入令牌（`InodeRef`、`WriteAccess`）、页缓存预读与脏页回写（`readahead`、`writeback`、`background`）、进程虚拟地址空间拷贝（`Mm::copyin`/`copyout`）以及字符设备/伪文件系统后端（`devfs`、`tty`、`procfs`、`kmsg`、`ktrace`）。
//!
//! # 核心设计与分层关系
//! ```text
//!   FileTable (进程描述符表)
//!     ├── fd 3 ─┐
//!     └── fd 4 ─┴─> Arc<dyn FileLike> (FileRef: 共享同一个打开文件实例)
//!                      ├── InodeFile     : 普通文件 OFD (Path + cursor 睡眠锁 + 预读状态 + 回写错误采样 + WriteAccess)
//!                      ├── DirectoryFile : 目录 OFD     (Path + DirectoryCursor 睡眠锁 + 元数据回写错误采样)
//!                      └── 其它实现      : PipeEnd / SocketFile / EpollFile / TTY / procfs / devfs ...
//!
//!   统一安全 I/O 缓冲适配器（解耦页缓存锁与用户态缺页异常）：
//!   - ReadBuffer  : 页缓存/内核片段 ──push()──> Kernel(&mut [u8]) | User(Mm::copyout) | Vector(iovec)
//!   - WriteBuffer : Kernel(&[u8]) | User(Mm::copyin) | Vector(iovec) ──pull()──> 内核中转切片 ──> 页缓存提交
//! ```
//! - **用户缺页与文件锁解耦**：向用户虚拟地址复制数据（`copyin`/`copyout`）可能触发缺页异常（甚至是由同一文件 `mmap` 引起的缺页）。因此 `ReadBuffer::push` 和 `WriteBuffer::pull` 严禁在持有页缓存 Folio 内容锁、块映射锁或页表 PTE 锁时调用。
//! - **回写错误采样推进（errseq 机制）**：磁盘上的 `InodeFile` 与 `DirectoryFile` 在打开时采样当前数据/元数据缓存的回写错误序号，调用 `fsync`/`sync_range` 时对比并推进游标，确保后台异步刷盘失败能被准确报告给执行同步的描述符。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **表锁在最外层并先行释放**：调用任何 `FileLike` 方法前，必须已释放 `FileTable` 自旋锁，仅持有克隆出的 `FileRef`。
//! 2. **OFD 位置睡眠锁（`cursor: SleepMutex`）**：顺序读写（`read_into`/`write_from`）、`seek` 与目录枚举（`readdir`/`for_each_dirent`）先获取 `cursor` 睡眠锁，再进入底层 `Inode` 读写路径；定位读写（`pread`/`pwrite`）不获取 `cursor` 锁。
//! 3. **叶级自旋锁与原子标志**：`InodeFile::readahead`（预读状态自旋锁）与 `wb_errors` / `wb_error`（回写错误游标自旋锁）仅做纯内存读写，持锁期间严禁发起磁盘 I/O 或睡眠；`flags` 使用 `AtomicU32` 无锁读写。
//! 4. **同步落盘与脏页节流位于 `cursor` 锁之外**：`write_from` 更新完共享游标后立即释放 `cursor` 锁，再执行 `O_DSYNC`/`O_SYNC` 刷盘和 `balance_dirty_pages`，防止慢速磁盘回写长时间阻塞并发 `lseek` 或属性查询。

use alloc::sync::Arc;
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::arch;
use crate::block::{AccessMode, BlockError};
use crate::fs::epoll::{EpollFile, WatchHead};
use crate::fs::memfs::MemorySpecial;
use crate::fs::pipe::PipeEnd;
#[cfg(feature = "ktest")]
use crate::fs::readahead::ReadaheadSnapshot;
use crate::fs::readahead::ReadaheadState;
use crate::fs::{
    background, writeback, DirectoryCursor, DirectoryEntry, FsError, InodeKind, InodeRef,
    InodeTime, Path, Pollable, Vfs, WriteAccess, S_IALLUGO, S_IFBLK, S_IFCHR, S_IFDIR, S_IFIFO,
    S_IFLNK, S_IFREG, S_IFSOCK,
};
use crate::fs::{devfs, kmsg, ktrace, procfs};
use crate::mm::space::{Mm, VmError};
use crate::net::socket::SocketFile;
use crate::sync::{SleepMutex, SleepMutexGuard, SpinLock};
use crate::task::UserThread;

/// 打开文件描述（OFD）的引用计数共享句柄；当最后一个强引用（`Arc`）被释放时，自动归还其持有的挂载路径（`Path`）与写准入计数（`WriteAccess`）。
pub type FileRef = Arc<dyn FileLike>;

/// 单次文件 I/O 或控制操作发起方的执行上下文。
///
/// 用于区分内核内部调用与具体用户线程调用（例如 TTY 控制终端绑定、`procfs` 自身视图查询、可中断信号检测等场景需要读取调用线程身份，不能仅凭缓冲区类型推断）。
#[derive(Clone, Copy)]
pub enum FileIoContext<'a> {
    /// 由内核子系统（如 ELF 加载器、内核线程或 `ktest`）直接发起的调用，无关联用户态信号与终端会话。
    Kernel,
    /// 由具体用户态线程通过系统调用发起的操作，携带该线程引用以供信号中断与凭据/会话检查。
    User(&'a UserThread),
}

/// 文件访问模式位掩码（提取 `O_RDONLY` / `O_WRONLY` / `O_RDWR` 低 2 位）。
pub(crate) const O_ACCMODE: u32 = 0o3;
/// 只读打开模式（`0`）。
pub(crate) const O_RDONLY: u32 = 0o0;
/// 只写打开模式（`1`）。
pub(crate) const O_WRONLY: u32 = 0o1;
/// 读写打开模式（`2`）。
pub(crate) const O_RDWR: u32 = 0o2;

/// 若目标文件不存在则创建普通文件（`O_CREAT`）。
pub(crate) const O_CREAT: u32 = 0o100;
/// 与 `O_CREAT` 联用，要求目标文件必须新建，若已存在则报错返回（`O_EXCL`）。
pub(crate) const O_EXCL: u32 = 0o200;
/// 打开已存在的普通文件时将其长度截断为 0（`O_TRUNC`）。
pub(crate) const O_TRUNC: u32 = 0o1000;
/// 追加写模式：每次写入前原子地将写入起点定位到文件当前末尾（`O_APPEND`）。
pub(crate) const O_APPEND: u32 = 0o2000;
/// 非阻塞 I/O 模式：操作无法立即完成时返回 `WouldBlock` 而非睡眠等待（`O_NONBLOCK`）。
pub(crate) const O_NONBLOCK: u32 = 0o4000;
/// 数据同步写标志：每次写入完成后同步刷回对应文件数据及寻址必需的元数据（`O_DSYNC`）。
pub(crate) const O_DSYNC: u32 = 0o10000;
/// Linux ABI 底层的完整文件同步附加位（与 `O_DSYNC` 组合成 `O_SYNC`）。
const __O_SYNC: u32 = 0o4000000;
/// 完整同步写标志：每次写入完成后同步刷回文件数据与全部相关元数据（`O_SYNC`）。
pub(crate) const O_SYNC: u32 = __O_SYNC | O_DSYNC;
/// 允许打开超过 2 GiB 的大文件标志（64 位架构兼容保留位，`O_LARGEFILE`）。
pub(crate) const O_LARGEFILE: u32 = 0o100000;
/// 要求打开目标必须是目录，否则返回 `NotDirectory`（`O_DIRECTORY`）。
pub(crate) const O_DIRECTORY: u32 = 0o200000;
/// 若路径末端组件为符号链接则拒绝跟随并返回错误（`O_NOFOLLOW`）。
pub(crate) const O_NOFOLLOW: u32 = 0o400000;
/// 为新分配的文件描述符设置 `FD_CLOEXEC` 属性（仅保存在描述符表槽位上，`O_CLOEXEC`）。
pub(crate) const O_CLOEXEC: u32 = 0o2000000;
/// 打开终端设备时不将其设为调用进程的控制终端（`O_NOCTTY`）。
pub(crate) const O_NOCTTY: u32 = 0o400;

/// 允许通过 `fcntl(F_SETFL)` 在打开后动态修改的 OFD 状态标志集合。
const MUTABLE_FLAGS: u32 = O_APPEND | O_NONBLOCK;
/// 允许保存在打开文件描述（OFD）上的状态与打开约束标志掩码（不含访问模式、`O_CREAT`/`O_EXCL`/`O_TRUNC` 和 `O_CLOEXEC`）。
const OPEN_FLAGS: u32 = MUTABLE_FLAGS | O_SYNC | O_LARGEFILE | O_DIRECTORY | O_NOFOLLOW | O_NOCTTY;

/// 从用户态 `struct iovec` 数组解码到内核栈/堆快照中的单段缓冲区描述（用于 `readv` / `writev` 等分散-聚集 I/O）。
///
/// 仅保存用户态虚拟地址与字节长度数值，不在内核中构造指向用户内存的 Rust 引用。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct UserIoVector {
    /// 用户空间缓冲区的起始虚拟地址（字节地址）。
    pub address: usize,
    /// 该段缓冲区的字节长度（允许为 0，零长度段在拷贝时直接跳过）。
    pub length: usize,
}

/// 分散-聚集（Scatter/Gather）用户态 `iovec` 数组的逐段推进游标。
struct UserVector<'a> {
    /// 目标用户进程的地址空间句柄，用于执行安全的跨地址空间逐页复制。
    mm: &'a Mm,
    /// 已拷入内核内存的 `UserIoVector` 切片快照。
    vectors: &'a [UserIoVector],
    /// 当前正在处理的段在 `vectors` 中的下标。
    index: usize,
    /// 当前段内已消费/已填充的字节偏移。
    offset: usize,
    /// 所有段累加后的总请求字节数（构造时已校验无算术溢出）。
    length: usize,
}

impl<'a> UserVector<'a> {
    /// 校验每一段非空用户地址区间均落在合法用户地址空间内，并无溢出地累加总请求字节数。
    fn new(mm: &'a Mm, vectors: &'a [UserIoVector]) -> Result<Self, FsError> {
        let mut length = 0usize;
        for vector in vectors {
            if vector.length != 0 {
                let end = vector
                    .address
                    .checked_add(vector.length)
                    .ok_or(FsError::BadAddress)?;
                if arch::USER_ADDRESS_END < end {
                    return Err(FsError::BadAddress);
                }
            }
            length = length
                .checked_add(vector.length)
                .ok_or(FsError::InvalidOperation)?;
        }

        Ok(Self {
            mm,
            vectors,
            index: 0,
            offset: 0,
            length,
        })
    }

    /// 跳过已耗尽的段和长度为 0 的空段，返回下一个待处理段的起始用户虚拟地址与剩余字节长度。
    fn next(&mut self) -> Option<(usize, usize)> {
        while let Some(vector) = self.vectors.get(self.index) {
            if self.offset < vector.length {
                return Some((vector.address + self.offset, vector.length - self.offset));
            }
            self.index += 1;
            self.offset = 0;
        }
        None
    }
}

/// 文件读取操作的统一接收端缓冲区，屏蔽内核切片、单段用户虚拟地址与多段 `iovec` 的拷贝差异。
pub struct ReadBuffer<'a> {
    /// 底层具体接收目标（内核切片、单段用户地址或用户 `iovec` 游标）。
    target: ReadTarget<'a>,
    /// 本次读取操作迄今已实际成功写入接收端的累计字节数。
    written: usize,
}

/// `ReadBuffer` 的内部接收目标变体。
enum ReadTarget<'a> {
    /// 直接写入内核态独占可变字节切片。
    Kernel(&'a mut [u8]),
    /// 通过绑定地址空间 `Mm::copyout` 写入单段连续用户虚拟地址范围。
    User {
        /// 接收方用户进程的地址空间。
        mm: &'a Mm,
        /// 用户缓冲区起始虚拟地址。
        address: usize,
        /// 用户缓冲区总字节长度。
        length: usize,
    },
    /// 通过 `UserVector` 游标依次写入多段用户 `iovec` 缓冲区。
    Vector(UserVector<'a>),
}

impl<'a> ReadBuffer<'a> {
    /// 将内核态可变字节切片封装为读取接收端。
    pub fn kernel(bytes: &'a mut [u8]) -> Self {
        Self {
            target: ReadTarget::Kernel(bytes),
            written: 0,
        }
    }

    /// 校验用户态虚拟地址区间 `[address, address + length)` 未溢出且位于用户地址空间内，构造单段用户态读取接收端。
    ///
    /// 当 `length == 0` 时不检查 `address` 可访问性，直接构造零容量接收端。
    pub fn user(mm: &'a Mm, address: usize, length: usize) -> Result<Self, FsError> {
        if length != 0 {
            let end = address.checked_add(length).ok_or(FsError::BadAddress)?;
            if arch::USER_ADDRESS_END < end {
                return Err(FsError::BadAddress);
            }
        }

        Ok(Self {
            target: ReadTarget::User {
                mm,
                address,
                length,
            },
            written: 0,
        })
    }

    /// 校验内核已快照的 `iovec` 数组并构造多段用户态分散读取接收端（用于 `readv`/`preadv`）。
    pub fn user_vector(mm: &'a Mm, vectors: &'a [UserIoVector]) -> Result<Self, FsError> {
        Ok(Self {
            target: ReadTarget::Vector(UserVector::new(mm, vectors)?),
            written: 0,
        })
    }

    /// 返回接收端缓冲区的总字节容量（不随写入推进而减少）。
    pub fn len(&self) -> usize {
        match &self.target {
            ReadTarget::Kernel(bytes) => bytes.len(),
            ReadTarget::User { length, .. } => *length,
            ReadTarget::Vector(vector) => vector.length,
        }
    }

    /// 返回目前已成功写入接收端的字节数（即使中途发生用户态缺页错误，此前已拷出的字节仍会计入）。
    pub fn copied(&self) -> usize {
        self.written
    }

    /// 将内核暂存切片 `bytes` 中的数据追加写入接收端（自动按剩余容量裁剪），返回本次写入的字节数。
    ///
    /// # 锁契约
    /// 向用户空间执行 `copyout` 可能触发用户页缺页异常并睡眠，因此调用本方法时**严禁持有源文件页缓存（Folio）内容锁或自旋锁**。
    pub fn push(&mut self, bytes: &[u8]) -> Result<usize, FsError> {
        let count = bytes.len().min(self.len() - self.written);
        let bytes = &bytes[..count];
        match &mut self.target {
            ReadTarget::Kernel(dst) => {
                dst[self.written..self.written + count].copy_from_slice(bytes);
                self.written += count;
                Ok(count)
            }
            ReadTarget::User { mm, address, .. } => {
                let progress = mm.copyout(*address + self.written, bytes);
                self.written += progress.copied;
                match progress.error {
                    None => Ok(progress.copied),
                    Some(_) => Err(FsError::BadAddress),
                }
            }
            ReadTarget::Vector(vector) => {
                let mut copied = 0;
                while copied < count {
                    let (address, remaining) =
                        vector.next().expect("read vector ended before length");
                    let end = count.min(copied + remaining);
                    let progress = vector.mm.copyout(address, &bytes[copied..end]);
                    vector.offset += progress.copied;
                    copied += progress.copied;
                    self.written += progress.copied;
                    if progress.error.is_some() {
                        return Err(FsError::BadAddress);
                    }
                }
                Ok(copied)
            }
        }
    }
}

/// 文件写入操作的统一输入源缓冲区，负责在进入页缓存临界区之前将用户或内核输入分批拉取到内核临时切片。
///
/// 注意：`prepared` 仅记录从输入源拷入临时缓冲区的字节数，文件最终实际写入并提交的字节数由底层 `Inode` 写循环单独统计返回。
pub struct WriteBuffer<'a> {
    /// 底层具体输入源（内核只读切片、单段用户地址或用户 `iovec` 游标）。
    source: WriteSource<'a>,
    /// 迄今已从输入源拉取到内核临时切片的累计字节数。
    prepared: usize,
    /// 上一次 `copyin` 在拉取部分有效字节后遇到的延迟错误；待已拉取字节优先提交后，在下一次 `pull` 时上报。
    error: Option<FsError>,
}

/// `WriteBuffer` 的内部输入源变体。
enum WriteSource<'a> {
    /// 直接从内核态只读字节切片读取。
    Kernel(&'a [u8]),
    /// 通过绑定地址空间 `Mm::copyin` 从单段连续用户虚拟地址范围拉取。
    User {
        /// 发送方用户进程的地址空间。
        mm: &'a Mm,
        /// 用户缓冲区起始虚拟地址。
        address: usize,
        /// 用户缓冲区总字节长度。
        length: usize,
    },
    /// 通过 `UserVector` 游标依次从多段用户 `iovec` 缓冲区拉取。
    Vector(UserVector<'a>),
}

impl<'a> WriteBuffer<'a> {
    /// 将内核态只读字节切片封装为写入输入源。
    pub fn kernel(bytes: &'a [u8]) -> Self {
        Self {
            source: WriteSource::Kernel(bytes),
            prepared: 0,
            error: None,
        }
    }

    /// 校验用户态虚拟地址区间 `[address, address + length)` 未溢出且位于用户地址空间内，构造单段用户态写入源。
    ///
    /// 当 `length == 0` 时不访问用户地址，直接构造零长度输入源。
    pub fn user(mm: &'a Mm, address: usize, length: usize) -> Result<Self, FsError> {
        if length != 0 {
            let end = address.checked_add(length).ok_or(FsError::BadAddress)?;
            if arch::USER_ADDRESS_END < end {
                return Err(FsError::BadAddress);
            }
        }

        Ok(Self {
            source: WriteSource::User {
                mm,
                address,
                length,
            },
            prepared: 0,
            error: None,
        })
    }

    /// 校验内核已快照的 `iovec` 数组并构造多段用户态聚集写入源（用于 `writev`/`pwritev`）。
    pub fn user_vector(mm: &'a Mm, vectors: &'a [UserIoVector]) -> Result<Self, FsError> {
        Ok(Self {
            source: WriteSource::Vector(UserVector::new(mm, vectors)?),
            prepared: 0,
            error: None,
        })
    }

    /// 返回输入源的总字节长度。
    pub fn len(&self) -> usize {
        match &self.source {
            WriteSource::Kernel(bytes) => bytes.len(),
            WriteSource::User { length, .. } => *length,
            WriteSource::Vector(vector) => vector.length,
        }
    }

    /// 返回目前已从输入源拉取准备的字节数（用于计算剩余待写请求量 `len() - copied()`）。
    pub fn copied(&self) -> usize {
        self.prepared
    }

    /// 从输入源拉取最多 `dst.len()` 字节填入内核中转切片 `dst`，返回实际拉取的字节数。
    ///
    /// # 短拷贝与延迟报错语义
    /// 若 `copyin` 先成功拷贝了若干前缀字节，随后在跨页边界遇到无效用户地址或缺页 I/O 错误：
    /// 本次调用会先暂存该错误并返回已成功拉取的非零字节数，确保调用方能先把这部分合法数据写入文件（实现 POSIX 短写语义）；下一次调用 `pull` 时再返回暂存的错误。
    ///
    /// # 锁契约
    /// 用户空间 `copyin` 可能触发缺页异常（包括同一文件的 `mmap` 缺页），因此调用时**严禁持有目标页缓存 Folio 锁、块映射写锁或页表 PTE 锁**。
    pub fn pull(&mut self, dst: &mut [u8]) -> Result<usize, FsError> {
        if let Some(error) = self.error {
            return Err(error);
        }
        let count = dst.len().min(self.len() - self.prepared);
        if count == 0 {
            return Ok(0);
        }

        let copied = match &mut self.source {
            WriteSource::Kernel(bytes) => {
                dst[..count].copy_from_slice(&bytes[self.prepared..self.prepared + count]);
                count
            }
            WriteSource::User { mm, address, .. } => {
                let progress = mm.copyin(&mut dst[..count], *address + self.prepared);
                self.error = progress.error.map(|error| match error {
                    VmError::NoMemory => FsError::NoMemory,
                    VmError::File(error) => error,
                    _ => FsError::BadAddress,
                });
                progress.copied
            }
            WriteSource::Vector(vector) => {
                let mut copied = 0;
                while copied < count {
                    let (address, remaining) =
                        vector.next().expect("write vector ended before length");
                    let end = count.min(copied + remaining);
                    let progress = vector.mm.copyin(&mut dst[copied..end], address);
                    vector.offset += progress.copied;
                    copied += progress.copied;
                    self.error = progress.error.map(|error| match error {
                        VmError::NoMemory => FsError::NoMemory,
                        VmError::File(error) => error,
                        _ => FsError::BadAddress,
                    });
                    if self.error.is_some() {
                        break;
                    }
                }
                copied
            }
        };

        self.prepared += copied;
        if copied == 0 {
            if let Some(error) = self.error {
                return Err(error);
            }
        }
        Ok(copied)
    }
}

/// 文件读写游标的定位基准（对应 `lseek` 系统调用的 `whence` 与 `offset`）。
///
/// 计算出的最终文件偏移必须可表示为非负有符号 64 位整数（`0 <= pos <= i64::MAX`）；若结果为负或溢出则保持原游标不变并返回 `InvalidOperation`。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SeekFrom {
    /// 从文件起始处起算的绝对字节偏移（对应 `SEEK_SET`）。
    Start(u64),
    /// 相对于当前文件共享游标位置的有符号字节位移（对应 `SEEK_CUR`）。
    Current(i64),
    /// 相对于文件当前末尾大小（或目录末尾 cookie）的有符号字节位移（对应 `SEEK_END`）。
    End(i64),
}

/// 文件或索引节点的元数据属性快照（对应 `stat` / `fstat` / `newfstatat` 系统调用输出的信息）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FileStat {
    /// 文件所在文件系统内的索引节点编号（`st_ino`）。
    pub inode: u64,
    /// 索引节点的高层对象类型（普通文件、目录、符号链接、字符设备等）。
    pub kind: InodeKind,
    /// 包含文件类型位（`S_IFMT`）与访问权限位（`0o7777`）的完整 POSIX 模式字（`st_mode`）。
    pub mode: u16,
    /// 文件逻辑字节大小（`st_size`，单位：字节）。
    pub size: u64,
    /// 所有者用户 ID（`st_uid`）。
    pub uid: u32,
    /// 所有者用户组 ID（`st_gid`）。
    pub gid: u32,
    /// 指向该索引节点的硬链接计数（`st_nlink`）。
    pub links: u32,
    /// 已分配存储空间计数（`st_blocks`，按 POSIX 惯例以 512 字节扇区为单位计数）。
    pub blocks: u64,
    /// 承载该文件的文件系统所属设备号（`st_dev`）。
    pub device: u64,
    /// 特殊文件（字符设备或块设备节点）所代表的设备号（`st_rdev`；普通文件与目录为 `0`）。
    pub rdev: u64,
    /// 文件系统建议的最优 I/O 块大小（`st_blksize`，单位：字节）。
    pub block_size: u32,
    /// 最后访问时间戳（`st_atime`）。
    pub atime: InodeTime,
    /// 最后内容修改时间戳（`st_mtime`）。
    pub mtime: InodeTime,
    /// 最后元数据/状态变更时间戳（`st_ctime`）。
    pub ctime: InodeTime,
}

/// 打开文件描述（OFD）的统一抽象接口。
///
/// 所有方法均运行在普通可睡眠任务上下文中；调用方必须在释放进程描述符表（`FileTable`）自旋锁之后再通过 `FileRef` 调用。
pub trait FileLike: Send + Sync {
    /// 从文件当前共享游标处顺序读取数据至内核缓冲区 `dst`，并按实际读取字节数推进游标。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "ktest 文件表和事件消费者使用内核缓冲核对共享位置与读取内容"
        )
    )]
    fn read(&self, dst: &mut [u8]) -> Result<usize, FsError> {
        self.read_into(&mut ReadBuffer::kernel(dst), FileIoContext::Kernel)
    }

    /// 从显式字节偏移 `offset` 处读取数据至内核缓冲区 `dst`（对应内核态 `pread`），不获取或修改文件的共享游标。
    fn read_at(&self, offset: u64, dst: &mut [u8]) -> Result<usize, FsError> {
        self.read_at_into(offset, &mut ReadBuffer::kernel(dst), FileIoContext::Kernel)
    }

    /// 从文件当前共享游标处读取数据并写入通用接收端 `target`，仅按成功写入接收端的字节数推进共享游标。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;

    /// 从显式字节偏移 `offset` 处读取数据并写入通用接收端 `target`（对应 `pread` / `preadv`），不获取或修改共享游标。
    fn read_at_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;

    /// 获取当前打开文件实例的顺序预读窗口状态快照（仅磁盘普通文件返回 `Some`，供 `ktest` 断言验证）。
    #[cfg(feature = "ktest")]
    fn ktest_readahead_snapshot(&self) -> Option<ReadaheadSnapshot> {
        None
    }

    /// 返回该文件绑定持有的挂载路径（`Path`）；匿名管道、套接字、`epoll` 等无名流对象返回 `None`（不可作为 `*at` 系列调用的 `dirfd` 起点）。
    fn path(&self) -> Option<&Path>;

    /// 若当前对象为支持 `mmap` 内存映射的普通文件，返回其底层 `InodeRef`；目录与流式设备返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef>;

    /// 当前文件是否支持显式字节偏移概念（决定是否允许执行 `pread` / `pwrite`；管道、套接字等流对象返回 `false` 并报 `ESPIPE`）。
    fn positioned_io(&self) -> bool {
        false
    }

    /// 若当前对象具有持续就绪通知能力（如管道、套接字、TTY、`epoll`），返回其 `Pollable` 接口；普通文件与目录返回 `None`（轮询时直接视为立即就绪）。
    fn pollable(&self) -> Option<&dyn Pollable> {
        None
    }

    /// 若当前对象为 `epoll` 实例本身，返回 `EpollFile` 引用（供 `epoll_ctl` / `epoll_pwait` 获取内部控制状态）。
    fn epoll(&self) -> Option<&EpollFile> {
        None
    }

    /// 若当前对象为网络或本地套接字端点，返回 `SocketFile` 引用（供 `bind` / `connect` / `sendmsg` 等网络系统调用使用）。
    fn socket(&self) -> Option<&SocketFile> {
        None
    }

    /// 若当前对象为普通文件打开实例，返回 `InodeFile` 引用（供 `copy_file_range` 与 `splice` 执行内核态文件传输）。
    fn inode_file(&self) -> Option<&InodeFile> {
        None
    }

    /// 若当前对象为匿名管道的读端或写端，返回 `PipeEnd` 引用（供 `splice` 访问底层管道环形缓冲）。
    fn pipe_end(&self) -> Option<&PipeEnd> {
        None
    }

    /// 返回可被 `epoll` 持久监听的反向观察链表头（`WatchHead`）；当该 OFD 的最后一个强引用释放时，借此同步清理所有挂载的 `epoll` 监听项。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        None
    }

    /// 对设备或特殊文件执行 `ioctl` 控制命令；不支持设备控制的对象默认返回 `FsError::NotTty`（`ENOTTY`）。
    fn ioctl(
        &self,
        _command: u32,
        _argument: usize,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::NotTty)
    }

    /// 读取当前打开文件描述（OFD）的访问模式与状态标志（`O_CLOEXEC` 属于描述符表槽位属性，不包含在此返回值中）。
    fn flags(&self) -> u32;

    /// 修改当前 OFD 的可变状态标志（如 `O_APPEND`、`O_NONBLOCK`，对应 `fcntl(F_SETFL)`），忽略访问模式与创建类标志。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError>;

    /// 原子开启或关闭当前 OFD 的 `O_NONBLOCK` 非阻塞标志（对应 `ioctl(FIONBIO)`），保留其余全部标志位不变。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError>;

    /// 将可写普通文件的逻辑大小截断或扩展为 `new_size` 字节（对应 `ftruncate`），不改变当前共享读写游标；可能睡眠等待磁盘 I/O。
    fn truncate(&self, new_size: u64) -> Result<(), FsError>;

    /// 将半开字节区间 `range` 内的脏数据页、必要元数据及块设备写缓存同步刷盘，并检查与推进当前 OFD 的回写错误采样游标（对应 `fsync` / `fdatasync`）。
    ///
    /// # 锁契约
    /// 本方法可能阻塞等待块设备 I/O，调用方**严禁持有描述符表锁、文件位置锁、`Inode` 写锁或虚拟内存映射/PTE 锁**。
    fn sync_range(&self, range: Range<u64>, data_only: bool) -> Result<(), FsError>;

    /// 按 `from` 调整当前打开文件的共享字节偏移（或目录 cookie）并返回新偏移（对应 `lseek`）；若目标位置为负或溢出则不修改原位置。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError>;

    /// 读取底层 `Inode` 当前的元数据属性快照（对应 `fstat`），无需获取文件位置锁。
    fn stat(&self) -> Result<FileStat, FsError>;

    /// 读取目录中的下一条目录项并推进共享目录游标（非目录对象返回 `NotDirectory`）。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "ktest 文件表逐项核对目录名字、类型与共享 cookie")
    )]
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError>;

    /// 循环枚举目录项并交给回调 `emit` 填充用户缓冲区（对应 `getdents64`）：仅当 `emit` 返回 `Ok(true)` 时才推进目录游标；返回 `Ok(false)`（如用户缓冲区已满）或报错时保留当前项游标以便下次续读。
    fn for_each_dirent(
        &self,
        emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError>;

    /// 将内核切片 `src` 顺序写入文件当前共享游标处（或追加至末尾），并更新共享游标。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(dead_code, reason = "ktest 普通内核写者验证顺序位置提交与回写背压")
    )]
    fn write(&self, src: &[u8]) -> Result<usize, FsError> {
        self.write_from(&mut WriteBuffer::kernel(src), FileIoContext::Kernel)
    }

    /// 将内核切片 `src` 写入显式字节偏移 `offset` 处（对应内核态 `pwrite`），不获取或修改共享游标。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "ktest 按位置写者验证共享页、截断和原生名字修改后的内容"
        )
    )]
    fn write_at(&self, offset: u64, src: &[u8]) -> Result<usize, FsError> {
        self.write_at_from(offset, &mut WriteBuffer::kernel(src), FileIoContext::Kernel)
    }

    /// 从通用输入源 `source` 向文件当前共享游标处（`O_APPEND` 时为文件末尾）执行写入，更新游标并在锁外完成同步与脏页节流。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;

    /// 从通用输入源 `source` 向显式字节偏移 `offset` 处执行定位写入（对应 `pwrite` / `pwritev`；若设置 `O_APPEND` 则仍由底层 `Inode` 追加至文件末尾），不修改共享游标。
    fn write_at_from(
        &self,
        offset: u64,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;
}

/// 普通文件的打开文件描述（OFD）实例。
///
/// 持有挂载路径 `Path` 以在打开期间保活所属文件系统挂载与底层 `Inode`；多个复制出的文件描述符共享此结构体中的读写偏移、预读窗口与回写错误游标。
pub struct InodeFile {
    /// 打开时绑定的已解析挂载路径，保活挂载点与底层普通文件 `Inode`。
    path: Path,
    /// 顺序读写与 `lseek` 共用的当前字节偏移游标（单位：字节），由睡眠互斥锁串行化。
    cursor: SleepMutex<u64>,
    /// 磁盘普通文件的顺序预读窗口跟踪器，由短临界区自旋锁保护（内存文件不使用）。
    readahead: SpinLock<ReadaheadState>,
    /// 当前 OFD 的访问模式与状态标志字（`O_ACCMODE`、`O_APPEND`、`O_NONBLOCK`、`O_SYNC` 等），支持无锁原子读写。
    flags: AtomicU32,
    /// 可写打开（`O_WRONLY` / `O_RDWR`）时持有的底层 `Inode` 写者准入凭证（与 `execve` 写互斥配合；只读打开时为 `None`）。
    write_access: Option<WriteAccess>,
    /// 磁盘文件的数据页缓存与卷元数据缓存的回写错误采样对 `(data_errseq, meta_errseq)`；内存文件系统文件为 `None`。
    wb_errors: SpinLock<Option<(u32, u32)>>,
}

/// 目录的打开文件描述（OFD）实例。
///
/// 维护底层文件系统的目录遍历 cookie 以及哈希目录（如 ext4 HTree）同哈希桶内的精确续读偏移。
pub struct DirectoryFile {
    /// 打开时绑定的目录挂载路径，同时支持作为 `*at` 系统调用的相对路径解析起点（`dirfd`）。
    path: Path,
    /// 目录项枚举游标（含文件系统 cookie 与同哈希冲突链偏移），由睡眠互斥锁保护。
    cursor: SleepMutex<DirectoryCursor>,
    /// 当前目录 OFD 的打开状态标志字。
    flags: AtomicU32,
    /// 磁盘目录所在卷元数据缓存的回写错误采样游标（供目录 `fsync` 检测异步元数据回写错误；内存目录为 `None`）。
    wb_error: SpinLock<Option<u32>>,
}

/// 以内核默认只读标志（`O_RDONLY`）打开已解析的路径 `path`，返回共享文件句柄。
pub fn open_path(vfs: &Vfs, path: Path) -> Result<FileRef, FsError> {
    open_path_with_flags(vfs, path, 0, FileIoContext::Kernel)
}

/// 根据已解析的路径 `path` 和打开标志 `flags` 构造对应的打开文件实例（`FileRef`）。
///
/// # 调用上下文与阶段说明
/// - 在普通可睡眠任务上下文、且未持有描述符表锁时调用（路径创建逻辑已在 `Vfs` 层先行完成）。
/// - 若节点为设备或伪文件系统特殊节点（`devfs` / `tty` / `procfs` / `kmsg` / `ktrace`），直接委派给对应后端构造专用 `FileLike`。
/// - 若带有 `O_TRUNC` 打开普通文件，会在 `InodeFile` 分配就绪并获取写准入凭证后将文件截断为 0；若为“只读 + `O_TRUNC`”组合，临时获取的 `WriteAccess` 仅用于本次截断，不会随只读 `InodeFile` 长期保留。
pub fn open_path_with_flags(
    vfs: &Vfs,
    path: Path,
    flags: u32,
    caller: FileIoContext<'_>,
) -> Result<FileRef, FsError> {
    let access = flags & O_ACCMODE;
    if access == O_ACCMODE {
        return Err(FsError::InvalidOperation);
    }
    let flags = flags & !O_CLOEXEC;
    if flags & !(OPEN_FLAGS | O_ACCMODE | O_TRUNC) != 0 {
        return Err(FsError::Unsupported);
    }

    // Step 1: 校验访问模式与标志位合法性，并将完整同步标志 __O_SYNC 归一化为同时置位 O_DSYNC，简化后续写后同步判断。
    let flags = if flags & (O_SYNC & !O_DSYNC) != 0 {
        flags | O_DSYNC
    } else {
        flags
    };

    // Step 2: 获取目标 Inode 类型，校验 O_DIRECTORY 约束，并将内存特殊节点（字符设备、TTY、procfs 等）分派给对应子系统打开。
    let inode = path.inode()?;
    let kind = inode.metadata_snapshot()?.kind;
    if flags & O_DIRECTORY != 0 && kind != InodeKind::Directory {
        return Err(FsError::NotDirectory);
    }

    if let Some(special) = inode.memory_special() {
        return match special {
            MemorySpecial::Null | MemorySpecial::Zero | MemorySpecial::TestHook => {
                devfs::open(path, flags, caller, special)
            }
            MemorySpecial::TtySerial | MemorySpecial::Console | MemorySpecial::ControllingTty => {
                crate::fs::tty::open(path, flags, caller, special)
            }
            MemorySpecial::Kmsg => kmsg::open(path, flags),
            MemorySpecial::Ktrace => ktrace::open(path, flags),
            MemorySpecial::Cmdline
            | MemorySpecial::Mounts
            | MemorySpecial::Meminfo
            | MemorySpecial::Uptime
            | MemorySpecial::NetDev
            | MemorySpecial::NetRoute
            | MemorySpecial::Interrupts => procfs::open(path, flags, special, vfs, caller),
        };
    }

    // Step 3: 构造常规普通文件或目录的 OFD，采样初始回写错误序号，并在返回前按需执行 O_TRUNC 截断。
    match kind {
        InodeKind::Regular => {
            if access != 0 || flags & O_TRUNC != 0 {
                path.check_writable()?;
            }
            let mut write_access = if access != 0 || flags & O_TRUNC != 0 {
                Some(inode.acquire_write()?)
            } else {
                None
            };
            let wb_errors = if inode.is_memory() {
                None
            } else {
                let volume = inode.disk_volume()?;
                Some((
                    inode.mapping().sample_wb_error(),
                    volume.metadata_cache().sample_wb_error(),
                ))
            };
            let file = Arc::try_new(InodeFile {
                path,
                cursor: SleepMutex::new(0),
                readahead: SpinLock::new(ReadaheadState::new()),
                flags: AtomicU32::new(flags & !O_TRUNC),
                write_access: if access != 0 {
                    write_access.take()
                } else {
                    None
                },
                wb_errors: SpinLock::new(wb_errors),
            })
            .map_err(|_| FsError::NoMemory)?;

            // OFD 结构体与内存分配已就绪后再执行截断，避免截断生效后却因 OOM 导致打开失败。
            if flags & O_TRUNC != 0 {
                let access = file
                    .write_access
                    .as_ref()
                    .or(write_access.as_ref())
                    .expect("truncating open has no write access");
                access.truncate(0)?;
            }
            Ok(file)
        }
        InodeKind::Directory => {
            if access != 0 || flags & O_TRUNC != 0 {
                path.check_writable()?;
                return Err(FsError::IsDirectory);
            }
            let cursor = inode.directory_cursor(0)?;
            let wb_error = if inode.is_memory() {
                None
            } else {
                Some(inode.disk_volume()?.metadata_cache().sample_wb_error())
            };
            Ok(Arc::try_new(DirectoryFile {
                path,
                cursor: SleepMutex::new(cursor),
                flags: AtomicU32::new(flags),
                wb_error: SpinLock::new(wb_error),
            })
            .map_err(|_| FsError::NoMemory)?)
        }
        InodeKind::Symlink => Err(FsError::TooManySymlinks),
        InodeKind::Character
        | InodeKind::Block
        | InodeKind::Fifo
        | InodeKind::Socket
        | InodeKind::Other => Err(FsError::Unsupported),
    }
}

impl FileLike for InodeFile {
    /// 返回当前普通文件实例引用，供内核态文件间传输接口直接调用。
    fn inode_file(&self) -> Option<&InodeFile> {
        Some(self)
    }

    /// 普通文件支持显式字节偏移读写（`pread` / `pwrite`）。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 持 `cursor` 睡眠锁从当前偏移读取数据至 `target`，并按实际写入接收端的字节数推进共享偏移。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        let mut cursor = self.cursor.lock();
        let inode = self.path.inode()?;
        let readahead = if inode.is_memory() {
            None
        } else {
            Some(&self.readahead)
        };
        let bytes = inode.read_into(*cursor, target, readahead)?;
        // 底层 Inode 保证实际读取字节数不超过文件剩余可读长度，遇到 EOF 或零长请求时返回 0。
        *cursor += bytes as u64;
        Ok(bytes)
    }

    /// 从显式偏移 `offset` 处读取数据至 `target`（不获取共享 `cursor` 锁，但仍更新当前 OFD 的顺序预读提示）。
    fn read_at_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        let inode = self.path.inode()?;
        let readahead = if inode.is_memory() {
            None
        } else {
            Some(&self.readahead)
        };
        inode.read_into(offset, target, readahead)
    }

    /// 获取磁盘普通文件 OFD 的预读状态快照；内存文件系统中的文件无磁盘预读状态，返回 `None`。
    #[cfg(feature = "ktest")]
    fn ktest_readahead_snapshot(&self) -> Option<ReadaheadSnapshot> {
        if self
            .path
            .inode()
            .expect("regular OFD lost its inode")
            .is_memory()
        {
            None
        } else {
            Some(self.readahead.lock().snapshot())
        }
    }

    /// 返回打开时绑定的挂载路径引用（可用于保活挂载及作为 `dirfd` 解析相对路径）。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 返回当前普通文件对应的 `InodeRef`（供 `mmap` 建立虚拟内存文件映射）。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        Some(self.path.inode().expect("regular OFD lost its inode"))
    }

    /// 原子读取当前 OFD 的访问模式与状态标志字（无需获取位置锁）。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新当前 OFD 的可变状态标志（如 `O_APPEND`、`O_NONBLOCK`），共享此 OFD 的所有描述符均观察到更新。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子切换当前 OFD 的 `O_NONBLOCK` 标志位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 利用打开时已获取的 `WriteAccess` 调整文件字节大小（对应 `ftruncate`，无需持有 `cursor` 锁，也不改变当前偏移）。
    fn truncate(&self, new_size: u64) -> Result<(), FsError> {
        if self.flags() & O_ACCMODE == 0 {
            return Err(FsError::InvalidOperation);
        }
        self.path.check_writable()?;
        let access = self
            .write_access
            .as_ref()
            .expect("writable OFD has no write access");
        access.truncate(new_size)
    }

    /// 将指定字节区间 `range` 的脏数据页与卷元数据同步刷盘，并对比推进当前 OFD 的回写错误采样游标。
    fn sync_range(&self, range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        if range.end < range.start {
            return Err(FsError::InvalidOperation);
        }

        let inode = self.path.inode()?;
        if inode.is_memory() {
            return Ok(());
        }
        let volume = inode.disk_volume()?;
        // Step 1: 若底层磁盘卷以读写模式挂载，即使当前描述符以只读方式打开，也必须将该文件范围内的脏数据页刷回磁盘。
        let writable = volume.mode() == AccessMode::ReadWrite;
        let data = if writable {
            writeback::sync_inode_range(inode, range)
        } else {
            Ok(())
        };

        // Step 2: 刷回卷级共享元数据缓存并下发块设备 FLUSH 命令（保证新分配的数据块在崩溃恢复后可寻址）。
        // 若前一步因块设备停机未确认（StopUnconfirmed）而中止，不能继续下发新的元数据写请求或 FLUSH。
        let metadata = if writable && !matches!(data, Err(FsError::Io(BlockError::StopUnconfirmed)))
        {
            writeback::sync_metadata(volume)
        } else {
            Ok(())
        };

        // Step 3: 持短自旋锁检查并推进数据与元数据的 errseq 采样游标，汇总直接 I/O 错误与历史异步回写错误。
        let mut samples = self.wb_errors.lock();
        let samples = samples.as_mut().expect("disk OFD has no writeback samples");
        let data_error = inode.mapping().check_and_advance_wb_error(&mut samples.0);
        let metadata_error = volume
            .metadata_cache()
            .check_and_advance_wb_error(&mut samples.1);
        data.and(data_error).and(metadata).and(metadata_error)
    }

    /// 持 `cursor` 睡眠锁调整当前字节偏移（允许将游标定位到超出当前文件末尾的位置以形成稀疏空洞）。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError> {
        seek_path(&self.path, &self.cursor, from)
    }

    /// 读取当前普通文件的元数据属性快照。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 普通文件不支持目录项读取，固定返回 `FsError::NotDirectory`。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 普通文件不支持目录流遍历，固定返回 `FsError::NotDirectory`。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 持 `cursor` 睡眠锁向文件写入数据：先完成页缓存写入、更新共享游标并释放 `cursor` 锁，再在锁外执行同步刷盘与脏页回写节流。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        let flags = self.flags();
        require_write(flags)?;
        self.path.check_writable()?;

        let mut cursor = self.cursor.lock();
        check_write_range(*cursor, source.len() - source.copied())?;
        let inode = self.path.inode()?;
        let access = self
            .write_access
            .as_ref()
            .expect("writable OFD has no write access");
        let (start, bytes) = access.write_from(*cursor, flags & O_APPEND != 0, source)?;
        if 0 < bytes {
            *cursor = start
                .checked_add(bytes as u64)
                .expect("inode write position overflow");
        }
        drop(cursor);

        sync_written(self, flags, start, bytes)?;
        if 0 < bytes && !inode.is_memory() {
            background::balance_dirty_pages(inode);
        }
        Ok(bytes)
    }

    /// 向显式字节偏移 `offset` 处写入数据（不获取或修改共享 `cursor`，`O_APPEND` 由底层 `Inode` 写锁内统一裁定），随后执行同步与脏页节流。
    fn write_at_from(
        &self,
        offset: u64,
        source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        let flags = self.flags();
        require_write(flags)?;
        self.path.check_writable()?;
        check_write_range(offset, source.len() - source.copied())?;

        let inode = self.path.inode()?;
        let access = self
            .write_access
            .as_ref()
            .expect("writable OFD has no write access");
        let (start, bytes) = access.write_from(offset, flags & O_APPEND != 0, source)?;

        sync_written(self, flags, start, bytes)?;
        if 0 < bytes && !inode.is_memory() {
            background::balance_dirty_pages(inode);
        }
        Ok(bytes)
    }
}

impl InodeFile {
    /// 获取当前普通文件 OFD 隐式位置游标的睡眠锁守卫（供 `transfer` 模块在进入管道锁之前按固定锁序加锁）。
    pub(crate) fn transfer_cursor(&self) -> SleepMutexGuard<'_, u64> {
        self.cursor.lock()
    }

    /// 校验当前 OFD 具备可写打开模式且所在挂载点允许写入（供内核态文件传输前置检查）。
    pub(crate) fn transfer_check_write(&self) -> Result<(), FsError> {
        require_write(self.flags())?;
        self.path.check_writable()
    }

    /// 供 `copy_file_range` / `splice` 向指定偏移 `offset` 直接写入内核字节切片，返回实际写入页缓存的字节数。
    ///
    /// 若当前 OFD 带有 `O_APPEND`，按调用方给定的系统调用错误码 `append_error` 拒绝；位置更新与写后同步由调用方在释放外层锁后调用 `transfer_finish` 完成。
    pub(crate) fn transfer_write_at(
        &self,
        offset: u64,
        bytes: &[u8],
        append_error: FsError,
    ) -> Result<usize, FsError> {
        self.transfer_check_write()?;
        if self.flags() & O_APPEND != 0 {
            return Err(append_error);
        }
        check_write_range(offset, bytes.len())?;
        let access = self
            .write_access
            .as_ref()
            .expect("writable OFD has no write access");
        let (start, accepted) =
            access.write_from(offset, false, &mut WriteBuffer::kernel(bytes))?;
        assert_eq!(start, offset);
        Ok(accepted)
    }

    /// 在位置睡眠锁和管道锁释放后，对跨文件传输实际写入的字节区间执行 `O_DSYNC`/`O_SYNC` 同步刷盘与脏页平衡节流。
    pub(crate) fn transfer_finish(
        &self,
        flags: u32,
        start: u64,
        accepted: usize,
    ) -> Result<(), FsError> {
        let result = sync_written(self, flags, start, accepted);
        let inode = self.path.inode()?;
        if 0 < accepted && !inode.is_memory() {
            background::balance_dirty_pages(inode);
        }
        result
    }
}

/// 当本次实际写入字节数非零且设置了 `O_DSYNC`（或包含它的 `O_SYNC`）时，将 `[start, start + bytes)` 范围同步刷回磁盘。
fn sync_written(file: &InodeFile, flags: u32, start: u64, bytes: usize) -> Result<(), FsError> {
    if bytes == 0 || flags & O_DSYNC == 0 {
        return Ok(());
    }
    let end = start
        .checked_add(bytes as u64)
        .expect("accepted write range overflow");
    file.sync_range(start..end, flags & O_SYNC != O_SYNC)
}

/// 校验写入起始偏移 `offset` 与请求长度 `length` 之和未溢出且不超过 `loff_t` 上限（`i64::MAX`）。
fn check_write_range(offset: u64, length: usize) -> Result<(), FsError> {
    let end = offset
        .checked_add(length as u64)
        .ok_or(FsError::InvalidOperation)?;
    if (i64::MAX as u64) < end {
        return Err(FsError::InvalidOperation);
    }
    Ok(())
}

impl FileLike for DirectoryFile {
    /// 目录具有游标位置概念（使得对目录调用 `pread`/`pwrite` 时进入读写方法返回 `EISDIR` 而非流对象的 `ESPIPE`）。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 目录不支持按普通字节流顺序读取，固定返回 `FsError::IsDirectory`（`EISDIR`）。
    fn read_into(
        &self,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IsDirectory)
    }

    /// 目录不支持按指定偏移读取字节，固定返回 `FsError::IsDirectory`（`EISDIR`）。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IsDirectory)
    }

    /// 返回目录绑定的挂载路径（可作为 `*at` 系列系统调用的相对路径解析基准 `dirfd`）。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 目录不支持内存映射（`mmap`），固定返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 原子读取当前目录 OFD 的状态标志字。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新当前目录 OFD 的可变状态标志（不影响目录枚举游标）。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子切换当前目录 OFD 的 `O_NONBLOCK` 标志位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 目录不支持 `ftruncate` 大小截断，固定返回 `FsError::InvalidOperation`。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 对磁盘目录执行同步（`fsync`）：刷回所属卷的脏元数据缓存并下发块设备 FLUSH，同时检查并推进目录的元数据回写错误游标。
    fn sync_range(&self, range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        if range.end < range.start {
            return Err(FsError::InvalidOperation);
        }
        let inode = self.path.inode()?;
        if inode.is_memory() {
            return Ok(());
        }

        let volume = inode.disk_volume()?;
        let result = if volume.mode() == AccessMode::ReadWrite {
            writeback::sync_metadata(volume)
        } else {
            Ok(())
        };

        let mut sample = self.wb_error.lock();
        let sample = sample
            .as_mut()
            .expect("disk directory OFD has no writeback sample");
        let error = volume.metadata_cache().check_and_advance_wb_error(sample);
        result.and(error)
    }

    /// 按目录 cookie 重新定位目录枚举游标（对应目录上的 `lseek`）。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError> {
        seek_directory(&self.path, &self.cursor, from)
    }

    /// 读取目录底层 `Inode` 的元数据属性快照。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 持目录 `cursor` 睡眠锁读取下一条目录项，读取成功后将游标推进至下一项位置。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        let mut cursor = self.cursor.lock();
        let entry = self.path.inode()?.dir_next(&cursor)?;
        if let Some(entry) = &entry {
            commit_dirent(&mut cursor, entry);
        }
        Ok(entry)
    }

    /// 持目录 `cursor` 睡眠锁循环读取目录项并传给 `emit`：仅当 `emit` 成功消费该项并返回 `Ok(true)` 时才推进游标。
    fn for_each_dirent(
        &self,
        emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        let mut cursor = self.cursor.lock();
        let inode = self.path.inode()?;
        while let Some(entry) = inode.dir_next(&cursor)? {
            if !emit(&entry)? {
                break;
            }
            commit_dirent(&mut cursor, &entry);
        }
        Ok(())
    }

    /// 目录不支持字节写入：先校验打开写权限与挂载可写性，随后返回 `FsError::IsDirectory`。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_write(self.flags())?;
        self.path.check_writable()?;
        Err(FsError::IsDirectory)
    }

    /// 目录不支持按偏移字节写入，复用 `write_from` 的权限检查与目录错误返回。
    fn write_at_from(
        &self,
        _offset: u64,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.write_from(source, caller)
    }
}

/// 校验 OFD 打开标志是否允许读取（若为只写模式 `O_WRONLY` 则返回 `FsError::BadFd`）。
pub(crate) fn require_read(flags: u32) -> Result<(), FsError> {
    if flags & O_ACCMODE == O_WRONLY {
        Err(FsError::BadFd)
    } else {
        Ok(())
    }
}

/// 校验 OFD 打开标志是否允许写入（若为只读模式 `O_RDONLY` 则返回 `FsError::BadFd`）。
pub(crate) fn require_write(flags: u32) -> Result<(), FsError> {
    if flags & O_ACCMODE == 0 {
        Err(FsError::BadFd)
    } else {
        Ok(())
    }
}

/// 原子更新 OFD 的可变状态标志（对应 `fcntl(F_SETFL)`）：按 POSIX 规范忽略访问模式、创建截断位与 `O_CLOEXEC`，拒绝不支持的未知位后原子替换 `O_APPEND | O_NONBLOCK`。
pub(crate) fn set_status_flags(flags: &AtomicU32, raw: u32) -> Result<(), FsError> {
    let ignored = O_ACCMODE | O_CREAT | O_EXCL | O_TRUNC | O_CLOEXEC;
    if raw & !(OPEN_FLAGS | ignored) != 0 {
        return Err(FsError::Unsupported);
    }
    let _ = flags.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |old| {
        Some((old & !MUTABLE_FLAGS) | (raw & MUTABLE_FLAGS))
    });
    Ok(())
}

/// 通过单次原子位或/位与修改 OFD 的 `O_NONBLOCK` 标志位，不覆盖并发修改的其他可变标志。
pub(crate) fn set_nonblocking(flags: &AtomicU32, enabled: bool) -> Result<(), FsError> {
    if enabled {
        flags.fetch_or(O_NONBLOCK, Ordering::Relaxed);
    } else {
        flags.fetch_and(!O_NONBLOCK, Ordering::Relaxed);
    }
    Ok(())
}

/// 在目录项成功写入调用方缓冲区后，将目录游标推进到该目录项记录的下一位置（同时更新可见 cookie 与哈希冲突链内部偏移）。
fn commit_dirent(cursor: &mut DirectoryCursor, entry: &DirectoryEntry) {
    *cursor = entry.next_cursor;
}

/// 在 `cursor` 睡眠锁保护下计算并校验普通文件的新字节偏移（要求 `0 <= position <= i64::MAX`），合法时更新游标并返回新位置。
fn seek_path(path: &Path, cursor: &SleepMutex<u64>, from: SeekFrom) -> Result<u64, FsError> {
    let mut cursor = cursor.lock();
    let position = match from {
        SeekFrom::Start(position) => Some(position),
        SeekFrom::Current(delta) => cursor.checked_add_signed(delta),
        SeekFrom::End(delta) => path
            .inode()?
            .metadata_snapshot()?
            .size
            .checked_add_signed(delta),
    }
    .ok_or(FsError::InvalidOperation)?;
    if (i64::MAX as u64) < position {
        return Err(FsError::InvalidOperation);
    }

    *cursor = position;
    Ok(position)
}

/// 在目录 `cursor` 睡眠锁保护下重定位目录遍历游标。
///
/// `SeekFrom::Current(0)` 直接返回当前 cookie 而不破坏同哈希桶内的续读进度；其他显式定位则按目标 cookie 重新构造目录游标（从对应哈希位置首项重新枚举）。
fn seek_directory(
    path: &Path,
    cursor: &SleepMutex<DirectoryCursor>,
    from: SeekFrom,
) -> Result<u64, FsError> {
    let mut cursor = cursor.lock();
    let position = match from {
        SeekFrom::Start(position) => position,
        SeekFrom::Current(0) => return Ok(cursor.cookie()),
        SeekFrom::Current(delta) => cursor
            .cookie()
            .checked_add_signed(delta)
            .ok_or(FsError::InvalidOperation)?,
        SeekFrom::End(delta) => path
            .inode()?
            .directory_end_cookie()?
            .checked_add_signed(delta)
            .ok_or(FsError::InvalidOperation)?,
    };
    if (i64::MAX as u64) < position {
        return Err(FsError::InvalidOperation);
    }

    let next = path.inode()?.directory_cursor(position)?;
    let cookie = next.cookie();
    *cursor = next;
    Ok(cookie)
}

/// 从已解析路径对应的底层 `Inode` 提取当前元数据快照并组装为 `FileStat`（支持未打开的符号链接本身属性查询，如 `lstat`）。
pub fn stat_path(path: &Path) -> Result<FileStat, FsError> {
    let inode = path.inode()?;
    let data = inode.metadata_snapshot()?;
    if data.kind == InodeKind::Other {
        return Err(FsError::Unsupported);
    }
    Ok(FileStat {
        inode: inode.number(),
        kind: data.kind,
        mode: (data.mode & S_IALLUGO)
            | match data.kind {
                InodeKind::Regular => S_IFREG,
                InodeKind::Directory => S_IFDIR,
                InodeKind::Symlink => S_IFLNK,
                InodeKind::Character => S_IFCHR,
                InodeKind::Block => S_IFBLK,
                InodeKind::Fifo => S_IFIFO,
                InodeKind::Socket => S_IFSOCK,
                InodeKind::Other => unreachable!(),
            },
        size: data.size,
        uid: data.uid,
        gid: data.gid,
        links: data.links,
        blocks: data.blocks,
        device: inode.device(),
        rdev: inode.rdev(),
        block_size: inode.block_size(),
        atime: data.atime,
        mtime: data.mtime,
        ctime: data.ctime,
    })
}
