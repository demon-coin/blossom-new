//! 每次打开拥有独立位置，描述符复制只共享同一个具体文件对象。

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

/// 共享一个打开实例及其位置；最后引用释放该实例持有的路径。
pub type FileRef = Arc<dyn FileLike>;

/// 显式保存本次调用者；内核缓冲与用户缓冲不能替代线程身份。
#[derive(Clone, Copy)]
pub enum FileIoContext<'a> {
    Kernel,
    User(&'a UserThread),
}

pub(crate) const O_ACCMODE: u32 = 0o3;
pub(crate) const O_RDONLY: u32 = 0o0;
pub(crate) const O_WRONLY: u32 = 0o1;
pub(crate) const O_RDWR: u32 = 0o2;
pub(crate) const O_CREAT: u32 = 0o100;
pub(crate) const O_EXCL: u32 = 0o200;
pub(crate) const O_TRUNC: u32 = 0o1000;
pub(crate) const O_APPEND: u32 = 0o2000;
pub(crate) const O_NONBLOCK: u32 = 0o4000;
pub(crate) const O_DSYNC: u32 = 0o10000;
const __O_SYNC: u32 = 0o4000000;
pub(crate) const O_SYNC: u32 = __O_SYNC | O_DSYNC;
pub(crate) const O_LARGEFILE: u32 = 0o100000;
pub(crate) const O_DIRECTORY: u32 = 0o200000;
pub(crate) const O_NOFOLLOW: u32 = 0o400000;
pub(crate) const O_CLOEXEC: u32 = 0o2000000;
pub(crate) const O_NOCTTY: u32 = 0o400;
const MUTABLE_FLAGS: u32 = O_APPEND | O_NONBLOCK;
const OPEN_FLAGS: u32 = MUTABLE_FLAGS | O_SYNC | O_LARGEFILE | O_DIRECTORY | O_NOFOLLOW | O_NOCTTY;

/// 已从目标 iovec ABI 解码的地址与长度；不借用用户字节。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct UserIoVector {
    pub address: usize,
    pub length: usize,
}

/// 同一请求的逐段游标，零长度段不产生用户访问。
struct UserVector<'a> {
    mm: &'a Mm,
    vectors: &'a [UserIoVector],
    index: usize,
    offset: usize,
    length: usize,
}

impl<'a> UserVector<'a> {
    /// 验证内核快照的非零范围并计算无溢出的完整请求长度。
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

    /// 跳过已完成和空段，返回下一段剩余范围。
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

/// 同一文件页循环的两个接收端，实际交付量独立于请求长度。
pub struct ReadBuffer<'a> {
    target: ReadTarget<'a>,
    written: usize,
}

/// 内核切片遵守独占借用；用户地址只由绑定的 Mm 执行逐页原始复制。
enum ReadTarget<'a> {
    Kernel(&'a mut [u8]),
    User {
        mm: &'a Mm,
        address: usize,
        length: usize,
    },
    Vector(UserVector<'a>),
}

impl<'a> ReadBuffer<'a> {
    /// 将内核消费者绑定到同一文件页交付循环。
    pub fn kernel(bytes: &'a mut [u8]) -> Self {
        Self {
            target: ReadTarget::Kernel(bytes),
            written: 0,
        }
    }

    /// 一次验证非零用户范围；零长请求不要求地址可访问。
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

    /// 借用已拥有的向量快照，同一 OFD 调用内逐段交付用户前缀。
    pub fn user_vector(mm: &'a Mm, vectors: &'a [UserIoVector]) -> Result<Self, FsError> {
        Ok(Self {
            target: ReadTarget::Vector(UserVector::new(mm, vectors)?),
            written: 0,
        })
    }

    /// 返回本次接收端的完整容量，不包含已交付进度。
    pub fn len(&self) -> usize {
        match &self.target {
            ReadTarget::Kernel(bytes) => bytes.len(),
            ReadTarget::User { length, .. } => *length,
            ReadTarget::Vector(vector) => vector.length,
        }
    }

    /// 观察已经真正写入接收端的前缀，故障不会撤回这段进度。
    pub fn copied(&self) -> usize {
        self.written
    }

    /// 从独占内核片段交付字节；可能处理用户 fault，调用时不得持源文件页内容锁。
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

/// 写入端只保存有限输入准备进度，文件实际接受量由 inode 单独提交。
pub struct WriteBuffer<'a> {
    source: WriteSource<'a>,
    prepared: usize,
    error: Option<FsError>,
}

/// 用户字节只能由指定 Mm 复制到独占片段，不建立用户内容的 Rust 借用。
enum WriteSource<'a> {
    Kernel(&'a [u8]),
    User {
        mm: &'a Mm,
        address: usize,
        length: usize,
    },
    Vector(UserVector<'a>),
}

impl<'a> WriteBuffer<'a> {
    /// 将已初始化的内核字节绑定到正式 inode 写循环。
    pub fn kernel(bytes: &'a [u8]) -> Self {
        Self {
            source: WriteSource::Kernel(bytes),
            prepared: 0,
            error: None,
        }
    }

    /// 验证非零用户范围并绑定真实 Mm；零长请求不访问地址。
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

    /// 借用向量快照，输入准备量与文件接受量继续分开记录。
    pub fn user_vector(mm: &'a Mm, vectors: &'a [UserIoVector]) -> Result<Self, FsError> {
        Ok(Self {
            source: WriteSource::Vector(UserVector::new(mm, vectors)?),
            prepared: 0,
            error: None,
        })
    }

    /// 返回输入的完整长度，不把准备片段当成已接受的文件字节。
    pub fn len(&self) -> usize {
        match &self.source {
            WriteSource::Kernel(bytes) => bytes.len(),
            WriteSource::User { length, .. } => *length,
            WriteSource::Vector(vector) => vector.length,
        }
    }

    /// 返回已取得的输入前缀，仅供剩余输入计算。
    pub fn copied(&self) -> usize {
        self.prepared
    }

    /// 只准备目的片段容量内的输入，调用时不能持目的 folio、块根写锁或 PTE 锁。
    /// 部分 copyin 先交出可提交前缀，首个错误留待下一次调用，不能吞掉已有字节。
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

/// 可表示为有符号64位的非负文件位置；相对位移失败时保留原位置。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SeekFrom {
    Start(u64),
    Current(i64),
    End(i64),
}

/// 真实 inode 当前属性与卷几何的小快照，不包含块映射根。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FileStat {
    pub inode: u64,
    pub kind: InodeKind,
    pub mode: u16,
    pub size: u64,
    pub uid: u32,
    pub gid: u32,
    pub links: u32,
    /// 已分配空间按 512 字节计数。
    pub blocks: u64,
    pub device: u64,
    /// 字符或块节点指向的设备号，普通节点为零。
    pub rdev: u64,
    pub block_size: u32,
    pub atime: InodeTime,
    pub mtime: InodeTime,
    pub ctime: InodeTime,
}

/// 普通可睡眠任务使用的文件操作；调用者须在描述符表锁外持有实例引用。
pub trait FileLike: Send + Sync {
    /// 内核顺序读取使用与用户读取相同的页交付和位置提交协议。
    fn read(&self, dst: &mut [u8]) -> Result<usize, FsError> {
        self.read_into(&mut ReadBuffer::kernel(dst), FileIoContext::Kernel)
    }

    /// 内核按位置读取使用同一页循环，不取得共享位置锁。
    fn read_at(&self, offset: u64, dst: &mut [u8]) -> Result<usize, FsError> {
        self.read_at_into(offset, &mut ReadBuffer::kernel(dst), FileIoContext::Kernel)
    }

    /// 从共享位置向具体接收端交付，只提交已交付的正前缀。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;

    /// 按指定位置向接收端交付，不获取或修改共享位置。
    fn read_at_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;

    /// 只复制本打开实例的预读提示，目录没有文件读历史；不取得位置或页内容锁。
    #[cfg(feature = "ktest")]
    fn ktest_readahead_snapshot(&self) -> Option<ReadaheadSnapshot> {
        None
    }

    /// 只借用对象实际拥有的路径，无路径的流不能作为 dirfd 起点。
    fn path(&self) -> Option<&Path>;

    /// 只由普通可映射文件提供实际 inode，映射者自行保活该身份。
    fn mapping_inode(&self) -> Option<&InodeRef>;

    /// 只有存在显式位置语义的对象允许pread/pwrite；流在访问模式和用户地址之前拒绝。
    fn positioned_io(&self) -> bool {
        false
    }

    /// 借用实际持续事件 provider，普通文件与目录没有持久通知源。
    fn pollable(&self) -> Option<&dyn Pollable> {
        None
    }

    /// 只有事件对象提供实际epoll状态，raw入口不得从文件外观猜对象类型。
    fn epoll(&self) -> Option<&EpollFile> {
        None
    }

    /// 只借用实际socket端点，普通文件、管道和其它匿名对象不提供网络控制能力。
    fn socket(&self) -> Option<&SocketFile> {
        None
    }

    /// 只借用实际普通文件打开实例，跨文件传输不从属性或名字推断对象。
    fn inode_file(&self) -> Option<&InodeFile> {
        None
    }

    /// 只借用实际匿名管道端点，其它流没有管道页环能力。
    fn pipe_end(&self) -> Option<&PipeEnd> {
        None
    }

    /// 可持久监测的OFD保留反向watch，最后释放先同步撤登记再结束其源拥有。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        None
    }

    /// 没有设备控制能力的文件返回 ENOTTY，不把未知命令当作成功。
    fn ioctl(
        &self,
        _command: u32,
        _argument: usize,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::NotTty)
    }

    /// 读取此 OFD 的访问模式和状态，CLOEXEC 只在描述符槽保存。
    fn flags(&self) -> u32;

    /// 只改变支持的可变状态位，不修改访问模式与打开时路径约束。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError>;

    /// 原子修改此 OFD 的非阻塞位，保留追加状态与全部不可变打开位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError>;

    /// 在表锁与共享位置锁外截断普通可写文件，保持原位置；本操作可能等待真实 I/O。
    fn truncate(&self, new_size: u64) -> Result<(), FsError>;

    /// 同步半开字节范围及其必要元数据和设备缓存，并推进原 OFD 的错误观察游标。
    /// 调用者不得持描述符表、位置、inode 写锁或 mmap/PTE 保护；本操作可能等待真实 I/O。
    fn sync_range(&self, range: Range<u64>, data_only: bool) -> Result<(), FsError>;

    /// 更新共享字节位置，负值或溢出不修改原位置。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError>;

    /// 读取 canonical inode 的属性快照，不获取位置锁。
    fn stat(&self) -> Result<FileStat, FsError>;

    /// 返回下一项及其原始名字，成功取得项后才推进目录位置。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError>;

    /// 回调完整交付一项并返回 true 后提交其 cookie；false 或错误保留该项位置。
    fn for_each_dirent(
        &self,
        emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError>;

    /// 内核顺序写与用户写共用同一输入准备及文件提交协议。
    fn write(&self, src: &[u8]) -> Result<usize, FsError> {
        self.write_from(&mut WriteBuffer::kernel(src), FileIoContext::Kernel)
    }

    /// 内核按位置写不取得共享位置锁，追加状态仍由 inode 仲裁。
    fn write_at(&self, offset: u64, src: &[u8]) -> Result<usize, FsError> {
        self.write_at_from(offset, &mut WriteBuffer::kernel(src), FileIoContext::Kernel)
    }

    /// 持共享位置直到实际写结束，只提交 inode 返回的起点和接受前缀。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;

    /// 按明确位置写入，不取得或修改共享位置。
    fn write_at_from(
        &self,
        offset: u64,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError>;
}

/// 普通文件的打开实例，路径保活挂载和唯一 inode 的文件页索引。
pub struct InodeFile {
    path: Path,
    cursor: SleepMutex<u64>,
    readahead: SpinLock<ReadaheadState>,
    flags: AtomicU32,
    write_access: Option<WriteAccess>,
    /// 只有磁盘 OFD 保存数据与共同元数据错误样本，短锁只覆盖检查与推进。
    wb_errors: SpinLock<Option<(u32, u32)>>,
}

/// 目录的打开实例保存后端cookie和相同hash项的真实续读位置。
pub struct DirectoryFile {
    path: Path,
    cursor: SleepMutex<DirectoryCursor>,
    flags: AtomicU32,
    wb_error: SpinLock<Option<u32>>,
}

/// 内核默认只读打开与用户带标志打开共用唯一构造。
pub fn open_path(vfs: &Vfs, path: Path) -> Result<FileRef, FsError> {
    open_path_with_flags(vfs, path, 0, FileIoContext::Kernel)
}

/// 按已确定的真实对象建立完整 OFD；设备与动态信息取得本次调用者，创建候选由 VFS 交入。
/// O_TRUNC 在 OFD 存储准备后执行；只读截断的临时写拥有不转入打开实例。
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
    // 独立的完整同步位同样隐含数据同步位，写后路径只需据 O_DSYNC 判断是否同步。
    let flags = if flags & (O_SYNC & !O_DSYNC) != 0 {
        flags | O_DSYNC
    } else {
        flags
    };
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
            // 完整 OFD 已分配，截断后只交出原拥有，描述符调用者再消费预留槽位。
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
    /// 传输调用者借用本打开实例，原 FileRef 继续承担完整寿命。
    fn inode_file(&self) -> Option<&InodeFile> {
        Some(self)
    }

    /// 普通文件的显式字节位置由同一inode维护，不依赖共享游标或访问模式。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 位置锁跨实际交付保持独占，用户故障的正前缀仍按真实长度推进。
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
        // inode 限制交付不超过 size-cursor，EOF 与零长返回零。
        *cursor += bytes as u64;
        Ok(bytes)
    }

    /// 按位置读取更新同一 OFD 的短预读提示，不参与共享位置的串行化。
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

    /// 只观察磁盘 OFD 的预读历史，RAM 文件没有磁盘预读提示。
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

    /// 由完整路径保活挂载，调用者可克隆此路径作为 dirfd 起点。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 构造时已经取得正 inode；路径身份在此 OFD 寿命内不重新绑定。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        Some(self.path.inode().expect("regular OFD lost its inode"))
    }

    /// OFD 状态由原子快照读取，不获取位置锁。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新同一 OFD 的可变状态，dup 的所有引用观察相同状态。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 复制引用共享同一非阻塞位，不读取或回写其它状态的旧快照。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 借用此 OFD 的原写拥有改变文件大小，不重新检查当前凭据、不消费位置或追加状态。
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

    /// 在所有写入与位置保护之外同步实际范围，直接错误优先于后续阶段，样本仍全部推进。
    fn sync_range(&self, range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        if range.end < range.start {
            return Err(FsError::InvalidOperation);
        }
        let inode = self.path.inode()?;
        if inode.is_memory() {
            return Ok(());
        }
        let volume = inode.disk_volume()?;
        // 只读卷没有可提交的写义务；只读 OFD 或挂载视图仍须同步 RW 卷上的共同内容。
        let writable = volume.mode() == AccessMode::ReadWrite;
        let data = if writable {
            writeback::sync_inode_range(inode, range)
        } else {
            Ok(())
        };
        // 共同元数据同步覆盖本卷当前选中的 inode、块树、分配和目录记录，可写出更多元数据。
        // 文件同步不保证跨文件原子提交，fdatasync 仍须提交使数据可寻址的记录和设备 FLUSH。
        // 未确认设备停止时保持原在途拥有，禁止继续提交元数据或 FLUSH。
        let metadata = if writable && !matches!(data, Err(FsError::Io(BlockError::StopUnconfirmed)))
        {
            writeback::sync_metadata(volume)
        } else {
            Ok(())
        };
        let mut samples = self.wb_errors.lock();
        let samples = samples.as_mut().expect("disk OFD has no writeback samples");
        let data_error = inode.mapping().check_and_advance_wb_error(&mut samples.0);
        let metadata_error = volume
            .metadata_cache()
            .check_and_advance_wb_error(&mut samples.1);
        data.and(data_error).and(metadata).and(metadata_error)
    }

    /// 与顺序读取共用位置锁，允许定位到文件末尾之后。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError> {
        seek_path(&self.path, &self.cursor, from)
    }

    /// 只复制属性字段，不复制完整元数据或文件内容。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 普通文件不能作为目录枚举。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 普通文件不提供目录流，不调用消费者。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 接受前缀先提交位置并放写保护，再完成同步和脏节流；失败不回滚已提交位置。
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

    /// pwrite 不取得共享位置，追加由 inode 仲裁，接受前缀在写保护外完成同步和脏节流。
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
    /// 在取得任何管道环之前锁住本 OFD 的隐式位置，调用者按稳定对象地址排序。
    pub(crate) fn transfer_cursor(&self) -> SleepMutexGuard<'_, u64> {
        self.cursor.lock()
    }

    /// 传输先核打开写拥有及当前挂载权限，追加目标由上层按 syscall 错误语义拒绝。
    pub(crate) fn transfer_check_write(&self) -> Result<(), FsError> {
        require_write(self.flags())?;
        self.path.check_writable()
    }

    /// 只执行原 inode 写循环并返回实际接受量；调用者随后提交位置并在锁外完成同步。
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

    /// 位置和管道环释放后履行真实接受范围的同步及脏节流义务，不撤回已接受数据。
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

/// 仅对真实正接受量执行同步，O_SYNC 同时包含 O_DSYNC，失败返回原错误而非假正前缀。
fn sync_written(file: &InodeFile, flags: u32, start: u64, bytes: usize) -> Result<(), FsError> {
    if bytes == 0 || flags & O_DSYNC == 0 {
        return Ok(());
    }
    let end = start
        .checked_add(bytes as u64)
        .expect("accepted write range overflow");
    file.sync_range(start..end, flags & O_SYNC != O_SYNC)
}

/// 调用者位置和本次输入范围均须可表示为非负 loff_t，追加也不能跳过 ABI 范围检查。
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
    /// 目录保留文件位置能力，普通字节访问仍由其操作返回目录错误。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 目录字节由目录项操作消费，不能按普通文件读取。
    fn read_into(
        &self,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IsDirectory)
    }

    /// 指定位置的普通字节读取同样拒绝目录。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IsDirectory)
    }

    /// 目录实例保活完整路径，可作为相对路径起点。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 目录只提供目录项流，不授予普通文件映射能力。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 读取此目录 OFD 状态，不获取枚举位置锁。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 目录状态与描述符属性分离，不改变枚举位置。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 只改变同一目录 OFD 的非阻塞位，不影响枚举位置与打开约束。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 目录描述符不提供普通文件截断，错误不改变枚举位置。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 目录同步沿其真实共同元数据与 FLUSH，不取得枚举位置锁或虚构名字创建依赖。
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

    /// 按真实后端cookie定位；显式seek从新位置重新枚举hash碰撞。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError> {
        seek_directory(&self.path, &self.cursor, from)
    }

    /// 目录属性直接来自真实 inode，不获取枚举位置锁。
    fn stat(&self) -> Result<FileStat, FsError> {
        stat_path(&self.path)
    }

    /// 内核单项枚举保留真实后端cookie和hash碰撞续读状态。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        let mut cursor = self.cursor.lock();
        let entry = self.path.inode()?.dir_next(&cursor)?;
        if let Some(entry) = &entry {
            commit_dirent(&mut cursor, entry);
        }
        Ok(entry)
    }

    /// 只有消费者完整接受一项后才提交其后端cookie及碰撞续读状态。
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

    /// 目录不接受字节写入，访问模式检查与挂载能力检查保持各自错误。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_write(self.flags())?;
        self.path.check_writable()?;
        Err(FsError::IsDirectory)
    }

    /// 按位置写同样拒绝目录，不获取枚举位置锁。
    fn write_at_from(
        &self,
        _offset: u64,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        self.write_from(source, caller)
    }
}

/// 写模式 OFD 不授予读取或文件映射能力，空请求同样核访问模式。
pub(crate) fn require_read(flags: u32) -> Result<(), FsError> {
    if flags & O_ACCMODE == O_WRONLY {
        Err(FsError::BadFd)
    } else {
        Ok(())
    }
}

/// 只读 OFD 不授予写能力，挂载检查不能替代打开时访问模式。
pub(crate) fn require_write(flags: u32) -> Result<(), FsError> {
    if flags & O_ACCMODE == 0 {
        Err(FsError::BadFd)
    } else {
        Ok(())
    }
}

/// 忽略创建、访问和描述符位；未支持的可变能力先失败，再原子提交 APPEND/NONBLOCK。
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

/// 单次原子读改写只修改非阻塞位，与其它状态更新共享同一原子对象。
pub(crate) fn set_nonblocking(flags: &AtomicU32, enabled: bool) -> Result<(), FsError> {
    if enabled {
        flags.fetch_or(O_NONBLOCK, Ordering::Relaxed);
    } else {
        flags.fetch_and(!O_NONBLOCK, Ordering::Relaxed);
    }
    Ok(())
}

/// 完整交付后同时提交可见cookie和后端续读状态。
fn commit_dirent(cursor: &mut DirectoryCursor, entry: &DirectoryEntry) {
    *cursor = entry.next_cursor;
}

/// 同一位置锁内计算并校验非负loff_t，再提交共享位置，失败不改变游标。
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
/// 显式seek按后端cookie重置碰撞状态；SEEK_CUR(0)保留真实续读位置。
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

/// 从 canonical inode 一次读取当前属性快照，未打开的符号链接同样能取得属性。
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
