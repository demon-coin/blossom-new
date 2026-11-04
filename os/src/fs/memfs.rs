//! 内存文件系统保存真实 inode 和目录内容，名字缓存只保存可退休的查找结果。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};
use spin::Once;

use crate::block::AccessMode;
use crate::fs::dentry::{Dentry, DentryCache, DentryRef};
use crate::fs::ext4::Ext4Volume;
use crate::fs::inode::{
    Inode, InodeCache, InodeMetadata, InodeRef, MemoryContent, MemoryDirectory,
};
use crate::fs::types::{InodeKind, S_IALLUGO, S_IFCHR, S_IFDIR, S_IFREG, S_IRWXUGO, S_ISVTX};
use crate::fs::FsError;
use crate::mm::PAGE_SIZE;
use crate::sync::{SleepMutex, SleepMutexGuard, SpinLock};

mod options;
pub use options::TmpfsOptions;

const TMPFS_MAGIC: u64 = 0x0102_1994;
const DEVFS_MAGIC: u64 = 0x1373;
const PROC_SUPER_MAGIC: u64 = 0x9fa0;
const MEMORY_DEVICE_BASE: u64 = 1u64 << u32::BITS;
static NEXT_DEVICE: AtomicU64 = AtomicU64::new(MEMORY_DEVICE_BASE);

/// 后端容量快照只保存块与 inode 计数；挂载模式由取得快照的完整 Path 提供。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FilesystemStat {
    pub fs_type: u64,
    pub block_size: u64,
    pub blocks: u64,
    pub free: u64,
    pub available: u64,
    pub files: u64,
    pub freefiles: u64,
    pub device: u64,
}

/// 挂载强持真实后端实例，磁盘身份与内存身份各自比较。
#[derive(Clone)]
pub enum Filesystem {
    Ext4(Arc<Ext4Volume>),
    Memory(Arc<MemoryFs>),
}

impl Filesystem {
    /// 返回实际实例的设备身份，内存实例不借用磁盘号。
    pub fn device(&self) -> u64 {
        match self {
            Self::Ext4(fs) => fs.device(),
            Self::Memory(fs) => fs.device(),
        }
    }

    /// 在普通任务中采样实际后端容量；磁盘元数据读取可能睡眠并保留原始失败。
    pub fn statfs(&self) -> Result<FilesystemStat, FsError> {
        match self {
            Self::Ext4(fs) => fs.statfs(),
            Self::Memory(fs) => fs.statfs(),
        }
    }

    /// 返回后端的名字与普通文件修改权限。
    pub fn mode(&self) -> AccessMode {
        match self {
            Self::Ext4(fs) => fs.mode(),
            Self::Memory(fs) => fs.mode(),
        }
    }

    /// 内存内容已经是权威状态，没有磁盘回写或 FLUSH 义务。
    pub fn sync(&self) -> Result<(), FsError> {
        match self {
            Self::Ext4(fs) => fs.sync(),
            Self::Memory(_) => Ok(()),
        }
    }

    /// 比较同一类实际后端的 Arc 身份。
    pub fn ptr_eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Ext4(a), Self::Ext4(b)) => Arc::ptr_eq(a, b),
            (Self::Memory(a), Self::Memory(b)) => Arc::ptr_eq(a, b),
            _ => false,
        }
    }

    /// 以实际 Arc 地址排序同实例 rename 锁，地址只用于内核同步且不作为设备号。
    pub(crate) fn identity_address(&self) -> usize {
        match self {
            Self::Ext4(fs) => Arc::as_ptr(fs) as usize,
            Self::Memory(fs) => Arc::as_ptr(fs) as usize,
        }
    }

    /// 按后端取得共同名字域，内存节点不登记磁盘 canonical 域。
    pub(crate) fn dentry_namespace(
        &self,
        inodes: &Arc<InodeCache>,
        capacity: usize,
    ) -> Result<(Arc<DentryCache>, DentryRef), FsError> {
        match self {
            Self::Ext4(fs) => inodes.dentry_namespace(fs, capacity),
            Self::Memory(fs) => fs.dentry_namespace(capacity),
        }
    }

    /// rename 的拓扑验证和多目录提交在同一实例睡眠互斥下执行。
    pub(crate) fn lock_rename(&self) -> SleepMutexGuard<'_, ()> {
        match self {
            Self::Ext4(fs) => fs.lock_rename(),
            Self::Memory(fs) => fs.rename.lock(),
        }
    }
}

/// 内存后端种类固定，不提供运行时插件登记。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemoryFsKind {
    Tmpfs,
    Devfs,
    Procfs,
}

/// 特殊 inode 只标识实际设备或动态快照来源，字节操作由具体 OFD 提供。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemorySpecial {
    Null,
    Zero,
    TtySerial,
    Console,
    ControllingTty,
    Kmsg,
    Ktrace,
    TestHook,
    Cmdline,
    Mounts,
    Meminfo,
    Uptime,
    Interrupts,
    NetDev,
    NetRoute,
}

/// 根与权威目录项持有 inode，inode 只弱指向本实例；外部引用保活本实例。
pub struct MemoryFs {
    kind: MemoryFsKind,
    device: u64,
    root: Once<Arc<Inode>>,
    rename: SleepMutex<()>,
    allocation: SleepMutex<()>,
    index: SpinLock<MemoryIndex>,
}

struct MemoryIndex {
    inodes: Vec<MemorySlot>,
    next_number: u64,
    pages: usize,
    max_pages: usize,
    max_inodes: usize,
    namespace: Weak<DentryCache>,
    root_dentry: Weak<Dentry>,
    /// 名字域首次公开后禁止可信初始化，即使弱 cache/root 此后已退休。
    namespace_published: bool,
}

struct MemorySlot {
    number: u64,
    inode: Weak<Inode>,
}

/// 单个实际原页的预留票由 folio 内容持有到最后归页，不能复制。
pub(crate) struct MemoryPageCharge {
    filesystem: Weak<MemoryFs>,
}

impl Drop for MemoryPageCharge {
    /// 原页最后释放或未发表申请失败时归还本实例额度。
    fn drop(&mut self) {
        if let Some(fs) = self.filesystem.upgrade() {
            let mut index = fs.index.lock();
            assert!(0 < index.pages, "memory page charge lost");
            index.pages -= 1;
        }
    }
}

impl MemoryFs {
    /// 普通 tmpfs 使用实际 RAM 默认额度，可信 dev/proc 节点不施加挂载额度。
    pub fn try_new(kind: MemoryFsKind, root_mode: u16) -> Result<Arc<Self>, FsError> {
        let limits = if kind == MemoryFsKind::Tmpfs {
            TmpfsOptions::default().limits(managed_pages())
        } else {
            (0, 0)
        };
        Self::try_with_limits(kind, root_mode, limits)
    }

    /// 解析用户挂载选项并构造同一真实状态，不预分配额度对应的 inode 或页。
    pub fn try_tmpfs(options: &[u8]) -> Result<Arc<Self>, FsError> {
        let pages = managed_pages();
        let options = TmpfsOptions::parse(options, pages)?;
        Self::try_with_limits(
            MemoryFsKind::Tmpfs,
            options.mode.unwrap_or(S_ISVTX | S_IRWXUGO),
            options.limits(pages),
        )
    }

    /// 所有内存实例共用根和计费状态，只有额度策略由可信调用者选定。
    fn try_with_limits(
        kind: MemoryFsKind,
        root_mode: u16,
        (max_pages, max_inodes): (usize, usize),
    ) -> Result<Arc<Self>, FsError> {
        let device = NEXT_DEVICE
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
                value.checked_add(1)
            })
            .map_err(|_| FsError::NoSpace)?;
        let fs = Arc::try_new(Self {
            kind,
            device,
            root: Once::new(),
            rename: SleepMutex::new(()),
            allocation: SleepMutex::new(()),
            index: SpinLock::new(MemoryIndex {
                inodes: Vec::new(),
                next_number: 1,
                pages: 0,
                max_pages,
                max_inodes,
                namespace: Weak::new(),
                root_dentry: Weak::new(),
                namespace_published: false,
            }),
        })
        .map_err(|_| FsError::NoMemory)?;
        let metadata = InodeMetadata::memory(
            InodeKind::Directory,
            S_IFDIR | (root_mode & S_IALLUGO),
            0,
            0,
            2,
            0,
        );
        let root = fs.allocate(
            metadata,
            MemoryContent::Directory(SpinLock::new(MemoryDirectory {
                parent: 1,
                next_cookie: 2,
                entries: Vec::new(),
            })),
            0,
        )?;
        fs.root.call_once(|| root.memory_arc());
        Ok(fs)
    }

    /// 返回实际构造种类，供挂载快照输出固定文件系统名称。
    pub fn kind(&self) -> MemoryFsKind {
        self.kind
    }

    /// 每个内存实例有独立且终身稳定的非零设备号。
    pub fn device(&self) -> u64 {
        self.device
    }

    /// 只有 tmpfs 名字和普通内容可修改；设备数据面不使用此权限。
    pub fn mode(&self) -> AccessMode {
        match self.kind {
            MemoryFsKind::Tmpfs => AccessMode::ReadWrite,
            MemoryFsKind::Devfs | MemoryFsKind::Procfs => AccessMode::ReadOnly,
        }
    }

    /// 同一短索引锁内观察额度、实际原页票与活 canonical inode；无限字段保持零。
    pub fn statfs(&self) -> Result<FilesystemStat, FsError> {
        let mut stat = FilesystemStat {
            fs_type: match self.kind {
                MemoryFsKind::Tmpfs => TMPFS_MAGIC,
                MemoryFsKind::Devfs => DEVFS_MAGIC,
                MemoryFsKind::Procfs => PROC_SUPER_MAGIC,
            },
            block_size: PAGE_SIZE as u64,
            blocks: 0,
            free: 0,
            available: 0,
            files: 0,
            freefiles: 0,
            device: self.device,
        };
        if self.kind == MemoryFsKind::Tmpfs {
            let index = self.index.lock();
            let used_inodes = index.used_inodes();
            stat.blocks = index.max_pages as u64;
            if index.max_pages != 0 {
                stat.free = (index.max_pages - index.pages) as u64;
            }
            stat.available = stat.free;
            stat.files = index.max_inodes as u64;
            if index.max_inodes != 0 {
                stat.freefiles = (index.max_inodes - used_inodes) as u64;
            }
        }
        Ok(stat)
    }

    /// 与 inode 构造和页票预留串行，全部 seen 额度校验成功后一次提交。
    /// mode 只用于初始根，重配不会修改根 inode 或替换原名字和 mapping。
    pub fn reconfigure(&self, bytes: &[u8]) -> Result<(), FsError> {
        if self.kind != MemoryFsKind::Tmpfs {
            return Err(FsError::InvalidOperation);
        }
        let options = TmpfsOptions::parse(bytes, managed_pages())?;
        let _allocation = self.allocation.lock();
        let mut index = self.index.lock();
        if let Some(limit) = options.blocks {
            if limit != 0 && (index.max_pages == 0 || limit < index.pages) {
                return Err(FsError::InvalidOperation);
            }
        }
        if let Some(limit) = options.inodes {
            if limit != 0 && (index.max_inodes == 0 || limit < index.used_inodes()) {
                return Err(FsError::InvalidOperation);
            }
        }
        if let Some(limit) = options.blocks {
            index.max_pages = limit;
        }
        if let Some(limit) = options.inodes {
            index.max_inodes = limit;
        }
        Ok(())
    }

    /// 外部根引用保活本实例，根内部 Arc 不反向强持文件系统。
    pub fn root(self: &Arc<Self>) -> InodeRef {
        InodeRef::from_memory(
            self.root.get().expect("memory root absent").clone(),
            self.clone(),
        )
    }

    /// 在未发表的 dev/proc 父目录安装真实特殊节点，不向用户开放可信初始化权限。
    pub(crate) fn initialize_special(
        self: &Arc<Self>,
        parent: &InodeRef,
        name: &[u8],
        special: MemorySpecial,
        mode: u16,
        rdev: u64,
    ) -> Result<(), FsError> {
        let _rename = self.rename.lock();
        self.require_initialization_parent(parent)?;
        let kind = match (self.kind, special) {
            (
                MemoryFsKind::Devfs,
                MemorySpecial::Null
                | MemorySpecial::Zero
                | MemorySpecial::TtySerial
                | MemorySpecial::Console
                | MemorySpecial::ControllingTty
                | MemorySpecial::Kmsg
                | MemorySpecial::Ktrace
                | MemorySpecial::TestHook,
            ) => InodeKind::Character,
            (
                MemoryFsKind::Procfs,
                MemorySpecial::Cmdline
                | MemorySpecial::Mounts
                | MemorySpecial::Meminfo
                | MemorySpecial::Uptime
                | MemorySpecial::Interrupts
                | MemorySpecial::NetDev
                | MemorySpecial::NetRoute,
            ) => InodeKind::Regular,
            _ => return Err(FsError::InvalidOperation),
        };
        let mode = (mode & S_IALLUGO)
            | if kind == InodeKind::Character {
                S_IFCHR
            } else {
                S_IFREG
            };
        let inode = self.allocate(
            InodeMetadata::memory(kind, mode, 0, 0, 1, 0),
            MemoryContent::Special(special),
            rdev,
        )?;
        parent.initialize_memory_child(name, &inode)
    }

    /// 可信构造只在未发表的 dev/proc 实例创建目录，父编号与链接数沿共同 inode 协议。
    pub(crate) fn initialize_directory(
        self: &Arc<Self>,
        parent: &InodeRef,
        name: &[u8],
        mode: u16,
    ) -> Result<InodeRef, FsError> {
        let _rename = self.rename.lock();
        self.require_initialization_parent(parent)?;
        let inode = self.allocate(
            InodeMetadata::memory(
                InodeKind::Directory,
                S_IFDIR | (mode & S_IALLUGO),
                0,
                0,
                2,
                0,
            ),
            MemoryContent::Directory(SpinLock::new(MemoryDirectory {
                parent: parent.number(),
                next_cookie: 2,
                entries: Vec::new(),
            })),
            0,
        )?;
        parent.initialize_memory_child(name, &inode)?;
        Ok(inode)
    }

    /// 持 rename 锁时核可信初始化边界，拒绝已发表实例和其它实例的父节点。
    fn require_initialization_parent(&self, parent: &InodeRef) -> Result<(), FsError> {
        if self.kind == MemoryFsKind::Tmpfs
            || self.index.lock().namespace_published
            || parent.device() != self.device
        {
            return Err(FsError::InvalidOperation);
        }
        if parent.metadata_snapshot()?.kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        Ok(())
    }

    /// 睡眠分配锁保持弱槽检查与发表串行，短索引锁不跨堆申请。
    pub(crate) fn allocate(
        self: &Arc<Self>,
        mut metadata: InodeMetadata,
        content: MemoryContent,
        rdev: u64,
    ) -> Result<InodeRef, FsError> {
        let _allocation = self.allocation.lock();
        let (position, number, growth) = {
            let mut index = self.index.lock();
            if index.max_inodes != 0 && index.max_inodes <= index.used_inodes() {
                return Err(FsError::NoSpace);
            }
            let position = index
                .inodes
                .iter()
                .position(|slot| slot.inode.strong_count() == 0);
            let growth = if position.is_none() && index.inodes.len() == index.inodes.capacity() {
                Some(
                    index
                        .inodes
                        .len()
                        .checked_mul(2)
                        .ok_or(FsError::NoMemory)?
                        .max(4),
                )
            } else {
                None
            };
            let number = index.next_number;
            index.next_number = number.checked_add(1).ok_or(FsError::NoSpace)?;
            (position, number, growth)
        };
        let mut prepared = Vec::new();
        if let Some(length) = growth {
            prepared
                .try_reserve(length)
                .map_err(|_| FsError::NoMemory)?;
        }
        metadata.number = number;
        metadata.rdev = rdev;
        let inode = InodeRef::try_memory(self, metadata, content)?;
        let weak = Arc::downgrade(&inode.memory_arc());
        let slot = MemorySlot {
            number,
            inode: weak,
        };
        let (retired_slot, retired_buffer) = {
            let mut index = self.index.lock();
            if let Some(position) = position {
                (
                    Some(core::mem::replace(&mut index.inodes[position], slot)),
                    None,
                )
            } else {
                let retired = if growth.is_some() {
                    prepared.append(&mut index.inodes);
                    Some(core::mem::replace(&mut index.inodes, prepared))
                } else {
                    None
                };
                index.inodes.push(slot);
                (None, retired)
            }
        };
        drop((retired_slot, retired_buffer));
        Ok(inode)
    }

    /// 从可增长弱 canonical 索引取得原对象，仍被外部拥有的身份不能复用。
    pub(crate) fn find(self: &Arc<Self>, number: u64) -> Result<InodeRef, FsError> {
        let inode = self
            .index
            .lock()
            .inodes
            .iter()
            .find(|slot| slot.number == number)
            .and_then(|slot| slot.inode.upgrade())
            .ok_or(FsError::NotFound)?;
        Ok(InodeRef::from_memory(inode, self.clone()))
    }

    /// 原页分配前预留额度，失败申请与最终归页都消费同一不可复制票。
    pub(crate) fn reserve_page(self: &Arc<Self>) -> Result<MemoryPageCharge, FsError> {
        let _allocation = self.allocation.lock();
        let mut index = self.index.lock();
        if index.max_pages != 0 && index.max_pages <= index.pages {
            return Err(FsError::NoSpace);
        }
        index.pages = index.pages.checked_add(1).ok_or(FsError::NoMemory)?;
        Ok(MemoryPageCharge {
            filesystem: Arc::downgrade(self),
        })
    }

    /// 只观察本实例尚未归还的实际原页票，外部 folio 也继续占额。
    pub fn used_pages(&self) -> usize {
        self.index.lock().pages
    }

    /// 名字域只弱登记 cache/root，清名字缓存不清目录权威内容。
    fn dentry_namespace(
        self: &Arc<Self>,
        capacity: usize,
    ) -> Result<(Arc<DentryCache>, DentryRef), FsError> {
        if capacity == 0 {
            return Err(FsError::InvalidOperation);
        }
        let _rename = self.rename.lock();
        let (cache, root) = {
            let index = self.index.lock();
            (
                index.namespace.upgrade(),
                DentryRef::upgrade(&index.root_dentry),
            )
        };
        if let Some(cache) = &cache {
            if cache.capacity() != capacity {
                return Err(FsError::InvalidOperation);
            }
        }
        let cache = match cache {
            Some(cache) => cache,
            None => DentryCache::try_new(capacity)?,
        };
        let root = match root {
            Some(root) => root,
            None => DentryRef::try_root(self.root())?,
        };
        let mut index = self.index.lock();
        index.namespace = Arc::downgrade(&cache);
        index.root_dentry = root.downgrade();
        index.namespace_published = true;
        drop(index);
        Ok((cache, root))
    }
}

impl MemoryIndex {
    /// 权威目录及外部 fd/VMA/folio 保活的原 inode 都继续占额。
    fn used_inodes(&self) -> usize {
        self.inodes
            .iter()
            .filter(|slot| 0 < slot.inode.strong_count())
            .count()
    }
}

/// 使用启动后稳定的受管 RAM 总页数，不把当前空闲页或物理地址跨度当作总容量。
fn managed_pages() -> usize {
    let stats = crate::mm::pages().stats();
    stats.present_pages - stats.reserved_pages - stats.metadata_pages
}
