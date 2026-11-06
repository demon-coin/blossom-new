//! 内存文件系统（`tmpfs` / `devfs` / `procfs`）核心后端与统一文件系统分发模块。
//!
//! ## 模块职责与架构定位
//! 本模块承担两项核心职责：
//! 1. 定义统一的后端枚举 [`Filesystem`]，为上层虚拟文件系统（[`crate::fs::Vfs`]）屏蔽磁盘卷（[`Ext4Volume`]）与内存卷（[`MemoryFs`]）在设备号、容量统计、同步及目录项名字缓存上的差异。
//! 2. 实现基于内存的权威 Inode 与容量记账后端 [`MemoryFs`]，同时支撑可读写并受配额限制的 `tmpfs`、只读固定设备目录 `devfs` 以及只读系统信息目录 `procfs`。
//!
//! ## 核心设计与引用关系
//! `MemoryFs` 的目录结构与数据页直接常驻内存，无需磁盘回写；而目录项缓存（[`DentryCache`]）仅保存可随时淘汰的路径查找结果：
//!
//! ```text
//! 外部引用 (Mount / Path / InodeRef / MemoryPageCharge)
//!       │
//!       ▼ (Arc / Weak)
//! ┌──────────────────────────────────────────────────────────────┐
//! │ MemoryFs                                                     │
//! │  ├── kind: Tmpfs | Devfs | Procfs                            │
//! │  ├── root: Once<Arc<Inode>> (常驻根目录 Inode)               │
//! │  ├── rename: SleepMutex<()> (目录树拓扑修改与初始化互斥)     │
//! │  ├── allocation: SleepMutex<()> (Inode 分配/配额重配互斥)    │
//! │  └── index: SpinLock<MemoryIndex>                            │
//! │        ├── inodes: Vec<MemorySlot { number, Weak<Inode> }>   │
//! │        ├── pages / max_pages / max_inodes (配额账本)         │
//! │        └── namespace: Weak<DentryCache>, namespace_published │
//! └──────────────────────────────────────────────────────────────┘
//!       ▲                                      │
//!       │ (Weak<MemoryFs> 记账归还)            ▼ (Arc<Inode> 强引用)
//! MemoryPageCharge (由物理页 Folio 持有)   权威目录项 (MemoryDirectory.entries)
//!                                         及外部打开文件 / VMA (InodeRef)
//! ```
//!
//! - **生命周期不变量**：根目录 Inode 与父目录中的子目录项强持有 `Arc<Inode>`，而 `Inode` 内部仅通过 `Weak<MemoryFs>` 弱引用所属文件系统，避免循环引用；当文件被 `unlink` 且最后一个外部打开句柄（FD）或内存映射（VMA）释放时，`Arc<Inode>` 计数归零，`MemoryIndex::inodes` 中的对应弱引用槽位自动失效并可在后续分配中复用。
//! - **物理页配额记账（[`MemoryPageCharge`]）**：每次为 `tmpfs` 普通文件分配物理页（Folio）前必须调用 [`MemoryFs::reserve_page`] 预留 1 页额度；返回的 [`MemoryPageCharge`] 随物理页一同存活，当物理页被截断或文件销毁导致最后一个强引用释放时，在 `Drop` 中自动将 1 页额度归还给 `MemoryIndex`。
//!
//! ## 锁获取顺序与并发契约
//! 本模块涉及三把锁，严禁颠倒获取顺序：
//! 1. `MemoryFs::rename`（[`SleepMutex`]）：目录拓扑修改、名字域首次创建（`dentry_namespace`）以及内核可信节点初始化（`initialize_special` / `initialize_directory`）的最外层睡眠锁。
//! 2. `MemoryFs::allocation`（[`SleepMutex`]）：Inode 分配（`allocate`）、物理页配额预留（`reserve_page`）以及挂载配额重配置（`reconfigure`）的串行化睡眠锁；在 `initialize_*` 路径中可在持有 `rename` 锁的情况下获取 `allocation` 锁。
//! 3. `MemoryFs::index`（[`SpinLock`]）：最内层自旋锁，保护 `MemoryIndex` 的槽位数组与配额计数器。**严禁**在持有 `index` 自旋锁期间执行堆内存分配（如 `Vec::try_reserve`、`Arc::try_new`）或释放旧对象（`drop` 必须在锁外执行）。

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

/// `tmpfs` 文件系统魔数（对应 Linux `TMPFS_MAGIC = 0x01021994`）。
const TMPFS_MAGIC: u64 = 0x0102_1994;
/// `devfs` 文件系统魔数（对应 `DEVFS_SUPER_MAGIC = 0x1373`）。
const DEVFS_MAGIC: u64 = 0x1373;
/// `procfs` 文件系统魔数（对应 `PROC_SUPER_MAGIC = 0x9fa0`）。
const PROC_SUPER_MAGIC: u64 = 0x9fa0;

/// 内存文件系统实例设备号（`st_dev`）的起始基址（从 $2^{32}$ 开始递增，避免与块设备号冲突）。
const MEMORY_DEVICE_BASE: u64 = 1u64 << u32::BITS;
/// 全局递增的下一个内存文件系统设备号分配器。
static NEXT_DEVICE: AtomicU64 = AtomicU64::new(MEMORY_DEVICE_BASE);

/// 文件系统容量与 Inode 使用情况统计快照（用于填充 `statfs` / `fstatfs` 系统调用结果）。
///
/// 仅包含底层文件系统的块与 Inode 计数；挂载点层面的标志位（如只读挂载标志）由外层 [`crate::fs::Path`] 补充。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FilesystemStat {
    /// 文件系统类型魔数（如 [`TMPFS_MAGIC`]、[`DEVFS_MAGIC`]、[`PROC_SUPER_MAGIC`] 或 `EXT4_SUPER_MAGIC`）。
    pub fs_type: u64,
    /// 文件系统基本块大小（单位：字节；内存文件系统固定为 [`PAGE_SIZE`]）。
    pub block_size: u64,
    /// 文件系统总数据块数（单位：`block_size`；无上限时为 `0`）。
    pub blocks: u64,
    /// 空闲数据块数（单位：`block_size`；无上限时为 `0`）。
    pub free: u64,
    /// 非特权用户可用的空闲数据块数（单位：`block_size`）。
    pub available: u64,
    /// 文件系统最大可分配的 Inode 总数（无上限时为 `0`）。
    pub files: u64,
    /// 剩余可分配的空闲 Inode 数量（无上限时为 `0`）。
    pub freefiles: u64,
    /// 文件系统实例的唯一设备号（`f_fsid` / `st_dev`）。
    pub device: u64,
}

/// 挂载点持有的底层文件系统实例枚举。
#[derive(Clone)]
pub enum Filesystem {
    /// 基于块设备的 `ext4` 磁盘文件系统卷。
    Ext4(Arc<Ext4Volume>),
    /// 基于内存的 `tmpfs`、`devfs` 或 `procfs` 文件系统实例。
    Memory(Arc<MemoryFs>),
}

impl Filesystem {
    /// 返回底层文件系统实例的唯一设备号（`st_dev`）。
    pub fn device(&self) -> u64 {
        match self {
            Self::Ext4(fs) => fs.device(),
            Self::Memory(fs) => fs.device(),
        }
    }

    /// 在普通可睡眠任务上下文中采样底层文件系统的容量与 Inode 统计信息。
    pub fn statfs(&self) -> Result<FilesystemStat, FsError> {
        match self {
            Self::Ext4(fs) => fs.statfs(),
            Self::Memory(fs) => fs.statfs(),
        }
    }

    /// 返回文件系统后端自身的读写访问权限模式（如 `tmpfs` 为读写，`devfs`/`procfs` 为只读）。
    pub fn mode(&self) -> AccessMode {
        match self {
            Self::Ext4(fs) => fs.mode(),
            Self::Memory(fs) => fs.mode(),
        }
    }

    /// 将文件系统的脏数据与元数据同步到底层持久化存储（内存文件系统无落盘义务，直接返回 `Ok(())`）。
    pub fn sync(&self) -> Result<(), FsError> {
        match self {
            Self::Ext4(fs) => fs.sync(),
            Self::Memory(_) => Ok(()),
        }
    }

    /// 判断两个 [`Filesystem`] 引用是否指向同一个底层文件系统实例（通过比较内部 `Arc` 指针地址）。
    pub fn ptr_eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Ext4(a), Self::Ext4(b)) => Arc::ptr_eq(a, b),
            (Self::Memory(a), Self::Memory(b)) => Arc::ptr_eq(a, b),
            _ => false,
        }
    }

    /// 返回底层文件系统 `Arc` 对象的内存地址，仅用于跨挂载点操作时按固定地址顺序获取锁以避免死锁。
    pub(crate) fn identity_address(&self) -> usize {
        match self {
            Self::Ext4(fs) => Arc::as_ptr(fs) as usize,
            Self::Memory(fs) => Arc::as_ptr(fs) as usize,
        }
    }

    /// 获取或创建该文件系统实例共享的目录项缓存（[`DentryCache`]）与根 [`DentryRef`]。
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

    /// 获取该文件系统实例的 `rename` 睡眠互斥锁，以串行化目录树拓扑检查与多目录重命名提交。
    pub(crate) fn lock_rename(&self) -> SleepMutexGuard<'_, ()> {
        match self {
            Self::Ext4(fs) => fs.lock_rename(),
            Self::Memory(fs) => fs.rename.lock(),
        }
    }
}

/// 内存文件系统的具体后端类型。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemoryFsKind {
    /// 可读写的临时内存文件系统（`tmpfs`），支持物理页与 Inode 配额限制。
    Tmpfs,
    /// 设备节点文件系统（`devfs`），目录结构只读，由内核初始化固定设备节点。
    Devfs,
    /// 进程与系统状态伪文件系统（`procfs`），目录与动态快照文件均为只读。
    Procfs,
}

/// `devfs` 与 `procfs` 中特殊 Inode 节点的具体功能角色标识。
///
/// 特殊 Inode 本身不存储文件数据页，在 `open` 时根据此枚举分派到对应的设备或快照 [`crate::fs::FileLike`] 实现。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemorySpecial {
    /// `/dev/null` 空字符设备。
    Null,
    /// `/dev/zero` 零字符设备。
    Zero,
    /// `/dev/ttyS0` 串口终端字符设备。
    TtySerial,
    /// `/dev/console` 系统控制台字符设备。
    Console,
    /// `/dev/tty` 当前进程控制终端字符设备。
    ControllingTty,
    /// `/dev/kmsg` 内核日志流字符设备。
    Kmsg,
    /// `/dev/ktrace` 内核追踪流字符设备。
    Ktrace,
    /// `/dev/testhook` 评测控制字符设备（仅限 root 只写）。
    TestHook,
    /// `/proc/cmdline` 内核启动命令行参数文件。
    Cmdline,
    /// `/proc/mounts` 当前可见挂载表文件。
    Mounts,
    /// `/proc/meminfo` 物理内存统计信息文件。
    Meminfo,
    /// `/proc/uptime` 系统运行与空闲时间文件。
    Uptime,
    /// `/proc/interrupts` 各 CPU 中断计数统计文件。
    Interrupts,
    /// `/proc/net/dev` 网络接口收发统计文件。
    NetDev,
    /// `/proc/net/route` IPv4 路由表文件。
    NetRoute,
}

/// 内存文件系统实例（同时支撑 `tmpfs`、`devfs` 与 `procfs`）。
///
/// 根目录与各级权威目录项强持有存活的 `Arc<Inode>`，`Inode` 仅弱引用本结构；
/// 外部挂载点或打开文件路径持有 `Arc<MemoryFs>` 以维持整个文件系统实例的生命周期。
pub struct MemoryFs {
    /// 内存文件系统子类型（`Tmpfs`、`Devfs` 或 `Procfs`）。
    kind: MemoryFsKind,
    /// 本实例分配到的全局唯一设备号（`st_dev`）。
    device: u64,
    /// 文件系统根目录的权威 Inode（在实例构造时一次性初始化，编号固定为 1）。
    root: Once<Arc<Inode>>,
    /// 串行化跨目录 `rename`、初始特殊节点安装及名字缓存发布的睡眠互斥锁。
    rename: SleepMutex<()>,
    /// 串行化 Inode 分配、物理页额度预留与挂载配额重配置的睡眠互斥锁。
    allocation: SleepMutex<()>,
    /// 保护 Inode 弱引用槽位表、配额账本及名字缓存弱引用的内层自旋锁。
    index: SpinLock<MemoryIndex>,
}

/// `MemoryFs` 内部受自旋锁保护的索引与配额状态。
struct MemoryIndex {
    /// 已分配 Inode 的弱引用槽位数组；当某个 Inode 的强引用归零后，其槽位可被新分配的 Inode 复用。
    inodes: Vec<MemorySlot>,
    /// 下一个待分配的 Inode 编号（从 1 开始单调递增，永不回退复用旧编号）。
    next_number: u64,
    /// 当前已预留且尚未归还的数据物理页总数（单位：页）。
    pages: usize,
    /// 允许分配的最大数据物理页数上限（`0` 表示无上限）。
    max_pages: usize,
    /// 允许同时存活的最大 Inode 数量上限（`0` 表示无上限）。
    max_inodes: usize,
    /// 指向本文件系统关联的目录项缓存（[`DentryCache`]）的弱引用。
    namespace: Weak<DentryCache>,
    /// 指向本文件系统根目录项（[`Dentry`]）的弱引用。
    root_dentry: Weak<Dentry>,
    /// 标记名字缓存域是否已向 VFS 公开；一旦置为 `true`，禁止再调用 `initialize_special` / `initialize_directory` 注入内核节点。
    namespace_published: bool,
}

/// `MemoryIndex` 中的单个 Inode 登记槽位。
struct MemorySlot {
    /// 该槽位当前登记的 Inode 唯一编号（`st_ino`）。
    number: u64,
    /// 指向对应 [`Inode`] 的弱引用；当强引用计数降为 0 时表示该 Inode 已销毁，槽位可复用。
    inode: Weak<Inode>,
}

/// 单个 `tmpfs` 数据物理页（Folio）的配额预留凭证（RAII 守卫）。
///
/// 由 [`MemoryFs::reserve_page`] 在分配物理页前创建，并由对应的缓存页持有；
/// 该凭证不可复制，当物理页最终释放或分配失败回滚时，通过 `Drop` 自动扣减文件系统已用页计数。
pub(crate) struct MemoryPageCharge {
    /// 指向所属 [`MemoryFs`] 实例的弱引用，用于在释放时归还 1 页配额。
    filesystem: Weak<MemoryFs>,
}

impl Drop for MemoryPageCharge {
    /// 当物理页释放或未提交的申请回滚时，若所属 `MemoryFs` 仍然存活，则原子归还 1 页配额。
    fn drop(&mut self) {
        if let Some(fs) = self.filesystem.upgrade() {
            let mut index = fs.index.lock();
            assert!(0 < index.pages, "memory page charge lost");
            index.pages -= 1;
        }
    }
}

impl MemoryFs {
    /// 按默认配额策略创建指定类型的内存文件系统实例。
    ///
    /// `Tmpfs` 默认使用受管物理内存页数的一半作为最大页数和最大 Inode 数；
    /// `Devfs` 与 `Procfs` 由内核固定节点构成，不设配额上限（`0`）。
    pub fn try_new(kind: MemoryFsKind, root_mode: u16) -> Result<Arc<Self>, FsError> {
        let limits = if kind == MemoryFsKind::Tmpfs {
            TmpfsOptions::default().limits(managed_pages())
        } else {
            (0, 0)
        };
        Self::try_with_limits(kind, root_mode, limits)
    }

    /// 解析 `tmpfs` 挂载选项字节串并创建对应的 `tmpfs` 文件系统实例。
    ///
    /// 若挂载选项未指定根目录权限 `mode`，则默认使用 `01777`（`S_ISVTX | S_IRWXUGO`）。
    pub fn try_tmpfs(options: &[u8]) -> Result<Arc<Self>, FsError> {
        let pages = managed_pages();
        let options = TmpfsOptions::parse(options, pages)?;
        Self::try_with_limits(
            MemoryFsKind::Tmpfs,
            options.mode.unwrap_or(S_ISVTX | S_IRWXUGO),
            options.limits(pages),
        )
    }

    /// 使用指定的类型、根目录权限与 `(max_pages, max_inodes)` 配额上限构造 [`MemoryFs`] 实例，并分配编号为 1 的根目录 Inode。
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

    /// 返回本内存文件系统的具体子类型（`Tmpfs`、`Devfs` 或 `Procfs`）。
    pub fn kind(&self) -> MemoryFsKind {
        self.kind
    }

    /// 返回本内存文件系统实例终身唯一的非零设备号（`st_dev`）。
    pub fn device(&self) -> u64 {
        self.device
    }

    /// 返回文件系统目录结构与普通文件的读写访问模式（仅 `Tmpfs` 为可读写，`Devfs` 和 `Procfs` 为只读）。
    pub fn mode(&self) -> AccessMode {
        match self.kind {
            MemoryFsKind::Tmpfs => AccessMode::ReadWrite,
            MemoryFsKind::Devfs | MemoryFsKind::Procfs => AccessMode::ReadOnly,
        }
    }

    /// 获取当前内存文件系统的容量与 Inode 统计快照。
    ///
    /// 对于 `Tmpfs`，在 `index` 自旋锁内读取配额上限、当前已预留页数以及存活 Inode 数量；
    /// 对于不设配额的 `Devfs`、`Procfs` 或无限额度的 `Tmpfs` 项，对应总块数/空闲块数字段保持为 `0`。
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

    /// 在挂载重配置（remount）时动态调整 `tmpfs` 的物理页配额与 Inode 配额。
    ///
    /// - 与 `allocate` 及 `reserve_page` 通过 `allocation` 睡眠锁串行化；
    /// - 若新指定的非零上限低于当前已使用的页数或存活 Inode 数，或者原上限为无限（`0`）而尝试改为有限非零值，则返回 [`FsError::InvalidOperation`]；
    /// - `mode` 选项仅在初次创建时生效，重配置不会修改现有根目录的权限。
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

    /// 获取指向本内存文件系统根目录的 [`InodeRef`]（由外部引用持有 `Arc<MemoryFs>` 以保持文件系统存活）。
    pub fn root(self: &Arc<Self>) -> InodeRef {
        InodeRef::from_memory(
            self.root.get().expect("memory root absent").clone(),
            self.clone(),
        )
    }

    /// 在尚未公开名字缓存的 `devfs` 或 `procfs` 父目录下创建并挂载一个特殊节点（字符设备或动态快照节点）。
    ///
    /// 仅限内核在构建 `devfs` / `procfs` 阶段调用；一旦文件系统通过 `dentry_namespace` 公开，后续调用将返回 [`FsError::InvalidOperation`]。
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

    /// 在尚未公开名字缓存的 `devfs` 或 `procfs` 父目录下创建并挂载一个子目录（如 `/proc/net`）。
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

    /// 在持有 `rename` 锁的前提下，校验当前实例是否处于允许内核初始化固定节点的阶段，并确认 `parent` 是属于本实例的目录。
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

    /// 为本内存文件系统分配一个新的 [`Inode`] 并登记到弱引用索引表中。
    ///
    /// 遵循自旋锁内不执行堆内存分配与释放的原则：先在锁内检查配额与槽位容量，在锁外完成 `Vec` 预留与 `Inode` 构造，最后再次入锁提交槽位并在锁外释放被替换的旧缓冲区。
    pub(crate) fn allocate(
        self: &Arc<Self>,
        mut metadata: InodeMetadata,
        content: MemoryContent,
        rdev: u64,
    ) -> Result<InodeRef, FsError> {
        let _allocation = self.allocation.lock();
        // Step 1: 在 `index` 自旋锁内检查 Inode 配额、寻找可复用的空弱引用槽位并分配递增的 Inode 编号。
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

        // Step 2: 在自旋锁外执行可能失败的堆内存预留与 Inode 对象构造。
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

        // Step 3: 重新获取 `index` 自旋锁提交新槽位，并将被替换的旧槽位或旧数组移出到锁外释放。
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

    /// 根据 Inode 编号在弱引用索引表中查找仍存活的 [`InodeRef`]；若未找到或强引用已归零则返回 [`FsError::NotFound`]。
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

    /// 在分配 `tmpfs` 数据物理页前预留 1 页配额，成功则返回绑定该页生命周期的 [`MemoryPageCharge`]。
    ///
    /// 若当前已用页数已达到 `max_pages` 上限，则返回 [`FsError::NoSpace`]（对应 `ENOSPC`）。
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

    /// 返回当前已预留且尚未归还的数据物理页总数（仅供内核测试验证配额回收）。
    #[cfg(feature = "ktest")]
    pub fn used_pages(&self) -> usize {
        self.index.lock().pages
    }

    /// 获取或初始化本内存文件系统的目录项缓存（[`DentryCache`]）与根 [`DentryRef`]。
    ///
    /// `MemoryIndex` 仅保存对 `DentryCache` 和根 `Dentry` 的弱引用，清理目录项缓存不会丢失保存在 `MemoryDirectory` 中的权威目录内容；
    /// 调用本函数会将 `namespace_published` 置为 `true`，锁定 `devfs`/`procfs` 的初始节点集合。
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
    /// 统计当前索引表中强引用计数大于 0（即仍被权威目录项、打开文件或内存映射持有）的存活 Inode 数量。
    fn used_inodes(&self) -> usize {
        self.inodes
            .iter()
            .filter(|slot| 0 < slot.inode.strong_count())
            .count()
    }
}

/// 计算系统受管物理内存总页数（总存在页数减去内核保留页与页帧元数据占用页）。
fn managed_pages() -> usize {
    let stats = crate::mm::pages().stats();
    stats.present_pages - stats.reserved_pages - stats.metadata_pages
}
