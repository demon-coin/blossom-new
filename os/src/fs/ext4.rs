//! ext4 文件系统卷核心管理与 VFS 后端入口。
//!
//! # 模块职责与架构定位
//! 本模块代表一个已打开的 ext4 文件系统实例（[`Ext4Volume`]），作为 VFS 层（`InodeDomain`、`Mount`、
//! `writeback`）与底层块设备 / 元数据缓存（`BlockView`、`FsClaim`、`MetadataCache`）之间的核心枢纽：
//! - **上游调用者**：`fs::inode`（加载/创建/截断/回收 inode、读写文件页）、`fs::path` / `fs::dentry`（目录查找与修改、`rename` 串行化）、`fs::writeback`（脏 inode 与元数据同步）。
//! - **内部子模块**：`codec`（小端字节编解码与校验）、`crc`（CRC32C/CRC16 计算）、`allocation`（块与 inode 位图预留与释放）、`mapping`（Extent 树与传统间接块映射）、`directory`（线性目录与 HTree 索引目录操作）。
//!
//! # 核心设计与盘面/引用布局
//! 1. **ext4 块组盘面布局（以第 0 组及含超级块备份的块组为例）**：
//! ```text
//! +-----------------------+----------------------------------------------------+
//! | Boot Padding (1024 B) | 主超级块 Superblock (1024 B, 位于第 0 页偏移 1024) |
//! +-----------------------+-------------------+--------------------+-----------+
//! | 块组描述符表 (GDT, descriptor_blocks 块)  | 预留 GDT 扩展块    | 数据位图  |
//! +-----------------------+-------------------+--------------------+-----------+
//! | Inode 位图            | Inode 表 (inode_table) | 文件数据块 / Extent 节点  |
//! +-----------------------+------------------------+---------------------------+
//! ```
//! 开启 `flex_bg` 时，各块组的位图和 Inode 表可跨物理块组集中放置，卷打开时将校验后的固定起始块号缓存在 `Ext4Volume::groups` 中。
//!
//! 2. **所有权与弱引用防环结构**：
//! ```text
//! Mount / Inode / Writeback Ticket ──(Arc)──► Ext4Volume ──(Arc)──► MetadataCache
//!                                                 ▲                      │
//!                                    (Weak)       │                      │ (Weak: filesystem)
//!                                  InodeDomain ◄──┴──────────────────────┘
//! ```
//! `MetadataCache` 与 `Ext4Volume::inode_domain` 反向仅持有 `Weak` 引用，确保所有挂载点卸载、关联内存页与后台回写任务排空后，卷与块设备占用（`FsClaim`）可按 RAII 自然释放，不产生引用环。
//!
//! # 锁获取顺序与并发契约
//! 本模块及子模块严格遵守以下由外向内的锁层级顺序：
//! 1. `Ext4Volume::rename`（睡眠锁 `SleepMutex`）：串行化同卷跨目录 `rename` 拓扑检查与提交；
//! 2. VFS `Inode` 睡眠读写锁（目录操作锁、块映射 `data` 读写锁）；
//! 3. `Ext4Volume::allocation`（睡眠锁 `SleepMutex`）：串行化位图扫描、块组描述符与超级块空闲计数修改；
//! 4. `MetadataCache` 页获取与 `BufferRef` 内容读写锁（可睡眠，严禁在自旋锁临界区内等待）；
//! 5. 短临界区自旋锁（`Ext4Volume::inode_domain`、`MetadataCache::filesystem`、`InodeDomain::index`）：持锁期间不可睡眠；当同时涉及卷绑定与域索引时，固定先锁 `Ext4Volume::inode_domain`，再锁 `InodeDomain::index`。

mod allocation;
pub(crate) use allocation::PreparedInodeRelease;
pub use allocation::{UnpublishedBlock, UnpublishedInode};
pub(crate) mod codec;
pub(crate) mod crc;
mod directory;
pub(crate) use directory::PreparedDirectoryEdit;
mod mapping;
pub(crate) use mapping::PreparedDirectoryMappings;

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::cmp::min;
use core::convert::TryFrom;
use core::sync::atomic::{AtomicU32, Ordering};
use spin::Once;

use crate::block::{
    AccessMode, BlockError, BlockView, Disk, DiskRef, FsClaim, IoResult, MetadataIoContext,
    SECTOR_BYTES,
};
use crate::cache::{BufferRef, MetadataCache};
use crate::fs::background::WritebackDomain;
use crate::fs::inode::{InodeDomain, InodeMetadata, InodeRef};
use crate::fs::memfs::FilesystemStat;
use crate::fs::{writeback, DirectoryCursor, DirectoryEntry, FsError, InodeKind, InodeTime};
use crate::mm::{FolioRead, PAGE_SIZE};
use crate::sync::{SleepMutex, SleepMutexGuard, SpinLock};

#[cfg(feature = "ktest")]
/// 内核测试专用的文件页分段填充完成观测回调，参数依次为页索引与当前页内已完成字节偏移。
type SegmentHook = Arc<dyn Fn(u64, usize) + Send + Sync>;

/// 从主超级块解码并校验后的 ext4 卷静态几何参数与特性配置。
#[derive(Debug)]
pub struct Geometry {
    /// 文件系统逻辑块字节数（`1024`、`2048` 或 `4096`，单位：字节），独立于底层块设备扇区大小。
    pub block_size: usize,
    /// 单个盘面 inode 记录的字节大小（如传统的 `128` 字节或扩展的 `256` 字节，单位：字节）。
    pub inode_size: usize,
    /// 单个块组描述符（Group Descriptor）的字节大小（`32` 字节或启用 64-bit 时的 `64` 字节，单位：字节）。
    pub descriptor_size: usize,
    /// 全卷文件系统块总数（单位：文件系统块）。
    pub blocks_count: u64,
    /// 全卷盘面 inode 总数。
    pub inodes_count: u64,
    /// 兼容特性位掩码（`s_feature_compat`）。
    pub features_compat: u32,
    /// 不兼容特性位掩码（`s_feature_incompat`，含 `EXTENTS`、`64BIT`、`FILETYPE` 等）。
    pub features_incompat: u32,
    /// 只读兼容特性位掩码（`s_feature_ro_compat`，遇到未知位时仅允许只读挂载）。
    pub features_ro_compat: u32,
    /// 第 0 个块组的起始物理块号（1KiB 块大小时为 `1`，2KiB/4KiB 块大小时为 `0`）。
    pub first_data_block: u64,
    /// 每个标准块组包含的文件系统块数（末组可能少于此值）。
    pub blocks_per_group: u64,
    /// 每个块组包含的盘面 inode 槽位数。
    pub inodes_per_group: u64,
    /// 全卷块组总数。
    pub groups_count: u64,
    /// 主块组描述符表（GDT）占用的文件系统块数。
    pub descriptor_blocks: u64,
    /// 开启 `METADATA_CSUM` 时用于派生各元数据 CRC32C 校验和的基础种子（源自卷 UUID 或 `s_checksum_seed`）。
    pub checksum_seed: u32,
    /// 超级块 `s_hash_seed` 保存的 4 个 32 位种子字，供 HTree 目录项名字哈希计算使用。
    pub hash_seed: [u32; 4],
    /// 为 `true` 时，HTree 目录名字哈希将名字字节按无符号字符（unsigned char）参与运算。
    pub hash_unsigned: bool,
    /// 新建或升级为 HTree 索引目录时采用的超级块默认哈希算法版本号（`s_def_hash_version`）。
    pub default_hash_version: u8,
    /// 卷的 128 位唯一标识符（`s_uuid`）。
    pub uuid: [u8; 16],
    /// 为在线扩容预留的块组描述符表扩展块数（`s_reserved_gdt_blocks`，单位：文件系统块）。
    pub reserved_gdt_blocks: u64,
    /// 开启 `SPARSE_SUPER2` 特性时显式存放备份超级块的两个块组编号（`s_backup_bgs`）。
    pub backup_groups: [u32; 2],
    /// 第一个非保留普通 inode 编号（`s_first_ino`，通常为 `11`）。
    pub first_inode: u64,
}

impl Geometry {
    /// 判断块组 `group` 的起始位置是否包含超级块与块组描述符表副本（第 0 组始终包含主超级块与主 GDT）。
    fn has_super(&self, group: u64) -> bool {
        if group == 0 {
            return true;
        }
        if self.features_compat & codec::FEATURE_COMPAT_SPARSE_SUPER2 != 0 {
            return self
                .backup_groups
                .iter()
                .any(|&backup| u64::from(backup) == group);
        }
        if self.features_ro_compat & codec::FEATURE_RO_COMPAT_SPARSE_SUPER == 0 || group == 1 {
            return true;
        }
        [3u64, 5, 7].iter().any(|base| {
            let mut value = group;
            while value % base == 0 {
                value /= base;
            }
            value == 1
        })
    }

    /// 校验物理块区间 `[physical, physical + len)` 是否落在卷合法数据范围内，且未覆盖主/备份超级块或保留 GDT 区域。
    pub(crate) fn validate_blocks(&self, physical: u64, len: u64) -> Result<(), FsError> {
        let end = physical.checked_add(len).ok_or(FsError::Corrupt)?;
        if len == 0 || physical <= self.first_data_block || self.blocks_count < end {
            return Err(FsError::Corrupt);
        }

        let first_group = (physical - self.first_data_block) / self.blocks_per_group;
        let last_group = (end - 1 - self.first_data_block) / self.blocks_per_group;
        for group in first_group..=last_group {
            if self.has_super(group) {
                let start = self.first_data_block + group * self.blocks_per_group;
                let reserved_end = start
                    .checked_add(1)
                    .and_then(|v| v.checked_add(self.descriptor_blocks))
                    .and_then(|v| v.checked_add(self.reserved_gdt_blocks))
                    .ok_or(FsError::Corrupt)?;
                if physical < reserved_end && start < end {
                    return Err(FsError::Corrupt);
                }
            }
        }
        Ok(())
    }
}

/// 从盘面 inode 记录解码出的完整静态与动态属性快照。
#[derive(Debug)]
pub struct InodeData {
    /// 1-based ext4 inode 编号。
    pub number: u64,
    /// 由 `i_mode` 文件类型位解析出的 VFS 节点种类。
    pub kind: InodeKind,
    /// 包含文件类型位与访问权限位的完整 16 位 `i_mode`。
    pub mode: u16,
    /// 字符设备或块设备特殊节点的 Linux 设备号（`rdev`）；普通文件、目录和符号链接为 `0`。
    pub rdev: u64,
    /// 文件逻辑字节大小（`i_size`，单位：字节）。
    pub size: u64,
    /// 组合高低 16 位后的 32 位属主用户 ID（`uid`）。
    pub uid: u32,
    /// 组合高低 16 位后的 32 位属组 ID（`gid`）。
    pub gid: u32,
    /// 硬链接计数（`i_links_count`）。
    pub links: u32,
    /// 已分配物理空间折算的 512 字节扇区数（含外部扩展属性块）。
    pub blocks: u64,
    /// 最近访问时间戳（含纳秒扩展字段）。
    pub atime: InodeTime,
    /// 最近内容修改时间戳（含纳秒扩展字段）。
    pub mtime: InodeTime,
    /// 最近状态变更时间戳（含纳秒扩展字段）。
    pub ctime: InodeTime,
    /// inode 代数号（`i_generation`），每次复用空闲 inode 槽位时递增。
    pub generation: u32,
    /// ext4 inode 标志位掩码（`i_flags`，如 `EXTENTS`、`INDEX`、`HUGE_FILE`、`EA_INODE`）。
    pub flags: u32,
    /// 盘面 `i_block` 原始 60 字节区域（存放内联 Extent 树根、12+3 间接块指针或快速符号链接目标字节）。
    pub block: [u8; 60],
    /// 结合卷 `checksum_seed`、inode 编号与 `generation` 预计算的该 inode 专属 CRC32C 种子。
    pub checksum_seed: u32,
    /// 是否为目标路径直接内联存放在 `block` 60 字节区内的快速符号链接（无外部数据块）。
    pub fast_symlink: bool,
    /// 外部扩展属性块（xattr block）的物理块号；无外部块时为 `0`。
    pub file_acl: u64,
    /// 大 inode 记录中实际使用的扩展头字节数（`i_extra_isize`，单位：字节）。
    pub extra_isize: usize,
}

/// 文件单个连续逻辑块区间的物理块映射查询结果。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlockMapping {
    /// 已分配并初始化的连续物理块区间（`physical` 为起始全卷物理块号，`len` 为连续映射的文件系统块数量）。
    Mapped { physical: u64, len: u64 },
    /// 未分配物理存储的稀疏文件空洞区间，读取时按全零填充（`len` 为连续空洞覆盖的文件系统块数量）。
    Hole { len: u64 },
    /// 已预分配物理块但尚未写入有效数据的未初始化 Extent（`physical` 为起始全卷物理块号，`len` 为连续未初始化块数量；读取时按全零处理，写入前需转换为已初始化）。
    Unwritten { physical: u64, len: u64 },
    /// 查询的起始逻辑块号已达到或超出文件当前 `size` 对应的逻辑块上限。
    Eof,
}

/// 更新已有 inode 的大小、硬链接数或块映射根之前预留的盘面记录提交上下文。
pub(crate) struct PreparedInodeMapping {
    /// 目标 inode 记录所在的 inode 表块缓存窗口。
    record: BufferRef,
    /// 目标 inode 记录在 `record` 缓冲区内的起始字节偏移（单位：字节）。
    offset: usize,
    /// 提交前核对过的旧 inode 元数据快照。
    old: InodeMetadata,
    /// 仅应用了权限、属主、链接数和时间戳变更后的中间属性快照（供 `patch_inode_metadata` 使用）。
    attributes: InodeMetadata,
    /// 包含新 `size`、`blocks` 与 `flags` 的最终目标元数据快照（供 `patch_inode_mapping` 使用）。
    next: InodeMetadata,
    /// 待写入盘面 `i_block` 的 60 字节新映射根数据。
    root: [u8; 60],
}

/// 已打开的 ext4 文件系统卷实例，管理卷几何、块设备占用、元数据缓存与块组布局。
pub struct Ext4Volume {
    /// 内核分配的进程内唯一非零虚拟卷设备号（供 `stat` / `statfs` 区分不同卷实例）。
    device: u32,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "ktest 通过卷的原磁盘核对系统盘身份并控制真实 I/O 完成"
        )
    )]
    /// 底层块设备强引用（`ktest` 下用于核对设备身份与驱动测试 I/O）。
    disk: DiskRef,
    /// 绑定在当前块视图（`BlockView`）上的共享元数据块缓存。
    metadata: Arc<MetadataCache>,
    /// 卷打开时解码并验证通过的只读几何与特性配置。
    geometry: Geometry,
    /// 向底层 `Disk` 申请的块范围文件系统读写占用凭证，授权合法的数据与元数据 I/O。
    claim: FsClaim,
    /// 当前绑定到本卷的唯一 VFS `InodeDomain` 弱引用，由自旋锁保护绑定与解绑转换。
    inode_domain: SpinLock<Weak<InodeDomain>>,
    /// 本卷关联的后台异步回写调度域（卷发布前完成单次初始化）。
    writeback: Once<Arc<WritebackDomain>>,
    /// 串行化本卷内跨目录 `rename` 祖先环路检查与目录项提交的睡眠互斥锁。
    rename: SleepMutex<()>,
    /// 串行化本卷块位图、inode 位图及超级块/块组空闲计数更新的睡眠互斥锁。
    allocation: SleepMutex<()>,
    /// 按块组号索引的各组位图与 inode 表物理起始块布局表（卷打开时一次性加载）。
    groups: Vec<allocation::GroupLayout>,
    /// 内核测试专用的文件页分段读取观测钩子。
    #[cfg(feature = "ktest")]
    segment_hook: SpinLock<Option<SegmentHook>>,
}

/// 单调递增的虚拟卷设备号生成器；达到 `u32::MAX` 后拒绝继续分配，防止回绕导致两个活动卷设备号冲突。
static NEXT_DEVICE: AtomicU32 = AtomicU32::new(1);

impl Ext4Volume {
    /// 在块范围 `view` 上按指定访问模式 `mode` 打开或复用唯一的 `Ext4Volume` 实例。
    ///
    /// 必须在可睡眠的普通任务上下文调用：
    /// - 若该 `BlockView` 上已有存活的 `Ext4Volume` 且访问模式一致则直接复用，模式不同则返回 `FsError::Busy`（只读挂载可在上层直接共享同一个读写卷，由挂载点自身限制写操作）；
    /// - 所有可能睡眠的超级块解码、块组描述符表加载、元数据预留与回写域创建均在自旋锁外完成，最后通过 `metadata.filesystem` 自旋锁做双重检查并原子发布弱引用；中途失败时按 RAII 自动释放局部占用。
    pub fn open(
        view: BlockView,
        mode: AccessMode,
        metadata_capacity: usize,
    ) -> Result<Arc<Self>, FsError> {
        // Step 1: 申请块设备范围的文件系统占用与共享元数据缓存，若已有存活卷则核对访问模式后直接复用。
        let claim = Disk::claim_fs(&view, mode)?;
        let metadata = MetadataCache::shared(&claim, metadata_capacity)?;
        let existing = metadata.filesystem.lock().upgrade();
        if let Some(volume) = existing {
            if volume.mode() != mode {
                return Err(FsError::Busy);
            }
            return Ok(volume);
        }

        let capacity = view
            .limits()
            .capacity_sectors
            .checked_mul(SECTOR_BYTES)
            .ok_or(FsError::Corrupt)?;
        // Step 2: 读取第 0 页偏移 1024 字节处的主超级块，校验卷几何、块对齐与只读兼容特性集。
        let superblock = metadata.get(0, PAGE_SIZE, None)?;
        let geometry = superblock.with_read(|data| {
            codec::decode_super(
                codec::bytes(data, codec::SUPERBLOCK_OFFSET, codec::SUPERBLOCK_BYTES)?,
                capacity,
            )
        })??;
        drop(superblock);
        let logical = view.limits().logical_block_size as usize;
        if geometry.block_size % logical != 0 {
            return Err(FsError::Unsupported);
        }

        // 未识别的 RO-compatible 格式可以读取，不能授予当前后端写能力。
        let supported_ro = codec::FEATURE_RO_COMPAT_SPARSE_SUPER
            | codec::FEATURE_RO_COMPAT_LARGE_FILE
            | codec::FEATURE_RO_COMPAT_HUGE_FILE
            | codec::FEATURE_RO_COMPAT_GDT_CSUM
            | codec::FEATURE_RO_COMPAT_DIR_NLINK
            | codec::FEATURE_RO_COMPAT_EXTRA_ISIZE
            | codec::METADATA_CSUM;
        if mode == AccessMode::ReadWrite && geometry.features_ro_compat & !supported_ro != 0 {
            return Err(FsError::Unsupported);
        }

        // Step 3: 分配虚拟设备号并遍历全部块组描述符，校验并缓存每组的位图与 inode 表起始块地址。
        let mut volume = Self {
            device: NEXT_DEVICE
                .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |next| {
                    next.checked_add(1)
                })
                .map_err(|_| FsError::NoMemory)?,
            disk: view.disk().clone(),
            metadata: metadata.clone(),
            geometry,
            claim,
            inode_domain: SpinLock::new(Weak::new()),
            writeback: Once::new(),
            rename: SleepMutex::new(()),
            allocation: SleepMutex::new(()),
            groups: Vec::new(),
            #[cfg(feature = "ktest")]
            segment_hook: SpinLock::new(None),
        };
        volume
            .groups
            .try_reserve_exact(volume.geometry.groups_count as usize)
            .map_err(|_| FsError::NoMemory)?;
        for group in 0..volume.geometry.groups_count {
            let (buffer, offset) = volume.group_buffer(group, None)?;
            let layout = buffer.with_read(|data| {
                codec::group_layout(
                    &volume.geometry,
                    group,
                    codec::bytes(data, offset, volume.geometry.descriptor_size)?,
                )
            })??;
            volume.groups.push(layout);
        }

        // Step 4: 为读写卷预留元数据回写 Bio 资源，初始化后台回写域，并在自旋锁内完成双重检查后发布卷弱引用。
        if mode == AccessMode::ReadWrite {
            metadata.prepare_reserve()?;
        }
        let volume = Arc::try_new(volume).map_err(|_| FsError::NoMemory)?;
        let writeback = WritebackDomain::try_new(&volume)?;
        volume.writeback.call_once(|| writeback);

        let mut published = metadata.filesystem.lock();
        if let Some(existing) = published.upgrade() {
            drop(published);
            if existing.mode() != mode {
                return Err(FsError::Busy);
            }
            return Ok(existing);
        }
        let retired = core::mem::replace(&mut *published, Arc::downgrade(&volume));
        drop(published);
        drop(retired);
        Ok(volume)
    }

    /// 返回本卷自身打开时的读写访问能力（挂载视图的只读限制由上层 `Mount` 独立维护）。
    pub fn mode(&self) -> AccessMode {
        self.claim.mode()
    }

    /// 获取本卷的 `rename` 睡眠互斥锁，用于串行化跨目录重命名的祖先环路检查与目录项提交。
    pub(crate) fn lock_rename(&self) -> SleepMutexGuard<'_, ()> {
        self.rename.lock()
    }

    /// 借用本卷在底层块设备上持有的文件系统占用凭证。
    pub fn claim(&self) -> &FsClaim {
        &self.claim
    }

    /// 借用本卷关联的后台异步回写调度域。
    pub(crate) fn writeback_domain(&self) -> &Arc<WritebackDomain> {
        self.writeback
            .get()
            .expect("published filesystem has no writeback domain")
    }

    /// 同步本卷当前缓存槽位内的脏 inode 及其共享元数据块（保留首个遇到的错误，遇到设备停机未确认错误时立即中止返回）。
    pub(crate) fn sync(&self) -> Result<(), FsError> {
        if self.mode() == AccessMode::ReadOnly {
            return Ok(());
        }
        let mut result = Ok(());
        let limit = self.dirty_inode_limit();
        for position in 0..limit {
            if let Some(inode) = self.dirty_inode_at(position) {
                let current = writeback::sync_inode(&inode);
                if current == Err(FsError::Io(BlockError::StopUnconfirmed)) {
                    return current;
                }
                result = result.and(current);
            }
        }

        result.and(writeback::sync_metadata(self))
    }

    /// 借用本卷关联的共享元数据块缓存。
    pub fn metadata_cache(&self) -> &Arc<MetadataCache> {
        &self.metadata
    }

    /// 将本卷绑定到唯一的 VFS `InodeDomain` 缓存域并激活后台回写调度。
    ///
    /// 同一时间一个存活卷只允许绑定一个 `InodeDomain`（重复绑定同一域直接成功，绑定不同域返回 `FsError::Busy`）；
    /// 加锁顺序严格遵循先 `self.inode_domain` 自旋锁、再 `candidate` 内部索引锁。
    pub(crate) fn bind_inode_domain(
        self: &Arc<Self>,
        candidate: &Arc<InodeDomain>,
    ) -> Result<(), FsError> {
        let retired = {
            let mut domain = self.inode_domain.lock();
            if let Some(previous) = domain.upgrade() {
                let same = Arc::ptr_eq(&previous, candidate);
                drop(domain);
                drop(previous);
                return if same { Ok(()) } else { Err(FsError::Busy) };
            }
            candidate.bind_volume(self)?;
            self.writeback_domain().activate();
            core::mem::replace(&mut *domain, Arc::downgrade(candidate))
        };
        drop(retired);
        Ok(())
    }

    /// 当指定 `domain` 中已无任何属于本卷的缓存 inode 槽位时，停止本卷后台回写并在收尾完成后解除弱绑定。
    ///
    /// 必须在可睡眠的普通任务上下文调用；在 `stop_sync()` 等待后台回写工作彻底退出前保留旧 `Weak` 占位，防止并发挂载提前重新激活回写。
    pub(crate) fn retire_inode_domain(self: &Arc<Self>, domain: &Arc<InodeDomain>) {
        let expected = Arc::downgrade(domain);
        let volume = {
            let binding = self.inode_domain.lock();
            if !Weak::ptr_eq(&binding, &expected) {
                return;
            }
            domain.take_unused_volume(self)
        };
        let Some(volume) = volume else {
            return;
        };

        volume.writeback_domain().stop_sync();
        let retired = {
            let mut binding = self.inode_domain.lock();
            core::mem::replace(&mut *binding, Weak::new())
        };
        drop(retired);
        drop(volume);
    }

    /// 通过当前卷绑定的 `InodeDomain` 加载或获取指定编号的大扩展属性值专用 `EA_INODE`。
    pub(crate) fn iget_ea_inode(
        &self,
        number: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<InodeRef, FsError> {
        let domain = self.inode_domain.lock().upgrade().ok_or(FsError::Busy)?;
        let filesystem = self
            .metadata
            .filesystem
            .lock()
            .upgrade()
            .ok_or(FsError::InvalidOperation)?;
        if !core::ptr::eq(filesystem.as_ref(), self) {
            return Err(FsError::InvalidOperation);
        }
        domain.iget(&filesystem, number, None, context)
    }

    /// 当新分配的 inode 编号在 `InodeDomain` 中仍有正在异步淘汰的旧内存实例时，等待其彻底退场（调用方不得持有 `self.allocation` 锁）。
    pub(crate) fn wait_inode_reuse(&self, number: u64) -> Result<(), FsError> {
        let domain = self.inode_domain.lock().upgrade();
        match domain {
            Some(domain) => domain.wait_inode_reuse(self, number),
            None => Ok(()),
        }
    }

    /// 返回当前绑定 `InodeDomain` 的固定槽位上限，供单轮回写扫描作为有界上界使用。
    pub(crate) fn dirty_inode_limit(&self) -> usize {
        let domain = self.inode_domain.lock().upgrade();
        domain.map_or(0, |domain| domain.dirty_inode_limit())
    }

    /// 读取绑定 `InodeDomain` 中第 `position` 槽位的脏 inode（仅当该脏 inode 确实归属本卷时返回；引用升级与释放均在索引自旋锁外完成）。
    pub(crate) fn dirty_inode_at(&self, position: usize) -> Option<InodeRef> {
        let domain = self.inode_domain.lock().upgrade();
        domain
            .and_then(|domain| domain.dirty_inode_at(position))
            .filter(|inode| {
                core::ptr::eq(
                    inode
                        .disk_volume()
                        .expect("disk dirty inode has no volume")
                        .as_ref(),
                    self,
                )
            })
    }

    /// 借用本卷只读的几何与特性配置。
    pub fn geometry(&self) -> &Geometry {
        &self.geometry
    }

    /// 返回本卷在内核运行期稳定的非零虚拟设备号。
    pub fn device(&self) -> u64 {
        u64::from(self.device)
    }

    /// 在 `self.allocation` 锁保护下读取主超级块的实时空闲块与空闲 inode 计数，组装 `statfs` 文件系统统计信息。
    pub fn statfs(&self) -> Result<FilesystemStat, FsError> {
        let _allocation = self.allocation.lock();
        let g = &self.geometry;
        // 主超级块固定位于第 0 页偏移 1024 字节处，属于保留块范围，需直接通过 MetadataCache 读取第 0 页。
        let superblock = self.metadata.get(0, PAGE_SIZE, None)?;
        superblock.with_read(|bytes| {
            let record = codec::bytes(bytes, codec::SUPERBLOCK_OFFSET, codec::SUPERBLOCK_BYTES)?;
            let mut free = u64::from(codec::le32(record, 12)?);
            let mut reserved = u64::from(codec::le32(record, 8)?);
            if g.features_incompat & codec::FEATURE_INCOMPAT_64BIT != 0 {
                free |= u64::from(codec::le32(record, 344)?) << 32;
                reserved |= u64::from(codec::le32(record, 340)?) << 32;
            }
            let freefiles = u64::from(codec::le32(record, 16)?);
            if g.blocks_count < free || g.inodes_count < freefiles {
                return Err(FsError::Corrupt);
            }
            Ok(FilesystemStat {
                fs_type: u64::from(codec::SUPER_MAGIC),
                block_size: g.block_size as u64,
                blocks: g.blocks_count,
                free,
                available: free.saturating_sub(reserved),
                files: g.inodes_count,
                freefiles,
                device: self.device(),
            })
        })?
    }

    /// 借用本卷底层关联的块设备实例（仅 `ktest` 测试可用）。
    #[cfg(feature = "ktest")]
    pub fn disk(&self) -> &DiskRef {
        &self.disk
    }

    /// 借用本卷对应的块设备扇区范围视图（`BlockView`）。
    pub fn view(&self) -> &BlockView {
        self.claim.view()
    }

    /// 注册或清除内核测试专用的文件页分段读取完成观测回调（旧回调在自旋锁外释放）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_segment_hook(&self, hook: Option<SegmentHook>) {
        let previous = core::mem::replace(&mut *self.segment_hook.lock(), hook);
        drop(previous);
    }

    /// 释放本卷元数据缓存中当前无人引用的干净缓冲区，返回回收数量。
    pub fn prune_unused(&self) -> usize {
        self.metadata.prune_unused()
    }

    /// 校验物理块号位于合法数据区间后，从元数据缓存读取对应文件系统块。
    pub(crate) fn metadata_block(&self, physical: u64) -> Result<BufferRef, FsError> {
        self.geometry.validate_blocks(physical, 1)?;
        Ok(self
            .metadata
            .get(physical, self.geometry.block_size, None)?)
    }

    /// 计算块组 `group` 的描述符在主 GDT 中的物理块号与块内字节偏移，并加载对应的元数据缓存窗口。
    fn group_buffer(
        &self,
        group: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<(BufferRef, usize), FsError> {
        let g = &self.geometry;
        if g.groups_count <= group {
            return Err(FsError::Corrupt);
        }
        let byte = group * g.descriptor_size as u64;
        let block = g.first_data_block + 1 + byte / g.block_size as u64;
        Ok((
            self.metadata.get(block, g.block_size, context)?,
            (byte % g.block_size as u64) as usize,
        ))
    }

    /// 查询块组 `group` 的 inode 表起始物理块号（源自卷打开时已验证的块组布局表）。
    fn inode_table(&self, group: u64) -> Result<u64, FsError> {
        self.groups
            .get(group as usize)
            .map(|layout| layout.inode_table)
            .ok_or(FsError::Corrupt)
    }

    /// 从盘面 inode 表加载并解码编号为 `number` 的完整 inode 记录，同时校验其内联块映射根的合法性。
    pub(crate) fn load_inode(
        &self,
        number: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<InodeData, FsError> {
        let g = &self.geometry;
        if number == 0 || g.inodes_count < number {
            return Err(FsError::Corrupt);
        }
        let group = (number - 1) / g.inodes_per_group;
        let table = self.inode_table(group)?;
        let byte = ((number - 1) % g.inodes_per_group) * g.inode_size as u64;
        let physical = table
            .checked_add(byte / g.block_size as u64)
            .ok_or(FsError::Corrupt)?;
        g.validate_blocks(physical, 1)?;
        let buffer = self.metadata.get(physical, g.block_size, context)?;
        let inode = buffer.with_read(|data| {
            codec::decode_inode(
                g,
                number,
                codec::bytes(data, (byte % g.block_size as u64) as usize, g.inode_size)?,
            )
        })??;
        if !inode.fast_symlink
            && !matches!(
                inode.kind,
                InodeKind::Character
                    | InodeKind::Block
                    | InodeKind::Fifo
                    | InodeKind::Socket
                    | InodeKind::Other
            )
        {
            mapping::validate_root(g, &inode)?;
        }
        Ok(inode)
    }

    /// 查询文件逻辑块号 `logical` 对应的物理块映射、稀疏空洞、未初始化区间或 `Eof`（调用方须持有该 inode 的块映射读锁）。
    pub(crate) fn map_block(
        &self,
        metadata: &InodeMetadata,
        root: &[u8; 60],
        logical: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<BlockMapping, FsError> {
        if let Some(context) = context {
            context.check()?;
        }
        let inode = metadata.with_block(root);
        if inode.fast_symlink
            || matches!(
                inode.kind,
                InodeKind::Character
                    | InodeKind::Block
                    | InodeKind::Fifo
                    | InodeKind::Socket
                    | InodeKind::Other
            )
        {
            return Err(FsError::Unsupported);
        }
        let end = codec::ceil_div(inode.size, self.geometry.block_size as u64)?;
        if end <= logical {
            return Ok(BlockMapping::Eof);
        }
        mapping::map(self, &inode, logical, end, context)
    }

    /// 按预先解析好的每块磁盘物理字节偏移数组 `targets` 逐块填充整页 `FolioRead`，对稀疏空洞和文件末尾（EOF）之后的部分清零，完成后统一发布整页可读状态。
    ///
    /// 调用方须持有防止页面被并发截断失效的保护直至本函数返回；块映射锁在准备完 `targets` 后即可释放，无需跨越磁盘数据 I/O 等待。
    pub fn fill_page(
        &self,
        index: u64,
        file_size: u64,
        targets: &[Option<u64>; PAGE_SIZE / SECTOR_BYTES as usize],
        mut read: FolioRead,
    ) -> Result<(), FsError> {
        let start = match index.checked_mul(PAGE_SIZE as u64) {
            Some(start) if read.offset() == 0 && read.len() == PAGE_SIZE => start,
            _ => return Err(finish_error(read, FsError::Corrupt)),
        };
        let size = self.geometry.block_size;
        let mut offset = 0usize;
        // Step 1: 按文件系统块粒度遍历整页，已映射块走块设备 DMA 读入，空洞与超出 file_size 的尾部由 CPU 填零。
        while offset < PAGE_SIZE {
            let byte = match start.checked_add(offset as u64) {
                Some(byte) => byte,
                None => return Err(finish_error(read, FsError::Corrupt)),
            };
            match targets[offset / size] {
                Some(disk_byte) => {
                    read = self.claim.fill_folio_range(disk_byte, read, offset, size)?;
                    #[cfg(feature = "ktest")]
                    {
                        let hook = self.segment_hook.lock().clone();
                        if let Some(hook) = hook {
                            hook(index, offset + size);
                        }
                    }
                }
                None => zero_range(&mut read, offset, size),
            }
            if file_size < byte.saturating_add(size as u64) {
                let valid = min(file_size.saturating_sub(byte), size as u64) as usize;
                zero_range(&mut read, offset + valid, size - valid);
            }
            offset += size;
        }

        // SAFETY:
        // 1. 循环已按 `block_size` 覆盖 `[0, PAGE_SIZE)` 全部区间：有映射段已由 `fill_folio_range` 在底层 DMA 彻底完成并交还物理页访问权后返回，空洞段及 EOF 尾部已由 `zero_range` 写入 `0`；
        // 2. `read` 独占持有该页的 `CONTENT_LOCKED` 初始化锁，当前已无任何在途设备 DMA 或并发可变切片借用，满足发布全部 `PAGE_SIZE` 字节为已初始化状态的前提。
        unsafe {
            read.complete(IoResult {
                completed: PAGE_SIZE,
                error: None,
            });
        }
        Ok(())
    }

    /// 在目录中查找单组件名字 `name`，命中时返回目标子项的 1-based inode 编号（调用方须持有目录块映射读锁）。
    pub fn lookup(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        name: &[u8],
    ) -> Result<Option<u64>, FsError> {
        directory::lookup(self, inode, root, name)
    }

    /// 从目录游标 `cursor` 位置读取下一条有效目录项（调用方须持有目录读锁与块映射读锁）。
    pub fn dir_next(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        cursor: &DirectoryCursor,
    ) -> Result<Option<DirectoryEntry>, FsError> {
        directory::next(self, inode, root, cursor)
    }

    /// 将用户态 `lseek` 传入的目录位置 `cookie` 转换为内部目录读取游标，并重置哈希碰撞链的续读偏移。
    pub(crate) fn directory_cursor(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        cookie: u64,
    ) -> Result<DirectoryCursor, FsError> {
        directory::cursor_at(self, inode, root, cookie)
    }

    /// 返回当前目录格式（线性目录或 HTree 索引目录）对应的 `SEEK_END` 结束位置 cookie。
    pub(crate) fn directory_end_cookie(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
    ) -> Result<u64, FsError> {
        directory::end_cookie(self, inode, root)
    }

    /// 读取符号链接的目标路径字节序列（支持内联快速符号链接与外部数据块符号链接；调用方须持有块映射读锁）。
    pub fn read_link(&self, inode: &InodeMetadata, root: &[u8; 60]) -> Result<Vec<u8>, FsError> {
        if inode.kind != InodeKind::Symlink {
            return Err(FsError::InvalidOperation);
        }
        let len = usize::try_from(inode.size).map_err(|_| FsError::Corrupt)?;
        if len == 0 || PAGE_SIZE < len {
            return Err(FsError::Corrupt);
        }

        let mut result = Vec::new();
        result
            .try_reserve_exact(len)
            .map_err(|_| FsError::NoMemory)?;
        if inode.fast_symlink {
            result.extend_from_slice(codec::bytes(root, 0, len)?);
        } else {
            let mut offset = 0usize;
            while offset < len {
                let logical = offset / self.geometry.block_size;
                let physical = match self.map_block(inode, root, logical as u64, None)? {
                    BlockMapping::Mapped { physical, .. } => physical,
                    _ => return Err(FsError::Corrupt),
                };
                let buffer = self.metadata_block(physical)?;
                let count = min(len - offset, self.geometry.block_size);
                buffer.with_read(|data| result.extend_from_slice(&data[..count]))?;
                offset += count;
            }
        }

        if result.contains(&0) {
            return Err(FsError::Corrupt);
        }
        Ok(result)
    }

    /// 校验给定的挂载占用凭证 `claim` 与本卷对应同一块范围，且挂载点与卷本身均处于读写模式。
    pub(crate) fn require_write(&self, claim: &FsClaim) -> Result<(), FsError> {
        if claim.mode() != AccessMode::ReadWrite || self.mode() != AccessMode::ReadWrite {
            return Err(FsError::ReadOnly);
        }
        if !claim.view().same_range(self.claim.view()) {
            return Err(FsError::InvalidOperation);
        }
        Ok(())
    }

    /// 计算编号为 `number` 的 inode 在盘面 inode 表中的物理块号与块内偏移，并加载对应的元数据缓存窗口。
    pub(crate) fn inode_record(
        &self,
        number: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<(BufferRef, usize), FsError> {
        let g = &self.geometry;
        if number == 0 || g.inodes_count < number {
            return Err(FsError::Corrupt);
        }
        let table = self.inode_table((number - 1) / g.inodes_per_group)?;
        let byte = ((number - 1) % g.inodes_per_group) * g.inode_size as u64;
        let physical = table + byte / g.block_size as u64;
        g.validate_blocks(physical, 1)?;
        Ok((
            self.metadata.get(physical, g.block_size, context)?,
            (byte % g.block_size as u64) as usize,
        ))
    }

    /// 在新分配的 `UnpublishedInode` 正式放入 `InodeDomain` 之前，校验并写入其初始元数据属性与块映射根（例如为内联快速符号链接清除默认 `EXTENTS` 标志并写入路径字节）。
    pub(crate) fn initialize_unpublished_inode(
        &self,
        allocation: &UnpublishedInode,
        old: &InodeData,
        next: &InodeMetadata,
        root: &[u8; 60],
    ) -> Result<(), FsError> {
        // Step 1: 校验未发布凭证归属、初始链接数与文件类型边界，确认快速符号链接与布局标志位自洽。
        if !allocation.belongs_to(self)
            || allocation.number() != old.number
            || next.number != old.number
            || next.kind != old.kind
            || old.links != 0
            || next.links != 0
            || old.file_acl != 0
            || next.file_acl != 0
            || old.flags & codec::EA_INODE != 0
            || matches!(
                old.kind,
                InodeKind::Character
                    | InodeKind::Block
                    | InodeKind::Fifo
                    | InodeKind::Socket
                    | InodeKind::Other
            )
        {
            return Err(FsError::InvalidOperation);
        }
        self.require_write(&self.claim)?;
        let old_root = old.block;
        let old = InodeMetadata::from_data(old);
        let mut expected = old;
        expected.mode = next.mode;
        expected.uid = next.uid;
        expected.gid = next.gid;
        expected.atime = next.atime;
        expected.mtime = next.mtime;
        expected.ctime = next.ctime;
        expected.size = next.size;
        expected.blocks = next.blocks;
        expected.flags = next.flags;
        expected.fast_symlink = next.fast_symlink;
        let fast_symlink = next.kind == InodeKind::Symlink
            && next.blocks == 0
            && next.size < codec::INODE_BLOCK_BYTES as u64;
        let inode_layout_flags =
            codec::HUGE_FILE | codec::INDEX | if fast_symlink { codec::EXTENTS } else { 0 };
        if expected != *next
            || next.flags & !inode_layout_flags != old.flags & !inode_layout_flags
            || (fast_symlink && next.flags & codec::EXTENTS != 0)
        {
            return Err(FsError::InvalidOperation);
        }
        if next.fast_symlink != fast_symlink {
            return Err(FsError::InvalidOperation);
        }
        if (i64::MAX as u64) < next.size {
            return Err(FsError::FileTooLarge);
        }
        if next.kind == InodeKind::Symlink {
            if next.size == 0 {
                return Err(FsError::InvalidName);
            }
            if (PAGE_SIZE as u64) < next.size {
                return Err(FsError::NameTooLong);
            }
        }

        // Step 2: 校验元数据字段与扇区数编码范围，核对缓存窗口内的初始盘面记录未被篡改后原子写入属性与映射根。
        let mut attributes = old;
        attributes.mode = next.mode;
        attributes.uid = next.uid;
        attributes.gid = next.gid;
        attributes.atime = next.atime;
        attributes.mtime = next.mtime;
        attributes.ctime = next.ctime;
        codec::validate_inode_metadata_update(&old, &attributes)?;
        codec::blocks_encoding(&self.geometry, next.blocks)?;

        let (record, offset) = allocation.record();
        let current = record.with_read(|bytes| {
            codec::decode_inode(
                &self.geometry,
                old.number,
                &bytes[offset..offset + self.geometry.inode_size],
            )
        })??;
        if InodeMetadata::from_data(&current) != old || current.block != old_root {
            return Err(FsError::Corrupt);
        }

        record
            .update(&self.claim, |bytes| {
                let record = &mut bytes[offset..offset + self.geometry.inode_size];
                codec::patch_inode_metadata(&self.geometry, record, &old, &attributes)
                    .expect("prepared unpublished inode attributes");
                codec::patch_inode_mapping(&self.geometry, record, next, root);
            })
            .map_err(FsError::Io)?;
        Ok(())
    }

    /// 当新 inode 加入 `InodeDomain` 失败而需要回滚时，将其盘面记录重置回刚分配时的空白状态，以便安全归还关联的块映射预留。
    pub(crate) fn clear_unpublished_inode(
        &self,
        allocation: &UnpublishedInode,
        initial: &InodeData,
    ) {
        assert!(
            allocation.belongs_to(self),
            "unpublished inode volume changed"
        );
        assert_eq!(allocation.number(), initial.number);
        let metadata = InodeMetadata::from_data(initial);
        let (record, offset) = allocation.record();
        let g = &self.geometry;
        record
            .update(&self.claim, |bytes| {
                codec::patch_inode_mapping(
                    g,
                    &mut bytes[offset..offset + g.inode_size],
                    &metadata,
                    &initial.block,
                )
            })
            .expect("prepared unpublished inode reset");
    }

    /// 为文件逻辑块 `logical` 的写入操作准备物理块分配与映射树更新上下文（调用方须持有该 inode 的写序列化锁；返回时尚未向盘面提交新块根）。
    pub(crate) fn prepare_write(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        logical: u64,
    ) -> Result<mapping::PreparedMapping, FsError> {
        mapping::prepare(self, &inode.with_block(root), logical)
    }

    /// 为目录扩展批量准备一组逻辑块的物理分配与映射树更新，返回待提交凭证。
    pub(crate) fn prepare_directory_mappings(
        &self,
        inode: &InodeData,
        root: &[u8; 60],
        logical: &[u64],
    ) -> Result<mapping::PreparedDirectoryMappings, FsError> {
        mapping::prepare_directory_mappings(self, inode, root, logical)
    }

    /// 为非内联符号链接准备外部数据块的物理分配与映射树更新，复用目录块批量映射逻辑。
    pub(crate) fn prepare_symlink_mappings(
        &self,
        inode: &InodeData,
        root: &[u8; 60],
        logical: &[u64],
    ) -> Result<mapping::PreparedDirectoryMappings, FsError> {
        mapping::prepare_symlink_mappings(self, inode, root, logical)
    }

    /// 预备向目录插入新目录项 `(name, number, kind)` 所需的叶块编辑及潜在的叶块/HTree 索引分裂资源。
    pub(crate) fn prepare_directory_add(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        name: &[u8],
        number: u64,
        kind: InodeKind,
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_add(self, inode, root, name, number, kind)
    }

    /// 在新子目录正式挂入父目录之前，预备初始化其自身的 `.`、`..` 目录项及初始块结构。
    pub(crate) fn prepare_directory_new(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        parent_number: u64,
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_new_directory(self, inode, root, parent_number)
    }

    /// 预备从目录中删除名字为 `name` 的目录项（实际盘面字节仅在返回凭证 `publish` 时修改）。
    pub(crate) fn prepare_directory_remove(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        name: &[u8],
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_remove(self, inode, root, name)
    }

    /// 预备将目录中已存在的目录项 `name` 原地改指到新的目标 `(number, kind)`。
    pub(crate) fn prepare_directory_replace(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        name: &[u8],
        number: u64,
        kind: InodeKind,
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_replace(self, inode, root, name, number, kind)
    }

    /// 预备同一目录内的重命名编辑（移除 `old_name` 并写入或覆盖 `new_name`），在同一份目录修改事务中完成。
    #[expect(
        clippy::too_many_arguments,
        reason = "同目录移动须将旧根、两名字、源身份和可覆盖目标交给同一次目录编辑"
    )]
    pub(crate) fn prepare_directory_move(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        old_name: &[u8],
        new_name: &[u8],
        number: u64,
        kind: InodeKind,
        target: Option<(u64, InodeKind)>,
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_move(self, inode, root, old_name, new_name, number, kind, target)
    }

    /// 预备跨目录移动子目录时将其内部 `..` 目录项更新为新父目录编号 `parent_number` 的编辑上下文。
    pub(crate) fn prepare_directory_dotdot(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        parent_number: u64,
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_dotdot(self, inode, root, parent_number)
    }

    /// 检查目录中除 `.` 与 `..` 之外是否不包含任何其他有效目录项。
    pub(crate) fn directory_is_empty(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
    ) -> Result<bool, FsError> {
        directory::is_empty(self, inode, root)
    }

    /// 校验已有 inode 的属性与块映射变更合法性，并预先加载和核对其 inode 表缓存窗口，生成无失败提交所需的 [`PreparedInodeMapping`]。
    pub(crate) fn prepare_inode_mapping(
        &self,
        old: &InodeMetadata,
        old_root: &[u8; 60],
        next: &InodeMetadata,
        root: &[u8; 60],
    ) -> Result<PreparedInodeMapping, FsError> {
        self.require_write(&self.claim)?;
        let mut expected = *old;
        expected.mode = next.mode;
        expected.uid = next.uid;
        expected.gid = next.gid;
        expected.links = next.links;
        expected.size = next.size;
        expected.blocks = next.blocks;
        expected.atime = next.atime;
        expected.mtime = next.mtime;
        expected.ctime = next.ctime;
        expected.flags = next.flags;
        let inode_layout_flags = codec::HUGE_FILE | codec::INDEX;
        if expected != *next || next.flags & !inode_layout_flags != old.flags & !inode_layout_flags
        {
            return Err(FsError::InvalidOperation);
        }

        let mut attributes = *old;
        attributes.mode = next.mode;
        attributes.uid = next.uid;
        attributes.gid = next.gid;
        attributes.links = next.links;
        attributes.atime = next.atime;
        attributes.mtime = next.mtime;
        attributes.ctime = next.ctime;
        codec::validate_inode_metadata_update(old, &attributes)?;
        codec::blocks_encoding(&self.geometry, next.blocks)?;
        if (i64::MAX as u64) < next.size {
            return Err(FsError::FileTooLarge);
        }

        let (record, offset) = self.inode_record(old.number, None)?;
        let current = record.with_read(|bytes| {
            codec::decode_inode(
                &self.geometry,
                old.number,
                &bytes[offset..offset + self.geometry.inode_size],
            )
        })??;
        if InodeMetadata::from_data(&current) != *old || current.block != *old_root {
            return Err(FsError::Corrupt);
        }
        Ok(PreparedInodeMapping {
            record,
            offset,
            old: *old,
            attributes,
            next: *next,
            root: *root,
        })
    }

    /// 在目录内容或映射树节点块均已落入缓存后，将预先准备好的属性与 60 字节块映射根写入 inode 表记录并重算校验和。
    pub(crate) fn publish_inode_mapping(&self, prepared: PreparedInodeMapping) {
        let g = &self.geometry;
        prepared
            .record
            .update(&self.claim, |bytes| {
                let record = &mut bytes[prepared.offset..prepared.offset + g.inode_size];
                codec::patch_inode_metadata(g, record, &prepared.old, &prepared.attributes)
                    .expect("prepared directory inode attributes");
                codec::patch_inode_mapping(g, record, &prepared.next, &prepared.root);
            })
            .expect("prepared writable directory inode record");
    }

    /// 在 inode 写序列化保护下准备文件截断到 `new_size` 所需的块映射裁剪与待释放物理块窗口；准备失败不会修改原映射树或位图。
    pub(crate) fn prepare_truncate(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        new_size: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<mapping::PreparedTruncate, FsError> {
        mapping::prepare_truncate(self, &inode.with_block(root), new_size, context)
    }

    /// 在内核测试中校验并直接将 inode 元数据属性变更（权限、属主、链接数、时间戳）打补丁写回 inode 表缓存块。
    #[cfg(feature = "ktest")]
    pub(crate) fn update_inode_metadata(
        &self,
        claim: &FsClaim,
        old: &InodeMetadata,
        next: &InodeMetadata,
    ) -> Result<(), FsError> {
        codec::validate_inode_metadata_update(old, next)?;
        let g = &self.geometry;
        let group = (old.number - 1) / g.inodes_per_group;
        let table = self.inode_table(group)?;
        let byte = ((old.number - 1) % g.inodes_per_group) * g.inode_size as u64;
        let buffer = self.metadata_block(table + byte / g.block_size as u64)?;
        let offset = (byte % g.block_size as u64) as usize;
        buffer.update(claim, |bytes| {
            codec::patch_inode_metadata(g, &mut bytes[offset..offset + g.inode_size], old, next)
        })??;
        Ok(())
    }
}

/// 在文件页读取或块映射准备失败时，将错误码发布到 `FolioRead` 并唤醒同页等待者，返回原始 `FsError`。
pub(crate) fn finish_error(mut read: FolioRead, error: FsError) -> FsError {
    // SAFETY:
    // 1. 进入错误收尾路径时，此前任何已提交的分段块设备 DMA 均已彻底结束并交还物理页访问权，当前页面由 CPU 通过 `read` 独占持有 `CONTENT_LOCKED`；
    // 2. 传入 `completed: 0` 与 `Some(...)` 错误码仅将页面标记为读取失败并唤醒等待者，不会将未完成初始化的页内存对外暴露为可读。
    unsafe {
        read.complete(IoResult {
            completed: 0,
            error: Some(error.folio_error()),
        });
    }
    error
}

/// 在独占持有 `FolioRead` 初始化锁期间，将页内 `[offset, offset + len)` 字节区间填充为零。
fn zero_range(read: &mut FolioRead, offset: usize, len: usize) {
    for byte in &mut read.bytes_mut()[offset..offset + len] {
        byte.write(0);
    }
}
