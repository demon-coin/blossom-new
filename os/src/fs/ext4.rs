//! ext4 卷保存实际 FS 权限和共同元数据页，块映射由 canonical inode 的 data-sem 保护。

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
type SegmentHook = Arc<dyn Fn(u64, usize) + Send + Sync>;

#[derive(Debug)]
pub struct Geometry {
    /// 文件系统块的字节数，与设备 sector 大小独立。
    pub block_size: usize,
    pub inode_size: usize,
    pub descriptor_size: usize,
    pub blocks_count: u64,
    pub inodes_count: u64,
    pub features_compat: u32,
    pub features_incompat: u32,
    pub features_ro_compat: u32,
    pub first_data_block: u64,
    pub blocks_per_group: u64,
    pub inodes_per_group: u64,
    pub groups_count: u64,
    pub descriptor_blocks: u64,
    pub checksum_seed: u32,
    /// 超级块为 HTree 名字哈希保存的四个 seed 字。
    pub hash_seed: [u32; 4],
    /// ext4 名字哈希把输入字节解释为无符号值时为 true。
    pub hash_unsigned: bool,
    /// 新建 indexed 目录选择的超级块默认哈希版本。
    pub default_hash_version: u8,
    pub uuid: [u8; 16],
    pub reserved_gdt_blocks: u64,
    pub backup_groups: [u32; 2],
    pub log_groups_per_flex: u8,
    pub first_inode: u64,
}

impl Geometry {
    /// 判断块组是否保留超级块副本，主描述符始终位于第一块组。
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
        [3u64, 5, 7].into_iter().any(|base| {
            let mut value = group;
            while value % base == 0 {
                value /= base;
            }
            value == 1
        })
    }

    /// 拒绝越界、零地址和覆盖超级块或保留描述符的物理范围。
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

#[derive(Debug)]
pub struct InodeData {
    pub number: u64,
    pub kind: InodeKind,
    pub mode: u16,
    /// 字符或块节点的 Linux 设备号，普通 inode 为零。
    pub rdev: u64,
    pub size: u64,
    pub uid: u32,
    pub gid: u32,
    pub links: u32,
    /// 已分配空间按 512 字节计数，包括独立 EA 块。
    pub blocks: u64,
    pub atime: InodeTime,
    pub mtime: InodeTime,
    pub ctime: InodeTime,
    pub generation: u32,
    pub flags: u32,
    pub block: [u8; 60],
    pub checksum_seed: u32,
    pub fast_symlink: bool,
    pub file_acl: u64,
    pub extra_isize: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlockMapping {
    Mapped { physical: u64, len: u64 },
    Hole { len: u64 },
    Unwritten { physical: u64, len: u64 },
    Eof,
}

/// inode size、映射根或 links 提交所需的现有 inode 记录窗口。
pub(crate) struct PreparedInodeMapping {
    record: BufferRef,
    offset: usize,
    old: InodeMetadata,
    attributes: InodeMetadata,
    next: InodeMetadata,
    root: [u8; 60],
}

pub struct Ext4Volume {
    device: u32,
    disk: DiskRef,
    metadata: Arc<MetadataCache>,
    geometry: Geometry,
    claim: FsClaim,
    inode_domain: SpinLock<Weak<InodeDomain>>,
    writeback: Once<Arc<WritebackDomain>>,
    rename: SleepMutex<()>,
    allocation: SleepMutex<()>,
    groups: Vec<allocation::GroupLayout>,
    #[cfg(feature = "ktest")]
    segment_hook: SpinLock<Option<SegmentHook>>,
}

/// 虚拟卷编号只表示本内核的独立卷实例，耗尽后拒绝构造而不回绕复用。
static NEXT_DEVICE: AtomicU32 = AtomicU32::new(1);

impl Ext4Volume {
    /// 普通任务按实际 BlockView 取得唯一卷；活卷的不同权限请求明确拒绝。
    /// RO Mount 可直接使用同一个 RW 卷，卷模式不替代挂载视图权限。
    /// 同盘所有活视图共享同一页预算，不同卷保留独立 canonical inode 域。
    /// 卷反向只存 Weak，外部文件页保 inode，元数据页保 cache；两者保持原占用。
    /// 加载失败不发表卷弱引用，局部 claim 与候选按 RAII 退出，不取消外部旧拥有。
    /// 候选在锁外完成元数据与回写准备，弱索引短锁只负责双查和发表。
    pub fn open(
        view: BlockView,
        mode: AccessMode,
        metadata_capacity: usize,
    ) -> Result<Arc<Self>, FsError> {
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
        // 主超级块在 1024 字节处；读设备对齐的整页后仅解码其固定范围。
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

    /// 返回卷自己的 FS 能力，不推导任何 Path/Mount 的写许可。
    pub fn mode(&self) -> AccessMode {
        self.claim.mode()
    }

    /// 在本卷内串行 rename 的拓扑检查与目录提交。
    pub(crate) fn lock_rename(&self) -> SleepMutexGuard<'_, ()> {
        self.rename.lock()
    }

    /// 借用真实 FS 占用；授权的 Bio 仍通过唯一 Disk 请求核心提交。
    pub fn claim(&self) -> &FsClaim {
        &self.claim
    }

    /// 借用发表前预备的卷回写服务，异步票独立保本卷到实际工作尾。
    pub(crate) fn writeback_domain(&self) -> &Arc<WritebackDomain> {
        self.writeback
            .get()
            .expect("published filesystem has no writeback domain")
    }

    /// 同步入口所见的有限 inode 集合及共同元数据；不追逐后继新写，首错保留。
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

    /// 正式回写借用同 Disk 的唯一元数据索引，不建立第二个设备内容域。
    pub fn metadata_cache(&self) -> &Arc<MetadataCache> {
        &self.metadata
    }

    /// 同一活卷只允许一个 inode 缓存域，旧 inode/外部文件页保留域身份时拒绝冲突。
    /// 反向仅存 Weak；先按卷绑定锁到域索引的次序取得实际卷拥有，再开放并发表新绑定。
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

    /// 普通 Task 仅退休目标域的无槽绑定；旧 Weak 保留到锁外 WB 全尾结束，阻止提前 activate。
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

    /// 隐藏 EA 使用原活卷与 canonical 域取得，不创建第二个缓存或从裸地址构造 Arc。
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

    /// 原 inode 位已临时取用但新记录尚未初始化，分配者须在 allocator 锁外等待旧 canonical 退场。
    pub(crate) fn wait_inode_reuse(&self, number: u64) -> Result<(), FsError> {
        let domain = self.inode_domain.lock().upgrade();
        match domain {
            Some(domain) => domain.wait_inode_reuse(self, number),
            None => Ok(()),
        }
    }

    /// 返回准备时固定的 canonical 槽预算，本轮扫描不随新 Dirty 或待回收对象无限扩大。
    pub(crate) fn dirty_inode_limit(&self) -> usize {
        let domain = self.inode_domain.lock().upgrade();
        domain.map_or(0, |domain| domain.dirty_inode_limit())
    }

    /// 只取得本卷同一 canonical inode 的真实脏义务，弱域和最后 put 均在短锁外。
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

    /// 借用一次解码并验证的有效几何。
    pub fn geometry(&self) -> &Geometry {
        &self.geometry
    }

    /// 同一卷的所有 Arc 引用共享稳定非零编号，不泄漏地址或宿主设备号。
    pub fn device(&self) -> u64 {
        u64::from(self.device)
    }

    /// 持卷分配睡眠锁采样主超级块当前空闲计数，使用分配与归还共同修改的元数据页。
    /// 总量保留完整卷几何，不扣除独立推算的元数据开销；读取和内容借用失败原样返回。
    pub fn statfs(&self) -> Result<FilesystemStat, FsError> {
        let _allocation = self.allocation.lock();
        let g = &self.geometry;
        // 主超级块位于页零的固定窗口，普通 metadata_block 的数据块校验不接受此保留块。
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

    /// 借用保活且被实际 FS 占用保护的磁盘。
    pub fn disk(&self) -> &DiskRef {
        &self.disk
    }

    /// 借用本卷的真实范围，文件块号只在提交 IO 时换算为物理地址。
    pub fn view(&self) -> &BlockView {
        self.claim.view()
    }

    /// 设置仅验收镜像使用的真实分段收权观察点；回调在卷锁之外执行。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_set_segment_hook(&self, hook: Option<SegmentHook>) {
        let previous = core::mem::replace(&mut *self.segment_hook.lock(), hook);
        drop(previous);
    }

    /// 仅释放无人使用的静止元数据缓存引用。
    pub fn prune_unused(&self) -> usize {
        self.metadata.prune_unused()
    }

    /// 普通任务读取一个已验证物理地址的元数据块，等待不跨内容借用。
    pub(crate) fn metadata_block(&self, physical: u64) -> Result<BufferRef, FsError> {
        self.geometry.validate_blocks(physical, 1)?;
        Ok(self
            .metadata
            .get(physical, self.geometry.block_size, None)?)
    }

    /// 定位主描述符的共同窗口，取得内容之前不持任何元数据短锁。
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

    /// 布局地址在卷打开时验证一次；可变计数和位图仍消费共同 Buffer 当前字段。
    fn inode_table(&self, group: u64) -> Result<u64, FsError> {
        self.groups
            .get(group as usize)
            .map(|layout| layout.inode_table)
            .ok_or(FsError::Corrupt)
    }

    /// 定位并校验一个完整 inode，外部编号不能穿过组或设备边界。
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

    /// caller 保持 data-sem 读侧到真实查询结束，合法空洞与 EOF 不掩盖格式错误。
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

    /// caller 保持 invalidate 保护到真实 IO 全尾；块根仅准备 targets，不跨数据等待。
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
        // SAFETY: 每个小块已由真实 DMA 全尾收权或 CPU 零填充，EOF 尾也已初始化；
        // read 仍独占整页内容锁，设备及所有可变借用均已结束。
        unsafe {
            read.complete(IoResult {
                completed: PAGE_SIZE,
                error: None,
            });
        }
        Ok(())
    }

    /// caller 持块映射读保护；单组件名字查找只在元数据 guard 内消费字节。
    pub fn lookup(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        name: &[u8],
    ) -> Result<Option<u64>, FsError> {
        directory::lookup(self, inode, root, name)
    }

    /// caller 持目录读侧和块映射保护，按真实后端 cookie 取下一项。
    pub fn dir_next(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        cursor: &DirectoryCursor,
    ) -> Result<Option<DirectoryEntry>, FsError> {
        directory::next(self, inode, root, cursor)
    }

    /// 把用户 lseek cookie 转为后端游标，并清除旧的哈希碰撞续读位置。
    pub(crate) fn directory_cursor(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        cookie: u64,
    ) -> Result<DirectoryCursor, FsError> {
        directory::cursor_at(self, inode, root, cookie)
    }

    /// 返回当前目录格式定义的 SEEK_END cookie，不将 HTree 哈希当作字节大小。
    pub(crate) fn directory_end_cookie(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
    ) -> Result<u64, FsError> {
        directory::end_cookie(self, inode, root)
    }

    /// caller 持块映射读保护，按当前属性与块根读取符号链接原始字节。
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

    /// 核实当次挂载能力属于同一真实视图，并且卷和挂载都授予写许可。
    pub(crate) fn require_write(&self, claim: &FsClaim) -> Result<(), FsError> {
        if claim.mode() != AccessMode::ReadWrite || self.mode() != AccessMode::ReadWrite {
            return Err(FsError::ReadOnly);
        }
        if !claim.view().same_range(self.claim.view()) {
            return Err(FsError::InvalidOperation);
        }
        Ok(())
    }

    /// 在块树或目的内容发表前保留本 inode 的真实记录窗口。
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

    /// 初始化仍由 UnpublishedInode 保有的格式，内嵌链接撤销 extent 位后才能 canonical iget。
    pub(crate) fn initialize_unpublished_inode(
        &self,
        allocation: &UnpublishedInode,
        old: &InodeData,
        next: &InodeMetadata,
        root: &[u8; 60],
    ) -> Result<(), FsError> {
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

    /// iget 失败时清除仍未发表 inode 根，之后才能取消其块映射 reservation。
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

    /// caller 保持 inode 写序列化；准备不会连接新资源，失败有显式本次释放。
    pub(crate) fn prepare_write(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        logical: u64,
    ) -> Result<mapping::PreparedMapping, FsError> {
        mapping::prepare(self, &inode.with_block(root), logical)
    }

    /// 准备一次目录索引增长所需的整批逻辑块，不发表 inode 块根。
    pub(crate) fn prepare_directory_mappings(
        &self,
        inode: &InodeData,
        root: &[u8; 60],
        logical: &[u64],
    ) -> Result<mapping::PreparedDirectoryMappings, FsError> {
        mapping::prepare_directory_mappings(self, inode, root, logical)
    }

    /// 符号链接短块树预备复用唯一 extent/indirect 分配与提交算法。
    pub(crate) fn prepare_symlink_mappings(
        &self,
        inode: &InodeData,
        root: &[u8; 60],
        logical: &[u64],
    ) -> Result<mapping::PreparedDirectoryMappings, FsError> {
        mapping::prepare_symlink_mappings(self, inode, root, logical)
    }

    /// 预备目录项插入及其可能新增的真实叶块和索引块。
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

    /// 在父名发表之前准备新目录的真实 `.`、`..` 与 HTree 根。
    pub(crate) fn prepare_directory_new(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        parent_number: u64,
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_new_directory(self, inode, root, parent_number)
    }

    /// 预备移除指定目录名，旧目录内容只在该 token 的 publish 中改变。
    pub(crate) fn prepare_directory_remove(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        name: &[u8],
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_remove(self, inode, root, name)
    }

    /// 预备把现存目录项替换为新的 inode 身份。
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

    /// 预备同目录中的原生 rename，可在同一份树编辑里覆盖目标项。
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

    /// 预备移动目录时的真实 `..` 记录，不拆为独立目录写入。
    pub(crate) fn prepare_directory_dotdot(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        parent_number: u64,
    ) -> Result<PreparedDirectoryEdit, FsError> {
        directory::prepare_dotdot(self, inode, root, parent_number)
    }

    /// 校验目录是否只有 `.` 和 `..` 两个实际记录。
    pub(crate) fn directory_is_empty(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
    ) -> Result<bool, FsError> {
        directory::is_empty(self, inode, root)
    }

    /// 为目录内容和新块映射先预留 inode-table 窗口，不改变共同记录。
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

    /// 目录项与映射块已写入后无失败地提交 inode 记录；canonical 状态由持锁 guard 发布。
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

    /// canonical 写序列化下准备准确截断窗口；失败不修改旧树、共同元数据或位图。
    pub(crate) fn prepare_truncate(
        &self,
        inode: &InodeMetadata,
        root: &[u8; 60],
        new_size: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<mapping::PreparedTruncate, FsError> {
        mapping::prepare_truncate(self, &inode.with_block(root), new_size, context)
    }

    /// 先准备 inode-table 真实窗口，再在同一页锁内仅修改当前 inode 的字段和校验和。
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

/// 将映射准备失败的原始原因发表给同页等待者，不依赖析构取消语义。
pub(crate) fn finish_error(mut read: FolioRead, error: FsError) -> FsError {
    // SAFETY: 映射准备时页仍由 CPU 独占，此前分段 DMA 已结束；失败不授予字节读取权。
    unsafe {
        read.complete(IoResult {
            completed: 0,
            error: Some(error.folio_error()),
        });
    }
    error
}

/// CPU 收权阶段初始化原页的一段合法零内容，不发表中间可读状态。
fn zero_range(read: &mut FolioRead, offset: usize, len: usize) {
    for byte in &mut read.bytes_mut()[offset..offset + len] {
        byte.write(0);
    }
}
