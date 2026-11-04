//! 卷内分配序列保护共同位图与计数；本次未发表资源由显式释放归还。

use crate::block::MetadataIoContext;
use crate::cache::BufferRef;
use crate::fs::ext4::codec::{self, GroupState};
use crate::fs::ext4::{Ext4Volume, Geometry, InodeData, InodeMetadata};
use crate::fs::types::{S_IALLUGO, S_IFDIR, S_IFLNK, S_IFMT, S_IFREG, S_IRWXUGO, S_ISGID, S_IXGRP};
use crate::fs::{FsError, InodeKind, InodeRef};
use crate::mm::PAGE_SIZE;
use crate::time;
use alloc::vec::Vec;

#[derive(Clone, Copy)]
pub(crate) struct GroupLayout {
    pub block_bitmap: u64,
    pub inode_bitmap: u64,
    pub inode_table: u64,
}

/// 保留本次位图拥有权及已经准备好的释放窗口；析构不得执行睡眠释放。
struct Reservation {
    volume: u32,
    group: u64,
    bit: usize,
    inode: bool,
    directory: bool,
    bitmap: BufferRef,
    descriptor: BufferRef,
    offset: usize,
    superblock: BufferRef,
    active: bool,
}

/// 尚未连接到文件块树的单块责任，不允许 Clone 或从任意块号构造。
pub struct UnpublishedBlock {
    physical: u64,
    reservation: Reservation,
}

/// 已有数据或节点的预备释放窗口；准备时尚未摘树，取消只释放窗口引用。
pub(crate) struct DetachedBlock {
    reservation: Reservation,
    /// 数据只保已经存在的物理元数据别名，节点始终保实际读取的准确窗口。
    alias: Option<BufferRef>,
}

/// 尚未连接到名字的 inode 责任，位图、记录与释放窗口均由本次申请保留。
pub struct UnpublishedInode {
    number: u64,
    reservation: Reservation,
    record: BufferRef,
    offset: usize,
}

/// 最后一个未链接 inode 的具体释放窗口；尚未清位图，析构不执行等待或盘面修改。
pub(crate) struct PreparedInodeRelease {
    reservation: Reservation,
    record: BufferRef,
    offset: usize,
    seed: u32,
    extra_isize: usize,
    acl: Option<(u64, DetachedBlock)>,
    ea: Vec<PreparedEaRelease>,
}

/// 一组真实 EA value 引用与其 canonical 使用、原记录窗口，取消时不改变计数。
struct PreparedEaRelease {
    inode: InodeRef,
    record: BufferRef,
    offset: usize,
    count: u64,
    references: u64,
}

impl DetachedBlock {
    /// 新根和父字段发表后保留必须消费的释放责任，未发表准备仍可直接取消。
    pub(crate) fn arm_release(&mut self) {
        assert!(
            !self.reservation.active,
            "ext4 detached block already armed"
        );
        self.reservation.active = true;
    }
}

impl UnpublishedBlock {
    /// 仅借用本次申请的物理地址，借用不会转移归还责任。
    pub fn physical(&self) -> u64 {
        self.physical
    }

    /// 映射指针正式发表后结束本次临时责任，不能再沿未发表接口归还。
    pub(crate) fn publish(mut self) {
        self.reservation.active = false;
    }
}

impl UnpublishedInode {
    /// 返回本次尚未发表的 inode 号。
    pub fn number(&self) -> u64 {
        self.number
    }

    /// 确认临时 inode 位仍属于给定卷且尚未转移到 canonical inode。
    pub(crate) fn belongs_to(&self, volume: &Ext4Volume) -> bool {
        self.reservation.active && self.reservation.volume == volume.device
    }
    /// 借用本次分配预留的 inode-table 窗口，不能转移其 bitmap 责任。
    pub(crate) fn record(&self) -> (&BufferRef, usize) {
        (&self.record, self.offset)
    }

    /// 原 inode 已由 canonical 使用或名字保有后结束临时分配责任，位图仍保持已分配。
    pub(crate) fn publish(mut self) -> u64 {
        self.reservation.active = false;
        self.number
    }
}

impl Drop for Reservation {
    /// 未发表资源必须经过显式释放，析构只暴露丢失拥有权的协议错误。
    fn drop(&mut self) {
        assert!(!self.active, "unpublished ext4 reservation lost");
    }
}

impl Ext4Volume {
    /// 普通任务按组提示扫描真实位图；零空间与缓存、页、设备失败分别返回。
    pub fn allocate_block(&self, hint: u64) -> Result<UnpublishedBlock, FsError> {
        let reservation = self.reserve(hint, false, false)?;
        let physical = self.geometry.first_data_block
            + reservation.group * self.geometry.blocks_per_group
            + reservation.bit as u64;
        Ok(UnpublishedBlock {
            physical,
            reservation,
        })
    }

    /// 当前操作尚未发表块指针时显式归还，不执行任何已接受前缀的回滚。
    pub fn release_unpublished_block(&self, mut block: UnpublishedBlock) {
        self.release_reservation(&mut block.reservation);
    }

    /// 取得临时 inode 位后先等待旧 canonical 身份退出，再初始化新记录；失败只归还本次临时位。
    pub fn allocate_inode(
        &self,
        hint: u64,
        mode: u16,
        uid: u32,
        gid: u32,
    ) -> Result<UnpublishedInode, FsError> {
        let directory = mode & S_IFMT == S_IFDIR;
        if !matches!(mode & S_IFMT, S_IFDIR | S_IFREG | S_IFLNK) {
            return Err(FsError::InvalidOperation);
        }
        let mut reservation = self.reserve(hint, true, directory)?;
        let number =
            reservation.group * self.geometry.inodes_per_group + reservation.bit as u64 + 1;
        // 临时位由 Reservation 保有，等待旧 canonical 身份退出时不持卷分配锁。
        if let Err(error) = self.wait_inode_reuse(number) {
            self.release_reservation(&mut reservation);
            return Err(error);
        }
        let result = self.inode_record(number, None);
        let (buffer, offset) = match result {
            Ok(record) => record,
            Err(error) => {
                self.release_reservation(&mut reservation);
                return Err(error);
            }
        };
        let g = &self.geometry;
        buffer
            .update(&self.claim, |bytes| {
                let record = &mut bytes[offset..offset + g.inode_size];
                let generation = codec::le32(record, 100)
                    .expect("prepared inode record")
                    .wrapping_add(1);
                record.fill(0);
                codec::put16(record, 0, mode);
                codec::put16(record, 2, uid as u16);
                codec::put16(record, 120, (uid >> 16) as u16);
                codec::put16(record, 24, gid as u16);
                codec::put16(record, 122, (gid >> 16) as u16);
                codec::put32(record, 100, generation);
                let extra = if g.inode_size == codec::LARGE_INODE_BYTES {
                    codec::INODE_EXTRA_BYTES
                } else {
                    0
                };
                if extra != 0 {
                    codec::put16(record, 128, extra as u16);
                }
                if g.features_incompat & codec::FEATURE_INCOMPAT_EXTENTS != 0 {
                    codec::put32(record, 32, codec::EXTENTS);
                    codec::put16(record, 40, codec::EXTENT_MAGIC);
                    codec::put16(record, 44, codec::EXTENT_ROOT_ENTRIES as u16);
                }
                let seed = codec::crc32c(
                    codec::crc32c(g.checksum_seed, &(number as u32).to_le_bytes()),
                    &generation.to_le_bytes(),
                );
                codec::inode_checksum(g, record, seed, extra);
            })
            .expect("prepared writable inode record");
        Ok(UnpublishedInode {
            number,
            reservation,
            record: buffer,
            offset,
        })
    }

    /// 本次 inode 尚未连接名字时归还位图与目录计数，不对已有对象作清理。
    pub fn release_unpublished_inode(&self, mut inode: UnpublishedInode) {
        self.release_reservation(&mut inode.reservation);
    }

    /// 按父目录 setgid、kind mode 和当前主 group 准备新 inode owner。
    pub(crate) fn allocate_inode_for_parent(
        &self,
        parent: &InodeMetadata,
        kind: InodeKind,
        requested_mode: u16,
        umask: u16,
        uid: u32,
        gid: u32,
    ) -> Result<UnpublishedInode, FsError> {
        if parent.kind != InodeKind::Directory {
            return Err(FsError::NotDirectory);
        }
        let kind_mode = match kind {
            InodeKind::Regular => S_IFREG,
            InodeKind::Directory => S_IFDIR,
            InodeKind::Symlink => S_IFLNK,
            InodeKind::Character
            | InodeKind::Block
            | InodeKind::Fifo
            | InodeKind::Socket
            | InodeKind::Other => return Err(FsError::InvalidOperation),
        };
        let parent_setgid = parent.mode & S_ISGID != 0;
        let mut mode = kind_mode
            | if kind == InodeKind::Symlink {
                S_IRWXUGO
            } else {
                requested_mode & S_IALLUGO
            };
        if kind == InodeKind::Directory && parent_setgid {
            mode |= S_ISGID;
        } else if kind != InodeKind::Directory
            && parent_setgid
            && mode & (S_ISGID | S_IXGRP) == (S_ISGID | S_IXGRP)
            && gid != parent.gid
        {
            mode &= !S_ISGID;
        }
        if kind != InodeKind::Symlink {
            mode &= !(umask & S_IRWXUGO);
        }
        let gid = if parent_setgid { parent.gid } else { gid };
        let hint =
            parent.number.checked_sub(1).ok_or(FsError::Corrupt)? / self.geometry.inodes_per_group;
        self.allocate_inode(hint, mode, uid, gid)
    }

    /// 节点合并或截断在发表之前保留真实节点和释放窗口，尚不改变位图或空闲计数。
    pub(crate) fn prepare_detached_block(
        &self,
        physical: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<DetachedBlock, FsError> {
        self.geometry.validate_blocks(physical, 1)?;
        let node = self
            .metadata
            .get(physical, self.geometry.block_size, context)?;
        self.prepare_block_release(physical, Some(node), context)
    }

    /// 数据删除只查当前物理元数据别名，缺失时不申请缓存页或读入旧文件数据。
    pub(crate) fn prepare_detached_data_block(
        &self,
        physical: u64,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<DetachedBlock, FsError> {
        self.geometry.validate_blocks(physical, 1)?;
        let alias = self
            .metadata
            .cached_buffer(physical, self.geometry.block_size)?;
        self.prepare_block_release(physical, alias, context)
    }

    /// 具体物理块保留同卷的位图、描述符和超级块视图，原块仍由旧树拥有。
    fn prepare_block_release(
        &self,
        physical: u64,
        alias: Option<BufferRef>,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<DetachedBlock, FsError> {
        self.geometry.validate_blocks(physical, 1)?;
        let group = (physical - self.geometry.first_data_block) / self.geometry.blocks_per_group;
        let bit =
            ((physical - self.geometry.first_data_block) % self.geometry.blocks_per_group) as usize;
        Ok(DetachedBlock {
            reservation: self.prepare_release_reservation(group, bit, false, false, context)?,
            alias,
        })
    }

    /// 块根和 inode 记录均已去掉引用后归还实际物理空间，可能等待共同元数据内容锁。
    ///
    /// # Safety
    /// 调用者保持 inode 根写侧且已摘离此块；旧数据填充、WB 和设备 I/O 已真实结束。
    /// 旧物理别名的使用者已结束且不会再提交内容，准备之后不能新建旧块别名；
    /// 元数据窗口只能由本次释放保留到 forget，不能把软件取消当作设备收权。
    pub(crate) unsafe fn release_detached_block(&self, mut block: DetachedBlock) {
        assert!(
            block.reservation.active,
            "ext4 block still belongs to its old tree"
        );
        if let Some(alias) = &block.alias {
            // SAFETY: 调用者已摘树、结束旧数据和别名使用并排除新旧窗口生产者；
            // alias 保准确物理范围，forget 等本页真实 WB 收权后只失效该窗口，位图尚未归还。
            unsafe { alias.forget(&self.claim) }.expect("detached physical block invalidation");
        }
        self.release_reservation(&mut block.reservation);
    }

    /// 最后使用交接后、修改 EOF 前准备 inode、外部 EA 和隐藏 value 的全部释放视图。
    /// 调用者保持 inode 写序列化和本卷元数据 WB 序列到消费结束；错误不改变原引用。
    pub(crate) fn prepare_inode_release(
        &self,
        inode: &InodeData,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<PreparedInodeRelease, FsError> {
        self.require_write(&self.claim)?;
        let g = &self.geometry;
        if inode.number < g.first_inode || g.inodes_count < inode.number || inode.links != 0 {
            return Err(FsError::InvalidOperation);
        }
        let (record, offset) = self.inode_record(inode.number, context)?;
        let mut numbers = [0u64; codec::MAX_EA_INODES];
        let count = record.with_read(|bytes| {
            let bytes = &bytes[offset..offset + g.inode_size];
            let current = codec::decode_inode(g, inode.number, bytes)?;
            if current.generation != inode.generation
                || current.kind != inode.kind
                || current.links != 0
                || current.file_acl != inode.file_acl
            {
                return Err(FsError::Corrupt);
            }
            codec::inline_ea_inodes(g, bytes, inode.extra_isize, &mut numbers)
        })??;
        let mut ea = Vec::new();
        self.prepare_ea_references(inode.number, &numbers[..count], &mut ea, context)?;
        let group = (inode.number - 1) / g.inodes_per_group;
        let bit = ((inode.number - 1) % g.inodes_per_group) as usize;
        let reservation = self.prepare_release_reservation(
            group,
            bit,
            true,
            inode.kind == InodeKind::Directory,
            context,
        )?;
        let acl = if inode.file_acl == 0 {
            None
        } else {
            let block = self.prepare_detached_block(inode.file_acl, context)?;
            let (references, count) = block
                .alias
                .as_ref()
                .expect("external EA buffer")
                .with_read(|bytes| {
                    let count =
                        codec::verify_xattr_release(g, inode.file_acl, bytes, &mut numbers)?;
                    Ok::<_, FsError>((codec::le32(bytes, 4)?, count))
                })??;
            if references == 1 {
                self.prepare_ea_references(inode.number, &numbers[..count], &mut ea, context)?;
            }
            Some((inode.file_acl, block))
        };
        for put in &ea {
            if put.references < put.count {
                return Err(FsError::Corrupt);
            }
        }
        Ok(PreparedInodeRelease {
            reservation,
            record,
            offset,
            seed: inode.checksum_seed,
            extra_isize: inode.extra_isize,
            acl,
            ea,
        })
    }

    /// 内嵌 EA 和最后外部 EA 块的逐项引用合并，所有窗口和 canonical 使用在发表前取得。
    fn prepare_ea_references(
        &self,
        parent: u64,
        numbers: &[u64],
        puts: &mut Vec<PreparedEaRelease>,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<(), FsError> {
        for &number in numbers {
            if number == parent {
                return Err(FsError::Corrupt);
            }
            if let Some(put) = puts.iter_mut().find(|put| put.inode.number() == number) {
                put.count += 1;
                continue;
            }
            puts.try_reserve(1).map_err(|_| FsError::NoMemory)?;
            let (record, offset) = self.inode_record(number, context)?;
            let references = record.with_read(|bytes| {
                let bytes = &bytes[offset..offset + self.geometry.inode_size];
                let data = codec::decode_inode(&self.geometry, number, bytes)?;
                let references = codec::ea_inode_references(bytes)?;
                if data.kind != InodeKind::Regular
                    || data.flags & codec::EA_INODE == 0
                    || data.links != 1
                    || data.file_acl != 0
                    || references == 0
                    || (i64::MAX as u64) < references
                {
                    return Err(FsError::Corrupt);
                }
                Ok(references)
            })??;
            let inode = self.iget_ea_inode(number, context)?;
            puts.push(PreparedEaRelease {
                inode,
                record,
                offset,
                count: 1,
                references,
            });
        }
        Ok(())
    }

    /// 消费最终回收窗口，清本 inode 记录及 EA 引用后归还 inode 位图和目录计数。
    /// 不申请内存或读盘；共享 EA 的当前引用计数由原卷分配锁串行化。
    ///
    /// # Safety
    /// 调用者已在 canonical 索引中禁止父 inode 新使用，nlink 为零且文件数据与树节点全部摘离。
    /// 父文件页、填充、WB、设备 I/O 和旧别名已结束，原根写侧已排空并释放；
    /// 持本卷元数据 WB 序列到返回，不能由 IRQ、任意 Drop 或索引短锁调用本睡眠操作。
    pub(crate) unsafe fn release_inode(&self, mut prepared: PreparedInodeRelease) {
        assert_eq!(prepared.reservation.volume, self.device);
        prepared.reservation.active = true;
        let g = &self.geometry;
        prepared
            .record
            .update(&self.claim, |bytes| {
                codec::patch_inode_release(
                    g,
                    &mut bytes[prepared.offset..prepared.offset + g.inode_size],
                    prepared.seed,
                    prepared.extra_isize,
                )
            })
            .expect("prepared freed inode record");
        if let Some((physical, mut block)) = prepared.acl.take() {
            let shared = {
                let _allocation = self.allocation.lock();
                let alias = block.alias.as_ref().expect("retained external EA buffer");
                let references = alias
                    .with_read(|bytes| codec::le32(bytes, 4))
                    .expect("retained external EA view")
                    .expect("retained external EA header");
                assert!(references != 0, "external EA already released");
                if 1 < references {
                    alias
                        .update(&self.claim, |bytes| {
                            codec::patch_xattr_refcount(g, physical, bytes, references - 1)
                        })
                        .expect("retained writable external EA");
                    true
                } else {
                    false
                }
            };
            if !shared {
                block.arm_release();
                // SAFETY: 最后 EA 引用已从本记录清除，卷分配锁下确认没有其他 inode 引用；
                // 调用者已结束旧别名及元数据 WB，block 保原窗口到失效，随后才允许复用。
                unsafe { self.release_detached_block(block) };
            }
        }
        for put in &prepared.ea {
            put.inode
                .release_ea_references(&self.claim, &put.record, put.offset, put.count);
        }
        self.release_reservation(&mut prepared.reservation);
    }

    /// 只准备已分配资源的归还窗口，active 保持关闭，取消不会归还仍可寻址的位。
    fn prepare_release_reservation(
        &self,
        group: u64,
        bit: usize,
        inode: bool,
        directory: bool,
        context: Option<MetadataIoContext<'_>>,
    ) -> Result<Reservation, FsError> {
        self.require_write(&self.claim)?;
        let (descriptor, offset) = self.group_buffer(group, context)?;
        let layout = self.groups[group as usize];
        let physical = if inode {
            layout.inode_bitmap
        } else {
            layout.block_bitmap
        };
        self.geometry.validate_blocks(physical, 1)?;
        let bitmap = self
            .metadata
            .get(physical, self.geometry.block_size, context)?;
        let superblock = self.metadata.get(0, PAGE_SIZE, context)?;
        Ok(Reservation {
            volume: self.device,
            group,
            bit,
            inode,
            directory,
            bitmap,
            descriptor,
            offset,
            superblock,
            active: false,
        })
    }

    /// 先取得所有可能睡眠的窗口；内容借用内只扫描、置位与计算 CRC。
    fn reserve(&self, hint: u64, inode: bool, directory: bool) -> Result<Reservation, FsError> {
        self.require_write(&self.claim)?;
        // ponytail: 卷分配睡眠锁串行化计数，真实分配竞争成为瓶颈时拆每组锁。
        let _allocation = self.allocation.lock();
        let g = &self.geometry;
        let superblock = self.metadata.get(0, PAGE_SIZE, None)?;
        let available = superblock.with_read(|bytes| -> Result<u64, FsError> {
            let record = &bytes[codec::SUPERBLOCK_OFFSET..codec::SUPERBLOCK_END];
            if inode {
                return Ok(u64::from(codec::le32(record, 16)?));
            }
            Ok(u64::from(codec::le32(record, 12)?)
                | if g.features_incompat & codec::FEATURE_INCOMPAT_64BIT != 0 {
                    u64::from(codec::le32(record, 344)?) << 32
                } else {
                    0
                })
        })??;
        if available == 0 {
            return Err(FsError::NoSpace);
        }
        for step in 0..g.groups_count {
            let group = (hint % g.groups_count + step) % g.groups_count;
            let (descriptor, offset) = self.group_buffer(group, None)?;
            let mut state = descriptor.with_read(|bytes| {
                let record = codec::bytes(bytes, offset, g.descriptor_size)?;
                codec::decode_group(g, group, record)?;
                codec::group_state(g, record)
            })??;
            if if inode {
                state.free_inodes
            } else {
                state.free_blocks
            } == 0
            {
                continue;
            }
            let layout = self.groups[group as usize];
            let bitmap = self.metadata_block(if inode {
                layout.inode_bitmap
            } else {
                layout.block_bitmap
            })?;
            if inode && state.flags & codec::BG_BLOCK_UNINIT != 0 {
                let blocks = self.metadata_block(layout.block_bitmap)?;
                self.initialize_bitmap(group, false, &blocks, &descriptor, offset, &mut state)?;
            }
            self.initialize_bitmap(group, inode, &bitmap, &descriptor, offset, &mut state)?;
            if g.features_ro_compat & codec::METADATA_CSUM != 0 {
                let sum = bitmap.with_read(|bits| codec::bitmap_checksum(g, bits, inode))?;
                let stored = if inode {
                    state.inode_checksum
                } else {
                    state.block_checksum
                };
                if stored != sum {
                    return Err(FsError::Checksum);
                }
            }
            let valid = valid_bits(g, group, inode);
            let first = if inode && group == 0 {
                g.first_inode.saturating_sub(1) as usize
            } else {
                0
            };
            let bit =
                bitmap.with_read(|bits| (first..valid).find(|&bit| !bit_is_set(bits, bit)))?;
            let Some(bit) = bit else {
                return Err(FsError::Corrupt);
            };
            let checksum = bitmap
                .update(&self.claim, |bits| {
                    assert!(!bit_is_set(bits, bit), "ext4 double allocation");
                    set_bit(bits, bit, true);
                    codec::bitmap_checksum(g, bits, inode)
                })
                .expect("prepared writable bitmap");
            if inode {
                state.free_inodes -= 1;
                state.inode_checksum = checksum;
                if directory {
                    state.used_dirs = state
                        .used_dirs
                        .checked_add(1)
                        .expect("directory count overflow");
                }
                state.unused_inodes = state
                    .unused_inodes
                    .min((g.inodes_per_group - bit as u64 - 1) as u32);
            } else {
                state.free_blocks -= 1;
                state.block_checksum = checksum;
            }
            self.commit_group(group, &descriptor, offset, state);
            let now_ns = time::realtime();
            superblock
                .update(&self.claim, |bytes| {
                    codec::patch_super_counts(
                        g,
                        &mut bytes[codec::SUPERBLOCK_OFFSET..codec::SUPERBLOCK_END],
                        if inode { 0 } else { -1 },
                        if inode { -1 } else { 0 },
                        now_ns,
                    )
                })
                .expect("prepared writable superblock");
            return Ok(Reservation {
                volume: self.device,
                group,
                bit,
                inode,
                directory,
                bitmap,
                descriptor,
                offset,
                superblock,
                active: true,
            });
        }
        Err(FsError::NoSpace)
    }

    /// 合法延迟初始化按物理布局保留全部 flex 元数据和末组尾位。
    fn initialize_bitmap(
        &self,
        group: u64,
        inode: bool,
        bitmap: &BufferRef,
        descriptor: &BufferRef,
        offset: usize,
        state: &mut GroupState,
    ) -> Result<(), FsError> {
        let flag = if inode {
            codec::BG_INODE_UNINIT
        } else {
            codec::BG_BLOCK_UNINIT
        };
        if state.flags & flag == 0 {
            return Ok(());
        }
        let g = &self.geometry;
        if group == 0
            || g.features_ro_compat & (codec::FEATURE_RO_COMPAT_GDT_CSUM | codec::METADATA_CSUM)
                == 0
        {
            return Err(FsError::Corrupt);
        }
        let valid = valid_bits(g, group, inode);
        let mut initialized = [0u8; codec::MAX_BLOCK_BYTES];
        let bits = &mut initialized[..g.block_size];
        if !inode {
            mark_metadata(g, &self.groups, group, bits);
        }
        for bit in valid..bits.len() * 8 {
            set_bit(bits, bit, true);
        }
        let free = (0..valid).filter(|&bit| !bit_is_set(bits, bit)).count() as u32;
        if free
            != if inode {
                state.free_inodes
            } else {
                state.free_blocks
            }
        {
            return Err(FsError::Corrupt);
        }
        let checksum = codec::bitmap_checksum(g, bits, inode);
        bitmap
            .update(&self.claim, |current| current.copy_from_slice(bits))
            .expect("prepared uninitialized bitmap");
        state.flags &= !flag;
        if inode {
            state.inode_checksum = checksum;
        } else {
            state.block_checksum = checksum;
        }
        self.commit_group(group, descriptor, offset, *state);
        Ok(())
    }

    /// 只更新当前描述符字段，不把此前读取的整块副本覆盖共同内容。
    fn commit_group(&self, group: u64, descriptor: &BufferRef, offset: usize, state: GroupState) {
        descriptor
            .update(&self.claim, |bytes| {
                codec::patch_group(
                    &self.geometry,
                    group,
                    &mut bytes[offset..offset + self.geometry.descriptor_size],
                    state,
                )
            })
            .expect("prepared writable group descriptor");
    }

    /// 释放窗口随 token 保留，释放不申请内存、不读盘，内容锁等待在短锁之外。
    fn release_reservation(&self, reservation: &mut Reservation) {
        assert_eq!(
            reservation.volume, self.device,
            "reservation belongs to another volume"
        );
        assert!(
            reservation.active,
            "reservation already published or released"
        );
        let _allocation = self.allocation.lock();
        let g = &self.geometry;
        let mut state = reservation
            .descriptor
            .with_read(|bytes| {
                codec::group_state(
                    g,
                    &bytes[reservation.offset..reservation.offset + g.descriptor_size],
                )
            })
            .expect("retained group buffer")
            .expect("retained group descriptor");
        let sum = reservation
            .bitmap
            .update(&self.claim, |bits| {
                assert!(bit_is_set(bits, reservation.bit), "ext4 double release");
                set_bit(bits, reservation.bit, false);
                codec::bitmap_checksum(g, bits, reservation.inode)
            })
            .expect("retained writable bitmap");
        if reservation.inode {
            state.free_inodes += 1;
            state.inode_checksum = sum;
            if reservation.directory {
                state.used_dirs -= 1;
            }
        } else {
            state.free_blocks += 1;
            state.block_checksum = sum;
        }
        self.commit_group(
            reservation.group,
            &reservation.descriptor,
            reservation.offset,
            state,
        );
        let now_ns = time::realtime();
        reservation
            .superblock
            .update(&self.claim, |bytes| {
                codec::patch_super_counts(
                    g,
                    &mut bytes[codec::SUPERBLOCK_OFFSET..codec::SUPERBLOCK_END],
                    if reservation.inode { 0 } else { 1 },
                    if reservation.inode { 1 } else { 0 },
                    now_ns,
                )
            })
            .expect("retained writable superblock");
        reservation.active = false;
    }
}

/// 位图有效位按实际组尾计算；inode 编号和块号具有各自独立的末组边界。
fn valid_bits(g: &Geometry, group: u64, inode: bool) -> usize {
    if inode {
        (g.inodes_count - group * g.inodes_per_group).min(g.inodes_per_group) as usize
    } else {
        (g.blocks_count - g.first_data_block - group * g.blocks_per_group).min(g.blocks_per_group)
            as usize
    }
}

/// 小端位序与盘面 ext4 位图一致，不将字节尾 padding 当作可分配空间。
fn bit_is_set(bits: &[u8], bit: usize) -> bool {
    bits[bit / 8] & (1 << (bit % 8)) != 0
}

/// 在已验证的位图范围内修改一位。
fn set_bit(bits: &mut [u8], bit: usize, used: bool) {
    if used {
        bits[bit / 8] |= 1 << (bit % 8);
    } else {
        bits[bit / 8] &= !(1 << (bit % 8));
    }
}

/// 元数据按物理交集置位，多个逻辑组的 flex 表可以共同占据当前物理组。
fn mark_metadata(g: &Geometry, layouts: &[GroupLayout], group: u64, bits: &mut [u8]) {
    let first = g.first_data_block + group * g.blocks_per_group;
    let end = (first + g.blocks_per_group).min(g.blocks_count);
    if g.has_super(group) {
        mark_range(
            bits,
            first,
            end,
            first,
            1 + g.descriptor_blocks + g.reserved_gdt_blocks,
        );
    }
    let table_blocks = g.inodes_per_group * g.inode_size as u64 / g.block_size as u64;
    for layout in layouts {
        mark_range(bits, first, end, layout.block_bitmap, 1);
        mark_range(bits, first, end, layout.inode_bitmap, 1);
        mark_range(bits, first, end, layout.inode_table, table_blocks);
    }
}

/// 范围求交后置位；重复占用只保同一个 used 位，不重复扣计数。
fn mark_range(bits: &mut [u8], first: u64, end: u64, start: u64, len: u64) {
    for physical in start.max(first)..(start + len).min(end) {
        set_bit(bits, (physical - first) as usize, true);
    }
}
