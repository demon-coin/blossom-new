//! ext4 物理块与 inode 位图的分配预留及事务化释放管理。
//!
//! # 模块职责与架构定位
//! 本模块归属 `Ext4Volume`，向上游 `mapping`（Extent / 间接块树更新）、`directory`（目录增长）以及
//! VFS `Inode` 生命周期（创建、截断、`nlink == 0` 最终回收）提供块与 inode 的位图分配和归还能力，
//! 向下游操作 `MetadataCache` 中的超级块、块组描述符表（GDT）和位图缓冲区（`BufferRef`）。
//!
//! # 核心设计：位图预留与释放状态机
//! 为了保证元数据更新在中途发生 I/O 错误或内存不足时不会泄漏位图位，也不会在 `Drop` 中隐式执行可能睡眠的缓存操作，
//! 所有分配与释放均通过带显式激活标志 `Reservation::active` 的线性凭证管理：
//!
//! ```text
//! [新资源分配路径]
//!   allocate_block / allocate_inode
//!       │ (持 allocation 锁置位位图、扣减组描述符与超级块空闲计数)
//!       ▼
//!   UnpublishedBlock / UnpublishedInode (reservation.active = true)
//!       ├──► 成功挂载到块树或目录/InodeCache ──► publish() (active = false，位图保持已分配)
//!       └──► 中途失败取消 ──► release_unpublished_*() (清位图、归还计数，active = false)
//!
//! [已有资源释放路径]
//!   prepare_detached_block / prepare_inode_release
//!       │ (预先钉住位图、组描述符、超级块 BufferRef，此时位图未改)
//!       ▼
//!   DetachedBlock / PreparedInodeRelease (reservation.active = false)
//!       ├──► 提交前取消 ──► 直接 Drop (active == false，仅释放 BufferRef 引用)
//!       └──► 块树或 Inode 已摘离引用 ──► arm_release() (active = true)
//!                                            │
//!                                            ▼
//!                              release_detached_block / release_inode
//!                              (失效旧缓存窗口、清位图并归还空闲计数，active = false)
//! ```
//! 若任何 `active == true` 的 `Reservation` 未经 `publish` 或显式 `release_*` 直接进入 `Drop`，将触发断言崩溃以暴露所有权遗漏。
//!
//! # 锁获取顺序与并发契约
//! - VFS Inode 写序列化锁 / Extent 映射写锁 ➔ `Ext4Volume::allocation`（睡眠互斥锁 `SleepMutex`）➔ `MetadataCache` 页/内容锁（`BufferRef::with_read` / `update`）。
//! - 在 `allocate_inode` 中等待旧内存中同号 `Inode` 彻底退场（`wait_inode_reuse`）时，必须在 `Ext4Volume::allocation` 锁**之外**睡眠等待，依靠已置位的位图阻止其他分配者重复选中同一位，防止阻塞全卷分配或与回收路径形成死锁。

use crate::block::MetadataIoContext;
use crate::cache::BufferRef;
use crate::fs::ext4::codec::{self, GroupState};
use crate::fs::ext4::{Ext4Volume, Geometry, InodeData, InodeMetadata};
use crate::fs::types::{S_IALLUGO, S_IFDIR, S_IFLNK, S_IFMT, S_IFREG, S_IRWXUGO, S_ISGID, S_IXGRP};
use crate::fs::{FsError, InodeKind, InodeRef};
use crate::mm::PAGE_SIZE;
use crate::time;
use alloc::vec::Vec;

/// 块组内固定元数据表项的物理块起始地址（卷打开时解码并校验一次，运行期只读）。
#[derive(Clone, Copy)]
pub(crate) struct GroupLayout {
    /// 本块组数据块分配位图所在的物理块号（单位：文件系统块）。
    pub block_bitmap: u64,
    /// 本块组 inode 分配位图所在的物理块号（单位：文件系统块）。
    pub inode_bitmap: u64,
    /// 本块组 inode 表起始物理块号（单位：文件系统块）。
    pub inode_table: u64,
}

/// 单个块位或 inode 位的分配/释放事务凭证，预先持有归还所需的全部元数据缓存引用。
struct Reservation {
    /// 所属 `Ext4Volume` 的内核实例编号，防止跨卷误传凭证。
    volume: u32,
    /// 目标位所在的块组索引（从 `0` 开始）。
    group: u64,
    /// 目标位在块组位图内的偏移（从 `0` 开始，单位：bit）。
    bit: usize,
    /// `true` 表示管理的是 inode 位图位，`false` 表示数据/元数据块位图位。
    inode: bool,
    /// 当 `inode == true` 时标识该 inode 是否为目录，用于同步增减块组的 `bg_used_dirs_count`。
    directory: bool,
    /// 目标块组的块位图或 inode 位图缓存窗口。
    bitmap: BufferRef,
    /// 目标块组描述符所在的 GDT 块缓存窗口。
    descriptor: BufferRef,
    /// 目标块组描述符在 `descriptor` 缓冲区内的字节偏移（单位：字节）。
    offset: usize,
    /// 主超级块所在的第 0 页缓存窗口，用于同步更新全卷空闲块/inode 计数。
    superblock: BufferRef,
    /// 是否负有待显式消费（`publish` 或 `release_reservation`）的位图责任；为 `true` 时禁止直接析构。
    active: bool,
}

/// 已在位图中置位但尚未挂载到任何文件块映射树的物理块所有权凭证。
pub struct UnpublishedBlock {
    /// 本次分配得到的全卷物理块号（单位：文件系统块）。
    physical: u64,
    /// 对应的块位图预留凭证（`active == true` 直到发布或显式归还）。
    reservation: Reservation,
}

/// 准备从文件块树或外部扩展属性（EA）摘除的物理块释放上下文。
pub(crate) struct DetachedBlock {
    /// 预先加载好的位图、组描述符与超级块缓存窗口（预备期 `active == false`）。
    reservation: Reservation,
    /// 该物理块在 `MetadataCache` 中的元数据缓存窗口：
    /// - 若释放的是树节点或外部 EA 块，总是持有其已读取的 `BufferRef`；
    /// - 若释放的是普通文件数据块（曾作为目录块等进入过元数据缓存），仅保留已驻留的缓存命中窗口，未命中时为 `None`。
    ///
    /// 最终释放物理块前需通过该引用失效缓存中的脏状态，防止后台回写把旧数据写回已被复用的物理块。
    alias: Option<BufferRef>,
}

/// 已在 inode 位图中置位并完成盘面初始记录清零、但尚未加入目录或 `InodeDomain` 的新 inode 凭证。
pub struct UnpublishedInode {
    /// 新分配的 1-based ext4 inode 编号。
    number: u64,
    /// 对应的 inode 位图预留凭证（`active == true` 直到发布或显式归还）。
    reservation: Reservation,
    /// 该 inode 盘面记录所在的 inode 表块缓存窗口。
    record: BufferRef,
    /// 该 inode 记录在 `record` 缓冲区内的起始字节偏移（单位：字节）。
    offset: usize,
}

/// 链接数归零且无活跃引用的 inode 在最终回收前准备好的全套盘面释放窗口。
pub(crate) struct PreparedInodeRelease {
    /// 该 inode 自身位图位的释放凭证（预备期 `active == false`，进入 `release_inode` 后激活消费）。
    reservation: Reservation,
    /// 该 inode 盘面记录所在的 inode 表块缓存窗口。
    record: BufferRef,
    /// 该 inode 记录在 `record` 缓冲区内的起始字节偏移（单位：字节）。
    offset: usize,
    /// 结合卷种子、inode 编号与 generation 预计算的 CRC32C 校验种子。
    seed: u32,
    /// 该 inode 使用的扩展头部字节数（`i_extra_isize`，单位：字节）。
    extra_isize: usize,
    /// 若该 inode 挂载了外部扩展属性块（`i_file_acl != 0`），保存其物理块号与预备释放窗口。
    acl: Option<(u64, DetachedBlock)>,
    /// 该 inode（含内嵌 xattr 及引用计数即将归零的外部 xattr 块）引用的全部大值 EA inode 释放列表。
    ea: Vec<PreparedEaRelease>,
}

/// 待扣减引用计数（`EA_INODE` 专用大扩展属性值 inode）的预备释放项。
struct PreparedEaRelease {
    /// 共享 `InodeDomain` 中对应的 EA inode 内存引用。
    inode: InodeRef,
    /// 该 EA inode 盘面记录所在的 inode 表块缓存窗口。
    record: BufferRef,
    /// 该 EA inode 记录在 `record` 缓冲区内的起始字节偏移（单位：字节）。
    offset: usize,
    /// 当前待删除宿主 inode 对该 EA inode 持有的总引用次数。
    count: u64,
    /// 预备时从盘面读取到的该 EA inode 当前总引用计数（用于校验不小于 `count`）。
    references: u64,
}

impl DetachedBlock {
    /// 在新的块映射根与 inode 字段正式提交后激活物理块释放责任；若提交前取消则无需调用此方法，直接丢弃即可。
    pub(crate) fn arm_release(&mut self) {
        assert!(
            !self.reservation.active,
            "ext4 detached block already armed"
        );
        self.reservation.active = true;
    }
}

impl UnpublishedBlock {
    /// 返回本次分配得到的物理块号（单位：文件系统块），不转移位图归还责任。
    pub fn physical(&self) -> u64 {
        self.physical
    }

    /// 在物理块指针正式写入块映射树或 inode 后提交所有权，解除临时未发布状态。
    pub(crate) fn publish(mut self) {
        self.reservation.active = false;
    }
}

impl UnpublishedInode {
    /// 返回本次分配的 1-based ext4 inode 编号。
    pub fn number(&self) -> u64 {
        self.number
    }

    /// 检查该未发布 inode 是否属于指定 `volume` 且仍处于未提交的活动预留状态。
    pub(crate) fn belongs_to(&self, volume: &Ext4Volume) -> bool {
        self.reservation.active && self.reservation.volume == volume.device
    }

    /// 借用该新 inode 在 inode 表中的元数据缓存窗口及块内字节偏移。
    pub(crate) fn record(&self) -> (&BufferRef, usize) {
        (&self.record, self.offset)
    }

    /// 在新 inode 已正式绑定到目录项或 `InodeDomain` 后解除临时预留责任，返回其 inode 编号。
    pub(crate) fn publish(mut self) -> u64 {
        self.reservation.active = false;
        self.number
    }
}

impl Drop for Reservation {
    /// 断言该预留凭证已完成显式发布（`publish`）或归还（`release_*`），防止静默泄漏位图位。
    fn drop(&mut self) {
        assert!(!self.active, "unpublished ext4 reservation lost");
    }
}

impl Ext4Volume {
    /// 从目标提示块组 `hint` 开始扫描块位图，分配一个空闲物理块并返回未发布凭证。
    ///
    /// 必须在可睡眠的普通任务上下文调用；内部会获取 `self.allocation` 睡眠锁并按需读取元数据缓存。
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

    /// 显式归还本次分配但尚未挂载到块映射树的物理块，清零对应位图位并恢复空闲块计数。
    ///
    /// 必须在可睡眠上下文调用，内部会获取 `self.allocation` 睡眠锁。
    pub fn release_unpublished_block(&self, mut block: UnpublishedBlock) {
        self.release_reservation(&mut block.reservation);
    }

    /// 分配一个新的盘面 inode 位并初始化其基础记录（递增 generation、设置权限属主及空 Extent 根）。
    ///
    /// 必须在可睡眠上下文且未持有 `self.allocation` 锁时调用。若选中的 inode 编号在 `InodeDomain` 中仍有正在异步淘汰的旧内存实例，
    /// 会在释放 `self.allocation` 锁的状态下等待旧实例彻底退场后再覆写 inode 表记录；任何中途失败都会自动归还本次预留的位图位。
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

        // Step 1: 在卷分配锁保护下扫描位图并置位，拿到独占的 inode 位预留凭证。
        let mut reservation = self.reserve(hint, true, directory)?;
        let number =
            reservation.group * self.geometry.inodes_per_group + reservation.bit as u64 + 1;
        // Step 2: 若内存中同编号的旧 Inode 尚未完成后台回收，在卷分配锁外等待其退场，避免阻塞其他分配或造成死锁。
        if let Err(error) = self.wait_inode_reuse(number) {
            self.release_reservation(&mut reservation);
            return Err(error);
        }

        // Step 3: 读取 inode 表元数据块，递增 generation 并初始化干净的盘面记录与校验和。
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

    /// 显式归还尚未加入目录或 `InodeDomain` 的新分配 inode，清零位图位并恢复空闲 inode 及目录计数。
    pub fn release_unpublished_inode(&self, mut inode: UnpublishedInode) {
        self.release_reservation(&mut inode.reservation);
    }

    /// 根据父目录的 `S_ISGID` 位、目标文件类型、`requested_mode` 与进程 `umask` 计算新 inode 的权限和属组，并按父目录所在块组就近分配。
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
        // Step 1: 依据标准 setgid 继承规则合成 mode 与有效 gid，防止非同组用户创建带 SGID+可执行位的文件。
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
        // Step 2: 以父目录所在的块组作为局域性提示（hint）分配新 inode。
        let hint =
            parent.number.checked_sub(1).ok_or(FsError::Corrupt)? / self.geometry.inodes_per_group;
        self.allocate_inode(hint, mode, uid, gid)
    }

    /// 为即将从映射树或 inode 摘除的元数据节点块（Extent 内部节点、间接块或外部 EA 块）预加载其自身缓存窗口与位图释放凭证。
    ///
    /// 预备期不修改位图或空闲计数；若后续操作取消，直接丢弃返回值即可无副作用释放缓存引用。
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

    /// 为即将释放的文件数据块（或目录块）准备位图释放凭证，并仅在 `MetadataCache` 已缓存该物理块时捕获其缓存窗口引用，避免无谓读盘。
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

    /// 校验物理块地址合法性，换算其所属块组与组内位偏移，并预留归还所需的组描述符、位图与超级块缓存窗口。
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

    /// 失效已摘除物理块的元数据缓存脏状态，并将该块位归还到块位图与空闲计数中。
    ///
    /// 可睡眠函数，内部会等待元数据页内容锁及获取 `self.allocation` 睡眠锁。
    ///
    /// # Safety
    /// 1. 调用方必须持有所属 inode 映射写锁，且已从盘面块映射树和 inode 记录中彻底移除对该物理块的引用（`block.arm_release()` 已调用）；
    /// 2. 针对该物理块的所有文件页填充、脏页回写（Writeback）及底层块设备 DMA 传输必须已全部完成，硬件不再访问对应扇区；
    /// 3. 自 `prepare_detached_*` 之后不存在也不再新建指向该物理块的 `MetadataCache` 读写消费者。
    pub(crate) unsafe fn release_detached_block(&self, mut block: DetachedBlock) {
        assert!(
            block.reservation.active,
            "ext4 block still belongs to its old tree"
        );
        if let Some(alias) = &block.alias {
            // SAFETY:
            // 1. 调用方契约保证该物理块已从块映射树摘除，且不再有并发的元数据读取或新增脏写；
            // 2. `alias` 在预备时绑定了该块精确的物理字节范围，且此时尚未清零块位图（不会被并发重分配）；
            // 3. `BufferRef::forget` 会在等待该页在途回写彻底结束后仅清除本块范围的缓存有效与脏标志。
            unsafe { alias.forget(&self.claim) }.expect("detached physical block invalidation");
        }
        self.release_reservation(&mut block.reservation);
    }

    /// 在 `nlink == 0` 的 inode 失去最后一个活跃引用后、正式截断数据与清除记录前，预加载并校验该 inode 本身、外部扩展属性块及所有关联 `EA_INODE` 的释放窗口。
    ///
    /// 调用方须持有该 inode 的写序列化保护直至 `release_inode` 消费完毕；若本函数返回错误，盘面记录与引用计数保持原状不变。
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

        // Step 1: 读取并核对盘面 inode 记录与内存快照一致，提取内嵌扩展属性引用的 EA inode 编号。
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
        // Step 2: 预加载内嵌 EA inode 引用以及当前宿主 inode 自身位图的释放凭证。
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

        // Step 3: 若存在外部扩展属性块（file_acl），加载并校验其块头；仅当该块引用计数为 1（即将随本 inode 一并释放）时，收集其中引用的 EA inode。
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
        // Step 4: 确认每个待扣减的 EA inode 当前盘面引用计数均不小于本次即将扣除的次数。
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

    /// 将 `numbers` 中的大扩展属性值 `EA_INODE` 编号合并去重后追加到 `puts`，并预先加载每个目标的盘面记录与 `InodeDomain` 内存引用。
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

    /// 消费已准备好的回收窗口：清零盘面 inode 记录，扣减或释放外部扩展属性块与关联 `EA_INODE` 引用，最后归还本 inode 的位图位及目录计数。
    ///
    /// 全程仅操作预备时已钉住的缓存窗口，不再申请堆内存或触发未缓存读盘；必须在可睡眠的任务或工作队列上下文中调用（严禁在中断上下文、任意 `Drop` 或自旋锁临界区内调用）。
    ///
    /// # Safety
    /// 1. 调用方必须已在 `InodeDomain` 索引中将该宿主 inode 标记为正在回收，阻止任何新查找获取其引用，且其 `nlink == 0`、全部数据块与映射树节点均已释放完毕；
    /// 2. 宿主 inode 关联的所有文件页读取、脏页回写、底层设备 DMA I/O 及旧元数据缓存窗口消费者均已彻底结束；
    /// 3. 调用方必须在同一卷回写序列化上下文中调用直至本函数返回，防止并发修改关联 `EA_INODE` 的引用计数。
    pub(crate) unsafe fn release_inode(&self, mut prepared: PreparedInodeRelease) {
        assert_eq!(prepared.reservation.volume, self.device);
        prepared.reservation.active = true;
        let g = &self.geometry;
        // Step 1: 将盘面 inode 记录标记为已释放（写入 dtime、清零块区并重算校验和）。
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

        // Step 2: 在卷分配锁下检查外部扩展属性块引用计数；若仍被其他 inode 共享则仅递减计数，否则摘除并释放该物理块。
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
                // SAFETY:
                // 1. Step 1 已从当前 inode 记录中抹除 `file_acl` 引用，且 Step 2 在 `self.allocation` 锁下确认该外部 EA 块引用计数为 1（无其他共享者）；
                // 2. 调用方契约保证针对该块的旧访问与在途回写已结束，`block` 持有预备时锁定的精确缓存窗口，在清位图重用前先由 `forget` 安全失效。
                unsafe { self.release_detached_block(block) };
            }
        }

        // Step 3: 逐项扣减关联大值 EA inode 的引用计数，最后归还本宿主 inode 的位图位。
        for put in &prepared.ea {
            put.inode
                .release_ea_references(&self.claim, &put.record, put.offset, put.count);
        }
        self.release_reservation(&mut prepared.reservation);
    }

    /// 预加载指定块组与位偏移归还所需的组描述符、位图及主超级块缓存窗口，初始 `active` 设为 `false` 以便中途取消时可直接丢弃。
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

    /// 在 `self.allocation` 锁保护下按提示块组循环扫描空闲位，必要时执行延迟位图初始化，置位位图并同步扣减块组描述符与超级块空闲计数。
    fn reserve(&self, hint: u64, inode: bool, directory: bool) -> Result<Reservation, FsError> {
        self.require_write(&self.claim)?;
        // ponytail: 卷分配睡眠锁串行化计数，真实分配竞争成为瓶颈时拆每组锁。
        let _allocation = self.allocation.lock();
        let g = &self.geometry;
        // Step 1: 读取主超级块确认全卷尚有空闲块或空闲 inode，若已耗尽则直接返回 NoSpace。
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

        // Step 2: 从 hint 所在块组开始环形遍历全卷各块组，跳过空闲计数为 0 的块组。
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
            // Step 3: 加载目标位图；若开启了延迟初始化（UNINIT），先在内存中按物理布局构造初始位图并落入缓存，随后校验位图 CRC。
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

            // Step 4: 在有效位范围内查找首个空闲位（第 0 组 inode 跳过保留区），置位后更新组描述符与主超级块的空闲计数及校验和。
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

    /// 处理带有 `BG_INODE_UNINIT` 或 `BG_BLOCK_UNINIT` 标志的未初始化块组位图：按卷几何与 `flex_bg` 物理布局标记元数据占用位和组尾越界位，校验空闲数一致后写入缓存并清除 `UNINIT` 标志。
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

    /// 在组描述符缓存窗口的更新锁内，将新的块组空闲计数、标志与重算后的组校验和写回当前描述符槽位。
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

    /// 消费活动的 `Reservation`：在 `self.allocation` 锁保护下清零目标位图位，递增块组描述符与主超级块的空闲计数（及目录计数），并将 `active` 置为 `false`。
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

/// 计算块组 `group` 位图中的实际有效位数（末个块组的块数或 inode 数可能少于标准每组上限）。
fn valid_bits(g: &Geometry, group: u64, inode: bool) -> usize {
    if inode {
        (g.inodes_count - group * g.inodes_per_group).min(g.inodes_per_group) as usize
    } else {
        (g.blocks_count - g.first_data_block - group * g.blocks_per_group).min(g.blocks_per_group)
            as usize
    }
}

/// 按 ext4 盘面小端位序（字节内 LSB 对应最小编号）检查位图第 `bit` 位是否已置为占用。
fn bit_is_set(bits: &[u8], bit: usize) -> bool {
    bits[bit / 8] & (1 << (bit % 8)) != 0
}

/// 按 ext4 盘面小端位序将位图切片中的第 `bit` 位置为占用（`true`）或空闲（`false`）。
fn set_bit(bits: &mut [u8], bit: usize, used: bool) {
    if used {
        bits[bit / 8] |= 1 << (bit % 8);
    } else {
        bits[bit / 8] &= !(1 << (bit % 8));
    }
}

/// 在初始化数据块位图时，将落入物理块组 `group` 地址区间内的本组超级块/GDT 备份以及全卷所有块组（支持 `flex_bg` 跨组放置）的位图和 inode 表物理块统一标记为已占用。
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

/// 求物理块区间 `[start, start + len)` 与当前块组物理范围 `[first, end)` 的交集，并将交集对应的组内相对位全部置为已占用。
fn mark_range(bits: &mut [u8], first: u64, end: u64, start: u64, len: u64) {
    for physical in start.max(first)..(start + len).min(end) {
        set_bit(bits, (physical - first) as usize, true);
    }
}
