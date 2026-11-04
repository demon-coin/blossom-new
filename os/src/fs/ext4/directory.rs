//! 目录记录、原生 HTree 修改与稳定续读游标共享同一后端。

use alloc::vec::Vec;

use super::mapping::PreparedDirectoryMappings;
use crate::block::SECTOR_BYTES;
use crate::cache::BufferRef;
use crate::fs::ext4::codec::{self, INDEX, METADATA_CSUM};
use crate::fs::ext4::{BlockMapping, Ext4Volume, Geometry};
use crate::fs::inode::InodeMetadata;
use crate::fs::types::NAME_MAX;
use crate::fs::{DirectoryCursor, DirectoryEntry, DirectoryPosition, FsError, InodeKind};

const DX_MAX_INDIRECT_LEVELS: usize = 2;
const DX_EOF_COOKIE: u64 = 0x7fff_ffff_ffff_ffff;
const DX_BLOCK_MASK: u32 = (1 << 28) - 1;
const DIRENT_HEADER_BYTES: usize = 8;
const DIRENT_ALIGNMENT: usize = 4;
const DIRENT_ROUND: usize = DIRENT_ALIGNMENT - 1;
const DIRENT_MIN_BYTES: usize = (DIRENT_HEADER_BYTES + 1 + DIRENT_ROUND) & !DIRENT_ROUND;
const DX_ENTRY_BYTES: usize = 8;
const DX_TAIL_BYTES: usize = 8;
const DX_NODE_ENTRIES_OFFSET: usize = 8;
const DX_ROOT_ENTRIES_OFFSET: usize = 32;

/// 一次已准备目录修改对 canonical inode 发布的映射与计数。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct DirectoryEditResult {
    /// 修改后的 inode inline 块映射根。
    pub(crate) root: [u8; 60],
    /// 目录按字节计的逻辑大小。
    pub(crate) size: u64,
    /// 目录的 512 字节 sector 计数。
    pub(crate) blocks: u64,
    /// 本次编辑更新后的 inode flags，包括可能的 INDEX promotion。
    pub(crate) flags: u32,
}

struct DirectoryImage {
    logical: u64,
    buffer: Option<BufferRef>,
    bytes: Vec<u8>,
}

/// 在位图归还前逐一失效本次未发表物理块的既有缓存窗口。
fn forget_unpublished_images(
    volume: &Ext4Volume,
    mapping: &PreparedDirectoryMappings,
    logical_blocks: &[u64],
) {
    for &logical in logical_blocks {
        let Some(physical) = mapping.physical(logical) else {
            continue;
        };
        let buffer = volume
            .metadata_cache()
            .cached_buffer(physical, volume.geometry().block_size)
            .expect("unpublished directory cache range invalid");
        let Some(buffer) = buffer else {
            continue;
        };
        buffer.wait_writeback();
        // SAFETY: 该缓存窗口经同步 metadata_block 读入，wait_writeback 再等完该页 WB；
        // 当前 reservation 仍持物理块，旧目录根和其他读者都不能引用此准确窗口。
        unsafe { buffer.forget(volume.claim()) }.expect("unpublished directory block invalidation");
    }
}

/// 失效本次新增目录块的缓存窗口后，再返还全部映射 reservation。
fn cancel_unpublished_mappings(
    volume: &Ext4Volume,
    mapping: PreparedDirectoryMappings,
    logical_blocks: &[u64],
) {
    forget_unpublished_images(volume, &mapping, logical_blocks);
    mapping.cancel(volume);
}

/// 所有失败条件与元数据窗口均已准备，发布只改字节并消费已分配块。
pub(crate) struct PreparedDirectoryEdit {
    mapping: Option<PreparedDirectoryMappings>,
    images: Vec<DirectoryImage>,
    logical_blocks: Vec<u64>,
    root: [u8; 60],
    size: u64,
    blocks: u64,
    flags: u32,
}

impl PreparedDirectoryEdit {
    /// 借用本次编辑将发布的 inode 块映射根。
    pub(crate) fn root(&self) -> &[u8; 60] {
        &self.root
    }

    /// 返回本次编辑将发布的目录字节大小。
    pub(crate) fn size(&self) -> u64 {
        self.size
    }

    /// 返回本次编辑将发布的 512 字节 sector 计数。
    pub(crate) fn blocks(&self) -> u64 {
        self.blocks
    }
    /// 返回本次编辑将发布的 inode flags。
    pub(crate) fn flags(&self) -> u32 {
        self.flags
    }

    /// 判断本次是否实际改变了 dirent；同 inode rename 的 no-op 不改时间或链接数。
    pub(crate) fn changed(&self) -> bool {
        !self.images.is_empty()
    }

    /// 写入已准备的目录块和映射节点，不分配、读取磁盘或返回可恢复错误。
    pub(crate) fn publish(self, volume: &Ext4Volume) -> DirectoryEditResult {
        let Self {
            mapping,
            images,
            logical_blocks,
            root,
            size,
            blocks,
            flags,
        } = self;
        // 新目录块先有完整内容，旧根和索引只在其后才指向这些块。
        for new_block in [true, false] {
            for image in &images {
                if logical_blocks.contains(&image.logical) != new_block {
                    continue;
                }
                let buffer = image.buffer.as_ref().expect("prepared directory block");
                buffer
                    .update(volume.claim(), |bytes| bytes.copy_from_slice(&image.bytes))
                    .expect("prepared writable directory block");
            }
        }
        if let Some(mapping) = mapping {
            mapping.publish(volume);
        }
        DirectoryEditResult {
            root,
            size,
            blocks,
            flags,
        }
    }

    /// 归还尚未连接到目录映射的块，保留所有已发表目录内容。
    pub(crate) fn cancel(self, volume: &Ext4Volume) {
        if let Some(mapping) = self.mapping {
            cancel_unpublished_mappings(volume, mapping, &self.logical_blocks);
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DirectoryLayout {
    Leaf { end: usize },
    Root,
    Index,
}

/// 按 HTree cookie 解释共享位置；布局切换也沿用原 cookie 数值而不重置到开头。
fn indexed_cursor(cookie: u64) -> Result<DirectoryCursor, FsError> {
    if DX_EOF_COOKIE < cookie {
        return Err(FsError::InvalidOperation);
    }
    Ok(DirectoryCursor::Indexed {
        major: ((cookie >> 32) as u32).wrapping_shl(1),
        minor: cookie as u32,
        collision: None,
    })
}

/// 从用户 cookie 建立后端位置，显式 seek 不沿用之前的碰撞状态。
pub(crate) fn cursor_at(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    _map_root: &[u8; 60],
    cookie: u64,
) -> Result<DirectoryCursor, FsError> {
    check_directory(volume, inode)?;
    if inode.flags & INDEX == 0 {
        return Ok(DirectoryCursor::Byte(cookie));
    }
    indexed_cursor(cookie)
}

/// 返回后端的 SEEK_END 位置，indexed 目录使用 hash EOF 哨兵。
pub(crate) fn end_cookie(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    _map_root: &[u8; 60],
) -> Result<u64, FsError> {
    check_directory(volume, inode)?;
    Ok(if inode.flags & INDEX != 0 {
        DX_EOF_COOKIE
    } else {
        inode.size
    })
}

#[derive(Clone, Copy)]
struct Record {
    number: u64,
    name_offset: usize,
    name_len: usize,
    kind: Option<InodeKind>,
    next: usize,
}

/// 检查目录类型及完整块大小，避免将不完整尾块解释成目录项。
fn check_directory(volume: &Ext4Volume, inode: &InodeMetadata) -> Result<(), FsError> {
    if inode.kind != InodeKind::Directory {
        return Err(FsError::NotDirectory);
    }
    if inode.size == 0 || inode.size % volume.geometry().block_size as u64 != 0 {
        return Err(FsError::Corrupt);
    }
    if inode.flags & INDEX != 0
        && volume.geometry().features_compat & codec::FEATURE_COMPAT_DIR_INDEX == 0
    {
        return Err(FsError::Corrupt);
    }
    Ok(())
}

/// 读取一个真实目录块，空洞或未写入块不能充当空目录。
fn directory_block(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
) -> Result<BufferRef, FsError> {
    let physical = match volume.map_block(inode, map_root, logical, None)? {
        BlockMapping::Mapped { physical, .. } => physical,
        _ => return Err(FsError::Corrupt),
    };
    volume.metadata_block(physical)
}

#[derive(Clone, Copy, Default)]
struct DxFrame {
    logical: u64,
    root: bool,
    position: usize,
    count: usize,
}

#[derive(Clone, Copy)]
struct DxEntry {
    hash: u32,
    block: u64,
}

struct DxPath {
    frames: [DxFrame; DX_MAX_INDIRECT_LEVELS + 1],
    len: usize,
    leaf: u64,
    lower_bound: u32,
}
/// 返回当前文件系统允许的最大 dx 间接层数。
fn dx_max_indirect_levels(g: &Geometry) -> usize {
    if g.features_incompat & codec::FEATURE_INCOMPAT_LARGEDIR != 0 {
        DX_MAX_INDIRECT_LEVELS
    } else {
        1
    }
}

/// 解码根或节点中的一个真实 dx 条目，首项的哈希隐含为零。
fn dx_entry(data: &[u8], offset: usize, position: usize) -> Result<DxEntry, FsError> {
    Ok(DxEntry {
        hash: if position == 0 {
            0
        } else {
            codec::le32(data, offset + position * DX_ENTRY_BYTES)?
        },
        block: u64::from(
            codec::le32(data, offset + position * DX_ENTRY_BYTES + 4)? & DX_BLOCK_MASK,
        ),
    })
}

/// 按前驱规则选择不大于目标哈希的最后一个 dx 条目。
fn dx_select(data: &[u8], offset: usize, count: usize, hash: u32) -> Result<usize, FsError> {
    let mut low = 1usize;
    let mut high = count;
    while low < high {
        let middle = low + (high - low) / 2;
        if codec::le32(data, offset + middle * DX_ENTRY_BYTES)? <= hash {
            low = middle + 1;
        } else {
            high = middle;
        }
    }
    Ok(low - 1)
}

/// 从一个已验证索引块中选择目标哈希的子范围。
fn read_dx_frame(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
    root: bool,
    hash: u32,
) -> Result<(DxFrame, DxEntry, u8), FsError> {
    let buffer = directory_block(volume, inode, map_root, logical)?;
    let g = volume.geometry();
    buffer.with_read(|data| {
        let layout = verify_directory(g, inode, logical, data, !root)?;
        if root && layout != DirectoryLayout::Root || !root && layout != DirectoryLayout::Index {
            return Err(FsError::Corrupt);
        }
        let offset = if root {
            DX_ROOT_ENTRIES_OFFSET
        } else {
            DX_NODE_ENTRIES_OFFSET
        };
        let count = usize::from(codec::le16(data, offset + 2)?);
        let position = dx_select(data, offset, count, hash)?;
        let entry = dx_entry(data, offset, position)?;
        Ok::<_, FsError>((
            DxFrame {
                logical,
                root,
                position,
                count,
            },
            entry,
            if root { data[30] } else { 0 },
        ))
    })?
}

/// 从已选中的索引块读取指定条目并核对目录锁保护下的原条目数。
fn read_dx_entry(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    frame: DxFrame,
    position: usize,
) -> Result<DxEntry, FsError> {
    let buffer = directory_block(volume, inode, map_root, frame.logical)?;
    let g = volume.geometry();
    buffer.with_read(|data| {
        let layout = verify_directory(g, inode, frame.logical, data, !frame.root)?;
        let expected = if frame.root {
            DirectoryLayout::Root
        } else {
            DirectoryLayout::Index
        };
        if layout != expected {
            return Err(FsError::Corrupt);
        }
        let offset = if frame.root {
            DX_ROOT_ENTRIES_OFFSET
        } else {
            DX_NODE_ENTRIES_OFFSET
        };
        let count = usize::from(codec::le16(data, offset + 2)?);
        if count != frame.count || count <= position {
            return Err(FsError::Corrupt);
        }
        dx_entry(data, offset, position)
    })?
}

/// 取得节点首项，用于从父索引的相邻范围继续下降。
fn read_dx_first(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
) -> Result<(DxFrame, DxEntry), FsError> {
    let buffer = directory_block(volume, inode, map_root, logical)?;
    let g = volume.geometry();
    buffer.with_read(|data| {
        if verify_directory(g, inode, logical, data, true)? != DirectoryLayout::Index {
            return Err(FsError::Corrupt);
        }
        let count = usize::from(codec::le16(data, 10)?);
        Ok::<_, FsError>((
            DxFrame {
                logical,
                root: false,
                position: 0,
                count,
            },
            dx_entry(data, 8, 0)?,
        ))
    })?
}

/// 按真实索引前驱定位叶块，并累积该叶区间的最小 major 哈希。
fn dx_probe(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    hash: u32,
) -> Result<DxPath, FsError> {
    let (root, entry, depth) = read_dx_frame(volume, inode, map_root, 0, true, hash)?;
    let mut path = DxPath {
        frames: [DxFrame::default(); DX_MAX_INDIRECT_LEVELS + 1],
        len: 1,
        leaf: entry.block,
        lower_bound: entry.hash & !1,
    };
    path.frames[0] = root;
    let mut logical = entry.block;
    let mut visited = [u64::MAX; DX_MAX_INDIRECT_LEVELS];
    let max_frames = dx_max_indirect_levels(volume.geometry()) + 1;
    let mut level = 0usize;
    while level < usize::from(depth) {
        if visited[..level].contains(&logical) || path.len == max_frames {
            return Err(FsError::Corrupt);
        }
        visited[level] = logical;
        let (frame, child, _) = read_dx_frame(volume, inode, map_root, logical, false, hash)?;
        path.lower_bound = path.lower_bound.max(child.hash & !1);
        path.frames[path.len] = frame;
        path.len += 1;
        logical = child.block;
        level += 1;
    }
    path.leaf = logical;
    Ok(path)
}
/// 按树内序查询下一叶区间下界；高于必须覆盖的 major 时不读取其子节点。
fn next_dx_leaf(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    path: &mut DxPath,
    through_major: Option<u32>,
) -> Result<Option<u32>, FsError> {
    let mut level = path.len;
    while 0 < level {
        level -= 1;
        let frame = path.frames[level];
        let next = frame.position + 1;
        if next == frame.count {
            continue;
        }
        let entry = read_dx_entry(volume, inode, map_root, frame, next)?;
        let lower_bound = path.lower_bound.max(entry.hash & !1);
        if through_major.is_some_and(|major| major < lower_bound) {
            return Ok(Some(lower_bound));
        }
        path.frames[level].position = next;
        let mut child = entry.block;
        for child_level in level + 1..path.len {
            let (frame, entry) = read_dx_first(volume, inode, map_root, child)?;
            path.frames[child_level] = frame;
            child = entry.block;
        }
        path.leaf = child;
        path.lower_bound = lower_bound;
        return Ok(Some(lower_bound));
    }
    Ok(None)
}

/// 递归穿过剩余 dx 索引层，按树内序访问真实叶块。
fn for_each_dx_leaf(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
    levels: u8,
    visit: &mut impl FnMut(u64) -> Result<bool, FsError>,
) -> Result<bool, FsError> {
    if levels == 0 {
        return visit(logical);
    }
    let node = directory_block(volume, inode, map_root, logical)?;
    let g = volume.geometry();
    let count = node.with_read(|data| {
        if verify_directory(g, inode, logical, data, true)? != DirectoryLayout::Index {
            return Err(FsError::Corrupt);
        }
        Ok::<_, FsError>(usize::from(codec::le16(data, 10)?))
    })??;
    for position in 0..count {
        let child =
            node.with_read(|data| dx_entry(data, 8, position).map(|entry| entry.block))??;
        if !for_each_dx_leaf(volume, inode, map_root, child, levels - 1, visit)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// 按树内序逐个访问叶块，不分配目录项或叶位置副本。
fn for_each_leaf(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    mut visit: impl FnMut(u64) -> Result<bool, FsError>,
) -> Result<bool, FsError> {
    let root = directory_block(volume, inode, map_root, 0)?;
    let g = volume.geometry();
    let (depth, count) = root.with_read(|data| {
        if verify_directory(g, inode, 0, data, false)? != DirectoryLayout::Root {
            return Err(FsError::Corrupt);
        }
        Ok::<_, FsError>((data[30], usize::from(codec::le16(data, 34)?)))
    })??;
    for position in 0..count {
        let entry = root.with_read(|data| dx_entry(data, 32, position))??;
        if !for_each_dx_leaf(volume, inode, map_root, entry.block, depth, &mut visit)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// 验证普通目录尾或 dx 根/节点及其正式校验范围，返回可枚举区域。
pub(crate) fn verify_directory(
    g: &Geometry,
    inode: &InodeMetadata,
    logical: u64,
    data: &[u8],
    index: bool,
) -> Result<codec::DirectoryLayout, FsError> {
    let data = codec::bytes(data, 0, g.block_size)?;
    let indexed = inode.flags & INDEX != 0;
    let root = indexed && logical == 0;
    let node = indexed && !root && index;
    if root || node {
        verify_dx(g, inode, logical, data, root)?;
        if root {
            let dot = record(g, data, 0, data.len())?;
            let parent = record(g, data, 12, data.len())?;
            if dot.number != inode.number
                || dot.next != 12
                || parent.next != data.len()
                || dot.name_len != 1
                || data[8] != b'.'
                || parent.name_len != 2
                || &data[20..22] != b".."
                || parent.number == 0
            {
                return Err(FsError::Corrupt);
            }
            return Ok(DirectoryLayout::Root);
        }
        return Ok(DirectoryLayout::Index);
    }
    let end = if g.features_ro_compat & METADATA_CSUM != 0 {
        let end = data.len() - codec::DIR_TAIL_BYTES;
        if codec::le32(data, end)? != 0
            || codec::le16(data, end + 4)? != codec::DIR_TAIL_BYTES as u16
            || data[end + 6] != 0
            || data[end + 7] != codec::FT_DIR_CSUM
        {
            return Err(FsError::Corrupt);
        }
        codec::verify_directory_checksum(g, inode, data, end)?;
        end
    } else {
        data.len()
    };
    let mut offset = 0;
    while offset < end {
        offset = record(g, data, offset, end)?.next;
    }
    if offset != end {
        return Err(FsError::Corrupt);
    }
    Ok(DirectoryLayout::Leaf { end })
}

/// 验证 dx 布局、条目排序与逻辑块地址，CRC 不包含未使用索引槽。
fn verify_dx(
    g: &Geometry,
    inode: &InodeMetadata,
    logical: u64,
    data: &[u8],
    root: bool,
) -> Result<(), FsError> {
    let offset = if root {
        if codec::le16(data, 4)? != 12
            || codec::le16(data, 16)? as usize != data.len() - 12
            || codec::le32(data, 24)? != 0
            || data[29] != 8
            || data[31] != 0
        {
            return Err(FsError::Corrupt);
        }
        if codec::DX_HASH_TEA_UNSIGNED < data[28] {
            return Err(FsError::Unsupported);
        }
        if dx_max_indirect_levels(g) < usize::from(data[30]) {
            return Err(FsError::Corrupt);
        }
        DX_ROOT_ENTRIES_OFFSET
    } else {
        if codec::le32(data, 0)? != 0
            || codec::le16(data, 6)? != 0
            || codec::le16(data, 4)? as usize != data.len()
        {
            return Err(FsError::Corrupt);
        }
        DX_NODE_ENTRIES_OFFSET
    };
    let limit = usize::from(codec::le16(data, offset)?);
    let count = usize::from(codec::le16(data, offset + 2)?);
    let tail_len = if g.features_ro_compat & METADATA_CSUM != 0 {
        DX_TAIL_BYTES
    } else {
        0
    };
    if limit != (data.len() - offset - tail_len) / DX_ENTRY_BYTES || count == 0 || limit < count {
        return Err(FsError::Corrupt);
    }
    let blocks = inode.size / g.block_size as u64;
    let mut previous_hash = 0u32;
    for index in 0..count {
        let entry = offset + index * DX_ENTRY_BYTES;
        if index != 0 {
            let hash = codec::le32(data, entry)?;
            if hash < previous_hash {
                return Err(FsError::Corrupt);
            }
            previous_hash = hash;
        }
        let block = u64::from(codec::le32(data, entry + 4)? & DX_BLOCK_MASK);
        if block == 0 || blocks <= block || block == logical {
            return Err(FsError::Corrupt);
        }
    }
    if tail_len != 0 {
        let tail = offset + limit * DX_ENTRY_BYTES;
        // dx 尾的保留字节只参与 CRC，读取时不赋予零值约束。
        codec::verify_dx_checksum(inode, data, offset + count * DX_ENTRY_BYTES, tail)?;
    }
    Ok(())
}

/// 解码一个真实记录并限制名称及下一记录边界，保留非 UTF-8 字节。
fn record(g: &Geometry, data: &[u8], offset: usize, end: usize) -> Result<Record, FsError> {
    let number = u64::from(codec::le32(data, offset)?);
    let len = usize::from(codec::le16(data, offset + 4)?);
    let filetype = g.features_incompat & codec::FEATURE_INCOMPAT_FILETYPE != 0;
    let header = codec::bytes(data, offset, DIRENT_HEADER_BYTES)?;
    let name_len = if filetype {
        usize::from(header[6])
    } else {
        usize::from(codec::le16(data, offset + 6)?)
    };
    let next = offset.checked_add(len).ok_or(FsError::Corrupt)?;
    if len < DIRENT_MIN_BYTES
        || len & DIRENT_ROUND != 0
        || NAME_MAX < name_len
        || len < ((DIRENT_HEADER_BYTES + name_len + DIRENT_ROUND) & !DIRENT_ROUND)
        || end < next
        || (next < end && end - next < DIRENT_MIN_BYTES)
        || g.inodes_count < number
    {
        return Err(FsError::Corrupt);
    }
    let name_offset = offset + DIRENT_HEADER_BYTES;
    let name = codec::bytes(data, name_offset, name_len)?;
    if number != 0 && (name.is_empty() || name.contains(&0) || name.contains(&b'/')) {
        return Err(FsError::Corrupt);
    }
    let kind = if filetype {
        match header[7] {
            0 => None,
            1 => Some(InodeKind::Regular),
            2 => Some(InodeKind::Directory),
            3 => Some(InodeKind::Character),
            4 => Some(InodeKind::Block),
            5 => Some(InodeKind::Fifo),
            6 => Some(InodeKind::Socket),
            7 => Some(InodeKind::Symlink),
            _ => return Err(FsError::Corrupt),
        }
    } else {
        None
    };
    Ok(Record {
        number,
        name_offset,
        name_len,
        kind,
        next,
    })
}

/// 取指定真实游标之后的首个活记录，索引伪项及尾记录不进入结果。
fn scan_next(
    g: &Geometry,
    data: &[u8],
    layout: DirectoryLayout,
    requested: usize,
) -> Result<Option<Record>, FsError> {
    if layout == DirectoryLayout::Index {
        return Ok(None);
    }
    let end = match layout {
        DirectoryLayout::Leaf { end } => end,
        _ => data.len(),
    };
    let mut offset = 0usize;
    while offset < end {
        let entry = record(g, data, offset, end)?;
        if offset < requested && requested < entry.next {
            return Err(FsError::InvalidOperation);
        }
        if requested <= offset && entry.number != 0 {
            return Ok(Some(entry));
        }
        offset = entry.next;
    }
    if end < requested && requested != data.len() {
        return Err(FsError::InvalidOperation);
    }
    Ok(None)
}

#[derive(Clone, Copy)]
struct FoundEntry {
    logical: u64,
    offset: usize,
    number: u64,
    name_len: usize,
    kind: Option<InodeKind>,
}

/// 在一个已校验的普通叶或目录根中按原始字节定位名字。
fn find_entry_in_block(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
    name: &[u8],
) -> Result<Option<FoundEntry>, FsError> {
    let buffer = directory_block(volume, inode, map_root, logical)?;
    let g = volume.geometry();
    buffer.with_read(|data| {
        let layout = verify_directory(g, inode, logical, data, false)?;
        let end = match layout {
            DirectoryLayout::Leaf { end } => end,
            DirectoryLayout::Root => data.len(),
            DirectoryLayout::Index => return Err(FsError::Corrupt),
        };
        let mut offset = 0usize;
        while offset < end {
            let entry = record(g, data, offset, end)?;
            if entry.number != 0
                && &data[entry.name_offset..entry.name_offset + entry.name_len] == name
            {
                return Ok(Some(FoundEntry {
                    logical,
                    offset,
                    number: entry.number,
                    name_len: entry.name_len,
                    kind: entry.kind,
                }));
            }
            offset = entry.next;
        }
        Ok(None)
    })?
}

/// 读取根中声明的哈希版本，不能用主超级块默认值替代旧目录自身格式。
fn root_hash_version(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
) -> Result<u8, FsError> {
    let root = directory_block(volume, inode, map_root, 0)?;
    root.with_read(|data| {
        if verify_directory(volume.geometry(), inode, 0, data, false)? != DirectoryLayout::Root {
            return Err(FsError::Corrupt);
        }
        Ok::<_, FsError>(data[28])
    })?
}

/// 在 indexed 目录沿 dx_probe 路径及同 major continuation 叶查找名字。
fn find_entry(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    name: &[u8],
) -> Result<Option<FoundEntry>, FsError> {
    check_directory(volume, inode)?;
    if name.is_empty() || NAME_MAX < name.len() || name.contains(&0) || name.contains(&b'/') {
        return Err(FsError::InvalidName);
    }
    if inode.flags & INDEX == 0 {
        for logical in 0..inode.size / volume.geometry().block_size as u64 {
            if let Some(entry) = find_entry_in_block(volume, inode, map_root, logical, name)? {
                return Ok(Some(entry));
            }
        }
        return Ok(None);
    }
    if name == b"." || name == b".." {
        return find_entry_in_block(volume, inode, map_root, 0, name);
    }
    let hash_version = root_hash_version(volume, inode, map_root)?;
    let (hash, _) = codec::directory_hash(volume.geometry(), hash_version, name)?;
    let mut path = dx_probe(volume, inode, map_root, hash)?;
    loop {
        if let Some(entry) = find_entry_in_block(volume, inode, map_root, path.leaf, name)? {
            return Ok(Some(entry));
        }
        let Some(lower_bound) = next_dx_leaf(volume, inode, map_root, &mut path, Some(hash))?
        else {
            return Ok(None);
        };
        if hash < lower_bound {
            return Ok(None);
        }
    }
}

/// 按 HTree 路由索引目录查询，普通目录保持真实块顺序扫描。
pub(crate) fn lookup(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    name: &[u8],
) -> Result<Option<u64>, FsError> {
    Ok(find_entry(volume, inode, map_root, name)?.map(|entry| entry.number))
}

#[derive(Clone, Copy)]
struct HashCandidate {
    major: u32,
    minor: u32,
    entry: FoundEntry,
}

/// 判断索引 cookie 之后的真实记录是否尚未交付。
fn follows_cursor(
    cursor: &DirectoryCursor,
    major: u32,
    minor: u32,
    position: DirectoryPosition,
) -> bool {
    let DirectoryCursor::Indexed {
        major: current_major,
        minor: current_minor,
        collision,
    } = cursor
    else {
        return false;
    };
    let key = (major, minor);
    let current = (*current_major, *current_minor);
    if current < key {
        return true;
    }
    if key < current {
        return false;
    }
    match *collision {
        None => true,
        Some(last) => (last.logical, last.offset) < (position.logical, position.offset),
    }
}

/// 比较同一目录中按 major、minor 和真实记录位置排序的候选项。
fn precedes(left: HashCandidate, right: HashCandidate) -> bool {
    (
        left.major,
        left.minor,
        left.entry.logical,
        left.entry.offset,
    ) < (
        right.major,
        right.minor,
        right.entry.logical,
        right.entry.offset,
    )
}

/// 按目录哈希和真实位置保留游标后的前两项。
fn keep_two_candidates(
    best: &mut Option<HashCandidate>,
    following: &mut Option<HashCandidate>,
    candidate: HashCandidate,
) {
    if (*best).is_none_or(|current| precedes(candidate, current)) {
        *following = *best;
        *best = Some(candidate);
    } else if (*following).is_none_or(|current| precedes(candidate, current)) {
        *following = Some(candidate);
    }
}

/// 按已提交哈希和碰撞位置选出下一项，只扫描其候选所在的 HTree 哈希区间。
fn next_indexed(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    cursor: &DirectoryCursor,
) -> Result<Option<DirectoryEntry>, FsError> {
    let DirectoryCursor::Indexed { major, .. } = cursor else {
        return Err(FsError::InvalidOperation);
    };
    if *major & 1 != 0 {
        return Err(FsError::InvalidOperation);
    }
    if cursor.cookie() == DX_EOF_COOKIE {
        return Ok(None);
    }
    let g = volume.geometry();
    let version = root_hash_version(volume, inode, map_root)?;
    let mut best = None;
    let mut following = None;
    let root = directory_block(volume, inode, map_root, 0)?;
    let dots = root.with_read(|data| {
        if verify_directory(g, inode, 0, data, false)? != DirectoryLayout::Root {
            return Err(FsError::Corrupt);
        }
        let dot = record(g, data, 0, data.len())?;
        let parent = record(g, data, 12, data.len())?;
        Ok::<_, FsError>((dot, parent))
    })??;
    for (major, entry) in [(0, dots.0), (2, dots.1)] {
        let candidate = HashCandidate {
            major,
            minor: 0,
            entry: FoundEntry {
                logical: 0,
                offset: if major == 0 { 0 } else { 12 },
                number: entry.number,
                name_len: entry.name_len,
                kind: entry.kind,
            },
        };
        let position = DirectoryPosition {
            logical: candidate.entry.logical,
            offset: candidate.entry.offset as u32,
        };
        if follows_cursor(cursor, major, 0, position) {
            keep_two_candidates(&mut best, &mut following, candidate);
        }
    }
    let mut path = dx_probe(volume, inode, map_root, *major)?;
    loop {
        let logical = path.leaf;
        let buffer = directory_block(volume, inode, map_root, logical)?;
        buffer.with_read(|data| {
            let layout = verify_directory(g, inode, logical, data, false)?;
            let DirectoryLayout::Leaf { end } = layout else {
                return Err(FsError::Corrupt);
            };
            let mut offset = 0usize;
            while offset < end {
                let entry = record(g, data, offset, end)?;
                if entry.number != 0 {
                    let name = &data[entry.name_offset..entry.name_offset + entry.name_len];
                    let (major, minor) = codec::directory_hash(g, version, name)?;
                    let position = DirectoryPosition {
                        logical,
                        offset: offset as u32,
                    };
                    let candidate = HashCandidate {
                        major,
                        minor,
                        entry: FoundEntry {
                            logical,
                            offset,
                            number: entry.number,
                            name_len: entry.name_len,
                            kind: entry.kind,
                        },
                    };
                    if follows_cursor(cursor, major, minor, position) {
                        keep_two_candidates(&mut best, &mut following, candidate);
                    }
                }
                offset = entry.next;
            }
            Ok::<_, FsError>(())
        })??;
        let Some(lower_bound) = next_dx_leaf(
            volume,
            inode,
            map_root,
            &mut path,
            following.map(|next| next.major),
        )?
        else {
            break;
        };
        // continuation 位清除后仍等于候选 major，minor 与真实位置尚未排尽，须扫描整个哈希桶。
        if following.is_some_and(|next| next.major < lower_bound) {
            break;
        }
    }
    let Some(candidate) = best else {
        return Ok(None);
    };
    let mut name = Vec::new();
    name.try_reserve_exact(candidate.entry.name_len)
        .map_err(|_| FsError::NoMemory)?;
    let buffer = directory_block(volume, inode, map_root, candidate.entry.logical)?;
    buffer.with_read(|data| {
        let entry = record(
            g,
            data,
            candidate.entry.offset,
            if candidate.entry.logical == 0 {
                data.len()
            } else {
                match verify_directory(g, inode, candidate.entry.logical, data, false)? {
                    DirectoryLayout::Leaf { end } => end,
                    _ => return Err(FsError::Corrupt),
                }
            },
        )?;
        name.extend_from_slice(&data[entry.name_offset..entry.name_offset + entry.name_len]);
        Ok::<_, FsError>(())
    })??;
    let next_cursor = match following {
        Some(next) => DirectoryCursor::Indexed {
            major: next.major,
            minor: next.minor,
            collision: if next.major == candidate.major && next.minor == candidate.minor {
                Some(DirectoryPosition {
                    logical: candidate.entry.logical,
                    offset: candidate.entry.offset as u32,
                })
            } else {
                None
            },
        },
        None => DirectoryCursor::Indexed {
            major: ((DX_EOF_COOKIE >> 32) as u32) << 1,
            minor: DX_EOF_COOKIE as u32,
            collision: None,
        },
    };
    let next_cookie = next_cursor.cookie();
    Ok(Some(DirectoryEntry {
        number: candidate.entry.number,
        name,
        kind: candidate.entry.kind,
        next_cookie,
        next_cursor,
    }))
}

/// 当前后端布局下返回下一项；升为 HTree 后沿用旧游标的 cookie 数值。
pub(crate) fn next(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    cursor: &DirectoryCursor,
) -> Result<Option<DirectoryEntry>, FsError> {
    check_directory(volume, inode)?;
    if matches!(cursor, DirectoryCursor::Memory(_)) {
        return Err(FsError::InvalidOperation);
    }
    if inode.flags & INDEX != 0 {
        let cursor = match cursor {
            DirectoryCursor::Byte(cookie) => indexed_cursor(*cookie)?,
            cursor => *cursor,
        };
        return next_indexed(volume, inode, map_root, &cursor);
    }
    let DirectoryCursor::Byte(mut offset) = *cursor else {
        return Err(FsError::InvalidOperation);
    };
    let g = volume.geometry();
    while offset < inode.size {
        let logical = offset / g.block_size as u64;
        let base = logical * g.block_size as u64;
        let buffer = directory_block(volume, inode, map_root, logical)?;
        let entry = buffer.with_read(|data| {
            let layout = verify_directory(g, inode, logical, data, false)?;
            scan_next(g, data, layout, (offset - base) as usize)
        })??;
        if let Some(entry) = entry {
            let mut name = Vec::new();
            name.try_reserve_exact(entry.name_len)
                .map_err(|_| FsError::NoMemory)?;
            buffer.with_read(|data| {
                name.extend_from_slice(&data[entry.name_offset..entry.name_offset + entry.name_len])
            })?;
            let next_cursor = DirectoryCursor::Byte(base + entry.next as u64);
            return Ok(Some(DirectoryEntry {
                number: entry.number,
                name,
                kind: entry.kind,
                next_cookie: next_cursor.cookie(),
                next_cursor,
            }));
        }
        offset = base + g.block_size as u64;
    }
    Ok(None)
}
struct DirectoryEditBuilder<'a> {
    volume: &'a Ext4Volume,
    inode: &'a InodeMetadata,
    map_root: &'a [u8; 60],
    images: Vec<DirectoryImage>,
    logical_blocks: Vec<u64>,
    next_logical: u64,
    next_flags: u32,
}

impl<'a> DirectoryEditBuilder<'a> {
    /// 创建只保存当前目录局部块图像的准备对象，不分配磁盘块。
    fn new(volume: &'a Ext4Volume, inode: &'a InodeMetadata, map_root: &'a [u8; 60]) -> Self {
        Self {
            volume,
            inode,
            map_root,
            images: Vec::new(),
            logical_blocks: Vec::new(),
            next_logical: inode.size / volume.geometry().block_size as u64,
            next_flags: inode.flags,
        }
    }

    /// 获取一次目录编辑内共享的可变块图像。
    fn image(&mut self, logical: u64) -> Result<&mut DirectoryImage, FsError> {
        if let Some(index) = self
            .images
            .iter()
            .position(|image| image.logical == logical)
        {
            return Ok(&mut self.images[index]);
        }
        let image = read_image(self.volume, self.inode, self.map_root, logical)?;
        self.images.try_reserve(1).map_err(|_| FsError::NoMemory)?;
        self.images.push(image);
        Ok(self.images.last_mut().expect("reserved directory image"))
    }

    /// 预留一个尚未映射的新目录逻辑块号。
    fn reserve_logical(&mut self) -> Result<u64, FsError> {
        let logical = self.next_logical;
        if u64::from(DX_BLOCK_MASK) < logical {
            return Err(FsError::FileTooLarge);
        }
        self.next_logical = logical.checked_add(1).ok_or(FsError::FileTooLarge)?;
        self.logical_blocks
            .try_reserve(1)
            .map_err(|_| FsError::NoMemory)?;
        self.logical_blocks.push(logical);
        Ok(logical)
    }

    /// 保存已构造的新块图像，物理窗口在整批映射准备后再取得。
    fn push_new_image(&mut self, logical: u64, bytes: Vec<u8>) -> Result<(), FsError> {
        if bytes.len() != self.volume.geometry().block_size
            || !self.logical_blocks.contains(&logical)
        {
            return Err(FsError::Corrupt);
        }
        self.images.try_reserve(1).map_err(|_| FsError::NoMemory)?;
        self.images.push(DirectoryImage {
            logical,
            buffer: None,
            bytes,
        });
        Ok(())
    }

    /// 准备目录叶、索引和块树，失败时仍未修改任何共同字节。
    fn finish(self) -> Result<PreparedDirectoryEdit, FsError> {
        let Self {
            volume,
            inode,
            map_root,
            images,
            logical_blocks,
            next_logical,
            next_flags,
        } = self;
        if logical_blocks.is_empty() {
            if images.iter().any(|image| image.buffer.is_none()) {
                return Err(FsError::Corrupt);
            }
            return Ok(PreparedDirectoryEdit {
                mapping: None,
                images,
                logical_blocks,
                root: *map_root,
                size: inode.size,
                blocks: inode.blocks,
                flags: next_flags,
            });
        }
        let inode_data = inode.with_block(map_root);
        let mapping = volume.prepare_directory_mappings(&inode_data, map_root, &logical_blocks)?;
        let size = match next_logical.checked_mul(volume.geometry().block_size as u64) {
            Some(size) => size,
            None => {
                cancel_unpublished_mappings(volume, mapping, &logical_blocks);
                return Err(FsError::FileTooLarge);
            }
        };
        let sectors = match mapping
            .added_blocks()
            .checked_mul((volume.geometry().block_size / SECTOR_BYTES as usize) as u64)
        {
            Some(sectors) => sectors,
            None => {
                cancel_unpublished_mappings(volume, mapping, &logical_blocks);
                return Err(FsError::FileTooLarge);
            }
        };
        let blocks = match inode.blocks.checked_add(sectors) {
            Some(blocks) => blocks,
            None => {
                cancel_unpublished_mappings(volume, mapping, &logical_blocks);
                return Err(FsError::FileTooLarge);
            }
        };
        let mut images = images;
        let mut failure = None;
        for image in &mut images {
            if image.buffer.is_some() {
                continue;
            }
            let Some(physical) = mapping.physical(image.logical) else {
                failure = Some(FsError::Corrupt);
                break;
            };
            match volume.metadata_block(physical) {
                Ok(buffer) => image.buffer = Some(buffer),
                Err(error) => {
                    failure = Some(error);
                    break;
                }
            }
        }
        if let Some(error) = failure {
            cancel_unpublished_mappings(volume, mapping, &logical_blocks);
            return Err(error);
        }
        let root = *mapping.root();
        Ok(PreparedDirectoryEdit {
            mapping: Some(mapping),
            images,
            logical_blocks,
            root,
            flags: next_flags,
            size,
            blocks,
        })
    }
}

/// 复制一个已存在目录块的当前缓存字节，后续只提交本块的局部变更。
fn read_image(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
) -> Result<DirectoryImage, FsError> {
    let buffer = directory_block(volume, inode, map_root, logical)?;
    let size = volume.geometry().block_size;
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(size)
        .map_err(|_| FsError::NoMemory)?;
    bytes.resize(size, 0);
    buffer.with_read(|data| {
        bytes.copy_from_slice(codec::bytes(data, 0, size)?);
        Ok::<_, FsError>(())
    })??;
    Ok(DirectoryImage {
        logical,
        buffer: Some(buffer),
        bytes,
    })
}

/// 为一个全新 ext4 目录块准备明确大小的零图像。
fn empty_block(size: usize) -> Result<Vec<u8>, FsError> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(size)
        .map_err(|_| FsError::NoMemory)?;
    bytes.resize(size, 0);
    Ok(bytes)
}

/// 返回含名称和固定八字节头的最小四字节对齐 dirent 长度。
fn minimum_record_len(name_len: usize) -> usize {
    (DIRENT_HEADER_BYTES + name_len + DIRENT_ROUND) & !DIRENT_ROUND
}

/// 将受支持 inode 类型编码成 ext4 的原生 file_type。
fn directory_file_type(kind: InodeKind) -> u8 {
    match kind {
        InodeKind::Regular => 1,
        InodeKind::Directory => 2,
        InodeKind::Character => 3,
        InodeKind::Block => 4,
        InodeKind::Fifo => 5,
        InodeKind::Socket => 6,
        InodeKind::Symlink => 7,
        InodeKind::Other => 0,
    }
}

/// 在准备好的槽中写入完整原始名字和准确记录长度。
fn write_record(
    g: &Geometry,
    data: &mut [u8],
    offset: usize,
    rec_len: usize,
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<(), FsError> {
    let minimum = minimum_record_len(name.len());
    if rec_len < minimum
        || rec_len & DIRENT_ROUND != 0
        || (u16::MAX as usize) < rec_len
        || (u32::MAX as u64) < number
    {
        return Err(FsError::Corrupt);
    }
    let end = offset.checked_add(rec_len).ok_or(FsError::Corrupt)?;
    data.get_mut(offset..end).ok_or(FsError::Corrupt)?.fill(0);
    codec::put32(data, offset, number as u32);
    codec::put16(data, offset + 4, rec_len as u16);
    if g.features_incompat & codec::FEATURE_INCOMPAT_FILETYPE != 0 {
        data[offset + 6] = name.len() as u8;
        data[offset + 7] = directory_file_type(kind);
    } else {
        codec::put16(data, offset + 6, name.len() as u16);
    }
    data[offset + 8..offset + 8 + name.len()].copy_from_slice(name);
    Ok(())
}

/// 在已有叶块的空记录或有效记录尾部插入 dirent，不移动其他名字。
fn insert_record(
    g: &Geometry,
    inode: &InodeMetadata,
    data: &mut [u8],
    end: usize,
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<bool, FsError> {
    let needed = minimum_record_len(name.len());
    let mut offset = 0usize;
    while offset < end {
        let entry = record(g, data, offset, end)?;
        let current_len = entry.next - offset;
        if entry.number == 0 {
            if needed <= current_len {
                write_record(g, data, offset, current_len, name, number, kind)?;
                codec::update_directory_checksum(g, inode, data)?;
                return Ok(true);
            }
        } else {
            let occupied = minimum_record_len(entry.name_len);
            if needed <= current_len - occupied {
                codec::put16(data, offset + 4, occupied as u16);
                write_record(
                    g,
                    data,
                    offset + occupied,
                    current_len - occupied,
                    name,
                    number,
                    kind,
                )?;
                codec::update_directory_checksum(g, inode, data)?;
                return Ok(true);
            }
        }
        offset = entry.next;
    }
    Ok(false)
}

/// 建立具有一个完整空闲记录和合法目录尾 CRC 的新叶块图像。
fn empty_leaf(g: &Geometry, inode: &InodeMetadata) -> Result<Vec<u8>, FsError> {
    let mut data = empty_block(g.block_size)?;
    let end = if g.features_ro_compat & METADATA_CSUM != 0 {
        g.block_size - codec::DIR_TAIL_BYTES
    } else {
        g.block_size
    };
    codec::put16(&mut data, 4, end as u16);
    codec::update_directory_checksum(g, inode, &mut data)?;
    Ok(data)
}

/// 校验一个新增、删除或移动所使用的单个原始组件。
fn check_edit_name(name: &[u8]) -> Result<(), FsError> {
    if name.is_empty()
        || NAME_MAX < name.len()
        || name.contains(&0)
        || name.contains(&b'/')
        || name == b"."
        || name == b".."
    {
        return Err(FsError::InvalidName);
    }
    Ok(())
}

/// 校验新目录项引用的 inode 号可以写入 ext4 的 32 位 dirent 字段。
fn check_inode_number(g: &Geometry, number: u64) -> Result<(), FsError> {
    if number == 0 || g.inodes_count < number || u64::from(u32::MAX) < number {
        return Err(FsError::InvalidOperation);
    }
    Ok(())
}

/// 返回普通目录数据区域在 checksum tail 之前的准确结束位置。
fn leaf_end(g: &Geometry) -> usize {
    g.block_size
        - if g.features_ro_compat & METADATA_CSUM != 0 {
            codec::DIR_TAIL_BYTES
        } else {
            0
        }
}

/// 删除目标记录，将空间并入前项；首项保留 rec_len 以维持后续记录位置。
fn remove_record(
    g: &Geometry,
    inode: &InodeMetadata,
    data: &mut [u8],
    end: usize,
    target: usize,
) -> Result<(), FsError> {
    let mut offset = 0usize;
    let mut previous = None;
    while offset < end {
        let entry = record(g, data, offset, end)?;
        if offset == target {
            if let Some(previous_offset) = previous {
                let length = entry.next - previous_offset;
                codec::put16(data, previous_offset + 4, length as u16);
                data[offset..entry.next].fill(0);
            } else {
                codec::put32(data, offset, 0);
                data[offset + 6..entry.next].fill(0);
            }
            codec::update_directory_checksum(g, inode, data)?;
            return Ok(());
        }
        previous = Some(offset);
        offset = entry.next;
    }
    Err(FsError::NotFound)
}

/// 替换已定位记录的 inode/type 字段，目录哈希和名字保持原样。
fn replace_record_inode(
    g: &Geometry,
    inode: &InodeMetadata,
    data: &mut [u8],
    end: usize,
    target: usize,
    expected: u64,
    number: u64,
    kind: InodeKind,
) -> Result<(), FsError> {
    let entry = record(g, data, target, end)?;
    if entry.number != expected {
        return Err(FsError::Corrupt);
    }
    codec::put32(data, target, number as u32);
    if g.features_incompat & codec::FEATURE_INCOMPAT_FILETYPE != 0 {
        data[target + 7] = directory_file_type(kind);
    }
    codec::update_directory_checksum(g, inode, data)
}

#[derive(Clone, Copy)]
struct HashedRecord {
    hash: u32,
    offset: usize,
    name_len: usize,
    rec_len: usize,
}

/// 收集一个叶的活动记录哈希和原始范围，名字仍由单个块图像借用。
fn hashed_records(
    g: &Geometry,
    data: &[u8],
    end: usize,
    hash_version: u8,
) -> Result<Vec<HashedRecord>, FsError> {
    let mut records = Vec::new();
    records
        .try_reserve_exact(end / DIRENT_MIN_BYTES)
        .map_err(|_| FsError::NoMemory)?;
    let mut offset = 0usize;
    while offset < end {
        let entry = record(g, data, offset, end)?;
        if entry.number != 0 {
            let name = &data[entry.name_offset..entry.name_offset + entry.name_len];
            let (hash, _) = codec::directory_hash(g, hash_version, name)?;
            records.push(HashedRecord {
                hash,
                offset,
                name_len: entry.name_len,
                rec_len: entry.next - offset,
            });
        }
        offset = entry.next;
    }
    records.sort_unstable_by_key(|entry| (entry.hash, entry.offset));
    Ok(records)
}

/// 将一组真实记录紧密写入新叶，并扩展末项覆盖剩余空间。
fn pack_leaf(
    g: &Geometry,
    inode: &InodeMetadata,
    source: &[u8],
    records: &[HashedRecord],
    end: usize,
    destination: &mut [u8],
) -> Result<(), FsError> {
    let mut offset = 0usize;
    for entry in records {
        let len = minimum_record_len(entry.name_len);
        let source_end = entry.offset.checked_add(len).ok_or(FsError::Corrupt)?;
        let destination_end = offset.checked_add(len).ok_or(FsError::Corrupt)?;
        destination
            .get_mut(offset..destination_end)
            .ok_or(FsError::Corrupt)?
            .copy_from_slice(
                source
                    .get(entry.offset..source_end)
                    .ok_or(FsError::Corrupt)?,
            );
        codec::put16(destination, offset + 4, len as u16);
        offset = destination_end;
    }
    if records.is_empty() {
        codec::put16(destination, 4, end as u16);
    } else {
        let last = offset - minimum_record_len(records.last().expect("nonempty leaf").name_len);
        codec::put16(destination, last + 4, (end - last) as u16);
    }
    codec::update_directory_checksum(g, inode, destination)
}

/// 计算 dx root 或普通 index-node 的可用条目容量。
fn dx_limit(g: &Geometry, root: bool) -> usize {
    let offset = if root {
        DX_ROOT_ENTRIES_OFFSET
    } else {
        DX_NODE_ENTRIES_OFFSET
    };
    let tail = if g.features_ro_compat & METADATA_CSUM != 0 {
        DX_TAIL_BYTES
    } else {
        0
    };
    (g.block_size - offset - tail) / DX_ENTRY_BYTES
}

/// 将已准备的 dx 条目和真实容量写入局部块图像并重算尾 CRC。
fn write_dx_node(
    builder: &mut DirectoryEditBuilder<'_>,
    logical: u64,
    root: bool,
    depth: u8,
    entries: &[DxEntry],
) -> Result<(), FsError> {
    let g = builder.volume.geometry();
    let inode = builder.inode;
    let offset = if root {
        DX_ROOT_ENTRIES_OFFSET
    } else {
        DX_NODE_ENTRIES_OFFSET
    };
    let limit = dx_limit(g, root);
    if entries.is_empty() || limit < entries.len() {
        return Err(FsError::NoSpace);
    }
    let image = builder.image(logical)?;
    if !root {
        image.bytes[..8].fill(0);
        codec::put16(&mut image.bytes, 4, g.block_size as u16);
    } else {
        image.bytes[30] = depth;
    }
    codec::put16(&mut image.bytes, offset, limit as u16);
    codec::put16(&mut image.bytes, offset + 2, entries.len() as u16);
    for (position, entry) in entries.iter().enumerate() {
        if position != 0 {
            codec::put32(
                &mut image.bytes,
                offset + position * DX_ENTRY_BYTES,
                entry.hash,
            );
        }
        codec::put32(
            &mut image.bytes,
            offset + position * DX_ENTRY_BYTES + 4,
            entry.block as u32,
        );
    }
    for position in entries.len()..limit {
        let start = offset + position * DX_ENTRY_BYTES;
        image.bytes[start..start + 8].fill(0);
    }
    if g.features_ro_compat & METADATA_CSUM != 0 {
        codec::update_dx_checksum(
            inode,
            &mut image.bytes,
            offset + entries.len() * DX_ENTRY_BYTES,
            offset + limit * DX_ENTRY_BYTES,
        )?;
    }
    Ok(())
}

struct DxNodeEdit {
    logical: u64,
    root: bool,
    position: usize,
    depth: u8,
    limit: usize,
    entries: Vec<DxEntry>,
}

/// 解码当前索引节点的条目到准备区，保留源块只在发布时修改。
fn load_dx_node(
    builder: &mut DirectoryEditBuilder<'_>,
    frame: DxFrame,
) -> Result<DxNodeEdit, FsError> {
    let g = builder.volume.geometry();
    let mut metadata = *builder.inode;
    metadata.flags = builder.next_flags;
    metadata.size = builder
        .next_logical
        .checked_mul(g.block_size as u64)
        .ok_or(FsError::FileTooLarge)?;
    let inode = &metadata;
    let image = builder.image(frame.logical)?;
    let layout = verify_directory(g, inode, frame.logical, &image.bytes, !frame.root)?;
    let expected = if frame.root {
        DirectoryLayout::Root
    } else {
        DirectoryLayout::Index
    };
    if layout != expected {
        return Err(FsError::Corrupt);
    }
    let offset = if frame.root {
        DX_ROOT_ENTRIES_OFFSET
    } else {
        DX_NODE_ENTRIES_OFFSET
    };
    let limit = usize::from(codec::le16(&image.bytes, offset)?);
    let count = usize::from(codec::le16(&image.bytes, offset + 2)?);
    if count != frame.count || count <= frame.position {
        return Err(FsError::Corrupt);
    }
    let mut entries = Vec::new();
    entries
        .try_reserve_exact(count)
        .map_err(|_| FsError::NoMemory)?;
    for position in 0..count {
        entries.push(dx_entry(&image.bytes, offset, position)?);
    }
    Ok(DxNodeEdit {
        logical: frame.logical,
        root: frame.root,
        position: frame.position,
        depth: if frame.root { image.bytes[30] } else { 0 },
        limit,
        entries,
    })
}

/// 在 dx 路径插入分裂边界，必要时分裂 index-node 或增长根深度。
fn insert_dx_block(
    builder: &mut DirectoryEditBuilder<'_>,
    path: &DxPath,
    hash: u32,
    block: u64,
) -> Result<(), FsError> {
    let mut level = path.len.checked_sub(1).ok_or(FsError::Corrupt)?;
    let mut addition = DxEntry { hash, block };
    loop {
        let frame = path.frames[level];
        let mut node = load_dx_node(builder, frame)?;
        let insertion = node.position + 1;
        node.entries.try_reserve(1).map_err(|_| FsError::NoMemory)?;
        node.entries.insert(insertion, addition);
        if node.entries.len() <= node.limit {
            return write_dx_node(builder, node.logical, node.root, node.depth, &node.entries);
        }
        if node.root {
            if dx_max_indirect_levels(builder.volume.geometry()) < usize::from(node.depth) + 1 {
                return Err(FsError::NoSpace);
            }
            let logical = builder.reserve_logical()?;
            let bytes = empty_block(builder.volume.geometry().block_size)?;
            builder.push_new_image(logical, bytes)?;
            write_dx_node(builder, logical, false, 0, &node.entries)?;
            let root = [DxEntry {
                hash: 0,
                block: logical,
            }];
            return write_dx_node(builder, node.logical, true, node.depth + 1, &root);
        }
        if level == 0 {
            return Err(FsError::Corrupt);
        }
        let split = node.entries.len() / 2;
        if split == 0 || split == node.entries.len() {
            return Err(FsError::NoSpace);
        }
        let promoted = node.entries[split].hash;
        let right = node.entries.split_off(split);
        write_dx_node(builder, node.logical, false, 0, &node.entries)?;
        let logical = builder.reserve_logical()?;
        let bytes = empty_block(builder.volume.geometry().block_size)?;
        builder.push_new_image(logical, bytes)?;
        write_dx_node(builder, logical, false, 0, &right)?;
        addition = DxEntry {
            hash: promoted,
            block: logical,
        };
        level -= 1;
    }
}

/// 分裂一个已满叶，按真实名字哈希分配记录并发表 continuation separator。
fn split_leaf(
    builder: &mut DirectoryEditBuilder<'_>,
    path: &DxPath,
    hash_version: u8,
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<(), FsError> {
    let g = builder.volume.geometry();
    let inode = builder.inode;
    let (hash, _) = codec::directory_hash(g, hash_version, name)?;
    let leaf = builder.image(path.leaf)?;
    let layout = verify_directory(g, inode, path.leaf, &leaf.bytes, false)?;
    let DirectoryLayout::Leaf { end } = layout else {
        return Err(FsError::Corrupt);
    };
    let records = hashed_records(g, &leaf.bytes, end, hash_version)?;
    if records.is_empty() {
        return Err(FsError::NoSpace);
    }
    let (split, boundary, continued) = if records.len() == 1 {
        let old_hash = records[0].hash;
        if old_hash <= hash {
            (1, hash, old_hash == hash)
        } else {
            (0, old_hash, false)
        }
    } else {
        let mut split = records.len();
        let mut right_size = 0usize;
        while 0 < split {
            let record = records[split - 1];
            if right_size + record.rec_len / 2 > end / 2 {
                break;
            }
            right_size += record.rec_len;
            split -= 1;
        }
        if split == 0 {
            split = records.len() / 2;
        }
        if split == 0 || records.len() <= split {
            return Err(FsError::NoSpace);
        }
        let boundary = records[split].hash;
        (split, boundary, records[split - 1].hash == boundary)
    };
    let new_logical = builder.reserve_logical()?;
    let mut left = empty_block(g.block_size)?;
    let mut right = empty_block(g.block_size)?;
    {
        let source = &builder.image(path.leaf)?.bytes;
        pack_leaf(g, inode, source, &records[..split], end, &mut left)?;
        pack_leaf(g, inode, source, &records[split..], end, &mut right)?;
    }
    let target = if hash < boundary {
        &mut left
    } else {
        &mut right
    };
    if !insert_record(g, inode, target, end, name, number, kind)? {
        return Err(FsError::NoSpace);
    }
    builder.image(path.leaf)?.bytes = left;
    builder.push_new_image(new_logical, right)?;
    insert_dx_block(builder, path, boundary + u32::from(continued), new_logical)
}

/// 将单块普通目录迁移为 HTree，原普通项移入真实叶后执行叶分裂。
fn make_indexed_directory(
    builder: &mut DirectoryEditBuilder<'_>,
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<(), FsError> {
    let g = builder.volume.geometry();
    let inode = builder.inode;
    let version = g.default_hash_version;
    let (parent_number, leaf) = {
        let root = builder.image(0)?;
        let layout = verify_directory(g, inode, 0, &root.bytes, false)?;
        let DirectoryLayout::Leaf { end } = layout else {
            return Err(FsError::Corrupt);
        };
        let dot = record(g, &root.bytes, 0, end)?;
        let parent = record(g, &root.bytes, 12, end)?;
        if dot.number != inode.number
            || dot.next != 12
            || dot.name_len != 1
            || &root.bytes[8..9] != b"."
            || parent.number == 0
            || parent.name_len != 2
            || &root.bytes[20..22] != b".."
        {
            return Err(FsError::Corrupt);
        }
        let mut records = hashed_records(g, &root.bytes, end, version)?;
        records.retain(|entry| {
            let name = &root.bytes[entry.offset + 8..entry.offset + 8 + entry.name_len];
            name != b"." && name != b".."
        });
        let mut leaf = empty_block(g.block_size)?;
        pack_leaf(g, inode, &root.bytes, &records, end, &mut leaf)?;
        (parent.number, leaf)
    };
    let leaf_logical = builder.reserve_logical()?;
    builder.push_new_image(leaf_logical, leaf)?;
    builder.next_flags |= INDEX;
    let mut root = empty_block(g.block_size)?;
    write_record(
        g,
        &mut root,
        0,
        12,
        b".",
        inode.number,
        InodeKind::Directory,
    )?;
    write_record(
        g,
        &mut root,
        12,
        g.block_size - 12,
        b"..",
        parent_number,
        InodeKind::Directory,
    )?;
    root[28] = version;
    root[29] = 8;
    root[30] = 0;
    root[31] = 0;
    builder.image(0)?.bytes = root;
    let root_entries = [DxEntry {
        hash: 0,
        block: leaf_logical,
    }];
    write_dx_node(builder, 0, true, 0, &root_entries)?;
    let mut frames = [DxFrame::default(); DX_MAX_INDIRECT_LEVELS + 1];
    frames[0] = DxFrame {
        logical: 0,
        root: true,
        position: 0,
        count: 1,
    };
    split_leaf(
        builder,
        &DxPath {
            frames,
            len: 1,
            leaf: leaf_logical,
            lower_bound: 0,
        },
        version,
        name,
        number,
        kind,
    )
}

/// 将名字写入既有目录叶，空间不足时沿当前 HTree 路径准备分裂。
fn add_to_builder(
    builder: &mut DirectoryEditBuilder<'_>,
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<(), FsError> {
    let g = builder.volume.geometry();
    let inode = builder.inode;
    if inode.flags & INDEX != 0 {
        let hash_version = root_hash_version(builder.volume, inode, builder.map_root)?;
        let (hash, _) = codec::directory_hash(g, hash_version, name)?;
        let path = dx_probe(builder.volume, inode, builder.map_root, hash)?;
        let inserted = {
            let image = builder.image(path.leaf)?;
            let layout = verify_directory(g, inode, path.leaf, &image.bytes, false)?;
            let DirectoryLayout::Leaf { end } = layout else {
                return Err(FsError::Corrupt);
            };
            insert_record(g, inode, &mut image.bytes, end, name, number, kind)?
        };
        if !inserted {
            split_leaf(builder, &path, hash_version, name, number, kind)?;
        }
        return Ok(());
    }
    for logical in 0..inode.size / g.block_size as u64 {
        let inserted = {
            let image = builder.image(logical)?;
            let layout = verify_directory(g, inode, logical, &image.bytes, false)?;
            let DirectoryLayout::Leaf { end } = layout else {
                return Err(FsError::Corrupt);
            };
            insert_record(g, inode, &mut image.bytes, end, name, number, kind)?
        };
        if inserted {
            return Ok(());
        }
    }
    if inode.size == g.block_size as u64 && g.features_compat & codec::FEATURE_COMPAT_DIR_INDEX != 0
    {
        return make_indexed_directory(builder, name, number, kind);
    }
    let logical = builder.reserve_logical()?;
    let mut data = empty_leaf(g, inode)?;
    if !insert_record(g, inode, &mut data, leaf_end(g), name, number, kind)? {
        return Err(FsError::Corrupt);
    }
    builder.push_new_image(logical, data)
}

/// 先完成原始字节名冲突检查，再准备目录项及全部新增 HTree 映射。
pub(crate) fn prepare_add(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<PreparedDirectoryEdit, FsError> {
    check_directory(volume, inode)?;
    check_edit_name(name)?;
    check_inode_number(volume.geometry(), number)?;
    if find_entry(volume, inode, map_root, name)?.is_some() {
        return Err(FsError::Exists);
    }
    let mut builder = DirectoryEditBuilder::new(volume, inode, map_root);
    add_to_builder(&mut builder, name, number, kind)?;
    builder.finish()
}

/// 删除父目录中这个名字的唯一 dirent，不释放目标 inode 或目录块。
pub(crate) fn prepare_remove(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    name: &[u8],
) -> Result<PreparedDirectoryEdit, FsError> {
    check_directory(volume, inode)?;
    check_edit_name(name)?;
    let found = find_entry(volume, inode, map_root, name)?.ok_or(FsError::NotFound)?;
    let mut builder = DirectoryEditBuilder::new(volume, inode, map_root);
    let g = volume.geometry();
    let image = builder.image(found.logical)?;
    let layout = verify_directory(g, inode, found.logical, &image.bytes, false)?;
    let DirectoryLayout::Leaf { end } = layout else {
        return Err(FsError::Corrupt);
    };
    remove_record(g, inode, &mut image.bytes, end, found.offset)?;
    builder.finish()
}

/// 替换同一父写侧重新查到的现有目录项 inode 身份。
pub(crate) fn prepare_replace(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<PreparedDirectoryEdit, FsError> {
    check_directory(volume, inode)?;
    check_edit_name(name)?;
    check_inode_number(volume.geometry(), number)?;
    let found = find_entry(volume, inode, map_root, name)?.ok_or(FsError::NotFound)?;
    let mut builder = DirectoryEditBuilder::new(volume, inode, map_root);
    let g = volume.geometry();
    let image = builder.image(found.logical)?;
    let layout = verify_directory(g, inode, found.logical, &image.bytes, false)?;
    let DirectoryLayout::Leaf { end } = layout else {
        return Err(FsError::Corrupt);
    };
    replace_record_inode(
        g,
        inode,
        &mut image.bytes,
        end,
        found.offset,
        found.number,
        number,
        kind,
    )?;
    builder.finish()
}

/// 同父原生 rename 同时准备 source 删除和 destination 安装或替换。
pub(crate) fn prepare_move(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    old_name: &[u8],
    new_name: &[u8],
    source_number: u64,
    source_kind: InodeKind,
    target: Option<(u64, InodeKind)>,
) -> Result<PreparedDirectoryEdit, FsError> {
    check_directory(volume, inode)?;
    check_edit_name(old_name)?;
    check_edit_name(new_name)?;
    check_inode_number(volume.geometry(), source_number)?;
    let source = find_entry(volume, inode, map_root, old_name)?.ok_or(FsError::NotFound)?;
    if source.number != source_number {
        return Err(FsError::Corrupt);
    }
    let existing = find_entry(volume, inode, map_root, new_name)?;
    if old_name == new_name {
        if target.is_some_and(|(expected, _)| expected != source_number) {
            return Err(FsError::Corrupt);
        }
        return DirectoryEditBuilder::new(volume, inode, map_root).finish();
    }
    match (target, existing) {
        (None, Some(_)) => return Err(FsError::Exists),
        (Some((expected, _)), Some(found)) if found.number == expected => {}
        (Some(_), Some(_)) => return Err(FsError::Corrupt),
        (Some(_), None) => return Err(FsError::NotFound),
        (None, None) => {}
    }
    if existing.is_some_and(|entry| entry.number == source_number) {
        return DirectoryEditBuilder::new(volume, inode, map_root).finish();
    }
    let mut builder = DirectoryEditBuilder::new(volume, inode, map_root);
    let g = volume.geometry();
    if let Some((target_number, _)) = target {
        let target_entry = existing.ok_or(FsError::NotFound)?;
        {
            let image = builder.image(target_entry.logical)?;
            let layout = verify_directory(g, inode, target_entry.logical, &image.bytes, false)?;
            let DirectoryLayout::Leaf { end } = layout else {
                return Err(FsError::Corrupt);
            };
            replace_record_inode(
                g,
                inode,
                &mut image.bytes,
                end,
                target_entry.offset,
                target_number,
                source_number,
                source_kind,
            )?;
        }
        let image = builder.image(source.logical)?;
        let layout = verify_directory(g, inode, source.logical, &image.bytes, false)?;
        let DirectoryLayout::Leaf { end } = layout else {
            return Err(FsError::Corrupt);
        };
        remove_record(g, inode, &mut image.bytes, end, source.offset)?;
    } else {
        {
            let image = builder.image(source.logical)?;
            let layout = verify_directory(g, inode, source.logical, &image.bytes, false)?;
            let DirectoryLayout::Leaf { end } = layout else {
                return Err(FsError::Corrupt);
            };
            remove_record(g, inode, &mut image.bytes, end, source.offset)?;
        }
        add_to_builder(&mut builder, new_name, source_number, source_kind)?;
    }
    builder.finish()
}

/// 更新真实根块中的 `..` inode，不修改目录 HTree 叶或索引。
pub(crate) fn prepare_dotdot(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    parent_number: u64,
) -> Result<PreparedDirectoryEdit, FsError> {
    check_directory(volume, inode)?;
    check_inode_number(volume.geometry(), parent_number)?;
    let found = find_entry(volume, inode, map_root, b"..")?.ok_or(FsError::Corrupt)?;
    if found.logical != 0 {
        return Err(FsError::Corrupt);
    }
    let mut builder = DirectoryEditBuilder::new(volume, inode, map_root);
    let image = builder.image(0)?;
    let g = volume.geometry();
    codec::put32(&mut image.bytes, found.offset, parent_number as u32);
    if inode.flags & INDEX != 0 && g.features_ro_compat & METADATA_CSUM != 0 {
        let limit = usize::from(codec::le16(&image.bytes, 32)?);
        let count = usize::from(codec::le16(&image.bytes, 34)?);
        codec::update_dx_checksum(
            inode,
            &mut image.bytes,
            DX_ROOT_ENTRIES_OFFSET + count * DX_ENTRY_BYTES,
            DX_ROOT_ENTRIES_OFFSET + limit * DX_ENTRY_BYTES,
        )?;
    } else {
        codec::update_directory_checksum(g, inode, &mut image.bytes)?;
    }
    builder.finish()
}

/// 验证单个普通叶只包含它所在位置允许的 `.` 与 `..`。
fn directory_leaf_empty(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    logical: u64,
) -> Result<bool, FsError> {
    let buffer = directory_block(volume, inode, map_root, logical)?;
    let g = volume.geometry();
    buffer.with_read(|data| {
        let layout = verify_directory(g, inode, logical, data, false)?;
        let DirectoryLayout::Leaf { end } = layout else {
            return Err(FsError::Corrupt);
        };
        let mut offset = 0usize;
        while offset < end {
            let entry = record(g, data, offset, end)?;
            if entry.number != 0 {
                let name = &data[entry.name_offset..entry.name_offset + entry.name_len];
                if name == b"." || name == b".." {
                    if inode.flags & INDEX != 0
                        || logical != 0
                        || (name == b"." && offset != 0)
                        || (name == b".." && offset != 12)
                    {
                        return Err(FsError::Corrupt);
                    }
                } else {
                    return Ok(false);
                }
            }
            offset = entry.next;
        }
        Ok::<_, FsError>(true)
    })?
}

/// 检查真实 `.`、`..` 与所有叶项，空目录不依赖 dcache 快照。
pub(crate) fn is_empty(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
) -> Result<bool, FsError> {
    check_directory(volume, inode)?;
    let root = directory_block(volume, inode, map_root, 0)?;
    let g = volume.geometry();
    root.with_read(|data| {
        let layout = verify_directory(g, inode, 0, data, false)?;
        let expected_end = if inode.flags & INDEX != 0 {
            data.len()
        } else {
            leaf_end(g)
        };
        let dot = record(g, data, 0, expected_end)?;
        let parent = record(g, data, 12, expected_end)?;
        let valid_layout = if inode.flags & INDEX != 0 {
            layout == DirectoryLayout::Root
        } else {
            matches!(layout, DirectoryLayout::Leaf { end } if end == expected_end)
        };
        if !valid_layout
            || dot.number != inode.number
            || dot.next != 12
            || dot.name_len != 1
            || data[8] != b'.'
            || (inode.flags & INDEX != 0 && parent.next != expected_end)
            || parent.name_len != 2
            || &data[20..22] != b".."
            || parent.number == 0
        {
            return Err(FsError::Corrupt);
        }
        Ok::<_, FsError>(())
    })??;
    if inode.flags & INDEX != 0 {
        let mut empty = true;
        for_each_leaf(volume, inode, map_root, |logical| {
            if !directory_leaf_empty(volume, inode, map_root, logical)? {
                empty = false;
                return Ok(false);
            }
            Ok(true)
        })?;
        return Ok(empty);
    }
    for logical in 0..inode.size / g.block_size as u64 {
        if !directory_leaf_empty(volume, inode, map_root, logical)? {
            return Ok(false);
        }
    }
    Ok(true)
}

/// 为未链接的新目录准备真实 dot 项、初始叶和可选 dx 根。
pub(crate) fn prepare_new_directory(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    parent_number: u64,
) -> Result<PreparedDirectoryEdit, FsError> {
    let g = volume.geometry();
    if inode.kind != InodeKind::Directory || inode.size != 0 || inode.blocks != 0 {
        return Err(FsError::InvalidOperation);
    }
    check_inode_number(g, inode.number)?;
    check_inode_number(g, parent_number)?;
    let mut builder = DirectoryEditBuilder::new(volume, inode, map_root);
    if inode.flags & INDEX != 0 {
        if g.features_compat & codec::FEATURE_COMPAT_DIR_INDEX == 0 {
            return Err(FsError::Corrupt);
        }
        let version = g.default_hash_version;
        let root_logical = builder.reserve_logical()?;
        let leaf_logical = builder.reserve_logical()?;
        let mut root = empty_block(g.block_size)?;
        write_record(
            g,
            &mut root,
            0,
            12,
            b".",
            inode.number,
            InodeKind::Directory,
        )?;
        write_record(
            g,
            &mut root,
            12,
            g.block_size - 12,
            b"..",
            parent_number,
            InodeKind::Directory,
        )?;
        root[28] = version;
        root[29] = 8;
        root[30] = 0;
        root[31] = 0;
        builder.push_new_image(root_logical, root)?;
        let root_entries = [DxEntry {
            hash: 0,
            block: leaf_logical,
        }];
        write_dx_node(&mut builder, root_logical, true, 0, &root_entries)?;
        builder.push_new_image(leaf_logical, empty_leaf(g, inode)?)?;
        // 根块指向叶时，先让叶字节完成发表。
        builder.images.swap(0, 1);
    } else {
        let root_logical = builder.reserve_logical()?;
        let mut root = empty_block(g.block_size)?;
        let end = leaf_end(g);
        write_record(
            g,
            &mut root,
            0,
            12,
            b".",
            inode.number,
            InodeKind::Directory,
        )?;
        write_record(
            g,
            &mut root,
            12,
            end - 12,
            b"..",
            parent_number,
            InodeKind::Directory,
        )?;
        codec::update_directory_checksum(g, inode, &mut root)?;
        builder.push_new_image(root_logical, root)?;
    }
    builder.finish()
}
