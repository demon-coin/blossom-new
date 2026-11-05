//! extent 与传统间接树只在内容借用中解码，树下降和真实 I/O 在借用之外。

use crate::block::MetadataIoContext;
use crate::cache::BufferRef;
use crate::fs::ext4::allocation::DetachedBlock;
use crate::fs::ext4::codec::{self, EXTENTS};
use crate::fs::ext4::{BlockMapping, Ext4Volume, Geometry, InodeData, UnpublishedBlock};
use crate::fs::{FsError, InodeKind};
use alloc::vec::Vec;
use core::convert::{TryFrom, TryInto};

#[derive(Clone, Copy)]
enum Step {
    Mapping(BlockMapping),
    Child {
        physical: u64,
        depth: u16,
        low: u64,
        high: u64,
    },
}

/// 校验 inode 内根节点或全部传统块指针，不读取额外节点。
pub(crate) fn validate_root(g: &Geometry, inode: &InodeData) -> Result<(), FsError> {
    if inode.flags & EXTENTS != 0 {
        extent_step(
            g,
            inode,
            &inode.block,
            0,
            None,
            None,
            codec::EXTENT_LOGICAL_BLOCKS,
        )?;
    } else {
        validate_pointers(g, &inode.block)?;
    }
    Ok(())
}

/// 依据文件的实际映射格式选择有限树算法。
pub(crate) fn map(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    end: u64,
    context: Option<MetadataIoContext<'_>>,
) -> Result<BlockMapping, FsError> {
    let mapping = if inode.flags & EXTENTS != 0 {
        extent_map(volume, inode, logical, context)?
    } else {
        indirect_map(volume, inode, logical, context)?
    };
    Ok(match mapping {
        BlockMapping::Mapped { physical, len } => BlockMapping::Mapped {
            physical,
            len: len.min(end - logical),
        },
        BlockMapping::Unwritten { physical, len } => BlockMapping::Unwritten {
            physical,
            len: len.min(end - logical),
        },
        BlockMapping::Hole { len } => BlockMapping::Hole {
            len: len.min(end - logical),
        },
        BlockMapping::Eof => return Err(FsError::Corrupt),
    })
}

/// 逐层下降 extent 树，每个已读取的外部节点先校验自身 CRC 和全部条目。
fn extent_map(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    context: Option<MetadataIoContext<'_>>,
) -> Result<BlockMapping, FsError> {
    if codec::EXTENT_LOGICAL_BLOCKS <= logical {
        return Err(FsError::Corrupt);
    }

    let g = volume.geometry();
    let mut step = extent_step(
        g,
        inode,
        &inode.block,
        logical,
        None,
        None,
        codec::EXTENT_LOGICAL_BLOCKS,
    )?;
    let mut visited = [0u64; codec::EXTENT_DEPTH_LIMIT];
    let mut count = 0usize;
    loop {
        match step {
            Step::Mapping(mapping) => return Ok(mapping),
            Step::Child {
                physical,
                depth,
                low,
                high,
            } => {
                if visited.len() <= count || visited[..count].contains(&physical) {
                    return Err(FsError::Corrupt);
                }
                visited[count] = physical;
                count += 1;
                g.validate_blocks(physical, 1)?;
                let buffer = volume
                    .metadata_cache()
                    .get(physical, g.block_size, context)?;
                step = buffer.with_read(|data| {
                    extent_step(g, inode, data, logical, Some(depth), Some(low), high)
                })??;
            }
        }
    }
}

/// 解码一个节点并选取下一层，完整验证条目排序、父边界及物理范围。
fn extent_step(
    g: &Geometry,
    inode: &InodeData,
    data: &[u8],
    logical: u64,
    depth: Option<u16>,
    low: Option<u64>,
    high: u64,
) -> Result<Step, FsError> {
    let header = codec::extent_header(data, depth)?;
    if depth.is_some() {
        codec::verify_extent_checksum(g, inode, data, header)?;
    }

    let mut previous_end = low.unwrap_or(0);
    let mut selected = None;
    let mut next_start = high;
    for index in 0..usize::from(header.entries) {
        let offset = codec::EXTENT_HEADER_BYTES + index * codec::EXTENT_ENTRY_BYTES;
        let start = u64::from(codec::le32(data, offset)?);
        if start < previous_end
            || high <= start
            || (index == 0 && low.is_some_and(|low| low != start))
        {
            return Err(FsError::Corrupt);
        }
        if header.depth == 0 {
            let (len, unwritten) = codec::extent_length(codec::le16(data, offset + 4)?)?;
            let end = start.checked_add(len).ok_or(FsError::Corrupt)?;
            if high < end {
                return Err(FsError::Corrupt);
            }
            let physical = u64::from(codec::le32(data, offset + 8)?)
                | (u64::from(codec::le16(data, offset + 6)?) << 32);
            g.validate_blocks(physical, len)?;
            if start <= logical && logical < end {
                let physical = physical + logical - start;
                selected = Some(Step::Mapping(if unwritten {
                    BlockMapping::Unwritten {
                        physical,
                        len: end - logical,
                    }
                } else {
                    BlockMapping::Mapped {
                        physical,
                        len: end - logical,
                    }
                }));
            }
            previous_end = end;
        } else {
            let physical = u64::from(codec::le32(data, offset + 4)?)
                | (u64::from(codec::le16(data, offset + 8)?) << 32);
            g.validate_blocks(physical, 1)?;
            if start <= logical {
                selected = Some(Step::Child {
                    physical,
                    depth: header.depth - 1,
                    low: start,
                    high,
                });
            } else if let Some(Step::Child {
                high: selected_high,
                ..
            }) = &mut selected
            {
                *selected_high = (*selected_high).min(start);
            }
            previous_end = start + 1;
        }
        if logical < start {
            next_start = next_start.min(start);
        }
    }

    Ok(selected.unwrap_or(Step::Mapping(BlockMapping::Hole {
        len: next_start - logical,
    })))
}

/// 校验传统指针数组中每个非零地址，零指针保持合法空洞含义。
fn validate_pointers(g: &Geometry, data: &[u8]) -> Result<(), FsError> {
    if data.len() % 4 != 0 {
        return Err(FsError::Corrupt);
    }
    for offset in (0..data.len()).step_by(4) {
        let physical = u64::from(codec::le32(data, offset)?);
        if physical != 0 {
            g.validate_blocks(physical, 1)?;
        }
    }
    Ok(())
}

/// 传统 direct、single、double、triple 树只下降最多三次，并拒绝祖先自引用。
fn indirect_map(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    context: Option<MetadataIoContext<'_>>,
) -> Result<BlockMapping, FsError> {
    let g = volume.geometry();
    validate_pointers(g, &inode.block)?;
    if logical < codec::DIRECT_BLOCKS as u64 {
        let physical = u64::from(codec::le32(&inode.block, logical as usize * 4)?);
        return Ok(if physical == 0 {
            BlockMapping::Hole { len: 1 }
        } else {
            BlockMapping::Mapped { physical, len: 1 }
        });
    }

    let pointers = (g.block_size / 4) as u64;
    let mut remaining = logical - codec::DIRECT_BLOCKS as u64;
    let mut span = pointers;
    let mut depth = 1usize;
    while span <= remaining && depth < codec::INDIRECT_LEVELS {
        remaining -= span;
        span = span.checked_mul(pointers).ok_or(FsError::Corrupt)?;
        depth += 1;
    }
    if span <= remaining {
        return Err(FsError::Corrupt);
    }

    let mut physical = u64::from(codec::le32(
        &inode.block,
        (codec::DIRECT_BLOCKS - 1 + depth) * 4,
    )?);
    let mut visited = [0u64; codec::INDIRECT_LEVELS];
    for level in 0..depth {
        if physical == 0 {
            return Ok(BlockMapping::Hole {
                len: span - remaining,
            });
        }
        if visited[..level].contains(&physical) {
            return Err(FsError::Corrupt);
        }
        visited[level] = physical;
        g.validate_blocks(physical, 1)?;
        let buffer = volume
            .metadata_cache()
            .get(physical, g.block_size, context)?;
        span /= pointers;
        let index = remaining / span;
        remaining %= span;
        physical = buffer.with_read(|data| {
            validate_pointers(g, data)?;
            codec::le32(data, index as usize * 4).map(u64::from)
        })??;
    }
    if physical == 0 {
        Ok(BlockMapping::Hole { len: 1 })
    } else if visited[..depth].contains(&physical) {
        Err(FsError::Corrupt)
    } else {
        Ok(BlockMapping::Mapped { physical, len: 1 })
    }
}

/// 一个 inode 的块树修改计划；只保存条目和字段，不保存可覆盖共同页的旧块快照。
pub(crate) struct PreparedMapping {
    pub root: [u8; 60],
    pub physical: u64,
    pub zero: bool,
    pub added_blocks: u64,
    pub removed_blocks: u64,
    retired: Vec<DetachedBlock>,
    nodes: Vec<NodeEdit>,
    indirect: Vec<IndirectEdit>,
    allocated: Vec<UnpublishedBlock>,
}

struct NodeEdit {
    buffer: Option<BufferRef>,
    physical: u64,
    entries: Vec<[u8; 12]>,
    max: usize,
    depth: u16,
    generation: u32,
    dirty: bool,
    new: bool,
}

struct IndirectEdit {
    buffer: BufferRef,
    physical: u64,
    offset: usize,
    pointer: u32,
    initialize: bool,
}

/// 目录一次扩展所需的新逻辑块映射，所有根和块 token 同属一个准备。
pub(crate) struct PreparedDirectoryMappings {
    mapping: PreparedMapping,
    logical: Vec<u64>,
    physical: Vec<u64>,
    seed: u32,
}

impl PreparedDirectoryMappings {
    /// 返回批次中逻辑目录块对应的实际物理块。
    pub(crate) fn physical(&self, logical: u64) -> Option<u64> {
        self.logical
            .iter()
            .position(|candidate| *candidate == logical)
            .map(|index| self.physical[index])
    }

    /// 返回完成整批修改后的 inline inode 块根。
    pub(crate) fn root(&self) -> &[u8; 60] {
        &self.mapping.root
    }

    /// 返回数据块和 extent/indirect 节点的总文件系统块数。
    pub(crate) fn added_blocks(&self) -> u64 {
        self.mapping.added_blocks
    }

    /// 目录块内容提交后接入全部映射节点，并消费所有未发表块位。
    pub(crate) fn publish(self, volume: &Ext4Volume) {
        let retired = self.mapping.publish(volume, self.seed);
        assert!(retired.is_empty(), "directory mapping retired old blocks");
    }

    /// 目录准备失败时只归还本批次仍未连接的块。
    pub(crate) fn cancel(self, volume: &Ext4Volume) {
        self.mapping.cancel(volume);
    }
}

impl PreparedMapping {
    /// 映射准备或目的页准备失败时只释放本次尚未连接的全部块。
    pub(crate) fn cancel(mut self, volume: &Ext4Volume) {
        for node in &self.nodes {
            if node.new {
                let buffer = node.buffer.as_ref().expect("new extent node lacks buffer");
                // SAFETY: 该块仍由本计划 reservation 保有且未接入旧根；metadata_block 读取已结束，
                // 新节点未发表写回且没有旧树消费者，失效准确窗口后才归还位图。
                unsafe { buffer.forget(volume.claim()) }.expect("new extent alias invalidation");
            }
        }
        for edit in &self.indirect {
            if edit.initialize {
                // SAFETY: initialize 仅标记本计划新间接节点；旧读取已结束、指针尚未进旧根，
                // 其 BufferRef 没有写回或其他消费者，失效窗口后才归还物理位。
                unsafe { edit.buffer.forget(volume.claim()) }
                    .expect("new indirect alias invalidation");
            }
        }

        self.nodes.clear();
        self.indirect.clear();
        self.retired.clear();
        while let Some(block) = self.allocated.pop() {
            volume.release_unpublished_block(block);
        }
    }

    /// caller 保持 root 写侧，所有窗口已有效且属于卷自身 RW claim；提交不分配或读盘。
    pub(crate) fn publish(mut self, volume: &Ext4Volume, seed: u32) -> Vec<DetachedBlock> {
        for node in &self.nodes {
            if !node.dirty {
                continue;
            }
            if let Some(buffer) = &node.buffer {
                buffer
                    .update(volume.claim(), |bytes| {
                        patch_node(volume.geometry(), bytes, node, seed)
                    })
                    .expect("prepared writable extent node");
            }
        }

        // 新间接链先初始化，最后才连接已有父指针；读者仍受 inode root 写侧排斥。
        for edit in &self.indirect {
            edit.buffer
                .update(volume.claim(), |bytes| {
                    if edit.initialize {
                        bytes.fill(0);
                    }
                    codec::put32(bytes, edit.offset, edit.pointer);
                })
                .expect("prepared writable indirect node");
        }

        while let Some(block) = self.allocated.pop() {
            block.publish();
        }
        for block in &mut self.retired {
            block.arm_release();
        }
        self.retired
    }
}

/// 建立一份仅持本次计划、不隐含任何已发表块拥有的修改容器。
fn empty_mapping(root: [u8; 60]) -> PreparedMapping {
    PreparedMapping {
        root,
        physical: 0,
        zero: false,
        added_blocks: 0,
        removed_blocks: 0,
        retired: Vec::new(),
        nodes: Vec::new(),
        indirect: Vec::new(),
        allocated: Vec::new(),
    }
}

/// 普通前台写按原读格式准备映射；调用者没有回写停止边界，查询不附 metadata IO context。
pub(crate) fn prepare(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
) -> Result<PreparedMapping, FsError> {
    let g = volume.geometry();
    let limit = if inode.flags & EXTENTS != 0 {
        codec::EXTENT_LOGICAL_BLOCKS
    } else {
        let n = (g.block_size / 4) as u64;
        codec::DIRECT_BLOCKS as u64 + n + n * n + n * n * n
    };
    if limit <= logical {
        return Err(FsError::FileTooLarge);
    }

    let existing = map(volume, inode, logical, logical + 1, None)?;
    let mut prepared = empty_mapping(inode.block);
    if let BlockMapping::Mapped { physical, .. } = existing {
        prepared.physical = physical;
        return Ok(prepared);
    }

    prepared.zero = true;
    let result = if inode.flags & EXTENTS != 0 {
        prepare_extent(volume, inode, logical, existing, &mut prepared, true)
    } else {
        prepare_indirect(volume, inode, logical, &mut prepared)
    };
    if let Err(error) = result {
        prepared.cancel(volume);
        return Err(error);
    }
    prepared.added_blocks = prepared.allocated.len() as u64;
    prepared.removed_blocks = prepared.retired.len() as u64;
    Ok(prepared)
}

/// 为一次目录扩展预留全部数据块和块树修改；新映射在统一发表前不接入旧 inode 根。
pub(crate) fn prepare_directory_mappings(
    volume: &Ext4Volume,
    inode: &InodeData,
    root: &[u8; 60],
    logical_blocks: &[u64],
) -> Result<PreparedDirectoryMappings, FsError> {
    if inode.kind != InodeKind::Directory {
        return Err(FsError::InvalidOperation);
    }
    prepare_mapping_batch(volume, inode, root, logical_blocks)
}

/// 符号链接使用相同的 block-tree planner 和 reservation path。
pub(crate) fn prepare_symlink_mappings(
    volume: &Ext4Volume,
    inode: &InodeData,
    root: &[u8; 60],
    logical_blocks: &[u64],
) -> Result<PreparedDirectoryMappings, FsError> {
    if inode.kind != InodeKind::Symlink {
        return Err(FsError::InvalidOperation);
    }
    prepare_mapping_batch(volume, inode, root, logical_blocks)
}

/// 普通目录或符号链接分配在同一预备树累积连续空洞，不继承回写停止边界或发表中间根。
fn prepare_mapping_batch(
    volume: &Ext4Volume,
    inode: &InodeData,
    root: &[u8; 60],
    logical_blocks: &[u64],
) -> Result<PreparedDirectoryMappings, FsError> {
    if inode.block != *root {
        return Err(FsError::InvalidOperation);
    }

    let mut logical = Vec::new();
    logical
        .try_reserve_exact(logical_blocks.len())
        .map_err(|_| FsError::NoMemory)?;
    let mut physical = Vec::new();
    physical
        .try_reserve_exact(logical_blocks.len())
        .map_err(|_| FsError::NoMemory)?;
    let mut previous: Option<u64> = None;
    for &block in logical_blocks {
        if previous.is_some_and(|previous| previous.checked_add(1) != Some(block)) {
            return Err(FsError::InvalidOperation);
        }
        if !matches!(
            map(
                volume,
                inode,
                block,
                block.checked_add(1).ok_or(FsError::FileTooLarge)?,
                None,
            )?,
            BlockMapping::Hole { .. }
        ) {
            return Err(FsError::Corrupt);
        }
        logical.push(block);
        previous = Some(block);
    }

    let mut mapping = empty_mapping(*root);
    for &block in &logical {
        mapping.zero = true;
        let result = if inode.flags & EXTENTS != 0 {
            prepare_extent(
                volume,
                inode,
                block,
                BlockMapping::Hole { len: 1 },
                &mut mapping,
                false,
            )
        } else {
            prepare_indirect(volume, inode, block, &mut mapping)
        };
        if let Err(error) = result {
            mapping.cancel(volume);
            return Err(error);
        }
        physical.push(mapping.physical);
        mapping.added_blocks = mapping.allocated.len() as u64;
    }
    Ok(PreparedDirectoryMappings {
        mapping,
        logical,
        physical,
        seed: inode.checksum_seed,
    })
}

/// 准备条目容量在任何共同 Buffer 内容借用之前完成。
fn entry_storage(max: usize) -> Result<Vec<[u8; 12]>, FsError> {
    let mut entries = Vec::new();
    entries
        .try_reserve_exact(max + 2)
        .map_err(|_| FsError::NoMemory)?;
    Ok(entries)
}

/// 已校验的节点条目进入预备独占空间，内容借用内不分配或重复计算 CRC。
fn read_node(bytes: &[u8], mut entries: Vec<[u8; 12]>) -> Result<NodeEdit, FsError> {
    let header = codec::extent_header(bytes, None)?;
    assert!(usize::from(header.max) + 2 <= entries.capacity());
    for index in 0..usize::from(header.entries) {
        let mut entry = [0; 12];
        entry.copy_from_slice(codec::bytes(bytes, 12 + index * 12, 12)?);
        entries.push(entry);
    }
    Ok(NodeEdit {
        buffer: None,
        physical: 0,
        entries,
        max: usize::from(header.max),
        depth: header.depth,
        generation: codec::le32(bytes, 8)?,
        dirty: false,
        new: false,
    })
}

/// 叶条目只对单个真正初始化的 FS 块转换，前后 unwritten 继续保持零语义。
fn prepare_extent(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    existing: BlockMapping,
    prepared: &mut PreparedMapping,
    consolidate: bool,
) -> Result<(), FsError> {
    let g = volume.geometry();
    let depth = if prepared.nodes.is_empty() {
        codec::extent_header(&inode.block, None)?.depth as usize
    } else {
        prepared.nodes[0].depth as usize
    };
    if codec::EXTENT_DEPTH_LIMIT < depth {
        return Err(FsError::FileTooLarge);
    }
    prepared
        .nodes
        .try_reserve_exact((depth + 2) * 2)
        .map_err(|_| FsError::NoMemory)?;
    prepared
        .allocated
        .try_reserve_exact(depth + 4)
        .map_err(|_| FsError::NoMemory)?;
    if consolidate {
        prepared
            .retired
            .try_reserve_exact(depth * 2)
            .map_err(|_| FsError::NoMemory)?;
    }

    if prepared.nodes.is_empty() {
        prepared.nodes.push(read_node(
            &inode.block,
            entry_storage(codec::EXTENT_ROOT_ENTRIES)?,
        )?);
    }

    let mut path = [0usize; codec::EXTENT_DEPTH_LIMIT + 1];
    let mut slots = [0usize; codec::EXTENT_DEPTH_LIMIT + 1];
    let mut visited = [0u64; codec::EXTENT_DEPTH_LIMIT];
    let mut high = codec::EXTENT_LOGICAL_BLOCKS;
    for level in 0..depth {
        let parent_index = path[level];
        let node = &prepared.nodes[parent_index];
        let selected = node
            .entries
            .iter()
            .rposition(|entry| entry_key(entry) <= logical)
            .unwrap_or(0);
        slots[level] = selected;
        let entry = node.entries.get(selected).ok_or(FsError::Corrupt)?;
        let physical = index_physical(entry);
        let next_low = entry_key(entry);
        let next_high = node
            .entries
            .get(selected + 1)
            .map(entry_key)
            .unwrap_or(high);
        if visited[..level].contains(&physical) {
            return Err(FsError::Corrupt);
        }
        visited[level] = physical;
        let child_index = if let Some(index) = prepared
            .nodes
            .iter()
            .position(|candidate| candidate.physical == physical)
        {
            if prepared.nodes[index].depth != (depth - level - 1) as u16 {
                return Err(FsError::Corrupt);
            }
            index
        } else {
            let entries = entry_storage(
                (g.block_size - codec::EXTENT_HEADER_BYTES) / codec::EXTENT_ENTRY_BYTES,
            )?;
            let buffer = volume.metadata_block(physical)?;
            let mut child = buffer.with_read(|bytes| {
                extent_step(
                    g,
                    inode,
                    bytes,
                    logical.max(next_low),
                    Some((depth - level - 1) as u16),
                    Some(next_low),
                    next_high,
                )?;
                read_node(bytes, entries)
            })??;
            child.buffer = Some(buffer);
            child.physical = physical;
            prepared.nodes.push(child);
            prepared.nodes.len() - 1
        };
        path[level + 1] = child_index;
        high = next_high;
    }

    prepared.physical = match existing {
        BlockMapping::Unwritten { physical, .. } => physical,
        BlockMapping::Hole { .. } => {
            let block = volume.allocate_block((inode.number - 1) / g.inodes_per_group)?;
            let physical = block.physical();
            if codec::EXTENT_PHYSICAL_LIMIT <= physical {
                volume.release_unpublished_block(block);
                return Err(FsError::FileTooLarge);
            }
            prepared.allocated.push(block);
            physical
        }
        _ => return Err(FsError::Corrupt),
    };
    let leaf = path[depth];
    change_leaf(
        &mut prepared.nodes[leaf].entries,
        logical,
        prepared.physical,
    )?;
    prepared.nodes[leaf].dirty = true;

    let mut split_nodes = false;
    let mut level = depth;
    loop {
        let node_index = path[level];
        let count = prepared.nodes[node_index].entries.len();
        if prepared.nodes[node_index].max < count {
            split_nodes = true;
            let split = count / 2;
            let node_depth = prepared.nodes[node_index].depth;
            let right = new_node(volume, inode, prepared, node_depth)?;
            let right_index = prepared.nodes.len() - 1;
            for index in split..count {
                let entry = prepared.nodes[node_index].entries[index];
                prepared.nodes[right_index].entries.push(entry);
            }
            prepared.nodes[node_index].entries.truncate(split);
            prepared.nodes[node_index].dirty = true;
            let right_key = entry_key(&prepared.nodes[right_index].entries[0]);
            if level == 0 {
                if codec::EXTENT_DEPTH_LIMIT <= usize::from(prepared.nodes[0].depth) {
                    return Err(FsError::FileTooLarge);
                }
                let node_depth = prepared.nodes[0].depth;
                let left = new_node(volume, inode, prepared, node_depth)?;
                let left_index = prepared.nodes.len() - 1;
                for index in 0..prepared.nodes[0].entries.len() {
                    let entry = prepared.nodes[0].entries[index];
                    prepared.nodes[left_index].entries.push(entry);
                }
                let left_key = entry_key(&prepared.nodes[left_index].entries[0]);
                prepared.nodes[0].entries.clear();
                prepared.nodes[0].entries.push(index_entry(left_key, left));
                prepared.nodes[0]
                    .entries
                    .push(index_entry(right_key, right));
                prepared.nodes[0].depth += 1;
                prepared.nodes[0].max = codec::EXTENT_ROOT_ENTRIES;
                break;
            }
            let left_key = entry_key(&prepared.nodes[node_index].entries[0]);
            let parent_index = path[level - 1];
            let parent = &mut prepared.nodes[parent_index];
            codec::put32(&mut parent.entries[slots[level - 1]], 0, left_key as u32);
            parent
                .entries
                .insert(slots[level - 1] + 1, index_entry(right_key, right));
            parent.dirty = true;
        } else if level != 0 {
            let key = entry_key(&prepared.nodes[node_index].entries[0]);
            let parent_index = path[level - 1];
            let parent = &mut prepared.nodes[parent_index];
            codec::put32(&mut parent.entries[slots[level - 1]], 0, key as u32);
            parent.dirty = true;
        }
        if level == 0 {
            break;
        }
        level -= 1;
    }

    if consolidate && !split_nodes {
        consolidate_path(volume, inode, prepared, &mut slots, depth)?;
    }
    patch_node(
        g,
        &mut prepared.root,
        &prepared.nodes[0],
        inode.checksum_seed,
    );
    Ok(())
}

/// 分配新外部节点并保留原 token；任何后续失败由 prepare 的唯一错误出口归还。
fn new_node(
    volume: &Ext4Volume,
    inode: &InodeData,
    prepared: &mut PreparedMapping,
    depth: u16,
) -> Result<u64, FsError> {
    let g = volume.geometry();
    let max = (g.block_size - codec::EXTENT_HEADER_BYTES) / codec::EXTENT_ENTRY_BYTES;
    let mut entries = Vec::new();
    entries
        .try_reserve_exact(max + 2)
        .map_err(|_| FsError::NoMemory)?;
    prepared
        .nodes
        .try_reserve(1)
        .map_err(|_| FsError::NoMemory)?;
    prepared
        .allocated
        .try_reserve(1)
        .map_err(|_| FsError::NoMemory)?;

    let block = volume.allocate_block((inode.number - 1) / g.inodes_per_group)?;
    let physical = block.physical();
    prepared.allocated.push(block);
    if codec::EXTENT_PHYSICAL_LIMIT <= physical {
        return Err(FsError::FileTooLarge);
    }
    let buffer = volume.metadata_block(physical)?;
    prepared.nodes.push(NodeEdit {
        buffer: Some(buffer),
        physical,
        entries,
        max,
        depth,
        generation: 0,
        dirty: true,
        new: true,
    });
    Ok(physical)
}

/// 相邻同层节点能容纳时合并条目并删父索引，再把单子根压回 inode。
/// 所有释放窗口在发表之前取得，旧树在准备失败时仍保持原样。
fn consolidate_path(
    volume: &Ext4Volume,
    inode: &InodeData,
    prepared: &mut PreparedMapping,
    slots: &mut [usize; codec::EXTENT_DEPTH_LIMIT + 1],
    depth: usize,
) -> Result<(), FsError> {
    let g = volume.geometry();
    for level in (1..=depth).rev() {
        let slot = slots[level - 1];
        let parent = &prepared.nodes[level - 1];
        let sibling_slot = if slot + 1 < parent.entries.len() {
            slot + 1
        } else if slot != 0 {
            slot - 1
        } else {
            continue;
        };
        let sibling_entry = parent.entries[sibling_slot];
        let sibling_physical = index_physical(&sibling_entry);
        let low = entry_key(&sibling_entry);
        let high = parent
            .entries
            .get(sibling_slot + 1)
            .map(entry_key)
            .unwrap_or(codec::EXTENT_LOGICAL_BLOCKS);
        let entries =
            entry_storage((g.block_size - codec::EXTENT_HEADER_BYTES) / codec::EXTENT_ENTRY_BYTES)?;
        let buffer = volume.metadata_block(sibling_physical)?;
        let node_depth = prepared.nodes[level].depth;
        let mut sibling = buffer.with_read(|bytes| {
            extent_step(g, inode, bytes, low, Some(node_depth), Some(low), high)?;
            read_node(bytes, entries)
        })??;
        sibling.buffer = Some(buffer);
        sibling.physical = sibling_physical;
        let current = &prepared.nodes[level];
        let (left, right) = if slot < sibling_slot {
            (current, &sibling)
        } else {
            (&sibling, current)
        };
        let boundary_merge = node_depth == 0
            && leaves_merge(left.entries.last().unwrap(), right.entries.first().unwrap())?;
        let count = left.entries.len() + right.entries.len() - usize::from(boundary_merge);
        if left.max < count {
            continue;
        }

        let detached = if slot < sibling_slot {
            sibling_physical
        } else {
            current.physical
        };
        let release = volume.prepare_detached_block(detached, None)?;
        if slot < sibling_slot {
            for entry in sibling.entries {
                prepared.nodes[level].entries.push(entry);
            }
        } else {
            for entry in &prepared.nodes[level].entries {
                sibling.entries.push(*entry);
            }
            prepared.nodes[level] = sibling;
            slots[level - 1] -= 1;
        }
        if node_depth == 0 {
            merge_leaf(&mut prepared.nodes[level].entries)?;
        }
        prepared.nodes[level].dirty = true;
        prepared.nodes[level - 1]
            .entries
            .remove(slot.max(sibling_slot));
        prepared.nodes[level - 1].dirty = true;
        prepared.retired.push(release);
    }

    // 单子根降深不需要申请新节点，inline 容量始终是四项。
    for level in 1..=depth {
        if prepared.nodes[0].entries.len() != 1
            || codec::EXTENT_ROOT_ENTRIES < prepared.nodes[level].entries.len()
        {
            break;
        }
        if index_physical(&prepared.nodes[0].entries[0]) != prepared.nodes[level].physical {
            break;
        }
        let release = volume.prepare_detached_block(prepared.nodes[level].physical, None)?;
        prepared.nodes[0].entries.clear();
        for index in 0..prepared.nodes[level].entries.len() {
            let entry = prepared.nodes[level].entries[index];
            prepared.nodes[0].entries.push(entry);
        }
        prepared.nodes[0].depth = prepared.nodes[level].depth;
        prepared.nodes[0].max = codec::EXTENT_ROOT_ENTRIES;
        prepared.nodes[level].dirty = false;
        prepared.retired.push(release);
    }
    Ok(())
}

/// 邻项只在状态、逻辑/物理连续和 ee_len 容量全部相容时合并。
fn leaves_merge(left: &[u8; 12], right: &[u8; 12]) -> Result<bool, FsError> {
    let (physical, len, unwritten) = leaf_fields(left)?;
    let (next_physical, next_len, next_unwritten) = leaf_fields(right)?;
    Ok(unwritten == next_unwritten
        && entry_key(left) + len == entry_key(right)
        && physical + len == next_physical
        && len + next_len
            <= u64::from(if unwritten {
                codec::EXTENT_UNWRITTEN_MAX_LEN
            } else {
                codec::EXTENT_INIT_MAX_LEN
            }))
}

/// 条目 key 是逻辑 FS 块号，与物理块地址的高低半编码无关。
fn entry_key(entry: &[u8; 12]) -> u64 {
    u32::from_le_bytes(entry[..4].try_into().unwrap()) as u64
}

/// 组合 extent index 的 48 位物理地址。
fn index_physical(entry: &[u8; 12]) -> u64 {
    u32::from_le_bytes(entry[4..8].try_into().unwrap()) as u64
        | ((u16::from_le_bytes(entry[8..10].try_into().unwrap()) as u64) << 32)
}

/// 构造正式 index 条目，保留两字节 unused 为零。
fn index_entry(logical: u64, physical: u64) -> [u8; 12] {
    let mut entry = [0; 12];
    codec::put32(&mut entry, 0, logical as u32);
    codec::put32(&mut entry, 4, physical as u32);
    codec::put16(&mut entry, 8, (physical >> 32) as u16);
    entry
}

/// 构造容量已经检查的 initialized 或 unwritten 叶条目。
fn leaf_entry(logical: u64, physical: u64, len: u64, unwritten: bool) -> [u8; 12] {
    let mut entry = [0; 12];
    codec::put32(&mut entry, 0, logical as u32);
    codec::put16(
        &mut entry,
        4,
        (len + if unwritten {
            u64::from(codec::EXTENT_INIT_MAX_LEN)
        } else {
            0
        }) as u16,
    );
    codec::put16(&mut entry, 6, (physical >> 32) as u16);
    codec::put32(&mut entry, 8, physical as u32);
    entry
}

/// 解码独占条目中的物理地址、长度和 unwritten 状态。
fn leaf_fields(entry: &[u8; 12]) -> Result<(u64, u64, bool), FsError> {
    let (len, unwritten) = codec::extent_length(codec::le16(entry, 4)?)?;
    Ok((
        u64::from(codec::le32(entry, 8)?) | (u64::from(codec::le16(entry, 6)?) << 32),
        len,
        unwritten,
    ))
}

/// 插入或三段切分只作用于准备空间，随后合并同状态且逻辑/物理连续的邻项。
fn change_leaf(entries: &mut Vec<[u8; 12]>, logical: u64, physical: u64) -> Result<(), FsError> {
    let mut index = 0;
    while index < entries.len() && entry_key(&entries[index]) <= logical {
        let start = entry_key(&entries[index]);
        let (old_physical, len, unwritten) = leaf_fields(&entries[index])?;
        if logical < start + len {
            if !unwritten {
                return Err(FsError::Corrupt);
            }
            entries.remove(index);
            if start < logical {
                entries.insert(
                    index,
                    leaf_entry(start, old_physical, logical - start, true),
                );
                index += 1;
            }
            entries.insert(index, leaf_entry(logical, physical, 1, false));
            if logical + 1 < start + len {
                entries.insert(
                    index + 1,
                    leaf_entry(logical + 1, physical + 1, start + len - logical - 1, true),
                );
            }
            merge_leaf(entries)?;
            return Ok(());
        }
        index += 1;
    }
    entries.insert(index, leaf_entry(logical, physical, 1, false));
    merge_leaf(entries)
}

/// 每次合并减少一项；状态和最大长度限制与盘面 ee_len 编码保持一致。
fn merge_leaf(entries: &mut Vec<[u8; 12]>) -> Result<(), FsError> {
    let mut index = 0;
    while index + 1 < entries.len() {
        let left = entries[index];
        let right = entries[index + 1];
        let (physical, len, unwritten) = leaf_fields(&left)?;
        let (next_physical, next_len, next_unwritten) = leaf_fields(&right)?;
        let max = u64::from(if unwritten {
            codec::EXTENT_UNWRITTEN_MAX_LEN
        } else {
            codec::EXTENT_INIT_MAX_LEN
        });
        if unwritten == next_unwritten
            && entry_key(&left) + len == entry_key(&right)
            && physical + len == next_physical
            && len + next_len <= max
        {
            entries[index] = leaf_entry(entry_key(&left), physical, len + next_len, unwritten);
            entries.remove(index + 1);
        } else {
            index += 1;
        }
    }
    Ok(())
}

/// 字段级编码节点及有效条目；共同 Buffer 的其他区域不从旧快照恢复。
fn patch_node(g: &Geometry, bytes: &mut [u8], node: &NodeEdit, seed: u32) {
    if node.new {
        bytes.fill(0);
    }
    codec::put16(bytes, 0, codec::EXTENT_MAGIC);
    codec::put16(bytes, 2, node.entries.len() as u16);
    codec::put16(bytes, 4, node.max as u16);
    codec::put16(bytes, 6, node.depth);
    codec::put32(bytes, 8, node.generation);
    for (index, entry) in node.entries.iter().enumerate() {
        bytes[12 + index * 12..24 + index * 12].copy_from_slice(entry);
    }

    if node.buffer.is_some() && g.features_ro_compat & codec::METADATA_CSUM != 0 {
        let tail = codec::EXTENT_HEADER_BYTES + node.max * codec::EXTENT_ENTRY_BYTES;
        codec::put32(bytes, tail, codec::crc32c(seed, &bytes[..tail]));
    }
}

/// 传统块号分解为 inode 槽及至多三层指针槽，不改变已有映射格式。
fn indirect_offsets(
    g: &Geometry,
    logical: u64,
) -> Result<([usize; codec::INDIRECT_LEVELS + 1], usize), FsError> {
    let mut offsets = [0usize; codec::INDIRECT_LEVELS + 1];
    if logical < codec::DIRECT_BLOCKS as u64 {
        offsets[0] = logical as usize;
        return Ok((offsets, 1));
    }
    let n = (g.block_size / 4) as u64;
    let mut remaining = logical - codec::DIRECT_BLOCKS as u64;
    let mut span = n;
    let mut depth = 1;
    while span <= remaining && depth < codec::INDIRECT_LEVELS {
        remaining -= span;
        span *= n;
        depth += 1;
    }
    if span <= remaining {
        return Err(FsError::FileTooLarge);
    }
    offsets[0] = codec::DIRECT_BLOCKS - 1 + depth;
    for slot in &mut offsets[1..=depth] {
        span /= n;
        *slot = (remaining / span) as usize;
        remaining %= span;
    }
    Ok((offsets, depth + 1))
}

/// 从真实节点窗口或本批修改中读取传统指针，不把未写入的新节点旧盘字节当成有效内容。
fn indirect_pointer(
    prepared: &PreparedMapping,
    buffer: &BufferRef,
    physical: u64,
    offset: usize,
) -> Result<u64, FsError> {
    if let Some(edit) = prepared
        .indirect
        .iter()
        .rev()
        .find(|edit| edit.physical == physical && edit.offset == offset)
    {
        return Ok(u64::from(edit.pointer));
    }
    if prepared
        .indirect
        .iter()
        .any(|edit| edit.physical == physical && edit.initialize)
    {
        return Ok(0);
    }
    Ok(u64::from(
        buffer.with_read(|bytes| codec::le32(bytes, offset))??,
    ))
}

/// 传统 direct/single/double/triple 分支累积到同一准备中再发表。
fn prepare_indirect(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    prepared: &mut PreparedMapping,
) -> Result<(), FsError> {
    let (offsets, length) = indirect_offsets(volume.geometry(), logical)?;
    prepared
        .indirect
        .try_reserve_exact(length)
        .map_err(|_| FsError::NoMemory)?;
    prepared
        .allocated
        .try_reserve_exact(length)
        .map_err(|_| FsError::NoMemory)?;

    let mut parent = None;
    let mut missing = 0;
    let mut physical = u64::from(codec::le32(&prepared.root, offsets[0] * 4)?);
    while physical != 0 && missing + 1 < length {
        let node_physical = physical;
        let buffer = volume.metadata_block(node_physical)?;
        missing += 1;
        physical = indirect_pointer(prepared, &buffer, node_physical, offsets[missing] * 4)?;
        parent = Some((buffer, node_physical));
    }
    if physical != 0 {
        return Err(FsError::Corrupt);
    }

    let mut next = 0u32;
    for level in (missing..length).rev() {
        let block =
            volume.allocate_block((inode.number - 1) / volume.geometry().inodes_per_group)?;
        let physical = block.physical();
        prepared.allocated.push(block);
        if u64::from(u32::MAX) < physical {
            return Err(FsError::FileTooLarge);
        }
        if level + 1 == length {
            prepared.physical = physical;
        } else {
            let buffer = volume.metadata_block(physical)?;
            prepared.indirect.push(IndirectEdit {
                buffer,
                physical,
                offset: offsets[level + 1] * 4,
                pointer: next,
                initialize: true,
            });
        }
        next = physical as u32;
    }

    if let Some((buffer, physical)) = parent {
        prepared.indirect.push(IndirectEdit {
            buffer,
            physical,
            offset: offsets[missing] * 4,
            pointer: next,
            initialize: false,
        });
    } else {
        codec::put32(&mut prepared.root, offsets[0] * 4, next);
    }
    Ok(())
}

/// 截断只预备原树的尾部字段和释放窗口，失败或取消不会修改已发表的块树与位图。
pub(crate) struct PreparedTruncate {
    pub root: [u8; 60],
    /// 实际摘离的文件系统块数，包括 initialized、unwritten 数据与外部树节点。
    pub removed_blocks: u64,
    retired: Vec<DetachedBlock>,
    nodes: Vec<NodeEdit>,
    indirect: Vec<(BufferRef, usize)>,
}

impl PreparedTruncate {
    /// 调用者保持 inode 根写侧并已准备本记录；这里只修改当前节点字段，不分配或读盘。
    /// 返回的窗口仍未归还位图，须在旧数据与元数据 I/O、别名使用全部结束后消费。
    pub(crate) fn publish(mut self, volume: &Ext4Volume, seed: u32) -> Vec<DetachedBlock> {
        for node in &self.nodes {
            node.buffer
                .as_ref()
                .expect("external truncate node")
                .update(volume.claim(), |bytes| {
                    patch_node(volume.geometry(), bytes, node, seed)
                })
                .expect("prepared writable truncate extent");
        }
        for (buffer, first) in &self.indirect {
            buffer
                .update(volume.claim(), |bytes| bytes[*first * 4..].fill(0))
                .expect("prepared writable truncate indirect");
        }
        for block in &mut self.retired {
            block.arm_release();
        }
        self.retired
    }
}

/// 在 inode 写序列化与根保护下预备字节 EOF 后的全部块；块内部分尾保留给页截断处理。
/// 增长只保留原根；所有可失败的窗口和容量申请均在任何共同树字段修改之前完成。
pub(crate) fn prepare_truncate(
    volume: &Ext4Volume,
    inode: &InodeData,
    new_size: u64,
    context: Option<MetadataIoContext<'_>>,
) -> Result<PreparedTruncate, FsError> {
    volume.require_write(volume.claim())?;
    let g = volume.geometry();
    let limit = if inode.flags & EXTENTS != 0 {
        codec::EXTENT_LOGICAL_BLOCKS
    } else {
        let n = (g.block_size / 4) as u64;
        codec::DIRECT_BLOCKS as u64 + n + n * n + n * n * n
    };
    let keep = codec::ceil_div(new_size, g.block_size as u64)?;
    if limit < keep {
        return Err(FsError::FileTooLarge);
    }

    let mut prepared = PreparedTruncate {
        root: inode.block,
        removed_blocks: 0,
        retired: Vec::new(),
        nodes: Vec::new(),
        indirect: Vec::new(),
    };
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
        if new_size != 0 {
            return Err(FsError::InvalidOperation);
        }
        prepared.root.fill(0);
        if inode.flags & EXTENTS != 0 {
            codec::put16(&mut prepared.root, 0, codec::EXTENT_MAGIC);
            codec::put16(&mut prepared.root, 4, codec::EXTENT_ROOT_ENTRIES as u16);
        }
    } else if new_size <= inode.size {
        validate_root(g, inode)?;
        if inode.flags & EXTENTS != 0 {
            prepare_extent_truncate(volume, inode, keep, &mut prepared, context)?;
        } else {
            prepare_indirect_truncate(volume, inode, keep, &mut prepared, context)?;
        }
    }
    prepared.removed_blocks = prepared.retired.len() as u64;
    Ok(prepared)
}

/// 数据块只保原位图与已缓存物理别名，不为删除读入文件数据或改动分配状态。
fn retire_data(
    volume: &Ext4Volume,
    physical: u64,
    count: u64,
    prepared: &mut PreparedTruncate,
    context: Option<MetadataIoContext<'_>>,
) -> Result<(), FsError> {
    prepared
        .retired
        .try_reserve(usize::try_from(count).map_err(|_| FsError::NoMemory)?)
        .map_err(|_| FsError::NoMemory)?;
    for block in physical..physical + count {
        prepared
            .retired
            .push(volume.prepare_detached_data_block(block, context)?);
    }
    Ok(())
}

/// 外部树节点只在确认从准备根不可寻址后登记，准备计划析构仍只放窗口引用。
fn retire_node(
    volume: &Ext4Volume,
    physical: u64,
    prepared: &mut PreparedTruncate,
    context: Option<MetadataIoContext<'_>>,
) -> Result<(), FsError> {
    prepared
        .retired
        .try_reserve(1)
        .map_err(|_| FsError::NoMemory)?;
    prepared
        .retired
        .push(volume.prepare_detached_block(physical, context)?);
    Ok(())
}

/// 在共同内容借用外申请条目容量，读取时沿原 extent 格式校验深度、父边界与 CRC。
fn load_truncate_extent(
    volume: &Ext4Volume,
    inode: &InodeData,
    physical: u64,
    depth: u16,
    low: u64,
    high: u64,
    context: Option<MetadataIoContext<'_>>,
) -> Result<NodeEdit, FsError> {
    let g = volume.geometry();
    let entries =
        entry_storage((g.block_size - codec::EXTENT_HEADER_BYTES) / codec::EXTENT_ENTRY_BYTES)?;
    g.validate_blocks(physical, 1)?;
    let buffer = volume
        .metadata_cache()
        .get(physical, g.block_size, context)?;
    let mut node = buffer.with_read(|bytes| {
        extent_step(g, inode, bytes, low, Some(depth), Some(low), high)?;
        read_node(bytes, entries)
    })??;
    node.buffer = Some(buffer);
    node.physical = physical;
    Ok(node)
}

/// 从右向左缩短叶尾并逐级摘空索引；只保留仍可寻址且确实变化的外部节点字段。
fn truncate_extent_node(
    volume: &Ext4Volume,
    inode: &InodeData,
    node: &mut NodeEdit,
    keep: u64,
    mut high: u64,
    prepared: &mut PreparedTruncate,
    context: Option<MetadataIoContext<'_>>,
) -> Result<(), FsError> {
    if node.depth == 0 {
        while let Some(entry) = node.entries.last().copied() {
            let start = entry_key(&entry);
            let (physical, len, unwritten) = leaf_fields(&entry)?;
            if start + len <= keep {
                break;
            }
            let remaining = keep.saturating_sub(start).min(len);
            retire_data(
                volume,
                physical + remaining,
                len - remaining,
                prepared,
                context,
            )?;
            if remaining == 0 {
                node.entries.pop();
            } else {
                let slot = node.entries.len() - 1;
                node.entries[slot] = leaf_entry(start, physical, remaining, unwritten);
            }
            node.dirty = true;
        }
        return Ok(());
    }

    while let Some(entry) = node.entries.last().copied() {
        if high <= keep {
            break;
        }
        let low = entry_key(&entry);
        let physical = index_physical(&entry);
        let mut child =
            load_truncate_extent(volume, inode, physical, node.depth - 1, low, high, context)?;
        truncate_extent_node(volume, inode, &mut child, keep, high, prepared, context)?;
        if child.entries.is_empty() {
            retire_node(volume, physical, prepared, context)?;
            node.entries.pop();
            node.dirty = true;
            high = low;
        } else {
            if child.dirty {
                prepared
                    .nodes
                    .try_reserve(1)
                    .map_err(|_| FsError::NoMemory)?;
                prepared.nodes.push(child);
            }
            break;
        }
    }
    Ok(())
}

/// 空树恢复 inline 叶根；单子节点四项内逐级降深，不申请新磁盘块。
fn prepare_extent_truncate(
    volume: &Ext4Volume,
    inode: &InodeData,
    keep: u64,
    prepared: &mut PreparedTruncate,
    context: Option<MetadataIoContext<'_>>,
) -> Result<(), FsError> {
    let mut root = read_node(&inode.block, entry_storage(codec::EXTENT_ROOT_ENTRIES)?)?;
    truncate_extent_node(
        volume,
        inode,
        &mut root,
        keep,
        codec::EXTENT_LOGICAL_BLOCKS,
        prepared,
        context,
    )?;

    if root.entries.is_empty() {
        root.depth = 0;
        root.max = codec::EXTENT_ROOT_ENTRIES;
    }
    while root.depth != 0 && root.entries.len() == 1 {
        let entry = root.entries[0];
        let physical = index_physical(&entry);
        let staged = prepared
            .nodes
            .iter()
            .position(|node| node.physical == physical);
        let child = if let Some(index) = staged {
            if codec::EXTENT_ROOT_ENTRIES < prepared.nodes[index].entries.len() {
                break;
            }
            prepared.nodes.remove(index)
        } else {
            let child = load_truncate_extent(
                volume,
                inode,
                physical,
                root.depth - 1,
                entry_key(&entry),
                codec::EXTENT_LOGICAL_BLOCKS,
                context,
            )?;
            if codec::EXTENT_ROOT_ENTRIES < child.entries.len() {
                break;
            }
            child
        };
        retire_node(volume, physical, prepared, context)?;
        root.entries.clear();
        for entry in child.entries {
            root.entries.push(entry);
        }
        root.depth = child.depth;
        root.max = codec::EXTENT_ROOT_ENTRIES;
    }

    patch_node(
        volume.geometry(),
        &mut prepared.root,
        &root,
        inode.checksum_seed,
    );
    Ok(())
}

/// 读取间接节点的指针字段到固定栈空间，内容锁内只解码和检查物理范围。
fn read_truncate_indirect(
    g: &Geometry,
    bytes: &[u8],
    pointers: &mut [u32; codec::MAX_BLOCK_BYTES / core::mem::size_of::<u32>()],
) -> Result<(), FsError> {
    for (index, pointer) in pointers[..g.block_size / 4].iter_mut().enumerate() {
        *pointer = codec::le32(bytes, index * 4)?;
        if *pointer != 0 {
            g.validate_blocks(u64::from(*pointer), 1)?;
        }
    }
    Ok(())
}

/// 传统树从尾部向上预备释放；剩余节点只记录要清零的指针后缀，不发表旧块快照。
fn truncate_indirect_node(
    volume: &Ext4Volume,
    physical: u64,
    depth: usize,
    low: u64,
    keep: u64,
    prepared: &mut PreparedTruncate,
    context: Option<MetadataIoContext<'_>>,
) -> Result<bool, FsError> {
    let g = volume.geometry();
    let n = g.block_size / 4;
    let span = (n as u64).pow((depth - 1) as u32);
    g.validate_blocks(physical, 1)?;
    let buffer = volume
        .metadata_cache()
        .get(physical, g.block_size, context)?;
    let mut pointers = [0u32; codec::MAX_BLOCK_BYTES / core::mem::size_of::<u32>()];
    buffer.with_read(|bytes| read_truncate_indirect(g, bytes, &mut pointers))??;

    let mut first = n;
    for index in (0..n).rev() {
        let start = low + index as u64 * span;
        if start + span <= keep {
            break;
        }
        let child = u64::from(pointers[index]);
        if child == 0 {
            continue;
        }
        let empty = if depth == 1 {
            retire_data(volume, child, 1, prepared, context)?;
            true
        } else {
            truncate_indirect_node(volume, child, depth - 1, start, keep, prepared, context)?
        };
        if empty {
            pointers[index] = 0;
            first = index;
        }
    }

    let empty = pointers[..n].iter().all(|pointer| *pointer == 0);
    if empty {
        retire_node(volume, physical, prepared, context)?;
    } else if first < n {
        prepared
            .indirect
            .try_reserve(1)
            .map_err(|_| FsError::NoMemory)?;
        prepared.indirect.push((buffer, first));
    }
    Ok(empty)
}

/// 先处理 direct 尾指针，再按各层逻辑跨度处理 single、double、triple 和空根。
fn prepare_indirect_truncate(
    volume: &Ext4Volume,
    inode: &InodeData,
    keep: u64,
    prepared: &mut PreparedTruncate,
    context: Option<MetadataIoContext<'_>>,
) -> Result<(), FsError> {
    for slot in (keep.min(codec::DIRECT_BLOCKS as u64) as usize..codec::DIRECT_BLOCKS).rev() {
        let physical = u64::from(codec::le32(&inode.block, slot * 4)?);
        if physical != 0 {
            retire_data(volume, physical, 1, prepared, context)?;
            codec::put32(&mut prepared.root, slot * 4, 0);
        }
    }

    let n = (volume.geometry().block_size / 4) as u64;
    let mut low = codec::DIRECT_BLOCKS as u64;
    let mut span = n;
    for depth in 1..=codec::INDIRECT_LEVELS {
        let physical = u64::from(codec::le32(
            &inode.block,
            (codec::DIRECT_BLOCKS - 1 + depth) * 4,
        )?);
        if keep < low + span
            && physical != 0
            && truncate_indirect_node(volume, physical, depth, low, keep, prepared, context)?
        {
            codec::put32(
                &mut prepared.root,
                (codec::DIRECT_BLOCKS - 1 + depth) * 4,
                0,
            );
        }
        low += span;
        span *= n;
    }
    Ok(())
}
