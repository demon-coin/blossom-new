//! ext4 文件逻辑块映射查询、块树写入预备与尾部截断模块。
//!
//! # 模块职责与架构定位
//! 本模块负责将文件内的逻辑块号（Logical Block）翻译或修改为磁盘上的物理块号（Physical Block），
//! 同时完整支持现代 **Extent B+ 树（`EXT4_EXTENTS_FL`）** 与传统 **直接/一二三级间接块指针树**：
//! - **上游调用者**：`Ext4Volume`（`ext4.rs`）、目录扩展与 HTree 索引分裂（`directory.rs`）以及 VFS Inode 的读写、缺页可写转换（`mkwrite`）与 `truncate` 路径。
//! - **下游依赖**：`ext4::codec`（节点头部、条目与 CRC32C 编解码）、`ext4::allocation`（未发布块 `UnpublishedBlock` 分配与待回收块 `DetachedBlock` 登记）以及 `MetadataCache` 元数据块缓存。
//!
//! # 核心设计：两阶段预备与提交状态机（Prepare / Publish / Cancel）
//! 为了保证在内存不足（OOM）、磁盘空间耗尽（ENOSPC）或元数据损坏时已生效的文件树与位图不出现半修改状态，
//! 所有块树变更均严格遵循两阶段协议，且只保存结构化条目而非整块旧字节快照：
//! ```text
//!   prepare / prepare_directory_mappings / prepare_truncate
//!   (在持缓存读锁外预先申请堆内存、分配新块、加载并校验树路径，修改仅写入私有计划)
//!                            |
//!             +--------------+--------------+
//!             | (后续准备失败)               | (数据页与 inode 记录就绪，持 disk_root 写锁)
//!             v                             v
//!       cancel(volume)               publish(volume, seed)
//!   1. 失效新分配树节点的缓存别名     1. 原地更新各脏树节点字段与尾部 CRC
//!   2. 将全部 UnpublishedBlock       2. 解除 UnpublishedBlock 临时回滚责任
//!      归还块位图与空闲计数          3. 激活 DetachedBlock 延迟释放令牌返回给调用方，
//!   (已生效的旧块树毫发无损)             待旧数据/元数据 I/O 排空后再归还位图
//! ```
//!
//! ## 两种盘面块映射树结构对比
//! ```text
//! 1. Extent 树（inode.i_block 60B = 12B Header + 4 * 12B Entry）：
//!    depth > 0: [ExtentIdx (logical_lo, leaf_phys_48b)] ---> 外部 Extent 块（末尾含 4B CRC32C）
//!    depth = 0: [ExtentLeaf (logical, len+unwritten_bit, data_phys_48b)] ---> 连续物理数据块
//!
//! 2. 传统块指针树（inode.i_block 60B = 15 * u32 块号，每个外部块含 block_size/4 个 u32）：
//!    slot 0..12 : 直接指向数据块
//!    slot 12    : 一级间接块 -> 数据块
//!    slot 13    : 二级间接块 -> 一级间接块 -> 数据块
//!    slot 14    : 三级间接块 -> 二级间接块 -> 一级间接块 -> 数据块
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **Inode 写序列化锁（外层）**：所有 `prepare*` 与 `truncate` 均在调用方持有目标 inode 写互斥/失效锁的前提下运行，保证同一时刻只有一个写者规划块树变更，期间并发读者仍可在 `disk_root` 读锁下安全遍历旧树。
//! 2. **`disk_root` 读写锁 -> 元数据块内容锁（`BufferRef`）**：
//!    - 只读映射查询（`map`）在调用方持有 `disk_root` 读锁时执行；每层外部树块仅在 `BufferRef::with_read` 闭包内短暂持有页内容读锁完成解码，绝不跨磁盘 I/O 或堆分配持锁。
//!    - `prepare*` 阶段需要分配物理块时会获取卷级 `allocation` 睡眠锁，此时尚未持有 `disk_root` 写锁，也未持有任何 `BufferRef` 内容锁。
//!    - `publish` 阶段由调用方持有 `disk_root` 写锁排斥并发读者，在锁内依次对已预留的 `BufferRef` 调用 `update` 原地写入新指针与校验和，全程不触发内存分配或读盘 I/O。

use crate::block::MetadataIoContext;
use crate::cache::BufferRef;
use crate::fs::ext4::allocation::DetachedBlock;
use crate::fs::ext4::codec::{self, EXTENTS};
use crate::fs::ext4::{BlockMapping, Ext4Volume, Geometry, InodeData, UnpublishedBlock};
use crate::fs::{FsError, InodeKind};
use alloc::vec::Vec;
use core::convert::{TryFrom, TryInto};

/// Extent 树单层节点解析后的下一步动作。
#[derive(Clone, Copy)]
enum Step {
    /// 已在叶节点命中目标逻辑块或确认其位于空洞区间，可直接返回映射结果。
    Mapping(BlockMapping),
    /// 当前为内部索引节点，需要继续下降读取指定的下一层子块。
    Child {
        /// 下一层子节点的物理块号。
        physical: u64,
        /// 下一层子节点应当匹配的期望深度（`parent.depth - 1`）。
        depth: u16,
        /// 该子树负责的起始逻辑块号下界（等于选中的索引条目起始逻辑块号）。
        low: u64,
        /// 该子树负责的逻辑块号排他上界（等于右侧相邻索引条目的起始块号或父节点上界）。
        high: u64,
    },
}

/// 仅校验 inode 内联 `i_block` 根节点（Extent 根头与根条目，或全部 15 个传统块指针）的合法性，不发起外部块读取。
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

/// 查询文件在 `[logical, end)` 区间起点的连续块映射关系（已映射物理块、未写入预分配块或文件空洞）。
///
/// 调用方须持有该 inode 块映射根的读锁（或独占写序列化保护），并保证 `logical < end`；
/// 返回的连续段长度 `len` 会自动按请求的排他上界 `end - logical` 截断。
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

/// 沿 Extent 树自根向叶逐层下降查找逻辑块 `logical` 的映射。
///
/// 通过固定长度的 `visited` 数组检测并拒绝深度超限或祖先自引用环路；每个外部节点在读取时都会校验其尾部 CRC32C 与全部条目边界。
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

/// 在单个 Extent 节点字节切片上解码校验全部条目，并选出覆盖 `logical` 的叶子映射或下一层子节点。
///
/// 完整检查头部深度、外部节点 CRC32C、条目逻辑块严格递增且互不重叠、首项与父索引键一致、以及所有物理块范围不越界。
/// 若 `logical` 落在条目之间的间隙，则精确计算出到达下一个已存在条目（或节点上界 `high`）的空洞长度。
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

/// 校验传统 32 位块指针切片中的每一个非零物理块号是否合法（值为 `0` 代表合法的空洞指针）。
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

/// 在传统直接块及一、二、三级间接块指针树中查找逻辑块 `logical` 的映射。
///
/// 最多下降三层外部指针块，使用 `visited` 数组拒绝指向祖先块或自身的环路；若中途遇到零指针，则按该层子树剩余跨度返回对应长度的空洞。
fn indirect_map(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    context: Option<MetadataIoContext<'_>>,
) -> Result<BlockMapping, FsError> {
    let g = volume.geometry();
    // Step 1: 校验内联指针数组；若逻辑块号落在前 12 个直接块内，直接返回对应数据块或单块空洞。
    validate_pointers(g, &inode.block)?;
    if logical < codec::DIRECT_BLOCKS as u64 {
        let physical = u64::from(codec::le32(&inode.block, logical as usize * 4)?);
        return Ok(if physical == 0 {
            BlockMapping::Hole { len: 1 }
        } else {
            BlockMapping::Mapped { physical, len: 1 }
        });
    }

    // Step 2: 计算逻辑块所属的间接层级（1..=3）及其在该层子树内的相对偏移。
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

    // Step 3: 从内联间接根槽位出发逐层读取外部指针块，校验整块指针并检测环路，直至定位到底层数据块指针。
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

/// 针对单个 inode 的块映射树写入修改计划。
///
/// 仅保存变更后的内联根字节、需要原地更新的结构化节点条目以及未发布的块分配令牌，
/// 避免保存整块旧快照，从而在提交时不会误覆盖共享同一缓存页的其他元数据。
pub(crate) struct PreparedMapping {
    /// 提交后将写入 inode `i_block` 的 60 字节新映射根。
    pub root: [u8; 60],
    /// 本次目标逻辑块最终映射到的目标数据块物理块号。
    pub physical: u64,
    /// 目标数据块是否为新分配或由未写入段（unwritten）首次转正，写入前需先在页缓存中清零未覆盖区域。
    pub zero: bool,
    /// 本计划新分配的文件系统块总数（含新数据块与新增的 Extent/间接树节点块）。
    pub added_blocks: u64,
    /// 本计划因路径合并（Consolidation）而摘除回收的外部树节点块数。
    pub removed_blocks: u64,
    /// 树路径合并后待延后释放回位图的旧外部树节点块列表。
    retired: Vec<DetachedBlock>,
    /// Extent 树路径上加载或新建的节点编辑状态（索引 `0` 固定为内联根节点）。
    nodes: Vec<NodeEdit>,
    /// 传统间接块树路径上待写入的指针槽编辑列表。
    indirect: Vec<IndirectEdit>,
    /// 本计划新申请但尚未接入已生效块树的物理块分配令牌列表。
    allocated: Vec<UnpublishedBlock>,
}

/// Extent 树单个节点（内联根或外部块节点）的内存编辑状态。
struct NodeEdit {
    /// 外部树节点对应的元数据块缓存句柄；内联根节点为 `None`。
    buffer: Option<BufferRef>,
    /// 外部树节点的物理块号；内联根节点为 `0`。
    physical: u64,
    /// 节点内按逻辑块升序排列的 12 字节原始条目列表（叶条目或索引条目）。
    entries: Vec<[u8; 12]>,
    /// 节点允许容纳的最大条目数（`eh_max`）。
    max: usize,
    /// 节点在树中的高度（`0` 为叶节点，大于 `0` 为索引节点）。
    depth: u16,
    /// 节点头保存的 `eh_generation` 字段值。
    generation: u32,
    /// 该节点在本次规划中是否发生了条目或头部变更，需要在 `publish` 时写回。
    dirty: bool,
    /// 该节点是否为本计划全新分配的外部块（提交时需先将整块清零，取消时需失效其缓存窗口）。
    new: bool,
}

/// 传统间接指针块上的单条 32 位块号写入操作。
struct IndirectEdit {
    /// 目标间接块对应的元数据块缓存句柄。
    buffer: BufferRef,
    /// 目标间接块的物理块号。
    physical: u64,
    /// 待写入指针在该间接块内的字节偏移（4 字节对齐）。
    offset: usize,
    /// 待写入的下级块物理块号（32 位）。
    pointer: u32,
    /// 该间接块是否为本计划全新分配的块（首次提交时需先将整块其余槽位清零，取消时需失效缓存窗口）。
    initialize: bool,
}

/// 目录或符号链接一次性批量扩展多个连续逻辑块时的映射修改计划。
pub(crate) struct PreparedDirectoryMappings {
    /// 聚合了整批逻辑块分配与树分裂操作的基础映射计划。
    mapping: PreparedMapping,
    /// 按顺序记录的待扩展逻辑块号列表。
    logical: Vec<u64>,
    /// 与 `logical` 一一对应的已分配物理数据块号列表。
    physical: Vec<u64>,
    /// 所属 inode 的元数据 CRC32C 校验种子，用于提交外部 Extent 树块尾部校验和。
    seed: u32,
}

impl PreparedDirectoryMappings {
    /// 查询本批次中指定逻辑块 `logical` 所分配到的物理数据块号。
    pub(crate) fn physical(&self, logical: u64) -> Option<u64> {
        self.logical
            .iter()
            .position(|candidate| *candidate == logical)
            .map(|index| self.physical[index])
    }

    /// 返回完成整批块扩展后的 60 字节新内联块映射根。
    pub(crate) fn root(&self) -> &[u8; 60] {
        &self.mapping.root
    }

    /// 返回本批次共新增分配的文件系统块总数（含目录/链接数据块及新增树节点块）。
    pub(crate) fn added_blocks(&self) -> u64 {
        self.mapping.added_blocks
    }

    /// 在新目录块内容落入缓存后正式提交全部树节点修改，并解除所有新分配块的临时回滚责任。
    pub(crate) fn publish(self, volume: &Ext4Volume) {
        let retired = self.mapping.publish(volume, self.seed);
        assert!(retired.is_empty(), "directory mapping retired old blocks");
    }

    /// 在目录操作准备失败时取消本批次修改，失效新建树节点缓存并将全部未连接的新块归还位图。
    pub(crate) fn cancel(self, volume: &Ext4Volume) {
        self.mapping.cancel(volume);
    }
}

impl PreparedMapping {
    /// 放弃尚未发布的映射修改计划，失效新建树节点的缓存窗口并将所有新分配的物理块归还给卷位图。
    pub(crate) fn cancel(mut self, volume: &Ext4Volume) {
        for node in &self.nodes {
            if node.new {
                let buffer = node.buffer.as_ref().expect("new extent node lacks buffer");
                // SAFETY:
                // 1. `node.new` 仅在 `new_node` 为本计划新分配的块上置位，该物理块仍由 `self.allocated` 中的 `Reservation` 独占持有，从未写入已生效的 `disk_root`；
                // 2. `new_node` 中的 `metadata_block` 读取已结束，该新建缓存窗口从未提交写回，也没有其他并发读者；
                // 3. 在下方释放位图前先失效该元数据窗口，可防止物理块被重新分配给其他用途后残留脏缓存别名。
                unsafe { buffer.forget(volume.claim()) }.expect("new extent alias invalidation");
            }
        }
        for edit in &self.indirect {
            if edit.initialize {
                // SAFETY:
                // 1. `edit.initialize` 仅标记 `prepare_indirect` 中由本计划全新分配的间接指针块，其指针尚未写入已生效的父节点或 `disk_root`；
                // 2. 该 `BufferRef` 没有正在进行的写回，也不存在并发树遍历读者；
                // 3. 在将物理块归还块位图前先失效其元数据缓存窗口，保证后续复用安全。
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

    /// 在调用方持有 inode `disk_root` 写锁时正式发布外部树节点的全部变更，并返回待延后释放的旧节点块列表。
    ///
    /// 所有目标 `BufferRef` 均已在准备阶段加载就绪且卷具备读写权限，本方法全程只做内存原地修补与标记，不进行内存分配或读盘。
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

        // `prepare_indirect` 按自底向上顺序压入编辑项：先初始化新分配的子间接块，最后才改写已有父节点的指针，且全程受 `disk_root` 写锁保护。
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

/// 创建一个以 `root` 为初始内联根、不含任何新分配块或节点变更的空映射修改计划。
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

/// 为普通文件前台写或缺页可写（`mkwrite`）规划单个逻辑块 `logical` 的物理块映射。
///
/// - 若 `logical` 已映射到已初始化数据块，直接复用现有物理块并返回空变更计划；
/// - 若 `logical` 位于文件空洞或预分配未写入段（unwritten），则分配数据块/切分叶子并维护树平衡，同时将 `zero` 置为 `true`；
/// - 准备中途若发生任何错误会自动调用 `cancel` 归还已申请的块，不会污染已生效的块树。
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

/// 为目录扩容一次性预留全部新逻辑块的数据块与块树节点修改计划（校验 `inode` 必须为目录）。
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

/// 为长符号链接分配外部数据块及块树映射计划（校验 `inode` 必须为符号链接）。
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

/// 在同一份 [`PreparedMapping`] 上依次为一组严格连续且当前为空洞的逻辑块规划物理块分配。
///
/// 整批修改在内存中累积到同一个待发布根与节点集合中，任一步失败即整体调用 `cancel` 回收本批全部未发布块。
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

/// 在获取任何元数据缓存块内容锁之前预先在堆上分配条目存储数组（额外预留 2 项分裂空间，支持未写入段三段切分）。
fn entry_storage(max: usize) -> Result<Vec<[u8; 12]>, FsError> {
    let mut entries = Vec::new();
    entries
        .try_reserve_exact(max + 2)
        .map_err(|_| FsError::NoMemory)?;
    Ok(entries)
}

/// 将已通过完整格式与 CRC 校验的 Extent 节点条目拷贝到预分配好的私有数组 `entries` 中，构造 [`NodeEdit`]。
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

/// 为单个逻辑块 `logical` 规划 Extent 树的插入或未写入段（unwritten）转正。
///
/// 若 `existing` 为空洞则分配新物理数据块；若为未写入预分配段则原地切分出单个已初始化块（前后剩余未写入子段保持零语义）。
/// 当 `consolidate` 为 `true` 且本次未发生节点分裂时，会尝试合并相邻兄弟节点并压缩单子根节点高度。
fn prepare_extent(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    existing: BlockMapping,
    prepared: &mut PreparedMapping,
    consolidate: bool,
) -> Result<(), FsError> {
    let g = volume.geometry();
    // Step 1: 确定当前树深度，预先为节点列表、新分配块令牌及待回收块令牌保留足够的堆容量，加载内联根节点。
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

    // Step 2: 从根节点向下定位 `logical` 所在的叶子路径，优先复用 `prepared.nodes` 中已缓存的节点，未加载的外部节点则从元数据缓存读取并校验。
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

    // Step 3: 确定目标数据块的物理块号（复用未写入段已有物理块或为空洞新分配一块），并在叶子节点上插入或切分合并条目。
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

    // Step 4: 自底向上检查节点条目数是否溢出 `max`；若溢出则对半分裂右兄弟节点并更新父索引，若根节点溢出则分配左右两个子块使树高加 1。
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

    // Step 5: 若未发生分裂且允许路径压缩（如未写入段连续转正使叶子条目减少），尝试合并相邻节点并编码更新内联根字节。
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

/// 为 Extent 树分裂分配一个全新的外部节点块，预先加载其元数据缓存窗口并登记到 `prepared` 中。
///
/// 新分配块的 `UnpublishedBlock` 令牌推入 `prepared.allocated`；若后续步骤报错，由外层 `prepare` 统一调用 `cancel` 失效缓存并归还位图。
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

/// 在叶子条目合并缩短后，自底向上尝试将当前路径节点与相邻兄弟节点合并，并在根节点仅剩单个子索引且容量允许时将子节点拉回内联根以降低树高。
///
/// 被摘除的外部树节点通过 `prepare_detached_block` 预留释放窗口并登记入 `prepared.retired`，待 `publish` 提交新树后才真正回收。
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

    // 当内联根仅剩 1 个子索引且该子节点的条目数不超过内联根上限（4 项）时，直接将其提升回内联根并淘汰原外部块。
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

/// 判断两个相邻叶子 Extent 条目是否满足合并条件：未写入状态一致、逻辑块号紧邻连续、物理块号紧邻连续，且总长度不超过对应编码上限。
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

/// 提取 12 字节 Extent 叶条目或索引条目头 4 字节记录的起始逻辑块号（`ee_block` / `ei_block`）。
fn entry_key(entry: &[u8; 12]) -> u64 {
    u32::from_le_bytes(entry[..4].try_into().unwrap()) as u64
}

/// 从 12 字节 Extent 索引条目（`ext4_extent_idx`）中拼出下一层子节点的 48 位物理块号。
fn index_physical(entry: &[u8; 12]) -> u64 {
    u32::from_le_bytes(entry[4..8].try_into().unwrap()) as u64
        | ((u16::from_le_bytes(entry[8..10].try_into().unwrap()) as u64) << 32)
}

/// 构造 12 字节 Extent 索引条目（`ext4_extent_idx`），末尾 2 字节保留字段保持为零。
fn index_entry(logical: u64, physical: u64) -> [u8; 12] {
    let mut entry = [0; 12];
    codec::put32(&mut entry, 0, logical as u32);
    codec::put32(&mut entry, 4, physical as u32);
    codec::put16(&mut entry, 8, (physical >> 32) as u16);
    entry
}

/// 构造 12 字节 Extent 叶条目（`ext4_extent`），根据 `unwritten` 标志编码 16 位长度字段并写入 48 位起始物理块号。
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

/// 解码 12 字节 Extent 叶条目的 `(起始物理块号, 块长度, 是否为未写入段)`。
fn leaf_fields(entry: &[u8; 12]) -> Result<(u64, u64, bool), FsError> {
    let (len, unwritten) = codec::extent_length(codec::le16(entry, 4)?)?;
    Ok((
        u64::from(codec::le32(entry, 8)?) | (u64::from(codec::le16(entry, 6)?) << 32),
        len,
        unwritten,
    ))
}

/// 在叶节点的内存条目数组中将单个逻辑块 `logical` 更新为已初始化物理块 `physical`。
///
/// 若命中已有的未写入（unwritten）段，则将其切分为“前段未写入（可选）+ 当前单块已初始化 + 后段未写入（可选）”最多三项；
/// 若位于空洞区间，则按序插入单块已初始化条目。更新完成后自动合并所有可相连的相邻条目。
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

/// 顺序扫描叶节点条目数组，将所有状态一致、逻辑与物理块均连续且合并后不超过 `ee_len` 上限的相邻条目原地合并。
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

/// 将 [`NodeEdit`] 的头部字段与有效条目原地编码到目标字节切片 `bytes` 中，并为外部树块计算写入末尾 CRC32C。
///
/// 新建外部节点（`node.new == true`）会先将整块清零；已有节点仅覆写头部、有效条目与尾部 CRC，不触碰同一缓存页的其他区域。
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

/// 将传统映射下的逻辑块号 `logical` 分解为 `(各层槽位下标数组, 路径有效长度)`。
///
/// `offsets[0]` 为 inode `i_block` 内联数组下标（`0..12` 为直接块，`12..15` 为一至三级间接根），
/// `offsets[1..length]` 依次为从顶层间接块到底层数据块指针槽的块内下标。
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

/// 读取传统间接块 `physical` 在 `offset` 字节处的 32 位下级指针。
///
/// 优先从 `prepared.indirect` 中查找同一批次先前已规划的修改；若该块是本批次新分配但尚未落盘清零的新间接块（`initialize == true`），
/// 则直接视其余槽位为 `0`，避免将未初始化盘面残留字节误判为有效块指针。
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

/// 为传统直接/间接块映射树中的空洞逻辑块 `logical` 分配目标数据块及沿途缺失的各级间接指针块，并将编辑操作追加到 `prepared` 中。
fn prepare_indirect(
    volume: &Ext4Volume,
    inode: &InodeData,
    logical: u64,
    prepared: &mut PreparedMapping,
) -> Result<(), FsError> {
    // Step 1: 计算逻辑块对应的多级槽位坐标，并预留编辑项与分配块令牌的堆容量。
    let (offsets, length) = indirect_offsets(volume.geometry(), logical)?;
    prepared
        .indirect
        .try_reserve_exact(length)
        .map_err(|_| FsError::NoMemory)?;
    prepared
        .allocated
        .try_reserve_exact(length)
        .map_err(|_| FsError::NoMemory)?;

    // Step 2: 从待发布根出发向下遍历已存在的间接块链，找到首个零指针断开层级 `missing`。
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

    // Step 3: 自底向上依次分配目标数据块与缺失的各级新间接块，最后将新链顶指针写入已有父间接块或内联 `prepared.root`。
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

/// 文件截断（`truncate`）操作的块树尾部裁剪计划。
///
/// 准备阶段仅在私有内存中计算裁剪后的新根节点与待修补的外部节点，并为将被摘除的数据块和树节点块预留释放令牌；
/// 若中途失败直接丢弃本结构即可释放缓存引用，不会改动已生效的块树或磁盘位图。
pub(crate) struct PreparedTruncate {
    /// 截断完成后将写入 inode `i_block` 的 60 字节新映射根。
    pub root: [u8; 60],
    /// 本次截断共摘除的文件系统块总数（含已初始化数据块、未写入预分配块以及被整体回收的外部树节点块）。
    pub removed_blocks: u64,
    /// 被摘除的数据块与外部树节点块的延迟释放令牌列表。
    retired: Vec<DetachedBlock>,
    /// 截断后仍保留在树中、但条目被缩短而需要原地更新的外部 Extent 节点列表。
    nodes: Vec<NodeEdit>,
    /// 截断后仍保留在树中、但需要将末尾指针区间 `[first * 4..]` 清零的外部传统间接块列表。
    indirect: Vec<(BufferRef, usize)>,
}

impl PreparedTruncate {
    /// 在调用方持有 inode `disk_root` 写锁且旧页缓存已清理完毕后，原地提交所有剩余树节点的裁剪修改，并激活待释放块的回收责任。
    ///
    /// 返回的 [`DetachedBlock`] 列表尚未归还块位图；调用方须在更新完 inode 记录并确认旧数据/元数据写回彻底结束后再逐一释放。
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

/// 在调用方持有 inode 写序列化锁时，规划将文件截断到 `new_size` 字节所需摘除的全部尾部块（即索引号 `>= ceil(new_size / block_size)` 的完整块）。
///
/// 跨越 `new_size` 的最后一个保留块内部的部分尾部字节清零由上层页缓存截断逻辑处理；文件扩大（`new_size > inode.size`）时保持原块树不变。
/// 所有可能失败的内存预留、块读取和校验均在修改任何共享元数据前完成。
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

/// 为一段即将被截断摘除的连续数据块 `[physical, physical + count)` 预留位图释放窗口与已存在的元数据缓存别名，不触发文件数据读盘。
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

/// 为一棵子树已全部清空、即将从父节点摘除的外部树节点块 `physical` 预留位图释放与缓存失效令牌。
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

/// 在元数据块读锁外预分配条目数组，随后读取并校验截断路径上的外部 Extent 节点块，返回其私有 [`NodeEdit`]。
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

/// 自右向左递归裁剪单个 Extent 树节点中逻辑块号 `>= keep` 的尾部区间。
///
/// - 叶节点（`depth == 0`）：从末尾向前移除完全超出 `keep` 的叶条目，或缩短跨界条目的长度，登记摘除的数据块；
/// - 索引节点（`depth > 0`）：从末尾向前递归裁剪覆盖 `keep` 右侧的子节点；完全清空的子节点直接登记回收并弹出父索引，部分保留且发生修改的最后一个跨界子节点加入 `prepared.nodes` 待写回。
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

/// 规划 Extent 树的尾部截断：递归裁剪根子树后，若树变空则重置为深度 0 的空叶根；若根仅剩单个子节点且其条目装得进内联根（不超过 4 项），则循环提升子节点降低树高。
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

/// 在元数据块读锁内将传统间接块中的全部 32 位块指针解码到固定栈数组 `pointers` 中，并逐一校验非零物理块号合法性。
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

/// 自右向左递归裁剪单个传统间接指针块中覆盖 `>= keep` 逻辑块区间的尾部子树，返回该间接块裁剪后是否已完全变空。
///
/// 若整块指针均归零则直接登记回收该间接块本身；若仍有前缀有效指针，则记录首个归零槽位下标 `first`，供 `publish` 阶段原地将 `[first * 4..]` 清零。
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

/// 规划传统块指针树的尾部截断：先倒序清理内联直接块槽位，再依次按跨度递归裁剪一、二、三级间接子树并在子树全空时清零对应的内联根指针。
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
