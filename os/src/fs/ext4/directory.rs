//! ext4 目录存储后端：负责线性目录块与 HTree（`dx`）哈希索引目录的读取、校验、稳定游标枚举与两阶段原子修改。
//!
//! # 模块职责与架构定位
//! 本模块位于 `crate::fs::ext4` 子系统内部，实现磁盘目录在块设备元数据缓存（`MetadataCache`）上的读写与结构演进：
//! - **上游调用者**：`Ext4Volume`（转发来自 VFS `Inode` 缓存层与打开文件描述 `File` 的名字查找、`getdents64` 遍历、`lseek` 定位，以及 `mkdir` / `create` / `unlink` / `rmdir` / `rename` 的目录修改请求）。
//! - **下游依赖**：`super::mapping`（逻辑块到物理块的查询与新块批量分配预备）、`super::codec`（小端字节编解码、目录名字哈希与 CRC32C 校验和计算）以及 `crate::cache::BufferRef`（元数据块缓存窗口读写与失效）。
//!
//! # 核心设计与磁盘布局
//! ## 1. 三种目录块物理布局（`DirectoryLayout`）
//! 1. **普通叶块（`Leaf`）**：线性目录的所有块，或 HTree 目录中存放实际目录项的叶子块（逻辑块号 `> 0`）。
//! ```text
//!  0                                            end             block_size
//! +------------------+------------------+------+---------------+
//! | ext4_dir_entry_2 | ext4_dir_entry_2 | ...  | dir_tail(12B) |
//! +------------------+------------------+------+---------------+
//!  <--------- 紧凑或含空闲槽的 4B 对齐记录链 -----> <-- 可选 CRC -->
//! ```
//! 每条 `ext4_dir_entry_2` 头部固定 8 字节：`inode(4B) | rec_len(2B) | name_len(1B) | file_type(1B)`（未启用 `FILETYPE` 特性时后两字节合并为 `name_len(2B)`），后跟变长原始名字字节并按 4 字节向上对齐。当卷启用 `METADATA_CSUM` 时，块末 12 字节固定为伪目录项形式的校验尾（`ft = 0xde`）。
//!
//! 2. **HTree 索引根块（`Root`，逻辑块 0 且 `inode.flags` 含 `INDEX`）**：复用 `.` 与 `..` 两条目录项的覆盖空间嵌入 `dx_root_info` 与顶层索引表。
//! ```text
//!  0            12           24             32                  limit*8+32   block_size
//! +------------+------------+--------------+-------------------+------------+
//! | "." (12B)  | ".." 头+名 | dx_root_info | dx_countlimit(8B) | dx_tail    |
//! | rec_len=12 | rec_len=   | hash_ver(1B) | + dx_entry[1..]   | (可选 8B)  |
//! |            | blk_sz-12  | info_len=8   | (每项 8B:         | 存放 CRC   |
//! |            |            | depth(1B)    |  hash + block)    |            |
//! +------------+------------+--------------+-------------------+------------+
//! ```
//! 注意第 0 个索引槽位于偏移 `32`，其前 4 字节被 `limit(2B) | count(2B)` 占用，因此首项的最小哈希值隐式为 `0`，仅偏移 `36..40` 存放首个子块逻辑块号。
//!
//! 3. **HTree 内部索引节点块（`Index`，逻辑块 `> 0` 的中间层索引块）**：块首 8 字节为全零 `fake_dirent`（`inode = 0, rec_len = block_size`），偏移 `8` 起为 `dx_countlimit` 与 `dx_entry` 数组，启用 `METADATA_CSUM` 时末尾保留 8 字节 `dx_tail`。
//!
//! ## 2. HTree 分裂、Continuation 位与升级演进
//! - 当单块线性目录（仅逻辑块 0）插入新项空间不足且卷支持 `DIR_INDEX` 时，`make_indexed_directory` 将原普通目录项迁入新分配的叶块，把逻辑块 0 原地改造为 `Root` 并置位 `INDEX` 标志，随后触发首次叶分裂。
//! - 叶块满时（`split_leaf`），按目录项哈希值排序并折半拆分到左右两个叶块。若分裂点两侧存在相同 `major` 哈希的记录，向父索引插入的分隔键最低位置 `1`（continuation 位），提示查找（`find_entry`）与遍历（`next_indexed`）在命中前驱叶后必须继续扫描相邻叶块。
//! - 内部索引块满时折半分裂；根索引满且未达最大间接层数上限（普通 HTree 为 1 层，启用 `LARGEDIR` 时为 2 层）时，将原根条目下推至新内部节点并使树深度 `depth + 1`。
//!
//! ## 3. 两阶段准备与无失败发布（Prepare / Publish）
//! 所有目录写操作均采用“锁内副本预演 + 物理映射预留 -> 原子字节拷贝发布”的两阶段状态流转：
//! 1. **准备阶段（`DirectoryEditBuilder` -> `PreparedDirectoryEdit`）**：在可睡眠上下文读取受影响目录块的内存副本（`DirectoryImage`），在副本上完成记录增删、叶分裂、索引树更新及 CRC 重算；若需新增目录块，批量调用 `prepare_directory_mappings` 预留物理块并绑定缓存窗口（`BufferRef`）。此阶段任何校验失败、空间不足（`NoSpace`）或内存不足（`NoMemory`）均直接回滚预留块并失效未发布缓存窗口，已生效的盘面与缓存字节零修改。
//! 2. **发布阶段（`PreparedDirectoryEdit::publish`）**：不做任何磁盘读取或内存分配，先将新分配目录块的完整字节写入缓存窗口，再覆盖已有目录块与索引块，最后消费块映射预留并返回新 `root` / `size` / `blocks` / `flags` 供调用方更新 Inode。
//!
//! # 锁获取顺序与并发契约
//! - **读路径（`lookup` / `next` / `cursor_at` / `end_cookie` / `is_empty`）**：调用方必须持有目标目录 Inode 的目录操作读/写保护以及 `disk_root` 块映射读锁，保证传入的 `InodeMetadata` 与 60 字节 `map_root` 在整个块映射查询期间一致。在 `BufferRef::with_read` 闭包内持有单页内容锁，闭包内仅做纯内存解码与校验，严禁在闭包内再次发起磁盘 I/O 或获取其他锁。
//! - **写准备路径（`prepare_*`）**：调用方持有目录写序列化锁与 `disk_root` 读锁（可睡眠上下文）。
//! - **写发布路径（`PreparedDirectoryEdit::publish`）**：调用方持有目录写序列化锁与 `disk_root` **写锁**，排斥并发块映射读者；内部依次对单个 `BufferRef` 执行短临界区 `update` 拷贝字节。

use alloc::vec::Vec;

use super::mapping::PreparedDirectoryMappings;
use crate::block::SECTOR_BYTES;
use crate::cache::BufferRef;
use crate::fs::ext4::codec::{self, INDEX, METADATA_CSUM};
use crate::fs::ext4::{BlockMapping, Ext4Volume, Geometry};
use crate::fs::inode::InodeMetadata;
use crate::fs::types::NAME_MAX;
use crate::fs::{DirectoryCursor, DirectoryEntry, DirectoryPosition, FsError, InodeKind};

/// 启用 `LARGEDIR` 特性时 HTree 允许的最大间接索引层数（根块之下最多 2 层内部索引节点，即 3 级索引树）。
const DX_MAX_INDIRECT_LEVELS: usize = 2;
/// HTree 哈希目录的 `SEEK_END` / 遍历结束 64 位 Cookie 哨兵值。
const DX_EOF_COOKIE: u64 = 0x7fff_ffff_ffff_ffff;
/// HTree `dx_entry` 中逻辑块号字段的有效位掩码（低 28 位表示逻辑块号，高 4 位保留清零）。
const DX_BLOCK_MASK: u32 = (1 << 28) - 1;

/// `ext4_dir_entry_2` 目录项的固定头部大小（单位：字节）。
const DIRENT_HEADER_BYTES: usize = 8;
/// 目录项记录长度（`rec_len`）的字节对齐粒度（4 字节对齐）。
const DIRENT_ALIGNMENT: usize = 4;
/// 计算 4 字节向上对齐所用的低位掩码（`3`）。
const DIRENT_ROUND: usize = DIRENT_ALIGNMENT - 1;
/// 包含至少 1 字节名字的最短合法目录项记录长度（12 字节）。
const DIRENT_MIN_BYTES: usize = (DIRENT_HEADER_BYTES + 1 + DIRENT_ROUND) & !DIRENT_ROUND;

/// 单个 HTree 索引条目（`dx_entry`：`hash(4B) + block(4B)`）的字节大小。
const DX_ENTRY_BYTES: usize = 8;
/// 启用 `METADATA_CSUM` 时 HTree 根块或内部索引块末尾 `dx_tail` 的字节大小（8 字节）。
const DX_TAIL_BYTES: usize = 8;
/// HTree 内部索引节点块（`dx_node`）中 `dx_countlimit` 与条目数组的起始字节偏移（跳过 8 字节伪目录项头）。
const DX_NODE_ENTRIES_OFFSET: usize = 8;
/// HTree 根块（`dx_root`）中 `dx_countlimit` 与条目数组的起始字节偏移（跳过 `.`、`..` 与 8 字节 `dx_root_info`）。
const DX_ROOT_ENTRIES_OFFSET: usize = 32;

/// 目录修改发布后需要回写至规范（Canonical）Inode 的块映射与元数据统计结果。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct DirectoryEditResult {
    /// 更新后的 Inode 内联 60 字节块映射根（Extent 树根或传统直接/间接块指针数组）。
    pub(crate) root: [u8; 60],
    /// 更新后的目录逻辑大小（单位：字节，始终为文件系统块大小的整数倍）。
    pub(crate) size: u64,
    /// 更新后的目录已分配磁盘空间计数（单位：512 字节扇区）。
    pub(crate) blocks: u64,
    /// 更新后的 Inode 标志位（例如单块目录升级为 HTree 后新增 `INDEX` 标志）。
    pub(crate) flags: u32,
}

/// 单次目录编辑过程中缓存在内存里的单个逻辑目录块工作副本。
struct DirectoryImage {
    /// 该目录块在目录文件内的逻辑块号（从 `0` 开始）。
    logical: u64,
    /// 对应的元数据缓存窗口句柄；对于编辑中途新预留的逻辑块，在 `finish` 批量分配物理块前暂为 `None`。
    buffer: Option<BufferRef>,
    /// 长度等于文件系统块大小（`block_size`）的完整块字节工作副本。
    bytes: Vec<u8>,
}

/// 在归还块位图分配前，逐一失效本次未发布的新增物理目录块在元数据缓存中的窗口。
///
/// 调用上下文：可睡眠任务上下文，仅在目录编辑准备失败或显式取消（`cancel`）时调用。
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
        // SAFETY:
        // 1. `volume.claim()` 持有当前可写挂载对同一底层块视图的读写凭据；
        // 2. `physical` 来自当前尚未发布的 `mapping` 预留块，从未写入任何已发布的 Inode 块映射根，其他读者无法通过目录树访问该块；
        // 3. 前一行的 `buffer.wait_writeback()` 已等待该缓存页上可能残余的回写操作彻底结束，且此时物理块尚未归还给块分配器位图，不会与并发重分配冲突。
        unsafe { buffer.forget(volume.claim()) }.expect("unpublished directory block invalidation");
    }
}

/// 先失效本次新增目录块的元数据缓存窗口，再取消并归还全部预留的物理块映射。
fn cancel_unpublished_mappings(
    volume: &Ext4Volume,
    mapping: PreparedDirectoryMappings,
    logical_blocks: &[u64],
) {
    forget_unpublished_images(volume, &mapping, logical_blocks);
    mapping.cancel(volume);
}

/// 已完成全部校验、块分配与缓存窗口绑定的待发布目录修改令牌。
///
/// 准备阶段已吸收所有可能的 I/O、空间不足与内存分配失败；调用 [`Self::publish`] 时仅执行内存拷贝与预留块提交，若放弃提交则必须调用 [`Self::cancel`] 归还物理块。
pub(crate) struct PreparedDirectoryEdit {
    /// 若本次修改新增了目录叶块或索引块，保存批量预留的块映射事务；纯原地修改时为 `None`。
    mapping: Option<PreparedDirectoryMappings>,
    /// 本次修改涉及的所有已变更或新分配目录块的内存镜像与目标缓存窗口。
    images: Vec<DirectoryImage>,
    /// 本次修改新分配的目录逻辑块号列表（用于区分新块与既有块的写入顺序及失败回滚失效范围）。
    logical_blocks: Vec<u64>,
    /// 修改发布后将生效的 Inode 60 字节内联块映射根。
    root: [u8; 60],
    /// 修改发布后将生效的目录逻辑大小（单位：字节）。
    size: u64,
    /// 修改发布后将生效的目录占用扇区计数（单位：512 字节扇区）。
    blocks: u64,
    /// 修改发布后将生效的 Inode 标志位。
    flags: u32,
}

impl PreparedDirectoryEdit {
    /// 借用本次修改发布后将生效的 Inode 60 字节块映射根。
    pub(crate) fn root(&self) -> &[u8; 60] {
        &self.root
    }

    /// 返回本次修改发布后将生效的目录逻辑大小（单位：字节）。
    pub(crate) fn size(&self) -> u64 {
        self.size
    }

    /// 返回本次修改发布后将生效的 512 字节扇区计数。
    pub(crate) fn blocks(&self) -> u64 {
        self.blocks
    }
    /// 返回本次修改发布后将生效的 Inode 标志位。
    pub(crate) fn flags(&self) -> u32 {
        self.flags
    }

    /// 判断本次准备是否实际改动了任何目录块字节（例如指向同一 Inode 的原地 `rename` 为无操作，不更新时间戳或链接数）。
    pub(crate) fn changed(&self) -> bool {
        !self.images.is_empty()
    }

    /// 将已准备就绪的全部目录块镜像与块映射树修改原子提交到元数据缓存窗口。
    ///
    /// 调用方必须持有目标目录的写序列化锁以及 `disk_root` 写锁。本函数不触发内存分配或磁盘读取，不会返回可恢复错误。
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

        // Step 1: 分两轮写入缓存窗口——第一轮先写入本次新增分配的块（`new_block == true`），
        // 确保新叶块或新索引节点内容完整就绪后，第二轮再更新指向它们的既有索引块或根块。
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

        // Step 2: 提交块映射预留（更新外部 Extent/间接块缓存窗口并消费新物理块所有权）。
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

    /// 放弃本次未发布的目录编辑，失效新块缓存窗口并归还预留的物理块，保持原有目录内容不变。
    pub(crate) fn cancel(self, volume: &Ext4Volume) {
        if let Some(mapping) = self.mapping {
            cancel_unpublished_mappings(volume, mapping, &self.logical_blocks);
        }
    }
}

/// 已通过完整性校验的单个目录块物理布局分类。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DirectoryLayout {
    /// 存放实际目录项链表的叶子块或线性目录块（`end` 为目录项有效载荷区的结束字节偏移，不含末尾可选的 12 字节校验尾）。
    Leaf { end: usize },
    /// HTree 索引目录的逻辑第 0 块（包含 `.`、`..` 记录、`dx_root_info` 以及顶层索引表）。
    Root,
    /// HTree 索引目录的内部索引节点块（包含 8 字节伪目录项头与下级索引表）。
    Index,
}

/// 将用户态 64 位目录偏移 Cookie 解码为 HTree 哈希索引游标。
///
/// 即使目录在打开期间由线性布局升级为 HTree 布局，仍沿用原 Cookie 的数值编码继续定位，并清空进程内哈希碰撞位置提示。
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

/// 根据 `lseek` 传入的 64 位用户态 Cookie 构造当前目录格式对应的遍历游标。
///
/// 调用方需持有目录读保护；显式定位不会沿用先前的哈希碰撞块内偏移状态。
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

/// 返回当前目录在 `SEEK_END` 语义下的末尾 Cookie 值（线性目录为字节大小，HTree 目录为 `DX_EOF_COOKIE` 哨兵）。
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

/// 从目录块字节流中解码出的单条 `ext4_dir_entry_2` 记录摘要。
#[derive(Clone, Copy)]
struct Record {
    /// 目录项指向的 Inode 编号；为 `0` 表示该槽位是已删除留下的空闲记录。
    number: u64,
    /// 名字原始字节在当前目录块切片内的起始偏移（单位：字节）。
    name_offset: usize,
    /// 名字原始字节的实际长度（单位：字节，范围 `0..=NAME_MAX`）。
    name_len: usize,
    /// 当卷启用 `FILETYPE` 特性且编码非 `0` 时解析出的目标对象类型，否则为 `None`。
    kind: Option<InodeKind>,
    /// 下一条相邻目录项在当前目录块内的起始字节偏移（即 `offset + rec_len`）。
    next: usize,
}

/// 校验 Inode 是否为目录且逻辑大小非零、按文件系统块大小对齐，并核对 `INDEX` 标志与卷特性兼容位的一致性。
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

/// 将目录逻辑块号映射为物理块并从元数据缓存读取对应窗口；若遇到稀疏空洞、未初始化区或越界则判定为文件系统损坏（`FsError::Corrupt`）。
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

/// HTree 索引下降路径中的单层节点访问帧。
#[derive(Clone, Copy, Default)]
struct DxFrame {
    /// 当前索引节点块在目录文件内的逻辑块号（根块为 `0`，内部索引节点 `> 0`）。
    logical: u64,
    /// 为 `true` 表示该层是逻辑第 0 块的 `dx_root`，为 `false` 表示内部索引节点 `dx_node`。
    root: bool,
    /// 当前选中子范围条目在该节点 `dx_entry` 数组中的下标（`0..count`）。
    position: usize,
    /// 该节点当前已使用的有效 `dx_entry` 条目总数。
    count: usize,
}

/// 解码后的单条 HTree 索引条目（`dx_entry`）。
#[derive(Clone, Copy)]
struct DxEntry {
    /// 子树或叶块覆盖区间的起始 32 位哈希阈值（最低位为 `1` 时表示与前一相邻区间存在跨块同哈希记录，即 continuation 标志）。
    hash: u32,
    /// 下级索引节点或目录叶块在目录文件内的逻辑块号。
    block: u64,
}

/// 从 HTree 根块下降至目标叶块的完整搜索路径栈。
struct DxPath {
    /// 自顶向下的各级索引节点访问帧（`frames[0]` 固定为 `dx_root`，后续元素为各级 `dx_node`）。
    frames: [DxFrame; DX_MAX_INDIRECT_LEVELS + 1],
    /// `frames` 数组中当前有效的索引层数（取值范围 `1..=DX_MAX_INDIRECT_LEVELS + 1`）。
    len: usize,
    /// 当前路径末端命中的目录叶块逻辑块号。
    leaf: u64,
    /// 当前叶块区间经路径各层累积后的最小 `major` 哈希下界（已清除最低位 continuation 标志）。
    lower_bound: u32,
}

/// 返回当前文件系统特性组合允许的最大 HTree 间接索引层数（启用 `LARGEDIR` 时为 2 层，否则为 1 层）。
fn dx_max_indirect_levels(g: &Geometry) -> usize {
    if g.features_incompat & codec::FEATURE_INCOMPAT_LARGEDIR != 0 {
        DX_MAX_INDIRECT_LEVELS
    } else {
        1
    }
}

/// 解码 HTree 根块或内部节点中下标为 `position` 的索引条目；首项（`position == 0`）因存储槽存放 `limit/count` 而隐含起始哈希为 `0`。
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

/// 在按哈希升序排列的 `dx_entry` 数组中二分查找不大于目标 `hash` 的最后一个前驱条目下标。
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

/// 读取并校验指定的 HTree 索引块，在其中二分选出覆盖目标 `hash` 的子范围条目。
///
/// 返回 `(当前层访问帧, 选中的子条目, 根块声明的间接深度)`（非根块第三项恒为 `0`）。
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

/// 重新读取 `frame` 所在索引块中下标为 `position` 的条目，并校验节点有效条目总数与初次探测时一致。
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

/// 读取并校验指定的内部索引节点块，返回指向其首项（`position == 0`）的访问帧与对应子条目，用于跨子树顺序推进。
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

/// 自 HTree 根块逐层探测目标 `hash` 所属的初始叶块路径，同时检测索引环路并累积叶区间的最小 `major` 哈希下界。
fn dx_probe(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    hash: u32,
) -> Result<DxPath, FsError> {
    // Step 1: 读取并校验逻辑第 0 块（`dx_root`），获取顶层前驱条目与间接索引层数 `depth`。
    let (root, entry, depth) = read_dx_frame(volume, inode, map_root, 0, true, hash)?;
    let mut path = DxPath {
        frames: [DxFrame::default(); DX_MAX_INDIRECT_LEVELS + 1],
        len: 1,
        leaf: entry.block,
        lower_bound: entry.hash & !1,
    };
    path.frames[0] = root;
    let mut logical = entry.block;
    // Step 2: 沿内部索引节点逐层下降，利用 `visited` 数组拦截指向已访问节点的环路损坏。
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

/// 按 HTree 树内中序将 `path` 推进到下一个叶块区间，并返回新叶区间的 `major` 哈希下界。
///
/// 若传入 `through_major` 且下一区间的起始下界已严格大于 `through_major`，则直接返回该下界而无需读取下级内部索引块；若整棵树已遍历完毕则返回 `Ok(None)`。
fn next_dx_leaf(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    path: &mut DxPath,
    through_major: Option<u32>,
) -> Result<Option<u32>, FsError> {
    // 自底向上回溯路径栈，寻找第一个尚未走到末尾槽位的索引层。
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

/// 递归遍历剩余 `levels` 层 HTree 内部索引节点，按树内顺序对每个叶块逻辑块号调用 `visit`（返回 `Ok(false)` 时提前终止遍历）。
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

/// 按 HTree 索引树顺序遍历目录的所有叶块，无需在堆上收集全部叶块列表；若所有叶块均返回 `true` 则最终返回 `Ok(true)`。
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

/// 校验单个目录块的物理布局、记录边界与元数据 CRC32C 校验和，返回该块的布局类型及可枚举范围。
///
/// 参数 `index` 指示当块不是 `dx_root` 时，调用方是否预期将其作为内部索引节点（`dx_node`）解析。
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
    // 分支 1: 校验 HTree 根块（`Root`）或内部索引节点（`Index`），根块还需核对 `.` 与 `..` 固定槽位。
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

    // 分支 2: 校验普通叶块（`Leaf`）；若启用 `METADATA_CSUM`，先核对末尾 12 字节伪目录项头与整块 CRC32C。
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

    // 逐条遍历叶块内的记录链表，确保所有 `rec_len` 严丝合缝地拼接到有效载荷末尾 `end`。
    let mut offset = 0;
    while offset < end {
        offset = record(g, data, offset, end)?.next;
    }
    if offset != end {
        return Err(FsError::Corrupt);
    }
    Ok(DirectoryLayout::Leaf { end })
}

/// 校验 HTree 根块或内部索引节点的头部字段、容量上限、条目哈希升序性、子块逻辑范围及尾部 CRC32C（不含未使用的空闲索引槽）。
fn verify_dx(
    g: &Geometry,
    inode: &InodeMetadata,
    logical: u64,
    data: &[u8],
    root: bool,
) -> Result<(), FsError> {
    // Step 1: 核对 `dx_root` 的 `.` / `..` 覆盖头与 `dx_root_info`，或 `dx_node` 的 8 字节全零伪目录项头。
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

    // Step 2: 核对 `dx_countlimit` 与块大小、可选 8 字节 `dx_tail` 计算出的槽位上限一致。
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

    // Step 3: 逐项检查哈希键非递减排序，并确保目标逻辑块号非零、未越过目录末尾且未自环指向当前块。
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
        // `dx_tail` 前 4 字节保留字段参与 CRC 计算，但磁盘格式不强制其为零值。
        codec::verify_dx_checksum(inode, data, offset + count * DX_ENTRY_BYTES, tail)?;
    }
    Ok(())
}

/// 解码偏移 `offset` 处的单条目录项记录，校验 4 字节对齐、名字边界、剩余碎片尺寸与 `file_type` 编码，原样保留非 UTF-8 原始字节名字。
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

/// 在已校验的目录块内查找偏移不小于 `requested` 的第一条非空目录项（忽略 `inode == 0` 的已删除槽位、HTree 内部节点及块尾校验伪项）。
///
/// 若 `requested` 落在某条记录中间或越过有效区且不等于整块末尾，则返回 `FsError::InvalidOperation`。
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

/// 在目录中成功定位到的有效目录项位置与属性摘要。
#[derive(Clone, Copy)]
struct FoundEntry {
    /// 该目录项所在的目录文件逻辑块号。
    logical: u64,
    /// 该目录项在所属目录块内的起始字节偏移。
    offset: usize,
    /// 目录项指向的目标 Inode 编号（非零）。
    number: u64,
    /// 目录项名字的字节长度。
    name_len: usize,
    /// 目录项中记录的目标对象类型（未启用 `FILETYPE` 或编码为 `0` 时为 `None`）。
    kind: Option<InodeKind>,
}

/// 读取并校验指定的普通叶块或 HTree 根块，按原始字节精确匹配查找名为 `name` 的有效目录项。
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

/// 读取 HTree 根块 `dx_root_info` 中声明的目录专属哈希算法版本号（不能用超级块的默认哈希版本替代既有目录自身的格式）。
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

/// 在目录中查找名为 `name` 的目录项：线性目录按逻辑块顺序扫描，HTree 目录沿索引前驱及同哈希溢出链探测。
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

    // 分支 1: 普通线性目录按逻辑块号从 `0` 到末尾依次扫描。
    if inode.flags & INDEX == 0 {
        for logical in 0..inode.size / volume.geometry().block_size as u64 {
            if let Some(entry) = find_entry_in_block(volume, inode, map_root, logical, name)? {
                return Ok(Some(entry));
            }
        }
        return Ok(None);
    }
    // 分支 2: HTree 索引目录中 `.` 与 `..` 固定存放在逻辑第 0 块头部，不参与哈希索引路由。
    if name == b"." || name == b".." {
        return find_entry_in_block(volume, inode, map_root, 0, name);
    }

    // 分支 3: 计算名字哈希并探测起始叶块；若相邻叶块起始下界仍不大于目标哈希（即存在跨块哈希碰撞），继续向后扫描。
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

/// 在目录中查找单个路径组件 `name` 对应的目标 Inode 编号（未命中返回 `Ok(None)`）。
///
/// 调用方需持有目录读保护以及 `disk_root` 块映射读保护。
pub(crate) fn lookup(
    volume: &Ext4Volume,
    inode: &InodeMetadata,
    map_root: &[u8; 60],
    name: &[u8],
) -> Result<Option<u64>, FsError> {
    Ok(find_entry(volume, inode, map_root, name)?.map(|entry| entry.number))
}

/// HTree 目录遍历时按哈希序与块内偏移参与排序的候选目录项。
#[derive(Clone, Copy)]
struct HashCandidate {
    /// 目录项的 32 位主哈希值（最低位始终为 `0`）。
    major: u32,
    /// 目录项的 32 位次哈希值。
    minor: u32,
    /// 目录项在目录块内的物理位置与属性摘要。
    entry: FoundEntry,
}

/// 判断指定目录项（由 `(major, minor)` 哈希对及块内位置 `position` 标识）是否严格排在当前遍历游标 `cursor` 之后（或恰为游标指向的未碰撞首项）。
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

/// 比较两个 HTree 遍历候选项的产出先后顺序：依次按 `(major, minor, 逻辑块号, 块内字节偏移)` 字典序升序排列。
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

/// 维护游标之后排序最靠前的两个候选项（`best` 为本次将返回的目录项，`following` 用于构造指向下一项的 `next_cursor`）。
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

/// 在 HTree 哈希索引目录中读取游标 `cursor` 对应的下一条目录项，并生成携带碰撞续读位置的下一个游标。
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

    // Step 1: 根块中的 `.` 与 `..` 不在叶块内，分别以固定伪主哈希 `0` 和 `2` 纳入候选比较。
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

    // Step 2: 从游标当前的 `major` 哈希探测起始叶块，仅扫描可能包含更优 `best` 或 `following` 的哈希桶区间。
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
        // 只要下一叶区间的起始下界等于 `following.major`，说明同主哈希桶跨块溢出尚未扫完，仍可能有更小的 `(minor, position)`。
        if following.is_some_and(|next| next.major < lower_bound) {
            break;
        }
    }

    // Step 3: 为选中的最佳候选项分配名字缓冲区并拷贝原始名字字节。
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

    // Step 4: 根据次优候选项 `following` 计算 `next_cursor`；若两者 64 位哈希完全相同，记录当前项物理位置以消解碰撞。
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

/// 读取目录游标之后的下一条有效目录项；若目录在打开期间由线性布局升级为 HTree 布局，自动将旧字节 Cookie 转换为哈希游标继续遍历。
///
/// 调用方需持有目录读保护以及 `disk_root` 块映射读保护。
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

/// 目录两阶段修改的准备器：在内存中维护受影响目录块的局部拷贝与新逻辑块预留列表，在 `finish` 前不分配物理块。
struct DirectoryEditBuilder<'a> {
    /// 目标目录所属的 ext4 卷引用。
    volume: &'a Ext4Volume,
    /// 修改前目标目录的 Inode 元数据快照。
    inode: &'a InodeMetadata,
    /// 修改前目标目录的 60 字节内联块映射根。
    map_root: &'a [u8; 60],
    /// 本次修改期间已加载或新构造的目录块内存镜像列表。
    images: Vec<DirectoryImage>,
    /// 本次修改新预留的目录逻辑块号列表。
    logical_blocks: Vec<u64>,
    /// 下一个可供分配的目录末尾逻辑块号（初始为原目录包含的总块数）。
    next_logical: u64,
    /// 修改完成后的目标 Inode 标志位（升级为 HTree 时置位 `INDEX`）。
    next_flags: u32,
}

impl<'a> DirectoryEditBuilder<'a> {
    /// 基于当前目录的元数据与块映射根创建空的编辑准备器，此时不触发任何磁盘读取或块分配。
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

    /// 获取指定逻辑块的可变内存镜像；首次访问既有块时从元数据缓存拷贝一份块字节副本。
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

    /// 在目录末尾顺序预留一个新的逻辑块号（受限于 HTree 28 位块号上限），暂不分配底层物理块。
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

    /// 登记一个已在内存中构造完毕的新增逻辑块字节镜像，对应的元数据缓存窗口留待 `finish` 批量分配物理块后绑定。
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

    /// 为所有新预留逻辑块批量分配物理块映射并绑定元数据缓存窗口，封装为可无失败发布的 [`PreparedDirectoryEdit`]。
    ///
    /// 若块分配、溢出检查或缓存窗口获取失败，会自动失效未发布的新块缓存窗口并归还物理块，不会改动任何已发布的盘面或缓存状态。
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
        // 快速路径：未新增任何逻辑块，所有变更均为既有目录块的原地修改。
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

        // Step 1: 批量分配新增目录块所需的物理数据块及 Extent/间接树内部节点，并核算更新后的字节大小与扇区计数。
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

        // Step 2: 为每个新增逻辑块获取对应的元数据缓存窗口句柄；若中途出错则统一回滚块映射预留。
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

/// 读取既有逻辑目录块的元数据缓存窗口并拷贝出一份独立字节副本，供本次编辑局部修改。
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

/// 精确分配一块长度等于 `size`（文件系统块大小）的全零目录块内存镜像。
fn empty_block(size: usize) -> Result<Vec<u8>, FsError> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(size)
        .map_err(|_| FsError::NoMemory)?;
    bytes.resize(size, 0);
    Ok(bytes)
}

/// 计算容纳 `name_len` 字节名字与 8 字节头部所需的最小 4 字节对齐 `rec_len`。
fn minimum_record_len(name_len: usize) -> usize {
    (DIRENT_HEADER_BYTES + name_len + DIRENT_ROUND) & !DIRENT_ROUND
}

/// 将 VFS `InodeKind` 映射为 ext4 目录项头部 `file_type` 字段的磁盘编码值。
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

/// 在目录块切片 `data[offset..offset + rec_len]` 槽位中清零填充并编码一条完整的 `ext4_dir_entry_2` 记录。
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

/// 尝试在叶块有效区 `data[..end]` 中复用空闲槽位或拆分有效记录的尾部剩余空间插入新目录项，成功后重算块尾 CRC 并返回 `Ok(true)`；空间不足时保持原字节不变并返回 `Ok(false)`。
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

/// 构造一个全新的空目录叶块镜像：包含一条覆盖整个有效区的空闲记录（`inode = 0, rec_len = end`）及合法的末尾校验尾。
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

/// 校验目录增删改操作的目标文件名组件：拒绝空名、超长名、含 `\0` 或 `/` 以及保留项 `.` 和 `..`。
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

/// 校验待写入目录项的 Inode 编号非零、未超过卷总 Inode 数且可装入 32 位 `inode` 字段。
fn check_inode_number(g: &Geometry, number: u64) -> Result<(), FsError> {
    if number == 0 || g.inodes_count < number || u64::from(u32::MAX) < number {
        return Err(FsError::InvalidOperation);
    }
    Ok(())
}

/// 返回普通目录叶块中目录项有效载荷区的结束字节偏移（启用 `METADATA_CSUM` 时扣除末尾 12 字节校验尾）。
fn leaf_end(g: &Geometry) -> usize {
    g.block_size
        - if g.features_ro_compat & METADATA_CSUM != 0 {
            codec::DIR_TAIL_BYTES
        } else {
            0
        }
}

/// 删除叶块中偏移为 `target` 的目录项并重算块尾 CRC：若存在前驱记录则将 `rec_len` 并入前驱槽位；若为块首第一条记录则保留原 `rec_len` 并将 `inode` 置零，避免移动后续记录改变并发遍历游标的字节偏移。
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

/// 在目标目录项名字与长度保持不变的前提下，原地替换其指向的 Inode 编号与 `file_type` 编码并重算块尾 CRC。
#[expect(
    clippy::too_many_arguments,
    reason = "记录窗口、旧 inode 身份和新 inode/type 在同一校验与校验和更新中直接使用"
)]
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

/// 叶块分裂或升级 HTree 时提取出的有效目录项哈希与源块切片范围摘要。
#[derive(Clone, Copy)]
struct HashedRecord {
    /// 目录项名字计算出的 32 位主哈希值（`major`）。
    hash: u32,
    /// 该记录在原目录块镜像中的起始字节偏移。
    offset: usize,
    /// 目录项名字的字节长度。
    name_len: usize,
    /// 该记录在原目录块中占用的 `rec_len` 字节长度。
    rec_len: usize,
}

/// 扫描叶块内的全部非空目录项，计算各自的 `major` 哈希并按 `(hash, 原块内偏移)` 升序排列返回。
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

/// 将 `records` 列表引用的原目录项按紧凑最小长度顺序拷贝到新叶块 `destination`，拉长最后一条记录覆盖至 `end`，并重算块尾 CRC。
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

/// 计算单个 HTree 根块（`root == true`）或内部索引节点块所能容纳的最大 `dx_entry` 槽位数（`limit`）。
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

/// 将排序完毕的 `dx_entry` 切片与容量头部写入指定索引块的内存镜像，清零未使用的尾部槽位并更新 `dx_tail` 校验和。
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

/// HTree 索引节点分裂或插入期间在内存中解码出的可变节点工作集。
struct DxNodeEdit {
    /// 该索引块在目录文件内的逻辑块号。
    logical: u64,
    /// 是否为逻辑第 0 块的 `dx_root`。
    root: bool,
    /// 本次下降路径在该节点中命中的前驱条目下标（新分隔条目将插入到 `position + 1`）。
    position: usize,
    /// 根块记录的当前间接索引深度（内部节点恒为 `0`）。
    depth: u8,
    /// 该节点可容纳的最大 `dx_entry` 条目上限。
    limit: usize,
    /// 节点内当前有效的全部 `dx_entry` 条目列表。
    entries: Vec<DxEntry>,
}

/// 从编辑准备器中校验并解码 `frame` 对应索引块的全部有效 `dx_entry` 到内存向量，供插入或分裂修改。
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

/// 向 HTree 路径末端索引节点插入新叶块的分隔条目 `(hash, block)`；若节点满则自底向上分裂内部节点，直至必要时将根块下推一层并增加树深度。
fn insert_dx_block(
    builder: &mut DirectoryEditBuilder<'_>,
    path: &DxPath,
    hash: u32,
    block: u64,
) -> Result<(), FsError> {
    let mut level = path.len.checked_sub(1).ok_or(FsError::Corrupt)?;
    let mut addition = DxEntry { hash, block };
    loop {
        // Step 1: 在当前层命中槽位之后插入新条目；若未超出节点容量上限则直接写回镜像并返回。
        let frame = path.frames[level];
        let mut node = load_dx_node(builder, frame)?;
        let insertion = node.position + 1;
        node.entries.try_reserve(1).map_err(|_| FsError::NoMemory)?;
        node.entries.insert(insertion, addition);
        if node.entries.len() <= node.limit {
            return write_dx_node(builder, node.logical, node.root, node.depth, &node.entries);
        }

        // Step 2: 若根块容量溢出且未超过最大间接层数上限，分配一个新内部索引块承接原根全部条目，根块改为单条目指向该新块并将 `depth + 1`。
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

        // Step 3: 普通内部索引节点溢出时折半拆分为左右两个节点，将右半部分首项哈希向上提升插入父节点。
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

/// 分裂空间不足的 HTree 叶块：按哈希排序折半重排既有记录、写入待插入新项，并将右叶块及其起始哈希（含跨块同哈希 continuation 标志）登记到上层索引树。
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

    // Step 1: 按累计记录字节数接近半块大小寻找分裂点 `split`，计算右半区的起始哈希 `boundary` 及是否存在跨块同哈希碰撞（`continued`）。
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

    // Step 2: 预留新叶块逻辑号，将左右两半记录分别紧密打包到两个块缓冲区，再将本次新增目录项插入对应的半区。
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

    // Step 3: 更新原叶块镜像、登记新右叶块镜像，并将右叶分隔键（最低位或上 `continued`）插入 HTree 索引路径。
    builder.image(path.leaf)?.bytes = left;
    builder.push_new_image(new_logical, right)?;
    insert_dx_block(builder, path, boundary + u32::from(continued), new_logical)
}

/// 将写满的单块线性目录升级为 HTree 哈希索引目录：把除 `.` / `..` 外的既有项迁入新叶块、将第 0 块原地改写为 `dx_root`，随后对叶块执行首次分裂并插入新项。
fn make_indexed_directory(
    builder: &mut DirectoryEditBuilder<'_>,
    name: &[u8],
    number: u64,
    kind: InodeKind,
) -> Result<(), FsError> {
    let g = builder.volume.geometry();
    let inode = builder.inode;
    let version = g.default_hash_version;
    // Step 1: 校验原第 0 块的 `.` 与 `..` 记录，提取其余全部有效目录项并打包进暂存叶块缓冲区。
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

    // Step 2: 预留首个叶块逻辑号并置位 `INDEX` 标志，将第 0 块重写为包含 `.`、`..`、`dx_root_info` 与单条初始索引项的 `dx_root`。
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

    // Step 3: 构造指向该初始叶块的单层路径，复用 `split_leaf` 完成叶块拆分与本次新目录项的写入。
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

/// 在编辑准备器中插入一条新目录项：HTree 目录沿哈希路径插入或分裂叶块；线性目录优先复用既有块空闲空间，满时升级为 HTree（单块且启用 `DIR_INDEX`）或追加新线性叶块。
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

/// 预备向目录插入新目录项 `(name, number, kind)`：先完成名字合法性与重名冲突检查，再构造包含所有块修改与新增块预留的 [`PreparedDirectoryEdit`]。
///
/// 调用方需持有目标目录的写序列化锁与 `disk_root` 读锁（可睡眠上下文）。
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

/// 预备从父目录删除名为 `name` 的目录项（仅清除或合并目录项记录，不释放目标 Inode 或回收空目录块）。
///
/// 调用方需持有父目录的写序列化锁与 `disk_root` 读锁（可睡眠上下文）。
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

/// 预备将父目录中已存在的目录项 `name` 原地改指新的目标 `(number, kind)`（用于跨目录 `rename` 覆盖已存在目标）。
///
/// 调用方需持有目标父目录的写序列化锁与 `disk_root` 读锁（可睡眠上下文）。
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

/// 预备同一父目录内的 `rename` 操作：在同一个目录编辑事务中原子完成源目录项 `old_name` 的移除与目标项 `new_name` 的新建或覆盖替换。
///
/// 调用方需持有该目录的写序列化锁与 `disk_root` 读锁；若新旧名字相同或已指向同一 Inode，返回 `changed() == false` 的空编辑。
#[expect(
    clippy::too_many_arguments,
    reason = "同父 rename 在同一旧根上核对源与覆盖目标并准备一份目录编辑"
)]
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
    // 分支 1: 目标名字已存在且允许覆盖——先原地替换目标记录的 Inode 身份，再删除源记录（两者可能位于同一叶块或不同叶块）。
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
        // 分支 2: 目标名字不存在——先删除源记录释放块内槽位，再插入新名字（可能触发叶块分裂或升级 HTree）。
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

/// 预备更新被跨目录移动的子目录第 0 块中 `..` 记录指向的新父目录 Inode 编号，并按当前根块布局重算尾部校验和。
///
/// 调用方需持有被移动子目录的写序列化锁与 `disk_root` 读锁。
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

/// 检查单个目录叶块是否不包含任何用户子项（仅允许非索引目录逻辑第 0 块的固定槽位出现 `.` 与 `..`）。
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

/// 直接扫描磁盘目录块校验目录是否为空（仅含合法的 `.` 与 `..` 记录），不依赖内存 dentry 缓存的局部视图。
///
/// 调用方需持有目标目录的读/写保护以及 `disk_root` 块映射读保护。
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

/// 为尚未链接到任何父目录的全新空目录 Inode 准备初始目录块（写入 `.` 与 `..` 记录，若预设了 `INDEX` 标志则同时构造初始 `dx_root` 与首个空叶块）。
///
/// 调用上下文：新目录 Inode 分配后的可睡眠准备阶段。
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
        // 交换镜像顺序使初始空叶块排在根块之前，确保 `publish` 时先初始化叶块内容再写入指向它的根块。
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
