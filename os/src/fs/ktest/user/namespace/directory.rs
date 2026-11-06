//! ext4 HTree 索引目录分裂、传统线性目录升级、目录游标（cookie）定位与共享挂载边界测试。
//!
//! # 模块职责与架构定位
//! 本模块位于 `crate::fs::ktest::user::namespace::directory`，负责对 VFS 目录命名空间与底层 ext4 目录索引机制进行端到端集成测试。
//! - **上游调用者**：由父模块 [`crate::fs::ktest::user::namespace::anchor`] 引用注册槽，交由 `ktest` 框架调度运行。
//! - **下游被测子系统**：`Vfs` 路径解析与目录修改接口（`mkdir_from`、`link_from`、`rename_from`、`unlink_from`、`rmdir_from`、`mount`/`unmount`）、`FileTable` 打开文件描述符与目录读取游标（`readdir`/`seek`/`for_each_dirent`）、`Ext4Volume` HTree（`dx_root`/`dx_node`）索引树分裂与 CRC32C 校验，以及 `writeback` 与 `InodeCache` 的冷盘重开持久化路径。
//!
//! # 核心设计与关键布局
//!
//! ## 1. ext4 HTree 目录索引两级结构（`depth = 1`）
//! 当目录项超出单个叶子块容量时，ext4 将逻辑块 0 转化为 `dx_root`，并在树深为 1 时引入中间索引块 `dx_node`：
//!
//! ```text
//! 逻辑块 0: dx_root (Root)
//! ┌──────────────────────────────────────────────────────────┐
//! │ fake "." & ".." (24B) │ dx_root_info (8B: version@28,   │
//! │                       │               depth@30)          │
//! ├───────────────────────┴──────────────────────────────────┤
//! │ dx_countlimit @32 (limit: u16, count: u16)               │
//! │ entry[0]: (hash=0,        block=node_logical_0)          │
//! │ entry[1]: (hash=h_root_1, block=node_logical_1)          │
//! │ ...                                                      │
//! └──────────────┬───────────────────────────────────────────┘
//!                │
//!                ▼
//! 逻辑块 node_logical_i: dx_node (Index)
//! ┌──────────────────────────────────────────────────────────┐
//! │ fake_dirent (8B) │ dx_countlimit @8 (limit: u16, count)  │
//! │ entry[0]: (hash=0,        block=leaf_logical_0)          │
//! │ entry[1]: (hash=h_node_1, block=leaf_logical_1)          │
//! │ ...                                                      │
//! └──────────────┬───────────────────────────────────────────┘
//!                │
//!                ▼
//! 逻辑块 leaf_logical_j: Directory Leaf
//! ┌──────────────────────────────────────────────────────────┐
//! │ ext4_dir_entry_2 线性记录序列 ... │ 尾部 checksum (12B)  │
//! └──────────────────────────────────────────────────────────┘
//! ```
//!
//! ## 2. 传统目录升级与哈希碰撞游标状态流
//! ```text
//! 普通线性目录 (flags & INDEX == 0)
//!   │  连续硬链接 255B 长文件名填满首块，再插入 trigger_record
//!   ▼
//! 升级为 HTree 索引目录 (flags & INDEX != 0, depth = 0)
//!   │  1. 验证升级前已打开并 dup 的文件描述符仍能推进 readdir 游标
//!   │  2. 按测试目标设置 hash_version (0 = legacy signed, 3 = legacy unsigned)
//!   │  3. 插入一对具有相同 (major, minor=0) 哈希值的碰撞文件名
//!   ▼
//! 验证独立打开、dup 共享游标、短回调中断（返回 Ok(false)）重读及 SeekFrom::Start(cookie) 精确恢复
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试运行在可睡眠内核线程（`context = "Thread"`）中。
//! - 读取底层目录块（[`directory_block`]）时，单次 `BufferRef::with_read` 借用在拷贝至栈缓冲区后立即释放，随后才进行下一块读取或调用 `Inode::with_map_read`，避免跨元数据块锁与 Inode 映射锁嵌套。
//! - 冷盘重开（[`cold_reopen_namespace`]）遵循严格的资源退役顺序：先刷盘并清理测试夹具持有的 `Path`/`InodeRef`/`Vfs` 强引用，再通过 `CacheWatch::wait_retired` 等待后台写回任务彻底退出，确认旧 `InodeCache`、`Ext4Volume` 与元数据缓存的 `Weak` 引用全部失效后，才在同一块设备上重建新缓存与新卷实例。

use alloc::sync::Arc;
use alloc::vec::Vec;
use virtio_drivers::BufferDirection;

use crate::block::{AccessMode, Bio, BioAllocator, BlkOperation, DiskRef, SECTOR_BYTES};
use crate::fs::ext4::{codec, BlockMapping, Ext4Volume};
use crate::fs::ktest::user::write::WriteOwner;
use crate::fs::ktest::user::UserSuite;
use crate::fs::ktest::*;
use crate::fs::{
    open_path, DescriptorFlags, DirectoryEntry, FileRef, FileTable, FsContext, FsError, InodeCache,
    InodeKind, InodeMetadata, InodeRef, OpenOptions, Path, ResolveMode, SeekFrom, Vfs,
};
use crate::mm::dma::MappedPages;

use super::{directory_block, prepare_namespace_view, NamespaceView};

/// ext4 Inode 启用 HTree 哈希索引目录的标志位（`EXT4_INDEX_FL = 0x1000`）。
const INDEX: u32 = 0x1000;

/// 用于在既有 HTree 目录中构造含非 UTF-8 字节（`0xff`）的 255 字节长文件名前缀。
const LONG_NAME_PREFIX: &[u8] = b"w27-\xff-";
/// 有符号传统哈希（`DX_HASH_LEGACY = 0`）目录升级测试所使用的文件名前缀。
const PROMOTION_PREFIX_SIGNED: &[u8] = b"signed-";
/// 无符号传统哈希（`DX_HASH_LEGACY_UNSIGNED = 3`）目录升级测试所使用的文件名前缀。
const PROMOTION_PREFIX_UNSIGNED: &[u8] = b"unsigned-";
/// 搜索落入指定 HTree 哈希区间的候选文件名时的最大尝试次数上限。
const HASH_CANDIDATE_LIMIT: u32 = 1_048_576;
/// 批量创建硬链接或目录项时触发一次元数据与数据同步落盘的批次大小。
const DIRECTORY_WRITE_BATCH: usize = 8;

/// HTree 中间索引块（`dx_node`）的拓扑与容量快照。
#[derive(Clone, Copy)]
struct DxNode {
    /// 该中间索引节点在根块（`dx_root`）索引表中的槽位下标（`0..root_count`）。
    root_position: usize,
    /// 该中间索引块在目录 Inode 内的逻辑块号。
    logical: u64,
    /// 该中间索引块当前已使用的索引项数量（`dx_countlimit.count`）。
    count: usize,
    /// 该中间索引块最多可容纳的索引项上限（`dx_countlimit.limit`）。
    limit: usize,
}

/// HTree 索引目录的整体树形结构与块映射快照。
#[derive(Clone)]
struct HTreeShape {
    /// 目录 Inode 的元数据快照。
    metadata: InodeMetadata,
    /// 目录 Inode 的 60 字节块映射根数组（`i_block`）。
    map_root: [u8; 60],
    /// 目录逻辑块 0（即 `dx_root` 根块）的完整原始字节拷贝。
    root: [u8; mm::PAGE_SIZE],
    /// HTree 间接索引层级深度（`0` 表示根块直接指向叶子块，`1` 表示经过一层 `dx_node` 中间索引块）。
    depth: u8,
    /// 根块（`dx_root`）当前已使用的索引项数量。
    root_count: usize,
    /// 根块（`dx_root`）最多可容纳的索引项上限。
    root_limit: usize,
    /// 整棵 HTree 当前挂载的目录叶子块总数量。
    leaves: usize,
    /// 当 `depth == 1` 时，按根块槽位顺序收集的全部中间索引节点（[`DxNode`]）列表；当 `depth == 0` 时为空。
    nodes: Vec<DxNode>,
}

/// 带有单调递增序号的固定 255 字节原始目录项名称。
#[derive(Clone, Copy)]
struct RawName {
    /// 生成该文件名时使用的 32 位无符号序号（编码为文件名中的 8 位十六进制字符）。
    nonce: u32,
    /// 长度为 255 字节的原始文件名字节数组（可包含非 UTF-8 字节 `0xff`）。
    bytes: [u8; 255],
}

/// 由普通线性目录升级为 HTree 索引目录后的测试样本上下文。
struct LegacyDirectory {
    /// 该目录在根命名空间下的静态路径字节串（如 `b"/legacy-signed"`）。
    pathname: &'static [u8],
    /// 该目录已解析的 VFS [`Path`] 实例。
    path: Path,
    /// 该目录对应的 [`InodeRef`] 引用。
    inode: InodeRef,
    /// 升级过程中及升级后已成功插入该目录的全部文件名列表（不含 `"."` 与 `".."`）。
    names: Vec<Vec<u8>>,
    /// 专门构造的一对在当前目录哈希版本下具有完全相同哈希值的碰撞文件名样本。
    collision: [&'static [u8]; 2],
}

/// 在指定 `dx_root` 根索引槽位所覆盖的哈希区间内搜索一个候选长文件名。
///
/// # 调用上下文
/// 纯内存计算与块哈希校验辅助函数，不涉及睡眠或 I/O。通过递增 `next_nonce` 构造 255 字节文件名，
/// 计算目录哈希并通过二分查找 [`dx_select`] 判断其是否落入 `root_position` 槽位。
///
/// # 参数与返回值
/// - `volume`：用于获取文件系统几何参数与哈希种子的 ext4 卷。
/// - `shape`：目标目录的 HTree 结构快照（使用其 `root` 块字节与 `root_count`）。
/// - `root_position`：期望命中的根索引槽位下标。
/// - `next_nonce`：单调递增的候选序号状态，每次尝试后自增 1。
/// - 成功时返回命中的 [`RawName`]；若超出 [`HASH_CANDIDATE_LIMIT`] 次仍未命中或序号溢出则返回 [`FsError::NoSpace`]。
fn name_for_root_range(
    volume: &Ext4Volume,
    shape: &HTreeShape,
    root_position: usize,
    next_nonce: &mut u32,
) -> Result<RawName, FsError> {
    for _ in 0..HASH_CANDIDATE_LIMIT {
        let nonce = *next_nonce;
        *next_nonce = next_nonce.checked_add(1).ok_or(FsError::NoSpace)?;
        let bytes = raw_long_name(LONG_NAME_PREFIX, nonce);
        let (major, _) = codec::directory_hash(volume.geometry(), shape.root[28], &bytes)?;
        if dx_select(&shape.root, 32, shape.root_count, major)? == root_position {
            return Ok(RawName { nonce, bytes });
        }
    }
    Err(FsError::NoSpace)
}

/// 在指定中间索引节点（`dx_node`）当前最宽的叶子哈希区间内搜索一个候选长文件名。
///
/// # 设计原因与步骤
/// 为了用最少的文件创建次数触发目标叶子块分裂并进而撑满目标 `dx_node`：
/// 1. 读取 `node.logical` 对应的 `dx_node` 块，遍历其所有叶子槽位 `0..node.count`，计算每个槽位覆盖的 32 位主哈希区间宽度 `upper - lower`，选出哈希跨度最宽的槽位 `selected`（跨度越宽，随机哈希命中的概率越高，且叶子分裂后会自动重新选择新的最宽区间）。
/// 2. 在 [`HASH_CANDIDATE_LIMIT`] 次预算内递增 `next_nonce`，找到同时在 `dx_root` 命中 `node.root_position` 且在 `dx_node` 命中 `selected` 槽位的 255 字节长文件名。
///
/// # 参数与返回值
/// - `volume`：目标 ext4 卷。
/// - `shape`：当前 HTree 快照。
/// - `node`：待填充并诱导分裂的目标中间索引节点。
/// - `next_nonce`：候选文件名序号计数器。
/// - 成功时返回命中的 [`RawName`]；失败时返回对应的 [`FsError`]。
fn name_for_split_leaf(
    volume: &Ext4Volume,
    shape: &HTreeShape,
    node: DxNode,
    next_nonce: &mut u32,
) -> Result<RawName, FsError> {
    // Step 1: 读取目标 `dx_node` 块，遍历其所有槽位找出覆盖 32 位哈希空间最宽的叶子槽位 `selected`。
    let node_bytes = directory_block(volume, &shape.metadata, &shape.map_root, node.logical)?;
    let mut selected = 0;
    let mut widest = 0;
    for position in 0..node.count {
        let lower = if position == 0 {
            if node.root_position == 0 {
                0
            } else {
                u64::from(codec::le32(&shape.root, 32 + node.root_position * 8)? & !1)
            }
        } else {
            u64::from(codec::le32(&node_bytes, 8 + position * 8)? & !1)
        };
        let upper = if position + 1 < node.count {
            u64::from(codec::le32(&node_bytes, 8 + (position + 1) * 8)? & !1)
        } else if node.root_position + 1 < shape.root_count {
            u64::from(codec::le32(&shape.root, 32 + (node.root_position + 1) * 8)? & !1)
        } else {
            1u64 << 32
        };
        let width = upper.checked_sub(lower).ok_or(FsError::Corrupt)?;
        if widest < width {
            selected = position;
            widest = width;
        }
    }

    // Step 2: 递增 `next_nonce` 搜索同时命中根槽位 `node.root_position` 与叶子槽位 `selected` 的长文件名。
    for _ in 0..HASH_CANDIDATE_LIMIT {
        let nonce = *next_nonce;
        *next_nonce = next_nonce.checked_add(1).ok_or(FsError::NoSpace)?;
        let bytes = raw_long_name(LONG_NAME_PREFIX, nonce);
        let (major, _) = codec::directory_hash(volume.geometry(), shape.root[28], &bytes)?;
        if dx_select(&shape.root, 32, shape.root_count, major)? == node.root_position
            && dx_select(&node_bytes, 8, node.count, major)? == selected
        {
            return Ok(RawName { nonce, bytes });
        }
    }
    Err(FsError::NoSpace)
}

/// 遍历 HTree 目录验证所有新插入的长文件名均能被枚举，并验证回调中途拒绝（返回 `Ok(false)`）时的游标回退与重读语义。
///
/// # 调用上下文
/// 在可睡眠测试线程中调用。会打开 `path` 对应的目录文件并通过 `for_each_dirent` 和 `readdir` 执行目录遍历。
///
/// # 验证契约
/// 1. 枚举过程中所有以 [`LONG_NAME_PREFIX`] 开头的文件名必须精确匹配 `expected` 集合中的某一项，指向 `inode_number` 且无重复。
/// 2. 当遇到最后一个期望项（`found == expected.len()`）时，令回调返回 `Ok(false)` 模拟用户态缓冲区不足导致的提前终止。
/// 3. 此时文件游标（`SeekFrom::Current(0)`）必须停留在前一项提交后的 `committed_cookie`，而紧接着调用一次 `file.readdir()` 必须重新交付刚才被回调拒绝的最后一项。
fn verify_indexed_new_names(
    context: &mut TestContext<'_>,
    vfs: &Vfs,
    path: &Path,
    expected: &[RawName],
    inode_number: u64,
) -> TestResult {
    if expected.is_empty() {
        return Err(TestError("indexed directory name set is empty"));
    }
    // Step 1: 打开目录并调用 `for_each_dirent` 遍历，遇到最后一个目标长文件名时返回 `Ok(false)` 中断交付。
    let file = checked(context, open_path(vfs, path.clone()))?;
    let mut seen = Vec::new();
    seen.try_reserve_exact(expected.len())
        .map_err(|_| TestError("indexed name bitmap OOM"))?;
    seen.resize(expected.len(), false);
    let mut found = 0usize;
    let mut committed_cookie = checked(context, file.seek(SeekFrom::Current(0)))?;
    let mut declined_cookie = committed_cookie;
    let mut declined_number = 0u64;
    let mut declined_name = [0u8; 255];
    let mut declined_len = 0usize;
    file.for_each_dirent(&mut |entry| {
        if entry.name.starts_with(LONG_NAME_PREFIX) {
            let nonce = raw_name_nonce(&entry.name).ok_or(FsError::Corrupt)?;
            let Some(index) = expected.iter().position(|name| name.nonce == nonce) else {
                return Err(FsError::Corrupt);
            };
            if seen[index]
                || expected[index].bytes.as_slice() != entry.name.as_slice()
                || entry.number != inode_number
                || entry.kind != Some(InodeKind::Regular)
            {
                return Err(FsError::Corrupt);
            }
            seen[index] = true;
            found += 1;
            if found == expected.len() {
                if 255 < entry.name.len() {
                    return Err(FsError::Corrupt);
                }
                declined_cookie = entry.next_cookie;
                declined_number = entry.number;
                declined_len = entry.name.len();
                declined_name[..declined_len].copy_from_slice(&entry.name);
                return Ok(false);
            }
        }
        committed_cookie = entry.next_cookie;
        Ok(true)
    })
    .map_err(|error| {
        context.diagnostic(format_args!("W27 indexed readdir error={:?}", error));
        TestError("indexed directory readdir failed")
    })?;
    context.expect_eq(found, expected.len());
    context.expect_eq(seen.iter().all(|present| *present), true);
    context.expect_eq(
        checked(context, file.seek(SeekFrom::Current(0)))?,
        committed_cookie,
    );

    // Step 2: 验证游标停留在被拒项之前的位置，且后续单次 `readdir` 能够完整重读该被拒项。
    let declined = checked(context, file.readdir())?
        .ok_or(TestError("indexed short delivery lost its final entry"))?;
    context.expect_eq(declined.next_cookie, declined_cookie);
    context.expect_eq(declined.number, declined_number);
    context.expect_eq(declined.name.as_slice(), &declined_name[..declined_len]);
    Ok(())
}

/// 从固定 255 字节长文件名中解析出紧随 [`LONG_NAME_PREFIX`] 之后的 8 位十六进制 `nonce` 序号。
///
/// 直接在原始字节切片上解析 `'0'..='9'` 与 `'a'..='f'`，避免将包含 `0xff` 字节的名称转换为 UTF-8 字符串。
/// 若长度不为 255、前缀不匹配或包含非法十六进制字节则返回 `None`。
fn raw_name_nonce(name: &[u8]) -> Option<u32> {
    if name.len() != 255 || !name.starts_with(LONG_NAME_PREFIX) {
        return None;
    }
    let digits = name.get(LONG_NAME_PREFIX.len()..LONG_NAME_PREFIX.len() + 8)?;
    let mut value = 0u32;
    for byte in digits {
        let nibble = match *byte {
            b'0'..=b'9' => *byte - b'0',
            b'a'..=b'f' => *byte - b'a' + 10,
            _ => return None,
        };
        value = value.checked_mul(16)?.checked_add(u32::from(nibble))?;
    }
    Some(value)
}

/// 在挂载文件系统前直接读写底层块设备，将 ext4 超级块的默认哈希版本修改为无符号传统哈希（`default_hash_version = 3`）。
///
/// # 调用上下文
/// 在可睡眠的测试线程上下文中、首次挂载 `Ext4Volume` 之前调用。此时测试夹具独占底层 `disk`，尚无任何上层缓存或并发 I/O。
///
/// # 执行步骤
/// 1. 按块设备逻辑块大小对齐计算出覆盖字节偏移 `[1024, 2048)`（ext4 主超级块位置）的读写区间 `[start, end)`。
/// 2. 读取包含超级块的磁盘扇区，解码校验原始超级块后，将偏移 `252`（`s_def_hash_version`）设为 `3`（`DX_HASH_LEGACY_UNSIGNED`）。
/// 3. 若启用了 `METADATA_CSUM`，重新计算超级块前 1020 字节的 CRC32C 校验和并写入末尾 4 字节。
/// 4. 分配一页 DMA 缓冲区，构造 `BlkOperation::Write` 类型的 [`Bio`] 写回磁盘，等待完成并排空完成队列。
fn select_unsigned_legacy_default(context: &mut TestContext<'_>, disk: &DiskRef) -> TestResult {
    // Step 1: 计算包含 ext4 主超级块（偏移 1024..2048）的块对齐读写范围，读取并修改 `default_hash_version = 3` 及 CRC32C 校验和。
    let limits = disk.limits();
    let logical = limits.logical_block_size as usize;
    if !matches!(logical, 512 | 1024 | 2048 | 4096) {
        return Err(TestError(
            "legacy hash preparation has unsupported Disk geometry",
        ));
    }

    let start = 1024 / logical * logical;
    let end = 2048_usize.div_ceil(logical) * logical;
    let length = end - start;
    let capacity = limits
        .capacity_sectors
        .checked_mul(SECTOR_BYTES)
        .ok_or(TestError("legacy hash preparation Disk capacity overflow"))?;
    let mut bytes = [0u8; mm::PAGE_SIZE];
    checked(
        context,
        disk.read_exact_at(start as u64, &mut bytes[..length])
            .map_err(FsError::from),
    )?;
    let super_offset = 1024 - start;
    let superblock = &mut bytes[super_offset..super_offset + 1024];
    let original = checked(context, codec::decode_super(superblock, capacity))?;
    superblock[252] = 3;
    if original.features_ro_compat & codec::METADATA_CSUM != 0 {
        let checksum = codec::crc32c(!0, &superblock[..1020]);
        superblock[1020..1024].copy_from_slice(&checksum.to_le_bytes());
    }

    let updated = checked(context, codec::decode_super(superblock, capacity))?;
    context.expect_eq(updated.default_hash_version, 3);
    context.expect_eq(updated.hash_unsigned, original.hash_unsigned);
    context.expect_eq(updated.hash_seed, original.hash_seed);
    context.expect_eq(updated.uuid, original.uuid);

    // Step 2: 分配 DMA 页面并通过 `Bio` 将更新后的超级块扇区写回底层磁盘，等待 I/O 完成。
    let pages = mm::alloc_pages(0, true).ok_or(TestError("legacy hash write page OOM"))?;
    let mut mapped = MappedPages::new(pages, 0, length, BufferDirection::DriverToDevice)
        .map_err(|_| TestError("legacy hash write mapping failed"))?;
    for (target, source) in mapped.bytes_mut().iter_mut().zip(&bytes[..length]) {
        target.write(*source);
    }
    let mut memory = Vec::new_in(BioAllocator::Heap);
    memory
        .try_reserve_exact(1)
        .map_err(|_| TestError("legacy hash Bio slot OOM"))?;
    memory.push(mapped);
    // SAFETY:
    // 1. `mapped.bytes_mut()` 前 `length` 字节已通过上方的循环完整写入超级块扇区数据，且可变借用在循环结束后已释放。
    // 2. `memory` 向量独占持有该 `MappedPages` 实例并在 `Bio::try_new` 时移交 DMA 访问权，直到下方 `completion.wait()` 返回并调用 `take_memory()` 收回物理页前，CPU 不会再访问该缓冲区。
    // 3. `disk` 引用在当前函数栈帧内全程保持存活，直到 `wait()` 确认设备中断与完成队列 Work 任务全部执行完毕。
    let bio = unsafe {
        Bio::try_new(
            disk.as_ref().get_ref(),
            BlkOperation::Write,
            start as u64 / SECTOR_BYTES,
            memory,
            0,
            None,
        )
    }
    .map_err(|error| {
        context.diagnostic(format_args!(
            "legacy-hash-super-write prepare-error={:?}",
            error
        ));
        TestError("legacy hash superblock Bio preparation failed")
    })?;
    let completion = bio.completion();
    disk.submit_bio(bio);
    let result = completion.wait();
    context.expect_eq(result.completed, length);
    context.expect_eq(result.error, None);
    drop(
        completion
            .take_memory()
            .map_err(|_| TestError("legacy hash superblock memory reclaim failed"))?,
    );
    disk.test_flush_completions();
    context.expect_eq(disk.test_snapshot().inflight, 0);
    context.diagnostic(format_args!(
        "W27-superblock default-hash-version=3 unsigned-legacy-explicit=1 original-hash-flags-preserved=1 metadata-csum-recomputed={} raw-write-completion=1",
        original.features_ro_compat & codec::METADATA_CSUM != 0
    ));
    Ok(())
}

/// 读取并校验目录 Inode 的 HTree 根块（`dx_root`）、中间索引块（`dx_node`）及可选的所有叶子块布局与 CRC 校验和。
///
/// # 调用上下文
/// 在可睡眠的测试线程中调用。通过 `inode.with_map_read` 获取元数据与块映射快照，随后逐块调用 [`directory_block`] 读取并调用 `codec::verify_directory` 验证。
///
/// # 参数与返回值
/// - `volume`：目标 ext4 卷。
/// - `inode`：待检查的索引目录 Inode 引用（必须为目录且设置了 [`INDEX`] 标志）。
/// - `verify_leaves`：若为 `true`，则额外读取每一个叶子块并验证其目录项结构与尾部校验和；若为 `false`，仅验证 `dx_root` 与 `dx_node` 索引块以加快高频插入循环中的拓扑检查。
/// - 成功时返回描述当前树深度、根槽位数、中间节点列表及叶子总数的 [`HTreeShape`]；若布局或校验和非法则返回 [`FsError`]。
fn htree_shape(
    volume: &Ext4Volume,
    inode: &InodeRef,
    verify_leaves: bool,
) -> Result<HTreeShape, FsError> {
    // Step 1: 读取并验证逻辑块 0 为合法的 `dx_root` 根块，解析树深度 `depth` 与根索引槽位数。
    let (metadata, map_root) = inode.with_map_read(|metadata, root| Ok((*metadata, *root)))?;
    if metadata.kind != InodeKind::Directory || metadata.flags & INDEX == 0 {
        return Err(FsError::NotDirectory);
    }
    let geometry = volume.geometry();
    let root = directory_block(volume, &metadata, &map_root, 0)?;
    if codec::verify_directory(geometry, &metadata, 0, &root[..geometry.block_size], false)?
        != codec::DirectoryLayout::Root
    {
        return Err(FsError::Corrupt);
    }
    let depth = root[30];
    if depth > 1 {
        return Err(FsError::Unsupported);
    }

    let root_count = usize::from(codec::le16(&root, 34)?);
    let root_limit = usize::from(codec::le16(&root, 32)?);
    let mut nodes = Vec::new();
    nodes
        .try_reserve_exact(root_count)
        .map_err(|_| FsError::NoMemory)?;
    let mut leaves = 0;
    // Step 2: 遍历根块索引槽位，按 `depth` 校验直接叶子块或 `dx_node` 中间索引块及其下属叶子块的布局与 CRC。
    for root_position in 0..root_count {
        let root_entry = 32 + root_position * 8;
        let logical = u64::from(codec::le32(&root, root_entry + 4)? & 0x0fff_ffff);
        if depth == 0 {
            if verify_leaves {
                let leaf = directory_block(volume, &metadata, &map_root, logical)?;
                if !matches!(
                    codec::verify_directory(
                        geometry,
                        &metadata,
                        logical,
                        &leaf[..geometry.block_size],
                        false,
                    )?,
                    codec::DirectoryLayout::Leaf { .. }
                ) {
                    return Err(FsError::Corrupt);
                }
            }
            leaves += 1;
            continue;
        }
        let node_bytes = directory_block(volume, &metadata, &map_root, logical)?;
        if codec::verify_directory(
            geometry,
            &metadata,
            logical,
            &node_bytes[..geometry.block_size],
            true,
        )? != codec::DirectoryLayout::Index
        {
            return Err(FsError::Corrupt);
        }
        let count = usize::from(codec::le16(&node_bytes, 10)?);
        let limit = usize::from(codec::le16(&node_bytes, 8)?);
        nodes.push(DxNode {
            root_position,
            logical,
            count,
            limit,
        });
        leaves += count;
        if verify_leaves {
            for position in 0..count {
                let entry = 8 + position * 8;
                let leaf_logical = u64::from(codec::le32(&node_bytes, entry + 4)? & 0x0fff_ffff);
                let leaf = directory_block(volume, &metadata, &map_root, leaf_logical)?;
                if !matches!(
                    codec::verify_directory(
                        geometry,
                        &metadata,
                        leaf_logical,
                        &leaf[..geometry.block_size],
                        false,
                    )?,
                    codec::DirectoryLayout::Leaf { .. }
                ) {
                    return Err(FsError::Corrupt);
                }
            }
        }
    }
    Ok(HTreeShape {
        metadata,
        map_root,
        root,
        depth,
        root_count,
        root_limit,
        leaves,
        nodes,
    })
}

/// 按照 ext4 HTree 索引表的前驱二分查找规则，计算给定 32 位主哈希值 `hash` 应路由到的索引槽位下标。
///
/// # 算法说明
/// `dx_root`（偏移 32 起）和 `dx_node`（偏移 8 起）的每个索引项占 8 字节，其中第 0 项不存储哈希阈值（隐式代表区间下界 `0`），
/// 第 `1..count` 项的前 4 字节存储升序排列的 32 位哈希下界。本函数通过二分查找返回满足 `entry_hash <= hash` 的最大槽位下标 `0..count`。
fn dx_select(data: &[u8], offset: usize, count: usize, hash: u32) -> Result<usize, FsError> {
    let mut low = 1usize;
    let mut high = count;
    while low < high {
        let middle = low + (high - low) / 2;
        if codec::le32(data, offset + middle * 8)? <= hash {
            low = middle + 1;
        } else {
            high = middle;
        }
    }
    Ok(low - 1)
}

/// 构造长度为 255 字节的固定格式原始目录项名称。
///
/// 格式为：`prefix` + 8 位小写十六进制编码的 `nonce` + 填充至 255 字节的 `'n'`（`0x6e`）。
/// 若 `prefix` 中包含 `0xff` 等非 UTF-8 字节，生成的名称同样包含非 UTF-8 字节，用于测试内核路径与目录层对原始字节文件名的透明支持。
fn raw_long_name(prefix: &[u8], nonce: u32) -> [u8; 255] {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut name = [b'n'; 255];
    name[..prefix.len()].copy_from_slice(prefix);
    for index in 0..8 {
        let shift = (7 - index) * 4;
        name[prefix.len() + index] = HEX[((nonce >> shift) & 0xf) as usize];
    }
    name
}

/// 将目录项名称切片精确复制为独立分配的 `Vec<u8>`。
///
/// 使用 `try_reserve_exact` 按实际长度精确分配堆内存，避免容量翻倍带来的额外内存开销，并在内存不足时返回 [`TestError`]。
fn copy_name(name: &[u8]) -> Result<Vec<u8>, TestError> {
    let mut copy = Vec::new();
    copy.try_reserve_exact(name.len())
        .map_err(|_| TestError("directory name copy OOM"))?;
    copy.extend_from_slice(name);
    Ok(copy)
}

/// 将源硬链接文件与目标目录的脏数据及元数据同步刷盘，并排空块设备完成队列。
///
/// 在批量插入目录项的过程中定期调用，确保脏缓存及时落盘回收，控制单次测试的内存与块缓存占用。
fn sync_namespace_batch(
    context: &mut TestContext<'_>,
    disk: &DiskRef,
    source: &FileRef,
    directory: &FileRef,
) -> TestResult {
    checked(context, source.sync_range(0..u64::MAX, false))?;
    checked(context, directory.sync_range(0..u64::MAX, false))?;
    disk.test_flush_completions();
    Ok(())
}

/// 创建普通线性目录并通过填满首块触发向 HTree 索引目录升级，同时验证升级前打开的目录游标与指定哈希版本下的碰撞处理。
///
/// # 调用上下文
/// 在可睡眠测试线程中调用。
///
/// # 执行步骤
/// 1. 创建新目录 `pathname`，并在其仍为普通线性目录（`flags & INDEX == 0`）时打开文件描述符 `directory_fd` 并复制出 `duplicate_fd`。
/// 2. 根据块大小与尾部校验和长度精确计算首块可容纳的 255 字节长目录项数量 `long_count` 以及刚好撑破首块触发 HTree 升级的边界项长度 `trigger_len`。
/// 3. 依次硬链接 `long_count` 个长文件名与 1 个 `trigger` 文件名，断言目录 Inode 的 [`INDEX`] 标志位被置位。
/// 4. 使用升级前打开的 `directory_fd` 与 `duplicate_fd` 调用 `readdir`，验证目录结构升级为 HTree 后旧打开文件描述（OFD）的游标仍能正确读取 `"."` 并继续向后推进。
/// 5. 若目标 `hash_version` 不等于默认的 `3`（例如需要测试有符号传统哈希 `0`），调用 [`set_root_hash_version`] 原地更新 `dx_root` 的哈希版本与校验和。
/// 6. 验证传入的 `collision` 两个文件名在目标哈希版本下产生完全相同的 `(major, minor=0)` 哈希值且不与已有文件名冲突，随后将它们硬链接入目录。
#[expect(
    clippy::too_many_arguments,
    reason = "目录提升用例直接给出原视图、源 OFD、名字前缀、哈希版本与独立碰撞样本"
)]
fn promote_legacy_directory(
    context: &mut TestContext<'_>,
    fixture: &mut WriteOwner,
    view: &NamespaceView,
    source_file: &FileRef,
    pathname: &'static [u8],
    prefix: &[u8],
    hash_version: u8,
    collision: [&'static [u8]; 2],
) -> Result<LegacyDirectory, TestError> {
    // Step 1: 创建普通线性目录，并在升级前打开文件描述符且调用 dup 复制一份共享游标。
    checked(
        context,
        view.vfs.mkdir_from(
            &view.context,
            &view.root,
            pathname,
            0o777,
            (0, 0),
            view.context.umask(),
        ),
    )?;
    let directory_file = fixture.open(context, pathname, 0)?;
    let directory_table = checked(context, FileTable::try_new())?;
    let directory_fd = checked(context, directory_table.reserve(1024))?
        .publish(directory_file.clone(), DescriptorFlags::default());
    let duplicate_fd = checked(context, directory_table.dup(directory_fd, 1024))?;
    let directory = directory_file.path().unwrap().clone();
    let inode = checked(context, directory.inode())?.clone();
    context.expect_eq(
        checked(context, inode.metadata_snapshot())?.flags & INDEX,
        0,
    );

    // Step 2: 计算首个目录块的可用空间，算出一批 255 字节长记录和恰好触发 HTree 升级的边界记录长度。
    let geometry = view.volume.geometry();
    let tail = if geometry.features_ro_compat & codec::METADATA_CSUM != 0 {
        12
    } else {
        0
    };
    let leaf_end = geometry
        .block_size
        .checked_sub(tail)
        .ok_or(TestError("promotion geometry has no leaf tail"))?;
    let long_record = (8 + 255 + 3) & !3;
    let long_count = leaf_end
        .checked_sub(24)
        .ok_or(TestError("promotion geometry cannot hold dot entries"))?
        / long_record;
    let leaf_free = leaf_end - long_count * long_record;
    let ordinary_free = leaf_free
        .checked_sub(24)
        .ok_or(TestError("promotion geometry cannot reserve dot records"))?;
    let trigger_record = (ordinary_free / 4 + 1) * 4;
    if leaf_free < trigger_record || trigger_record < 8 {
        return Err(TestError("promotion trigger does not fit the indexed leaf"));
    }
    let trigger_len = trigger_record - 8;
    if trigger_len < prefix.len() + 8 || 255 < trigger_len {
        return Err(TestError("promotion trigger component is invalid"));
    }

    // Step 3: 写入 `long_count` 个长目录项填满首块，再写入 `trigger` 目录项触发目录升级为 HTree（设置 `INDEX` 标志）。
    let mut names = Vec::new();
    names
        .try_reserve_exact(long_count + 3)
        .map_err(|_| TestError("legacy directory name storage OOM"))?;
    let mut writes = 0usize;
    for nonce in 0..long_count {
        let name = raw_long_name(prefix, nonce as u32);
        checked(
            context,
            view.vfs
                .link_from(&view.source, &view.context, &directory, &name, (0, 0)),
        )?;
        names.push(copy_name(&name)?);
        writes += 1;
        if writes % DIRECTORY_WRITE_BATCH == 0 {
            sync_namespace_batch(context, view.volume.disk(), source_file, &directory_file)?;
        }
    }

    let trigger = raw_long_name(prefix, u32::MAX);
    checked(
        context,
        view.vfs.link_from(
            &view.source,
            &view.context,
            &directory,
            &trigger[..trigger_len],
            (0, 0),
        ),
    )?;
    names.push(copy_name(&trigger[..trigger_len])?);
    writes += 1;
    sync_namespace_batch(context, view.volume.disk(), source_file, &directory_file)?;
    context.expect_eq(
        checked(context, inode.metadata_snapshot())?.flags & INDEX,
        INDEX,
    );

    // Step 4: 验证升级前打开并复制的 `directory_fd` 与 `duplicate_fd` 仍能继续共享推进 `readdir` 游标。
    let first = checked(context, directory_table.readdir(directory_fd))?.ok_or(TestError(
        "old-open directory lost its first entry after promotion",
    ))?;
    context.expect_eq(first.name.as_slice(), b".".as_slice());
    let continued = checked(context, directory_table.readdir(duplicate_fd))?.ok_or(TestError(
        "duplicated old-open cursor stopped after promotion",
    ))?;
    context.expect_eq(continued.name.as_slice() != b".".as_slice(), true);
    let shape = checked(context, htree_shape(&view.volume, &inode, true))?;
    context.expect_eq(shape.root[28], 3);
    if hash_version != 3 {
        set_root_hash_version(context, &view.volume, &inode, hash_version, &names)?;
        let updated = checked(context, htree_shape(&view.volume, &inode, true))?;
        context.expect_eq(updated.root[28], hash_version);
    }

    // Step 5: 校验哈希碰撞样本在目标 `hash_version` 下具有相同的 `(major, minor=0)` 哈希且不与已有名字冲突，随后插入目录。
    let first_hash = checked(
        context,
        codec::directory_hash(geometry, hash_version, collision[0]),
    )?;
    let second_hash = checked(
        context,
        codec::directory_hash(geometry, hash_version, collision[1]),
    )?;
    context.expect_eq(first_hash, second_hash);
    context.expect_eq(first_hash.1, 0);
    for name in &names {
        let existing_hash = checked(context, codec::directory_hash(geometry, hash_version, name))?;
        if existing_hash == first_hash {
            return Err(TestError("fixed collision vector overlaps promotion names"));
        }
    }

    for name in collision {
        checked(
            context,
            view.vfs
                .link_from(&view.source, &view.context, &directory, name, (0, 0)),
        )?;
        names.push(copy_name(name)?);
        writes += 1;
        if writes % DIRECTORY_WRITE_BATCH == 0 {
            sync_namespace_batch(context, view.volume.disk(), source_file, &directory_file)?;
        }
    }

    context.expect_eq(
        checked(context, inode.metadata_snapshot())?.flags & INDEX,
        INDEX,
    );
    sync_namespace_batch(context, view.volume.disk(), source_file, &directory_file)?;
    let final_shape = checked(context, htree_shape(&view.volume, &inode, true))?;
    context.expect_eq(final_shape.root[28], hash_version);
    Ok(LegacyDirectory {
        pathname,
        path: directory,
        inode,
        names,
        collision,
    })
}

/// 在保持已有目录项哈希值不变的前提下，更新 HTree 根块（`dx_root`）记录的哈希版本字段并重算校验和。
///
/// # 调用上下文
/// 在可睡眠测试线程中调用。要求调用时仅包含纯 ASCII 目录项（使得有符号版本 `0` 与无符号版本 `3` 对已有名称计算出的哈希完全一致），且尚未插入含高比特位字节（如 `0xff`）的碰撞样本。
///
/// # 执行步骤
/// 1. 读取逻辑块 0 并校验其为合法的 `DirectoryLayout::Root` 布局。
/// 2. 遍历 `names` 确认在旧哈希版本与新哈希版本 `version` 下，所有已存在目录项的哈希值完全一致，不会改变 HTree 路由顺序。
/// 3. 修改 `bytes[28]`（`dx_root_info.hash_version`），若启用了 `METADATA_CSUM` 则重算 `dx_root` 尾部校验和，并通过 `buffer.update` 写回元数据缓存块。
fn set_root_hash_version(
    context: &TestContext<'_>,
    volume: &Ext4Volume,
    inode: &InodeRef,
    version: u8,
    names: &[Vec<u8>],
) -> TestResult {
    // Step 1: 读取逻辑块 0 并确认切换哈希版本不会改变已有目录项的哈希值与路由结果。
    let (metadata, map_root) = checked(
        context,
        inode.with_map_read(|metadata, root| Ok((*metadata, *root))),
    )?;
    let geometry = volume.geometry();
    let mut bytes = checked(context, directory_block(volume, &metadata, &map_root, 0))?;
    if codec::verify_directory(geometry, &metadata, 0, &bytes[..geometry.block_size], false)
        .map_err(|error| {
            context.diagnostic(format_args!("W27 legacy root validation error={:?}", error));
            TestError("legacy directory root validation failed")
        })?
        != codec::DirectoryLayout::Root
    {
        return Err(TestError("legacy directory root layout changed"));
    }
    let old_version = bytes[28];
    for name in names {
        let old_hash = checked(context, codec::directory_hash(geometry, old_version, name))?;
        let new_hash = checked(context, codec::directory_hash(geometry, version, name))?;
        if old_hash != new_hash {
            return Err(TestError(
                "legacy hash version changes existing directory routing",
            ));
        }
    }

    // Step 2: 写入新哈希版本字段，重算 `dx_root` 校验和并更新到底层元数据块缓存。
    bytes[28] = version;
    if geometry.features_ro_compat & codec::METADATA_CSUM != 0 {
        let count =
            usize::from(codec::le16(&bytes, 34).map_err(|_| TestError("dx count decode failed"))?);
        let limit =
            usize::from(codec::le16(&bytes, 32).map_err(|_| TestError("dx limit decode failed"))?);
        codec::update_dx_checksum(
            &metadata,
            &mut bytes[..geometry.block_size],
            32 + count * 8,
            32 + limit * 8,
        )
        .map_err(|_| TestError("dx root checksum preparation failed"))?;
    }
    if codec::verify_directory(geometry, &metadata, 0, &bytes[..geometry.block_size], false)
        .map_err(|_| TestError("dx root checksum verification failed"))?
        != codec::DirectoryLayout::Root
    {
        return Err(TestError("legacy hash version changed the root layout"));
    }

    let physical = match volume.map_block(&metadata, &map_root, 0, None) {
        Ok(BlockMapping::Mapped { physical, .. }) => physical,
        Ok(_) => return Err(TestError("legacy root block is not mapped")),
        Err(error) => {
            context.diagnostic(format_args!("W27 legacy root mapping error={:?}", error));
            return Err(TestError("legacy root mapping failed"));
        }
    };
    let buffer = checked(context, volume.metadata_block(physical))?;
    checked(
        context,
        buffer
            .update(volume.claim(), |target| {
                target.copy_from_slice(&bytes[..geometry.block_size])
            })
            .map_err(FsError::from),
    )?;
    Ok(())
}

/// 遍历升级后的目录，验证 `for_each_dirent` 中途拒绝交付（返回 `Ok(false)`）后的单步 `readdir` 恢复、完整目录项集合以及到达 EOF 后的游标状态。
///
/// # 验证步骤
/// 1. 调用 `for_each_dirent` 仅接受前 2 个目录项，在第 3 个目录项返回 `Ok(false)` 中断遍历，断言文件当前偏移停留在第 2 项的 `next_cookie`。
/// 2. 调用单步 `file.readdir()`，断言返回的恰好是刚才第 3 个被拒绝的目录项，且其名称、Inode 号与 `next_cookie` 均未改变。
/// 3. 再次调用 `for_each_dirent` 读完剩余所有目录项，验证 `"."`、`".."` 以及 `names` 中的全部名称均被唯一枚举且指向 `inode_number`。
/// 4. 验证在 EOF 处继续枚举返回 0 项，且 `SeekFrom::Current(0)` 与 `SeekFrom::End(0)` 相等。
fn complete_directory_names(
    context: &mut TestContext<'_>,
    vfs: &Vfs,
    path: &Path,
    names: &[Vec<u8>],
    inode_number: u64,
) -> TestResult {
    // Step 1: 打开目录并在前 2 项后通过返回 `Ok(false)` 触发短回调中断，验证游标未越过第 3 项。
    let file = checked(context, open_path(vfs, path.clone()))?;
    let mut seen = Vec::new();
    seen.try_reserve_exact(names.len())
        .map_err(|_| TestError("directory name bitmap OOM"))?;
    seen.resize(names.len(), false);
    let mut dots = 0usize;
    let mut entries = 0usize;
    let mut accepted = 0usize;
    let mut accepted_cookie = checked(context, file.seek(SeekFrom::Current(0)))?;
    let mut declined_cookie = accepted_cookie;
    let mut declined_number = 0;
    let mut declined_name = [0u8; 255];
    let mut declined_len = 0usize;
    file.for_each_dirent(&mut |entry| {
        accepted += 1;
        if accepted <= 2 {
            observe_directory_name(
                entry,
                names,
                inode_number,
                &mut seen,
                &mut dots,
                &mut entries,
            )?;
            accepted_cookie = entry.next_cookie;
            return Ok(true);
        }
        if 255 < entry.name.len() {
            return Err(FsError::Corrupt);
        }
        declined_len = entry.name.len();
        declined_name[..declined_len].copy_from_slice(&entry.name);
        declined_number = entry.number;
        declined_cookie = entry.next_cookie;
        Ok(false)
    })
    .map_err(|error| {
        context.diagnostic(format_args!(
            "W27 short directory delivery error={:?}",
            error
        ));
        TestError("directory short delivery failed")
    })?;
    if accepted < 3 {
        return Err(TestError("legacy directory lacks short-delivery entries"));
    }
    context.expect_eq(
        checked(context, file.seek(SeekFrom::Current(0)))?,
        accepted_cookie,
    );

    // Step 2: 单步 `readdir` 重读被拒的第 3 项，随后继续遍历剩余全部目录项并核对 EOF 游标状态。
    let declined =
        checked(context, file.readdir())?.ok_or(TestError("short directory entry was lost"))?;
    context.expect_eq(declined.next_cookie, declined_cookie);
    context.expect_eq(declined.number, declined_number);
    context.expect_eq(declined.name.as_slice(), &declined_name[..declined_len]);
    observe_directory_name(
        &declined,
        names,
        inode_number,
        &mut seen,
        &mut dots,
        &mut entries,
    )
    .map_err(|_| TestError("short directory entry changed before retry"))?;

    file.for_each_dirent(&mut |entry| {
        observe_directory_name(
            entry,
            names,
            inode_number,
            &mut seen,
            &mut dots,
            &mut entries,
        )?;
        Ok(true)
    })
    .map_err(|error| {
        context.diagnostic(format_args!(
            "W27 full directory enumeration error={:?}",
            error
        ));
        TestError("directory enumeration failed")
    })?;
    context.expect_eq(dots, 2);
    context.expect_eq(entries, names.len() + 2);
    context.expect_eq(seen.iter().all(|present| *present), true);

    let mut after_eof = 0;
    file.for_each_dirent(&mut |_| {
        after_eof += 1;
        Ok(true)
    })
    .map_err(|_| TestError("directory EOF enumeration failed"))?;
    context.expect_eq(after_eof, 0);
    context.expect_eq(
        checked(context, file.seek(SeekFrom::Current(0)))?,
        checked(context, file.seek(SeekFrom::End(0)))?,
    );
    Ok(())
}

/// 校验单个枚举出的目录项是否为 `"."`/`".."` 或 `names` 中的预期项，并标记已访问位图。
///
/// 若遇到未知名称、重复名称、Inode 编号不匹配或文件类型不是普通文件，则返回 [`FsError::Corrupt`]。
fn observe_directory_name(
    entry: &DirectoryEntry,
    names: &[Vec<u8>],
    inode_number: u64,
    seen: &mut [bool],
    dots: &mut usize,
    entries: &mut usize,
) -> Result<(), FsError> {
    *entries += 1;
    if entry.name == b"." || entry.name == b".." {
        *dots += 1;
        return Ok(());
    }
    let Some(index) = names.iter().position(|name| name.as_slice() == entry.name) else {
        return Err(FsError::Corrupt);
    };
    if seen[index] || entry.number != inode_number || entry.kind != Some(InodeKind::Regular) {
        return Err(FsError::Corrupt);
    }
    seen[index] = true;
    Ok(())
}

/// 验证 HTree 索引目录下哈希碰撞目录项的独立打开遍历、`dup` 共享游标续读以及显式 `seek(cookie)` 重定位语义。
///
/// # 验证步骤
/// 1. 确认 `legacy.collision` 中的两个文件名在当前目录哈希版本下具有相同的 `(major, minor=0)` 哈希值，且都能通过路径解析定位到源 Inode。
/// 2. 分别打开两个独立文件描述符 `fd` 与 `independent`，并通过 `dup(fd)` 创建共享同一打开文件描述（OFD）的 `duplicate`。
/// 3. 使用 `fd` 持续 `readdir` 直到读出碰撞对中的第一个目录项 `first`，记录其 `first_cookie = first.next_cookie`。
/// 4. 立即从共享游标的 `duplicate` 调用 `readdir`，验证能够紧接着读出同一哈希桶内的第二个碰撞目录项 `second`。
/// 5. 在独立描述符 `independent` 上调用 `seek(SeekFrom::Start(first_cookie))`，验证重定位后依次调用 `readdir` 能精确复现 `first` 和 `second` 的读取顺序，且穿插 `SeekFrom::Current(0)` 不会丢失同哈希桶内的次序状态。
fn check_collision_cursors(
    context: &mut TestContext<'_>,
    view: &NamespaceView,
    legacy: &LegacyDirectory,
) -> TestResult {
    // Step 1: 验证碰撞文件名对在当前目录哈希版本下具有相同的 `(major, minor=0)` 哈希且均可解析到源 Inode。
    let source_inode = checked(context, view.source.inode())?;
    let shape = checked(context, htree_shape(&view.volume, &legacy.inode, true))?;
    let version = shape.root[28];
    let first_hash = checked(
        context,
        codec::directory_hash(view.volume.geometry(), version, legacy.collision[0]),
    )?;
    let second_hash = checked(
        context,
        codec::directory_hash(view.volume.geometry(), version, legacy.collision[1]),
    )?;
    context.expect_eq((first_hash.0, first_hash.1), (second_hash.0, second_hash.1));
    context.expect_eq(first_hash.1, 0);
    for name in legacy.collision {
        let path = checked(
            context,
            view.vfs.resolve_from(
                &view.context,
                &legacy.path,
                name,
                ResolveMode::Follow,
                Some((0, 0)),
            ),
        )?;
        context.expect_eq(checked(context, path.inode())?.ptr_eq(source_inode), true);
    }

    // Step 2: 打开独立文件描述符 `fd`、`independent` 以及共享游标的 `duplicate`，定位到第一个碰撞项并验证 `duplicate` 续读第二个碰撞项。
    let table = checked(context, FileTable::try_new())?;
    let options = OpenOptions {
        directory: true,
        ..OpenOptions::default()
    };
    let fd = checked(
        context,
        table.open(&view.vfs, &view.context, legacy.pathname, options, 1024),
    )?;
    let independent = checked(
        context,
        table.open(&view.vfs, &view.context, legacy.pathname, options, 1024),
    )?;
    let duplicate = checked(context, table.dup(fd, 1024))?;
    let independent_dot = checked(context, table.readdir(independent))?
        .ok_or(TestError("independent indexed directory is empty"))?;
    context.expect_eq(independent_dot.name.as_slice(), b".".as_slice());
    let fd_dot = checked(context, table.readdir(fd))?
        .ok_or(TestError("original indexed directory is empty"))?;
    context.expect_eq(fd_dot.name.as_slice(), b".".as_slice());

    let mut first = None;
    while let Some(entry) = checked(context, table.readdir(fd))? {
        if legacy
            .collision
            .iter()
            .any(|name| *name == entry.name.as_slice())
        {
            first = Some(entry);
            break;
        }
    }
    let first = first.ok_or(TestError("real same-hash collision was not enumerated"))?;
    let first_name = first.name.clone();
    let first_cookie = first.next_cookie;
    let second = checked(context, table.readdir(duplicate))?
        .ok_or(TestError("dup lost the same-hash continuation"))?;
    context.expect_eq(
        second.name.as_slice() != first_name.as_slice()
            && legacy
                .collision
                .iter()
                .any(|name| *name == second.name.as_slice()),
        true,
    );

    // Step 3: 在独立描述符上使用 `SeekFrom::Start(first_cookie)` 重定位，验证碰撞桶内的读取顺序可精确复现。
    checked(
        context,
        table.seek(independent, SeekFrom::Start(first_cookie)),
    )?;
    let reset = checked(context, table.readdir(independent))?
        .ok_or(TestError("explicit seek lost the first collision"))?;
    context.expect_eq(reset.name.as_slice(), first_name.as_slice());
    context.expect_eq(
        checked(context, table.seek(independent, SeekFrom::Current(0)))?,
        first_cookie,
    );
    let continued = checked(context, table.readdir(independent))?
        .ok_or(TestError("SEEK_CUR(0) discarded collision continuation"))?;
    context.expect_eq(continued.name.as_slice(), second.name.as_slice());

    for descriptor in [fd, independent, duplicate] {
        checked(context, table.close(descriptor))?;
    }
    Ok(())
}

/// 断言 HTree 目录在经历扩展分裂、重命名与删除操作后，依然保持 `INDEX` 标志、合法的 `dx_root` 布局以及单调不减的树深度与节点数。
///
/// # 验证条件
/// - 目录 Inode 元数据的 [`INDEX`] 标志位未丢失。
/// - 扩展后的树深度 `after.depth >= before.depth`。
/// - 根块索引项数 `after.root_count > before.root_count`，中间索引节点数 `after.nodes.len() > before.nodes.len()`，叶子块总数 `after.leaves > before.leaves`。
/// - 逻辑块 0 仍能通过 `codec::verify_directory` 的 `DirectoryLayout::Root` 校验。
fn preserve_indexed_layout(
    context: &mut TestContext<'_>,
    volume: &Ext4Volume,
    inode: &InodeRef,
    before: &HTreeShape,
    after: &HTreeShape,
) -> TestResult {
    let metadata = checked(context, inode.metadata_snapshot())?;
    context.expect_eq(metadata.flags & INDEX, INDEX);
    context.expect_eq(before.depth <= after.depth, true);
    context.expect_eq(before.root_count < after.root_count, true);
    context.expect_eq(before.nodes.len() < after.nodes.len(), true);
    context.expect_eq(before.leaves < after.leaves, true);
    context.expect_eq(
        matches!(
            codec::verify_directory(
                volume.geometry(),
                &after.metadata,
                0,
                &after.root[..volume.geometry().block_size],
                false,
            ),
            Ok(codec::DirectoryLayout::Root)
        ),
        true,
    );
    Ok(())
}

/// 将当前文件系统状态全量落盘并彻底卸载旧缓存与卷实例，随后在同一块设备上冷启动挂载新缓存与新卷，验证持久化数据一致性。
///
/// # 执行步骤
/// 1. **全量写回**：同步测试夹具记录的所有 Inode 与 `old_volume` 元数据块，并排空块设备完成队列。
/// 2. **卸载与等待退役**：清理测试夹具持有的路径与 VFS 视图，移除 `InodeCache` 加载钩子，获取 `old_cache`、`old_volume` 与元数据缓存的 `Weak` 指针，通过 `CacheWatch::wait_retired` 等待后台回写任务彻底执行完毕并退出。
/// 3. **校验引用归零**：释放 `old_volume`，断言上述三个 `Weak` 引用均无法升级（`upgrade().is_none()`），确保内存中不残留任何旧缓存数据。
/// 4. **冷盘重建与核对**：基于同一底层 `disk` 重新打开 `Ext4Volume`、新建 `InodeCache` 与 `Vfs`，解析 `sample_path` 并校验其 Inode 编号、64 字节文件内容，同时校验 `/indexed` 目录的 `INDEX` 标志与两级 HTree 块 CRC 校验和。
fn cold_reopen_namespace(
    context: &mut TestContext<'_>,
    fixture: &mut WriteOwner,
    sample_path: &[u8],
    expected_inode: u64,
    expected_bytes: &[u8; 64],
) -> TestResult {
    // Step 1: 将所有打开的 Inode 与卷元数据同步刷盘，清理测试夹具持有的视图引用。
    let disk = fixture.user.path.owner.disk()?.clone();
    let old_volume = fixture
        .cold_volume
        .as_ref()
        .ok_or(TestError("namespace original Volume owner missing"))?
        .clone();
    for inode in &fixture.inodes {
        checked(context, crate::fs::writeback::sync_inode(inode))?;
    }
    checked(context, crate::fs::writeback::sync_metadata(&old_volume))?;
    disk.test_flush_completions();
    fixture.inodes.clear();
    fixture.user.path.release_view();

    // Step 2: 拆下旧 `InodeCache` 并等待后台回写任务退役，确认缓存与卷的 `Weak` 引用全部失效。
    let old_cache = fixture
        .user
        .path
        .owner
        .cache
        .take()
        .ok_or(TestError("namespace original InodeCache owner missing"))?;
    old_cache.ktest_set_load_hook(None);
    let weak_cache = Arc::downgrade(&old_cache);
    let weak_volume = Arc::downgrade(&old_volume);
    let weak_metadata = Arc::downgrade(old_volume.metadata_cache());
    assert!(fixture.user.path.owner.cache_watch.is_none());
    fixture.user.path.owner.cache_watch = Some(old_cache.watch());
    drop(old_cache);
    checked(
        context,
        fixture
            .user
            .path
            .owner
            .cache_watch
            .as_ref()
            .expect("retired namespace cache has no watch")
            .wait_retired(),
    )?;
    drop(fixture.user.path.owner.cache_watch.take());
    checked(context, crate::fs::writeback::sync_metadata(&old_volume))?;
    disk.test_flush_completions();
    context.expect_eq(disk.test_snapshot().inflight, 0);
    drop(fixture.cold_volume.take());
    drop(old_volume);
    context.expect_eq(weak_cache.upgrade().is_none(), true);
    context.expect_eq(weak_volume.upgrade().is_none(), true);
    context.expect_eq(weak_metadata.upgrade().is_none(), true);

    // Step 3: 在同一块设备上重新打开 `Ext4Volume`、创建新 `InodeCache` 与 `Vfs`，核对文件内容与 HTree 目录 CRC。
    let volume = checked(
        context,
        Ext4Volume::open(BlockView::whole(disk), AccessMode::ReadWrite, 64),
    )?;
    let cache = Arc::try_new(checked(context, InodeCache::try_new(64))?)
        .map_err(|_| TestError("cold namespace cache owner OOM"))?;
    fixture.user.path.owner.cache = Some(cache.clone());
    fixture.cold_volume = Some(volume.clone());
    let vfs = Arc::try_new(checked(
        context,
        Vfs::try_new(
            crate::fs::Filesystem::Ext4(volume.clone()),
            cache,
            128,
            4,
            AccessMode::ReadWrite,
        ),
    )?)
    .map_err(|_| TestError("cold namespace Vfs owner OOM"))?;
    let fs_context = checked(context, FsContext::new(vfs.root(), vfs.root()))?;
    let path = checked(
        context,
        vfs.resolve(&fs_context, sample_path, ResolveMode::Follow),
    )?;
    let inode = checked(context, path.inode())?;
    context.expect_eq(inode.number(), expected_inode);
    context.expect_eq(
        checked(context, inode.metadata_snapshot())?.kind,
        InodeKind::Regular,
    );
    let mut bytes = [0xa7; 64];
    context.expect_eq(checked(context, inode.read_at(0, &mut bytes))?, bytes.len());
    context.expect_eq(&bytes, expected_bytes);

    let index_path = checked(
        context,
        vfs.resolve(&fs_context, b"/indexed", ResolveMode::Follow),
    )?;
    let index_inode = checked(context, index_path.inode())?;
    context.expect_eq(
        checked(context, index_inode.metadata_snapshot())?.flags & INDEX,
        INDEX,
    );
    let index_shape = checked(context, htree_shape(&volume, index_inode, true))?;
    context.expect_eq(index_shape.depth, 1);
    context.diagnostic(format_args!(
        "W27 cold-read old-volume-weak=0 old-metadata-weak=0 old-cache-weak=0 same-disk=1 new-canonical-cache=1 inode-number={} full-bytes=64 indexed-CRC-verified=1",
        expected_inode
    ));
    Ok(())
}

/// 目录扩展、哈希碰撞游标与共享挂载边界测试套件。
#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-path", "fs-namespace"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 验证既有两级 HTree 索引目录在持续插入长文件名时触发叶子块与中间索引块分裂，并验证重命名、删除、传统哈希碰撞游标及冷盘重开一致性。
    ///
    /// # 测试目标
    /// - 验证 `depth = 1` 的 HTree 目录（`/indexed`）在定向填满某个 `dx_node` 中间索引节点时，能够依次触发叶子块分裂（`leaf_splits`）和中间索引节点分裂（`dx_root` 槽位数增加 1）。
    /// - 验证包含非 UTF-8 字节（`0xff`）的 255 字节长文件名在分裂、同目录 `rename`、`unlink`、`readdir` 短回调中断重读以及冷盘重新挂载后仍保持完整一致。
    /// - 验证有符号（version 0）与无符号（version 3）传统哈希（legacy hash）下的目录升级与同哈希碰撞游标定位。
    ///
    /// # 场景步骤（Given-When-Then）
    /// 1. **Given（准备环境）**：在挂载前将超级块 `default_hash_version` 设为 `3`（unsigned legacy），挂载文件系统并读取 `/pattern` 源文件内容与初始硬链接数，读取 `/indexed` 目录的初始两级 HTree 拓扑 `initial`。
    /// 2. **When（定向分裂 HTree 叶子与中间节点）**：选出当前索引项最多的 `dx_node` 作为 `target`，循环调用 [`name_for_split_leaf`] 生成落入该节点最宽哈希区间的 255 字节文件名并创建硬链接，直到触发 `split_limit` 次叶子分裂并最终使 `dx_root` 的索引项计数 `root_count` 增加 1（即中间索引节点分裂）。
    /// 3. **When（目录项重命名与删除）**：在分裂后的目录中将 `names[0]` 重命名为 `rename_target`，并将 `names[1]`（`deleted`）解除硬链接，同步落盘。
    /// 4. **Then（验证 HTree 拓扑与目录项枚举）**：调用 [`preserve_indexed_layout`] 确认树结构与 CRC 完整，验证源 Inode 硬链接计数精确等于 `initial_links + operations - 1`，被删名称返回 `FsError::NotFound`，其余名称均可解析且能通过 [`verify_indexed_new_names`] 完整枚举。
    /// 5. **When / Then（验证有符号与无符号传统哈希目录升级及碰撞游标）**：分别对 `/legacy-signed`（version 0）与 `/legacy-unsigned`（version 3）执行 [`promote_legacy_directory`]、[`complete_directory_names`] 与 [`check_collision_cursors`]。
    /// 6. **Then（验证冷盘重开持久化）**：全量刷盘并释放所有内存对象，调用 [`cold_reopen_namespace`] 在同一磁盘上冷启动新缓存，验证重命名后的长文件名路径仍能解析到同一 Inode 且 64 字节数据与 HTree 索引 CRC 完好无损。
    #[test]
    #[ktest(tags("W27"))]
    fn indexed_extension_splits_leaf_and_index_node(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 将底层磁盘超级块默认哈希版本设置为 3（无符号传统哈希），初始化命名空间视图并读取源 Inode 基准信息。
        let disk = fixture.user.path.owner.disk()?.clone();
        select_unsigned_legacy_default(context, &disk)?;
        let view = prepare_namespace_view(context, fixture)?;
        context.expect_eq(view.volume.geometry().default_hash_version, 3);
        context.expect_eq(view.volume.geometry().hash_unsigned, false);
        context.expect_eq(
            view.volume
                .geometry()
                .hash_seed
                .iter()
                .any(|seed| *seed != 0),
            true,
        );

        let source_inode = checked(context, view.source.inode())?.clone();
        let source_number = source_inode.number();
        let initial_links = checked(context, source_inode.metadata_snapshot())?.links;
        let mut source_bytes_before = [0xa7; 64];
        context.expect_eq(
            checked(context, source_inode.read_at(0, &mut source_bytes_before))?,
            source_bytes_before.len(),
        );

        // Step 2: 读取 `/indexed` 目录的初始两级 HTree 结构，计算撑满并分裂目标中间索引节点所需的叶子分裂次数 `split_limit` 与操作预算。
        let indexed = checked(
            context,
            view.vfs
                .resolve(&view.context, b"/indexed", ResolveMode::Follow),
        )?;
        let indexed_inode = checked(context, indexed.inode())?.clone();
        let initial = checked(context, htree_shape(&view.volume, &indexed_inode, true))?;
        context.expect_eq(initial.depth, 1);
        context.expect_eq(
            checked(context, indexed_inode.metadata_snapshot())?.flags & INDEX,
            INDEX,
        );
        let target = initial
            .nodes
            .iter()
            .max_by_key(|node| node.count)
            .copied()
            .ok_or(TestError("existing HTree has no index node"))?;
        if target.limit < target.count || initial.root_limit <= initial.root_count {
            return Err(TestError("existing HTree index capacity is inconsistent"));
        }
        let split_limit = target.limit - target.count + 1;
        // 255 字节长文件名在目录叶子块中按 4 字节对齐占 264 字节；每个固定叶子哈希区间最多写入一个叶子容量就会触发叶子块分裂。
        let leaf_capacity = view.volume.geometry().block_size / ((8 + 255 + 3) & !3);
        let operation_limit = (split_limit * (leaf_capacity + 1)).max(3);
        let link_total = initial_links
            .checked_add(operation_limit as u32)
            .ok_or(TestError("hardlink count overflow"))?;
        if (u16::MAX as u32) < link_total {
            return Err(TestError(
                "bounded hardlink batch exceeds ext4 link-count capacity",
            ));
        }

        let mut names = Vec::new();
        names
            .try_reserve_exact(operation_limit)
            .map_err(|_| TestError("W27 hardlink name storage OOM"))?;
        let mut next_nonce = 0u32;
        let root_file = fixture.open(context, b"/", 0)?;
        let source_file = fixture.open(context, b"/pattern", 0)?;
        let directory_file = fixture.open(context, b"/indexed", 0)?;
        let mut current = initial.clone();
        let mut index_split = false;
        let mut leaf_splits = 0usize;
        let mut operations = 0usize;
        // Step 3: 循环向目标中间索引节点最宽的叶子区间插入硬链接，直至触发 `split_limit` 次叶子块分裂并引起 `dx_node` 中间索引块分裂。
        for _ in 0..operation_limit {
            let node = current
                .nodes
                .iter()
                .find(|node| node.logical == target.logical)
                .copied()
                .ok_or(TestError("target index node disappeared before split"))?;
            let name = name_for_split_leaf(&view.volume, &current, node, &mut next_nonce).map_err(
                |error| {
                    context.diagnostic(format_args!("W27 leaf-range candidate error={:?}", error));
                    TestError("bounded leaf-range hash candidate search failed")
                },
            )?;
            let old_leaves = current.leaves;
            checked(
                context,
                view.vfs
                    .link_from(&view.source, &view.context, &indexed, &name.bytes, (0, 0)),
            )?;
            names.push(name);
            operations += 1;
            let next = checked(context, htree_shape(&view.volume, &indexed_inode, false))?;
            if next.leaves != old_leaves {
                context.expect_eq(next.leaves, old_leaves + 1);
                leaf_splits += 1;
            }
            if current.root_count < next.root_count {
                context.expect_eq(next.root_count, current.root_count + 1);
                context.expect_eq(next.nodes.len(), current.nodes.len() + 1);
                context.expect_eq(next.depth, current.depth);
                index_split = true;
                current = next;
                break;
            }
            context.expect_eq(next.root_count, current.root_count);
            context.expect_eq(next.nodes.len(), current.nodes.len());
            context.expect_eq(next.depth, current.depth);
            let updated_node = next
                .nodes
                .iter()
                .find(|entry| entry.logical == node.logical)
                .ok_or(TestError("split leaf lost its parent index node"))?;
            context.expect_eq(
                updated_node.count,
                node.count + usize::from(old_leaves < next.leaves),
            );
            current = next;
            if operations % DIRECTORY_WRITE_BATCH == 0 {
                sync_namespace_batch(context, &disk, &source_file, &directory_file)?;
            }
        }
        if !index_split {
            return Err(TestError(
                "bounded HTree writes did not split a full index node",
            ));
        }
        context.expect_eq(leaf_splits, split_limit);

        while names.len() < 3 {
            let name = name_for_root_range(
                &view.volume,
                &initial,
                target.root_position,
                &mut next_nonce,
            )
            .map_err(|error| {
                context.diagnostic(format_args!("W27 root-range candidate error={:?}", error));
                TestError("bounded root-range hash candidate search failed")
            })?;
            checked(
                context,
                view.vfs
                    .link_from(&view.source, &view.context, &indexed, &name.bytes, (0, 0)),
            )?;
            names.push(name);
            operations += 1;
            if operations % DIRECTORY_WRITE_BATCH == 0 {
                sync_namespace_batch(context, &disk, &source_file, &directory_file)?;
            }
        }

        // Step 4: 在分裂后的 HTree 目录上执行硬链接重命名（rename）与删除（unlink），验证树布局、硬链接计数与 `readdir` 枚举结果。
        let rename_target = name_for_root_range(
            &view.volume,
            &initial,
            target.root_position,
            &mut next_nonce,
        )
        .map_err(|error| {
            context.diagnostic(format_args!("W27 rename candidate error={:?}", error));
            TestError("bounded rename hash candidate search failed")
        })?;
        let deleted = names[1];
        checked(
            context,
            view.vfs.rename_from(
                &view.context,
                &indexed,
                &names[0].bytes,
                &indexed,
                &rename_target.bytes,
                false,
                (0, 0),
            ),
        )?;
        checked(
            context,
            view.vfs
                .unlink_from(&view.context, &indexed, &deleted.bytes, (0, 0)),
        )?;
        names[0] = rename_target;
        names.remove(1);
        names.sort_unstable_by_key(|name| name.nonce);
        sync_namespace_batch(context, &disk, &source_file, &directory_file)?;
        checked(context, root_file.sync_range(0..u64::MAX, false))?;

        let after = checked(context, htree_shape(&view.volume, &indexed_inode, true))?;
        preserve_indexed_layout(context, &view.volume, &indexed_inode, &initial, &after)?;
        let final_links = checked(context, source_inode.metadata_snapshot())?.links;
        context.expect_eq(final_links, initial_links + operations as u32 - 1);
        for name in &names {
            let path = checked(
                context,
                view.vfs.resolve_from(
                    &view.context,
                    &indexed,
                    &name.bytes,
                    ResolveMode::Follow,
                    Some((0, 0)),
                ),
            )?;
            context.expect_eq(checked(context, path.inode())?.ptr_eq(&source_inode), true);
        }
        context.expect_eq(
            view.vfs
                .resolve_from(
                    &view.context,
                    &indexed,
                    &deleted.bytes,
                    ResolveMode::Follow,
                    Some((0, 0)),
                )
                .err(),
            Some(FsError::NotFound),
        );
        verify_indexed_new_names(context, &view.vfs, &indexed, &names, source_number)?;

        let mut cold_path = Vec::new();
        cold_path
            .try_reserve_exact(9 + rename_target.bytes.len())
            .map_err(|_| TestError("cold namespace path OOM"))?;
        cold_path.extend_from_slice(b"/indexed/");
        cold_path.extend_from_slice(&rename_target.bytes);

        // Step 5: 验证有符号（version 0）与无符号（version 3）传统哈希下的普通目录升级、包含 `0xff` 字节的哈希碰撞以及 `readdir`/`seek` 游标语义。
        let collision_signed = promote_legacy_directory(
            context,
            fixture,
            &view,
            &source_file,
            b"/legacy-signed",
            PROMOTION_PREFIX_SIGNED,
            0,
            [b"w27-00000788-\xff", b"w27-0000078f-\xff"],
        )?;
        let collision_unsigned = promote_legacy_directory(
            context,
            fixture,
            &view,
            &source_file,
            b"/legacy-unsigned",
            PROMOTION_PREFIX_UNSIGNED,
            3,
            [b"w27-000050b2-\xff", b"w27-000050bb-\xff"],
        )?;
        let signed_version = checked(
            context,
            htree_shape(&view.volume, &collision_signed.inode, true),
        )?;
        let unsigned_version = checked(
            context,
            htree_shape(&view.volume, &collision_unsigned.inode, true),
        )?;
        context.expect_eq(signed_version.root[28], 0);
        context.expect_eq(unsigned_version.root[28], 3);
        complete_directory_names(
            context,
            &view.vfs,
            &collision_signed.path,
            &collision_signed.names,
            source_number,
        )?;
        complete_directory_names(
            context,
            &view.vfs,
            &collision_unsigned.path,
            &collision_unsigned.names,
            source_number,
        )?;
        check_collision_cursors(context, &view, &collision_signed)?;
        check_collision_cursors(context, &view, &collision_unsigned)?;

        // Step 6: 同步所有脏数据与元数据，彻底释放当前 VFS 与缓存引用后重新冷挂载磁盘，验证数据与 HTree 索引的持久化完整性。
        sync_namespace_batch(context, &disk, &source_file, &directory_file)?;
        checked(context, root_file.sync_range(0..u64::MAX, false))?;
        for inode in &fixture.inodes {
            checked(context, crate::fs::writeback::sync_inode(inode))?;
        }
        checked(context, crate::fs::writeback::sync_metadata(&view.volume))?;
        disk.test_flush_completions();

        let depth_before = initial.depth;
        let roots_before = initial.root_count;
        let leaves_before = initial.leaves;
        let depth_after = after.depth;
        let roots_after = after.root_count;
        let leaves_after = after.leaves;
        drop((collision_signed, collision_unsigned));
        drop((directory_file, source_file, root_file));
        drop((
            source_inode,
            indexed_inode,
            indexed,
            view,
            initial,
            current,
            after,
        ));
        fixture.inodes.clear();

        cold_reopen_namespace(
            context,
            fixture,
            &cold_path,
            source_number,
            &source_bytes_before,
        )?;
        context.diagnostic(format_args!(
            "W27 existing-indexed depth-before={} root-nodes-before={} leaf-blocks-before={} depth-after={} root-nodes-after={} leaf-blocks-after={} index-node-split=1 leaf-split=1 INDEX-preserved=1 links-added={} renamed=1 unlinked=1 signed-and-unsigned-legacy-cookies=1 complete-new-directory-names=1 cold-same-disk-read=1",
            depth_before, roots_before, leaves_before, depth_after, roots_after, leaves_after, operations
        ));
        Ok(())
    }

    /// 验证同一底层卷在读写与只读 VFS 视图间的命名空间共享、Inode 唯一性、绑定挂载（alias mount）跨挂载点限制以及目录操作边界条件。
    ///
    /// # 测试目标
    /// - 验证共享同一 `Ext4Volume` 与 `InodeCache` 的可读写视图（`view.vfs`）和只读视图（`read_only_vfs`）在创建、重命名、删除并重建同名文件时保持目录项同步与规范 Inode（canonical inode）唯一性，且已打开的旧路径仍保活原 Inode 数据。
    /// - 验证目录自硬链接拒绝（`NotPermitted`）、非空目录删除/覆盖拒绝（`NotEmpty`）、被持为当前工作目录（`cwd`）的空目录删除后禁止在其下新建子项（`NotFound`）。
    /// - 验证将同一卷二次挂载到子目录 `/w26-alias` 后，跨挂载点 `link`/`rename` 返回 `CrossDevice`（`EXDEV`），对挂载点或根目录执行 `rmdir`/`rename` 返回 `Busy`（`EBUSY`），以及非特权用户跨只读父目录重命名返回 `Permission`（`EACCES`）。
    ///
    /// # 场景步骤（Given-When-Then）
    /// 1. **Given（构建共享卷的双 VFS 视图）**：基于可读写测试夹具的同一 `Ext4Volume` 和 `InodeCache` 构建只读 `read_only_vfs`，预先在只读视图解析 `/pattern` 保存旧 `Path` 与旧 `InodeRef`。
    /// 2. **When / Then（验证跨视图目录创建、重命名与同名文件替换）**：在可读写视图创建 `/w26-created` 并重命名为 `/w26-renamed`，验证只读视图同步看到新名称且指向同一 `InodeRef`；随后在可读写视图 `unlink("/pattern")` 并重新创建同名 `/pattern`，验证只读视图再次解析得到新 `InodeRef`，而此前持有的 `old_pattern_inode` 仍能读出原 64 字节内容。
    /// 3. **When / Then（验证目录自链接、非空目录删除与覆盖边界）**：在工作线程中尝试将目录硬链接到自身子项返回 `FsError::NotPermitted`；在含子目录的 `/w26-nonempty` 上执行 `rmdir` 或作为 `rename` 覆盖目标均返回 `FsError::NotEmpty`。
    /// 4. **When / Then（验证当前工作目录被删除后的行为）**：将 `/w26-held-cwd` 设为 `held_context` 的当前工作目录后将其 `rmdir` 删除，验证继续通过已删除的 `held_cwd` 创建子目录返回 `FsError::NotFound`。
    /// 5. **When / Then（验证绑定挂载跨设备边界与挂载点忙碌保护）**：将同一 `Ext4Volume` 挂载到 `/w26-alias`，验证跨根挂载与 `/w26-alias` 执行 `link` 或 `rename` 返回 `FsError::CrossDevice`，对 `/` 或挂载点 `/w26-alias` 执行 `rmdir`/`rename` 返回 `FsError::Busy`。
    /// 6. **When / Then（验证非 root 用户跨只读父目录重命名权限检查）**：卸载 `/w26-alias` 后，以普通用户凭证 `(1001, 1001)` 尝试从权限为 `0o555` 的只读源父目录将子目录移出，验证返回 `FsError::Permission`。
    #[test]
    #[ktest(min_cpus = 2, tags("W26"))]
    fn shared_views_alias_mount_and_directory_boundaries(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let view = prepare_namespace_view(context, fixture)?;
        drop(fixture.open(context, b"/", 0)?);
        drop(fixture.open(context, b"/pattern", 0)?);

        // Step 1: 基于同一 `Ext4Volume` 与 `InodeCache` 创建只读 VFS 视图，记录 `/pattern` 的初始 Inode 与数据。
        let cache = fixture.user.path.owner.cache().clone();
        let read_only_vfs = Arc::try_new(checked(
            context,
            Vfs::try_new(
                crate::fs::Filesystem::Ext4(view.volume.clone()),
                cache,
                128,
                4,
                AccessMode::ReadOnly,
            ),
        )?)
        .map_err(|_| TestError("read-only namespace view OOM"))?;
        let read_only_root = read_only_vfs.root();
        let read_only_context = checked(
            context,
            FsContext::new(read_only_root.clone(), read_only_root.clone()),
        )?;
        let old_pattern = checked(
            context,
            read_only_vfs.resolve(&read_only_context, b"/pattern", ResolveMode::Follow),
        )?;
        let old_pattern_inode = checked(context, old_pattern.inode())?.clone();
        let mut old_bytes_before = [0xa7; 64];
        context.expect_eq(
            checked(context, old_pattern_inode.read_at(0, &mut old_bytes_before))?,
            old_bytes_before.len(),
        );
        context.expect_eq(
            read_only_vfs
                .resolve(&read_only_context, b"/w26-created", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );

        // Step 2: 在可读写视图创建并重命名目录，验证只读视图同步反映正负目录项变化且共享同一规范 Inode。
        checked(
            context,
            view.vfs
                .mkdir_from(&view.context, &view.root, b"/w26-created", 0o777, (0, 0), 0),
        )?;
        drop(fixture.open(context, b"/w26-created", 0)?);
        let old_created = checked(
            context,
            read_only_vfs.resolve(&read_only_context, b"/w26-created", ResolveMode::Follow),
        )?;
        let created_inode = checked(context, old_created.inode())?.clone();
        checked(
            context,
            view.vfs.rename_from(
                &view.context,
                &view.root,
                b"w26-created",
                &view.root,
                b"w26-renamed",
                false,
                (0, 0),
            ),
        )?;
        context.expect_eq(
            read_only_vfs
                .resolve(&read_only_context, b"/w26-created", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );
        let renamed = checked(
            context,
            read_only_vfs.resolve(&read_only_context, b"/w26-renamed", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, renamed.inode())?.ptr_eq(&created_inode),
            true,
        );

        // Step 3: 在可读写视图删除并重建同名 `/pattern`，验证新解析路径指向新 Inode，而旧已解析路径仍保活旧 Inode 内容。
        checked(
            context,
            view.vfs
                .unlink_from(&view.context, &view.root, b"pattern", (0, 0)),
        )?;
        let new_file = checked(
            context,
            view.vfs.open_with(
                &view.context,
                &view.root,
                b"pattern",
                (0, 0),
                Some((0o666, view.context.umask())),
                false,
                false,
                |path, _| open_path(&view.vfs, path),
            ),
        )?;
        let new_pattern = new_file.path().unwrap().clone();
        let new_pattern_inode = checked(context, new_pattern.inode())?.clone();
        fixture.inodes.push(new_pattern_inode.clone());
        let current_pattern = checked(
            context,
            read_only_vfs.resolve(&read_only_context, b"/pattern", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, current_pattern.inode())?.ptr_eq(&new_pattern_inode),
            true,
        );
        context.expect_eq(
            checked(context, old_pattern.inode())?.ptr_eq(&old_pattern_inode),
            true,
        );
        context.expect_eq(
            old_pattern.dentry().name().as_slice(),
            b"pattern".as_slice(),
        );
        let mut old_bytes = [0xa7; 64];
        context.expect_eq(
            checked(context, old_pattern_inode.read_at(0, &mut old_bytes))?,
            old_bytes.len(),
        );
        context.expect_eq(old_bytes, old_bytes_before);

        // Step 4: 验证目录自硬链接返回 `NotPermitted`，以及非空目录直接 `rmdir` 或作为 `rename` 覆盖目标返回 `NotEmpty`。
        let self_directory = b"/w26-nonempty";
        checked(
            context,
            view.vfs
                .mkdir_from(&view.context, &view.root, self_directory, 0o777, (0, 0), 0),
        )?;
        drop(fixture.open(context, self_directory, 0)?);
        let self_path = checked(
            context,
            view.vfs
                .resolve(&view.context, self_directory, ResolveMode::Follow),
        )?;
        let self_link = fixture.consume_sync(0, {
            let vfs = view.vfs.clone();
            let fs_context = view.context.clone();
            let start = self_path.clone();
            let source = self_path.clone();
            move || vfs.link_from(&source, &fs_context, &start, b"self", (0, 0))
        })?;
        until("directory self-hardlink did not finish", || {
            self_link.result().is_some()
        })?;
        fixture.join_consumers()?;
        context.expect_eq(self_link.result(), Some(Err(FsError::NotPermitted)));

        checked(
            context,
            view.vfs
                .mkdir_from(&view.context, &self_path, b"child", 0o777, (0, 0), 0),
        )?;
        drop(fixture.open(context, b"/w26-nonempty/child", 0)?);
        context.expect_eq(
            view.vfs
                .rmdir_from(&view.context, &view.root, self_directory, (0, 0))
                .err(),
            Some(FsError::NotEmpty),
        );

        checked(
            context,
            view.vfs.mkdir_from(
                &view.context,
                &view.root,
                b"/w26-empty-source",
                0o777,
                (0, 0),
                0,
            ),
        )?;
        drop(fixture.open(context, b"/w26-empty-source", 0)?);
        context.expect_eq(
            view.vfs
                .rename_from(
                    &view.context,
                    &view.root,
                    b"w26-empty-source",
                    &view.root,
                    b"w26-nonempty",
                    false,
                    (0, 0),
                )
                .err(),
            Some(FsError::NotEmpty),
        );

        // Step 5: 验证作为 `FsContext` 当前工作目录（cwd）的空目录被删除后，无法再在该幽灵目录下创建子目录（返回 `NotFound`）。
        checked(
            context,
            view.vfs.mkdir_from(
                &view.context,
                &view.root,
                b"/w26-held-cwd",
                0o777,
                (0, 0),
                0,
            ),
        )?;
        drop(fixture.open(context, b"/w26-held-cwd", 0)?);
        let held_cwd = checked(
            context,
            view.vfs
                .resolve(&view.context, b"/w26-held-cwd", ResolveMode::Follow),
        )?;
        let held_context = checked(context, FsContext::new(view.root.clone(), held_cwd.clone()))?;
        checked(
            context,
            view.vfs
                .rmdir_from(&held_context, &view.root, b"/w26-held-cwd", (0, 0)),
        )?;
        context.expect_eq(
            view.vfs
                .mkdir_from(&held_context, &held_cwd, b"child", 0o777, (0, 0), 0)
                .err(),
            Some(FsError::NotFound),
        );
        context.expect_eq(
            view.vfs
                .resolve(&view.context, b"/w26-held-cwd", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );

        // Step 6: 将同一卷挂载到 `/w26-alias` 形成别名挂载点，验证跨挂载点 `link`/`rename` 返回 `CrossDevice`、删除或移动挂载点返回 `Busy`。
        checked(
            context,
            view.vfs
                .mkdir_from(&view.context, &view.root, b"/w26-alias", 0o777, (0, 0), 0),
        )?;
        drop(fixture.open(context, b"/w26-alias", 0)?);
        let alias_covered = checked(
            context,
            view.vfs
                .resolve(&view.context, b"/w26-alias", ResolveMode::Follow),
        )?;
        checked(
            context,
            view.vfs.mount(
                &alias_covered,
                crate::fs::Filesystem::Ext4(view.volume.clone()),
                AccessMode::ReadWrite,
            ),
        )?;
        let alias_root = checked(
            context,
            view.vfs
                .resolve(&view.context, b"/w26-alias", ResolveMode::Follow),
        )?;
        context.expect_eq(alias_root.ptr_eq(&alias_covered), false);
        context.expect_eq(alias_root.filesystem().ptr_eq(view.root.filesystem()), true);
        context.expect_eq(
            view.vfs
                .link_from(
                    &view.source,
                    &view.context,
                    &alias_root,
                    b"w26-cross-link",
                    (0, 0),
                )
                .err(),
            Some(FsError::CrossDevice),
        );
        context.expect_eq(
            view.vfs
                .rename_from(
                    &view.context,
                    &view.root,
                    b"pattern",
                    &alias_root,
                    b"w26-cross-rename",
                    false,
                    (0, 0),
                )
                .err(),
            Some(FsError::CrossDevice),
        );

        context.expect_eq(
            view.vfs
                .rmdir_from(&view.context, &view.root, b"/", (0, 0))
                .err(),
            Some(FsError::Busy),
        );
        context.expect_eq(
            view.vfs
                .rename_from(
                    &view.context,
                    &view.root,
                    b"/",
                    &view.root,
                    b"w26-root-move",
                    false,
                    (0, 0),
                )
                .err(),
            Some(FsError::Busy),
        );
        context.expect_eq(
            view.vfs
                .rmdir_from(&view.context, &view.root, b"/w26-alias", (0, 0))
                .err(),
            Some(FsError::Busy),
        );
        context.expect_eq(
            view.vfs
                .rename_from(
                    &view.context,
                    &view.root,
                    b"/w26-alias",
                    &view.root,
                    b"/w26-alias-moved",
                    false,
                    (0, 0),
                )
                .err(),
            Some(FsError::Busy),
        );
        drop(alias_root);
        checked(context, view.vfs.unmount(&alias_covered))?;

        // Step 7: 卸载 `/w26-alias` 后，验证非特权用户 `(1001, 1001)` 从 `0o555` 只读父目录移出子目录时因缺少父目录写权限返回 `Permission`。
        checked(
            context,
            view.vfs.mkdir_from(
                &view.context,
                &view.root,
                b"/w26-source-readonly",
                0o555,
                (0, 0),
                0,
            ),
        )?;
        let source_parent_file = fixture.open(context, b"/w26-source-readonly", 0)?;
        let source_parent = source_parent_file.path().unwrap().clone();
        checked(
            context,
            view.vfs.mkdir_from(
                &view.context,
                &source_parent,
                b"restricted",
                0o555,
                (0, 0),
                0,
            ),
        )?;
        drop(fixture.open(context, b"/w26-source-readonly/restricted", 0)?);
        checked(
            context,
            view.vfs.mkdir_from(
                &view.context,
                &view.root,
                b"/w26-target-parent",
                0o777,
                (0, 0),
                0,
            ),
        )?;
        let target_parent_file = fixture.open(context, b"/w26-target-parent", 0)?;
        let target_parent = target_parent_file.path().unwrap().clone();
        context.expect_eq(
            view.vfs
                .rename_from(
                    &view.context,
                    &source_parent,
                    b"restricted",
                    &target_parent,
                    b"restricted",
                    false,
                    (1001, 1001),
                )
                .err(),
            Some(FsError::Permission),
        );

        context.diagnostic(format_args!(
            "W26 shared-volume-ro-rw-positive-negative-and-current-names=1 old-path-inode-retained=1 alias-mount-rename-link=EXDEV self-directory-link=NotPermitted nonempty-rmdir-rename=ENOTEMPTY held-cwd-removal=1 root-mountpoint-removal-move=EBUSY restricted-cross-parent=EACCES"
        ));
        Ok(())
    }
}
