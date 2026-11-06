//! ext4 磁盘元数据编解码、目录名字哈希与 CRC 校验和模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 ext4 文件系统内部，专职处理原始小端字节切片与内核内存结构之间的编码与解码：
//! - **上游调用者**：`ext4` 卷管理（`ext4.rs`）、块与 inode 分配器（`allocation.rs`）、
//!   目录与 HTree 索引操作（`directory.rs`）、逻辑块映射（`mapping.rs`）以及 VFS Inode 层。
//! - **下游依赖**：仅依赖 `ext4::crc` 提供的 CRC16 / CRC32C 计算原语与文件系统类型常量，
//!   自身不持有任何锁，也不直接发起块设备或缓存 I/O。
//!
//! # 核心盘面布局与位域编码
//! 1. **Inode 记录布局（128 字节基础区 + 可选扩展区，最大 256 字节）**：
//! ```text
//! +--------+--------+--------------------------------------------------------+
//! | Offset | Length | Description                                            |
//! +--------+--------+--------------------------------------------------------+
//! | 0x00   | 40 B   | 基础属性（mode, uid_lo, size_lo, a/c/mtime, gid_lo,    |
//! |        |        | links_count, blocks_lo, flags, i_version_lo）          |
//! | 0x28   | 60 B   | i_block：Extent 树根节点（头+4项）、12直接+3间接指针、 |
//! |        |        | 快速符号链接内联字节或字符/块设备号                    |
//! | 0x64   | 28 B   | generation, file_acl_lo, size_high, blocks_high,       |
//! |        |        | file_acl_high, uid_high, gid_high, checksum_lo (124)   |
//! | 0x80   | extra  | extra_isize(128), checksum_hi(130), c/m/atime_extra    |
//! | 0x80+e | 剩余   | 内联扩展属性区（以 XATTR_MAGIC 0xea020000 开头）       |
//! +--------+--------+--------------------------------------------------------+
//! ```
//!
//! 2. **扩展时间戳（32 位 `*_extra` 字段）与 EA Inode 64 位引用计数**：
//! ```text
//! Inode 扩展时间戳字（ctime_extra / mtime_extra / atime_extra）：
//!  31                                        2  1        0
//! +-------------------------------------------+-----------+
//! |              nanoseconds (30b)            | epoch (2b)|
//! +-------------------------------------------+-----------+
//! 实际秒数 = (i32 基础秒 as i64) + ((epoch as i64) << 32)
//!
//! 隐藏 EA 值 Inode（EXT4_EA_INODE_FL）的 64 位引用计数复用字段：
//! +-----------------------------------+-----------------------------------+
//! |        i_ctime [63:32]            |       i_version (offset 36) [31:0]|
//! +-----------------------------------+-----------------------------------+
//! ```
//!
//! 3. **Extent 节点与叶子长度最高位状态编码**：
//! ```text
//! 节点字节布局：[ExtentHeader: 12B] + max * [ExtentEntry: 12B] + [CRC32C Tail: 4B]
//! 叶子 ee_len（u16）：
//!   0x0001 ..= 0x8000  => 已初始化数据段，块数为 raw（1..=32768）
//!   0x8001 ..= 0xFFFF  => 预分配未写入段（unwritten），块数为 raw - 0x8000（1..=32767）
//! ```
//!
//! # 并发契约与局部原地修补原则
//! - 本模块所有函数均为无锁纯计算函数，仅在调用方传入的切片边界内读写。
//! - 所有 `patch_*` 修改函数均在调用方持有的元数据缓冲区内容锁（`BufferRef::update`）内执行，
//!   且仅原地覆写目标记录的指定字段与对应的校验和，保留同一磁盘块内相邻记录与未知扩展字段不变，
//!   避免全块快照写回覆盖并发操作修改过的其他元数据记录。

use crate::block::SECTOR_BYTES;
pub(crate) use crate::fs::ext4::crc::{crc16, crc32c};
#[cfg(feature = "ktest")]
pub(crate) use crate::fs::ext4::directory::verify_directory;
pub(crate) use crate::fs::ext4::directory::DirectoryLayout;
use crate::fs::ext4::{Geometry, InodeData};
use crate::fs::inode::InodeMetadata;
use crate::fs::types::{S_IFBLK, S_IFCHR, S_IFDIR, S_IFIFO, S_IFLNK, S_IFMT, S_IFREG, S_IFSOCK};
use crate::fs::{FsError, InodeKind, InodeTime};
use crate::time::NSEC_PER_SEC;

/// Inode 标志（`EXT4_EXTENTS_FL`）：`i_block` 区域存放 Extent 树而非传统直接/间接块指针。
pub(crate) const EXTENTS: u32 = 0x0008_0000;
/// Inode 标志（`EXT4_EA_INODE_FL`）：该 inode 专用于存储大尺寸扩展属性（xattr）的值。
pub(crate) const EA_INODE: u32 = 0x0020_0000;
/// Inode 标志（`EXT4_INDEX_FL`）：目录启用了 HTree（DX）哈希树索引布局。
pub(crate) const INDEX: u32 = 0x1000;

/// 只读兼容特性位（`EXT4_FEATURE_RO_COMPAT_METADATA_CSUM`）：元数据启用 CRC32C 校验和。
pub(crate) const METADATA_CSUM: u32 = 0x400;

// Inode 标志字段（i_flags）与超级块特性字段处于各自独立的命名空间，同名特性的位掩码不同，切勿混用。
/// Inode 标志（`EXT4_HUGE_FILE_FL`）：`i_blocks` 按文件系统块大小而非 512 字节扇区为单位计数。
pub(crate) const HUGE_FILE: u32 = 0x0004_0000;
/// Inode 标志（`EXT4_COMPR_FL`）：文件启用透明压缩（当前不支持）。
const COMPR: u32 = 0x0000_0004;
/// Inode 标志（`EXT4_IMMUTABLE_FL`）：不可变文件标志。
const IMMUTABLE: u32 = 0x0000_0010;
/// Inode 标志（`EXT4_APPEND_FL`）：仅允许追加写入标志。
const APPEND: u32 = 0x0000_0020;
/// Inode 标志（`EXT4_ENCRYPT_FL`）：文件启用 fscrypt 加密（当前不支持）。
const ENCRYPT: u32 = 0x0000_0800;
/// Inode 标志（`EXT4_VERITY_FL`）：文件启用 fs-verity 只读完整性校验（当前不支持）。
const VERITY: u32 = 0x0010_0000;
/// Inode 标志（`EXT4_INLINE_DATA_FL`）：小文件或目录数据内联存储在 inode 内部（当前不支持）。
const INLINE_DATA: u32 = 0x1000_0000;
/// Inode 标志（`EXT4_CASEFOLD_FL`）：目录名比较启用大小写折叠（当前不支持）。
const CASEFOLD: u32 = 0x4000_0000;
// 兼容特性位（s_feature_compat）：未知兼容特性不影响安全读写。
pub(crate) const FEATURE_COMPAT_DIR_INDEX: u32 = 0x0020;
pub(crate) const FEATURE_COMPAT_SPARSE_SUPER2: u32 = 0x0200;

// 不兼容特性位（s_feature_incompat）：若包含未知位则必须拒绝挂载。
pub(crate) const FEATURE_INCOMPAT_FILETYPE: u32 = 0x0002;
const FEATURE_INCOMPAT_RECOVER: u32 = 0x0004;
pub(crate) const FEATURE_INCOMPAT_EXTENTS: u32 = 0x0040;
pub(crate) const FEATURE_INCOMPAT_64BIT: u32 = 0x0080;
const FEATURE_INCOMPAT_FLEX_BG: u32 = 0x0200;
pub(crate) const FEATURE_INCOMPAT_EA_INODE: u32 = 0x0400;
const FEATURE_INCOMPAT_CSUM_SEED: u32 = 0x2000;
pub(crate) const FEATURE_INCOMPAT_LARGEDIR: u32 = 0x4000;

// 只读兼容特性位（s_feature_ro_compat）：若包含不支持修改的格式位则拒绝挂载。
pub(crate) const FEATURE_RO_COMPAT_SPARSE_SUPER: u32 = 0x0001;
pub(crate) const FEATURE_RO_COMPAT_LARGE_FILE: u32 = 0x0002;
const FEATURE_RO_COMPAT_BTREE_DIR: u32 = 0x0004;
pub(crate) const FEATURE_RO_COMPAT_HUGE_FILE: u32 = 0x0008;
pub(crate) const FEATURE_RO_COMPAT_GDT_CSUM: u32 = 0x0010;
pub(crate) const FEATURE_RO_COMPAT_DIR_NLINK: u32 = 0x0020;
pub(crate) const FEATURE_RO_COMPAT_EXTRA_ISIZE: u32 = 0x0040;
const FEATURE_RO_COMPAT_BIGALLOC: u32 = 0x0200;
const FEATURE_RO_COMPAT_VERITY: u32 = 0x8000;

// 超级块位置、魔数与块尺寸边界（支持 1KiB / 2KiB / 4KiB 块）。
pub(crate) const SUPER_MAGIC: u16 = 0xef53;
pub(crate) const SUPERBLOCK_OFFSET: usize = 1024;
pub(crate) const SUPERBLOCK_BYTES: usize = 1024;
pub(crate) const SUPERBLOCK_END: usize = SUPERBLOCK_OFFSET + SUPERBLOCK_BYTES;
const SUPER_CHECKSUM_OFFSET: usize = SUPERBLOCK_BYTES - core::mem::size_of::<u32>();
const CRC32C_CHECKSUM: u8 = 1;

const MIN_BLOCK_BYTES: usize = 1024;
const MAX_BLOCK_LOG: u32 = 2;
pub(crate) const MAX_BLOCK_BYTES: usize = MIN_BLOCK_BYTES << MAX_BLOCK_LOG;

pub(crate) const ROOT_INODE: u64 = 2;
const FIRST_NON_RESERVED_INODE: u64 = 11;

pub(crate) const GOOD_OLD_INODE_BYTES: usize = 128;
pub(crate) const LARGE_INODE_BYTES: usize = 256;
pub(crate) const INODE_EXTRA_BYTES: usize = 160 - GOOD_OLD_INODE_BYTES;
const MIN_DESCRIPTOR_BYTES: usize = 32;
const LARGE_DESCRIPTOR_BYTES: usize = 64;

/// 块组描述符标志：该组的 inode 位图与表尚未在盘面初始化。
pub(crate) const BG_INODE_UNINIT: u16 = 0x0001;
/// 块组描述符标志：该组的数据块位图尚未在盘面初始化。
pub(crate) const BG_BLOCK_UNINIT: u16 = 0x0002;

pub(crate) const DIRECT_BLOCKS: usize = 12;
pub(crate) const INDIRECT_LEVELS: usize = 3;
pub(crate) const INODE_BLOCK_BYTES: usize =
    (DIRECT_BLOCKS + INDIRECT_LEVELS) * core::mem::size_of::<u32>();
const INODE_BLOCKS_LIMIT: u64 = 1u64 << (u32::BITS + u16::BITS);

pub(crate) const INODE_CTIME_EXTRA_OFFSET: usize = 132;
pub(crate) const INODE_MTIME_EXTRA_OFFSET: usize = 136;
pub(crate) const INODE_ATIME_EXTRA_OFFSET: usize = 140;
const EPOCH_BITS: u32 = 2;
const EPOCH_MASK: u32 = (1 << EPOCH_BITS) - 1;

pub(crate) const EXTENT_MAGIC: u16 = 0xf30a;
pub(crate) const EXTENT_HEADER_BYTES: usize = 12;
pub(crate) const EXTENT_ENTRY_BYTES: usize = 12;
pub(crate) const EXTENT_ROOT_ENTRIES: usize =
    (INODE_BLOCK_BYTES - EXTENT_HEADER_BYTES) / EXTENT_ENTRY_BYTES;
pub(crate) const EXTENT_LOGICAL_BLOCKS: u64 = 1u64 << u32::BITS;
pub(crate) const EXTENT_PHYSICAL_LIMIT: u64 = 1u64 << (u32::BITS + u16::BITS);
/// Extent 树的最大允许深度上限；遍历与修改路径栈均基于此常数定界（路径数组额外预留 1 个根节点槽位）。
pub(crate) const EXTENT_DEPTH_LIMIT: usize = 32;
/// 已初始化叶子 Extent 单项可表示的最大文件系统块数（`32768`，编码值 `0x8000`）。
pub(crate) const EXTENT_INIT_MAX_LEN: u16 = 1 << 15;
/// 未写入（unwritten）叶子 Extent 单项可表示的最大文件系统块数（`32767`，编码值 `0xFFFF`）。
pub(crate) const EXTENT_UNWRITTEN_MAX_LEN: u16 = EXTENT_INIT_MAX_LEN - 1;

pub(crate) const DIR_TAIL_BYTES: usize = 12;
pub(crate) const FT_DIR_CSUM: u8 = 0xde;

const XATTR_MAGIC: u32 = 0xea02_0000;
const XATTR_ENTRY_BYTES: usize = 16;
pub(crate) const MAX_EA_INODES: usize = MAX_BLOCK_BYTES / XATTR_ENTRY_BYTES;

const FLAGS_SIGNED_HASH: u32 = 0x0001;
const FLAGS_UNSIGNED_HASH: u32 = 0x0002;
const DX_HASH_LEGACY: u8 = 0;
const DX_HASH_HALF_MD4: u8 = 1;
const DX_HASH_TEA: u8 = 2;
const DX_HASH_LEGACY_UNSIGNED: u8 = 3;
const DX_HASH_HALF_MD4_UNSIGNED: u8 = 4;
pub(crate) const DX_HASH_TEA_UNSIGNED: u8 = 5;
const DX_HASH_SIPHASH: u8 = 6;

/// 从 `data` 中安全借用 `[offset, offset + len)` 范围的子切片。
///
/// 若 `offset + len` 发生整数溢出或超出 `data` 长度，返回 `FsError::Corrupt`。
pub(crate) fn bytes(data: &[u8], offset: usize, len: usize) -> Result<&[u8], FsError> {
    data.get(offset..offset.checked_add(len).ok_or(FsError::Corrupt)?)
        .ok_or(FsError::Corrupt)
}

/// 从 `data` 的 `offset` 字节处读取 16 位小端无符号整数，越界时返回 `FsError::Corrupt`。
pub(crate) fn le16(data: &[u8], offset: usize) -> Result<u16, FsError> {
    let b = bytes(data, offset, 2)?;
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

/// 从 `data` 的 `offset` 字节处读取 32 位小端无符号整数，越界时返回 `FsError::Corrupt`。
pub(crate) fn le32(data: &[u8], offset: usize) -> Result<u32, FsError> {
    let b = bytes(data, offset, 4)?;
    Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// 根据 HTree 目录指定的哈希算法版本计算目录项名称 `name` 的 `(major, minor)` 哈希值。
///
/// 返回的 `major` 会清除最低位（HTree 最低位保留作碰撞跨叶继续标志），并将全 1 边界值
/// `0xffff_fffe` 规避为 `0xffff_fffc`（避免与 HTree 索引游标结束哨兵冲突）。
/// 若遇到不支持的哈希版本（如 SipHash 或未知编号），返回 `FsError::Unsupported`。
pub(crate) fn directory_hash(
    geometry: &Geometry,
    hash_version: u8,
    name: &[u8],
) -> Result<(u32, u32), FsError> {
    let version = if hash_version <= DX_HASH_TEA && geometry.hash_unsigned {
        hash_version + DX_HASH_LEGACY_UNSIGNED
    } else {
        hash_version
    };
    let (mut major, minor) = match version {
        DX_HASH_LEGACY => (legacy_hash(name, false), 0),
        DX_HASH_HALF_MD4 | DX_HASH_HALF_MD4_UNSIGNED => half_md4_hash(
            name,
            version == DX_HASH_HALF_MD4_UNSIGNED,
            geometry.hash_seed,
        ),
        DX_HASH_TEA | DX_HASH_TEA_UNSIGNED => {
            tea_hash(name, version == DX_HASH_TEA_UNSIGNED, geometry.hash_seed)
        }
        DX_HASH_LEGACY_UNSIGNED => (legacy_hash(name, true), 0),
        DX_HASH_SIPHASH => return Err(FsError::Unsupported),
        _ => return Err(FsError::Unsupported),
    };

    major &= !1;
    if major == u32::MAX - 1 {
        major = u32::MAX - 3;
    }
    Ok((major, minor))
}

/// 计算不依赖超级块种子的传统（Legacy）32 位目录名哈希值。
///
/// `unsigned` 为 `true` 时将每个名称字节视为无符号 `0..=255`，否则按有符号 `i8` 符号扩展。
fn legacy_hash(name: &[u8], unsigned: bool) -> u32 {
    let mut hash0 = 0x12a3_fe2d_u32;
    let mut hash1 = 0x37ab_e8f9_u32;
    for &byte in name {
        let value = if unsigned {
            i32::from(byte) as u32
        } else {
            i32::from(byte as i8) as u32
        };
        let mut hash = hash1.wrapping_add(hash0 ^ value.wrapping_mul(7_152_373));
        if hash & 0x8000_0000 != 0 {
            hash = hash.wrapping_sub(0x7fff_ffff);
        }
        hash1 = hash0;
        hash0 = hash;
    }
    hash0.wrapping_shl(1)
}

/// 将目录名从 `offset` 起的一段字节打包并填充为 Half-MD4 或 TEA 哈希变换所需的 8 个 `u32` 输入字。
///
/// 不足整块的部分使用剩余字节数 `remaining` 重复拼接成的 32 位填充字补齐。
fn hash_buffer(
    name: &[u8],
    offset: usize,
    remaining: usize,
    words: usize,
    unsigned: bool,
) -> [u32; 8] {
    let pad = (remaining as u32) | ((remaining as u32) << 8);
    let pad = pad | (pad << 16);
    let mut result = [pad; 8];
    let mut value = pad;
    let mut word = 0usize;
    let count = core::cmp::min(remaining, words * 4);
    for index in 0..count {
        let byte = if unsigned {
            i32::from(name[offset + index])
        } else {
            i32::from(name[offset + index] as i8)
        };
        value = (byte as u32).wrapping_add(value.wrapping_shl(8));
        if index % 4 == 3 {
            result[word] = value;
            word += 1;
            value = pad;
        }
    }
    if count % 4 != 0 {
        result[word] = value;
    }
    result
}

/// 对 4 字状态向量执行一轮三阶段 Half-MD4 压缩变换，并返回更新后的主哈希字 `state[1]`。
fn half_md4_transform(state: &mut [u32; 4], input: &[u32; 8]) -> u32 {
    let [mut a, mut b, mut c, mut d] = *state;
    macro_rules! round {
        ($f:expr, $a:ident, $b:ident, $c:ident, $d:ident, $x:expr, $s:expr) => {
            $a = $a
                .wrapping_add($f($b, $c, $d))
                .wrapping_add($x)
                .rotate_left($s);
        };
    }
    let f = |x: u32, y: u32, z: u32| z ^ (x & (y ^ z));
    let g = |x: u32, y: u32, z: u32| (x & y).wrapping_add((x ^ y) & z);
    let h = |x: u32, y: u32, z: u32| x ^ y ^ z;

    round!(f, a, b, c, d, input[0], 3);
    round!(f, d, a, b, c, input[1], 7);
    round!(f, c, d, a, b, input[2], 11);
    round!(f, b, c, d, a, input[3], 19);
    round!(f, a, b, c, d, input[4], 3);
    round!(f, d, a, b, c, input[5], 7);
    round!(f, c, d, a, b, input[6], 11);
    round!(f, b, c, d, a, input[7], 19);

    const K2: u32 = 0x5a82_7999;
    round!(g, a, b, c, d, input[1].wrapping_add(K2), 3);
    round!(g, d, a, b, c, input[3].wrapping_add(K2), 5);
    round!(g, c, d, a, b, input[5].wrapping_add(K2), 9);
    round!(g, b, c, d, a, input[7].wrapping_add(K2), 13);
    round!(g, a, b, c, d, input[0].wrapping_add(K2), 3);
    round!(g, d, a, b, c, input[2].wrapping_add(K2), 5);
    round!(g, c, d, a, b, input[4].wrapping_add(K2), 9);
    round!(g, b, c, d, a, input[6].wrapping_add(K2), 13);

    const K3: u32 = 0x6ed9_eba1;
    round!(h, a, b, c, d, input[3].wrapping_add(K3), 3);
    round!(h, d, a, b, c, input[7].wrapping_add(K3), 9);
    round!(h, c, d, a, b, input[2].wrapping_add(K3), 11);
    round!(h, b, c, d, a, input[6].wrapping_add(K3), 15);
    round!(h, a, b, c, d, input[1].wrapping_add(K3), 3);
    round!(h, d, a, b, c, input[5].wrapping_add(K3), 9);
    round!(h, c, d, a, b, input[0].wrapping_add(K3), 11);
    round!(h, b, c, d, a, input[4].wrapping_add(K3), 15);

    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
    state[1]
}

/// 使用 Half-MD4 算法按 32 字节分组计算目录名 `name` 的 `(major, minor)` 哈希值。
///
/// 若超级块 `seed` 全为 0，则回退使用标准 MD4 初始幻数状态。
fn half_md4_hash(name: &[u8], unsigned: bool, seed: [u32; 4]) -> (u32, u32) {
    let mut state = if seed.iter().all(|&word| word == 0) {
        [0x6745_2301, 0xefcd_ab89, 0x98ba_dcfe, 0x1032_5476]
    } else {
        seed
    };
    let mut offset = 0usize;
    while offset < name.len() {
        let remaining = name.len() - offset;
        let input = hash_buffer(name, offset, remaining, 8, unsigned);
        half_md4_transform(&mut state, &input);
        offset += 32;
    }
    (state[1], state[2])
}

/// 对状态向量前两个字执行一轮 16 步 TEA（Tiny Encryption Algorithm）Feistel 轮变换。
fn tea_transform(state: &mut [u32; 4], input: &[u32; 8]) {
    const DELTA: u32 = 0x9e37_79b9;
    let mut sum = 0u32;
    let mut b0 = state[0];
    let mut b1 = state[1];
    for _ in 0..16 {
        sum = sum.wrapping_add(DELTA);
        b0 = b0.wrapping_add(
            (b1.wrapping_shl(4).wrapping_add(input[0]))
                ^ b1.wrapping_add(sum)
                ^ (b1.wrapping_shr(5).wrapping_add(input[1])),
        );
        b1 = b1.wrapping_add(
            (b0.wrapping_shl(4).wrapping_add(input[2]))
                ^ b0.wrapping_add(sum)
                ^ (b0.wrapping_shr(5).wrapping_add(input[3])),
        );
    }
    state[0] = state[0].wrapping_add(b0);
    state[1] = state[1].wrapping_add(b1);
}

/// 使用 TEA 算法按 16 字节分组计算目录名 `name` 的 `(major, minor)` 哈希值。
///
/// 若超级块 `seed` 全为 0，则回退使用标准初始幻数状态。
fn tea_hash(name: &[u8], unsigned: bool, seed: [u32; 4]) -> (u32, u32) {
    let mut state = if seed.iter().all(|&word| word == 0) {
        [0x6745_2301, 0xefcd_ab89, 0x98ba_dcfe, 0x1032_5476]
    } else {
        seed
    };
    let mut offset = 0usize;
    while offset < name.len() {
        let remaining = name.len() - offset;
        let input = hash_buffer(name, offset, remaining, 4, unsigned);
        tea_transform(&mut state, &input);
        offset += 16;
    }
    (state[0], state[1])
}

/// 初始化普通线性目录叶块末尾 12 字节的伪目录项尾部（`ext4_dir_entry_tail`），并更新 CRC32C 校验和。
///
/// 若卷未启用 `METADATA_CSUM` 特性则直接返回 `Ok(())`。调用方须持有该目录块的可变内容访问权。
pub(crate) fn update_directory_checksum(
    geometry: &Geometry,
    inode: &InodeMetadata,
    data: &mut [u8],
) -> Result<(), FsError> {
    if geometry.features_ro_compat & METADATA_CSUM == 0 {
        return Ok(());
    }
    let end = geometry
        .block_size
        .checked_sub(DIR_TAIL_BYTES)
        .ok_or(FsError::Corrupt)?;
    let tail = data
        .get_mut(end..geometry.block_size)
        .ok_or(FsError::Corrupt)?;
    tail.fill(0);
    put16(data, end + 4, DIR_TAIL_BYTES as u16);
    data[end + 7] = FT_DIR_CSUM;
    let checksum = crc32c(inode.checksum_seed, bytes(data, 0, end)?);
    put32(data, end + 8, checksum);
    Ok(())
}

/// 重算并写入 HTree 根节点或内部索引节点块尾（`ext4_dx_tail`）的 CRC32C 校验和。
///
/// `entries_end` 为当前已使用索引条目的结束偏移，`tail` 为 8 字节 `dx_tail` 结构的起始偏移。
pub(crate) fn update_dx_checksum(
    inode: &InodeMetadata,
    data: &mut [u8],
    entries_end: usize,
    tail: usize,
) -> Result<(), FsError> {
    put32(data, tail, 0);
    put32(data, tail + 4, 0);
    let checksum = dx_checksum(inode.checksum_seed, data, entries_end, tail)?;
    put32(data, tail + 4, checksum);
    Ok(())
}

/// 计算 HTree 节点块的 CRC32C 校验和。
///
/// 校验范围依次覆盖：`[0, entries_end)` 内的已用条目、`[tail, tail + 4)` 的保留字段，以及视作全零的 4 字节校验和字段本身。
pub(crate) fn dx_checksum(
    seed: u32,
    data: &[u8],
    entries_end: usize,
    tail: usize,
) -> Result<u32, FsError> {
    let mut checksum = crc32c(seed, bytes(data, 0, entries_end)?);
    checksum = crc32c(checksum, bytes(data, tail, 4)?);
    Ok(crc32c(checksum, &[0; 4]))
}

/// 校验普通目录叶块的末尾 CRC32C 校验和（`end` 为尾部 `DIR_TAIL_BYTES` 之前的有效目录项结束偏移）。
///
/// 若卷未启用 `METADATA_CSUM` 则跳过检查；校验和不匹配时返回 `FsError::Checksum`。
pub(crate) fn verify_directory_checksum(
    geometry: &Geometry,
    inode: &InodeMetadata,
    data: &[u8],
    end: usize,
) -> Result<(), FsError> {
    if geometry.features_ro_compat & METADATA_CSUM != 0
        && crc32c(inode.checksum_seed, bytes(data, 0, end)?) != le32(data, end + 8)?
    {
        return Err(FsError::Checksum);
    }
    Ok(())
}

/// 校验 HTree 根节点或内部索引节点块尾存储的 CRC32C 校验和，不匹配时返回 `FsError::Checksum`。
pub(crate) fn verify_dx_checksum(
    inode: &InodeMetadata,
    data: &[u8],
    entries_end: usize,
    tail: usize,
) -> Result<(), FsError> {
    if dx_checksum(inode.checksum_seed, data, entries_end, tail)? != le32(data, tail + 4)? {
        return Err(FsError::Checksum);
    }
    Ok(())
}

/// 计算无符号整数向上取整除法 `ceil(value / unit)`，避免 `(value + unit - 1)` 发生加法溢出。
///
/// 若除数 `unit` 为 0，返回 `FsError::Corrupt`。
pub(crate) fn ceil_div(value: u64, unit: u64) -> Result<u64, FsError> {
    if unit == 0 {
        return Err(FsError::Corrupt);
    }
    Ok(value / unit + u64::from(value % unit != 0))
}

/// 解码并校验 1024 字节主超级块，构造卷的不可变几何参数 [`Geometry`]。
///
/// 本实现不挂载 JBD2 日志或执行崩溃日志重放，仅依据超级块当前记录的特征位、块尺寸、块组划分与校验种子建立几何视图。
/// 若盘面不变量矛盾或校验和错误，返回 `FsError::Corrupt` / `FsError::Checksum`；遇到未实现的扩展特性则返回 `FsError::Unsupported`。
pub(crate) fn decode_super(data: &[u8], device_bytes: u64) -> Result<Geometry, FsError> {
    // Step 1: 校验超级块魔数，并在启用 METADATA_CSUM 时校验超级块自身的 CRC32C。
    let data = bytes(data, 0, SUPERBLOCK_BYTES)?;
    if le16(data, 56)? != SUPER_MAGIC {
        return Err(FsError::Corrupt);
    }

    let features_compat = le32(data, 92)?;
    let features_incompat = le32(data, 96)?;
    let features_ro_compat = le32(data, 100)?;
    if features_ro_compat & METADATA_CSUM != 0 {
        if data[373] != CRC32C_CHECKSUM {
            return Err(FsError::Unsupported);
        }
        if crc32c(!0, &data[..SUPER_CHECKSUM_OFFSET]) != le32(data, SUPER_CHECKSUM_OFFSET)? {
            return Err(FsError::Checksum);
        }
    }

    // Step 2: 校验兼容、不兼容与只读兼容特性集，拒绝未知不兼容位或未支持的布局特性（如 bigalloc、fs-verity、旧式 btree 目录）。
    let supported_incompat = FEATURE_INCOMPAT_FILETYPE
        | FEATURE_INCOMPAT_RECOVER
        | FEATURE_INCOMPAT_EXTENTS
        | FEATURE_INCOMPAT_64BIT
        | FEATURE_INCOMPAT_FLEX_BG
        | FEATURE_INCOMPAT_EA_INODE
        | FEATURE_INCOMPAT_CSUM_SEED
        | FEATURE_INCOMPAT_LARGEDIR;
    if features_incompat & !supported_incompat != 0
        || features_ro_compat
            & (FEATURE_RO_COMPAT_BTREE_DIR | FEATURE_RO_COMPAT_BIGALLOC | FEATURE_RO_COMPAT_VERITY)
            != 0
    {
        return Err(FsError::Unsupported);
    }
    if features_incompat & FEATURE_INCOMPAT_CSUM_SEED != 0
        && features_ro_compat & METADATA_CSUM == 0
    {
        return Err(FsError::Corrupt);
    }
    if le32(data, 72)? != 0 || 1 < le32(data, 76)? {
        return Err(FsError::Unsupported);
    }

    // Step 3: 解码文件系统块大小、簇（cluster）大小及首个数据块编号，验证 1KiB/2KiB/4KiB 布局约束。
    let log = le32(data, 24)?;
    if MAX_BLOCK_LOG < log {
        return Err(FsError::Unsupported);
    }
    let block_size = MIN_BLOCK_BYTES << log;
    if le32(data, 28)? != log || le32(data, 36)? != le32(data, 32)? {
        return Err(FsError::Corrupt);
    }
    let first_data_block = u64::from(le32(data, 20)?);
    if first_data_block != u64::from(block_size == MIN_BLOCK_BYTES) {
        return Err(FsError::Corrupt);
    }

    // Step 4: 解码块总数、inode 总数、每组容量、inode 记录尺寸（128 或 256 字节）与块组描述符尺寸（32 或 64 字节）。
    let has64 = features_incompat & FEATURE_INCOMPAT_64BIT != 0;
    let blocks_count = u64::from(le32(data, 4)?)
        | if has64 {
            u64::from(le32(data, 336)?) << 32
        } else {
            0
        };
    let inodes_count = u64::from(le32(data, 0)?);
    let blocks_per_group = u64::from(le32(data, 32)?);
    let inodes_per_group = u64::from(le32(data, 40)?);
    let first_inode = if le32(data, 76)? == 0 {
        FIRST_NON_RESERVED_INODE
    } else {
        u64::from(le32(data, 84)?)
    };
    let inode_size = if le32(data, 76)? == 0 {
        GOOD_OLD_INODE_BYTES
    } else {
        usize::from(le16(data, 88)?)
    };
    if !matches!(inode_size, GOOD_OLD_INODE_BYTES | LARGE_INODE_BYTES) {
        return Err(FsError::Unsupported);
    }
    let descriptor_size = if has64 {
        usize::from(le16(data, 254)?)
    } else {
        MIN_DESCRIPTOR_BYTES
    };
    if has64 && descriptor_size != LARGE_DESCRIPTOR_BYTES {
        return Err(FsError::Unsupported);
    }

    // Step 5: 校验位图容量、设备字节边界以及由块数和 inode 数分别推导出的块组总数一致性。
    if blocks_count <= first_data_block
        || inodes_count == 0
        || blocks_per_group == 0
        || inodes_per_group == 0
        || block_size as u64 * 8 < blocks_per_group
        || block_size as u64 * 8 < inodes_per_group
        || inodes_per_group % (block_size / inode_size) as u64 != 0
        || first_inode < FIRST_NON_RESERVED_INODE
        || inodes_per_group < first_inode
        || blocks_per_group % 8 != 0
        || inodes_per_group % 8 != 0
    {
        return Err(FsError::Corrupt);
    }
    if device_bytes
        < blocks_count
            .checked_mul(block_size as u64)
            .ok_or(FsError::Corrupt)?
    {
        return Err(FsError::Corrupt);
    }

    let groups_count = ceil_div(blocks_count - first_data_block, blocks_per_group)?;
    if u64::from(u32::MAX) < groups_count
        || groups_count != ceil_div(inodes_count, inodes_per_group)?
    {
        return Err(FsError::Corrupt);
    }
    let descriptor_blocks = ceil_div(
        groups_count
            .checked_mul(descriptor_size as u64)
            .ok_or(FsError::Corrupt)?,
        block_size as u64,
    )?;
    if blocks_count <= first_data_block + descriptor_blocks {
        return Err(FsError::Corrupt);
    }

    // Step 6: 提取卷 UUID、元数据 CRC32C 种子以及 HTree 目录名字哈希种子与默认版本。
    let mut uuid = [0u8; 16];
    uuid.copy_from_slice(bytes(data, 104, 16)?);
    let checksum_seed = if features_incompat & FEATURE_INCOMPAT_CSUM_SEED != 0 {
        le32(data, 624)?
    } else {
        crc32c(!0, &uuid)
    };
    let hash_seed = [
        le32(data, 236)?,
        le32(data, 240)?,
        le32(data, 244)?,
        le32(data, 248)?,
    ];
    let hash_flags = le32(data, 352)?;
    let hash_unsigned = hash_flags & FLAGS_UNSIGNED_HASH != 0
        || hash_flags & (FLAGS_SIGNED_HASH | FLAGS_UNSIGNED_HASH) == 0;
    let default_hash_version = data[252];
    if DX_HASH_TEA_UNSIGNED < default_hash_version {
        return Err(FsError::Unsupported);
    }
    let log_groups_per_flex = data[372];
    if 31 < log_groups_per_flex {
        return Err(FsError::Corrupt);
    }

    Ok(Geometry {
        block_size,
        inode_size,
        descriptor_size,
        blocks_count,
        inodes_count,
        features_compat,
        features_incompat,
        features_ro_compat,
        first_data_block,
        blocks_per_group,
        inodes_per_group,
        groups_count,
        descriptor_blocks,
        checksum_seed,
        hash_seed,
        hash_unsigned,
        default_hash_version,
        uuid,
        reserved_gdt_blocks: u64::from(le16(data, 206)?),
        backup_groups: [le32(data, 588)?, le32(data, 592)?],
        first_inode,
    })
}

/// 校验第 `group` 号块组描述符的完整性，并返回该组的 inode 表起始物理块号。
///
/// 会根据卷特性校验描述符 CRC32C 或传统 GDT CRC16，并验证块位图、inode 位图与 inode 表三者的物理块范围合法且互不重叠。
/// 启用 `FLEX_BG` 时允许这三个元数据区跨出当前块组落入卷内其他数据区，否则要求严格位于本块组范围内。
pub(crate) fn decode_group(g: &Geometry, group: u64, data: &[u8]) -> Result<u64, FsError> {
    let data = bytes(data, 0, g.descriptor_size)?;
    if g.groups_count <= group {
        return Err(FsError::Corrupt);
    }
    // Step 1: 校验块组描述符校验和（优先使用 METADATA_CSUM 的 CRC32C，否则检查 GDT_CSUM 的 CRC16）。
    let number = (group as u32).to_le_bytes();
    let stored = le16(data, 30)?;
    if g.features_ro_compat & METADATA_CSUM != 0 {
        let crc = crc32c(g.checksum_seed, &number);
        let crc = crc32c(crc32c(crc32c(crc, &data[..30]), &[0, 0]), &data[32..]);
        if crc as u16 != stored {
            return Err(FsError::Checksum);
        }
    } else if g.features_ro_compat & FEATURE_RO_COMPAT_GDT_CSUM != 0 {
        let crc = crc16(crc16(crc16(!0, &g.uuid), &number), &data[..30]);
        let crc = if g.descriptor_size == LARGE_DESCRIPTOR_BYTES {
            crc16(crc, &data[32..])
        } else {
            crc
        };
        if crc != stored {
            return Err(FsError::Checksum);
        }
    }

    // Step 2: 计算元数据允许存放的物理块边界（FLEX_BG 放宽至全卷数据区），逐一解码并验证块位图、inode 位图和 inode 表起点。
    let first = if g.features_incompat & FEATURE_INCOMPAT_FLEX_BG != 0 {
        g.first_data_block
    } else {
        g.first_data_block + group * g.blocks_per_group
    };
    let end = if g.features_incompat & FEATURE_INCOMPAT_FLEX_BG != 0 {
        g.blocks_count
    } else {
        core::cmp::min(first + g.blocks_per_group, g.blocks_count)
    };
    let mut pointers = [0u64; 3];
    for (index, value) in pointers.iter_mut().enumerate() {
        *value = u64::from(le32(data, index * 4)?);
        if g.descriptor_size == LARGE_DESCRIPTOR_BYTES {
            *value |= u64::from(le32(data, 32 + index * 4)?) << 32;
        }
        g.validate_blocks(*value, 1)?;
        if *value < first || end <= *value || *value <= g.first_data_block + g.descriptor_blocks {
            return Err(FsError::Corrupt);
        }
    }
    // Step 3: 计算 inode 表占用的总块数，确保块位图和 inode 位图互不重合且均不落入 inode 表区间内。
    let table_blocks = ceil_div(
        g.inodes_per_group
            .checked_mul(g.inode_size as u64)
            .ok_or(FsError::Corrupt)?,
        g.block_size as u64,
    )?;
    let table_end = pointers[2]
        .checked_add(table_blocks)
        .ok_or(FsError::Corrupt)?;
    if end < table_end
        || pointers[0] == pointers[1]
        || (pointers[2] <= pointers[0] && pointers[0] < table_end)
        || (pointers[2] <= pointers[1] && pointers[1] < table_end)
    {
        return Err(FsError::Corrupt);
    }
    Ok(pointers[2])
}

/// 解码并完整校验单个磁盘 inode 记录，构造内存 [`InodeData`] 快照。
///
/// 返回的 `blocks` 字段统一换算为 512 字节扇区数（即使盘面置位了 `HUGE_FILE` 按文件系统块计数也会在此折算），
/// 以便上层 `stat` 系统调用和空间配额统计使用统一单位。
pub(crate) fn decode_inode(g: &Geometry, number: u64, data: &[u8]) -> Result<InodeData, FsError> {
    let data = bytes(data, 0, g.inode_size)?;
    if number == 0 || g.inodes_count < number {
        return Err(FsError::Corrupt);
    }

    // Step 1: 解析扩展头长度 extra_isize，计算该 inode 专属的 CRC32C 种子（融合卷种子、inode 号与 generation）并验证记录校验和。
    let extra_isize = if g.inode_size == GOOD_OLD_INODE_BYTES {
        0
    } else {
        usize::from(le16(data, 128)?)
    };
    if g.inode_size < GOOD_OLD_INODE_BYTES + extra_isize || extra_isize & 3 != 0 {
        return Err(FsError::Corrupt);
    }

    let generation = le32(data, 100)?;
    let checksum_seed = crc32c(
        crc32c(g.checksum_seed, &(number as u32).to_le_bytes()),
        &generation.to_le_bytes(),
    );
    if g.features_ro_compat & METADATA_CSUM != 0 {
        let mut crc = crc32c(
            crc32c(crc32c(checksum_seed, &data[..124]), &[0, 0]),
            &data[126..128],
        );
        let mut stored = u32::from(le16(data, 124)?);
        if g.inode_size != GOOD_OLD_INODE_BYTES {
            crc = crc32c(crc, &data[128..130]);
            if 4 <= extra_isize {
                crc = crc32c(crc32c(crc, &[0, 0]), &data[132..]);
                stored |= u32::from(le16(data, 130)?) << 16;
            } else {
                crc = crc32c(crc, &data[130..]);
            }
        }
        if g.inode_size == GOOD_OLD_INODE_BYTES || extra_isize < 4 {
            crc &= 0xffff;
        }
        if stored != crc {
            return Err(FsError::Checksum);
        }
    }

    // Step 2: 解码文件类型模式位与 inode 标志位，拒绝压缩、加密、内联数据等未支持特性以及与超级块特性冲突的标志。
    let mode = le16(data, 0)?;
    let kind = match mode & S_IFMT {
        S_IFREG => InodeKind::Regular,
        S_IFDIR => InodeKind::Directory,
        S_IFLNK => InodeKind::Symlink,
        S_IFCHR => InodeKind::Character,
        S_IFBLK => InodeKind::Block,
        S_IFIFO => InodeKind::Fifo,
        S_IFSOCK => InodeKind::Socket,
        _ => return Err(FsError::Corrupt),
    };
    let flags = le32(data, 32)?;
    if flags & (COMPR | ENCRYPT | VERITY | INLINE_DATA | CASEFOLD) != 0 {
        return Err(FsError::Unsupported);
    }
    if flags & EXTENTS != 0 && g.features_incompat & FEATURE_INCOMPAT_EXTENTS == 0 {
        return Err(FsError::Corrupt);
    }
    if flags & EA_INODE != 0 && g.features_incompat & FEATURE_INCOMPAT_EA_INODE == 0 {
        return Err(FsError::Corrupt);
    }
    if kind == InodeKind::Symlink && flags & (IMMUTABLE | APPEND) != 0 {
        return Err(FsError::Corrupt);
    }

    // Step 3: 组合 64 位文件字节大小与已分配块计数，将 HUGE_FILE 模式下的文件系统块数统一折算为 512 字节扇区单位。
    let size = u64::from(le32(data, 4)?)
        | if kind == InodeKind::Regular || g.features_incompat & FEATURE_INCOMPAT_LARGEDIR != 0 {
            u64::from(le32(data, 108)?) << 32
        } else {
            0
        };
    if (i64::MAX as u64) < size {
        return Err(FsError::Corrupt);
    }
    let mut blocks = u64::from(le32(data, 28)?);
    if g.features_ro_compat & FEATURE_RO_COMPAT_HUGE_FILE != 0 {
        blocks |= u64::from(le16(data, 116)?) << 32;
        if flags & HUGE_FILE != 0 {
            blocks = blocks
                .checked_mul((g.block_size / SECTOR_BYTES as usize) as u64)
                .ok_or(FsError::Corrupt)?;
        }
    } else if flags & HUGE_FILE != 0 {
        return Err(FsError::Corrupt);
    }

    // Step 4: 解码外部扩展属性块（file_acl），并结合剩余扇区计数判断符号链接是否采用内联快速符号链接（fast symlink）编码。
    let file_acl = u64::from(le32(data, 104)?)
        | if g.features_incompat & FEATURE_INCOMPAT_64BIT != 0 {
            u64::from(le16(data, 118)?) << 32
        } else {
            0
        };
    if file_acl != 0 {
        g.validate_blocks(file_acl, 1)?;
    }
    let ea_blocks = if file_acl == 0 {
        0
    } else {
        (g.block_size / SECTOR_BYTES as usize) as u64
    };
    let fast_symlink = kind == InodeKind::Symlink
        && if flags & EA_INODE == 0 {
            blocks.checked_sub(ea_blocks).ok_or(FsError::Corrupt)? == 0
        } else {
            0 < size && size < INODE_BLOCK_BYTES as u64
        };
    if fast_symlink && INODE_BLOCK_BYTES as u64 <= size {
        return Err(FsError::Corrupt);
    }

    // Step 5: 拷贝 60 字节 i_block 区域，还原特殊设备号、32 位 UID/GID 及扩展纳秒时间戳。
    let mut block = [0u8; 60];
    block.copy_from_slice(bytes(data, 40, 60)?);
    let rdev = decode_device(kind, &block);
    Ok(InodeData {
        number,
        kind,
        mode,
        rdev,
        size,
        uid: u32::from(le16(data, 2)?) | (u32::from(le16(data, 120)?) << 16),
        gid: u32::from(le16(data, 24)?) | (u32::from(le16(data, 122)?) << 16),
        links: u32::from(le16(data, 26)?),
        blocks,
        atime: decode_time(data, 8, INODE_ATIME_EXTRA_OFFSET, extra_isize)?,
        mtime: decode_time(data, 16, INODE_MTIME_EXTRA_OFFSET, extra_isize)?,
        ctime: decode_time(data, 12, INODE_CTIME_EXTRA_OFFSET, extra_isize)?,
        generation,
        flags,
        block,
        checksum_seed,
        fast_symlink,
        file_acl,
        extra_isize,
    })
}

/// 从字符设备或块设备 inode 的 `i_block` 前 8 字节解码主次设备号（兼容旧版 `i_block[0]` 16 位编码与新版 `i_block[1]` 32 位编码）。
fn decode_device(kind: InodeKind, block: &[u8; 60]) -> u64 {
    if !matches!(kind, InodeKind::Character | InodeKind::Block) {
        return 0;
    }
    let old = u32::from_le_bytes([block[0], block[1], block[2], block[3]]);
    let (major, minor) = if old != 0 {
        (u64::from((old >> 8) & 0xff), u64::from(old & 0xff))
    } else {
        let encoded = u32::from_le_bytes([block[4], block[5], block[6], block[7]]);
        (
            u64::from((encoded >> 8) & 0xfff),
            u64::from((encoded & 0xff) | ((encoded >> 12) & 0xfff00)),
        )
    };
    encode_device(major, minor)
}

/// 将主设备号 `major` 与次设备号 `minor` 按 Linux `new_encode_dev` 64 位格式编码为统一设备号。
fn encode_device(major: u64, minor: u64) -> u64 {
    ((major & 0xffff_f000) << 32)
        | ((major & 0x0000_0fff) << 8)
        | ((minor & 0xffff_ff00) << 12)
        | (minor & 0x0000_00ff)
}

/// 解码 inode 时间戳：基础 32 位字段按有符号 `i32` 秒数符号扩展，仅当 `extra_isize` 覆盖对应扩展字段时才叠加上高 2 位 epoch 与 30 位纳秒。
fn decode_time(
    data: &[u8],
    base: usize,
    extra: usize,
    extra_isize: usize,
) -> Result<InodeTime, FsError> {
    let seconds = i64::from(le32(data, base)? as i32);
    let extension = if extra + 4 <= GOOD_OLD_INODE_BYTES + extra_isize {
        le32(data, extra)?
    } else {
        0
    };
    Ok(InodeTime {
        seconds: seconds + (i64::from(extension & EPOCH_MASK) << 32),
        nanoseconds: extension >> EPOCH_BITS,
    })
}

/// 校验元数据属性更新请求是否合法。
///
/// 属性更新仅允许修改权限位（不含文件类型 `S_IFMT`）、`uid`、`gid`、`links` 以及三个时间戳，不得篡改文件大小、块布局或 inode 身份；
/// 同时预先检查硬链接数是否超出 `u16` 上限、三个新时间戳是否能被当前 inode 的 `extra_isize` 无损编码。
pub(crate) fn validate_inode_metadata_update(
    old: &InodeMetadata,
    next: &InodeMetadata,
) -> Result<(), FsError> {
    let mut fixed = *next;
    fixed.mode = old.mode;
    fixed.rdev = old.rdev;
    fixed.uid = old.uid;
    fixed.gid = old.gid;
    fixed.links = old.links;
    fixed.atime = old.atime;
    fixed.mtime = old.mtime;
    fixed.ctime = old.ctime;
    if fixed != *old || old.mode & S_IFMT != next.mode & S_IFMT || (u16::MAX as u32) < next.links {
        return Err(FsError::InvalidOperation);
    }
    time_encoding(next.atime, INODE_ATIME_EXTRA_OFFSET, next.extra_isize)?;
    time_encoding(next.mtime, INODE_MTIME_EXTRA_OFFSET, next.extra_isize)?;
    time_encoding(next.ctime, INODE_CTIME_EXTRA_OFFSET, next.extra_isize)?;
    Ok(())
}

/// 将 [`InodeTime`] 编码为盘面基础秒字段与可选的 32 位扩展字段 `(base_sec, extra_word)`。
///
/// 若纳秒超出 `[0, 10^9)`，或当前 `extra_isize` 不足以容纳扩展字段但时间带有非零纳秒/跨 32 位纪元偏移，
/// 则返回 `FsError::InvalidOperation` 而非静默截断。
fn time_encoding(
    time: InodeTime,
    extra: usize,
    extra_isize: usize,
) -> Result<(u32, Option<u32>), FsError> {
    if (NSEC_PER_SEC as u32) <= time.nanoseconds {
        return Err(FsError::InvalidOperation);
    }
    let base = time.seconds as i32;
    let epoch = time
        .seconds
        .checked_sub(i64::from(base))
        .ok_or(FsError::InvalidOperation)?
        >> 32;
    if extra + 4 <= GOOD_OLD_INODE_BYTES + extra_isize {
        if !(0..=i64::from(EPOCH_MASK)).contains(&epoch) {
            return Err(FsError::InvalidOperation);
        }
        Ok((
            base as u32,
            Some((time.nanoseconds << EPOCH_BITS) | epoch as u32),
        ))
    } else {
        if epoch != 0 || time.nanoseconds != 0 {
            return Err(FsError::InvalidOperation);
        }
        Ok((base as u32, None))
    }
}

/// 根据当前 inode 的 `extra_isize` 将待写入时间戳规范化为盘面实际可表达的精度。
///
/// 若记录未包含该时间戳的扩展纳秒字段，则主动抹零纳秒部分（保留整秒精度），随后校验秒数范围是否合法。
pub(crate) fn prepare_inode_time(
    mut time: InodeTime,
    extra: usize,
    extra_isize: usize,
) -> Result<InodeTime, FsError> {
    if (NSEC_PER_SEC as u32) <= time.nanoseconds {
        return Err(FsError::InvalidOperation);
    }
    if GOOD_OLD_INODE_BYTES + extra_isize < extra + 4 {
        time.nanoseconds = 0;
    }
    time_encoding(time, extra, extra_isize)?;
    Ok(time)
}

/// 将单调或实时时钟纳秒值转换为匹配当前 inode 盘面精度的 `(mtime, ctime)` 时间戳对。
///
/// 由于 `mtime_extra`（偏移 136）与 `ctime_extra`（偏移 132）在扩展区位置不同，两者分别按各自的字段边界决定是否保留纳秒。
pub(crate) fn prepare_inode_write_times(
    nanoseconds: u64,
    extra_isize: usize,
) -> Result<(InodeTime, InodeTime), FsError> {
    let modified = InodeTime {
        seconds: (nanoseconds / NSEC_PER_SEC) as i64,
        nanoseconds: (nanoseconds % NSEC_PER_SEC) as u32,
    };
    let mtime = prepare_inode_time(modified, INODE_MTIME_EXTRA_OFFSET, extra_isize)?;
    let ctime = prepare_inode_time(modified, INODE_CTIME_EXTRA_OFFSET, extra_isize)?;
    Ok((mtime, ctime))
}

/// 在调用方持有的元数据块可变锁内原地更新单个 inode 记录的常规属性与校验和。
///
/// 修改前会先重新解码校验盘面当前属性与 `old` 一致；仅覆写权限、属主、链接数、三个时间戳及 CRC，
/// 保持 `i_block` 映射根、未知扩展字段以及同一磁盘块内的相邻 inode 记录原封不动。
pub(crate) fn patch_inode_metadata(
    g: &Geometry,
    record: &mut [u8],
    old: &InodeMetadata,
    next: &InodeMetadata,
) -> Result<(), FsError> {
    validate_inode_metadata_update(old, next)?;
    let current = decode_inode(g, old.number, record)?;
    if InodeMetadata::from_data(&current) != *old {
        return Err(FsError::Corrupt);
    }

    let record = record.get_mut(..g.inode_size).ok_or(FsError::Corrupt)?;
    let atime = time_encoding(next.atime, INODE_ATIME_EXTRA_OFFSET, next.extra_isize)?;
    let mtime = time_encoding(next.mtime, INODE_MTIME_EXTRA_OFFSET, next.extra_isize)?;
    let ctime = time_encoding(next.ctime, INODE_CTIME_EXTRA_OFFSET, next.extra_isize)?;

    put16(record, 0, next.mode);
    put16(record, 2, next.uid as u16);
    put16(record, 120, (next.uid >> 16) as u16);
    put16(record, 24, next.gid as u16);
    put16(record, 122, (next.gid >> 16) as u16);
    put16(record, 26, next.links as u16);
    put_time(record, 8, INODE_ATIME_EXTRA_OFFSET, atime);
    put_time(record, 16, INODE_MTIME_EXTRA_OFFSET, mtime);
    put_time(record, 12, INODE_CTIME_EXTRA_OFFSET, ctime);
    inode_checksum(g, record, next.checksum_seed, next.extra_isize);
    Ok(())
}

/// 向已通过长度校验的记录切片 `offset` 处写入 16 位小端整数。
pub(crate) fn put16(record: &mut [u8], offset: usize, value: u16) {
    record[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

/// 将预先通过 [`time_encoding`] 校验的时间戳编码无失败地写入记录的基础秒偏移 `base` 与可选扩展偏移 `extra`。
fn put_time(record: &mut [u8], base: usize, extra: usize, encoding: (u32, Option<u32>)) {
    record[base..base + 4].copy_from_slice(&encoding.0.to_le_bytes());
    if let Some(extension) = encoding.1 {
        record[extra..extra + 4].copy_from_slice(&extension.to_le_bytes());
    }
}

/// Extent 树节点头部的 12 字节解码结果（对应磁盘 `ext4_extent_header`）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExtentHeader {
    /// 当前节点内实际有效的条目数（`eh_entries`）。
    pub entries: u16,
    /// 当前节点可容纳的最大条目数（`eh_max`，内联根节点通常为 4，外部块节点由块大小决定）。
    pub max: u16,
    /// 当前节点距离叶子层的高度（`eh_depth`：`0` 表示存放数据映射的叶节点，大于 `0` 表示内部索引节点）。
    pub depth: u16,
}

/// 解码并校验 Extent 树节点头（`ext4_extent_header`）。
///
/// 检查魔数 `0xf30a`、最大深度上限、容量边界以及非零深度节点不能为空的不变量；
/// 若传入 `expected_depth`，还会强制要求节点记录的 `depth` 与父节点推导出的期望深度严格相等。
pub(crate) fn extent_header(
    data: &[u8],
    expected_depth: Option<u16>,
) -> Result<ExtentHeader, FsError> {
    if le16(data, 0)? != EXTENT_MAGIC {
        return Err(FsError::Corrupt);
    }
    let h = ExtentHeader {
        entries: le16(data, 2)?,
        max: le16(data, 4)?,
        depth: le16(data, 6)?,
    };
    if EXTENT_DEPTH_LIMIT < usize::from(h.depth)
        || h.max == 0
        || h.max < h.entries
        || data.len() < EXTENT_HEADER_BYTES + usize::from(h.max) * EXTENT_ENTRY_BYTES
        || (h.entries == 0 && h.depth != 0)
        || expected_depth.is_some_and(|depth| depth != h.depth)
    {
        return Err(FsError::Corrupt);
    }
    Ok(h)
}

/// 解码叶子 Extent 的 16 位长度字段 `ee_len`，返回 `(文件系统块数, 是否为未写入预分配段)`。
///
/// 盘面编码规则：`0` 非法；`1..=0x8000` 表示已初始化块（最大 32768 块）；`0x8001..=0xFFFF` 表示未写入（unwritten）块，实际长度为 `raw - 0x8000`。
pub(crate) fn extent_length(raw: u16) -> Result<(u64, bool), FsError> {
    if raw == 0 {
        return Err(FsError::Corrupt);
    }
    let unwritten = EXTENT_INIT_MAX_LEN < raw;
    Ok((
        u64::from(if unwritten {
            raw - EXTENT_INIT_MAX_LEN
        } else {
            raw
        }),
        unwritten,
    ))
}

/// 校验外部 Extent 树节点块末尾（`ext4_extent_tail`）的 CRC32C 校验和。
///
/// 内联在 inode `i_block` 中的根节点没有独立尾部，其完整性直接由所在 inode 记录的整体 CRC32C 保护。
pub(crate) fn verify_extent_checksum(
    g: &Geometry,
    inode: &InodeData,
    data: &[u8],
    header: ExtentHeader,
) -> Result<(), FsError> {
    if g.features_ro_compat & METADATA_CSUM != 0 {
        let tail = EXTENT_HEADER_BYTES + usize::from(header.max) * EXTENT_ENTRY_BYTES;
        if le32(data, tail)? != crc32c(inode.checksum_seed, bytes(data, 0, tail)?) {
            return Err(FsError::Checksum);
        }
    }
    Ok(())
}

/// 向已通过长度校验的记录切片 `offset` 处写入 32 位小端整数。
pub(crate) fn put32(record: &mut [u8], offset: usize, value: u32) {
    record[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// 解码并校验第 `group` 号块组描述符中的静态元数据块位置（块位图、inode 位图与 inode 表物理块号）。
///
/// 这些物理位置在格式化后固定不变，卷打开时缓存为 `GroupLayout`；而空闲计数与标志位则在每次分配/释放时从描述符缓冲区实时读取。
pub(crate) fn group_layout(
    g: &Geometry,
    group: u64,
    record: &[u8],
) -> Result<crate::fs::ext4::allocation::GroupLayout, FsError> {
    let inode_table = decode_group(g, group, record)?;
    g.validate_blocks(
        inode_table,
        g.inodes_per_group * g.inode_size as u64 / g.block_size as u64,
    )?;
    let high = |offset| -> Result<u64, FsError> {
        Ok(if g.descriptor_size == LARGE_DESCRIPTOR_BYTES {
            u64::from(le32(record, offset)?) << 32
        } else {
            0
        })
    };
    Ok(crate::fs::ext4::allocation::GroupLayout {
        block_bitmap: u64::from(le32(record, 0)?) | high(32)?,
        inode_bitmap: u64::from(le32(record, 4)?) | high(36)?,
        inode_table,
    })
}

/// 块组描述符中的动态可变计数与状态快照（兼容 32 字节与 64 字节描述符格式）。
#[derive(Clone, Copy)]
pub(crate) struct GroupState {
    /// 本块组剩余空闲数据块数量（`bg_free_blocks_count`）。
    pub free_blocks: u32,
    /// 本块组剩余空闲 inode 数量（`bg_free_inodes_count`）。
    pub free_inodes: u32,
    /// 本块组已分配的目录 inode 数量（`bg_used_dirs_count`）。
    pub used_dirs: u32,
    /// 块组标志位（`bg_flags`，如 [`BG_INODE_UNINIT`]、[`BG_BLOCK_UNINIT`]）。
    pub flags: u16,
    /// 本块组 inode 表尾部从未被使用过的连续槽位数量（`bg_itable_unused`）。
    pub unused_inodes: u32,
    /// 数据块位图的 CRC32C 校验和（32 字节描述符仅存低 16 位，64 字节存完整 32 位）。
    pub block_checksum: u32,
    /// Inode 位图的 CRC32C 校验和（32 字节描述符仅存低 16 位，64 字节存完整 32 位）。
    pub inode_checksum: u32,
}

/// 从块组描述符切片中解码动态状态 [`GroupState`]。
///
/// 当描述符大小为 64 字节时拼接高低两个 16 位半字，32 字节传统描述符仅读取低 16 位字段。
pub(crate) fn group_state(g: &Geometry, record: &[u8]) -> Result<GroupState, FsError> {
    let field = |low, high| -> Result<u32, FsError> {
        Ok(u32::from(le16(record, low)?)
            | if g.descriptor_size == LARGE_DESCRIPTOR_BYTES {
                u32::from(le16(record, high)?) << 16
            } else {
                0
            })
    };
    Ok(GroupState {
        free_blocks: field(12, 44)?,
        free_inodes: field(14, 46)?,
        used_dirs: field(16, 48)?,
        flags: le16(record, 18)?,
        unused_inodes: field(28, 50)?,
        block_checksum: field(24, 56)?,
        inode_checksum: field(26, 58)?,
    })
}

/// 在调用方持有卷分配锁及描述符缓冲区可变锁的前提下，原地写回 [`GroupState`] 各字段并重算块组描述符校验和（CRC32C 或 CRC16）。
pub(crate) fn patch_group(g: &Geometry, group: u64, record: &mut [u8], state: GroupState) {
    for (low, high, value) in [
        (12, 44, state.free_blocks),
        (14, 46, state.free_inodes),
        (16, 48, state.used_dirs),
        (28, 50, state.unused_inodes),
        (24, 56, state.block_checksum),
        (26, 58, state.inode_checksum),
    ] {
        put16(record, low, value as u16);
        if g.descriptor_size == LARGE_DESCRIPTOR_BYTES {
            put16(record, high, (value >> 16) as u16);
        }
    }
    put16(record, 18, state.flags);

    let number = (group as u32).to_le_bytes();
    if g.features_ro_compat & METADATA_CSUM != 0 {
        put16(record, 30, 0);
        let sum = crc32c(crc32c(g.checksum_seed, &number), record);
        put16(record, 30, sum as u16);
    } else if g.features_ro_compat & FEATURE_RO_COMPAT_GDT_CSUM != 0 {
        let sum = crc16(crc16(crc16(!0, &g.uuid), &number), &record[..30]);
        let sum = if g.descriptor_size == LARGE_DESCRIPTOR_BYTES {
            crc16(sum, &record[32..])
        } else {
            sum
        };
        put16(record, 30, sum);
    }
}

/// 计算单个块位图或 inode 位图的 CRC32C 校验和。
///
/// 按照 ext4 盘面规范，校验字节长度固定为 `inodes_per_group / 8` 或 `blocks_per_group / 8`（最后一个不满整组的块组同样按整组位宽计算）；
/// 32 字节描述符格式下截断保留低 16 位。
pub(crate) fn bitmap_checksum(g: &Geometry, bitmap: &[u8], inode: bool) -> u32 {
    let bits = if inode {
        g.inodes_per_group
    } else {
        g.blocks_per_group
    };
    let sum = crc32c(g.checksum_seed, &bitmap[..(bits / 8) as usize]);
    if g.descriptor_size == LARGE_DESCRIPTOR_BYTES {
        sum
    } else {
        sum & 0xffff
    }
}

/// 校验已初始化的数据块位图或 inode 位图的 CRC32C 校验和是否与块组描述符记录一致。
///
/// 带有 `UNINIT` 未初始化标志的位图在首次分配时由分配器在内存中直接构造，不经过此函数校验旧盘面字节。
#[cfg(feature = "ktest")]
pub(crate) fn verify_bitmap(
    g: &Geometry,
    record: &[u8],
    bitmap: &[u8],
    inode: bool,
) -> Result<(), FsError> {
    if g.features_ro_compat & METADATA_CSUM != 0 {
        let state = group_state(g, record)?;
        let stored = if inode {
            state.inode_checksum
        } else {
            state.block_checksum
        };
        if stored != bitmap_checksum(g, bitmap, inode) {
            return Err(FsError::Checksum);
        }
    }
    Ok(())
}

/// 在调用方持有卷分配锁（`allocation`）及主超级块可变锁时，按有符号增量更新全局空闲块数、空闲 inode 数、最近写入时间（`s_wtime`）与超级块 CRC32C。
pub(crate) fn patch_super_counts(
    g: &Geometry,
    record: &mut [u8],
    blocks: i64,
    inodes: i64,
    now_ns: u64,
) {
    let old_blocks = u64::from(le32(record, 12).expect("prepared superblock"))
        | if g.features_incompat & FEATURE_INCOMPAT_64BIT != 0 {
            u64::from(le32(record, 344).expect("prepared superblock")) << 32
        } else {
            0
        };
    let old_inodes = u64::from(le32(record, 16).expect("prepared superblock"));
    let next_blocks = old_blocks
        .checked_add_signed(blocks)
        .expect("free block count overflow");
    let next_inodes = old_inodes
        .checked_add_signed(inodes)
        .expect("free inode count overflow");
    assert!(next_blocks <= g.blocks_count && next_inodes <= g.inodes_count);

    put32(record, 12, next_blocks as u32);
    if g.features_incompat & FEATURE_INCOMPAT_64BIT != 0 {
        put32(record, 344, (next_blocks >> 32) as u32);
    }
    put32(record, 16, next_inodes as u32);

    // 盘面规范：偏移 48 的 s_wtime（低 32 位）与偏移 0x274 的 s_wtime_hi（高 8 位）共同组成 40 位写入秒数。
    let write_seconds = now_ns / NSEC_PER_SEC;
    put32(record, 48, write_seconds as u32);
    record[0x274] = (write_seconds >> 32) as u8;
    if g.features_ro_compat & METADATA_CSUM != 0 {
        put32(
            record,
            SUPER_CHECKSUM_OFFSET,
            crc32c(!0, &record[..SUPER_CHECKSUM_OFFSET]),
        );
    }
}

/// 将内存统一维护的 512 字节扇区计数 `sectors` 转换为盘面 `i_blocks` 计数值与是否置位 `HUGE_FILE` 标志。
///
/// - 若 `sectors <= u32::MAX`，直接以 512 字节扇区为单位编码，不置 `HUGE_FILE`；
/// - 若超出 32 位上限但启用了 `RO_COMPAT_HUGE_FILE`：48 位扇区数装得下时仍按扇区编码，否则切换为按文件系统块为单位并置位 `HUGE_FILE`；
/// - 若仍超出 48 位上限或卷未开启 `RO_COMPAT_HUGE_FILE`，返回 `FsError::FileTooLarge`。
pub(crate) fn blocks_encoding(g: &Geometry, sectors: u64) -> Result<(u64, bool), FsError> {
    if sectors <= u32::MAX as u64 {
        return Ok((sectors, false));
    }
    if g.features_ro_compat & FEATURE_RO_COMPAT_HUGE_FILE == 0 {
        return Err(FsError::FileTooLarge);
    }
    if sectors < INODE_BLOCKS_LIMIT {
        return Ok((sectors, false));
    }
    let blocks = sectors / (g.block_size / SECTOR_BYTES as usize) as u64;
    if INODE_BLOCKS_LIMIT <= blocks {
        return Err(FsError::FileTooLarge);
    }
    Ok((blocks, true))
}

/// 在元数据块可变锁内原地更新单个 inode 的大小、块计数、`HUGE_FILE` 标志、60 字节块映射根、`mtime`/`ctime` 及校验和。
///
/// 调用方在进入此函数前已通过预检保证时间戳与块计数可无损编码；同一磁盘块内的相邻 inode 记录及本记录其余字段保持不变。
pub(crate) fn patch_inode_mapping(
    g: &Geometry,
    record: &mut [u8],
    next: &InodeMetadata,
    root: &[u8; 60],
) {
    let (blocks, huge) = blocks_encoding(g, next.blocks).expect("prepared inode blocks");
    let mtime = time_encoding(next.mtime, INODE_MTIME_EXTRA_OFFSET, next.extra_isize)
        .expect("prepared inode mtime");
    let ctime = time_encoding(next.ctime, INODE_CTIME_EXTRA_OFFSET, next.extra_isize)
        .expect("prepared inode ctime");

    put32(record, 4, next.size as u32);
    put32(record, 108, (next.size >> 32) as u32);
    put32(record, 28, blocks as u32);
    put16(record, 116, (blocks >> 32) as u16);
    put32(
        record,
        32,
        (next.flags & !HUGE_FILE) | if huge { HUGE_FILE } else { 0 },
    );
    record[40..100].copy_from_slice(root);
    put_time(record, 16, INODE_MTIME_EXTRA_OFFSET, mtime);
    put_time(record, 12, INODE_CTIME_EXTRA_OFFSET, ctime);
    inode_checksum(g, record, next.checksum_seed, next.extra_isize);
}

/// 先将 inode 记录内的校验和字段（偏移 124 低 16 位及可选偏移 130 高 16 位）清零，再以 `seed` 计算整条记录的 CRC32C 并写回。
pub(crate) fn inode_checksum(g: &Geometry, record: &mut [u8], seed: u32, extra: usize) {
    if g.features_ro_compat & METADATA_CSUM == 0 {
        return;
    }
    put16(record, 124, 0);
    if 4 <= extra {
        put16(record, 130, 0);
    }
    let sum = crc32c(seed, record);
    put16(record, 124, sum as u16);
    if 4 <= extra {
        put16(record, 130, (sum >> 16) as u16);
    }
}

/// 在 inode 彻底释放回位图前清理其磁盘记录：清零大小、块数、链接数与外部 ACL 块指针，重置空的 `i_block`（保留 Extent 头），清除内联大 EA 引用，写入 `i_dtime` 删除时间并重算 CRC。
///
/// 保留 `i_generation`、未知扩展字段及同块相邻 inode 记录，供后续分配复用时递增版本号。
pub(crate) fn patch_inode_release(g: &Geometry, record: &mut [u8], seed: u32, extra: usize) {
    let flags = le32(record, 32).expect("prepared freed inode flags") & !HUGE_FILE;
    let deletion_time = le32(record, 12).expect("prepared freed inode ctime");
    put32(record, 4, 0);
    put32(record, 108, 0);
    put32(record, 28, 0);
    put16(record, 116, 0);
    put16(record, 26, 0);
    put32(record, 104, 0);
    if g.features_incompat & FEATURE_INCOMPAT_64BIT != 0 {
        put16(record, 118, 0);
    }
    put32(record, 32, flags);
    record[40..100].fill(0);
    if flags & EXTENTS != 0 {
        put16(record, 40, EXTENT_MAGIC);
        put16(record, 44, EXTENT_ROOT_ENTRIES as u16);
    }
    if g.features_incompat & FEATURE_INCOMPAT_EA_INODE != 0 {
        clear_inline_ea_references(record, extra);
    }
    put32(record, 20, deletion_time);
    inode_checksum(g, record, seed, extra);
}

/// 计算外部独立扩展属性（xattr）物理块的 CRC32C 校验和。
///
/// 种子由卷种子与 64 位物理块号小端字节混合生成，偏移 `[16, 20)` 的校验和字段自身按全零参与整块计算。
fn xattr_block_checksum(g: &Geometry, physical: u64, bytes: &[u8]) -> u32 {
    let seed = crc32c(g.checksum_seed, &physical.to_le_bytes());
    let prefix = crc32c(seed, &bytes[..16]);
    crc32c(crc32c(prefix, &[0; 4]), &bytes[20..g.block_size])
}

/// 在释放外部扩展属性块前校验其头部魔数、单块计数、非零引用计数以及整块 CRC32C，并提取其中引用的全部大属性（EA inode）编号填入 `numbers`，返回提取数量。
pub(crate) fn verify_xattr_release(
    g: &Geometry,
    physical: u64,
    data: &[u8],
    numbers: &mut [u64; MAX_EA_INODES],
) -> Result<usize, FsError> {
    let data = bytes(data, 0, g.block_size)?;
    if le32(data, 0)? != XATTR_MAGIC || le32(data, 8)? != 1 || le32(data, 4)? == 0 {
        return Err(FsError::Corrupt);
    }
    if g.features_ro_compat & METADATA_CSUM != 0
        && le32(data, 16)? != xattr_block_checksum(g, physical, data)
    {
        return Err(FsError::Checksum);
    }
    if g.features_incompat & FEATURE_INCOMPAT_EA_INODE != 0 {
        return xattr_ea_inodes(data, 32, numbers);
    }
    Ok(0)
}

/// 扫描 inode 记录扩展区之后的内联扩展属性表，提取其中引用外部 EA 值 inode 的编号填入 `numbers` 并返回数量。
///
/// 直接内联存放值字节的普通扩展属性随 inode 记录本身一同失效，不占用独立块或外部 inode。
pub(crate) fn inline_ea_inodes(
    g: &Geometry,
    record: &[u8],
    extra: usize,
    numbers: &mut [u64; MAX_EA_INODES],
) -> Result<usize, FsError> {
    if g.features_incompat & FEATURE_INCOMPAT_EA_INODE == 0 {
        return Ok(0);
    }
    let header = GOOD_OLD_INODE_BYTES + extra;
    if header + 4 <= record.len() && le32(record, header)? == XATTR_MAGIC {
        return xattr_ea_inodes(record, header + 4, numbers);
    }
    Ok(0)
}

/// 从给定 `offset` 开始顺序遍历 xattr 条目表（以 4 字节全零头结束），收集所有 `e_value_inum != 0` 的 EA 值 inode 编号。
///
/// 每个条目长度按 `16 + name_len` 向上对齐到 4 字节边界；`MAX_EA_INODES` 足以容纳单个 4KiB 块内的最大条目数。
fn xattr_ea_inodes(
    data: &[u8],
    mut offset: usize,
    numbers: &mut [u64; MAX_EA_INODES],
) -> Result<usize, FsError> {
    let mut count = 0;
    loop {
        if le32(data, offset)? == 0 {
            return Ok(count);
        }
        let entry = bytes(data, offset, XATTR_ENTRY_BYTES)?;
        let number = u64::from(le32(entry, 4)?);
        if number != 0 {
            numbers[count] = number;
            count += 1;
        }
        offset += (XATTR_ENTRY_BYTES + usize::from(entry[0]) + 3) & !3;
    }
}

/// 在回收 inode 记录时将内联扩展属性表中指向外部 EA 值 inode 的编号（`e_value_inum`）与值长度清零。
fn clear_inline_ea_references(record: &mut [u8], extra: usize) {
    let header = GOOD_OLD_INODE_BYTES + extra;
    if record.len() < header + 4
        || le32(record, header).expect("prepared inline EA header") != XATTR_MAGIC
    {
        return;
    }
    let mut offset = header + 4;
    while le32(record, offset).expect("prepared inline EA entry") != 0 {
        let span = (XATTR_ENTRY_BYTES + usize::from(record[offset]) + 3) & !3;
        if le32(record, offset + 4).expect("prepared inline EA reference") != 0 {
            put32(record, offset + 4, 0);
            put32(record, offset + 8, 0);
        }
        offset += span;
    }
}

/// 解码专用于存放大尺寸扩展属性值的隐藏 EA inode 的 64 位引用计数（高 32 位存于 `i_ctime`，低 32 位存于偏移 36 的 `i_version`）。
pub(crate) fn ea_inode_references(record: &[u8]) -> Result<u64, FsError> {
    Ok((u64::from(le32(record, 12)?) << 32) | u64::from(le32(record, 36)?))
}

/// 在目标 EA 值 inode 的写锁与卷元数据更新保护下，将其 64 位 EA 引用计数扣减 `count` 并原地更新记录与校验和。
///
/// 当引用计数归零时同步将 `i_links_count` 置为 0，使其在最后一个内存 `InodeRef` 释放时走标准无链接 inode 回收流程释放数据块与位图。
pub(crate) fn patch_ea_inode_references(
    g: &Geometry,
    record: &mut [u8],
    next: &mut InodeMetadata,
    count: u64,
) {
    let references = ea_inode_references(record)
        .expect("prepared EA inode reference")
        .checked_sub(count)
        .expect("EA inode reference underflow");
    next.links = if references == 0 { 0 } else { 1 };
    next.ctime = InodeTime {
        seconds: (references >> 32) as i64,
        nanoseconds: 0,
    };
    let ctime = time_encoding(next.ctime, INODE_CTIME_EXTRA_OFFSET, next.extra_isize)
        .expect("prepared EA inode ctime");
    put_time(record, 12, INODE_CTIME_EXTRA_OFFSET, ctime);
    put32(record, 36, references as u32);
    if 156 <= GOOD_OLD_INODE_BYTES + next.extra_isize {
        put32(record, 152, 0);
    }
    put16(record, 26, next.links as u16);
    inode_checksum(g, record, next.checksum_seed, next.extra_isize);
}

/// 更新多个 inode 共享的外部扩展属性块头部引用计数（`h_refcount`），并在启用 `METADATA_CSUM` 时重算整块 CRC32C。
pub(crate) fn patch_xattr_refcount(
    g: &Geometry,
    physical: u64,
    record: &mut [u8],
    references: u32,
) {
    put32(record, 4, references);
    if g.features_ro_compat & METADATA_CSUM != 0 {
        put32(record, 16, xattr_block_checksum(g, physical, record));
    }
}
