//! 磁盘结构仅从有界小端字节解码，校验和与布局共用同一实现。

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

pub(crate) const EXTENTS: u32 = 0x0008_0000;
pub(crate) const EA_INODE: u32 = 0x0020_0000;
pub(crate) const INDEX: u32 = 0x1000;
pub(crate) const METADATA_CSUM: u32 = 0x400;
// inode 标志与超级块 feature 属于不同字段；同名能力不能共用位值。
pub(crate) const HUGE_FILE: u32 = 0x0004_0000;
const COMPR: u32 = 0x0000_0004;
const IMMUTABLE: u32 = 0x0000_0010;
const APPEND: u32 = 0x0000_0020;
const ENCRYPT: u32 = 0x0000_0800;
const VERITY: u32 = 0x0010_0000;
const INLINE_DATA: u32 = 0x1000_0000;
const CASEFOLD: u32 = 0x4000_0000;

pub(crate) const FEATURE_COMPAT_DIR_INDEX: u32 = 0x0020;
pub(crate) const FEATURE_COMPAT_SPARSE_SUPER2: u32 = 0x0200;
pub(crate) const FEATURE_INCOMPAT_FILETYPE: u32 = 0x0002;
const FEATURE_INCOMPAT_RECOVER: u32 = 0x0004;
pub(crate) const FEATURE_INCOMPAT_EXTENTS: u32 = 0x0040;
pub(crate) const FEATURE_INCOMPAT_64BIT: u32 = 0x0080;
const FEATURE_INCOMPAT_FLEX_BG: u32 = 0x0200;
pub(crate) const FEATURE_INCOMPAT_EA_INODE: u32 = 0x0400;
const FEATURE_INCOMPAT_CSUM_SEED: u32 = 0x2000;
pub(crate) const FEATURE_INCOMPAT_LARGEDIR: u32 = 0x4000;
pub(crate) const FEATURE_RO_COMPAT_SPARSE_SUPER: u32 = 0x0001;
pub(crate) const FEATURE_RO_COMPAT_LARGE_FILE: u32 = 0x0002;
const FEATURE_RO_COMPAT_BTREE_DIR: u32 = 0x0004;
pub(crate) const FEATURE_RO_COMPAT_HUGE_FILE: u32 = 0x0008;
pub(crate) const FEATURE_RO_COMPAT_GDT_CSUM: u32 = 0x0010;
pub(crate) const FEATURE_RO_COMPAT_DIR_NLINK: u32 = 0x0020;
pub(crate) const FEATURE_RO_COMPAT_EXTRA_ISIZE: u32 = 0x0040;
const FEATURE_RO_COMPAT_BIGALLOC: u32 = 0x0200;
const FEATURE_RO_COMPAT_VERITY: u32 = 0x8000;

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
pub(crate) const BG_INODE_UNINIT: u16 = 0x0001;
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
// 树遍历与准备路径使用同一深度上限，路径栈另留一个根节点位置。
pub(crate) const EXTENT_DEPTH_LIMIT: usize = 32;
pub(crate) const EXTENT_INIT_MAX_LEN: u16 = 1 << 15;
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

/// 借用明确的字节范围，拒绝短输入与加法溢出。
pub(crate) fn bytes(data: &[u8], offset: usize, len: usize) -> Result<&[u8], FsError> {
    data.get(offset..offset.checked_add(len).ok_or(FsError::Corrupt)?)
        .ok_or(FsError::Corrupt)
}

/// 解码有界的十六位小端值。
pub(crate) fn le16(data: &[u8], offset: usize) -> Result<u16, FsError> {
    let b = bytes(data, offset, 2)?;
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

/// 解码有界的三十二位小端值。
pub(crate) fn le32(data: &[u8], offset: usize) -> Result<u32, FsError> {
    let b = bytes(data, offset, 4)?;
    Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}
/// 按目录哈希版本计算原始字节的 major/minor 哈希。
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

/// 计算不使用目录种子的旧版 32 位名字哈希。
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

/// 将一个名字分组转换为半 MD4 使用的整数缓冲。
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

/// 以三轮布尔函数更新半 MD4 状态并返回其主要哈希字。
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

/// 计算半 MD4 的 major/minor 哈希。
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

/// 执行 TEA 目录哈希的一轮十六步变换。
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

/// 计算 TEA 的 major/minor 哈希。
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

/// 重新计算带 metadata_csum 的普通目录尾 CRC。
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

/// 按 dx 中实际使用的条目计算 root 或 index-node 尾 CRC。
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

/// dx checksum 覆盖已用条目、tail 保留字与置零的 checksum 字段。
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

/// 从 metadata_csum 目录尾验证普通 dirent 块 CRC。
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

/// 解码 dx 根或 index-node 的已用条目数并验证真实尾 CRC。
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

/// 向上取整块数，不让外部计数加法溢出。
pub(crate) fn ceil_div(value: u64, unit: u64) -> Result<u64, FsError> {
    if unit == 0 {
        return Err(FsError::Corrupt);
    }
    Ok(value / unit + u64::from(value % unit != 0))
}

/// 从当前盘面数据建立只读几何；日志状态不参与解码，也不执行日志恢复。
pub(crate) fn decode_super(data: &[u8], device_bytes: u64) -> Result<Geometry, FsError> {
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
    // 只读兼容位可忽略未知值，已知扩展读取格式仍须明确拒绝。
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
        log_groups_per_flex,
        first_inode,
    })
}

/// 校验组描述符并提取完整 inode 表地址；flex 组允许表位于其他块组。
pub(crate) fn decode_group(g: &Geometry, group: u64, data: &[u8]) -> Result<u64, FsError> {
    let data = bytes(data, 0, g.descriptor_size)?;
    if g.groups_count <= group {
        return Err(FsError::Corrupt);
    }
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

/// 解码并校验完整 inode，保留块计数的统一 512 字节单位。
pub(crate) fn decode_inode(g: &Geometry, number: u64, data: &[u8]) -> Result<InodeData, FsError> {
    let data = bytes(data, 0, g.inode_size)?;
    if number == 0 || g.inodes_count < number {
        return Err(FsError::Corrupt);
    }
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

/// 按 ext4 特殊 inode 的旧版或新版盘面编码还原内核设备号。
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

/// 将 major/minor 组合成本内核与 Linux new_encode_dev 相同的设备号。
fn encode_device(major: u64, minor: u64) -> u64 {
    ((major & 0xffff_f000) << 32)
        | ((major & 0x0000_0fff) << 8)
        | ((minor & 0xffff_ff00) << 12)
        | (minor & 0x0000_00ff)
}

/// 基础秒有符号扩展；只有 extra_isize 覆盖该字段时才消费 epoch 和纳秒。
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

/// 受控属性更新不改变文件内容、块布局或 inode 身份；格式有限位宽在改字节前检查。
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

/// 根据本 inode 的真实 extra_isize 编码秒扩展与纳秒，不静默截断不可表示时间。
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

/// 按该时间字段的真实记录精度准备值，整秒记录不保留无法写回的纳秒。
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

/// 写提交使用已有时钟的时间值，mtime 和 ctime 各按实际扩展字段保留纳秒或整秒。
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

/// 同一元数据 guard 内只改当前记录的属性和 CRC，保留块根、未知字段及邻记录。
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

/// 写入已由固定 inode 几何验证的字段，修改不跨出本记录。
pub(crate) fn put16(record: &mut [u8], offset: usize, value: u16) {
    record[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

/// 使用预先检查的时间编码，不在修改了部分记录后再引入可失败步骤。
fn put_time(record: &mut [u8], base: usize, extra: usize, encoding: (u32, Option<u32>)) {
    record[base..base + 4].copy_from_slice(&encoding.0.to_le_bytes());
    if let Some(extension) = encoding.1 {
        record[extra..extra + 4].copy_from_slice(&extension.to_le_bytes());
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ExtentHeader {
    pub entries: u16,
    pub max: u16,
    pub depth: u16,
}

/// 解码 extent 头，并在下降时要求深度严格匹配。
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

/// 0x8000 仍是已初始化的 32768 块，只有更大的编码表示未写入。
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

/// 验证已解码外部 extent 节点的正式尾 CRC，根节点由 inode CRC 保护。
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

/// 写入已经过几何检查的四字节字段，不复制共同块的其余字节。
pub(crate) fn put32(record: &mut [u8], offset: usize, value: u32) {
    record[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

/// 组的物理布局固定，计数与 flags 每次从共同描述符重新解码。
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

#[derive(Clone, Copy)]
pub(crate) struct GroupState {
    pub free_blocks: u32,
    pub free_inodes: u32,
    pub used_dirs: u32,
    pub flags: u16,
    pub unused_inodes: u32,
    pub block_checksum: u32,
    pub inode_checksum: u32,
}

/// 组合描述符的低高半字段，32 字节格式只消费实际存在的低半。
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

/// 对当前描述符作字段级提交，CRC 消费包含邻字段的当前记录。
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

/// 位图 CRC 只覆盖每组实际格式规定的位数，末组仍使用整组校验长度。
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

/// 已初始化位图先校验正式 CRC，UNINIT 的构造路径由分配器单独负责。
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

/// 在主超级块当前字段上提交空闲计数与本次写入时间；调用者已经串行化卷的分配。
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
    // ABI: s_wtime 与 s_wtime_hi 共同编码 40 位秒数，使用与 inode 时间相同的时钟。
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

/// 根据统一 sector 计数选择盘面编码；超过当前格式容量时在发表前拒绝。
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

/// 只更新当前 inode 的大小、sector 计数、块根、修改时间及 CRC，保留未知字段和邻记录。
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

/// 零化校验字段后以 inode 固定种子计算完整记录的 CRC。
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

/// 最终回收只清当前 inode 的内容引用和分配计数，保留 generation、未知字段和邻记录。
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

/// 外部 EA 的 CRC 使用卷种子和物理块号，校验字段按零参与整块计算。
fn xattr_block_checksum(g: &Geometry, physical: u64, bytes: &[u8]) -> u32 {
    let seed = crc32c(g.checksum_seed, &physical.to_le_bytes());
    let prefix = crc32c(seed, &bytes[..16]);
    crc32c(crc32c(prefix, &[0; 4]), &bytes[20..g.block_size])
}

/// 删除外部 EA 前校验真实头和物理块 CRC，返回其中隐藏 value inode 的准确编号。
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

/// inode 内 EA 的隐藏 value 逐项返回，普通内嵌值随本记录清除而不另占块。
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

/// entry 跨度由名字长度决定；固定数组覆盖 4KiB 块内全部最小 16 字节条目。
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

/// 准备阶段已验证 entry 跨度；最终清除隐藏引用和值长度，其他 EA 字节保持原样。
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

/// 隐藏 EA inode 的引用计数按 ctime 高半与 i_version 低半解码，单位为 EA 引用。
pub(crate) fn ea_inode_references(record: &[u8]) -> Result<u64, FsError> {
    Ok((u64::from(le32(record, 12)?) << 32) | u64::from(le32(record, 36)?))
}

/// canonical 写序列下仅减真实 EA 引用；最后引用清 nlink，存储仍由正常末使用回收。
/// 准备已验证记录和计数，当前值仍由同卷元数据 WB 序列保护；不读盘或申请容量。
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

/// 共享 EA 只修改当前引用计数并重算物理块 CRC，不复制其余名字和值。
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
