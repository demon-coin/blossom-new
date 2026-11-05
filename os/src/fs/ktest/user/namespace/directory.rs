//! HTree 增长、目录游标与共享挂载边界的真实消费者。

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

const INDEX: u32 = 0x1000;

const LONG_NAME_PREFIX: &[u8] = b"w27-\xff-";
const PROMOTION_PREFIX_SIGNED: &[u8] = b"signed-";
const PROMOTION_PREFIX_UNSIGNED: &[u8] = b"unsigned-";
const HASH_CANDIDATE_LIMIT: u32 = 1_048_576;
const DIRECTORY_WRITE_BATCH: usize = 8;

#[derive(Clone, Copy)]
struct DxNode {
    root_position: usize,
    logical: u64,
    count: usize,
    limit: usize,
}

#[derive(Clone)]
struct HTreeShape {
    metadata: InodeMetadata,
    map_root: [u8; 60],
    root: [u8; mm::PAGE_SIZE],
    depth: u8,
    root_count: usize,
    root_limit: usize,
    leaves: usize,
    nodes: Vec<DxNode>,
}

#[derive(Clone, Copy)]
struct RawName {
    nonce: u32,
    bytes: [u8; 255],
}

struct LegacyDirectory {
    pathname: &'static [u8],
    path: Path,
    inode: InodeRef,
    names: Vec<Vec<u8>>,
    collision: [&'static [u8]; 2],
}

/// 在原 root hash 区间内取得一个固定原始名字，后续目录修改仍走真实 HTree 路由。
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

/// 在同一叶的有效 hash 区间连续取名；树分裂后重新选择最宽区间，不依赖输入装填率。
fn name_for_split_leaf(
    volume: &Ext4Volume,
    shape: &HTreeShape,
    node: DxNode,
    next_nonce: &mut u32,
) -> Result<RawName, FsError> {
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

/// 从真实 OFD readdir 找到所有新名字；短 callback 拒绝的末项必须按原位置重交付。
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

    let declined = checked(context, file.readdir())?
        .ok_or(TestError("indexed short delivery lost its final entry"))?;
    context.expect_eq(declined.next_cookie, declined_cookie);
    context.expect_eq(declined.number, declined_number);
    context.expect_eq(declined.name.as_slice(), &declined_name[..declined_len]);
    Ok(())
}

/// 解码本测试固定十六进制 nonce，不把 raw 名称转成 UTF-8。
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

/// 原 RootOwner 已持有本例唯一 Disk 与完成域；Superblock 默认版本在首次 FS open 前改为有效 legacy unsigned。
fn select_unsigned_legacy_default(context: &mut TestContext<'_>, disk: &DiskRef) -> TestResult {
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
    // SAFETY: bytes_mut 已初始化整个 Write 范围且可变借用已结束；
    // Bio 接管唯一映射拥有，原 Disk 保活至 wait 确认设备与完成 Work 全尾结束。
    // take_memory 收回原拥有之前没有 CPU 内容别名。
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

/// 校验当前共享元数据页中的 HTree root、节点及可选全部叶 CRC。
fn htree_shape(
    volume: &Ext4Volume,
    inode: &InodeRef,
    verify_leaves: bool,
) -> Result<HTreeShape, FsError> {
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

/// 按后端 dx 前驱选择规则返回 hash 所属的真实索引槽。
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

/// 在名称预算内逐字节构造固定长度原始名称，是否含非 UTF-8 字节由前缀决定。
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

/// 为已发表目录项准备独立字节拥有，不依赖 UTF-8 解释或意外分配。
fn copy_name(name: &[u8]) -> Result<Vec<u8>, TestError> {
    let mut copy = Vec::new();
    copy.try_reserve_exact(name.len())
        .map_err(|_| TestError("directory name copy OOM"))?;
    copy.extend_from_slice(name);
    Ok(copy)
}

/// 把已发表的有限名字落盘并等实际 metadata/FLUSH 完成，保持本例原分配预算。
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

/// 普通目录在首块满时转为 HTree，并用转换前打开且复制的 OFD 验证 cookie 续读。
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

/// 碰撞名尚未插入且所有已有名字的 hash 保持相同时，更新真实 dx 根版本及 checksum。
fn set_root_hash_version(
    context: &TestContext<'_>,
    volume: &Ext4Volume,
    inode: &InodeRef,
    version: u8,
    names: &[Vec<u8>],
) -> TestResult {
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

/// 核对目录字节名、拒绝交付位置与 OFD cookie 推进，读完后保留末尾位置。
fn complete_directory_names(
    context: &mut TestContext<'_>,
    vfs: &Vfs,
    path: &Path,
    names: &[Vec<u8>],
    inode_number: u64,
) -> TestResult {
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

/// 只接受测试记录中的原始字节名并确认它们全部指向同一真实 hardlink inode。
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

/// 按目录 root 的实际版本计算碰撞，核验 dup、独立打开、显式 seek 和续读状态。
fn check_collision_cursors(
    context: &mut TestContext<'_>,
    view: &NamespaceView,
    legacy: &LegacyDirectory,
) -> TestResult {
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

/// 改写后端已连接 metadata 窗口前核主目录索引未清除INDEX或降低真实深度。
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

/// 原 FS 回写与 Work 退役、旧 Weak 失效后同 Disk 建新 cache，读取仍须匹配本轮原内容。
fn cold_reopen_namespace(
    context: &mut TestContext<'_>,
    fixture: &mut WriteOwner,
    sample_path: &[u8],
    expected_inode: u64,
    expected_bytes: &[u8; 64],
) -> TestResult {
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

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-path", "fs-namespace"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 既有 indexed 目录真实扩展并拆叶/索引节点，rename/unlink 后保留完整 raw 名与冷盘读取。
    #[test]
    #[ktest(tags("W27"))]
    fn indexed_extension_splits_leaf_and_index_node(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
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
        // 255 字节新名占 264 字节；每个固定叶区间最多再写一个叶容量就必须分裂。
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

    /// 原卷读写视图共享名字与 canonical inode，alias Mount 保留各自权限和目录边界。
    #[test]
    #[ktest(min_cpus = 2, tags("W26"))]
    fn shared_views_alias_mount_and_directory_boundaries(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let view = prepare_namespace_view(context, fixture)?;
        drop(fixture.open(context, b"/", 0)?);
        drop(fixture.open(context, b"/pattern", 0)?);

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
