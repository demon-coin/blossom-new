//! 同源 codec 边界与真实镜像上的读取、目录、链接和错误分类。

use crate::fs::ext4::{codec as ext4_codec, BlockMapping};
use crate::fs::ktest::*;
use crate::fs::{InodeMetadata, InodeTime};

/// 小输入只构造格式字段，解码、CRC、特性判定全部使用生产函数。
fn super_bytes() -> [u8; 1024] {
    let mut bytes = [0u8; 1024];
    for (offset, value) in [
        (0, 64u32),
        (4, 1024),
        (20, 1),
        (32, 8192),
        (36, 8192),
        (40, 64),
        (76, 1),
        (84, 11),
        (96, 2),
    ] {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    bytes[56..58].copy_from_slice(&0xef53u16.to_le_bytes());
    bytes[88..90].copy_from_slice(&128u16.to_le_bytes());
    bytes
}

/// 只向显式 scratch 的有界页片段写入格式输入，真实 Bio 完成后才释放原页。
fn write_view_fixture(disk: &DiskRef, byte: u64, bytes: &[u8]) -> Result<(), FsError> {
    use crate::block::{Bio, BioAllocator, BlkOperation};
    use crate::mm::dma::MappedPages;
    use virtio_drivers::BufferDirection;

    let pages = mm::alloc_pages(0, true).ok_or(FsError::NoMemory)?;
    let mut mapped = MappedPages::new(pages, 0, bytes.len(), BufferDirection::DriverToDevice)
        .map_err(|_| FsError::NoMemory)?;
    for (destination, byte) in mapped.bytes_mut().iter_mut().zip(bytes) {
        destination.write(*byte);
    }
    let mut memory = Vec::new_in(BioAllocator::Heap);
    memory.try_reserve_exact(1).map_err(|_| FsError::NoMemory)?;
    memory.push(mapped);
    // SAFETY: 独占页的完整映射范围已初始化，无裸别名或回调；memory 移入唯一 Bio，
    // completion 全尾前没有 CPU 借用，真实设备收权后才领取并释放原页。
    let bio = unsafe {
        Bio::try_new(
            disk,
            BlkOperation::Write,
            byte / crate::block::SECTOR_BYTES,
            memory,
            0,
            None,
        )
    }?;
    let completion = bio.completion();
    disk.submit_bio(bio);
    let result = completion.wait();
    drop(completion.take_memory()?);
    if let Some(error) = result.error {
        return Err(error.into());
    }
    if result.completed != bytes.len() {
        return Err(BlockError::ShortIo.into());
    }
    Ok(())
}

/// 建立无日志小卷的主元数据及单块普通文件，格式与文件内容均由生产路径读取。
fn format_view_fixture(view: &BlockView, log_block: u32) -> Result<(), FsError> {
    let size = 1024usize << log_block;
    let start = view.absolute_byte(0, 64 * size)?;
    let mut page = [0u8; mm::PAGE_SIZE];
    let mut superblock = super_bytes();
    for (offset, value) in [
        (4, 64u32),
        (20, u32::from(log_block == 0)),
        (24, log_block),
        (28, log_block),
        (32, 64),
        (36, 64),
    ] {
        superblock[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    page[0] = 0x52;
    page[1024..2048].copy_from_slice(&superblock);
    let descriptor = (u64::from(log_block == 0) + 1) * size as u64;
    let mut group = [0u8; 32];
    for (offset, pointer) in [(0, 5u32), (4, 6), (8, 7)] {
        group[offset..offset + 4].copy_from_slice(&pointer.to_le_bytes());
    }
    if descriptor < mm::PAGE_SIZE as u64 {
        page[descriptor as usize..descriptor as usize + group.len()].copy_from_slice(&group);
    }
    write_view_fixture(view.disk(), start, &page)?;
    if mm::PAGE_SIZE as u64 <= descriptor {
        page.fill(0);
        page[..group.len()].copy_from_slice(&group);
        write_view_fixture(view.disk(), start + descriptor, &page)?;
    }
    page.fill(0);
    let inode = 11 * 128;
    page[inode..inode + 2].copy_from_slice(&0x81a4u16.to_le_bytes());
    page[inode + 4..inode + 8].copy_from_slice(&(size as u32).to_le_bytes());
    page[inode + 26..inode + 28].copy_from_slice(&1u16.to_le_bytes());
    page[inode + 28..inode + 32].copy_from_slice(&(size as u32 / 512).to_le_bytes());
    page[inode + 40..inode + 44].copy_from_slice(&32u32.to_le_bytes());
    write_view_fixture(view.disk(), start + 7 * size as u64, &page)?;
    page.fill(0x61 + log_block as u8);
    write_view_fixture(view.disk(), start + 32 * size as u64, &page[..size])?;
    Ok(())
}

/// 只验证外部组件名，不把字节名转成 UTF-8 或造路径层。
fn indexed_name(index: usize) -> [u8; 191] {
    let mut name = [b'n'; 191];
    name[..5].copy_from_slice(b"item-");
    let mut digits = index;
    for place in (5..10).rev() {
        name[place] = b'0' + (digits % 10) as u8;
        digits /= 10;
    }
    name[10] = b'-';
    name
}

/// 被拒挂载仍由 typed fixture 保持原 Disk，原错误不落入假零或 NotFound。
fn open_error(context: &mut TestContext<'_>, fixture: &Owner, expected: FsError) -> TestResult {
    let result = Ext4Volume::open(
        BlockView::whole(fixture.disk()?.clone()),
        AccessMode::ReadOnly,
        64,
    );
    context.expect_eq(result.err(), Some(expected));
    context.diagnostic(format_args!(
        "ext4-reject error={:?} hardware-EIO-covered=0",
        expected
    ));
    Ok(())
}

#[ktest::suite(context = "Thread", tags("ext4-codec"), timeout_ms = 15000)]
pub(crate) mod codec {
    use super::*;
    /// initialized/unwritten 分界与零长度都消费同源解码，不复制 ee_len 判定。
    #[test]
    #[ktest(tags("B13"))]
    fn extent_length_boundary(context: &mut TestContext<'_>) -> TestResult {
        for (raw, expected) in [
            (1, (1, false)),
            (32767, (32767, false)),
            (32768, (32768, false)),
            (32769, (1, true)),
            (65535, (32767, true)),
        ] {
            context.expect_eq(ext4_codec::extent_length(raw), Ok(expected));
        }
        context.expect_eq(ext4_codec::extent_length(0), Err(FsError::Corrupt));
        Ok(())
    }
    /// 短头、entries 越界及下降深度错误必须拒绝，不能作为 hole。
    #[test]
    #[ktest(tags("B13"))]
    fn extent_header_bounds(context: &mut TestContext<'_>) -> TestResult {
        let mut bytes = [0u8; 60];
        bytes[0..2].copy_from_slice(&0xf30au16.to_le_bytes());
        bytes[2..4].copy_from_slice(&1u16.to_le_bytes());
        bytes[4..6].copy_from_slice(&4u16.to_le_bytes());
        let header = ext4_codec::extent_header(&bytes, Some(0))
            .map_err(|_| TestError("valid extent header rejected"))?;
        context.expect_eq((header.entries, header.max, header.depth), (1, 4, 0));
        context.expect_eq(
            ext4_codec::extent_header(&bytes[..59], None).err(),
            Some(FsError::Corrupt),
        );
        context.expect_eq(
            ext4_codec::extent_header(&bytes, Some(1)).err(),
            Some(FsError::Corrupt),
        );
        bytes[2..4].copy_from_slice(&5u16.to_le_bytes());
        context.expect_eq(
            ext4_codec::extent_header(&bytes, None).err(),
            Some(FsError::Corrupt),
        );
        Ok(())
    }
    /// 标准 CRC 向量覆盖算法，seed 与分段只由生产实现执行。
    #[test]
    #[ktest(tags("B12"))]
    fn crc_vectors(context: &mut TestContext<'_>) -> TestResult {
        context.expect_eq(ext4_codec::crc32c(!0, b"123456789"), 0x1cf96d7c);
        context.expect_eq(ext4_codec::crc16(0, b"123456789"), 0xbb3d);
        context.expect_eq(
            ext4_codec::crc32c(ext4_codec::crc32c(0x13572468, b"1234"), b"56789"),
            ext4_codec::crc32c(0x13572468, b"123456789"),
        );
        Ok(())
    }
    /// 三类特性遵守只读类别语义，忽略日志提示位后仍校验真实几何和已拒格式。
    #[test]
    #[ktest(tags("B12"))]
    fn features_layout_and_recovery(context: &mut TestContext<'_>) -> TestResult {
        let original = super_bytes();
        for (offset, value) in [
            (92, 0x80000000u32),
            (100, 0x80000000),
            (96, 6),
            (100, 0x10000),
            (232, 1),
        ] {
            let mut bytes = original;
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            let geometry = checked(context, ext4_codec::decode_super(&bytes, 1024 * 1024))?;
            context.expect_eq(
                (
                    geometry.block_size,
                    geometry.inode_size,
                    geometry.descriptor_size,
                ),
                (1024, 128, 32),
            );
        }
        for (offset, value, expected) in [
            (96, 0x80000002u32, FsError::Unsupported),
            (96, 0x12, FsError::Unsupported),
            (96, 0x8002, FsError::Unsupported),
            (100, 4, FsError::Unsupported),
            (100, 0x200, FsError::Unsupported),
            (100, 0x8000, FsError::Unsupported),
            (96, 0x80000006, FsError::Unsupported),
            (20, 0, FsError::Corrupt),
            (40, 0, FsError::Corrupt),
        ] {
            let mut bytes = original;
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            context.expect_eq(
                ext4_codec::decode_super(&bytes, 1024 * 1024).err(),
                Some(expected),
            );
        }
        context.expect_eq(
            ext4_codec::decode_super(&original[..1023], 1024 * 1024).err(),
            Some(FsError::Corrupt),
        );
        context.expect_eq(
            ext4_codec::decode_super(&original, 512).err(),
            Some(FsError::Corrupt),
        );
        Ok(())
    }
    /// root 与 index 的尾保留字节可以非零，修改它们仍必须通过完整 CRC 校验。
    #[test]
    #[ktest(tags("B12"))]
    fn dx_tail_reserved_bytes_keep_crc_contract(context: &mut TestContext<'_>) -> TestResult {
        let mut geometry = checked(
            context,
            ext4_codec::decode_super(&super_bytes(), 1024 * 1024),
        )?;
        geometry.features_compat |= 0x20;
        geometry.features_ro_compat |= ext4_codec::METADATA_CSUM;
        let time = InodeTime {
            seconds: 0,
            nanoseconds: 0,
        };
        let inode = InodeMetadata {
            number: 2,
            kind: InodeKind::Directory,
            mode: 0o40755,
            size: 3072,
            uid: 0,
            gid: 0,
            links: 2,
            blocks: 6,
            rdev: 0,
            atime: time,
            mtime: time,
            ctime: time,
            generation: 0,
            flags: ext4_codec::INDEX,
            checksum_seed: 0x1357_2468,
            fast_symlink: false,
            file_acl: 0,
            extra_isize: 0,
        };
        for (logical, expected) in [
            (0, ext4_codec::DirectoryLayout::Root),
            (2, ext4_codec::DirectoryLayout::Index),
        ] {
            let mut bytes = [0u8; 1024];
            let offset = if logical == 0 {
                ext4_codec::put32(&mut bytes, 0, 2);
                ext4_codec::put16(&mut bytes, 4, 12);
                bytes[6..9].copy_from_slice(&[1, 2, b'.']);
                ext4_codec::put32(&mut bytes, 12, 1);
                ext4_codec::put16(&mut bytes, 16, 1012);
                bytes[18..22].copy_from_slice(&[2, 2, b'.', b'.']);
                bytes[28..30].copy_from_slice(&[1, 8]);
                32
            } else {
                ext4_codec::put16(&mut bytes, 4, 1024);
                8
            };
            let tail = bytes.len() - 8;
            ext4_codec::put16(&mut bytes, offset, ((tail - offset) / 8) as u16);
            ext4_codec::put16(&mut bytes, offset + 2, 1);
            ext4_codec::put32(&mut bytes, offset + 4, 1);
            ext4_codec::put32(&mut bytes, tail, 0x89ab_cdef);
            let checksum = checked(
                context,
                ext4_codec::dx_checksum(inode.checksum_seed, &bytes, offset + 8, tail),
            )?;
            ext4_codec::put32(&mut bytes, tail + 4, checksum);
            context.expect_eq(
                ext4_codec::verify_directory(&geometry, &inode, logical, &bytes, logical != 0),
                Ok(expected),
            );
            bytes[tail] ^= 1;
            context.expect_eq(
                ext4_codec::verify_directory(&geometry, &inode, logical, &bytes, logical != 0),
                Err(FsError::Checksum),
            );
        }
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4", "ext4-valid"), resources("root"), timeout_ms = 90000)]
pub(crate) mod volume {
    use super::*;
    /// 实盘几何包括三种块大小、inode/descriptor 尺寸与 CRC seed，并核验文件内容和 EOF。
    #[test]
    #[ktest(tags("B12", "B22"))]
    fn geometry_and_pattern_eof(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        let volume = fixture.open(context)?;
        let g = volume.geometry();
        context.expect_eq(
            matches!(
                (g.block_size, g.inode_size, g.descriptor_size),
                (1024, 128, 32) | (2048, 256, 32) | (4096, 256, 64)
            ),
            true,
        );
        context.expect_eq(g.features_incompat & 4, 0);
        if g.block_size == 1024 {
            context.expect_eq(g.features_ro_compat & 0x10, 0x10);
            context.expect_eq(g.features_incompat & 0x40, 0);
        } else {
            context.expect_eq(g.features_ro_compat & 0x400, 0x400);
            context.expect_eq(g.features_incompat & 0x2000 != 0, g.block_size == 4096);
        }
        let file = fixture.file(context, &volume, b"pattern")?;
        let metadata = checked(context, file.metadata_snapshot())?;
        context.expect_eq(metadata.kind, InodeKind::Regular);
        context.expect_eq(metadata.size, PATTERN_SIZE);
        context.expect_eq(metadata.links, 2);
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        for offset in [0, 4093, 12288, PATTERN_SIZE - 37] {
            let expected = (PATTERN_SIZE - offset).min(mm::PAGE_SIZE as u64) as usize;
            context.expect_eq(
                checked(context, file.read_at(offset, &mut bytes))?,
                expected,
            );
            context.expect_eq(patterned(&bytes[..expected], offset), true);
        }
        let last = PATTERN_SIZE / mm::PAGE_SIZE as u64;
        let folio = file
            .mapping()
            .cached_folio(last)
            .ok_or(TestError("EOF folio missing"))?;
        checked(
            context,
            folio
                .copy_to(37, &mut bytes[..mm::PAGE_SIZE - 37])
                .map_err(FsError::from),
        )?;
        context.expect_eq(
            bytes[..mm::PAGE_SIZE - 37].iter().all(|byte| *byte == 0),
            true,
        );
        let before = file.mapping().len();
        context.expect_eq(checked(context, file.read_at(PATTERN_SIZE, &mut bytes))?, 0);
        context.expect_eq(checked(context, file.read_at(u64::MAX, &mut bytes))?, 0);
        context.expect_eq(checked(context, file.read_at(0, &mut []))?, 0);
        context.expect_eq(file.mapping().len(), before);
        let empty = fixture.file(context, &volume, b"empty")?;
        context.expect_eq(checked(context, empty.read_at(0, &mut bytes))?, 0);
        context.expect_eq(empty.mapping().len(), 0);
        context.diagnostic(format_args!("ext4-geometry block-size={} inode-size={} descriptor-size={} compat={:#x} incompat={:#x} ro-compat={:#x} EOF-tail-zero=1", g.block_size, g.inode_size, g.descriptor_size, g.features_compat, g.features_incompat, g.features_ro_compat));
        Ok(())
    }
    /// 非连续实盘映射、合法 hole 与 unwritten 都以原文件页完成真实内容。
    #[test]
    #[ktest(tags("B13"))]
    fn sparse_unwritten_and_deep_mapping(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let bs = volume.geometry().block_size as u64;
        let sparse = fixture.file(context, &volume, b"sparse")?;
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        for offset in [4096, 8192] {
            context.expect_eq(
                checked(context, sparse.read_at(offset, &mut bytes))?,
                mm::PAGE_SIZE,
            );
            context.expect_eq(bytes.iter().all(|byte| *byte == 0), true);
        }
        context.expect_eq(checked(context, sparse.read_at(12288, &mut bytes))?, 37);
        context.expect_eq(patterned(&bytes[..37], 12288), true);
        context.expect_eq(matches!(checked(context, sparse.map_block(4096 / bs))?, BlockMapping::Hole { len } if 0 < len), true);
        let unwritten = fixture.file(context, &volume, b"unwritten")?;
        let mapped = checked(context, unwritten.map_block(0))?;
        context.expect_eq(
            if volume.geometry().features_incompat & 0x40 != 0 {
                matches!(mapped, BlockMapping::Unwritten { len, .. } if 0 < len)
            } else {
                matches!(mapped, BlockMapping::Hole { len } if 0 < len)
            },
            true,
        );
        context.expect_eq(
            checked(context, unwritten.read_at(0, &mut bytes))?,
            mm::PAGE_SIZE,
        );
        context.expect_eq(bytes.iter().all(|byte| *byte == 0), true);
        let deep = fixture.file(context, &volume, b"deep")?;
        if volume.geometry().features_incompat & 0x40 != 0 {
            let header = checked(
                context,
                deep.with_map_read(|_, block| ext4_codec::extent_header(block, None)),
            )?;
            context.expect_eq(2 <= header.depth, true);
        }
        for logical in [0, 2, 998, (DEEP_EXTENTS - 1) * 2] {
            let offset = logical * bs;
            context.expect_eq(
                checked(context, deep.read_at(offset, &mut bytes[..bs as usize]))?,
                bs as usize,
            );
            context.expect_eq(patterned(&bytes[..bs as usize], offset), true);
            context.expect_eq(matches!(checked(context, deep.map_block(logical))?, BlockMapping::Mapped { len, .. } if 0 < len), true);
            if logical < (DEEP_EXTENTS - 1) * 2 {
                context.expect_eq(
                    checked(
                        context,
                        deep.read_at(offset + bs, &mut bytes[..bs as usize]),
                    )?,
                    bs as usize,
                );
                context.expect_eq(bytes[..bs as usize].iter().all(|byte| *byte == 0), true);
            }
        }
        if bs == 1024 {
            let traditional = fixture.file(context, &volume, b"indirect")?;
            for logical in [0, 12, 268, 65804] {
                context.expect_eq(
                    checked(
                        context,
                        traditional.read_at(logical * bs, &mut bytes[..1024]),
                    )?,
                    1024,
                );
                context.expect_eq(patterned(&bytes[..1024], logical * bs), true);
                context.expect_eq(matches!(checked(context, traditional.map_block(logical))?, BlockMapping::Mapped { len, .. } if 0 < len), true);
            }
        }
        context.expect_eq(
            checked(context, deep.map_block(DEEP_EXTENTS * 2))?,
            BlockMapping::Eof,
        );
        Ok(())
    }
    /// 原始字节名、硬链接、fast 与块 symlink 直接消费 canonical inode。
    #[test]
    #[ktest(tags("B16", "B19"))]
    fn links_names_and_lookup_errors(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let root = checked(context, fixture.cache().iget(&volume, 2))?;
        let first = fixture.file(context, &volume, b"pattern")?;
        let hard = fixture.file(context, &volume, b"hardlink")?;
        context.expect_eq(first.ptr_eq(&hard), true);
        context.expect_eq(core::ptr::eq(first.mapping(), hard.mapping()), true);
        let fast = fixture.file(context, &volume, b"fastlink")?;
        context.expect_eq(
            checked(context, fast.metadata_snapshot())?.fast_symlink,
            true,
        );
        context.expect_eq(checked(context, fast.read_link())?.as_slice(), b"pattern");
        let long = fixture.file(context, &volume, b"blocklink")?;
        context.expect_eq(
            checked(context, long.metadata_snapshot())?.fast_symlink,
            false,
        );
        context.expect_eq(checked(context, long.read_link())?.as_slice(), &[b't'; 180]);
        let raw = fixture.file(context, &volume, b"name-\xff\xfe")?;
        let mut bytes = [0xa7; 9];
        context.expect_eq(checked(context, raw.read_at(0, &mut bytes))?, 9);
        context.expect_eq(bytes, *b"raw-name\n");
        let mut cursor = checked(context, root.directory_cursor(0))?;
        let mut raw_entries = 0;
        while let Some(entry) = checked(context, root.dir_next(&cursor))? {
            cursor = entry.next_cursor;
            if entry.name == b"name-\xff\xfe" {
                context.expect_eq(entry.number, raw.number());
                context.expect_eq(entry.kind, Some(InodeKind::Regular));
                raw_entries += 1;
            }
        }
        context.expect_eq(raw_entries, 1);
        context.expect_eq(
            root.lookup(fixture.cache(), b"absent").err(),
            Some(FsError::NotFound),
        );
        for name in [b"".as_slice(), b"a/b", b"a\0b"] {
            context.expect_eq(
                root.lookup(fixture.cache(), name).err(),
                Some(FsError::InvalidName),
            );
        }
        context.expect_eq(
            first.lookup(fixture.cache(), b"x").err(),
            Some(FsError::NotDirectory),
        );
        context.expect_eq(first.read_link().err(), Some(FsError::InvalidOperation));
        for number in [0, u64::MAX] {
            context.expect_eq(
                volume.load_inode(number, None).err(),
                Some(FsError::Corrupt),
            );
            context.expect_eq(
                fixture.cache().iget(&volume, number).err(),
                Some(FsError::Corrupt),
            );
        }
        Ok(())
    }
    /// 跨块与真实 dx 多层目录枚举不得返回 unused、tail 或 index 伪项。
    #[test]
    #[ktest(tags("B19"))]
    fn indexed_directory_bytes_and_cursor(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let directory = fixture.file(context, &volume, b"indexed")?;
        context.expect_eq(
            checked(context, directory.metadata_snapshot())?.flags & 0x1000,
            0x1000,
        );
        let mut seen = [false; DIRECTORY_ENTRIES];
        let mut dots = 0;
        let mut cursor = checked(context, directory.directory_cursor(0))?;
        while let Some(entry) = checked(context, directory.dir_next(&cursor))? {
            cursor = entry.next_cursor;
            if entry.name == b"." || entry.name == b".." {
                dots += 1;
                continue;
            }
            if entry.name.len() != 191
                || &entry.name[..5] != b"item-"
                || entry.name[10] != b'-'
                || !entry.name[11..].iter().all(|byte| *byte == b'n')
            {
                return Err(TestError("directory returned malformed or pseudo name"));
            }
            let mut index = 0usize;
            for digit in &entry.name[5..10] {
                if !digit.is_ascii_digit() {
                    return Err(TestError("directory name digit invalid"));
                }
                index = index * 10 + usize::from(*digit - b'0');
            }
            if DIRECTORY_ENTRIES <= index || seen[index] {
                return Err(TestError("directory duplicate or unexpected entry"));
            }
            seen[index] = true;
            context.expect_eq(entry.kind, Some(InodeKind::Regular));
        }
        context.expect_eq(dots, 2);
        context.expect_eq(seen.iter().all(|present| *present), true);
        for index in [0, 5999, DIRECTORY_ENTRIES - 1] {
            let item = checked(
                context,
                directory.lookup(fixture.cache(), &indexed_name(index)),
            )?;
            context.expect_eq(
                checked(context, item.metadata_snapshot())?.kind,
                InodeKind::Regular,
            );
        }
        context.expect_eq(
            directory.lookup(fixture.cache(), b"absent").err(),
            Some(FsError::NotFound),
        );
        let plain = fixture.file(context, &volume, b"plain")?;
        let mut count = 0;
        let mut cursor = checked(context, plain.directory_cursor(0))?;
        while let Some(entry) = checked(context, plain.dir_next(&cursor))? {
            cursor = entry.next_cursor;
            count += 1;
        }
        context.expect_eq(count, 42);
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-identity"), resources("root", "extra"), timeout_ms = 30000)]
pub(crate) mod identity {
    use super::*;
    /// 同卷硬链接同 inode/mapping，两个真实卷的同号 inode 必须各自拥有身份。
    #[test]
    #[ktest(tags("B16"))]
    fn canonical_hardlink_and_cross_volume(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let first_volume = fixture.open(context)?;
        let second_volume = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(fixture.disks[1].clone()),
                AccessMode::ReadOnly,
                64,
            ),
        )?;
        let first = fixture.file(context, &first_volume, b"pattern")?;
        let hard = fixture.file(context, &first_volume, b"hardlink")?;
        let second = checked(
            context,
            fixture.cache().iget(&second_volume, first.number()),
        )?;
        context.expect_eq(first.ptr_eq(&hard), true);
        context.expect_eq(core::ptr::eq(first.mapping(), hard.mapping()), true);
        context.expect_eq(first.number(), second.number());
        context.expect_eq(first.ptr_eq(&second), false);
        context.expect_eq(core::ptr::eq(first.mapping(), second.mapping()), false);
        context.expect_eq(
            Arc::ptr_eq(
                checked(context, first.disk_volume())?,
                checked(context, second.disk_volume())?,
            ),
            false,
        );
        Ok(())
    }

    /// 同盘相邻非页对齐卷共用物理预算，独立 canonical、脏范围、扫描、授权和 WB 错误。
    #[test]
    #[ktest(
        tags("block-view", "fs-sync"),
        resources("scratch"),
        timeout_ms = 60000
    )]
    fn adjacent_views_keep_one_physical_cache_and_separate_volume_state(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        fixture.disk()?;
        let disk = fixture
            .disks
            .get(2)
            .ok_or(TestError("view boundary scratch disk missing"))?
            .clone();
        if disk.limits().logical_block_size != 512 || disk.limits().readonly {
            return Err(TestError(
                "view boundary fixture requires writable 512-byte scratch",
            ));
        }
        let first_view = checked(
            context,
            BlockView::new(disk.clone(), 1, 128).map_err(FsError::from),
        )?;
        let second_view = checked(
            context,
            BlockView::new(disk.clone(), 129, 512).map_err(FsError::from),
        )?;
        checked(context, format_view_fixture(&first_view, 0))?;
        checked(context, format_view_fixture(&second_view, 2))?;
        let last_byte = checked(
            context,
            first_view
                .absolute_byte(63 * 1024, 1024)
                .map_err(FsError::from),
        )?;
        checked(context, write_view_fixture(&disk, last_byte, &[0x31; 1024]))?;
        let first = checked(
            context,
            Ext4Volume::open(first_view.clone(), AccessMode::ReadWrite, 64),
        )?;
        let second = checked(
            context,
            Ext4Volume::open(second_view.clone(), AccessMode::ReadWrite, 64),
        )?;
        let first_inode = checked(context, fixture.cache().iget(&first, 12))?;
        let second_inode = checked(context, fixture.cache().iget(&second, 12))?;
        first.writeback_domain().stop_sync();
        second.writeback_domain().stop_sync();
        let result = (|| {
            let reopened = checked(
                context,
                Ext4Volume::open(first_view.clone(), AccessMode::ReadWrite, 64),
            )?;
            context.expect_eq(Arc::ptr_eq(&first, &reopened), true);
            context.expect_eq(first.device() != second.device(), true);
            context.expect_eq(first_inode.ptr_eq(&second_inode), false);
            context.expect_eq(
                (first.geometry().block_size, second.geometry().block_size),
                (1024, 4096),
            );
            let mut data = [0u8; mm::PAGE_SIZE];
            context.expect_eq(
                checked(context, first_inode.read_at(0, &mut data[..1024]))?,
                1024,
            );
            context.expect_eq(data[..1024].iter().all(|byte| *byte == 0x61), true);
            context.expect_eq(checked(context, second_inode.read_at(0, &mut data))?, 4096);
            context.expect_eq(data.iter().all(|byte| *byte == 0x63), true);
            let first_cache = first.metadata_cache();
            let second_cache = second.metadata_cache();
            context.expect_eq(Arc::ptr_eq(first_cache, second_cache), false);
            context.expect_eq(
                Arc::ptr_eq(
                    &first_cache.reserve().unwrap(),
                    &second_cache.reserve().unwrap(),
                ),
                true,
            );
            context.expect_eq(
                crate::cache::MetadataCache::shared(second.claim(), 63).err(),
                Some(BlockError::Busy),
            );
            let first_zero = checked(
                context,
                first_cache.get(0, 4096, None).map_err(FsError::from),
            )?;
            let second_zero = checked(
                context,
                second_cache.get(0, 4096, None).map_err(FsError::from),
            )?;
            context.expect_eq(
                first_zero.folio().phys_addr() != second_zero.folio().phys_addr(),
                true,
            );
            let last = checked(
                context,
                first_cache.get(63, 1024, None).map_err(FsError::from),
            )?;
            context.expect_eq(
                checked(
                    context,
                    last.with_read(|bytes| bytes.iter().all(|byte| *byte == 0x31))
                        .map_err(FsError::from),
                )?,
                true,
            );
            context.expect_eq(
                last.update(second.claim(), |_| ()).err(),
                Some(BlockError::InvalidRange),
            );
            context.expect_eq(
                first.require_write(second.claim()),
                Err(FsError::InvalidOperation),
            );
            context.expect_eq(
                first_inode.update_metadata(second.claim(), |_| ()).err(),
                Some(FsError::InvalidOperation),
            );
            checked(
                context,
                last.update(first.claim(), |bytes| bytes.fill(0xa1))
                    .map_err(FsError::from),
            )?;
            checked(
                context,
                second_zero
                    .update(second.claim(), |bytes| bytes[0] = 0xa2)
                    .map_err(FsError::from),
            )?;
            let boundary = 65536;
            context.expect_eq(first_cache.page_mask(boundary), 0x01);
            context.expect_eq(second_cache.page_mask(boundary), 0xfe);
            let mut from_first = [None];
            let mut from_second = [None];
            first_cache.select_range(
                boundary,
                boundary + 1,
                crate::cache::FolioMark::Dirty,
                &mut from_first,
            );
            second_cache.select_range(
                boundary,
                boundary + 1,
                crate::cache::FolioMark::Dirty,
                &mut from_second,
            );
            context.expect_eq(
                from_first[0].as_ref().unwrap().1.phys_addr(),
                from_second[0].as_ref().unwrap().1.phys_addr(),
            );
            first_cache.mark_to_write_range(0, u64::MAX);
            second_cache.mark_to_write_range(0, u64::MAX);
            first_cache.set_to_write(boundary, false);
            from_first[0] = None;
            from_second[0] = None;
            first_cache.select_range(
                boundary,
                boundary + 1,
                crate::cache::FolioMark::ToWrite,
                &mut from_first,
            );
            second_cache.select_range(
                boundary,
                boundary + 1,
                crate::cache::FolioMark::ToWrite,
                &mut from_second,
            );
            context.expect_eq(from_first[0].is_none(), true);
            context.expect_eq(from_second[0].is_some(), true);
            let first_domain = first.writeback_domain();
            let second_domain = second.writeback_domain();
            let first_context =
                first_domain.metadata_io_context(first_domain.snapshot().stop_sequence);
            let second_context =
                second_domain.metadata_io_context(second_domain.snapshot().stop_sequence);
            first_domain.stop_sync();
            context.expect_eq(first_context.check(), Err(BlockError::NotReady));
            context.expect_eq(second_context.check(), Ok(()));
            let mut first_error = first_cache.sample_wb_error();
            let mut second_error = second_cache.sample_wb_error();
            disk.test_fail_next_metadata(BlockError::Io);
            context.expect_eq(
                crate::fs::writeback::sync_metadata(&first),
                Err(FsError::Io(BlockError::Io)),
            );
            context.expect_eq(disk.test_completion_error_pending(), false);
            context.expect_eq(
                first_cache.check_and_advance_wb_error(&mut first_error),
                Err(FsError::Io(BlockError::Io)),
            );
            context.expect_eq(
                second_cache.check_and_advance_wb_error(&mut second_error),
                Ok(()),
            );
            context.expect_eq(first_cache.has_dirty_or_writeback(), false);
            context.expect_eq(second_cache.has_dirty_or_writeback(), true);
            context.expect_eq(
                checked(
                    context,
                    second_zero
                        .with_read(|bytes| bytes[0])
                        .map_err(FsError::from),
                )?,
                0xa2,
            );
            context.expect_eq(
                checked(
                    context,
                    last.with_read(|bytes| bytes.iter().all(|byte| *byte == 0xa1))
                        .map_err(FsError::from),
                )?,
                true,
            );
            Ok(())
        })();
        let first_sync = checked(context, crate::fs::writeback::sync_metadata(&first));
        let second_sync = checked(context, crate::fs::writeback::sync_metadata(&second));
        first.writeback_domain().activate();
        second.writeback_domain().activate();
        result.and(first_sync).and(second_sync)
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bounded"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bounded {
    use super::*;
    /// inode 全活预算明确 NoMemory；稀疏文件页可增长，clean 回收保留外部拥有与 canonical 身份。
    #[test]
    #[ktest(tags("B23"))]
    fn inode_and_file_budget_clean_prune(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let cache = InodeCache::try_new(2).map_err(|_| TestError("bounded inode cache failed"))?;
        let root = checked(context, cache.iget(&volume, 2))?;
        let file = checked(context, root.lookup(&cache, b"pattern"))?;
        let number = file.number();
        context.expect_eq(cache.len(), 2);
        context.expect_eq(root.lookup(&cache, b"empty").err(), Some(FsError::NoMemory));
        context.expect_eq(cache.prune_unused(), 0);
        let hard = checked(context, root.lookup(&cache, b"hardlink"))?;
        context.expect_eq(file.ptr_eq(&hard), true);
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        let mut held = [const { None }; 2];
        for (page, slot) in held.iter_mut().enumerate() {
            context.expect_eq(
                checked(context, file.read_at(page as u64 * 4096, &mut bytes))?,
                4096,
            );
            context.expect_eq(patterned(&bytes, page as u64 * 4096), true);
            let folio = file
                .mapping()
                .cached_folio(page as u64)
                .ok_or(TestError("live bounded folio missing"))?;
            *slot = Some(folio);
        }
        context.expect_eq(checked(context, file.read_at(8192, &mut bytes))?, 4096);
        context.expect_eq(patterned(&bytes, 8192), true);
        context.expect_eq(file.mapping().prune_unused(), 1);
        drop(held[0].take());
        context.expect_eq(file.mapping().prune_unused(), 1);
        context.expect_eq(checked(context, file.read_at(0, &mut bytes))?, 4096);
        context.expect_eq(patterned(&bytes, 0), true);
        drop(held);
        context.expect_eq(file.mapping().prune_unused(), 2);
        drop(root);
        context.expect_eq(cache.prune_unused(), 1);
        let again = checked(context, cache.iget(&volume, file.number()))?;
        context.expect_eq(again.ptr_eq(&file), true);
        context.expect_eq(core::ptr::eq(again.mapping(), file.mapping()), true);
        drop(again);
        drop(file);
        drop(hard);
        context.expect_eq(cache.prune_unused(), 1);
        context.expect_eq(cache.len(), 0);
        let root = checked(context, cache.iget(&volume, 2))?;
        let empty = checked(context, root.lookup(&cache, b"empty"))?;
        context.expect_eq(checked(context, empty.metadata_snapshot())?.size, 0);
        drop(empty);
        drop(root);
        cache.prune_unused();
        let only = checked(context, cache.iget(&volume, number))?;
        context.expect_eq(checked(context, only.read_at(0, &mut bytes))?, 4096);
        let detached = only
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("detached original folio missing"))?
            .with_file_owner(only.clone());
        let inode_address = &*only as *const _ as usize;
        let mapping_address = only.mapping() as *const _ as usize;
        let page_address = detached.phys_addr();
        let loads = cache.ktest_load_count();
        drop(only);
        let held_prune = cache.prune_unused();
        context.expect_eq(held_prune, 0);
        let again = checked(context, cache.iget(&volume, number))?;
        context.expect_eq(&*again as *const _ as usize, inode_address);
        context.expect_eq(again.mapping() as *const _ as usize, mapping_address);
        context.expect_eq(cache.ktest_load_count(), loads);
        let same_page = again
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("detached mapping identity lost"))?;
        context.expect_eq(same_page.phys_addr(), page_address);
        checked(
            context,
            detached.copy_to(0, &mut bytes).map_err(FsError::from),
        )?;
        context.expect_eq(patterned(&bytes, 0), true);
        drop(same_page);
        drop(again);
        context.expect_eq(cache.prune_unused(), 0);
        drop(detached);
        let last_prune = cache.prune_unused();
        context.expect_eq(last_prune, 1);
        context.expect_eq(cache.len(), 0);
        context.diagnostic(format_args!("fs-detached-folio inode-address={} mapping-address={} original-PFN={} all-external-InodeRef-dropped=1 prune-held-folio={} prune-after-last-folio={}", inode_address, mapping_address, page_address, held_prune, last_prune));
        context.expect_eq(0 < volume.prune_unused(), true);
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("fs-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod oom {
    use super::*;
    /// 真实页与堆 NULL 下已有 inode/页仍读，新 miss 原 NoMemory，放压后恢复真实加载。
    #[test]
    #[ktest(tags("B23"))]
    fn live_hits_and_new_misses_under_allocator_oom(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let root = checked(context, fixture.cache().iget(&volume, 2))?;
        let inode = fixture.file(context, &volume, b"pattern")?;
        let empty_number = checked(
            context,
            root.with_map_read(|metadata, block| volume.lookup(metadata, block, b"empty")),
        )?
        .ok_or(TestError("OOM empty directory entry missing"))?;
        let mut bytes = [0xa7; 4096];
        context.expect_eq(checked(context, inode.read_at(0, &mut bytes))?, 4096);
        let folio = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("OOM live page missing"))?;
        let pressure = fixture
            .pressure
            .take()
            .ok_or(TestError("OOM pressure was not prepared"))?;
        fixture.publish(
            0,
            Job::Oom {
                cache: fixture.cache().clone(),
                volume: volume.clone(),
                inode: inode.clone(),
                empty_number,
                pressure,
            },
        );
        fixture.join_tasks()?;
        let result = fixture.observer().results.lock()[0]
            .take()
            .ok_or(TestError("OOM native result absent"))?;
        let returned = checked(context, result)?;
        let Returned::Oom {
            producer_cpu,
            evidence,
            same_inode,
            same_mapping,
            bytes: count,
            valid,
            inode_error,
            page_error,
            address,
            submitted_before,
            submitted_after,
        } = returned
        else {
            return Err(TestError("OOM native producer returned wrong kind"));
        };
        context.expect_eq(producer_cpu, 0);
        context.expect_eq(evidence, (true, true));
        context.expect_eq(same_inode, true);
        context.expect_eq(same_mapping, true);
        context.expect_eq(count, 4096);
        context.expect_eq(valid, true);
        context.expect_eq(inode_error, Some(FsError::NoMemory));
        context.expect_eq(page_error, Some(FsError::NoMemory));
        context.expect_eq(submitted_after, submitted_before);
        context.expect_eq(address, folio.phys_addr());
        checked(context, folio.copy_to(0, &mut bytes).map_err(FsError::from))?;
        context.expect_eq(patterned(&bytes, 0), true);
        let empty = checked(context, fixture.cache().iget(&volume, empty_number))?;
        context.expect_eq(checked(context, empty.metadata_snapshot())?.size, 0);
        context.expect_eq(checked(context, inode.read_at(4096, &mut bytes))?, 4096);
        context.expect_eq(patterned(&bytes, 4096), true);
        let original = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("OOM original page changed"))?;
        context.expect_eq(original.phys_addr(), address);
        context.diagnostic(format_args!("fs-allocator-OOM actual-producer-CPU={} real-page-NULL={} real-heap-NULL={} live-hit-original-PFN={} new-inode-NoMemory=1 new-page-NoMemory=1 released-pressure-recovered=1 hardware-EIO-covered=0", producer_cpu, evidence.0, evidence.1, address));
        Ok(())
    }
}

macro_rules! rejected_volume {
    ($name:ident, $tag:literal, $error:expr) => {
        #[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags($tag), resources("root"), timeout_ms = 30000)]
        pub(crate) mod $name {
            use super::*;
            /// 独立受控镜像的挂载错误必须保持原因，不能映射到硬件 EIO。
            #[test]
            #[ktest(tags("B12"))]
            fn preserves_open_error(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
                open_error(context, fixture, $error)
            }
        }
    };
}
rejected_volume!(bad_super, "ext4-bad-super", FsError::Checksum);
rejected_volume!(bad_geometry, "ext4-bad-geometry", FsError::Unsupported);
rejected_volume!(bad_features, "ext4-bad-features", FsError::Unsupported);

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-group"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_group {
    use super::*;
    /// descriptor CRC 在实际 open 或首次 inode 加载的负责层保持 Checksum。
    #[test]
    #[ktest(tags("B12"))]
    fn group_checksum(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        match Ext4Volume::open(
            BlockView::whole(fixture.disk()?.clone()),
            AccessMode::ReadOnly,
            64,
        ) {
            Err(error) => {
                context.expect_eq(error, FsError::Checksum);
                context.diagnostic(format_args!("ext4-group-checksum rejected-at=open"));
            }
            Ok(volume) => {
                context.expect_eq(volume.load_inode(2, None).err(), Some(FsError::Checksum));
                context.diagnostic(format_args!("ext4-group-checksum rejected-at=inode-load"));
            }
        }
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-inode"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_inode {
    use super::*;
    /// inode 自身 CRC 错误不误报目录 NotFound，失败 iget 不留 Loading 占位。
    #[test]
    #[ktest(tags("B12"))]
    fn inode_checksum_and_retry(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        let volume = fixture.open(context)?;
        let root = checked(context, fixture.cache().iget(&volume, 2))?;
        let number = checked(
            context,
            root.with_map_read(|metadata, block| volume.lookup(metadata, block, b"pattern")),
        )?
        .ok_or(TestError("bad inode name missing"))?;
        let before = fixture.cache().len();
        let loads = fixture.cache().ktest_load_count();
        context.expect_eq(
            fixture.cache().iget(&volume, number).err(),
            Some(FsError::Checksum),
        );
        context.expect_eq(fixture.cache().len(), before);
        context.expect_eq(
            fixture.cache().iget(&volume, number).err(),
            Some(FsError::Checksum),
        );
        context.expect_eq(fixture.cache().ktest_load_count(), loads + 2);
        context.expect_eq(fixture.cache().len(), before);
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-extent"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_extent {
    use super::*;
    /// 外部 extent node 的 CRC 实际失败，读页发布原 Checksum 而非假零。
    #[test]
    #[ktest(tags("B13"))]
    fn external_extent_checksum(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        let volume = fixture.open(context)?;
        let inode = fixture.file(context, &volume, b"deep")?;
        context.expect_eq(inode.map_block(0).err(), Some(FsError::Checksum));
        let mut bytes = [0xa7; 4096];
        context.expect_eq(inode.read_at(0, &mut bytes).err(), Some(FsError::Checksum));
        let folio = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("failed extent folio missing"))?;
        context.expect_eq(folio.wait_ready(), Err(BlockError::Checksum));
        context.expect_eq(folio.range_ready(0, 4096), Err(BlockError::Checksum));
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-dir"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_directory {
    use super::*;
    /// 普通目录 tail CRC 失败传播到 lookup 和 dir_next。
    #[test]
    #[ktest(tags("B19"))]
    fn leaf_checksum(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        let volume = fixture.open(context)?;
        let directory = fixture.file(context, &volume, b"plain")?;
        context.expect_eq(
            directory
                .directory_cursor(0)
                .and_then(|cursor| directory.dir_next(&cursor))
                .err(),
            Some(FsError::Checksum),
        );
        context.expect_eq(
            directory.lookup(fixture.cache(), b"item-0000").err(),
            Some(FsError::Checksum),
        );
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-dx"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_dx {
    use super::*;
    /// dx 根 CRC 与普通 leaf 的覆盖范围不同，索引伪项不能绕过实际校验。
    #[test]
    #[ktest(tags("B19"))]
    fn index_checksum(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        let volume = fixture.open(context)?;
        let directory = fixture.file(context, &volume, b"indexed")?;
        context.expect_eq(
            directory
                .directory_cursor(0)
                .and_then(|cursor| directory.dir_next(&cursor))
                .err(),
            Some(FsError::Checksum),
        );
        context.expect_eq(
            directory.lookup(fixture.cache(), &indexed_name(0)).err(),
            Some(FsError::Checksum),
        );
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-dir-layout"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_directory_layout {
    use super::*;
    /// 无目录 CRC 时零 rec_len 仍须拒绝，不能无限枚举或假 NotFound。
    #[test]
    #[ktest(tags("B19"))]
    fn rejects_zero_record_length(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let directory = fixture.file(context, &volume, b"plain")?;
        context.expect_eq(
            directory
                .directory_cursor(0)
                .and_then(|cursor| directory.dir_next(&cursor))
                .err(),
            Some(FsError::Corrupt),
        );
        context.expect_eq(
            directory.lookup(fixture.cache(), b"item-0000").err(),
            Some(FsError::Corrupt),
        );
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-dx-layout"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_dx_layout {
    use super::*;
    /// 无 CRC 的 dx 空索引计数有独立 Corrupt，不能被跳过当作 unused。
    #[test]
    #[ktest(tags("B19"))]
    fn rejects_empty_index(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        let volume = fixture.open(context)?;
        let directory = fixture.file(context, &volume, b"indexed")?;
        context.expect_eq(
            directory
                .directory_cursor(0)
                .and_then(|cursor| directory.dir_next(&cursor))
                .err(),
            Some(FsError::Corrupt),
        );
        context.expect_eq(
            directory.lookup(fixture.cache(), &indexed_name(0)).err(),
            Some(FsError::Corrupt),
        );
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-bad-mapping"), resources("root"), timeout_ms = 30000)]
pub(crate) mod bad_mapping {
    use super::*;
    /// 坏 root 可在 inode 层提前拒绝；合法 root 下的 IND data 指针由实际映射层拒绝。
    #[test]
    #[ktest(tags("B13"))]
    fn format_mapping_error(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        let volume = fixture.open(context)?;
        let root = checked(context, fixture.cache().iget(&volume, 2))?;
        let number = checked(
            context,
            root.with_map_read(|metadata, block| volume.lookup(metadata, block, b"pattern")),
        )?
        .ok_or(TestError("bad mapping pattern directory entry missing"))?;
        match fixture.cache().iget(&volume, number) {
            Err(error) => {
                context.expect_eq(error, FsError::Corrupt);
                context.diagnostic(format_args!(
                    "ext4-mapping-error rejected-at=inode-load original-error={:?}",
                    error
                ));
            }
            Ok(inode) => {
                let logical = if volume.geometry().block_size == 1024 {
                    12
                } else {
                    0
                };
                let mut bytes = [0xa7; 4096];
                context.expect_eq(inode.map_block(logical).err(), Some(FsError::Corrupt));
                context.expect_eq(
                    inode
                        .read_at(logical * volume.geometry().block_size as u64, &mut bytes)
                        .err(),
                    Some(FsError::Corrupt),
                );
                context.diagnostic(format_args!("ext4-mapping-error rejected-at=map/fill logical-block={} original-error=Corrupt", logical));
            }
        }
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", tags("ext4-prefix"), resources("root"), timeout_ms = 30000)]
pub(crate) mod prefix {
    use super::*;
    /// 合法 direct root 实际交付前三页，下一页在真实 IND 节点解析时失败并返回正前缀。
    #[test]
    #[ktest(tags("B22"))]
    fn successful_prefix_before_corrupt_mapping(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let inode = fixture.file(context, &volume, b"pattern")?;
        context.expect_eq(volume.geometry().block_size, 1024);
        let mut bytes = [0xa7; 16384];
        context.expect_eq(checked(context, inode.read_at(0, &mut bytes))?, 12288);
        context.expect_eq(patterned(&bytes[..12288], 0), true);
        context.expect_eq(bytes[12288..].iter().all(|byte| *byte == 0xa7), true);
        context.expect_eq(
            inode.read_at(12288, &mut bytes[..4096]).err(),
            Some(FsError::Corrupt),
        );
        context.diagnostic(format_args!("ext4-prefix delivered=12288 subsequent-error=Corrupt root-valid=1 indirect-data-pointer-invalid=1 hardware-EIO-covered=0"));
        Ok(())
    }
}
