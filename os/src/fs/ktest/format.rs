//! ext4 磁盘格式编解码（`codec`）、卷几何、块映射、目录索引、跨卷身份隔离与损坏镜像错误分类测试套件。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统内核测试子系统（`fs::ktest`）中，负责验证 `fs::ext4::codec` 与 `Ext4Volume`
//! 对标准及损坏 ext4 镜像的解析行为。上游由 `fs::ktest` 测试注册表统一挂载，下游调用 `ext4::codec`、
//! `Ext4Volume`、`InodeCache`、`MetadataCache` 以及块设备 `Bio` 接口。
//!
//! # 核心设计与测试卷布局
//! 本模块包含两类测试：
//! 1. **纯内存编解码边界测试（`codec`）**：构造最小超级块、Extent 头部、HTree（dx）目录尾与 CRC 向量，
//!    验证长度高位未初始化标记（`ee_len`）、特性位分类与校验和契约。
//! 2. **块设备卷与损坏镜像集成测试**：在块设备或扇区切片（`BlockView`）上验证稀疏文件、深层 Extent 树、
//!    传统间接块、硬链接/符号链接、HTree 大目录游标、相邻分区缓存隔离、内存受限回收以及各类损坏镜像的错误返回码。
//!
//! 其中 `format_view_fixture` 在 `BlockView` 上构造的最小无日志 ext4 测试卷布局如下：
//!
//! ```text
//! BlockView 偏移 (字节 / 逻辑块):
//! +-----------------------+-----------------------+-----------------------+
//! | 0..1024B              | 1024..2048B           | Group Descriptor      |
//! | 引导扇区填充 (0x52)   | ext4 Superblock       | (块位图=5, Inode位图=6|
//! |                       | (Magic = 0xef53)      |  Inode表起点=块 7)    |
//! +-----------------------+-----------------------+-----------------------+
//! | 块 7: Inode Table                             | 块 32: 数据块         |
//! | Inode #12 (偏移 11*128B): 普通文件, 指向块 32 | 填充 0x61 + log_block |
//! +-----------------------------------------------+-----------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试在独立内核线程中运行，遵循 VFS 与缓存层既有的锁顺序：`Inode` 元数据锁 -> `AddressSpace` / `MetadataCache` 锁 -> 块设备提交队列锁。
//! - 直接提交 `Bio` 写入测试夹具数据（`write_view_fixture`）时，在 DMA 传输完成前不从 CPU 侧访问已移交的 `MappedPages`，待 `completion.wait()` 返回后才收回并释放物理页。

use crate::fs::ext4::{codec as ext4_codec, BlockMapping};
use crate::fs::ktest::*;
use crate::fs::{InodeMetadata, InodeTime};

/// 构造用于测试的 1024 字节最小合法 ext4 超级块字节数组，包含魔数 `0xef53`、1024B 块大小、128B Inode 大小及基础块组几何参数。
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

/// 向块设备 `disk` 的指定字节偏移 `byte` 同步写入测试夹具数据 `bytes`，等待 `Bio` DMA 传输完成后收回并释放物理页。
fn write_view_fixture(disk: &DiskRef, byte: u64, bytes: &[u8]) -> Result<(), FsError> {
    use crate::block::{Bio, BioAllocator, BlkOperation};
    use crate::mm::dma::MappedPages;
    use virtio_drivers::BufferDirection;

    // Step 1: 分配单个物理页并建立 `DriverToDevice` 方向的 DMA 映射缓冲区，拷入待写入的夹具字节。
    let pages = mm::alloc_pages(0, true).ok_or(FsError::NoMemory)?;
    let mut mapped = MappedPages::new(pages, 0, bytes.len(), BufferDirection::DriverToDevice)
        .map_err(|_| FsError::NoMemory)?;
    for (destination, byte) in mapped.bytes_mut().iter_mut().zip(bytes) {
        destination.write(*byte);
    }
    let mut memory = Vec::new_in(BioAllocator::Heap);
    memory.try_reserve_exact(1).map_err(|_| FsError::NoMemory)?;
    memory.push(mapped);
    // Step 2: 构造写方向的 `Bio` 请求并提交到块设备，阻塞等待 I/O 完成再收回 DMA 页面所有权。
    // SAFETY:
    // 1. `mapped` 指向刚由 `mm::alloc_pages(0, true)` 分配的独占物理页，且前 `bytes.len()` 字节已在上方循环中完整初始化。
    // 2. `memory` 容器的所有权完整移交至唯一构造的 `Bio` 实例，且未注册异步完成回调（`None`）。
    // 3. 在下方 `completion.wait()` 阻塞等待设备完成 DMA 传输之前，CPU 侧不保留任何指向该物理页的裸指针或切片借用；待 I/O 完成后才通过 `take_memory()` 收回物理页并释放。
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

/// 在给定的块设备视图 `view` 上格式化一个块大小为 `1024 << log_block` 的最小无日志 ext4 卷，并写入含单块数据的 12 号普通文件 Inode。
fn format_view_fixture(view: &BlockView, log_block: u32) -> Result<(), FsError> {
    // Step 1: 根据 `log_block` 计算块大小，构造超级块与块组描述符并写入视图起始位置。
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

    // Step 2: 在块 7 的 Inode 表中初始化 12 号 Inode（索引 11），设为普通文件（`0x81a4`）并将首个直接块指针指向块 32。
    page.fill(0);
    let inode = 11 * 128;
    page[inode..inode + 2].copy_from_slice(&0x81a4u16.to_le_bytes());
    page[inode + 4..inode + 8].copy_from_slice(&(size as u32).to_le_bytes());
    page[inode + 26..inode + 28].copy_from_slice(&1u16.to_le_bytes());
    page[inode + 28..inode + 32].copy_from_slice(&(size as u32 / 512).to_le_bytes());
    page[inode + 40..inode + 44].copy_from_slice(&32u32.to_le_bytes());
    write_view_fixture(view.disk(), start + 7 * size as u64, &page)?;

    // Step 3: 在块 32 写入一整块重复填充字节（`0x61 + log_block`）作为文件数据。
    page.fill(0x61 + log_block as u8);
    write_view_fixture(view.disk(), start + 32 * size as u64, &page[..size])?;
    Ok(())
}

/// 根据十进制序号 `index` 生成长度为 191 字节的测试目录项名称（格式为 `item-XXXXX-nnn...`），用于大目录查找与游标遍历验证。
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

/// 辅助断言函数：以只读模式打开 `fixture` 中的整盘 ext4 卷，验证返回预期的格式或校验错误 `expected`。
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

    /// 验证 ext4 Extent 长度字段（`ee_len`）的解码边界：区分已初始化区间（`1..=32768`）、未初始化（Unwritten）区间（`32769..=65535`）以及非法的零长度（返回 `FsError::Corrupt`）。
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

    /// 验证 Extent 节点头部（`ext4_extent_header`）的边界检查：拒绝缓冲区过短、深度不匹配以及有效项数 `eh_entries` 超过容量上限 `eh_max` 的损坏头部。
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

    /// 使用标准测试向量 `"123456789"` 验证 `crc32c` 与 `crc16` 校验和算法以及分段增量计算的正确性。
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

    /// 验证超级块解码对兼容（compat）、只读兼容（ro_compat）、不兼容（incompat）特性位以及日志恢复标志的分类判定与几何参数校验。
    #[test]
    #[ktest(tags("B12"))]
    fn features_layout_and_recovery(context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 验证未知 compat 特性位、日志 `needs_recovery` 提示位及受支持的只读特性组合均可正常解码出几何参数。
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

        // Step 2: 验证不支持的 incompat/ro_compat 特性位返回 `FsError::Unsupported`，非法块大小或每组 Inode 数返回 `FsError::Corrupt`，以及缓冲区或磁盘容量过小返回 `FsError::Corrupt`。
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

    /// 验证 HTree（dx）索引目录根块与内部索引块尾部保留字节（`dt_reserved`）参与目录 CRC32C 校验：保留字节非零但校验和匹配时应通过校验，翻转保留字节任意位时必须返回 `FsError::Checksum`。
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

        // 分别针对逻辑块 0（HTree Root 块）与逻辑块 2（HTree 内部 Index 块）构造带非零保留字段的目录尾并校验 CRC。
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

    /// 验证 ext4 卷的几何参数（块大小、Inode 与块组描述符尺寸、特性位）以及普通文件读取、尾页零填充与 EOF 边界行为。
    ///
    /// # 测试目标
    /// - 验证不同块大小（1024B/2048B/4096B）镜像的几何参数组合与特性标志符合预期。
    /// - 验证 `/pattern` 文件的跨页读取、非对齐偏移读取及尾部不足一页的读取内容正确，且尾页超出 EOF 的部分在页缓存中被清零。
    /// - 验证在 EOF、`u64::MAX` 偏移或空缓冲区处读取返回 0 且不额外分配缓存页，空文件读取同样保持零缓存页占用。
    #[test]
    #[ktest(tags("B12", "B22"))]
    fn geometry_and_pattern_eof(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        // Step 1: 打开测试卷并校验块大小、Inode 大小、块组描述符大小与特性标志组合。
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

        // Step 2: 读取 `pattern` 文件在不同偏移处的内容并校验图样，检查最后一页超出 EOF 区域的字节是否全部清零。
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
        // Step 3: 验证 EOF 处、`u64::MAX` 偏移及空文件的读取返回 0 且不会污染页缓存计数。
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

    /// 验证稀疏文件空洞（Hole）、未初始化 Extent（Unwritten）、多层深树 Extent 以及传统多级间接块（Indirect Blocks）的逻辑块映射与读取结果。
    ///
    /// # 测试目标
    /// - 验证稀疏文件空洞区间与未初始化 Extent 区间读取均返回全零字节，且 `map_block` 返回对应的 `Hole` 或 `Unwritten` 映射状态。
    /// - 验证深度不小于 2 的多层 Extent 树在已映射块与交替空洞块处均能正确读取预期图样或全零数据。
    /// - 验证 1024B 块大小下传统直接块、一级/二级/三级间接块的映射与内容读取，以及超出文件末尾时返回 `BlockMapping::Eof`。
    #[test]
    #[ktest(tags("B13"))]
    fn sparse_unwritten_and_deep_mapping(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let bs = volume.geometry().block_size as u64;
        // Step 1: 验证 `sparse` 稀疏文件在空洞偏移处读出全零、在末尾数据块处读出预期图样。
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

        // Step 2: 验证 `unwritten` 文件的块映射类型（Extent 卷为 `Unwritten`，传统卷为 `Hole`），且读取返回全零。
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

        // Step 3: 验证 `deep` 多层 Extent 树文件的树深、交替分布的数据块与空洞块读取结果。
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

        // Step 4: 在 1024B 块大小镜像上验证传统直接块与一级/二级/三级间接块映射，并检查 EOF 边界映射。
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

    /// 验证硬链接共享唯一 Inode 实例、内联（Fast）与数据块符号链接读取、非 UTF-8 原始字节文件名枚举，以及非法目录查找与非法 Inode 号的错误码。
    #[test]
    #[ktest(tags("B16", "B19"))]
    fn links_names_and_lookup_errors(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        // Step 1: 验证指向同一文件的两个硬链接目录项解析出同一个 `Inode` 与 `AddressSpace` 实例。
        let volume = fixture.open(context)?;
        let root = checked(context, fixture.cache().iget(&volume, 2))?;
        let first = fixture.file(context, &volume, b"pattern")?;
        let hard = fixture.file(context, &volume, b"hardlink")?;
        context.expect_eq(first.ptr_eq(&hard), true);
        context.expect_eq(core::ptr::eq(first.mapping(), hard.mapping()), true);

        // Step 2: 验证内联符号链接（`fastlink`）与外置数据块符号链接（`blocklink`）的标志位及目标路径读取。
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

        // Step 3: 验证包含非 UTF-8 字节（`0xff, 0xfe`）的文件名可通过查找和目录游标遍历正常访问。
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

        // Step 4: 验证不存在的名称、含 `/` 或 `\0` 的非法名称、在普通文件上查找目录项、对普通文件调用 `read_link` 以及越界 Inode 号返回的错误类型。
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

    /// 验证跨块目录与 HTree（dx）哈希索引目录的游标遍历及按名查找：游标只返回有效目录项（过滤未使用的空闲槽、校验尾与 HTree 索引伪项），且所有条目无遗漏、无重复。
    #[test]
    #[ktest(tags("B19"))]
    fn indexed_directory_bytes_and_cursor(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        // Step 1: 遍历开启了 HTree 索引（`EXT4_INDEX_FL = 0x1000`）的大目录 `indexed`，验证 `.`、`..` 与全部 `DIRECTORY_ENTRIES` 个条目均被精确枚举一次。
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

        // Step 2: 验证大目录的首、中、尾条目按名查找命中，不存在的条目返回 `FsError::NotFound`，并校验普通线性目录 `plain` 的总条目数。
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

    /// 验证同一卷内的硬链接共享唯一 `Inode` 与 `AddressSpace` 实例，而不同卷上具有相同 Inode 编号的文件拥有各自独立的 `Inode`、`AddressSpace` 和卷引用。
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

    /// 验证同一块设备上两个相邻且扇区非页对齐的分片视图（`BlockView`）共享底层物理页缓存预算，同时保持各自独立的卷实例、Inode 身份、脏页范围掩码、写权限令牌（`Claim`）、写回域及写回错误状态。
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
        // Step 1: 在可写测试盘上划分两个相邻但非 4KiB 页对齐的 `BlockView`（扇区 1..129 与 129..641），分别格式化为 1024B 块和 4096B 块的 ext4 卷并打开。
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
            // Step 2: 验证重复打开同一视图返回同一 `Ext4Volume` 实例，而两个相邻视图拥有不同的设备号、Inode 实例与文件内容。
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

            // Step 3: 验证两个卷的 `MetadataCache` 实例独立但共享同一个底层物理页预算池，且拒绝使用对方的 `Claim` 令牌越权更新元数据。
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

            // Step 4: 修改两个视图在物理页边界 `boundary`（65536 字节）两侧的元数据块，验证两者共享同一个底层物理页帧但持有互不重叠的页内子块掩码（`0x01` 与 `0xfe`）和独立的 `ToWrite` 标记。
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

            // Step 5: 验证两个卷的写回域（`WritebackDomain`）生命周期与元数据写回错误（`WritebackError`）彼此隔离：向第一个卷注入 I/O 失败不会污染第二个卷的写回状态。
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

    /// 验证 `InodeCache` 与文件页缓存在容量上限下的淘汰回收（`prune_unused`）与对象身份保持语义。
    ///
    /// # 测试目标
    /// - 验证 `InodeCache` 达到容量上限且所有槽位均被外部引用持有时，加载新 Inode 返回 `FsError::NoMemory`，而查找已有 Inode 的硬链接仍命中同一实例。
    /// - 验证文件页缓存（`AddressSpace`）仅回收未被外部持有的干净页（Clean Folio），被持有的页在释放后方可被回收。
    /// - 验证通过 `with_file_owner` 绑定了所属文件强引用的独立 `Folio`（`detached`）在外部 `InodeRef` 全部释放后仍能保活所属 `Inode` 与 `AddressSpace`；直到该 `Folio` 最后一个强引用被释放后，`prune_unused` 才能回收该 Inode。
    #[test]
    #[ktest(tags("B23"))]
    fn inode_and_file_budget_clean_prune(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        // Step 1: 创建容量为 2 的 `InodeCache`，加载根目录与 `pattern` 文件占满槽位，验证加载第 3 个 Inode 返回 `FsError::NoMemory`，而硬链接查找仍复用已有 Inode。
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

        // Step 2: 读取 `pattern` 文件的前 3 页并持有前 2 页的 `Folio` 引用，验证 `prune_unused` 仅回收未被外部持有的页，释放引用后其余页可被正常回收。
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

        // Step 3: 逐一释放 `root`、`file` 与 `hard` 引用，验证未被引用的 Inode 可被 `prune_unused` 回收并腾出槽位加载 `empty` 文件。
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

        // Step 4: 构造绑定了所属文件引用的 `detached` 页面并丢弃直接持有的 `InodeRef`，验证 `detached` 存活期间 Inode 与首页物理地址保持不变，待 `detached` 释放后 Inode 才被最终回收。
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

    /// 验证在物理页分配器与内核堆分配器同时耗尽（OOM）的压力下，已缓存的 Inode 与文件页仍可正常命中读取，未命中的新 Inode 或新缓存页加载返回 `FsError::NoMemory` 且不发起多余块 I/O，解除内存压力后恢复正常加载。
    #[test]
    #[ktest(tags("B23"))]
    fn live_hits_and_new_misses_under_allocator_oom(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        // Step 1: 预先加载根目录与 `pattern` 文件的第 0 页到缓存中，并解析出尚未加载的 `empty` 文件 Inode 编号。
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

        // Step 2: 将预分配的内存压力容器移交给工作线程，在页分配器与堆分配器均耗尽的条件下执行缓存命中读取与未命中加载测试。
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

        // Step 3: 校验工作线程返回结果：已缓存的第 0 页读取成功且物理页地址不变，未命中的 `empty` Inode 与第 1 页读取均返回 `FsError::NoMemory` 且未提交块设备 I/O。
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

        // Step 4: 验证内存压力释放后，`empty` Inode 与 `pattern` 文件的第 1 页均可正常从磁盘加载，且第 0 页依然保持原有物理页帧。
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

            /// 验证打开损坏或特性不受支持的 ext4 镜像时，挂载入口准确返回预期的格式或校验和错误码（而非笼统的硬件 `Io` 错误）。
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

    /// 验证块组描述符（Group Descriptor）CRC 校验失败时，在卷打开阶段或首次加载根目录 Inode 阶段准确返回 `FsError::Checksum`。
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

    /// 验证目标 Inode 自身 CRC 校验失败时，`iget` 返回 `FsError::Checksum`（而非误报 `FsError::NotFound`），且不会在 `InodeCache` 中残留 `Loading` 占位项，后续重试仍触发实际加载并报告相同错误。
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

    /// 验证深层 Extent 树的外部节点（Extent Index/Leaf Block）CRC 校验失败时，`map_block` 与 `read_at` 均返回 `FsError::Checksum`，且对应缓存页记录 `BlockError::Checksum` 错误状态。
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

    /// 验证普通线性目录数据块尾部（Directory Tail）CRC 校验失败时，目录游标遍历（`dir_next`）与按名查找（`lookup`）均返回 `FsError::Checksum`。
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

    /// 验证 HTree（dx）索引目录根块尾部 CRC 校验失败时，目录游标遍历（`dir_next`）与按名查找（`lookup`）均返回 `FsError::Checksum`。
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

    /// 验证未启用目录 CRC 的镜像中遇到 `rec_len == 0` 的损坏目录项时，`dir_next` 与 `lookup` 立即拒绝并返回 `FsError::Corrupt`，防止死循环或误报 `NotFound`。
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

    /// 验证未启用目录 CRC 的 HTree（dx）索引目录中遇到 `count == 0` 的非法空索引头时，`dir_next` 与 `lookup` 均返回 `FsError::Corrupt`。
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

    /// 验证损坏的块映射指针（根 Extent 头非法或间接块数据指针越界）在 Inode 加载阶段或块映射（`map_block` / `read_at`）阶段被拦截并返回 `FsError::Corrupt`。
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

    /// 验证文件前 3 页（直接块映射合法）与第 4 页（一级间接块映射损坏）连续读取时的部分进度（Partial Progress）语义：首次跨 4 页读取成功交付已完成的 12288 字节有效前缀且不覆盖后续缓冲区，紧接着从偏移 12288 处再次读取时返回 `FsError::Corrupt`。
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
