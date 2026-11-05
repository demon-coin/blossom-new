//! 同一真实磁盘根下挂入内存文件系统，消费设备数据和每次打开的动态快照。

use crate::boot;
use crate::fs::ext4::crc::crc32c;
use crate::fs::file::{open_path_with_flags, FileIoContext};
use crate::fs::ktest::path::PathOwner;
use crate::fs::ktest::*;
use crate::fs::memfs::{Filesystem, MemoryFs, MemoryFsKind};
use crate::fs::{devfs, procfs, InodeKind, ReadBuffer, SeekFrom, WriteBuffer};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::Mm;
use crate::mm::user_table::PagePermissions;
use alloc::sync::Arc;
use alloc::vec::Vec;

const ADDRESS: usize = 0x4800_0000;

const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 只消费生产路径解析，不以字符串选择设备或后端。
fn resolve(context: &TestContext<'_>, fixture: &PathOwner, name: &[u8]) -> Result<Path, TestError> {
    checked(
        context,
        fixture
            .vfs()
            .resolve(fixture.context(), name, ResolveMode::Follow),
    )
}

/// 同一 RO 磁盘根已有的目录是覆盖点，内存文件系统使用独立 RW 挂载。
fn prepare_tmp(
    context: &TestContext<'_>,
    fixture: &mut PathOwner,
    options: &[u8],
) -> Result<Arc<MemoryFs>, TestError> {
    fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
    let fs = checked(context, MemoryFs::try_tmpfs(options))?;
    let covered = resolve(context, fixture, b"/plain")?;
    checked(
        context,
        fixture.vfs().mount(
            &covered,
            Filesystem::Memory(fs.clone()),
            AccessMode::ReadWrite,
        ),
    )?;
    Ok(fs)
}

/// 测试创建仍走父目录准备、真实 OFD 准备及无失败名字发表。
fn create(
    context: &TestContext<'_>,
    fixture: &PathOwner,
    name: &[u8],
) -> Result<FileRef, TestError> {
    let vfs = fixture.vfs();
    checked(
        context,
        vfs.open_with(
            fixture.context(),
            fixture.context().cwd(),
            name,
            (0, 0),
            Some((0o666, 0)),
            true,
            false,
            |path, _| open_path_with_flags(vfs, path, 2, FileIoContext::Kernel),
        ),
    )
}

/// 原 VFS 凭据和挂载协议决定每次打开的真实操作能力。
fn open(
    context: &TestContext<'_>,
    fixture: &PathOwner,
    name: &[u8],
    flags: u32,
) -> Result<FileRef, TestError> {
    let path = resolve(context, fixture, name)?;
    checked(
        context,
        open_path_with_flags(fixture.vfs(), path, flags, FileIoContext::Kernel),
    )
}

/// 内存目录是后续实际挂载的覆盖点，不修改 RO 磁盘树。
fn mkdir(context: &TestContext<'_>, fixture: &PathOwner, name: &[u8]) -> TestResult {
    checked(
        context,
        fixture.vfs().mkdir_from(
            fixture.context(),
            fixture.context().cwd(),
            name,
            0o777,
            (0, 0),
            0,
        ),
    )
}

/// 读取本 OFD 的完整长度，末尾必须准确返回 EOF。
fn text(context: &TestContext<'_>, file: &FileRef) -> Result<Vec<u8>, TestError> {
    let length = checked(context, file.stat())?.size as usize;
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| TestError("NoMemory: proc snapshot test bytes"))?;
    bytes.resize(length, 0xa7);
    assert_eq!(checked(context, file.read(&mut bytes))?, length);
    assert_eq!(checked(context, file.read(&mut [0xa7; 1]))?, 0);
    Ok(bytes)
}

/// 从真实六列记录核对八个后端身份和完整转义路径，卸载的目标不得留在新快照。
fn growth_mount_records(
    context: &mut TestContext<'_>,
    bytes: &[u8],
    escaped_base: &[u8],
    devices: &[u64; 8],
    mounted: usize,
) -> TestResult {
    let text = core::str::from_utf8(bytes).map_err(|_| TestError("mount snapshot is not UTF-8"))?;
    context.expect_eq(text.lines().count(), mounted + 3);
    let mut seen = 0u16;
    for line in text.lines() {
        context.expect_eq(line.split_ascii_whitespace().count(), 6);
        let mut fields = line.split_ascii_whitespace();
        let device = fields
            .next()
            .and_then(|source| source.strip_prefix("tmpfs:"))
            .and_then(|digits| digits.parse::<u64>().ok());
        let Some(index) = devices
            .iter()
            .position(|&expected| Some(expected) == device)
        else {
            continue;
        };
        context.expect_eq(seen & (1 << index), 0);
        seen |= 1 << index;
        let suffix = fields
            .next()
            .and_then(|path| path.as_bytes().strip_prefix(escaped_base));
        context.expect_eq(suffix, Some([b'/', b'm', b'0' + index as u8].as_slice()));
        context.expect_eq(fields.next(), Some("tmpfs"));
        context.expect_eq(fields.next(), Some("rw"));
        context.expect_eq(fields.next(), Some("0"));
        context.expect_eq(fields.next(), Some("0"));
        context.expect_eq(fields.next(), None);
    }
    context.expect_eq(seen, (1u16 << mounted) - 1);
    Ok(())
}

/// 从实际 meminfo 文本取十进制 kB，不把预先生成值当作数据来源。
fn field(bytes: &[u8], name: &str) -> Result<usize, TestError> {
    let text = core::str::from_utf8(bytes).map_err(|_| TestError("proc text is not UTF-8"))?;
    let line = text
        .lines()
        .find(|line| line.starts_with(name))
        .ok_or(TestError("proc memory field missing"))?;
    let mut words = line.split_ascii_whitespace();
    let _ = words.next();
    let value = words
        .next()
        .ok_or(TestError("proc memory value missing"))?
        .parse()
        .map_err(|_| TestError("proc memory value invalid"))?;
    if words.next() != Some("kB") || words.next().is_some() {
        return Err(TestError("proc memory unit invalid"));
    }
    Ok(value)
}

/// 生产页表和匿名范围用于真实 uaccess 故障，禁止用模拟缓冲区替代。
fn address_space() -> Result<Mm, TestError> {
    let mm = Mm::try_new().map_err(|_| TestError("NoMemory: pseudo file MM"))?;
    mm.map_anonymous(ADDRESS, mm::PAGE_SIZE, RW, RW)
        .map_err(|_| TestError("pseudo file MM mapping failed"))?;
    Ok(mm)
}

/// 父模块仅收集实际宏锚点，不维护另一张测试名单。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 1] {
    [filesystems::__ktest_anchor()]
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-pseudo"), resources("root"), timeout_ms = 90000)]
pub(crate) mod filesystems {
    use super::*;

    /// 后端与设备身份来自同 VFS 的实际 inode；固定项可在清名字缓存后重新取得。
    #[test]
    #[ktest(tags("T01"))]
    fn real_mount_kinds_devices_and_fixed_names(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let tmp = prepare_tmp(context, fixture, b"mode=0777")?;
        mkdir(context, fixture, b"/plain/dev")?;
        mkdir(context, fixture, b"/plain/proc")?;
        let dev = checked(context, devfs::try_new(0o755))?;
        let proc = checked(context, procfs::try_new(0o555))?;
        let dev_cover = resolve(context, fixture, b"/plain/dev")?;
        let proc_cover = resolve(context, fixture, b"/plain/proc")?;
        checked(
            context,
            fixture.vfs().mount(
                &dev_cover,
                Filesystem::Memory(dev.clone()),
                AccessMode::ReadOnly,
            ),
        )?;
        checked(
            context,
            fixture.vfs().mount(
                &proc_cover,
                Filesystem::Memory(proc.clone()),
                AccessMode::ReadOnly,
            ),
        )?;

        context.expect_eq(tmp.kind(), MemoryFsKind::Tmpfs);
        context.expect_eq(dev.kind(), MemoryFsKind::Devfs);
        context.expect_eq(proc.kind(), MemoryFsKind::Procfs);
        let disk = checked(context, open(context, fixture, b"/pattern", 0)?.stat())?;
        let null = open(context, fixture, b"/plain/dev/null", 2)?;
        let zero = open(context, fixture, b"/plain/dev/zero", 2)?;
        let mounts = open(context, fixture, b"/plain/proc/mounts", 0)?;
        let null_stat = checked(context, null.stat())?;
        let zero_stat = checked(context, zero.stat())?;
        context.expect_eq(
            (null_stat.kind, null_stat.rdev, null_stat.mode & 0o170000),
            (InodeKind::Character, 0x103, 0o020000),
        );
        context.expect_eq(
            (zero_stat.kind, zero_stat.rdev),
            (InodeKind::Character, 0x105),
        );
        context.expect_eq(null_stat.device, dev.device());
        context.expect_eq(zero_stat.device, null_stat.device);
        context.expect_eq(checked(context, mounts.stat())?.device, proc.device());
        context.expect_eq(
            disk.device == dev.device()
                || dev.device() == proc.device()
                || proc.device() == tmp.device()
                || disk.device == tmp.device(),
            false,
        );

        let old_inode = null.path().unwrap().inode().unwrap().clone();
        drop(null);
        fixture.vfs().prune_unused();
        let reopened = open(context, fixture, b"/plain/dev/null", 2)?;
        context.expect_eq(
            reopened.path().unwrap().inode().unwrap().ptr_eq(&old_inode),
            true,
        );
        let directory = open(context, fixture, b"/plain/dev", 0)?;
        let mut names = 0;
        while let Some(entry) = checked(context, directory.readdir())? {
            if entry.name == b"null" || entry.name == b"zero" {
                context.expect_eq(entry.kind, Some(InodeKind::Character));
                names |= if entry.name == b"null" { 1 } else { 2 };
            }
        }
        context.expect_eq(names, 3);

        context.expect_eq(
            fixture
                .vfs()
                .open_with(
                    fixture.context(),
                    fixture.context().cwd(),
                    b"/plain/dev/unregistered",
                    (0, 0),
                    Some((0o666, 0)),
                    true,
                    false,
                    |path, _| open_path_with_flags(fixture.vfs(), path, 2, FileIoContext::Kernel),
                )
                .err(),
            Some(FsError::ReadOnly),
        );
        context.expect_eq(
            fixture.vfs().mkdir_from(
                fixture.context(),
                fixture.context().cwd(),
                b"/plain/proc/custom",
                0o777,
                (0, 0),
                0,
            ),
            Err(FsError::ReadOnly),
        );
        Ok(())
    }

    /// 清缓存不能丢 RAM 权威内容；截断、删除与最后外部页拥有分别决定归页。
    #[test]
    #[ktest(tags("T02"))]
    fn ram_bytes_truncate_unlink_and_last_page_owner(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let tmp = prepare_tmp(context, fixture, b"mode=0777,nr_inodes=2,nr_blocks=2")?;
        let file = create(context, fixture, b"/plain/data")?;
        let bytes = [0x6d; mm::PAGE_SIZE + 17];
        context.expect_eq(checked(context, file.write(&bytes))?, bytes.len());
        context.expect_eq(tmp.used_pages(), 2);
        let identity = file.mapping_inode().unwrap().number();
        drop(file);
        fixture.vfs().prune_unused();
        let file = open(context, fixture, b"/plain/data", 2)?;
        context.expect_eq(file.mapping_inode().unwrap().number(), identity);
        let mut actual = [0; mm::PAGE_SIZE + 17];
        context.expect_eq(checked(context, file.read(&mut actual))?, actual.len());
        context.expect_eq(actual, bytes);

        checked(context, file.truncate(13))?;
        context.expect_eq(tmp.used_pages(), 1);
        checked(context, file.truncate(100))?;
        let mut tail = [0xa7; 100];
        context.expect_eq(checked(context, file.read_at(0, &mut tail))?, tail.len());
        context.expect_eq(&tail[..13], &[0x6d; 13]);
        context.expect_eq(&tail[13..], &[0; 87]);

        checked(
            context,
            fixture.vfs().unlink_from(
                fixture.context(),
                fixture.context().cwd(),
                b"/plain/data",
                (0, 0),
            ),
        )?;
        context.expect_eq(
            fixture
                .vfs()
                .resolve(fixture.context(), b"/plain/data", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );
        context.expect_eq(checked(context, file.stat())?.links, 0);
        context.expect_eq(checked(context, file.write_at(7, b"open"))?, 4);

        let mut attempt = ReclaimAttempt::new();
        let page = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        drop(file);
        fixture.vfs().prune_unused();
        context.expect_eq(tmp.used_pages(), 1);
        context.expect_eq(checked(context, tmp.statfs())?.freefiles, 0);
        let mut retained = [0; 4];
        checked(
            context,
            page.copy_to(7, &mut retained).map_err(FsError::from),
        )?;
        context.expect_eq(&retained, b"open");

        drop(page);
        fixture.vfs().prune_unused();
        context.expect_eq(tmp.used_pages(), 0);
        context.expect_eq(checked(context, tmp.statfs())?.freefiles, 1);
        let reused = create(context, fixture, b"/plain/new-identity")?;
        context.expect_eq(identity < reused.mapping_inode().unwrap().number(), true);
        Ok(())
    }

    /// 每次打开从本 VFS 和原 MM、时间账务生成文本，旧 OFD 保留原来的快照。
    #[test]
    #[ktest(tags("T03"))]
    fn proc_reopen_observes_mount_memory_and_time(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let _tmp = prepare_tmp(context, fixture, b"mode=0777")?;
        mkdir(context, fixture, b"/plain/proc")?;
        mkdir(context, fixture, b"/plain/change")?;
        let cover = resolve(context, fixture, b"/plain/proc")?;
        checked(
            context,
            fixture.vfs().mount(
                &cover,
                Filesystem::Memory(checked(context, procfs::try_new(0o555))?),
                AccessMode::ReadOnly,
            ),
        )?;

        let first = open(context, fixture, b"/plain/proc/mounts", 0)?;
        let before = text(context, &first)?;
        context.expect_eq(
            &before,
            &checked(context, fixture.vfs().mounts_text(&fixture.vfs().root()))?,
        );
        let change_cover = resolve(context, fixture, b"/plain/change")?;
        let change = checked(context, MemoryFs::try_new(MemoryFsKind::Tmpfs, 0o777))?;
        checked(
            context,
            fixture.vfs().mount(
                &change_cover,
                Filesystem::Memory(change),
                AccessMode::ReadWrite,
            ),
        )?;
        let second = open(context, fixture, b"/plain/proc/mounts", 0)?;
        let after = text(context, &second)?;
        context.expect_eq(
            &after,
            &checked(context, fixture.vfs().mounts_text(&fixture.vfs().root()))?,
        );
        context.expect_eq(before == after, false);
        checked(context, first.seek(SeekFrom::Start(0)))?;
        context.expect_eq(&text(context, &first)?, &before);

        checked(context, fixture.vfs().unmount(&change_cover))?;
        context.expect_eq(
            &text(context, &open(context, fixture, b"/plain/proc/mounts", 0)?)?,
            &before,
        );
        context.expect_eq(first.readdir().err(), Some(FsError::NotDirectory));
        context.expect_eq(first.write(b""), Err(FsError::BadFd));
        context.expect_eq(
            first.ioctl(0, 0, FileIoContext::Kernel),
            Err(FsError::NotTty),
        );

        let mut prefix = [0; 8];
        context.expect_eq(
            checked(context, first.read_at(0, &mut prefix))?,
            prefix.len(),
        );
        context.expect_eq(prefix.as_slice(), &before[..8]);
        context.expect_eq(
            checked(context, first.seek(SeekFrom::Current(0)))?,
            before.len() as u64,
        );
        context.expect_eq(
            checked(context, first.seek(SeekFrom::End(-1)))?,
            before.len() as u64 - 1,
        );
        context.expect_eq(checked(context, first.read(&mut prefix))?, 1);
        context.expect_eq(
            first.seek(SeekFrom::Start(u64::MAX)),
            Err(FsError::InvalidOperation),
        );

        checked(context, first.seek(SeekFrom::Start(0)))?;
        let shared = first.clone();
        context.expect_eq(checked(context, shared.read(&mut prefix))?, prefix.len());
        context.expect_eq(
            checked(context, first.seek(SeekFrom::Current(0)))?,
            prefix.len() as u64,
        );
        let independent = open(context, fixture, b"/plain/proc/mounts", 0)?;
        context.expect_eq(checked(context, independent.seek(SeekFrom::Current(0)))?, 0);

        let memory_before = text(context, &open(context, fixture, b"/plain/proc/meminfo", 0)?)?;
        let stats = mm::stats();
        context.expect_eq(
            field(&memory_before, "MemTotal:")?,
            (stats.present_pages - stats.reserved_pages - stats.metadata_pages)
                * (mm::PAGE_SIZE / 1024),
        );
        let address_space =
            Mm::try_new().map_err(|_| TestError("NoMemory: proc memory observation MM"))?;
        address_space
            .map_anonymous(ADDRESS, 64 * mm::PAGE_SIZE, RW, RW)
            .map_err(|_| TestError("proc memory observation mapping failed"))?;
        for page in 0..64 {
            context.expect_eq(
                address_space
                    .copyout(ADDRESS + page * mm::PAGE_SIZE, &[0x72])
                    .copied,
                1,
            );
        }
        let memory_after = text(context, &open(context, fixture, b"/plain/proc/meminfo", 0)?)?;
        context.expect_eq(
            field(&memory_after, "MemFree:")? < field(&memory_before, "MemFree:")?,
            true,
        );

        let uptime_before = text(context, &open(context, fixture, b"/plain/proc/uptime", 0)?)?;
        timer::sleep(time::Duration::from_millis(25));
        let uptime_after = text(context, &open(context, fixture, b"/plain/proc/uptime", 0)?)?;
        context.expect_eq(uptime_before == uptime_after, false);
        context.expect_eq(
            core::str::from_utf8(&uptime_after)
                .unwrap()
                .split_ascii_whitespace()
                .count(),
            2,
        );
        Ok(())
    }

    /// 在真实 procfs 打开实例上核对冻结启动参数的完整字节、分块读取与共享游标。
    #[test]
    #[ktest(tags("F19", "fs-proc-cmdline"))]
    fn cmdline_preserves_all_boot_arguments(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let _tmp = prepare_tmp(context, fixture, b"mode=0777")?;
        mkdir(context, fixture, b"/plain/proc")?;
        let cover = resolve(context, fixture, b"/plain/proc")?;
        checked(
            context,
            fixture.vfs().mount(
                &cover,
                Filesystem::Memory(checked(context, procfs::try_new(0o555))?),
                AccessMode::ReadOnly,
            ),
        )?;
        let root = resolve(context, fixture, b"/plain")?;
        let scoped = checked(context, FsContext::new(root.clone(), root))?;
        let path = checked(
            context,
            fixture
                .vfs()
                .resolve(&scoped, b"/proc/cmdline", ResolveMode::Follow),
        )?;
        let first = checked(
            context,
            open_path_with_flags(fixture.vfs(), path.clone(), 0, FileIoContext::Kernel),
        )?;

        let bootargs = boot::info().bootargs();
        let length = bootargs
            .len()
            .checked_add(1)
            .ok_or(TestError("proc cmdline expected length overflow"))?;
        let mut expected = Vec::new();
        expected
            .try_reserve_exact(length)
            .map_err(|_| TestError("NoMemory: proc cmdline expected bytes"))?;
        expected.extend_from_slice(bootargs);
        expected.push(b'\n');
        let actual = text(context, &first)?;
        context.diagnostic(format_args!(
            "fs-proc-cmdline bootargs-bytes={} bootargs-crc32c={:08x} snapshot-bytes={} snapshot-crc32c={:08x} chunk-bytes=257",
            bootargs.len(),
            !crc32c(!0, bootargs),
            actual.len(),
            !crc32c(!0, &actual),
        ));
        context.expect_eq(actual.as_slice(), expected.as_slice());
        context.expect_eq(checked(context, first.stat())?.size, length as u64);

        checked(context, first.seek(SeekFrom::Start(0)))?;
        let mut window = [0xa7; 257];
        let mut offset = 0;
        while offset < length {
            let count = window.len().min(length - offset);
            assert_eq!(checked(context, first.read(&mut window[..count]))?, count);
            context.expect_eq(&window[..count], &expected[offset..offset + count]);
            offset += count;
        }
        context.expect_eq(checked(context, first.read(&mut window))?, 0);
        context.expect_eq(
            checked(context, first.read_at(length as u64, &mut window))?,
            0,
        );

        checked(context, first.seek(SeekFrom::Start(0)))?;
        let tail = length.saturating_sub(13);
        let count = length - tail;
        context.expect_eq(
            checked(context, first.read_at(tail as u64, &mut window))?,
            count,
        );
        context.expect_eq(&window[..count], &expected[tail..]);
        context.expect_eq(checked(context, first.seek(SeekFrom::Current(0)))?, 0);

        let table = checked(context, FileTable::try_new())?;
        let fd =
            checked(context, table.reserve(2))?.publish(first.clone(), DescriptorFlags::default());
        let duplicate = checked(context, table.dup(fd, 2))?;
        let count = length.min(13);
        context.expect_eq(
            checked(context, table.read(duplicate, &mut window[..13]))?,
            count,
        );
        context.expect_eq(&window[..count], &expected[..count]);
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(0)))?,
            count as u64,
        );
        context.expect_eq(
            checked(context, first.seek(SeekFrom::Current(0)))?,
            count as u64,
        );

        let independent = checked(
            context,
            open_path_with_flags(fixture.vfs(), path, 0, FileIoContext::Kernel),
        )?;
        context.expect_eq(checked(context, independent.seek(SeekFrom::Current(0)))?, 0);
        context.expect_eq(text(context, &independent)?.as_slice(), expected.as_slice());
        context.expect_eq(
            checked(context, first.seek(SeekFrom::Current(0)))?,
            count as u64,
        );

        drop(table);
        drop(independent);
        drop(first);
        checked(context, fixture.vfs().unmount(&cover))?;
        Ok(())
    }

    /// 合法长挂载路径生成超过 64KiB 的完整快照，旧 OFD 与真实卸载后的新 OFD 各自冻结。
    #[test]
    #[ktest(tags("F23", "fs-proc-growth"))]
    fn proc_large_mount_snapshot_keeps_bytes_and_open_description(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 12)?;
        let cover = resolve(context, fixture, b"/plain")?;
        checked(
            context,
            fixture.vfs().mount(
                &cover,
                Filesystem::Memory(checked(context, MemoryFs::try_tmpfs(b"mode=0777"))?),
                AccessMode::ReadWrite,
            ),
        )?;
        mkdir(context, fixture, b"/plain/proc")?;
        let proc_cover = resolve(context, fixture, b"/plain/proc")?;
        checked(
            context,
            fixture.vfs().mount(
                &proc_cover,
                Filesystem::Memory(checked(context, procfs::try_new(0o555))?),
                AccessMode::ReadOnly,
            ),
        )?;

        let mut path = Vec::new();
        path.try_reserve_exact(4096)
            .map_err(|_| TestError("NoMemory: proc growth pathname"))?;
        path.extend_from_slice(b"/plain");
        let mut escaped = Vec::new();
        escaped
            .try_reserve_exact(6 + 12 * (1 + 255 * 4))
            .map_err(|_| TestError("NoMemory: proc growth escaped pathname"))?;
        escaped.extend_from_slice(b"/plain");
        // 十二个合法组件的八进制转义使八条真实记录超过旧快照容量，原路径仍小于 PATH_MAX。
        for _ in 0..12 {
            path.push(b'/');
            path.resize(path.len() + 255, b'#');
            mkdir(context, fixture, &path)?;
            escaped.push(b'/');
            for _ in 0..255 {
                escaped.extend_from_slice(b"\\043");
            }
        }
        let base_length = path.len();
        let mut covers = Vec::new();
        covers
            .try_reserve_exact(8)
            .map_err(|_| TestError("NoMemory: proc growth mount covers"))?;
        for index in 0..8 {
            path.extend_from_slice(&[b'/', b'm', b'0' + index]);
            assert!(path.len() < 4096);
            mkdir(context, fixture, &path)?;
            covers.push(resolve(context, fixture, &path)?);
            path.truncate(base_length);
        }

        let mut devices = [0; 8];
        for (index, cover) in covers.iter().enumerate() {
            let fs = checked(context, MemoryFs::try_tmpfs(b"mode=0777"))?;
            let device = fs.device();
            context.expect_eq(devices[..index].contains(&device), false);
            devices[index] = device;
            checked(
                context,
                fixture
                    .vfs()
                    .mount(cover, Filesystem::Memory(fs), AccessMode::ReadWrite),
            )?;
            path.extend_from_slice(&[b'/', b'm', b'0' + index as u8]);
            context.expect_eq(
                checked(context, open(context, fixture, &path, 0)?.stat())?.device,
                device,
            );
            path.truncate(base_length);
        }

        let first = open(context, fixture, b"/plain/proc/mounts", 0)?;
        let before = text(context, &first)?;
        assert!(65536 < before.len());
        growth_mount_records(context, &before, &escaped, &devices, 8)?;
        context.expect_eq(checked(context, first.stat())?.size, before.len() as u64);
        context.expect_eq(checked(context, first.seek(SeekFrom::Start(65520)))?, 65520);
        let mut window = [0xa7; 64];
        context.expect_eq(checked(context, first.read(&mut window))?, window.len());
        context.expect_eq(window.as_slice(), &before[65520..65584]);
        context.expect_eq(
            checked(context, first.read_at(65530, &mut window))?,
            window.len(),
        );
        context.expect_eq(window.as_slice(), &before[65530..65594]);
        context.expect_eq(checked(context, first.seek(SeekFrom::Current(0)))?, 65584);

        let table = checked(context, FileTable::try_new())?;
        let fd =
            checked(context, table.reserve(2))?.publish(first.clone(), DescriptorFlags::default());
        let duplicate = checked(context, table.dup(fd, 2))?;
        context.expect_eq(
            checked(context, table.read(duplicate, &mut window))?,
            window.len(),
        );
        context.expect_eq(window.as_slice(), &before[65584..65648]);
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(0)))?,
            65648,
        );
        let independent = open(context, fixture, b"/plain/proc/mounts", 0)?;
        context.expect_eq(checked(context, independent.seek(SeekFrom::Current(0)))?, 0);
        context.expect_eq(
            checked(context, independent.read(&mut window))?,
            window.len(),
        );
        context.expect_eq(window.as_slice(), &before[..window.len()]);
        context.expect_eq(checked(context, first.seek(SeekFrom::Current(0)))?, 65648);

        context.expect_eq(
            checked(context, first.seek(SeekFrom::End(-17)))?,
            before.len() as u64 - 17,
        );
        context.expect_eq(checked(context, first.read(&mut window))?, 17);
        context.expect_eq(&window[..17], &before[before.len() - 17..]);
        context.expect_eq(checked(context, first.read(&mut window))?, 0);
        context.expect_eq(
            checked(
                context,
                first.read_at(before.len() as u64 - 17, &mut window),
            )?,
            17,
        );
        context.expect_eq(&window[..17], &before[before.len() - 17..]);
        context.expect_eq(
            checked(context, first.seek(SeekFrom::Current(0)))?,
            before.len() as u64,
        );
        context.expect_eq(
            checked(context, first.read_at(before.len() as u64, &mut window))?,
            0,
        );
        context.expect_eq(
            checked(context, first.seek(SeekFrom::End(7)))?,
            before.len() as u64 + 7,
        );
        context.expect_eq(checked(context, first.read(&mut window))?, 0);

        checked(context, fixture.vfs().unmount(&covers[7]))?;
        let second = open(context, fixture, b"/plain/proc/mounts", 0)?;
        let after = text(context, &second)?;
        growth_mount_records(context, &after, &escaped, &devices, 7)?;
        context.expect_eq(after.len() < before.len(), true);
        context.expect_eq(checked(context, first.stat())?.size, before.len() as u64);
        context.expect_eq(checked(context, second.stat())?.size, after.len() as u64);
        checked(context, first.seek(SeekFrom::Start(0)))?;
        context.expect_eq(text(context, &first)?.as_slice(), before.as_slice());
        context.expect_eq(first.write(b""), Err(FsError::BadFd));
        context.expect_eq(first.truncate(0), Err(FsError::ReadOnly));
        Ok(())
    }

    /// RO 磁盘不能写而 RW tmpfs 可写，字符设备独立数据面消费真实 uaccess 前缀。
    #[test]
    #[ktest(tags("T04"))]
    fn readonly_root_device_data_and_user_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let _tmp = prepare_tmp(context, fixture, b"mode=0777")?;
        let disk = resolve(context, fixture, b"/pattern")?;
        context.expect_eq(
            open_path_with_flags(fixture.vfs(), disk, 2, FileIoContext::Kernel).err(),
            Some(FsError::ReadOnly),
        );
        let data = create(context, fixture, b"/plain/rw")?;
        context.expect_eq(checked(context, data.write(b"ram"))?, 3);

        mkdir(context, fixture, b"/plain/dev")?;
        let cover = resolve(context, fixture, b"/plain/dev")?;
        checked(
            context,
            fixture.vfs().mount(
                &cover,
                Filesystem::Memory(checked(context, devfs::try_new(0o755))?),
                AccessMode::ReadOnly,
            ),
        )?;
        let null = open(context, fixture, b"/plain/dev/null", 2)?;
        let zero = open(context, fixture, b"/plain/dev/zero", 2)?;
        context.expect_eq(checked(context, null.write(&[0xa7; 1027]))?, 1027);
        context.expect_eq(checked(context, null.read(&mut [0xa7; 17]))?, 0);
        let mut zeros = [0xa7; 1027];
        context.expect_eq(checked(context, zero.read(&mut zeros))?, zeros.len());
        context.expect_eq(zeros, [0; 1027]);
        for file in [&null, &zero] {
            context.expect_eq(checked(context, file.seek(SeekFrom::Start(u64::MAX)))?, 0);
            context.expect_eq(checked(context, file.seek(SeekFrom::Current(-7)))?, 0);
            context.expect_eq(checked(context, file.seek(SeekFrom::End(-9)))?, 0);
            context.expect_eq(
                file.ioctl(0, 0, FileIoContext::Kernel),
                Err(FsError::NotTty),
            );
            context.expect_eq(file.readdir().err(), Some(FsError::NotDirectory));
            context.expect_eq(file.truncate(0), Err(FsError::InvalidOperation));
            checked(context, file.set_status_flags(0o4000))?;
            context.expect_eq(file.flags() & 0o3, 2);
            context.expect_eq(file.flags() & 0o4000, 0o4000);
        }
        let read_only = open(context, fixture, b"/plain/dev/null", 0)?;
        context.expect_eq(read_only.write(b""), Err(FsError::BadFd));
        let write_only = open(context, fixture, b"/plain/dev/zero", 1)?;
        context.expect_eq(write_only.read(&mut []), Err(FsError::BadFd));

        let mm = address_space()?;
        context.expect_eq(
            mm.copyout(ADDRESS + mm::PAGE_SIZE - 17, &[0x72; 17]).copied,
            17,
        );
        for file in [&null, &zero] {
            let mut source = checked(
                context,
                WriteBuffer::user(&mm, ADDRESS + mm::PAGE_SIZE - 17, 64),
            )?;
            context.expect_eq(
                checked(context, file.write_from(&mut source, FileIoContext::Kernel))?,
                17,
            );
            context.expect_eq(source.copied(), 17);
            let mut source = checked(context, WriteBuffer::user(&mm, ADDRESS + mm::PAGE_SIZE, 64))?;
            context.expect_eq(
                file.write_at_from(7, &mut source, FileIoContext::Kernel),
                Err(FsError::BadAddress),
            );
            let mut empty = checked(context, WriteBuffer::user(&mm, usize::MAX, 0))?;
            context.expect_eq(
                checked(context, file.write_from(&mut empty, FileIoContext::Kernel))?,
                0,
            );
        }

        let mut target = checked(
            context,
            ReadBuffer::user(&mm, ADDRESS + mm::PAGE_SIZE - 17, 64),
        )?;
        context.expect_eq(
            checked(context, zero.read_into(&mut target, FileIoContext::Kernel))?,
            17,
        );
        context.expect_eq(target.copied(), 17);
        let mut bytes = [0xa7; 17];
        context.expect_eq(
            mm.copyin(&mut bytes, ADDRESS + mm::PAGE_SIZE - 17).copied,
            bytes.len(),
        );
        context.expect_eq(bytes, [0; 17]);
        let mut invalid = checked(context, ReadBuffer::user(&mm, ADDRESS + mm::PAGE_SIZE, 64))?;
        context.expect_eq(
            zero.read_at_into(7, &mut invalid, FileIoContext::Kernel),
            Err(FsError::BadAddress),
        );
        context.expect_eq(
            checked(context, null.read_into(&mut invalid, FileIoContext::Kernel))?,
            0,
        );
        let mut empty = checked(context, ReadBuffer::user(&mm, usize::MAX, 0))?;
        context.expect_eq(
            checked(context, zero.read_into(&mut empty, FileIoContext::Kernel))?,
            0,
        );
        Ok(())
    }

    /// 活 inode 与实际原页的上限分别拒绝新拥有，释放后可继续使用同一实例。
    #[test]
    #[case("inode", 0u8)]
    #[case("page", 1u8)]
    #[ktest(tags("T02", "fs-oom"))]
    fn ram_capacity_release_and_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
        quota: u8,
    ) -> TestResult {
        let tmp = prepare_tmp(context, fixture, b"mode=0777,nr_inodes=128,nr_blocks=4096")?;
        if quota == 0 {
            for number in 0..127 {
                let mut name = *b"/plain/000";
                name[7] = b'0' + (number / 100) as u8;
                name[8] = b'0' + (number / 10 % 10) as u8;
                name[9] = b'0' + (number % 10) as u8;
                drop(create(context, fixture, &name)?);
                fixture.vfs().prune_unused();
            }

            context.expect_eq(
                fixture
                    .vfs()
                    .open_with(
                        fixture.context(),
                        fixture.context().cwd(),
                        b"/plain/full",
                        (0, 0),
                        Some((0o666, 0)),
                        true,
                        false,
                        |path, _| {
                            open_path_with_flags(fixture.vfs(), path, 2, FileIoContext::Kernel)
                        },
                    )
                    .err(),
                Some(FsError::NoSpace),
            );

            checked(
                context,
                fixture.vfs().unlink_from(
                    fixture.context(),
                    fixture.context().cwd(),
                    b"/plain/000",
                    (0, 0),
                ),
            )?;
            fixture.vfs().prune_unused();
            let reused = create(context, fixture, b"/plain/reused")?;
            context.expect_eq(reused.write(b"live"), Ok(4));
            let mut bytes = [0; 4];
            context.expect_eq(reused.read_at(0, &mut bytes), Ok(4));
            context.expect_eq(&bytes, b"live");
        } else {
            assert_eq!(quota, 1);
            let file = create(context, fixture, b"/plain/pages")?;
            let bytes = [0x3d; mm::PAGE_SIZE];
            for page in 0..4096 {
                context.expect_eq(
                    file.write_at((page * mm::PAGE_SIZE) as u64, &bytes),
                    Ok(bytes.len()),
                );
            }
            context.expect_eq(tmp.used_pages(), 4096);
            context.expect_eq(
                file.write_at((4096 * mm::PAGE_SIZE) as u64, &[0xa7]),
                Err(FsError::NoSpace),
            );
            context.expect_eq(
                checked(context, file.stat())?.size,
                (4096 * mm::PAGE_SIZE) as u64,
            );
            let mut last = [0; 4];
            context.expect_eq(
                file.read_at((4096 * mm::PAGE_SIZE - 4) as u64, &mut last),
                Ok(4),
            );
            context.expect_eq(last, [0x3d; 4]);

            checked(context, file.truncate(0))?;
            context.expect_eq(tmp.used_pages(), 0);
            context.expect_eq(file.write_at(0, b"again"), Ok(5));
            let mut bytes = [0; 5];
            context.expect_eq(file.read_at(0, &mut bytes), Ok(5));
            context.expect_eq(&bytes, b"again");
        }
        Ok(())
    }

    /// 真实页和堆耗尽时取消私有候选，不留下尺寸或原页额度；归还压力后正常写入。
    #[test]
    #[ktest(tags("T02", "fs-oom"))]
    fn ram_unpublished_page_oom_keeps_file_and_charge(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let tmp = prepare_tmp(context, fixture, b"mode=0777")?;
        let file = create(context, fixture, b"/plain/oom")?;

        let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let probes = pressure.exhaust();
        let result = file.write_at(0, b"lost");
        drop(pressure);
        context.expect_eq(probes, (true, true));
        context.expect_eq(result, Err(FsError::NoMemory));
        context.expect_eq(tmp.used_pages(), 0);
        context.expect_eq(checked(context, file.stat())?.size, 0);

        context.expect_eq(file.write_at(0, b"kept"), Ok(4));
        let mut bytes = [0; 4];
        context.expect_eq(file.read_at(0, &mut bytes), Ok(4));
        context.expect_eq(&bytes, b"kept");
        Ok(())
    }
}
