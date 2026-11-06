//! 内存与伪文件系统（`tmpfs`、`devfs`、`procfs`）内核集成测试套件。
//!
//! # 模块定位与职责
//! 本模块位于文件系统测试子系统（`fs::ktest`）中，负责验证挂载在磁盘根文件系统之上的内存文件系统（`tmpfs`）、设备文件系统（`devfs`）与进程/内核信息伪文件系统（`procfs`）的功能正确性与资源配额管理。
//! 上游由 `ktest` 框架通过 `anchors()` 收集并执行；下游复用 `PathOwner` 夹具管理只读磁盘根与 VFS 实例，并调用 `MemoryFs`、`devfs`、`procfs`、`Vfs`、`FileTable` 以及虚拟内存（`Mm`）接口。
//!
//! # 核心机制与挂载拓扑
//! 测试在只读（`ReadOnly`）ext4 磁盘根目录 `/plain` 上挂载可写（`ReadWrite`）的 `tmpfs`，并在其内部创建挂载点以挂载只读的 `devfs` 与 `procfs`：
//!
//! ```text
//!   / (ext4 磁盘根, ReadOnly)
//!   ├── /pattern (磁盘只读文件，写入被拒绝)
//!   └── /plain   (挂载点 ──► tmpfs, ReadWrite, 独立 device 号与页/inode 配额)
//!       ├── /plain/dev  (挂载点 ──► devfs, ReadOnly)
//!       │   ├── null    (字符设备 rdev = 0x103, 写吞没 / 读立即 EOF)
//!       │   └── zero    (字符设备 rdev = 0x105, 写吞没 / 读全零填充)
//!       └── /plain/proc (挂载点 ──► procfs, ReadOnly, 每次 open 生成独立动态快照)
//!           ├── mounts  (VFS 挂载表 6 列文本，特殊字符八进制转义)
//!           ├── meminfo (物理内存统计 kB 文本)
//!           ├── uptime  (系统运行与空闲时间文本)
//!           └── cmdline (内核启动参数快照 + '\n')
//! ```
//!
//! 关键语义契约：
//! - **`tmpfs` 页与 Inode 生命周期**：文件内容存储在内存页缓存中，清理未引用目录项（`prune_unused`）不会丢失未删除文件的内存数据；仅当文件被 `unlink`、所有打开文件对象（`FileRef`）关闭、且外部持有的文件页引用（`Page`）全部释放后，内存页与 inode 配额才真正归还。
//! - **`procfs` 打开时快照（Snapshot-on-Open）**：每次 `open` 动态读取当前 VFS 挂载表、物理内存或时钟状态并冻结在该打开文件描述（OFD）内部；后续挂载变更或系统状态更新仅对新打开的 `FileRef` 可见，旧 `FileRef`（及其 `dup` 共享句柄）始终读取打开时刻的快照。
//!
//! # 锁与资源回收契约
//! - 测试通过 `PathOwner` 夹具统一管理挂载树与块设备资源；测试内的额外子挂载点在用例内或夹具 `quiesce` 阶段按逆序卸载并清理缓存，确保页缓存与内存页在用例结束时完全归还。

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

/// 用户空间测试映射的起始虚拟地址。
const ADDRESS: usize = 0x4800_0000;

/// 用户空间测试页的可读写、不可执行权限配置。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 在测试夹具的 VFS 上下文中按跟随符号链接模式（`ResolveMode::Follow`）解析路径 `name`。
fn resolve(context: &TestContext<'_>, fixture: &PathOwner, name: &[u8]) -> Result<Path, TestError> {
    checked(
        context,
        fixture
            .vfs()
            .resolve(fixture.context(), name, ResolveMode::Follow),
    )
}

/// 初始化只读磁盘根文件系统，并根据 `options` 创建 `tmpfs` 实例以可读写模式挂载到 `/plain` 目录。
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

/// 在当前 VFS 上下文中以 `0o666` 权限创建并打开指定路径 `name` 的可读写文件。
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

/// 解析路径 `name` 并按指定的 `flags` 打开文件，返回内核态 I/O 上下文的 `FileRef`。
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

/// 在当前 VFS 上下文中以 `0o777` 权限创建目录 `name`，常用作后续文件系统的挂载点。
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

/// 根据 `file.stat()` 返回的文件大小读取完整内容，并断言随后继续读取返回 `0`（EOF）。
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

/// 解析并校验 `/proc/mounts` 快照文本：检查总行数、每行 6 列格式、以及 `mounted` 个 `tmpfs` 子挂载点的设备号与八进制转义路径。
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

/// 从 `/proc/meminfo` 格式的文本 `bytes` 中查找以 `name` 开头的行，解析并返回其以 `kB` 为单位的整数值。
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

/// 创建独立的用户地址空间 `Mm`，并在 `ADDRESS` 处映射一页匿名可读写内存，用于测试跨页边界的用户态拷贝与缺页错误。
fn address_space() -> Result<Mm, TestError> {
    let mm = Mm::try_new().map_err(|_| TestError("NoMemory: pseudo file MM"))?;
    mm.map_anonymous(ADDRESS, mm::PAGE_SIZE, RW, RW)
        .map_err(|_| TestError("pseudo file MM mapping failed"))?;
    Ok(mm)
}

/// 返回本模块测试套件的静态注册槽位数组供父模块汇总。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 1] {
    [filesystems::__ktest_anchor()]
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-pseudo"), resources("root"), timeout_ms = 90000)]
pub(crate) mod filesystems {
    use super::*;

    /// 测试目标：验证 `tmpfs`、`devfs` 与 `procfs` 挂载后的文件系统类型、独立设备号（`device`）、字符设备主次设备号（`rdev`）、目录遍历与只读保护行为。
    /// 场景步骤：
    /// 1. 在 `/plain` 挂载可读写 `tmpfs`，并在其下创建 `/plain/dev` 与 `/plain/proc` 分别以只读模式挂载 `devfs` 和 `procfs`。
    /// 2. 打开磁盘文件、`/plain/dev/null`、`/plain/dev/zero` 及 `/plain/proc/mounts`，校验各自的 `MemoryFsKind`、`rdev`（`0x103` 与 `0x105`）以及各挂载实例互不相同的 `device` 标识。
    /// 3. 关闭 `/plain/dev/null` 并执行 `prune_unused` 清理未引用缓存，重新打开后验证仍复用同一个固定设备 inode，且 `readdir` 能完整列出 `null` 和 `zero`。
    /// 4. 验证在只读挂载的 `devfs` 与 `procfs` 中创建文件或目录均返回 `Err(FsError::ReadOnly)`。
    #[test]
    #[ktest(tags("T01"))]
    fn real_mount_kinds_devices_and_fixed_names(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 挂载 tmpfs、devfs 与 procfs，构建多后端伪文件系统层次结构。
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

        // Step 2: 校验各文件系统类型、字符设备号 (rdev) 以及不同挂载实例间的设备号 (device) 隔离性。
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

        // Step 3: 验证 prune_unused 不会销毁 devfs 固定设备节点，且目录读取包含 null 与 zero。
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

        // Step 4: 验证只读挂载的 devfs 与 procfs 拒绝创建新文件或子目录。
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

    /// 测试目标：验证 `tmpfs` 内存文件的数据持久性、截断（`truncate`）缩容与零扩展、删除（`unlink`）后打开句柄与外部页面引用（`Page`）对物理页和 inode 配额回收的精确控制。
    /// 场景步骤：
    /// 1. 挂载限制为 2 个 inode 和 2 个数据页的 `tmpfs`，写入跨两页的数据，验证 `prune_unused` 清理目录项缓存后重新打开仍能读出完整数据且 inode 编号不变。
    /// 2. 将文件截断至 13 字节验证释放第 2 个数据页，再扩展至 100 字节验证新增区间全部读出 `0`。
    /// 3. 执行 `unlink` 删除路径，验证路径查找返回 `NotFound` 且硬链接数变为 `0`，但已打开的 `FileRef` 仍可继续读写。
    /// 4. 通过 `file_page(0)` 获取底层缓存页引用后关闭 `FileRef` 并执行 `prune_unused`，验证因外部仍持有页引用，该页与 inode 配额尚未释放且页内数据可读；丢弃该页引用并再次 `prune_unused` 后，已用页数归零且 inode 配额恢复可分配。
    #[test]
    #[ktest(tags("T02"))]
    fn ram_bytes_truncate_unlink_and_last_page_owner(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 创建跨页文件并写入数据，验证 prune_unused 后重新打开仍保留同一 inode 与完整字节。
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

        // Step 2: 验证 truncate 缩小跨越页边界时立即回收多余物理页，扩大文件大小时自动零填充尾部。
        checked(context, file.truncate(13))?;
        context.expect_eq(tmp.used_pages(), 1);
        checked(context, file.truncate(100))?;
        let mut tail = [0xa7; 100];
        context.expect_eq(checked(context, file.read_at(0, &mut tail))?, tail.len());
        context.expect_eq(&tail[..13], &[0x6d; 13]);
        context.expect_eq(&tail[13..], &[0; 87]);

        // Step 3: 删除目录项 (unlink)，验证名字不可见但已打开的 FileRef 仍可修改页内数据。
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

        // Step 4: 持有外部缓存页引用并关闭 FileRef，验证仅在外部 Page 引用也释放后才归还物理页与 inode 配额。
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

    /// 测试目标：验证 `procfs` 中 `/proc/mounts`、`/proc/meminfo` 与 `/proc/uptime` 在每次重新打开时生成反映最新系统状态的动态快照，同时已打开的 `FileRef` 保持打开时刻的快照不变。
    /// 场景步骤：
    /// 1. 打开 `/plain/proc/mounts` 获取初始挂载快照 `before`；挂载新 `tmpfs` 到 `/plain/change` 后再次打开获取新快照 `after`，验证 `after` 包含新挂载点而重读旧句柄仍返回 `before`；卸载后新打开的快照恢复为 `before`。
    /// 2. 验证 `procfs` 快照文件拒绝 `readdir`、`write` 与 `ioctl`，支持 `read_at` 与 `seek`，且 `clone` 的句柄共享文件游标而独立 `open` 的句柄拥有独立游标。
    /// 3. 打开 `/plain/proc/meminfo` 核对 `MemTotal:` 计算公式；分配并写入 64 页匿名用户内存后重新打开，验证 `MemFree:` 显著下降。
    /// 4. 打开 `/plain/proc/uptime`、睡眠 25ms 后再次打开，验证两次快照文本不同且包含 2 个空白分隔字段。
    #[test]
    #[ktest(tags("T03"))]
    fn proc_reopen_observes_mount_memory_and_time(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 挂载 procfs，验证新挂载与卸载反映在新打开的 /proc/mounts 快照中，而旧 OFD 快照保持不变。
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
        // Step 2: 验证 procfs 文件的非法操作错误码、随机读取 (read_at)、游标寻址 (seek) 以及共享/独立 OFD 游标语义。
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

        // Step 3: 验证 /proc/meminfo 的 MemTotal 计算与实际物理页分配后的 MemFree 递减。
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

        // Step 4: 验证睡眠等待后重新打开 /proc/uptime 可观测到递增的运行时间文本。
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

    /// 测试目标：验证 `/proc/cmdline` 快照精确保存内核启动参数（`bootargs`）加尾随换行符的完整字节，并支持分块顺序读取、`read_at` 随机读取与 `dup` 共享游标。
    /// 场景步骤：
    /// 1. 挂载 `procfs` 并以 `/plain` 为根构造局部 `FsContext`，打开 `/proc/cmdline`，验证其内容严格等于 `boot::info().bootargs()` 拼接 `'\n'`。
    /// 2. 将游标重置到开头，以 257 字节窗口循环分块读取直至 EOF，校验每块内容与预期一致。
    /// 3. 验证 `read_at` 读取尾部 13 字节不改变当前文件游标；验证通过 `FileTable::dup` 复制的描述符共享游标，而重新 `open` 的句柄游标独立为 0。
    #[test]
    #[ktest(tags("F19", "fs-proc-cmdline"))]
    fn cmdline_preserves_all_boot_arguments(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 挂载 procfs 并在作用域根下打开 /proc/cmdline，核对快照内容与 bootargs + '\n' 完全一致。
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

        // Step 2: 以 257 字节窗口分块顺序读取并校验 EOF 边界，随后验证 read_at 不修改当前文件游标。
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

        // Step 3: 验证 FileTable 内 dup 描述符共享文件偏移量，而新打开的 OFD 游标从 0 独立起算。
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

    /// 测试目标：验证当包含需要八进制转义字符的深层挂载路径使 `/proc/mounts` 快照超过 64 KiB 时，快照能够动态扩容并完整保存所有记录，且旧 OFD 与卸载后的新 OFD 各自保持独立快照。
    /// 场景步骤：
    /// 1. 在 `/plain` 下连续创建 12 层由 255 个 `'#'` 字符组成的子目录（原始路径长度小于 `PATH_MAX = 4096`，但每个 `'#'` 在 `mounts` 中转义为 4 字节 `\043`），并在最深层创建 8 个挂载点 `m0`..`m7` 分别挂载独立 `tmpfs`。
    /// 2. 打开 `/plain/proc/mounts`，验证生成的快照总长度超过 65536 字节，且 8 个 `tmpfs` 的设备号与转义路径全部准确记录。
    /// 3. 在 64 KiB 边界附近测试 `seek`、`read`、`read_at`、`dup` 共享游标以及 `SeekFrom::End` 负偏移与超出末尾寻址。
    /// 4. 卸载第 8 个挂载点 `m7` 后重新打开 `/plain/proc/mounts`，验证新快照缩减为 7 个子挂载点且长度变短，而此前打开的旧句柄仍完整保留 8 个挂载点的原快照。
    #[test]
    #[ktest(tags("F23", "fs-proc-growth"))]
    fn proc_large_mount_snapshot_keeps_bytes_and_open_description(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 准备深层含 '#' 字符的目录树并挂载 8 个 tmpfs，使八进制转义后的挂载表文本超过 64 KiB。
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
        // 12 层各 255 个 '#' 字符的目录名在 /proc/mounts 中会被八进制转义为 "\043"，使 8 条挂载记录总长度突破 64 KiB，同时原始路径仍小于 4096 字节。
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

        // Step 2: 打开 /proc/mounts 验证超过 64 KiB 的快照内容完整，并在 64 KiB 边界处校验 seek、read 与 read_at。
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

        // Step 3: 验证大快照下的 dup 游标共享、独立 open 游标隔离以及相对于文件末尾 (SeekFrom::End) 的寻址行为。
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

        // Step 4: 卸载最后一个子挂载点，验证新打开的快照移除该记录，而旧 OFD 仍保留卸载前的完整快照。
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

    /// 测试目标：验证只读磁盘根与可读写 `tmpfs` 的写权限隔离，以及 `/dev/null` 和 `/dev/zero` 字符设备的数据读写语义与跨页用户缓冲区（`ReadBuffer`/`WriteBuffer`）部分传输行为。
    /// 场景步骤：
    /// 1. 验证以可写标志打开只读磁盘根下的 `/pattern` 返回 `Err(FsError::ReadOnly)`，而在挂载于 `/plain` 的 `tmpfs` 中可正常创建并写入文件。
    /// 2. 挂载 `devfs` 并打开 `/plain/dev/null` 与 `/plain/dev/zero`，验证 `null` 吞没写入且读取返回 `0`、`zero` 读取填充全 `0`，以及两者的 `seek`（恒返回 `0`）、非法 `ioctl`/`readdir`/`truncate` 错误码、状态标志更新和读写打开模式检查。
    /// 3. 构造仅映射单页的用户地址空间 `Mm`，测试从页尾最后 17 字节发起的 64 字节跨页 `WriteBuffer::user` 与 `ReadBuffer::user`，验证 `null` 和 `zero` 均准确消费前 17 字节有效前缀；验证起始地址即未映射时 `write_at_from` 与 `zero.read_at_into` 返回 `BadAddress`，而 `null.read_into` 直接返回 `0`。
    #[test]
    #[ktest(tags("T04"))]
    fn readonly_root_device_data_and_user_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 验证只读磁盘根拒绝可写打开，而子目录挂载的 RW tmpfs 允许创建与写入。
        let _tmp = prepare_tmp(context, fixture, b"mode=0777")?;
        let disk = resolve(context, fixture, b"/pattern")?;
        context.expect_eq(
            open_path_with_flags(fixture.vfs(), disk, 2, FileIoContext::Kernel).err(),
            Some(FsError::ReadOnly),
        );
        let data = create(context, fixture, b"/plain/rw")?;
        context.expect_eq(checked(context, data.write(b"ram"))?, 3);

        // Step 2: 挂载 devfs，验证 /dev/null 与 /dev/zero 的基础读写、seek、状态标志及访问模式限制。
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

        // Step 3: 使用单页用户地址空间测试跨页边界的 WriteBuffer/ReadBuffer，验证有效前缀传输与未映射地址错误处理。
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

    /// 测试目标：验证 `tmpfs` 的最大 inode 数（`nr_inodes`）与最大数据页数（`nr_blocks`）配额上限在耗尽时返回 `Err(FsError::NoSpace)`，且在删除文件或截断释放配额后可立即复用。
    /// 场景步骤：
    /// 1. 当 `quota == 0`（测试 inode 配额）时：在 `nr_inodes=128`（含根目录共 128 个）的 `tmpfs` 中连续创建 127 个文件，验证第 128 个文件创建返回 `NoSpace`；删除 `/plain/000` 并清理缓存后，验证可成功创建新文件并正常读写。
    /// 2. 当 `quota == 1`（测试数据页配额）时：在 `nr_blocks=4096` 的 `tmpfs` 中向单个文件连续写入 4096 页，验证继续写入第 4097 页返回 `NoSpace` 且原文件大小与尾部数据保持不变；将文件 `truncate(0)` 释放全部数据页后，验证可重新写入数据。
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
        // 分支 1: 验证 inode 配额耗尽时拒绝创建新文件，unlink 释放后可重新创建。
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
        // 分支 2: 验证数据页配额 (nr_blocks) 耗尽时拒绝扩展写入，truncate(0) 释放后可重新写入。
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

    /// 测试目标：验证当系统物理页与内核堆同时耗尽（OOM）导致 `tmpfs` 分配新页面或插入页缓存失败时，未发布的候选页与配额记账被完整回滚，文件大小保持为 `0`；内存压力解除后同一文件可正常写入。
    /// 场景步骤：
    /// 1. 在 `tmpfs` 中创建空文件 `/plain/oom`。
    /// 2. 使用 `HeapPressure` 耗尽物理页与堆内存后调用 `file.write_at(0, b"lost")`，验证返回 `Err(FsError::NoMemory)`。
    /// 3. 释放内存压力后，验证 `tmp.used_pages()` 仍为 `0` 且文件大小仍为 `0`（无残留配额泄漏），随后再次写入 `"kept"` 并读回校验成功。
    #[test]
    #[ktest(tags("T02", "fs-oom"))]
    fn ram_unpublished_page_oom_keeps_file_and_charge(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 准备 tmpfs 空文件，并施加全局物理页与堆内存耗尽压力。
        let tmp = prepare_tmp(context, fixture, b"mode=0777")?;
        let file = create(context, fixture, b"/plain/oom")?;

        let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let probes = pressure.exhaust();
        let result = file.write_at(0, b"lost");
        drop(pressure);
        // Step 2: 验证 OOM 失败未留下残留页计数或文件长度，且内存恢复后可正常写入。
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
