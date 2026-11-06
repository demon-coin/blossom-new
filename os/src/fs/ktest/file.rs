//! 打开文件描述（Open File Description）、动态扩容文件描述符表与文件页映射（mmap）的内核集成测试。
//!
//! ## 模块职责与架构定位
//! 本模块位于文件系统内核测试层（`fs::ktest::file`），向上由 `fs::ktest` 注册到 `ktest` 运行器，
//! 向下验证 `FileTable`（描述符分配、预留槽位、复制与并发关闭）、VFS 打开文件对象（独立/共享游标、
//! 普通文件与目录遍历、挂载点忙引用保护）以及 `Mm::map_file` 与页缓存（Page Cache）的协同行为。
//!
//! ## 核心机制与状态流转
//!
//! ```text
//!   [FileTable]
//!     |-- fd 0 (Reserved) --publish()--> [Arc<File> (OFD A, cursor)]
//!     |-- fd 1 (dup)      -------------> [Arc<File> (OFD A, shared cursor)]
//!     +-- fd 2 (open)     -------------> [Arc<File> (OFD B, independent cursor)]
//!                                               |
//!                                               v
//!                                        [VFS Inode / Ext4Volume]
//!                                               |
//!                                               v
//!                                        [Page Cache Folio] <--- [Mm::map_file / COW]
//! ```
//!
//! - **描述符槽位状态机**：槽位经历 `Empty -> Reserved -> Open(Arc<File>) -> Empty`；`Reserved` 期间
//!   对 `lookup`/`close` 不可见，但阻止 `dup_exact` 覆盖（返回 `Busy`），丢弃未发布的 `Reservation` 自动回退为空槽。
//! - **异步 I/O 期间的引用保活**：当读取或页错误（Page Fault）在块设备段钩子（`segment_hook`）处挂起时，
//!   操作路径持有独立的 `Arc<File>` 或 `Inode` 引用且不持有 `FileTable` 或 `Mm` 锁；此时并发关闭 `fd`、
//!   复用同编号打开新文件或 `unmap` 重映射均不会被阻塞，也不会破坏正在进行的旧读取。
//!
//! ## 锁获取顺序与并发契约
//! - `FileTable` 内部锁仅在槽位查询、预留、发布和关闭时短暂获取，绝不跨越底层块 I/O 或页缓存加载。
//! - 用户地址空间缺页处理（`Mm::fault`）在触发底层磁盘读块前释放地址空间锁，读盘完成后重新校验虚拟地址映射是否已被并发 `unmap` 或替换。

use crate::arch::{self, UserAccess, UserContext, UserTrap};
use crate::fs::ktest::path::PathOwner;
use crate::fs::ktest::*;
use crate::fs::{DescriptorFlags, FileTable, OpenOptions, SeekFrom};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;
use crate::task::UserThread;

/// 文件与描述符表测试夹具，封装底层路径/块设备夹具 `PathOwner`、被测 `FileTable` 及页加载钩子。
struct FileOwner {
    /// 底层路径、VFS、Ext4 卷与后台任务管理夹具。
    path: PathOwner,
    /// 当前用例使用的文件描述符表实例。
    table: Option<Arc<FileTable>>,
    /// 安装了块段拦截钩子（`segment_hook`）的 Ext4 卷引用，用于在清理阶段卸载钩子。
    hook_volume: Option<Arc<Ext4Volume>>,
    /// 当前用例已发布的用户态访问子任务槽位索引。
    user: Option<usize>,
    /// 当前用例允许分配的最大文件描述符数量上限。
    limit: usize,
}

impl FileOwner {
    /// 初始化只读根文件系统路径环境，创建新的 `FileTable`，并记录本例的描述符配额上限 `limit`。
    fn prepare(&mut self, context: &TestContext<'_>, limit: usize) -> TestResult {
        self.path.prepare_path(context, AccessMode::ReadOnly, 4)?;
        self.limit = limit;
        let table = Arc::try_new(checked(context, FileTable::try_new())?)
            .map_err(|_| TestError("file table owner OOM"))?;
        self.table = Some(table);
        Ok(())
    }

    /// 获取当前用例 `FileTable` 的强引用借用。
    fn table(&self) -> &Arc<FileTable> {
        self.table
            .as_ref()
            .expect("file table consumed before body")
    }

    /// 使用当前用例的 VFS 实例、路径上下文和描述符上限 `self.limit` 打开字节路径 `pathname`。
    fn open(
        &self,
        context: &TestContext<'_>,
        pathname: &[u8],
        options: OpenOptions,
    ) -> Result<usize, TestError> {
        checked(
            context,
            self.table().open(
                self.path.vfs(),
                self.path.context(),
                pathname,
                options,
                self.limit,
            ),
        )
    }

    /// 在 `/pattern` 所在 Ext4 卷上安装块段拦截钩子：当首次加载第 1 号文件页的首个块时递增计数并在门闸上阻塞等待。
    fn hold_first_page(&mut self, context: &TestContext<'_>) -> TestResult {
        let path = checked(
            context,
            self.path
                .vfs()
                .resolve(self.path.context(), b"/pattern", ResolveMode::Follow),
        )?;
        let volume = checked(context, checked(context, path.inode())?.disk_volume())?.clone();
        let observer = self.path.owner.observer().clone();
        let block_size = volume.geometry().block_size;
        let hook: Arc<dyn Fn(u64, usize) + Send + Sync> = Arc::try_new(move |page, bytes| {
            if page == 1 && bytes == block_size && observer.hold_loader.load(Ordering::Acquire) {
                observer.loader_entered.fetch_add(1, Ordering::AcqRel);
                observer.gate.wait();
            }
        })
        .map_err(|_| TestError("file page observer OOM"))?;
        self.path.vfs().dcache().ktest_set_lookup_hook(None);
        volume.ktest_set_segment_hook(Some(hook));
        self.hook_volume = Some(volume);
        self.path
            .owner
            .observer()
            .hold_loader
            .store(true, Ordering::Release);
        Ok(())
    }
}

impl Fixture<DeviceInventory> for FileOwner {
    /// 用例初始化：委托 `PathOwner::set_up` 初始化块设备与任务槽位，表与卷钩子延后到用例正文按需配置。
    fn set_up(
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        Ok(Self {
            path: <PathOwner as Fixture<DeviceInventory>>::set_up(context, inventory)?,
            table: None,
            hook_volume: None,
            user: None,
            limit: 0,
        })
    }

    /// 用例静止化：移除 Ext4 卷的块段拦截钩子、放行等待门闸、释放 `FileTable`，并交由 `PathOwner` 等待所有子任务与块设备静止。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if let Some(volume) = &self.hook_volume {
            volume.ktest_set_segment_hook(None);
        }
        self.path
            .owner
            .observer()
            .hold_loader
            .store(false, Ordering::Release);
        self.path.owner.observer().gate.complete();
        self.user = None;
        drop(self.table.take());
        drop(self.hook_volume.take());
        <PathOwner as Fixture<DeviceInventory>>::quiesce(&mut self.path, context)
    }

    /// 用例拆解：确保本层持有的表与卷引用全部释放后，交由 `PathOwner::tear_down` 校验内存与块设备资源回收。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let Self {
            path,
            table,
            hook_volume,
            user: _,
            limit: _,
        } = self;
        drop(table);
        drop(hook_volume);
        <PathOwner as Fixture<DeviceInventory>>::tear_down(path, context)
    }
}

/// 从文件描述符 `fd` 读取 `len` 字节（不超过一页），断言实际读取长度等于 `len` 且字节内容与 `/pattern` 在 `offset` 处的预期图样完全一致。
fn read_pattern(
    context: &mut TestContext<'_>,
    table: &FileTable,
    fd: usize,
    offset: u64,
    len: usize,
) -> TestResult {
    let mut bytes = [0xa7; mm::PAGE_SIZE];
    context.expect_eq(checked(context, table.read(fd, &mut bytes[..len]))?, len);
    context.expect_eq(patterned(&bytes[..len], offset), true);
    Ok(())
}

/// 返回本文件内全部 9 个子测试套件的静态注册锚点数组。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 9] {
    [
        regular::__ktest_anchor(),
        directories::__ktest_anchor(),
        slots::__ktest_anchor(),
        racing::__ktest_anchor(),
        prefix::__ktest_anchor(),
        mounts::__ktest_anchor(),
        oom::__ktest_anchor(),
        vm_lifetime::__ktest_anchor(),
        vm_racing::__ktest_anchor(),
    ]
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-file", "fs-file-regular"), resources("root"), timeout_ms = 90000)]
pub(crate) mod regular {
    use super::*;

    /// 测试目标：验证独立打开、硬链接打开与 `dup` 复制描述符的游标隔离/共享语义，以及 `read_at`、`seek` 边界、只读拒绝写入和 `stat` 元数据一致性。
    /// 场景步骤：
    /// 1. 分别打开 `/pattern` 两次（其中首次带 `close_on_exec`）并打开其硬链接 `/hardlink`，比较各自的 `Arc<File>` 指针与 `stat` 快照。
    /// 2. 分别读取不同长度后对首个描述符执行 `dup`，验证 `dup` 出的新描述符清除 `close_on_exec` 且与原描述符共享读写游标。
    /// 3. 调用 `read_at` 跨页读取并检查游标未变；测试负向 `seek`、`SeekFrom::End`、溢出 `seek`（`i64::MIN`）、只读文件 `write` 和普通文件 `readdir`。
    /// 4. 将游标重置到文件开头，按页循环顺序读完整个 `/pattern` 文件，最后检查另外两个独立打开描述符的游标保持原位。
    /// 预期结果：独立 `open` 拥有各自独立的 `File` 实例与游标，`dup` 共享同一 `File` 实例与游标，`read_at` 不推进游标，非法操作返回对应 `FsError`。
    #[test]
    #[ktest(tags("B20", "B21", "B22"))]
    fn independent_shared_cursor_and_bytes(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        // Step 1: 独立打开同一文件及硬链接，验证生成不同的打开文件对象但共享相同的 Inode 元数据。
        fixture.prepare(context, 8)?;
        let fd = fixture.open(
            context,
            b"/pattern",
            OpenOptions {
                close_on_exec: true,
                ..OpenOptions::default()
            },
        )?;
        let independent = fixture.open(context, b"/pattern", OpenOptions::default())?;
        let hard = fixture.open(context, b"/hardlink", OpenOptions::default())?;
        let table = fixture.table();
        let first = checked(context, table.lookup(fd))?;
        let second = checked(context, table.lookup(independent))?;
        let hard_file = checked(context, table.lookup(hard))?;
        context.expect_eq(Arc::ptr_eq(&first, &second), false);
        context.expect_eq(Arc::ptr_eq(&first, &hard_file), false);
        let stat = checked(context, table.stat(fd))?;
        context.expect_eq(checked(context, table.stat(hard))?, stat);
        let path = checked(
            context,
            fixture
                .path
                .vfs()
                .resolve(fixture.path.context(), b"/pattern", ResolveMode::Follow),
        )?;
        let inode = checked(context, path.inode())?;
        let data = checked(context, inode.metadata_snapshot())?;
        context.expect_eq(
            (
                stat.inode,
                stat.kind,
                stat.mode,
                stat.size,
                stat.uid,
                stat.gid,
                stat.links,
                stat.blocks,
            ),
            (
                inode.number(),
                data.kind,
                data.mode,
                data.size,
                data.uid,
                data.gid,
                data.links,
                data.blocks,
            ),
        );
        context.expect_eq(stat.size, PATTERN_SIZE);

        // Step 2: 验证 dup 复制的描述符清除 close_on_exec 标志并与原描述符共享读写游标。
        read_pattern(context, table, fd, 0, 17)?;
        read_pattern(context, table, independent, 0, 11)?;
        read_pattern(context, table, hard, 0, 7)?;
        let duplicate = checked(context, table.dup(fd, fixture.limit))?;
        context.expect_eq(
            Arc::ptr_eq(&first, &checked(context, table.lookup(duplicate))?),
            true,
        );
        context.expect_eq(checked(context, table.flags(fd))?.close_on_exec, true);
        context.expect_eq(
            checked(context, table.flags(duplicate))?.close_on_exec,
            false,
        );
        read_pattern(context, table, duplicate, 17, 19)?;
        context.expect_eq(checked(context, table.seek(fd, SeekFrom::Current(0)))?, 36);

        // Step 3: 验证定位读取 read_at 不改变当前游标，并检查 seek 边界及非法写入/目录读取错误码。
        let mut bytes = [0xa7; 29];
        context.expect_eq(
            checked(context, table.read_at(fd, 4091, &mut bytes))?,
            bytes.len(),
        );
        context.expect_eq(patterned(&bytes, 4091), true);
        context.expect_eq(
            checked(context, table.seek(duplicate, SeekFrom::Current(0)))?,
            36,
        );

        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(-16)))?,
            20,
        );
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::End(-5)))?,
            PATTERN_SIZE - 5,
        );
        read_pattern(context, table, duplicate, PATTERN_SIZE - 5, 5)?;
        context.expect_eq(checked(context, table.read(fd, &mut bytes))?, 0);
        context.expect_eq(checked(context, table.read(fd, &mut []))?, 0);
        context.expect_eq(
            table.seek(fd, SeekFrom::Current(i64::MIN)).err(),
            Some(FsError::InvalidOperation),
        );
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(0)))?,
            PATTERN_SIZE,
        );
        context.expect_eq(table.write(fd, b"write").err(), Some(FsError::BadFd));
        context.expect_eq(table.write(fd, b"").err(), Some(FsError::BadFd));
        context.expect_eq(table.readdir(fd).err(), Some(FsError::NotDirectory));

        // Step 4: 从头按页完整读取整个文件，确认其他独立打开的描述符游标不受影响。
        checked(context, table.seek(fd, SeekFrom::Start(0)))?;
        let mut offset = 0;
        while offset < PATTERN_SIZE {
            let len = (PATTERN_SIZE - offset).min(mm::PAGE_SIZE as u64) as usize;
            read_pattern(context, table, fd, offset, len)?;
            offset += len as u64;
        }
        context.expect_eq(
            checked(context, table.seek(independent, SeekFrom::Current(0)))?,
            11,
        );
        context.expect_eq(checked(context, table.seek(hard, SeekFrom::Current(0)))?, 7);
        Ok(())
    }
}

/// 遍历目录描述符 `fd` 的全部目录项，验证每次读取后当前游标等于 `next_cookie`，且 `.`、`..` 与全部预期子项名称、编号及文件类型无遗漏无重复。
fn enumerate_names(
    context: &mut TestContext<'_>,
    table: &FileTable,
    fd: usize,
    indexed: bool,
) -> TestResult {
    let count = if indexed { DIRECTORY_ENTRIES } else { 40 };
    let mut seen = [false; DIRECTORY_ENTRIES];
    let mut dots = 0;
    while let Some(entry) = checked(context, table.readdir(fd))? {
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(0)))?,
            entry.next_cookie,
        );
        if entry.name == b"." || entry.name == b".." {
            dots += 1;
            continue;
        }
        let digits = if indexed { 5 } else { 4 };
        let len = if indexed { 191 } else { 9 };
        if entry.name.len() != len || &entry.name[..5] != b"item-" {
            return Err(TestError("file directory unexpected byte name"));
        }
        if indexed && (entry.name[10] != b'-' || !entry.name[11..].iter().all(|byte| *byte == b'n'))
        {
            return Err(TestError("file indexed directory name tail changed"));
        }
        let mut index = 0;
        for digit in &entry.name[5..5 + digits] {
            if !digit.is_ascii_digit() {
                return Err(TestError("file directory name digit invalid"));
            }
            index = index * 10 + usize::from(*digit - b'0');
        }
        if count <= index || seen[index] {
            return Err(TestError("file directory duplicate or out of range name"));
        }
        seen[index] = true;
        context.expect_eq(entry.kind, Some(InodeKind::Regular));
    }
    context.expect_eq(dots, 2);
    context.expect_eq(seen[..count].iter().all(|seen| *seen), true);
    context.expect_eq(checked(context, table.readdir(fd))?, None);
    Ok(())
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-file", "fs-file-directory"), resources("root"), timeout_ms = 90000)]
pub(crate) mod directories {
    use super::*;

    /// 测试目标：验证普通线性目录与 HTree 索引目录的完整遍历、`dup` 共享目录游标、`seek` 定位重读、原始非 UTF-8 字节文件名以及符号链接跟随与类型检查。
    /// 场景步骤：
    /// 1. 依次打开线性目录 `/plain` 与索引目录 `/indexed`，通过 `dup` 和独立打开交替调用 `readdir` 与 `seek(SeekFrom::Start(cookie))`，验证目录项重读与 `SeekFrom::End` 行为，随后调用 `enumerate_names` 校验全部目录项。
    /// 2. 对目录描述符尝试调用 `read`、`read_at` 和 `write`。
    /// 3. 遍历根目录 `/` 查找包含非 UTF-8 字节的 `name-\xff\xfe`，并测试 `/fastlink` 符号链接在默认跟随、`no_follow`、`directory` 组合选项下的打开行为。
    /// 预期结果：两类目录的所有条目及 cookie 游标均精确匹配；目录读写返回 `IsDirectory`/`BadFd`；符号链接与非目录在 `no_follow`/`directory` 约束下分别返回 `TooManySymlinks` 和 `NotDirectory`。
    #[test]
    #[ktest(tags("B20", "B21", "B22"))]
    fn entries_cookie_types_and_nofollow(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        fixture.prepare(context, 8)?;
        // Step 1: 分别验证普通目录与 HTree 索引目录的 cookie 定位、游标共享、完整遍历及目录非法读写拦截。
        for (pathname, indexed) in [
            (b"/plain".as_slice(), false),
            (b"/indexed".as_slice(), true),
        ] {
            let fd = fixture.open(
                context,
                pathname,
                OpenOptions {
                    directory: true,
                    ..OpenOptions::default()
                },
            )?;
            let independent = fixture.open(context, pathname, OpenOptions::default())?;
            let table = fixture.table();
            let duplicate = checked(context, table.dup(fd, fixture.limit))?;
            let first = checked(context, table.readdir(fd))?
                .ok_or(TestError("directory first entry absent"))?;
            let reread_cookie = first.next_cookie;
            context.expect_eq(checked(context, table.readdir(independent))?, Some(first));
            let next = checked(context, table.readdir(duplicate))?
                .ok_or(TestError("directory second entry absent"))?;
            let position_cookie = next.next_cookie;
            checked(
                context,
                table.seek(independent, SeekFrom::Start(reread_cookie)),
            )?;
            context.expect_eq(checked(context, table.readdir(independent))?, Some(next));
            context.expect_eq(
                checked(context, table.seek(fd, SeekFrom::Current(0)))?,
                checked(context, table.seek(duplicate, SeekFrom::Current(0)))?,
            );
            context.expect_eq(
                table.seek(fd, SeekFrom::Start(i64::MAX as u64 + 1)).err(),
                Some(FsError::InvalidOperation),
            );
            context.expect_eq(
                checked(context, table.seek(fd, SeekFrom::Current(0)))?,
                position_cookie,
            );
            let end_cookie = checked(context, table.seek(fd, SeekFrom::End(0)))?;
            context.expect_eq(checked(context, table.readdir(fd))?, None);
            context.expect_eq(
                checked(context, table.seek(fd, SeekFrom::Current(0)))?,
                end_cookie,
            );
            context.expect_eq(
                table.seek(fd, SeekFrom::End(i64::MIN)).err(),
                Some(FsError::InvalidOperation),
            );
            context.expect_eq(
                checked(context, table.seek(fd, SeekFrom::Current(0)))?,
                end_cookie,
            );

            checked(context, table.seek(fd, SeekFrom::Start(0)))?;
            enumerate_names(context, table, duplicate, indexed)?;
            let mut byte = [0xa7; 1];
            context.expect_eq(table.read(fd, &mut byte).err(), Some(FsError::IsDirectory));
            context.expect_eq(
                table.read_at(fd, 0, &mut byte).err(),
                Some(FsError::IsDirectory),
            );
            context.expect_eq(table.write(fd, b"x").err(), Some(FsError::BadFd));
            for fd in [fd, independent, duplicate] {
                checked(context, table.close(fd))?;
            }
        }

        // Step 2: 验证根目录下的非 UTF-8 原始字节文件名可被正常遍历与打开读取。
        let fd = fixture.open(context, b"/", OpenOptions::default())?;
        let table = fixture.table();
        let mut raw = 0;
        while let Some(entry) = checked(context, table.readdir(fd))? {
            if entry.name == b"name-\xff\xfe" {
                context.expect_eq(entry.kind, Some(InodeKind::Regular));
                raw += 1;
            }
        }
        context.expect_eq(raw, 1);

        // Step 3: 验证符号链接跟随读取，以及 no_follow 和 directory 打开标志的错误码检查。
        let followed = fixture.open(context, b"/fastlink", OpenOptions::default())?;
        read_pattern(context, table, followed, 0, 31)?;
        context.expect_eq(
            table
                .open(
                    fixture.path.vfs(),
                    fixture.path.context(),
                    b"/fastlink",
                    OpenOptions {
                        no_follow: true,
                        ..OpenOptions::default()
                    },
                    fixture.limit,
                )
                .err(),
            Some(FsError::TooManySymlinks),
        );
        context.expect_eq(
            table
                .open(
                    fixture.path.vfs(),
                    fixture.path.context(),
                    b"/pattern",
                    OpenOptions {
                        directory: true,
                        ..OpenOptions::default()
                    },
                    fixture.limit,
                )
                .err(),
            Some(FsError::NotDirectory),
        );
        context.expect_eq(
            table
                .open(
                    fixture.path.vfs(),
                    fixture.path.context(),
                    b"/fastlink",
                    OpenOptions {
                        directory: true,
                        no_follow: true,
                        ..OpenOptions::default()
                    },
                    fixture.limit,
                )
                .err(),
            Some(FsError::NotDirectory),
        );

        let raw_fd = fixture.open(context, b"/name-\xff\xfe", OpenOptions::default())?;
        let mut bytes = [0xa7; 9];
        context.expect_eq(checked(context, table.read(raw_fd, &mut bytes))?, 9);
        context.expect_eq(bytes, *b"raw-name\n");
        Ok(())
    }
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-file", "fs-file-slots"), resources("root"), timeout_ms = 30000)]
pub(crate) mod slots {
    use super::*;

    /// 测试目标：验证文件描述符表的槽位预留（`Reservation`）、配额上限（`limit`）、关闭后最小编号复用、大编号扩容（跨 1024 至 4096）、表克隆隔离以及 `close_on_exec` 清理。
    /// 场景步骤：
    /// 1. 验证配额为 0 时 `reserve(0)` 返回 `TooManyFiles`；在配额为 2 的表中打开 `fd = 0` 并预留槽位 `1`，验证未发布槽位对 `lookup`/`flags`/`close` 返回 `BadFd`、对 `dup_exact` 返回 `Busy`，且超出配额的新操作返回 `TooManyFiles`。
    /// 2. 丢弃预留槽位后重新预留并 `publish`，验证关闭、复用及 `dup_exact` 覆盖已打开描述符的行为（含清除目标的 `close_on_exec`）。
    /// 3. 创建大配额文件表（4097），测试 `dup_exact` 到 `299`、`1023`、`1024`、`1025`、`4096` 等跨段编号。
    /// 4. 在存在未发布预留槽位时调用 `try_clone` 克隆子表，验证预留槽位不泄漏到子表；在父表设置 `4096` 为 `close_on_exec` 并执行 `close_on_exec()`，验证父表关闭而子表保留，最后测试 `dup_min`。
    /// 预期结果：所有槽位状态转换、配额拦截、跨页表扩容、父子表隔离及 `close_on_exec` 均符合 POSIX 描述符语义。
    #[test]
    #[ktest(tags("B20", "B21"))]
    fn reservation_capacity_reuse_and_wide_fd(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        // Step 1: 验证槽位预留期间的不可见性、Busy 冲突保护以及描述符配额上限拦截。
        context.expect_eq(
            checked(context, FileTable::try_new())?.reserve(0).err(),
            Some(FsError::TooManyFiles),
        );
        fixture.prepare(context, 2)?;
        let fd = fixture.open(
            context,
            b"/pattern",
            OpenOptions {
                close_on_exec: true,
                ..OpenOptions::default()
            },
        )?;
        let table = fixture.table();
        context.expect_eq(2 <= table.allocated_slots(), true);
        let reservation = checked(context, table.reserve(fixture.limit))?;
        context.expect_eq(reservation.fd(), 1);
        context.expect_eq(table.lookup(1).err(), Some(FsError::BadFd));
        context.expect_eq(table.flags(1).err(), Some(FsError::BadFd));
        context.expect_eq(table.close(1).err(), Some(FsError::BadFd));
        context.expect_eq(
            table.dup_exact(fd, 1, fixture.limit).err(),
            Some(FsError::Busy),
        );
        context.expect_eq(
            table.reserve(fixture.limit).err(),
            Some(FsError::TooManyFiles),
        );
        context.expect_eq(
            table.dup(fd, fixture.limit).err(),
            Some(FsError::TooManyFiles),
        );
        context.expect_eq(
            table
                .open(
                    fixture.path.vfs(),
                    fixture.path.context(),
                    b"/empty",
                    OpenOptions::default(),
                    fixture.limit,
                )
                .err(),
            Some(FsError::TooManyFiles),
        );

        // Step 2: 验证丢弃预留后槽位自动回收、正式发布、关闭复用以及 dup_exact 覆盖语义。
        drop(reservation);
        let reservation = checked(context, table.reserve(fixture.limit))?;
        context.expect_eq(reservation.fd(), 1);
        let file = checked(context, table.lookup(fd))?;
        let second = reservation.publish(file.clone(), DescriptorFlags::default());
        read_pattern(context, table, second, 0, 23)?;
        context.expect_eq(checked(context, table.seek(fd, SeekFrom::Current(0)))?, 23);
        context.expect_eq(
            checked(context, table.dup_exact(fd, fd, fixture.limit))?,
            fd,
        );
        context.expect_eq(checked(context, table.flags(fd))?.close_on_exec, true);
        context.expect_eq(table.lookup(usize::MAX).err(), Some(FsError::BadFd));
        context.expect_eq(
            table.dup_exact(fd, 2, fixture.limit).err(),
            Some(FsError::BadFd),
        );

        checked(context, table.close(second))?;
        context.expect_eq(table.close(second).err(), Some(FsError::BadFd));
        context.expect_eq(checked(context, table.dup(fd, fixture.limit))?, second);
        checked(context, table.close(fd))?;
        context.expect_eq(table.read(fd, &mut [0; 1]).err(), Some(FsError::BadFd));
        context.expect_eq(
            table
                .open(
                    fixture.path.vfs(),
                    fixture.path.context(),
                    b"/missing",
                    OpenOptions::default(),
                    fixture.limit,
                )
                .err(),
            Some(FsError::NotFound),
        );
        context.expect_eq(
            fixture.open(context, b"/empty", OpenOptions::default())?,
            fd,
        );
        context.expect_eq(
            checked(context, table.dup_exact(second, fd, fixture.limit))?,
            fd,
        );
        context.expect_eq(
            Arc::ptr_eq(&file, &checked(context, table.lookup(fd))?),
            true,
        );
        context.expect_eq(checked(context, table.flags(fd))?.close_on_exec, false);

        // Step 3: 验证大编号描述符（299、1023..1025、4096）的动态表扩容与读写共享。
        let wide = checked(context, FileTable::try_new())?;
        let base = checked(context, wide.reserve(4097))?.publish(file, DescriptorFlags::default());
        context.expect_eq(checked(context, wide.dup_exact(base, 299, 4097))?, 299);
        read_pattern(context, &wide, 299, 23, 13)?;
        context.expect_eq(checked(context, wide.seek(base, SeekFrom::Current(0)))?, 36);
        checked(context, wide.close(299))?;
        context.expect_eq(wide.lookup(299).err(), Some(FsError::BadFd));
        for fd in [1023, 1024, 1025, 4096] {
            context.expect_eq(checked(context, wide.dup_exact(base, fd, 4097))?, fd);
            context.expect_eq(
                Arc::ptr_eq(
                    &checked(context, wide.lookup(base))?,
                    &checked(context, wide.lookup(fd))?,
                ),
                true,
            );
        }

        // Step 4: 验证克隆子表忽略父表未发布的预留槽位，以及 close_on_exec 和 dup_min 的行为。
        let reserved = checked(context, wide.reserve(4097))?;
        let child = checked(context, wide.try_clone())?;
        context.expect_eq(child.lookup(reserved.fd()).err(), Some(FsError::BadFd));
        drop(reserved);
        context.expect_eq(checked(context, child.dup(base, 4097))?, 1);
        checked(
            context,
            wide.set_flags(
                4096,
                DescriptorFlags {
                    close_on_exec: true,
                },
            ),
        )?;
        wide.close_on_exec();
        context.expect_eq(wide.lookup(4096).err(), Some(FsError::BadFd));
        context.expect_eq(child.lookup(4096).is_ok(), true);
        context.expect_eq(
            checked(
                context,
                wide.dup_min(base, 4096, DescriptorFlags::default(), 4097),
            )?,
            4096,
        );
        Ok(())
    }
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-file", "fs-file-race"), resources("root"), timeout_ms = 90000)]
pub(crate) mod racing {
    use super::*;

    /// 测试目标：验证当文件读取在磁盘块加载处挂起时，并发关闭原描述符、复用同一编号打开新文件、读取其他文件以及并发 `dup_exact` 均互不阻塞且数据隔离。
    /// 场景步骤：
    /// 1. 打开 `/pattern`（获得 `fd = 0`）并定位到第 1 页，启用 `hold_first_page` 钩子后在后台子任务 0 中发起整页读取，等待其阻塞在块段门闸上。
    /// 2. 主线程关闭 `fd = 0`，随后打开 `/name-\xff\xfe`，验证其立即复用编号 `fd = 0` 并成功读出 `"raw-name\n"`；再打开 `/deep` 发起磁盘读取，验证块设备请求数增加且正常完成。
    /// 3. 预留一个目标槽位 `target`，在子任务 1 中并发执行 `dup_exact(reused, target)` 验证返回 `Busy`；释放预留后在子任务 2 中再次执行 `dup_exact` 验证成功。
    /// 4. 放行块段门闸并等待子任务 0 完成，检查其读出的整页数据以及复用后 `reused` 描述符的游标位置。
    /// 预期结果：挂起的旧读取通过自身持有的 `Arc<File>` 完整读出 `/pattern` 的一页数据，复用同编号的新文件游标保持在 `9`，期间其他文件 I/O 与槽位操作均正常推进。
    #[test]
    #[ktest(tags("B20", "B21", "B22", "B23"))]
    fn read_close_reuse_progress_and_reserved_dup(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        // Step 1: 启动后台任务读取 /pattern 的第一页，并通过块段钩子将其挂起在 I/O 途中。
        fixture.prepare(context, 4)?;
        let fd = fixture.open(context, b"/pattern", OpenOptions::default())?;
        let table = fixture.table().clone();
        checked(
            context,
            table.seek(fd, SeekFrom::Start(mm::PAGE_SIZE as u64)),
        )?;
        fixture.hold_first_page(context)?;
        fixture.path.owner.publish(
            0,
            Job::FileRead {
                table: table.clone(),
                fd,
                len: mm::PAGE_SIZE,
                expected_offset: mm::PAGE_SIZE as u64,
            },
        );
        until("file read did not reach first segment gate", || {
            fixture
                .path
                .owner
                .observer()
                .loader_entered
                .load(Ordering::Acquire)
                == 1
                && fixture.path.owner.observer().blocked(0)
        })?;

        // Step 2: 在旧读取挂起期间关闭原 fd，验证同一 fd 编号可立即被新打开的文件复用且不影响其他文件磁盘 I/O。
        checked(context, table.close(fd))?;
        let reused = fixture.open(context, b"/name-\xff\xfe", OpenOptions::default())?;
        context.expect_eq(reused, fd);
        let mut bytes = [0xa7; 9];
        context.expect_eq(checked(context, table.read(reused, &mut bytes))?, 9);
        context.expect_eq(bytes, *b"raw-name\n");
        let other = fixture.open(context, b"/deep", OpenOptions::default())?;
        let submitted = fixture.path.owner.disk()?.test_snapshot().submitted;
        read_pattern(context, &table, other, 0, 31)?;
        context.expect_eq(
            submitted < fixture.path.owner.disk()?.test_snapshot().submitted,
            true,
        );

        // Step 3: 验证并发任务对预留槽位执行 dup_exact 返回 Busy，释放预留后再次执行成功。
        let reservation = checked(context, table.reserve(fixture.limit))?;
        let target = reservation.fd();
        fixture.path.owner.publish(
            1,
            Job::FileDupExact {
                table: table.clone(),
                source: reused,
                target,
            },
        );
        until("native reserved dup did not return", || {
            fixture.path.owner.observer().results.lock()[1].is_some()
        })?;
        let result = fixture.path.owner.observer().results.lock()[1]
            .take()
            .ok_or(TestError("reserved dup result absent"))?;
        context.expect_eq(result.err(), Some(FsError::Busy));
        context.expect_eq(table.lookup(target).err(), Some(FsError::BadFd));
        drop(reservation);
        fixture.path.owner.publish(
            2,
            Job::FileDupExact {
                table: table.clone(),
                source: reused,
                target,
            },
        );
        until("native exact dup did not return", || {
            fixture.path.owner.observer().results.lock()[2].is_some()
        })?;
        let result = fixture.path.owner.observer().results.lock()[2]
            .take()
            .ok_or(TestError("exact dup result absent"))?;
        let Returned::FileDescriptor(returned_fd) = checked(context, result)? else {
            return Err(TestError("exact dup returned wrong kind"));
        };
        context.expect_eq(returned_fd, target);
        context.expect_eq(
            Arc::ptr_eq(
                &checked(context, table.lookup(reused))?,
                &checked(context, table.lookup(target))?,
            ),
            true,
        );

        // Step 4: 放行块段门闸，验证原先挂起的旧文件读取完整返回，且新复用描述符的游标未被干扰。
        fixture
            .path
            .owner
            .observer()
            .hold_loader
            .store(false, Ordering::Release);
        fixture.path.owner.observer().gate.complete();
        fixture.path.owner.join_tasks()?;
        let result = fixture.path.owner.observer().results.lock()[0]
            .take()
            .ok_or(TestError("held file read result absent"))?;
        let Returned::FileRead { bytes, valid } = checked(context, result)? else {
            return Err(TestError("file read returned wrong kind"));
        };
        context.expect_eq(bytes, mm::PAGE_SIZE);
        context.expect_eq(valid, true);
        context.expect_eq(
            checked(context, table.seek(reused, SeekFrom::Current(0)))?,
            9,
        );
        context.diagnostic(format_args!("file-reuse old-read-pattern=1 close-reopen-same-fd=1 other-file-real-IO=1 reserved-native-Busy=1 simultaneous-hardware-IO-covered=0 hardware-EIO-covered=0"));
        Ok(())
    }
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-file", "fs-file-prefix"), resources("root"), timeout_ms = 30000)]
pub(crate) mod prefix {
    use super::*;

    /// 测试目标：验证跨块读取遇到损坏块（`Corrupt`）时能够返回遇到错误前已成功传输的字节数并推进游标，且后续重试、零长读取和 EOF 读取行为正确。
    /// 场景步骤：
    /// 1. 打开 `/pattern`（其第 3 页在测试镜像中被构造为损坏映射），一次性请求读取前 4 页（16384 字节）。
    /// 2. 在游标停留在 `12288`（第 3 页起始处）时再次请求读取 4096 字节。
    /// 3. 在同一位置调用零长度读取 `read(&mut [])`，随后 `seek` 到文件末尾并读取 1 字节。
    /// 预期结果：首次读取返回前 3 页已成功传输的 `12288` 字节且缓冲区后 4096 字节未被修改，游标停在 `12288`；第二次在损坏页起始处读取直接返回 `Err(FsError::Corrupt)` 且游标不变；零长读取与 EOF 读取均返回 `Ok(0)`。
    #[test]
    #[ktest(tags("B22"))]
    fn successful_prefix_preserves_cursor_and_error(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        fixture.prepare(context, 2)?;
        let fd = fixture.open(context, b"/pattern", OpenOptions::default())?;
        let table = fixture.table();
        let mut bytes = [0xa7; 16384];
        context.expect_eq(checked(context, table.read(fd, &mut bytes))?, 12288);
        context.expect_eq(patterned(&bytes[..12288], 0), true);
        context.expect_eq(bytes[12288..].iter().all(|byte| *byte == 0xa7), true);
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(0)))?,
            12288,
        );

        context.expect_eq(
            table.read(fd, &mut bytes[..4096]).err(),
            Some(FsError::Corrupt),
        );
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(0)))?,
            12288,
        );

        context.expect_eq(checked(context, table.read(fd, &mut []))?, 0);
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::End(0)))?,
            PATTERN_SIZE,
        );
        context.expect_eq(checked(context, table.read(fd, &mut bytes[..1]))?, 0);
        context.diagnostic(format_args!("file-prefix delivered=12288 subsequent-error=Corrupt cursor-after-error=12288 hardware-EIO-covered=0"));
        Ok(())
    }
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-file", "fs-file-mount"), resources("root", "extra"), timeout_ms = 90000)]
pub(crate) mod mounts {
    use super::*;

    /// 测试目标：验证第二块磁盘挂载后，描述符表中的打开文件与 `lookup` 获得的临时 `Arc<File>` 强引用均会阻止卸载（返回 `Busy`），直到最后一个强引用释放后方可成功卸载。
    /// 场景步骤：
    /// 1. 将第二块测试磁盘 `extra` 以只读 Ext4 挂载到 `/mnt/extra`，打开 `/mnt/extra/pattern` 并读取前 31 字节。
    /// 2. 在描述符未关闭时尝试 `unmount`，随后通过 `table.lookup(fd)` 获取强引用 `file` 并关闭表内描述符 `fd`，再次尝试 `unmount` 并通过 `file` 继续读取 37 字节。
    /// 3. 释放 `file` 强引用后调用 `unmount` 与 `prune_unused`，最后再次解析 `/mnt/extra/pattern`。
    /// 预期结果：只要表内或栈上仍持有该挂载点下文件的强引用，`unmount` 均返回 `FsError::Busy` 且已打开文件可继续读取；释放最后一个强引用后卸载成功，挂载点恢复为空目录且原路径返回 `NotFound`。
    #[test]
    #[ktest(tags("B20", "B21", "B23"))]
    fn open_and_lookup_hold_mount_until_last_put(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        fixture.prepare(context, 2)?;
        let vfs = fixture.path.vfs();
        let covered = checked(
            context,
            vfs.resolve(fixture.path.context(), b"/mnt/extra", ResolveMode::Follow),
        )?;
        let disk = fixture
            .path
            .owner
            .disks
            .get(1)
            .ok_or(TestError("file extra Disk absent"))?;
        let volume = checked(
            context,
            Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadOnly, 64),
        )?;
        checked(
            context,
            vfs.mount(
                &covered,
                crate::fs::Filesystem::Ext4(volume.clone()),
                AccessMode::ReadOnly,
            ),
        )?;

        let fd = fixture.open(context, b"/mnt/extra/pattern", OpenOptions::default())?;
        let table = fixture.table();
        read_pattern(context, table, fd, 0, 31)?;
        context.expect_eq(vfs.unmount(&covered).err(), Some(FsError::Busy));
        let file = checked(context, table.lookup(fd))?;
        checked(context, table.close(fd))?;
        context.expect_eq(vfs.unmount(&covered).err(), Some(FsError::Busy));
        let mut bytes = [0xa7; 37];
        context.expect_eq(checked(context, file.read(&mut bytes))?, 37);
        context.expect_eq(patterned(&bytes, 31), true);

        drop(file);
        checked(context, vfs.unmount(&covered))?;
        vfs.prune_unused();
        context.expect_eq(
            vfs.resolve(
                fixture.path.context(),
                b"/mnt/extra/pattern",
                ResolveMode::Follow,
            )
            .err(),
            Some(FsError::NotFound),
        );
        drop(volume);
        Ok(())
    }
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-file", "fs-oom", "fs-file-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod oom {
    use super::*;

    /// 测试目标：验证在 CPU0 内存耗尽（OOM）压力下，已有打开文件的引用复制与已缓存热页读取无需分配即可成功，而新建打开与表扩容失败时会自动回滚预留槽位并在内存恢复后正常复用。
    /// 场景步骤：
    /// 1. 打开 `/pattern` 并预先读取第一页使其进入页缓存，同时获取其 `Arc<File>` 引用 `cached`。
    /// 2. 发布 `Job::FileOom` 子任务：在耗尽堆与物理页后，测试对已有 `cached` 的槽位发布与热页读取，以及新打开文件与超容量表扩容。
    /// 3. 内存压力解除并回收子任务后，校验各项返回指标，并在主线程重新打开 `/pattern` 读取数据。
    /// 预期结果：OOM 期间热页读取与已有引用发布零 I/O 成功完成；新打开与表扩容均返回 `FsError::NoMemory` 且未泄漏预留槽位；内存恢复后新打开成功复用刚才回滚的槽位编号。
    #[test]
    #[ktest(tags("B20", "B21", "B22", "B23"))]
    fn real_pressure_existing_file_and_open_recovery(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        fixture.prepare(context, 3)?;
        let fd = fixture.open(context, b"/pattern", OpenOptions::default())?;
        let table = fixture.table().clone();
        read_pattern(context, &table, fd, 0, mm::PAGE_SIZE)?;
        let cached = checked(context, table.lookup(fd))?;
        let pressure = fixture
            .path
            .owner
            .pressure
            .take()
            .ok_or(TestError("file heap pressure absent"))?;
        fixture.path.owner.publish(
            0,
            Job::FileOom {
                table: table.clone(),
                vfs: fixture.path.vfs().clone(),
                context: fixture.path.context().clone(),
                fd,
                cached,
                pathname: b"/pattern",
                pressure,
            },
        );
        fixture.path.owner.join_tasks()?;
        let result = fixture.path.owner.observer().results.lock()[0]
            .take()
            .ok_or(TestError("file OOM result absent"))?;
        let Returned::FileOom {
            producer_cpu,
            evidence,
            same_ofd,
            bytes,
            valid,
            new_error,
            reserved_fd,
            reused_fd,
            growth_errors,
            growth_reused_fd,
            growth_initial_slots,
            submitted_before,
            submitted_after,
        } = checked(context, result)?
        else {
            return Err(TestError("file OOM returned wrong kind"));
        };
        context.expect_eq(producer_cpu, 0);
        context.expect_eq(evidence, (true, true));
        context.expect_eq(same_ofd, true);
        context.expect_eq(bytes, mm::PAGE_SIZE);
        context.expect_eq(valid, true);
        context.expect_eq(new_error, Some(FsError::NoMemory));
        context.expect_eq(reserved_fd, 1);
        context.expect_eq(reused_fd, reserved_fd);
        context.expect_eq(growth_errors, [Some(FsError::NoMemory); 2]);
        context.expect_eq(growth_reused_fd, growth_initial_slots);
        context.expect_eq(submitted_after, submitted_before);
        context.expect_eq(
            checked(context, table.seek(fd, SeekFrom::Current(0)))?,
            mm::PAGE_SIZE as u64,
        );

        let reopened = fixture.open(context, b"/pattern", OpenOptions::default())?;
        context.expect_eq(reopened, reused_fd);
        read_pattern(context, &table, reopened, 0, 47)?;
        read_pattern(context, &table, fd, mm::PAGE_SIZE as u64, 43)?;
        context.expect_eq(
            Arc::ptr_eq(
                &checked(context, table.lookup(fd))?,
                &checked(context, table.lookup(reopened))?,
            ),
            false,
        );
        Ok(())
    }
}

/// 用户态测试代码页映射的虚拟地址（字节单位，页对齐）。
const VM_CODE: usize = 0x10000;
/// 用户态测试栈页映射的虚拟地址（字节单位，页对齐）。
const VM_STACK: usize = 0x30000;
/// 被测文件映射（`map_file`）在用户地址空间中的目标虚拟地址。
const VM_TARGET: usize = 0x50000;

/// 用户态只读页权限（可读、不可写、不可执行）。
const VM_RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};
/// 用户态读写页权限（可读、可写、不可执行）。
const VM_RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
/// 用户态代码执行页权限（可读、不可写、可执行）。
const VM_RX: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: true,
};

/// 创建一个包含受控用户态测试指令页（`VM_CODE`）和零初始化用户栈页（`VM_STACK`）的新地址空间 `Mm`。
fn vm_image() -> Result<Mm, TestError> {
    let mut mm = Mm::try_new().map_err(|_| TestError("NoMemory: file-mapping root"))?;
    for (address, permissions, contents) in [
        (VM_CODE, VM_RX, arch::user_code()),
        (VM_STACK, VM_RW, &[][..]),
    ] {
        let mut pages = mm::alloc_pages(0, true).ok_or(TestError(
            "NoMemory: file-mapping instruction or stack page",
        ))?;
        assert!(contents.len() <= mm::PAGE_SIZE);
        // SAFETY:
        // 1. `pages` 由 `mm::alloc_pages(0, true)` 刚刚分配并清零，当前栈帧持有其唯一所有权，与只读内核镜像切片 `contents` 物理内存完全不重叠；
        // 2. 前置断言 `assert!(contents.len() <= mm::PAGE_SIZE)` 保证了拷贝长度不会越过单页边界，且此时该页尚未映射到任何页表。
        unsafe {
            core::ptr::copy_nonoverlapping(
                contents.as_ptr(),
                pages.as_mut_ptr().cast::<u8>(),
                contents.len(),
            )
        };
        // SAFETY:
        // 1. `pages` 为刚刚分配并完成字节初始化的独占物理页，仅包含受控测试指令或全零栈数据；
        // 2. 调用 `map_owned` 将该物理页的唯一所有权移交给尚未共享的新建地址空间 `mm`，此后不再通过原裸指针访问或重复释放该页。
        unsafe { mm.map_owned(address, pages, permissions, permissions) }
            .map_err(|_| TestError("file-mapping instruction or stack mapping failed"))?;
    }
    Ok(mm)
}

/// 用户态线程执行一次文件映射内存访问后的观测结果。
pub(super) struct VmAccessResult {
    /// 访问完成前捕获并成功处理的缺页异常（Page Fault）次数。
    faults: usize,
    /// 用户指令实际读出的寄存器值（读访问）或写入后在目标地址观测到的 64 位值（写访问）。
    value: usize,
}

/// 在测试子任务中驱动用户线程 `user` 执行一次读或写指令，处理预期的 `VM_TARGET` 缺页异常并在 `Syscall` 陷入时提取访问结果。
pub(super) fn run_vm_access(
    user: Arc<UserThread>,
    direction: UserAccess,
    cancelled: &AtomicBool,
) -> Result<VmAccessResult, TestError> {
    #[cfg(target_arch = "riscv64")]
    let result_register = 5;
    #[cfg(target_arch = "loongarch64")]
    let result_register = 12;
    let mut faults = 0;
    loop {
        if cancelled.load(Ordering::Acquire) {
            return Err(TestError("file-mapping user stopped"));
        }
        match user.run_user() {
            UserTrap::Interrupt => {
                sched::cond_resched();
            }
            UserTrap::PageFault { address, access } => {
                if address != VM_TARGET || access != direction {
                    return Err(TestError(
                        "unexpected file-mapping fault address or direction",
                    ));
                }
                let pc = user.context().pc;
                if let Err(error) = user.mm().unwrap().fault(address, access) {
                    crate::ktest::report::diagnostic(format_args!(
                        "file-mapping fault: {:?}",
                        error
                    ));
                    return Err(TestError("file-mapping fault resolution failed"));
                }
                if user.context().pc != pc {
                    return Err(TestError("file-mapping fault advanced user PC"));
                }
                faults += 1;
                if faults != 1 {
                    return Err(TestError("resolved file fault repeated"));
                }
            }
            UserTrap::Syscall => {
                let value = if direction == UserAccess::Read {
                    user.context().gpr[result_register]
                } else {
                    user.mm()
                        .unwrap()
                        .read_u64(VM_TARGET)
                        .map_err(|_| TestError("file-mapping user store observation failed"))?
                        as usize
                };
                return Ok(VmAccessResult { faults, value });
            }
            trap => {
                crate::ktest::report::diagnostic(format_args!("file-mapping trap: {:?}", trap));
                return Err(TestError("unexpected file-mapping user trap"));
            }
        }
    }
}

impl FileOwner {
    /// 将用户地址空间 `mm` 与目标地址 `VM_TARGET` 绑定到第 `index` 号预备内核任务，并发布后台用户态访问任务。
    fn start_vm_access(&mut self, index: usize, mm: Arc<Mm>, direction: UserAccess) -> TestResult {
        let mut context = UserContext::new(
            VM_CODE + arch::fault_offset(direction),
            VM_STACK + mm::PAGE_SIZE,
        );
        #[cfg(target_arch = "riscv64")]
        let address_register = 15;
        #[cfg(target_arch = "loongarch64")]
        let address_register = 9;
        context.gpr[address_register] = VM_TARGET;
        let user = UserThread::try_new(mm, context)
            .map_err(|_| TestError("NoMemory: file-mapping user context"))?;
        self.path.owner.prepared[index]
            .as_ref()
            .ok_or(TestError("file-mapping native task already published"))?
            .bind_user(Arc::clone(&user));
        self.path.owner.publish(
            index,
            Job::User {
                thread: user,
                access: direction,
            },
        );
        self.user = Some(index);
        Ok(())
    }

    /// 等待已发布的用户态访问子任务执行完毕，断言其精确触发 1 次缺页异常且观测值等于 `expected`。
    fn join_vm_user(&self, expected: usize) -> TestResult {
        let index = self.user.expect("file-mapping user was not published");
        let join = self
            .path
            .owner
            .joins
            .last()
            .expect("file-mapping native join absent");
        until("file-mapping user did not finish", || {
            join.try_join().is_some()
        })?;
        let returned = self.path.owner.observer().results.lock()[index]
            .take()
            .ok_or(TestError("file-mapping native result absent"))?;
        let Returned::User(result) =
            returned.map_err(|_| TestError("file-mapping native job failed"))?
        else {
            return Err(TestError("file-mapping native result kind differs"));
        };
        let result = result?;
        assert_eq!(result.faults, 1);
        assert_eq!(result.value, expected);
        Ok(())
    }
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", tags("user-vm-file"), resources("root"), timeout_ms = 90000)]
pub(crate) mod vm_lifetime {
    use super::*;

    /// 测试目标：验证文件映射（`map_file`）只读缺页直接共享页缓存物理页、权限升级后写缺页触发私有写时复制（COW）、关闭文件描述符后尾页零填充与超出 EOF 缺页拦截，以及空文件、稀疏文件和目录的映射语义。
    /// 场景步骤：
    /// 1. 打开 `/pattern` 并预取第 0 页的页缓存物理地址 `original`；将其以只读权限（最大权限读写）映射到 `VM_TARGET`，启动用户线程读取并比对映射后的物理页地址与文件游标。
    /// 2. 调用 `protect` 将 `VM_TARGET` 升级为可写，启动用户线程执行写入触发 COW，验证映射物理页改变而页缓存数据与底层文件数据未被污染。
    /// 3. 在 `tail` 处映射跨越文件末尾的两页区域后立即关闭 `fd` 并释放 `file` 引用，验证尾页有效前缀之后自动清零且可私有写入，而完全位于 EOF 之后的第二页读写缺页均返回 `BeyondEof`。
    /// 4. 依次测试空文件 `/empty` 缺页、稀疏文件 `/sparse` 空洞页读取以及对目录 `/` 调用 `map_file`。
    /// 预期结果：只读映射与页缓存共享同一物理页，私有写入分配新 COW 页且不修改原缓存；关闭 `fd` 后映射依然有效；越界页与空文件缺页返回 `BeyondEof`，稀疏空洞页读出全 0，目录映射返回 `NotMappable`。
    #[test]
    fn file_page_identity_eof_and_private_write(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        fixture.prepare(context, 4)?;
        let fd = fixture.open(context, b"/pattern", OpenOptions::default())?;
        let file = checked(context, fixture.table().lookup(fd))?;
        context.expect_eq(checked(context, file.stat())?.rdev, 0);
        context.expect_eq(
            file.ioctl(0, 0, crate::fs::FileIoContext::Kernel),
            Err(FsError::NotTty),
        );
        checked(context, file.seek(SeekFrom::Start(19)))?;
        let mut expected = [0; 8];
        assert_eq!(
            checked(context, file.read_at(0, &mut expected))?,
            expected.len()
        );
        assert!(patterned(&expected, 0));

        // Step 1: 只读映射文件第 0 页并由用户态读取，验证页表直接指向页缓存中的同一物理页且不改变文件游标。
        let mut file_page_attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(0, &mut file_page_attempt),
        )?;
        let original = folio.phys_addr();
        let mm = vm_image()?;
        mm.map_file(
            VM_TARGET,
            mm::PAGE_SIZE,
            VM_RO,
            VM_RW,
            Arc::clone(&file),
            0,
            true,
        )
        .unwrap();
        let mm = Arc::try_new(mm).map_err(|_| TestError("NoMemory: file-mapping MM owner"))?;
        fixture.start_vm_access(0, Arc::clone(&mm), UserAccess::Read)?;
        fixture.join_vm_user(u64::from_le_bytes(expected) as usize)?;
        assert_eq!(mm.mapping(VM_TARGET).unwrap().physical, original);
        assert_eq!(checked(context, file.seek(SeekFrom::Current(0)))?, 19);

        // Step 2: 修改保护权限为可写并由用户态写入，验证触发写时复制（COW）且底层页缓存内容保持不变。
        mm.protect(VM_TARGET, mm::PAGE_SIZE, VM_RW).unwrap();
        fixture.start_vm_access(2, Arc::clone(&mm), UserAccess::Write)?;
        fixture.join_vm_user(0)?;
        assert_ne!(mm.mapping(VM_TARGET).unwrap().physical, original);
        checked(
            context,
            folio.copy_to(0, &mut expected).map_err(FsError::from),
        )?;
        assert!(patterned(&expected, 0));
        let private = b"private-COW";
        assert_eq!(mm.copyout(VM_TARGET + 13, private).error, None);
        let mut cached = [0; 11];
        assert_eq!(
            checked(context, file.read_at(13, &mut cached))?,
            cached.len()
        );
        assert!(patterned(&cached, 13));
        assert_ne!(&cached, private);

        // Step 3: 映射文件尾部两页后关闭文件描述符，验证半页尾部自动零填充、私有可写，且超出 EOF 的整页返回 BeyondEof。
        let tail = VM_TARGET + 2 * mm::PAGE_SIZE;
        let offset = PATTERN_SIZE & !(mm::PAGE_SIZE as u64 - 1);
        mm.map_file(
            tail,
            2 * mm::PAGE_SIZE,
            VM_RW,
            VM_RW,
            Arc::clone(&file),
            offset,
            true,
        )
        .unwrap();
        checked(context, fixture.table().close(fd))?;
        drop(file);
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        let progress = mm.copyin(&mut bytes, tail);
        assert_eq!((progress.copied, progress.error), (mm::PAGE_SIZE, None));
        let valid = (PATTERN_SIZE - offset) as usize;
        assert!(patterned(&bytes[..valid], offset));
        assert!(bytes[valid..].iter().all(|byte| *byte == 0));
        assert_eq!(mm.copyout(tail + valid + 3, b"tail").error, None);
        let mut suffix = [0; 4];
        assert_eq!(mm.copyin(&mut suffix, tail + valid + 3).error, None);
        assert_eq!(&suffix, b"tail");
        assert_eq!(
            mm.fault(tail + mm::PAGE_SIZE, UserAccess::Read),
            Err(VmError::BeyondEof)
        );
        assert_eq!(
            mm.fault(tail + mm::PAGE_SIZE, UserAccess::Write),
            Err(VmError::BeyondEof)
        );

        // Step 4: 验证空文件缺页返回 BeyondEof、稀疏文件空洞页读出全零，以及目录拒绝内存映射（NotMappable）。
        let empty = fixture.open(context, b"/empty", OpenOptions::default())?;
        let empty_file = checked(context, fixture.table().lookup(empty))?;
        let empty_address = tail + 3 * mm::PAGE_SIZE;
        mm.map_file(
            empty_address,
            mm::PAGE_SIZE,
            VM_RO,
            VM_RW,
            empty_file,
            0,
            true,
        )
        .unwrap();
        assert_eq!(
            mm.fault(empty_address, UserAccess::Read),
            Err(VmError::BeyondEof)
        );

        let sparse = fixture.open(context, b"/sparse", OpenOptions::default())?;
        let sparse_file = checked(context, fixture.table().lookup(sparse))?;
        let sparse_address = empty_address + mm::PAGE_SIZE;
        mm.map_file(
            sparse_address,
            mm::PAGE_SIZE,
            VM_RO,
            VM_RW,
            sparse_file,
            4096,
            true,
        )
        .unwrap();
        bytes.fill(0xa7);
        assert_eq!(mm.copyin(&mut bytes, sparse_address).error, None);
        assert!(bytes.iter().all(|byte| *byte == 0));

        let directory = fixture.open(context, b"/", OpenOptions::default())?;
        let directory_file = checked(context, fixture.table().lookup(directory))?;
        assert_eq!(
            directory_file.ioctl(0, 0, crate::fs::FileIoContext::Kernel),
            Err(FsError::NotTty)
        );
        assert_eq!(
            mm.map_file(
                sparse_address + mm::PAGE_SIZE,
                mm::PAGE_SIZE,
                VM_RO,
                VM_RO,
                directory_file,
                0,
                false
            ),
            Err(VmError::NotMappable),
        );
        drop(mm);
        drop(folio);
        Ok(())
    }
}

#[ktest::suite(fixture = FileOwner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("user-vm-file"), resources("root"), timeout_ms = 90000)]
pub(crate) mod vm_racing {
    use super::*;

    /// 测试目标：验证用户文件映射缺页在等待磁盘读块期间不持有地址空间锁，若此时同虚拟地址被并发 `unmap` 并重新映射为匿名页，磁盘读完后不会将旧文件页错误安装到新映射上。
    /// 场景步骤：
    /// 1. 将 `/pattern` 的第 1 页映射到 `VM_TARGET`，启用 `hold_first_page` 钩子后在另一 CPU 启动用户态读线程，等待其触发缺页并阻塞在磁盘块加载门闸上。
    /// 2. 主线程在缺页挂起期间验证 `VM_TARGET` 尚未建立页表项，随后直接对 `VM_TARGET` 执行 `unmap` 并重新 `map_anonymous` 为只读匿名零页。
    /// 3. 放行磁盘加载门闸并等待用户线程完成，检查用户线程读到的值、当前 `VM_TARGET` 映射的物理页地址以及原文件页缓存内容。
    /// 预期结果：缺页挂起期间 `unmap` 与重映射顺利完成（未发生死锁）；用户线程重试访问时读到新匿名页的值 `0`，而原文件第 1 页虽已完成加载进入页缓存，但未被安装到已被替换的 `VM_TARGET` 页表项中。
    #[test]
    fn file_fault_unmap_and_remap(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        sched::test_migrate_to(0);
        // Step 1: 映射文件第 1 页并启动用户态读访问，使其在缺页加载磁盘块时挂起于同步门闸。
        fixture.prepare(context, 2)?;
        let fd = fixture.open(context, b"/pattern", OpenOptions::default())?;
        let file = checked(context, fixture.table().lookup(fd))?;
        let mm = vm_image()?;
        mm.map_file(
            VM_TARGET,
            mm::PAGE_SIZE,
            VM_RO,
            VM_RW,
            Arc::clone(&file),
            mm::PAGE_SIZE as u64,
            true,
        )
        .unwrap();
        let mm = Arc::try_new(mm).map_err(|_| TestError("NoMemory: remapped file MM owner"))?;
        fixture.hold_first_page(context)?;
        fixture.start_vm_access(1, Arc::clone(&mm), UserAccess::Read)?;
        until("file fault did not reach its actual segment gate", || {
            fixture
                .path
                .owner
                .observer()
                .loader_entered
                .load(Ordering::Acquire)
                == 1
                && fixture.path.owner.observer().gate.test_waiting() == 1
        })?;
        assert!(mm.mapping(VM_TARGET).is_none());

        // Step 2: 在文件缺页挂起期间卸载原文件映射，并在同一虚拟地址重新映射匿名零页。
        mm.unmap(VM_TARGET, mm::PAGE_SIZE).unwrap();
        mm.map_anonymous(VM_TARGET, mm::PAGE_SIZE, VM_RO, VM_RW)
            .unwrap();
        // Step 3: 放行磁盘读取门闸，验证用户线程读到新匿名零页，且旧文件物理页未覆盖新映射。
        fixture
            .path
            .owner
            .observer()
            .hold_loader
            .store(false, Ordering::Release);
        fixture.path.owner.observer().gate.complete();
        fixture.join_vm_user(0)?;
        assert_eq!(mm.read_u64(VM_TARGET), Ok(0));

        let mut attempt = ReclaimAttempt::new();
        let old = checked(
            context,
            file.mapping_inode().unwrap().file_page(1, &mut attempt),
        )?;
        assert_ne!(mm.mapping(VM_TARGET).unwrap().physical, old.phys_addr());
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        checked(context, old.copy_to(0, &mut bytes).map_err(FsError::from))?;
        assert!(patterned(&bytes, mm::PAGE_SIZE as u64));
        assert_eq!(checked(context, file.seek(SeekFrom::Current(0)))?, 0);
        drop(old);
        drop(mm);
        Ok(())
    }
}
