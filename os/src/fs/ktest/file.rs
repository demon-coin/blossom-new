//! 具体打开文件、可增长描述符表和真实异步拥有的消费回归。

use crate::arch::{self, UserAccess, UserContext, UserTrap};
use crate::fs::ktest::path::PathOwner;
use crate::fs::ktest::*;
use crate::fs::{DescriptorFlags, FileTable, OpenOptions, SeekFrom};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;
use crate::task::UserThread;

struct FileOwner {
    path: PathOwner,
    table: Option<Arc<FileTable>>,
    hook_volume: Option<Arc<Ext4Volume>>,
    user: Option<usize>,
    limit: usize,
}

impl FileOwner {
    /// 正文构造完整路径视图和小表，本例额度独立保存。
    fn prepare(&mut self, context: &TestContext<'_>, limit: usize) -> TestResult {
        self.path.prepare_path(context, AccessMode::ReadOnly, 4)?;
        self.limit = limit;
        let table = Arc::try_new(checked(context, FileTable::try_new())?)
            .map_err(|_| TestError("file table owner OOM"))?;
        self.table = Some(table);
        Ok(())
    }

    /// 正文借用完整表，静止化以后不再允许发表描述符。
    fn table(&self) -> &Arc<FileTable> {
        self.table
            .as_ref()
            .expect("file table consumed before body")
    }

    /// 使用本例真实根和工作目录打开字节路径。
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

    /// 本例首次读取的冷页在首段收权后暂停，回调不持表锁或卷锁。
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
    /// 准备阶段只拥有完整旧夹具，路径、表和观察入口均由正文准备。
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

    /// 先撤自己的页观察并放行，再释放表；旧路径夹具唯一负责用户和文件任务的取消、join 与设备停止。
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

    /// 原路径夹具消费设备拥有者并核第二层完整六域。
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

/// 每个字节均与外部文件图样比较，并核消费者实际交付的长度。
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

/// 静态宏锚点交给既有父入口，不维护第二份用例列表。
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

    /// 独立打开和硬链接各有位置，复制描述符共享位置，定位读不修改位置且元数据来自真实 inode。
    #[test]
    #[ktest(tags("B20", "B21", "B22"))]
    fn independent_shared_cursor_and_bytes(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
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

/// 逐项核原始目录名字和唯一编号，不缓存或重新实现目录扫描。
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

    /// 普通与索引目录逐项读出原字节，复制共享 cookie，独立打开与真实类型检查互不混淆。
    #[test]
    #[ktest(tags("B20", "B21", "B22"))]
    fn entries_cookie_types_and_nofollow(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        fixture.prepare(context, 8)?;
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

    /// 移动预留只撤自己的槽，精确复制拒绝预留目标，容量与大编号都由真实调用消费。
    #[test]
    #[ktest(tags("B20", "B21"))]
    fn reservation_capacity_reuse_and_wide_fd(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
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

    /// 页读暂停时关闭与复用同编号仍保活旧 OFD，另一个真实文件可前进且预留目标保持 Busy。
    #[test]
    #[ktest(tags("B20", "B21", "B22", "B23"))]
    fn read_close_reuse_progress_and_reserved_dup(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
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

    /// 原映射错误经描述符保留正前缀，后次 Corrupt 不改变位置，零长和 EOF 不触发坏页。
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

    /// 第二盘打开文件与 lookup 取得的强引用分别阻止卸载，末引用释放后可正常卸载并清理缓存。
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

    /// CPU0 真堆压力下既有引用发表和热页命中不分配，新打开失败撤槽并在放压后恢复。
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

const VM_CODE: usize = 0x10000;
const VM_STACK: usize = 0x30000;
const VM_TARGET: usize = 0x50000;

const VM_RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};
const VM_RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
const VM_RX: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: true,
};

/// 文件用户场景也使用原受控指令和真实拥有根，不复制模拟页表或设置测试 syscall。
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
        // SAFETY: 原页独占且完整清零，稳定代码字节在已检查页内；两块内核 RAM 不重叠，
        // 本次初始化结束前没有页表发表或用户访问。
        unsafe {
            core::ptr::copy_nonoverlapping(
                contents.as_ptr(),
                pages.as_mut_ptr().cast::<u8>(),
                contents.len(),
            )
        };
        // SAFETY: 全页已初始化，仅包含受控用户代码或零栈；排他引用与原始初始化已结束，
        // 唯一拥有转交尚未共享的 Mm，旧裸地址不再被使用或恢复为拥有者。
        unsafe { mm.map_owned(address, pages, permissions, permissions) }
            .map_err(|_| TestError("file-mapping instruction or stack mapping failed"))?;
    }
    Ok(mm)
}

/// 原生用户 Task 的真实访问结果；只保存已发生的缺页数和实际指令观察值。
pub(super) struct VmAccessResult {
    faults: usize,
    value: usize,
}

/// 在原文件 fixture 预备的 Task 中执行真实用户访问；错误交回父正文，不跨原生栈展开断言。
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
    /// 绑定原固定槽的尚未发表 Task，正常返回和 MM 离栈继续由原文件 fixture 的 JoinHandle 拥有。
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

    /// 原 JoinState 完成后消费真实访问结果，用户栈、活动根与 MM 运行引用均已离开。
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

    /// 用户读写沿同一缓存页与私有 COW，尾页零填、整页 EOF 和关闭 fd 各保真实语义。
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

    /// 真实用户文件 fault 在读盘时放锁；同址重新映射后不安装先前文件的准备页。
    #[test]
    fn file_fault_unmap_and_remap(
        context: &mut TestContext<'_>,
        fixture: &mut FileOwner,
    ) -> TestResult {
        sched::test_migrate_to(0);
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

        mm.unmap(VM_TARGET, mm::PAGE_SIZE).unwrap();
        mm.map_anonymous(VM_TARGET, mm::PAGE_SIZE, VM_RO, VM_RW)
            .unwrap();
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
