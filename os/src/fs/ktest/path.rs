//! 真实目录名字、逐组件路径和只读挂载的拥有与消费回归。

use crate::fs::dentry::DentryRef;
use crate::fs::file::{open_path_with_flags, FileIoContext};
use crate::fs::ktest::*;
use crate::fs::memfs::{Filesystem, MemoryFs, MemoryFsKind, MemorySpecial};
use crate::fs::{FsContext, Path, ResolveMode, Vfs};

pub(super) struct PathOwner {
    pub(super) owner: Owner,
    vfs: Option<Arc<Vfs>>,
    context: Option<FsContext>,
    volume: Option<Arc<Ext4Volume>>,
}

impl PathOwner {
    /// 按显式模式与挂载容量准备根视图，失败时原设备保留完整停止责任。
    pub(super) fn prepare_path(
        &mut self,
        context: &TestContext<'_>,
        mode: AccessMode,
        mount_capacity: usize,
    ) -> TestResult {
        // 同一 gate 只观察名字加载，不能同时暂停另一个名字的 inode 后端。
        self.owner.cache().ktest_set_load_hook(None);
        let volume = checked(
            context,
            Ext4Volume::open(BlockView::whole(self.owner.disk()?.clone()), mode, 64),
        )?;
        let vfs = Arc::try_new(checked(
            context,
            Vfs::try_new(
                Filesystem::Ext4(volume.clone()),
                self.owner.cache().clone(),
                128,
                mount_capacity,
                mode,
            ),
        )?)
        .map_err(|_| TestError("path view owner OOM"))?;
        let fs_context = checked(context, FsContext::new(vfs.root(), vfs.root()))?;
        let observer = self.owner.observer().clone();
        let hook: Arc<dyn Fn(&DentryRef) + Send + Sync> =
            Arc::try_new(move |dentry: &DentryRef| {
                if dentry.name().as_slice() == b"pattern"
                    && observer.hold_loader.load(Ordering::Acquire)
                {
                    observer.loader_entered.fetch_add(1, Ordering::AcqRel);
                    observer.gate.wait();
                }
            })
            .map_err(|_| TestError("path lookup observer OOM"))?;
        vfs.dcache().ktest_set_lookup_hook(Some(hook));
        self.volume = Some(volume);
        self.context = Some(fs_context);
        self.vfs = Some(vfs);
        Ok(())
    }

    /// 正文借用完整视图，静止化之后不再允许取得新的路径。
    pub(super) fn vfs(&self) -> &Arc<Vfs> {
        self.vfs.as_ref().expect("path view consumed before body")
    }

    /// 根与工作目录均持有真实 Path，不从当前任务推导查找上下文。
    pub(super) fn context(&self) -> &FsContext {
        self.context
            .as_ref()
            .expect("path context consumed before body")
    }
    /// 真实任务已停稳、外部 Path 已释放后撤观察入口并释放本例 FS 视图。
    pub(super) fn release_view(&mut self) {
        if let Some(vfs) = &self.vfs {
            vfs.dcache().ktest_set_lookup_hook(None);
        }
        drop(self.context.take());
        drop(self.vfs.take());
        drop(self.volume.take());
    }

    /// 普通路径错误保留生产原因，成功结果仍由调用者拥有。
    fn resolve(&self, context: &TestContext<'_>, pathname: &[u8]) -> Result<Path, TestError> {
        checked(
            context,
            self.vfs()
                .resolve(self.context(), pathname, ResolveMode::Follow),
        )
    }

    /// 清理自己的观察入口后真 join，释放所有路径与卷，再停止真实设备。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if let Some(vfs) = &self.vfs {
            vfs.dcache().ktest_set_lookup_hook(None);
        }
        self.owner
            .observer()
            .hold_loader
            .store(false, Ordering::Release);
        self.owner.observer().gate.complete();
        self.owner.quiesce_jobs()?;
        self.release_view();
        self.owner.quiesce_devices(context)
    }
}

impl Fixture<DeviceInventory> for PathOwner {
    /// 每例只构造完整原设备拥有者，卷与路径的准备失败属于正文并保留正常清理责任。
    fn set_up(
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        Ok(Self {
            owner: Owner::prepare(context, inventory)?,
            vfs: None,
            context: None,
            volume: None,
        })
    }

    /// 框架在失败时也保留本例原对象，无法收敛由既有 case-abort 终结。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        PathOwner::quiesce(self, context)
    }

    /// 路径、任务和卷已结束后消费原设备拥有者，核第二层完整六域。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let Self {
            owner,
            vfs,
            context: fs_context,
            volume,
        } = self;
        drop(fs_context);
        drop(vfs);
        drop(volume);
        owner.tear_down(context)
    }
}

/// 读取消费者可见的完整首段图样，不以 inode 编号或名字缓存命中替代数据。
fn read_pattern(context: &mut TestContext<'_>, path: &Path) -> TestResult {
    let inode = checked(context, path.inode())?;
    context.expect_eq(
        checked(context, inode.metadata_snapshot())?.size,
        PATTERN_SIZE,
    );
    let mut bytes = [0xa7; 4096];
    context.expect_eq(checked(context, inode.read_at(0, &mut bytes))?, bytes.len());
    context.expect_eq(patterned(&bytes, 0), true);
    Ok(())
}

/// 负名字与真实加载错误都通过最终 inode 消费分类，不假定缓存返回对象的形状。
fn lookup_error(vfs: &Vfs, parent: &DentryRef, name: &[u8]) -> Option<FsError> {
    vfs.dcache()
        .lookup(vfs.inodes(), parent, name)
        .and_then(|dentry| dentry.inode().map(|_| ()))
        .err()
}

/// 父入口只并入真实静态宏锚点，不另建中央 case 名单。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 7] {
    [
        walk::__ktest_anchor(),
        boundary::__ktest_anchor(),
        mounts::__ktest_anchor(),
        pruning::__ktest_anchor(),
        loading::__ktest_anchor(),
        checksum::__ktest_anchor(),
        oom::__ktest_anchor(),
    ]
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-path", "fs-path-walk"), resources("root"), timeout_ms = 90000)]
pub(crate) mod walk {
    use super::*;

    /// 组件必须先存在且为目录，字节名与两个硬链接保持各自名字和同一文件身份。
    #[test]
    #[ktest(tags("B16", "B17", "B19"))]
    fn components_bytes_and_types(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs();
        let root = fixture.resolve(context, b"////")?;
        context.expect_eq(root.ptr_eq(fixture.context().root()), true);
        let walk = fixture.resolve(context, b"/walk")?;
        let sub = fixture.resolve(context, b"//walk///sub/./")?;
        context.expect_eq(
            checked(context, checked(context, sub.inode())?.metadata_snapshot())?.kind,
            InodeKind::Directory,
        );
        context.expect_eq(
            fixture.resolve(context, b"/walk/sub/..")?.ptr_eq(&walk),
            true,
        );
        for pathname in [b"/absent/..".as_slice(), b"/walk/absent/../sub"] {
            context.expect_eq(
                vfs.resolve(fixture.context(), pathname, ResolveMode::Follow)
                    .err(),
                Some(FsError::NotFound),
            );
        }
        for pathname in [
            b"/pattern/".as_slice(),
            b"/pattern/.",
            b"/pattern/..",
            b"/pattern/child",
        ] {
            context.expect_eq(
                vfs.resolve(fixture.context(), pathname, ResolveMode::Follow)
                    .err(),
                Some(FsError::NotDirectory),
            );
        }
        context.expect_eq(
            vfs.resolve(fixture.context(), b"/walk/a\0b", ResolveMode::Follow)
                .err(),
            Some(FsError::InvalidName),
        );
        context.expect_eq(
            vfs.resolve(fixture.context(), b"/Pattern", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );
        for name in [b"".as_slice(), b"a/b", b"a\0b"] {
            context.expect_eq(
                lookup_error(vfs, root.dentry(), name),
                Some(FsError::InvalidName),
            );
        }
        context.expect_eq(
            lookup_error(vfs, root.dentry(), &[b'n'; 256]),
            Some(FsError::NameTooLong),
        );
        let file = fixture.resolve(context, b"/pattern")?;
        let hard = fixture.resolve(context, b"/hardlink")?;
        context.expect_eq(
            lookup_error(vfs, file.dentry(), b"child"),
            Some(FsError::NotDirectory),
        );
        context.expect_eq(file.dentry().ptr_eq(hard.dentry()), false);
        context.expect_eq(
            checked(context, file.inode())?.ptr_eq(checked(context, hard.inode())?),
            true,
        );
        context.expect_eq(
            core::ptr::eq(
                checked(context, file.inode())?.mapping(),
                checked(context, hard.inode())?.mapping(),
            ),
            true,
        );
        read_pattern(context, &file)?;
        read_pattern(context, &hard)?;
        let raw = fixture.resolve(context, b"/name-\xff\xfe")?;
        context.expect_eq(raw.dentry().name().as_slice(), b"name-\xff\xfe".as_slice());
        let mut bytes = [0xa7; 9];
        context.expect_eq(
            checked(
                context,
                checked(context, raw.inode())?.read_at(0, &mut bytes),
            )?,
            9,
        );
        context.expect_eq(bytes, *b"raw-name\n");
        Ok(())
    }

    /// 同一可写卷的独立挂载视图共用 inode，RO 视图拒绝写意图但读取原文件内容。
    #[test]
    #[ktest(tags("W01"))]
    fn rw_volume_keeps_readonly_mount_view(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let volume = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(fixture.owner.disk()?.clone()),
                AccessMode::ReadWrite,
                64,
            ),
        )?;
        let reopened = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(fixture.owner.disk()?.clone()),
                AccessMode::ReadWrite,
                64,
            ),
        )?;
        context.expect_eq(Arc::ptr_eq(&volume, &reopened), true);
        let ro = checked(
            context,
            Vfs::try_new(
                Filesystem::Ext4(volume.clone()),
                fixture.owner.cache().clone(),
                16,
                1,
                AccessMode::ReadOnly,
            ),
        )?;
        let rw = checked(
            context,
            Vfs::try_new(
                Filesystem::Ext4(reopened),
                fixture.owner.cache().clone(),
                16,
                1,
                AccessMode::ReadWrite,
            ),
        )?;
        let ro_context = checked(context, FsContext::new(ro.root(), ro.root()))?;
        let rw_context = checked(context, FsContext::new(rw.root(), rw.root()))?;
        let ro_file = checked(
            context,
            ro.resolve(&ro_context, b"/pattern", ResolveMode::Follow),
        )?;
        let rw_file = checked(
            context,
            rw.resolve(&rw_context, b"/pattern", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, ro_file.inode())?.ptr_eq(checked(context, rw_file.inode())?),
            true,
        );
        context.expect_eq(ro_file.check_access(0, 0, 2), Err(FsError::ReadOnly));
        context.expect_eq(rw_file.check_access(0, 0, 2), Ok(()));
        context.expect_eq(
            crate::fs::file::open_path_with_flags(
                &ro,
                ro_file.clone(),
                1,
                crate::fs::FileIoContext::Kernel,
            )
            .err(),
            Some(FsError::ReadOnly),
        );
        read_pattern(context, &ro_file)?;
        read_pattern(context, &rw_file)?;
        context.diagnostic(format_args!(
            "mount-mode volume-RW=1 same-volume=1 canonical-inode=1 RO-write-intent-rejected=1 RW-access-granted=1 full-read-pattern=1 file-write-fsync-covered=0"
        ));
        Ok(())
    }

    /// RAM 根与原 inode 缓存持续存活时，同盘普通卸载允许切换写模式并冷读同步后的完整数据。
    #[test]
    #[ktest(tags("fs-mount-lifetime"))]
    fn ro_unmount_rw_reopen_persists_file(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        let memory = checked(context, MemoryFs::try_new(MemoryFsKind::Tmpfs, 0o755))?;
        fixture.vfs = Some(
            Arc::try_new(checked(
                context,
                Vfs::try_new(
                    Filesystem::Memory(memory),
                    fixture.owner.cache().clone(),
                    128,
                    2,
                    AccessMode::ReadWrite,
                ),
            )?)
            .map_err(|_| TestError("mount lifetime RAM root OOM"))?,
        );
        fixture.context = Some(checked(
            context,
            FsContext::new(fixture.vfs().root(), fixture.vfs().root()),
        )?);
        let vfs = fixture.vfs();
        let fs_context = fixture.context();
        checked(
            context,
            vfs.mkdir_from(fs_context, fs_context.root(), b"/disk", 0o755, (0, 0), 0),
        )?;
        let covered = fixture.resolve(context, b"/disk")?;
        let view = BlockView::whole(fixture.owner.disk()?.clone());
        let volume = checked(
            context,
            Ext4Volume::open(view.clone(), AccessMode::ReadOnly, 64),
        )?;
        checked(
            context,
            vfs.mount(&covered, Filesystem::Ext4(volume), AccessMode::ReadOnly),
        )?;
        {
            let path = fixture.resolve(context, b"/disk/pattern")?;
            let file = checked(
                context,
                open_path_with_flags(vfs, path, 0, FileIoContext::Kernel),
            )?;
            context.expect_eq(checked(context, file.stat())?.size, PATTERN_SIZE);
            let mut bytes = [0xa7; 4096];
            let mut offset = 0;
            while offset < PATTERN_SIZE {
                let length = (PATTERN_SIZE - offset).min(bytes.len() as u64) as usize;
                context.expect_eq(
                    checked(context, file.read_at(offset, &mut bytes[..length]))?,
                    length,
                );
                context.expect_eq(patterned(&bytes[..length], offset), true);
                offset += length as u64;
            }
        }
        checked(context, vfs.unmount(&covered))?;

        let volume = checked(
            context,
            Ext4Volume::open(view.clone(), AccessMode::ReadWrite, 64),
        )?;
        checked(
            context,
            vfs.mount(&covered, Filesystem::Ext4(volume), AccessMode::ReadWrite),
        )?;
        const WRITTEN: &[u8] = b"persisted after unmount";
        const EXPECTED: &[u8] = b"persisted";
        {
            let file = checked(
                context,
                vfs.open_with(
                    fs_context,
                    fs_context.root(),
                    b"/disk/mount-lifetime.tmp",
                    (0, 0),
                    Some((0o644, 0)),
                    true,
                    false,
                    |path, _| open_path_with_flags(vfs, path, 2, FileIoContext::Kernel),
                ),
            )?;
            context.expect_eq(checked(context, file.write_at(0, WRITTEN))?, WRITTEN.len());
            checked(
                context,
                vfs.rename_from(
                    fs_context,
                    fs_context.root(),
                    b"/disk/mount-lifetime.tmp",
                    fs_context.root(),
                    b"/disk/mount-lifetime.done",
                    false,
                    (0, 0),
                ),
            )?;
            checked(context, file.truncate(EXPECTED.len() as u64))?;
            checked(context, file.sync_range(0..u64::MAX, false))?;
            checked(context, vfs.sync())?;
        }
        checked(context, vfs.unmount(&covered))?;

        let volume = checked(context, Ext4Volume::open(view, AccessMode::ReadOnly, 64))?;
        checked(
            context,
            vfs.mount(&covered, Filesystem::Ext4(volume), AccessMode::ReadOnly),
        )?;
        {
            context.expect_eq(
                vfs.resolve(fs_context, b"/disk/mount-lifetime.tmp", ResolveMode::Follow)
                    .err(),
                Some(FsError::NotFound),
            );
            let path = fixture.resolve(context, b"/disk/mount-lifetime.done")?;
            let file = checked(
                context,
                open_path_with_flags(vfs, path, 0, FileIoContext::Kernel),
            )?;
            context.expect_eq(checked(context, file.stat())?.size, EXPECTED.len() as u64);
            let mut bytes = [0xa7; WRITTEN.len()];
            context.expect_eq(
                checked(context, file.read_at(0, &mut bytes))?,
                EXPECTED.len(),
            );
            context.expect_eq(&bytes[..EXPECTED.len()], EXPECTED);
        }
        checked(context, vfs.unmount(&covered))?;
        Ok(())
    }

    /// 链接消费父目录、当前根和末尾目录语义，缓存命中也服从 NoFollow 与四十次上限。
    #[test]
    #[ktest(tags("B18"))]
    fn links_follow_modes_and_limit(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs();
        let walk = fixture.resolve(context, b"/walk")?;
        let sub = fixture.resolve(context, b"/walk/sub")?;
        let cwd = checked(context, FsContext::new(vfs.root(), walk.clone()))?;
        let pattern = fixture.resolve(context, b"/pattern")?;
        for pathname in [
            b"relative".as_slice(),
            b"absolute",
            b"../pattern",
            b"chain02",
        ] {
            let path = checked(context, vfs.resolve(&cwd, pathname, ResolveMode::Follow))?;
            context.expect_eq(path.ptr_eq(&pattern), true);
            read_pattern(context, &path)?;
        }
        for mode in [
            ResolveMode::Follow,
            ResolveMode::NoFollow,
            ResolveMode::Follow,
            ResolveMode::NoFollow,
        ] {
            let path = checked(context, vfs.resolve(&cwd, b"relative", mode))?;
            let expected = if matches!(mode, ResolveMode::NoFollow) {
                InodeKind::Symlink
            } else {
                InodeKind::Regular
            };
            context.expect_eq(
                checked(context, checked(context, path.inode())?.metadata_snapshot())?.kind,
                expected,
            );
        }
        let link = checked(
            context,
            vfs.resolve(&cwd, b"directory", ResolveMode::NoFollow),
        )?;
        context.expect_eq(
            checked(context, checked(context, link.inode())?.read_link())?.as_slice(),
            b"sub",
        );
        context.expect_eq(
            checked(
                context,
                vfs.resolve(&cwd, b"directory/", ResolveMode::NoFollow),
            )?
            .ptr_eq(&sub),
            true,
        );
        context.expect_eq(
            checked(
                context,
                vfs.resolve(&cwd, b"directory/..", ResolveMode::NoFollow),
            )?
            .ptr_eq(&walk),
            true,
        );
        context.expect_eq(
            vfs.resolve(&cwd, b"relative/", ResolveMode::NoFollow).err(),
            Some(FsError::NotDirectory),
        );
        for pathname in [b"chain01".as_slice(), b"loop"] {
            context.expect_eq(
                vfs.resolve(&cwd, pathname, ResolveMode::Follow).err(),
                Some(FsError::TooManySymlinks),
            );
        }
        let loop_link = checked(context, vfs.resolve(&cwd, b"loop", ResolveMode::NoFollow))?;
        context.expect_eq(
            checked(context, checked(context, loop_link.inode())?.read_link())?.as_slice(),
            b"loop",
        );
        Ok(())
    }
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-path", "fs-path-boundary"), resources("root"), timeout_ms = 30000)]
pub(crate) mod boundary {
    use super::*;

    /// 显式子树根限制点点与绝对链接，工作目录必须位于该根的真实祖先关系内。
    #[test]
    #[ktest(tags("B17", "B18"))]
    fn scoped_root_and_cwd(context: &mut TestContext<'_>, fixture: &mut PathOwner) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs();
        let walk = fixture.resolve(context, b"/walk")?;
        let sub = fixture.resolve(context, b"/walk/sub")?;
        let scoped = checked(context, FsContext::new(walk.clone(), sub.clone()))?;
        for pathname in [b"../..".as_slice(), b"/../..", b"../../../../"] {
            context.expect_eq(
                checked(context, vfs.resolve(&scoped, pathname, ResolveMode::Follow))?
                    .ptr_eq(&walk),
                true,
            );
        }
        context.expect_eq(
            checked(context, vfs.resolve(&scoped, b"/sub", ResolveMode::Follow))?.ptr_eq(&sub),
            true,
        );
        for pathname in [b"../absolute".as_slice(), b"../relative"] {
            context.expect_eq(
                vfs.resolve(&scoped, pathname, ResolveMode::Follow).err(),
                Some(FsError::NotFound),
            );
        }
        context.expect_eq(FsContext::new(walk.clone(), vfs.root()).is_err(), true);
        let regular = fixture.resolve(context, b"/pattern")?;
        context.expect_eq(
            FsContext::new(regular.clone(), regular).err(),
            Some(FsError::NotDirectory),
        );
        Ok(())
    }
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-path", "fs-path-mount"), resources("root", "extra"), timeout_ms = 90000)]
pub(crate) mod mounts {
    use super::*;

    /// 第二个真实磁盘穿越有独立文件身份，活 Path 阻止普通卸载且点点退回父视图。
    #[test]
    #[ktest(tags("B16", "B18", "B23"))]
    fn second_disk_busy_release_and_parent(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs();
        let covered = fixture.resolve(context, b"/mnt/extra")?;
        let mnt = fixture.resolve(context, b"/mnt")?;
        let other = fixture.resolve(context, b"/walk")?;
        let disk = fixture
            .owner
            .disks
            .get(1)
            .ok_or(TestError("path extra Disk absent"))?;
        let volume = checked(
            context,
            Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadOnly, 64),
        )?;
        checked(
            context,
            vfs.mount(
                &covered,
                Filesystem::Ext4(volume.clone()),
                AccessMode::ReadOnly,
            ),
        )?;
        let mounted = fixture.resolve(context, b"/mnt/extra")?;
        context.expect_eq(mounted.ptr_eq(&covered), false);
        context.expect_eq(
            mounted
                .filesystem()
                .ptr_eq(&Filesystem::Ext4(volume.clone())),
            true,
        );
        context.expect_eq(checked(context, mounted.inode())?.number(), 2);
        context.expect_eq(
            checked(context, mounted.inode())?.ptr_eq(checked(context, vfs.root().inode())?),
            false,
        );
        context.expect_eq(
            fixture.resolve(context, b"/mnt/extra/..")?.ptr_eq(&mnt),
            true,
        );
        let extra_context = checked(context, FsContext::new(mounted.clone(), mounted.clone()))?;
        context.expect_eq(
            checked(
                context,
                vfs.resolve(&extra_context, b"/../../", ResolveMode::Follow),
            )?
            .ptr_eq(&mounted),
            true,
        );
        let file = fixture.resolve(context, b"/mnt/extra/pattern")?;
        let host = fixture.resolve(context, b"/pattern")?;
        context.expect_eq(
            file.filesystem().ptr_eq(&Filesystem::Ext4(volume.clone())),
            true,
        );
        context.expect_eq(
            checked(context, file.inode())?.ptr_eq(checked(context, host.inode())?),
            false,
        );
        read_pattern(context, &file)?;
        checked(
            context,
            vfs.mount(
                &other,
                Filesystem::Ext4(volume.clone()),
                AccessMode::ReadOnly,
            ),
        )?;
        let alias = fixture.resolve(context, b"/walk")?;
        context.expect_eq(alias.ptr_eq(&mounted), false);
        context.expect_eq(
            checked(context, alias.inode())?.ptr_eq(checked(context, mounted.inode())?),
            true,
        );
        context.expect_eq(vfs.unmount(&other).err(), Some(FsError::Busy));
        context.expect_eq(
            vfs.mount(
                &other,
                Filesystem::Ext4(fixture.volume.as_ref().expect("root volume alive").clone()),
                AccessMode::ReadOnly,
            )
            .err(),
            Some(FsError::Busy),
        );
        drop(alias);
        checked(context, vfs.unmount(&other))?;
        context.expect_eq(vfs.unmount(&covered).err(), Some(FsError::Busy));
        drop(extra_context);
        drop(mounted);
        context.expect_eq(vfs.unmount(&covered).err(), Some(FsError::Busy));
        read_pattern(context, &file)?;
        drop(file);
        checked(context, vfs.unmount(&covered))?;
        context.expect_eq(
            fixture.resolve(context, b"/mnt/extra")?.ptr_eq(&covered),
            true,
        );
        context.expect_eq(
            vfs.resolve(
                fixture.context(),
                b"/mnt/extra/pattern",
                ResolveMode::Follow,
            )
            .err(),
            Some(FsError::NotFound),
        );
        vfs.prune_unused();
        checked(
            context,
            vfs.mount(
                &covered,
                Filesystem::Ext4(volume.clone()),
                AccessMode::ReadOnly,
            ),
        )?;
        let reread = fixture.resolve(context, b"/mnt/extra/pattern")?;
        read_pattern(context, &reread)?;
        drop(reread);
        checked(context, vfs.unmount(&covered))?;
        read_pattern(context, &host)?;
        Ok(())
    }

    /// 同一VFS挂入RAM目录，真实名字修改、清缓存、挂点转义与Busy卸载共用原路径协议。
    #[test]
    #[ktest(tags("T01", "T30", "fs-path-memory"))]
    fn memory_names_and_actual_mount_snapshot(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs();
        let fs_context = fixture.context();
        let covered = fixture.resolve(context, b"/mnt/extra")?;
        let memory = checked(context, MemoryFs::try_new(MemoryFsKind::Tmpfs, 0o755))?;
        checked(
            context,
            vfs.mount(
                &covered,
                Filesystem::Memory(memory.clone()),
                AccessMode::ReadWrite,
            ),
        )?;
        let ram_root = fixture.resolve(context, b"/mnt/extra")?;
        context.expect_eq(
            ram_root.filesystem().ptr_eq(&Filesystem::Memory(memory)),
            true,
        );
        for pathname in [
            b"/mnt/extra/a".as_slice(),
            b"/mnt/extra/b",
            b"/mnt/extra/a/raw \t\n\\#",
        ] {
            checked(
                context,
                vfs.mkdir_from(fs_context, fs_context.root(), pathname, 0o755, (0, 0), 0),
            )?;
        }
        checked(
            context,
            vfs.mkdir_from(
                fs_context,
                fs_context.root(),
                b"/mnt/extra/b/empty",
                0o755,
                (0, 0),
                0,
            ),
        )?;
        checked(
            context,
            vfs.rmdir_from(fs_context, fs_context.root(), b"/mnt/extra/b/empty", (0, 0)),
        )?;
        context.expect_eq(
            vfs.resolve(fs_context, b"/mnt/extra/b/empty", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );
        context.expect_eq(
            vfs.rmdir_from(fs_context, fs_context.root(), b"/mnt/extra/a", (0, 0))
                .err(),
            Some(FsError::NotEmpty),
        );
        let file = checked(
            context,
            vfs.open_with(
                fs_context,
                fs_context.root(),
                b"/mnt/extra/a/file",
                (0, 0),
                Some((0o644, 0)),
                true,
                false,
                |path, _| Ok(path),
            ),
        )?;
        let inode = checked(context, file.inode())?.clone();
        checked(
            context,
            vfs.link_from(
                &file,
                fs_context,
                fs_context.root(),
                b"/mnt/extra/b/hard",
                (0, 0),
            ),
        )?;
        checked(
            context,
            vfs.symlink_from(
                fs_context,
                fs_context.root(),
                b"/mnt/extra/link",
                b"b/hard",
                (0, 0),
            ),
        )?;
        context.expect_eq(
            checked(
                context,
                fixture.resolve(context, b"/mnt/extra/link")?.inode(),
            )?
            .ptr_eq(&inode),
            true,
        );
        checked(
            context,
            vfs.rename_from(
                fs_context,
                fs_context.root(),
                b"/mnt/extra/a",
                fs_context.root(),
                b"/mnt/extra/b/moved",
                false,
                (0, 0),
            ),
        )?;
        let parent = fixture.resolve(context, b"/mnt/extra/b")?;
        context.expect_eq(
            fixture
                .resolve(context, b"/mnt/extra/b/moved/..")?
                .ptr_eq(&parent),
            true,
        );
        context.expect_eq(
            vfs.rename_from(
                fs_context,
                fs_context.root(),
                b"/mnt/extra/b",
                fs_context.root(),
                b"/mnt/extra/b/moved/cycle",
                false,
                (0, 0),
            )
            .err(),
            Some(FsError::InvalidOperation),
        );
        context.expect_eq(
            vfs.link_from(
                &fixture.resolve(context, b"/pattern")?,
                fs_context,
                fs_context.root(),
                b"/mnt/extra/cross",
                (0, 0),
            )
            .err(),
            Some(FsError::CrossDevice),
        );
        drop(file);
        vfs.prune_unused();
        let reopened = fixture.resolve(context, b"/mnt/extra/b/moved/file")?;
        context.expect_eq(checked(context, reopened.inode())?.ptr_eq(&inode), true);
        let nested = fixture.resolve(context, b"/mnt/extra/b/moved/raw \t\n\\#")?;
        let procfs = checked(context, MemoryFs::try_new(MemoryFsKind::Procfs, 0o555))?;
        checked(
            context,
            vfs.mount(&nested, Filesystem::Memory(procfs), AccessMode::ReadOnly),
        )?;
        let proc_root = fixture.resolve(context, b"/mnt/extra/b/moved/raw \t\n\\#")?;
        context.expect_eq(proc_root.check_writable(), Err(FsError::ReadOnly));
        context.expect_eq(vfs.unmount(&nested).err(), Some(FsError::Busy));
        let text = checked(context, vfs.mounts_text(&vfs.root()))?;
        let lines: Vec<&[u8]> = text
            .split(|&byte| byte == b'\n')
            .filter(|line| !line.is_empty())
            .collect();
        context.expect_eq(lines.len(), 3);
        for line in &lines {
            context.expect_eq(line.split(|&byte| byte == b' ').count(), 6);
        }
        for (line, device) in lines.iter().zip([
            vfs.root().filesystem().device(),
            ram_root.filesystem().device(),
            proc_root.filesystem().device(),
        ]) {
            let source = line.split(|&byte| byte == b' ').next().unwrap();
            let identity = source
                .split(|&byte| byte == b':')
                .nth(1)
                .and_then(|digits| core::str::from_utf8(digits).ok())
                .and_then(|digits| digits.parse::<u64>().ok());
            context.expect_eq(identity, Some(device));
        }
        context.expect_eq(lines[0].ends_with(b" / ext4 ro 0 0"), true);
        context.expect_eq(lines[1].ends_with(b" /mnt/extra tmpfs rw 0 0"), true);
        context.expect_eq(
            lines[2].ends_with(b" /mnt/extra/b/moved/raw\\040\\011\\012\\134\\043 proc ro 0 0"),
            true,
        );
        drop(proc_root);
        checked(context, vfs.unmount(&nested))?;
        context.expect_eq(
            checked(context, vfs.mounts_text(&vfs.root()))?
                .split(|&byte| byte == b'\n')
                .filter(|line| !line.is_empty())
                .count(),
            2,
        );
        let devfs = checked(context, MemoryFs::try_new(MemoryFsKind::Devfs, 0o755))?;
        checked(
            context,
            devfs.initialize_special(&devfs.root(), b"null", MemorySpecial::Null, 0o20600, 0x103),
        )?;
        checked(
            context,
            vfs.mount(&nested, Filesystem::Memory(devfs), AccessMode::ReadOnly),
        )?;
        let device = fixture.resolve(context, b"/mnt/extra/b/moved/raw \t\n\\#/null")?;
        context.expect_eq(device.check_writable(), Err(FsError::ReadOnly));
        context.expect_eq(device.check_access(0, 0, 2), Ok(()));
        context.expect_eq(device.check_access(1, 1, 2), Err(FsError::Permission));
        context.expect_eq(
            vfs.unlink_from(
                fs_context,
                fs_context.root(),
                b"/mnt/extra/b/moved/raw \t\n\\#/null",
                (0, 0),
            )
            .err(),
            Some(FsError::ReadOnly),
        );
        drop(device);
        checked(context, vfs.unmount(&nested))?;
        checked(
            context,
            vfs.unlink_from(
                fs_context,
                fs_context.root(),
                b"/mnt/extra/b/moved/file",
                (0, 0),
            ),
        )?;
        context.expect_eq(checked(context, inode.metadata_snapshot())?.links, 1);
        checked(
            context,
            vfs.unlink_from(fs_context, fs_context.root(), b"/mnt/extra/b/hard", (0, 0)),
        )?;
        context.expect_eq(checked(context, reopened.inode())?.ptr_eq(&inode), true);
        context.expect_eq(checked(context, inode.metadata_snapshot())?.links, 0);
        drop(reopened);
        drop(nested);
        drop(parent);
        drop(ram_root);
        checked(context, vfs.unmount(&covered))?;
        context.expect_eq(
            fixture.resolve(context, b"/mnt/extra")?.ptr_eq(&covered),
            true,
        );
        Ok(())
    }
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", tags("fs-path", "fs-path-prune"), resources("root"), timeout_ms = 30000)]
pub(crate) mod pruning {
    use super::*;

    /// 正负名字有界回收不修改盘面，活孩子仍保持父身份，放弃后可重新查得同一文件。
    #[test]
    #[ktest(tags("B23"))]
    fn clean_positive_negative_and_parent(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs();
        let root = vfs.root();
        let file = fixture.resolve(context, b"/pattern")?;
        read_pattern(context, &file)?;
        let number = checked(context, file.inode())?.number();
        let loads = vfs.dcache().ktest_load_count();
        context.expect_eq(
            lookup_error(vfs, root.dentry(), b"absent"),
            Some(FsError::NotFound),
        );
        context.expect_eq(
            lookup_error(vfs, root.dentry(), b"absent"),
            Some(FsError::NotFound),
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), loads + 1);
        context.expect_eq(vfs.dcache().prune_unused(), 1);
        context.expect_eq(fixture.resolve(context, b"/pattern")?.ptr_eq(&file), true);
        drop(file);
        context.expect_eq(vfs.dcache().prune_unused(), 1);
        context.expect_eq(vfs.dcache().is_empty(), true);
        let reread = fixture.resolve(context, b"/pattern")?;
        context.expect_eq(checked(context, reread.inode())?.number(), number);
        read_pattern(context, &reread)?;
        context.expect_eq(
            lookup_error(vfs, root.dentry(), b"absent"),
            Some(FsError::NotFound),
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), loads + 3);
        let sub = fixture.resolve(context, b"/walk/sub")?;
        let parent = sub
            .dentry()
            .parent()
            .ok_or(TestError("path child parent absent"))?;
        vfs.dcache().prune_unused();
        context.expect_eq(
            fixture.resolve(context, b"/walk")?.dentry().ptr_eq(&parent),
            true,
        );
        context.expect_eq(
            fixture
                .resolve(context, b"/walk/sub/..")?
                .dentry()
                .ptr_eq(&parent),
            true,
        );
        Ok(())
    }

    /// 两个 inode 槽顺序读取已关闭的文件，名字缓存不得阻止有限回收和下一次真实查找。
    #[test]
    #[ktest(tags("B23", "fs-path-cache-budget"))]
    fn inode_budget_reclaims_unused_names(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let first = fixture.resolve(context, b"/pattern")?;
        read_pattern(context, &first)?;
        context.expect_eq(fixture.vfs().inodes().len(), 2);
        drop(first);

        let empty = fixture.resolve(context, b"/empty")?;
        let inode = checked(context, empty.inode())?;
        context.expect_eq(checked(context, inode.metadata_snapshot())?.size, 0);
        let mut bytes = [0xa7; 1];
        context.expect_eq(checked(context, inode.read_at(0, &mut bytes))?, 0);
        context.expect_eq(fixture.vfs().inodes().len(), 2);
        drop(empty);

        let again = fixture.resolve(context, b"/pattern")?;
        read_pattern(context, &again)?;
        context.expect_eq(fixture.vfs().inodes().len(), 2);
        Ok(())
    }
}

/// 同名加载停点只挡 pattern，真实等待者登记后才放行首次真实后端查找。
fn same_name(context: &mut TestContext<'_>, fixture: &mut PathOwner, failure: bool) -> TestResult {
    fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
    let vfs = fixture.vfs().clone();
    let root = vfs.root();
    let before_len = vfs.dcache().len();
    let before_load = vfs.dcache().ktest_load_count();
    fixture
        .owner
        .observer()
        .hold_loader
        .store(true, Ordering::Release);
    for index in 0..3 {
        fixture.owner.publish(
            index,
            Job::Lookup {
                vfs: vfs.clone(),
                parent: root.dentry().clone(),
                name: b"pattern",
            },
        );
        if index == 0 {
            until("path loader did not reach published InLookup", || {
                fixture
                    .owner
                    .observer()
                    .loader_entered
                    .load(Ordering::Acquire)
                    == 1
                    && fixture.owner.observer().blocked(0)
            })?;
        }
    }
    until("path same-name waiters did not register", || {
        vfs.dcache().ktest_waiting(root.dentry(), b"pattern") == 2
            && fixture.owner.observer().blocked(1)
            && fixture.owner.observer().blocked(2)
    })?;
    context.expect_eq(vfs.dcache().ktest_load_count(), before_load + 1);
    fixture
        .owner
        .observer()
        .hold_loader
        .store(false, Ordering::Release);
    fixture.owner.observer().gate.complete();
    fixture.owner.join_tasks()?;
    let results = core::mem::replace(
        &mut *fixture.owner.observer().results.lock(),
        [const { None }; 3],
    );
    if failure {
        for result in results {
            context.expect_eq(result.and_then(Result::err), Some(FsError::Checksum));
        }
        context.expect_eq(vfs.dcache().len(), before_len);
        context.expect_eq(
            lookup_error(&vfs, root.dentry(), b"pattern"),
            Some(FsError::Checksum),
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), before_load + 2);
        context.expect_eq(vfs.dcache().len(), before_len);
    } else {
        let mut first: Option<DentryRef> = None;
        for result in results {
            let returned = checked(
                context,
                result.ok_or(TestError("path lookup result absent"))?,
            )?;
            let Returned::Dentry(dentry) = returned else {
                return Err(TestError("path lookup returned wrong kind"));
            };
            context.expect_eq(
                checked(
                    context,
                    checked(context, dentry.inode())?.metadata_snapshot(),
                )?
                .size,
                PATTERN_SIZE,
            );
            if let Some(first) = &first {
                context.expect_eq(dentry.ptr_eq(first), true);
                context.expect_eq(
                    checked(context, dentry.inode())?.ptr_eq(checked(context, first.inode())?),
                    true,
                );
            } else {
                first = Some(dentry);
            }
        }
        let path = fixture.resolve(context, b"/pattern")?;
        read_pattern(context, &path)?;
        context.expect_eq(vfs.dcache().ktest_load_count(), before_load + 1);
    }
    context.expect_eq(vfs.dcache().ktest_waiting(root.dentry(), b"pattern"), 0);
    Ok(())
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-path", "fs-path-loading"), resources("root"), timeout_ms = 30000)]
pub(crate) mod loading {
    use super::*;

    /// 三个原生任务消费同一名字与 inode，首次后端查找只有一个胜出者。
    #[test]
    fn registered_waiters_share_name(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        same_name(context, fixture, false)
    }

    /// 一个名字停在已发表加载点时，另一原生任务仍能完整读出不同名字。
    #[test]
    #[ktest(tags("B15"))]
    fn other_name_progresses_while_loader_held(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs().clone();
        let root = vfs.root();
        fixture
            .owner
            .observer()
            .hold_loader
            .store(true, Ordering::Release);
        fixture.owner.publish(
            0,
            Job::Lookup {
                vfs: vfs.clone(),
                parent: root.dentry().clone(),
                name: b"pattern",
            },
        );
        until("path independent loader did not stop", || {
            fixture
                .owner
                .observer()
                .loader_entered
                .load(Ordering::Acquire)
                == 1
                && fixture.owner.observer().blocked(0)
        })?;
        fixture.owner.publish(
            1,
            Job::Resolve {
                vfs: vfs.clone(),
                context: fixture.context().clone(),
                pathname: b"/empty",
                mode: ResolveMode::Follow,
            },
        );
        until("other path name did not complete independently", || {
            fixture.owner.observer().results.lock()[1].is_some()
        })?;
        context.expect_eq(fixture.owner.observer().blocked(0), true);
        let returned = fixture.owner.observer().results.lock()[1]
            .take()
            .ok_or(TestError("independent path result absent"))?;
        let Returned::Path(path) = checked(context, returned)? else {
            return Err(TestError("independent path returned wrong kind"));
        };
        let mut bytes = [0xa7; 1];
        context.expect_eq(
            checked(
                context,
                checked(context, path.inode())?.read_at(0, &mut bytes),
            )?,
            0,
        );
        context.expect_eq(
            checked(context, checked(context, path.inode())?.metadata_snapshot())?.kind,
            InodeKind::Regular,
        );
        fixture
            .owner
            .observer()
            .hold_loader
            .store(false, Ordering::Release);
        fixture.owner.observer().gate.complete();
        fixture.owner.join_tasks()?;
        let returned = fixture.owner.observer().results.lock()[0]
            .take()
            .ok_or(TestError("held path result absent"))?;
        let Returned::Dentry(dentry) = checked(context, returned)? else {
            return Err(TestError("held lookup returned wrong kind"));
        };
        context.expect_eq(
            checked(
                context,
                checked(context, dentry.inode())?.metadata_snapshot(),
            )?
            .size,
            PATTERN_SIZE,
        );
        let path = fixture.resolve(context, b"/pattern")?;
        read_pattern(context, &path)?;
        context.diagnostic(format_args!("path-other-name held-InLookup=1 other-native-complete=1 simultaneous-hardware-IO-covered=0"));
        Ok(())
    }
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-path", "fs-path-checksum"), resources("root"), timeout_ms = 30000)]
pub(crate) mod checksum {
    use super::*;

    /// 普通 inode CRC 副本使两个真实等待者收到原 Checksum，失败名字不成为负缓存。
    #[test]
    #[ktest(tags("B12"))]
    fn checksum_wakes_waiters_and_unhashes(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        same_name(context, fixture, true)
    }
}

#[ktest::suite(fixture = PathOwner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-path", "fs-oom", "fs-path-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod oom {
    use super::*;

    /// 真实页和堆压力下命中保持身份，新名字失败不留下占位，放压后可读原盘文件。
    #[test]
    #[ktest(tags("B23"))]
    fn real_pressure_miss_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        fixture.prepare_path(context, AccessMode::ReadOnly, 4)?;
        let vfs = fixture.vfs().clone();
        let cached = fixture.resolve(context, b"/pattern")?;
        read_pattern(context, &cached)?;
        let root = vfs.root();
        let pressure = fixture
            .owner
            .pressure
            .take()
            .ok_or(TestError("path heap pressure absent"))?;
        fixture.owner.publish(
            0,
            Job::LookupOom {
                vfs: vfs.clone(),
                parent: root.dentry().clone(),
                cached: cached.dentry().clone(),
                name: b"empty",
                pressure,
            },
        );
        fixture.owner.join_tasks()?;
        let returned = fixture.owner.observer().results.lock()[0]
            .take()
            .ok_or(TestError("path OOM result absent"))?;
        let Returned::LookupOom {
            producer_cpu,
            evidence,
            same_dentry,
            error,
            len_before,
            len_after,
            submitted_before,
            submitted_after,
        } = checked(context, returned)?
        else {
            return Err(TestError("path OOM returned wrong kind"));
        };
        context.expect_eq(producer_cpu, 0);
        context.expect_eq(evidence, (true, true));
        context.expect_eq(same_dentry, true);
        context.expect_eq(error, Some(FsError::NoMemory));
        context.expect_eq(len_after, len_before);
        context.expect_eq(submitted_after, submitted_before);
        context.expect_eq(vfs.dcache().ktest_waiting(root.dentry(), b"empty"), 0);
        let empty = fixture.resolve(context, b"/empty")?;
        let mut bytes = [0xa7; 1];
        context.expect_eq(
            checked(
                context,
                checked(context, empty.inode())?.read_at(0, &mut bytes),
            )?,
            0,
        );
        context.expect_eq(
            checked(
                context,
                checked(context, empty.inode())?.metadata_snapshot(),
            )?
            .kind,
            InodeKind::Regular,
        );
        context.expect_eq(fixture.resolve(context, b"/pattern")?.ptr_eq(&cached), true);
        read_pattern(context, &cached)?;
        Ok(())
    }
}
