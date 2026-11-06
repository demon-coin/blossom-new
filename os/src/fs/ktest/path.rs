//! VFS 路径解析、目录项缓存（`DentryCache`）、符号链接跟随、子树根边界、挂载点穿越与并发加载测试套件。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统内核测试子系统（`fs::ktest`）中，负责验证 `Vfs`、`FsContext`、`Path` 与 `DentryCache`
//! 的路径查找与挂载生命周期语义。上游由 `fs::ktest` 测试注册表挂载执行，下游驱动 `Vfs`、`Ext4Volume`、
//! `MemoryFs`（`tmpfs`/`procfs`/`devfs`）以及并发测试工作线程（`Owner`）。
//!
//! # 核心设计与目录项并发加载状态机
//! 本模块围绕两项核心机制展开测试：
//! 1. **挂载树与路径边界遍历**：验证 `.`、`..`、多余 `/`、非 UTF-8 原始字节文件名、最多 40 层符号链接展开、
//!    显式子树根（`chroot` 风格边界）以及跨磁盘/内存文件系统的挂载覆盖（Mount Shadowing）与忙状态（`FsError::Busy`）卸载保护。
//! 2. **目录项单飞（Singleflight）并发加载协议**：验证多个线程并发查找同一未缓存目录项时的等待合并与唤醒行为：
//!
//! ```text
//!   线程 0 (Loader)                      线程 1, 2 (Waiters)
//!   ----------------                     -------------------
//!   lookup("pattern")
//!     |-> 插入 InLookup 占位槽
//!     |-> 触发 lookup_hook 阻塞在 gate    lookup("pattern")
//!     |                                    |-> 命中 InLookup 占位槽
//!     |                                    |-> 登记为等待者并睡眠等待
//!     |   (此刻另一线程查找 "/empty" 不受阻塞，可独立完成)
//!     |<-- gate.complete() 放行
//!     |-> 完成后端磁盘目录查找与 Inode 加载
//!     |   - 成功: 替换为正缓存 Dentry，唤醒线程 1, 2 共享同一 DentryRef 与 InodeRef
//!     |   - 失败(如 Checksum): 移除占位槽(不留负缓存)，唤醒线程 1, 2 传播相同错误
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试线程与后台工作线程遵循 VFS 锁层级：挂载表锁 -> `DentryCache` 分片锁 -> `InodeCache` 锁 -> 底层卷元数据锁。
//! - `lookup_hook` 在 `DentryCache` 释放内部自旋锁后、发起后端磁盘查找前触发阻塞，因此在 `gate.wait()` 期间不会持有自旋锁，其他无关目录项的查找可以并发推进。
//! - 测试收尾（`quiesce`）必须先卸载 `lookup_hook` 并打开 `gate` 唤醒所有阻塞的工作线程，等待线程全部汇合（Join）并释放持有的 `Path`/`Vfs`/`Ext4Volume` 后，再停止底层块设备。

use crate::fs::dentry::DentryRef;
use crate::fs::file::{open_path_with_flags, FileIoContext};
use crate::fs::ktest::*;
use crate::fs::memfs::{Filesystem, MemoryFs, MemoryFsKind, MemorySpecial};
use crate::fs::{FsContext, Path, ResolveMode, Vfs};

/// 路径与挂载测试夹具，在底层块设备夹具 `Owner` 之上维护当前用例的 `Vfs`、查找上下文与根卷实例。
pub(super) struct PathOwner {
    /// 底层块设备、Inode 缓存、并发工作线程与内存压力夹具。
    pub(super) owner: Owner,
    /// 当前测试用例创建的虚拟文件系统实例；在 `prepare_path` 后初始化，在 `release_view` 时释放。
    vfs: Option<Arc<Vfs>>,
    /// 当前测试用例的默认文件系统路径上下文（持有根目录与当前工作目录 `Path`）。
    context: Option<FsContext>,
    /// 当前测试用例打开的根 ext4 卷引用。
    volume: Option<Arc<Ext4Volume>>,
}

impl PathOwner {
    /// 按指定的访问模式 `mode` 与最大挂载容量 `mount_capacity` 打开 ext4 根卷并构建 `Vfs` 与 `FsContext`，同时安装针对 `"pattern"` 目录项的并发查找拦截钩子。
    pub(super) fn prepare_path(
        &mut self,
        context: &TestContext<'_>,
        mode: AccessMode,
        mount_capacity: usize,
    ) -> TestResult {
        // Step 1: 清除 Inode 缓存上的加载钩子，确保门控（gate）仅在目录项查找阶段拦截，避免在 Inode 加载阶段重复阻塞。
        self.owner.cache().ktest_set_load_hook(None);
        // Step 2: 打开底层 ext4 卷并创建 `Vfs` 实例与根路径上下文 `FsContext`。
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

        // Step 3: 注册 `DentryCache` 查找钩子：当 `hold_loader` 置位且查找目标为 `"pattern"` 时，记录进入计数并在门控上阻塞等待。
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

    /// 获取当前已初始化的 `Vfs` 实例引用；若在初始化前或 `release_view` 后调用则触发 panic。
    pub(super) fn vfs(&self) -> &Arc<Vfs> {
        self.vfs.as_ref().expect("path view consumed before body")
    }

    /// 获取当前已初始化的 `FsContext` 查找上下文引用（包含测试根目录与当前工作目录）。
    pub(super) fn context(&self) -> &FsContext {
        self.context
            .as_ref()
            .expect("path context consumed before body")
    }

    /// 移除 `DentryCache` 查找钩子，并按顺序释放 `FsContext`、`Vfs` 与 `Ext4Volume` 强引用。
    pub(super) fn release_view(&mut self) {
        if let Some(vfs) = &self.vfs {
            vfs.dcache().ktest_set_lookup_hook(None);
        }
        drop(self.context.take());
        drop(self.vfs.take());
        drop(self.volume.take());
    }

    /// 在默认 `FsContext` 下以跟随符号链接模式（`ResolveMode::Follow`）解析路径 `pathname`。
    fn resolve(&self, context: &TestContext<'_>, pathname: &[u8]) -> Result<Path, TestError> {
        checked(
            context,
            self.vfs()
                .resolve(self.context(), pathname, ResolveMode::Follow),
        )
    }

    /// 停止测试夹具：先移除查找钩子并放行门控以唤醒所有阻塞的工作线程，等待工作线程全部退出后释放 VFS 视图并停止底层块设备。
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
    /// 初始化测试夹具的底层块设备与工作线程资源（`Vfs` 视图留待测试正文中按需调用 `prepare_path` 构建）。
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

    /// 在测试结束或失败时执行静默化清理，确保所有后台任务汇合且块设备安全停止。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        PathOwner::quiesce(self, context)
    }

    /// 销毁测试夹具，释放残留的 `FsContext`、`Vfs` 与 `Ext4Volume` 引用，并校验底层设备与缓存的引用计数归零。
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

/// 辅助验证函数：通过 `path` 获取目标 `Inode`，校验其文件大小等于 `PATTERN_SIZE`，并读取前 4096 字节验证内容符合预期测试图样。
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

/// 辅助查找函数：在 `parent` 目录下查找 `name` 并尝试获取其关联 `Inode`，提取查找或加载过程中产生的 `FsError`。
fn lookup_error(vfs: &Vfs, parent: &DentryRef, name: &[u8]) -> Option<FsError> {
    vfs.dcache()
        .lookup(vfs.inodes(), parent, name)
        .and_then(|dentry| dentry.inode().map(|_| ()))
        .err()
}

/// 返回本模块下全部 7 个路径测试套件的静态注册槽位数组，供上层 `fs::ktest` 统一挂载。
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

    /// 验证路径组件解析规则、`.` 与 `..` 导航、非目录组件拦截、非法名称过滤、硬链接身份共享以及非 UTF-8 原始字节文件名读取。
    ///
    /// # 测试目标
    /// - 验证连续斜杠 `////`、`.`、`..` 在存在目录上的正确折叠与回退，以及不存在的中间组件（如 `/absent/..`）不会被 `..` 静默消解而是返回 `FsError::NotFound`。
    /// - 验证将普通文件当作目录遍历（如 `/pattern/`、`/pattern/.`）返回 `FsError::NotDirectory`，含空字节 `\0`、斜杠或超长名称返回 `InvalidName` / `NameTooLong`。
    /// - 验证硬链接 `/pattern` 与 `/hardlink` 拥有各自独立的 `Dentry` 但共享同一个 `Inode` 与 `AddressSpace` 实例。
    /// - 验证非 UTF-8 原始字节文件名 `/name-\xff\xfe` 可正常解析并读取内容。
    #[test]
    #[ktest(tags("B16", "B17", "B19"))]
    fn components_bytes_and_types(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 准备只读 VFS 并验证多斜杠、`.` 与 `..` 在合法目录层级上的解析结果。
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

        // Step 2: 验证缺失父目录、在普通文件后追加路径组件、非法字符与超长组件名等错误分类。
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

        // Step 3: 验证两个硬链接路径拥有不同 `Dentry` 但指向同一 `Inode` 与页缓存映射，且均可读出完整图样。
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

        // Step 4: 验证包含非 UTF-8 字节的文件名解析与内容读取。
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

    /// 验证同一个可写 `Ext4Volume` 同时挂载到只读（RO）与可写（RW）两个 `Vfs` 视图时，两者共享底层卷与 `Inode` 实例，且只读视图拒绝写权限检查与写打开请求。
    #[test]
    #[ktest(tags("W01"))]
    fn rw_volume_keeps_readonly_mount_view(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 两次以可写模式打开同一块设备视图，验证返回同一个 `Ext4Volume` 强引用实例。
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

        // Step 2: 基于该可写卷分别构建只读 `Vfs`（`ro`）和可写 `Vfs`（`rw`），解析同一文件 `/pattern`。
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

        // Step 3: 验证两个视图解析出的路径指向同一 `Inode`，只读视图拒绝写访问（`FsError::ReadOnly`），而两个视图均可正常读取文件内容。
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

    /// 验证在内存根文件系统（`tmpfs`）和 `InodeCache` 持续存活期间，同一块设备卷历经“只读挂载 -> 卸载 -> 可写重新挂载修改同步 -> 卸载 -> 只读重新挂载”全流程的数据持久化与生命周期管理。
    #[test]
    #[ktest(tags("fs-mount-lifetime"))]
    fn ro_unmount_rw_reopen_persists_file(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 以 `tmpfs` 作为根文件系统创建 `Vfs`，建立 `/disk` 挂载点并以只读模式首次挂载 ext4 卷，读取校验 `/disk/pattern` 内容后卸载。
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

        // Step 2: 以可写模式重新打开并挂载同一块设备卷，创建临时文件、写入数据、重命名、截断并执行 `sync` 刷盘后再次卸载。
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

        // Step 3: 再次以只读模式打开并挂载该卷，验证重命名前的旧路径不存在，重命名并截断后的文件大小与内容已持久化到磁盘。
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

    /// 验证相对/绝对符号链接解析、`ResolveMode::Follow` 与 `ResolveMode::NoFollow` 切换、末尾斜杠强制目录语义以及 40 层符号链接展开上限（`FsError::TooManySymlinks`）。
    #[test]
    #[ktest(tags("B18"))]
    fn links_follow_modes_and_limit(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 在工作目录 `/walk` 下跟随解析相对符号链接、绝对符号链接与双跳符号链接链，验证均指向 `/pattern`。
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

        // Step 2: 验证交替使用 `Follow` 与 `NoFollow` 解析同一缓存符号链接时，分别返回目标普通文件与符号链接自身。
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

        // Step 3: 验证指向目录的符号链接在末尾带 `/` 或 `/..` 时即使指定 `NoFollow` 也会跟随进入目标目录，而指向普通文件的符号链接带末尾 `/` 返回 `FsError::NotDirectory`。
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

        // Step 4: 验证超过 40 层的符号链接链（`chain01`）与自环符号链接（`loop`）在 `Follow` 下返回 `FsError::TooManySymlinks`，而在 `NoFollow` 下可正常读取链接自身内容。
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

    /// 验证显式指定子树根目录（Scoped Root）时的路径越界限制与工作目录合法性校验：
    /// - `..` 回退与绝对路径（包括绝对符号链接）均被限制在子树根 `/walk` 内部，无法逃逸到上层目录；
    /// - 若工作目录不在子树根的派生层级内，或根目录/工作目录不是目录类型，则拒绝构造 `FsContext`。
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

    /// 验证跨磁盘挂载点穿越、父目录 `..` 回退、同卷多点挂载（Bind/Alias Mount）以及活跃 `Path` 引用阻止卸载（`FsError::Busy`）的生命周期语义。
    ///
    /// # 测试目标
    /// - 验证在 `/mnt/extra` 挂载第二块 ext4 磁盘后，路径解析切换到新卷根 Inode，且 `/mnt/extra/..` 正确回退到宿主挂载点的父目录 `/mnt`。
    /// - 验证同一 `Ext4Volume` 同时挂载到 `/mnt/extra` 与 `/walk` 时，两个挂载根拥有不同 `Path` 但共享同一个根 `Inode`。
    /// - 验证只要存在指向挂载子树内部目录或文件的活跃 `Path`（或 `FsContext`）引用，`unmount` 即返回 `FsError::Busy`；待全部引用释放后方可成功卸载并恢复被遮蔽的原目录视图。
    #[test]
    #[ktest(tags("B16", "B18", "B23"))]
    fn second_disk_busy_release_and_parent(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 打开第二块磁盘的 ext4 卷并挂载到 `/mnt/extra`，验证挂载根切换、跨卷 Inode 身份隔离以及 `/mnt/extra/..` 回退到 `/mnt`。
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

        // Step 2: 验证以挂载点为子树根时的 `..` 边界，并读取挂载卷内的 `/mnt/extra/pattern` 文件。
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

        // Step 3: 将同一个卷再次挂载到 `/walk`，验证共享根 Inode、活跃引用阻止卸载与重复挂载返回 `FsError::Busy`。
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

        // Step 4: 逐步释放 `extra_context`、`mounted` 与 `file` 引用，验证在最后一个内部引用释放前卸载返回 `FsError::Busy`，全部释放后卸载成功并恢复原 `/mnt/extra` 目录。
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

        // Step 5: 清理缓存后再次挂载并重读文件，验证卸载后再挂载工作正常且不影响宿主根卷文件读取。
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

    /// 验证在同一 VFS 中混合挂载 `tmpfs`、`procfs` 与 `devfs` 内存文件系统时的目录增删、硬链接/符号链接、目录重命名、跨设备硬链接拦截、`/proc/mounts` 特殊字符八进制转义以及设备节点权限检查。
    ///
    /// # 测试目标
    /// - 验证在 `tmpfs` 中创建含空格、制表符、换行、反斜杠与 `#` 的目录名，验证空目录删除、非空目录拒绝删除（`FsError::NotEmpty`）、同卷硬链接与符号链接创建。
    /// - 验证目录重命名后 `..` 自动指向新父目录，拒绝将父目录重命名到自身子孙目录中（`FsError::InvalidOperation`），并拒绝跨文件系统创建硬链接（`FsError::CrossDevice`）。
    /// - 验证嵌套挂载 `procfs` 后，`mounts_text` 生成的挂载表快照正确对挂载路径中的空白和特殊字符进行八进制转义（如 `\040`、`\011`、`\012`、`\134`、`\043`）。
    /// - 验证只读挂载 `devfs` 下的字符设备节点（`null`）遵守只读挂载限制与 UID/GID 权限位检查，最后验证删除全部硬链接后链接计数归零但已打开的 `Path` 仍保持同一 `Inode`。
    #[test]
    #[ktest(tags("T01", "T30", "fs-path-memory"))]
    fn memory_names_and_actual_mount_snapshot(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 在 `/mnt/extra` 挂载可写 `tmpfs`，创建包含特殊字符的目录层级，并验证 `rmdir` 对空目录和非空目录的处理。
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

        // Step 2: 在 `tmpfs` 内创建普通文件、硬链接与相对符号链接，验证三者解析到同一个 `Inode`。
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

        // Step 3: 将目录 `/mnt/extra/a` 重命名为 `/mnt/extra/b/moved`，验证 `..` 父指针更新、拒绝目录自环移动以及拒绝跨卷硬链接。
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

        // Step 4: 在含特殊字符的子目录上嵌套只读挂载 `procfs`，校验 `mounts_text` 输出的设备号、文件系统类型及八进制转义路径。
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

        // Step 5: 卸载 `procfs` 后在同一位置只读挂载 `devfs` 并初始化 `null` 设备节点，验证只读挂载拦截与 root/非 root 用户权限检查。
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

        // Step 6: 依次 `unlink` 删除文件的两个硬链接路径，验证链接计数递减至 0，释放全部活跃 `Path` 引用后卸载 `tmpfs`。
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

    /// 验证 `DentryCache` 对正缓存项（Positive Dentry）、负缓存项（Negative Dentry）及父目录引用的按需回收（`prune_unused`）行为。
    ///
    /// # 测试目标
    /// - 验证查找不存在的名称后生成负缓存项，重复查找直接命中负缓存而不触发后端磁盘查找；调用 `prune_unused` 可回收无外部引用的负缓存项。
    /// - 验证被外部 `Path` 持有的正缓存项不会被 `prune_unused` 回收；释放 `Path` 后正缓存项可被回收，再次解析时重新从磁盘加载同一 Inode 号。
    /// - 验证当子目录 `/walk/sub` 被外部持有时，其父目录 `/walk` 通过子目录的父指针保持存活，不会在 `prune_unused` 中丢失身份。
    #[test]
    #[ktest(tags("B23"))]
    fn clean_positive_negative_and_parent(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 解析 `/pattern` 建立正缓存，查找两次 `"absent"` 验证第二次命中负缓存，随后调用 `prune_unused` 回收负缓存项而保留被持有的 `/pattern`。
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

        // Step 2: 释放 `/pattern` 的 `Path` 引用并调用 `prune_unused` 清空目录项缓存，验证重新解析与再次查找 `"absent"` 均触发后端重新加载。
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

        // Step 3: 持有子目录 `/walk/sub` 并执行 `prune_unused`，验证父目录 `/walk` 的 `Dentry` 被子目录保活且身份不变。
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

    /// 验证当 `InodeCache` 容量受限时（仅 2 个槽位：根目录占 1 个，普通文件占 1 个），关闭上一个文件后，`DentryCache` 中未使用的缓存项不会阻碍 `InodeCache` 回收旧 Inode 并加载新文件。
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

/// 辅助并发测试驱动函数：启动 3 个工作线程同时查找根目录下的 `"pattern"`，验证 `DentryCache` 的单飞（Singleflight）并发加载与等待者唤醒机制。
///
/// - 当 `failure == false` 时，验证仅首个线程执行后端查找，其余两个等待线程被唤醒后共享完全相同的 `DentryRef` 与 `InodeRef`。
/// - 当 `failure == true` 时（配合损坏的 Inode CRC 镜像），验证全部 3 个线程均收到 `FsError::Checksum`，且失败条目不会残留在 `DentryCache` 中成为负缓存。
fn same_name(context: &mut TestContext<'_>, fixture: &mut PathOwner, failure: bool) -> TestResult {
    // Step 1: 开启 `hold_loader` 拦截标志并依次发布 3 个查找 `"pattern"` 的工作任务，等待线程 0 进入 `InLookup` 阻塞点且线程 1、2 登记为等待者。
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

    // Step 2: 清除拦截标志并打开门控放行线程 0，等待全部 3 个工作线程执行完毕。
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
    // Step 3: 根据 `failure` 模式校验返回结果：失败模式下验证错误传播且未残留缓存项；成功模式下验证 3 个线程共享同一 `Dentry` 与 `Inode` 且后端仅加载 1 次。
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

    /// 验证 3 个并发内核线程同时查找同一未缓存目录项 `"pattern"` 时，仅由单个加载线程发起后端磁盘查找，其余等待线程登记等待并在完成后共享同一 `Dentry` 与 `Inode` 实例。
    #[test]
    fn registered_waiters_share_name(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        same_name(context, fixture, false)
    }

    /// 验证当一个线程在查找 `"pattern"` 处于 `InLookup` 阻塞等待状态时，另一个线程查找不相关的路径 `"/empty"` 不会被阻塞，能够独立完成路径解析与文件读取。
    #[test]
    #[ktest(tags("B15"))]
    fn other_name_progresses_while_loader_held(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 开启 `hold_loader` 并让工作线程 0 在查找 `"pattern"` 时阻塞于 `InLookup` 状态。
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

        // Step 2: 在线程 0 仍处于阻塞状态时，发布工作线程 1 解析 `"/empty"`，验证线程 1 独立完成解析并成功读取空文件。
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

        // Step 3: 放行门控唤醒线程 0，验证 `"pattern"` 查找随后正常完成且文件内容可读。
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

    /// 验证并发查找的目标 Inode 发生 CRC 校验失败时，加载线程与所有已登记的等待线程均收到 `FsError::Checksum`，且失败的目录项从缓存中移除（不转为负缓存）。
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

    /// 验证在物理页与内核堆同时耗尽（OOM）的压力下，已缓存的目录项仍能正常命中并保持同一 `Dentry` 身份，未命中的新目录项查找返回 `FsError::NoMemory` 且不在 `DentryCache` 中残留占位项，解除内存压力后可正常解析读取。
    #[test]
    #[ktest(tags("B23"))]
    fn real_pressure_miss_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut PathOwner,
    ) -> TestResult {
        // Step 1: 预先解析并缓存 `"/pattern"`，随后将内存压力容器移交给工作线程，在 OOM 条件下测试缓存命中（`"pattern"`）与未命中查找（`"empty"`）。
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

        // Step 2: 校验工作线程在 OOM 期间的观测结果：已缓存目录项命中原实例，新名称查找返回 `FsError::NoMemory`，缓存条目数与块设备 I/O 提交数均未增加。
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

        // Step 3: 验证内存压力释放后，`"/empty"` 可正常解析并读取，且 `"/pattern"` 依旧命中原有缓存路径。
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
