//! 可写文件系统（Ext4）与用户态写路径的集成测试公共夹具与校验工具模块。
//!
//! ## 模块职责与架构定位
//! 本模块隶属于 `fs::ktest::user` 测试套件，为六大类可写文件行为测试子模块（`buffered` 缓冲写、`mmap` 共享内存映射写、
//! `namespace` 目录与命名空间变更、`reclaim` 页面回收、`truncate` 截断与打洞、`writeback` 脏页回写与同步）提供统一的
//! 生命周期管理夹具 [`WriteOwner`] 以及底层磁盘元数据校验工具。
//! 上游由 `fs::ktest::user` 通过 [`anchor`] 注册测试套件，下游驱动 `/test/bin/user-write` 用户态程序并直接检验 `Ext4Volume`
//! 的位图、Inode 记录及 Extent/间接块树结构。
//!
//! ## 核心机制与虚拟地址空间/清理状态机布局
//! 1. **共享控制页与测试映射虚拟地址布局**：
//! ```text
//! 用户态地址空间 (Mm) 关键测试区域:
//! +-----------------------+-------------------------------------------------------+
//! | 虚拟地址              | 用途与内部偏移布局                                    |
//! +-----------------------+-------------------------------------------------------+
//! | 0x4800_0000 (SOURCE)  | 用户态写操作的数据源缓冲区映射基址                    |
//! +-----------------------+-------------------------------------------------------+
//! | 0x4900_0000           | 4 KiB 共享匿名控制页 (SharedAnonymous):               |
//! |   (SHARED_CONTROL)    |   +0  (u32)  : 用户态发布的当前阶段检查点编号         |
//! |                       |   +4  (u32)  : 内核写入的阶段放行门控字               |
//! |                       |   +16 (usize): fork 子进程的真实线程 TID              |
//! |                       |   +24 (u64)  : 共享文件映射的目标偏移量               |
//! +-----------------------+-------------------------------------------------------+
//! | 0x4a00_0000           | 独立地址空间 (`shared_alias`) 验证共享文件页一致性的  |
//! |   (SHARED_ALIAS)      | 8 KiB 映射基址                                        |
//! +-----------------------+-------------------------------------------------------+
//! ```
//!
//! 2. **测试夹具静默与回收流程（`WriteOwner::quiesce`）**：
//! ```text
//! [1. 卸载测试钩子并唤醒所有挂起门控 (mkwrite/fault/truncate/unmap gates)]
//!         │
//!         ▼
//! [2. 恢复模拟块设备 I/O 完成，等待内核消费线程与用户进程完全退出内核栈并回收]
//!         │
//!         ▼
//! [3. 逐个 Inode 执行 sync_inode 回写文件脏页，验证无残留 Dirty/Writeback 页]
//!         │
//!         ▼
//! [4. 若存在冷重载缓存 (cold_cache)，释放 Inode 引用并等待缓存驱逐异步任务完成]
//!         │
//!         ▼
//! [5. 执行 sync_metadata 回写卷元数据并下发设备 FLUSH，验证无残留脏元数据块]
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 1. 观测同步消费线程状态（[`SyncObservation`]）时，`executor` 与 `result` 各自使用独立的短临界区自旋锁，不可嵌套持有。
//! 2. 准备已解除链接（Unlinked）文件的脏页回写时，必须先等待已有回写完成（`folio.wait_writeback()`），再获取 `folio.lock_content()`
//!    内容锁，并在持锁期间调用 `mkclean` 清除共享可写页表项及完成跨核 TLB 刷新，最后启动块回写状态。

use crate::arch::UserAccess;
use crate::block::{Bio, BioAllocator, BlkOperation, IoResult, SECTOR_BYTES};
use crate::cache::FileUnmapPhase;
use crate::fs::ext4::{codec, BlockMapping, InodeData};
use crate::fs::file::{open_path_with_flags, FileIoContext, WriteBuffer};
use crate::fs::inode::{InodeCacheWatch, InodeRef, TruncatePhase};
use crate::fs::ktest::user::*;
use crate::fs::{InodeKind, SeekFrom};
use crate::mm::dma::MappedPages;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::{MmapAddress, MmapRequest, MmapSource};
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;
use crate::sync::PreemptGuard;
use crate::task::futex;
use crate::task::{ReclaimMode, ReclaimScope};
use virtio_drivers::BufferDirection;

mod buffered;
mod mmap;
mod namespace;
mod reclaim;
mod truncate;
mod writeback;

/// 可读可写、不可执行的用户页权限常量。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
/// 只读、不可执行的用户页权限常量。
const RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};

/// 用户态写测试使用的源数据映射虚拟地址。
const SOURCE: usize = 0x4800_0000;
/// 用户态可写文件测试程序的路径。
const PROGRAM_WRITE: &[u8] = b"/test/bin/user-write";

/// 用户态与内核测试线程共享的匿名控制页虚拟地址。
const SHARED_CONTROL: usize = 0x4900_0000;
/// 共享控制页中存放文件映射偏移量的字节偏移。
const SHARED_FILE_OFFSET: usize = 24;
/// 独立验证地址空间中映射同一文件页的别名虚拟地址。
pub(super) const SHARED_ALIAS: usize = 0x4a00_0000;

/// 异步内核同步调用（如 `fsync` / `sync_inode`）消费线程的调度状态与返回结果观察器。
pub(super) struct SyncObservation {
    /// 正在执行同步调用的内核线程弱引用，由自旋锁保护。
    executor: SpinLock<Option<alloc::sync::Weak<Task>>>,
    /// 同步调用完成后的返回值，未完成时为 `None`，由自旋锁保护。
    result: SpinLock<Option<Result<(), FsError>>>,
}

impl SyncObservation {
    /// 检查执行同步调用的内核线程当前是否处于调度器的 `Blocked`（睡眠阻塞）状态。
    pub(super) fn blocked(&self) -> bool {
        self.executor
            .lock()
            .as_ref()
            .and_then(alloc::sync::Weak::upgrade)
            .is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 读取同步调用线程已写入的最终执行结果；若调用尚未返回则返回 `None`。
    pub(super) fn result(&self) -> Option<Result<(), FsError>> {
        *self.result.lock()
    }
}

/// 文件截断（Truncate）与页表反向映射解除（Unmap）并发测试的阶段同步控制器。
struct TruncateControls {
    /// 截断钩子当前到达的阶段计数。
    phase: AtomicUsize,
    /// 测试进入错误清理流程时的取消标志，置位后放行所有钩子不再阻塞。
    cancelled: AtomicBool,
    /// 用于在截断或反向映射的关键阶段暂停与恢复执行线程的三个完成门控。
    gates: [Completion; 3],
    /// 页面缓存解除映射钩子（`FileUnmapPhase`）当前到达的阶段计数。
    unmap_phase: AtomicUsize,
    /// 跨核 TLB 刷新同步（CSD）开始前记录的阶段快照。
    before_sync: SpinLock<Option<(usize, usize)>>,
    /// 跨核 TLB 刷新同步（CSD）完成后记录的阶段快照。
    after_sync: SpinLock<Option<(usize, usize)>>,
}

impl TruncateControls {
    /// 分配并初始化截断与反向映射并发测试的同步控制器。
    fn new() -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            phase: AtomicUsize::new(0),
            cancelled: AtomicBool::new(false),
            gates: core::array::from_fn(|_| Completion::new()),
            unmap_phase: AtomicUsize::new(0),
            before_sync: SpinLock::new(None),
            after_sync: SpinLock::new(None),
        })
        .map_err(|_| TestError("truncate controls OOM"))
    }
}

/// 可写文件系统与用户态写测试的核心夹具，负责统一管理用户进程、打开的 Inode、后台同步线程及冷重载卷的生命周期。
pub(super) struct WriteOwner {
    /// 底层用户态测试环境与块设备持有者。
    pub(super) user: UserOwner,
    /// 测试期间打开或操作过的 Inode 强引用列表，保留至清理阶段以执行强制回写与脏页检查。
    pub(super) inodes: Vec<InodeRef>,
    /// 追加写（Append）并发测试使用的阶段同步门控。
    append_gate: Option<Arc<Completion>>,
    /// 当前活跃的共享控制页所属用户地址空间引用，用于在清理阶段统一卸载测试钩子。
    shared_mm: Option<Arc<Mm>>,
    /// 页面准备阶段并发测试使用的同步门控。
    prepare_gate: Option<Arc<Completion>>,
    /// 截断与反向映射并发测试使用的同步控制器。
    truncate_controls: Option<Arc<TruncateControls>>,
    /// 后台执行阻塞式文件同步操作的内核辅助线程句柄槽位。
    consumers: [Option<JoinHandle>; 2],
    /// 冷挂载验证或命名空间测试中单独持有的 Ext4 卷实例。
    pub(super) cold_volume: Option<Arc<Ext4Volume>>,
    /// 冷挂载验证中单独创建的 Inode 缓存实例。
    cold_cache: Option<Arc<InodeCache>>,
    /// 用于等待 `cold_cache` 中所有 Inode 异步回收完毕的观察句柄。
    eviction: Option<InodeCacheWatch>,
}

impl WriteOwner {
    /// 预留 Inode 跟踪槽位，并以读写模式初始化底层 Ext4 卷与根挂载命名空间。
    fn prepare(&mut self, context: &TestContext<'_>) -> TestResult {
        self.inodes
            .try_reserve_exact(16)
            .map_err(|_| TestError("write inode owner OOM"))?;
        self.user.prepare_mode(context, AccessMode::ReadWrite, 4)
    }

    /// 初始化读写命名空间测试环境，并记录卷引用以便在清理阶段强制同步元数据。
    pub(super) fn prepare_namespace(&mut self, context: &TestContext<'_>) -> TestResult {
        self.prepare(context)?;
        assert!(self.cold_volume.is_none() && self.cold_cache.is_none());
        self.cold_volume = Some(self.volume());
        Ok(())
    }

    /// 创建具有 `setgid` 继承属性（GID 1001）的工作目录 `/test/ns-work`，以及仅允许 root 访问的权限测试子目录。
    fn prepare_namespace_parent(&mut self, context: &TestContext<'_>) -> TestResult {
        let vfs = self.user.path.vfs().clone();
        let fs_context = self.user.path.context().clone();
        let root = vfs.root();
        checked(
            context,
            vfs.mkdir_from(&fs_context, &root, b"/test/ns-work", 0o777, (0, 0), 0),
        )?;
        let workspace = self.open(context, b"/test/ns-work", 0x10000)?;
        let inode = checked(context, workspace.path().unwrap().inode())?.clone();

        // mkdir 系统调用会掩去 S_ISGID 位，因此通过底层的受保护元数据更新接口显式设置父目录的 SGID 位与所属组。
        checked(
            context,
            inode.update_metadata(checked(context, inode.disk_volume())?.claim(), |metadata| {
                metadata.mode |= crate::fs::types::S_ISGID;
                metadata.gid = 1001;
            }),
        )?;
        drop(workspace);

        checked(
            context,
            vfs.mkdir_from(
                &fs_context,
                &root,
                b"/test/ns-work/ns-permission",
                0o700,
                (0, 0),
                0,
            ),
        )?;
        drop(self.open(context, b"/test/ns-work/ns-permission", 0x10000)?);
        Ok(())
    }

    /// 清理测试期间创建的 `/test/ns-work/ns-permission` 与 `/test/ns-work` 目录。
    fn remove_namespace_parent(&self, context: &TestContext<'_>) -> TestResult {
        let vfs = self.user.path.vfs();
        let fs_context = self.user.path.context();
        let root = vfs.root();
        checked(
            context,
            vfs.rmdir_from(fs_context, &root, b"/test/ns-work/ns-permission", (0, 0)),
        )?;
        checked(
            context,
            vfs.rmdir_from(fs_context, &root, b"/test/ns-work", (0, 0)),
        )?;
        Ok(())
    }

    /// 解析路径并以指定标志打开文件，同时将该文件的 Inode 纳入夹具跟踪列表以确保清理时刷盘。
    pub(super) fn open(
        &mut self,
        context: &TestContext<'_>,
        pathname: &[u8],
        flags: u32,
    ) -> Result<FileRef, TestError> {
        let path = checked(
            context,
            self.user
                .path
                .vfs()
                .resolve(self.user.path.context(), pathname, ResolveMode::Follow),
        )?;
        let inode = checked(context, path.inode())?.clone();
        if !self.inodes.iter().any(|old| old.ptr_eq(&inode)) {
            if self.inodes.len() == self.inodes.capacity() {
                return Err(TestError("write inode owner capacity"));
            }
            self.inodes.push(inode);
        }

        checked(
            context,
            open_path_with_flags(self.user.path.vfs(), path, flags, FileIoContext::Kernel),
        )
    }

    /// 以不跟随末尾符号链接（`ResolveMode::NoFollow`）的方式解析路径，并将目标本身的 Inode 纳入夹具跟踪列表。
    fn hold_nofollow(
        &mut self,
        context: &TestContext<'_>,
        pathname: &[u8],
    ) -> Result<InodeRef, TestError> {
        let path = checked(
            context,
            self.user
                .path
                .vfs()
                .resolve(self.user.path.context(), pathname, ResolveMode::NoFollow),
        )?;
        let inode = checked(context, path.inode())?.clone();
        if !self.inodes.iter().any(|old| old.ptr_eq(&inode)) {
            if self.inodes.len() == self.inodes.capacity() {
                return Err(TestError("write inode owner capacity"));
            }
            self.inodes.push(inode.clone());
        }
        Ok(inode)
    }

    /// 获取当前根文件系统挂载的底层 `Ext4Volume` 强引用。
    pub(super) fn volume(&self) -> Arc<Ext4Volume> {
        self.user
            .path
            .vfs()
            .root()
            .inode()
            .unwrap()
            .disk_volume()
            .unwrap()
            .clone()
    }

    /// 预打开测试目标文件以纳入回写跟踪，随后启动指定模式的 `/test/bin/user-write` 程序并等待其正常退出。
    fn run(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        for pathname in [
            b"/test/write-user" as &[u8],
            b"/test/write-append",
            b"/test/write-map",
        ] {
            drop(self.open(context, pathname, 0)?);
        }

        let image = self.user.load_named(context, PROGRAM_WRITE, mode)?;
        self.user.start_program(
            context,
            image,
            self.user.path.context().clone(),
            false,
            false,
            Credentials::ROOT,
        )?;

        let event = self.user.wait(context)?;
        if event.status != 0 {
            self.user.graph().request_root_exit(9);
        }
        self.user.reap_children(context, event.status != 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 加载指定模式的 `/test/bin/user-write` 程序，并在其地址空间中映射及清零 `SHARED_CONTROL` 共享匿名控制页。
    fn shared_image(
        &mut self,
        context: &TestContext<'_>,
        mode: &[u8],
    ) -> Result<LoadedImage, TestError> {
        let image = self.user.load_named(context, PROGRAM_WRITE, mode)?;
        vm(
            context,
            image.mm.mmap(MmapRequest {
                address: MmapAddress::FixedNoReplace(SHARED_CONTROL),
                length: 4096,
                permissions: RW,
                maximum: RW,
                source: MmapSource::SharedAnonymous,
            }),
        )?;

        if image.mm.copyout(SHARED_CONTROL, &[0; 32]).copied != 32 {
            return Err(TestError("shared control initialization failed"));
        }
        self.shared_mm = Some(image.mm.clone());
        Ok(image)
    }

    /// 以 root 权限在异核上启动已配置好共享控制页的用户程序。
    fn start_shared(&mut self, context: &TestContext<'_>, image: LoadedImage) -> TestResult {
        self.user.start_program(
            context,
            image,
            self.user.path.context().clone(),
            true,
            false,
            Credentials::ROOT,
        )
    }

    /// 以指定的用户凭证（UID/GID 等）在异核上启动已配置好共享控制页的用户程序。
    fn start_shared_as(
        &mut self,
        context: &TestContext<'_>,
        image: LoadedImage,
        credentials: Credentials,
    ) -> TestResult {
        self.user.start_program(
            context,
            image,
            self.user.path.context().clone(),
            true,
            false,
            credentials,
        )
    }

    /// 等待共享控制模式的用户进程退出内核栈并回收全部子进程，验证退出状态码为 `0`。
    fn finish_shared(&mut self, context: &mut TestContext<'_>) -> TestResult {
        until("shared user did not leave original stack", || {
            self.user
                .running
                .as_ref()
                .unwrap()
                .join
                .try_join()
                .is_some()
        })?;

        let event = self.user.wait(context)?;
        if event.status != 0 {
            self.user.graph().request_root_exit(9);
        }
        self.user.reap_children(context, event.status != 0)?;
        context.expect_eq(event.status, 0);

        self.shared_mm.take();
        Ok(())
    }

    /// 等待用户进程在 `SHARED_CONTROL + offset` 处写入预期的阶段号，同时检测进程是否异常提前退出。
    fn phase(&self, mm: &Mm, offset: usize, expected: u32) -> TestResult {
        until("shared user checkpoint missing", || {
            mm.load_user_u32(SHARED_CONTROL + offset) == Ok(expected)
                || self
                    .user
                    .running
                    .as_ref()
                    .unwrap()
                    .join
                    .try_join()
                    .is_some()
        })?;
        if mm.load_user_u32(SHARED_CONTROL + offset) != Ok(expected) {
            return Err(TestError("shared user exited before checkpoint"));
        }
        Ok(())
    }

    /// 向 `SHARED_CONTROL + 4` 门控字写入指定值，放行正在等待门控的用户态线程。
    fn release(mm: &Mm, value: u32) -> TestResult {
        mm.store_user_u32(SHARED_CONTROL + 4, value)
            .map_err(|_| TestError("shared user gate publication failed"))
    }

    /// 等待用户态 `fork` 将子进程 TID 写入 `SHARED_CONTROL + 16`，并在任务图中查找对应的子线程句柄。
    fn child(&self, mm: &Mm) -> Result<Arc<task::UserThread>, TestError> {
        let mut tid = 0;
        until("shared fork child identity missing", || {
            tid = mm.read_usize(SHARED_CONTROL + 16).unwrap_or(0);
            tid != 0
        })?;
        self.user
            .graph()
            .lookup_thread(tid)
            .ok_or(TestError("shared fork child binding missing"))
    }

    /// 在指定槽位启动一个后台内核线程以执行可能阻塞的同步闭包 `operation`，并返回其状态观察器。
    pub(super) fn consume_sync(
        &mut self,
        index: usize,
        operation: impl FnOnce() -> Result<(), FsError> + Send + 'static,
    ) -> Result<Arc<SyncObservation>, TestError> {
        assert!(self.consumers[index].is_none());
        let observation = Arc::try_new(SyncObservation {
            executor: SpinLock::new(None),
            result: SpinLock::new(None),
        })
        .map_err(|_| TestError("sync observation OOM"))?;
        let record = observation.clone();
        let prepared = task::prepare_kernel_on(cpu::current().id, move || {
            *record.executor.lock() = Some(Arc::downgrade(&task::current_task()));
            *record.result.lock() = Some(operation());
            0
        })
        .map_err(|_| TestError("sync consumer prepare failed"))?;

        self.consumers[index] = Some(prepared.publish());
        Ok(observation)
    }

    /// 在底层块设备上新建独立的 `Ext4Volume`、`InodeCache` 与 `Vfs` 实例（模拟冷启动挂载），并打开 `/test/write-reserve` 文件。
    fn cold_open(&mut self, context: &TestContext<'_>) -> Result<(FileRef, Vfs), TestError> {
        assert!(self.cold_volume.is_none() && self.cold_cache.is_none());
        self.inodes
            .try_reserve_exact(16)
            .map_err(|_| TestError("cold inode slots OOM"))?;
        let volume = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(self.user.path.owner.disk()?.clone()),
                AccessMode::ReadWrite,
                64,
            ),
        )?;
        self.cold_volume = Some(volume.clone());
        let cache = Arc::try_new(checked(context, InodeCache::try_new(64))?)
            .map_err(|_| TestError("cold cache owner OOM"))?;
        self.cold_cache = Some(cache.clone());

        let vfs = checked(
            context,
            Vfs::try_new(
                crate::fs::Filesystem::Ext4(volume),
                cache,
                128,
                4,
                AccessMode::ReadWrite,
            ),
        )?;
        let fs_context = checked(context, FsContext::new(vfs.root(), vfs.root()))?;
        let path = checked(
            context,
            vfs.resolve(&fs_context, b"/test/write-reserve", ResolveMode::Follow),
        )?;
        self.inodes.push(checked(context, path.inode())?.clone());
        let file = checked(
            context,
            open_path_with_flags(&vfs, path, 2, FileIoContext::Kernel),
        )?;
        Ok((file, vfs))
    }

    /// 释放当前所有的内存映射、文件、Inode、缓存与卷引用，等待异步缓存回收与块设备完成队列彻底排空后，重新执行冷挂载打开。
    fn cold_reopen(
        &mut self,
        context: &mut TestContext<'_>,
        mm: Arc<Mm>,
        file: FileRef,
        inode: InodeRef,
        volume: Arc<Ext4Volume>,
    ) -> Result<(FileRef, usize), TestError> {
        let disk = self.user.path.owner.disk()?.clone();
        let weak_mm = Arc::downgrade(&mm);
        let weak_file = Arc::downgrade(&file);
        let weak_volume = Arc::downgrade(&volume);
        let weak_metadata = Arc::downgrade(volume.metadata_cache());
        let weak_cache = Arc::downgrade(self.cold_cache.as_ref().unwrap());
        assert!(self.eviction.is_none());
        self.eviction = Some(self.cold_cache.as_ref().unwrap().watch());

        // Step 1: 释放全部强引用并等待 InodeCache 异步驱逐任务与块设备在途完成回调彻底排空。
        drop((mm, file, inode, volume));
        self.inodes.clear();
        drop(self.cold_cache.take());
        checked(context, self.eviction.as_ref().unwrap().wait_retired())?;
        drop(self.eviction.take());
        drop(self.cold_volume.take());
        disk.test_flush_completions();

        // Step 2: 断言块设备无在途 I/O 且旧的文件系统各层对象强引用计数全部归零。
        context.expect_eq(disk.test_snapshot().inflight, 0);
        context.expect_eq(weak_mm.upgrade().is_none(), true);
        context.expect_eq(weak_file.upgrade().is_none(), true);
        context.expect_eq(weak_cache.upgrade().is_none(), true);
        context.expect_eq(weak_metadata.upgrade().is_none(), true);
        context.expect_eq(weak_volume.upgrade().is_none(), true);
        if weak_mm.strong_count() != 0
            || weak_file.strong_count() != 0
            || weak_cache.strong_count() != 0
            || weak_volume.strong_count() != 0
            || weak_metadata.strong_count() != 0
        {
            return Err(TestError("cold reload still has old FS owners"));
        }

        // Step 3: 记录当前块设备累计提交次数，并在同一块设备上重新构建冷挂载视图。
        let reads_before = disk.test_snapshot().submitted;
        Ok((self.cold_open(context)?.0, reads_before))
    }

    /// 等待所有后台同步消费线程退出内核栈并检查其返回值。
    pub(super) fn join_consumers(&mut self) -> TestResult {
        for join in self.consumers.iter().flatten() {
            until("shared kernel consumer did not leave stack", || {
                join.try_join().is_some()
            })?;
        }

        let mut first = None;
        for join in &mut self.consumers {
            if let Some(join) = join.take() {
                if join.join() != 0 && first.is_none() {
                    first = Some(TestError("shared kernel consumer failed"));
                }
            }
        }
        first.map_or(Ok(()), Err)
    }
}

impl Fixture<UserSuite> for WriteOwner {
    /// 创建可写文件测试夹具实例，复用底层 `UserOwner` 的块设备与运行环境。
    fn set_up(context: &mut TestContext<'_>, suite: &UserSuite) -> Result<Self, TestError> {
        Ok(Self {
            user: <UserOwner as Fixture<UserSuite>>::set_up(context, suite)?,
            inodes: Vec::new(),
            append_gate: None,
            shared_mm: None,
            prepare_gate: None,
            truncate_controls: None,
            consumers: core::array::from_fn(|_| None),
            cold_volume: None,
            cold_cache: None,
            eviction: None,
        })
    }

    /// 静默并清理可写文件测试环境：卸载钩子、唤醒阻塞线程、回收用户进程，并将所有文件脏页与卷元数据同步刷回磁盘。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 卸载地址空间与 Inode 上挂载的所有测试钩子，并完成所有同步门控以唤醒阻塞中的线程。
        if let Some(mm) = self.shared_mm.take() {
            mm.ktest_set_mkwrite_hook(None);
            mm.ktest_set_fault_hook(None);
            mm.ktest_set_fork_hook(None);
            mm.ktest_set_vma_unmap_hook(None);
        }
        for inode in &self.inodes {
            inode.ktest_set_truncate_hook(None);
            inode.mapping().ktest_set_unmap_hook(None);
        }
        if let Some(controls) = self.truncate_controls.take() {
            controls.cancelled.store(true, Ordering::Release);
            for gate in &controls.gates {
                gate.complete();
            }
        }
        if let Some(gate) = self.prepare_gate.take() {
            gate.complete();
        }
        if let Some(gate) = self.append_gate.take() {
            gate.complete();
            gate.complete();
        }

        // Step 2: 恢复块设备完成回调，通知残留用户进程退出，并等待所有内核消费线程与用户线程完全离开内核栈。
        self.user
            .path
            .owner
            .observer()
            .cancelled
            .store(true, Ordering::Release);
        self.user.path.owner.observer().gate.complete();
        for disk in &self.user.path.owner.disks {
            disk.test_resume_completions();
        }
        if let Some(graph) = &self.user.graph {
            graph.request_root_exit(9);
        }

        let mut first = self.join_consumers().err();
        if self.consumers.iter().any(Option::is_some) {
            return Err(first.unwrap_or(TestError("consumer still owns original stack")));
        }
        if let Err(error) = self.user.path.owner.join_tasks() {
            if first.is_none() {
                first = Some(error);
            }
        }
        if self.user.running.is_some() {
            until("shared user cleanup did not leave original stack", || {
                self.user
                    .running
                    .as_ref()
                    .unwrap()
                    .join
                    .try_join()
                    .is_some()
            })?;
            self.user.wait(context)?;
        }
        if self.user.graph.is_some() {
            self.user.reap_children(context, true)?;
        }
        self.user.release_pressure();

        // Step 3: 将测试期间触碰过的所有 Inode 脏页同步回写到磁盘，并验证无残留 Dirty/Writeback 状态。
        for inode in &self.inodes {
            let result = crate::fs::writeback::sync_inode(inode);
            if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
                return Err(TestError(
                    "writeback stop unconfirmed; retaining original owners",
                ));
            }
            if let Err(error) = checked(context, result) {
                if first.is_none() {
                    first = Some(error);
                }
            }
        }
        if self
            .inodes
            .iter()
            .any(|inode| inode.mapping().has_dirty_or_writeback())
        {
            return Err(first.unwrap_or(TestError("write cleanup retains actual Dirty or WB")));
        }

        // Step 4: 若存在冷挂载缓存，清空 Inode 引用并等待缓存异步回收彻底完成。
        if self.eviction.is_none() {
            self.eviction = self.cold_cache.as_ref().map(|cache| cache.watch());
        }
        if let Some(eviction) = self.eviction.as_ref() {
            self.inodes.clear();
            drop(self.cold_cache.take());
            let result = eviction.wait_retired();
            if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
                return Err(TestError(
                    "inode eviction stop unconfirmed; retaining original owners",
                ));
            }
            if let Err(error) = checked(context, result) {
                return Err(first.unwrap_or(error));
            }
            drop(self.eviction.take());
        }

        // Step 5: 同步 Ext4 卷的元数据缓存并验证所有元数据块均已落盘 clean，最后交由底层 UserOwner 完成清理。
        let volume = self
            .cold_volume
            .clone()
            .or_else(|| self.user.graph.as_ref().map(|_| self.volume()));
        if let Some(volume) = volume {
            let result = crate::fs::writeback::sync_metadata(&volume);
            if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
                return Err(TestError(
                    "metadata stop unconfirmed; retaining original owners",
                ));
            }
            if let Err(error) = checked(context, result) {
                if first.is_none() {
                    first = Some(error);
                }
            }
        }
        if let Some(volume) = self.cold_volume.as_ref() {
            if metadata_pending(volume) {
                return Err(first.unwrap_or(TestError("cold cleanup retains metadata Dirty or WB")));
            }
        } else if self.user.graph.is_some() && metadata_pending(&self.volume()) {
            return Err(first.unwrap_or(TestError("write cleanup retains metadata Dirty or WB")));
        }

        self.inodes.clear();
        drop(core::mem::take(&mut self.inodes));
        drop(self.cold_cache.take());
        drop(self.cold_volume.take());
        let cleanup = <UserOwner as Fixture<UserSuite>>::quiesce(&mut self.user, context);
        context.expect_eq(first, None);
        cleanup
    }

    /// 销毁夹具并委托底层 `UserOwner` 校验全局内存、任务及块设备统计计数是否平衡。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <UserOwner as Fixture<UserSuite>>::tear_down(self.user, context)
    }
}

/// 从文件指定偏移读取字节并与预期切片精确比对，若出现短读或数据不一致则断言失败。
pub(super) fn bytes_at(
    context: &mut TestContext<'_>,
    file: &FileRef,
    offset: u64,
    expected: &[u8],
) -> TestResult {
    let mut bytes = [0xa7; 4096];
    assert!(expected.len() <= bytes.len());
    context.expect_eq(
        checked(context, file.read_at(offset, &mut bytes[..expected.len()]))?,
        expected.len(),
    );
    context.expect_eq(&bytes[..expected.len()], expected);
    Ok(())
}

/// 将虚拟内存操作错误转换为测试错误，并输出详细的 `VmError` 诊断日志。
fn vm<T>(context: &TestContext<'_>, result: Result<T, VmError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("write-vm error={:?}", error));
        TestError("write VM operation failed")
    })
}

/// 创建独立的地址空间并将文件指定偏移映射到 `SHARED_ALIAS`，预触发一次读缺页以建立共享页表映射。
pub(super) fn shared_alias(
    context: &TestContext<'_>,
    file: &FileRef,
    offset: u64,
) -> Result<Arc<Mm>, TestError> {
    let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
        .map_err(|_| TestError("shared alias MM owner OOM"))?;
    vm(
        context,
        mm.map_file(SHARED_ALIAS, 8192, RW, RW, file.clone(), offset, false),
    )?;
    vm(context, mm.fault(SHARED_ALIAS, UserAccess::Read))?;
    Ok(mm)
}

/// 从用户地址空间的指定虚拟地址拷贝字节并与预期数据比对。
fn mapped_bytes(context: &mut TestContext<'_>, mm: &Mm, address: usize, expected: &[u8]) {
    let mut bytes = [0xa7; 4096];
    context.expect_eq(
        mm.copyin(&mut bytes[..expected.len()], address).copied,
        expected.len(),
    );
    context.expect_eq(&bytes[..expected.len()], expected);
}

/// 遍历 Ext4 卷的所有块组描述符与已初始化的块/Inode 位图，校验 CRC 校验和并断言块组汇总计数与超级块空闲计数一致。
pub(super) fn allocation_counts(
    context: &mut TestContext<'_>,
    volume: &Ext4Volume,
) -> Result<(u64, u64), TestError> {
    let g = volume.geometry();
    let cache = volume.metadata_cache();
    // Step 1: 读取超级块中的全局空闲块数与空闲 Inode 数。
    let superblock = checked(
        context,
        cache
            .get(1024 / g.block_size as u64, g.block_size, None)
            .map_err(FsError::Io),
    )?;
    let decoded = checked(
        context,
        superblock
            .with_read(|bytes| {
                let at = 1024 % g.block_size;
                let b = &bytes[at..at + 1024];
                Ok((
                    u64::from(codec::le32(b, 12)?)
                        | if g.features_incompat & 0x80 != 0 {
                            u64::from(codec::le32(b, 344)?) << 32
                        } else {
                            0
                        },
                    u64::from(codec::le32(b, 16)?),
                ))
            })
            .map_err(FsError::Io),
    )?;
    let super_free = checked(context, decoded)?;
    drop(superblock);

    // Step 2: 逐个块组读取组描述符，累加空闲计数，并对已初始化的位图校验 CRC 及逐位统计空闲位数量。
    let mut sum = (0, 0);
    for group in 0..g.groups_count {
        let byte = group * g.descriptor_size as u64;
        let descriptor = checked(
            context,
            cache
                .get(
                    g.first_data_block + 1 + byte / g.block_size as u64,
                    g.block_size,
                    None,
                )
                .map_err(FsError::Io),
        )?;
        let mut record = [0u8; 64];
        checked(
            context,
            descriptor
                .with_read(|bytes| {
                    let at = (byte % g.block_size as u64) as usize;
                    record[..g.descriptor_size].copy_from_slice(&bytes[at..at + g.descriptor_size]);
                })
                .map_err(FsError::Io),
        )?;
        drop(descriptor);

        let layout = checked(
            context,
            codec::group_layout(g, group, &record[..g.descriptor_size]),
        )?;
        let state = checked(context, codec::group_state(g, &record[..g.descriptor_size]))?;
        sum.0 += u64::from(state.free_blocks);
        sum.1 += u64::from(state.free_inodes);

        for (inode, physical, uninitialized, expected, total) in [
            (
                false,
                layout.block_bitmap,
                state.flags & 2 != 0,
                u64::from(state.free_blocks),
                core::cmp::min(
                    g.blocks_per_group,
                    g.blocks_count - (g.first_data_block + group * g.blocks_per_group),
                ),
            ),
            (
                true,
                layout.inode_bitmap,
                state.flags & 1 != 0,
                u64::from(state.free_inodes),
                core::cmp::min(
                    g.inodes_per_group,
                    g.inodes_count - group * g.inodes_per_group,
                ),
            ),
        ] {
            if uninitialized {
                continue;
            }
            let bitmap = checked(
                context,
                cache.get(physical, g.block_size, None).map_err(FsError::Io),
            )?;
            let decoded = checked(
                context,
                bitmap
                    .with_read(|bytes| {
                        codec::verify_bitmap(g, &record[..g.descriptor_size], bytes, inode)?;
                        Ok((0..total)
                            .filter(|bit| bytes[*bit as usize / 8] & (1 << (*bit % 8)) == 0)
                            .count() as u64)
                    })
                    .map_err(FsError::Io),
            )?;
            let free = checked(context, decoded)?;
            context.expect_eq(free, expected);
        }
    }
    // Step 3: 验证所有块组的空闲计数总和与超级块记录完全相等。
    context.expect_eq(sum, super_free);
    context.diagnostic(format_args!(
        "write-allocation free-blocks={} free-inodes={} bitmap-count-crc=1",
        sum.0, sum.1
    ));
    Ok(sum)
}

/// 查询指定编号的 Inode 在所属块组的 Inode 位图中是否已置位（已分配），同时校验位图 CRC。
pub(super) fn inode_allocation_bit(volume: &Ext4Volume, number: u64) -> Result<bool, FsError> {
    let g = volume.geometry();
    let group = (number - 1) / g.inodes_per_group;
    let bit = (number - 1) % g.inodes_per_group;
    let byte = group * g.descriptor_size as u64;
    let descriptor = volume.metadata_cache().get(
        g.first_data_block + 1 + byte / g.block_size as u64,
        g.block_size,
        None,
    )?;
    let mut record = [0u8; 64];
    descriptor.with_read(|bytes| {
        let offset = (byte % g.block_size as u64) as usize;
        record[..g.descriptor_size].copy_from_slice(&bytes[offset..offset + g.descriptor_size]);
    })?;
    drop(descriptor);

    let layout = codec::group_layout(g, group, &record[..g.descriptor_size])?;
    let state = codec::group_state(g, &record[..g.descriptor_size])?;
    if state.flags & 1 != 0 {
        return Err(FsError::Corrupt);
    }

    let bitmap = volume
        .metadata_cache()
        .get(layout.inode_bitmap, g.block_size, None)?;
    bitmap.with_read(|bytes| {
        codec::verify_bitmap(g, &record[..g.descriptor_size], bytes, true)?;
        Ok(bytes[bit as usize / 8] & (1 << (bit % 8)) != 0)
    })?
}

/// 文件块映射树（Extent 树或传统多级间接块树）的遍历统计结果。
#[derive(Default, Debug)]
struct Tree {
    /// 文件映射的数据块总数（逻辑叶节点指向的物理块数）。
    data: u64,
    /// 外部元数据索引节点块总数（不含嵌入在 Inode 内的根部区域）。
    nodes: u64,
    /// Extent 叶节点中的连续区间条目（Extent）总数。
    extents: usize,
    /// Extent 树的根部深度（0 表示叶节点直接保存在 Inode 内）。
    depth: u16,
    /// 遍历期间收集到的所有数据块与索引节点块的物理块号列表（用于去重与位图校验）。
    blocks: Vec<u64>,
}

/// 查询指定物理块号在所属块组的块位图中是否处于已占用状态。
///
/// 若物理块位于超级块/块组描述符保留区或已在位图中置位，返回 `Ok(Some(true))`；
/// 若所属块组位图尚未初始化且仍有剩余空闲块，返回 `Ok(None)`。
pub(super) fn block_allocation_bit(
    volume: &Ext4Volume,
    physical: u64,
) -> Result<Option<bool>, FsError> {
    let g = volume.geometry();
    // validate_blocks 检查卷边界以及固定的超级块和块组描述符保留区，这些保留块视为已被占用。
    if g.validate_blocks(physical, 1).is_err() {
        return Ok(Some(true));
    }

    let group = (physical - g.first_data_block) / g.blocks_per_group;
    let bit = (physical - g.first_data_block) % g.blocks_per_group;
    let byte = group * g.descriptor_size as u64;
    let descriptor = volume.metadata_cache().get(
        g.first_data_block + 1 + byte / g.block_size as u64,
        g.block_size,
        None,
    )?;
    let mut record = [0u8; 64];
    descriptor.with_read(|bytes| {
        let at = (byte % g.block_size as u64) as usize;
        record[..g.descriptor_size].copy_from_slice(&bytes[at..at + g.descriptor_size]);
    })?;
    drop(descriptor);

    let layout = codec::group_layout(g, group, &record[..g.descriptor_size])?;
    let state = codec::group_state(g, &record[..g.descriptor_size])?;
    if state.flags & 2 != 0 {
        if group == 0 || g.features_ro_compat & (0x10 | codec::METADATA_CSUM) == 0 {
            return Err(FsError::Corrupt);
        }
        // 未初始化的块位图不直接提供逐位占用信息；仅当组描述符记录的 free_blocks 为 0 时可确认为全部占用。
        return Ok((state.free_blocks == 0).then_some(true));
    }

    let bitmap = volume
        .metadata_cache()
        .get(layout.block_bitmap, g.block_size, None)?;
    bitmap.with_read(|bytes| {
        codec::verify_bitmap(g, &record[..g.descriptor_size], bytes, false)?;
        Ok(Some(bytes[bit as usize / 8] & (1 << (bit % 8)) != 0))
    })?
}

/// 验证物理块号合法且在块位图中已标记为分配，并将其追加到遍历树的块列表中。
fn account_block(volume: &Ext4Volume, physical: u64, tree: &mut Tree) -> Result<(), FsError> {
    volume.geometry().validate_blocks(physical, 1)?;
    if block_allocation_bit(volume, physical)? != Some(true)
        || tree.blocks.capacity() == tree.blocks.len()
    {
        return Err(FsError::Corrupt);
    }
    tree.blocks.push(physical);
    Ok(())
}

/// 递归遍历 Ext4 Extent 树节点，校验各层外部索引块的 CRC 校验和，并统计数据块、索引块与 Extent 条目数。
fn extent_tree(
    volume: &Ext4Volume,
    inode: &InodeData,
    node: &[u8],
    header: codec::ExtentHeader,
    tree: &mut Tree,
) -> Result<(), FsError> {
    for index in 0..header.entries as usize {
        let at = 12 + index * 12;
        if header.depth == 0 {
            let count = codec::extent_length(codec::le16(node, at + 4)?)?.0;
            let physical = u64::from(codec::le32(node, at + 8)?)
                | (u64::from(codec::le16(node, at + 6)?) << 32);
            for block in physical..physical + count {
                account_block(volume, block, tree)?;
            }
            tree.data += count;
            tree.extents += 1;
        } else {
            let physical = u64::from(codec::le32(node, at + 4)?)
                | (u64::from(codec::le16(node, at + 8)?) << 32);
            account_block(volume, physical, tree)?;
            let buffer = volume.metadata_block(physical)?;
            let mut bytes = [0u8; 4096];
            buffer
                .with_read(|data| bytes[..data.len()].copy_from_slice(data))
                .map_err(FsError::Io)?;
            drop(buffer);

            let child = codec::extent_header(
                &bytes[..volume.geometry().block_size],
                Some(header.depth - 1),
            )?;
            codec::verify_extent_checksum(
                volume.geometry(),
                inode,
                &bytes[..volume.geometry().block_size],
                child,
            )?;
            tree.nodes += 1;
            extent_tree(
                volume,
                inode,
                &bytes[..volume.geometry().block_size],
                child,
                tree,
            )?;
        }
    }
    Ok(())
}

/// 递归遍历传统的直接/间接块指针树（0 表示空洞跳过），统计所有数据块与各级间接索引块。
fn indirect_tree(
    volume: &Ext4Volume,
    physical: u64,
    depth: u16,
    tree: &mut Tree,
) -> Result<(), FsError> {
    if physical == 0 {
        return Ok(());
    }

    account_block(volume, physical, tree)?;
    if depth == 0 {
        tree.data += 1;
        return Ok(());
    }

    let buffer = volume.metadata_block(physical)?;
    let mut bytes = [0u8; 4096];
    buffer
        .with_read(|data| bytes[..data.len()].copy_from_slice(data))
        .map_err(FsError::Io)?;
    drop(buffer);
    tree.nodes += 1;
    for at in (0..volume.geometry().block_size).step_by(4) {
        indirect_tree(volume, u64::from(codec::le32(&bytes, at)?), depth - 1, tree)?;
    }
    Ok(())
}

/// 加载磁盘上的 Inode 记录，遍历其块映射树（Extent 或间接块），并验证无重复物理块且 `i_blocks` 扇区计数准确。
fn tree(context: &mut TestContext<'_>, inode: &InodeRef) -> Result<Tree, TestError> {
    let volume = checked(context, inode.disk_volume())?;
    let data = checked(context, volume.load_inode(inode.number(), None))?;
    let mut tree = Tree::default();
    tree.blocks
        .try_reserve_exact(
            (data.blocks * SECTOR_BYTES / volume.geometry().block_size as u64) as usize,
        )
        .map_err(|_| TestError("write tree observations OOM"))?;

    // Step 1: 根据 Inode 的 EXTENTS 标志选择遍历 Extent 树或传统的 15 槽位直接/间接指针树。
    if data.flags & codec::EXTENTS != 0 {
        let header = checked(context, codec::extent_header(&data.block, None))?;
        tree.depth = header.depth;
        checked(
            context,
            extent_tree(volume, &data, &data.block, header, &mut tree),
        )?;
    } else {
        for index in 0..15 {
            checked(
                context,
                indirect_tree(
                    volume,
                    u64::from(checked(context, codec::le32(&data.block, index * 4))?),
                    if index < 12 { 0 } else { (index - 11) as u16 },
                    &mut tree,
                ),
            )?;
        }
    }

    // Step 2: 对收集到的全部物理块排序查重，并验证磁盘与内存中的 i_blocks 512 字节扇区计数完全匹配。
    tree.blocks.sort_unstable();
    context.expect_eq(tree.blocks.windows(2).all(|pair| pair[0] != pair[1]), true);
    context.expect_eq(
        data.blocks,
        (tree.data + tree.nodes) * volume.geometry().block_size as u64 / 512,
    );
    context.expect_eq(
        checked(context, inode.metadata_snapshot())?.blocks,
        data.blocks,
    );
    context.diagnostic(format_args!(
        "write-tree inode={} data={} nodes={} extents={} depth={} i-blocks={}",
        inode.number(),
        tree.data,
        tree.nodes,
        tree.extents,
        tree.depth,
        data.blocks
    ));
    Ok(tree)
}

/// 物理页内存压力测试辅助器，用于在预留好管理槽位的前提下耗尽全局伙伴系统（Buddy Allocator）的空闲物理页。
struct PagePressure {
    /// 已从伙伴系统分配并持有的物理页凭证列表，在销毁时自动归还给页分配器。
    pages: Vec<mm::OwnedPages>,
}

impl PagePressure {
    /// 根据当前系统存在的物理页总数预分配 `Vec` 容量，确保后续施压分配过程中不会触发堆内存扩容。
    fn prepare() -> Result<Self, TestError> {
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(mm::stats().present_pages)
            .map_err(|_| TestError("write page pressure slots OOM"))?;
        Ok(Self { pages })
    }

    /// 收缩缓存并循环分配高阶与单页物理内存直至分配器返回 `None`，确认物理页已被完全耗尽。
    fn exhaust(&mut self) -> bool {
        mm::shrink_all();
        mm::drain_all_pages();
        for order in [10, 0] {
            while let Some(page) = mm::alloc_pages(order, false) {
                assert!(self.pages.len() < self.pages.capacity());
                self.pages.push(page);
            }
        }

        mm::drain_all_pages();
        mm::alloc_pages(0, false).is_none()
    }
}

/// 检查指定 Ext4 卷的元数据缓存中是否仍存在处于 `Dirty` 或 `Writeback` 状态的缓存页。
fn metadata_pending(volume: &Ext4Volume) -> bool {
    let cache = volume.metadata_cache();
    for mark in [
        crate::cache::FolioMark::Dirty,
        crate::cache::FolioMark::Writeback,
    ] {
        let mut cursor = 0;
        for _ in 0..cache.len() {
            let mut selected = [None];
            let next = cache.select_range(cursor, u64::MAX, mark, &mut selected);
            if selected[0].is_some() {
                return true;
            }
            let Some(next) = next else {
                break;
            };
            cursor = next;
        }
    }
    false
}

/// 为已解除目录链接（Unlinked）但仍持有物理页的 Inode 准备底层数据回写范围与 `FolioWriteback` 凭证。
fn prepare_unlinked_original_writeback(
    inode: &InodeRef,
    folio: &crate::mm::FolioRef,
) -> Result<Vec<(u64, crate::mm::FolioWriteback)>, FsError> {
    let volume = inode.disk_volume()?;
    let block_size = volume.geometry().block_size;
    let folio = folio.clone().with_file_owner(inode.clone());
    folio.wait_writeback();
    let (_, targets) =
        crate::fs::writeback::prepare_file_targets(inode, 0, &(0..PAGE_SIZE as u64), None)?;
    let mut ranges = Vec::new();
    ranges
        .try_reserve_exact(PAGE_SIZE / block_size)
        .map_err(|_| FsError::NoMemory)?;

    let mut content = folio.lock_content();
    inode.mapping().mkclean(0, &folio, &mut content);
    let dirty = folio.dirty_sectors();
    let block_mask = ((1u16 << (block_size / SECTOR_BYTES as usize)) - 1) as u8;
    for offset in (0..PAGE_SIZE).step_by(block_size) {
        if dirty & (block_mask << (offset / SECTOR_BYTES as usize)) == 0 {
            continue;
        }
        let target = targets[offset / block_size].ok_or(FsError::Corrupt)?;
        // SAFETY:
        // 1. 当前持有 `folio.lock_content()` 内容锁，排斥并发的写许可提交。
        // 2. 前置 `inode.mapping().mkclean(0, &folio, &mut content)` 已撤销所有共享可写页表映射并完成跨核 TLB 同步。
        // 3. 返回的 `FolioWriteback` 凭证将移交底层块设备 I/O 请求持有，直至 Bio 完成回调执行完毕后释放页面的回写状态。
        if let Some(writeback) = unsafe { content.begin_writeback(offset, block_size)? } {
            ranges.push((
                volume.view().absolute_byte(target, block_size)?,
                writeback.with_writeback_domain(volume.writeback_domain(), false),
            ));
        }
    }

    drop(content);
    drop(folio);
    if ranges.is_empty() {
        return Err(FsError::Corrupt);
    }
    Ok(ranges)
}

/// 读取并拷贝磁盘上指定编号 Inode 的原始字节记录（支持 128 字节与 256 字节两种 Ext4 Inode 尺寸）。
fn inode_record_snapshot(
    context: &mut TestContext<'_>,
    volume: &Ext4Volume,
    number: u64,
) -> Result<([u8; 256], usize), TestError> {
    let size = volume.geometry().inode_size;
    if size != 128 && size != 256 {
        return Err(TestError("inode reuse record size is unsupported"));
    }

    let (buffer, offset) = checked(context, volume.inode_record(number, None))?;
    let mut record = [0; 256];
    checked(
        context,
        buffer
            .with_read(|bytes| record[..size].copy_from_slice(&bytes[offset..offset + size]))
            .map_err(FsError::Io),
    )?;
    Ok((record, size))
}

/// 汇总六类可写文件测试子模块的注册锚点，供上层测试套件统一引用以防止链接器丢弃。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    core::hint::black_box([
        buffered::cases::__ktest_anchor(),
        mmap::cases::__ktest_anchor(),
        writeback::cases::__ktest_anchor(),
        truncate::cases::__ktest_anchor(),
        reclaim::cases::__ktest_anchor(),
        namespace::cases::__ktest_anchor(),
    ])[0]
}
