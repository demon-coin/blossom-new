//! 可写文件测试的共同拥有、真实同步与六类行为 suite 锚点。

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

const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
const RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};
const SOURCE: usize = 0x4800_0000;
const PROGRAM_WRITE: &[u8] = b"/test/bin/user-write";
const SHARED_CONTROL: usize = 0x4900_0000;
const SHARED_FILE_OFFSET: usize = 24;
pub(super) const SHARED_ALIAS: usize = 0x4a00_0000;

/// 普通测试消费者的真实 Task 状态与同步结果观察，不承担 join 或文件拥有。
pub(super) struct SyncObservation {
    executor: SpinLock<Option<alloc::sync::Weak<Task>>>,
    result: SpinLock<Option<Result<(), FsError>>>,
}

impl SyncObservation {
    /// 只查询实际执行者的调度状态，entered 标记不能替代 Blocked。
    pub(super) fn blocked(&self) -> bool {
        self.executor
            .lock()
            .as_ref()
            .and_then(alloc::sync::Weak::upgrade)
            .is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
    /// 返回真实同步调用已经写下的结果，不用观察任务退出代替底层结果。
    pub(super) fn result(&self) -> Option<Result<(), FsError>> {
        *self.result.lock()
    }
}

struct TruncateControls {
    phase: AtomicUsize,
    cancelled: AtomicBool,
    gates: [Completion; 3],
    unmap_phase: AtomicUsize,
    before_sync: SpinLock<Option<(usize, usize)>>,
    after_sync: SpinLock<Option<(usize, usize)>>,
}

impl TruncateControls {
    /// 固定门仅暂停真实截断或 CSD 前后的拥有交接，错误清理放行同一批原参与者。
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

/// 复用原用户、任务、卷及异步消费者拥有边界的可写文件夹具。
pub(super) struct WriteOwner {
    pub(super) user: UserOwner,
    pub(super) inodes: Vec<InodeRef>,
    append_gate: Option<Arc<Completion>>,
    shared_mm: Option<Arc<Mm>>,
    prepare_gate: Option<Arc<Completion>>,
    truncate_controls: Option<Arc<TruncateControls>>,
    consumers: [Option<JoinHandle>; 2],
    pub(super) cold_volume: Option<Arc<Ext4Volume>>,
    cold_cache: Option<Arc<InodeCache>>,
    eviction: Option<InodeCacheWatch>,
}

impl WriteOwner {
    /// 正文显式准备 RW 卷及同一根挂载，普通用户运行仍借原身份拥有者。
    fn prepare(&mut self, context: &TestContext<'_>) -> TestResult {
        self.inodes
            .try_reserve_exact(16)
            .map_err(|_| TestError("write inode owner OOM"))?;
        self.user.prepare_mode(context, AccessMode::ReadWrite, 4)
    }
    /// 原生 namespace 消费者沿同一 RW PathOwner 准备，并让原清理无论 graph 状态都同步此卷。
    pub(super) fn prepare_namespace(&mut self, context: &TestContext<'_>) -> TestResult {
        self.prepare(context)?;
        assert!(self.cold_volume.is_none() && self.cold_cache.is_none());
        self.cold_volume = Some(self.volume());
        Ok(())
    }
    /// 准备可由测试组写入的 setgid 目录与只允许 root 搜索的负例子目录。
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
        // mkdir 不接受 SGID 模式位；父目录的继承属性通过受保护元数据更新设置。
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
    /// 删除普通用户无权移除的预置权限目录与最后一层测试目录。
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

    /// 保留实际 inode 到同步排空之后；失败路径也不能丢弃已接受修改。
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
    /// 保留末组件本身的 canonical inode，symlink 不跟随到其目标。
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

    /// 从同一原根取得卷，根引用仅在正文和同步期间短暂保活。
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

    /// 用户普通 syscall 自己断言返回值，内核仍消费 wait 状态与真实离栈 join。
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

    /// 普通 ELF 自己 mmap 文件；共享匿名控制页只承担检查点，不改变生产访问路径。
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

    /// 用户执行固定在另一 CPU，控制者不持文件锁等待其普通指令检查点。
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
    /// 按指定的真实用户身份启动同一普通 ELF，并保留原 UserOwner 的 wait/join。
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

    /// 实际退出离栈必须在原十秒内可观察，随后消费原 wait 事件和全部子进程。
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

    /// 检查点同时核原用户绑定仍活，避免把用户提前死亡当作阶段完成。
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

    /// 原 copyout 发布普通控制字，不能借文件原页或 PTE 裸写绕过 mkwrite。
    fn release(mm: &Mm, value: u32) -> TestResult {
        mm.store_user_u32(SHARED_CONTROL + 4, value)
            .map_err(|_| TestError("shared user gate publication failed"))
    }

    /// 由普通 fork 已发表的真实 TID 取 child 原绑定，诊断引用不承担执行寿命。
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

    /// 原 Task 运行真实同步调用，结果另存以消费预期错误，句柄保在 fixture 的清理边界外。
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

    /// 冷消费者取得同一真实卷的文件与 Vfs；只需文件的调用者立即释放视图，旧卷完全退休后才可重开。
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

    /// 正式同步后的全部旧 FS 拥有先退出，真实完成 Work 尾和弱引用到期后才重建同设备的卷及缓存。
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
        drop((mm, file, inode, volume));
        self.inodes.clear();
        drop(self.cold_cache.take());
        checked(context, self.eviction.as_ref().unwrap().wait_retired())?;
        drop(self.eviction.take());
        drop(self.cold_volume.take());
        disk.test_flush_completions();
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
        let reads_before = disk.test_snapshot().submitted;
        Ok((self.cold_open(context)?.0, reads_before))
    }
    /// 未离栈句柄留在 fixture，错误返回仍先恢复设备完成再进行有限 join。
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
    /// 设备、网络域与压力存储均复用整组拥有，正文失败仍保原停止责任。
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

    /// 所有生产者先退出和离栈，再真实写出文件页、元数据及 FLUSH，最后释放根并停止设备。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
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

    /// 两层六域和 Task/worker/domain 均由原拥有者比较，不重置统计。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <UserOwner as Fixture<UserSuite>>::tear_down(self.user, context)
    }
}

/// 每个期望字节来自独立输入，短读取或读到旧缓存均失败。
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

/// VM 错误保原原因，用户输入不经测试专用页或旁路复制。
fn vm<T>(context: &TestContext<'_>, result: Result<T, VmError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("write-vm error={:?}", error));
        TestError("write VM operation failed")
    })
}

/// 每份别名有独立 Mm 和 VA，文件页偏移由本例实际准备的范围决定。
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

/// 完整复制的字节与独立输入比较，PFN 诊断不能替代实际内容。
fn mapped_bytes(context: &mut TestContext<'_>, mm: &Mm, address: usize, expected: &[u8]) {
    let mut bytes = [0xa7; 4096];
    context.expect_eq(
        mm.copyin(&mut bytes[..expected.len()], address).copied,
        expected.len(),
    );
    context.expect_eq(&bytes[..expected.len()], expected);
}

/// 位图只在 writer 已停稳后读取；未初始化组仍使用正式描述符合法计数。
pub(super) fn allocation_counts(
    context: &mut TestContext<'_>,
    volume: &Ext4Volume,
) -> Result<(u64, u64), TestError> {
    let g = volume.geometry();
    let cache = volume.metadata_cache();
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
    context.expect_eq(sum, super_free);
    context.diagnostic(format_args!(
        "write-allocation free-blocks={} free-inodes={} bitmap-count-crc=1",
        sum.0, sum.1
    ));
    Ok(sum)
}

/// 正常分配和末使用释放都从当前共同位图取真实 inode 位，并验证同一记录的 checksum。
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

#[derive(Default, Debug)]
struct Tree {
    data: u64,
    nodes: u64,
    extents: usize,
    depth: u16,
    blocks: Vec<u64>,
}

/// 停稳其他分配与释放者后核物理位；几何排除或真实 used 返回 Some(true)，未初始化且仍有空闲返回 None。
pub(super) fn block_allocation_bit(
    volume: &Ext4Volume,
    physical: u64,
) -> Result<Option<bool>, FsError> {
    let g = volume.geometry();
    // validate_blocks 只核卷边界和固定 super/GDT 保留范围，这些地址不能成为新 data。
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
        // 未初始化 bitmap 不含权威位；正式 free_blocks=0 时 allocator 会跳过整组。
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

/// 每个连接到树的物理块必须有自己的真实位图位，不能只靠总计数相等。
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

/// 实际读取每个 extent 节点并消费正式 CRC，不以逻辑块扫描代替 grow/split 的树证据。
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

/// 传统指针树逐个访问实际节点，空指针不计块，所有层的节点都计入 i_blocks。
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

/// inode 本身的 CRC、实际根格式和全部外部节点共同决定数据/节点计数。
fn tree(context: &mut TestContext<'_>, inode: &InodeRef) -> Result<Tree, TestError> {
    let volume = checked(context, inode.disk_volume())?;
    let data = checked(context, volume.load_inode(inode.number(), None))?;
    let mut tree = Tree::default();
    tree.blocks
        .try_reserve_exact(
            (data.blocks * SECTOR_BYTES / volume.geometry().block_size as u64) as usize,
        )
        .map_err(|_| TestError("write tree observations OOM"))?;
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

/// 真实 PagePressure 只占物理页，堆类别不耗尽，控制槽在施压之前完成预留。
struct PagePressure {
    pages: Vec<mm::OwnedPages>,
}
impl PagePressure {
    /// 固定容量记录原页凭证，压力期间不分配控制对象。
    fn prepare() -> Result<Self, TestError> {
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(mm::stats().present_pages)
            .map_err(|_| TestError("write page pressure slots OOM"))?;
        Ok(Self { pages })
    }
    /// 原 allocator 的真实 None 是页耗尽证据；放压由原凭证 Drop 归还。
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

/// 生产者停稳后逐个有界检查原 metadata 索引，残留真实义务时不得销毁 cache 平账。
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

/// 用真实文件块地址和原页凭证准备数据回写，并将完成字节归入所属卷。
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
        // SAFETY: 与生产准备路径一样，内容 guard 排斥许可提交；mkclean 已撤销
        // 同源共享 W 并完成全 CPU CSD，返回凭证随后由原 FsClaim/Bio 全尾收权。
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

/// 在短内容锁内复制完整的真实 inode 记录，支持本例两种 ext4 记录尺寸。
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

/// 保留六类行为的实际注册槽，父入口继续只借一个槽而不维护逐例分派。
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
