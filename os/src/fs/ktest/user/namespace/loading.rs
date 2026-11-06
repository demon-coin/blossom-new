//! 符号链接块、目录项缓存（`DentryCache`）与文件页缓存（`PageCache`）并发加载交错及错误恢复测试。
//!
//! # 模块定位
//! 本模块属于文件系统命名空间与缓存加载测试套件（`fs::ktest::user::namespace::loading`），
//! 上游由 `namespace::anchor` 挂载进 `ktest` 框架，下游直接操作 `Vfs`、`DentryCache`、`MetadataCache`、
//! `Inode::mapping`（文件页映射）以及块设备测试钩子（`DiskRef`）。
//!
//! # 核心机制与单飞（Singleflight）加载状态机
//! 目录项查找与文件页读取均采用单飞（Singleflight）合并加载模式，避免并发请求重复下发底层 I/O：
//!
//! ```text
//! [调用方 A 发起查找/读页]
//!        |
//!        v
//! +----------------------------+     并发到达     +----------------------------+
//! | 插入占位项 (InLookup /      | <-------------- | 调用方 B (同目录项/同页读者)|
//! | 未就绪 Folio) 并下发底层 I/O |                 | 在同一占位项上阻塞等待     |
//! +----------------------------+                 +----------------------------+
//!        |                                              ^
//!        | 并发写者 (link / write_at)                    |
//!        +----------------------------------------------+ (等待先行加载完成)
//!        |
//!        +---> [成功完成] ---> 填充数据并标记 Ready ---> 唤醒全部等待者 ---> 写者随后覆盖更新
//!        |
//!        +---> [加载失败 (OOM / CRC 校验错 / I/O 错)]
//!                 |
//!                 +---> 广播错误唤醒同批等待者
//!                 +---> 移除失败占位项 / 回收未就绪块窗口（保留同物理页内已就绪的邻居块窗口）
//!                 +---> 后续新请求重新发起独立加载，不残留坏状态
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试钩子（`MetadataLoadHook`、`ktest_set_lookup_hook`）在缓存条目已插入但尚未完成加载的窗口期触发。
//! - `SymlinkLoadState` 由独立的 `SpinLock` 保护；在调用可能触发磁盘 I/O 或页分配的 `cache.get` / `with_read` 前，
//!   必须先释放或避免长期持有 `state` 自旋锁，仅在更新观测快照字段时短暂持锁。

use alloc::sync::Arc;

use crate::block::{BlockError, DiskRef};
use crate::cache::ktest::HeapPressure;
use crate::cache::{BufferRef, MetadataCache, MetadataLoadHook};
use crate::fs::ext4::{codec, BlockMapping};
use crate::fs::ktest::user::write::{
    allocation_counts, block_allocation_bit, inode_allocation_bit, WriteOwner,
};
use crate::fs::ktest::user::UserSuite;
use crate::fs::ktest::*;
use crate::fs::{DentryRef, FsError, InodeKind, ResolveMode, Vfs};

use super::{directory_block, prepare_namespace_view};

/// 长符号链接创建过程中注入内存耗尽（OOM）时的元数据块缓存观测状态。
struct SymlinkLoadState {
    /// 堆与物理页帧压力控制器，在钩子触发时取出并耗尽空闲内存。
    pressure: Option<HeapPressure>,
    /// 标记是否已在目标块上触发过一次 OOM 拦截，防止重复触发。
    captured: bool,
    /// 发生 OOM 的目标数据块缓存引用（预期在回滚后变为 `NotReady`）。
    failed: Option<BufferRef>,
    /// 双块长符号链接的首个数据块缓存引用（预期随事务取消一并失效为 `NotReady`）。
    prefix: Option<BufferRef>,
    /// 与目标块位于同一 4096B 物理页内的相邻块缓存引用（预期在回滚后保持就绪且内容完好）。
    neighbor: Option<BufferRef>,
    /// 相邻块在 OOM 发生前读取到的原始字节快照，用于事后比对内容是否被破坏。
    neighbor_bytes: [u8; mm::PAGE_SIZE],
    /// `HeapPressure::exhaust` 返回的（页分配器耗尽, 堆分配器耗尽）观测结果。
    evidence: Option<(bool, bool)>,
    /// 钩子准备阶段若发生意外块层错误，记录在此字段供主线程断言。
    preparation_error: Option<BlockError>,
}

/// 元数据加载钩子回调：当长符号链接的最后一个数据块槽位刚在缓存中分配、但尚未完成读取时，
/// 预先加载同一物理页内的相邻块窗口，随后耗尽物理页与堆内存以触发后续路径的 OOM 回滚。
fn observe_symlink_oom(
    cache: &Arc<MetadataCache>,
    disk: &DiskRef,
    state: &Arc<SpinLock<SymlinkLoadState>>,
    block: u64,
    block_size: usize,
    failed_block: u64,
    prefix_block: Option<u64>,
) -> Result<(), BlockError> {
    // Step 1: 仅拦截指定的目标失败块，且只触发一次。
    if block != failed_block {
        return Ok(());
    }
    let mut pressure = {
        let mut state = state.lock();
        if state.captured {
            return Ok(());
        }
        state.captured = true;
        state.pressure.take().expect("symlink load pressure absent")
    };
    // Step 2: 获取目标块（以及双块符号链接的前缀块）的缓存视图引用，并确保前缀块已完成读取。
    let failed = cache
        .cached_buffer(block, block_size)?
        .ok_or(BlockError::NotReady)?;
    let prefix = prefix_block
        .map(|block| {
            cache
                .cached_buffer(block, block_size)?
                .ok_or(BlockError::NotReady)
        })
        .transpose()?;
    if let Some(prefix) = &prefix {
        prefix.with_read(|_| ())?;
    }
    // Step 3: 在同一 4096B 物理页内寻找一个既非目标块也非前缀块的邻居块，预先加载并备份其字节内容。
    let windows = mm::PAGE_SIZE / block_size;
    let page = block / windows as u64 * windows as u64;
    let neighbor_block = (page..page + windows as u64)
        .find(|&candidate| candidate != block && Some(candidate) != prefix_block);
    let neighbor = neighbor_block
        .map(|block| cache.get(block, block_size, None))
        .transpose()?;
    if let Some(neighbor) = &neighbor {
        neighbor.with_read(|bytes| {
            state.lock().neighbor_bytes[..block_size].copy_from_slice(bytes);
        })?;
    }

    // Step 4: 排空已完成的磁盘请求，正式耗尽物理页与内核堆内存，并将观测引用写回共享状态。
    disk.test_flush_completions();
    let evidence = pressure.exhaust();
    let mut state = state.lock();
    state.failed = Some(failed);
    state.prefix = prefix;
    state.neighbor = neighbor;
    state.evidence = Some(evidence);
    state.pressure = Some(pressure);
    Ok(())
}

/// 在 VFS 目录项缓存（`DentryCache`）上注册测试钩子：当指定名称 `name` 的目录项进入 `InLookup` 状态时阻塞加载线程。
fn install_lookup_gate(
    _context: &TestContext<'_>,
    vfs: &Vfs,
    observer: Arc<Observation>,
    name: &'static [u8],
) -> TestResult {
    let hook: Arc<dyn Fn(&DentryRef) + Send + Sync> = Arc::try_new(move |dentry: &DentryRef| {
        if dentry.name().as_slice() == name && observer.hold_loader.load(Ordering::Acquire) {
            observer.loader_entered.fetch_add(1, Ordering::AcqRel);
            observer.gate.wait();
        }
    })
    .map_err(|_| TestError("namespace lookup observer OOM"))?;
    vfs.dcache().ktest_set_lookup_hook(Some(hook));
    Ok(())
}

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-path", "fs-namespace"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 验证创建跨块长符号链接期间发生内存耗尽（OOM）时，文件系统的事务回滚与块缓存失效隔离机制。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：探测下一个待分配的符号链接 Inode 号及数据块号，注册 `MetadataLoadHook` 在最后一个数据块加载时预读同物理页邻居块并耗尽内存。
    /// - **When**：调用 `vfs.symlink_from` 创建长符号链接（1024B 块下长 1084B 占 2 块，4096B 块下长 128B 占 1 块），在元数据加载钩子中触发 OOM。
    /// - **Then**：
    ///   1. `symlink_from` 返回 `FsError::NoMemory`；
    ///   2. 失败块与前缀块的缓存窗口均被取消并回退为 `BlockError::NotReady`，而共享同一物理页帧（PFN）的邻居块窗口保持就绪且数据完好；
    ///   3. Inode 与数据块分配位图完整退回，目录中不残留 `/w23-symlink-oom`；
    ///   4. 释放内存压力后重新创建同名符号链接，能够正常复用刚才退回的 Inode 号与物理块号并正确读出链接目标。
    #[test]
    #[ktest(tags("W23", "fs-oom"))]
    fn long_symlink_load_oom_cancels_exact_aliases(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 预分配并立即退回一次 Inode 与数据块，探测下一次创建符号链接将使用的准确 Inode 号和物理块号。
        let view = prepare_namespace_view(context, fixture)?;
        let volume = view.volume.clone();
        let cache = volume.metadata_cache().clone();
        let disk = fixture.user.path.owner.disk()?.clone();
        let block_size = volume.geometry().block_size;
        let target_len: usize = if block_size == 1024 { 1084 } else { 128 };
        let blocks = target_len.div_ceil(block_size);
        let parent = checked(context, view.root.inode())?;
        let metadata = checked(context, parent.metadata_snapshot())?;
        let allocation = checked(
            context,
            volume.allocate_inode_for_parent(&metadata, InodeKind::Symlink, 0o777, 0, 0, 0),
        )?;
        let number = allocation.number();
        let hint = (number - 1) / volume.geometry().inodes_per_group;
        let first = checked(context, volume.allocate_block(hint))?;
        let first_block = first.physical();
        let second = if blocks == 2 {
            Some(checked(context, volume.allocate_block(hint))?)
        } else {
            None
        };
        let failed_block = second
            .as_ref()
            .map_or(first_block, |block| block.physical());
        if let Some(second) = second {
            volume.release_unpublished_block(second);
        }
        volume.release_unpublished_block(first);
        volume.release_unpublished_inode(allocation);
        let before = allocation_counts(context, &volume)?;

        // Step 2: 构造 OOM 观测状态与元数据缓存加载钩子，并在后台工作线程中发起 `symlink_from`。
        let pressure = fixture
            .user
            .path
            .owner
            .pressure
            .take()
            .ok_or(TestError("symlink real OOM pressure absent"))?;
        let state = Arc::try_new(SpinLock::new(SymlinkLoadState {
            pressure: Some(pressure),
            captured: false,
            failed: None,
            prefix: None,
            neighbor: None,
            neighbor_bytes: [0; mm::PAGE_SIZE],
            evidence: None,
            preparation_error: None,
        }))
        .map_err(|_| TestError("symlink load observation OOM"))?;
        let observer = Arc::downgrade(&state);
        let hook_disk = disk.clone();
        let prefix_block = (blocks == 2).then_some(first_block);
        let hook: MetadataLoadHook =
            Arc::try_new(move |cache: &Arc<MetadataCache>, block, size| {
                let Some(state) = observer.upgrade() else {
                    return;
                };
                if let Err(error) = observe_symlink_oom(
                    cache,
                    &hook_disk,
                    &state,
                    block,
                    size,
                    failed_block,
                    prefix_block,
                ) {
                    state.lock().preparation_error = Some(error);
                }
            })
            .map_err(|_| TestError("symlink load hook OOM"))?;

        let vfs = view.vfs.clone();
        let fs_context = view.context.clone();
        let root = view.root.clone();
        cache.ktest_set_load_hook(Some(hook));
        let consumer = fixture.consume_sync(0, move || {
            let target = [b'x'; mm::PAGE_SIZE];
            vfs.symlink_from(
                &fs_context,
                &root,
                b"/w23-symlink-oom",
                &target[..target_len],
                (0, 0),
            )
        });
        let joined = if consumer.is_ok() {
            fixture.join_consumers()
        } else {
            Ok(())
        };
        cache.ktest_set_load_hook(None);
        let pressure = state.lock().pressure.take();
        drop(pressure);
        joined?;
        let consumer = consumer?;
        context.expect_eq(state.lock().captured, true);
        context.expect_eq(state.lock().preparation_error, None);
        context.expect_eq(state.lock().evidence, Some((true, true)));
        context.expect_eq(consumer.result(), Some(Err(FsError::NoMemory)));

        // Step 3: 校验失败块与前缀块窗口已失效为 NotReady，同物理页邻居块窗口仍然有效，且位图计数完全恢复。
        let (failed, prefix, neighbor, neighbor_bytes) = {
            let mut state = state.lock();
            (
                state.failed.take(),
                state.prefix.take(),
                state.neighbor.take(),
                state.neighbor_bytes,
            )
        };
        let failed = failed.ok_or(TestError("symlink failed-range observer absent"))?;
        context.expect_eq(failed.with_read(|_| ()), Err(BlockError::NotReady));
        if let Some(prefix) = &prefix {
            context.expect_eq(prefix.with_read(|_| ()), Err(BlockError::NotReady));
        }
        if let Some(neighbor) = &neighbor {
            context.expect_eq(neighbor.folio().phys_addr(), failed.folio().phys_addr());
            let unchanged = checked(
                context,
                neighbor
                    .with_read(|bytes| bytes == &neighbor_bytes[..block_size])
                    .map_err(FsError::Io),
            )?;
            context.expect_eq(unchanged, true);
        }
        let after_failure = allocation_counts(context, &volume)?;
        context.expect_eq(after_failure, before);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            false,
        );
        context.expect_eq(
            checked(context, block_allocation_bit(&volume, failed_block))?,
            Some(false),
        );
        if let Some(prefix_block) = prefix_block {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, prefix_block))?,
                Some(false),
            );
        }
        context.expect_eq(
            view.vfs
                .resolve(&view.context, b"/w23-symlink-oom", ResolveMode::NoFollow)
                .err(),
            Some(FsError::NotFound),
        );

        // Step 4: 解除内存压力后重新创建同名长符号链接，验证原 Inode 号与物理块号被正常复用且内容完整。
        let target = [b'x'; mm::PAGE_SIZE];
        checked(
            context,
            view.vfs.symlink_from(
                &view.context,
                &view.root,
                b"/w23-symlink-oom",
                &target[..target_len],
                (0, 0),
            ),
        )?;
        let path = checked(
            context,
            view.vfs
                .resolve(&view.context, b"/w23-symlink-oom", ResolveMode::NoFollow),
        )?;
        let inode = checked(context, path.inode())?;
        context.expect_eq(inode.number(), number);
        context.expect_eq(
            checked(context, inode.read_link())?.as_slice(),
            &target[..target_len],
        );
        context.expect_eq(
            checked(context, inode.map_block((blocks - 1) as u64))?,
            BlockMapping::Mapped {
                physical: failed_block,
                len: 1,
            },
        );
        let restored = checked(
            context,
            failed
                .with_read(|bytes| {
                    let offset = (blocks - 1) * block_size;
                    bytes[..target_len - offset] == target[offset..target_len]
                })
                .map_err(FsError::Io),
        )?;
        context.expect_eq(restored, true);

        checked(
            context,
            view.vfs
                .unlink_from(&view.context, &view.root, b"/w23-symlink-oom", (0, 0)),
        )?;
        drop(path);
        checked(context, volume.wait_inode_reuse(number))?;
        disk.test_flush_completions();
        let after_reuse = allocation_counts(context, &volume)?;
        context.expect_eq(after_reuse, before);
        context.diagnostic(format_args!(
            "W23 long-symlink blocks={} block-size={} post-slot-real-page-and-heap-OOM=1 exact-failed-and-prefix-alias-invalidated=1 same-page-neighbor-preserved={} bits-returned=1 normal-reuse-and-readlink=1 controlled-completion-error=0",
            blocks,
            block_size,
            neighbor.is_some()
        ));
        Ok(())
    }

    /// 验证目录项缓存（`DentryCache`）正在执行单飞加载（`InLookup`）时，并发同名查找与硬链接创建的串行化顺序。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：在 `DentryCache` 上为 `"w28-publish"` 安装阻塞钩子，启动读取任务 0 发起查找使其停留在 `InLookup` 状态。
    /// - **When**：依次启动读取任务 1 查找同名目录项（在占位项上阻塞等待）以及写入任务调用 `vfs.link_from` 创建同名硬链接（在目录锁上阻塞等待），随后放行查找钩子。
    /// - **Then**：
    ///   1. 先行的目录项加载首先完成，两个并发查找任务均收到当时的查找结果 `FsError::NotFound`，且底层目录加载仅执行 1 次；
    ///   2. 写入任务随后完成硬链接发布，再次解析 `"/w28-publish"` 能够查找到新绑定的目标 Inode，不会误命中先前的未找到状态。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn directory_loading_serializes_link_publication(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 准备根目录与源文件的缓存状态，并为 `"w28-publish"` 安装 `InLookup` 阻塞门闩。
        fixture.prepare_namespace(context)?;
        let vfs = fixture.user.path.vfs().clone();
        let fs_context = fixture.user.path.context().clone();
        let root = vfs.root();
        let source = checked(
            context,
            vfs.resolve(&fs_context, b"/pattern", ResolveMode::Follow),
        )?;
        let source_inode = checked(context, source.inode())?.clone();
        drop(fixture.open(context, b"/", 0)?);
        drop(fixture.open(context, b"/pattern", 0)?);
        let observer = fixture.user.path.owner.observer().clone();
        install_lookup_gate(context, &vfs, observer.clone(), b"w28-publish")?;
        observer.hold_loader.store(true, Ordering::Release);
        let before_loads = vfs.dcache().ktest_load_count();
        let parent = root.dentry().clone();
        fixture.user.path.owner.publish(
            0,
            Job::Lookup {
                vfs: vfs.clone(),
                parent: parent.clone(),
                name: b"w28-publish",
            },
        );
        let first_ready = until("metadata loader did not reach actual InLookup", || {
            observer.loader_entered.load(Ordering::Acquire) == 1
                && fixture.user.path.owner.observer().blocked(0)
        });
        // Step 2: 在任务 0 停留在 `InLookup` 期间，依次派发同名查找任务 1 与硬链接创建任务，确认二者均进入阻塞等待。
        let mut observation = None;
        let scenario = (|| -> TestResult {
            first_ready?;
            fixture.user.path.owner.publish(
                1,
                Job::Lookup {
                    vfs: vfs.clone(),
                    parent: parent.clone(),
                    name: b"w28-publish",
                },
            );
            until("metadata lookup waiter did not Block", || {
                vfs.dcache().ktest_waiting(root.dentry(), b"w28-publish") == 1
                    && fixture.user.path.owner.observer().blocked(1)
            })?;
            observation = Some(fixture.consume_sync(0, {
                let vfs = vfs.clone();
                let context = fs_context.clone();
                let start = root.clone();
                let source = source.clone();
                move || vfs.link_from(&source, &context, &start, b"w28-publish", (0, 0))
            })?);
            until(
                "namespace writer did not Block on directory read side",
                || observation.as_ref().is_some_and(|state| state.blocked()),
            )?;
            context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 1);
            Ok(())
        })();

        // Step 3: 放行 `InLookup` 门闩，汇合全部读写任务，验证先行查找返回 `NotFound` 而后续解析命中新发布的硬链接。
        observer.hold_loader.store(false, Ordering::Release);
        observer.gate.complete();
        let reader_join = fixture.user.path.owner.join_tasks();
        let writer_join = fixture.join_consumers();
        vfs.dcache().ktest_set_lookup_hook(None);
        scenario?;
        reader_join?;
        writer_join?;
        context.expect_eq(
            observation
                .as_ref()
                .ok_or(TestError("namespace writer observation absent"))?
                .result(),
            Some(Ok(())),
        );
        let results = core::mem::replace(&mut *observer.results.lock(), [const { None }; 3]);
        for result in IntoIterator::into_iter(results).take(2) {
            context.expect_eq(
                result
                    .ok_or(TestError("metadata lookup waiter result absent"))?
                    .err(),
                Some(FsError::NotFound),
            );
        }
        context.expect_eq(vfs.dcache().ktest_waiting(root.dentry(), b"w28-publish"), 0);

        let published = checked(
            context,
            vfs.resolve(&fs_context, b"/w28-publish", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, published.inode())?.ptr_eq(&source_inode),
            true,
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 1);
        context.diagnostic(format_args!(
            "W28 metadata-loading unique-InLookup=1 real-waiters=1 writer-Blocked=1 old-result=NotFound post-write-current-inode=1 negative-snapshot-not-republished=1 hardware-EIO-covered=0"
        ));
        Ok(())
    }

    /// 验证目录块发生元数据校验失败时，所有合并等待的查找线程共享同一错误、失败占位项被清理，且修复目录块后可重新加载成功。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：读取根目录第 0 个数据块并构造破坏目录项长度字段（偏移 4..6 置零）的损坏副本；安装 `"w28-checksum"` 查找门闩并启动任务 0 停留在 `InLookup`。
    /// - **When**：在任务 0 阻塞期间将损坏字节写入根目录元数据缓存块，启动任务 1 合并等待同一目录项，随后放行门闩。
    /// - **Then**：
    ///   1. 任务 0 与任务 1 均收到预期的一致性校验错误，且 `DentryCache` 中不残留失败占位项；
    ///   2. 将根目录块恢复为原始合法内容后，调用 `vfs.link_from` 创建 `"w28-checksum"` 并解析，触发第二次目录加载并成功命中目标 Inode。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn directory_loading_preserves_metadata_error_and_reloads(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 读取根目录第 0 块原始字节，构造破坏 `rec_len`（偏移 4..6 置 0）的损坏镜像并计算出预期的校验错误码。
        fixture.prepare_namespace(context)?;
        let vfs = fixture.user.path.vfs().clone();
        let fs_context = fixture.user.path.context().clone();
        let root = vfs.root();
        let volume = fixture.volume();
        let root_inode = checked(context, root.inode())?.clone();
        let source = checked(
            context,
            vfs.resolve(&fs_context, b"/pattern", ResolveMode::Follow),
        )?;
        let source_inode = checked(context, source.inode())?.clone();
        drop(fixture.open(context, b"/", 0)?);
        drop(fixture.open(context, b"/pattern", 0)?);
        let (metadata, map_root) = checked(
            context,
            root_inode.with_map_read(|metadata, root| Ok((*metadata, *root))),
        )?;
        let physical = match volume.map_block(&metadata, &map_root, 0, None) {
            Ok(BlockMapping::Mapped { physical, .. }) => physical,
            Ok(_) => return Err(TestError("root directory block is not mapped")),
            Err(error) => {
                context.diagnostic(format_args!("W28 root block lookup error={:?}", error));
                return Err(TestError("root directory mapping failed"));
            }
        };
        let buffer = checked(context, volume.metadata_block(physical))?;
        let geometry = volume.geometry();
        let original = directory_block(&volume, &metadata, &map_root, 0).map_err(|error| {
            context.diagnostic(format_args!("W28 root metadata read error={:?}", error));
            TestError("root metadata read failed")
        })?;
        let mut corrupted = original;
        corrupted[4..6].copy_from_slice(&0u16.to_le_bytes());
        let expected = codec::verify_directory(
            geometry,
            &metadata,
            0,
            &corrupted[..geometry.block_size],
            false,
        )
        .err()
        .ok_or(TestError(
            "controlled metadata corruption did not fail layout or CRC",
        ))?;

        // Step 2: 启动查找任务 0 并拦截在 `InLookup` 状态，此时注入根目录块损坏数据并派发任务 1 加入同一等待队列。
        let observer = fixture.user.path.owner.observer().clone();
        install_lookup_gate(context, &vfs, observer.clone(), b"w28-checksum")?;
        observer.hold_loader.store(true, Ordering::Release);
        let before_len = vfs.dcache().len();
        let before_loads = vfs.dcache().ktest_load_count();
        let parent = root.dentry().clone();
        fixture.user.path.owner.publish(
            0,
            Job::Lookup {
                vfs: vfs.clone(),
                parent: parent.clone(),
                name: b"w28-checksum",
            },
        );
        let first_ready = until("checksum loader did not reach published Loading", || {
            observer.loader_entered.load(Ordering::Acquire) == 1
                && fixture.user.path.owner.observer().blocked(0)
        });
        let scenario = (|| -> TestResult {
            first_ready?;
            checked(
                context,
                buffer
                    .update(volume.claim(), |bytes| {
                        bytes.copy_from_slice(&corrupted[..geometry.block_size]);
                    })
                    .map_err(FsError::from),
            )?;
            fixture.user.path.owner.publish(
                1,
                Job::Lookup {
                    vfs: vfs.clone(),
                    parent: parent.clone(),
                    name: b"w28-checksum",
                },
            );
            until("checksum metadata waiter did not Block", || {
                vfs.dcache().ktest_waiting(root.dentry(), b"w28-checksum") == 1
                    && fixture.user.path.owner.observer().blocked(1)
            })?;
            observer.hold_loader.store(false, Ordering::Release);
            observer.gate.complete();
            Ok(())
        })();

        // Step 3: 汇合查找任务并恢复根目录块原始数据，验证两个等待者收到相同错误、失败占位项已移除，且后续创建并解析同名硬链接成功。
        observer.hold_loader.store(false, Ordering::Release);
        observer.gate.complete();
        let joined = fixture.user.path.owner.join_tasks();
        let restored = buffer
            .update(volume.claim(), |bytes| {
                bytes.copy_from_slice(&original[..geometry.block_size])
            })
            .map_err(FsError::from);
        vfs.dcache().ktest_set_lookup_hook(None);
        joined?;
        checked(context, restored)?;
        scenario?;
        let results = core::mem::replace(&mut *observer.results.lock(), [const { None }; 3]);
        for result in IntoIterator::into_iter(results).take(2) {
            context.expect_eq(
                result
                    .ok_or(TestError("metadata waiter result absent"))?
                    .err(),
                Some(expected),
            );
        }
        context.expect_eq(vfs.dcache().len(), before_len);
        context.expect_eq(
            vfs.dcache().ktest_waiting(root.dentry(), b"w28-checksum"),
            0,
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 1);
        context.expect_eq(
            vfs.dcache()
                .lookup(vfs.inodes(), root.dentry(), b"w28-checksum")
                .err(),
            Some(FsError::NotFound),
        );

        checked(
            context,
            vfs.link_from(&source, &fs_context, &root, b"w28-checksum", (0, 0)),
        )?;
        let current = checked(
            context,
            vfs.resolve(&fs_context, b"/w28-checksum", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, current.inode())?.ptr_eq(&source_inode),
            true,
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 2);
        context.diagnostic(format_args!(
            "W28 metadata-error first-error={:?} shared-waiters=1 failed-placeholder-removed=1 restored-current-value=1 hardware-EIO-covered=0",
            expected
        ));
        Ok(())
    }

    /// 验证文件页（`Folio`）处于磁盘读入（I/O 未完成）状态时，同页并发读者与写者均等待原始页加载完成，随后写者原地更新页面字节。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：预热 `"/pattern"` 文件第 0 页的逻辑块映射，挂起块设备完成回调（`test_suspend_completions`），启动读任务 0 触发第 0 页下发磁盘读请求并阻塞。
    /// - **When**：在磁盘读请求在途期间，启动读任务 1 读取同一页，并启动写任务向文件偏移 0 写入 32 字节的 `0x5a`，确认二者均阻塞且未重复下发读 I/O；随后恢复块设备完成回调。
    /// - **Then**：
    ///   1. 两个读任务共享同一个物理页帧（PFN）并获得完整的 4096 字节就绪页；
    ///   2. 写任务在原始页加载完成后覆盖前 32 字节，最终文件前 32 字节为 `0x5a`、后续字节仍保留原始 `pattern` 数据，且页缓存中的标准页（Canonical Folio）物理地址不变。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn file_page_loading_serializes_write_before_current_value(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 预热第 0 页的块映射，挂起磁盘完成中断，派发读任务 0 使第 0 页停留在磁盘读 I/O 在途状态。
        fixture.prepare_namespace(context)?;
        let file = fixture.open(context, b"/pattern", 2)?;
        let path = file.path().unwrap().clone();
        let inode = checked(context, path.inode())?.clone();
        let volume = fixture.volume();
        let block_size = volume.geometry().block_size as u64;
        for logical in 0..(mm::PAGE_SIZE as u64 / block_size) {
            checked(context, inode.map_block(logical))?;
        }

        let disk = fixture.user.path.owner.disk()?.clone();
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        fixture.user.path.owner.publish(
            0,
            Job::Read {
                inode: inode.clone(),
                page: 0,
            },
        );
        let loader_ready = until("file page loader did not enter real data I/O", || {
            let snapshot = disk.test_snapshot();
            before.file_read_submitted < snapshot.file_read_submitted
                && snapshot.inflight != 0
                && fixture.user.path.owner.observer().blocked(0)
        });
        // Step 2: 在第 0 页未就绪期间，依次派发同页读任务 1 与前 32 字节覆写任务，确认二者均阻塞在同一未就绪 Folio 上且无重复读请求。
        let mut writer = None;
        let scenario = (|| -> TestResult {
            loader_ready?;
            fixture.user.path.owner.publish(
                1,
                Job::Read {
                    inode: inode.clone(),
                    page: 0,
                },
            );
            until(
                "same-page reader did not Block on the published folio",
                || fixture.user.path.owner.observer().blocked(1),
            )?;
            let folio = inode
                .mapping()
                .cached_folio(0)
                .ok_or(TestError("loading folio owner disappeared"))?;
            context.expect_eq(folio.is_ready(), false);
            let file_read_requests = disk.test_snapshot().file_read_submitted;
            writer = Some(fixture.consume_sync(0, {
                let file = file.clone();
                move || {
                    let replacement = [0x5a; 32];
                    if file.write_at(0, &replacement)? != replacement.len() {
                        return Err(FsError::Corrupt);
                    }
                    Ok(())
                }
            })?);
            until(
                "same-page writer did not Block behind original page load",
                || writer.as_ref().is_some_and(|state| state.blocked()),
            )?;
            context.expect_eq(disk.test_snapshot().file_read_submitted, file_read_requests);
            context.expect_eq(folio.is_ready(), false);
            drop(folio);
            Ok(())
        })();

        // Step 3: 恢复磁盘完成回调并汇合读写线程，验证读者共享同一物理页帧且最终页面保留了写者的前 32 字节更新与其余原始字节。
        disk.test_resume_completions();
        let readers = fixture.user.path.owner.join_tasks();
        let writers = fixture.join_consumers();
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        scenario?;
        readers?;
        writers?;
        context.expect_eq(
            writer
                .as_ref()
                .ok_or(TestError("file writer observation absent"))?
                .result(),
            Some(Ok(())),
        );
        let results = core::mem::replace(
            &mut *fixture.user.path.owner.observer().results.lock(),
            [const { None }; 3],
        );
        let mut address = None;
        for result in IntoIterator::into_iter(results).take(2) {
            let returned = checked(context, result.ok_or(TestError("file read result absent"))?)?;
            let Returned::Page {
                folio,
                bytes,
                valid,
            } = returned
            else {
                return Err(TestError("file read returned wrong result"));
            };
            context.expect_eq(
                (bytes, valid, folio.is_ready()),
                (mm::PAGE_SIZE, true, true),
            );
            if let Some(address) = address {
                context.expect_eq(folio.phys_addr(), address);
            } else {
                address = Some(folio.phys_addr());
            }
        }

        let mut data = [0xa7; mm::PAGE_SIZE];
        context.expect_eq(checked(context, inode.read_at(0, &mut data))?, data.len());
        context.expect_eq(&data[..32], &[0x5a; 32]);
        context.expect_eq(patterned(&data[32..], 32), true);
        let cached = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("current file folio disappeared"))?;
        context.expect_eq(inode.mapping().is_canonical(0, &cached), true);
        context.expect_eq(Some(cached.phys_addr()), address);
        context.diagnostic(format_args!(
            "W28 file-page real-data-request-accepted=1 completion-paused=1 same-page-reader-Blocked=1 writer-Blocked=1 one-canonical-folio=1 old-read-valid=1 final-current-bytes=1 pre-IO-hook-used=0 hardware-DMA-state-claimed=0"
        ));
        Ok(())
    }

    /// 验证文件页磁盘读入发生 I/O 错误时，合并等待的读者共享该错误，且在失败页帧从缓存回收后重新读取会再次下发磁盘请求。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：挂起块设备完成回调，启动读任务 0 发起 `"/pattern"` 第 0 页的磁盘读取。
    /// - **When**：注入下一次文件读完成失败（`BlockError::Io`），启动读任务 1 合并等待同一未就绪 `Folio`，随后恢复块设备完成回调。
    /// - **Then**：
    ///   1. 读任务 0 与读任务 1 均返回 `FsError::Io(BlockError::Io)`；
    ///   2. 释放局部引用并调用 `prune_unused` 回收失败的缓存页后，再次调用 `inode.read_at` 会重新提交磁盘读请求并读出正确的 `pattern` 内容。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn failed_file_page_loading_retries_only_after_real_retirement(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 预热块映射并挂起磁盘完成回调，启动读任务 0 使第 0 页进入磁盘读 I/O 在途状态。
        fixture.prepare_namespace(context)?;
        let file = fixture.open(context, b"/pattern", 0)?;
        let inode = checked(context, file.path().unwrap().inode())?.clone();
        let volume = fixture.volume();
        let block_size = volume.geometry().block_size as u64;
        for logical in 0..(mm::PAGE_SIZE as u64 / block_size) {
            checked(context, inode.map_block(logical))?;
        }

        let disk = fixture.user.path.owner.disk()?.clone();
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        fixture.user.path.owner.publish(
            0,
            Job::Read {
                inode: inode.clone(),
                page: 0,
            },
        );
        let loader_ready = until("failing page did not submit original data request", || {
            let snapshot = disk.test_snapshot();
            before.file_read_submitted < snapshot.file_read_submitted
                && snapshot.inflight != 0
                && fixture.user.path.owner.observer().blocked(0)
        });
        // Step 2: 注入单次文件读 I/O 错误并启动读任务 1 合并等待同一加载中的 Folio。
        let scenario = (|| -> TestResult {
            loader_ready?;
            disk.test_fail_next_file_read(BlockError::Io);
            fixture.user.path.owner.publish(
                1,
                Job::Read {
                    inode: inode.clone(),
                    page: 0,
                },
            );
            until(
                "failed page waiter did not Block on unique loading folio",
                || fixture.user.path.owner.observer().blocked(1),
            )?;
            let folio = inode
                .mapping()
                .cached_folio(0)
                .ok_or(TestError("failing page disappeared before completion"))?;
            context.expect_eq(folio.is_ready(), false);
            Ok(())
        })();

        // Step 3: 恢复磁盘完成回调，确认两个读任务收到相同的 `BlockError::Io`；回收失败 Folio 后重新读取文件，验证新 I/O 请求成功读回完整数据。
        disk.test_resume_completions();
        let readers = fixture.user.path.owner.join_tasks();
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        scenario?;
        readers?;
        context.expect_eq(disk.test_completion_error_pending(), false);
        let results = core::mem::replace(
            &mut *fixture.user.path.owner.observer().results.lock(),
            [const { None }; 3],
        );
        for result in IntoIterator::into_iter(results).take(2) {
            context.expect_eq(
                result
                    .ok_or(TestError("failed page waiter result absent"))?
                    .err(),
                Some(FsError::Io(BlockError::Io)),
            );
        }
        let failed = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("failed page cache owner disappeared"))?;
        context.expect_eq(failed.wait_ready(), Err(BlockError::Io));
        drop(failed);
        context.expect_eq(0 < inode.mapping().prune_unused(), true);
        context.expect_eq(inode.mapping().len(), 0);

        let before_retry = disk.test_snapshot();
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        context.expect_eq(checked(context, inode.read_at(0, &mut bytes))?, bytes.len());
        context.expect_eq(patterned(&bytes, 0), true);
        context.expect_eq(
            before_retry.file_read_submitted < disk.test_snapshot().file_read_submitted,
            true,
        );
        context.diagnostic(format_args!(
            "W28 controlled-data-completion-error=Io original-waiters=2 failed-folio-pruned=1 subsequent-normal-load=1 hardware-EIO-claimed=0"
        ));
        Ok(())
    }
}
