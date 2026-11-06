//! 文件页受限回收、物理页与堆耗尽下的回写救援、预留存储与任务回收作用域测试。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `fs::ktest::user::write` 测试套件中，负责验证页面缓存（Page Cache）与虚拟内存（MM）
//! 在物理页分配器（Buddy）及内核堆（SLUB）耗尽时的页回收（Page Reclaim）与脏页回写（Writeback）协同机制。
//! 上游由 `ktest` 框架调度运行，下游直接驱动 `InodeMapping`、`Mm`、`ReclaimAttempt`、`ReclaimScope`
//! 以及块设备写回域（`WbDomain` / `WbStorage`）。
//!
//! ## 核心机制与状态机流转
//!
//! ### 1. 慢速分配消费者（Slow Consumer）页回收与重试路径
//! ```text
//! [普通页分配: mm::alloc_pages -> None]
//!         │
//!         ▼
//! [触发 ReclaimAttempt::reclaim() (扫描 ≤ 64, 隔离 ≤ 16, rmap 检查 ≤ 64)]
//!         ├── 命中干净且无外部 Pin 的文件页 ──► 解除页表映射 (Unmap + 跨核 TLB 同步) ──► 释放回物理页分配器 ──► 重试分配成功
//!         ├── 命中脏页 (Dirty) 且处于 Normal 模式 ──► 唤醒后台回写任务 (WbDomain Work) 推进一次写回
//!         └── 全部候选页均被 Pin 或处于 NoFs/NoIo 受限模式无进展 ──► 立即停止扫描并返回 FsError::NoMemory / VmError::NoMemory
//! ```
//!
//! ### 2. 任务级回收作用域（`ReclaimScope`）状态转换
//! ```text
//! ReclaimMode::Normal (允许扫描 rmap、卸载文件映射、触发并等待回写 I/O)
//!    │
//!    ├─► ReclaimScope::nofs() ──► ReclaimMode::NoFs (仅回收未映射干净缓存页，禁止扫描 rmap 与发起文件系统写回)
//!    │      │
//!    │      └─► ReclaimScope::noio() ──► ReclaimMode::NoIo (禁止发起任何块设备 I/O，随 Task 跨 CPU 迁移保持)
//!    │
//!    └─► 新派生内核线程 (spawn) ──► 默认恢复为独立的 ReclaimMode::Normal
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 1. **元数据同步与预留缓冲区顺序**：`sync_range` 必须先等待元数据写回互斥锁（`metadata_cache.lock_writeback()`），
//!    随后才按需从预留池借入辅助缓冲区（`aux`）。绝不能在持有 `aux` 的状态下阻塞等待写回锁，否则会导致冷 Inode 读取无法获取 `aux` 而死锁。
//! 2. **文件系统内部锁与回收作用域**：在持有 `inode.lock_invalidate()` 或块映射读锁（`with_map_read`）期间，
//!    当前任务自动处于 `NoFs` / `NoIo` 作用域，回收器不得反向获取页表锁或发起块写回，从而消除重入死锁风险。

use super::*;

/// 内核小对象堆（SLUB）半满预热夹具。
///
/// 在耗尽物理页之前预先为每个 SLUB 大小等级分配一批小对象，随后释放其中一半，
/// 从而在现有的 Slab 页内留下空闲槽位。这样后续测试代码在进行小对象堆分配时
/// 可以直接复用 Slab 内部空闲槽位，而不会向底层物理页分配器申请新页，确保物理页耗尽条件不因辅助分配而提前失败。
struct SmallHeapRoom {
    /// 按 11 个 SLUB 大小等级（`mm::SLUB_SIZES`）分组、每组持有的 16 个堆字节向量槽位。
    objects: [[Option<Vec<u8>>; 16]; 11],
}

impl SmallHeapRoom {
    /// 为每个 SLUB 大小等级预分配 16 个精确容量的向量，撑开各等级活跃的 Slab 页面。
    fn prepare() -> Result<Self, TestError> {
        let mut room = Self {
            objects: core::array::from_fn(|_| core::array::from_fn(|_| None)),
        };
        for (objects, size) in room.objects.iter_mut().zip(mm::SLUB_SIZES) {
            for object in objects {
                let mut bytes = Vec::new();
                bytes
                    .try_reserve_exact(size)
                    .map_err(|_| TestError("slow consumer small heap preparation OOM"))?;
                *object = Some(bytes);
            }
        }
        Ok(room)
    }

    /// 交错释放每个 SLUB 等级中偶数下标的对象，在保持各 Slab 页面不被整页回收的前提下腾出小对象分配空间。
    fn release_half(&mut self) {
        for objects in &mut self.objects {
            for object in objects.iter_mut().step_by(2) {
                drop(object.take());
            }
        }
    }
}

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 测试目标：验证文件缓存未命中加载、文件映射缺页与匿名零页缺页在物理页耗尽时能回收干净文件页完成分配，且当所有文件页被外部引用钉住无进展时返回 `NoMemory`。
    /// 场景步骤：
    /// 1. 写入并同步 19 页文件数据，冷重开后建立只读文件映射、写时复制（COW）私有映射与匿名映射，并将前 16 页预热进页面缓存作为可回收候选页。
    /// 2. 耗尽全部空闲物理页，验证底层物理页分配快速路径不会自动偷收文件缓存；随后根据用例参数触发第 16 页读取、文件映射缺页或匿名缺页。
    /// 3. 断言回收器在有限预算内回收了干净缓存页、完成了目标分配且保留了私有 COW 页；接着持有所剩全部缓存页的 `FolioRef` 再次耗尽物理页，验证再次触发分配时因无页可收而返回 `NoMemory`。
    #[test]
    #[case("file_like_miss", 0u8)]
    #[case("file_fault", 1u8)]
    #[case("anonymous_zero_fault", 2u8)]
    #[ktest(tags("W32", "W33", "fs-reclaim", "fs-slow-consumer"))]
    fn normal_consumers_retry_original_allocator_after_page_none(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        consumer: u8,
    ) -> TestResult {
        const FILE: usize = 0x4b00_0000;
        const ANON: usize = 0x4d00_0000;

        let file = fixture.cold_open(context)?.0;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        checked(context, file.truncate(0))?;
        for index in 0..19 {
            context.expect_eq(
                checked(
                    context,
                    file.write_at((index * PAGE_SIZE) as u64, &[0x60 + index as u8; PAGE_SIZE]),
                )?,
                PAGE_SIZE,
            );
        }
        checked(context, file.sync_range(0..u64::MAX, false))?;
        let old_mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("slow consumer old MM owner OOM"))?;
        let (file, _) = fixture.cold_reopen(context, old_mm, file, inode, volume)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let disk = checked(context, inode.disk_volume())?.disk().clone();
        context.expect_eq(checked(context, file.stat())?.size, (19 * PAGE_SIZE) as u64);

        let mm = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            mm.map_file(
                FILE,
                4 * PAGE_SIZE,
                RO,
                RO,
                file.clone(),
                (16 * PAGE_SIZE) as u64,
                false,
            ),
        )?;
        vm(
            context,
            mm.map_file(SHARED_ALIAS, PAGE_SIZE, RW, RW, file.clone(), 0, true),
        )?;
        vm(context, mm.map_anonymous(ANON, 2 * PAGE_SIZE, RW, RW))?;
        // Step 1: 在同一二级页表覆盖范围内预先建立并触发一个匿名叶子页，固定住上级页表目录，防止后续卸载文件映射叶时因页表页释放干扰回收计数。
        vm(
            context,
            mm.map_anonymous(FILE + 4 * PAGE_SIZE, PAGE_SIZE, RW, RW),
        )?;
        vm(context, mm.fault(FILE + 4 * PAGE_SIZE, UserAccess::Read))?;
        context.expect_eq(mm.copyout(SHARED_ALIAS, &[0x93]).copied, 1);
        let private = mm
            .mapping(SHARED_ALIAS)
            .ok_or(TestError("slow consumer private COW leaf absent"))?
            .physical;

        let mut donors = [0usize; 16];
        let mut preparing = ReclaimAttempt::new();
        for (index, physical) in donors.iter_mut().enumerate() {
            let folio = checked(
                context,
                file.mapping_inode()
                    .unwrap()
                    .file_page(index as u64, &mut preparing),
            )?;
            context.expect_eq(folio.dirty_sectors(), 0);
            context.expect_eq(folio.writeback_sectors(), 0);
            *physical = folio.phys_addr();
        }
        context.expect_eq(inode.mapping().cached_folio(16).is_none(), true);
        context.expect_eq(mm.mapping(FILE).is_none(), true);
        context.expect_eq(mm.mapping(ANON).is_none(), true);
        let mut eof = [0xa7];
        context.expect_eq(file.read_at((19 * PAGE_SIZE) as u64, &mut eof), Ok(0));
        context.expect_eq(eof, [0xa7]);
        context.expect_eq(
            mm.fault(FILE + 3 * PAGE_SIZE, UserAccess::Read),
            Err(VmError::BeyondEof),
        );

        sched::test_migrate_to(0);
        let mut room = SmallHeapRoom::prepare()?;
        let mut pressure = PagePressure::prepare()?;
        disk.test_flush_completions();
        let before_io = disk.test_snapshot();

        // Step 2: 耗尽物理页并交错释放半数 SLUB 对象，验证在显式触发慢速消费者前全部 16 个候选缓存页完好无损。
        let result = (|| -> TestResult {
            let page_none = {
                let _pin = PreemptGuard::new();
                pressure.exhaust()
            };
            room.release_half();
            let small_heap_page_none = mm::alloc_pages(0, false).is_none();
            context.expect_eq(page_none && small_heap_page_none, true);
            if !page_none || !small_heap_page_none {
                return Err(TestError("slow consumer did not start at real page None"));
            }
            let fastpath_preserved = donors.iter().enumerate().all(|(index, physical)| {
                inode
                    .mapping()
                    .cached_folio(index as u64)
                    .map(|folio| folio.phys_addr())
                    == Some(*physical)
            });
            context.expect_eq(fastpath_preserved, true);
            if !fastpath_preserved {
                return Err(TestError(
                    "raw allocator reclaimed file cache before the explicit consumer",
                ));
            }

            // Step 3: 在 Normal 回收模式下分别触发冷文件页读取、文件映射缺页或匿名页缺页，验证干净文件缓存被回收并复用物理页。
            context.expect_eq(ReclaimMode::current(), ReclaimMode::Normal);
            let mut attempt = ReclaimAttempt::new();
            let mut target = None;
            match consumer {
                0 => {
                    target = Some(checked(
                        context,
                        file.mapping_inode().unwrap().file_page(16, &mut attempt),
                    )?)
                }
                1 => vm(context, mm.fault(FILE, UserAccess::Read))?,
                2 => vm(context, mm.fault(ANON, UserAccess::Read))?,
                _ => unreachable!(),
            }

            let retired = (0..16)
                .filter(|index| inode.mapping().cached_folio(*index).is_none())
                .count();
            context.expect_eq(0 < retired, true);
            // 候选页未持额外引用，被回收后由目标请求从分配器重新申请并填入新内容；校验目标页的数据内容与物理页帧号。
            let physical = if consumer == 2 {
                let physical = mm
                    .mapping(ANON)
                    .ok_or(TestError("slow anonymous fault leaf absent"))?
                    .physical;
                mapped_bytes(context, &mm, ANON, &[0; PAGE_SIZE]);
                physical
            } else {
                if target.is_none() {
                    target = inode.mapping().cached_folio(16);
                }
                let folio = target
                    .as_ref()
                    .ok_or(TestError("slow file consumer canonical absent"))?;
                let mut bytes = [0xa7; PAGE_SIZE];
                checked(context, folio.copy_to(0, &mut bytes).map_err(FsError::from))?;
                context.expect_eq(bytes, [0x70; PAGE_SIZE]);
                if consumer == 1 {
                    context.expect_eq(
                        mm.mapping(FILE).map(|leaf| leaf.physical),
                        Some(folio.phys_addr()),
                    );
                    mapped_bytes(context, &mm, FILE, &[0x70; PAGE_SIZE]);
                }
                folio.phys_addr()
            };
            context.expect_eq(
                mm.mapping(SHARED_ALIAS).map(|leaf| leaf.physical),
                Some(private),
            );
            mapped_bytes(context, &mm, SHARED_ALIAS, &[0x93]);
            let stats = attempt.stats();
            context.expect_eq(
                stats.scanned <= 64 && stats.isolated <= 16 && stats.rmap_checks <= 64,
                true,
            );
            if consumer == 0 {
                context.expect_eq(0 < stats.freed_file, true);
            }
            let after_io = disk.test_snapshot();
            context.expect_eq(after_io.data_submitted, before_io.data_submitted);
            if consumer != 2 {
                context.expect_eq(before_io.submitted < after_io.submitted, true);
                context.expect_eq(before_io.completed < after_io.completed, true);
            }
            context.diagnostic(format_args!("W32 slow consumer={} page-none={} small-heap-page-none={} donors={:?} retired-cache={} consumed-physical={:#x} private-physical={:#x} file-api-budget={:?} MM-budget-observer=none IO-submitted={} IO-completed={}",
                consumer, page_none, small_heap_page_none, donors, retired, physical, private,
                if consumer == 0 { Some(stats) } else { None }, after_io.submitted - before_io.submitted, after_io.completed - before_io.completed));

            // Step 4: 持有所剩缓存页的强引用以阻止其被回收，再次耗尽物理页后触发分配，验证回收无进展时准确返回 NoMemory。
            let pins: [Option<FolioRef>; 17] =
                core::array::from_fn(|index| inode.mapping().cached_folio(index as u64));
            let finite_none = {
                let _pin = PreemptGuard::new();
                pressure.exhaust()
            };
            context.expect_eq(finite_none, true);
            if !finite_none {
                return Err(TestError(
                    "slow consumer finite failure did not start at page None",
                ));
            }

            let mut finite = ReclaimAttempt::new();
            let io = disk.test_snapshot();
            if consumer == 0 {
                context.expect_eq(
                    file.mapping_inode()
                        .unwrap()
                        .file_page(17, &mut finite)
                        .err(),
                    Some(FsError::NoMemory),
                );
            } else {
                let address = if consumer == 1 {
                    FILE + PAGE_SIZE
                } else {
                    ANON + PAGE_SIZE
                };
                let error = if consumer == 1 {
                    VmError::File(FsError::NoMemory)
                } else {
                    VmError::NoMemory
                };
                context.expect_eq(mm.fault(address, UserAccess::Read), Err(error));
                context.expect_eq(mm.mapping(address).is_none(), true);
            }

            let finite_stats = finite.stats();
            context.expect_eq(finite_stats.freed_file + finite_stats.freed_tables, 0);
            context.expect_eq(
                finite_stats.scanned <= 64
                    && finite_stats.isolated <= 16
                    && finite_stats.rmap_checks <= 64,
                true,
            );
            context.expect_eq(disk.test_snapshot().submitted, io.submitted);
            context.expect_eq(
                mm.mapping(SHARED_ALIAS).map(|leaf| leaf.physical),
                Some(private),
            );
            mapped_bytes(context, &mm, SHARED_ALIAS, &[0x93]);
            context.diagnostic(format_args!("W33 slow consumer={} no-progress-page-none={} real-NoMemory=1 file-api-budget={:?} completed-is-not-freed=1 retained-canonical-pins={}", consumer, finite_none,
                if consumer == 0 { Some(finite_stats) } else { None }, pins.iter().flatten().count()));
            drop((pins, target));
            Ok(())
        })();

        drop(pressure);
        result?;
        context.expect_eq(checked(context, file.stat())?.size, (19 * PAGE_SIZE) as u64);
        bytes_at(context, &file, (16 * PAGE_SIZE) as u64, &[0x70; PAGE_SIZE])?;
        mapped_bytes(context, &mm, SHARED_ALIAS, &[0x93]);
        Ok(())
    }

    /// 测试目标：验证当缓存中的脏页持有外部 `FolioRef` 引用且物理页与内核堆全部耗尽时，慢速分配路径会唤醒回写工作队列（Work）刷盘并等待一次回写完成，但因页被外部钉住无法释放而最终返回 `NoMemory`。
    /// 场景步骤：
    /// 1. 创建 3 页大小的文件并同步前 2 页，持有第 0 页（待写脏）和第 1 页（干净）的 `FolioRef` 强引用。
    /// 2. 主线程先锁住 Inode 的写回互斥锁（`lock_writeback`），后台消费者线程耗尽物理页与堆后请求读取第 2 页；主线程将第 0 页写脏后放行消费者。
    /// 3. 观测到消费者阻塞等待写回工作任务运行后，主线程释放写回互斥锁，验证后台回写任务利用预留资源将第 0 页刷盘清脏，而消费者因无法释放被钉住的页最终返回 `FsError::NoMemory`。
    #[test]
    #[ktest(
        tags("W33", "fs-reclaim", "fs-reserve", "fs-slow-consumer"),
        min_cpus = 2
    )]
    fn dirty_reclaim_requests_original_work_under_full_heap_none(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let file = fixture.cold_open(context)?.0;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        let disk = volume.disk().clone();
        let domain = volume.writeback_domain().clone();
        checked(context, file.truncate(0))?;
        context.expect_eq(
            checked(context, file.write_at(0, &[0x81; 2 * PAGE_SIZE]))?,
            2 * PAGE_SIZE,
        );
        checked(context, file.truncate((3 * PAGE_SIZE) as u64))?;
        checked(context, file.sync_range(0..u64::MAX, false))?;

        let mut preparing = ReclaimAttempt::new();
        let dirty = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut preparing),
        )?;
        let clean = checked(
            context,
            file.mapping_inode().unwrap().file_page(1, &mut preparing),
        )?;
        let original = dirty.phys_addr();
        let block = volume.geometry().block_size;
        let mut targets = [0u64; 4];
        for (part, target) in targets.iter_mut().enumerate().take(PAGE_SIZE / block) {
            let BlockMapping::Mapped { physical, .. } =
                checked(context, inode.map_block(part as u64))?
            else {
                return Err(TestError("dirty slow consumer original disk block absent"));
            };
            *target = physical * block as u64;
        }

        let pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let gate = Arc::try_new(Completion::new())
            .map_err(|_| TestError("dirty slow consumer gate OOM"))?;
        fixture.prepare_gate = Some(gate.clone());
        let entered = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("dirty slow consumer observation OOM"))?;
        let evidence = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("dirty slow consumer result OOM"))?;
        let actor_entered = entered.clone();
        let actor_evidence = evidence.clone();
        let actor_file = file.clone();
        let actor_gate = gate.clone();
        sched::test_migrate_to(0);
        // Step 1: 预先持有写回互斥锁拦截后台写回任务提交，由预创建的消费者线程在门控放行后耗尽堆与物理页并触发 Normal 回收。
        let writeback = inode.mapping().lock_writeback();
        let actor = fixture.consume_sync(0, move || {
            actor_gate.wait();
            let mut pressure = pressure;
            let allocation = {
                let _pin = PreemptGuard::new();
                pressure.exhaust()
            };
            actor_entered.store(true, Ordering::Release);
            let mut attempt = ReclaimAttempt::new();
            let error = actor_file
                .mapping_inode()
                .unwrap()
                .file_page(2, &mut attempt)
                .err();
            let stats = attempt.stats();
            drop(pressure);
            *actor_evidence.lock() = Some((allocation, stats, error));
            error.map_or(Ok(()), Err)
        })?;

        context.expect_eq(
            checked(context, file.write_at(0, &[0x82; PAGE_SIZE]))?,
            PAGE_SIZE,
        );
        let dirty_mask = dirty.dirty_sectors();
        context.expect_eq(dirty_mask != 0, true);
        disk.test_flush_completions();
        let before_io = disk.test_snapshot();
        let before_domain = domain.snapshot();

        // Step 2: 放行消费者线程，等待其触发后台回写工作队列并进入阻塞等待状态，随后释放写回互斥锁让回写任务完成刷盘。
        gate.complete();
        let waiting = until(
            "dirty slow consumer did not wait for original WB Work",
            || entered.load(Ordering::Acquire) && actor.blocked() && domain.snapshot().running,
        );
        drop(writeback);
        fixture.join_consumers()?;
        waiting?;
        let (allocation, stats, error) = evidence
            .lock()
            .take()
            .ok_or(TestError("dirty slow consumer result absent"))?;
        context.expect_eq(allocation, (true, true));
        context.expect_eq(error, Some(FsError::NoMemory));
        context.expect_eq(actor.result(), Some(Err(FsError::NoMemory)));
        context.expect_eq(
            stats.scanned <= 64 && stats.isolated <= 16 && stats.rmap_checks <= 64,
            true,
        );
        context.expect_eq(stats.freed_file, 0);
        context.expect_eq(stats.freed_tables, 0);
        context.expect_eq(0 < stats.submitted_bytes, true);
        context.expect_eq(0 < stats.completed_bytes, true);
        context.expect_eq(stats.completed_bytes <= stats.submitted_bytes, true);
        context.expect_eq(stats.io_error, false);

        until(
            "dirty slow consumer original Work did not finish its finite round",
            || {
                let state = domain.snapshot();
                !state.running && !state.pending
            },
        )?;
        disk.test_flush_completions();
        let after_domain = domain.snapshot();
        let after_io = disk.test_snapshot();
        context.expect_eq(
            before_domain.finished_rounds < after_domain.finished_rounds,
            true,
        );
        context.expect_eq(before_io.data_submitted < after_io.data_submitted, true);
        context.expect_eq(before_io.completed < after_io.completed, true);
        context.expect_eq(dirty.dirty_sectors(), 0);
        context.expect_eq(dirty.writeback_sectors(), 0);
        context.expect_eq(
            inode
                .mapping()
                .cached_folio(0)
                .map(|folio| folio.phys_addr()),
            Some(original),
        );
        context.expect_eq(
            inode
                .mapping()
                .cached_folio(1)
                .map(|folio| folio.phys_addr()),
            Some(clean.phys_addr()),
        );
        context.expect_eq(inode.mapping().cached_folio(2).is_none(), true);

        bytes_at(context, &file, 0, &[0x82; PAGE_SIZE])?;
        for target in targets.iter().copied().take(PAGE_SIZE / block) {
            disk_block_bytes(context, volume.claim(), target, &[0x82; PAGE_SIZE][..block])?;
        }
        context.diagnostic(format_args!("W33 dirty-original-Work page-none={} heap-none={} original-physical={:#x} dirty-before={} stats={:?} domain-before={:?} domain-after={:?} IO-data-submitted={} IO-completed={} retained-original-pin=1 actual-freed=0 real-NoMemory={:?} independent-disk-bytes={}",
            allocation.0, allocation.1, original, dirty_mask, stats, before_domain, after_domain,
            after_io.data_submitted - before_io.data_submitted, after_io.completed - before_io.completed, error, PAGE_SIZE));
        Ok(())
    }

    /// 测试目标：验证元数据同步（`sync_range`）在等待元数据写回互斥锁期间不会提前占用预留的辅助缓冲区（`aux`），从而允许持有写回互斥锁的并发路径正常使用 `aux` 读取冷 Inode 表块。
    /// 场景步骤：
    /// 1. 写入 1 页文件并完成同步后，计算该 Inode 所在磁盘块组与 Inode 表块号，并通过 `prune_unused` 将 Inode 表块从元数据缓存中驱逐为冷块。
    /// 2. 主线程持有 `cache.lock_writeback()` 锁住元数据写回，唤醒后台同步线程执行 `sync_range` 并等待其在写回互斥锁上阻塞。
    /// 3. 断言此时预留池的 `aux` 缓冲区仍处于空闲状态（`aux_free == true`），主线程随后直接调用 `volume.inode_record` 借用 `aux` 从磁盘冷读并解码 Inode 记录成功，最后释放写回锁让同步线程完成 FLUSH。
    #[test]
    #[ktest(tags("W33", "fs-reserve", "fs-reserve-lock-order"), min_cpus = 2)]
    fn metadata_writeback_wait_leaves_aux_for_cold_inode_read(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let file = fixture.cold_open(context)?.0;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        let disk = volume.disk().clone();
        let cache = volume.metadata_cache().clone();
        let source = cache
            .reserve()
            .ok_or(TestError("metadata lock regression RW source absent"))?;
        if !disk.limits().flush {
            return Err(TestError("metadata lock regression requires real FLUSH"));
        }

        checked(context, file.truncate(0))?;
        context.expect_eq(
            checked(context, file.write_at(0, &[0x76; PAGE_SIZE]))?,
            PAGE_SIZE,
        );
        checked(context, file.sync_range(0..u64::MAX, false))?;
        until(
            "metadata lock regression initial Work did not finish",
            || {
                let state = volume.writeback_domain().snapshot();
                !state.pending && !state.running
            },
        )?;

        let (expected, record_size) = inode_record_snapshot(context, &volume, inode.number())?;
        let geometry = volume.geometry();
        let descriptor_byte =
            (inode.number() - 1) / geometry.inodes_per_group * geometry.descriptor_size as u64;
        let descriptor = checked(
            context,
            cache
                .get(
                    geometry.first_data_block + 1 + descriptor_byte / geometry.block_size as u64,
                    geometry.block_size,
                    None,
                )
                .map_err(FsError::from),
        )?;
        let mut record = [0u8; 64];
        checked(
            context,
            descriptor
                .with_read(|bytes| {
                    let at = (descriptor_byte % geometry.block_size as u64) as usize;
                    record[..geometry.descriptor_size]
                        .copy_from_slice(&bytes[at..at + geometry.descriptor_size]);
                })
                .map_err(FsError::from),
        )?;
        drop(descriptor);
        let layout = checked(
            context,
            codec::group_layout(
                geometry,
                (inode.number() - 1) / geometry.inodes_per_group,
                &record[..geometry.descriptor_size],
            ),
        )?;
        let inode_byte = (inode.number() - 1) % geometry.inodes_per_group * record_size as u64;
        let table_block = layout.inode_table + inode_byte / geometry.block_size as u64;

        let gate = Arc::try_new(Completion::new())
            .map_err(|_| TestError("metadata lock regression gate OOM"))?;
        fixture.prepare_gate = Some(gate.clone());
        let entered = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("metadata lock regression observer OOM"))?;
        let actor_entered = entered.clone();
        let syncing = file.clone();
        let actor_gate = gate.clone();
        let actor = fixture.consume_sync(0, move || {
            actor_gate.wait();
            actor_entered.store(true, Ordering::Release);
            syncing.sync_range(0..u64::MAX, false)
        })?;

        cache.prune_unused();
        let cold = checked(
            context,
            cache
                .cached_buffer(table_block, geometry.block_size)
                .map_err(FsError::from),
        )?
        .is_none();
        context.expect_eq(cold, true);
        if !cold {
            return Err(TestError(
                "metadata lock regression inode table did not become cold",
            ));
        }

        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        let before = disk.test_snapshot();
        // Step 1: 主线程先持有元数据写回互斥锁，随后放行同步线程，观察其阻塞在写回互斥锁上且未提前占用预留的 aux 缓冲区。
        let writeback = cache.lock_writeback();
        gate.complete();
        let result = (|| -> TestResult {
            until("metadata sync did not block on the held WB mutex", || {
                entered.load(Ordering::Acquire) && actor.blocked()
            })?;
            context.expect_eq(actor.result(), None);
            context.expect_eq(disk.test_snapshot().submitted, before.submitted);
            let free_aux = source.snapshot().aux_free;
            context.expect_eq(free_aux, true);
            context.diagnostic(format_args!("W33 metadata-lock-order inode={} cold-block={} sync-blocked=1 held-metadata-WB=1 aux-free-before-cold-read={}", inode.number(), table_block, free_aux));
            // 若同步线程在等待写回锁前错误地先占用了 aux，必须立即中止测试并释放写回锁，避免主线程后续读取冷 Inode 时发生死锁。
            if !free_aux {
                return Err(TestError(
                    "metadata sync acquired aux before waiting for WB",
                ));
            }

            // Step 2: 在持有写回互斥锁且同步线程阻塞的窗口内，借用空闲的 aux 缓冲区从磁盘冷读 Inode 表块并校验 CRC 与解码内容。
            let (buffer, offset) = checked(context, volume.inode_record(inode.number(), None))?;
            let mut actual = [0u8; 256];
            checked(
                context,
                buffer
                    .with_read(|bytes| {
                        actual[..record_size].copy_from_slice(&bytes[offset..offset + record_size]);
                    })
                    .map_err(FsError::from),
            )?;
            context.expect_eq(&actual[..record_size], &expected[..record_size]);
            let decoded = checked(
                context,
                codec::decode_inode(geometry, inode.number(), &actual[..record_size]),
            )?;
            context.expect_eq(decoded.number, inode.number());
            context.expect_eq(decoded.size, PAGE_SIZE as u64);
            drop(buffer);
            disk.test_flush_completions();
            let cold_io = disk.test_snapshot();
            context.expect_eq(before.submitted < cold_io.submitted, true);
            context.expect_eq(before.completed < cold_io.completed, true);
            context.expect_eq(cold_io.inflight, 0);
            context.expect_eq(cold_io.flush_submitted, before.flush_submitted);
            context.expect_eq(source.snapshot().aux_free, true);
            context.expect_eq(actor.result(), None);
            context.diagnostic(format_args!("W33 metadata-lock-order full-record-bytes={} CRC-decoded=1 cold-read-submitted={} cold-read-completed={} aux-returned=1 sync-still-blocked=1", record_size, cold_io.submitted - before.submitted, cold_io.completed - before.completed));
            Ok(())
        })();

        drop(writeback);
        fixture.join_consumers()?;
        result?;
        context.expect_eq(actor.result(), Some(Ok(())));
        disk.test_flush_completions();
        let after = disk.test_snapshot();
        context.expect_eq(after.flush_submitted - before.flush_submitted, 1);
        context.expect_eq(after.inflight, 0);
        context.expect_eq(source.snapshot().aux_free, true);
        bytes_at(context, &file, 0, &[0x76; PAGE_SIZE])?;
        context.diagnostic(format_args!("W33 metadata-lock-order original-sync-FLUSH={} IO-submitted={} IO-completed={} joined=1 fulltail-aux-free={}", after.flush_submitted - before.flush_submitted, after.submitted - before.submitted, after.completed - before.completed, source.snapshot().aux_free));
        Ok(())
    }

    /// 在受限回收作用域（`NoFs` / `NoIo`）下执行一次页面回收尝试，断言扫描预算受限且绝不检查反向映射（rmap）、卸载页表或发起回写 I/O，返回本次释放的文件缓存页数。
    fn cache_only_attempt(context: &mut TestContext<'_>) -> usize {
        let mut attempt = ReclaimAttempt::new();
        let progress = attempt.reclaim();
        let stats = attempt.stats();
        context.expect_eq(stats.scanned <= 64, true);
        context.expect_eq(stats.isolated <= 16, true);
        context.expect_eq(stats.rmap_checks, 0);
        context.expect_eq(stats.unmapped, 0);
        context.expect_eq(stats.freed_tables, 0);
        context.expect_eq(stats.submitted_bytes, 0);
        context.expect_eq(stats.completed_bytes, 0);
        context.expect_eq(stats.io_error, false);
        context.expect_eq(progress, stats.freed_file != 0);
        context.diagnostic(format_args!(
            "W33 cache-only mode={:?} stats={:?}",
            ReclaimMode::current(),
            stats
        ));
        stats.freed_file
    }

    /// 构造并提交一个直接读取块设备的 `Bio` 请求，在 DMA 传输完成后逐字节校验磁盘指定物理偏移处的数据内容。
    fn disk_block_bytes(
        context: &mut TestContext<'_>,
        claim: &crate::block::FsClaim,
        byte: u64,
        expected: &[u8],
    ) -> TestResult {
        let disk = claim.disk();
        let byte = checked(
            context,
            claim
                .view()
                .absolute_byte(byte, expected.len())
                .map_err(FsError::from),
        )?;

        let pages = mm::alloc_pages(0, true).ok_or(TestError("W33 disk readback page OOM"))?;
        let mapped = MappedPages::new(pages, 0, expected.len(), BufferDirection::DeviceToDriver)
            .map_err(|_| TestError("W33 disk readback mapping failed"))?;
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(1)
            .map_err(|_| TestError("W33 disk readback vector OOM"))?;
        memory.push(mapped);
        let bio = checked(
            context,
            // SAFETY:
            // 1. `pages` 是刚刚分配并清零的独占物理页，`mapped` 封装其 `[0, expected.len())` 区间且无任何其他 CPU 引用或别名；
            // 2. `Bio::try_new` 将 `memory` 的 DMA 写入访问权移交给块设备驱动，且未注册异步回调，仅在 `completion.wait()` 确认设备 I/O 结束后才通过 `take_memory()` 收回所有权。
            unsafe {
                Bio::try_new(
                    disk,
                    BlkOperation::Read,
                    byte / crate::block::SECTOR_BYTES,
                    memory,
                    0,
                    None,
                )
            }
            .map_err(FsError::from),
        )?;

        let completion = bio.completion();
        claim.submit_bio(bio);
        let result = completion.wait();
        let memory = checked(context, completion.take_memory().map_err(FsError::from))?;
        if result
            != (IoResult {
                completed: expected.len(),
                error: None,
            })
        {
            return Err(TestError("W33 direct disk readback incomplete"));
        }

        context.expect_eq(memory.len(), 1);
        // SAFETY:
        // 1. `completion.wait()` 已返回且 `result` 校验确认设备成功读入 `expected.len()` 字节，后续不会再有设备 DMA 写入；
        // 2. `take_memory()` 已收回 `memory` 的独占所有权，不存在并发修改或可变别名，切片内全部字节均已由块设备初始化。
        context.expect_eq(unsafe { memory[0].bytes() }, expected);
        drop(memory);
        drop(completion);
        Ok(())
    }

    /// 从 Inode 的块映射根节点中提取外部间接块（传统三级间接块模式）或 Extent 树内部索引节点（Extent 模式）的物理块号列表，要求树深度大于 0 且不能全部为空洞。
    fn external_root_nodes(inode: &InodeRef, traditional: bool) -> Result<[u64; 4], FsError> {
        inode.with_map_read(|metadata, root| {
            if (metadata.flags & codec::EXTENTS == 0) != traditional {
                return Err(FsError::InvalidOperation);
            }
            let mut nodes = [0; 4];
            if traditional {
                for (slot, index) in (12..15).enumerate() {
                    nodes[slot] = u64::from(codec::le32(root, index * 4)?);
                }
            } else {
                let header = codec::extent_header(root, None)?;
                if header.depth == 0 || nodes.len() < header.entries as usize {
                    return Err(FsError::InvalidOperation);
                }
                for (slot, node) in nodes.iter_mut().enumerate().take(header.entries as usize) {
                    let at = 12 + slot * 12;
                    *node = u64::from(codec::le32(root, at + 4)?)
                        | (u64::from(codec::le16(root, at + 8)?) << 32);
                }
            }
            if nodes.iter().all(|node| *node == 0) {
                return Err(FsError::InvalidOperation);
            }
            Ok(nodes)
        })
    }

    /// 输出指定测试阶段的元数据缓存活跃统计、已创建任务总数以及各 CPU 工作队列池（Workqueue Pool）的救援线程执行计数诊断日志。
    fn reclaim_cost(context: &TestContext<'_>, phase: &str) {
        context.diagnostic(format_args!(
            "W33 cost phase={} live={:?} task-created={} task-stack-bytes={} cpus={} prebuilt-rescuers-in-live-baseline=1",
            phase, metadata_stats(), task::test_created_tasks(), 16 * PAGE_SIZE, cpu::count()
        ));
        for id in 0..cpu::count() {
            let pool = crate::workqueue::testing::pool_stats(id);
            context.diagnostic(format_args!(
                "W33 worker-cost phase={} cpu={} workers={} created={} retired={} create-failures={} rescued-executions={}",
                phase, id, pool.workers, pool.created, pool.retired, pool.create_failures, pool.rescued
            ));
        }
    }

    /// 测试目标：验证在外部 Extent 树和传统间接块树两种格式下，当物理页与内核堆全部耗尽时，文件数据写回、冷块树索引节点按需读入、元数据写回与设备 FLUSH 均能完全依靠预留存储（`WbStorage` 与 `Reserve`）在 `NoIo` 作用域下顺利完成。
    /// 场景步骤：
    /// 1. 预借入写回域的唯一预留存储 `WbStorage`，向文件跨度写入 16 个数据页迫使块树生长出外部索引节点，完成初次同步并将全部 16 页重新写脏，随后驱逐块树外部索引节点使其变为冷未命中状态。
    /// 2. 耗尽系统的物理页与内核堆，调用 `sync_inode_with_storage` 和 `sync_metadata_with_storage` 完成数据与元数据同步。
    /// 3. 断言在此期间未发生内存分配失败，冷索引节点通过预留的 `aux` 缓冲区完成磁盘读入，16 个脏页直接在当前缓存物理页上完成 DMA 回写并最终发出 FLUSH，最后直接从块设备读回全部物理块验证落盘数据一致。
    #[test]
    #[case("external_extent", false)]
    #[case("traditional_indirect", true)]
    #[ktest(tags("W33", "fs-reserve", "fs-reclaim"))]
    fn reserved_writeback_cold_tree_and_flush_under_full_pressure(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        traditional: bool,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-tree", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.volume();
        let disk = volume.disk().clone();
        let block = volume.geometry().block_size;
        if !disk.limits().flush || (traditional && block != 1024) {
            return Err(TestError(
                "W33 requires real FLUSH and the selected tree disk geometry",
            ));
        }

        checked(context, file.truncate(0))?;
        let source = volume
            .metadata_cache()
            .reserve()
            .ok_or(TestError("W33 RW reserve absent"))?;
        let domain = volume.writeback_domain().clone();
        // Step 1: 在获取任何 Inode 锁或写回锁之前预先借入写回域的预留存储 `WbStorage`。
        // 后台写回任务若竞争该预留存储会自行等待，当前任务不依赖动态分配任何新的写回上下文。
        let mut storage = checked(context, domain.borrow_storage())?;
        context.expect_eq(
            storage
                .metadata_io_context(&domain)
                .is_some_and(|io| io.belongs_to(&source)),
            true,
        );

        let mut folios: [Option<FolioRef>; 16] = core::array::from_fn(|_| None);
        let mut targets = [[0u64; 4]; 16];
        let mut offsets = [0u64; 16];
        let mut prepare_attempt = ReclaimAttempt::new();
        for index in 0..16 {
            offsets[index] = ((4 + index * 2) * PAGE_SIZE) as u64;
            let input = [0x40 + index as u8; PAGE_SIZE];
            context.expect_eq(
                checked(context, file.write_at(offsets[index], &input))?,
                PAGE_SIZE,
            );
            folios[index] = Some(checked(
                context,
                file.mapping_inode()
                    .unwrap()
                    .file_page(offsets[index] / PAGE_SIZE as u64, &mut prepare_attempt),
            )?);
            for part in 0..PAGE_SIZE / block {
                let BlockMapping::Mapped { physical, .. } = checked(
                    context,
                    inode.map_block((offsets[index] + (part * block) as u64) / block as u64),
                )?
                else {
                    return Err(TestError("W33 prepared data block is not mapped"));
                };
                targets[index][part] = physical * block as u64;
            }
        }

        let nodes = checked(context, external_root_nodes(&inode, traditional))?;
        let (record, _) = checked(context, volume.inode_record(inode.number(), None))?;
        // Step 2: 先将新建的外部块树节点刷盘落稳，随后将 16 个文件页重新写脏，并调用 `prune_unused` 驱逐未被引用的外部块树节点以构造冷读取场景。
        checked(
            context,
            crate::fs::writeback::sync_inode_with_storage(&inode, 0..u64::MAX, &mut storage),
        )?;
        checked(
            context,
            crate::fs::writeback::sync_metadata_with_storage(&volume, &mut storage),
        )?;
        disk.test_flush_completions();
        for node in nodes.iter().copied().filter(|node| *node != 0) {
            let buffer = checked(context, volume.metadata_block(node))?;
            context.expect_eq(buffer.folio().dirty_sectors(), 0);
            context.expect_eq(buffer.folio().writeback_sectors(), 0);
            drop(buffer);
        }

        for index in 0..16 {
            let input = [0x70 + index as u8; PAGE_SIZE];
            context.expect_eq(
                checked(context, file.write_at(offsets[index], &input))?,
                PAGE_SIZE,
            );
            context.expect_eq(folios[index].as_ref().unwrap().dirty_sectors(), u8::MAX);
        }
        volume.metadata_cache().prune_unused();
        for node in nodes.iter().copied().filter(|node| *node != 0) {
            if checked(
                context,
                volume
                    .metadata_cache()
                    .cached_buffer(node, block)
                    .map_err(FsError::from),
            )?
            .is_some()
            {
                return Err(TestError(
                    "W33 external tree node did not become a real cold miss",
                ));
            }
        }

        sched::test_migrate_to(0);
        reclaim_cost(context, "pressure-ready");
        let layouts = source.snapshot();
        context.expect_eq(layouts.ready, true);
        context.expect_eq(layouts.metadata_capacity, 64);
        context.diagnostic(format_args!(
            "W33 source-before={:?} scratch-bytes={} node-payload-bytes={}",
            layouts,
            core::mem::size_of::<crate::fs::writeback::WbStorage>(),
            crate::cache::ktest_node_bytes()
        ));
        let before = disk.test_snapshot();
        let domain_before = domain.snapshot();
        let evidence = {
            let _pin = PreemptGuard::new();
            fixture.user.pressure().exhaust()
        };

        // Step 3: 在物理页与内核堆双耗尽状态下，使用预留存储依次完成文件数据写回（含冷块树节点读入）与元数据写回（含设备 FLUSH）。
        let pressured = (|| -> Result<_, FsError> {
            if evidence != (true, true) {
                return Err(FsError::NoMemory);
            }
            let mode = ReclaimMode::current();
            crate::fs::writeback::sync_inode_with_storage(&inode, 0..u64::MAX, &mut storage)?;
            disk.test_flush_completions();
            let data = disk.test_snapshot();
            let data_domain = domain.snapshot();
            let dma = disk.ktest_last_folio_dma();
            let after_data_source = source.snapshot();

            crate::fs::writeback::sync_metadata_with_storage(&volume, &mut storage)?;
            disk.test_flush_completions();
            Ok((
                mode,
                data,
                data_domain,
                dma,
                after_data_source,
                disk.test_snapshot(),
                domain.snapshot(),
                source.snapshot(),
            ))
        })();

        // Step 4: 同步接口已在内部等待全部已提交 I/O 完成；释放内存压力后校验 I/O 计数、DMA 散聚表（SG）物理地址及预留池归还状态，并直接读盘核验数据。
        fixture.user.release_pressure();
        let (mode, data, data_domain, dma, data_source, after, domain_after, after_source) =
            checked(context, pressured)?;
        context.expect_eq(evidence, (true, true));
        context.expect_eq(mode, ReclaimMode::NoIo);
        context.expect_eq(data.metadata_submitted, before.metadata_submitted);
        context.expect_eq(data.flush_submitted, before.flush_submitted);
        context.expect_eq(
            0 < data.submitted - before.submitted - (data.data_submitted - before.data_submitted),
            true,
        );
        context.expect_eq(2 <= data.data_submitted - before.data_submitted, true);
        context.expect_eq(
            data.submitted - before.submitted,
            data.completed - before.completed,
        );
        context.expect_eq(data.inflight, 0);
        context.expect_eq(
            data_domain.submitted_bytes - domain_before.submitted_bytes,
            (16 * PAGE_SIZE) as u64,
        );
        context.expect_eq(
            data_domain.completed_bytes - domain_before.completed_bytes,
            (16 * PAGE_SIZE) as u64,
        );
        let (original, address, offset, length) =
            dma.ok_or(TestError("W33 real data SG sample absent"))?;
        context.expect_eq(
            folios
                .iter()
                .flatten()
                .any(|folio| folio.phys_addr() == original),
            true,
        );
        context.expect_eq(address, (original + offset) as u64);
        context.expect_eq(0 < length && offset + length <= PAGE_SIZE, true);
        context.expect_eq(0 < after.metadata_submitted - data.metadata_submitted, true);
        context.expect_eq(after.data_submitted, data.data_submitted);
        context.expect_eq(after.flush_submitted - data.flush_submitted, 1);
        context.expect_eq(
            after.submitted - data.submitted,
            after.completed - data.completed,
        );
        context.expect_eq(after.inflight, 0);
        context.expect_eq(
            domain_after.submitted_bytes - domain_before.submitted_bytes,
            domain_after.completed_bytes - domain_before.completed_bytes,
        );
        context.expect_eq(
            domain_after.accepted_bytes - domain_before.accepted_bytes,
            domain_after.completed_bytes - domain_before.completed_bytes,
        );
        for snapshot in [data_source, after_source] {
            context.expect_eq(snapshot.control_live, 0);
            context.expect_eq(snapshot.payload_live, 0);
            context.expect_eq(snapshot.leased, 0);
            context.expect_eq(snapshot.aux_free, true);
        }

        let mut verify_attempt = ReclaimAttempt::new();
        for (index, folio) in folios.iter().flatten().enumerate() {
            context.expect_eq(folio.dirty_sectors(), 0);
            context.expect_eq(folio.writeback_sectors(), 0);
            context.expect_eq(
                checked(
                    context,
                    file.mapping_inode()
                        .unwrap()
                        .file_page(offsets[index] / PAGE_SIZE as u64, &mut verify_attempt),
                )?
                .phys_addr(),
                folio.phys_addr(),
            );
            context.diagnostic(format_args!(
                "W33 original-page index={} file-offset={} physical={:#x}",
                index,
                offsets[index],
                folio.phys_addr()
            ));
        }
        context.expect_eq(record.folio().dirty_sectors(), 0);
        context.expect_eq(record.folio().writeback_sectors(), 0);

        drop(storage);
        let returned = source.snapshot();
        context.expect_eq(returned.free_wb, 8);
        context.expect_eq(returned.booked, 0);
        context.diagnostic(format_args!("W33 format={} page-null={} heap-null={} data-submitted={} cold-read-submitted={} data-stage-completed={} metadata-submitted={} flush-submitted={} metadata-flush-stage-completed={} domain-before={:?} domain-after={:?} recent-real-SG={:?} source-after={:?}",
            if traditional { "traditional" } else { "external-extent" }, evidence.0, evidence.1,
            data.data_submitted - before.data_submitted,
            data.submitted - before.submitted - (data.data_submitted - before.data_submitted),
            data.completed - before.completed, after.metadata_submitted - data.metadata_submitted,
            after.flush_submitted - data.flush_submitted, after.completed - data.completed,
            domain_before, domain_after, dma, returned));

        let readback = disk.test_snapshot();
        for (index, target) in targets[..16].iter().enumerate() {
            let expected = [0x70 + index as u8; PAGE_SIZE];
            for part in 0..PAGE_SIZE / block {
                disk_block_bytes(
                    context,
                    volume.claim(),
                    target[part],
                    &expected[part * block..(part + 1) * block],
                )?;
            }
        }
        disk.test_flush_completions();
        let readback_after = disk.test_snapshot();
        context.expect_eq(
            readback_after.submitted - readback.submitted,
            16 * PAGE_SIZE / block,
        );
        context.expect_eq(
            readback_after.completed - readback.completed,
            16 * PAGE_SIZE / block,
        );
        context.diagnostic(format_args!(
            "W33 independent-device-readback-bytes={} expected-pages=16 cache-readback=0",
            16 * PAGE_SIZE
        ));
        reclaim_cost(context, "pressure-ended");
        Ok(())
    }

    /// 测试目标：验证任务级回收作用域（`ReclaimScope`）在同任务嵌套、跨核迁移以及派生新任务时的传播与隔离规则，并确认在持有文件系统锁的 `NoFs`/`NoIo` 作用域下仅回收未映射的干净缓存页、遇到全压无进展时立即返回 `NoMemory`。
    /// 场景步骤：
    /// 1. 准备干净但已映射的文件页、待写脏的文件页、正在回写的文件页与匿名页，验证 `NoFs` 与 `NoIo` 嵌套作用域在跨全部 CPU 迁移时保持在当前任务上，且新派生的子任务默认恢复为 `Normal` 模式。
    /// 2. 挂起块设备完成通知使 `wb` 页停留在在途回写状态、将 `dirty` 页写脏，随后在持有 `inode.lock_invalidate()` 与块映射读锁的 `NoFs`/`NoIo` 作用域内执行回收，断言不会扫描 rmap、不会触碰已映射页/脏页/回写页/元数据页。
    /// 3. 先耗尽所有可回收的未映射干净缓存，再耗尽物理页与内核堆，验证在持有文件系统锁时调用 `prepare_file` 会受限于有限预算立即返回 `FsError::NoMemory`；最后恢复 `Normal` 模式并释放外部引用，验证此前映射的干净文件页能通过 rmap 卸载并成功回收。
    #[test]
    #[ktest(tags("W33", "fs-reclaim", "fs-reserve"), min_cpus = 2)]
    fn task_scopes_migrate_cache_only_and_finite_no_memory(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.volume();
        let io_file = fixture.open(context, b"/test/write-user", 2)?;
        let io_inode = fixture.inodes.last().unwrap().clone();
        context.expect_eq(
            checked(context, file.write_at(0, &[0x51; 2 * PAGE_SIZE]))?,
            2 * PAGE_SIZE,
        );
        context.expect_eq(
            checked(context, io_file.write_at(0, &[0x52; PAGE_SIZE]))?,
            PAGE_SIZE,
        );
        checked(context, crate::fs::writeback::sync_inode(&inode))?;
        checked(context, crate::fs::writeback::sync_inode(&io_inode))?;
        checked(context, crate::fs::writeback::sync_metadata(&volume))?;

        let mut file_attempt = ReclaimAttempt::new();
        let clean = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(0, &mut file_attempt),
        )?;
        let dirty = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(1, &mut file_attempt),
        )?;
        let mut io_attempt = ReclaimAttempt::new();
        let wb = checked(
            context,
            io_file
                .mapping_inode()
                .unwrap()
                .file_page(0, &mut io_attempt),
        )?;

        let mm = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            mm.map_file(SOURCE, PAGE_SIZE, RO, RO, file.clone(), 0, false),
        )?;
        vm(context, mm.map_anonymous(SHARED_ALIAS, PAGE_SIZE, RW, RW))?;
        mapped_bytes(context, &mm, SOURCE, &[0x51]);
        context.expect_eq(mm.copyout(SHARED_ALIAS, &[0x93]).copied, 1);
        let anonymous = mm
            .mapping(SHARED_ALIAS)
            .ok_or(TestError("W33 anonymous page absent"))?
            .physical;
        let physical = clean.phys_addr();
        let (record, _) = checked(context, volume.inode_record(inode.number(), None))?;
        let metadata_physical = record.folio().phys_addr();
        let disk = volume.disk().clone();
        disk.test_flush_completions();

        // Step 1: 验证同一任务内 NoFs/NoIo 作用域的嵌套、析构恢复与跨 CPU 迁移保持特性，以及新派生任务的 Normal 默认模式。
        let id = task::current_task().id();
        context.expect_eq(ReclaimMode::current(), ReclaimMode::Normal);
        let outer = ReclaimScope::nofs();
        context.expect_eq(ReclaimMode::current(), ReclaimMode::NoFs);
        {
            let inner = ReclaimScope::nofs();
            context.expect_eq(ReclaimMode::current(), ReclaimMode::NoFs);
            drop(inner);
            context.expect_eq(ReclaimMode::current(), ReclaimMode::NoFs);
            let noio = ReclaimScope::noio();
            context.expect_eq(ReclaimMode::current(), ReclaimMode::NoIo);
            for target in 0..cpu::count() {
                sched::test_migrate_to(target);
                context.expect_eq(cpu::current().id, target);
                context.expect_eq(task::current_task().id(), id);
                context.expect_eq(ReclaimMode::current(), ReclaimMode::NoIo);
            }
            drop(noio);
            context.expect_eq(ReclaimMode::current(), ReclaimMode::NoFs);
        }

        let fresh = fixture.consume_sync(1, || {
            if ReclaimMode::current() == ReclaimMode::Normal {
                Ok(())
            } else {
                Err(FsError::InvalidOperation)
            }
        })?;
        until("W33 new Task did not leave its original stack", || {
            fixture.consumers[1].as_ref().unwrap().try_join().is_some()
        })?;
        context.expect_eq(fixture.consumers[1].take().unwrap().join(), 0);
        context.expect_eq(fresh.result(), Some(Ok(())));
        context.expect_eq(ReclaimMode::current(), ReclaimMode::NoFs);

        // Step 2: 构造同时存在脏页（Dirty）和在途回写页（Writeback）的场景，验证持有 Inode 失效锁与块映射读锁时的受限回收不会触碰它们或扫描反向映射。
        let writeback_lock = inode.mapping().lock_writeback();
        context.expect_eq(
            checked(context, file.write_at(PAGE_SIZE as u64, &[0x61; PAGE_SIZE]))?,
            PAGE_SIZE,
        );
        context.expect_eq(
            checked(context, io_file.write_at(0, &[0x62; PAGE_SIZE]))?,
            PAGE_SIZE,
        );
        let ranges = checked(context, prepare_unlinked_original_writeback(&io_inode, &wb))?;
        let syncing_volume = volume.clone();
        disk.test_suspend_completions();
        let io = fixture.consume_sync(0, move || {
            let mut ranges = ranges;
            let mut bios = [const { None }; 8];
            let mut completions = [const { None }; 8];
            syncing_volume
                .claim()
                .writeback_batch(
                    &mut ranges,
                    None,
                    &mut bios,
                    &mut completions,
                    syncing_volume.writeback_domain(),
                    || false,
                )
                .map_err(FsError::from)
        })?;
        until("W33 original WB did not enter actual blocked IO", || {
            wb.writeback_sectors() != 0 && disk.test_snapshot().inflight != 0 && io.blocked()
        })?;
        let dirty_mask = dirty.dirty_sectors();
        let wb_mask = wb.writeback_sectors();
        context.expect_eq(dirty_mask != 0, true);
        context.expect_eq(wb_mask != 0, true);
        let before = disk.test_snapshot();

        {
            let invalidate = checked(context, inode.lock_invalidate())?;
            let limited = inode.with_map_read(|_, _| {
                context.expect_eq(ReclaimMode::current(), ReclaimMode::NoFs);
                cache_only_attempt(context);
                let noio = ReclaimScope::noio();
                context.expect_eq(ReclaimMode::current(), ReclaimMode::NoIo);
                cache_only_attempt(context);
                drop(noio);
                Ok(())
            });
            checked(context, limited)?;
            drop(invalidate);
        }
        context.expect_eq(dirty.dirty_sectors(), dirty_mask);
        context.expect_eq(wb.writeback_sectors(), wb_mask);
        context.expect_eq(
            mm.mapping(SOURCE).map(|mapping| mapping.physical),
            Some(physical),
        );
        context.expect_eq(
            mm.mapping(SHARED_ALIAS).map(|mapping| mapping.physical),
            Some(anonymous),
        );
        context.expect_eq(record.folio().phys_addr(), metadata_physical);
        context.expect_eq(disk.test_snapshot().submitted, before.submitted);
        context.expect_eq(disk.test_snapshot().completed, before.completed);

        // Step 3: 先用同一个回收请求耗尽所有未映射的可回收缓存页，随后在物理页与堆双耗尽状态下验证受限作用域内的文件页准备操作平稳返回 NoMemory。
        let mut attempt = ReclaimAttempt::new();
        for _ in 0..64 {
            let old = attempt.stats().scanned;
            attempt.reclaim();
            if attempt.stats().scanned == old {
                break;
            }
        }
        sched::test_migrate_to(0);
        reclaim_cost(context, "scope-pressure-ready");
        let evidence = {
            let _pin = PreemptGuard::new();
            fixture.user.pressure().exhaust()
        };
        let no_memory = (|| -> Result<_, FsError> {
            let _invalidate = inode.lock_invalidate()?;
            inode.with_map_read(|_, _| {
                let failed = inode
                    .mapping()
                    .prepare_file(&inode, 4096, &mut attempt)
                    .err();
                Ok((failed, attempt.stats(), ReclaimMode::current()))
            })
        })();

        fixture.user.release_pressure();
        let (error, stats, mode) = checked(context, no_memory)?;
        context.expect_eq(evidence, (true, true));
        context.expect_eq(error, Some(FsError::NoMemory));
        context.expect_eq(mode, ReclaimMode::NoFs);
        context.expect_eq(stats.scanned <= 64 && stats.isolated <= 16, true);
        context.expect_eq(stats.rmap_checks, 0);
        context.expect_eq(stats.submitted_bytes, 0);
        context.expect_eq(stats.completed_bytes, 0);
        context.expect_eq(dirty.dirty_sectors(), dirty_mask);
        context.expect_eq(wb.writeback_sectors(), wb_mask);

        drop(writeback_lock);
        drop(outer);
        context.expect_eq(ReclaimMode::current(), ReclaimMode::Normal);
        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(io.result(), Some(Ok(())));

        // Step 4: 退出受限作用域并释放 clean 页的外部引用，验证在 Normal 模式下能够通过扫描 rmap 卸载页表映射并回收该文件页。
        drop(clean);
        let mut freed = 0;
        for _ in 0..4 {
            let mut normal = ReclaimAttempt::new();
            normal.reclaim();
            let result = normal.stats();
            freed += result.freed_file;
            context.expect_eq(
                result.scanned <= 64 && result.isolated <= 16 && result.rmap_checks <= 64,
                true,
            );
            context.diagnostic(format_args!("W33 normal stats={:?}", result));
            if inode.mapping().cached_folio(0).is_none() {
                break;
            }
        }
        context.expect_eq(inode.mapping().cached_folio(0).is_none(), true);
        context.expect_eq(mm.mapping(SOURCE).is_none(), true);
        context.expect_eq(0 < freed, true);
        context.expect_eq(
            mm.mapping(SHARED_ALIAS).map(|mapping| mapping.physical),
            Some(anonymous),
        );
        mapped_bytes(context, &mm, SHARED_ALIAS, &[0x93]);
        context.expect_eq(record.folio().phys_addr(), metadata_physical);
        context.diagnostic(format_args!("W33 scope task={} online-cpus={} nested-NOFS-NOFS-NOIO-restored=1 new-Task-normal=1 held-invalidate-root-cache-only=1 dirty-mask={} WB-mask={} active-metadata-physical={:#x} anonymous-physical={:#x} finite-NoMemory={:?} page-null={} heap-null={} normal-freed-file={}",
            id.as_usize(), cpu::count(), dirty_mask, wb_mask, metadata_physical, anonymous, error, evidence.0, evidence.1, freed));
        reclaim_cost(context, "scope-ended");
        Ok(())
    }

    /// 测试目标：验证父子进程通过 `fork` 共享的干净文件映射页在被回收器卸载页表（含跨核 TLB 同步）并释放后，再次访问能触发缺页（Refault）从磁盘重新加载，且父子进程共享的写时复制（COW）私有匿名页不受文件页回收影响。
    /// 场景步骤：
    /// 1. 写入并同步 8 页文件数据，启动用户态测试程序完成文件映射、私有映射写时复制与 `fork`，校验父子进程共享同一个文件物理页及同一个私有 COW 匿名物理页。
    /// 2. 安装页表卸载观测钩子（`ktest_set_unmap_hook`），执行 `ReclaimAttempt::reclaim()` 直到文件缓存页及父子进程的页表叶子项全部被卸载回收，验证卸载钩子在跨核同步前后准确观测到 2 个文件页表项被移除，而私有 COW 页保持原物理页不变。
    /// 3. 放行父子进程继续访问已卸载的文件映射虚拟地址触发缺页重读，断言页面缓存重新加载该文件页并重新建立父子页表映射，且私有 COW 页内容完好无损。
    #[test]
    #[ktest(tags("W32", "fs-reclaim", "fs-shared", "user-fs"), min_cpus = 2)]
    fn clean_shared_leaves_refault_and_private_cow_survives(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        const PARENT_FILE: usize = 0x4b00_0000;
        const PRIVATE: usize = 0x4c00_0000;
        const CHILD_FILE: usize = 0x4d00_0000;

        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let input = [0x62; PAGE_SIZE];
        for page in 0..8 {
            context.expect_eq(
                checked(context, file.write_at((page * PAGE_SIZE) as u64, &input))?,
                PAGE_SIZE,
            );
        }
        checked(context, file.sync_range(0..u64::MAX, false))?;

        let image = fixture.shared_image(context, b"R")?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        let child = fixture.child(&mm)?;
        let child_mm = child
            .mm()
            .ok_or(TestError("reclaim fork child has no original MM"))?;
        let parent_file = mm
            .mapping(PARENT_FILE)
            .ok_or(TestError("reclaim parent did not fault its real file leaf"))?;
        let child_file = child_mm.mapping(CHILD_FILE).ok_or(TestError(
            "reclaim child did not fault its relocated file leaf",
        ))?;
        let parent_private = mm.mapping(PRIVATE).ok_or(TestError(
            "reclaim parent did not create private anonymous COW",
        ))?;
        let child_private = child_mm.mapping(PRIVATE).ok_or(TestError(
            "reclaim fork did not retain private anonymous COW",
        ))?;
        context.expect_eq(parent_file.physical, child_file.physical);
        context.expect_eq(parent_private.physical, child_private.physical);
        context.expect_eq(parent_file.physical != parent_private.physical, true);

        // Step 1: 注册页表卸载钩子，监测文件页回收过程中在跨核 TLB 同步（CSD）前后的卸载页表项数量。
        let controls = TruncateControls::new()?;
        fixture.truncate_controls = Some(controls.clone());
        let observing = controls.clone();
        inode.mapping().ktest_set_unmap_hook(Some(
            Arc::try_new(move |phase, pages, tables| {
                if pages == 0 {
                    return;
                }
                assert_eq!(pages, 2, "file reclaim retired a private anonymous leaf");
                let slot = match phase {
                    FileUnmapPhase::BeforeSync => &observing.before_sync,
                    FileUnmapPhase::AfterSync => &observing.after_sync,
                };
                let mut slot = slot.lock();
                assert!(
                    slot.is_none(),
                    "file reclaim repeated the same resident retirement"
                );
                *slot = Some((pages, tables));
            })
            .map_err(|_| TestError("reclaim CSD observation preparation OOM"))?,
        ));

        // Step 2: 执行多轮 Normal 页回收，直到第 0 页从页面缓存以及父子进程页表中彻底移除，同时确认私有 COW 匿名页未被误回收。
        let gone = || {
            inode.mapping().cached_folio(0).is_none()
                && mm.mapping(PARENT_FILE).is_none()
                && child_mm.mapping(CHILD_FILE).is_none()
        };
        let mut freed_file = 0;
        let mut freed_tables = 0;
        let mut unmapped = 0;
        for round in 0..4 {
            let mut attempt = ReclaimAttempt::new();
            let _ = attempt.reclaim();
            let result = attempt.stats();
            context.expect_eq(result.scanned <= 64, true);
            context.expect_eq(result.isolated <= 16, true);
            context.expect_eq(result.rmap_checks <= 64, true);
            context.expect_eq(result.io_error, false);
            freed_file += result.freed_file;
            freed_tables += result.freed_tables;
            unmapped += result.unmapped;
            context.diagnostic(format_args!(
                "W32 reclaim-round={} result={:?}",
                round, result
            ));
            if gone() {
                break;
            }
        }

        inode.mapping().ktest_set_unmap_hook(None);
        context.expect_eq(gone(), true);
        context.expect_eq(0 < freed_file, true);
        context.expect_eq(2 <= unmapped, true);
        let before_sync = *controls.before_sync.lock();
        let after_sync = *controls.after_sync.lock();
        context.expect_eq(before_sync.map(|counts| counts.0), Some(2));
        context.expect_eq(after_sync, before_sync);
        context.expect_eq(
            mm.mapping(PRIVATE).map(|mapping| mapping.physical),
            Some(parent_private.physical),
        );
        context.expect_eq(
            child_mm.mapping(PRIVATE).map(|mapping| mapping.physical),
            Some(child_private.physical),
        );

        // Step 3: 唤醒用户态父子进程重新读取文件映射区域触发缺页（Refault），验证文件页从磁盘重新载入且父子进程重新映射至新物理页。
        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;
        fixture.phase(&mm, 8, 2)?;
        let refaulted = inode.mapping().cached_folio(0).ok_or(TestError(
            "ordinary refault did not load the canonical file page",
        ))?;
        context.expect_eq(
            mm.mapping(PARENT_FILE).map(|mapping| mapping.physical),
            Some(refaulted.phys_addr()),
        );
        context.expect_eq(
            child_mm.mapping(CHILD_FILE).map(|mapping| mapping.physical),
            Some(refaulted.phys_addr()),
        );
        context.expect_eq(
            mm.mapping(PRIVATE).map(|mapping| mapping.physical),
            Some(parent_private.physical),
        );
        context.expect_eq(
            child_mm.mapping(PRIVATE).map(|mapping| mapping.physical),
            Some(child_private.physical),
        );

        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        context.diagnostic(format_args!(
            "W32 original-file-physical={:#x} refault-file-physical={:#x} private-physical={:#x} retired-before={:?} retired-after={:?} freed-file={} freed-tables={} unmapped={} ordinary-fork-wait=0",
            parent_file.physical,
            refaulted.phys_addr(),
            parent_private.physical,
            before_sync,
            after_sync,
            freed_file,
            freed_tables,
            unmapped,
        ));
        Ok(())
    }
}
