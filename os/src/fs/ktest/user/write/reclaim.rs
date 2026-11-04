//! 文件页有限回收、真实全压不足、预备存储与 Task 回收作用域。

use super::*;

/// 普通小对象沿原堆预留已存在 slab 的空位，物理页施压时不耗尽堆类别。
struct SmallHeapRoom {
    objects: [[Option<Vec<u8>>; 16]; 11],
}

impl SmallHeapRoom {
    /// 保留每个小对象类别的真实分配，避免页压力准备时收缩其 backing。
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

    /// 交错释放同 slab 内小对象，仍保其余真实分配；随后必须重新观察原页 None。
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

    /// 冷文件 miss 和文件、匿名 fault 在原页 None 后消费可收文件页；无进展返回真实 NoMemory。
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
        // 同页表区域的匿名叶保住目录；有限无进展检查不能把撤 File 叶后的表页归还算成无进展。
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
            // 原 donor 没有诊断 folio pin；缓存退休后由正式入口同源申请目标页，核验其内容与 PFN。
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

    /// 原 Dirty 页有外部 pin 时只获得一次 WB 值进度，实际全堆 None 下由原 Work 提交且申请仍失败。
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
        // WB 互斥只挡住原 Work 的提交；施压和 Normal 回收由另一份预建 Task 执行。
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

    /// 同步先等共同 metadata WB，持该互斥的正常冷 inode 读取仍能取得原 aux 并完成真实 IO。
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
            // 旧错误顺序已经占住 aux；先退出观察段并放 WB，不能在测试控制者中形成同一死锁。
            if !free_aux {
                return Err(TestError(
                    "metadata sync acquired aux before waiting for WB",
                ));
            }
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

    /// 只检查本请求的真实有限预算，受限作用域不得扫描 rmap 或产生回写进度等待。
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

    /// 普通 Read Bio 从本卷准确块范围读回，真实 DMA 全尾后逐字节核验独立输入。
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
            // SAFETY: 唯一清零原页没有外部 CPU 别名，Read 的整个映射移交给同一真实 Bio；
            // 无回调，只有完整完成并领取原 memory 后才借用设备写入的字节。
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
        // SAFETY: 本次完整 Read 成功，completion 已结束设备访问；独占原 memory 保活，
        // 无回调、并发写者或裸别名，准确映射的全部字节已由实际设备初始化。
        context.expect_eq(unsafe { memory[0].bytes() }, expected);
        drop(memory);
        drop(completion);
        Ok(())
    }

    /// 从当前真实根挑选外部节点；不把 inline 根或普通数据块冒充冷 metadata。
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

    /// 记录真实常驻与新建 Task 成本，预备救援者已包含在同一 VM 的 live 基线内。
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

    /// 两种真实外部块树在同源页与全堆耗尽时完成两个窗口原页 WB、冷节点读入及 FLUSH。
    /// 每次整窗原页准备和同步后的身份检查各共用一份有限回收预算。
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
        // 唯一预备窗口在任何 inode invalidate/root/content/WB 依赖之前借入。
        // 后台可等待此 scratch，但本 Task 不等待后台轮，也不依赖其创建新的存储。
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
        // 冷节点必须先完成正式写盘；独立 Buffer 归还之后仅由原 prune 摘离静止 Node。
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
        // 同步接口先消费全部已接受 IO 尾；解除真实压力后才归还 scratch/book 或建立读回存储。
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

    /// 同 Task 嵌套及迁核保持回收限制；真实 FS 依赖下只收缓存，匿名页和原 Dirty/WB 不变。
    /// 原文件的两页准备共用请求预算，另一个 IO 文件保留自己的准备预算。
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
        // 一个共享请求预算先消费所有可收缓存；再用同一 Task 的真实 allocator 填满释放的存储。
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

    /// 普通 fork 的两个文件叶经历真实撤叶和 CSD，refault 重读原盘，私有匿名修改保留。
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
