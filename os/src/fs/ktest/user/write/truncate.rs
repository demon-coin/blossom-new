//! 截断失效、尾零、原回写等待与 canonical inode 最后使用退休。

use super::*;

/// 三类截断停点共用的观察回调拥有，原 inode 和阶段仍由调用者借入。
type TruncateObserver = Arc<dyn Fn(&InodeRef, TruncatePhase) + Send + Sync>;

/// 有效前缀和新尾由独立图样决定；普通读取、原共享 VMA 与冷 FS 读取采用相同逐页字节断言。
fn truncate_contents(
    context: &mut TestContext<'_>,
    file: &FileRef,
    mapping: Option<&Mm>,
    prefix: usize,
    size: usize,
) -> TestResult {
    for offset in (0..size).step_by(PAGE_SIZE) {
        let length = PAGE_SIZE.min(size - offset);
        let mut expected = [0u8; PAGE_SIZE];
        let valid = prefix.saturating_sub(offset).min(length);
        expected[..valid].fill(0x65);
        bytes_at(context, file, offset as u64, &expected[..length])?;
        if let Some(mm) = mapping {
            mapped_bytes(context, mm, SOURCE + offset, &expected[..length]);
        }
    }
    Ok(())
}

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 同块、跨块和精确边界缩小后增长，原共享叶重新受 mkwrite 约束；正式同步并结束旧 FS 后重新读设备。
    #[test]
    #[case("same_block_9_3_9", 0u8)]
    #[case("cross_block_partial", 1u8)]
    #[case("block_boundary", 2u8)]
    #[case("page_boundary", 3u8)]
    #[ktest(tags("W22", "fs-truncate", "fs-write-cold", "fs-shared", "fs-sync"))]
    fn shrink_grow_zero_tail_live_mapping_and_cold_reload(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        boundary: u8,
    ) -> TestResult {
        let file = fixture.cold_open(context)?.0;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        let disk = fixture.user.path.owner.disk()?.clone();
        if !disk.limits().flush {
            return Err(TestError("W22 cold reload requires real FLUSH"));
        }
        let block = volume.geometry().block_size;
        let (size, prefix) = match boundary {
            0 => (9, 3),
            1 => (block + 9, block - 3),
            2 => (2 * block, block),
            3 => (2 * PAGE_SIZE + 9, PAGE_SIZE),
            _ => unreachable!(),
        };
        checked(context, file.truncate(0))?;
        for offset in (0..size).step_by(PAGE_SIZE) {
            let length = PAGE_SIZE.min(size - offset);
            context.expect_eq(
                checked(
                    context,
                    file.write_at(offset as u64, &[0x65; PAGE_SIZE][..length]),
                )?,
                length,
            );
        }
        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("truncate shared MM owner OOM"))?;
        vm(
            context,
            mm.map_file(
                SOURCE,
                size.div_ceil(PAGE_SIZE) * PAGE_SIZE,
                RW,
                RW,
                file.clone(),
                0,
                false,
            ),
        )?;
        context.expect_eq(mm.copyout(SOURCE + size - 1, &[0xe7]).copied, 1);
        let private = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            private.map_file(SHARED_ALIAS, PAGE_SIZE, RW, RW, file.clone(), 0, true),
        )?;
        context.expect_eq(private.copyout(SHARED_ALIAS, &[0x83]).copied, 1);
        let private_pfn = private.mapping(SHARED_ALIAS).unwrap().physical;
        checked(context, file.seek(SeekFrom::Start((size + 17) as u64)))?;
        checked(context, file.truncate(prefix as u64))?;
        context.expect_eq(checked(context, file.stat())?.size, prefix as u64);
        context.expect_eq(
            checked(context, file.seek(SeekFrom::Current(0)))?,
            (size + 17) as u64,
        );
        truncate_contents(context, &file, Some(&mm), prefix, prefix)?;
        mapped_bytes(context, &private, SHARED_ALIAS, &[0x83]);
        context.expect_eq(private.mapping(SHARED_ALIAS).unwrap().physical, private_pfn);
        let mut eof = [0xa7];
        context.expect_eq(file.read_at(prefix as u64, &mut eof), Ok(0));
        context.expect_eq(eof, [0xa7]);
        // 页内 EOF 的有效页再次暖 W；精确页边界之前的整页写许可无需因增长撤销。
        context.expect_eq(mm.copyout(SOURCE + prefix - 1, &[0x65]).copied, 1);
        let old_tail = SOURCE + (prefix - 1) / PAGE_SIZE * PAGE_SIZE;
        context.expect_eq(mm.mapping(old_tail).unwrap().permissions.write, true);
        let grants = mm.ktest_mkwrite_count();
        checked(context, file.truncate(size as u64))?;
        if prefix % PAGE_SIZE != 0 {
            context.expect_eq(
                mm.mapping(old_tail)
                    .is_none_or(|leaf| !leaf.permissions.write),
                true,
            );
        }
        context.expect_eq(checked(context, file.stat())?.size, size as u64);
        context.expect_eq(
            checked(context, file.seek(SeekFrom::Current(0)))?,
            (size + 17) as u64,
        );
        truncate_contents(context, &file, Some(&mm), prefix, size)?;
        context.expect_eq(mm.copyout(SOURCE + size - 1, &[0x7e]).copied, 1);
        context.expect_eq(grants < mm.ktest_mkwrite_count(), true);
        context.expect_eq(
            matches!(
                checked(context, inode.map_block((size - 1) as u64 / block as u64))?,
                BlockMapping::Mapped { .. }
            ),
            true,
        );
        bytes_at(context, &file, (size - 1) as u64, &[0x7e])?;
        context.expect_eq(mm.copyout(SOURCE + size - 1, &[0]).copied, 1);
        truncate_contents(context, &file, Some(&mm), prefix, size)?;
        drop(private);
        let before_sync = disk.test_snapshot();
        checked(context, file.sync_range(0..u64::MAX, false))?;
        context.expect_eq(
            before_sync.data_submitted < disk.test_snapshot().data_submitted,
            true,
        );
        context.expect_eq(
            disk.test_snapshot().flush_submitted - before_sync.flush_submitted,
            1,
        );
        let stat = checked(context, file.stat())?;
        let root = checked(context, inode.with_map_read(|_, root| Ok(*root)))?;
        let mapped = tree(context, &inode)?;
        let (reopened, reads_before) = fixture.cold_reopen(context, mm, file, inode, volume)?;
        let reloaded_inode = fixture.inodes.last().unwrap().clone();
        let mut reloaded_stat = checked(context, reopened.stat())?;
        context.expect_eq(reloaded_stat.device != stat.device, true);
        reloaded_stat.device = stat.device;
        context.expect_eq(reloaded_stat, stat);
        context.expect_eq(
            checked(context, reloaded_inode.with_map_read(|_, root| Ok(*root)))?,
            root,
        );
        let reloaded = tree(context, &reloaded_inode)?;
        context.expect_eq(
            (reloaded.data, reloaded.nodes, &reloaded.blocks),
            (mapped.data, mapped.nodes, &mapped.blocks),
        );
        truncate_contents(context, &reopened, None, prefix, size)?;
        context.expect_eq(reads_before < disk.test_snapshot().submitted, true);
        context.diagnostic(format_args!("W22 boundary={} block-size={} size={}->{}->{} normal-read=1 original-shared-mmap=1 private-valid-prefix-preserved=1 partial-EOF-W-revoked={} new-range-mkwrite-and-block-prepared=1 production-sync-FLUSH=1 old-FS-owners-ended=1 fresh-FS-real-device-read=1 tail-zero=1", boundary, block, size, prefix, size, prefix % PAGE_SIZE != 0));
        Ok(())
    }

    /// 普通用户直接消费截断 ABI、准入 errno 和 OFD 位置；权限修改仅作用于本例正常工作文件。
    #[test]
    #[case("read_write", 0u8)]
    #[case("read_only_mount", 1u8)]
    #[case("write_permission", 2u8)]
    #[ktest(tags("W22", "fs-truncate", "user-fs"))]
    fn ordinary_truncate_open_errors_size_and_cursor(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        admission: u8,
    ) -> TestResult {
        if admission == 1 {
            fixture
                .inodes
                .try_reserve_exact(16)
                .map_err(|_| TestError("readonly truncate inode slots OOM"))?;
            fixture.user.prepare(context)?;
            let file = fixture.open(context, b"/test/write-map", 0)?;
            let before = checked(context, file.stat())?;
            let mut prefix = [0; 32];
            context.expect_eq(file.read_at(0, &mut prefix), Ok(prefix.len()));
            fixture.run(context, b"v")?;
            context.expect_eq(checked(context, file.stat())?, before);
            bytes_at(context, &file, 0, &prefix)?;
        } else {
            fixture.prepare(context)?;
            let file = fixture.open(context, b"/test/write-reserve", 2)?;
            if admission == 0 {
                context.expect_eq(file.truncate(u64::MAX), Err(FsError::FileTooLarge));
                fixture.run(context, b"t")?;
                context.expect_eq(checked(context, file.stat())?.size, 0);
            } else {
                assert_eq!(admission, 2);
                checked(context, file.truncate(0))?;
                context.expect_eq(file.write_at(0, b"123456789"), Ok(9));
                let inode = fixture.inodes.last().unwrap().clone();
                let original = checked(context, inode.metadata_snapshot())?.mode;
                checked(
                    context,
                    inode.update_metadata(
                        checked(context, inode.disk_volume())?.claim(),
                        |metadata| {
                            metadata.mode = (metadata.mode & !0o777) | 0o444;
                        },
                    ),
                )?;
                let event = fixture.user.run_named_as(
                    context,
                    PROGRAM_WRITE,
                    b"h",
                    false,
                    0,
                    Credentials {
                        uid: 1001,
                        gid: 1001,
                    },
                );
                checked(
                    context,
                    inode.update_metadata(
                        checked(context, inode.disk_volume())?.claim(),
                        |metadata| {
                            metadata.mode = original;
                        },
                    ),
                )?;
                context.expect_eq(event?.status, 0);
                bytes_at(context, &file, 0, b"123456789")?;
                context.expect_eq(checked(context, file.stat())?.size, 9);
            }
        }
        context.diagnostic(format_args!(
            "truncate-user-ABI admission={} real-user-syscalls=1 errno-and-size-checked-in-ELF=1",
            admission
        ));
        Ok(())
    }

    /// 真实 private 候选在首失效后完成 store 与 fork；另一个参数在真实 fork 快照后失效，禁止提交过期子叶。
    /// 原文件页在失效前由显式有限请求准备，并保活到两次失效完成。
    #[test]
    #[case("late_private_cow", false)]
    #[case("fork_snapshot", true)]
    #[ktest(tags("W20", "fs-truncate", "fs-shared", "user-fs"), min_cpus = 2)]
    fn first_unmap_private_cow_fork_and_second_invalidation(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        fork_snapshot: bool,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        checked(context, file.truncate(0))?;
        context.expect_eq(file.write_at(0, &[0x65; 8192]), Ok(8192));
        checked(context, file.sync_range(0..u64::MAX, false))?;
        let inode = fixture.inodes.last().unwrap().clone();
        let mut attempt = ReclaimAttempt::new();
        let original = checked(
            context,
            file.mapping_inode().unwrap().file_page(1, &mut attempt),
        )?;
        let image = fixture.shared_image(context, b"k")?;
        let mm = image.mm.clone();
        let file_offset = (PAGE_SIZE as u64).to_le_bytes();
        if mm
            .copyout(SHARED_CONTROL + SHARED_FILE_OFFSET, &file_offset)
            .copied
            != file_offset.len()
        {
            return Err(TestError("truncate file offset initialization failed"));
        }
        let controls = TruncateControls::new()?;
        fixture.truncate_controls = Some(controls.clone());
        let stages = controls.clone();
        let hook: TruncateObserver = Arc::try_new(move |_: &InodeRef, phase| {
            let (value, gate) = match phase {
                TruncatePhase::AfterFirstUnmap => (1, Some(0)),
                TruncatePhase::BeforeSecondUnmap => (2, Some(1)),
                TruncatePhase::BeforeBlockRelease => (3, None),
                TruncatePhase::AfterStorageRelease => return,
            };
            stages.phase.store(value, Ordering::Release);
            if let Some(gate) = gate {
                if !stages.cancelled.load(Ordering::Acquire) {
                    stages.gates[gate].wait();
                }
            }
        })
        .map_err(|_| TestError("truncate phase hook OOM"))?;
        inode.ktest_set_truncate_hook(Some(hook));
        if !fork_snapshot {
            let retirement = controls.clone();
            let hook: Arc<dyn Fn(FileUnmapPhase, usize, usize) + Send + Sync> =
                Arc::try_new(move |phase, pages, tables| {
                    if pages == 0 {
                        return;
                    }
                    match phase {
                        FileUnmapPhase::BeforeSync => {
                            *retirement.before_sync.lock() = Some((pages, tables));
                            retirement.unmap_phase.store(1, Ordering::Release);
                            if !retirement.cancelled.load(Ordering::Acquire) {
                                retirement.gates[2].wait();
                            }
                        }
                        FileUnmapPhase::AfterSync => {
                            *retirement.after_sync.lock() = Some((pages, tables));
                            retirement.unmap_phase.store(2, Ordering::Release);
                        }
                    }
                })
                .map_err(|_| TestError("unmap retirement hook OOM"))?;
            inode.mapping().ktest_set_unmap_hook(Some(hook));
        }
        let ready = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("truncate fault readiness OOM"))?;
        let gate =
            Arc::try_new(Completion::new()).map_err(|_| TestError("truncate fault gate OOM"))?;
        fixture.prepare_gate = Some(gate.clone());
        let entered = ready.clone();
        let pause = gate.clone();
        let hook: Arc<dyn Fn(usize) + Send + Sync> = Arc::try_new(move |address| {
            if address == SOURCE && !entered.swap(true, Ordering::AcqRel) {
                pause.wait();
            }
        })
        .map_err(|_| TestError("truncate fault or fork hook OOM"))?;
        if fork_snapshot {
            mm.ktest_set_fork_hook(Some(hook));
        } else {
            mm.ktest_set_fault_hook(Some(hook));
        }
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        WriteOwner::release(&mm, 1)?;
        if fork_snapshot {
            fixture.phase(&mm, 0, 2)?;
            context.expect_eq(
                mm.mapping(SOURCE).unwrap().physical != original.phys_addr(),
                true,
            );
            WriteOwner::release(&mm, 2)?;
        }
        until("W20 original private fault or fork did not pause", || {
            ready.load(Ordering::Acquire)
        })?;
        let user = fixture
            .user
            .graph()
            .lookup_thread(fixture.user.running.as_ref().unwrap().pid)
            .ok_or(TestError("W20 private user binding missing"))?;
        let executor = user
            .executor()
            .ok_or(TestError("W20 private executor missing"))?;
        until("W20 private preparer did not really block", || {
            sched::test_state(&executor) == State::Blocked
        })?;
        let truncating = file.clone();
        let observed = fixture.consume_sync(0, move || truncating.truncate(PAGE_SIZE as u64))?;
        until("W20 truncate did not finish first unmap", || {
            controls.phase.load(Ordering::Acquire) == 1
        })?;
        context.expect_eq(checked(context, file.stat())?.size, PAGE_SIZE as u64);
        context.expect_eq(mm.mapping(SOURCE), None);
        gate.complete();
        mm.ktest_set_fault_hook(None);
        mm.ktest_set_fork_hook(None);
        fixture.prepare_gate.take();
        if !fork_snapshot {
            fixture.phase(&mm, 0, 2)?;
            let late = mm
                .mapping(SOURCE)
                .ok_or(TestError("W20 real late COW leaf missing"))?;
            context.expect_eq(late.physical != original.phys_addr(), true);
            context.expect_eq(late.permissions.write, true);
            mapped_bytes(context, &mm, SOURCE, &[0xb6]);
            WriteOwner::release(&mm, 2)?;
        }
        fixture.phase(&mm, 0, 3)?;
        fixture.phase(&mm, 8, 3)?;
        let child = fixture.child(&mm)?;
        let child_mm = child.mm().ok_or(TestError("W20 fork child MM missing"))?;
        if fork_snapshot {
            context.expect_eq(mm.mapping(SOURCE), None);
            context.expect_eq(child_mm.mapping(SOURCE), None);
        } else {
            let parent = mm
                .mapping(SOURCE)
                .ok_or(TestError("W20 parent late COW missing"))?;
            let inherited = child_mm
                .mapping(SOURCE)
                .ok_or(TestError("W20 child late COW missing"))?;
            context.expect_eq(parent.physical, inherited.physical);
            context.expect_eq(parent.physical != original.phys_addr(), true);
            context.expect_eq(
                (parent.permissions.write, inherited.permissions.write),
                (false, false),
            );
        }
        controls.gates[0].complete();
        until("W20 cache phase did not reach second unmap", || {
            controls.phase.load(Ordering::Acquire) == 2
        })?;
        context.expect_eq(inode.mapping().is_canonical(1, &original), false);
        if !fork_snapshot {
            context.expect_eq(mm.mapping(SOURCE).is_some(), true);
            context.expect_eq(child_mm.mapping(SOURCE).is_some(), true);
        }
        controls.gates[1].complete();
        if !fork_snapshot {
            until(
                "W20 second sweep did not retain real owners before CSD",
                || controls.unmap_phase.load(Ordering::Acquire) == 1,
            )?;
            context.expect_eq(mm.mapping(SOURCE), None);
            context.expect_eq(child_mm.mapping(SOURCE), None);
            let before = controls
                .before_sync
                .lock()
                .ok_or(TestError("W20 real retirement observation missing"))?;
            context.expect_eq(before.0, 2);
            context.expect_eq(2 <= before.1, true);
            controls.gates[2].complete();
            until(
                "W20 CSD did not finish with original retirement owners",
                || controls.unmap_phase.load(Ordering::Acquire) == 2,
            )?;
            context.expect_eq(*controls.after_sync.lock(), Some(before));
        }
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        context.expect_eq(controls.phase.load(Ordering::Acquire), 3);
        context.expect_eq(mm.mapping(SOURCE), None);
        context.expect_eq(child_mm.mapping(SOURCE), None);
        context.expect_eq(mm.fault(SOURCE, UserAccess::Read), Err(VmError::BeyondEof));
        context.expect_eq(
            child_mm.fault(SOURCE, UserAccess::Read),
            Err(VmError::BeyondEof),
        );
        bytes_at(context, &file, 0, &[0x65; PAGE_SIZE])?;
        inode.ktest_set_truncate_hook(None);
        inode.mapping().ktest_set_unmap_hook(None);
        fixture.truncate_controls.take();
        WriteOwner::release(&mm, 3)?;
        fixture.finish_shared(context)?;
        context.diagnostic(format_args!("W20 fork-snapshot={} original-private-fault-or-fork-paused=1 first-unmap-before-resume=1 real-store-and-clone=1 second-range-invalidation=1 stale-child-snapshot-rejected={} real-retired-pages-and-tables-held-through-CSD={} no-synthetic-leaf=1", fork_snapshot, fork_snapshot, !fork_snapshot));
        drop((child_mm, child, user, mm, original));
        Ok(())
    }

    /// 普通 ftruncate 等原数据或真实树节点 WB；旧位图仍 used，真实全尾结束后旧 data 和 metadata 块才复用为新数据。
    /// 旧树全部原页共享准备预算，块复用后的新页检查使用独立有限请求。
    #[test]
    #[case("data", false)]
    #[case("metadata_node", true)]
    #[ktest(tags("W21", "fs-truncate", "fs-sync", "user-fs"), min_cpus = 2)]
    fn old_writeback_waits_before_bitmap_release_and_real_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        metadata: bool,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.volume();
        let disk = fixture.user.path.owner.disk()?.clone();
        if !disk.limits().flush {
            return Err(TestError("W21 requires production FLUSH"));
        }
        checked(context, file.truncate(0))?;
        let block = volume.geometry().block_size;
        let traditional = checked(context, inode.metadata_snapshot())?.flags & codec::EXTENTS == 0;
        let logical = if traditional {
            [0, 12, 268, 65804, 65806]
        } else {
            [0, 2, 4, 6, 8]
        };
        let mut data_blocks = [0; 5];
        let mut old_folios = Vec::new();
        old_folios
            .try_reserve_exact(5)
            .map_err(|_| TestError("old WB folio slots OOM"))?;
        let mut prepare_attempt = ReclaimAttempt::new();
        for (index, logical) in logical.iter().copied().enumerate() {
            let offset = logical * block as u64;
            context.expect_eq(
                file.write_at(offset, &[0x3a; PAGE_SIZE][..block]),
                Ok(block),
            );
            let BlockMapping::Mapped { physical, .. } = checked(context, inode.map_block(logical))?
            else {
                return Err(TestError("W21 producer data not mapped"));
            };
            data_blocks[index] = physical;
            old_folios.push(checked(
                context,
                file.mapping_inode()
                    .unwrap()
                    .file_page(offset / PAGE_SIZE as u64, &mut prepare_attempt),
            )?);
        }
        let old = tree(context, &inode)?;
        context.expect_eq(old.data, 5);
        context.expect_eq(0 < old.nodes, true);
        let node = old
            .blocks
            .iter()
            .copied()
            .find(|physical| !data_blocks.contains(physical))
            .ok_or(TestError("W21 actual old tree node missing"))?;
        let node_buffer = checked(context, volume.metadata_block(node))?;
        let node_folio = node_buffer.folio().clone();
        checked(context, file.sync_range(0..u64::MAX, false))?;
        let image = fixture.shared_image(context, b"j")?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        let user = fixture
            .user
            .graph()
            .lookup_thread(fixture.user.running.as_ref().unwrap().pid)
            .ok_or(TestError("W21 truncating user binding missing"))?;
        let executor = user
            .executor()
            .ok_or(TestError("W21 truncating executor missing"))?;
        let first_unmap = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("W21 first unmap observation OOM"))?;
        let reached = first_unmap.clone();
        let hook: TruncateObserver = Arc::try_new(move |_: &InodeRef, phase| {
            if phase == TruncatePhase::AfterFirstUnmap {
                reached.store(true, Ordering::Release);
            }
        })
        .map_err(|_| TestError("W21 truncate observer OOM"))?;
        inode.ktest_set_truncate_hook(Some(hook));
        if metadata {
            // 原节点条目和 CRC 保持合法；update 重新记其真实 Dirty，正式 metadata WB 写出旧树字节。
            checked(
                context,
                node_buffer
                    .update(volume.claim(), |_| ())
                    .map_err(FsError::Io),
            )?;
        } else {
            for logical in logical.iter().copied() {
                context.expect_eq(
                    file.write_at(logical * block as u64, &[0x4b; PAGE_SIZE][..block]),
                    Ok(block),
                );
            }
        }
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        let syncing_inode = inode.clone();
        let syncing_volume = volume.clone();
        let observed = fixture.consume_sync(0, move || {
            if metadata {
                crate::fs::writeback::sync_metadata(&syncing_volume)
            } else {
                crate::fs::writeback::sync_inode(&syncing_inode)
            }
        })?;
        until("W21 real original WB did not accept IO and block", || {
            let snapshot = disk.test_snapshot();
            let submitted = if metadata {
                before.metadata_submitted < snapshot.metadata_submitted
                    && node_folio.writeback_sectors() != 0
            } else {
                before.data_submitted < snapshot.data_submitted
                    && old_folios
                        .iter()
                        .any(|folio| folio.writeback_sectors() != 0)
            };
            submitted && snapshot.inflight != 0 && observed.blocked()
        })?;
        WriteOwner::release(&mm, 1)?;
        until(
            "W21 ordinary ftruncate did not block on original IO",
            || {
                sched::test_state(&executor) == State::Blocked
                    || mm.load_user_u32(SHARED_CONTROL) == Ok(2)
            },
        )?;
        context.expect_eq(sched::test_state(&executor), State::Blocked);
        context.expect_eq(mm.load_user_u32(SHARED_CONTROL), Ok(1));
        if !metadata {
            until(
                "W21 data wait began before new size and first unmap",
                || first_unmap.load(Ordering::Acquire),
            )?;
            context.expect_eq(checked(context, file.stat())?.size, 0);
        }
        for physical in &old.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(true),
            );
        }
        let hint = (inode.number() - 1) / volume.geometry().inodes_per_group;
        let probe = checked(context, volume.allocate_block(hint))?;
        let allocated = probe.physical();
        let excluded = !old.blocks.contains(&allocated);
        volume.release_unpublished_block(probe);
        context.expect_eq(excluded, true);
        context.expect_eq(disk.test_snapshot().inflight != 0, true);
        context.expect_eq(observed.result.lock().is_none(), true);
        context.diagnostic(format_args!("W21 kind={} completion-consumption-paused=1 ordinary-sys46-blocked=1 first-unmap-observed={} old-data-and-node-bitmap-used=1 real-allocation-excludes-old-blocks=1 allocated-probe={} hardware-DMA-inflight-claim=0", if metadata { "metadata-node" } else { "data" }, first_unmap.load(Ordering::Acquire), allocated));
        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        fixture.phase(&mm, 0, 2)?;
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        context.expect_eq(checked(context, file.stat())?.size, 0);
        context.expect_eq(checked(context, inode.metadata_snapshot())?.blocks, 0);
        context.expect_eq(inode.mapping().is_empty(), true);
        for physical in &old.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(false),
            );
        }
        drop((node_buffer, node_folio));
        let replacement = [0xd7; PAGE_SIZE];
        for (logical, physical) in old.blocks.iter().copied().enumerate() {
            context.expect_eq(
                file.write_at((logical * block) as u64, &replacement[..block]),
                Ok(block),
            );
            let BlockMapping::Mapped {
                physical: reused, ..
            } = checked(context, inode.map_block(logical as u64))?
            else {
                return Err(TestError("W21 released block not reused as real file data"));
            };
            context.expect_eq(reused, physical);
            bytes_at(
                context,
                &file,
                (logical * block) as u64,
                &replacement[..block],
            )?;
        }
        let mut reuse_attempt = ReclaimAttempt::new();
        let fresh = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(0, &mut reuse_attempt),
        )?;
        context.expect_eq(
            old_folios
                .iter()
                .all(|old| old.phys_addr() != fresh.phys_addr()),
            true,
        );
        checked(context, file.sync_range(0..u64::MAX, false))?;
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        let mut device_bytes = [0; PAGE_SIZE];
        for physical in &old.blocks {
            checked(
                context,
                volume
                    .claim()
                    .read_exact_at(*physical * block as u64, &mut device_bytes[..block])
                    .map_err(FsError::Io),
            )?;
            context.expect_eq(&device_bytes[..block], &replacement[..block]);
        }
        inode.ktest_set_truncate_hook(None);
        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        context.diagnostic(format_args!("W21 kind={} real-old-IO-and-Work-tails-ended=1 released-data={} released-nodes={} actual-all-old-physical-block-reuse=1 detached-folio-not-reattached=1 production-sync-and-device-readback=1 old-write-does-not-overwrite-new-data=1", if metadata { "metadata-node" } else { "data" }, old.data, old.nodes));
        Ok(())
    }

    /// 合法未链接 inode 的缓存先退出，OFD、VMA、外部原页和真实 IO 分别保活；最后使用交原 Task 回收并返块和 inode 位。
    /// 外部原页由一份显式有限请求取得，其最后释放仍承担原 inode 寿命。
    #[test]
    #[ktest(tags("fs-truncate", "fs-inode-lifetime", "fs-sync"), min_cpus = 2)]
    fn unlinked_last_ofd_vma_folio_and_io_release_canonical_inode(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let (linked, cold_vfs) = fixture.cold_open(context)?;
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        let cache = fixture.cold_cache.as_ref().unwrap().clone();
        let linked_inode = fixture.inodes.last().unwrap().clone();
        context.expect_eq(
            linked
                .path()
                .unwrap()
                .unhashed_inode(linked_inode.clone())
                .err(),
            Some(FsError::InvalidOperation),
        );
        let before = allocation_counts(context, &volume)?;
        let token = checked(context, volume.allocate_inode(0, 0o100600, 0, 0))?;
        let number = token.number();
        let inode = match cache.iget(&volume, number) {
            Ok(inode) => {
                context.expect_eq(token.publish(), number);
                inode
            }
            Err(error) => {
                volume.release_unpublished_inode(token);
                return checked(context, Err(error));
            }
        };
        fixture.inodes.push(inode.clone());
        fixture.eviction = Some(cache.watch());
        let path = checked(
            context,
            linked.path().unwrap().unhashed_inode(inode.clone()),
        )?;
        let file = checked(
            context,
            open_path_with_flags(&cold_vfs, path, 2, FileIoContext::Kernel),
        )?;
        drop(cold_vfs);
        context.expect_eq(checked(context, file.stat())?.links, 0);
        context.expect_eq(file.write_at(0, &[0x91; PAGE_SIZE]), Ok(PAGE_SIZE));
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("unlinked live VMA MM owner OOM"))?;
        vm(
            context,
            mm.map_file(SOURCE, PAGE_SIZE, RW, RW, file.clone(), 0, false),
        )?;
        context.expect_eq(mm.copyout(SOURCE + 1, &[0x72]).copied, 1);
        let mapped = tree(context, &inode)?;
        context.expect_eq(checked(context, file.stat())?.size, PAGE_SIZE as u64);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        let disk = fixture.user.path.owner.disk()?.clone();
        let ranges = checked(context, prepare_unlinked_original_writeback(&inode, &folio))?;
        disk.test_suspend_completions();
        let io_volume = volume.clone();
        let observed = fixture.consume_sync(0, move || {
            let mut ranges = ranges;
            let mut bios = [const { None }; 8];
            let mut completions = [const { None }; 8];
            io_volume
                .claim()
                .writeback_batch(
                    &mut ranges,
                    None,
                    &mut bios,
                    &mut completions,
                    io_volume.writeback_domain(),
                    || false,
                )
                .map_err(FsError::from)
        })?;
        until(
            "unlinked inode original data WB did not really block",
            || {
                folio.writeback_sectors() != 0
                    && disk.test_snapshot().inflight != 0
                    && observed.blocked()
            },
        )?;
        fixture.inodes.clear();
        drop(inode);
        drop(fixture.cold_cache.take());
        drop(cache);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        context.expect_eq(checked(context, file.stat())?.size, PAGE_SIZE as u64);
        drop(file);
        mapped_bytes(context, &mm, SOURCE, &[0x91, 0x72]);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        drop(mm);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        context.expect_eq(folio.writeback_sectors() != 0, true);
        drop(folio);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        for physical in &mapped.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(true),
            );
        }
        context.diagnostic(format_args!("inode-last-use number={} legitimate-published-nlink0=1 unhashed-real-OFD-and-VMA=1 cache-owner-ended=1 OFD-then-VMA-then-folio-ended=1 original-IO-credential-only-owns-inode=1 bitmap-and-size-preserved=1", number));
        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        disk.test_flush_completions();
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), false);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            false,
        );
        for physical in &mapped.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(false),
            );
        }
        drop((linked, linked_inode));
        checked(context, fixture.eviction.as_ref().unwrap().wait_retired())?;
        let retired_counts = allocation_counts(context, &volume)?;
        context.expect_eq(retired_counts, before);
        let reused = checked(context, volume.allocate_inode(0, 0o100600, 123, 234))?;
        let reused_number = reused.number();
        let reloaded = volume.load_inode(reused_number, None);
        volume.release_unpublished_inode(reused);
        context.expect_eq(reused_number, number);
        let reloaded = checked(context, reloaded)?;
        context.expect_eq(
            (
                reloaded.size,
                reloaded.blocks,
                reloaded.links,
                reloaded.uid,
                reloaded.gid,
            ),
            (0, 0, 0, 123, 234),
        );
        checked(context, crate::fs::writeback::sync_metadata(&volume))?;
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        let final_counts = allocation_counts(context, &volume)?;
        context.expect_eq(final_counts, before);
        drop(fixture.eviction.take());
        context.diagnostic(format_args!("inode-last-use number={} original-IO-and-Work-tails-ended=1 canonical-slot-removed=1 task-context-eviction=1 file-blocks-and-inode-bitmap-returned=1 actual-inode-number-reuse=1 production-metadata-sync=1", number));
        Ok(())
    }
    struct InodeReuseControls {
        phase: AtomicUsize,
        cancelled: AtomicBool,
        gate: Completion,
    }

    struct ReusedCanonicalInode {
        number: u64,
        slot_removed_before_iget: bool,
        inode: InodeRef,
    }

    /// 分配成功后先由同一缓存取得 canonical inode，再发表并结束临时 reservation。
    fn allocate_published_inode(
        volume: &Arc<Ext4Volume>,
        cache: &Arc<InodeCache>,
    ) -> Result<ReusedCanonicalInode, FsError> {
        let watch = cache.watch();
        let token = volume.allocate_inode(0, 0o100600, 123, 234)?;
        let number = token.number();
        let slot_removed_before_iget = !watch.contains(number);
        let inode = match cache.iget(volume, number) {
            Ok(inode) => inode,
            Err(error) => {
                volume.release_unpublished_inode(token);
                return Err(error);
            }
        };
        Ok(ReusedCanonicalInode {
            number: token.publish(),
            slot_removed_before_iget,
            inode,
        })
    }

    /// 真实未链接 inode 释放存储后，分配者须等 post-delete metadata WB 和原 canonical 槽退场。
    #[test]
    #[case("running", false)]
    #[case("failed", true)]
    #[ktest(tags("fs-truncate", "fs-inode-lifetime", "fs-sync"), min_cpus = 2)]
    fn unlinked_inode_reuse_waits_for_post_delete_writeback(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        fail_metadata: bool,
    ) -> TestResult {
        let disk = fixture.user.path.owner.disk()?.clone();
        let controls = Arc::try_new(InodeReuseControls {
            phase: AtomicUsize::new(0),
            cancelled: AtomicBool::new(false),
            gate: Completion::new(),
        })
        .map_err(|_| TestError("inode reuse controls OOM"))?;
        let allocation = Arc::try_new(SpinLock::new(None::<Result<ReusedCanonicalInode, FsError>>))
            .map_err(|_| TestError("inode reuse result OOM"))?;
        let body = (|| -> TestResult {
            let (linked, cold_vfs) = fixture.cold_open(context)?;
            let volume = fixture.cold_volume.as_ref().unwrap().clone();
            let cache = fixture.cold_cache.as_ref().unwrap().clone();
            let linked_inode = fixture.inodes.last().unwrap().clone();
            let before = allocation_counts(context, &volume)?;
            let original = checked(context, allocate_published_inode(&volume, &cache))?;
            context.expect_eq(original.slot_removed_before_iget, true);
            let number = original.number;
            let original_metadata = checked(context, original.inode.metadata_snapshot())?;
            context.expect_eq(
                checked(context, inode_allocation_bit(&volume, number))?,
                true,
            );
            let allocated_counts = allocation_counts(context, &volume)?;
            context.expect_eq(allocated_counts, (before.0, before.1 - 1));
            let path = checked(
                context,
                linked
                    .path()
                    .unwrap()
                    .unhashed_inode(original.inode.clone()),
            )?;
            let file = checked(
                context,
                open_path_with_flags(&cold_vfs, path, 2, FileIoContext::Kernel),
            )?;
            drop(cold_vfs);
            context.expect_eq(checked(context, file.stat())?.links, 0);
            let paused = controls.clone();
            let hook: TruncateObserver = Arc::try_new(move |_: &InodeRef, phase| {
                if phase != TruncatePhase::AfterStorageRelease {
                    return;
                }
                paused.phase.store(1, Ordering::Release);
                if !paused.cancelled.load(Ordering::Acquire) {
                    paused.gate.wait();
                }
            })
            .map_err(|_| TestError("inode reuse truncate hook OOM"))?;
            original.inode.ktest_set_truncate_hook(Some(hook));
            fixture.eviction = Some(cache.watch());
            drop((file, linked, linked_inode));
            fixture.inodes.clear();
            drop(original.inode);
            until("inode reuse did not reach real storage release", || {
                controls.phase.load(Ordering::Acquire) == 1
            })?;
            disk.test_flush_completions();
            let released_counts = allocation_counts(context, &volume)?;
            context.expect_eq(released_counts, before);
            let released_bit = checked(context, inode_allocation_bit(&volume, number))?;
            context.expect_eq(released_bit, false);
            context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
            context.expect_eq(disk.test_snapshot().inflight, 0);
            context.expect_eq(disk.test_completion_error_pending(), false);
            let (old_record, record_size) = inode_record_snapshot(context, &volume, number)?;
            let old_generation = checked(context, codec::le32(&old_record[..record_size], 100))?;
            context.expect_eq(old_generation, original_metadata.generation);

            let result = allocation.clone();
            let actor_volume = volume.clone();
            let actor_cache = cache.clone();
            let observed = fixture.consume_sync(0, move || {
                *result.lock() = Some(allocate_published_inode(&actor_volume, &actor_cache));
                Ok(())
            })?;
            until("inode reuse allocator neither blocked nor returned", || {
                observed.blocked() || allocation.lock().is_some()
            })?;
            if !observed.blocked() || allocation.lock().is_some() {
                return Err(TestError("inode reuse allocator missed canonical wait"));
            }
            context.expect_eq(*observed.result.lock(), None);
            context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
            let reserved_bit = checked(context, inode_allocation_bit(&volume, number))?;
            context.expect_eq(reserved_bit, true);
            let reserved_counts = allocation_counts(context, &volume)?;
            context.expect_eq(reserved_counts.0, released_counts.0);
            context.expect_eq(reserved_counts.1 + 1, released_counts.1);
            let (waiting_record, waiting_size) = inode_record_snapshot(context, &volume, number)?;
            context.expect_eq(waiting_size, record_size);
            context.expect_eq(&waiting_record[..record_size], &old_record[..record_size]);
            context.expect_eq(
                codec::le32(&waiting_record[..record_size], 100),
                Ok(old_generation),
            );

            disk.test_suspend_completions();
            let before_writeback = disk.test_snapshot();
            context.expect_eq(before_writeback.inflight, 0);
            if fail_metadata {
                disk.test_fail_next_metadata(BlockError::Io);
            }
            controls.gate.complete();
            until(
                "post-delete metadata WB did not submit a real in-flight request",
                || {
                    let current = disk.test_snapshot();
                    before_writeback.metadata_submitted < current.metadata_submitted
                        && before_writeback.inflight < current.inflight
                },
            )?;
            context.expect_eq(observed.blocked(), true);
            context.expect_eq(allocation.lock().is_none(), true);
            context.expect_eq(disk.test_completion_error_pending(), fail_metadata);
            disk.test_resume_completions();
            let eviction = fixture.eviction.as_ref().unwrap().wait();
            until("inode reuse metadata slot tail did not end", || {
                let current = disk.test_snapshot();
                current.inflight == 0 && before_writeback.completed < current.completed
            })?;
            disk.test_flush_completions();
            let completed = disk.test_snapshot();
            context.expect_eq(completed.inflight, 0);
            context.expect_eq(before_writeback.completed < completed.completed, true);
            context.expect_eq(disk.test_completion_error_pending(), false);

            let reused = if fail_metadata {
                until(
                    "inode reuse allocator did not return the real metadata error",
                    || allocation.lock().is_some(),
                )?;
                let attempt = allocation
                    .lock()
                    .take()
                    .ok_or(TestError("inode reuse allocation result missing"))?;
                let expected = FsError::Io(BlockError::Io);
                let error = match attempt {
                    Err(error) => error,
                    Ok(reused) => {
                        drop(reused);
                        return Err(TestError("failed inode reuse unexpectedly succeeded"));
                    }
                };
                if eviction != Err(expected) || error != expected {
                    context.expect_eq(eviction, Err(expected));
                    context.expect_eq(error, expected);
                    return Err(TestError(
                        "inode reuse failure did not preserve the original error",
                    ));
                }
                context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
                let cancelled_bit = checked(context, inode_allocation_bit(&volume, number))?;
                let cancelled_counts = allocation_counts(context, &volume)?;
                context.expect_eq(cancelled_bit, false);
                context.expect_eq(cancelled_counts, released_counts);
                let (failed_record, failed_size) = inode_record_snapshot(context, &volume, number)?;
                context.expect_eq(failed_size, record_size);
                context.expect_eq(&failed_record[..record_size], &old_record[..record_size]);
                context.expect_eq(
                    codec::le32(&failed_record[..record_size], 100),
                    Ok(old_generation),
                );

                let before_retry = disk.test_snapshot();
                let pruned = cache.prune_unused();
                context.expect_eq(0 < pruned, true);
                disk.test_flush_completions();
                let recovered = disk.test_snapshot();
                context.expect_eq(
                    before_retry.metadata_submitted < recovered.metadata_submitted,
                    true,
                );
                context.expect_eq(before_retry.completed < recovered.completed, true);
                context.expect_eq(recovered.inflight, 0);
                context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), false);
                let recovered_bit = checked(context, inode_allocation_bit(&volume, number))?;
                let recovered_counts = allocation_counts(context, &volume)?;
                context.expect_eq(recovered_bit, false);
                context.expect_eq(recovered_counts, released_counts);
                let (retried_record, retried_size) =
                    inode_record_snapshot(context, &volume, number)?;
                context.expect_eq(retried_size, record_size);
                context.expect_eq(&retried_record[..record_size], &old_record[..record_size]);
                checked(context, allocate_published_inode(&volume, &cache))?
            } else {
                checked(context, eviction)?;
                until(
                    "inode reuse allocator did not resume after slot removal",
                    || allocation.lock().is_some(),
                )?;
                context.expect_eq(*observed.result.lock(), Some(Ok(())));
                let attempt = allocation
                    .lock()
                    .take()
                    .ok_or(TestError("inode reuse allocation result missing"))?;
                checked(context, attempt)?
            };
            context.expect_eq(reused.number, number);
            context.expect_eq(reused.slot_removed_before_iget, true);
            let metadata = checked(context, reused.inode.metadata_snapshot())?;
            context.expect_eq(metadata.number, number);
            context.expect_eq(metadata.generation, old_generation.wrapping_add(1));
            context.expect_eq(
                (metadata.mode, metadata.uid, metadata.gid),
                (0o100600, 123, 234),
            );
            context.expect_eq((metadata.links, metadata.size, metadata.blocks), (0, 0, 0));
            drop(reused.inode);
            checked(context, fixture.eviction.as_ref().unwrap().wait())?;
            disk.test_flush_completions();
            context.expect_eq(disk.test_snapshot().inflight, 0);
            context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), false);
            let final_bit = checked(context, inode_allocation_bit(&volume, number))?;
            let final_counts = allocation_counts(context, &volume)?;
            context.expect_eq(final_bit, false);
            context.expect_eq(final_counts, before);
            drop(fixture.eviction.take());
            context.diagnostic(format_args!(
                "inode-reuse phase={} reserved-before-canonical-wait=1 real-post-delete-metadata-WB=1 failed-slot-retried-explicitly={} generation-and-credentials=1 final-resources-restored=1",
                if fail_metadata { "failed" } else { "running" },
                fail_metadata
            ));
            Ok(())
        })();
        controls.cancelled.store(true, Ordering::Release);
        controls.gate.complete();
        disk.test_resume_completions();
        disk.test_flush_completions();
        let joined = fixture.join_consumers();
        drop(allocation.lock().take());
        if let Err(error) = body {
            joined?;
            return Err(error);
        }
        joined?;
        Ok(())
    }

    /// 新 VMA 发表而必撤叶尚在时，旧文件来源与登记仍须供真实截断找到；CSD 结束后才能丢原页表拥有。
    /// 旧文件与可选新文件分别用独立有限请求准备原页，保留各自的来源身份。
    #[test]
    #[case("file_to_file", 0u8)]
    #[case("file_to_anonymous", 1u8)]
    #[case("munmap", 2u8)]
    #[ktest(tags("fs-truncate", "vm-file-retirement", "fs-shared"), min_cpus = 2)]
    fn mandatory_vma_leaf_retirement_preserves_old_file_source(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        replacement: u8,
    ) -> TestResult {
        fixture.prepare(context)?;
        let old = fixture.open(context, b"/test/write-reserve", 2)?;
        checked(context, old.truncate(0))?;
        context.expect_eq(old.write_at(0, &[0x4f; PAGE_SIZE]), Ok(PAGE_SIZE));
        checked(context, old.sync_range(0..u64::MAX, false))?;
        let old_inode = fixture.inodes.last().unwrap().clone();
        let mut old_attempt = ReclaimAttempt::new();
        let old_folio = checked(
            context,
            old.mapping_inode().unwrap().file_page(0, &mut old_attempt),
        )?;
        let mut expected = [0; 32];
        let (new, new_folio) = if replacement == 0 {
            let file = fixture.open(context, b"/test/write-map", 0)?;
            context.expect_eq(file.read_at(0, &mut expected), Ok(expected.len()));
            let mut new_attempt = ReclaimAttempt::new();
            let folio = checked(
                context,
                file.mapping_inode().unwrap().file_page(0, &mut new_attempt),
            )?;
            (Some(file), Some(folio))
        } else {
            (None, None)
        };
        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("mandatory retirement MM owner OOM"))?;
        vm(
            context,
            mm.map_file(SOURCE, PAGE_SIZE, RO, RO, old.clone(), 0, false),
        )?;
        mapped_bytes(context, &mm, SOURCE, &[0x4f; 32]);
        context.expect_eq(mm.mapping(SOURCE).unwrap().physical, old_folio.phys_addr());
        fixture.shared_mm = Some(mm.clone());
        let ready = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("mandatory retirement ready OOM"))?;
        let gate = Arc::try_new(Completion::new())
            .map_err(|_| TestError("mandatory retirement gate OOM"))?;
        fixture.prepare_gate = Some(gate.clone());
        let entered = ready.clone();
        let pause = gate.clone();
        let hook: Arc<dyn Fn(usize, usize) + Send + Sync> = Arc::try_new(move |start, end| {
            if start == SOURCE && end == SOURCE + PAGE_SIZE && !entered.swap(true, Ordering::AcqRel)
            {
                pause.wait();
            }
        })
        .map_err(|_| TestError("mandatory retirement VMA hook OOM"))?;
        mm.ktest_set_vma_unmap_hook(Some(hook));
        let controls = TruncateControls::new()?;
        fixture.truncate_controls = Some(controls.clone());
        let retirement = controls.clone();
        let hook: Arc<dyn Fn(FileUnmapPhase, usize, usize) + Send + Sync> =
            Arc::try_new(move |phase, pages, tables| {
                if pages == 0 {
                    return;
                }
                match phase {
                    FileUnmapPhase::BeforeSync => {
                        *retirement.before_sync.lock() = Some((pages, tables));
                        retirement.unmap_phase.store(1, Ordering::Release);
                        if !retirement.cancelled.load(Ordering::Acquire) {
                            retirement.gates[2].wait();
                        }
                    }
                    FileUnmapPhase::AfterSync => {
                        *retirement.after_sync.lock() = Some((pages, tables));
                        retirement.unmap_phase.store(2, Ordering::Release);
                    }
                }
            })
            .map_err(|_| TestError("mandatory retirement ownership hook OOM"))?;
        old_inode.mapping().ktest_set_unmap_hook(Some(hook));
        let result = Arc::try_new(SpinLock::new(None::<Result<(), VmError>>))
            .map_err(|_| TestError("mandatory retirement result OOM"))?;
        let recorded = result.clone();
        let replacing = mm.clone();
        let file = new;
        fixture.consumers[0] = Some(
            task::prepare_kernel_on(cpu::current().id, move || {
                let operation = if replacement == 2 {
                    replacing.unmap(SOURCE, PAGE_SIZE)
                } else {
                    let source = if replacement == 0 {
                        let file = file.expect("file replacement has no original OFD");
                        let inode = file
                            .mapping_inode()
                            .expect("regular replacement has no mapping inode")
                            .clone();
                        MmapSource::File {
                            file,
                            inode,
                            offset: 0,
                            private: false,
                        }
                    } else {
                        assert_eq!(replacement, 1);
                        MmapSource::PrivateAnonymous
                    };
                    replacing
                        .mmap(MmapRequest {
                            address: MmapAddress::Fixed(SOURCE),
                            length: PAGE_SIZE,
                            permissions: RO,
                            maximum: RO,
                            source,
                        })
                        .map(|_| ())
                };
                *recorded.lock() = Some(operation);
                0
            })
            .map_err(|_| TestError("mandatory retirement producer prepare failed"))?
            .publish(),
        );
        until(
            "mandatory replacement did not pause with original leaf",
            || ready.load(Ordering::Acquire),
        )?;
        context.expect_eq(mm.mapping(SOURCE).unwrap().physical, old_folio.phys_addr());
        let truncating = old.clone();
        let observed = fixture.consume_sync(1, move || truncating.truncate(0))?;
        until("old file rmap lost mandatory retirement source", || {
            controls.unmap_phase.load(Ordering::Acquire) == 1 || observed.result.lock().is_some()
        })?;
        context.expect_eq(mm.mapping(SOURCE), None);
        let before = controls.before_sync.lock().ok_or(TestError(
            "mandatory old file retirement did not retain original owners",
        ))?;
        context.expect_eq(before.0, 1);
        context.expect_eq(2 <= before.1, true);
        context.expect_eq(old_inode.mapping().is_canonical(0, &old_folio), true);
        controls.gates[2].complete();
        until("mandatory old file retirement CSD did not finish", || {
            controls.unmap_phase.load(Ordering::Acquire) == 2
        })?;
        context.expect_eq(*controls.after_sync.lock(), Some(before));
        until("mandatory old file truncate did not finish", || {
            observed.result.lock().is_some()
        })?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        context.expect_eq(checked(context, old.stat())?.size, 0);
        context.expect_eq(mm.mapping(SOURCE), None);
        gate.complete();
        mm.ktest_set_vma_unmap_hook(None);
        fixture.prepare_gate.take();
        fixture.join_consumers()?;
        context.expect_eq(*result.lock(), Some(Ok(())));
        old_inode.mapping().ktest_set_unmap_hook(None);
        fixture.truncate_controls.take();
        if replacement == 2 {
            context.expect_eq(mm.fault(SOURCE, UserAccess::Read), Err(VmError::Unmapped));
        } else {
            vm(context, mm.fault(SOURCE, UserAccess::Read))?;
            if replacement == 0 {
                context.expect_eq(
                    mm.mapping(SOURCE).unwrap().physical,
                    new_folio.as_ref().unwrap().phys_addr(),
                );
                mapped_bytes(context, &mm, SOURCE, &expected);
            } else {
                mapped_bytes(context, &mm, SOURCE, &[0; 32]);
                context.expect_eq(
                    mm.mapping(SOURCE).unwrap().physical != old_folio.phys_addr(),
                    true,
                );
            }
        }
        fixture.shared_mm.take();
        context.diagnostic(format_args!("mandatory-file-retirement replacement={} new-VMA-published-old-leaf-still-live=1 real-old-file-truncate-finds-source=1 old-leaf-cleared=1 original-page-and-table-owners-held-through-CSD=1 actual-new-fault-or-unmapped=1 native-MM=1 active-user-root-claim=0 fail-before-runtime-claim=0", replacement));
        Ok(())
    }
}
