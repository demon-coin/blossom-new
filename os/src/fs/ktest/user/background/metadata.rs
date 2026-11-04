//! 空闲元数据回写与真实慢完成下的普通 writer 节流行为。

use super::*;
use crate::block::BioAllocator;
use crate::fs::file::WriteBuffer;
use crate::fs::ktest::user::write::{bytes_at, SyncObservation};
use crate::fs::SeekFrom;
use crate::mm::reclaim::ReclaimAttempt;
use crate::workqueue::testing::{metadata_stats, pool_stats};

const PRESSURE_PAGES: usize = 4097;
const PRESSURE_BYTE: u8 = 0x86;

/// 每 CPU 原池累计创建数解释共享 worker 的实际分配，不用 live 差掩盖瞬时 Task。
fn worker_creations() -> usize {
    (0..cpu::count()).map(|id| pool_stats(id).created).sum()
}

/// 同一窗口没有其他测试 actor；manager 真正阻塞后读取原 ID 和原池累计创建事实。
fn creation_snapshot() -> Result<(usize, usize), TestError> {
    until(
        "worker manager did not finish actual Task preparation",
        || (0..cpu::count()).all(crate::workqueue::testing::manager_blocked),
    )?;
    Ok((task::test_created_tasks(), worker_creations()))
}

/// 控制 Task 保持就绪并让出 CPU，观察短于普通采样间隔的真实限速阻塞；期限不随写进度延长。
fn throttled_writer(
    domain: &WritebackDomain,
    writer: &SyncObservation,
    earlier_waits: u64,
) -> Result<WritebackSnapshot, TestError> {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(60))
        .ok_or(TestError("dirty writer observation deadline overflow"))?;
    loop {
        let snapshot = domain.snapshot();
        let (limit, _) = domain.ktest_limits();
        if limit < snapshot.dirty_pages + snapshot.writeback_pages
            && earlier_waits < snapshot.writer_waits
            && 0 < snapshot.waiting_writers
            && writer.blocked()
        {
            return Ok(snapshot);
        }
        if writer.result().is_some() {
            return Err(TestError(
                "pressure writer exited before a real throttled wait",
            ));
        }
        if deadline <= time::now() {
            return Err(TestError(
                "ordinary writer never blocked at real domain pressure",
            ));
        }
        sched::yield_now();
    }
}

#[ktest::suite(fixture = BackgroundOwner, suite_resources = UserSuite, context = "Thread", tags("fs-background", "fs-sync", "fs-path"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 卷已静止后只修改 inode 属性，不借普通文件写或显式 sync 启动后台义务。
    #[test]
    #[ktest(tags("W30", "fs-background-metadata"))]
    fn idle_metadata_update_starts_real_background_writeback(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        let file = fixture.prepare_file(context, 1)?;
        let volume = fixture.write.volume();
        let inode = checked(context, file.path().unwrap().inode())?.clone();
        let before = volume.writeback_domain().snapshot();
        let old_mode = checked(context, inode.metadata_snapshot())?.mode;
        let control = fixture.observe_rounds(true)?;
        checked(
            context,
            inode.update_metadata(volume.claim(), |attributes| {
                attributes.mode ^= 0o100;
            }),
        )?;
        context.expect_eq(checked(context, file.stat())?.mode, old_mode ^ 0o100);
        let after = control.round(1)?;
        context.expect_eq(volume.metadata_cache().has_dirty_or_writeback(), false);
        context.expect_eq(before.completed_bytes < after.completed_bytes, true);
        context.expect_eq(after.round_selected_metadata, 1);
        context.diagnostic(format_args!(
            "background-idle-metadata before={:?} after={:?}",
            before, after
        ));
        fixture.release_rounds();
        drained(&volume)?;
        Ok(())
    }

    /// 空文件创建只产生名字和 inode 元数据，静止卷仍须自行完成到期回写。
    #[test]
    #[ktest(tags("W30", "fs-background-metadata"))]
    fn idle_empty_create_starts_real_background_writeback(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        drop(fixture.prepare_file(context, 1)?);
        let volume = fixture.write.volume();
        let vfs = fixture.write.user.path.vfs().clone();
        let fs_context = fixture.write.user.path.context().clone();
        let root = vfs.root();
        let pathname = b"/test/background-empty";
        let before = volume.writeback_domain().snapshot();
        let control = fixture.observe_rounds(true)?;
        let created = checked(
            context,
            vfs.open_with(
                &fs_context,
                &root,
                pathname,
                (0, 0),
                Some((0o644, 0)),
                true,
                false,
                |path, _| crate::fs::open_path(&vfs, path),
            ),
        )?;
        let result = (|| {
            context.expect_eq(checked(context, created.stat())?.size, 0);
            let after = control.round(1)?;
            context.expect_eq(volume.metadata_cache().has_dirty_or_writeback(), false);
            context.expect_eq(after.round_selected_data, 0);
            context.expect_eq(before.completed_bytes < after.completed_bytes, true);
            context.diagnostic(format_args!(
                "background-idle-empty-create before={:?} after={:?}",
                before, after,
            ));
            Ok(())
        })();
        fixture.release_rounds();
        let removed = checked(
            context,
            vfs.unlink_from(&fs_context, &root, pathname, (0, 0)),
        );
        drop(created);
        let synced = checked(context, volume.sync());
        let quiescent = drained(&volume);
        result.and(removed).and(synced).and(quiescent)
    }

    /// 在真实域压力下观察普通写阻塞、锁外并行 I/O、真实完成带宽和独立 OFD 错误游标。
    #[test]
    #[case("completed_write", false)]
    #[case("accepted_eio_pwrite", true)]
    #[ktest(tags("W31", "controlled-ledger"), min_cpus = 2)]
    fn slow_accepted_wave_balances_real_writer(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
        fail: bool,
    ) -> TestResult {
        let file = fixture.prepare_file(context, PRESSURE_PAGES)?;
        let independent = fixture.write.open(context, b"/test/write-reserve", 2)?;
        let duplicate = file.clone();
        let inode = checked(context, file.path().unwrap().inode())?.clone();
        let volume = fixture.write.volume();
        let disk = fixture.write.user.path.owner.disk()?.clone();
        if !disk.limits().flush || global_snapshot().dirty_limit != 4096 {
            return Err(TestError(
                "pressure case requires real FLUSH and the unchanged 4096-page ceiling",
            ));
        }
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        let control = fixture.observe_rounds(false)?;
        let creations_before = creation_snapshot()?;
        let metadata_before = metadata_stats();
        let before = volume.writeback_domain().snapshot();
        let global_before = global_snapshot();
        let disk_before = disk.test_snapshot();
        disk.test_hold_file_writeback(&folio);
        if fail {
            disk.test_fail_next_data(BlockError::Io);
        }
        {
            let _writeback = inode.mapping().lock_writeback();
            context.expect_eq(file.write_at(0, &[0x42; PAGE_SIZE]), Ok(PAGE_SIZE));
        }
        until(
            "pressure anchor never reached a real used completion",
            || {
                disk.test_file_writeback_is_held(&folio)
                    && folio.writeback_sectors() != 0
                    && before.rounds < volume.writeback_domain().snapshot().rounds
            },
        )?;
        let held_at = time::now();
        let held = volume.writeback_domain().snapshot();
        context.expect_eq(held.round_selected_data, 1);
        context.expect_eq(
            disk_before.data_submitted < disk.test_snapshot().data_submitted,
            true,
        );
        let writing = file.clone();
        let writer_control = control.clone();
        let writer_domain = volume.writeback_domain().clone();
        let producer_deadline = time::now()
            .checked_add(time::Duration::from_secs(120))
            .ok_or(TestError("pressure producer deadline overflow"))?;
        let writer = fixture.write.consume_sync(0, move || {
            let input = [PRESSURE_BYTE; PAGE_SIZE];
            let mut page = 1;
            while !writer_control.producer_stop.load(Ordering::Acquire) {
                if producer_deadline <= time::now() {
                    return Err(FsError::InvalidOperation);
                }
                let offset = (page * PAGE_SIZE) as u64;
                let accepted = if fail {
                    writing.write_at(offset, &input)?
                } else {
                    writing.seek(SeekFrom::Start(offset))?;
                    writing.write(&input)?
                };
                if accepted != input.len() {
                    return Err(FsError::InvalidOperation);
                }
                writer_control
                    .accepted_bytes
                    .fetch_add(accepted, Ordering::Release);
                writer_control.produced.fetch_add(1, Ordering::Release);
                writer_control.returned_sequence.store(
                    writer_domain.snapshot().completion_seq as usize,
                    Ordering::Release,
                );
                page = (page + 1).min(PRESSURE_PAGES - 1);
                sched::cond_resched();
            }
            Ok(())
        })?;
        let pressured = throttled_writer(volume.writeback_domain(), &writer, held.writer_waits)?;
        let global_pressured = global_snapshot();
        let pressure_limits = volume.writeback_domain().ktest_limits();
        context.expect_eq(pressure_limits.1 <= pressured.dirty_pages, true);
        context.expect_eq(
            pressure_limits.0 < pressured.dirty_pages + pressured.writeback_pages,
            true,
        );
        context.expect_eq(
            0 < pressure_limits.1 && pressure_limits.1 <= pressure_limits.0,
            true,
        );
        context.expect_eq(pressure_limits.0 <= global_pressured.dirty_limit, true);
        context.expect_eq(1 <= pressured.writeback_pages, true);
        context.expect_eq(global_pressured.dirty_limit, 4096);
        context.expect_eq(global_pressured.background_limit, 2048);
        context.expect_eq(pressured.completed_bytes, held.completed_bytes);
        context.expect_eq(pressured.error_seq, held.error_seq);
        context.expect_eq(pressured.sample_end_ns, held.sample_end_ns);
        context.expect_eq(disk.test_file_writeback_is_held(&folio), true);
        let observed_cursor = Arc::try_new(AtomicUsize::new(usize::MAX))
            .map_err(|_| TestError("pressure cursor observation OOM"))?;
        let cursor = observed_cursor.clone();
        let probing = file.clone();
        let probe_inode = inode.clone();
        let probe_volume = volume.clone();
        let io_before = disk.test_snapshot();
        let independent_io = fixture.write.consume_sync(1, move || {
            cursor.store(
                probing.seek(SeekFrom::Current(0))? as usize,
                Ordering::Release,
            );
            // 仅用原写能力探测 inode/invalidate 独占已释放；压力主体始终经过 FileLike 节流。
            let access = probe_inode.acquire_write()?;
            let mut input = WriteBuffer::kernel(&[PRESSURE_BYTE]);
            if access.write_from(PAGE_SIZE as u64, false, &mut input)? != (PAGE_SIZE as u64, 1) {
                return Err(FsError::InvalidOperation);
            }
            probe_volume
                .claim()
                .flush(Vec::new_in(BioAllocator::Heap))
                .map_err(FsError::from)
        })?;
        until(
            "throttled writer retained cursor or inode locks or blocked unrelated real I/O",
            || independent_io.result().is_some(),
        )?;
        context.expect_eq(independent_io.result(), Some(Ok(())));
        context.expect_eq(observed_cursor.load(Ordering::Acquire) != usize::MAX, true);
        if fail {
            context.expect_eq(observed_cursor.load(Ordering::Acquire), 0);
        } else {
            let cursor = observed_cursor.load(Ordering::Acquire);
            context.expect_eq(
                (2 * PAGE_SIZE..=PRESSURE_PAGES * PAGE_SIZE).contains(&cursor),
                true,
            );
            context.expect_eq(cursor % PAGE_SIZE, 0);
        }
        let io_after = disk.test_snapshot();
        context.expect_eq(io_after.flush_submitted, io_before.flush_submitted + 1);
        context.expect_eq(io_before.completed < io_after.completed, true);
        context.expect_eq(disk.test_file_writeback_is_held(&folio), true);
        context.expect_eq(folio.writeback_sectors() != 0, true);
        let waits_before_release =
            throttled_writer(volume.writeback_domain(), &writer, held.writer_waits)?;
        let release_at = time::now();
        disk.test_resume_completions();
        control.producer_stop.store(true, Ordering::Release);
        let completed = control.round(1)?;
        until(
            "real completion did not wake the already throttled writer",
            || {
                held.completion_seq < control.returned_sequence.load(Ordering::Acquire) as u64
                    && waits_before_release.writer_wakes
                        < volume.writeback_domain().snapshot().writer_wakes
            },
        )?;
        let returned = volume.writeback_domain().snapshot();
        let global_after = global_snapshot();
        context.expect_eq(independent_io.result(), Some(Ok(())));
        finite_round(context, completed);
        context.expect_eq(completed.round_selected_data, 1);
        context.expect_eq(completed.later_dirty, true);
        context.expect_eq(held.completion_seq < returned.completion_seq, true);
        context.expect_eq(held.accepted_bytes < returned.accepted_bytes, true);
        context.expect_eq(
            waits_before_release.writer_wakes < returned.writer_wakes,
            true,
        );
        context.expect_eq(0 < returned.bandwidth_bytes_per_sec, true);
        context.expect_eq(0 < global_after.bandwidth_bytes_per_sec, true);
        context.expect_eq(held_at < release_at, true);
        context.expect_eq(
            held.completion_seq < control.returned_sequence.load(Ordering::Acquire) as u64,
            true,
        );
        context.expect_eq((folio.dirty_sectors(), folio.writeback_sectors()), (0, 0));
        if fail {
            context.expect_eq(held.error_seq < returned.error_seq, true);
            context.expect_eq(disk.test_completion_error_pending(), false);
            context.expect_eq(
                PAGE_SIZE as u64
                    <= completed.round_accepted_bytes - completed.round_completed_bytes,
                true,
            );
            let data_after_error = disk.test_snapshot().data_submitted;
            let error = Err(FsError::Io(BlockError::Io));
            context.expect_eq(file.sync_range(0..PAGE_SIZE as u64, false), error);
            context.expect_eq(independent.sync_range(0..PAGE_SIZE as u64, false), error);
            context.expect_eq(duplicate.sync_range(0..PAGE_SIZE as u64, false), Ok(()));
            context.expect_eq(file.sync_range(0..PAGE_SIZE as u64, false), Ok(()));
            context.expect_eq(disk.test_snapshot().data_submitted, data_after_error);
        } else {
            context.expect_eq(returned.error_seq, held.error_seq);
            context.expect_eq(
                completed.round_completed_bytes,
                completed.round_accepted_bytes,
            );
            context.expect_eq(
                held.completed_bytes + PAGE_SIZE as u64 <= returned.completed_bytes,
                true,
            );
            context.expect_eq(
                global_before.completed_bytes < global_after.completed_bytes,
                true,
            );
        }
        fixture.release_rounds();
        fixture.write.join_consumers()?;
        context.expect_eq(writer.result(), Some(Ok(())));
        context.expect_eq(independent_io.result(), Some(Ok(())));
        let accepted = control.accepted_bytes.load(Ordering::Acquire);
        context.expect_eq((PRESSURE_PAGES - 1) * PAGE_SIZE <= accepted, true);
        context.expect_eq(
            accepted,
            control.produced.load(Ordering::Acquire) * PAGE_SIZE,
        );
        bytes_at(context, &file, PAGE_SIZE as u64, &[PRESSURE_BYTE; 32])?;
        bytes_at(
            context,
            &file,
            ((PRESSURE_PAGES - 1) * PAGE_SIZE) as u64,
            &[PRESSURE_BYTE; 32],
        )?;
        checked(context, file.sync_range(0..u64::MAX, false))?;
        drained(&volume)?;
        let final_state = volume.writeback_domain().snapshot();
        context.expect_eq(inode.mapping().has_dirty_or_writeback(), false);
        context.expect_eq(final_state.waiting_writers, 0);
        context.expect_eq(returned.completed_bytes < final_state.completed_bytes, true);
        if final_state.sample_bytes == 0 || final_state.sample_elapsed_ns == 0 {
            return Err(TestError(
                "completed pressure wave has no measured bandwidth sample",
            ));
        }
        let measured_rate = (u128::from(final_state.sample_bytes) * 1_000_000_000
            / u128::from(final_state.sample_elapsed_ns)) as u64;
        context.expect_eq(final_state.bandwidth_bytes_per_sec, measured_rate);
        context.expect_eq(held.sample_end_ns < final_state.sample_end_ns, true);
        context.expect_eq(release_at.as_nanos() <= final_state.sample_end_ns, true);
        context.expect_eq(final_state.sample_end_ns <= time::now().as_nanos(), true);
        context.expect_eq(
            final_state.sample_elapsed_ns <= final_state.sample_end_ns,
            true,
        );
        context.expect_eq(
            final_state.sample_bytes <= final_state.completed_bytes,
            true,
        );
        let creations_after = creation_snapshot()?;
        let metadata_after = metadata_stats();
        context.expect_eq(
            creations_after.0 - creations_before.0,
            creations_after.1 - creations_before.1 + 2,
        );
        context.expect_eq(metadata_after.domains, metadata_before.domains);
        context.diagnostic(format_args!(
            "W31 accepted-error={} global-before={:?} global-pressure={:?} domain-limit={} domain-background={} hold-ns={} writer-accepted={}",
            fail, global_before, global_pressured, pressure_limits.0, pressure_limits.1,
            release_at.as_nanos() - held_at.as_nanos(), accepted,
        ));
        context.diagnostic(format_args!(
            "W31 held={:?} pressured={:?}",
            held, pressured
        ));
        context.diagnostic(format_args!(
            "W31 completed={:?} returned={:?}",
            completed, returned
        ));
        context.diagnostic(format_args!(
            "W31 final={:?} first-wave-sample-fresh={} final-sample-fresh={} measured-rate={}",
            final_state,
            held.sample_end_ns < completed.sample_end_ns,
            held.sample_end_ns < final_state.sample_end_ns,
            measured_rate,
        ));
        context.diagnostic(format_args!(
            "W31 allocated-task-ids={} shared-worker-creations={} owned-consumers=2 unattributed-task-ids={} metadata-before={:?} metadata-after={:?} real-used-software-tail=1 physical-DMA-held=0 hardware-fault=0",
            creations_after.0 - creations_before.0, creations_after.1 - creations_before.1,
            (creations_after.0 - creations_before.0) as i128
                - (creations_after.1 - creations_before.1) as i128 - 2,
            metadata_before, metadata_after,
        ));
        Ok(())
    }
}
