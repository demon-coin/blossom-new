//! 空闲状态下元数据自动回写与脏页压力下写入者节流（Dirty Throttling）的集成测试。
//!
//! # 模块职责与架构定位
//! 本模块隶属于 `fs::ktest::user::background` 测试套件，专门验证 `WritebackDomain` 的两类核心调度策略：
//! 1. **空闲卷元数据定时回写**：当卷处于无数据写入的静止状态时，单纯修改 Inode 属性（如 `chmod`）或创建空文件所产生的脏元数据块（`MetadataCache`），能够在后台定时回写轮次中被自动选中并落盘，无需显式调用 `fsync` 或借由普通文件写入顺带触发。
//! 2. **脏页上限节流与写回带宽统计**：当慢速块设备挂起写回完成事件、并发写入持续产生脏页并突破回写域上限（`dirty_limit`）时，验证普通写入线程在锁外被准确阻塞节流、无关 Inode/缓存页的并发 I/O 不受影响、写回完成后等待线程被及时唤醒、写回带宽（`bandwidth_bytes_per_sec`）按采样窗口准确计算，以及写回 I/O 错误通过独立打开文件描述（OFD）游标向 `sync_range` 正确报告。
//!
//! # 核心机制：脏页压力节流与锁外等待示意
//! ```text
//!   [并发写入线程 writer]               [WritebackDomain]               [无关探测线程 independent_io]
//!           |                                   |                                   |
//!   write() / write_at() 写入脏页               |                                   |
//!   释放 Inode 写锁与页内容锁                   |                                   |
//!           |                                   |                                   |
//!   balance_dirty_pages() 检查水位:             |                                   |
//!   dirty + writeback > domain_limit ---------> 进入 waiting_writers 队列睡眠       |
//!                                               |                           seek() 读取 OFD 游标
//!                                               |                           acquire_write() 写入第 1 页
//!                                               |                           volume.claim().flush() 正常完成!
//!                                               |                                   |
//!   [块设备恢复完成: test_resume_completions()] |                                   |
//!                                               |                                   |
//!   挂起的第 0 页写回完成 --------------------> writeback_pages 下降 / 更新带宽采样 |
//!   writer_wakes +1 <-------------------------- 唤醒阻塞的 writer 线程              |
//! ```
//!
//! # 锁顺序与并发约束
//! - **脏页节流必须在锁外执行**：普通文件写入路径在调用 `WritebackDomain` 节流等待前，必须已释放 `Inode` 写锁、页表/截断互斥锁以及文件偏移量游标锁；否则当回写线程或无关 I/O 线程尝试访问同一 Inode 时会发生死锁。
//! - **轮次门控与设备完成挂起配合**：测试在持 `lock_writeback()` 期间写入初始页并挂起块设备完成（`test_hold_file_writeback`），释放 `lock_writeback` 后后台回写将第 0 页提交给块设备并停在设备完成前，从而制造稳定的在途写回（`writeback_pages >= 1`）窗口。

use super::*;
use crate::block::BioAllocator;
use crate::fs::file::WriteBuffer;
use crate::fs::ktest::user::write::{bytes_at, SyncObservation};
use crate::fs::SeekFrom;
use crate::mm::reclaim::ReclaimAttempt;
use crate::workqueue::testing::{metadata_stats, pool_stats};

/// 用于触发回写域脏页上限（4096 页）压力测试的文件总页数（4097 页）。
const PRESSURE_PAGES: usize = 4097;
/// 脏页压力写入生产者填充的测试字节值。
const PRESSURE_BYTE: u8 = 0x86;

/// 汇总所有 CPU 核心工作队列池自启动以来累计创建的共享 Worker 线程总数。
///
/// 直接读取各 CPU 池的累计 `created` 计数器，避免因线程创建与销毁相互抵消而漏计瞬态创建的内核线程。
fn worker_creations() -> usize {
    (0..cpu::count()).map(|id| pool_stats(id).created).sum()
}

/// 等待所有 CPU 的工作队列管理线程（Manager）进入阻塞静止状态后，采集全局累计任务数与共享 Worker 创建数快照。
fn creation_snapshot() -> Result<(usize, usize), TestError> {
    until(
        "worker manager did not finish actual Task preparation",
        || (0..cpu::count()).all(crate::workqueue::testing::manager_blocked),
    )?;
    Ok((task::test_created_tasks(), worker_creations()))
}

/// 轮询等待并发写入线程因回写域脏页超限而进入节流阻塞状态，并返回阻塞时刻的回写域快照。
///
/// 在最长 60 秒的固定期限内反复检查回写域水位（`dirty_pages + writeback_pages > limit`）、
/// 累计节流等待次数（`writer_waits > earlier_waits`）、当前等待写者数（`waiting_writers > 0`）以及写入线程的调度器阻塞状态；
/// 每次未满足时调用 `sched::yield_now()` 让出 CPU，避免忙等饿死后台线程。
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

    /// 测试目标：验证在静止的 Ext4 卷上仅修改 Inode 元数据属性时，后台定时回写能够自动启动并将脏元数据块落盘。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备 1 页大小的干净测试文件并等待回写域完全排空，注册仅拦截元数据回写轮次的 `WaveControl` 门控；
    /// 2. When：直接调用 `inode.update_metadata` 翻转可执行权限位（`mode ^= 0o100`），不执行任何文件数据写入或显式 `sync`；
    /// 3. Then：后台回写自动完成第 1 轮元数据回写（`round_selected_metadata == 1`），`MetadataCache` 中不再有脏页或在途写回页，且累计完成字节数递增。
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

    /// 测试目标：验证在静止的 Ext4 卷上创建空文件时，新分配的目录项与 Inode 元数据块能由后台定时回写自动落盘。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：预热测试目录并使卷处于干净静止状态，注册拦截元数据回写轮次的门控；
    /// 2. When：通过 `vfs.open_with` 创建 `/test/background-empty` 空文件（文件大小为 0，仅产生目录项与 Inode 表脏元数据）；
    /// 3. Then：后台回写自动触发并完成第 1 轮（选中数据页为 0，元数据缓存全部变干净，累计完成字节数增加）；随后删除测试文件并同步排空。
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

    /// 测试目标：验证在慢速块设备写回挂起与脏页超限压力下，普通文件写入者的阻塞节流、锁外并发 I/O 独立性、写回带宽采样以及写回错误的独立文件描述符报告。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备 4097 页文件，挂起第 0 页的块设备写回完成回调（当 `fail == true` 时预设下一次数据写返回 `BlockError::Io`），写入第 0 页使其进入在途写回（Writeback）状态；
    /// 2. When：启动并发写入线程 `writer` 持续从第 1 页向后写入，直至脏页与写回页之和突破回写域上限并触发节流阻塞；在 `writer` 阻塞期间，启动 `independent_io` 线程读取文件游标、向第 1 页写入 1 字节并下发块设备 `flush`；随后恢复块设备完成回调并等待本轮回写与所有数据落盘；
    /// 3. Then：
    ///    - `writer` 阻塞期间未持有文件游标锁或 Inode 写锁，`independent_io` 顺利完成写入与设备冲刷；
    ///    - 恢复块设备完成回调后，阻塞的 `writer` 被唤醒（`writer_wakes` 递增），回写域与全局快照均计算出非零写回带宽；
    ///    - 若 `fail == true`（`accepted_eio_pwrite`），`error_seq` 递增，两个独立打开的文件句柄（`file` 与 `independent`）首次调用 `sync_range` 均捕获到 `FsError::Io(BlockError::Io)`，而共享同一底层 OFD 的克隆句柄（`duplicate`）及后续重试调用则返回 `Ok(())`；
    ///    - 核对内核任务 ID 分配总数，确认除共享工作队列 Worker 与测试显式创建的 2 个消费者线程外，回写过程未泄露或额外创建任何隐式任务。
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

        // Step 1: 挂起第 0 页的块设备写回完成事件（可选注入 I/O 错误），写入第 0 页并等待后台回写将其提交至块设备。
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

        // Step 2: 启动并发写入生产者持续写入后续页面，直至回写域脏页超限并将该写入者阻塞节流。
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

        // Step 3: 在写入者处于节流睡眠期间，启动独立探测线程验证文件游标与 Inode 写锁均已释放，且无关块设备 I/O 不受阻塞。
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
            // 直接获取 Inode 写访问权写入 1 字节，验证处于节流阻塞中的主写入线程未持有 Inode 独占写锁。
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

        // Step 4: 恢复块设备完成回调，验证写回完成事件唤醒被节流的写入线程，并核对错误游标或完成字节数。
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

        // Step 5: 解除轮次门控并汇合并发线程，执行全文件同步，校验写回带宽采样公式与内核线程创建账目。
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
