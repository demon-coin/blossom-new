//! 卷服务、同源保留页与最后 observer 的退休行为。

use super::*;
use crate::block::{Bio, BioAllocator, BioLease, BlkOperation};
use crate::mm::dma::MappedPages;
use crate::workqueue::testing::metadata_stats;

/// 同字节更新真实盘块窗口并取得原 metadata WB 凭证，不更改盘面内容或复制请求来源。
fn unchanged_metadata_writeback(
    volume: &Ext4Volume,
    block: u64,
) -> Result<crate::mm::FolioWriteback, FsError> {
    let size = volume.geometry().block_size;
    let buffer = volume.metadata_cache().get(block, size, None)?;
    buffer.update(volume.claim(), |bytes| {
        let first = bytes[0];
        bytes[..1].copy_from_slice(&[first]);
    })?;
    if PAGE_SIZE < buffer.offset() + size {
        return Err(FsError::InvalidOperation);
    }
    let mut content = buffer.folio().lock_content();
    // SAFETY: 真实 metadata 窗口已读入且同字节更新建立 Dirty；内容锁排斥修改，
    // 此终身 metadata 页没有用户映射，凭证沿原 FsClaim/Bio 全尾收权。
    let writeback = unsafe { content.begin_writeback(buffer.offset(), size)? }
        .ok_or(FsError::InvalidOperation)?;
    Ok(writeback.with_writeback_domain(volume.writeback_domain(), false))
}

/// 一个实际完整 lease 只构造一份原页父；构造结束后 lease 尾独立归还。
fn reserved_metadata_bio(
    volume: &Ext4Volume,
    block: u64,
    writeback: crate::mm::FolioWriteback,
    lease: BioLease,
) -> Result<Bio, FsError> {
    let mut memory = lease.memory()?;
    let mapped = MappedPages::from_writeback(writeback).map_err(|(_, writeback)| {
        drop(writeback);
        FsError::InvalidOperation
    })?;
    memory.push(mapped);
    // SAFETY: 映射消耗已初始化原 metadata WB 凭证，未导出 CPU 裸别名，
    // 原 FsClaim 持页及设备责任到全部 child/生成者真实收权，None 不创建完成 Task。
    let bio = unsafe {
        Bio::try_new(
            volume.claim().disk(),
            BlkOperation::Write,
            volume.view().absolute_byte(
                block * volume.geometry().block_size as u64,
                volume.geometry().block_size,
            )? / crate::block::SECTOR_BYTES,
            memory,
            0,
            None,
        )
    }?;
    drop(lease);
    Ok(bio)
}

/// CSD 同步参数只移动真实末拥有；回调不等待、分配或执行文件系统操作。
unsafe fn put_observer<T: Send>(argument: *mut ()) {
    assert!(!crate::arch::irq_enabled());
    // SAFETY: caller 将独占 Option<T> 栈槽保持到 cpu::call 返回；同步回调是唯一访问者，
    // T 为真实 observer 且其 Drop 只归还 allocator/原页，不进入睡眠或 join。
    let observer = unsafe { &mut *argument.cast::<Option<T>>() };
    drop(observer.take());
}

/// 调用者保持独占槽到同步 CSD 完成，只向真实在线异核交末次 observer。
///
/// # Safety
/// T 的末次 Drop 必须只归还 allocator/原页，不得睡眠、等待 IO 或 join；target 必须在线。
unsafe fn remote_observer_put<T: Send>(observer: &mut Option<T>, target: usize) {
    // SAFETY: target 由在线 CPU 表选择，栈槽在同步返回前存活且只由回调访问；
    // 本核无回调所需锁并服务 IPI，observer 末 put 仅运行非睡眠 allocator 析构。
    unsafe {
        cpu::call(
            target,
            put_observer::<T>,
            core::ptr::from_mut(observer).cast(),
        )
    };
}

#[ktest::suite(fixture = BackgroundOwner, suite_resources = UserSuite, context = "Thread", tags("fs-background", "fs-sync", "fs-path"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 正常卸载与最后 canonical 使用先退休卷服务；唯一真 Node Weak 的 allocator 才延长同源 C 页。
    #[test]
    #[ktest(tags("W34", "fs-reserve-retire"), resources("extra"), min_cpus = 2)]
    fn unmounted_volume_retires_before_remote_last_node_weak_returns_source_pages(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        let root_disk = fixture.write.user.path.owner.disk()?.clone();
        let extra_disk = fixture
            .write
            .user
            .path
            .owner
            .disks
            .get(1)
            .ok_or(TestError("reserve retirement requires ordinary extra disk"))?
            .clone();
        let host = checked(
            context,
            Ext4Volume::open(BlockView::whole(root_disk), AccessMode::ReadOnly, 64),
        )?;
        let volume = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(extra_disk.clone()),
                AccessMode::ReadWrite,
                64,
            ),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        if volume.geometry().block_size != PAGE_SIZE {
            return Err(TestError(
                "source retirement requires ordinary 4KiB extra image",
            ));
        }
        let cache = Arc::try_new(checked(context, InodeCache::try_new(16))?)
            .map_err(|_| TestError("mounted retirement inode cache OOM"))?;
        fixture.retired = Some(cache.watch());
        let watch = cache.watch();
        let vfs = checked(
            context,
            Vfs::try_new(
                crate::fs::Filesystem::Ext4(host.clone()),
                cache.clone(),
                128,
                4,
                AccessMode::ReadOnly,
            ),
        )?;
        let fs_context = checked(context, FsContext::new(vfs.root(), vfs.root()))?;
        let covered = checked(
            context,
            vfs.resolve(&fs_context, b"/mnt/extra", ResolveMode::Follow),
        )?;
        checked(
            context,
            vfs.mount(
                &covered,
                crate::fs::Filesystem::Ext4(volume.clone()),
                AccessMode::ReadWrite,
            ),
        )?;
        let mounted = checked(
            context,
            vfs.resolve(&fs_context, b"/mnt/extra", ResolveMode::Follow),
        )?;
        let inode = checked(context, mounted.inode())?.clone();
        checked(
            context,
            inode.update_metadata(volume.claim(), |attributes| attributes.mode ^= 0o100),
        )?;
        checked(context, volume.sync())?;
        drained(&volume)?;
        let metadata = volume.metadata_cache().clone();
        let source = metadata
            .reserve()
            .ok_or(TestError("mounted RW source absent"))?;
        fixture.source = Some(source.clone());
        if source.capacity() != 64 {
            return Err(TestError("source retirement requires actual C=64 reserve"));
        }
        let block = volume.geometry().blocks_count - 1;
        let buffer = checked(
            context,
            metadata.get(block, PAGE_SIZE, None).map_err(FsError::from),
        )?;
        let physical = buffer.folio().phys_addr();
        let slot = source
            .metadata_slot_for(buffer.folio())
            .ok_or(TestError("cold metadata is not an original source page"))?;
        let mut observer = Some(
            metadata
                .ktest_node(block * PAGE_SIZE as u64)
                .ok_or(TestError("cold metadata has no genuine Node Weak"))?,
        );
        let held = source.snapshot();
        drop(buffer);
        metadata.prune_unused();
        context.expect_eq(observer.as_ref().unwrap().upgrade().is_none(), true);
        context.expect_eq(source.snapshot().metadata_control_live, 1);
        context.expect_eq(source.metadata_folio(slot).phys_addr(), physical);
        context.expect_eq(vfs.unmount(&covered).err(), Some(FsError::Busy));
        drop(inode);
        drop(mounted);
        checked(context, vfs.unmount(&covered))?;
        let after_unmount = checked(
            context,
            vfs.resolve(&fs_context, b"/mnt/extra", ResolveMode::Follow),
        )?;
        context.expect_eq(after_unmount.ptr_eq(&covered), true);
        drop(after_unmount);
        drop(covered);
        drop(fs_context);
        drop(vfs);
        drop(host);
        drop(cache);
        checked(context, watch.wait_retired())?;
        drop(watch);
        drop(fixture.retired.take());
        extra_disk.test_flush_completions();
        let stopped = volume.writeback_domain().snapshot();
        context.expect_eq(
            (stopped.active, stopped.pending, stopped.running),
            (false, false, false),
        );
        let weak_volume = Arc::downgrade(&volume);
        let weak_metadata = Arc::downgrade(&metadata);
        let weak_domain = Arc::downgrade(volume.writeback_domain());
        let weak_source = Arc::downgrade(&source);
        let mut original_pages = [0; 64];
        for (index, address) in original_pages.iter_mut().enumerate() {
            let folio = source.metadata_folio(index);
            *address = folio.phys_addr();
            context.expect_eq(folio.ref_count(), 2);
        }
        context.expect_eq(
            original_pages
                .iter()
                .enumerate()
                .all(|(index, address)| !original_pages[..index].contains(address)),
            true,
        );
        context.diagnostic(format_args!(
            "W34 retired-before-last-weak source={:p} C={} observed-slot={} observed-pfn={} original-pages={:?} layout-held={:?} source-tail={:?} stopped={:?} driver={:?} work-metadata={:?}",
            Arc::as_ptr(&source), source.capacity(), slot, physical / PAGE_SIZE, original_pages,
            held, source.snapshot(), stopped, extra_disk.test_snapshot(), metadata_stats(),
        ));
        drop(metadata);
        drop(volume);
        drop(fixture.write.cold_volume.take());
        drop(fixture.source.take());
        drop(source);
        context.expect_eq(weak_volume.upgrade().is_none(), true);
        context.expect_eq(weak_metadata.upgrade().is_none(), true);
        context.expect_eq(weak_domain.upgrade().is_none(), true);
        context.expect_eq(weak_source.strong_count(), 1);
        let before_put = mm::stats();
        let target = (cpu::current().id + 1) % cpu::count();
        // SAFETY: 真 Node Weak 仅结束 IndexAllocator 控制尾及同源无 owner 基准页，
        // cache/canonical/Work 已退休；目标在线，独占栈槽保持到同步返回，无 FS/源锁。
        unsafe { remote_observer_put(&mut observer, target) };
        let after_put = mm::stats();
        context.expect_eq(weak_source.upgrade().is_none(), true);
        context.expect_eq(weak_source.strong_count(), 0);
        let free_before = before_put.buddy_free_pages + before_put.pcp_cached_pages;
        let free_after = after_put.buddy_free_pages + after_put.pcp_cached_pages;
        context.expect_eq(free_before + 64 <= free_after, true);
        context.diagnostic(format_args!(
            "W34 final-node-weak remote-put-cpu={} callback-irq-save=1 sleeping-callback=0 source-expired={} C-original-pages=64 actual-free-before={} actual-free-after={} before={:?} after={:?} volume-metadata-work-expired-before-put=1 normal-unmount=1",
            target, weak_source.upgrade().is_none(), free_before, free_after, before_put, after_put,
        ));
        Ok(())
    }

    /// 先在活卷回收旧 canonical，再由 cache owner 退休唤醒真实源与 scratch 等待者；新绑定不复活旧边界。
    #[test]
    #[case("zero_submit", false)]
    #[case("real_heap_oom", true)]
    #[ktest(tags("W34", "fs-reserve-stop", "fs-oom"), min_cpus = 2)]
    fn stopped_source_wakes_original_borrower_and_rejects_late_metadata_context(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
        oom: bool,
    ) -> TestResult {
        let disk = fixture.write.user.path.owner.disk()?.clone();
        let volume = checked(
            context,
            Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadWrite, 64),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        let cache = Arc::try_new(checked(context, InodeCache::try_new(4))?)
            .map_err(|_| TestError("source stop canonical cache OOM"))?;
        let watch = cache.watch();
        fixture.retired = Some(cache.watch());
        drop(checked(context, cache.iget(&volume, 2))?);
        let domain = volume.writeback_domain().clone();
        let source = volume
            .metadata_cache()
            .reserve()
            .ok_or(TestError("source stop has no reserve"))?;
        fixture.source = Some(source.clone());
        let old_context = domain.metadata_io_context(domain.snapshot().stop_sequence);
        let cold_block = volume.geometry().blocks_count - 1;
        context.expect_eq(
            checked(
                context,
                volume
                    .metadata_cache()
                    .cached_buffer(cold_block, volume.geometry().block_size)
                    .map_err(FsError::from),
            )?
            .is_none(),
            true,
        );
        let before = domain.snapshot();
        // cache_open 时末槽回收仍保卷服务，旧 inode 的 evict 必须在零 lane 压力与 stop 之前完成。
        context.diagnostic(format_args!(
            "W34 stop-stage=old-canonical-evict source={:p} oom={} slots={} active={}",
            Arc::as_ptr(&source),
            oom,
            cache.len(),
            before.active,
        ));
        context.expect_eq(cache.prune_unused(), 1);
        checked(context, watch.wait())?;
        context.expect_eq(watch.contains(2), false);
        context.expect_eq(cache.is_empty(), true);
        context.expect_eq(domain.snapshot().active, true);
        if !cache.is_empty() {
            return Err(TestError(
                "old canonical slot survived active-domain eviction",
            ));
        }
        fixture.lane = Some(checked(
            context,
            source.acquire_wb_lane(|| false).map_err(FsError::from),
        )?);
        context.expect_eq(source.snapshot().free_wb, 0);
        let control = WaveControl::try_new()?;
        fixture.control = Some(control.clone());
        let borrowing = domain.clone();
        let borrower = fixture
            .write
            .consume_sync(0, move || borrowing.borrow_storage().map(drop))?;
        let stopping = cache.watch();
        let stop_control = control.clone();
        let stopper = fixture.write.consume_sync(1, move || {
            stop_control.gate.wait();
            drop(cache);
            stopping.wait_retired()?;
            Ok(())
        })?;
        until("original storage borrower did not block on zero WB", || {
            borrower.blocked() && 0 < source.ktest_waiting()
        })?;
        let boundary = domain.request_progress(&volume);
        until(
            "original callback did not block behind borrowed scratch",
            || domain.snapshot().running,
        )?;
        let blocked = domain.snapshot();
        context.diagnostic(format_args!(
            "W34 stop-stage=old-owner-retire source={:p} zero-wb={} borrower-blocked={} source-waiting={} callback-running={} old-canonical-absent={}",
            Arc::as_ptr(&source), source.snapshot().free_wb == 0, borrower.blocked(),
            source.ktest_waiting(), blocked.running, !watch.contains(2),
        ));
        control.gate.complete();
        until(
            "external stop did not wake and finish original borrower and callback",
            || borrower.result().is_some() && stopper.result().is_some(),
        )?;
        fixture.write.join_consumers()?;
        context.expect_eq(
            borrower.result(),
            Some(Err(FsError::Io(BlockError::QueueStopped))),
        );
        context.expect_eq(stopper.result(), Some(Ok(())));
        let stopped = domain.snapshot();
        let old_progress = domain.wait_progress(boundary);
        context.expect_eq(old_progress.stopped, true);
        context.expect_eq(before.stop_sequence < stopped.stop_sequence, true);
        context.expect_eq(before.finished_rounds < stopped.finished_rounds, true);
        context.expect_eq(stopped.running, false);
        context.expect_eq(stopped.pending, false);
        context.expect_eq(source.ktest_waiting(), 0);
        let reads = disk.test_snapshot().submitted;
        context.expect_eq(
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, Some(old_context))
                .err(),
            Some(BlockError::NotReady),
        );
        context.expect_eq(disk.test_snapshot().submitted, reads);
        drop(fixture.lane.take());
        checked(context, watch.wait_retired())?;
        drop(watch);
        drop(fixture.retired.take());
        context.diagnostic(format_args!(
            "W34 stop-stage=new-canonical-bind source={:p} old-stop-sequence={} active={}",
            Arc::as_ptr(&source),
            stopped.stop_sequence,
            domain.snapshot().active,
        ));
        let next = Arc::try_new(checked(context, InodeCache::try_new(4))?)
            .map_err(|_| TestError("new source canonical cache OOM"))?;
        let next_watch = next.watch();
        fixture.retired = Some(next.watch());
        let root = checked(context, next.iget(&volume, 2))?;
        context.expect_eq(domain.snapshot().active, true);
        context.expect_eq(domain.wait_progress(boundary).stopped, true);
        context.expect_eq(
            borrower.result(),
            Some(Err(FsError::Io(BlockError::QueueStopped))),
        );
        context.expect_eq(
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, Some(old_context))
                .err(),
            Some(BlockError::NotReady),
        );
        drop(checked(
            context,
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, None)
                .map_err(FsError::from),
        )?);
        let pressure = if oom {
            Some(
                fixture
                    .write
                    .user
                    .path
                    .owner
                    .pressure
                    .take()
                    .ok_or(TestError("finite round real heap pressure missing"))?,
            )
        } else {
            None
        };
        let evidence = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("finite round evidence OOM"))?;
        let actor_evidence = evidence.clone();
        let round_domain = domain.clone();
        let round_volume = volume.clone();
        let finite = fixture.write.consume_sync(0, move || {
            let mut pressure = pressure;
            let allocation = pressure.as_mut().map(|pressure| pressure.exhaust());
            let start = round_domain.snapshot();
            let boundary = round_domain.request_progress(&round_volume);
            let progress = round_domain.wait_progress(boundary);
            let finish = round_domain.snapshot();
            drop(pressure);
            *actor_evidence.lock() = Some((allocation, start, finish, progress));
            Ok(())
        })?;
        until(
            "finite zero-submit/OOM round did not publish its actual finished boundary",
            || finite.result().is_some(),
        )?;
        fixture.write.join_consumers()?;
        context.expect_eq(finite.result(), Some(Ok(())));
        let (allocation, start, finish, progress) = evidence
            .lock()
            .take()
            .ok_or(TestError("finite round has no real boundary evidence"))?;
        context.expect_eq(allocation, oom.then_some((true, true)));
        context.expect_eq(start.finished_rounds < finish.finished_rounds, true);
        context.expect_eq(
            (
                progress.submitted_bytes,
                progress.completed_bytes,
                progress.stopped,
                progress.error,
            ),
            (0, 0, false, false),
        );
        context.expect_eq(
            (finish.round_selected_data, finish.round_selected_metadata),
            (0, 0),
        );
        // 第二次 owner 退休也先结束 canonical；同字节 Buffer 更新保原 metadata Dirty 与真实接受责任。
        drop(root);
        context.diagnostic(format_args!(
            "W34 stop-stage=new-canonical-evict source={:p} slots={} active={} finite-finished-rounds={}",
            Arc::as_ptr(&source), next.len(), domain.snapshot().active, finish.finished_rounds,
        ));
        context.expect_eq(next.prune_unused(), 1);
        checked(context, next_watch.wait())?;
        context.expect_eq(next_watch.contains(2), false);
        context.expect_eq(next.is_empty(), true);
        context.expect_eq(domain.snapshot().active, true);
        if !next.is_empty() {
            return Err(TestError(
                "new canonical slot survived active-domain eviction",
            ));
        }
        let buffer = checked(
            context,
            volume
                .metadata_cache()
                .get(cold_block, volume.geometry().block_size, None)
                .map_err(FsError::from),
        )?;
        disk.test_suspend_completions();
        checked(
            context,
            buffer
                .update(volume.claim(), |bytes| {
                    let first = bytes[0];
                    bytes[..1].copy_from_slice(&[first]);
                })
                .map_err(FsError::from),
        )?;
        drop(buffer);
        let accepted_boundary = domain.request_progress(&volume);
        until(
            "background callback did not accept original metadata before stop",
            || {
                domain.snapshot().running
                    && 0 < disk.test_snapshot().inflight
                    && 0 < source.snapshot().control_live
            },
        )?;
        let accepted = disk.test_snapshot();
        context.diagnostic(format_args!(
            "W34 stop-stage=accepted-owner-retire source={:p} old-canonical-absent={} accepted-driver={:?} source-layout={:?}",
            Arc::as_ptr(&source), !next_watch.contains(2), accepted, source.snapshot(),
        ));
        let io_stopping = next.watch();
        let io_stopper = fixture.write.consume_sync(1, move || {
            drop(next);
            io_stopping.wait_retired()?;
            Ok(())
        })?;
        until(
            "external stop did not wait for accepted IO/Work tail",
            || finish.stop_sequence < domain.snapshot().stop_sequence && io_stopper.blocked(),
        )?;
        context.expect_eq(io_stopper.result(), None);
        context.expect_eq(domain.wait_progress(accepted_boundary).stopped, true);
        disk.test_resume_completions();
        fixture.write.join_consumers()?;
        disk.test_flush_completions();
        context.expect_eq(io_stopper.result(), Some(Ok(())));
        context.expect_eq(domain.snapshot().active, false);
        context.expect_eq(domain.snapshot().pending, false);
        context.expect_eq(domain.snapshot().running, false);
        context.expect_eq(disk.test_snapshot().inflight, 0);
        context.expect_eq(source.snapshot().control_live, 0);
        context.expect_eq(source.snapshot().payload_live, 0);
        checked(context, next_watch.wait_retired())?;
        drop(next_watch);
        drop(fixture.retired.take());
        context.diagnostic(format_args!(
            "W34 stop source={:p} oom={} allocation-null={:?} borrower-blocked=1 callback-running-with-scratch-owned=1 before={:?} blocked={:?} stopped={:?} finite-start={:?} finite-finished={:?} progress={:?} accepted-driver={:?} finished-driver={:?} source-layout={:?} old-context-after-stop-and-rebind=NotReady old-waiter-stopped=1 canonical-evicted-before-owner-stop=1 canonical-retired-before-rebind=1 accepted-stop-by-owner=1",
            Arc::as_ptr(&source), oom, allocation, before, blocked, stopped, start, finish, progress,
            accepted, disk.test_snapshot(), source.snapshot(),
        ));
        fixture.control.take();
        fixture.source.take();
        Ok(())
    }

    /// 八个已完成的原父仍占真实 allocator 尾，第九个预建消费者只能从同源一单元前进。
    #[test]
    #[ktest(tags("W33", "W34", "fs-reserve-observer"), min_cpus = 2)]
    fn eight_completed_reserved_parents_block_ninth_until_real_weak_tail(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        let disk = fixture.write.user.path.owner.disk()?.clone();
        let volume = checked(
            context,
            Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadWrite, 64),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        if volume.geometry().block_size != PAGE_SIZE {
            return Err(TestError(
                "reserve observer requires ordinary 4KiB root image",
            ));
        }
        let source = volume
            .metadata_cache()
            .reserve()
            .ok_or(TestError("RW metadata has no real reserve"))?;
        context.expect_eq(volume.writeback_domain().snapshot().active, false);
        context.expect_eq(volume.writeback_domain().snapshot().rounds, 0);
        fixture.source = Some(source.clone());
        let lane = checked(
            context,
            source.acquire_wb_lane(|| false).map_err(FsError::from),
        )?;
        context.expect_eq(lane.len(), 8);
        fixture.lane = Some(lane);
        let control = WaveControl::try_new()?;
        fixture.control = Some(control.clone());
        let ninth = checked(context, unchanged_metadata_writeback(&volume, 9))?;
        let record = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("ninth acceptance observation OOM"))?;
        let actor_record = record.clone();
        let actor_source = source.clone();
        let actor_volume = volume.clone();
        let actor_control = control.clone();
        let actor = fixture.write.consume_sync(0, move || {
            actor_control.gate.wait();
            let lane =
                actor_source.acquire_wb_lane(|| actor_control.cancelled.load(Ordering::Acquire))?;
            let count = lane.len();
            let (_, lease) =
                lane.lease_available(|| actor_control.cancelled.load(Ordering::Acquire))?;
            let bio = reserved_metadata_bio(&actor_volume, 9, ninth, lease)?;
            let completion = bio.completion();
            actor_volume.claim().submit_bio(bio);
            let result = completion.wait();
            let memory = completion.take_memory()?;
            let reserved = matches!(memory.allocator(), BioAllocator::Reserved { .. });
            *actor_record.lock() = Some((
                count,
                completion.test_acceptance(),
                result,
                reserved,
                memory.as_ptr() as usize,
            ));
            drop(memory);
            drop(completion);
            drop(lane);
            result.error.map_or(Ok(()), |error| Err(FsError::Io(error)))
        })?;
        let before = disk.test_snapshot();
        for index in 0..8 {
            let block = index as u64 + 1;
            let writeback = checked(context, unchanged_metadata_writeback(&volume, block))?;
            let lease = checked(
                context,
                fixture
                    .lane
                    .as_ref()
                    .unwrap()
                    .try_lease(index, || false)
                    .map_err(FsError::from),
            )?
            .ok_or(TestError("prepared WB book lost a complete unit"))?;
            let bio = checked(
                context,
                reserved_metadata_bio(&volume, block, writeback, lease),
            )?;
            let completion = bio.completion();
            fixture.observers[index] = Some(completion.clone());
            volume.claim().submit_bio(bio);
            let result = completion.wait();
            context.expect_eq((result.completed, result.error), (PAGE_SIZE, None));
            context.expect_eq(completion.test_acceptance().2, PAGE_SIZE);
        }
        disk.test_flush_completions();
        let completed = disk.test_snapshot();
        let held = source.snapshot();
        context.expect_eq(completed.inflight, 0);
        context.expect_eq(held.control_live, 8);
        context.expect_eq(held.payload_live, 8);
        let mut returned = [const { None }; 8];
        let mut payload_addresses = [0; 8];
        for (index, (memory, completion)) in returned.iter_mut().zip(&fixture.observers).enumerate()
        {
            let owned = checked(
                context,
                completion
                    .as_ref()
                    .unwrap()
                    .take_memory()
                    .map_err(FsError::from),
            )?;
            context.expect_eq(
                matches!(owned.allocator(), BioAllocator::Reserved { .. }),
                true,
            );
            payload_addresses[index] = owned.as_ptr() as usize;
            *memory = Some(owned);
        }
        context.expect_eq(source.snapshot().payload_live, 8);
        drop(returned);
        context.expect_eq(source.snapshot().payload_live, 0);
        context.expect_eq(source.snapshot().control_live, 8);
        let mut weak = Some(fixture.observers[0].as_ref().unwrap().ktest_weak());
        drop(fixture.lane.take());
        control.gate.complete();
        until(
            "ninth consumer never blocked on the original empty source",
            || actor.blocked() && 0 < source.ktest_waiting(),
        )?;
        let blocked = disk.test_snapshot();
        context.expect_eq(actor.result(), None);
        context.expect_eq(source.snapshot().free_wb, 0);
        context.expect_eq(blocked.metadata_submitted, completed.metadata_submitted);
        drop(fixture.observers[0].take());
        context.expect_eq(source.snapshot().control_live, 8);
        context.expect_eq(source.snapshot().free_wb, 0);
        context.expect_eq(actor.blocked(), true);
        let target = (cpu::current().id + 1) % cpu::count();
        // SAFETY: 真 Bio Weak 保有实际 BioAllocator 控制尾；返回 Vec 已 Drop，
        // driver/生成者尾已 flush，目标在线，末 put 只归控制 allocator，不睡眠或 join。
        unsafe { remote_observer_put(&mut weak, target) };
        until(
            "one real allocator tail did not admit and finish the ninth parent",
            || actor.result().is_some(),
        )?;
        fixture.write.join_consumers()?;
        context.expect_eq(actor.result(), Some(Ok(())));
        let (count, acceptance, result, reserved, ninth_payload) = record
            .lock()
            .take()
            .ok_or(TestError("ninth parent has no actual acceptance evidence"))?;
        context.expect_eq(count, 1);
        context.expect_eq(
            (acceptance.2, result.completed, result.error, reserved),
            (PAGE_SIZE, PAGE_SIZE, None, true),
        );
        context.expect_eq(ninth_payload, payload_addresses[0]);
        disk.test_flush_completions();
        let after = source.snapshot();
        context.expect_eq(after.control_live, 7);
        context.expect_eq(after.payload_live, 0);
        context.expect_eq(after.free_wb, 1);
        context.expect_eq(
            completed.metadata_submitted < disk.test_snapshot().metadata_submitted,
            true,
        );
        context.diagnostic(format_args!(
            "W33-W34 observer source={:p} domain-inactive=1 background-rounds={} real-wb-parents=8 ninth-blocked=1 returned-Vec-before-control=1 last-strong-before-last-weak=1 ninth-book={} ninth-reserved={} payload-addresses={:?} ninth-payload={} driver-before={:?} driver-completed={:?} driver-blocked={:?} driver-after={:?} layout-held={:?} layout-after={:?} weak-put-cpu={}",
            Arc::as_ptr(&source), volume.writeback_domain().snapshot().rounds, count, reserved, payload_addresses, ninth_payload,
            before, completed, blocked, disk.test_snapshot(), held, after, target,
        ));
        for observer in &mut fixture.observers {
            drop(observer.take());
        }
        fixture.control.take();
        fixture.source.take();
        Ok(())
    }

    /// cache 空退场、首次加载 OOM 和带真实外部使用的退场均结束旧卷服务后才允许新域取得。
    #[test]
    #[case("pruned_empty", 0)]
    #[case("first_load_oom", 1)]
    #[case("live_inode", 2)]
    #[ktest(tags("W30", "fs-background-retire", "fs-oom"), min_cpus = 2)]
    fn retired_cache_allows_same_volume_new_canonical_domain(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
        path: usize,
    ) -> TestResult {
        let volume = checked(
            context,
            Ext4Volume::open(
                BlockView::whole(fixture.write.user.path.owner.disk()?.clone()),
                AccessMode::ReadWrite,
                64,
            ),
        )?;
        fixture.write.cold_volume = Some(volume.clone());
        let cache = Arc::try_new(checked(context, InodeCache::try_new(4))?)
            .map_err(|_| TestError("retirement cache owner OOM"))?;
        let watch = cache.watch();
        let held = if path == 1 {
            let mut pressure = fixture
                .write
                .user
                .path
                .owner
                .pressure
                .take()
                .ok_or(TestError("retirement allocator pressure was not prepared"))?;
            let observed = Arc::try_new(SpinLock::new(None))
                .map_err(|_| TestError("retirement OOM observation allocation failed"))?;
            let record = observed.clone();
            let original = cache.clone();
            let filesystem = volume.clone();
            let consumer = fixture.write.consume_sync(0, move || {
                let evidence = pressure.exhaust();
                let result = original.iget(&filesystem, 2).err();
                drop(pressure);
                *record.lock() = Some((evidence, result));
                Ok(())
            })?;
            fixture.write.join_consumers()?;
            context.expect_eq(consumer.result(), Some(Ok(())));
            let (evidence, result) = observed.lock().take().ok_or(TestError(
                "retirement OOM producer did not publish its real result",
            ))?;
            context.expect_eq(evidence, (true, true));
            context.expect_eq(result, Some(FsError::NoMemory));
            None
        } else {
            let inode = checked(context, cache.iget(&volume, 2))?;
            checked(
                context,
                inode.update_metadata(volume.claim(), |attributes| {
                    attributes.mode ^= 0o100;
                }),
            )?;
            checked(context, volume.sync())?;
            drained(&volume)?;
            context.expect_eq(
                0 < volume.writeback_domain().snapshot().completed_bytes,
                true,
            );
            if path == 0 {
                drop(inode);
                context.expect_eq(cache.prune_unused(), 1);
                context.expect_eq(cache.len(), 0);
                None
            } else {
                Some(inode)
            }
        };
        drop(cache);
        if let Some(inode) = held {
            context.expect_eq(watch.contains(inode.number()), true);
            let competing = checked(context, InodeCache::try_new(4))?;
            context.expect_eq(competing.iget(&volume, 2).err(), Some(FsError::Busy));
            drop(competing);
            drop(inode);
        }
        checked(context, watch.wait_retired())?;
        drop(watch);
        let next = checked(context, InodeCache::try_new(4))?;
        let next_watch = next.watch();
        let inode = checked(context, next.iget(&volume, 2))?;
        context.expect_eq(
            checked(context, inode.metadata_snapshot())?.kind,
            InodeKind::Directory,
        );
        drop(inode);
        drop(next);
        checked(context, next_watch.wait_retired())?;
        context.diagnostic(format_args!(
            "background-retirement path={} same-volume={} real-domain-rebind=1 snapshot={:?}",
            path,
            volume.device(),
            volume.writeback_domain().snapshot(),
        ));
        Ok(())
    }
}
