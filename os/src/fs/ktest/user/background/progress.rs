//! 有限后台轮的公平扫描、后继脏页与显式范围同步行为。

use super::*;
use crate::cache::FolioMark;
use crate::fs::ktest::user::write::{bytes_at, shared_alias, SyncObservation, SHARED_ALIAS};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::Mm;

const ROUND_PAGES: usize = 96;

/// 同一根的正式脏 inode 槽继续拥有后继义务，不建立测试专用 inode 名单。
fn dirty_inode_present(volume: &Ext4Volume, inode: &InodeRef) -> bool {
    (0..volume.dirty_inode_limit())
        .filter_map(|slot| volume.dirty_inode_at(slot))
        .any(|dirty| dirty.ptr_eq(inode))
}

/// 至少两次实际覆盖与映射写结束后观察 Dirty/WB 同页共存，未运行的 actor 不能充当生产者。
fn redirty_observed(
    control: &WaveControl,
    producer: &SyncObservation,
    folio: &FolioRef,
) -> TestResult {
    until(
        "ordinary and mapped producer did not redirty the held original page",
        || {
            producer.result().is_some()
                || (2 <= control.produced.load(Ordering::Acquire)
                    && folio.dirty_sectors() != 0
                    && folio.writeback_sectors() != 0)
        },
    )?;

    if producer.result().is_some() {
        return Err(TestError(
            "redirty producer exited before the original completion",
        ));
    }
    Ok(())
}

impl BackgroundOwner {
    /// 普通覆盖与真实共享映射持续产生后继 Dirty，控制者不能直接改页标记。
    fn redirty(
        &mut self,
        file: FileRef,
        mapping: Arc<Mm>,
        control: Arc<WaveControl>,
        outside: u64,
    ) -> Result<Arc<SyncObservation>, TestError> {
        let deadline = time::now()
            .checked_add(time::Duration::from_secs(60))
            .ok_or(TestError("redirty producer deadline overflow"))?;

        self.write.consume_sync(1, move || {
            while !control.producer_stop.load(Ordering::Acquire) {
                if deadline <= time::now() {
                    return Err(FsError::InvalidOperation);
                }
                if file.write_at(1, &[0x52])? != 1 {
                    return Err(FsError::InvalidOperation);
                }
                let copied = mapping.copyout(SHARED_ALIAS + 2, &[0x73]);
                if copied.copied != 1 || copied.error.is_some() {
                    return Err(FsError::BadAddress);
                }
                if file.write_at(outside, &[0x64])? != 1 {
                    return Err(FsError::InvalidOperation);
                }
                control.produced.fetch_add(1, Ordering::Release);
                sched::cond_resched();
            }
            Ok(())
        })
    }
}

#[ktest::suite(fixture = BackgroundOwner, suite_resources = UserSuite, context = "Thread", tags("fs-background", "fs-sync", "fs-path"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 每轮重新覆盖前半脏集合，后半原页仍须由后续有限后台轮真实写完。
    #[test]
    #[case("file_pages", false)]
    #[case("inode_table_pages", true)]
    #[ktest(tags("W30", "fs-background-progress"), min_cpus = 2)]
    fn hot_prefix_does_not_starve_later_original_pages(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
        metadata: bool,
    ) -> TestResult {
        let file = fixture.prepare_file(context, if metadata { 1 } else { ROUND_PAGES })?;
        let volume = fixture.write.volume();
        let inode = checked(context, file.path().unwrap().inode())?.clone();
        let mut records = Vec::new();
        let mut record_keys = Vec::new();
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(ROUND_PAGES)
            .map_err(|_| TestError("hot prefix original page observation OOM"))?;

        if metadata {
            records
                .try_reserve_exact(32)
                .map_err(|_| TestError("hot metadata inode observation OOM"))?;
            record_keys
                .try_reserve_exact(32)
                .map_err(|_| TestError("hot metadata original key observation OOM"))?;
            let cache = fixture.write.user.path.vfs().inodes().clone();
            let root = checked(context, cache.iget(&volume, 2))?;
            let directory = checked(context, root.lookup(&cache, b"indexed"))?;
            let mut cursor = checked(context, directory.directory_cursor(0))?;
            let geometry = volume.geometry();
            let per_page = (PAGE_SIZE / geometry.inode_size) as u64;
            for _ in 0..DIRECTORY_ENTRIES + 2 {
                let Some(entry) = checked(context, directory.dir_next(&cursor))? else {
                    break;
                };
                cursor = entry.next_cursor;
                if entry.name == b"." || entry.name == b".." {
                    continue;
                }
                let group = (entry.number - 1) / geometry.inodes_per_group;
                let page = ((entry.number - 1) % geometry.inodes_per_group) / per_page;
                if records.iter().any(|(key, _)| *key == (group, page)) {
                    continue;
                }
                records.push((
                    (group, page),
                    checked(context, cache.iget(&volume, entry.number))?,
                ));
                if records.len() == 32 {
                    break;
                }
            }
            if records.len() != 32 {
                return Err(TestError(
                    "actual indexed directory has fewer than 32 inode table pages",
                ));
            }
            checked(context, volume.sync())?;
            drained(&volume)?;
        } else {
            let mut attempt = ReclaimAttempt::new();
            for index in 0..ROUND_PAGES {
                pages.push((
                    index as u64,
                    checked(
                        context,
                        file.mapping_inode()
                            .unwrap()
                            .file_page(index as u64, &mut attempt),
                    )?,
                ));
            }
        }

        let control = fixture.observe_rounds(metadata)?;
        {
            let _data = inode.mapping().lock_writeback();
            let _metadata = volume.metadata_cache().lock_writeback();
            if metadata {
                for (_, record) in &records {
                    checked(
                        context,
                        record.update_metadata(volume.claim(), |attributes| {
                            attributes.mode ^= 0o100;
                        }),
                    )?;
                    let previous = pages.len();
                    let mut cursor = 0;
                    for _ in 0..volume.metadata_cache().capacity() {
                        let mut selected = [None];
                        let next = volume.metadata_cache().select_range(
                            cursor,
                            u64::MAX,
                            FolioMark::Dirty,
                            &mut selected,
                        );
                        if let Some((key, folio)) = selected[0].take() {
                            if !pages.iter().any(|(old, _)| *old == key) {
                                pages.push((key, folio));
                            }
                        }
                        let Some(next) = next else { break };
                        cursor = next;
                    }
                    if pages.len() != previous + 1 {
                        return Err(TestError(
                            "inode update did not dirty its unique original table page",
                        ));
                    }
                    record_keys.push(pages.last().unwrap().0);
                }
            }
            if checked(context, file.write_at(0, &[0x45]))? != 1 {
                return Err(TestError(
                    "hot prefix ordinary producer accepted a short write",
                ));
            }
            if !metadata {
                for index in 1..ROUND_PAGES {
                    if checked(context, file.write_at((index * PAGE_SIZE) as u64, &[0x45]))? != 1 {
                        return Err(TestError(
                            "hot prefix ordinary producer accepted a short write",
                        ));
                    }
                }
            }
        }

        let first = control.round(1)?;
        finite_round(context, first);
        let prefix = if metadata { 16 } else { 48 };
        pages.sort_unstable_by_key(|(key, page)| (page.dirty_sectors() != 0, *key));
        context.expect_eq(
            pages[..prefix]
                .iter()
                .all(|(_, page)| page.dirty_sectors() == 0),
            true,
        );
        context.expect_eq(
            pages[prefix..]
                .iter()
                .all(|(_, page)| page.dirty_sectors() != 0),
            true,
        );

        for round in 2..=3 {
            if metadata {
                for (position, (_, record)) in records.iter().enumerate() {
                    if pages[..prefix]
                        .iter()
                        .any(|(key, _)| *key == record_keys[position])
                    {
                        checked(
                            context,
                            record.update_metadata(volume.claim(), |attributes| {
                                attributes.mode ^= 0o100;
                            }),
                        )?;
                    }
                }
            } else {
                for (index, _) in &pages[..prefix] {
                    if checked(
                        context,
                        file.write_at(*index * PAGE_SIZE as u64, &[round as u8]),
                    )? != 1
                    {
                        return Err(TestError("hot prefix redirty accepted a short write"));
                    }
                }
            }

            control.gate.complete();
            let completed = control.round(round)?;
            finite_round(context, completed);
            context.diagnostic(format_args!(
                "background-hot-prefix metadata={} round={} prefix={} tail-dirty={} snapshot={:?}",
                metadata,
                round,
                prefix,
                pages[prefix..]
                    .iter()
                    .filter(|(_, page)| page.dirty_sectors() != 0)
                    .count(),
                completed,
            ));
            if pages[prefix..]
                .iter()
                .all(|(_, page)| page.dirty_sectors() == 0)
            {
                break;
            }
        }

        context.expect_eq(
            pages[prefix..]
                .iter()
                .all(|(_, page)| page.dirty_sectors() == 0),
            true,
        );

        fixture.release_rounds();
        checked(context, volume.sync())?;
        drained(&volume)?;
        Ok(())
    }

    /// 原轮尾门先稳定扫描续点，再产生96页脏集合；持续再脏期间两轮均须有限结束并保留下轮义务。
    #[test]
    #[ktest(tags("W30", "fs-shared"), min_cpus = 2)]
    fn finite_background_round_keeps_successor_dirty(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        let file = fixture.prepare_file(context, ROUND_PAGES)?;
        let inode = checked(context, file.path().unwrap().inode())?.clone();
        let volume = fixture.write.volume();
        let disk = fixture.write.user.path.owner.disk()?.clone();
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        let mapping = shared_alias(context, &file, 0)?;
        let control = fixture.observe_rounds(false)?;

        // 先让末页通过一次真实后台轮，原单向扫描到末尾后续点归零；同一轮尾门挡住后续扫描。
        context.expect_eq(
            file.write_at(((ROUND_PAGES - 1) * PAGE_SIZE) as u64, &[0x41]),
            Ok(1),
        );
        let prepared = control.round(1)?;
        finite_round(context, prepared);
        context.expect_eq(prepared.round_selected_data, 1);
        context.expect_eq(PAGE_SIZE as u64 <= prepared.round_accepted_bytes, true);
        context.expect_eq(
            prepared.round_completed_bytes,
            prepared.round_accepted_bytes,
        );

        let before = disk.test_snapshot();
        let entry_slots = volume.dirty_inode_limit();
        // 原扫描互斥仅稳定入口集合，普通写仍通过正式 inode/invalidate/content 路径。
        {
            let _writeback = inode.mapping().lock_writeback();
            for index in 0..ROUND_PAGES {
                if checked(context, file.write_at((index * PAGE_SIZE) as u64, &[0x41]))? != 1 {
                    return Err(TestError(
                        "background initial Dirty producer accepted a short write",
                    ));
                }
            }
            disk.test_hold_file_writeback(&folio);
        }
        // 96页已全部发表且0号页完成尾已武装，才放行原回调进入被测的48页轮。
        control.gate.complete();
        until(
            "background data did not reach real used with its software tail held",
            || {
                disk.test_file_writeback_is_held(&folio)
                    && folio.writeback_sectors() != 0
                    && volume.writeback_domain().snapshot().round_selected_data == 48
            },
        )?;

        let producer = fixture.redirty(
            file.clone(),
            mapping.clone(),
            control.clone(),
            ((ROUND_PAGES - 1) * PAGE_SIZE) as u64,
        )?;
        redirty_observed(&control, &producer, &folio)?;
        context.expect_eq(
            mapping.mapping(SHARED_ALIAS).unwrap().physical,
            folio.phys_addr(),
        );
        bytes_at(context, &file, 0, &[0x41, 0x52, 0x73])?;

        disk.test_resume_completions();
        let first = control.round(2)?;
        finite_round(context, first);
        context.expect_eq(first.round_limit, entry_slots);
        context.expect_eq(first.round_selected_data, 48);
        context.expect_eq(first.round_completed_bytes, first.round_accepted_bytes);
        context.expect_eq(first.later_dirty, true);
        context.expect_eq(ROUND_PAGES - 48 <= first.dirty_pages, true);
        context.expect_eq(
            (folio.dirty_sectors() != 0, folio.writeback_sectors()),
            (true, 0),
        );
        context.expect_eq(dirty_inode_present(&volume, &inode), true);
        context.expect_eq(producer.result(), None);

        let produced = control.produced.load(Ordering::Acquire);
        until(
            "producer stopped while the completed round remained gated",
            || produced < control.produced.load(Ordering::Acquire),
        )?;

        control.gate.complete();
        let second = control.round(3)?;
        finite_round(context, second);
        context.expect_eq(second.rounds, first.rounds + 1);
        context.expect_eq(first.completed_bytes < second.completed_bytes, true);
        context.expect_eq(second.round_selected_data, 48);
        context.expect_eq(second.later_dirty, true);
        context.expect_eq(producer.result(), None);
        context.expect_eq(
            before.data_submitted < disk.test_snapshot().data_submitted,
            true,
        );

        control.producer_stop.store(true, Ordering::Release);
        fixture.write.join_consumers()?;
        context.expect_eq(producer.result(), Some(Ok(())));
        fixture.release_rounds();
        checked(context, file.sync_range(0..u64::MAX, false))?;
        drained(&volume)?;
        context.expect_eq(inode.mapping().has_dirty_or_writeback(), false);
        context.diagnostic(format_args!(
            "W30 background first={:?} second={:?} producer-writes={} original-folio={} real-used-software-tail=1 physical-DMA-held=0 finite-entry-and-64-page-bound=1 next-round-progress=1",
            first, second, control.produced.load(Ordering::Acquire), folio.phys_addr(),
        ));
        Ok(())
    }

    /// 显式范围同步返回时原页再脏和范围外义务仍在，持续写入不能延长入口 ToWrite 集合。
    #[test]
    #[ktest(tags("W30", "fs-shared"), min_cpus = 2)]
    fn explicit_range_returns_while_future_dirty_is_produced(
        context: &mut TestContext<'_>,
        fixture: &mut BackgroundOwner,
    ) -> TestResult {
        let file = fixture.prepare_file(context, ROUND_PAGES)?;
        let inode = checked(context, file.path().unwrap().inode())?.clone();
        let volume = fixture.write.volume();
        let disk = fixture.write.user.path.owner.disk()?.clone();
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        let outside = checked(
            context,
            file.mapping_inode().unwrap().file_page(64, &mut attempt),
        )?;
        let mapping = shared_alias(context, &file, 0)?;
        let control = fixture.observe_rounds(false)?;

        context.expect_eq(file.write_at(95 * PAGE_SIZE as u64, &[0x45]), Ok(1));
        let background = control.round(1)?;
        finite_round(context, background);

        for index in 0..8 {
            context.expect_eq(file.write_at((index * PAGE_SIZE) as u64, &[0x41]), Ok(1));
        }
        context.expect_eq(file.write_at(64 * PAGE_SIZE as u64, &[0x64]), Ok(1));
        let before = volume.writeback_domain().snapshot();

        disk.test_hold_file_writeback(&folio);
        let syncing = file.clone();
        let sync = fixture.write.consume_sync(0, move || {
            syncing.sync_range(0..(8 * PAGE_SIZE) as u64, false)
        })?;
        until(
            "explicit range did not wait for its accepted original data",
            || disk.test_file_writeback_is_held(&folio) && sync.blocked(),
        )?;

        let producer = fixture.redirty(
            file.clone(),
            mapping,
            control.clone(),
            64 * PAGE_SIZE as u64,
        )?;
        redirty_observed(&control, &producer, &folio)?;

        disk.test_resume_completions();
        until(
            "explicit range chased future Dirty instead of ending its entry selection",
            || sync.result().is_some(),
        )?;
        context.expect_eq(sync.result(), Some(Ok(())));
        context.expect_eq(producer.result(), None);
        context.expect_eq(control.producer_stop.load(Ordering::Acquire), false);
        context.expect_eq(
            (folio.dirty_sectors() != 0, folio.writeback_sectors()),
            (true, 0),
        );
        context.expect_eq(
            (outside.dirty_sectors() != 0, outside.writeback_sectors()),
            (true, 0),
        );
        context.expect_eq(dirty_inode_present(&volume, &inode), true);
        let after = volume.writeback_domain().snapshot();
        let accepted = after.accepted_bytes - before.accepted_bytes;
        // 文件不增长且已有全部块；本轮只有八个数据页和同一 inode 记录所在的一个元数据页。
        context.expect_eq((8 * PAGE_SIZE) as u64 <= accepted, true);
        context.expect_eq(accepted <= (9 * PAGE_SIZE) as u64, true);
        context.expect_eq(after.rounds, background.rounds);

        let produced = control.produced.load(Ordering::Acquire);
        until(
            "range producer no longer advances after sync returned",
            || produced < control.produced.load(Ordering::Acquire),
        )?;

        control.producer_stop.store(true, Ordering::Release);
        fixture.write.join_consumers()?;
        context.expect_eq(producer.result(), Some(Ok(())));
        fixture.release_rounds();
        checked(context, file.sync_range(0..u64::MAX, false))?;
        drained(&volume)?;
        context.diagnostic(format_args!(
            "W30 explicit-range-pages=8 accepted-bytes={} later-original-Dirty=1 outside-page=64 outside-Dirty=1 producer-active-at-return=1 background-round={}",
            accepted, background.rounds,
        ));
        Ok(())
    }
}
