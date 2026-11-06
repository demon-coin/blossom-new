//! 后台回写单轮公平扫描、写回期间并发修改（Redirty）保留与显式范围同步（`sync_range`）边界测试。
//!
//! # 模块职责与架构定位
//! 本模块隶属于 `fs::ktest::user::background` 测试套件，专门验证回写扫描算法与页状态机在并发写入下的有界性与公平性：
//! 1. **热前缀不饿死后续页面（游标公平推进）**：当单次后台回写受预算限制（数据页最多 48 页、元数据页最多 16 页）只能写回部分脏页，且前半部分页面在每轮回写后被高频重复写脏时，回写域的扫描游标能够从上一轮结束位置继续向后推进，保证后半部分的脏数据页或 Inode 表元数据页在有限轮次内全部落盘。
//! 2. **在途写回页并发修改（Dirty + Writeback 共存）与下一轮回写接续**：当某个物理页（`Folio`）处于写回在途状态（`writeback_sectors != 0`）时，并发的 `write_at` 或共享内存映射（`mmap` / `copyout`）修改该页会重新设置其脏扇区位图（`dirty_sectors != 0`）。当前回写完成时仅清除 `Writeback` 位而保留 `Dirty` 位，并将该 Inode 保留在卷的脏 Inode 队列中，由下一轮后台回写继续处理。
//! 3. **显式范围同步（`sync_range`）的有界快照语义**：调用 `sync_range(0..8*PAGE_SIZE)` 时，仅等待进入同步时选中的待写（`ToWrite`）页面完成写回即返回，不会被同步期间并发写入产生的新脏页或范围外的脏页无限拖延。
//!
//! # 核心机制：单轮扫描游标推进与页状态转换
//! ```text
//!   [96 页脏文件 / 32 页脏 Inode 表]
//!   +-----------------------------------+-----------------------------------+
//!   | 前缀区 (0..48 数据 / 0..16 元数据) | 后缀区 (48..96 数据 / 16..32 元数据) |
//!   +-----------------------------------+-----------------------------------+
//!   Round 1: 游标从 0 扫描至 prefix -----> 写回前缀区，游标停在 prefix
//!   (并发写入再次将前缀区全部标脏)
//!   Round 2: 游标从 prefix 继续向后扫描 -> 写回后缀区，不会重头扫描前缀区而饿死后缀!
//!
//!   [在途写回期间并发修改 (Redirty) 页状态机]
//!   Dirty (dirty=1, wb=0)
//!     |-- begin_writeback() 迁入写回
//!     v
//!   Writeback (dirty=0, wb=1)
//!     |-- 并发 write_at / mmap copyout 写入同一物理页
//!     v
//!   Dirty + Writeback 共存 (dirty=1, wb=1)
//!     |-- 本轮块设备 DMA 完成 (end_writeback)
//!     v
//!   Dirty 保留 (dirty=1, wb=0) + later_dirty=true + Inode 仍挂在卷脏队列中
//! ```
//!
//! # 锁顺序与并发约束
//! - 测试在批量构造初始脏页集合时，可短暂获取 `inode.mapping().lock_writeback()` 与 `volume.metadata_cache().lock_writeback()` 阻塞后台回写扫描，以原子地建立 96 页脏数据或 32 页脏元数据快照；但在启动普通 `write_at` 前必须遵守锁层级（回写互斥锁在外，页内容锁在内）。

use super::*;
use crate::cache::FolioMark;
use crate::fs::ktest::user::write::{bytes_at, shared_alias, SyncObservation, SHARED_ALIAS};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::Mm;

/// 单轮回写公平性与边界测试使用的文件总页数（96 页，恰好为单轮 48 页数据预算的 2 倍）。
const ROUND_PAGES: usize = 96;

/// 检查指定 `inode` 是否仍登记在 Ext4 卷的脏 Inode 槽位表中。
fn dirty_inode_present(volume: &Ext4Volume, inode: &InodeRef) -> bool {
    (0..volume.dirty_inode_limit())
        .filter_map(|slot| volume.dirty_inode_at(slot))
        .any(|dirty| dirty.ptr_eq(inode))
}

/// 等待并发写入生产者至少完成两轮写入，且目标缓存页同时处于脏（`dirty_sectors != 0`）与在途写回（`writeback_sectors != 0`）共存状态。
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
    /// 启动并发消费者线程，持续通过普通 `write_at` 与共享内存映射 `copyout` 反复标脏第 0 页及范围外页面。
    ///
    /// 在收到 `control.producer_stop` 停止信号前，循环向文件偏移 `1`（由 `write_at` 写入）、共享映射虚拟地址 `SHARED_ALIAS + 2`（由 `copyout` 写入，对应同一物理页偏移 `2`）以及范围外偏移 `outside` 写入数据，并递增 `control.produced` 计数。
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

    /// 测试目标：验证当文件或 Inode 表的前半部分页面在每轮回写后被反复写脏时，后台回写扫描游标仍能公平推进并在后续有限轮次内写完后半部分脏页。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：
    ///    - 当 `metadata == false`（`file_pages`）时，准备 96 页文件并在持有回写锁期间将 96 页全部写脏；
    ///    - 当 `metadata == true`（`inode_table_pages`）时，从 `/indexed` 目录中选取分布在 32 个不同 Inode 表页上的 Inode，在持有回写锁期间修改其权限位使这 32 个元数据页全部变脏；
    /// 2. When：释放回写锁，等待第 1 轮后台回写完成（写回前 48 个数据页或前 16 个元数据页）；随后在第 2、3 轮开始前，故意将刚刚写干净的前缀页面全部重新写脏，再放行下一轮后台回写；
    /// 3. Then：每轮回写均严格遵守单轮页预算（`finite_round`），且后台扫描游标继续向后推进，后半部分（后 48 个数据页或后 16 个元数据页）在第 2 或第 3 轮结束时全部变干净（`dirty_sectors() == 0`），未因热前缀高频变脏而发生饥饿。
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

        // Step 1: 准备待观测的 32 个跨页 Inode（元数据模式）或 96 个文件数据页引用（文件数据模式）。
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

        // Step 2: 注册轮尾门控，并在持回写互斥锁期间一次性标脏全部目标页，确保首轮回写面对完整的初始脏集合。
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

        // Step 3: 等待第 1 轮回写完成，验证前缀区（48 页数据或 16 页元数据）已清脏，而后缀区仍保留脏标记。
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

        // Step 4: 在第 2~3 轮中反复重脏前缀区并放行门控，验证扫描游标继续推进直至后缀区全部写回清脏。
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

    /// 测试目标：验证单轮后台回写在持续并发重脏（Redirty）下仍能按 48 页预算有限结束，且在途写回页并发修改后的后继脏标记与卷级脏 Inode 登记得以保留并在下一轮继续推进。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备 96 页文件及第 0 页的共享内存映射 `mapping`；先写脏最后一页（第 95 页）并等待第 1 轮后台回写完成，使单向扫描游标归零并停在轮尾门控处；
    /// 2. When：在持 `lock_writeback` 期间将全部 96 页写脏并挂起第 0 页的块设备写回完成回调，随后放行门控进入第 2 轮回写；当第 2 轮选中 48 页且第 0 页处于在途写回状态时，启动 `redirty` 线程通过 `write_at` 与 `mapping.copyout` 持续写脏第 0 页与第 95 页，观测到第 0 页处于 Dirty + Writeback 共存状态后恢复块设备完成回调；
    /// 3. Then：
    ///    - 第 2 轮回写按 48 页上限正常结束，标记 `later_dirty == true`，第 0 页结束写回后仍保持脏状态（`dirty_sectors() != 0, writeback_sectors() == 0`），且 Inode 仍保留在卷的脏队列中；
    ///    - 放行门控后，第 3 轮回写继续选中剩余 48 页并推进累计完成字节数；最后停止并发写入并同步排空。
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

        // Step 1: 先写脏末页（第 95 页）并跑完第 1 轮回写，使单向扫描游标到达末尾后归零，并在轮尾门控处暂停后续轮次。
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
        // Step 2: 在持回写扫描锁期间将 0..96 页全部写脏，并挂起第 0 页的块设备写回完成回调。
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
        // Step 3: 放行门控使后台回写启动第 2 轮（选中前 48 页），等待第 0 页进入块设备在途写回状态。
        control.gate.complete();
        until(
            "background data did not reach real used with its software tail held",
            || {
                disk.test_file_writeback_is_held(&folio)
                    && folio.writeback_sectors() != 0
                    && volume.writeback_domain().snapshot().round_selected_data == 48
            },
        )?;

        // Step 4: 启动并发重脏生产者，通过 write_at 与共享 mmap 映射同时修改在途写回的第 0 页，确认 Dirty 与 Writeback 共存。
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

        // Step 5: 恢复块设备完成回调，验证第 2 轮以 48 页上限结束，第 0 页保留 Dirty 标记且第 3 轮继续推进。
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

    /// 测试目标：验证显式范围同步 `sync_range` 仅等待调用时刻选中的范围内脏页写回完成即返回，不会被同步期间并发写入产生的新脏页或范围外的脏页阻塞。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备 96 页文件，先通过修改第 95 页触发第 1 轮后台回写并使其停留在轮尾门控处（从而暂停后续自动后台回写轮次）；随后写脏范围内 `0..8` 页与范围外第 64 页，并挂起第 0 页的块设备写回完成回调；
    /// 2. When：在消费者线程 0 中调用 `sync_range(0..8*PAGE_SIZE, false)`，等待其在第 0 页的写回完成上阻塞；启动 `redirty` 线程持续重脏第 0 页（范围内）与第 64 页（范围外），待第 0 页出现 Dirty + Writeback 共存后恢复块设备完成回调；
    /// 3. Then：
    ///    - `sync_range` 在初始 8 个数据页（及最多 1 个关联的 Inode 元数据页）完成写回后立即返回 `Ok(())`，不会追逐并发重脏产生的新脏页；
    ///    - `sync_range` 返回时，并发生产者仍在运行，第 0 页与第 64 页均保留脏标记，且 Inode 仍在卷的脏队列中等待后续回写。
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

        // Step 1: 触发并门控第 1 轮后台回写以冻结后台自动调度，随后写脏范围内 0..8 页与范围外第 64 页。
        context.expect_eq(file.write_at(95 * PAGE_SIZE as u64, &[0x45]), Ok(1));
        let background = control.round(1)?;
        finite_round(context, background);

        for index in 0..8 {
            context.expect_eq(file.write_at((index * PAGE_SIZE) as u64, &[0x41]), Ok(1));
        }
        context.expect_eq(file.write_at(64 * PAGE_SIZE as u64, &[0x64]), Ok(1));
        let before = volume.writeback_domain().snapshot();

        // Step 2: 挂起第 0 页写回完成回调，启动显式范围同步 sync_range(0..8*PAGE_SIZE) 并确认其阻塞等待第 0 页。
        disk.test_hold_file_writeback(&folio);
        let syncing = file.clone();
        let sync = fixture.write.consume_sync(0, move || {
            syncing.sync_range(0..(8 * PAGE_SIZE) as u64, false)
        })?;
        until(
            "explicit range did not wait for its accepted original data",
            || disk.test_file_writeback_is_held(&folio) && sync.blocked(),
        )?;

        // Step 3: 在 sync_range 等待期间启动并发重脏线程，反复修改第 0 页与第 64 页。
        let producer = fixture.redirty(
            file.clone(),
            mapping,
            control.clone(),
            64 * PAGE_SIZE as u64,
        )?;
        redirty_observed(&control, &producer, &folio)?;

        // Step 4: 恢复块设备完成回调，验证 sync_range 完成初始 8 页同步后立即返回，不被后续并发脏页拖延。
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
        // 文件大小不变且盘块已预分配；本次范围同步仅提交 8 个数据页及最多 1 个 Inode 表元数据页。
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
