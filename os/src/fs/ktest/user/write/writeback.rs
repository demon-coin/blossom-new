//! 原页批次、同步顺序、错误序列及普通同步 ABI 的真实完成窗口。

use super::*;

impl UserOwner {
    /// 普通写程序同步退出后关闭原缓存域，在同盘只读新生命周期核全文与内嵌链接盘面格式。
    pub(in crate::fs::ktest::user) fn production_writeback_cold_reopen(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        const PROGRAM: &[u8] = b"/test/bin/user-writeback";
        self.prepare_mode(context, AccessMode::ReadWrite, 4)?;
        let event = self.run_named(context, PROGRAM, b"w", false, 0)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            return Err(TestError("production writeback writer failed"));
        }

        let disk = self.path.owner.disk()?.clone();
        let root = self.path.vfs().root();
        let old_device = root.filesystem().device();
        let inode = checked(context, root.inode())?;
        let volume = checked(context, inode.disk_volume())?;
        let weak_volume = Arc::downgrade(volume);
        let weak_metadata = Arc::downgrade(volume.metadata_cache());
        let weak_root = root.dentry().downgrade();
        let weak_vfs = Arc::downgrade(self.path.vfs());
        let weak_graph = Arc::downgrade(self.graph());
        drop(root);
        drop(self.graph.take());
        self.path.release_view();

        // 开放缓存仍持卷绑定；必须关闭最后 cache owner 并等原收据，不能把清槽当作冷重开。
        let cache = self
            .path
            .owner
            .cache
            .take()
            .ok_or(TestError("production writeback original cache missing"))?;
        cache.ktest_set_load_hook(None);
        let weak_cache = Arc::downgrade(&cache);
        assert!(self.path.owner.cache_watch.is_none());
        self.path.owner.cache_watch = Some(cache.watch());
        drop(cache);
        checked(
            context,
            self.path
                .owner
                .cache_watch
                .as_ref()
                .expect("production writeback retirement watch missing")
                .wait_retired(),
        )?;
        if weak_cache.strong_count() != 0
            || weak_volume.strong_count() != 0
            || weak_metadata.strong_count() != 0
            || weak_root.strong_count() != 0
            || weak_vfs.strong_count() != 0
            || weak_graph.strong_count() != 0
        {
            return Err(TestError(
                "production writeback still has old FS or user owners",
            ));
        }

        drop((
            weak_cache,
            weak_volume,
            weak_metadata,
            weak_root,
            weak_vfs,
            weak_graph,
        ));
        disk.test_flush_completions();
        if disk.test_snapshot().inflight != 0 {
            return Err(TestError("production writeback old IO is still inflight"));
        }
        drop(self.path.owner.cache_watch.take());

        let reads_before = disk.test_snapshot().submitted;
        self.path.owner.cache = Some(
            Arc::try_new(checked(context, InodeCache::try_new(64))?)
                .map_err(|_| TestError("production writeback cold cache OOM"))?,
        );
        self.prepare_mode(context, AccessMode::ReadOnly, 4)?;
        let root = self.path.vfs().root();
        context.expect_eq(
            checked(context, checked(context, root.inode())?.disk_volume())?
                .disk()
                .identity(),
            disk.identity(),
        );
        context.expect_eq(root.filesystem().mode(), AccessMode::ReadOnly);
        context.expect_eq(root.filesystem().device() != old_device, true);
        drop(root);

        let event = self.run_named(context, PROGRAM, b"r", false, 0)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            return Err(TestError("production writeback cold reader failed"));
        }
        {
            let link = checked(
                context,
                self.path.vfs().resolve(
                    self.path.context(),
                    b"/test/writeback-dir/link",
                    ResolveMode::NoFollow,
                ),
            )?;
            let inode = checked(context, link.inode())?;
            let metadata = checked(context, inode.metadata_snapshot())?;
            context.expect_eq(metadata.fast_symlink, true);
            context.expect_eq(metadata.flags & codec::EXTENTS, 0);
        }
        let reads_after = disk.test_snapshot().submitted;
        context.expect_eq(reads_before < reads_after, true);
        if reads_after <= reads_before {
            return Err(TestError(
                "production writeback cold reader issued no disk IO",
            ));
        }

        context.diagnostic(format_args!(
            "W35 production-writeback bytes=8229 writer-status=0 reader-status=0 old-cache-strong=0 old-volume-strong=0 old-metadata-strong=0 old-root-strong=0 old-vfs-strong=0 old-graph-strong=0 same-disk=1 cold-read-submitted={} disk-stopped=0",
            reads_after - reads_before
        ));
        Ok(())
    }
}

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 目标页确认为空洞后占住首原页的真实物理后继，多个父请求先发表并 kick，随后消费全部原尾。
    /// 十六页准备共用一份回收预算，完成后的整窗身份检查另用一份预算。
    #[test]
    #[ktest(tags("W10", "fs-sync"), min_cpus = 2)]
    fn original_folio_batch_publishes_prefix_before_wait(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = checked(context, inode.disk_volume())?.clone();
        let allocation_hint = (inode.number() - 1) / volume.geometry().inodes_per_group;
        let block = volume.geometry().block_size as u64;
        let base = checked(context, file.stat())?
            .size
            .div_ceil(PAGE_SIZE as u64)
            * PAGE_SIZE as u64;
        let end = base + 16 * PAGE_SIZE as u64;

        checked(
            context,
            inode.with_map_read(|metadata, root| {
                // 本地查询终点只放开 EOF 裁剪，不修改 canonical size 或块树。
                let mut lookup = *metadata;
                lookup.size = end;
                for logical in base / block..end / block {
                    let mapping = volume.map_block(&lookup, root, logical, None)?;
                    if !matches!(mapping, BlockMapping::Hole { .. }) {
                        context.diagnostic(format_args!(
                            "W10 preparation requires actual hole logical={} mapping={:?}",
                            logical, mapping
                        ));
                        return Err(FsError::InvalidOperation);
                    }
                }
                Ok(())
            }),
        )?;

        let mut folios: [Option<mm::FolioRef>; 16] = core::array::from_fn(|_| None);
        let mut held = Vec::new();
        let prepared = (|| -> Result<(u64, u64, u64, usize), TestError> {
            let mut prepare_attempt = ReclaimAttempt::new();
            context.expect_eq(checked(context, file.write_at(base, &[0xc4; 4096]))?, 4096);
            folios[0] = Some(checked(
                context,
                file.mapping_inode()
                    .unwrap()
                    .file_page(base / PAGE_SIZE as u64, &mut prepare_attempt),
            )?);
            let mapping = checked(context, inode.map_block(base / block))?;
            context.diagnostic(format_args!("W10 first-page mapping={:?}", mapping));
            let BlockMapping::Mapped {
                physical: first, ..
            } = mapping
            else {
                return Err(TestError("batch first file page not mapped"));
            };

            let target = first + PAGE_SIZE as u64 / block;
            let budget = volume.geometry().blocks_count - volume.geometry().first_data_block;
            // 本例没有并发分配者；second 的首 data 申请先于同次节点摘离，已占用目标此时不可申请。
            let mut target_state = checked(context, block_allocation_bit(&volume, target))?;
            context.diagnostic(format_args!(
                "W10 fragmentation first-physical={} target-successor={} target-allocation-state={:?} reservation-budget={} second-unprepared=1",
                first, target, target_state, budget
            ));
            let mut occupied = target_state == Some(true);
            let mut reserved_bounds: Option<(u64, u64)> = None;
            for _ in 0..budget {
                if occupied {
                    break;
                }
                // 容量准备先于新责任；同组较低空洞的 token 一并保留，直到真实目标位被占住。
                held.try_reserve(1)
                    .map_err(|_| TestError("W10 physical separator owner OOM"))?;
                let separator = checked(context, volume.allocate_block(allocation_hint))?;
                let physical = separator.physical();
                held.push(separator);
                reserved_bounds =
                    Some(reserved_bounds.map_or((physical, physical), |(low, high)| {
                        (low.min(physical), high.max(physical))
                    }));
                context.diagnostic(format_args!(
                    "W10 fragmentation first-physical={} target-successor={} separator-physical={} reserved-physical-bounds={:?} held-reservations={} second-unprepared=1",
                    first, target, physical, reserved_bounds, held.len()
                ));
                if physical == target {
                    occupied = true;
                    break;
                }
                if target_state.is_none() {
                    // 只在本次真实分配之后重核未知组；不把 BLOCK_UNINIT 的随机位当已拥有。
                    target_state = checked(context, block_allocation_bit(&volume, target))?;
                    occupied = target_state == Some(true);
                    context.diagnostic(format_args!(
                        "W10 target-after-real-allocation first-physical={} target-successor={} target-allocation-state={:?} held-reservations={}",
                        first, target, target_state, held.len()
                    ));
                }
            }
            if !occupied {
                return Err(TestError("W10 physical successor neither unavailable nor reserved within geometry block budget"));
            }
            context.diagnostic(format_args!(
                "W10 target-exclusion first-physical={} target-successor={} target-allocation-state={:?} exclusion-source={} reserved-physical-bounds={:?} held-reservations={}",
                first, target, target_state,
                if target_state == Some(true) { "existing-unavailable" } else { "owned-token" },
                reserved_bounds, held.len()
            ));

            let separators = held.len();
            let mut second = 0;
            for (index, slot) in folios.iter_mut().enumerate().skip(1) {
                context.expect_eq(
                    checked(
                        context,
                        file.write_at(base + (index * PAGE_SIZE) as u64, &[0xc4; 4096]),
                    )?,
                    4096,
                );
                *slot = Some(checked(
                    context,
                    file.mapping_inode()
                        .unwrap()
                        .file_page(base / PAGE_SIZE as u64 + index as u64, &mut prepare_attempt),
                )?);
                if index == 1 {
                    let mapping =
                        checked(context, inode.map_block((base + PAGE_SIZE as u64) / block))?;
                    context.diagnostic(format_args!(
                        "W10 second-page first-physical={} target-successor={} reserved-physical-bounds={:?} held-reservations={} mapping={:?}",
                        first, target, reserved_bounds, held.len(), mapping
                    ));
                    let BlockMapping::Mapped { physical, .. } = mapping else {
                        return Err(TestError("batch second file page not mapped"));
                    };
                    second = physical;
                    context.diagnostic(format_args!(
                        "W10 fragmentation first-physical={} target-successor={} second-physical={} reserved-physical-bounds={:?} held-reservations={}",
                        first, target, second, reserved_bounds, held.len()
                    ));
                    for separator in held.drain(..) {
                        volume.release_unpublished_block(separator);
                    }
                }
            }
            Ok((first, target, second, separators))
        })();

        // 任一步失败均先显式归还全部未发表 token；已接受文件内容仍由原 fixture 真实同步。
        for separator in held.drain(..) {
            volume.release_unpublished_block(separator);
        }
        let (first, target, second, separators) = prepared?;
        context.expect_eq(first + PAGE_SIZE as u64 / block != second, true);

        let disk = fixture.user.path.owner.disk()?.clone();
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        let syncing = inode.clone();
        let observed = fixture.consume_sync(0, move || {
            crate::fs::writeback::sync_inode_range(&syncing, base..base + 16 * PAGE_SIZE as u64)
        })?;
        until(
            "W10 accepted batch did not make progress before wait",
            || {
                1 < disk.test_snapshot().inflight
                    && before.data_submitted + 1 < disk.test_snapshot().data_submitted
                    && observed.blocked()
            },
        )?;
        let accepted = disk.test_snapshot();
        context.expect_eq(before.notifications < accepted.notifications, true);
        context.expect_eq(
            2 <= folios
                .iter()
                .flatten()
                .filter(|folio| folio.writeback_sectors() != 0)
                .count(),
            true,
        );

        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        context.expect_eq(
            folios
                .iter()
                .flatten()
                .all(|folio| folio.dirty_sectors() == 0 && folio.writeback_sectors() == 0),
            true,
        );
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);

        let mut verify_attempt = ReclaimAttempt::new();
        for (index, folio) in folios[..16].iter().enumerate() {
            bytes_at(
                context,
                &file,
                base + (index * PAGE_SIZE) as u64,
                &[0xc4; 4096],
            )?;
            context.expect_eq(
                folio.as_ref().unwrap().phys_addr(),
                checked(
                    context,
                    file.mapping_inode()
                        .unwrap()
                        .file_page(base / PAGE_SIZE as u64 + index as u64, &mut verify_attempt),
                )?
                .phys_addr(),
            );
        }
        context.diagnostic(format_args!("W10 real-accepted-prefix-before-wait=1 original-folios=16 real-parent-and-queue-tails=1 first-physical={} target-successor={} second-physical={} all-unpublished-separators-released={} accepted-inflight={} notifications={} hardware-reverse-order-tested=0", first, target, second, separators, accepted.inflight, accepted.notifications - before.notifications));
        Ok(())
    }

    /// 正式数据、元数据与 FLUSH 后结束全部 FS 和 VMA 拥有，弱引用到期后重新读盘验证新根、CRC 和内容。
    #[test]
    #[ktest(tags("W07", "fs-write-cold", "fs-sync"))]
    fn formal_sync_drop_all_owners_and_cold_reload(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        assert!(
            fixture.user.graph.is_none()
                && fixture.user.running.is_none()
                && fixture.shared_mm.is_none()
        );
        assert!(
            fixture.inodes.is_empty()
                && fixture.cold_volume.is_none()
                && fixture.cold_cache.is_none()
        );

        let disk = fixture.user.path.owner.disk()?.clone();
        let file = fixture.cold_open(context)?.0;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        if !disk.limits().flush {
            return Err(TestError("W07 cold reload requires real FLUSH"));
        }
        let block = volume.geometry().block_size as u64;
        let base = checked(context, file.stat())?
            .size
            .div_ceil(PAGE_SIZE as u64)
            * PAGE_SIZE as u64;
        let traditional = checked(context, inode.metadata_snapshot())?.flags & codec::EXTENTS == 0;
        let logical = if traditional {
            [0, 12, 268, 65804, 65806]
        } else {
            [0, 2, 4, 6, 8]
        };

        let mut physical = [0u64; 5];
        for (index, logical) in logical.iter().copied().enumerate() {
            let offset = base + logical * block;
            let input = [0x41 + index as u8; 4096];
            context.expect_eq(
                file.write_at(offset, &input[..block as usize]),
                Ok(block as usize),
            );
            let BlockMapping::Mapped {
                physical: mapped, ..
            } = checked(context, inode.map_block(offset / block))?
            else {
                return Err(TestError("cold producer block not mapped"));
            };
            physical[index] = mapped;
        }
        let mapped = tree(context, &inode)?;
        if !traditional {
            context.expect_eq(0 < mapped.depth, true);
        }

        let before = disk.test_snapshot();
        checked(context, file.sync_range(0..u64::MAX, false))?;
        let synced = disk.test_snapshot();
        context.expect_eq(before.data_submitted < synced.data_submitted, true);
        context.expect_eq(before.metadata_submitted < synced.metadata_submitted, true);
        context.expect_eq(synced.flush_submitted - before.flush_submitted, 1);
        let stat = checked(context, file.stat())?;
        let metadata = checked(context, inode.metadata_snapshot())?;
        let root = checked(context, inode.with_map_read(|_, root| Ok(*root)))?;
        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("cold VMA owner OOM"))?;
        vm(
            context,
            mm.map_file(SOURCE, 4096, RO, RO, file.clone(), base, false),
        )?;
        mapped_bytes(context, &mm, SOURCE, &[0x41]);

        let (reopened, reads_before) = fixture.cold_reopen(context, mm, file, inode, volume)?;
        let reloaded_inode = fixture.inodes.last().unwrap().clone();
        let mut reloaded_stat = checked(context, reopened.stat())?;
        context.expect_eq(reloaded_stat.device != stat.device, true);
        // 新卷实例分配新的内核设备号；盘面属性仍须逐项相同。
        reloaded_stat.device = stat.device;
        context.expect_eq(reloaded_stat, stat);
        context.expect_eq(
            checked(context, reloaded_inode.metadata_snapshot())?,
            metadata,
        );
        context.expect_eq(
            checked(context, reloaded_inode.with_map_read(|_, root| Ok(*root)))?,
            root,
        );
        let reloaded = tree(context, &reloaded_inode)?;
        context.expect_eq(
            (
                reloaded.depth,
                reloaded.extents,
                reloaded.data,
                reloaded.nodes,
                &reloaded.blocks,
            ),
            (
                mapped.depth,
                mapped.extents,
                mapped.data,
                mapped.nodes,
                &mapped.blocks,
            ),
        );
        for (index, logical) in logical.iter().copied().enumerate() {
            let offset = base + logical * block;
            let expected = [0x41 + index as u8; 4096];
            bytes_at(context, &reopened, offset, &expected[..block as usize])?;
            let BlockMapping::Mapped {
                physical: mapped, ..
            } = checked(context, reloaded_inode.map_block(offset / block))?
            else {
                return Err(TestError("cold reloaded block not mapped"));
            };
            context.expect_eq(mapped, physical[index]);
        }
        context.expect_eq(reads_before < disk.test_snapshot().submitted, true);
        context.diagnostic(format_args!("W07 cold-reload original-MM-OFD-inode-cache-metadata-volume-ended=1 fresh-volume-and-cache=1 real-device-reads=1 root-and-all-blocks=1 production-inode-and-tree-codec=1 format={} base={}", if traditional { "traditional" } else { "extent" }, base));
        Ok(())
    }

    /// 真正在途的原页允许普通覆盖和映射再脏，旧父完成只撤自己的 WB，下一轮写出新义务。
    /// 原页由显式有限请求准备，随后持有同一凭证观察再脏与完成。
    #[test]
    #[ktest(tags("W09", "fs-sync", "fs-shared"), min_cpus = 2)]
    fn coherent_redirty_during_original_writeback(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let offset = checked(context, file.stat())?
            .size
            .div_ceil(PAGE_SIZE as u64)
            * PAGE_SIZE as u64;
        context.expect_eq(file.write_at(offset, &[0x31; 4096]), Ok(4096));

        let inode = fixture.inodes.last().unwrap().clone();
        // 普通增长可能给旧 EOF 尾页留下合法义务，先真实同步它，再只覆盖本例新页。
        checked(context, crate::fs::writeback::sync_inode(&inode))?;
        context.expect_eq(file.write_at(offset, &[0x31; 4096]), Ok(4096));
        let volume = fixture.volume();
        context.expect_eq(
            (0..volume.dirty_inode_limit())
                .filter_map(|slot| volume.dirty_inode_at(slot))
                .any(|dirty| dirty.ptr_eq(&inode)),
            true,
        );
        let alias = shared_alias(context, &file, offset)?;
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(offset / PAGE_SIZE as u64, &mut attempt),
        )?;

        let disk = fixture.user.path.owner.disk()?.clone();
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        let syncing = inode.clone();
        let observed = fixture.consume_sync(0, move || {
            crate::fs::writeback::sync_inode_range(&syncing, offset..offset + PAGE_SIZE as u64)
        })?;
        until("W09 original WB did not block with accepted IO", || {
            folio.writeback_sectors() != 0
                && disk.test_snapshot().inflight != 0
                && observed.blocked()
        })?;
        context.expect_eq(folio.dirty_sectors(), 0);

        let writing = file.clone();
        let mapping = alias.clone();
        let producer = fixture.consume_sync(1, move || {
            if writing.write_at(offset + 1, &[0x52])? != 1 {
                return Err(FsError::InvalidOperation);
            }
            let progress = mapping.copyout(SHARED_ALIAS + 2, &[0x73]);
            if progress.copied == 1 && progress.error.is_none() {
                Ok(())
            } else {
                Err(FsError::BadAddress)
            }
        })?;
        until(
            "W09 redirty producer did not reach a determinate state",
            || producer.result.lock().is_some() || producer.blocked(),
        )?;
        context.expect_eq(*producer.result.lock(), Some(Ok(())));
        context.expect_eq(
            alias.mapping(SHARED_ALIAS).unwrap().physical,
            folio.phys_addr(),
        );
        context.expect_eq(
            (folio.dirty_sectors() != 0, folio.writeback_sectors() != 0),
            (true, true),
        );
        bytes_at(context, &file, offset, &[0x31, 0x52, 0x73])?;

        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        context.expect_eq(folio.writeback_sectors(), 0);
        context.expect_eq(folio.dirty_sectors() != 0, true);
        context.expect_eq(
            (0..volume.dirty_inode_limit())
                .filter_map(|slot| volume.dirty_inode_at(slot))
                .any(|dirty| dirty.ptr_eq(&inode)),
            true,
        );

        let first = disk.test_snapshot();
        checked(
            context,
            crate::fs::writeback::sync_inode_range(&inode, offset..offset + PAGE_SIZE as u64),
        )?;
        context.expect_eq((folio.dirty_sectors(), folio.writeback_sectors()), (0, 0));
        context.expect_eq(before.data_submitted < first.data_submitted, true);
        context.expect_eq(
            first.data_submitted < disk.test_snapshot().data_submitted,
            true,
        );
        context.expect_eq(
            (0..volume.dirty_inode_limit())
                .filter_map(|slot| volume.dirty_inode_at(slot))
                .any(|dirty| dirty.ptr_eq(&inode)),
            false,
        );
        context.diagnostic(format_args!("W09 original-folio={} original-parent-blocked=1 ordinary-and-mapped-redirty=1 old-completion-preserves-dirty=1 next-round-progress=1", folio.phys_addr()));
        Ok(())
    }

    /// 两独立 OFD 各观察一次错误，dup 共游标；没有任何观察者的新错误仍由后来 open 报告。
    /// 错误观察结束后用独立有限请求取得原页检查剩余 Dirty/WB。
    #[test]
    #[ktest(tags("W11", "fs-sync", "controlled-ledger"))]
    fn independent_open_dup_and_unseen_old_errseq(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        context.expect_eq(crate::cache::ktest_wb_error_protocol(), true);
        let first = fixture.open(context, b"/test/write-reserve", 2)?;
        let second = fixture.open(context, b"/test/write-reserve", 2)?;
        context.expect_eq(Arc::ptr_eq(&first, &second), false);
        let inode = fixture.inodes.last().unwrap().clone();
        let disk = fixture.user.path.owner.disk()?.clone();
        let table = checked(context, FileTable::try_new())?;
        let fd = checked(context, table.reserve(1024))?
            .publish(first.clone(), DescriptorFlags::default());
        let duplicate = checked(context, table.dup(fd, 1024))?;
        let duplicated = checked(context, table.lookup(duplicate))?;
        context.expect_eq(Arc::ptr_eq(&first, &duplicated), true);

        context.expect_eq(first.write_at(0, &[0x91; 4096]), Ok(4096));
        disk.test_fail_next_data(BlockError::Io);
        let error = Err(FsError::Io(BlockError::Io));
        context.expect_eq(first.sync_range(0..u64::MAX, false), error);
        context.expect_eq(disk.test_completion_error_pending(), false);
        let failed = disk.test_snapshot();
        context.expect_eq(second.sync_range(0..u64::MAX, false), error);
        context.expect_eq(duplicated.sync_range(0..u64::MAX, false), Ok(()));
        context.expect_eq(first.sync_range(0..u64::MAX, false), Ok(()));
        let seen = fixture.open(context, b"/test/write-reserve", 2)?;
        context.expect_eq(seen.sync_range(0..u64::MAX, false), Ok(()));
        context.expect_eq(disk.test_snapshot().data_submitted, failed.data_submitted);

        context.expect_eq(first.write_at(0, &[0xa2; 4096]), Ok(4096));
        disk.test_fail_next_data(BlockError::Io);
        context.expect_eq(crate::fs::writeback::sync_inode(&inode), error);
        context.expect_eq(disk.test_completion_error_pending(), false);
        let unseen = fixture.open(context, b"/test/write-reserve", 2)?;
        context.expect_eq(unseen.sync_range(0..u64::MAX, false), error);
        context.expect_eq(unseen.sync_range(0..u64::MAX, false), Ok(()));
        context.expect_eq(first.sync_range(0..u64::MAX, false), error);
        context.expect_eq(duplicated.sync_range(0..u64::MAX, false), Ok(()));
        context.expect_eq(second.sync_range(0..u64::MAX, false), error);

        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            first.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        context.expect_eq((folio.dirty_sectors(), folio.writeback_sectors()), (0, 0));
        context.diagnostic(format_args!("W11 real-used-controlled-EIO=1 two-independent-observers=1 dup-shared-cursor=1 seen-new-open-skips=1 unseen-new-open-reports=1 accepted-error-no-replay=1"));
        Ok(())
    }

    /// 页已清脏但原 WB 未完成时，普通 sys82 必须真实 Blocked，放行真实完成后才返回。
    /// 在暂停完成前以显式有限请求取得待观察的原页。
    #[test]
    #[ktest(tags("W12", "fs-sync", "user-fs"), min_cpus = 2)]
    fn fsync_waits_for_existing_clean_writeback(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        context.expect_eq(file.write_at(0, &[0x45; 4096]), Ok(4096));
        let inode = fixture.inodes.last().unwrap().clone();
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;

        let image = fixture.shared_image(context, b"b")?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        let executor = fixture
            .user
            .graph()
            .lookup_thread(fixture.user.running.as_ref().unwrap().pid)
            .and_then(|user| user.executor())
            .ok_or(TestError("W12 user executor missing"))?;

        let disk = fixture.user.path.owner.disk()?.clone();
        disk.test_suspend_completions();
        let syncing = inode.clone();
        let first = fixture.consume_sync(0, move || crate::fs::writeback::sync_inode(&syncing))?;
        until("W12 clean original page not in WB", || {
            folio.dirty_sectors() == 0
                && folio.writeback_sectors() != 0
                && disk.test_snapshot().inflight != 0
                && first.blocked()
        })?;

        WriteOwner::release(&mm, 1)?;
        until("W12 sys82 did not really block on existing WB", || {
            sched::test_state(&executor) == State::Blocked
        })?;
        context.expect_eq(mm.load_user_u32(SHARED_CONTROL), Ok(1));
        context.expect_eq(folio.dirty_sectors(), 0);
        context.expect_eq(folio.writeback_sectors() != 0, true);
        context.expect_eq(
            fixture
                .user
                .running
                .as_ref()
                .unwrap()
                .join
                .try_join()
                .is_none(),
            true,
        );

        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(*first.result.lock(), Some(Ok(())));
        fixture.phase(&mm, 0, 2)?;
        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        context.expect_eq((folio.dirty_sectors(), folio.writeback_sectors()), (0, 0));
        context.diagnostic(format_args!("W12 Dirty0-WB1=1 original-data-parent-blocked=1 ordinary-sys82-blocked=1 completion-before-return=1"));
        Ok(())
    }

    /// 数据真实完成后才消费一次元数据或 FLUSH 错误，显式后次同步和静止窗口都不得自动 replay。
    /// 错误后原页检查使用独立有限请求，不改变完成错误或重放数据。
    #[test]
    #[case("metadata", false)]
    #[case("flush", true)]
    #[ktest(tags("W13", "fs-sync", "controlled-ledger"))]
    fn data_then_metadata_flush_error_without_replay(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        flush: bool,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let disk = fixture.user.path.owner.disk()?.clone();
        if !disk.limits().flush {
            return Err(TestError("W13 requires negotiated real FLUSH"));
        }

        context.expect_eq(file.write_at(0, &[0xb3; 4096]), Ok(4096));
        let before = disk.test_snapshot();
        if flush {
            disk.test_fail_next_flush(BlockError::Io);
        } else {
            disk.test_fail_next_metadata(BlockError::Io);
        }
        context.expect_eq(
            file.sync_range(0..u64::MAX, false),
            Err(FsError::Io(BlockError::Io)),
        );
        context.expect_eq(disk.test_completion_error_pending(), false);
        let failed = disk.test_snapshot();
        context.expect_eq(before.data_submitted < failed.data_submitted, true);
        context.expect_eq(before.metadata_submitted < failed.metadata_submitted, true);
        if flush {
            context.expect_eq(failed.flush_submitted - before.flush_submitted, 1);
        }
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        context.expect_eq((folio.dirty_sectors(), folio.writeback_sectors()), (0, 0));

        disk.test_flush_completions();
        timer::sleep(time::Duration::from_millis(10));
        context.expect_eq(disk.test_snapshot().submitted, failed.submitted);

        context.expect_eq(file.sync_range(0..u64::MAX, false), Ok(()));
        let next = disk.test_snapshot();
        context.expect_eq(next.data_submitted, failed.data_submitted);
        context.expect_eq(next.metadata_submitted, failed.metadata_submitted);
        context.expect_eq(next.flush_submitted - failed.flush_submitted, 1);
        bytes_at(context, &file, 0, &[0xb3; 4096])?;
        context.diagnostic(format_args!("W13 kind={} actual-used-controlled-EIO=1 data-before-error=1 automatic-replay=0 explicit-next-sync-data-replay=0 explicit-next-sync-metadata-replay=0", if flush { "FLUSH" } else { "metadata" }));
        Ok(())
    }

    /// 小块几何核同原页的精确 Dirty 交集；原 OFD 与 VMA 偏移决定写出范围，private COW 和异步不写源页。
    /// 两页初始准备共享预算，各次同步后的身份检查分别使用独立有限请求。
    #[test]
    #[ktest(tags("W14", "fs-sync", "fs-shared", "user-fs"))]
    fn msync_ranges_private_and_ordinary_abi(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        context.expect_eq(file.write_at(0, &[0x66; 8192]), Ok(8192));
        let inode = fixture.inodes.last().unwrap().clone();
        checked(context, crate::fs::writeback::sync_inode(&inode))?;
        let mut prepare_attempt = ReclaimAttempt::new();
        let first = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(0, &mut prepare_attempt),
        )?;
        let second = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(1, &mut prepare_attempt),
        )?;

        let block = inode.block_size() as usize;
        let disk = fixture.user.path.owner.disk()?.clone();
        if block < PAGE_SIZE {
            let (first_mask, neighbor_mask) = match block {
                1024 => (0x03u8, 0x0cu8),
                2048 => (0x0fu8, 0xf0u8),
                _ => return Err(TestError("W14 unsupported small FS block geometry")),
            };
            let original = first.phys_addr();
            context.expect_eq((first.dirty_sectors(), first.writeback_sectors()), (0, 0));
            context.expect_eq(checked(context, file.write_at(64, &[0xa6; 3]))?, 3);
            context.expect_eq(
                checked(context, file.write_at(block as u64 + 64, &[0xb7; 3]))?,
                3,
            );
            context.expect_eq(
                (first.dirty_sectors(), first.writeback_sectors()),
                (first_mask | neighbor_mask, 0),
            );

            let before_range = disk.test_snapshot();
            checked(context, file.sync_range(64..67, true))?;
            context.expect_eq(
                (first.dirty_sectors(), first.writeback_sectors()),
                (neighbor_mask, 0),
            );
            let mut range_attempt = ReclaimAttempt::new();
            context.expect_eq(
                checked(
                    context,
                    file.mapping_inode()
                        .unwrap()
                        .file_page(0, &mut range_attempt),
                )?
                .phys_addr(),
                original,
            );
            context.expect_eq(
                before_range.data_submitted < disk.test_snapshot().data_submitted,
                true,
            );
            bytes_at(context, &file, 64, &[0xa6; 3])?;
            bytes_at(context, &file, block as u64 + 64, &[0xb7; 3])?;

            let before_neighbor = disk.test_snapshot();
            checked(
                context,
                file.sync_range(block as u64 + 64..block as u64 + 67, true),
            )?;
            context.expect_eq((first.dirty_sectors(), first.writeback_sectors()), (0, 0));
            let mut neighbor_attempt = ReclaimAttempt::new();
            context.expect_eq(
                checked(
                    context,
                    file.mapping_inode()
                        .unwrap()
                        .file_page(0, &mut neighbor_attempt),
                )?
                .phys_addr(),
                original,
            );
            context.expect_eq(
                before_neighbor.data_submitted < disk.test_snapshot().data_submitted,
                true,
            );
            bytes_at(context, &file, 64, &[0xa6; 3])?;
            bytes_at(context, &file, block as u64 + 64, &[0xb7; 3])?;
            bytes_at(context, &file, 29, &[0x66])?;
            context.diagnostic(format_args!("W14 small-FS-block-range block-size={} original-folio={} initial-dirty-mask={:#04x} neighbor-dirty-mask={:#04x} first-intersection-only=1 neighbor-explicit-sync=1 WB0-after-each-range=1", block, original, first_mask | neighbor_mask, neighbor_mask));
        } else {
            context.diagnostic(format_args!("W14 small-FS-block-range block-size={} branch-exercised=0 same-folio-neighbor-unavailable=1", block));
        }

        let mm = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            mm.map_file(SOURCE, 4096, RW, RW, file.clone(), 4096, false),
        )?;
        vm(
            context,
            mm.map_file(SHARED_ALIAS, 4096, RW, RW, file.clone(), 0, true),
        )?;
        context.expect_eq(mm.copyout(SOURCE + 29, &[0x77]).copied, 1);
        context.expect_eq(mm.copyout(SHARED_ALIAS + 29, &[0x88]).copied, 1);

        let before = disk.test_snapshot();
        vm(
            context,
            mm.sync_mapping_range(SOURCE..SOURCE + 4096, false, true),
        )?;
        context.expect_eq(disk.test_snapshot().submitted, before.submitted);
        context.expect_eq(mm.mapping(SOURCE).unwrap().permissions.write, true);
        vm(
            context,
            mm.sync_mapping_range(SHARED_ALIAS..SHARED_ALIAS + 4096, true, false),
        )?;
        context.expect_eq(disk.test_snapshot().submitted, before.submitted);
        context.expect_eq(second.dirty_sectors() != 0, true);
        context.expect_eq(first.dirty_sectors(), 0);

        context.expect_eq(
            mm.sync_mapping_range(SOURCE..SOURCE + 8192, true, false),
            Err(VmError::Unmapped),
        );
        context.expect_eq((second.dirty_sectors(), second.writeback_sectors()), (0, 0));
        context.expect_eq(
            before.data_submitted < disk.test_snapshot().data_submitted,
            true,
        );
        bytes_at(context, &file, 4096 + 29, &[0x77])?;
        bytes_at(context, &file, 29, &[0x66])?;
        mapped_bytes(context, &mm, SHARED_ALIAS + 29, &[0x88]);
        drop(mm);

        let image = fixture.shared_image(context, b"y")?;
        let user_mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&user_mm, 0, 1)?;
        let no_io = disk.test_snapshot();
        WriteOwner::release(&user_mm, 1)?;
        fixture.phase(&user_mm, 0, 2)?;
        context.expect_eq(disk.test_snapshot().submitted, no_io.submitted);
        context.expect_eq(second.dirty_sectors() != 0, true);

        WriteOwner::release(&user_mm, 2)?;
        fixture.finish_shared(context)?;
        context.diagnostic(format_args!("W14 nonzero-file-offset=1 range-only-page=1 private-and-async-no-IO=1 invalidate-no-purge=1 hole-ENOMEM-after-progress=1 ordinary-sys82-83-227-flags-and-range=1"));
        Ok(())
    }

    /// 同步 write 提交后释放 inode/cursor 锁，受控 EIO 不回滚接受的字节和位置。
    /// 后续 sync 完成真实 FLUSH，两个独立 OFD 分别消费各自的 errseq 样本。
    #[test]
    #[case("O_SYNC", b"d" as &[u8])]
    #[case("O_DSYNC", b"z" as &[u8])]
    #[ktest(tags("W14", "fs-sync", "user-fs"), min_cpus = 2)]
    fn synchronous_write_releases_inode_and_cursor_locks(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        mode: &[u8],
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        context.expect_eq(file.write_at(0, &[0x22; 4096]), Ok(4096));
        let inode = fixture.inodes.last().unwrap().clone();
        checked(context, file.sync_range(0..u64::MAX, false))?;

        let image = fixture.shared_image(context, mode)?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        let user = fixture
            .user
            .graph()
            .lookup_thread(fixture.user.running.as_ref().unwrap().pid)
            .ok_or(TestError("sync writer user missing"))?;
        let executor = user
            .executor()
            .ok_or(TestError("sync writer executor missing"))?;
        let user_fd = mm
            .read_usize(SHARED_CONTROL + 16)
            .map_err(|_| TestError("sync writer fd missing"))?;
        let original = checked(
            context,
            user.runtime()
                .ok_or(TestError("sync writer runtime missing"))?
                .files
                .lookup(user_fd),
        )?;
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;

        let disk = fixture.user.path.owner.disk()?.clone();
        disk.test_suspend_completions();
        disk.test_fail_next_data(BlockError::Io);
        WriteOwner::release(&mm, 1)?;
        until("O_SYNC write did not block in actual accepted WB", || {
            folio.writeback_sectors() != 0
                && disk.test_snapshot().inflight != 0
                && sched::test_state(&executor) == State::Blocked
        })?;

        let cursor = original.clone();
        let writing = file.clone();
        let unlocked = fixture.consume_sync(0, move || {
            if cursor.seek(SeekFrom::Current(0))? != 1 {
                return Err(FsError::InvalidOperation);
            }
            if writing.write_at(1, &[0x33])? != 1 {
                return Err(FsError::InvalidOperation);
            }
            Ok(())
        })?;
        until(
            "sync write lock probe did not reach a determinate state",
            || unlocked.result.lock().is_some() || unlocked.blocked(),
        )?;
        context.expect_eq(*unlocked.result.lock(), Some(Ok(())));
        context.expect_eq(folio.dirty_sectors() != 0, true);

        disk.test_resume_completions();
        fixture.join_consumers()?;
        fixture.phase(&mm, 0, 2)?;
        context.expect_eq(disk.test_completion_error_pending(), false);
        bytes_at(context, &file, 0, &[0x39, 0x33])?;
        context.expect_eq(original.seek(SeekFrom::Current(0)), Ok(1));

        if !disk.limits().flush {
            return Err(TestError("sync error probe requires device FLUSH"));
        }
        let before_sync = disk.test_snapshot();
        disk.test_fail_next_flush(BlockError::Io);
        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        context.expect_eq(disk.test_completion_error_pending(), false);
        context.expect_eq(
            before_sync.flush_submitted < disk.test_snapshot().flush_submitted,
            true,
        );
        checked(context, crate::fs::writeback::sync_inode(&inode))?;
        context.diagnostic(format_args!("W14 mode={} actual-sync-write-blocked=1 inode-lock-released=1 OFD-cursor-lock-released=1 accepted-data-and-cursor-not-rolled-back-on-EIO=1", if mode == b"d" { "O_SYNC" } else { "O_DSYNC" }));
        Ok(())
    }
}
