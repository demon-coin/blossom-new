//! 页面缓存批量回写、冷挂载持久化校验、并发重脏（Redirty）、错误序列（errseq）与同步系统调用测试。
//!
//! ## 模块职责与架构定位
//! 本模块属于 `fs::ktest::user::write` 测试套件，负责验证页面缓存脏页回写（Writeback）与用户态同步接口
//! （`fsync`、`fdatasync`、`msync`、`O_SYNC`、`O_DSYNC`）的端到端持久性及并发一致性契约。
//! 上游由 `ktest` 测试运行器调用，下游覆盖 `writeback::sync_inode_range`、`File::sync_range`、
//! `Mm::sync_mapping_range`、`InodeCache` 冷卸载与重挂载以及块设备故障注入机制。
//!
//! ## 核心机制与状态机流转
//!
//! ### 1. 物理页（Folio）扇区级 Dirty / Writeback 位图状态机
//! ```text
//! [普通写入 write_at / 共享映射写 copyout]
//!         │ (置位对应块/扇区的 dirty_sectors，保持 writeback_sectors 不变)
//!         ▼
//!   Dirty != 0, WB == 0
//!         │
//!         ├─► [启动回写: 清理 dirty_sectors 并转置为 writeback_sectors，提交 Bio 至块设备]
//!         │         │
//!         │         ├── (回写 I/O 在途期间未发生新写入)
//!         │         │         └──► [Bio 完成回调: 清除 writeback_sectors] ──► Dirty == 0, WB == 0 (干净页)
//!         │         │
//!         │         └── (回写 I/O 在途期间发生并发覆盖写或共享映射写: Redirty)
//!         │                   └──► Dirty != 0, WB != 0
//!         │                             └──► [旧 Bio 完成回调: 仅清除本次提交的 writeback_sectors，保留 dirty_sectors]
//!         │                                       └──► Dirty != 0, WB == 0 (等待下一轮回写刷盘)
//!         │
//!         └─► [fsync 等待契约]: 即使 Dirty == 0，只要 WB != 0（在途回写未完成），
//!                               后续 fsync 仍必须阻塞等待在途 Bio 彻底完成后才可返回。
//! ```
//!
//! ### 2. 写回错误序列（Writeback Error Sequence / `errseq`）游标推进
//! ```text
//! Inode 共享错误状态字: [ 新错误发生 (Data / Metadata / Flush EIO) ──► 递增错误序号 ]
//!    ├── OpenFileDescription A ──► 首次调用 sync_range 观察到序号落后 ──► 推进游标并返回 EIO (再次调用若无新错则返回 Ok)
//!    ├── dup(A) 共享同一 OFD   ──► 与 A 共享同一游标，A 消费错误后 dup 句柄再次调用直接返回 Ok
//!    ├── OpenFileDescription B ──► 独立游标，首次调用 sync_range 同样观察并消费一次该错误返回 EIO
//!    └── 错误未被任何 OFD 观察 ──► 标记为“未见错误”，后续新 open 的文件句柄首次 sync 仍会捕获并报告该历史错误
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 1. **同步写入（`O_SYNC` / `O_DSYNC`）锁释放契约**：`write` 将数据拷贝进页面缓存并更新文件偏移游标后，
//!    必须在进入回写 I/O 阻塞等待之前**释放 Inode 内容锁与文件偏移游标（Cursor）锁**，保证回写等待期间不阻塞并发 `lseek` 或同行无冲突写入。
//! 2. **冷重开（Cold Reopen）生命周期契约**：验证落盘持久性时，必须先释放全部用户进程图、VFS 挂载视图、目录项与 `InodeCache` 强引用，
//!    并等待缓存退役监视器（`wait_retired`）与在途磁盘 I/O 归零后，才能在同一块设备上以新 `InodeCache` 只读重挂载。

use super::*;

impl UserOwner {
    /// 运行用户态回写测试程序（写入模式）后彻底释放旧文件系统与缓存实例，随后在同一块设备上以只读模式冷挂载并运行校验程序，验证文件内容及内联短符号链接（Fast Symlink）的磁盘格式。
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

        // Step 1: 释放目录树、进程图与路径视图后，取出最后一个 InodeCache 强引用并等待其异步退役完成，确保没有任何残留的内存缓存对象。
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

        // Step 2: 在确认全部旧强引用与在途磁盘 I/O 归零后，创建全新的 InodeCache 并以只读模式重新挂载同一块设备。
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

        // Step 3: 启动用户态校验程序（只读模式）从磁盘冷读验证文件全文，并直接检查内联短符号链接的磁盘元数据标志。
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

    /// 测试目标：验证当 16 个页面缓存页对应的磁盘物理块因碎片隔离而不连续时，批量回写（`sync_inode_range`）会拆分为多个父 Bio 请求，并在进入完成等待之前将所有已接受的请求提交并通知块设备队列。
    /// 场景步骤：
    /// 1. 校验文件末尾后方 16 页区间全部为磁盘空洞（Hole），写入第 0 页使其分配首个物理块 `first`，随后通过块分配器预占紧随 `first` 之后的连续物理块 `target`，迫使后续写入的第 1 页分配到非连续物理块 `second`。
    /// 2. 释放预占的未发布隔离块，继续写满剩余 14 页数据，挂起块设备完成通知后在后台线程启动 16 页范围同步。
    /// 3. 断言在同步线程阻塞等待完成期间，块设备上已有多个在途请求（`inflight > 1`）且已触发队列通知（Kick）；恢复设备完成通知后验证全部 16 页清脏、落盘内容一致且缓存物理页地址未变。
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
                // Step 1: 临时将本地元数据副本的 size 扩展至 `end` 以绕过 EOF 截断检查（不修改 Inode 正式大小或块树），确认目标逻辑块范围确为空洞。
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
            // Step 2: 在分配第 1 页的数据块前，先检查并预占第 0 页末尾紧邻的物理后继块 `target`，强制制造物理块不连续以触发多 Bio 批次拆分。
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
                // 先为预留容器扩容再申请新块；同块组内编号更低的空闲块令牌一并暂存，直到成功占住目标后继块。
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
                    // 仅在首次实际分配初始化了未知块组位图后重新查询目标块状态，避免将未初始化块组（BLOCK_UNINIT）误判为已占用。
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

            // Step 3: 依次写入第 1 至第 15 页；在第 1 页完成非连续物理块映射后立即归还所有临时占用的隔离块令牌。
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

        // 若上述准备过程因错误提前中断，在此兜底归还所有尚未释放的临时块令牌。
        for separator in held.drain(..) {
            volume.release_unpublished_block(separator);
        }
        let (first, target, second, separators) = prepared?;
        context.expect_eq(first + PAGE_SIZE as u64 / block != second, true);

        // Step 4: 挂起块设备完成中断并发起 16 页批量回写，验证在等待首个 Bio 完成前已向设备提交并通知了多个非连续物理段请求。
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

    /// 测试目标：验证在完成文件数据、元数据与设备 FLUSH 同步后，释放全部 VMA、打开文件、Inode、缓存与卷强引用，从磁盘冷重开能完整还原块映射树、CRC 校验与全部落盘数据。
    /// 场景步骤：
    /// 1. 在文件末尾后的 5 个跨度逻辑块偏移处写入特征字节（覆盖传统间接块或 Extent 树的深层节点），调用 `sync_range` 触发数据、元数据写回及设备 FLUSH，并建立一页只读文件映射。
    /// 2. 调用 `cold_reopen` 销毁地址空间、文件句柄、Inode、元数据缓存与卷实例，确认所有弱引用计数归零后重新挂载该磁盘。
    /// 3. 校验重新加载的 Inode 统计信息、块映射树结构（深度、Extent 数量、节点与物理块分布）以及各逻辑偏移处读回的字节内容与同步前完全一致。
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

        // Step 1: 执行全范围同步，确认数据写、元数据写与设备 FLUSH 均已下发，并记录同步后的 Inode 元数据与块树根快照。
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

        // Step 2: 释放全部内存所有者并冷重开卷，校验新加载的 Inode 元数据、块映射树及各逻辑块数据与落盘前完全相同。
        let (reopened, reads_before) = fixture.cold_reopen(context, mm, file, inode, volume)?;
        let reloaded_inode = fixture.inodes.last().unwrap().clone();
        let mut reloaded_stat = checked(context, reopened.stat())?;
        context.expect_eq(reloaded_stat.device != stat.device, true);
        // 重新挂载的卷实例会分配新的内核虚拟设备号，其余磁盘持久化属性必须逐项相等。
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

    /// 测试目标：验证正处于在途回写（Writeback）状态的物理页允许并发的 `write_at` 覆盖写与 `mmap` 共享映射写将其重新标脏（Redirty），且当旧回写请求完成时仅清除 `writeback` 位而保留新产生的 `dirty` 位，由下一轮同步刷出新数据。
    /// 场景步骤：
    /// 1. 写入并同步一页数据后再次写脏该页，建立 `SHARED_ALIAS` 共享可写映射，挂起块设备完成通知并在后台启动第 1 轮范围回写。
    /// 2. 等待该页进入 `dirty == 0 && writeback != 0` 的在途回写阻塞状态，随后在另一个线程中对该页执行 `write_at` 与 `copyout` 共享映射写入。
    /// 3. 断言并发写入无需等待旧回写完成即可返回，且物理页变为 `dirty != 0 && writeback != 0`；恢复块设备完成后验证旧回写仅清除 `writeback` 位而保留 `dirty` 位及卷级脏 Inode 登记，最后执行第 2 轮同步将新数据彻底刷盘。
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
        // Step 1: 文件扩展可能会使原先的 EOF 尾页变脏，先将其同步干净，随后单独写脏目标测试页并建立共享映射。
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

        // Step 2: 挂起块设备完成通知并发起第 1 轮回写，等待目标页清除 dirty 位并进入 writeback 在途状态。
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

        // Step 3: 在回写 I/O 在途期间并发执行普通写入（write_at）与共享内存映射写入（copyout），验证缓存页在保持物理地址不变的同时重新置位 dirty 标志。
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

        // Step 4: 恢复块设备完成通知使第 1 轮回写结束，验证仅 writeback 位清零而 dirty 位与脏 Inode 队列保留，最后通过第 2 轮回写刷出新修改。
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

    /// 测试目标：验证独立打开的文件描述（OFD）各自独立消费一次回写错误序号（`errseq`），`dup` 复制的描述符共享同一错误游标，且在错误发生时尚未打开文件的后续 `open` 能否正确按“已见/未见”规则报告错误。
    /// 场景步骤：
    /// 1. 对同一文件分别调用两次 `open` 得到独立句柄 `first` 与 `second`，并对 `first` 执行 `dup` 得到共享同一 OFD 的句柄 `duplicated`。
    /// 2. 写入脏数据并注入一次数据块写 EIO：`first.sync_range` 与 `second.sync_range` 均返回一次 `EIO`，而共享游标的 `duplicated`、重试的 `first` 以及错误已被观察后新打开的 `seen` 句柄调用 `sync_range` 均返回 `Ok(())` 且不触发多余写盘。
    /// 3. 再次写入脏数据并注入一次数据块写 EIO，通过内核内部 `sync_inode` 触发回写失败（此时没有任何 OFD 消费该错误）；验证随后新打开的 `unseen` 句柄首次 `sync_range` 能捕获该未见错误并返回 `EIO`，且 `first` 与 `second` 也各自观测到这一次新错误。
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

        // Step 1: 注入第 1 次数据回写 EIO，验证两个独立 OFD 各报告一次错误、dup 句柄共享游标不重复报错，且错误被消费后新 open 的句柄不再报错。
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

        // Step 2: 注入第 2 次数据回写 EIO 并由不绑定 OFD 的内部接口触发，验证没有任何观察者消费该错误时，后续新 open 的句柄首次同步仍能捕获该错误。
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

    /// 测试目标：验证当文件页已清除 `dirty` 位但前序回写 I/O 仍在途（`writeback != 0`）时，用户态调用 `fsync`（系统调用号 82）必须阻塞等待该在途回写完成后才能返回。
    /// 场景步骤：
    /// 1. 写入 1 页数据并预先获取其 `FolioRef`，启动用户态测试进程进入第一阶段就绪点。
    /// 2. 挂起块设备完成通知，在后台线程发起 `sync_inode` 使该页进入 `dirty == 0 && writeback != 0` 的在途回写状态。
    /// 3. 放行用户态进程执行 `fsync`，断言用户态线程进入 `State::Blocked` 阻塞状态且未提前推进阶段号；随后恢复块设备完成通知，验证前序回写与用户态 `fsync` 共同完成且页状态归零。
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

        // Step 1: 挂起块设备完成中断并启动后台回写，使目标页处于已清脏（dirty == 0）但回写仍在途（writeback != 0）的状态。
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

        // Step 2: 唤醒用户态进程调用 sys_fsync (82)，确认其不会因为 dirty == 0 而直接跳过等待，而是阻塞在在途的 writeback 上。
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

        // Step 3: 恢复块设备完成通知，确认后台回写与用户态 fsync 相继完成返回。
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

    /// 测试目标：验证当文件数据回写已成功完成、仅后续的元数据写回或设备 FLUSH 发生 I/O 错误时，`sync_range` 准确报告一次 `EIO`，且后台工作队列或后续显式同步均不会重复回写已经落盘的数据或无新修改的元数据。
    /// 场景步骤：
    /// 1. 写入 1 页数据，根据用例参数注入下一次元数据写 EIO 或下一次设备 FLUSH EIO，调用 `file.sync_range` 并断言返回 `EIO`。
    /// 2. 校验此时数据与元数据请求均已提交且页面缓存页的 `dirty` 与 `writeback` 位均已清零；休眠等待 10ms，确认后台不会自动发起任何重放 I/O。
    /// 3. 再次显式调用 `file.sync_range`，断言返回 `Ok(())`，且期间数据写与元数据写提交计数保持不变（仅重新发出一次 FLUSH），落盘数据完好可读。
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

        // Step 1: 写入一页数据并注入受控的元数据或 FLUSH 错误，验证首次同步消费该错误且数据页已完成清脏。
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

        // Step 2: 排空完成队列并静置一段时间，确认不会触发后台自动重放，且再次显式同步不会重传已落盘的数据与元数据。
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

    /// 测试目标：验证小于页尺寸的块几何下按字节区间同步仅刷写相交的脏块扇区，并验证 `msync` 对文件偏移映射、私有写时复制（COW）映射、`MS_ASYNC`、`MS_INVALIDATE` 以及系统调用参数边界的处理契约。
    /// 场景步骤：
    /// 1. 若文件系统块大小小于页大小（1024B 或 2048B），在同一物理页的第 0 块与第 1 块各写入 3 字节，先仅同步第 0 块区间，断言第 0 块脏扇区位清零而同页相邻的第 1 块脏扇区位保留，再同步第 1 块区间使其彻底清脏。
    /// 2. 建立文件第 2 页（偏移 4096）的可写共享映射 `SOURCE` 与第 1 页（偏移 0）的可写私有映射 `SHARED_ALIAS`，分别写入 1 字节；验证对共享映射执行 `MS_ASYNC | MS_INVALIDATE` 及对私有 COW 映射执行 `MS_SYNC` 均不产生磁盘 I/O。
    /// 3. 对横跨已映射页与未映射区的区间调用 `sync_mapping_range(SOURCE..SOURCE + 8192, true, false)`，断言在同步完已映射的前半段脏页后返回 `VmError::Unmapped`；最后运行用户态测试程序验证 `fsync`/`fdatasync`/`msync` 系统调用的参数与标志位校验。
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

        // Step 1: 当文件系统块小于 4KiB 页大小时，验证同一物理页内不同文件块的精细脏扇区掩码交集同步。
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

        // Step 2: 验证非零文件偏移的共享映射、私有 COW 映射、MS_ASYNC / MS_INVALIDATE 标志以及跨未映射区间的部分同步行为。
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

        // Step 3: 启动用户态进程执行 sys_fsync / sys_fdatasync / sys_msync 的 ABI 标志与范围边界检查。
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

    /// 测试目标：验证带有 `O_SYNC` 或 `O_DSYNC` 标志的同步 `write` 在等待底层回写 I/O 完成期间已释放 Inode 锁与文件偏移游标锁，且当底层回写遭遇 `EIO` 时不会回滚已写入页面缓存的数据和已推进的文件偏移游标。
    /// 场景步骤：
    /// 1. 初始化 1 页文件并同步，启动用户态测试进程以 `O_SYNC` 或 `O_DSYNC` 打开文件并进入第一阶段就绪点。
    /// 2. 挂起块设备完成通知并注入一次数据写 `EIO`，放行用户态进程执行 1 字节同步 `write`，等待其阻塞在在途回写 I/O 上。
    /// 3. 在同步 `write` 阻塞期间，从内核辅助线程对同一打开文件句柄调用 `seek(SeekFrom::Current(0))` 并向同文件偏移 1 处执行 `write_at`，断言两者均不阻塞且成功完成（证明游标锁与 Inode 锁已在等待 I/O 前释放）；随后恢复设备完成通知，验证即使同步写返回 `EIO`，已写入缓存的字节与推进到 1 的文件游标均保留不回滚。
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

        // Step 1: 挂起块设备完成中断并预设数据写 EIO，放行用户态进程执行 O_SYNC/O_DSYNC write 使其阻塞在回写等待阶段。
        let disk = fixture.user.path.owner.disk()?.clone();
        disk.test_suspend_completions();
        disk.test_fail_next_data(BlockError::Io);
        WriteOwner::release(&mm, 1)?;
        until("O_SYNC write did not block in actual accepted WB", || {
            folio.writeback_sectors() != 0
                && disk.test_snapshot().inflight != 0
                && sched::test_state(&executor) == State::Blocked
        })?;

        // Step 2: 在用户态同步写阻塞等待 I/O 期间，并发探测同一 OFD 的游标位置（seek）并向同一 Inode 写入相邻字节，确认游标锁与 Inode 锁均已释放。
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

        // Step 3: 恢复块设备完成中断以触发受控 EIO，确认页面缓存中已接受的新数据与 OFD 游标位置均未被回滚，随后验证后续显式同步的 FLUSH 错误传递。
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
