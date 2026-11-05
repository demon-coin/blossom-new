//! 原生任务上的 inode 加载、同页等待、不同页并行与原格式错误唤醒。

use crate::fs::ktest::*;

/// 唯一 root 的真实 used.idx 前进才构成设备完成证据，不制造 used 元素。
fn used(fixture: &Owner, target: u16) -> TestResult {
    let disk = fixture.disk()?;
    let legacy = matches!(
        disk.identity(),
        DiskIdentity::Virtio(VirtioIdentity::Mmio { version: 1, .. })
    );
    until("filesystem real used did not progress", || {
        block::tests::used_index(disk, legacy) == target
    })
}

/// 当前仍由 fixture 保活的真实硬件 ring 只读标量。
fn used_index(fixture: &Owner) -> Result<u16, TestError> {
    let disk = fixture.disk()?;
    let legacy = matches!(
        disk.identity(),
        DiskIdentity::Virtio(VirtioIdentity::Mmio { version: 1, .. })
    );
    Ok(block::tests::used_index(disk, legacy))
}

/// inode number 从真实目录获取，允许目标 inode CRC 故意坏而目录仍有效。
fn file_number(
    context: &TestContext<'_>,
    fixture: &Owner,
    volume: &Arc<Ext4Volume>,
) -> Result<u64, TestError> {
    let root = checked(
        context,
        fixture
            .cache()
            .iget(volume, crate::fs::ext4::codec::ROOT_INODE),
    )?;
    checked(
        context,
        root.with_map_read(|metadata, block| volume.lookup(metadata, block, b"pattern")),
    )?
    .ok_or(TestError("concurrency pattern directory entry missing"))
}

/// 两个真实 WaitQueue 成员登记后，才允许唯一加载者离开已发表 Loading 停点。
fn inode_flight(context: &mut TestContext<'_>, fixture: &mut Owner, failure: bool) -> TestResult {
    let volume = fixture.open(context)?;
    let number = file_number(context, fixture, &volume)?;
    let before_len = fixture.cache().len();
    let before_load = fixture.cache().ktest_load_count();
    fixture
        .observer()
        .hold_loader
        .store(true, Ordering::Release);

    for index in 0..3 {
        fixture.publish(
            index,
            Job::Get {
                cache: fixture.cache().clone(),
                volume: volume.clone(),
                number,
            },
        );
        if index == 0 {
            until("inode loader did not reach published Loading", || {
                fixture.observer().loader_entered.load(Ordering::Acquire) == 1
                    && fixture.observer().blocked(0)
            })?;
        }
    }
    until("inode same-key waiters did not register", || {
        fixture.cache().ktest_waiting(&volume, number) == 2
            && fixture.observer().blocked(1)
            && fixture.observer().blocked(2)
    })?;
    context.expect_eq(fixture.cache().ktest_load_count(), before_load + 1);

    fixture
        .observer()
        .hold_loader
        .store(false, Ordering::Release);
    fixture.observer().gate.complete();
    fixture.join_tasks()?;

    let results = core::mem::replace(&mut *fixture.observer().results.lock(), [const { None }; 3]);
    if failure {
        for result in results {
            context.expect_eq(result.and_then(Result::err), Some(FsError::Checksum));
        }
        context.expect_eq(fixture.cache().len(), before_len);
        context.expect_eq(fixture.cache().ktest_waiting(&volume, number), 0);
        context.expect_eq(
            fixture.cache().iget(&volume, number).err(),
            Some(FsError::Checksum),
        );
        context.expect_eq(fixture.cache().ktest_load_count(), before_load + 2);
        context.expect_eq(fixture.cache().len(), before_len);
    } else {
        let mut original: Option<InodeRef> = None;
        for result in results {
            let returned = checked(context, result.ok_or(TestError("inode result absent"))?)?;
            let Returned::Inode(inode) = returned else {
                return Err(TestError("inode producer returned wrong kind"));
            };
            context.expect_eq(inode.number(), number);
            context.expect_eq(
                checked(context, inode.metadata_snapshot())?.size,
                PATTERN_SIZE,
            );
            if let Some(original) = &original {
                context.expect_eq(original.ptr_eq(&inode), true);
                context.expect_eq(core::ptr::eq(original.mapping(), inode.mapping()), true);
            } else {
                original = Some(inode);
            }
        }
        context.expect_eq(fixture.cache().ktest_load_count(), before_load + 1);
        context.expect_eq(fixture.cache().ktest_waiting(&volume, number), 0);
    }
    context.diagnostic(format_args!("inode-singleflight registered-waiters=2 loading-winners=1 error={:?} failed-placeholder-removed={} hardware-EIO-covered=0", if failure { Some(FsError::Checksum) } else { None }, failure));
    Ok(())
}

/// 每个等待者读取完整图样，同页引用保持同 PFN，不从控制槽伪造内容。
fn pages(context: &mut TestContext<'_>, fixture: &Owner, count: usize, same: bool) -> TestResult {
    let results = core::mem::replace(&mut *fixture.observer().results.lock(), [const { None }; 3]);
    let mut address = None;
    for (index, result) in IntoIterator::into_iter(results).enumerate().take(count) {
        let returned = checked(
            context,
            result.ok_or(TestError("file producer result absent"))?,
        )?;
        let Returned::Page {
            folio,
            bytes,
            valid,
        } = returned
        else {
            return Err(TestError("file producer returned wrong kind"));
        };
        context.expect_eq(bytes, mm::PAGE_SIZE);
        context.expect_eq(valid, true);
        context.expect_eq(folio.is_ready(), true);
        if let Some(address) = address {
            context.expect_eq(folio.phys_addr() == address, same);
        } else {
            address = Some(folio.phys_addr());
        }
        let offset = if same {
            0
        } else {
            index as u64 * mm::PAGE_SIZE as u64
        };
        let mut data = [0xa7; mm::PAGE_SIZE];
        checked(context, folio.copy_to(0, &mut data).map_err(FsError::from))?;
        context.expect_eq(patterned(&data, offset), true);
    }
    Ok(())
}

/// 在首个真实完成尚未消费时形成同页等待或两页在途，放 used 后取得实际数据。
fn file_flight(context: &mut TestContext<'_>, fixture: &mut Owner, same: bool) -> TestResult {
    let volume = fixture.open(context)?;
    let inode = fixture.file(context, &volume, b"pattern")?;
    let count = if same { 3 } else { 2 };
    let before = fixture.disk()?.test_snapshot();
    let first_used = used_index(fixture)?;
    fixture.disk()?.test_suspend_completions();

    fixture.publish(
        0,
        Job::Read {
            inode: inode.clone(),
            page: 0,
        },
    );
    used(fixture, first_used.wrapping_add(1))?;
    until("file first producer did not block on real IO", || {
        fixture.observer().blocked(0)
    })?;
    for index in 1..count {
        fixture.publish(
            index,
            Job::Read {
                inode: inode.clone(),
                page: if same { 0 } else { index as u64 },
            },
        );
    }
    if !same {
        used(fixture, first_used.wrapping_add(2))?;
    }
    until("file followers did not Block", || {
        (1..count).all(|index| fixture.observer().blocked(index))
    })?;

    context.expect_eq(
        fixture.disk()?.test_snapshot().submitted - before.submitted,
        if same { 1 } else { 2 },
    );
    context.expect_eq(
        fixture.disk()?.test_snapshot().inflight,
        if same { 1 } else { 2 },
    );
    let first_folio = inode
        .mapping()
        .cached_folio(0)
        .ok_or(TestError("inflight mapping disappeared"))?;
    context.expect_eq(first_folio.is_ready(), false);
    context.expect_eq(first_folio.range_ready(0, 4096), Err(BlockError::NotReady));
    let address = first_folio.phys_addr();

    fixture.disk()?.test_resume_completions();
    fixture.join_tasks()?;
    fixture.disk()?.test_flush_completions();

    let pages_loaded = if same { 1 } else { 2 };
    context.expect_eq(
        fixture.disk()?.test_snapshot().submitted - before.submitted,
        pages_loaded * mm::PAGE_SIZE / volume.geometry().block_size,
    );
    pages(context, fixture, count, same)?;
    let ready = inode
        .mapping()
        .cached_folio(0)
        .ok_or(TestError("completed mapping disappeared"))?;
    context.expect_eq(ready.phys_addr(), address);
    let mut data = [0xa7; mm::PAGE_SIZE];
    checked(context, ready.copy_to(0, &mut data).map_err(FsError::from))?;
    context.expect_eq(patterned(&data, 0), true);
    context.diagnostic(format_args!("file-singleflight same-page={} Blocked-consumers={} simultaneous-inflight={} original-PFN={} real-used-resumed=1 hardware-EIO-covered=0", same, count, if same { 1 } else { 2 }, address));
    Ok(())
}

/// 页 loader 在真实格式解析前受控停住，等待者须收到同原错误并撤掉真实等待节点。
fn page_failure(context: &mut TestContext<'_>, fixture: &mut Owner, checksum: bool) -> TestResult {
    let volume = fixture.open(context)?;
    let inode = fixture.file(
        context,
        &volume,
        if checksum { b"deep" } else { b"pattern" },
    )?;
    let page = if checksum { 0 } else { 3 };
    fixture
        .observer()
        .hold_loader
        .store(true, Ordering::Release);

    fixture.publish(
        0,
        Job::Fill {
            inode: inode.clone(),
            page,
        },
    );
    until("format error unique page loader did not pause", || {
        fixture.observer().loader_entered.load(Ordering::Acquire) == 1
    })?;
    let folio = inode
        .mapping()
        .cached_folio(page)
        .ok_or(TestError("error page owner disappeared"))?;
    for index in 1..3 {
        fixture.publish(
            index,
            Job::Read {
                inode: inode.clone(),
                page,
            },
        );
    }
    until("format error page followers did not Block", || {
        fixture.observer().blocked(1) && fixture.observer().blocked(2)
    })?;
    context.expect_eq(folio.is_ready(), false);

    fixture
        .observer()
        .hold_loader
        .store(false, Ordering::Release);
    fixture.observer().gate.complete();
    fixture.join_tasks()?;

    let expected = if checksum {
        FsError::Checksum
    } else {
        FsError::Corrupt
    };
    let results = core::mem::replace(&mut *fixture.observer().results.lock(), [const { None }; 3]);
    for result in results {
        context.expect_eq(result.and_then(Result::err), Some(expected));
    }
    context.expect_eq(folio.wait_ready(), Err(expected.folio_error()));
    context.expect_eq(folio.range_ready(0, 4096), Err(expected.folio_error()));

    drop(folio);
    context.expect_eq(inode.mapping().prune_unused(), 1);
    context.expect_eq(inode.mapping().len(), 0);
    let mut bytes = [0xa7; 4096];
    context.expect_eq(
        inode.read_at(page * mm::PAGE_SIZE as u64, &mut bytes).err(),
        Some(expected),
    );
    context.diagnostic(format_args!("file-format-error file-page={} waiters=2 all-error={:?} clean-failed-page-pruned=1 hardware-EIO-covered=0", page, expected));
    Ok(())
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-inode-loading"), resources("root"), timeout_ms = 30000)]
pub(crate) mod inode_loading {
    use super::*;
    /// 同 inode 三个普通任务取得唯一 Live 身份与唯一 FileMapping。
    #[test]
    #[ktest(tags("B14"))]
    fn registered_waiters_share_inode(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        inode_flight(context, fixture, false)
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-file-loading"), resources("root"), timeout_ms = 30000)]
pub(crate) mod file_loading {
    use super::*;

    /// 一个文件页只有一个 loading owner，两个真实等待者消费同原 PFN。
    #[test]
    #[ktest(tags("B14"))]
    fn same_page_unique_fill(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        file_flight(context, fixture, true)
    }

    /// 不同文件页在真实设备上同时保持两个在途请求，不能用全 inode 读锁串行。
    #[test]
    #[ktest(tags("B15"))]
    fn different_pages_two_inflight(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        file_flight(context, fixture, false)
    }

    /// 首段真实收权后仍 Locked；后续 hole 与非连续 Mapped 块在同原 PFN 上完成。
    #[test]
    #[ktest(tags("B13"))]
    fn segmented_page_stays_unreadable_until_all_blocks(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        let inode = fixture.file(context, &volume, b"deep")?;
        let bs = volume.geometry().block_size;
        let observer = fixture.observer().clone();
        let hook: Arc<dyn Fn(u64, usize) + Send + Sync> = Arc::try_new(move |page, end| {
            if page == 0 && end == bs && !observer.cancelled.load(Ordering::Acquire) {
                observer.loader_entered.fetch_add(1, Ordering::AcqRel);
                observer.gate.wait();
            }
        })
        .map_err(|_| TestError("segment observer OOM"))?;
        volume.ktest_set_segment_hook(Some(hook));

        fixture.publish(
            0,
            Job::Fill {
                inode: inode.clone(),
                page: 0,
            },
        );
        until("segment did not return original read from real IO", || {
            fixture.observer().loader_entered.load(Ordering::Acquire) == 1
        })?;

        let folio = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("segmented page disappeared"))?;
        let address = folio.phys_addr();
        let same = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("segmented original page disappeared"))?;
        context.expect_eq(same.phys_addr(), address);
        context.expect_eq(folio.is_ready(), false);
        context.expect_eq(folio.range_ready(0, 4096), Err(BlockError::NotReady));
        context.expect_eq(fixture.disk()?.test_snapshot().inflight, 0);
        let first_dma = fixture
            .disk()?
            .ktest_last_folio_dma()
            .ok_or(TestError("first real folio SG observation missing"))?;
        context.expect_eq(first_dma, (address, address as u64, 0, bs));

        fixture.observer().gate.complete();
        fixture.join_tasks()?;
        volume.ktest_set_segment_hook(None);

        context.expect_eq(folio.phys_addr(), address);
        context.expect_eq(folio.is_ready(), true);
        let last_offset = ((mm::PAGE_SIZE / bs - 1) / 2) * 2 * bs;
        let last_dma = fixture
            .disk()?
            .ktest_last_folio_dma()
            .ok_or(TestError("last real folio SG observation missing"))?;
        context.expect_eq(
            last_dma,
            (address, (address + last_offset) as u64, last_offset, bs),
        );
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        checked(context, folio.copy_to(0, &mut bytes).map_err(FsError::from))?;
        context.expect_eq(
            bytes.chunks(bs).enumerate().all(|(logical, block)| {
                if logical % 2 == 0 {
                    patterned(block, (logical * bs) as u64)
                } else {
                    block.iter().all(|byte| *byte == 0)
                }
            }),
            true,
        );
        context.diagnostic(format_args!("ext4-segment original-PFN={} block-size={} first-real-segment-complete-NotReady=1 actual-first-SG={:?} actual-last-SG={:?} mapped-hole-mapped={} Uptodate-published-after-all-blocks=1", address, bs, first_dma, last_dma, bs < 4096));
        Ok(())
    }
}

#[ktest::suite(fixture = Owner, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("fs-failed-loading"), resources("root"), timeout_ms = 30000)]
pub(crate) mod failed_loading {
    use super::*;

    /// 两个登记 waiter 收同 inode Checksum，失败占位被移除且下一次真正重载。
    #[test]
    #[ktest(tags("fs-bad-inode-loading", "B12", "B14"))]
    fn inode_checksum_wakes_all(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        inode_flight(context, fixture, true)
    }

    /// extent CRC 失败完成同原 file folio，所有消费者收到 Checksum。
    #[test]
    #[ktest(tags("fs-bad-extent-loading", "B13", "B14"))]
    fn page_checksum_wakes_all(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        page_failure(context, fixture, true)
    }

    /// 无 CRC 的传统块越界保持 Corrupt，失败页可 clean prune 后真实重读失败。
    #[test]
    #[ktest(tags("fs-bad-mapping-loading", "B13", "B14"))]
    fn page_corrupt_wakes_all(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        page_failure(context, fixture, false)
    }
}
