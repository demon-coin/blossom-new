//! 文件系统 Inode 缓存单飞加载、文件页缓存并发加载及格式错误传播测试套件。
//!
//! # 模块职责与架构定位
//! 本模块验证 `InodeCache` 与 `FileMapping` 在多核并发任务访问下的单飞（Singleflight）合并、
//! 跨页并发 I/O 非阻塞性、块大小小于页大小时的分段加载可见性，以及磁盘格式错误（Checksum/Corrupt）
//! 对所有同键等待者的唤醒与占位清理语义。
//! - **上游调用者**：由 `fs::ktest::anchors` 注册到 `ktest` 框架的三个测试套件
//!   （`inode_loading`、`file_loading`、`failed_loading`）。
//! - **下游被测模块**：`fs::inode::InodeCache`、`mm::page_cache::FileMapping`、`fs::ext4::Ext4Volume`
//!   以及 `block::Disk` 的完成中断路径。
//!
//! # 核心单飞状态机与并发时序
//!
//! ```text
//!   任务 0 (首个加载者 Winner)                任务 1, 2 (后续同键请求者 Waiters)
//! ------------------------------            ------------------------------------
//! iget(ino) / publish_file(page)
//!   |-- 插入 Loading 占位或未就绪 Folio
//!   |-- 触发测试钩子并等待 gate -------------> iget(ino) / read_at(page)
//!                                               |-- 命中在途占位，登记到等待队列并睡眠 (Blocked)
//!   <-- 主线程确认 2 个等待者就位后放行 gate --+
//!   |-- 执行块设备读取与 ext4 格式解析
//!   +-- 成功: 转为 Live / 标记 Folio 就绪 ---> 唤醒任务 1, 2，共享同一 InodeRef / 同一物理页 PFN
//!   +-- 失败: 移除占位 / 标记 Folio 错误  ---> 唤醒任务 1, 2，所有等待者收到相同 FsError
//! ```
//!
//! # 锁与同步契约
//! - 测试通过 `Observation::hold_loader` 与 `Observation::gate` 在不持有自旋锁的上下文中暂停首个加载线程，
//!   或者通过 `Disk::test_suspend_completions` 暂缓块设备完成分发，从而确定性地构造多核同键等待或跨页双请求在途场景。

use crate::fs::ktest::*;

/// 轮询等待主块设备硬件 Virtqueue 的 `used.idx` 前进到目标值 `target`，确认底层设备已完成相应数量的 I/O 请求。
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

/// 读取主块设备硬件 Virtqueue 当前的 `used.idx` 索引值。
fn used_index(fixture: &Owner) -> Result<u16, TestError> {
    let disk = fixture.disk()?;
    let legacy = matches!(
        disk.identity(),
        DiskIdentity::Virtio(VirtioIdentity::Mmio { version: 1, .. })
    );
    Ok(block::tests::used_index(disk, legacy))
}

/// 从根目录中查找名为 `"pattern"` 的目录项并返回其 Inode 编号，即使目标 Inode 自身校验和损坏也可正常获取编号。
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

/// 驱动 3 个并发任务同时通过 `InodeCache::iget` 获取同一 Inode，验证单飞合并加载（`failure = false`）或错误唤醒与占位清理（`failure = true`）。
fn inode_flight(context: &mut TestContext<'_>, fixture: &mut Owner, failure: bool) -> TestResult {
    let volume = fixture.open(context)?;
    let number = file_number(context, fixture, &volume)?;
    let before_len = fixture.cache().len();
    let before_load = fixture.cache().ktest_load_count();
    fixture
        .observer()
        .hold_loader
        .store(true, Ordering::Release);

    // Step 1: 发布任务 0 触发首次加载并停在 Loading 钩子处，随后发布任务 1、2 进入同一 Inode 的等待队列。
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

    // Step 2: 放行首个加载者完成磁盘读取与解析，并等待全部 3 个任务退出。
    fixture
        .observer()
        .hold_loader
        .store(false, Ordering::Release);
    fixture.observer().gate.complete();
    fixture.join_tasks()?;

    // Step 3: 校验 3 个任务的返回结果：失败场景下均收到 Checksum 错误且占位被移除；成功场景下共享同一 InodeRef 与 FileMapping。
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

/// 收集并校验 `count` 个文件页读取任务的返回结果，确认页面均已就绪、物理页地址符合同页共享（`same = true`）或异页独立（`same = false`）预期，且页内数据匹配测试图样。
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

/// 暂缓块设备完成回调以构造同页并发单飞等待（`same = true`）或不同页并行在途 I/O（`same = false`），恢复完成回调后校验数据与物理页身份。
fn file_flight(context: &mut TestContext<'_>, fixture: &mut Owner, same: bool) -> TestResult {
    let volume = fixture.open(context)?;
    let inode = fixture.file(context, &volume, b"pattern")?;
    let count = if same { 3 } else { 2 };
    let before = fixture.disk()?.test_snapshot();
    let first_used = used_index(fixture)?;
    fixture.disk()?.test_suspend_completions();

    // Step 1: 暂缓块设备完成分发，启动任务 0 读取第 0 页并等待其提交硬件请求后进入睡眠，再启动后续任务读取同页或相邻页。
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

    // Step 2: 断言在途块请求数与提交数（同页仅 1 个，异页为 2 个），并确认第 0 页在完成前处于未就绪（NotReady）状态。
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

    // Step 3: 恢复块设备完成处理并等待全部任务完成，校验总提交块数、物理页帧号（PFN）及页内图样数据。
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

/// 让首个文件页加载任务在发布未就绪页后暂停，待两个同页读取任务进入等待队列后放行触发底层格式错误，验证所有等待者收到相同错误且失败页可被清理重读。
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

    // Step 1: 启动任务 0 将目标文件页以未就绪状态发布到页缓存并暂停在门控处，随后启动任务 1、2 阻塞在该未就绪页上。
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

    // Step 2: 放行任务 0 执行块映射解析并触发 Checksum 或 Corrupt 错误，等待所有任务结束。
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
    // Step 3: 验证全部 3 个任务均收到预期错误、Folio 记录了对应错误状态，且释放引用后可通过 prune_unused 回收失败页。
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
    /// 验证多个内核任务并发请求同一个未缓存 Inode 时，仅发起一次磁盘加载，且所有等待者最终共享同一个 `InodeRef` 实例与同一个 `FileMapping`。
    ///
    /// - **测试目标**：确认 `InodeCache::iget` 的单飞（Singleflight）加载机制不会对同一 Inode 产生重复磁盘 I/O 或构造重复的内存结构。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 开启加载暂停钩子，发布任务 0 加载 `pattern` 文件 Inode 并使其停在 `Loading` 状态；
    ///   2. 发布任务 1 和任务 2 请求同一 Inode 编号，等待两者均登记到该 Inode 的等待队列并进入 `Blocked` 状态；
    ///   3. 放行加载门控并等待 3 个任务完成，断言总加载次数仅增加 1 次，且 3 个任务返回的 `InodeRef` 指针与 `FileMapping` 指针完全相等。
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

    /// 验证多个任务并发读取同一个尚未缓存的文件页时，仅由首个任务发起一次块设备读取，其余等待者唤醒后共享同一个物理页帧（PFN）。
    ///
    /// - **测试目标**：确认 `FileMapping` 页缓存在单页粒度上的单飞合并语义，避免同页并发读产生冗余磁盘 I/O 或重复分配物理页。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 暂缓块设备完成回调，启动任务 0 读取第 0 页，待硬件 `used.idx` 前进且任务 0 阻塞后，启动任务 1、2 读取同一页；
    ///   2. 等待任务 1、2 均进入 `Blocked` 睡眠态，断言块设备在途请求数与新增提交数均为 1，且页缓存中的 Folio 处于未就绪状态；
    ///   3. 恢复块设备完成回调并等待 3 个任务结束，断言 3 个任务读到完整的测试图样且持有的 `FolioRef` 指向同一个物理页地址。
    #[test]
    #[ktest(tags("B14"))]
    fn same_page_unique_fill(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        file_flight(context, fixture, true)
    }

    /// 验证并发读取同一文件的不同逻辑页时，两个页面的块设备请求能够同时在途执行，不会被 Inode 级别的粗粒度锁串行化。
    ///
    /// - **测试目标**：确认文件读路径的锁粒度允许同一 Inode 下不同页的磁盘 I/O 并发提交与在途等待。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 暂缓块设备完成回调，依次启动任务 0 读取第 0 页、任务 1 读取第 1 页；
    ///   2. 等待硬件 `used.idx` 前进 2 次且两个任务均进入 `Blocked` 状态，断言块设备同时存在 2 个在途请求；
    ///   3. 恢复块设备完成回调并等待两个任务结束，断言两页各自拥有独立的物理页地址且页内数据分别匹配偏移 `0` 与 `PAGE_SIZE` 的图样。
    #[test]
    #[ktest(tags("B15"))]
    fn different_pages_two_inflight(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        file_flight(context, fixture, false)
    }

    /// 验证当文件系统块大小小于页大小且单页包含多个非连续块与空洞（Hole）时，页面在首个块 DMA 完成后仍保持未就绪（Locked）状态，直到所有子块填充完毕才对读者可见。
    ///
    /// - **测试目标**：确认分段填充（Segmented Fill）过程中不会因首个子块 I/O 完成而提前将页面标记为 `Uptodate`（就绪），且所有子块 DMA 与空洞清零均作用于同一个物理页（PFN）。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 注册分段钩子，在首个块（偏移 `0..bs`）完成时暂停加载线程，启动任务 0 填充 `deep` 文件的第 0 页；
    ///   2. 当钩子暂停时，检查页缓存中的第 0 页仍处于未就绪状态（`is_ready() == false`），且首个块的 DMA 目标地址指向该页首地址；
    ///   3. 放行门控让后续空洞清零与非连续映射块读取全部完成，断言最终页面就绪、物理页地址未变，且页内偶数逻辑块为有效图样、奇数逻辑块（空洞）为全零。
    #[test]
    #[ktest(tags("B13"))]
    fn segmented_page_stays_unreadable_until_all_blocks(
        context: &mut TestContext<'_>,
        fixture: &mut Owner,
    ) -> TestResult {
        let volume = fixture.open(context)?;
        // Step 1: 安装在首个子块填充完毕后暂停的钩子，并启动任务 0 填充第 0 页。
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

        // Step 2: 在首个子块 DMA 已完成但后续子块尚未处理时，验证该文件页仍处于未就绪（NotReady）状态。
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

        // Step 3: 放行门控完成剩余空洞与映射块的填充，验证页面转为就绪且各子块内容符合“映射块-空洞”交替分布。
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

    /// 验证当 Inode 磁盘元数据 CRC 校验失败时，所有等待该 Inode 加载的任务均被唤醒并收到 `FsError::Checksum`，且失败的 `Loading` 占位项被清除以允许后续重试。
    ///
    /// - **测试目标**：确认 `InodeCache` 在加载失败时不会残留死锁占位项，并能将底层格式错误准确广播给所有同键等待者。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 对损坏了 Inode 校验和的镜像启动 3 个并发任务获取同一 Inode，使任务 0 停在 `Loading` 阶段、任务 1 和 2 进入等待队列；
    ///   2. 放行任务 0 触发 `FsError::Checksum`，断言全部 3 个任务均返回 `Some(FsError::Checksum)` 且缓存条目数恢复原状；
    ///   3. 再次调用 `iget` 请求该 Inode，断言触发新的磁盘加载（加载计数加 1）并再次返回 `FsError::Checksum`。
    #[test]
    #[ktest(tags("fs-bad-inode-loading", "B12", "B14"))]
    fn inode_checksum_wakes_all(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        inode_flight(context, fixture, true)
    }

    /// 验证当文件页加载过程中遇到 Extent 树块 CRC 校验失败时，首个加载者与所有同页等待者均收到 `FsError::Checksum`，且失败页可被安全回收。
    ///
    /// - **测试目标**：确认文件页块映射解析阶段的 `FsError::Checksum` 会通过 `finish_error` 正确唤醒并终止未就绪 Folio 上的所有等待者。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 任务 0 发布未就绪页后暂停，任务 1、2 读取同一页并进入睡眠等待；
    ///   2. 放行任务 0 解析损坏的 Extent 节点触发 `FsError::Checksum`，断言 3 个任务均收到 `FsError::Checksum`；
    ///   3. 释放 Folio 引用并调用 `prune_unused` 清理失败页，再次调用 `read_at` 验证重新发起加载并同样返回 `FsError::Checksum`。
    #[test]
    #[ktest(tags("fs-bad-extent-loading", "B13", "B14"))]
    fn page_checksum_wakes_all(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        page_failure(context, fixture, true)
    }

    /// 验证当文件页加载过程中检测到块映射越界损坏（`FsError::Corrupt`）时，首个加载者与所有同页等待者均收到 `FsError::Corrupt`，且失败页可被回收并在重读时再次报错。
    ///
    /// - **测试目标**：确认文件页加载前的块号合法性校验失败同样能唤醒全部同页等待者，不会留下永远处于 `Locked` 状态的僵尸页。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 任务 0 发布第 3 页为未就绪状态后暂停，任务 1、2 读取第 3 页并阻塞等待；
    ///   2. 放行任务 0 触发块映射越界错误 `FsError::Corrupt`，断言 3 个任务均返回 `FsError::Corrupt`；
    ///   3. 释放 Folio 引用并通过 `prune_unused` 回收失败页，验证后续重读第 3 页仍然返回 `FsError::Corrupt`。
    #[test]
    #[ktest(tags("fs-bad-mapping-loading", "B13", "B14"))]
    fn page_corrupt_wakes_all(context: &mut TestContext<'_>, fixture: &mut Owner) -> TestResult {
        page_failure(context, fixture, false)
    }
}
