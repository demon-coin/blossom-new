//! 同一 canonical inode 上的独立读历史、真实多原页 DMA、文件 fault 与截断等待。

use crate::arch::UserAccess;
use crate::block::{Bio, BioAllocator, BlkOperation, IoResult, SECTOR_BYTES};
use crate::fs::ext4::BlockMapping;
use crate::fs::ktest::user::write::WriteOwner;
use crate::fs::ktest::user::UserSuite;
use crate::fs::ktest::*;
use crate::fs::readahead::ReadaheadSnapshot;
use crate::fs::SeekFrom;
use crate::mm::dma::MappedPages;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;

const PATHNAME: &[u8] = b"/test/write-reserve";
const FULL_PAGES: usize = 64;
const TAIL_BYTES: usize = 37;
const FILE_BYTES: u64 = (FULL_PAGES * PAGE_SIZE + TAIL_BYTES) as u64;
const FAULT_ADDRESS: usize = 0x4c00_0000;

const RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};

/// 页号和页内偏移共同决定固定输入，错页与未初始化尾部不能满足内容断言。
fn pattern_byte(position: u64) -> u8 {
    0x39u8
        .wrapping_add(((position / PAGE_SIZE as u64) as u8).wrapping_mul(17))
        .wrapping_add((position % PAGE_SIZE as u64 % 251) as u8)
}

/// 按实际文件偏移核全部交付字节，不把请求长度或缓存命中当成读取成功。
fn matches_pattern(bytes: &[u8], offset: u64) -> bool {
    bytes
        .iter()
        .enumerate()
        .all(|(index, byte)| *byte == pattern_byte(offset + index as u64))
}

/// 本例用普通 write 和 pwrite 重建输入，真实同步后只清页缓存，保留原 canonical inode。
fn prepare_file(
    context: &mut TestContext<'_>,
    fixture: &mut WriteOwner,
) -> Result<InodeRef, TestError> {
    fixture.prepare_namespace(context)?;
    let writer = fixture.open(context, PATHNAME, 2)?;
    let inode = checked(context, writer.path().unwrap().inode())?.clone();
    checked(context, writer.truncate(0))?;
    let mut bytes = [0u8; PAGE_SIZE];
    for page in 0..FULL_PAGES {
        let offset = (page * PAGE_SIZE) as u64;
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = pattern_byte(offset + index as u64);
        }
        let written = checked(context, writer.write(&bytes))?;
        if written != bytes.len() {
            return Err(TestError("readahead input write was short"));
        }
    }

    let offset = (FULL_PAGES * PAGE_SIZE) as u64;
    for (index, byte) in bytes[..TAIL_BYTES].iter_mut().enumerate() {
        *byte = pattern_byte(offset + index as u64);
    }
    context.expect_eq(
        checked(context, writer.write_at(offset, &bytes[..TAIL_BYTES]))?,
        TAIL_BYTES,
    );
    context.expect_eq(checked(context, writer.seek(SeekFrom::Current(0)))?, offset);
    context.expect_eq(checked(context, writer.stat())?.size, FILE_BYTES);
    checked(context, writer.sync_range(0..u64::MAX, false))?;
    drop(writer);
    finish_reads(checked(context, inode.disk_volume())?.disk())?;
    context.expect_eq(
        checked(context, inode.disk_volume())?
            .disk()
            .test_snapshot()
            .inflight,
        0,
    );
    prune_clean(context, &inode)?;
    Ok(inode)
}

/// 等已接受请求归还真实 slot 和完成 Work，资源基线仍只由原 fixture 停稳后比较。
fn finish_reads(disk: &DiskRef) -> TestResult {
    until(
        "readahead requests did not release their original slots",
        || disk.test_snapshot().inflight == 0,
    )?;
    disk.test_flush_completions();
    Ok(())
}

/// 一次清理只摘已停稳的 clean 原页，未能清空的真实拥有直接报错，不轮询平账。
fn prune_clean(context: &mut TestContext<'_>, inode: &InodeRef) -> TestResult {
    context.expect_eq(inode.mapping().has_dirty_or_writeback(), false);
    let present = inode.mapping().len();
    context.expect_eq(inode.mapping().prune_unused(), present);
    if !inode.mapping().is_empty() {
        return Err(TestError("clean file pages still have actual owners"));
    }
    Ok(())
}

/// 普通文件必须提供自己的只读提示，目录或缺少观察不能代替空历史。
fn history(file: &FileRef) -> Result<ReadaheadSnapshot, TestError> {
    file.ktest_readahead_snapshot()
        .ok_or(TestError("ordinary file readahead observation missing"))
}

/// 真实消费者从共享 cursor 交付完整一页，原任务保存内容错误供控制者消费。
fn read_page(file: &FileRef, page: u64) -> Result<(), FsError> {
    let mut bytes = [0xa7; PAGE_SIZE];
    if file.read(&mut bytes)? != bytes.len() {
        return Err(FsError::Io(BlockError::ShortIo));
    }
    if !matches_pattern(&bytes, page * PAGE_SIZE as u64) {
        return Err(FsError::Corrupt);
    }
    Ok(())
}

/// 随机消费者仍沿正式 pread 交付原页，比较全部字节而不修改当前位置。
fn read_page_at(context: &mut TestContext<'_>, file: &FileRef, page: u64) -> TestResult {
    let mut bytes = [0xa7; PAGE_SIZE];
    let offset = page * PAGE_SIZE as u64;
    context.expect_eq(
        checked(context, file.read_at(offset, &mut bytes))?,
        bytes.len(),
    );
    context.expect_eq(matches_pattern(&bytes, offset), true);
    Ok(())
}

/// 只查本盘最近真实接受波的序号，其他盘的旧观察不能满足本例停点。
fn batch_sequence(inode: &InodeRef) -> u64 {
    inode
        .disk_volume()
        .expect("disk readahead fixture has no volume")
        .claim()
        .ktest_read_batch_snapshot()
        .map_or(0, |snapshot| snapshot.sequence)
}

/// 核原批次接受量、每页真实地址和内容状态；完成后的原页还须逐字节等准备输入。
fn batch_evidence(
    context: &mut TestContext<'_>,
    inode: &InodeRef,
    start: u64,
    pages: usize,
    ready: bool,
) -> TestResult {
    let snapshot = inode
        .disk_volume()
        .map_err(|_| TestError("disk readahead fixture has no volume"))?
        .claim()
        .ktest_read_batch_snapshot()
        .ok_or(TestError("actual read batch was not captured"))?;
    context.expect_eq(snapshot.pages, pages);
    context.expect_eq(snapshot.captured_pages, pages);
    context.expect_eq(snapshot.accepted_bytes, pages * PAGE_SIZE);
    context.expect_eq(0 < snapshot.bios && 0 < snapshot.multi_page_bios, true);

    let first = checked(
        context,
        inode.map_block(start * (PAGE_SIZE / inode.block_size() as usize) as u64),
    )?;
    let BlockMapping::Mapped { physical, .. } = first else {
        return Err(TestError("read batch first file block is not mapped"));
    };
    context.expect_eq(
        snapshot.first_sector,
        physical * inode.block_size() as u64 / SECTOR_BYTES,
    );
    for page in 0..pages {
        let index = start + page as u64;
        let folio = inode
            .mapping()
            .cached_folio(index)
            .ok_or(TestError("read batch canonical folio is missing"))?
            .with_file_owner(inode.clone());
        context.expect_eq(snapshot.phys_addrs[page], folio.phys_addr());
        context.expect_eq(snapshot.dma_addrs[page], folio.phys_addr() as u64);
        context.expect_eq(inode.mapping().is_canonical(index, &folio), true);
        if ready {
            checked(context, folio.wait_ready().map_err(FsError::Io))?;
            let mut bytes = [0xa7; PAGE_SIZE];
            checked(context, folio.copy_to(0, &mut bytes).map_err(FsError::Io))?;
            context.expect_eq(matches_pattern(&bytes, index * PAGE_SIZE as u64), true);
        } else {
            context.expect_eq(folio.range_ready(0, PAGE_SIZE), Err(BlockError::NotReady));
        }
    }
    context.diagnostic(format_args!(
        "W29 read-batch={:?} first-file-page={} original-pages-ready={} disk={:?} hardware-inflight-claimed=0",
        snapshot, start, ready, checked(context, inode.disk_volume())?.disk().test_snapshot()
    ));
    Ok(())
}

/// 真实 fault 及复制错误保留 VM 原因，测试不构造替代叶或私有数据镜像。
fn vm<T>(context: &TestContext<'_>, result: Result<T, VmError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("readahead-vm error={:?}", error));
        TestError("readahead VM operation failed")
    })
}

/// 共用原可写拥有者的停止、I/O 全尾、MM 六域和用户 metadata 三域清理。
#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-readahead", "fs-write", "W29"), resources("root"), timeout_ms = 180000)]
pub(crate) mod reads {
    use super::*;

    /// 顺序和随机独立 OFD 共享数据页，dup 共享历史；原 cursor 被占用时真实 fault 仍可完成。
    #[test]
    #[ktest(min_cpus = 2)]
    fn independent_windows_dup_pread_and_filefault(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let inode = prepare_file(context, fixture)?;
        let first = fixture.open(context, PATHNAME, 0)?;
        let random = fixture.open(context, PATHNAME, 0)?;
        context.expect_eq(Arc::ptr_eq(&first, &random), false);
        context.expect_eq(
            checked(context, random.path().unwrap().inode())?.ptr_eq(&inode),
            true,
        );
        let table = checked(context, FileTable::try_new())?;
        let fd = checked(context, table.reserve(1024))?
            .publish(first.clone(), DescriptorFlags::default());
        let duplicate = checked(context, table.dup(fd, 1024))?;
        let shared = checked(context, table.lookup(duplicate))?;
        context.expect_eq(Arc::ptr_eq(&first, &shared), true);
        let empty = ReadaheadSnapshot {
            start: 0,
            size: 0,
            async_size: 0,
            prev_pos: u64::MAX,
            ra_pages: 32,
        };
        context.expect_eq(history(&first)?, empty);
        context.expect_eq(history(&random)?, empty);

        // 暖页只供故障者验证 cursor 不形成等待环，首页及其整个预读窗仍为空。
        let mut fault_attempt = ReclaimAttempt::new();
        let fault_page = checked(
            context,
            first
                .mapping_inode()
                .unwrap()
                .file_page(63, &mut fault_attempt),
        )?;
        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("readahead fault MM owner OOM"))?;
        vm(
            context,
            mm.map_file(
                FAULT_ADDRESS,
                PAGE_SIZE,
                RO,
                RO,
                first.clone(),
                63 * PAGE_SIZE as u64,
                false,
            ),
        )?;
        context.expect_eq(history(&first)?, empty);
        context.expect_eq(mm.mapping(FAULT_ADDRESS).is_none(), true);
        let disk = checked(context, inode.disk_volume())?.disk().clone();
        finish_reads(&disk)?;
        let before = disk.test_snapshot();
        let old_sequence = batch_sequence(&inode);
        disk.test_suspend_completions();
        let reading = first.clone();
        let demand = fixture.consume_sync(0, move || read_page(&reading, 0))?;
        let held = (|| -> TestResult {
            until(
                "initial readahead did not submit and block its demand reader",
                || {
                    batch_sequence(&inode) != old_sequence
                        && before.file_read_submitted < disk.test_snapshot().file_read_submitted
                        && demand.blocked()
                },
            )?;
            context.expect_eq(
                history(&first)?,
                ReadaheadSnapshot {
                    start: 0,
                    size: 4,
                    async_size: 3,
                    prev_pos: u64::MAX,
                    ra_pages: 32,
                },
            );
            context.expect_eq(history(&random)?, empty);
            context.expect_eq(demand.result(), None);
            context.expect_eq(disk.test_snapshot().completed, before.completed);
            batch_evidence(context, &inode, 0, 4, false)?;

            let previous = history(&first)?;
            let faulting = mm.clone();
            let fault = fixture.consume_sync(1, move || {
                faulting
                    .fault(FAULT_ADDRESS, UserAccess::Read)
                    .map_err(|error| match error {
                        VmError::File(error) => error,
                        VmError::NoMemory => FsError::NoMemory,
                        _ => FsError::BadAddress,
                    })
            })?;
            until("file fault waited on the blocked OFD cursor", || {
                fault.result().is_some()
            })?;
            context.expect_eq(fault.result(), Some(Ok(())));
            context.expect_eq(demand.blocked(), true);
            context.expect_eq(history(&first)?, previous);
            context.expect_eq(
                mm.mapping(FAULT_ADDRESS).map(|leaf| leaf.physical),
                Some(fault_page.phys_addr()),
            );
            let mut bytes = [0xa7; PAGE_SIZE];
            let copied = mm.copyin(&mut bytes, FAULT_ADDRESS);
            context.expect_eq(copied.copied, bytes.len());
            context.expect_eq(copied.error, None);
            context.expect_eq(matches_pattern(&bytes, 63 * PAGE_SIZE as u64), true);
            Ok(())
        })();

        disk.test_resume_completions();
        let joined = fixture.join_consumers();
        held?;
        joined?;
        context.expect_eq(demand.result(), Some(Ok(())));
        batch_evidence(context, &inode, 0, 4, true)?;
        finish_reads(&disk)?;
        let after = disk.test_snapshot();
        context.expect_eq(after.inflight, 0);
        context.expect_eq(
            after.completed - before.completed,
            after.submitted - before.submitted,
        );
        context.expect_eq(
            0 < after.file_read_submitted - before.file_read_submitted,
            true,
        );
        context.expect_eq(
            history(&first)?,
            ReadaheadSnapshot {
                start: 0,
                size: 4,
                async_size: 3,
                prev_pos: PAGE_SIZE as u64,
                ra_pages: 32,
            },
        );

        // 命中触发页必须先返回需求数据，新八页仍由原 Bio 持有内容锁。
        let before_async = disk.test_snapshot();
        let old_sequence = batch_sequence(&inode);
        disk.test_suspend_completions();
        let reading = shared.clone();
        let trigger = fixture.consume_sync(0, move || read_page(&reading, 1))?;
        let asynchronous = (|| -> TestResult {
            until(
                "cached trigger did not return with its readahead still pending",
                || trigger.result().is_some(),
            )?;
            context.expect_eq(trigger.result(), Some(Ok(())));
            context.expect_eq(batch_sequence(&inode) != old_sequence, true);
            context.expect_eq(
                history(&first)?,
                ReadaheadSnapshot {
                    start: 4,
                    size: 8,
                    async_size: 8,
                    prev_pos: 2 * PAGE_SIZE as u64,
                    ra_pages: 32,
                },
            );
            context.expect_eq(history(&shared)?, history(&first)?);
            context.expect_eq(history(&random)?, empty);
            context.expect_eq(disk.test_snapshot().completed, before_async.completed);
            batch_evidence(context, &inode, 4, 8, false)
        })();

        disk.test_resume_completions();
        let joined = fixture.join_consumers();
        asynchronous?;
        joined?;
        batch_evidence(context, &inode, 4, 8, true)?;
        finish_reads(&disk)?;
        let after_async = disk.test_snapshot();
        context.expect_eq(after_async.inflight, 0);
        context.expect_eq(
            after_async.completed - before_async.completed,
            after_async.submitted - before_async.submitted,
        );

        let sequential = history(&first)?;
        let random_cursor = checked(context, random.seek(SeekFrom::Current(0)))?;
        let random_before = disk.test_snapshot();
        read_page_at(context, &random, 40)?;
        context.expect_eq(
            checked(context, random.seek(SeekFrom::Current(0)))?,
            random_cursor,
        );
        context.expect_eq(
            history(&random)?,
            ReadaheadSnapshot {
                prev_pos: 41 * PAGE_SIZE as u64,
                ..empty
            },
        );
        context.expect_eq(history(&first)?, sequential);
        context.expect_eq(inode.mapping().cached_folio(39).is_none(), true);
        context.expect_eq(inode.mapping().cached_folio(41).is_none(), true);
        context.expect_eq(
            random_before.file_read_submitted < disk.test_snapshot().file_read_submitted,
            true,
        );

        checked(context, random.seek(SeekFrom::Start(52 * PAGE_SIZE as u64)))?;
        checked(context, read_page(&random, 52))?;
        context.expect_eq(
            history(&random)?,
            ReadaheadSnapshot {
                prev_pos: 53 * PAGE_SIZE as u64,
                ..empty
            },
        );
        context.expect_eq(inode.mapping().cached_folio(51).is_none(), true);
        context.expect_eq(inode.mapping().cached_folio(53).is_none(), true);
        context.expect_eq(history(&first)?, sequential);
        context.expect_eq(
            checked(context, shared.seek(SeekFrom::Current(0)))?,
            2 * PAGE_SIZE as u64,
        );
        read_page_at(context, &shared, 20)?;
        context.expect_eq(
            checked(context, first.seek(SeekFrom::Current(0)))?,
            2 * PAGE_SIZE as u64,
        );
        context.expect_eq(history(&shared)?, history(&first)?);
        context.expect_eq(history(&first)?.prev_pos, 21 * PAGE_SIZE as u64);

        // 冷 fault 自己发出真实读入，不继承任何 OFD 预读窗或推进其位置。
        vm(
            context,
            mm.map_file(
                FAULT_ADDRESS + PAGE_SIZE,
                PAGE_SIZE,
                RO,
                RO,
                first.clone(),
                30 * PAGE_SIZE as u64,
                false,
            ),
        )?;
        context.expect_eq(inode.mapping().cached_folio(30).is_none(), true);
        let before_fault = history(&first)?;
        let fault_reads = disk.test_snapshot().file_read_submitted;
        vm(
            context,
            mm.fault(FAULT_ADDRESS + PAGE_SIZE, UserAccess::Read),
        )?;
        context.expect_eq(history(&first)?, before_fault);
        context.expect_eq(
            checked(context, first.seek(SeekFrom::Current(0)))?,
            2 * PAGE_SIZE as u64,
        );
        let mut canonical_attempt = ReclaimAttempt::new();
        let canonical = checked(
            context,
            random
                .mapping_inode()
                .unwrap()
                .file_page(30, &mut canonical_attempt),
        )?;
        context.expect_eq(
            mm.mapping(FAULT_ADDRESS + PAGE_SIZE)
                .map(|leaf| leaf.physical),
            Some(canonical.phys_addr()),
        );
        context.expect_eq(fault_reads < disk.test_snapshot().file_read_submitted, true);
        context.expect_eq(inode.mapping().cached_folio(29).is_none(), true);
        context.expect_eq(inode.mapping().cached_folio(31).is_none(), true);
        let mut bytes = [0xa7; PAGE_SIZE];
        let copied = mm.copyin(&mut bytes, FAULT_ADDRESS + PAGE_SIZE);
        context.expect_eq((copied.copied, copied.error), (PAGE_SIZE, None));
        context.expect_eq(matches_pattern(&bytes, 30 * PAGE_SIZE as u64), true);

        let random_history = history(&random)?;
        for page in 2..FULL_PAGES as u64 {
            checked(context, read_page(&first, page))?;
        }
        context.expect_eq(history(&random)?, random_history);
        context.expect_eq(
            checked(context, random.seek(SeekFrom::Current(0)))?,
            53 * PAGE_SIZE as u64,
        );
        let mut first_attempt = ReclaimAttempt::new();
        let same_first = checked(
            context,
            first
                .mapping_inode()
                .unwrap()
                .file_page(40, &mut first_attempt),
        )?;
        let mut random_attempt = ReclaimAttempt::new();
        let same_random = checked(
            context,
            random
                .mapping_inode()
                .unwrap()
                .file_page(40, &mut random_attempt),
        )?;
        context.expect_eq(same_first.phys_addr(), same_random.phys_addr());
        finish_reads(&disk)?;
        context.diagnostic(format_args!(
            "W29 independent-OFD=2 dup-shared-history=1 pread-cursor-preserved=1 initial-window=4 next-window=8 canonical-fault-page={} demand-cursor-blocked-during-fault=1 cold-page-cache-only=1 submitted={} completed={}",
            canonical.phys_addr(), disk.test_snapshot().submitted, disk.test_snapshot().completed
        ));
        Ok(())
    }

    /// 短 EOF 保留原尾零；已返回的异步窗口仍用原内容锁阻止 truncate 提前摘页返块。
    #[test]
    #[ktest(min_cpus = 2)]
    fn eof_tail_and_truncate_wait_for_async_fill(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let inode = prepare_file(context, fixture)?;
        let file = fixture.open(context, PATHNAME, 2)?;
        let disk = checked(context, inode.disk_volume())?.disk().clone();
        let mut bytes = [0xa7; 128];
        let offset = (FULL_PAGES * PAGE_SIZE - 19) as u64;
        context.expect_eq(
            checked(context, file.read_at(offset, &mut bytes))?,
            19 + TAIL_BYTES,
        );
        context.expect_eq(matches_pattern(&bytes[..19 + TAIL_BYTES], offset), true);
        context.expect_eq(
            bytes[19 + TAIL_BYTES..].iter().all(|byte| *byte == 0xa7),
            true,
        );
        context.expect_eq(checked(context, file.read_at(FILE_BYTES, &mut bytes))?, 0);
        let mut tail_attempt = ReclaimAttempt::new();
        let tail = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(FULL_PAGES as u64, &mut tail_attempt),
        )?;
        let mut page = [0xa7; PAGE_SIZE];
        checked(context, tail.copy_to(0, &mut page).map_err(FsError::Io))?;
        context.expect_eq(
            matches_pattern(&page[..TAIL_BYTES], (FULL_PAGES * PAGE_SIZE) as u64),
            true,
        );
        context.expect_eq(page[TAIL_BYTES..].iter().all(|byte| *byte == 0), true);
        context.expect_eq(
            inode
                .mapping()
                .cached_folio(FULL_PAGES as u64 + 1)
                .is_none(),
            true,
        );
        context.expect_eq(checked(context, file.seek(SeekFrom::Current(0)))?, 0);
        drop(tail);
        finish_reads(&disk)?;
        prune_clean(context, &inode)?;

        let sequential = fixture.open(context, PATHNAME, 0)?;
        checked(context, read_page(&sequential, 0))?;
        batch_evidence(context, &inode, 0, 4, true)?;
        finish_reads(&disk)?;
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        let reading = sequential.clone();
        let trigger = fixture.consume_sync(0, move || read_page(&reading, 1))?;
        let blocked = (|| -> Result<_, TestError> {
            until(
                "truncate input did not leave an actual async read window",
                || trigger.result().is_some(),
            )?;
            context.expect_eq(trigger.result(), Some(Ok(())));
            batch_evidence(context, &inode, 4, 8, false)?;

            let original = inode
                .mapping()
                .cached_folio(4)
                .ok_or(TestError("truncate original async folio missing"))?
                .with_file_owner(inode.clone());
            let truncating = file.clone();
            let truncate =
                fixture.consume_sync(1, move || truncating.truncate(4 * PAGE_SIZE as u64))?;
            until(
                "truncate did not wait on original async folio contents",
                || {
                    truncate.blocked()
                        && inode
                            .metadata_snapshot()
                            .is_ok_and(|metadata| metadata.size == 4 * PAGE_SIZE as u64)
                },
            )?;
            context.expect_eq(truncate.result(), None);
            context.expect_eq(inode.mapping().is_canonical(4, &original), true);
            context.expect_eq(
                original.range_ready(0, PAGE_SIZE),
                Err(BlockError::NotReady),
            );
            context.expect_eq(disk.test_snapshot().completed, before.completed);
            Ok((original, truncate))
        })();

        disk.test_resume_completions();
        let joined = fixture.join_consumers();
        let (original, truncate) = blocked?;
        joined?;
        context.expect_eq(truncate.result(), Some(Ok(())));
        finish_reads(&disk)?;
        let after = disk.test_snapshot();
        context.expect_eq(after.inflight, 0);
        context.expect_eq(
            after.completed - before.completed,
            after.submitted - before.submitted,
        );
        context.expect_eq(before.file_read_submitted < after.file_read_submitted, true);
        context.expect_eq(inode.mapping().is_canonical(4, &original), false);
        context.expect_eq(inode.mapping().cached_folio(4).is_none(), true);
        context.expect_eq(
            checked(
                context,
                inode.map_block((4 * PAGE_SIZE / inode.block_size() as usize) as u64),
            )?,
            BlockMapping::Eof,
        );
        context.expect_eq(checked(context, file.stat())?.size, 4 * PAGE_SIZE as u64);
        read_page_at(context, &file, 3)?;
        context.expect_eq(
            checked(context, file.read_at(4 * PAGE_SIZE as u64, &mut bytes))?,
            0,
        );
        checked(context, file.sync_range(0..u64::MAX, false))?;
        context.diagnostic(format_args!(
            "W29 eof-tail-bytes={} zero-tail=1 truncate-blocked-on-original-page={} async-pages=8 actual-read-used={} software-completion-gate=1 hardware-inflight-claimed=0",
            TAIL_BYTES, original.phys_addr(), after.completed - before.completed
        ));
        Ok(())
    }

    /// 同一多页父的首页完成可读，真实 used 后受控失败的后页保错误且父结果仍失败。
    #[test]
    fn multi_page_parent_keeps_successful_first_page(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let inode = prepare_file(context, fixture)?;
        let file = fixture.open(context, PATHNAME, 0)?;
        let disk = checked(context, inode.disk_volume())?.disk().clone();
        let block_size = inode.block_size() as usize;
        let block_count = 2 * PAGE_SIZE / block_size;
        let blocks_per_page = PAGE_SIZE / block_size;
        let mut pair = None;
        for page in 0..FULL_PAGES - 1 {
            let logical = page * blocks_per_page;
            let mut first_block = None;
            let mut contiguous = true;
            for offset in 0..block_count {
                let BlockMapping::Mapped { physical, .. } =
                    checked(context, inode.map_block((logical + offset) as u64))?
                else {
                    return Err(TestError(
                        "prefix regression requires allocated ordinary file blocks",
                    ));
                };
                let first = *first_block.get_or_insert(physical);
                if physical != first + offset as u64 {
                    contiguous = false;
                    break;
                }
            }
            if contiguous {
                pair = Some((page as u64, first_block.unwrap()));
                break;
            }
        }
        let (first_page, first_block) = pair.ok_or(TestError(
            "ordinary file has no contiguous two-page read range",
        ))?;
        let limits = disk.limits();
        if limits.max_bytes < 2 * PAGE_SIZE
            || limits.max_segments < 2
            || limits.max_segment_bytes < PAGE_SIZE as u32
        {
            return Err(TestError(
                "prefix regression requires one real two-page read slot",
            ));
        }

        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(2)
            .map_err(|_| TestError("prefix DMA owner vector OOM"))?;
        let mut originals = [None, None];
        let mut attempt = ReclaimAttempt::new();
        let mut prepared = [const { None }; 2];
        for (index, candidate) in prepared.iter_mut().enumerate() {
            *candidate = Some(checked(
                context,
                inode
                    .mapping()
                    .prepare_file(&inode, first_page + index as u64, &mut attempt),
            )?);
        }
        let invalidate = checked(context, inode.lock_invalidate())?;
        for (index, original) in originals.iter_mut().enumerate() {
            let (folio, read) = checked(
                context,
                inode
                    .mapping()
                    .publish_file(&inode, first_page + index as u64, &mut prepared[index])
                    .map_err(FsError::Io),
            )?
            .ok_or(TestError(
                "prefix file candidate retired before publication",
            ))?;
            let read = read.ok_or(TestError("prefix regression file page was not cold"))?;
            *original = Some(folio);
            let mapped = MappedPages::from_folio(read).map_err(|(error, _read)| {
                context.diagnostic(format_args!(
                    "prefix original-page DMA preparation error={:?}",
                    error
                ));
                TestError("prefix original-page DMA preparation failed")
            })?;
            context.expect_eq(
                mapped
                    .segment(crate::mm::dma::DmaDomain::qemu_coherent())
                    .expect("QEMU fixture page exceeds DMA domain")
                    .address
                    .value(),
                original.as_ref().unwrap().phys_addr() as u64,
            );
            memory.push(mapped);
        }

        let sector = checked(context, inode.disk_volume())?
            .view()
            .absolute_byte(first_block * block_size as u64, 2 * PAGE_SIZE)
            .map_err(|_| TestError("prefix read escapes filesystem view"))?
            / SECTOR_BYTES;
        // SAFETY: 两份 FolioRead 各持唯一原页内容锁并附原 inode，CPU 未借用未初始化内容。
        // 设备只读入已经核实的连续文件块；同一 Bio 接管两原 DMA 映射，真实 used 后才收权。
        // 无回调和裸别名，claim 与 invalidate 保提交来源，后续由原完成者发表每页结果。
        let bio = checked(
            context,
            unsafe { Bio::try_new(&disk, BlkOperation::Read, sector, memory, 0, None) }
                .map_err(FsError::Io),
        )?;
        let completion = bio.completion();
        finish_reads(&disk)?;
        let before = disk.test_snapshot();
        disk.test_fail_next_file_read_after(PAGE_SIZE, BlockError::Io);
        checked(context, inode.disk_volume())?
            .claim()
            .submit_bio(bio);
        drop(invalidate);
        drop(prepared);

        let result = completion.wait();
        completion.test_flush_finalizer();
        finish_reads(&disk)?;
        context.expect_eq(
            completion.test_acceptance(),
            (1, 2 * PAGE_SIZE, 2 * PAGE_SIZE),
        );
        context.expect_eq(
            result,
            IoResult {
                completed: PAGE_SIZE,
                error: Some(BlockError::Io),
            },
        );
        context.expect_eq(disk.test_completion_error_pending(), false);
        let after = disk.test_snapshot();
        context.expect_eq(after.file_read_submitted - before.file_read_submitted, 1);
        context.expect_eq(after.submitted - before.submitted, 1);
        context.expect_eq(after.completed - before.completed, 1);
        context.expect_eq(after.inflight, 0);

        let first = originals[0].as_ref().unwrap();
        let failed = originals[1].as_ref().unwrap();
        context.expect_eq(first.phys_addr() == failed.phys_addr(), false);
        context.expect_eq(inode.mapping().is_canonical(first_page, first), true);
        context.expect_eq(inode.mapping().is_canonical(first_page + 1, failed), true);
        context.expect_eq(first.wait_ready(), Ok(()));
        let mut bytes = [0xa7; PAGE_SIZE];
        checked(context, first.copy_to(0, &mut bytes).map_err(FsError::Io))?;
        context.expect_eq(matches_pattern(&bytes, first_page * PAGE_SIZE as u64), true);
        context.expect_eq(failed.wait_ready(), Err(BlockError::Io));
        context.expect_eq(failed.copy_to(0, &mut bytes), Err(BlockError::Io));
        read_page_at(context, &file, first_page)?;
        context.expect_eq(
            file.read_at((first_page + 1) * PAGE_SIZE as u64, &mut bytes),
            Err(FsError::Io(BlockError::Io)),
        );
        context.expect_eq(disk.test_snapshot().submitted, after.submitted);
        context.expect_eq(completion.result(), Some(result));
        context.diagnostic(format_args!(
            "W29 actual-multi-page-parent=1 first-file-page={} accepted-bytes={} completed-prefix={} parent-error={:?} first-original-page={} failed-original-page={} controlled-software-completion-error=1 hardware-EIO-claimed=0",
            first_page, 2 * PAGE_SIZE, result.completed, result.error, first.phys_addr(), failed.phys_addr()
        ));
        Ok(())
    }
}
