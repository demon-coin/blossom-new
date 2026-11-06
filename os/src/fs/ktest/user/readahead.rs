//! 普通文件预读窗口（Readahead）、多页块 I/O（Bio）DMA 映射、文件映射缺页（File Fault）与并发截断（`truncate`）的内核单元测试（ktest）。
//!
//! # 模块职责与架构定位
//! 本模块位于 `os/src/fs/ktest/user/readahead.rs`，在真实的块设备（Ext4 文件系统）与页缓存（Page Cache）之上，
//! 全面验证普通文件读取路径上的预读状态机、多页合并 DMA 读取、虚拟内存文件缺页以及 `truncate` 截断同步。
//! 上游作为 `reads` 测试套件接入 `ktest` 框架，下游直接操作 VFS 打开文件描述（`File`）、`Inode` 页缓存映射（`AddressSpace`）、块层 `Bio` 及虚拟内存空间 `Mm`。
//!
//! # 核心设计与预读状态机 / 并发协作关系
//!
//! ## 1. 顺序预读窗口演进与独立打开文件描述（OFD）隔离
//! ```text
//! 文件页索引:  [0]   [1]   [2]   [3] | [4]   [5]   [6]   [7]   [8]   [9]  [10]  [11]
//!              +-----+-----+-----+---+ +-----+-----+-----+-----+-----+-----+-----+-----+
//! 首次读 page 0: |<--- 同步+预读窗 --->|
//!              start=0, size=4, async_size=3 (page 1 为触发页/Readahead Marker)
//!
//! 顺序读至触发页 page 1（命中缓存）:
//!              立即向调用方返回 page 1 数据，同时异步提交下一批 8 页预读 Bio:
//!                                      |<--------------- 异步扩展预读窗 -------------->|
//!                                      start=4, size=8, async_size=8, prev_pos=2*PAGE_SIZE
//!
//! 打开文件描述（OFD）与缺页隔离规则:
//! - `dup(fd)` 共享同一个底层 `File` 实例 ──► 共享文件游标（cursor）与 `ReadaheadState`；
//! - 独立 `open()` 产生不同的 `File` 实例 ──► 各自维护独立的预读历史，但共享同一个 `Inode` 的页缓存（Folio）；
//! - `pread (read_at)` 随机单页读 ──► 只更新 `prev_pos`，不开启多页预读窗口，也不改变文件当前游标；
//! - `Mm::fault` 文件映射缺页 ──► 直接通过 `Inode` 页缓存按需装入目标页，不等待被阻塞的 OFD 游标锁，也不污染 OFD 的预读状态。
//! ```
//!
//! ## 2. 异步预读填充与 `truncate` 截断的页内容锁同步
//! ```text
//! 异步预读线程 / Bio 提交                     并发 truncate(4 * PAGE_SIZE) 线程
//! ───────────────────────────                 ─────────────────────────────────
//! 1. 分配并发布 page 4..12 的缓存页（Folio）
//! 2. 持有各 Folio 的读填充锁（FolioRead）
//! 3. 提交多页 Bio 至块设备（此时盘上完成被挂起）
//!                                             4. 更新 Inode 元数据 size = 4 * PAGE_SIZE
//!                                             5. 尝试失效并摘除索引 >= 4 的缓存页
//!                                             6. 发现 page 4..12 正处于 I/O 填充中（NotReady），
//!                                                必须在页内容锁上睡眠等待，禁止提前摘页或释放磁盘块！
//! 7. 块设备恢复完成通知，Bio 回调解锁 Folio
//!                                             8. truncate 被唤醒，安全摘除缓存页并释放后端磁盘块
//! ```
//!
//! # 锁获取顺序与并发契约
//! 1. **打开文件游标锁 vs 页缓存内容锁**：顺序 `read` 先获取 OFD 游标锁，再访问页缓存并等待 `Folio` 就绪；而 `pread` 与 `Mm::fault` 不获取 OFD 游标锁，因此当顺序读阻塞在慢速磁盘 I/O 上时，同一文件的已缓存页缺页（Fault）依然可以无阻塞完成。
//! 2. **截断屏障锁（`invalidate` 锁）与 Folio I/O 生命周期**：向块设备发布冷页读取 `Bio` 时，必须先持有 `inode.lock_invalidate()` 并在其保护下调用 `publish_file` 获取 `FolioRead` 令牌；在 DMA 传输尚未完成前，任何并发 `truncate` 都必须等待 `FolioRead` 完成并释放页内容锁后，才能将该页从 `AddressSpace` 中移除。

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

/// 预读与截断测试使用的 Ext4 普通文件路径。
const PATHNAME: &[u8] = b"/test/write-reserve";
/// 测试文件包含的完整物理页数量（64 页）。
const FULL_PAGES: usize = 64;
/// 测试文件末尾追加的不满一页的尾部字节数（37 字节）。
const TAIL_BYTES: usize = 37;
/// 测试文件的预期总字节大小（`64 * PAGE_SIZE + 37`）。
const FILE_BYTES: u64 = (FULL_PAGES * PAGE_SIZE + TAIL_BYTES) as u64;
/// 文件映射缺页（File Fault）测试使用的用户态虚拟基地址。
const FAULT_ADDRESS: usize = 0x4c00_0000;

/// 只读、不可执行的虚拟内存页权限常量。
const RO: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: false,
};

/// 根据文件字节偏移 `position` 生成确定性的测试字节模式。
///
/// 计算公式同时融合了页号（`position / PAGE_SIZE`）与页内偏移（`position % PAGE_SIZE`），
/// 确保一旦发生错页映射、DMA 偏移错位或未初始化尾部残留，都能被字节比对立即检出。
fn pattern_byte(position: u64) -> u8 {
    0x39u8
        .wrapping_add(((position / PAGE_SIZE as u64) as u8).wrapping_mul(17))
        .wrapping_add((position % PAGE_SIZE as u64 % 251) as u8)
}

/// 逐字节校验缓冲区 `bytes` 是否与文件偏移 `offset` 处的预期字节模式完全一致。
fn matches_pattern(bytes: &[u8], offset: u64) -> bool {
    bytes
        .iter()
        .enumerate()
        .all(|(index, byte)| *byte == pattern_byte(offset + index as u64))
}

/// 创建并初始化测试文件，写入完整模式数据并刷盘后清空干净页缓存，返回对应的 `InodeRef`。
///
/// # 执行流程
/// 1. 初始化命名空间并以读写模式打开测试文件，先将其截断为 `0`；
/// 2. 使用顺序 `write` 写入前 `FULL_PAGES`（64 页）完整页数据；
/// 3. 使用 `write_at`（`pwrite`）在文件末尾追加 `TAIL_BYTES`（37 字节）不对齐尾部数据，验证 `write_at` 不推进文件当前游标；
/// 4. 调用 `sync_range` 将全部脏页同步写回磁盘，等待块设备在途 I/O 归零后，调用 `prune_clean` 驱逐全部干净缓存页，为后续冷读测试提供干净的初始状态。
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

/// 等待指定磁盘上的所有在途 I/O 请求（`inflight`）完成，并排空底层的完成回调队列。
fn finish_reads(disk: &DiskRef) -> TestResult {
    until(
        "readahead requests did not release their original slots",
        || disk.test_snapshot().inflight == 0,
    )?;
    disk.test_flush_completions();
    Ok(())
}

/// 断言指定 `Inode` 不存在脏页或回写页，并一次性驱逐其页缓存中的全部干净页。
///
/// 若驱逐后页缓存仍非空（说明存在未释放的外部 `Folio` 强引用），则返回测试错误。
fn prune_clean(context: &mut TestContext<'_>, inode: &InodeRef) -> TestResult {
    context.expect_eq(inode.mapping().has_dirty_or_writeback(), false);
    let present = inode.mapping().len();
    context.expect_eq(inode.mapping().prune_unused(), present);
    if !inode.mapping().is_empty() {
        return Err(TestError("clean file pages still have actual owners"));
    }
    Ok(())
}

/// 获取普通文件当前的预读状态快照（`ReadaheadSnapshot`）。
///
/// 若目标文件描述不支持预读观测（如目录文件），则返回测试错误。
fn history(file: &FileRef) -> Result<ReadaheadSnapshot, TestError> {
    file.ktest_readahead_snapshot()
        .ok_or(TestError("ordinary file readahead observation missing"))
}

/// 通过推进共享文件游标的顺序 `read` 接口读取完整一页，并校验其字节内容是否匹配第 `page` 页。
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

/// 通过不修改文件游标的随机 `read_at`（`pread`）接口读取第 `page` 页，并校验其全部字节内容。
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

/// 查询指定 `Inode` 所属磁盘卷最近一次提交的批量读（Read Batch）递增序号。
fn batch_sequence(inode: &InodeRef) -> u64 {
    inode
        .disk_volume()
        .expect("disk readahead fixture has no volume")
        .claim()
        .ktest_read_batch_snapshot()
        .map_or(0, |snapshot| snapshot.sequence)
}

/// 校验最近一次提交的预读/批量读 I/O 批次的页数、物理地址/DMA 地址映射以及缓存页就绪状态。
///
/// # 参数说明
/// - `start`: 预期批次的起始文件页索引；
/// - `pages`: 预期批次包含的连续文件页数；
/// - `ready`: 为 `true` 时等待每页 I/O 完成并逐字节核对内容；为 `false` 时断言每页当前仍处于未就绪（`BlockError::NotReady`）的在途 DMA 状态。
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

/// 将虚拟内存操作结果 `Result<T, VmError>` 转换为测试结果，失败时输出诊断日志。
fn vm<T>(context: &TestContext<'_>, result: Result<T, VmError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("readahead-vm error={:?}", error));
        TestError("readahead VM operation failed")
    })
}

/// 普通文件预读、文件缺页与并发截断测试套件。
#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-readahead", "fs-write", "W29"), resources("root"), timeout_ms = 180000)]
pub(crate) mod reads {
    use super::*;

    /// 测试目标：验证独立 `open` 与 `dup` 在预读窗口和文件游标上的隔离/共享语义，以及顺序读阻塞期间热页缺页（Warm Fault）与冷页缺页（Cold Fault）的并发行为。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备 64 页加 37 字节的测试文件并清空页缓存；分别通过两次 `open` 获得 `first` 与 `random` 两个独立文件描述，并通过 `dup(first)` 获得共享同一个文件描述的 `shared`；预先装入第 63 页并在 `mm` 中建立只读文件映射；
    /// 2. When（初始同步预读与并发热页缺页）：挂起磁盘完成通知，在后台线程对 `first` 顺序读取第 0 页使其触发 4 页初始预读（`start=0, size=4, async_size=3`）并阻塞在磁盘 I/O 上；此时在另一线程触发 `mm` 对第 63 页的读缺页；
    /// 3. Then：第 63 页缺页无需等待被 `first` 占用的 OFD 游标锁即可立即完成并建立页表映射；恢复磁盘完成后，第 0..4 页 DMA 填充完毕且内容正确；
    /// 4. When（命中触发页开启异步预读窗口）：再次挂起磁盘完成通知，通过 `shared` 顺序读取第 1 页（触发页/Readahead Marker）；
    /// 5. Then：由于第 1 页已在缓存中，`read` 立即成功返回，同时后台异步提交第 4..12 页（8 页）的预读批次，且 `first` 与 `shared` 观测到完全一致的预读窗口，而 `random` 的预读历史仍为空；
    /// 6. When & Then（随机读与冷页缺页隔离）：在 `random` 上执行 `read_at(40)` 与 `seek + read(52)`，以及在 `mm` 上对冷页第 30 页触发缺页，验证随机读与冷缺页均只按需读取目标单页，既不触发前后相邻页预读，也不干扰 `first` 的顺序预读窗口与游标位置。
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

        // Step 1: 预先将第 63 页装入页缓存（作为暖页）并建立虚拟内存文件映射，用于稍后验证缺页处理不会被阻塞的 OFD 游标锁卡住；此时第 0 页及初始预读窗口仍为冷态。
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
        // Step 2: 挂起磁盘 I/O 完成事件，发起第 0 页顺序读以触发 4 页初始预读，并在其阻塞期间并发执行第 63 页的文件缺页。
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

        // Step 3: 验证读取已缓存的预读触发页（第 1 页）时能够立即向调用方返回数据，同时在后台异步提交下一批 8 页（第 4..12 页）的预读 Bio。
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

        // Step 4: 验证独立文件描述 `random` 上的 `pread` 与 `seek + read` 以及共享描述 `shared` 上的 `pread` 行为，确认随机单页读只更新 `prev_pos` 而不开启预读窗口。
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

        // Step 5: 验证未缓存页（第 30 页）的文件缺页（Cold Fault）会自行发起单页磁盘读取，既不借用或修改任何 OFD 的预读窗口，也不推进 OFD 的文件游标。
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

        // Step 6: 顺序读完剩余全部完整页，验证两个独立文件描述最终对同一逻辑页映射到完全相同的物理缓存页（Canonical Folio）。
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

    /// 测试目标：验证文件末尾（EOF）不足一页时的短读与页内尾部清零语义，以及并发 `truncate` 截断必须等待在途异步预读页（`Folio`）完成 DMA 填充后才能摘除缓存页并释放磁盘块。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备 64 个完整页外加 37 字节尾部的测试文件并清空页缓存；
    /// 2. When（EOF 边界短读）：从第 63 页末尾前 19 字节处发起 128 字节 `read_at` 跨越最后一页边界，并直接读取最后一页（第 64 页）的 `Folio` 内容；
    /// 3. Then：`read_at` 精确返回 `19 + 37 = 56` 字节且不触碰接收缓冲区剩余字节；最后一页 `Folio` 的前 37 字节匹配测试模式，页内 `[37..PAGE_SIZE)` 区间被严格清零，且不会越界分配第 65 页；
    /// 4. When（异步预读与并发 `truncate` 竞争）：重新清空缓存，先顺序读第 0 页完成初始 4 页预读，再挂起磁盘完成通知并读第 1 页触发第 4..12 页（8 页）异步预读；当第 4..12 页处于在途 DMA 未就绪状态时，在另一线程调用 `truncate(4 * PAGE_SIZE)`；
    /// 5. Then：`truncate` 先将 Inode 元数据大小更新为 `4 * PAGE_SIZE`，随后在尝试失效第 4 页时因该页仍由在途 Bio 持有内容锁而阻塞等待；恢复磁盘完成后，`truncate` 被唤醒并将第 4 页及后续缓存页从映射中彻底摘除，且对应磁盘块映射变为 `BlockMapping::Eof`。
    #[test]
    #[ktest(min_cpus = 2)]
    fn eof_tail_and_truncate_wait_for_async_fill(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 测试跨越最后一个完整页与尾部不完整页边界的 `read_at` 短读，并验证最后一页 Folio 的尾部零填充。
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

        // Step 2: 挂起磁盘完成通知，通过读取触发页（第 1 页）发起第 4..12 页的异步预读，并在 I/O 尚未完成时并发发起 `truncate(4 * PAGE_SIZE)`。
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

        // Step 3: 恢复磁盘 I/O 完成通知，验证 `truncate` 在异步预读 I/O 结束后成功摘除被截断的缓存页并释放底层块映射。
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

    /// 测试目标：验证在同一个包含 2 个物理页的多页父 `Bio` 读取请求中，若第 1 页读取成功而后第 2 页发生 I/O 错误，第 1 页的缓存页（`Folio`）依然正常标记为就绪可读，而第 2 页标记为错误且父 `Bio` 返回已完成前缀字节数与错误码。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：准备测试文件并清空缓存，扫描文件块映射找到一段在磁盘上物理连续的 2 页区间 `[first_page, first_page + 1]`，为这两页分配并发布冷态 `Folio` 及 `FolioRead` 令牌，构造单个覆盖 2 页（`2 * PAGE_SIZE` 字节）的读取 `Bio`；
    /// 2. When：通过 `disk.test_fail_next_file_read_after(PAGE_SIZE, BlockError::Io)` 注入故障（使本次读取在成功传输前 `PAGE_SIZE` 字节后在第 2 页失败），提交该双页 `Bio` 并等待其完成；
    /// 3. Then：父 `Bio` 的完成结果为 `IoResult { completed: PAGE_SIZE, error: Some(BlockError::Io) }`；第 1 页 `Folio` 状态变为就绪且数据与预期模式完全一致（`file.read_at` 读取成功），而第 2 页 `Folio` 状态记录为 `Err(BlockError::Io)`（`file.read_at` 返回 `FsError::Io(BlockError::Io)`）。
    #[test]
    fn multi_page_parent_keeps_successful_first_page(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 在测试文件中查找两个物理块连续的相邻文件页，以便构造单个不拆分的双页读取 Bio。
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

        // Step 2: 准备并发布这两个冷文件页到页缓存，获取独占的 `FolioRead` 填充令牌并构建对应的 DMA 物理页映射。
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
        // SAFETY:
        // 1. `memory` 中的两个 `MappedPages` 分别由上方 `publish_file` 返回的独占 `FolioRead` 令牌构造，每个令牌持有对应物理缓存页（`Folio`）的内容写锁，保证在 I/O 完成前 CPU 不会读取未初始化数据，也不会与其它读取者产生并发别名冲突；
        // 2. `sector` 已由 `inode.map_block` 与 `disk_volume().view().absolute_byte` 校验，确保连续的 `2 * PAGE_SIZE` 字节严格落在文件系统分区合法的已分配数据块范围内；
        // 3. `Bio` 接管这两个 DMA 映射的所有权，直至底层块设备完成 DMA 传输并触发完成回调后才将物理页访问权交还给页缓存，且在提交期间持有 `invalidate` 锁防止并发截断释放底层块。
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

        // Step 3: 分别校验双页 Bio 中第 1 页（成功就绪）与第 2 页（保留 I/O 错误）的页缓存状态及文件读取表现。
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
