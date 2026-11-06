//! 文件截断（Truncate）、页缓存与页表映射失效、在途回写等待以及孤儿 Inode（`nlink == 0`）生命周期回收测试套件。
//!
//! # 模块职责与架构定位
//! 本模块属于可写文件系统测试套件（`fs::ktest::user::write`）的子模块，专门验证文件尺寸截断（缩小/扩展）和未链接 Inode 最终释放全链路中的跨子系统不变量：
//! - **上游调用者**：内核测试框架（`ktest`）调度执行本模块中的 `cases` 测试集，或通过用户态辅助测试程序发起 `truncate` / `ftruncate` 系统调用。
//! - **下游依赖**：VFS 文件抽象（`File`）、Inode 缓存与生命周期管理（`Inode` / `InodeCache`）、页缓存（Page Cache / `AddressSpace`）、虚拟内存管理（`Mm` 共享/私有 VMA 反向映射与 TLB 跨核同步）、Ext4 磁盘卷块/Inode 分配器（`Ext4Volume`）以及底层块设备回写（Writeback）。
//!
//! # 核心机制与截断/回收状态机
//! 文件截断缩小与孤儿 Inode 销毁需要严格按照两轮反向映射卸载、跨核 TLB 同步（CSD）和在途 I/O 排空顺序推进，防止并发缺页、`fork` 或后台回写访问已释放的物理页或磁盘块：
//!
//! ```text
//! [步骤 0: 更新尺寸与首轮页表卸载 (First Unmap)]
//!     │  1. 更新 Inode 尺寸为新 EOF，并对文件尾部残留半页清零（Zero Tail）。
//!     │  2. 第一次扫描反向映射（rmap），卸载越界区间的用户页表叶项（PTE）并撤销部分尾页的写权限。
//!     ▼  (TruncatePhase::AfterFirstUnmap)
//! [步骤 1: 摘除页缓存与次轮兜底卸载 (Cache Invalidate & Second Unmap)]
//!     │  3. 从 AddressSpace 摘除越界物理页（Folio），使之不再是权威缓存页（Non-canonical）。
//!     │  4. (TruncatePhase::BeforeSecondUnmap) 第二次扫描反向映射，清理在步骤 0 与步骤 1 窗口期内
//!     │     因并发私有写时复制（COW）缺页或 fork 残留的页表映射；
//!     │     在跨核 TLB 刷新（FileUnmapPhase::BeforeSync -> CSD -> AfterSync）彻底完成前，
//!     │     保持被卸载物理页与页表页的引用计数不释放。
//!     ▼  (TruncatePhase::BeforeBlockRelease)
//! [步骤 2: 等待在途回写并释放磁盘块 (Wait Writeback & Block Release)]
//!     │  5. 等待被截断数据页及 Ext4 映射树元数据块上的在途回写 I/O 及完成回调彻底结束。
//!     │  6. 释放磁盘数据块与间接块/Extent 节点块，清除块位图（Block Bitmap）分配位。
//!     ▼  (TruncatePhase::AfterStorageRelease — 仅孤儿 Inode 最后一个引用释放时触发)
//! [步骤 3: 孤儿 Inode 元数据落盘与缓存槽退场 (Post-Delete Writeback & Slot Eviction)]
//!        7. 归还 Inode 位图分配位，提交清空后的磁盘 Inode 记录回写（Post-delete Metadata WB）。
//!        8. 并发分配同一 Inode 号的线程必须在 `iget` 处等待旧缓存槽完成落盘并退场，随后递增 generation 复用。
//! ```
//!
//! # 锁获取顺序与并发同步契约
//! - **业务锁层级**：遵循 VFS 与内存管理标准锁顺序——先获取 Inode 元数据/尺寸串行化上下文，再操作页缓存（`AddressSpace`）与反向映射锁，最后进入单个 `Mm` 页表锁或 Ext4 卷位图/块分配锁。
//! - **测试同步门控（`TruncateControls` / `InodeReuseControls`）**：测试钩子（Hook）回调在内核路径中触发时，仅通过原子变量（`AtomicUsize` / `AtomicBool`）记录当前进度并在 `Completion` 门控上等待，**严禁在持有自旋锁（`SpinLock`）期间阻塞等待 `Completion`**；测试局部状态（如 `before_sync`、`after_sync`、`allocation`）均在极短的 `SpinLock` 临界区内读写后立即释放锁。

use super::*;

/// 截断关键节点观察回调的类型别名。
///
/// 由 `Arc` 共享所有权并注册到目标 Inode 上；当截断流程推进到各个关键节点（[`TruncatePhase`]）时，
/// 内核以借用形式传入当前 `InodeRef` 与进度枚举，供测试观测状态或阻塞在同步门控上。
type TruncateObserver = Arc<dyn Fn(&InodeRef, TruncatePhase) + Send + Sync>;

/// 逐页校验文件在截断或扩展后的字节内容（有效前缀为 `0x65`，扩展尾部为零字节）。
///
/// # 参数
/// - `context`：测试上下文，用于执行断言。
/// - `file`：待校验的打开文件句柄（通过普通 `read_at` 读取验证）。
/// - `mapping`：可选的共享内存映射所属地址空间（`Mm`）；若提供，则同时校验虚拟地址 `SOURCE + offset` 处的映射字节。
/// - `prefix`：保留原始数据（填充为 `0x65`）的有效前缀字节数（单位：字节）。
/// - `size`：文件当前总大小（单位：字节）；`[prefix, size)` 区间内的所有字节必须为 `0x00`。
fn truncate_contents(
    context: &mut TestContext<'_>,
    file: &FileRef,
    mapping: Option<&Mm>,
    prefix: usize,
    size: usize,
) -> TestResult {
    // 按页粒度遍历 [0, size) 区间，构造每页预期字节图样并对比文件读取与内存映射视图
    for offset in (0..size).step_by(PAGE_SIZE) {
        let length = PAGE_SIZE.min(size - offset);
        let mut expected = [0u8; PAGE_SIZE];
        let valid = prefix.saturating_sub(offset).min(length);
        expected[..valid].fill(0x65);
        bytes_at(context, file, offset as u64, &expected[..length])?;
        if let Some(mm) = mapping {
            mapped_bytes(context, mm, SOURCE + offset, &expected[..length]);
        }
    }
    Ok(())
}

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
/// 文件截断、映射失效、回写等待与孤儿 Inode 回收测试用例集。
pub(super) mod cases {
    use super::*;

    /// 测试文件在块内、跨块、块边界及页边界处先截断缩小再扩展增长时，尾部清零、共享/私有映射权限同步以及冷卸载重读的一致性。
    ///
    /// - **测试目标**：验证 `truncate` 缩小文件时保留私有映射（`MAP_PRIVATE`）已触发写时复制（COW）的物理页、清除越界数据并保持文件偏移量（Offset）不变；验证再次 `truncate` 扩大文件时，新扩展区间读取为零，半页 EOF 所在的共享映射页写权限被撤销以重新受 `page_mkwrite` 追踪，且 `sync_range` 落盘并冷重启文件系统后磁盘数据与块树结构完全一致。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. **Given**：根据 `boundary` 参数（`0`=同块 `9->3->9`，`1`=跨块半尾，`2`=块边界，`3`=页边界）向冷挂载文件写入 `size` 字节的 `0x65`，建立共享可写映射（`mm`）与私有可写映射（`private`）并各自通过 `copyout` 写入脏字节触发缺页/COW。
    ///   2. **When**：将文件游标 `seek` 到 `size + 17`，调用 `truncate(prefix)` 缩小文件；随后对新尾部写入使共享页获得写权限，再调用 `truncate(size)` 扩回原大小，并通过共享映射向新尾部 `size - 1` 写入字节触发 `mkwrite` 块分配，最后执行 `sync_range` 刷盘并卸载旧文件系统实例重新挂载读取。
    ///   3. **Then**：缩小后文件大小变为 `prefix`，游标仍为 `size + 17`，私有映射的 COW 页物理帧号（PFN）与内容 `0x83` 保持不变；扩回 `size` 后，若 `prefix` 不在整页边界上则原尾页写权限被撤销，`[prefix, size)` 区间读出全零，向新尾部写入会增加 `mkwrite` 计数并完成磁盘块映射，冷重开后从底层块设备读出的文件属性、块映射树与字节内容与同步前一致。
    #[test]
    #[case("same_block_9_3_9", 0u8)]
    #[case("cross_block_partial", 1u8)]
    #[case("block_boundary", 2u8)]
    #[case("page_boundary", 3u8)]
    #[ktest(tags("W22", "fs-truncate", "fs-write-cold", "fs-shared", "fs-sync"))]
    fn shrink_grow_zero_tail_live_mapping_and_cold_reload(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        boundary: u8,
    ) -> TestResult {
        let file = fixture.cold_open(context)?.0;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        let disk = fixture.user.path.owner.disk()?.clone();
        if !disk.limits().flush {
            return Err(TestError("W22 cold reload requires real FLUSH"));
        }
        let block = volume.geometry().block_size;
        let (size, prefix) = match boundary {
            0 => (9, 3),
            1 => (block + 9, block - 3),
            2 => (2 * block, block),
            3 => (2 * PAGE_SIZE + 9, PAGE_SIZE),
            _ => unreachable!(),
        };

        // Step 1: 将文件清空并按页写入初始大小 `size` 的测试字节（0x65）
        checked(context, file.truncate(0))?;
        for offset in (0..size).step_by(PAGE_SIZE) {
            let length = PAGE_SIZE.min(size - offset);
            context.expect_eq(
                checked(
                    context,
                    file.write_at(offset as u64, &[0x65; PAGE_SIZE][..length]),
                )?,
                length,
            );
        }

        // Step 2: 建立共享可写映射（mm）与私有可写映射（private），分别写入字节以建立可写叶页表项与私有 COW 页
        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("truncate shared MM owner OOM"))?;
        vm(
            context,
            mm.map_file(
                SOURCE,
                size.div_ceil(PAGE_SIZE) * PAGE_SIZE,
                RW,
                RW,
                file.clone(),
                0,
                false,
            ),
        )?;
        context.expect_eq(mm.copyout(SOURCE + size - 1, &[0xe7]).copied, 1);
        let private = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            private.map_file(SHARED_ALIAS, PAGE_SIZE, RW, RW, file.clone(), 0, true),
        )?;
        context.expect_eq(private.copyout(SHARED_ALIAS, &[0x83]).copied, 1);
        let private_pfn = private.mapping(SHARED_ALIAS).unwrap().physical;
        // Step 3: 将文件偏移量设置到 EOF 之外，执行截断缩小到 `prefix`，验证游标不变、共享视图截断且私有 COW 页不受影响
        checked(context, file.seek(SeekFrom::Start((size + 17) as u64)))?;
        checked(context, file.truncate(prefix as u64))?;
        context.expect_eq(checked(context, file.stat())?.size, prefix as u64);
        context.expect_eq(
            checked(context, file.seek(SeekFrom::Current(0)))?,
            (size + 17) as u64,
        );
        truncate_contents(context, &file, Some(&mm), prefix, prefix)?;
        mapped_bytes(context, &private, SHARED_ALIAS, &[0x83]);
        context.expect_eq(private.mapping(SHARED_ALIAS).unwrap().physical, private_pfn);
        let mut eof = [0xa7];
        context.expect_eq(file.read_at(prefix as u64, &mut eof), Ok(0));
        context.expect_eq(eof, [0xa7]);
        // Step 4: 重新向 `prefix - 1` 写入使尾页在页表中恢复可写权限，随后将文件扩展回 `size`；
        // 若 `prefix` 位于页内中间位置，扩展操作会撤销该半页的写权限以便后续写入时重新经过 `page_mkwrite` 分配底层块，
        // 而当 `prefix` 恰好位于整页边界时，前面完整页的写权限无需撤销。
        context.expect_eq(mm.copyout(SOURCE + prefix - 1, &[0x65]).copied, 1);
        let old_tail = SOURCE + (prefix - 1) / PAGE_SIZE * PAGE_SIZE;
        context.expect_eq(mm.mapping(old_tail).unwrap().permissions.write, true);
        let grants = mm.ktest_mkwrite_count();
        checked(context, file.truncate(size as u64))?;
        if prefix % PAGE_SIZE != 0 {
            context.expect_eq(
                mm.mapping(old_tail)
                    .is_none_or(|leaf| !leaf.permissions.write),
                true,
            );
        }
        context.expect_eq(checked(context, file.stat())?.size, size as u64);
        context.expect_eq(
            checked(context, file.seek(SeekFrom::Current(0)))?,
            (size + 17) as u64,
        );
        truncate_contents(context, &file, Some(&mm), prefix, size)?;
        context.expect_eq(mm.copyout(SOURCE + size - 1, &[0x7e]).copied, 1);
        context.expect_eq(grants < mm.ktest_mkwrite_count(), true);
        context.expect_eq(
            matches!(
                checked(context, inode.map_block((size - 1) as u64 / block as u64))?,
                BlockMapping::Mapped { .. }
            ),
            true,
        );
        bytes_at(context, &file, (size - 1) as u64, &[0x7e])?;
        context.expect_eq(mm.copyout(SOURCE + size - 1, &[0]).copied, 1);
        truncate_contents(context, &file, Some(&mm), prefix, size)?;
        // Step 5: 同步文件数据与 FLUSH 命令到底层磁盘，销毁旧文件系统与页缓存实例，冷重开后验证磁盘持久化的元数据、块树与尾零内容
        drop(private);
        let before_sync = disk.test_snapshot();
        checked(context, file.sync_range(0..u64::MAX, false))?;
        context.expect_eq(
            before_sync.data_submitted < disk.test_snapshot().data_submitted,
            true,
        );
        context.expect_eq(
            disk.test_snapshot().flush_submitted - before_sync.flush_submitted,
            1,
        );
        let stat = checked(context, file.stat())?;
        let root = checked(context, inode.with_map_read(|_, root| Ok(*root)))?;
        let mapped = tree(context, &inode)?;

        let (reopened, reads_before) = fixture.cold_reopen(context, mm, file, inode, volume)?;
        let reloaded_inode = fixture.inodes.last().unwrap().clone();
        let mut reloaded_stat = checked(context, reopened.stat())?;
        context.expect_eq(reloaded_stat.device != stat.device, true);
        reloaded_stat.device = stat.device;
        context.expect_eq(reloaded_stat, stat);
        context.expect_eq(
            checked(context, reloaded_inode.with_map_read(|_, root| Ok(*root)))?,
            root,
        );
        let reloaded = tree(context, &reloaded_inode)?;
        context.expect_eq(
            (reloaded.data, reloaded.nodes, &reloaded.blocks),
            (mapped.data, mapped.nodes, &mapped.blocks),
        );
        truncate_contents(context, &reopened, None, prefix, size)?;
        context.expect_eq(reads_before < disk.test_snapshot().submitted, true);
        context.diagnostic(format_args!("W22 boundary={} block-size={} size={}->{}->{} normal-read=1 original-shared-mmap=1 private-valid-prefix-preserved=1 partial-EOF-W-revoked={} new-range-mkwrite-and-block-prepared=1 production-sync-FLUSH=1 old-FS-owners-ended=1 fresh-FS-real-device-read=1 tail-zero=1", boundary, block, size, prefix, size, prefix % PAGE_SIZE != 0));
        Ok(())
    }

    /// 测试用户态程序调用截断相关系统调用（`truncate` / `ftruncate` / `open(O_TRUNC)`）时的权限准入检查、错误码（errno）、文件大小更新与文件游标保持行为。
    ///
    /// - **测试目标**：验证读写模式下的尺寸边界检查（如 `u64::MAX` 返回 `FileTooLarge`）与正常截断语义，只读挂载点拒绝截断且不破坏原文件，以及非特权用户（`uid=1001, gid=1001`）对只读权限（`0o444`）文件发起截断时被正确拦截且原文件内容保持完好。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. **Given**：根据 `admission` 分支准备环境：`1`（`read_only_mount`）准备只读映射测试文件并记录初始 `stat` 与前 32 字节；`0`（`read_write`）打开可写预留文件；`2`（`write_permission`）写入 `"123456789"` 并将文件权限临时改为 `0o444`。
    ///   2. **When**：分别运行用户态测试程序（参数 `"v"`、`"t"` 或以 `uid=1001` 身份运行 `"h"`）发起截断系统调用，测试结束后恢复被修改的文件权限。
    ///   3. **Then**：只读挂载与无写权限场景下用户态收到预期拒绝错误码，文件大小与原有字节内容未发生任何改变；可写场景下超大尺寸截断返回 `FsError::FileTooLarge`，合法截断将文件大小更新为预期值。
    #[test]
    #[case("read_write", 0u8)]
    #[case("read_only_mount", 1u8)]
    #[case("write_permission", 2u8)]
    #[ktest(tags("W22", "fs-truncate", "user-fs"))]
    fn ordinary_truncate_open_errors_size_and_cursor(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        admission: u8,
    ) -> TestResult {
        if admission == 1 {
            // 分支 1（read_only_mount）：验证只读条件下截断被拒绝，文件元数据与前缀字节保持不变
            fixture
                .inodes
                .try_reserve_exact(16)
                .map_err(|_| TestError("readonly truncate inode slots OOM"))?;
            fixture.user.prepare(context)?;
            let file = fixture.open(context, b"/test/write-map", 0)?;
            let before = checked(context, file.stat())?;
            let mut prefix = [0; 32];
            context.expect_eq(file.read_at(0, &mut prefix), Ok(prefix.len()));

            fixture.run(context, b"v")?;
            context.expect_eq(checked(context, file.stat())?, before);
            bytes_at(context, &file, 0, &prefix)?;
        } else {
            fixture.prepare(context)?;
            let file = fixture.open(context, b"/test/write-reserve", 2)?;
            if admission == 0 {
                // 分支 0（read_write）：验证超大长度返回 FileTooLarge，随后由用户态程序完成正常截断与游标检查
                context.expect_eq(file.truncate(u64::MAX), Err(FsError::FileTooLarge));
                fixture.run(context, b"t")?;
                context.expect_eq(checked(context, file.stat())?.size, 0);
            } else {
                // 分支 2（write_permission）：将文件模式临时改为 0o444，以普通用户（uid=1001）运行测试程序验证权限拦截，之后恢复原权限
                assert_eq!(admission, 2);
                checked(context, file.truncate(0))?;
                context.expect_eq(file.write_at(0, b"123456789"), Ok(9));
                let inode = fixture.inodes.last().unwrap().clone();
                let original = checked(context, inode.metadata_snapshot())?.mode;
                checked(
                    context,
                    inode.update_metadata(
                        checked(context, inode.disk_volume())?.claim(),
                        |metadata| {
                            metadata.mode = (metadata.mode & !0o777) | 0o444;
                        },
                    ),
                )?;

                let event = fixture.user.run_named_as(
                    context,
                    PROGRAM_WRITE,
                    b"h",
                    false,
                    0,
                    Credentials {
                        uid: 1001,
                        gid: 1001,
                    },
                );
                checked(
                    context,
                    inode.update_metadata(
                        checked(context, inode.disk_volume())?.claim(),
                        |metadata| {
                            metadata.mode = original;
                        },
                    ),
                )?;
                context.expect_eq(event?.status, 0);
                bytes_at(context, &file, 0, b"123456789")?;
                context.expect_eq(checked(context, file.stat())?.size, 9);
            }
        }
        context.diagnostic(format_args!(
            "truncate-user-ABI admission={} real-user-syscalls=1 errno-and-size-checked-in-ELF=1",
            admission
        ));
        Ok(())
    }

    /// 测试文件截断缩小过程中，首轮反向映射卸载（First Unmap）与并发私有写时复制（COW）缺页或 `fork` 页表复制的交错同步，以及次轮卸载（Second Unmap）和跨核 TLB 刷新（CSD）的生命周期保障。
    ///
    /// - **测试目标**：验证在截断完成首轮页表卸载但尚未从页缓存摘除旧页的窗口期内：（1）`late_private_cow` 分支下恢复执行的私有写缺页若已完成 COW 分配并 `fork` 给子进程，次轮反向映射卸载必须清除父子进程对应的越界页表项，且在跨核 TLB 同步（CSD）彻底完成前保持被卸载的 2 个物理页及页表页引用不释放；（2）`fork_snapshot` 分支下若 `fork` 在首轮卸载前暂停快照，恢复后不得向子进程提交已越界的过期页表项。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. **Given**：向文件写入 8192 字节（2 页）并同步落盘，预先获取第 1 页的页缓存 `Folio`（`original`）保活至测试结束；注册截断观察钩子（在 `AfterFirstUnmap` 与 `BeforeSecondUnmap` 阻塞）及页表卸载同步钩子（在 `FileUnmapPhase::BeforeSync` 阻塞），并在虚拟地址 `SOURCE`（映射文件第 1 页）上挂载缺页钩子或 `fork` 钩子。
    ///   2. **When**：启动用户态进程触发 `SOURCE` 处的私有写缺页或 `fork` 并使其在钩子处暂停阻塞；后台线程发起 `truncate(PAGE_SIZE)` 完成首轮卸载并在 `AfterFirstUnmap` 处暂停；随后放行用户态进程完成 COW 写入与 `fork`，再依次放行截断线程执行页缓存摘除、次轮反向映射扫描与跨核 TLB 同步（CSD）。
    ///   3. **Then**：`fork_snapshot` 模式下父子进程在 `SOURCE` 处均无残留映射；`late_private_cow` 模式下首轮卸载后建立的父子 COW 页在次轮卸载中被彻底清除，`BeforeSync` 与 `AfterSync` 钩子观测到被保留的物理页数为 2、页表数不少于 2，最终父子进程访问 `SOURCE` 均返回 `VmError::BeyondEof`。
    #[test]
    #[case("late_private_cow", false)]
    #[case("fork_snapshot", true)]
    #[ktest(tags("W20", "fs-truncate", "fs-shared", "user-fs"), min_cpus = 2)]
    fn first_unmap_private_cow_fork_and_second_invalidation(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        fork_snapshot: bool,
    ) -> TestResult {
        // Step 1: 准备 2 页（8192 字节）文件数据并同步落盘，获取第 1 页的页缓存 Folio 引用，初始化用户态共享控制区偏移量
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        checked(context, file.truncate(0))?;
        context.expect_eq(file.write_at(0, &[0x65; 8192]), Ok(8192));
        checked(context, file.sync_range(0..u64::MAX, false))?;
        let inode = fixture.inodes.last().unwrap().clone();
        let mut attempt = ReclaimAttempt::new();
        let original = checked(
            context,
            file.mapping_inode().unwrap().file_page(1, &mut attempt),
        )?;

        let image = fixture.shared_image(context, b"k")?;
        let mm = image.mm.clone();
        let file_offset = (PAGE_SIZE as u64).to_le_bytes();
        if mm
            .copyout(SHARED_CONTROL + SHARED_FILE_OFFSET, &file_offset)
            .copied
            != file_offset.len()
        {
            return Err(TestError("truncate file offset initialization failed"));
        }

        // Step 2: 注册 Inode 截断进度钩子、页表卸载跨核同步（CSD）钩子以及 Mm 缺页/fork 暂停钩子
        let controls = TruncateControls::new()?;
        fixture.truncate_controls = Some(controls.clone());
        let stages = controls.clone();
        let hook: TruncateObserver = Arc::try_new(move |_: &InodeRef, phase| {
            let (value, gate) = match phase {
                TruncatePhase::AfterFirstUnmap => (1, Some(0)),
                TruncatePhase::BeforeSecondUnmap => (2, Some(1)),
                TruncatePhase::BeforeBlockRelease => (3, None),
                TruncatePhase::AfterStorageRelease => return,
            };
            stages.phase.store(value, Ordering::Release);
            if let Some(gate) = gate {
                if !stages.cancelled.load(Ordering::Acquire) {
                    stages.gates[gate].wait();
                }
            }
        })
        .map_err(|_| TestError("truncate phase hook OOM"))?;
        inode.ktest_set_truncate_hook(Some(hook));
        if !fork_snapshot {
            let retirement = controls.clone();
            let hook: Arc<dyn Fn(FileUnmapPhase, usize, usize) + Send + Sync> =
                Arc::try_new(move |phase, pages, tables| {
                    if pages == 0 {
                        return;
                    }
                    match phase {
                        FileUnmapPhase::BeforeSync => {
                            *retirement.before_sync.lock() = Some((pages, tables));
                            retirement.unmap_phase.store(1, Ordering::Release);
                            if !retirement.cancelled.load(Ordering::Acquire) {
                                retirement.gates[2].wait();
                            }
                        }
                        FileUnmapPhase::AfterSync => {
                            *retirement.after_sync.lock() = Some((pages, tables));
                            retirement.unmap_phase.store(2, Ordering::Release);
                        }
                    }
                })
                .map_err(|_| TestError("unmap retirement hook OOM"))?;
            inode.mapping().ktest_set_unmap_hook(Some(hook));
        }

        let ready = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("truncate fault readiness OOM"))?;
        let gate =
            Arc::try_new(Completion::new()).map_err(|_| TestError("truncate fault gate OOM"))?;
        fixture.prepare_gate = Some(gate.clone());
        let entered = ready.clone();
        let pause = gate.clone();
        let hook: Arc<dyn Fn(usize) + Send + Sync> = Arc::try_new(move |address| {
            if address == SOURCE && !entered.swap(true, Ordering::AcqRel) {
                pause.wait();
            }
        })
        .map_err(|_| TestError("truncate fault or fork hook OOM"))?;
        if fork_snapshot {
            mm.ktest_set_fork_hook(Some(hook));
        } else {
            mm.ktest_set_fault_hook(Some(hook));
        }

        // Step 3: 启动用户态进程推进至缺页或 fork 钩子处阻塞，随后发起异步 truncate(PAGE_SIZE) 完成首轮卸载并暂停
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        WriteOwner::release(&mm, 1)?;
        if fork_snapshot {
            fixture.phase(&mm, 0, 2)?;
            context.expect_eq(
                mm.mapping(SOURCE).unwrap().physical != original.phys_addr(),
                true,
            );
            WriteOwner::release(&mm, 2)?;
        }
        until("W20 original private fault or fork did not pause", || {
            ready.load(Ordering::Acquire)
        })?;
        let user = fixture
            .user
            .graph()
            .lookup_thread(fixture.user.running.as_ref().unwrap().pid)
            .ok_or(TestError("W20 private user binding missing"))?;
        let executor = user
            .executor()
            .ok_or(TestError("W20 private executor missing"))?;
        until("W20 private preparer did not really block", || {
            sched::test_state(&executor) == State::Blocked
        })?;

        let truncating = file.clone();
        let observed = fixture.consume_sync(0, move || truncating.truncate(PAGE_SIZE as u64))?;
        until("W20 truncate did not finish first unmap", || {
            controls.phase.load(Ordering::Acquire) == 1
        })?;
        context.expect_eq(checked(context, file.stat())?.size, PAGE_SIZE as u64);
        context.expect_eq(mm.mapping(SOURCE), None);

        // Step 4: 唤醒暂停的缺页或 fork 线程，验证首轮卸载后的并发 COW 建立与子进程复制行为，然后放行截断进入页缓存摘除流程
        gate.complete();
        mm.ktest_set_fault_hook(None);
        mm.ktest_set_fork_hook(None);
        fixture.prepare_gate.take();
        if !fork_snapshot {
            fixture.phase(&mm, 0, 2)?;
            let late = mm
                .mapping(SOURCE)
                .ok_or(TestError("W20 real late COW leaf missing"))?;
            context.expect_eq(late.physical != original.phys_addr(), true);
            context.expect_eq(late.permissions.write, true);
            mapped_bytes(context, &mm, SOURCE, &[0xb6]);
            WriteOwner::release(&mm, 2)?;
        }
        fixture.phase(&mm, 0, 3)?;
        fixture.phase(&mm, 8, 3)?;
        let child = fixture.child(&mm)?;
        let child_mm = child.mm().ok_or(TestError("W20 fork child MM missing"))?;
        if fork_snapshot {
            context.expect_eq(mm.mapping(SOURCE), None);
            context.expect_eq(child_mm.mapping(SOURCE), None);
        } else {
            let parent = mm
                .mapping(SOURCE)
                .ok_or(TestError("W20 parent late COW missing"))?;
            let inherited = child_mm
                .mapping(SOURCE)
                .ok_or(TestError("W20 child late COW missing"))?;
            context.expect_eq(parent.physical, inherited.physical);
            context.expect_eq(parent.physical != original.phys_addr(), true);
            context.expect_eq(
                (parent.permissions.write, inherited.permissions.write),
                (false, false),
            );
        }

        controls.gates[0].complete();
        until("W20 cache phase did not reach second unmap", || {
            controls.phase.load(Ordering::Acquire) == 2
        })?;
        context.expect_eq(inode.mapping().is_canonical(1, &original), false);
        if !fork_snapshot {
            context.expect_eq(mm.mapping(SOURCE).is_some(), true);
            context.expect_eq(child_mm.mapping(SOURCE).is_some(), true);
        }

        // Step 5: 放行次轮反向映射卸载与跨核 TLB 同步（CSD），验证被卸载物理页与页表在 CSD 前后受保护且最终父子地址空间均返回 BeyondEof
        controls.gates[1].complete();
        if !fork_snapshot {
            until(
                "W20 second sweep did not retain real owners before CSD",
                || controls.unmap_phase.load(Ordering::Acquire) == 1,
            )?;
            context.expect_eq(mm.mapping(SOURCE), None);
            context.expect_eq(child_mm.mapping(SOURCE), None);
            let before = controls
                .before_sync
                .lock()
                .ok_or(TestError("W20 real retirement observation missing"))?;
            context.expect_eq(before.0, 2);
            context.expect_eq(2 <= before.1, true);
            controls.gates[2].complete();
            until(
                "W20 CSD did not finish with original retirement owners",
                || controls.unmap_phase.load(Ordering::Acquire) == 2,
            )?;
            context.expect_eq(*controls.after_sync.lock(), Some(before));
        }
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        context.expect_eq(controls.phase.load(Ordering::Acquire), 3);
        context.expect_eq(mm.mapping(SOURCE), None);
        context.expect_eq(child_mm.mapping(SOURCE), None);
        context.expect_eq(mm.fault(SOURCE, UserAccess::Read), Err(VmError::BeyondEof));
        context.expect_eq(
            child_mm.fault(SOURCE, UserAccess::Read),
            Err(VmError::BeyondEof),
        );

        bytes_at(context, &file, 0, &[0x65; PAGE_SIZE])?;
        inode.ktest_set_truncate_hook(None);
        inode.mapping().ktest_set_unmap_hook(None);
        fixture.truncate_controls.take();
        WriteOwner::release(&mm, 3)?;
        fixture.finish_shared(context)?;
        context.diagnostic(format_args!("W20 fork-snapshot={} original-private-fault-or-fork-paused=1 first-unmap-before-resume=1 real-store-and-clone=1 second-range-invalidation=1 stale-child-snapshot-rejected={} real-retired-pages-and-tables-held-through-CSD={} no-synthetic-leaf=1", fork_snapshot, fork_snapshot, !fork_snapshot));
        drop((child_mm, child, user, mm, original));
        Ok(())
    }

    /// 测试 `ftruncate` 截断文件时必须等待在途的数据页或块映射树元数据节点回写（Writeback）完成，之后才释放磁盘块位图并允许旧物理块被安全复用。
    ///
    /// - **测试目标**：验证当被截断文件存在正在飞行的底层磁盘数据回写（`metadata = false`）或间接块/Extent 树节点元数据回写（`metadata = true`）时，用户态 `ftruncate` 会阻塞等待回写 I/O 及其完成回调彻底结束，在此期间旧数据块与元数据块在位图中保持已分配状态（不会被新块分配请求提前复用），防止在途旧 DMA 回写覆盖新分配块的数据。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. **Given**：根据 Inode 布局（传统间接块或 Extent 树）在 5 个逻辑块偏移处写入数据以构造包含元数据内部节点的块映射树，记录全部旧物理块号及对应的页缓存 `Folio`，并执行首次同步落盘。
    ///   2. **When**：再次标脏全部数据块或元数据树节点块，挂起块设备完成中断（`test_suspend_completions`）并启动后台回写使其处于飞行阻塞状态；随后放行用户态线程发起 `ftruncate(fd, 0)`，检查阻塞期间的块位图与探测分配结果，再恢复块设备完成中断让回写与截断全部执行完毕，最后向文件重新写入新数据（`0xd7`）以复用刚才释放的物理块并刷盘重读。
    ///   3. **Then**：回写飞行期间用户态 `ftruncate` 线程处于 `State::Blocked`，所有旧数据块与树节点块的位图分配位均为 `true`，探测分配的新块绝不与旧块重合；回写完成后截断返回，文件大小与块数归零，旧块位图全部清零；后续写入成功复用全部旧物理块且分配全新的页缓存物理页，底层磁盘对应物理块读出新数据 `0xd7`（未被早前的旧回写覆盖）。
    #[test]
    #[case("data", false)]
    #[case("metadata_node", true)]
    #[ktest(tags("W21", "fs-truncate", "fs-sync", "user-fs"), min_cpus = 2)]
    fn old_writeback_waits_before_bitmap_release_and_real_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        metadata: bool,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.volume();
        let disk = fixture.user.path.owner.disk()?.clone();
        if !disk.limits().flush {
            return Err(TestError("W21 requires production FLUSH"));
        }

        // Step 1: 写入 5 个跨度较大的逻辑块以生成多层块映射树（数据块 + 元数据树节点），记录所有旧物理块与页缓存 Folio 并先行同步
        checked(context, file.truncate(0))?;
        let block = volume.geometry().block_size;
        let traditional = checked(context, inode.metadata_snapshot())?.flags & codec::EXTENTS == 0;
        let logical = if traditional {
            [0, 12, 268, 65804, 65806]
        } else {
            [0, 2, 4, 6, 8]
        };
        let mut data_blocks = [0; 5];
        let mut old_folios = Vec::new();
        old_folios
            .try_reserve_exact(5)
            .map_err(|_| TestError("old WB folio slots OOM"))?;
        let mut prepare_attempt = ReclaimAttempt::new();
        for (index, logical) in logical.iter().copied().enumerate() {
            let offset = logical * block as u64;
            context.expect_eq(
                file.write_at(offset, &[0x3a; PAGE_SIZE][..block]),
                Ok(block),
            );
            let BlockMapping::Mapped { physical, .. } = checked(context, inode.map_block(logical))?
            else {
                return Err(TestError("W21 producer data not mapped"));
            };
            data_blocks[index] = physical;
            old_folios.push(checked(
                context,
                file.mapping_inode()
                    .unwrap()
                    .file_page(offset / PAGE_SIZE as u64, &mut prepare_attempt),
            )?);
        }

        let old = tree(context, &inode)?;
        context.expect_eq(old.data, 5);
        context.expect_eq(0 < old.nodes, true);
        let node = old
            .blocks
            .iter()
            .copied()
            .find(|physical| !data_blocks.contains(physical))
            .ok_or(TestError("W21 actual old tree node missing"))?;
        let node_buffer = checked(context, volume.metadata_block(node))?;
        let node_folio = node_buffer.folio().clone();
        checked(context, file.sync_range(0..u64::MAX, false))?;

        // Step 2: 启动用户态测试进程准备发起 ftruncate，标脏目标元数据块或数据块，挂起磁盘完成回调并提交回写使 I/O 停留在飞行状态
        let image = fixture.shared_image(context, b"j")?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        let user = fixture
            .user
            .graph()
            .lookup_thread(fixture.user.running.as_ref().unwrap().pid)
            .ok_or(TestError("W21 truncating user binding missing"))?;
        let executor = user
            .executor()
            .ok_or(TestError("W21 truncating executor missing"))?;
        let first_unmap = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("W21 first unmap observation OOM"))?;
        let reached = first_unmap.clone();
        let hook: TruncateObserver = Arc::try_new(move |_: &InodeRef, phase| {
            if phase == TruncatePhase::AfterFirstUnmap {
                reached.store(true, Ordering::Release);
            }
        })
        .map_err(|_| TestError("W21 truncate observer OOM"))?;
        inode.ktest_set_truncate_hook(Some(hook));

        if metadata {
            // 保持树节点原有条目与校验和合法，通过空闭包 update 将元数据缓冲区重新标记为 Dirty，触发元数据回写
            checked(
                context,
                node_buffer
                    .update(volume.claim(), |_| ())
                    .map_err(FsError::Io),
            )?;
        } else {
            for logical in logical.iter().copied() {
                context.expect_eq(
                    file.write_at(logical * block as u64, &[0x4b; PAGE_SIZE][..block]),
                    Ok(block),
                );
            }
        }

        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        let syncing_inode = inode.clone();
        let syncing_volume = volume.clone();
        let observed = fixture.consume_sync(0, move || {
            if metadata {
                crate::fs::writeback::sync_metadata(&syncing_volume)
            } else {
                crate::fs::writeback::sync_inode(&syncing_inode)
            }
        })?;
        until("W21 real original WB did not accept IO and block", || {
            let snapshot = disk.test_snapshot();
            let submitted = if metadata {
                before.metadata_submitted < snapshot.metadata_submitted
                    && node_folio.writeback_sectors() != 0
            } else {
                before.data_submitted < snapshot.data_submitted
                    && old_folios
                        .iter()
                        .any(|folio| folio.writeback_sectors() != 0)
            };
            submitted && snapshot.inflight != 0 && observed.blocked()
        })?;

        // Step 3: 放行用户态 ftruncate，验证其阻塞等待在途回写，且阻塞期间旧物理块位图仍标记为占用、不会被新块分配请求提前拿走
        WriteOwner::release(&mm, 1)?;
        until(
            "W21 ordinary ftruncate did not block on original IO",
            || {
                sched::test_state(&executor) == State::Blocked
                    || mm.load_user_u32(SHARED_CONTROL) == Ok(2)
            },
        )?;
        context.expect_eq(sched::test_state(&executor), State::Blocked);
        context.expect_eq(mm.load_user_u32(SHARED_CONTROL), Ok(1));
        if !metadata {
            until(
                "W21 data wait began before new size and first unmap",
                || first_unmap.load(Ordering::Acquire),
            )?;
            context.expect_eq(checked(context, file.stat())?.size, 0);
        }
        for physical in &old.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(true),
            );
        }
        let hint = (inode.number() - 1) / volume.geometry().inodes_per_group;
        let probe = checked(context, volume.allocate_block(hint))?;
        let allocated = probe.physical();
        let excluded = !old.blocks.contains(&allocated);
        volume.release_unpublished_block(probe);
        context.expect_eq(excluded, true);
        context.expect_eq(disk.test_snapshot().inflight != 0, true);
        context.expect_eq(observed.result.lock().is_none(), true);
        context.diagnostic(format_args!("W21 kind={} completion-consumption-paused=1 ordinary-sys46-blocked=1 first-unmap-observed={} old-data-and-node-bitmap-used=1 real-allocation-excludes-old-blocks=1 allocated-probe={} hardware-DMA-inflight-claim=0", if metadata { "metadata-node" } else { "data" }, first_unmap.load(Ordering::Acquire), allocated));

        // Step 4: 恢复磁盘完成回调并排空工作队列，验证截断彻底完成后旧块位图归还，随后写入新数据可安全复用全部旧物理块且分配新 Folio
        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        fixture.phase(&mm, 0, 2)?;
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        context.expect_eq(checked(context, file.stat())?.size, 0);
        context.expect_eq(checked(context, inode.metadata_snapshot())?.blocks, 0);
        context.expect_eq(inode.mapping().is_empty(), true);
        for physical in &old.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(false),
            );
        }

        drop((node_buffer, node_folio));
        let replacement = [0xd7; PAGE_SIZE];
        for (logical, physical) in old.blocks.iter().copied().enumerate() {
            context.expect_eq(
                file.write_at((logical * block) as u64, &replacement[..block]),
                Ok(block),
            );
            let BlockMapping::Mapped {
                physical: reused, ..
            } = checked(context, inode.map_block(logical as u64))?
            else {
                return Err(TestError("W21 released block not reused as real file data"));
            };
            context.expect_eq(reused, physical);
            bytes_at(
                context,
                &file,
                (logical * block) as u64,
                &replacement[..block],
            )?;
        }
        let mut reuse_attempt = ReclaimAttempt::new();
        let fresh = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(0, &mut reuse_attempt),
        )?;
        context.expect_eq(
            old_folios
                .iter()
                .all(|old| old.phys_addr() != fresh.phys_addr()),
            true,
        );

        checked(context, file.sync_range(0..u64::MAX, false))?;
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        let mut device_bytes = [0; PAGE_SIZE];
        for physical in &old.blocks {
            checked(
                context,
                volume
                    .claim()
                    .read_exact_at(*physical * block as u64, &mut device_bytes[..block])
                    .map_err(FsError::Io),
            )?;
            context.expect_eq(&device_bytes[..block], &replacement[..block]);
        }

        inode.ktest_set_truncate_hook(None);
        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        context.diagnostic(format_args!("W21 kind={} real-old-IO-and-Work-tails-ended=1 released-data={} released-nodes={} actual-all-old-physical-block-reuse=1 detached-folio-not-reattached=1 production-sync-and-device-readback=1 old-write-does-not-overwrite-new-data=1", if metadata { "metadata-node" } else { "data" }, old.data, old.nodes));
        Ok(())
    }

    /// 测试合法已发布但未链接（`nlink == 0`）的孤儿 Inode 在 InodeCache 引用、打开文件描述符（OFD）、内存映射（VMA）、外部页缓存 `Folio` 引用以及在途回写 I/O 依次释放时的生命周期管理与资源回收。
    ///
    /// - **测试目标**：验证上述任意一类持有者存活期间，孤儿 Inode 的缓存槽位、磁盘块与 Inode 位图均保持占用；仅当最后一个持有者（本例中为在途回写 I/O 及其完成回调）彻底退出后，才在可睡眠任务上下文中触发 Inode 驱逐，归还数据块位图与 Inode 位图，并允许该 Inode 号被新分配请求安全复用。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. **Given**：分配并发布一个硬链接数为 0（`links == 0`）的新 Inode，通过未哈希路径打开为可写 `File`，写入 1 页数据，获取其第 0 页的 `Folio` 句柄，建立共享内存映射（`mm`），并挂起磁盘完成中断后发起该页的后台回写使其处于飞行阻塞状态。
    ///   2. **When**：按顺序依次释放 `InodeCache` 持有引用、打开文件句柄 `file`、虚拟内存空间 `mm` 以及外部 `Folio` 引用，每次释放后观测驱逐状态与位图；最后恢复磁盘完成中断，等待在途回写及其工作队列回调执行完毕，并尝试重新分配一个新的 Inode。
    ///   3. **Then**：在前四类引用依次释放后，由于飞行回写仍持有 Inode 引用，缓存槽位、Inode 位图与数据块位图始终保持占用（`true`）；当回写彻底完成后，缓存槽被移除，Inode 位图与数据块位图全部归还（`false`），卷空闲计数恢复至初始值，且随后的 `allocate_inode` 成功复用了同一个 Inode 号。
    #[test]
    #[ktest(tags("fs-truncate", "fs-inode-lifetime", "fs-sync"), min_cpus = 2)]
    fn unlinked_last_ofd_vma_folio_and_io_release_canonical_inode(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        // Step 1: 验证已链接 Inode 拒绝构造 unhashed 路径，随后分配并发布一个 nlink == 0 的孤儿 Inode，打开文件写入 1 页数据并获取 Folio 与共享 VMA 映射
        let (linked, cold_vfs) = fixture.cold_open(context)?;
        let volume = fixture.cold_volume.as_ref().unwrap().clone();
        let cache = fixture.cold_cache.as_ref().unwrap().clone();
        let linked_inode = fixture.inodes.last().unwrap().clone();
        context.expect_eq(
            linked
                .path()
                .unwrap()
                .unhashed_inode(linked_inode.clone())
                .err(),
            Some(FsError::InvalidOperation),
        );
        let before = allocation_counts(context, &volume)?;

        let token = checked(context, volume.allocate_inode(0, 0o100600, 0, 0))?;
        let number = token.number();
        let inode = match cache.iget(&volume, number) {
            Ok(inode) => {
                context.expect_eq(token.publish(), number);
                inode
            }
            Err(error) => {
                volume.release_unpublished_inode(token);
                return checked(context, Err(error));
            }
        };
        fixture.inodes.push(inode.clone());
        fixture.eviction = Some(cache.watch());
        let path = checked(
            context,
            linked.path().unwrap().unhashed_inode(inode.clone()),
        )?;
        let file = checked(
            context,
            open_path_with_flags(&cold_vfs, path, 2, FileIoContext::Kernel),
        )?;
        drop(cold_vfs);
        context.expect_eq(checked(context, file.stat())?.links, 0);
        context.expect_eq(file.write_at(0, &[0x91; PAGE_SIZE]), Ok(PAGE_SIZE));
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("unlinked live VMA MM owner OOM"))?;
        vm(
            context,
            mm.map_file(SOURCE, PAGE_SIZE, RW, RW, file.clone(), 0, false),
        )?;
        context.expect_eq(mm.copyout(SOURCE + 1, &[0x72]).copied, 1);
        let mapped = tree(context, &inode)?;
        context.expect_eq(checked(context, file.stat())?.size, PAGE_SIZE as u64);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );

        // Step 2: 挂起块设备完成回调并提交孤儿 Inode 数据页回写，使底层回写 I/O 处于飞行阻塞状态
        let disk = fixture.user.path.owner.disk()?.clone();
        let ranges = checked(context, prepare_unlinked_original_writeback(&inode, &folio))?;
        disk.test_suspend_completions();
        let io_volume = volume.clone();
        let observed = fixture.consume_sync(0, move || {
            let mut ranges = ranges;
            let mut bios = [const { None }; 8];
            let mut completions = [const { None }; 8];
            io_volume
                .claim()
                .writeback_batch(
                    &mut ranges,
                    None,
                    &mut bios,
                    &mut completions,
                    io_volume.writeback_domain(),
                    || false,
                )
                .map_err(FsError::from)
        })?;
        until(
            "unlinked inode original data WB did not really block",
            || {
                folio.writeback_sectors() != 0
                    && disk.test_snapshot().inflight != 0
                    && observed.blocked()
            },
        )?;

        // Step 3: 依次释放缓存句柄、OFD（file）、VMA（mm）与外部 Folio 引用，验证在途回写单独保活 Inode 及所有磁盘位图
        fixture.inodes.clear();
        drop(inode);
        drop(fixture.cold_cache.take());
        drop(cache);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        context.expect_eq(checked(context, file.stat())?.size, PAGE_SIZE as u64);

        drop(file);
        mapped_bytes(context, &mm, SOURCE, &[0x91, 0x72]);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );

        drop(mm);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        context.expect_eq(folio.writeback_sectors() != 0, true);

        drop(folio);
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        for physical in &mapped.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(true),
            );
        }
        context.diagnostic(format_args!("inode-last-use number={} legitimate-published-nlink0=1 unhashed-real-OFD-and-VMA=1 cache-owner-ended=1 OFD-then-VMA-then-folio-ended=1 original-IO-credential-only-owns-inode=1 bitmap-and-size-preserved=1", number));

        // Step 4: 恢复磁盘完成中断让在途回写彻底结束，验证任务上下文完成最终驱逐、归还块与 Inode 位图，且同一 Inode 号可被重新分配
        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        disk.test_flush_completions();
        checked(context, fixture.eviction.as_ref().unwrap().wait())?;
        context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), false);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            false,
        );
        for physical in &mapped.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(false),
            );
        }

        drop((linked, linked_inode));
        checked(context, fixture.eviction.as_ref().unwrap().wait_retired())?;
        let retired_counts = allocation_counts(context, &volume)?;
        context.expect_eq(retired_counts, before);

        let reused = checked(context, volume.allocate_inode(0, 0o100600, 123, 234))?;
        let reused_number = reused.number();
        let reloaded = volume.load_inode(reused_number, None);
        volume.release_unpublished_inode(reused);
        context.expect_eq(reused_number, number);
        let reloaded = checked(context, reloaded)?;
        context.expect_eq(
            (
                reloaded.size,
                reloaded.blocks,
                reloaded.links,
                reloaded.uid,
                reloaded.gid,
            ),
            (0, 0, 0, 123, 234),
        );

        checked(context, crate::fs::writeback::sync_metadata(&volume))?;
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        let final_counts = allocation_counts(context, &volume)?;
        context.expect_eq(final_counts, before);
        drop(fixture.eviction.take());
        context.diagnostic(format_args!("inode-last-use number={} original-IO-and-Work-tails-ended=1 canonical-slot-removed=1 task-context-eviction=1 file-blocks-and-inode-bitmap-returned=1 actual-inode-number-reuse=1 production-metadata-sync=1", number));
        Ok(())
    }

    /// 孤儿 Inode 存储释放后与复用分配之间的测试同步门控。
    struct InodeReuseControls {
        /// 标识被删除 Inode 是否已到达 `TruncatePhase::AfterStorageRelease` 节点（`0` 为未到达，`1` 为已到达）。
        phase: AtomicUsize,
        /// 异常退出或测试结束时的取消标志，防止钩子回调永久阻塞在 `gate` 上。
        cancelled: AtomicBool,
        /// 控制被删除 Inode 从 `AfterStorageRelease` 继续推进到删除后元数据回写（Post-delete Metadata WB）的同步门。
        gate: Completion,
    }

    /// 已完成分配并在 `InodeCache` 中发布登记的权威 Inode 结果。
    struct ReusedCanonicalInode {
        /// 分配得到的 Ext4 Inode 编号。
        number: u64,
        /// 在调用 `cache.iget` 之前，旧 Inode 在缓存中的槽位是否已经移除。
        slot_removed_before_iget: bool,
        /// 从 `InodeCache` 获取并已正式发布（`token.publish()`）的权威 Inode 引用。
        inode: InodeRef,
    }

    /// 在指定 Ext4 卷上分配一个新 Inode，通过同一 `InodeCache` 获取其权威缓存实例并完成正式发布。
    ///
    /// 若 `cache.iget` 失败（例如旧缓存槽的删除后元数据回写失败），则自动归还未发布的临时 Inode 预留令牌。
    fn allocate_published_inode(
        volume: &Arc<Ext4Volume>,
        cache: &Arc<InodeCache>,
    ) -> Result<ReusedCanonicalInode, FsError> {
        let watch = cache.watch();
        let token = volume.allocate_inode(0, 0o100600, 123, 234)?;
        let number = token.number();
        let slot_removed_before_iget = !watch.contains(number);
        let inode = match cache.iget(volume, number) {
            Ok(inode) => inode,
            Err(error) => {
                volume.release_unpublished_inode(token);
                return Err(error);
            }
        };
        Ok(ReusedCanonicalInode {
            number: token.publish(),
            slot_removed_before_iget,
            inode,
        })
    }

    /// 测试未链接（`nlink == 0`）Inode 释放磁盘块与位图后，并发复用该 Inode 号的分配者必须等待删除后元数据回写（Post-delete Metadata WB）及旧缓存槽退场，并验证元数据回写失败时的回滚与重试恢复。
    ///
    /// - **测试目标**：验证当旧孤儿 Inode 已归还位图但尚未完成磁盘 Inode 记录清零回写时，新分配者虽然能在位图中预留同一 Inode 号，但在 `cache.iget` 获取权威缓存槽时必须阻塞等待旧槽完成回写并退场；若回写成功（`running`），新 Inode 继承该编号并将 `generation` 加 1；若回写发生 I/O 错误（`failed`），新分配者的 `iget` 收到原始错误并自动回滚位图预留，旧缓存槽保留在缓存中直到显式调用 `prune_unused()` 重试回写成功后才允许再次分配复用。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. **Given**：分配并发布一个 `links == 0` 的孤儿 Inode，记录其初始 `generation` 与磁盘记录快照；注册 `TruncatePhase::AfterStorageRelease` 钩子以在位图归还后、删除后元数据回写提交前暂停驱逐流程。
    ///   2. **When**：丢弃该孤儿 Inode 的全部引用使其暂停在 `AfterStorageRelease`；启动后台分配线程调用 `allocate_published_inode` 使其预留同一 Inode 号并在 `cache.iget` 处阻塞等待旧槽退场；随后挂起磁盘完成中断（在 `failed` 分支下注入一次元数据 I/O 错误 `test_fail_next_metadata`），放行旧 Inode 提交删除后元数据回写，再恢复磁盘完成中断观测结果。
    ///   3. **Then**：`running` 分支下回写完成后旧槽退场，后台分配者被唤醒并成功返回新 Inode，`generation` 为 `old_generation + 1`；`failed` 分支下回写报错导致驱逐与后台 `iget` 均返回 `FsError::Io(BlockError::Io)`，预留位图被回滚且旧槽仍留在缓存中，随后调用 `cache.prune_unused()` 重新提交元数据回写并移除旧槽，再次调用 `allocate_published_inode` 成功复用该编号并将 `generation` 加 1。
    #[test]
    #[case("running", false)]
    #[case("failed", true)]
    #[ktest(tags("fs-truncate", "fs-inode-lifetime", "fs-sync"), min_cpus = 2)]
    fn unlinked_inode_reuse_waits_for_post_delete_writeback(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        fail_metadata: bool,
    ) -> TestResult {
        let disk = fixture.user.path.owner.disk()?.clone();
        let controls = Arc::try_new(InodeReuseControls {
            phase: AtomicUsize::new(0),
            cancelled: AtomicBool::new(false),
            gate: Completion::new(),
        })
        .map_err(|_| TestError("inode reuse controls OOM"))?;
        let allocation = Arc::try_new(SpinLock::new(None::<Result<ReusedCanonicalInode, FsError>>))
            .map_err(|_| TestError("inode reuse result OOM"))?;
        let body = (|| -> TestResult {
            // Step 1: 分配并发布初始孤儿 Inode（links == 0），注册 AfterStorageRelease 暂停钩子，随后释放全部引用使其停在位图已归还但缓存槽未退场的位置
            let (linked, cold_vfs) = fixture.cold_open(context)?;
            let volume = fixture.cold_volume.as_ref().unwrap().clone();
            let cache = fixture.cold_cache.as_ref().unwrap().clone();
            let linked_inode = fixture.inodes.last().unwrap().clone();
            let before = allocation_counts(context, &volume)?;
            let original = checked(context, allocate_published_inode(&volume, &cache))?;
            context.expect_eq(original.slot_removed_before_iget, true);
            let number = original.number;
            let original_metadata = checked(context, original.inode.metadata_snapshot())?;
            context.expect_eq(
                checked(context, inode_allocation_bit(&volume, number))?,
                true,
            );
            let allocated_counts = allocation_counts(context, &volume)?;
            context.expect_eq(allocated_counts, (before.0, before.1 - 1));

            let path = checked(
                context,
                linked
                    .path()
                    .unwrap()
                    .unhashed_inode(original.inode.clone()),
            )?;
            let file = checked(
                context,
                open_path_with_flags(&cold_vfs, path, 2, FileIoContext::Kernel),
            )?;
            drop(cold_vfs);
            context.expect_eq(checked(context, file.stat())?.links, 0);

            let paused = controls.clone();
            let hook: TruncateObserver = Arc::try_new(move |_: &InodeRef, phase| {
                if phase != TruncatePhase::AfterStorageRelease {
                    return;
                }
                paused.phase.store(1, Ordering::Release);
                if !paused.cancelled.load(Ordering::Acquire) {
                    paused.gate.wait();
                }
            })
            .map_err(|_| TestError("inode reuse truncate hook OOM"))?;
            original.inode.ktest_set_truncate_hook(Some(hook));
            fixture.eviction = Some(cache.watch());
            drop((file, linked, linked_inode));
            fixture.inodes.clear();
            drop(original.inode);
            until("inode reuse did not reach real storage release", || {
                controls.phase.load(Ordering::Acquire) == 1
            })?;
            disk.test_flush_completions();
            let released_counts = allocation_counts(context, &volume)?;
            context.expect_eq(released_counts, before);
            let released_bit = checked(context, inode_allocation_bit(&volume, number))?;
            context.expect_eq(released_bit, false);
            context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
            context.expect_eq(disk.test_snapshot().inflight, 0);
            context.expect_eq(disk.test_completion_error_pending(), false);

            let (old_record, record_size) = inode_record_snapshot(context, &volume, number)?;
            let old_generation = checked(context, codec::le32(&old_record[..record_size], 100))?;
            context.expect_eq(old_generation, original_metadata.generation);
            // Step 2: 启动并发分配者，验证其在位图中重新预留该 Inode 号后，在 cache.iget 处阻塞等待旧权威缓存槽退场
            let result = allocation.clone();
            let actor_volume = volume.clone();
            let actor_cache = cache.clone();
            let observed = fixture.consume_sync(0, move || {
                *result.lock() = Some(allocate_published_inode(&actor_volume, &actor_cache));
                Ok(())
            })?;
            until("inode reuse allocator neither blocked nor returned", || {
                observed.blocked() || allocation.lock().is_some()
            })?;
            if !observed.blocked() || allocation.lock().is_some() {
                return Err(TestError("inode reuse allocator missed canonical wait"));
            }
            context.expect_eq(*observed.result.lock(), None);
            context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
            let reserved_bit = checked(context, inode_allocation_bit(&volume, number))?;
            context.expect_eq(reserved_bit, true);
            let reserved_counts = allocation_counts(context, &volume)?;
            context.expect_eq(reserved_counts.0, released_counts.0);
            context.expect_eq(reserved_counts.1 + 1, released_counts.1);
            let (waiting_record, waiting_size) = inode_record_snapshot(context, &volume, number)?;
            context.expect_eq(waiting_size, record_size);
            context.expect_eq(&waiting_record[..record_size], &old_record[..record_size]);
            context.expect_eq(
                codec::le32(&waiting_record[..record_size], 100),
                Ok(old_generation),
            );

            // Step 3: 挂起磁盘完成回调（可选注入元数据 I/O 失败），放行旧 Inode 发起删除后元数据回写，随后恢复磁盘完成回调等待回写结束
            disk.test_suspend_completions();
            let before_writeback = disk.test_snapshot();
            context.expect_eq(before_writeback.inflight, 0);
            if fail_metadata {
                disk.test_fail_next_metadata(BlockError::Io);
            }
            controls.gate.complete();
            until(
                "post-delete metadata WB did not submit a real in-flight request",
                || {
                    let current = disk.test_snapshot();
                    before_writeback.metadata_submitted < current.metadata_submitted
                        && before_writeback.inflight < current.inflight
                },
            )?;
            context.expect_eq(observed.blocked(), true);
            context.expect_eq(allocation.lock().is_none(), true);
            context.expect_eq(disk.test_completion_error_pending(), fail_metadata);

            disk.test_resume_completions();
            let eviction = fixture.eviction.as_ref().unwrap().wait();
            until("inode reuse metadata slot tail did not end", || {
                let current = disk.test_snapshot();
                current.inflight == 0 && before_writeback.completed < current.completed
            })?;
            disk.test_flush_completions();
            let completed = disk.test_snapshot();
            context.expect_eq(completed.inflight, 0);
            context.expect_eq(before_writeback.completed < completed.completed, true);
            context.expect_eq(disk.test_completion_error_pending(), false);

            // Step 4: 验证回写失败分支（错误传播、位图预留回滚、prune_unused 显式重试恢复）与正常完成分支下的 Inode 复用及 generation 递增
            let reused = if fail_metadata {
                until(
                    "inode reuse allocator did not return the real metadata error",
                    || allocation.lock().is_some(),
                )?;
                let attempt = allocation
                    .lock()
                    .take()
                    .ok_or(TestError("inode reuse allocation result missing"))?;
                let expected = FsError::Io(BlockError::Io);
                let error = match attempt {
                    Err(error) => error,
                    Ok(reused) => {
                        drop(reused);
                        return Err(TestError("failed inode reuse unexpectedly succeeded"));
                    }
                };
                if eviction != Err(expected) || error != expected {
                    context.expect_eq(eviction, Err(expected));
                    context.expect_eq(error, expected);
                    return Err(TestError(
                        "inode reuse failure did not preserve the original error",
                    ));
                }
                context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), true);
                let cancelled_bit = checked(context, inode_allocation_bit(&volume, number))?;
                let cancelled_counts = allocation_counts(context, &volume)?;
                context.expect_eq(cancelled_bit, false);
                context.expect_eq(cancelled_counts, released_counts);
                let (failed_record, failed_size) = inode_record_snapshot(context, &volume, number)?;
                context.expect_eq(failed_size, record_size);
                context.expect_eq(&failed_record[..record_size], &old_record[..record_size]);
                context.expect_eq(
                    codec::le32(&failed_record[..record_size], 100),
                    Ok(old_generation),
                );

                let before_retry = disk.test_snapshot();
                let pruned = cache.prune_unused();
                context.expect_eq(0 < pruned, true);
                disk.test_flush_completions();
                let recovered = disk.test_snapshot();
                context.expect_eq(
                    before_retry.metadata_submitted < recovered.metadata_submitted,
                    true,
                );
                context.expect_eq(before_retry.completed < recovered.completed, true);
                context.expect_eq(recovered.inflight, 0);
                context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), false);
                let recovered_bit = checked(context, inode_allocation_bit(&volume, number))?;
                let recovered_counts = allocation_counts(context, &volume)?;
                context.expect_eq(recovered_bit, false);
                context.expect_eq(recovered_counts, released_counts);
                let (retried_record, retried_size) =
                    inode_record_snapshot(context, &volume, number)?;
                context.expect_eq(retried_size, record_size);
                context.expect_eq(&retried_record[..record_size], &old_record[..record_size]);
                checked(context, allocate_published_inode(&volume, &cache))?
            } else {
                checked(context, eviction)?;
                until(
                    "inode reuse allocator did not resume after slot removal",
                    || allocation.lock().is_some(),
                )?;
                context.expect_eq(*observed.result.lock(), Some(Ok(())));
                let attempt = allocation
                    .lock()
                    .take()
                    .ok_or(TestError("inode reuse allocation result missing"))?;
                checked(context, attempt)?
            };
            context.expect_eq(reused.number, number);
            context.expect_eq(reused.slot_removed_before_iget, true);
            let metadata = checked(context, reused.inode.metadata_snapshot())?;
            context.expect_eq(metadata.number, number);
            context.expect_eq(metadata.generation, old_generation.wrapping_add(1));
            context.expect_eq(
                (metadata.mode, metadata.uid, metadata.gid),
                (0o100600, 123, 234),
            );
            context.expect_eq((metadata.links, metadata.size, metadata.blocks), (0, 0, 0));

            drop(reused.inode);
            checked(context, fixture.eviction.as_ref().unwrap().wait())?;
            disk.test_flush_completions();
            context.expect_eq(disk.test_snapshot().inflight, 0);
            context.expect_eq(fixture.eviction.as_ref().unwrap().contains(number), false);
            let final_bit = checked(context, inode_allocation_bit(&volume, number))?;
            let final_counts = allocation_counts(context, &volume)?;
            context.expect_eq(final_bit, false);
            context.expect_eq(final_counts, before);
            drop(fixture.eviction.take());
            context.diagnostic(format_args!(
                "inode-reuse phase={} reserved-before-canonical-wait=1 real-post-delete-metadata-WB=1 failed-slot-retried-explicitly={} generation-and-credentials=1 final-resources-restored=1",
                if fail_metadata { "failed" } else { "running" },
                fail_metadata
            ));
            Ok(())
        })();

        controls.cancelled.store(true, Ordering::Release);
        controls.gate.complete();
        disk.test_resume_completions();
        disk.test_flush_completions();
        let joined = fixture.join_consumers();
        drop(allocation.lock().take());
        if let Err(error) = body {
            joined?;
            return Err(error);
        }
        joined?;
        Ok(())
    }

    /// 测试当虚拟内存区间（VMA）被覆盖映射（文件映射替换、匿名映射替换）或 `munmap` 卸载，且在旧页表叶项尚未清理前暂停时，并发截断旧文件仍能通过反向映射找到并清理旧页表项，并在跨核 TLB 同步（CSD）完成前保有物理页与页表引用。
    ///
    /// - **测试目标**：验证在 `mmap(MAP_FIXED)` 覆盖或 `munmap` 操作已发布新 VMA 元数据、但正阻塞在页表叶项清理钩子（`vma_unmap_hook`）的窗口期内，旧文件对应的反向映射登记依然有效；此时对旧文件执行 `truncate(0)` 能够定位并清除 `SOURCE` 处的旧页表叶项，在 `FileUnmapPhase::BeforeSync` 到 `AfterSync`（CSD）期间安全持有 1 个物理页及至少 2 级页表的引用，且后续放行 VMA 替换线程后新映射缺页或 `munmap` 状态完全正常。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. **Given**：向旧文件写入 1 页 `0x4f` 并落盘，获取其第 0 页 `old_folio`，将其只读映射到 `mm` 的 `SOURCE` 地址并触发缺页建立叶页表项；若 `replacement == 0`（`file_to_file`），同时准备新文件及其第 0 页 `new_folio`；在 `mm` 上注册 `vma_unmap_hook` 并在 `old_inode` 上注册 `unmap_hook`。
    ///   2. **When**：启动后台线程在 `SOURCE` 处执行新文件 `mmap(Fixed)`、私有匿名 `mmap(Fixed)` 或 `unmap`，使其在 `vma_unmap_hook` 处暂停（此时旧叶页表项仍指向 `old_folio`）；启动另一后台线程对旧文件执行 `truncate(0)`，观测 `BeforeSync` 与 `AfterSync` 计数后完成截断；最后放行 `vma_unmap_hook` 让 VMA 替换/卸载线程完成，并对 `SOURCE` 发起读缺页验证。
    ///   3. **Then**：旧文件 `truncate(0)` 通过反向映射成功清除 `SOURCE` 处的旧叶页表项，CSD 同步前后均观测到保活 1 个物理页和不少于 2 个页表对象；VMA 操作完成后，`munmap` 分支下访问 `SOURCE` 返回 `VmError::Unmapped`，文件替换分支下缺页映射到 `new_folio` 并读出新文件数据，匿名替换分支下缺页映射到非 `old_folio` 的零页。
    #[test]
    #[case("file_to_file", 0u8)]
    #[case("file_to_anonymous", 1u8)]
    #[case("munmap", 2u8)]
    #[ktest(tags("fs-truncate", "vm-file-retirement", "fs-shared"), min_cpus = 2)]
    fn mandatory_vma_leaf_retirement_preserves_old_file_source(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
        replacement: u8,
    ) -> TestResult {
        // Step 1: 准备旧文件（填充 0x4f）及可选替换用的新文件，各自获取第 0 页 Folio，在 mm 的 SOURCE 处建立旧文件映射并注册 VMA 卸载与文件反向映射钩子
        fixture.prepare(context)?;
        let old = fixture.open(context, b"/test/write-reserve", 2)?;
        checked(context, old.truncate(0))?;
        context.expect_eq(old.write_at(0, &[0x4f; PAGE_SIZE]), Ok(PAGE_SIZE));
        checked(context, old.sync_range(0..u64::MAX, false))?;
        let old_inode = fixture.inodes.last().unwrap().clone();
        let mut old_attempt = ReclaimAttempt::new();
        let old_folio = checked(
            context,
            old.mapping_inode().unwrap().file_page(0, &mut old_attempt),
        )?;
        let mut expected = [0; 32];
        let (new, new_folio) = if replacement == 0 {
            let file = fixture.open(context, b"/test/write-map", 0)?;
            context.expect_eq(file.read_at(0, &mut expected), Ok(expected.len()));
            let mut new_attempt = ReclaimAttempt::new();
            let folio = checked(
                context,
                file.mapping_inode().unwrap().file_page(0, &mut new_attempt),
            )?;
            (Some(file), Some(folio))
        } else {
            (None, None)
        };

        let mm = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("mandatory retirement MM owner OOM"))?;
        vm(
            context,
            mm.map_file(SOURCE, PAGE_SIZE, RO, RO, old.clone(), 0, false),
        )?;
        mapped_bytes(context, &mm, SOURCE, &[0x4f; 32]);
        context.expect_eq(mm.mapping(SOURCE).unwrap().physical, old_folio.phys_addr());
        fixture.shared_mm = Some(mm.clone());

        let ready = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("mandatory retirement ready OOM"))?;
        let gate = Arc::try_new(Completion::new())
            .map_err(|_| TestError("mandatory retirement gate OOM"))?;
        fixture.prepare_gate = Some(gate.clone());
        let entered = ready.clone();
        let pause = gate.clone();
        let hook: Arc<dyn Fn(usize, usize) + Send + Sync> = Arc::try_new(move |start, end| {
            if start == SOURCE && end == SOURCE + PAGE_SIZE && !entered.swap(true, Ordering::AcqRel)
            {
                pause.wait();
            }
        })
        .map_err(|_| TestError("mandatory retirement VMA hook OOM"))?;
        mm.ktest_set_vma_unmap_hook(Some(hook));
        let controls = TruncateControls::new()?;
        fixture.truncate_controls = Some(controls.clone());
        let retirement = controls.clone();
        let hook: Arc<dyn Fn(FileUnmapPhase, usize, usize) + Send + Sync> =
            Arc::try_new(move |phase, pages, tables| {
                if pages == 0 {
                    return;
                }
                match phase {
                    FileUnmapPhase::BeforeSync => {
                        *retirement.before_sync.lock() = Some((pages, tables));
                        retirement.unmap_phase.store(1, Ordering::Release);
                        if !retirement.cancelled.load(Ordering::Acquire) {
                            retirement.gates[2].wait();
                        }
                    }
                    FileUnmapPhase::AfterSync => {
                        *retirement.after_sync.lock() = Some((pages, tables));
                        retirement.unmap_phase.store(2, Ordering::Release);
                    }
                }
            })
            .map_err(|_| TestError("mandatory retirement ownership hook OOM"))?;
        old_inode.mapping().ktest_set_unmap_hook(Some(hook));

        // Step 2: 启动后台任务在 SOURCE 处执行覆盖 mmap 或 munmap，并使其在 vma_unmap_hook 处暂停（此时旧页表叶项尚未被 VMA 路径清除）
        let result = Arc::try_new(SpinLock::new(None::<Result<(), VmError>>))
            .map_err(|_| TestError("mandatory retirement result OOM"))?;
        let recorded = result.clone();
        let replacing = mm.clone();
        let file = new;
        fixture.consumers[0] = Some(
            task::prepare_kernel_on(cpu::current().id, move || {
                let operation = if replacement == 2 {
                    replacing.unmap(SOURCE, PAGE_SIZE)
                } else {
                    let source = if replacement == 0 {
                        let file = file.expect("file replacement has no original OFD");
                        let inode = file
                            .mapping_inode()
                            .expect("regular replacement has no mapping inode")
                            .clone();
                        MmapSource::File {
                            file,
                            inode,
                            offset: 0,
                            private: false,
                        }
                    } else {
                        assert_eq!(replacement, 1);
                        MmapSource::PrivateAnonymous
                    };
                    replacing
                        .mmap(MmapRequest {
                            address: MmapAddress::Fixed(SOURCE),
                            length: PAGE_SIZE,
                            permissions: RO,
                            maximum: RO,
                            source,
                        })
                        .map(|_| ())
                };
                *recorded.lock() = Some(operation);
                0
            })
            .map_err(|_| TestError("mandatory retirement producer prepare failed"))?
            .publish(),
        );
        until(
            "mandatory replacement did not pause with original leaf",
            || ready.load(Ordering::Acquire),
        )?;
        context.expect_eq(mm.mapping(SOURCE).unwrap().physical, old_folio.phys_addr());

        // Step 3: 对旧文件并发执行 truncate(0)，验证反向映射仍能定位并清除 SOURCE 处的旧页表叶项，且跨核 TLB 同步（CSD）期间保有物理页与页表引用
        let truncating = old.clone();
        let observed = fixture.consume_sync(1, move || truncating.truncate(0))?;
        until("old file rmap lost mandatory retirement source", || {
            controls.unmap_phase.load(Ordering::Acquire) == 1 || observed.result.lock().is_some()
        })?;
        context.expect_eq(mm.mapping(SOURCE), None);
        let before = controls.before_sync.lock().ok_or(TestError(
            "mandatory old file retirement did not retain original owners",
        ))?;
        context.expect_eq(before.0, 1);
        context.expect_eq(2 <= before.1, true);
        context.expect_eq(old_inode.mapping().is_canonical(0, &old_folio), true);

        controls.gates[2].complete();
        until("mandatory old file retirement CSD did not finish", || {
            controls.unmap_phase.load(Ordering::Acquire) == 2
        })?;
        context.expect_eq(*controls.after_sync.lock(), Some(before));
        until("mandatory old file truncate did not finish", || {
            observed.result.lock().is_some()
        })?;
        context.expect_eq(*observed.result.lock(), Some(Ok(())));
        context.expect_eq(checked(context, old.stat())?.size, 0);
        context.expect_eq(mm.mapping(SOURCE), None);

        // Step 4: 唤醒 VMA 覆盖/卸载任务完成收尾，随后触发 SOURCE 读缺页验证新映射（文件或匿名零页）或 Unmapped 状态
        gate.complete();
        mm.ktest_set_vma_unmap_hook(None);
        fixture.prepare_gate.take();
        fixture.join_consumers()?;
        context.expect_eq(*result.lock(), Some(Ok(())));
        old_inode.mapping().ktest_set_unmap_hook(None);
        fixture.truncate_controls.take();

        if replacement == 2 {
            context.expect_eq(mm.fault(SOURCE, UserAccess::Read), Err(VmError::Unmapped));
        } else {
            vm(context, mm.fault(SOURCE, UserAccess::Read))?;
            if replacement == 0 {
                context.expect_eq(
                    mm.mapping(SOURCE).unwrap().physical,
                    new_folio.as_ref().unwrap().phys_addr(),
                );
                mapped_bytes(context, &mm, SOURCE, &expected);
            } else {
                mapped_bytes(context, &mm, SOURCE, &[0; 32]);
                context.expect_eq(
                    mm.mapping(SOURCE).unwrap().physical != old_folio.phys_addr(),
                    true,
                );
            }
        }
        fixture.shared_mm.take();
        context.diagnostic(format_args!("mandatory-file-retirement replacement={} new-VMA-published-old-leaf-still-live=1 real-old-file-truncate-finds-source=1 old-leaf-cleared=1 original-page-and-table-owners-held-through-CSD=1 actual-new-fault-or-unmapped=1 native-MM=1 active-user-root-claim=0 fail-before-runtime-claim=0", replacement));
        Ok(())
    }
}
