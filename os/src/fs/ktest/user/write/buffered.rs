//! 普通文件缓冲写入、并发追加（`O_APPEND`）、内存与磁盘配额耗尽（`ENOMEM` / `ENOSPC`）、Extent 分配树分裂合并、传统多级间接块分配以及可执行映像写入排斥（`ETXTBSY`）集成测试。
//!
//! # 模块职责与架构定位
//! 本模块属于可写文件系统测试套件（`WriteOwner`）的缓冲写入子模块，负责验证 VFS 文件描述层、页缓存（Page Cache）与底层 Ext4 块分配器在缓冲写路径上的协作不变量。
//! - 上游：由 `os/src/fs/ktest/user/write.rs` 挂载，复用 `WriteOwner` 夹具管理测试用磁盘卷、Inode 生命周期与用户态协同进程。
//! - 下游：被测对象涵盖 `File::write_at` / `write_from`、`FileTable`、Ext4 块与 Inode 位图分配器、Extent B+ 树分裂/合并算法、传统直接/间接块映射以及 ELF 加载器（`exec::load`）的写入互斥引用计数。
//!
//! # 核心设计与验证场景
//! ```text
//!   用户态缓冲区 / 文件映射源页
//!               │
//!               ▼  WriteBuffer::user (跨页边界 / 同文件缺页回填)
//!   ┌───────────────────────────────────────────────────────────┐
//!   │ VFS File 层 (游标更新 / O_APPEND 串行化 / ETXTBSY 检查)   │
//!   └───────────────────────────┬───────────────────────────────┘
//!                               │
//!                               ▼
//!   ┌───────────────────────────────────────────────────────────┐
//!   │ 页缓存与 Ext4 块映射层                                    │
//!   │  - 未初始化 Extent (Unwritten) 局部覆写与头尾零填充       │
//!   │  - 内存压力 (Page / Heap OOM) 下的已提交前缀保留与块回滚  │
//!   │  - 磁盘空间耗尽 (ENOSPC) 下的元数据树回滚与部分写入提交   │
//!   │  - Extent 树深度增长 (0->1->2) 与相邻段合并回收索引块     │
//!   └───────────────────────────────────────────────────────────┘
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 追加写入（`O_APPEND`）与普通写入必须在 Inode 读写锁保护下原子采样 EOF 并推进写入偏移，保证多打开文件描述（OFD）并发追加时记录既不重叠也不丢失。
//! - 当输入源缓冲区本身映射到同一文件的只读页时，拷贝缺页路径（Fault-in）必须在未持有目标 Inode 排他写锁的阶段完成预取，避免同 Inode 读写自死锁。

use super::*;

/// 校验文件 `stat` 视图与磁盘 Inode 记录中的 `mtime` 和 `ctime` 一致，且落在给定实时钟区间 `[earlier, later]` 内。
///
/// # 参数与边界说明
/// - `context`：测试断言上下文。
/// - `file`：待检查的打开文件引用。
/// - `inode`：对应的内存 Inode 引用，用于从底层磁盘卷重新加载盘面记录。
/// - `earlier` / `later`：写入操作前后的实时钟纳秒时间戳（`u64`）。
/// - 当磁盘 Inode 的 `extra_isize` 足以容纳对应的高精度纳秒扩展字段（`ctime_extra` 在偏移 132，`mtime_extra` 在偏移 136）时，按纳秒粒度比较；对于 128 字节传统 Inode 格式，自动向下取整按整秒（`1_000_000_000` 纳秒）粒度比较。
fn modified_between(
    context: &mut TestContext<'_>,
    file: &FileRef,
    inode: &InodeRef,
    earlier: u64,
    later: u64,
) -> TestResult {
    let stat = checked(context, file.stat())?;
    let record = checked(
        context,
        checked(context, inode.disk_volume())?.load_inode(inode.number(), None),
    )?;

    for (modified, recorded, extra) in [
        (stat.mtime, record.mtime, 136),
        (stat.ctime, record.ctime, 132),
    ] {
        context.expect_eq(modified, recorded);
        let quantum = if extra + 4 <= 128 + record.extra_isize {
            1
        } else {
            1_000_000_000
        };
        let first = earlier / quantum * quantum;
        let last = later / quantum * quantum;
        let lower = (
            (first / 1_000_000_000) as i64,
            (first % 1_000_000_000) as u32,
        );
        let upper = ((last / 1_000_000_000) as i64, (last % 1_000_000_000) as u32);
        let actual = (modified.seconds, modified.nanoseconds);
        context.expect_eq(lower <= actual && actual <= upper, true);
    }
    Ok(())
}

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 测试内核与用户缓冲区写入、跨未映射页的部分拷贝（Partial Progress）、共享只读映射同步可见性及同文件映射源缺页写入（W04）。
    ///
    /// - **测试目标**：验证 `write_at` / `write_from` 的文件游标语义、跨无效用户地址时的已提交前缀保留、写入对共享只读映射页的即时可见性，以及将同一文件的映射页作为写入源时的缺页处理。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 使用 `write_at` 跨越第一个页边界写入数据，验证文件洞区零填充、`mtime`/`ctime` 更新且文件当前游标保持为 0。
    ///   2. 构造尾部紧邻未映射虚拟页的用户态缓冲区（仅前 17 字节有效），调用 `write_from` 写入；随后从完全未映射的地址再次调用 `write_from`。
    ///   3. 建立该文件的共享只读内存映射（`RO`），先后向文件已存在页与超出原大小的新偏移写入字节，通过映射虚拟地址读取验证。
    ///   4. 打开另一个文件并建立未预触发缺页的私有只读映射，将该映射区直接作为 `write_at_from` 的数据源追加写入同一文件尾部。
    /// - **预期结果**：部分有效的用户缓冲区成功写入前 17 字节并推进游标，完全无效地址返回 `BadAddress` 且不修改时间戳；共享只读映射实时反映文件覆写与页内增长；同文件映射作为输入源时正常触发缺页加载并完成写入，不发生锁重入死锁。
    #[test]
    #[ktest(tags("W04"))]
    fn kernel_user_prefix_and_same_file_source(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-kernel", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();

        // Step 1: 验证指定偏移写入（write_at）产生的前置空洞零填充、时间戳更新，且不改变文件当前读写游标。
        let first_write = time::realtime();
        let input = [0x39; 1027];
        context.expect_eq(checked(context, file.write_at(4093, &input))?, input.len());
        modified_between(context, &file, &inode, first_write, time::realtime())?;
        context.expect_eq(checked(context, file.stat())?.size, 5120);
        bytes_at(context, &file, 0, &[0; 4093])?;
        bytes_at(context, &file, 4093, &input)?;
        context.expect_eq(checked(context, file.seek(SeekFrom::Current(0)))?, 0);

        // Step 2: 构造跨越已映射页与未映射页边界的用户缓冲区，验证遇到缺页错误前已成功拷贝的 17 字节前缀正常提交。
        let address_space = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(context, address_space.map_anonymous(SOURCE, 4096, RW, RW))?;
        context.expect_eq(address_space.copyout(SOURCE + 4079, &[0x72; 17]).copied, 17);
        let mut source = checked(
            context,
            WriteBuffer::user(&address_space, SOURCE + 4079, 64),
        )?;
        let prefix_write = time::realtime();
        context.expect_eq(
            checked(context, file.write_from(&mut source, FileIoContext::Kernel))?,
            17,
        );
        modified_between(context, &file, &inode, prefix_write, time::realtime())?;
        context.expect_eq(checked(context, file.seek(SeekFrom::Current(0)))?, 17);
        context.expect_eq(checked(context, file.stat())?.size, 5120);
        bytes_at(context, &file, 0, &[0x72; 17])?;

        // Step 3: 验证起始地址即位于未映射区的写入直接返回 BadAddress，且文件游标与时间戳保持不变。
        let mut source = checked(
            context,
            WriteBuffer::user(&address_space, SOURCE + 4096, 64),
        )?;
        let before_fault = checked(context, file.stat())?;
        context.expect_eq(
            file.write_from(&mut source, FileIoContext::Kernel),
            Err(FsError::BadAddress),
        );
        context.expect_eq(checked(context, file.seek(SeekFrom::Current(0)))?, 17);
        let after_fault = checked(context, file.stat())?;
        context.expect_eq(
            (after_fault.mtime, after_fault.ctime),
            (before_fault.mtime, before_fault.ctime),
        );

        // Step 4: 建立共享只读文件映射，验证对已映射页的数据覆写与同页内 EOF 扩展对内存映射视图立即可见。
        let shared = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            shared.map_file(SOURCE, 8192, RO, RO, file.clone(), 0, false),
        )?;
        let mut visible = [0u8; 1];
        context.expect_eq(shared.copyin(&mut visible, SOURCE + 4096).copied, 1);
        context.expect_eq(visible, [0x39]);

        context.expect_eq(checked(context, file.write_at(4096, &[0x64]))?, 1);
        bytes_at(context, &file, 4096, &[0x64])?;
        context.expect_eq(shared.copyin(&mut visible, SOURCE + 4096).copied, 1);
        context.expect_eq(visible, [0x64]);

        context.expect_eq(shared.copyin(&mut visible, SOURCE + 6000).copied, 1);
        context.expect_eq(visible, [0]);
        context.expect_eq(checked(context, file.write_at(6000, &[0x42]))?, 1);
        context.expect_eq(shared.copyin(&mut visible, SOURCE + 6000).copied, 1);
        context.expect_eq(visible, [0x42]);
        drop(shared);

        // Step 5: 将同一文件的未预热映射页作为 write_at_from 的数据源写入该文件自身，验证内部缺页读取不会与写入路径发生同 Inode 死锁。
        let mapped = fixture.open(context, b"/test/write-map", 2)?;
        let mm = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            mm.map_file(SOURCE, 8192, RO, RO, mapped.clone(), 0, true),
        )?;
        // 输入虚拟页尚未建立页表项；write_at_from 内部拉取用户数据时必须支持触发同一 Inode 的文件缺页加载。
        let mut source = checked(context, WriteBuffer::user(&mm, SOURCE + 4093, 1027))?;
        context.expect_eq(
            checked(
                context,
                mapped.write_at_from(8192, &mut source, FileIoContext::Kernel),
            )?,
            1027,
        );

        let mut expected = [0u8; 1027];
        for (index, byte) in expected.iter_mut().enumerate() {
            *byte = ((4093 + index) & 255) as u8;
        }
        bytes_at(context, &mapped, 8192, &expected)?;
        context.expect_eq(checked(context, mapped.stat())?.size, 9219);

        drop(mm);
        drop(mapped);
        fixture.run(context, b"w")
    }

    /// 测试多文件描述并发追加写入（`O_APPEND`）的无损串行化，以及 `dup` 共享游标与 `pwrite` 位置语义（W05）。
    ///
    /// - **测试目标**：验证两个独立打开的 `O_APPEND` 文件描述在多核并发写入时不会相互覆盖，并验证复制描述符（`dup`）下的 `write_at`（`pwrite`）与普通 `write` 游标协议。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 对同一路径分别以 `O_RDWR | O_APPEND` 打开两个独立的文件对象 `first` 和 `second`。
    ///   2. 启动两个工作线程在同步门（`Completion`）处对齐阻塞，同时放行后各自循环追加 32 条 512 字节的特征记录。
    ///   3. 扫描最终文件的全部 64 条 512 字节记录块，检查记录头标识与序号位图。
    ///   4. 将 `first` 注册到文件描述符表并执行 `dup`，先后对副本描述符调用 `write_at(dup, 0, ...)` 和普通 `write(dup, ...)`。
    /// - **预期结果**：两线程共写入 32768 字节，64 条记录各自完整无交错；对 `O_APPEND` 描述符执行 `write_at` 仍追加到文件末尾但不改变原共享文件游标，执行普通 `write` 则追加数据并同步更新共享游标。
    #[test]
    #[ktest(tags("W05"), min_cpus = 2)]
    fn append_race_dup_and_pwrite(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        // Step 1: 创建两个独立的 O_APPEND 打开文件描述，并通过同步门让两个后台写入任务在同一时刻开始竞争追加。
        let first = fixture.open(context, b"/test/write-append-kernel", 2 | 0x400)?;
        let second = fixture.open(context, b"/test/write-append-kernel", 2 | 0x400)?;
        context.expect_eq(Arc::ptr_eq(&first, &second), false);
        let gate = Arc::try_new(Completion::new()).map_err(|_| TestError("append gate OOM"))?;
        fixture.append_gate = Some(gate.clone());

        fixture.user.path.owner.publish(
            0,
            Job::Append {
                file: first.clone(),
                gate: gate.clone(),
                marker: 0x51,
                records: 32,
            },
        );
        fixture.user.path.owner.publish(
            1,
            Job::Append {
                file: second.clone(),
                gate: gate.clone(),
                marker: 0x92,
                records: 32,
            },
        );
        until("append writers did not block at causal gate", || {
            fixture.user.path.owner.observer().blocked(0)
                && fixture.user.path.owner.observer().blocked(1)
        })?;
        gate.complete();
        gate.complete();
        fixture.user.path.owner.join_tasks()?;

        // Step 2: 汇总两个追加任务的写入字节数，并逐块校验全部 64 条记录的完整性与不重不漏位图。
        for index in 0..2 {
            let result = fixture.user.path.owner.observer().results.lock()[index]
                .take()
                .ok_or(TestError("append result missing"))?;
            let Returned::Written(count) = checked(context, result)? else {
                return Err(TestError("append result type"));
            };
            context.expect_eq(count, 16384);
        }
        context.expect_eq(checked(context, first.stat())?.size, 32768);

        let mut seen = [0u32; 2];
        let mut bytes = [0u8; 512];
        for record in 0..64 {
            context.expect_eq(
                checked(context, first.read_at(record * 512, &mut bytes))?,
                512,
            );
            let writer = match bytes[0] {
                0x51 => 0,
                0x92 => 1,
                _ => return Err(TestError("append invalid record")),
            };
            let index = bytes[1] as usize;
            if 32 <= index
                || seen[writer] & (1 << index) != 0
                || !bytes[2..].iter().all(|byte| *byte == bytes[0])
            {
                return Err(TestError("append overlap or partial record"));
            }
            seen[writer] |= 1 << index;
        }
        context.expect_eq(seen, [u32::MAX; 2]);

        // Step 3: 验证 dup 产生的描述符与原描述符共享底层打开文件对象，且 O_APPEND 下 write_at 追加至末尾但不移动共享游标。
        let table = checked(context, FileTable::try_new())?;
        let fd = checked(context, table.reserve(1024))?
            .publish(first.clone(), DescriptorFlags::default());
        let dup = checked(context, table.dup(fd, 1024))?;
        context.expect_eq(
            Arc::ptr_eq(&checked(context, table.lookup(dup))?, &first),
            true,
        );

        let cursor = checked(context, first.seek(SeekFrom::Current(0)))?;
        context.expect_eq(checked(context, table.write_at(dup, 0, b"positioned"))?, 10);
        context.expect_eq(checked(context, first.seek(SeekFrom::Current(0)))?, cursor);
        bytes_at(context, &second, 32768, b"positioned")?;
        context.expect_eq(checked(context, table.write(dup, b"shared"))?, 6);
        context.expect_eq(checked(context, first.seek(SeekFrom::Current(0)))?, 32784);
        fixture.run(context, b"a")
    }

    /// 测试物理页与内核堆内存耗尽（OOM）时的错误恢复、块回滚及未初始化 Extent（Unwritten）局部写入零填充（W06）。
    ///
    /// - **测试目标**：验证新块/Inode 分配的未发布回滚机制、页分配器与各阶堆分配器耗尽时写入操作返回 `NoMemory` 且不泄漏磁盘块，以及在预分配未初始化块中局部写入时的头尾清零行为。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 预先向测试文件写入第一个 4KB 页，随后演练 `allocate_block` / `allocate_inode` 分配后再调用 `release_unpublished_*` 释放，核对空闲块与空闲 Inode 计数。
    ///   2. 使用 `PagePressure` 耗尽空闲物理页，尝试向文件第二个页（偏移 4096）写入数据，检查返回值与磁盘空闲块数。
    ///   3. 在后台任务中开启堆内存压力，先覆写已缓存的第一页（偏移 0，暖路径），再尝试扩展第二页（偏移 4096，冷路径）。
    ///   4. 打开含未初始化 Extent（Unwritten）的文件，向块内偏移 513 处写入 17 字节，读取整块及相邻块验证。
    /// - **预期结果**：未发布的块和 Inode 释放后计数完全复原；物理页耗尽时冷写入返回 `NoMemory` 且不泄漏磁盘块；堆内存耗尽时已驻留页覆写成功（17 字节），需要分配新元数据的扩展写入返回 `NoMemory` 且不泄漏磁盘块；未初始化块局部写入后，块内其余字节均为 0。
    #[test]
    #[ktest(tags("W06"))]
    fn real_heap_page_oom_and_unwritten_zero(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-oom", 2)?;
        context.expect_eq(checked(context, file.write(&[0x41; 4096]))?, 4096);

        // Step 1: 验证未发布的磁盘数据块与磁盘 Inode 在中止释放后能将分配位图计数精确还原。
        let volume = fixture.volume();
        let before = allocation_counts(context, &volume)?;
        let unpublished = checked(context, volume.allocate_block(0))?;
        let allocated = allocation_counts(context, &volume);
        volume.release_unpublished_block(unpublished);
        let allocated = allocated?;
        context.expect_eq(allocated, (before.0 - 1, before.1));
        let restored = allocation_counts(context, &volume)?;
        context.expect_eq(restored, before);

        let unpublished = checked(context, volume.allocate_inode(0, 0o100600, 123, 234))?;
        let record = volume.load_inode(unpublished.number(), None);
        let allocated = allocation_counts(context, &volume);
        volume.release_unpublished_inode(unpublished);
        let record = checked(context, record)?;
        context.expect_eq(
            (
                record.mode,
                record.uid,
                record.gid,
                record.size,
                record.blocks,
            ),
            (0o100600, 123, 234, 0, 0),
        );
        context.expect_eq(allocated?, (before.0, before.1 - 1));
        let restored = allocation_counts(context, &volume)?;
        context.expect_eq(restored, before);

        // Step 2: 耗尽物理页分配器，验证触发新页分配的文件写入返回 NoMemory 且不会残留已分配的磁盘块。
        let mut page_pressure = PagePressure::prepare()?;
        let (evidence, page_error) = {
            let evidence = {
                let _pin = PreemptGuard::new();
                page_pressure.exhaust()
            };
            let error = file.write_at(4096, &[0x9b; 17]);
            drop(page_pressure);
            (evidence, error)
        };
        context.expect_eq(evidence, true);
        context.expect_eq(page_error, Err(FsError::NoMemory));
        let after_page_failure = allocation_counts(context, &volume)?;
        context.expect_eq(after_page_failure, before);

        // Step 3: 在内核堆分配器耗尽的条件下，验证已驻留缓存页的原地覆写成功提交，而需要分配新结构的冷块写入返回 NoMemory 且不泄漏磁盘块。
        let pressure = fixture
            .user
            .path
            .owner
            .pressure
            .take()
            .ok_or(TestError("write heap pressure missing"))?;
        fixture.user.path.owner.publish(
            0,
            Job::WriteOom {
                file: file.clone(),
                pressure,
            },
        );
        fixture.user.path.owner.join_tasks()?;

        let result = fixture.user.path.owner.observer().results.lock()[0]
            .take()
            .ok_or(TestError("write OOM result missing"))?;
        let Returned::WriteOom {
            evidence,
            warm,
            cold,
        } = checked(context, result)?
        else {
            return Err(TestError("write OOM result type"));
        };
        context.expect_eq(evidence, (true, true));
        context.expect_eq(warm, Ok(17));
        context.expect_eq(cold, Err(FsError::NoMemory));
        let after_heap_failure = allocation_counts(context, &volume)?;
        context.expect_eq(after_heap_failure, before);
        context.expect_eq(checked(context, file.stat())?.size, 4096);
        bytes_at(context, &file, 0, &[0x6d; 17])?;

        // Step 4: 解除内存压力后验证此前失败的偏移可正常写入，并验证对未初始化 Extent（Unwritten）局部写入时未覆盖区域自动清零。
        context.expect_eq(checked(context, file.write_at(4096, &[0x9b; 17]))?, 17);

        let unwritten = fixture.open(context, b"/test/write-unwritten", 2)?;
        context.expect_eq(checked(context, unwritten.write_at(513, &[0x63; 17]))?, 17);
        let mut expected = [0u8; 4096];
        expected[513..530].fill(0x63);
        bytes_at(context, &unwritten, 0, &expected)?;
        bytes_at(context, &unwritten, 4096, &[0; 4096])?;
        context.expect_eq(checked(context, unwritten.stat())?.size, 16384);
        Ok(())
    }

    /// 测试磁盘空间接近耗尽（`ENOSPC`）时的 Extent 分裂原子回滚与跨多块写入的部分成功前缀保留（W06）。
    ///
    /// - **测试目标**：验证当单次块写入需要分裂 Extent 树节点但剩余空闲块不足以同时分配数据块与索引块时，文件元数据与空闲块计数能完整回滚；以及多块连续写入在耗尽最后一个空闲块时能正确提交已写入前缀（Partial Progress）。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 向 `/test/write-rollback` 间隔写入 4 个非连续逻辑块（0, 2, 4, 6），填满 Inode 根部的 4 个 Extent 槽位。
    ///   2. 预留除最后 1 个空闲块以外的所有磁盘块，随后向逻辑块 8 写入 1 字节（需要 1 个数据块 + 1 个外部 Extent 节点块）。
    ///   3. 释放预留块，打开新文件 `/test/write-full`，发起一次长度超过当前全部剩余空闲块数的连续写入请求，随后再追加写入 512 字节。
    /// - **预期结果**：步骤 2 因缺少外部索引块返回 `NoSpace`，先申请到的唯一数据块被归还，Inode 元数据与根部 Extent 数组毫发无损；步骤 3 首次大数据量写入返回大于 0 的已成功写入字节数并推进游标，磁盘空闲块归零，紧接着的写入返回 `NoSpace` 且游标不再移动，已消耗块数严格等于文件的有效数据块与 Extent 索引块之和。
    #[test]
    #[ktest(tags("W06", "fs-write-enospc"))]
    fn real_enospc_keeps_accepted_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let volume = fixture.volume();
        let initial = allocation_counts(context, &volume)?;
        if initial.0 < 8 || 256 < initial.0 {
            return Err(TestError(
                "ENOSPC fixture must have 8..256 genuinely free blocks",
            ));
        }

        // Step 1: 写入 4 个不相邻的逻辑块，占满 Inode 内联根节点的 4 个 Extent 条目槽位，随后将卷空闲块消耗至仅剩 1 块。
        let rollback = fixture.open(context, b"/test/write-rollback", 2)?;
        let rollback_inode = fixture.inodes.last().unwrap().clone();
        let block_size = volume.geometry().block_size as u64;
        for logical in [0, 2, 4, 6] {
            context.expect_eq(
                checked(context, rollback.write_at(logical * block_size, &[0x5d]))?,
                1,
            );
        }
        let original_tree = tree(context, &rollback_inode)?;
        context.expect_eq(original_tree.extents, 4);
        let metadata = checked(context, rollback_inode.metadata_snapshot())?;
        let root = checked(context, rollback_inode.with_map_read(|_, root| Ok(*root)))?;
        let available = allocation_counts(context, &volume)?;

        let mut held = Vec::new();
        held.try_reserve_exact(available.0 as usize)
            .map_err(|_| TestError("ENOSPC reservation slots OOM"))?;
        for _ in 1..available.0 {
            match volume.allocate_block(0) {
                Ok(block) => held.push(block),
                Err(error) => {
                    for block in held.drain(..) {
                        volume.release_unpublished_block(block);
                    }
                    context.diagnostic(format_args!(
                        "ENOSPC reservation preparation error={:?}",
                        error
                    ));
                    return Err(TestError("ENOSPC reservation preparation failed"));
                }
            }
        }

        // Step 2: 写入第 5 个稀疏逻辑块时会先申请仅存的 1 个空闲块作为数据块，随后在分裂 Extent 根节点申请外部索引块时触发 ENOSPC，必须将刚申请的数据块完整归还。
        let single_before = allocation_counts(context, &volume);
        let failed = rollback.write_at(8 * block_size, &[0x28]);
        let single_after = allocation_counts(context, &volume);
        let metadata_after = rollback_inode.metadata_snapshot();
        let root_after = rollback_inode.with_map_read(|_, root| Ok(*root));
        for block in held.drain(..) {
            volume.release_unpublished_block(block);
        }

        context.expect_eq(single_before?, (1, available.1));
        context.expect_eq(single_after?, (1, available.1));
        context.expect_eq(failed, Err(FsError::NoSpace));
        context.expect_eq(checked(context, metadata_after)?, metadata);
        context.expect_eq(checked(context, root_after)?, root);
        for logical in [0, 2, 4, 6] {
            bytes_at(context, &rollback, logical * block_size, &[0x5d])?;
        }
        context.expect_eq(checked(context, rollback.seek(SeekFrom::Current(0)))?, 0);
        let mut untouched = [0xa7; 17];
        context.expect_eq(
            checked(context, rollback.read_at(8 * block_size, &mut untouched))?,
            0,
        );
        context.expect_eq(untouched, [0xa7; 17]);
        let restored = allocation_counts(context, &volume)?;
        context.expect_eq(restored, available);

        // Step 3: 释放预留块后发起超出剩余总块数的大缓冲区写入，验证耗尽磁盘前已成功分配的块正常提交并返回已写入字节数，后续写入返回 NoSpace。
        let file = fixture.open(context, b"/test/write-full", 2)?;
        let full_inode = fixture.inodes.last().unwrap().clone();
        let before = allocation_counts(context, &volume)?;
        let mut request = Vec::new();
        let length = (before.0 as usize + 1) * volume.geometry().block_size;
        request
            .try_reserve_exact(length)
            .map_err(|_| TestError("ENOSPC input OOM"))?;
        request.resize(length, 0x87);

        let count = checked(context, file.write(&request))?;
        context.expect_eq(0 < count && count < request.len(), true);
        context.expect_eq(
            checked(context, file.seek(SeekFrom::Current(0)))?,
            count as u64,
        );
        context.expect_eq(checked(context, file.stat())?.size, count as u64);
        context.expect_eq(file.write(&[0x18; 512]), Err(FsError::NoSpace));
        context.expect_eq(
            checked(context, file.seek(SeekFrom::Current(0)))?,
            count as u64,
        );
        for offset in (0..count).step_by(4096) {
            bytes_at(
                context,
                &file,
                offset as u64,
                &request[offset..core::cmp::min(count, offset + 4096)],
            )?;
        }

        let after = allocation_counts(context, &volume)?;
        context.expect_eq(after.0, 0);
        let tree = tree(context, &full_inode)?;
        context.expect_eq(before.0 - after.0, tree.data + tree.nodes);
        context.expect_eq(before.1, after.1);
        Ok(())
    }

    /// 测试稀疏写入驱动的 Extent B+ 树深度增长（0->1->2）、空洞填补合并、外部索引块回收以及回收块重用为新文件数据块（W07）。
    ///
    /// - **测试目标**：验证大量不连续逻辑块插入能使 Extent 树从深度 0 分裂至深度 2，验证填补中间空洞或覆盖未初始化段能将相邻 Extent 合并并把退化的外部索引节点归还给卷分配器，且回收的物理块可被后续文件安全重用为数据块。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 向 `/test/write-tree` 依次写入偶数逻辑块（间隔留洞），使 Extent 条目数突破深度 0（4 项）和深度 1 的容量上限，直至树深度增长为 2。
    ///   2. 打开预置了连续未初始化 Extent（64 块）的 `/test/write-merge`，先写入中间块触发未初始化段一分为三，再写入两侧相邻块验证合并。
    ///   3. 继续向 `/test/write-merge` 间隔写入奇数块，迫使 Extent 树分裂出深度为 1 的外部索引节点；随后补齐所有偶数块使全文件变为单一连续已初始化段。
    ///   4. 收集步骤 3 中树折叠退化回深度 0 时释放的物理索引块号，向 `/test/write-rollback` 写入新数据并刷盘，直接从块设备读取这些物理块验证。
    /// - **预期结果**：稀疏写入后树深度达到 2 且块消耗量等于数据块加索引节点块；补齐所有间隔块后，64 个块合并为深度 0 的单个 Extent，原外部索引节点块被悉数释放；新文件写入直接重用这些退役的物理索引块存储用户数据，落盘内容准确无误。
    #[test]
    #[ktest(tags("W07", "fs-write-extents"))]
    fn extent_grow_split_merge_and_allocation(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        // Step 1: 持续按偶数逻辑块偏移稀疏写入，迫使 Extent 树依次从深度 0 分裂为深度 1，最终达到深度 2。
        let file = fixture.open(context, b"/test/write-tree", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.volume();
        let block_size = volume.geometry().block_size;
        let before = allocation_counts(context, &volume)?;
        let entries = ((block_size - 16) / 12) * 4 + 1;
        let data = [0x4e; 4096];

        for index in 0..entries {
            context.expect_eq(
                checked(
                    context,
                    file.write_at((index * 2 * block_size) as u64, &data[..block_size]),
                )?,
                block_size,
            );
            if index == 4 {
                let grown = tree(context, &inode)?;
                context.expect_eq(grown.depth, 1);
            }
        }

        let grown = tree(context, &inode)?;
        context.expect_eq(grown.depth, 2);
        context.expect_eq(grown.extents, entries);
        context.expect_eq(grown.data, entries as u64);
        let after = allocation_counts(context, &volume)?;
        context.expect_eq(before.0 - after.0, grown.data + grown.nodes);
        context.expect_eq(before.1, after.1);
        for index in 0..entries {
            bytes_at(
                context,
                &file,
                (index * 2 * block_size) as u64,
                &data[..block_size],
            )?;
            if index + 1 < entries {
                bytes_at(
                    context,
                    &file,
                    ((index * 2 + 1) * block_size) as u64,
                    &[0; 4096][..block_size],
                )?;
            }
            context.expect_eq(
                matches!(
                    checked(context, inode.map_block((index * 2) as u64))?,
                    BlockMapping::Mapped { .. }
                ),
                true,
            );
        }

        // Step 2: 在预分配的未初始化 Extent 中间写入单块触发段分裂，再补齐相邻块验证段合并。
        let merge = fixture.open(context, b"/test/write-merge", 2)?;
        let merge_inode = fixture.inodes.last().unwrap().clone();
        let original = tree(context, &merge_inode)?;
        context.expect_eq(
            checked(
                context,
                merge.write_at(block_size as u64, &data[..block_size]),
            )?,
            block_size,
        );
        let split = tree(context, &merge_inode)?;
        context.expect_eq(split.extents, original.extents + 2);

        context.expect_eq(
            checked(context, merge.write_at(0, &data[..block_size]))?,
            block_size,
        );
        context.expect_eq(
            checked(
                context,
                merge.write_at((2 * block_size) as u64, &data[..block_size]),
            )?,
            block_size,
        );
        let merged = tree(context, &merge_inode)?;
        context.expect_eq(merged.extents, original.extents + 1);
        context.expect_eq((merged.data, merged.nodes), (original.data, original.nodes));
        let merge_before = allocation_counts(context, &volume)?;

        // Step 3: 间隔写入大量奇数块使树分裂出外部索引节点，再补齐全部偶数块使全树合并折叠回深度 0（单条 Extent），释放外部索引节点块。
        context.expect_eq(original.data, 64);
        for logical in (3..8).step_by(2) {
            context.expect_eq(
                checked(
                    context,
                    merge.write_at((logical * block_size) as u64, &data[..block_size]),
                )?,
                block_size,
            );
        }
        let first_split = tree(context, &merge_inode)?;
        context.expect_eq((first_split.depth, first_split.nodes), (1, 2));

        for logical in (9..64).step_by(2) {
            context.expect_eq(
                checked(
                    context,
                    merge.write_at((logical * block_size) as u64, &data[..block_size]),
                )?,
                block_size,
            );
        }
        let expanded = tree(context, &merge_inode)?;
        context.expect_eq(expanded.depth, 1);
        context.expect_eq(0 < expanded.nodes, true);
        context.expect_eq(expanded.data, original.data);
        let split_counts = allocation_counts(context, &volume)?;
        context.expect_eq(merge_before.0 - split_counts.0, expanded.nodes);

        for logical in (4..64).step_by(2) {
            context.expect_eq(
                checked(
                    context,
                    merge.write_at((logical * block_size) as u64, &data[..block_size]),
                )?,
                block_size,
            );
        }
        let collapsed = tree(context, &merge_inode)?;
        context.expect_eq(
            (collapsed.depth, collapsed.nodes, collapsed.extents),
            (0, 0, 1),
        );
        context.expect_eq(collapsed.data, original.data);
        let collapsed_counts = allocation_counts(context, &volume)?;
        context.expect_eq(collapsed_counts, merge_before);

        // Step 4: 验证折叠释放的原 Extent 索引节点物理块被块分配器回收并重用为新文件的数据块，同步落盘后校验底层扇区字节。
        let reused = fixture.open(context, b"/test/write-rollback", 2)?;
        let reused_inode = fixture.inodes.last().unwrap().clone();
        let replacement = [0xb9; 4096];
        for (logical, physical) in first_split
            .blocks
            .iter()
            .filter(|physical| !collapsed.blocks.contains(physical))
            .enumerate()
        {
            context.expect_eq(
                checked(
                    context,
                    reused.write_at((logical * block_size) as u64, &replacement[..block_size]),
                )?,
                block_size,
            );
            let BlockMapping::Mapped {
                physical: actual, ..
            } = checked(context, reused_inode.map_block(logical as u64))?
            else {
                return Err(TestError("retired extent node was not reused as file data"));
            };
            context.expect_eq(actual, *physical);
            context.diagnostic(format_args!(
                "write-retired-node-reuse physical={} logical={}",
                physical, logical
            ));
        }

        for inode in &fixture.inodes {
            checked(context, crate::fs::writeback::sync_inode(inode))?;
        }
        checked(context, crate::fs::writeback::sync_metadata(&volume))?;

        let mut device_bytes = [0u8; 4096];
        for physical in first_split
            .blocks
            .iter()
            .filter(|physical| !collapsed.blocks.contains(physical))
        {
            checked(
                context,
                volume
                    .claim()
                    .read_exact_at(
                        *physical * block_size as u64,
                        &mut device_bytes[..block_size],
                    )
                    .map_err(FsError::Io),
            )?;
            context.expect_eq(&device_bytes[..block_size], &replacement[..block_size]);
        }
        for logical in 0..64 {
            bytes_at(
                context,
                &merge,
                (logical * block_size) as u64,
                &data[..block_size],
            )?;
        }
        Ok(())
    }

    /// 测试传统 Ext2/Ext3 间接块映射文件（无 `EXT4_EXTENTS_FL`）的直接块、一级、二级与三级间接块按需分配及 128 字节 Inode 整秒时间戳（W07）。
    ///
    /// - **测试目标**：验证不启用 Extent 特性的传统格式 Inode 在跨级稀疏写入时能正确构建直接块（Direct）、一级间接（Single Indirect）、二级间接（Double Indirect）和三级间接（Triple Indirect）索引链，并准确统计数据块与间接索引块消耗。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 确认测试卷块大小为 1024 字节，且目标 Inode 未设置 `EXT4_EXTENTS_FL`（`0x80000`）标志。
    ///   2. 依次向逻辑块 0（直接块）、12（一级间接块起点）、268（二级间接块起点）和 65804（三级间接块起点）的块内偏移 513 写入 17 字节，并在块尾偏移 1023 写入 1 字节 `0` 以将文件大小推进到块边界。
    ///   3. 检查 `mtime`/`ctime` 时间戳、映射树统计的块数与卷空闲块差值。
    /// - **预期结果**：4 个目标数据块与 6 个间接索引块（1 个一级 + 2 个二级 + 3 个三级）被精确分配，总计消耗 10 个磁盘块；块内未显式写入的字节均为 0；128 字节 Inode 按整秒精度正确记录修改时间。
    #[test]
    #[ktest(tags("W07", "fs-write-traditional"))]
    fn traditional_direct_single_double_triple(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-tree", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let volume = fixture.volume();
        context.expect_eq(volume.geometry().block_size, 1024);
        context.expect_eq(
            checked(context, inode.metadata_snapshot())?.flags & 0x80000,
            0,
        );
        let before = allocation_counts(context, &volume)?;

        // 依次向直接块（0）、一级间接块（12）、二级间接块（268）和三级间接块（65804）写入测试数据。
        let first_write = time::realtime();
        for (index, logical) in [0u64, 12, 268, 65804].iter().copied().enumerate() {
            let bytes = [0x31 + index as u8; 17];
            context.expect_eq(
                checked(context, file.write_at(logical * 1024 + 513, &bytes))?,
                17,
            );
            let mut expected = [0u8; 1024];
            expected[513..530].copy_from_slice(&bytes);
            // 文件末尾块的读取受限于文件字节大小（EOF），因此先在块末偏移 1023 处写入 1 字节零以扩充 EOF，再校验整块的零初始化结果。
            context.expect_eq(
                checked(context, file.write_at(logical * 1024 + 1023, &[0]))?,
                1,
            );
            bytes_at(context, &file, logical * 1024, &expected)?;
        }

        modified_between(context, &file, &inode, first_write, time::realtime())?;
        let mapped = tree(context, &inode)?;
        context.expect_eq((mapped.data, mapped.nodes), (4, 6));
        let after = allocation_counts(context, &volume)?;
        context.expect_eq(before.0 - after.0, 10);
        context.expect_eq(before.1, after.1);
        context.expect_eq(checked(context, file.stat())?.size, (65804 + 1) * 1024);
        Ok(())
    }

    /// 测试可执行文件的写入排斥计数（`ETXTBSY`）在 ELF 加载、`fork`、任务准备、私有文件映射以及 OOM / 格式错误回滚路径上的生命周期（W08）。
    ///
    /// - **测试目标**：验证当文件已被可写打开时拒绝作为可执行映像加载（返回 `TextBusy`），且当文件已加载为可执行映像后，在映像、`fork` 子地址空间及未发布的准备态任务全部释放前，拒绝任何可写打开或截断打开请求；同时验证私有只读映射不阻塞写入，且加载或 `fork` 中途失败能正确释放写排斥引用。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 先以可写标志（`O_WRONLY`）打开可执行文件 `PROGRAM_WRITE`，尝试调用 `exec::load`，随后关闭写句柄再次加载。
    ///   2. 在持有已加载 `image` 期间尝试以 `O_WRONLY` 和 `O_WRONLY | O_TRUNC` 打开该文件；接着对 `image.mm` 执行 `fork`，依次释放 `image`、父 `mm` 和子 `mm`，每步检查可写打开结果。
    ///   3. 建立该可执行文件的普通私有只读映射（`map_file`），测试是否仍可可写打开；尝试加载非 ELF 格式文件，测试失败后是否可可写打开。
    ///   4. 调用 `task::prepare_program` 构造未发布的准备态任务，检查期间与丢弃后的可写打开行为；最后分别在堆内存耗尽与物理页耗尽下触发 `fork`、`prepare_program` 和 `exec::load` 失败，检查错误释放后文件是否恢复可写打开。
    /// - **预期结果**：存在可写文件描述时 `exec::load` 返回 `TextBusy`；存在任何存活的执行映像引用（含 `fork` 派生 `Mm` 与准备态任务）时可写打开返回 `TextBusy`，最后一个引用释放后立即恢复可写；普通私有文件映射不持有写排斥；格式错误或 OOM 导致的加载/准备失败均不会泄漏写排斥计数。
    #[test]
    #[ktest(tags("W08"))]
    fn executable_deny_prepare_fork_and_private_map(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let vfs = fixture.user.path.vfs().clone();
        let fs_context = fixture.user.path.context().clone();
        let program = fixture.open(context, PROGRAM_WRITE, 0)?;
        let path = checked(
            context,
            fixture.user.path.vfs().resolve(
                fixture.user.path.context(),
                PROGRAM_WRITE,
                ResolveMode::Follow,
            ),
        )?;
        // Step 1: 当文件存在可写打开句柄时，拒绝将其作为可执行程序加载（返回 TextBusy）；关闭写句柄后加载成功。
        let writer = checked(
            context,
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            ),
        )?;
        let args = Arguments::from_kernel(
            PROGRAM_WRITE,
            &[PROGRAM_WRITE, b"e"],
            &[],
            exec::DEFAULT_STACK_BYTES,
        )
        .map_err(|_| TestError("deny arguments OOM"))?;

        context.expect_eq(
            exec::load(&vfs, &fs_context, program.clone(), &args, Credentials::ROOT).err(),
            Some(ExecError::File(FsError::TextBusy)),
        );
        drop(writer);

        // Step 2: 可执行映像加载成功后持有写排斥引用，fork 派生的子地址空间同样继承该引用，直至父子 Mm 全部销毁后才允许可写打开。
        let image = exec::load(&vfs, &fs_context, program.clone(), &args, Credentials::ROOT)
            .map_err(|_| TestError("deny real ELF load failed"))?;
        context.expect_eq(
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            )
            .err(),
            Some(FsError::TextBusy),
        );
        context.expect_eq(
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1 | 0x200,
                FileIoContext::Kernel,
            )
            .err(),
            Some(FsError::TextBusy),
        );

        let child = vm(context, image.mm.fork())?;
        let old_mm = image.mm.clone();
        drop(image);
        context.expect_eq(
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            )
            .err(),
            Some(FsError::TextBusy),
        );
        drop(old_mm);

        context.expect_eq(
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            )
            .err(),
            Some(FsError::TextBusy),
        );
        drop(child);

        drop(checked(
            context,
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            ),
        )?);

        // Step 3: 普通私有文件映射（mmap）不会增加可执行写排斥计数，且加载非法 ELF 格式失败后会立即归还已获取的写排斥引用。
        let mm = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            mm.map_file(SOURCE, 4096, RO, RO, program.clone(), 0, true),
        )?;
        drop(checked(
            context,
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            ),
        )?);

        let bad = fixture.open(context, b"/test/write-map", 0)?;
        context.expect_eq(
            exec::load(&vfs, &fs_context, bad.clone(), &args, Credentials::ROOT).err(),
            Some(ExecError::Format),
        );
        drop(fixture.open(context, b"/test/write-map", 1)?);

        // Step 4: 验证尚未发布的准备态用户任务（PreparedTask）以及在堆/页 OOM 下中途失败的 fork、prepare_program 与 exec::load 均能正确维护和归还写排斥计数。
        let image = exec::load(&vfs, &fs_context, program.clone(), &args, Credentials::ROOT)
            .map_err(|_| TestError("prepare deny real load failed"))?;
        let runtime = checked(
            context,
            UserRuntime::try_new(
                fixture.user.path.vfs().clone(),
                fixture.user.path.context().clone(),
                Credentials::ROOT,
                fixture.user.net.clone(),
            ),
        )?;
        let prepared = task::prepare_program(
            fixture.user.graph(),
            None,
            image,
            runtime,
            Placement::Cpu(cpu::current().id),
        )
        .map_err(|_| TestError("unpublished executable prepare failed"))?;
        context.expect_eq(
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            )
            .err(),
            Some(FsError::TextBusy),
        );
        drop(prepared);

        drop(checked(
            context,
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            ),
        )?);

        let image = exec::load(&vfs, &fs_context, program.clone(), &args, Credentials::ROOT)
            .map_err(|_| TestError("OOM deny real load failed"))?;
        let runtime = checked(
            context,
            UserRuntime::try_new(
                fixture.user.path.vfs().clone(),
                fixture.user.path.context().clone(),
                Credentials::ROOT,
                fixture.user.net.clone(),
            ),
        )?;
        let graph = fixture.user.graph().clone();
        let pressure = fixture
            .user
            .path
            .owner
            .pressure
            .as_mut()
            .ok_or(TestError("exec pressure missing"))?;
        let (evidence, fork_error, prepare_error) = {
            let evidence = {
                let _pin = PreemptGuard::new();
                pressure.exhaust()
            };
            let fork_error = image.mm.fork().err();
            let prepare_error = task::prepare_program(
                &graph,
                None,
                image,
                runtime,
                Placement::Cpu(cpu::current().id),
            )
            .err();
            fixture.user.release_pressure();
            (evidence, fork_error, prepare_error)
        };

        context.expect_eq(evidence, (true, true));
        context.expect_eq(fork_error, Some(VmError::NoMemory));
        context.expect_eq(
            prepare_error,
            Some(crate::task::ProgramError::Identity(
                crate::task::process::ProcessError::NoMemory,
            )),
        );
        drop(checked(
            context,
            open_path_with_flags(
                fixture.user.path.vfs(),
                path.clone(),
                1,
                FileIoContext::Kernel,
            ),
        )?);

        // ELF 加载器在获取写排斥引用后若因物理页耗尽而中止，必须在错误退出路径上释放写排斥引用。
        let mut pages = PagePressure::prepare()?;
        let (evidence, load_error) = {
            let evidence = {
                let _pin = PreemptGuard::new();
                pages.exhaust()
            };
            let error =
                exec::load(&vfs, &fs_context, program.clone(), &args, Credentials::ROOT).err();
            drop(pages);
            (evidence, error)
        };
        context.expect_eq(evidence, true);
        context.expect_eq(load_error, Some(ExecError::NoMemory));
        drop(checked(
            context,
            open_path_with_flags(fixture.user.path.vfs(), path, 1, FileIoContext::Kernel),
        )?);
        fixture.run(context, b"e")
    }
}
