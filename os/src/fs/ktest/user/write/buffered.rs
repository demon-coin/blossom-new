//! 普通写入、追加、真实资源不足、分配树与可执行映像写入排斥。

use super::*;

/// stat 和当前记录须交付本次已接受写入的真实时间，未覆盖扩展字段的格式仅比较整秒。
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

    /// 源故障只接受真实前缀；写提交更新时间，共享只读尾页继续观察覆盖与同页增长。
    #[test]
    #[ktest(tags("W04"))]
    fn kernel_user_prefix_and_same_file_source(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-kernel", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let first_write = time::realtime();
        let input = [0x39; 1027];
        context.expect_eq(checked(context, file.write_at(4093, &input))?, input.len());
        modified_between(context, &file, &inode, first_write, time::realtime())?;
        context.expect_eq(checked(context, file.stat())?.size, 5120);
        bytes_at(context, &file, 0, &[0; 4093])?;
        bytes_at(context, &file, 4093, &input)?;
        context.expect_eq(checked(context, file.seek(SeekFrom::Current(0)))?, 0);
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
        let mapped = fixture.open(context, b"/test/write-map", 2)?;
        let mm = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            mm.map_file(SOURCE, 8192, RO, RO, mapped.clone(), 0, true),
        )?;
        // 输入页未预触；write_from 内部 pull 必须允许同 inode 的真实 filefault。
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

    /// 两独立打开在同一因果 gate 后竞争 EOF，记录不能覆盖；dup 与 APPEND pwrite 保各自位置协议。
    #[test]
    #[ktest(tags("W05"), min_cpus = 2)]
    fn append_race_dup_and_pwrite(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
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

    /// 真实页和全部堆类别不足保 NoMemory，暖成功前缀保留，冷失败不得泄漏盘块。
    #[test]
    #[ktest(tags("W06"))]
    fn real_heap_page_oom_and_unwritten_zero(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-oom", 2)?;
        context.expect_eq(checked(context, file.write(&[0x41; 4096]))?, 4096);
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

    /// 正常近满盘由真实分配耗尽，跨剩余块请求返回正前缀，下一次保 ENOSPC 和当前位置。
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
        // 四项根的第五个稀疏数据块会先申请唯一余块，再因外部节点无空间而归还。
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

    /// 稀疏插入使实际树增长和分裂；合并归还外部节点后，新文件重用原块仍保完整盘面字节。
    #[test]
    #[ktest(tags("W07", "fs-write-extents"))]
    fn extent_grow_split_merge_and_allocation(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
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

    /// 传统缺链由普通写建立，数据与节点计数准确，128B inode 的修改时间按整秒编码。
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
        let first_write = time::realtime();
        for (index, logical) in [0u64, 12, 268, 65804].iter().copied().enumerate() {
            let bytes = [0x31 + index as u8; 17];
            context.expect_eq(
                checked(context, file.write_at(logical * 1024 + 513, &bytes))?,
                17,
            );
            let mut expected = [0u8; 1024];
            expected[513..530].copy_from_slice(&bytes);
            // 末块的 EOF 是字节数，先写块尾再核完整零初始化。
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

    /// 真实 loader、fork 和未发表 Task 控制 deny 寿命，文件系统视图在 OOM 前保活；普通 private 文件映射不取得 deny。
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
        // loader 已取得 deny 后格式失败与真实 Mm 分配失败都必须归还原责任。
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
