//! 共享文件映射的真实授权、远核撤权、同址替换、fork 与 futex 身份。

use super::*;

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 两个实际 Mm 的同源尾页保留双向内容；EOF 外 store 不分配，普通增长清新有效 gap。
    /// 已映射尾页仍经正式有限准备请求取得 canonical 原页来核对物理身份。
    #[test]
    #[ktest(tags("W15", "fs-shared"), min_cpus = 2)]
    fn shared_canonical_tail_growth(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-reserve", 2)?;
        let page_offset = checked(context, file.stat())?
            .size
            .div_ceil(PAGE_SIZE as u64)
            .checked_mul(PAGE_SIZE as u64)
            .ok_or(TestError("shared tail file offset overflow"))?;
        context.expect_eq(
            checked(context, file.write_at(page_offset, &[0x39; 17]))?,
            17,
        );
        let inode = fixture.inodes.last().unwrap().clone();
        let before = checked(context, file.stat())?;

        let alias = shared_alias(context, &file, page_offset)?;
        let image = fixture.shared_image(context, b"s")?;
        let mm = image.mm.clone();
        if mm
            .copyout(
                SHARED_CONTROL + SHARED_FILE_OFFSET,
                &page_offset.to_le_bytes(),
            )
            .copied
            != core::mem::size_of::<u64>()
        {
            return Err(TestError("shared file offset publication failed"));
        }
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;

        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode()
                .unwrap()
                .file_page(page_offset / PAGE_SIZE as u64, &mut attempt),
        )?;
        context.expect_eq(mm.mapping(SOURCE).unwrap().physical, folio.phys_addr());
        context.expect_eq(
            alias.mapping(SHARED_ALIAS).unwrap().physical,
            folio.phys_addr(),
        );
        mapped_bytes(context, &alias, SHARED_ALIAS, &[0x61]);
        mapped_bytes(context, &alias, SHARED_ALIAS + 128, &[0xde]);
        context.expect_eq(alias.copyout(SHARED_ALIAS + 1, &[0x62]).copied, 1);
        mapped_bytes(context, &mm, SOURCE + 1, &[0x62]);
        let polluted = checked(context, file.stat())?;
        context.expect_eq(
            (polluted.size, polluted.blocks),
            (before.size, before.blocks),
        );

        context.expect_eq(
            mm.fault(SOURCE + 4096, UserAccess::Read),
            Err(VmError::BeyondEof),
        );

        context.expect_eq(
            checked(context, file.write_at(page_offset + 512, &[0x73]))?,
            1,
        );
        bytes_at(context, &file, page_offset, &[0x61, 0x62])?;
        bytes_at(context, &file, page_offset + 2, &[0x39; 15])?;
        bytes_at(context, &file, page_offset + 17, &[0; 495])?;
        bytes_at(context, &file, page_offset + 512, &[0x73])?;
        context.expect_eq(mm.mapping(SOURCE).unwrap().permissions.write, false);
        context.expect_eq(
            alias.mapping(SHARED_ALIAS).unwrap().permissions.write,
            false,
        );

        let grants = mm.ktest_mkwrite_count();
        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;
        context.expect_eq(grants < mm.ktest_mkwrite_count(), true);
        mapped_bytes(context, &alias, SHARED_ALIAS + 128, &[0x74]);
        bytes_at(context, &file, page_offset + 128, &[0x74])?;
        context.expect_eq(checked(context, file.stat())?.size, page_offset + 513);
        context.expect_eq(folio.dirty_sectors() != 0, true);

        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        drop((mm, alias, folio));
        checked(context, crate::fs::writeback::sync_inode(&inode))?;
        context.diagnostic(format_args!(
            "W15 canonical-cross-mm=1 tail-store-no-growth=1 growth-gap-zero=1 renewed-mkwrite=1"
        ));
        Ok(())
    }

    /// 同一远核用户先填真实 W TLB，另一 CPU 同步撤权后由原执行者再次 store。
    /// 同步前使用一份有限请求取得 canonical 原页，保留撤权与 refault 的身份断言。
    /// 清理者固定在发表用户时的协调 CPU，控制 Task 后续迁核不改变真实撤权来源。
    #[test]
    #[ktest(tags("W16", "fs-shared"), min_cpus = 2)]
    fn remote_write_tlb_mkclean_refault(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-map", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();

        let alias = shared_alias(context, &file, 0)?;
        context.expect_eq(alias.copyout(SHARED_ALIAS + 3, &[0x84]).copied, 1);
        let image = fixture.shared_image(context, b"r")?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;

        let remote = fixture.user.running.as_ref().unwrap().target.unwrap();
        mapped_bytes(context, &alias, SHARED_ALIAS, &[0x81]);
        context.expect_eq(mm.mapping(SOURCE).unwrap().permissions.write, true);
        let grants = mm.ktest_mkwrite_count();
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        let cleaner = fixture.user.running.as_ref().unwrap().coordinator;
        context.expect_eq(remote != cleaner, true);

        let cleaned = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("remote cleaner result OOM"))?;
        let recorded = cleaned.clone();
        let syncing = inode.clone();
        fixture.consumers[0] = Some(
            task::prepare_kernel_on(cleaner, move || {
                assert_eq!(cpu::current().id, cleaner);
                *recorded.lock() = Some(crate::fs::writeback::sync_inode(&syncing));
                0
            })
            .map_err(|_| TestError("remote cleaner task preparation failed"))?
            .publish(),
        );
        fixture.join_consumers()?;
        checked(
            context,
            cleaned
                .lock()
                .take()
                .ok_or(TestError("remote cleaner result missing"))?,
        )?;
        context.expect_eq(mm.mapping(SOURCE).unwrap().permissions.write, false);
        context.expect_eq(
            alias.mapping(SHARED_ALIAS).unwrap().permissions.write,
            false,
        );
        context.expect_eq((folio.dirty_sectors(), folio.writeback_sectors()), (0, 0));

        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;
        context.expect_eq(grants < mm.ktest_mkwrite_count(), true);
        context.expect_eq(folio.dirty_sectors() != 0, true);
        mapped_bytes(context, &alias, SHARED_ALIAS + 1, &[0x82]);
        bytes_at(context, &file, 1, &[0x82])?;
        context.diagnostic(format_args!(
            "W16 remote-cpu={} cleaner-cpu={} real-user-w-store=1 post-csd-mkwrite=1 dirty=1",
            remote, cleaner
        ));

        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        drop((mm, alias, folio));
        Ok(())
    }

    /// 真实 prepare 已结束且锁全部退出时 remap 同 VA，旧准备不得向新 source 发布。
    /// 新旧文件的原页分别经独立有限请求准备，再保活到同址 remap 检查结束。
    #[test]
    #[ktest(tags("W17", "fs-shared", "fs-oom"), min_cpus = 2)]
    fn prepared_mkwrite_same_address_remap(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let old = fixture.open(context, b"/test/write-map", 2)?;
        context.expect_eq(checked(context, old.write_at(0, &[0, 1]))?, 2);
        let old_inode = fixture.inodes.last().unwrap().clone();
        let replacement = fixture.open(context, b"/test/write-reserve", 2)?;
        context.expect_eq(checked(context, replacement.write(&[0x5a; 4096]))?, 4096);
        let mut replacement_attempt = ReclaimAttempt::new();
        let replacement_folio = checked(
            context,
            replacement
                .mapping_inode()
                .unwrap()
                .file_page(0, &mut replacement_attempt),
        )?;
        let mut old_attempt = ReclaimAttempt::new();
        let old_folio = checked(
            context,
            old.mapping_inode().unwrap().file_page(0, &mut old_attempt),
        )?;

        let image = fixture.shared_image(context, b"m")?;
        let mm = image.mm.clone();
        let ready =
            Arc::try_new(AtomicBool::new(false)).map_err(|_| TestError("prepare ready OOM"))?;
        let gate = Arc::try_new(Completion::new()).map_err(|_| TestError("prepare gate OOM"))?;
        fixture.prepare_gate = Some(gate.clone());
        let once = ready.clone();
        let pause = gate.clone();
        let hook: Arc<dyn Fn() + Send + Sync> = Arc::try_new(move || {
            if !once.swap(true, Ordering::AcqRel) {
                pause.wait();
            }
        })
        .map_err(|_| TestError("prepare hook OOM"))?;
        mm.ktest_set_mkwrite_hook(Some(hook));

        fixture.start_shared(context, image)?;
        until("real mkwrite prepare did not pause", || {
            ready.load(Ordering::Acquire)
        })?;
        let pid = fixture.user.running.as_ref().unwrap().pid;
        let preparer = fixture
            .user
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("mkwrite preparer binding missing"))?;
        let executor = preparer
            .executor()
            .ok_or(TestError("mkwrite preparer executor missing"))?;
        until("mkwrite preparer did not block at lock-free hook", || {
            sched::test_state(&executor) == State::Blocked
        })?;
        context.expect_eq(
            mm.mapping(SOURCE)
                .is_none_or(|leaf| !leaf.permissions.write),
            true,
        );

        vm(
            context,
            mm.mmap(MmapRequest {
                address: MmapAddress::Fixed(SOURCE),
                length: 4096,
                permissions: RW,
                maximum: RW,
                source: MmapSource::File {
                    file: replacement.clone(),
                    inode: replacement
                        .mapping_inode()
                        .expect("regular replacement has no mapping inode")
                        .clone(),
                    offset: 0,
                    private: false,
                },
            }),
        )?;
        mm.ktest_set_mkwrite_hook(None);
        gate.complete();
        fixture.prepare_gate.take();
        fixture.phase(&mm, 0, 1)?;
        context.expect_eq(
            mm.mapping(SOURCE).unwrap().physical,
            replacement_folio.phys_addr(),
        );
        bytes_at(context, &old, 0, &[0])?;
        bytes_at(context, &replacement, 0, &[0x81])?;
        context.expect_eq(old_folio.phys_addr() != replacement_folio.phys_addr(), true);

        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;
        bytes_at(context, &replacement, 1, &[0x82])?;
        bytes_at(context, &old, 1, &[1])?;

        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        drop((mm, old_folio, replacement_folio));
        checked(context, crate::fs::writeback::sync_inode(&old_inode))?;

        // 暖 hole 页和页表后耗尽真实堆；prepare 失败不能把可写零页发表为成功。
        let sparse = fixture.open(context, b"/test/write-reserve", 2)?;
        let hole_offset = checked(context, sparse.stat())?
            .size
            .div_ceil(PAGE_SIZE as u64)
            .checked_mul(PAGE_SIZE as u64)
            .ok_or(TestError("mkwrite hole offset overflow"))?;
        let data_offset = hole_offset
            .checked_add(2 * PAGE_SIZE as u64)
            .ok_or(TestError("mkwrite sparse data offset overflow"))?;
        context.expect_eq(checked(context, sparse.write_at(data_offset, &[0x71]))?, 1);
        let probe = shared_alias(context, &sparse, hole_offset)?;
        let original = probe.mapping(SHARED_ALIAS).unwrap().physical;
        let pressure = fixture
            .user
            .path
            .owner
            .pressure
            .as_mut()
            .ok_or(TestError("mkwrite heap pressure missing"))?;
        let evidence = {
            let _pin = PreemptGuard::new();
            pressure.exhaust()
        };
        let error = probe.fault(SHARED_ALIAS, UserAccess::Write);
        fixture.user.release_pressure();

        context.expect_eq(evidence, (true, true));
        context.expect_eq(error, Err(VmError::File(FsError::NoMemory)));
        context.expect_eq(probe.mapping(SHARED_ALIAS).unwrap().physical, original);
        context.expect_eq(
            probe.mapping(SHARED_ALIAS).unwrap().permissions.write,
            false,
        );

        vm(context, probe.fault(SHARED_ALIAS, UserAccess::Write))?;
        context.diagnostic(format_args!("W17 real-prepare-pause=1 same-va-remap=1 old-not-committed=1 real-prepare-oom=1 truncate-tested=0"));
        Ok(())
    }

    /// 真实 fork 子叶先 RO，mprotect 只改许可，coherent WB 中 copyout 重新授权并留下新 Dirty。
    /// fork 前的 canonical 原页由显式有限请求准备，观察期间保持其拥有。
    #[test]
    #[ktest(tags("W18", "fs-shared"), min_cpus = 2)]
    fn fork_protect_copyout_writeback_private_cow(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-map", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            file.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;

        let image = fixture.shared_image(context, b"c")?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;
        fixture.phase(&mm, 8, 3)?;
        let child = fixture.child(&mm)?;
        let child_mm = child.mm().ok_or(TestError("fork child MM missing"))?;
        context.expect_eq(
            child_mm.mapping(SOURCE).unwrap().physical,
            folio.phys_addr(),
        );
        context.expect_eq(child_mm.mapping(SOURCE).unwrap().permissions.write, false);
        vm(context, child_mm.protect(SOURCE, 4096, RO))?;
        vm(context, child_mm.protect(SOURCE, 4096, RW))?;
        context.expect_eq(child_mm.mapping(SOURCE).unwrap().permissions.write, false);

        let disk = fixture.user.path.owner.disk()?.clone();
        disk.test_suspend_completions();
        let sync_inode = inode.clone();
        fixture.consumers[0] = Some(
            task::prepare_kernel_on(cpu::current().id, move || {
                if crate::fs::writeback::sync_inode(&sync_inode).is_ok() {
                    0
                } else {
                    1
                }
            })
            .map_err(|_| TestError("WB consumer prepare failed"))?
            .publish(),
        );
        until("file page did not enter real WB", || {
            folio.writeback_sectors() != 0 && disk.test_snapshot().inflight != 0
        })?;
        context.expect_eq(mm.mapping(SOURCE).unwrap().permissions.write, false);
        context.expect_eq(child_mm.mapping(SOURCE).unwrap().permissions.write, false);

        let copying = child_mm.clone();
        let copy = task::prepare_kernel_on(cpu::current().id, move || {
            let progress = copying.copyout(SOURCE + 4, &[0x85]);
            if progress.copied == 1 && progress.error.is_none() {
                0
            } else {
                1
            }
        })
        .map_err(|_| TestError("copyout consumer prepare failed"))?;
        let grants = child_mm.ktest_mkwrite_count();
        fixture.consumers[1] = Some(copy.publish());
        until("coherent copyout did not progress during real WB", || {
            fixture.consumers[1].as_ref().unwrap().try_join().is_some()
        })?;
        context.expect_eq(folio.writeback_sectors() != 0, true);
        context.expect_eq(disk.test_snapshot().inflight != 0, true);
        context.expect_eq(grants < child_mm.ktest_mkwrite_count(), true);
        context.expect_eq(child_mm.mapping(SOURCE).unwrap().permissions.write, true);
        context.expect_eq(
            child_mm.mapping(SOURCE).unwrap().physical,
            folio.phys_addr(),
        );
        context.expect_eq(folio.dirty_sectors() != 0, true);

        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(folio.writeback_sectors(), 0);
        context.expect_eq(folio.dirty_sectors() != 0, true);
        context.expect_eq(grants < child_mm.ktest_mkwrite_count(), true);
        mapped_bytes(context, &mm, SOURCE + 4, &[0x85]);
        bytes_at(context, &file, 4, &[0x85])?;
        checked(context, crate::fs::writeback::sync_inode(&inode))?;

        let mut original = [0; 1];
        context.expect_eq(file.read_at(5, &mut original), Ok(1));
        let private = vm(context, Mm::try_new().map_err(VmError::from))?;
        vm(
            context,
            private.map_file(SHARED_ALIAS, 4096, RW, RW, file.clone(), 0, true),
        )?;
        vm(context, private.fault(SHARED_ALIAS, UserAccess::Read))?;
        context.expect_eq(
            private.mapping(SHARED_ALIAS).unwrap().physical,
            folio.phys_addr(),
        );
        context.expect_eq(private.copyout(SHARED_ALIAS + 5, &[0xef]).copied, 1);
        context.expect_eq(
            private.mapping(SHARED_ALIAS).unwrap().physical != folio.phys_addr(),
            true,
        );
        context.expect_eq(folio.dirty_sectors(), 0);
        bytes_at(context, &file, 5, &original)?;
        mapped_bytes(context, &private, SHARED_ALIAS + 5, &[0xef]);

        WriteOwner::release(&mm, 3)?;
        until("shared fork child did not exit", || child.mm().is_none())?;
        bytes_at(context, &file, 2, &[0x83])?;
        WriteOwner::release(&mm, 4)?;
        fixture.finish_shared(context)?;
        drop((private, child_mm, child, mm, folio, disk));
        context.diagnostic(format_args!("W18 fork-child-ro=1 protect-no-direct-w=1 coherent-copyout-during-wb=1 original-dma-pfn=1 old-completion-preserves-dirty=1 private-cow-isolated=1"));
        Ok(())
    }

    /// 用户 WAIT/WAKE 在跨 Mm/VA 文件字节上匹配；PRIVATE、inode 与实际复用 PFN 分别隔离。
    /// 摘离前准备与归还目标 PFN 后的真实 miss 各使用独立有限请求。
    #[test]
    #[ktest(tags("W19", "fs-shared", "user-futex"), min_cpus = 2)]
    fn file_futex_identity_private_cancel_and_pfn_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-map", 2)?;
        let old_inode = fixture.inodes.last().unwrap().clone();
        context.expect_eq(checked(context, file.write_at(64, &[0; 8]))?, 8);
        let other = fixture.open(context, b"/test/write-reserve", 2)?;
        let new_inode = fixture.inodes.last().unwrap().clone();
        context.expect_eq(checked(context, other.write_at(0, &[0; 4096]))?, 4096);

        for mode in [b"f" as &[u8], b"p", b"q"] {
            let image = fixture.shared_image(context, mode)?;
            let mm = image.mm.clone();
            fixture.start_shared(context, image)?;
            fixture.phase(&mm, 0, 1)?;

            let child = fixture.child(&mm)?;
            let child_mm = child.mm().ok_or(TestError("file futex child MM missing"))?;
            let private = mode[0] == b'p';
            let snapshot =
                UserOwner::futex_waiting(&child_mm, &child, SHARED_ALIAS + 64, private, u32::MAX)?;
            context.expect_eq(snapshot.first_wait_id.is_some(), true);
            context.expect_eq(snapshot.first_deadline.is_some(), true);
            let parent = futex::test_snapshot(&mm, SOURCE + 64, private, u32::MAX)
                .map_err(|_| TestError("file futex parent snapshot failed"))?;
            context.expect_eq(parent.count, usize::from(mode[0] == b'f'));
            if mode[0] == b'f' {
                context.expect_eq(parent.first_wait_id, snapshot.first_wait_id);
                let neighbor = futex::test_snapshot(&mm, SOURCE + 68, false, u32::MAX)
                    .map_err(|_| TestError("file futex neighbor snapshot failed"))?;
                context.expect_eq(neighbor.count, 0);
            }

            WriteOwner::release(&mm, 1)?;
            fixture.finish_shared(context)?;
            let cancelled = futex::test_snapshot(&child_mm, SHARED_ALIAS + 64, private, u32::MAX)
                .map_err(|_| TestError("file futex cancelled snapshot failed"))?;
            context.expect_eq(cancelled.count, 0);
            context.expect_eq(child.mm().is_none(), true);
            drop((child, child_mm, mm));
        }

        checked(context, crate::fs::writeback::sync_inode(&old_inode))?;
        checked(context, crate::fs::writeback::sync_inode(&new_inode))?;
        let image = fixture.shared_image(context, b"u")?;
        let mm = image.mm.clone();
        fixture.start_shared(context, image)?;
        fixture.phase(&mm, 0, 1)?;
        let child = fixture.child(&mm)?;
        let child_mm = child
            .mm()
            .ok_or(TestError("reused PFN futex child MM missing"))?;
        let waiting =
            UserOwner::futex_waiting(&child_mm, &child, SHARED_ALIAS + 64, false, u32::MAX)?;
        let old_physical = child_mm.mapping(SHARED_ALIAS).unwrap().physical;
        let old_key = child_mm
            .futex_read()
            .key(SHARED_ALIAS + 64, false)
            .map_err(|_| TestError("old file futex key failed"))?;

        // 只建无驻留的身份观察范围；旧 node 与 key 均保 inode，不保文件原页。
        let identity = Arc::try_new(vm(context, Mm::try_new().map_err(VmError::from))?)
            .map_err(|_| TestError("old futex identity MM OOM"))?;
        vm(
            context,
            identity.map_file(SOURCE, 4096, RO, RO, file.clone(), 0, false),
        )?;
        vm(context, mm.unmap(SOURCE, 4096))?;
        vm(context, child_mm.unmap(SHARED_ALIAS, 4096))?;
        let mut prepare_attempt = ReclaimAttempt::new();
        drop(checked(
            context,
            other
                .mapping_inode()
                .unwrap()
                .file_page(0, &mut prepare_attempt),
        )?);

        new_inode.mapping().prune_unused();
        let mut pressure = PagePressure::prepare()?;
        context.expect_eq(old_inode.mapping().prune_unused(), 1);
        mm::drain_all_pages();
        let mut recycled = None;
        {
            let _pin = PreemptGuard::new();
            for _ in 0..pressure.pages.capacity() {
                let Some(page) = mm::alloc_pages(0, false) else {
                    break;
                };
                if page.phys_addr() == old_physical {
                    recycled = Some(page);
                    break;
                }
                pressure.pages.push(page);
            }
        }
        let recycled =
            recycled.ok_or(TestError("released file PFN was not actually reacquired"))?;

        // 先归还其他候选并排空 PCP，目标仍保活；最后归还目标后立即走生产 canonical miss。
        drop(pressure);
        mm::drain_all_pages();
        drop(recycled);
        let mut reuse_attempt = ReclaimAttempt::new();
        let reused = other
            .mapping_inode()
            .unwrap()
            .file_page(0, &mut reuse_attempt);
        let reused = checked(context, reused)?;
        context.expect_eq(reused.phys_addr(), old_physical);

        vm(
            context,
            mm.mmap(MmapRequest {
                address: MmapAddress::Fixed(SOURCE),
                length: 4096,
                permissions: RW,
                maximum: RW,
                source: MmapSource::File {
                    file: other.clone(),
                    inode: new_inode.clone(),
                    offset: 0,
                    private: false,
                },
            }),
        )?;
        vm(context, mm.fault(SOURCE, UserAccess::Read))?;
        context.expect_eq(mm.mapping(SOURCE).unwrap().physical, old_physical);
        let new_key = mm
            .futex_read()
            .key(SOURCE + 64, false)
            .map_err(|_| TestError("new file futex key failed"))?;
        context.expect_eq(old_key.matches(&new_key), false);
        context.expect_eq(old_inode.ptr_eq(&new_inode), false);

        vm(context, identity.fault(SOURCE, UserAccess::Read))?;
        let still_waiting = futex::test_snapshot(&identity, SOURCE + 64, false, u32::MAX)
            .map_err(|_| TestError("old live futex identity snapshot failed"))?;
        context.expect_eq(still_waiting.count, 1);
        context.expect_eq(still_waiting.first_wait_id, waiting.first_wait_id);
        let isolated = futex::test_snapshot(&mm, SOURCE + 64, false, u32::MAX)
            .map_err(|_| TestError("reused PFN futex identity snapshot failed"))?;
        context.expect_eq(isolated.count, 0);

        WriteOwner::release(&mm, 1)?;
        fixture.finish_shared(context)?;
        let cancelled = futex::test_snapshot(&identity, SOURCE + 64, false, u32::MAX)
            .map_err(|_| TestError("reused PFN cancelled snapshot failed"))?;
        context.expect_eq(cancelled.count, 0);
        context.diagnostic(format_args!("W19 shared-file-byte-wake=1 private-domain=1 different-inode=1 timed-cancel-join=1 reused-pfn={} live-old-node-isolated=1", reused.phys_addr()));
        drop((old_key, new_key, identity, mm, child_mm, child, reused));
        Ok(())
    }
}
