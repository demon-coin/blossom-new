//! 共享文件内存映射（`MAP_SHARED`）的写缺页授权（`page_mkwrite`）、跨核 TLB 写权限撤回（`page_mkclean`）、同址重映射（`MAP_FIXED`）竞态、写回期间的并发拷贝及文件关联 Futex 身份集成测试。
//!
//! # 模块职责与架构定位
//! 本模块属于可写文件系统测试套件（`WriteOwner`）的内存映射子模块，负责验证虚拟内存子系统（`Mm`）与页缓存（Page Cache / `Folio`）、回写引擎（Writeback）以及 Futex 同步原语在共享与私有文件映射路径上的协同不变量。
//! - 上游：由 `os/src/fs/ktest/user/write.rs` 挂载，通过 `WriteOwner` 驱动多核用户态协同进程与后台回写内核线程。
//! - 下游：被测对象涵盖 `Mm::fault`（写缺页 `page_mkwrite` 块预留与脏标记）、跨 CPU 页表写权限清理与 IPI Shootdown、`Mm::mmap`（`MAP_FIXED` 覆盖映射）、`Mm::fork` / `protect` / `copyout` 以及 `futex` 基于 `(Inode, 字节偏移)` 的共享等待队列键值。
//!
//! # 核心设计与状态转换
//! ```text
//!   只读映射叶 PTE (W=0, 指向页缓存 Folio)
//!       │
//!       │ 用户态 Store 或内核 copyout 触发写缺页 (page_mkwrite)
//!       ▼
//!   第一步 (无 Mm 锁): 文件系统预留底层磁盘块 (稀疏洞填补 / OOM 检查)
//!       │
//!       │ 重新获取 Mm 锁并校验 VMA 映射源与页表代次是否仍在位
//!       ▼
//!   第二步 (持 Mm 锁): 升级叶 PTE (W=1) + 标记 Folio 脏扇区位图
//!       │
//!       │ sync_inode / 回写启动 (page_mkclean)
//!       ▼
//!   清除所有共享映射叶 PTE 写权限 (W=0) + 跨核 TLB Shootdown + 转入 Writeback 态
//!       │
//!       ├──► 回写完成: 清除 Writeback 扇区位
//!       └──► 回写途中再次写入: 重新触发 page_mkwrite 升级 W=1 并重设 Dirty 位
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `page_mkwrite` 准备阶段在未持有地址空间锁（`Mm` 锁）的情况下调用文件系统分配磁盘块；当重新获取 `Mm` 锁准备安装可写 PTE 时，必须复核虚拟地址对应的映射源是否已被并发的 `mmap(MAP_FIXED)` 或 `munmap` 替换，防止将旧文件的物理页错误发布到新映射中。
//! - 回写清理（`page_mkclean`）必须在标记页进入回写状态前撤回所有映射该页的地址空间写权限并完成远端 CPU 的 TLB 刷新，确保后续用户态写入必定触发新一轮写缺页并重新标记脏位。

use super::*;

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 测试跨地址空间共享尾页的一致性、EOF 之外写入不扩充文件大小，以及文件扩展时的尾页空洞清零与写权限回收（W15）。
    ///
    /// - **测试目标**：验证两个独立 `Mm` 映射同一文件的末尾页时共享同一个页缓存物理页（`Folio`），验证在文件末尾（EOF）之后但仍在同一尾页内的内存写入不会增大文件 `size` 或分配新块，且后续通过 `write_at` 扩展文件时会自动将该尾页内原本位于旧 EOF 之后的区间清零并撤回已有可写页表项的写权限。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 在文件末尾页对齐偏移处写入 17 字节，分别创建内核别名映射 `alias` 与用户态共享映射 `mm`，比对两者的物理页地址与页缓存 `Folio` 的物理地址。
    ///   2. 用户态进程向页内偏移 0（EOF 内）和偏移 128（超出当前 17 字节 EOF）写入数据，检查文件内容与 `stat` 大小/块数；尝试访问下一页（`SOURCE + 4096`）验证越界缺页。
    ///   3. 调用 `file.write_at(page_offset + 512, &[0x73])` 扩展文件大小，检查旧 EOF（17）至新写入点（512）之间的字节以及 `mm` 和 `alias` 的页表写权限位。
    ///   4. 放行用户态进程再次向偏移 128（此时已处于新 EOF 内）写入 `0x74`，检查 `ktest_mkwrite_count` 计数、文件内容及 `Folio` 脏扇区位。
    /// - **预期结果**：两个 `Mm` 映射的物理页帧与页缓存 `Folio` 完全一致；超出 EOF 的页内内存写入不会改变文件 `size`；`write_at` 扩展文件后，旧 `[17, 512)` 区间内此前残留的页内写入（如偏移 128 处的 `0xde`）被清零，两个 `Mm` 的页表项均被降级为只读（`write == false`）；后续用户态再次写入会重新触发 `page_mkwrite` 并将新数据写入文件且标记 `Folio` 为脏。
    #[test]
    #[ktest(tags("W15", "fs-shared"), min_cpus = 2)]
    fn shared_canonical_tail_growth(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        // Step 1: 在新页起点写入 17 字节，建立两个独立的共享映射（mm 与 alias），验证二者指向同一个页缓存物理页。
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
        // Step 2: 验证跨 Mm 的页内写入立即可见，向超出当前 EOF（偏移 128）的位置写入不会扩充文件大小，且访问完全超出文件页范围的虚拟地址返回 BeyondEof。
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

        // Step 3: 通过 write_at 将文件大小扩展到页内偏移 513，验证旧 EOF 到新写入点之间的区间被清零，且所有共享映射的页表写权限被撤回。
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

        // Step 4: 唤醒用户态再次写入偏移 128，验证重新触发 page_mkwrite 写缺页授权，并将修改后的字节与脏扇区状态同步到文件。
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

    /// 测试远端 CPU 持有可写 TLB 期间由另一 CPU 发起同步回写撤回写权限（`page_mkclean`），以及原 CPU 再次写入时的重触发缺页（W16）。
    ///
    /// - **测试目标**：验证当用户态线程在远端 CPU（`remote`）上已完成首次写缺页并装填了可写 TLB 后，由运行在协调 CPU（`cleaner`）上的内核任务执行 `sync_inode` 回写，能够通过跨核 IPI 撤回远端 CPU 的页表写权限与 TLB 项，使远端 CPU 的下一次内存写入重新触发 `page_mkwrite` 并置脏该页。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动用户态进程在 `remote` CPU 上向共享映射页偏移 0 写入 `0x81`，确认 `mm` 页表项已获得写权限（`write == true`）并记录当前 `mkwrite` 计数。
    ///   2. 在另一个不同的 CPU（`cleaner`）上派发内核任务执行 `sync_inode` 将该文件页刷盘，等待完成后检查 `mm` 和 `alias` 的页表写权限以及 `Folio` 的脏/回写位图。
    ///   3. 放行仍在 `remote` CPU 上运行的用户态进程向同页偏移 1 写入 `0x82`，检查 `mkwrite` 计数、脏扇区位图与文件内容。
    /// - **预期结果**：跨核 `sync_inode` 完成后，两个映射的页表项写权限均被清除，`Folio` 的脏位与回写位归零；远端 CPU 随后执行的写入不会因陈旧的可写 TLB 漏掉脏页追踪，而是准确触发新一轮 `page_mkwrite`，使 `mkwrite` 计数递增并将 `Folio` 重新标记为脏。
    #[test]
    #[ktest(tags("W16", "fs-shared"), min_cpus = 2)]
    fn remote_write_tlb_mkclean_refault(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let file = fixture.open(context, b"/test/write-map", 2)?;
        let inode = fixture.inodes.last().unwrap().clone();

        // Step 1: 让运行在远端 CPU 上的用户态进程完成首次写入，建立可写页表项与活跃的可写 TLB 缓存。
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

        // Step 2: 在另一颗 CPU（cleaner）上执行 sync_inode 同步落盘，触发跨核 TLB Shootdown 撤回写权限并清空脏扇区位。
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

        // Step 3: 放行远端 CPU 上的用户态进程继续向同一页写入，验证写权限撤回生效，触发新的 page_mkwrite 并重新标记脏扇区。
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

    /// 测试 `page_mkwrite` 准备阶段（无锁窗口）与同虚拟地址 `mmap(MAP_FIXED)` 重映射的并发竞态及堆内存耗尽回滚（W17）。
    ///
    /// - **测试目标**：验证当线程在 `page_mkwrite` 完成文件系统块准备、但尚未重新获取 `Mm` 锁安装可写 PTE 的窗口期内，同一虚拟地址被 `mmap(MAP_FIXED)` 替换为另一个文件时，旧文件的物理页不会被错误安装到新映射中；同时验证稀疏空洞页在 `page_mkwrite` 遇到堆 OOM 时不会泄露可写权限。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 准备旧文件 `old`（首字节为 `0, 1`）与替换文件 `replacement`（填充 `0x5a`），注册 `ktest_set_mkwrite_hook` 钩子。
    ///   2. 启动用户态进程触发对 `SOURCE`（映射 `old`）的写缺页，使其在钩子处（文件系统准备已完成、尚未持 `Mm` 锁提交 PTE）阻塞。
    ///   3. 在阻塞期间，调用 `mm.mmap(MmapAddress::Fixed(SOURCE), ...)` 将 `SOURCE` 原子重映射到 `replacement` 文件，随后移除钩子并放行阻塞的写缺页线程。
    ///   4. 检查 `SOURCE` 最终映射的物理页地址、`old` 与 `replacement` 的首字节内容；最后在稀疏文件空洞页上开启堆内存压力并触发写缺页。
    /// - **预期结果**：被暂停的旧 `page_mkwrite` 恢复后识别出映射源已变更，放弃旧页安装并重试对 `replacement` 的写入，最终 `SOURCE` 指向 `replacement` 的物理页，写入的 `0x81` / `0x82` 落在 `replacement` 中而 `old` 保持 `[0, 1]` 不变；堆 OOM 时空洞页的写缺页返回 `NoMemory`，原只读零页映射保持只读且不被替换为可写页。
    #[test]
    #[ktest(tags("W17", "fs-shared", "fs-oom"), min_cpus = 2)]
    fn prepared_mkwrite_same_address_remap(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        // Step 1: 准备旧文件与替换文件的首页缓存 Folio，并安装 mkwrite 同步钩子以拦截写缺页的无锁准备尾部窗口。
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

        // Step 2: 启动用户态进程触发写缺页，等待其阻塞在已完成文件系统准备但尚未持有 Mm 锁安装可写 PTE 的时刻。
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

        // Step 3: 在写缺页暂停期间将同一虚拟地址 SOURCE 通过 MAP_FIXED 重映射到 replacement 文件，随后放行写缺页线程。
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
        // Step 4: 验证重映射生效后用户态写入准确落在 replacement 的物理页上，旧文件 old 的内容未被修改。
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

        // Step 5: 预热稀疏文件的空洞只读映射页与页表结构后耗尽内核堆，验证写缺页（page_mkwrite）分配元数据失败时保持原只读映射，不会错误授予写权限。
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

    /// 测试 `fork` 对共享文件映射的写权限降级、`mprotect` 权限变更、回写在途期间的一致性 `copyout` 重授权及私有映射写时复制（W18）。
    ///
    /// - **测试目标**：验证共享可写文件映射在 `fork` 后子进程页表项初始为只读（强制后续写入经由 `page_mkwrite` 追踪），`mprotect` 切换 `RO -> RW` 不会绕过脏页追踪直接点亮 PTE 写位，以及当文件页正处于块设备回写（Writeback）在途状态时，内核 `copyout` 能在同一物理页上重新获得写授权并保留新脏位；同时验证 `MAP_PRIVATE` 写时复制（COW）与原文件页完全隔离。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动用户态进程完成首次写入并 `fork` 子进程，检查子进程 `child_mm` 在 `SOURCE` 处的物理页与写权限位；对 `child_mm` 依次执行 `protect(RO)` 和 `protect(RW)`，再次检查写权限位。
    ///   2. 挂起块设备完成通知（`test_suspend_completions`），启动后台任务调用 `sync_inode` 使该页进入回写在途状态（`writeback_sectors() != 0` 且父子页表写权限均被清除）。
    ///   3. 在回写尚未完成期间，启动另一后台任务对 `child_mm` 执行 `copyout(SOURCE + 4, &[0x85])`，观察其是否在回写完成前成功返回，并检查物理页地址、PTE 写权限与脏扇区位。
    ///   4. 恢复块设备完成通知使首轮回写结束，检查 `writeback_sectors` 与 `dirty_sectors`；最后建立同一文件的私有可写映射（`private = true`），先读缺页共享原 `Folio`，再写入 `0xef` 触发 COW。
    /// - **预期结果**：`fork` 后子页表项及 `mprotect(RW)` 后的叶 PTE 均保持 `write == false`；回写在途期间 `copyout` 无需等待旧 I/O 结束即可在同一 DMA 物理页上完成 `page_mkwrite` 授权并写入新字节，且旧回写完成后 `dirty_sectors` 依然保持非零（不会丢失在途期间的新修改）；私有映射写入后分配独立物理页，不污染原文件与原 `Folio` 脏位。
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

        // Step 1: 启动父进程并等待其 fork 出子进程，验证子进程继承同一文件物理页但页表项写权限初始关闭，且 mprotect(RW) 不会直接打开硬件写位。
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

        // Step 2: 暂停块设备完成回调并发起 sync_inode，使文件页停留在实际块设备回写在途状态（Writeback），此时父子进程的叶页表写权限均已撤回。
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

        // Step 3: 在块设备回写仍未完成时向子地址空间执行 copyout，验证其能在原文件物理页上完成 page_mkwrite 重授权并立即标记新的脏扇区。
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

        // Step 4: 放行首轮块设备回写完成，验证回写位清零但回写途中新产生的脏扇区位得以保留，随后再次执行 sync_inode 将新脏数据落盘。
        disk.test_resume_completions();
        fixture.join_consumers()?;
        context.expect_eq(folio.writeback_sectors(), 0);
        context.expect_eq(folio.dirty_sectors() != 0, true);
        context.expect_eq(grants < child_mm.ktest_mkwrite_count(), true);
        mapped_bytes(context, &mm, SOURCE + 4, &[0x85]);
        bytes_at(context, &file, 4, &[0x85])?;
        checked(context, crate::fs::writeback::sync_inode(&inode))?;

        // Step 5: 验证私有可写文件映射（MAP_PRIVATE）在只读缺页时复用页缓存物理页，而在写入时触发写时复制（COW）分配新物理页，不弄脏原文件页。
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

    /// 测试共享文件映射的跨进程 Futex 唤醒身份、私有映射与不同 Inode 的隔离，以及页缓存物理页帧（PFN）回收重用后的 Futex 队列隔离（W19）。
    ///
    /// - **测试目标**：验证共享文件映射（`MAP_SHARED`）上的 Futex 等待与唤醒以 `(Inode, 文件偏移)` 为唯一标识，支持跨 `Mm` 与跨虚拟地址（`SOURCE` 与 `SHARED_ALIAS`）匹配；验证私有文件映射（`MAP_PRIVATE`）或不同 Inode 之间互不干扰；并验证当原文件页缓存物理页被释放并重用给另一个 Inode 的同一文件偏移时，新旧文件的 Futex 等待队列严格按各自的 Inode 身份隔离。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 依次以模式 `"f"`（同一文件共享映射）、`"p"`（私有映射）和 `"q"`（不同文件共享映射）启动父子进程，子进程在 `SHARED_ALIAS + 64` 上执行带超时的 `FUTEX_WAIT`，父进程在 `SOURCE + 64` 处采样等待队列快照。
    ///   2. 针对物理页重用场景（模式 `"u"`），让子进程在 `old_inode` 的 `SHARED_ALIAS + 64` 处进入 `FUTEX_WAIT`，记录当时的物理页地址 `old_physical` 与 Futex 键 `old_key`。
    ///   3. 保留一个仅映射 `old_inode` 但尚未缺页的观察地址空间 `identity`，解除父子进程对 `old_inode` 的映射，调用 `prune_unused` 释放 `old_inode` 的缓存页并从页分配器中重新捕获 `old_physical`，随后立即释放使其被 `new_inode`（`other` 文件）的缺页路径分配重用。
    ///   4. 将 `new_inode` 映射到父进程 `SOURCE`，验证其获得的物理页地址恰好等于 `old_physical`；分别比较新旧 Futex 键，并通过 `identity`（对应 `old_inode`）和 `mm`（对应 `new_inode`）查询偏移 64 处的 Futex 等待队列快照。
    /// - **预期结果**：仅共享同文件的 `"f"` 模式能在父进程 `SOURCE + 64` 观察到子进程等待者（相邻偏移 68 及模式 `"p"` / `"q"` 计数均为 0），退出后等待项被清理；即使 `new_inode` 的文件页复用了完全相同的物理页帧 `old_physical` 并映射在相同虚拟地址，`old_key` 与 `new_key` 依然不匹配，原等待者仅在 `old_inode` 关联的 `identity` 视图中可见，而 `new_inode` 视图中的等待计数为 0。
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

        // Step 1: 分别测试同文件共享映射（"f"）、私有文件映射（"p"）以及不同文件共享映射（"q"）下的跨虚拟地址 Futex 等待队列可见性与退出清理。
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

        // Step 2: 启动物理页重用测试（"u"），让子进程在 old_inode 的共享映射页上进入 FUTEX_WAIT，并记录该页的物理地址 old_physical 与 Futex Key。
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

        // Step 3: 创建仅保留 VMA 映射关系而未驻留物理页的观察地址空间 identity，解除原映射并回收 old_inode 的缓存页，精确捕获该物理页帧。
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

        // Step 4: 释放其他临时页并归还目标物理页 old_physical，立即触发 new_inode 的文件页分配使其重用同一物理页帧。
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

        // Step 5: 将重用了 old_physical 的 new_inode 映射到 SOURCE，验证新旧 Futex Key 因所属 Inode 不同而严格隔离，原等待者仍归属于 old_inode。
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
