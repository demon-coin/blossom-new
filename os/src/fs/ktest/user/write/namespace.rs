//! 普通用户（非 root）通过系统调用创建文件/目录/链接、`unlink` 延迟回收、`rename` 覆盖与跨父目录双向并发改名拓扑集成测试。
//!
//! # 模块职责与架构定位
//! 本模块属于可写文件系统测试套件（`WriteOwner`）的命名空间修改子模块，负责端到端验证非特权用户（`uid=1001, gid=1001`）通过系统调用 ABI 修改目录项、Inode 链接计数（`nlink`）及目录树拓扑时的权限、持久性与生命周期不变量。
//! - 上游：由 `os/src/fs/ktest/user/write.rs` 挂载，通过 `WriteOwner` 准备 `/test/ns-work` 沙盒目录并以指定用户凭据启动用户态测试进程。
//! - 下游：被测对象涵盖 `openat`（`O_CREAT` / `O_EXCL` / `umask` / `S_ISGID` 继承）、`mkdirat`、`symlinkat`（内联短链接与块存储长链接）、`unlinkat`（孤儿 Inode 在途回写保活与磁盘资源回收）、`renameat2`（原子覆盖、`RENAME_NOREPLACE`、跨目录 `..` 更新及反向重命名锁序）。
//!
//! # 核心设计与生命周期模型
//! ```text
//!   目录项与 Inode 生命周期 (Unlink / Overwrite Rename):
//!   ┌─────────────────────┐    unlinkat / rename 覆盖    ┌─────────────────────────────┐
//!   │ 目录项可见 (nlink>0) │ ───────────────────────────► │ 目录项移除 (nlink==0)        │
//!   └─────────────────────┘                              │ 已有 OFD 与 mmap 仍可读写    │
//!                                                        └──────────────┬──────────────┘
//!                                                                       │
//!                                           最后一个文件句柄与内存映射关闭 + 在途回写 I/O 完成
//!                                                                       ▼
//!                                                        ┌─────────────────────────────┐
//!                                                        │ 释放磁盘数据块与 Inode 位图  │
//!                                                        └─────────────────────────────┘
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 跨父目录 `rename` 必须按照全局统一的目录 Inode 锁顺序（或基于祖先/Inode 号的固定次序）获取源父目录与目标父目录锁，并对涉及硬链接的共享目标文件按固定顺序加锁，防止两个进程同时执行反向跨目录移动（如 `A/x -> B/x` 与 `B/y -> A/y`）时发生 AB-BA 死锁。
//! - 当 `nlink == 0` 的已删除文件仍有在途回写（Writeback）请求尚未完全退出时，即便所有用户态文件描述符、地址空间映射和内核测试句柄均已释放，Inode 回收路径也必须等待在途回写彻底执行完毕后才能释放底层磁盘块与 Inode 槽位。

use super::*;

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 测试非特权用户通过系统调用创建文件与目录、`O_CREAT | O_EXCL` 并发竞争、`umask` 与 `S_ISGID` 权限继承、短/长/悬空符号链接及非 UTF-8 原始字节文件名（W23）。
    ///
    /// - **测试目标**：验证普通用户（`uid=1001, gid=1001`）执行文件创建、目录创建、硬链接与符号链接系统调用时的权限掩码、组 ID 继承、并发排他创建唯一胜出、内联与外置符号链接持久化，以及文件描述符配额上限（`EMFILE`）和路径边界检查。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 初始化命名空间测试目录，以 `uid=1001, gid=1001` 启动用户态进程执行第一轮操作：多线程并发 `O_CREAT | O_EXCL` 创建 `ns-race`，结合不同 `umask` 和特殊权限位创建普通文件、只读文件、子目录 `ns-dir`、含 `0xff` 字节的 `ns-raw-\xff` 以及短（内联）、长（>60 字节）、悬空符号链接。
    ///   2. 在内核侧打开上述产物，逐一核对 Inode 指针同一性、`nlink`、`size`、`mode` 权限位、符号链接目标内容及工作目录 `gid`，随后将符号链接与卷元数据同步刷盘。
    ///   3. 放行用户态进入第二轮操作：创建悬空链接原本指向的目标文件 `ns-missing` 以及带 `S_ISGID` 目录下的子文件 `ns-sgid-child`，验证其 `uid`/`gid` 与继承的权限位。
    ///   4. 结束用户进程并清理测试目录树，验证所有测试路径均已被彻底移除（解析返回 `NotFound`）。
    /// - **预期结果**：并发 `O_CREAT | O_EXCL` 仅一个线程成功创建且各句柄指向同一 Inode；`umask` 与 `S_ISGID` 组继承符合 POSIX 规范；短符号链接、超过 60 字节的外部块符号链接与非 ASCII 原始字节文件名均准确读写并持久化；清理后无残留目录项。
    #[test]
    #[ktest(tags("W23", "user-fs", "fs-path"), min_cpus = 2)]
    fn user_create_exclusive_umask_links_and_raw_names(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        fixture.prepare_namespace_parent(context)?;
        let vfs = fixture.user.path.vfs().clone();
        let fs_context = fixture.user.path.context().clone();

        // Step 1: 以非特权用户（uid=1001, gid=1001）启动测试进程，等待其完成并发排他创建、umask 权限测试、目录创建、原始字节命名及各类符号链接创建。
        let image = fixture.shared_image(context, b"N3")?;
        let mm = image.mm.clone();
        fixture.start_shared_as(
            context,
            image,
            Credentials {
                uid: 1001,
                gid: 1001,
            },
        )?;
        fixture.phase(&mm, 0, 1)?;

        // Step 2: 校验并发排他创建文件的唯一 Inode、各文件与子目录的 mode 权限掩码及原始字节文件内容。
        let race = fixture.open(context, b"/test/ns-work/ns-race", 2)?;
        let race_again = fixture.open(context, b"/test/ns-work/ns-race", 2)?;
        context.expect_eq(
            checked(context, race.path().unwrap().inode())?
                .ptr_eq(checked(context, race_again.path().unwrap().inode())?),
            true,
        );
        let race_stat = checked(context, race.stat())?;
        context.expect_eq((race_stat.links, race_stat.size), (1, 4));
        let mode = fixture.open(context, b"/test/ns-work/ns-mode", 2)?;
        let mode_stat = checked(context, mode.stat())?;
        context.expect_eq(mode_stat.kind, InodeKind::Regular);
        context.expect_eq((mode_stat.links, mode_stat.size), (1, 0));
        context.expect_eq(mode_stat.mode & 0o7777, 0o7750);
        let data = fixture.open(context, b"/test/ns-work/ns-data", 2)?;
        let data_stat = checked(context, data.stat())?;
        context.expect_eq((data_stat.links, data_stat.size), (1, 4));
        context.expect_eq(data_stat.mode & 0o777, 0o640);
        bytes_at(context, &data, 0, b"MODE")?;
        let readonly = fixture.open(context, b"/test/ns-work/ns-ro", 0)?;
        let readonly_stat = checked(context, readonly.stat())?;
        context.expect_eq((readonly_stat.size, readonly_stat.mode & 0o777), (0, 0o640));
        let shared = fixture.open(context, b"/test/ns-work/ns-shared", 0)?;
        let copied = fixture.open(context, b"/test/ns-work/ns-copy", 0)?;
        context.expect_eq(checked(context, shared.stat())?.mode & 0o777, 0o660);
        context.expect_eq(checked(context, copied.stat())?.mode & 0o777, 0o600);
        let directory = fixture.open(context, b"/test/ns-work/ns-dir", 0x10000)?;
        let directory_stat = checked(context, directory.stat())?;
        context.expect_eq(directory_stat.links, 2);
        context.expect_eq(directory_stat.mode & 0o7777, 0o3750);
        let raw = fixture.open(context, b"/test/ns-work/ns-raw-\xff", 2)?;
        let mut raw_bytes = [0; 4];
        context.expect_eq(checked(context, raw.read_at(0, &mut raw_bytes))?, 4);
        context.expect_eq(&raw_bytes, b"RAW!");

        // Step 3: 校验内联短符号链接、跨出内联阈值的长符号链接及悬空符号链接的目标路径，并将元数据同步落盘。
        let short = fixture.hold_nofollow(context, b"/test/ns-work/ns-short")?;
        context.expect_eq(
            checked(context, short.metadata_snapshot())?.kind,
            InodeKind::Symlink,
        );
        context.expect_eq(
            checked(context, short.read_link())?.as_slice(),
            b"/test/ns-work/ns-data",
        );
        let long = fixture.hold_nofollow(context, b"/test/ns-work/ns-long")?;
        context.expect_eq(60 < checked(context, long.read_link())?.len(), true);
        let dangling = fixture.hold_nofollow(context, b"/test/ns-work/ns-dangling")?;
        context.expect_eq(
            checked(context, dangling.read_link())?.as_slice(),
            b"/test/ns-work/ns-missing",
        );
        let workspace = fixture.open(context, b"/test/ns-work", 0x10000)?;
        context.expect_eq(checked(context, workspace.stat())?.gid, 1001);
        context.expect_eq(
            vfs.resolve(&fs_context, b"/test/ns-work/ns-lex", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );

        for inode in [&short, &long, &dangling] {
            checked(context, crate::fs::writeback::sync_inode(inode))?;
        }
        let volume = fixture.volume();
        checked(context, crate::fs::writeback::sync_metadata(&volume))?;
        WriteOwner::release(&mm, 1)?;

        // Step 4: 放行用户进程补齐悬空链接目标并测试 S_ISGID 组继承，最后回收测试目录树并确认所有路径均已清理。
        fixture.phase(&mm, 0, 2)?;
        let missing = fixture.open(context, b"/test/ns-work/ns-missing", 2)?;
        let missing_stat = checked(context, missing.stat())?;
        context.expect_eq((missing_stat.uid, missing_stat.gid), (1001, 1001));
        let inherited = fixture.open(context, b"/test/ns-work/ns-sgid-child", 2)?;
        let inherited_stat = checked(context, inherited.stat())?;
        context.expect_eq((inherited_stat.uid, inherited_stat.gid), (1001, 1001));
        context.expect_eq(inherited_stat.mode & 0o7777, 0o7750);

        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;

        context.expect_eq(
            vfs.resolve(
                &fs_context,
                b"/test/ns-work/ns-permission/denied",
                ResolveMode::Follow,
            )
            .err(),
            Some(FsError::NotFound),
        );
        fixture.remove_namespace_parent(context)?;
        for pathname in [
            b"/test/ns-work/ns-race".as_slice(),
            b"/test/ns-work/ns-mode",
            b"/test/ns-work/ns-data",
            b"/test/ns-work/ns-ro",
            b"/test/ns-work/ns-shared",
            b"/test/ns-work/ns-copy",
            b"/test/ns-work/ns-dir",
            b"/test/ns-work/ns-raw-\xff",
            b"/test/ns-work/ns-short",
            b"/test/ns-work/ns-long",
            b"/test/ns-work/ns-dangling",
            b"/test/ns-work/ns-missing",
            b"/test/ns-work/ns-sgid-child",
        ] {
            context.expect_eq(
                vfs.resolve(&fs_context, pathname, ResolveMode::Follow)
                    .err(),
                Some(FsError::NotFound),
            );
        }
        context.diagnostic(format_args!(
            "W23 ordinary-openat=1 exclusive-race=2 one-winner=1 shared-canonical=1 umask-old-and-inheritance=1 file-mode-special-bits=1 directory-dot-nlink=1 short-long-dangling-symlink=1 nofollow-and-exclusive-symlink=1 raw-byte-name=1 sgid-gid-inheritance=1 fd-reservation-EMFILE=1 permission-tail-slash-root-boundary=1"
        ));
        Ok(())
    }

    /// 测试硬链接 `unlink` 后已打开文件描述符与内存映射的继续访问，以及最后一个引用释放时等待在途回写（Writeback）彻底完成后再回收磁盘块与 Inode（W24）。
    ///
    /// - **测试目标**：验证对多硬链接文件执行 `unlink` 并创建同名新文件后，原打开文件描述（OFD）、共享映射与私有映射仍绑定旧 Inode 且内容互不干扰；验证当旧文件所有链接被删除且所有句柄/映射关闭时，若其缓存页仍有在途块设备回写尚未结束，内核会保持磁盘块与 Inode 分配位直至回写彻底完成。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动用户态进程完成首轮准备（建立硬链接与映射），在内核侧打开 `ns-old`（此时 `links == 2`），获取其首页 `Folio` 并写入 `"OLD!"`。
    ///   2. 构造针对 `old_inode` 的回写批次，通过 `test_hold_file_writeback` 拦截底层块设备回写完成路径，在后台线程发起回写并等待其进入在途阻塞态（`inflight != 0` 且 `writeback_sectors() != 0`）。
    ///   3. 放行用户态执行次轮操作：删除旧路径的所有硬链接并在原路径创建同名新文件写入 `"NEW!"`，核对新旧文件的 Inode 指针、`links`（旧为 0，新为 1）及各自读取的内容。
    ///   4. 放行用户态执行第三轮操作（关闭所有用户态旧文件描述符与映射），并在内核侧丢弃 `old`、`old_inode` 和 `folio` 的全部强引用；在回写仍被挂起期间检查 Inode 缓存观察器、磁盘 Inode 位图与数据块位图。
    ///   5. 恢复块设备回写完成通知（`test_resume_completions`），等待后台回写线程汇合及 Inode 重用栅栏完成，再次检查磁盘 Inode 位图与数据块位图。
    /// - **预期结果**：同名替换文件的创建不影响旧句柄与映射读取 `"OLD!"`；当最后一个文件与内存引用在回写在途期间被释放时，旧 Inode 及其磁盘数据块在回写挂起期间保持已分配状态（防止 DMA 期间块被重用踩内存）；一旦在途回写彻底执行完毕，旧 Inode 槽位与数据块位图立即被清零回收。
    #[test]
    #[ktest(tags("W24", "user-fs", "fs-sync", "fs-inode-lifetime"), min_cpus = 2)]
    fn user_unlink_keeps_old_descriptors_until_real_writeback_retires(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        fixture.prepare_namespace_parent(context)?;

        // Step 1: 启动非特权用户进程建立硬链接与文件映射，并在内核侧向旧文件写入 "OLD!"。
        let image = fixture.shared_image(context, b"N4")?;
        let mm = image.mm.clone();
        fixture.start_shared_as(
            context,
            image,
            Credentials {
                uid: 1001,
                gid: 1001,
            },
        )?;
        fixture.phase(&mm, 0, 1)?;

        let old = fixture.open(context, b"/test/ns-work/ns-old", 2)?;
        let old_inode = fixture.inodes.last().unwrap().clone();
        let number = old_inode.number();
        let volume = fixture.volume();
        let cache = fixture.user.path.owner.cache().clone();
        let watch = cache.watch();
        let disk = fixture.user.path.owner.disk()?.clone();
        let old_stat = checked(context, old.stat())?;
        context.expect_eq((old_stat.links, old_stat.size), (2, 4));
        let mut attempt = ReclaimAttempt::new();
        let folio = checked(
            context,
            old.mapping_inode().unwrap().file_page(0, &mut attempt),
        )?;
        context.expect_eq(old.write_at(0, b"OLD!"), Ok(4));
        let old_tree = tree(context, &old_inode)?;
        context.expect_eq(old_tree.data, 1);

        // Step 2: 启动旧文件首页的块设备回写并挂起其完成回调，使该页保持在途回写状态（Writeback）。
        let ranges = checked(
            context,
            prepare_unlinked_original_writeback(&old_inode, &folio),
        )?;
        let before = disk.test_snapshot();
        disk.test_hold_file_writeback(&folio);
        let syncing_volume = volume.clone();
        let observed = fixture.consume_sync(0, move || {
            let mut ranges = ranges;
            let mut bios = [const { None }; 8];
            let mut completions = [const { None }; 8];
            syncing_volume
                .claim()
                .writeback_batch(
                    &mut ranges,
                    None,
                    &mut bios,
                    &mut completions,
                    syncing_volume.writeback_domain(),
                    || false,
                )
                .map_err(FsError::from)
        })?;
        until(
            "W24 old-file WB did not remain held after real used",
            || {
                let snapshot = disk.test_snapshot();
                before.data_submitted < snapshot.data_submitted
                    && folio.writeback_sectors() != 0
                    && snapshot.inflight != 0
                    && observed.blocked()
                    && disk.test_file_writeback_is_held(&folio)
            },
        )?;

        // Step 3: 放行用户进程 unlink 删光旧文件的目录项并创建同名新文件写入 "NEW!"，验证旧句柄仍读取 "OLD!" 且新旧 Inode 彼此独立。
        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;
        let replacement = fixture.open(context, b"/test/ns-work/ns-old", 2)?;
        let replacement_stat = checked(context, replacement.stat())?;
        context.expect_eq((replacement_stat.links, replacement_stat.size), (1, 4));
        context.expect_eq(
            checked(context, replacement.path().unwrap().inode())?.ptr_eq(&old_inode),
            false,
        );
        context.expect_eq(checked(context, old.stat())?.links, 0);
        bytes_at(context, &old, 0, b"OLD!")?;
        bytes_at(context, &replacement, 0, b"NEW!")?;
        context.expect_eq(disk.test_snapshot().inflight != 0, true);

        // Step 4: 放行用户进程关闭所有旧文件描述符与内存映射，随后释放内核侧持有的全部引用；验证在途回写结束前，磁盘 Inode 与数据块位图仍保持占用。
        WriteOwner::release(&mm, 2)?;
        fixture.phase(&mm, 0, 3)?;
        context.expect_eq(disk.test_file_writeback_is_held(&folio), true);
        context.expect_eq(folio.writeback_sectors() != 0, true);

        fixture.inodes.retain(|inode| !inode.ptr_eq(&old_inode));
        drop((old, old_inode, folio));
        context.expect_eq(disk.test_snapshot().inflight != 0, true);
        context.expect_eq(watch.contains(number), true);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            true,
        );
        for physical in &old_tree.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(true),
            );
        }

        // Step 5: 恢复块设备回写完成回调并等待回写线程与 Inode 退役完成，验证旧文件的磁盘 Inode 位与数据块位被完整释放回收。
        disk.test_resume_completions();
        fixture.join_consumers()?;
        if observed.result() != Some(Ok(())) {
            if let Ok(inode) = cache.iget(&volume, number) {
                fixture.inodes.push(inode);
            }
            return Err(TestError("W24 old-file WB failed after user unlink"));
        }

        disk.test_flush_completions();
        checked(context, watch.wait())?;
        checked(context, volume.wait_inode_reuse(number))?;
        context.expect_eq(watch.contains(number), false);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            false,
        );
        for physical in &old_tree.blocks {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, *physical))?,
                Some(false),
            );
        }

        WriteOwner::release(&mm, 3)?;
        fixture.finish_shared(context)?;
        fixture.remove_namespace_parent(context)?;
        context.expect_eq(
            fixture
                .user
                .path
                .vfs()
                .resolve(
                    fixture.user.path.context(),
                    b"/test/ns-work/ns-old",
                    ResolveMode::Follow,
                )
                .err(),
            Some(FsError::NotFound),
        );
        context.diagnostic(format_args!("W24 real-user-hardlink-unlink=1 old-fd-shared-map-private-map=1 replacement-distinct-canonical=1 accepted-data-WB-used-held-during-unlink=1 old-user-owners-closed-before-WB-release=1 old-slot-and-blocks-held-by-original-WB=1 real-join-and-per-inode-reuse-gate=1 old-slot-and-blocks-returned-after-WB=1"));
        Ok(())
    }

    /// 测试覆盖重命名（`renameat2`）将目标路径原子切换到源 Inode，同时已打开的旧目标文件句柄与映射继续保持原文件数据（W25）。
    ///
    /// - **测试目标**：验证覆盖式 `rename` 原子替换目录项指向源 Inode、将原目标 Inode 的链接计数减至 0，同时不破坏已打开的旧目标文件描述符和内存映射；并验证普通改名与 `RENAME_NOREPLACE` 的冲突拒绝（`EEXIST`）和成功路径。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动用户态进程完成首轮准备，在内核侧预先打开重命名源文件 `ns-source`（内容 `"SRC!"`）与待被覆盖的目标文件 `ns-target`（内容 `"TGT!"`），确认二者为不同 Inode 且 `links` 均为 1。
    ///   2. 放行用户态进程执行次轮操作：执行覆盖重命名（`ns-source -> ns-target`）、普通重命名（`ns-plain-source -> ns-plain-target`）以及 `RENAME_NOREPLACE` 重命名。
    ///   3. 重新通过路径打开 `/test/ns-work/ns-target` 得到 `new_target`，对比 `new_target`、`source` 与 `old_target` 的 Inode 指针、`links` 计数及读取内容，并检查所有源路径是否已不存在。
    ///   4. 放行用户进程退出并清理测试目录树，验证所有目标路径均被删除。
    /// - **预期结果**：覆盖重命名后，按路径新打开的 `ns-target` 与原 `source` 指向同一 Inode 且读取到 `"SRC!"`，`links` 保持为 1；预先打开的 `old_target` 句柄 `links` 变为 0 但仍能完整读取到旧内容 `"TGT!"`；所有源路径解析均返回 `NotFound`。
    #[test]
    #[ktest(tags("W25", "user-fs", "fs-path"), min_cpus = 2)]
    fn user_rename_replaces_name_without_replacing_open_target(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        fixture.prepare_namespace_parent(context)?;

        // Step 1: 启动用户进程准备重命名源文件与目标文件，并在内核侧提前持有 source 与 old_target 的打开句柄。
        let image = fixture.shared_image(context, b"N5")?;
        let mm = image.mm.clone();
        fixture.start_shared_as(
            context,
            image,
            Credentials {
                uid: 1001,
                gid: 1001,
            },
        )?;
        fixture.phase(&mm, 0, 1)?;

        let source = fixture.open(context, b"/test/ns-work/ns-source", 2)?;
        let old_target = fixture.open(context, b"/test/ns-work/ns-target", 2)?;
        let source_stat = checked(context, source.stat())?;
        let target_stat = checked(context, old_target.stat())?;
        context.expect_eq((source_stat.links, source_stat.size), (1, 4));
        context.expect_eq((target_stat.links, target_stat.size), (1, 4));
        context.expect_eq(
            checked(context, source.path().unwrap().inode())?
                .ptr_eq(checked(context, old_target.path().unwrap().inode())?),
            false,
        );

        // Step 2: 放行用户进程执行覆盖重命名、普通重命名与 RENAME_NOREPLACE 操作，验证新路径指向源 Inode 而旧目标句柄仍保留原数据。
        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;

        let new_target = fixture.open(context, b"/test/ns-work/ns-target", 2)?;
        context.expect_eq(
            checked(context, new_target.path().unwrap().inode())?
                .ptr_eq(checked(context, source.path().unwrap().inode())?),
            true,
        );
        context.expect_eq(
            checked(context, new_target.path().unwrap().inode())?
                .ptr_eq(checked(context, old_target.path().unwrap().inode())?),
            false,
        );
        context.expect_eq(checked(context, old_target.stat())?.links, 0);
        context.expect_eq(checked(context, source.stat())?.links, 1);
        bytes_at(context, &new_target, 0, b"SRC!")?;
        bytes_at(context, &old_target, 0, b"TGT!")?;

        let plain_target = fixture.open(context, b"/test/ns-work/ns-plain-target", 0)?;
        bytes_at(context, &plain_target, 0, b"PLAIN")?;
        for pathname in [
            b"/test/ns-work/ns-source".as_slice(),
            b"/test/ns-work/ns-plain-source",
            b"/test/ns-work/ns-noreplace-source",
        ] {
            context.expect_eq(
                fixture
                    .user
                    .path
                    .vfs()
                    .resolve(fixture.user.path.context(), pathname, ResolveMode::Follow)
                    .err(),
                Some(FsError::NotFound),
            );
        }

        // Step 3: 放行用户进程完成映射与旧句柄校验后退出，清理测试目录并确认所有路径已移除。
        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        fixture.remove_namespace_parent(context)?;

        for pathname in [
            b"/test/ns-work/ns-target".as_slice(),
            b"/test/ns-work/ns-plain-target",
            b"/test/ns-work/ns-noreplace-target",
        ] {
            context.expect_eq(
                fixture
                    .user
                    .path
                    .vfs()
                    .resolve(fixture.user.path.context(), pathname, ResolveMode::Follow)
                    .err(),
                Some(FsError::NotFound),
            );
        }
        context.diagnostic(format_args!("W25 real-renameat2-overwrite=1 new-lookup-source-canonical=1 old-target-fd-and-maps-old-content=1 target-nlink-zero=1 source-nlink-preserved=1 ordinary-rename=1 noreplace-EEXIST-and-success=1 metadata-flush-cleanup=1"));
        Ok(())
    }

    /// 测试用户态目录重命名时的 `..` 父指针与父目录 `nlink` 更新，以及跨目录反向并发重命名（`A -> B` 与 `B -> A`）的无死锁执行（W26）。
    ///
    /// - **测试目标**：验证同 Inode 硬链接重命名无副作用（No-op）、将目录移入自身子孙目录被拒绝（`EINVAL`）、跨父目录移动子目录时原子更新子目录的 `..` 项与新旧父目录的 `nlink` 计数，以及通过共享匿名内存门同时释放两个线程执行反向跨父目录重命名和反向硬链接 Inode 锁序重命名时不发生死锁。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动用户态进程完成首轮准备，在内核侧核对初始拓扑：硬链接对 `ns-alias-source` / `ns-alias-target`（指向同一 Inode，`links == 2`）、跨目录硬链接文件 `ns-cross-a` / `ns-cross-b`（同时链接在 `ns-d1` 和 `ns-d2` 下，`links == 3`），以及父目录 `ns-a`（含 2 个子目录，`links == 4`）与 `ns-b`（含 1 个子目录，`links == 3`）。
    ///   2. 放行用户态进程执行次轮操作：执行同 Inode 重命名、非法祖先移动检查、将子目录从 `ns-a` 移入 `ns-b`，并通过共享同步门在多核上同时发起 `ns-a <-> ns-b` 反向子目录移动与 `ns-d1 <-> ns-d2` 反向硬链接覆盖重命名。
    ///   3. 核对操作完成后的目录拓扑：`ns-a` 与 `ns-b` 的 `nlink` 互换为 `(3, 4)`，移动后的子目录 `..` 解析指向新父目录；`ns-cross-a` 与 `ns-cross-b` 的 `nlink` 均降为 1，且在 `ns-d1/b` 和 `ns-d2/a` 下指向正确的目标 Inode 与数据内容；已被删除的旧工作目录 `ns-cwd` 解析返回 `NotFound`。
    ///   4. 结束用户进程并清理测试目录树。
    /// - **预期结果**：所有目录移动与并发反向重命名均无死锁完成；父目录 `nlink` 与子目录 `..` 链接严格一致；跨目录反向硬链接覆盖后剩余链接计数与文件内容完全准确。
    #[test]
    #[ktest(tags("W26", "user-fs", "fs-path"), min_cpus = 2)]
    fn user_directory_rename_topology_and_opposite_race(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        fixture.prepare_namespace_parent(context)?;
        let vfs = fixture.user.path.vfs().clone();
        let fs_context = fixture.user.path.context().clone();

        // Step 1: 启动用户进程构建初始目录树与跨目录硬链接拓扑，核对重命名前各目录与文件的链接计数（nlink）和 Inode 指向。
        let image = fixture.shared_image(context, b"N6")?;
        let mm = image.mm.clone();
        fixture.start_shared_as(
            context,
            image,
            Credentials {
                uid: 1001,
                gid: 1001,
            },
        )?;
        fixture.phase(&mm, 0, 1)?;

        let alias_source = fixture.open(context, b"/test/ns-work/ns-alias-source", 2)?;
        let alias_target = fixture.open(context, b"/test/ns-work/ns-alias-target", 2)?;
        context.expect_eq(
            checked(context, alias_source.path().unwrap().inode())?
                .ptr_eq(checked(context, alias_target.path().unwrap().inode())?),
            true,
        );
        context.expect_eq(checked(context, alias_source.stat())?.links, 2);
        let cross_a = fixture.open(context, b"/test/ns-work/ns-cross-a", 2)?;
        let cross_b = fixture.open(context, b"/test/ns-work/ns-cross-b", 2)?;
        context.expect_eq(
            (
                checked(context, cross_a.stat())?.links,
                checked(context, cross_b.stat())?.links,
            ),
            (3, 3),
        );
        let d1 = fixture.open(context, b"/test/ns-work/ns-d1", 0x10000)?;
        let d2 = fixture.open(context, b"/test/ns-work/ns-d2", 0x10000)?;
        context.expect_eq(
            (
                checked(context, d1.stat())?.links,
                checked(context, d2.stat())?.links,
            ),
            (2, 2),
        );

        let d1_a = fixture.open(context, b"/test/ns-work/ns-d1/a", 0)?;
        let d1_b = fixture.open(context, b"/test/ns-work/ns-d1/b", 0)?;
        let d2_a = fixture.open(context, b"/test/ns-work/ns-d2/a", 0)?;
        let d2_b = fixture.open(context, b"/test/ns-work/ns-d2/b", 0)?;
        context.expect_eq(
            checked(context, d1_a.path().unwrap().inode())?
                .ptr_eq(checked(context, cross_a.path().unwrap().inode())?),
            true,
        );
        context.expect_eq(
            checked(context, d2_a.path().unwrap().inode())?
                .ptr_eq(checked(context, cross_a.path().unwrap().inode())?),
            true,
        );
        context.expect_eq(
            checked(context, d1_b.path().unwrap().inode())?
                .ptr_eq(checked(context, cross_b.path().unwrap().inode())?),
            true,
        );
        context.expect_eq(
            checked(context, d2_b.path().unwrap().inode())?
                .ptr_eq(checked(context, cross_b.path().unwrap().inode())?),
            true,
        );
        drop((d1_a, d1_b, d2_a, d2_b));
        context.expect_eq(
            (
                checked(context, d1.stat())?.links,
                checked(context, d2.stat())?.links,
            ),
            (2, 2),
        );

        let a = fixture.open(context, b"/test/ns-work/ns-a", 0x10000)?;
        let b = fixture.open(context, b"/test/ns-work/ns-b", 0x10000)?;
        let a_stat = checked(context, a.stat())?;
        let b_stat = checked(context, b.stat())?;
        context.expect_eq((a_stat.links, b_stat.links), (4, 3));
        for pathname in [
            b"/test/ns-work/ns-a/x".as_slice(),
            b"/test/ns-work/ns-a/a2",
            b"/test/ns-work/ns-b/y",
            b"/test/ns-work/ns-cwd",
        ] {
            drop(fixture.open(context, pathname, 0x10000)?);
        }

        // Step 2: 放行用户进程执行跨父目录移动及双线程反向并发重命名，随后核对新旧父目录的 nlink 变化与子目录 ".." 的父目录指向。
        WriteOwner::release(&mm, 1)?;
        fixture.phase(&mm, 0, 2)?;

        let a_stat = checked(context, a.stat())?;
        let b_stat = checked(context, b.stat())?;
        context.expect_eq((a_stat.links, b_stat.links), (3, 4));
        let a_path = checked(
            context,
            vfs.resolve(&fs_context, b"/test/ns-work/ns-a", ResolveMode::Follow),
        )?;
        let b_path = checked(
            context,
            vfs.resolve(&fs_context, b"/test/ns-work/ns-b", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(
                context,
                vfs.resolve(&fs_context, b"/test/ns-work/ns-b/x/..", ResolveMode::Follow),
            )?
            .ptr_eq(&b_path),
            true,
        );
        context.expect_eq(
            checked(
                context,
                vfs.resolve(&fs_context, b"/test/ns-work/ns-a/y/..", ResolveMode::Follow),
            )?
            .ptr_eq(&a_path),
            true,
        );
        context.expect_eq(
            vfs.resolve(&fs_context, b"/test/ns-work/ns-a/x", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );
        context.expect_eq(
            vfs.resolve(&fs_context, b"/test/ns-work/ns-b/a2", ResolveMode::Follow)
                .is_ok(),
            true,
        );
        context.expect_eq(
            vfs.resolve(&fs_context, b"/test/ns-work/ns-a/y", ResolveMode::Follow)
                .is_ok(),
            true,
        );

        // Step 3: 校验反向硬链接覆盖重命名后 cross_a 与 cross_b 的剩余链接计数、目录项绑定关系及文件内容。
        context.expect_eq(checked(context, cross_a.stat())?.links, 1);
        context.expect_eq(checked(context, cross_b.stat())?.links, 1);
        bytes_at(context, &cross_a, 0, b"AAAA")?;
        bytes_at(context, &cross_b, 0, b"BBBB")?;
        let d1_b = fixture.open(context, b"/test/ns-work/ns-d1/b", 0)?;
        let d2_a = fixture.open(context, b"/test/ns-work/ns-d2/a", 0)?;
        context.expect_eq(
            checked(context, d1_b.path().unwrap().inode())?
                .ptr_eq(checked(context, cross_a.path().unwrap().inode())?),
            true,
        );
        context.expect_eq(
            checked(context, d2_a.path().unwrap().inode())?
                .ptr_eq(checked(context, cross_b.path().unwrap().inode())?),
            true,
        );
        bytes_at(context, &d1_b, 0, b"AAAA")?;
        bytes_at(context, &d2_a, 0, b"BBBB")?;
        drop((d1_b, d2_a));
        for pathname in [
            b"/test/ns-work/ns-d1/a".as_slice(),
            b"/test/ns-work/ns-d2/b",
        ] {
            context.expect_eq(
                vfs.resolve(&fs_context, pathname, ResolveMode::Follow)
                    .err(),
                Some(FsError::NotFound),
            );
        }

        context.expect_eq(checked(context, alias_source.stat())?.links, 2);
        context.expect_eq(
            checked(context, alias_source.path().unwrap().inode())?
                .ptr_eq(checked(context, alias_target.path().unwrap().inode())?),
            true,
        );
        context.expect_eq(
            vfs.resolve(&fs_context, b"/test/ns-work/ns-cwd", ResolveMode::Follow)
                .err(),
            Some(FsError::NotFound),
        );

        // Step 4: 结束用户进程并清理整个命名空间测试目录树。
        WriteOwner::release(&mm, 2)?;
        fixture.finish_shared(context)?;
        fixture.remove_namespace_parent(context)?;

        for pathname in [
            b"/test/ns-work/ns-alias-source".as_slice(),
            b"/test/ns-work/ns-alias-target",
            b"/test/ns-work/ns-a",
            b"/test/ns-work/ns-b",
            b"/test/ns-work/ns-cwd",
            b"/test/ns-work/ns-d1",
            b"/test/ns-work/ns-d2",
            b"/test/ns-work/ns-d1/a",
            b"/test/ns-work/ns-d1/b",
            b"/test/ns-work/ns-d2/a",
            b"/test/ns-work/ns-d2/b",
            b"/test/ns-work/ns-cross-a",
            b"/test/ns-work/ns-cross-b",
        ] {
            context.expect_eq(
                vfs.resolve(&fs_context, pathname, ResolveMode::Follow)
                    .err(),
                Some(FsError::NotFound),
            );
        }
        context.diagnostic(format_args!("W26 same-inode-same-parent-noop=1 missing-missing-ENOENT=1 descendant-move-EINVAL=1 sequential-cross-parent-dotdot-nlink=1 old-cwd-rmdir-and-create-rejected=1 opposite-parent-clones-shared-gate-reaped=1 reverse-hardlink-inode-lock-order-rename-complete=1 old-fd-content-and-nlink=1 new-parent-lookups-canonical=1 exchange-whiteout-flags-rejected=1"));
        Ok(())
    }
}
