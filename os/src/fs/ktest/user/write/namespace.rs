//! 普通用户创建、unlink、rename 与目录拓扑的真实 ABI 消费。

use super::*;

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-write", "fs-path", "fs-oom"), resources("root"), timeout_ms = 180000)]
pub(super) mod cases {
    use super::*;

    /// 普通用户通过真实 ABI 创建、竞态排他、umask、短长链接与原始字节名字。
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

    /// 硬链接unlink后旧OFD与两类mmap仍读原内容；旧页真实used后的WB软件责任暂留，末使用等待其尾部。
    /// 旧 OFD 的原页由显式有限请求取得，之后只观察真实写入与 WB 尾部。
    #[test]
    #[ktest(tags("W24", "user-fs", "fs-sync", "fs-inode-lifetime"), min_cpus = 2)]
    fn user_unlink_keeps_old_descriptors_until_real_writeback_retires(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        fixture.prepare_namespace_parent(context)?;

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

    /// 覆盖rename之后新路径指向source，旧target fd与映射仍锚定原canonical inode。
    #[test]
    #[ktest(tags("W25", "user-fs", "fs-path"), min_cpus = 2)]
    fn user_rename_replaces_name_without_replacing_open_target(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        fixture.prepare_namespace_parent(context)?;

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

    /// 真实用户目录改名更新点点与父 nlink，并以共享匿名门同时启动反向父锁顺序。
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
