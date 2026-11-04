//! 链接、目录与文件页的加载交错及失败退休消费者。

use alloc::sync::Arc;

use crate::block::{BlockError, DiskRef};
use crate::cache::ktest::HeapPressure;
use crate::cache::{BufferRef, MetadataCache, MetadataLoadHook};
use crate::fs::ext4::{codec, BlockMapping};
use crate::fs::ktest::user::write::{
    allocation_counts, block_allocation_bit, inode_allocation_bit, WriteOwner,
};
use crate::fs::ktest::user::UserSuite;
use crate::fs::ktest::*;
use crate::fs::{DentryRef, FsError, InodeKind, ResolveMode, Vfs};

use super::{directory_block, prepare_namespace_view};

struct SymlinkLoadState {
    pressure: Option<HeapPressure>,
    captured: bool,
    failed: Option<BufferRef>,
    prefix: Option<BufferRef>,
    neighbor: Option<BufferRef>,
    neighbor_bytes: [u8; mm::PAGE_SIZE],
    evidence: Option<(bool, bool)>,
    preparation_error: Option<BlockError>,
}

/// 在目标槽已发表、范围读尚未认领时保留准确视图；邻窗口读完后才耗尽原页和堆。
fn observe_symlink_oom(
    cache: &Arc<MetadataCache>,
    disk: &DiskRef,
    state: &Arc<SpinLock<SymlinkLoadState>>,
    block: u64,
    block_size: usize,
    failed_block: u64,
    prefix_block: Option<u64>,
) -> Result<(), BlockError> {
    if block != failed_block {
        return Ok(());
    }
    let mut pressure = {
        let mut state = state.lock();
        if state.captured {
            return Ok(());
        }
        state.captured = true;
        state.pressure.take().expect("symlink load pressure absent")
    };
    let failed = cache
        .cached_buffer(block, block_size)?
        .ok_or(BlockError::NotReady)?;
    let prefix = prefix_block
        .map(|block| {
            cache
                .cached_buffer(block, block_size)?
                .ok_or(BlockError::NotReady)
        })
        .transpose()?;
    if let Some(prefix) = &prefix {
        prefix.with_read(|_| ())?;
    }
    let windows = mm::PAGE_SIZE / block_size;
    let page = block / windows as u64 * windows as u64;
    let neighbor_block = (page..page + windows as u64)
        .find(|&candidate| candidate != block && Some(candidate) != prefix_block);
    let neighbor = neighbor_block
        .map(|block| cache.get(block, block_size, None))
        .transpose()?;
    if let Some(neighbor) = &neighbor {
        neighbor.with_read(|bytes| {
            state.lock().neighbor_bytes[..block_size].copy_from_slice(bytes);
        })?;
    }
    disk.test_flush_completions();
    let evidence = pressure.exhaust();
    let mut state = state.lock();
    state.failed = Some(failed);
    state.prefix = prefix;
    state.neighbor = neighbor;
    state.evidence = Some(evidence);
    state.pressure = Some(pressure);
    Ok(())
}

/// 在原 DentryCache 的 InLookup 停点只阻塞指定原始组件。
fn install_lookup_gate(
    _context: &TestContext<'_>,
    vfs: &Vfs,
    observer: Arc<Observation>,
    name: &'static [u8],
) -> TestResult {
    let hook: Arc<dyn Fn(&DentryRef) + Send + Sync> = Arc::try_new(move |dentry: &DentryRef| {
        if dentry.name().as_slice() == name && observer.hold_loader.load(Ordering::Acquire) {
            observer.loader_entered.fetch_add(1, Ordering::AcqRel);
            observer.gate.wait();
        }
    })
    .map_err(|_| TestError("namespace lookup observer OOM"))?;
    vfs.dcache().ktest_set_lookup_hook(Some(hook));
    Ok(())
}

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-path", "fs-namespace"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 长链接的真实加载 OOM 先取消全部数据窗口再返位；同页邻窗口保留，正常重用不继承旧错。
    #[test]
    #[ktest(tags("W23", "fs-oom"))]
    fn long_symlink_load_oom_cancels_exact_aliases(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        let view = prepare_namespace_view(context, fixture)?;
        let volume = view.volume.clone();
        let cache = volume.metadata_cache().clone();
        let disk = fixture.user.path.owner.disk()?.clone();
        let block_size = volume.geometry().block_size;
        let target_len: usize = if block_size == 1024 { 1084 } else { 128 };
        let blocks = target_len.div_ceil(block_size);
        let parent = checked(context, view.root.inode())?;
        let metadata = checked(context, parent.metadata_snapshot())?;
        let allocation = checked(
            context,
            volume.allocate_inode_for_parent(&metadata, InodeKind::Symlink, 0o777, 0, 0, 0),
        )?;
        let number = allocation.number();
        let hint = (number - 1) / volume.geometry().inodes_per_group;
        let first = checked(context, volume.allocate_block(hint))?;
        let first_block = first.physical();
        let second = if blocks == 2 {
            Some(checked(context, volume.allocate_block(hint))?)
        } else {
            None
        };
        let failed_block = second
            .as_ref()
            .map_or(first_block, |block| block.physical());
        if let Some(second) = second {
            volume.release_unpublished_block(second);
        }
        volume.release_unpublished_block(first);
        volume.release_unpublished_inode(allocation);
        let before = allocation_counts(context, &volume)?;
        let pressure = fixture
            .user
            .path
            .owner
            .pressure
            .take()
            .ok_or(TestError("symlink real OOM pressure absent"))?;
        let state = Arc::try_new(SpinLock::new(SymlinkLoadState {
            pressure: Some(pressure),
            captured: false,
            failed: None,
            prefix: None,
            neighbor: None,
            neighbor_bytes: [0; mm::PAGE_SIZE],
            evidence: None,
            preparation_error: None,
        }))
        .map_err(|_| TestError("symlink load observation OOM"))?;
        let observer = Arc::downgrade(&state);
        let hook_disk = disk.clone();
        let prefix_block = (blocks == 2).then_some(first_block);
        let hook: MetadataLoadHook =
            Arc::try_new(move |cache: &Arc<MetadataCache>, block, size| {
                let Some(state) = observer.upgrade() else {
                    return;
                };
                if let Err(error) = observe_symlink_oom(
                    cache,
                    &hook_disk,
                    &state,
                    block,
                    size,
                    failed_block,
                    prefix_block,
                ) {
                    state.lock().preparation_error = Some(error);
                }
            })
            .map_err(|_| TestError("symlink load hook OOM"))?;
        let vfs = view.vfs.clone();
        let fs_context = view.context.clone();
        let root = view.root.clone();
        cache.ktest_set_load_hook(Some(hook));
        let consumer = fixture.consume_sync(0, move || {
            let target = [b'x'; mm::PAGE_SIZE];
            vfs.symlink_from(
                &fs_context,
                &root,
                b"/w23-symlink-oom",
                &target[..target_len],
                (0, 0),
            )
        });
        let joined = if consumer.is_ok() {
            fixture.join_consumers()
        } else {
            Ok(())
        };
        cache.ktest_set_load_hook(None);
        let pressure = state.lock().pressure.take();
        drop(pressure);
        joined?;
        let consumer = consumer?;
        context.expect_eq(state.lock().captured, true);
        context.expect_eq(state.lock().preparation_error, None);
        context.expect_eq(state.lock().evidence, Some((true, true)));
        context.expect_eq(consumer.result(), Some(Err(FsError::NoMemory)));
        let (failed, prefix, neighbor, neighbor_bytes) = {
            let mut state = state.lock();
            (
                state.failed.take(),
                state.prefix.take(),
                state.neighbor.take(),
                state.neighbor_bytes,
            )
        };
        let failed = failed.ok_or(TestError("symlink failed-range observer absent"))?;
        context.expect_eq(failed.with_read(|_| ()), Err(BlockError::NotReady));
        if let Some(prefix) = &prefix {
            context.expect_eq(prefix.with_read(|_| ()), Err(BlockError::NotReady));
        }
        if let Some(neighbor) = &neighbor {
            context.expect_eq(neighbor.folio().phys_addr(), failed.folio().phys_addr());
            let unchanged = checked(
                context,
                neighbor
                    .with_read(|bytes| bytes == &neighbor_bytes[..block_size])
                    .map_err(FsError::Io),
            )?;
            context.expect_eq(unchanged, true);
        }
        let after_failure = allocation_counts(context, &volume)?;
        context.expect_eq(after_failure, before);
        context.expect_eq(
            checked(context, inode_allocation_bit(&volume, number))?,
            false,
        );
        context.expect_eq(
            checked(context, block_allocation_bit(&volume, failed_block))?,
            Some(false),
        );
        if let Some(prefix_block) = prefix_block {
            context.expect_eq(
                checked(context, block_allocation_bit(&volume, prefix_block))?,
                Some(false),
            );
        }
        context.expect_eq(
            view.vfs
                .resolve(&view.context, b"/w23-symlink-oom", ResolveMode::NoFollow)
                .err(),
            Some(FsError::NotFound),
        );
        let target = [b'x'; mm::PAGE_SIZE];
        checked(
            context,
            view.vfs.symlink_from(
                &view.context,
                &view.root,
                b"/w23-symlink-oom",
                &target[..target_len],
                (0, 0),
            ),
        )?;
        let path = checked(
            context,
            view.vfs
                .resolve(&view.context, b"/w23-symlink-oom", ResolveMode::NoFollow),
        )?;
        let inode = checked(context, path.inode())?;
        context.expect_eq(inode.number(), number);
        context.expect_eq(
            checked(context, inode.read_link())?.as_slice(),
            &target[..target_len],
        );
        context.expect_eq(
            checked(context, inode.map_block((blocks - 1) as u64))?,
            BlockMapping::Mapped {
                physical: failed_block,
                len: 1,
            },
        );
        let restored = checked(
            context,
            failed
                .with_read(|bytes| {
                    let offset = (blocks - 1) * block_size;
                    bytes[..target_len - offset] == target[offset..target_len]
                })
                .map_err(FsError::Io),
        )?;
        context.expect_eq(restored, true);
        checked(
            context,
            view.vfs
                .unlink_from(&view.context, &view.root, b"/w23-symlink-oom", (0, 0)),
        )?;
        drop(path);
        checked(context, volume.wait_inode_reuse(number))?;
        disk.test_flush_completions();
        let after_reuse = allocation_counts(context, &volume)?;
        context.expect_eq(after_reuse, before);
        context.diagnostic(format_args!(
            "W23 long-symlink blocks={} block-size={} post-slot-real-page-and-heap-OOM=1 exact-failed-and-prefix-alias-invalidated=1 same-page-neighbor-preserved={} bits-returned=1 normal-reuse-and-readlink=1 controlled-completion-error=0",
            blocks,
            block_size,
            neighbor.is_some()
        ));
        Ok(())
    }

    /// 原同名加载先结束并唤醒等待者，写者随后发表硬链接；新查找不得复用旧负绑定。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn directory_loading_serializes_link_publication(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        let vfs = fixture.user.path.vfs().clone();
        let fs_context = fixture.user.path.context().clone();
        let root = vfs.root();
        let source = checked(
            context,
            vfs.resolve(&fs_context, b"/pattern", ResolveMode::Follow),
        )?;
        let source_inode = checked(context, source.inode())?.clone();
        drop(fixture.open(context, b"/", 0)?);
        drop(fixture.open(context, b"/pattern", 0)?);
        let observer = fixture.user.path.owner.observer().clone();
        install_lookup_gate(context, &vfs, observer.clone(), b"w28-publish")?;
        observer.hold_loader.store(true, Ordering::Release);
        let before_loads = vfs.dcache().ktest_load_count();
        let parent = root.dentry().clone();
        fixture.user.path.owner.publish(
            0,
            Job::Lookup {
                vfs: vfs.clone(),
                parent: parent.clone(),
                name: b"w28-publish",
            },
        );
        let first_ready = until("metadata loader did not reach actual InLookup", || {
            observer.loader_entered.load(Ordering::Acquire) == 1
                && fixture.user.path.owner.observer().blocked(0)
        });
        let mut observation = None;
        let scenario = (|| -> TestResult {
            first_ready?;
            fixture.user.path.owner.publish(
                1,
                Job::Lookup {
                    vfs: vfs.clone(),
                    parent: parent.clone(),
                    name: b"w28-publish",
                },
            );
            until("metadata lookup waiter did not Block", || {
                vfs.dcache().ktest_waiting(root.dentry(), b"w28-publish") == 1
                    && fixture.user.path.owner.observer().blocked(1)
            })?;
            observation = Some(fixture.consume_sync(0, {
                let vfs = vfs.clone();
                let context = fs_context.clone();
                let start = root.clone();
                let source = source.clone();
                move || vfs.link_from(&source, &context, &start, b"w28-publish", (0, 0))
            })?);
            until(
                "namespace writer did not Block on directory read side",
                || observation.as_ref().is_some_and(|state| state.blocked()),
            )?;
            context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 1);
            Ok(())
        })();
        observer.hold_loader.store(false, Ordering::Release);
        observer.gate.complete();
        let reader_join = fixture.user.path.owner.join_tasks();
        let writer_join = fixture.join_consumers();
        vfs.dcache().ktest_set_lookup_hook(None);
        scenario?;
        reader_join?;
        writer_join?;
        context.expect_eq(
            observation
                .as_ref()
                .ok_or(TestError("namespace writer observation absent"))?
                .result(),
            Some(Ok(())),
        );
        let results = core::mem::replace(&mut *observer.results.lock(), [const { None }; 3]);
        for result in IntoIterator::into_iter(results).take(2) {
            context.expect_eq(
                result
                    .ok_or(TestError("metadata lookup waiter result absent"))?
                    .err(),
                Some(FsError::NotFound),
            );
        }
        context.expect_eq(vfs.dcache().ktest_waiting(root.dentry(), b"w28-publish"), 0);
        let published = checked(
            context,
            vfs.resolve(&fs_context, b"/w28-publish", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, published.inode())?.ptr_eq(&source_inode),
            true,
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 1);
        context.diagnostic(format_args!(
            "W28 metadata-loading unique-InLookup=1 real-waiters=1 writer-Blocked=1 old-result=NotFound post-write-current-inode=1 negative-snapshot-not-republished=1 hardware-EIO-covered=0"
        ));
        Ok(())
    }

    /// 真实目录校验失败结束唯一 Loading；恢复校验字节后同名新链接必须重新加载成功。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn directory_loading_preserves_metadata_error_and_reloads(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        let vfs = fixture.user.path.vfs().clone();
        let fs_context = fixture.user.path.context().clone();
        let root = vfs.root();
        let volume = fixture.volume();
        let root_inode = checked(context, root.inode())?.clone();
        let source = checked(
            context,
            vfs.resolve(&fs_context, b"/pattern", ResolveMode::Follow),
        )?;
        let source_inode = checked(context, source.inode())?.clone();
        drop(fixture.open(context, b"/", 0)?);
        drop(fixture.open(context, b"/pattern", 0)?);
        let (metadata, map_root) = checked(
            context,
            root_inode.with_map_read(|metadata, root| Ok((*metadata, *root))),
        )?;
        let physical = match volume.map_block(&metadata, &map_root, 0, None) {
            Ok(BlockMapping::Mapped { physical, .. }) => physical,
            Ok(_) => return Err(TestError("root directory block is not mapped")),
            Err(error) => {
                context.diagnostic(format_args!("W28 root block lookup error={:?}", error));
                return Err(TestError("root directory mapping failed"));
            }
        };
        let buffer = checked(context, volume.metadata_block(physical))?;
        let geometry = volume.geometry();
        let original = directory_block(&volume, &metadata, &map_root, 0).map_err(|error| {
            context.diagnostic(format_args!("W28 root metadata read error={:?}", error));
            TestError("root metadata read failed")
        })?;
        let mut corrupted = original;
        corrupted[4..6].copy_from_slice(&0u16.to_le_bytes());
        let expected = codec::verify_directory(
            geometry,
            &metadata,
            0,
            &corrupted[..geometry.block_size],
            false,
        )
        .err()
        .ok_or(TestError(
            "controlled metadata corruption did not fail layout or CRC",
        ))?;
        let observer = fixture.user.path.owner.observer().clone();
        install_lookup_gate(context, &vfs, observer.clone(), b"w28-checksum")?;
        observer.hold_loader.store(true, Ordering::Release);
        let before_len = vfs.dcache().len();
        let before_loads = vfs.dcache().ktest_load_count();
        let parent = root.dentry().clone();
        fixture.user.path.owner.publish(
            0,
            Job::Lookup {
                vfs: vfs.clone(),
                parent: parent.clone(),
                name: b"w28-checksum",
            },
        );
        let first_ready = until("checksum loader did not reach published Loading", || {
            observer.loader_entered.load(Ordering::Acquire) == 1
                && fixture.user.path.owner.observer().blocked(0)
        });
        let scenario = (|| -> TestResult {
            first_ready?;
            checked(
                context,
                buffer
                    .update(volume.claim(), |bytes| {
                        bytes.copy_from_slice(&corrupted[..geometry.block_size]);
                    })
                    .map_err(FsError::from),
            )?;
            fixture.user.path.owner.publish(
                1,
                Job::Lookup {
                    vfs: vfs.clone(),
                    parent: parent.clone(),
                    name: b"w28-checksum",
                },
            );
            until("checksum metadata waiter did not Block", || {
                vfs.dcache().ktest_waiting(root.dentry(), b"w28-checksum") == 1
                    && fixture.user.path.owner.observer().blocked(1)
            })?;
            observer.hold_loader.store(false, Ordering::Release);
            observer.gate.complete();
            Ok(())
        })();
        observer.hold_loader.store(false, Ordering::Release);
        observer.gate.complete();
        let joined = fixture.user.path.owner.join_tasks();
        let restored = buffer
            .update(volume.claim(), |bytes| {
                bytes.copy_from_slice(&original[..geometry.block_size])
            })
            .map_err(FsError::from);
        vfs.dcache().ktest_set_lookup_hook(None);
        joined?;
        checked(context, restored)?;
        scenario?;
        let results = core::mem::replace(&mut *observer.results.lock(), [const { None }; 3]);
        for result in IntoIterator::into_iter(results).take(2) {
            context.expect_eq(
                result
                    .ok_or(TestError("metadata waiter result absent"))?
                    .err(),
                Some(expected),
            );
        }
        context.expect_eq(vfs.dcache().len(), before_len);
        context.expect_eq(
            vfs.dcache().ktest_waiting(root.dentry(), b"w28-checksum"),
            0,
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 1);
        context.expect_eq(
            vfs.dcache()
                .lookup(vfs.inodes(), root.dentry(), b"w28-checksum")
                .err(),
            Some(FsError::NotFound),
        );
        checked(
            context,
            vfs.link_from(&source, &fs_context, &root, b"w28-checksum", (0, 0)),
        )?;
        let current = checked(
            context,
            vfs.resolve(&fs_context, b"/w28-checksum", ResolveMode::Follow),
        )?;
        context.expect_eq(
            checked(context, current.inode())?.ptr_eq(&source_inode),
            true,
        );
        context.expect_eq(vfs.dcache().ktest_load_count(), before_loads + 2);
        context.diagnostic(format_args!(
            "W28 metadata-error first-error={:?} shared-waiters=1 failed-placeholder-removed=1 restored-current-value=1 hardware-EIO-covered=0",
            expected
        ));
        Ok(())
    }

    /// 实际文件页读入 completion 停点下同页 writer 等待原填充，再只留下新当前字节。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn file_page_loading_serializes_write_before_current_value(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        let file = fixture.open(context, b"/pattern", 2)?;
        let path = file.path().unwrap().clone();
        let inode = checked(context, path.inode())?.clone();
        let volume = fixture.volume();
        let block_size = volume.geometry().block_size as u64;
        for logical in 0..(mm::PAGE_SIZE as u64 / block_size) {
            checked(context, inode.map_block(logical))?;
        }
        let disk = fixture.user.path.owner.disk()?.clone();
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        fixture.user.path.owner.publish(
            0,
            Job::Read {
                inode: inode.clone(),
                page: 0,
            },
        );
        let loader_ready = until("file page loader did not enter real data I/O", || {
            let snapshot = disk.test_snapshot();
            before.file_read_submitted < snapshot.file_read_submitted
                && snapshot.inflight != 0
                && fixture.user.path.owner.observer().blocked(0)
        });
        let mut writer = None;
        let scenario = (|| -> TestResult {
            loader_ready?;
            fixture.user.path.owner.publish(
                1,
                Job::Read {
                    inode: inode.clone(),
                    page: 0,
                },
            );
            until(
                "same-page reader did not Block on the published folio",
                || fixture.user.path.owner.observer().blocked(1),
            )?;
            let folio = inode
                .mapping()
                .cached_folio(0)
                .ok_or(TestError("loading folio owner disappeared"))?;
            context.expect_eq(folio.is_ready(), false);
            let file_read_requests = disk.test_snapshot().file_read_submitted;
            writer = Some(fixture.consume_sync(0, {
                let file = file.clone();
                move || {
                    let replacement = [0x5a; 32];
                    if file.write_at(0, &replacement)? != replacement.len() {
                        return Err(FsError::Corrupt);
                    }
                    Ok(())
                }
            })?);
            until(
                "same-page writer did not Block behind original page load",
                || writer.as_ref().is_some_and(|state| state.blocked()),
            )?;
            context.expect_eq(disk.test_snapshot().file_read_submitted, file_read_requests);
            context.expect_eq(folio.is_ready(), false);
            drop(folio);
            Ok(())
        })();
        disk.test_resume_completions();
        let readers = fixture.user.path.owner.join_tasks();
        let writers = fixture.join_consumers();
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        scenario?;
        readers?;
        writers?;
        context.expect_eq(
            writer
                .as_ref()
                .ok_or(TestError("file writer observation absent"))?
                .result(),
            Some(Ok(())),
        );
        let results = core::mem::replace(
            &mut *fixture.user.path.owner.observer().results.lock(),
            [const { None }; 3],
        );
        let mut address = None;
        for result in IntoIterator::into_iter(results).take(2) {
            let returned = checked(context, result.ok_or(TestError("file read result absent"))?)?;
            let Returned::Page {
                folio,
                bytes,
                valid,
            } = returned
            else {
                return Err(TestError("file read returned wrong result"));
            };
            context.expect_eq(
                (bytes, valid, folio.is_ready()),
                (mm::PAGE_SIZE, true, true),
            );
            if let Some(address) = address {
                context.expect_eq(folio.phys_addr(), address);
            } else {
                address = Some(folio.phys_addr());
            }
        }
        let mut data = [0xa7; mm::PAGE_SIZE];
        context.expect_eq(checked(context, inode.read_at(0, &mut data))?, data.len());
        context.expect_eq(&data[..32], &[0x5a; 32]);
        context.expect_eq(patterned(&data[32..], 32), true);
        let cached = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("current file folio disappeared"))?;
        context.expect_eq(inode.mapping().is_canonical(0, &cached), true);
        context.expect_eq(Some(cached.phys_addr()), address);
        context.diagnostic(format_args!(
            "W28 file-page real-data-request-accepted=1 completion-paused=1 same-page-reader-Blocked=1 writer-Blocked=1 one-canonical-folio=1 old-read-valid=1 final-current-bytes=1 pre-IO-hook-used=0 hardware-DMA-state-claimed=0"
        ));
        Ok(())
    }

    /// 真实文件页读入的受控完成错误由原等待者共享，失败 folio 回收后新读取重新提交。
    #[test]
    #[ktest(min_cpus = 2, tags("W28"))]
    fn failed_file_page_loading_retries_only_after_real_retirement(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        let file = fixture.open(context, b"/pattern", 0)?;
        let inode = checked(context, file.path().unwrap().inode())?.clone();
        let volume = fixture.volume();
        let block_size = volume.geometry().block_size as u64;
        for logical in 0..(mm::PAGE_SIZE as u64 / block_size) {
            checked(context, inode.map_block(logical))?;
        }
        let disk = fixture.user.path.owner.disk()?.clone();
        let before = disk.test_snapshot();
        disk.test_suspend_completions();
        fixture.user.path.owner.publish(
            0,
            Job::Read {
                inode: inode.clone(),
                page: 0,
            },
        );
        let loader_ready = until("failing page did not submit original data request", || {
            let snapshot = disk.test_snapshot();
            before.file_read_submitted < snapshot.file_read_submitted
                && snapshot.inflight != 0
                && fixture.user.path.owner.observer().blocked(0)
        });
        let scenario = (|| -> TestResult {
            loader_ready?;
            disk.test_fail_next_file_read(BlockError::Io);
            fixture.user.path.owner.publish(
                1,
                Job::Read {
                    inode: inode.clone(),
                    page: 0,
                },
            );
            until(
                "failed page waiter did not Block on unique loading folio",
                || fixture.user.path.owner.observer().blocked(1),
            )?;
            let folio = inode
                .mapping()
                .cached_folio(0)
                .ok_or(TestError("failing page disappeared before completion"))?;
            context.expect_eq(folio.is_ready(), false);
            Ok(())
        })();
        disk.test_resume_completions();
        let readers = fixture.user.path.owner.join_tasks();
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        scenario?;
        readers?;
        context.expect_eq(disk.test_completion_error_pending(), false);
        let results = core::mem::replace(
            &mut *fixture.user.path.owner.observer().results.lock(),
            [const { None }; 3],
        );
        for result in IntoIterator::into_iter(results).take(2) {
            context.expect_eq(
                result
                    .ok_or(TestError("failed page waiter result absent"))?
                    .err(),
                Some(FsError::Io(BlockError::Io)),
            );
        }
        let failed = inode
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("failed page cache owner disappeared"))?;
        context.expect_eq(failed.wait_ready(), Err(BlockError::Io));
        drop(failed);
        context.expect_eq(0 < inode.mapping().prune_unused(), true);
        context.expect_eq(inode.mapping().len(), 0);
        let before_retry = disk.test_snapshot();
        let mut bytes = [0xa7; mm::PAGE_SIZE];
        context.expect_eq(checked(context, inode.read_at(0, &mut bytes))?, bytes.len());
        context.expect_eq(patterned(&bytes, 0), true);
        context.expect_eq(
            before_retry.file_read_submitted < disk.test_snapshot().file_read_submitted,
            true,
        );
        context.diagnostic(format_args!(
            "W28 controlled-data-completion-error=Io original-waiters=2 failed-folio-pruned=1 subsequent-normal-load=1 hardware-EIO-claimed=0"
        ));
        Ok(())
    }
}
