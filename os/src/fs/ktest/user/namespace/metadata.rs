//! Ext4 超级块元数据缓存（`MetadataCache`）多粒度子窗口视图与并发更新集成测试。
//!
//! # 模块定位
//! 本模块属于文件系统命名空间与元数据写入测试子集（`fs::ktest::user::namespace::metadata`），
//! 上游由 `namespace::anchor` 挂载进内核单元测试框架，下游直接调用 `Ext4Volume::metadata_cache`、
//! `BufferRef::update` 与 `sync_metadata` 回写路径。
//!
//! # 核心机制与物理页子窗口共享布局
//! 当 Ext4 块大小小于物理页大小（`PAGE_SIZE = 4096`，例如 1024B 或 2048B 块）时，
//! 磁盘块号 0 所在的同一个物理页（`Folio`）可被不同长度的 `BufferRef` 视图共享访问：
//!
//! ```text
//! 物理页 Folio (PFN = X, 4096 Bytes)
//! +-----------------------+-----------------------+-----------------------------------+
//! | 0..1024 (short 视图)   | 1024..2048            | 2048..4096                        |
//! |   [17]: 引导区保留字节 |                       |                                   |
//! +-----------------------+-----------------------+-----------------------------------+
//! | 0..2048 (wide 视图)                            |                                   |
//! |                           [31]: 引导区保留字节 |                                   |
//! +-----------------------------------------------+-----------------------------------+
//! | 0..4096 (full 整页视图，覆盖引导扇区与 Ext4 超级块偏移 1024..2048)                   |
//! +-----------------------------------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 测试线程通过 `Completion` 门闩同步起跑后，并发调用 `BufferRef::update`。
//! - `BufferRef::update` 内部先获取底层 `Folio` 的内容读写锁，再执行字节修改闭包并标记脏位，
//!   各线程仅在独立持锁期间修改字节，不存在跨把自旋锁的嵌套持有。

use alloc::sync::Arc;

use crate::fs::ktest::user::write::WriteOwner;
use crate::fs::ktest::user::UserSuite;
use crate::fs::ktest::*;
use crate::fs::writeback::sync_metadata;

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-path", "fs-namespace"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 验证同一物理页上的整页（4096B）与子窗口（1024B、2048B）元数据缓存视图共享底层物理页帧，
    /// 且两个并发写任务对 x86 引导扇区保留字节（偏移 17 与 31）的更新不会相互覆盖。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：打开 Ext4 卷，在块号 0 处分别获取 4096B（`full`）、1024B（`short`）和 2048B（`wide`）三个 `BufferRef` 视图，确认三者指向同一物理页帧（PFN）。
    /// - **When**：启动两个并发工作线程在门闩处对齐，同时通过 `short` 视图异或修改偏移 17 字节、通过 `wide` 视图异或修改偏移 31 字节。
    /// - **Then**：`full` 整页视图能够同时观察到两处修改且其余超级块内容完好无损；随后恢复原始引导字节并执行 `sync_metadata` 刷盘成功。
    #[test]
    #[ktest(min_cpus = 2, tags("W02", "fs-metadata-super"))]
    fn initial_super_page_and_short_views_keep_both_updates(
        context: &mut TestContext<'_>,
        fixture: &mut WriteOwner,
    ) -> TestResult {
        fixture.prepare_namespace(context)?;
        let volume = fixture.volume();
        let cache = volume.metadata_cache();
        let full = checked(
            context,
            cache.get(0, mm::PAGE_SIZE, None).map_err(FsError::from),
        )?;
        let short = checked(context, cache.get(0, 1024, None).map_err(FsError::from))?;
        let wide = checked(context, cache.get(0, 2048, None).map_err(FsError::from))?;
        let physical = full.folio().phys_addr();
        context.expect_eq(short.folio().phys_addr(), physical);
        context.expect_eq(wide.folio().phys_addr(), physical);
        let mut expected = checked(
            context,
            full.with_read(|bytes| {
                let mut snapshot = [0; mm::PAGE_SIZE];
                snapshot.copy_from_slice(bytes);
                snapshot
            })
            .map_err(FsError::from),
        )?;
        let original = [expected[17], expected[31]];
        expected[17] ^= 0x6e;
        expected[31] ^= 0xb1;

        // Step 1: 构造并发起跑门闩，启动两个工作线程分别通过 1024B 和 2048B 视图修改偏移 17 与 31。
        let gate = Arc::try_new(crate::sync::Completion::new())
            .map_err(|_| TestError("metadata view start gate OOM"))?;
        let entered = Arc::try_new(AtomicUsize::new(0))
            .map_err(|_| TestError("metadata view participants OOM"))?;
        let mut observations = [None, None];
        let scenario = (|| -> TestResult {
            for (slot, buffer, at, value) in [
                (0, short.clone(), 17, expected[17]),
                (1, wide.clone(), 31, expected[31]),
            ] {
                let gate = gate.clone();
                let entered = entered.clone();
                let volume = volume.clone();
                observations[slot] = Some(fixture.consume_sync(slot, move || {
                    entered.fetch_add(1, Ordering::Release);
                    gate.wait();
                    buffer
                        .update(volume.claim(), |bytes| bytes[at] = value)
                        .map_err(FsError::from)
                })?);
            }
            until(
                "metadata view participants did not reach start gate",
                || entered.load(Ordering::Acquire) == 2,
            )
        })();

        // Step 2: 释放两个门闩许可唤醒全部等待线程（即便前置等待失败也必须先放行已派发线程再汇合）。
        gate.complete();
        gate.complete();
        let joined = fixture.join_consumers();
        let current = full
            .with_read(|bytes| bytes == expected)
            .map_err(FsError::from);

        // Step 3: 仅原地恢复偏移 17 与 31 两个引导扇区保留字节，避免破坏偏移 1024 处的 Ext4 超级块字段。
        let restored = short
            .update(volume.claim(), |bytes| {
                bytes[17] = original[0];
                bytes[31] = original[1];
            })
            .map_err(FsError::from);
        checked(context, restored)?;
        joined?;
        scenario?;
        context.expect_eq(checked(context, current)?, true);
        for observation in observations {
            context.expect_eq(
                observation
                    .ok_or(TestError("metadata view participant absent"))?
                    .result(),
                Some(Ok(())),
            );
        }

        checked(context, sync_metadata(&volume))?;
        context.diagnostic(format_args!(
            "W02 actual-ext4-block-bytes={} initial-super-page={} short-view-bytes={}/{} original-PFN={} participants=2 full-content-preserved=1 restored-only-boot-bytes=1",
            volume.geometry().block_size,
            full.len(),
            short.len(),
            wide.len(),
            physical,
        ));
        Ok(())
    }
}
