//! 超级块整页与短窗口共享原页的并发更新消费者。

use alloc::sync::Arc;

use crate::fs::ktest::user::write::WriteOwner;
use crate::fs::ktest::user::UserSuite;
use crate::fs::ktest::*;
use crate::fs::writeback::sync_metadata;

#[ktest::suite(fixture = WriteOwner, suite_resources = UserSuite, context = "Thread", tags("fs-path", "fs-namespace"), resources("root"), timeout_ms = 240000)]
pub(super) mod cases {
    use super::*;

    /// 已打开卷的整页超级块视图与短窗口保持同一原页；两个任务只改 boot 保留字节并恢复。
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

        // 每个真实等待者消费一个 token；准备失败也先放行已发表任务再 join。
        gate.complete();
        gate.complete();
        let joined = fixture.join_consumers();
        let current = full
            .with_read(|bytes| bytes == expected)
            .map_err(FsError::from);

        // 只恢复两个 boot 保留字节，不覆盖超级块或其它访问者的共同记录。
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
