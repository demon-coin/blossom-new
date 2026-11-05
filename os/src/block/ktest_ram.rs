//! 专用原页承载真实 RAM 磁盘，视图、占用和缓存用例共用生产请求与停止路径。

use ::ktest::{self, Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use virtio_drivers::BufferDirection;

use crate::block::{
    AccessMode, Bio, BioAllocator, BioCompletion, BlkOperation, BlockError, BlockView, Disk,
    DiskRef, EndIo, FsClaim, IoResult, NATIVE_TAG_COUNT,
};
use crate::boot::Region;
use crate::cache::MetadataCache;
use crate::mm::dma::{MappedPages, PayloadRange};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, Task};
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, mm, sched, time, timer};

static CALLBACK: SpinLock<Option<Arc<Callback>>> = SpinLock::new(None);

struct Callback {
    disk: DiskRef,
    gate: Completion,
    task: SpinLock<Option<Arc<Task>>>,
    peer_finished: AtomicBool,
    valid: AtomicBool,
    calls: AtomicUsize,
    returned: AtomicBool,
}

/// 同盘 Task endio 先完成独立 Read 再等待外部 gate，不能占住 RAM 的唯一复制通道。
fn held_endio(result: &IoResult, memory: &[MappedPages]) {
    let owner = CALLBACK
        .lock()
        .as_ref()
        .expect("RAM callback owner missing")
        .clone();
    owner.calls.fetch_add(1, Ordering::AcqRel);
    *owner.task.lock() = Some(task::current_task());
    let mut bytes = [0_u8; 512];
    let peer = owner.disk.read_exact_at(8 * 512, &mut bytes);
    owner.valid.store(
        *result
            == IoResult {
                completed: 1024,
                error: None,
            }
            && memory.len() == 1
            && !memory[0].is_device_owned()
            && peer == Ok(())
            && bytes.iter().all(|byte| *byte == 0x59),
        Ordering::Release,
    );
    owner.peer_finished.store(true, Ordering::Release);
    owner.gate.wait();
    owner.returned.store(true, Ordering::Release);
}

/// 期限仅约束对生产状态的观察，不把 sleep 当作异步完成或资源归还证据。
fn until(label: &'static str, mut predicate: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("RAM observation deadline overflow"))?;
    while !predicate() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 全部参与者结束后整理同一 VM 的真实页缓存，保存完整六域账本。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("RAM resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

struct Request {
    pending: Option<Bio>,
    completion: BioCompletion,
    original: PayloadRange,
}

impl Request {
    /// 每请求消费独占原页，CPU 范围只保存真实物理身份，不构造设备地址或假 DMA。
    fn prepare(
        disk: &Disk,
        operation: BlkOperation,
        sector: u64,
        len: usize,
        tag: u8,
        endio: Option<EndIo>,
    ) -> Result<Self, TestError> {
        if len == 0 || mm::PAGE_SIZE < len {
            return Err(TestError("RAM request exceeds one original page"));
        }
        let pages =
            mm::alloc_pages(0, true).ok_or(TestError("RAM request original page unavailable"))?;
        let direction = if operation == BlkOperation::Write {
            BufferDirection::DriverToDevice
        } else {
            BufferDirection::DeviceToDriver
        };
        let mut mapped = MappedPages::new(pages, 0, len, direction)
            .map_err(|_| TestError("RAM payload preparation failed"))?;
        if operation == BlkOperation::Write {
            for byte in mapped.bytes_mut() {
                byte.write(tag);
            }
        }
        let original = mapped.range();
        let mut memory = Vec::new_in(BioAllocator::Heap);
        memory
            .try_reserve_exact(1)
            .map_err(|_| TestError("RAM payload owner allocation failed"))?;
        memory.push(mapped);
        // SAFETY: 原页独占且已清零，Write 的完整范围又写入图样；CPU 裸别名均已结束。
        // 回调只访问已收权来源，等待独立同盘 Read 与外部 gate，不等待自身或本盘 stop。
        let bio = unsafe { Bio::try_new(disk, operation, sector, memory, 0, endio) }
            .map_err(|_| TestError("RAM Bio construction failed"))?;
        let completion = bio.completion();
        Ok(Self {
            pending: Some(bio),
            completion,
            original,
        })
    }

    /// 原 view 和 FS claim 使用相同父请求，只在正式准入处判断范围和权限。
    fn submit(&mut self, view: &BlockView, claim: Option<&FsClaim>) {
        let bio = self.pending.take().expect("RAM request submitted twice");
        match claim {
            Some(claim) => claim.submit_bio(bio),
            None => view.submit_bio(bio),
        }
    }

    /// 原完成者结束后只取一次原页，成功 Read 核全长内容，拒绝不冒充设备发表。
    fn finish(self, context: &mut TestContext<'_>, tag: Option<u8>) -> Result<IoResult, TestError> {
        until("RAM parent completion did not progress", || {
            self.completion.is_complete()
        })?;
        let result = self.completion.wait();
        let memory = self
            .completion
            .take_memory()
            .map_err(|_| TestError("RAM original payload unavailable"))?;
        context.expect_eq(memory.len(), 1);
        if let Some(mapped) = memory.first() {
            let range = mapped.range();
            context.expect_eq(
                (range.physical, range.length, range.direction),
                (
                    self.original.physical,
                    self.original.length,
                    self.original.direction,
                ),
            );
            context.expect_eq(mapped.is_device_owned(), false);
            if let Some(tag) = tag {
                // SAFETY: 原页完整清零且无共享写者，整个成功 Read 已收权，观察者独占领取原 memory。
                let bytes = unsafe { mapped.bytes() };
                context.expect_eq(bytes.iter().all(|byte| *byte == tag), true);
            }
        }
        context.expect_eq(self.completion.take_memory().err(), Some(BlockError::Busy));
        drop(memory);
        Ok(result)
    }

    /// 完成和预期结果比较分开借用测试上下文，不改变原请求的真实错误值。
    fn expect_result(
        self,
        context: &mut TestContext<'_>,
        tag: Option<u8>,
        expected: IoResult,
    ) -> TestResult {
        let result = self.finish(context, tag)?;
        context.expect_eq(result, expected);
        Ok(())
    }
}

struct Ram {
    baseline: mm::MemoryStats,
    disk: DiskRef,
    queue: WorkQueue,
    callback: Arc<Callback>,
    caches: [Option<Arc<MetadataCache>>; 4],
    // 必须晚于 Disk 与 WorkQueue 析构；只有真实 stop/close 全尾以后才归还此原阶。
    _backing: mm::OwnedPages,
    activation_error: Option<BlockError>,
}

impl Ram {
    /// 在普通 Task 中准备独占 64KiB backing、原完成域和软件槽，所有 fallible 存储先于 start。
    fn prepare(context: &mut TestContext<'_>) -> Result<Self, TestError> {
        let baseline = settled()?;
        let backing =
            mm::alloc_pages(4, true).ok_or(TestError("RAM backing original pages unavailable"))?;
        let region = Region {
            start: backing.phys_addr(),
            end: backing.phys_addr() + backing.len(),
        };
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(NATIVE_TAG_COUNT) + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("RAM completion domain unavailable"))?;
        // SAFETY: 本 fixture 独占完整已清零原页块，永久直映有效；未创建内容 slice、用户映射
        // 或设备映射，backing 保活到 Disk stop、全部 Work close/join 和 Disk 析构以后。
        let disk = match unsafe { Disk::prepare_ram(region, &queue) } {
            Ok(disk) => disk,
            Err(_) => {
                queue.close_sync();
                return Err(TestError("RAM disk preparation failed"));
            }
        };
        let callback = match Arc::try_new(Callback {
            disk: disk.clone(),
            gate: Completion::new(),
            task: SpinLock::new(None),
            peer_finished: AtomicBool::new(false),
            valid: AtomicBool::new(false),
            calls: AtomicUsize::new(0),
            returned: AtomicBool::new(false),
        }) {
            Ok(callback) => callback,
            Err(_) => {
                disk.stop()
                    .map_err(|_| TestError("unpublished RAM disk did not stop"))?;
                drop(disk);
                queue.close_sync();
                return Err(TestError("RAM callback observation unavailable"));
            }
        };
        {
            let mut slot = CALLBACK.lock();
            assert!(
                slot.is_none(),
                "previous RAM callback still owns its locator"
            );
            *slot = Some(callback.clone());
        }
        let activation_error = disk.start().err();
        if let Some(error) = activation_error {
            context.diagnostic(format_args!(
                "RAM activation error={:?}; retaining original owner for quiesce",
                error
            ));
        }
        Ok(Self {
            baseline,
            disk,
            queue,
            callback,
            caches: [const { None }; 4],
            _backing: backing,
            activation_error,
        })
    }

    /// 主体只使用真实成功启动的原磁盘，不更换后端或初始化另一块内存。
    fn ready(&self) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("RAM activation failed"));
        }
        Ok(())
    }

    /// 同步原请求保完整结果；字节图样检查只作用于真正全长成功的读取。
    fn io(
        &self,
        context: &mut TestContext<'_>,
        view: &BlockView,
        operation: BlkOperation,
        sector: u64,
        len: usize,
        tag: u8,
    ) -> Result<IoResult, TestError> {
        let mut request = Request::prepare(&self.disk, operation, sector, len, tag, None)?;
        request.submit(view, None);
        let read_tag = if operation == BlkOperation::Read {
            Some(tag)
        } else {
            None
        };
        request.finish(context, read_tag)
    }

    /// 同步请求返回完整真实结果以后再比较，避免嵌套借用测试上下文。
    #[expect(
        clippy::too_many_arguments,
        reason = "RAM 请求测试显式列出视图、操作、扇区、长度、字节图样和独立预期，不合并实际值与断言来源"
    )]
    fn expect_io(
        &self,
        context: &mut TestContext<'_>,
        view: &BlockView,
        operation: BlkOperation,
        sector: u64,
        len: usize,
        tag: u8,
        expected: IoResult,
    ) -> TestResult {
        let result = self.io(context, view, operation, sector, len, tag)?;
        context.expect_eq(result, expected);
        Ok(())
    }

    /// 全部真实页访问和私有缓存消费者停止以后结束本盘，再同步关闭原完成域。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.callback.gate.complete();
        self.disk.test_flush_completions();
        for cache in self.caches.iter().flatten() {
            // SAFETY: 本例没有 FS 卷、用户映射或 WB 生产者；主体局部 Buffer 已结束，
            // 所有缓存消费者及原 Work 均已停止。相交 RO 范围也无存活消费者，
            // 仅独占 RAM fixture 的私有 metadata 修改无需持久化。
            unsafe { cache.discard_private() }
                .map_err(|_| TestError("RAM private cache retained an external owner"))?;
        }
        self.caches.fill_with(|| None);
        self.disk
            .stop()
            .map_err(|_| TestError("RAM stop did not converge; retaining original backing"))?;
        self.queue.close_sync();
        let locator = { CALLBACK.lock().take() };
        drop(locator);
        let task = { self.callback.task.lock().take() };
        drop(task);
        context.expect_eq(self.disk.test_snapshot().inflight, 0);
        context.expect_eq(self.disk.queue.lock().generators, 0);
        Ok(())
    }

    /// 原 Disk、观察者、backing 和 Work 真实析构以后核本例完整六域，没有跨 VM 合账。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.baseline;
        drop(self);
        let mut after = settled()?;
        until("RAM six-domain resources did not return", || {
            mm::shrink_all();
            mm::drain_all_pages();
            after = mm::stats();
            after == before
        })?;
        context.expect_eq(after, before);
        context.diagnostic(format_args!("RAM resources six-domains-before={:?} six-domains-after={:?} real-stop-work-join=1 original-backing-returned=1", before, after));
        Ok(())
    }
}

impl Fixture<()> for Ram {
    /// 每例使用独立的真实原页 backing，不依赖启动 initrd 或前一例留下的数据。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Self::prepare(context)
    }
    /// 主体失败也释放合法 gate 并等待本盘和原工作域的全部真实尾部。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        Ram::quiesce(self, context)
    }
    /// 安全静止化成功后消费原 fixture 并比较自身资源。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        Ram::tear_down(self, context)
    }
}

#[ktest::suite(fixture = Ram, context = "Thread", tags("block", "ram", "block-view", "P11"), timeout_ms = 30000)]
mod views {
    use super::*;

    /// 两个视图的偏移、最后 sector 与整盘位置一致，越界请求不能发表或修改相邻范围。
    #[test]
    fn nonoverlapping_offsets_and_tail_bounds(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        let first = BlockView::new(fixture.disk.clone(), 8, 8)
            .map_err(|_| TestError("first RAM view unavailable"))?;
        let second = BlockView::new(fixture.disk.clone(), 16, 8)
            .map_err(|_| TestError("second RAM view unavailable"))?;
        let whole = BlockView::whole(fixture.disk.clone());
        for (view, sector, tag) in [
            (&first, 8, 0x19),
            (&first, 15, 0x37),
            (&second, 16, 0xa3),
            (&second, 23, 0xc5),
        ] {
            fixture.expect_io(
                context,
                view,
                BlkOperation::Write,
                sector,
                512,
                tag,
                IoResult {
                    completed: 512,
                    error: None,
                },
            )?;
        }
        for (sector, tag) in [(8, 0x19), (15, 0x37), (16, 0xa3), (23, 0xc5)] {
            fixture.expect_io(
                context,
                &whole,
                BlkOperation::Read,
                sector,
                512,
                tag,
                IoResult {
                    completed: 512,
                    error: None,
                },
            )?;
        }
        let mut bytes = [0_u8; 512];
        first
            .read_exact_at(7 * 512, &mut bytes)
            .map_err(|_| TestError("RAM view final sector read failed"))?;
        context.expect_eq(bytes, [0x37; 512]);
        let before = fixture.disk.test_snapshot().submitted;
        let mut rejected =
            Request::prepare(&fixture.disk, BlkOperation::Write, 15, 1024, 0xee, None)?;
        rejected.submit(&first, None);
        rejected.expect_result(
            context,
            None,
            IoResult {
                completed: 0,
                error: Some(BlockError::InvalidRange),
            },
        )?;
        bytes.fill(0x7b);
        context.expect_eq(
            first.read_exact_at(8 * 512, &mut bytes),
            Err(BlockError::InvalidRange),
        );
        context.expect_eq(bytes, [0x7b; 512]);
        context.expect_eq(fixture.disk.test_snapshot().submitted, before);
        fixture.expect_io(
            context,
            &whole,
            BlkOperation::Read,
            16,
            512,
            0xa3,
            IoResult {
                completed: 512,
                error: None,
            },
        )?;
        Ok(())
    }

    /// 原 FS 范围拒重叠 raw 写，独立 scratch 可写；回调尾仍保占用且同盘 Read 可完成。
    #[test]
    fn claims_keep_accepted_callback_tail_and_release_range(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        let root = BlockView::new(fixture.disk.clone(), 8, 8)
            .map_err(|_| TestError("RAM root view unavailable"))?;
        let scratch = BlockView::new(fixture.disk.clone(), 16, 8)
            .map_err(|_| TestError("RAM scratch view unavailable"))?;
        let claim = Disk::claim_fs(&root, AccessMode::ReadWrite)
            .map_err(|_| TestError("RAM FS claim unavailable"))?;
        let before = fixture.disk.test_snapshot().submitted;
        fixture.expect_io(
            context,
            &root,
            BlkOperation::Write,
            8,
            512,
            0xee,
            IoResult {
                completed: 0,
                error: Some(BlockError::ReadOnly),
            },
        )?;
        context.expect_eq(fixture.disk.test_snapshot().submitted, before);
        fixture.expect_io(
            context,
            &scratch,
            BlkOperation::Write,
            16,
            512,
            0x83,
            IoResult {
                completed: 512,
                error: None,
            },
        )?;
        let mut held = Request::prepare(
            &fixture.disk,
            BlkOperation::Write,
            8,
            1024,
            0x59,
            Some(EndIo::Task(held_endio)),
        )?;
        held.submit(&root, Some(&claim));
        until(
            "RAM callback did not complete its independent same-disk Read",
            || fixture.callback.peer_finished.load(Ordering::Acquire),
        )?;
        until("RAM endio did not block on its external gate", || {
            let task = fixture.callback.task.lock().clone();
            task.is_some_and(|task| sched::test_state(&task) == task::State::Blocked)
        })?;
        context.expect_eq(fixture.callback.valid.load(Ordering::Acquire), true);
        drop(claim);
        let before = fixture.disk.test_snapshot().submitted;
        fixture.expect_io(
            context,
            &root,
            BlkOperation::Write,
            8,
            512,
            0xee,
            IoResult {
                completed: 0,
                error: Some(BlockError::ReadOnly),
            },
        )?;
        context.expect_eq(fixture.disk.test_snapshot().submitted, before);
        fixture.callback.gate.complete();
        held.expect_result(
            context,
            None,
            IoResult {
                completed: 1024,
                error: None,
            },
        )?;
        until("RAM accepted callback or slot tail remained active", || {
            fixture.disk.test_snapshot().inflight == 0 && fixture.disk.queue.lock().mounts == 0
        })?;
        context.expect_eq(fixture.callback.calls.load(Ordering::Acquire), 1);
        context.expect_eq(fixture.callback.returned.load(Ordering::Acquire), true);
        fixture.expect_io(
            context,
            &root,
            BlkOperation::Write,
            8,
            512,
            0x6d,
            IoResult {
                completed: 512,
                error: None,
            },
        )?;
        fixture.expect_io(
            context,
            &root,
            BlkOperation::Read,
            8,
            512,
            0x6d,
            IoResult {
                completed: 512,
                error: None,
            },
        )?;
        Ok(())
    }

    /// RO 热页转交 RW 同源控制后，相邻卷保留脏边界页；退休 A 后 raw 改尾并重开读取新盘面。
    #[test]
    fn retired_range_reloads_raw_tail_without_losing_adjacent_dirty(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        let whole = BlockView::whole(fixture.disk.clone());
        for (sector, tag) in [(16, 0x33), (18, 0x44)] {
            fixture.expect_io(
                context,
                &whole,
                BlkOperation::Write,
                sector,
                1024,
                tag,
                IoResult {
                    completed: 1024,
                    error: None,
                },
            )?;
        }
        let a = BlockView::new(fixture.disk.clone(), 16, 2)
            .map_err(|_| TestError("RAM retired view unavailable"))?;
        let b = BlockView::new(fixture.disk.clone(), 18, 2)
            .map_err(|_| TestError("RAM adjacent view unavailable"))?;
        let a_claim = Disk::claim_fs(&a, AccessMode::ReadOnly)
            .map_err(|_| TestError("RAM retired claim unavailable"))?;
        let a_peer = Disk::claim_fs(&a, AccessMode::ReadOnly)
            .map_err(|_| TestError("RAM duplicate claim unavailable"))?;
        let b_claim = Disk::claim_fs(&b, AccessMode::ReadWrite)
            .map_err(|_| TestError("RAM adjacent claim unavailable"))?;
        for (index, claim) in core::iter::IntoIterator::into_iter([&a_claim, &b_claim]).enumerate()
        {
            fixture.caches[index] = Some(
                MetadataCache::shared(claim, 8)
                    .map_err(|_| TestError("RAM retirement metadata cache unavailable"))?,
            );
        }
        let left = fixture.caches[0]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("RAM retired block read failed"))?;
        let right = fixture.caches[1]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("RAM adjacent block read failed"))?;
        context.expect_eq(left.folio().phys_addr(), right.folio().phys_addr());
        let source = fixture.caches[1]
            .as_ref()
            .unwrap()
            .prepare_reserve()
            .map_err(|_| TestError("RAM hot metadata source upgrade failed"))?;
        context.expect_eq(source.metadata_has_baseline(left.folio()), true);
        context.expect_eq(source.metadata_has_baseline(right.folio()), true);
        drop(source);
        right
            .update(&b_claim, |bytes| bytes.fill(0xb7))
            .map_err(|_| TestError("RAM adjacent dirty update failed"))?;
        let dirty = right.folio().dirty_sectors();
        context.expect_eq(dirty, 0b0000_1100);
        context.expect_eq(fixture.caches[0].as_ref().unwrap().prune_unused(), 0);
        drop(a_claim);
        drop(fixture.caches[0].take());
        drop(a_peer);
        // facade 已撤离，唯一剩下的 A Buffer 仍保活真实 claim，不能提前失效或准入 raw。
        context.expect_eq(right.folio().range_ready(0, 1024), Ok(()));
        fixture.expect_io(
            context,
            &a,
            BlkOperation::Write,
            17,
            512,
            0x6d,
            IoResult {
                completed: 0,
                error: Some(BlockError::ReadOnly),
            },
        )?;
        let (a_ready, b_ready) = {
            let content = right.folio().lock_content();
            drop(left);
            (
                content.range_ready(0, 1024),
                content.range_ready(right.offset(), right.len()),
            )
        };
        context.expect_eq(a_ready, Err(BlockError::NotReady));
        context.expect_eq(b_ready, Ok(()));
        context.expect_eq(right.folio().dirty_sectors(), dirty);
        fixture.expect_io(
            context,
            &a,
            BlkOperation::Write,
            17,
            512,
            0x6d,
            IoResult {
                completed: 512,
                error: None,
            },
        )?;
        let reopened = Disk::claim_fs(&a, AccessMode::ReadOnly)
            .map_err(|_| TestError("RAM retired range did not reopen"))?;
        fixture.caches[0] = Some(
            MetadataCache::shared(&reopened, 8)
                .map_err(|_| TestError("RAM reopened cache unavailable"))?,
        );
        let fresh = fixture.caches[0]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("RAM reopened block read failed"))?;
        context.expect_eq(fresh.folio().phys_addr(), right.folio().phys_addr());
        context.expect_eq(
            fresh.with_read(|bytes| {
                bytes[..512].iter().all(|byte| *byte == 0x33)
                    && bytes[512..].iter().all(|byte| *byte == 0x6d)
            }),
            Ok(true),
        );
        context.expect_eq(
            right.with_read(|bytes| bytes.iter().all(|byte| *byte == 0xb7)),
            Ok(true),
        );
        context.expect_eq(right.folio().dirty_sectors(), dirty);
        context.expect_eq(right.folio().writeback_sectors(), 0);
        Ok(())
    }

    /// 重叠只读 claim 保留公共 sector，结束外层范围后仅两侧空段重新准入 raw。
    #[test]
    fn overlapping_readonly_claim_preserves_live_middle_range(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        let whole = BlockView::whole(fixture.disk.clone());
        fixture.expect_io(
            context,
            &whole,
            BlkOperation::Write,
            16,
            3072,
            0x35,
            IoResult {
                completed: 3072,
                error: None,
            },
        )?;
        let outer = BlockView::new(fixture.disk.clone(), 16, 6)
            .map_err(|_| TestError("RAM outer readonly view unavailable"))?;
        let middle = BlockView::new(fixture.disk.clone(), 18, 2)
            .map_err(|_| TestError("RAM middle readonly view unavailable"))?;
        let outer_claim = Disk::claim_fs(&outer, AccessMode::ReadOnly)
            .map_err(|_| TestError("RAM outer readonly claim unavailable"))?;
        let middle_claim = Disk::claim_fs(&middle, AccessMode::ReadOnly)
            .map_err(|_| TestError("RAM middle readonly claim unavailable"))?;
        for (index, claim) in
            core::iter::IntoIterator::into_iter([&outer_claim, &middle_claim]).enumerate()
        {
            fixture.caches[index] = Some(
                MetadataCache::shared(claim, 8)
                    .map_err(|_| TestError("RAM readonly metadata cache unavailable"))?,
            );
        }
        for block in 0..3 {
            fixture.caches[0]
                .as_ref()
                .unwrap()
                .get(block, 1024, None)
                .map_err(|_| TestError("RAM outer metadata block read failed"))?;
        }
        let live = fixture.caches[1]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("RAM middle metadata block read failed"))?;
        drop(outer_claim);
        drop(fixture.caches[0].take());
        context.expect_eq(live.folio().range_ready(0, 1024), Err(BlockError::NotReady));
        context.expect_eq(
            live.folio().range_ready(2048, 1024),
            Err(BlockError::NotReady),
        );
        context.expect_eq(
            live.with_read(|bytes| bytes.iter().all(|byte| *byte == 0x35)),
            Ok(true),
        );
        for (sector, tag) in [(16, 0x56), (20, 0x78)] {
            fixture.expect_io(
                context,
                &outer,
                BlkOperation::Write,
                sector,
                1024,
                tag,
                IoResult {
                    completed: 1024,
                    error: None,
                },
            )?;
        }
        fixture.expect_io(
            context,
            &middle,
            BlkOperation::Write,
            18,
            1024,
            0xee,
            IoResult {
                completed: 0,
                error: Some(BlockError::ReadOnly),
            },
        )?;
        let reopened = Disk::claim_fs(&outer, AccessMode::ReadOnly)
            .map_err(|_| TestError("RAM overlapping readonly range did not reopen"))?;
        fixture.caches[0] = Some(
            MetadataCache::shared(&reopened, 8)
                .map_err(|_| TestError("RAM reopened readonly cache unavailable"))?,
        );
        for (block, tag) in [(0, 0x56), (1, 0x35), (2, 0x78)] {
            let buffer = fixture.caches[0]
                .as_ref()
                .unwrap()
                .get(block, 1024, None)
                .map_err(|_| TestError("RAM reopened readonly block read failed"))?;
            context.expect_eq(buffer.folio().phys_addr(), live.folio().phys_addr());
            context.expect_eq(
                buffer.with_read(|bytes| bytes.iter().all(|byte| *byte == tag)),
                Ok(true),
            );
        }
        Ok(())
    }

    /// 相对块零按绝对物理页区分，同页不相交范围共用 PFN 且更新不能越过各自 claim。
    #[test]
    fn physical_cache_keeps_view_zero_and_shared_page_ranges(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        let whole = BlockView::whole(fixture.disk.clone());
        for (sector, tag) in [(0, 0x11), (8, 0x22), (16, 0x33), (18, 0x44)] {
            fixture.expect_io(
                context,
                &whole,
                BlkOperation::Write,
                sector,
                1024,
                tag,
                IoResult {
                    completed: 1024,
                    error: None,
                },
            )?;
        }
        let mut claims = Vec::new();
        claims
            .try_reserve_exact(4)
            .map_err(|_| TestError("RAM claim observers unavailable"))?;
        for (index, (start, sectors)) in
            core::iter::IntoIterator::into_iter([(0, 8), (8, 8), (16, 2), (18, 2)]).enumerate()
        {
            let view = BlockView::new(fixture.disk.clone(), start, sectors)
                .map_err(|_| TestError("RAM cache view unavailable"))?;
            let claim = Disk::claim_fs(&view, AccessMode::ReadWrite)
                .map_err(|_| TestError("RAM cache claim unavailable"))?;
            fixture.caches[index] = Some(
                MetadataCache::shared(&claim, 8)
                    .map_err(|_| TestError("RAM physical metadata cache unavailable"))?,
            );
            claims.push(claim);
        }
        let first = fixture.caches[0]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("first RAM metadata block zero failed"))?;
        let second = fixture.caches[1]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("second RAM metadata block zero failed"))?;
        let left = fixture.caches[2]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("left RAM shared-page block failed"))?;
        let right = fixture.caches[3]
            .as_ref()
            .unwrap()
            .get(0, 1024, None)
            .map_err(|_| TestError("right RAM shared-page block failed"))?;
        context.expect_eq(
            first.folio().phys_addr() != second.folio().phys_addr(),
            true,
        );
        context.expect_eq(left.folio().phys_addr(), right.folio().phys_addr());
        context.expect_eq((left.offset(), right.offset()), (0, 1024));
        for (buffer, tag) in [
            (&first, 0x11),
            (&second, 0x22),
            (&left, 0x33),
            (&right, 0x44),
        ] {
            context.expect_eq(
                buffer.with_read(|bytes| bytes.iter().all(|byte| *byte == tag)),
                Ok(true),
            );
        }
        context.expect_eq(
            left.update(&claims[3], |bytes| bytes.fill(0xee)),
            Err(BlockError::InvalidRange),
        );
        left.update(&claims[2], |bytes| bytes.fill(0x95))
            .map_err(|_| TestError("left RAM metadata update failed"))?;
        context.expect_eq(
            left.with_read(|bytes| bytes.iter().all(|byte| *byte == 0x95)),
            Ok(true),
        );
        context.expect_eq(
            right.with_read(|bytes| bytes.iter().all(|byte| *byte == 0x44)),
            Ok(true),
        );
        context.expect_eq(left.folio().dirty_sectors(), 0b0000_0011);
        context.expect_eq(
            fixture.caches[3].as_ref().unwrap().has_dirty_or_writeback(),
            false,
        );
        context.expect_eq(MetadataCache::shared(&claims[1], 7).err().is_some(), true);
        context.diagnostic(format_args!("RAM metadata first-PFN={} second-PFN={} shared-PFN={} exact-view-authorization=1 shared-content-preserved=1", first.folio().phys_addr(), second.folio().phys_addr(), left.folio().phys_addr()));
        Ok(())
    }
}
