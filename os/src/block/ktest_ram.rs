//! 基于物理内存页后端的 RAM 块设备（`Disk::prepare_ram`）、块视图（`BlockView`）、文件系统范围声明（`FsClaim`）与元数据缓存（`MetadataCache`）单元测试。
//!
//! # 模块职责与架构定位
//! 本模块属于块设备子系统的内核单元测试（`ktest`），使用专门分配的 64 KiB（16 个 4 KiB 物理页，共 128 个 512B 扇区）
//! 连续物理页构建真实的 RAM 块设备，全面测试块设备子系统在 `BlockView` 分区裁剪、`FsClaim` 读写权限与重叠范围保护、
//! `EndIo::Task` 异步回调阻塞期间的范围占用保持，以及 `MetadataCache` 跨视图共享物理页（`Folio`）与部分失效重读机制。
//!
//! # 核心设计与测试扇区布局
//! 每个测试用例独占一块 64 KiB 的 RAM 盘后端，各用例围绕如下扇区（Sector，512 字节/扇区）与 4 KiB 物理页边界展开验证：
//!
//! ```text
//! 64 KiB RAM Disk (128 Sectors, 512B/Sector, 8 Sectors/4KiB Folio)
//! +------------------+------------------+------------------+------------------+
//! | Sectors 0..8     | Sectors 8..16    | Sectors 16..24   | Sectors 24..128  |
//! | (Folio PFN #0)   | (Folio PFN #1)   | (Folio PFN #2)   | (Remaining)      |
//! +------------------+------------------+------------------+------------------+
//!                          |                  |
//!                          |                  +--> 细分为子范围测试同页共享与部分失效：
//!                          |                       - [16..18) (1 KiB, 页内偏移 0..1024)
//!                          |                       - [18..20) (1 KiB, 页内偏移 1024..2048)
//!                          |                       - [20..22) (1 KiB, 页内偏移 2048..3072)
//!                          +--> 视图边界与 FsClaim 占用保护测试区 ([8..16))
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 全局回调定位锁 `CALLBACK`：叶子自旋锁，仅在注册或进入 `held_endio` 时短暂获取以克隆 `Arc<Callback>`，严禁持锁执行 I/O 或睡眠。
//! - `Callback::task` 自旋锁：叶子自旋锁，仅保护回调执行任务 `Option<Arc<Task>>` 的读写；查询调度状态前必须先克隆 `Arc<Task>` 并释放该锁。
//! - `Folio` 内容睡眠锁（`folio.lock_content()`）与磁盘队列自旋锁（`Disk::queue`）：遵循块层与缓存层标准锁顺序，不在持有自旋锁时获取睡眠锁。

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

/// 全局当前活跃测试夹具的回调观测槽位，供 `held_endio` 函数指针定位当前用例的 `Callback` 实例。
static CALLBACK: SpinLock<Option<Arc<Callback>>> = SpinLock::new(None);

/// `EndIo::Task` 异步完成回调的跨线程同步与状态观测结构。
struct Callback {
    /// 当前测试用例持有的 RAM 块设备引用，供回调内部发起同盘独立读取请求。
    disk: DiskRef,
    /// 控制回调何时退出的外部完成门控（Gate）。
    gate: Completion,
    /// 记录执行 `held_endio` 回调的内核任务（`Task`）强引用，由自旋锁保护。
    task: SpinLock<Option<Arc<Task>>>,
    /// 标记回调内部的同盘同步读取是否已执行完毕。
    peer_finished: AtomicBool,
    /// 标记回调收到的 `IoResult`、物理页所有权状态及同盘读取数据是否全部符合预期。
    valid: AtomicBool,
    /// 记录 `held_endio` 回调被调用的次数。
    calls: AtomicUsize,
    /// 标记 `held_endio` 回调在通过 `gate` 后是否已彻底执行完毕并准备退出。
    returned: AtomicBool,
}

/// 阻塞型 `EndIo::Task` 回调函数：先在同一 RAM 盘上发起并完成一次同步读取（验证回调不会占死 RAM 盘的唯一复制通道），随后在 `gate` 上阻塞等待测试主线程放行。
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

/// 在 5 秒单调时钟期限内轮询条件 `predicate`，未满足时每次睡眠 1 毫秒；超时则返回 `TestError(label)`。
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

/// 等待除当前 CPU 外的所有核心进入空闲（idle）状态，收缩 SLUB 缓存并排空每 CPU 页面缓存，返回全系统 6 个内存域的统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("RAM resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 单页 `Bio` 请求封装，负责管理待提交请求、完成句柄与初始物理缓冲区的元数据快照。
struct Request {
    /// 待提交的 `Bio` 请求实例（提交时通过 `take()` 取出所有权）。
    pending: Option<Bio>,
    /// 该请求对应的异步完成等待句柄。
    completion: BioCompletion,
    /// 构造请求时记录的初始物理页载荷范围（物理地址、长度与 DMA 方向），用于完成后的不变量核对。
    original: PayloadRange,
}

impl Request {
    /// 分配一个独占清零物理页（写操作填充字节 `tag`）并构造单页 `Bio` 请求。
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

        // SAFETY:
        // 1. `pages` 由 `mm::alloc_pages(0, true)` 分配并清零（写操作已将 `[0..len)` 完整初始化为 `tag`），且所有权已移交 `memory` 向量，无任何外部可变别名；
        // 2. 可选的 `EndIo::Task` 回调仅访问已交还 CPU 访问权的缓冲区并等待同盘独立读取与外部 `gate`，不会等待自身完成或磁盘停止。
        let bio = unsafe { Bio::try_new(disk, operation, sector, memory, 0, endio) }
            .map_err(|_| TestError("RAM Bio construction failed"))?;
        let completion = bio.completion();
        Ok(Self {
            pending: Some(bio),
            completion,
            original,
        })
    }

    /// 将请求提交到指定的 `BlockView` 或已授权的 `FsClaim` 通道（若 `claim` 为 `Some` 则经由文件系统声明提交，否则作为裸视图请求提交）。
    fn submit(&mut self, view: &BlockView, claim: Option<&FsClaim>) {
        let bio = self.pending.take().expect("RAM request submitted twice");
        match claim {
            Some(claim) => claim.submit_bio(bio),
            None => view.submit_bio(bio),
        }
    }

    /// 等待请求完成并取回物理页所有权，校验物理地址、长度、DMA 方向、CPU 访问权收回状态以及（若指定 `tag`）读取到的全长字节内容。
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
                // SAFETY:
                // 1. 上方已验证 `!mapped.is_device_owned()`，物理页的访问权已回到 CPU，且通过 `take_memory()` 由当前调用方独占拥有；
                // 2. 该物理页在分配时已清零且经过 RAM 盘读取填充，在 `[0..len)` 范围内构造只读字节切片是安全的。
                let bytes = unsafe { mapped.bytes() };
                context.expect_eq(bytes.iter().all(|byte| *byte == tag), true);
            }
        }
        context.expect_eq(self.completion.take_memory().err(), Some(BlockError::Busy));
        drop(memory);
        Ok(result)
    }

    /// 等待请求完成、校验缓冲区与可选字节图样 `tag`，并断言返回的 `IoResult` 等于 `expected`。
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

/// RAM 块设备测试夹具，持有 64 KiB 底层物理页、RAM `Disk` 实例、专属工作队列与元数据缓存槽位。
struct Ram {
    /// 用例启动前全系统 6 个内存域的统计基线。
    baseline: mm::MemoryStats,
    /// 被测 RAM 块设备引用。
    disk: DiskRef,
    /// 供 RAM 块设备处理请求与完成回调的工作队列。
    queue: WorkQueue,
    /// 供 `held_endio` 回调使用的跨线程同步与观测对象。
    callback: Arc<Callback>,
    /// 测试用例按需创建的元数据缓存槽位（在 `quiesce` 中统一丢弃私有缓存并释放）。
    caches: [Option<Arc<MetadataCache>>; 4],
    /// 承载 64 KiB RAM 磁盘数据的底层高阶物理页（阶数 4，即 16 页）；声明在 `disk` 与 `queue` 之后，确保最后析构释放。
    _backing: mm::OwnedPages,
    /// 记录 `disk.start()` 阶段是否发生错误。
    activation_error: Option<BlockError>,
}

impl Ram {
    /// 分配 64 KiB 清零物理页后端、工作队列与 `Callback` 观测器，初始化并启动 RAM `Disk` 实例。
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
        // SAFETY:
        // 1. `backing` 是当前夹具通过 `mm::alloc_pages(4, true)` 独占分配并清零的 64 KiB 连续物理页，内核恒等/直接映射在整个生命周期内有效；
        // 2. 该物理区间未建立任何外部切片、用户态映射或硬件 DMA 映射，且 `_backing` 字段在 `Ram` 结构体中排在 `disk` 与 `queue` 之后，保证在 `Disk::stop`、工作队列关闭及 `Disk` 析构之后才释放物理页。
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

    /// 检查 RAM 块设备是否在 `set_up` 阶段成功启动。
    fn ready(&self) -> TestResult {
        if self.activation_error.is_some() {
            return Err(TestError("RAM activation failed"));
        }
        Ok(())
    }

    /// 在指定 `BlockView` 上同步执行一次读写请求（读取时自动校验全长字节是否为 `tag`），返回 `IoResult`。
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

    /// 在指定 `BlockView` 上同步执行一次读写请求，并断言返回结果等于 `expected`。
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

    /// 静止化清理流程：放行回调门控、刷新完成任务、丢弃测试期间创建的私有元数据缓存，停止 RAM 盘并同步关闭工作队列。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.callback.gate.complete();
        self.disk.test_flush_completions();
        for cache in self.caches.iter().flatten() {
            // SAFETY:
            // 1. 当前测试用例未挂载持久化文件系统、未创建用户态映射，也未启动后台回写线程；
            // 2. 测试函数体内的局部 `Buffer` 守卫均已释放，关联的异步任务已停止，且底层为当前用例独占的临时 RAM 盘，直接丢弃未落盘的私有脏元数据是安全的。
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

    /// 销毁 `Ram` 夹具（释放 `Disk`、`WorkQueue` 与 64 KiB 物理页后端），等待全系统 6 个内存域统计恢复至测试前基线。
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
    /// 为每个测试用例创建独立的 64 KiB RAM 块设备测试夹具。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Self::prepare(context)
    }
    /// 在测试用例结束（无论成功或失败）时静止化回调、缓存、RAM 磁盘与工作队列。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        Ram::quiesce(self, context)
    }
    /// 在静止化完成后销毁夹具并校验内存零泄漏。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        Ram::tear_down(self, context)
    }
}

#[ktest::suite(fixture = Ram, context = "Thread", tags("block", "ram", "block-view", "P11"), timeout_ms = 30000)]
mod views {
    use super::*;

    /// 测试目标：验证相邻不重叠 `BlockView` 的起始偏移与末尾扇区映射正确，且跨视图边界的越界读写请求在准入阶段被拒绝（返回 `BlockError::InvalidRange`），不会篡改相邻视图的数据。
    #[test]
    fn nonoverlapping_offsets_and_tail_bounds(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        // Step 1: 创建扇区 [8..16) 与 [16..24) 两个相邻视图以及整盘视图，并在各自首尾扇区写入特征字节。
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

        // Step 2: 通过整盘视图与视图相对字节偏移接口读取并校验刚写入的数据。
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

        // Step 3: 提交跨越第一个视图末尾边界（扇区 15 起、长度 1024B）的写请求及越界同步读请求，验证均以 `InvalidRange` 拒绝且相邻扇区 16 的数据完好无损。
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

    /// 测试目标：验证 `FsClaim` 占用期间拒绝同一视图上的裸写请求（`BlockError::ReadOnly`）但不影响相邻视图；
    /// 且当经由 `FsClaim` 提交的请求仍阻塞在 `EndIo::Task` 回调中时，提前 Drop `FsClaim` 门面仍会保持该范围的占用保护，直到回调彻底退出后才允许裸写。
    #[test]
    fn claims_keep_accepted_callback_tail_and_release_range(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        // Step 1: 在 `root` ([8..16)) 上建立读写 `FsClaim`，验证对 `root` 的直接裸写被拒绝，而对不重叠 `scratch` ([16..24)) 的裸写成功。
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

        // Step 2: 经由 `claim` 提交带 `held_endio` 回调的写请求，等待回调内部完成同盘独立读取并阻塞在外部 `gate` 上。
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
        // Step 3: 在回调阻塞期间释放 `claim` 句柄，验证在途回调仍保持 `root` 范围的占用保护，此时对 `root` 的裸写依然返回 `ReadOnly`。
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

        // Step 4: 放行回调门控 `gate` 并等待请求与槽位彻底结束，验证范围占用已解除，对 `root` 的裸写与回读恢复成功。
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

    /// 测试目标：验证共享同一 4 KiB 物理页（`Folio`）的两个相邻视图 A（只读）与 B（读写）中，
    /// 当 A 的 `FsClaim` 与缓存门面全部释放且最后一个 `Buffer` 守卫析构后，仅 A 所属的半页范围失效并允许裸写更新，而 B 所属半页的脏数据与状态完全保留。
    #[test]
    fn retired_range_reloads_raw_tail_without_losing_adjacent_dirty(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        // Step 1: 初始化扇区 [16..18) 与 [18..20) 的底层数据，分别为视图 A（RO）与视图 B（RW）建立声明和共享 `MetadataCache`，验证两者映射到同一物理页（`Folio`）。
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

        // Step 2: 释放视图 A 的 `FsClaim` 与缓存实例，验证仅存的 `left` (`Buffer`) 仍保持范围占用，此时对 A 的裸写仍被拒绝。
        context.expect_eq(fixture.caches[0].as_ref().unwrap().prune_unused(), 0);
        drop(a_claim);
        drop(fixture.caches[0].take());
        drop(a_peer);
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
        // Step 3: 释放 `left` 守卫，验证同一 `Folio` 内仅 A 的字节范围 `[0..1024)` 变为 `NotReady`，而 B 的范围与脏扇区位图保持有效，且对 A 的裸写获准执行。
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

        // Step 4: 重新声明视图 A 并通过缓存读取，验证从 RAM 盘重新加载了裸写修改后的新数据，且相邻视图 B 的未落盘脏数据完好无损。
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

    /// 测试目标：验证外层大范围只读声明（扇区 `[16..22)`）与内层子范围只读声明（扇区 `[18..20)`）重叠时，
    /// 释放外层声明后仅使外层独有的两侧范围（`[16..18)` 与 `[20..22)`）失效并重新允许裸写，而内层声明保护的中间范围保持就绪且拒绝裸写。
    #[test]
    fn overlapping_readonly_claim_preserves_live_middle_range(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        // Step 1: 在扇区 `[16..22)` 写入初始图样，分别建立外层 `[16..22)` 与中间 `[18..20)` 两个重叠只读声明及元数据缓存，并预热读取全部 3 个 1 KiB 块。
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

        // Step 2: 释放外层声明与外层缓存，验证共享 `Folio` 的首尾两段（`[0..1024)` 与 `[2048..3072)`）变为 `NotReady`，而中间段保持可读。
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

        // Step 3: 验证已释放的外层两侧扇区（16 与 20）允许裸写更新，而仍被内层声明占用的中间扇区（18）拒绝裸写。
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

        // Step 4: 重新建立外层只读声明并读取全部 3 个块，验证两侧块读到新写入的数据，中间块仍保留原数据，且三者共用同一个 `Folio`。
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

    /// 测试目标：验证不同起始偏移的 `BlockView` 在访问各自的相对逻辑块 0 时映射到各自独立的物理页（PFN），
    /// 而位于同一 4 KiB 物理页内的不相交视图共享同一个 `Folio` PFN，且每个 `Buffer::update` 严格校验传入的 `FsClaim` 是否覆盖该缓冲区范围。
    #[test]
    fn physical_cache_keeps_view_zero_and_shared_page_ranges(
        context: &mut TestContext<'_>,
        fixture: &mut Ram,
    ) -> TestResult {
        fixture.ready()?;
        // Step 1: 在扇区 0、8、16、18 写入不同特征字节，并为 `[0..8)`、`[8..16)`、`[16..18)`、`[18..20)` 四个视图分别建立读写声明与元数据缓存。
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

        // Step 2: 读取四个缓存各自的相对块 0，断言前两个 4 KiB 视图的 `Folio` PFN 不同，而后两个同页 1 KiB 子视图共享同一个 `Folio` PFN 且页内偏移分别为 0 和 1024。
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

        // Step 3: 验证使用视图 3 的 `FsClaim` 更新视图 2 的 `Buffer` 会因范围不匹配返回 `InvalidRange`，而使用视图 2 自身的 `FsClaim` 可正常更新并仅置位对应扇区的脏位图。
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
