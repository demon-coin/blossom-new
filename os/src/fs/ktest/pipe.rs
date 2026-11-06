//! 匿名管道（`pipe`）内核集成测试套件。
//!
//! # 模块定位与职责
//! 本模块位于文件系统测试子系统（`fs::ktest`）中，负责验证 `crate::fs::pipe` 实现的匿名管道在单线程与多任务并发场景下的正确性。
//! 上游由 `ktest` 框架通过 `anchor()` 发现并调度执行；下游直接调用 `pipe::try_new`、`FileRef`、`FileTable`、`PollContext`/`PollTable`、以及内存管理（`Mm`）和任务调度（`task`/`sched`）接口。
//!
//! # 核心机制与状态流转
//! 管道内部基于固定数量（16 个槽位，每槽位 `PIPE_BUF = 4096` 字节，总容量 `PIPE_CAPACITY = 64 KiB`）的环形物理页槽位工作：
//!
//! ```text
//!   写端 (Writer FileRef)                     读端 (Reader FileRef)
//!          │                                         ▲
//!          ▼                                         │
//!   ┌─────────────────────────────────────────────────┐
//!   │          Pipe 环形缓冲区 (16 × 4096B)           │
//!   │  [Slot 0] [Slot 1] ... [Slot 15]                │
//!   │  - 写入 <= PIPE_BUF 时保持记录原子性             │
//!   │  - 槽位内数据未完全读空前，该物理槽位不可重用     │
//!   └─────────────────────────────────────────────────┘
//!     │ 满槽阻塞 / 腾出整槽唤醒          │ 空环阻塞 / 写入提交唤醒
//!     ▼                                  ▼
//!   Write WaitQueue                    Read WaitQueue & PollWaitHead
//! ```
//!
//! 关键生命周期与就绪态转换：
//! - **正常传输**：读端在缓冲区非空时呈现 `POLLIN | POLLRDNORM`，写端在存在空闲物理页槽位时呈现 `POLLOUT | POLLWRNORM`。
//! - **写端全部关闭（最后一个写端 `FileRef` 释放）**：读端在读尽剩余缓冲数据后返回 `Ok(0)`（EOF），且 Poll 状态置位 `POLLHUP`。
//! - **读端全部关闭（最后一个读端 `FileRef` 释放）**：写端非空写入立即返回 `Err(FsError::BrokenPipe)`（对应 `EPIPE`），且 Poll 状态置位 `POLLERR`。
//!
//! # 锁与资源回收契约
//! - 测试夹具 `PipeOwner` 中的 `TaskObserver` 使用外层 `SpinLock<Option<Weak<Task>>>` 记录子任务弱引用，仅在任务启动与主线程轮询状态时短暂加锁，不与管道内部锁嵌套。
//! - `quiesce` 阶段先主动丢弃夹具持有的读写端 `FileRef`，确保所有阻塞在管道上的子任务收到 EOF 或 `BrokenPipe` 并唤醒退出，随后再执行 `join` 回收任务句柄并校验物理内存与任务元数据零泄漏。

use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::fs::file::{FileIoContext, ReadBuffer, UserIoVector, WriteBuffer};
use crate::fs::ktest::{checked, recovered, settled, until};
use crate::fs::pipe::{self, PIPE_BUF, PIPE_CAPACITY};
use crate::fs::poll::{PollContext, PollMask, PollNotify, PollTable};
use crate::fs::{DescriptorFlags, FileRef, FileTable, FsError, InodeKind, SeekFrom};
use crate::mm::space::Mm;
use crate::mm::user_table::PagePermissions;
use crate::sync::SpinLock;
use crate::task::{JoinHandle, State, Task};
use crate::{cpu, mm, sched, task};

use crate::fs::{O_CLOEXEC as CLOEXEC, O_NONBLOCK as NONBLOCK};

/// 用户空间测试映射的起始虚拟地址。
const ADDRESS: usize = 0x4600_0000;
/// 用户空间测试页的可读写、不可执行权限配置。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 工作队列与任务元数据统计快照类型别名。
type Metadata = crate::workqueue::testing::MetadataStats;
/// 子任务弱引用观察槽：保存子任务启动后写入的 `Weak<Task>`，使测试主线程能够查询子任务调度状态而不延长 `Task` 的强引用生命周期。
type TaskObserver = Arc<SpinLock<Option<Weak<Task>>>>;

/// 管道测试套件级资源夹具，用于在整组测试前后记录并校验全局物理内存与任务元数据基线。
struct PipeSuite {
    /// 套件启动前系统稳定后的物理内存统计快照（单位：页/块计数）。
    memory: mm::MemoryStats,
    /// 套件启动前的工作队列与任务元数据统计基线。
    metadata: Metadata,
}
impl Fixture for PipeSuite {
    /// 等待日志、工作队列及各核定时器常驻服务进入阻塞态后，记录套件级内存与任务元数据基线。
    fn set_up(_context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        crate::klog::testing::wait_service_blocked();
        crate::workqueue::testing::wait_services_blocked();
        until("pipe timer services did not block", || {
            (0..cpu::count()).all(|id| {
                crate::timer::testing::service_task(id)
                    .is_some_and(|task| sched::test_state(&task) == State::Blocked)
            })
        })?;
        Ok(Self {
            memory: settled()?,
            metadata: crate::workqueue::testing::metadata_stats(),
        })
    }

    /// 套件内全部用例执行完毕后，校验全局内存统计与任务元数据是否完全恢复至初始基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        recovered(context, "pipe-suite-last-release", self.memory)?;
        let actual = crate::workqueue::testing::metadata_stats();
        context.diagnostic(format_args!(
            "pipe-suite metadata-before={:?} metadata-after={:?}",
            self.metadata, actual
        ));
        context.expect_eq(actual, self.metadata);
        if actual != self.metadata {
            return Err(TestError("pipe suite task metadata leaked"));
        }
        Ok(())
    }
}

/// 单个管道测试用例的运行期上下文与资源持有者。
struct PipeOwner {
    /// 当前用例开始前的物理内存统计基线。
    before: mm::MemoryStats,
    /// 当前用例开始前的任务元数据统计基线。
    metadata: Metadata,
    /// 管道读端打开文件对象（`FileRef`），可在测试中途提前取走关闭以触发 `BrokenPipe`。
    reader: Option<FileRef>,
    /// 管道写端打开文件对象（`FileRef`），可在测试中途提前取走关闭以触发读端 EOF。
    writer: Option<FileRef>,
    /// 当前用例启动的后台内核线程等待句柄列表。
    joins: Vec<JoinHandle>,
    /// 最多 4 个后台测试线程的弱引用观察槽，用于查询线程是否进入调度器的 `Blocked` 状态。
    observers: [Option<TaskObserver>; 4],
}

impl PipeOwner {
    /// 根据指定的打开标志位（如 `O_NONBLOCK`、`O_CLOEXEC`）创建一对匿名管道读写端打开文件对象。
    fn prepare(&mut self, context: &TestContext<'_>, flags: u32) -> TestResult {
        let (reader, writer) = checked(context, pipe::try_new(flags, 0, 0))?;
        self.reader = Some(reader);
        self.writer = Some(writer);
        Ok(())
    }

    /// 获取当前夹具持有的管道读端 `FileRef` 引用。
    fn reader(&self) -> &FileRef {
        self.reader.as_ref().unwrap()
    }

    /// 获取当前夹具持有的管道写端 `FileRef` 引用。
    fn writer(&self) -> &FileRef {
        self.writer.as_ref().unwrap()
    }

    /// 在指定 CPU 核心上创建并启动一个后台内核线程，并在线程入口处将其 `Weak<Task>` 写入 `index` 对应的观察槽。
    fn run(
        &mut self,
        index: usize,
        cpu: usize,
        action: impl FnOnce() -> i32 + Send + 'static,
    ) -> TestResult {
        let observer =
            Arc::try_new(SpinLock::new(None)).map_err(|_| TestError("pipe task observer OOM"))?;
        let observed = observer.clone();
        let prepared = task::prepare_kernel_on(cpu, move || {
            *observed.lock() = Some(Arc::downgrade(&task::current_task()));
            action()
        })
        .map_err(|_| TestError("pipe task preparation OOM"))?;
        self.observers[index] = Some(observer);
        self.joins.push(prepared.publish());
        Ok(())
    }

    /// 查询 `index` 号后台线程在调度器中是否已进入睡眠阻塞状态（`State::Blocked`）。
    fn blocked(&self, index: usize) -> bool {
        let task = self.observers[index]
            .as_ref()
            .unwrap()
            .lock()
            .as_ref()
            .and_then(Weak::upgrade);
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 等待所有已启动的后台线程执行完毕并断言其返回值为 `0`，随后清空全部任务观察槽。
    fn join(&mut self, context: &mut TestContext<'_>) {
        while let Some(join) = self.joins.pop() {
            context.expect_eq(join.join(), 0);
        }
        self.observers.fill_with(|| None);
    }
}

impl Fixture<PipeSuite> for PipeOwner {
    /// 在每个测试用例开始前记录内存与任务元数据基线，并预分配 4 个线程 `JoinHandle` 槽位。
    fn set_up(_context: &mut TestContext<'_>, _suite: &PipeSuite) -> Result<Self, TestError> {
        let before = settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let mut joins = Vec::new();
        joins
            .try_reserve_exact(4)
            .map_err(|_| TestError("pipe fixture join slots OOM"))?;
        Ok(Self {
            before,
            metadata,
            reader: None,
            writer: None,
            joins,
            observers: core::array::from_fn(|_| None),
        })
    }

    /// 先释放夹具持有的管道读写端引用，唤醒可能阻塞的子线程使其收到 EOF 或 `BrokenPipe`，再等待所有子线程退出。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        drop(self.reader.take());
        drop(self.writer.take());
        self.join(context);
        Ok(())
    }

    /// 销毁当前用例夹具持有的全部对象后，校验物理内存页与任务元数据是否完全回收至用例启动前基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.before;
        let metadata = self.metadata;
        drop(self);
        recovered(context, "pipe-case-last-release", before)?;
        let actual = crate::workqueue::testing::metadata_stats();
        context.diagnostic(format_args!(
            "pipe-case metadata-before={:?} metadata-after={:?}",
            metadata, actual
        ));
        context.expect_eq(actual, metadata);
        if actual != metadata {
            return Err(TestError("pipe case task metadata leaked"));
        }
        Ok(())
    }
}

/// 调用文件对象的 `Pollable::poll(None)` 接口，查询当前时刻的就绪事件掩码。
fn mask(context: &TestContext<'_>, file: &FileRef) -> Result<PollMask, TestError> {
    checked(context, file.pollable().unwrap().poll(None))
}

/// 根据字节流偏移量 `offset` 生成确定性的校验字节，用于验证大块或跨页传输的数据完整性。
fn byte(offset: usize) -> u8 {
    (offset as u8)
        .wrapping_mul(37)
        .wrapping_add((offset / 251) as u8)
}

/// 返回本模块测试套件的静态注册槽位供父模块挂载。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    streams::__ktest_anchor()
}

#[ktest::suite(fixture = PipeOwner, suite_resources = PipeSuite, context = "Thread", tags("fs-pipe"), timeout_ms = 45000)]
pub(crate) mod streams {
    use super::*;

    /// 测试目标：验证超过管道总容量的流式传输完整性，以及写端关闭后读端先读尽残留尾部数据再返回 EOF 与 `POLLHUP` 的状态转换。
    /// 场景步骤：
    /// 1. 启动后台写线程写入超过 2 倍 `PIPE_CAPACITY` 的字节流，主线程循环读取并逐字节校验。
    /// 2. 写线程退出并释放最后一个写端引用后，验证读端返回 0（EOF）且 Poll 状态变为 `POLLHUP`。
    /// 3. 重新创建管道，写入 `"tail"` 后立即关闭写端，验证此时 Poll 同时报告 `POLLIN | POLLRDNORM | POLLHUP`，且分段读取完 `"tail"` 后返回 EOF。
    #[test]
    #[ktest(tags("T05"))]
    fn complete_transfer_tail_eof_and_in_hup(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
        // Step 1: 准备默认阻塞管道，启动后台写线程持续写入超过两倍容量的确定性字节流。
        fixture.prepare(context, 0)?;
        context.expect_eq(mask(context, fixture.reader())?, PollMask::default());
        const LENGTH: usize = PIPE_CAPACITY * 2 + 73;
        let writer = fixture.writer().clone();
        drop(fixture.writer.take());
        fixture.run(0, cpu::current().id, move || {
            let mut offset = 0;
            let mut bytes = [0; 997];
            while offset < LENGTH {
                let count = bytes.len().min(LENGTH - offset);
                for (index, value) in bytes[..count].iter_mut().enumerate() {
                    *value = byte(offset + index);
                }
                match writer.write(&bytes[..count]) {
                    Ok(done) if done == count => offset += done,
                    _ => return 1,
                }
            }
            0
        })?;

        // Step 2: 主线程以不同步长循环读取并逐字节比对，随后等待写线程退出并验证 EOF 与 POLLHUP。
        let mut offset = 0;
        let mut bytes = [0xa5; 733];
        while offset < LENGTH {
            let count = checked(context, fixture.reader().read(&mut bytes))?;
            if count == 0 {
                return Err(TestError("pipe lost stream before EOF"));
            }
            for (index, value) in bytes[..count].iter().enumerate() {
                context.expect_eq(*value, byte(offset + index));
            }
            offset += count;
        }
        fixture.join(context);
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 0);
        context.expect_eq(mask(context, fixture.reader())?, PollMask::POLLHUP);
        drop(fixture.reader.take());

        // Step 3: 验证写端已关闭但缓冲区尚有残留数据时，读端可分次读出尾部字节，读空后才返回 EOF。
        fixture.prepare(context, 0)?;
        checked(context, fixture.writer().write(b"tail"))?;
        drop(fixture.writer.take());
        context.expect_eq(
            mask(context, fixture.reader())?,
            PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLHUP,
        );
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes[..2]))?, 2);
        context.expect_eq(&bytes[..2], b"ta");
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 2);
        context.expect_eq(&bytes[..2], b"il");
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 0);
        Ok(())
    }

    /// 测试目标：验证多生产者并发执行小于 `PIPE_BUF` 的写入时，即使跨容量发生阻塞与重调度，每条记录依然保持原子且不交错。
    /// 场景步骤：
    /// 1. 启动 3 个并发写线程，每个线程循环写入 128 条长度为 257 字节（含生产者编号与递增序号头部）的记录。
    /// 2. 主线程以 113 字节的不对齐窗口循环读取，按 257 字节定长重组记录，校验每条记录归属同一生产者且序号严格递增。
    #[test]
    #[ktest(tags("T06"), min_cpus = 2)]
    fn concurrent_small_writes_keep_whole_records(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
        // Step 1: 启动 3 个并发生产者线程，各自向同一管道写入带头部序号的定长记录。
        fixture.prepare(context, 0)?;
        const RECORD: usize = 257;
        const COUNT: usize = 128;
        for producer in 0..3 {
            let writer = fixture.writer().clone();
            fixture.run(producer, producer % cpu::count(), move || {
                let mut bytes = [0; RECORD];
                for sequence in 0..COUNT {
                    bytes.fill(producer as u8 + 0x50);
                    bytes[0] = producer as u8;
                    bytes[1] = sequence as u8;
                    if writer.write(&bytes) != Ok(RECORD) {
                        return 1;
                    }
                    sched::yield_now();
                }
                0
            })?;
        }
        drop(fixture.writer.take());

        // Step 2: 主线程以不对齐缓冲读取并重组定长记录，验证跨线程记录未发生字节交错且总数完整。
        let mut sequences = [0usize; 3];
        let mut record = [0; RECORD];
        let mut offset = 0;
        let mut chunk = [0; 113];
        let mut records = 0;
        loop {
            let count = checked(context, fixture.reader().read(&mut chunk))?;
            if count == 0 {
                break;
            }
            for value in &chunk[..count] {
                record[offset] = *value;
                offset += 1;
                if offset == RECORD {
                    let producer = usize::from(record[0]);
                    if 3 <= producer {
                        return Err(TestError("pipe record interleaved producer header"));
                    }
                    context.expect_eq(record[1] as usize, sequences[producer]);
                    context.expect_eq(
                        record[2..]
                            .iter()
                            .all(|byte| *byte == producer as u8 + 0x50),
                        true,
                    );
                    sequences[producer] += 1;
                    records += 1;
                    offset = 0;
                }
            }
        }
        fixture.join(context);
        context.expect_eq((records, offset, sequences), (3 * COUNT, 0, [COUNT; 3]));
        Ok(())
    }

    /// 测试目标：验证非阻塞管道（`O_NONBLOCK`）的物理页槽位复用规则、零长度读写行为、超出 `PIPE_BUF` 的部分写入（Partial Progress）以及读端关闭后的 `BrokenPipe` 行为。
    /// 场景步骤：
    /// 1. 创建 `O_NONBLOCK` 管道，验证空环读取返回 `WouldBlock`、零长度读取返回 `0`。
    /// 2. 写满全部 16 个 `PIPE_BUF` 槽位，验证仅读出 1 字节但未腾出整槽时写入仍返回 `WouldBlock`；读完该槽剩余字节后槽位释放，写端恢复可写。
    /// 3. 验证写入 `PIPE_BUF + 1` 字节时仅写入前 `PIPE_BUF` 字节并返回成功字节数；验证槽位内尾随空闲空间只能容纳不超过剩余空间的写入。
    /// 4. 关闭读端后，验证零长度写入返回 `0`、非空写入返回 `BrokenPipe` 且 Poll 报告 `POLLERR`。
    #[test]
    #[ktest(tags("T07"))]
    fn nonblock_physical_slots_zero_and_short_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
        // Step 1: 写满 16 个物理页槽位，验证槽位未完全读空前不可重用，读空整槽后才恢复 POLLOUT。
        fixture.prepare(context, NONBLOCK | CLOEXEC)?;
        let page = [0x8d; PIPE_BUF];
        let mut bytes = [0; PIPE_BUF];
        context.expect_eq(fixture.reader().read(&mut bytes), Err(FsError::WouldBlock));
        context.expect_eq(fixture.reader().read(&mut []), Ok(0));
        for _ in 0..16 {
            context.expect_eq(fixture.writer().write(&page), Ok(PIPE_BUF));
        }
        context.expect_eq(mask(context, fixture.writer())?, PollMask::default());
        context.expect_eq(fixture.writer().write(b"x"), Err(FsError::WouldBlock));
        context.expect_eq(fixture.reader().read(&mut bytes[..1]), Ok(1));
        context.expect_eq(fixture.writer().write(b"x"), Err(FsError::WouldBlock));
        context.expect_eq(
            fixture.reader().read(&mut bytes[..PIPE_BUF - 1]),
            Ok(PIPE_BUF - 1),
        );
        context.expect_eq(
            mask(context, fixture.writer())?,
            PollMask::POLLOUT | PollMask::POLLWRNORM,
        );

        // Step 2: 验证大于 PIPE_BUF 的非阻塞写入截断为可用槽位容量，以及单槽尾部剩余空间的填充边界。
        let large = [0x37; PIPE_BUF + 1];
        context.expect_eq(fixture.writer().write(&large), Ok(PIPE_BUF));
        context.expect_eq(fixture.writer().write(&[]), Ok(0));
        for _ in 0..15 {
            context.expect_eq(fixture.reader().read(&mut bytes), Ok(PIPE_BUF));
            context.expect_eq(bytes, page);
        }
        context.expect_eq(fixture.reader().read(&mut bytes), Ok(PIPE_BUF));
        context.expect_eq(bytes, [0x37; PIPE_BUF]);

        for _ in 0..15 {
            context.expect_eq(fixture.writer().write(&page), Ok(PIPE_BUF));
        }
        context.expect_eq(
            fixture.writer().write(&page[..PIPE_BUF - 7]),
            Ok(PIPE_BUF - 7),
        );
        context.expect_eq(mask(context, fixture.writer())?, PollMask::default());
        context.expect_eq(
            fixture.writer().write(b"too long"),
            Err(FsError::WouldBlock),
        );
        context.expect_eq(fixture.writer().write(b"fits007"), Ok(7));
        for _ in 0..15 {
            context.expect_eq(fixture.reader().read(&mut bytes), Ok(PIPE_BUF));
        }
        context.expect_eq(fixture.reader().read(&mut bytes), Ok(PIPE_BUF));
        context.expect_eq(&bytes[..PIPE_BUF - 7], &page[..PIPE_BUF - 7]);
        context.expect_eq(&bytes[PIPE_BUF - 7..], b"fits007");
        context.expect_eq(fixture.writer().flags() & CLOEXEC, 0);

        // Step 3: 关闭读端，验证非阻塞写端对空写入与非空写入的错误码及 POLLERR 掩码。
        drop(fixture.reader.take());
        context.expect_eq(fixture.writer().write(&[]), Ok(0));
        context.expect_eq(fixture.writer().write(b"x"), Err(FsError::BrokenPipe));
        context.expect_eq(
            mask(context, fixture.writer())?,
            PollMask::POLLOUT | PollMask::POLLWRNORM | PollMask::POLLERR,
        );
        Ok(())
    }

    /// 测试目标：验证阻塞模式管道的读写睡眠与唤醒条件，以及 `PollTable` 注册、事件触发与销毁解绑的完整生命周期。
    /// 场景步骤：
    /// 1. 启动读线程在空管道上阻塞，待调度器确认其进入 `Blocked` 后由主线程写入数据唤醒并校验。
    /// 2. 写满 16 个槽位后启动写线程使其阻塞，验证读出 1 字节不会使写线程完成写入，直到读完整个槽位（`PIPE_BUF`）才唤醒写线程完成写入。
    /// 3. 通过 `PollTable` 将 `PollContext` 注册到读端，验证写入数据能触发通知并唤醒 `park_until`；丢弃 `PollTable` 解绑后再次写入不再触发旧通知。
    #[test]
    #[ktest(tags("T07"))]
    fn blocking_read_write_and_persistent_notification(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
        // Step 1: 验证读线程在空管道上进入调度器 Blocked 状态，并在主线程写入后被唤醒读出数据。
        fixture.prepare(context, 0)?;
        let reader = fixture.reader().clone();
        fixture.run(0, cpu::current().id, move || {
            let mut bytes = [0; 5];
            if reader.read(&mut bytes) == Ok(5) && &bytes == b"awake" {
                0
            } else {
                1
            }
        })?;
        until("pipe reader did not really block", || fixture.blocked(0))?;
        checked(context, fixture.writer().write(b"awake"))?;
        fixture.join(context);

        // Step 2: 写满全部槽位使写线程阻塞，验证必须腾出完整物理页槽位才能使写线程完成写入。
        for _ in 0..16 {
            checked(context, fixture.writer().write(&[0x42; PIPE_BUF]))?;
        }
        let writer = fixture.writer().clone();
        fixture.run(0, cpu::current().id, move || {
            if writer.write(b"resume") == Ok(6) {
                0
            } else {
                1
            }
        })?;
        until("pipe writer did not really block", || fixture.blocked(0))?;
        context.expect_eq(checked(context, fixture.reader().read(&mut [0; 1]))?, 1);
        until(
            "pipe writer did not recheck unavailable physical slot",
            || fixture.blocked(0),
        )?;
        let mut bytes = [0; PIPE_BUF];
        context.expect_eq(
            checked(context, fixture.reader().read(&mut bytes[..PIPE_BUF - 1]))?,
            PIPE_BUF - 1,
        );
        fixture.join(context);
        for _ in 0..15 {
            checked(context, fixture.reader().read(&mut bytes))?;
        }
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 6);
        context.expect_eq(&bytes[..6], b"resume");

        // Step 3: 验证 PollTable 注册后写端提交会触发 PollContext 通知，且销毁 PollTable 后自动解绑。
        let poll = checked(context, PollContext::try_new())?;
        let sink: Arc<dyn PollNotify> = poll.clone();
        let mut table = checked(context, PollTable::new(sink, 1))?;
        checked(
            context,
            fixture.reader().pollable().unwrap().poll(Some(&mut table)),
        )?;
        poll.clear_triggered();
        checked(context, fixture.writer().write(b"fact"))?;
        context.expect_eq(poll.triggered(), true);
        context.expect_eq(poll.park_until(false, None), crate::wait::WaitResult::Woken);
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 4);
        drop(table);
        poll.clear_triggered();
        checked(context, fixture.writer().write(b"unregistered"))?;
        context.expect_eq(poll.triggered(), false);
        Ok(())
    }

    /// 测试目标：验证 `dup` 与 `fork`（`FileTable::try_clone`）共享同一个底层打开文件对象（OFD），且仅当所有文件描述符与外部 `FileRef` 强引用全部释放后才触发 EOF 或 `BrokenPipe`。
    /// 场景步骤：
    /// 1. 在父文件表中注册读写端并执行 `dup` 与 `try_clone`，验证 `O_CLOEXEC` 是描述符级属性，而状态标志（`O_NONBLOCK`）和 `stat` 属于共享的 OFD。
    /// 2. 依次关闭父子文件表中的所有写端描述符，验证只要仍有一个通过 `lookup` 持有的 `FileRef` 未释放，读端就不会出现 `POLLHUP`；释放该引用后立即出现 `POLLHUP`。
    /// 3. 同理验证读端在所有描述符及外部 `FileRef` 全部释放前写端可正常写入，最后一个读端引用释放后写端返回 `BrokenPipe` 与 `POLLERR`。
    #[test]
    #[ktest(tags("T08"))]
    fn dup_fork_table_last_ofd_and_anonymous_identity(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
        // Step 1: 构建父子文件表与 dup 描述符，验证共享 OFD 状态标志、FD 级 close_on_exec 隔离以及 FIFO 元数据。
        fixture.prepare(context, NONBLOCK)?;
        let parent = checked(context, FileTable::try_new())?;
        let r = checked(context, parent.reserve(1024))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        let w = checked(context, parent.reserve(1024))?.publish(
            fixture.writer().clone(),
            DescriptorFlags {
                close_on_exec: true,
            },
        );
        let duplicate = checked(context, parent.dup(w, 1024))?;
        let child = checked(context, parent.try_clone())?;
        let held = checked(context, child.lookup(w))?;
        context.expect_eq(Arc::ptr_eq(&held, fixture.writer()), true);
        checked(context, held.set_status_flags(0))?;
        context.expect_eq(fixture.writer().flags(), 1);
        context.expect_eq(checked(context, child.flags(w))?.close_on_exec, true);
        context.expect_eq(
            checked(context, parent.flags(duplicate))?.close_on_exec,
            false,
        );

        let stat = checked(context, fixture.reader().stat())?;
        context.expect_eq(stat, checked(context, fixture.writer().stat())?);
        context.expect_eq(
            (stat.kind, stat.mode, stat.device, stat.rdev),
            (InodeKind::Fifo, 0o010600, 0, 0),
        );
        context.expect_eq(
            fixture.reader().seek(SeekFrom::Start(0)),
            Err(FsError::IllegalSeek),
        );
        context.expect_eq(fixture.writer().write_at(0, &[]), Err(FsError::IllegalSeek));
        context.expect_eq(
            fixture.reader().read_at(0, &mut []),
            Err(FsError::IllegalSeek),
        );
        context.expect_eq(
            fixture.reader().readdir().err(),
            Some(FsError::NotDirectory),
        );
        context.expect_eq(
            fixture.reader().ioctl(0xffff, 0, FileIoContext::Kernel),
            Err(FsError::NotTty),
        );

        // Step 2: 关闭文件表内全部写端描述符，验证仅当最后一个外部写端 FileRef (held) 释放时读端才收到 POLLHUP。
        checked(context, fixture.writer().write(b"shared"))?;
        drop(fixture.writer.take());
        checked(context, parent.close(w))?;
        checked(context, parent.close(duplicate))?;
        checked(context, child.close(w))?;
        checked(context, child.close(duplicate))?;
        context.expect_eq(
            mask(context, fixture.reader())?.contains(PollMask::POLLHUP),
            false,
        );
        drop(held);
        context.expect_eq(
            mask(context, fixture.reader())?,
            PollMask::POLLIN | PollMask::POLLRDNORM | PollMask::POLLHUP,
        );
        let mut bytes = [0; 8];
        context.expect_eq(checked(context, parent.read(r, &mut bytes))?, 6);
        context.expect_eq(&bytes[..6], b"shared");
        context.expect_eq(checked(context, child.read(r, &mut bytes))?, 0);
        drop(parent);
        drop(child);
        drop(fixture.reader.take());

        // Step 3: 重新创建管道验证匿名 inode 唯一性，并验证最后一个读端 FileRef (held_reader) 释放时写端才转为 BrokenPipe。
        fixture.prepare(context, NONBLOCK)?;
        context.expect_eq(
            checked(context, fixture.reader().stat())?.inode != stat.inode,
            true,
        );
        let parent = checked(context, FileTable::try_new())?;
        let r = checked(context, parent.reserve(1024))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        let duplicate = checked(context, parent.dup(r, 1024))?;
        let child = checked(context, parent.try_clone())?;
        let held_reader = checked(context, child.lookup(r))?;
        drop(fixture.reader.take());
        checked(context, parent.close(r))?;
        checked(context, parent.close(duplicate))?;
        checked(context, child.close(r))?;
        checked(context, child.close(duplicate))?;
        context.expect_eq(fixture.writer().write(b"live lookup"), Ok(11));
        context.expect_eq(
            mask(context, fixture.writer())?.contains(PollMask::POLLERR),
            false,
        );
        drop(held_reader);
        context.expect_eq(fixture.writer().write(b"closed"), Err(FsError::BrokenPipe));
        context.expect_eq(
            mask(context, fixture.writer())?.contains(PollMask::POLLERR),
            true,
        );
        // 尚未被读取的环形缓冲数据继续驻留在管道物理页槽位中，待最后一个写端 FileRef 释放时随物理页一并归还分配器。
        drop(parent);
        drop(child);
        Ok(())
    }

    /// 测试目标：验证管道读写通过用户地址空间（`Mm`）传输时，散列向量（`UserIoVector`）合并为单次请求，且跨页遇到未映射地址（`EFAULT` / `BadAddress`）时保留已成功传输的前缀数据。
    /// 场景步骤：
    /// 1. 映射单页用户内存，构造包含零长无效地址段与两段有效地址的 `UserIoVector`，验证 `write_from` 一次性写入全部 23 字节。
    /// 2. 构造跨越已映射页尾进入未映射区域的 `ReadBuffer::user` 与 `WriteBuffer::user`，验证在页边界前已成功拷贝的前缀字节数被正常提交和消费。
    /// 3. 验证起始地址即位于未映射区域时直接返回 `Err(FsError::BadAddress)`，且不破坏管道内已有未读数据。
    #[test]
    #[ktest(tags("T05", "T06"))]
    fn user_fault_prefix_and_one_vector_request(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
        // Step 1: 映射单页用户地址空间，验证包含零长度段的 UserIoVector 向量可一次性写入管道。
        fixture.prepare(context, NONBLOCK)?;
        let memory = Mm::try_new().map_err(|_| TestError("pipe MM OOM"))?;
        memory
            .map_anonymous(ADDRESS, PIPE_BUF, RW, RW)
            .map_err(|_| TestError("pipe anonymous mapping failed"))?;
        let input = [0x61; 23];
        context.expect_eq(memory.copyout(ADDRESS + PIPE_BUF - 23, &input).copied, 23);
        let vectors = [
            UserIoVector {
                address: usize::MAX,
                length: 0,
            },
            UserIoVector {
                address: ADDRESS + PIPE_BUF - 23,
                length: 7,
            },
            UserIoVector {
                address: ADDRESS + PIPE_BUF - 16,
                length: 16,
            },
        ];
        let mut source = checked(context, WriteBuffer::user_vector(&memory, &vectors))?;
        context.expect_eq(
            checked(
                context,
                fixture
                    .writer()
                    .write_from(&mut source, FileIoContext::Kernel),
            )?,
            23,
        );

        // Step 2: 验证跨页边界的用户态读写缓冲区仅消费已映射前缀字节，剩余部分留存或截断。
        let mut target = checked(
            context,
            ReadBuffer::user(&memory, ADDRESS + PIPE_BUF - 11, 23),
        )?;
        context.expect_eq(
            checked(
                context,
                fixture
                    .reader()
                    .read_into(&mut target, FileIoContext::Kernel),
            )?,
            11,
        );
        context.expect_eq(target.copied(), 11);
        let mut bytes = [0; 32];
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 12);
        context.expect_eq(&bytes[..12], &[0x61; 12]);

        let mut source = checked(
            context,
            WriteBuffer::user(&memory, ADDRESS + PIPE_BUF - 9, 20),
        )?;
        context.expect_eq(
            checked(
                context,
                fixture
                    .writer()
                    .write_from(&mut source, FileIoContext::Kernel),
            )?,
            9,
        );
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 9);
        context.expect_eq(&bytes[..9], &[0x61; 9]);

        // Step 3: 验证首字节即缺页失败时返回 BadAddress，且不会丢失管道中已有的未读数据。
        let mut source = checked(context, WriteBuffer::user(&memory, ADDRESS + PIPE_BUF, 1))?;
        context.expect_eq(
            fixture
                .writer()
                .write_from(&mut source, FileIoContext::Kernel),
            Err(FsError::BadAddress),
        );

        checked(context, fixture.writer().write(b"retained"))?;
        let mut target = checked(context, ReadBuffer::user(&memory, ADDRESS + PIPE_BUF, 8))?;
        context.expect_eq(
            fixture
                .reader()
                .read_into(&mut target, FileIoContext::Kernel),
            Err(FsError::BadAddress),
        );
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 8);
        context.expect_eq(&bytes[..8], b"retained");
        Ok(())
    }

    /// 测试目标：验证当全局物理页分配器耗尽（OOM）时，向管道申请新槽位返回 `Err(FsError::NoMemory)` 但不破坏已有槽位数据；释放内存压力后管道可恢复正常分配与读写。
    /// 场景步骤：
    /// 1. 先向管道写入一整页（`PIPE_BUF`）数据占满首个槽位。
    /// 2. 耗尽全局空闲物理页后尝试继续写入，验证因无法分配新槽位返回 `NoMemory`，而首个槽位的数据仍可完整读出。
    /// 3. 释放占用的物理页后，验证同一管道写端可重新分配物理页并完成正常读写。
    #[test]
    #[ktest(tags("T07", "T08"))]
    fn real_page_oom_preserves_existing_data_and_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
        // Step 1: 预先写入一页数据，随后抽干全局物理页分配器。
        fixture.prepare(context, NONBLOCK)?;
        checked(context, fixture.writer().write(&[0x79; PIPE_BUF]))?;
        let mut pressure = Vec::new();
        pressure
            .try_reserve_exact(mm::stats().present_pages)
            .map_err(|_| TestError("pipe page pressure slots OOM"))?;

        mm::shrink_all();
        mm::drain_all_pages();
        for order in [10, 0] {
            while let Some(page) = mm::alloc_pages(order, false) {
                pressure.push(page);
            }
        }
        mm::drain_all_pages();
        context.expect_eq(mm::alloc_pages(0, false).is_none(), true);
        // Step 2: 验证 OOM 下扩展新槽位返回 NoMemory 且旧数据完好，释放内存压力后管道恢复正常写入。
        context.expect_eq(fixture.writer().write(b"new page"), Err(FsError::NoMemory));
        let mut bytes = [0; PIPE_BUF];
        context.expect_eq(fixture.reader().read(&mut bytes), Ok(PIPE_BUF));
        context.expect_eq(bytes, [0x79; PIPE_BUF]);

        drop(pressure);
        context.expect_eq(fixture.writer().write(b"after OOM"), Ok(9));
        context.expect_eq(fixture.reader().read(&mut bytes), Ok(9));
        context.expect_eq(&bytes[..9], b"after OOM");
        Ok(())
    }
}
