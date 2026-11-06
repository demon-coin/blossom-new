//! `epoll` 事件多路复用子系统的内核集成与并发回归测试。
//!
//! ## 模块职责与架构定位
//! 本模块位于文件系统内核测试层（`fs::ktest::epoll`），作为 `EpollFile` 的直接集成测试入口，
//! 向上承接 `ktest` 测试运行器，向下驱动 `fs::epoll`（控制与扫描）、`fs::pipe`（匿名管道事件源）、
//! `fs::poll`（等待表与唤醒通知）、`fs::FileTable`（文件描述符表）以及 `mm::space::Mm`（用户地址空间拷贝）。
//!
//! ## 核心测试机制与生命周期模型
//! 测试围绕 `(OpenFileDescription, fd)` 复合兴趣键（Interest Key）、触发模式状态机与引用计数展开：
//!
//! ```text
//!   [Pipe Writer] --write()--> [Pipe Reader (OFD)]
//!                                    |
//!                       PollNotify   v
//!   [FileTable (fd)] --------> [Epoll Interest (OFD, fd)]
//!                                    |
//!                    ready queue     v
//!                              [EpollFile::scan()] ---> [User Mm / Callback]
//! ```
//!
//! - **触发模式转换**：覆盖水平触发（LT，缓冲区有数据即重入就绪队列）、边缘触发（ET，仅在新通知到达或溢出回退时入队）
//!   与单次触发（`EPOLLONESHOT`，完整交付一项后进入禁用态，需 `EPOLL_CTL_MOD` 重新激活）。
//! - **延迟释放（Deferred Drop）**：当 `scan` 回调内部关闭最后一个指向目标文件的描述符时，扫描帧持有的临时强引用
//!   保证目标文件在 `EpollFile` 控制锁释放后才执行析构与反向注销，避免锁重入死锁。
//!
//! ## 锁获取顺序与并发契约
//! - 测试中的 `TaskObserver` 自旋锁（`SpinLock`）仅在子任务启动和主任务轮询睡眠状态时短暂获取，持锁期间不睡眠、不嵌套其他锁。
//! - 验证 `EpollFile::scan` 的睡眠互斥锁（`SleepMutex`）与底层管道自旋锁、描述符表锁之间的解耦：
//!   回调执行与用户页拷贝（`Mm::copyout`）期间不持有底层管道或描述符表锁。

use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::fs::epoll::{
    EpollEvent, EpollFile, EPOLLET, EPOLLONESHOT, EPOLL_CTL_ADD, EPOLL_CTL_DEL, EPOLL_CTL_MOD,
};
use crate::fs::ktest::{checked, recovered, settled, until};
use crate::fs::pipe::{self, PIPE_BUF};
use crate::fs::poll::{PollContext, PollMask, PollNotify, PollTable};
use crate::fs::{DescriptorFlags, FileIoContext, FileRef, FileTable, FsError, InodeKind, SeekFrom};
use crate::mm::space::Mm;
use crate::mm::user_table::PagePermissions;
use crate::sync::{Completion, SpinLock};
use crate::task::{JoinHandle, State, Task};
use crate::wait::WaitResult;
use crate::{cpu, mm, sched, task};

use crate::fs::{O_CLOEXEC as CLOEXEC, O_NONBLOCK as NONBLOCK};

/// 用户空间测试映射的起始虚拟地址（字节单位，页对齐）。
const ADDRESS: usize = 0x4700_0000;
/// 用户空间测试页的读写权限配置（可读、可写、不可执行）。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 工作队列与任务元数据的统计快照类型别名。
type Metadata = crate::workqueue::testing::MetadataStats;
/// 指向 `epoll` 内部监听节点生命周期守卫的弱引用类型别名。
type WatchWeak = Weak<dyn Send + Sync>;
/// 保存子任务自身发布的 `Task` 弱引用，用于在主线程观测子任务调度状态而不延长其生命周期。
type TaskObserver = Arc<SpinLock<Option<Weak<Task>>>>;

/// `epoll` 测试套件级夹具，负责在整组测试前后校验全局物理内存与任务元数据无泄漏。
struct EpollSuite {
    /// 套件启动前等待后台服务静止后记录的物理页与内核堆内存基线快照。
    memory: mm::MemoryStats,
    /// 套件启动前记录的任务、工作队列 Worker 与 Domain 计数基线快照。
    metadata: Metadata,
}

impl Fixture for EpollSuite {
    /// 等待日志、工作队列及各核定时器常驻服务进入阻塞态后，记录套件级内存与任务元数据基线。
    fn set_up(_context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        crate::klog::testing::wait_service_blocked();
        crate::workqueue::testing::wait_services_blocked();
        until("epoll timer services did not block", || {
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

    /// 套件内所有用例执行完毕且资源释放后，校验内存统计与任务元数据计数完全回到套件基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        recovered(context, "epoll-suite-last-release", self.memory)?;
        let actual = crate::workqueue::testing::metadata_stats();
        context.diagnostic(format_args!(
            "epoll-suite metadata-before={:?} metadata-after={:?}",
            self.metadata, actual
        ));
        context.expect_eq(actual, self.metadata);
        if actual != self.metadata {
            return Err(TestError("epoll suite task metadata leaked"));
        }
        Ok(())
    }
}

/// 单个 `epoll` 测试用例的资源拥有者夹具，统一管理测试管道、事件实例、并发子任务与同步门闸。
struct EpollOwner {
    /// 当前用例启动前的内存统计基线快照。
    before: mm::MemoryStats,
    /// 当前用例启动前的任务与工作队列元数据基线快照。
    metadata: Metadata,
    /// 被测 `EpollFile` 对应的打开文件对象（`FileRef`）。
    owner: Option<FileRef>,
    /// 默认测试匿名管道的读端打开文件对象。
    reader: Option<FileRef>,
    /// 默认测试匿名管道的写端打开文件对象。
    writer: Option<FileRef>,
    /// 本例派生的并发内核子任务等待句柄列表。
    joins: Vec<JoinHandle>,
    /// 最多 4 个并发子任务的弱引用槽位，由各槽内部自旋锁保护，用于查询调度器阻塞状态。
    observers: [Option<TaskObserver>; 4],
    /// 跨线程同步门闸（`Completion`）列表，在清理阶段统一放行以防子任务永久卡死。
    gates: Vec<Arc<Completion>>,
    /// 通知后台持续生产子任务退出的原子停止标志。
    stop: Arc<AtomicBool>,
}

impl EpollOwner {
    /// 创建一对非阻塞匿名管道（读端与写端）和一个 `EpollFile` 实例，并存入夹具字段。
    fn prepare(&mut self, context: &TestContext<'_>) -> TestResult {
        let (reader, writer) = checked(context, pipe::try_new(NONBLOCK, 0, 0))?;
        let owner: FileRef = checked(context, EpollFile::try_new(0))?;
        self.reader = Some(reader);
        self.writer = Some(writer);
        self.owner = Some(owner);
        Ok(())
    }

    /// 获取被测 `EpollFile` 的 `FileRef` 引用。
    fn owner(&self) -> &FileRef {
        self.owner.as_ref().unwrap()
    }
    /// 从被测文件引用中提取具体的 `EpollFile` 引用，供直接调用 `control` 和 `scan`。
    fn epoll(&self) -> &EpollFile {
        self.owner().epoll().unwrap()
    }
    /// 获取默认测试管道读端的 `FileRef` 引用。
    fn reader(&self) -> &FileRef {
        self.reader.as_ref().unwrap()
    }
    /// 获取默认测试管道写端的 `FileRef` 引用。
    fn writer(&self) -> &FileRef {
        self.writer.as_ref().unwrap()
    }

    /// 从预分配槽位中创建一个同步门闸（`Completion`），由夹具统一持有以便异常清理时放行等待任务。
    fn gate(&mut self) -> Result<Arc<Completion>, TestError> {
        assert!(self.gates.len() < self.gates.capacity());
        let gate =
            Arc::try_new(Completion::new()).map_err(|_| TestError("epoll completion OOM"))?;
        self.gates.push(Arc::clone(&gate));
        Ok(gate)
    }

    /// 在指定 CPU `cpu` 上创建并启动第 `index` 号内核测试子任务，记录其弱引用观察槽与 `JoinHandle`。
    fn run(
        &mut self,
        index: usize,
        cpu: usize,
        action: impl FnOnce() -> i32 + Send + 'static,
    ) -> TestResult {
        let observer =
            Arc::try_new(SpinLock::new(None)).map_err(|_| TestError("epoll task observer OOM"))?;
        let observed = Arc::clone(&observer);
        let prepared = task::prepare_kernel_on(cpu, move || {
            *observed.lock() = Some(Arc::downgrade(&task::current_task()));
            action()
        })
        .map_err(|_| TestError("epoll task preparation OOM"))?;
        assert!(self.joins.len() < self.joins.capacity());
        self.observers[index] = Some(observer);
        self.joins.push(prepared.publish());
        Ok(())
    }

    /// 检查第 `index` 号子任务在调度器中是否已进入 `State::Blocked` 阻塞等待状态。
    fn blocked(&self, index: usize) -> bool {
        let task = self.observers[index]
            .as_ref()
            .unwrap()
            .lock()
            .as_ref()
            .and_then(Weak::upgrade);
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 逐个等待所有已派生的子任务退出并断言返回值为 0，随后清空全部任务弱引用观察槽。
    fn join(&mut self, context: &mut TestContext<'_>) {
        while let Some(join) = self.joins.pop() {
            context.expect_eq(join.join(), 0);
        }
        self.observers.fill_with(|| None);
    }
}

impl Fixture<EpollSuite> for EpollOwner {
    /// 用例初始化：采集内存与任务元数据基线，并预分配 `joins`、`gates` 容器容量与 `stop` 原子标志。
    fn set_up(_context: &mut TestContext<'_>, _suite: &EpollSuite) -> Result<Self, TestError> {
        let before = settled()?;
        let metadata = crate::workqueue::testing::metadata_stats();
        let mut joins = Vec::new();
        joins
            .try_reserve_exact(4)
            .map_err(|_| TestError("epoll join slots OOM"))?;
        let mut gates = Vec::new();
        gates
            .try_reserve_exact(8)
            .map_err(|_| TestError("epoll gate slots OOM"))?;
        let stop =
            Arc::try_new(AtomicBool::new(false)).map_err(|_| TestError("epoll stop flag OOM"))?;
        Ok(Self {
            before,
            metadata,
            owner: None,
            reader: None,
            writer: None,
            joins,
            observers: core::array::from_fn(|_| None),
            gates,
            stop,
        })
    }

    /// 用例静止化：置位停止标志、唤醒所有同步门闸、释放 `epoll` 与管道文件引用，并等待全部子任务退出。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.stop.store(true, Ordering::Release);
        for gate in &self.gates {
            gate.complete();
        }
        drop(self.owner.take());

        drop(self.reader.take());
        drop(self.writer.take());
        self.join(context);
        Ok(())
    }

    /// 用例拆解：释放夹具自身持有的容器与同步对象后，校验物理内存与任务元数据完全恢复到用例前基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.before;
        let metadata = self.metadata;
        drop(self);

        recovered(context, "epoll-case-last-release", before)?;
        let actual = crate::workqueue::testing::metadata_stats();
        context.diagnostic(format_args!(
            "epoll-case metadata-before={:?} metadata-after={:?}",
            metadata, actual
        ));
        context.expect_eq(actual, metadata);
        if actual != metadata {
            return Err(TestError("epoll case task metadata leaked"));
        }
        Ok(())
    }
}

/// 构造一个包含指定事件掩码 `events` 与 64 位用户数据 `data` 的 `EpollEvent`。
fn event(events: u32, data: u64) -> EpollEvent {
    EpollEvent { events, data }
}

/// 调用 `EpollFile::scan` 扫描最多 `output.len()` 个就绪事件并顺序写入 `output` 切片，返回实际交付数量。
fn collect(file: &EpollFile, output: &mut [EpollEvent]) -> Result<usize, FsError> {
    let mut count = 0;
    file.scan(output.len(), &mut |event| {
        output[count] = event;
        count += 1;
        Ok(())
    })
}

/// 将 `EpollEvent` 编码为 16 字节的小端序用户态 ABI 布局（前 4 字节为 `events`，中间 4 字节清零填充，后 8 字节为 `data`）。
fn encoded(event: EpollEvent) -> [u8; 16] {
    let mut bytes = [0; 16];
    bytes[..4].copy_from_slice(&event.events.to_le_bytes());
    bytes[8..].copy_from_slice(&event.data.to_le_bytes());
    bytes
}

/// 将单个 `EpollEvent` 通过 `Mm::copyout` 拷贝到用户虚拟地址 `address`；若发生缺页错误或部分写入则返回 `FsError::BadAddress`。
fn user_emit(memory: &Mm, address: usize, event: EpollEvent) -> Result<(), FsError> {
    let bytes = encoded(event);
    let result = memory.copyout(address, &bytes);
    if result.error.is_some() || result.copied != bytes.len() {
        Err(FsError::BadAddress)
    } else {
        Ok(())
    }
}

/// 获取 `(target, fd)` 对应 `epoll` 内部监听节点的弱引用，用于断言节点在注销或关闭后是否被彻底销毁。
fn watch(file: &EpollFile, target: &FileRef, fd: i32) -> WatchWeak {
    file.ktest_watch_lifetime(target, fd)
        .expect("epoll interest missing")
}

/// 对目标文件调用无等待表的 `poll(None)`，查询其当前的即时就绪事件掩码。
fn mask(context: &TestContext<'_>, file: &FileRef) -> Result<PollMask, TestError> {
    checked(context, file.pollable().unwrap().poll(None))
}

/// 先从文件描述符表 `table` 查找 `fd` 对应的打开文件对象，再对其执行 `EpollFile::control` 操作。
fn control_fd(
    context: &TestContext<'_>,
    fixture: &EpollOwner,
    table: &FileTable,
    operation: u32,
    fd: usize,
    event: Option<EpollEvent>,
) -> TestResult {
    let target = checked(context, table.lookup(fd))?;
    checked(
        context,
        fixture
            .epoll()
            .control(fixture.owner(), operation, &target, fd as i32, event),
    )
}

/// 分配并初始化长度为 `count` 的布尔位图，用于在大规模测试中校验每个预期事件是否均被精确交付一次。
fn delivery_slots(count: usize) -> Result<Vec<bool>, TestError> {
    let mut seen = Vec::new();
    seen.try_reserve_exact(count)
        .map_err(|_| TestError("epoll delivery slots OOM"))?;
    seen.resize(count, false);
    Ok(seen)
}

/// 扫描 `expected`（即 `seen.len()`）个就绪事件，校验每项 `events` 均为 `POLLIN` 且 `data` 落在 `[base, base + expected)` 范围内无重复、无遗漏。
fn scan_data(
    context: &mut TestContext<'_>,
    file: &EpollFile,
    seen: &mut [bool],
    base: u64,
) -> TestResult {
    seen.fill(false);
    let expected = seen.len();
    let result = file.scan(expected, &mut |item| {
        if item.data < base || base + expected as u64 <= item.data {
            return Err(FsError::Corrupt);
        }
        let slot = (item.data - base) as usize;
        if seen[slot] || item.events != PollMask::POLLIN.0 {
            return Err(FsError::Corrupt);
        }
        seen[slot] = true;
        Ok(())
    });
    context.expect_eq(checked(context, result)?, expected);
    context.expect_eq(seen.iter().all(|slot| *slot), true);
    Ok(())
}

/// 返回 `epoll` 测试套件的静态注册锚点，供上层文件系统测试入口汇总。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    events::__ktest_anchor()
}

#[ktest::suite(fixture = EpollOwner, suite_resources = EpollSuite, context = "Thread", tags("fs-epoll"), timeout_ms = 45000)]
pub(crate) mod events {
    use super::*;

    /// 测试目标：验证 `EPOLL_CTL_ADD` 对已就绪管道的即时交付、`EPOLL_CTL_MOD` 修改监听方向后复用原唤醒源，以及无掩码时的 `POLLERR` 无条件上报。
    /// 场景步骤：
    /// 1. 先向管道写入数据，再将读端以 `POLLIN | EPOLLONESHOT` 注册到 `epoll`，验证重复 `ADD` 返回 `Exists` 且首次扫描立即返回 `POLLIN`。
    /// 2. 写满管道缓冲区后，将写端先以 `POLLIN` 注册（扫描返回 0），随后 `MOD` 为 `POLLOUT | EPOLLONESHOT`，读出一块数据腾出空间后验证唤醒并交付 `POLLOUT`。
    /// 3. 删除写端后以事件掩码 `0` 重新注册写端，关闭读端最后一个引用触发断管。
    /// 预期结果：各阶段就绪事件掩码与 `data` 精确匹配；即使注册掩码为 `0`，读端关闭后写端仍能收到 `POLLERR` 事件。
    #[test]
    #[ktest(tags("T13"))]
    fn add_ready_modify_out_and_existing_sources(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let data = 0xf123_4567_89ab_cdef;
        // Step 1: 预先写入数据并注册读端，验证已就绪文件在 ADD 时立即入队且重复 ADD 报错。
        checked(context, fixture.writer().write(b"already"))?;
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, data)),
            ),
        )?;
        context.expect_eq(
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0, 0)),
            ),
            Err(FsError::Exists),
        );

        let mut events = [EpollEvent::default(); 4];
        context.expect_eq(
            mask(context, fixture.owner())?,
            PollMask::POLLIN | PollMask::POLLRDNORM,
        );
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(events[0], event(PollMask::POLLIN.0, data));
        let mut bytes = [0; PIPE_BUF];
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 7);
        context.expect_eq(&bytes[..7], b"already");

        // Step 2: 写满管道后注册写端为 POLLIN，再通过 MOD 改为 POLLOUT，验证读出数据后原有唤醒源能触发写就绪。
        for _ in 0..16 {
            checked(context, fixture.writer().write(&[0x5d; PIPE_BUF]))?;
        }
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.writer(),
                1,
                Some(event(PollMask::POLLIN.0, 11)),
            ),
        )?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_MOD,
                fixture.writer(),
                1,
                Some(event(PollMask::POLLOUT.0 | EPOLLONESHOT, data + 1)),
            ),
        )?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        context.expect_eq(
            checked(context, fixture.reader().read(&mut bytes))?,
            PIPE_BUF,
        );
        context.expect_eq(bytes, [0x5d; PIPE_BUF]);
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(events[0], event(PollMask::POLLOUT.0, data + 1));
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);

        // Step 3: 验证 DEL 之后再次 MOD 返回 NotFound。
        checked(
            context,
            fixture
                .epoll()
                .control(fixture.owner(), EPOLL_CTL_DEL, fixture.writer(), 1, None),
        )?;
        context.expect_eq(
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_MOD,
                fixture.writer(),
                1,
                Some(event(PollMask::POLLOUT.0, 0)),
            ),
            Err(FsError::NotFound),
        );

        // Step 4: 验证 POLLERR 不受用户掩码限制，读端最后一个强引用释放后写端收到 POLLERR 通知。
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.writer(),
                1,
                Some(event(0, data + 2)),
            ),
        )?;
        drop(fixture.reader.take());
        context.expect_eq(fixture.writer().write(b"closed"), Err(FsError::BrokenPipe));
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(events[0], event(PollMask::POLLERR.0, data + 2));
        Ok(())
    }

    /// 测试目标：验证 `(OpenFileDescription, fd)` 复合兴趣键的独立性、描述符编号复用隔离，以及最后一个外部 `Arc<File>` 释放时的自动注销。
    /// 场景步骤：
    /// 1. 将同一管道读端通过 `dup` 复制为两个描述符 `(fd, duplicate)` 并分别注册到 `epoll`，同时克隆一份子文件表保留额外引用 `held`。
    /// 2. 关闭新旧表中的 `fd` 并将新管道读端分配到同一整数编号 `reused == fd`，验证对新管道执行 `DEL` 返回 `NotFound`，再将新管道注册到该编号。
    /// 3. 向新旧两个管道分别写入数据并扫描，随后关闭所有表中的 `duplicate` 并释放 `held`。
    /// 预期结果：第一次扫描同时收到旧管道的两个事件（`fd` 与 `duplicate`）和新管道的 `reused` 事件；直到最后一个外部强引用 `held` 释放后，旧管道的两个监听节点才被彻底销毁。
    #[test]
    #[ktest(tags("T14"))]
    fn dup_fd_reuse_and_last_external_ofd(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        // Step 1: 为同一打开文件对象创建两个描述符并分别注册，同时通过子表持有额外强引用。
        let table = checked(context, FileTable::try_new())?;
        let fd = checked(context, table.reserve(1024))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        let duplicate = checked(context, table.dup(fd, 1024))?;
        let child = checked(context, table.try_clone())?;
        let held = checked(context, child.lookup(fd))?;
        let old_weak = Arc::downgrade(&held);
        control_fd(
            context,
            fixture,
            &table,
            EPOLL_CTL_ADD,
            fd,
            Some(event(PollMask::POLLIN.0 | EPOLLET, 0xa001)),
        )?;
        control_fd(
            context,
            fixture,
            &table,
            EPOLL_CTL_ADD,
            duplicate,
            Some(event(PollMask::POLLIN.0 | EPOLLET, 0xa002)),
        )?;
        let first_watch = watch(fixture.epoll(), &held, fd as i32);
        let dup_watch = watch(fixture.epoll(), &held, duplicate as i32);

        // Step 2: 关闭原 fd 并在相同描述符编号上绑定新管道，验证新对象不会误匹配旧监听项。
        checked(context, table.close(fd))?;
        checked(context, child.close(fd))?;
        let (new_reader, new_writer) = checked(context, pipe::try_new(NONBLOCK, 0, 0))?;
        let reused = checked(context, table.reserve(1024))?
            .publish(new_reader.clone(), DescriptorFlags::default());
        context.expect_eq(reused, fd);
        context.expect_eq(
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_DEL,
                &new_reader,
                reused as i32,
                None,
            ),
            Err(FsError::NotFound),
        );
        control_fd(
            context,
            fixture,
            &table,
            EPOLL_CTL_ADD,
            reused,
            Some(event(PollMask::POLLIN.0 | EPOLLET, 0xb001)),
        )?;
        checked(context, fixture.writer().write(b"old"))?;
        checked(context, new_writer.write(b"new"))?;
        let mut events = [EpollEvent::default(); 4];
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 3);
        for data in [0xa001, 0xa002, 0xb001] {
            context.expect_eq(
                events[..3]
                    .iter()
                    .filter(|event| event.data == data && event.events == PollMask::POLLIN.0)
                    .count(),
                1,
            );
        }

        // Step 3: 关闭表内剩余描述符但保留外部强引用 held，验证此时监听节点尚未注销。
        checked(context, table.close(duplicate))?;
        checked(context, child.close(duplicate))?;
        drop(fixture.reader.take());
        context.expect_eq(fixture.writer().write(b"tail"), Ok(4));
        context.expect_eq(
            first_watch.strong_count() != 0 && dup_watch.strong_count() != 0,
            true,
        );
        let mut bytes = [0; 7];
        context.expect_eq(checked(context, held.read(&mut bytes))?, 7);
        context.expect_eq(&bytes, b"oldtail");

        // Step 4: 释放最后一个外部强引用 held，验证旧管道的监听节点全部销毁且不再产生事件。
        drop(held);
        context.expect_eq(old_weak.upgrade().is_none(), true);
        context.expect_eq(first_watch.upgrade().is_none(), true);
        context.expect_eq(dup_watch.upgrade().is_none(), true);
        context.expect_eq(fixture.writer().write(b"ended"), Err(FsError::BrokenPipe));
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        context.expect_eq(checked(context, new_reader.read(&mut bytes))?, 3);
        context.expect_eq(&bytes[..3], b"new");
        checked(context, new_writer.write(b"again"))?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(events[0], event(PollMask::POLLIN.0, 0xb001));
        Ok(())
    }

    /// 测试目标：验证水平触发（LT）、边缘触发（ET）和单次触发（`EPOLLONESHOT`）的状态转换与重新激活（re-arm）语义。
    /// 场景步骤：
    /// 1. 以默认 LT 模式注册读端并写入 4 字节，连续扫描两次后再分两次读空缓冲区。
    /// 2. 通过 `EPOLL_CTL_MOD` 切换为 `EPOLLET` 模式，写入 1 字节后连续扫描两次，再追加写入 1 字节并扫描。
    /// 3. 切换为 `EPOLLONESHOT` 模式，写入数据扫描一次后再次写入，验证被禁用；随后调用 `EPOLL_CTL_MOD` 重新激活并扫描。
    /// 预期结果：LT 在缓冲区非空时每次扫描均返回事件；ET 仅在首次或新写入到达后返回一次；`EPOLLONESHOT` 交付一次后静默，直到 `MOD` 重新激活后立即交付剩余数据。
    #[test]
    #[ktest(tags("T15"))]
    fn level_edge_oneshot_and_rearm(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        // Step 1: 验证水平触发（LT）只要管道仍有未读字节就会在每次扫描时持续报告就绪。
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0, 0xc001)),
            ),
        )?;
        checked(context, fixture.writer().write(b"abcd"))?;
        let mut events = [EpollEvent::default(); 2];
        for _ in 0..2 {
            context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
            context.expect_eq(events[0], event(PollMask::POLLIN.0, 0xc001));
        }
        let mut bytes = [0; 8];
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes[..2]))?, 2);
        context.expect_eq(&bytes[..2], b"ab");
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 2);
        context.expect_eq(&bytes[..2], b"cd");
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);

        // Step 2: 切换为边缘触发（ET），验证未读取完数据时不重复报告，但新写入到达会再次触发。
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_MOD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0 | EPOLLET, 0xc002)),
            ),
        )?;
        checked(context, fixture.writer().write(b"E"))?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(events[0], event(PollMask::POLLIN.0, 0xc002));
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        checked(context, fixture.writer().write(b"F"))?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 2);
        context.expect_eq(&bytes[..2], b"EF");

        // Step 3: 切换为单次触发（EPOLLONESHOT），验证交付一次后自动禁用，且可通过 MOD 重新激活。
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_MOD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, 0xc003)),
            ),
        )?;
        checked(context, fixture.writer().write(b"single"))?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(events[0], event(PollMask::POLLIN.0, 0xc003));
        checked(context, fixture.writer().write(b"!"))?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_MOD,
                fixture.reader(),
                0,
                Some(event(
                    PollMask::POLLIN.0 | EPOLLONESHOT,
                    0xffff_eeee_dddd_cccc,
                )),
            ),
        )?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 1);
        context.expect_eq(events[0], event(PollMask::POLLIN.0, 0xffff_eeee_dddd_cccc));
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 7);
        context.expect_eq(&bytes[..7], b"single!");
        Ok(())
    }

    /// 测试目标：验证边缘触发（ET）在 `scan` 回调执行期间并发到达的新写入通知不会丢失，且受 `maxevents` 限制未交付的溢出项会被保留。
    /// 场景步骤：
    /// 1. 注册两个 ET 管道读端并各写入初始数据，使两项均进入就绪队列。
    /// 2. 以 `maxevents = 1` 调用 `scan`；在第一个管道的回调中读空初始字节 `"a"`，随后放行另一 CPU 上的子任务向两个管道分别追加写入 `"b"` 和 `"more"` 并等待其完成。
    /// 3. 结束第一次 `scan` 后再次调用 `collect` 扫描。
    /// 预期结果：第一次扫描仅交付第一个管道；第二次扫描同时交付因并发新写入重新入队的第一个管道，以及第一次因 `maxevents = 1` 溢出的第二个管道。
    #[test]
    #[ktest(tags("T16", "T11"), min_cpus = 2)]
    fn scan_overflow_new_input_from_other_cpu(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        // Step 1: 注册两个边缘触发管道并写入初始数据，准备跨核并发写入任务。
        let (reader, writer) = checked(context, pipe::try_new(NONBLOCK, 0, 0))?;
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0 | EPOLLET, 0xd001)),
            ),
        )?;
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                &reader,
                1,
                Some(event(PollMask::POLLIN.0 | EPOLLET, 0xd002)),
            ),
        )?;
        checked(context, fixture.writer().write(b"a"))?;
        checked(context, writer.write(b"tail"))?;

        let release = fixture.gate()?;
        let done = fixture.gate()?;
        let task_release = Arc::clone(&release);
        let task_done = Arc::clone(&done);
        let first_writer = fixture.writer().clone();
        fixture.run(0, (cpu::current().id + 1) % cpu::count(), move || {
            task_release.wait();
            let result = first_writer.write(b"b") == Ok(1) && writer.write(b"more") == Ok(4);
            task_done.complete();
            if result {
                0
            } else {
                1
            }
        })?;

        // Step 2: 在 maxevents=1 的扫描回调内读完首管道旧数据，并同步等待另一核完成新写入。
        let mut delivered = None;
        let mut consumed = [0; 1];
        let result = fixture.epoll().scan(1, &mut |event| {
            delivered = Some(event);
            if fixture.reader().read(&mut consumed)? != 1 {
                return Err(FsError::InvalidOperation);
            }
            release.complete();
            done.wait();
            Ok(())
        });
        context.expect_eq(checked(context, result)?, 1);
        context.expect_eq(delivered, Some(event(PollMask::POLLIN.0, 0xd001)));
        context.expect_eq(&consumed, b"a");
        fixture.join(context);

        // Step 3: 验证后续扫描既能收到首管道的新写入通知，也能收到次管道被推迟的溢出事件。
        let mut events = [EpollEvent::default(); 4];
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 2);
        context.expect_eq(
            events[..2]
                .iter()
                .filter(|event| event.data == 0xd001 && event.events == PollMask::POLLIN.0)
                .count(),
            1,
        );
        context.expect_eq(
            events[..2]
                .iter()
                .filter(|event| {
                    event.data == 0xd002 && event.events == (PollMask::POLLIN | PollMask::POLLHUP).0
                })
                .count(),
            1,
        );
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        context.expect_eq(checked(context, fixture.reader().read(&mut consumed))?, 1);
        context.expect_eq(&consumed, b"b");
        let mut tail = [0; 8];
        context.expect_eq(checked(context, reader.read(&mut tail))?, 8);
        context.expect_eq(&tail, b"tailmore");
        Ok(())
    }

    /// 测试目标：验证向用户空间拷贝事件发生跨页缺页错误（`EFAULT`）时，已成功交付的前缀计数与未完整交付的 `EPOLLONESHOT` 状态保持一致。
    /// 场景步骤：
    /// 1. 注册两个 `EPOLLONESHOT` 管道并写入数据；在用户地址空间映射单页，将输出起始地址设在页尾前 8 字节（`ADDRESS + PIPE_BUF - 8`），使首项 16 字节跨入未映射页。
    /// 2. 将输出起始地址设在页尾前 16 字节（`ADDRESS + PIPE_BUF - 16`）再次扫描 2 项，使首项成功写入、次项跨入未映射页。
    /// 3. 将输出地址设在页首 `ADDRESS` 扫描剩余项，随后验证两项均已因 `EPOLLONESHOT` 禁用，最后通过 `MOD` 重新激活。
    /// 预期结果：首项跨界时返回 `BadAddress`，但页内前 8 字节已写入且两项均未消费；次项跨界时返回已成功交付数 `1`，次项保留在就绪队列并在第三次扫描中成功交付。
    #[test]
    #[ktest(tags("T17", "T11"))]
    fn output_fault_preserves_first_and_positive_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let (reader, writer) = checked(context, pipe::try_new(NONBLOCK, 0, 0))?;
        let first = 0xfedc_ba98_7654_3210;
        let second = 0x8765_4321_ffff_eeee;
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, first)),
            ),
        )?;
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                &reader,
                1,
                Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, second)),
            ),
        )?;
        checked(context, fixture.writer().write(b"first"))?;
        checked(context, writer.write(b"second"))?;

        // Step 1: 映射单页用户内存，使首项输出跨越页边界，验证返回 BadAddress 且首项保留就绪。
        let memory = Mm::try_new().map_err(|_| TestError("epoll user MM OOM"))?;
        memory
            .map_anonymous(ADDRESS, PIPE_BUF, RW, RW)
            .map_err(|_| TestError("epoll user mapping failed"))?;
        let first_address = ADDRESS + PIPE_BUF - 8;
        context.expect_eq(
            fixture
                .epoll()
                .scan(2, &mut |event| user_emit(&memory, first_address, event)),
            Err(FsError::BadAddress),
        );
        let mut partial = [0xa5; 8];
        context.expect_eq(memory.copyin(&mut partial, first_address).copied, 8);
        context.expect_eq(
            &partial[..],
            &encoded(event(PollMask::POLLIN.0, first))[..8],
        );

        // Step 2: 从页尾前 16 字节开始输出，使首项完整写入而次项跨界，验证返回成功前缀 1 且次项未丢失。
        let output = ADDRESS + PIPE_BUF - 16;
        let mut address = output;
        let mut complete = 0;
        let result = fixture.epoll().scan(2, &mut |event| {
            user_emit(&memory, address, event)?;
            complete += 1;
            address += 16;
            Ok(())
        });
        context.expect_eq(checked(context, result)?, 1);
        context.expect_eq(complete, 1);
        let mut bytes = [0; 16];
        context.expect_eq(memory.copyin(&mut bytes, output).copied, 16);
        context.expect_eq(bytes, encoded(event(PollMask::POLLIN.0, first)));
        context.expect_eq(
            checked(
                context,
                fixture
                    .epoll()
                    .scan(2, &mut |event| user_emit(&memory, ADDRESS, event)),
            )?,
            1,
        );
        context.expect_eq(memory.copyin(&mut bytes, ADDRESS).copied, 16);
        context.expect_eq(bytes, encoded(event(PollMask::POLLIN.0, second)));
        let mut events = [EpollEvent::default(); 2];
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);

        // Step 3: 验证两项完整交付后均进入 ONESHOT 禁用态，经 MOD 重新激活后可再次交付。
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_MOD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, first + 1)),
            ),
        )?;
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_MOD,
                &reader,
                1,
                Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, second + 1)),
            ),
        )?;
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 2);
        let mut payload = [0; 6];
        context.expect_eq(checked(context, fixture.reader().read(&mut payload))?, 5);
        context.expect_eq(&payload[..5], b"first");
        context.expect_eq(checked(context, reader.read(&mut payload))?, 6);
        context.expect_eq(&payload, b"second");
        Ok(())
    }

    /// 测试目标：验证对 `EpollFile` 自身执行带等待表的 `poll` 注册后，并发扫描持有的睡眠互斥锁（`SleepMutex`）能正确阻塞后续轮询查询且仅维护单一等待节点。
    /// 场景步骤：
    /// 1. 对空 `epoll` 实例调用 `poll(Some(&mut table))` 注册等待者，写入管道验证等待者被唤醒。
    /// 2. 启动子任务 0 进入 `scan` 回调并暂停（持有 `scan` 睡眠锁）；启动子任务 1 先通过 `park_until` 消费唤醒，再调用 `poll(None)` 竞争睡眠锁。
    /// 3. 主线程确认子任务 1 在调度器中进入 `State::Blocked` 后放行子任务 0，待两任务汇合后检查等待表长度与触发状态。
    /// 预期结果：子任务 1 阻塞在 `scan` 的 `SleepMutex` 上；锁释放后查询成功完成，等待表长度保持为 1，且销毁等待表后新写入不再触发旧通知。
    #[test]
    #[ktest(tags("T11"), min_cpus = 2)]
    fn early_notifications_and_sleeping_query_have_one_waiter(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        checked(
            context,
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.reader(),
                0,
                Some(event(PollMask::POLLIN.0, 0xe001)),
            ),
        )?;
        let poll = checked(context, PollContext::try_new())?;
        let sink: Arc<dyn PollNotify> = poll.clone();
        let mut table = checked(context, PollTable::new(sink, 2))?;
        context.expect_eq(
            checked(
                context,
                fixture.owner().pollable().unwrap().poll(Some(&mut table)),
            )?,
            PollMask::default(),
        );
        poll.clear_triggered();
        checked(context, fixture.writer().write(b"wake"))?;
        context.expect_eq(poll.triggered(), true);

        // Step 1: 子任务 0 在 scan 回调内挂起以持有睡眠锁，子任务 1 唤醒后调用 poll(None) 竞争该锁。
        let inside = fixture.gate()?;
        let release = fixture.gate()?;
        let parked = fixture.gate()?;
        let task_inside = Arc::clone(&inside);
        let task_release = Arc::clone(&release);
        let scanner = fixture.owner().clone();
        fixture.run(0, cpu::current().id, move || {
            let result = scanner.epoll().unwrap().scan(1, &mut |_event| {
                task_inside.complete();
                task_release.wait();
                Ok(())
            });
            if result == Ok(1) {
                0
            } else {
                1
            }
        })?;

        inside.wait();
        let task_poll = Arc::clone(&poll);
        let task_parked = Arc::clone(&parked);
        let querying = fixture.owner().clone();
        fixture.run(1, (cpu::current().id + 1) % cpu::count(), move || {
            if task_poll.park_until(false, None) != WaitResult::Woken {
                task_parked.complete();
                return 1;
            }
            task_parked.complete();
            task_poll.clear_triggered();
            match querying.pollable().unwrap().poll(None) {
                Ok(mask) if mask == PollMask::POLLIN | PollMask::POLLRDNORM => 0,
                _ => 1,
            }
        })?;
        parked.wait();
        until("epoll query did not block on real scan mutex", || {
            fixture.blocked(1)
        })?;

        // Step 2: 确认子任务 1 进入调度器 Blocked 状态后放行扫描锁，并验证等待表清理后不再收到通知。
        release.complete();
        fixture.join(context);
        context.expect_eq(table.len(), 1);
        context.expect_eq(poll.triggered(), true);
        let mut bytes = [0; 4];
        context.expect_eq(checked(context, fixture.reader().read(&mut bytes))?, 4);
        context.expect_eq(&bytes, b"wake");
        let mut events = [EpollEvent::default(); 1];
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);

        drop(table);
        poll.clear_triggered();
        checked(context, fixture.writer().write(b"after unregister"))?;
        context.expect_eq(poll.triggered(), false);
        Ok(())
    }

    /// 测试目标：验证后台持续向管道写入触发通知的同时，并发执行 `EPOLL_CTL_DEL`、释放 `EpollFile` 或关闭目标文件最后一个描述符的内存安全与注销彻底性。
    /// 场景步骤：
    /// 1. 阶段 0：另一 CPU 子任务循环向管道写入字节，主线程在累计至少 100 字节后执行 `EPOLL_CTL_DEL`，验证监听节点弱引用归零且后续写入不再唤醒 `epoll` 的轮询上下文。
    /// 2. 阶段 1：重新注册并在并发写入期间直接释放 `EpollFile` 最后一个强引用，验证 `epoll` 实例与监听节点同步销毁且无悬挂通知。
    /// 3. 阶段 2：新建 `epoll` 注册读端，在并发写入期间关闭描述符表中的最后一个读端描述符，验证写端收到 `BrokenPipe` 且监听节点同步销毁。
    /// 预期结果：三种并发注销路径均无死锁或释放后使用（UAF），弱引用立即失效，且管道内已生产的字节数与读出字节数完全相等。
    #[test]
    #[ktest(tags("T18"), min_cpus = 2)]
    fn del_epoll_drop_and_last_close_race_real_notifications(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let poll = checked(context, PollContext::try_new())?;
        let sink: Arc<dyn PollNotify> = poll.clone();
        let mut table = checked(context, PollTable::new(sink, 1))?;
        checked(
            context,
            fixture.owner().pollable().unwrap().poll(Some(&mut table)),
        )?;
        // Step 1: 分两轮测试持续写入下并发执行 EPOLL_CTL_DEL（phase 0）与直接丢弃 EpollFile（phase 1）。
        for phase in 0..2 {
            let data = 0xab00 + phase as u64;
            checked(
                context,
                fixture.epoll().control(
                    fixture.owner(),
                    EPOLL_CTL_ADD,
                    fixture.reader(),
                    0,
                    Some(event(PollMask::POLLIN.0 | EPOLLET, data)),
                ),
            )?;
            let lifetime = watch(fixture.epoll(), fixture.reader(), 0);
            let epoll_weak = Arc::downgrade(fixture.owner());
            let produced = Arc::try_new(AtomicUsize::new(0))
                .map_err(|_| TestError("epoll producer count OOM"))?;
            let count = Arc::clone(&produced);
            let stop = Arc::clone(&fixture.stop);
            stop.store(false, Ordering::Release);
            let writer = fixture.writer().clone();
            fixture.run(phase, (cpu::current().id + 1) % cpu::count(), move || {
                while !stop.load(Ordering::Acquire) {
                    match writer.write(b"n") {
                        Ok(1) => {
                            count.fetch_add(1, Ordering::Release);
                        }
                        Err(FsError::WouldBlock) => sched::yield_now(),
                        _ => return 1,
                    }
                    sched::cond_resched();
                }
                0
            })?;
            until("epoll concurrent source produced no bytes", || {
                100 <= produced.load(Ordering::Acquire)
            })?;
            if phase == 0 {
                checked(
                    context,
                    fixture.epoll().control(
                        fixture.owner(),
                        EPOLL_CTL_DEL,
                        fixture.reader(),
                        0,
                        None,
                    ),
                )?;
            } else {
                drop(fixture.owner.take());
                context.expect_eq(epoll_weak.upgrade().is_none(), true);
            }
            context.expect_eq(lifetime.upgrade().is_none(), true);
            poll.clear_triggered();

            // Step 2: 后台继续写入管道，验证已注销的 epoll 监听钩子不会再触发上层 PollContext。
            let before = produced.load(Ordering::Acquire);
            let mut bytes = [0; 257];
            let mut read = checked(context, fixture.reader().read(&mut bytes))?;
            context.expect_eq(bytes[..read].iter().all(|byte| *byte == b'n'), true);
            until("epoll source stopped after unregister", || {
                before < produced.load(Ordering::Acquire)
            })?;
            context.expect_eq(poll.triggered(), false);
            fixture.stop.store(true, Ordering::Release);
            fixture.join(context);
            loop {
                match fixture.reader().read(&mut bytes) {
                    Ok(count) => {
                        context.expect_eq(bytes[..count].iter().all(|byte| *byte == b'n'), true);
                        read += count;
                    }
                    Err(FsError::WouldBlock) => break,
                    other => {
                        context.diagnostic(format_args!("epoll drain failed: {:?}", other));
                        return Err(TestError("epoll producer drain failed"));
                    }
                }
            }
            context.expect_eq(read, produced.load(Ordering::Acquire));
            if phase == 0 {
                let mut events = [EpollEvent::default(); 1];
                context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
            }
        }

        // Step 3: 测试持续写入期间并发关闭目标读端的最后一个文件描述符，验证自动注销与 BrokenPipe 传递。
        drop(table);
        let owner: FileRef = checked(context, EpollFile::try_new(0))?;
        fixture.owner = Some(owner);
        let descriptors = checked(context, FileTable::try_new())?;
        let fd = checked(context, descriptors.reserve(1024))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        control_fd(
            context,
            fixture,
            &descriptors,
            EPOLL_CTL_ADD,
            fd,
            Some(event(PollMask::POLLIN.0, 0xabff)),
        )?;
        let lifetime = watch(fixture.epoll(), fixture.reader(), fd as i32);
        let target_weak = Arc::downgrade(fixture.reader());
        drop(fixture.reader.take());
        let produced =
            Arc::try_new(AtomicUsize::new(0)).map_err(|_| TestError("epoll close count OOM"))?;
        let broken = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("epoll close observation OOM"))?;
        let task_count = Arc::clone(&produced);
        let task_broken = Arc::clone(&broken);
        let stop = Arc::clone(&fixture.stop);
        stop.store(false, Ordering::Release);
        let writer = fixture.writer().clone();
        fixture.run(2, (cpu::current().id + 1) % cpu::count(), move || {
            while !stop.load(Ordering::Acquire) {
                match writer.write(b"m") {
                    Ok(1) => {
                        task_count.fetch_add(1, Ordering::Release);
                    }
                    Err(FsError::BrokenPipe) => {
                        task_broken.store(true, Ordering::Release);
                        return 0;
                    }
                    Err(FsError::WouldBlock) => sched::yield_now(),
                    _ => return 1,
                }
                sched::cond_resched();
            }
            0
        })?;
        until("epoll last-close producer produced no bytes", || {
            100 <= produced.load(Ordering::Acquire)
        })?;
        checked(context, descriptors.close(fd))?;
        fixture.join(context);
        context.expect_eq(broken.load(Ordering::Acquire), true);
        context.expect_eq(target_weak.upgrade().is_none(), true);
        context.expect_eq(lifetime.upgrade().is_none(), true);
        let mut events = [EpollEvent::default(); 1];
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        context.expect_eq(fixture.writer().write(b"closed"), Err(FsError::BrokenPipe));
        Ok(())
    }

    /// 测试目标：验证在 `scan` 回调内部关闭目标文件的最后一个文件描述符时，扫描帧持有的临时强引用会将文件析构推迟到控制锁释放之后。
    /// 场景步骤：
    /// 1. 注册管道读端到 `epoll` 并丢弃夹具持有的读端副本，使文件描述符表中的 `fd` 成为唯一外部强引用。
    /// 2. 写入数据后调用 `scan`，在回调内部调用 `table.close(fd)` 关闭该描述符，当场检查 `target.strong_count()` 并尝试从写端继续写入。
    /// 3. `scan` 返回后再次检查目标文件与监听节点的弱引用，并尝试向写端写入。
    /// 预期结果：回调内部关闭 `fd` 后强引用计数仍为 `1`（由 `scan` 临时持有），写端写入成功；`scan` 退出并释放内部锁后，目标文件与监听节点才被销毁，后续写入返回 `BrokenPipe`。
    #[test]
    #[ktest(tags("T14", "T18"))]
    fn scan_defers_last_target_put_until_control_unlock(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let table = checked(context, FileTable::try_new())?;
        let fd = checked(context, table.reserve(1024))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        control_fd(
            context,
            fixture,
            &table,
            EPOLL_CTL_ADD,
            fd,
            Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, 0xac01)),
        )?;
        let lifetime = watch(fixture.epoll(), fixture.reader(), fd as i32);
        let target = Arc::downgrade(fixture.reader());
        drop(fixture.reader.take());
        checked(context, fixture.writer().write(b"last"))?;

        let mut delivered = None;
        let mut query_holds_last = false;
        let mut write_while_pinned = None;
        let result = fixture.epoll().scan(1, &mut |item| {
            delivered = Some(item);
            table.close(fd)?;
            query_holds_last = target.strong_count() == 1;
            write_while_pinned = Some(fixture.writer().write(b"live"));
            Ok(())
        });
        context.expect_eq(checked(context, result)?, 1);
        context.expect_eq(delivered, Some(event(PollMask::POLLIN.0, 0xac01)));
        context.expect_eq(query_holds_last, true);
        context.expect_eq(write_while_pinned, Some(Ok(4)));
        context.expect_eq(target.upgrade().is_none(), true);
        context.expect_eq(lifetime.upgrade().is_none(), true);
        context.expect_eq(fixture.writer().write(b"ended"), Err(FsError::BrokenPipe));
        let mut events = [EpollEvent::default(); 1];
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut events))?, 0);
        Ok(())
    }

    /// 测试目标：验证 `EpollFile` 作为匿名内核文件对象的 `stat` 元数据、`seek` 空操作语义，以及不支持的文件操作和非法控制参数的错误码。
    /// 场景步骤：
    /// 1. 验证 `EpollFile::try_new(NONBLOCK)` 被拒绝，而 `CLOEXEC` 创建成功且两次创建的匿名 inode 编号递增；检查 `stat` 各字段默认值。
    /// 2. 对 `EpollFile` 依次调用 `read`、`write`、`truncate`、`sync_range`、`read_at`、`write_at`、`seek`、`readdir` 与 `ioctl`。
    /// 3. 验证将 `epoll` 自身或另一 `epoll` 实例注册为监听目标、传入未知事件位或 `maxevents = 0` 时的错误返回。
    /// 预期结果：不支持的读写与控制操作分别返回 `InvalidOperation`、`IllegalSeek`、`NotDirectory` 或 `NotTty`，`seek` 始终返回 `Ok(0)`。
    #[test]
    #[ktest(tags("T13", "T14", "T18"))]
    fn anonymous_event_capabilities(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        context.expect_eq(
            EpollFile::try_new(NONBLOCK).err(),
            Some(FsError::InvalidOperation),
        );
        let cloexec: FileRef = checked(context, EpollFile::try_new(CLOEXEC))?;
        context.expect_eq(cloexec.flags(), 2);
        let stat = checked(context, fixture.owner().stat())?;
        context.expect_eq(
            (
                stat.kind,
                stat.mode,
                stat.device,
                stat.rdev,
                stat.size,
                stat.blocks,
            ),
            (InodeKind::Other, 0o600, 0, 0, 0, 0),
        );
        context.expect_eq(
            (
                stat.atime.seconds,
                stat.mtime.seconds,
                stat.ctime.seconds,
                stat.atime.nanoseconds,
                stat.mtime.nanoseconds,
                stat.ctime.nanoseconds,
            ),
            (0, 0, 0, 0, 0, 0),
        );
        context.expect_eq(stat.inode < checked(context, cloexec.stat())?.inode, true);

        let mut untouched = [0xa5; 3];
        context.expect_eq(
            fixture.owner().read(&mut untouched),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(untouched, [0xa5; 3]);
        context.expect_eq(
            fixture.owner().write(b"bytes"),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(fixture.owner().truncate(1), Err(FsError::InvalidOperation));
        context.expect_eq(
            fixture.owner().sync_range(0..1, false),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(
            fixture.owner().read_at(0, &mut untouched),
            Err(FsError::IllegalSeek),
        );
        context.expect_eq(fixture.owner().write_at(0, b"x"), Err(FsError::IllegalSeek));
        for from in [
            SeekFrom::Start(u64::MAX),
            SeekFrom::Current(-1),
            SeekFrom::End(99),
        ] {
            context.expect_eq(fixture.owner().seek(from), Ok(0));
        }
        context.expect_eq(fixture.owner().readdir().err(), Some(FsError::NotDirectory));
        context.expect_eq(
            fixture.owner().ioctl(0xffff, 0, FileIoContext::Kernel),
            Err(FsError::NotTty),
        );

        context.expect_eq(
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.owner(),
                0,
                Some(event(1, 0)),
            ),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                &cloexec,
                1,
                Some(event(1, 0)),
            ),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(
            fixture.epoll().control(
                fixture.owner(),
                EPOLL_CTL_ADD,
                fixture.reader(),
                0,
                Some(event(1 << 28, 0)),
            ),
            Err(FsError::InvalidOperation),
        );
        context.expect_eq(
            fixture.epoll().scan(0, &mut |_event| Ok(())),
            Err(FsError::InvalidOperation),
        );
        Ok(())
    }

    /// 测试目标：验证监听项数量跨越内部初始容量（127、128、129、1024）动态扩容时，64 位 `data` 完整性、陈旧水平触发复核、删除重加以及描述符复用均正常工作。
    /// 场景步骤：
    /// 1. 通过 `dup` 创建 `count` 个指向同一管道读端的描述符，全部注册到 `epoll` 并绑定高位 `base + fd` 数据；写入数据后调用 `scan_data` 校验全部交付。
    /// 2. 读空管道后对 `epoll` 自身调用 `poll(None)`，验证所有陈旧的 LT 候选节点被一次性复核清空并返回空掩码。
    /// 3. 删除中间编号描述符再重新添加，关闭首描述符 `first` 并将新管道复用在 `first` 编号上注册，向两个管道写入数据后校验 `count + 1` 项全部精确交付。
    /// 4. 逆序逐个删除并关闭所有描述符，释放新旧管道读端。
    /// 预期结果：所有规模下事件数据无截断或错位，全部引用释放后监听节点弱引用归零且写端返回 `BrokenPipe`。
    #[test]
    #[case("127", 127usize)]
    #[case("128", 128usize)]
    #[case("129", 129usize)]
    #[case("1024", 1024usize)]
    #[ktest(tags("F08", "fs-epoll-growth"))]
    fn growing_interests_preserve_data_reuse_delete_and_last_target(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
        count: usize,
    ) -> TestResult {
        // Step 1: 批量复制描述符并注册 count 个监听项，验证首次全量扫描的数据完整性。
        fixture.prepare(context)?;
        let table = checked(context, FileTable::try_new())?;
        let first = checked(context, table.reserve(count + 1))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        for fd in 1..count {
            context.expect_eq(checked(context, table.dup(first, count + 1))?, fd);
        }
        let mut seen = delivery_slots(count + 1)?;
        let base = 0xf123_4567_89ab_0000;
        let bytes = b"growth-old-full-data";
        checked(context, fixture.writer().write(bytes))?;
        for fd in 0..count {
            control_fd(
                context,
                fixture,
                &table,
                EPOLL_CTL_ADD,
                fd,
                Some(event(PollMask::POLLIN.0, base + fd as u64)),
            )?;
        }

        scan_data(context, fixture.epoll(), &mut seen[..count], base)?;
        let mut old_bytes = [0; 20];
        context.expect_eq(
            checked(context, fixture.reader().read(&mut old_bytes))?,
            bytes.len(),
        );
        context.expect_eq(old_bytes.as_slice(), bytes.as_slice());
        // Step 2: 读空管道后查询 epoll 自身就绪态，验证全部陈旧 LT 候选节点被一次遍历清理。
        context.expect_eq(mask(context, fixture.owner())?, PollMask::default());

        checked(context, fixture.writer().write(bytes))?;
        let middle_fd = count / 2;
        let middle = watch(fixture.epoll(), fixture.reader(), middle_fd as i32);
        control_fd(context, fixture, &table, EPOLL_CTL_DEL, middle_fd, None)?;
        context.expect_eq(middle.upgrade().is_none(), true);
        control_fd(
            context,
            fixture,
            &table,
            EPOLL_CTL_ADD,
            middle_fd,
            Some(event(PollMask::POLLIN.0, base + middle_fd as u64)),
        )?;

        // Step 3: 关闭首描述符并在原编号上复用新管道，验证新旧管道事件互不干扰。
        checked(context, table.close(first))?;
        let (new_reader, new_writer) = checked(context, pipe::try_new(NONBLOCK, 0, 0))?;
        let reused = checked(context, table.reserve(count + 1))?
            .publish(new_reader.clone(), DescriptorFlags::default());
        context.expect_eq(reused, first);
        control_fd(
            context,
            fixture,
            &table,
            EPOLL_CTL_ADD,
            reused,
            Some(event(
                PollMask::POLLIN.0 | EPOLLONESHOT,
                base + count as u64,
            )),
        )?;
        let new_lifetime = watch(fixture.epoll(), &new_reader, reused as i32);
        let new_target = Arc::downgrade(&new_reader);
        let replacement = b"growth-new-full-data";
        checked(context, new_writer.write(replacement))?;
        scan_data(context, fixture.epoll(), &mut seen, base)?;
        context.expect_eq(
            checked(context, fixture.reader().read(&mut old_bytes))?,
            bytes.len(),
        );
        context.expect_eq(old_bytes.as_slice(), bytes.as_slice());
        let mut new_bytes = [0; 20];
        context.expect_eq(
            checked(context, new_reader.read(&mut new_bytes))?,
            replacement.len(),
        );
        context.expect_eq(new_bytes.as_slice(), replacement.as_slice());

        // Step 4: 逆序注销并关闭所有描述符，确认全部监听节点与底层管道正常析构。
        for fd in (1..count).rev() {
            let lifetime = watch(fixture.epoll(), fixture.reader(), fd as i32);
            control_fd(context, fixture, &table, EPOLL_CTL_DEL, fd, None)?;
            context.expect_eq(lifetime.upgrade().is_none(), true);
            checked(context, table.close(fd))?;
        }
        let old = watch(fixture.epoll(), fixture.reader(), first as i32);
        let old_target = Arc::downgrade(fixture.reader());
        drop(fixture.reader.take());
        context.expect_eq(old_target.upgrade().is_none(), true);
        context.expect_eq(old.upgrade().is_none(), true);
        context.expect_eq(fixture.writer().write(b"ended"), Err(FsError::BrokenPipe));
        drop(new_reader);
        checked(context, table.close(reused))?;
        context.expect_eq(new_target.upgrade().is_none(), true);
        context.expect_eq(new_lifetime.upgrade().is_none(), true);
        context.expect_eq(new_writer.write(b"ended"), Err(FsError::BrokenPipe));
        let mut output = [EpollEvent::default(); 1];
        context.expect_eq(checked(context, collect(fixture.epoll(), &mut output))?, 0);
        Ok(())
    }

    /// 测试目标：验证当 1024 个已就绪监听项与上层 `PollTable` 同时存在时，直接销毁 `EpollFile` 能安全分批撤销所有底层唤醒源而不损坏原管道数据。
    /// 场景步骤：
    /// 1. 复制 1024 个管道读端描述符并全部注册到 `epoll`，写入 `"before-drop"` 使全部 1024 项进入就绪队列并扫描验证。
    /// 2. 将 `epoll` 自身注册到上层 `PollTable`，随后直接丢弃 `EpollFile` 的唯一强引用。
    /// 3. 向管道继续写入 `"after-drop"`，检查上层 `PollContext` 是否被误唤醒，并从管道读端读取全部字节。
    /// 预期结果：`EpollFile` 及其 1024 个监听节点全部销毁，后续写入不再触发已销毁 `epoll` 的等待上下文，且管道先后写入的 21 字节被完整读出。
    #[test]
    #[ktest(tags("F08", "fs-epoll-growth"))]
    fn grown_ready_interests_last_epoll_drop_drains_all_sources(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let count = 1024;
        let table = checked(context, FileTable::try_new())?;
        let first = checked(context, table.reserve(count))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        let mut lifetimes = Vec::new();
        lifetimes
            .try_reserve_exact(count)
            .map_err(|_| TestError("epoll lifetime slots OOM"))?;
        let mut seen = delivery_slots(count)?;
        let base = 0xe123_4567_89ab_0000;
        checked(context, fixture.writer().write(b"before-drop"))?;
        for fd in 0..count {
            if fd != first {
                context.expect_eq(checked(context, table.dup(first, count))?, fd);
            }
            control_fd(
                context,
                fixture,
                &table,
                EPOLL_CTL_ADD,
                fd,
                Some(event(PollMask::POLLIN.0, base + fd as u64)),
            )?;
            lifetimes.push(watch(fixture.epoll(), fixture.reader(), fd as i32));
        }
        scan_data(context, fixture.epoll(), &mut seen, base)?;

        let poll = checked(context, PollContext::try_new())?;
        let sink: Arc<dyn PollNotify> = poll.clone();
        let mut sources = checked(context, PollTable::new(sink, 2))?;
        checked(
            context,
            fixture.owner().pollable().unwrap().poll(Some(&mut sources)),
        )?;
        poll.clear_triggered();

        let owner = Arc::downgrade(fixture.owner());
        drop(fixture.owner.take());
        context.expect_eq(owner.upgrade().is_none(), true);
        context.expect_eq(lifetimes.iter().all(|node| node.upgrade().is_none()), true);
        checked(context, fixture.writer().write(b"after-drop"))?;
        context.expect_eq(poll.triggered(), false);
        let mut bytes = [0; 21];
        context.expect_eq(
            checked(context, fixture.reader().read(&mut bytes))?,
            bytes.len(),
        );
        context.expect_eq(bytes.as_slice(), b"before-dropafter-drop".as_slice());
        drop(sources);
        Ok(())
    }

    /// 测试目标：验证超过内部批次容量的 129 个独立管道在同一次 `scan` 的回调中各自关闭最后一个描述符时，所有目标文件的析构均被安全推迟到锁外完成。
    /// 场景步骤：
    /// 1. 创建 129 对独立管道，向每个管道写入 `"payload"`，将读端存入描述符表并以 `EPOLLONESHOT` 注册到 `epoll`，随后丢弃栈上的读端变量。
    /// 2. 调用一次 `scan(129, ...)`，在每项回调中读出 `"payload"`、关闭描述符表中的唯一 `fd`，并当场断言读端强引用计数仍为 `1` 且写端仍可写入 `"held"`。
    /// 3. `scan` 返回后遍历全部 129 对管道，检查读端与监听节点的弱引用并对写端执行写入。
    /// 预期结果：回调期间所有读端均由扫描帧保活；`scan` 结束后全部 129 个读端和监听节点均已释放，所有写端写入均返回 `BrokenPipe`。
    #[test]
    #[ktest(tags("F08", "fs-epoll-growth"))]
    fn grown_scan_defers_all_last_targets_until_control_unlock(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let count = 129;
        let table = checked(context, FileTable::try_new())?;
        let mut targets = Vec::new();
        targets
            .try_reserve_exact(count)
            .map_err(|_| TestError("epoll target slots OOM"))?;
        let base = 0xd123_4567_89ab_0000;
        for fd in 0..count {
            let (reader, writer) = checked(context, pipe::try_new(NONBLOCK, 0, 0))?;
            checked(context, writer.write(b"payload"))?;
            context.expect_eq(
                checked(context, table.reserve(count))?
                    .publish(reader.clone(), DescriptorFlags::default()),
                fd,
            );
            checked(
                context,
                fixture.epoll().control(
                    fixture.owner(),
                    EPOLL_CTL_ADD,
                    &reader,
                    fd as i32,
                    Some(event(PollMask::POLLIN.0 | EPOLLONESHOT, base + fd as u64)),
                ),
            )?;
            let lifetime = watch(fixture.epoll(), &reader, fd as i32);
            targets.push((writer, Arc::downgrade(&reader), lifetime));
        }

        let mut seen = delivery_slots(count)?;
        let result = fixture.epoll().scan(count, &mut |item| {
            if item.data < base || base + count as u64 <= item.data {
                return Err(FsError::Corrupt);
            }
            let fd = (item.data - base) as usize;
            if seen[fd] || item.events != PollMask::POLLIN.0 {
                return Err(FsError::Corrupt);
            }
            let reader = table.lookup(fd)?;
            let mut bytes = [0; 7];
            if reader.read(&mut bytes)? != bytes.len() || bytes != *b"payload" {
                return Err(FsError::Corrupt);
            }
            drop(reader);
            table.close(fd)?;
            if targets[fd].1.strong_count() != 1 || targets[fd].0.write(b"held")? != 4 {
                return Err(FsError::Corrupt);
            }
            seen[fd] = true;
            Ok(())
        });

        context.expect_eq(checked(context, result)?, count);
        context.expect_eq(seen.iter().all(|slot| *slot), true);
        for (writer, target, lifetime) in targets {
            context.expect_eq(target.upgrade().is_none(), true);
            context.expect_eq(lifetime.upgrade().is_none(), true);
            context.expect_eq(writer.write(b"ended"), Err(FsError::BrokenPipe));
        }
        Ok(())
    }

    /// 测试目标：验证在内核堆与物理页耗尽（OOM）的内存压力下，新建 `epoll` 与新增监听项优雅返回 `NoMemory`，而已注册的 129 个监听项仍可无分配地正常扫描，且内存压力解除后可恢复注册。
    /// 场景步骤：
    /// 1. 预先注册 129 个管道描述符并写入数据，再额外复制一个描述符 `extra` 备用。
    /// 2. 开启 `HeapPressure` 耗尽空闲物理页与堆内存，尝试调用 `EpollFile::try_new` 和对 `extra` 执行 `EPOLL_CTL_ADD`，随后对已有的 129 项执行 `scan`。
    /// 3. 释放内存压力后，再次将 `extra` 注册到 `epoll` 并扫描全部 130 项。
    /// 预期结果：OOM 期间创建与新增均返回 `FsError::NoMemory` 且未留下半成品节点，已注册的 129 项扫描无需内存分配即可全部成功交付；解除压力后第 130 项成功加入并交付。
    #[test]
    #[ktest(tags("F08", "fs-epoll-growth", "fs-oom"))]
    fn real_oom_preserves_grown_interests_and_recovers_add(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let count = 129;
        let table = checked(context, FileTable::try_new())?;
        let first = checked(context, table.reserve(count + 1))?
            .publish(fixture.reader().clone(), DescriptorFlags::default());
        let mut seen = delivery_slots(count + 1)?;
        let base = 0xc123_4567_89ab_0000;
        checked(context, fixture.writer().write(b"real-oom-data"))?;
        for fd in 0..count {
            if fd != first {
                checked(context, table.dup(first, count + 1))?;
            }
            control_fd(
                context,
                fixture,
                &table,
                EPOLL_CTL_ADD,
                fd,
                Some(event(PollMask::POLLIN.0, base + fd as u64)),
            )?;
        }
        let extra = checked(context, table.dup(first, count + 1))?;

        // Step 1: 施加堆与页分配压力，验证 try_new 与 ADD 返回 NoMemory，而既有监听项扫描不受影响。
        let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let evidence = pressure.exhaust();
        let creation_error = EpollFile::try_new(0).err();
        let add_error = fixture.epoll().control(
            fixture.owner(),
            EPOLL_CTL_ADD,
            fixture.reader(),
            extra as i32,
            Some(event(PollMask::POLLIN.0, base + extra as u64)),
        );
        let absent = fixture
            .epoll()
            .ktest_watch_lifetime(fixture.reader(), extra as i32)
            .is_none();
        let scanned = fixture.epoll().scan(count, &mut |item| {
            if item.data < base || base + count as u64 <= item.data {
                return Err(FsError::Corrupt);
            }
            let slot = (item.data - base) as usize;
            if seen[slot] || item.events != PollMask::POLLIN.0 {
                return Err(FsError::Corrupt);
            }
            seen[slot] = true;
            Ok(())
        });
        drop(pressure);

        context.expect_eq(evidence, (true, true));
        context.expect_eq(creation_error, Some(FsError::NoMemory));
        context.expect_eq(add_error, Err(FsError::NoMemory));
        context.expect_eq(absent, true);
        context.expect_eq(checked(context, scanned)?, count);
        context.expect_eq(seen[..count].iter().all(|slot| *slot), true);

        // Step 2: 释放内存压力后重新注册 extra 描述符，验证控制路径恢复正常。
        control_fd(
            context,
            fixture,
            &table,
            EPOLL_CTL_ADD,
            extra,
            Some(event(PollMask::POLLIN.0, base + extra as u64)),
        )?;
        scan_data(context, fixture.epoll(), &mut seen, base)?;
        let mut bytes = [0; 13];
        context.expect_eq(
            checked(context, fixture.reader().read(&mut bytes))?,
            bytes.len(),
        );
        context.expect_eq(bytes, *b"real-oom-data");
        Ok(())
    }
}
