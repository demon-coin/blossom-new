//! 正式 control/scan、真实管道、用户页和任务并发的永久事件回归。

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
const ADDRESS: usize = 0x4700_0000;
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
type Metadata = crate::workqueue::testing::MetadataStats;
type WatchWeak = Weak<dyn Send + Sync>;
/// 保存子任务自己发表的弱身份，观察拥有不延长 Task 生命周期。
type TaskObserver = Arc<SpinLock<Option<Weak<Task>>>>;

struct EpollSuite {
    memory: mm::MemoryStats,
    metadata: Metadata,
}

impl Fixture for EpollSuite {
    /// 原常驻服务停稳后记录同 VM 两层账的组基线，不创建事件或管道。
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

    /// 最后 case 的真实拥有结束后比较第二层六域及 tasks/workers/domains。
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

struct EpollOwner {
    before: mm::MemoryStats,
    metadata: Metadata,
    owner: Option<FileRef>,
    reader: Option<FileRef>,
    writer: Option<FileRef>,
    joins: Vec<JoinHandle>,
    observers: [Option<TaskObserver>; 4],
    gates: Vec<Arc<Completion>>,
    stop: Arc<AtomicBool>,
}

impl EpollOwner {
    /// 事件与两端均为实际 OFD，未准备成功的对象按原拥有结束。
    fn prepare(&mut self, context: &TestContext<'_>) -> TestResult {
        let (reader, writer) = checked(context, pipe::try_new(NONBLOCK, 0, 0))?;
        let owner: FileRef = checked(context, EpollFile::try_new(0))?;
        self.reader = Some(reader);
        self.writer = Some(writer);
        self.owner = Some(owner);
        Ok(())
    }

    /// 控制入口只借原事件 OFD，不构造另一层包装。
    fn owner(&self) -> &FileRef {
        self.owner.as_ref().unwrap()
    }
    /// scan 直接消费原具体事件对象。
    fn epoll(&self) -> &EpollFile {
        self.owner().epoll().unwrap()
    }
    /// 读端借用结束不会改变端点寿命。
    fn reader(&self) -> &FileRef {
        self.reader.as_ref().unwrap()
    }
    /// 写端由原管道实现产生 source 通知。
    fn writer(&self) -> &FileRef {
        self.writer.as_ref().unwrap()
    }

    /// gate 保存在主体 catch 之外，清理能放行尚在真实 emit 窗口的任务。
    fn gate(&mut self) -> Result<Arc<Completion>, TestError> {
        assert!(self.gates.len() < self.gates.capacity());
        let gate =
            Arc::try_new(Completion::new()).map_err(|_| TestError("epoll completion OOM"))?;
        self.gates.push(Arc::clone(&gate));
        Ok(gate)
    }

    /// 发表前准备真实弱任务观察与 JoinHandle，清理不依赖主体正常返回。
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

    /// 原 scheduler 的 Blocked 是真实 SleepMutex 等待证据，开始标记或 sleep 不能替代。
    fn blocked(&self, index: usize) -> bool {
        let task = self.observers[index]
            .as_ref()
            .unwrap()
            .lock()
            .as_ref()
            .and_then(Weak::upgrade);
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 逐个 join 确认离栈并保留子任务错误，之后才放观察引用。
    fn join(&mut self, context: &mut TestContext<'_>) {
        while let Some(join) = self.joins.pop() {
            context.expect_eq(join.join(), 0);
        }
        self.observers.fill_with(|| None);
    }
}

impl Fixture<EpollSuite> for EpollOwner {
    /// 每例在临时申请前保存资源，固定预留清理 gate 和任务句柄槽。
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

    /// 先停生产并放行 gate，结束控制者 OFD 后真实 join；不遗留 source 或后台栈。
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

    /// fixture 的 Vec、gate、观察及全部 OFD 结束后比较每例完整六域和三元数据。
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

/// 兴趣数据带高位，目标 poll 位在正式交付时去掉 ET/ONESHOT 私有位。
fn event(events: u32, data: u64) -> EpollEvent {
    EpollEvent { events, data }
}

/// 本地接收端直接消费正式 scan，输出界限由同次 maxevents 保证。
fn collect(file: &EpollFile, output: &mut [EpollEvent]) -> Result<usize, FsError> {
    let mut count = 0;
    file.scan(output.len(), &mut |event| {
        output[count] = event;
        count += 1;
        Ok(())
    })
}

/// 用户输出只编码目标字段并初始化 padding，不复制宿主结构裸内存。
fn encoded(event: EpollEvent) -> [u8; 16] {
    let mut bytes = [0; 16];
    bytes[..4].copy_from_slice(&event.events.to_le_bytes());
    bytes[8..].copy_from_slice(&event.data.to_le_bytes());
    bytes
}

/// 真实 Mm 的完整项交付才能成功，跨未映射页保原复制前缀而报告 EFAULT。
fn user_emit(memory: &Mm, address: usize, event: EpollEvent) -> Result<(), FsError> {
    let bytes = encoded(event);
    let result = memory.copyout(address, &bytes);
    if result.error.is_some() || result.copied != bytes.len() {
        Err(FsError::BadAddress)
    } else {
        Ok(())
    }
}

/// 观察原节点真实强拥有结束，弱控制尾在测试末释放后参与原资源账。
fn watch(file: &EpollFile, target: &FileRef, fd: i32) -> WatchWeak {
    file.ktest_watch_lifetime(target, fd)
        .expect("epoll interest missing")
}

/// 只由正式 provider 复核事件自身，不以候选链非空冒充真正就绪。
fn mask(context: &TestContext<'_>, file: &FileRef) -> Result<PollMask, TestError> {
    checked(context, file.pollable().unwrap().poll(None))
}

/// 接收队列不冻结表；lookup 取得的原 OFD 在 control 返回后才结束临时拥有。
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

/// 按真实规模准备消费位图，扫描只消费完整高位 data，不按规模展开内核栈。
fn delivery_slots(count: usize) -> Result<Vec<bool>, TestError> {
    let mut seen = Vec::new();
    seen.try_reserve_exact(count)
        .map_err(|_| TestError("epoll delivery slots OOM"))?;
    seen.resize(count, false);
    Ok(seen)
}

/// 完整消费本轮每个 data 和普通事件位，重复、遗漏或外来 data 都失败。
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

/// 原锚点集合只保留实际宏生成 suite，不增加中心分派或新 runner。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    events::__ktest_anchor()
}

#[ktest::suite(fixture = EpollOwner, suite_resources = EpollSuite, context = "Thread", tags("fs-epoll"), timeout_ms = 45000)]
pub(crate) mod events {
    use super::*;

    /// ADD 已 ready 立即交付；初始 IN 的写 OFD 改 OUT 后仍使用 ADD 登记的真实源。
    #[test]
    #[ktest(tags("T13"))]
    fn add_ready_modify_out_and_existing_sources(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let data = 0xf123_4567_89ab_cdef;
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
        // ERR 不依赖用户兴趣；读 OFD 末释放撤其自身 watch 后通知写端真实源。
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

    /// dup 的两个整数键独立，同号新 OFD 不串旧 interest，最后外部 Arc 才撤旧端所有 hook。
    #[test]
    #[ktest(tags("T14"))]
    fn dup_fd_reuse_and_last_external_ofd(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
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

    /// LT 复核剩余字节，ET 仅新通知再交付，ONESHOT 仅完整交付后禁用并可 MOD 重 arm。
    #[test]
    #[ktest(tags("T15"))]
    fn level_edge_oneshot_and_rearm(
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

    /// ET 的真实 emit 边界消耗旧输入后放行另一 CPU 写入，overflow 与未处理尾都必须保留。
    #[test]
    #[ktest(tags("T16", "T11"), min_cpus = 2)]
    fn scan_overflow_new_input_from_other_cpu(
        context: &mut TestContext<'_>,
        fixture: &mut EpollOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
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

    /// 实际用户页跨界故障保首项和正前缀，未完整交付的 ONESHOT 不禁用。
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

    /// 早通知先结束唯一 park Waiter，随后 provider 真正竞争 SleepMutex 仍可再创建原等待者。
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

    /// 真 source 写入与 DEL、事件末释放、目标末关闭并行，注销后不再通知旧事件源。
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
            // 原源继续产生真实字节，旧 epoll hook 已同步撤完，不能再触达旧 poll context。
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

    /// emit 内关闭最后描述符时 query 强拥有继续保端点，扫描解锁后末 put 才反向撤自身 watch。
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

    /// 匿名事件保持真实 stat、noop seek 和能力错误，不把兴趣数量当作对象能力。
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

    /// 实际 dup 表跨越旧额度；完整 data、陈旧复核、同键重加和 fd 复用都走原生产路径。
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
        // 全部 LT 候选已陈旧，自身 poll 必须一次走完真实增长规模的扫描拥有链。
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

    /// 千项 ready 和持续源在末事件拥有结束时分批撤销，原管道仍能完整消费后续字节。
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

    /// 超过旧额度的独立目标在各自 emit 内失去最后描述符，扫描链逐项锁外 put 才能反向撤源。
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

    /// 真实堆页耗尽使新兴趣返回 ENOMEM；已发表的超额候选无分配扫描，放压后同键可正式加入。
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
