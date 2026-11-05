//! 匿名管道真实数据、原子记录、等待、描述符共享及原页归还的永久回归。

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

const ADDRESS: usize = 0x4600_0000;
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

type Metadata = crate::workqueue::testing::MetadataStats;
/// 保存子任务自己发表的弱身份，观察拥有不延长 Task 生命周期。
type TaskObserver = Arc<SpinLock<Option<Weak<Task>>>>;

struct PipeSuite {
    memory: mm::MemoryStats,
    metadata: Metadata,
}

impl Fixture for PipeSuite {
    /// 常驻服务停稳后保存同一 VM 的整组资源，不建立管道或子任务。
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

    /// 全部 case 拥有结束后核第二层六域及真实任务元数据。
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

struct PipeOwner {
    before: mm::MemoryStats,
    metadata: Metadata,
    reader: Option<FileRef>,
    writer: Option<FileRef>,
    joins: Vec<JoinHandle>,
    observers: [Option<TaskObserver>; 4],
}

impl PipeOwner {
    /// 按本例状态准备两个实际 OFD；失败保留原资源账与清理责任。
    fn prepare(&mut self, context: &TestContext<'_>, flags: u32) -> TestResult {
        let (reader, writer) = checked(context, pipe::try_new(flags, 0, 0))?;
        self.reader = Some(reader);
        self.writer = Some(writer);
        Ok(())
    }

    /// 正文只借读 OFD，不在表锁内执行管道操作。
    fn reader(&self) -> &FileRef {
        self.reader.as_ref().unwrap()
    }

    /// 正文只借写 OFD，复制仍是同一打开对象。
    fn writer(&self) -> &FileRef {
        self.writer.as_ref().unwrap()
    }

    /// 在发表真实任务前准备观察控制块；任务记录自己的实际弱身份。
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

    /// 观察原 scheduler 的真实 Blocked，不把延时或开始标志当作等待证据。
    fn blocked(&self, index: usize) -> bool {
        let task = self.observers[index]
            .as_ref()
            .unwrap()
            .lock()
            .as_ref()
            .and_then(Weak::upgrade);
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 显式 join 离栈后结束所有观察拥有，返回值保留每个生产消费者的错误。
    fn join(&mut self, context: &mut TestContext<'_>) {
        while let Some(join) = self.joins.pop() {
            context.expect_eq(join.join(), 0);
        }
        self.observers.fill_with(|| None);
    }
}

impl Fixture<PipeSuite> for PipeOwner {
    /// 每例在任何临时对象准备前保存资源，固定预留四个任务句柄槽。
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

    /// 先结束控制者两端使等待读写者收到 EOF/EPIPE，再逐个真实 join，最后释放观察者。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        drop(self.reader.take());
        drop(self.writer.take());
        self.join(context);
        Ok(())
    }

    /// 控制 Vec、OFD、原页及任务观察者全部结束后核本例完整六域和元数据。
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

/// 只使用实际 provider 查询，读写流不能借普通文件默认 mask 冒充就绪。
fn mask(context: &TestContext<'_>, file: &FileRef) -> Result<PollMask, TestError> {
    checked(context, file.pollable().unwrap().poll(None))
}

/// 图样由外部逻辑位置决定，所有字节均独立比较。
fn byte(offset: usize) -> u8 {
    (offset as u8)
        .wrapping_mul(37)
        .wrapping_add((offset / 251) as u8)
}

/// 父模块只保留此真实宏锚点，不增加新的 runner 或逐例分派。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    streams::__ktest_anchor()
}

#[ktest::suite(fixture = PipeOwner, suite_resources = PipeSuite, context = "Thread", tags("fs-pipe"), timeout_ms = 45000)]
pub(crate) mod streams {
    use super::*;

    /// 超过容量的真实生产者和消费者传输完整图样，写端最后 put 后先读尾部再 EOF。
    #[test]
    #[ktest(tags("T05"))]
    fn complete_transfer_tail_eof_and_in_hup(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
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

    /// 三个实际任务重复小写，跨容量阻塞后每条记录仍完整，消费者逐字节及每源序号检查。
    #[test]
    #[ktest(tags("T06"), min_cpus = 2)]
    fn concurrent_small_writes_keep_whole_records(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
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

    /// 满环不能重用已读前缀，小写没有容量则 EAGAIN，大写保正前缀，零长无条件返回零。
    #[test]
    #[ktest(tags("T07"))]
    fn nonblock_physical_slots_zero_and_short_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
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

        drop(fixture.reader.take());
        context.expect_eq(fixture.writer().write(&[]), Ok(0));
        context.expect_eq(fixture.writer().write(b"x"), Err(FsError::BrokenPipe));
        context.expect_eq(
            mask(context, fixture.writer())?,
            PollMask::POLLOUT | PollMask::POLLWRNORM | PollMask::POLLERR,
        );
        Ok(())
    }

    /// 原 scheduler 确认真正读写 Blocked，数据提交及腾出整槽通知后恢复，登记前事实也不会丢失。
    #[test]
    #[ktest(tags("T07"))]
    fn blocking_read_write_and_persistent_notification(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
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

    /// 真实 dup 和 fork 表克隆共享原 OFD，最后 descriptor 与外部 Arc 结束后才 EOF/EPIPE。
    #[test]
    #[ktest(tags("T08"))]
    fn dup_fork_table_last_ofd_and_anonymous_identity(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
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
        // 未读数据继续由原 ring 拥有，最后写 OFD 释放时随原页一起归还。
        drop(parent);
        drop(child);
        Ok(())
    }

    /// 真实 Mm 的跨页 EFAULT 仅消费交付前缀，输入向量是一笔请求且故障后的前缀仍可读。
    #[test]
    #[ktest(tags("T05", "T06"))]
    fn user_fault_prefix_and_one_vector_request(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
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

    /// 页分配器真实耗尽时新槽 ENOMEM，原槽图样保留；放压后同一 OFD 可再次申请并归还页。
    #[test]
    #[ktest(tags("T07", "T08"))]
    fn real_page_oom_preserves_existing_data_and_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut PipeOwner,
    ) -> TestResult {
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
