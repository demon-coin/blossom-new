//! 固定 CPU 的真实调度与 Completion 等待消费停止后的 trace 和独立 OFD 快照。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::fs::{FileIoContext, FileRef, FsError, SeekFrom};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, State, Task};
use crate::{cpu, sched, trace};
use ::ktest::{Fixture, TestContext, TestError, TestResult};

const START: u32 = 0x4201;
const STOP: u32 = 0x4202;
const ROUNDS: usize = 72;

struct Control {
    stop: AtomicBool,
    go: Completion,
    done: Completion,
    rounds: AtomicUsize,
    worker: SpinLock<Option<Arc<Task>>>,
}

impl Control {
    /// 只放行本例业务等待，真实退出及离栈仍由原 scope 的 join 完成。
    fn release(&self) {
        if !self.stop.swap(true, Ordering::AcqRel) {
            self.go.complete();
        }
    }

    /// 保存的原任务引用只观察 scheduler 的真实 Blocked 状态。
    fn blocked(&self) -> bool {
        self.worker
            .lock()
            .as_ref()
            .is_some_and(|worker| sched::test_state(worker) == State::Blocked)
    }

    /// 每轮先证实业务任务真阻塞，再完成并等待其真实计算，不直接调用事件生产 API。
    fn run_rounds(&self, count: usize) -> TestResult {
        for _ in 0..count {
            super::wait(|| self.blocked(), "trace worker did not block")?;
            let before = self.rounds.load(Ordering::Acquire);
            self.go.complete();
            self.done.wait();
            assert_eq!(self.rounds.load(Ordering::Acquire), before + 1);
        }
        super::wait(|| self.blocked(), "trace worker final wait did not block")
    }
}

/// 固定 CPU 普通任务每次真正等待一个 Completion，再计算并提交本轮业务完成。
fn worker(control: Arc<Control>) -> i32 {
    *control.worker.lock() = Some(task::current_task());
    loop {
        control.go.wait();
        if control.stop.load(Ordering::Acquire) {
            return 0;
        }
        assert_eq!(cpu::current().id, 0);
        let round = control.rounds.load(Ordering::Relaxed);
        let value = core::hint::black_box(round)
            .wrapping_mul(17)
            .wrapping_add(7);
        core::hint::black_box(value);
        control.rounds.fetch_add(1, Ordering::Release);
        control.done.complete();
    }
}

struct TraceFixture {
    control: Arc<Control>,
    initial: super::Baseline,
}

impl Fixture<super::SuiteBaseline> for TraceFixture {
    /// 资源基线之后准备具体业务控制，真实任务和 OFD 在 fixed-0 主体中沿原 scope 发表。
    fn set_up(_: &mut TestContext<'_>, _: &super::SuiteBaseline) -> Result<Self, TestError> {
        let initial = super::baseline();
        let control = Arc::try_new(Control {
            stop: AtomicBool::new(false),
            go: Completion::new(),
            done: Completion::new(),
            rounds: AtomicUsize::new(0),
            worker: SpinLock::new(None),
        })
        .map_err(|_| TestError("NoMemory: trace native control"))?;
        Ok(Self { control, initial })
    }

    /// 失败路径也停止真实采集并放行业务等待，原 scope 随后 join 全部已发表任务。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.control.release();
        trace::stop().map_err(|_| TestError("trace native stop failed"))
    }

    /// 原任务离栈后释放观察引用和所有源，完整 MM 与元数据只比较本例同一 VM。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert_eq!(self.control.go.test_waiting(), 0);
        assert_eq!(self.control.done.test_waiting(), 0);
        drop(self.control.worker.lock().take());
        let initial = self.initial;
        drop(self);
        let actual = super::baseline();
        context.diagnostic(format_args!(
            "TRACE resources initial={:?} final={:?}",
            initial, actual
        ));
        if actual != initial {
            return Err(TestError("trace native resources did not converge"));
        }
        Ok(())
    }
}

/// 消费真实 OFD 的完整字节流，并以固定环的文本上界限制本例快照缓冲。
fn read_all(file: &FileRef) -> Vec<u8> {
    let mut bytes = Vec::new();
    let mut chunk = [0u8; 4096];
    loop {
        let count = file.read(&mut chunk).expect("trace OFD read failed");
        if count == 0 {
            return bytes;
        }
        assert!(bytes.len() + count <= crate::boot::MAX_CPUS * 128 * 192 + 1024);
        bytes
            .try_reserve(count)
            .expect("trace test snapshot allocation failed");
        bytes.extend_from_slice(&chunk[..count]);
    }
}

/// 字段标签、十进制值与字段数量均从实际导出行校验，不补缺失值。
fn number<'a>(words: &mut impl Iterator<Item = &'a str>, key: &str) -> u64 {
    let word = words.next().expect("trace text field missing");
    let (actual, value) = word
        .split_once('=')
        .expect("trace text field has no separator");
    assert_eq!(actual, key);
    assert!(!value.is_empty() && value.bytes().all(|byte| byte.is_ascii_digit()));
    value.parse().expect("trace text value overflow")
}

/// 每个 CPU 的真实序号范围必须连续且恰保容量内完整记录，覆盖量由实际 next_seq 核算。
fn assert_capture(bytes: &[u8], worker_id: u64, covered: bool) {
    let text = core::str::from_utf8(bytes).expect("trace snapshot is not UTF-8");
    assert!(text.ends_with('\n'));
    let mut lines = text.lines().peekable();
    let mut header = crate::klog::StackText::<128>::new();
    use core::fmt::Write;
    write!(
        header,
        "ktrace version=1 unit=ns cpus={} capacity=128",
        cpu::count()
    )
    .unwrap();
    assert_eq!(lines.next().unwrap().as_bytes(), header.as_bytes());
    let mut worker_dispatches = 0;
    let mut worker_resumes = 0;
    for id in 0..cpu::count() {
        let mut words = lines
            .next()
            .expect("trace CPU summary missing")
            .split_ascii_whitespace();
        assert_eq!(words.next(), Some("cpu"));
        assert_eq!(number(&mut words, "id"), id as u64);
        let next = number(&mut words, "next_seq");
        let overwritten = number(&mut words, "overwritten");
        assert!(words.next().is_none());
        assert!(1 <= next);
        let committed = next - 1;
        let retained = committed.min(128);
        assert_eq!(overwritten, committed - retained);
        if id == 0 {
            if covered {
                assert!(128 < committed);
            } else {
                assert!(0 < committed && committed <= 128);
            }
            assert_eq!(retained, if covered { 128 } else { committed });
            crate::ktest::report::diagnostic(format_args!(
                "TRACE capture cpu=0 next_seq={} retained={} overwritten={} covered={}",
                next,
                retained,
                overwritten,
                u8::from(covered)
            ));
        }
        let mut previous_ns = 0;
        for sequence in next - retained..next {
            let mut words = lines
                .next()
                .expect("trace complete record missing")
                .split_ascii_whitespace();
            assert_eq!(words.next(), Some("record"));
            assert_eq!(number(&mut words, "seq"), sequence);
            let ns = number(&mut words, "ns");
            assert!(0 < ns && previous_ns <= ns);
            previous_ns = ns;
            assert_eq!(number(&mut words, "cpu"), id as u64);
            let task_id = number(&mut words, "task");
            let kind = number(&mut words, "kind");
            let object = number(&mut words, "object");
            let value = number(&mut words, "value");
            assert!(words.next().is_none());
            assert!((1..=6).contains(&kind));
            if kind == trace::Kind::TaskDispatch as u64 {
                assert_eq!(object, task_id);
                assert!(0 < task_id);
                assert_eq!(value, 0);
                worker_dispatches += usize::from(task_id == worker_id);
            } else if kind == trace::Kind::WaitResume as u64 {
                assert_eq!(object, task_id);
                assert!(0 < task_id && 0 < value);
                worker_resumes += usize::from(task_id == worker_id);
            }
        }
        assert!(!lines.peek().is_some_and(|line| line.starts_with("record ")));
    }
    assert!(lines.next().is_none());
    assert!(0 < worker_dispatches && 0 < worker_resumes);
}

/// 两次采集均由原 Task/Completion 生产，停止后全环冻结且旧 OFD 保留原快照。
fn exercise(control: Arc<Control>) -> TestResult {
    assert_eq!(cpu::current().id, 0);
    assert!(!trace::enabled());
    let old = super::open_observation(b"/ktrace", 0).expect("trace device open failed");
    let task_control = control.clone();
    let child = crate::ktest::spawn_result_on(0, move |_| Ok(worker(task_control)))?;
    let off = trace::snapshot().expect("disabled trace snapshot failed");
    control.run_rounds(2)?;
    assert_eq!(trace::snapshot().unwrap(), off);
    drop(off);

    assert_eq!(old.ioctl(START, 0, FileIoContext::Kernel), Ok(0));
    assert!(trace::enabled());
    assert_eq!(
        old.ioctl(START, 0, FileIoContext::Kernel),
        Err(FsError::Busy)
    );
    assert_eq!(old.read(&mut [0u8; 1]), Err(FsError::Busy));
    control.run_rounds(ROUNDS)?;
    assert_eq!(old.ioctl(STOP, 0, FileIoContext::Kernel), Ok(0));
    assert!(!trace::enabled());
    let frozen = trace::snapshot().expect("stopped trace snapshot failed");
    assert_capture(&frozen, child.task_id().as_usize() as u64, true);
    let original = read_all(&old);
    assert_eq!(original, frozen);
    control.run_rounds(8)?;
    assert_eq!(old.ioctl(STOP, 0, FileIoContext::Kernel), Ok(0));
    assert_eq!(trace::snapshot().unwrap(), frozen);

    assert_eq!(old.ioctl(START, 0, FileIoContext::Kernel), Ok(0));
    assert_eq!(old.read(&mut [0u8; 1]), Err(FsError::Busy));
    control.run_rounds(4)?;
    assert_eq!(old.ioctl(STOP, 0, FileIoContext::Kernel), Ok(0));
    let fresh = super::open_observation(b"/ktrace", 0).expect("new trace OFD open failed");
    let current = read_all(&fresh);
    assert_capture(&current, child.task_id().as_usize() as u64, false);
    assert_eq!(current, trace::snapshot().unwrap());
    assert_ne!(current, frozen);
    assert_eq!(old.seek(SeekFrom::Start(0)), Ok(0));
    assert_eq!(read_all(&old), original);
    assert_eq!(control.rounds.load(Ordering::Acquire), 2 + ROUNDS + 8 + 4);
    control.release();
    assert_eq!(child.join()?, 0);
    drop((child, old, fresh, original, current, frozen));
    Ok(())
}

#[::ktest::suite(fixture = TraceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "trace"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// 真实 fixed-0 等待恢复超过128事件，核默认关闭、冻结、精确覆盖与旧 OFD 快照拥有。
    #[test]
    fn real_wait_freeze_overwrite_and_old_snapshot(
        _: &mut TestContext<'_>,
        fixture: &mut TraceFixture,
    ) -> TestResult {
        let control = fixture.control.clone();
        super::super::on_zero(control.clone(), Control::release, move || exercise(control))
    }
}
