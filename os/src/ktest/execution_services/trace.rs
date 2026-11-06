//! 内核追踪（`ktrace`）服务与 `/ktrace` 观测设备集成测试。
//!
//! ## 模块职责与架构定位
//! 本模块属于执行服务测试套件（`execution_services`），负责验证内核追踪子系统（`crate::trace`）与只读 devfs 下的 `/ktrace` 字符设备在任务调度、等待唤醒与快照导出全链路中的行为。
//! - **上游调用者**：`ktest` 框架通过 `cases` 测试套件入口驱动本模块。
//! - **下游依赖**：调度器（`crate::sched`）、同步原语（`Completion`、`SpinLock`）、内核追踪环（`crate::trace`）以及虚拟文件系统观测节点（`/ktrace`）。
//!
//! ## 核心设计与状态流转
//! 测试在 CPU 0 上固定运行协调者与工作任务，通过 `Completion` 交替阻塞与唤醒产生真实的调度切换（`TaskDispatch`）和等待恢复（`WaitResume`）追踪事件，并验证 `/ktrace` 打开文件描述（OFD）的快照隔离性：
//!
//! ```text
//! [默认关闭 (Off)]
//!    │  run_rounds(2) ──> 不产生新记录，全局快照保持为空
//!    ▼
//! ioctl(START) ──> [采集中 (Enabled)]
//!    │  再次 ioctl(START) 或 read() ──> 返回 Err(FsError::Busy)
//!    │  run_rounds(72) ──> 产生 >128 条事件，触发容量为 128 的每 CPU 环形缓冲区覆盖
//!    ▼
//! ioctl(STOP)  ──> [已停止并冻结 (Frozen)]
//!    │  读取旧 OFD (old) ──> 惰性捕获并固定首次停止时的文本快照 (original)
//!    │  run_rounds(8) ──> 停止状态下不再记录新事件
//!    ▼
//! 再次 ioctl(START) -> run_rounds(4) -> ioctl(STOP)
//!    ├── 打开新 OFD (fresh) 读取 ──> 获取第二轮 4 次循环的新快照 (current, 未满 128 条)
//!    └──旧 OFD (old) seek(0) 重读 ──> 仍返回首轮已固定的快照 (original)，与新快照互不干扰
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 本模块仅包含一把自旋锁 `Control::worker`（`SpinLock<Option<Arc<Task>>>`），用于保存工作任务的句柄以供状态轮询。
//! - 持锁期间严禁调用任何可能睡眠或触发调度的函数，也无需嵌套获取其他自旋锁。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::fs::{FileIoContext, FileRef, FsError, SeekFrom};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, State, Task};
use crate::{cpu, sched, trace};
use ::ktest::{Fixture, TestContext, TestError, TestResult};

/// `/ktrace` 设备启动追踪采集的 `ioctl` 命令码。
const START: u32 = 0x4201;
/// `/ktrace` 设备停止追踪采集的 `ioctl` 命令码。
const STOP: u32 = 0x4202;

/// 第一轮开启追踪时驱动工作任务循环阻塞与唤醒的轮数（72 轮足以在 CPU 0 上产生超过 128 条追踪记录，从而触发环形缓冲区覆盖）。
const ROUNDS: usize = 72;

/// 协调者与工作任务之间的同步控制块。
struct Control {
    /// 通知工作任务退出循环的停止标志。
    stop: AtomicBool,
    /// 协调者唤醒工作任务开始下一轮计算的完成量。
    go: Completion,
    /// 工作任务通知协调者本轮计算已完成的完成量。
    done: Completion,
    /// 工作任务已完成的循环轮数计数。
    rounds: AtomicUsize,
    /// 工作任务的句柄，由自旋锁保护，供协调者查询其在调度器中的阻塞状态。
    worker: SpinLock<Option<Arc<Task>>>,
}

impl Control {
    /// 设置停止标志并唤醒可能正阻塞在 `go` 上的工作任务，使其安全退出循环。
    fn release(&self) {
        if !self.stop.swap(true, Ordering::AcqRel) {
            self.go.complete();
        }
    }

    /// 检查工作任务当前在调度器中是否已进入阻塞（`State::Blocked`）状态。
    fn blocked(&self) -> bool {
        self.worker
            .lock()
            .as_ref()
            .is_some_and(|worker| sched::test_state(worker) == State::Blocked)
    }

    /// 驱动工作任务连续执行 `count` 轮“阻塞等待 -> 唤醒执行 -> 完成通知”循环。
    ///
    /// 每轮都会先等待工作任务真正进入 `State::Blocked` 状态，再通过 `go.complete()` 将其唤醒，从而触发真实的调度与等待恢复追踪事件。
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

/// 绑定在 CPU 0 上的工作任务入口函数：循环等待 `go` 信号、执行算术计算并发出 `done` 完成信号。
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

/// 追踪测试套件的夹具（Fixture），负责管理单次测试的控制块与系统内存/元数据资源基线。
struct TraceFixture {
    /// 协调者与工作任务共享的同步控制块。
    control: Arc<Control>,
    /// 测试开始前记录的内存与工作队列元数据资源基线，用于拆卸阶段校验资源是否完全回收。
    initial: super::Baseline,
}

impl Fixture<super::SuiteBaseline> for TraceFixture {
    /// 初始化测试夹具：记录测试前的系统资源基线，并分配同步控制块 `Control`。
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

    /// 静默阶段（Quiesce）：通知工作任务退出并停止全局追踪采集，确保无论测试成功或失败都能安全停机。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.control.release();
        trace::stop().map_err(|_| TestError("trace native stop failed"))
    }

    /// 拆卸阶段（Tear Down）：校验无残留等待者，释放任务引用与控制块，并断言系统内存及元数据回归初始基线。
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

/// 从打开的 `/ktrace` 文件对象中循环读取全部导出文本，并校验总字节数不超过理论上限。
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

/// 从空白分隔的单词迭代器中提取下一个形如 `key=value` 的字段，校验键名匹配并将十进制值解析为 `u64`。
fn number<'a>(words: &mut impl Iterator<Item = &'a str>, key: &str) -> u64 {
    let word = words.next().expect("trace text field missing");
    let (actual, value) = word
        .split_once('=')
        .expect("trace text field has no separator");
    assert_eq!(actual, key);
    assert!(!value.is_empty() && value.bytes().all(|byte| byte.is_ascii_digit()));
    value.parse().expect("trace text value overflow")
}

/// 解析并校验 `/ktrace` 导出的文本快照格式、每 CPU 序号连续性、环形缓冲区覆盖计数以及工作任务的事件记录。
///
/// - `bytes`：从 `/ktrace` 或 `trace::snapshot()` 读取的完整快照字节流。
/// - `worker_id`：工作任务的 `TaskId` 数值，用于核对 `TaskDispatch` 与 `WaitResume` 事件中的目标任务。
/// - `covered`：指示 CPU 0 上的已提交事件数是否预期超过环形缓冲区容量（128 条）并发生覆盖。
fn assert_capture(bytes: &[u8], worker_id: u64, covered: bool) {
    // Step 1: 校验全局文件头（版本号、时间单位、CPU 数量与单核环容量 128）。
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
    // Step 2: 逐个 CPU 校验汇总行（cpu id/next_seq/overwritten）以及紧随其后的保留记录行。
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
        // Step 3: 逐条校验当前 CPU 环形缓冲区内保留的追踪记录（序号连续、时间戳单调非减、事件类型合法）。
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

/// 执行追踪采集状态机与 `/ktrace` 文件句柄快照隔离性的完整测试流程。
fn exercise(control: Arc<Control>) -> TestResult {
    // Step 1: 确认初始状态下追踪关闭，打开首个 `/ktrace` 文件句柄 `old` 并启动工作任务，验证关闭期间运行任务不会产生追踪记录。
    assert_eq!(cpu::current().id, 0);
    assert!(!trace::enabled());
    let old = super::open_observation(b"/ktrace", 0).expect("trace device open failed");
    let task_control = control.clone();
    let child = crate::ktest::spawn_result_on(0, move |_| Ok(worker(task_control)))?;
    let off = trace::snapshot().expect("disabled trace snapshot failed");
    control.run_rounds(2)?;
    assert_eq!(trace::snapshot().unwrap(), off);
    drop(off);

    // Step 2: 通过 `ioctl(START)` 开启追踪，验证重复开启或采集中读取均返回 `FsError::Busy`，并运行 72 轮任务触发超过 128 条事件的环形缓冲区覆盖。
    assert_eq!(old.ioctl(START, 0, FileIoContext::Kernel), Ok(0));
    assert!(trace::enabled());
    assert_eq!(
        old.ioctl(START, 0, FileIoContext::Kernel),
        Err(FsError::Busy)
    );
    assert_eq!(old.read(&mut [0u8; 1]), Err(FsError::Busy));
    control.run_rounds(ROUNDS)?;
    // Step 3: 通过 `ioctl(STOP)` 停止追踪，校验冻结快照的内容与覆盖计数，并通过 `old` 句柄读取以固定其内部快照，随后验证停止状态下继续运行任务不会改变全局快照。
    assert_eq!(old.ioctl(STOP, 0, FileIoContext::Kernel), Ok(0));
    assert!(!trace::enabled());
    let frozen = trace::snapshot().expect("stopped trace snapshot failed");
    assert_capture(&frozen, child.task_id().as_usize() as u64, true);
    let original = read_all(&old);
    assert_eq!(original, frozen);
    control.run_rounds(8)?;
    assert_eq!(old.ioctl(STOP, 0, FileIoContext::Kernel), Ok(0));
    assert_eq!(trace::snapshot().unwrap(), frozen);

    // Step 4: 开启第二轮短采集（4 轮任务，不发生环覆盖）后停止，打开新句柄 `fresh` 验证其读取到最新快照，而将旧句柄 `old` 寻道回开头重读仍保持第一轮固定的旧快照。
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

    // Step 5: 通知工作任务退出并回收句柄资源。
    control.release();
    assert_eq!(child.join()?, 0);
    drop((child, old, fresh, original, current, frozen));
    Ok(())
}

#[::ktest::suite(fixture = TraceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "trace"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// 测试目标：验证内核追踪环在任务阻塞/唤醒下的事件采集、环形缓冲区覆盖计算、停止冻结语义以及 `/ktrace` 不同打开文件描述（OFD）之间的快照隔离性。
    /// 场景步骤：
    /// 1. Given: 追踪默认关闭，在 CPU 0 上启动工作任务并打开首个 `/ktrace` 句柄 `old`，运行 2 轮等待唤醒；
    /// 2. When: 执行 `ioctl(START)` 开启采集（验证重复 `START` 和运行期 `read` 返回 `Busy`），运行 72 轮触发环形缓冲区（容量 128）覆盖后执行 `ioctl(STOP)`，从 `old` 读取快照；随后再开启第二轮运行 4 轮并停止，打开新句柄 `fresh` 读取；
    /// 3. Then: 关闭期间不记录事件；首轮停止后 CPU 0 记录数超过 128 且 `overwritten = committed - 128`；次轮停止后 `fresh` 读到未覆盖的新快照，而 `old` 寻道回零后重读仍完整保留首轮快照。
    #[test]
    fn real_wait_freeze_overwrite_and_old_snapshot(
        _: &mut TestContext<'_>,
        fixture: &mut TraceFixture,
    ) -> TestResult {
        let control = fixture.control.clone();
        super::super::on_zero(control.clone(), Control::release, move || exercise(control))
    }
}
