//! 内核线程（`Task`）上下文切换、内核栈回收、跨核迁移与调度通知边界测试。
//!
//! # 模块职责与架构定位
//! 本模块归属 `ktest::execution_core`，在多线程 SMP 环境下验证内核任务与调度器的核心执行不变量：
//! - 上游入口：由 `execution_core::anchors` 注册为 `context`、`migration` 与 `fatal` 三个测试套件。
//! - 下游被测模块：`crate::task`（内核线程生命周期、64 KiB 独立内核栈与退出回收）、`crate::sched`（主动让出 `yield_now`、
//!   CPU 亲和性与跨核唤醒/抢占通知）、`crate::cpu`（Per-CPU 指针、`cpu::call` 跨核同步调用与 idle 停点）以及
//!   `crate::arch`（被调用者保存寄存器上下文切换）。
//!
//! # 核心场景与生命周期流转
//! ```text
//!   Tasks::set_up (建立内存与工作队列元数据基线，分配共享 Control 控制块)
//!        |
//!        +---> context 套件：
//!        |       - deep_switch_return : 双线程深层递归栈帧交替切换 -> 验证寄存器与栈数据无串扰 -> 离栈后归还 32 页栈内存
//!        |       - join_exit_observers: 外部保留 Arc<Task> 强引用 -> 线程退出进入 Exited -> 先释放 16 页内核栈，后读取返回值
//!        |
//!        +---> migration 套件：
//!        |       - same_task_round_trip                 : 单线程遍历全部在线 CPU 两轮，验证任务 ID、Per-CPU 基址与栈地址恒定
//!        |       - yield_does_not_reschedule_busy_cpu   : 繁忙远端 CPU 在其他核执行 yield_now 时不收到冗余 KICK_RESCHEDULE
//!        |       - yield_wakes_idle_cpu_without_reschedule: 空闲远端 CPU 被 KICK_SCHED_WAKE 唤醒接管就绪线程，且不置位 KICK_RESCHEDULE
//!        |
//!        +---> fatal 套件（独立 VM 进程级隔离）：
//!                - 关中断（IrqSaveGuard）或关抢占（PreemptGuard）期间调用 yield_now 触发预期致命中止
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `Control::external` 与 `Control::notification_tasks` 各自使用独立的 `SpinLock` 保护测试观察用的 `Option<Arc<Task>>`。
//! - 严禁在持有上述测试自旋锁期间调用 `sched::test_state`、`sched::yield_now` 或析构 `Arc<Task>`：读取时先克隆 `Arc<Task>` 并释放锁，
//!   清理时通过 `Option::take()` 移出引用并在锁外执行 `drop`，防止与调度器运行队列锁发生锁序反转。

use crate::ktest::execution_core::{baseline, reclaimed, until, Baseline};
use crate::ktest::execution_services::SuiteBaseline;
use crate::ktest::{spawn_result, spawn_result_on, ChildHandle, ChildToken};
use crate::sync::{Completion, IrqSaveGuard, SpinLock};
use crate::{arch, cpu, mm, sched, task};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::hint::{black_box, spin_loop};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// 深层递归上下文切换测试中每个工作线程交替让出 CPU 的轮次数。
const ROUNDS: usize = 8;
/// 深层栈测试的递归调用层数（含入口层共产生 `DEPTH + 1` 层栈帧）。
const DEPTH: usize = 8;
/// 每个内核线程独立内核栈的字节大小（64 KiB，对应 4 KiB 页大小下的 16 个物理页）。
const STACK_BYTES: usize = 64 * 1024;

/// 跨线程共享的测试协调控制块与观测状态。
struct Control {
    /// 协调者向工作线程发出的统一启动门控标志（`Release` 置位，`Acquire` 轮询）。
    go: AtomicBool,
    /// 用例静止化（`quiesce`）或异常中止时的协作停止标志。
    stop: AtomicBool,
    /// 已到达指定递归深度或阻塞点的工作线程计数。
    arrived: AtomicUsize,
    /// 控制双线程严格交替执行的轮次令牌（槽位 `0` 与 `1` 交替持有），或在跨核通知测试中充当关中断阶段的门控信号。
    turn: AtomicUsize,
    /// 记录两个槽位工作线程入口栈帧局部变量的虚拟地址（单位：字节地址）。
    stacks: [AtomicUsize; 2],
    /// 记录两个槽位工作线程各自内核栈底部的物理基地址（单位：字节地址，按 64 KiB 对齐）。
    bases: [AtomicUsize; 2],
    /// 记录两个槽位工作线程的内核任务 ID（`TaskId` 数值）。
    ids: [AtomicUsize; 2],
    /// 记录两个槽位工作线程进入入口函数的次数（用于校验线程仅启动一次或通知参与者已就绪）。
    starts: [AtomicUsize; 2],
    /// 记录两个槽位工作线程已完成的交替切换步数或阶段推进标志。
    steps: [AtomicUsize; 2],
    /// 在上下文切换测试中记录各槽位当前处于运行断言段的并发度；在空闲核唤醒测试中槽位 `1` 记录 CPU 1 的 `TestPause::Idle` 停点是否处于已武装状态。
    active: [AtomicUsize; 2],
    /// 记录两个槽位栈上 `LocalDrop` 守卫实际执行析构的次数。
    drops: [AtomicUsize; 2],
    /// 在递归测试中累计校验通过并弹出的栈帧总数；在跨核通知测试中充当释放子线程退出的门控信号。
    frames: AtomicUsize,
    /// 保存被测子线程的外部 `Arc<Task>` 强引用（由独立自旋锁保护），用于验证线程退出后控制块存活但内核栈已提前释放。
    external: SpinLock<Option<Arc<task::Task>>>,
    /// 跨核通知测试中两个参与线程发布的 `Arc<Task>` 引用槽位（各自受独立自旋锁保护）。
    notification_tasks: [SpinLock<Option<Arc<task::Task>>>; 2],
    /// 用于阻塞子线程并在指定时机唤醒退出的完成量同步源。
    exit: Completion,
}

/// 上下文与跨核迁移测试套件的每用例夹具（Fixture）。
struct Tasks {
    /// 用例启动前采集的内核内存与工作队列元数据静止基线。
    initial: Baseline,
    /// 当前用例独占的跨线程协调控制块。
    state: Arc<Control>,
    /// 当前用例派生的子线程句柄槽位，供主线程与 `quiesce`/`tear_down` 观察线程离栈状态。
    handles: [Option<ChildHandle>; 2],
    /// 若当前用例在目标逻辑 CPU 上配置了 `TestPause::Idle` 测试停点，记录该目标 CPU 编号以便异常清理时撤销。
    idle_pause: Option<usize>,
}

impl Fixture<SuiteBaseline> for Tasks {
    /// 采集用例执行前的全局内存与工作队列资源基线，并分配独立的共享控制块 `Control`。
    fn set_up(_: &mut TestContext<'_>, _: &SuiteBaseline) -> Result<Self, TestError> {
        // Step 1: 等待后台服务静止并排空每核缓存，记录当前内存与元数据基线。
        let initial = baseline()?;
        // Step 2: 在进入测试主体前分配控制块，避免运行期缺页或分配失败干扰测试控制流。
        let state = Arc::try_new(Control {
            go: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            arrived: AtomicUsize::new(0),
            turn: AtomicUsize::new(0),
            stacks: core::array::from_fn(|_| AtomicUsize::new(0)),
            bases: core::array::from_fn(|_| AtomicUsize::new(0)),
            ids: core::array::from_fn(|_| AtomicUsize::new(0)),
            starts: core::array::from_fn(|_| AtomicUsize::new(0)),
            steps: core::array::from_fn(|_| AtomicUsize::new(0)),
            active: core::array::from_fn(|_| AtomicUsize::new(0)),
            drops: core::array::from_fn(|_| AtomicUsize::new(0)),
            frames: AtomicUsize::new(0),
            external: SpinLock::new(None),
            notification_tasks: core::array::from_fn(|_| SpinLock::new(None)),
            exit: Completion::new(),
        })
        .map_err(|_| TestError("NoMemory: context controls"))?;
        Ok(Self {
            initial,
            state,
            handles: core::array::from_fn(|_| None),
            idle_pause: None,
        })
    }

    /// 打开所有协作停止门控并唤醒阻塞源，撤销未释放的 CPU idle 停点，等待全部子线程彻底退出内核栈。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        // Step 1: 置位停止与放行标志并触发完成量，解除所有子线程在门控或睡眠点上的等待。
        self.state.stop.store(true, Ordering::Release);
        self.state.go.store(true, Ordering::Release);
        self.state.exit.complete();
        // Step 2: 若本用例曾武装过远端 CPU 的 idle 停点且尚未由测试主体手动释放，在此撤销停点以恢复目标核调度。
        if let Some(target) = self.idle_pause.take() {
            if self.state.active[1].swap(0, Ordering::AcqRel) != 0 {
                cpu::test_cancel_pause(target, cpu::TestPause::Idle);
            }
        }
        // Step 3: 轮询确认所有子线程均已完成上下文切换并彻底离开各自内核栈，且完成量上无残留等待节点。
        until(
            || {
                self.handles
                    .iter()
                    .flatten()
                    .all(|handle| handle.try_join().is_some())
            },
            "context stop did not reach original off-stack completion",
        )?;
        if self.state.exit.test_waiting() != 0 {
            return Err(TestError("exit source retains live stack nodes"));
        }
        Ok(())
    }

    /// 在所有子线程已由测试作用域回收后，释放夹具持有的任务引用与控制块，并校验全局资源完全恢复至初始基线。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        let Self {
            initial,
            state,
            handles,
            idle_pause: _,
        } = self;
        // Step 1: 校验全部子线程均已离栈结束。
        if handles
            .iter()
            .flatten()
            .any(|handle| handle.try_join().is_none())
        {
            return Err(TestError("context Task still on execution stack"));
        }
        // Step 2: 将测试期间保留的外部 `Arc<Task>` 引用移出自旋锁并在锁外释放，随后清空完成量状态。
        let external = state.external.lock().take();
        drop(external);
        for slot in &state.notification_tasks {
            let task = slot.lock().take();
            drop(task);
        }
        drop(handles);
        while state.exit.try_wait() {}
        if state.exit.test_waiting() != 0 {
            return Err(TestError("exit source retains wait nodes"));
        }
        // Step 3: 释放控制块本身，校验物理页与工作队列元数据与 `set_up` 基线完全一致。
        drop(state);
        reclaimed(initial)
    }
}

/// 绑定在工作线程入口栈帧上的局部析构探针，用于验证线程正常返回或栈展开时局部变量恰好析构一次。
struct LocalDrop {
    /// 指向当前用例共享控制块的强引用，用于回写析构计数。
    state: Arc<Control>,
    /// 当前工作线程对应的槽位下标（`0` 或 `1`）。
    slot: usize,
}

impl Drop for LocalDrop {
    /// 递增对应槽位的析构计数，并断言该局部守卫此前从未被重复析构。
    fn drop(&mut self) {
        assert_eq!(
            self.state.drops[self.slot].fetch_add(1, Ordering::AcqRel),
            0
        );
    }
}

/// 递归构造多层栈帧并在最深层执行多轮跨线程上下文切换，返回时逐层校验栈帧数据未被破坏。
///
/// # 参数
/// - `state`：共享协调控制块。
/// - `token`：测试框架提供的协作停止令牌。
/// - `slot`：当前工作线程的槽位编号（`0` 或 `1`）。
/// - `depth`：剩余递归层数；递减至 `0` 时进入最深层交替切换循环。
/// - `root`：入口栈帧局部变量的虚拟地址，用于校验所有递归栈帧均落在同一 64 KiB 内核栈内且向低地址增长。
///
/// # 返回值
/// 返回当前调用链实际完成校验的递归栈帧层数。
#[inline(never)]
fn deep_calls(
    state: &Control,
    token: &ChildToken,
    slot: usize,
    depth: usize,
    root: usize,
) -> Result<usize, TestError> {
    // Step 1: 在当前栈帧填充与槽位及深度绑定的特征魔数，并校验栈帧仍处于同一 64 KiB 栈空间内。
    let pattern = 0x51a0 + slot * 32 + depth;
    let frame = [pattern; 64];
    let address = black_box(&frame) as *const _ as usize;
    assert_eq!(address & !(STACK_BYTES - 1), root & !(STACK_BYTES - 1));
    // Step 2: 若已到达最深层（depth == 0），通知协调者就绪，并在启动门开启后与伙伴线程交替执行 `ROUNDS` 轮切换与寄存器完整性校验。
    let frames = if depth == 0 {
        assert!(address < root, "deep task stack did not descend");
        state.arrived.fetch_add(1, Ordering::Release);
        until(
            || state.go.load(Ordering::Acquire) || token.stop_requested(),
            "task-start-gate",
        )?;
        for round in 0..ROUNDS {
            until(
                || {
                    state.turn.load(Ordering::Acquire) == slot
                        || state.stop.load(Ordering::Acquire)
                        || token.stop_requested()
                },
                "task-turn",
            )?;
            if state.stop.load(Ordering::Acquire) || token.stop_requested() {
                break;
            }
            assert_eq!(state.active[slot].fetch_add(1, Ordering::AcqRel), 0);
            assert_eq!(cpu::current().id, 0);
            assert_eq!(
                task::current_task().id().as_usize(),
                state.ids[slot].load(Ordering::Acquire)
            );
            assert!(
                arch::test_task_context_preservation(),
                "task register preservation failed"
            );
            assert_eq!(state.steps[slot].fetch_add(1, Ordering::AcqRel), round);
            assert_eq!(state.active[slot].fetch_sub(1, Ordering::AcqRel), 1);
            state.turn.store(1 - slot, Ordering::Release);
            sched::yield_now();
        }
        1
    } else {
        deep_calls(state, token, slot, depth - 1, root)? + 1
    };
    // Step 3: 从深层调用或切换循环返回后，校验当前栈帧上的 64 个特征字均未被篡改，并累加完成帧计数。
    assert!(black_box(&frame).iter().all(|value| *value == pattern));
    state.frames.fetch_add(1, Ordering::Relaxed);
    Ok(frames)
}

/// 深栈上下文切换工作线程入口：校验自身 64 KiB 内核栈的物理地址对齐与范围，执行深层递归切换并返回特征退出码。
fn switch_worker(state: Arc<Control>, token: ChildToken, slot: usize) -> Result<i32, TestError> {
    // Step 1: 记录线程启动并在栈顶放置析构探针 `LocalDrop`，校验当前内核栈物理区间大小与对齐满足 64 KiB 契约。
    assert_eq!(state.starts[slot].fetch_add(1, Ordering::AcqRel), 0);
    let local = LocalDrop {
        state: Arc::clone(&state),
        slot,
    };
    let root = black_box(&local) as *const _ as usize;
    let (base, bytes) = task::test_current_stack_range();
    assert_eq!(bytes, STACK_BYTES);
    assert_eq!(base % bytes, 0);
    let physical_root = arch::virt_to_phys(root);
    assert!(base <= physical_root && physical_root < base + bytes);
    // Step 2: 发布栈基址、入口栈帧地址与任务 ID 供协调者核对，随后进入 `DEPTH` 层递归上下文切换。
    state.bases[slot].store(base, Ordering::Release);
    state.stacks[slot].store(root, Ordering::Release);
    state.ids[slot].store(task::current_task().id().as_usize(), Ordering::Release);
    assert_eq!(deep_calls(&state, &token, slot, DEPTH, root)?, DEPTH + 1);
    Ok(20 + slot as i32)
}

/// 校验指定内核栈物理基址对应的全部 16 个物理页帧均位于物理内存可分配范围内，且未被 SLUB 分配器占用。
fn check_stack_pages(base: usize) {
    for pfn in base / mm::PAGE_SIZE..base / mm::PAGE_SIZE + STACK_BYTES / mm::PAGE_SIZE {
        let descriptor = mm::pages()
            .get(pfn)
            .expect("task stack has no page descriptor");
        assert!(mm::pages()
            .free_ranges()
            .iter()
            .any(|range| range.start <= pfn && pfn < range.end));
        assert_eq!(descriptor.slab_head.load(Ordering::Acquire), usize::MAX);
    }
}

/// 计算当前 Buddy 空闲页、每核 PCP 缓存页与 SLUB 后备页三者之和，用于观测内核栈分配与释放前后的 4 KiB 物理页差值。
fn page_total() -> usize {
    let stats = mm::stats();
    stats.buddy_free_pages + stats.pcp_cached_pages + stats.slab_backing_pages
}

/// 在不调用 `sched::yield_now()` 的前提下自旋等待条件成立，避免等待辅助函数自身触发额外的调度通知干扰被测 CPU。
///
/// 等待上限为 2 秒单调时钟，若收到协作停止请求或超时则返回 `TestError`。
fn notification_until(
    state: &Control,
    token: &ChildToken,
    mut ready: impl FnMut() -> bool,
    reason: &'static str,
) -> TestResult {
    let end = crate::time::now()
        .checked_add(crate::time::Duration::from_secs(2))
        .ok_or(TestError("notification deadline overflow"))?;
    loop {
        if state.stop.load(Ordering::Acquire) || token.stop_requested() {
            return Err(TestError("notification participant stopped"));
        }
        if ready() {
            return Ok(());
        }
        if end <= crate::time::now() {
            return Err(TestError(reason));
        }
        spin_loop();
    }
}

/// 从测试控制块克隆并返回指定槽位已发布的 `Arc<Task>` 强引用（锁外持有，便于后续查询调度器状态）。
fn notification_task(state: &Control, slot: usize) -> Arc<task::Task> {
    state.notification_tasks[slot]
        .lock()
        .as_ref()
        .cloned()
        .expect("notification Task not published")
}

/// 跨核通知测试中的主动让出线程：迁移至 CPU 0 发布自身任务引用，待协调者放行后调用 `sched::yield_now()` 并验证恢复执行后的状态。
///
/// 当 `idle` 为 `true` 时，先将亲和性限制在前两个逻辑核（CPU 0 与 CPU 1），验证 CPU 0 被其他就绪线程接管时当前线程会被空闲的 CPU 1 唤醒执行。
fn notification_yielder(
    state: Arc<Control>,
    token: ChildToken,
    idle: bool,
) -> Result<i32, TestError> {
    // Step 1: 按场景设置 CPU 亲和性并迁移到 CPU 0，确认中断开启后发布当前 `Task` 强引用并通告就绪。
    if idle {
        sched::set_affinity(&task::current_task(), cpu::CpuSet::prefix(2));
    }
    sched::test_migrate_to(0);
    assert_eq!(cpu::current().id, 0);
    assert!(arch::irq_enabled());
    assert!(state.notification_tasks[0]
        .lock()
        .replace(task::current_task())
        .is_none());
    state.starts[0].store(1, Ordering::Release);
    // Step 2: 自旋等待协调者打开 `go` 门控后执行一次 `sched::yield_now()`。
    notification_until(
        &state,
        &token,
        || state.go.load(Ordering::Acquire),
        "notification Yield gate",
    )?;
    sched::yield_now();
    // Step 3: 校验恢复执行后的中断开启状态与所在逻辑核，保持线程存活直到协调者完成全部断言并置位 `frames` 放行。
    assert!(arch::irq_enabled());
    if idle {
        assert_eq!(cpu::current().id, 1);
    }
    state.steps[0].store(1, Ordering::Release);
    notification_until(
        &state,
        &token,
        || state.frames.load(Ordering::Acquire) == 1,
        "notification Yield release",
    )?;
    Ok(20)
}

/// 供 `cpu::call` 使用的空跨核同步调用（CSD）回调，利用 IPI 处理顺序充当跨核栅栏，确保目标 CPU 已排空此前挂起的软件通知位。
///
/// # Safety
/// 函数体为空且不解引用入参指针，在任意关中断 IPI 上下文中调用均安全。
unsafe fn notification_barrier(_: *mut ()) {}

#[expect(
    clippy::module_inception,
    reason = "具名 suite 的 context 层级属于现有测试目录，外层文件模块承载同组 fixture"
)]
#[::ktest::suite(fixture = Tasks, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "context"), timeout_ms = 15000)]
pub(crate) mod context {
    use super::*;

    /// 验证同核两个内核线程在深层递归栈帧上交替上下文切换时寄存器与栈数据无串扰，且退出后完整释放两个 64 KiB 内核栈。
    ///
    /// - **测试目标**：检验汇编上下文切换对被调用者保存寄存器及多层活跃栈帧的隔离性，以及线程离栈退出后的栈物理页归还账务。
    /// - **场景步骤**：在 CPU 0 上派生两个工作线程，各自递归进入 `DEPTH` 层栈帧并在 `arrived` 栅栏汇合；核对两者任务 ID 与互不重叠的 64 KiB 内核栈区间后记录物理页总量 `live`，再打开 `go` 门控让两线程严格交替 `yield_now()` 共 `ROUNDS` 轮直至返回。
    /// - **预期结果**：两线程分别返回 `20` 与 `21`，栈上 `LocalDrop` 各析构 1 次，累计校验 `2 * (DEPTH + 1)` 层栈帧未损坏，且线程退出后物理页总量精确增加 32 页（2 个 64 KiB 栈）。
    #[test]
    fn deep_switch_return(_: &mut TestContext<'_>, fixture: &mut Tasks) -> TestResult {
        // Step 1: 在 CPU 0 上派生两个工作线程，等待二者各自递归进入最深层栈帧。
        sched::test_migrate_to(0);
        for slot in 0..2 {
            let state = Arc::clone(&fixture.state);
            fixture.handles[slot] = Some(spawn_result_on(0, move |token| {
                switch_worker(state, token, slot)
            })?);
        }
        let state = &fixture.state;
        until(
            || state.arrived.load(Ordering::Acquire) == 2,
            "task-deep-arrival",
        )?;
        // Step 2: 校验两线程拥有不同的任务 ID、互不重叠的 64 KiB 独立内核栈以及合法的页帧描述符。
        let stacks =
            core::array::from_fn::<_, 2, _>(|slot| state.stacks[slot].load(Ordering::Acquire));
        let bases =
            core::array::from_fn::<_, 2, _>(|slot| state.bases[slot].load(Ordering::Acquire));
        let ids = core::array::from_fn::<_, 2, _>(|slot| state.ids[slot].load(Ordering::Acquire));
        assert_ne!(ids[0], ids[1]);
        for (slot, id) in ids.iter().enumerate() {
            assert_eq!(
                fixture.handles[slot].as_ref().unwrap().task_id().as_usize(),
                *id
            );
        }
        assert!(
            STACK_BYTES
                <= (stacks[0] & !(STACK_BYTES - 1)).abs_diff(stacks[1] & !(STACK_BYTES - 1))
        );
        assert!(STACK_BYTES <= bases[0].abs_diff(bases[1]));
        bases.iter().copied().for_each(check_stack_pages);

        // Step 3: 记录两线程在栈运行时的物理页总量，打开启动门控并等待两线程交替切换完毕后正常退出离栈。
        let live = page_total();
        state.go.store(true, Ordering::Release);
        until(
            || {
                fixture
                    .handles
                    .iter()
                    .flatten()
                    .all(|handle| handle.try_join().is_some())
            },
            "task-return",
        )?;
        // Step 4: 校验退出码、切换轮次、栈上局部守卫析构次数、总弹出栈帧数以及释放归还的 32 个物理页。
        assert_eq!(fixture.handles[0].as_ref().unwrap().try_join(), Some(20));
        assert_eq!(fixture.handles[1].as_ref().unwrap().try_join(), Some(21));
        assert_eq!(page_total(), live + 32);
        for slot in 0..2 {
            assert_eq!(state.starts[slot].load(Ordering::Acquire), 1);
            assert_eq!(state.steps[slot].load(Ordering::Acquire), ROUNDS);
            assert_eq!(state.active[slot].load(Ordering::Acquire), 0);
            assert_eq!(state.drops[slot].load(Ordering::Acquire), 1);
        }
        assert_eq!(state.frames.load(Ordering::Acquire), 2 * (DEPTH + 1));
        bases.iter().copied().for_each(check_stack_pages);
        Ok(())
    }

    /// 验证外部仍持有 `Arc<Task>` 与 `ChildHandle` 观察句柄时，已退出线程的 64 KiB 内核栈先于任务控制块被立即释放。
    ///
    /// - **测试目标**：确认内核线程退出离栈后立即归还 16 页内核栈物理内存，而退出码与 `TaskState::Exited` 状态保存在轻量的任务控制块与 `JoinState` 中。
    /// - **场景步骤**：在 CPU 0 上派生工作线程，将其 `Arc<Task>` 存入 `external` 并阻塞在 `exit` 完成量上；记录当前页总量 `live` 后触发完成量放行线程返回 `1..=8` 求和值 `36`，等待 `try_join` 可见但暂不释放 `external` 与 `handle`。
    /// - **预期结果**：此时 `external` 状态为 `Exited` 且强引用计数为 1，栈上 `LocalDrop` 已析构，物理页总量比阻塞期间增加 16 页（1 个 64 KiB 栈），随后 `join()` 仍可正常读取返回值 `36`。
    #[test]
    fn join_exit_observers(_: &mut TestContext<'_>, fixture: &mut Tasks) -> TestResult {
        // Step 1: 在 CPU 0 上派生子线程，令其发布自身 `Arc<Task>` 强引用并阻塞在 `exit` 完成量上。
        sched::test_migrate_to(0);
        let state = Arc::clone(&fixture.state);
        fixture.handles[0] = Some(spawn_result_on(0, move |_| {
            let _local = LocalDrop {
                state: Arc::clone(&state),
                slot: 0,
            };
            assert!(state
                .external
                .lock()
                .replace(task::current_task())
                .is_none());
            state.arrived.store(1, Ordering::Release);
            state.exit.wait();
            Ok((1..=8).sum())
        })?);
        until(
            || {
                fixture.state.arrived.load(Ordering::Acquire) == 1
                    && fixture
                        .state
                        .external
                        .lock()
                        .as_ref()
                        .is_some_and(|task| sched::test_state(task) == task::State::Blocked)
            },
            "exit-worker-blocked",
        )?;
        // Step 2: 记录子线程持栈阻塞时的物理页总量，触发完成量唤醒子线程并等待其退出离栈。
        let live = page_total();
        fixture.state.exit.complete();
        let handle = fixture.handles[0].as_ref().unwrap();
        until(|| handle.try_join().is_some(), "wait-exit-result")?;
        // Step 3: 在仍持有外部 `Arc<Task>` 与 `ChildHandle` 的状态下，断言 16 页内核栈已归还且返回值 36 可正常读取。
        let external = fixture
            .state
            .external
            .lock()
            .take()
            .expect("exit task Arc not retained");
        assert_eq!(sched::test_state(&external), task::State::Exited);
        assert_eq!(Arc::strong_count(&external), 1);
        assert_eq!(
            Arc::strong_count(&external) + handle.try_join().is_some() as usize,
            2
        );
        assert_eq!(fixture.state.drops[0].load(Ordering::Acquire), 1);
        assert_eq!(page_total().checked_sub(live), Some(16));
        assert_eq!(handle.join()?, (1..=8).sum::<i32>());
        drop(external);
        Ok(())
    }
}

#[::ktest::suite(fixture = Tasks, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "migration"), min_cpus = 2, timeout_ms = 15000)]
pub(crate) mod migration {
    use super::*;

    /// 验证单个任意核亲和性（Any）内核线程遍历全部在线逻辑 CPU 往返迁移两轮时，任务身份、Per-CPU 指针与内核栈局部变量地址始终一致。
    ///
    /// - **测试目标**：检验跨核上下文迁移后硬件寄存器绑定的 `CpuLocal` 基址随目标核实时更新，同时线程自身的 `TaskId` 与内核栈帧虚拟地址保持不变。
    /// - **场景步骤**：派生一个子线程记录栈上局部变量 `local` 的地址与初始 `TaskId`，依次调用 `sched::test_migrate_to` 走遍 `0..cpu::count()` 两整轮，每跳核对 `cpu::current().id`、`arch::cpu_local()`、任务 ID 及栈变量内容。
    /// - **预期结果**：全部跨核跳转断言通过，子线程离栈后 `try_join` 与 `join` 均返回 `31`，句柄记录的任务 ID 与线程内读取值一致。
    #[test]
    fn same_task_round_trip(_: &mut TestContext<'_>, fixture: &mut Tasks) -> TestResult {
        let state = Arc::clone(&fixture.state);
        fixture.handles[0] = Some(spawn_result(move |token| {
            let local = 0x7301usize;
            let stack = black_box(&local) as *const _ as usize;
            let id = task::current_task().id().as_usize();
            state.ids[0].store(id, Ordering::Release);
            for hop in 0..=2 * cpu::count() {
                if state.stop.load(Ordering::Acquire) || token.stop_requested() {
                    return Ok(0);
                }
                let target = hop % cpu::count();
                sched::test_migrate_to(target);
                assert_eq!(cpu::current().id, target);
                assert_eq!(arch::cpu_local(), cpu::get(target) as *const _);
                assert_eq!(cpu::current() as *const _, arch::cpu_local());
                assert_eq!(task::current_task().id().as_usize(), id);
                assert_eq!(task::current_task_id().map(|id| id.as_usize()), Some(id));
                assert_eq!(black_box(&local) as *const _ as usize, stack);
                assert_eq!(*black_box(&local), 0x7301);
            }
            Ok(31)
        })?);
        let handle = fixture.handles[0].as_ref().unwrap();
        until(|| handle.try_join().is_some(), "task-migration-return")?;
        assert_eq!(handle.try_join(), Some(31));
        assert_eq!(handle.join()?, 31);
        assert_eq!(
            handle.task_id().as_usize(),
            fixture.state.ids[0].load(Ordering::Acquire)
        );
        Ok(())
    }

    /// 验证当远端 CPU 处于非空闲运行状态时，其他核上线程调用 `sched::yield_now()` 不会向该繁忙核发送冗余的 `KICK_RESCHEDULE` 抢占通知。
    ///
    /// - **测试目标**：检验 `yield_now()` 的跨核通知抑制策略——主动让出仅在存在空闲核可接管时唤醒空闲核，绝不打扰正在执行其他任务的繁忙 CPU。
    /// - **场景步骤**：在 CPU 2 上启动协调线程，让线程 A 在 CPU 0 就绪等待 `go` 门控，让线程 B 固定在 CPU 1 运行；协调线程先通过 `cpu::call` 排空 CPU 1 既有 IPI，再通知线程 B 持 `IrqSaveGuard` 关中断自旋以冻结 CPU 1 的 `kick_reasons` 位掩码；随后放行线程 A 在 CPU 0 执行 `yield_now()` 并等待其恢复。
    /// - **预期结果**：线程 A 从 `yield_now()` 恢复运行且就绪队列残留计数为 0，线程 B 仍在 CPU 1 运行，且 CPU 1 的挂起软件通知位中 `KICK_RESCHEDULE` 始终为 0。
    #[test]
    #[ktest(min_cpus = 3)]
    fn yield_does_not_reschedule_busy_cpu(
        _: &mut TestContext<'_>,
        fixture: &mut Tasks,
    ) -> TestResult {
        let state = Arc::clone(&fixture.state);
        // Step 1: 在 CPU 2 启动协调者，安排线程 A 停在 CPU 0 的 yield 门前，安排线程 B 停在 CPU 1 的关中断门前。
        fixture.handles[0] = Some(spawn_result_on(2, move |token| {
            assert_eq!(cpu::current().id, 2);
            let a_state = Arc::clone(&state);
            let a = spawn_result(move |token| notification_yielder(a_state, token, false))?;
            notification_until(
                &state,
                &token,
                || state.starts[0].load(Ordering::Acquire) == 1,
                "busy notification A arrival",
            )?;

            let b_state = Arc::clone(&state);
            let b = spawn_result_on(1, move |token| {
                assert_eq!(cpu::current().id, 1);
                assert!(arch::irq_enabled());
                assert!(b_state.notification_tasks[1]
                    .lock()
                    .replace(task::current_task())
                    .is_none());
                b_state.starts[1].store(1, Ordering::Release);
                notification_until(
                    &b_state,
                    &token,
                    || b_state.turn.load(Ordering::Acquire) == 1,
                    "busy notification IRQ gate",
                )?;
                let irq = IrqSaveGuard::new();
                b_state.steps[1].store(1, Ordering::Release);
                let held = notification_until(
                    &b_state,
                    &token,
                    || b_state.frames.load(Ordering::Acquire) == 1,
                    "busy notification IRQ release",
                );
                drop(irq);
                held?;
                Ok(21)
            })?;
            notification_until(
                &state,
                &token,
                || state.starts[1].load(Ordering::Acquire) == 1,
                "busy notification B arrival",
            )?;

            // Step 2: 先在 CPU 1 上同步执行一次空 CSD 回调排空历史挂起通知，随后令线程 B 关闭 CPU 1 本地中断以捕获后续所有跨核通知位。
            let a_task = notification_task(&state, 0);
            let b_task = notification_task(&state, 1);
            assert_eq!(sched::test_state(&a_task), task::State::Running(0));
            assert_eq!(sched::test_state(&b_task), task::State::Running(1));
            // SAFETY:
            // 1. 目标核 `1` 在 `min_cpus = 3` 约束下有效且在线，此时线程 B 正处于开中断自旋等待段，能够立即响应 IPI；
            // 2. 协调线程运行在 CPU 2 上，本地中断开启且未持有任何锁，不会造成跨核等待死锁；
            // 3. `notification_barrier` 为空函数，不解引用空指针参数，也不执行任何睡眠或加锁操作。
            unsafe { cpu::call(1, notification_barrier, core::ptr::null_mut()) };
            state.turn.store(1, Ordering::Release);
            notification_until(
                &state,
                &token,
                || state.steps[1].load(Ordering::Acquire) == 1,
                "busy notification IRQ held",
            )?;

            // Step 3: 确认 CPU 1 处于繁忙且无 `KICK_RESCHEDULE` 状态后，放行 CPU 0 上的线程 A 执行 `yield_now()`，断言 CPU 1 未被置位抢占通知。
            assert!(!cpu::get(1).idle.load(Ordering::Acquire));
            assert_eq!(cpu::test_pending_kicks(1) & cpu::KICK_RESCHEDULE, 0);
            state.go.store(true, Ordering::Release);
            notification_until(
                &state,
                &token,
                || state.steps[0].load(Ordering::Acquire) == 1,
                "busy notification Yield returned",
            )?;
            assert!(matches!(
                sched::test_state(&a_task),
                task::State::Running(_)
            ));
            assert_eq!(sched::test_ready_occurrences(&a_task), 0);
            assert_eq!(sched::test_state(&b_task), task::State::Running(1));
            assert_eq!(cpu::test_pending_kicks(1) & cpu::KICK_RESCHEDULE, 0);
            assert_eq!(a.try_join(), None);
            assert_eq!(b.try_join(), None);

            // Step 4: 置位 `frames` 释放线程 A 与线程 B 恢复开中断并正常退出，回收两个子线程结果。
            state.frames.store(1, Ordering::Release);
            assert_eq!(a.join()?, 20);
            assert_eq!(b.join()?, 21);
            Ok(31)
        })?);
        assert_eq!(fixture.handles[0].as_ref().unwrap().join()?, 31);
        Ok(())
    }

    /// 验证当本地核有其他就绪线程接管而目标亲和集内存在空闲核时，`sched::yield_now()` 会向空闲核发送 `KICK_SCHED_WAKE` 唤醒通知且不附加 `KICK_RESCHEDULE`。
    ///
    /// - **测试目标**：检验主动让出后被重新放回就绪队列的线程能够通过 `KICK_SCHED_WAKE` 精确唤醒亲和范围内的空闲 CPU，并在该空闲核恢复执行。
    /// - **场景步骤**：在 CPU 2 上启动协调线程，先让单核亲和 CPU 0 的线程 B 在 CPU 0 阻塞，再让亲和 `{0, 1}` 的线程 A 占据 CPU 0；唤醒线程 B 使其进入 CPU 0 就绪队列，通过 `cpu::call` 排空 CPU 1 历史通知后武装 CPU 1 的 `TestPause::Idle` 停点并触发其停在 idle 循环边界；接着放行线程 A 调用 `yield_now()` 让出 CPU 0 给线程 B，观测此时挂在 CPU 1 上的通知位，最后释放 CPU 1 的 idle 停点。
    /// - **预期结果**：线程 A 让出后线程 B 在 CPU 0 运行，线程 A 处于 `Ready` 状态，CPU 1 挂起位包含 `KICK_SCHED_WAKE` 且不包含 `KICK_RESCHEDULE`；释放 CPU 1 停点后线程 A 在 CPU 1 恢复执行并正常返回。
    #[test]
    #[ktest(min_cpus = 3)]
    fn yield_wakes_idle_cpu_without_reschedule(
        _: &mut TestContext<'_>,
        fixture: &mut Tasks,
    ) -> TestResult {
        fixture.idle_pause = Some(1);
        let state = Arc::clone(&fixture.state);
        // Step 1: 在 CPU 2 启动协调者，安排固定在 CPU 0 的线程 B 先阻塞，再让亲和 CPU 0/1 的线程 A 在 CPU 0 就绪运行。
        fixture.handles[0] = Some(spawn_result_on(2, move |token| {
            assert_eq!(cpu::current().id, 2);
            let b_state = Arc::clone(&state);
            let b = spawn_result_on(0, move |token| {
                assert_eq!(cpu::current().id, 0);
                assert!(b_state.notification_tasks[1]
                    .lock()
                    .replace(task::current_task())
                    .is_none());
                b_state.starts[1].store(1, Ordering::Release);
                b_state.exit.wait();
                assert_eq!(cpu::current().id, 0);
                b_state.steps[1].store(1, Ordering::Release);
                notification_until(
                    &b_state,
                    &token,
                    || b_state.frames.load(Ordering::Acquire) == 1,
                    "idle notification B release",
                )?;
                Ok(21)
            })?;
            notification_until(
                &state,
                &token,
                || {
                    state.starts[1].load(Ordering::Acquire) == 1
                        && sched::test_state(&notification_task(&state, 1)) == task::State::Blocked
                },
                "idle notification B blocked",
            )?;

            let a_state = Arc::clone(&state);
            let a = spawn_result(move |token| notification_yielder(a_state, token, true))?;
            notification_until(
                &state,
                &token,
                || state.starts[0].load(Ordering::Acquire) == 1,
                "idle notification A arrival",
            )?;
            let a_task = notification_task(&state, 0);
            let b_task = notification_task(&state, 1);
            assert_eq!(sched::test_state(&a_task), task::State::Running(0));
            assert_eq!(sched::affinity(&a_task), cpu::CpuSet::prefix(2));
            assert_eq!(sched::affinity(&b_task), cpu::CpuSet::single(0));

            // Step 2: 唤醒线程 B 使其进入就绪态，清空 CPU 1 既有通知后武装并触发 CPU 1 停在 `TestPause::Idle` 观察点。
            state.exit.complete();
            assert_eq!(sched::test_state(&b_task), task::State::Ready);
            assert_eq!(sched::test_ready_occurrences(&b_task), 1);
            // SAFETY:
            // 1. 目标核 `1` 在 `min_cpus = 3` 约束下有效且在线，且此时尚未武装 `TestPause::Idle` 停点，可正常响应 IPI；
            // 2. 协调线程运行在 CPU 2 上，本地中断开启且未持有任何自旋锁；
            // 3. `notification_barrier` 为静态空函数，不访问空指针参数、不睡眠且不获取任何锁。
            unsafe { cpu::call(1, notification_barrier, core::ptr::null_mut()) };
            cpu::test_arm_pause(1, cpu::TestPause::Idle);
            // 停点武装完成后记录 `active[1] = 1`，以便用例中途失败时 `Tasks::quiesce` 能精确取消该停点。
            state.active[1].store(1, Ordering::Release);
            cpu::kick(1, cpu::KICK_TIMER_REPROGRAM);
            notification_until(
                &state,
                &token,
                || cpu::test_is_paused(1, cpu::TestPause::Idle),
                "idle notification boundary",
            )?;
            assert!(cpu::get(1).idle.load(Ordering::Acquire));
            assert_eq!(
                cpu::test_pending_kicks(1) & (cpu::KICK_SCHED_WAKE | cpu::KICK_RESCHEDULE),
                0
            );

            // Step 3: 放行线程 A 在 CPU 0 调用 `yield_now()`；等待线程 B 接管 CPU 0 后，断言暂停在 idle 边界的 CPU 1 仅收到 `KICK_SCHED_WAKE` 而无 `KICK_RESCHEDULE`。
            state.go.store(true, Ordering::Release);
            notification_until(
                &state,
                &token,
                || state.steps[1].load(Ordering::Acquire) == 1,
                "idle notification B running",
            )?;
            assert_eq!(sched::test_state(&b_task), task::State::Running(0));
            assert_eq!(sched::test_ready_occurrences(&b_task), 0);
            assert_eq!(sched::test_state(&a_task), task::State::Ready);
            assert_eq!(sched::test_ready_occurrences(&a_task), 1);
            assert_eq!(state.steps[0].load(Ordering::Acquire), 0);
            let pending = cpu::test_pending_kicks(1);
            assert_eq!(pending & cpu::KICK_SCHED_WAKE, cpu::KICK_SCHED_WAKE);
            assert_eq!(pending & cpu::KICK_RESCHEDULE, 0);

            // Step 4: 释放 CPU 1 的 idle 停点，验证线程 A 被唤醒并迁移至 CPU 1 恢复执行，最后放行两线程正常退出。
            cpu::test_release_pause(1, cpu::TestPause::Idle);
            state.active[1].store(0, Ordering::Release);
            notification_until(
                &state,
                &token,
                || state.steps[0].load(Ordering::Acquire) == 1,
                "idle notification A resumed",
            )?;
            assert_eq!(sched::test_state(&a_task), task::State::Running(1));
            assert_eq!(sched::test_ready_occurrences(&a_task), 0);
            assert_eq!(a.try_join(), None);

            state.frames.store(1, Ordering::Release);
            assert_eq!(a.join()?, 20);
            assert_eq!(b.join()?, 21);
            Ok(31)
        })?);
        assert_eq!(fixture.handles[0].as_ref().unwrap().join()?, 31);
        Ok(())
    }
}

#[::ktest::suite(tags("execution", "fatal"), isolation = "Case", timeout_ms = 5000)]
pub(crate) mod fatal {
    use super::*;

    /// 验证在持有 `IrqSaveGuard` 关闭本地中断期间调用 `sched::yield_now()` 会立即触发 `"irq-schedule"` 致命错误中止。
    ///
    /// - **测试目标**：检验调度入口对“禁止在关中断临界区内主动切换上下文”前置不变量的强校验。
    /// - **场景步骤**：在独立隔离的测试用例上下文中构造 `IrqSaveGuard` 关闭当前 CPU 本地中断，随后直接调用 `sched::yield_now()`。
    /// - **预期结果**：调度器前置检查拦截该违规调用并上报 `"irq-schedule"` 致命事件，执行流绝不会越过切换点到达后续 `panic!`。
    #[test]
    #[ktest(expected_fatal = "irq-schedule")]
    fn irq_guard_schedule() {
        assert!(task::current_task_id().is_some());
        let _guard = crate::sync::IrqSaveGuard::new();
        sched::yield_now();
        panic!("execution fault crossed prohibited switch boundary");
    }

    /// 验证在持有 `PreemptGuard` 禁用抢占期间调用 `sched::yield_now()` 会立即触发 `"preempt-schedule"` 致命错误中止。
    ///
    /// - **测试目标**：检验调度入口对“禁止在显式禁抢占临界区内睡眠或让出 CPU”前置不变量的强校验。
    /// - **场景步骤**：在独立隔离的测试用例上下文中构造 `PreemptGuard` 递增本核禁抢占计数，随后直接调用 `sched::yield_now()`。
    /// - **预期结果**：调度器前置检查拦截该违规调用并上报 `"preempt-schedule"` 致命事件，执行流绝不会继续向下执行。
    #[test]
    #[ktest(expected_fatal = "preempt-schedule")]
    fn preempt_guard_schedule() {
        assert!(task::current_task_id().is_some());
        let _guard = crate::sync::PreemptGuard::new();
        sched::yield_now();
        panic!("execution fault crossed prohibited switch boundary");
    }
}
