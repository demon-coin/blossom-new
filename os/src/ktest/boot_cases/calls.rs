//! 跨核同步函数调用（`cpu::call` / SMP IPI Call）的并发拓扑、生命周期与关中断语义自检。
//!
//! # 模块职责与架构定位
//! 本模块属于 `ktest` 线程级（`Thread` 上下文）测试套件，用于验证 `crate::cpu::call` 在本核自调用、
//! 双核成对互发（Pair）、环状转发（Ring）、多核扇入汇聚（Fan-in）、延迟回调以及回调处理期间新请求到达
//! 等场景下的正确性。
//! - **上游调用者**：`ktest` 线程级测试运行器。
//! - **下游被测模块**：`crate::cpu::call`、`crate::cpu::pending_calls` 跨核调用队列与 IPI 中断处理循环。
//!
//! # 核心机制与同步协议
//! ```text
//! 发送核 Task (Sender CPU)                        目标核 IPI/本地执行上下文 (Target CPU)
//! ────────────────────────                        ──────────────────────────────────────
//! 1. ready += 1，等待启动门 (go == true)
//! 2. 构造栈上 Payload (含 Arc<Control>)
//! 3. 调用 unsafe { cpu::call(target, cb, &mut payload) }
//!      │
//!      ├─ 入队请求并触发目标核 IPI ──────────────► 4. 关中断 (IRQ disabled) 取出请求并调用 cb(&mut payload)
//!      │                                               ├─ 校验当前核 == target 且中断已关闭
//!      ├─ 自旋等待期间保持可响应远端 IPI               ├─ (可选 delayed) 忙等观察队列到达或校验发送方未提前返回
//!      │                                               └─ 写入 payload.result = id + 0x100，标记完成
//!      ◄─ Release/Acquire 同步完成信号 ───────────────┘
//! 5. 标记 returned[sender] = true
//! 6. 校验本核中断仍开启且 payload.result 正确
//! ```
//!
//! # 锁获取顺序与并发契约
//! - **无锁死锁预防**：发起 `cpu::call` 的线程仅持有 `PreemptGuard` 以固定在发送核上，绝不持有任何会被回调函数获取的锁，从而在多核成对互发（`pair`）或环状互发（`ring`）时允许发送方在等待期间处理来自对方的 IPI 请求。
//! - **栈参数生命周期契约**：每个发送任务在自己的内核栈上独立构造 `Payload` 并克隆 `Arc<Control>`（不借用父夹具栈）。`cpu::call` 严格阻塞至目标核回调执行完毕才返回，因此回调执行期间对 `Payload` 的可变引用始终独占且有效。

use crate::ktest::boot_cases::hardware::wait;
use crate::ktest::{spawn_on, ChildHandle};
use crate::sync::PreemptGuard;
use crate::{arch, boot, cpu};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// 跨核同步调用测试的共享控制门与原子观测账本。
struct Control {
    /// 并发起跑门：置为 `true` 后所有就绪的发送任务同时发起 `cpu::call`。
    go: AtomicBool,
    /// 协作停止标志：测试收尾或超时退出时通知自旋等待循环提前终止。
    stop: AtomicBool,
    /// 已到达起跑门前等待的发送任务计数。
    ready: AtomicUsize,
    /// 标记 `delayed` 回调函数是否已在目标核上开始执行。
    entered: AtomicBool,
    /// 标记在 `delayed` 回调驻留期间，目标核待处理队列中是否如期积压了全部后续到达的请求。
    queued: AtomicBool,
    /// 标记回调执行期间是否检测到任何不变量违规（如中断未关闭、运行在错误的 CPU、发送方提前返回或等待超时）。
    fault: AtomicBool,
    /// 记录实际执行过回调函数的目标 CPU 集合位图。
    executed: cpu::AtomicCpuSet,
    /// 按发送核编号索引的回调执行次数统计（每个发送核预期恰好为 1 次）。
    hits: [AtomicUsize; boot::MAX_CPUS],
    /// 按发送核编号索引的 `cpu::call` 返回状态标志，用于校验发送方绝不会在回调结束前提前返回。
    returned: [AtomicBool; boot::MAX_CPUS],
    /// 按发送核编号索引的回调计算结果记录（初始为 `usize::MAX`，完成后为 `target + 0x100`）。
    results: [AtomicUsize; boot::MAX_CPUS],
}

/// 跨核同步调用测试夹具，管理共享控制块与各 CPU 上的发送子任务句柄。
struct Calls {
    /// 供主测试线程与所有发送子任务共享的控制与观测状态。
    state: Arc<Control>,
    /// 按发送核逻辑 ID 存放的子任务句柄数组。
    handles: [Option<ChildHandle>; boot::MAX_CPUS],
}

impl Fixture for Calls {
    /// 在发起任何跨核调用前预分配控制块与任务句柄槽位。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let state = Arc::try_new(Control {
            go: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            ready: AtomicUsize::new(0),
            entered: AtomicBool::new(false),
            queued: AtomicBool::new(false),
            fault: AtomicBool::new(false),
            executed: cpu::AtomicCpuSet::new(),
            hits: core::array::from_fn(|_| AtomicUsize::new(0)),
            returned: core::array::from_fn(|_| AtomicBool::new(false)),
            results: core::array::from_fn(|_| AtomicUsize::new(usize::MAX)),
        })
        .map_err(|_| TestError("NoMemory: synchronous call controls"))?;
        Ok(Self {
            state,
            handles: core::array::from_fn(|_| None),
        })
    }

    /// 置位协作停止标志与起跑门，唤醒所有尚在等待门前的子任务，使已发出的同步调用能够正常跑完返回。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.state.stop.store(true, Ordering::Release);
        self.state.go.store(true, Ordering::Release);
        Ok(())
    }

    /// 在测试框架回收全部子任务后，断言所有保存的任务句柄均已完成退出。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        assert!(self
            .handles
            .iter()
            .flatten()
            .all(|h| h.try_join().is_some()));
        Ok(())
    }
}

/// 单个发送任务在自身内核栈上持有的同步调用上下文参数。
struct Payload {
    /// 指向共享控制与观测状态的强引用。
    state: Arc<Control>,
    /// 发起本次调用的源 CPU 逻辑编号。
    sender: usize,
    /// 目标执行回调的 CPU 逻辑编号。
    target: usize,
    /// 由目标核回调函数写入的返回结果（预期写入 `target + 0x100`）。
    result: usize,
    /// 是否在回调执行期间等待并验证其他核心发往同一目标核的后续请求已进入待处理队列。
    arrival: bool,
}

/// 基础跨核回调函数：校验目标核身份与关中断环境，记录执行次数并将结果写回发送方的 `Payload`。
///
/// # Safety
/// 1. `argument` 必须指向由 `cpu::call` 调用方栈上持有且对齐良好的有效 `Payload` 实例；
/// 2. 在 `cpu::call` 完成 Release/Acquire 同步返回前，调用方不得销毁或并发访问该 `Payload`；
/// 3. 回调函数运行在关中断上下文中，内部不得睡眠或递归发起会导致死锁的同步等待。
unsafe fn identity(argument: *mut ()) {
    // SAFETY:
    // 1. 调用方 `Calls::sender` 将栈上局部变量 `payload: Payload` 的可变指针传入同步 `cpu::call`；
    // 2. `cpu::call` 在本回调执行完毕并发布完成标志前不会返回到发送方，因此当前核在回调期间独占该 `Payload`。
    let payload = unsafe { &mut *argument.cast::<Payload>() };
    if arch::irq_enabled() || cpu::current().id != payload.target {
        payload.state.fault.store(true, Ordering::Release);
    }
    payload.state.hits[payload.sender].fetch_add(1, Ordering::Relaxed);
    payload
        .state
        .executed
        .insert(cpu::current().id, Ordering::Release);
    payload.result = cpu::current().id + 0x100;
}

/// 带延迟驻留与队列积压观测的跨核回调函数：验证回调执行期间发送方不会提前从 `cpu::call` 返回，
/// 并在 `arrival` 置位时验证目标核在处理当前回调期间能收到后续排队的请求。
///
/// # Safety
/// 1. `argument` 必须满足 [`identity`] 的全部指针有效性、独占访问与生命周期前提；
/// 2. `Payload::state` 强引用必须在整个回调执行期间保持有效。
unsafe fn delayed(argument: *mut ()) {
    // SAFETY:
    // 1. `argument` 来自 `Calls::sender` 中栈上 `Payload` 的可变引用转换；
    // 2. 当前处于 `cpu::call` 回调执行期内，完成标志尚未置位，发送方阻塞等待，故此处独占借用安全。
    let payload = unsafe { &mut *argument.cast::<Payload>() };
    // Step 1: 标记已进入延迟回调；若启用了 arrival 检查，则自旋等待其余跟随者请求全部进入目标核待处理队列。
    payload.state.entered.store(true, Ordering::Release);
    if payload.arrival {
        let end = arch::read_counter() + arch::counter_frequency() * 5;
        while cpu::pending_calls(payload.target) != cpu::count() - 2 {
            if payload.state.stop.load(Ordering::Acquire) {
                break;
            }
            if end <= arch::read_counter() {
                payload.state.fault.store(true, Ordering::Release);
                break;
            }
            core::hint::spin_loop();
        }
        payload.state.queued.store(
            cpu::pending_calls(payload.target) == cpu::count() - 2,
            Ordering::Release,
        );
    }
    // Step 2: 忙等约 2ms，在此期间持续检查发送方是否违反同步契约提前从 cpu::call 返回。
    let end = arch::read_counter() + (arch::counter_frequency() / 500).max(1);
    while arch::read_counter() < end {
        if payload.state.returned[payload.sender].load(Ordering::Acquire) {
            payload.state.fault.store(true, Ordering::Release);
        }
        core::hint::spin_loop();
    }
    // Step 3: 委托 identity 完成目标核身份校验、计数更新与结果写回，并在返回前再次确认发送方尚未提前返回。
    // SAFETY: 当前仍处于同一次 `cpu::call` 回调内且未发布完成状态，`argument` 依然指向有效的独占 `Payload`。
    unsafe { identity(argument) };
    // SAFETY: `identity` 已释放其临时可变引用，且 `cpu::call` 仍未发布完成标志，此处创建共享引用读取状态是安全的。
    let payload = unsafe { &*argument.cast::<Payload>() };
    if payload.state.returned[payload.sender].load(Ordering::Acquire) {
        payload.state.fault.store(true, Ordering::Release);
    }
}

impl Calls {
    /// 在指定的 `sender` CPU 上启动一个绑定核子任务，等待起跑信号后向 `target` CPU 发起一次同步 `cpu::call`。
    fn sender(
        &mut self,
        sender: usize,
        target: usize,
        delay: bool,
        arrival: bool,
        follower: bool,
    ) -> TestResult {
        let state = Arc::clone(&self.state);
        self.handles[sender] = Some(spawn_on(sender, move |token| {
            // Step 1: 登记就绪并等待统一并发起跑门（若为跟随者则额外等待首个延迟回调进入执行态）。
            state.ready.fetch_add(1, Ordering::Release);
            while !state.go.load(Ordering::Acquire) {
                if token.stop_requested() || state.stop.load(Ordering::Acquire) {
                    return Ok(());
                }
                crate::sched::yield_now();
            }
            if follower {
                while !state.entered.load(Ordering::Acquire) {
                    if token.stop_requested() || state.stop.load(Ordering::Acquire) {
                        return Ok(());
                    }
                    crate::sched::yield_now();
                }
            }
            if token.stop_requested() || state.stop.load(Ordering::Acquire) {
                return Ok(());
            }

            // Step 2: 禁用抢占以锁定在发送核上，在当前任务栈上构造 Payload 并发起同步跨核调用。
            let _preempt = PreemptGuard::new();
            assert_eq!(cpu::current().id, sender);
            let mut payload = Payload {
                state: Arc::clone(&state),
                sender,
                target,
                result: usize::MAX,
                arrival,
            };
            // SAFETY:
            // 1. `target` 为在线 CPU 编号（调用方测试用例在 `0..cpu::count()` 范围内选取）；
            // 2. 当前线程仅持有 `PreemptGuard` 且开启中断，未持有回调函数所需的任何锁，等待期间可正常响应 IPI；
            // 3. `payload` 位于当前任务栈上，`cpu::call` 会阻塞到目标核回调彻底执行完毕才返回，保证回调期间指针始终有效。
            unsafe {
                cpu::call(
                    target,
                    if delay { delayed } else { identity },
                    (&mut payload as *mut Payload).cast(),
                )
            };
            // Step 3: 记录调用返回，校验本核中断状态已恢复且回调写入的计算结果与执行次数完全正确。
            state.returned[sender].store(true, Ordering::Release);
            assert!(arch::irq_enabled());
            assert_eq!(payload.result, target + 0x100);
            assert_eq!(state.hits[sender].load(Ordering::Acquire), 1);
            state.results[sender].store(payload.result, Ordering::Release);
            Ok(())
        })?);
        Ok(())
    }

    /// 等待 `count` 个发送任务全部到达起跑门后打开 `go` 标志，使所有发送者同时发起跨核调用。
    fn release(&self, count: usize) -> TestResult {
        wait(
            || self.state.ready.load(Ordering::Acquire) == count,
            "call sender did not reach gate",
        )?;
        self.state.go.store(true, Ordering::Release);
        Ok(())
    }

    /// 等待指定 `sender` 核上的子任务执行完毕，并校验其观测到的返回值、回调命中次数及无违规标志。
    fn finish(&self, context: &mut TestContext<'_>, sender: usize, target: usize) -> TestResult {
        wait(
            || self.handles[sender].as_ref().unwrap().try_join().is_some(),
            "synchronous call did not complete",
        )?;

        context.expect_eq(
            self.state.results[sender].load(Ordering::Acquire),
            target + 0x100,
        );
        context.expect_eq(self.state.hits[sender].load(Ordering::Acquire), 1);
        assert!(self.state.returned[sender].load(Ordering::Acquire));
        assert!(
            !self.state.fault.load(Ordering::Acquire),
            "callback IRQ/lifetime/arrival invariant failed"
        );
        Ok(())
    }
}

/// 计算成对互发测试中 `id` 号核心的目标核编号：相邻偶奇核互配（`id ^ 1`）；若总核数为奇数，最后一个核发往 0 号核（BSP）。
fn pair_target(id: usize) -> usize {
    let paired = id ^ 1;
    if paired < cpu::count() {
        paired
    } else {
        0
    }
}

#[::ktest::suite(context = "Thread", fixture = Calls, tags("boot", "smp-call"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// 测试目标：
    /// 验证每个在线 CPU 向自身发起同步 `cpu::call(id, id)` 时，回调在关中断状态下执行且返回后原核中断保持开启。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：在所有在线 CPU（`0..cpu::count()`）上各启动一个目标为自身的发送任务；
    /// 2. When：打开起跑门让所有核心并发执行本核 `cpu::call`；
    /// 3. Then：所有核心均精确执行 1 次回调并返回 `id + 0x100`，`executed` 位图等于当前全体在线 CPU 集合。
    #[test]
    fn self_call(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 0..cpu::count() {
            fixture.sender(id, id, false, false, false)?;
        }

        fixture.release(cpu::count())?;
        for id in 0..cpu::count() {
            fixture.finish(context, id, id)?;
        }
        context.expect_eq(
            fixture.state.executed.load(Ordering::Acquire),
            cpu::actual_set(),
        );
        Ok(())
    }

    /// 测试目标：
    /// 验证多核成对同时互相发起同步 `cpu::call` 时，等待中的发送核仍能响应对方的 IPI 请求而不会发生互锁死锁。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：在每个在线 CPU 上启动发送任务，目标设为配对核 `pair_target(id)`；
    /// 2. When：同时放行所有发送任务，令成对核心同时处于等待对方完成与服务对方 IPI 的交错状态；
    /// 3. Then：所有请求均顺利返回且无死锁，目标核执行集合覆盖所有被配对的目标 CPU。
    #[test]
    #[ktest(min_cpus = 2)]
    fn pair(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 0..cpu::count() {
            fixture.sender(id, pair_target(id), false, false, false)?;
        }

        fixture.release(cpu::count())?;
        for id in 0..cpu::count() {
            fixture.finish(context, id, pair_target(id))?;
        }
        let targets = if cpu::count() % 2 == 0 {
            cpu::actual_set()
        } else {
            cpu::CpuSet::prefix(cpu::count() - 1)
        };
        context.expect_eq(fixture.state.executed.load(Ordering::Acquire), targets);
        Ok(())
    }

    /// 测试目标：
    /// 验证所有在线 CPU 按环状拓扑（`id -> (id + 1) % N`）同时发起远程同步调用时，每个逻辑核均能正确处理入站 IPI 并完成出站等待。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：为每个 CPU `id` 配置发往 `(id + 1) % cpu::count()` 的发送任务；
    /// 2. When：统一放行起跑门，触发全环并发跨核调用；
    /// 3. Then：所有发送任务均收到来自下一核的计算结果，`executed` 位图覆盖全部在线 CPU。
    #[test]
    #[ktest(min_cpus = 2)]
    fn ring(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 0..cpu::count() {
            fixture.sender(id, (id + 1) % cpu::count(), false, false, false)?;
        }

        fixture.release(cpu::count())?;
        for id in 0..cpu::count() {
            fixture.finish(context, id, (id + 1) % cpu::count())?;
        }
        context.expect_eq(
            fixture.state.executed.load(Ordering::Acquire),
            cpu::actual_set(),
        );
        Ok(())
    }

    /// 测试目标：
    /// 验证所有从核（AP，`1..cpu::count()`）同时向主核（BSP，CPU 0）扇入发起同步调用时，主核队列不会丢失请求且各发送方独立收到正确结果。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：在 `1..cpu::count()` 的所有 AP 上启动目标为 CPU 0 的发送任务；
    /// 2. When：同时放行全部 `cpu::count() - 1` 个发送任务并发冲击 CPU 0；
    /// 3. Then：所有 AP 均精确完成 1 次调用并读回 `0x100`，期间无任何并发违规。
    #[test]
    #[ktest(min_cpus = 2)]
    fn fan_in(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 1..cpu::count() {
            fixture.sender(id, 0, false, false, false)?;
        }

        fixture.release(cpu::count() - 1)?;
        for id in 1..cpu::count() {
            fixture.finish(context, id, 0)?;
        }
        Ok(())
    }

    /// 测试目标：
    /// 验证本核自调用在回调函数主动忙等延迟期间，同样严格遵守同步等待语义且不破坏中断状态。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：在 CPU 0 上配置目标为 CPU 0 且启用 `delayed` 回调的发送任务；
    /// 2. When：放行任务执行带延迟的自调用；
    /// 3. Then：`entered` 标志被置位，调用在延迟回调彻底完成后正常返回且无违规标志。
    #[test]
    fn delayed_local(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        fixture.sender(0, 0, true, false, false)?;
        fixture.release(1)?;
        fixture.finish(context, 0, 0)?;
        assert!(fixture.state.entered.load(Ordering::Acquire));
        Ok(())
    }

    /// 测试目标：
    /// 验证跨核远程调用在目标核回调函数忙等延迟期间，发送核严格阻塞等待，绝不会在回调完成前提前返回并释放栈上参数。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：在 CPU 1 上配置发往 CPU 0 且启用 `delayed` 回调的远程发送任务；
    /// 2. When：放行任务，CPU 0 在 `delayed` 回调中忙等并持续断言 `returned[1] == false`；
    /// 3. Then：CPU 1 仅在 CPU 0 完成 `delayed` 回调后才返回，`fault` 保持为 `false`。
    #[test]
    #[ktest(min_cpus = 2)]
    fn delayed_remote(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        fixture.sender(1, 0, true, false, false)?;
        fixture.release(1)?;
        fixture.finish(context, 1, 0)?;
        assert!(fixture.state.entered.load(Ordering::Acquire));
        Ok(())
    }

    /// 测试目标：
    /// 验证当目标核正在执行某个耗时回调时，其他核心新发来的跨核调用请求能够排入目标核 FIFO 队列，并由目标核在同一轮 IPI 处理循环中继续取出执行完毕。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：至少 4 核环境下，CPU 1 向 CPU 0 发送开启 `arrival` 观测的 `delayed` 请求，CPU `2..N` 作为跟随者等待 CPU 1 的回调进入执行态后再向 CPU 0 发送请求；
    /// 2. When：放行所有任务，CPU `2..N` 在 CPU 0 忙于执行 CPU 1 的回调期间将各自请求入队；
    /// 3. Then：CPU 0 在回调中观测到待处理队列长度为 `cpu::count() - 2`（`queued == true`），且退出当前回调后继续处理完所有排队请求，全部发送者正常返回。
    #[test]
    #[ktest(min_cpus = 4)]
    fn processing_arrival(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        fixture.sender(1, 0, true, true, false)?;
        for id in 2..cpu::count() {
            fixture.sender(id, 0, false, false, true)?;
        }

        fixture.release(cpu::count() - 1)?;
        for id in 1..cpu::count() {
            fixture.finish(context, id, 0)?;
        }
        assert!(fixture.state.queued.load(Ordering::Acquire));
        Ok(())
    }
}
