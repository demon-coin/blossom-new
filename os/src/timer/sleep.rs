//! 基于栈上固定高精度定时器的任务定时睡眠实现。
//!
//! # 模块职责与架构定位
//! 本模块向上层内核任务、系统调用（如 `nanosleep` / `clock_nanosleep`）以及内核自检提供单调时钟睡眠原语：
//! - 不可被用户信号打断的睡眠（[`sleep`]、[`sleep_until`]）；
//! - 可响应待处理用户信号提前返回的睡眠（[`sleep_until_interruptible`]）；
//! - 单次相对延迟换算（[`after`]）与无漂移周期相位推进（[`periodic_deadline`]）。
//!
//! # 核心设计与生命周期安全协议
//! 睡眠路径不进行堆内存分配，而是在当前任务内核栈上固定（`pin!`）一对 [`Waiter`]/[`Waker`] 与借用模式的
//! 高精度 [`Timer`]，通过 [`Timer::arm_borrowed`] 将定时器挂载到当前 CPU 的红黑树中。
//!
//! ## 1. 栈上借用节点退栈安全顺序
//! 当任务从 `park()` 或 `park_signal()` 返回时（可能源于定时器正常到期、外部虚假唤醒或用户信号打断），
//! 时钟中断回调 [`notify`] 可能仍在另一个上下文或正处于收尾阶段。为防止悬垂指针踩栈，每轮循环在退出栈作用域前
//! 严格执行两阶段清理：
//! ```text
//! Stack Frame (per iteration)
//! +-------------------------------------------------------------------+
//! | waiter: Waiter                                                    |
//! | waker:  Pin<&mut Waker>  <--- timer.context 指向该栈上地址         |
//! | timer:  Pin<&mut Timer>  (TimerKind::Precise, callback = notify)  |
//! +-------------------------------------------------------------------+
//!   1. arm_borrowed(deadline)  --> 登记入当前 CPU 红黑树
//!   2. waiter.park()           --> 阻塞等待 notify() 或信号唤醒
//!   3. drop(waiter)            --> 结束当前栈上 Waiter 的活跃等待身份
//!   4. timer.cancel_sync()     --> 摘除未到期节点，并在需要时新建内部 Waiter
//!                                  阻塞等待正在执行的 notify() 彻底退出
//!   5. 安全弹出本轮栈帧（此时 timer 既不在树中，也无正在执行的回调）
//! ```
//! 注意第 3 步必须先 `drop(waiter)` 再在第 4 步调用 `cancel_sync()`：因为单个任务同一时刻只允许持有一个活跃
//! `Waiter`，而 `cancel_sync()` 在遇到回调正在执行时需要构造自己的 `Waiter` 在 `TimerBase::finished` 上等待。
//!
//! ## 2. 虚假唤醒与固定绝对截止时间
//! 若任务在 `deadline` 尚未到达前因提前通知被唤醒，循环会构造新一轮 `Waiter` 与 `Timer`，但**始终传入最初计算的
//! 同一绝对截止时间 `deadline`**，不会重新累加相对时长，从而消除重入误差。
//!
//! # 锁与上下文契约
//! - [`sleep`]、[`sleep_until`] 与 [`sleep_until_interruptible`] 必须在可睡眠的普通任务上下文中调用（严禁在中断、
//!   关抢占底半部或持有自旋锁时调用），且调用时当前任务不得已持有其他未释放的 `Waiter`。
//! - 硬中断回调 [`notify`] 仅通过共享引用调用基于原子操作与极短自旋锁临界区的 [`Waker::notify`]，不会阻塞或重入定时器锁。

use core::convert::TryFrom;
use core::pin::Pin;

use super::{Timer, TimerKind};
use crate::time::{self, Duration, MonotonicInstant};
use crate::wait::{Waiter, Waker};

/// 基于当前单调时钟时刻计算延迟 `delay` 后的绝对截止时间；若发生 64 位纳秒加法溢出则返回 `None`。
pub fn after(delay: Duration) -> Option<MonotonicInstant> {
    time::now().checked_add(delay)
}

/// 基于上一次周期基准时刻 `previous` 与非零周期 `period`，跳过所有已落后的周期相位，计算严格晚于 `now` 的下一个周期截止时间。
///
/// 若 `period` 为零、超出 `u64` 纳秒范围或计算结果超出单调时钟可表示上限，则返回 `None`。
#[cfg_attr(
    not(feature = "ktest"),
    expect(dead_code, reason = "定时器测试核对周期相位跳过、零周期和纳秒上溢边界")
)]
pub fn periodic_deadline(
    previous: MonotonicInstant,
    period: Duration,
    now: MonotonicInstant,
) -> Option<MonotonicInstant> {
    let period = u64::try_from(period.as_nanos()).ok()?;
    if period == 0 {
        return None;
    }

    let steps = match now.as_nanos().checked_sub(previous.as_nanos()) {
        Some(elapsed) => elapsed.checked_div(period)?.checked_add(1)?,
        None => 1,
    };
    previous
        .as_nanos()
        .checked_add(steps.checked_mul(period)?)
        .map(MonotonicInstant::from_nanos)
}

/// 使当前任务睡眠至少 `duration` 指定的时间长度。
///
/// 内部先将相对时长换算为单调时钟绝对截止时间，再阻塞等待至该时刻到达。
///
/// # Panics
/// 若当前时刻加上 `duration` 超出 64 位纳秒单调时钟上限，则触发 panic。
pub fn sleep(duration: Duration) {
    let deadline = after(duration).expect("sleep deadline overflows monotonic time");
    sleep_until(deadline);
}

/// 使当前任务睡眠直至单调时钟达到指定的绝对截止时间 `deadline`（若 `deadline` 已过去则立即返回）。
pub fn sleep_until(deadline: MonotonicInstant) {
    sleep_inner(deadline, None);
}

/// 睡眠定时器的时钟硬中断回调函数：读取 `timer.context()` 中保存的栈上 [`Waker`] 指针并唤醒睡眠任务。
fn notify(timer: Pin<&Timer>) {
    // SAFETY:
    // 1. `timer.context()` 由下方 `sleep_inner` 或 `sleep_until_interruptible` 设置为本轮栈帧中 `pin!` 固定的 `Waker` 地址；
    // 2. 睡眠循环在 `waker` 离开作用域退栈前必定调用 `timer.cancel_sync()`，等待本回调彻底执行完毕，
    //    因此在本回调运行期间 `Waker` 内存始终有效且只读共享。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// 不可中断睡眠的内部循环：在栈上固定 `Waiter`、`Waker` 与高精度 `Timer`，循环等待直至单调时钟达到 `deadline`。
///
/// 可选钩子 `after_arm` 仅供内核自检在定时器完成入队后注入提前唤醒通知，生产路径恒传入 `None`。
fn sleep_inner(deadline: MonotonicInstant, after_arm: Option<fn(Waker)>) {
    while time::now() < deadline {
        let waiter = Waiter::new();
        let waker = core::pin::pin!(waiter.waker());
        let timer = core::pin::pin!(Timer::new(
            crate::cpu::current().id,
            TimerKind::Precise,
            notify,
            waker.as_ref().get_ref() as *const Waker as usize,
        ));
        // SAFETY:
        // 1. `timer` 与 `waker` 均通过 `core::pin::pin!` 固定在当前循环轮次的栈帧中，地址在退栈前不变；
        // 2. 下方在离开本轮作用域前必定执行 `timer.as_ref().cancel_sync()`，确保定时器已从队列摘除且 `notify` 回调已彻底退出。
        unsafe { timer.as_ref().arm_borrowed(deadline) }
            .expect("fresh precise sleep timer rejected");
        #[cfg(feature = "ktest")]
        if after_arm.is_some() {
            TEST_SLEEP_DEADLINE.store(
                super::testing::snapshot(timer.as_ref()).expiry.as_nanos(),
                core::sync::atomic::Ordering::Release,
            );
        }
        if let Some(after_arm) = after_arm {
            after_arm(waiter.waker());
        }
        waiter.park();

        // 先释放本轮睡眠使用的 Waiter，以便后续 cancel_sync 在需要等待回调退出时能够创建新的 Waiter。
        drop(waiter);
        timer.as_ref().cancel_sync();
    }
}

/// 可响应待处理用户信号的定时睡眠：等待单调时钟达到绝对截止时间 `deadline`，或在收到可中断信号时提前返回。
///
/// # 返回值
/// - [`crate::wait::WaitResult::Woken`]：单调时钟已达到或超过 `deadline`；
/// - [`crate::wait::WaitResult::Interrupted`]：在 `deadline` 到达前检测到当前任务有待处理的用户信号。
pub(crate) fn sleep_until_interruptible(deadline: MonotonicInstant) -> crate::wait::WaitResult {
    while time::now() < deadline {
        // Step 1: 在当前栈帧固定可响应信号的 Waiter、Waker 与高精度借用 Timer，并注册绝对截止时间 deadline。
        let waiter = Waiter::new_interruptible();
        let waker = core::pin::pin!(waiter.waker());
        let timer = core::pin::pin!(Timer::new(
            crate::cpu::current().id,
            TimerKind::Precise,
            notify,
            waker.as_ref().get_ref() as *const Waker as usize,
        ));
        // SAFETY:
        // 1. `timer` 与 `waker` 均通过 `core::pin::pin!` 固定在本轮栈帧中；
        // 2. 无论 `park_signal()` 因超时到期还是因信号打断返回，下方均会先执行 `timer.as_ref().cancel_sync()`，
        //    等待 `notify` 回调彻底退出后才允许弹出本轮栈帧。
        unsafe { timer.as_ref().arm_borrowed(deadline) }.expect("fresh user sleep timer rejected");

        // Step 2: 阻塞等待定时器唤醒或用户信号打断；返回后立即释放本轮 Waiter 并同步取消栈上定时器。
        let result = waiter.park_signal();
        drop(waiter);
        timer.as_ref().cancel_sync();

        // Step 3: 若截止时间已到则优先报告正常唤醒（Woken）；若尚未到期且被信号打断则返回 Interrupted。
        if deadline <= time::now() {
            return crate::wait::WaitResult::Woken;
        }
        if matches!(result, crate::wait::WaitResult::Interrupted) {
            return crate::wait::WaitResult::Interrupted;
        }
    }
    crate::wait::WaitResult::Woken
}

/// 内核自检专用入口：复用 [`sleep_inner`] 睡眠至 `deadline`，并在每轮定时器入队后调用 `after_arm` 注入测试动作（如提前唤醒）。
#[cfg(feature = "ktest")]
pub(crate) fn test_sleep_until(deadline: MonotonicInstant, after_arm: fn(Waker)) {
    sleep_inner(deadline, Some(after_arm));
}

/// 内核自检模式下记录 [`test_sleep_until`] 最近一轮入队后从定时器快照读出的截止时间（单位：纳秒）。
#[cfg(feature = "ktest")]
static TEST_SLEEP_DEADLINE: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// 读取 [`test_sleep_until`] 最近一轮实际注册到定时器节点中的绝对截止时间。
#[cfg(feature = "ktest")]
pub(super) fn test_sleep_deadline() -> MonotonicInstant {
    MonotonicInstant::from_nanos(TEST_SLEEP_DEADLINE.load(core::sync::atomic::Ordering::Acquire))
}
