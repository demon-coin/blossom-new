//! 一次等待用固定栈节点登记同一绝对期限，提前通知只更换等待代次。

use core::convert::TryFrom;
use core::pin::Pin;

use super::{Timer, TimerKind};
use crate::time::{self, Duration, MonotonicInstant};
use crate::wait::{Waiter, Waker};

/// 从当前单调时刻计算一次有限期限，溢出时不制造可表示范围外的时间。
pub fn after(delay: Duration) -> Option<MonotonicInstant> {
    time::now().checked_add(delay)
}

/// 按前一次相位加非零周期，跳过过期相位并返回严格晚于 now 的下一次期限。
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

/// 只计算一次绝对截止并真实阻塞；不能表示的 Duration 明确报告溢出。
pub fn sleep(duration: Duration) {
    let deadline = after(duration).expect("sleep deadline overflows monotonic time");
    sleep_until(deadline);
}

/// 已到期立即返回；提前通知不延长原期限，退栈前等待回调的真实尾部。
pub fn sleep_until(deadline: MonotonicInstant) {
    sleep_inner(deadline, None);
}

/// 精确硬 IRQ 回调只通知本轮 Waker；sleep_inner 的固定栈上下文保活至同步停止返回。
fn notify(timer: Pin<&Timer>) {
    // SAFETY: sleep_inner 的固定 Waker 存储在同步停止返回前始终有效，回调只共享访问。
    let waker = unsafe { &*(timer.context() as *const Waker) };
    waker.notify();
}

/// 在可睡且无活 Waiter 的普通任务中逐轮登记同一绝对期限，提前通知只更换等待身份。
/// 每轮固定 timer/Waker 栈存储，先结束活等待再同步停 timer，确认回调尾部后才退栈。
/// after_arm 仅供验收在实际登记后注入通知，不改变截止或存储停止责任。
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
        // SAFETY: timer 与 waker 的栈地址均固定，同步停止确认真实尾部后才允许退栈。
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

        // 先结束这一轮唯一 active wait，cancel_sync 必要时才能开始自己的尾部等待。
        drop(waiter);
        timer.as_ref().cancel_sync();
    }
}

/// 用户睡眠始终等待同一绝对期限；信号中断先撤唯一活等待，再确认回调真实尾部。
pub(crate) fn sleep_until_interruptible(deadline: MonotonicInstant) -> crate::wait::WaitResult {
    while time::now() < deadline {
        let waiter = Waiter::new_interruptible();
        let waker = core::pin::pin!(waiter.waker());
        let timer = core::pin::pin!(Timer::new(
            crate::cpu::current().id,
            TimerKind::Precise,
            notify,
            waker.as_ref().get_ref() as *const Waker as usize,
        ));
        // SAFETY: timer 与 Waker 在本栈固定，退出该轮前同步停止确认回调尾部，
        // 存储不移动、不提前释放，且回调只读取 Waker 并通知。
        unsafe { timer.as_ref().arm_borrowed(deadline) }.expect("fresh user sleep timer rejected");
        let result = waiter.park_signal();
        drop(waiter);
        timer.as_ref().cancel_sync();

        if deadline <= time::now() {
            return crate::wait::WaitResult::Woken;
        }
        if matches!(result, crate::wait::WaitResult::Interrupted) {
            return crate::wait::WaitResult::Interrupted;
        }
    }
    crate::wait::WaitResult::Woken
}

/// 提前通知验收走生产睡眠路径，每一轮仍登记原绝对截止，不提供另一套等待后端。
#[cfg(feature = "ktest")]
pub(crate) fn test_sleep_until(deadline: MonotonicInstant, after_arm: fn(Waker)) {
    sleep_inner(deadline, Some(after_arm));
}

#[cfg(feature = "ktest")]
static TEST_SLEEP_DEADLINE: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// 读取测试睡眠最近一次真实登记的节点快照，不以传入参数替代入队证据。
#[cfg(feature = "ktest")]
pub(super) fn test_sleep_deadline() -> MonotonicInstant {
    MonotonicInstant::from_nanos(TEST_SLEEP_DEADLINE.load(core::sync::atomic::Ordering::Acquire))
}
