//! 用户线程与线程组分别累计真实 CPU 片段，三个 interval timer 只拥有自身时间状态。

use alloc::sync::{Arc, Weak};
use core::alloc::AllocError;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use spin::Once;

use crate::sync::{SleepMutex, SpinLock};
use crate::task::signal::{SignalGroup, SIGALRM, SIGPROF, SIGVTALRM};
use crate::time::{self, MonotonicInstant};
use crate::timer::{Timer, TimerKind};

pub(crate) const USER_HZ: u64 = 100;
pub(crate) const ITIMER_REAL: usize = 0;
pub(crate) const ITIMER_VIRTUAL: usize = 1;
pub(crate) const ITIMER_PROF: usize = 2;
const ITIMER_COUNT: usize = ITIMER_PROF + 1;

#[derive(Clone, Copy, Default)]
pub(crate) struct CpuTime {
    pub(crate) user_ns: u64,
    pub(crate) kernel_ns: u64,
}

impl CpuTime {
    /// 返回真实用户和内核片段之和，累计空间耗尽直接暴露。
    pub(crate) fn total(self) -> u64 {
        self.user_ns
            .checked_add(self.kernel_ns)
            .expect("CPU time overflow")
    }
}

struct CpuTotals {
    user: AtomicU64,
    kernel: AtomicU64,
}

impl CpuTotals {
    /// 创建空执行账，内核任务不会持有此对象。
    const fn new() -> Self {
        Self {
            user: AtomicU64::new(0),
            kernel: AtomicU64::new(0),
        }
    }

    /// 只累计已经结束的 on-CPU 片段，不取时间或信号锁。
    fn account(&self, user: bool, ns: u64) {
        add(if user { &self.user } else { &self.kernel }, ns);
    }

    /// 跨 CPU 读取已经结算的累计值，字段可来自相邻片段边界。
    fn snapshot(&self) -> CpuTime {
        CpuTime {
            user_ns: self.user.load(Ordering::Relaxed),
            kernel_ns: self.kernel.load(Ordering::Relaxed),
        }
    }
}

/// 并发线程组累计使用检查过的原子加法，不能发生数值回绕。
fn add(total: &AtomicU64, ns: u64) {
    total
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |old| {
            old.checked_add(ns)
        })
        .expect("user CPU accounting overflow");
}

pub(crate) struct UserTime {
    totals: CpuTotals,
    group: Arc<GroupTime>,
    active: AtomicBool,
}

impl UserTime {
    /// 在用户身份发表前绑定真实组账；clone 的私有线程账从零开始。
    pub(crate) fn new(group: Arc<GroupTime>) -> Self {
        Self {
            totals: CpuTotals::new(),
            group,
            active: AtomicBool::new(true),
        }
    }

    /// accounting 的 IRQ-off 片段边界只做原子累计，不反取 siglock。
    pub(crate) fn account_execution(&self, user: bool, ns: u64) {
        assert!(self.is_active(), "retired user CPU time was charged");
        self.totals.account(user, ns);
        self.group.totals.account(user, ns);
    }

    /// 线程最后退出后不再恢复 Linux 记账，诊断拥有和离栈清理仍可保留总账。
    pub(crate) fn retire(&self) {
        self.active.store(false, Ordering::Relaxed);
    }

    /// 调度恢复只为尚未退出的用户执行者缓存片段关联。
    pub(crate) fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }

    /// 返回线程独有 CPU 账，不用 Task 总时间推算用户时间。
    pub(crate) fn snapshot(&self) -> CpuTime {
        self.totals.snapshot()
    }

    /// 返回原绑定的线程组时间对象，调用者不另建 pending 或时间身份。
    pub(crate) fn group(&self) -> &Arc<GroupTime> {
        &self.group
    }

    /// 在用户循环或 syscall 的合法锁外点消费 CPU timer，不在记账短段发信号。
    pub(crate) fn flush_cpu_timers(&self) {
        self.group.flush_cpu_timers();
    }
}

#[derive(Clone, Copy, Default)]
pub(crate) struct ItimerValue {
    pub(crate) interval_ns: u64,
    pub(crate) value_ns: u64,
}

#[derive(Clone, Copy, Default)]
struct IntervalTimer {
    expires: u64,
    interval: u64,
}

impl IntervalTimer {
    /// 读取当前时钟域的剩余值，待消费到期仍返回最小非零值。
    fn remaining(self, now: u64) -> ItimerValue {
        ItimerValue {
            interval_ns: self.interval,
            value_ns: if self.expires == 0 {
                0
            } else {
                self.expires.saturating_sub(now).max(1)
            },
        }
    }

    /// 到期只消费一次，按原相位直接跳到未来；表示空间耗尽时停止该周期。
    fn expire(&mut self, now: u64) -> bool {
        if self.expires == 0 || now < self.expires {
            return false;
        }
        self.expires = next_phase(self.expires, self.interval, now).unwrap_or(0);
        true
    }
}

/// 不补排迟到的每一个周期，下一期限严格晚于已经观察到的时间。
fn next_phase(previous: u64, interval: u64, now: u64) -> Option<u64> {
    if interval == 0 {
        return None;
    }
    let steps = now
        .checked_sub(previous)?
        .checked_div(interval)?
        .checked_add(1)?;
    previous.checked_add(steps.checked_mul(interval)?)
}

struct TimerState {
    timers: [IntervalTimer; ITIMER_COUNT],
    closed: bool,
}

pub(crate) struct GroupTime {
    totals: CpuTotals,
    children: CpuTotals,
    signals: Weak<SignalGroup>,
    state: SpinLock<TimerState>,
    real_operation: SleepMutex<()>,
    real: Once<Timer>,
    real_started: AtomicBool,
}

impl GroupTime {
    /// 在稳定 Arc 分配中建立 REAL 节点；弱信号关联不强持 Task、Mm 或线程组。
    pub(crate) fn try_new(signals: Arc<SignalGroup>) -> Result<Arc<Self>, AllocError> {
        let group = Arc::try_new(Self {
            totals: CpuTotals::new(),
            children: CpuTotals::new(),
            signals: Arc::downgrade(&signals),
            state: SpinLock::new(TimerState {
                timers: [IntervalTimer::default(); ITIMER_COUNT],
                closed: false,
            }),
            real_operation: SleepMutex::new(()),
            real: Once::new(),
            real_started: AtomicBool::new(false),
        })?;
        group.real.call_once(|| {
            Timer::new(
                crate::cpu::current().id,
                TimerKind::Precise,
                real_expired,
                Arc::as_ptr(&group) as usize,
            )
        });
        Ok(group)
    }

    /// 返回整个线程组已经结算的 CPU 账，已退出成员的执行量继续保留。
    pub(crate) fn snapshot(&self) -> CpuTime {
        self.totals.snapshot()
    }

    /// 返回已经实际消费退出事件的子进程累计账。
    pub(crate) fn children(&self) -> CpuTime {
        self.children.snapshot()
    }

    /// 父消费退出事件后只调用一次，WNOWAIT 不累计，子孙账随同组账一并转交。
    pub(crate) fn add_reaped_child(&self, child: &GroupTime) {
        let own = child.snapshot();
        let descendants = child.children();
        add(
            &self.children.user,
            own.user_ns
                .checked_add(descendants.user_ns)
                .expect("child user time overflow"),
        );
        add(
            &self.children.kernel,
            own.kernel_ns
                .checked_add(descendants.kernel_ns)
                .expect("child kernel time overflow"),
        );
    }

    /// 读取三个具体 timer 中的一项；which 必须已在 ABI 边界验证为 0、1、2。
    pub(crate) fn get_itimer(&self, which: usize) -> ItimerValue {
        let state = self.state.lock();
        state.timers[which].remaining(self.clock(which))
    }

    /// 原 clock 值与相对期限在同一短锁计算，溢出或已关闭不修改已登记状态。
    pub(crate) fn set_itimer(&self, which: usize, value: ItimerValue) -> Option<ItimerValue> {
        if which == ITIMER_REAL {
            return self.set_real(value);
        }
        let mut state = self.state.lock();
        if state.closed {
            return None;
        }
        let now = self.clock(which);
        let expires = if value.value_ns == 0 {
            0
        } else {
            now.checked_add(value.value_ns)?
        };
        let old = state.timers[which].remaining(now);
        state.timers[which] = IntervalTimer {
            expires,
            interval: value.interval_ns,
        };
        Some(old)
    }

    /// REAL 更新者由可睡锁串行化，旧回调真实结束后才建立新一期。
    fn set_real(&self, value: ItimerValue) -> Option<ItimerValue> {
        let _operation = self.real_operation.lock();
        let mut state = self.state.lock();
        if state.closed {
            return None;
        }
        let now = time::now().as_nanos();
        let expires = if value.value_ns == 0 {
            0
        } else {
            now.checked_add(value.value_ns)?
        };
        let old = state.timers[ITIMER_REAL].remaining(now);
        state.timers[ITIMER_REAL] = IntervalTimer::default();
        drop(state);
        self.real_timer().cancel_sync();
        let mut state = self.state.lock();
        state.timers[ITIMER_REAL] = IntervalTimer {
            expires,
            interval: value.interval_ns,
        };
        if expires != 0 {
            self.real_started.store(true, Ordering::Relaxed);
            // SAFETY: GroupTime 只能在稳定 Arc 中建立，节点保存在不可移出的 Once 内；
            // 最后拥有释放以及组退出均同步停止，当前共享借用保活至 arm 返回。
            unsafe {
                self.real_timer()
                    .arm_borrowed(MonotonicInstant::from_nanos(expires))
            }
            .expect("live REAL timer was closed");
        }
        Some(old)
    }

    /// 在可睡且无活 Waiter 的组最后退出路径停止所有生产者，确认 REAL 回调尾部。
    pub(crate) fn stop(&self) {
        let _operation = self.real_operation.lock();
        let mut state = self.state.lock();
        state.closed = true;
        state.timers = [IntervalTimer::default(); ITIMER_COUNT];
        drop(state);
        if self.real_started.swap(false, Ordering::Relaxed) {
            self.real_timer().close_sync();
        }
    }

    /// CPU 记账后的合法锁外点一次检查两项，不逐周期补排，不持时间锁通知信号。
    pub(crate) fn flush_cpu_timers(&self) {
        let mut state = self.state.lock();
        let virtual_due = state.timers[ITIMER_VIRTUAL].expire(self.clock(ITIMER_VIRTUAL));
        let prof_due = state.timers[ITIMER_PROF].expire(self.clock(ITIMER_PROF));
        drop(state);
        if virtual_due {
            self.send(SIGVTALRM);
        }
        if prof_due {
            self.send(SIGPROF);
        }
    }

    /// 各项使用真实单调、用户或用户加内核的具体时间域。
    fn clock(&self, which: usize) -> u64 {
        match which {
            ITIMER_REAL => time::now().as_nanos(),
            ITIMER_VIRTUAL => self.snapshot().user_ns,
            ITIMER_PROF => self.snapshot().total(),
            _ => panic!("invalid interval timer"),
        }
    }

    /// 固定 Arc 分配和不可移出字段为内部 Pin 借用提供稳定地址。
    fn real_timer(&self) -> Pin<&Timer> {
        // SAFETY: GroupTime 创建后只通过 Arc 共享，Once 中的 Timer 不被取出或移动；
        // GroupTime 析构同步确认回调结束以后才销毁此字段。
        unsafe { Pin::new_unchecked(self.real.get().expect("REAL timer not initialized")) }
    }

    /// 时间锁外通过真实组 pending 发普通 timer 信号，无临时队列或分配。
    fn send(&self, signum: u8) {
        if let Some(signals) = self.signals.upgrade() {
            signals.send_timer(signum);
        }
    }
}

impl Drop for GroupTime {
    /// 组最后退出须已确认回调尾部，未发表对象从未登记；析构不能隐式引入活等待。
    fn drop(&mut self) {
        assert!(
            !self.real_started.load(Ordering::Relaxed),
            "REAL timer owner dropped before stop"
        );
    }
}

/// 精确回调沿经过时间推进 REAL，睡眠线程无需自行结算；通知始终在时间锁外。
fn real_expired(timer: Pin<&Timer>) {
    // SAFETY: 上下文指向稳定 GroupTime，借用登记拥有者在 stop/析构同步确认尾部
    // 以前不释放存储；回调不强持该对象，不形成自身最后 put 或强引用环。
    let group = unsafe { &*(timer.context() as *const GroupTime) };
    let mut state = group.state.lock();
    let due = state.timers[ITIMER_REAL].expire(time::now().as_nanos());
    let expires = state.timers[ITIMER_REAL].expires;
    if due && expires != 0 {
        // SAFETY: 原借用拥有者仍保持稳定存储至同步停止，运行回调可登记独立下一轮；
        // state 锁串行化更新和停止，锁序为时间状态到 timer base，回调入口不持 base。
        unsafe { timer.arm_borrowed(MonotonicInstant::from_nanos(expires)) }
            .expect("running REAL timer was closed");
    }
    drop(state);
    if due {
        group.send(SIGALRM);
    }
}
