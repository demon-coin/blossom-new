//! 用户线程与线程组 CPU 执行时间统计及 POSIX 间隔定时器（Interval Timers）。
//!
//! ## 模块职责与架构定位
//! 本模块维护每个用户线程私有的 CPU 时间账本（`UserTime`）与整个进程（线程组）共享的 CPU 时间账本及三类间隔定时器（`GroupTime`）。
//! - **上游调用者**：`crate::accounting`（在关中断上下文结算用户态/内核态 CPU 执行片段）、`crate::task::user`（返回用户态前结算并触发 CPU 定时器）、`crate::task::process`（子进程被父进程 `wait4` 回收时汇总子孙 CPU 时间）及系统调用层（`getitimer`、`setitimer`、`times`、`getrusage`、`clock_gettime`）。
//! - **下游依赖**：`crate::time`（读取系统单调时钟纳秒值）、`crate::timer`（驱动 `ITIMER_REAL` 的高精度硬件定时器）、`crate::task::signal`（到期时向线程组发送 `SIGALRM`、`SIGVTALRM` 或 `SIGPROF`）。
//!
//! ## 核心设计与三类间隔定时器机制
//!
//! ```text
//!   定时器类型       | 参考时钟域                     | 推进与触发时机                | 到期信号
//!  ------------------+--------------------------------+-------------------------------+-----------
//!   ITIMER_REAL (0)  | 系统单调时钟 (Monotonic ns)    | 硬件精确 Timer 中断回调实时触发 | SIGALRM
//!   ITIMER_VIRTUAL(1)| 线程组累计用户态 CPU 时间      | 线程返回用户态前 flush 轮询检查 | SIGVTALRM
//!   ITIMER_PROF (2)  | 线程组累计 (用户态+内核态) 时间| 线程返回用户态前 flush 轮询检查 | SIGPROF
//! ```
//!
//! - **无锁原子 CPU 记账**：`CpuTotals` 使用 `AtomicU64` 累计已完成的执行片段，使 `accounting` 在关中断的上下文切换/进出用户态边界无需获取任何自旋锁即可更新线程与组时间。
//! - **跳跃式周期重排（防信号风暴）**：周期性定时器若因线程睡眠或高负载错过了多个周期，`next_phase()` 直接按原相位步进跳到严格晚于当前时刻 `now` 的下一个未来周期点，只发送一次到期信号，不会按落后的周期数补发信号。
//! - **`ITIMER_REAL` 借用定时器生命周期**：`GroupTime` 必须存放在稳定的 `Arc` 堆分配中，其内部的 `Once<Timer>` 地址固定不变。进程最后一个成员退出时必须调用 `GroupTime::stop()` 执行 `close_sync()`，等待任何正在执行的定时器中断回调彻底退出后才允许析构 `GroupTime`（由 `Drop` 中的 `real_started` 断言守卫）。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - **层级顺序**：`GroupTime::real_operation`（`SleepMutex`，串行化 `setitimer(ITIMER_REAL)` 与进程退出 `stop()`） ➔ `GroupTime::state`（`SpinLock`） ➔ 底层 `Timer` 基座锁（`arm_borrowed`）。
//! - **锁外同步取消与锁外发信号**：
//!   1. `cancel_sync()` / `close_sync()` 等待并发中断回调退出，而中断回调 `real_expired` 内部会获取 `GroupTime::state` 锁，因此调用 `cancel_sync()` / `close_sync()` 前**必须先释放 `GroupTime::state` 自旋锁**，否则将导致死锁。
//!   2. 严禁在持有 `GroupTime::state` 自旋锁时调用 `SignalGroup::send_timer`（避免时间锁与信号锁产生交叉依赖），必须在释放 `state` 锁后再发送信号。

use alloc::sync::{Arc, Weak};
use core::alloc::AllocError;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use spin::Once;

use crate::sync::{SleepMutex, SpinLock};
use crate::task::signal::{SignalGroup, SIGALRM, SIGPROF, SIGVTALRM};
use crate::time::{self, MonotonicInstant};
use crate::timer::{Timer, TimerKind};

/// 向用户态报告传统时钟滴答数（如 `times`、`SIGCHLD` 的 `utime`/`stime`）时采用的标准频率（100 Hz，即每滴答 10 ms）。
pub(crate) const USER_HZ: u64 = 100;

/// 真实时间（单调时钟）间隔定时器索引，到期发送 `SIGALRM`。
pub(crate) const ITIMER_REAL: usize = 0;
/// 进程用户态 CPU 时间间隔定时器索引，到期发送 `SIGVTALRM`。
pub(crate) const ITIMER_VIRTUAL: usize = 1;
/// 进程总 CPU 时间（用户态 + 内核态）性能剖析定时器索引，到期发送 `SIGPROF`。
pub(crate) const ITIMER_PROF: usize = 2;
/// 每个进程维护的间隔定时器总类数（3 类）。
const ITIMER_COUNT: usize = ITIMER_PROF + 1;

/// 线程或进程的 CPU 执行时间快照（单位均为纳秒）。
#[derive(Clone, Copy, Default)]
pub(crate) struct CpuTime {
    /// 累计在用户态执行指令消耗的纳秒数。
    pub(crate) user_ns: u64,
    /// 累计在内核态代表该任务执行（如系统调用、缺页异常）消耗的纳秒数。
    pub(crate) kernel_ns: u64,
}

impl CpuTime {
    /// 返回用户态与内核态累计 CPU 时间的总和（纳秒）；算术溢出时触发 panic。
    pub(crate) fn total(self) -> u64 {
        self.user_ns
            .checked_add(self.kernel_ns)
            .expect("CPU time overflow")
    }
}

/// 支持多核并发无锁累加的原子 CPU 时间账本。
struct CpuTotals {
    /// 原子累计的用户态执行时间（纳秒）。
    user: AtomicU64,
    /// 原子累计的内核态执行时间（纳秒）。
    kernel: AtomicU64,
}

impl CpuTotals {
    /// 创建初始为零的原子 CPU 时间账本。
    const fn new() -> Self {
        Self {
            user: AtomicU64::new(0),
            kernel: AtomicU64::new(0),
        }
    }

    /// 将一段已结束的 CPU 执行时长 `ns` 原子累加到用户态或内核态计数器（无需获取任何锁，可在关中断路径安全调用）。
    fn account(&self, user: bool, ns: u64) {
        add(if user { &self.user } else { &self.kernel }, ns);
    }

    /// 读取当前已结算的用户态与内核态累计纳秒数快照。
    fn snapshot(&self) -> CpuTime {
        CpuTime {
            user_ns: self.user.load(Ordering::Relaxed),
            kernel_ns: self.kernel.load(Ordering::Relaxed),
        }
    }
}

/// 对原子 64 位时间计数器执行带溢出检查的无锁加法。
fn add(total: &AtomicU64, ns: u64) {
    total
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |old| {
            old.checked_add(ns)
        })
        .expect("user CPU accounting overflow");
}

/// 单个用户线程私有的 CPU 时间统计组件，同时持有其所属线程组时间账本的共享引用。
pub(crate) struct UserTime {
    /// 本线程独立累计的用户态与内核态 CPU 时间。
    totals: CpuTotals,
    /// 所属进程（线程组）的共享时间与间隔定时器控制块。
    group: Arc<GroupTime>,
    /// 标记本线程是否仍处于活跃计费生命周期内；线程退出解绑后置为 `false`。
    active: AtomicBool,
}

impl UserTime {
    /// 创建绑定到指定线程组 `group` 的新线程时间账本（线程私有计数从零开始，初始处于活跃状态）。
    pub(crate) fn new(group: Arc<GroupTime>) -> Self {
        Self {
            totals: CpuTotals::new(),
            group,
            active: AtomicBool::new(true),
        }
    }

    /// 在 `accounting` 关中断结算边界将 `ns` 同时累加到本线程私有账本和所属线程组共享账本（不做锁操作与信号发送）。
    pub(crate) fn account_execution(&self, user: bool, ns: u64) {
        assert!(self.is_active(), "retired user CPU time was charged");
        self.totals.account(user, ns);
        self.group.totals.account(user, ns);
    }

    /// 在线程退出时将本账本标记为已退役，此后禁止继续向该线程记入新的 CPU 时间，但保留历史累计值供查询。
    pub(crate) fn retire(&self) {
        self.active.store(false, Ordering::Relaxed);
    }

    /// 返回本线程是否仍处于活跃的 CPU 记账周期内。
    pub(crate) fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }

    /// 读取本线程私有的累计 CPU 时间快照（供 `CLOCK_THREAD_CPUTIME_ID` 与 `RUSAGE_THREAD` 查询）。
    pub(crate) fn snapshot(&self) -> CpuTime {
        self.totals.snapshot()
    }

    /// 获取所属线程组共享的 `GroupTime` 引用。
    pub(crate) fn group(&self) -> &Arc<GroupTime> {
        &self.group
    }

    /// 在线程安全点（未持调度/记账锁）检查并触发所属线程组已到期的 `ITIMER_VIRTUAL` 与 `ITIMER_PROF` 定时器。
    pub(crate) fn flush_cpu_timers(&self) {
        self.group.flush_cpu_timers();
    }
}

/// `getitimer` / `setitimer` 在内核内部传递的纳秒级间隔定时器配置与剩余时间值。
#[derive(Clone, Copy, Default)]
pub(crate) struct ItimerValue {
    /// 定时器首次到期后自动重装填的周期步长（纳秒）；为 0 表示单次触发（One-shot）定时器。
    pub(crate) interval_ns: u64,
    /// 距离下一次到期的剩余时间或新设定的初始倒计时（纳秒）；为 0 表示定时器已停用或请求停用。
    pub(crate) value_ns: u64,
}

/// 单个间隔定时器的内部绝对到期相位与周期状态（单位均为对应时钟域的纳秒）。
#[derive(Clone, Copy, Default)]
struct IntervalTimer {
    /// 下一次触发在对应时钟域上的绝对纳秒刻度；为 0 表示当前未启用。
    expires: u64,
    /// 周期重装填间隔（纳秒）；为 0 表示首次到期后自动停用。
    interval: u64,
}

impl IntervalTimer {
    /// 根据对应时钟域的当前读数 `now` 计算剩余时间：若定时器已启用且已达或超过 `expires` 但尚未消费，仍返回最小非零值 `1 ns` 以避免用户态误判为已停用。
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

    /// 检查当前时刻 `now` 是否已到达或越过 `expires`：若到期则推进到严格晚于 `now` 的下一个未来周期点（单次定时器或算术溢出时归零停用）并返回 `true`。
    fn expire(&mut self, now: u64) -> bool {
        if self.expires == 0 || now < self.expires {
            return false;
        }
        self.expires = next_phase(self.expires, self.interval, now).unwrap_or(0);
        true
    }
}

/// 按初始相位 `previous` 与周期 `interval` 一步计算出严格大于当前时刻 `now` 的下一个未来到期点，跳过中间已延误的全部周期。
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

/// 受 `GroupTime::state` 自旋锁保护的三类间隔定时器状态。
struct TimerState {
    /// 按 `ITIMER_REAL`、`ITIMER_VIRTUAL`、`ITIMER_PROF` 下标排列的三个定时器内部状态。
    timers: [IntervalTimer; ITIMER_COUNT],
    /// 标记整个线程组是否已进入最后退出阶段关闭了所有定时器（一旦为 `true` 拒绝任何后续 `setitimer`）。
    closed: bool,
}

/// 进程（线程组）共享的 CPU 时间统计、已回收子孙进程时间账本及三类间隔定时器。
pub(crate) struct GroupTime {
    /// 本线程组全体成员（含已退出线程）共同累计的用户态与内核态 CPU 时间。
    totals: CpuTotals,
    /// 已通过 `wait4` 实际消费退出事件的子进程（及其已回收后代）累计 CPU 时间。
    children: CpuTotals,
    /// 指向所属线程组信号控制块的弱引用，供定时器到期时在锁外发送信号（避免与进程对象形成 `Arc` 循环引用）。
    signals: Weak<SignalGroup>,
    /// 保护 `timers` 数组与 `closed` 标志的自旋锁。
    state: SpinLock<TimerState>,
    /// 串行化并发 `setitimer(ITIMER_REAL)` 与进程退出 `stop()` 的睡眠互斥锁，确保同步取消旧硬件定时器期间不会交错启动新周期。
    real_operation: SleepMutex<()>,
    /// 原地初始化并永久驻留在 `Arc<GroupTime>` 固定地址内的 `ITIMER_REAL` 硬件高精度定时器节点。
    real: Once<Timer>,
    /// 记录 `real` 硬件定时器自进程创建以来是否曾被启动过，供 `stop()` 判断是否需要调用 `close_sync()` 及 `Drop` 断言检查。
    real_started: AtomicBool,
}

impl GroupTime {
    /// 在堆上分配 `Arc<GroupTime>` 并原地绑定指回自身裸指针的 `ITIMER_REAL` 定时器节点。
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

    /// 读取本线程组全体成员（包含已退出的历史成员）累计结算的 CPU 时间快照。
    pub(crate) fn snapshot(&self) -> CpuTime {
        self.totals.snapshot()
    }

    /// 读取已被本进程正式回收（不含 `WNOWAIT` 仅窥视）的子进程及后代累计 CPU 时间快照。
    pub(crate) fn children(&self) -> CpuTime {
        self.children.snapshot()
    }

    /// 在父进程正式消费并回收子进程退出状态时调用一次，将子进程自身及其已回收子孙的 CPU 时间并入父进程的 `children` 账本。
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

    /// 查询指定间隔定时器（`which` 须已校验为 `ITIMER_REAL`、`ITIMER_VIRTUAL` 或 `ITIMER_PROF`）当前的周期与剩余时间。
    pub(crate) fn get_itimer(&self, which: usize) -> ItimerValue {
        let state = self.state.lock();
        state.timers[which].remaining(self.clock(which))
    }

    /// 更新指定间隔定时器的配置并返回旧剩余值；若进程定时器已关闭或相对时间加法溢出则不修改原状态并返回 `None`。
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

    /// 更新 `ITIMER_REAL` 硬件定时器：在 `real_operation` 睡眠锁串行化保护下，先清空状态并在锁外同步取消旧回调，再重新装填新到期时间。
    fn set_real(&self, value: ItimerValue) -> Option<ItimerValue> {
        let _operation = self.real_operation.lock();
        // Step 1: 在 state 自旋锁下计算目标绝对时间、快照旧剩余值并暂时清零定时器，防止旧回调继续重装填。
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

        // Step 2: 释放 state 自旋锁后同步取消硬件定时器，等待任何正在运行的中断回调彻底退出。
        self.real_timer().cancel_sync();

        // Step 3: 重新获取 state 自旋锁写入新周期参数，并在需要启用时重新挂载硬件定时器。
        let mut state = self.state.lock();
        state.timers[ITIMER_REAL] = IntervalTimer {
            expires,
            interval: value.interval_ns,
        };
        if expires != 0 {
            self.real_started.store(true, Ordering::Relaxed);
            // SAFETY:
            // 1. `GroupTime` 仅通过 `try_new` 构造在 `Arc` 堆内存中，其 `Once<Timer>` 内部地址在 `GroupTime` 销毁前永不移动；
            // 2. 进程最后退出路径必定调用 `stop()` 执行 `close_sync()`（且由 `Drop` 中的 `real_started` 断言校验），保证 `GroupTime` 释放前所有异步定时器回调均已彻底退出。
            unsafe {
                self.real_timer()
                    .arm_borrowed(MonotonicInstant::from_nanos(expires))
            }
            .expect("live REAL timer was closed");
        }
        Some(old)
    }

    /// 在进程最后一个线程退出的可睡眠路径上永久关闭本组全部定时器，并同步等待正在执行的 `ITIMER_REAL` 硬件回调彻底退出。
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

    /// 轮询检查 `ITIMER_VIRTUAL` 与 `ITIMER_PROF` 是否达到进程累计 CPU 时间阈值，并在释放 `state` 锁后发送对应的到期信号。
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

    /// 读取指定间隔定时器类别对应的当前时钟刻度（纳秒）。
    fn clock(&self, which: usize) -> u64 {
        match which {
            ITIMER_REAL => time::now().as_nanos(),
            ITIMER_VIRTUAL => self.snapshot().user_ns,
            ITIMER_PROF => self.snapshot().total(),
            _ => panic!("invalid interval timer"),
        }
    }

    /// 返回固定在 `Arc<GroupTime>` 堆内存中的 `ITIMER_REAL` 硬件定时器 `Pin` 引用。
    fn real_timer(&self) -> Pin<&Timer> {
        // SAFETY:
        // 1. `GroupTime` 始终封装在 `Arc` 内，其字段 `real: Once<Timer>` 自 `try_new` 初始化后绝不会被移出或替换；
        // 2. `stop()` 通过 `close_sync()` 保证在 `GroupTime` 内存释放前底层定时器已彻底注销且回调全部退出。
        unsafe { Pin::new_unchecked(self.real.get().expect("REAL timer not initialized")) }
    }

    /// 若所属线程组的信号控制块仍然存活，向其发送免分配的内核定时器信号。
    fn send(&self, signum: u8) {
        if let Some(signals) = self.signals.upgrade() {
            signals.send_timer(signum);
        }
    }
}

impl Drop for GroupTime {
    /// 断言一旦启动过 `ITIMER_REAL` 硬件定时器，销毁 `GroupTime` 前必须已经过 `stop()` 同步关闭，防止悬垂上下文指针导致 UAF。
    fn drop(&mut self) {
        assert!(
            !self.real_started.load(Ordering::Relaxed),
            "REAL timer owner dropped before stop"
        );
    }
}

/// `ITIMER_REAL` 硬件定时器到期中断回调函数：推进周期相位、按需重装填下一周期，并在时间锁外发送 `SIGALRM`。
fn real_expired(timer: Pin<&Timer>) {
    // SAFETY:
    // 1. `timer.context()` 在 `GroupTime::try_new` 中初始化为 `Arc<GroupTime>` 的稳定数据裸指针；
    // 2. `GroupTime::stop()` 在进程销毁前会调用 `close_sync()` 等待本回调彻底执行完毕并退出，因此本回调运行期间 `&GroupTime` 必定有效且无悬垂风险。
    let group = unsafe { &*(timer.context() as *const GroupTime) };
    let mut state = group.state.lock();
    let due = state.timers[ITIMER_REAL].expire(time::now().as_nanos());
    let expires = state.timers[ITIMER_REAL].expires;
    if due && expires != 0 {
        // SAFETY:
        // 1. `timer` 驻留在 `Arc<GroupTime>` 内部固定地址，生命周期由 `GroupTime::stop()` 的 `close_sync()` 兜底保证；
        // 2. 当前持有 `group.state` 锁且未持有定时器基座锁（符合 `state -> timer base` 锁顺序），若并发 `stop()` 已置空 `timers` 则 `expires == 0` 不会进入此分支。
        unsafe { timer.arm_borrowed(MonotonicInstant::from_nanos(expires)) }
            .expect("running REAL timer was closed");
    }
    drop(state);

    if due {
        group.send(SIGALRM);
    }
}
