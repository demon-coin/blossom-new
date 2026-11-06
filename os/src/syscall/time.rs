//! 时间查询、睡眠等待、间隔定时器（itimer）与 CPU 资源统计系统调用实现。
//!
//! ## 模块职责与架构定位
//! - **上游调用者**：`crate::syscall` 主分派器，以及信号处理与系统调用重启路径（调用 `resume_sleep` 与 `interrupt_sleep`）。
//! - **下游依赖**：
//!   - `crate::accounting` 与 `crate::task::time`：结算并读取进程组或线程的 CPU 时间账目，管理间隔定时器（`ITIMER_REAL` / `ITIMER_VIRTUAL` / `ITIMER_PROF`）。
//!   - `crate::time` 与 `crate::timer`：提供单调时钟（`MonotonicInstant`）、实时日历时钟（UTC 纳秒）以及可中断的定时睡眠等待队列。
//!   - `crate::syscall::error`：提供用户空间内存安全读写（`copyin` / `copyout`）。
//!
//! ## 核心设计与睡眠信号中断/重启状态机
//!
//! `nanosleep` 与 `clock_nanosleep` 在首次进入内核时会将用户请求的睡眠时长统一换算为单调时钟的**绝对截止时间（`deadline: MonotonicInstant`）**，避免信号中断重启时重新按相对时长累加导致睡眠时间漂移：
//!
//! ```text
//! [用户调用 nanosleep / clock_nanosleep]
//!           │
//!           ▼
//!   解析并校验 clock / flags / timespec
//!   计算首次绝对截止时间 deadline
//!           │
//!           ▼
//!   timer::sleep_until_interruptible(deadline)
//!      ├── WaitResult::Woken (到期正常唤醒) ──────────► Outcome::Return(0)
//!      └── WaitResult::Interrupted (被信号打断)
//!                 │
//!                 ▼
//!   返回 Outcome::Restart(Restart::Sleep { request, deadline, remaining })
//!      ├── 无用户信号处理函数 / 经 restart_syscall 恢复 ──► resume_sleep(deadline) (沿用原 deadline 继续睡眠)
//!      └── 决定向用户态信号处理函数交付 EINTR ──────────► interrupt_sleep(deadline, remaining)
//!                                                         (若为相对睡眠且 rem 非空，写出剩余时间后返回 EINTR)
//! ```
//!
//! ## 关键 ABI 内存布局（LP64）
//! - `struct timespec`（16 字节）：`tv_sec: i64`（偏移 0..8），`tv_nsec: i64`（偏移 8..16，范围 `0..1_000_000_000`）。
//! - `struct timeval`（16 字节）：`tv_sec: i64`（偏移 0..8），`tv_usec: i64`（偏移 8..16，范围 `0..1_000_000`）。
//! - `struct itimerval`（32 字节）：`it_interval: timeval`（偏移 0..16），`it_value: timeval`（偏移 16..32）。
//! - `struct tms`（32 字节）：依次为 `tms_utime`、`tms_stime`、`tms_cutime`、`tms_cstime`（各 8 字节 `clock_t`，单位为 `1 / USER_HZ` 秒）。
//! - `struct rusage`（144 字节）：`ru_utime: timeval`（偏移 0..16），`ru_stime: timeval`（偏移 16..32），其余统计字段（偏移 32..144）全部填零。
//!
//! ## 锁与并发契约
//! - 所有用户态内存读写（`copyin` / `copyout`）均在栈上完全初始化的字节数组与用户地址空间之间完成，不在持有任务时间锁或定时器自旋锁期间访问用户内存。
//! - 阻塞睡眠期间不借用任何用户空间指针或地址空间锁。

use core::convert::{TryFrom, TryInto};

use crate::accounting;
use crate::syscall::error::{self, Errno, SysResult};
use crate::syscall::{Outcome, Request, Restart};
use crate::task::time::{CpuTime, ItimerValue, ITIMER_PROF, ITIMER_REAL, USER_HZ};
use crate::task::UserThread;
use crate::time::{self, MonotonicInstant, NSEC_PER_SEC, NSEC_PER_USEC, USEC_PER_SEC};
use crate::timer;
use crate::wait::WaitResult;

const TIMER_ABSTIME: usize = 1;

const CLOCK_REALTIME: i32 = 0;
const CLOCK_MONOTONIC: i32 = 1;
const CLOCK_PROCESS_CPUTIME_ID: i32 = 2;
const CLOCK_THREAD_CPUTIME_ID: i32 = 3;
const CLOCK_REALTIME_COARSE: i32 = 5;
const CLOCK_MONOTONIC_COARSE: i32 = 6;

const RUSAGE_SELF: i32 = 0;
const RUSAGE_CHILDREN: i32 = -1;
const RUSAGE_THREAD: i32 = 1;

/// 时间相关系统调用的统一分派入口。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。进入分派前会先结算当前任务的运行态 CPU 时间并刷写 CPU 定时器状态。
/// - **参数**：`user` 为当前用户线程，`request` 包含系统调用号与 6 个原始寄存器参数。
/// - **返回值**：普通系统调用返回 `Outcome::Return`（错误码在此处统一编码为负值）；若睡眠被信号打断，则返回 `Outcome::Restart` 交由外层信号处理路径决策。
pub(super) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    accounting::settle_current();
    user.user_time()
        .expect("time syscall has no user time")
        .flush_cpu_timers();

    let raw = request.arguments;
    let result = match request.number {
        101 | 115 => return sleep(user, request),
        102 => getitimer(user, raw),
        103 => setitimer(user, raw),
        113 => clock_gettime(user, raw),
        114 => clock_getres(user, raw),
        153 => times(user, raw[0]),
        165 => getrusage(user, raw),
        169 => gettimeofday(user, raw),
        _ => Err(Errno::NoSyscall),
    };
    Outcome::Return(result.unwrap_or_else(Errno::result))
}

/// 继续执行此前已建立绝对截止时间的睡眠请求（供首次睡眠、无信号处理函数的自动重试以及 `restart_syscall` 共同调用）。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。直接使用已保存的 `deadline`，不重新读取用户空间的相对时长参数。
/// - **参数**：`request` 为原始系统调用请求，`deadline` 为单调时钟绝对到期时刻，`remaining` 为相对睡眠时的用户态 `rem` 指针（绝对睡眠或空指针时为 `None`）。
/// - **返回值**：到期唤醒返回 `Outcome::Return(0)`；被信号唤醒返回 `Outcome::Restart(Restart::Sleep { .. })`。
pub(super) fn resume_sleep(
    _user: &UserThread,
    request: Request,
    deadline: MonotonicInstant,
    remaining: Option<usize>,
) -> Outcome {
    match timer::sleep_until_interruptible(deadline) {
        WaitResult::Woken => Outcome::Return(0),
        WaitResult::Interrupted => Outcome::Restart(Restart::Sleep {
            request,
            deadline,
            remaining,
        }),
    }
}

/// 在决定中断睡眠并向用户态交付信号（返回 `EINTR`）时，计算并向用户空间写回剩余睡眠时间。
///
/// - **调用上下文**：信号处理准备路径（普通可睡眠用户任务上下文）。
/// - **参数**：`deadline` 为睡眠的绝对截止时间，`remaining` 为相对睡眠传入的非空 `rem` 缓冲区地址（若为 `CLOCK_NANOSLEEP` 绝对睡眠或 `rem == NULL` 则为 `None`）。
/// - **返回值**：若当前时间已达到或超过 `deadline`，视为睡眠已完成并返回 `Ok(0)`；否则在写回 `rem`（若需要）后返回 `Err(Errno::Interrupted)`。
pub(crate) fn interrupt_sleep(
    user: &UserThread,
    deadline: MonotonicInstant,
    remaining: Option<usize>,
) -> SysResult {
    let now = time::now();
    if deadline <= now {
        return Ok(0);
    }
    if let Some(address) = remaining {
        let mm = user.mm().expect("interrupted sleep has no MM");
        error::copyout(
            &mm,
            address,
            &timespec(deadline.as_nanos() - now.as_nanos()),
        )?;
    }
    Err(Errno::Interrupted)
}

/// `nanosleep`（101）与 `clock_nanosleep`（115）系统调用入口：校验时钟与标志位，读取用户请求的睡眠时间并换算为单调时钟绝对截止时间。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。若时钟类型、标志位或用户态 `timespec` 非法，直接返回错误而不登记任何定时器等待。
fn sleep(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    // Step 1: 区分 nanosleep 与 clock_nanosleep 的参数位置，校验时钟 ID 与 TIMER_ABSTIME 标志。
    let parsed = (|| {
        let (clock, absolute, address, remaining) = if request.number == 101 {
            (CLOCK_MONOTONIC, false, raw[0], raw[1])
        } else {
            validate_clock(raw[0])?;
            if raw[1] & !TIMER_ABSTIME != 0 {
                return Err(Errno::InvalidArgument);
            }
            (raw[0] as i32, raw[1] == TIMER_ABSTIME, raw[2], raw[3])
        };
        if clock != CLOCK_REALTIME && clock != CLOCK_MONOTONIC {
            return Err(Errno::Unsupported);
        }

        // Step 2: 从用户空间拷贝 16 字节 timespec 并校验秒与纳秒字段范围。
        let mm = user.mm().expect("sleep has no MM");
        let mut bytes = [0; 16];
        error::copyin(&mm, &mut bytes, address)?;
        let ns = decode_time(&bytes, NSEC_PER_SEC, 1)?;

        // Step 3: 将请求时长换算为内部单调时钟的绝对截止时间 deadline；仅相对睡眠且 rem 非空时记录 remaining 地址。
        let deadline = if absolute && clock == CLOCK_REALTIME {
            time::realtime_deadline(ns)
        } else if absolute {
            MonotonicInstant::from_nanos(ns)
        } else {
            MonotonicInstant::from_nanos(
                time::now()
                    .as_nanos()
                    .checked_add(ns)
                    .ok_or(Errno::Overflow)?,
            )
        };
        Ok((
            deadline,
            if absolute || remaining == 0 {
                None
            } else {
                Some(remaining)
            },
        ))
    })();

    // Step 4: 参数解析成功后进入可中断睡眠等待，失败则直接返回对应负 errno。
    match parsed {
        Ok((deadline, remaining)) => resume_sleep(user, request, deadline, remaining),
        Err(error) => Outcome::Return(error.result()),
    }
}

/// 校验用户传入的 POSIX 时钟 ID 是否受当前内核支持。
///
/// 支持实时日历时钟（`CLOCK_REALTIME` / `CLOCK_REALTIME_COARSE`）、单调时钟（`CLOCK_MONOTONIC` / `CLOCK_MONOTONIC_COARSE`）以及进程/线程 CPU 时间时钟（`CLOCK_PROCESS_CPUTIME_ID` / `CLOCK_THREAD_CPUTIME_ID`）。
fn validate_clock(raw: usize) -> Result<i32, Errno> {
    let clock = raw as i32;
    if matches!(
        clock,
        CLOCK_REALTIME
            | CLOCK_MONOTONIC
            | CLOCK_PROCESS_CPUTIME_ID
            | CLOCK_THREAD_CPUTIME_ID
            | CLOCK_REALTIME_COARSE
            | CLOCK_MONOTONIC_COARSE
    ) {
        Ok(clock)
    } else {
        Err(Errno::InvalidArgument)
    }
}

/// 读取指定 POSIX 时钟的当前时间值（单位：纳秒）。
///
/// - 实时时钟基于启动时的 RTC 基准时间加上单调经过时间计算；
/// - `_COARSE` 变体直接复用高精度时钟源读取样本；
/// - 进程与线程 CPU 时钟读取已结算的用户态与内核态累计 CPU 纳秒数之和。
fn read_clock(user: &UserThread, raw: usize) -> Result<u64, Errno> {
    match validate_clock(raw)? {
        CLOCK_REALTIME | CLOCK_REALTIME_COARSE => Ok(time::realtime()),
        CLOCK_MONOTONIC | CLOCK_MONOTONIC_COARSE => Ok(time::now().as_nanos()),
        CLOCK_PROCESS_CPUTIME_ID => Ok(user
            .user_time()
            .expect("process clock has no time")
            .group()
            .snapshot()
            .total()),
        CLOCK_THREAD_CPUTIME_ID => Ok(user
            .user_time()
            .expect("thread clock has no time")
            .snapshot()
            .total()),
        _ => unreachable!(),
    }
}

/// `clock_gettime` 系统调用：读取指定时钟的当前纳秒值，并编码为 16 字节 `struct timespec` 写入用户空间。
fn clock_gettime(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let ns = read_clock(user, raw[0])?;
    let mm = user.mm().expect("clock_gettime has no MM");
    error::copyout(&mm, raw[1], &timespec(ns))?;
    Ok(0)
}

/// `clock_getres` 系统调用：查询指定时钟的分辨率，并在 `res` 指针非空时写入用户空间。
///
/// 精细时钟的分辨率由硬件计数器频率（`counter_frequency`）向上取整为纳秒计算（至少 1 纳秒）；`_COARSE` 时钟的分辨率报告为内核定时器基础节拍周期 `TICK_NS`。
fn clock_getres(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let clock = validate_clock(raw[0])?;
    if raw[1] != 0 {
        let frequency = crate::boot::info().counter_frequency;
        assert!(0 < frequency, "clock resolution has no counter frequency");
        let resolution = if matches!(clock, CLOCK_REALTIME_COARSE | CLOCK_MONOTONIC_COARSE) {
            timer::TICK_NS
        } else {
            NSEC_PER_SEC / frequency + u64::from(NSEC_PER_SEC % frequency != 0)
        };
        let mm = user.mm().expect("clock_getres has no MM");
        error::copyout(&mm, raw[1], &timespec(resolution.max(1)))?;
    }
    Ok(0)
}

/// `gettimeofday` 系统调用：向用户空间写入当前 UTC 实时时间（`struct timeval`）与时区结构（`struct timezone`）。
///
/// 若 `tv`（`raw[0]`）非空则写出微秒精度的 UTC 时间；内核不维护本地时区偏移，若 `tz`（`raw[1]`）非空则写出全零的 8 字节结构。
fn gettimeofday(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let mm = user.mm().expect("gettimeofday has no MM");
    if raw[0] != 0 {
        error::copyout(&mm, raw[0], &timeval(time::realtime(), false))?;
    }
    if raw[1] != 0 {
        error::copyout(&mm, raw[1], &[0; 8])?;
    }
    Ok(0)
}

/// 校验 `getitimer` / `setitimer` 的定时器类型参数 `which`，仅接受 `ITIMER_REAL`（0）、`ITIMER_VIRTUAL`（1）和 `ITIMER_PROF`（2）。
fn which(raw: usize) -> Result<usize, Errno> {
    let which = raw as i32;
    if (ITIMER_REAL as i32..=ITIMER_PROF as i32).contains(&which) {
        Ok(which as usize)
    } else {
        Err(Errno::InvalidArgument)
    }
}

/// `getitimer` 系统调用：获取当前线程组指定间隔定时器的周期与剩余到期时间，并写入用户空间。
///
/// 若定时器仍处于激活状态但剩余纳秒数不足 1 微秒，`itimerval` 编码会向上取整为 1 微秒，避免用户态误判定时器已停止。
fn getitimer(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let which = which(raw[0])?;
    let value = user
        .user_time()
        .expect("getitimer has no time")
        .group()
        .get_itimer(which);
    let mm = user.mm().expect("getitimer has no MM");
    error::copyout(&mm, raw[1], &itimerval(value))?;
    Ok(0)
}

/// `setitimer` 系统调用：解析用户态传入的新 `itimerval` 并更新线程组的间隔定时器，若 `old_value` 指针（`raw[2]`）非空则写回先前的定时器状态。
///
/// 只有当 `it_interval` 与 `it_value` 两个 `timeval` 字段均通过合法性校验后才会修改定时器状态；若后续向 `old_value` 拷贝失败，已生效的新定时器状态不会回滚。
fn setitimer(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let which = which(raw[0])?;
    let mm = user.mm().expect("setitimer has no MM");
    let mut bytes = [0; 32];
    error::copyin(&mm, &mut bytes, raw[1])?;
    let value = ItimerValue {
        interval_ns: decode_time(&bytes[..16], USEC_PER_SEC, NSEC_PER_USEC)?,
        value_ns: decode_time(&bytes[16..], USEC_PER_SEC, NSEC_PER_USEC)?,
    };

    let old = user
        .user_time()
        .expect("setitimer has no time")
        .group()
        .set_itimer(which, value)
        .ok_or(Errno::Overflow)?;

    if raw[2] != 0 {
        error::copyout(&mm, raw[2], &itimerval(old))?;
    }
    Ok(0)
}

/// `times` 系统调用：将当前进程组及已回收（reap）子进程的用户态/内核态 CPU 时间写入用户空间 `struct tms`（当 `address != 0` 时），并返回系统启动以来的单调时钟节拍数。
///
/// 所有时间值均按固定的 `USER_HZ`（与底层硬件计数器频率及调度时间片长度无关）换算为 `clock_t` 节拍数。
fn times(user: &UserThread, address: usize) -> SysResult {
    let time = user.user_time().expect("times has no time");
    let own = time.group().snapshot();
    let children = time.group().children();
    if address != 0 {
        let mut bytes = [0; 32];
        for (index, ns) in [
            own.user_ns,
            own.kernel_ns,
            children.user_ns,
            children.kernel_ns,
        ]
        .iter()
        .copied()
        .enumerate()
        {
            bytes[index * 8..index * 8 + 8].copy_from_slice(&ticks(ns)?.to_le_bytes());
        }
        let mm = user.mm().expect("times has no MM");
        error::copyout(&mm, address, &bytes)?;
    }
    Ok(ticks(time::now().as_nanos())? as isize)
}

/// `getrusage` 系统调用：查询进程组自身（`RUSAGE_SELF`）、已回收子进程汇总（`RUSAGE_CHILDREN`）或当前线程（`RUSAGE_THREAD`）的资源使用统计。
///
/// 填入用户态 CPU 时间（`ru_utime`）与内核态 CPU 时间（`ru_stime`），其余未追踪的资源统计字段（偏移 32..144）均填零写出。
fn getrusage(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let time = user.user_time().expect("getrusage has no time");
    let value: CpuTime = match raw[0] as i32 {
        RUSAGE_SELF => time.group().snapshot(),
        RUSAGE_CHILDREN => time.group().children(),
        RUSAGE_THREAD => time.snapshot(),
        _ => return Err(Errno::InvalidArgument),
    };

    let mut bytes = [0; 144];
    bytes[..16].copy_from_slice(&timeval(value.user_ns, false));
    bytes[16..32].copy_from_slice(&timeval(value.kernel_ns, false));
    let mm = user.mm().expect("getrusage has no MM");
    error::copyout(&mm, raw[1], &bytes)?;
    Ok(0)
}

/// 解码用户空间传入的 16 字节时间结构（`timespec` 或 `timeval`），校验秒与亚秒字段非负且亚秒未越界，并安全换算为总纳秒数。
///
/// - **参数**：`bytes` 为 16 字节小端数据，`units_per_sec` 为每秒包含的亚秒单位数（纳秒为 $10^9$，微秒为 $10^6$），`ns_per_unit` 为每个亚秒单位对应的纳秒数（`timespec` 为 1，`timeval` 为 1000）。
/// - **返回值**：成功返回总纳秒数 `Ok(u64)`；负值或亚秒 >= `units_per_sec` 返回 `Err(Errno::InvalidArgument)`；算术溢出 `u64` 返回 `Err(Errno::Overflow)`。
pub(super) fn decode_time(
    bytes: &[u8],
    units_per_sec: u64,
    ns_per_unit: u64,
) -> Result<u64, Errno> {
    let seconds = i64::from_le_bytes(bytes[..8].try_into().expect("time seconds width"));
    let fraction = i64::from_le_bytes(bytes[8..16].try_into().expect("time fraction width"));
    if seconds < 0 || fraction < 0 || units_per_sec <= fraction as u64 {
        return Err(Errno::InvalidArgument);
    }
    (seconds as u64)
        .checked_mul(NSEC_PER_SEC)
        .and_then(|ns| ns.checked_add((fraction as u64).checked_mul(ns_per_unit)?))
        .ok_or(Errno::Overflow)
}

/// 将纳秒数 `ns` 编码为 LP64 ABI 的 16 字节小端 `struct timespec`（前 8 字节为秒 `tv_sec`，后 8 字节为纳秒 `tv_nsec`）。
fn timespec(ns: u64) -> [u8; 16] {
    let mut bytes = [0; 16];
    bytes[..8].copy_from_slice(&((ns / NSEC_PER_SEC) as i64).to_le_bytes());
    bytes[8..].copy_from_slice(&((ns % NSEC_PER_SEC) as i64).to_le_bytes());
    bytes
}

/// 将纳秒数 `ns` 编码为 LP64 ABI 的 16 字节小端 `struct timeval`（前 8 字节为秒 `tv_sec`，后 8 字节为微秒 `tv_usec`）。
///
/// 当 `round_up` 为 `false`（如墙钟时间与 CPU 统计）时直接向零截断至微秒；当 `round_up` 为 `true`（如间隔定时器剩余时间）时对不足 1 微秒的余数向上取整，防止未到期定时器显示为 `0`（已停止）。
fn timeval(ns: u64, round_up: bool) -> [u8; 16] {
    let micros = ns / NSEC_PER_USEC + u64::from(round_up && ns % NSEC_PER_USEC != 0);
    let mut bytes = [0; 16];
    bytes[..8].copy_from_slice(&((micros / USEC_PER_SEC) as i64).to_le_bytes());
    bytes[8..].copy_from_slice(&((micros % USEC_PER_SEC) as i64).to_le_bytes());
    bytes
}

/// 将内核间隔定时器状态 `ItimerValue` 编码为 32 字节小端 `struct itimerval`（偏移 0..16 为 `it_interval`，偏移 16..32 为向上取整的 `it_value`）。
fn itimerval(value: ItimerValue) -> [u8; 32] {
    let mut bytes = [0; 32];
    bytes[..16].copy_from_slice(&timeval(value.interval_ns, false));
    bytes[16..].copy_from_slice(&timeval(value.value_ns, true));
    bytes
}

/// 将纳秒数按固定 `USER_HZ` 换算为 `clock_t` 节拍数（先除以每节拍纳秒数 `NSEC_PER_SEC / USER_HZ` 以避免乘法溢出）。
fn ticks(ns: u64) -> Result<i64, Errno> {
    let ticks = ns / (NSEC_PER_SEC / USER_HZ);
    i64::try_from(ticks).map_err(|_| Errno::Overflow)
}
