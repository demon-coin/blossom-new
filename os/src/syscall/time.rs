//! 时间 ABI 使用完整初始化字节，睡眠重启保留首次绝对期限，用户现场不跨阻塞借用。

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

/// 分派真实时间调用，错误只在普通返回处编码，内部重启保持控制结果。
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

/// restart_syscall 和无 handler 的重试只消费原请求和已保存期限，不重读相对时长。
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

/// 决定向 handler 交付 EINTR 时复制相对剩余时间；绝对睡眠完全不访问 rem。
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

/// 在 ABI 边界只建立一次 deadline，clock 与 flags 错误不登记任何等待。
fn sleep(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
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

        let mm = user.mm().expect("sleep has no MM");
        let mut bytes = [0; 16];
        error::copyin(&mm, &mut bytes, address)?;
        let ns = decode_time(&bytes, NSEC_PER_SEC, 1)?;

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

    match parsed {
        Ok((deadline, remaining)) => resume_sleep(user, request, deadline, remaining),
        Err(error) => Outcome::Return(error.result()),
    }
}

/// 只公开 RTC 日历、原 counter 时间域及真实 CPU 账支持的具体 clock IDs。
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

/// 日历沿启动 RTC 与单调经过时间推进，coarse 查询可交付更细的实际样本。
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

/// clock_gettime 读取真实已结算账，完整交付 64-bit timespec。
fn clock_gettime(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let ns = read_clock(user, raw[0])?;
    let mm = user.mm().expect("clock_gettime has no MM");
    error::copyout(&mm, raw[1], &timespec(ns))?;
    Ok(0)
}

/// 精细 clock 使用 counter 分辨率，coarse 声明既有基础 tick，不建立周期缓存。
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

/// timeval 交付实际 UTC，timezone 保持无本地时区配置的零字段。
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

/// itimer selector 是 ABI int，只接受三个真实 timer。
fn which(raw: usize) -> Result<usize, Errno> {
    let which = raw as i32;
    if (ITIMER_REAL as i32..=ITIMER_PROF as i32).contains(&which) {
        Ok(which as usize)
    } else {
        Err(Errno::InvalidArgument)
    }
}

/// 获取组拥有的 timer 相对剩余值，过期未消费保留最小非零 timeval。
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

/// 两个 timeval 完整解析后才修改 timer，旧值 copyout 失败不撤回已生效的新状态。
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

/// tms 的四项来自组账与实际 reap 的子账，USER_HZ 独立于硬件频率和调度量子。
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

/// getrusage 支持自身、已收集子进程及本线程的真实 CPU 账，未实现资源统计保持零。
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

/// ABI 的有符号秒和亚秒在相乘前验证，内部有限纳秒范围不接受溢出。
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

/// 有限纳秒完整编码为双 64-bit timespec，内部范围保证秒可表示。
fn timespec(ns: u64) -> [u8; 16] {
    let mut bytes = [0; 16];
    bytes[..8].copy_from_slice(&((ns / NSEC_PER_SEC) as i64).to_le_bytes());
    bytes[8..].copy_from_slice(&((ns % NSEC_PER_SEC) as i64).to_le_bytes());
    bytes
}

/// CPU/墙钟 timeval 截断至微秒，timer 剩余值向上取整避免伪装已停。
fn timeval(ns: u64, round_up: bool) -> [u8; 16] {
    let micros = ns / NSEC_PER_USEC + u64::from(round_up && ns % NSEC_PER_USEC != 0);
    let mut bytes = [0; 16];
    bytes[..8].copy_from_slice(&((micros / USEC_PER_SEC) as i64).to_le_bytes());
    bytes[8..].copy_from_slice(&((micros % USEC_PER_SEC) as i64).to_le_bytes());
    bytes
}

/// interval 和 remaining 均写入初始化的 itimerval 字节，无 ABI padding 泄漏。
fn itimerval(value: ItimerValue) -> [u8; 32] {
    let mut bytes = [0; 32];
    bytes[..16].copy_from_slice(&timeval(value.interval_ns, false));
    bytes[16..].copy_from_slice(&timeval(value.value_ns, true));
    bytes
}

/// USER_HZ 使用独立常数，先除以纳秒刻度避免无必要的乘法溢出。
fn ticks(ns: u64) -> Result<i64, Errno> {
    let ticks = ns / (NSEC_PER_SEC / USER_HZ);
    i64::try_from(ticks).map_err(|_| Errno::Overflow)
}
