//! futex ABI只解码本次raw参数；比较、选择与取消由同一真实等待源承担。

use crate::syscall::error::{self, Errno};
use crate::syscall::{time, Outcome, Request, Restart};
use crate::task::futex::{self, WaitOutcome};
use crate::task::UserThread;
use crate::time::{self as clock, MonotonicInstant, NSEC_PER_SEC};

const WAIT: u32 = 0;
const WAKE: u32 = 1;
const REQUEUE: u32 = 3;
const CMP_REQUEUE: u32 = 4;
const WAIT_BITSET: u32 = 9;
const WAKE_BITSET: u32 = 10;
const PRIVATE: u32 = 0x80;
const CLOCK_REALTIME: u32 = 0x100;
const COMMAND_MASK: u32 = !(PRIVATE | CLOCK_REALTIME);

/// 只接受真实非PI操作；timeout指针和requeue整数val2按各自ABI解释。
pub(super) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    let operation = raw[1] as u32;
    let command = operation & COMMAND_MASK;
    let private = operation & PRIVATE != 0;
    if matches!(command, WAIT | WAIT_BITSET) {
        let parsed = parse_deadline(user, raw[3], operation);
        let deadline = match parsed {
            Ok(deadline) => deadline,
            Err(error) => return Outcome::Return(error.result()),
        };
        if operation & CLOCK_REALTIME != 0 && command != WAIT_BITSET {
            return Outcome::Return(Errno::NoSyscall.result());
        }
        return wait(user, request, deadline);
    }
    if operation & CLOCK_REALTIME != 0 {
        return Outcome::Return(Errno::NoSyscall.result());
    }
    let result = (|| {
        let mm = user.mm().expect("futex has no MM");
        match command {
            WAKE | WAKE_BITSET => futex::wake(
                &mm,
                raw[0],
                private,
                (raw[2] as u32 as i32).max(1) as usize,
                if command == WAKE {
                    u32::MAX
                } else {
                    raw[5] as u32
                },
            )
            .map_err(Errno::from),
            REQUEUE | CMP_REQUEUE => futex::requeue(
                &mm,
                raw[0],
                raw[4],
                private,
                quota(raw[2])?,
                quota(raw[3])?,
                (command == CMP_REQUEUE).then_some(raw[5] as u32),
            )
            .map_err(Errno::from),
            _ => Err(Errno::NoSyscall),
        }
    })();
    Outcome::Return(
        result
            .map(|count| count as isize)
            .unwrap_or_else(Errno::result),
    )
}

/// restart_syscall消费原PC/参数及首次deadline，不重新访问旧timeout指针。
pub(super) fn resume_wait(
    user: &UserThread,
    request: Request,
    deadline: MonotonicInstant,
) -> Outcome {
    wait(user, request, Some(deadline))
}

/// 已选择wake优先于信号/超时；未被选择的中断在原合法返回点裁决restart。
fn wait(user: &UserThread, request: Request, deadline: Option<MonotonicInstant>) -> Outcome {
    let raw = request.arguments;
    let operation = raw[1] as u32;
    let mm = user.mm().expect("futex wait has no MM");
    let mask = if operation & COMMAND_MASK == WAIT {
        u32::MAX
    } else {
        raw[5] as u32
    };
    match futex::wait(
        &mm,
        raw[0],
        operation & PRIVATE != 0,
        raw[2] as u32,
        mask,
        deadline,
    ) {
        Ok(WaitOutcome::Woken) => Outcome::Return(0),
        Ok(WaitOutcome::TimedOut) => Outcome::Return(Errno::TimedOut.result()),
        Ok(WaitOutcome::Interrupted) => Outcome::Restart(match deadline {
            Some(deadline) => Restart::Futex { request, deadline },
            None => Restart::Syscall { request },
        }),
        Err(error) => Outcome::Return(Errno::from(error).result()),
    }
}

/// 重排的32位ABI配额拒绝负数；旧WAKE则在选择首项后才检查配额。
fn quota(raw: usize) -> Result<usize, Errno> {
    let count = raw as u32 as i32;
    if count < 0 {
        Err(Errno::InvalidArgument)
    } else {
        Ok(count as usize)
    }
}

/// WAIT 相对时长只换算一次；BITSET 按本次时钟位将绝对期限投影到单调等待域。
fn parse_deadline(
    user: &UserThread,
    address: usize,
    operation: u32,
) -> Result<Option<MonotonicInstant>, Errno> {
    if address == 0 {
        return Ok(None);
    }
    let mm = user.mm().expect("futex timeout has no MM");
    let mut bytes = [0; 16];
    error::copyin(&mm, &mut bytes, address)?;
    let ns = time::decode_time(&bytes, NSEC_PER_SEC, 1)?;
    let deadline = if operation & COMMAND_MASK == WAIT {
        MonotonicInstant::from_nanos(
            clock::now()
                .as_nanos()
                .checked_add(ns)
                .ok_or(Errno::Overflow)?,
        )
    } else if operation & CLOCK_REALTIME != 0 {
        clock::realtime_deadline(ns)
    } else {
        MonotonicInstant::from_nanos(ns)
    };
    Ok(Some(deadline))
}
