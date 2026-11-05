//! 信号ABI只解释完整初始化的字节快照，领取和状态提交不因后续交付错误回滚。

use core::convert::TryInto;

use crate::mm::space::Mm;
use crate::syscall::error::{copyin, copyout, Errno, SysResult};
use crate::syscall::{Outcome, Request, Restart};
use crate::task::signal::{
    AltStack, SignalAction, SignalError, SignalInfo, SignalPayload, SIGINFO_BYTES, SIGNAL_COUNT,
    SIGSEGV, SIGSET_BYTES, UNBLOCKABLE,
};
use crate::task::UserThread;
use crate::time::{self, Duration, MonotonicInstant, NSEC_PER_SEC};
use crate::wait::{WaitMode, WaitResult, Waiter};

#[cfg(target_arch = "riscv64")]
const STACK_REGISTER: usize = 2;
#[cfg(target_arch = "loongarch64")]
const STACK_REGISTER: usize = 3;

const SIG_BLOCK: i32 = 0;
const SIG_UNBLOCK: i32 = 1;
const SIG_SETMASK: i32 = 2;

/// 分派实际信号调用；恢复现场和中断重启不经过普通返回尾部。
pub(crate) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    if request.number == 139 {
        return sigreturn(user);
    }
    if request.number == 133 {
        return suspend(user, request);
    }

    let raw = request.arguments;
    let result = match request.number {
        129..=131 => send(user, request.number, raw),
        132 => altstack(user, raw),
        134 => action(user, raw),
        135 => mask(user, raw),
        136 => pending(user, raw),
        137 => timedwait(user, raw),
        138 | 240 => queue(user, request.number, raw),
        _ => Err(Errno::NoSyscall),
    };
    Outcome::Return(result.unwrap_or_else(Errno::result))
}

/// 在用户ABI边界转换原信号错误，不另建内核errno体系。
fn signal_error(error: SignalError) -> Errno {
    match error {
        SignalError::InvalidSignal | SignalError::InvalidStack => Errno::InvalidArgument,
        SignalError::OnStack => Errno::NotPermitted,
        SignalError::StackTooSmall => Errno::NoMemory,
        SignalError::QueueFull => Errno::Again,
        SignalError::NoMemory => Errno::NoMemory,
        SignalError::NoProcess => Errno::NoProcess,
        SignalError::Permission => Errno::NotPermitted,
    }
}

/// 信号号码先按int解释再验证，不能将超范围整数截断成有效信号。
fn signum(raw: usize) -> Result<u8, Errno> {
    let signal = raw as i32;
    if !(0..=SIGNAL_COUNT as i32).contains(&signal) {
        return Err(Errno::InvalidArgument);
    }
    Ok(signal as u8)
}

/// kill的组角色与tkill/tgkill的线程角色由原TaskGraph一次解析和检查权限。
fn send(user: &UserThread, number: usize, raw: [usize; 6]) -> SysResult {
    let identity = user.identity().expect("signal sender has no identity");
    let graph = identity.graph();
    match number {
        129 => graph.send_signal(&identity, raw[0] as i32, signum(raw[1])?),
        130 => {
            let tid = raw[0] as i32;
            if tid <= 0 {
                return Err(Errno::InvalidArgument);
            }
            graph.send_thread_signal(&identity, tid as usize, None, signum(raw[1])?)
        }
        131 => {
            let tgid = raw[0] as i32;
            let tid = raw[1] as i32;
            if tgid <= 0 || tid <= 0 {
                return Err(Errno::InvalidArgument);
            }
            graph.send_thread_signal(
                &identity,
                tid as usize,
                Some(tgid as usize),
                signum(raw[2])?,
            )
        }
        _ => unreachable!(),
    }
    .map_err(signal_error)?;
    Ok(0)
}

/// 完整快照24字节动作后一次替换；旧动作copyout失败不撤已生效的新动作。
fn action(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[3] != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    let mm = user.mm().expect("sigaction has no MM");
    let identity = user.identity().expect("sigaction has no identity");
    let replacement = if raw[1] == 0 {
        None
    } else {
        let mut bytes = [0u8; 24];
        copyin(&mm, &mut bytes, raw[1])?;
        Some(SignalAction {
            handler: word(&bytes, 0) as usize,
            flags: word(&bytes, 8) as usize,
            mask: word(&bytes, 16),
        })
    };

    let previous = identity
        .graph()
        .update_action(&identity, raw[0] as i32 as usize, replacement)
        .map_err(signal_error)?;

    if raw[2] != 0 {
        let mut bytes = [0u8; 24];
        bytes[0..8].copy_from_slice(&(previous.handler as u64).to_le_bytes());
        bytes[8..16].copy_from_slice(&(previous.flags as u64).to_le_bytes());
        bytes[16..24].copy_from_slice(&previous.mask.to_le_bytes());
        copyout(&mm, raw[2], &bytes)?;
    }
    Ok(0)
}

/// mask提交去除不可阻断位；查询不要求how有效，新值输入先完整取得。
fn mask(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[3] != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    let mm = user.mm().expect("sigprocmask has no MM");
    let signals = user.signals().expect("sigprocmask has no signals");
    let mut old = signals.mask();
    if raw[1] != 0 {
        let value = sigset(&mm, raw[1])?;
        let replacement = match raw[0] as i32 {
            SIG_BLOCK => old | value,
            SIG_UNBLOCK => old & !value,
            SIG_SETMASK => value,
            _ => return Err(Errno::InvalidArgument),
        };
        old = signals.replace_mask(replacement);
    }

    if raw[2] != 0 {
        copyout(&mm, raw[2], &old.to_le_bytes())?;
    }
    Ok(0)
}

/// 只报告当前线程阻塞的private和group pending号码集合。
fn pending(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[1] != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    let mm = user.mm().expect("sigpending has no MM");
    let bits = user.signals().expect("sigpending has no signals").pending();
    copyout(&mm, raw[0], &bits.to_le_bytes())?;
    Ok(0)
}

/// stack_t的padding不进入Rust对象；ONSTACK由当前用户SP计算，设置与查询同锁。
fn altstack(user: &UserThread, raw: [usize; 6]) -> SysResult {
    let mm = user.mm().expect("sigaltstack has no MM");
    let sp = user.context().gpr[STACK_REGISTER];
    let replacement = if raw[0] == 0 {
        None
    } else {
        let mut bytes = [0u8; 24];
        copyin(&mm, &mut bytes, raw[0])?;
        Some(AltStack {
            base: word(&bytes, 0) as usize,
            flags: u32::from_le_bytes(bytes[8..12].try_into().expect("stack flags width")),
            size: word(&bytes, 16) as usize,
        })
    };

    let previous = user
        .signals()
        .expect("sigaltstack has no signals")
        .altstack(sp, replacement)
        .map_err(signal_error)?;

    if raw[1] != 0 {
        let mut bytes = [0u8; 24];
        bytes[0..8].copy_from_slice(&(previous.base as u64).to_le_bytes());
        bytes[8..12].copy_from_slice(&previous.flags.to_le_bytes());
        bytes[16..24].copy_from_slice(&(previous.size as u64).to_le_bytes());
        copyout(&mm, raw[1], &bytes)?;
    }
    Ok(0)
}

/// 临时mask跨停止/继续保留，实际handler帧保存原mask；无handler只重放本次原请求。
fn suspend(user: &UserThread, request: Request) -> Outcome {
    let result = (|| -> Result<(), Errno> {
        if request.arguments[1] != SIGSET_BYTES {
            return Err(Errno::InvalidArgument);
        }
        let mm = user.mm().expect("sigsuspend has no MM");
        let value = sigset(&mm, request.arguments[0])?;
        let signals = user.signals().expect("sigsuspend has no signals");
        user.save_mask(signals.replace_mask(value));

        loop {
            let waiter = Waiter::new_interruptible();
            if signals.interruption_pending(WaitMode::Interruptible) {
                break;
            }
            if waiter.park_signal() == WaitResult::Interrupted {
                break;
            }
        }
        Ok(())
    })();

    match result {
        Ok(()) => Outcome::Restart(Restart::NoHandler { request }),
        Err(error) => Outcome::Return(error.result()),
    }
}

/// 期限只建立一次；暂时解除所请求信号的阻断以复用唯一Waiter通知，领取后不回队。
fn timedwait(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[3] != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    let mm = user.mm().expect("sigtimedwait has no MM");
    let set = sigset(&mm, raw[0])? & !UNBLOCKABLE;
    let deadline = if raw[2] == 0 {
        None
    } else {
        Some(timeout(&mm, raw[2])?)
    };

    let signals = user.signals().expect("sigtimedwait has no signals");
    let mut selected = signals.take_waited(set);
    if selected.is_none() && deadline.is_none_or(|limit| time::now() < limit) {
        let old = signals.begin_waited(set);
        loop {
            let result = if let Some(deadline) = deadline {
                crate::timer::sleep_until_interruptible(deadline)
            } else {
                let waiter = Waiter::new_interruptible();
                selected = signals.take_waited(set);
                if selected.is_some() {
                    break;
                }
                waiter.park_signal()
            };
            selected = signals.take_waited(set);
            if selected.is_some()
                || result == WaitResult::Interrupted
                || deadline.is_some_and(|limit| limit <= time::now())
            {
                break;
            }
        }
        signals.end_waited(old);
    }

    if let Some(info) = selected {
        if raw[1] != 0 {
            let mut bytes = [0u8; SIGINFO_BYTES];
            info.encode(&mut bytes);
            copyout(&mm, raw[1], &bytes)?;
        }
        return Ok(info.signum as isize);
    }
    Err(if signals.interruption_pending(WaitMode::Interruptible) {
        Errno::Interrupted
    } else {
        Errno::Again
    })
}

/// 用户队列info完整读取，目标身份、权限和可伪装si_code由原图发送边界检查。
fn queue(user: &UserThread, number: usize, raw: [usize; 6]) -> SysResult {
    let identity = user.identity().expect("queue sender has no identity");
    let mm = user.mm().expect("sigqueue has no MM");
    let (pid, tid, signal, address) = if number == 138 {
        (raw[0] as i32, None, signum(raw[1])?, raw[2])
    } else {
        (raw[0] as i32, Some(raw[1] as i32), signum(raw[2])?, raw[3])
    };
    if pid <= 0 || tid.is_some_and(|tid| tid <= 0) {
        return Err(Errno::InvalidArgument);
    }

    let mut bytes = [0u8; SIGINFO_BYTES];
    copyin(&mm, &mut bytes, address)?;
    let info = SignalInfo {
        signum: signal,
        errno: i32::from_le_bytes(bytes[4..8].try_into().expect("siginfo errno width")),
        code: i32::from_le_bytes(bytes[8..12].try_into().expect("siginfo code width")),
        payload: SignalPayload::Sender {
            pid: i32::from_le_bytes(bytes[16..20].try_into().expect("siginfo pid width")) as usize,
            uid: u32::from_le_bytes(bytes[20..24].try_into().expect("siginfo uid width")),
            value: word(&bytes, 24) as usize,
        },
    };

    if let Some(tid) = tid {
        identity
            .graph()
            .send_thread_queue(&identity, tid as usize, pid as usize, info)
    } else {
        identity.graph().send_queue(&identity, pid as usize, info)
    }
    .map_err(signal_error)?;
    Ok(0)
}

/// 完整decode成功后一次提交用户现场与mask/altstack；坏帧有限致命SIGSEGV。
fn sigreturn(user: &UserThread) -> Outcome {
    let _ = user.take_restart();
    let mm = user.mm().expect("sigreturn has no MM");
    let context = user.context().fork();
    match context.restore_signal(&mm) {
        Ok(restored) => {
            let sp = restored.context.gpr[STACK_REGISTER];
            let signals = user.signals().expect("sigreturn has no signals");
            signals.restore(restored.mask, restored.altstack, sp);
            *user.context() = restored.context;
            if let Some(info) = restored.pending_signal {
                signals.force(info);
            }
            Outcome::Restored
        }
        Err(_) => {
            let identity = user.identity().expect("sigreturn has no identity");
            identity.graph().request_exit(&identity, SIGSEGV as i32);
            Outcome::GroupExit(SIGSEGV as i32)
        }
    }
}

/// 读取已初始化字节中的64位little-endian ABI字段。
fn word(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("ABI word width"),
    )
}

/// 两目标sigset_t固定8字节，输入不建立用户引用。
fn sigset(mm: &Mm, address: usize) -> Result<u64, Errno> {
    let mut bytes = [0u8; SIGSET_BYTES];
    copyin(mm, &mut bytes, address)?;
    Ok(u64::from_le_bytes(bytes))
}

/// sigtimedwait的相对timespec只计算一次单调deadline，负值或溢出真实失败。
fn timeout(mm: &Mm, address: usize) -> Result<MonotonicInstant, Errno> {
    let mut bytes = [0u8; 16];
    copyin(mm, &mut bytes, address)?;
    let seconds = word(&bytes, 0) as i64;
    let nanos = word(&bytes, 8) as i64;
    if seconds < 0 || !(0..NSEC_PER_SEC as i64).contains(&nanos) {
        return Err(Errno::InvalidArgument);
    }
    time::now()
        .checked_add(Duration::new(seconds as u64, nanos as u32))
        .ok_or(Errno::Overflow)
}
