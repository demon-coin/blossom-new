//! POSIX / Linux 信号（Signal）系统调用 ABI 编解码与分发模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于系统调用总入口（`crate::syscall::dispatch`）与进程图/线程信号状态机（`crate::task::signal`、`TaskGraph`）之间，
//! 负责处理编号 `129..=139` 及 `240` 的全部信号系统调用：
//! - **信号发送与排队**：`kill`（129）、`tkill`（130）、`tgkill`（131）、`rt_sigqueueinfo`（138）、`rt_tgsigqueueinfo`（240）；
//! - **信号处置与掩码配置**：`sigaltstack`（132）、`rt_sigaction`（134）、`rt_sigprocmask`（135）、`rt_sigpending`（136）；
//! - **同步等待与现场恢复**：`rt_sigsuspend`（133）、`rt_sigtimedwait`（137）、`rt_sigreturn`（139）。
//!
//! ## 核心 ABI 内存布局（64 位小端序 RISC-V / LoongArch）
//! 为避免直接将未对齐或含未初始化填充位（padding）的用户态字节强转为 Rust 结构体导致未定义行为，
//! 本模块一律先通过 `copyin` 将数据读入栈上初始化为零的字节数组，再按显式偏移解析字段：
//!
//! ```text
//! 1. kernel_sigaction (24 字节, RISC-V64 / LoongArch64 无 sa_restorer 字段)
//! +-------------------+-------------------+-------------------+
//! | 0..8: sa_handler  | 8..16: sa_flags   | 16..24: sa_mask   |
//! +-------------------+-------------------+-------------------+
//!
//! 2. stack_t (24 字节, sigaltstack 使用)
//! +-------------------+-----------+-------+-------------------+
//! | 0..8: ss_sp       | 8..12:flg | 12..16| 16..24: ss_size   |
//! +-------------------+-----------+-------+-------------------+
//!                                  (padding)
//!
//! 3. siginfo_t 队列发送前缀 (128 字节中的前 32 字节, rt_[tg]sigqueueinfo 使用)
//! +---------+---------+---------+---------+---------+---------+-------------------+
//! | 0..4    | 4..8    | 8..12   | 12..16  | 16..20  | 20..24  | 24..32            |
//! | si_signo| si_errno| si_code | (pad)   | si_pid  | si_uid  | si_value (sival)  |
//! +---------+---------+---------+---------+---------+---------+-------------------+
//! ```
//!
//! ### 状态提交与交付失败边界
//! 在 `rt_sigaction`、`rt_sigprocmask`、`sigaltstack` 和 `rt_sigtimedwait` 中，内核状态更新（如替换信号动作、消费挂起信号）
//! 与旧值/信号详情的 `copyout` 分为两步完成：一旦内核状态已更新或挂起信号已被摘除，即使后续写回用户缓冲区发生缺页失败（`EFAULT`），
//! 已生效的状态变更或已出队的信号也不会回滚，这与 Linux 内核语义保持一致。
//!
//! ## 锁获取顺序与并发契约
//! - 所有用户态内存读写（`copyin` / `copyout`）均在获取 `TaskGraph` 锁或线程信号锁（`ThreadSignals`）**之前或之后**完成，绝不持锁跨页表拷贝。
//! - 阻塞等待（`sigsuspend` / `sigtimedwait`）使用线程中断等待器 `Waiter`，在检查挂起信号与睡眠挂起之间通过序列/状态重查消除唤醒丢失竞态。

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

/// RISC-V 64 架构用户上下文 `gpr` 数组中栈指针寄存器（`sp` / `x2`）的下标。
#[cfg(target_arch = "riscv64")]
const STACK_REGISTER: usize = 2;
/// LoongArch64 架构用户上下文 `gpr` 数组中栈指针寄存器（`$sp` / `$r3`）的下标。
#[cfg(target_arch = "loongarch64")]
const STACK_REGISTER: usize = 3;

/// `rt_sigprocmask` 操作码：将指定信号集合并入当前阻塞掩码（`old | set`）。
const SIG_BLOCK: i32 = 0;
/// `rt_sigprocmask` 操作码：从当前阻塞掩码中移除指定信号集合（`old & !set`）。
const SIG_UNBLOCK: i32 = 1;
/// `rt_sigprocmask` 操作码：直接用指定信号集合替换当前阻塞掩码。
const SIG_SETMASK: i32 = 2;

/// 信号相关系统调用的二级分发入口。
///
/// - `rt_sigreturn`（139）和 `rt_sigsuspend`（133）需要直接控制用户寄存器现场恢复或无处理函数重启逻辑，单独返回对应的 [`Outcome`]；
/// - 其余信号调用统一返回常规 `SysResult` 并包装为 `Outcome::Return`。
///
/// - **调用上下文**：普通可睡眠系统调用上下文。
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

/// 将信号子系统的内部错误 [`SignalError`] 映射为标准系统调用 [`Errno`]。
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

/// 校验并转换用户传入的信号编号。
///
/// 先按 32 位有符号整数（C `int`）解释，防止高位非零的超范围大整数被直接截断为合法的 `0..=64` 信号号（`0` 用于探测进程存活与权限）。
fn signum(raw: usize) -> Result<u8, Errno> {
    let signal = raw as i32;
    if !(0..=SIGNAL_COUNT as i32).contains(&signal) {
        return Err(Errno::InvalidArgument);
    }
    Ok(signal as u8)
}

/// 处理 `kill`（129）、`tkill`（130）和 `tgkill`（131）系统调用：向目标进程组、进程或特定线程发送信号。
///
/// - `kill(pid, sig)`：按 `pid` 正负/零语义向目标进程或进程组投递组级信号；
/// - `tkill(tid, sig)`：向任意线程组中 TID 匹配的线程投递线程定向信号（`tid` 必须大于 0）；
/// - `tgkill(tgid, tid, sig)`：限定在指定线程组 `tgid` 内向线程 `tid` 投递信号，防止 TID 复用误伤其他进程。
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

/// 处理 `rt_sigaction`（134）系统调用：查询和/或替换指定信号的处理动作。
///
/// - **参数**：`raw[0]=signum`，`raw[1]=act`（新动作指针，可为 0），`raw[2]=oldact`（旧动作输出指针，可为 0），`raw[3]=sigsetsize`（必须为 8）。
/// - 若 `oldact` 写回用户空间失败，此前已在进程组信号表中生效的新动作保持不变。
fn action(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[3] != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    // Step 1: 若提供了新动作指针，先在锁外完整拷入 24 字节并解码。
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

    // Step 2: 原子更新线程组共享的信号处理表并取得旧动作，随后按需写回用户空间。
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

/// 处理 `rt_sigprocmask`（135）系统调用：查询或修改当前线程的信号阻塞掩码。
///
/// - **参数**：`raw[0]=how`（仅当 `set != NULL` 时校验），`raw[1]=set`（新掩码指针，可为 0），`raw[2]=oldset`（旧掩码输出指针，可为 0），`raw[3]=sigsetsize`。
/// - `SIGKILL` 与 `SIGSTOP` 等不可阻塞位会在 `replace_mask` 内部自动剔除。
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

/// 处理 `rt_sigpending`（136）系统调用：查询当前处于挂起（Pending）且被当前线程阻塞的信号位图。
///
/// 返回集合同时包含线程私有挂起信号与所属进程组共享挂起信号中受当前掩码阻塞的部分。
fn pending(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[1] != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    let mm = user.mm().expect("sigpending has no MM");
    let bits = user.signals().expect("sigpending has no signals").pending();
    copyout(&mm, raw[0], &bits.to_le_bytes())?;
    Ok(0)
}

/// 处理 `sigaltstack`（132）系统调用：设置或查询当前线程的备用信号栈（Alternate Signal Stack）。
///
/// - 根据当前用户态栈指针 `sp` 动态判定线程是否正运行在备用信号栈上（`SS_ONSTACK`），正在使用时禁止更换信号栈（返回 `EPERM`）。
/// - C `stack_t` 中 `ss_flags` 后的 4 字节对齐填充位在解码时跳过，写回时固定清零。
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

/// 处理 `rt_sigsuspend`（133）系统调用：临时替换当前线程的信号掩码并挂起，直至收到可触发处理函数或终止进程的信号。
///
/// - 原信号掩码通过 `user.save_mask` 暂存：若唤醒后投递了用户信号处理函数，信号栈帧中会保存原掩码以便 `sigreturn` 恢复；
/// - 若唤醒原因未触发用户处理函数（例如收到 `SIGSTOP`/`SIGCONT` 后继续运行），返回 `Restart::NoHandler` 在出口恢复原掩码并重放等待。
fn suspend(user: &UserThread, request: Request) -> Outcome {
    let result = (|| -> Result<(), Errno> {
        if request.arguments[1] != SIGSET_BYTES {
            return Err(Errno::InvalidArgument);
        }
        // Step 1: 读取临时掩码并替换当前线程掩码，将原始掩码暂存到线程恢复槽位。
        let mm = user.mm().expect("sigsuspend has no MM");
        let value = sigset(&mm, request.arguments[0])?;
        let signals = user.signals().expect("sigsuspend has no signals");
        user.save_mask(signals.replace_mask(value));

        // Step 2: 循环注册可中断等待器并休眠，直到检测到非阻塞的可中断信号到达。
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

/// 处理 `rt_sigtimedwait`（137）系统调用：在给定超时内同步等待并直接摘除指定集合中的挂起信号。
///
/// - 若目标信号已在挂起队列中，立即摘除并写回 `siginfo_t`；
/// - 否则通过 `begin_waited` 登记等待集合以接收唤醒通知，循环睡眠直至摘除成功、被其他未等待信号打断（`EINTR`）或超时到期（`EAGAIN`）。
fn timedwait(user: &UserThread, raw: [usize; 6]) -> SysResult {
    if raw[3] != SIGSET_BYTES {
        return Err(Errno::InvalidArgument);
    }

    // Step 1: 解析目标信号集（剔除不可阻塞的 SIGKILL/SIGSTOP）并一次性计算单调时钟截止时间。
    let mm = user.mm().expect("sigtimedwait has no MM");
    let set = sigset(&mm, raw[0])? & !UNBLOCKABLE;
    let deadline = if raw[2] == 0 {
        None
    } else {
        Some(timeout(&mm, raw[2])?)
    };

    // Step 2: 先尝试非阻塞摘除；若尚无匹配信号且未超时，则进入可中断等待循环。
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

    // Step 3: 若成功摘除信号，按需编码 128 字节 siginfo_t 写回用户空间并返回信号编号；否则区分中断与超时。
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

/// 处理 `rt_sigqueueinfo`（138）与 `rt_tgsigqueueinfo`（240）系统调用：携带自定义 `siginfo_t` 载荷向进程或线程排队发送信号。
///
/// 从用户空间读取完整的 128 字节 `siginfo_t`，提取 `si_errno`、`si_code` 及发送者载荷字段后交由 `TaskGraph` 校验发送权限与 `si_code` 伪造限制（非自身进程禁止注入非负或内核保留 `si_code`）。
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

/// 处理 `rt_sigreturn`（139）系统调用：在用户态信号处理函数执行完毕后恢复信号前的用户寄存器现场、信号掩码与备用栈状态。
///
/// - 若用户栈上的信号帧被篡改或无法访问导致解码失败，内核直接向当前线程组发出致命 `SIGSEGV` 请求并终止进程组，防止利用伪造上下文提权或破坏执行流。
fn sigreturn(user: &UserThread) -> Outcome {
    // Step 1: 清除残留的系统调用重启状态，并基于当前上下文副本尝试从用户栈解码信号保存帧。
    let _ = user.take_restart();
    let mm = user.mm().expect("sigreturn has no MM");
    let context = user.context().fork();
    match context.restore_signal(&mm) {
        // Step 2: 解码成功，一次性恢复信号掩码、备用栈配置及全部通用/程序计数器寄存器。
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
        // Step 3: 信号帧损坏或地址非法，触发致命 SIGSEGV 终止整个线程组。
        Err(_) => {
            let identity = user.identity().expect("sigreturn has no identity");
            identity.graph().request_exit(&identity, SIGSEGV as i32);
            Outcome::GroupExit(SIGSEGV as i32)
        }
    }
}

/// 从切片指定字节偏移处按小端序读取一个 64 位无符号整数（`u64`）。
fn word(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("ABI word width"),
    )
}

/// 从用户空间读取 8 字节的 `sigset_t` 信号位图并解码为 `u64`。
fn sigset(mm: &Mm, address: usize) -> Result<u64, Errno> {
    let mut bytes = [0u8; SIGSET_BYTES];
    copyin(mm, &mut bytes, address)?;
    Ok(u64::from_le_bytes(bytes))
}

/// 从用户空间读取 `rt_sigtimedwait` 的相对 `struct timespec` 参数，并一次性换算为单调时钟截止时间。
///
/// 若秒数为负或纳秒字段不在 `0..1_000_000_000` 范围内则返回 `EINVAL`；若与当前时间相加溢出则返回 `EOVERFLOW`。
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
