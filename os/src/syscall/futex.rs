//! 快速用户态互斥锁（`futex`）系统调用 ABI 解码与等待重启适配模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于系统调用分发器（`crate::syscall`）与内核 futex 等待队列实现（`crate::task::futex`）之间：
//! - 上游由 `syscall::dispatch` 在处理系统调用号 `98`（`futex`）与 `128`（`restart_syscall`）时调用；
//! - 下游委托 `crate::task::futex` 执行原子值比较、睡眠排队、按位掩码唤醒及队列转移（requeue），
//!   并借助 `crate::time` 将用户态 `timespec` 统一换算为内核单调时钟截止时间（`MonotonicInstant`）。
//!
//! ## 核心设计与参数复用布局
//! Linux `futex(uaddr, futex_op, val, timeout/val2, uaddr2, val3)` 系统调用在不同子命令下复用第 4~6 个参数，
//! 且 `futex_op` 低位编码主命令，高位编码修饰标志位：
//!
//! ```text
//! futex_op (u32) 位域结构：
//! +-----------------------+------------------+---------------+------------------+
//! | 31 .. 9 (其他标志位)  | 8: CLOCK_REALTIME| 7: PRIVATE    | 6 .. 0: COMMAND  |
//! +-----------------------+------------------+---------------+------------------+
//!
//! 六个寄存器参数在不同子命令下的含义：
//! - WAIT / WAIT_BITSET : raw[0]=uaddr, raw[1]=op, raw[2]=val,  raw[3]=timeout*, raw[4]=忽略,   raw[5]=val3(bitset)
//! - WAKE / WAKE_BITSET : raw[0]=uaddr, raw[1]=op, raw[2]=val,  raw[3]=忽略,     raw[4]=忽略,   raw[5]=val3(bitset)
//! - REQUEUE            : raw[0]=uaddr, raw[1]=op, raw[2]=val,  raw[3]=val2,     raw[4]=uaddr2, raw[5]=忽略
//! - CMP_REQUEUE        : raw[0]=uaddr, raw[1]=op, raw[2]=val,  raw[3]=val2,     raw[4]=uaddr2, raw[5]=val3(expected)
//! ```
//!
//! ### 信号中断与 `restart_syscall` 状态流转
//! `FUTEX_WAIT` 传入的是**相对超时**，若被携带 `SA_RESTART`（或无用户处理函数）的信号打断后直接重放原指令，
//! 会错误地重新读取相对时长导致超时顺延。因此：
//! 1. 首次进入 `dispatch` 时，`parse_deadline` 将相对时长一次性换算为单调时钟绝对截止点 `deadline`；
//! 2. 若睡眠被信号打断，`wait` 返回 `Outcome::Restart(Restart::Futex { request, deadline })`；
//! 3. 内核信号返回路径改写用户寄存器，使线程重返内核时执行 `restart_syscall`，由 `resume_wait` 直接复用首次算出的 `deadline`，不再访问原用户态 `timeout` 指针。
//!
//! ## 锁获取顺序与并发契约
//! - 本模块在进入 `crate::task::futex` 前先通过 `copyin` 完成用户态 `timespec` 读取，此过程不持有任何 futex 桶自旋锁。
//! - 实际的哈希桶锁与地址空间页表访问顺序由下游 `crate::task::futex` 统一管理。

use crate::syscall::error::{self, Errno};
use crate::syscall::{time, Outcome, Request, Restart};
use crate::task::futex::{self, WaitOutcome};
use crate::task::UserThread;
use crate::time::{self as clock, MonotonicInstant, NSEC_PER_SEC};

/// `FUTEX_WAIT`：若 `*uaddr == val` 则休眠等待（`timeout` 解释为相对时长）。
const WAIT: u32 = 0;
/// `FUTEX_WAKE`：唤醒最多 `val` 个在 `uaddr` 上等待的线程。
const WAKE: u32 = 1;
/// `FUTEX_REQUEUE`：唤醒最多 `val` 个等待者，并将剩余最多 `val2` 个等待者转移到 `uaddr2`。
const REQUEUE: u32 = 3;
/// `FUTEX_CMP_REQUEUE`：先校验 `*uaddr == val3`，相等时执行与 `FUTEX_REQUEUE` 相同的唤醒与转移。
const CMP_REQUEUE: u32 = 4;
/// `FUTEX_WAIT_BITSET`：带 32 位位掩码的等待操作（`timeout` 解释为绝对截止时间）。
const WAIT_BITSET: u32 = 9;
/// `FUTEX_WAKE_BITSET`：唤醒位掩码与 `val3` 有交集的等待线程。
const WAKE_BITSET: u32 = 10;

/// `FUTEX_PRIVATE_FLAG`：指示 futex 仅在当前进程地址空间内共享，无需跨进程共享内存键查找。
const PRIVATE: u32 = 0x80;
/// `FUTEX_CLOCK_REALTIME`：指示绝对超时时间基于 `CLOCK_REALTIME` 而非 `CLOCK_MONOTONIC`。
const CLOCK_REALTIME: u32 = 0x100;
/// 用于从 `futex_op` 中剥离 `PRIVATE` 与 `CLOCK_REALTIME` 修饰位、提取基础命令码的掩码。
const COMMAND_MASK: u32 = !(PRIVATE | CLOCK_REALTIME);

/// 分发并执行 `futex` 系统调用的各个非优先级继承（non-PI）子命令。
///
/// - **调用上下文**：普通可睡眠系统调用上下文。
/// - **参数**：
///   - `user`：发起调用的当前用户线程。
///   - `request`：捕获的系统调用请求快照（含原始 PC 与 6 个寄存器参数）。
/// - **返回值**：正常完成或出错返回 `Outcome::Return`，等待期间被信号中断返回 `Outcome::Restart`。
pub(super) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    let operation = raw[1] as u32;
    let command = operation & COMMAND_MASK;
    let private = operation & PRIVATE != 0;
    // Step 1: 处理阻塞等待类命令（WAIT / WAIT_BITSET），解析超时参数并进入可中断睡眠。
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
    // 非等待类命令以及传统相对超时的 WAIT 均不支持 CLOCK_REALTIME 标志位。
    if operation & CLOCK_REALTIME != 0 {
        return Outcome::Return(Errno::NoSyscall.result());
    }

    // Step 2: 处理非阻塞的唤醒（WAKE / WAKE_BITSET）与重排（REQUEUE / CMP_REQUEUE）命令。
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

/// 在 `restart_syscall` 系统调用中恢复此前被信号中断的带超时 `futex` 等待。
///
/// 直接复用首次调用时已换算好的单调时钟截止时间 `deadline`，避免重新读取用户态 `timespec` 指针或重复累加相对超时。
///
/// - **调用上下文**：由 `syscall::dispatch` 在匹配到 `RESTART_SYSCALL` 且保存状态为 `Restart::Futex` 时调用。
pub(super) fn resume_wait(
    user: &UserThread,
    request: Request,
    deadline: MonotonicInstant,
) -> Outcome {
    wait(user, request, Some(deadline))
}

/// 执行底层 `futex::wait` 并将等待结果映射为系统调用出口控制流（`Outcome`）。
///
/// - 若已被并发 `FUTEX_WAKE` 选中唤醒，返回 `0`；
/// - 若到达截止时间，返回 `-ETIMEDOUT`；
/// - 若被信号中断，根据是否带有截止时间封装 `Restart::Futex`（走 `restart_syscall`）或 `Restart::Syscall`（直接重放原系统调用）。
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

/// 将 `REQUEUE` / `CMP_REQUEUE` 中的 32 位有符号配额参数（`val` 或 `val2`）校验并转换为非负计数。
///
/// 负数配额违反 Linux ABI 约定，直接返回 `EINVAL`。
fn quota(raw: usize) -> Result<usize, Errno> {
    let count = raw as u32 as i32;
    if count < 0 {
        Err(Errno::InvalidArgument)
    } else {
        Ok(count as usize)
    }
}

/// 从用户空间读取 `struct timespec` 并换算为内核单调时钟的绝对截止时间。
///
/// - `address == 0` 表示无限期等待，返回 `Ok(None)`；
/// - `FUTEX_WAIT` 将 `timespec` 视为相对等待时长，与当前单调时间 `clock::now()` 相加；
/// - `FUTEX_WAIT_BITSET` 将 `timespec` 视为绝对时间点，根据是否设置 `CLOCK_REALTIME` 分别从实时钟或单调钟投影为 `MonotonicInstant`。
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
