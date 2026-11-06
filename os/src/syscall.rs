//! 用户态系统调用分发器、重启状态机与基础杂项系统调用实现。
//!
//! # 模块职责与架构定位
//! 本模块位于硬件陷入入口与内核各业务子系统之间，负责解码用户态系统调用请求、路由到对应处理函数，并裁决信号中断后的系统调用重启语义：
//! - **上游调用者**：用户态任务主循环（`task::user`），在捕获 `UserTrap::Syscall` 后调用 [`Request::snapshot`] 拍下寄存器快照并交由 [`dispatch`] 执行；在信号投递前调用 [`resolve_restart`] 裁决被中断的系统调用是返回 `-EINTR` 还是透明重启。
//! - **下游子系统**：文件与路径操作（`file`）、虚拟内存映射（`memory`）、进程与等待生命周期（`task`）、信号控制（`signal`）、时间与定时器（`time`）、多路复用（`poll`）、快速用户态互斥锁（`futex`）、网络套接字（`net`）、CPU 亲和性（`affinity`）、系统管理（`system`），以及本文件内直接实现的 `getrandom` 与 `syslog`。
//!
//! # 核心设计：寄存器 ABI 布局与中断重启状态机
//!
//! ## 1. 跨架构系统调用寄存器约定与单次快照（[`Request`]）
//! 进入内核后，`Request::snapshot` 在阻塞或借用用户态上下文前一次性从保存的 `UserContext` 中拷出原始 PC、系统调用号与 6 个参数寄存器，使后续可睡眠路径无需继续借用陷阱帧：
//!
//! ```text
//! +------------------+--------------------+--------------------+
//! | 语义角色         | RISC-V 64 寄存器   | LoongArch 64 寄存器|
//! +------------------+--------------------+--------------------+
//! | 系统调用号       | a7 (x17, gpr[17])  | a7 (r11, gpr[11])  |
//! | 参数 arg0..arg5  | a0..a5 (gpr[10..16])| a0..a5 (gpr[4..10])|
//! | 返回值           | a0 (x10, gpr[10])  | a0 (r4,  gpr[4])   |
//! | 触发指令与步长   | ecall (4 字节)     | syscall 0 (4 字节) |
//! +------------------+--------------------+--------------------+
//! ```
//!
//! ## 2. 系统调用结果（[`Outcome`]）与信号重启状态机（[`Restart`]）
//! 系统调用并非总是简单写回 `a0` 并令 `PC += 4`（例如 `execve` 切换新映像、`sigreturn` 恢复信号栈帧、`exit` 终止线程、或阻塞调用被信号打断）。因此 `dispatch` 返回互斥的控制流结果 [`Outcome`]，由任务主循环在下一次返回用户态或建立信号栈帧前统一结算：
//!
//! ```text
//!   用户态执行 ecall / syscall (位于原 PC)
//!                  │
//!                  ▼
//!       Request::snapshot(&context)
//!                  │
//!                  ▼
//!          syscall::dispatch()
//!                  │
//!      ┌───────────┼───────────┬───────────────────────┐
//!      ▼           ▼           ▼                       ▼
//! Outcome::    Outcome::   Outcome::               Outcome::Restart(state)
//! Return(val)  Restored /  Exit / GroupExit            │ (调用在阻塞等待中被信号唤醒)
//!      │       NewImage        │                       ▼
//!      ▼           │           ▼             返回用户态前检查待处理信号 (signals.select)
//! a0 = val         ▼       退出当前线程        ┌───────┴────────────────────────┐
//! PC = 原 PC + 4 保持新现场 或请求全组退出      │ 选中用户态信号处理函数          │ 无用户态处理函数 (如信号被忽略或被他线程取走)
//!                                              ▼                                ▼
//!                                 resolve_restart(Some(sa_flags))   resolve_restart(None)
//!                                      │                │                       │
//!                         含 SA_RESTART│                │无 SA_RESTART          │ 总是自动重启
//!                         (仅 Syscall) ▼                ▼ (或 NoHandler/Sleep/Futex)
//!                                  回退至原 PC       写回 -EINTR            回退至原 PC 恢复原参
//!                                  恢复原参数       (Sleep 先写回剩余时间)  (Sleep/Futex 改系统调用号为
//!                                  (再压入信号帧)    PC = 原 PC + 4          128 restart_syscall 并保存绝对期限)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - **无跨阻塞借用陷阱帧**：`user.context()` 的可变借用仅在 `return_to`、`restart_to` 等纯内存写回瞬间短暂持有；系统调用分发与睡眠等待期间不持有 `UserContext` 锁或借用，允许信号投递与上下文检查安全访问寄存器现场。
//! - **子系统锁顺序**：各系统调用进入下游模块时，必须先释放当前临时自旋锁再执行用户空间拷贝（`copyin`/`copyout` 可能触发缺页睡眠）或进入等待队列；具体锁层级遵循目标子系统（`fs`、`mm`、`task`、`net`）各自的锁顺序规范。

mod affinity;
mod error;
mod file;
mod futex;
mod memory;
mod net;
mod poll;
pub(crate) mod signal;
mod system;
mod task;
pub(crate) mod time;

use alloc::vec::Vec;

use crate::arch::UserContext;
use crate::fs::ReadBuffer;
use crate::klog;
use crate::random;
use crate::sched;
use crate::syscall::error::{copyout, SysResult};
use crate::task::process::{EXIT_STATUS_MASK, EXIT_STATUS_SHIFT};
use crate::task::UserThread;
pub(crate) use error::Errno;

/// RISC-V 64 Linux ABI 中第 0 个系统调用参数兼返回值寄存器 `a0`（`x10`）在通用寄存器数组中的下标。
#[cfg(target_arch = "riscv64")]
const ARGUMENT_REGISTER: usize = 10;
/// LoongArch 64 Linux ABI 中第 0 个系统调用参数兼返回值寄存器 `a0`（`r4`）在通用寄存器数组中的下标。
#[cfg(target_arch = "loongarch64")]
const ARGUMENT_REGISTER: usize = 4;

/// RISC-V 64 Linux ABI 中存放系统调用号的寄存器 `a7`（`x17`）在通用寄存器数组中的下标。
#[cfg(target_arch = "riscv64")]
const NUMBER_REGISTER: usize = 17;
/// LoongArch 64 Linux ABI 中存放系统调用号的寄存器 `a7`（`r11`）在通用寄存器数组中的下标。
#[cfg(target_arch = "loongarch64")]
const NUMBER_REGISTER: usize = 11;

/// 单条系统调用陷入指令（RISC-V `ecall` 或 LoongArch `syscall 0`）的固定机器码长度（单位：字节）。
const SYSCALL_INSTRUCTION_BYTES: usize = 4;
/// Linux 通用 ABI 中的 `rt_sigreturn` 重启辅助系统调用号 `restart_syscall`（编号 128），用于按已保存的绝对超时时间恢复睡眠或 futex 等待。
const RESTART_SYSCALL: usize = 128;

/// 单次系统调用入口处的用户态寄存器只读快照。
///
/// 在进入可能阻塞或修改用户现场的系统调用处理逻辑前构造，保留原始指令地址、调用号与全部 6 个参数，
/// 用于普通返回时推进 `PC` 或在信号中断重启时精确还原原始入参（避免 `a0` 被覆盖后丢失第 0 个参数）。
#[derive(Clone, Copy, Debug)]
pub(crate) struct Request {
    /// 触发本次系统调用的指令虚拟地址（单位：字节地址）。
    pub(crate) pc: usize,
    /// 用户态通过 `a7` 寄存器传入的系统调用编号。
    pub(crate) number: usize,
    /// 用户态通过 `a0`–`a5` 寄存器依次传入的 6 个原始机器字参数。
    pub(crate) arguments: [usize; 6],
}

impl Request {
    /// 从当前线程保存的用户态陷阱帧 `context` 中一次性提取 `PC`、系统调用号与 6 个原始参数。
    ///
    /// 在任务陷入内核后的普通线程上下文中调用；快照完成后即可释放对 `context` 的借用并进入可睡眠的分发路径。
    pub(crate) fn snapshot(context: &UserContext) -> Self {
        Self {
            pc: context.pc,
            number: context.gpr[NUMBER_REGISTER],
            arguments: core::array::from_fn(|index| context.gpr[ARGUMENT_REGISTER + index]),
        }
    }

    /// 将系统调用普通返回值 `result` 写入返回值寄存器 `a0`，并将用户态 `PC` 推进 4 字节以越过系统调用指令。
    ///
    /// 仅在系统调用正常完成（[`Outcome::Return`]）或确认以 `-EINTR` 结束时调用；`execve`、`sigreturn`、线程退出及信号重启路径不得调用此函数。
    pub(crate) fn return_to(self, context: &mut UserContext, result: isize) {
        context.gpr[ARGUMENT_REGISTER] = result as usize;
        context.pc = self
            .pc
            .checked_add(SYSCALL_INSTRUCTION_BYTES)
            .expect("syscall PC overflow");
    }

    /// 将用户态陷阱帧回退到系统调用触发前的状态，以供返回用户态后重新执行系统调用指令。
    ///
    /// 会将 `a0`–`a5` 恢复为快照中的原始参数（防止此前的操作或信号帧准备覆盖 `a0`），并将 `PC` 重置为原始系统调用指令地址。
    /// 若 `restart_block` 为 `true`，则将系统调用号寄存器 `a7` 改写为 [`RESTART_SYSCALL`]（128），使重入内核时按已保存的绝对截止时间继续等待而非重新计算相对超时。
    pub(crate) fn restart_to(self, context: &mut UserContext, restart_block: bool) {
        context.gpr[ARGUMENT_REGISTER..ARGUMENT_REGISTER + self.arguments.len()]
            .copy_from_slice(&self.arguments);
        context.gpr[NUMBER_REGISTER] = if restart_block {
            RESTART_SYSCALL
        } else {
            self.number
        };
        context.pc = self.pc;
    }
}

/// 系统调用因可中断睡眠被唤醒后挂起的重启上下文。
///
/// 当阻塞型系统调用检测到待处理信号中断时，并不立即向 `UserContext` 写入 `-EINTR`，而是将该枚举返回给任务主循环，
/// 待信号选择阶段确定是否实际执行用户态信号处理函数及 `SA_RESTART` 标志后，再由 [`resolve_restart`] 决定最终行为。
#[derive(Clone, Copy)]
pub(crate) enum Restart {
    /// 支持 `SA_RESTART` 自动重启的常规系统调用（如尚未传输任何字节的 `read`/`write`、`ioctl`、`wait4`、无超时的 `futex` 等）。
    /// 若命中的信号处理函数带有 `SA_RESTART` 标志，或最终未执行任何用户态处理函数，则直接按原编号与原参数重放。
    Syscall {
        /// 被中断系统调用的原始寄存器快照。
        request: Request,
    },
    /// 始终不支持 `SA_RESTART` 的系统调用（如 `ppoll`、`pselect6`、`epoll_pwait`、`rt_sigtimedwait`）。
    /// 一旦实际调用了任何用户态信号处理函数（无论是否设置 `SA_RESTART`）都必须返回 `-EINTR`；仅当未触发用户态处理函数（虚假中断或信号被忽略）时才恢复掩码并透明重试。
    NoHandler {
        /// 被中断系统调用的原始寄存器快照。
        request: Request,
    },
    /// 带超时截止时间的睡眠系统调用（`nanosleep` / `clock_nanosleep`）。
    /// 若执行了用户态信号处理函数，需将剩余未睡满的时间写回用户缓冲区（若存在）并返回 `-EINTR`；若未执行处理函数，则通过 `restart_syscall` 按原绝对截止时间继续睡眠。
    Sleep {
        /// 被中断系统调用的原始寄存器快照。
        request: Request,
        /// 首次进入睡眠时换算好的单调时钟绝对截止时间点。
        deadline: crate::time::MonotonicInstant,
        /// 需要写回剩余睡眠时长的用户态 `timespec` 结构体虚拟地址（单位：字节地址；如 `CLOCK_ABSTIME` 绝对睡眠或用户传入空指针则为 `None`）。
        remaining: Option<usize>,
    },
    /// 带相对或绝对超时的 `futex` 等待调用。
    /// 若执行了用户态信号处理函数则返回 `-EINTR`；若未执行处理函数，则通过 `restart_syscall` 按原绝对截止时间继续等待，防止多次中断导致相对超时被无限拉长。
    Futex {
        /// 被中断系统调用的原始寄存器快照。
        request: Request,
        /// 首次进入等待时换算好的单调时钟绝对截止时间点。
        deadline: crate::time::MonotonicInstant,
    },
}

/// 系统调用分发完成后的顶层控制流结果，指导用户态主循环如何处置当前线程上下文。
pub(crate) enum Outcome {
    /// 普通返回：将带符号返回值（非负成功结果或负 Linux `errno`）写入 `a0` 并将用户态 `PC` 前进 4 字节。
    Return(isize),
    /// 仅终止当前线程（对应 `exit` 系统调用），载荷为按 `wait` 编码左移后的退出状态字。
    Exit(i32),
    /// 请求终止整个线程组（对应 `exit_group`、致命信号或关机退出），载荷为按 `wait` 编码左移后的退出状态字。
    GroupExit(i32),
    /// 系统调用在可中断等待中被唤醒，携带 [`Restart`] 状态交由返回用户态前的信号投递逻辑裁决重启或 `-EINTR`。
    Restart(Restart),
    /// 用户态寄存器现场已由 `rt_sigreturn` 完整恢复，不得再修改 `a0` 或递增 `PC`。
    Restored,
    /// 进程地址空间与入口点已由 `execve` 替换为新程序映像，清空挂起的重启状态并直接跳转新入口运行。
    NewImage,
}

/// 在信号投递阶段根据选中的信号处理函数标志 `handler_flags` 裁决被中断系统调用的重启或 `-EINTR` 返回。
///
/// 在普通可睡眠的用户线程返回路径上调用（压入信号栈帧之前）：
/// - `handler_flags` 为 `Some(flags)` 表示即将调用用户态信号处理函数，`flags` 为该 `sigaction` 的标志位；
/// - `handler_flags` 为 `None` 表示本次中断未触发任何用户态信号处理函数（例如被忽略的信号或已被其他线程消费）。
///
/// 若决定不重启且状态为 [`Restart::Sleep`]，会先在原用户现场下将剩余睡眠时间拷贝回用户空间，随后再由调用方基于更新后的寄存器现场构建信号栈帧。
pub(crate) fn resolve_restart(user: &UserThread, state: Restart, handler_flags: Option<usize>) {
    // Step 1: 根据重启类别与是否实际进入用户态信号处理函数，判定本次调用是否应当自动重启。
    let (request, restart) = match state {
        Restart::Syscall { request } => (
            request,
            handler_flags.is_none_or(|flags| flags & crate::task::signal::SA_RESTART != 0),
        ),
        Restart::NoHandler { request }
        | Restart::Sleep { request, .. }
        | Restart::Futex { request, .. } => (request, handler_flags.is_none()),
    };

    // Step 2: 若满足重启条件，恢复必要的临时信号掩码或挂起 restart_block 状态，并将寄存器回退到系统调用指令处。
    if restart {
        if matches!(state, Restart::NoHandler { .. }) {
            user.restore_saved_mask();
        }
        let block = matches!(state, Restart::Sleep { .. } | Restart::Futex { .. });
        if block {
            user.save_restart(state);
        }
        request.restart_to(&mut user.context(), block);
    } else {
        // Step 3: 若不重启（即将进入无 SA_RESTART 的信号处理函数），向用户空间写回必要的剩余睡眠时间，并将返回值设为 -EINTR（或写回时的 -EFAULT）且推进 PC。
        let result = match state {
            Restart::Sleep {
                deadline,
                remaining,
                ..
            } => time::interrupt_sleep(user, deadline, remaining).unwrap_or_else(Errno::result),
            _ => Errno::Interrupted.result(),
        };
        request.return_to(&mut user.context(), result);
    }
}

/// 根据 Linux 通用架构系统调用编号（`asm-generic/unistd.h`）将请求分发至各子系统处理函数。
///
/// 在普通可睡眠的用户线程上下文中调用。内部按功能域分阶段路由：
/// 1. 自带特殊 [`Outcome`] 控制流的模块（`poll`、`futex`、`time`、`restart_syscall`、`signal`、`net`）直接返回其结果；
/// 2. 可能在零字节进度下被信号打断的流式 I/O 与 `ioctl` 调用，将内部的 [`Errno::RestartSyscall`] 转换为 [`Outcome::Restart`]；
/// 3. 其余常规系统调用统一将 `SysResult` 编码为正返回值或负 Linux `errno`，未实现的编号返回 `-ENOSYS`。
pub(crate) fn dispatch(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    // Step 1: 路由自带重启状态机或特殊 Outcome 的独立子模块。
    if matches!(request.number, 20..=22 | 72 | 73) {
        return poll::dispatch(user, request);
    }
    if request.number == 98 {
        return futex::dispatch(user, request);
    }
    if matches!(request.number, 101..=103 | 113..=115 | 153 | 165 | 169) {
        return time::dispatch(user, request);
    }
    // Step 2: 处理编号 128 的 restart_syscall，恢复此前被非处理函数信号打断的带期限睡眠或 futex 等待。
    if request.number == RESTART_SYSCALL {
        return match user.take_restart() {
            Some(Restart::Sleep {
                request,
                deadline,
                remaining,
            }) => time::resume_sleep(user, request, deadline, remaining),
            Some(Restart::Futex { request, deadline }) => {
                futex::resume_wait(user, request, deadline)
            }
            Some(Restart::Syscall { .. } | Restart::NoHandler { .. }) | None => {
                Outcome::Return(Errno::Interrupted.result())
            }
        };
    }
    if matches!(request.number, 129..=139 | 240) {
        return signal::dispatch(user, request);
    }
    if matches!(request.number, 198..=212 | 242) {
        return net::dispatch(user, request);
    }
    // Step 3: 路由可被信号打断且支持 SA_RESTART 的文件/管道/终端读写与设备控制调用。
    if matches!(request.number, 29 | 63..=66 | 76 | 285) {
        let result = match request.number {
            29 => file::ioctl(user, raw),
            63 => file::read(user, raw),
            64 => file::write(user, raw),
            65 => file::readv(user, raw),
            66 => file::writev(user, raw),
            76 => file::splice(user, raw),
            285 => file::copy_file_range(user, raw),
            _ => unreachable!(),
        };
        return match result {
            Err(Errno::RestartSyscall) => Outcome::Restart(Restart::Syscall { request }),
            other => Outcome::Return(other.unwrap_or_else(Errno::result)),
        };
    }

    // Step 4: 分发其余常规文件、内存、任务、亲和性及系统管理调用，统一将错误转换为负 errno。
    let result: SysResult = match request.number {
        93 => {
            return Outcome::Exit(
                ((raw[0] & EXIT_STATUS_MASK as usize) << EXIT_STATUS_SHIFT) as i32,
            )
        }
        94 => {
            return Outcome::GroupExit(
                ((raw[0] & EXIT_STATUS_MASK as usize) << EXIT_STATUS_SHIFT) as i32,
            )
        }
        17 => file::getcwd(user, raw),
        23 => file::dup(user, raw[0]),
        24 => file::dup3(user, raw),
        25 => file::fcntl(user, raw),
        34 => file::mkdirat(user, raw),
        35 => file::unlinkat(user, raw),
        36 => file::symlinkat(user, raw),
        37 => file::linkat(user, raw),
        39 => system::umount(user, &raw),
        40 => system::mount(user, &raw),
        43 => system::statfs(user, &raw),
        44 => system::fstatfs(user, &raw),
        45 => file::truncate(user, raw),
        46 => file::ftruncate(user, raw),
        48 => file::faccessat(user, raw, 0),
        49 => file::chdir(user, raw[0]),
        50 => file::fchdir(user, raw),
        51 => file::chroot(user, raw[0]),
        52 => file::fchmod(user, raw),
        53 => file::fchmodat(user, raw),
        56 => file::openat(user, raw),
        57 => file::close(user, raw[0]),
        59 => file::pipe2(user, raw),
        61 => file::getdents(user, raw),
        62 => file::seek(user, raw),
        67 => file::pread(user, raw),
        68 => file::pwrite(user, raw),
        78 => file::readlink(user, raw),
        79 => file::statat(user, raw),
        80 => file::fstat(user, raw),
        81 => system::sync(user),
        82 => file::fsync(user, raw[0]),
        83 => file::fdatasync(user, raw[0]),
        88 => file::utimensat(user, raw),
        116 => syslog(user, raw),
        166 => file::umask(user, raw[0]),
        95 => return task::waitid(user, request),
        96 => Ok(user.set_tid_address(raw[0]) as isize),
        154..=157 | 173..=177 => task::identity(user, request.number, raw),
        160 => system::uname(user, &raw),
        163 => task::getrlimit(user, raw),
        164 => task::setrlimit(user, raw),
        261 => task::prlimit64(user, raw),
        142 => return system::reboot(user, &raw),
        122 => affinity::set(user, raw),
        123 => affinity::get(user, raw),
        168 => affinity::getcpu(user, raw),
        124 => {
            sched::yield_now();
            Ok(0)
        }
        172 | 178 => {
            let identity = user.identity().expect("syscall has no user identity");
            Ok(if request.number == 172 {
                identity.tgid()
            } else {
                identity.tid()
            } as isize)
        }
        179 => system::sysinfo(user, &raw),
        214 => Ok(memory::brk(user, raw[0])),
        215 => memory::munmap(user, raw),
        220 => task::clone(user, request),
        221 => return task::execve(user, request),
        222 => memory::mmap(user, raw),
        226 => memory::mprotect(user, raw),
        227 => memory::msync(user, raw),
        260 => return task::wait4(user, request),
        276 => file::renameat2(user, raw),
        278 => getrandom(user, raw),
        291 => file::statx(user, raw),
        439 => file::faccessat(user, raw, raw[3] as u32),
        _ => Err(Errno::NoSyscall),
    };
    Outcome::Return(result.unwrap_or_else(Errno::result))
}

/// 实现 `getrandom(buf, buflen, flags)` 系统调用（编号 278），向用户缓冲区填充伪随机字节。
///
/// 在可睡眠的用户线程上下文中调用：
/// - 参数 `raw[0]`：用户态输出缓冲区首地址 `buf`；
/// - 参数 `raw[1]`：请求字节数 `buflen`（单次上限截断至 `MAX_RW_COUNT`）；
/// - 参数 `raw[2]`：标志位 `flags`（支持 `GRND_NONBLOCK`、`GRND_RANDOM`、`GRND_INSECURE`）。
///
/// 当前内核仅提供非密码学 PRNG（[`random::fill_insecure`]），尚未接入硬件密码学真随机熵源：
/// - 仅当调用方显式指定 `GRND_INSECURE` 时才填充伪随机字节；
/// - 若未指定 `GRND_INSECURE`（请求密码学安全随机字节），非阻塞模式返回 `-EAGAIN`，阻塞模式返回 `-ENOSYS` 以避免用非安全数据冒充密码学熵源。
/// - 若向用户空间分块拷贝途中发生缺页失败（`-EFAULT`），只要此前已成功写入至少 1 个字节就返回已成功写入的字节数（Partial Progress），仅在零字节写入时返回 `-EFAULT`。
fn getrandom(user: &UserThread, raw: [usize; 6]) -> SysResult {
    const GRND_NONBLOCK: u32 = 1;
    const GRND_RANDOM: u32 = 2;
    const GRND_INSECURE: u32 = 4;

    // Step 1: 校验 flags 合法性（禁止未知位，且 GRND_RANDOM 与 GRND_INSECURE 互斥）。
    let flags = raw[2] as u32;
    if flags & !(GRND_NONBLOCK | GRND_RANDOM | GRND_INSECURE) != 0
        || flags & (GRND_RANDOM | GRND_INSECURE) == (GRND_RANDOM | GRND_INSECURE)
    {
        return Err(Errno::InvalidArgument);
    }
    // 当前内核没有密码学安全熵池，拒绝为安全随机数请求填充非密码学 PRNG 数据。
    if flags & GRND_INSECURE == 0 {
        return Err(if flags & GRND_NONBLOCK != 0 {
            Errno::Again
        } else {
            Errno::NoSyscall
        });
    }

    // Step 2: 按 256 字节栈上分块生成伪随机字节并写入用户空间，处理中途页故障的部分成功返回。
    let mm = user.mm().expect("getrandom has no MM");
    let mut target = ReadBuffer::user(&mm, raw[0], raw[1].min(file::MAX_RW_COUNT))?;
    let mut bytes = [0; 256];
    while target.copied() < target.len() {
        let count = bytes.len().min(target.len() - target.copied());
        random::fill_insecure(&mut bytes[..count]);
        if let Err(error) = target.push(&bytes[..count]) {
            return if target.copied() == 0 {
                Err(error.into())
            } else {
                Ok(target.copied() as isize)
            };
        }
    }

    Ok(target.copied() as isize)
}

/// 实现 `syslog(type, bufp, len)` 系统调用（编号 116，Linux 中亦称 `klogctl`），查询内核日志环形缓冲区大小或无损读取历史日志。
///
/// 在可睡眠的用户线程上下文中调用：
/// - 参数 `raw[0]`：操作命令 `type`（当前支持 `SYSLOG_ACTION_READ_ALL = 3` 读取尾部完整记录、`SYSLOG_ACTION_SIZE_BUFFER = 10` 查询缓冲区总字节容量）；
/// - 参数 `raw[1]`：用户态接收缓冲区指针 `bufp`；
/// - 参数 `raw[2]`：用户态接收缓冲区字节容量 `len`。
///
/// 读取操作使用独立的只读 [`klog::Cursor`] 快照遍历环形缓冲区，格式化为标准的 `<level>message\n` 文本行；
/// 若格式化后的总长超过用户提供的 `len`，则从最旧的整条记录开始丢弃，确保返回的是能够装入 `len` 的最新完整日志后缀，且不会清空日志环或推进后台串口打印器及 `/dev/kmsg` 打开文件的游标。
fn syslog(user: &UserThread, raw: [usize; 6]) -> SysResult {
    const SYSLOG_ACTION_READ_ALL: i32 = 3;
    const SYSLOG_ACTION_SIZE_BUFFER: i32 = 10;

    // Step 1: 解析操作码并校验用户缓冲区指针与容量参数。
    let action = raw[0] as i32;
    if action == SYSLOG_ACTION_SIZE_BUFFER {
        return Ok((klog::CAPACITY * klog::TEXT_CAPACITY) as isize);
    }
    if action != SYSLOG_ACTION_READ_ALL {
        return Err(if (0..SYSLOG_ACTION_SIZE_BUFFER).contains(&action) {
            Errno::NoSyscall
        } else {
            Errno::InvalidArgument
        });
    }
    let capacity = raw[2] as i32;
    if raw[1] == 0 || capacity < 0 {
        return Err(Errno::InvalidArgument);
    }
    if capacity == 0 {
        return Ok(0);
    }

    // Step 2: 记录当前序号上界 end，从最老记录开始将环形缓冲区快照格式化为 `<level>text\n` 并记录每条字节长度。
    let end = klog::next_sequence();
    let mut cursor = klog::Cursor::oldest();
    let mut text = Vec::new();
    let mut lengths = [0; klog::CAPACITY];
    let mut count = 0;
    while count < lengths.len() {
        let Some(read) = cursor.read() else {
            break;
        };
        let record = read.record;
        if end <= record.seq {
            break;
        }
        if text.capacity() == 0 {
            text.try_reserve_exact(klog::CAPACITY * (klog::TEXT_CAPACITY + 4))
                .map_err(|_| Errno::NoMemory)?;
        }
        let begin = text.len();
        let level = match record.level {
            klog::Level::Error => b'3',
            klog::Level::Warn => b'4',
            klog::Level::Info => b'6',
            klog::Level::Debug => b'7',
        };
        text.extend_from_slice(&[b'<', level, b'>']);
        text.extend_from_slice(record.text().as_bytes());
        if !record.text().ends_with('\n') {
            text.push(b'\n');
        }
        lengths[count] = text.len() - begin;
        count += 1;
    }

    // Step 3: 若总文本超出用户缓冲区容量，按整条记录边界裁掉最旧的前缀，再一次性拷贝到用户空间。
    let mut start = 0;
    for length in &lengths[..count] {
        if text.len() - start <= capacity as usize {
            break;
        }
        start += length;
    }

    let output = &text[start..];
    let mm = user.mm().expect("syslog has no MM");
    copyout(&mm, raw[1], output)?;
    Ok(output.len() as isize)
}
