//! 进程与线程生命周期、程序加载（`execve`）、资源限额及子进程等待系统调用模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于系统调用分发入口（`crate::syscall::dispatch`）与进程图/克隆/ELF 加载子系统
//! （`crate::task::process`、`crate::task::clone`、`crate::task::exec`）之间，负责以下核心系统调用：
//! - **创建与程序替换**：`clone`（220）、`execve`（221）；
//! - **子进程状态等待与回收**：`waitid`（95）、`wait4`（260）；
//! - **进程身份与会话管理**：`setpgid`（154）、`getpgid`（155）、`getsid`（156）、`setsid`（157）、
//!   `getppid`（173）、`getuid`（174）、`geteuid`（175）、`getgid`（176）、`getegid`（177）；
//! - **资源软硬限额**：`getrlimit`（163）、`setrlimit`（164）、`prlimit64`（261）。
//!
//! ## 核心机制与布局说明
//!
//! ### 1. `execve` 的不可回返点（Point of No Return）状态机
//! ```text
//! sys_execve(pathname, argv, envp)
//!   │
//!   ├─ Phase 1: prepare_exec (可安全回退阶段)
//!   │    读取路径与参数、解析 ELF / `#!` 脚本并构建新地址空间映像 LoadedImage
//!   │    └── 若失败 -> 返回 Outcome::Return(-errno)，原进程地址空间与兄弟线程完好无损
//!   │
//!   ├─ === 跨过不可回返点：identity.graph().de_thread() 终止同组其他线程 ===
//!   │
//!   └─ Phase 2: install_image & complete_exec (不可回退阶段)
//!        关闭 O_CLOEXEC 文件、替换当前线程地址空间与初始栈、重置信号处理器
//!        ├── 成功 -> 返回 Outcome::NewImage，直接跳转新程序入口点运行
//!        └── 失败 -> abort_exec()，发出致命退出请求（默认 SIGSEGV）并返回 Outcome::GroupExit
//! ```
//!
//! ### 2. `rusage` 与 `rlimit64` 用户态二进制布局（LP64 小端序）
//! - `struct rlimit64`（16 字节）：`0..8` 为软限制 `rlim_cur`（`u64`），`8..16` 为硬限制 `rlim_max`（`u64`）。
//! - `struct rusage`（144 字节）：
//!   ```text
//!   +-----------------------+-----------------------+-------------------------------+
//!   | 0..16: ru_utime       | 16..32: ru_stime      | 32..144: 其他统计字段 (清零)  |
//!   |  0..8 : tv_sec  (u64) | 16..24: tv_sec  (u64) |                               |
//!   |  8..16: tv_usec (u64) | 24..32: tv_usec (u64) |                               |
//!   +-----------------------+-----------------------+-------------------------------+
//!   ```
//!
//! ## 锁获取顺序与并发契约
//! - `prepare_exec` 中仅短暂获取 `runtime.fs.lock()` 克隆 `Vfs` 与路径上下文句柄，随即释放锁再执行耗时的文件读取与 ELF 装载。
//! - `resource_limit`、`waitid`、`wait4` 的所有用户态内存读写（`copyin` / `copyout`）均在 `TaskGraph` 锁之外执行；
//!   若子进程事件已被消费或新限额已写入内核，后续 `copyout` 遇到 `EFAULT` 时不会回滚已提交的内核状态。
//! - `wait_event` 遵循“先尝试非阻塞消费 -> 注册栈上 `WaitNode` -> 二次复查 -> 可中断挂起”的标准防唤醒丢失协议。

use core::convert::TryInto;

use crate::fs::PATH_MAX;
use crate::mm::space::VmError;
use crate::syscall::error::{copyin, copyout, Errno, SysResult};
use crate::syscall::{Outcome, Request, Restart};
use crate::task::clone::{self, CloneArguments, CloneError};
use crate::task::exec::{self, Arguments, ExecError, LoadedImage};
use crate::task::process::{
    ChildEvent, Resource, ResourceLimit, WaitSelector, EXIT_STATUS_SHIFT, WAIT_CONTINUED,
    WAIT_CORE_DUMP, WAIT_STOPPED, WALL, WCLONE, WCONTINUED as CONTINUED, WEXITED as EXITED,
    WNOHANG as NOHANG, WNOWAIT as NOWAIT, WSTOPPED as STOPPED,
};
use crate::task::signal::{
    SignalInfo, SignalPayload, CLD_CONTINUED, CLD_DUMPED, CLD_EXITED, CLD_KILLED, CLD_STOPPED,
    SIGCHLD, SIGINFO_BYTES, SIGSEGV,
};
use crate::task::{UserRuntime, UserThread};
use crate::time::{NSEC_PER_SEC, NSEC_PER_USEC};
use crate::wait::{WaitNode, WaitResult, Waiter};

/// `RLIMIT_STACK`：主线程用户栈最大字节数资源编号。
const RLIMIT_STACK: u32 = 3;
/// `RLIMIT_NPROC`：用户可创建的最大进程/线程数资源编号。
const RLIMIT_NPROC: u32 = 6;
/// `RLIMIT_NOFILE`：单进程最大文件描述符数量上限资源编号。
const RLIMIT_NOFILE: u32 = 7;
/// `RLIMIT_SIGPENDING`：最大挂起排队信号数量资源编号。
const RLIMIT_SIGPENDING: u32 = 11;
/// `RLIM_NLIMITS`：Linux ABI 定义的资源限制类别总数上限。
const RLIM_NLIMITS: u32 = 16;

/// `waitid` 选择器类型 `P_ALL`：等待任意子进程。
const P_ALL: i32 = 0;
/// `waitid` 选择器类型 `P_PID`：按指定的进程 ID（PID）等待单个子进程。
const P_PID: i32 = 1;
/// `waitid` 选择器类型 `P_PGID`：按指定的进程组 ID（PGID）等待组内任意子进程。
const P_PGID: i32 = 2;

/// 处理 `clone`（220）系统调用：按当前架构寄存器次序解码克隆参数并创建新进程或线程。
///
/// - **调用上下文**：普通可睡眠系统调用上下文。
/// - **返回值**：在父进程/调用线程中返回新建子任务的线程 ID（TID），失败返回对应 `Errno`。
pub(super) fn clone(user: &UserThread, request: Request) -> SysResult {
    let arguments = CloneArguments::decode(request.arguments).map_err(clone_error)?;
    clone::create(user, request, arguments)
        .map(|tid| tid as isize)
        .map_err(clone_error)
}

/// 处理 `execve`（221）系统调用：加载新可执行映像并替换当前进程地址空间与执行上下文。
///
/// - **调用上下文**：普通可睡眠系统调用上下文。
/// - **返回值**：准备阶段失败返回 `Outcome::Return(-errno)` 并保留原程序继续运行；跨过 `de_thread` 后成功返回 `Outcome::NewImage`，失败则走 `abort_exec` 终止整个线程组。
pub(super) fn execve(user: &UserThread, request: Request) -> Outcome {
    // Step 1: 在不破坏当前进程状态的前提下，读取路径/参数并完成新映像的解析与内存装载。
    let (image, runtime) = match prepare_exec(user, request.arguments) {
        Ok(prepared) => prepared,
        Err(error) => return Outcome::Return(error.result()),
    };

    // Step 2: 跨过不可回返点——回收同线程组的其他兄弟线程，并执行文件表 O_CLOEXEC 清理。
    let identity = user.identity().expect("exec has no identity");
    if identity.graph().de_thread(&identity).is_err() {
        return abort_exec(user);
    }
    let runtime = match runtime.try_exec() {
        Ok(runtime) => runtime,
        Err(_) => return abort_exec(user),
    };

    // Step 3: 安装新地址空间与入口上下文，并在 TaskGraph 中完成 exec 收尾（如重置信号动作、唤醒 vfork 父进程）。
    user.install_image(image, runtime);
    if identity.graph().complete_exec(&identity).is_err() {
        return abort_exec(user);
    }
    Outcome::NewImage
}

/// 在保持原进程现场不变的条件下，解析用户态路径与参数向量并构建待切换的 [`LoadedImage`]。
///
/// 路径、参数表与环境变量受当前线程组的 `RLIMIT_STACK` 软限制约束，并在同一文件系统根目录/工作目录快照与凭据下完成可执行文件或 `#!` 解释器加载。
fn prepare_exec(user: &UserThread, raw: [usize; 6]) -> Result<(LoadedImage, UserRuntime), Errno> {
    let runtime = user.runtime().expect("exec has no runtime");
    let mm = user.mm().expect("exec has no MM");
    let pathname = mm.read_c_string(raw[0], PATH_MAX)?;
    let stack_limit = user
        .identity()
        .expect("exec has no identity")
        .group()
        .resource_limit(Resource::Stack)
        .current;
    let arguments =
        Arguments::from_user(&mm, pathname, raw[1], raw[2], stack_limit).map_err(exec_error)?;

    let (vfs, context) = {
        let fs = runtime.fs.lock();
        (fs.vfs.clone(), fs.context.clone())
    };
    let image =
        exec::prepare(&vfs, &context, arguments, runtime.credentials).map_err(exec_error)?;
    Ok((image, runtime))
}

/// 处理跨过不可回返点（`de_thread`）之后的 `execve` 失败：终止当前线程组。
///
/// 此时原地址空间或兄弟线程已被销毁，无法再向旧用户态返回错误码；若线程组已有待处理的致命退出状态则沿用该状态，否则以致命 `SIGSEGV` 请求整个进程组退出。
fn abort_exec(user: &UserThread) -> Outcome {
    let identity = user.identity().expect("failed exec has no identity");
    let status = identity
        .graph()
        .exit_request(&identity)
        .unwrap_or(SIGSEGV as i32);
    identity.graph().request_exit(&identity, status);
    Outcome::GroupExit(status)
}

/// 处理 `getrlimit`（163）系统调用：读取当前线程组的指定资源软硬限制并写回用户空间。
pub(super) fn getrlimit(user: &UserThread, raw: [usize; 6]) -> SysResult {
    resource_limit(user, 0, raw[0] as u32, None, Some(raw[1]))
}

/// 处理 `setrlimit`（164）系统调用：从用户空间读取 16 字节限额并更新当前线程组的资源限制。
pub(super) fn setrlimit(user: &UserThread, raw: [usize; 6]) -> SysResult {
    resource_limit(user, 0, raw[0] as u32, Some(raw[1]), None)
}

/// 处理 `prlimit64`（261）系统调用：原子地更新和/或读取指定进程（`0` 表示本进程组）的资源限制。
pub(super) fn prlimit64(user: &UserThread, raw: [usize; 6]) -> SysResult {
    resource_limit(
        user,
        raw[0] as i32,
        raw[1] as u32,
        (raw[2] != 0).then_some(raw[2]),
        (raw[3] != 0).then_some(raw[3]),
    )
}

/// `getrlimit` / `setrlimit` / `prlimit64` 共用的资源限额读写实现。
///
/// - 所有用户空间 `copyin` / `copyout` 均在 `TaskGraph` 锁外完成；
/// - 若同时指定了新值与旧值缓冲区，先提交新限额并取得旧限额快照，随后即使 `copyout` 写回旧值失败也不会撤销已生效的新限额。
fn resource_limit(
    user: &UserThread,
    pid: i32,
    resource: u32,
    new_address: Option<usize>,
    old_address: Option<usize>,
) -> SysResult {
    // Step 1: 校验资源编号合法性并映射为内核已支持的 Resource 枚举。
    if RLIM_NLIMITS <= resource {
        return Err(Errno::InvalidArgument);
    }
    let resource = match resource {
        RLIMIT_STACK => Resource::Stack,
        RLIMIT_NPROC => Resource::Nproc,
        RLIMIT_NOFILE => Resource::NoFile,
        RLIMIT_SIGPENDING => Resource::SigPending,
        _ => return Err(Errno::NoSyscall),
    };

    // Step 2: 若提供了新限额地址，先在锁外从用户空间拷入 16 字节的 rlimit64 结构。
    let mm = user.mm().expect("rlimit has no MM");
    let new = match new_address {
        Some(address) => {
            let mut bytes = [0u8; 16];
            copyin(&mm, &mut bytes, address)?;
            Some(ResourceLimit {
                current: u64::from_le_bytes(bytes[..8].try_into().unwrap()),
                maximum: u64::from_le_bytes(bytes[8..].try_into().unwrap()),
            })
        }
        None => None,
    };
    if pid < 0 {
        return Err(Errno::NoProcess);
    }

    // Step 3: 通过 TaskGraph 校验权限、应用新限额并获取修改前的旧限额，最后按需写回用户空间。
    let identity = user.identity().expect("rlimit has no identity");
    let old = identity
        .graph()
        .prlimit(&identity, pid as usize, resource, new)?;

    if let Some(address) = old_address {
        let mut bytes = [0u8; 16];
        bytes[..8].copy_from_slice(&old.current.to_le_bytes());
        bytes[8..].copy_from_slice(&old.maximum.to_le_bytes());
        copyout(&mm, address, &bytes)?;
    }
    Ok(0)
}

/// 处理进程凭据、进程组、会话及父进程 ID 查询与设置系列系统调用。
///
/// 覆盖调用号：`154`（`setpgid`）、`155`（`getpgid`）、`156`（`getsid`）、`157`（`setsid`）、
/// `173`（`getppid`）、`174`（`getuid`）、`175`（`geteuid`）、`176`（`getgid`）、`177`（`getegid`）。
pub(super) fn identity(user: &UserThread, number: usize, raw: [usize; 6]) -> SysResult {
    if (174..=177).contains(&number) {
        let credentials = user
            .runtime()
            .expect("identity syscall has no runtime")
            .credentials;
        return Ok(match number {
            174 | 175 => credentials.uid as isize,
            176 | 177 => credentials.gid as isize,
            _ => unreachable!("credential dispatch outside range"),
        });
    }

    let identity = user.identity().expect("identity syscall has no identity");
    let graph = identity.graph();
    match number {
        154 => {
            let pid = raw[0] as i32;
            let pgid = raw[1] as i32;
            if pgid < 0 {
                return Err(Errno::InvalidArgument);
            }
            if pid < 0 {
                return Err(Errno::NoProcess);
            }
            graph.set_pgid(&identity, pid as usize, pgid as usize)?;
            Ok(0)
        }
        155 => Ok(graph.get_pgid(&identity, raw[0] as i32 as usize)? as isize),
        156 => Ok(graph.get_sid(&identity, raw[0] as i32 as usize)? as isize),
        157 => Ok(graph.create_session(&identity)? as isize),
        173 => Ok(graph.parent_pid(&identity) as isize),
        _ => unreachable!("unknown identity dispatch"),
    }
}

/// 处理 `waitid`（95）系统调用：按选择器类型等待子进程状态变化（退出、停止或继续），并输出 `siginfo_t` 与 `rusage`。
///
/// - `options` 必须至少包含 `WEXITED`、`WSTOPPED`、`WCONTINUED` 之一；
/// - 若指定 `WNOHANG` 且当前无就绪事件，向非空 `infop` 写入全零的 `siginfo_t` 并返回 `0`；
/// - 一旦子进程事件被消费（未设 `WNOWAIT` 时），后续向用户态拷贝 `rusage` 或 `siginfo_t` 失败不会将已回收的子进程恢复回僵尸态。
pub(super) fn waitid(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    // Step 1: 校验等待选项位掩码，并将 (idtype, id) 转换为稳定的内核身份对象选择器 WaitSelector。
    let options = raw[3] as u32;
    if options & !(NOHANG | STOPPED | EXITED | CONTINUED | NOWAIT | WALL | WCLONE) != 0
        || options & (STOPPED | EXITED | CONTINUED) == 0
    {
        return Outcome::Return(Errno::InvalidArgument.result());
    }

    let identity = user.identity().expect("waitid has no identity");
    let selector = match raw[0] as i32 {
        P_ALL => Ok(WaitSelector::Any),
        P_PID if raw[1] as i32 > 0 => identity
            .graph()
            .pid_object(raw[1] as i32 as usize)
            .map(WaitSelector::Pid)
            .ok_or(Errno::NoChild),
        P_PGID if raw[1] as i32 == 0 => Ok(WaitSelector::Group(identity.group().pgid_object())),
        P_PGID if raw[1] as i32 > 0 => identity
            .graph()
            .pid_object(raw[1] as i32 as usize)
            .map(WaitSelector::Group)
            .ok_or(Errno::NoChild),
        _ => Err(Errno::InvalidArgument),
    };
    let selector = match selector {
        Ok(selector) => selector,
        Err(error) => return Outcome::Return(error.result()),
    };

    // Step 2: 等待并获取满足条件的子进程状态事件（可能阻塞或因信号返回 Outcome::Restart）。
    let event = match wait_event(user, request, &selector, options) {
        Ok(event) => event,
        Err(outcome) => return outcome,
    };

    // Step 3: 将事件编码为 rusage 与 SIGCHLD 格式的 siginfo_t 写回用户态缓冲区。
    let result: SysResult = (|| {
        let mm = user.mm().expect("waitid has no MM");
        let mut bytes = [0u8; SIGINFO_BYTES];
        if let Some(event) = event {
            if raw[4] != 0 {
                copyout(&mm, raw[4], &usage(event))?;
            }
            SignalInfo {
                signum: SIGCHLD,
                errno: 0,
                code: event.code,
                payload: SignalPayload::Child {
                    pid: event.pid,
                    uid: event.uid,
                    status: event.status,
                    user_ticks: 0,
                    kernel_ticks: 0,
                },
            }
            .encode(&mut bytes);
        }
        if raw[2] != 0 {
            copyout(&mm, raw[2], &bytes)?;
        }
        Ok(0)
    })();
    Outcome::Return(result.unwrap_or_else(Errno::result))
}

/// 处理 `wait4`（260）系统调用：等待匹配的子进程退出、停止或继续，并输出传统 32 位等待状态字与 `rusage`。
///
/// `pid` 参数语义：
/// - `< -1`：等待进程组 ID 等于 `|pid|` 的任意子进程；
/// - `-1`：等待任意子进程；
/// - `0`：等待与调用进程处于同一进程组的任意子进程；
/// - `> 0`：等待进程 ID 等于 `pid` 的特定子进程。
pub(super) fn wait4(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
    // Step 1: 校验选项标志，并根据 pid 的正负与零值构造对应的 WaitSelector。
    let options = raw[2] as u32;
    if options & !(NOHANG | STOPPED | CONTINUED | WALL | WCLONE) != 0 {
        return Outcome::Return(Errno::InvalidArgument.result());
    }

    let identity = user.identity().expect("wait4 has no identity");
    let pid = raw[0] as i32;
    let selector = match pid {
        i32::MIN => Err(Errno::NoProcess),
        -1 => Ok(WaitSelector::Any),
        1.. => identity
            .graph()
            .pid_object(pid as usize)
            .map(WaitSelector::Pid)
            .ok_or(Errno::NoChild),
        0 => Ok(WaitSelector::Group(identity.group().pgid_object())),
        _ => identity
            .graph()
            .pid_object(pid.unsigned_abs() as usize)
            .map(WaitSelector::Group)
            .ok_or(Errno::NoChild),
    };
    let selector = match selector {
        Ok(selector) => selector,
        Err(error) => return Outcome::Return(error.result()),
    };

    // Step 2: 隐式附带 WEXITED 标志等待子进程事件；WNOHANG 且无就绪事件时直接返回 0（不修改用户态缓冲区）。
    let event = match wait_event(user, request, &selector, options | EXITED) {
        Ok(Some(event)) => event,
        Ok(None) => return Outcome::Return(0),
        Err(outcome) => return outcome,
    };

    // Step 3: 将子进程事件编码为传统 waitpid 32 位状态码，连同 rusage 一并写回用户空间并返回子进程 PID。
    let result: SysResult = (|| {
        let mm = user.mm().expect("wait4 has no MM");
        let status = match event.code {
            CLD_EXITED => event.status << EXIT_STATUS_SHIFT,
            CLD_KILLED => event.status,
            CLD_DUMPED => event.status | WAIT_CORE_DUMP,
            CLD_STOPPED => (event.status << EXIT_STATUS_SHIFT) | WAIT_STOPPED,
            CLD_CONTINUED => WAIT_CONTINUED,
            _ => unreachable!("unknown child event"),
        };
        if raw[1] != 0 {
            copyout(&mm, raw[1], &status.to_le_bytes())?;
        }
        if raw[3] != 0 {
            copyout(&mm, raw[3], &usage(event))?;
        }
        Ok(event.pid as isize)
    })();
    Outcome::Return(result.unwrap_or_else(Errno::result))
}

/// 在 `TaskGraph` 上执行带防丢失唤醒保护的可中断子进程事件等待循环。
///
/// - 若已有就绪子进程事件或无匹配子进程（`ECHILD`），立即返回；
/// - 若设置 `WNOHANG` 且暂无事件，返回 `Ok(None)`；
/// - 阻塞期间若被未屏蔽信号打断，返回 `Err(Outcome::Restart(Restart::Syscall { request }))`，交由信号派发路径根据 `SA_RESTART` 决定自动重启或返回 `-EINTR`。
fn wait_event(
    user: &UserThread,
    request: Request,
    selector: &WaitSelector,
    options: u32,
) -> Result<Option<ChildEvent>, Outcome> {
    let identity = user.identity().expect("wait has no identity");
    let group = identity.group();
    let graph = identity.graph();
    loop {
        match graph.try_wait_event(&group, selector, options) {
            Ok(Some(event)) => return Ok(Some(event)),
            Err(error) => return Err(Outcome::Return(Errno::from(error).result())),
            Ok(None) if options & NOHANG != 0 => return Ok(None),
            Ok(None) => {}
        }

        let waiter = Waiter::new_interruptible();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY:
        // 1. `graph` 来自上方持有的 `identity: Arc<...>` 内部引用，在整个循环及 `_registration` 存活期间底层 `WaitQueue` 始终保活；
        // 2. `node` 通过 `core::pin::pin!` 固定在当前栈帧上，且 `_registration` 声明在 `node` 与 `waiter` 之后，
        //    按 Rust 局部变量逆序析构规则，任何正常返回路径都会先执行 `_registration` 的 `Drop` 将节点从队列脱链，再析构栈上的 `node`，绝不会调用 `mem::forget`。
        let _registration = unsafe { graph.waiters().register(node.as_mut()) };
        match graph.try_wait_event(&group, selector, options) {
            Ok(Some(event)) => return Ok(Some(event)),
            Err(error) => return Err(Outcome::Return(Errno::from(error).result())),
            Ok(None) => {}
        }
        if waiter.park_signal() == WaitResult::Interrupted {
            return Err(Outcome::Restart(Restart::Syscall { request }));
        }
    }
}

/// 将 [`ChildEvent`] 中累计的用户态与内核态 CPU 纳秒耗时编码为 144 字节的 `struct rusage` 二进制表示。
///
/// 偏移 `0..16` 为 `ru_utime`（`struct timeval`：8 字节秒 + 8 字节微秒），偏移 `16..32` 为 `ru_stime`，其余暂未统计的字段固定填零。
fn usage(event: ChildEvent) -> [u8; 144] {
    let mut bytes = [0u8; 144];
    for (offset, ns) in [(0, event.user_ns), (16, event.kernel_ns)] {
        bytes[offset..offset + 8].copy_from_slice(&(ns / NSEC_PER_SEC).to_le_bytes());
        bytes[offset + 8..offset + 16]
            .copy_from_slice(&((ns % NSEC_PER_SEC) / NSEC_PER_USEC).to_le_bytes());
    }
    bytes
}

/// 将可执行文件装载错误 [`ExecError`] 映射为标准系统调用 [`Errno`]。
fn exec_error(error: ExecError) -> Errno {
    match error {
        ExecError::Format => Errno::ExecFormat,
        ExecError::ArgumentsTooBig => Errno::ArgumentsTooBig,
        ExecError::TooManyInterpreters => Errno::TooManySymlinks,
        ExecError::NoMemory | ExecError::Vm(VmError::NoMemory) => Errno::NoMemory,
        ExecError::File(error) | ExecError::Vm(VmError::File(error)) => error.into(),
        ExecError::Vm(_) => Errno::BadAddress,
    }
}

/// 将任务克隆错误 [`CloneError`] 映射为标准系统调用 [`Errno`]。
fn clone_error(error: CloneError) -> Errno {
    match error {
        CloneError::InvalidFlags => Errno::InvalidArgument,
        CloneError::Unsupported => Errno::NoSyscall,
        CloneError::NoMemory => Errno::NoMemory,
        CloneError::Vm(error) => error.into(),
        CloneError::File(error) => error.into(),
        CloneError::Identity(error) => error.into(),
    }
}
