//! `clone` 系统调用处理模块：创建新的用户进程或线程。
//!
//! ## 模块职责与架构定位
//! 本模块位于系统调用分发层（`crate::syscall`）与任务管理层（`crate::task`）之间：
//! - **上游调用者**：`sys_clone` 系统调用入口，传入原始寄存器参数与当前运行中的 [`UserThread`]。
//! - **下游依赖**：调用虚拟内存（`crate::mm::space::Mm`）克隆或共享地址空间、调用运行时上下文（`UserRuntime`）克隆或共享文件描述符表/文件系统上下文/信号处理表、调用进程关系图（`ProcessGraph`）分配 PID/TID 并注册父子或线程组成员关系，最终由 [`prepare_bound_user`] 构造可调度的子任务并投入调度器。
//!
//! ## 核心机制：资源共享维度与 ABI 寄存器布局
//! `clone` 将资源共享拆分为正交的标志位，按位组合决定创建独立进程还是同组线程：
//!
//! ```text
//!   clone(flags, stack, parent_tid, tls/child_tid, child_tid/tls)
//!     │
//!     ├── CLONE_VM       ──► 共享虚拟地址空间 Arc<Mm>（否则 fork 写时复制）
//!     ├── CLONE_FILES    ──► 共享打开文件表 FdTable（否则深拷贝）
//!     ├── CLONE_FS       ──► 共享根目录/当前工作目录/umask（否则深拷贝）
//!     ├── CLONE_SIGHAND  ──► 共享信号处理动作表（要求同时设置 CLONE_VM）
//!     ├── CLONE_THREAD   ──► 加入同一线程组共享 TGID（要求同时设置 CLONE_SIGHAND）
//!     └── CLONE_VFORK    ──► 挂起父任务，直到子任务 execve 替换或退出释放旧 Mm
//! ```
//!
//! ### 双架构 `clone` 参数寄存器差异
//! Linux `clone` 系统调用在不同架构下的第 4、5 个参数顺序不同，由 [`CloneArguments::decode`] 统一归一化：
//!
//! | 参数序号 | `raw` 下标 | RISC-V 64 (`riscv64`) | LoongArch 64 (`loongarch64`) |
//! |----------|------------|-----------------------|------------------------------|
//! | 1        | `raw[0]`   | `flags`               | `flags`                      |
//! | 2        | `raw[1]`   | `stack` (`x2`/`sp`)   | `stack` (`r3`/`sp`)          |
//! | 3        | `raw[2]`   | `parent_tid`          | `parent_tid`                 |
//! | 4        | `raw[3]`   | `tls` (`x4`/`tp`)     | `child_tid`                  |
//! | 5        | `raw[4]`   | `child_tid`           | `tls` (`r2`/`tp`)            |
//!
//! ## 锁获取顺序与并发契约
//! 1. **锁外预分配原则**：在 [`create`] 中，子任务的 `UserRuntime`、`Mm`（含写时复制页表拷贝与 TLB 刷新）、`Identity` 预订以及内核栈/任务控制块分配均在获取进程图全局发布锁之前完成，避免持锁期间执行耗时的内存分配或页表操作。
//! 2. **发布与唤醒顺序**：通过 `program.publish_child` 完成父空间 `parent_tid` 写出（若开启 `CLONE_PARENT_SETTID`）、将子任务正式插入进程图并标记为可调度状态；若开启 `CLONE_VFORK`，父任务在释放子任务强引用句柄后进入可被致命信号中断的睡眠等待（`wait_killable`）。

use alloc::sync::Arc;

use crate::fs::FsError;
use crate::mm::space::VmError;
use crate::sync::Completion;
use crate::syscall::Request;
use crate::task::exec::LoadedImage;
use crate::task::process::ProcessError;
use crate::task::user::{prepare_bound_user, ProgramError};
use crate::task::{Placement, UserThread};
use crate::wait::WaitResult;

use crate::task::signal::{SIGCHLD, SIGNAL_COUNT};

/// 子进程退出时向父进程发送的信号掩码（低 8 位）。
const CSIGNAL: usize = 0xff;

/// `CLONE_VM`：父子任务共享同一虚拟地址空间（`Mm`）。
const VM: usize = 0x100;
/// `CLONE_FS`：父子任务共享文件系统上下文（根目录、当前工作目录与 `umask`）。
const FS: usize = 0x200;
/// `CLONE_FILES`：父子任务共享打开文件描述符表。
const FILES: usize = 0x400;
/// `CLONE_SIGHAND`：父子任务共享信号处理函数表（必须同时指定 `CLONE_VM`）。
const SIGHAND: usize = 0x800;
/// `CLONE_VFORK`：阻塞父任务，直至子任务调用 `execve` 替换地址空间或退出。
const VFORK: usize = 0x4000;
/// `CLONE_THREAD`：将子任务放入父任务所在的线程组（必须同时指定 `CLONE_SIGHAND`）。
const THREAD: usize = 0x1_0000;
/// `CLONE_SYSVSEM`：共享 System V 信号量 `SEM_UNDO` 列表（本内核未实现 SysV IPC，该列表为空，作为兼容标志放行）。
const SYSVSEM: usize = 0x4_0000;
/// `CLONE_SETTLS`：将子任务的线程本地存储（TLS）寄存器设置为指定值。
const SETTLS: usize = 0x8_0000;
/// `CLONE_PARENT_SETTID`：在父任务用户空间地址写入新创建的子任务 TID。
const PARENT_SETTID: usize = 0x10_0000;
/// `CLONE_CHILD_CLEARTID`：子任务退出时清零指定用户空间地址并唤醒其上的 `futex` 等待者。
const CHILD_CLEARTID: usize = 0x20_0000;
/// `CLONE_DETACHED`：历史遗留标志位，Linux ABI 中已忽略，生命周期回收仍由进程关系图管理。
const DETACHED: usize = 0x40_0000;
/// `CLONE_CHILD_SETTID`：在子任务用户空间地址写入新创建的子任务 TID。
const CHILD_SETTID: usize = 0x100_0000;
/// 当前内核支持的所有 `clone` 标志位掩码合集。
const SUPPORTED: usize = CSIGNAL
    | VM
    | FS
    | FILES
    | SIGHAND
    | VFORK
    | THREAD
    | SYSVSEM
    | SETTLS
    | PARENT_SETTID
    | CHILD_SETTID
    | CHILD_CLEARTID
    | DETACHED;

/// RISC-V 64 架构下用户栈指针寄存器（`sp` / `x2`）在通用寄存器数组中的下标。
#[cfg(target_arch = "riscv64")]
const STACK_REGISTER: usize = 2;
/// LoongArch 64 架构下用户栈指针寄存器（`sp` / `r3`）在通用寄存器数组中的下标。
#[cfg(target_arch = "loongarch64")]
const STACK_REGISTER: usize = 3;

/// RISC-V 64 架构下线程指针寄存器（`tp` / `x4`）在通用寄存器数组中的下标。
#[cfg(target_arch = "riscv64")]
const TLS_REGISTER: usize = 4;
/// LoongArch 64 架构下线程指针寄存器（`tp` / `r2`）在通用寄存器数组中的下标。
#[cfg(target_arch = "loongarch64")]
const TLS_REGISTER: usize = 2;

/// `clone` 操作可能返回的错误类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CloneError {
    /// 标志位组合违反 POSIX/Linux 约束（如指定 `CLONE_THREAD` 却未指定 `CLONE_SIGHAND`，或退出信号编号越界）。
    InvalidFlags,
    /// 包含未支持的 `clone` 标志位，或非线程克隆指定了除 `0` 和 `SIGCHLD` 以外的退出信号。
    Unsupported,
    /// 内核堆或物理页分配失败（对应 `ENOMEM`）。
    NoMemory,
    /// 虚拟内存操作失败（如地址空间 `fork` 或用户态指针访问错误）。
    Vm(VmError),
    /// 文件描述符表或文件系统上下文克隆失败（如超出资源上限）。
    File(FsError),
    /// 进程/线程身份分配或进程关系图状态错误（如父进程正在退出）。
    Identity(ProcessError),
}

/// 解码后的架构无关 `clone` 参数。
///
/// 各 `Option<usize>` 字段对应是否设置了相应的 `CLONE_*` 标志；
/// 只要标志位置位，即便传入的用户态地址为 `0`，也会封装为 `Some(0)` 并在后续流程中触发真实的用户地址访问与缺页/错误检查。
pub(crate) struct CloneArguments {
    /// 是否共享虚拟地址空间（`CLONE_VM`）。
    share_vm: bool,
    /// 是否共享打开文件描述符表（`CLONE_FILES`）。
    share_files: bool,
    /// 是否共享文件系统根目录、当前工作目录与 `umask`（`CLONE_FS`）。
    share_fs: bool,
    /// 是否共享信号处理函数表（`CLONE_SIGHAND`）。
    share_sighand: bool,
    /// 是否作为同线程组线程创建（`CLONE_THREAD`）。
    thread: bool,
    /// 是否在子任务释放或替换地址空间前阻塞父任务（`CLONE_VFORK`）。
    vfork: bool,
    /// 子进程退出时发送给父进程的信号编号（低 8 位，`0` 表示不发送信号）。
    exit_signal: u8,
    /// 子任务的用户栈指针；为 `0` 时沿用父任务的当前栈指针。
    stack: usize,
    /// 若设置了 `CLONE_SETTLS`，保存写入子任务 TLS 寄存器的值。
    tls: Option<usize>,
    /// 若设置了 `CLONE_PARENT_SETTID`，保存父任务用户空间中用于接收子 TID 的虚拟地址。
    parent_tid: Option<usize>,
    /// 若设置了 `CLONE_CHILD_SETTID`，保存子任务用户空间中用于接收子 TID 的虚拟地址。
    child_tid: Option<usize>,
    /// 若设置了 `CLONE_CHILD_CLEARTID`，保存子任务退出时需清零并唤醒 `futex` 的用户虚拟地址；未设置时为 `0`。
    clear_child_tid: usize,
}

impl CloneArguments {
    /// 校验并按当前目标架构 ABI 解码系统调用原始 6 元组参数。
    ///
    /// 在普通系统调用上下文中调用。检查标志位掩码合法性及 `CLONE_THREAD`、`CLONE_SIGHAND`、`CLONE_VM` 之间的依赖关系。
    pub(crate) fn decode(raw: [usize; 6]) -> Result<Self, CloneError> {
        let flags = raw[0];
        if flags & !SUPPORTED != 0 {
            return Err(CloneError::Unsupported);
        }
        let share_vm = flags & VM != 0;
        let share_sighand = flags & SIGHAND != 0;
        let thread = flags & THREAD != 0;
        if (thread && !share_sighand)
            || (share_sighand && !share_vm)
            || SIGNAL_COUNT < (flags & CSIGNAL)
        {
            return Err(CloneError::InvalidFlags);
        }
        if !thread && !matches!((flags & CSIGNAL) as u8, 0 | SIGCHLD) {
            return Err(CloneError::Unsupported);
        }

        #[cfg(target_arch = "riscv64")]
        let (tls, child_tid) = (raw[3], raw[4]);
        #[cfg(target_arch = "loongarch64")]
        let (tls, child_tid) = (raw[4], raw[3]);
        Ok(Self {
            share_vm,
            share_files: flags & FILES != 0,
            share_fs: flags & FS != 0,
            share_sighand,
            thread,
            vfork: flags & VFORK != 0,
            exit_signal: (flags & CSIGNAL) as u8,
            stack: raw[1],
            tls: (flags & SETTLS != 0).then_some(tls),
            parent_tid: (flags & PARENT_SETTID != 0).then_some(raw[2]),
            child_tid: (flags & CHILD_SETTID != 0).then_some(child_tid),
            clear_child_tid: if flags & CHILD_CLEARTID != 0 {
                child_tid
            } else {
                0
            },
        })
    }
}

/// 根据解码后的 [`CloneArguments`] 创建并发布新的用户进程或线程。
///
/// 在可睡眠的系统调用上下文中调用。成功时返回新子任务的线程/进程 ID（`pid`）。
/// 若设置了 `CLONE_VFORK`，本函数会在子任务发布后挂起当前父任务，直到子任务调用 `execve`、退出或当前父任务收到致命信号为止。
pub(crate) fn create(
    user: &UserThread,
    request: Request,
    arguments: CloneArguments,
) -> Result<usize, CloneError> {
    let identity = user.identity().expect("clone has no user identity");
    let group = identity.group();
    let graph = identity.graph();
    let parent_mm = user.mm().expect("clone has no parent MM");
    // Step 1: 克隆或共享运行时资源（文件表、文件系统上下文、信号处理表）与待决信号掩码。
    let runtime = user
        .runtime()
        .expect("clone has no user runtime")
        .try_fork(
            arguments.share_files,
            arguments.share_fs,
            arguments.share_sighand,
        )
        .map_err(CloneError::File)?;
    let inherited = identity
        .signals()
        .inherit(arguments.share_vm && !arguments.vfork);

    // Step 2: 构造子任务的用户态陷入上下文，令子任务从系统调用返回 0，并按需覆盖栈指针与 TLS 寄存器。
    let mut context = user.context().fork();
    request.return_to(&mut context, 0);
    if arguments.stack != 0 {
        context.gpr[STACK_REGISTER] = arguments.stack;
    }
    if let Some(tls) = arguments.tls {
        context.gpr[TLS_REGISTER] = tls;
    }

    // Step 3: 准备虚拟地址空间（共享 Arc<Mm> 或执行写时复制 fork）并在进程关系图中预订身份槽位。
    let mm = if arguments.share_vm {
        parent_mm.clone()
    } else {
        Arc::try_new(parent_mm.fork().map_err(CloneError::Vm)?).map_err(|_| CloneError::NoMemory)?
    };

    let identity = if arguments.thread {
        graph.prepare_thread(&group, inherited)
    } else {
        graph.prepare_process(
            Some(&group),
            runtime.sighand.clone(),
            runtime.credentials.uid,
            inherited,
            arguments.exit_signal,
        )
    }
    .map_err(CloneError::Identity)?;
    let program = prepare_bound_user(
        identity,
        LoadedImage { mm, context },
        runtime,
        Placement::Any,
        arguments.child_tid,
        arguments.clear_child_tid,
    )
    .map_err(program_error)?;

    // Step 4: 继承父任务的 CPU 亲和性掩码，并在需要 vfork 时绑定完成通知原语。
    let parent = user.executor().expect("clone has no parent executor");
    let child = program
        .user()
        .executor()
        .expect("clone has no child executor");
    crate::sched::inherit_affinity(&parent, &child);
    let pid = program.pid();
    let vfork = if arguments.vfork {
        let completion = Arc::try_new(Completion::new()).map_err(|_| CloneError::NoMemory)?;
        let child = program.user();
        child.bind_vfork(completion.clone());
        Some((Arc::downgrade(&child), completion))
    } else {
        None
    };

    // Step 5: 将子任务正式发布到进程关系图并投入就绪队列（期间向父空间写入 parent_tid）。
    let handle = program
        .publish_child(&parent_mm, arguments.parent_tid)
        .map_err(CloneError::Identity)?;
    // 立即释放局部强引用句柄，避免父任务在下方 vfork 阻塞等待期间无意延长子任务执行体的生命周期。
    drop(handle);
    if let Some((child, completion)) = vfork {
        if completion.wait_killable() == WaitResult::Interrupted {
            if let Some(child) = child.upgrade() {
                child.cancel_vfork(&completion);
            }
        }
    }
    Ok(pid)
}

/// 将用户程序准备阶段的 [`ProgramError`] 转换为 [`CloneError`]。
fn program_error(error: ProgramError) -> CloneError {
    match error {
        ProgramError::Identity(error) => CloneError::Identity(error),
        ProgramError::Image(crate::task::exec::ExecError::NoMemory) => CloneError::NoMemory,
        ProgramError::Image(crate::task::exec::ExecError::Vm(error)) => CloneError::Vm(error),
        ProgramError::Image(crate::task::exec::ExecError::File(error)) => CloneError::File(error),
        ProgramError::Image(_) => panic!("clone preparation unexpectedly parsed an image"),
    }
}
