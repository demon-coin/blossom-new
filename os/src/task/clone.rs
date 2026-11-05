//! 原始clone请求转为独立用户执行者；资源共享按每个标志自己的轴决定。

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

const CSIGNAL: usize = 0xff;

const VM: usize = 0x100;
const FS: usize = 0x200;
const FILES: usize = 0x400;
const SIGHAND: usize = 0x800;
const VFORK: usize = 0x4000;
const THREAD: usize = 0x1_0000;
// 未实现 SysV IPC 时 undo 集为空；该位不增加新的执行资源。
const SYSVSEM: usize = 0x4_0000;
const SETTLS: usize = 0x8_0000;
const PARENT_SETTID: usize = 0x10_0000;
const CHILD_CLEARTID: usize = 0x20_0000;
// 历史 DETACHED 位不决定成员回收，真实 join 与身份退休仍由原图承担。
const DETACHED: usize = 0x40_0000;
const CHILD_SETTID: usize = 0x100_0000;
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

#[cfg(target_arch = "riscv64")]
const STACK_REGISTER: usize = 2;
#[cfg(target_arch = "loongarch64")]
const STACK_REGISTER: usize = 3;

#[cfg(target_arch = "riscv64")]
const TLS_REGISTER: usize = 4;
#[cfg(target_arch = "loongarch64")]
const TLS_REGISTER: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CloneError {
    InvalidFlags,
    Unsupported,
    NoMemory,
    Vm(VmError),
    File(FsError),
    Identity(ProcessError),
}

/// 架构raw参数只在此解码；Option保存对应SET标志，即使地址为零也必须尝试交付。
pub(crate) struct CloneArguments {
    share_vm: bool,
    share_files: bool,
    share_fs: bool,
    share_sighand: bool,
    thread: bool,
    vfork: bool,
    exit_signal: u8,
    stack: usize,
    tls: Option<usize>,
    parent_tid: Option<usize>,
    child_tid: Option<usize>,
    clear_child_tid: usize,
}

impl CloneArguments {
    /// 按目标 ABI 解码共享轴；空 SysV undo 与历史 DETACHED 不改变线程的真实退出责任。
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

/// 锁外准备全部成员拥有，父COW/TLB结束、图发表和父TID尝试完成后才移交child Ready。
pub(crate) fn create(
    user: &UserThread,
    request: Request,
    arguments: CloneArguments,
) -> Result<usize, CloneError> {
    let identity = user.identity().expect("clone has no user identity");
    let group = identity.group();
    let graph = identity.graph();
    let parent_mm = user.mm().expect("clone has no parent MM");
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

    let mut context = user.context().fork();
    request.return_to(&mut context, 0);
    if arguments.stack != 0 {
        context.gpr[STACK_REGISTER] = arguments.stack;
    }
    if let Some(tls) = arguments.tls {
        context.gpr[TLS_REGISTER] = tls;
    }

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

    let handle = program
        .publish_child(&parent_mm, arguments.parent_tid)
        .map_err(CloneError::Identity)?;
    // 图只保离栈观察；vfork只等child结束旧Mm使用，不持child Task或运行资源。
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

/// 未发表成员的准备失败保持文件、VM、身份与真实分配原因，不转换为已创建成功。
fn program_error(error: ProgramError) -> CloneError {
    match error {
        ProgramError::Identity(error) => CloneError::Identity(error),
        ProgramError::Image(crate::task::exec::ExecError::NoMemory) => CloneError::NoMemory,
        ProgramError::Image(crate::task::exec::ExecError::Vm(error)) => CloneError::Vm(error),
        ProgramError::Image(crate::task::exec::ExecError::File(error)) => CloneError::File(error),
        ProgramError::Image(_) => panic!("clone preparation unexpectedly parsed an image"),
    }
}
