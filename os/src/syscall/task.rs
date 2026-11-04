//! 用户任务ABI只复制已初始化字节，状态消费与用户交付分开完成。

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

const RLIMIT_STACK: u32 = 3;
const RLIMIT_NPROC: u32 = 6;
const RLIMIT_NOFILE: u32 = 7;
const RLIMIT_SIGPENDING: u32 = 11;
const RLIM_NLIMITS: u32 = 16;
const P_ALL: i32 = 0;
const P_PID: i32 = 1;
const P_PGID: i32 = 2;

/// 按架构次序解码本次六参数并发表真实child，返回用户TID而非TaskId。
pub(super) fn clone(user: &UserThread, request: Request) -> SysResult {
    let arguments = CloneArguments::decode(request.arguments).map_err(clone_error)?;
    clone::create(user, request, arguments)
        .map(|tid| tid as isize)
        .map_err(clone_error)
}

/// 准备失败保留旧程序，跨不可回返点后的任何失败只沿真实组退出清理。
pub(super) fn execve(user: &UserThread, request: Request) -> Outcome {
    let (image, runtime) = match prepare_exec(user, request.arguments) {
        Ok(prepared) => prepared,
        Err(error) => return Outcome::Return(error.result()),
    };
    let identity = user.identity().expect("exec has no identity");
    if identity.graph().de_thread(&identity).is_err() {
        return abort_exec(user);
    }
    let runtime = match runtime.try_exec() {
        Ok(runtime) => runtime,
        Err(_) => return abort_exec(user),
    };
    user.install_image(image, runtime);
    if identity.graph().complete_exec(&identity).is_err() {
        return abort_exec(user);
    }
    Outcome::NewImage
}

/// 路径与参数只读取一次，在同一 FS、凭据和组主栈限额快照下准备 ELF 或脚本映像。
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

/// 不可回返点后不伪造errno返回旧现场，已存在的致命组原因保持优先。
fn abort_exec(user: &UserThread) -> Outcome {
    let identity = user.identity().expect("failed exec has no identity");
    let status = identity
        .graph()
        .exit_request(&identity)
        .unwrap_or(SIGSEGV as i32);
    identity.graph().request_exit(&identity, status);
    Outcome::GroupExit(status)
}

/// getrlimit 的输出指针必须完整交付两个 LP64 字段，空指针保真实 EFAULT。
pub(super) fn getrlimit(user: &UserThread, raw: [usize; 6]) -> SysResult {
    resource_limit(user, 0, raw[0] as u32, None, Some(raw[1]))
}

/// setrlimit 先导入完整 LP64 值，提交只影响组限额，不修改已经存在的栈 VMA。
pub(super) fn setrlimit(user: &UserThread, raw: [usize; 6]) -> SysResult {
    resource_limit(user, 0, raw[0] as u32, Some(raw[1]), None)
}

/// prlimit64 的零 PID 选择本组；新旧指针为空时分别省略修改或交付。
pub(super) fn prlimit64(user: &UserThread, raw: [usize; 6]) -> SysResult {
    resource_limit(
        user,
        raw[0] as i32,
        raw[1] as u32,
        (raw[2] != 0).then_some(raw[2]),
        (raw[3] != 0).then_some(raw[3]),
    )
}

/// 用户复制全部在 Graph 和组限额锁外，旧值 copyout 故障不撤销已经提交的新值。
fn resource_limit(
    user: &UserThread,
    pid: i32,
    resource: u32,
    new_address: Option<usize>,
    old_address: Option<usize>,
) -> SysResult {
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

/// 凭据查询返回当前真实 uid/gid；进程组、会话和亲缘查询继续使用原身份图。
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

/// waitid按原身份对象筛选；领取后rusage或siginfo交付失败不恢复事件。
pub(super) fn waitid(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
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
    let event = match wait_event(user, request, &selector, options) {
        Ok(event) => event,
        Err(outcome) => return outcome,
    };
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

/// wait4的负PID选择进程组，零选择本组；无事件不改status和rusage。
pub(super) fn wait4(user: &UserThread, request: Request) -> Outcome {
    let raw = request.arguments;
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
    let event = match wait_event(user, request, &selector, options | EXITED) {
        Ok(Some(event)) => event,
        Ok(None) => return Outcome::Return(0),
        Err(outcome) => return outcome,
    };
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

/// 同一Waiter登记后重查真实事件，中断只授权原PC/参数的内部重启。
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
        // SAFETY: identity保活通知源；node固定本栈，registration先于node/Waiter撤除。
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

/// timeval使用真实组及已reaped后代CPU账；尚无生产计数的其他rusage字段明确为零。
fn usage(event: ChildEvent) -> [u8; 144] {
    let mut bytes = [0u8; 144];
    for (offset, ns) in [(0, event.user_ns), (16, event.kernel_ns)] {
        bytes[offset..offset + 8].copy_from_slice(&(ns / NSEC_PER_SEC).to_le_bytes());
        bytes[offset + 8..offset + 16]
            .copy_from_slice(&((ns % NSEC_PER_SEC) / NSEC_PER_USEC).to_le_bytes());
    }
    bytes
}

/// ELF准备保格式和参数预算，用户参数访问错误独立编码为EFAULT。
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

/// 创建错误只在syscall边界编码，不把未支持标志或失败构造伪装为child存在。
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
