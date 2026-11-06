//! POSIX/Linux 信号处置表、线程组控制与线程私有信号递送子系统。
//!
//! ## 模块职责与架构定位
//! 本模块实现 64 个标准与实时信号的完整生命周期管理，包括信号处置表（`Sighand`）、线程组共享信号与作业停止/继续/致命退出控制（`SignalGroup`）、线程私有阻塞掩码与备用信号栈（`ThreadSignals`），以及 128 字节 UAPI `siginfo_t` 编码。
//! - **上游调用者**：进程关系图 `crate::task::process`（协调跨进程/线程组发送、父子 `SIGCHLD` 通知与成员增删）、用户态执行循环 `crate::task::user`（返回用户态前领取信号构造处理帧、硬件异常同步强制信号）、系统调用层（`rt_sigaction`、`rt_sigprocmask`、`rt_sigtimedwait`、`sigaltstack`、`rt_sigreturn` 等）及间隔定时器 `crate::task::time`。
//! - **下游依赖**：子模块 `self::pending`（混合 FIFO 待处理队列与延迟锁外析构）、`crate::wait`（唤醒处于 `Interruptible` 或 `Killable` 睡眠状态的线程）。
//!
//! ## 核心设计与数据结构分层
//! 1. **三级状态解耦**：
//!    - `Sighand`：保存 64 个信号的处置动作（`SignalAction`），可通过 `CLONE_SIGHAND` 在不同线程组之间共享，但 `execve` 时会解绑私有化。
//!    - `SignalGroup`：每个线程组（进程）独占一份，维护组级待处理队列（`group.pending`）、全体成员作业停止计数与组退出码；即使两个进程共享 `Sighand`，其待处理信号与组控制状态也严格隔离。
//!    - `ThreadSignals`：每个线程独占一份，维护线程私有待处理队列（`private.pending`）、硬件同步异常槽位（`fault`）、信号阻塞掩码（`mask`）、备用信号栈（`altstack`）以及当前注册的可中断睡眠句柄（`wait`）。
//! 2. **128 字节 `siginfo_t` 内存布局（RISC-V 64 / LoongArch 64 通用小端格式）**：
//!
//! ```text
//!   Byte Offset | Field / Union Variant
//!  -------------+-------------------------------------------------------------
//!   0..4        | si_signo  (i32)
//!   4..8        | si_errno  (i32)
//!   8..12       | si_code   (i32)
//!   12..16      | _pad0     (4 bytes zeroed alignment padding)
//!   16..48      | _sifields union:
//!               | - Sender : [16..20] pid (i32), [20..24] uid (u32), [24..32] sigval (usize)
//!               | - Fault  : [16..24] addr (usize)
//!               | - Child  : [16..20] pid (i32), [20..24] uid (u32), [24..28] status (i32),
//!               |            [28..32] pad, [32..40] utime (u64), [40..48] stime (u64)
//!   48..128     | zeroed tail padding (total 128 bytes)
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - **严格锁层级**：`TaskGraph` 关系锁（外层可选） ➔ `SignalGroup::sighand`（动作表绑定自旋锁） ➔ `Sighand::actions`（信号动作自旋锁 `siglock`） ➔ `SignalGroup::state`（线程组状态自旋锁） ➔ `ThreadSignals::state`（线程私有状态自旋锁；遍历多线程时按线程列表顺序逐个短暂获取并释放）。
//! - **锁外通知与锁外析构铁律**：严禁在持有任何信号自旋锁期间调用 `Waker::notify()`、反查 `TaskGraph` 或释放 `QueuedSignal` 堆节点；所有唤醒通知与 `Retired` 节点回收必须在释放全部信号自旋锁后执行。

use alloc::alloc::AllocError;
use alloc::sync::{Arc, Weak};
use core::mem;
use core::sync::atomic::{AtomicUsize, Ordering};

pub(crate) use self::pending::PreparedSignal;
use self::pending::{Pending, Retired};
use crate::arch::UserContext;
use crate::sync::SpinLock;
use crate::task::process::{TaskGraph, ThreadGroup};

mod pending;
use crate::wait::{WaitId, WaitMode, Waker};

/// 系统支持的信号编号总数（1..=64，涵盖标准信号 1..31 与实时信号 32..64）。
pub(crate) const SIGNAL_COUNT: usize = 64;
/// 终端挂断或控制进程终止信号（1）。
pub(crate) const SIGHUP: u8 = 1;
/// 终端键盘中断信号（2，通常由 Ctrl-C 触发）。
pub(crate) const SIGINT: u8 = 2;
/// 终端键盘退出信号（3，通常由 Ctrl-\ 触发）。
pub(crate) const SIGQUIT: u8 = 3;
/// 非法指令同步硬件异常信号（4）。
pub(crate) const SIGILL: u8 = 4;
/// 断点或调试陷阱同步硬件异常信号（5）。
pub(crate) const SIGTRAP: u8 = 5;
#[expect(
    dead_code,
    reason = "Linux 通用 signal ABI 将 SIGABRT 固定为 6，保留命名以区分用户传入的终止信号编号"
)]
/// 进程异常中止信号（6，通常由用户态 `abort()` 发送）。
pub(crate) const SIGABRT: u8 = 6;
/// 总线错误（如未对齐访问或超出文件映射末尾）信号（7）。
pub(crate) const SIGBUS: u8 = 7;
/// 算术或浮点异常信号（8）。
pub(crate) const SIGFPE: u8 = 8;
/// 不可捕获、不可阻塞、不可忽略的强制终止信号（9）。
pub(crate) const SIGKILL: u8 = 9;
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "futex 中断与重启测试以 Linux SIGUSR1 递送用户处理器信号"
    )
)]
/// 用户自定义信号 1（10）。
pub(crate) const SIGUSR1: u8 = 10;
/// 段错误（无效虚拟内存映射或权限违例）同步异常信号（11）。
pub(crate) const SIGSEGV: u8 = 11;
/// 向无读端的管道或套接字写入时触发的管道破裂信号（13）。
pub(crate) const SIGPIPE: u8 = 13;
/// 真实时间间隔定时器（`ITIMER_REAL`）到期信号（14）。
pub(crate) const SIGALRM: u8 = 14;
/// 子进程退出、停止或继续运行时向父进程发送的通知信号（17）。
pub(crate) const SIGCHLD: u8 = 17;
/// 使已停止的作业恢复继续运行的控制信号（18）。
pub(crate) const SIGCONT: u8 = 18;
/// 不可捕获、不可阻塞、不可忽略的强制停止作业信号（19）。
pub(crate) const SIGSTOP: u8 = 19;
/// 终端驱动发出的交互式停止信号（20，通常由 Ctrl-Z 触发）。
pub(crate) const SIGTSTP: u8 = 20;
/// 后台进程组尝试读取控制终端时触发的停止信号（21）。
pub(crate) const SIGTTIN: u8 = 21;
/// 后台进程组尝试写入控制终端时触发的停止信号（22）。
pub(crate) const SIGTTOU: u8 = 22;
/// 套接字上出现带外紧急数据（Out-of-Band）通知信号（23）。
pub(crate) const SIGURG: u8 = 23;
/// 用户态 CPU 时间间隔定时器（`ITIMER_VIRTUAL`）到期信号（26）。
pub(crate) const SIGVTALRM: u8 = 26;
/// 进程总 CPU 时间性能剖析定时器（`ITIMER_PROF`）到期信号（27）。
pub(crate) const SIGPROF: u8 = 27;
/// 控制终端窗口大小变更信号（28）。
pub(crate) const SIGWINCH: u8 = 28;
/// 非法系统调用参数或沙箱拦截信号（31）。
pub(crate) const SIGSYS: u8 = 31;
/// 实时信号（Real-time Signals）起始编号（32）。
pub(crate) const SIGRTMIN: u8 = 32;

/// 用户态 ABI 标准 `siginfo_t` 结构体的固定字节长度（128 字节）。
pub(crate) const SIGINFO_BYTES: usize = 128;
/// 64 位信号集 `sigset_t` 的字节长度（8 字节）。
pub(crate) const SIGSET_BYTES: usize = core::mem::size_of::<u64>();
/// 表示信号由内核直接生成的 `si_code` 常量（`0x80`）。
const SI_KERNEL: i32 = 128;

/// 默认信号处理方式（`SIG_DFL`）：按信号编号执行终止、忽略或停止等内核默认语义。
pub(crate) const SIG_DFL: usize = 0;
/// 显式忽略信号（`SIG_IGN`）：丢弃该信号且不递送给用户态。
pub(crate) const SIG_IGN: usize = 1;
/// `SIGCHLD` 行为标志：子进程暂停或恢复运行时不向父进程发送 `SIGCHLD`。
const SA_NOCLDSTOP: usize = 1;

/// `SIGCHLD` 的 `si_code`：子进程正常调用 `exit` 退出。
pub(crate) const CLD_EXITED: i32 = 1;
/// `SIGCHLD` 的 `si_code`：子进程被致命信号终止（未生成核心转储）。
pub(crate) const CLD_KILLED: i32 = 2;
/// `SIGCHLD` 的 `si_code`：子进程被致命信号终止并伴随核心转储语义。
pub(crate) const CLD_DUMPED: i32 = 3;
/// `SIGCHLD` 的 `si_code`：子进程因停止信号进入暂停状态。
pub(crate) const CLD_STOPPED: i32 = 5;
/// `SIGCHLD` 的 `si_code`：已暂停的子进程因 `SIGCONT` 恢复运行。
pub(crate) const CLD_CONTINUED: i32 = 6;
/// `tkill` / `tgkill` 发送线程定向信号时填写的 `si_code`。
pub(crate) const SI_TKILL: i32 = -6;
/// `sigaction` 标志：使用三参数信号处理函数原型并传递完整 `siginfo_t` 与 `ucontext_t`。
const SA_SIGINFO: usize = 4;
/// 永远禁止被阻塞或忽略的信号位掩码（包含 `SIGKILL` 与 `SIGSTOP`）。
pub(crate) const UNBLOCKABLE: u64 = (1 << (SIGKILL - 1)) | (1 << (SIGSTOP - 1));

/// `sigaction` 标志：若当前线程配置了启用的备用信号栈（`sigaltstack`），则切到备用栈上建立信号帧。
pub(crate) const SA_ONSTACK: usize = 0x0800_0000;
/// `sigaction` 标志：信号处理函数返回后自动重启被中断的可重启系统调用。
pub(crate) const SA_RESTART: usize = 0x1000_0000;
/// `sigaction` 标志：执行该信号的用户处理函数期间，不自动阻塞该信号自身。
pub(crate) const SA_NODEFER: usize = 0x4000_0000;
/// `sigaction` 标志：进入用户信号处理函数前一次性将该信号的处置重置为 `SIG_DFL`。
pub(crate) const SA_RESETHAND: usize = 0x8000_0000;
/// `stack_t::ss_flags` 状态位：查询时报告当前用户栈指针正位于备用信号栈范围内。
pub(crate) const SS_ONSTACK: u32 = 1;
/// `stack_t::ss_flags` 控制位：禁用备用信号栈。
pub(crate) const SS_DISABLE: u32 = 2;
/// `SIGCHLD` 行为标志：子进程退出后立即自动回收，不保留僵尸（Zombie）状态。
const SA_NOCLDWAIT: usize = 2;
/// 内核当前识别并保留的全部合法 `sa_flags` 位掩码集合。
const ACTION_FLAGS: usize =
    SA_NOCLDSTOP | SA_NOCLDWAIT | SA_SIGINFO | SA_ONSTACK | SA_RESTART | SA_NODEFER | SA_RESETHAND;

/// 所有会导致线程组暂停运行的停止类信号位掩码（`SIGSTOP`、`SIGTSTP`、`SIGTTIN`、`SIGTTOU`）。
const STOP_MASK: u64 =
    (1 << (SIGSTOP - 1)) | (1 << (SIGTSTP - 1)) | (1 << (SIGTTIN - 1)) | (1 << (SIGTTOU - 1));

/// 单个信号在内核中保存的处置配置（RISC-V 64 与 LoongArch 64 均通过内核映射的跳板页返回，UAPI 无 `sa_restorer` 字段）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SignalAction {
    /// 用户信号处理函数虚拟地址，或特殊常量 `SIG_DFL`（0）/ `SIG_IGN`（1）。
    pub(crate) handler: usize,
    /// `SA_*` 行为修饰标志位组合。
    pub(crate) flags: usize,
    /// 执行该信号处理函数期间额外并入线程阻塞掩码的信号集。
    pub(crate) mask: u64,
}

impl SignalAction {
    /// 初始默认信号处置（`SIG_DFL`，无附加标志与额外阻塞集）。
    pub(crate) const DEFAULT: Self = Self {
        handler: 0,
        flags: 0,
        mask: 0,
    };
}

/// 信号子系统操作错误枚举，由系统调用入口转换为对应 errno。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SignalError {
    /// 信号编号越界（为 0 或超过 64），或尝试修改 `SIGKILL` / `SIGSTOP` 的处置（`EINVAL`）。
    InvalidSignal,
    /// `sigaltstack` 传入了未定义的标志位或地址与大小相加溢出（`EINVAL`）。
    InvalidStack,
    /// 当前线程正在备用信号栈上运行，禁止在此期间切换或禁用备用信号栈（`EPERM`）。
    OnStack,
    /// 启用的备用信号栈容量小于架构要求的 `SIGNAL_MIN_STACK`（`ENOMEM`）。
    StackTooSmall,
    /// 目标用户的待处理排队信号数量已达到 `RLIMIT_SIGPENDING` 软限制（`EAGAIN`）。
    QueueFull,
    /// 为排队信号节点分配内核堆内存失败（`ENOMEM`）。
    NoMemory,
    /// 目标进程或线程已不存在或已彻底回收（`ESRCH`）。
    NoProcess,
    /// 发送方缺乏向目标进程发送信号的权限（`EPERM`）。
    Permission,
}

/// 随 `SignalInfo` 携带的分类载荷，对应 `siginfo_t::_sifields` 联合体的不同分支。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SignalPayload {
    /// 由进程/线程或内核通用事件发送的信号载荷（对应 `_kill` / `_rt` 分支）。
    Sender {
        /// 发送方进程号（PID）；内核直接生成时为 0。
        pid: usize,
        /// 发送方真实用户 ID（UID）；内核直接生成时为 0。
        uid: u32,
        /// 实时信号附带的 `sigval_t` 整数/指针值（普通 `kill` 发送时为 0）。
        value: usize,
    },
    /// 硬件同步异常产生的故障载荷（对应 `_sigfault` 分支）。
    Fault {
        /// 触发异常的故障虚拟地址（对于非法指令、断点或浮点异常则为故障指令 PC）。
        address: usize,
        /// 触发异常时的用户态程序计数器（PC）。
        pc: usize,
        /// 架构原始异常编码或访存类型标识。
        trap: usize,
    },
    /// 子进程状态变更通知载荷（对应 `SIGCHLD` 的 `_sigchld` 分支）。
    Child {
        /// 状态发生变更的子进程 PID。
        pid: usize,
        /// 子进程的真实用户 ID（UID）。
        uid: u32,
        /// 子进程退出码或触发状态变更的信号编号。
        status: i32,
        /// 子进程累计消耗的用户态时钟滴答数（按 `USER_HZ` 折算）。
        user_ticks: u64,
        /// 子进程累计消耗的内核态时钟滴答数（按 `USER_HZ` 折算）。
        kernel_ticks: u64,
    },
}

/// 内核内部传递与排队的完整信号实例描述。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SignalInfo {
    /// 信号编号（`1..=64`）。
    pub(crate) signum: u8,
    /// 关联的错误码 `si_errno`（通常为 0）。
    pub(crate) errno: i32,
    /// 标识信号来源原因的 `si_code`（如 `SI_KERNEL`、`SI_TKILL`、`SEGV_MAPERR`、`CLD_EXITED`）。
    pub(crate) code: i32,
    /// 与信号来源对应的附加字段载荷。
    pub(crate) payload: SignalPayload,
}

impl SignalInfo {
    /// 构造由内核直接产生的普通异步信号（`si_code = SI_KERNEL`，发送者 PID/UID 均为 0）。
    pub(crate) fn kernel(signum: u8) -> Self {
        Self {
            signum,
            errno: 0,
            code: SI_KERNEL,
            payload: SignalPayload::Sender {
                pid: 0,
                uid: 0,
                value: 0,
            },
        }
    }

    /// 将信号信息按 64 位 Linux UAPI 布局序列化为 128 字节小端字节数组，所有对齐空洞与未使用联合体字节均清零。
    pub(crate) fn encode(&self, bytes: &mut [u8]) {
        assert_eq!(bytes.len(), SIGINFO_BYTES);
        bytes.fill(0);
        bytes[0..4].copy_from_slice(&(self.signum as i32).to_le_bytes());
        bytes[4..8].copy_from_slice(&self.errno.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.code.to_le_bytes());

        match self.payload {
            SignalPayload::Sender { pid, uid, value } => {
                bytes[16..20].copy_from_slice(&(pid as i32).to_le_bytes());
                bytes[20..24].copy_from_slice(&uid.to_le_bytes());
                bytes[24..32].copy_from_slice(&value.to_le_bytes());
            }
            SignalPayload::Fault { address, .. } => {
                bytes[16..24].copy_from_slice(&address.to_le_bytes())
            }
            SignalPayload::Child {
                pid,
                uid,
                status,
                user_ticks,
                kernel_ticks,
            } => {
                bytes[16..20].copy_from_slice(&(pid as i32).to_le_bytes());
                bytes[20..24].copy_from_slice(&uid.to_le_bytes());
                bytes[24..28].copy_from_slice(&status.to_le_bytes());
                bytes[32..40].copy_from_slice(&user_ticks.to_le_bytes());
                bytes[40..48].copy_from_slice(&kernel_ticks.to_le_bytes());
            }
        }
    }
}

/// 用户态备用信号栈（`stack_t`）配置状态。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct AltStack {
    /// 备用栈内存区域的起始（低地址）虚拟地址 `ss_sp`。
    pub(crate) base: usize,
    /// 备用栈的字节大小 `ss_size`。
    pub(crate) size: usize,
    /// 备用栈配置标志位（`0` 或 `SS_DISABLE`；动态 `SS_ONSTACK` 在查询时根据实时 SP 计算）。
    pub(crate) flags: u32,
}

impl AltStack {
    /// 默认禁用状态的备用信号栈常量。
    pub(crate) const DISABLED: Self = Self {
        base: 0,
        size: 0,
        flags: SS_DISABLE,
    };

    /// 判断备用信号栈当前是否处于启用状态（仅依据 `SS_DISABLE` 标志位判定）。
    pub(crate) fn enabled(&self) -> bool {
        self.flags & SS_DISABLE == 0
    }

    /// 判断给定的用户栈指针 `sp` 是否正落在当前已启用的向下生长备用栈区间 `(base, base + size]` 内（包含初始栈顶 `base + size`，排除已彻底耗尽越界的 `base`）。
    pub(crate) fn on_stack(&self, sp: usize) -> bool {
        self.enabled()
            && self.base < sp
            && self
                .base
                .checked_add(self.size)
                .is_some_and(|end| sp <= end)
    }

    /// 生成 `sigaltstack` 查询或压入信号栈帧时对外报告的 `AltStack` 快照，根据当前 `sp` 实时计算 `SS_ONSTACK` 标志。
    pub(crate) fn reported(&self, sp: usize) -> Self {
        Self {
            flags: if !self.enabled() {
                SS_DISABLE
            } else if self.on_stack(sp) {
                SS_ONSTACK
            } else {
                0
            },
            ..*self
        }
    }

    /// 校验 `stack_t` 标志位合法性及启用状态下的 `[base, base + size)` 地址加法不溢出。
    pub(crate) fn validate(&self) -> Result<(), SignalError> {
        if !matches!(self.flags, 0 | SS_ONSTACK | SS_DISABLE)
            || (self.enabled() && self.base.checked_add(self.size).is_none())
        {
            return Err(SignalError::InvalidStack);
        }
        Ok(())
    }
}

/// 在用户栈上构造信号处理栈帧（`rt_sigframe`）所需的输入参数包。
pub(crate) struct SignalFrameArgs {
    /// 本次递送的信号详情（用于写入信号栈帧中的 `siginfo_t` 及设置第一参数 `a0`）。
    pub(crate) info: SignalInfo,
    /// 本次选中的用户信号处理函数配置（入口地址与 `SA_*` 标志）。
    pub(crate) action: SignalAction,
    /// 进入信号处理函数前保存的原线程信号阻塞掩码，将在 `rt_sigreturn` 时恢复。
    pub(crate) restore_mask: u64,
    /// 建立信号栈帧前保存的原线程备用信号栈配置，写入 `ucontext.uc_stack`。
    pub(crate) altstack: AltStack,
    /// 用户态信号处理函数返回时跳转执行的 `rt_sigreturn` 跳板虚拟地址。
    pub(crate) trampoline: usize,
}

/// 从用户栈 `rt_sigframe` 解码恢复出的完整现场与信号状态（供 `rt_sigreturn` 使用）。
pub(crate) struct SignalFrameRestore {
    /// 恢复后的用户态通用与浮点寄存器现场。
    pub(crate) context: UserContext,
    /// 待恢复的线程信号阻塞掩码。
    pub(crate) mask: u64,
    /// 信号栈帧 `uc_stack` 中携带的备用栈配置。
    pub(crate) altstack: AltStack,
    /// 架构帧解码时若检测到异常需立即补发的附加信号（通常为 `None`）。
    pub(crate) pending_signal: Option<SignalInfo>,
}

/// `clone` 创建子进程或新线程时从父线程继承的信号初始属性（待处理信号队列始终置空不继承）。
#[derive(Clone, Copy)]
pub(crate) struct SignalInheritance {
    /// 继承的初始信号阻塞掩码。
    pub(crate) mask: u64,
    /// 继承的初始备用信号栈配置（共享地址空间的线程创建时会被重置为 `DISABLED`）。
    pub(crate) altstack: AltStack,
}

impl SignalInheritance {
    /// 内核初始进程使用的全空信号继承属性（无阻塞信号，禁用备用栈）。
    pub(crate) const EMPTY: Self = Self {
        mask: 0,
        altstack: AltStack::DISABLED,
    };
}

/// 可在多个线程或 `CLONE_SIGHAND` 进程间共享的信号处置动作表。
pub(crate) struct Sighand {
    /// 受自旋锁（`siglock`）保护的 64 项信号处理配置数组（索引为 `signum - 1`）。
    actions: SpinLock<[SignalAction; SIGNAL_COUNT]>,
    /// 当前绑定并正在使用该动作表的活跃用户线程数（不含临时快照或 `SignalGroup` 自身的引用），用于 `execve` 判断是否需要写时复制解绑。
    users: AtomicUsize,
}

impl Sighand {
    /// 分配并初始化全部信号均为 `SIG_DFL` 的新信号处置表（初始活跃用户数为 0）。
    pub(crate) fn try_new() -> Result<Arc<Self>, AllocError> {
        Arc::try_new(Self {
            actions: SpinLock::new([SignalAction::DEFAULT; SIGNAL_COUNT]),
            users: AtomicUsize::new(0),
        })
    }

    /// 为非 `CLONE_SIGHAND` 的 `fork` 或共享状态下的 `execve` 复制当前信号处置表快照（先拷贝栈上数组再在锁外分配 `Arc`）。
    pub(crate) fn try_copy(&self) -> Result<Arc<Self>, AllocError> {
        let actions = *self.actions.lock();
        Arc::try_new(Self {
            actions: SpinLock::new(actions),
            users: AtomicUsize::new(0),
        })
    }

    /// 当新用户线程绑定运行时资源时，递增活跃执行者使用计数。
    pub(crate) fn add_user(&self) {
        self.users
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .expect("sighand user count overflow");
    }

    /// 当用户线程解绑运行时资源时，递减活跃执行者使用计数。
    pub(crate) fn remove_user(&self) {
        let previous = self.users.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "sighand user count underflow");
    }

    /// 读取当前绑定该动作表的活跃用户线程数量。
    pub(crate) fn user_count(&self) -> usize {
        self.users.load(Ordering::Acquire)
    }
}

/// 受 `SignalGroup::state` 自旋锁保护的线程组共享信号与作业控制状态。
struct GroupState {
    /// 发送给整个进程（线程组）的共享待处理信号队列。
    pending: Pending,
    /// 当前线程组中已发布且尚未退出的活跃线程成员总数。
    members: usize,
    /// 当前已在安全点确认进入作业停止（Stopped）状态的线程成员数。
    stopped: usize,
    /// 当前生效的组停止信号编号；收到 `SIGCONT` 或触发致命退出时清零。
    stop_signal: Option<u8>,
    /// 当前轮次的全体成员停止事件是否已生成并通知过父进程（防止重复报告 `CLD_STOPPED`）。
    stop_reported: bool,
    /// 线程组首个确立的致命退出状态码（一旦设置，全体成员在安全点统一退出）。
    exit: Option<i32>,
}

/// 单个进程（线程组）独占的信号控制中心，管理组级待处理队列、作业停止状态及当前绑定的 `Sighand`。
pub(crate) struct SignalGroup {
    /// 指向所属全局进程关系图的弱引用，用于在信号锁外发起跨成员调度唤醒与等待队列通知。
    graph: Weak<TaskGraph>,
    /// 本线程组的主线程号（TGID / PID）。
    tgid: usize,
    /// 是否为系统的初始根进程（PID 1，`init` 进程默认忽略未安装用户处理函数的普通致命信号以及 `SIGKILL`/`SIGSTOP`）。
    system_init: bool,
    /// 当前线程组绑定的信号处置表句柄；外层绑定自旋锁串行化 `execve` 期间的动作表替换。
    sighand: SpinLock<Arc<Sighand>>,
    /// 线程组共享待处理信号队列与停止/退出控制状态。
    state: SpinLock<GroupState>,
}

/// 线程在安全点向线程组确认作业控制状态后的裁决指令。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SignalControl {
    /// 无需停止或退出，正常继续执行。
    Run,
    /// 线程组处于暂停状态，调用方应挂入控制等待队列睡眠直至收到 `SIGCONT` 或致命退出。
    Stop,
    /// 线程组或本线程已被要求退出，携带终止状态码。
    Exit(i32),
}

/// 用户线程返回用户态前从 `ThreadSignals::select` 领取的单步信号处置动作。
pub(crate) enum SignalSelection {
    /// 捕获到安装了用户态处理函数的信号，需要在用户栈上压入信号帧并跳转到 `action.handler`。
    Handler {
        /// 被选中的信号实例详情。
        info: SignalInfo,
        /// 快照保存的用户态处理函数与标志位。
        action: SignalAction,
        /// 处理函数执行前需要保存在信号栈帧中的原线程阻塞掩码。
        restore_mask: u64,
        /// 建立信号栈帧时参考的当前备用栈配置。
        altstack: AltStack,
    },
    /// 领到了默认停止信号或组内已有生效的停止状态，线程应回到控制等待点进入暂停。
    Stop,
    /// 领到了默认致命信号或组/线程退出请求，线程应立即终止并携带该状态码退出。
    Exit(i32),
}

impl SignalGroup {
    /// 在新进程发布前创建线程组信号控制块，固定 `system_init` 身份并绑定初始信号处置表。
    pub(crate) fn try_new(
        graph: Weak<TaskGraph>,
        tgid: usize,
        sighand: Arc<Sighand>,
        system_init: bool,
    ) -> Result<Arc<Self>, AllocError> {
        Arc::try_new(Self {
            graph,
            tgid,
            system_init,
            sighand: SpinLock::new(sighand),
            state: SpinLock::new(GroupState {
                pending: Pending::new(),
                members: 0,
                stopped: 0,
                stop_signal: None,
                stop_reported: false,
                exit: None,
            }),
        })
    }

    /// 返回本线程组是否为系统初始根进程（`init`，PID 1）。
    pub(crate) fn is_system_init(&self) -> bool {
        self.system_init
    }

    /// 供间隔定时器在锁外（包括硬中断到期回调）免堆分配发送 `SIGALRM`、`SIGVTALRM` 或 `SIGPROF`（调用方严禁已持有信号锁）。
    pub(crate) fn send_timer(&self, signum: u8) {
        assert!(matches!(signum, SIGALRM | SIGVTALRM | SIGPROF));
        self.send(SignalInfo::kernel(signum))
            .expect("ordinary timer signal failed");
    }

    /// 向本线程组发送一个内核信号实例：升级 `TaskGraph` 后在关系锁与信号锁内完成入队，并在锁外触发成员唤醒。
    pub(crate) fn send(&self, info: SignalInfo) -> Result<(), SignalError> {
        signal_bit(info.signum)?;
        let graph = self.graph.upgrade().ok_or(SignalError::NoProcess)?;
        graph.send_group_info(self.tgid, self, info)
    }

    /// 在全部信号锁和关系锁外回收已丢弃的排队节点，并委托 `TaskGraph` 唤醒组内可处理当前待处理信号或组控制的成员。
    pub(crate) fn notify(&self) {
        self.reclaim_pending();
        if let Some(graph) = self.graph.upgrade() {
            graph.notify_group(self.tgid, self);
        }
    }

    /// 提取并在调用点（必须位于 `TaskGraph` 锁和全部信号锁外）释放组队列中已丢弃的堆节点。
    pub(super) fn reclaim_pending(&self) {
        let retired = self.state.lock().pending.retired();
        drop(retired);
    }

    /// 在新线程加入线程组发布事务时递增活跃成员计数，并重置本轮全体停止报告标志。
    pub(super) fn add_member(&self) {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut group = self.state.lock();
        group.members = group
            .members
            .checked_add(1)
            .expect("signal member overflow");
        group.stop_reported = false;
    }

    /// 在线程退出移出线程组时清空其私有待处理队列、递减组活跃成员计数；若剩余全体成员恰好均已确认停止，则返回停止信号编号以通知父进程。
    pub(super) fn remove_member(&self, thread: &ThreadSignals) -> Option<u8> {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut group = self.state.lock();
        let mut private = thread.state.lock();
        private.pending.discard(u64::MAX);
        group.members = group
            .members
            .checked_sub(1)
            .expect("signal member underflow");
        if group.members == 0 {
            group.pending.discard(u64::MAX);
        }
        if private.stopped {
            private.stopped = false;
            group.stopped = group.stopped.checked_sub(1).expect("stop count underflow");
        }
        stopped_event(&mut group)
    }

    /// 在已持有 `TaskGraph` 锁的前提下，获取动作表锁执行信号递送事务（只修改队列与控制状态，不直接执行调度唤醒）。返回是否使已报告停止的作业恢复了继续运行（`CLD_CONTINUED`）。
    pub(super) fn deliver<'a>(
        &self,
        info: SignalInfo,
        target: Option<&ThreadSignals>,
        prepared: &mut PreparedSignal,
        account: Option<&ThreadGroup>,
        threads: impl Iterator<Item = &'a Arc<ThreadSignals>>,
    ) -> Result<bool, SignalError> {
        let bit = signal_bit(info.signum)?;
        let binding = self.sighand.lock();
        let actions = binding.actions.lock();
        self.deliver_locked(
            info,
            bit,
            actions[info.signum as usize - 1],
            target,
            prepared,
            account,
            threads,
        )
    }

    /// 在已持有 `Sighand` 绑定锁与 `actions` 锁的内部递送核心：处理停止/继续信号互消、扫描成员阻塞状态、入队信号以及确立默认致命退出。
    #[expect(
        clippy::too_many_arguments,
        reason = "同一动作锁下分别传递信号、目标、预备队列和成员迭代器，保持递送事务及锁归属可见"
    )]
    fn deliver_locked<'a>(
        &self,
        info: SignalInfo,
        bit: u64,
        action: SignalAction,
        target: Option<&ThreadSignals>,
        prepared: &mut PreparedSignal,
        account: Option<&ThreadGroup>,
        threads: impl Iterator<Item = &'a Arc<ThreadSignals>>,
    ) -> Result<bool, SignalError> {
        // Step 1: PID 1 系统初始进程免疫外部发送的 SIGKILL 与 SIGSTOP；空组或已进入致命退出的组（除 SIGCONT 外）不再接收新信号。
        if self.system_init && matches!(info.signum, SIGKILL | SIGSTOP) {
            return Ok(false);
        }
        let mut group = self.state.lock();
        if group.members == 0 || (group.exit.is_some() && info.signum != SIGCONT) {
            return Ok(false);
        }

        // Step 2: 处理 SIGCONT 与停止信号（SIGSTOP/SIGTSTP/SIGTTIN/SIGTTOU）之间的双向抵消规则。
        let continued = info.signum == SIGCONT && group.stop_reported;
        let discard = match info.signum {
            SIGCONT => STOP_MASK,
            SIGSTOP..=SIGTTOU => 1 << (SIGCONT - 1),
            _ => 0,
        };
        group.pending.discard(discard);
        if info.signum == SIGCONT {
            group.stop_signal = None;
            group.stop_reported = false;
            group.stopped = 0;
        }

        // Step 3: 遍历组内线程，同步清除被抵消的线程私有信号，并统计目标范围内是否存在阻塞或未阻塞该信号的成员。
        let mut blocked = false;
        let mut eligible = false;
        for thread in threads {
            let mut private = thread.state.lock();
            private.pending.discard(discard);
            if info.signum == SIGCONT {
                private.stopped = false;
            }
            if target.is_none()
                || target.is_some_and(|target| core::ptr::eq(target, thread.as_ref()))
            {
                blocked |= (private.mask | private.real_blocked.unwrap_or(0)) & bit != 0;
                eligible |= (private.mask | private.real_blocked.unwrap_or(0)) & bit == 0;
            }
        }

        // Step 4: 若线程组已在退出途中，仅保留上方已完成的 SIGCONT 唤醒清理并返回；否则将有效信号压入目标队列并判定是否立即确立组级致命退出。
        if group.exit.is_some() {
            return Ok(continued);
        }
        if info.signum == SIGKILL {
            group.exit.get_or_insert(SIGKILL as i32);
            group.stop_signal = None;
        } else if blocked || !ignored(action, info.signum, self.system_init) {
            match target {
                Some(thread) => thread.state.lock().pending.push(info, prepared, account)?,
                None => group.pending.push(info, prepared, account)?,
            }
            if eligible
                && action.handler == SIG_DFL
                && !ignored(action, info.signum, self.system_init)
                && !matches!(info.signum, SIGSTOP..=SIGTTOU)
            {
                group.exit.get_or_insert(info.signum as i32);
                group.stop_signal = None;
            }
        }
        Ok(continued)
    }

    /// 在子进程退出时（持有 `TaskGraph` 锁）原子检查父进程的 `SIGCHLD` 处置：返回是否需要自动回收子进程（`SIG_IGN` 或 `SA_NOCLDWAIT`），并在未显式忽略时向父进程入队 `SIGCHLD`。
    pub(super) fn deliver_child_exit<'a>(
        &self,
        info: SignalInfo,
        threads: impl Iterator<Item = &'a Arc<ThreadSignals>>,
    ) -> bool {
        assert!(info.signum == SIGCHLD && matches!(info.code, CLD_EXITED..=CLD_DUMPED));
        let binding = self.sighand.lock();
        let actions = binding.actions.lock();
        let action = actions[SIGCHLD as usize - 1];
        let autoreap = action.handler == SIG_IGN || action.flags & SA_NOCLDWAIT != 0;
        if action.handler != SIG_IGN {
            match self.deliver_locked(
                info,
                1 << (SIGCHLD - 1),
                action,
                None,
                &mut PreparedSignal::kernel(),
                None,
                threads,
            ) {
                Ok(_) => {}
                Err(error) => panic!("ordinary child signal failed: {:?}", error),
            }
        }
        autoreap
    }

    /// 查询或替换指定信号的 `SignalAction`（对应 `rt_sigaction`）：若新处置使信号在本组变为忽略状态，则在同一把锁下立即丢弃本组及全体成员中已排队的该信号。
    pub(super) fn update_action<'a>(
        &self,
        signal: usize,
        action: Option<SignalAction>,
        threads: impl Iterator<Item = &'a Arc<ThreadSignals>>,
    ) -> Result<SignalAction, SignalError> {
        if signal == 0
            || SIGNAL_COUNT < signal
            || (action.is_some() && (signal == SIGKILL as usize || signal == SIGSTOP as usize))
        {
            return Err(SignalError::InvalidSignal);
        }

        let binding = self.sighand.lock();
        let mut actions = binding.actions.lock();
        let previous = actions[signal - 1];
        if let Some(mut action) = action {
            action.flags &= ACTION_FLAGS;
            action.mask &= !UNBLOCKABLE;
            actions[signal - 1] = action;
            if ignored(action, signal as u8, self.system_init) {
                let discard = 1 << (signal - 1);
                self.state.lock().pending.discard(discard);
                for thread in threads {
                    thread.state.lock().pending.discard(discard);
                }
            }
        }
        Ok(previous)
    }

    /// 在持有外层关系锁时设置线程组致命退出码并清除待定的停止状态（仅标记状态，不在关系锁内执行唤醒）。
    pub(super) fn mark_exit(&self, status: i32) {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut group = self.state.lock();
        group.exit.get_or_insert(status);
        group.stop_signal = None;
    }

    /// 查询当前线程组是否已被标记为致命退出，若已标记则返回终止状态码。
    pub(crate) fn exit_request(&self) -> Option<i32> {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let exit = self.state.lock().exit;
        exit
    }

    /// 判断本线程组全体活跃成员是否均已确认进入作业停止状态（供 `wait4(..., WUNTRACED)` 查询）。
    pub(crate) fn has_stopped_jobs(&self) -> bool {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let stopped = self.state.lock().stop_reported;
        stopped
    }

    /// 向父进程发送子进程停止/继续通知信号：若父进程对 `SIGCHLD` 设置了 `SA_NOCLDSTOP` 则直接抑制该通知。
    pub(crate) fn send_child(&self, info: SignalInfo) {
        let suppress = {
            let binding = self.sighand.lock();
            let actions = binding.actions.lock();
            matches!(info.code, CLD_STOPPED | CLD_CONTINUED)
                && actions[SIGCHLD as usize - 1].flags & SA_NOCLDSTOP != 0
        };
        if !suppress {
            match self.send(info) {
                Ok(()) | Err(SignalError::NoProcess) => {}
                Err(error) => panic!("ordinary child signal failed: {:?}", error),
            }
        }
    }
}

/// 受 `ThreadSignals::state` 自旋锁保护的线程私有信号与等待状态。
struct ThreadState {
    /// 专门发往本线程（如 `tkill`、`tgkill`）的私有待处理信号队列。
    pending: Pending,
    /// 当前线程触发的同步硬件异常信号槽位；出队优先级高于普通异步队列。
    fault: Option<SignalInfo>,
    /// 当前生效的线程信号阻塞位掩码（第 `signum - 1` 位为 1 表示阻塞）。
    mask: u64,
    /// 执行 `rt_sigtimedwait` 临时放开等待信号期间备份的真实原阻塞掩码。
    real_blocked: Option<u64>,
    /// 当前 `rt_sigtimedwait` 正在等待捕获的信号位掩码集合。
    waited_set: u64,
    /// 本线程配置的备用信号栈（`sigaltstack`）。
    altstack: AltStack,
    /// 本线程是否已在安全点确认响应了当前线程组的停止请求。
    stopped: bool,
    /// 同组其他线程发起多线程 `execve`（de-thread）时请求本线程立即退出的私有标志。
    exec_exit: bool,
    /// 本线程当前处于 `Interruptible` 或 `Killable` 睡眠时注册的唯一唤醒器与等待模式。
    wait: Option<(WaitId, Waker, WaitMode)>,
}

/// 单个线程持有的信号控制块，绑定所属线程组并封装线程私有信号状态。
pub(crate) struct ThreadSignals {
    /// 所属线程组的共享信号控制中心。
    group: Arc<SignalGroup>,
    /// 线程私有待处理信号、阻塞掩码、备用栈与睡眠等待登记状态。
    state: SpinLock<ThreadState>,
}

impl ThreadSignals {
    /// 在新线程发布为可运行状态之前，于锁外分配并初始化其私有信号控制块。
    pub(crate) fn try_new(
        group: Arc<SignalGroup>,
        inherited: SignalInheritance,
    ) -> Result<Arc<Self>, AllocError> {
        Arc::try_new(Self {
            group,
            state: SpinLock::new(ThreadState {
                pending: Pending::new(),
                fault: None,
                mask: inherited.mask & !UNBLOCKABLE,
                real_blocked: None,
                waited_set: 0,
                altstack: inherited.altstack,
                stopped: false,
                exec_exit: false,
                wait: None,
            }),
        })
    }

    /// 为 `clone` 派生子任务快照当前线程的继承属性：始终复制信号阻塞掩码；若共享虚拟地址空间（`CLONE_VM` 且非 `vfork`）则将子线程备用栈置为禁用以避免双线程共用同一备用栈踩内存。
    pub(crate) fn inherit(&self, share_vm: bool) -> SignalInheritance {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let private = self.state.lock();
        SignalInheritance {
            mask: private.mask,
            altstack: if share_vm {
                AltStack::DISABLED
            } else {
                private.altstack
            },
        }
    }

    /// 注入由用户态同步硬件异常（缺页失败、非法指令、未对齐访问、断点、浮点异常等）产生的强制信号。
    pub(crate) fn force_fault(&self, info: SignalInfo) {
        assert!(matches!(
            info.signum,
            SIGILL | SIGTRAP | SIGBUS | SIGFPE | SIGSEGV | SIGSYS
        ));
        assert!(matches!(info.payload, SignalPayload::Fault { .. }));
        self.force(info);
    }

    /// 强制向本线程递送高优先级同步信号：若该信号当前被线程阻塞或设为 `SIG_IGN`，则自动重置为 `SIG_DFL` 并解除该位阻塞，防止返回用户态后无限重入同一条故障指令。
    pub(crate) fn force(&self, info: SignalInfo) {
        signal_bit(info.signum).expect("invalid forced signal");
        {
            let binding = self.group.sighand.lock();
            let mut actions = binding.actions.lock();
            let mut private = self.state.lock();
            let bit = 1 << (info.signum - 1);
            let action = &mut actions[info.signum as usize - 1];
            if private.mask & bit != 0 || action.handler == SIG_IGN {
                *action = SignalAction::DEFAULT;
            }
            private.mask &= !bit;
            private.fault.get_or_insert(info);
        }

        self.notify_wait();
    }

    /// 返回当前被本线程阻塞且正在等待处理的信号位掩码（合并线程私有队列与线程组共享队列快照，供 `rt_sigpending` 使用）。
    pub(crate) fn pending(&self) -> u64 {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let group = self.group.state.lock();
        let private = self.state.lock();
        (private.pending.bits | group.pending.bits) & private.mask
    }

    /// 替换本线程的信号阻塞掩码（强制剥离不可阻塞的 `SIGKILL` 与 `SIGSTOP`），并在锁外通知线程组重新评估是否有新解除阻塞的信号可供消费。
    pub(crate) fn replace_mask(&self, mask: u64) -> u64 {
        let old = {
            let binding = self.group.sighand.lock();
            let _siglock = binding.actions.lock();
            let mut private = self.state.lock();
            let old = private.mask;
            private.mask = mask & !UNBLOCKABLE;
            old
        };

        self.group.notify();
        old
    }

    /// 在信号锁保护下读取当前线程的信号阻塞掩码（供 `rt_sigsuspend` 等保存旧掩码使用）。
    pub(crate) fn mask(&self) -> u64 {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let mask = self.state.lock().mask;
        mask
    }

    /// 原子判断指定信号对于当前线程是否处于被阻塞或忽略状态（供终端驱动对后台读写判断是否跳过发送 `SIGTTIN`/`SIGTTOU` 等场景使用）。
    pub(crate) fn blocked_or_ignored(&self, signum: u8) -> bool {
        let bit = signal_bit(signum).expect("invalid terminal signal");
        let binding = self.group.sighand.lock();
        let actions = binding.actions.lock();
        let private = self.state.lock();
        private.mask & bit != 0
            || ignored(actions[signum as usize - 1], signum, self.group.system_init)
    }

    /// 进入 `rt_sigtimedwait` 等待前临时解除 `set` 中信号的阻塞，并备份真实阻塞掩码 `real_blocked`（保证即使目标信号处置为忽略或默认致命也能入队被等待者取走）。
    pub(crate) fn begin_waited(&self, set: u64) -> u64 {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut private = self.state.lock();
        assert!(private.real_blocked.is_none(), "nested timed signal wait");
        let old = private.mask;
        private.real_blocked = Some(old);
        private.waited_set = set & !UNBLOCKABLE;
        private.mask = old & !private.waited_set;
        old
    }

    /// 退出 `rt_sigtimedwait` 等待后恢复原先备份的真实阻塞掩码，并在锁外通知组内其他成员接手剩余未阻塞信号。
    pub(crate) fn end_waited(&self, old: u64) {
        {
            let binding = self.group.sighand.lock();
            let _siglock = binding.actions.lock();
            let mut private = self.state.lock();
            assert_eq!(private.real_blocked.take(), Some(old));
            private.mask = old & !UNBLOCKABLE;
            private.waited_set = 0;
        }

        self.group.notify();
    }

    /// 查询当前备用信号栈状态（实时附带 `SS_ONSTACK`），并在传入 `replacement` 时校验当前未处于备用栈上后更新配置。
    pub(crate) fn altstack(
        &self,
        sp: usize,
        replacement: Option<AltStack>,
    ) -> Result<AltStack, SignalError> {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut private = self.state.lock();
        let previous = private.altstack.reported(sp);
        if let Some(stack) = replacement {
            set_altstack(&mut private, stack, sp)?;
        }
        Ok(previous)
    }

    /// 在 `rt_sigreturn` 完成用户栈帧解码后恢复信号阻塞掩码，并尽力恢复帧中保存的备用栈配置（遵循 Linux `rt_sigreturn` 语义忽略非法备用栈恢复错误）。
    pub(crate) fn restore(&self, mask: u64, altstack: AltStack, restored_sp: usize) {
        {
            let binding = self.group.sighand.lock();
            let _siglock = binding.actions.lock();
            let mut private = self.state.lock();
            private.mask = mask & !UNBLOCKABLE;
            let _ = set_altstack(&mut private, altstack, restored_sp);
        }

        self.group.notify();
    }

    /// 为 `rt_sigtimedwait` 从线程私有队列或组共享队列中弹出一个属于 `set` 的待处理信号实例。
    pub(crate) fn take_waited(&self, set: u64) -> Option<SignalInfo> {
        let mut retired = Retired::new();
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut group = self.group.state.lock();
        let mut private = self.state.lock();
        let allowed = set & !UNBLOCKABLE;
        private
            .pending
            .take(allowed)
            .or_else(|| group.pending.take(allowed))
            .map(|removed| retired.add(removed))
    }

    /// 在返回用户态边界领取下一个需要处理的控制动作或非阻塞信号。
    ///
    /// 若命中用户自定义处理函数，则在同一临界区内完成 `SA_RESETHAND` 重置与执行期阻塞掩码更新。
    pub(crate) fn select(&self) -> Option<SignalSelection> {
        let mut retired = Retired::new();
        let binding = self.group.sighand.lock();
        let mut actions = binding.actions.lock();
        let mut group = self.group.state.lock();
        let mut private = self.state.lock();
        // Step 1: 优先检查线程组致命退出、多线程 execve 清场退出以及组停止请求。
        if let Some(status) = group.exit {
            return Some(SignalSelection::Exit(status));
        }
        if private.exec_exit {
            return Some(SignalSelection::Exit(0));
        }
        if group.stop_signal.is_some() {
            return Some(SignalSelection::Stop);
        }

        // Step 2: 循环弹出未阻塞信号（硬件同步异常槽位 fault 优先于私有队列，私有队列优先于组共享队列），跳过忽略项。
        loop {
            let allowed = !private.mask;
            let forced = private.fault.is_some();
            let info = if let Some(info) = private.fault.take() {
                info
            } else {
                let removed = private
                    .pending
                    .take(allowed)
                    .or_else(|| group.pending.take(allowed))?;
                retired.add(removed)
            };
            let mut action = actions[info.signum as usize - 1];
            if forced && action.handler == SIG_IGN {
                action = SignalAction::DEFAULT;
            }
            if ignored(action, info.signum, !forced && self.group.system_init) {
                continue;
            }
            if action.handler == SIG_DFL {
                if matches!(info.signum, SIGSTOP..=SIGTTOU) {
                    group.stop_signal = Some(info.signum);
                    return Some(SignalSelection::Stop);
                }
                let status = info.signum as i32;
                group.exit.get_or_insert(status);
                return Some(SignalSelection::Exit(status));
            }

            // Step 3: 命中用户态处理函数，应用 SA_RESETHAND 与执行期信号掩码（含 SA_NODEFER 判断）。
            if action.flags & SA_RESETHAND != 0 {
                actions[info.signum as usize - 1].handler = SIG_DFL;
            }
            let restore_mask = private.mask;
            private.mask |= action.mask;
            if action.flags & SA_NODEFER == 0 {
                private.mask |= 1 << (info.signum - 1);
            }
            private.mask &= !UNBLOCKABLE;
            return Some(SignalSelection::Handler {
                info,
                action,
                restore_mask,
                altstack: private.altstack,
            });
        }
    }

    /// 在用户态主循环顶部的安全点确认组级控制状态：若处于停止状态则登记本线程已暂停，并在最后一个活跃成员确认暂停时返回停止信号号以触发一次父进程 `CLD_STOPPED` 通知。
    pub(super) fn acknowledge_control(&self) -> (SignalControl, Option<u8>) {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut group = self.group.state.lock();
        let mut private = self.state.lock();
        if let Some(status) = group.exit {
            return (SignalControl::Exit(status), None);
        }
        if private.exec_exit {
            return (SignalControl::Exit(0), None);
        }
        if group.stop_signal.is_none() {
            return (SignalControl::Run, None);
        }
        if !private.stopped {
            private.stopped = true;
            group.stopped = group.stopped.checked_add(1).expect("stop count overflow");
        }
        (SignalControl::Stop, stopped_event(&mut group))
    }

    /// 为即将进入可中断或可杀死睡眠的 `Waiter` 注册唤醒句柄，随后立即在锁外执行一次复查通知以消除注册前的信号竞态窗口。
    pub(crate) fn register_wait(&self, id: WaitId, waker: Waker, mode: WaitMode) {
        {
            let binding = self.group.sighand.lock();
            let _siglock = binding.actions.lock();
            let mut private = self.state.lock();
            assert!(private.wait.is_none(), "nested signal wait");
            private.wait = Some((id, waker, mode));
        }

        self.notify_wait();
    }

    /// 撤销指定 `WaitId` 对应的睡眠等待登记，并将取出的 `Waker` 带到信号自旋锁外释放。
    pub(crate) fn unregister_wait(&self, id: WaitId) {
        let removed = {
            let binding = self.group.sighand.lock();
            let _siglock = binding.actions.lock();
            let mut private = self.state.lock();
            if private
                .wait
                .as_ref()
                .is_some_and(|(current, _, _)| *current == id)
            {
                private.wait.take()
            } else {
                None
            }
        };

        drop(removed);
    }

    /// 判断当前是否存在足以打断指定睡眠模式（`Killable` 仅响应致命退出，`Interruptible` 额外响应停止、同步故障及未阻塞非忽略信号）的事件。
    pub(crate) fn interruption_pending(&self, mode: WaitMode) -> bool {
        let binding = self.group.sighand.lock();
        let actions = binding.actions.lock();
        let group = self.group.state.lock();
        let private = self.state.lock();
        interruption(&actions, &group, &private, mode, self.group.system_init)
    }

    /// 回收线程私有队列的已丢弃节点，并在检测到当前注册的睡眠可被打断时克隆 `Waker` 于锁外唤醒任务。
    pub(crate) fn notify_wait(&self) {
        self.reclaim_pending();
        let waker = {
            let binding = self.group.sighand.lock();
            let actions = binding.actions.lock();
            let group = self.group.state.lock();
            let private = self.state.lock();
            private.wait.as_ref().and_then(|(_, waker, mode)| {
                interruption(&actions, &group, &private, *mode, self.group.system_init)
                    .then(|| waker.clone())
            })
        };

        if let Some(waker) = waker {
            waker.notify();
        }
    }

    /// 提取并在锁外释放线程私有队列中累积的已丢弃堆节点。
    pub(super) fn reclaim_pending(&self) {
        let retired = self.state.lock().pending.retired();
        drop(retired);
    }

    /// 在多线程进程执行 `execve` 时通知本副线程在安全点立即退出，并打断其正在进行的睡眠。
    pub(crate) fn request_exec_exit(&self) {
        {
            let binding = self.group.sighand.lock();
            let _siglock = binding.actions.lock();
            self.state.lock().exec_exit = true;
        }

        self.notify_wait();
    }

    /// 查询本线程是否收到了同组 `execve` 发起的副线程清场退出请求。
    pub(crate) fn exec_exit_requested(&self) -> bool {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let requested = self.state.lock().exec_exit;
        requested
    }

    /// 在进程已成为单线程且无活跃等待者后为 `execve` 重置信号状态：切换至私有动作表（将所有自定义处理函数重置为 `SIG_DFL`，仅保留 `SIG_IGN`），禁用备用信号栈，保留原阻塞掩码与待处理队列。
    pub(crate) fn reset_for_exec(&self, sighand: Arc<Sighand>) {
        let old = {
            let mut binding = self.group.sighand.lock();
            {
                let mut actions = sighand.actions.lock();
                let group = self.group.state.lock();
                let mut private = self.state.lock();
                assert_eq!(group.members, 1, "exec has live signal peers");
                assert!(private.wait.is_none(), "exec has an active signal waiter");
                for action in actions.iter_mut() {
                    *action = SignalAction {
                        handler: usize::from(action.handler == 1),
                        ..SignalAction::DEFAULT
                    };
                }
                if let Some(mask) = private.real_blocked.take() {
                    private.mask = mask;
                }
                private.waited_set = 0;
                private.altstack = AltStack::DISABLED;
            }
            mem::replace(&mut *binding, sighand)
        };

        drop(old);
    }
}

/// 校验信号编号是否在 `1..=64` 合法范围内，并返回对应的单比特掩码 `1 << (signum - 1)`。
fn signal_bit(signum: u8) -> Result<u64, SignalError> {
    if signum == 0 || SIGNAL_COUNT < signum as usize {
        return Err(SignalError::InvalidSignal);
    }
    Ok(1 << (signum - 1))
}

/// 判断给定信号在当前处置动作下是否应被直接忽略（包含 `SIG_IGN`、默认忽略的四类信号，以及 PID 1 初始进程对未捕获信号和 `SIGKILL`/`SIGSTOP` 的特殊免疫）。
fn ignored(action: SignalAction, signum: u8, system_init: bool) -> bool {
    (system_init && (action.handler == SIG_DFL || matches!(signum, SIGKILL | SIGSTOP)))
        || action.handler == SIG_IGN
        || (action.handler == SIG_DFL && matches!(signum, SIGCHLD | SIGCONT | SIGURG | SIGWINCH))
}

/// 当非空且未退出的线程组内所有活跃成员均已确认暂停、且本轮尚未向父进程报告过时，置位 `stop_reported` 并返回停止信号号。
fn stopped_event(group: &mut GroupState) -> Option<u8> {
    if 0 < group.members
        && group.stopped == group.members
        && !group.stop_reported
        && group.exit.is_none()
    {
        if let Some(signum) = group.stop_signal {
            group.stop_reported = true;
            return Some(signum);
        }
    }
    None
}

/// 在持有全部信号锁的条件下纯内存判定当前线程是否存在可打断 `mode` 睡眠模式的退出、停止或未忽略信号。
fn interruption(
    actions: &[SignalAction; SIGNAL_COUNT],
    group: &GroupState,
    private: &ThreadState,
    mode: WaitMode,
    system_init: bool,
) -> bool {
    if group.exit.is_some() || private.exec_exit {
        return true;
    }
    let pending = private.pending.bits | group.pending.bits;
    if !system_init && pending & (1 << (SIGKILL - 1)) != 0 {
        return true;
    }
    if mode == WaitMode::Killable {
        return false;
    }
    if pending & private.waited_set != 0 {
        return true;
    }
    if group.stop_signal.is_some() || private.fault.is_some() {
        return true;
    }

    let mut allowed = pending & !private.mask;
    while allowed != 0 {
        let index = allowed.trailing_zeros() as usize;
        if !ignored(actions[index], index as u8 + 1, system_init) {
            return true;
        }
        allowed &= allowed - 1;
    }
    false
}

/// 校验并更新线程的备用信号栈配置（`sigaltstack` 与 `rt_sigreturn` 共用）：禁止在正使用备用栈时修改，禁用时统一规整为 `AltStack::DISABLED`。
fn set_altstack(private: &mut ThreadState, stack: AltStack, sp: usize) -> Result<(), SignalError> {
    if private.altstack.on_stack(sp) {
        return Err(SignalError::OnStack);
    }
    stack.validate()?;
    if stack.enabled() && stack.size < crate::arch::SIGNAL_MIN_STACK {
        return Err(SignalError::StackTooSmall);
    }

    private.altstack = if stack.enabled() {
        stack
    } else {
        AltStack::DISABLED
    };
    Ok(())
}
