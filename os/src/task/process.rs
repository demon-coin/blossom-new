//! 进程/线程身份关系图（`TaskGraph`）、四类 PID 角色绑定、作业控制会话与信号路由。
//!
//! ## 模块职责与架构定位
//! 本模块维护用户态执行实体的全局身份拓扑（`TID`、`TGID`、`PGID`、`SID`）、父子亲缘关系、控制终端（Controlling TTY）绑定以及按身份寻址的信号投递。
//! - **上游调用者**：`clone`/`exec`/`user` 模块通过预备与发布两步流程（`prepare_process`/`prepare_thread` ➔ `PreparedIdentity::publish`）注册新进程与线程；系统调用层查询/修改进程组与会话（`getpgid`、`setpgid`、`getsid`、`setsid`）、等待子进程（`try_wait_event`）、管理资源限额（`prlimit`）并发送信号（`kill`、`tkill`、`tgkill`、`rt_sigqueueinfo`）；`crate::fs::tty` 调用控制终端绑定与前台作业检查接口；系统主流程调用 `request_shutdown` 与 `wait_root` 协调关机与回收。
//! - **下游依赖**：子模块 [`numbers`] 提供按需分页的槽位表与空闲位图，[`resources`] 管理 `rlimit` 与按 UID 聚合的配额，[`lifecycle`] 实现 `exec` 收拢、退出、过继与 `wait`；`crate::task::signal` 处理组与线程信号队列；`crate::task::time` 统计进程组 CPU 时间与间隔定时器；`crate::wait` 提供等待队列。
//!
//! ## 核心设计与引用拓扑
//! 1. **无强引用环的单向所有权与弱反向关联**：
//!    运行中的 `UserThread` 持有 `Arc<UserIdentity>`，进而强引用 `Arc<TaskGraph>` 与 `Arc<ThreadGroup>`；而 `TaskGraph` 槽位内的 `ThreadEntry` 仅持有 `Weak<UserThread>` 和轻量的内核栈完成观察器 `PreparedExecution`（`JoinObserver`）。当进程变为僵尸（`ProcessEntry::exit = Some(...)`）时，其地址空间 `Mm`、打开文件表与内核栈均不滞留在僵尸记录中。
//! 2. **数值与身份对象分离（`Arc<Pid>`）及四角色引用计数**：
//!    每个已发布的号码对应唯一的 `Arc<Pid>` 实例，槽位 `Slot::roles` 分别统计该 `Pid` 当前担任 `TID`、`TGID`、`PGID`、`SID` 四种角色的次数（而非依赖 `Arc` 强引用计数）。即使线程或进程已退出，只要该 `Pid` 仍作为某个存活进程的 `PGID` 或 `SID`，该号码就不会被空闲位图复用；同时外部保存的旧 `Arc<Pid>` 句柄通过指针相等性（`Arc::ptr_eq`）校验，绝不会在号码复用后误命中新创建的同名进程。
//!
//! ```text
//! UserThread (持有 Mm / FileTable / UserContext)
//!   │  ▲
//!   │  └────── Weak<UserThread> ───────────────────────┐
//!   ▼                                                  │
//! UserIdentity { graph, group, tid: Arc<Pid>, signals }│
//!   │          │                                       │
//!   │          ▼                                       │
//!   │     ThreadGroup { pid(TGID), relations(PGID/SID/ctty), uid, signals, time, limits, account }
//!   ▼                                                  │
//! TaskGraph { slots: SpinLock<Numbers>, closing, waiters, control_waiters }
//!   └─ Slot[pid - 1]
//!        ├─ reserved: bool                (锁外准备期间占坑)
//!        ├─ pid: Option<Arc<Pid>>         (当前占用该号码的身份实例)
//!        ├─ roles: [TID, TGID, PGID, SID] (四角色活跃计数，全 0 且 !reserved 时置位空闲位图)
//!        ├─ thread: Option<ThreadEntry>   (存活线程登记，含 Weak<UserThread> 与 ProcessCharge)
//!        ├─ process: Option<ProcessEntry> (存活线程组或待 wait 回收的僵尸进程记录)
//!        └─ completed: Option<Box<...>>   (已 wait 回收但尚未切离内核栈的 JoinObserver 链表)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 多锁协同必须严格遵守以下从外到内的获取顺序，严禁逆序加锁：
//! 1. TTY 睡眠复制锁（`tty::Endpoint::copy`，睡眠锁，仅在控制终端获取/释放路径由上层先持有）
//! 2. 任务图槽位锁 `TaskGraph::slots`（`SpinLock<Numbers>`）
//! 3. 线程身份锁 `UserIdentity::tid` 或线程组关系锁 `ThreadGroup::relations` / 限额锁 `ThreadGroup::limits`
//! 4. 终端控制自旋锁 `tty::Endpoint::control()` 或信号子系统锁（`Sighand` ➔ `SignalGroup` / `ThreadSignals`）
//!
//! 所有堆内存预分配（`Numbers::prepare_growth`、`PreparedSignal::user`、`Arc::try_new`）、阻塞等待（`Waiter::park`）、跨核唤醒（`UserThread::kick`、`WaitQueue::wake_all`）以及可能释放最后一个强引用的对象析构都必须在 `TaskGraph::slots` 自旋锁之外执行。

use self::numbers::Numbers;
pub(crate) use self::numbers::MAX_PID;
#[cfg(feature = "ktest")]
pub(crate) use self::resources::DEFAULT_NOFILE;
use self::resources::{Limits, ProcessCharge, UserAccount};
pub(crate) use self::resources::{Resource, ResourceLimit, SignalCharge};
use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};

mod lifecycle;
mod numbers;
mod resources;

use core::pin::pin;
use core::ptr;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::fs::{tty, FsError};
use crate::sched;
use crate::sync::SpinLock;
use crate::task::signal::{
    PreparedSignal, Sighand, SignalAction, SignalControl, SignalError, SignalGroup, SignalInfo,
    SignalInheritance, SignalPayload, ThreadSignals, CLD_CONTINUED, CLD_DUMPED, CLD_EXITED,
    CLD_KILLED, CLD_STOPPED, SIGCHLD, SIGCONT, SIGHUP, SIGNAL_COUNT, SIGTTIN, SIGTTOU, SI_TKILL,
};
use crate::task::time::{GroupTime, USER_HZ};
use crate::task::{JoinObserver, TaskId, UserThread};
use crate::time::NSEC_PER_SEC;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// `Slot::roles` 数组下标：线程标识符（Thread ID）角色计数。
const TID: usize = 0;
/// `Slot::roles` 数组下标：线程组标识符（Thread Group ID，即用户态可见的进程 `PID`）角色计数。
const TGID: usize = 1;
/// `Slot::roles` 数组下标：进程组标识符（Process Group ID）角色计数。
const PGID: usize = 2;
/// `Slot::roles` 数组下标：会话标识符（Session ID）角色计数。
const SID: usize = 3;

/// `wait4` / `waitid` 选项位：非阻塞轮询，若无满足条件的子进程事件则立即返回 `0`（不睡眠）。
pub(crate) const WNOHANG: u32 = 1;
/// `wait4` / `waitid` 选项位：关注因作业控制信号而进入停止态（Stopped）的子进程事件。
pub(crate) const WSTOPPED: u32 = 2;
/// `waitid` 选项位：关注正常退出或被信号终止的子进程事件。
pub(crate) const WEXITED: u32 = 4;
/// `wait4` / `waitid` 选项位：关注收到 `SIGCONT` 从停止态恢复继续执行的子进程事件。
pub(crate) const WCONTINUED: u32 = 8;
/// `waitid` 选项位：仅观察子进程状态事件，不消费该事件也不回收僵尸进程。
pub(crate) const WNOWAIT: u32 = 0x0100_0000;
/// `wait4` / `waitid` 选项位：同时匹配退出信号为 `SIGCHLD` 的普通子进程与自定义退出信号的 `clone` 子进程。
pub(crate) const WALL: u32 = 0x4000_0000;
/// `wait4` / `waitid` 选项位：仅匹配退出信号非 `SIGCHLD` 的 `clone` 子进程。
pub(crate) const WCLONE: u32 = 0x8000_0000;

/// Linux `wstatus` 低 7 位掩码：提取导致进程终止的信号编号（全 `0` 表示正常 `exit` 退出，`0x7f` 表示停止态）。
pub(crate) const WAIT_SIGNAL_MASK: i32 = 0x7f;
/// Linux `wstatus` 第 7 位标志：标记信号终止时是否伴随核心转储（Core Dump）。
pub(crate) const WAIT_CORE_DUMP: i32 = 0x80;
/// Linux `wstatus` 单字节掩码（用于提取低 8 位终止标志或右移后的 8 位退出码）。
pub(crate) const EXIT_STATUS_MASK: i32 = 0xff;
/// Linux `wstatus` 中正常退出码字段的左移位数（位于位 `[15:8]`）。
pub(crate) const EXIT_STATUS_SHIFT: u32 = 8;
/// Linux `wstatus` 表示子进程处于停止态的低字节特征值（`0x7f`，此时高字节存放停止信号编号）。
pub(crate) const WAIT_STOPPED: i32 = 0x7f;
/// Linux `wstatus` 表示子进程收到 `SIGCONT` 恢复运行的固定状态值（`0xffff`）。
pub(crate) const WAIT_CONTINUED: i32 = 0xffff;
/// Linux `wstatus` 全部合法编码位的总掩码（低 16 位 `0xffff`）。
pub(crate) const WAIT_STATUS_MASK: i32 = (EXIT_STATUS_MASK << EXIT_STATUS_SHIFT) | EXIT_STATUS_MASK;

/// 进程与线程关系图操作返回的错误类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ProcessError {
    /// 构造任务图时传入的 `max_pid` 为 `0` 或超出系统允许的 [`MAX_PID`] 上限。
    InvalidCapacity,
    /// 内核堆内存不足，无法分配控制结构、信号对象或槽位扩容页（对应 `ENOMEM`）。
    NoMemory,
    /// PID 号码耗尽或真实用户的 `RLIMIT_NPROC` 软限额已满（对应 `EAGAIN`）。
    NoPid,
    /// 指定的父线程组已退出、不属于当前图，或根进程创建违背 `PID 1` 唯一性约束。
    InvalidParent,
    /// 调用者名下不存在符合筛选条件的直属子进程（对应 `ECHILD`）。
    NoChild,
    /// 任务图正在关闭，或目标线程组正在执行 `execve` / 退出流程（对应 `EBUSY` 或重试拒绝）。
    Busy,
    /// 目标 PID/TID 未发布、已回收或不属于当前任务图（对应 `ESRCH`）。
    NoProcess,
    /// 跨用户/跨会话权限不足，或违反会话首进程/`NOFILE` 上限规则（对应 `EPERM`）。
    Permission,
    /// 父进程试图对已经执行过 `execve` 的子进程调用 `setpgid`（对应 `EACCES`）。
    Access,
    /// 系统调用参数超出合法范围或语义非法（对应 `EINVAL`）。
    InvalidArgument,
}

/// 任务图根进程的孤儿收养与系统关机角色（在构造 [`TaskGraph`] 时固定，不靠数字 `1` 推断）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RootRole {
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "用户执行 fixture 由内核回收根进程，普通 PID1 使用 System 角色"
        )
    )]
    /// 内核测试夹具模式：根进程由内核测试协调器直接等待回收，孤儿进程转归内核根（`parent = None`）。
    Kernel,
    /// 正式系统运行模式：首个无父进程固定为系统 `init`（`PID 1`），负责收养所有孤儿进程并授权触发系统关机。
    System,
}

/// 由用户态通过 `reboot` 系统调用提交并被任务图接受的系统停机动作。
///
/// 枚举判别值直接编码进 [`TaskGraph::closing`] 原子状态字（`0` 与 `1` 保留给运行态和内核取消态）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum ShutdownOperation {
    /// 停止所有操作系统活动并进入 CPU 停机挂起状态（Halt）。
    Halt = 2,
    /// 停止系统并请求底层固件/硬件切断电源（Power Off）。
    PowerOff = 3,
    /// 停止系统并请求底层固件/硬件执行冷重启（Restart）。
    Restart = 4,
}

/// `TaskGraph::closing` 状态值：任务图处于正常运行态，允许创建新进程与线程。
const RUNNING: u8 = 0;
/// `TaskGraph::closing` 状态值：任务图已被内核测试协调器取消，禁止新派生并请求所有存活组退出，但不触发机器关机动作。
const CANCELLED: u8 = 1;

/// 数值不可变的身份标识对象。
///
/// 每次在槽位上发布新身份都会构造独立的 `Arc<Pid>`；通过比较 `Arc` 指针即可区分先后复用同一数字的两个不同进程。外部诊断或选择器持有的 `Arc<Pid>` 不计入 `Slot::roles`，不会阻碍号码回收。
pub(crate) struct Pid {
    /// 1 基的正整数身份编号（对应槽位下标 `number - 1`）。
    number: usize,
}

impl Pid {
    /// 返回该身份对象绑定的 1 基整数编号（即使其关联的所有角色均已解绑，该编号仍保持不变）。
    pub(crate) fn number(&self) -> usize {
        self.number
    }
}

/// 线程组（即用户态视角下的“进程”）共享的身份、信号、时间与资源配额实体。
///
/// 不持有用户地址空间 `Mm`、打开文件表或内核栈，确保僵尸态进程或控制终端引用不会滞留重量级运行资源。
pub(crate) struct ThreadGroup {
    /// 所属任务图的弱引用，用于在关系校验时确认跨图边界且不形成强引用环。
    graph: Weak<TaskGraph>,
    /// 线程组不可变的 `TGID` 身份对象（在进程整个生命周期内保持不变）。
    pid: Arc<Pid>,
    /// 受自旋锁保护的可变作业控制关系（`PGID`、`SID`、退出信号及控制终端）。
    relations: SpinLock<GroupRelations>,
    /// 进程所属的不可变真实用户标识符（UID），用于信号发送与 `prlimit` 权限检查。
    uid: u32,
    /// 线程组共享的信号队列、作业停止控制状态与信号处理动作表绑定。
    signals: Arc<SignalGroup>,
    /// 线程组累计的用户态/内核态 CPU 时间、已回收子代 CPU 时间与间隔定时器（`itimer`）。
    time: Arc<GroupTime>,
    /// 受自旋锁保护的线程组共享资源软硬限额（`rlimit`）。
    limits: SpinLock<Limits>,
    /// 所属真实用户 UID 的全局资源记账账户，用于扣除与归还 `NPROC` 和 `SIGPENDING` 配额。
    account: Arc<UserAccount>,
}

/// 线程组在作业控制与退出通知中的可变关系状态（由 `ThreadGroup::relations` 自旋锁保护）。
struct GroupRelations {
    /// 当前所属进程组的 `PGID` 身份对象（可通过 `setpgid` 或 `setsid` 更新）。
    pgid: Arc<Pid>,
    /// 当前所属会话的 `SID` 身份对象（可通过 `setsid` 更新为自身 `pid`）。
    sid: Arc<Pid>,
    /// 进程终止时向父进程发送的通知信号编号（标准 `fork` 为 `SIGCHLD`，`execve` 或过继给 `init` 时重置为 `SIGCHLD`）。
    exit_signal: u8,
    /// 当前会话绑定的控制终端静态端点引用；未绑定或调用 `setsid`/脱离终端后为 `None`。
    controlling_tty: Option<&'static tty::Endpoint>,
}

/// 保存在 TTY 端点内的控制会话与前台进程组绑定记录。
///
/// 仅持有所属 `TaskGraph` 的弱引用以及会话/前台进程组的 `Arc<Pid>` 指针，不额外增加 `Slot::roles` 占用计数。
pub(crate) struct TerminalControl {
    /// 拥有该控制会话的任务图弱引用。
    pub(crate) graph: Weak<TaskGraph>,
    /// 拥有该控制终端的会话首 `SID` 身份对象。
    pub(crate) session: Arc<Pid>,
    /// 当前接收终端键盘信号与前台读写的进程组 `PGID` 身份对象。
    pub(crate) foreground: Arc<Pid>,
}

impl ThreadGroup {
    /// 返回线程组的数字标识符 `TGID`（即用户态 `getpid()` 返回的进程号）。
    pub(crate) fn tgid(&self) -> usize {
        self.pid.number
    }

    /// 返回线程组当前所属进程组的数字标识符 `PGID`。
    pub(crate) fn pgid(&self) -> usize {
        self.relations.lock().pgid.number
    }

    /// 克隆并返回当前所属进程组的 `Arc<Pid>` 身份对象（供阻塞 `wait` 等路径锚定原进程组，防止后续号码复用混淆目标）。
    pub(crate) fn pgid_object(&self) -> Arc<Pid> {
        Arc::clone(&self.relations.lock().pgid)
    }

    /// 返回线程组当前所属会话的数字标识符 `SID`。
    pub(crate) fn sid(&self) -> usize {
        self.relations.lock().sid.number
    }

    /// 克隆并返回当前所属会话的 `Arc<Pid>` 身份对象。
    pub(crate) fn sid_object(&self) -> Arc<Pid> {
        Arc::clone(&self.relations.lock().sid)
    }

    /// 返回线程组共享信号管理对象 [`SignalGroup`] 的引用。
    pub(crate) fn signals(&self) -> &Arc<SignalGroup> {
        &self.signals
    }

    /// 判断本线程组是否为 `System` 模式下的系统 `init` 根进程（仅由构造时的角色标志决定，普通模式下的 `PID 1` 返回 `false`）。
    pub(crate) fn is_system_init(&self) -> bool {
        self.signals.is_system_init()
    }

    /// 返回线程组共享 CPU 时间统计与间隔定时器对象 [`GroupTime`] 的引用。
    pub(crate) fn time(&self) -> &Arc<GroupTime> {
        &self.time
    }

    /// 读取指定资源类别 `resource` 当前生效的软硬限额副本。
    pub(crate) fn resource_limit(&self, resource: Resource) -> ResourceLimit {
        self.limits.lock().get(resource)
    }

    /// 按限额 `limit` 为发往本组用户的排队信号扣除 1 个 `SIGPENDING` 配额；达到上限时返回 [`SignalError::QueueFull`]。
    pub(super) fn charge_signal(&self, limit: u64) -> Result<SignalCharge, SignalError> {
        self.account
            .signal(limit)
            .map_err(|_| SignalError::QueueFull)
    }
}

/// 单个用户态线程持有的身份句柄。
///
/// 由运行中的 `UserThread` 强引用持有；反向由 `TaskGraph` 槽位通过 `Weak<UserThread>` 弱引用关联，不构成循环引用。
pub(crate) struct UserIdentity {
    /// 该线程所属的任务关系图。
    graph: Arc<TaskGraph>,
    /// 该线程所属的线程组。
    group: Arc<ThreadGroup>,
    /// 受自旋锁保护的线程私有 `TID` 身份对象（非组首线程执行 `execve` 接管组首时会将其替换为组首 `Pid`）。
    tid: SpinLock<Arc<Pid>>,
    /// 线程私有的挂起信号队列、信号屏蔽字与等待唤醒器状态。
    signals: Arc<ThreadSignals>,
}

impl UserIdentity {
    /// 返回线程当前的数字标识符 `TID`（非组首线程完成 `de_thread` 后将返回接管后的 `TGID`）。
    pub(crate) fn tid(&self) -> usize {
        self.tid.lock().number
    }

    /// 返回线程所属线程组的数字标识符 `TGID`。
    pub(crate) fn tgid(&self) -> usize {
        self.group.tgid()
    }

    /// 克隆并返回所属线程组 [`ThreadGroup`] 的强引用。
    pub(crate) fn group(&self) -> Arc<ThreadGroup> {
        self.group.clone()
    }

    /// 返回所属任务关系图 [`TaskGraph`] 的引用。
    pub(crate) fn graph(&self) -> &Arc<TaskGraph> {
        &self.graph
    }

    /// 克隆并返回当前线程的 `Arc<Pid>` 身份对象。
    pub(crate) fn tid_object(&self) -> Arc<Pid> {
        self.tid.lock().clone()
    }

    /// 克隆并返回线程私有信号状态 [`ThreadSignals`] 的强引用。
    pub(crate) fn signals(&self) -> Arc<ThreadSignals> {
        self.signals.clone()
    }
}

/// 内核协调器与 `try_wait` 消费的子进程退出摘要事件。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct WaitEvent {
    /// 已退出子进程的 `TGID` 编号。
    pub pid: usize,
    /// 符合 Linux `wait4` ABI 编码格式的 16 位终止状态字。
    pub status: i32,
    /// 该进程所有成员线程在用户态运行期间累计触发的外部中断总次数。
    pub user_interrupts: usize,
    /// 该进程所有成员线程在用户态运行期间累计触发的时钟中断总次数。
    pub user_timer_interrupts: usize,
    /// 进程退出时组首线程对应的内核 [`TaskId`]。
    pub task_id: TaskId,
    /// 最后一个退出成员提交退出事件时所在的逻辑 CPU 编号。
    pub cpu: usize,
}

/// 面向用户态 `wait4` / `waitid` 与 `SIGCHLD` `siginfo_t` 的子进程状态变更事件（涵盖退出、停止与继续）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ChildEvent {
    /// 状态发生变更的子进程 `TGID`。
    pub(crate) pid: usize,
    /// 子进程所属的真实用户 UID。
    pub(crate) uid: u32,
    /// `SIGCHLD` 事件原因码（`CLD_EXITED`、`CLD_KILLED`、`CLD_DUMPED`、`CLD_STOPPED` 或 `CLD_CONTINUED`）。
    pub(crate) code: i32,
    /// 未编码的原始退出码（`CLD_EXITED` 时为 `0..=255`）或导致终止/停止/继续的信号编号。
    pub(crate) status: i32,
    /// 子进程自身及其已回收后代累计的用户态 CPU 时间（单位：纳秒）。
    pub(crate) user_ns: u64,
    /// 子进程自身及其已回收后代累计的内核态 CPU 时间（单位：纳秒）。
    pub(crate) kernel_ns: u64,
}

/// 由 `wait4` / `waitid` 系统调用参数解析得到的目标子进程筛选器。
pub(crate) enum WaitSelector {
    /// 仅匹配指定 `Pid` 身份对象的单个子进程。
    Pid(Arc<Pid>),
    /// 匹配当前进程组等于指定 `Pid` 身份对象的任意直属子进程。
    Group(Arc<Pid>),
    /// 匹配调用者名下的任意直属子进程。
    Any,
}

/// 单个成员线程的内核栈切出完成观察节点（构成侵入式单向链表）。
///
/// 在发布线程前于锁外预先装箱，线程退出后挂入 `ProcessEntry::completed` 或 `Slot::completed`，仅持有轻量的 [`JoinObserver`] 而不滞留 `Task` 控制块或用户态资源。
pub(crate) struct PreparedExecution {
    /// 用于观察或阻塞等待对应内核 `Task` 彻底切离内核栈并释放栈页的观察句柄。
    join: JoinObserver,
    /// 指向同进程或同槽位下一个待确认切离内核栈观察节点的链表指针。
    next: Option<Box<PreparedExecution>>,
}

impl PreparedExecution {
    /// 在任务图锁外为新线程预分配单节点完成观察器；堆内存不足时返回 [`ProcessError::NoMemory`]。
    pub(crate) fn try_new(join: JoinObserver) -> Result<Box<Self>, ProcessError> {
        Box::try_new(Self { join, next: None }).map_err(|_| ProcessError::NoMemory)
    }
}

/// 槽位中记录单个存活线程的运行时表项（线程调用 `exit` 时即从槽位摘除）。
struct ThreadEntry {
    /// 指向用户线程运行时对象的弱引用，供跨核信号投递调用 `kick()` 打断执行。
    user: Weak<UserThread>,
    /// 该线程所属的线程组。
    group: Arc<ThreadGroup>,
    /// 该线程对应的内核栈切出完成观察节点（线程退出时移入 `ProcessEntry::completed`）。
    execution: Box<PreparedExecution>,
    /// 线程私有的信号队列与等待状态。
    signals: Arc<ThreadSignals>,
    /// 该线程占用的 1 个 `NPROC` 计费凭据（非组首退出时立即销毁退费，组首退出时移交 `ProcessEntry::leader_charge`）。
    charge: ProcessCharge,
}

/// 槽位中记录单个线程组（存活进程或尚未被回收的僵尸进程）的表项。
struct ProcessEntry {
    /// 线程组共享的身份与控制结构。
    group: Arc<ThreadGroup>,
    /// 当前直属父进程的 `TGID`（根进程或 `Kernel` 模式下父进程已退出时为 `None`；父进程退出时必须在号码复用前过继所有子进程）。
    parent: Option<usize>,
    /// 当前组内尚未调用 `exit` 的存活成员线程数量（归零表示整个进程已进入退出或僵尸状态）。
    members: usize,
    /// 当前组首线程对应的内核 [`TaskId`]（非组首线程通过 `de_thread` 接管组首时会更新此字段）。
    leader_task_id: TaskId,
    /// 组首线程先行退出时暂存的退出状态字（若进程未因组级致命信号退出，最终退出码优先采用组首状态）。
    leader_status: Option<i32>,
    /// 当前正在执行多线程 `execve` 收拢过程（`de_thread` ➔ `complete_exec`）的独占任务 ID。
    exec_owner: Option<TaskId>,
    /// 标记本进程自 `fork` 以来是否已成功完成过 `execve`（为 `true` 后禁止父进程再对其调用 `setpgid`）。
    did_exec: bool,
    /// 已退出成员线程汇总的用户态外部中断次数。
    user_interrupts: usize,
    /// 已退出成员线程汇总的用户态时钟中断次数。
    user_timer_interrupts: usize,
    /// 已调用 `exit` 但尚未被确认切离内核栈的成员观察节点单向链表。
    completed: Option<Box<PreparedExecution>>,
    /// 最后一个成员完成退出清理后发布的进程级退出事件（`Some` 表示进程已处于可被 `wait` 回收的僵尸态）。
    exit: Option<WaitEvent>,
    /// 待父进程通过 `wait4`/`waitid` 消费的最新作业控制状态事件（`CLD_STOPPED` 或 `CLD_CONTINUED`）。
    control_event: Option<ChildEvent>,
    /// 在图关系锁内提交、留待释放锁后由 `flush_group_notifications` 统一派发的延迟唤醒或父进程通知。
    notification: Option<GroupNotification>,
    /// 组首线程先行退出后暂存在僵尸记录中的 `NPROC` 计费凭据，直到进程被 `wait` 回收或 `execve` 接管时才释放退费。
    leader_charge: Option<ProcessCharge>,
}

/// 在持有 `TaskGraph::slots` 期间记下、留待锁外由 `flush_group_notifications` 执行的延迟通知任务。
enum GroupNotification {
    /// 仅需在锁外唤醒目标线程组成员检查新挂起的信号。
    Signal,
    /// 除唤醒目标线程组外，还需向其父进程发送 `CLD_CONTINUED` `SIGCHLD` 并唤醒父进程 `waiters`（优先级高于普通 `Signal`，不可被后者覆盖）。
    Continued(ChildEvent),
}

impl ProcessEntry {
    /// 逐个阻塞等待链表中所有已退出成员线程彻底切离各自的内核栈（仅由内核协调器在锁外且无活跃 `Waiter` 时调用）。
    fn join_members(&mut self) {
        while let Some(mut execution) = self.completed.take() {
            self.completed = execution.next.take();
            execution.join.join();
        }
    }
}

impl Drop for ProcessEntry {
    /// 循环摘除链表节点进行迭代释放，防止成员线程较多时触发深层递归析构导致内核栈溢出。
    fn drop(&mut self) {
        while let Some(mut execution) = self.completed.take() {
            self.completed = execution.next.take();
        }
    }
}

/// 从槽位中采集或回收子进程后打包返回的结果，用于在释放 `TaskGraph::slots` 自旋锁后完成延迟通知与对象析构。
struct CollectedChild {
    /// 子进程的退出摘要事件。
    event: WaitEvent,
    /// 被正式回收移除的 `ProcessEntry` 所有权（`nowait == true` 仅观察时为 `None`）。
    process: Option<ProcessEntry>,
    /// 若被回收进程在退出前仍有未派发的 `GroupNotification::Continued`，记录其父进程引用以便锁外补发通知。
    notification_parent: Option<Arc<ThreadGroup>>,
    /// 解绑 `TGID`、`PGID`、`SID` 三个角色时可能归还的最后一个 `Arc<Pid>` 引用，统一延迟到自旋锁外 Drop。
    _released: [Option<Arc<Pid>>; 3],
}

/// 分页槽位表中的单个号码槽位（下标 `i` 对应公开编号 `i + 1`）。
struct Slot {
    /// 标记该号码是否正被某个尚未调用 `publish` 的 [`NumberReservation`] 预留占用。
    reserved: bool,
    /// 当前占用该槽位编号的唯一身份对象；仅当 `reserved == false` 且 `roles == [0; 4]` 时为 `None`。
    pid: Option<Arc<Pid>>,
    /// 该槽位的 `pid` 对象当前分别担任 `[TID, TGID, PGID, SID]` 四类角色的活跃引用计数。
    roles: [usize; 4],
    /// 以该编号作为 `TID` 运行的存活线程表项。
    thread: Option<ThreadEntry>,
    /// 以该编号作为 `TGID` 存在的存活线程组或待回收僵尸进程表项。
    process: Option<ProcessEntry>,
    /// 进程已被用户态 `wait` 或自动回收释放后，转移至此暂存的未确认切离内核栈观察节点链表（不占用任何 PID 角色，下次分配前通过 `prune_completed` 清理）。
    completed: Option<Box<PreparedExecution>>,
}

impl Slot {
    /// 构造全空的初始空闲槽位（在锁外初始化新 [`numbers::SlotPage`] 时调用）。
    fn vacant() -> Self {
        Self {
            reserved: false,
            pid: None,
            roles: [0; 4],
            thread: None,
            process: None,
            completed: None,
        }
    }
}

impl Drop for Slot {
    /// 任务图整体销毁时迭代释放残余的完成观察节点（避免链表递归 Drop 消耗栈空间；尚未切出的内核栈仍由调度器独立保活）。
    fn drop(&mut self) {
        while let Some(mut execution) = self.completed.take() {
            self.completed = execution.next.take();
        }
    }
}

/// 管理一组用户进程/线程身份分配、父子关系、作业控制与生命周期同步的核心关系图。
pub(crate) struct TaskGraph {
    /// 受自旋锁保护的分页 PID 槽位表与全图亲缘关系状态。
    slots: SpinLock<Numbers>,
    /// 构造时选定的根进程收养与关机权限角色（`System` 或 `Kernel`）。
    root_role: RootRole,
    /// 图生命周期状态（`RUNNING`、`CANCELLED` 或 [`ShutdownOperation`] 的判别值；仅在持有 `slots` 锁时写入，锁外通过 `Acquire` 读取）。
    closing: AtomicU8,
    /// 父进程或内核协调器等待子进程退出/停止/继续事件的睡眠队列。
    waiters: WaitQueue,
    /// 处于作业控制停止态的线程以及等待多线程 `execve` 收拢完成的线程共用的控制等待队列。
    control_waiters: WaitQueue,
}

/// 未发布 PID 号码的独占预留守卫（RAII）。
///
/// 若锁外准备新进程或新线程中途失败（如内存不足），`Drop` 会自动加锁清除 `Slot::reserved` 并将槽位重新标为空闲。
struct NumberReservation {
    /// 预留槽位所属的任务图。
    graph: Arc<TaskGraph>,
    /// 预留的 0 基槽位下标（对应公开编号 `index + 1`）。
    index: usize,
    /// 标记预留是否仍未发布；一旦 `PreparedIdentity::publish` 成功提交则置为 `false`。
    active: bool,
}

/// 在锁外完成预分配与配额扣除、等待最终与 `PreparedTask` 一同原子发布的待生效用户身份包。
pub(crate) struct PreparedIdentity {
    /// 预先构造好的新线程身份对象。
    identity: Arc<UserIdentity>,
    /// 创建新进程时记录的父线程组引用（创建同组新线程或根进程时为 `None`）。
    parent: Option<Arc<ThreadGroup>>,
    /// 独占预留的 PID/TID 槽位守卫。
    reservation: NumberReservation,
    /// 已预先扣除的 1 个 `NPROC` 计费凭据（在 `publish` 中移入 `ThreadEntry::charge`）。
    charge: Option<ProcessCharge>,
}

impl TaskGraph {
    /// 创建最大号码上限为 `max_pid`、根进程角色为 `root_role` 的任务关系图（初始仅分配第一页 32 个槽位）。
    pub(crate) fn try_new(max_pid: usize, root_role: RootRole) -> Result<Arc<Self>, ProcessError> {
        let slots = Numbers::try_new(max_pid)?;
        Arc::try_new(Self {
            slots: SpinLock::new(slots),
            root_role,
            closing: AtomicU8::new(RUNNING),
            waiters: WaitQueue::new(),
            control_waiters: WaitQueue::new(),
        })
        .map_err(|_| ProcessError::NoMemory)
    }

    /// 返回本任务图在创建时设定的根进程角色（[`RootRole::System`] 或 [`RootRole::Kernel`]）。
    pub(crate) fn root_role(&self) -> RootRole {
        self.root_role
    }

    /// 记录首次用户态系统关机请求（`Halt`/`PowerOff`/`Restart`），禁止后续新进程/线程派生，并向图内所有存活线程组广播退出请求。
    ///
    /// 仅在 `RootRole::System` 模式且系统 `init`（`PID 1`）存活时允许调用；重复请求返回 [`ProcessError::Busy`]。
    pub(crate) fn request_shutdown(
        &self,
        operation: ShutdownOperation,
    ) -> Result<(), ProcessError> {
        let capacity = {
            let slots = self.slots.lock();
            if self.root_role != RootRole::System {
                return Err(ProcessError::InvalidArgument);
            }
            if self.closing.load(Ordering::Relaxed) != RUNNING {
                return Err(ProcessError::Busy);
            }
            let root = slots[0]
                .process
                .as_ref()
                .ok_or(ProcessError::InvalidParent)?;
            if !root.group.is_system_init()
                || root.parent.is_some()
                || root.members == 0
                || root.exit.is_some()
                || root.group.signals.exit_request().is_some()
            {
                return Err(ProcessError::InvalidParent);
            }
            self.closing.store(operation as u8, Ordering::Release);
            for process in slots.iter().filter_map(|slot| slot.process.as_ref()) {
                if 0 < process.members {
                    process.group.signals.mark_exit(0);
                }
            }
            slots.len()
        };

        self.notify_exiting_groups(capacity);
        Ok(())
    }

    /// 读取任务图已接受的用户态系统关机操作；若仍处于运行态或仅被内核测试取消（`CANCELLED`），则返回 `None`。
    pub(crate) fn shutdown_operation(&self) -> Option<ShutdownOperation> {
        const HALT: u8 = ShutdownOperation::Halt as u8;
        const POWER_OFF: u8 = ShutdownOperation::PowerOff as u8;
        const RESTART: u8 = ShutdownOperation::Restart as u8;
        match self.closing.load(Ordering::Acquire) {
            RUNNING | CANCELLED => None,
            HALT => Some(ShutdownOperation::Halt),
            POWER_OFF => Some(ShutdownOperation::PowerOff),
            RESTART => Some(ShutdownOperation::Restart),
            _ => panic!("invalid task graph shutdown state"),
        }
    }

    /// 在锁外准备新进程（新线程组及组首线程）的身份、信号队列、时间记账与 `NPROC` 配额，并预留一个空闲 `PID` 槽位。
    ///
    /// 若指定了 `parent`，新进程复制父进程当前的四项 `rlimit` 并在发布时继承父进程的 `PGID`、`SID` 与控制终端；若 `parent` 为 `None`，则自建进程组与会话。中途任何分配或配额检查失败都会通过 RAII 自动退费并归还预留号码。
    pub(crate) fn prepare_process(
        self: &Arc<Self>,
        parent: Option<&Arc<ThreadGroup>>,
        sighand: Arc<Sighand>,
        uid: u32,
        inherited: SignalInheritance,
        exit_signal: u8,
    ) -> Result<PreparedIdentity, ProcessError> {
        // Step 1: 获取目标 UID 记账账户、复制父进程限额，并先行扣除 1 个 NPROC 配额与预留 PID 槽位。
        let account = UserAccount::get(uid)?;
        let limits = parent.map_or_else(Limits::new, |parent| *parent.limits.lock());
        let charge = account.process(limits.get(Resource::Nproc).current)?;
        let reservation = self.reserve_number(parent, false)?;
        let pid = Arc::try_new(Pid {
            number: reservation.index + 1,
        })
        .map_err(|_| ProcessError::NoMemory)?;

        // Step 2: 快照初始 PGID/SID，构造组级信号、线程私有信号、组时间及 ThreadGroup / UserIdentity 对象。
        let (pgid, sid) = match parent {
            Some(parent) => {
                let relations = parent.relations.lock();
                (relations.pgid.clone(), relations.sid.clone())
            }
            None => (pid.clone(), pid.clone()),
        };

        let system_init = self.root_role == RootRole::System && parent.is_none();
        let signals = SignalGroup::try_new(Arc::downgrade(self), pid.number, sighand, system_init)
            .map_err(|_| ProcessError::NoMemory)?;
        let private = ThreadSignals::try_new(signals.clone(), inherited)
            .map_err(|_| ProcessError::NoMemory)?;
        let time = GroupTime::try_new(signals.clone()).map_err(|_| ProcessError::NoMemory)?;
        let group = Arc::try_new(ThreadGroup {
            graph: Arc::downgrade(self),
            pid: pid.clone(),
            relations: SpinLock::new(GroupRelations {
                pgid,
                sid,
                exit_signal,
                controlling_tty: None,
            }),
            uid,
            signals,
            time,
            limits: SpinLock::new(limits),
            account,
        })
        .map_err(|_| ProcessError::NoMemory)?;

        let identity = Arc::try_new(UserIdentity {
            graph: self.clone(),
            group,
            tid: SpinLock::new(pid),
            signals: private,
        })
        .map_err(|_| ProcessError::NoMemory)?;
        Ok(PreparedIdentity {
            identity,
            parent: parent.cloned(),
            reservation,
            charge: Some(charge),
        })
    }

    /// 在锁外为已有线程组 `group` 准备同组新线程的私有 `TID`、信号状态与 `NPROC` 配额。
    ///
    /// 新线程共享所属 `group` 的 `TGID`、信号动作表与资源限额；预留与后续 `publish` 时均会核对所属线程组仍存活且未在执行 `execve`。
    pub(crate) fn prepare_thread(
        self: &Arc<Self>,
        group: &Arc<ThreadGroup>,
        inherited: SignalInheritance,
    ) -> Result<PreparedIdentity, ProcessError> {
        let charge = group
            .account
            .process(group.resource_limit(Resource::Nproc).current)?;
        let reservation = self.reserve_number(Some(group), true)?;
        let tid = Arc::try_new(Pid {
            number: reservation.index + 1,
        })
        .map_err(|_| ProcessError::NoMemory)?;
        let signals = ThreadSignals::try_new(group.signals.clone(), inherited)
            .map_err(|_| ProcessError::NoMemory)?;
        let identity = Arc::try_new(UserIdentity {
            graph: self.clone(),
            group: group.clone(),
            tid: SpinLock::new(tid),
            signals,
        })
        .map_err(|_| ProcessError::NoMemory)?;

        Ok(PreparedIdentity {
            identity,
            parent: None,
            reservation,
            charge: Some(charge),
        })
    }

    /// 清理已切离内核栈的旧观察节点后，在锁外按需预分配槽位扩容页并在 `slots` 锁内原子预留最小空闲号码。
    ///
    /// 预留前校验任务图处于 `RUNNING` 态、父进程组（若有）存活且未处于同组 `execve` 收拢期间；`System` 模式下创建无父根进程强制要求占用 0 号槽位（`PID 1`）。
    fn reserve_number(
        self: &Arc<Self>,
        parent: Option<&Arc<ThreadGroup>>,
        thread: bool,
    ) -> Result<NumberReservation, ProcessError> {
        self.prune_completed();
        let mut growth = None;
        loop {
            let (pages, directory) = {
                let mut slots = self.slots.lock();
                if self.closing.load(Ordering::Relaxed) != RUNNING {
                    return Err(ProcessError::Busy);
                }
                if let Some(parent) = parent {
                    self.live_parent(&slots, parent)?;
                    if thread
                        && slots[parent.tgid() - 1]
                            .process
                            .as_ref()
                            .expect("live group disappeared")
                            .exec_owner
                            .is_some()
                    {
                        return Err(ProcessError::Busy);
                    }
                }
                let root = self.root_role == RootRole::System && parent.is_none();
                if root && (slots[0].reserved || slots[0].pid.is_some()) {
                    return Err(ProcessError::InvalidParent);
                }
                if let Some(index) = if root { Some(0) } else { slots.first_free() } {
                    slots[index].reserved = true;
                    slots.set_free(index, false);
                    return Ok(NumberReservation {
                        graph: self.clone(),
                        index,
                        active: true,
                    });
                }
                if slots.exhausted() {
                    return Err(ProcessError::NoPid);
                }
                if let Some(prepared) = growth.as_mut() {
                    if slots.install(prepared) {
                        continue;
                    }
                }
                slots.growth_requirement()
            };

            growth = Some(Numbers::prepare_growth(pages, directory)?);
        }
    }

    /// 遍历全图槽位，摘除所有经非阻塞检查确认已切离内核栈的 `PreparedExecution` 节点并在自旋锁外统一释放。
    fn prune_completed(&self) {
        let mut discarded = None;
        {
            let mut slots = self.slots.lock();
            for slot in slots.iter_mut() {
                discard_completed(&mut slot.completed, &mut discarded);
                if let Some(process) = slot.process.as_mut() {
                    discard_completed(&mut process.completed, &mut discarded);
                }
            }
        }

        while let Some(mut execution) = discarded.take() {
            discarded = execution.next.take();
        }
    }

    /// 按数字 `tid` 查找当前已发布的存活线程，并在释放图锁后尝试升级为 `Arc<UserThread>` 强引用。
    pub(crate) fn lookup_thread(&self, tid: usize) -> Option<Arc<UserThread>> {
        let user = {
            let slots = self.slots.lock();
            slots
                .get(tid.checked_sub(1)?)?
                .thread
                .as_ref()?
                .user
                .clone()
        };

        user.upgrade()
    }

    /// 按 `Arc<Pid>` 身份对象精确查找对应的存活线程；若该数字已被新线程复用（`Pid` 指针不同）则返回 `None`。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "PID 复用和候选回滚测试核对原身份对象不会命中新同号线程"
        )
    )]
    pub(crate) fn lookup_pid(&self, pid: &Arc<Pid>) -> Option<Arc<UserThread>> {
        let user = {
            let slots = self.slots.lock();
            let slot = slots.get(pid.number.checked_sub(1)?)?;
            if !slot
                .pid
                .as_ref()
                .is_some_and(|entry| Arc::ptr_eq(entry, pid))
            {
                return None;
            }
            slot.thread.as_ref()?.user.clone()
        };

        user.upgrade()
    }

    /// 读取指定数字编号 `pid` 当前槽位上绑定的 `Arc<Pid>` 身份对象快照（用于 `wait` 等路径在睡眠前锚定目标身份）。
    pub(crate) fn pid_object(&self, pid: usize) -> Option<Arc<Pid>> {
        self.slots.lock().get(pid.checked_sub(1)?)?.pid.clone()
    }

    /// 为指定用户身份所属的线程组记录首个组级退出状态码（如致命信号或 `exit_group`），并在锁外唤醒同组所有成员前往安全点退出。
    pub(crate) fn request_exit(&self, identity: &UserIdentity, status: i32) {
        {
            let slots = self.slots.lock();
            assert!(ptr::eq(self, identity.graph.as_ref()));
            let process = slots[identity.tgid() - 1]
                .process
                .as_ref()
                .expect("exit requested for an unpublished process");
            assert!(Arc::ptr_eq(&process.group, &identity.group));
            identity.group.signals.mark_exit(status);
        }

        identity.group.signals.notify();
    }

    /// 查询当前线程所属线程组是否已收到组级退出请求及其退出状态字。
    pub(crate) fn exit_request(&self, identity: &UserIdentity) -> Option<i32> {
        identity.group.signals.exit_request()
    }

    /// 返回供父进程或内核协调器睡眠等待子进程状态变更的等待队列。
    pub(crate) fn waiters(&self) -> &WaitQueue {
        &self.waiters
    }

    /// 由内核测试协调器主动取消整个任务图：若图仍处于 `RUNNING` 态则置为 `CANCELLED` 拒绝新派生，并向所有存活线程组发出退出请求。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "用户 fixture 清理时取消图内真实成员，随后仍等待退出与离栈"
        )
    )]
    pub(crate) fn request_root_exit(&self, status: i32) {
        let capacity = {
            let slots = self.slots.lock();
            if self.closing.load(Ordering::Relaxed) == RUNNING {
                self.closing.store(CANCELLED, Ordering::Release);
            }
            for process in slots.iter().filter_map(|slot| slot.process.as_ref()) {
                if 0 < process.members {
                    process.group.signals.mark_exit(status);
                }
            }
            slots.len()
        };

        self.notify_exiting_groups(capacity);
    }

    /// 按槽位容量上界逐个克隆存活线程组引用并在释放图锁后发出信号唤醒通知，避免为全图广播额外分配堆内存。
    fn notify_exiting_groups(&self, capacity: usize) {
        for index in 0..capacity {
            let group = {
                let slots = self.slots.lock();
                slots[index]
                    .process
                    .as_ref()
                    .filter(|process| 0 < process.members)
                    .map(|process| process.group.clone())
            };
            if let Some(group) = group {
                group.signals.notify();
            }
        }
    }

    /// 返回供进入作业控制停止态的线程或等待 `de_thread` 收拢的线程睡眠的控制等待队列。
    pub(crate) fn control_waiters(&self) -> &WaitQueue {
        &self.control_waiters
    }

    /// 由运行到内核返回用户态安全点的线程调用，确认本线程响应组级停止控制；当组内最后一个存活成员完成停止确认时，发布 `CLD_STOPPED` 事件并通知父进程。
    pub(crate) fn acknowledge_control(&self, identity: &UserIdentity) -> SignalControl {
        let (control, event) = {
            let mut slots = self.slots.lock();
            let thread = slots[identity.tid() - 1]
                .thread
                .as_ref()
                .expect("unpublished member acknowledged signal control");
            assert!(Arc::ptr_eq(&thread.group, &identity.group));
            let (control, stopped) = thread.signals.acknowledge_control();
            let event =
                stopped.map(|signum| child_event(&identity.group, CLD_STOPPED, signum as i32));
            if let Some(event) = event {
                slots[identity.tgid() - 1]
                    .process
                    .as_mut()
                    .expect("stopping member has no process")
                    .control_event = Some(event);
            }
            (control, event)
        };

        if let Some(event) = event {
            self.notify_parent(&identity.group, event);
            self.waiters.wake_all();
        }
        control
    }

    /// 非阻塞地扫描调用者 `parent` 名下符合 `selector` 与 `options` 条件的直属子进程，消费（除非指定 `WNOWAIT`）并返回首个就绪的退出、停止或继续事件。
    ///
    /// 若不存在任何满足选择器与 `WALL`/`WCLONE` 类别的直属子进程则返回 [`ProcessError::NoChild`]（`ECHILD`）；若存在匹配子进程但尚无就绪事件则返回 `Ok(None)`。
    pub(crate) fn try_wait_event(
        &self,
        parent: &ThreadGroup,
        selector: &WaitSelector,
        options: u32,
    ) -> Result<Option<ChildEvent>, ProcessError> {
        let mut retired = None;
        let event = {
            // Step 1: 在关系锁内验证父进程存活，扫描直属子进程并按选择器与克隆标志匹配退出或控制事件。
            let mut slots = self.slots.lock();
            let parent_number = self
                .live_parent(&slots, parent)
                .map_err(|_| ProcessError::NoChild)?;
            let mut eligible = false;
            let mut selected = None;
            for index in 0..slots.len() {
                let Some(process) = slots[index].process.as_ref() else {
                    continue;
                };
                if process.parent != Some(parent_number) {
                    continue;
                }
                let matches = match selector {
                    WaitSelector::Pid(pid) => Arc::ptr_eq(&process.group.pid, pid),
                    WaitSelector::Group(pgid) => {
                        Arc::ptr_eq(&process.group.relations.lock().pgid, pgid)
                    }
                    WaitSelector::Any => true,
                };
                let clone = process.group.relations.lock().exit_signal != SIGCHLD;
                if !matches || (options & WALL == 0 && clone != (options & WCLONE != 0)) {
                    continue;
                }
                eligible = true;
                if options & WEXITED != 0 {
                    if let Some(exit) = process.exit {
                        selected = Some(exit_child_event(&process.group, exit.status));
                        if options & WNOWAIT == 0 {
                            parent.time.add_reaped_child(&process.group.time);
                            retired = Some(reap_child(&mut slots, index, false));
                        }
                        break;
                    }
                }
                let control = process.control_event.filter(|event| {
                    (event.code == CLD_STOPPED && options & WSTOPPED != 0)
                        || (event.code == CLD_CONTINUED && options & WCONTINUED != 0)
                });
                if let Some(event) = control {
                    selected = Some(event);
                    if options & WNOWAIT == 0 {
                        slots[index]
                            .process
                            .as_mut()
                            .expect("selected process disappeared")
                            .control_event = None;
                    }
                    break;
                }
            }
            if !eligible {
                return Err(ProcessError::NoChild);
            }
            selected
        };

        // Step 2: 在自旋锁外补发被回收子进程遗留的延迟通知、唤醒其他等待者并销毁已回收的进程结构。
        if let Some(retired) = retired.as_mut() {
            self.complete_retired_notification(retired);
        }
        if retired.is_some() {
            self.waiters.wake_all();
        }
        drop(retired);
        Ok(event)
    }

    /// 查询 `identity` 当前存活直属父进程的 `TGID`（对应 `getppid`）；若属于无父根进程或父进程位于内核根下则返回 `0`。
    pub(crate) fn parent_pid(&self, identity: &UserIdentity) -> usize {
        let slots = self.slots.lock();
        if !ptr::eq(self, identity.graph.as_ref()) {
            return 0;
        }
        slots
            .get(identity.tgid() - 1)
            .and_then(|slot| slot.process.as_ref())
            .filter(|process| Arc::ptr_eq(&process.group, &identity.group))
            .and_then(|process| process.parent)
            .and_then(|pid| slots[pid - 1].process.as_ref())
            .filter(|parent| parent.members != 0)
            .map_or(0, |parent| parent.group.tgid())
    }

    /// 查询目标进程（`pid == 0` 表示调用者自身）当前所属的进程组编号 `PGID`（对应 `getpgid`）。
    pub(crate) fn get_pgid(
        &self,
        caller: &UserIdentity,
        pid: usize,
    ) -> Result<usize, ProcessError> {
        let slots = self.slots.lock();
        let group = self.identity_target(&slots, caller, pid)?;
        Ok(group.pgid())
    }

    /// 查询目标进程（`pid == 0` 表示调用者自身）当前所属的会话编号 `SID`（对应 `getsid`）。
    pub(crate) fn get_sid(&self, caller: &UserIdentity, pid: usize) -> Result<usize, ProcessError> {
        let slots = self.slots.lock();
        let group = self.identity_target(&slots, caller, pid)?;
        Ok(group.sid())
    }

    /// 在持有 `slots` 锁的前提下校验调用者仍为存活组成员，并解析 `pid`（`0` 取调用者 `TGID`）对应的进程或线程所属 [`ThreadGroup`]。
    fn identity_target<'a>(
        &self,
        slots: &'a Numbers,
        caller: &UserIdentity,
        pid: usize,
    ) -> Result<&'a ThreadGroup, ProcessError> {
        self.live_parent(slots, &caller.group)
            .map_err(|_| ProcessError::NoProcess)?;
        let pid = if pid == 0 { caller.tgid() } else { pid };
        let slot = slots.get(pid - 1).ok_or(ProcessError::NoProcess)?;
        slot.process
            .as_ref()
            .map(|process| process.group.as_ref())
            .or_else(|| slot.thread.as_ref().map(|thread| thread.group.as_ref()))
            .ok_or(ProcessError::NoProcess)
    }

    /// 在持有 `slots` 锁的前提下校验终端操作调用者仍是本图中已发布的存活线程与存活线程组成员。
    fn terminal_caller(&self, slots: &Numbers, caller: &UserIdentity) -> Result<(), FsError> {
        if !ptr::eq(self, caller.graph.as_ref()) {
            return Err(FsError::NoProcess);
        }
        self.live_parent(slots, &caller.group)
            .map_err(|_| FsError::NoProcess)?;
        let thread = slots
            .get(caller.tid() - 1)
            .and_then(|slot| slot.thread.as_ref());
        if !thread.is_some_and(|thread| {
            Arc::ptr_eq(&thread.group, &caller.group)
                && Arc::ptr_eq(&thread.signals, &caller.signals)
        }) {
            return Err(FsError::NoProcess);
        }
        Ok(())
    }

    /// 为会话首进程绑定控制终端 `tty`，并将终端初始前台进程组设为调用者当前的进程组（对应 `open` 自动获取控制终端或 `TIOCSCTTY`）。
    ///
    /// 调用方须先持有 `tty::Endpoint::copy` 睡眠锁；本方法按 `slots` ➔ `relations` ➔ `tty.control()` 锁顺序提交。当不满足会话首或空闲终端条件时，若 `automatic == true`（`open` 路径）则静默保持原状返回 `Ok(())`，否则返回 `FsError::NotPermitted`。
    pub(crate) fn acquire_tty(
        &self,
        caller: &UserIdentity,
        tty: &'static tty::Endpoint,
        automatic: bool,
    ) -> Result<(), FsError> {
        let slots = self.slots.lock();
        self.terminal_caller(&slots, caller)?;
        let mut relations = caller.group.relations.lock();
        let mut control = tty.control().lock();
        let same = relations
            .controlling_tty
            .is_some_and(|bound| ptr::eq(bound, tty))
            && control.as_ref().is_some_and(|control| {
                ptr::eq(control.graph.as_ptr(), self)
                    && Arc::ptr_eq(&control.session, &relations.sid)
            });
        if Arc::ptr_eq(&caller.group.pid, &relations.sid) && same {
            return Ok(());
        }
        if !Arc::ptr_eq(&caller.group.pid, &relations.sid)
            || relations.controlling_tty.is_some()
            || control.is_some()
        {
            return if automatic {
                Ok(())
            } else {
                Err(FsError::NotPermitted)
            };
        }

        *control = Some(TerminalControl {
            graph: Arc::downgrade(&caller.graph),
            session: relations.sid.clone(),
            foreground: relations.pgid.clone(),
        });
        relations.controlling_tty = Some(tty);
        Ok(())
    }

    /// 查询调用者当前绑定的控制终端端点（用于打开 `/dev/tty` 时解析目标设备）；若调用者已退出或未绑定控制终端则返回 `None`。
    pub(crate) fn controlling_tty(&self, caller: &UserIdentity) -> Option<&'static tty::Endpoint> {
        let slots = self.slots.lock();
        self.terminal_caller(&slots, caller).ok()?;
        let tty = caller.group.relations.lock().controlling_tty;
        tty
    }

    /// 当且仅当 `tty` 是调用者当前绑定的控制终端且归属同一会话时，返回该终端关联的 `(会话 SID 对象, 前台进程组 PGID 对象)`（对应 `TIOCGSID` / `TIOCGPGRP`）。
    pub(crate) fn terminal_ids(
        &self,
        caller: &UserIdentity,
        tty: &'static tty::Endpoint,
    ) -> Result<(Arc<Pid>, Arc<Pid>), FsError> {
        let slots = self.slots.lock();
        self.terminal_caller(&slots, caller)?;
        let relations = caller.group.relations.lock();
        if !relations
            .controlling_tty
            .is_some_and(|bound| ptr::eq(bound, tty))
        {
            return Err(FsError::NotTty);
        }
        let control = tty.control().lock();
        let control = control
            .as_ref()
            .filter(|control| {
                ptr::eq(control.graph.as_ptr(), self)
                    && Arc::ptr_eq(&control.session, &relations.sid)
            })
            .ok_or(FsError::NotTty)?;
        Ok((control.session.clone(), control.foreground.clone()))
    }

    /// 将控制终端 `tty` 的前台进程组切换为同会话内的目标编号 `number`（对应 `TIOCSPGRP`）。
    ///
    /// 在同一 `slots` 锁临界区内验证 `tty` 归属调用者所在会话，并确认目标 `Pid` 在同一会话中具有活跃进程组或存活线程角色；被替换的旧前台 `Arc<Pid>` 在锁外释放。
    pub(crate) fn set_terminal_foreground(
        &self,
        caller: &UserIdentity,
        tty: &'static tty::Endpoint,
        number: usize,
    ) -> Result<(), FsError> {
        if (i32::MAX as usize) < number {
            return Err(FsError::InvalidOperation);
        }

        let retired = {
            let slots = self.slots.lock();
            self.terminal_caller(&slots, caller)?;
            let session = {
                let relations = caller.group.relations.lock();
                if !relations
                    .controlling_tty
                    .is_some_and(|bound| ptr::eq(bound, tty))
                {
                    return Err(FsError::NotTty);
                }
                relations.sid.clone()
            };

            let slot = slots
                .get(number.checked_sub(1).ok_or(FsError::NoProcess)?)
                .ok_or(FsError::NoProcess)?;
            let target = slot.pid.as_ref().ok_or(FsError::NoProcess)?;
            let target_sid = session_of_pgrp(&slots, target).ok_or(FsError::NotPermitted)?;
            if !Arc::ptr_eq(&target_sid, &session) {
                return Err(FsError::NotPermitted);
            }

            let mut control = tty.control().lock();
            let control = control
                .as_mut()
                .filter(|control| {
                    ptr::eq(control.graph.as_ptr(), self) && Arc::ptr_eq(&control.session, &session)
                })
                .ok_or(FsError::NotTty)?;
            core::mem::replace(&mut control.foreground, target.clone())
        };

        drop(retired);
        Ok(())
    }

    /// 在终端挂断（Hangup）路径中解除会话 `session` 内所有进程对 `tty` 的控制终端引用，并取走端点内的 [`TerminalControl`]（调用方在外层持有 `tty::Endpoint::copy` 锁）。
    pub(crate) fn release_tty_session(
        &self,
        tty: &'static tty::Endpoint,
        session: &Arc<Pid>,
    ) -> Option<TerminalControl> {
        let slots = self.slots.lock();
        if !has_role(&slots, session, SID) {
            return None;
        }
        release_terminal_locked(&slots, self, tty, session)
    }

    /// 脱离当前进程对控制终端 `tty` 的关联（对应 `TIOCNOTTY`）：非会话首仅清除本组 `controlling_tty`；会话首则解除整个会话的绑定并返回 [`TerminalControl`] 供调用者在锁外向原前台组发送 `SIGHUP`/`SIGCONT`。
    pub(crate) fn disassociate_tty(
        &self,
        caller: &UserIdentity,
        tty: &'static tty::Endpoint,
    ) -> Result<Option<TerminalControl>, FsError> {
        let slots = self.slots.lock();
        self.terminal_caller(&slots, caller)?;
        let session = {
            let mut relations = caller.group.relations.lock();
            if !relations
                .controlling_tty
                .is_some_and(|bound| ptr::eq(bound, tty))
            {
                return Err(FsError::NotTty);
            }
            if !Arc::ptr_eq(&relations.sid, &caller.group.pid) {
                relations.controlling_tty = None;
                return Ok(None);
            }
            relations.sid.clone()
        };
        Ok(release_terminal_locked(&slots, self, tty, &session))
    }

    /// 向进程组 `pgid` 内的全部存活进程广播内核信号 `signum`（供终端键盘中断/挂断等路径调用），并在释放图锁后统一派发唤醒与父进程通知。
    pub(crate) fn send_process_group(
        &self,
        pgid: &Arc<Pid>,
        signum: u8,
    ) -> Result<(), SignalError> {
        if signum == 0 || SIGNAL_COUNT < signum as usize {
            return Err(SignalError::InvalidSignal);
        }

        let result = {
            let mut slots = self.slots.lock();
            deliver_process_group_locked(&mut slots, pgid, signum)
        };

        self.flush_group_notifications();
        result
    }

    /// 检查调用者对控制终端 `tty` 的后台读/写/配置访问权限（`signum` 为 `SIGTTIN` 或 `SIGTTOU`）。
    ///
    /// 若非自身控制终端或本组处于前台则直接放行；后台访问时，若阻塞/忽略 `SIGTTOU` 则放行，若阻塞/忽略 `SIGTTIN` 或所属进程组已成孤儿组则返回 `FsError::DeviceIo`（`EIO`），否则向整个后台进程组投递 `signum` 并返回 `FsError::Interrupted` 触发上层阻塞与重试。
    pub(crate) fn check_terminal_access(
        &self,
        caller: &UserIdentity,
        tty: &'static tty::Endpoint,
        signum: u8,
    ) -> Result<(), FsError> {
        assert!(
            matches!(signum, SIGTTIN | SIGTTOU),
            "invalid terminal access signal"
        );
        let result = {
            let mut slots = self.slots.lock();
            self.terminal_caller(&slots, caller)?;
            let pgid = {
                let relations = caller.group.relations.lock();
                if !relations
                    .controlling_tty
                    .is_some_and(|bound| ptr::eq(bound, tty))
                {
                    return Ok(());
                }
                let control = tty.control().lock();
                let control = control
                    .as_ref()
                    .filter(|control| {
                        ptr::eq(control.graph.as_ptr(), self)
                            && Arc::ptr_eq(&control.session, &relations.sid)
                    })
                    .ok_or(FsError::NotTty)?;
                if Arc::ptr_eq(&control.foreground, &relations.pgid) {
                    return Ok(());
                }
                relations.pgid.clone()
            };
            if caller.signals.blocked_or_ignored(signum) {
                return if signum == SIGTTOU {
                    Ok(())
                } else {
                    Err(FsError::DeviceIo)
                };
            }
            if orphaned_process_group(&slots, &pgid) {
                return Err(FsError::DeviceIo);
            }

            match deliver_process_group_locked(&mut slots, &pgid, signum) {
                Ok(()) => Err(FsError::Interrupted),
                Err(SignalError::NoProcess) => Err(FsError::NoProcess),
                Err(error) => panic!("ordinary terminal signal failed: {:?}", error),
            }
        };

        self.flush_group_notifications();
        result
    }

    /// 逐槽取出在持有 `slots` 锁期间登记的 `GroupNotification`，并在自旋锁外完成目标组信号唤醒与父进程 `CLD_CONTINUED` 通知。
    fn flush_group_notifications(&self) {
        let capacity = self.slots.lock().len();
        for index in 0..capacity {
            let pending = {
                let mut slots = self.slots.lock();
                let parent = slots[index]
                    .process
                    .as_ref()
                    .and_then(|process| {
                        if !matches!(process.notification, Some(GroupNotification::Continued(_))) {
                            return None;
                        }
                        process.parent
                    })
                    .and_then(|number| slots[number - 1].process.as_ref())
                    .map(|parent| parent.group.clone());
                slots[index].process.as_mut().and_then(|process| {
                    process
                        .notification
                        .take()
                        .map(|notification| (process.group.clone(), notification, parent))
                })
            };

            if let Some((group, notification, parent)) = pending {
                self.complete_group_notification(&group, notification, parent.as_deref());
            }
        }
    }

    /// 在不持有任何图锁、关系锁或信号锁的前提下，执行单个线程组的信号唤醒及可选的父进程 `SIGCHLD`（`CLD_CONTINUED`）投递。
    fn complete_group_notification(
        &self,
        group: &ThreadGroup,
        notification: GroupNotification,
        parent: Option<&ThreadGroup>,
    ) {
        group.signals.notify();
        if let GroupNotification::Continued(event) = notification {
            send_parent_event(parent, event);
            self.waiters.wake_all();
        }
    }

    /// 若刚被 `wait` 或自动回收移除的进程上仍携带着尚未派发的延迟通知，在彻底销毁该进程前于锁外补发完毕。
    fn complete_retired_notification(&self, retired: &mut CollectedChild) {
        if let Some(process) = retired.process.as_mut() {
            if let Some(notification) = process.notification.take() {
                self.complete_group_notification(
                    &process.group,
                    notification,
                    retired.notification_parent.as_deref(),
                );
            }
        }
    }

    /// 修改调用者自身或直属子进程的进程组 `PGID`（对应 `setpgid` 系统调用），并在原进程组或新进程组变为孤儿且含停止态作业时触发 `SIGHUP`/`SIGCONT`。
    ///
    /// 拒绝修改会话首进程、跨会话子进程或已完成/正在执行 `execve` 的子进程；跨入已有进程组时要求该进程组在同会话内存在成员。
    pub(crate) fn set_pgid(
        &self,
        caller: &UserIdentity,
        pid: usize,
        pgid: usize,
    ) -> Result<(), ProcessError> {
        if (i32::MAX as usize) < pid || (i32::MAX as usize) < pgid {
            return Err(ProcessError::InvalidArgument);
        }

        // Step 1: 在关系锁内校验调用者与目标进程的身份、父子关系、会话边界及 execve 状态。
        let released = {
            let mut slots = self.slots.lock();
            self.live_parent(&slots, &caller.group)
                .map_err(|_| ProcessError::NoProcess)?;
            let pid = if pid == 0 { caller.tgid() } else { pid };
            let pgid = if pgid == 0 { pid } else { pgid };
            let index = pid - 1;
            let slot = slots.get(index).ok_or(ProcessError::NoProcess)?;
            let process = match slot.process.as_ref() {
                Some(process) => process,
                None if slot.thread.is_some() => return Err(ProcessError::InvalidArgument),
                None => return Err(ProcessError::NoProcess),
            };
            if process.members == 0 {
                return Err(ProcessError::NoProcess);
            }
            let child = process.parent == Some(caller.tgid());
            if pid != caller.tgid() && !child {
                return Err(ProcessError::NoProcess);
            }
            let group = process.group.clone();
            let caller_sid = caller.group.relations.lock().sid.clone();
            if child {
                if !Arc::ptr_eq(&group.relations.lock().sid, &caller_sid) {
                    return Err(ProcessError::Permission);
                }
                if process.did_exec || process.exec_owner.is_some() {
                    return Err(ProcessError::Access);
                }
            }
            if Arc::ptr_eq(&group.relations.lock().sid, &group.pid) {
                return Err(ProcessError::Permission);
            }

            // Step 2: 解析目标新 PGID 对象（新建同名组或加入同会话内已存在的进程组）。
            let new_pgid = if pgid == pid {
                group.pid.clone()
            } else {
                let slot = slots.get(pgid - 1).ok_or(ProcessError::Permission)?;
                let target = slot
                    .pid
                    .as_ref()
                    .filter(|_| slot.roles[PGID] != 0)
                    .ok_or(ProcessError::Permission)?;
                let same_session =
                    slots
                        .iter()
                        .filter_map(|slot| slot.process.as_ref())
                        .any(|process| {
                            let relations = process.group.relations.lock();
                            Arc::ptr_eq(&relations.pgid, target)
                                && Arc::ptr_eq(&relations.sid, &caller_sid)
                        });
                if !same_session {
                    return Err(ProcessError::Permission);
                }
                target.clone()
            };
            let old_pgid = group.pgid_object();
            if Arc::ptr_eq(&old_pgid, &new_pgid) {
                return Ok(());
            }

            // Step 3: 更新 PGID 角色引用计数，并检查原进程组与新进程组是否因此新变为含停止作业的孤儿进程组。
            let old_orphan = orphaned_process_group(&slots, &old_pgid);
            let new_orphan = orphaned_process_group(&slots, &new_pgid);
            attach_role(&mut slots, &new_pgid, PGID);
            let old = core::mem::replace(&mut group.relations.lock().pgid, new_pgid.clone());
            let released = detach_role(&mut slots, &old, PGID);
            notify_new_orphan_locked(&mut slots, &old_pgid, old_orphan);
            notify_new_orphan_locked(&mut slots, &new_pgid, new_orphan);
            (old, released)
        };

        drop(released);
        self.flush_group_notifications();
        Ok(())
    }

    /// 为非进程组组长的调用进程创建新会话与同名新进程组，清除其控制终端关联，并检查原进程组及失去父会话连接的直属子进程组是否变为孤儿组（对应 `setsid`）。
    pub(crate) fn create_session(&self, caller: &UserIdentity) -> Result<usize, ProcessError> {
        let released = {
            let mut slots = self.slots.lock();
            self.live_parent(&slots, &caller.group)
                .map_err(|_| ProcessError::NoProcess)?;
            let group = &caller.group;
            let old_pgid = group.pgid_object();
            let old_orphan = orphaned_process_group(&slots, &old_pgid);
            let mut relations = group.relations.lock();
            if Arc::ptr_eq(&relations.sid, &group.pid) || slots[caller.tgid() - 1].roles[PGID] != 0
            {
                return Err(ProcessError::Permission);
            }

            attach_role(&mut slots, &group.pid, PGID);
            attach_role(&mut slots, &group.pid, SID);
            let old_pgid = core::mem::replace(&mut relations.pgid, group.pid.clone());
            let old_sid = core::mem::replace(&mut relations.sid, group.pid.clone());
            relations.controlling_tty = None;
            drop(relations);
            let pgid = detach_role(&mut slots, &old_pgid, PGID);
            let sid = detach_role(&mut slots, &old_sid, SID);
            notify_new_orphan_locked(&mut slots, &old_pgid, old_orphan);
            notify_unlinked_children_locked(&mut slots, caller.tgid(), &old_pgid, &old_sid);
            (old_pgid, old_sid, pgid, sid)
        };

        drop(released);
        self.flush_group_notifications();
        Ok(caller.tgid())
    }

    /// 按 `kill` 系统调用语义向单个进程（`pid > 0`）、调用者所在进程组（`pid == 0`）、除 `init` 与自身外的全部进程（`pid == -1`）或指定进程组（`pid < -1`）发送信号 `signum`（`signum == 0` 仅校验目标存在性与发送权限）。
    pub(crate) fn send_signal(
        &self,
        sender: &UserIdentity,
        pid: i32,
        signum: u8,
    ) -> Result<(), SignalError> {
        if SIGNAL_COUNT < signum as usize {
            return Err(SignalError::InvalidSignal);
        }
        let info = SignalInfo {
            signum,
            errno: 0,
            code: 0,
            payload: SignalPayload::Sender {
                pid: sender.tgid(),
                uid: sender.group.uid,
                value: 0,
            },
        };

        let capacity = self.slots.lock().len();
        let mut found = false;
        let mut delivered = false;
        let mut error = SignalError::NoProcess;
        for index in 0..capacity {
            let group = {
                let slots = self.slots.lock();
                slots[index]
                    .process
                    .as_ref()
                    .filter(|process| {
                        let target = &process.group;
                        match pid {
                            1.. => target.tgid() == pid as usize,
                            0 => target.pgid() == sender.group.pgid(),
                            -1 => !target.is_system_init() && target.tgid() != sender.tgid(),
                            _ => target.pgid() == pid.unsigned_abs() as usize,
                        }
                    })
                    .map(|process| process.group.clone())
            };
            let Some(group) = group else {
                continue;
            };
            found = true;
            if !signal_permission(sender, &group, signum) {
                error = SignalError::Permission;
                continue;
            }
            if signum == 0 {
                delivered = true;
                continue;
            }
            match self.deliver_group(&group, info, false) {
                Ok(()) => delivered = true,
                Err(failure) => error = failure,
            }
        }

        if delivered {
            Ok(())
        } else if found {
            Err(error)
        } else {
            Err(SignalError::NoProcess)
        }
    }

    /// 按 `tkill`（`tgid == None`）或 `tgkill`（`tgid == Some(...)`）语义向指定 `tid` 线程发送携带 `SI_TKILL` 标识的线程级信号。
    pub(crate) fn send_thread_signal(
        &self,
        sender: &UserIdentity,
        tid: usize,
        tgid: Option<usize>,
        signum: u8,
    ) -> Result<(), SignalError> {
        if tid == 0 || tgid == Some(0) || SIGNAL_COUNT < signum as usize {
            return Err(SignalError::InvalidSignal);
        }
        let info = SignalInfo {
            signum,
            errno: 0,
            code: SI_TKILL,
            payload: SignalPayload::Sender {
                pid: sender.tgid(),
                uid: sender.group.uid,
                value: 0,
            },
        };
        self.deliver_thread(sender, tid, tgid, info, false, None)
    }

    /// 按 `rt_sigqueueinfo` 语义向指定进程 `pid` 发送携带自定义 `siginfo_t` 载荷的排队信号（禁止向其他进程伪造内核保留的非负 `si_code` 或 `SI_TKILL`）。
    pub(crate) fn send_queue(
        &self,
        sender: &UserIdentity,
        pid: usize,
        info: SignalInfo,
    ) -> Result<(), SignalError> {
        if pid == 0 || SIGNAL_COUNT < info.signum as usize {
            return Err(SignalError::InvalidSignal);
        }
        if (0 <= info.code || info.code == SI_TKILL) && sender.tid() != pid {
            return Err(SignalError::Permission);
        }
        let group = self.group_target(pid)?;
        if !signal_permission(sender, &group, info.signum) {
            return Err(SignalError::Permission);
        }
        if info.signum == 0 {
            return Ok(());
        }
        self.deliver_group(&group, info, true)
    }

    /// 按 `rt_tgsigqueueinfo` 语义校验目标线程组 `tgid` 与成员 `tid`，并将自定义载荷信号挂入该线程私有 FIFO 队列（配额耗尽时返回 [`SignalError::QueueFull`]）。
    pub(crate) fn send_thread_queue(
        &self,
        sender: &UserIdentity,
        tid: usize,
        tgid: usize,
        info: SignalInfo,
    ) -> Result<(), SignalError> {
        if tid == 0 || tgid == 0 || SIGNAL_COUNT < info.signum as usize {
            return Err(SignalError::InvalidSignal);
        }
        if (0 <= info.code || info.code == SI_TKILL) && sender.tid() != tid {
            return Err(SignalError::Permission);
        }
        self.deliver_thread(sender, tid, Some(tgid), info, true, None)
    }

    /// 由内核测试或控制路径按数字 `pid` 向目标进程组投递携带 `SI_KERNEL` 的标准信号。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "用户信号、futex 重启与停止测试由内核控制者递送真实组信号"
        )
    )]
    pub(crate) fn send_kernel(&self, pid: usize, signum: u8) -> Result<(), SignalError> {
        if signum == 0 || SIGNAL_COUNT < signum as usize {
            return Err(SignalError::InvalidSignal);
        }
        let group = self.group_target(pid)?;
        self.deliver_group(&group, SignalInfo::kernel(signum), false)
    }

    /// 将当前线程运行期间同步触发的信号（如 `SIGPIPE`）精确投递到本线程私有的挂起信号队列中。
    pub(crate) fn send_kernel_thread(
        &self,
        identity: &UserIdentity,
        signum: u8,
    ) -> Result<(), SignalError> {
        if signum == 0 || SIGNAL_COUNT < signum as usize {
            return Err(SignalError::InvalidSignal);
        }
        let info = SignalInfo {
            signum,
            errno: 0,
            code: 0,
            payload: SignalPayload::Sender {
                pid: identity.tgid(),
                uid: identity.group.uid,
                value: 0,
            },
        };
        self.deliver_thread(
            identity,
            identity.tid(),
            Some(identity.tgid()),
            info,
            false,
            Some(&identity.signals),
        )
    }

    /// 在持有 `slots` 锁期间按 `pid` 编号克隆对应的 [`ThreadGroup`] 引用；若槽位无进程表项则返回 [`SignalError::NoProcess`]。
    fn group_target(&self, pid: usize) -> Result<Arc<ThreadGroup>, SignalError> {
        let slots = self.slots.lock();
        slots
            .get(pid.checked_sub(1).ok_or(SignalError::NoProcess)?)
            .and_then(|slot| slot.process.as_ref())
            .map(|process| process.group.clone())
            .ok_or(SignalError::NoProcess)
    }

    /// 在锁外准备信号队列节点，随后按 `slots` ➔ 信号锁顺序复查 `group` 仍驻留在槽位中并提交组级信号，最后在锁外唤醒线程及通知父进程。
    fn deliver_group(
        &self,
        group: &Arc<ThreadGroup>,
        info: SignalInfo,
        queued: bool,
    ) -> Result<(), SignalError> {
        // Step 1: 在自旋锁外预分配排队信号节点（内核普通信号无需堆分配）。
        let mut prepared = if queued {
            PreparedSignal::user(info)?
        } else {
            PreparedSignal::kernel()
        };

        // Step 2: 在图锁内核对原 ThreadGroup 未被回收复用，投递信号并记录可能的 SIGCONT 恢复事件。
        let result = (|| {
            let mut slots = self.slots.lock();
            let actual = slots[group.tgid() - 1]
                .process
                .as_ref()
                .ok_or(SignalError::NoProcess)?;
            if !Arc::ptr_eq(&actual.group, group) {
                return Err(SignalError::NoProcess);
            }
            let continued = group.signals.deliver(
                info,
                None,
                &mut prepared,
                Some(group),
                slots
                    .iter()
                    .filter_map(|slot| slot.thread.as_ref())
                    .filter(|thread| Arc::ptr_eq(&thread.group, group))
                    .map(|thread| &thread.signals),
            )?;
            let event = if continued {
                let event = child_event(group, CLD_CONTINUED, SIGCONT as i32);
                slots[group.tgid() - 1]
                    .process
                    .as_mut()
                    .expect("continuing group disappeared")
                    .control_event = Some(event);
                Some(event)
            } else {
                None
            };
            Ok(event)
        })();

        // Step 3: 在所有自旋锁外唤醒目标线程组及等待子进程状态事件的父进程；未被消费的预备节点随作用域自动释放退费。
        group.signals.notify();
        if let Some(event) = result? {
            self.notify_parent(group, event);
            self.waiters.wake_all();
        }
        Ok(())
    }

    /// 供间隔定时器到期回调或父子信号路径通过 `Weak<TaskGraph>` 投递组级信号，额外核对 `source` 指针防止 `PID` 复用后误发给新进程。
    pub(super) fn send_group_info(
        &self,
        pid: usize,
        source: &SignalGroup,
        info: SignalInfo,
    ) -> Result<(), SignalError> {
        let group = self.group_target(pid)?;
        if !ptr::eq(group.signals.as_ref(), source) {
            return Err(SignalError::NoProcess);
        }
        self.deliver_group(&group, info, false)
    }

    /// 向指定线程投递信号的核心实现：先加锁快照目标对象并校验权限，锁外分配节点后再重新加锁核对原线程对象提交信号，最后在锁外唤醒目标线程。
    fn deliver_thread(
        &self,
        sender: &UserIdentity,
        tid: usize,
        tgid: Option<usize>,
        info: SignalInfo,
        queued: bool,
        expected_private: Option<&Arc<ThreadSignals>>,
    ) -> Result<(), SignalError> {
        // Step 1: 首次获取图锁查找目标线程、校验 tgid/私有队列身份与发送权限（signum == 0 探测到此直接返回）。
        let (group, signals, user) = {
            let slots = self.slots.lock();
            let thread = slots
                .get(tid.checked_sub(1).ok_or(SignalError::NoProcess)?)
                .and_then(|slot| slot.thread.as_ref())
                .ok_or(SignalError::NoProcess)?;
            if tgid.is_some_and(|tgid| thread.group.tgid() != tgid)
                || expected_private.is_some_and(|expected| !Arc::ptr_eq(expected, &thread.signals))
            {
                return Err(SignalError::NoProcess);
            }
            if !signal_permission(sender, &thread.group, info.signum) {
                return Err(SignalError::Permission);
            }
            if info.signum == 0 {
                return Ok(());
            }
            (
                thread.group.clone(),
                thread.signals.clone(),
                thread.user.clone(),
            )
        };

        // Step 2: 在自旋锁外预分配用户排队信号节点。
        let mut prepared = if queued {
            PreparedSignal::user(info)?
        } else {
            PreparedSignal::kernel()
        };

        // Step 3: 重新加锁核对原线程与原线程组对象未在锁外窗口内退出或被同号复用，随后完成信号入队。
        let result = (|| {
            let mut slots = self.slots.lock();
            let actual = slots
                .get(tid - 1)
                .and_then(|slot| slot.thread.as_ref())
                .ok_or(SignalError::NoProcess)?;
            if !Arc::ptr_eq(&actual.group, &group) || !Arc::ptr_eq(&actual.signals, &signals) {
                return Err(SignalError::NoProcess);
            }
            if !signal_permission(sender, &group, info.signum) {
                return Err(SignalError::Permission);
            }
            let continued = group.signals.deliver(
                info,
                Some(&signals),
                &mut prepared,
                Some(&group),
                slots
                    .iter()
                    .filter_map(|slot| slot.thread.as_ref())
                    .filter(|thread| Arc::ptr_eq(&thread.group, &group))
                    .map(|thread| &thread.signals),
            )?;
            let event = if continued {
                let event = child_event(&group, CLD_CONTINUED, SIGCONT as i32);
                slots[group.tgid() - 1]
                    .process
                    .as_mut()
                    .expect("continuing group disappeared")
                    .control_event = Some(event);
                Some(event)
            } else {
                None
            };
            Ok(event)
        })();

        // Step 4: 在自旋锁外唤醒目标线程等待器、发送跨核重调度请求（kick）并通知父进程。
        signals.notify_wait();
        group.signals.notify();
        if let Some(user) = user.upgrade() {
            user.kick();
        }
        if let Some(event) = result? {
            self.notify_parent(&group, event);
            self.waiters.wake_all();
        }
        Ok(())
    }

    /// 查询或替换调用者所在线程组对 `signal` 的处理动作（对应 `rt_sigaction`）；若新动作为忽略且按规范需丢弃挂起信号，在同一事务内一并清理组队列与全部成员线程的私有队列。
    pub(crate) fn update_action(
        &self,
        identity: &UserIdentity,
        signal: usize,
        action: Option<SignalAction>,
    ) -> Result<SignalAction, SignalError> {
        let previous = {
            let slots = self.slots.lock();
            let process = slots[identity.tgid() - 1]
                .process
                .as_ref()
                .ok_or(SignalError::NoProcess)?;
            if !Arc::ptr_eq(&process.group, &identity.group) {
                return Err(SignalError::NoProcess);
            }
            identity.group.signals.update_action(
                signal,
                action,
                slots
                    .iter()
                    .filter_map(|slot| slot.thread.as_ref())
                    .filter(|thread| Arc::ptr_eq(&thread.group, &identity.group))
                    .map(|thread| &thread.signals),
            )?
        };

        identity.group.signals.notify();
        Ok(previous)
    }

    /// 按槽位上界逐个克隆属于原 `source` 信号组的存活线程引用，并在释放图锁后唤醒等待器与发送跨核打断请求（无需分配临时 `Vec`）。
    pub(super) fn notify_group(&self, tgid: usize, source: &SignalGroup) {
        let capacity = self.slots.lock().len();
        for index in 0..capacity {
            let member = {
                let slots = self.slots.lock();
                slots[index]
                    .thread
                    .as_ref()
                    .filter(|thread| {
                        thread.group.tgid() == tgid
                            && ptr::eq(thread.group.signals.as_ref(), source)
                    })
                    .map(|thread| (thread.signals.clone(), thread.user.clone()))
            };

            if let Some((signals, user)) = member {
                signals.notify_wait();
                if let Some(user) = user.upgrade() {
                    user.kick();
                }
            }
        }
        self.control_waiters.wake_all();
    }

    /// 在图锁内快照子进程 `group` 当前的存活父进程引用，随后在锁外向父进程发送 `SIGCHLD` 控制状态通知。
    fn notify_parent(&self, group: &ThreadGroup, event: ChildEvent) {
        let parent = {
            let slots = self.slots.lock();
            let process = slots[group.tgid() - 1].process.as_ref();
            process
                .filter(|process| ptr::eq(process.group.as_ref(), group))
                .and_then(|process| process.parent)
                .and_then(|pid| slots[pid - 1].process.as_ref())
                .map(|process| process.group.clone())
        };

        send_parent_event(parent.as_deref(), event);
    }

    /// 在持有 `slots` 锁的前提下确认 `parent` 属于本图、仍驻留在对应槽位、尚有存活成员且未收到组退出请求，成功则返回其 `TGID`。
    fn live_parent(&self, slots: &Numbers, parent: &ThreadGroup) -> Result<usize, ProcessError> {
        if !ptr::eq(self, parent.graph.as_ptr()) {
            return Err(ProcessError::InvalidParent);
        }
        let slot = slots
            .get(parent.tgid() - 1)
            .ok_or(ProcessError::InvalidParent)?;
        let process = slot.process.as_ref().ok_or(ProcessError::InvalidParent)?;
        if !ptr::eq(process.group.as_ref(), parent)
            || process.exit.is_some()
            || process.members == 0
            || process.group.signals.exit_request().is_some()
        {
            return Err(ProcessError::InvalidParent);
        }
        Ok(parent.tgid())
    }
}

impl PreparedIdentity {
    /// 克隆并返回待发布身份包内部的 [`UserIdentity`] 引用（供上层在调用 `publish` 前构造 `UserThread` 与任务入口闭包）。
    pub(crate) fn identity(&self) -> Arc<UserIdentity> {
        self.identity.clone()
    }

    /// 在同一 `slots` 锁事务内复查任务图运行态、父进程存活状态与 `execve` 互斥边界，将预留槽位正式发布为存活线程（及新进程组首），并转移 `NPROC` 计费凭据。
    pub(crate) fn publish(
        mut self,
        user: Weak<UserThread>,
        execution: Box<PreparedExecution>,
    ) -> Result<(), ProcessError> {
        let graph = self.reservation.graph.clone();
        let mut old_relations = None;
        {
            // Step 1: 在关系锁内复查任务图未关闭，校验父进程（新进程）或目标线程组（新线程）仍然存活且未在执行 execve，并同步继承父进程最新的 PGID/SID/控制终端。
            let mut slots = graph.slots.lock();
            if graph.closing.load(Ordering::Relaxed) != RUNNING {
                return Err(ProcessError::Busy);
            }
            let leader = self.identity.tid() == self.identity.tgid();
            let parent = if leader {
                match self.parent.as_ref() {
                    Some(parent) => {
                        let number = graph.live_parent(&slots, parent)?;
                        let inherited = parent.relations.lock();
                        let mut relations = self.identity.group.relations.lock();
                        relations.controlling_tty = inherited.controlling_tty;
                        old_relations = Some((
                            core::mem::replace(&mut relations.pgid, inherited.pgid.clone()),
                            core::mem::replace(&mut relations.sid, inherited.sid.clone()),
                        ));
                        Some(number)
                    }
                    None => {
                        if graph.root_role == RootRole::System
                            && (self.reservation.index != 0
                                || self.identity.tgid() != 1
                                || !self.identity.group.is_system_init()
                                || slots[0].process.is_some())
                        {
                            return Err(ProcessError::InvalidParent);
                        }
                        None
                    }
                }
            } else {
                graph.live_parent(&slots, &self.identity.group)?;
                if slots[self.identity.tgid() - 1]
                    .process
                    .as_ref()
                    .expect("live group disappeared")
                    .exec_owner
                    .is_some()
                {
                    return Err(ProcessError::Busy);
                }
                None
            };

            // Step 2: 解除预留标志，绑定 Pid 身份对象、增加四类角色计数并写入 ThreadEntry / ProcessEntry。
            let index = self.reservation.index;
            assert!(slots[index].reserved, "PID reservation disappeared");
            assert!(slots[index].pid.is_none(), "reserved PID already published");
            assert!(
                execution.next.is_none(),
                "prepared member is already linked"
            );

            let task_id = execution.join.task_id();
            slots[index].reserved = false;
            let tid = self.identity.tid_object();
            slots[index].pid = Some(tid.clone());
            attach_role(&mut slots, &tid, TID);
            slots[index].thread = Some(ThreadEntry {
                user,
                group: self.identity.group.clone(),
                execution,
                signals: self.identity.signals.clone(),
                charge: self.charge.take().expect("identity has no NPROC charge"),
            });
            self.identity.group.signals.add_member();
            if leader {
                attach_role(&mut slots, &self.identity.group.pid, TGID);
                let relations = self.identity.group.relations.lock();
                attach_role(&mut slots, &relations.pgid, PGID);
                attach_role(&mut slots, &relations.sid, SID);
                slots[index].process = Some(ProcessEntry {
                    group: self.identity.group.clone(),
                    parent,
                    members: 1,
                    leader_task_id: task_id,
                    leader_status: None,
                    exec_owner: None,
                    did_exec: false,
                    user_interrupts: 0,
                    user_timer_interrupts: 0,
                    completed: None,
                    exit: None,
                    control_event: None,
                    notification: None,
                    leader_charge: None,
                });
            } else {
                let process = slots[self.identity.tgid() - 1]
                    .process
                    .as_mut()
                    .expect("thread published without a process");
                process.members = process
                    .members
                    .checked_add(1)
                    .expect("group member overflow");
            }
            self.reservation.active = false;
        }

        // Step 3: 在自旋锁外释放继承关系替换下来的旧 Pid 引用。
        drop(old_relations);
        Ok(())
    }
}

impl Drop for NumberReservation {
    /// 若预留号码尚未通过 `PreparedIdentity::publish` 正式发布，在析构时加锁清除 `reserved` 标志并将槽位重新归还空闲位图。
    fn drop(&mut self) {
        if self.active {
            let mut slots = self.graph.slots.lock();
            let slot = &mut slots[self.index];
            assert!(
                slot.reserved && slot.pid.is_none(),
                "invalid private PID reservation"
            );
            slot.reserved = false;
            slots.set_free(self.index, true);
        }
    }
}

/// 为槽位中已绑定的 `pid` 身份对象递增指定角色 `role`（`TID`/`TGID`/`PGID`/`SID`）的活跃引用计数。
fn attach_role(slots: &mut Numbers, pid: &Arc<Pid>, role: usize) {
    let slot = &mut slots[pid.number - 1];
    assert!(
        slot.pid
            .as_ref()
            .is_some_and(|entry| Arc::ptr_eq(entry, pid)),
        "attaching a stale PID role"
    );
    slot.roles[role] = slot.roles[role]
        .checked_add(1)
        .expect("PID role count overflow");
}

/// 递减 `pid` 身份对象的指定角色 `role` 计数；当四个角色计数全部归零时，解除槽位绑定、置位空闲位图并返回 `Arc<Pid>` 供调用者在锁外析构。
fn detach_role(slots: &mut Numbers, pid: &Arc<Pid>, role: usize) -> Option<Arc<Pid>> {
    let slot = &mut slots[pid.number - 1];
    assert!(
        slot.pid
            .as_ref()
            .is_some_and(|entry| Arc::ptr_eq(entry, pid)),
        "detaching a stale PID role"
    );
    slot.roles[role] = slot.roles[role]
        .checked_sub(1)
        .expect("PID role count underflow");
    if slot.roles == [0; 4] {
        assert!(
            !slot.reserved && slot.thread.is_none() && slot.process.is_none(),
            "freeing a PID with attached identities"
        );
        let released = slot.pid.take();
        slots.set_free(pid.number - 1, true);
        released
    } else {
        None
    }
}

/// 判断 `sender` 是否有权向 `target` 发送信号 `signum`：`root`（`uid == 0`）或相同真实 `uid` 总是允许，`SIGCONT` 额外允许同一会话内的进程发送。
fn signal_permission(sender: &UserIdentity, target: &ThreadGroup, signum: u8) -> bool {
    sender.group.uid == 0
        || sender.group.uid == target.uid
        || (signum == SIGCONT && sender.group.sid() == target.sid())
}

/// 将 Linux `wait4` 格式的 16 位编码状态字 `status` 解码为 `waitid` / `SIGCHLD` 使用的 `CLD_*` 原因码与原始退出码/信号号。
fn exit_child_event(group: &ThreadGroup, status: i32) -> ChildEvent {
    let signal = status & WAIT_SIGNAL_MASK;
    child_event(
        group,
        if signal == 0 {
            CLD_EXITED
        } else if status & WAIT_CORE_DUMP != 0 {
            CLD_DUMPED
        } else {
            CLD_KILLED
        },
        if signal == 0 {
            (status >> EXIT_STATUS_SHIFT) & EXIT_STATUS_MASK
        } else {
            signal
        },
    )
}

/// 采集 `group` 自身及其已回收子代的累计用户态/内核态 CPU 时间快照，构造 [`ChildEvent`]。
fn child_event(group: &ThreadGroup, code: i32, status: i32) -> ChildEvent {
    let own = group.time.snapshot();
    let descendants = group.time.children();
    ChildEvent {
        pid: group.tgid(),
        uid: group.uid,
        code,
        status,
        user_ns: own
            .user_ns
            .checked_add(descendants.user_ns)
            .expect("child user time overflow"),
        kernel_ns: own
            .kernel_ns
            .checked_add(descendants.kernel_ns)
            .expect("child kernel time overflow"),
    }
}

/// 非阻塞地遍历 `completed` 链表，将已确认切离内核栈（`try_join().is_some()`）的节点摘除并移入 `discarded` 链表留待锁外销毁。
fn discard_completed(
    completed: &mut Option<Box<PreparedExecution>>,
    discarded: &mut Option<Box<PreparedExecution>>,
) {
    let mut link = completed;
    while let Some(execution) = link.as_ref() {
        if execution.join.try_join().is_some() {
            let mut execution = link.take().expect("completed member disappeared");
            *link = execution.next.take();
            execution.next = discarded.take();
            *discarded = Some(execution);
        } else {
            link = &mut link.as_mut().expect("member disappeared").next;
        }
    }
}

/// 从槽位中移除已退出的僵尸 `ProcessEntry` 并解绑其 `TGID`、`PGID`、`SID` 角色：`kernel_join == false` 时将未切离内核栈的观察节点挂回 `Slot::completed`，`true` 时保留在返回结果中供内核协调器在锁外 `join`。
fn reap_child(slots: &mut Numbers, index: usize, kernel_join: bool) -> CollectedChild {
    let mut retired = slots[index]
        .process
        .take()
        .expect("wait process disappeared");
    assert_eq!(retired.members, 0, "reaping a live group");
    assert!(slots[index].thread.is_none(), "reaping a live leader");
    let event = retired.exit.expect("reaping without exit event");
    if !kernel_join {
        while let Some(mut execution) = retired.completed.take() {
            retired.completed = execution.next.take();
            execution.next = slots[index].completed.take();
            slots[index].completed = Some(execution);
        }
    }

    let notification_parent = retired
        .parent
        .filter(|_| matches!(retired.notification, Some(GroupNotification::Continued(_))))
        .and_then(|number| slots[number - 1].process.as_ref())
        .map(|parent| parent.group.clone());
    let released = {
        let relations = retired.group.relations.lock();
        [
            detach_role(slots, &retired.group.pid, TGID),
            detach_role(slots, &relations.pgid, PGID),
            detach_role(slots, &relations.sid, SID),
        ]
    };

    CollectedChild {
        event,
        process: Some(retired),
        notification_parent,
        _released: released,
    }
}

/// 向父进程组投递子进程停止或继续的 `SIGCHLD` 信号（内部由信号层根据 `SA_NOCLDSTOP` 决定是否实际入队）。
fn send_parent_event(parent: Option<&ThreadGroup>, event: ChildEvent) {
    if let Some(parent) = parent {
        parent.signals.send_child(child_signal_info(SIGCHLD, event));
    }
}

/// 将 [`ChildEvent`] 转换为信号投递所需的 [`SignalInfo`]，并将纳秒级 CPU 时间按公开 `USER_HZ` 换算为时钟滴答数（ticks）。
fn child_signal_info(signum: u8, event: ChildEvent) -> SignalInfo {
    SignalInfo {
        signum,
        errno: 0,
        code: event.code,
        payload: SignalPayload::Child {
            pid: event.pid,
            uid: event.uid,
            status: event.status,
            user_ticks: event.user_ns / (NSEC_PER_SEC / USER_HZ),
            kernel_ticks: event.kernel_ns / (NSEC_PER_SEC / USER_HZ),
        },
    }
}

/// 检查 `pid` 对应的槽位当前是否仍绑定同一个 `Arc<Pid>` 实例且在指定 `role` 上具有非零活跃计数。
fn has_role(slots: &Numbers, pid: &Arc<Pid>, role: usize) -> bool {
    slots.get(pid.number - 1).is_some_and(|slot| {
        slot.roles[role] != 0
            && slot
                .pid
                .as_ref()
                .is_some_and(|actual| Arc::ptr_eq(actual, pid))
    })
}

/// 查询 `pid` 所标识的进程组所属的会话 `SID`：优先从具有该 `PGID` 的存活进程读取会话，若无存活进程组成员则回退检查同对象的存活 `TID` 线程。
fn session_of_pgrp(slots: &Numbers, pid: &Arc<Pid>) -> Option<Arc<Pid>> {
    if has_role(slots, pid, PGID) {
        for process in slots.iter().filter_map(|slot| slot.process.as_ref()) {
            if 0 < process.members {
                let relations = process.group.relations.lock();
                if Arc::ptr_eq(&relations.pgid, pid) {
                    return Some(relations.sid.clone());
                }
            }
        }
    }
    if has_role(slots, pid, TID) {
        return slots[pid.number - 1]
            .thread
            .as_ref()
            .map(|thread| thread.group.sid_object());
    }
    None
}

/// 在持有 `slots` 锁的前提下核对 `tty` 当前仍绑定于 `graph` 的 `session` 会话，清除该会话所有进程的 `controlling_tty` 指针并取走端点内的 [`TerminalControl`]。
fn release_terminal_locked(
    slots: &Numbers,
    graph: &TaskGraph,
    tty: &'static tty::Endpoint,
    session: &Arc<Pid>,
) -> Option<TerminalControl> {
    let matches = tty.control().lock().as_ref().is_some_and(|control| {
        ptr::eq(control.graph.as_ptr(), graph) && Arc::ptr_eq(&control.session, session)
    });
    if !matches {
        return None;
    }

    for process in slots.iter().filter_map(|slot| slot.process.as_ref()) {
        let mut relations = process.group.relations.lock();
        if Arc::ptr_eq(&relations.sid, session)
            && relations
                .controlling_tty
                .is_some_and(|bound| ptr::eq(bound, tty))
        {
            relations.controlling_tty = None;
        }
    }

    tty.control().lock().take()
}

/// 按 POSIX 孤儿进程组定义判定 `pgid` 是否为孤儿组：当组内不存在任何“其父进程存活、非系统 `init`、且与子进程同会话但不同进程组”的存活进程时，该进程组即为孤儿组。
fn orphaned_process_group(slots: &Numbers, pgid: &Arc<Pid>) -> bool {
    !slots
        .iter()
        .filter_map(|slot| slot.process.as_ref())
        .any(|process| {
            if process.members == 0 {
                return false;
            }
            let relations = process.group.relations.lock();
            if !Arc::ptr_eq(&relations.pgid, pgid) {
                return false;
            }
            let parent = process
                .parent
                .and_then(|number| slots[number - 1].process.as_ref());
            parent.is_some_and(|parent| {
                if parent.members == 0 || parent.group.is_system_init() {
                    return false;
                }
                let parent_relations = parent.group.relations.lock();
                !Arc::ptr_eq(&parent_relations.pgid, pgid)
                    && Arc::ptr_eq(&parent_relations.sid, &relations.sid)
            })
        })
}

/// 在持有 `slots` 锁期间向进程组 `pgid` 内所有存活进程投递内核信号 `signum`，并将唤醒与 `CLD_CONTINUED` 通知登记到各进程的 `notification` 字段留待锁外派发。
fn deliver_process_group_locked(
    slots: &mut Numbers,
    pgid: &Arc<Pid>,
    signum: u8,
) -> Result<(), SignalError> {
    if !has_role(slots, pgid, PGID) {
        return Err(SignalError::NoProcess);
    }
    let mut found = false;
    for index in 0..slots.len() {
        let Some(process) = slots[index].process.as_ref().filter(|process| {
            0 < process.members && Arc::ptr_eq(&process.group.relations.lock().pgid, pgid)
        }) else {
            continue;
        };
        found = true;
        let group = &process.group;
        let continued = group.signals.deliver(
            SignalInfo::kernel(signum),
            None,
            &mut PreparedSignal::kernel(),
            None,
            slots
                .iter()
                .filter_map(|slot| slot.thread.as_ref())
                .filter(|thread| Arc::ptr_eq(&thread.group, group))
                .map(|thread| &thread.signals),
        )?;
        let event = continued.then(|| child_event(group, CLD_CONTINUED, SIGCONT as i32));
        let process = slots[index]
            .process
            .as_mut()
            .expect("signalled process disappeared");
        if let Some(event) = event {
            process.control_event = Some(event);
            process.notification = Some(GroupNotification::Continued(event));
        } else if process.notification.is_none() {
            process.notification = Some(GroupNotification::Signal);
        }
    }
    if found {
        Ok(())
    } else {
        Err(SignalError::NoProcess)
    }
}

/// 当进程组 `pgid` 在本次关系变更中刚从非孤儿变为孤儿（`!was_orphan`）且组内包含已进入停止态的进程时，按 POSIX 规范依次投递 `SIGHUP` 与 `SIGCONT`。
fn notify_new_orphan_locked(slots: &mut Numbers, pgid: &Arc<Pid>, was_orphan: bool) {
    if was_orphan || !orphaned_process_group(slots, pgid) {
        return;
    }
    let stopped = slots
        .iter()
        .filter_map(|slot| slot.process.as_ref())
        .any(|process| {
            0 < process.members
                && Arc::ptr_eq(&process.group.relations.lock().pgid, pgid)
                && process.group.signals.has_stopped_jobs()
        });
    if stopped {
        deliver_process_group_locked(slots, pgid, SIGHUP).expect("orphan hangup delivery failed");
        deliver_process_group_locked(slots, pgid, SIGCONT)
            .expect("orphan continue delivery failed");
    }
}

/// 当进程 `parent` 退出或调用 `setsid` 脱离原会话 `old_sid` 时，检查其原本保持同会话跨组连接的全部存活直属子进程组是否因此新变为孤儿进程组。
fn notify_unlinked_children_locked(
    slots: &mut Numbers,
    parent: usize,
    old_pgid: &Arc<Pid>,
    old_sid: &Arc<Pid>,
) {
    for index in 0..slots.len() {
        let pgid = slots[index]
            .process
            .as_ref()
            .filter(|child| child.parent == Some(parent) && 0 < child.members)
            .and_then(|child| {
                let relations = child.group.relations.lock();
                (Arc::ptr_eq(&relations.sid, old_sid) && !Arc::ptr_eq(&relations.pgid, old_pgid))
                    .then(|| relations.pgid.clone())
            });
        if let Some(pgid) = pgid {
            notify_new_orphan_locked(slots, &pgid, false);
        }
    }
}
