//! 绑定锁串行动作表切换，动作锁保护处置与组、线程状态；共享处置不合并不同组pending。

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

pub(crate) const SIGNAL_COUNT: usize = 64;
pub(crate) const SIGHUP: u8 = 1;
pub(crate) const SIGINT: u8 = 2;
pub(crate) const SIGQUIT: u8 = 3;
pub(crate) const SIGILL: u8 = 4;
pub(crate) const SIGTRAP: u8 = 5;
#[expect(
    dead_code,
    reason = "Linux 通用 signal ABI 将 SIGABRT 固定为 6，保留命名以区分用户传入的终止信号编号"
)]
pub(crate) const SIGABRT: u8 = 6;
pub(crate) const SIGBUS: u8 = 7;
pub(crate) const SIGFPE: u8 = 8;
pub(crate) const SIGKILL: u8 = 9;
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "futex 中断与重启测试以 Linux SIGUSR1 递送用户处理器信号"
    )
)]
pub(crate) const SIGUSR1: u8 = 10;
pub(crate) const SIGSEGV: u8 = 11;
pub(crate) const SIGPIPE: u8 = 13;
pub(crate) const SIGALRM: u8 = 14;
pub(crate) const SIGCHLD: u8 = 17;
pub(crate) const SIGCONT: u8 = 18;
pub(crate) const SIGSTOP: u8 = 19;
pub(crate) const SIGTSTP: u8 = 20;
pub(crate) const SIGTTIN: u8 = 21;
pub(crate) const SIGTTOU: u8 = 22;
pub(crate) const SIGURG: u8 = 23;
pub(crate) const SIGVTALRM: u8 = 26;
pub(crate) const SIGPROF: u8 = 27;
pub(crate) const SIGWINCH: u8 = 28;
pub(crate) const SIGSYS: u8 = 31;
pub(crate) const SIGRTMIN: u8 = 32;
pub(crate) const SIGINFO_BYTES: usize = 128;
pub(crate) const SIGSET_BYTES: usize = core::mem::size_of::<u64>();
const SI_KERNEL: i32 = 128;
pub(crate) const SIG_DFL: usize = 0;
pub(crate) const SIG_IGN: usize = 1;
const SA_NOCLDSTOP: usize = 1;
pub(crate) const CLD_EXITED: i32 = 1;
pub(crate) const CLD_KILLED: i32 = 2;
pub(crate) const CLD_DUMPED: i32 = 3;
pub(crate) const CLD_STOPPED: i32 = 5;
pub(crate) const CLD_CONTINUED: i32 = 6;
pub(crate) const SI_TKILL: i32 = -6;
const SA_SIGINFO: usize = 4;
pub(crate) const UNBLOCKABLE: u64 = (1 << (SIGKILL - 1)) | (1 << (SIGSTOP - 1));
pub(crate) const SA_ONSTACK: usize = 0x0800_0000;
pub(crate) const SA_RESTART: usize = 0x1000_0000;
pub(crate) const SA_NODEFER: usize = 0x4000_0000;
pub(crate) const SA_RESETHAND: usize = 0x8000_0000;
pub(crate) const SS_ONSTACK: u32 = 1;
pub(crate) const SS_DISABLE: u32 = 2;
const SA_NOCLDWAIT: usize = 2;
const ACTION_FLAGS: usize =
    SA_NOCLDSTOP | SA_NOCLDWAIT | SA_SIGINFO | SA_ONSTACK | SA_RESTART | SA_NODEFER | SA_RESETHAND;
const STOP_MASK: u64 =
    (1 << (SIGSTOP - 1)) | (1 << (SIGTSTP - 1)) | (1 << (SIGTTIN - 1)) | (1 << (SIGTTOU - 1));

/// 两目标的动作对象没有restorer字段；内核保存用户handler地址但不解引用它。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SignalAction {
    pub(crate) handler: usize,
    pub(crate) flags: usize,
    pub(crate) mask: u64,
}

impl SignalAction {
    pub(crate) const DEFAULT: Self = Self {
        handler: 0,
        flags: 0,
        mask: 0,
    };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SignalError {
    InvalidSignal,
    InvalidStack,
    OnStack,
    StackTooSmall,
    QueueFull,
    NoMemory,
    NoProcess,
    Permission,
}

/// 现场事实仅随同步fault保存；siginfo只编码目标UAPI实际存在的字段。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SignalPayload {
    Sender {
        pid: usize,
        uid: u32,
        value: usize,
    },
    Fault {
        address: usize,
        pc: usize,
        trap: usize,
    },
    Child {
        pid: usize,
        uid: u32,
        status: i32,
        user_ticks: u64,
        kernel_ticks: u64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SignalInfo {
    pub(crate) signum: u8,
    pub(crate) errno: i32,
    pub(crate) code: i32,
    pub(crate) payload: SignalPayload,
}

impl SignalInfo {
    /// 内核生成的普通信号携带SI_KERNEL，不借用用户发送者身份。
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

    /// 编码64位两目标共同的128字节siginfo，所有padding与未使用union字节清零。
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct AltStack {
    pub(crate) base: usize,
    pub(crate) size: usize,
    pub(crate) flags: u32,
}

impl AltStack {
    pub(crate) const DISABLED: Self = Self {
        base: 0,
        size: 0,
        flags: SS_DISABLE,
    };

    /// 禁用位决定是否提供备用栈，不用地址为零推测启用状态。
    pub(crate) fn enabled(&self) -> bool {
        self.flags & SS_DISABLE == 0
    }

    /// 向下生长的栈包含初始top，排除已经耗尽的base。
    pub(crate) fn on_stack(&self, sp: usize) -> bool {
        self.enabled()
            && self.base < sp
            && self
                .base
                .checked_add(self.size)
                .is_some_and(|end| sp <= end)
    }

    /// 查询结果的ONSTACK取当前SP，内核保存值不固化动态栈状态。
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

    /// 验证实际支持的stack_t设置与地址范围；目标最小容量由调用者另行判断。
    pub(crate) fn validate(&self) -> Result<(), SignalError> {
        if !matches!(self.flags, 0 | SS_ONSTACK | SS_DISABLE)
            || (self.enabled() && self.base.checked_add(self.size).is_none())
        {
            return Err(SignalError::InvalidStack);
        }
        Ok(())
    }
}

pub(crate) struct SignalFrameArgs {
    pub(crate) info: SignalInfo,
    pub(crate) action: SignalAction,
    pub(crate) restore_mask: u64,
    pub(crate) altstack: AltStack,
    pub(crate) trampoline: usize,
}

pub(crate) struct SignalFrameRestore {
    pub(crate) context: UserContext,
    pub(crate) mask: u64,
    pub(crate) altstack: AltStack,
    pub(crate) pending_signal: Option<SignalInfo>,
}

/// clone只复制阻塞集与备用栈，pending与等待登记始终从空状态开始。
#[derive(Clone, Copy)]
pub(crate) struct SignalInheritance {
    pub(crate) mask: u64,
    pub(crate) altstack: AltStack,
}

impl SignalInheritance {
    pub(crate) const EMPTY: Self = Self {
        mask: 0,
        altstack: AltStack::DISABLED,
    };
}

/// 绑定锁先于动作siglock、组和private短锁，选择与消费在同一事务中。
pub(crate) struct Sighand {
    actions: SpinLock<[SignalAction; SIGNAL_COUNT]>,
    users: AtomicUsize,
}

impl Sighand {
    /// 锁外构造完整默认处置，尚未发表到可运行执行者。
    pub(crate) fn try_new() -> Result<Arc<Self>, AllocError> {
        Arc::try_new(Self {
            actions: SpinLock::new([SignalAction::DEFAULT; SIGNAL_COUNT]),
            users: AtomicUsize::new(0),
        })
    }

    /// fork复制处置快照，新表分配不占原siglock。
    pub(crate) fn try_copy(&self) -> Result<Arc<Self>, AllocError> {
        let actions = *self.actions.lock();
        Arc::try_new(Self {
            actions: SpinLock::new(actions),
            users: AtomicUsize::new(0),
        })
    }

    /// 增加真实运行绑定责任，诊断引用与组的动作表拥有不参与计数。
    pub(crate) fn add_user(&self) {
        self.users
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .expect("sighand user count overflow");
    }

    /// 归还真实运行绑定，动作表最后引用仍由调用者在短锁外释放。
    pub(crate) fn remove_user(&self) {
        let previous = self.users.fetch_sub(1, Ordering::AcqRel);
        assert!(previous != 0, "sighand user count underflow");
    }

    /// 返回真实运行绑定数，不把普通拥有快照误判为外部共享者。
    pub(crate) fn user_count(&self) -> usize {
        self.users.load(Ordering::Acquire)
    }
}

struct GroupState {
    pending: Pending,
    members: usize,
    stopped: usize,
    stop_signal: Option<u8>,
    stop_reported: bool,
    exit: Option<i32>,
}

/// 每个真实线程组独有pending与组控制，绑定锁保护当前动作表的完整事务。
pub(crate) struct SignalGroup {
    graph: Weak<TaskGraph>,
    tgid: usize,
    system_init: bool,
    sighand: SpinLock<Arc<Sighand>>,
    state: SpinLock<GroupState>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SignalControl {
    Run,
    Stop,
    Exit(i32),
}

pub(crate) enum SignalSelection {
    Handler {
        info: SignalInfo,
        action: SignalAction,
        restore_mask: u64,
        altstack: AltStack,
    },
    Stop,
    Exit(i32),
}

impl SignalGroup {
    /// 身份发表前固定系统首根角色并绑定完整动作表，弱图关联仅供锁外通知。
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

    /// 不可变角色随真实信号组共享，exec和线程成员变化都不改变首根职责。
    pub(crate) fn is_system_init(&self) -> bool {
        self.system_init
    }

    /// IRQ普通定时信号无分配，发送与通知在siglock两侧，调用者不得已持siglock。
    pub(crate) fn send_timer(&self, signum: u8) {
        assert!(matches!(signum, SIGALRM | SIGVTALRM | SIGPROF));
        self.send(SignalInfo::kernel(signum))
            .expect("ordinary timer signal failed");
    }

    /// 同一真实图发送事务保护处置、pending和跨成员控制，通知仍由锁外边界完成。
    pub(crate) fn send(&self, info: SignalInfo) -> Result<(), SignalError> {
        signal_bit(info.signum)?;
        let graph = self.graph.upgrade().ok_or(SignalError::NoProcess)?;
        graph.send_group_info(self.tgid, self, info)
    }

    /// 稳定组拥有在图锁外发出调度提醒和等待通知，不在siglock中反查图。
    pub(crate) fn notify(&self) {
        self.reclaim_pending();
        if let Some(graph) = self.graph.upgrade() {
            graph.notify_group(self.tgid, self);
        }
    }

    /// 只在 Graph 和信号锁外调用，摘走此前忽略或控制取消的节点再析构。
    pub(super) fn reclaim_pending(&self) {
        let retired = self.state.lock().pending.retired();
        drop(retired);
    }

    /// 图发表事务登记真实成员，早到的pending保留到其首次用户返回点。
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

    /// 图退出事务移出真实成员，已确认停止计数跟随成员责任减少。
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

    /// 图→siglock的一次递送事务处理控制取消、忽略、默认致命与真实FIFO，不进行通知。
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

    /// 持canonical动作锁递送；系统根忽略普通默认处置及KILL/STOP，强制故障另行领取。
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
        if self.system_init && matches!(info.signum, SIGKILL | SIGSTOP) {
            return Ok(false);
        }
        let mut group = self.state.lock();
        if group.members == 0 || (group.exit.is_some() && info.signum != SIGCONT) {
            return Ok(false);
        }
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
        // 已经致命退出的停止成员仍须完成CONT恢复；不再为退出组增加pending。
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

    /// 图锁内以父当前处置决定本次SIGCHLD自动回收，同锁发布pending，显式忽略即使blocked也不发送。
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

    /// 动作替换与当前组忽略pending同锁执行，系统首根角色不影响共享处置的其他组。
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

    /// 图根回收事务只写合法点退出请求，不在持关系锁时通知或反查成员。
    pub(super) fn mark_exit(&self, status: i32) {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let mut group = self.state.lock();
        group.exit.get_or_insert(status);
        group.stop_signal = None;
    }

    /// 图或用户安全点读取组退出请求，不改变任何调度器状态。
    pub(crate) fn exit_request(&self) -> Option<i32> {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let exit = self.state.lock().exit;
        exit
    }

    /// 只报告全体实际成员已经确认的停止，不把待处理停止信号算作停止作业。
    pub(crate) fn has_stopped_jobs(&self) -> bool {
        let binding = self.sighand.lock();
        let _siglock = binding.actions.lock();
        let stopped = self.state.lock().stop_reported;
        stopped
    }

    /// 父停止/继续通知遵守SA_NOCLDSTOP，沿原锁外发送路径处理SIGCHLD。
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

struct ThreadState {
    pending: Pending,
    fault: Option<SignalInfo>,
    mask: u64,
    real_blocked: Option<u64>,
    waited_set: u64,
    altstack: AltStack,
    stopped: bool,
    exec_exit: bool,
    wait: Option<(WaitId, Waker, WaitMode)>,
}

/// 每个身份拥有自己的private pending、mask、altstack和唯一可中断WaitId。
pub(crate) struct ThreadSignals {
    group: Arc<SignalGroup>,
    state: SpinLock<ThreadState>,
}

impl ThreadSignals {
    /// 锁外准备完整private状态，所有fallible准备早于身份与Ready发表。
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

    /// clone继承阻塞集；CLONE_VM且无VFORK时禁用child备用栈，不复制pending。
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

    /// 同步用户fault保存地址与原trap事实，内核强制信号走同一优先领取责任。
    pub(crate) fn force_fault(&self, info: SignalInfo) {
        assert!(matches!(
            info.signum,
            SIGILL | SIGTRAP | SIGBUS | SIGFPE | SIGSEGV | SIGSYS
        ));
        assert!(matches!(info.payload, SignalPayload::Fault { .. }));
        self.force(info);
    }

    /// 强制同步信号优先于异步队列；阻断或忽略时恢复默认处置，不重新执行硬件异常。
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

    /// pending查询只报告阻塞的实例，私有与组pending按同一siglock快照合并号码。
    pub(crate) fn pending(&self) -> u64 {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let group = self.group.state.lock();
        let private = self.state.lock();
        (private.pending.bits | group.pending.bits) & private.mask
    }

    /// 替换阻塞集时始终去除KILL/STOP，解除阻断后锁外提醒其他可处理组成员。
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

    /// 在siglock中读取实际线程阻塞集，供sigsuspend与原现场恢复。
    pub(crate) fn mask(&self) -> u64 {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let mask = self.state.lock().mask;
        mask
    }

    /// 同一动作锁与private锁裁决终端信号，系统根默认处置遵守真实角色。
    pub(crate) fn blocked_or_ignored(&self, signum: u8) -> bool {
        let bit = signal_bit(signum).expect("invalid terminal signal");
        let binding = self.group.sighand.lock();
        let actions = binding.actions.lock();
        let private = self.state.lock();
        private.mask & bit != 0
            || ignored(actions[signum as usize - 1], signum, self.group.system_init)
    }

    /// sigtimedwait暂时解除请求集阻断，保存真实旧mask以免忽略或默认致命丢失等待实例。
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

    /// 原等待结束后恢复真实阻塞集，清请求集责任再锁外重新提醒组pending消费者。
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

    /// sigaltstack查询返回动态ONSTACK，设置只能在当前备用栈之外进行。
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

    /// 完整帧解码之后提交mask；无效备用栈恢复保留旧值，遵守rt_sigreturn的忽略设置错误语义。
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

    /// sigtimedwait只领取请求集合中的真实实例；领取后copyout失败不得重新排队。
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

    /// 一次领取与处置快照同时完成；强制同步故障不应用系统根默认忽略规则。
    pub(crate) fn select(&self) -> Option<SignalSelection> {
        let mut retired = Retired::new();
        let binding = self.group.sighand.lock();
        let mut actions = binding.actions.lock();
        let mut group = self.group.state.lock();
        let mut private = self.state.lock();
        if let Some(status) = group.exit {
            return Some(SignalSelection::Exit(status));
        }
        if private.exec_exit {
            return Some(SignalSelection::Exit(0));
        }
        if group.stop_signal.is_some() {
            return Some(SignalSelection::Stop);
        }
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

    /// 仅真实成员到合法安全点才确认停止，末个确认产生一次父stopped事件。
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

    /// 原Waiter登记唯一WaitId；先保存登记再锁外重查pending，关闭早发送窗口。
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

    /// 只撤销本轮登记，最后Weak引用析构在siglock外。
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

    /// Killable接受退出控制，Interruptible还接受按真实首根角色可处理的pending。
    pub(crate) fn interruption_pending(&self, mode: WaitMode) -> bool {
        let binding = self.group.sighand.lock();
        let actions = binding.actions.lock();
        let group = self.group.state.lock();
        let private = self.state.lock();
        interruption(&actions, &group, &private, mode, self.group.system_init)
    }

    /// 短锁中按组角色取得可中断等待快照，真正通知与最后Waker拥有都在锁外。
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

    /// 已摘除节点的最后拥有不进入 siglock 或 Graph 持有期。
    pub(super) fn reclaim_pending(&self) {
        let retired = self.state.lock().pending.retired();
        drop(retired);
    }

    /// 请求本线程在合法点退出并打断等待，不改变组退出原因或其他成员控制。
    pub(crate) fn request_exec_exit(&self) {
        {
            let binding = self.group.sighand.lock();
            let _siglock = binding.actions.lock();
            self.state.lock().exec_exit = true;
        }
        self.notify_wait();
    }

    /// 查询本线程的私有exec退出请求，不把组退出原因当作本请求。
    pub(crate) fn exec_exit_requested(&self) -> bool {
        let binding = self.group.sighand.lock();
        let _siglock = binding.actions.lock();
        let requested = self.state.lock().exec_exit;
        requested
    }

    /// 唯一成员且无活等待登记时切换动作表，只保IGNORE的handler；阻塞集、pending与组计时拥有不变。
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

/// 验证用户信号范围并生成号码位，不允许零信号进入真实pending。
fn signal_bit(signum: u8) -> Result<u64, SignalError> {
    if signum == 0 || SIGNAL_COUNT < signum as usize {
        return Err(SignalError::InvalidSignal);
    }
    Ok(1 << (signum - 1))
}

/// 系统首根忽略普通默认处置和KILL/STOP；forced同步故障调用时不传此角色。
fn ignored(action: SignalAction, signum: u8, system_init: bool) -> bool {
    (system_init && (action.handler == SIG_DFL || matches!(signum, SIGKILL | SIGSTOP)))
        || action.handler == SIG_IGN
        || (action.handler == SIG_DFL && matches!(signum, SIGCHLD | SIGCONT | SIGURG | SIGWINCH))
}

/// 实际活成员全部确认时记录停止完成，成员退出不能制造空组停止事件。
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

/// 短锁中按真实系统角色判断中断，不分配、通知或反查身份图。
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

/// 备用栈设置共享sigaltstack和sigreturn约束，禁用时规范化全部保存字段。
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
