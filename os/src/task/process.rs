//! 有限用户身份域与退出亲缘；图只弱关联执行者，僵尸记录不拥有运行资源。

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

const TID: usize = 0;
const TGID: usize = 1;
const PGID: usize = 2;
const SID: usize = 3;

pub(crate) const WNOHANG: u32 = 1;
pub(crate) const WSTOPPED: u32 = 2;
pub(crate) const WEXITED: u32 = 4;
pub(crate) const WCONTINUED: u32 = 8;
pub(crate) const WNOWAIT: u32 = 0x0100_0000;
pub(crate) const WALL: u32 = 0x4000_0000;
pub(crate) const WCLONE: u32 = 0x8000_0000;
pub(crate) const WAIT_SIGNAL_MASK: i32 = 0x7f;
pub(crate) const WAIT_CORE_DUMP: i32 = 0x80;
pub(crate) const EXIT_STATUS_MASK: i32 = 0xff;
pub(crate) const EXIT_STATUS_SHIFT: u32 = 8;
pub(crate) const WAIT_STOPPED: i32 = 0x7f;
pub(crate) const WAIT_CONTINUED: i32 = 0xffff;
pub(crate) const WAIT_STATUS_MASK: i32 = (EXIT_STATUS_MASK << EXIT_STATUS_SHIFT) | EXIT_STATUS_MASK;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ProcessError {
    InvalidCapacity,
    NoMemory,
    NoPid,
    InvalidParent,
    NoChild,
    Busy,
    NoProcess,
    Permission,
    Access,
    InvalidArgument,
}

/// 根回收职责属于构造图时选定的角色，不由首个号码推断。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RootRole {
    Kernel,
    System,
}

/// 用户接受的最终机器动作，图只保存意图并停止用户生产者。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum ShutdownOperation {
    Halt = 2,
    PowerOff = 3,
    Restart = 4,
}

const RUNNING: u8 = 0;
const CANCELLED: u8 = 1;

/// 数字不可变的身份对象；诊断引用不参与号码占用判定。
pub(crate) struct Pid {
    number: usize,
}

impl Pid {
    /// 读取该对象原来的数值，即使它的最后一个角色已解绑也不改变。
    pub(crate) fn number(&self) -> usize {
        self.number
    }
}

/// 组身份对象保持不变；可变角色和退出信号由关系锁事务更新。
pub(crate) struct ThreadGroup {
    graph: Weak<TaskGraph>,
    pid: Arc<Pid>,
    relations: SpinLock<GroupRelations>,
    uid: u32,
    signals: Arc<SignalGroup>,
    time: Arc<GroupTime>,
    limits: SpinLock<Limits>,
    account: Arc<UserAccount>,
}

struct GroupRelations {
    pgid: Arc<Pid>,
    sid: Arc<Pid>,
    exit_signal: u8,
    controlling_tty: Option<&'static tty::Endpoint>,
}

/// 控制端点只弱关联原图；会话和前台保稳定身份，不增加任何号码角色。
pub(crate) struct TerminalControl {
    pub(crate) graph: Weak<TaskGraph>,
    pub(crate) session: Arc<Pid>,
    pub(crate) foreground: Arc<Pid>,
}

impl ThreadGroup {
    /// 返回线程组身份，退出后诊断对象仍保留原值。
    pub(crate) fn tgid(&self) -> usize {
        self.pid.number
    }

    /// 返回根组自身建立或从父组继承的进程组身份。
    pub(crate) fn pgid(&self) -> usize {
        self.relations.lock().pgid.number
    }

    /// 在关系锁内取得当前PGID原对象，等待不得先取数字再回图查同号身份。
    pub(crate) fn pgid_object(&self) -> Arc<Pid> {
        Arc::clone(&self.relations.lock().pgid)
    }

    /// 返回根组自身建立或从父组继承的会话身份。
    pub(crate) fn sid(&self) -> usize {
        self.relations.lock().sid.number
    }

    /// 在关系锁内保留当前SID对象，号码复用不改变旧会话身份。
    pub(crate) fn sid_object(&self) -> Arc<Pid> {
        Arc::clone(&self.relations.lock().sid)
    }

    /// 借用组独有pending与控制拥有，不把共享动作轴等同于线程组。
    pub(crate) fn signals(&self) -> &Arc<SignalGroup> {
        &self.signals
    }

    /// 系统首根角色仅保存在原信号对象中，普通号码1没有特殊退出或信号权限。
    pub(crate) fn is_system_init(&self) -> bool {
        self.signals.is_system_init()
    }

    /// 借用真实组CPU时间和itimer拥有，线程私有时间由执行者另持。
    pub(crate) fn time(&self) -> &Arc<GroupTime> {
        &self.time
    }

    /// 快照组共享的完整软硬限额，调用者不持有组限额锁跨业务操作。
    pub(crate) fn resource_limit(&self, resource: Resource) -> ResourceLimit {
        self.limits.lock().get(resource)
    }

    /// 只对实际入队节点增加目标用户用量，额度耗尽不混成分配错误。
    pub(super) fn charge_signal(&self, limit: u64) -> Result<SignalCharge, SignalError> {
        self.account
            .signal(limit)
            .map_err(|_| SignalError::QueueFull)
    }
}

/// 执行者拥有自己的图与组；图反向只持 Weak<UserThread>，没有强引用环。
pub(crate) struct UserIdentity {
    graph: Arc<TaskGraph>,
    group: Arc<ThreadGroup>,
    tid: SpinLock<Arc<Pid>>,
    signals: Arc<ThreadSignals>,
}

impl UserIdentity {
    /// 返回当前TID；exec可在保留本身份对象时接管原组首的Pid。
    pub(crate) fn tid(&self) -> usize {
        self.tid.lock().number
    }

    /// 返回用户线程所属线程组的 Linux 数字身份。
    pub(crate) fn tgid(&self) -> usize {
        self.group.tgid()
    }

    /// 取得不持 MM、文件或执行栈的线程组引用。
    pub(crate) fn group(&self) -> Arc<ThreadGroup> {
        self.group.clone()
    }

    /// 借用负责当前用户身份关系的原图。
    pub(crate) fn graph(&self) -> &Arc<TaskGraph> {
        &self.graph
    }

    /// 保存稳定身份对象供诊断；数值复用后此对象不会命中新执行者。
    pub(crate) fn tid_object(&self) -> Arc<Pid> {
        self.tid.lock().clone()
    }

    /// 身份准备时已构造private信号状态，退出解绑运行资源不会丢失早到信号。
    pub(crate) fn signals(&self) -> Arc<ThreadSignals> {
        self.signals.clone()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct WaitEvent {
    pub pid: usize,
    pub status: i32,
    pub user_interrupts: usize,
    pub user_timer_interrupts: usize,
    pub task_id: TaskId,
    pub cpu: usize,
}

/// 有限waitid消费的真实子组状态，停止事件只在全部实际成员到安全点后产生。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ChildEvent {
    pub(crate) pid: usize,
    pub(crate) uid: u32,
    pub(crate) code: i32,
    pub(crate) status: i32,
    pub(crate) user_ns: u64,
    pub(crate) kernel_ns: u64,
}

/// 已由ABI解码的子进程选择条件，扫描和消费共用同一关系锁。
pub(crate) enum WaitSelector {
    Pid(Arc<Pid>),
    Group(Arc<Pid>),
    Any,
}

/// 每个真实执行者的离栈观察；节点锁外准备，退出后只保小完成对象。
pub(crate) struct PreparedExecution {
    join: JoinObserver,
    next: Option<Box<PreparedExecution>>,
}

impl PreparedExecution {
    /// 在身份发表前准备唯一成员节点，不将Task或用户运行资源带入僵尸记录。
    pub(crate) fn try_new(join: JoinObserver) -> Result<Box<Self>, ProcessError> {
        Box::try_new(Self { join, next: None }).map_err(|_| ProcessError::NoMemory)
    }
}

struct ThreadEntry {
    user: Weak<UserThread>,
    group: Arc<ThreadGroup>,
    execution: Box<PreparedExecution>,
    signals: Arc<ThreadSignals>,
    charge: ProcessCharge,
}

struct ProcessEntry {
    group: Arc<ThreadGroup>,
    // 父组最后成员退出时转交根回收者，父数值复用前解除所有子引用。
    parent: Option<usize>,
    members: usize,
    leader_task_id: TaskId,
    leader_status: Option<i32>,
    exec_owner: Option<TaskId>,
    did_exec: bool,
    user_interrupts: usize,
    user_timer_interrupts: usize,
    completed: Option<Box<PreparedExecution>>,
    exit: Option<WaitEvent>,
    control_event: Option<ChildEvent>,
    notification: Option<GroupNotification>,
    leader_charge: Option<ProcessCharge>,
}

/// 已提交递送的锁外通知责任，继续事件不能被后续普通信号覆盖。
enum GroupNotification {
    Signal,
    Continued(ChildEvent),
}

impl ProcessEntry {
    /// 内核协调者消费进程事件后等待各成员真正离栈，不把用户退出事件当栈已归还。
    fn join_members(&mut self) {
        while let Some(mut execution) = self.completed.take() {
            self.completed = execution.next.take();
            execution.join.join();
        }
    }
}

impl Drop for ProcessEntry {
    /// 仅迭代释放小完成观察，不递归展开成员数，也不在用户wait中等待离栈。
    fn drop(&mut self) {
        while let Some(mut execution) = self.completed.take() {
            self.completed = execution.next.take();
        }
    }
}

struct CollectedChild {
    event: WaitEvent,
    process: Option<ProcessEntry>,
    notification_parent: Option<Arc<ThreadGroup>>,
    _released: [Option<Arc<Pid>>; 3],
}

struct Slot {
    reserved: bool,
    pid: Option<Arc<Pid>>,
    roles: [usize; 4],
    thread: Option<ThreadEntry>,
    process: Option<ProcessEntry>,
    // 用户wait与autoreap移出的未确认离栈观察，不继续占用任何号码角色。
    completed: Option<Box<PreparedExecution>>,
}

impl Slot {
    /// 槽页在关系锁外构造，号码占用只由预留和实际角色决定。
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
    /// 整图释放时迭代归还残余小观察，未确认离栈者仍由scheduler独立拥有运行资源。
    fn drop(&mut self) {
        while let Some(mut execution) = self.completed.take() {
            self.completed = execution.next.take();
        }
    }
}

/// 按需槽页保护四角色与亲缘，退出只保未确认离栈的观察。
pub(crate) struct TaskGraph {
    slots: SpinLock<Numbers>,
    root_role: RootRole,
    // 仅持slots关系锁提交；锁外Acquire读取只观察已接受的最终动作。
    closing: AtomicU8,
    waiters: WaitQueue,
    control_waiters: WaitQueue,
}

/// 未发表号码的唯一归还责任，任何锁外准备失败都正常清除预留。
struct NumberReservation {
    graph: Arc<TaskGraph>,
    index: usize,
    active: bool,
}

/// 身份与运行资源分别准备；只有成功发表后才允许原 PreparedTask 进入 Ready。
pub(crate) struct PreparedIdentity {
    identity: Arc<UserIdentity>,
    parent: Option<Arc<ThreadGroup>>,
    reservation: NumberReservation,
    charge: Option<ProcessCharge>,
}

impl TaskGraph {
    /// 号码域与存储量分离，首次仅分配一个小槽页。
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

    /// 根角色在构造后不变，系统调用与协调者使用同一图判断职责。
    pub(crate) fn root_role(&self) -> RootRole {
        self.root_role
    }

    /// 保存首次系统关机动作并在同一关系事务关闭派生准入、请求全部活组退出。
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

    /// 只返回实际接受的用户机器动作，kernel取消不伪造关机操作。
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

    /// 新进程复制父组四项限额，真实用户承担唯一线程的计费责任。
    pub(crate) fn prepare_process(
        self: &Arc<Self>,
        parent: Option<&Arc<ThreadGroup>>,
        sighand: Arc<Sighand>,
        uid: u32,
        inherited: SignalInheritance,
        exit_signal: u8,
    ) -> Result<PreparedIdentity, ProcessError> {
        let account = UserAccount::get(uid)?;
        let limits = parent.map_or_else(Limits::new, |parent| *parent.limits.lock());
        let charge = account.process(limits.get(Resource::Nproc).current)?;
        let reservation = self.reserve_number(parent, false)?;
        let pid = Arc::try_new(Pid {
            number: reservation.index + 1,
        })
        .map_err(|_| ProcessError::NoMemory)?;
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

    /// 新线程只预留自己的TID，保相同组身份，最终发表再次确认组仍有活成员。
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

    /// 锁外预备槽页，短锁重验关闭、亲缘、exec拥有和空闲号码后才预留。
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

    /// 再次创建前移出活组及已回收槽中确认离栈的观察，锁外释放节点。
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

    /// 锁内只快照已发表的弱执行关联；升级在锁外，不以 Ready 为查询条件。
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

    /// 同数值仍必须核对原 Pid 对象，旧诊断对象永远不会指向新同号线程。
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

    /// 保存此刻已发表的原身份，阻塞等待不能因数值复用改选另一个对象。
    pub(crate) fn pid_object(&self, pid: usize) -> Option<Arc<Pid>> {
        self.slots.lock().get(pid.checked_sub(1)?)?.pid.clone()
    }

    /// 图关系锁下保留首个组退出原因，锁外通知真实成员在合法点退出。
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

    /// 当前执行者在IRQ和业务短锁外读取本组控制，不允许远核改用户现场或释放栈。
    pub(crate) fn exit_request(&self, identity: &UserIdentity) -> Option<i32> {
        identity.group.signals.exit_request()
    }

    /// 用户waitid登记沿用原退出通知源，业务事件在登记后仍须复查。
    pub(crate) fn waiters(&self) -> &WaitQueue {
        &self.waiters
    }

    /// kernel明确取消关闭派生准入，保留已接受的机器动作与各组首次退出原因。
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

    /// 逐槽保活当前退出组并锁外提醒，不为全图取消或关机分配通知列表。
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

    /// 已确认停止的成员在独立控制源停驻，完成计数不由普通Blocked推导。
    pub(crate) fn control_waiters(&self) -> &WaitQueue {
        &self.control_waiters
    }

    /// 真实成员在合法安全点确认组控制，末个停止确认才发表父事件。
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

    /// 同锁筛选全部亲缘、观察及唯一消费；WNOWAIT保留事件且不计入父的子CPU账。
    pub(crate) fn try_wait_event(
        &self,
        parent: &ThreadGroup,
        selector: &WaitSelector,
        options: u32,
    ) -> Result<Option<ChildEvent>, ProcessError> {
        let mut retired = None;
        let event = {
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
        if let Some(retired) = retired.as_mut() {
            self.complete_retired_notification(retired);
        }
        if retired.is_some() {
            self.waiters.wake_all();
        }
        drop(retired);
        Ok(event)
    }

    /// 活父仍属于原亲缘组时返回其TGID；内核根回收者没有用户PID。
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

    /// pid为零查询本组，否则允许查询仍发表的TID或僵尸组首。
    pub(crate) fn get_pgid(
        &self,
        caller: &UserIdentity,
        pid: usize,
    ) -> Result<usize, ProcessError> {
        let slots = self.slots.lock();
        let group = self.identity_target(&slots, caller, pid)?;
        Ok(group.pgid())
    }

    /// 查询有限身份域的会话角色，零选择调用者原组。
    pub(crate) fn get_sid(&self, caller: &UserIdentity, pid: usize) -> Result<usize, ProcessError> {
        let slots = self.slots.lock();
        let group = self.identity_target(&slots, caller, pid)?;
        Ok(group.sid())
    }

    /// 关系锁内验证调用者原组，数字查询只命中当前实际发表的身份。
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

    /// 调用者必须仍是本图发表的活线程与活组，TTY入口不接受旧诊断身份。
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

    /// 在copy之后的Graph事务取得控制终端；自动取得不满足条件时保持原关联。
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

    /// 同Graph锁读取本组控制端点，无活身份或没有关联时不返回其它会话的端点。
    pub(crate) fn controlling_tty(&self, caller: &UserIdentity) -> Option<&'static tty::Endpoint> {
        let slots = self.slots.lock();
        self.terminal_caller(&slots, caller).ok()?;
        let tty = caller.group.relations.lock().controlling_tty;
        tty
    }

    /// 只有本组自身ctty和端点原会话一致时才能取得稳定会话与前台对象。
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

    /// 同事务按实际PGID角色或活PID角色确定目标会话，旧端点拥有在锁外释放。
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

    /// 调用者持copy；Graph中清原会话全部关联并移出控制拥有，不调用TTY或通知。
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

    /// 非会话首只清本组关联；会话首移出全会话控制责任，HUP/CONT由锁外调用者完成。
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

    /// 一个Graph持有期按稳定PGID实际角色递送，提交后才完成成员和父等待通知。
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

    /// 自身控制终端的后台访问在同图中裁决，阻塞或忽略TTOU允许，TTIN与孤儿返回EIO。
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

    /// 逐槽移出已提交通知与原组拥有，合并普通提醒时仍保留继续事件责任。
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

    /// 通知已经提交的原组；父SIGCHLD与等待唤醒不持Graph、关系或信号短锁。
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

    /// wait或autoreap移出的进程继续承担原通知，最后拥有释放前完成已经提交的责任。
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

    /// 只允许本组或同会话的直接子改变PGID；已exec子、会话首和跨会话加入被拒绝。
    pub(crate) fn set_pgid(
        &self,
        caller: &UserIdentity,
        pid: usize,
        pgid: usize,
    ) -> Result<(), ProcessError> {
        if (i32::MAX as usize) < pid || (i32::MAX as usize) < pgid {
            return Err(ProcessError::InvalidArgument);
        }
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

    /// 建立新会话时只清本组ctty，并提交原组及实际直接子组失去连接后的孤儿通知。
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

    /// kill按真实进程、进程组或全域选目标，零信号只执行存在性与权限检查。
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

    /// tkill与tgkill共用真实TID查找，tgid条件在同一关系快照中核对，零信号不递送。
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

    /// queueinfo只寻具体进程，外部发送者不得伪造非负kernel code或SI_TKILL。
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

    /// tgsigqueueinfo核对真实组与成员后送private FIFO，队列满保留旧实例并返回QueueFull。
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

    /// 内核控制者通过真实组身份发送普通信号，所有kick与uaccess仍在图和siglock之外。
    pub(crate) fn send_kernel(&self, pid: usize, signum: u8) -> Result<(), SignalError> {
        if signum == 0 || SIGNAL_COUNT < signum as usize {
            return Err(SignalError::InvalidSignal);
        }
        let group = self.group_target(pid)?;
        self.deliver_group(&group, SignalInfo::kernel(signum), false)
    }

    /// 将当前执行者触发的异步信号递送到稳定private pending，保留原发送身份和锁外通知。
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

    /// 在关系锁中取得稳定组拥有，退出组仍可被kill0探测但发送不会重新建立成员。
    fn group_target(&self, pid: usize) -> Result<Arc<ThreadGroup>, SignalError> {
        let slots = self.slots.lock();
        slots
            .get(pid.checked_sub(1).ok_or(SignalError::NoProcess)?)
            .and_then(|slot| slot.process.as_ref())
            .map(|process| process.group.clone())
            .ok_or(SignalError::NoProcess)
    }

    /// 用户节点先在锁外准备，Graph→siglock 中重查原组并提交；所有退费和通知在锁外。
    fn deliver_group(
        &self,
        group: &Arc<ThreadGroup>,
        info: SignalInfo,
        queued: bool,
    ) -> Result<(), SignalError> {
        let mut prepared = if queued {
            PreparedSignal::user(info)?
        } else {
            PreparedSignal::kernel()
        };
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
        group.signals.notify();
        if let Some(event) = result? {
            self.notify_parent(group, event);
            self.waiters.wake_all();
        }
        Ok(())
    }

    /// 定时IRQ与父事件由原SignalGroup弱图发送，同号重用必须核对原信号拥有。
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

    /// 锁外保存原 private 身份并准备节点，再取 Graph 核对同一对象，防止号码复用改选目标。
    fn deliver_thread(
        &self,
        sender: &UserIdentity,
        tid: usize,
        tgid: Option<usize>,
        info: SignalInfo,
        queued: bool,
        expected_private: Option<&Arc<ThreadSignals>>,
    ) -> Result<(), SignalError> {
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
        let mut prepared = if queued {
            PreparedSignal::user(info)?
        } else {
            PreparedSignal::kernel()
        };
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

    /// rt_sigaction替换和当前真实组全部pending清理同事务完成，避免裸动作更新绕开组责任。
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

    /// 逐个快照真实成员后放锁通知，无临时Vec分配；原组对象阻止同号新组接受旧提醒。
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

    /// 控制事件的父关系快照在图锁中取得，SIGCHLD发送与等待通知仍在锁外。
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

    /// 同关系锁核对原组身份和实际活成员；旧组对象与同号新组不能互为父子。
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
    /// 供父模块在身份发表前绑定完整 UserThread 与原 PreparedTask 资源。
    pub(crate) fn identity(&self) -> Arc<UserIdentity> {
        self.identity.clone()
    }

    /// 最终关系发表与关闭准入同锁重查，私有首根必须仍是System唯一PID1。
    pub(crate) fn publish(
        mut self,
        user: Weak<UserThread>,
        execution: Box<PreparedExecution>,
    ) -> Result<(), ProcessError> {
        let graph = self.reservation.graph.clone();
        let mut old_relations = None;
        {
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
        drop(old_relations);
        Ok(())
    }
}

impl Drop for NumberReservation {
    /// 未发表准备失败只归还自己的私有号码，不解绑任何已发表身份。
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

/// 已有身份槽中增加实际角色成员；不能通过 Arc 引用数推导号码是否占用。
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

/// 最后实际角色解绑时立即释放数值；移出的身份引用由调用者在关系锁外析构。
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

/// 单一不可变uid凭据满足同uid或root权限，CONT另允许同会话的真实组成员。
fn signal_permission(sender: &UserIdentity, target: &ThreadGroup, signum: u8) -> bool {
    sender.group.uid == 0
        || sender.group.uid == target.uid
        || (signum == SIGCONT && sender.group.sid() == target.sid())
}

/// 把普通Linux退出状态转成waitid字段，不把编码后的status误作si_status。
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

/// 事件产生时快照真实组账与已消费后代账，不依赖诊断引用或线程数估算。
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

/// 图锁中仅摘走Acquire确认离栈的节点，保留未完成链并把最后拥有交给锁外调用者。
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

/// 图锁中移走退出组并解绑角色；用户消费保留原执行观察，kernel消费移出链供锁外join。
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

/// 控制事件保留原SA_NOCLDSTOP发送路径，不参与退出处置和自动回收事务。
fn send_parent_event(parent: Option<&ThreadGroup>, event: ChildEvent) {
    if let Some(parent) = parent {
        parent.signals.send_child(child_signal_info(SIGCHLD, event));
    }
}

/// 由同一子事件编码退出与控制通知的siginfo，CPU账按公开USER_HZ换算。
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

/// 实际角色与身份指针必须同时匹配，旧诊断Arc不使同号新身份成为目标。
fn has_role(slots: &Numbers, pid: &Arc<Pid>, role: usize) -> bool {
    slots.get(pid.number - 1).is_some_and(|slot| {
        slot.roles[role] != 0
            && slot
                .pid
                .as_ref()
                .is_some_and(|actual| Arc::ptr_eq(actual, pid))
    })
}

/// 先按PGID实际活组查会话；没有PGID成员才按同对象活PID角色查会话。
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

/// 持Graph先验证端点原绑定，再逐组清关联；控制锁不跨关系锁扫描。
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

/// 孤儿判断排除真正系统首根，普通父组仍以不同PGID、相同SID连接判断。
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

/// 一个Graph持有期递送稳定PGID的全部活组，不分配或执行锁外通知。
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

/// 仅对本次false到true的孤儿变化和已确认停止作业提交HUP随后CONT。
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

/// 旧父同SID且不同PGID就是变化前的非孤儿连接，只检查其实际直接子组。
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
