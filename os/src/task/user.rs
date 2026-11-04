//! 私有用户现场与独立 MM 引用；反向执行关联不保活 Task 或内核栈。

#[cfg(feature = "ktest")]
use crate::cpu::AtomicCpuSet;
use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};
use core::alloc::AllocError;
use core::cell::{Cell, RefCell, RefMut};
use core::mem;
use core::sync::atomic::Ordering;
use spin::Once;

use crate::arch::{self, UserContext, UserTrap};
#[cfg(target_arch = "loongarch64")]
use crate::arch::{
    ale::{EmulationResult, Instruction, UserMemory},
    UserAccess,
};
use crate::fs::O_RDWR;
use crate::fs::{
    open_path_with_flags, DescriptorFlags, FileIoContext, FileTable, FsContext, FsError,
    ResolveMode, Vfs,
};
use crate::mm::space::{self, Mm, MmUser, VmError};
use crate::net::NetStackRef;
use crate::sync::{Completion, IrqSaveGuard, SleepMutex, SpinLock};
use crate::syscall::Restart;
use crate::task::exec::{ExecError, LoadedImage};
use crate::task::process::{
    PreparedExecution, PreparedIdentity, ProcessError, TaskGraph, ThreadGroup, UserIdentity,
};
use crate::task::signal::{
    Sighand, SignalControl, SignalFrameArgs, SignalInfo, SignalInheritance, SignalPayload,
    SignalSelection, ThreadSignals, SIGBUS, SIGCHLD, SIGFPE, SIGILL, SIGKILL, SIGSEGV, SIGTRAP,
};
use crate::task::time::UserTime;
use crate::task::{self, JoinHandle, Placement, PreparedTask, Task};
use crate::wait::{WaitMode, Waiter};

const ILL_ILLOPC: i32 = 1;
const BUS_ADRALN: i32 = 1;
const BUS_ADRERR: i32 = 2;
const BUS_OBJERR: i32 = 3;
const SEGV_MAPERR: i32 = 1;
const SEGV_ACCERR: i32 = 2;
const TRAP_BRKPT: i32 = 1;
#[cfg(target_arch = "loongarch64")]
const FPE_FLTDIV: i32 = 3;
#[cfg(target_arch = "loongarch64")]
const FPE_FLTOVF: i32 = 4;
#[cfg(target_arch = "loongarch64")]
const FPE_FLTUND: i32 = 5;
#[cfg(target_arch = "loongarch64")]
const FPE_FLTRES: i32 = 6;
#[cfg(target_arch = "loongarch64")]
const FPE_FLTINV: i32 = 7;
#[cfg(target_arch = "loongarch64")]
const FPE_FLTUNK: i32 = 14;

/// 当前用户的不可变凭据；目录和文件权限判断使用真实 inode uid/gid/mode。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Credentials {
    pub(crate) uid: u32,
    pub(crate) gid: u32,
}

impl Credentials {
    pub(crate) const ROOT: Self = Self { uid: 0, gid: 0 };
}

/// 同一文件系统上下文保留完整挂载视图以及可独立共享的 root/cwd。
pub(crate) struct UserFs {
    pub(crate) vfs: Arc<Vfs>,
    pub(crate) context: FsContext,
}

/// 文件、FS、信号各保留明确拥有；可选网络引用随派生程序共享。
#[derive(Clone)]
pub(crate) struct UserRuntime {
    pub(crate) files: Arc<FileTable>,
    pub(crate) fs: Arc<SleepMutex<UserFs>>,
    pub(crate) credentials: Credentials,
    pub(crate) sighand: Arc<Sighand>,
    pub(crate) net: Option<NetStackRef>,
}

impl UserRuntime {
    /// 在普通任务锁外准备运行资源；无网络夹具显式传入 None，失败不发表用户身份。
    pub(crate) fn try_new(
        vfs: Arc<Vfs>,
        context: FsContext,
        credentials: Credentials,
        net: Option<NetStackRef>,
    ) -> Result<Self, crate::fs::FsError> {
        let files = FileTable::try_new()?;
        let files = Arc::try_new(files).map_err(|_| crate::fs::FsError::NoMemory)?;
        let fs = Arc::try_new(SleepMutex::new(UserFs { vfs, context }))
            .map_err(|_| crate::fs::FsError::NoMemory)?;
        let sighand = Sighand::try_new().map_err(|_| FsError::NoMemory)?;
        Ok(Self {
            files,
            fs,
            credentials,
            sighand,
            net,
        })
    }

    /// 在尚未发表的用户表中一次准备并发表共享 console 标准流，失败时保留空表契约。
    pub(crate) fn install_console(&self) -> Result<(), FsError> {
        let (vfs, context) = {
            let fs = self.fs.lock();
            (Arc::clone(&fs.vfs), fs.context.clone())
        };
        let path = vfs.resolve(&context, b"/dev/console", ResolveMode::Follow)?;
        let console = open_path_with_flags(&vfs, path, O_RDWR, FileIoContext::Kernel)?;
        let first = self.files.reserve(3)?;
        if first.fd() != 0 {
            return Err(FsError::InvalidOperation);
        }
        let second = self.files.reserve(3)?;
        if second.fd() != 1 {
            return Err(FsError::InvalidOperation);
        }
        let third = self.files.reserve(3)?;
        if third.fd() != 2 {
            return Err(FsError::InvalidOperation);
        }
        let flags = DescriptorFlags::default();
        first.publish(console.clone(), flags);
        second.publish(console.clone(), flags);
        third.publish(console, flags);
        Ok(())
    }

    /// 按三个独立共享轴复制运行资源；可选网络引用保持原值，所有分配在短锁外。
    pub(crate) fn try_fork(
        self,
        share_files: bool,
        share_fs: bool,
        share_sighand: bool,
    ) -> Result<Self, FsError> {
        let Self {
            files,
            fs,
            credentials,
            sighand,
            net,
        } = self;
        let files = if share_files {
            files
        } else {
            Arc::try_new(files.try_clone()?).map_err(|_| FsError::NoMemory)?
        };
        let fs = if share_fs {
            fs
        } else {
            let snapshot = {
                let current = fs.lock();
                UserFs {
                    vfs: current.vfs.clone(),
                    context: current.context.clone(),
                }
            };
            Arc::try_new(SleepMutex::new(snapshot)).map_err(|_| FsError::NoMemory)?
        };
        let sighand = if share_sighand {
            sighand
        } else {
            sighand.try_copy().map_err(|_| FsError::NoMemory)?
        };
        Ok(Self {
            files,
            fs,
            credentials,
            sighand,
            net,
        })
    }

    /// 其他实际执行者仍共享时才私有化；普通资源快照不触发复制。
    pub(crate) fn try_exec(self) -> Result<Self, FsError> {
        let Self {
            files,
            fs,
            credentials,
            sighand,
            net,
        } = self;
        let files = if 1 < files.user_count() {
            Arc::try_new(files.try_clone()?).map_err(|_| FsError::NoMemory)?
        } else {
            files
        };
        let sighand = if 1 < sighand.user_count() {
            sighand.try_copy().map_err(|_| FsError::NoMemory)?
        } else {
            sighand
        };
        Ok(Self {
            files,
            fs,
            credentials,
            sighand,
            net,
        })
    }
}

/// 只计真实用户运行绑定，未发表失败与正常退出共用同一归还责任。
struct RuntimeBinding(UserRuntime);

impl RuntimeBinding {
    /// 在发表前移动完整资源，并取得两项独立运行责任。
    fn new(runtime: UserRuntime) -> Self {
        runtime.files.add_user();
        runtime.sighand.add_user();
        Self(runtime)
    }
}

impl Drop for RuntimeBinding {
    /// 先解除运行责任，字段最后拥有由原锁外调用者释放。
    fn drop(&mut self) {
        self.0.files.remove_user();
        self.0.sighand.remove_user();
    }
}

struct UserProcess {
    identity: Arc<UserIdentity>,
    runtime: RuntimeBinding,
    child_set_tid: Option<usize>,
    clear_child_tid: usize,
    vfork: Option<Arc<Completion>>,
}

pub struct UserThread {
    executor: Once<Weak<Task>>,
    mm: SpinLock<Option<MmUser>>,
    context: RefCell<UserContext>,
    process: SpinLock<Option<UserProcess>>,
    user_timer_interrupts: Cell<usize>,
    user_time: Once<UserTime>,
    restart: Cell<Option<Restart>>,
    saved_mask: Cell<Option<u64>>,
    #[cfg(feature = "ktest")]
    user_cpu_mask: AtomicCpuSet,
}

// SAFETY: context、user_timer_interrupts、restart与saved_mask仅由弱关联指定的唯一Running Task访问；
// 现场/重启访问核对当前身份，RefCell排除现场重叠借用，任务迁核由原调度交接串行化。
// mm/process远端拥有快照由短锁保护；Ready前初始化的UserTime仅通过同步原子记账接口共享。
unsafe impl Sync for UserThread {}

impl UserThread {
    /// 在发表 Task 前准备完整私有用户现场和地址空间拥有。
    pub(crate) fn try_new(mm: Arc<Mm>, context: UserContext) -> Result<Arc<Self>, AllocError> {
        Arc::try_new(Self {
            executor: Once::new(),
            mm: SpinLock::new(Some(MmUser::new(mm))),
            context: RefCell::new(context),
            process: SpinLock::new(None),
            user_timer_interrupts: Cell::new(0),
            user_time: Once::new(),
            restart: Cell::new(None),
            saved_mask: Cell::new(None),
            #[cfg(feature = "ktest")]
            user_cpu_mask: AtomicCpuSet::new(),
        })
    }

    /// 在唯一 PreparedTask 尚未发表时建立弱反向关联，不产生强引用环。
    pub(crate) fn bind_executor(&self, executor: Weak<Task>) {
        assert!(
            self.executor.get().is_none(),
            "user executor is already bound"
        );
        self.executor.call_once(|| executor);
    }

    /// 取得当前实际资源快照，退出解绑后的诊断引用不再保有 MM。
    pub(crate) fn mm(&self) -> Option<Arc<Mm>> {
        self.mm
            .lock()
            .as_ref()
            .map(|binding| Arc::clone(binding.mm()))
    }

    /// 完整运行资源与child入口/退出TID地址在原PreparedTask发表之前绑定。
    pub(crate) fn bind_process(
        &self,
        identity: Arc<UserIdentity>,
        runtime: UserRuntime,
        child_set_tid: Option<usize>,
        clear_child_tid: usize,
    ) {
        assert!(self.user_time.get().is_none(), "user time is already bound");
        self.user_time
            .call_once(|| UserTime::new(identity.group().time().clone()));
        let mut process = self.process.lock();
        assert!(process.is_none(), "user process is already bound");
        *process = Some(UserProcess {
            identity,
            runtime: RuntimeBinding::new(runtime),
            child_set_tid,
            clear_child_tid,
            vfork: None,
        });
    }

    /// 只有绑定的当前Task消费一次child_tid地址，用户访问必须在此短锁外执行。
    fn take_child_tid(&self) -> Option<usize> {
        self.assert_current();
        self.process
            .lock()
            .as_mut()
            .expect("user entry has no process")
            .child_set_tid
            .take()
    }

    /// 只记录本线程退出时的用户清零地址；是否可写在实际旧Mm清理处决定。
    pub(crate) fn set_tid_address(&self, address: usize) -> usize {
        self.assert_current();
        let mut process = self.process.lock();
        let process = process
            .as_mut()
            .expect("set_tid_address has no user process");
        process.clear_child_tid = address;
        process.identity.tid()
    }

    /// 当前用户执行者生成线程定向异步信号，siginfo保留真实发送者，动作与mask沿原private协议处理。
    pub fn send_kernel_signal(&self, signum: u8) {
        self.assert_current();
        let identity = self
            .identity()
            .expect("kernel signal has no current user identity");
        identity
            .graph()
            .send_kernel_thread(&identity, signum)
            .expect("current kernel signal delivery failed");
    }

    /// 在关系和信号操作前取得身份快照，不跨图锁保存用户资源锁。
    pub(crate) fn identity(&self) -> Option<Arc<UserIdentity>> {
        self.process
            .lock()
            .as_ref()
            .map(|process| Arc::clone(&process.identity))
    }

    /// 短锁中只克隆独立文件/FS引用，I/O、用户访问与最后拥有释放由锁外调用者完成。
    pub(crate) fn runtime(&self) -> Option<UserRuntime> {
        self.process
            .lock()
            .as_ref()
            .map(|process| process.runtime.0.clone())
    }

    /// child尚未Ready时绑定双方拥有的完成对象，不借父Task栈。
    pub(crate) fn bind_vfork(&self, completion: Arc<Completion>) {
        let mut process = self.process.lock();
        let process = process.as_mut().expect("vfork child has no process");
        assert!(process.vfork.is_none(), "vfork completion is already bound");
        process.vfork = Some(completion);
    }

    /// 父致命取消只解绑本次关系，child资源与执行仍走自己的正常清理。
    pub(crate) fn cancel_vfork(&self, completion: &Arc<Completion>) {
        let removed = {
            let mut process = self.process.lock();
            process.as_mut().and_then(|process| {
                if process
                    .vfork
                    .as_ref()
                    .is_some_and(|bound| Arc::ptr_eq(bound, completion))
                {
                    process.vfork.take()
                } else {
                    None
                }
            })
        };
        drop(removed);
    }

    /// 在同一执行者提交新现场与活动根，旧MM实际使用结束后才恢复vfork父。
    pub(crate) fn install_image(&self, image: LoadedImage, runtime: UserRuntime) {
        self.assert_current();
        let runtime = RuntimeBinding::new(runtime);
        runtime.0.files.close_on_exec();
        self.signals()
            .expect("exec has no signals")
            .reset_for_exec(runtime.0.sighand.clone());
        let old_mm = self.mm().expect("exec has no old MM");
        let clear_child_tid = {
            let mut process = self.process.lock();
            let process = process.as_mut().expect("exec has no process");
            mem::take(&mut process.clear_child_tid)
        };
        clear_tid(&old_mm, clear_child_tid);
        let binding = MmUser::new(image.mm);
        let active = binding.mm().clone();
        self.restart.set(None);
        self.saved_mask.set(None);
        let (old_runtime, old_binding, old_active, completion) = {
            let _irq = IrqSaveGuard::new();
            let (old_runtime, completion) = {
                let mut process = self.process.lock();
                let process = process.as_mut().expect("exec has no process");
                process.child_set_tid = None;
                (
                    mem::replace(&mut process.runtime, runtime),
                    process.vfork.take(),
                )
            };
            let old_active = space::leave_current();
            let old_binding = self.mm.lock().replace(binding);
            *self.context.borrow_mut() = image.context;
            space::enter_current(Some(active));
            (old_runtime, old_binding, old_active, completion)
        };
        drop(old_runtime);
        drop(old_binding);
        drop(old_active);
        drop(old_mm);
        if let Some(completion) = completion {
            completion.complete();
        }
    }

    /// 身份拥有private信号状态；裸内核probe没有Linux信号绑定。
    pub(crate) fn signals(&self) -> Option<Arc<ThreadSignals>> {
        self.identity().map(|identity| identity.signals())
    }

    /// 升级原弱执行关联以观察稳定Task，不建立另一个线程表或改变调度状态。
    pub(crate) fn executor(&self) -> Option<Arc<Task>> {
        self.executor.get().and_then(Weak::upgrade)
    }

    /// 通知稳定执行者在自己的安全返回点处理工作，不远端改现场或调度状态。
    pub(crate) fn kick(&self) {
        if let Some(executor) = self.executor() {
            crate::sched::request_user_work(&executor);
        }
    }

    /// 借用Ready前绑定的线程账；accounting短段不克隆拥有或取得业务锁。
    pub(crate) fn user_time(&self) -> Option<&UserTime> {
        self.user_time.get()
    }

    /// 累计已结束的真实执行片段；无进程身份的内核probe不产生LinuxCPU账。
    pub(crate) fn account_execution(&self, user: bool, ns: u64) {
        if let Some(time) = self.user_time.get() {
            time.account_execution(user, ns);
        }
    }

    /// 当前执行者保存原请求及首次deadline，供restart_syscall领取。
    pub(crate) fn save_restart(&self, state: Restart) {
        self.assert_current();
        self.restart.set(Some(state));
    }

    /// restart_syscall和sigreturn各自一次取走旧restart block。
    pub(crate) fn take_restart(&self) -> Option<Restart> {
        self.assert_current();
        self.restart.take()
    }

    /// 临时等待保存进入前 mask，由普通返回恢复或下一实际 handler 帧消费一次。
    pub(crate) fn save_mask(&self, mask: u64) {
        self.assert_current();
        if self.saved_mask.get().is_none() {
            self.saved_mask.set(Some(mask));
        }
    }

    /// 普通返回或无 handler 重放前结束临时 mask 责任，参数交付失败也不遗留保存值。
    pub(crate) fn restore_saved_mask(&self) {
        self.assert_current();
        if let Some(mask) = self.saved_mask.take() {
            self.signals()
                .expect("saved signal mask has no signal binding")
                .replace_mask(mask);
        }
    }

    /// 实际 handler 帧保存临时等待之前的 mask，否则保留选择时的快照。
    fn mask_to_restore(&self, selected: u64) -> u64 {
        self.saved_mask.take().unwrap_or(selected)
    }

    /// 当前用户循环先解除运行拥有，再用返回的身份发表退出；CPU仍独立保活活动根。
    pub(crate) fn release_process(&self) -> Arc<UserIdentity> {
        self.assert_current();
        crate::accounting::detach_user();
        let mm = self.mm.lock().take();
        let process = self
            .process
            .lock()
            .take()
            .expect("exiting user thread has no process resources");
        let UserProcess {
            identity,
            runtime,
            clear_child_tid,
            vfork,
            ..
        } = process;
        if let Some(binding) = &mm {
            clear_tid(binding.mm(), clear_child_tid);
        }
        drop(runtime);
        drop(mm);
        if let Some(completion) = vfork {
            completion.complete();
        }
        identity
    }

    /// 只由所绑定的 Running Task 短暂访问现场，不得持此 guard 睡眠或调度。
    pub(crate) fn context(&self) -> RefMut<'_, UserContext> {
        self.assert_current();
        self.context.borrow_mut()
    }

    /// 真正执行用户指令并返回原 Rust 调用栈；中断分派结束后才归还现场独占借用。
    pub(crate) fn run_user(&self) -> UserTrap {
        self.assert_current();
        let mm = self.mm().expect("running user thread has no MM");
        let mut context = self.context.borrow_mut();
        let _irq = IrqSaveGuard::new();
        let active = space::current_mm().expect("running user thread has no active MM");
        assert!(
            Arc::ptr_eq(&active, &mm),
            "running user thread has another active MM"
        );
        let local = crate::cpu::current();
        crate::accounting::user_enter(self);
        let timer_before = local.timer_interrupts.load(Ordering::Relaxed);
        // SAFETY: 弱关联已核对当前唯一 Running Task，RefCell 的独占借用排除现场别名；
        // IRQ guard 关闭本地中断，MM 与 CPU 活动拥有保证全部根、用户页及内核映射存活。
        // 架构在同一调用栈返回，IRQ 关闭且硬中断 guard 已结束，期间不切换 Task。
        let trap = unsafe { arch::run_user(&mut context) };
        #[cfg(feature = "ktest")]
        self.user_cpu_mask.insert(local.id, Ordering::Relaxed);
        crate::accounting::user_leave();
        if local.timer_interrupts.load(Ordering::Relaxed) != timer_before {
            self.user_timer_interrupts.set(
                self.user_timer_interrupts
                    .get()
                    .checked_add(1)
                    .expect("user timer interrupt count overflow"),
            );
        }
        trap
    }

    /// 取走自上次观察以来真实返回用户入口的 CPU 位集合。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_take_user_cpu_mask(&self) -> crate::cpu::CpuSet {
        self.user_cpu_mask.take(Ordering::AcqRel)
    }

    /// 用户入口正常结束时先解绑运行资源，最后 MM 拥有在锁外释放。
    pub(crate) fn release_mm(&self) {
        let mm = self.mm.lock().take();
        drop(mm);
    }

    /// 核对稳定执行者身份，并拒绝中断域或没有普通 Task 的调用。
    fn assert_current(&self) {
        let executor = self
            .executor
            .get()
            .and_then(Weak::upgrade)
            .expect("user executor is not alive");
        let current = task::current_task();
        assert_eq!(
            current.id(),
            executor.id(),
            "user context belongs to another task"
        );
    }
}

/// 尚未发表的完整程序只保留本次身份、用户对象和原调度任务，失败由普通拥有关系归还。
pub(crate) struct PreparedProgram {
    identity: PreparedIdentity,
    user: Arc<UserThread>,
    task: PreparedTask,
    execution: Box<PreparedExecution>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ProgramError {
    Image(ExecError),
    Identity(ProcessError),
}

impl From<ExecError> for ProgramError {
    /// 保留映像准备的格式、参数、文件与 VM 原因。
    fn from(error: ExecError) -> Self {
        Self::Image(error)
    }
}

impl From<FsError> for ProgramError {
    /// 根文件准备错误沿映像文件原因传递，不替换成缺失程序。
    fn from(error: FsError) -> Self {
        Self::Image(ExecError::File(error))
    }
}

impl From<ProcessError> for ProgramError {
    /// 身份预算及父关系错误保持可区分。
    fn from(error: ProcessError) -> Self {
        Self::Identity(error)
    }
}

impl From<AllocError> for ProgramError {
    /// 未发表用户对象、入口和执行栈的分配失败按真实 NoMemory 返回。
    fn from(_: AllocError) -> Self {
        Self::Image(ExecError::NoMemory)
    }
}

impl PreparedProgram {
    /// 在发表前取得稳定用户号码，供协调者之后选择同一子进程事件。
    pub(crate) fn pid(&self) -> usize {
        self.identity.identity().tid()
    }

    /// 身份发表成功后才移交Ready；没有parent_tid交付的普通启动走同一提交。
    pub(crate) fn publish(self) -> Result<JoinHandle, ProcessError> {
        self.publish_to(None)
    }

    /// 创建已成功时parent_tid写失败不撤child，尝试写入完成后才允许child运行。
    pub(crate) fn publish_child(
        self,
        parent_mm: &Mm,
        parent_tid: Option<usize>,
    ) -> Result<JoinHandle, ProcessError> {
        self.publish_to(parent_tid.map(|address| (parent_mm, address)))
    }

    /// 在合法关系发表和Ready之间向父Mm复制小端TID；部分写入或错误不撤销创建。
    /// 用户访问可处理COW并睡眠，期间不持图锁或用户现场借用。
    fn publish_to(self, parent_tid: Option<(&Mm, usize)>) -> Result<JoinHandle, ProcessError> {
        let pid = self.pid();
        let Self {
            identity,
            user,
            task,
            execution,
        } = self;
        identity.publish(Arc::downgrade(&user), execution)?;
        if let Some((mm, address)) = parent_tid {
            // child已经建立，父地址错误不改变创建结果；写入仍走父自己的COW/uaccess。
            let bytes = (pid as u32).to_le_bytes();
            let _ = mm.copyout(address, &bytes);
        }
        Ok(task.publish())
    }

    /// 内核协调者在Ready前取得拥有快照，可为真实并发测试保留原文件表预留责任。
    pub(crate) fn user(&self) -> Arc<UserThread> {
        self.user.clone()
    }
}

/// 在普通任务锁外完成身份、私有现场、独立运行资源和内核栈的全部 fallible 准备。
pub(crate) fn prepare_program(
    graph: &Arc<TaskGraph>,
    parent: Option<&Arc<ThreadGroup>>,
    image: LoadedImage,
    runtime: UserRuntime,
    placement: Placement,
) -> Result<PreparedProgram, ProgramError> {
    let identity = graph.prepare_process(
        parent,
        runtime.sighand.clone(),
        runtime.credentials.uid,
        SignalInheritance::EMPTY,
        SIGCHLD,
    )?;
    prepare_bound_user(identity, image, runtime, placement, None, 0)
}

/// 将进程或线程身份与完整私有现场绑定，成员离栈观察同样在发表前fallible准备。
pub(crate) fn prepare_bound_user(
    identity: PreparedIdentity,
    image: LoadedImage,
    runtime: UserRuntime,
    placement: Placement,
    child_set_tid: Option<usize>,
    clear_child_tid: usize,
) -> Result<PreparedProgram, ProgramError> {
    let user = UserThread::try_new(image.mm, image.context)?;
    user.bind_process(identity.identity(), runtime, child_set_tid, clear_child_tid);
    let task = task::prepare_user(Arc::clone(&user), placement, run_program)?;
    let execution = PreparedExecution::try_new(task.observer())?;
    Ok(PreparedProgram {
        identity,
        user,
        task,
        execution,
    })
}

#[cfg(target_arch = "loongarch64")]
struct AleMemory<'a>(&'a Mm);

#[cfg(target_arch = "loongarch64")]
impl AleMemory<'_> {
    /// 短映射读侧只访问一个字节，缺页或 COW 在回调外完成后重试。
    fn with_byte<R>(
        &self,
        address: usize,
        access: UserAccess,
        mut visit: impl FnMut(usize) -> R,
    ) -> Result<R, VmError> {
        loop {
            if let Some(value) = self.0.with_page(address, access, &mut visit)? {
                return Ok(value);
            }
            self.0.fault(address, access)?;
        }
    }
}

#[cfg(target_arch = "loongarch64")]
impl UserMemory for AleMemory<'_> {
    /// 只在可读页保活期间执行原始字节复制，不构造用户内容引用。
    fn read_byte(&mut self, address: usize) -> Result<u8, VmError> {
        self.with_byte(address, UserAccess::Read, |source| {
            let mut value = 0u8;
            // SAFETY: with_page 保活当前可读映射，单字节必定位于页内；
            // 内核目标独占且已初始化，两侧不重叠，原始用户指针不逃逸。
            unsafe { arch::copy_user_bytes(&mut value, source as *const u8, 1) };
            value
        })
    }

    /// 只在可写且非 COW 的驻留页保活期间提交一个字节。
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), VmError> {
        self.with_byte(address, UserAccess::Write, |target| {
            // SAFETY: with_page 保活当前可写非 COW 映射，单字节位于页内；
            // 源来自已初始化内核局部量，两侧不重叠，用户指针不逃逸。
            unsafe { arch::copy_user_bytes(target as *mut u8, &value, 1) };
        })
    }
}

/// 取指、源快照与结果提交各自短暂借用现场；访存错误与未支持指令分别返回。
#[cfg(target_arch = "loongarch64")]
fn emulate_ale(user: &UserThread, address: usize) -> EmulationResult {
    let pc = user.context().pc;
    if address == pc || pc & 3 != 0 {
        return EmulationResult::Unsupported;
    }
    let mm = user.mm().expect("user ALE has no MM");
    let mut bytes = [0u8; 4];
    let fetched = mm.with_page(pc, UserAccess::Read, |source| {
        // SAFETY: PC 已按四字节对齐，四字节指令不会跨越 4KiB 页；
        // with_page 保活可读页，内核目标独占且足够大，指针只在回调内使用。
        unsafe { arch::copy_user_bytes(bytes.as_mut_ptr(), source as *const u8, 4) };
    });
    if !matches!(fetched, Ok(Some(()))) {
        return EmulationResult::Fault;
    }
    let Some(instruction) = Instruction::decode(u32::from_le_bytes(bytes)) else {
        return EmulationResult::Unsupported;
    };
    let source = instruction.source_value(&user.context());
    let Ok(value) = instruction.access(&mut AleMemory(&mm), address, source) else {
        return EmulationResult::Fault;
    };
    instruction.complete(&mut user.context(), value);
    EmulationResult::Emulated
}

/// 同一真实Task先向自身Mm尝试复制小端child_tid，部分写入或错误不取消创建。
/// 随后循环执行用户现场，只有返回 Rust 并结束 IRQ/现场借用后才允许等待或调度。
/// 临时 mask 保留至真实信号选择；没有 handler 时恢复尚未领取的旧 mask。
fn run_program(user: Arc<UserThread>) -> i32 {
    let identity = user.identity().expect("user entry has no identity");
    if let Some(address) = user.take_child_tid() {
        // child在自己的Mm首次入口尝试写入；地址错误不取消已经成功的创建。
        let bytes = (identity.tid() as u32).to_le_bytes();
        let _ = user
            .mm()
            .expect("child entry has no MM")
            .copyout(address, &bytes);
    }
    let mut interrupts = 0usize;
    let signals = identity.signals();
    let mut restart = None;
    let status = loop {
        if let Some(status) = wait_control(&identity) {
            break status;
        }
        crate::sched::cond_resched();
        crate::accounting::settle_current();
        user.user_time()
            .expect("user loop has no CPU time")
            .flush_cpu_timers();
        match signals.select() {
            Some(SignalSelection::Exit(status)) => {
                identity.group().signals().notify();
                break status;
            }
            Some(SignalSelection::Stop) => {
                identity.group().signals().notify();
                continue;
            }
            Some(SignalSelection::Handler {
                info,
                action,
                restore_mask,
                altstack,
            }) => {
                if let Some(state) = restart.take() {
                    crate::syscall::resolve_restart(&user, state, Some(action.flags));
                }
                let args = SignalFrameArgs {
                    info,
                    action,
                    restore_mask: user.mask_to_restore(restore_mask),
                    altstack,
                    trampoline: arch::SIGNAL_RETURN_PAGE,
                };
                let mm = user.mm().expect("signal delivery has no MM");
                let mut candidate = user.context().fork();
                if candidate.setup_signal(&mm, &args).is_err() {
                    identity.graph().request_exit(&identity, SIGSEGV as i32);
                    break SIGSEGV as i32;
                }
                *user.context() = candidate;
            }
            None => {
                if let Some(state) = restart.take() {
                    crate::syscall::resolve_restart(&user, state, None);
                }
                user.restore_saved_mask();
            }
        }
        let trap = {
            let _irq = IrqSaveGuard::new();
            if signals.interruption_pending(WaitMode::Interruptible)
                || crate::cpu::current().need_resched.load(Ordering::Acquire)
            {
                continue;
            }
            user.run_user()
        };
        match trap {
            UserTrap::Interrupt => {
                interrupts = interrupts
                    .checked_add(1)
                    .expect("user interrupt count overflow");
            }
            UserTrap::Syscall => {
                let request = crate::syscall::Request::snapshot(&user.context());
                match crate::syscall::dispatch(&user, request) {
                    crate::syscall::Outcome::Return(result) => {
                        request.return_to(&mut user.context(), result)
                    }
                    crate::syscall::Outcome::Restored => {}
                    crate::syscall::Outcome::NewImage => restart = None,
                    crate::syscall::Outcome::Restart(state) => restart = Some(state),
                    crate::syscall::Outcome::Exit(status) => break status,
                    crate::syscall::Outcome::GroupExit(status) => {
                        identity.graph().request_exit(&identity, status);
                        break status;
                    }
                }
            }
            UserTrap::PageFault { address, access } => {
                let pc = user.context().pc;
                if let Err(error) = user
                    .mm()
                    .expect("user fault has no MM")
                    .fault(address, access)
                {
                    let (signum, code) = fault_signal(error);
                    if signum == SIGKILL {
                        identity.graph().request_exit(&identity, SIGKILL as i32);
                        break SIGKILL as i32;
                    }
                    signals.force_fault(SignalInfo {
                        signum,
                        errno: 0,
                        code,
                        payload: SignalPayload::Fault {
                            address,
                            pc,
                            trap: access as usize,
                        },
                    });
                }
            }
            UserTrap::Exception { code, address } => {
                #[cfg(target_arch = "loongarch64")]
                let ale_fault = if code == arch::ECODE_ALE {
                    match emulate_ale(&user, address) {
                        EmulationResult::Emulated => continue,
                        EmulationResult::Unsupported => false,
                        EmulationResult::Fault => true,
                    }
                } else {
                    false
                };
                #[cfg(target_arch = "riscv64")]
                let context = user.context();
                #[cfg(target_arch = "loongarch64")]
                let mut context = user.context();
                let pc = context.pc;
                let fcsr = context.fcsr;
                let (signum, si_code) = exception_signal(code, fcsr);
                #[cfg(target_arch = "loongarch64")]
                let (signum, si_code) = if ale_fault {
                    (SIGSEGV, SEGV_MAPERR)
                } else {
                    (signum, si_code)
                };
                #[cfg(target_arch = "loongarch64")]
                if code == arch::ECODE_FPE {
                    context.fcsr = fcsr
                        & !((fcsr & arch::FCSR_ENABLE_MASK as usize) << arch::FCSR_CAUSE_SHIFT);
                }
                drop(context);
                let address = if matches!(signum, SIGILL | SIGTRAP | SIGFPE) {
                    pc
                } else {
                    address
                };
                signals.force_fault(SignalInfo {
                    signum,
                    errno: 0,
                    code: si_code,
                    payload: SignalPayload::Fault {
                        address,
                        pc,
                        trap: code,
                    },
                });
            }
        }
    };
    drop(identity);
    let identity = user.release_process();
    identity.graph().exit(
        &identity,
        status,
        interrupts,
        user.user_timer_interrupts.get(),
    );
    status
}

/// 地址失效仍完成退出或exec；CPU独立拥有旧根，真实用户数不包含诊断引用。
fn clear_tid(mm: &Arc<Mm>, address: usize) {
    if address != 0 && mm.user_count() > 1 {
        let _ = mm.store_user_u32(address, 0);
        let _ = crate::task::futex::wake(mm, address, false, 1, u32::MAX);
    }
}

/// 真实成员确认停止后在原控制源登记，登记后重查关闭CONT/KILL早通知窗口。
fn wait_control(identity: &UserIdentity) -> Option<i32> {
    loop {
        match identity.graph().acknowledge_control(identity) {
            SignalControl::Run => return None,
            SignalControl::Exit(status) => return Some(status),
            SignalControl::Stop => {}
        }
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(crate::wait::WaitNode::new(waiter.waker()));
        // SAFETY: identity保活原图和控制源；node固定本Task栈，登记先于node和Waiter撤除，
        // 所有控制检查和通知不持登记源锁，返回和重试都先撤链再退栈。
        let _registration = unsafe { identity.graph().control_waiters().register(node.as_mut()) };
        match identity.graph().acknowledge_control(identity) {
            SignalControl::Run => return None,
            SignalControl::Exit(status) => return Some(status),
            SignalControl::Stop => waiter.park(),
        }
    }
}

/// 将发生时VM原因保存为siginfo，EOF与IO保持SIGBUS，内存耗尽请求合法点退出。
fn fault_signal(error: VmError) -> (u8, i32) {
    match error {
        VmError::NoMemory
        | VmError::File(FsError::NoMemory | FsError::Io(crate::block::BlockError::NoMemory)) => {
            (SIGKILL, 0)
        }
        VmError::BeyondEof => (SIGBUS, BUS_ADRERR),
        VmError::NotMappable => panic!("user fault has no file mapping capability"),
        VmError::File(_) => (SIGBUS, BUS_OBJERR),
        VmError::Permission => (SIGSEGV, SEGV_ACCERR),
        VmError::InvalidRange | VmError::Occupied | VmError::Unmapped => (SIGSEGV, SEGV_MAPERR),
    }
}

/// 将实际同步异常保存为目标信号与si_code，不改变内核fatal路径。
#[cfg(target_arch = "riscv64")]
fn exception_signal(code: usize, _fcsr: usize) -> (u8, i32) {
    match code {
        arch::EXC_INST_MISALIGNED | arch::EXC_LOAD_MISALIGNED | arch::EXC_STORE_MISALIGNED => {
            (SIGBUS, BUS_ADRALN)
        }
        arch::EXC_INST_ACCESS | arch::EXC_LOAD_ACCESS | arch::EXC_STORE_ACCESS => {
            (SIGSEGV, SEGV_ACCERR)
        }
        arch::EXC_BREAKPOINT => (SIGTRAP, TRAP_BRKPT),
        _ => (SIGILL, ILL_ILLOPC),
    }
}

/// 对齐异常、断点和非法指令保持目标同步事实，内核异常仍由架构独立处理。
#[cfg(target_arch = "loongarch64")]
fn exception_signal(code: usize, fcsr: usize) -> (u8, i32) {
    match code {
        arch::ECODE_PPI => (SIGSEGV, SEGV_ACCERR),
        arch::ECODE_ADE | arch::ECODE_BCE => (SIGBUS, BUS_ADRERR),
        arch::ECODE_ALE => (SIGBUS, BUS_ADRALN),
        arch::ECODE_BRK => (SIGTRAP, TRAP_BRKPT),
        arch::ECODE_FPE => (
            SIGFPE,
            if fcsr & (1 << (arch::FCSR_CAUSE_SHIFT + 4)) != 0 {
                FPE_FLTINV
            } else if fcsr & (1 << (arch::FCSR_CAUSE_SHIFT + 3)) != 0 {
                FPE_FLTDIV
            } else if fcsr & (1 << (arch::FCSR_CAUSE_SHIFT + 2)) != 0 {
                FPE_FLTOVF
            } else if fcsr & (1 << (arch::FCSR_CAUSE_SHIFT + 1)) != 0 {
                FPE_FLTUND
            } else if fcsr & (1 << arch::FCSR_CAUSE_SHIFT) != 0 {
                FPE_FLTRES
            } else {
                FPE_FLTUNK
            },
        ),
        _ => (SIGILL, ILL_ILLOPC),
    }
}
