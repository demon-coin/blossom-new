//! 用户线程执行现场（`UserThread`）、运行时资源绑定（`UserRuntime`）与用户态陷入/返回主循环。
//!
//! ## 模块职责与架构定位
//! 本模块将内核调度任务 `Task`、虚拟地址空间 `Mm`、进程关系身份 `UserIdentity` 以及文件系统/信号/网络运行时资源组装成完整的 Linux 用户线程执行体，并驱动从首次进入用户态到线程退出的全生命周期循环。
//! - **上游调用者**：系统初始化启动入口、`clone` 子进程/新线程派生（`crate::task::clone`）、`execve` 映像替换（`crate::task::exec`）以及系统调用处理器（`crate::syscall`）。
//! - **下游依赖**：`crate::arch`（切换用户态寄存器上下文与硬件陷入返回 `run_user`、LoongArch 未对齐访存模拟）、`crate::mm::space`（地址空间绑定、缺页处理、`copyout`）、`crate::task::process` 与 `crate::task::signal`（作业停止确认、信号领取与退出汇报）、`crate::fs`（打开初始控制台标准流、`O_CLOEXEC` 清理）。
//!
//! ## 核心设计与主循环（`run_program`）状态流转
//! 1. **两阶段准备与发布（`PreparedProgram`）**：所有可能因内存不足或资源超限失败的分配（`UserIdentity`、`UserThread`、内核栈 `PreparedTask`、离栈观察器 `PreparedExecution`）全部在任务发布前完成；任一步失败均由 RAII 自动回滚。发布时先写入 `TaskGraph`，可选向父进程地址空间拷贝 `parent_tid`，最后将 `Task` 移交就绪队列。
//! 2. **用户态执行与返回点安全协议**：
//!
//! ```text
//!   [run_program 入口: 按需 copyout child_tid]
//!                       |
//!                       v
//!       +-----> 1. wait_control(): 响应组退出或挂起暂停 (SIGSTOP/SIGCONT)
//!       |       2. cond_resched() + settle_current() + flush_cpu_timers()
//!       |       3. signals.select():
//!       |          - Exit(code) ------> 跳出循环 -> release_process() -> graph.exit()
//!       |          - Stop ------------> continue 回到步骤 1 挂起
//!       |          - Handler(args) ---> 结算 syscall restart -> 压入用户信号栈帧
//!       |          - None ------------> 结算 syscall restart -> 恢复临时 saved_mask
//!       |       4. 关中断复查 pending 信号与 need_resched -> arch::run_user(&mut ctx)
//!       |       5. 硬件陷入返回内核 (UserTrap):
//!       |          - Interrupt -------> 计数后回到步骤 1
//!       |          - Syscall ---------> syscall::dispatch (Return / Restart / NewImage / Exit)
//!       |          - PageFault -------> mm.fault() 调页/COW；失败转 SIGSEGV/SIGBUS/SIGKILL
//!       +--------- - Exception -------> (LoongArch ALE 模拟或) 注入同步强制故障信号
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - **单执行者独占字段**：`UserThread` 的 `context`（`RefCell`）、`user_timer_interrupts`、`restart`、`saved_mask` **仅允许其绑定的唯一 `Running` `Task` 访问**（入口通过 `assert_current()` 强制校验）；严禁跨越睡眠或调度点持有 `context` 的 `RefMut` 借用。
//! - **跨线程共享快照锁**：`UserThread::mm` 与 `UserThread::process` 为极短临界区自旋锁，仅用于克隆或取走 `Arc` / 句柄；严禁在持有这两把锁时执行用户内存拷贝（如 `clear_tid`、`copyout`）、获取 `TaskGraph` 锁、获取文件表睡眠锁或触发最后一次 `Arc` 析构。
//! - **`execve` 切换契约（`install_image`）**：先在原地址空间执行 `clear_tid`，随后在关中断临界区内原子切换当前 CPU 活动页表、`UserThread::mm`、`UserContext` 与 `UserRuntime`，最后在开中断锁外释放旧地址空间并唤醒阻塞的 `vfork` 父进程。

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

/// `SIGILL` 的 `si_code`：遇到非法或未定义的操作码。
const ILL_ILLOPC: i32 = 1;
/// `SIGBUS` 的 `si_code`：访存虚拟地址未满足硬件要求的对齐边界。
const BUS_ADRALN: i32 = 1;
/// `SIGBUS` 的 `si_code`：访问不存在的物理地址或越过文件映射有效范围（Beyond EOF）。
const BUS_ADRERR: i32 = 2;
/// `SIGBUS` 的 `si_code`：底层映射文件发生硬件或文件系统 I/O 错误。
const BUS_OBJERR: i32 = 3;
/// `SIGSEGV` 的 `si_code`：访问未建立映射的虚拟地址。
const SEGV_MAPERR: i32 = 1;
/// `SIGSEGV` 的 `si_code`：访问权限违例（如向只读页写入或执行不可执行页）。
const SEGV_ACCERR: i32 = 2;
/// `SIGTRAP` 的 `si_code`：命中软件或硬件断点指令。
const TRAP_BRKPT: i32 = 1;
/// `SIGFPE` 的 `si_code`：浮点除以零异常（LoongArch FPE）。
#[cfg(target_arch = "loongarch64")]
const FPE_FLTDIV: i32 = 3;
/// `SIGFPE` 的 `si_code`：浮点运算上溢异常（LoongArch FPE）。
#[cfg(target_arch = "loongarch64")]
const FPE_FLTOVF: i32 = 4;
/// `SIGFPE` 的 `si_code`：浮点运算下溢异常（LoongArch FPE）。
#[cfg(target_arch = "loongarch64")]
const FPE_FLTUND: i32 = 5;
/// `SIGFPE` 的 `si_code`：浮点运算结果不精确异常（LoongArch FPE）。
#[cfg(target_arch = "loongarch64")]
const FPE_FLTRES: i32 = 6;
/// `SIGFPE` 的 `si_code`：无效浮点操作异常（LoongArch FPE）。
#[cfg(target_arch = "loongarch64")]
const FPE_FLTINV: i32 = 7;
/// `SIGFPE` 的 `si_code`：未归类的浮点异常（LoongArch FPE）。
#[cfg(target_arch = "loongarch64")]
const FPE_FLTUNK: i32 = 14;

/// 用户进程/线程的自主访问控制凭据（UID / GID）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Credentials {
    /// 用户 ID（`0` 代表超级用户 root）。
    pub(crate) uid: u32,
    /// 主用户组 ID。
    pub(crate) gid: u32,
}

impl Credentials {
    /// 超级用户（root，`uid = 0, gid = 0`）默认凭据常量。
    pub(crate) const ROOT: Self = Self { uid: 0, gid: 0 };
}

/// 用户任务持有的文件系统命名空间视图及当前根目录/工作目录上下文。
pub(crate) struct UserFs {
    /// 虚拟文件系统挂载树实例。
    pub(crate) vfs: Arc<Vfs>,
    /// 进程当前的根目录（root）、当前工作目录（cwd）与 `umask` 上下文。
    pub(crate) context: FsContext,
}

/// 用户线程运行期绑定的外部内核子系统资源集合（支持按 `clone` 标志细粒度共享或独立复制）。
#[derive(Clone)]
pub(crate) struct UserRuntime {
    /// 打开文件描述符表（可通过 `CLONE_FILES` 共享）。
    pub(crate) files: Arc<FileTable>,
    /// 文件系统根目录、工作目录与 `umask` 状态（可通过 `CLONE_FS` 共享）。
    pub(crate) fs: Arc<SleepMutex<UserFs>>,
    /// 当前线程的用户与组身份凭据。
    pub(crate) credentials: Credentials,
    /// 信号处置动作表（可通过 `CLONE_SIGHAND` 共享）。
    pub(crate) sighand: Arc<Sighand>,
    /// 可选的网络协议栈实例引用（无网络测试环境为 `None`，派生子任务自动继承同一引用）。
    pub(crate) net: Option<NetStackRef>,
}

impl UserRuntime {
    /// 在任务发布前分配并初始化全新的文件描述符表、文件系统上下文锁与默认信号处置表。
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

    /// 在尚未对外发布的空文件描述符表中打开 `/dev/console`，并依次安装为标准输入（fd 0）、标准输出（fd 1）和标准错误（fd 2）。
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

    /// 按 `clone` 系统调用指定的三个共享维度（`CLONE_FILES`、`CLONE_FS`、`CLONE_SIGHAND`）共享或深拷贝运行时资源。
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

    /// 为 `execve` 准备独占资源：若文件描述符表或信号处置表当前正被其他活跃线程共享（`user_count() > 1`），则复制出独立副本以解除共享。
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

/// RAII 包装器：记录当前活跃用户线程对 `FileTable` 与 `Sighand` 的绑定引用计数（用于判定是否被多线程共享）。
///
/// 无论是准备阶段中途失败丢弃，还是线程正常退出解绑，`Drop` 都会对称归还活跃用户计数。
struct RuntimeBinding(UserRuntime);

impl RuntimeBinding {
    /// 绑定运行时资源，递增 `FileTable` 与 `Sighand` 的活跃执行者计数。
    fn new(runtime: UserRuntime) -> Self {
        runtime.files.add_user();
        runtime.sighand.add_user();
        Self(runtime)
    }
}

impl Drop for RuntimeBinding {
    /// 解除运行时绑定责任，递减 `FileTable` 与 `Sighand` 的活跃执行者计数。
    fn drop(&mut self) {
        self.0.files.remove_user();
        self.0.sighand.remove_user();
    }
}

/// 用户线程在存活期间持有的进程身份、运行时资源及线程退出/启动同步钩子。
struct UserProcess {
    /// 进程关系图中的线程/进程唯一身份句柄（含 PID/TID 与信号控制块）。
    identity: Arc<UserIdentity>,
    /// 计入活跃共享计数的文件、文件系统、凭据、信号处置表与网络资源绑定。
    runtime: RuntimeBinding,
    /// `CLONE_CHILD_SETTID` 指定的子线程用户态地址；子线程首次进入 `run_program` 时写入自身 TID 并取空。
    child_set_tid: Option<usize>,
    /// `CLONE_CHILD_CLEARTID` 或 `set_tid_address` 注册的用户态地址；线程退出或 `execve` 脱离旧地址空间时向其写 0 并唤醒 1 个 Futex 等待者。
    clear_child_tid: usize,
    /// `vfork` 创建子进程时绑定的完成同步量；子进程执行 `execve` 或退出释放旧地址空间后触发 `complete()` 唤醒父进程。
    vfork: Option<Arc<Completion>>,
}

/// 内核 `Task` 挂载的用户态线程扩展控制块，保存用户态寄存器上下文、地址空间与进程资源绑定。
pub struct UserThread {
    /// 指向承载本用户线程的唯一内核 `Task` 的弱引用（避免 `Task <-> UserThread` 循环强引用导致内核栈无法释放）。
    executor: Once<Weak<Task>>,
    /// 当前线程持有的用户虚拟地址空间活跃引用（计入 `Mm::user_count`）；线程退出解绑后置为 `None`。
    mm: SpinLock<Option<MmUser>>,
    /// 用户态通用/浮点/控制寄存器保存区，仅允许绑定的 `Running` `Task` 在本地独占借用。
    context: RefCell<UserContext>,
    /// 进程身份、运行时资源与 TID/vfork 钩子状态；线程退出进入 `release_process` 时整体取出。
    process: SpinLock<Option<UserProcess>>,
    /// 统计本线程在用户态执行期间遭遇本地时钟中断的次数。
    user_timer_interrupts: Cell<usize>,
    /// 任务就绪前一次性初始化的线程私有与组共享 CPU 时间统计器。
    user_time: Once<UserTime>,
    /// 最近一次被信号打断的可重启系统调用保存块（供 `ERESTARTSYS` / `restart_syscall` 恢复）。
    restart: Cell<Option<Restart>>,
    /// `rt_sigsuspend` / `ppoll` / `pselect6` 等系统调用临时替换信号掩码期间保存的原信号掩码。
    saved_mask: Cell<Option<u64>>,
    /// 内核测试（`ktest`）记录本线程曾经在哪些物理 CPU 核上实际进入并运行过用户态指令。
    #[cfg(feature = "ktest")]
    user_cpu_mask: AtomicCpuSet,
}

// SAFETY:
// 1. 非线程安全内部可变字段（`context`、`user_timer_interrupts`、`restart`、`saved_mask`）均在访问入口通过 `assert_current()` 严格限制为仅能由 `executor` 指向的唯一 `Running` `Task` 访问；`Task` 跨核迁移由调度器上下文切换屏障串行化，`RefCell` 在单线程内消除重叠可变借用；
// 2. 跨线程并发访问的 `mm` 与 `process` 字段均受 `SpinLock` 保护；
// 3. `executor` 与 `user_time` 在任务发布为 `Ready` 前通过 `Once` 完成初始化，后续仅并发读取（`UserTime` 内部采用原子变量记账）。
unsafe impl Sync for UserThread {}

impl UserThread {
    /// 在创建新用户任务时分配 `UserThread`，保存初始用户态上下文并登记对 `mm` 的活跃用户引用。
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

    /// 在底层 `PreparedTask` 尚未发布到调度器前建立指回 `Task` 的弱引用（仅允许绑定一次）。
    pub(crate) fn bind_executor(&self, executor: Weak<Task>) {
        assert!(
            self.executor.get().is_none(),
            "user executor is already bound"
        );
        self.executor.call_once(|| executor);
    }

    /// 获取当前线程绑定的用户地址空间 `Arc<Mm>` 快照；若线程已退出解绑则返回 `None`。
    pub(crate) fn mm(&self) -> Option<Arc<Mm>> {
        self.mm
            .lock()
            .as_ref()
            .map(|binding| Arc::clone(binding.mm()))
    }

    /// 在任务发布前绑定进程身份、CPU 时间账本、运行时资源以及启动/退出 TID 钩子地址。
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

    /// 由当前线程在首次进入用户态前一次性取出 `child_set_tid` 地址（后续实际的用户内存写入在锁外执行）。
    fn take_child_tid(&self) -> Option<usize> {
        self.assert_current();
        self.process
            .lock()
            .as_mut()
            .expect("user entry has no process")
            .child_set_tid
            .take()
    }

    /// 更新当前线程退出或 `execve` 时需要清零并唤醒 Futex 的用户态地址（实现 `set_tid_address` 系统调用），返回当前线程 TID。
    pub(crate) fn set_tid_address(&self, address: usize) -> usize {
        self.assert_current();
        let mut process = self.process.lock();
        let process = process
            .as_mut()
            .expect("set_tid_address has no user process");
        process.clear_child_tid = address;
        process.identity.tid()
    }

    /// 向当前用户线程自身发送由内核事件触发的线程定向信号（如 `SIGPIPE`）。
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

    /// 克隆获取当前线程的 `UserIdentity` 快照（若线程尚未解绑退出）。
    pub(crate) fn identity(&self) -> Option<Arc<UserIdentity>> {
        self.process
            .lock()
            .as_ref()
            .map(|process| Arc::clone(&process.identity))
    }

    /// 在短临界区内克隆当前线程的 `UserRuntime` 资源句柄集合，供锁外执行文件 I/O 或系统调用逻辑。
    pub(crate) fn runtime(&self) -> Option<UserRuntime> {
        self.process
            .lock()
            .as_ref()
            .map(|process| process.runtime.0.clone())
    }

    /// 在 `vfork` 子进程发布运行前绑定用于唤醒父进程的完成量句柄。
    pub(crate) fn bind_vfork(&self, completion: Arc<Completion>) {
        let mut process = self.process.lock();
        let process = process.as_mut().expect("vfork child has no process");
        assert!(process.vfork.is_none(), "vfork completion is already bound");
        process.vfork = Some(completion);
    }

    /// 当阻塞等待中的 `vfork` 父进程因致命信号提前终止时，解除子进程身上绑定的同一 `Completion` 句柄。
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

    /// 在当前线程上提交 `execve` 加载完成的新程序映像与独占运行时资源，释放旧地址空间并唤醒可能存在的 `vfork` 父进程。
    pub(crate) fn install_image(&self, image: LoadedImage, runtime: UserRuntime) {
        self.assert_current();
        // Step 1: 绑定新运行时资源，关闭带有 O_CLOEXEC 的文件描述符，并重置信号处置表与备用栈。
        let runtime = RuntimeBinding::new(runtime);
        runtime.0.files.close_on_exec();
        self.signals()
            .expect("exec has no signals")
            .reset_for_exec(runtime.0.sighand.clone());

        // Step 2: 在脱离旧地址空间之前，向旧 MM 中注册的 clear_child_tid 地址写 0 并执行 futex 唤醒。
        let old_mm = self.mm().expect("exec has no old MM");
        let clear_child_tid = {
            let mut process = self.process.lock();
            let process = process.as_mut().expect("exec has no process");
            mem::take(&mut process.clear_child_tid)
        };
        clear_tid(&old_mm, clear_child_tid);

        // Step 3: 清除上一程序的系统调用重启状态与临时信号掩码，关中断原子切换当前活动地址空间、寄存器现场与运行时绑定。
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

        // Step 4: 在锁外与开中断状态下释放旧运行时、旧地址空间引用，并通知 vfork 父进程子进程已不再共享其内存。
        drop(old_runtime);
        drop(old_binding);
        drop(old_active);
        drop(old_mm);
        if let Some(completion) = completion {
            completion.complete();
        }
    }

    /// 获取当前线程的私有信号控制块句柄（若已绑定用户进程身份）。
    pub(crate) fn signals(&self) -> Option<Arc<ThreadSignals>> {
        self.identity().map(|identity| identity.signals())
    }

    /// 尝试将弱引用升级为承载本用户线程的内核 `Arc<Task>`（若任务尚未彻底销毁）。
    pub(crate) fn executor(&self) -> Option<Arc<Task>> {
        self.executor.get().and_then(Weak::upgrade)
    }

    /// 请求承载本用户线程的内核 `Task` 在安全点尽快返回内核重新检查信号与控制状态（跨核时发送重调度/提醒）。
    pub(crate) fn kick(&self) {
        if let Some(executor) = self.executor() {
            crate::sched::request_user_work(&executor);
        }
    }

    /// 借用本线程的 `UserTime` 统计对象（供 `accounting` 在关中断路径零分配读取）。
    pub(crate) fn user_time(&self) -> Option<&UserTime> {
        self.user_time.get()
    }

    /// 将一段已结束的用户态或内核态执行时长（纳秒）记入本线程及所属线程组的 CPU 时间账本。
    pub(crate) fn account_execution(&self, user: bool, ns: u64) {
        if let Some(time) = self.user_time.get() {
            time.account_execution(user, ns);
        }
    }

    /// 保存被信号打断的系统调用重启参数块（供后续 `restart_syscall` 系统调用读取）。
    pub(crate) fn save_restart(&self, state: Restart) {
        self.assert_current();
        self.restart.set(Some(state));
    }

    /// 一次性取出已保存的系统调用重启参数块（由 `restart_syscall` 或 `rt_sigreturn` 消费）。
    pub(crate) fn take_restart(&self) -> Option<Restart> {
        self.assert_current();
        self.restart.take()
    }

    /// 保存系统调用（如 `rt_sigsuspend`）临时更改阻塞掩码前的原始信号掩码（若此前尚未保存）。
    pub(crate) fn save_mask(&self, mask: u64) {
        self.assert_current();
        if self.saved_mask.get().is_none() {
            self.saved_mask.set(Some(mask));
        }
    }

    /// 若存在尚未被信号处理帧消费的临时保存掩码，将其恢复为当前线程的正式信号阻塞掩码。
    pub(crate) fn restore_saved_mask(&self) {
        self.assert_current();
        if let Some(mask) = self.saved_mask.take() {
            self.signals()
                .expect("saved signal mask has no signal binding")
                .replace_mask(mask);
        }
    }

    /// 决定压入用户态信号处理栈帧的恢复掩码：优先取走临时等待前保存的原始掩码 `saved_mask`，否则使用信号选中时的掩码快照。
    fn mask_to_restore(&self, selected: u64) -> u64 {
        self.saved_mask.take().unwrap_or(selected)
    }

    /// 在线程退出路径上解除 CPU 记账关联，移出并释放 `Mm` 与 `UserRuntime`，执行 `clear_child_tid` 唤醒与 `vfork` 完成通知，最后返回 `UserIdentity` 供向 `TaskGraph` 汇报退出。
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

    /// 获取当前线程用户态寄存器现场的可变借用（仅限绑定的当前 `Running` `Task` 短暂持有，严禁持借用睡眠或跨调度点）。
    pub(crate) fn context(&self) -> RefMut<'_, UserContext> {
        self.assert_current();
        self.context.borrow_mut()
    }

    /// 切换到用户态执行指令直至发生下一次硬件中断、系统调用、缺页或同步异常后返回当前内核栈。
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
        // SAFETY:
        // 1. 入口 `assert_current()` 已校验当前 CPU 正在运行本 `UserThread` 绑定的唯一 `Task`，且 `context.borrow_mut()` 保证了 `&mut UserContext` 无别名冲突；
        // 2. `_irq` 守卫已关闭本地中断，且上方 `Arc::ptr_eq(&active, &mm)` 已校验当前硬件激活页表与本线程持有的 `Mm` 一致，用户页表与内核栈映射在陷入返回前全程有效；
        // 3. `arch::run_user` 保存内核被调用者保存寄存器并在同一内核栈帧上返回，期间不会发生内核任务切换，返回时本地中断仍保持关闭状态。
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

    /// 原子取走并清零自上次采样以来本用户线程实际进入运行过的物理 CPU 集合（供 `ktest` 亲和性与负载均衡验证）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_take_user_cpu_mask(&self) -> crate::cpu::CpuSet {
        self.user_cpu_mask.take(Ordering::AcqRel)
    }

    /// 单独移出并在自旋锁外释放本线程持有的 `MmUser` 地址空间引用。
    pub(crate) fn release_mm(&self) {
        let mm = self.mm.lock().take();
        drop(mm);
    }

    /// 校验当前正在执行的内核 `Task` 必须与本 `UserThread` 绑定的唯一 `executor` 一致，防止跨线程或中断上下文非法访问线程私有状态。
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

/// 已完成全部内存与身份预分配、尚未发布到 `TaskGraph` 与调度器的新用户程序句柄。
///
/// 若在调用 `publish` / `publish_child` 前发生错误丢弃，其包含的全部句柄通过常规 `Drop` 自动归还配额与内存。
pub(crate) struct PreparedProgram {
    /// 尚未挂入全局进程关系图的预备进程/线程身份。
    identity: PreparedIdentity,
    /// 已初始化好寄存器上下文与运行时资源的用户线程扩展控制块。
    user: Arc<UserThread>,
    /// 已分配好内核栈与入口闭包的预备调度任务。
    task: PreparedTask,
    /// 用于跟踪线程彻底离开内核栈的预备观察器节点。
    execution: Box<PreparedExecution>,
}

/// 准备新用户程序（`prepare_program` / `prepare_bound_user`）阶段的失败原因。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ProgramError {
    /// ELF 映像加载、文件系统读取或地址空间/内核栈内存分配失败。
    Image(ExecError),
    /// 进程数超限（`RLIMIT_NPROC`）、PID 耗尽或父进程已处于退出状态。
    Identity(ProcessError),
}

impl From<ExecError> for ProgramError {
    /// 将 ELF 映像准备阶段的错误包装为 `ProgramError::Image`。
    fn from(error: ExecError) -> Self {
        Self::Image(error)
    }
}

impl From<FsError> for ProgramError {
    /// 将文件系统访问错误转为 `ExecError::File` 并包装为 `ProgramError::Image`。
    fn from(error: FsError) -> Self {
        Self::Image(ExecError::File(error))
    }
}

impl From<ProcessError> for ProgramError {
    /// 将进程关系与配额检查错误包装为 `ProgramError::Identity`。
    fn from(error: ProcessError) -> Self {
        Self::Identity(error)
    }
}

impl From<AllocError> for ProgramError {
    /// 将内核堆分配失败（`AllocError`）统一映射为 `ExecError::NoMemory`。
    fn from(_: AllocError) -> Self {
        Self::Image(ExecError::NoMemory)
    }
}

impl PreparedProgram {
    /// 在正式发布前读取已预留的新进程/线程 TID（PID）。
    pub(crate) fn pid(&self) -> usize {
        self.identity.identity().tid()
    }

    /// 将预备身份发布到 `TaskGraph` 并将内核 `Task` 投入就绪队列（用于无需写回 `parent_tid` 的初始程序或普通派生）。
    pub(crate) fn publish(self) -> Result<JoinHandle, ProcessError> {
        self.publish_to(None)
    }

    /// 发布子进程/线程并在放入就绪队列前按需向父进程地址空间写入 `CLONE_PARENT_SETTID`（即使向父地址拷贝失败也不回滚已建立的子任务）。
    pub(crate) fn publish_child(
        self,
        parent_mm: &Mm,
        parent_tid: Option<usize>,
    ) -> Result<JoinHandle, ProcessError> {
        self.publish_to(parent_tid.map(|address| (parent_mm, address)))
    }

    /// 内部统一发布流程：先提交 `TaskGraph` 关系身份，随后在无锁可睡眠上下文向父 `Mm` 写出小端 32 位 TID，最后唤醒新 `Task` 开始调度。
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
            // 子身份已在关系图中正式确立，此处向父空间 copyout 即使触发缺页错误也不撤销子任务。
            let bytes = (pid as u32).to_le_bytes();
            let _ = mm.copyout(address, &bytes);
        }
        Ok(task.publish())
    }

    /// 在发布前克隆获取内部 `Arc<UserThread>` 快照（供内核协调或并发测试预置状态）。
    pub(crate) fn user(&self) -> Arc<UserThread> {
        self.user.clone()
    }
}

/// 在所有任务锁外完成独立新进程的完整准备（分配进程身份、绑定用户线程、分配内核栈与退出观察器）。
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

/// 将已准备好的进程或线程身份（`PreparedIdentity`）与加载好的映像、运行时资源及内核调度任务组装为 `PreparedProgram`。
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

/// LoongArch 64 用户态地址未对齐异常（ALE）软件模拟器所使用的单字节访存适配器。
#[cfg(target_arch = "loongarch64")]
struct AleMemory<'a>(&'a Mm);

#[cfg(target_arch = "loongarch64")]
impl AleMemory<'_> {
    /// 在目标虚拟页驻留且权限就绪的前提下对单字节直映地址执行 `visit`；若页未驻留或需写时复制，则在页表锁外执行 `mm.fault` 后重试。
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
    /// 从用户虚拟地址安全读取 1 个字节（不构造指向用户内存的 Rust 引用）。
    fn read_byte(&mut self, address: usize) -> Result<u8, VmError> {
        self.with_byte(address, UserAccess::Read, |source| {
            let mut value = 0u8;
            // SAFETY:
            // 1. `with_page` 持有映射与页表锁，保证 `source` 为可读驻留物理页内的有效直映地址（单字节访问绝不跨页）；
            // 2. `value` 为当前栈上已初始化的局部变量，与用户物理页内存无重叠，且裸指针不逃逸出闭包。
            unsafe { arch::copy_user_bytes(&mut value, source as *const u8, 1) };
            value
        })
    }

    /// 向用户虚拟地址安全写入 1 个字节（写时复制已在 `with_byte` 重试路径中提前解开）。
    fn write_byte(&mut self, address: usize, value: u8) -> Result<(), VmError> {
        self.with_byte(address, UserAccess::Write, |target| {
            // SAFETY:
            // 1. `with_page` 以 `UserAccess::Write` 保活当前可写且非 COW 的驻留物理页，`target` 单字节访问位于页内；
            // 2. `value` 为栈上局部变量，源与目标内存不重叠且裸指针仅在当前回调内使用。
            unsafe { arch::copy_user_bytes(target as *mut u8, &value, 1) };
        })
    }
}

/// 在 LoongArch 64 上尝试软件模拟用户态触发的未对齐访存指令（ALE）：依次取指、译码、逐字节访存并写回寄存器结果。
#[cfg(target_arch = "loongarch64")]
fn emulate_ale(user: &UserThread, address: usize) -> EmulationResult {
    let pc = user.context().pc;
    if address == pc || pc & 3 != 0 {
        return EmulationResult::Unsupported;
    }
    let mm = user.mm().expect("user ALE has no MM");
    let mut bytes = [0u8; 4];
    let fetched = mm.with_page(pc, UserAccess::Read, |source| {
        // SAFETY:
        // 1. 上方已校验 `pc & 3 == 0`（4 字节自然对齐），因此 4 字节指令必定完整落在单个 4 KiB 物理页内；
        // 2. `with_page` 保活该可读驻留页，`bytes` 为栈上独占的 4 字节缓冲区，内存不重叠且裸指针不逃逸。
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

/// 用户线程在内核态的顶层入口函数：完成首次 `child_tid` 写回后，循环驱动控制确认、定时器与信号处理、进入用户态及硬件陷入分发，直至线程退出。
fn run_program(user: Arc<UserThread>) -> i32 {
    let identity = user.identity().expect("user entry has no identity");
    if let Some(address) = user.take_child_tid() {
        // Step 1: 若设置了 CLONE_CHILD_SETTID，在首次进入用户态前向自身地址空间写入小端 32 位 TID（即使地址无效也不中止线程启动）。
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
        // Step 2: 安全点检查作业控制（暂停/退出）、主动让出 CPU、结算 CPU 时间片段并触发到期的进程 CPU 定时器。
        if let Some(status) = wait_control(&identity) {
            break status;
        }
        crate::sched::cond_resched();
        crate::accounting::settle_current();
        user.user_time()
            .expect("user loop has no CPU time")
            .flush_cpu_timers();

        // Step 3: 领取待处理信号或退出/暂停指令；若命中用户处理函数则结算系统调用重启并在用户栈上构造信号帧。
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

        // Step 4: 关中断复查待处理信号与重调度标志，确认无误后进入用户态执行直至硬件陷入返回。
        let trap = {
            let _irq = IrqSaveGuard::new();
            if signals.interruption_pending(WaitMode::Interruptible)
                || crate::cpu::current().need_resched.load(Ordering::Acquire)
            {
                continue;
            }
            user.run_user()
        };

        // Step 5: 根据陷入原因分发处理外部中断、系统调用、缺页异常或同步架构异常。
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

    // Step 6: 跳出主循环后先释放局部身份克隆，解绑地址空间与运行时资源，最后通知 TaskGraph 完成线程退出登记。
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

/// 在线程退出或 `execve` 脱离旧地址空间时处理 `clear_child_tid`：仅当地址非零且仍有其他活跃线程共享该 `Mm`（`user_count() > 1`）时，向该地址写 0 并唤醒 1 个共享 Futex 等待者。
fn clear_tid(mm: &Arc<Mm>, address: usize) {
    if address != 0 && mm.user_count() > 1 {
        let _ = mm.store_user_u32(address, 0);
        let _ = crate::task::futex::wake(mm, address, false, 1, u32::MAX);
    }
}

/// 在返回用户态前同步线程组作业控制状态：若组已退出则返回退出码；若组处于停止状态则挂入控制等待队列睡眠，直至收到 `SIGCONT` 或致命退出信号。
fn wait_control(identity: &UserIdentity) -> Option<i32> {
    loop {
        match identity.graph().acknowledge_control(identity) {
            SignalControl::Run => return None,
            SignalControl::Exit(status) => return Some(status),
            SignalControl::Stop => {}
        }

        let waiter = Waiter::new();
        let mut node = core::pin::pin!(crate::wait::WaitNode::new(waiter.waker()));
        // SAFETY:
        // 1. `identity` 强引用保活所属 `TaskGraph` 及其内部的 `control_waiters` 等待队列；
        // 2. `node` 已通过 `pin!` 固定在当前任务栈上，`_registration` 守卫作用域严格位于 `node` 与 `waiter` 之内，任何返回或下一轮循环都会先在队列锁下完成解链再销毁栈上 `node`。
        let _registration = unsafe { identity.graph().control_waiters().register(node.as_mut()) };
        match identity.graph().acknowledge_control(identity) {
            SignalControl::Run => return None,
            SignalControl::Exit(status) => return Some(status),
            SignalControl::Stop => waiter.park(),
        }
    }
}

/// 将无法通过 `mm.fault` 解决的用户态缺页错误映射为对应的同步信号编号与 `si_code`（内核内存耗尽时映射为 `SIGKILL` 请求退出）。
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

/// 将 RISC-V 64 用户态同步硬件异常码（`scause`）映射为对应的信号编号与 `si_code`。
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

/// 将 LoongArch 64 用户态同步硬件异常码（`ESTAT.Ecode`）及浮点控制状态字（`FCSR`）映射为对应的信号编号与 `si_code`。
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
