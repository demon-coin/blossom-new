//! 用户态 ELF 程序集成测试的核心夹具与套件生命周期管理模块。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统与用户态集成测试层（`fs::ktest::user`），提供运行用户态 ELF 测试程序所需的
//! 套件级设备库存（`UserSuite`）与用例级执行夹具（`UserOwner`）。
//! 上游由 `fs::ktest` 注册各测试套件锚点（`anchor`、`write_anchor`、`namespace_anchor` 等），
//! 下游整合 `PathOwner`（块设备与 ext4/devfs 挂载）、`TaskGraph`（进程组/会话身份树）、
//! `exec::prepare`（ELF 镜像加载）以及 `NetStack` / `UartStage`（网络与串口外设协同）。
//!
//! # 核心生命周期与状态流转
//! 每个测试用例在共享 `UserSuite` 设备库存的基础上，独立经历以下阶段：
//!
//! ```text
//! [UserSuite::set_up]  扫描并初始化底层块设备库存（整个测试套件共享一次）
//!        |
//!        v
//! [UserOwner::set_up]  记录任务元数据基线，为单条用例初始化 PathOwner
//!        |
//!        +-- prepare / prepare_tty / prepare_system
//!        |     挂载只读/读写 ext4 根文件系统（及可选 /dev 控制台），创建 TaskGraph 身份树
//!        |
//!        +-- load / load_named + start_program
//!        |     解析 ELF 镜像与初始栈参数，绑定目标 CPU（或任意 CPU），发布用户态根进程
//!        |
//!        +-- wait + reap_children
//!        |     等待根进程退出并确认其内核线程已完全退出内核栈（Join），随后回收所有子孙进程
//!        v
//! [UserOwner::quiesce] 释放预留描述符、向进程树发送退出请求、恢复块设备完成队列与网络配置、检查串口阶段已关闭
//!        |
//!        v
//! [UserOwner::tear_down] 销毁网络栈与文件系统视图，校验内存六域与任务/工作队列元数据无泄漏
//! ```
//!
//! # 锁与资源回收顺序契约
//! 1. **先停业务任务，后拆底层设施**：在 `quiesce` 中必须先唤醒可能阻塞在 `FileTable` 预留槽位的辅助内核线程（`release_reserved`），
//!    再通过 `wait` 与 `reap_children` 确保进程树中的所有用户线程均已退出并脱离内核栈（`JoinHandle::join` 完成），最后才能销毁 `TaskGraph` 与 `PathOwner`。
//! 2. **块设备完成队列恢复优先于 `wait`**：若测试期间主动挂起了块设备完成中断（如模拟 I/O 停顿），在 `quiesce` 等待用户进程退出前必须先调用 `test_resume_completions()`，否则等待磁盘 I/O 的用户进程将无法退出从而导致死锁。

use crate::fs::ktest::path::PathOwner;
use crate::fs::ktest::tty::SerialTtyOwner;
use crate::fs::ktest::user::network::NetworkMode;
use crate::fs::ktest::*;
use crate::mm::space::{Mm, VmError};
use crate::net::route::{DefaultRouteRequest, EthernetConfig, InterfaceSnapshot, RouteSnapshot};
use crate::net::{NetStack, NetStackOwner, NetStackRef};
use crate::task::exec::{self, Arguments, ExecError, LoadedImage};
use crate::task::process::{RootRole, TaskGraph, WaitEvent, WaitSelector, MAX_PID};
use crate::task::{Credentials, JoinObserver, Placement, TaskId, UserRuntime};
use crate::workqueue::testing::{metadata_stats, MetadataStats};

mod affinity;
mod background;
mod descriptors;
mod events;
mod futex;
mod jobs;
mod layout;
mod libc;
mod lifecycle;
mod namespace;
mod network;
mod observation;
mod pipe;
mod preparation;
mod readahead;
mod resources;
mod runtime;
mod signal;
mod stack;
mod system;
mod tmpfs;
mod tty;
mod write;

/// 默认用户态冒烟测试 ELF 程序路径。
const PROGRAM: &[u8] = b"/test/bin/user-smoke";
/// 用户态与内核测试线程共享的 Futex/事件协同控制页虚拟地址。
const FUTEX_CONTROL: usize = 0x4400_0000;

/// 网络控制类测试修改网卡配置前的接口与路由表快照（包含 2 个接口快照与 3 条路由快照），用于在 `quiesce` 阶段恢复初始网络状态。
type NetworkControlSnapshot = ([Option<InterfaceSnapshot>; 2], [Option<RouteSnapshot>; 3]);

/// 正在运行的主测试用户进程状态跟踪结构。
struct RunningProgram {
    /// 用户态根进程的 PID。
    pid: usize,
    /// 当启用跨核亲和性绑定时，记录预期的目标运行 CPU 编号；未绑定时为 `None`。
    target: Option<usize>,
    /// 发起启动操作的协调者内核线程所在 CPU 编号。
    coordinator: usize,
    /// 预期的根线程内核任务 ID（`TaskId`），用于校验 `wait` 回收的事件是否来自同一个执行体。
    expected_task: Option<TaskId>,
    /// 根线程的内核任务汇合句柄（`JoinHandle`），用于等待其完全退出内核栈。
    join: JoinHandle,
}

/// 预先占用用户进程文件描述符表 0 号槽位的辅助内核任务控制结构。
struct ReservedDescriptor {
    /// 通知辅助内核任务释放 0 号文件描述符预留槽位的完成信号。
    release: Arc<Completion>,
    /// 辅助内核任务的汇合句柄（`JoinHandle`）。
    join: JoinHandle,
}

/// 普通用户态与文件系统测试共享的套件级设备库存夹具。
struct UserSuite {
    /// 套件初始化阶段发现并持有的底层块设备库存。
    inventory: DeviceInventory,
}

impl Fixture for UserSuite {
    /// 在套件首个用例开始前发现并初始化块设备库存（此阶段不启动网络协议栈或串口接收）。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let inventory = <DeviceInventory as Fixture>::set_up(context, &())?;
        Ok(Self { inventory })
    }

    /// 在套件所有用例及其路径资源停止后，使底层块设备库存进入静默状态。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        <DeviceInventory as Fixture>::quiesce(&mut self.inventory, context)
    }

    /// 销毁块设备库存并校验设备级资源是否完整归还。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <DeviceInventory as Fixture>::tear_down(self.inventory, context)
    }
}

/// 单条用户态测试用例的资源与生命周期管理器。
struct UserOwner {
    /// 本用例的文件系统路径、VFS 挂载视图与块设备缓存管理器。
    path: PathOwner,
    /// 本用例可选挂载的独立网络协议栈引用。
    net: Option<NetStackRef>,
    /// 网络控制类用例在修改接口/路由前保存的快照；即使测试中途异常退出，也会由 `quiesce` 恢复。
    network_control: Option<NetworkControlSnapshot>,
    /// 用例开始前的任务与工作队列元数据统计基线，用于在 `tear_down` 时校验是否发生泄漏。
    metadata_before: MetadataStats,
    /// 本用例的用户进程身份关系图（`TaskGraph`）。
    graph: Option<Arc<TaskGraph>>,
    /// 当前正在受控运行的根用户进程状态。
    running: Option<RunningProgram>,
    /// 可选的 0 号文件描述符预留占用任务（用于测试首个可用 FD 非 0 的边界场景）。
    reserved: Option<ReservedDescriptor>,
    /// 可选的后台状态观察内核线程汇合句柄。
    observer: Option<JoinHandle>,
    /// 标记本用例是否开启了内核事件追踪（Trace）；若开启则由 `quiesce` 负责停止捕获。
    observation_trace: bool,
    /// 进程生命周期测试中额外跟踪的子线程离栈观察器。
    lifecycle_child: Option<JoinObserver>,
    /// 宿主双工串口（UART）测试阶段的协议握手与配额控制器。
    tty_io: crate::fs::ktest::tty::UartStage,
}

impl UserOwner {
    /// 使用默认只读模式与 4 个挂载槽位容量初始化本用例的文件系统视图与内核根进程图（`RootRole::Kernel`）。
    fn prepare(&mut self, context: &TestContext<'_>) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadOnly, 4)
    }

    /// 按指定的块设备访问模式（只读/读写）与最大挂载槽位数初始化文件系统路径，并创建 `RootRole::Kernel` 类型的进程身份图。
    fn prepare_mode(
        &mut self,
        context: &TestContext<'_>,
        mode: AccessMode,
        mount_capacity: usize,
    ) -> TestResult {
        self.path.prepare_path(context, mode, mount_capacity)?;
        self.graph = Some(
            TaskGraph::try_new(MAX_PID, RootRole::Kernel).map_err(|error| {
                context.diagnostic(format_args!("user-graph-prepare error={:?}", error));
                TestError("user identity graph preparation failed")
            })?,
        );
        Ok(())
    }

    /// 按指定挂载槽位容量准备只读根文件系统，并将 `devfs` 挂载到 `/dev` 目录以提供控制台与串口设备节点。
    fn prepare_tty(&mut self, context: &TestContext<'_>, mount_capacity: usize) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadOnly, mount_capacity)?;
        self.mount_console_devfs(context)
    }

    /// 准备 8 槽位只读根文件系统，创建系统级进程身份图（`RootRole::System`），并在 `/dev` 挂载 `devfs`。
    fn prepare_system(&mut self, context: &TestContext<'_>) -> TestResult {
        self.path.prepare_path(context, AccessMode::ReadOnly, 8)?;
        self.graph = Some(
            TaskGraph::try_new(MAX_PID, RootRole::System).map_err(|error| {
                context.diagnostic(format_args!("user-system-graph-prepare error={:?}", error));
                TestError("system identity graph preparation failed")
            })?,
        );
        self.mount_console_devfs(context)
    }

    /// 解析根文件系统中的 `/dev` 挂载点，创建内存态 `devfs` 实例并以只读模式挂载。
    fn mount_console_devfs(&self, context: &TestContext<'_>) -> TestResult {
        let mountpoint = checked(
            context,
            self.path
                .vfs()
                .resolve(self.path.context(), b"/dev", ResolveMode::Follow),
        )?;
        let devfs = checked(context, crate::fs::devfs::try_new(0o755))?;
        checked(
            context,
            self.path.vfs().mount(
                &mountpoint,
                crate::fs::memfs::Filesystem::Memory(devfs),
                AccessMode::ReadOnly,
            ),
        )
    }

    /// 获取本用例已初始化的进程身份图（`TaskGraph`）引用。
    fn graph(&self) -> &Arc<TaskGraph> {
        self.graph.as_ref().expect("user graph was not prepared")
    }

    /// 在本用例的 VFS 视图中解析指定路径并打开对应的普通文件，返回持有底层映射生命周期的 `FileRef`。
    fn file(&self, context: &TestContext<'_>, pathname: &[u8]) -> Result<FileRef, TestError> {
        let path = checked(
            context,
            self.path
                .vfs()
                .resolve(self.path.context(), pathname, ResolveMode::Follow),
        )?;
        checked(context, crate::fs::open_path(self.path.vfs(), path))
    }

    /// 构造默认冒烟测试程序（`/test/bin/user-smoke`）的命令行参数（`argv`）、环境变量（`envp`）与默认用户栈大小。
    fn arguments(&self) -> Result<Arguments, ExecError> {
        Arguments::from_kernel(
            PROGRAM,
            &[PROGRAM, b"raw"],
            &[b"USER_SMOKE=1"],
            exec::DEFAULT_STACK_BYTES,
        )
    }

    /// 从当前 VFS 视图加载默认冒烟测试程序镜像（包含 ELF 解析、凭据绑定及动态链接解释器准备）。
    fn load(&self, context: &TestContext<'_>) -> Result<LoadedImage, TestError> {
        let arguments = self.arguments().map_err(|error| {
            context.diagnostic(format_args!("user-arguments-prepare error={:?}", error));
            TestError("user arguments preparation failed")
        })?;
        exec::prepare(
            self.path.vfs(),
            self.path.context(),
            arguments,
            Credentials::ROOT,
        )
        .map_err(|error| {
            context.diagnostic(format_args!("user-image-prepare error={:?}", error));
            TestError("user image preparation failed")
        })
    }

    /// 使用根凭据（`Credentials::ROOT`）启动已加载的用户程序镜像，默认将其绑定到非当前 CPU 上运行且不预留 0 号描述符。
    fn start(&mut self, context: &TestContext<'_>, image: LoadedImage) -> TestResult {
        let fs_context = self.path.context().clone();
        self.start_program(context, image, fs_context, true, false, Credentials::ROOT)
    }

    /// 创建用户态运行时（`UserRuntime`）与内核任务，可选地预先占用 0 号文件描述符，然后将用户进程正式发布到调度器。
    fn start_program(
        &mut self,
        context: &TestContext<'_>,
        image: LoadedImage,
        fs_context: FsContext,
        affine_group: bool,
        reserve_parent_fd: bool,
        credentials: Credentials,
    ) -> TestResult {
        // Step 1: 确保当前夹具没有尚未回收的运行中子进程，并基于给定的文件系统上下文、凭据和网络栈构建 UserRuntime。
        assert!(
            self.running.is_none(),
            "user fixture already has a running child"
        );
        let runtime = checked(
            context,
            UserRuntime::try_new(
                self.path.vfs().clone(),
                fs_context,
                credentials,
                self.net.clone(),
            ),
        )?;

        // Step 2: 选择目标 CPU（协调核的下一个 CPU），在 TaskGraph 中准备用户态根进程及其地址空间与执行上下文。
        let coordinator = cpu::current().id;
        let target = (coordinator + 1) % cpu::count();
        let prepared =
            task::prepare_program(self.graph(), None, image, runtime, Placement::Cpu(target))
                .map_err(|error| {
                    context.diagnostic(format_args!("user-program-prepare error={:?}", error));
                    TestError("user program task preparation failed")
                })?;

        // Step 3: 若调用方要求预先占用 0 号文件描述符（`reserve_parent_fd`），则在目标 CPU 上启动一个辅助内核线程，
        //         在用户进程发布前抢先预留槽位 0，并等待其就绪后再继续。
        if reserve_parent_fd {
            let files = prepared
                .user()
                .runtime()
                .expect("prepared user has no runtime")
                .files;
            let ready = Arc::try_new(Completion::new())
                .map_err(|_| TestError("reserved ready allocation failed"))?;
            let release = Arc::try_new(Completion::new())
                .map_err(|_| TestError("reserved release allocation failed"))?;
            let worker_ready = ready.clone();
            let worker_release = release.clone();
            let worker = task::prepare_kernel_on(target, move || {
                let reservation = files
                    .reserve(crate::task::process::DEFAULT_NOFILE.current as usize)
                    .expect("fresh user table has no reservation slot");
                assert_eq!(
                    reservation.fd(),
                    0,
                    "fresh user reservation is not descriptor zero"
                );
                worker_ready.complete();
                worker_release.wait();
                drop(reservation);
                0
            })
            .map_err(|_| TestError("reserved owner task preparation failed"))?;
            self.reserved = Some(ReservedDescriptor {
                release,
                join: worker.publish(),
            });
            ready.wait();
        }

        // Step 4: 将准备好的用户进程正式发布到调度器队列，并在夹具中记录其 PID、目标核与 JoinHandle。
        let pid = prepared.pid();
        let join = prepared.publish().map_err(|error| {
            context.diagnostic(format_args!("user-program-publish error={:?}", error));
            TestError("user identity publication failed")
        })?;
        self.running = Some(RunningProgram {
            pid,
            target: affine_group.then_some(target),
            coordinator,
            expected_task: Some(join.task_id()),
            join,
        });
        Ok(())
    }

    /// 使用根凭据加载指定路径（`program`）与单模式参数（`mode`）的用户态 ELF 镜像。
    fn load_named(
        &self,
        context: &TestContext<'_>,
        program: &[u8],
        mode: &[u8],
    ) -> Result<LoadedImage, TestError> {
        let arguments =
            Arguments::from_kernel(program, &[program, mode], &[], exec::DEFAULT_STACK_BYTES)
                .map_err(|error| {
                    context.diagnostic(format_args!("user-named-arguments error={:?}", error));
                    TestError("named user arguments failed")
                })?;
        exec::prepare(
            self.path.vfs(),
            self.path.context(),
            arguments,
            Credentials::ROOT,
        )
        .map_err(|error| {
            context.diagnostic(format_args!("user-named-load error={:?}", error));
            TestError("named user image failed")
        })
    }

    /// 以根凭据加载并运行指定路径与模式参数的用户态程序，校验其退出状态码等于 `expected_status` 并回收全部子进程。
    fn run_named(
        &mut self,
        context: &mut TestContext<'_>,
        program: &[u8],
        mode: &[u8],
        reserve_parent_fd: bool,
        expected_status: i32,
    ) -> Result<WaitEvent, TestError> {
        self.run_named_as(
            context,
            program,
            mode,
            reserve_parent_fd,
            expected_status,
            Credentials::ROOT,
        )
    }

    /// 以指定凭据加载并运行具名用户态程序，等待其退出并回收整个进程树与预留资源。
    fn run_named_as(
        &mut self,
        context: &mut TestContext<'_>,
        program: &[u8],
        mode: &[u8],
        reserve_parent_fd: bool,
        expected_status: i32,
        credentials: Credentials,
    ) -> Result<WaitEvent, TestError> {
        // Step 1: 组装命令行参数并从 VFS 加载目标 ELF 镜像。
        let arguments =
            Arguments::from_kernel(program, &[program, mode], &[], exec::DEFAULT_STACK_BYTES)
                .map_err(|_| TestError("named user arguments failed"))?;
        let image = exec::prepare(self.path.vfs(), self.path.context(), arguments, credentials)
            .map_err(|error| {
                context.diagnostic(format_args!("user-named-load error={:?}", error));
                TestError("named user image failed")
            })?;

        // Step 2: 根据模式首字母决定是否强制跨核亲和性断言，随后启动用户态进程。
        let affine_group = !matches!(mode.first(), Some(b't' | b'T' | b'g' | b'c'));
        let fs_context = self.path.context().clone();
        self.start_program(
            context,
            image,
            fs_context,
            affine_group,
            reserve_parent_fd,
            credentials,
        )?;

        // Step 3: 等待根进程退出；若退出状态码不符合预期，则向整个进程图广播 SIGKILL (9) 强制终止残留进程。
        let event = self.wait(context)?;
        let cancelling = event.status != expected_status;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        // Step 4: 回收进程图中所有过继给根节点的子进程，并释放可能预留的 0 号文件描述符辅助任务。
        self.reap_children(context, cancelling)?;
        self.release_reserved(context);
        context.diagnostic(format_args!(
            "user-named-result program={:?} mode={:?} status={}",
            program, mode, event.status
        ));
        Ok(event)
    }

    /// 循环调用 `wait_root` 回收进程图中所有已退出的子进程并等待其内核线程完全脱离内核栈。
    ///
    /// 当 `cancelling` 为 `false` 时，同时断言每个被回收子进程的退出状态码均为 0。
    fn reap_children(&self, context: &mut TestContext<'_>, cancelling: bool) -> TestResult {
        loop {
            match self.graph().wait_root() {
                Ok(event) => {
                    if !cancelling {
                        context.expect_eq(event.status, 0);
                    }
                    context.diagnostic(format_args!(
                        "user-child-reaped pid={} task={} status={} interrupts={} timers={} cpu={}",
                        event.pid,
                        event.task_id.as_usize(),
                        event.status,
                        event.user_interrupts,
                        event.user_timer_interrupts,
                        event.cpu
                    ));
                }
                Err(crate::task::process::ProcessError::NoChild) => return Ok(()),
                Err(error) => {
                    context.diagnostic(format_args!("user-child-reap error={:?}", error));
                    return Err(TestError("user process tree reap failed"));
                }
            }
        }
    }

    /// 触发完成信号通知占用 0 号文件描述符预留槽位的辅助内核任务退出，并等待其完全脱离内核栈。
    fn release_reserved(&mut self, context: &mut TestContext<'_>) {
        if let Some(reserved) = self.reserved.take() {
            reserved.release.complete();
            context.expect_eq(reserved.join.join(), 0);
        }
    }

    /// 等待当前受控根用户进程退出（`TaskGraph::wait`），随后等待其内核线程完全脱离内核栈（`JoinHandle::join`），
    /// 并校验两次观测到的 PID、任务 ID、运行 CPU 与退出状态码严格一致。
    fn wait(&mut self, context: &mut TestContext<'_>) -> Result<WaitEvent, TestError> {
        // Step 1: 在进程身份图上等待目标 PID 的退出事件。
        let running = self.running.as_ref().expect("user fixture has no child");
        let event = self.graph().wait(None, running.pid).map_err(|error| {
            context.diagnostic(format_args!("user-program-wait error={:?}", error));
            TestError("user child wait failed")
        })?;

        // Step 2: 取出运行记录，校验退出事件的 PID 与 TaskId 与启动时记录的执行体一致。
        let RunningProgram {
            pid,
            target,
            coordinator,
            expected_task,
            join,
        } = self.running.take().unwrap();
        context.expect_eq(event.pid, pid);
        if let Some(expected_task) = expected_task {
            context.expect_eq(event.task_id, expected_task);
        }

        // Step 3: 等待底层内核线程完全退出内核栈，并在绑定了目标核的场景下校验实际退出 CPU 与状态码。
        let task_status = join.join();
        if let Some(target) = target {
            context.expect_eq(event.cpu, target);
            context.expect_eq(task_status, event.status);
        }
        context.diagnostic(format_args!(
            "user-program-wait pid={} task={} status={} interrupts={} timers={} cpu={} coordinator_cpu={}",
            event.pid,
            event.task_id.as_usize(),
            event.status,
            event.user_interrupts,
            event.user_timer_interrupts,
            event.cpu,
            coordinator,
        ));
        Ok(event)
    }

    /// 获取底层块设备夹具预先分配的堆内存压力控制器（`HeapPressure`）可变引用，用于在 OOM 测试中精确消耗堆空间。
    fn pressure(&mut self) -> &mut crate::cache::ktest::HeapPressure {
        self.path
            .owner
            .pressure
            .as_mut()
            .expect("user pressure was not prepared")
    }

    /// 释放并销毁堆内存压力控制器，恢复空闲堆空间以便后续诊断输出与资源清理能够正常分配内存。
    fn release_pressure(&mut self) {
        drop(self.path.owner.pressure.take());
    }
}

impl Fixture<UserSuite> for UserOwner {
    /// 在单条用户态测试用例开始前记录工作队列与任务元数据基线，并基于套件设备库存初始化 `PathOwner`。
    fn set_up(context: &mut TestContext<'_>, suite: &UserSuite) -> Result<Self, TestError> {
        let metadata_before = metadata_stats();
        Ok(Self {
            path: <PathOwner as Fixture<DeviceInventory>>::set_up(context, &suite.inventory)?,
            net: None,
            network_control: None,
            metadata_before,
            graph: None,
            running: None,
            reserved: None,
            observer: None,
            lifecycle_child: None,
            observation_trace: false,
            tty_io: crate::fs::ktest::tty::UartStage::new(),
        })
    }

    /// 在单条测试用例结束或失败中止时，按安全顺序停止所有用户进程与辅助线程、恢复外设状态并静默文件系统路径。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 释放可能预留的 0 号描述符辅助任务，并向进程图广播退出信号（SIGKILL = 9）。
        self.release_reserved(context);
        if let Some(graph) = &self.graph {
            graph.request_root_exit(9);
        }

        // Step 2: 恢复所有测试磁盘的完成队列处理，防止正在等待磁盘 I/O 的用户进程无法响应退出信号。
        for disk in &self.path.owner.disks {
            disk.test_resume_completions();
        }
        // Step 3: 等待主测试进程、后台观察线程以及进程图中的所有子进程完全退出并脱离内核栈。
        if self.running.is_some() {
            self.wait(context)?;
        }

        if let Some(observer) = self.observer.take() {
            let _ = observer.join();
        }
        if self.graph.is_some() {
            self.reap_children(context, true)?;
            self.lifecycle_child_left_stack()?;
        }

        // Step 4: 停止本用例开启的追踪捕获、恢复网络接口与路由配置，并确认宿主 UART 阶段已正常结束。
        if self.observation_trace {
            checked(context, crate::trace::stop())?;
            self.observation_trace = false;
        }
        self.restore_network_control(context)?;
        if self.tty_io.active() {
            return Err(TestError("TTY UART stage is still active at quiesce"));
        }

        // Step 5: 销毁进程身份图与堆内存压力对象，最后静默底层文件系统与块设备路径。
        drop(self.graph.take());
        self.release_pressure();
        <PathOwner as Fixture<DeviceInventory>>::quiesce(&mut self.path, context)
    }

    /// 在所有进程与外设静默后销毁夹具，校验底层块设备内存六域以及任务/工作队列元数据统计是否完全恢复至用例前基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert!(
            self.running.is_none()
                && self.graph.is_none()
                && self.network_control.is_none()
                && self.reserved.is_none()
                && self.observer.is_none()
                && !self.observation_trace
                && self.lifecycle_child.is_none()
                && !self.tty_io.active(),
        );
        let before = self.metadata_before;
        drop(self.net);
        <PathOwner as Fixture<DeviceInventory>>::tear_down(self.path, context)?;

        let after = metadata_stats();
        context.diagnostic(format_args!(
            "user-resources metadata-before={:?} metadata-after={:?}",
            before, after
        ));
        if after != before {
            return Err(TestError("用户Task/worker/domain资源未归还"));
        }
        Ok(())
    }
}

/// 保活并导出所有用户态测试子套件的注册锚点，确保链接器不会剔除各子模块的测试用例。
pub(crate) fn anchor() -> &'static ::ktest::SuiteSlot {
    core::hint::black_box(network_programs::__ktest_anchor());
    core::hint::black_box(serial_programs::__ktest_anchor());
    core::hint::black_box(affinity::behavior::__ktest_anchor());
    core::hint::black_box(descriptors::descriptor_growth::__ktest_anchor());
    core::hint::black_box(resources::resource_limits::__ktest_anchor());
    core::hint::black_box(tmpfs::tmpfs_growth::__ktest_anchor());
    core::hint::black_box(stack::behavior::__ktest_anchor());
    programs::__ktest_anchor()
}

/// 导出可写文件系统用户态测试套件的注册锚点。
pub(crate) fn write_anchor() -> &'static ::ktest::SuiteSlot {
    write::anchor()
}

/// 导出命名空间（Namespace）用户态测试套件的注册锚点。
pub(crate) fn namespace_anchor() -> &'static ::ktest::SuiteSlot {
    namespace::anchor()
}

/// 导出文件预读（Readahead）与多页批量读取用户态测试套件的注册锚点。
pub(crate) fn readahead_anchor() -> &'static ::ktest::SuiteSlot {
    readahead::reads::__ktest_anchor()
}

/// 导出后台回写（Background Writeback）用户态测试套件的注册锚点。
pub(crate) fn background_anchor() -> &'static ::ktest::SuiteSlot {
    background::anchor()
}

// 通过 `include!` 将通用程序、网络程序与串口程序测试套件内联到当前模块命名空间，保持测试路径与宏锚点一致。
include!("user/programs.rs");
include!("user/network_programs.rs");
include!("user/serial_programs.rs");
