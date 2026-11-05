//! 用户 ELF 按普通、网络、串口业务共享具体 suite 资源；每例 wait、离栈 join 与资源归还保持独立。

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

const PROGRAM: &[u8] = b"/test/bin/user-smoke";
const FUTEX_CONTROL: usize = 0x4400_0000;

/// 用户控制例修改前的接口与路由快照，由原 quiesce 恢复。
type NetworkControlSnapshot = ([Option<InterfaceSnapshot>; 2], [Option<RouteSnapshot>; 3]);

struct RunningProgram {
    pid: usize,
    target: Option<usize>,
    coordinator: usize,
    expected_task: Option<TaskId>,
    join: JoinHandle,
}

struct ReservedDescriptor {
    release: Arc<Completion>,
    join: JoinHandle,
}

/// 普通用户与文件测试只共享一次发现的设备库存。
struct UserSuite {
    inventory: DeviceInventory,
}

impl Fixture for UserSuite {
    /// 稳定库存先于 case 基线准备，不启动网络或串口接收。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let inventory = <DeviceInventory as Fixture>::set_up(context, &())?;
        Ok(Self { inventory })
    }

    /// case 的任务和路径结束后停止原库存。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        <DeviceInventory as Fixture>::quiesce(&mut self.inventory, context)
    }

    /// 消费同一库存，保留原设备资源归还检查。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <DeviceInventory as Fixture>::tear_down(self.inventory, context)
    }
}

struct UserOwner {
    path: PathOwner,
    net: Option<NetStackRef>,
    /// 控制例保存修改前的小值，用户异常退出后仍由原 fixture 恢复共享域。
    network_control: Option<NetworkControlSnapshot>,
    metadata_before: MetadataStats,
    graph: Option<Arc<TaskGraph>>,
    running: Option<RunningProgram>,
    reserved: Option<ReservedDescriptor>,
    observer: Option<JoinHandle>,
    /// 观测例负责停止自己开启的固定捕获，失败清理仍沿原用户 owning 边界。
    observation_trace: bool,
    lifecycle_child: Option<JoinObserver>,
    /// 本例共享UART协议阶段的显式owner。
    tty_io: crate::fs::ktest::tty::UartStage,
}

impl UserOwner {
    /// 每例只用本例设备与根建立身份域，库存由同一 suite 保活到最后一个 case。
    fn prepare(&mut self, context: &TestContext<'_>) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadOnly, 4)
    }

    /// 显式卷权限与挂载容量决定本例视图，普通消费者始终建立 Kernel 根图。
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

    /// 按本例显式容量准备根视图，再把真实 devfs 挂在 ext4 的空 `/dev` 目录。
    fn prepare_tty(&mut self, context: &TestContext<'_>, mount_capacity: usize) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadOnly, mount_capacity)?;
        self.mount_console_devfs(context)
    }

    /// 系统例直接在同源八槽视图中构造唯一系统图，准备失败仍由原夹具回收。
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

    /// 所有 console 消费者在原 ext4 的实际挂载点安装同一种 devfs。
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

    /// 仅借本例已构造的身份图，不取得调度队列或运行资源拥有。
    fn graph(&self) -> &Arc<TaskGraph> {
        self.graph.as_ref().expect("user graph was not prepared")
    }

    /// 正式逐组件解析并打开真实普通文件，文件引用保活根与同源 FileMapping。
    fn file(&self, context: &TestContext<'_>, pathname: &[u8]) -> Result<FileRef, TestError> {
        let path = checked(
            context,
            self.path
                .vfs()
                .resolve(self.path.context(), pathname, ResolveMode::Follow),
        )?;
        checked(context, crate::fs::open_path(self.path.vfs(), path))
    }

    /// 内核初次参数来自已拥有字节，与普通启动使用相同初始栈契约。
    fn arguments(&self) -> Result<Arguments, ExecError> {
        Arguments::from_kernel(
            PROGRAM,
            &[PROGRAM, b"raw"],
            &[b"USER_SMOKE=1"],
            exec::DEFAULT_STACK_BYTES,
        )
    }

    /// 从本例文件系统快照准备真实程序，路径、凭据和解释器解析共用正式入口。
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

    /// 原首ELF保持真实远核退出断言，具名后续负载复用同一准备与发表。
    fn start(&mut self, context: &TestContext<'_>, image: LoadedImage) -> TestResult {
        let fs_context = self.path.context().clone();
        self.start_program(context, image, fs_context, true, false, Credentials::ROOT)
    }

    /// 映像与运行资源使用调用者同一 FS 快照，完整预留准备后才发表身份和普通用户 Task。
    fn start_program(
        &mut self,
        context: &TestContext<'_>,
        image: LoadedImage,
        fs_context: FsContext,
        affine_group: bool,
        reserve_parent_fd: bool,
        credentials: Credentials,
    ) -> TestResult {
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
        let coordinator = cpu::current().id;
        let target = (coordinator + 1) % cpu::count();
        let prepared =
            task::prepare_program(self.graph(), None, image, runtime, Placement::Cpu(target))
                .map_err(|error| {
                    context.diagnostic(format_args!("user-program-prepare error={:?}", error));
                    TestError("user program task preparation failed")
                })?;
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

    /// 具名普通程序以原始 argv 和真实根凭据经过同一程序准备入口。
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

    /// 具名模式显式传入完整预期退出状态，正常信号退出不关闭可继续运行的身份图。
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

    /// 映像与 Runtime 共用真实凭据；只有实际 wait 状态不符才永久取消本图的余下用户。
    fn run_named_as(
        &mut self,
        context: &mut TestContext<'_>,
        program: &[u8],
        mode: &[u8],
        reserve_parent_fd: bool,
        expected_status: i32,
        credentials: Credentials,
    ) -> Result<WaitEvent, TestError> {
        let arguments =
            Arguments::from_kernel(program, &[program, mode], &[], exec::DEFAULT_STACK_BYTES)
                .map_err(|_| TestError("named user arguments failed"))?;
        let image = exec::prepare(self.path.vfs(), self.path.context(), arguments, credentials)
            .map_err(|error| {
                context.diagnostic(format_args!("user-named-load error={:?}", error));
                TestError("named user image failed")
            })?;
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
        let event = self.wait(context)?;
        let cancelling = event.status != expected_status;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        self.release_reserved(context);
        context.diagnostic(format_args!(
            "user-named-result program={:?} mode={:?} status={}",
            program, mode, event.status
        ));
        Ok(event)
    }

    /// 逐个消费转亲的其他子组并等全部成员离栈，取消的退出状态不充当主体断言。
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

    /// 发出原完成token后等待预留拥有者离栈，不能先丢仍借文件表的worker。
    fn release_reserved(&mut self, context: &mut TestContext<'_>) {
        if let Some(reserved) = self.reserved.take() {
            reserved.release.complete();
            context.expect_eq(reserved.join.join(), 0);
        }
    }

    /// 消费一次用户退出事件，再等待原 Task 离栈；两次观察必须属于同一实际执行者。
    fn wait(&mut self, context: &mut TestContext<'_>) -> Result<WaitEvent, TestError> {
        let running = self.running.as_ref().expect("user fixture has no child");
        let event = self.graph().wait(None, running.pid).map_err(|error| {
            context.diagnostic(format_args!("user-program-wait error={:?}", error));
            TestError("user child wait failed")
        })?;
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

    /// 使用原设备拥有者已经预留的真实堆压力对象，不在 OOM 期准备观测存储。
    fn pressure(&mut self) -> &mut crate::cache::ktest::HeapPressure {
        self.path
            .owner
            .pressure
            .as_mut()
            .expect("user pressure was not prepared")
    }

    /// 移出并销毁原压力拥有，之后才能进行可能分配的诊断与恢复准备。
    fn release_pressure(&mut self) {
        drop(self.path.owner.pressure.take());
    }
}

impl Fixture<UserSuite> for UserOwner {
    /// 每例保留原设备清理责任；普通路径与身份不取得无关网络。
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

    /// 取消时向本例整个身份图请求合法点退出，原wait及离栈join完成后才释放两层资源。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.release_reserved(context);
        if let Some(graph) = &self.graph {
            graph.request_root_exit(9);
        }
        // 用户wait仍可能等原盘完成；只恢复消费，不提前释放DMA拥有。
        for disk in &self.path.owner.disks {
            disk.test_resume_completions();
        }
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
        if self.observation_trace {
            checked(context, crate::trace::stop())?;
            self.observation_trace = false;
        }
        self.restore_network_control(context)?;
        if self.tty_io.active() {
            return Err(TestError("TTY UART stage is still active at quiesce"));
        }
        drop(self.graph.take());
        self.release_pressure();
        <PathOwner as Fixture<DeviceInventory>>::quiesce(&mut self.path, context)
    }

    /// 原设备夹具完成离栈和六域停稳后，只读取一次本例Task/worker/domain归还计数。
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

/// 保活各用户行为叶的原生 suite 槽，所有叶仍由同一具名目录发现。
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

/// 可写消费者仍注册在原文件测试目录，不构造另一运行入口。
pub(crate) fn write_anchor() -> &'static ::ktest::SuiteSlot {
    write::anchor()
}

/// 新名字空间消费者沿唯一 user ELF 锚点进入现有具名测试目录。
pub(crate) fn namespace_anchor() -> &'static ::ktest::SuiteSlot {
    namespace::anchor()
}

/// 独立读历史与真实多原页读批次沿现有原生宏注册槽保活。
pub(crate) fn readahead_anchor() -> &'static ::ktest::SuiteSlot {
    readahead::reads::__ktest_anchor()
}

/// 卷级后台进度消费者复用原设备库存与文件测试入口。
pub(crate) fn background_anchor() -> &'static ::ktest::SuiteSlot {
    background::anchor()
}

// suite 宏消费完整 inline 模块；include 不改变原全名、锚点与 module_path。
include!("user/programs.rs");
include!("user/network_programs.rs");
include!("user/serial_programs.rs");
