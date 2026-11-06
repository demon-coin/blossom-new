//! 系统根进程（PID 1）与普通测试根进程的角色语义内核单元测试（ktest）。
//!
//! # 模块职责与架构定位
//! 本模块位于 `os/src/fs/ktest/user/system.rs`，作为 `UserOwner` 测试夹具的扩展方法，
//! 专门验证进程树根角色（`RootRole::System` vs `RootRole::Kernel`）对用户态 PID 1 进程行为的影响。
//! 上游由用户态综合测试套件调用，下游依赖 VFS、`exec::prepare`、`UserRuntime` 以及进程管理子系统（`ProcessGraph`）。
//!
//! # 核心设计与角色对比
//! 进程树支持两种根角色模式，它们在孤儿进程收养、关机请求与致命信号处理上具有严格区分的语义：
//!
//! ```text
//! +-----------------------+-----------------------------------+-----------------------------------+
//! | 特性 / 根角色         | RootRole::System (系统 init 根)   | RootRole::Kernel (普通测试夹具根) |
//! +-----------------------+-----------------------------------+-----------------------------------+
//! | 用户态 PID 1 角色     | 系统级 init 守护进程              | 受控的单次测试根进程              |
//! | 孤儿进程收养与回收    | 由用户态 PID 1 自行 wait 回收     | 退出后剩余进程组由内核统一回收    |
//! | reboot(POWER_OFF)     | 记录 PowerOff 关机意图并正常退出  | 无关机特权或由测试角色策略拦截    |
//! | 致命信号 (如 SIGKILL) | 受 init 信号保护语义约束          | 保持标准信号终止语义 (status = 9) |
//! +-----------------------+-----------------------------------+-----------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块作为测试驱动层不直接定义自旋锁；在启动和回收用户进程时，遵循 `UserOwner` 夹具的同步规范：
//!   先通过 `wait()` 等待用户态进程的主生命周期事件，再调用 `reap_children()` 回收后台残留任务并释放执行栈（`join`）。

use super::*;
use crate::task::process::ShutdownOperation;

/// 系统根与普通测试根角色测试共用的用户态测试程序路径。
const LIBC: &[u8] = b"/bin/libc-smoke";

impl UserOwner {
    /// 测试目标：验证在 `RootRole::System` 模式下，用户态 PID 1 进程能够收养并 `wait` 回收孤儿进程，且在发起关机调用后正确记录 `PowerOff` 意图。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：通过 `prepare_system` 初始化系统级根环境，确认当前进程图的根角色为 `RootRole::System`；
    /// 2. When：以 `ROOT` 凭据启动 `/bin/libc-smoke --pid1 stage7` 作为 PID 1 运行，使其在用户态派孙进程、产生孤儿并自行等待回收，最后请求关机退出；
    /// 3. Then：PID 1 以状态码 `0` 正常退出，进程图记录的关机操作为 `Some(ShutdownOperation::PowerOff)`，随后由夹具清理并回收剩余进程组。
    pub(super) fn system_pid1_adopts_orphans(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        self.prepare_system(context)?;
        context.expect_eq(self.graph().root_role(), RootRole::System);

        let event = self.run_root_role(context, b"--pid1", Credentials::ROOT)?;
        context.expect_eq(event.pid, 1);
        context.expect_eq(event.status, 0);
        context.expect_eq(
            self.graph().shutdown_operation(),
            Some(ShutdownOperation::PowerOff),
        );
        if event.status != 0
            || self.graph().shutdown_operation() != Some(ShutdownOperation::PowerOff)
        {
            return Err(TestError("system root did not request PowerOff"));
        }

        // Step 1: 日常孤儿进程的 wait 回收已在用户态 PID 1 内完成；关机意图被接受后，由内核夹具执行最终的进程组清扫。
        self.reap_children(context, true)?;
        context.diagnostic(format_args!(
            "system-pid1 pid={} status={} operation={:?} user-orphan-wait=1 hardware-action=0",
            event.pid,
            event.status,
            self.graph().shutdown_operation()
        ));
        Ok(())
    }

    /// 测试目标：验证在 `RootRole::Kernel`（普通测试夹具根）模式下，数值 PID 为 1 的普通根进程不具备系统级 init 的豁免权与关机特权，收到 `SIGKILL` 后按状态码 `9` 终止。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：通过 `prepare_tty` 初始化普通测试夹具环境，确认根角色为 `RootRole::Kernel`，并根据 `nonroot` 参数选择 `ROOT`（uid=0, gid=0）或普通用户凭据（uid=1000, gid=2000）；
    /// 2. When：以选定凭据将 `/bin/libc-smoke --fixture-root stage7` 作为数值 PID 1 启动运行；
    /// 3. Then：该进程被信号 `SIGKILL`（状态码 `9`）终止，进程图的 `shutdown_operation()` 保持为 `None`，且后台子进程被完整回收。
    pub(super) fn fixture_root_signal_semantics(
        &mut self,
        context: &mut TestContext<'_>,
        nonroot: bool,
    ) -> TestResult {
        self.prepare_tty(context, 8)?;
        context.expect_eq(self.graph().root_role(), RootRole::Kernel);
        let credentials = if nonroot {
            Credentials {
                uid: 1000,
                gid: 2000,
            }
        } else {
            Credentials::ROOT
        };

        let event = self.run_root_role(context, b"--fixture-root", credentials)?;
        context.expect_eq(event.pid, 1);
        context.expect_eq(event.status, 9);
        context.expect_eq(self.graph().shutdown_operation(), None);

        self.reap_children(context, true)?;
        context.diagnostic(format_args!(
            "fixture-root pid={} status={} uid={} gid={} operation={:?}",
            event.pid,
            event.status,
            credentials.uid,
            credentials.gid,
            self.graph().shutdown_operation()
        ));
        if event.status != 9 || self.graph().shutdown_operation().is_some() {
            return Err(TestError("fixture root signal or shutdown role mismatch"));
        }
        Ok(())
    }

    /// 在当前进程图中装载、配置控制台并发布运行指定模式的 `/bin/libc-smoke` 根进程。
    ///
    /// # 调用上下文与职责
    /// - 在普通可睡眠的测试线程上下文中调用；
    /// - 使用给定的 `credentials` 同时完成 ELF 映像权限检查（`exec::prepare`）与运行时凭据绑定（`UserRuntime::try_new`）；
    /// - 将新进程调度到相邻的目标 CPU（`(coordinator + 1) % cpu::count()`）上发布执行，随后阻塞等待其退出事件（`WaitEvent`），并将任务回收句柄保存在 `self.running` 中供后续 `reap_children` 回收内核栈。
    fn run_root_role(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
        credentials: Credentials,
    ) -> Result<WaitEvent, TestError> {
        // Step 1: 构造内核参数并准备 ELF 进程映像与用户态运行时（安装控制台标准流）。
        let arguments = Arguments::from_kernel(
            LIBC,
            &[LIBC, mode, b"stage7"],
            &[],
            exec::DEFAULT_STACK_BYTES,
        )
        .map_err(|_| TestError("root role arguments preparation failed"))?;
        let image = exec::prepare(self.path.vfs(), self.path.context(), arguments, credentials)
            .map_err(|error| {
                context.diagnostic(format_args!("root-role-image-prepare error={:?}", error));
                TestError("root role image preparation failed")
            })?;
        let runtime = checked(
            context,
            UserRuntime::try_new(
                self.path.vfs().clone(),
                self.path.context().clone(),
                credentials,
                self.net.clone(),
            ),
        )?;
        checked(context, runtime.install_console())?;

        // Step 2: 选择目标 CPU，在进程图中准备并发布根进程，随后等待其退出事件。
        let coordinator = cpu::current().id;
        let target = (coordinator + 1) % cpu::count();
        let prepared =
            task::prepare_program(self.graph(), None, image, runtime, Placement::Cpu(target))
                .map_err(|error| {
                    context.diagnostic(format_args!("root-role-task-prepare error={:?}", error));
                    TestError("root role task preparation failed")
                })?;
        let pid = prepared.pid();
        let join = prepared.publish().map_err(|error| {
            context.diagnostic(format_args!("root-role-publish error={:?}", error));
            TestError("root role publication failed")
        })?;
        self.running = Some(RunningProgram {
            pid,
            target: Some(target),
            coordinator,
            expected_task: Some(join.task_id()),
            join,
        });

        self.wait(context)
    }
}
