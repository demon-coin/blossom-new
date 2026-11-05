//! 系统根与普通夹具根沿同一磁盘、标准流及用户执行生命周期消费真实角色语义。

use super::*;
use crate::task::process::ShutdownOperation;

const LIBC: &[u8] = b"/bin/libc-smoke";

impl UserOwner {
    /// 用户 PID1 自己消费所有日常子事件，内核仅观察最终退出与接受的关机意图。
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

        // 日常 wait 已由用户完成；接受关机后的剩余组才允许由 kernel 最终回收。
        self.reap_children(context, true)?;
        context.diagnostic(format_args!(
            "system-pid1 pid={} status={} operation={:?} user-orphan-wait=1 hardware-action=0",
            event.pid,
            event.status,
            self.graph().shutdown_operation()
        ));
        Ok(())
    }

    /// 数值一的普通根保持真实 SIGKILL 状态，权限断言来自映像和 Runtime 的同一凭据。
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

    /// 明确的根模式在原图发表一次真实 console 程序，用户返回事件与原离栈 join 分别消费。
    fn run_root_role(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
        credentials: Credentials,
    ) -> Result<WaitEvent, TestError> {
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
