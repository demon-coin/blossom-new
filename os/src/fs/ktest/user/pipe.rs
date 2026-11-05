//! 普通管道 ELF 通过原 UserOwner 执行；共享控制字只放行已被正式观察为 Blocked 的并发线程。

use super::*;

const PROGRAM: &[u8] = b"/test/bin/user-pipe";

impl UserOwner {
    /// 本例建立原文件和身份夹具；各模式由客体验证字节、fd、信号与真实退出状态。
    pub(super) fn pipe_abi(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        if mode == b"v" {
            self.prepare_mode(context, AccessMode::ReadWrite, 4)?;
        } else {
            self.prepare(context)?;
        }

        let event = if matches!(mode, b"b" | b"e" | b"r") {
            self.pipe_blocked(context, mode)?
        } else if mode == b"p" {
            self.pipe_positioned(context)?
        } else if mode == b"u" {
            self.run_named_as(
                context,
                PROGRAM,
                mode,
                false,
                0,
                Credentials {
                    uid: 1000,
                    gid: 2000,
                },
            )?
        } else {
            self.run_named(context, PROGRAM, mode, false, 0)?
        };

        context.expect_eq(event.status, 0);
        if event.status != 0 {
            return Err(TestError("ordinary pipe ABI program failed"));
        }
        Ok(())
    }

    /// 保存真实Mm至wait结束，诊断按位置调用的实际负errno而不依赖退出编号推断。
    fn pipe_positioned(&mut self, context: &mut TestContext<'_>) -> Result<WaitEvent, TestError> {
        let image = self.load_named(context, PROGRAM, b"p")?;
        let mm = image.mm.clone();
        self.start_program(
            context,
            image,
            self.path.context().clone(),
            false,
            false,
            Credentials::ROOT,
        )?;

        let event = self.wait(context)?;
        let result = mm
            .load_user_u32(FUTEX_CONTROL + 12)
            .map_err(|_| TestError("positioned pipe result is not readable"))?
            as i32;
        context.diagnostic(format_args!(
            "user-pipe positioned-result={} status={}",
            result, event.status
        ));

        self.reap_children(context, event.status != 0)?;
        Ok(event)
    }

    /// 只有原 Task 在管道 I/O 真实阻塞后才放行子进程；退出树仍由原 wait 和 join 回收。
    fn pipe_blocked(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
    ) -> Result<WaitEvent, TestError> {
        let image = self.load_named(context, PROGRAM, mode)?;
        let mm = image.mm.clone();
        self.start_program(
            context,
            image,
            self.path.context().clone(),
            false,
            false,
            Credentials::ROOT,
        )?;

        let pid = self
            .running
            .as_ref()
            .expect("pipe program was not started")
            .pid;
        let user = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("pipe root exited before observation"))?;
        let executor = user
            .executor()
            .ok_or(TestError("pipe root executor disappeared"))?;
        let phases = if mode == b"b" { 2 } else { 1 };

        for phase in 1..=phases {
            until("pipe root did not block in the requested I/O", || {
                mm.load_user_u32(FUTEX_CONTROL) == Ok(phase)
                    && sched::test_state(&executor) == State::Blocked
            })?;
            context.expect_eq(mm.load_user_u32(FUTEX_CONTROL + 4), Ok(0));
            mm.store_user_u32(FUTEX_CONTROL + 4, 1)
                .map_err(|_| TestError("pipe child gate is not writable"))?;
            context.diagnostic(format_args!(
                "user-pipe mode={:?} phase={} native-root-blocked=1 child-gate-after-block=1",
                mode, phase
            ));
        }

        drop(executor);
        drop(user);
        drop(mm);

        let event = self.wait(context)?;
        let cancelling = event.status != 0;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        Ok(event)
    }
}
