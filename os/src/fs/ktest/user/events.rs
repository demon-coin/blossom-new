//! 事件 ELF 复用原用户夹具；并发门控只放行真实进入生产等待的任务。

use super::*;

const PROGRAM: &[u8] = b"/test/bin/user-events";

impl UserOwner {
    /// 从只读盘运行目标事件程序，普通退出和整棵用户树的离栈回收仍由原身份图完成。
    pub(super) fn events_abi(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        self.prepare(context)?;

        let event = if matches!(mode, b"b" | b"s") {
            self.events_blocked(context, mode)?
        } else {
            self.run_named(context, PROGRAM, mode, false, 0)?
        };

        context.expect_eq(event.status, 0);
        if event.status != 0 {
            return Err(TestError("ordinary event ABI program failed"));
        }
        Ok(())
    }

    /// 三轮各自先观察调用者真实阻塞，再允许其子进程通知；旧轮完成后才进入下一轮。
    fn events_blocked(
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
            .expect("event program was not started")
            .pid;
        let user = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("event root exited before observation"))?;
        let executor = user
            .executor()
            .ok_or(TestError("event root executor disappeared"))?;

        for phase in 1..=3 {
            until("event root did not block in the requested syscall", || {
                mm.load_user_u32(FUTEX_CONTROL) == Ok(phase)
                    && sched::test_state(&executor) == State::Blocked
            })?;
            context.expect_eq(mm.load_user_u32(FUTEX_CONTROL + 4), Ok(0));
            mm.store_user_u32(FUTEX_CONTROL + 4, 1)
                .map_err(|_| TestError("event child gate is not writable"))?;
            context.diagnostic(format_args!(
                "user-events mode={:?} phase={} native-root-blocked=1 child-gate-after-block=1",
                mode, phase
            ));
        }

        drop(executor);
        drop(user);
        let event = self.wait(context)?;
        let result = mm
            .load_user_u32(FUTEX_CONTROL + 12)
            .map_err(|_| TestError("event result is not readable"))? as i32;
        context.diagnostic(format_args!(
            "user-events mode={:?} syscall-result={} status={}",
            mode, result, event.status
        ));

        drop(mm);
        let cancelling = event.status != 0;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        Ok(event)
    }
}
