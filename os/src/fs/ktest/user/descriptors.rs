//! 描述符增长和阻塞轮询使用真实用户程序及原生关闭复用交错。

use super::*;
use crate::fs::{pipe, DescriptorFlags};
use crate::task::process::DEFAULT_NOFILE;
use crate::task::{self, Credentials, State};
use crate::{cpu, sched};

const PROGRAM: &[u8] = b"/test/selftests/fs/descriptor_growth";

#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-descriptors"), resources("root"), timeout_ms = 90000)]
pub(crate) mod descriptor_growth {
    use super::*;

    /// 真实 C 同时消费高号、组额度、共享表、fork/exec 和动态多路复用，TAP 输出收为注释。
    #[test]
    fn limits_growth_fork_exec_and_multiplex(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[], Credentials::ROOT)
    }

    /// 只有真实用户已阻塞时才关闭并复用编号，旧 OFD 的事件必须完成原等待。
    #[test]
    fn poll_holds_original_ofd_across_close_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        let runtime = checked(
            context,
            UserRuntime::try_new(
                fixture.path.vfs().clone(),
                fixture.path.context().clone(),
                Credentials::ROOT,
                None,
            ),
        )?;
        checked(context, runtime.install_console())?;
        let (reader, writer) = checked(context, pipe::try_new(0, 0, 0))?;
        let old_reader = Arc::downgrade(&reader);
        let (ready_reader, ready_writer) = checked(context, pipe::try_new(0, 0, 0))?;
        checked(context, ready_reader.set_nonblocking(true))?;
        let files = runtime.files.clone();
        let limit = DEFAULT_NOFILE.current as usize;
        let fd =
            checked(context, files.reserve(limit))?.publish(reader, DescriptorFlags::default());
        let ready_fd = checked(context, files.reserve(limit))?
            .publish(ready_writer, DescriptorFlags::default());
        context.expect_eq((fd, ready_fd), (3, 4));

        let arguments = Arguments::from_kernel(
            PROGRAM,
            &[PROGRAM, b"--poll-original"],
            &[],
            exec::DEFAULT_STACK_BYTES,
        )
        .map_err(|_| TestError("descriptor race arguments failed"))?;
        let image = exec::prepare(
            fixture.path.vfs(),
            fixture.path.context(),
            arguments,
            Credentials::ROOT,
        )
        .map_err(|_| TestError("descriptor race image failed"))?;
        let prepared = task::prepare_program(fixture.graph(), None, image, runtime, Placement::Any)
            .map_err(|_| TestError("descriptor race task preparation failed"))?;
        let pid = prepared.pid();
        let user = prepared.user();
        let executor = user
            .executor()
            .ok_or(TestError("descriptor race executor absent"))?;
        let join = prepared
            .publish()
            .map_err(|_| TestError("descriptor race publication failed"))?;
        fixture.running = Some(RunningProgram {
            pid,
            target: None,
            coordinator: cpu::current().id,
            expected_task: Some(join.task_id()),
            join,
        });

        let mut ready = [0u8; 1];
        until("descriptor user did not announce poll", || {
            ready_reader.read(&mut ready) == Ok(1)
        })?;
        context.expect_eq(ready, *b"R");
        until("descriptor poll did not block", || {
            sched::test_state(&executor) == State::Blocked
        })?;

        checked(context, files.close(fd))?;
        context.expect_eq(old_reader.upgrade().is_some(), true);
        let (replacement, replacement_writer) = checked(context, pipe::try_new(0, 0, 0))?;
        let reused = checked(context, files.reserve(limit))?
            .publish(replacement, DescriptorFlags::default());
        context.expect_eq(reused, fd);
        context.expect_eq(checked(context, writer.write(b"X"))?, 1);

        let event = fixture.wait(context)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            fixture.graph().request_root_exit(9);
        }
        fixture.reap_children(context, event.status != 0)?;
        context.expect_eq(old_reader.upgrade().is_none(), true);

        drop((
            files,
            writer,
            ready_reader,
            replacement_writer,
            user,
            executor,
        ));
        Ok(())
    }
}
