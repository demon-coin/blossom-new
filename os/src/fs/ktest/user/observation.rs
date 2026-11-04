//! 普通观测 ELF 复用原身份图与私有管道，日志由合法内核任务提交。

use super::*;
use crate::task::UserThread;

const PROGRAM: &[u8] = b"/bin/observation-smoke";

impl UserOwner {
    /// 两个模式沿真实 devfs、Runtime 与文件表运行，用户和协议退休后才返回原资源检查。
    pub(super) fn observation_mode(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
    ) -> TestResult {
        self.prepare_tty(context, 4)?;
        if let Some(net) = &self.net {
            if net.ktest_live_endpoints() != 0 {
                return Err(TestError(
                    "observation case inherited live network endpoints",
                ));
            }
        }
        if mode == b"trace" && self.net.is_none() {
            return Err(TestError("observation trace suite has no network"));
        }
        self.observation_trace = mode == b"trace";
        let result = self.observation_run(context, mode);
        if result.is_err() {
            self.graph().request_root_exit(9);
            if self.running.is_some() {
                self.wait(context)?;
            }
            self.reap_children(context, true)?;
        }
        let observer_status = self.observer.take().map(|observer| observer.join());
        if self.observation_trace {
            checked(context, crate::trace::stop())?;
            self.observation_trace = false;
        }
        if let Some(net) = &self.net {
            let end = time::now()
                .checked_add(time::Duration::from_secs(30))
                .ok_or(TestError(
                    "observation protocol retirement deadline overflow",
                ))?;
            while net.ktest_live_endpoints() != 0 {
                if end <= time::now() {
                    return Err(TestError("observation loopback protocols did not retire"));
                }
                timer::sleep(time::Duration::from_millis(1));
            }
        }
        if let Some(status) = observer_status {
            context.expect_eq(status, 0);
            if status != 0 {
                return Err(TestError("observation coordinator failed"));
            }
        }
        result
    }

    /// 先准备两个普通 pipe 和所有 Task；fd 3/4 只承担用户命令与内核确认。
    fn observation_run(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        let runtime = checked(
            context,
            UserRuntime::try_new(
                self.path.vfs().clone(),
                self.path.context().clone(),
                Credentials::ROOT,
                self.net.clone(),
            ),
        )?;
        checked(context, runtime.install_console())?;
        let (reader, user_writer) = checked(context, crate::fs::pipe::try_new(0, 0, 0))?;
        let (user_reader, writer) = checked(context, crate::fs::pipe::try_new(0, 0, 0))?;
        checked(context, reader.set_status_flags(crate::fs::O_NONBLOCK))?;
        let command_fd = checked(
            context,
            runtime
                .files
                .reserve(crate::task::process::DEFAULT_NOFILE.current as usize),
        )?
        .publish(user_writer, DescriptorFlags::default());
        let reply_fd = checked(
            context,
            runtime
                .files
                .reserve(crate::task::process::DEFAULT_NOFILE.current as usize),
        )?
        .publish(user_reader, DescriptorFlags::default());
        if command_fd != 3 || reply_fd != 4 {
            return Err(TestError("observation private pipe descriptors changed"));
        }
        let image = self.load_named(context, PROGRAM, mode)?;
        let prepared = task::prepare_program(self.graph(), None, image, runtime, Placement::Any)
            .map_err(|_| TestError("observation user task preparation failed"))?;
        let root = prepared.user();
        let graph = self.graph().clone();
        let trace_mode = mode == b"trace";
        let coordinator = task::prepare_kernel(move || {
            if let Err(error) = coordinate(&reader, &writer, &root, trace_mode) {
                crate::ktest::report::diagnostic(format_args!(
                    "observation coordinator-error={:?}",
                    error
                ));
                graph.request_root_exit(9);
                return 1;
            }
            0
        })
        .map_err(|_| TestError("observation coordinator preparation failed"))?;
        let pid = prepared.pid();
        let join = prepared
            .publish()
            .map_err(|_| TestError("observation user publication failed"))?;
        self.running = Some(RunningProgram {
            pid,
            target: None,
            coordinator: cpu::current().id,
            expected_task: Some(join.task_id()),
            join,
        });
        self.observer = Some(coordinator.publish());
        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, event.status != 0)?;
        context.diagnostic(format_args!(
            "observation-user mode={:?} status={}",
            mode, event.status
        ));
        if event.status != 0 {
            return Err(TestError("ordinary observation program failed"));
        }
        Ok(())
    }
}

/// 普通非阻塞 pipe 消费一条命令，用户退出和期限到达保留为协调失败。
fn command(reader: &FileRef, root: &UserThread) -> Result<u8, TestError> {
    let end = time::now()
        .checked_add(time::Duration::from_secs(10))
        .ok_or(TestError("observation command deadline overflow"))?;
    let mut byte = [0u8; 1];
    loop {
        match reader.read(&mut byte) {
            Ok(1) => return Ok(byte[0]),
            Ok(_) => return Err(TestError("observation command pipe ended")),
            Err(FsError::WouldBlock) => {
                if root.mm().is_none() || end <= time::now() {
                    return Err(TestError("observation command did not arrive"));
                }
                timer::sleep(time::Duration::from_millis(1));
            }
            Err(_) => return Err(TestError("observation command pipe read failed")),
        }
    }
}

/// 唤醒前确认原用户执行者真实 Blocked，保证 pipe 等待和 kmsg poll 都消费延期通知。
fn blocked(root: &UserThread) -> TestResult {
    let executor = root
        .executor()
        .ok_or(TestError("observation user executor disappeared"))?;
    until("observation user did not block", || {
        sched::test_state(&executor) == State::Blocked || root.mm().is_none()
    })?;
    if root.mm().is_none() {
        return Err(TestError("observation user exited before blocking"));
    }
    Ok(())
}

/// 固定短日志由合法可睡眠 Task 生产；trace 模式只放行实际 pipe 等待，不人工写事件。
fn coordinate(
    reader: &FileRef,
    writer: &FileRef,
    root: &UserThread,
    trace_mode: bool,
) -> TestResult {
    loop {
        let operation = command(reader, root)?;
        let sequence = match operation {
            b'Q' => 0,
            b'T' if trace_mode => {
                blocked(root)?;
                0
            }
            b'F' if !trace_mode => {
                let first = crate::klog!(
                    crate::klog::Level::Info,
                    "observation format\t\n\r\\\u{7f}é"
                );
                crate::klog!(crate::klog::Level::Warn, "observation shared second");
                crate::klog!(crate::klog::Level::Error, "observation shared third");
                first
            }
            b'O' if !trace_mode => {
                let first = crate::klog!(crate::klog::Level::Debug, "observation cover 0");
                for index in 1..65 {
                    crate::klog!(crate::klog::Level::Debug, "observation cover {}", index);
                }
                first
            }
            b'D' if !trace_mode => {
                let first = crate::klog!(crate::klog::Level::Info, "observation small consumed");
                crate::klog!(crate::klog::Level::Info, "observation after small");
                first
            }
            b'E' if !trace_mode => {
                let first = crate::klog!(crate::klog::Level::Info, "observation fault consumed");
                crate::klog!(crate::klog::Level::Info, "observation after fault");
                first
            }
            b'P' if !trace_mode => {
                blocked(root)?;
                crate::klog!(crate::klog::Level::Info, "observation poll wake")
            }
            _ => return Err(TestError("observation command does not belong to mode")),
        };
        if writer.write(&sequence.to_le_bytes()) != Ok(8) {
            return Err(TestError("observation reply pipe write failed"));
        }
        if operation == b'Q' {
            return Ok(());
        }
    }
}
