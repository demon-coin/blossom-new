//! 用户态内核日志（`/dev/kmsg`）与事件追踪（`/dev/trace`）观测测试模块。
//!
//! # 模块职责与架构定位
//! 本模块作为 `UserOwner` 测试夹具的扩展实现，负责驱动用户态 `/bin/observation-smoke` 程序，
//! 在挂载了 `/dev`（`devfs`）的文件系统环境中验证内核日志环形缓冲区（`klog`）与追踪子系统（`trace`）的
//! 格式化转义、环形覆盖、短缓冲区读取、缺页容错、`poll` 唤醒以及网络环回事件捕获。
//! 上游由用户态测试套件调用，下游通过一对私有匿名管道（`fd 3` 与 `fd 4`）与内核协调线程（`coordinator`）交互，
//! 并在测试结束后调用 `trace::stop()` 停止追踪、等待网络环回端点彻底释放。
//!
//! # 核心设计与关键协议
//! ## 用户态与内核协调线程的双管道握手协议
//! 被测用户进程启动前，文件描述符表中除标准流（`fd 0/1/2`）外预先安装了两条同步管道：
//! - `fd 3`（`command_fd`）：用户态写端 -> 内核协调线程非阻塞读端（`reader`），传递 1 字节命令码。
//! - `fd 4`（`reply_fd`）：内核协调线程写端（`writer`） -> 用户态读端，返回 8 字节小端序的起始日志序列号（`u64`）。
//!
//! ```text
//! 命令字节  适用模式   内核协调线程执行动作                              返回给用户态的 8B 序列号
//! --------  ---------  ------------------------------------------------  ------------------------
//! b'F'      kmsg       写入含控制字符/UTF-8 的 Info 及 Warn/Error 日志    第一条 Info 日志的序列号
//! b'O'      kmsg       连续写入 65 条 Debug 日志以触发环形缓冲区覆盖      第 0 条 Debug 日志的序列号
//! b'D'      kmsg       写入两条 Info 日志供用户态测试短缓冲区读取         第一条日志的序列号
//! b'E'      kmsg       写入两条 Info 日志供用户态测试坏指针 EFAULT 恢复   第一条日志的序列号
//! b'P'      kmsg       等待用户线程进入 poll 阻塞后写入一条 Info 日志唤醒 该唤醒日志的序列号
//! b'T'      trace      等待用户线程在管道读端进入 Blocked 状态后放行      0
//! b'Q'      kmsg/trace 写入最终确认并正常退出协调循环                     0
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块不直接持有自旋锁；协调线程作为普通可睡眠内核任务运行，通过非阻塞读配合 `timer::sleep` 轮询命令管道。
//! - 若用户进程异常退出或协调失败，必须先调用 `request_root_exit(9)` 终止并回收用户进程树，再汇合（`join`）协调线程、
//!   停止全局 `trace` 采集并等待网络协议栈的环回端点计数（`ktest_live_endpoints`）归零，防止残留状态污染后续测试。

use super::*;
use crate::task::UserThread;

/// 用户态内核日志与追踪观测测试程序路径。
const PROGRAM: &[u8] = b"/bin/observation-smoke";

impl UserOwner {
    /// 按指定模式（`b"kmsg"` 或 `b"trace"`）运行用户态观测测试，并在进程树退出后负责汇合协调线程、关闭全局追踪以及等待网络环回端点彻底释放。
    pub(super) fn observation_mode(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
    ) -> TestResult {
        // Step 1: 准备 TTY 与 `/dev` 挂载环境，校验初始网络端点计数为 0 且 trace 模式具备网络栈。
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

        // Step 2: 运行观测主流程；若中途出错则强制终止并回收用户进程树。
        self.observation_trace = mode == b"trace";
        let result = self.observation_run(context, mode);
        if result.is_err() {
            self.graph().request_root_exit(9);
            if self.running.is_some() {
                self.wait(context)?;
            }
            self.reap_children(context, true)?;
        }

        // Step 3: 汇合后台协调线程，停止 trace 采集，并等待测试期间创建的网络环回端点全部退役归零。
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

    /// 准备用户运行时、控制台标准流与 `fd 3`/`fd 4` 协调管道，启动用户态 `observation-smoke` 进程与内核态协调线程并等待执行完成。
    fn observation_run(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        // Step 1: 创建用户运行时并安装 `/dev/console`（fd 0/1/2），随后分配两条管道分别挂载到 fd 3（命令写端）与 fd 4（应答读端）。
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

        // Step 2: 加载观测测试 ELF 映像，准备被测用户进程与后台协调内核线程并一同发布调度。
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

        // Step 3: 等待用户根进程退出并回收全部子进程，校验退出状态码为 0。
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

/// 从非阻塞命令管道读取 1 字节的用户态请求命令；若管道暂时无数据则每次休眠 1 毫秒重试，若被测用户进程已退出或超过 10 秒期限则返回错误。
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

/// 轮询等待被测用户线程进入调度器的 `State::Blocked` 阻塞状态，确保后续写入管道或写入 `klog` 触发的是真实的阻塞唤醒路径。
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

/// 内核协调线程主循环：接收用户态通过 `fd 3` 发送的单字节指令，按模式生成指定格式的 `klog` 日志或确认阻塞状态，并将 8 字节序列号写回 `fd 4`。
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
