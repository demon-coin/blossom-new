//! 用户态静态 libc 冒烟测试、脚本解释执行与跨 CPU 迁移测试辅助实现。
//!
//! # 模块职责与架构定位
//! 本模块作为 `UserOwner` 测试夹具的扩展实现，负责驱动用户态 C 测试程序（`/bin/libc-smoke`）、
//! BusyBox Shell（`/bin/busybox`）以及 `#!` 脚本的加载、标准流绑定、执行与回收。
//! 上游由 `runtime`、`resources`、`stack` 等测试套件调用，下游依赖 `task::exec` 加载映像、
//! `UserRuntime` 管理进程文件表与控制台设备，并通过 `TaskGraph` 完成进程树等待与内核栈退出回收。
//!
//! # 核心设计与关键状态机
//! ## 跨核迁移双管道握手协议（`libc_migration`）
//! 测试使用两条匿名管道（`ready` 与 `go`）在用户态被测线程与内核态协调线程（`coordinator`）之间同步：
//!
//! ```text
//! 用户态线程 (/bin/libc-smoke --migrate)         内核协调线程 (coordinator)
//! -------------------------------------         -------------------------
//! 1. 在初始 CPU 运行并记录足迹
//! 2. write(ready_write, "R")  ----------------> 3. read(ready_reader) 收到 'R'
//! 3. read(go_read) 进入阻塞 (State::Blocked)    4. 轮询确认用户线程已处于 State::Blocked
//!                                               5. 读取并清空用户态 CPU 运行掩码
//!                                               6. 调用 test_request_migration 迁移至目标核
//! 8. 唤醒并在新 CPU 上继续执行 <---------------- 7. write(go_writer, "M") 唤醒用户线程
//! ```
//!
//! ## 自测输出收集管道（`run_selftest_named`）
//! - 标准输入（`fd 0`）保持指向 `/dev/console`。
//! - 标准输出（`fd 1`）与标准错误（`fd 2`）在进程发布前通过 `dup_exact` 重定向至同一匿名管道写端。
//! - 后台收集线程（`collector`）独占管道读端，将用户态输出逐块转换为 KTAP 诊断注释，直至所有写端随子进程树退出关闭。
//!
//! # 锁获取顺序与并发契约
//! - `initial_mask` 与 `first_mask` 自旋锁（`SpinLock`）仅在协调线程记录掩码与主测试线程最终输出诊断时短暂获取，
//!   持锁期间不调用任何可能睡眠或触发调度的函数，也不与其他锁嵌套。
//! - 后台观察线程（`self.observer`）必须在被测用户进程树完成 `wait` 与 `reap_children` 之后再执行 `join`，
//!   确保管道写端已全部关闭，避免收集线程阻塞导致死锁。

use super::*;
use crate::sync::SpinLock;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::fs::ktest::user::network::network_until;
use crate::fs::{DescriptorFlags, FileRef};
use crate::task::exec::{self, Arguments, LoadedImage};
use crate::task::{self, Credentials, Placement, State, UserRuntime};
use crate::{cpu, sched};

/// 用户态静态 libc 综合冒烟测试程序路径。
const LIBC: &[u8] = b"/bin/libc-smoke";
/// 用户态 BusyBox 可执行文件路径，用于提供 `/bin/sh` 解释器。
const BUSYBOX: &[u8] = b"/bin/busybox";
/// BusyBox 基础 ABI 验证脚本路径。
const BASE_ABI: &[u8] = b"/test/scripts/base-abi";
/// 脚本执行测试注入的固定环境变量字节串。
const SCRIPT_ENV: &[u8] = b"BLOSSOM_SCRIPT=stage7";

impl UserOwner {
    /// 测试用户态 `uname` 缓冲区复制边界、进程组与会话查询、凭据变更权限检查，以及 `fork` 后子进程独立的根目录（`root`）与当前工作目录（`cwd`）视图。
    pub(super) fn runtime_view(
        &mut self,
        context: &mut TestContext<'_>,
        credentials: Credentials,
    ) -> TestResult {
        self.prepare_tty(context, 4)?;
        self.run_console_named(context, LIBC, &[b"--runtime-view"], credentials)
    }

    /// 测试进程组的资源限额（`rlimit`）继承与修改、`fork`/线程间的限额共享语义、`exec` 栈空间边界，以及用户态非法指针复制的错误返回。
    pub(super) fn runtime_limits(
        &mut self,
        context: &mut TestContext<'_>,
        credentials: Credentials,
    ) -> TestResult {
        self.prepare_tty(context, 4)?;
        self.run_console_named(context, LIBC, &[b"--runtime-limits"], credentials)
    }

    /// 在可写文件系统与 `/dev` 挂载环境下，测试用户态通过 `utimensat` 修改文件时间戳（含 `UTIME_NOW`、`UTIME_OMIT` 语义）及非所有者权限拒绝行为。
    pub(super) fn runtime_file_times(
        &mut self,
        context: &mut TestContext<'_>,
        credentials: Credentials,
    ) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadWrite, 4)?;
        self.mount_console_devfs(context)?;
        self.run_console_named(context, LIBC, &[b"--runtime-file-times"], credentials)
    }

    /// 运行用户态套接字超时（`SO_RCVTIMEO` / `SO_SNDTIMEO`）测试，并在用户进程退出后等待底层网络协议栈的所有活动端点彻底释放。
    pub(super) fn runtime_socket_timeouts(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.prepare_tty(context, 4)?;
        let result = self.run_console_named(
            context,
            LIBC,
            &[b"--runtime-socket-timeouts"],
            Credentials::ROOT,
        );

        // 用户进程退出后，TCP TIME_WAIT 或异步关闭流程可能尚未结束；等待协议栈端点计数归零后再返回，防止后续资源基线检查误报泄漏。
        let net = self
            .net
            .as_ref()
            .ok_or(TestError("socket timeout suite has no network"))?;
        network_until(
            || net.ktest_live_endpoints() == 0,
            "runtime socket protocols did not retire",
            30,
        )?;
        result
    }

    /// 在可写文件系统环境下，测试用户态实时时钟（`CLOCK_REALTIME`）、单调时钟（`CLOCK_MONOTONIC`）读取与绝对时间睡眠（`clock_nanosleep`），并验证新建文件的时间戳与系统时钟一致。
    pub(super) fn runtime_clock(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadWrite, 4)?;
        self.mount_console_devfs(context)?;
        self.run_console_named(context, LIBC, &[b"--runtime-clock"], Credentials::ROOT)
    }

    /// 将静态链接的 `libc-smoke` 绑定到非当前 CPU 核心启动，验证标准流（`fd 0/1/2`）挂载、跨核初始放置与进程正常退出回收。
    pub(super) fn libc_static_smoke(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.prepare_tty(context, 4)?;
        // Step 1: 准备只读根文件系统与 `/dev/console`，加载 ELF 映像并选择下一个 CPU 核心作为初始放置目标。
        let image = self.load_named(context, LIBC, b"stage7")?;
        let target = (cpu::current().id + 1) % cpu::count();
        self.start_console_program(
            context,
            image,
            Placement::Cpu(target),
            Credentials::ROOT,
            None,
        )?;

        // Step 2: 等待根进程退出，若异常退出则广播信号终止残留子进程，并等待所有任务彻底退出内核栈。
        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, event.status != 0)?;
        context.diagnostic(format_args!(
            "libc-static status={} target_cpu={} console_fds=0,1,2",
            event.status, target
        ));
        Ok(())
    }

    /// 测试自由放置（`Placement::Any`）的用户线程在两次管道读阻塞窗口期间，由内核协调线程依次请求迁移至 CPU0 和 CPU1，验证线程在目标核唤醒恢复执行并记录正确的 CPU 运行掩码。
    pub(super) fn libc_migration(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 准备两组同步管道（ready 用于用户态通知就绪，go 用于内核态放行），并安装到新建的用户运行时文件表中。
        self.prepare_tty(context, 4)?;
        let (ready_reader, ready_writer) = checked(context, crate::fs::pipe::try_new(0, 0, 0))?;
        let (go_reader, go_writer) = checked(context, crate::fs::pipe::try_new(0, 0, 0))?;
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
        let ready_write = publish_descriptor(context, &runtime, ready_writer)?;
        let go_read = publish_descriptor(context, &runtime, go_reader)?;

        // Step 2: 将管道文件描述符格式化为十进制字符串参数，构造用户进程执行映像与初始任务对象。
        let (ready_text, ready_start) = decimal_fd(ready_write);
        let (go_text, go_start) = decimal_fd(go_read);
        let arguments = Arguments::from_kernel(
            LIBC,
            &[
                LIBC,
                b"stage7",
                b"--migrate",
                &ready_text[ready_start..],
                &go_text[go_start..],
            ],
            &[],
            exec::DEFAULT_STACK_BYTES,
        )
        .map_err(|_| TestError("libc migration arguments failed"))?;
        let image = exec::prepare(
            self.path.vfs(),
            self.path.context(),
            arguments,
            Credentials::ROOT,
        )
        .map_err(|error| {
            context.diagnostic(format_args!("libc-image-prepare error={:?}", error));
            TestError("libc image preparation failed")
        })?;
        let prepared = task::prepare_program(self.graph(), None, image, runtime, Placement::Any)
            .map_err(|_| TestError("libc migration task preparation failed"))?;
        let pid = prepared.pid();
        let user = prepared.user();
        let task = user
            .executor()
            .ok_or(TestError("libc migration executor disappeared"))?;
        let initial_mask = Arc::try_new(SpinLock::new(cpu::CpuSet::empty()))
            .map_err(|_| TestError("migration initial mask allocation failed"))?;
        let first_mask = Arc::try_new(SpinLock::new(cpu::CpuSet::empty()))
            .map_err(|_| TestError("migration first mask allocation failed"))?;

        // Step 3: 创建内核态协调线程，在两轮循环中分别等待用户线程写入 'R' 并进入阻塞状态，采样其运行过的 CPU 掩码后发起向 CPU0 / CPU1 的迁移请求。
        let coordination_task = task.clone();
        let coordination_user = user.clone();
        let initial_observed = initial_mask.clone();
        let first_observed = first_mask.clone();
        let coordinator = task::prepare_kernel(move || {
            for target in 0..2 {
                let mut byte = [0u8; 1];
                if ready_reader.read(&mut byte) != Ok(1) || byte[0] != b'R' {
                    return 1;
                }
                while sched::test_state(&coordination_task) != State::Blocked {
                    if coordination_user.mm().is_none() {
                        return 1;
                    }
                    sched::yield_now();
                }
                if !coordination_user
                    .executor()
                    .is_some_and(|executor| Arc::ptr_eq(&executor, &coordination_task))
                {
                    return 1;
                }
                let mask = coordination_user.ktest_take_user_cpu_mask();
                if target == 0 {
                    *initial_observed.lock() = mask;
                } else {
                    *first_observed.lock() = mask;
                }
                if mask == cpu::CpuSet::empty() || (target == 1 && !mask.contains(0)) {
                    return 1;
                }

                sched::test_request_migration(&coordination_task, target);
                if go_writer.write(b"M") != Ok(1) {
                    return 1;
                }
            }
            0
        })
        .map_err(|_| TestError("libc migration coordinator preparation failed"))?;

        // Step 4: 发布被测用户任务与内核协调任务，等待用户进程树完成并核对最终在 CPU1 上运行的掩码记录。
        let join = prepared
            .publish()
            .map_err(|_| TestError("libc migration publication failed"))?;
        self.running = Some(RunningProgram {
            pid,
            target: None,
            coordinator: cpu::current().id,
            expected_task: Some(join.task_id()),
            join,
        });
        self.observer = Some(coordinator.publish());

        let event = self.wait(context)?;
        let final_mask = user.ktest_take_user_cpu_mask();
        context.expect_eq(final_mask.contains(1), true);
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, event.status != 0)?;
        context.expect_eq(self.observer.take().unwrap().join(), 0);
        context.diagnostic(format_args!(
            "libc-migration task={} status={} blocked_windows=2 initial_mask={:?} cpu0_mask={:?} final_mask={:?}",
            task.id().as_usize(),
            event.status,
            *initial_mask.lock(),
            *first_mask.lock(),
            final_mask
        ));
        Ok(())
    }

    /// 测试 `#!` 脚本解释执行（无选项解释器、带选项解释器、多级链式脚本解释器）、脚本加载错误码（`ELOOP`、`ENOENT`、`ENOEXEC`）保持原进程映像不损坏，以及 BusyBox Shell 基础 ABI 与非特权用户执行行为。
    pub(super) fn script_exec(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 依次直接执行无参数解释器脚本、带参解释器脚本与链式解释器脚本，验证内核 `#!` 解析与参数拼接。
        self.prepare_tty(context, 8)?;
        for script in [
            b"/test/scripts/no-option".as_slice(),
            b"/test/scripts/option".as_slice(),
            b"/test/scripts/chain1".as_slice(),
        ] {
            let arguments = Arguments::from_kernel(
                script,
                &[script, b"tail"],
                &[SCRIPT_ENV],
                exec::DEFAULT_STACK_BYTES,
            )
            .map_err(|_| TestError("script arguments failed"))?;
            let image = exec::prepare(
                self.path.vfs(),
                self.path.context(),
                arguments,
                Credentials::ROOT,
            )
            .map_err(|error| {
                context.diagnostic(format_args!("script-image-prepare error={:?}", error));
                TestError("script image preparation failed")
            })?;
            self.start_console_program(context, image, Placement::Any, Credentials::ROOT, None)?;
            let event = self.wait(context)?;
            context.expect_eq(event.status, 0);
            if event.status != 0 {
                self.graph().request_root_exit(9);
            }
            self.reap_children(context, event.status != 0)?;
        }

        // Step 2: 运行 C 测试程序验证循环脚本（ELOOP）、缺失解释器（ENOENT）和格式错误（ENOEXEC）下调用进程的旧地址空间保持完整。
        self.run_console_named(
            context,
            LIBC,
            &[b"--script-errors", b"stage7"],
            Credentials::ROOT,
        )?;

        // Step 3: 使用 BusyBox sh 执行基础 ABI 脚本，并以普通 UID/GID（1000/2000）身份验证非特权执行语义。
        self.run_console_named(context, BUSYBOX, &[b"sh", BASE_ABI], Credentials::ROOT)?;
        self.run_console_named(
            context,
            LIBC,
            &[b"--abi-user", b"stage7"],
            Credentials {
                uid: 1000,
                gid: 2000,
            },
        )?;
        context.diagnostic(format_args!(
            "script-exec scripts=no-option,option,chain1,no-exec errors=eloop,enoent,enoexec mode0644=executed base-abi=busybox"
        ));
        Ok(())
    }

    /// 创建用户态运行时并安装控制台标准流（`fd 0/1/2`）；若提供了自定义输出文件对象，则在发布进程前将 `stdout`（`fd 1`）与 `stderr`（`fd 2`）重定向到该文件对象。
    fn start_console_program(
        &mut self,
        context: &TestContext<'_>,
        image: LoadedImage,
        placement: Placement,
        credentials: Credentials,
        output: Option<FileRef>,
    ) -> Result<(), TestError> {
        // Step 1: 初始化用户运行时并打开 `/dev/console` 作为 fd 0、1、2。
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
        if let Some(output) = output {
            let fd = publish_descriptor(context, &runtime, output)?;
            let limit = crate::task::process::DEFAULT_NOFILE.current as usize;
            checked(context, runtime.files.dup_exact(fd, 1, limit))?;
            checked(context, runtime.files.dup_exact(fd, 2, limit))?;
            checked(context, runtime.files.close(fd))?;
        }

        // Step 2: 根据放置策略准备用户任务并发布到调度器，在夹具中记录运行句柄以供后续 wait 与离栈核对。
        let target = match &placement {
            Placement::Cpu(target) => Some(*target),
            Placement::Any => None,
        };
        let prepared = task::prepare_program(self.graph(), None, image, runtime, placement)
            .map_err(|_| TestError("console user task preparation failed"))?;
        let pid = prepared.pid();
        let expected_task = prepared.user().executor();
        let join = prepared
            .publish()
            .map_err(|_| TestError("console user publication failed"))?;
        self.running = Some(RunningProgram {
            pid,
            target,
            coordinator: cpu::current().id,
            expected_task: expected_task.map(|task| task.id()),
            join,
        });
        Ok(())
    }

    /// 使用指定的程序路径、附加参数列表与凭据启动绑定到控制台的用户进程，等待其退出并回收所有子进程。
    pub(super) fn run_console_named(
        &mut self,
        context: &mut TestContext<'_>,
        program: &[u8],
        tail: &[&[u8]],
        credentials: Credentials,
    ) -> TestResult {
        let image = self.load_console_named(context, program, tail, credentials)?;
        self.start_console_program(context, image, Placement::Any, credentials, None)?;
        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, event.status != 0)
    }

    /// 启动用户态自测程序并通过匿名管道捕获其标准输出与标准错误，由后台内核线程实时转化为 KTAP 诊断注释，待用户进程树完全退出回收后再汇合收集线程。
    pub(super) fn run_selftest_named(
        &mut self,
        context: &mut TestContext<'_>,
        program: &[u8],
        tail: &[&[u8]],
        credentials: Credentials,
    ) -> TestResult {
        assert!(
            self.observer.is_none(),
            "selftest output observer already owned"
        );
        let image = self.load_console_named(context, program, tail, credentials)?;
        let (reader, writer) = checked(
            context,
            crate::fs::pipe::try_new(0, credentials.uid, credentials.gid),
        )?;
        // Step 1: 准备后台输出收集线程；该线程仅持有管道读端，在所有写端随子进程关闭后自动读到 EOF（Ok(0)）并退出。
        let collector = task::prepare_kernel(move || {
            let mut bytes = [0u8; 128];
            loop {
                match reader.read(&mut bytes) {
                    Ok(0) => return 0,
                    Ok(count) => crate::ktest::report::write(format_args!(
                        "  # selftest output: {}\n",
                        bytes[..count].escape_ascii()
                    )),
                    Err(_) => return 1,
                }
            }
        })
        .map_err(|_| TestError("selftest output collector preparation failed"))?;

        // Step 2: 将管道写端作为 stdout/stderr 注入被测进程并发布收集线程，等待进程树彻底退出后再 join 收集线程。
        self.start_console_program(context, image, Placement::Any, credentials, Some(writer))?;
        self.observer = Some(collector.publish());

        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, event.status != 0)?;
        context.expect_eq(self.observer.take().unwrap().join(), 0);
        Ok(())
    }

    /// 组装命令行参数向量（`argv`）与必要的环境变量（`envp`），并调用 `exec::prepare` 解析加载目标 ELF 或脚本映像。
    fn load_console_named(
        &self,
        context: &TestContext<'_>,
        program: &[u8],
        tail: &[&[u8]],
        credentials: Credentials,
    ) -> Result<LoadedImage, TestError> {
        let mut argv = Vec::new();
        argv.try_reserve_exact(tail.len() + 1)
            .map_err(|_| TestError("console argv allocation failed"))?;
        argv.push(program);
        argv.extend_from_slice(tail);
        let env = if program == BUSYBOX {
            &[SCRIPT_ENV][..]
        } else {
            &[][..]
        };
        let arguments = Arguments::from_kernel(program, &argv, env, exec::DEFAULT_STACK_BYTES)
            .map_err(|_| TestError("console named arguments failed"))?;
        exec::prepare(self.path.vfs(), self.path.context(), arguments, credentials).map_err(
            |error| {
                context.diagnostic(format_args!("console-image-prepare error={:?}", error));
                TestError("console image preparation failed")
            },
        )
    }
}

/// 在用户运行时的文件描述符表中分配一个空闲槽位，并将给定的文件对象（`FileRef`）发布为普通可继承的文件描述符。
fn publish_descriptor(
    context: &TestContext<'_>,
    runtime: &UserRuntime,
    file: FileRef,
) -> Result<usize, TestError> {
    Ok(checked(
        context,
        runtime
            .files
            .reserve(crate::task::process::DEFAULT_NOFILE.current as usize),
    )?
    .publish(file, DescriptorFlags::default()))
}

/// 将非负整数文件描述符转换为栈上固定长度缓冲区的十进制 ASCII 字节切片（返回缓冲区与有效数字起始下标），避免在准备阶段分配堆内存。
fn decimal_fd(fd: usize) -> ([u8; 20], usize) {
    let mut digits = [0u8; 20];
    let mut cursor = digits.len();
    let mut value = fd;
    loop {
        cursor -= 1;
        digits[cursor] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    (digits, cursor)
}
