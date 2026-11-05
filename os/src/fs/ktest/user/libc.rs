//! 静态 libc、真实脚本解释和确定迁核复用同一 UserOwner 生命周期。

use super::*;
use crate::sync::SpinLock;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::fs::ktest::user::network::network_until;
use crate::fs::{DescriptorFlags, FileRef};
use crate::task::exec::{self, Arguments, LoadedImage};
use crate::task::{self, Credentials, Placement, State, UserRuntime};
use crate::{cpu, sched};

const LIBC: &[u8] = b"/bin/libc-smoke";
const BUSYBOX: &[u8] = b"/bin/busybox";
const BASE_ABI: &[u8] = b"/test/scripts/base-abi";
const SCRIPT_ENV: &[u8] = b"BLOSSOM_SCRIPT=stage7";

impl UserOwner {
    /// 真实 C 核 uname 复制边界及 fork 后独立 root/cwd；凭据错误也由用户程序检查。
    pub(super) fn runtime_view(
        &mut self,
        context: &mut TestContext<'_>,
        credentials: Credentials,
    ) -> TestResult {
        self.prepare_tty(context, 4)?;
        self.run_console_named(context, LIBC, &[b"--runtime-view"], credentials)
    }

    /// 真实 C 消费组限额、fork/线程共享、exec 栈边界与用户复制错误，再回收全部执行尾部。
    pub(super) fn runtime_limits(
        &mut self,
        context: &mut TestContext<'_>,
        credentials: Credentials,
    ) -> TestResult {
        self.prepare_tty(context, 4)?;
        self.run_console_named(context, LIBC, &[b"--runtime-limits"], credentials)
    }

    /// 可写真实卷上的 C 直接消费 canonical 时间修改与错误权限，退出仍由同一图回收。
    pub(super) fn runtime_file_times(
        &mut self,
        context: &mut TestContext<'_>,
        credentials: Credentials,
    ) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadWrite, 4)?;
        self.mount_console_devfs(context)?;
        self.run_console_named(context, LIBC, &[b"--runtime-file-times"], credentials)
    }

    /// 普通 C 的用户拥有结束后继续等待真实协议退休，再交原夹具比较资源。
    pub(super) fn runtime_socket_timeouts(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.prepare_tty(context, 4)?;
        let result = self.run_console_named(
            context,
            LIBC,
            &[b"--runtime-socket-timeouts"],
            Credentials::ROOT,
        );

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

    /// 可写卷上的 C 核 UTC、单调与绝对睡眠，并用真实新文件消费共同日期来源。
    pub(super) fn runtime_clock(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.prepare_mode(context, AccessMode::ReadWrite, 4)?;
        self.mount_console_devfs(context)?;
        self.run_console_named(context, LIBC, &[b"--runtime-clock"], Credentials::ROOT)
    }

    /// 在完整标准流和真实执行映像上运行静态 libc smoke，并消费原 wait/reap 资源。
    pub(super) fn libc_static_smoke(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.prepare_tty(context, 4)?;
        let image = self.load_named(context, LIBC, b"stage7")?;
        let target = (cpu::current().id + 1) % cpu::count();
        self.start_console_program(
            context,
            image,
            Placement::Cpu(target),
            Credentials::ROOT,
            None,
        )?;

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

    /// 让同一 Any Task 在两个真实 pipe 阻塞窗口指定 CPU0、CPU1，再消费完整用户树。
    pub(super) fn libc_migration(&mut self, context: &mut TestContext<'_>) -> TestResult {
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

    /// 逐个运行普通脚本和 BusyBox 基础 ABI，并让 C 自身核对脚本失败后的旧映像。
    pub(super) fn script_exec(&mut self, context: &mut TestContext<'_>) -> TestResult {
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

        self.run_console_named(
            context,
            LIBC,
            &[b"--script-errors", b"stage7"],
            Credentials::ROOT,
        )?;

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

    /// 安装 console 标准流，可在发表前把 stdout/stderr 替换为同一输出 OFD，stdin 保持 console。
    fn start_console_program(
        &mut self,
        context: &TestContext<'_>,
        image: LoadedImage,
        placement: Placement,
        credentials: Credentials,
        output: Option<FileRef>,
    ) -> Result<(), TestError> {
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

    /// 以实际 argv/env 准备一个明确 console 的普通程序并消费退出树。
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

    /// 把真实 selftest 输出收集为 KTAP 注释；用户树退出、离栈和回收后才消费收集任务。
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
        // 收集任务只持读端；未发表时可正常 Drop，发表后由 observer 承担唯一 join 责任。
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

    /// 共用普通 console 与 selftest 的 argv/env 和 ELF 准备，不改变程序凭据及解释器语义。
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

/// 把真实文件表预留发布为一个普通继承描述符，不依赖数值槽特判。
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

/// 为迁核 argv 生成栈内有界十进制描述符字符串，不引入准备期堆分配。
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
