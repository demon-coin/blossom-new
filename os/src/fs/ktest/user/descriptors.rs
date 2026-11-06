//! 用户态文件描述符表动态扩容与 `poll` 并发关闭复用集成测试模块。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统与用户态集成测试层（`fs::ktest::user`），负责验证进程文件描述符表（FD Table）
//! 的容量增长、`RLIMIT_NOFILE` 限制、`fork`/`exec` 传递语义，以及当某个描述符正在被 `poll` 等待时
//! 被并发关闭并复用同一 FD 编号的打开文件描述（Open File Description, OFD）生命周期契约。
//! 上游由 `fs::ktest::user::anchor` 注册到 `ktest` 框架，下游依赖 `UserOwner`、`pipe` 管道与 `FileTable`。
//!
//! # `poll` 期间关闭与复用描述符编号的并发时序
//! ```text
//! [用户态进程 (--poll-original)]            [内核测试线程]
//!               |                                 |
//!   1. 向 ready_fd (fd=4) 写入 'R' -------------> | 收到 'R'，等待用户线程进入 State::Blocked
//!   2. poll(fd=3) 持原 OFD 强引用并阻塞 --------> |
//!               |                                 +-- files.close(3)：从 FD 表移除槽位 3
//!               |                                 |   （此时原 OFD 仍由 poll 栈帧持有，未销毁）
//!               |                                 +-- files.reserve() + publish(replacement)
//!               |                                 |   （槽位 3 被新管道复用）
//!               | <------------------------------ +-- writer.write("X")：向原管道写入数据
//!   3. 原 OFD 就绪唤醒 poll，返回 0 退出 -------> | wait() 回收进程，验证原 OFD 随进程退出释放
//! ```
//!
//! # 锁与并发契约
//! - `FileTable` 内部使用自旋锁保护描述符槽位数组；`poll` 系统调用在进入阻塞前已克隆目标槽位的 `FileRef`（`Arc<dyn File>`）并释放表锁；
//! - 并发调用 `files.close(fd)` 仅移除表内映射，不会提前销毁正在被 `poll` 监听的底层 `File` 实例，从而避免悬垂引用或漏唤醒。

use super::*;
use crate::fs::{pipe, DescriptorFlags};
use crate::task::process::DEFAULT_NOFILE;
use crate::task::{self, Credentials, State};
use crate::{cpu, sched};

/// 用户态文件描述符扩容与多路复用自测程序路径。
const PROGRAM: &[u8] = b"/test/selftests/fs/descriptor_growth";

/// 用户态文件描述符表增长与并发复用测试套件。
#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-descriptors"), resources("root"), timeout_ms = 90000)]
pub(crate) mod descriptor_growth {
    use super::*;

    /// 验证高编号描述符分配、`RLIMIT_NOFILE` 限制、共享文件表、`fork`/`exec` 继承与动态多路复用。
    ///
    /// - **测试目标**：确认文件描述符表在跨段扩容、资源上限截断、多进程/多线程共享及多路复用场景下的正确性。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：挂载只读 ext4 根文件系统与 `/dev` 控制台；
    ///   2. When：以 `ROOT` 凭据运行 `/test/selftests/fs/descriptor_growth` 用户态测试程序；
    ///   3. Then：所有子测试项通过并以状态码 0 退出。
    #[test]
    fn limits_growth_fork_exec_and_multiplex(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[], Credentials::ROOT)
    }

    /// 验证用户线程阻塞在 `poll` 期间，目标文件描述符被并发关闭并立即复用为新管道时，原 `poll` 仍持有旧打开文件描述（OFD）并被旧管道事件正确唤醒。
    ///
    /// - **测试目标**：确认 `poll` 持有的是打开文件描述（`FileRef`）而非文件描述符编号（`fd`），关闭并复用同一 `fd` 不会破坏正在进行的等待。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：创建 `UserRuntime` 并预装控制台（占用 `fd` 0..=2），随后在 `fd=3` 安装被监听管道读端 `reader`（保留弱引用 `old_reader`），在 `fd=4` 安装就绪通知管道写端 `ready_writer`；
    ///   2. When：启动 `/test/selftests/fs/descriptor_growth --poll-original`，等待其向 `fd=4` 写入 `'R'` 且内核执行线程进入 `State::Blocked`；随后在内核侧关闭 `fd=3`（断言 `old_reader` 仍存活）、将新管道安装到复用后的 `fd=3`，再向原管道写端 `writer` 写入 `"X"`；
    ///   3. Then：用户进程被原管道可读事件唤醒并以状态码 0 退出；回收子进程后 `old_reader` 弱引用升级失败，证明原 OFD 已在等待结束后完全释放。
    #[test]
    fn poll_holds_original_ofd_across_close_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        // Step 1: 准备文件系统视图与用户运行时，安装标准输入/输出/错误（fd 0, 1, 2）。
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
        // Step 2: 创建被监听管道（fd=3）和握手通知管道（fd=4），并记录原管道读端的弱引用以观测其生命周期。
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

        // Step 3: 加载并启动带 `--poll-original` 参数的用户态程序，使其对 fd=3 发起阻塞式 poll。
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

        // Step 4: 等待用户进程发出 'R' 握手字节，并确认其内核线程已真正进入 Blocked 状态。
        let mut ready = [0u8; 1];
        until("descriptor user did not announce poll", || {
            ready_reader.read(&mut ready) == Ok(1)
        })?;
        context.expect_eq(ready, *b"R");
        until("descriptor poll did not block", || {
            sched::test_state(&executor) == State::Blocked
        })?;

        // Step 5: 并发关闭 fd=3 并立即分配新管道复用 fd=3，验证原 OFD 仍由 poll 持有且可被原写端唤醒。
        checked(context, files.close(fd))?;
        context.expect_eq(old_reader.upgrade().is_some(), true);
        let (replacement, replacement_writer) = checked(context, pipe::try_new(0, 0, 0))?;
        let reused = checked(context, files.reserve(limit))?
            .publish(replacement, DescriptorFlags::default());
        context.expect_eq(reused, fd);
        context.expect_eq(checked(context, writer.write(b"X"))?, 1);

        // Step 6: 等待用户进程正常退出并回收进程树，验证原 OFD 的最后一个强引用已随之释放。
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
