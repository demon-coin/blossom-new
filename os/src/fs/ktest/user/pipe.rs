//! 用户态管道（Pipe）系统调用 ABI、阻塞/唤醒同步与错误语义集成测试。
//!
//! # 模块定位与职责
//! 本模块位于文件系统与用户态运行时集成测试层（`fs::ktest::user`），
//! 负责加载并执行 `/test/bin/user-pipe` 测试程序，验证管道读写、文件描述符传递、
//! `EPIPE` / `SIGPIPE` 触发、不支持按偏移定位读写（`pread`/`pwrite`/`lseek` 返回 `ESPIPE`）、
//! 权限凭证检查以及管道满/空时的内核阻塞与唤醒行为。
//!
//! # 阻塞观测与共享控制页布局
//! 在需要验证管道满/空阻塞的子模式（`b`/`e`/`r`）中，内核协调器与用户态父子进程通过
//! `FUTEX_CONTROL` 共享控制页同步，确保用户态根线程确实已在内核管道队列中进入 `State::Blocked`
//! 后才放行子进程执行配对读写或关闭操作：
//! ```text
//! +-----------------------+-------------------------------------------------------+
//! | 字节偏移 (Offset)     | 用途说明                                              |
//! +-----------------------+-------------------------------------------------------+
//! | +0  (u32)             | 用户态根线程即将发起阻塞管道 I/O 前发布的阶段号       |
//! | +4  (u32)             | 内核协调器观测到根线程进入 Blocked 后写入的子进程门禁 |
//! | +12 (i32)             | `p` 模式下用户态记录的定位读写系统调用返回码（errno） |
//! +-----------------------+-------------------------------------------------------+
//! ```
//!
//! # 锁顺序与并发契约
//! - 协调器轮询 `mm.load_user_u32` 与 `sched::test_state` 时分别短暂获取地址空间锁与任务状态锁，
//!   两次调用相互独立，不存在跨锁嵌套。

use super::*;

/// 受测用户态管道测试程序的虚拟文件系统路径。
const PROGRAM: &[u8] = b"/test/bin/user-pipe";

impl UserOwner {
    /// 按指定子模式（`mode`）运行 `/test/bin/user-pipe` 管道集成测试。
    ///
    /// - **测试目标**：验证管道常规读写、`pread`/`pwrite` 返回 `ESPIPE`（`p`）、非特权用户凭证（`u`）、
    ///   与块设备/文件交互（`v`）以及管道阻塞与唤醒（`b`/`e`/`r`）等完整 ABI 行为。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 根据模式准备只读或读写测试文件系统环境；
    ///   2. 分发至对应的阻塞协调器（[`Self::pipe_blocked`]）、定位读写诊断（[`Self::pipe_positioned`]）或直接运行辅助函数；
    ///   3. 断言用户态测试主进程退出状态码为 `0`。
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

    /// 运行 `p` 子模式（对管道执行按位置读写/定位调用），并在进程退出后从共享控制页读取记录的系统调用返回码用于诊断。
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

    /// 运行需要内核阻塞观测的管道子模式（`b`/`e`/`r`）。
    ///
    /// 协调器轮询等待用户态根线程发布阶段号且其底层调度任务（`Task`）真正进入 `State::Blocked` 后，
    /// 才向 `FUTEX_CONTROL + 4` 写入 `1` 放行子进程执行对端读取或关闭，最后回收全部子进程。
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

        // Step 1: 逐阶段等待根线程在管道 I/O 上进入内核 Blocked 状态，确认门禁尚未打开后放行子进程。
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

        // Step 2: 释放协调器持有的内核对象引用，等待根进程退出并回收整个子进程树。
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
