//! 用户态事件等待与唤醒（Event/Futex/Poll 等）ABI 集成测试模块。
//!
//! # 模块职责与架构定位
//! 本模块位于用户态集成测试层（`fs::ktest::user`），为 `UserOwner` 提供运行
//! `/test/bin/user-events` 测试程序的方法（`events_abi` 与 `events_blocked`）。
//! 上游由 `user/programs.rs` 中的事件类测试用例调用，下游依赖 `UserOwner` 的进程加载、
//! `Mm::load_user_u32` / `store_user_u32` 跨地址空间共享控制字访问，以及调度器任务状态观测。
//!
//! # 阻塞门控协同协议（`FUTEX_CONTROL` 内存布局）
//! 在需要验证“父进程真正进入内核 `Blocked` 状态后才允许子进程发起唤醒”的测试模式（`"b"` / `"s"`）中，
//! 内核测试线程与用户态进程通过虚拟地址 `FUTEX_CONTROL`（`0x4400_0000`）处的 32 位控制字协同：
//!
//! ```text
//! 虚拟地址               含义与读写方向
//! +--------------------+-------------------------------------------------------------+
//! | FUTEX_CONTROL + 0  | 当前阶段号 phase (1..=3)：用户态根进程进入阻塞系统调用前写入 |
//! | FUTEX_CONTROL + 4  | 子进程放行门控 gate (0/1)：内核确认根进程已 Blocked 后置为 1 |
//! | FUTEX_CONTROL + 12 | 系统调用返回值 result：用户态记录系统调用完成结果供内核诊断  |
//! +--------------------+-------------------------------------------------------------+
//! ```
//!
//! # 锁与并发契约
//! - 内核测试线程通过 `until` 轮询用户地址空间控制字与 `sched::test_state(&executor) == State::Blocked`；
//! - 必须在调用 `self.wait(context)` 回收进程前显式释放 `executor` 与 `user` 强引用，并在调用 `reap_children` 前释放 `mm` 引用，确保进程退出后任务控制块与地址空间引用计数归零。

use super::*;

/// 用户态事件测试 ELF 程序的路径。
const PROGRAM: &[u8] = b"/test/bin/user-events";

impl UserOwner {
    /// 加载并运行 `/test/bin/user-events` 指定测试模式，验证退出状态码为 0。
    ///
    /// - **参数**：
    ///   - `context`：`ktest` 测试上下文，用于记录诊断信息与断言；
    ///   - `mode`：传递给 `/test/bin/user-events` 的命令行模式参数（`"b"` 与 `"s"` 走内核阻塞门控协同路径，其余模式直接运行至结束）。
    pub(super) fn events_abi(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        self.prepare(context)?;

        // Step 1: 根据测试模式选择带内核阻塞状态观测的协同执行路径，或普通直通执行路径。
        let event = if matches!(mode, b"b" | b"s") {
            self.events_blocked(context, mode)?
        } else {
            self.run_named(context, PROGRAM, mode, false, 0)?
        };

        // Step 2: 校验用户态测试程序以状态码 0 正常退出。
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            return Err(TestError("ordinary event ABI program failed"));
        }
        Ok(())
    }

    /// 运行需要内核协同观测阻塞状态的三阶段事件测试（模式 `"b"` 或 `"s"`）。
    ///
    /// 在每一阶段（共 3 轮）中，先等待用户态根线程将阶段号写入 `FUTEX_CONTROL` 且其内核任务状态
    /// 真正变为 `State::Blocked`，然后再将 `FUTEX_CONTROL + 4` 处的子进程门控置为 `1`，放行用户态子进程触发唤醒。
    fn events_blocked(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
    ) -> Result<WaitEvent, TestError> {
        // Step 1: 加载用户态程序镜像并保留其地址空间引用 `mm`，以便后续直接读写 `FUTEX_CONTROL` 共享控制区。
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

        // Step 2: 从进程图中查找根线程及其对应的内核执行任务（`executor`），用于观测调度状态。
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

        // Step 3: 依次推进 3 个测试阶段；每阶段确认根线程已在目标系统调用中进入 Blocked 状态后，再打开子进程通知门控。
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

        // Step 4: 释放对根线程与执行任务的临时强引用，等待根进程退出并读取记录在用户内存中的系统调用结果。
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

        // Step 5: 释放地址空间引用，若根进程异常退出则请求终止整个进程图，最后回收所有孤儿或子进程。
        drop(mm);
        let cancelling = event.status != 0;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        Ok(event)
    }
}
