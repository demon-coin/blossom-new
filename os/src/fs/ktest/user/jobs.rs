//! 用户态作业控制（Job Control）、控制终端（Controlling TTY）与会话生命周期集成测试。
//!
//! # 模块定位与职责
//! 本模块位于文件系统与用户态运行时集成测试层（`fs::ktest::user`），
//! 负责加载并执行 `/test/bin/user-jobs` 用户态测试程序，通过内核测试协调器（`UserOwner`）
//! 与 UART/TTY 夹具协同，验证前台/后台进程组切换、终端输入信号（`SIGINT`/`SIGTSTP`）、
//! 后台读写暂停信号（`SIGTTIN`/`SIGTTOU`）、孤儿进程组 `SIGHUP` 终止、控制终端跨 `execve` 保持、
//! 退役进程组 ID（`PGID`）对象隔离，以及终端窗口大小变更（`SIGWINCH`）等核心语义。
//!
//! # 阶段协调状态机与共享控制页布局
//! 每个测试阶段（`phase`）按如下四步握手推进：
//! ```text
//! [用户态进程]                           [内核协调器 (UserOwner)]
//!      |                                            |
//!      |--- 1. 写入 phase (+0) -------------------->|
//!      |    (若需预先进入 I/O 或停止阻塞，          |--- 2. 观测目标任务进入 Blocked / Stopped，
//!      |     子进程在此刻阻塞于内核队列)            |       按需预装 UART RX/TX 测试字节，
//!      |                                            |       写入 gate = phase (+4) 放行
//!      |<-- 3. 读取 gate 放行（或由串口中断唤醒）---|
//!      |    执行系统调用 / 消费 waitid 事件         |
//!      |--- 4. 写入 done = phase (+8) ------------->|
//!      |                                            |--- 5. 校验串口收发完成与子进程退出状态
//! ```
//!
//! `FUTEX_CONTROL` 共享控制页在作业测试中的字节偏移布局：
//! ```text
//! +-----------------------+-------------------------------------------------------+
//! | 字节偏移 (Offset)     | 用途说明                                              |
//! +-----------------------+-------------------------------------------------------+
//! | +0  (u32)             | 用户态当前发布的阶段号（`phase`）                     |
//! | +4  (u32)             | 内核协调器写入的阶段放行门禁（`gate`）                |
//! | +8  (u32)             | 用户态完成当前阶段检查后写入的确认号（`done`）        |
//! | +12 (i32)             | 用户态记录的最近一次关键系统调用返回值                |
//! | +16 (u32)             | 当前受测子进程（或旧会话首进程/替代进程）的 PID       |
//! | +20 (u32)             | 孤儿进程组测试中的孙进程 PID                          |
//! | +24 (u32)             | 替代进程组收到 `SIGUSR1` 的信号处理函数计数           |
//! | +32 (u32)             | 用户态通过 `waitid(WSTOPPED)` 累计观测到的停止事件数  |
//! | +36 (u32)             | 阶段 1 子进程成功完成前台读取的标志                   |
//! | +40 (u32)             | 用户态内部断言失败时的检查点编号（正常为 0）          |
//! | +44 (u32)             | `setsid` 脱离控制终端测试子进程的 PID                 |
//! | +48 (u32)             | `setsid` 子进程完成控制终端检查的确认标志             |
//! | +64 (u32)             | 内核协调器通知 `setsid` 子进程退出的门禁字            |
//! +-----------------------+-------------------------------------------------------+
//! ```
//!
//! # 锁顺序与并发契约
//! - 协调器通过 `mm.load_user_u32` / `mm.store_user_u32` 访问共享控制页，内部仅短暂获取 `Mm` 锁。
//! - 观测任务阻塞与停止状态时，依次调用 `sched::test_state` 与 `group.signals().has_stopped_jobs()`，
//!   两把自旋锁各自独立获取并立即释放，不存在嵌套持锁。

use super::*;
use crate::fs::file::FileIoContext;
use crate::fs::tty::{self, termios::Termios};
use crate::task::signal::SignalError;

/// 受测用户态作业控制 ELF 程序的虚拟文件系统路径。
const PROGRAM: &[u8] = b"/test/bin/user-jobs";

/// 单个作业控制测试阶段中，内核协调器对受测进程阻塞/停止状态的观测时机要求。
#[derive(Clone, Copy)]
enum JobsWait {
    /// 仅同步阶段门禁（`gate`），不在放行前后额外等待任务进入内核阻塞状态。
    GateOnly,
    /// 在打开阶段门禁（`gate`）之前，要求受测子进程已进入内核阻塞状态（`State::Blocked`）且未处于作业停止态。
    ChildBlockedBefore,
    /// 在打开阶段门禁（`gate`）之后、阶段完成（`done`）之前，要求受测子进程进入内核阻塞状态。
    ChildBlockedAfter,
    /// 在打开阶段门禁（`gate`）之后、阶段完成（`done`）之前，要求用户态根进程进入内核阻塞状态。
    RootBlockedAfter,
    /// 在打开阶段门禁（`gate`）之前，要求受测子进程已因作业控制信号进入内核停止状态（`State::Blocked` 且 `has_stopped_jobs() == true`）。
    ChildStopped,
}

impl UserOwner {
    /// 验证前台/后台进程组切换、终端信号递送（`SIGINT`/`SIGTSTP`/`SIGTTIN`/`SIGTTOU`）、作业停止与继续（`SIGCONT`）以及孤儿进程组 `SIGHUP` 终止。
    ///
    /// - **测试目标**：确认控制终端在各个作业阶段正确向目标进程组递送信号，后台进程读写终端时按规范停止，
    ///   且当中间父进程退出导致已停止的孙进程组变为孤儿进程组时，内核自动向其发送 `SIGHUP` + `SIGCONT` 使其终止。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动 `/test/bin/user-jobs` 的 `j` 模式，依次推进阶段 1~7，涵盖前台读取、`^C`（`SIGINT`）、`^Z`（`SIGTSTP`）、
    ///      `SIGCONT` 恢复前台读取、后台读触发 `SIGTTIN` 停止、忽略 `SIGTTOU` 后台写、不忽略 `SIGTTOU` 后台写停止；
    ///   2. 推进阶段 8（[`Self::jobs_orphan`]），使中间父进程退出并验证停止态孤儿孙进程被 `SIGHUP` 终止；
    ///   3. 推进阶段 99 收尾，验证根进程退出码为 `0` 并恢复终端默认 `termios` 配置。
    pub(super) fn jobs_front_background_signals(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        let mm = self.start_jobs(context, b"j")?;
        self.jobs_stage(
            context,
            &mm,
            1,
            b"a",
            b"",
            JobsWait::ChildBlockedBefore,
            1,
            None,
        )?;
        context.expect_eq(jobs_word(&mm, 36)?, 1);
        self.jobs_stage(
            context,
            &mm,
            2,
            b"\x03",
            b"",
            JobsWait::ChildBlockedAfter,
            1,
            Some(2),
        )?;
        self.jobs_stage(
            context,
            &mm,
            3,
            b"\x1a",
            b"",
            JobsWait::ChildBlockedAfter,
            1,
            None,
        )?;
        context.expect_eq(jobs_word(&mm, 32)?, 3);

        self.jobs_stage(
            context,
            &mm,
            4,
            b"b",
            b"",
            JobsWait::ChildBlockedBefore,
            3,
            Some(0),
        )?;
        self.jobs_stage(
            context,
            &mm,
            5,
            b"",
            b"D",
            JobsWait::ChildStopped,
            4,
            Some(0),
        )?;
        self.jobs_stage(context, &mm, 6, b"", b"MI", JobsWait::GateOnly, 4, Some(0))?;
        self.jobs_stage(
            context,
            &mm,
            7,
            b"",
            b"",
            JobsWait::ChildStopped,
            5,
            Some(0),
        )?;
        self.jobs_orphan(context, &mm)?;

        self.finish_jobs(context, mm, b"j")
    }

    /// 验证 `setsid` 控制终端脱离、跨 `execve` 的控制终端保持、会话挂断（Hangup）以及退役 `PGID` 引用与同号新进程组的隔离。
    ///
    /// - **测试目标**：确认 `setsid` 子进程脱离原控制终端并彻底退出内核栈；旧会话首进程退出后，
    ///   即使新创建的替代进程组复用了相同的数字 `PGID`，向旧 `Pgid` 内核对象发送信号也必须返回 `ESRCH`（`SignalError::NoProcess`），
    ///   绝不能误中同号的新进程组；同时验证控制终端通过继承的管道跨越 `execve` 保持有效。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动 `/test/bin/user-jobs` 的 `h` 模式，等待 `setsid` 子进程完成控制终端检查并验证其调度任务彻底退出；
    ///   2. 阶段 1 记录原会话首进程的 `Pgid` 强引用（`old_pgid`）及其数字编号，等待其退出；
    ///   3. 阶段 2 获取复用了相同数字编号的新进程组对象（`new_pgid`），断言两者数字相同但非同一内核对象，
    ///      并向 `old_pgid` 发送 `SIGUSR1`，断言返回 `SignalError::NoProcess` 且新进程组未收到信号；
    ///   4. 阶段 3 配合串口收发完成挂断后的控制终端读写验证，最后调用 [`Self::finish_jobs`] 收尾。
    pub(super) fn jobs_open_hangup_identity(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        let mm = self.start_jobs(context, b"h")?;
        // Step 1: 等待 setsid 子进程完成脱离控制终端的自检，随后放行退出门禁并等待其彻底离开内核栈。
        until(
            "setsid child did not finish its controlling TTY checks",
            || jobs_word(&mm, 48) == Ok(1),
        )?;
        let detached = self.jobs_thread(&mm, 44)?;
        let executor = detached
            .executor()
            .ok_or(TestError("setsid child executor disappeared"))?;
        let detached_join = executor.join_observer();
        drop((detached, executor));
        mm.store_user_u32(FUTEX_CONTROL + 64, 1)
            .map_err(|_| TestError("setsid child exit gate is not writable"))?;
        let detached_status = detached_join.join();
        context.expect_eq(detached_status, 0);
        if detached_status != 0 {
            return Err(TestError(
                "setsid child did not leave its original execution stack",
            ));
        }

        // Step 2: 阶段 1 记录旧会话首进程的 Pgid 对象及其数字编号，随后放行其退出。
        jobs_phase(&mm, 1)?;
        let leader = self.jobs_thread(&mm, 16)?;
        let identity = leader
            .identity()
            .ok_or(TestError("hangup session leader identity disappeared"))?;
        let old_pgid = identity.group().pgid_object();
        let old_number = old_pgid.number();
        drop(identity);
        drop(leader);
        self.jobs_stage(
            context,
            &mm,
            1,
            b"",
            b"",
            JobsWait::ChildBlockedBefore,
            0,
            Some(0),
        )?;

        // Step 3: 阶段 2 验证新进程组复用了相同的 PGID 编号但对应全新的 Pgid 对象，且向旧 Pgid 对象发信号返回 NoProcess。
        jobs_phase(&mm, 2)?;
        let replacement = self.jobs_thread(&mm, 16)?;
        let identity = replacement
            .identity()
            .ok_or(TestError("replacement process group identity disappeared"))?;
        let new_pgid = identity.group().pgid_object();
        let executor = replacement
            .executor()
            .ok_or(TestError("replacement process executor disappeared"))?;
        jobs_blocked(&executor, &identity.group(), false)?;
        context.expect_eq(new_pgid.number(), old_number);
        context.expect_eq(Arc::ptr_eq(&old_pgid, &new_pgid), false);
        if new_pgid.number() != old_number || Arc::ptr_eq(&old_pgid, &new_pgid) {
            return Err(TestError("PGID number was not reused by a distinct object"));
        }

        let delivered = self
            .graph()
            .send_process_group(&old_pgid, crate::task::signal::SIGUSR1);
        context.expect_eq(delivered, Err(SignalError::NoProcess));
        if delivered != Err(SignalError::NoProcess) {
            return Err(TestError(
                "retired PGID targeted the replacement process group",
            ));
        }
        context.expect_eq(jobs_word(&mm, 24)?, 0);
        drop((executor, identity, replacement, new_pgid));
        self.jobs_stage(
            context,
            &mm,
            2,
            b"",
            b"",
            JobsWait::ChildBlockedBefore,
            0,
            Some(0),
        )?;
        context.expect_eq(jobs_word(&mm, 24)?, 0);
        context.diagnostic(format_args!(
            "user-jobs retired-pgid={} new-object=1 old-send=ESRCH user-handler-count=0",
            old_number
        ));
        drop(old_pgid);

        // Step 4: 阶段 3 验证跨 execve 继承管道的控制终端读写与挂断行为，最后完成收尾。
        self.jobs_stage(
            context,
            &mm,
            3,
            b"H",
            b"N",
            JobsWait::RootBlockedAfter,
            0,
            None,
        )?;
        self.finish_jobs(context, mm, b"h")
    }

    /// 验证终端窗口大小（`winsize`）变更时向当前前台进程组递送 `SIGWINCH` 信号。
    ///
    /// - **测试目标**：确认 `TIOCSWINSZ` 更新终端尺寸后，前台子进程可通过 `sigtimedwait` 同步接收 `SIGWINCH` 并读取新尺寸。
    /// - **场景步骤（Given-When-Then）**：启动 `w` 模式，在阶段 1 观测子进程进入 `sigtimedwait` 阻塞后放行门禁，等待根进程校验通过并正常退出。
    pub(super) fn jobs_winsize_changes(&mut self, context: &mut TestContext<'_>) -> TestResult {
        let mm = self.start_jobs(context, b"w")?;
        self.jobs_stage(
            context,
            &mm,
            1,
            b"",
            b"",
            JobsWait::ChildBlockedBefore,
            0,
            Some(0),
        )?;
        self.finish_jobs(context, mm, b"w")
    }

    /// 初始化 TTY 设备与默认 `Termios` 属性，加载并启动指定模式（`mode`）的 `/test/bin/user-jobs` 程序，返回其地址空间 `Arc<Mm>`。
    fn start_jobs(&mut self, context: &TestContext<'_>, mode: &[u8]) -> Result<Arc<Mm>, TestError> {
        self.prepare_tty(context, 4)?;
        checked(
            context,
            tty::set_termios(Termios::default(), 2, FileIoContext::Kernel),
        )?;
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
        Ok(mm)
    }

    /// 从共享控制页指定偏移（`FUTEX_CONTROL + offset`）读取用户态发布的进程 PID，并在任务图中查找对应的 `UserThread`。
    fn jobs_thread(&self, mm: &Mm, offset: usize) -> Result<Arc<task::UserThread>, TestError> {
        let pid = jobs_word(mm, offset)? as usize;
        self.graph()
            .lookup_thread(pid)
            .ok_or(TestError("jobs published child has no live user binding"))
    }

    /// 执行单个作业控制测试阶段的握手、阻塞状态观测、串口收发驱动与子进程退出校验。
    ///
    /// 若本阶段包含串口收发数据（`rx` 或 `tx` 非空），则在打开阶段门禁（`gate`）之前先调用 `tty_io.begin` 启动串口事务，
    /// 并在收到阶段完成标志（`done`）之后调用 `tty_io.finish` 完成校验；无串口数据时不占用底层 UART 测试通道。
    #[expect(
        clippy::too_many_arguments,
        reason = "单段作业测试显式列出客体阶段、线路字节、真实等待状态、停止事件与退出预期"
    )]
    fn jobs_stage(
        &mut self,
        context: &mut TestContext<'_>,
        mm: &Arc<Mm>,
        phase: u32,
        rx: &[u8],
        tx: &[u8],
        waiting: JobsWait,
        stopped_events: u32,
        expected_exit: Option<i32>,
    ) -> TestResult {
        // Step 1: 等待用户态发布当前阶段号，校验累计停止事件数，并解析受测线程与进程组。
        jobs_phase(mm, phase)?;
        context.expect_eq(jobs_word(mm, 32)?, stopped_events);
        let user = if matches!(waiting, JobsWait::RootBlockedAfter) {
            let pid = self.running.as_ref().expect("jobs root missing").pid;
            self.graph()
                .lookup_thread(pid)
                .ok_or(TestError("jobs root user binding disappeared"))?
        } else {
            self.jobs_thread(mm, 16)?
        };
        let executor = user
            .executor()
            .ok_or(TestError("jobs observed executor disappeared"))?;
        let group = user
            .identity()
            .ok_or(TestError("jobs observed identity disappeared"))?
            .group();
        let completion = expected_exit.map(|status| (executor.join_observer(), status));
        if matches!(
            waiting,
            JobsWait::ChildBlockedBefore | JobsWait::ChildStopped
        ) {
            jobs_blocked(&executor, &group, matches!(waiting, JobsWait::ChildStopped))?;
        }

        // Step 2: 按需预置 UART 收发缓冲（RX 设置延迟以便协调器观测子进程阻塞窗口），随后打开阶段门禁并等待阶段完成。
        let wire = !rx.is_empty() || !tx.is_empty();
        if wire {
            // 为串口接收注入设置短暂延迟，确保协调器能在字节到达前观测到子进程进入内核 I/O 阻塞状态。
            let rx_delay = if rx.is_empty() { 0 } else { 80 };
            self.tty_io.begin(context, rx, tx, rx_delay, 0)?;
        }
        context.expect_eq(jobs_word(mm, 4)?, 0);
        jobs_gate(mm, phase)?;
        if matches!(
            waiting,
            JobsWait::ChildBlockedAfter | JobsWait::RootBlockedAfter
        ) {
            jobs_blocked(&executor, &group, false)?;
        }
        jobs_done(mm, phase)?;
        if wire {
            self.tty_io.finish()?;
        }

        context.diagnostic(format_args!(
            "user-jobs phase={} rx={} tx={} native-blocked={} native-stopped={} user-wait-stops={}",
            phase,
            rx.len(),
            tx.len(),
            u8::from(!matches!(waiting, JobsWait::GateOnly)),
            u8::from(matches!(waiting, JobsWait::ChildStopped)),
            jobs_word(mm, 32)?,
        ));
        drop((user, executor, group));

        // Step 3: 若本阶段预期子进程退出，等待其调度任务彻底离开内核栈并核对退出状态码。
        if let Some((observer, expected)) = completion {
            let actual = observer.join();
            context.expect_eq(actual, expected);
            if actual != expected {
                return Err(TestError(
                    "jobs child scheduler exit disagrees with user wait",
                ));
            }
        }
        Ok(())
    }

    /// 执行阶段 8 的孤儿进程组（Orphaned Process Group）生命周期测试。
    ///
    /// 在打开阶段门禁前预先保存中间父进程与处于停止态的孙进程的退出观察者（`JoinObserver`）；
    /// 放行门禁使中间父进程退出后，原本停止的孙进程组变为孤儿进程组，内核自动向其广播 `SIGHUP` 与 `SIGCONT`，
    /// 协调器随后通过内核父进程 `wait` 与调度器 `JoinObserver` 双重确认孙进程因 `SIGHUP`（状态码 `1`）终止。
    fn jobs_orphan(&mut self, context: &mut TestContext<'_>, mm: &Arc<Mm>) -> TestResult {
        jobs_phase(mm, 8)?;
        context.expect_eq(jobs_word(mm, 32)?, 6);
        let middle = self.jobs_thread(mm, 16)?;
        let middle_executor = middle
            .executor()
            .ok_or(TestError("orphan middle parent executor disappeared"))?;
        let middle_join = middle_executor.join_observer();
        drop((middle, middle_executor));
        let pid = jobs_word(mm, 20)? as usize;
        let user = self.jobs_thread(mm, 20)?;
        let identity = user
            .identity()
            .ok_or(TestError("stopped orphan identity disappeared"))?;
        let executor = user
            .executor()
            .ok_or(TestError("stopped orphan executor disappeared"))?;
        jobs_blocked(&executor, &identity.group(), true)?;
        assert!(
            self.lifecycle_child.is_none(),
            "jobs orphan observer already owned"
        );
        self.lifecycle_child = Some(executor.join_observer());
        drop((executor, identity, user));

        jobs_gate(mm, 8)?;
        jobs_done(mm, 8)?;
        let middle_status = middle_join.join();
        context.expect_eq(middle_status, 0);
        if middle_status != 0 {
            return Err(TestError("orphan middle parent did not exit normally"));
        }

        let event = self.graph().wait(None, pid).map_err(|error| {
            context.diagnostic(format_args!("user-jobs orphan-wait error={:?}", error));
            TestError("orphan HUP exit was not available to the real kernel parent")
        })?;
        let observer = self
            .lifecycle_child
            .take()
            .expect("jobs orphan observer missing");
        context.expect_eq(event.pid, pid);
        context.expect_eq(event.status, 1);
        context.expect_eq(event.task_id, observer.task_id());
        context.expect_eq(observer.try_join(), Some(1));
        let joined = observer.join();
        context.expect_eq(joined, 1);
        if event.status != 1 || joined != 1 {
            return Err(TestError(
                "stopped orphan did not terminate from actual SIGHUP",
            ));
        }
        context.diagnostic(format_args!(
            "user-jobs orphan={} stop-wait=19 hup-status={} scheduler-joined={} root-user-ECHILD=1",
            pid, event.status, joined
        ));
        Ok(())
    }

    /// 放行阶段 99 收尾门禁，等待 `/test/bin/user-jobs` 根进程退出，回收子进程并重置 TTY 工作队列与默认 `Termios` 属性。
    fn finish_jobs(
        &mut self,
        context: &mut TestContext<'_>,
        mm: Arc<Mm>,
        mode: &[u8],
    ) -> TestResult {
        jobs_phase(&mm, 99)?;
        jobs_gate(&mm, 99)?;
        let event = self.wait(context)?;
        let result = jobs_word(&mm, 12)? as i32;
        let failed_check = jobs_word(&mm, 40)?;
        context.diagnostic(format_args!(
            "user-jobs mode={:?} syscall-result={} failed-check={} status={}",
            mode, result, failed_check, event.status
        ));
        drop(mm);
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            return Err(TestError("ordinary jobs ABI program failed"));
        }

        self.reap_children(context, false)?;
        tty::flush_work();
        checked(
            context,
            tty::set_termios(Termios::default(), 2, FileIoContext::Kernel),
        )
    }
}

/// 通过 `Mm` 的用户空间安全访问接口读取 `FUTEX_CONTROL + offset` 处的 32 位控制字。
fn jobs_word(mm: &Mm, offset: usize) -> Result<u32, TestError> {
    mm.load_user_u32(FUTEX_CONTROL + offset)
        .map_err(|_| TestError("jobs shared control word is not readable"))
}

/// 轮询等待用户态根进程在共享控制页偏移 `+0` 处发布指定的阶段号 `phase`。
fn jobs_phase(mm: &Mm, phase: u32) -> TestResult {
    until("jobs root did not publish the requested phase", || {
        jobs_word(mm, 0) == Ok(phase)
    })
}

/// 向共享控制页偏移 `+4` 处写入阶段号 `phase`，通知用户态进程跨过当前阶段门禁。
fn jobs_gate(mm: &Mm, phase: u32) -> TestResult {
    mm.store_user_u32(FUTEX_CONTROL + 4, phase)
        .map_err(|_| TestError("jobs shared gate is not writable"))
}

/// 轮询等待用户态根进程完成当前阶段的系统调用与事件检查，并在共享控制页偏移 `+8` 处写入完成阶段号 `phase`。
fn jobs_done(mm: &Mm, phase: u32) -> TestResult {
    until("jobs root did not complete the requested phase", || {
        jobs_word(mm, 8) == Ok(phase)
    })
}

/// 轮询等待目标调度任务进入 `State::Blocked` 状态，同时验证其所属线程组的已确认作业停止状态（`has_stopped_jobs()`）与 `stopped` 预期一致。
fn jobs_blocked(
    executor: &Arc<task::Task>,
    group: &Arc<crate::task::process::ThreadGroup>,
    stopped: bool,
) -> TestResult {
    until(
        "jobs executor did not enter the required production wait",
        || {
            sched::test_state(executor) == State::Blocked
                && group.signals().has_stopped_jobs() == stopped
        },
    )
}
