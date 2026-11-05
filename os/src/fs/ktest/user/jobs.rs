//! 普通作业控制 ELF 的阶段协调；用户 wait、生产阻塞、稳定 PGID 与离栈完成分别核对。

use super::*;
use crate::fs::file::FileIoContext;
use crate::fs::tty::{self, termios::Termios};
use crate::task::signal::SignalError;

const PROGRAM: &[u8] = b"/test/bin/user-jobs";

#[derive(Clone, Copy)]
enum JobsWait {
    GateOnly,
    ChildBlockedBefore,
    ChildBlockedAfter,
    RootBlockedAfter,
    ChildStopped,
}

impl UserOwner {
    /// 同一原 UserOwner 承担真实前后台、停止、继续、输入信号与孤儿组退出的全部拥有。
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

    /// exec 通过真实继承 pipe 保 ctty；旧 Arc 的递送不得命中新同号进程组。
    pub(super) fn jobs_open_hangup_identity(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        let mm = self.start_jobs(context, b"h")?;
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

    /// winsize 的实际信号由前台子进程 sigtimedwait 消费，不声明空 UART 阶段。
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

    /// 与旧串口客体共用 devfs、真实磁盘 loader、TaskGraph 和 UART 阶段 owner。
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

    /// 从客体公布的真实 PID 找原执行者，不通过 phase/yield 推断其生产等待状态。
    fn jobs_thread(&self, mm: &Mm, offset: usize) -> Result<Arc<task::UserThread>, TestError> {
        let pid = jobs_word(mm, offset)? as usize;
        self.graph()
            .lookup_thread(pid)
            .ok_or(TestError("jobs published child has no live user binding"))
    }

    /// 有线路先 begin 再放 gate，done 后 finish；无线路完全不取得 raw UART 所有权。
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
        let wire = !rx.is_empty() || !tx.is_empty();
        if wire {
            // RX 延迟给协调者实际观察 child 的生产 Blocked 窗口，不替代该观察。
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

    /// 停止孙组的观察者在 gate 前保存，HUP 退出事件与原 join 均在正常路径消费。
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

    /// 99 gate 只在所有 UART 和子事件结束后放行；正常退出仍由原 wait/reap/join 收尾。
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

/// 共享页只经正式 Mm uaccess 读数，不创建用户可写字节的 Rust 引用。
fn jobs_word(mm: &Mm, offset: usize) -> Result<u32, TestError> {
    mm.load_user_u32(FUTEX_CONTROL + offset)
        .map_err(|_| TestError("jobs shared control word is not readable"))
}

/// guest 发布 phase 后保持 gate 等待，控制者不把该等待算作 I/O 阻塞。
fn jobs_phase(mm: &Mm, phase: u32) -> TestResult {
    until("jobs root did not publish the requested phase", || {
        jobs_word(mm, 0) == Ok(phase)
    })
}

/// 放行只更新既有共享 gate，不增加测试 syscall 或伪造 scheduler 状态。
fn jobs_gate(mm: &Mm, phase: u32) -> TestResult {
    mm.store_user_u32(FUTEX_CONTROL + 4, phase)
        .map_err(|_| TestError("jobs shared gate is not writable"))
}

/// done 由普通客体完成真实 syscall 和 wait 后发布，UART ACK 不提前消费。
fn jobs_done(mm: &Mm, phase: u32) -> TestResult {
    until("jobs root did not complete the requested phase", || {
        jobs_word(mm, 8) == Ok(phase)
    })
}

/// 生产 Task Blocked 与全组已确认 stop 分开查询，pending stop 不充当停止作业。
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
