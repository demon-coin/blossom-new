//! 普通用户串口 ELF 消费真实 UART 阶段；门控、生产等待与退出回收分别观察。

use super::*;
use crate::fs::file::FileIoContext;
use crate::fs::tty::{self, termios::Termios};

const PROGRAM: &[u8] = b"/test/bin/user-tty";

#[derive(Clone, Copy)]
enum TtyWait {
    GateOnly,
    Blocked,
    OutputFull,
}

impl UserOwner {
    /// 本例挂载真实 devfs 后执行双目标 ELF，所有阶段结束才允许关闭最后串口 OFD。
    pub(super) fn tty_abi(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        self.prepare_tty(context, 4)?;
        checked(
            context,
            tty::set_termios(
                Termios::default(),
                tty::termios::TCSAFLUSH,
                FileIoContext::Kernel,
            ),
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
        let pid = self
            .running
            .as_ref()
            .expect("TTY program was not started")
            .pid;
        let user = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("TTY root exited before observation"))?;
        let executor = user
            .executor()
            .ok_or(TestError("TTY root executor disappeared"))?;
        match mode {
            b"r" => {
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    1,
                    &[0, 0xc3, 0xa9, 0xff, 10, 0x80],
                    &[0, 0xc3, 0xa9, 0xff, 10, 0x80],
                    30,
                    0,
                    TtyWait::Blocked,
                )?;
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    2,
                    b"abc",
                    b"",
                    30,
                    0,
                    TtyWait::Blocked,
                )?;
            }
            b"c" => self.tty_stage(
                context,
                &mm,
                &executor,
                1,
                b"abX\x7fc\nword\x17z\nkilled\x15ok\npartial\x04\x04\x16\x04\n",
                b"",
                30,
                0,
                TtyWait::Blocked,
            )?,
            b"v" => {
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    1,
                    b"",
                    b"",
                    0,
                    0,
                    TtyWait::GateOnly,
                )?;
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    2,
                    b"xyz",
                    b"",
                    80,
                    0,
                    TtyWait::Blocked,
                )?;
                self.tty_stage(context, &mm, &executor, 3, b"", b"", 0, 0, TtyWait::Blocked)?;
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    4,
                    b"!",
                    b"",
                    30,
                    0,
                    TtyWait::Blocked,
                )?;
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    5,
                    b"A",
                    b"",
                    30,
                    0,
                    TtyWait::Blocked,
                )?;
            }
            b"p" => {
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    1,
                    b"z",
                    b"",
                    30,
                    0,
                    TtyWait::Blocked,
                )?;
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    2,
                    b"q",
                    b"",
                    30,
                    0,
                    TtyWait::Blocked,
                )?;
            }
            b"e" => self.tty_stage(
                context,
                &mm,
                &executor,
                1,
                b"abX\x7fc\r",
                b"abX\x08 \x08c\r\n",
                0,
                0,
                TtyWait::GateOnly,
            )?,
            b"o" => {
                let output = b"O\r\n".repeat(32768);
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    1,
                    b"",
                    &output,
                    0,
                    1,
                    TtyWait::OutputFull,
                )?;
            }
            b"t" => {
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    1,
                    b"old",
                    b"",
                    0,
                    0,
                    TtyWait::GateOnly,
                )?;
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    2,
                    b"",
                    b"",
                    0,
                    0,
                    TtyWait::GateOnly,
                )?;
            }
            b"x" => {
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    1,
                    b"\x13",
                    b"",
                    0,
                    0,
                    TtyWait::GateOnly,
                )?;
                let output = alloc::vec![b'F'; 12288];
                self.tty_stage(
                    context,
                    &mm,
                    &executor,
                    2,
                    b"\x11",
                    &output,
                    100,
                    1,
                    TtyWait::Blocked,
                )?;
            }
            _ => return Err(TestError("unknown TTY user mode")),
        }
        until("TTY root did not reach the final close gate", || {
            mm.load_user_u32(FUTEX_CONTROL) == Ok(99)
        })?;
        mm.store_user_u32(FUTEX_CONTROL + 4, 99)
            .map_err(|_| TestError("TTY close gate is not writable"))?;
        drop(executor);
        drop(user);
        let event = self.wait(context)?;
        let result = mm
            .load_user_u32(FUTEX_CONTROL + 12)
            .map_err(|_| TestError("TTY result is not readable"))? as i32;
        context.diagnostic(format_args!(
            "user-tty mode={:?} syscall-result={} status={}",
            mode, result, event.status
        ));
        drop(mm);
        let cancelling = event.status != 0;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        context.expect_eq(event.status, 0);
        if cancelling {
            return Err(TestError("ordinary TTY ABI program failed"));
        }
        tty::flush_work();
        checked(
            context,
            tty::set_termios(Termios::default(), 2, FileIoContext::Kernel),
        )
    }

    /// 声明线路前等待配置已发表；放行后独立观察实际阻塞，不把 yield 门控当等待。
    fn tty_stage(
        &mut self,
        context: &mut TestContext<'_>,
        mm: &Arc<Mm>,
        executor: &Arc<task::Task>,
        phase: u32,
        rx: &[u8],
        tx: &[u8],
        rx_delay_ms: u32,
        tx_delay_ms: u32,
        waiting: TtyWait,
    ) -> TestResult {
        until("TTY root did not publish the requested phase", || {
            mm.load_user_u32(FUTEX_CONTROL) == Ok(phase)
        })?;
        if matches!(waiting, TtyWait::OutputFull) {
            checked(context, tty::flow(0))?;
        }
        let wire = !rx.is_empty() || !tx.is_empty();
        if wire {
            self.tty_io
                .begin(context, rx, tx, rx_delay_ms, tx_delay_ms)?;
        }
        context.expect_eq(mm.load_user_u32(FUTEX_CONTROL + 4), Ok(0));
        mm.store_user_u32(FUTEX_CONTROL + 4, phase)
            .map_err(|_| TestError("TTY stage gate is not writable"))?;
        let blocked = !matches!(waiting, TtyWait::GateOnly);
        if blocked {
            until("TTY root did not enter production waiting", || {
                sched::test_state(executor) == State::Blocked
            })?;
        }
        if matches!(waiting, TtyWait::OutputFull) {
            checked(context, tty::flow(1))?;
        }
        until("TTY root did not complete the UART stage", || {
            mm.load_user_u32(FUTEX_CONTROL + 8) == Ok(phase)
        })?;
        if wire {
            self.tty_io.finish()?;
        }
        context.diagnostic(format_args!(
            "user-tty phase={} rx={} tx={} native-root-blocked={}",
            phase,
            rx.len(),
            tx.len(),
            u8::from(blocked)
        ));
        Ok(())
    }
}
