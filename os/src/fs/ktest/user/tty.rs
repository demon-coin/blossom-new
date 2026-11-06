//! 用户态 TTY 终端与线路规程（Line Discipline）的集成测试模块。
//!
//! ## 模块职责与架构定位
//! 本模块隶属于 `fs::ktest::user` 测试套件，用于驱动 `/test/bin/user-tty` 用户态测试程序，验证终端子系统
//! （`fs::tty`）在原始模式（Raw）、规范模式（Canonical）、`VMIN`/`VTIME` 超时控制、`poll`/`epoll` 事件就绪、
//! 字符回显（Echo）、输出缓冲区写满阻塞以及 `IXON`（`VSTOP`/`VSTART`）软件流控等场景下的端到端行为。
//! 上游由 `user.rs` 测试用例调用，下游依赖 `devfs` 终端设备与 `TtyIoOwner` 硬件串口模拟辅助夹具。
//!
//! ## 核心机制与 `FUTEX_CONTROL` 阶段同步布局
//! 内核测试线程与用户态 `/test/bin/user-tty` 进程通过固定虚拟地址 `FUTEX_CONTROL` 处的 4 个 32 位无符号整数
//! 进行阶段门控（Gate）与完成确认：
//!
//! ```text
//! FUTEX_CONTROL 虚拟地址布局 (共 16 字节):
//! +-----------------------+-------------------------------------------------------+
//! | 偏移                  | 含义与状态转换                                        |
//! +-----------------------+-------------------------------------------------------+
//! | +0  (u32: 阶段请求)   | 用户态写入当前准备进入的阶段编号 (1..N)，或 99 表示   |
//! |                       | 已完成所有测试阶段、准备关闭终端文件描述符            |
//! +-----------------------+-------------------------------------------------------+
//! | +4  (u32: 放行门控)   | 内核准备好 UART 收发数据或流控状态后，写入对应阶段号  |
//! |                       | (或 99) 放行用户态执行系统调用                        |
//! +-----------------------+-------------------------------------------------------+
//! | +8  (u32: 阶段完成)   | 用户态完成当前阶段的读写/轮询断言后，写入阶段编号通知 |
//! |                       | 内核回收本阶段的串口异步 I/O 任务                     |
//! +-----------------------+-------------------------------------------------------+
//! | +12 (i32: 最终结果)   | 用户态退出前写入测试结果码 (0 表示全部通过)           |
//! +-----------------------+-------------------------------------------------------+
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 测试线程在放行每个阶段前配置好 `TtyIoOwner` 与流控状态，放行后仅通过 `mm.load_user_u32` 和调度器状态只读轮询，
//! 不持有终端锁跨阶段等待，避免与用户态系统调用或串口工作队列死锁。

use super::*;
use crate::fs::file::FileIoContext;
use crate::fs::tty::{self, termios::Termios};

/// 用户态 TTY 测试程序的路径。
const PROGRAM: &[u8] = b"/test/bin/user-tty";

/// 用户态进程在单个 TTY 测试阶段中的预期等待行为分类。
#[derive(Clone, Copy)]
enum TtyWait {
    /// 仅通过阶段门控同步，不要求观测到内核线程进入 `Blocked` 调度状态（如纯 `tcsetattr` 或立即完成的回显）。
    GateOnly,
    /// 预期用户态线程因等待串口输入或流控恢复而在内核中进入 `Blocked` 状态。
    Blocked,
    /// 预期用户态线程因终端输出缓冲区写满且流控暂停而在内核中进入 `Blocked` 状态，随后由内核恢复流控放行。
    OutputFull,
}

impl UserOwner {
    /// 测试目标：验证用户态程序通过标准文件描述符与 `termios` 接口操作 `/dev/tty` 时的各项终端线路规程行为。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 挂载包含终端设备的 `devfs`，将全局终端属性重置为默认 `Termios` 并刷新输入输出队列，加载指定 `mode` 的 `/test/bin/user-tty` 程序。
    /// 2. When: 启动用户进程，根据 `mode`（`"r"` 原始读写、`"c"` 规范行编辑、`"v"` `VMIN`/`VTIME` 组合、`"p"` 多路复用轮询、`"e"` 字符回显、`"o"` 输出满阻塞、`"t"` 属性切换刷新、`"x"` 软件流控）分阶段注入接收字节、校验发送字节并观测线程阻塞状态。
    /// 3. Then: 在最终关闭门控（阶段 `99`）放行用户进程关闭终端描述符并退出，验证退出状态码为 `0`，最后排空终端异步工作队列并恢复默认 `Termios`。
    pub(super) fn tty_abi(&mut self, context: &mut TestContext<'_>, mode: &[u8]) -> TestResult {
        // Step 1: 准备 TTY 测试环境并重置终端 Termios 为默认配置，加载并启动用户态测试程序。
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

        // Step 2: 根据测试模式分阶段驱动串口收发数据，并验证用户线程的阻塞与唤醒状态。
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

        // Step 3: 等待用户进程到达最终关闭门控 (阶段 99)，放行其关闭终端描述符并等待进程退出。
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

        // Step 4: 冲刷终端残留异步工作并恢复默认 Termios 配置。
        tty::flush_work();
        checked(
            context,
            tty::set_termios(Termios::default(), 2, FileIoContext::Kernel),
        )
    }

    /// 执行单个 TTY 测试阶段的门控握手、模拟串口收发启动、线程阻塞观测与完成回收。
    #[expect(
        clippy::too_many_arguments,
        reason = "每段线路用例显式列出客体阶段、完整双向字节、延迟与真实生产等待预期"
    )]
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
        // Step 1: 等待用户态声明已到达目标阶段，按需配置流控暂停并启动模拟串口收发。
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
        // Step 2: 写入阶段门控字放行用户态执行系统调用，并按预期观测其是否进入内核 Blocked 状态。
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

        // Step 3: 等待用户态确认本阶段系统调用完成，并等待后台模拟串口收发任务结束。
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
