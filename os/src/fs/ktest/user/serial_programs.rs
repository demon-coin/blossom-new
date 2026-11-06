// ============================================================================
// 串口 TTY 与作业控制（Job Control）用户态测试套件（通过 `include!` 内联进 `user.rs`）
//
// 模块职责与架构定位：
// 本文件定义 `SerialSuite` 套件级夹具及 `serial_programs` 测试套件模块，
// 上游由 `fs::ktest::user::anchor()` 注册到内核测试框架，下游驱动 `UserOwner`
// 在挂载了 UART 接收中断与 TTY 行规程的环境下执行用户态测试程序。
//
// 核心设计（串口接收生命周期与测试阶段划分）：
// ```text
// SerialSuite::set_up
//   ├── 初始化底层块设备库存 (UserSuite)
//   ├── SerialTtyOwner::attach() 挂载 UART RX 到全局 TTY 行规程
//   └── 等待后台 workqueue 服务进入稳定阻塞态
//         │
//         ▼
// 逐个执行 serial_programs 测试用例 (独占 "uart" 硬件资源)
//   ├── Raw / Canonical 行规程输入输出、编辑键、EOF 与 ONLCR 转换
//   ├── VMIN / VTIME 定时读取与 poll 独立可读谓词
//   ├── IXON / IXOFF 软件流控 (Ctrl-S 暂停 / Ctrl-Q 恢复)
//   └── 会话控制终端 (Controlling TTY)、前后台进程组信号 (SIGTTIN/SIGTTOU/SIGHUP/SIGWINCH)
//         │
//         ▼
// SerialSuite::quiesce / tear_down
//   ├── SerialTtyOwner::stop() 解除串口测试附着并排空中断/Work
//   └── 归还底层设备库存并校验资源零泄漏
// ```
// ============================================================================

/// 串口与作业控制测试套件的共享资源夹具。
///
/// 在整个测试套件启动前挂载 UART 接收通路到 TTY 子系统，且不启动网络协议栈。
struct SerialSuite {
    /// 复用的基础用户态测试套件资源（管理底层块设备库存）。
    resources: UserSuite,
    /// 串口 TTY 接收中断与行规程测试归属守卫。
    serial: SerialTtyOwner,
}

impl Fixture for SerialSuite {
    /// 在首个测试用例运行前初始化设备库存并挂载串口 TTY 接收处理，最后等待后台工作队列稳定。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let resources = <UserSuite as Fixture>::set_up(context, &())?;
        let serial = SerialTtyOwner::attach()?;
        crate::workqueue::testing::wait_services_blocked();
        Ok(Self { resources, serial })
    }

    /// 在套件静默阶段停止串口接收附着并恢复默认 Raw 状态，随后静默底层设备库存。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.serial.stop()?;
        <UserSuite as Fixture>::quiesce(&mut self.resources, context)
    }

    /// 销毁套件资源：确保串口接收已停止（不销毁内核常驻 TTY 服务），随后拆卸底层设备库存。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        self.serial.stop()?;
        <UserSuite as Fixture>::tear_down(self.resources, context)
    }
}

impl Fixture<SerialSuite> for UserOwner {
    /// 为单个串口测试用例创建独立的 `UserOwner`（建立独立的文件系统路径与进程图，共享套件级串口附着）。
    fn set_up(context: &mut TestContext<'_>, suite: &SerialSuite) -> Result<Self, TestError> {
        <Self as Fixture<UserSuite>>::set_up(context, &suite.resources)
    }

    /// 按单测生命周期顺序停止当前用例的用户进程图与打开文件。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::quiesce(self, context)
    }

    /// 回收当前用例占用的文件系统与进程资源；套件级串口附着保持存活直至整组测试结束。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::tear_down(self, context)
    }
}

// 串口 TTY 行规程与终端作业控制用户态集成测试套件。
#[ktest::suite(fixture = UserOwner, suite_resources = SerialSuite, context = "Thread", tags("user-elf", "user-abi", "user-signal", "fs-path", "fs-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod serial_programs {
    use super::*;

    /// 测试目标：验证 TTY Raw 模式下的逐字节无缓冲输入输出，以及跨多次 `write` 的 UTF-8 多字节序列与任意二进制字节完整性。
    #[test]
    #[ktest(tags("T19", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_raw_bytes_utf8_and_binary(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"r")
    }

    /// 测试目标：验证 TTY Canonical（规范行）模式下的退格编辑键、按行分段短读、EOF 控制字符切分以及零长度读边界语义。
    #[test]
    #[ktest(tags("T20", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_canonical_edit_and_eof(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"c")
    }

    /// 测试目标：验证 `termios` 四种 `VMIN`/`VTIME` 组合下的读取超时语义（含无定时器等待、纯总超时、首字节无限等待加字节间超时、以及首字节即启动的字节间超时）。
    #[test]
    #[ktest(tags("T21", "user-tty"), resources("uart"), timeout_ms = 120000)]
    fn tty_vmin_vtime_read_deadlines(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"v")
    }

    /// 测试目标：验证 TTY 的 `read`（受 `VMIN` 最小字节数门槛约束）与 `poll`（只要接收缓冲区非空即判定可读）使用各自独立的就绪判定条件，且 `VMIN=0, VTIME=0` 下的空读立即返回 0 不会误使 `poll` 报就绪。
    #[test]
    #[ktest(tags("T22", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_poll_uses_independent_predicate(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"p")
    }

    /// 测试目标：验证当宿主机通过串口发送字节而此时没有任何用户进程调用 `read` 时，UART 中断下半部（Workqueue 任务）仍能及时拉取 RX FIFO 并向串口回显（Echo）字符。
    #[test]
    #[ktest(tags("T23", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_irq_echo_without_reader(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"e")
    }

    /// 测试目标：验证开启 `ONLCR` 时用户态输出的 `\n` 仅被转换为一次 `\r\n`，并在宿主机施加串口背压时能正确阻塞等待，直到硬件发送移位寄存器彻底排空（`TEMT`）。
    #[test]
    #[ktest(tags("T24", "user-tty"), resources("uart"), timeout_ms = 120000)]
    fn tty_output_crlf_backpressure_and_drain(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"o")
    }

    /// 测试目标：验证 36 字节 `termios` 结构体通过 `ioctl` 读写时的跨页内存故障（`EFAULT`）处理、属性校验失败时的原子无半更新保证，以及 `TCSETSW`/`TCSETSF` 等待输出排空与刷新输入队列的语义。
    #[test]
    #[ktest(tags("T25", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_termios_copy_fault_and_setter(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"t")
    }

    /// 测试目标：验证开启软件流控（`IXON`）后，接收到 `Ctrl-S`（XOFF）会使正在写入的用户进程阻塞，后续接收到 `Ctrl-Q`（XON）后由后台 Work 唤醒写者并按序继续发送剩余字节。
    #[test]
    #[ktest(tags("T26", "user-tty"), resources("uart"), timeout_ms = 120000)]
    fn tty_software_flow_control_resume(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"x")
    }

    /// 测试目标：验证前后台进程组切换时的终端信号投递语义，包括后台读触发 `SIGTTIN`、后台写触发 `SIGTTOU`、终端输入控制字符触发组信号，以及孤儿进程组挂断时的 `SIGHUP`/`SIGCONT` 投递。
    #[test]
    #[ktest(
        min_cpus = 2,
        tags("T27", "user-tty", "user-jobs"),
        resources("uart"),
        timeout_ms = 120000
    )]
    fn jobs_front_background_signals(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.jobs_front_background_signals(context)
    }

    /// 测试目标：验证打开终端时的三种控制终端绑定策略（是否成为会话控制终端）、跨 `execve` 继承行为、会话首进程（Session Leader）退出触发的前台挂断，以及释放后同数值编号的新 PGID 复用。
    #[test]
    #[ktest(
        min_cpus = 2,
        tags("T28", "user-tty", "user-jobs"),
        resources("uart"),
        timeout_ms = 120000
    )]
    fn jobs_open_hangup_identity(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.jobs_open_hangup_identity(context)
    }

    /// 测试目标：验证通过 `TIOCSWINSZ` 修改终端窗口大小（`winsize`）时仅在实际尺寸发生变化时向当前前台进程组发送 `SIGWINCH`，且写入相同尺寸或遇到用户地址拷贝故障（`EFAULT`）时不会误发信号。
    #[test]
    #[ktest(min_cpus = 2, tags("T29", "user-tty", "user-jobs"), timeout_ms = 90000)]
    fn jobs_winsize_changes(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.jobs_winsize_changes(context)
    }
}
