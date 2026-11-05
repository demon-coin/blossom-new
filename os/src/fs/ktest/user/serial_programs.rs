// 串口 suite 独占真实接收附着；include 保持原注册命名空间。
/// 串口业务在整组基线前附着真实 TTY 接收，不启动网络。
struct SerialSuite {
    resources: UserSuite,
    serial: SerialTtyOwner,
}

impl Fixture for SerialSuite {
    /// 共享库存与真实接收在首例基线前准备，console 输出不要求此附着。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let resources = <UserSuite as Fixture>::set_up(context, &())?;
        let serial = SerialTtyOwner::attach()?;
        crate::workqueue::testing::wait_services_blocked();
        Ok(Self { resources, serial })
    }

    /// 用户图、OFD 与 raw 阶段结束后先恢复 Raw，再停止库存。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.serial.stop()?;
        <UserSuite as Fixture>::quiesce(&mut self.resources, context)
    }

    /// 不销毁常驻 TTY 服务；确认串口停止后消费原库存。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        self.serial.stop()?;
        <UserSuite as Fixture>::tear_down(self.resources, context)
    }
}

impl Fixture<SerialSuite> for UserOwner {
    /// 串口整组持有接收附着，每例仍只建立自己的路径与用户图。
    fn set_up(context: &mut TestContext<'_>, suite: &SerialSuite) -> Result<Self, TestError> {
        <Self as Fixture<UserSuite>>::set_up(context, &suite.resources)
    }

    /// 用户图、文件和 raw 阶段按原 case 顺序停止。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::quiesce(self, context)
    }

    /// 每例先归还自身资源，接收附着保留至最后一例结束。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::tear_down(self, context)
    }
}

#[ktest::suite(fixture = UserOwner, suite_resources = SerialSuite, context = "Thread", tags("user-elf", "user-abi", "user-signal", "fs-path", "fs-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod serial_programs {
    use super::*;

    /// 普通用户raw模式逐字节交付UART输入输出，跨write保留UTF-8与二进制值。
    #[test]
    #[ktest(tags("T19", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_raw_bytes_utf8_and_binary(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"r")
    }

    /// 真实canonical行规程执行编辑键、短读、EOF token和零长边界。
    #[test]
    #[ktest(tags("T20", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_canonical_edit_and_eof(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"c")
    }

    /// 用户read真实验证四种VMIN/VTIME组合的总期限与首字节后间隔期限。
    #[test]
    #[ktest(tags("T21", "user-tty"), resources("uart"), timeout_ms = 120000)]
    fn tty_vmin_vtime_read_deadlines(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"v")
    }

    /// 用户read与poll分别验证MIN门槛和可读谓词，空立即read不伪造poll ready。
    #[test]
    #[ktest(tags("T22", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_poll_uses_independent_predicate(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"p")
    }

    /// host实际发字节时用户没有read，唯一UART Work仍消费IRQ并输出真实echo。
    #[test]
    #[ktest(tags("T23", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_irq_echo_without_reader(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"e")
    }

    /// 用户输出经ONLCR只转换一次，宿主真实背压并等到硬件TEMT。
    #[test]
    #[ktest(tags("T24", "user-tty"), resources("uart"), timeout_ms = 120000)]
    fn tty_output_crlf_backpressure_and_drain(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"o")
    }

    /// 用户36字节termios结构与跨页故障、setter无半更新和W/F提交沿raw ioctl验证。
    #[test]
    #[ktest(tags("T25", "user-tty"), resources("uart"), timeout_ms = 90000)]
    fn tty_termios_copy_fault_and_setter(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"t")
    }

    /// Ctrl-S期间用户write真实阻塞，后续RX Ctrl-Q经原Work恢复且续传字节有序。
    #[test]
    #[ktest(tags("T26", "user-tty"), resources("uart"), timeout_ms = 120000)]
    fn tty_software_flow_control_resume(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.tty_abi(context, b"x")
    }

    /// 真实前后台组通过普通wait消费TTIN、TTOU、输入信号与孤儿HUP/CONT事件。
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

    /// 三种打开身份、exec继承、leader挂断和同号新PGID沿真实OFD及用户生命周期验证。
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

    /// 完整winsize变更只向当时前台组发送WINCH，实际sigtimedwait区分相同值与copy故障。
    #[test]
    #[ktest(min_cpus = 2, tags("T29", "user-tty", "user-jobs"), timeout_ms = 90000)]
    fn jobs_winsize_changes(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.jobs_winsize_changes(context)
    }
}
