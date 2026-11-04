//! 普通静态网络程序复用原用户身份图、文件表和整组网络域。

use super::*;
use crate::fs::FileRef;
use crate::task::UserThread;
use ::core::convert::TryInto;

const PROGRAM: &[u8] = b"/bin/network-smoke";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum NetworkMode {
    Identity,
    Listener,
    Connect,
    Stream,
    Backpressure,
    HalfClose,
    Address,
    Signal,
    Epoll,
    UdpAddress,
    UdpMessage,
    UnixPair,
    MessageVector,
    SocketOptions,
    Ipv6Tcp,
    Ipv6Udp,
    Ipv6Mapped,
    BufferBudget,
    ProtocolOptions,
    LargeLoopback,
    NicTcp,
    NicUdp,
    Control,
    NicControl,
}

impl NetworkMode {
    /// 参数名直接选择普通 C main 的实际消费者。
    fn argument(self) -> &'static [u8] {
        match self {
            Self::Identity => b"identity",
            Self::Listener => b"listener",
            Self::Connect => b"connect",
            Self::Stream => b"stream",
            Self::Backpressure => b"backpressure",
            Self::HalfClose => b"half-close",
            Self::Address => b"address",
            Self::Signal => b"signal",
            Self::Epoll => b"epoll",
            Self::UdpAddress => b"udp-address",
            Self::UdpMessage => b"udp-message",
            Self::UnixPair => b"unix-pair",
            Self::MessageVector => b"message-vector",
            Self::SocketOptions => b"socket-options",
            Self::Ipv6Tcp => b"ipv6-tcp",
            Self::Ipv6Udp => b"ipv6-udp",
            Self::Ipv6Mapped => b"ipv6-mapped",
            Self::BufferBudget => b"buffer-budget",
            Self::ProtocolOptions => b"protocol-options",
            Self::LargeLoopback => b"large-loopback",
            Self::NicTcp => b"nic-tcp",
            Self::NicUdp => b"nic-udp",
            Self::Control => b"control",
            Self::NicControl => b"nic-control",
        }
    }

    /// 需要协议观察或完成确认的模式继承原来的两条普通管道。
    fn coordinated(self) -> bool {
        matches!(
            self,
            Self::Connect
                | Self::Backpressure
                | Self::Signal
                | Self::UdpAddress
                | Self::UdpMessage
                | Self::UnixPair
                | Self::MessageVector
                | Self::SocketOptions
        )
    }
}

impl UserOwner {
    /// 普通 C 完成 wait/reap 后继续等待实际协议退休，整组网络 owner 保持运行。
    pub(super) fn network_mode(
        &mut self,
        context: &mut TestContext<'_>,
        mode: NetworkMode,
    ) -> TestResult {
        self.prepare_tty(context, 4)?;
        let net = self
            .net
            .as_ref()
            .ok_or(TestError("network suite has no network"))?
            .clone();
        if net.ktest_live_endpoints() != 0 {
            return Err(TestError("network user case inherited live endpoints"));
        }
        if matches!(mode, NetworkMode::Control | NetworkMode::NicControl) {
            self.network_control = Some((net.interface_snapshots(), net.route_snapshots()));
            self.mount_network_proc(context)?;
        }
        let result = self.network_run(context, mode);
        if result.is_err() {
            self.graph().request_root_exit(9);
            if self.running.is_some() {
                self.wait(context)?;
            }
            self.reap_children(context, true)?;
        }
        let observer_result = self.observer.take().map(|observer| observer.join());
        if net.ktest_has_held_syn() {
            checked(context, net.ktest_release_syn())?;
        }
        checked(context, net.ktest_clear_packet_control())?;
        network_until(
            || net.ktest_live_endpoints() == 0,
            "network user protocols did not retire",
            30,
        )?;
        if let Some(status) = observer_result {
            context.expect_eq(status, 0);
            if status != 0 {
                return Err(TestError("network user coordinator failed"));
            }
        }
        context.diagnostic(format_args!(
            "network-user mode={:?} live_endpoints={} protocols_retired=1",
            mode,
            net.ktest_live_endpoints()
        ));
        result
    }

    /// 控制例在原磁盘已有目录安装真实 proc，用户发表前完成挂载准备。
    fn mount_network_proc(&self, context: &TestContext<'_>) -> TestResult {
        let mountpoint = checked(
            context,
            self.path
                .vfs()
                .resolve(self.path.context(), b"/plain", ResolveMode::Follow),
        )?;
        let procfs = checked(context, crate::fs::procfs::try_new(0o755))?;
        checked(
            context,
            self.path.vfs().mount(
                &mountpoint,
                crate::fs::memfs::Filesystem::Memory(procfs),
                AccessMode::ReadOnly,
            ),
        )
    }

    /// 原 quiesce 在用户 join/reap 后恢复本例修改的 flags 和默认项，恢复失败保留为清理错误。
    pub(super) fn restore_network_control(&mut self, context: &TestContext<'_>) -> TestResult {
        let Some((interfaces, routes)) = self.network_control else {
            return Ok(());
        };
        let net = self
            .net
            .as_ref()
            .ok_or(TestError("network control lost its network"))?;
        for interface in interfaces.iter().flatten() {
            let current = checked(context, net.interface_snapshot(interface.id))?;
            if current.up != interface.up {
                checked(context, net.set_interface_up(interface.id, interface.up))?;
            }
        }
        let original = routes
            .iter()
            .flatten()
            .find(|route| route.prefix_len == 0 && route.gateway.is_some())
            .map(|route| (route.interface, route.gateway.unwrap()));
        let current = net
            .route_snapshots()
            .iter()
            .flatten()
            .find(|route| route.prefix_len == 0 && route.gateway.is_some())
            .map(|route| (route.interface, route.gateway.unwrap()));
        if current != original {
            if let Some((interface, gateway)) = current {
                checked(
                    context,
                    net.change_default_route(
                        DefaultRouteRequest {
                            interface: Some(interface),
                            gateway,
                        },
                        false,
                    ),
                )?;
            }
            if let Some((interface, gateway)) = original {
                checked(
                    context,
                    net.change_default_route(
                        DefaultRouteRequest {
                            interface: Some(interface),
                            gateway,
                        },
                        true,
                    ),
                )?;
            }
        }
        self.network_control = None;
        Ok(())
    }

    /// 映像、真实文件描述符及协调 Task 均在发表用户之前准备，失败由原拥有析构。
    fn network_run(&mut self, context: &mut TestContext<'_>, mode: NetworkMode) -> TestResult {
        let net = self
            .net
            .as_ref()
            .ok_or(TestError("network suite has no network"))?
            .clone();
        let credentials = if mode == NetworkMode::Identity {
            Credentials {
                uid: 1000,
                gid: 2000,
            }
        } else {
            Credentials::ROOT
        };
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
        let channels = if mode.coordinated() {
            let (ready_reader, ready_writer) = checked(context, crate::fs::pipe::try_new(0, 0, 0))?;
            let (go_reader, go_writer) = checked(context, crate::fs::pipe::try_new(0, 0, 0))?;
            checked(
                context,
                ready_reader.set_status_flags(crate::fs::O_NONBLOCK),
            )?;
            let ready_fd = checked(
                context,
                runtime
                    .files
                    .reserve(crate::task::process::DEFAULT_NOFILE.current as usize),
            )?
            .publish(ready_writer, DescriptorFlags::default());
            let go_fd = checked(
                context,
                runtime
                    .files
                    .reserve(crate::task::process::DEFAULT_NOFILE.current as usize),
            )?
            .publish(go_reader, DescriptorFlags::default());
            Some((ready_reader, go_writer, ready_fd, go_fd))
        } else {
            None
        };
        let nic = matches!(
            mode,
            NetworkMode::NicTcp | NetworkMode::NicUdp | NetworkMode::NicControl
        );
        let (first, second) = if nic {
            let (tcp, udp) = nic_ports()?.ok_or(TestError("network NIC ports are missing"))?;
            (usize::from(tcp), usize::from(udp))
        } else {
            (
                channels.as_ref().map_or(0, |x| x.2),
                channels.as_ref().map_or(0, |x| x.3),
            )
        };
        let (first_text, first_start) = decimal_fd(first);
        let (second_text, second_start) = decimal_fd(second);
        let argv: [&[u8]; 5] = [
            PROGRAM,
            mode.argument(),
            if mode == NetworkMode::Control {
                b"/plain"
            } else {
                &first_text[first_start..]
            },
            &second_text[second_start..],
            b"/plain",
        ];
        let count = match mode {
            NetworkMode::Control => 3,
            NetworkMode::NicControl => 5,
            _ if nic || channels.is_some() => 4,
            _ => 2,
        };
        let arguments =
            Arguments::from_kernel(PROGRAM, &argv[..count], &[], exec::DEFAULT_STACK_BYTES)
                .map_err(|_| TestError("network user arguments failed"))?;
        let image = exec::prepare(self.path.vfs(), self.path.context(), arguments, credentials)
            .map_err(|error| {
                context.diagnostic(format_args!("network-user image-error={:?}", error));
                TestError("network user image preparation failed")
            })?;
        let prepared = task::prepare_program(self.graph(), None, image, runtime, Placement::Any)
            .map_err(|_| TestError("network user task preparation failed"))?;
        let pid = prepared.pid();
        let coordinator = if let Some((reader, writer, _, _)) = channels {
            let user = prepared.user();
            let graph = self.graph().clone();
            let net = net.clone();
            Some(
                task::prepare_kernel(move || {
                    let result = coordinate(mode, &reader, &writer, &graph, &user, &net);
                    if let Err(error) = result {
                        crate::ktest::report::diagnostic(format_args!(
                            "network-user coordinator-error={:?}",
                            error
                        ));
                        graph.request_root_exit(9);
                        if net.ktest_has_held_syn() {
                            let _ = net.ktest_release_syn();
                        }
                        return 1;
                    }
                    0
                })
                .map_err(|_| TestError("network coordinator preparation failed"))?,
            )
        } else {
            None
        };
        match mode {
            NetworkMode::Connect => checked(context, net.ktest_hold_syn(45204))?,
            NetworkMode::Signal => checked(context, net.ktest_hold_syn(45214))?,
            _ => {}
        }
        let join = prepared
            .publish()
            .map_err(|_| TestError("network user publication failed"))?;
        self.running = Some(RunningProgram {
            pid,
            target: None,
            coordinator: cpu::current().id,
            expected_task: Some(join.task_id()),
            join,
        });
        self.observer = coordinator.map(|task| task.publish());
        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        if event.status != 0 {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, event.status != 0)?;
        if event.status != 0 {
            return Err(TestError("ordinary network program failed"));
        }
        Ok(())
    }
}

/// 测试仅观察事实；有限期限不驱动协议或覆盖生产时钟。
pub(super) fn network_until(
    mut ready: impl FnMut() -> bool,
    error: &'static str,
    seconds: u64,
) -> TestResult {
    let end = time::now()
        .checked_add(time::Duration::from_secs(seconds))
        .ok_or(TestError("network user observation deadline overflow"))?;
    while !ready() {
        if end <= time::now() {
            return Err(TestError(error));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 原非阻塞 pipe 逐字节取得 C 交付的三个 uint32 字段，退出和 EOF 都是失败。
fn message(reader: &FileRef, root: &UserThread) -> Result<[u32; 3], TestError> {
    let mut bytes = [0u8; 12];
    let mut done = 0;
    let end = time::now()
        .checked_add(time::Duration::from_secs(10))
        .ok_or(TestError("network report deadline overflow"))?;
    while done < bytes.len() {
        match reader.read(&mut bytes[done..]) {
            Ok(0) => return Err(TestError("network report ended before expected phase")),
            Ok(count) => done += count,
            Err(FsError::WouldBlock) => {
                if root.mm().is_none() || end <= time::now() {
                    return Err(TestError("network report did not arrive"));
                }
                timer::sleep(time::Duration::from_millis(1));
            }
            Err(_) => return Err(TestError("network report read failed")),
        }
    }
    Ok(::core::array::from_fn(|i| {
        u32::from_le_bytes(bytes[i * 4..i * 4 + 4].try_into().unwrap())
    }))
}

/// 阶段和 errno 必须由 C 真实报告，不能只确认有一条管道数据。
fn phase(
    reader: &FileRef,
    root: &UserThread,
    expected: u8,
    value: u32,
) -> Result<usize, TestError> {
    let message = message(reader, root)?;
    if message[0] != u32::from(expected) || message[2] != value {
        return Err(TestError("network C reported unexpected phase or errno"));
    }
    crate::ktest::report::diagnostic(format_args!(
        "network-user report-phase={} pid={} value={}",
        expected as char, message[1], message[2]
    ));
    Ok(message[1] as usize)
}

/// 选择原身份图中的实际 Task 并观察 Blocked，不把用户提示字当成等待证明。
fn blocked(graph: &TaskGraph, pid: usize) -> TestResult {
    let user = graph
        .lookup_thread(pid)
        .ok_or(TestError("network reported PID disappeared"))?;
    let task = user
        .executor()
        .ok_or(TestError("network reported Task disappeared"))?;
    network_until(
        || sched::test_state(&task) == State::Blocked || user.mm().is_none(),
        "network user did not block",
        5,
    )?;
    if user.mm().is_none() {
        return Err(TestError("network user exited before blocking"));
    }
    Ok(())
}

/// 放行只写真实 pipe，不直接修改用户内存或协议状态。
fn go(writer: &FileRef) -> TestResult {
    if writer.write(b"G") != Ok(1) {
        return Err(TestError("network control pipe write failed"));
    }
    Ok(())
}

/// 按真实阶段观察协议、发送阻塞和信号，消息模式最后确认普通 C 已完成全部断言。
fn coordinate(
    mode: NetworkMode,
    reader: &FileRef,
    writer: &FileRef,
    graph: &TaskGraph,
    root: &UserThread,
    net: &NetStackRef,
) -> TestResult {
    match mode {
        NetworkMode::Connect => {
            phase(reader, root, b'C', 114)?;
            network_until(|| net.ktest_has_held_syn(), "connect SYN was not held", 5)?;
            net.ktest_release_syn()
                .map_err(|_| TestError("connect SYN release failed"))?;
            go(writer)
        }
        NetworkMode::Backpressure => {
            let pid = phase(reader, root, b'B', 0)?;
            blocked(graph, pid)?;
            crate::ktest::report::diagnostic(format_args!(
                "network-user writer-pid={} full-send-blocked=1",
                pid
            ));
            go(writer)
        }
        NetworkMode::Signal => {
            let pid = phase(reader, root, b'R', 0)?;
            blocked(graph, pid)?;
            graph
                .send_kernel(pid, crate::task::signal::SIGUSR1)
                .map_err(|_| TestError("empty read signal failed"))?;
            phase(reader, root, b'I', 4)?;
            go(writer)?;
            let child = phase(reader, root, b'T', 0)?;
            blocked(graph, child)?;
            graph
                .send_kernel(child, crate::task::signal::SIGUSR1)
                .map_err(|_| TestError("restarted read signal failed"))?;
            if phase(reader, root, b'H', 10)? != child {
                return Err(TestError("read handler ran in wrong process"));
            }
            blocked(graph, child)?;
            go(writer)?;
            let pid = phase(reader, root, b'K', 0)?;
            network_until(
                || net.ktest_has_held_syn(),
                "interrupted connect SYN missing",
                5,
            )?;
            blocked(graph, pid)?;
            graph
                .send_kernel(pid, crate::task::signal::SIGUSR1)
                .map_err(|_| TestError("connect signal failed"))?;
            phase(reader, root, b'J', 4)?;
            net.ktest_release_syn()
                .map_err(|_| TestError("interrupted SYN release failed"))?;
            go(writer)?;
            phase(reader, root, b'V', 0)?;
            net.ktest_clear_packet_control()
                .map_err(|_| TestError("previous SYN control did not end"))?;
            net.ktest_hold_syn(45215)
                .map_err(|_| TestError("restart SYN control failed"))?;
            go(writer)?;
            let pid = phase(reader, root, b'W', 0)?;
            network_until(
                || net.ktest_has_held_syn(),
                "restarted connect SYN missing",
                5,
            )?;
            blocked(graph, pid)?;
            graph
                .send_kernel(pid, crate::task::signal::SIGUSR1)
                .map_err(|_| TestError("restart connect signal failed"))?;
            if phase(reader, root, b'H', 10)? != pid {
                return Err(TestError("connect handler ran in wrong process"));
            }
            blocked(graph, pid)?;
            net.ktest_release_syn()
                .map_err(|_| TestError("restart SYN release failed"))?;
            phase(reader, root, b'Z', 0)?;
            Ok(())
        }
        NetworkMode::UnixPair => {
            let pid = phase(reader, root, b'U', 0)?;
            blocked(graph, pid)?;
            go(writer)?;
            phase(reader, root, b'4', 0)?;
            go(writer)
        }
        NetworkMode::UdpAddress
        | NetworkMode::UdpMessage
        | NetworkMode::MessageVector
        | NetworkMode::SocketOptions => {
            phase(reader, root, b'4', 0)?;
            go(writer)
        }
        _ => Err(TestError("uncoordinated network mode has coordinator")),
    }
}

/// 两个明确的启动端口必须成对且各出现一次；没有 token 时保留原 lo 域。
pub(super) fn nic_ports() -> Result<Option<(u16, u16)>, TestError> {
    let mut tcp = None;
    let mut udp = None;
    for token in crate::boot::info()
        .bootargs()
        .split(|byte| byte.is_ascii_whitespace())
    {
        let (slot, value) = if let Some(value) = token.strip_prefix(b"ktest-net-tcp=") {
            (&mut tcp, value)
        } else if let Some(value) = token.strip_prefix(b"ktest-net-udp=") {
            (&mut udp, value)
        } else if token == b"ktest-net-tcp" || token == b"ktest-net-udp" {
            return Err(TestError("network NIC port token has no value"));
        } else {
            continue;
        };
        if slot.is_some() {
            return Err(TestError("network NIC port token is duplicated"));
        }
        *slot = Some(service_port(value)?);
    }
    match (tcp, udp) {
        (Some(tcp), Some(udp)) => Ok(Some((tcp, udp))),
        (None, None) => Ok(None),
        _ => Err(TestError("network NIC ports must be supplied together")),
    }
}

/// 服务端口只接受十进制数字的 1..65535，空值、符号和溢出均是设置错误。
fn service_port(bytes: &[u8]) -> Result<u16, TestError> {
    let mut port = 0u16;
    if bytes.is_empty() {
        return Err(TestError("network NIC port is empty"));
    }
    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return Err(TestError("network NIC port is not decimal"));
        }
        port = port
            .checked_mul(10)
            .and_then(|value| value.checked_add(u16::from(byte - b'0')))
            .ok_or(TestError("network NIC port exceeds 65535"))?;
    }
    if port == 0 {
        return Err(TestError("network NIC port is zero"));
    }
    Ok(port)
}

/// 描述符或服务端口只在准备期编码一次，固定栈数组不创建额外字符串拥有。
fn decimal_fd(mut value: usize) -> ([u8; 20], usize) {
    let mut bytes = [0u8; 20];
    let mut start = bytes.len();
    loop {
        start -= 1;
        bytes[start] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
            return (bytes, start);
        }
    }
}
