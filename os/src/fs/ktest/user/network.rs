//! 用户态网络测试程序（`/bin/network-smoke`）的进程装载、双管道阶段同步与协议栈状态恢复执行器。
//!
//! # 模块定位
//! 本模块属于用户态集成测试子集（`fs::ktest::user::network`），为 `network_programs` 测试套件中的
//! 各个网络测试用例提供统一的运行时准备、用户进程与内核协调线程（Coordinator）同步、以及测试后网络状态清理逻辑。
//!
//! # 核心机制与双管道同步协议
//! 需要内核侧配合（如拦截/放行 SYN 报文、观测发送缓冲区阻塞、注入 `SIGUSR1` 信号）的测试模式，
//! 会在创建用户进程前预置一对匿名管道（`ready` 报告管道与 `go` 放行管道）：
//!
//! ```text
//! 用户态进程 (/bin/network-smoke)                 内核协调线程 (coordinate)
//! +------------------------------+               +----------------------------------+
//! | 执行 socket / connect / read |               |                                  |
//! | 写入 12B 阶段报告:           |  ready 管道   |                                  |
//! | [phase: u32, pid: u32, val]  | ------------> | message() / phase() 校验阶段与值 |
//! |                              |               | blocked() 轮询确认任务进入睡眠   |
//! | (在套接字或 go 管道上阻塞)   |               | 释放 SYN / 发送 SIGUSR1 信号     |
//! |                              |   go 管道     |                                  |
//! | 收到 'G' 后继续下一阶段断言  | <------------ | go() 写入单字节 b"G" 放行        |
//! +------------------------------+               +----------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 用户进程与内核协调线程通过无共享锁的管道 I/O（`FileRef::read` / `write`）与任务状态快照（`sched::test_state`）通信。
//! - 测试收尾时严格按顺序执行：等待用户根进程退出并回收全部子进程 -> 汇合协调线程 -> 释放挂起的 SYN 拦截钩子 ->
//!   等待所有活跃网络端点（`ktest_live_endpoints`）排空归零 -> 恢复网卡开关与默认路由配置。

use super::*;
use crate::fs::FileRef;
use crate::task::UserThread;
use ::core::convert::TryInto;

/// 磁盘镜像中用于承载全部用户态网络集成测试的子命令聚合二进制路径。
const PROGRAM: &[u8] = b"/bin/network-smoke";

/// `/bin/network-smoke` 支持的子测试模式枚举，每个变体对应一个命令行子模式参数。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum NetworkMode {
    /// 非特权用户（UID 1000 / GID 2000）创建套接字，验证 `dup`/`dup3` 共享打开文件描述（OFD）与独立描述符标志。
    Identity,
    /// 监听套接字描述符复制后交替调用 `accept`，验证每次接受连接均返回独立的打开文件描述。
    Listener,
    /// 非阻塞 TCP `connect` 在 SYN 挂起期间返回 `EINPROGRESS`、对关闭端口返回 `ECONNREFUSED`，并验证 `SO_ERROR` 单次读取清零。
    Connect,
    /// TCP 字节流读取测试：覆盖完整读取、`MSG_PEEK` 窥视、短缓冲区前缀读取以及对端关闭后的 FIN/EOF 行为。
    Stream,
    /// TCP 发送缓冲区写满后阻塞写进程，待对端读取并由后台处理 ACK 后恢复写入进度。
    Backpressure,
    /// TCP `shutdown(SHUT_WR)` 半关闭后继续接收数据，并验证向已关闭通道写入时的 `SIGPIPE` 信号与 `MSG_NOSIGNAL` 抑制。
    HalfClose,
    /// 验证 `accept` 的地址缓冲区截断、无效用户地址返回 `EFAULT` 以及进程文件描述符表满返回 `EMFILE` 时的连接保留语义。
    Address,
    /// 验证阻塞 `read` 与 `connect` 被 `SIGUSR1` 信号打断时的 `EINTR` 返回、`SA_RESTART` 自动重启及 `poll` 超时行为。
    Signal,
    /// 验证同一套接字在 `epoll` 中的水平/边缘触发、监听事件修改、删除以及描述符关闭复用行为。
    Epoll,
    /// 验证 UDP 套接字显式目标地址发送、`connect` 设置默认对端、源地址过滤以及 `AF_UNSPEC` 断开默认对端。
    UdpAddress,
    /// 验证 UDP 零长度数据报收发、短缓冲区截断丢弃尾部以及 `MSG_PEEK | MSG_TRUNC` 返回完整报文长度。
    UdpMessage,
    /// 验证 `socketpair(AF_UNIX, SOCK_STREAM)` 匿名双向流跨环形缓冲区容量的阻塞传输、共享 OFD 及对端关闭后的错误码。
    UnixPair,
    /// 验证 `sendmsg`/`recvmsg` 多段 `iovec` 分散/聚集传输，以及部分段遇到无效用户指针（`EFAULT`）时的已传输前缀处理。
    MessageVector,
    /// 验证 `SO_REUSEADDR` 地址复用策略、监听端口冲突、UDP 绑定地址选择以及 TCP 监听选项向 `accept` 子套接字的继承。
    SocketOptions,
    /// 验证 IPv6 回环（`::1`）TCP 握手、`getsockname`/`getpeername` 地址长度、父子进程双向流传输与半关闭。
    Ipv6Tcp,
    /// 验证 IPv6 回环（`::1`）UDP 多目标收发、默认对端、跨页大报文及 64 位 `msghdr`/`iovec` 向量读写。
    Ipv6Udp,
    /// 验证 IPv6 双栈套接字接收 IPv4 映射地址（`::ffff:127.0.0.1`）、端口冲突检测以及 `IPV6_V6ONLY` 隔离语义。
    Ipv6Mapped,
    /// 验证 `SO_SNDBUF`/`SO_RCVBUF` 缓冲区预算控制：TCP 发送满后恢复、UDP 单报文按预算接纳，以及调小预算不丢失已入队数据。
    BufferBudget,
    /// 验证 IPv4/IPv6 TCP 连接查询 `TCP_MAXSEG`、`TCP_CONGESTION`（`"reno"`）与 `TCP_NODELAY`，以及非 TCP 套接字和非法长度的错误码。
    ProtocolOptions,
    /// 验证回环接口上接近协议上限的大尺寸 IPv4/IPv6 UDP 数据报完整收发，以及超出上限 1 字节时返回 `EMSGSIZE`。
    LargeLoopback,
    /// 通过外部网卡（NIC）与宿主机测试搭档建立 TCP 连接，验证双向正文传输、短读、半关闭及端点地址。
    NicTcp,
    /// 通过外部网卡（NIC）与宿主机测试搭档收发 3 条 UDP 报文，并验证超过链路 MTU 的报文被拒绝。
    NicUdp,
    /// 回环控制面测试：通过 `ioctl` 配置 IPv4/IPv6 接口地址与标志，并读取 `/proc/net`（挂载于 `/plain`）验证快照隔离与共享游标。
    Control,
    /// 外部网卡控制面测试：删除并重新添加默认路由后，通过外部网卡与宿主机完成 TCP 双向通信并核对 `/proc/net` 计数。
    NicControl,
}

impl NetworkMode {
    /// 返回传递给 `/bin/network-smoke` 第一个命令行参数的子模式名称字节串。
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

    /// 判断当前测试模式是否需要创建内核协调线程（Coordinator）及配套的 `ready`/`go` 同步管道。
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
    /// 运行指定的用户态网络测试模式 `mode`，并在用户进程退出后等待全部网络端点与后台协议状态彻底排空。
    pub(super) fn network_mode(
        &mut self,
        context: &mut TestContext<'_>,
        mode: NetworkMode,
    ) -> TestResult {
        // Step 1: 初始化终端与网络栈引用，校验测试开始前无残留的活跃网络端点；控制面模式额外保存初始接口/路由快照并挂载 procfs。
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

        // Step 2: 启动用户态测试进程并等待其执行完毕；若中途出错则强制请求进程组退出并回收子进程。
        let result = self.network_run(context, mode);
        if result.is_err() {
            self.graph().request_root_exit(9);
            if self.running.is_some() {
                self.wait(context)?;
            }
            self.reap_children(context, true)?;
        }

        // Step 3: 汇合内核协调线程，释放可能挂起的 SYN 报文拦截器，并等待所有套接字端点彻底关闭回收。
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

    /// 为网络控制面测试在磁盘已有的 `/plain` 目录上只读挂载 `procfs`，供用户态程序读取网络接口与路由表快照。
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

    /// 在测试套件静默（`quiesce`）阶段，将本测试修改过的网络接口启停标志（UP/DOWN）及默认路由恢复为测试前的初始快照。
    pub(super) fn restore_network_control(&mut self, context: &TestContext<'_>) -> TestResult {
        let Some((interfaces, routes)) = self.network_control else {
            return Ok(());
        };
        let net = self
            .net
            .as_ref()
            .ok_or(TestError("network control lost its network"))?;
        // Step 1: 逐一比对并恢复每个网络接口的 UP/DOWN 状态。
        for interface in interfaces.iter().flatten() {
            let current = checked(context, net.interface_snapshot(interface.id))?;
            if current.up != interface.up {
                checked(context, net.set_interface_up(interface.id, interface.up))?;
            }
        }

        // Step 2: 比对当前默认路由与初始快照，若发生变化则先删除当前默认路由再恢复原始默认路由。
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

    /// 构造用户态运行时、同步管道、命令行参数与 ELF 进程镜像，并发启动 `/bin/network-smoke` 与内核协调线程并等待根进程退出。
    fn network_run(&mut self, context: &mut TestContext<'_>, mode: NetworkMode) -> TestResult {
        // Step 1: 根据测试模式配置进程凭证（`Identity` 模式使用普通用户 1000:2000，其余使用 root）并初始化带控制台的 `UserRuntime`。
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
        // Step 2: 若当前模式需要内核协调，创建 `ready`（用户写、内核非阻塞读）与 `go`（内核写、用户阻塞读）两条管道并安装到用户文件描述符表。
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

        // Step 3: 组装传递给 `/bin/network-smoke` 的命令行参数（子模式名、管道描述符或外部网卡测试端口号、以及 procfs 挂载路径）。
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
        // Step 4: 解析 ELF 镜像并准备用户进程任务与内核协调任务；在发布进程前按需开启目标端口的 SYN 报文拦截。
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
        // Step 5: 正式发布用户进程与内核协调线程投入调度，等待用户根进程退出并回收全部子进程。
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

/// 在给定的超时秒数 `seconds` 内以 1ms 间隔轮询条件 `ready`，超时仍未满足时返回测试错误 `error`。
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

/// 从非阻塞管道 `reader` 中读取用户态 C 程序写入的 12 字节阶段报告（按小端序解析为 `[phase, pid, value]` 三个 `u32`）。
/// 若在读取满 12 字节前遇到管道 EOF、根进程地址空间已销毁或超过 10 秒期限，则返回测试错误。
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

/// 读取一条用户态阶段报告，校验阶段标识字节等于 `expected` 且附加状态码等于 `value`，返回报告中携带的进程/线程 PID。
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

/// 在任务图 `graph` 中查找指定 `pid` 对应的内核执行线程，并轮询等待其调度状态真正进入 `State::Blocked`（睡眠阻塞）。
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

/// 向控制管道 `writer` 写入单字节放行指令 `b"G"`，唤醒正在等待内核协调完成的用户态测试进程。
fn go(writer: &FileRef) -> TestResult {
    if writer.write(b"G") != Ok(1) {
        return Err(TestError("network control pipe write failed"));
    }
    Ok(())
}

/// 内核协调线程主函数：根据测试模式 `mode` 与用户态进程通过管道握手机制配合完成 SYN 拦截放行、发送背压阻塞检测与信号注入。
fn coordinate(
    mode: NetworkMode,
    reader: &FileRef,
    writer: &FileRef,
    graph: &TaskGraph,
    root: &UserThread,
    net: &NetStackRef,
) -> TestResult {
    match mode {
        // Connect 模式：等待用户态发起非阻塞 connect 并报告 EINPROGRESS (114)，确认 SYN 被拦截后释放 SYN 并放行用户态。
        NetworkMode::Connect => {
            phase(reader, root, b'C', 114)?;
            network_until(|| net.ktest_has_held_syn(), "connect SYN was not held", 5)?;
            net.ktest_release_syn()
                .map_err(|_| TestError("connect SYN release failed"))?;
            go(writer)
        }
        // Backpressure 模式：等待用户态写满发送缓冲区进入阻塞，确认内核任务状态为 Blocked 后放行对端读取。
        NetworkMode::Backpressure => {
            let pid = phase(reader, root, b'B', 0)?;
            blocked(graph, pid)?;
            crate::ktest::report::diagnostic(format_args!(
                "network-user writer-pid={} full-send-blocked=1",
                pid
            ));
            go(writer)
        }
        // Signal 模式：依次配合用户态验证阻塞 read 的 EINTR 中断、SA_RESTART 自动重启，以及阻塞 connect 的信号中断与重启。
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
        // UnixPair 模式：等待写进程写满 UNIX 流缓冲区阻塞后放行读进程，最后确认第 4 阶段全部断言完成。
        NetworkMode::UnixPair => {
            let pid = phase(reader, root, b'U', 0)?;
            blocked(graph, pid)?;
            go(writer)?;
            phase(reader, root, b'4', 0)?;
            go(writer)
        }
        // 其余协调模式：等待用户态报告全部阶段（'4'）完成后写入放行字节。
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

/// 解析内核启动命令行（`bootargs`）中的 `ktest-net-tcp=<port>` 与 `ktest-net-udp=<port>` 参数对。
///
/// - 若两个参数成对出现且端口合法，返回 `Ok(Some((tcp_port, udp_port)))` 以启用外部网卡（NIC）测试；
/// - 若两个参数均未出现，返回 `Ok(None)` 以使用本地回环（loopback）网络栈；
/// - 若仅出现其中一个、重复出现或格式非法，返回对应的 `TestError`。
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

/// 将十进制 ASCII 字节切片解析为合法的非零端口号（`1..=65535`）。
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

/// 将无符号整数 `value` 格式化为栈上固定 20 字节缓冲区中的十进制 ASCII 数字，返回 `(缓冲区, 有效数字起始下标)`。
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
