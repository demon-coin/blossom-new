// ============================================================================
// 用户态网络测试套件（`network_programs`）及其套件级资源夹具（`NetworkSuite`）
//
// # 模块定位
// 本文件通过 `include!("user/network_programs.rs")` 内联包含于 `os/src/fs/ktest/user.rs` 尾部，
// 保持原有的 `fs::ktest::user::network_programs` 命名空间。上游由 `ktest` 框架调度运行，
// 下游调用 `UserOwner::network_mode`、`UserOwner::observation_mode` 及 `UserOwner::runtime_socket_timeouts`。
//
// # 核心机制与测试夹具生命周期状态机
// `NetworkSuite` 在整个测试套件启动时构造唯一协议栈实例（`NetStackOwner`），各测试用例通过
// `UserOwner` 借用 `Arc<NetStack>`，套件结束时严格按逆序排空并销毁：
//
// ```text
// [NetworkSuite::set_up]
//    |---> 1. 解析 bootargs 中的 ktest-net-tcp / ktest-net-udp 端口对
//    |---> 2. 初始化 UserSuite（发现块设备、打开 Ext4 根卷、建立基础快照）
//    |---> 3. 构造 NetStackOwner：
//    |        - 有端口对：独占绑定 VirtIO-Net 网卡并配置 10.0.2.15/24 与网关 10.0.2.2
//    |        - 无端口对：构造纯本地回环（loopback）协议栈
//    v
// [每个测试用例：UserOwner::set_up -> test_fn -> UserOwner::quiesce -> UserOwner::tear_down]
//    |---> 克隆 Arc<NetStack> 注入 UserOwner
//    |---> 运行用户态测试程序 (/bin/network-smoke 等) 并等待全部套接字端点排空
//    |---> 恢复网络接口 UP/DOWN 状态与默认路由，校验内存/任务计数无泄漏
//    v
// [NetworkSuite::quiesce / tear_down]
//    |---> 1. 先 Drop NetStackOwner：停止网络收发后台工作任务并等待中断/Work 回调彻底退出
//    +---> 2. 后销毁 UserSuite：同步并释放底层块设备与缓存资源
// ```
//
// # 锁获取顺序与并发契约
// - `ktest` 框架串行执行各个测试套件，保证在同一时刻最多只有一个 `NetStackOwner` 独占底层网卡硬件队列。
// - 套件销毁时必须先释放 `net_owner`（等待所有网络 Work 任务与中断回调彻底退出），随后才能调用 `UserSuite` 的清理逻辑。
// ============================================================================

/// 用户态网络测试套件的共享资源夹具，在基础文件系统环境（`UserSuite`）之上独占持有网络协议栈实例。
struct NetworkSuite {
    /// 用户态测试共享的块设备、Ext4 根卷与内存基线资源。
    resources: UserSuite,
    /// 套件级独占的网络协议栈生命周期守卫；在 `quiesce` 或 `tear_down` 时取出并销毁。
    net_owner: Option<NetStackOwner>,
}

impl Fixture for NetworkSuite {
    /// 初始化用户态网络测试套件资源：若启动参数提供了一对外部测试端口则绑定 VirtIO-Net 网卡并配置静态 IPv4 地址，否则创建纯回环网络栈。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let ports = network::nic_ports()?;
        let resources = <UserSuite as Fixture>::set_up(context, &())?;
        let net_owner = if ports.is_some() {
            // SAFETY:
            // 1. `ktest` 框架按套件串行分派执行，前序网络测试套件已在 `quiesce`/`tear_down` 中同步停止并释放了网卡；
            // 2. `resources.inventory` 由 `UserSuite::set_up` 完成一次性设备发现，其 MMIO 映射在整个 `NetworkSuite` 生命周期内有效，
            //    且 `net_owner` 会在 `resources` 释放前先行 Drop。
            let owner = checked(context, unsafe {
                NetStack::try_new_nic(&resources.inventory)
            })?;
            checked(
                context,
                owner.stack().configure_ethernet(EthernetConfig {
                    address: [10, 0, 2, 15],
                    prefix_len: 24,
                    gateway: Some([10, 0, 2, 2]),
                }),
            )?;
            owner
        } else {
            checked(context, NetStack::try_new_lo())?
        };
        crate::workqueue::testing::wait_services_blocked();
        Ok(Self {
            resources,
            net_owner: Some(net_owner),
        })
    }

    /// 套件正常静默阶段：先销毁 `net_owner` 使所有网络后台工作队列任务与回调彻底执行完毕并退出，再静默底层文件系统资源。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        drop(self.net_owner.take());
        <UserSuite as Fixture>::quiesce(&mut self.resources, context)
    }

    /// 套件最终拆除阶段：确保即便未经过 `quiesce` 也会先停止并释放网络协议栈，再销毁底层设备资源。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        drop(self.net_owner.take());
        <UserSuite as Fixture>::tear_down(self.resources, context)
    }
}

impl Fixture<NetworkSuite> for UserOwner {
    /// 为单个网络测试用例初始化 `UserOwner`，复用套件共享的底层文件系统资源并注入 `NetStack` 强引用。
    fn set_up(context: &mut TestContext<'_>, suite: &NetworkSuite) -> Result<Self, TestError> {
        let mut owner = <Self as Fixture<UserSuite>>::set_up(context, &suite.resources)?;
        owner.net = Some(Arc::clone(
            suite
                .net_owner
                .as_ref()
                .expect("network suite stopped")
                .stack(),
        ));
        Ok(owner)
    }

    /// 单个测试用例静默阶段：委托 `UserSuite` 完成用户进程等待、子进程回收、网络控制状态恢复及文件系统同步。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::quiesce(self, context)
    }

    /// 单个测试用例拆除阶段：委托 `UserSuite` 校验地址空间、文件描述符与内核任务资源已完全释放。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::tear_down(self, context)
    }
}

#[ktest::suite(fixture = UserOwner, suite_resources = NetworkSuite, context = "Thread", tags("user-elf", "user-abi", "user-signal", "fs-path", "fs-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod network_programs {
    use super::*;

    /// 验证用户态进程通过管道与回环 TCP 产生可观测事件后，使用 `ioctl` 冻结跟踪缓冲区时已打开文件描述（OFD）的快照字节保持独立。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：启动可观测性测试程序并传入 `"trace"` 模式参数。
    /// - **When**：在用户态产生固定数量的管道与回环 TCP 事件，通过 `ioctl` 冻结跟踪状态并对比新旧打开文件描述读取到的记录。
    /// - **Then**：冻结后的旧文件描述保持独立快照字节与游标，进程以状态码 0 正常退出。
    #[test]
    #[ktest(tags("observation-user", "trace", "network-user"))]
    fn observation_trace(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.observation_mode(context, b"trace")
    }

    /// 验证用户态通过 `ioctl` 配置 IPv4/IPv6 回环接口地址与启停标志，并通过 `/proc/net` 读取接口与路由快照。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：保存测试前网络接口与路由表快照，在 `/plain` 目录挂载只读 `procfs`。
    /// - **When**：运行 `/bin/network-smoke control`，调用 `ioctl` 修改双栈回环接口配置，并多次打开 `/proc/net` 文件验证快照隔离与 `dup` 共享游标。
    /// - **Then**：所有控制操作与 `procfs` 读取断言通过，测试结束后接口状态与默认路由完整恢复。
    #[test]
    #[ktest(tags("network-user", "network-control", "fs-proc"))]
    fn network_control(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Control)
    }

    /// 验证在外部网卡（NIC）上删除并重新添加默认路由后，TCP 连接仍可与宿主机完成双向全文收发。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：挂载 `procfs` 并获取启动命令行配置的宿主机 TCP/UDP 测试端口。
    /// - **When**：运行 `/bin/network-smoke nic-control`，先删除默认路由验证外网连接按预期失败，再恢复默认路由并与宿主机建立 TCP 连接传输数据。
    /// - **Then**：路由变更与恢复即时生效，TCP 双向数据传输与 `/proc/net` 统计核对全部通过。
    #[test]
    #[ktest(
        tags(
            "network-user",
            "network-control",
            "network-nic",
            "network-nic-tcp",
            "fs-proc"
        ),
        resources("net-user")
    )]
    fn network_nic_control(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::NicControl)
    }

    /// 验证用户态 TCP 套接字通过外部网卡（NIC）与宿主机对端完成连接建立、双向正文传输、短缓冲区前缀读取、半关闭及端点地址查询。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：配置好 VirtIO-Net 网卡静态地址（`10.0.2.15/24`）及宿主机 TCP 服务端口。
    /// - **When**：运行 `/bin/network-smoke nic-tcp` 连接宿主机测试服务，依次执行完整报文收发、短读、`shutdown(SHUT_WR)` 半关闭及 `getsockname`/`getpeername` 检查。
    /// - **Then**：双向字节流完整无误，半关闭后仍能接收对端尾部数据并正常到达 EOF。
    #[test]
    #[ktest(
        tags("network-user", "network-nic", "network-nic-tcp"),
        resources("net-user")
    )]
    fn network_nic_tcp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::NicTcp)
    }

    /// 验证用户态 UDP 套接字通过外部网卡（NIC）与宿主机收发多条数据报，并拒绝超过链路 MTU 的超大报文。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：配置好 VirtIO-Net 网卡及宿主机 UDP 服务端口。
    /// - **When**：运行 `/bin/network-smoke nic-udp`，使用同一个 UDP 套接字连续向宿主机发送请求并接收 3 条完整响应，随后尝试发送超过链路 MTU 的 UDP 数据报。
    /// - **Then**：3 条正常数据报内容与源地址完全匹配，超 MTU 发送调用返回预期错误码（`EMSGSIZE`）。
    #[test]
    #[ktest(
        tags("network-user", "network-nic", "network-nic-udp"),
        resources("net-user")
    )]
    fn network_nic_udp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::NicUdp)
    }

    /// 验证非特权用户（UID 1000 / GID 2000）创建 TCP 套接字，以及复制文件描述符时的共享打开文件描述（OFD）与独立描述符标志（`FD_CLOEXEC`）。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：以 `uid=1000, gid=2000` 凭证启动 `/bin/network-smoke identity`。
    /// - **When**：创建套接字并通过 `dup`/`dup3`/`fcntl` 复制描述符，修改其中一个描述符的 `O_NONBLOCK`（OFD 级）与 `FD_CLOEXEC`（描述符级）标志。
    /// - **Then**：`O_NONBLOCK` 状态在共享同一 OFD 的所有描述符间同步生效，而 `FD_CLOEXEC` 仅在目标描述符上独立生效。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_identity(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Identity)
    }

    /// 验证指向同一个监听套接字打开文件描述（OFD）的多个文件描述符交替调用 `accept` 时，每次均成功接收连接并返回全新的连接 OFD。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：在回环接口上绑定并监听 TCP 端口，通过 `dup` 复制监听套接字描述符。
    /// - **When**：客户端连续发起两次 TCP 连接，服务端分别通过原监听描述符和复制出的别名描述符调用 `accept`。
    /// - **Then**：两次 `accept` 均成功返回已完成三次握手的独立连接套接字，且各自可独立收发数据。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_listener(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Listener)
    }

    /// 验证非阻塞 TCP `connect` 在 SYN 挂起期间返回 `EINPROGRESS`、对未监听端口返回连接拒绝，且 `getsockopt(SO_ERROR)` 具有单次读取并清零语义。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：内核预先在端口 45204 上开启 SYN 报文拦截，并启动协调线程。
    /// - **When**：用户进程对 45204 发起非阻塞 `connect` 观察 `EINPROGRESS`，由内核协调线程释放 SYN 完成握手；随后对未监听端口发起非阻塞 `connect` 触发拒绝并连续两次读取 `SO_ERROR`。
    /// - **Then**：挂起连接在释放 SYN 后变为可写且 `SO_ERROR` 为 0；被拒连接首次读取 `SO_ERROR` 返回 `ECONNREFUSED`，第二次读取返回 0。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_connect(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Connect)
    }

    /// 验证 TCP 字节流套接字的完整读取、`MSG_PEEK` 窥视不消费数据、短缓冲区前缀读取，以及对端关闭后的 FIN/EOF 传递。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：建立一对回环 TCP 连接并由发送端写入固定格式的字节序列后关闭写端。
    /// - **When**：接收端依次使用 `MSG_PEEK` 预览数据、使用短缓冲区读取前缀字节、读取剩余全部字节，最后再次调用 `read`。
    /// - **Then**：`MSG_PEEK` 不推进接收游标，分段读取拼接出的字节流与发送内容严格一致，读尽缓冲后返回 0（EOF）。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_stream(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Stream)
    }

    /// 验证 TCP 发送缓冲区写满后阻塞写进程，并在对端读取消费数据、后台协议栈处理 ACK 窗口更新后自动唤醒写进程继续传输。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：建立回环 TCP 连接，写进程持续写入数据直至填满发送缓冲区并进入睡眠阻塞。
    /// - **When**：内核协调线程确认写进程任务状态已变为 `State::Blocked` 后，通过控制管道通知读进程开始读取数据。
    /// - **Then**：随着读进程消费接收缓冲区并回传窗口更新 ACK，写进程被唤醒并完成剩余全部字节的发送，收发两端数据完全一致。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_backpressure(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Backpressure)
    }

    /// 验证 TCP `shutdown(SHUT_WR)` 半关闭后仍能正常接收对端数据，以及向已关闭写通道发送数据时的 `SIGPIPE` 信号投递与 `MSG_NOSIGNAL` 抑制。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：建立回环 TCP 连接，一端调用 `shutdown(SHUT_WR)` 进行发送方向半关闭。
    /// - **When**：对端读取到 EOF 后继续向半关闭端发送回包；半关闭端分别使用带 `MSG_NOSIGNAL` 和不带 `MSG_NOSIGNAL` 的 `send`/`write` 尝试写入。
    /// - **Then**：半关闭端可完整接收对端发来的数据；带 `MSG_NOSIGNAL` 的写入仅返回 `EPIPE` 而不触发信号，普通写入则触发 `SIGPIPE` 并返回 `EPIPE`。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_half_close(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::HalfClose)
    }

    /// 验证 `accept` 系统调用在地址缓冲区过小、用户空间地址指针无效（`EFAULT`）以及进程文件描述符表已满（`EMFILE`）时的错误处理与连接保留语义。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：在回环 TCP 监听套接字上准备好已完成握手的待接受连接。
    /// - **When**：依次使用截断长度的 `sockaddr` 缓冲区、无效用户指针以及填满描述符表后的状态调用 `accept`，随后释放空闲描述符并传入合法缓冲区再次 `accept`。
    /// - **Then**：小缓冲区截断写入但返回完整地址长度；`EFAULT` 与 `EMFILE` 错误不会丢失或破坏已建立连接，恢复后仍能正常接受该连接并收发数据。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_address(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Address)
    }

    /// 验证阻塞在 TCP `read` 与 `connect` 上的线程被 `SIGUSR1` 信号打断时的 `EINTR` 返回、`SA_RESTART` 自动重启行为以及 `poll` 超时控制。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：启动 `/bin/network-smoke signal` 与内核协调线程，并对指定端口配置 SYN 拦截。
    /// - **When**：
    ///   1. 分别在不带 `SA_RESTART` 和带 `SA_RESTART` 的 `SIGUSR1` 处理器下阻塞于空 TCP `read`，由协调线程确认 `Blocked` 后发送 `SIGUSR1`；
    ///   2. 分别在不带 `SA_RESTART` 和带 `SA_RESTART` 下发起阻塞 `connect`（SYN 被挂起），由协调线程注入 `SIGUSR1` 并随后释放 SYN。
    /// - **Then**：无 `SA_RESTART` 时阻塞调用返回 `EINTR`（`connect` 转入后台继续并可通过 `poll` 等待完成）；有 `SA_RESTART` 时信号处理函数返回后系统调用自动重启并成功完成。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_signal(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Signal)
    }

    /// 验证 TCP 套接字在 `epoll` 实例中的水平触发（LT）、边缘触发（ET）、监听掩码修改（`EPOLL_CTL_MOD`）、移除（`EPOLL_CTL_DEL`）及描述符复用语义。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：创建 `epoll` 实例并将回环 TCP 监听与连接套接字注册到实例中。
    /// - **When**：依次测试水平触发与边缘触发下的读写就绪通知、修改关注事件集、删除描述符以及关闭并复用文件描述符编号。
    /// - **Then**：`epoll_wait` 返回的就绪事件与触发模式严格匹配，描述符关闭后自动从 `epoll` 集合移除且新复用描述符不会产生伪事件。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_epoll(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Epoll)
    }

    /// 验证同一个 UDP 套接字在无连接显式目的地址发送、`connect` 绑定默认对端、对端源地址过滤以及 `AF_UNSPEC` 解除默认对端时的行为。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：在回环接口上创建多个绑定不同端口的 UDP 套接字。
    /// - **When**：使用 `sendto` 指定目标地址发送，随后调用 `connect` 设置默认对端并测试非对端报文过滤与默认 `send`/`write`，最后用 `AF_UNSPEC` 断开连接。
    /// - **Then**：绑定默认对端期间仅接收来自该对端的报文且可直接调用 `send`；断开默认对端后恢复为可接收任意源报文的无连接状态。
    #[test]
    #[ktest(tags("network-user", "network-udp"))]
    fn network_udp_address(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::UdpAddress)
    }

    /// 验证 UDP 数据报套接字的零长度报文收发、短接收缓冲区截断丢弃尾部以及 `MSG_PEEK | MSG_TRUNC` 标志组合语义。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：建立一对回环 UDP 套接字，依次发送 0 字节数据报和定长多字节数据报。
    /// - **When**：接收端先接收 0 字节报文，再使用 `MSG_PEEK | MSG_TRUNC` 传入短缓冲区窥视下一条报文，最后用短缓冲区实际消费该报文。
    /// - **Then**：0 字节报文返回长度 0 且不阻塞；`MSG_PEEK | MSG_TRUNC` 返回数据报的完整原始长度且不移除报文；短缓冲区实际读取返回缓冲区长度并丢弃同报文剩余尾部。
    #[test]
    #[ktest(tags("network-user", "network-udp"))]
    fn network_udp_message(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::UdpMessage)
    }

    /// 验证 `socketpair(AF_UNIX, SOCK_STREAM)` 匿名本地流套接字在跨环形缓冲区容量时的阻塞流控、共享 OFD 以及最后一个写端关闭后的 EOF 与 `EPIPE` 行为。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：通过 `socketpair` 创建一对连通的 `AF_UNIX` 流套接字并复制描述符。
    /// - **When**：写进程持续写入超过内核环形缓冲区容量的数据直至阻塞，由协调线程确认 `Blocked` 后唤醒读进程消费数据；最后依序关闭写端描述符。
    /// - **Then**：跨容量字节流完整无损传输；仅当指向某端 OFD 的全部描述符都关闭后对端才观察到 EOF 或 `EPIPE`。
    #[test]
    #[ktest(tags("network-user", "network-unix"))]
    fn network_unix_pair(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::UnixPair)
    }

    /// 验证 `sendmsg`/`recvmsg` 多段向量（`iovec`）的分散/聚集读写，以及在后续向量段遇到无效用户指针（`EFAULT`）时已传输前缀的处理语义。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：建立套接字连接并构造包含多段缓冲区的 `msghdr` / `iovec` 数组（其中部分测试用例在后继段放入非法用户地址）。
    /// - **When**：调用 `sendmsg` 与 `recvmsg` 执行跨段读写，随后继续在原连接上收发正常报文。
    /// - **Then**：正常多段向量按序拼接与拆分；遇到后继段 `EFAULT` 时正确处理已成功传输的前缀或返回预期错误，且不破坏套接字上后续报文的收发。
    #[test]
    #[ktest(tags("network-user", "network-message"))]
    fn network_message_vector(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::MessageVector)
    }

    /// 验证 `SO_REUSEADDR` 端口复用规则、活跃监听冲突拒绝、UDP 绑定地址匹配优先级以及 TCP 监听套接字选项向 `accept` 子连接的继承。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：在回环接口上创建多个配置不同 `SO_REUSEADDR` 和缓冲区选项的 TCP/UDP 套接字。
    /// - **When**：测试相同端口重复绑定/监听、特定 IP 与通配地址（`INADDR_ANY`）的 UDP 分发选择，以及在监听套接字设置选项后调用 `accept` 检查子套接字选项。
    /// - **Then**：冲突监听返回 `EADDRINUSE`；UDP 报文优先路由至精确匹配地址的套接字；`accept` 生成的子套接字正确继承监听套接字的可继承选项。
    #[test]
    #[ktest(tags("network-user", "network-options"))]
    fn network_socket_options(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::SocketOptions)
    }

    /// 验证 IPv6 回环地址（`::1`）上的 TCP 三次握手、`sockaddr_in6` 端点地址查询、父子进程双向流传输与半关闭回收。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：创建 `AF_INET6` TCP 监听套接字并绑定到 `[::1]:port`，通过 `fork` 创建子进程作为客户端。
    /// - **When**：子进程连接 `[::1]:port`，父子进程核对 `getsockname`/`getpeername` 返回的 `sockaddr_in6` 结构后进行双向全文收发与半关闭，最后父进程 `waitpid` 回收子进程。
    /// - **Then**：IPv6 地址长度与字段完全准确，双向字节流传输与半关闭符合预期，子进程以 0 退出并被干净回收。
    #[test]
    #[ktest(tags("network-user", "network-ipv6", "network-tcp"))]
    fn network_ipv6_tcp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Ipv6Tcp)
    }

    /// 验证 IPv6 回环地址（`::1`）上的 UDP 多目标收发、默认对端连接、跨页大报文传输以及 64 位 `msghdr`/`iovec` 向量接口。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：在 `::1` 上绑定多个 `AF_INET6` UDP 套接字。
    /// - **When**：交替使用 `sendto`/`recvfrom`、`connect` 默认对端以及 `sendmsg`/`recvmsg`（含跨页大负载与多段 `iovec`）在多个 IPv6 端点间收发数据。
    /// - **Then**：所有 IPv6 源/目的地址、报文长度与负载字节均准确无误。
    #[test]
    #[ktest(tags("network-user", "network-ipv6", "network-udp"))]
    fn network_ipv6_udp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Ipv6Udp)
    }

    /// 验证双栈（`IPV6_V6ONLY = 0`）IPv6 监听套接字同时接受 IPv4 与 IPv6 连接、IPv4 映射地址（`::ffff:127.0.0.1`）展示，以及开启 `IPV6_V6ONLY = 1` 时的命名空间隔离。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：创建 `AF_INET6` 套接字，分别测试默认/关闭 `IPV6_V6ONLY` 与开启 `IPV6_V6ONLY` 下的绑定和监听。
    /// - **When**：使用 IPv4 客户端（`127.0.0.1`）和 IPv6 客户端（`::1`）连接双栈监听套接字并检查 `getpeername`；同时测试 `IPV6_V6ONLY` 开启时同端口独立绑定 `AF_INET` 套接字。
    /// - **Then**：双栈套接字将 IPv4 客户端地址表示为 `::ffff:127.0.0.1` 并正常通信；开启 `IPV6_V6ONLY` 后仅接收 IPv6 连接且允许同端口绑定独立的 IPv4 套接字。
    #[test]
    #[ktest(tags("network-user", "network-ipv6"))]
    fn network_ipv6_mapped(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Ipv6Mapped)
    }

    /// 验证 `SO_SNDBUF` 与 `SO_RCVBUF` 缓冲区配额生效：TCP 发送达上限后阻塞并在消费后恢复，UDP 按接收配额接纳单条报文，且调小配额不会丢弃已入队数据。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：通过 `setsockopt` 为 TCP 和 UDP 套接字设置自定义发送/接收缓冲区字节预算。
    /// - **When**：向套接字写入数据直至触及配额上限，随后调小 `SO_RCVBUF`/`SO_SNDBUF` 并读取已入队的数据。
    /// - **Then**：超出预算的非阻塞写入返回 `EAGAIN`（或阻塞等待消费后恢复），已在队列中的数据在调小配额后仍可完整读出。
    #[test]
    #[ktest(tags("network-user", "network-buffer"))]
    fn network_buffer_budget(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::BufferBudget)
    }

    /// 验证 IPv4 与 IPv6 TCP 连接上查询/设置 `TCP_MAXSEG`、`TCP_CONGESTION`（`"reno"`）和 `TCP_NODELAY` 选项，以及非 TCP 套接字或非法缓冲区长度的错误码。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：建立 IPv4 和 IPv6 回环 TCP 连接，并额外创建一个 UDP 套接字用于负向验证。
    /// - **When**：在 TCP 连接上调用 `getsockopt`/`setsockopt` 操作 `TCP_MAXSEG`、`TCP_CONGESTION` 与 `TCP_NODELAY`，并在 UDP 套接字或过短/非法长度参数下调用相同选项。
    /// - **Then**：TCP 连接返回有效的正数 MSS、`"reno"` 拥塞算法名及正确的 `TCP_NODELAY` 开关值；非 TCP 套接字或非法长度参数返回标准 POSIX 错误码（如 `EOPNOTSUPP`/`EINVAL`）。
    #[test]
    #[ktest(tags("network-user", "network-options"))]
    fn network_protocol_options(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::ProtocolOptions)
    }

    /// 验证回环接口上 IPv4（最大 65507 字节）与 IPv6（最大 65527 字节）超大 UDP 数据报的原子收发，以及超出协议上限 1 字节时返回 `EMSGSIZE`。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：在回环接口上创建调大缓冲区配额的 IPv4 与 IPv6 UDP 套接字对。
    /// - **When**：分别发送恰好等于 IPv4 UDP 负载上限（65507B）和 IPv6 UDP 负载上限（65527B）的数据报，随后尝试发送 65508B（IPv4）与 65528B（IPv6）数据报。
    /// - **Then**：上限尺寸的数据报保持单条消息边界完整收发且源地址正确；超出上限 1 字节的发送请求立即返回 `EMSGSIZE`。
    #[test]
    #[ktest(tags("network-user", "network-large"))]
    fn network_large_loopback(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::LargeLoopback)
    }

    /// 验证用户态套接字接收/发送超时选项（`SO_RCVTIMEO` / `SO_SNDTIMEO`）的限时阻塞等待、跨 `dup` 共享、跨 `fork` 子进程继承以及信号中断行为。
    ///
    /// # 测试场景（Given-When-Then）
    /// - **Given**：启动 `runtime_socket_timeouts` C 测试程序。
    /// - **When**：设置 `SO_RCVTIMEO` 与 `SO_SNDTIMEO`，测量空读/满写阻塞到期返回 `EAGAIN` 的实际耗时，并验证 `dup` 别名与 `fork` 子进程继承该超时配置及信号打断返回 `EINTR`。
    /// - **Then**：超时等待在设定时间窗口内返回 `EAGAIN`，共享与继承关系及信号中断语义全部符合标准规范。
    #[test]
    #[ktest(
        tags("runtime-socket-timeouts", "user-libc", "user-network"),
        timeout_ms = 120000
    )]
    fn runtime_socket_timeouts(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.runtime_socket_timeouts(context)
    }
}
