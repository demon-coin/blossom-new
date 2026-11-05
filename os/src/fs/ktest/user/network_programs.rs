// 网络 suite 独占后台网络拥有；include 保持原注册命名空间。
/// 网络业务在整组基线前持有唯一网络 owner，普通文件测试不构造此组合。
struct NetworkSuite {
    resources: UserSuite,
    net_owner: Option<NetStackOwner>,
}

impl Fixture for NetworkSuite {
    /// 成对服务端口选择真实 NIC；没有端口时显式构造 lo。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let ports = network::nic_ports()?;
        let resources = <UserSuite as Fixture>::set_up(context, &())?;
        let net_owner = if ports.is_some() {
            // SAFETY: 框架串行分派 suite，前组已同步停止 NIC；本组是所选设备唯一构造者。
            // 库存来自一次真实 discover，永久强序映射有效，并保留到网络 owner 完整停止之后。
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

    /// 用户与协议拥有结束后先停止网络回调全尾，再停止库存。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        drop(self.net_owner.take());
        <UserSuite as Fixture>::quiesce(&mut self.resources, context)
    }

    /// 未进入 quiesce 的路径也先结束真实网络，再消费库存。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        drop(self.net_owner.take());
        <UserSuite as Fixture>::tear_down(self.resources, context)
    }
}

impl Fixture<NetworkSuite> for UserOwner {
    /// 借同一库存并保活整组真实网络，停止责任仍由 suite owner 承担。
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

    /// 网络恢复在原用户 wait、join 和 reap 之后执行。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::quiesce(self, context)
    }

    /// 保留完整 MM 与任务元数据断言，不重置网络统计。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        <Self as Fixture<UserSuite>>::tear_down(self, context)
    }
}

#[ktest::suite(fixture = UserOwner, suite_resources = NetworkSuite, context = "Thread", tags("user-elf", "user-abi", "user-signal", "fs-path", "fs-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod network_programs {
    use super::*;

    /// 普通 pipe 与 lo TCP 产生固定事件，用户 ioctl 冻结后核旧 OFD 字节独立。
    #[test]
    #[ktest(tags("observation-user", "trace", "network-user"))]
    fn observation_trace(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.observation_mode(context, b"trace")
    }

    /// 普通 ioctl 配置双族 lo，并通过实际 proc 打开验证旧快照和共享游标。
    #[test]
    #[ktest(tags("network-user", "network-control", "fs-proc"))]
    fn network_control(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Control)
    }

    /// 删除并恢复默认路由后沿原 TCP peer 完成真实 NIC 全文业务。
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

    /// 普通 TCP 经实际 NIC 与宿主核验双向全文、短前缀、半关闭和真实地址。
    #[test]
    #[ktest(
        tags("network-user", "network-nic", "network-nic-tcp"),
        resources("net-user")
    )]
    fn network_nic_tcp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::NicTcp)
    }

    /// 同一普通 UDP socket 经实际 NIC 消费三条完整响应并拒绝超 MTU 消息。
    #[test]
    #[ktest(
        tags("network-user", "network-nic", "network-nic-udp"),
        resources("net-user")
    )]
    fn network_nic_udp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::NicUdp)
    }

    /// 普通用户身份创建 socket，并核验共享 OFD 与独立描述符 flags。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_identity(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Identity)
    }

    /// 同一监听 OFD 的别名连续接受已完成握手的连接，每次得到新 OFD。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_listener(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Listener)
    }

    /// 真实 SYN 等待与拒绝分别产生非阻塞连接结果，SO_ERROR 按一次消费核验。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_connect(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Connect)
    }

    /// 普通流读取核验完整字节、PEEK、短前缀与数据后的 FIN 和 EOF。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_stream(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Stream)
    }

    /// 发送空间用尽后观察真实用户阻塞，再由接收及后台 ACK 恢复进度。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_backpressure(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Backpressure)
    }

    /// 真实半关闭保留接收方向，并区分 SIGPIPE 与 MSG_NOSIGNAL。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_half_close(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::HalfClose)
    }

    /// 地址容量、用户复制错误与 fd 满分别核验真实 accept 的领取和发表。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_address(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Address)
    }

    /// 信号打断真实 read 和 connect 等待，并沿原重启与 poll 绝对期限消费结果。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_signal(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Signal)
    }

    /// 同一 socket 身份在原 epoll 中核验触发方式、修改、删除和 fd 复用。
    #[test]
    #[ktest(tags("network-user", "network-tcp"))]
    fn network_epoll(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Epoll)
    }

    /// 同一 UDP OFD 的消息目的、默认 peer、来源与断连接都由普通 C 核验。
    #[test]
    #[ktest(tags("network-user", "network-udp"))]
    fn network_udp_address(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::UdpAddress)
    }

    /// 普通消息入口区分零长消费、短消息丢尾与 PEEK/TRUNC。
    #[test]
    #[ktest(tags("network-user", "network-udp"))]
    fn network_udp_message(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::UdpMessage)
    }

    /// 匿名 UNIX 双向流跨真实环容量，共享 OFD 和末关闭错误仍沿原用户拥有。
    #[test]
    #[ktest(tags("network-user", "network-unix"))]
    fn network_unix_pair(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::UnixPair)
    }

    /// 目标消息头与向量跨段消费核验真实接受前缀及用户故障后的后继。
    #[test]
    #[ktest(tags("network-user", "network-message"))]
    fn network_message_vector(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::MessageVector)
    }

    /// 复用策略、监听冲突、实际 UDP 选择和 TCP child 继承由普通 syscall 核验。
    #[test]
    #[ktest(tags("network-user", "network-options"))]
    fn network_socket_options(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::SocketOptions)
    }

    /// 普通 IPv6 TCP 握手、名称容量、双向全文与半关闭沿真实用户 fork 和 reap。
    #[test]
    #[ktest(tags("network-user", "network-ipv6", "network-tcp"))]
    fn network_ipv6_tcp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Ipv6Tcp)
    }

    /// 两个 ::1 UDP 目的与默认 peer、大 lo 消息和 LP64 向量由普通 syscall 核验。
    #[test]
    #[ktest(tags("network-user", "network-ipv6", "network-udp"))]
    fn network_ipv6_udp(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Ipv6Udp)
    }

    /// 双面监听共用同一 OFD，映射名称、端口冲突及 V6ONLY 隔离由普通连接核验。
    #[test]
    #[ktest(tags("network-user", "network-ipv6"))]
    fn network_ipv6_mapped(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::Ipv6Mapped)
    }

    /// TCP 发送满后恢复，UDP 单消息按预算接纳，缩小不丢两者已接受的数据。
    #[test]
    #[ktest(tags("network-user", "network-buffer"))]
    fn network_buffer_budget(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.network_mode(context, NetworkMode::BufferBudget)
    }

    /// 双族真实连接查询 MSS、Reno 及开关，非 TCP 请求和选项容量保正式 ABI 错误。
    #[test]
    #[ktest(tags("network-user", "network-options"))]
    fn network_protocol_options(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::ProtocolOptions)
    }

    /// 大 lo UDP 保单条消息、全文及来源，分别核两族协议上界与下一字节拒绝。
    #[test]
    #[ktest(tags("network-user", "network-large"))]
    fn network_large_loopback(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.network_mode(context, NetworkMode::LargeLoopback)
    }

    /// 原生 C 核 socket 的真实有限等待、共享选项、child 继承与信号重启差异。
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
