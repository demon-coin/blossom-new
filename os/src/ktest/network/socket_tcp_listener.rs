// TCP 监听套接字（Listener）、被动握手连接队列（Backlog）、`accept` 与 `EPOLLET` 边沿通知集成测试套件。
//
// # 模块职责与架构定位
// 本套件由 `os/src/ktest/network.rs` 通过 `include!` 内联引入，负责在完整的后台网络栈（`StackOwner`）环境中
// 验证 TCP 监听端点的子连接派生、半连接成熟、边沿触发（Edge-Triggered）通知语义以及大规模监听/积压队列增长。
// 上游由 `ktest` 框架在 `Thread` 上下文中驱动，下游调用 `SocketFile`、`TcpEndpoint`、`NetTables` 与 `EpollFile`。
//
// # 核心机制与被动握手/就绪队列状态流
// 监听端点（`State::Listen`）在收到客户端 `SYN` 时在全局连接表与自身半连接集合中派生被动子端点（`State::SynReceived`）；
// 当第三个握手确认报文（纯 `ACK` 或携带有效确认号的 `FIN+ACK`）到达后，子端点晋升至就绪队列（Ready Queue）并唤醒监听端等待队列：
//
// ```text
//   客户端 (Client)                监听端点 (Listener: Listen)           Epoll (EPOLLET)
// +-----------------+            +-----------------------------+       +-----------------+
// | connect() 发 SYN| ---------> | 创建被动子端点 (SynReceived) |       |                 |
// |                 | <--SYN+ACK-| 注册至全局 TCP 四元组表     |       |                 |
// | 发 ACK / FIN+ACK| ---------> | 子端点成熟 (Established /   | ----> | 触发一次 POLLIN |
// |                 |            | CloseWait)，移入 Ready 队列 |       | 边沿通知        |
// +-----------------+            +--------------+--------------+       +-----------------+
//                                               |
//                                  accept() 弹出子连接包装为新 SocketFile
//                                  (已领取子连接后续关闭退役不会再次触发监听端边沿事件)
// ```
//
// # 锁获取顺序与并发契约
// - 所有测试用例均在普通可睡眠任务上下文（`context = "Thread"`）中运行，使用 `StackOwner` 夹具管理后台回环网络栈。
// - 测试代码通过 `SocketFile` 或 `TcpEndpoint::lock()` 访问协议状态；在调用 `observe` 或 `socket_retired` 休眠等待前
//   必须释放所有端点锁，交由后台工作队列并发推进握手、数据传输与 `FIN`/`TIME_WAIT` 回收。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-socket", "network-tcp-listener"), timeout_ms = 45000)]
pub(crate) mod socket_tcp_listener {
    use ::ktest::{TestContext, TestError, TestResult};
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use smoltcp::socket::tcp::State;

    use super::owner::StackOwner;
    use super::socket_helpers::{
        socket_epoll_collect, socket_listener, socket_read_exact, socket_write_all,
    };
    use super::support::{observe, operation, socket_retired, SERVER};
    use crate::fs::epoll::{EpollEvent, EpollFile, EPOLLET, EPOLL_CTL_ADD};
    use crate::fs::{FileIoContext, FileLike, FileRef, FsError, PollMask, Pollable, O_NONBLOCK};
    use crate::fs::{ReadBuffer, WriteBuffer};
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::net::address::{AddressFamily, InetEndpoint, SocketAddress};
    use crate::net::socket::{SocketFile, SHUT_WR};
    use crate::net::table::TcpTuple;
    use crate::net::IoProgress;
    use crate::{time, timer};

    /// 验证已 `accept` 领取的子连接关闭退役不会误重发监听端 `EPOLLET` 事件，且后续新握手完成仍能正常触发边沿通知。
    ///
    /// - **测试目标**：确认当监听套接字处于仍有未领取就绪连接（连接 B）的水平可读状态但 `EPOLLET` 边沿已被消费时，
    ///   先前已被 `accept` 领走的子连接（连接 A）执行双向 `FIN` 关闭与端点销毁不会重新唤醒监听端 `EPOLLET`；
    ///   同时确认新的客户端（连接 C）完成握手入队时，能够再次触发监听端的边沿事件。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动监听套接字，建立连接 A 并立即 `accept` 取出 `server_a`，随后将监听套接字以 `POLLIN | EPOLLET` 注册到 `EpollFile`。
    ///   2. 建立连接 B 使监听队列变为可读，调用 `socket_epoll_collect` 消费一次边沿事件，并再次扫描确认事件已被排空（返回 0）。
    ///   3. 在不领取连接 B 的情况下，对已领取的连接 A 执行双向 `SHUT_WR`、丢弃 `server_a` 并等待其底层 `TcpEndpoint` 彻底销毁，再次扫描 `epoll`。
    ///   4. 建立连接 C，等待并收集新触发的 `epoll` 事件，随后依次 `accept` 取出连接 B 与连接 C，校验指针独立性并关闭回收全部资源。
    /// - **预期结果**：连接 A 退役后 `epoll` 扫描结果保持为 0（不产生虚假边沿唤醒）；连接 C 握手完成后准确投递 1 个事件；最终所有文件与端点弱引用全部清空且存活端点数归零。
    #[test]
    fn accepted_child_retirement_does_not_rearm_listener_edge(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建监听套接字，建立首个客户端连接 A 并立即 accept 领取，记录各对象弱引用以便后续验证析构。
        let local = InetEndpoint {
            port: 45650,
            ..SERVER
        };
        let listener = socket_listener(context, &fixture.net, local.port)?;
        let listener_weak = Arc::downgrade(listener.tcp_endpoint().unwrap());
        let listener_file_weak = Arc::downgrade(&listener);
        let listener_watch_weak = Arc::downgrade(
            listener
                .watch_head()
                .ok_or(TestError("native listener has no real watch head"))?,
        );
        let client_a = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        operation(
            context,
            client_a.connect(SocketAddress::Inet(local), FileIoContext::Kernel),
        )?;
        let server_a = operation(context, listener.accept(0, 0, 0, FileIoContext::Kernel))?;
        context.expect_eq(Arc::ptr_eq(&listener, &server_a), false);
        context.expect_eq(
            Arc::ptr_eq(
                listener.tcp_endpoint().unwrap(),
                server_a.tcp_endpoint().unwrap(),
            ),
            false,
        );
        context.expect_eq(
            Arc::ptr_eq(
                client_a.tcp_endpoint().unwrap(),
                server_a.tcp_endpoint().unwrap(),
            ),
            false,
        );
        context.expect_eq(listener.tcp_endpoint().unwrap().state(), State::Listen);
        let server_a_weak = Arc::downgrade(server_a.tcp_endpoint().unwrap());
        let server_a_file_weak = Arc::downgrade(&server_a);
        let client_a_weak = Arc::downgrade(client_a.tcp_endpoint().unwrap());

        // Step 2: 创建 Epoll 实例并将监听套接字以边沿触发模式（POLLIN | EPOLLET）加入监控。
        let epoll = operation(context, EpollFile::try_new(0))?;
        let epoll_weak = Arc::downgrade(&epoll);
        let expected = EpollEvent {
            events: PollMask::POLLIN.0,
            data: 0xa1b2_c3d4_e5f6_0758,
        };
        {
            let owner: FileRef = epoll.clone();
            let target: FileRef = listener.clone();
            operation(
                context,
                epoll.control(
                    &owner,
                    EPOLL_CTL_ADD,
                    &target,
                    0,
                    Some(EpollEvent {
                        events: expected.events | EPOLLET,
                        data: expected.data,
                    }),
                ),
            )?;
        }

        // Step 3: 发起第二个连接 B（暂不 accept），验证监听端可读并消费掉本次产生的单次 EPOLLET 边沿事件。
        let client_b = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        operation(
            context,
            client_b.connect(SocketAddress::Inet(local), FileIoContext::Kernel),
        )?;
        let readable = PollMask::POLLIN | PollMask::POLLRDNORM;
        observe(
            || listener.poll(None) == Ok(readable) && epoll.poll(None) == Ok(readable),
            "second handshake did not make listener and real epoll readable",
        )?;
        let peer_b = operation(context, client_b.tcp_endpoint().unwrap().local_address())?;
        let server_b_weak = Arc::downgrade(
            &fixture
                .net
                .tables()
                .lookup_tcp(TcpTuple {
                    local,
                    peer: peer_b,
                })
                .ok_or(TestError("ready B is missing from actual TCP tuple table"))?,
        );
        let client_b_weak = Arc::downgrade(client_b.tcp_endpoint().unwrap());
        let mut events = [EpollEvent::default(); 1];
        let first = operation(context, socket_epoll_collect(&epoll, &mut events))?;
        context.expect_eq(first, 1);
        context.expect_eq(events[0], expected);
        if first != 1 || events[0] != expected {
            return Err(TestError(
                "ready B did not deliver exactly one listener IN/data event",
            ));
        }
        let second = operation(context, socket_epoll_collect(&epoll, &mut events))?;
        context.expect_eq(second, 0);
        if second != 0 {
            return Err(TestError(
                "listener edge was not consumed before accepted A retirement",
            ));
        }

        // Step 4: 对已领取的连接 A 完成双向优雅关闭并销毁服务端文件，等待其底层端点异步回收（含定时器/Work 清理与父子表解绑）。
        operation(context, client_a.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, server_a.read(&mut [0u8; 1]))?, 0);
        operation(context, server_a.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, client_a.read(&mut [0u8; 1]))?, 0);
        drop(server_a);
        context.expect_eq(server_a_file_weak.upgrade().is_none(), true);
        observe(
            || server_a_weak.upgrade().is_none(),
            "accepted A did not finish normal FIN/EOF endpoint retirement",
        )?;
        // Step 5: 验证尽管监听队列中连接 B 仍使 poll 保持可读，连接 A 的退役不会重新触发监听端的 EPOLLET 事件。
        context.expect_eq(listener.tcp_endpoint().unwrap().state(), State::Listen);
        context.expect_eq(operation(context, listener.poll(None))?, readable);
        context.expect_eq(
            server_b_weak.upgrade().map(|endpoint| endpoint.state()),
            Some(State::Established),
        );
        let third = operation(context, socket_epoll_collect(&epoll, &mut events))?;
        context.diagnostic(format_args!(
            "network-socket accepted_a_retired={} listener_state={:?} listener_mask={:?} retirement_scan_count={} retirement_scan_event={:?}",
            server_a_weak.upgrade().is_none(), listener.tcp_endpoint().unwrap().state(),
            operation(context, listener.poll(None))?, third,
            if third == 0 { None } else { Some(events[0]) }
        ));
        context.expect_eq(third, 0);
        if third != 0 {
            return Err(TestError(
                "accepted child retirement rearmed listener ET without a new ready child",
            ));
        }

        // Step 6: 发起第三个连接 C，验证新连接进入就绪队列能够重新触发一次监听端 EPOLLET 事件。
        let client_c = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        operation(
            context,
            client_c.connect(SocketAddress::Inet(local), FileIoContext::Kernel),
        )?;
        observe(
            || epoll.poll(None) == Ok(readable),
            "new ready C did not notify the consumed listener edge",
        )?;
        let peer_c = operation(context, client_c.tcp_endpoint().unwrap().local_address())?;
        let server_c_weak = Arc::downgrade(
            &fixture
                .net
                .tables()
                .lookup_tcp(TcpTuple {
                    local,
                    peer: peer_c,
                })
                .ok_or(TestError("ready C is missing from actual TCP tuple table"))?,
        );
        let client_c_weak = Arc::downgrade(client_c.tcp_endpoint().unwrap());
        let fourth = operation(context, socket_epoll_collect(&epoll, &mut events))?;
        context.expect_eq(fourth, 1);
        context.expect_eq(events[0], expected);
        if fourth != 1 || events[0] != expected {
            return Err(TestError(
                "new ready C did not deliver exactly one listener IN/data event",
            ));
        }
        context.expect_eq(
            operation(context, socket_epoll_collect(&epoll, &mut events))?,
            0,
        );

        // Step 7: 依次 accept 取出连接 B 与连接 C，核对底层端点身份与监听端队列排空状态，最后优雅关闭并验证全部资源归零。
        let server_b = operation(context, listener.accept(0, 0, 0, FileIoContext::Kernel))?;
        let server_c = operation(context, listener.accept(0, 0, 0, FileIoContext::Kernel))?;
        context.expect_eq(
            server_b_weak
                .upgrade()
                .is_some_and(|endpoint| Arc::ptr_eq(&endpoint, server_b.tcp_endpoint().unwrap())),
            true,
        );
        context.expect_eq(
            server_c_weak
                .upgrade()
                .is_some_and(|endpoint| Arc::ptr_eq(&endpoint, server_c.tcp_endpoint().unwrap())),
            true,
        );
        context.expect_eq(Arc::ptr_eq(&listener, &server_b), false);
        context.expect_eq(Arc::ptr_eq(&listener, &server_c), false);
        context.expect_eq(Arc::ptr_eq(&server_b, &server_c), false);
        context.expect_eq(
            Arc::ptr_eq(
                server_b.tcp_endpoint().unwrap(),
                server_c.tcp_endpoint().unwrap(),
            ),
            false,
        );
        context.expect_eq(
            Arc::ptr_eq(
                server_b.tcp_endpoint().unwrap(),
                client_b.tcp_endpoint().unwrap(),
            ),
            false,
        );
        context.expect_eq(
            Arc::ptr_eq(
                server_c.tcp_endpoint().unwrap(),
                client_c.tcp_endpoint().unwrap(),
            ),
            false,
        );
        context.expect_eq(listener.tcp_endpoint().unwrap().state(), State::Listen);
        context.expect_eq(
            operation(context, listener.poll(None))?,
            PollMask::default(),
        );

        for (client, server) in [(&client_b, &server_b), (&client_c, &server_c)] {
            operation(context, client.shutdown(SHUT_WR))?;
            context.expect_eq(operation(context, server.read(&mut [0u8; 1]))?, 0);
            operation(context, server.shutdown(SHUT_WR))?;
            context.expect_eq(operation(context, client.read(&mut [0u8; 1]))?, 0);
        }
        drop((
            epoll, listener, client_a, client_b, client_c, server_b, server_c,
        ));
        context.expect_eq(epoll_weak.upgrade().is_none(), true);
        context.expect_eq(listener_file_weak.upgrade().is_none(), true);
        context.expect_eq(listener_watch_weak.upgrade().is_none(), true);
        socket_retired(&fixture.net)?;
        for endpoint in [
            &listener_weak,
            &server_a_weak,
            &client_a_weak,
            &server_b_weak,
            &client_b_weak,
            &server_c_weak,
            &client_c_weak,
        ] {
            context.expect_eq(endpoint.upgrade().is_none(), true);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        drop((
            epoll_weak,
            listener_file_weak,
            listener_watch_weak,
            server_a_file_weak,
            listener_weak,
            server_a_weak,
            client_a_weak,
            server_b_weak,
            client_b_weak,
            server_c_weak,
            client_c_weak,
        ));
        Ok(())
    }

    /// 验证三次握手第三步纯 `ACK` 丢失后，客户端直接发送 `FIN+ACK` 能完成被动握手并允许非阻塞 `accept` 正常接收该半关闭连接。
    ///
    /// - **测试目标**：确认当被动子端点停留在 `State::SynReceived` 时，若客户端未发应用数据而直接调用 `shutdown(SHUT_WR)` 发送 `FIN+ACK`，
    ///   该报文携带的确认号能使被动子端点同时完成握手成熟并转换至 `State::CloseWait`，非阻塞 `accept` 能立即返回该子连接且不影响反向（服务端到客户端）数据流发送。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建非阻塞监听套接字与绑定固定端口的客户端套接字，配置包控制器丢弃客户端发往服务端的三次握手纯 `ACK`。
    ///   2. 客户端发起 `connect`，等待握手 `ACK` 被丢弃；此时客户端已处于 `Established`，而服务端连接表中的被动子端点停留在 `SynReceived`。
    ///   3. 客户端立即执行 `shutdown(SHUT_WR)` 发送首个后续报文（`FIN+ACK`），等待被动子端点进入 `CloseWait` 后调用非阻塞 `accept`。
    ///   4. 验证服务端读取立即返回 `0`（EOF），随后由服务端向客户端反向写入 `"accepted after FIN"` 并完成双向关闭与端点回收。
    /// - **预期结果**：非阻塞 `accept` 成功返回同一个被动子端点（而非返回 `WouldBlock`），服务端可正常发送反向字节流并被客户端完整接收，关闭后所有端点正常销毁。
    #[test]
    fn passive_fin_ack_matures_before_accept(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 准备非阻塞监听套接字与客户端，开启故障注入以精准丢弃三次握手的第三步纯 ACK 报文。
        let local = InetEndpoint {
            port: 45640,
            ..SERVER
        };
        let peer = InetEndpoint {
            port: 45641,
            ..SERVER
        };
        let listener = socket_listener(context, &fixture.net, local.port)?;
        operation(context, listener.set_status_flags(O_NONBLOCK))?;
        let client = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        operation(context, client.bind(SocketAddress::Inet(peer)))?;
        operation(
            context,
            fixture.net.ktest_drop_handshake_ack(peer.port, local.port),
        )?;
        operation(
            context,
            client.connect(SocketAddress::Inet(local), FileIoContext::Kernel),
        )?;
        observe(
            || fixture.net.ktest_handshake_ack_dropped(),
            "selected handshake ACK was not dropped",
        )?;
        let child = fixture
            .net
            .tables()
            .lookup_tcp(TcpTuple { local, peer })
            .ok_or(TestError(
                "passive handshake child is missing from actual TCP tuple table",
            ))?;

        let initial_state = child.state();
        context.diagnostic(format_args!(
            "network-socket handshake_ack_dropped={} child_id={} initial_state={:?} listener_state={:?}",
            fixture.net.ktest_handshake_ack_dropped(), child.id(), initial_state, listener.tcp_endpoint().unwrap().state()
        ));
        if initial_state != State::SynReceived {
            return Err(TestError(
                "dropped handshake ACK did not leave actual child in SYN-RECEIVED",
            ));
        }
        context.expect_eq(client.tcp_endpoint().unwrap().state(), State::Established);
        context.expect_eq(listener.tcp_endpoint().unwrap().state(), State::Listen);

        // Step 2: 客户端不发送任何应用载荷（否则数据包携带的 ACK 会提前完成握手），直接半关闭写端发送 FIN+ACK，使子端点从 SynReceived 一步转入 CloseWait。
        operation(context, client.shutdown(SHUT_WR))?;
        observe(
            || child.state() == State::CloseWait,
            "passive child did not process actual FIN into CLOSE-WAIT",
        )?;
        let fin_state = child.state();
        // Step 3: 执行非阻塞 accept，验证已进入 CloseWait 的被动子端点已被放入就绪队列并能成功交付。
        let accepted = listener.accept(0, 0, 0, FileIoContext::Kernel);
        let server = match accepted {
            Ok(server) => {
                context.diagnostic(format_args!(
                    "network-socket handshake_ack_dropped={} child_id={} fin_state={:?} accept_result=Ok accepted_child_id={}",
                    fixture.net.ktest_handshake_ack_dropped(), child.id(), fin_state, server.tcp_endpoint().unwrap().id()
                ));
                server
            }
            Err(error) => {
                context.diagnostic(format_args!(
                    "network-socket handshake_ack_dropped={} child_id={} fin_state={:?} accept_result=Err({:?}) listener_state={:?}",
                    fixture.net.ktest_handshake_ack_dropped(), child.id(), fin_state, error, listener.tcp_endpoint().unwrap().state()
                ));
                return Err(TestError(match error {
                    FsError::WouldBlock => {
                        "passive FIN completed handshake but nonblocking accept returned WouldBlock"
                    }
                    _ => "passive FIN nonblocking accept failed",
                }));
            }
        };
        context.expect_eq(Arc::ptr_eq(server.tcp_endpoint().unwrap(), &child), true);
        context.expect_eq(Arc::ptr_eq(&listener, &server), false);
        context.expect_eq(listener.tcp_endpoint().unwrap().state(), State::Listen);
        context.expect_eq(operation(context, server.read(&mut [0u8; 1]))?, 0);
        context.expect_eq(server.tcp_endpoint().unwrap().state(), State::CloseWait);

        // Step 4: 验证 CloseWait 状态下的子连接仍保留完整的服务端至客户端发送通道，随后完成反向 FIN 关闭与资源回收。
        socket_write_all(&server, b"accepted after FIN")?;
        let mut output = [0u8; 18];
        socket_read_exact(&client, &mut output)?;
        context.expect_eq(&output, b"accepted after FIN");
        operation(context, server.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, client.read(&mut [0u8; 1]))?, 0);
        context.expect_eq(listener.tcp_endpoint().unwrap().state(), State::Listen);

        let child_weak = Arc::downgrade(&child);
        let client_weak = Arc::downgrade(client.tcp_endpoint().unwrap());
        let server_file_weak = Arc::downgrade(&server);
        let client_file_weak = Arc::downgrade(&client);
        operation(context, fixture.net.ktest_clear_packet_control())?;
        drop((child, client, server, listener));
        context.expect_eq(client_file_weak.upgrade().is_none(), true);
        context.expect_eq(server_file_weak.upgrade().is_none(), true);
        socket_retired(&fixture.net)?;
        context.expect_eq(child_weak.upgrade().is_none(), true);
        context.expect_eq(client_weak.upgrade().is_none(), true);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        drop((child_weak, client_weak, server_file_weak, client_file_weak));
        Ok(())
    }

    /// 验证监听索引表支持突破初始容量上限（129 个并发监听端口）动态扩容，且关闭回收后原端口可在同一网络栈上重新绑定。
    ///
    /// - **测试目标**：确认 `NetTables` 的监听端口映射表不受固定小配额限制，能够容纳 129 个独立监听端点；
    ///   确认所有监听端点关闭并完成异步槽位回收后，监听索引项被彻底清除，原端口范围可再次绑定。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 循环创建 129 个 `TcpEndpoint`，依次绑定 `46000..46129` 端口并调用 `listen(1)`，逐个核对监听表查找结果。
    ///   2. 关闭全部 129 个监听端点并释放强引用，调用 `socket_retired` 等待后台完成端点回收。
    ///   3. 再次遍历 `46000..46129` 端口，先断言监听表查找为空，再新建端点重新绑定同一端口并关闭回收。
    /// - **预期结果**：129 个并发监听端点全部注册成功，关闭后监听表无残留项，二次重绑定全部返回预期本地地址且无资源泄漏。
    #[test]
    #[ktest(tags("network-growth"))]
    fn listener_indices_grow_and_rebind_without_endpoint_quota(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let mut listeners = Vec::new();
        listeners
            .try_reserve_exact(129)
            .map_err(|_| TestError("listener growth observers OOM"))?;
        // Step 1: 连续创建并监听 129 个不同端口，验证监听索引表能够动态扩容并准确命中每个端点。
        for ordinal in 0..129 {
            let endpoint = operation(
                context,
                fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
            )?;
            let local = InetEndpoint {
                port: 46000 + ordinal,
                ..SERVER
            };
            operation(context, endpoint.bind(local))?;
            operation(context, endpoint.listen(1))?;
            context.expect_eq(
                fixture
                    .net
                    .tables()
                    .lookup_listener(local)
                    .is_some_and(|found| Arc::ptr_eq(&found, &endpoint)),
                true,
            );
            listeners.push(endpoint);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), 129);

        // Step 2: 关闭所有监听端点并等待后台回收全部端点槽位，随后验证全部端口均可干净地重新绑定。
        for endpoint in &listeners {
            endpoint.close();
        }
        drop(listeners);
        socket_retired(&fixture.net)?;

        for ordinal in 0..129 {
            let local = InetEndpoint {
                port: 46000 + ordinal,
                ..SERVER
            };
            context.expect_eq(fixture.net.tables().lookup_listener(local).is_none(), true);
            let endpoint = operation(
                context,
                fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
            )?;
            context.expect_eq(operation(context, endpoint.bind(local))?, local);
            endpoint.close();
        }
        socket_retired(&fixture.net)?;
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }

    /// 验证监听套接字能接纳 129 个小环形缓冲区并发连接，在内存耗尽（OOM）下重复调用 `listen` 调整上限不丢失已就绪连接，且全量连接可正常传输与走完 `FIN`/`TIME_WAIT`。
    ///
    /// - **测试目标**：确认监听端的积压队列（Backlog）独立管理已握手成功的子连接（共 1 个监听端 + 129 对客户端/服务端子端点 = 259 个活跃端点）；
    ///   确认在堆内存耗尽条件下重复调用 `listen`（放大至 `usize::MAX`、缩减至 `0`、恢复为 `4096`）仅原地更新队列上限而不触发堆分配或丢弃已就绪子连接；
    ///   确认全部 129 条连接在被领取后均能完成数据收发与标准的 `FIN` ➔ `CloseWait` ➔ `TimeWait` 状态迁移。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建监听套接字（`listen(4096)`），循环创建 129 个 256 字节小缓冲区的客户端连接并等待全部握手进入 `Established`。
    ///   2. 冲刷工作队列后用 `HeapPressure` 占满堆内存，连续调用 `listen(usize::MAX)`、`listen(0)`、`listen(4096)`，随后释放内存压力。
    ///   3. 依次从监听端领取全部 129 个服务端连接，验证领取完毕后监听端 `poll` 恢复为空掩码。
    ///   4. 遍历每一对连接发送 97 字节非周期规律数据并核对接收内容，随后依次关闭客户端与服务端，观察服务端进入 `CloseWait`、客户端进入 `TimeWait`。
    ///   5. 关闭监听端并释放所有句柄，等待全部 259 个端点（含 `TIME_WAIT` 超时）彻底回收。
    /// - **预期结果**：OOM 期间三次重复 `listen` 全部返回 `Ok(())`，129 个积压连接无一丢失、字节流逐字节一致，且最终所有端点槽位正常归零。
    #[test]
    #[ktest(tags("network-growth", "network-backlog"))]
    fn backlog_accepted_and_time_wait_grow_with_small_native_rings(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let local = InetEndpoint {
            port: 46200,
            ..SERVER
        };
        let listener = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        operation(context, listener.bind(local))?;
        operation(context, listener.listen(4096))?;
        let mut clients = Vec::new();
        let mut servers = Vec::new();
        clients
            .try_reserve_exact(129)
            .map_err(|_| TestError("TCP growth client observers OOM"))?;
        servers
            .try_reserve_exact(129)
            .map_err(|_| TestError("TCP growth accepted observers OOM"))?;
        // Step 1: 建立 129 个并发客户端握手连接并积压在监听队列中，验证活跃端点总数达到 259（1 监听 + 129 客户端 + 129 被动子端点）。
        for _ in 0..129 {
            let client = operation(
                context,
                fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
            )?;
            operation(context, client.connect(local))?;
            observe(
                || client.state() == State::Established,
                "growth client did not complete real handshake",
            )?;
            clients.push(client);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), 259);

        // Step 2: 注入堆内存耗尽压力，验证在已处于监听状态的端点上重复调用 listen 修改积压上限无需新堆分配且不会丢弃已有就绪连接。
        fixture.net.work_queue().flush();
        let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let evidence = pressure.exhaust();
        let repeated = listener.listen(usize::MAX);
        let shrunk = listener.listen(0);
        let restored = listener.listen(4096);
        drop(pressure);
        context.expect_eq(evidence, (true, true));
        context.expect_eq(repeated, Ok(()));
        context.expect_eq(shrunk, Ok(()));
        context.expect_eq(restored, Ok(()));
        if evidence != (true, true) || repeated.is_err() || shrunk.is_err() || restored.is_err() {
            return Err(TestError("repeated listen allocated or discarded backlog"));
        }

        // Step 3: 逐个 accept 取出全部 129 个已就绪的服务端连接，确认队列排空后监听套接字不再报告可读。
        for client in &clients {
            servers.push(super::stack_helpers::accepted(context, &listener, client)?);
        }
        context.expect_eq(
            operation(context, listener.poll(None))?,
            PollMask::default(),
        );

        // Step 4: 对每一条连接验证数据完整收发，并驱动主动关闭方进入 TimeWait、被动关闭方经历 CloseWait。
        for (ordinal, (client, server)) in clients.iter().zip(&servers).enumerate() {
            let input: [u8; 97] =
                ::core::array::from_fn(|offset| super::support::pattern(ordinal * 97 + offset));
            context.expect_eq(
                operation(
                    context,
                    client.lock().send(&mut WriteBuffer::kernel(&input)),
                )?,
                IoProgress::Ready(input.len()),
            );
            let mut output = [0u8; 97];
            let mut copied = 0;
            let end = time::now()
                .checked_add(time::Duration::from_secs(5))
                .unwrap();
            while copied < output.len() {
                if let IoProgress::Ready(count) = operation(
                    context,
                    server
                        .lock()
                        .receive(&mut ReadBuffer::kernel(&mut output[copied..]), false),
                )? {
                    copied += count;
                }
                if end <= time::now() {
                    return Err(TestError("growth accepted stream full delivery deadline"));
                }
                timer::sleep(time::Duration::from_millis(1));
            }
            context.expect_eq(output, input);

            client.close();
            observe(
                || server.state() == State::CloseWait,
                "growth child did not receive real FIN",
            )?;
            context.expect_eq(
                operation(
                    context,
                    server
                        .lock()
                        .receive(&mut ReadBuffer::kernel(&mut [0; 1]), false),
                )?,
                IoProgress::Eof,
            );
            server.close();
            observe(
                || client.state() == State::TimeWait,
                "growth client did not enter actual TIME_WAIT",
            )?;
        }

        // Step 5: 释放全部端点引用，等待后台定时器和工作队列推进完 TIME_WAIT 并回收全部端点槽位。
        let first = Arc::downgrade(&clients[0]);
        let last = Arc::downgrade(&servers[128]);
        listener.close();
        drop((clients, servers, listener));
        socket_retired(&fixture.net)?;
        context.expect_eq(first.upgrade().is_none(), true);
        context.expect_eq(last.upgrade().is_none(), true);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }
}
