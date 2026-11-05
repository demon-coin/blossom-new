// listener 的 ready child、accept 与边沿通知共享完整握手交错。
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

    /// 已领取 child 正常退休不得重发旧 ready 的 ET；后继握手仍须产生新事件并正常归还全部端点。
    #[test]
    fn accepted_child_retirement_does_not_rearm_listener_edge(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
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

        operation(context, client_a.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, server_a.read(&mut [0u8; 1]))?, 0);
        operation(context, server_a.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, client_a.read(&mut [0u8; 1]))?, 0);
        drop(server_a);
        context.expect_eq(server_a_file_weak.upgrade().is_none(), true);
        // 被动端最后 Weak 失效覆盖退休者等待 Timer/Work、撤 parent child 和释放表拥有的全尾。
        observe(
            || server_a_weak.upgrade().is_none(),
            "accepted A did not finish normal FIN/EOF endpoint retirement",
        )?;
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

    /// 首笔握手 ACK 丢失后仅用 FIN 完成被动握手，accept 必须交付同一 child 并保留反向流。
    #[test]
    fn passive_fin_ack_matures_before_accept(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
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

        // 首笔应用发送就是 FIN；任何先行 payload 都会另带 ACK，使这个被动握手窗口消失。
        operation(context, client.shutdown(SHUT_WR))?;
        observe(
            || child.state() == State::CloseWait,
            "passive child did not process actual FIN into CLOSE-WAIT",
        )?;
        let fin_state = child.state();
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

    /// 监听键独立于冷 TCP 拥有和绑定数量增长，端口撤销后仍可在同一活根重新绑定。
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

    /// 独立 backlog 接纳129个成熟 child；重复 listen 保 ready，全文和 FIN/TIME_WAIT 都由原 raw 推进。
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

        for client in &clients {
            servers.push(super::stack_helpers::accepted(context, &listener, client)?);
        }
        context.expect_eq(
            operation(context, listener.poll(None))?,
            PollMask::default(),
        );

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
