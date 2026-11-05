// 完整 inline suite 保留原 module_path 与静态注册身份。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-stack"), timeout_ms = 30000)]
pub(crate) mod stack {
    use ::core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use ::ktest::{TestContext, TestError, TestResult};
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use smoltcp::socket::tcp::State;
    use smoltcp::socket::udp as raw_udp;
    use smoltcp::wire::{IpAddress, IpProtocol, IpRepr, Ipv4Address, Ipv6Address, UdpRepr};

    use super::owner::StackOwner;
    use super::socket_udp_lease::receive_exact;
    use super::stack_helpers::{accepted, listener, udp_buffer};
    use super::support::{
        child_operation, observe, operation, pattern, socket_retired, CLIENT, SERVER,
    };
    use crate::cache::ktest::HeapPressure;
    use crate::fs::{FsError, PollMask, ReadBuffer, WriteBuffer};
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::net::address::{AddressFamily, InetEndpoint, SocketAddress};
    use crate::net::packet::OwnedPacket;
    use crate::net::route::InterfaceId;
    use crate::net::table::NetTables;
    use crate::net::udp::UdpEndpoint;
    use crate::net::{IoProgress, LOOPBACK_CAPACITY, LOOPBACK_MTU};
    use crate::sync::{Completion, SpinLock};
    use crate::{cpu, time, timer};

    /// 独立生产表填满实际容量后，真实 OOM 只阻止新索引发表，原端点和双向消息仍可恢复。
    #[test]
    #[ktest(tags("network-growth"))]
    fn udp_owner_table_growth_no_memory_preserves_prepared_objects(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let first = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        child_operation(first.bind(SERVER))?;
        let candidate = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        let source = child_operation(candidate.bind(CLIENT))?;
        // 独立表不接管回调寿命；原 fixture 的正式表始终覆盖全部端点直到同步停止。
        let table = child_operation(NetTables::try_new())?;
        child_operation(table.insert_udp(Arc::clone(&first)))?;
        let capacity = table.ktest_udp_storage()[0].1;
        let mut fillers = Vec::new();
        fillers
            .try_reserve_exact(capacity - 1)
            .map_err(|_| TestError("UDP owner table fillers OOM"))?;
        while table.ktest_udp_storage()[0].0 < capacity {
            let endpoint = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
            child_operation(table.insert_udp(Arc::clone(&endpoint)))?;
            fillers.push(endpoint);
        }
        let before = table.ktest_udp_storage();
        let root_before = fixture.net.tables().ktest_udp_storage();
        context.expect_eq(before[0].0, before[0].1);

        let mut pressure = HeapPressure::prepare()?;
        fixture.net.work_queue().flush();
        let evidence = pressure.exhaust();
        let result = table.insert_udp(Arc::clone(&candidate));
        let after = table.ktest_udp_storage();
        let root_after = fixture.net.tables().ktest_udp_storage();
        let old_present = ::core::iter::once(&first)
            .chain(fillers.iter())
            .all(|endpoint| {
                table
                    .udp_by_id(endpoint.id())
                    .is_some_and(|entry| Arc::ptr_eq(&entry, endpoint))
            });
        let unpublished = table.udp_by_id(candidate.id()).is_none();
        let root_key = fixture
            .net
            .tables()
            .lookup_udp(SERVER, source)
            .map(|entry| entry.id());
        drop(pressure);

        context.diagnostic(format_args!(
            "UDP owner table capacity={} pressure={:?} insert={:?}",
            before[0].1, evidence, result
        ));
        context.expect_eq(evidence, (true, true));
        context.expect_eq(result, Err(FsError::NoMemory));
        context.expect_eq(after, before);
        context.expect_eq(root_after, root_before);
        context.expect_eq(old_present, true);
        context.expect_eq(unpublished, true);
        context.expect_eq(root_key, Some(first.id()));

        child_operation(table.insert_udp(Arc::clone(&candidate)))?;
        context.expect_eq(table.ktest_udp_storage()[0].0, before[0].0 + 1);
        context.expect_eq(
            table
                .udp_by_id(candidate.id())
                .is_some_and(|entry| Arc::ptr_eq(&entry, &candidate)),
            true,
        );
        for (sender, receiver, expected_source, destination) in [
            (&candidate, &first, source, SERVER),
            (&first, &candidate, SERVER, source),
        ] {
            let payload: [u8; 513] = ::core::array::from_fn(pattern);
            context.expect_eq(
                child_operation(sender.send(&payload, Some(destination)))?,
                IoProgress::Ready(payload.len()),
            );
            receive_exact(receiver, &payload, expected_source)?;
        }

        drop(table);
        for endpoint in &fillers {
            endpoint.close();
        }
        first.close();
        candidate.close();
        drop((first, candidate, fillers));
        socket_retired(&fixture.net)
    }

    /// Completion 同时释放两个真实 Task 的关闭与创建工作，全部成功端点逐个消费真实消息。
    #[test]
    #[ktest(min_cpus = 2, tags("network-growth"))]
    fn udp_close_and_create_publish_parallel_consumes_all_successes(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let sender = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        let source = child_operation(sender.bind(CLIENT))?;
        let mut old = Vec::new();
        old.try_reserve_exact(129)
            .map_err(|_| TestError("UDP closing owners OOM"))?;
        let mut new = Vec::new();
        new.try_reserve_exact(129)
            .map_err(|_| TestError("UDP creating owners OOM"))?;
        for index in 0..129u16 {
            let endpoint = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
            let local = child_operation(endpoint.bind(InetEndpoint {
                port: 47000 + index,
                ..SERVER
            }))?;
            let bytes = index.to_le_bytes();
            context.expect_eq(
                child_operation(sender.send(&bytes, Some(local)))?,
                IoProgress::Ready(bytes.len()),
            );
            receive_exact(&endpoint, &bytes, source)?;
            old.push(endpoint);
        }

        let before = fixture.net.tables().ktest_udp_storage();
        let start =
            Arc::try_new(Completion::new()).map_err(|_| TestError("UDP parallel start OOM"))?;
        let ready =
            Arc::try_new(Completion::new()).map_err(|_| TestError("UDP parallel ready OOM"))?;
        let closing_start = Arc::clone(&start);
        let closing_ready = Arc::clone(&ready);
        let closed = Arc::try_new(AtomicUsize::new(0))
            .map_err(|_| TestError("UDP close observation OOM"))?;
        let closing_count = Arc::clone(&closed);
        let closer =
            crate::ktest::spawn_result_on((cpu::current().id + 1) % cpu::count(), move |_| {
                closing_ready.complete();
                closing_start.wait();
                for endpoint in old {
                    endpoint.close();
                    closing_count.fetch_add(1, Ordering::Release);
                }
                Ok(0)
            })?;

        ready.wait();
        start.complete();
        for index in 0..129u16 {
            let endpoint = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
            child_operation(endpoint.bind(InetEndpoint {
                port: 47200 + index,
                ..SERVER
            }))?;
            new.push(endpoint);
        }
        context.expect_eq(closer.join()?, 0);
        context.expect_eq(closed.load(Ordering::Acquire), 129);
        context.expect_eq(new.len(), 129);
        context.diagnostic(format_args!(
            "UDP parallel close={} created={} storage-before={:?} storage-after={:?}",
            closed.load(Ordering::Acquire),
            new.len(),
            before,
            fixture.net.tables().ktest_udp_storage()
        ));

        for (index, endpoint) in new.iter().enumerate() {
            let local = endpoint
                .local()
                .ok_or(TestError("UDP concurrent publication lost binding"))?;
            let bytes: [u8; 513] = ::core::array::from_fn(|offset| pattern(offset + index * 513));
            context.expect_eq(
                child_operation(sender.send(&bytes, Some(local)))?,
                IoProgress::Ready(bytes.len()),
            );
            receive_exact(endpoint, &bytes, source)?;
        }

        for endpoint in &new {
            endpoint.close();
        }
        sender.close();
        drop((new, sender));
        socket_retired(&fixture.net)
    }

    /// IPv6 通配 port0 的同一租约覆盖 IPv4，V6ONLY port0 则允许同端口的 IPv4 绑定。
    #[test]
    fn udp_dual_stack_port_zero_reserves_both_protocol_faces(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let dual = operation(context, fixture.net.try_new_udp(AddressFamily::Ipv6))?;
        let local = operation(
            context,
            dual.bind(InetEndpoint {
                address: AddressFamily::Ipv6.unspecified(),
                port: 0,
            }),
        )?;
        context.expect_eq(local.address, AddressFamily::Ipv6.unspecified());
        context.expect_eq(49152 <= local.port, true);
        let ipv4 = operation(context, fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        context.expect_eq(
            ipv4.bind(InetEndpoint {
                address: SERVER.address,
                port: local.port,
            }),
            Err(FsError::AddressInUse),
        );

        let native = operation(context, fixture.net.try_new_udp(AddressFamily::Ipv6))?;
        operation(context, native.set_v6_only(true))?;
        let native_local = operation(
            context,
            native.bind(InetEndpoint {
                address: AddressFamily::Ipv6.unspecified(),
                port: 0,
            }),
        )?;
        let ipv4_local = InetEndpoint {
            address: SERVER.address,
            port: native_local.port,
        };
        context.expect_eq(operation(context, ipv4.bind(ipv4_local))?, ipv4_local);

        dual.close();
        native.close();
        ipv4.close();
        drop((dual, native, ipv4));
        socket_retired(&fixture.net)
    }

    /// 同域跨 Task 使用原通配绑定冲突；TCP 与 UDP 的端口拥有彼此独立。
    #[test]
    fn cross_task_bind_wildcard_and_protocol_spaces(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let wildcard = InetEndpoint {
            address: AddressFamily::Ipv4.unspecified(),
            port: 45120,
        };
        let first = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 128, 128),
        )?;
        context.expect_eq(operation(context, first.bind(wildcard))?, wildcard);
        operation(context, first.listen(4))?;
        let net = Arc::clone(&fixture.net);
        let child =
            crate::ktest::spawn_result_on((cpu::current().id + 1) % cpu::count(), move |_| {
                let tcp = child_operation(net.try_new_tcp(AddressFamily::Ipv4, 128, 128))?;
                if tcp.bind(InetEndpoint {
                    address: SERVER.address,
                    port: wildcard.port,
                }) != Err(FsError::AddressInUse)
                {
                    return Err(TestError("cross Task wildcard bind conflict missing"));
                }
                let udp = child_operation(net.try_new_udp(AddressFamily::Ipv4))?;
                if child_operation(udp.bind(wildcard))? != wildcard || udp.local() != Some(wildcard)
                {
                    return Err(TestError("UDP wildcard or protocol space changed"));
                }
                Ok(0)
            })?;
        context.expect_eq(child.join()?, 0);
        context.expect_eq(first.lock().listen_endpoint(AddressFamily::Ipv4), wildcard);
        Ok(())
    }

    /// 两个固定 CPU 的原 Task 同时申请自动端口，每个协议的全部活绑定均不重复。
    #[test]
    #[ktest(min_cpus = 2)]
    fn concurrent_port_zero_is_unique(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let mut ports = Vec::new();
        ports
            .try_reserve_exact(64)
            .map_err(|_| TestError("port observation allocation failed"))?;
        let ports = Arc::try_new(SpinLock::new(ports))
            .map_err(|_| TestError("port observation owner allocation failed"))?;
        let start = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("port start allocation failed"))?;
        let ready = Arc::try_new(AtomicUsize::new(0))
            .map_err(|_| TestError("port readiness allocation failed"))?;
        let mut handles = Vec::new();
        handles
            .try_reserve_exact(2)
            .map_err(|_| TestError("port child handles allocation failed"))?;
        for id in 0..2 {
            let net = Arc::clone(&fixture.net);
            let ports = Arc::clone(&ports);
            let start = Arc::clone(&start);
            let ready = Arc::clone(&ready);
            handles.push(crate::ktest::spawn_result_on(id, move |token| {
                if cpu::current().id != id {
                    return Err(TestError(
                        "automatic port Task did not run on its fixed CPU",
                    ));
                }
                ready.fetch_add(1, Ordering::Release);
                while !start.load(Ordering::Acquire) {
                    if token.stop_requested() {
                        return Ok(0);
                    }
                    timer::sleep(time::Duration::from_millis(1));
                }
                for _ in 0..16 {
                    let tcp = child_operation(net.try_new_tcp(AddressFamily::Ipv4, 128, 128))?;
                    let local = child_operation(tcp.bind(InetEndpoint {
                        address: AddressFamily::Ipv4.unspecified(),
                        port: 0,
                    }))?;
                    if local.address != AddressFamily::Ipv4.unspecified() || local.port < 49152 {
                        return Err(TestError(
                            "automatic TCP bind changed wildcard or port range",
                        ));
                    }
                    ports.lock().push((true, local.port));
                    let udp = child_operation(net.try_new_udp(AddressFamily::Ipv4))?;
                    let local = child_operation(udp.bind(InetEndpoint {
                        address: AddressFamily::Ipv4.unspecified(),
                        port: 0,
                    }))?;
                    if local.address != AddressFamily::Ipv4.unspecified() || local.port < 49152 {
                        return Err(TestError("automatic bind changed wildcard or port range"));
                    }
                    ports.lock().push((false, local.port));
                }
                Ok(0)
            })?);
        }

        observe(
            || ready.load(Ordering::Acquire) == 2,
            "automatic port participants did not reach start gate",
        )?;
        start.store(true, Ordering::Release);
        for handle in handles {
            context.expect_eq(handle.join()?, 0);
        }

        let ports = ports.lock();
        context.expect_eq(ports.len(), 64);
        for (index, port) in ports.iter().enumerate() {
            if ports[..index].contains(port) {
                return Err(TestError("concurrent automatic ports repeated"));
            }
        }
        Ok(())
    }

    /// 握手、全文、环回绕、PEEK 和 FIN 尾部仅由独立后台包处理推进。
    #[test]
    fn background_handshake_full_stream_peek_and_fin(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener = listener(context, &fixture.net, SERVER.port)?;
        let client = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        operation(context, client.connect(SERVER))?;
        let server = accepted(context, &listener, &client)?;

        let input: [u8; 1025] = ::core::array::from_fn(pattern);
        let mut output = [0u8; 1025];
        let mut sent = 0;
        let mut received = 0;
        let end = time::now()
            .checked_add(time::Duration::from_secs(5))
            .unwrap();
        while sent < input.len() || received < input.len() {
            if sent < input.len() {
                if let IoProgress::Ready(count) = operation(
                    context,
                    client.lock().send(&mut WriteBuffer::kernel(&input[sent..])),
                )? {
                    sent += count;
                }
            }
            let mut core = server.lock();
            if 0 < core.receive_queue() {
                let before = core.receive_queue();
                let mut peek = [0u8; 31];
                let IoProgress::Ready(count) = operation(
                    context,
                    core.receive(&mut ReadBuffer::kernel(&mut peek), true),
                )?
                else {
                    return Err(TestError("background TCP peek lost queued data"));
                };
                context.expect_eq(&peek[..count], &input[received..received + count]);
                context.expect_eq(core.receive_queue(), before);
                let IoProgress::Ready(count) = operation(
                    context,
                    core.receive(&mut ReadBuffer::kernel(&mut output[received..]), false),
                )?
                else {
                    return Err(TestError("background TCP read lost queued data"));
                };
                received += count;
            }
            drop(core);
            if end <= time::now() {
                return Err(TestError("background TCP full stream deadline"));
            }
            timer::sleep(time::Duration::from_millis(1));
        }
        context.expect_eq(output, input);

        let tail = b"background final tail";
        context.expect_eq(
            operation(context, client.lock().send(&mut WriteBuffer::kernel(tail)))?,
            IoProgress::Ready(tail.len()),
        );
        client.close();
        observe(
            || server.state() == State::CloseWait,
            "background FIN not received",
        )?;
        let mut bytes = [0u8; 32];
        context.expect_eq(
            operation(
                context,
                server
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?,
            IoProgress::Ready(tail.len()),
        );
        context.expect_eq(&bytes[..tail.len()], tail);
        context.expect_eq(
            operation(
                context,
                server
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?,
            IoProgress::Eof,
        );
        Ok(())
    }

    /// A 真持操作 guard 时，另一个 CPU 的 B 原 Task 和后台连接仍完成全文传输。
    #[test]
    #[ktest(min_cpus = 2)]
    fn independent_connection_advances_while_other_is_owned(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener_a = listener(context, &fixture.net, 45210)?;
        let listener_b = listener(context, &fixture.net, 45211)?;
        let client_a = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        let client_b = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        if client_a.work_cpu() == client_b.work_cpu() {
            return Err(TestError("independent clients share protocol CPU"));
        }
        operation(
            context,
            client_a.connect(InetEndpoint {
                port: 45210,
                ..SERVER
            }),
        )?;
        operation(
            context,
            client_b.connect(InetEndpoint {
                port: 45211,
                ..SERVER
            }),
        )?;
        let server_a = accepted(context, &listener_a, &client_a)?;
        let server_b = accepted(context, &listener_b, &client_b)?;

        let held = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("operation ownership observation allocation failed"))?;
        let held_by_a = Arc::clone(&held);
        let release = Arc::clone(&fixture.release);
        let a = Arc::clone(&client_a);
        let a_cpu = a.work_cpu();
        let holder = crate::ktest::spawn_result_on(a_cpu, move |token| {
            if cpu::current().id != a_cpu {
                return Err(TestError("A operation owner ran on the wrong CPU"));
            }
            let owned = a.lock();
            held_by_a.store(true, Ordering::Release);
            let end = time::now()
                .checked_add(time::Duration::from_secs(5))
                .unwrap();
            while !release.load(Ordering::Acquire) && !token.stop_requested() {
                if end <= time::now() {
                    return Err(TestError("A ownership release deadline"));
                }
                timer::sleep(time::Duration::from_millis(1));
            }
            if owned.state() != State::Established {
                return Err(TestError("owned A protocol state changed"));
            }
            Ok(0)
        })?;
        observe(
            || held.load(Ordering::Acquire),
            "A did not acquire actual operation guard",
        )?;

        let b = Arc::clone(&client_b);
        let target_cpu = b.work_cpu();
        let child = crate::ktest::spawn_result_on(target_cpu, move |_| {
            if cpu::current().id != target_cpu {
                return Err(TestError(
                    "independent client Task did not run on its fixed CPU",
                ));
            }
            let input: [u8; 97] = ::core::array::from_fn(pattern);
            if child_operation(b.lock().send(&mut WriteBuffer::kernel(&input)))?
                != IoProgress::Ready(input.len())
            {
                return Err(TestError("independent B send prefix incomplete"));
            }
            Ok(0)
        })?;
        context.expect_eq(child.join()?, 0);
        observe(
            || server_b.lock().receive_queue() == 97,
            "B stalled while A operation was owned",
        )?;
        let mut output = [0u8; 97];
        context.expect_eq(
            operation(
                context,
                server_b
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut output), false),
            )?,
            IoProgress::Ready(97),
        );
        context.expect_eq(output, ::core::array::from_fn(pattern));
        context.expect_eq(server_a.state(), State::Established);

        fixture.release.store(true, Ordering::Release);
        context.expect_eq(holder.join()?, 0);
        Ok(())
    }

    /// UDP 经真实 lo ingress 交付全文及源，17 字节和空消息各计一包，双向 IP 字节精确累计 73。
    #[test]
    #[ktest(tags("N28"))]
    fn udp_index_delivers_owned_and_zero_length_records(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let udp = operation(context, fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        let destination = operation(
            context,
            udp.bind(InetEndpoint {
                address: AddressFamily::Ipv4.unspecified(),
                port: 45310,
            }),
        )?;
        context.expect_eq(destination.address, AddressFamily::Ipv4.unspecified());
        let before = operation(context, fixture.net.statistics(InterfaceId::Loopback))?;
        for payload in [&b"real UDP record!!"[..], &b""[..]] {
            let packet = operation(
                context,
                OwnedPacket::try_from_udp(
                    CLIENT,
                    InetEndpoint {
                        address: SERVER.address,
                        port: destination.port,
                    },
                    payload,
                    LOOPBACK_MTU,
                ),
            )?;
            operation(context, fixture.net.send_ip(packet))?;
            observe(
                || {
                    udp.poll(None)
                        .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
                },
                "UDP ingress record missing",
            )?;
            let mut bytes = [0u8; 32];
            let record = operation(
                context,
                udp.receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?
            .ok_or(TestError("UDP observed record disappeared"))?;
            context.expect_eq(record.source, Some(SocketAddress::Inet(CLIENT)));
            context.expect_eq(record.copied, payload.len());
            context.expect_eq(record.original_len, payload.len());
            context.expect_eq(&bytes[..record.copied], payload);
        }

        let after = operation(context, fixture.net.statistics(InterfaceId::Loopback))?;
        context.expect_eq(after.rx_bytes.wrapping_sub(before.rx_bytes), 73);
        context.expect_eq(after.rx_packets.wrapping_sub(before.rx_packets), 2);
        context.expect_eq(after.tx_bytes.wrapping_sub(before.tx_bytes), 73);
        context.expect_eq(after.tx_packets.wrapping_sub(before.tx_packets), 2);
        context.diagnostic(format_args!(
            "network-stack lo-stats-before={:?} lo-stats-after={:?} payload-bytes=17 ip-bytes=73 packets=2",
            before, after
        ));
        let mut bytes = [0u8; 1];
        context.expect_eq(
            operation(
                context,
                udp.receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?,
            None,
        );

        udp.close();
        observe(
            || {
                fixture
                    .net
                    .tables()
                    .lookup_udp(
                        InetEndpoint {
                            port: destination.port,
                            ..SERVER
                        },
                        CLIENT,
                    )
                    .is_none()
            },
            "closed UDP did not release actual receive index",
        )?;
        Ok(())
    }

    /// 原 raw 接纳事实确定字节及 metadata 满；缩小保旧记录，调大恢复，不推测网络交付。
    #[test]
    fn udp_inbox_record_and_payload_capacity_recovers(
        context: &mut TestContext<'_>,
        _: &mut StackOwner,
    ) -> TestResult {
        for (address, source) in [
            (
                SERVER.address,
                IpAddress::Ipv4(Ipv4Address::new(127, 0, 0, 2)),
            ),
            (
                IpAddress::Ipv6(Ipv6Address::LOCALHOST),
                IpAddress::Ipv6(Ipv6Address::new(0, 0, 0, 0, 0, 0, 0, 2)),
            ),
        ] {
            let mut raw = raw_udp::Socket::new(udp_buffer(32, 128)?, udp_buffer(32, 128)?);
            raw.bind((address, SERVER.port))
                .map_err(|_| TestError("raw UDP bind failed"))?;
            let mut cx = raw_udp::UdpContextState::new(Some(address), None);
            let repr = UdpRepr {
                src_port: CLIENT.port,
                dst_port: SERVER.port,
            };
            let zero_ip = IpRepr::new(source, address, IpProtocol::Udp, 8, 64);
            for _ in 0..32 {
                context.expect_eq(
                    raw.process(&mut cx, Default::default(), &zero_ip, &repr, &b""[..]),
                    true,
                );
            }
            context.expect_eq(
                raw.process(&mut cx, Default::default(), &zero_ip, &repr, &b""[..]),
                false,
            );
            for _ in 0..32 {
                let (payload, metadata) = raw
                    .recv()
                    .map_err(|_| TestError("raw UDP zero record lost"))?;
                context.expect_eq(payload, &b""[..]);
                context.expect_eq(metadata.endpoint.addr, source);
                context.expect_eq(metadata.endpoint.port, CLIENT.port);
                context.expect_eq(metadata.local_address, Some(address));
            }
            context.expect_eq(raw.can_recv(), false);

            raw.set_recv_budget(64);
            let first: [u8; 32] = ::core::array::from_fn(pattern);
            let second: [u8; 32] = ::core::array::from_fn(|index| pattern(index + 32));
            let ip = IpRepr::new(source, address, IpProtocol::Udp, 40, 64);
            for input in [&first[..], &second[..]] {
                context.expect_eq(
                    raw.process(&mut cx, Default::default(), &ip, &repr, input),
                    true,
                );
            }
            context.expect_eq(
                raw.process(&mut cx, Default::default(), &ip, &repr, &first[..]),
                false,
            );
            raw.set_recv_budget(16);
            for input in [&first[..], &second[..]] {
                let (payload, metadata) = raw
                    .recv()
                    .map_err(|_| TestError("lowered UDP budget lost old record"))?;
                context.expect_eq(payload, input);
                context.expect_eq(metadata.local_address, Some(address));
            }
            context.expect_eq(
                raw.process(&mut cx, Default::default(), &ip, &repr, &first[..]),
                false,
            );
            raw.set_recv_budget(64);
            context.expect_eq(
                raw.process(&mut cx, Default::default(), &ip, &repr, &second[..]),
                true,
            );
            context.expect_eq(
                raw.recv()
                    .map_err(|_| TestError("raised UDP budget did not recover"))?
                    .0,
                &second[..],
            );

            raw.set_send_budget(32);
            let metadata = raw_udp::UdpMetadata {
                endpoint: (source, CLIENT.port).into(),
                local_address: Some(address),
                meta: Default::default(),
                local_port: Some(SERVER.port),
            };
            raw.send_slice(&first, metadata)
                .map_err(|_| TestError("raw UDP TX acceptance failed"))?;
            context.expect_eq(raw.can_send(), false);
            context.expect_eq(
                raw.send_slice(b"x", metadata),
                Err(raw_udp::SendError::BufferFull),
            );
            raw.set_send_budget(16);
            let rejected = raw.dispatch(&mut cx, |_, _, (_, _, payload)| {
                context.expect_eq(payload, &first[..]);
                Err::<(), _>(FsError::WouldBlock)
            });
            context.expect_eq(rejected, Err(FsError::WouldBlock));
            raw.set_send_budget(64);
            let dispatched = raw.dispatch(&mut cx, |_, _, (ip, _, payload)| {
                context.expect_eq(ip.src_addr(), address);
                context.expect_eq(payload, &first[..]);
                Ok(())
            });
            operation(context, dispatched)?;
            context.expect_eq(raw.can_send(), true);
            raw.send_slice(&second, metadata)
                .map_err(|_| TestError("raised UDP TX budget did not recover"))?;
            let dispatched = raw.dispatch(&mut cx, |_, _, (_, _, payload)| {
                context.expect_eq(payload, &second[..]);
                Ok(())
            });
            operation(context, dispatched)?;
        }
        Ok(())
    }

    /// 真实 Closed 之后表与退休 worker 释放最后节点拥有，外部 Weak 不延长 Work/Timer 寿命。
    #[test]
    fn closed_connections_retire_work_timer_and_endpoint_slots(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener = listener(context, &fixture.net, 45410)?;
        let client = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        operation(
            context,
            client.connect(InetEndpoint {
                port: 45410,
                ..SERVER
            }),
        )?;
        let server = accepted(context, &listener, &client)?;
        let client_weak = Arc::downgrade(&client);
        let server_weak = Arc::downgrade(&server);
        client.abandon();
        drop(client);
        drop(server);
        observe(
            || client_weak.upgrade().is_none() && server_weak.upgrade().is_none(),
            "closed protocol nodes retained after retirement",
        )?;
        listener.close();
        let listener_weak = Arc::downgrade(&listener);
        drop(listener);
        observe(
            || listener_weak.upgrade().is_none(),
            "closed listener retained after retirement",
        )?;

        let mut nodes: Vec<Arc<UdpEndpoint>> = Vec::new();
        nodes
            .try_reserve_exact(129)
            .map_err(|_| TestError("endpoint growth observation allocation failed"))?;
        for _ in 0..129 {
            nodes.push(operation(
                context,
                fixture.net.try_new_udp(AddressFamily::Ipv4),
            )?);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), 129);
        for endpoint in &nodes {
            endpoint.close();
        }
        drop(nodes);
        socket_retired(&fixture.net)?;

        let recovered = operation(context, fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        recovered.close();
        Ok(())
    }

    /// 固定 CPU0 的短 IRQ 停点使真实 lo 满；另一核 SYN emitter 登记等待后由容量恢复唤醒。
    #[test]
    #[ktest(min_cpus = 2)]
    fn full_egress_waits_for_real_capacity_and_resumes(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener = listener(context, &fixture.net, 45420)?;
        let client = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        context.expect_eq(client.work_cpu(), 1);
        let destination = InetEndpoint {
            port: 45421,
            ..SERVER
        };
        let mut packets = Vec::new();
        packets
            .try_reserve_exact(LOOPBACK_CAPACITY)
            .map_err(|_| TestError("egress prepared packets allocation failed"))?;
        for _ in 0..LOOPBACK_CAPACITY {
            packets.push(operation(
                context,
                OwnedPacket::try_from_udp(
                    CLIENT,
                    destination,
                    b"real egress capacity",
                    LOOPBACK_MTU,
                ),
            )?);
        }
        let extra = operation(
            context,
            OwnedPacket::try_from_udp(
                CLIENT,
                destination,
                b"unaccepted owned packet",
                LOOPBACK_MTU,
            ),
        )?;

        let entered = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("egress gate observation allocation failed"))?;
        let gate_entered = Arc::clone(&entered);
        let release = Arc::clone(&fixture.release);
        let net = Arc::clone(&fixture.net);
        let controlled_client = Arc::clone(&client);
        let controller = crate::ktest::spawn_result_on(1, move |_| {
            let client = controlled_client;
            let gate_release = Arc::clone(&release);
            let child = crate::ktest::spawn_result_on(0, move |token| {
                // CPU0 只阻止 ingress 执行；包、路由与队列操作留在普通可睡控制任务。
                let irq = crate::sync::IrqSaveGuard::new();
                gate_entered.store(true, Ordering::Release);
                let end = time::now()
                    .checked_add(time::Duration::from_secs(5))
                    .unwrap();
                let mut error = None;
                while !gate_release.load(Ordering::Acquire) && !token.stop_requested() {
                    if end <= time::now() {
                        error = Some("egress IRQ gate release deadline");
                        break;
                    }
                    ::core::hint::spin_loop();
                }
                drop(irq);
                error.map_or(Ok(0), |message| Err(TestError(message)))
            })?;
            // 验证失败也先解除停点并 join；未接纳包始终在 CPU1 普通任务结束拥有。
            let result = (|| -> TestResult {
                observe(
                    || entered.load(Ordering::Acquire) || child.try_join().is_some(),
                    "egress ingress CPU did not enter actual IRQ gate",
                )?;
                if !entered.load(Ordering::Acquire) {
                    return Err(TestError("egress gate failed before entry observation"));
                }
                for packet in packets {
                    net.try_emit(packet).map_err(|_| {
                        TestError("prepared egress queue did not accept 32 records")
                    })?;
                }
                let length = extra.len();
                let unaccepted = match net.try_emit(extra) {
                    Err((FsError::WouldBlock, packet))
                        if packet.len() == length
                            && packet.udp_payload() == Ok(&b"unaccepted owned packet"[..]) =>
                    {
                        packet
                    }
                    Err(_) => {
                        return Err(TestError(
                            "full egress did not preserve unaccepted packet ownership",
                        ));
                    }
                    Ok(()) => {
                        return Err(TestError("egress queue exceeded actual packet capacity"))
                    }
                };

                child_operation(client.connect(InetEndpoint {
                    port: 45420,
                    ..SERVER
                }))?;
                observe(
                    || net.egress_waiting(client.id()),
                    "full SYN emitter did not register actual egress wait",
                )?;
                if client.state() != State::SynSent {
                    return Err(TestError(
                        "full egress submitted SYN before capacity recovered",
                    ));
                }
                timer::sleep(time::Duration::from_millis(20));
                if !net.egress_waiting(client.id()) {
                    return Err(TestError(
                        "egress wait disappeared while ingress CPU was stopped",
                    ));
                }
                drop(unaccepted);
                Ok(())
            })();

            release.store(true, Ordering::Release);
            let joined = child.join();
            result?;
            if joined? != 0 {
                return Err(TestError("egress IRQ gate child failed"));
            }
            Ok(0)
        })?;
        context.expect_eq(controller.join()?, 0);
        let server = accepted(context, &listener, &client)?;
        context.expect_eq(fixture.net.egress_waiting(client.id()), false);

        let input = b"capacity recovered stream";
        context.expect_eq(
            operation(context, client.lock().send(&mut WriteBuffer::kernel(input)))?,
            IoProgress::Ready(input.len()),
        );
        observe(
            || server.lock().receive_queue() == input.len(),
            "capacity-restored stream did not advance independently",
        )?;
        let mut output = [0u8; 64];
        context.expect_eq(
            operation(
                context,
                server
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut output), false),
            )?,
            IoProgress::Ready(input.len()),
        );
        context.expect_eq(&output[..input.len()], input);
        Ok(())
    }

    /// 真实 owner 停止仍被外部 Arc 保活的连接与 UDP 队列，关闭后的域不再接受节点或包。
    #[test]
    fn owner_stop_ends_live_protocol_and_rejects_new_ownership(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener = listener(context, &fixture.net, 45430)?;
        let client = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        operation(
            context,
            client.connect(InetEndpoint {
                port: 45430,
                ..SERVER
            }),
        )?;
        let server = accepted(context, &listener, &client)?;
        let udp = operation(context, fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        let local = operation(
            context,
            udp.bind(InetEndpoint {
                port: 45431,
                ..SERVER
            }),
        )?;
        operation(
            context,
            udp.enqueue(operation(
                context,
                OwnedPacket::try_from_udp(CLIENT, local, b"owned until shutdown", LOOPBACK_MTU),
            )?),
        )?;
        operation(
            context,
            client
                .lock()
                .send(&mut WriteBuffer::kernel(b"pending TCP shutdown")),
        )?;

        drop(fixture.owner.take());
        context.expect_eq(fixture.net.is_stopping(), true);
        context.expect_eq(client.state(), State::Closed);
        context.expect_eq(server.state(), State::Closed);
        context.expect_eq(listener.state(), State::Closed);
        let mut bytes = [0u8; 1];
        context.expect_eq(
            udp.receive(&mut ReadBuffer::kernel(&mut bytes), false),
            Err(FsError::NotConnected),
        );
        context.expect_eq(udp.local(), None);

        match fixture.net.try_new_udp(AddressFamily::Ipv4) {
            Err(FsError::NotConnected) => {}
            _ => return Err(TestError("stopped network accepted new endpoint ownership")),
        }
        let packet = operation(
            context,
            OwnedPacket::try_from_udp(CLIENT, local, b"", LOOPBACK_MTU),
        )?;
        match fixture.net.try_emit(packet) {
            Err((FsError::NotConnected, packet)) => {
                context.expect_eq(operation(context, packet.udp_payload())?, &b""[..]);
            }
            _ => return Err(TestError("stopped network accepted new egress ownership")),
        }
        Ok(())
    }
}
