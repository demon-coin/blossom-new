// 每连接推进、raw deadline、选项继承与正常 FIN/TIME_WAIT 归同一生命周期套件。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-socket", "network-tcp-progress"), timeout_ms = 45000)]
pub(crate) mod socket_tcp_progress {
    use ::core::sync::atomic::{AtomicBool, Ordering};
    use ::ktest::{TestContext, TestError, TestResult};
    use alloc::sync::Arc;
    use smoltcp::socket::tcp::{CongestionControl, State};

    use super::owner::StackOwner;
    use super::socket_helpers::{socket_listener, socket_read_exact, socket_write_all};
    use super::support::{observe, operation, pattern, socket_retired, SERVER};
    use crate::fs::{FileIoContext, FileLike, FsError};
    use crate::ktest::execution_services::{baseline, SuiteBaseline};
    use crate::net::address::{AddressFamily, InetEndpoint, SocketAddress};
    use crate::net::core::ProtocolDeadline;
    use crate::net::socket::{SocketFile, SHUT_WR};
    use crate::{cpu, mm, time, timer};

    /// 冷 OFD 和全部末弱观察先交给原 fixture，正文失败也由相同拥有结束。
    fn prepare_cold_tcp(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
        count: usize,
    ) -> Result<usize, TestError> {
        fixture
            .tcp_candidates
            .try_reserve_exact(count + 2)
            .map_err(|_| TestError("cold TCP candidate storage OOM"))?;
        fixture
            .tcp_observers
            .try_reserve_exact(count + 2)
            .map_err(|_| TestError("cold TCP observer storage OOM"))?;
        let mut configured_bytes = 0;
        for _ in 0..count {
            let file = operation(
                context,
                SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
            )?;
            let endpoint = operation(context, file.tcp_endpoint())?;
            context.expect_eq(
                fixture
                    .tcp_candidates
                    .iter()
                    .all(|other| other.id() != file.id()),
                true,
            );
            context.expect_eq(endpoint.state(), State::Closed);
            context.expect_eq(operation(context, endpoint.local_address())?.port, 0);
            configured_bytes += endpoint.receive_capacity() + endpoint.send_capacity();
            fixture.tcp_observers.push(Arc::downgrade(endpoint));
            fixture.tcp_candidates.push(file);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), count);
        Ok(configured_bytes)
    }

    /// 物理页和 SLUB 的真实增量须小于任一方向 payload 总量，不能靠配置查询证明惰性。
    fn expect_cold_memory(
        context: &mut TestContext<'_>,
        before: mm::MemoryStats,
        after: mm::MemoryStats,
        configured_bytes: usize,
    ) -> TestResult {
        let before_free = before.buddy_free_pages + before.pcp_cached_pages;
        let after_free = after.buddy_free_pages + after.pcp_cached_pages;
        let pages = before_free.saturating_sub(after_free);
        let slab = after
            .slab_backing_pages
            .saturating_sub(before.slab_backing_pages);
        context.diagnostic(format_args!(
            "cold TCP configured-bytes={} occupied-bytes={} slab-bytes={} before={:?} after={:?}",
            configured_bytes,
            pages * mm::PAGE_SIZE,
            slab * mm::PAGE_SIZE,
            before,
            after
        ));
        context.expect_eq(after.present_pages, before.present_pages);
        context.expect_eq(after.reserved_pages, before.reserved_pages);
        context.expect_eq(after.metadata_pages, before.metadata_pages);
        if configured_bytes / 4 <= pages * mm::PAGE_SIZE
            || configured_bytes / 4 <= slab * mm::PAGE_SIZE
        {
            return Err(TestError("cold TCP allocated payload-scale backing"));
        }
        Ok(())
    }

    /// 成功退休必须来自末 OFD close 和原 reaper；根停止只属于 fixture 的失败清理。
    fn retire_cold_tcp(context: &mut TestContext<'_>, fixture: &mut StackOwner) -> TestResult {
        fixture.tcp_candidates.clear();
        socket_retired(&fixture.net)?;
        for observer in &fixture.tcp_observers {
            context.expect_eq(observer.upgrade().is_none(), true);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        fixture.tcp_observers.clear();
        Ok(())
    }

    /// 默认配置的真实冷 TCP 跨过旧端点边界，全部身份和末弱引用按普通 close 归还。
    #[test]
    #[case("tcp129", 129usize)]
    #[case("tcp1024", 1024usize)]
    #[ktest(tags("network-growth", "network-tcp-cold"))]
    fn cold_tcp_owners_grow_without_payload_backing(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
        count: usize,
    ) -> TestResult {
        let before = baseline().0;
        let configured_bytes = prepare_cold_tcp(context, fixture, count)?;
        fixture.net.work_queue().flush();
        expect_cold_memory(context, before, baseline().0, configured_bytes)?;
        retire_cold_tcp(context, fixture)
    }

    /// 明确端口绑定保持冷 ring，释放中间端点后同端口取得新租约且不能被旧身份撤销。
    #[test]
    #[ktest(tags("network-growth", "network-tcp-cold"))]
    fn cold_tcp_1024_bindings_reuse_middle_port_with_new_lease(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let before = baseline().0;
        let configured_bytes = prepare_cold_tcp(context, fixture, 1024)?;
        let cold = baseline().0;
        expect_cold_memory(context, before, cold, configured_bytes)?;
        for (ordinal, file) in fixture.tcp_candidates.iter().enumerate() {
            let local = InetEndpoint {
                port: 47000 + ordinal as u16,
                ..SERVER
            };
            operation(context, file.bind(SocketAddress::Inet(local)))?;
            let endpoint = operation(context, file.tcp_endpoint())?;
            context.expect_eq(operation(context, endpoint.local_address())?, local);
            context.expect_eq(endpoint.state(), State::Closed);
        }
        fixture.net.work_queue().flush();
        expect_cold_memory(context, cold, baseline().0, configured_bytes)?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 1024);
        let middle = 512;
        let local = InetEndpoint {
            port: 47512,
            ..SERVER
        };
        let old_id = fixture.tcp_candidates[middle].id();
        let old_lease = operation(context, fixture.tcp_candidates[middle].tcp_endpoint())?
            .ktest_binding_identity()
            .ok_or(TestError("cold TCP binding has no lease"))?;
        drop(fixture.tcp_candidates.remove(middle));
        // Weak 失效先于字段析构结束，端点退休还须等待 EndpointSlot 归还计数。
        observe(
            || {
                fixture.tcp_observers[middle].upgrade().is_none()
                    && fixture.net.ktest_live_endpoints() == 1023
            },
            "middle cold TCP did not retire normally",
        )?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 1023);
        let replacement = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        fixture.tcp_observers.push(Arc::downgrade(operation(
            context,
            replacement.tcp_endpoint(),
        )?));
        fixture.tcp_candidates.push(replacement);
        let replacement = fixture.tcp_candidates.last().unwrap();
        operation(context, replacement.bind(SocketAddress::Inet(local)))?;
        let replacement_endpoint = operation(context, replacement.tcp_endpoint())?;
        let lease = replacement_endpoint
            .ktest_binding_identity()
            .ok_or(TestError("replacement TCP binding has no lease"))?;
        context.expect_eq(replacement.id() != old_id, true);
        context.expect_eq(lease != old_lease, true);
        context.expect_eq(
            operation(context, replacement_endpoint.local_address())?,
            local,
        );
        context.expect_eq(fixture.net.ktest_live_endpoints(), 1024);
        let probe = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        fixture
            .tcp_observers
            .push(Arc::downgrade(operation(context, probe.tcp_endpoint())?));
        fixture.tcp_candidates.push(probe);
        context.expect_eq(
            fixture
                .tcp_candidates
                .last()
                .unwrap()
                .bind(SocketAddress::Inet(local)),
            Err(FsError::AddressInUse),
        );
        retire_cold_tcp(context, fixture)
    }

    /// 只借原协议对象读取选项和预算，不安装后备或安排协议处理。
    fn tcp_policy(
        file: &SocketFile,
    ) -> Result<(bool, bool, CongestionControl, usize, usize), FsError> {
        let endpoint = file.tcp_endpoint()?;
        Ok((
            endpoint.nodelay()?,
            endpoint.keepalive(),
            endpoint.congestion_control(),
            endpoint.receive_budget(),
            endpoint.send_budget(),
        ))
    }

    /// 首次激活真实 OOM 保留同一未绑定对象；放压后完成成熟握手、双向全文及 FIN。
    #[test]
    #[case("listen", true)]
    #[case("connect", false)]
    #[ktest(tags("network-growth", "network-tcp-cold"))]
    fn cold_tcp_activation_oom_preserves_policy_and_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
        listening: bool,
    ) -> TestResult {
        prepare_cold_tcp(context, fixture, 2)?;
        let listener = fixture.tcp_candidates[0].clone();
        let client = fixture.tcp_candidates[1].clone();
        let subject = if listening {
            listener.clone()
        } else {
            client.clone()
        };
        let local = InetEndpoint {
            port: 48600,
            ..SERVER
        };
        operation(context, subject.set_nodelay(true))?;
        operation(context, subject.set_keepalive(true))?;
        let endpoint = operation(context, subject.tcp_endpoint())?;
        operation(context, endpoint.set_receive_budget(1024))?;
        operation(context, endpoint.set_send_budget(2048))?;
        let policy = operation(context, tcp_policy(&subject))?;
        context.expect_eq(policy, (true, true, CongestionControl::Reno, 1024, 2048));
        if !listening {
            operation(context, listener.bind(SocketAddress::Inet(local)))?;
            operation(context, listener.listen(1))?;
        }
        for file in [&listener, &client] {
            file.set_timeout(true, Some(5_000_000_000));
            file.set_timeout(false, Some(5_000_000_000));
        }
        fixture.net.work_queue().flush();
        fixture.pressure = Some(crate::cache::ktest::HeapPressure::prepare()?);
        let evidence = fixture.pressure.as_mut().unwrap().exhaust();
        let result = if listening {
            subject.listen(1)
        } else {
            subject.connect(SocketAddress::Inet(local), FileIoContext::Kernel)
        };
        let address_after = endpoint.local_address();
        let policy_after = tcp_policy(&subject);
        let state_after = endpoint.state();
        drop(fixture.pressure.take());
        context.diagnostic(format_args!(
            "cold TCP activation listen={} pressure={:?} result={:?}",
            listening, evidence, result
        ));
        context.expect_eq(evidence, (true, true));
        context.expect_eq(result, Err(FsError::NoMemory));
        context.expect_eq(
            address_after,
            Ok(InetEndpoint {
                address: AddressFamily::Ipv4.unspecified(),
                port: 0,
            }),
        );
        context.expect_eq(state_after, State::Closed);
        context.expect_eq(policy_after, Ok(policy));
        context.expect_eq(endpoint.ktest_binding_identity(), None);
        if evidence != (true, true) || result != Err(FsError::NoMemory) {
            return Err(TestError("cold TCP activation did not encounter real OOM"));
        }
        if listening {
            operation(context, listener.bind(SocketAddress::Inet(local)))?;
            operation(context, listener.listen(1))?;
        }
        operation(
            context,
            client.connect(SocketAddress::Inet(local), FileIoContext::Kernel),
        )?;
        let server = operation(context, listener.accept(0, 0, 0, FileIoContext::Kernel))?;
        fixture
            .tcp_observers
            .push(Arc::downgrade(operation(context, server.tcp_endpoint())?));
        fixture.tcp_candidates.push(server.clone());
        server.set_timeout(true, Some(5_000_000_000));
        server.set_timeout(false, Some(5_000_000_000));
        context.expect_eq(operation(context, tcp_policy(&subject))?, policy);
        if listening {
            context.expect_eq(operation(context, tcp_policy(&server))?, policy);
        }
        context.expect_eq(
            operation(context, client.tcp_endpoint())?.state(),
            State::Established,
        );
        context.expect_eq(
            operation(context, server.tcp_endpoint())?.state(),
            State::Established,
        );
        let mut position = 0;
        while position < 8193 {
            let count = (8193 - position).min(97);
            let input: [u8; 97] = ::core::array::from_fn(|offset| pattern(position + offset));
            let mut output = [0u8; 97];
            socket_write_all(&client, &input[..count])?;
            socket_read_exact(&server, &mut output[..count])?;
            context.expect_eq(&output[..count], &input[..count]);
            socket_write_all(&server, &output[..count])?;
            socket_read_exact(&client, &mut output[..count])?;
            context.expect_eq(&output[..count], &input[..count]);
            position += count;
        }
        operation(context, client.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, server.read(&mut [0u8; 1]))?, 0);
        operation(context, server.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, client.read(&mut [0u8; 1]))?, 0);
        drop((server, subject, client, listener));
        retire_cold_tcp(context, fixture)
    }
    /// A 的真实处理拥有持续期间，另一 CPU 的 B 普通文件收发和后台 ACK 独立完成。
    #[test]
    #[ktest(min_cpus = 2)]
    fn socket_other_cpu_completes_while_connection_is_owned(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener_a = socket_listener(context, &fixture.net, 45610)?;
        let listener_b = socket_listener(context, &fixture.net, 45611)?;
        let client_a = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        let client_b = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        let cpu_a = client_a.tcp_endpoint().unwrap().work_cpu();
        let cpu_b = client_b.tcp_endpoint().unwrap().work_cpu();
        if cpu_a == cpu_b {
            return Err(TestError(
                "native socket clients do not have distinct protocol CPUs",
            ));
        }
        operation(
            context,
            client_a.connect(
                SocketAddress::Inet(InetEndpoint {
                    port: 45610,
                    ..SERVER
                }),
                FileIoContext::Kernel,
            ),
        )?;
        operation(
            context,
            client_b.connect(
                SocketAddress::Inet(InetEndpoint {
                    port: 45611,
                    ..SERVER
                }),
                FileIoContext::Kernel,
            ),
        )?;
        let server_a = operation(context, listener_a.accept(0, 0, 0, FileIoContext::Kernel))?;
        let server_b = operation(context, listener_b.accept(0, 0, 0, FileIoContext::Kernel))?;
        let held = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("native socket ownership observation allocation failed"))?;
        let held_by_a = held.clone();
        let release = fixture.release.clone();
        let a = client_a.clone();
        let holder = crate::ktest::spawn_result_on(cpu_a, move |token| {
            let guard = a.tcp_endpoint().unwrap().lock();
            if cpu::current().id != cpu_a || guard.state() != State::Established {
                return Err(TestError("native socket owner has wrong CPU or state"));
            }
            held_by_a.store(true, Ordering::Release);
            while !release.load(Ordering::Acquire) && !token.stop_requested() {
                timer::sleep(time::Duration::from_millis(1));
            }
            drop(guard);
            Ok(0)
        })?;
        observe(
            || held.load(Ordering::Acquire),
            "native A did not take processing ownership",
        )?;
        let b = client_b.clone();
        let b_complete = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("native B completion observation allocation failed"))?;
        let completed = b_complete.clone();
        let child = crate::ktest::spawn_result_on(cpu_b, move |_| {
            if cpu::current().id != cpu_b {
                return Err(TestError("native B ran on wrong CPU"));
            }
            let input: [u8; 97] = ::core::array::from_fn(pattern);
            socket_write_all(&b, &input)?;
            let mut reply = [0u8; 97];
            socket_read_exact(&b, &mut reply)?;
            if reply != input {
                return Err(TestError("native B response bytes differ"));
            }
            completed.store(true, Ordering::Release);
            Ok(0)
        })?;
        let mut output = [0u8; 97];
        socket_read_exact(&server_b, &mut output)?;
        context.expect_eq(output, ::core::array::from_fn(pattern));
        socket_write_all(&server_b, &output)?;
        observe(
            || b_complete.load(Ordering::Acquire),
            "native B stalled under A ownership",
        )?;
        context.expect_eq(fixture.release.load(Ordering::Acquire), false);
        context.expect_eq(child.join()?, 0);
        fixture.release.store(true, Ordering::Release);
        context.expect_eq(holder.join()?, 0);
        drop((
            client_a, client_b, server_a, server_b, listener_a, listener_b,
        ));
        socket_retired(&fixture.net)
    }

    /// 只丢一笔普通 data 段，真实 raw At/RTO 后同 seq 重到达并交付全部字节。
    #[test]
    fn one_data_segment_retransmits_at_real_raw_deadline(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener = socket_listener(context, &fixture.net, 45620)?;
        let client = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        operation(
            context,
            client.bind(SocketAddress::Inet(InetEndpoint {
                port: 45621,
                ..SERVER
            })),
        )?;
        operation(
            context,
            client.connect(
                SocketAddress::Inet(InetEndpoint {
                    port: 45620,
                    ..SERVER
                }),
                FileIoContext::Kernel,
            ),
        )?;
        let server = operation(context, listener.accept(0, 0, 0, FileIoContext::Kernel))?;
        let healthy_listener = socket_listener(context, &fixture.net, 45622)?;
        let healthy_client = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        operation(
            context,
            healthy_client.connect(
                SocketAddress::Inet(InetEndpoint {
                    port: 45622,
                    ..SERVER
                }),
                FileIoContext::Kernel,
            ),
        )?;
        let healthy_server = operation(
            context,
            healthy_listener.accept(0, 0, 0, FileIoContext::Kernel),
        )?;
        operation(context, fixture.net.ktest_drop_next_data(45621, 45620))?;
        let input: [u8; 97] = ::core::array::from_fn(pattern);
        context.expect_eq(operation(context, client.write(&input))?, input.len());
        observe(
            || fixture.net.ktest_loss_observation().dropped == 1,
            "selected data segment was not dropped",
        )?;
        let first = fixture.net.ktest_loss_observation();
        let deadline = match client.tcp_endpoint().unwrap().lock().deadline(time::now()) {
            ProtocolDeadline::At(at) => at,
            _ => {
                return Err(TestError(
                    "dropped data did not retain actual raw retransmission deadline",
                ))
            }
        };
        if deadline.as_nanos() <= first.first_ns
            || first.retransmissions != 0
            || first.sequence.is_none()
        {
            return Err(TestError("data drop observation did not precede raw RTO"));
        }
        let mut output = [0u8; 97];
        socket_read_exact(&server, &mut output)?;
        context.expect_eq(output, input);
        let loss = fixture.net.ktest_loss_observation();
        context.expect_eq(loss.dropped, 1);
        context.expect_eq(loss.sequence, first.sequence);
        if loss.retransmissions == 0 || loss.repeated_ns < deadline.as_nanos() {
            return Err(TestError(
                "data arrived without actual At deadline retransmission",
            ));
        }
        context.expect_eq(client.tcp_endpoint().unwrap().state(), State::Established);
        context.expect_eq(server.tcp_endpoint().unwrap().state(), State::Established);
        context.expect_eq(
            healthy_client.tcp_endpoint().unwrap().state(),
            State::Established,
        );
        context.expect_eq(
            healthy_server.tcp_endpoint().unwrap().state(),
            State::Established,
        );
        socket_write_all(&healthy_client, &input)?;
        socket_read_exact(&healthy_server, &mut output)?;
        context.expect_eq(output, input);
        context.diagnostic(format_args!(
            "network-socket controlled_drop={} sequence={:?} first_ns={} raw_deadline_ns={} repeated_ns={} retransmissions={}",
            loss.dropped, loss.sequence, loss.first_ns, deadline.as_nanos(), loss.repeated_ns, loss.retransmissions
        ));
        operation(context, fixture.net.ktest_clear_packet_control())?;
        drop((
            client,
            server,
            listener,
            healthy_client,
            healthy_server,
            healthy_listener,
        ));
        socket_retired(&fixture.net)
    }

    /// 末 OFD 先释放 watch，协议仍送 FIN 并进入真实 TIME_WAIT，最终 Weak 和 slot 同时归还。
    #[test]
    fn last_ofd_fin_time_wait_and_endpoint_slot_retire_normally(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener = socket_listener(context, &fixture.net, 45630)?;
        let client = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 1000, 2000),
        )?;
        operation(
            context,
            client.connect(
                SocketAddress::Inet(InetEndpoint {
                    port: 45630,
                    ..SERVER
                }),
                FileIoContext::Kernel,
            ),
        )?;
        let server = operation(context, listener.accept(0, 0, 0, FileIoContext::Kernel))?;
        let file_weak = Arc::downgrade(&client);
        let client_weak = Arc::downgrade(client.tcp_endpoint().unwrap());
        let server_weak = Arc::downgrade(server.tcp_endpoint().unwrap());
        socket_write_all(&client, b"tail before FIN")?;
        drop(client);
        context.expect_eq(file_weak.upgrade().is_none(), true);
        context.expect_eq(client_weak.upgrade().is_some(), true);
        let mut output = [0u8; 15];
        socket_read_exact(&server, &mut output)?;
        context.expect_eq(&output, b"tail before FIN");
        context.expect_eq(operation(context, server.read(&mut [0u8; 1]))?, 0);
        context.expect_eq(server.tcp_endpoint().unwrap().state(), State::CloseWait);
        drop(server);
        observe(
            || {
                client_weak
                    .upgrade()
                    .is_some_and(|endpoint| endpoint.state() == State::TimeWait)
            },
            "last OFD did not progress through real TIME_WAIT",
        )?;
        context.expect_eq(fixture.net.is_stopping(), false);
        context.expect_eq(0 < fixture.net.ktest_live_endpoints(), true);
        drop(listener);
        socket_retired(&fixture.net)?;
        context.expect_eq(client_weak.upgrade().is_none(), true);
        context.expect_eq(server_weak.upgrade().is_none(), true);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        drop((file_weak, client_weak, server_weak));
        Ok(())
    }

    /// TCP_NODELAY 修改真正 raw Nagle，child 继承后仍可独立修改而不改变 listener。
    #[test]
    fn socket_options_modify_raw_nagle_and_child_inherits(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let listener = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        let endpoint = operation(context, listener.tcp_endpoint())?;
        context.expect_eq(endpoint.lock().nagle_enabled(), true);
        operation(context, listener.set_nodelay(true))?;
        context.expect_eq(endpoint.lock().nagle_enabled(), false);
        operation(context, listener.set_nodelay(false))?;
        context.expect_eq(endpoint.lock().nagle_enabled(), true);
        operation(context, listener.set_nodelay(true))?;
        operation(context, listener.set_reuse_address(true))?;
        operation(context, endpoint.set_receive_budget(1024))?;
        operation(context, endpoint.set_send_budget(2048))?;
        operation(context, endpoint.set_keepalive(true))?;
        operation(
            context,
            listener.bind(SocketAddress::Inet(InetEndpoint {
                port: 45660,
                ..SERVER
            })),
        )?;
        operation(context, listener.listen(4))?;
        let client = operation(
            context,
            SocketFile::try_new_tcp(fixture.net.clone(), AddressFamily::Ipv4, 0, 0, 0),
        )?;
        operation(
            context,
            client.connect(
                SocketAddress::Inet(InetEndpoint {
                    port: 45660,
                    ..SERVER
                }),
                FileIoContext::Kernel,
            ),
        )?;
        let child = operation(context, listener.accept(0, 0, 0, FileIoContext::Kernel))?;
        let child_endpoint = operation(context, child.tcp_endpoint())?;
        context.expect_eq(child_endpoint.receive_budget(), 1024);
        context.expect_eq(child_endpoint.send_budget(), 2048);
        context.expect_eq(
            child_endpoint.receive_capacity(),
            endpoint.receive_capacity(),
        );
        context.expect_eq(child_endpoint.send_capacity(), endpoint.send_capacity());
        context.expect_eq(child_endpoint.keepalive(), true);
        context.expect_eq(
            child_endpoint.congestion_control(),
            smoltcp::socket::tcp::CongestionControl::Reno,
        );
        operation(context, child_endpoint.set_keepalive(false))?;
        context.expect_eq(child_endpoint.keepalive(), false);
        context.expect_eq(endpoint.keepalive(), true);
        context.expect_eq(
            operation(context, child.tcp_endpoint())?
                .lock()
                .nagle_enabled(),
            false,
        );
        context.expect_eq(child.reuse_address(), true);
        operation(context, child.set_nodelay(false))?;
        operation(context, child.set_reuse_address(false))?;
        context.expect_eq(
            operation(context, child.tcp_endpoint())?
                .lock()
                .nagle_enabled(),
            true,
        );
        context.expect_eq(endpoint.lock().nagle_enabled(), false);
        context.expect_eq(listener.reuse_address(), true);
        socket_write_all(&client, b"raw policy child")?;
        let mut output = [0u8; 16];
        socket_read_exact(&child, &mut output)?;
        context.expect_eq(&output, b"raw policy child");
        operation(context, client.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, child.read(&mut [0u8; 1]))?, 0);
        operation(context, child.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, client.read(&mut [0u8; 1]))?, 0);
        drop((child, client, listener));
        socket_retired(&fixture.net)?;
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }
}
