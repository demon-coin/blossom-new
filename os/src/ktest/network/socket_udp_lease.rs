// UDP 当前绑定、在途旧租约和后继接收键归同一完整交错。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-socket", "network-udp-lease"), timeout_ms = 45000)]
pub(crate) mod socket_udp_lease {
    use ::core::sync::atomic::{AtomicBool, Ordering};
    use ::ktest::{TestContext, TestError, TestResult};
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use super::owner::StackOwner;
    use super::support::{child_operation, observe, socket_retired, SERVER};
    use crate::cache::ktest::HeapPressure;
    use crate::fs::{FsError, PollMask, ReadBuffer};
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::net::address::{AddressFamily, InetEndpoint, SocketAddress};
    use crate::net::udp::UdpEndpoint;
    use crate::net::IoProgress;
    use crate::time;

    /// 每次只等待已有记录，核对完整消息、源地址及消费后的空队列。
    pub(super) fn receive_exact(
        receiver: &UdpEndpoint,
        expected: &[u8],
        source: InetEndpoint,
    ) -> TestResult {
        observe(
            || {
                receiver
                    .poll(None)
                    .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
            },
            "UDP growth record did not arrive",
        )?;
        let mut bytes = [0u8; 513];
        let record = child_operation(receiver.receive(&mut ReadBuffer::kernel(&mut bytes), false))?
            .ok_or(TestError("UDP growth readable record disappeared"))?;
        if record.source != Some(SocketAddress::Inet(source))
            || record.original_len != expected.len()
            || record.copied != expected.len()
            || record.flags != 0
            || &bytes[..record.copied] != expected
        {
            return Err(TestError("UDP growth full payload or source differs"));
        }
        if child_operation(receiver.receive(&mut ReadBuffer::kernel(&mut bytes), false))?.is_some()
        {
            return Err(TestError("UDP growth received an extra record"));
        }
        Ok(())
    }

    /// 大量冷端点只持控制对象，原两向 payload 和 metadata 仍为空，最后别名独立保活槽位。
    #[test]
    #[case("udp129", 129usize)]
    #[case("udp1024", 1024usize)]
    #[ktest(tags("network-growth"))]
    fn cold_udp_objects_keep_configuration_without_payload(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
        count: usize,
    ) -> TestResult {
        let mut endpoints = Vec::new();
        endpoints
            .try_reserve_exact(count)
            .map_err(|_| TestError("cold UDP owners OOM"))?;
        for _ in 0..count {
            let endpoint = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv6))?;
            child_operation(endpoint.set_receive_budget(8193))?;
            child_operation(endpoint.set_send_budget(4097))?;
            child_operation(endpoint.set_reuse_address(true))?;
            child_operation(endpoint.set_v6_only(true))?;
            endpoint.set_timeout(true, Some(17));
            endpoint.set_timeout(false, Some(29));
            context.expect_eq(endpoint.ktest_buffer_storage(), (false, [0; 4]));
            context.expect_eq(endpoint.receive_budget(), 8193);
            context.expect_eq(endpoint.send_budget(), 4097);
            context.expect_eq(endpoint.reuse_address(), true);
            context.expect_eq(endpoint.v6_only(), true);
            context.expect_eq(endpoint.timeouts().receive, Some(17));
            context.expect_eq(endpoint.timeouts().send, Some(29));
            context.expect_eq(
                child_operation(endpoint.poll(None))?.contains(PollMask::POLLOUT),
                true,
            );
            context.expect_eq(endpoint.local(), None);
            context.expect_eq(endpoint.peer_address(), Err(FsError::NotConnected));
            endpoints.push(endpoint);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), count);

        let alias = Arc::clone(endpoints.last().unwrap());
        let weak = Arc::downgrade(&alias);
        for endpoint in &endpoints {
            endpoint.close();
        }
        drop(endpoints);
        observe(
            || fixture.net.ktest_live_endpoints() == 1,
            "cold UDP non-last owners did not retire",
        )?;
        context.expect_eq(weak.upgrade().is_some(), true);

        drop(alias);
        observe(
            || weak.upgrade().is_none(),
            "cold UDP last alias retained endpoint",
        )?;
        socket_retired(&fixture.net)
    }

    /// 首次激活的真实 OOM 保持冷 raw、配置和全部地址索引；放压后同一对象交付完整消息。
    #[test]
    #[case("bind", 0u8)]
    #[case("connect", 1u8)]
    #[case("automatic_send", 2u8)]
    #[ktest(tags("network-growth"))]
    fn first_udp_activation_no_memory_is_atomic_and_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
        activation: u8,
    ) -> TestResult {
        let receiver = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        child_operation(receiver.bind(SERVER))?;
        let sender = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        child_operation(sender.set_receive_budget(8193))?;
        child_operation(sender.set_send_budget(4097))?;
        child_operation(sender.set_reuse_address(true))?;
        sender.set_timeout(true, Some(31));
        sender.set_timeout(false, Some(47));
        let local = InetEndpoint {
            port: 45701,
            ..SERVER
        };
        let payload: [u8; 513] = ::core::array::from_fn(super::support::pattern);
        let before = fixture.net.tables().ktest_udp_storage();

        let mut pressure = HeapPressure::prepare()?;
        fixture.net.work_queue().flush();
        let evidence = pressure.exhaust();
        let result = match activation {
            0 => sender.bind(local).map(|_| ()),
            1 => sender.connect(Some(SERVER)),
            _ => sender.send(&payload, Some(SERVER)).map(|_| ()),
        };
        let after = fixture.net.tables().ktest_udp_storage();
        let storage = sender.ktest_buffer_storage();
        let binding = sender.local();
        let peer = sender.peer_address();
        let mask = sender.poll(None);
        drop(pressure);

        context.diagnostic(format_args!(
            "UDP first activation={} pressure={:?} result={:?}",
            activation, evidence, result
        ));
        context.expect_eq(evidence, (true, true));
        context.expect_eq(result, Err(FsError::NoMemory));
        context.expect_eq(after, before);
        context.expect_eq(storage, (false, [0; 4]));
        context.expect_eq(binding, None);
        context.expect_eq(peer, Err(FsError::NotConnected));
        context.expect_eq(child_operation(mask)?.contains(PollMask::POLLOUT), true);
        context.expect_eq(sender.receive_budget(), 8193);
        context.expect_eq(sender.send_budget(), 4097);
        context.expect_eq(sender.reuse_address(), true);
        context.expect_eq(sender.timeouts().receive, Some(31));
        context.expect_eq(sender.timeouts().send, Some(47));
        context.expect_eq(sender.take_error(), None);

        match activation {
            0 => {
                context.expect_eq(child_operation(sender.bind(local))?, local);
            }
            1 => {
                child_operation(sender.connect(Some(SERVER)))?;
            }
            _ => {}
        }
        let destination = (activation != 1).then_some(SERVER);
        context.expect_eq(
            child_operation(sender.send(&payload, destination))?,
            IoProgress::Ready(payload.len()),
        );
        let source = InetEndpoint {
            address: SERVER.address,
            port: child_operation(sender.local_address())?.port,
        };
        receive_exact(&receiver, &payload, source)?;
        context.expect_eq(sender.receive_budget(), 8193);
        context.expect_eq(sender.send_budget(), 4097);
        context.expect_eq(sender.reuse_address(), true);
        context.expect_eq(sender.timeouts().receive, Some(31));
        context.expect_eq(sender.timeouts().send, Some(47));

        sender.close();
        receiver.close();
        drop((sender, receiver));
        socket_retired(&fixture.net)
    }

    /// 每个独立接收键消费自己的消息，不以成功绑定替代真实分派和全文接收。
    #[test]
    #[ktest(tags("network-growth"))]
    fn udp_129_receive_keys_deliver_each_full_record(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let mut receivers = Vec::new();
        receivers
            .try_reserve_exact(129)
            .map_err(|_| TestError("UDP receive key owners OOM"))?;
        for index in 0..129u16 {
            let receiver = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
            child_operation(receiver.bind(InetEndpoint {
                port: 46000 + index,
                ..SERVER
            }))?;
            receivers.push(receiver);
        }

        let sender = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        for (index, receiver) in receivers.iter().enumerate() {
            let payload: [u8; 513] =
                ::core::array::from_fn(|offset| super::support::pattern(offset + index * 513));
            let local = receiver
                .local()
                .ok_or(TestError("UDP growth binding disappeared"))?;
            context.expect_eq(
                child_operation(sender.send(&payload, Some(local)))?,
                IoProgress::Ready(payload.len()),
            );
            let source = InetEndpoint {
                address: SERVER.address,
                port: child_operation(sender.local_address())?.port,
            };
            receive_exact(receiver, &payload, source)?;
        }

        context.expect_eq(fixture.net.tables().ktest_udp_storage()[2].0, 129 + 1);
        sender.close();
        for receiver in &receivers {
            receiver.close();
        }
        drop((receivers, sender));
        socket_retired(&fixture.net)
    }

    /// 满后备时同身份重发表只调整优先级，新增键的真实 OOM 不影响旧 lease 和接收顺序。
    #[test]
    #[ktest(tags("network-growth"))]
    fn full_udp_key_storage_updates_without_allocation_and_growth_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let sender = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        let source = child_operation(sender.bind(super::support::CLIENT))?;
        let first = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        child_operation(first.set_reuse_address(true))?;
        child_operation(first.bind(SERVER))?;
        let candidate = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        child_operation(candidate.set_reuse_address(true))?;
        child_operation(candidate.bind(SERVER))?;
        fixture.net.tables().unpublish_udp(candidate.id());
        let capacity = fixture.net.tables().ktest_udp_storage()[2].1;
        let mut receivers = Vec::new();
        receivers
            .try_reserve_exact(capacity)
            .map_err(|_| TestError("UDP full key owners OOM"))?;
        receivers.push(Arc::clone(&first));
        while fixture.net.tables().ktest_udp_storage()[2].0 < capacity {
            let receiver = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
            child_operation(receiver.set_reuse_address(true))?;
            child_operation(receiver.bind(SERVER))?;
            receivers.push(receiver);
        }
        let old_winner = fixture
            .net
            .tables()
            .lookup_udp(SERVER, source)
            .ok_or(TestError("UDP reuse key missing before pressure"))?;
        context.expect_eq(old_winner.id(), receivers.last().unwrap().id());
        let before = fixture.net.tables().ktest_udp_storage();
        context.expect_eq(before[2], (capacity, capacity));

        let mut pressure = HeapPressure::prepare()?;
        fixture.net.work_queue().flush();
        let evidence = pressure.exhaust();
        let connected = first.connect(Some(source));
        let connected_winner = fixture
            .net
            .tables()
            .lookup_udp(SERVER, source)
            .map(|entry| entry.id());
        let failed = fixture
            .net
            .tables()
            .publish_udp(candidate.id(), SERVER, None);
        let after_failure = fixture.net.tables().ktest_udp_storage();
        let disconnected = first.connect(None);
        let final_winner = fixture
            .net
            .tables()
            .lookup_udp(SERVER, source)
            .map(|entry| entry.id());
        let binding = first.local();
        drop(pressure);

        context.diagnostic(format_args!(
            "UDP key capacity={} pressure={:?} connected={:?} insert={:?} disconnected={:?}",
            capacity, evidence, connected, failed, disconnected
        ));
        context.expect_eq(evidence, (true, true));
        context.expect_eq(connected, Ok(()));
        context.expect_eq(connected_winner, Some(first.id()));
        context.expect_eq(failed, Err(FsError::NoMemory));
        context.expect_eq(after_failure, before);
        context.expect_eq(disconnected, Ok(()));
        context.expect_eq(final_winner, Some(first.id()));
        context.expect_eq(binding, Some(SERVER));
        context.expect_eq(first.peer_address(), Err(FsError::NotConnected));

        let payload = b"same id retained its original lease";
        context.expect_eq(
            child_operation(sender.send(payload, Some(SERVER)))?,
            IoProgress::Ready(payload.len()),
        );
        receive_exact(&first, payload, source)?;
        child_operation(candidate.connect(None))?;
        context.expect_eq(
            fixture
                .net
                .tables()
                .lookup_udp(SERVER, source)
                .map(|entry| entry.id()),
            Some(candidate.id()),
        );
        context.expect_eq(
            child_operation(sender.send(b"new key recovered", Some(SERVER)))?,
            IoProgress::Ready(17),
        );
        receive_exact(&candidate, b"new key recovered", source)?;

        for (index, receiver) in receivers.iter().enumerate() {
            child_operation(receiver.connect(None))?;
            context.expect_eq(
                fixture
                    .net
                    .tables()
                    .lookup_udp(SERVER, source)
                    .map(|entry| entry.id()),
                Some(receiver.id()),
            );
            let bytes: [u8; 37] =
                ::core::array::from_fn(|offset| super::support::pattern(offset + index));
            context.expect_eq(
                child_operation(sender.send(&bytes, Some(SERVER)))?,
                IoProgress::Ready(bytes.len()),
            );
            receive_exact(receiver, &bytes, source)?;
        }

        let contender = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        context.expect_eq(contender.bind(SERVER), Err(FsError::AddressInUse));
        contender.close();
        first.close();
        candidate.close();
        sender.close();
        for receiver in &receivers {
            receiver.close();
        }
        drop((receivers, old_winner, first, candidate, sender, contender));
        socket_retired(&fixture.net)
    }

    /// AF_UNSPEC 保已接纳消息的旧租约，旧消息交付后归端口且后继绑定仍保持唯一。
    #[test]
    #[ktest(min_cpus = 2)]
    fn udp_prepared_lease_survives_disconnect_without_unbinding_successor(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let net = Arc::clone(&fixture.net);
        let release = Arc::clone(&fixture.release);
        let controller = crate::ktest::spawn_result_on(1, move |_| {
            let receiver = child_operation(net.try_new_udp(AddressFamily::Ipv4))?;
            let destination = child_operation(receiver.bind(InetEndpoint {
                port: 45670,
                ..SERVER
            }))?;
            let sender = child_operation(net.try_new_udp(AddressFamily::Ipv4))?;
            child_operation(sender.connect(Some(destination)))?;
            let entered = Arc::try_new(AtomicBool::new(false))
                .map_err(|_| TestError("UDP gate observation OOM"))?;
            let held = Arc::clone(&entered);
            let gate_release = Arc::clone(&release);
            let gate = crate::ktest::spawn_result_on(0, move |token| {
                let irq = crate::sync::IrqSaveGuard::new();
                held.store(true, Ordering::Release);
                let end = time::now()
                    .checked_add(time::Duration::from_secs(5))
                    .unwrap();
                while !gate_release.load(Ordering::Acquire) && !token.stop_requested() {
                    if end <= time::now() {
                        return Err(TestError("UDP lease IRQ gate release deadline"));
                    }
                    ::core::hint::spin_loop();
                }
                drop(irq);
                Ok(0)
            })?;
            observe(
                || entered.load(Ordering::Acquire),
                "UDP Work CPU did not enter actual IRQ gate",
            )?;

            let old = child_operation(sender.local_address())?;
            let old_input = b"old prepared message";
            child_operation(sender.set_send_budget(old_input.len()))?;
            if child_operation(sender.send(old_input, None))? != IoProgress::Ready(old_input.len())
            {
                return Err(TestError("UDP old record not accepted"));
            }
            if child_operation(sender.poll(None))?.contains(PollMask::POLLOUT) {
                return Err(TestError(
                    "UDP accepted bytes did not exhaust real TX budget",
                ));
            }
            child_operation(sender.set_send_budget(64))?;
            if !child_operation(sender.poll(None))?.contains(PollMask::POLLOUT) {
                return Err(TestError("UDP raised TX budget did not restore OUT"));
            }
            child_operation(sender.connect(None))?;
            if child_operation(sender.local_address())?
                != (InetEndpoint {
                    address: AddressFamily::Ipv4.unspecified(),
                    port: 0,
                })
            {
                return Err(TestError(
                    "UDP automatic disconnect retained current binding",
                ));
            }
            let contender = child_operation(net.try_new_udp(AddressFamily::Ipv4))?;
            if contender.bind(old) != Err(FsError::AddressInUse) {
                return Err(TestError(
                    "accepted old UDP record lost binding before dispatch",
                ));
            }

            let input = b"successor prepared datagram";
            if child_operation(sender.send(input, Some(destination)))?
                != IoProgress::Ready(input.len())
            {
                return Err(TestError("UDP successor record not accepted"));
            }
            let new = child_operation(sender.local_address())?;
            if old.port == new.port || sender.peer_address() != Err(FsError::NotConnected) {
                return Err(TestError(
                    "UDP successor changed disconnect or reused live old port",
                ));
            }
            let third = child_operation(net.try_new_udp(AddressFamily::Ipv4))?;
            if third.bind(new) != Err(FsError::AddressInUse) {
                return Err(TestError("UDP successor binding not retained"));
            }

            release.store(true, Ordering::Release);
            if gate.join()? != 0 {
                return Err(TestError("UDP lease IRQ gate child failed"));
            }
            for (expected, source) in [(&old_input[..], old), (&input[..], new)] {
                observe(
                    || {
                        receiver
                            .poll(None)
                            .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
                    },
                    "accepted UDP record not received",
                )?;
                let mut bytes = [0u8; 64];
                let record =
                    child_operation(receiver.receive(&mut ReadBuffer::kernel(&mut bytes), false))?
                        .ok_or(TestError("accepted UDP record disappeared"))?;
                let expected_source = SocketAddress::Inet(InetEndpoint {
                    address: SERVER.address,
                    port: source.port,
                });
                if record.source != Some(expected_source)
                    || record.original_len != expected.len()
                    || &bytes[..record.copied] != expected
                {
                    return Err(TestError("UDP old or successor source and bytes differ"));
                }
            }

            let mut bound = false;
            let mut failure = None;
            observe(
                || {
                    match contender.bind(old) {
                        Ok(local) => bound = local == old,
                        Err(FsError::AddressInUse) => {}
                        Err(error) => failure = Some(error),
                    }
                    bound || failure.is_some()
                },
                "old UDP lease did not retire after actual delivery",
            )?;
            if let Some(error) = failure {
                child_operation::<()>(Err(error))?;
            }
            if third.bind(new) != Err(FsError::AddressInUse) {
                return Err(TestError("old UDP lease final put unbound successor"));
            }

            release.store(false, Ordering::Release);
            entered.store(false, Ordering::Release);
            let held = Arc::clone(&entered);
            let gate_release = Arc::clone(&release);
            let gate = crate::ktest::spawn_result_on(0, move |token| {
                let irq = crate::sync::IrqSaveGuard::new();
                held.store(true, Ordering::Release);
                let end = time::now()
                    .checked_add(time::Duration::from_secs(5))
                    .unwrap();
                while !gate_release.load(Ordering::Acquire) && !token.stop_requested() {
                    if end <= time::now() {
                        return Err(TestError("UDP close IRQ gate release deadline"));
                    }
                    ::core::hint::spin_loop();
                }
                drop(irq);
                Ok(0)
            })?;
            observe(
                || entered.load(Ordering::Acquire),
                "UDP close Work CPU did not enter actual IRQ gate",
            )?;
            let tail = b"accepted before close";
            if child_operation(sender.send(tail, Some(destination)))?
                != IoProgress::Ready(tail.len())
            {
                return Err(TestError("UDP pre-close record not accepted"));
            }
            sender.close();
            release.store(true, Ordering::Release);
            if gate.join()? != 0 {
                return Err(TestError("UDP close IRQ gate child failed"));
            }

            observe(
                || {
                    receiver
                        .poll(None)
                        .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
                },
                "UDP close discarded accepted raw TX record",
            )?;
            let mut bytes = [0u8; 64];
            let record =
                child_operation(receiver.receive(&mut ReadBuffer::kernel(&mut bytes), false))?
                    .ok_or(TestError("UDP close delivery disappeared"))?;
            if record.source
                != Some(SocketAddress::Inet(InetEndpoint {
                    address: SERVER.address,
                    port: new.port,
                }))
                || record.original_len != tail.len()
                || &bytes[..record.copied] != tail
            {
                return Err(TestError("UDP close delivery source or full bytes differ"));
            }

            receiver.close();
            contender.close();
            third.close();
            drop((sender, receiver, contender, third));
            socket_retired(&net)?;
            Ok(0)
        })?;
        context.expect_eq(controller.join()?, 0);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }
}
