// UDP 真实分配错误、后继记录及回调全尾退休共用原压力停点协议。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-socket", "network-udp-retirement"), timeout_ms = 45000)]
pub(crate) mod socket_udp_retirement {
    use ::core::sync::atomic::{AtomicBool, Ordering};
    use ::ktest::{TestContext, TestError, TestResult};
    use alloc::sync::Arc;

    use super::owner::StackOwner;
    use super::socket_helpers::prepare_udp_oom_gate;
    use super::support::{child_operation, observe, socket_retired, SERVER};
    use crate::fs::{FileIoContext, FsError, PollMask, Pollable, ReadBuffer, WriteBuffer};
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::net::address::{AddressFamily, InetEndpoint, SocketAddress};
    use crate::net::socket::{SocketFile, MSG_DONTWAIT};
    use crate::net::IoProgress;

    /// 真实发送准备 OOM 后结束末 SocketFile，放压不再提交工作，原 reaper 须归还节点和绑定。
    #[test]
    #[ktest(min_cpus = 2)]
    fn udp_real_no_memory_last_ofd_retires_without_new_work(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let net = Arc::clone(&fixture.net);
        let release = Arc::clone(&fixture.release);

        let controller = crate::ktest::spawn_result_on(1, move |_| {
            let sender = child_operation(SocketFile::try_new_udp(
                net.clone(),
                AddressFamily::Ipv4,
                0,
                0,
                0,
            ))?;
            child_operation(sender.connect(SocketAddress::Inet(SERVER), FileIoContext::Kernel))?;
            let endpoint = net
                .tables()
                .udp_by_id(sender.id())
                .ok_or(TestError("UDP OOM file endpoint is not published"))?;
            let local = child_operation(endpoint.local_address())?;
            let id = sender.id();
            let file_weak = Arc::downgrade(&sender);
            let endpoint_weak = Arc::downgrade(&endpoint);
            let payload = [0x97u8; 8193];
            child_operation(endpoint.set_send_budget(payload.len()))?;

            let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
            let entered = Arc::try_new(AtomicBool::new(false))
                .map_err(|_| TestError("UDP OOM gate observation allocation failed"))?;
            let finished = Arc::try_new(AtomicBool::new(false))
                .map_err(|_| TestError("UDP OOM gate lifetime allocation failed"))?;
            let prepared_gate = prepare_udp_oom_gate(
                Arc::clone(&entered),
                Arc::clone(&release),
                Arc::clone(&finished),
            )?;
            net.work_queue().flush();
            // exhaust 包含同步 SLUB IPI；停点尚未发表，预备任务栈已占有，CPU0 仍可服务 IPI。
            let evidence = pressure.exhaust();
            crate::ktest::report::diagnostic(format_args!(
                "UDP OOM case=last-ofd evidence={:?}",
                evidence
            ));
            if evidence != (true, true) {
                return Err(TestError("UDP OOM did not exhaust real pages and heap"));
            }
            let gate = prepared_gate.publish()?;

            let result = (|| -> TestResult {
                observe(
                    || entered.load(Ordering::Acquire),
                    "UDP OOM Work CPU did not enter IRQ gate",
                )?;

                // OFD 保有真实端点；直接提交预备字节，避免在压力中另做文件层 copyin 分配。
                if child_operation(endpoint.send(&payload, None))?
                    != IoProgress::Ready(payload.len())
                {
                    return Err(TestError("UDP OOM message was not actually accepted"));
                }
                if child_operation(endpoint.poll(None))?.contains(PollMask::POLLOUT) {
                    return Err(TestError(
                        "UDP OOM accepted message did not consume TX budget",
                    ));
                }
                release.store(true, Ordering::Release);
                net.work_queue().flush();
                if !child_operation(endpoint.poll(None))?.contains(PollMask::POLLERR) {
                    return Err(TestError("UDP OOM dispatch did not report POLLERR"));
                }
                let error = sender.take_error();
                crate::ktest::report::diagnostic(format_args!(
                    "UDP OOM case=last-ofd dispatch-error={:?}",
                    error
                ));
                if error != Some(FsError::NoMemory) {
                    return Err(TestError("UDP dispatch did not fail with real NoMemory"));
                }

                // take_error 的 Modify guard 也安排原 Work；这份票必须仍在压力下结束。
                net.work_queue().flush();
                drop(endpoint);
                drop(sender);
                net.work_queue().flush();
                // close 回调中提交的 reaper 属于后继颜色，再等一次覆盖其真实停止尾部。
                net.work_queue().flush();
                if file_weak.upgrade().is_some() || gate.try_join().is_some() {
                    return Err(TestError(
                        "UDP OOM OFD or gate lifetime did not end as required",
                    ));
                }
                Ok(())
            })();

            drop(pressure);
            release.store(true, Ordering::Release);
            finished.store(true, Ordering::Release);
            let joined = gate.join();
            result?;
            if joined? != 0 {
                return Err(TestError("UDP OOM IRQ gate child failed"));
            }

            // 放压后只观察末端点，不升级它调用 Modify，也不发送或手动安排工作。
            observe(
                || endpoint_weak.upgrade().is_none() && net.ktest_live_endpoints() == 0,
                "UDP terminal NoMemory stranded the last OFD endpoint or slot",
            )?;

            let binding = child_operation(net.tables().reserve_binding(
                &net,
                id,
                crate::net::table::TransportProtocol::Udp,
                local,
                false,
                false,
            ))?;
            if binding.local() != local || net.is_stopping() {
                return Err(TestError(
                    "UDP terminal NoMemory did not return the live-root binding",
                ));
            }
            drop((binding, file_weak, endpoint_weak));
            Ok(0)
        })?;

        context.expect_eq(controller.join()?, 0);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }

    /// OOM 结束队首大消息，超过原批预算的零长后继保同条 source，放压后正常全文不夹带失败消息。
    #[test]
    #[ktest(min_cpus = 2)]
    fn udp_real_no_memory_preserves_successor_records_sources_and_leases(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let net = Arc::clone(&fixture.net);
        let release = Arc::clone(&fixture.release);

        let controller = crate::ktest::spawn_result_on(1, move |_| {
            let receiver = child_operation(SocketFile::try_new_udp(
                net.clone(),
                AddressFamily::Ipv4,
                0,
                0,
                0,
            ))?;
            let destination = InetEndpoint {
                port: 45682,
                ..SERVER
            };
            child_operation(receiver.bind(SocketAddress::Inet(destination)))?;
            let sender = child_operation(SocketFile::try_new_udp(
                net.clone(),
                AddressFamily::Ipv4,
                0,
                0,
                0,
            ))?;
            child_operation(
                sender.connect(SocketAddress::Inet(destination), FileIoContext::Kernel),
            )?;
            let endpoint = net
                .tables()
                .udp_by_id(sender.id())
                .ok_or(TestError("UDP successor file endpoint is not published"))?;
            let old = child_operation(endpoint.local_address())?;
            let payload = [0x69u8; 8193];

            let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
            let entered = Arc::try_new(AtomicBool::new(false))
                .map_err(|_| TestError("UDP successor gate observation allocation failed"))?;
            let finished = Arc::try_new(AtomicBool::new(false))
                .map_err(|_| TestError("UDP successor gate lifetime allocation failed"))?;
            let prepared_gate = prepare_udp_oom_gate(
                Arc::clone(&entered),
                Arc::clone(&release),
                Arc::clone(&finished),
            )?;
            net.work_queue().flush();
            let evidence = pressure.exhaust();
            crate::ktest::report::diagnostic(format_args!(
                "UDP OOM case=successor evidence={:?}",
                evidence
            ));
            if evidence != (true, true) {
                return Err(TestError(
                    "UDP successor OOM did not exhaust real pages and heap",
                ));
            }
            let gate = prepared_gate.publish()?;

            let result = (|| -> TestResult {
                observe(
                    || entered.load(Ordering::Acquire),
                    "UDP successor Work CPU did not enter IRQ gate",
                )?;

                if child_operation(endpoint.send(&payload, None))?
                    != IoProgress::Ready(payload.len())
                {
                    return Err(TestError("UDP successor OOM message was not accepted"));
                }
                // 原 metadata 容量为32；大消息之后31条零长记录跨过16条批预算，不需新 payload 后备。
                for _ in 0..31 {
                    if child_operation(endpoint.send(&[], None))? != IoProgress::Ready(0) {
                        return Err(TestError("UDP zero successor was not actually accepted"));
                    }
                }
                if child_operation(endpoint.poll(None))?.contains(PollMask::POLLOUT) {
                    return Err(TestError(
                        "UDP successor records did not consume metadata capacity",
                    ));
                }
                release.store(true, Ordering::Release);
                net.work_queue().flush();
                if !child_operation(endpoint.poll(None))?.contains(PollMask::POLLERR) {
                    return Err(TestError(
                        "UDP successor OOM dispatch did not report POLLERR",
                    ));
                }
                let error = sender.take_error();
                crate::ktest::report::diagnostic(format_args!(
                    "UDP OOM case=successor dispatch-error={:?}",
                    error
                ));
                if error != Some(FsError::NoMemory) {
                    return Err(TestError(
                        "UDP successor dispatch did not fail with real NoMemory",
                    ));
                }

                // 等查询票及原批预算后的后继票；接收逐条等待真正 IN，不把 raw 消费冒充交付。
                net.work_queue().flush();
                for _ in 0..31 {
                    observe(
                        || {
                            receiver
                                .poll(None)
                                .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
                        },
                        "UDP zero successor did not reach the actual receiver",
                    )?;
                    let record = child_operation(receiver.receive(
                        &mut ReadBuffer::kernel(&mut []),
                        MSG_DONTWAIT,
                        FileIoContext::Kernel,
                    ))?;
                    if record.source != Some(SocketAddress::Inet(old))
                        || record.original_len != 0
                        || record.copied != 0
                        || record.flags != 0
                    {
                        return Err(TestError(
                            "UDP OOM successor record and source were mispaired",
                        ));
                    }
                }
                net.work_queue().flush();
                if !child_operation(endpoint.poll(None))?.contains(PollMask::POLLOUT)
                    || gate.try_join().is_some()
                {
                    return Err(TestError(
                        "UDP OOM successor budget or gate lifetime did not recover",
                    ));
                }
                Ok(())
            })();

            drop(pressure);
            release.store(true, Ordering::Release);
            finished.store(true, Ordering::Release);
            let joined = gate.join();
            result?;
            if joined? != 0 {
                return Err(TestError("UDP successor IRQ gate child failed"));
            }

            let mut bytes = [0u8; 64];
            if receiver.receive(
                &mut ReadBuffer::kernel(&mut bytes),
                MSG_DONTWAIT,
                FileIoContext::Kernel,
            ) != Err(FsError::WouldBlock)
            {
                return Err(TestError(
                    "UDP failed OOM message was delivered after pressure release",
                ));
            }

            {
                let input = b"normal after OOM";
                let sent = child_operation(sender.send(
                    &mut WriteBuffer::kernel(input),
                    None,
                    MSG_DONTWAIT,
                    FileIoContext::Kernel,
                ))?;
                if sent != input.len() {
                    return Err(TestError("UDP post-OOM full message was not accepted"));
                }
                observe(
                    || {
                        receiver
                            .poll(None)
                            .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
                    },
                    "UDP post-OOM full message was not actually delivered",
                )?;
                let record = child_operation(receiver.receive(
                    &mut ReadBuffer::kernel(&mut bytes),
                    MSG_DONTWAIT,
                    FileIoContext::Kernel,
                ))?;
                if record.source != Some(SocketAddress::Inet(old))
                    || record.original_len != input.len()
                    || record.copied != input.len()
                    || &bytes[..record.copied] != input
                {
                    return Err(TestError(
                        "UDP post-OOM full bytes or source were mispaired",
                    ));
                }
            }

            child_operation(sender.connect(SocketAddress::Unspecified, FileIoContext::Kernel))?;
            let input = b"new lease after OOM";
            if child_operation(sender.send(
                &mut WriteBuffer::kernel(input),
                Some(SocketAddress::Inet(destination)),
                MSG_DONTWAIT,
                FileIoContext::Kernel,
            ))? != input.len()
            {
                return Err(TestError("UDP post-OOM new lease message was not accepted"));
            }
            let new = child_operation(endpoint.local_address())?;
            if old.port == new.port {
                return Err(TestError(
                    "UDP post-OOM automatic binding reused the old source",
                ));
            }
            observe(
                || {
                    receiver
                        .poll(None)
                        .is_ok_and(|mask| mask.contains(PollMask::POLLIN))
                },
                "UDP post-OOM new lease message was not actually delivered",
            )?;
            let record = child_operation(receiver.receive(
                &mut ReadBuffer::kernel(&mut bytes),
                MSG_DONTWAIT,
                FileIoContext::Kernel,
            ))?;
            // 未连接 sendto 保通配绑定名；报文源来自本次 lo 路由，端口仍属于同一新租约。
            let expected_source = SocketAddress::Inet(InetEndpoint {
                address: SERVER.address,
                port: new.port,
            });
            if record.source != Some(expected_source)
                || record.original_len != input.len()
                || record.copied != input.len()
                || &bytes[..record.copied] != input
            {
                crate::ktest::report::diagnostic(format_args!(
                    "UDP OOM case=successor new-lease record={:?} expected-source={:?} bytes={:?} expected-bytes={:?}",
                    record, expected_source, &bytes[..record.copied], input,
                ));
                return Err(TestError(
                    "UDP post-OOM new lease bytes or source were mispaired",
                ));
            }

            net.work_queue().flush();
            let binding = child_operation(net.tables().reserve_binding(
                &net,
                endpoint.id(),
                crate::net::table::TransportProtocol::Udp,
                old,
                false,
                false,
            ))?;
            match net.tables().reserve_binding(
                &net,
                endpoint.id(),
                crate::net::table::TransportProtocol::Udp,
                new,
                false,
                false,
            ) {
                Err(FsError::AddressInUse) => {}
                _ => {
                    return Err(TestError(
                        "UDP failed lease final put unbound the successor",
                    ))
                }
            }

            drop((binding, endpoint, sender, receiver));
            socket_retired(&net)?;
            if net.is_stopping() {
                return Err(TestError(
                    "UDP OOM successor required stopping the network root",
                ));
            }
            Ok(0)
        })?;

        context.expect_eq(controller.join()?, 0);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }
}
