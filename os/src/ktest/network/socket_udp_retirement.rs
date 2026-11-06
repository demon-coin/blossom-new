/// UDP 异步发送内存耗尽（`ENOMEM`）、后继记录保活与端点彻底回收（Retirement）内核测试套件。
///
/// ## 1. 模块职责与架构定位
/// 本模块位于 `ktest::network` 测试子系统中，专注于验证 `SocketFile` 与 `UdpEndpoint` 在网络工作队列
/// （WorkQueue）异步分派阶段遭遇物理页与堆内存双双耗尽（`ENOMEM`）时的错误处理、资源回收与恢复能力。
///
/// ## 2. 核心状态流转与并发模型
/// ```text
///   CPU 1 (测试控制线程)                              CPU 0 (网络工作队列 WorkQueue 所在核)
/// ──────────────────────────────────────────────    ──────────────────────────────────────────────
/// 1. 预分配关中断闸门任务 (prepare_udp_oom_gate)
/// 2. 耗尽物理页与堆内存 (HeapPressure::exhaust)
/// 3. 发布闸门任务到 CPU 0 (publish) ──────────────► 进入 IrqSaveGuard 关中断自旋，暂停软中断执行
/// 4. 写入 8193B 大报文 + 零长后继记录入队
/// 5. 置位 release 放行 CPU 0 ─────────────────────► 退出关中断，工作队列运行：
///                                                     - 队首 8193B 报文分配回环包失败 -> 记录 POLLERR/NoMemory
///                                                     - 跨批次继续分派 31 条零长后继记录 -> 成功送达接收端
///                                                     - 若最后一个 SocketFile 已 Drop -> 调度异步 reaper 回收端点与端口绑定
/// ```
///
/// ## 3. 锁与并发顺序说明
/// - `HeapPressure::exhaust` 内部会触发跨核 SLUB 刷回 IPI，因此必须在 CPU 0 关中断闸门启动**之前**完成内存耗尽；
///   否则 CPU 0 关中断自旋将导致同步 IPI 死锁。
/// - 释放最后一个 `SocketFile` 强引用时，其 `Drop` 实现会在不分配新堆内存的前提下向工作队列提交异步清理任务（reaper），
///   确保即使在极端内存压力下也不会泄漏端点槽位或永久占用本地端口绑定。
/// UDP 异步分派内存耗尽（`ENOMEM`）、后继小报文跨批投递与末端点回收测试套件。
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

    /// 测试目标：验证当 UDP 发送队列在后台软中断分派阶段遭遇真实内存耗尽（`ENOMEM`），且调用方随即消费错误并释放最后一个 `SocketFile`（打开文件描述）引用时，无需后续手动追加工作任务，后台异步回收流程（reaper）即可自动销毁端点并归还已绑定的端口。
    ///
    /// 场景步骤：
    /// 1. 创建 IPv4 UDP `SocketFile` 并连接 `SERVER`，设置发送预算为 8193 字节；
    /// 2. 预分配 CPU 0 关中断闸门任务，调用 `HeapPressure::exhaust` 彻底耗尽物理页与堆内存，随后启动 CPU 0 关中断闸门暂停网络工作队列；
    /// 3. 将 8193 字节载荷写入 `endpoint` 发送队列，确认发送预算已扣减（`POLLOUT` 清零）；
    /// 4. 放行 CPU 0 关中断闸门并刷新工作队列，验证软中断因内存不足无法构造回环报文，向端点报告 `POLLERR` 与 `FsError::NoMemory`；
    /// 5. 在内存压力仍然存在的情况下读取 `take_error()`、丢弃 `endpoint` 与 `sender` 的全部强引用，并连续刷新工作队列以执行关闭回调与异步 reaper；
    /// 6. 释放内存压力，验证无需对已关闭端点发起任何额外操作，其弱引用 `endpoint_weak` 已失效、全局活跃端点归零，且原本地端口 `local` 可被重新预约绑定。
    #[test]
    #[ktest(min_cpus = 2)]
    fn udp_real_no_memory_last_ofd_retires_without_new_work(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let net = Arc::clone(&fixture.net);
        let release = Arc::clone(&fixture.release);

        let controller = crate::ktest::spawn_result_on(1, move |_| {
            // Step 1: 创建并连接 UDP 套接字文件，配置与测试报文等长的发送缓冲区预算。
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

            // Step 2: 准备 CPU 0 关中断闸门并耗尽系统物理页与堆内存。
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
            // exhaust 会发送同步 SLUB IPI；此时关中断闸门尚未发布，预分配的任务栈已就绪，CPU 0 仍可正常响应 IPI。
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

                // Step 3: 在 CPU 0 关中断期间向端点写入大报文，直接操作底层 endpoint 避免文件层额外分配，确认报文入队并耗尽发送预算。
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
                // Step 4: 放行 CPU 0 关中断闸门并刷新工作队列，验证后台分派因内存耗尽触发 POLLERR 与 NoMemory 错误。
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

                // Step 5: 保持内存压力不变，冲洗掉 take_error 触发的队列任务后，释放最后一个 SocketFile 强引用并等待异步 reaper 完成。
                net.work_queue().flush();
                drop(endpoint);
                drop(sender);
                net.work_queue().flush();
                // 关闭回调中唤醒的异步 reaper 任务排在下一轮工作队列批次，再次调用 flush 等待其彻底执行完毕并退出。
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

            // Step 6: 释放内存压力后，仅通过弱引用与表状态观察端点已被彻底销毁，并验证原绑定端口已归还可用。
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

    /// 测试目标：验证当队首大报文在后台分派时因内存耗尽（`ENOMEM`）被丢弃后，同一发送队列中排在后面的 31 条零长度后继报文（跨越单次 16 条的批处理配额）仍能携带原源端口租约完整送达接收端；且内存压力解除后，失败的大报文不会残留重发，新连接与断开重发均正常运作。
    ///
    /// 场景步骤：
    /// 1. 创建绑定在 `45682` 端口的 `receiver` 与连接到该地址的 `sender`，记录 `sender` 的初始源端口 `old`；
    /// 2. 预分配 CPU 0 关中断闸门并耗尽物理页与堆内存，随后启动闸门暂停后台工作队列；
    /// 3. 在 `sender` 中依次入队 1 条 8193 字节大报文和 31 条零长度报文（共占满 32 个元数据槽位，使 `POLLOUT` 变为不可写）；
    /// 4. 放行 CPU 0 关中断闸门并刷新工作队列，验证队首 8193 字节大报文因分配失败上报 `POLLERR` 与 `FsError::NoMemory`；
    /// 5. 再次刷新工作队列以跨越单次 16 条的分派批次上限，验证后续 31 条零长度报文无需额外载荷分配即可全部送达 `receiver`，且每条报文的源地址均为 `old`，消费后 `sender` 恢复 `POLLOUT`；
    /// 6. 释放内存压力，确认 `receiver` 队列已空（失败的大报文未混入），接着在原连接上发送正常报文验证送达；
    /// 7. 调用 `connect(Unspecified)` 断开连接并通过 `sendto` 发送新报文，验证自动分配了新端口 `new`，且旧端口 `old` 的租约已彻底归还、新端口 `new` 仍受保护。
    #[test]
    #[ktest(min_cpus = 2)]
    fn udp_real_no_memory_preserves_successor_records_sources_and_leases(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let net = Arc::clone(&fixture.net);
        let release = Arc::clone(&fixture.release);

        let controller = crate::ktest::spawn_result_on(1, move |_| {
            // Step 1: 建立收发端 UDP 套接字并完成连接，获取发送端初始分配的本地端口 old。
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

            // Step 2: 准备内存压力环境与 CPU 0 关中断闸门，耗尽堆内存后暂停后台工作队列。
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

                // Step 3: 入队 1 条 8193 字节大报文与 31 条零长度报文，占满全部 32 个发送元数据槽位。
                if child_operation(endpoint.send(&payload, None))?
                    != IoProgress::Ready(payload.len())
                {
                    return Err(TestError("UDP successor OOM message was not accepted"));
                }
                // 发送队列元数据容量上限为 32；紧随大报文之后的 31 条零长度记录跨越了单批 16 条的处理配额，且不占用额外载荷空间。
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
                // Step 4: 放行 CPU 0 关中断闸门，验证队首大报文分派失败并上报 POLLERR 与 NoMemory。
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

                // Step 5: 刷新工作队列以驱动跨批次的后继分派，逐条等待接收端可读并验证全部 31 条零长度报文均携带源地址 old 完整送达。
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

            // Step 6: 解除内存压力，验证此前因 OOM 失败的队首大报文已被彻底丢弃，不会在内存恢复后幽灵送达。
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

            // Step 7: 在已恢复的原连接上发送正常非空报文，验证载荷与源地址 old 正常投递。
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

            // Step 8: 断开原连接后向目标地址发送新报文，验证自动绑定了新端口 new，且旧端口 old 已被释放归还、新端口 new 仍处于占用状态。
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
            // 未连接状态下调用 sendto 时端点本地绑定地址保持通配（0.0.0.0），但实际发出的报文源 IP 由回环路由填充，源端口来自新分配的绑定租约。
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
