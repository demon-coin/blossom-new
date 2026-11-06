/// UDP 端点冷激活、接收分派表扩容与端口绑定租约（`BindingLease`）生命周期内核测试套件。
///
/// ## 1. 模块职责与架构定位
/// 本模块位于 `ktest::network` 测试子系统中，是面向 `crate::net::udp::UdpEndpoint` 与
/// `crate::net::table::EndpointTables` 的集成测试套件。上游由 `ktest` 测试运行器在独立内核线程中
/// 挂载 `StackOwner` 夹具驱动，下游直接测试 UDP 套接字端点、端口绑定租约以及底层 loopback 工作队列分派。
///
/// ## 2. 核心机制与状态演进
/// - **冷端点延迟分配（Lazy Buffer Allocation）**：新建的 `UdpEndpoint` 在首次执行 `bind`、`connect`
///   或带自动绑定的 `send` 之前处于“冷状态”（不分配底层 smoltcp 收发环形缓冲区与元数据槽位，仅保存
///   套接字预算、超时、`SO_REUSEADDR`/`IPV6_V6ONLY` 等配置项）。首次激活若遇内存不足（`ENOMEM`），
///   必须保持原子回滚，不残留半初始化缓冲区或地址表索引。
/// - **UDP 接收键（Receive Key）分派与原地重发表**：`EndpointTables` 维护从本地绑定地址和可选对端地址
///   到 `UdpEndpoint` 的接收路由表。当同一套接字调用 `connect` 更新对端过滤条件时，已在表中的槽位会
///   原地更新而无需重新分配堆内存。
/// - **在途报文端口租约（`BindingLease`）生命周期**：
///   ```text
///   [connect 自动分配临时端口 old]
///          │
///          ▼
///   [send 入队报文 1 (携带 Arc<BindingLease(old)>)] ──(CPU0 关中断闸门暂停软中断分派)
///          │
///          ▼
///   [connect(AF_UNSPEC) 断开连接] ──► 端点当前绑定清空，但报文 1 仍持有 old 租约强引用，old 端口仍不可被抢占
///          │
///          ▼
///   [sendto 入队报文 2] ────────────► 自动分配新临时端口 new，与在途的 old 端口互不冲突
///          │
///          ▼
///   [放行 CPU0 软中断分派] ─────────► 报文 1 与报文 2 按各自源端口送达；报文 1 消费后释放 old 租约，old 端口回归可用
///   ```
///
/// ## 3. 锁与并发顺序说明
/// - 测试在双核场景下使用 `IrqSaveGuard` 在 CPU 0（网络工作队列所在核心）制造可观测的关中断闸门窗口，
///   并在 CPU 1 控制线程中推进套接字状态变更；控制线程通过 `AtomicBool`（`Acquire`/`Release`）与
///   CPU 0 同步，禁止在持有 `IrqSaveGuard` 期间睡眠或等待需要本核软中断推进的条件。
/// UDP 端点延迟激活、接收路由键扩容与在途端口租约（`BindingLease`）并发交错测试套件。
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

    /// 等待 `receiver` 变为可读并接收单条 UDP 数据报，校验报文载荷、源地址与标志位完全匹配，
    /// 同时确认接收队列中无多余残留报文。
    ///
    /// # 参数
    /// - `receiver`：已绑定的接收端 UDP 端点。
    /// - `expected`：期望收到的完整字节载荷。
    /// - `source`：期望的发送方 IPv4/IPv6 地址与端口。
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

    /// 测试目标：验证大量未激活（冷状态）的 UDP 端点仅占用控制结构而不提前分配收发数据缓冲区，
    /// 且套接字配置项读写正常、最后一个 `Arc` 强引用释放前端点槽位保持存活。
    ///
    /// 场景步骤：
    /// 1. 批量创建 `count` 个 IPv6 UDP 端点，逐一配置收发预算、`SO_REUSEADDR`、`IPV6_V6ONLY` 与收发超时；
    /// 2. 检查每个端点的底层缓冲区尚未分配（`ktest_buffer_storage` 为空），且配置值与未连接状态正确；
    /// 3. 克隆最后一个端点的强引用 `alias`，随后关闭并丢弃整个端点数组；
    /// 4. 验证其余 `count - 1` 个端点全部完成回收，仅剩 `alias` 对应的 1 个存活端点，最后丢弃 `alias` 并确认归零。
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
        // Step 1: 批量创建冷状态 UDP 端点并设置各类套接字选项，验证未触发底层收发缓冲区分配。
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

        // Step 2: 保留最后一个端点的额外强引用，关闭并释放其余所有端点，验证非末次引用不会提前销毁存活对象。
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

        // Step 3: 释放最后一个强引用，确认该端点及网络栈全局活跃计数彻底归零。
        drop(alias);
        observe(
            || weak.upgrade().is_none(),
            "cold UDP last alias retained endpoint",
        )?;
        socket_retired(&fixture.net)
    }

    /// 测试目标：验证 UDP 端点在首次激活（通过 `bind`、`connect` 或未绑定直接 `send`）遭遇内存耗尽（`ENOMEM`）时具备原子回滚保证，且内存压力解除后同一端点可正常激活并收发完整报文。
    ///
    /// 场景步骤：
    /// 1. 创建接收端并绑定 `SERVER`，创建待激活的发送端并配置收发预算、地址复用及超时参数；
    /// 2. 借助 `HeapPressure::exhaust` 耗尽内核堆与物理页，触发指定的首次激活路径（`bind` / `connect` / `send`）；
    /// 3. 验证激活返回 `FsError::NoMemory`，且全局 UDP 表容量、端点内部缓冲区状态、本地绑定、对端地址及配置项均保持未激活前的原状；
    /// 4. 释放内存压力后重新执行激活与报文发送，验证接收端收到完整的 513 字节载荷且源地址正确。
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
        // Step 1: 准备已绑定的接收端与处于冷状态的发送端，预设自定义套接字参数。
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

        // Step 2: 耗尽堆内存后执行首次激活，验证操作原子失败且不残留半初始化状态。
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

        // Step 3: 释放内存压力后在同一发送端对象上重试激活与发送，确认功能与原配置完好恢复。
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

    /// 测试目标：验证当 UDP 接收路由表扩展至 129 个独立绑定端口（跨越初始容量边界）时，每个接收端点都能准确分派并接收各自对应的完整数据报。
    ///
    /// 场景步骤：
    /// 1. 创建 129 个 IPv4 UDP 接收端点并分别绑定到连续端口 `46000..46129`；
    /// 2. 使用单个发送端点依次向每个接收端点的绑定地址发送具有唯一字节图案的 513 字节数据报；
    /// 3. 逐个验证接收端点收到的报文内容与源端口准确无误，最后关闭所有端点并确认资源全部回收。
    #[test]
    #[ktest(tags("network-growth"))]
    fn udp_129_receive_keys_deliver_each_full_record(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建并绑定 129 个独立端口的 UDP 接收端点，触发接收路由表扩容。
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

        // Step 2: 向每个接收端口发送唯一图案的数据报并逐一校验完整接收。
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

        // Step 3: 检查 UDP 路由表计数（129 个接收端 + 1 个自动绑定发送端），随后关闭并回收全部端点。
        context.expect_eq(fixture.net.tables().ktest_udp_storage()[2].0, 129 + 1);
        sender.close();
        for receiver in &receivers {
            receiver.close();
        }
        drop((receivers, sender));
        socket_retired(&fixture.net)
    }

    /// 测试目标：验证当 UDP 接收键存储表满载时，已注册端点通过 `connect` 调整对端匹配优先级无需新分配内存；而新端点的插入在 `ENOMEM` 下原子失败且不破坏已有租约与收发路由。
    ///
    /// 场景步骤：
    /// 1. 创建开启 `SO_REUSEADDR` 的 `first` 与 `candidate` 端点绑定到 `SERVER`，随后将 `candidate` 从接收表临时撤下，再用新端点填满接收键表剩余容量；
    /// 2. 耗尽堆内存后，让已在表中的 `first` 调用 `connect(Some(source))` 提升为精确四元组匹配，验证原地更新成功且路由命中 `first`；
    /// 3. 在同样内存压力下尝试重新发布 `candidate`，验证因扩容失败返回 `FsError::NoMemory` 且表状态不变；
    /// 4. 释放内存压力后验证 `first`、`candidate` 及所有复用端点依次更新优先级并准确接收各自数据报，且未开启复用的竞争者绑定同一地址仍返回 `AddressInUse`。
    #[test]
    #[ktest(tags("network-growth"))]
    fn full_udp_key_storage_updates_without_allocation_and_growth_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 构造共享同一 SERVER 端口的多个复用接收端点，将 UDP 接收键表恰好填满至当前容量上限。
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

        // Step 2: 在堆内存耗尽状态下验证已入表端点可无分配更新连接过滤条件，而新键插入返回 NoMemory。
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

        // Step 3: 解除内存压力后验证原租约、恢复注册的候选端点以及其余所有复用端点均可正常路由并接收报文。
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

        // Step 4: 验证未开启地址复用的竞争端点仍被拒绝绑定，随后关闭并清理全部端点。
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

    /// 测试目标：验证 UDP 端点在发送报文入队后执行 `connect(None)`（`AF_UNSPEC`）断开连接或 `close()` 关闭端点时，已入队的在途报文仍持有旧端口绑定租约（`BindingLease`）直至软中断完成投递，期间后继报文可分配新端口且新旧租约互不干扰。
    ///
    /// 场景步骤：
    /// 1. 在 CPU 0 上启动关中断闸门暂停网络工作队列分派，在 CPU 1 上让 `sender` 连接 `destination` 并发送 `old_input` 报文（此时报文携带旧端口 `old` 的租约停留在发送队列）；
    /// 2. 验证发送预算耗尽与调高预算后 `POLLOUT` 恢复，接着调用 `sender.connect(None)` 断开连接，确认 `sender` 当前本地绑定已清空，但 `contender` 尝试绑定 `old` 端口仍因在途租约未释放而返回 `AddressInUse`；
    /// 3. 使用 `sender.send(..., Some(destination))` 发送后继报文 `input`，确认自动分配了不同于 `old` 的新临时端口 `new`；
    /// 4. 释放 CPU 0 关中断闸门，等待工作队列将新旧两条报文依次投递给 `receiver`，验证两条报文的载荷及源端口（分别为 `old` 与 `new`）完全匹配；
    /// 5. 验证旧报文投递完成后 `old` 端口租约释放、`contender` 可成功绑定 `old`，而 `new` 端口仍被 `sender` 占用；
    /// 6. 再次开启 CPU 0 关中断闸门，发送 `tail` 报文后立即调用 `sender.close()`，放行闸门后确认 `tail` 依然成功送达 `receiver`。
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
            // Step 1: 在 CPU 0 上建立关中断闸门，暂停软中断工作队列，确保后续发送的报文驻留在待分派队列中。
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

            // Step 2: 发送首条报文并验证发送缓冲区预算扣减与恢复，随后通过 connect(None) 解除当前连接。
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
            // Step 3: 验证虽然 sender 当前绑定已重置，但在途首条报文持有的旧端口租约仍阻止其他套接字抢占该端口。
            let contender = child_operation(net.try_new_udp(AddressFamily::Ipv4))?;
            if contender.bind(old) != Err(FsError::AddressInUse) {
                return Err(TestError(
                    "accepted old UDP record lost binding before dispatch",
                ));
            }

            // Step 4: 在未连接状态下向目标地址发送后继报文，验证自动分配了新的临时端口且新端口同样受租约保护。
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

            // Step 5: 放行 CPU 0 关中断闸门，验证新旧两条报文均按各自源端口完整送达接收端。
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

            // Step 6: 首条报文投递完毕后，验证旧端口租约已归还（contender 可成功绑定 old），而后继新端口租约依然存活。
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

            // Step 7: 再次开启关中断闸门，验证在报文入队后立即调用 close() 不会丢弃已接纳的在途发送报文。
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
