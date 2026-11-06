// 后台网络协议栈（`NetStack`）回环转发、端口索引并发与生命周期回归测试套件。
//
// # 模块职责与架构定位
// 本套件由 `os/src/ktest/network.rs` 通过 `include!` 内联引入，使 `#[ktest::suite]` 宏保留原始模块路径与静态注册槽。
// 与无后台线程的 `core.rs` 不同，本套件在完整的 `StackOwner` 夹具下启动带异步工作队列的回环 `NetStack`，
// 端到端验证连接建立、报文出入站、索引表扩容、端口分配、出站背压以及网络栈同步停机语义。
// - **上游调用者**：`ktest` 测试运行器在普通线程上下文（`context = "Thread"`）中驱动本套件。
// - **下游依赖**：`crate::net`（`NetStack`、`NetTables`、`TcpEndpoint`、`UdpEndpoint`、`OwnedPacket`）、
//   `smoltcp::socket::udp` 底层缓冲区、工作队列与堆压力守卫（`HeapPressure`）。
//
// # 核心设计与后台回环转发模型
// 测试夹具 `StackOwner` 维护一个独立的回环 `NetStack` 实例，各协议端点通过异步出站/入站队列与连接级状态机解耦协作：
//
// ```text
//   [测试主线程 / 跨 CPU 子任务]                      [后台工作队列 (WorkQueue)]
//   +----------------------------------+             +----------------------------------+
//   | TcpEndpoint / UdpEndpoint        |             | NetStack ingress Work (CPU0)     |
//   |  - bind(): NetTables 端口租约    |             |  - 消费 Egress 队列 (容量 32 包) |
//   |  - send()/connect(): 编码报文    | ----------> |  - 唤醒阻塞在 Egress 的等待端点  |
//   |    调用 NetStack::try_emit()     | Egress 队列 |  - smoltcp 解析线格式并查表匹配  |
//   |  - close()/abandon(): 移交退休链 |             |  - 投递至目标端点并调度端点 Work |
//   +----------------------------------+             +----------------------------------+
// ```
//
// # 锁获取顺序与并发契约
// - 测试代码在普通可睡眠任务中可短暂获取 `TcpEndpoint::lock()`（连接级 `SleepMutex`）执行收发；
//   持锁期间后台线程推进其他 CPU 上的独立连接不受影响（验证连接级锁隔离）。
// - `full_egress_waits_for_real_capacity_and_resumes` 在 CPU0 子任务中使用 `IrqSaveGuard` 短暂关闭本地中断以暂停 CPU0 上的入站 `Work`；
//   关中断临界区内仅执行原子标志读取与自旋等待，所有内存分配、睡眠与网络调用均留在 CPU1 普通可睡眠控制任务中执行。
// - `StackOwner::quiesce` 会在回收子任务前将 `fixture.release` 置为 `true` 并释放 `HeapPressure`，确保异常分支下门控任务也能正常退出。
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

    /// 验证 `NetTables` 在填满容量后遭遇堆内存耗尽（OOM）扩容失败时，原有已登记端点与双向 UDP 收发能力不受破坏。
    ///
    /// - **测试目标**：确认 `NetTables::insert_udp` 在扩容分配失败返回 `Err(FsError::NoMemory)` 时具备强异常安全保证：
    ///   既不破坏表内已有端点索引，也不泄露半成品槽位，且释放内存压力后能够成功插入并正常通信。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建绑定在 `SERVER` 与 `CLIENT` 的两个 UDP 端点 `first`、`candidate`，另建一张独立的 `NetTables` 并用填充端点将其精确塞满当前容量。
    ///   2. 通过 `HeapPressure` 耗尽剩余堆内存，尝试向已满的独立表中插入 `candidate`，记录返回结果与表容量状态，随后释放内存压力。
    ///   3. 内存恢复后再次将 `candidate` 插入独立表，并在 `candidate` 与 `first` 之间双向收发 513 字节的 UDP 负载。
    /// - **预期结果**：OOM 期间 `insert_udp` 返回 `Err(FsError::NoMemory)`，表容量与原有端点指针保持不变且未暴露 `candidate`；
    ///   释放内存压力后插入成功，双向 513 字节报文完整送达，关闭全部端点后资源完全回收。
    #[test]
    #[ktest(tags("network-growth"))]
    fn udp_owner_table_growth_no_memory_preserves_prepared_objects(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 准备待测端点与一张独立的 NetTables，并用填充端点精确占满该表的初始容量。
        let first = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        child_operation(first.bind(SERVER))?;
        let candidate = child_operation(fixture.net.try_new_udp(AddressFamily::Ipv4))?;
        let source = child_operation(candidate.bind(CLIENT))?;
        // 独立的 NetTables 仅用于测试表扩容行为，不接管异步回调生命周期；fixture.net 的主表始终跟踪全部端点直至测试结束。
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

        // Step 2: 注入堆内存耗尽压力，验证满表插入扩容返回 NoMemory 且不破坏已有表项。
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

        // Step 3: 释放内存压力后重试插入，并验证双向 UDP 报文收发与最终端点退役回收。
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

    /// 验证两个不同 CPU 核心并发批量关闭旧 UDP 端点与创建绑定新 UDP 端点时，索引表并发安全性及全部新端点的报文可达性。
    ///
    /// - **测试目标**：确认在多核高并发下同时执行 129 个 UDP 端点的 `close()` 注销与另外 129 个新 UDP 端点的创建及 `bind()` 发布时，
    ///   `NetTables` 不会发生锁竞争死锁、槽位错乱或绑定丢失。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 预先创建并绑定 129 个旧 UDP 端点（端口 `47000..47129`），逐一发送 2 字节消息确认其已正式挂入接收表。
    ///   2. 在另一 CPU 上派生关闭任务并在 `Completion` 门控处就绪；主任务触发门控后，双核同时分别关闭 129 个旧端点、创建并绑定 129 个新端点（端口 `47200..47329`）。
    ///   3. 等待并发阶段结束，向全部 129 个新端点逐一发送 513 字节特征报文并校验接收内容。
    /// - **预期结果**：129 个旧端点全部完成关闭，129 个新端点无一丢失本地绑定且全部准确收到各自的 513 字节消息。
    #[test]
    #[ktest(min_cpus = 2, tags("network-growth"))]
    fn udp_close_and_create_publish_parallel_consumes_all_successes(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 预创建 129 个旧 UDP 端点并逐个完成一次收发，确保它们均已在协议栈索引表中生效。
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

        // Step 2: 使用 Completion 门控对齐起跑线，在两个 CPU 上并行执行旧端点批量关闭与新端点批量创建绑定。
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

        // Step 3: 逐个向并发期间创建的 129 个新端点发送 513 字节消息，确认每个绑定均正常工作。
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

    /// 验证双栈（Dual-Stack）IPv6 通配自动端口（`port = 0`）同时占用 IPv4 同端口，而启用 `IPV6_V6ONLY` 后允许 IPv4 绑定同一端口。
    ///
    /// - **测试目标**：确认未开启 `V6ONLY` 的 IPv6 通配套接字申请临时端口时会跨协议族保留 IPv4 端口空间，开启 `V6ONLY` 后则互不冲突。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建默认双栈 IPv6 UDP 端点 `dual` 并绑定 `[::]:0`，获取分配的临时端口后，尝试用 IPv4 端点绑定 `127.0.0.1:<同一端口>`。
    ///   2. 创建另一个 IPv6 UDP 端点 `native`，设置 `set_v6_only(true)` 后绑定 `[::]:0`，再用上述 IPv4 端点绑定 `127.0.0.1:<native 的端口>`。
    /// - **预期结果**：分配的临时端口位于 `49152..=65535` 范围内；双栈端口拒绝同端口 IPv4 绑定并返回 `AddressInUse`，而 `V6ONLY` 端口允许同端口 IPv4 绑定成功。
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

    /// 验证跨任务下 TCP 通配地址绑定与具体 IPv4 地址绑定的端口冲突规则，以及 TCP 与 UDP 端口命名空间的相互独立性。
    ///
    /// - **测试目标**：确认同一协议内 `0.0.0.0:port` 通配监听会阻止其他任务在同一端口绑定具体地址 `127.0.0.1:port`，而不同传输层协议（UDP）仍可绑定相同端口。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 主任务创建 TCP 端点绑定 `0.0.0.0:45120` 并进入 `listen` 状态。
    ///   2. 在另一 CPU 派生子任务，先尝试将新 TCP 端点绑定到 `127.0.0.1:45120`，再将新 UDP 端点绑定到 `0.0.0.0:45120`。
    /// - **预期结果**：跨任务 TCP 具体地址绑定返回 `Err(FsError::AddressInUse)`，UDP 同端口通配绑定成功且原有 TCP 监听端点状态保持不变。
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

    /// 验证两个固定在不同 CPU 上的并发任务同时高频申请自动临时端口（`port = 0`）时，同协议内分配的端口互不重复。
    ///
    /// - **测试目标**：确认多核并发执行 `bind(0.0.0.0:0)` 时，TCP 与 UDP 各自的临时端口分配器具备原子唯一性，不会把同一端口同时租给两个活跃端点。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 在 CPU0 与 CPU1 上各派生一个固定亲和性的子任务，通过自旋/睡眠门控对齐后同时开始循环。
    ///   2. 每个子任务连续循环 16 轮，每轮各创建一个 TCP 端点和一个 UDP 端点并绑定 `0.0.0.0:0`，记录协议标识与分配到的端口号。
    ///   3. 汇总全部 64 条 `(is_tcp, port)` 记录并检查是否存在重复项。
    /// - **预期结果**：所有分配端口均落入临时端口区间（`>= 49152`），且 64 次自动绑定产生的 `(协议, 端口)` 组合两两互不相同。
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

    /// 验证在完全依赖后台工作队列转发回环报文的条件下，TCP 三次握手、超长流式回绕传输、`MSG_PEEK` 窥视读与 `FIN` 优雅关闭全流程。
    ///
    /// - **测试目标**：确认无需测试线程手动泵送报文，仅靠 `NetStack` 后台 `Work` 即可推进连接握手、超过 256 字节窗口的 1025 字节流式传输、
    ///   `MSG_PEEK` 非破坏性读取以及对端 `close()` 后的尾部数据读取与 `EOF` 通知。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立监听端点并发起客户端 `connect(SERVER)`，等待后台完成三次握手并通过 `accepted` 获取服务端连接。
    ///   2. 客户端与服务端并发循环收发 1025 字节规律数据；服务端每次收到数据时先执行最多 31 字节的 `MSG_PEEK` 校验内容及队列长度不变，再执行正式读取消费数据。
    ///   3. 客户端继续发送 `"background final tail"` 并调用 `close()`，等待服务端进入 `CloseWait` 后依次执行两次 `receive`。
    /// - **预期结果**：1025 字节主体流与尾部字节全部准确送达，读取完尾部数据后的下一次读取返回 `IoProgress::Eof`。
    #[test]
    fn background_handshake_full_stream_peek_and_fin(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立监听并发起客户端连接，由后台回环工作队列自动完成三次握手。
        let listener = listener(context, &fixture.net, SERVER.port)?;
        let client = operation(
            context,
            fixture.net.try_new_tcp(AddressFamily::Ipv4, 256, 256),
        )?;
        operation(context, client.connect(SERVER))?;
        let server = accepted(context, &listener, &client)?;

        // Step 2: 经后台回环传输 1025 字节数据流（超过 256 字节缓冲区触发多次回绕），并在每次消费前验证 MSG_PEEK。
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

        // Step 3: 发送尾部数据并主动关闭客户端，验证服务端进入 CloseWait 后先读出尾部字节再收到 Eof。
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

    /// 验证连接级睡眠锁的并发隔离性：当一个 CPU 上的任务长期持有连接 A 的操作锁时，另一 CPU 上的连接 B 仍能独立完成收发。
    ///
    /// - **测试目标**：确认 `NetStack` 未采用全局协议大锁，单条 TCP 连接持有 `TcpEndpoint::lock()` 期间不会阻塞归属其他 CPU 的独立连接推进。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立分配在不同协议 CPU 上的两条独立 TCP 连接 A（端口 `45210`）与 B（端口 `45211`）。
    ///   2. 在连接 A 所属 CPU 上派生 `holder` 子任务获取并一直持有 `client_a.lock()`，确认其已进入临界区。
    ///   3. 在连接 B 所属 CPU 上派生子任务，通过 `client_b` 发送 97 字节数据，并由主任务从 `server_b` 接收校验，最后释放连接 A 的锁。
    /// - **预期结果**：在连接 A 操作锁始终被持有的期间，连接 B 的 97 字节报文完整到达 `server_b` 且内容一致，两条连接均保持 `Established` 状态。
    #[test]
    #[ktest(min_cpus = 2)]
    fn independent_connection_advances_while_other_is_owned(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立归属在不同工作 CPU 上的两条独立 TCP 已建立连接 A 与 B。
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

        // Step 2: 在连接 A 所属 CPU 上获取并持续持有 client_a 的连接级操作锁。
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

        // Step 3: 在连接 B 所属 CPU 上发送 97 字节数据并在服务端完整接收，随后通知 holder 释放锁退出。
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

    /// 验证回环接口（`lo`）入站路径对非空与零长度 UDP 数据报的精确投递、接口流量统计累加以及关闭后的接收索引注销。
    ///
    /// - **测试目标**：确认 17 字节有效载荷与 0 字节空载荷 UDP 报文经回环收发后，均作为独立数据报保留边界与源地址，
    ///   回环接口收发包数与 IP 总字节数（`20 + 8 + 17 = 45` 与 `20 + 8 + 0 = 28`，合计 `73` 字节）精确统计，且 `close()` 后立即从索引表摘除。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建绑定 `0.0.0.0:45310` 的 IPv4 UDP 端点，记录回环接口初始统计快照 `before`。
    ///   2. 依次构造 17 字节和 0 字节的 `OwnedPacket` 并通过 `send_ip` 注入回环栈，等待 `POLLIN` 就绪后调用 `receive` 读取报文及源地址。
    ///   3. 再次读取回环统计 `after` 计算差值，确认队列已空后调用 `udp.close()` 并等待 `lookup_udp` 返回 `None`。
    /// - **预期结果**：两帧数据报的内容、长度与源地址完全匹配；回环 `rx/tx` 各增加 2 包、73 字节；关闭后接收索引被清除。
    #[test]
    #[ktest(tags("N28"))]
    fn udp_index_delivers_owned_and_zero_length_records(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 绑定通配监听端口并采样回环接口初始流量统计。
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
        // Step 2: 依次发送并接收 17 字节非空报文与 0 字节空报文，校验报文边界与源端点。
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

        // Step 3: 核对回环接口收发字节数（45 + 28 = 73 字节）与包数（2 包），并确认关闭端点后索引表项已移除。
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

    /// 验证底层 `smoltcp` UDP 套接字缓冲区在元数据槽满、字节预算满、动态调低预算与恢复预算时的接纳和留存行为。
    ///
    /// - **测试目标**：确认 IPv4/IPv6 `raw_udp::Socket` 同时受限于报文元数据槽位数（32 条）与有效载荷字节预算；
    ///   动态调小收发预算不会丢弃已入队的旧报文，调大预算并排空后可恢复新报文的接纳与发送。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 分别针对 IPv4 与 IPv6 构造容量为 32 条元数据、128 字节载荷的 `raw_udp::Socket`，连续注入 32 个空报文占满元数据槽，验证第 33 个被拒后再全部读出。
    ///   2. 将接收预算设为 64 字节，注入两个 32 字节报文占满预算并确认第 3 个被拒；将预算调低至 16 字节后验证旧有的两个报文仍能完整读出，调回 64 字节后恢复接纳。
    ///   3. 将发送预算设为 32 字节，写入一个 32 字节报文验证满预算拒绝追加；调低预算并模拟 `WouldBlock` 发送失败后，调大预算至 64 字节完成分发与后续报文发送。
    /// - **预期结果**：满元数据或满字节预算时准确拒绝新包，缩减预算不丢失已入队记录，恢复预算后收发状态机均正常继续推进。
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
            // Step 1: 注入 32 个零长度 UDP 包占满元数据槽位，验证第 33 个包被拒收且前 32 个包及源/本地地址完整可读。
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

            // Step 2: 验证接收字节预算上限（64 字节）、调低预算（16 字节）保留已入队旧记录，以及调高预算后恢复接收。
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

            // Step 3: 验证发送字节预算上限、调低预算与下游 WouldBlock 拒发保留原包，以及调高预算后恢复发送。
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

    /// 验证连接与监听端点在关闭后，后台退休任务会彻底释放底层端点对象、关联 `Work`/`Timer` 及表槽位，使外部 `Weak` 引用归零。
    ///
    /// - **测试目标**：确认 TCP 已建立连接调用 `abandon()`、监听端点调用 `close()` 以及大批量（129 个）UDP 端点关闭后，
    ///   协议栈表与后台退休队列会释放最后一个强引用（`Arc`），不会因内部异步回调残留而造成泄漏，且退役后可继续创建新端点。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立一对 TCP 客户端与服务端连接，保留两者的 `Weak` 观察指针后调用 `client.abandon()` 并丢弃局部 `Arc`，等待两个 `Weak` 均无法升级；随后同样关闭并验证 `listener` 释放。
    ///   2. 连续创建 129 个 UDP 端点（触发索引表扩容），校验活跃端点数为 129 后全部关闭并丢弃局部引用，调用 `socket_retired` 等待后台彻底回收。
    ///   3. 在全部退役后新建并关闭一个 UDP 端点，验证协议栈仍可正常分配。
    /// - **预期结果**：所有已关闭 TCP/UDP 端点的强引用均被完全释放（`Weak::upgrade()` 为 `None`、活跃计数归零），后续新建端点正常工作。
    #[test]
    fn closed_connections_retire_work_timer_and_endpoint_slots(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立 TCP 连接后执行 abandon 与 close，通过 Weak 指针验证连接和监听端点实例被彻底销毁。
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

        // Step 2: 批量创建 129 个 UDP 端点触发扩容，随后全部关闭并等待后台退休队列清空，验证槽位与端点计数归零。
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

    /// 验证软件出站队列（`Egress`）满载时的未接纳报文所有权归还、TCP `SYN` 发送方的背压等待登记，以及消费端恢复后的自动唤醒建连。
    ///
    /// - **测试目标**：确认当回环入站消费所在核心（CPU0）暂时被关中断挂起导致 32 槽出站队列填满时：
    ///   1. `try_emit` 拒绝第 33 个包并原样归还 `OwnedPacket` 所有权；
    ///   2. 另一核心（CPU1）发起 `connect` 的 TCP 客户端因无法发出 `SYN` 而停留在 `SynSent` 并挂入出站等待队列；
    ///   3. CPU0 恢复中断排空出站队列后，自动唤醒等待中的客户端完成三次握手与后续流式通信。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建监听端点与绑定在 CPU1 的 TCP 客户端，预先分配 `LOOPBACK_CAPACITY`（32）个填充报文和 1 个额外报文 `extra`。
    ///   2. 在 CPU1 控制任务中派生固定于 CPU0 的门控子任务，获取 `IrqSaveGuard` 短暂阻止 CPU0 执行入站 `Work`。
    ///   3. 在 CPU1 普通任务中提交 32 个报文塞满出站队列，验证提交 `extra` 返回 `Err((FsError::WouldBlock, packet))` 且载荷完好；接着调用 `client.connect` 并观察 `egress_waiting` 持续为真。
    ///   4. 置位 `release` 让 CPU0 恢复中断并回收门控子任务，等待三次握手完成并传输 `"capacity recovered stream"`。
    /// - **预期结果**：满队列准确退回第 33 个包并阻塞 `SYN` 发送，恢复中断后自动完成建连、清除等待登记并成功收发应用层字节流。
    #[test]
    #[ktest(min_cpus = 2)]
    fn full_egress_waits_for_real_capacity_and_resumes(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立监听与归属 CPU1 的客户端，并在普通上下文中预先分配填满出站队列所需的 32 个报文及 1 个溢出探测报文。
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

        // Step 2: 在 CPU0 上进入短暂关中断门控以暂停入站消费，在 CPU1 上填满出站队列并验证 TCP SYN 背压等待登记。
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
                // CPU0 关中断期间仅暂停本核的后台入站 Work 调度；所有报文提交、路由与睡眠观察均留在 CPU1 普通可睡眠任务中执行。
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
            // 无论断言成功或失败均先置位 release 解除 CPU0 关中断门控并 join 子任务，确保未接纳报文在 CPU1 普通任务上下文中安全析构。
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
        // Step 3: CPU0 恢复中断排空出站队列后，验证被阻塞的客户端自动完成握手并正常收发数据流。
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

    /// 验证销毁 `NetStackOwner` 触发同步停机时，即使外部仍持有端点 `Arc` 强引用，所有活跃连接与 UDP 队列也会立即终止并拒绝后续新请求。
    ///
    /// - **测试目标**：确认 `NetStackOwner` 析构会强制将所有存活的 TCP 监听/已建立连接推入 `Closed` 状态、清空并关闭已绑定的 UDP 接收队列，
    ///   且处于停机状态（`is_stopping() == true`）的网络栈拒绝创建新端点或接纳新出站报文（原样退回 `OwnedPacket`）。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立活跃的 TCP 监听/客户端/服务端连接与已入队一条待读报文的 UDP 端点，并向 TCP 客户端写入待发数据，同时在当前栈帧保留全部 `Arc`。
    ///   2. 显式调用 `drop(fixture.owner.take())` 同步关停网络栈，随后检查 `is_stopping()`、各 TCP 端点状态及 UDP 端点的 `receive`/`local()` 结果。
    ///   3. 尝试在已停机的 `fixture.net` 上调用 `try_new_udp` 创建新端点和调用 `try_emit` 发送新报文。
    /// - **预期结果**：停机后所有 TCP 端点均为 `State::Closed`，UDP 读取返回 `Err(FsError::NotConnected)` 且本地绑定清空；
    ///   新建端点与发送报文均返回 `NotConnected`，且被拒发的 `OwnedPacket` 完好归还给调用方。
    #[test]
    fn owner_stop_ends_live_protocol_and_rejects_new_ownership(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立处于活跃工作状态的 TCP 监听/连接与带排队报文的 UDP 端点，并由当前栈帧持有其强引用。
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

        // Step 2: 显式丢弃 NetStackOwner 触发同步停机，验证所有存活端点均已进入终止状态。
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

        // Step 3: 验证已停机的网络栈拒绝创建新端点与接纳新出站报文，并安全归还被拒报文的所有权。
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
