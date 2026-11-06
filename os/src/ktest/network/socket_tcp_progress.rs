// TCP 套接字惰性内存分配（Cold TCP）、每连接跨核并发隔离、超时重传、选项继承与 `FIN`/`TIME_WAIT` 生命周期测试套件。
//
// # 模块职责与架构定位
// 本套件由 `os/src/ktest/network.rs` 通过 `include!` 内联引入，负责验证 `SocketFile` 与底层 `TcpEndpoint`
// 从冷态创建到连接关闭回收的全生命周期语义。
// 上游由 `ktest` 框架在 `Thread` 上下文中驱动（使用 `StackOwner` 夹具），下游调用 `SocketFile`、`TcpEndpoint`、
// `HeapPressure`、内核内存统计 `mm::MemoryStats` 以及多核子任务派生接口 `spawn_result_on`。
//
// # 核心机制与惰性激活/退役状态机
// 新创建的 TCP 套接字在未执行 `listen` 或 `connect` 前处于冷态（Cold State），仅记录套接字选项与收发预算，
// 不提前分配大块收发环形缓冲区；直到首次激活时才按需分配缓冲区，并在关闭后经后台异步回收槽位：
//
// ```text
//   try_new_tcp()                   listen() / connect()                  最后强引用释放 (Drop)
// +------------------------+      +-------------------------------+      +-------------------------------+
// | 冷态 (Cold Closed)     |      | 激活态 (Active Endpoint)      |      | 协议退役 (Protocol Retirement)|
// | - 仅保存选项与收发预算 | ---> | - 按预算分配 RX/TX 环形缓冲区 | ---> | - 发送尾部字节与 FIN 报文     |
// | - bind() 仅登记端口租约|      |   (若 OOM 则保持冷态与原选项) |      | - 经历 CloseWait / TimeWait   |
// | - 无载荷级物理页开销   |      | - 绑定专属协议 Work CPU 并发  |      | - 销毁端点并归还 EndpointSlot |
// +------------------------+      +-------------------------------+      +-------------------------------+
// ```
//
// # 锁获取顺序与并发契约
// - 所有测试用例运行在可睡眠 `Thread` 上下文中；批量创建的候选套接字统一寄存在 `fixture.tcp_candidates` 中，
//   确保即使测试中途断言失败，`StackOwner::quiesce` 也能在关停网络栈前释放强引用并恢复堆内存。
// - 跨核并发测试（`socket_other_cpu_completes_while_connection_is_owned`）中，辅助线程在 CPU A 持有连接 A 的
//   端点睡眠锁期间仅执行 `timer::sleep`，验证连接 B 在 CPU B 上拥有独立的端点锁与协议工作队列，互不阻塞。
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

    /// 批量创建 `count` 个处于冷态（未激活收发缓冲区）的 IPv4 TCP `SocketFile`，并登记到 `fixture` 中托管。
    ///
    /// 在普通可睡眠任务上下文中调用。所有创建的 `SocketFile` 强引用与底层 `TcpEndpoint` 弱引用均存入 `fixture`，
    /// 保证测试中途报错时仍由夹具生命周期统一清理。返回全部端点声明的收发缓冲区配置总字节数，供后续对比实际物理内存开销。
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

    /// 校验冷态 TCP 套接字集合的实际物理页与 SLUB 后备页增量远小于其声明的收发载荷缓冲区总容量。
    ///
    /// 对比 `before` 与 `after` 两次内存统计快照，断言总物理页、保留页、元数据页不变，
    /// 且新占用的伙伴系统/PCP 页字节数与 SLUB 页字节数均严格低于配置载荷总容量 `configured_bytes` 的四分之一，
    /// 证明冷态套接字确实采用了惰性缓冲区分配而非提前分配收发环。可在任意任务上下文中调用。
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

    /// 清空夹具中托管的全部冷态 TCP 套接字强引用，等待后台完成正常协议回收并断言所有弱引用失效。
    ///
    /// 在普通可睡眠任务上下文中调用。验证端点销毁完全由常规文件关闭与异步回收驱动，期间后台网络栈保持运行（`!is_stopping()`）。
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

    /// 验证批量创建 129 或 1024 个冷态 TCP 套接字不会提前分配收发载荷缓冲区，且关闭后能全部正常回收。
    ///
    /// - **测试目标**：确认突破历史固定表规模（129 与 1024 个并发套接字）时，未激活的冷态 TCP 套接字仅占用轻量元数据，
    ///   不分配载荷级环形缓冲区内存；确认清空文件句柄后所有端点身份与槽位均能按常规路径归还。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 采样初始内存基线 `before`，调用 `prepare_cold_tcp` 创建 `count`（129 或 1024）个默认配置的 TCP 套接字并冲刷工作队列。
    ///   2. 调用 `expect_cold_memory` 比较当前内存占用与声明的收发缓冲区总字节数。
    ///   3. 调用 `retire_cold_tcp` 释放所有套接字并等待活跃端点数归零。
    /// - **预期结果**：内存实际增量远小于配置缓冲区容量的 1/4，关闭后全部弱引用清空且活跃端点数回归 0。
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

    /// 验证 1024 个冷态 TCP 套接字绑定显式端口后仍保持惰性内存占用，且关闭中间端点后同端口可立即由新套接字以新租约重用。
    ///
    /// - **测试目标**：确认 `bind` 显式端口不会触发冷态 TCP 分配收发环形缓冲区；确认释放第 512 号套接字并完成退役后，
    ///   同一端口（`47512`）可被新套接字重新绑定并获得全新的文件 ID 与端口绑定租约（Lease），同时阻止第三个探测套接字重复绑定。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建 1024 个冷态 TCP 套接字并核对冷态内存开销，随后依次绑定 `47000..48024` 端口并再次核对内存未膨胀至载荷规模。
    ///   2. 记录索引 512（端口 `47512`）的文件 ID 与绑定租约，移除并丢弃该套接字，等待其弱引用失效且存活端点数降至 1023。
    ///   3. 新建替代套接字绑定同一端口 `47512`，核对新文件 ID 与新绑定租约均不同于旧实例；再新建探测套接字尝试绑定 `47512`。
    ///   4. 统一关闭并回收全部套接字。
    /// - **预期结果**：端口绑定全过程维持低内存占用，释放后的中间端口成功重绑新租约，冲突绑定准确返回 `Err(FsError::AddressInUse)`。
    #[test]
    #[ktest(tags("network-growth", "network-tcp-cold"))]
    fn cold_tcp_1024_bindings_reuse_middle_port_with_new_lease(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建 1024 个冷态套接字并为每个套接字绑定独立端口，验证绑定操作本身不会提前分配收发载荷环形缓冲区。
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

        // Step 2: 单独关闭并移除中间位置（端口 47512）的套接字，同时等待其弱引用失效与底层 EndpointSlot 计数减一。
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
        observe(
            || {
                fixture.tcp_observers[middle].upgrade().is_none()
                    && fixture.net.ktest_live_endpoints() == 1023
            },
            "middle cold TCP did not retire normally",
        )?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 1023);

        // Step 3: 用新套接字重新绑定刚才释放的 47512 端口，验证生成了全新的绑定租约且在用端口仍互斥拒绝第三个套接字绑定。
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

    /// 读取指定 TCP `SocketFile` 当前的协议选项与收发缓冲区预算元组。
    ///
    /// 返回 `(TCP_NODELAY, SO_KEEPALIVE, 拥塞控制算法, 接收预算字节数, 发送预算字节数)`；
    /// 仅只读查询端点元数据，不触发冷态缓冲区分配或协议工作队列调度。可在普通任务上下文中调用。
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

    /// 验证冷态 TCP 套接字在首次激活（`listen` 或 `connect`）遭遇内存耗尽（OOM）时完整保留原选项且不泄漏半绑定状态，释放内存压力后可正常建连收发。
    ///
    /// - **测试目标**：确认预先配置了 `TCP_NODELAY`、`SO_KEEPALIVE` 与自定义收发预算的冷态套接字，在首次执行 `listen` 或 `connect`
    ///   分配环形缓冲区遇到 `FsError::NoMemory` 时，保持 `State::Closed`、未绑定端口（`0.0.0.0:0`、无绑定租约）且原策略配置完好无损；
    ///   确认解除内存压力后，同一套接字对象可继续完成三次握手、8193 字节双向回显传输及 `FIN` 优雅关闭。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建监听端与客户端两个冷态套接字，对被测对象 `subject`（按用例参数选择 `listen` 端或 `connect` 端）设置选项与 1024/2048 字节收发预算。
    ///   2. 开启 `HeapPressure` 耗尽堆内存，对 `subject` 执行 `listen(1)` 或 `connect(...)`，记录返回的错误码、本地地址、状态与策略快照，随后释放内存压力。
    ///   3. 在内存恢复后用同一 `subject` 完成绑定/监听或连接、`accept` 接收连接，核对连接及派生的服务端子端点均继承了预期策略。
    ///   4. 分批双向回显传输共 8193 字节（远超 1024/2048 预算容量），随后执行双向 `SHUT_WR` 关闭并回收全部端点。
    /// - **预期结果**：OOM 激活尝试准确返回 `Err(FsError::NoMemory)` 且状态无污染；恢复后同一对象顺利建连、8193 字节往返完全一致并干净退役。
    #[test]
    #[case("listen", true)]
    #[case("connect", false)]
    #[ktest(tags("network-growth", "network-tcp-cold"))]
    fn cold_tcp_activation_oom_preserves_policy_and_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
        listening: bool,
    ) -> TestResult {
        // Step 1: 准备冷态套接字并在激活前预设 nodelay、keepalive 与收发缓冲区预算。
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

        // Step 2: 占满堆内存触发首次激活 OOM，断言返回 NoMemory 且套接字仍保持未绑定 Closed 状态与原配置策略。
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

        // Step 3: 释放内存压力后复用同一个套接字完成握手建连，并校验监听端派生的子连接同样继承了预设策略。
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

        // Step 4: 执行超过缓冲区容量数倍的 8193 字节双向流式读写，随后双向优雅关闭并回收端点。
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

    /// 验证绑定在不同协议工作 CPU 上的两条 TCP 连接拥有独立的锁与后台处理通道：当连接 A 的端点锁被长时间持有时，另一 CPU 上的连接 B 仍能独立完成双向收发。
    ///
    /// - **测试目标**：确认多核环境下 TCP 连接按各自绑定的 `work_cpu` 隔离并发度，单条连接在 CPU A 上持有端点睡眠锁不会阻塞
    ///   CPU B 上另一条连接的 `SocketFile` 读写与后台 `ACK` 处理。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建两组监听端与客户端（A 与 B），校验 `client_a` 与 `client_b` 分配到了不同的协议处理核（`cpu_a != cpu_b`），完成两组握手与 `accept`。
    ///   2. 在 `cpu_a` 上派生持锁子任务 `holder`，获取 `client_a` 底层端点的睡眠锁并持续休眠持有，直至测试结束信号 `release` 置位。
    ///   3. 在连接 A 被锁住期间，在 `cpu_b` 上派生子任务通过 `client_b` 写入 97 字节并等待回显，主线程通过 `server_b` 读出并原样写回。
    ///   4. 确认连接 B 读写子任务顺利完成后，再置位 `release` 释放连接 A 的锁并回收全部端点。
    /// - **预期结果**：在 `release` 仍为 `false`（连接 A 全程持锁）期间，连接 B 在 `cpu_b` 上无阻塞地完成 97 字节双向收发校验。
    #[test]
    #[ktest(min_cpus = 2)]
    fn socket_other_cpu_completes_while_connection_is_owned(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立两条分配在不同协议工作 CPU 上的独立 TCP 连接 A 与 B。
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

        // Step 2: 在 cpu_a 上启动子任务长时间持有连接 A 的端点睡眠锁。
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

        // Step 3: 在连接 A 持锁不放的同时，于 cpu_b 上驱动连接 B 完成双向字节流读写与后台 ACK 交互。
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

        // Step 4: 通知持锁子任务释放连接 A，汇合所有子任务并等待全部套接字退役。
        fixture.release.store(true, Ordering::Release);
        context.expect_eq(holder.join()?, 0);
        drop((
            client_a, client_b, server_a, server_b, listener_a, listener_b,
        ));
        socket_retired(&fixture.net)
    }

    /// 验证单笔数据包被丢弃后，连接能依据底层协议栈计算的超时重传截止时间（RTO Deadline）重传相同序列号报文并完整交付数据，且不干扰旁路健康连接。
    ///
    /// - **测试目标**：确认发送段丢失后，`TcpEndpoint` 保留精确的 `ProtocolDeadline::At(at)` 重传定时器截止时间；
    ///   确认单调时钟到达 `at` 后触发同序列号（Sequence Number）重传并使接收端读出完整 97 字节，同时验证同栈上的另一条健康连接收发不受影响。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立被测连接（`45621 -> 45620`）与旁路健康连接（`-> 45622`）。
    ///   2. 配置故障注入器丢弃 `45621 -> 45620` 方向的下一笔数据包，客户端写入 97 字节并等待观察到 `dropped == 1`。
    ///   3. 查询客户端当前的协议截止时间 `ProtocolDeadline::At(deadline)`，校验首次丢包时刻早于 `deadline` 且此时尚未发生重传。
    ///   4. 服务端阻塞调用 `socket_read_exact` 等待重传到达并读满 97 字节，核对重传计数、序列号及重传时间戳 `repeated_ns >= deadline`，最后校验健康连接正常收发并清理规则。
    /// - **预期结果**：仅丢弃 1 个包，协议栈在到达 `deadline` 后执行重传且序列号一致，服务端与健康连接均收到完整无误的 97 字节。
    #[test]
    fn one_data_segment_retransmits_at_real_raw_deadline(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立待注入丢包的被测 TCP 连接与一条作为对照的健康 TCP 连接。
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

        // Step 2: 配置单次数据包丢弃规则并发送 97 字节，验证包被丢弃后客户端登记了未来的 RTO 重传截止时间。
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

        // Step 3: 阻塞读取服务端直至超时重传完成，校验重传发生在 RTO 截止时间之后且序列号与原包一致。
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

        // Step 4: 验证旁路健康连接正常传输，清除包控制规则并等待全部端点正常退役。
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

    /// 验证最后一个打开文件描述（`SocketFile`）关闭后，后台网络栈仍能继续发送缓冲区尾部数据与 `FIN`、走完 `TIME_WAIT` 并最终归还端点槽位。
    ///
    /// - **测试目标**：确认当应用程序写完数据立即 `drop(client)` 释放最后一个 `SocketFile` 引用时，虽然文件层对象立即析构，
    ///   但底层 `TcpEndpoint` 仍由网络栈保活，能够将待发尾部数据与 `FIN` 送达服务端，并在服务端关闭后独立走完 `TIME_WAIT` 定时周期再彻底释放。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立连接，记录 `client` 文件与两端 `TcpEndpoint` 的弱引用。
    ///   2. 客户端写入 `"tail before FIN"` 后立即 `drop(client)`，断言 `SocketFile` 弱引用已清空而 `TcpEndpoint` 弱引用仍存活。
    ///   3. 服务端依次读取 15 字节尾部数据与随后的 `0`（EOF），验证服务端进入 `CloseWait`。
    ///   4. 丢弃 `server` 触发反向 `FIN`，等待已无文件句柄的客户端端点自动进入 `State::TimeWait`，随后丢弃监听端并调用 `socket_retired` 等待 `TIME_WAIT` 结束。
    /// - **预期结果**：客户端文件句柄提前销毁不影响尾部字节和 `FIN` 交付，端点准确经历 `TimeWait` 后在网络栈不停机的情况下自动销毁并归还槽位。
    #[test]
    fn last_ofd_fin_time_wait_and_endpoint_slot_retire_normally(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 建立 TCP 连接（配置较短超时参数），记录文件对象与底层协议端点的弱引用。
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

        // Step 2: 客户端写入尾部字节后立即丢弃最后一个 SocketFile 引用，验证服务端仍能完整读出尾部数据与 FIN EOF。
        socket_write_all(&client, b"tail before FIN")?;
        drop(client);
        context.expect_eq(file_weak.upgrade().is_none(), true);
        context.expect_eq(client_weak.upgrade().is_some(), true);
        let mut output = [0u8; 15];
        socket_read_exact(&server, &mut output)?;
        context.expect_eq(&output, b"tail before FIN");
        context.expect_eq(operation(context, server.read(&mut [0u8; 1]))?, 0);
        context.expect_eq(server.tcp_endpoint().unwrap().state(), State::CloseWait);

        // Step 3: 丢弃服务端触发被动关闭方发送 FIN，验证已脱离文件句柄的客户端端点独立推进至 TimeWait，最终超时销毁并归还槽位。
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

    /// 验证 `TCP_NODELAY` 等套接字选项直接作用于底层协议栈的 Nagle 算法开关，且 `accept` 派生的子连接继承监听端选项后可与监听端互不干扰地独立修改。
    ///
    /// - **测试目标**：确认 `set_nodelay` 直接翻转底层 `TcpCore` 的 `nagle_enabled()` 状态（`nodelay=true` 对应 `nagle_enabled=false`）；
    ///   确认监听套接字上设置的 `nodelay`、`reuse_address`、`keepalive` 与收发预算会被 `accept` 产出的子连接完整继承，
    ///   且后续修改子连接的选项不会反向篡改监听套接字的配置。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 创建监听套接字，反复切换 `set_nodelay(true/false)` 并核对底层 `nagle_enabled()` 状态翻转。
    ///   2. 在监听端开启 `nodelay`、`reuse_address`、`keepalive` 并设置 1024/2048 收发预算，随后 `bind` 并 `listen`。
    ///   3. 客户端连接并由监听端 `accept` 得到 `child`，逐项核对 `child` 继承的预算、容量、保活、拥塞控制算法、Nagle 状态与地址复用标志。
    ///   4. 在 `child` 上关闭 `keepalive`、`nodelay` 与 `reuse_address`，对比验证 `child` 已更新而 `listener` 原配置保持不变，最后完成数据收发与关闭回收。
    /// - **预期结果**：底层 Nagle 开关与 `TCP_NODELAY` 严格反向同步，子连接准确继承全部监听端策略且后续修改与监听端完全隔离。
    #[test]
    fn socket_options_modify_raw_nagle_and_child_inherits(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 验证切换 TCP_NODELAY 直接翻转底层协议核心的 Nagle 算法开关，并在监听端配置全套选项与收发预算。
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

        // Step 2: 建立连接并 accept 派生子套接字，核对子套接字完整继承了监听端的预算、容量、保活、Nagle 与地址复用选项。
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

        // Step 3: 独立翻转子套接字上的各项开关，确认监听端原有选项不受影响，最后验证数据传输与优雅关闭。
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
