// 同步 TCP 协议核心（`TcpCore`）测试套件。
//
// # 模块职责与架构定位
// 本套件由 `os/src/ktest/network.rs` 通过 `include!` 内联引入，专门在无后台网络线程的同步环境下
// 验证 `crate::net::core::TcpCore` 的协议状态机与收发缓冲区边界语义。
// 上游由 `ktest` 框架驱动，测试夹具为 `super::wire::NetworkOwner`；下游直接调用 `TcpCore`、
// `ReadBuffer`、`WriteBuffer` 与用户地址空间 `Mm`。
//
// # 核心设计与数据传输模型
// 测试夹具 `NetworkOwner` 维护客户端与服务端两个 `TcpCore` 实例（每个实例的收发环形缓冲区容量为
// 256 字节）以及一个容量为 `LOOPBACK_CAPACITY` 的内存数据包队列（`wire`）：
//
// ```text
//   Client TcpCore (256B Ring)                   Server TcpCore (256B Ring)
//   +------------------------+                   +------------------------+
//   | send() / dispatch()    |                   | process() / receive()  |
//   +-----------+------------+                   +-----------+------------+
//               |                                            ^
//               | enqueue(OwnedPacket)         take_packet() |
//               v                                            |
//        +------+--------------------------------------------+------+
//        |       NetworkOwner::wire (SpinLock<VecDeque>)            |
//        |       容量上限: LOOPBACK_CAPACITY (满时返回 WouldBlock)  |
//        +----------------------------------------------------------+
// ```
//
// # 锁获取顺序与并发契约
// - 本套件在单线程同步驱动模式下运行（`context = "Thread"`），通过 `fixture.pump()` 轮流锁定
//   `client`（`SleepMutex`）或 `server`（`SleepMutex`）。
// - 在 `dispatch` 回调内部会短暂获取 `fixture.wire`（`SpinLock`）以入队数据包，锁顺序固定为：
//   `TcpCore` 睡眠锁 -> `wire` 自旋锁，且绝不会同时持有 `client` 与 `server` 两把睡眠锁。
#[ktest::suite(fixture = NetworkOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-core"), timeout_ms = 30000)]
pub(crate) mod core {
    use ::ktest::{TestContext, TestError, TestResult};
    use smoltcp::socket::tcp::State;

    use super::support::{operation, pattern, ADDRESS, CLIENT, RW, SERVER};
    use super::wire::NetworkOwner;
    use crate::fs::{FsError, ReadBuffer, WriteBuffer};
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::mm::space::Mm;
    use crate::net::address::InetEndpoint;
    use crate::net::core::{ProtocolDeadline, TcpCore};
    use crate::net::{IoProgress, LOOPBACK_CAPACITY, LOOPBACK_MTU};
    use crate::{time, timer};

    /// 验证超过环形缓冲区容量的流式传输、`MSG_PEEK` 窥视读以及 `FIN` 半关闭后的尾部数据交付与 `EOF` 语义。
    ///
    /// - **测试目标**：确认当总数据量（1025 字节）远超单端环形缓冲区容量（256 字节）时，
    ///   `TcpCore` 能正确执行多轮环形回绕传输；确认 `receive(..., true)`（`MSG_PEEK`）只读取而不消费队列字节；
    ///   确认对端调用 `close()` 发送 `FIN` 后，接收端先读完缓冲区剩余尾部数据，随后再返回 `IoProgress::Eof`。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立客户端与服务端的 TCP 连接，准备 1025 字节的规律填充数据。
    ///   2. 循环由客户端分批写入、驱动 `pump()` 转发，服务端在有数据时先执行最多 31 字节的 `MSG_PEEK` 校验内容，
    ///      再执行普通读取消费数据并校验队列长度递减，直至 1025 字节全部收完。
    ///   3. 客户端追加发送 `"final tcp tail"` 并立即调用 `close()` 发起主动关闭，驱动状态机直至服务端进入 `CloseWait`。
    ///   4. 服务端先读取尾部数据，随后再次调用 `receive()`。
    /// - **预期结果**：1025 字节流与尾部字节全部完整匹配，且在读取完尾部数据后的下一次读取准确返回 `IoProgress::Eof`。
    #[test]
    fn owned_packets_stream_wrap_peek_and_fin(
        context: &mut TestContext<'_>,
        fixture: &mut NetworkOwner,
    ) -> TestResult {
        // Step 1: 完成两端三次握手并初始化 1025 字节测试数据与超时截止时间。
        fixture.connect(context)?;
        let input: [u8; 1025] = ::core::array::from_fn(pattern);
        let mut output = [0u8; 1025];
        let mut sent = 0;
        let mut received = 0;
        let end = time::now()
            .checked_add(time::Duration::from_secs(5))
            .unwrap();

        // Step 2: 分批推进流式发送与接收，验证 256 字节环形缓冲区的多次回绕以及 MSG_PEEK 不消费数据的特性。
        while sent < input.len() || received < input.len() {
            if sent < input.len() {
                let mut source = WriteBuffer::kernel(&input[sent..]);
                if let IoProgress::Ready(count) =
                    operation(context, fixture.client.lock().send(&mut source))?
                {
                    sent += count;
                }
            }
            fixture.pump(context)?;
            if 0 < fixture.server.lock().receive_queue() {
                let mut peek = [0u8; 31];
                let observed = operation(
                    context,
                    fixture
                        .server
                        .lock()
                        .receive(&mut ReadBuffer::kernel(&mut peek), true),
                )?;
                let IoProgress::Ready(count) = observed else {
                    return Err(TestError("queued TCP bytes did not peek"));
                };
                context.expect_eq(&peek[..count], &input[received..received + count]);
                let before = fixture.server.lock().receive_queue();
                let mut target = ReadBuffer::kernel(&mut output[received..]);
                let IoProgress::Ready(count) =
                    operation(context, fixture.server.lock().receive(&mut target, false))?
                else {
                    return Err(TestError("queued TCP bytes did not read"));
                };
                received += count;
                context.expect_eq(fixture.server.lock().receive_queue(), before - count);
            }
            if end <= time::now() {
                return Err(TestError("native TCP full stream deadline"));
            }
            timer::sleep(time::Duration::from_millis(1));
        }
        context.expect_eq(output, input);

        // Step 3: 发送尾部数据并主动关闭客户端，验证服务端进入 CloseWait 后先交付剩余字节、再返回 Eof。
        let tail = b"final tcp tail";
        let mut source = WriteBuffer::kernel(tail);
        context.expect_eq(
            operation(context, fixture.client.lock().send(&mut source))?,
            IoProgress::Ready(tail.len()),
        );
        fixture.client.lock().close();
        fixture.until(context, || {
            fixture.server.lock().state() == State::CloseWait
        })?;
        let mut bytes = [0u8; 32];
        context.expect_eq(
            operation(
                context,
                fixture
                    .server
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?,
            IoProgress::Ready(tail.len()),
        );
        context.expect_eq(&bytes[..tail.len()], tail);
        context.expect_eq(
            operation(
                context,
                fixture
                    .server
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?,
            IoProgress::Eof,
        );
        Ok(())
    }

    /// 验证底层发包队列已满导致 `SYN` 发送返回 `WouldBlock` 时，协议核心不会丢失待发送的握手状态。
    ///
    /// - **测试目标**：确认当外部包队列（`wire`）满载拒绝入队时，`TcpCore::dispatch` 能安全回滚或保留
    ///   待发送的 `SYN` 包与初始序列号（ISN），将协议截止时间保持为 `ProtocolDeadline::Now`；
    ///   待队列腾出空间后重试 `dispatch`，仍发出序列号一致的 `SYN` 并正常完成三次握手。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 构造 `LOOPBACK_CAPACITY` 个临时 `TcpCore` 并发起连接，将其产生的 `SYN` 包塞满 `fixture` 的发包队列。
    ///   2. 调用待测 `fixture.client` 的 `dispatch`，记录尝试发出的 TCP 序列号，预期因队列已满返回 `Err(FsError::WouldBlock)`。
    ///   3. 排空发包队列中的全部占坑数据包，再次调用 `fixture.client` 的 `dispatch`。
    ///   4. 驱动两端完成后续三次握手并检查连接端点。
    /// - **预期结果**：队列满时 `deadline` 为 `Now`；排空队列后重试发出的 `SYN` 序列号与首次尝试完全一致，
    ///   `deadline` 推进至未来的重传超时时间点，且最终客户端与服务端成功建立连接。
    #[test]
    fn full_owned_queue_preserves_emit_commit(
        context: &mut TestContext<'_>,
        fixture: &mut NetworkOwner,
    ) -> TestResult {
        // Step 1: 创建临时 TCP 端点并塞满底层发包队列（达到 LOOPBACK_CAPACITY 上限）。
        for index in 0..LOOPBACK_CAPACITY {
            let mut endpoint = operation(
                context,
                TcpCore::try_new(64, 64, LOOPBACK_MTU, 3 + index as u64),
            )?;
            let local = InetEndpoint {
                address: CLIENT.address,
                port: 36000 + index as u16,
            };
            operation(context, endpoint.connect(local, SERVER, time::now()))?;
            operation(
                context,
                endpoint.dispatch(time::now(), |packet| fixture.enqueue(packet)),
            )?;
        }

        // Step 2: 在队列满载时触发待测客户端发包，验证返回 WouldBlock 且协议调度截止时间保持为立即重试（Now）。
        let mut attempted = None;
        let failure = fixture.client.lock().dispatch(time::now(), |packet| {
            attempted = Some(packet.tcp()?.1.seq_number);
            fixture.enqueue(packet)
        });
        context.expect_eq(failure, Err(FsError::WouldBlock));
        context.expect_eq(
            fixture.client.lock().deadline(time::now()),
            ProtocolDeadline::Now,
        );

        // Step 3: 排空底层队列以释放缓冲容量，随后重新触发客户端发包，验证重发的 SYN 序列号未发生偏移且重传定时器已启动。
        for _ in 0..LOOPBACK_CAPACITY {
            fixture
                .take_packet()
                .ok_or(TestError("full queue lost an accepted packet"))?;
        }

        let mut submitted = None;
        operation(
            context,
            fixture.client.lock().dispatch(time::now(), |packet| {
                submitted = Some(packet.tcp()?.1.seq_number);
                fixture.enqueue(packet)
            }),
        )?;
        context.expect_eq(submitted, attempted);
        let now = time::now();
        match fixture.client.lock().deadline(now) {
            ProtocolDeadline::At(at) if now < at => {}
            _ => return Err(TestError("accepted SYN has no future protocol deadline")),
        }
        // Step 4: 驱动两端完成剩余三次握手，验证连接成功建立。
        fixture.connect(context)?;
        context.expect_eq(fixture.client.lock().local_endpoint(), Some(CLIENT));
        context.expect_eq(fixture.server.lock().remote_endpoint(), Some(CLIENT));
        Ok(())
    }

    /// 验证用户态缓冲区跨页缺页故障（Page Fault）时的部分传输（Partial Progress）与 `MSG_PEEK` 原子性语义。
    ///
    /// - **测试目标**：确认当用户态 `ReadBuffer` 或 `WriteBuffer` 跨越已映射页与未映射页边界时，
    ///   `TcpCore::receive`（普通读）与 `TcpCore::send` 仅提交故障前位于合法页内的字节数；
    ///   而 `TcpCore::receive(..., true)`（`MSG_PEEK`）在拷贝前缀后遇到未映射页时，只返回已拷贝的合法前缀长度，且不消费接收队列中的任何字节。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 建立 TCP 连接，创建用户地址空间 `Mm` 并仅在 `[ADDRESS, ADDRESS + 4096)` 映射单张可读写匿名页（紧邻的下一页未映射）。
    ///   2. 客户端发送 23 字节数据并等待服务端全部入队。
    ///   3. 服务端构造起始于页尾前 7 字节（`ADDRESS + 4096 - 7`）、长度为 23 字节的跨页用户 `ReadBuffer`：
    ///      先以 `MSG_PEEK` 模式读取，再以普通模式读取，最后用内核缓冲区读取剩余部分。
    ///   4. 在用户页尾前 9 字节写入 `"nine real"`，服务端构造起始于 `ADDRESS + 4096 - 9`、长度为 20 字节的跨页用户 `WriteBuffer` 调用 `send()`，
    ///      随后在客户端接收并校验。
    /// - **预期结果**：跨页 `MSG_PEEK` 返回 `Ready(7)` 且服务端接收队列仍保留全部 23 字节；
    ///   跨页普通读取返回 `Ready(7)` 并消费前 7 字节，后续内核读出剩余 16 字节；
    ///   跨页发送仅提交页内合法的 9 字节，客户端准确收到 `"nine real"`。
    #[test]
    fn user_copy_fault_keeps_exact_stream_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut NetworkOwner,
    ) -> TestResult {
        // Step 1: 建立连接并分配仅映射单张 4KB 物理页的用户地址空间，使 `ADDRESS + 4096` 成为未映射的缺页边界。
        fixture.connect(context)?;
        let memory = Mm::try_new().map_err(|_| TestError("network MM allocation failed"))?;
        memory
            .map_anonymous(ADDRESS, 4096, RW, RW)
            .map_err(|_| TestError("network user page mapping failed"))?;

        // Step 2: 客户端发送测试字节流，等待服务端接收队列完整入队。
        let input = b"real tcp fault boundary";
        let mut source = WriteBuffer::kernel(input);
        context.expect_eq(
            operation(context, fixture.client.lock().send(&mut source))?,
            IoProgress::Ready(input.len()),
        );
        fixture.until(context, || {
            fixture.server.lock().receive_queue() == input.len()
        })?;

        // Step 3: 构造跨越页边界的用户态读取缓冲区（页内仅剩 7 字节可用），验证 MSG_PEEK 与普通读取在缺页边界处的截断行为。
        let mut target = operation(
            context,
            ReadBuffer::user(&memory, ADDRESS + 4096 - 7, input.len()),
        )?;
        context.expect_eq(
            operation(context, fixture.server.lock().receive(&mut target, true))?,
            IoProgress::Ready(7),
        );
        context.expect_eq(fixture.server.lock().receive_queue(), input.len());

        let mut target = operation(
            context,
            ReadBuffer::user(&memory, ADDRESS + 4096 - 7, input.len()),
        )?;
        context.expect_eq(
            operation(context, fixture.server.lock().receive(&mut target, false))?,
            IoProgress::Ready(7),
        );
        let mut output = [0u8; 64];
        context.expect_eq(
            operation(
                context,
                fixture
                    .server
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut output), false),
            )?,
            IoProgress::Ready(input.len() - 7),
        );
        context.expect_eq(&output[..input.len() - 7], &input[7..]);

        // Step 4: 在页尾前 9 字节写入数据，构造跨越页边界的用户态发送缓冲区，验证 send 仅提取并发送页内合法的 9 字节前缀。
        let progress = memory.copyout(ADDRESS + 4096 - 9, b"nine real");
        context.expect_eq(progress.copied, 9);
        context.expect_eq(progress.error, None);
        let mut source = operation(context, WriteBuffer::user(&memory, ADDRESS + 4096 - 9, 20))?;
        context.expect_eq(
            operation(context, fixture.server.lock().send(&mut source))?,
            IoProgress::Ready(9),
        );
        fixture.until(context, || fixture.client.lock().receive_queue() == 9)?;
        context.expect_eq(
            operation(
                context,
                fixture
                    .client
                    .lock()
                    .receive(&mut ReadBuffer::kernel(&mut output), false),
            )?,
            IoProgress::Ready(9),
        );
        context.expect_eq(&output[..9], b"nine real");
        Ok(())
    }
}
