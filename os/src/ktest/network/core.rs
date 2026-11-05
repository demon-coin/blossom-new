// 完整 inline suite 保留原 module_path 与静态注册身份。
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

    /// 超过 ring 容量的全文经过真正 TCP 包，PEEK 不消费，FIN 后先交付尾部再 EOF。
    #[test]
    fn owned_packets_stream_wrap_peek_and_fin(
        context: &mut TestContext<'_>,
        fixture: &mut NetworkOwner,
    ) -> TestResult {
        fixture.connect(context)?;
        let input: [u8; 1025] = ::core::array::from_fn(pattern);
        let mut output = [0u8; 1025];
        let mut sent = 0;
        let mut received = 0;
        let end = time::now()
            .checked_add(time::Duration::from_secs(5))
            .unwrap();

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

    /// 真队列满使 SYN 未提交，领取容量后同一核心仍发出可完成握手的原始 SYN。
    #[test]
    fn full_owned_queue_preserves_emit_commit(
        context: &mut TestContext<'_>,
        fixture: &mut NetworkOwner,
    ) -> TestResult {
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
        fixture.connect(context)?;
        context.expect_eq(fixture.client.lock().local_endpoint(), Some(CLIENT));
        context.expect_eq(fixture.server.lock().remote_endpoint(), Some(CLIENT));
        Ok(())
    }

    /// 用户 copyout 与 copyin 真实跨页故障只提交已交付前缀，PEEK 故障不消费 ring。
    #[test]
    fn user_copy_fault_keeps_exact_stream_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut NetworkOwner,
    ) -> TestResult {
        fixture.connect(context)?;
        let memory = Mm::try_new().map_err(|_| TestError("network MM allocation failed"))?;
        memory
            .map_anonymous(ADDRESS, 4096, RW, RW)
            .map_err(|_| TestError("network user page mapping failed"))?;

        let input = b"real tcp fault boundary";
        let mut source = WriteBuffer::kernel(input);
        context.expect_eq(
            operation(context, fixture.client.lock().send(&mut source))?,
            IoProgress::Ready(input.len()),
        );
        fixture.until(context, || {
            fixture.server.lock().receive_queue() == input.len()
        })?;

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
