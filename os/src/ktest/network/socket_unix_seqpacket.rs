// UNIX seqpacket 的记录边界、背压与半关闭保持在同一行为套件。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-socket", "network-unix-seqpacket"), timeout_ms = 45000)]
pub(crate) mod socket_unix_seqpacket {
    use ::ktest::{TestContext, TestResult};
    use alloc::sync::Arc;

    use super::owner::StackOwner;
    use super::support::{operation, pattern, socket_retired};
    use crate::fs::{
        FileIoContext, FileLike, FsError, PollMask, Pollable, ReadBuffer, WriteBuffer, O_NONBLOCK,
    };
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::net::socket::{SocketFile, MSG_NOSIGNAL, MSG_PEEK, MSG_TRUNC, SHUT_WR};
    use crate::net::UnixSocketType;
    /// 有序记录不合并；PEEK、短读和零容量 recv 保原长，空消息与关闭 EOF 分别消费。
    #[test]
    fn unix_seqpacket_record_peek_truncation_empty_and_shutdown(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let [sender, receiver] = operation(
            context,
            SocketFile::try_new_unix_pair(
                fixture.net.clone(),
                UnixSocketType::SeqPacket,
                O_NONBLOCK,
                0,
                0,
            ),
        )?;
        context.expect_eq(sender.socket_type(), 5);
        context.expect_eq(receiver.socket_type(), 5);
        for bytes in [b"first".as_slice(), b"second".as_slice()] {
            context.expect_eq(
                operation(
                    context,
                    sender.send(
                        &mut WriteBuffer::kernel(bytes),
                        None,
                        0,
                        FileIoContext::Kernel,
                    ),
                )?,
                bytes.len(),
            );
        }
        let mut prefix = [0u8; 2];
        let peek = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut prefix),
                MSG_PEEK,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq((peek.copied, peek.original_len, peek.flags), (2, 5, 0x20));
        context.expect_eq(&prefix, b"fi");
        context.expect_eq(peek.source, None);
        let short = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut prefix),
                MSG_TRUNC,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq(
            (short.copied, short.original_len, short.flags),
            (2, 5, 0x20),
        );
        context.expect_eq(&prefix, b"fi");
        let mut output = [0u8; 32];
        let next = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                0,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq((next.copied, next.original_len, next.flags), (6, 6, 0));
        context.expect_eq(&output[..next.copied], b"second");
        context.expect_eq(
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::WouldBlock),
        );

        operation(
            context,
            sender.send(
                &mut WriteBuffer::kernel(b"discard"),
                None,
                0,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq(operation(context, receiver.read(&mut []))?, 0);
        context.expect_eq(
            operation(context, receiver.poll(None))?.contains(PollMask::POLLIN),
            true,
        );
        let zero_capacity = operation(
            context,
            receiver.receive(&mut ReadBuffer::kernel(&mut []), 0, FileIoContext::Kernel),
        )?;
        context.expect_eq(
            (
                zero_capacity.copied,
                zero_capacity.original_len,
                zero_capacity.flags,
            ),
            (0, 7, 0x20),
        );
        context.expect_eq(
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::WouldBlock),
        );

        context.expect_eq(
            operation(
                context,
                sender.send(
                    &mut WriteBuffer::kernel(&[]),
                    None,
                    0,
                    FileIoContext::Kernel,
                ),
            )?,
            0,
        );
        context.expect_eq(operation(context, receiver.read(&mut []))?, 0);
        context.expect_eq(
            operation(context, receiver.poll(None))?.contains(PollMask::POLLIN),
            true,
        );
        let empty_peek = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                MSG_PEEK,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq(
            (empty_peek.copied, empty_peek.original_len, empty_peek.flags),
            (0, 0, 0),
        );
        context.expect_eq(
            operation(context, receiver.poll(None))?.contains(PollMask::POLLIN),
            true,
        );
        let empty = operation(
            context,
            receiver.receive(&mut ReadBuffer::kernel(&mut []), 0, FileIoContext::Kernel),
        )?;
        context.expect_eq((empty.copied, empty.original_len, empty.flags), (0, 0, 0));
        context.expect_eq(
            operation(context, receiver.poll(None))?.contains(PollMask::POLLIN),
            false,
        );
        context.expect_eq(
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::WouldBlock),
        );
        for bytes in [b"".as_slice(), b"tail".as_slice()] {
            context.expect_eq(
                operation(
                    context,
                    sender.send(
                        &mut WriteBuffer::kernel(bytes),
                        None,
                        0,
                        FileIoContext::Kernel,
                    ),
                )?,
                bytes.len(),
            );
        }
        operation(context, sender.shutdown(SHUT_WR))?;
        let empty = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                0,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq((empty.copied, empty.original_len, empty.flags), (0, 0, 0));
        context.expect_eq(operation(context, receiver.read(&mut output))?, 4);
        context.expect_eq(&output[..4], b"tail");
        context.expect_eq(operation(context, receiver.read(&mut output))?, 0);
        context.expect_eq(
            operation(context, receiver.poll(None))?.contains(PollMask::POLLRDHUP),
            true,
        );
        context.expect_eq(
            operation(
                context,
                receiver.send(
                    &mut WriteBuffer::kernel(b"reply"),
                    None,
                    0,
                    FileIoContext::Kernel,
                ),
            )?,
            5,
        );
        operation(context, receiver.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, sender.read(&mut output))?, 5);
        context.expect_eq(&output[..5], b"reply");
        context.expect_eq(operation(context, sender.read(&mut output))?, 0);
        for endpoint in [&sender, &receiver] {
            context.expect_eq(
                endpoint.send(
                    &mut WriteBuffer::kernel(&[]),
                    None,
                    MSG_NOSIGNAL,
                    FileIoContext::Kernel,
                ),
                Err(FsError::BrokenPipe),
            );
        }
        drop((sender, receiver));
        socket_retired(&fixture.net)?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }

    /// 字节预算和32记录槽分别施加非阻塞背压，读取释放额度，末别名结束才归还节点。
    #[test]
    fn unix_seqpacket_capacity_and_last_ofd_return_slots(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let [sender, receiver] = operation(
            context,
            SocketFile::try_new_unix_pair(
                fixture.net.clone(),
                UnixSocketType::SeqPacket,
                O_NONBLOCK,
                0,
                0,
            ),
        )?;
        let alias = sender.clone();
        let weak = Arc::downgrade(&sender);
        drop(sender);
        context.expect_eq(weak.upgrade().is_some(), true);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 2);
        let mut payload = operation(context, crate::net::packet::zeroed_bytes(16385))?;
        for (index, byte) in payload.iter_mut().enumerate() {
            *byte = pattern(index);
        }
        context.expect_eq(
            alias.send(
                &mut WriteBuffer::kernel(&payload),
                None,
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::MessageTooLarge),
        );
        let mut output = operation(context, crate::net::packet::zeroed_bytes(16384))?;
        context.expect_eq(
            operation(
                context,
                alias.send(
                    &mut WriteBuffer::kernel(&payload[..16384]),
                    None,
                    0,
                    FileIoContext::Kernel,
                ),
            )?,
            16384,
        );
        let maximum = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                0,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq(
            (maximum.copied, maximum.original_len, maximum.flags),
            (16384, 16384, 0),
        );
        context.expect_eq(output.as_slice(), &payload[..16384]);
        for bytes in [&payload[..8193], &payload[8193..16384]] {
            context.expect_eq(
                operation(
                    context,
                    alias.send(
                        &mut WriteBuffer::kernel(bytes),
                        None,
                        0,
                        FileIoContext::Kernel,
                    ),
                )?,
                bytes.len(),
            );
        }
        context.expect_eq(
            operation(context, alias.poll(None))?.contains(PollMask::POLLOUT),
            false,
        );
        context.expect_eq(
            alias.send(
                &mut WriteBuffer::kernel(&[]),
                None,
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::WouldBlock),
        );
        let mut first = [0u8; 1];
        let peek = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut first),
                MSG_PEEK,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq(
            (peek.copied, peek.original_len, peek.flags),
            (1, 8193, 0x20),
        );
        context.expect_eq(first[0], payload[0]);
        context.expect_eq(
            operation(context, alias.poll(None))?.contains(PollMask::POLLOUT),
            false,
        );
        let short = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut first),
                0,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq(
            (short.copied, short.original_len, short.flags),
            (1, 8193, 0x20),
        );
        context.expect_eq(
            operation(context, alias.poll(None))?.contains(PollMask::POLLOUT),
            true,
        );
        for _ in 0..31 {
            context.expect_eq(
                operation(
                    context,
                    alias.send(
                        &mut WriteBuffer::kernel(&[]),
                        None,
                        0,
                        FileIoContext::Kernel,
                    ),
                )?,
                0,
            );
        }
        context.expect_eq(
            operation(context, alias.poll(None))?.contains(PollMask::POLLOUT),
            false,
        );
        context.expect_eq(
            alias.send(
                &mut WriteBuffer::kernel(b"z"),
                None,
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::WouldBlock),
        );
        let second = operation(
            context,
            receiver.receive(
                &mut ReadBuffer::kernel(&mut output),
                0,
                FileIoContext::Kernel,
            ),
        )?;
        context.expect_eq(
            (second.copied, second.original_len, second.flags),
            (8191, 8191, 0),
        );
        context.expect_eq(&output[..second.copied], &payload[8193..16384]);
        context.expect_eq(
            operation(
                context,
                alias.send(
                    &mut WriteBuffer::kernel(b"z"),
                    None,
                    0,
                    FileIoContext::Kernel,
                ),
            )?,
            1,
        );
        for _ in 0..31 {
            let empty = operation(
                context,
                receiver.receive(
                    &mut ReadBuffer::kernel(&mut first),
                    0,
                    FileIoContext::Kernel,
                ),
            )?;
            context.expect_eq((empty.copied, empty.original_len, empty.flags), (0, 0, 0));
        }
        context.expect_eq(operation(context, receiver.read(&mut first))?, 1);
        context.expect_eq(&first, b"z");
        context.expect_eq(
            receiver.receive(
                &mut ReadBuffer::kernel(&mut first),
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::WouldBlock),
        );
        drop(alias);
        context.expect_eq(weak.upgrade().is_none(), true);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 1);
        context.expect_eq(operation(context, receiver.read(&mut first))?, 0);
        drop(receiver);
        socket_retired(&fixture.net)?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        drop((weak, payload, output));
        Ok(())
    }
}
