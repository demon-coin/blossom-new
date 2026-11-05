// 共享原页 FIFO 的字节、跨页 fault 与真实原页压力归同一套件。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-socket", "network-unix-stream"), timeout_ms = 45000)]
pub(crate) mod socket_unix_stream {
    use ::ktest::{TestContext, TestError, TestResult};
    use alloc::vec::Vec;

    use super::owner::StackOwner;
    use super::support::{operation, pattern, socket_retired, ADDRESS, RW};
    use crate::fs::{FsError, PollMask, ReadBuffer, WriteBuffer};
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::mm::space::Mm;
    use crate::net::{IoProgress, UnixSocketType};

    /// 四页槽满后读一字节不归槽，跨槽 PEEK 部分 fault 保留全文；整页释放才写就绪。
    #[test]
    fn unix_shared_fifo_slots_and_cross_page_peek_fault(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let [sender, receiver] = operation(
            context,
            fixture.net.try_new_unix_pair(UnixSocketType::Stream),
        )?;
        let mut fifo = crate::fs::pipe::PipeBuffer::<4>::new();
        let page: [u8; 4096] = ::core::array::from_fn(pattern);
        for _ in 0..4 {
            context.expect_eq(
                operation(context, sender.send(&page))?,
                IoProgress::Ready(page.len()),
            );
            context.expect_eq(
                operation(context, fifo.append(&mut WriteBuffer::kernel(&page)))?,
                page.len(),
            );
        }

        let mut bytes = [0u8; 4096];
        for count in [1, 4088] {
            context.expect_eq(
                operation(
                    context,
                    receiver.receive(&mut ReadBuffer::kernel(&mut bytes[..count]), false),
                )?,
                IoProgress::Ready(count),
            );
            context.expect_eq(
                operation(
                    context,
                    fifo.receive(&mut ReadBuffer::kernel(&mut bytes[..count]), false),
                )?,
                count,
            );
            context.expect_eq(
                operation(context, sender.poll(None))?.contains(PollMask::POLLOUT),
                false,
            );
            context.expect_eq(fifo.empty_slot_capacity(), 0);
        }

        let memory = Mm::try_new().map_err(|_| TestError("UNIX MM allocation failed"))?;
        memory
            .map_anonymous(ADDRESS, 4096, RW, RW)
            .map_err(|_| TestError("UNIX fault page mapping failed"))?;
        for unix in [true, false] {
            let mut target = operation(context, ReadBuffer::user(&memory, ADDRESS + 4096 - 9, 32))?;
            if unix {
                context.expect_eq(
                    operation(context, receiver.receive(&mut target, true))?,
                    IoProgress::Ready(9),
                );
            } else {
                context.expect_eq(operation(context, fifo.receive(&mut target, true))?, 9);
            }
            context.expect_eq(target.copied(), 9);
            let mut peeked = [0u8; 9];
            context.expect_eq(memory.copyin(&mut peeked, ADDRESS + 4096 - 9).copied, 9);
            context.expect_eq(&peeked[..7], &page[4089..]);
            context.expect_eq(&peeked[7..], &page[..2]);
        }
        context.expect_eq(
            operation(context, sender.poll(None))?.contains(PollMask::POLLOUT),
            false,
        );

        context.expect_eq(
            operation(
                context,
                receiver.receive(&mut ReadBuffer::kernel(&mut bytes[..7]), false),
            )?,
            IoProgress::Ready(7),
        );
        context.expect_eq(&bytes[..7], &page[4089..]);
        context.expect_eq(
            operation(
                context,
                fifo.receive(&mut ReadBuffer::kernel(&mut bytes[..7]), false),
            )?,
            7,
        );
        context.expect_eq(&bytes[..7], &page[4089..]);
        context.expect_eq(
            operation(context, sender.poll(None))?.contains(PollMask::POLLOUT),
            true,
        );
        context.expect_eq(fifo.empty_slot_capacity(), page.len());

        for _ in 0..3 {
            context.expect_eq(
                operation(
                    context,
                    receiver.receive(&mut ReadBuffer::kernel(&mut bytes), false),
                )?,
                IoProgress::Ready(page.len()),
            );
            context.expect_eq(bytes, page);
            context.expect_eq(
                operation(
                    context,
                    fifo.receive(&mut ReadBuffer::kernel(&mut bytes), false),
                )?,
                page.len(),
            );
            context.expect_eq(bytes, page);
        }

        sender.close();
        receiver.close();
        drop((sender, receiver, fifo, memory));
        socket_retired(&fixture.net)
    }

    /// 正常原页压力使新槽零进度 ENOMEM，旧页保图样，放压后同方向再次发送。
    #[test]
    fn unix_shared_fifo_real_page_oom_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let [sender, receiver] = operation(
            context,
            fixture.net.try_new_unix_pair(UnixSocketType::Stream),
        )?;
        let mut fifo = crate::fs::pipe::PipeBuffer::<4>::new();
        let page = [0x79; 4096];
        context.expect_eq(
            operation(context, sender.send(&page))?,
            IoProgress::Ready(page.len()),
        );
        context.expect_eq(
            operation(context, fifo.append(&mut WriteBuffer::kernel(&page)))?,
            page.len(),
        );

        let mut pressure = Vec::new();
        pressure
            .try_reserve_exact(crate::mm::stats().present_pages)
            .map_err(|_| TestError("UNIX page pressure slots OOM"))?;
        crate::mm::shrink_all();
        crate::mm::drain_all_pages();
        for order in [10, 0] {
            while let Some(page) = crate::mm::alloc_pages(order, false) {
                pressure.push(page);
            }
        }
        crate::mm::drain_all_pages();
        context.expect_eq(crate::mm::alloc_pages(0, false).is_none(), true);
        context.expect_eq(sender.send(b"new page"), Err(FsError::NoMemory));
        let mut source = WriteBuffer::kernel(b"new page");
        context.expect_eq(fifo.append(&mut source), Err(FsError::NoMemory));
        context.expect_eq(source.copied(), 0);

        let mut bytes = [0u8; 4096];
        context.expect_eq(
            receiver.receive(&mut ReadBuffer::kernel(&mut bytes), false),
            Ok(IoProgress::Ready(page.len())),
        );
        context.expect_eq(bytes, page);
        context.expect_eq(
            fifo.receive(&mut ReadBuffer::kernel(&mut bytes), false),
            Ok(page.len()),
        );
        context.expect_eq(bytes, page);

        drop(pressure);
        context.expect_eq(
            operation(context, sender.send(b"after OOM"))?,
            IoProgress::Ready(9),
        );
        context.expect_eq(
            operation(
                context,
                receiver.receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?,
            IoProgress::Ready(9),
        );
        context.expect_eq(&bytes[..9], b"after OOM");
        context.expect_eq(
            operation(context, fifo.append(&mut WriteBuffer::kernel(b"after OOM")))?,
            9,
        );
        context.expect_eq(
            operation(
                context,
                fifo.receive(&mut ReadBuffer::kernel(&mut bytes), false),
            )?,
            9,
        );
        context.expect_eq(&bytes[..9], b"after OOM");

        sender.close();
        receiver.close();
        drop((sender, receiver, fifo));
        socket_retired(&fixture.net)
    }
}
