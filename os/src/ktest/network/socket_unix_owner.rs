// UNIX 文件别名、watch 和 pair 的最后拥有由同一套件消费。
#[ktest::suite(fixture = StackOwner, suite_resources = SuiteBaseline, context = "Thread", tags("network-socket", "network-unix-owner"), timeout_ms = 45000)]
pub(crate) mod socket_unix_owner {
    use ::ktest::{TestContext, TestError, TestResult};
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use super::owner::StackOwner;
    use super::socket_helpers::{socket_epoll_collect, socket_read_exact};
    use super::support::{operation, socket_retired};
    use crate::fs::epoll::{EpollEvent, EpollFile, EPOLLET, EPOLL_CTL_ADD};
    use crate::fs::{
        FileIoContext, FileLike, FileRef, FsError, PollMask, ReadBuffer, WriteBuffer,
    };
    use crate::ktest::execution_services::SuiteBaseline;
    use crate::net::socket::{SocketFile, SHUT_WR};
    use crate::net::{IoProgress, UnixSocketType};

    /// UNIX 末 OFD 撤真实 watch，别名保活到最后 put，两个 slot 在根未停止时归还。
    #[test]
    fn unix_last_ofd_watch_weak_and_slots_retire_normally(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        let [first, second] = operation(
            context,
            SocketFile::try_new_unix_pair(fixture.net.clone(), UnixSocketType::Stream, 0, 0, 0),
        )?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 2);
        let alias = first.clone();
        let file_weak = Arc::downgrade(&first);
        let watch_weak = Arc::downgrade(
            first
                .watch_head()
                .ok_or(TestError("UNIX file has no watch"))?,
        );
        let epoll = operation(context, EpollFile::try_new(0))?;
        {
            let owner: FileRef = epoll.clone();
            let target: FileRef = first.clone();
            operation(
                context,
                epoll.control(
                    &owner,
                    EPOLL_CTL_ADD,
                    &target,
                    0,
                    Some(EpollEvent {
                        events: PollMask::POLLIN.0 | EPOLLET,
                        data: first.id(),
                    }),
                ),
            )?;
        }

        drop(first);
        context.expect_eq(file_weak.upgrade().is_some(), true);
        context.expect_eq(
            operation(
                context,
                second.send(
                    &mut WriteBuffer::kernel(b"last alias bytes"),
                    None,
                    0,
                    FileIoContext::Kernel,
                ),
            )?,
            16,
        );
        let mut events = [EpollEvent::default(); 1];
        context.expect_eq(
            operation(context, socket_epoll_collect(&epoll, &mut events))?,
            1,
        );
        context.expect_eq(events[0].data, alias.id());
        let mut output = [0u8; 16];
        socket_read_exact(&alias, &mut output)?;
        context.expect_eq(&output, b"last alias bytes");

        operation(context, alias.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, second.read(&mut [0u8; 1]))?, 0);
        operation(context, second.shutdown(SHUT_WR))?;
        context.expect_eq(operation(context, alias.read(&mut [0u8; 1]))?, 0);

        drop(alias);
        context.expect_eq(file_weak.upgrade().is_none(), true);
        context.expect_eq(watch_weak.upgrade().is_none(), true);
        context.expect_eq(
            operation(context, socket_epoll_collect(&epoll, &mut events))?,
            0,
        );
        drop((second, epoll));
        socket_retired(&fixture.net)?;

        let [first, second] = operation(
            context,
            fixture.net.try_new_unix_pair(UnixSocketType::Stream),
        )?;
        let first_weak = Arc::downgrade(&first);
        let second_weak = Arc::downgrade(&second);
        context.expect_eq(first.id() != second.id(), true);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 2);
        context.expect_eq(
            operation(context, first.send(b"endpoint bytes"))?,
            IoProgress::Ready(14),
        );
        context.expect_eq(
            operation(
                context,
                second.receive(&mut ReadBuffer::kernel(&mut output), false),
            )?,
            IoProgress::Ready(14),
        );
        context.expect_eq(&output[..14], b"endpoint bytes");
        first.close();
        second.close();
        drop((first, second));
        context.expect_eq(first_weak.upgrade().is_none(), true);
        context.expect_eq(second_weak.upgrade().is_none(), true);
        socket_retired(&fixture.net)?;

        // pair 准备面对真实堆耗尽时不得留下任一未发表端点；放压后原根继续创建。
        let mut pressure = crate::cache::ktest::HeapPressure::prepare()?;
        let evidence = pressure.exhaust();
        let failed = matches!(
            fixture.net.try_new_unix_pair(UnixSocketType::Stream),
            Err(FsError::NoMemory)
        );
        let live = fixture.net.ktest_live_endpoints();
        drop(pressure);
        context.expect_eq(evidence, (true, true));
        context.expect_eq(failed, true);
        context.expect_eq(live, 0);
        if evidence != (true, true) || !failed || live != 0 {
            return Err(TestError("UNIX pair OOM left partial endpoint ownership"));
        }
        socket_retired(&fixture.net)?;

        let [first, second] = operation(
            context,
            fixture.net.try_new_unix_pair(UnixSocketType::Stream),
        )?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 2);
        first.close();
        second.close();
        drop((first, second));
        socket_retired(&fixture.net)?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        drop((file_weak, watch_weak, first_weak, second_weak));
        Ok(())
    }

    /// 跨越旧端点和 pair 数量边界，逐对交付真实字节；最后别名结束前仍保留唯一活端点。
    #[test]
    #[case("stream129", 129usize, UnixSocketType::Stream)]
    #[case("stream1024", 1024usize, UnixSocketType::Stream)]
    #[case("seqpacket129", 129usize, UnixSocketType::SeqPacket)]
    #[ktest(tags("network-growth"))]
    fn unix_pairs_grow_and_last_alias_retires_each_pair(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
        count: usize,
        kind: UnixSocketType,
    ) -> TestResult {
        let mut pairs = Vec::new();
        pairs
            .try_reserve_exact(count)
            .map_err(|_| TestError("UNIX growth observers OOM"))?;
        for _ in 0..count {
            pairs.push(operation(
                context,
                SocketFile::try_new_unix_pair(fixture.net.clone(), kind, 0, 0, 0),
            )?);
        }
        context.expect_eq(fixture.net.ktest_live_endpoints(), count * 2);

        let alias = pairs[0][0].clone();
        let weak = Arc::downgrade(&alias);
        for (index, [sender, receiver]) in pairs.into_iter().enumerate() {
            let input: [u8; 17] =
                ::core::array::from_fn(|offset| super::support::pattern(index * 17 + offset));
            context.expect_eq(operation(context, sender.write(&input))?, input.len());
            let mut output = [0u8; 17];
            socket_read_exact(&receiver, &mut output)?;
            context.expect_eq(output, input);
            drop((sender, receiver));
        }

        context.expect_eq(weak.upgrade().is_some(), true);
        context.expect_eq(fixture.net.ktest_live_endpoints(), 1);
        context.expect_eq(
            alias.send(
                &mut WriteBuffer::kernel(b"closed peer"),
                None,
                0,
                FileIoContext::Kernel,
            ),
            Err(FsError::BrokenPipe),
        );

        drop(alias);
        context.expect_eq(weak.upgrade().is_none(), true);
        socket_retired(&fixture.net)?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }
}
