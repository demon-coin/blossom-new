/// UNIX 域套接字对（`socketpair`）引用计数生命周期、`epoll` 观察链表注销与扩容内核测试套件。
///
/// ## 1. 模块职责与架构定位
/// 本模块位于 `ktest::network` 测试子系统中，负责验证 UNIX 域套接字文件（`SocketFile`）及其底层端点对
/// （`UnixEndpoint`）在多别名共享、`epoll` 事件监听（`WatchHead`）挂载、堆内存耗尽（`ENOMEM`）原子回滚
/// 以及大规模扩容场景下的所有权与资源回收行为。
///
/// ## 2. 核心生命周期与引用关系
/// ```text
///   SocketFile (Arc 强引用: first, alias)
///       ├──► WatchHead (Arc 强引用，挂载在 EpollFile 监听树中)
///       └──► UnixEndpoint (注册在 NetworkStack 全局活跃端点槽位表)
///
///   生命周期演进：
///   1. drop(first) ──► 仍存在 alias 强引用：WatchHead 继续向 epoll 投递 POLLIN，端点继续正常收发
///   2. drop(alias) ──► 最后一个 SocketFile 强引用释放（Drop）：
///                      - 自动销毁 WatchHead 并从 epoll 移除监听项
///                      - 关闭底层 UnixEndpoint，触发对端收到 EOF / BrokenPipe
///   3. drop(second) ─► 成对的两个 UnixEndpoint 槽位全部从全局端点表归还（live_endpoints == 0）
/// ```
///
/// ## 3. 锁与并发顺序说明
/// - 本套件在单线程上下文中顺序驱动 `SocketFile`、`EpollFile` 与 `HeapPressure`，创建套接字对时若遭遇
///   `ENOMEM`，必须在内部锁临界区外原子释放已半创建的配对端点，不得在全局端点表中遗留孤立槽位。
/// UNIX 域套接字文件引用生命周期、`epoll` 监听项自动清理与大规模套接字对扩容测试套件。
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

    /// 测试目标：验证 UNIX 流式套接字对在多个 `Arc<SocketFile>` 别名共享时，仅当最后一个强引用被释放（Drop）后才会注销 `epoll` 监听链表并关闭端点，同时验证 `try_new_unix_pair` 在内存耗尽（`ENOMEM`）时不会泄漏半创建的端点槽位。
    ///
    /// 场景步骤：
    /// 1. 创建一对 UNIX 流式 `SocketFile`（`[first, second]`），克隆 `first` 得到 `alias`，并将 `first` 注册到 `epoll` 监听 `POLLIN | EPOLLET`；
    /// 2. 丢弃 `first` 引用，验证因 `alias` 存活，文件与 `watch_head` 弱引用均未失效，且 `second` 发送的数据仍能触发 `epoll` 事件并由 `alias` 完整读取；
    /// 3. 双向执行 `shutdown(SHUT_WR)` 并确认两端读取均返回 EOF（0 字节）；
    /// 4. 丢弃最后一个引用 `alias`，验证 `file_weak` 与 `watch_weak` 立即失效、`epoll` 不再返回事件，随后丢弃 `second` 并确认两个端点槽位全部归还；
    /// 5. 直接在底层 `UnixEndpoint` 对上验证收发与关闭回收；
    /// 6. 使用 `HeapPressure::exhaust` 耗尽堆内存后调用 `try_new_unix_pair`，验证返回 `FsError::NoMemory` 且活跃端点计数保持为 0，解除压力后再次创建与回收恢复正常。
    #[test]
    fn unix_last_ofd_watch_weak_and_slots_retire_normally(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建 UNIX 流式套接字对，保存 first 的额外强引用 alias 与弱引用，并将 first 注册到 epoll。
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

        // Step 2: 释放初始的 first 引用，验证最后一个强引用 alias 仍保持套接字与 epoll 监听有效。
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

        // Step 3: 双向关闭写方向（SHUT_WR），验证两端均观察到 EOF，随后释放最后一个强引用 alias 并确认 watch 与文件对象同步销毁。
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

        // Step 4: 验证直接创建的底层 UnixEndpoint 对在收发数据及关闭后能正常销毁并归还槽位。
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

        // Step 5: 在堆内存耗尽状态下创建 UNIX 套接字对，验证分配失败时具备原子回滚保证，不会残留半边未发布的端点；解除内存压力后恢复正常创建。
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

    /// 测试目标：验证批量创建 129 对或 1024 对 UNIX 套接字（覆盖 `Stream` 与 `SeqPacket` 两种类型并跨越端点表扩容边界）时，每一对均可独立收发数据；且当所有配对释放后，仅被单独保留强引用 `alias` 的那一个端点保持存活并对已关闭对端返回 `BrokenPipe`，释放 `alias` 后全部槽位归零。
    ///
    /// 场景步骤：
    /// 1. 连续创建 `count` 对指定类型的 UNIX `SocketFile`，确认全局活跃端点数量达到 `count * 2`；
    /// 2. 克隆第 0 对首个端点的强引用 `alias`，随后遍历全部 `count` 对套接字，逐对写入 17 字节特征数据、校验对端完整读出，并立即丢弃该对的两个端点引用；
    /// 3. 验证遍历结束后全局活跃端点数恰好降为 1（即 `alias`），且通过 `alias` 向已关闭的对端发送数据返回 `FsError::BrokenPipe`；
    /// 4. 丢弃 `alias`，验证弱引用失效且全局活跃端点数归零。
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
        // Step 1: 批量创建 count 对 UNIX 套接字，触发底层端点槽位表扩容并验证活跃端点数为 count * 2。
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

        // Step 2: 保留首个端点的额外强引用 alias，逐对验证数据收发并释放每一对的原始引用。
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

        // Step 3: 验证其余 2 * count - 1 个端点均已回收，仅剩 alias 存活，且向已销毁对端写入返回 BrokenPipe。
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

        // Step 4: 释放最后一个强引用 alias，确认全部端点槽位回收完毕。
        drop(alias);
        context.expect_eq(weak.upgrade().is_none(), true);
        socket_retired(&fixture.net)?;
        context.expect_eq(fixture.net.ktest_live_endpoints(), 0);
        context.expect_eq(fixture.net.is_stopping(), false);
        Ok(())
    }
}
