/// UNIX 域字节流套接字（`SOCK_STREAM`）与底层管道（`PipeBuffer`）共享物理页环形队列内核测试套件。
///
/// ## 1. 模块职责与架构定位
/// 本模块位于 `ktest::network` 测试子系统中，验证 `UnixSocketType::Stream` 套接字对与内核管道
/// `crate::fs::pipe::PipeBuffer<4>` 共享的 4 页（`4 * 4096 B`）物理页槽位环形队列行为，涵盖整页槽位
/// 释放阈值、跨页 `MSG_PEEK` 遇到未映射用户页缺页异常（Page Fault）时的部分拷贝进展，以及物理页分配器
/// 耗尽（`ENOMEM`）下的无损恢复。
///
/// ## 2. 4 页物理页槽位（`PipeBuffer<4>`）布局与跨页缺页示意
/// ```text
///   物理页环形槽位 (4 个 4096B 物理页槽位)：
///   ┌────────────────────────┬────────────────────────┬────────────┬────────────┐
///   │ Slot 0 (4096B)         │ Slot 1 (4096B)         │ Slot 2     │ Slot 3     │
///   │ [已读 4089B | 剩余 7B] │ [未读 4096B: 0..2 ...] │ [4096B]    │ [4096B]    │
///   └────────────────────────┴────────────────────────┴────────────┴────────────┘
///                 │            │
///                 ▼            ▼
///   用户态目标缓冲区 ReadBuffer::user(ADDRESS + 4096 - 9, len = 32)：
///   ┌──────────────────────────┬──────────────────────────┐
///   │ 已映射页 [ADDRESS..+4K)  │ 未映射页 [ADDRESS+4K..)  │
///   │ 末尾 9B 可写：           │ 触发缺页错误 (Fault)：   │
///   │ 写入 Slot 0 尾部 7B      │ 停止后续拷贝，返回       │
///   │ + Slot 1 头部 2B (共 9B) │ 已成功拷贝的 9B          │
///   └──────────────────────────┴──────────────────────────┘
/// ```
/// - **整页释放才恢复可写（`POLLOUT`）**：当 4 个物理页槽位全部写满后，仅读取部分字节（如读取 4089 字节，
///   Slot 0 仍剩 7 字节未消费）不会释放物理页槽位，发送端 `POLLOUT` 仍为 `false`；直到 Slot 0 剩余 7 字节
///   被正式消费（非 `PEEK`），整张物理页归还并腾出空槽位，`POLLOUT` 才恢复为 `true`。
///
/// ## 3. 锁与并发顺序说明
/// - 本套件在单线程上下文中同步对比 `UnixEndpoint` 与 `PipeBuffer<4>` 的行为，物理页耗尽测试通过
///   `mm::alloc_pages` 抽干伙伴系统空闲页并在断言后立即释放，不在持锁期间跨核等待。
/// UNIX `SOCK_STREAM` 字节流套接字与 `PipeBuffer<4>` 物理页槽位管理、跨页窥视缺页与页耗尽恢复测试套件。
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

    /// 测试目标：验证 UNIX 流式套接字与 4 槽位 `PipeBuffer<4>` 在写满 4 张物理页后，部分读取不会提前释放槽位或恢复 `POLLOUT`；跨两个物理页槽位向用户态缓冲区执行 `MSG_PEEK` 并在第二页边界遭遇未映射缺页时，能准确返回已拷贝的 9 字节且不消费队列数据；正式读完首槽剩余 7 字节腾出整张物理页后，`POLLOUT` 立即恢复。
    ///
    /// 场景步骤：
    /// 1. 创建 UNIX `Stream` 套接字对与 `PipeBuffer::<4>::new()`，向两者各写入 4 张完整的 4096 字节特征页；
    /// 2. 先后从两者各读取 1 字节和 4088 字节（首张物理页累计消费 4089 字节，剩余 7 字节），验证此时首个物理页槽位尚未腾空，`sender` 的 `POLLOUT` 为 `false` 且 `fifo.empty_slot_capacity()` 为 0；
    /// 3. 创建独立用户地址空间 `Mm`，仅映射 `[ADDRESS, ADDRESS + 4096)` 单页，构造起始于 `ADDRESS + 4096 - 9`、长度为 32 字节（跨越已映射页末尾 9 字节与未映射下一页）的用户态 `ReadBuffer`；
    /// 4. 分别对 UNIX 接收端和 `fifo` 执行 `peek = true` 的读取，验证两者均将首槽尾部 7 字节与次槽头部 2 字节（共 9 字节）拷入用户页，并在触碰未映射页时停止，返回成功拷贝的 9 字节且队列消费位置不变；
    /// 5. 正式读取（`peek = false`）7 字节以排空首张物理页槽位，验证 `sender` 的 `POLLOUT` 恢复为 `true` 且 `fifo` 腾出 4096 字节空槽容量；
    /// 6. 依次读出剩余 3 张完整的 4096 字节物理页并校验内容一致。
    #[test]
    fn unix_shared_fifo_slots_and_cross_page_peek_fault(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建 UNIX 流式套接字对与对照的 PipeBuffer<4>，分别写满 4 张 4096 字节物理页。
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

        // Step 2: 从首张物理页消费 1 + 4088 = 4089 字节（剩 7 字节未读），验证槽位尚未释放且不可写（POLLOUT 为 false）。
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

        // Step 3: 构造仅前 9 字节已映射、后续跨入未映射区的用户态缓冲区，验证跨物理页槽位的 PEEK 读取在缺页处返回已成功拷贝的 9 字节且不消费原队列数据。
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

        // Step 4: 正式消费首张物理页剩余的 7 字节，验证整页槽位释放后立即恢复可写（POLLOUT 为 true）。
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

        // Step 5: 依次读出剩余 3 个槽位中的完整 4096 字节页并校验数据完整性。
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

    /// 测试目标：验证当系统物理页分配器耗尽（`alloc_pages` 返回 `None`）时，向已占用 1 个槽位的 UNIX 流式套接字或 `PipeBuffer<4>` 追加新页会以 0 字节进度返回 `FsError::NoMemory`，且已入队的旧物理页数据毫发无损，释放物理页压力后可继续正常收发。
    ///
    /// 场景步骤：
    /// 1. 创建 UNIX `Stream` 套接字对与 `PipeBuffer<4>`，各写入 1 张 4096 字节的 `0x79` 特征页（占满 Slot 0）；
    /// 2. 回收缓存并抽干伙伴系统中所有阶（order 10 与 order 0）的空闲物理页，确认 `alloc_pages(0, false)` 返回 `None`；
    /// 3. 尝试向 `sender` 和 `fifo` 写入 `"new page"`（需为 Slot 1 分配新物理页），验证两者均返回 `Err(FsError::NoMemory)` 且源缓冲区 `copied()` 为 0；
    /// 4. 读取 `receiver` 与 `fifo` 中原有的 4096 字节物理页，验证数据完整无损；
    /// 5. 释放占用的全部物理页，验证 `sender` 与 `fifo` 均可重新分配物理页并正常收发 `"after OOM"`。
    #[test]
    fn unix_shared_fifo_real_page_oom_recovers(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建 UNIX 流式套接字对与 PipeBuffer<4>，各写入 1 张完整的 4096 字节数据页。
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

        // Step 2: 抽干伙伴系统全部可用物理页，验证追加写入新槽位时因无法分配物理页返回 NoMemory 且已拷贝字节数为 0。
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

        // Step 3: 验证物理页耗尽期间，此前已写入首槽的 4096 字节数据仍可完整读出。
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

        // Step 4: 释放物理页压力，验证套接字与管道恢复正常物理页分配和数据收发。
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
