/// UNIX 域有序分组套接字（`SOCK_SEQPACKET`）记录边界、双重背压与半关闭内核测试套件。
///
/// ## 1. 模块职责与架构定位
/// 本模块位于 `ktest::network` 测试子系统中，验证 `UnixSocketType::SeqPacket` 类型套接字的面向记录
/// （Record-Oriented）传输语义、截断标志（`MSG_TRUNC`）、窥视读取（`MSG_PEEK`）、零长度记录与半关闭
/// EOF 的区分，以及字节容量和记录槽位双维度的非阻塞背压控制。
///
/// ## 2. 核心队列结构与背压机制
/// ```text
///   SOCK_SEQPACKET 单向接收队列（双重配额约束）
///   ┌───────────────────────────────────────────────────────────────┐
///   │ 1. 字节预算上限：16384 Bytes                                  │
///   │    - 单条消息 > 16384B ──► 立即返回 Err(FsError::MessageTooLarge) │
///   │    - 队列累计字节达上限 ──► POLLOUT 清零，非阻塞写入返回 WouldBlock │
///   ├───────────────────────────────────────────────────────────────┤
///   │ 2. 记录元数据槽位上限：32 条记录（含 0 字节空记录）           │
///   │    - [Rec 0: len=8193] [Rec 1: len=0] ... [Rec 31: len=0]     │
///   │    - 即使剩余字节充裕，满 32 条记录同样触发 WouldBlock 背压   │
///   └───────────────────────────────────────────────────────────────┘
/// ```
/// - **记录边界保持**：每次 `send` 入队一条独立记录；`receive` 每次最多消费一条记录，若接收缓冲区小于记录长度，
///   多余字节被丢弃并在返回标志中置位 `MSG_TRUNC`（`0x20`），若传入 `MSG_TRUNC` 标志则 `original_len` 返回原记录总长。
/// - **零长度记录与 EOF 区分**：发送 `0` 字节切片会入队一条真实空记录（触发 `POLLIN`，`receive` 消费后队列变空）；
///   而 `read(&mut [])` 仅做零容量探测，不消费队列中的非空或空记录。对端执行 `shutdown(SHUT_WR)` 后，
///   必须在队列中已有的空记录与尾随数据全部被读取完毕后，后续读取才返回 `0`（EOF）并置位 `POLLRDHUP`。
///
/// ## 3. 锁与并发顺序说明
/// - 测试在单线程非阻塞（`O_NONBLOCK`）模式下交替操作 `sender` 与 `receiver`，直接检验端点内部状态机与队列锁后状态。
/// UNIX `SOCK_SEQPACKET` 套接字记录边界、截断标志、双重容量背压与双向半关闭测试套件。
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

    /// 测试目标：验证 `SOCK_SEQPACKET` 套接字严格保持每条记录边界不粘包，支持 `MSG_PEEK` 预览、短读截断并返回原始长度（`MSG_TRUNC`）、零字节 `read` 不误删记录、零长度记录独立入队与消费，以及写半关闭（`SHUT_WR`）后有序排空残留记录再报告 EOF。
    ///
    /// 场景步骤：
    /// 1. 创建非阻塞 `SeqPacket` 套接字对，验证 `socket_type()` 为 `5`（`SOCK_SEQPACKET`），连续发送 `"first"`（5B）与 `"second"`（6B）；
    /// 2. 使用 2 字节缓冲区带 `MSG_PEEK` 预览首条记录，确认返回 `copied=2, original_len=5, flags=MSG_TRUNC(0x20)` 且记录未出队；再用 2 字节缓冲区正式消费首条记录，验证同样返回截断信息并将剩余 3 字节丢弃；
    /// 3. 读取第二条记录，确认完整获得 `"second"`（6B），且再次读取返回 `WouldBlock`；
    /// 4. 发送 `"discard"`（7B），验证 `read(&mut [])` 返回 0 但不消费该记录，而 `receive(&mut [], 0)` 会消费该 7B 记录并报告 `original_len=7, flags=MSG_TRUNC`；
    /// 5. 发送 0 字节空记录，验证 `read(&mut [])` 不消费它，`MSG_PEEK` 可预览到 `(0, 0, 0)`，正式 `receive` 消费后 `POLLIN` 清零；
    /// 6. 依次发送空记录与 `"tail"` 后对 `sender` 执行 `shutdown(SHUT_WR)`，验证 `receiver` 先后读出空记录、`"tail"`，最后才读到 EOF（0）并观察到 `POLLRDHUP`；反向回复 `"reply"` 并关闭写方向后，验证双向再次发送均返回 `BrokenPipe`。
    #[test]
    fn unix_seqpacket_record_peek_truncation_empty_and_shutdown(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建非阻塞 SOCK_SEQPACKET 套接字对，连续发送两条记录验证不发生字节流合并。
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

        // Step 2: 验证 MSG_PEEK 窥视截断与短缓冲区正式接收截断（MSG_TRUNC = 0x20），确认单次接收消费整条记录。
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

        // Step 3: 对比 read(&mut []) 零长缓冲探测与 receive(&mut []) 零容量接收：前者保留队首记录，后者消费记录并报告截断。
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

        // Step 4: 发送并接收 0 字节空记录，验证空记录独立占用一条队列项且可通过 MSG_PEEK 预览。
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

        // Step 5: 验证写半关闭（SHUT_WR）在排空队列中的空记录与非空记录后返回 EOF（0）及 POLLRDHUP，且双向关闭后写入返回 BrokenPipe。
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

    /// 测试目标：验证 `SOCK_SEQPACKET` 队列同时受 16384 字节载荷上限与 32 条记录槽位上限的双重背压约束，确认消费记录可释放对应配额，且当最后一个发送端强引用释放后对端读取返回 EOF、两端全部关闭后归还全局端点槽位。
    ///
    /// 场景步骤：
    /// 1. 创建非阻塞 `SeqPacket` 套接字对，克隆 `sender` 为 `alias` 后丢弃 `sender`，确认端点依然存活；
    /// 2. 发送 16385 字节超长报文，验证立即返回 `FsError::MessageTooLarge`；随后发送恰好 16384 字节的最大报文并验证完整收发；
    /// 3. 分两次发送 8193 字节与 8191 字节填满 16384 字节配额，验证此时 `POLLOUT` 为 `false`，且连 0 字节空记录也因字节配额已满而返回 `WouldBlock`；
    /// 4. 对队首 8193 字节记录执行 1 字节 `MSG_PEEK`（不释放配额，`POLLOUT` 仍为 `false`），再执行 1 字节正式短读消费掉整条 8193 字节记录，验证 `POLLOUT` 立即恢复为 `true`；
    /// 5. 连续发送 31 条 0 字节空记录（连同队列中剩余的 1 条 8191B 记录共占满 32 个记录槽位），验证此时 `POLLOUT` 再次变为 `false` 且继续发送返回 `WouldBlock`；
    /// 6. 消费掉 8191B 记录释放 1 个记录槽位，验证可再发送 1 字节记录 `"z"`，随后依次读完 31 条空记录与 `"z"`；
    /// 7. 丢弃 `alias`（发送端最后一个强引用），验证活跃端点数减为 1 且接收端读到 EOF（0），最后丢弃 `receiver` 确认活跃端点数归零。
    #[test]
    fn unix_seqpacket_capacity_and_last_ofd_return_slots(
        context: &mut TestContext<'_>,
        fixture: &mut StackOwner,
    ) -> TestResult {
        // Step 1: 创建非阻塞 SOCK_SEQPACKET 套接字对，保留发送端别名 alias 并释放原句柄。
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

        // Step 2: 验证单条消息超过 16384 字节上限返回 MessageTooLarge，而恰好 16384 字节可正常完整收发。
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

        // Step 3: 用两条记录（8193B + 8191B）填满 16384 字节预算，验证 POLLOUT 清零且非阻塞发送返回 WouldBlock。
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

        // Step 4: 验证 MSG_PEEK 不会释放发送配额，而消费掉队首 8193B 记录后立即恢复 POLLOUT 可写状态。
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

        // Step 5: 连续追加 31 条零长度记录填满 32 个记录槽位上限，验证在字节未满时同样触发记录数背压。
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

        // Step 6: 消费 1 条记录腾出槽位后验证可再次写入，随后按序排空全部 31 条空记录与尾随记录。
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

        // Step 7: 释放发送端最后一个强引用 alias，验证接收端读到 EOF 且在两端全部释放后归还所有端点槽位。
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
