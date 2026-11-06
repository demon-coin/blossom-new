//! `SocketFile` 测试辅助函数集：提供套接字监听创建、完整流读写、Epoll 事件收集及 UDP OOM 关中断门控任务准备。
//!
//! # 模块职责与架构定位
//! 本模块作为 `os/src/ktest/network` 下各套接字行为测试套件（`socket_*.rs`）的公共辅助层，
//! 封装了基于 VFS `FileLike` 接口的循环读写、Epoll 就绪扫描以及用于 UDP 内存压力测试的跨核同步门控任务。
//! 上游由各 `socket_*` 测试用例调用，下游调用 `SocketFile`、`EpollFile`、中断保护守卫 `IrqSaveGuard` 与任务调度器。
//!
//! # 核心设计与 UDP OOM 门控状态机
//! 在测试 UDP 套接字遭遇内存耗尽（OOM）及并发关中断竞争时，`prepare_udp_oom_gate` 构造了一个绑定在 CPU0 上的
//! 预备子任务（`PreparedChild`），其三阶段同步时序如下：
//!
//! ```text
//!   主测试任务 (Main Test Task)                    CPU0 门控子任务 (PreparedChild)
//! +-----------------------------+               +-----------------------------------+
//! | 1. prepare_udp_oom_gate()   |               | (仅分配任务栈，尚未加入就绪队列)  |
//! |    预先分配子任务内核栈     |               |                                   |
//! | 2. 构造 HeapPressure 占满堆 |               |                                   |
//! |    (此时 CPU0 仍开中断，    |               |                                   |
//! |     可响应 SLUB flush IPI)  |               |                                   |
//! | 3. 启动门控子任务 --------> | ------------> | 4. IrqSaveGuard::new() 关本地中断 |
//! |    等待 entered == true     | <------------ |    entered.store(true, Release)   |
//! | 5. 执行并发 UDP / 关停操作  |               |    自旋等待 release == true       |
//! | 6. release.store(true) ---> | ------------> | 7. drop(irq) 恢复本地中断         |
//! |    推进后台 Work 回收       |               |    循环 yield_now() 让出 CPU      |
//! | 8. finished.store(true) --> | ------------> |    (保持任务栈存活，避免归还页框) |
//! |                             |               | 9. 任务退出                       |
//! +-----------------------------+               +-----------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `socket_read_exact`、`socket_write_all`、`socket_epoll_collect` 均在普通可睡眠任务上下文中调用，
//!   内部由 `SocketFile` 或 `EpollFile` 自行管理锁，调用方在调用前不应持有自旋锁或关中断守卫。
//! - `prepare_udp_oom_gate` 内部在持有 `IrqSaveGuard` 关中断期间仅执行原子变量读取（`Acquire`）与时钟读取，
//!   严禁在 `drop(irq)` 之前调用 `yield_now()` 或任何可能睡眠的函数。

use ::core::sync::atomic::{AtomicBool, Ordering};
use ::ktest::{TestContext, TestError, TestResult};
use alloc::sync::Arc;

use super::support::{child_operation, operation, SERVER};
use crate::fs::epoll::{EpollEvent, EpollFile};
use crate::fs::{FileLike, FsError};
use crate::net::address::{AddressFamily, InetEndpoint, SocketAddress};
use crate::net::socket::SocketFile;
use crate::net::NetStackRef;
use crate::time;

/// 在指定的后台网络栈 `net` 上创建、绑定并启动监听一个 IPv4 TCP `SocketFile`。
///
/// 监听地址固定使用环回地址 `SERVER.address`（`127.0.0.1`）与传入的 `port` 端口，
/// 监听队列积压上限（backlog）设为 4。在普通可睡眠任务上下文中调用。
pub(super) fn socket_listener(
    context: &TestContext<'_>,
    net: &NetStackRef,
    port: u16,
) -> Result<Arc<SocketFile>, TestError> {
    let file = operation(
        context,
        SocketFile::try_new_tcp(net.clone(), AddressFamily::Ipv4, 0, 0, 0),
    )?;
    operation(
        context,
        file.bind(SocketAddress::Inet(InetEndpoint { port, ..SERVER })),
    )?;
    operation(context, file.listen(4))?;
    Ok(file)
}

/// 通过 `FileLike::read` 循环从 `file` 读取数据，直到填满整个 `output` 缓冲区。
///
/// 自动处理流式套接字的合法短读（Partial Read）；若在填满 `output` 之前读到 `0`（即对端提前关闭连接产生 `EOF`），
/// 则立即返回 `TestError` 报错。
pub(super) fn socket_read_exact(file: &SocketFile, output: &mut [u8]) -> TestResult {
    let mut done = 0;
    while done < output.len() {
        let count = child_operation(file.read(&mut output[done..]))?;
        if count == 0 {
            return Err(TestError("native socket ended before full byte delivery"));
        }
        done += count;
    }
    Ok(())
}

/// 通过 `FileLike::write` 循环向 `file` 写入数据，直到 `input` 切片中的全部字节均被接收。
///
/// 自动根据每次 `write` 实际提交的字节数推进偏移；若单次写入返回 `0`（无任何前进进度），
/// 则立即返回 `TestError` 报错。
pub(super) fn socket_write_all(file: &SocketFile, input: &[u8]) -> TestResult {
    let mut done = 0;
    while done < input.len() {
        let count = child_operation(file.write(&input[done..]))?;
        if count == 0 {
            return Err(TestError("native socket send made no progress"));
        }
        done += count;
    }
    Ok(())
}

/// 调用 `EpollFile::scan` 扫描当前就绪事件，并将最多 `output.len()` 个事件依次写入 `output` 切片。
///
/// 返回本次扫描实际收集到的就绪事件数量（或底层 `FsError`），供测试代码核对水平触发（LT）或边沿触发（ET）事件字段。
pub(super) fn socket_epoll_collect(
    file: &EpollFile,
    output: &mut [EpollEvent],
) -> Result<usize, FsError> {
    let mut count = 0;
    file.scan(output.len(), &mut |event| {
        output[count] = event;
        count += 1;
        Ok(())
    })
}

/// 预先在 CPU0 上分配并准备一个关中断门控子任务（`PreparedChild`），但暂不将其投入调度运行。
///
/// # 设计原因与阶段说明
/// 1. **预先分配但不立即运行**：在施加 `HeapPressure` 占满堆内存时，内存回收器需要向所有 CPU 同步发送
///    SLUB 刷回核间中断（IPI）。若子任务提前在 CPU0 上关中断自旋，会导致 CPU0 无法响应 IPI 而死锁；
///    若在占满堆之后再创建子任务，又会因无可用内存而无法分配子任务内核栈。因此必须先预分配任务结构（`prepare_result_on`），
///    待堆压力就位后再启动该任务。
/// 2. **关中断自旋阶段**：子任务启动后立即通过 `IrqSaveGuard` 关闭 CPU0 本地中断，置位 `entered` 通知主任务，
///    并在 5 秒超时上限内自旋等待 `release` 信号。
/// 3. **开中断让出但保留任务栈阶段**：收到 `release` 后立即释放 `IrqSaveGuard` 恢复中断，随后循环调用 `yield_now()`
///    主动让出 CPU 以允许同核后台 Work 任务运行，直到 `finished` 置位才真正退出函数。这样可以避免子任务提前退出并释放其内核栈，
///    防止归还的栈内存页破坏测试构造的 OOM 边界条件。
pub(super) fn prepare_udp_oom_gate(
    entered: Arc<AtomicBool>,
    release: Arc<AtomicBool>,
    finished: Arc<AtomicBool>,
) -> Result<crate::ktest::PreparedChild, TestError> {
    crate::ktest::prepare_result_on(0, move |token| {
        // Step 1: 关闭当前 CPU 本地中断并置位 entered，在关中断状态下自旋等待主任务发出 release 信号。
        let irq = crate::sync::IrqSaveGuard::new();
        entered.store(true, Ordering::Release);
        let end = time::now()
            .checked_add(time::Duration::from_secs(5))
            .ok_or(TestError("UDP OOM IRQ gate deadline overflow"))?;
        let mut error = None;
        while !release.load(Ordering::Acquire) && !token.stop_requested() {
            if end <= time::now() {
                error = Some(TestError("UDP OOM IRQ gate release deadline"));
                break;
            }
            ::core::hint::spin_loop();
        }
        drop(irq);

        // Step 2: 恢复中断后循环调用 yield_now() 显式让出 CPU，避免纯自旋饿死同核的后台网络 Work 任务，
        // 同时保持当前子任务栈不被回收，直到主测试任务置位 finished。
        while !finished.load(Ordering::Acquire) && !token.stop_requested() {
            crate::sched::yield_now();
        }

        error.map_or(Ok(0), Err)
    })
}
