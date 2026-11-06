//! 后台网络栈（`NetStack`）协议层端点构建与异步握手等待辅助模块。
//!
//! # 模块职责与架构定位
//! 本模块为 `os/src/ktest/network/stack.rs` 等协议栈层测试提供底层端点辅助工具，
//! 包括基于可失败内存分配（Fallible Allocation）的 `smoltcp` UDP 报文缓冲区构造、
//! `TcpEndpoint` 监听器初始化，以及在后台回环服务驱动下等待 TCP 三次握手并提取已建立连接子端点的辅助函数。
//! 上游由 `stack` 测试套件调用，下游调用 `NetStack::try_new_tcp`、`TcpEndpoint` 及 `super::support::observe`。
//!
//! # 核心设计与后台握手观察流程
//! 与 `core.rs` 中由测试线程手动 `pump()` 同步转发数据包不同，本模块配合后台 `NetStack` Work 线程工作；
//! 测试线程仅发起 `connect` / `listen` 并通过 `accepted()` 轮询观察后台握手完成：
//!
//! ```text
//!   Client TcpEndpoint              NetStack 后台回环服务             Listener TcpEndpoint
//! +--------------------+          +----------------------+          +----------------------+
//! | connect() 发起连接 | -------> | 转发 SYN / SYN+ACK / | -------> | 内部创建候选子端点   |
//! |                    | <------- | ACK 完成三次握手     | <------- | 握手完成后入就绪队列 |
//! +---------+----------+          +----------------------+          +-----------+----------+
//!           |                                                                   |
//!           v (Step 1: observe 状态变为 Established)                            v (Step 2: observe accept() 返回 Some(child))
//! +---------+-------------------------------------------------------------------+----------+
//! | accepted() 校验 listener 仍处于 Listen 状态，且返回的 child 处于 Established 状态      |
//! +----------------------------------------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块所有函数均在普通可睡眠任务上下文中调用（`accepted` 内部通过 `observe` 在轮询间隙调用 `timer::sleep`）。
//! - 调用方在调用本模块函数时不得持有自旋锁或关中断守卫。

use ::ktest::{TestContext, TestError};
use alloc::sync::Arc;
use alloc::vec::Vec;
use smoltcp::socket::tcp::State;
use smoltcp::socket::udp as raw_udp;

use super::support::{observe, operation, SERVER};
use crate::net::address::{AddressFamily, InetEndpoint};
use crate::net::tcp::TcpEndpoint;
use crate::net::NetStackRef;

/// 使用可失败堆分配（`try_reserve_exact`）构造指定包条目数与字节容量的 `smoltcp` UDP `PacketBuffer`。
///
/// - `records`：元数据环形队列可容纳的最大 UDP 数据报个数（构造 `PacketMetadata` 数组）。
/// - `bytes`：负载环形缓冲区可容纳的最大总字节数（构造 `u8` 字节数组）。
///
/// 若堆内存不足，返回 `Err(TestError)` 而不是触发内核 Panic。
pub(super) fn udp_buffer(
    records: usize,
    bytes: usize,
) -> Result<raw_udp::PacketBuffer<'static>, TestError> {
    let mut metadata = Vec::new();
    metadata
        .try_reserve_exact(records)
        .map_err(|_| TestError("UDP metadata allocation failed"))?;
    metadata.resize(records, raw_udp::PacketMetadata::EMPTY);
    let mut payload = Vec::new();
    payload
        .try_reserve_exact(bytes)
        .map_err(|_| TestError("UDP payload allocation failed"))?;
    payload.resize(bytes, 0);
    Ok(raw_udp::PacketBuffer::new(metadata, payload))
}

/// 在后台网络栈 `net` 上创建、绑定并启动监听一个 IPv4 `TcpEndpoint`。
///
/// 收发缓冲区容量均设为 256 字节，绑定地址为 `127.0.0.1:port`（基于 `SERVER` 常量），
/// 监听队列积压上限（backlog）设为 4。
pub(super) fn listener(
    context: &TestContext<'_>,
    net: &NetStackRef,
    port: u16,
) -> Result<Arc<TcpEndpoint>, TestError> {
    let endpoint = operation(context, net.try_new_tcp(AddressFamily::Ipv4, 256, 256))?;
    operation(context, endpoint.bind(InetEndpoint { port, ..SERVER }))?;
    operation(context, endpoint.listen(4))?;
    Ok(endpoint)
}

/// 等待后台网络栈完成 `client` 与 `listener` 之间的 TCP 三次握手，并从 `listener` 取出已建立连接的子端点。
///
/// 先轮询等待 `client` 状态迁移至 `State::Established`，再轮询调用 `listener.accept()` 直至取得
/// 握手完毕的子端点（或捕获底层错误），最后校验 `listener` 仍保持 `State::Listen` 且子端点处于 `State::Established`。
pub(super) fn accepted(
    context: &mut TestContext<'_>,
    listener: &Arc<TcpEndpoint>,
    client: &Arc<TcpEndpoint>,
) -> Result<Arc<TcpEndpoint>, TestError> {
    // Step 1: 轮询等待客户端端点在后台回环服务驱动下进入 Established 状态。
    observe(
        || client.state() == State::Established,
        "background loopback handshake did not finish",
    )?;
    // Step 2: 轮询调用 listener.accept() 提取已完成三次握手的子连接端点。
    let mut child = None;
    let mut failure = None;
    observe(
        || {
            match listener.accept() {
                Ok(value) => child = value,
                Err(error) => failure = Some(error),
            }
            child.is_some() || failure.is_some()
        },
        "background listener child did not become ready",
    )?;
    if let Some(error) = failure {
        return operation(context, Err(error));
    }
    // Step 3: 验证监听端点自身仍保持 Listen 状态，且新接受的子端点已处于 Established 状态。
    context.expect_eq(listener.state(), State::Listen);
    let child = child.ok_or(TestError("listener ready child missing"))?;
    context.expect_eq(child.state(), State::Established);
    Ok(child)
}
