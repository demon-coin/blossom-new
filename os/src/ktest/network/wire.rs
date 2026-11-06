//! 同步内存包队列（In-Memory Wire）与双 `TcpCore` 测试夹具（`NetworkOwner`）模块。
//!
//! # 模块职责与架构定位
//! 本模块为同步协议核心测试套件（`super::core`）提供专用的 `NetworkOwner` 测试夹具。
//! 它在不启动后台网络栈（`NetStack`）或异步工作队列的前提下，直接在内存中构造一对客户端/服务端 `TcpCore` 实例
//! 和一个固定容量的内存数据包队列（`wire`），用于确定性地单步验证 TCP 状态机、滑动窗口、环形缓冲区回绕及背压回退。
//! 上游由 `ktest` 框架管理用例生命周期并供 `core.rs` 调用，下游依赖 `crate::net::core::TcpCore`、`OwnedPacket`
//! 以及内核内存/任务基线采样器 `execution_services::baseline`。
//!
//! # 核心设计与同步包泵（Packet Pump）数据流
//! `NetworkOwner::pump` 在单次调用中依次收集两端待发送的协议段放入 `wire` 队列，再逐个弹出并根据目的端口路由给对端处理，
//! 对端若立即产生应答段（如 `SYN+ACK`、`ACK` 或 `RST`）则继续入队直至队列排空：
//!
//! ```text
//!   client: SleepMutex<TcpCore>                 server: SleepMutex<TcpCore>
//!   (127.0.0.1:35701, 256B RX/TX)               (127.0.0.1:44702, 256B RX/TX)
//!   +---------------------------+               +---------------------------+
//!   | 1. dispatch() 吐出待发包   |               | 1. dispatch() 吐出待发包   |
//!   +-------------+-------------+               +-------------+-------------+
//!                 | enqueue()                                 | enqueue()
//!                 v                                           v
//!        +--------+-------------------------------------------+--------+
//!        |          wire: SpinLock<VecDeque<OwnedPacket>>              |
//!        |   (容量上限 LOOPBACK_CAPACITY；队满返回 FsError::WouldBlock) |
//!        +--------+-------------------------------------------+--------+
//!                 | 2. take_packet() 按目的端点分发            |
//!                 v                                           v
//!   +-------------+-------------+               +-------------+-------------+
//!   | 3. process() 处理入站包    |               | 3. process() 处理入站包    |
//!   |    若有即时应答包再入队    |               |    若有即时应答包再入队    |
//!   +---------------------------+               +---------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 锁层级顺序严格固定为：**`client` 或 `server`（`SleepMutex<TcpCore>`） ➔ `wire`（`SpinLock<VecDeque<OwnedPacket>>`）**。
//! - **禁止同时持有多把端点锁**：`pump` 与 `connect` 每次仅锁定单个 `TcpCore`，绝不同时持有 `client` 与 `server` 两把睡眠锁。
//! - **短临界区出队**：`take_packet` 获取 `wire` 自旋锁弹出单个 `OwnedPacket` 后立即释放锁，包头解析与 `TcpCore::process`
//!   均在 `wire` 自旋锁临界区之外执行，避免在自旋锁内尝试获取 `SleepMutex` 导致违反不可睡眠约束或死锁。

use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::collections::VecDeque;
use smoltcp::socket::tcp::State;

use super::support::{operation, CLIENT, SERVER};
use crate::fs::FsError;
use crate::ktest::execution_services::{baseline, Baseline, SuiteBaseline};
use crate::net::core::TcpCore;
use crate::net::packet::OwnedPacket;
use crate::net::{LOOPBACK_CAPACITY, LOOPBACK_MTU};
use crate::sync::{SleepMutex, SpinLock};
use crate::{time, timer};

/// 同步双端 `TcpCore` 与内存环回包队列测试夹具（Fixture）。
///
/// 独立持有客户端与服务端两个协议核心实例、共享包缓冲队列以及用例启动前的资源基线快照。
pub(super) struct NetworkOwner {
    /// 测试用例启动前采样的物理内存六维统计与任务元数据基线，用于在 `tear_down` 时校验零泄漏。
    before: Baseline,
    /// 模拟物理链路的内存 FIFO 数据包队列；容量上限为 `LOOPBACK_CAPACITY` 个包，由自旋锁保护。
    wire: SpinLock<VecDeque<OwnedPacket>>,
    /// 主动发起连接的客户端 TCP 协议核心（本地地址 `CLIENT`，收发缓冲区各 256 字节），由可睡眠互斥锁保护。
    pub(super) client: SleepMutex<TcpCore>,
    /// 被动监听的服务端 TCP 协议核心（监听地址 `SERVER`，收发缓冲区各 256 字节），由可睡眠互斥锁保护。
    pub(super) server: SleepMutex<TcpCore>,
}

impl Fixture<SuiteBaseline> for NetworkOwner {
    /// 在普通任务上下文中记录初始资源基线，分配内存包队列与双端 `TcpCore`，并启动监听与初始 `SYN` 准备。
    ///
    /// 若中途发生内存不足或协议错误，已分配的局部队列和 `TcpCore` 会在栈展开返回时自动析构释放，不遗留后台任务。
    fn set_up(context: &mut TestContext<'_>, _: &SuiteBaseline) -> Result<Self, TestError> {
        let before = baseline();
        let mut wire = VecDeque::new();
        wire.try_reserve_exact(LOOPBACK_CAPACITY)
            .map_err(|_| TestError("network core wire allocation failed"))?;
        let client = SleepMutex::new(operation(
            context,
            TcpCore::try_new(256, 256, LOOPBACK_MTU, 1),
        )?);
        let server = SleepMutex::new(operation(
            context,
            TcpCore::try_new(256, 256, LOOPBACK_MTU, 2),
        )?);
        operation(context, server.lock().listen(SERVER))?;
        operation(context, client.lock().connect(CLIENT, SERVER, time::now()))?;
        Ok(Self {
            before,
            wire: SpinLock::new(wire),
            client,
            server,
        })
    }

    /// 显式销毁夹具持有的双端 `TcpCore` 与内存包队列，并断言内核物理内存统计与任务元数据完全回归初始基线。
    ///
    /// 由于本夹具不启动任何后台异步线程或工作队列任务，`drop(self)` 完成后所有动态分配的内存应当立即归还。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.before;
        drop(self);
        let after = baseline();
        context.diagnostic(format_args!(
            "network-core resources-before={:?} resources-after={:?}",
            before, after
        ));
        context.expect_eq(after.0, before.0);
        context.expect_eq(after.1, before.1);
        if after != before {
            return Err(TestError("network core resources did not converge"));
        }
        Ok(())
    }
}

impl NetworkOwner {
    /// 将单个 `OwnedPacket` 移入内存链路队列尾部。
    ///
    /// 可在 `TcpCore::dispatch` 发送回调或普通任务上下文中调用（内部短暂获取 `wire` 自旋锁）。
    /// 当队列长度已达 `LOOPBACK_CAPACITY` 上限时，拒绝入队并返回 `Err(FsError::WouldBlock)`，以触发发送端协议层的背压回退。
    pub(super) fn enqueue(&self, packet: OwnedPacket) -> Result<(), FsError> {
        let mut wire = self.wire.lock();
        if LOOPBACK_CAPACITY <= wire.len() {
            return Err(FsError::WouldBlock);
        }
        wire.push_back(packet);
        Ok(())
    }

    /// 从内存链路队列头部弹出一个待投递的数据包。
    ///
    /// 仅在 `wire` 自旋锁临界区内执行 `pop_front` 指针移动，调用方在锁外安全执行后续 TCP 报文解析与状态机处理。
    pub(super) fn take_packet(&self) -> Option<OwnedPacket> {
        self.wire.lock().pop_front()
    }

    /// 执行一轮完整的同步双向发包采集与入站报文投递循环，直至内存链路队列排空。
    ///
    /// 在普通可睡眠任务上下文中调用：先让两端各自将待发报文推入 `wire`，再循环取出报文按目的地址交给目标端处理，
    /// 并将处理期间产生的即时回复报文继续入队投递。
    pub(super) fn pump(&self, context: &TestContext<'_>) -> TestResult {
        let now = time::now();
        // Step 1: 依次驱动客户端与服务端将当前时刻可发送的 TCP 报文段排入内存队列。
        for endpoint in [&self.client, &self.server] {
            operation(
                context,
                endpoint.lock().dispatch(now, |packet| self.enqueue(packet)),
            )?;
        }
        // Step 2: 循环从队列弹出报文，解析目的 TCP 端点并交由对应核心处理，同时将即时应答包继续入队直到链路静默。
        while let Some(packet) = self.take_packet() {
            let (_, destination) = operation(context, packet.tcp_endpoints())?;
            let endpoint = if destination == SERVER {
                &self.server
            } else if destination == CLIENT {
                &self.client
            } else {
                return Err(TestError("unexpected native TCP destination"));
            };
            let reply = operation(context, endpoint.lock().process(&packet, now))?;
            if let Some(reply) = reply {
                operation(context, self.enqueue(reply))?;
            }
        }
        Ok(())
    }

    /// 在 5 秒单调时钟截止期限内循环调用 `pump` 驱动两端协议栈，直至谓词 `condition` 返回 `true`。
    ///
    /// 必须在普通可睡眠任务上下文中调用。每轮驱动后若未超时则休眠 1 毫秒，使单调时钟向前推进以触发超时重传（RTO）等时间相关逻辑。
    pub(super) fn until(
        &self,
        context: &TestContext<'_>,
        condition: impl Fn() -> bool,
    ) -> TestResult {
        let end = time::now()
            .checked_add(time::Duration::from_secs(5))
            .ok_or(TestError("network core observation deadline overflow"))?;
        while !condition() {
            self.pump(context)?;
            if end <= time::now() {
                return Err(TestError("native TCP core did not progress"));
            }
            timer::sleep(time::Duration::from_millis(1));
        }
        Ok(())
    }

    /// 驱动内存链路完成三次握手，等待客户端与服务端双方的 TCP 状态机均进入 `State::Established`。
    ///
    /// 在普通可睡眠任务上下文中调用，确保后续数据传输测试在双向连接完全建立后开始。
    pub(super) fn connect(&self, context: &TestContext<'_>) -> TestResult {
        self.until(context, || {
            self.client.lock().state() == State::Established
                && self.server.lock().state() == State::Established
        })
    }
}
