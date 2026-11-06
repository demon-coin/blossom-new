//! 网络与套接字测试套件的公共端点常量、错误转换、测试字节流生成与异步状态轮询辅助模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 `os/src/ktest/network` 辅助支撑层，为同步协议核心套件（`core`）、后台网络栈套件（`stack`）
//! 以及全部套接字集成测试套件（`socket_*`）提供无状态的共享常量与工具函数。
//! 上游由 `ktest::network` 下各子模块与内联测试套件直接调用，下游依赖 `ktest` 报告诊断接口、
//! `crate::net::NetStack` 活跃端点计数接口以及内核定时器（`crate::time` / `crate::timer`）。
//!
//! # 核心机制与被动轮询模型
//! 后台网络栈（`NetStack`）的数据包收发、超时重传及 `FIN` / `TIME_WAIT` 回收均由内核异步工作队列（WorkQueue）
//! 和定时器回调驱动。本模块的等待函数采用“纯被动观察 + 定时让出 CPU”模型，自身绝不代为驱动协议栈逻辑：
//!
//! ```text
//!   测试线程 (Test Task)                              后台网络服务 (WorkQueue / Timer)
//! +-----------------------------------+             +-----------------------------------+
//! | observe() / socket_retired()      |             |                                   |
//! |   +--> 计算单调时钟截止时间 end   |             |                                   |
//! |   +--> while !ready() {           |             |                                   |
//! |   |      检查是否超时 (end <= now)|             |                                   |
//! |   |      timer::sleep(1ms) -------+--(让出CPU)->| 执行协议 Work、重传定时器、        |
//! |   |    }  <-----------------------+--(唤醒返回)-| 完成 FIN/TIME_WAIT 并归还端点槽位 |
//! |   +--> 条件满足，返回 Ok(())      |             |                                   |
//! +-----------------------------------+             +-----------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块自身不定义也不持有任何互斥锁或自旋锁。
//! - `observe` 与 `socket_retired` 内部调用 `timer::sleep` 挂起当前任务，**必须**在普通可睡眠任务上下文中调用，
//!   调用方在进入等待前不得持有自旋锁（`SpinLock`）或关中断守卫（`IrqSaveGuard`）；传入 `observe` 的 `ready` 闭包
//!   内部若需加锁，必须在闭包返回前释放锁，防止跨睡眠持锁导致死锁。

use ::ktest::{TestContext, TestError, TestResult};
use smoltcp::wire::{IpAddress, Ipv4Address};

use crate::fs::FsError;
use crate::mm::user_table::PagePermissions;
use crate::net::address::InetEndpoint;
use crate::net::NetStackRef;
use crate::{time, timer};

/// 测试客户端默认使用的 IPv4 环回端点（`127.0.0.1:35701`）。
pub(super) const CLIENT: InetEndpoint = InetEndpoint {
    address: IpAddress::Ipv4(Ipv4Address::new(127, 0, 0, 1)),
    port: 35701,
};
/// 测试服务端默认使用的 IPv4 环回监听端点（`127.0.0.1:44702`）。
pub(super) const SERVER: InetEndpoint = InetEndpoint {
    address: IpAddress::Ipv4(Ipv4Address::new(127, 0, 0, 1)),
    port: 44702,
};

/// 用户态缓冲区拷贝与缺页边界测试使用的默认映射虚拟基地址（`0x4600_0000`，按页对齐）。
pub(super) const ADDRESS: usize = 0x4600_0000;
/// 测试用户态虚拟页默认使用的可读写、不可执行页表权限（`R + W`）。
pub(super) const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 将内核文件系统/网络操作返回的 `Result<T, FsError>` 转换为 `ktest` 测试结果。
///
/// 若操作返回 `Err(error)`，先通过当前测试上下文 `context.diagnostic` 输出包含具体 `FsError` 变体的诊断日志，
/// 再返回统一的 `TestError("network operation failed")` 中止当前用例，便于测试失败时排查根因。
/// 可在持有 `TestContext` 的测试主线程上下文中调用。
pub(super) fn operation<T>(
    context: &TestContext<'_>,
    result: Result<T, FsError>,
) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("network operation-error={:?}", error));
        TestError("network operation failed")
    })
}

/// 根据字节在数据流中的逻辑偏移量 `position` 生成确定性的非周期测试填充字节。
///
/// 结合质数乘法（`37`）与跨页/跨环回绕步进项（`position / 251`），使相邻窗口或 256 字节环形缓冲区回绕位置上的字节序列互不相同，
/// 从而能够可靠检出流式传输中的偏移错位、重复拷贝或环形缓冲区覆盖错误。纯计算函数，可在任意上下文中调用。
pub(super) fn pattern(position: usize) -> u8 {
    (position as u8)
        .wrapping_mul(37)
        .wrapping_add((position / 251) as u8)
}

/// 在 5 秒单调时钟超时上限内，以 1 毫秒为间隔休眠轮询等待条件 `ready` 成立。
///
/// 必须在普通可睡眠任务上下文中调用。每轮检查条件不满足时调用 `timer::sleep(1ms)` 主动让出 CPU，
/// 交由后台工作队列与定时器推进网络协议状态；若超过 5 秒仍未满足，则返回 `Err(TestError(error))`。
pub(super) fn observe(mut ready: impl FnMut() -> bool, error: &'static str) -> TestResult {
    let end = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("network stack observation deadline overflow"))?;
    while !ready() {
        if end <= time::now() {
            return Err(TestError(error));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 在派生的测试子任务（无 `&TestContext` 借用）中将 `Result<T, FsError>` 转换为 `ktest` 测试结果。
///
/// 失败时通过全局 `crate::ktest::report::diagnostic` 记录具体的 `FsError` 错误码，
/// 并返回 `TestError("network stack child operation failed")` 供父任务 `join()` 捕获。可在任意任务上下文中调用。
pub(super) fn child_operation<T>(result: Result<T, FsError>) -> Result<T, TestError> {
    result.map_err(|error| {
        crate::ktest::report::diagnostic(format_args!("network-stack child-error={:?}", error));
        TestError("network stack child operation failed")
    })
}

/// 等待后台网络栈 `net` 中所有活跃端点槽位（`EndpointSlot`）全部完成协议回收并归零。
///
/// 必须在普通可睡眠任务上下文中调用。由于 TCP 连接关闭后可能经历异步 `FIN` 挥手与 `TIME_WAIT` 定时器等待，
/// 本函数设置最长 30 秒的单调时钟截止期限，每 1 毫秒轮询一次 `net.ktest_live_endpoints()`，
/// 验证在不强制关停整个 `NetStack` 的前提下，所有套接字端点均能依靠正常协议生命周期自动回收完毕。
pub(super) fn socket_retired(net: &NetStackRef) -> TestResult {
    let end = time::now()
        .checked_add(time::Duration::from_secs(30))
        .ok_or(TestError("socket retirement deadline overflow"))?;
    while net.ktest_live_endpoints() != 0 {
        if end <= time::now() {
            return Err(TestError(
                "normal socket protocol retirement did not finish",
            ));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}
