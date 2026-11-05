//! 同步核心与后台消费者共享的输入、错误和只读等待。

use ::ktest::{TestContext, TestError, TestResult};
use smoltcp::wire::{IpAddress, Ipv4Address};

use crate::fs::FsError;
use crate::mm::user_table::PagePermissions;
use crate::net::address::InetEndpoint;
use crate::net::NetStackRef;
use crate::{time, timer};

pub(super) const CLIENT: InetEndpoint = InetEndpoint {
    address: IpAddress::Ipv4(Ipv4Address::new(127, 0, 0, 1)),
    port: 35701,
};
pub(super) const SERVER: InetEndpoint = InetEndpoint {
    address: IpAddress::Ipv4(Ipv4Address::new(127, 0, 0, 1)),
    port: 44702,
};

pub(super) const ADDRESS: usize = 0x4600_0000;
pub(super) const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 保留具体失败原因供 guest 原诊断消费，不将错误改成零进度或通过。
pub(super) fn operation<T>(
    context: &TestContext<'_>,
    result: Result<T, FsError>,
) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("network operation-error={:?}", error));
        TestError("network operation failed")
    })
}

/// 输入按外部逻辑位置变化，环回绕或复制错误不能满足全文比较。
pub(super) fn pattern(position: usize) -> u8 {
    (position as u8)
        .wrapping_mul(37)
        .wrapping_add((position / 251) as u8)
}

/// 只观察后台已产生的事实，等待本身不处理包、不安排协议 Work。
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

/// 子任务保留真实错误诊断，不能把绑定或队列错误转换成成功退出。
pub(super) fn child_operation<T>(result: Result<T, FsError>) -> Result<T, TestError> {
    result.map_err(|error| {
        crate::ktest::report::diagnostic(format_args!("network-stack child-error={:?}", error));
        TestError("network stack child operation failed")
    })
}

/// 只等待实际 EndpointSlot 归还，根服务继续存活并推进正常 FIN/TIME_WAIT。
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
