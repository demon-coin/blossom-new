//! 后台网络栈（`NetStack`）测试夹具（`StackOwner`）与资源生命周期管理模块。
//!
//! # 模块职责与架构定位
//! 本模块定义 `StackOwner` 测试夹具，供后台网络栈套件（`stack.rs`）及全部七个套接字行为套件
//! （`socket_*.rs`）共同复用。它负责在每个测试用例执行前创建独立的回环（Loopback）网络栈实例，
//! 并在测试结束时按严格顺序解除阻塞、等待子任务退出、关停后台协议服务并校验内核内存与任务基线。
//!
//! # 核心设计与三阶段拆卸生命周期（Three-Phase Teardown）
//! 由于网络测试常涉及内存压力注入（`HeapPressure`）、关中断门控任务以及后台异步 Work 队列，
//! 若直接销毁网络栈会导致后台任务或阻塞中的测试子任务发生悬垂引用或死锁。
//! 因此 `StackOwner` 实现了 `Fixture` 的三阶段生命周期：
//!
//! ```text
//!   set_up()                     quiesce()                         tear_down()
//! +-----------------------+    +------------------------------+    +-------------------------------+
//! | 1. 记录初始基线 before|    | 1. 释放 HeapPressure 归还内存|    | 1. Drop NetStackOwner         |
//! | 2. 创建回环 NetStack  | -> | 2. 清空 tcp_candidates 强引用| -> |    (等待后台 Work 彻底执行完) |
//! | 3. 初始化 release=false|   | 3. 置位 release=true 解锁门控|    | 2. Drop StackOwner 剩余引用   |
//! +-----------------------+    +------------------------------+    | 3. 采样 after 并断言与 before |
//!                                       |                          |    完全收敛（零泄漏）         |
//!                                       v                          +-------------------------------+
//!                              [ktest 框架在此间隙等待所有
//!                               派生的测试子任务彻底退出]
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `release` 采用 `AtomicBool`（`Release`/`Acquire` 内存序）在主测试线程与关中断/自旋子任务之间传递退出信号，不涉及互斥锁。
//! - `tear_down` 必须在 `quiesce` 唤醒所有子任务且 `ktest` 框架回收（Join）全部测试子任务栈之后调用，
//!   此时丢弃 `NetStackOwner` 可在普通可睡眠任务上下文中安全等待后台网络 Work 任务执行完毕。

use ::core::sync::atomic::{AtomicBool, Ordering};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use super::support::operation;
use crate::cache::ktest::HeapPressure;
use crate::ktest::execution_services::{baseline, Baseline, SuiteBaseline};
use crate::net::socket::SocketFile;
use crate::net::tcp::TcpEndpoint;
use crate::net::{NetStack, NetStackOwner, NetStackRef};

/// 后台网络栈与套接字测试套件共用的测试夹具（Fixture）。
///
/// 统一管理单次测试用例的回环网络栈实例、内存压力守卫、待清理套接字集合以及资源泄漏基线。
pub(super) struct StackOwner {
    /// 测试用例启动前采样的内核内存与任务资源基线，用于在 `tear_down` 时校验资源是否完全归还。
    before: Baseline,
    /// 后台网络栈的所有权守卫；在 `tear_down` 中显式 `take()` 并丢弃，以同步关停后台网络服务。
    pub(super) owner: Option<NetStackOwner>,
    /// 指向被测回环网络栈（`NetStack`）的强引用，供测试代码创建 TCP/UDP/Unix 端点或套接字。
    pub(super) net: NetStackRef,
    /// 跨任务退出门控标志；在 `quiesce` 阶段置为 `true`（`Ordering::Release`），通知自旋或门控子任务解除阻塞。
    pub(super) release: Arc<AtomicBool>,
    /// 测试过程中暂存的候选 `SocketFile` 强引用列表；在 `quiesce` 阶段统一清空以触发套接字正常关闭。
    pub(super) tcp_candidates: Vec<Arc<SocketFile>>,
    /// 指向底层 `TcpEndpoint` 的弱引用（`Weak`）观察列表，用于在不延长端点生命周期的前提下检查其是否已被销毁。
    pub(super) tcp_observers: Vec<Weak<TcpEndpoint>>,
    /// 可选的堆内存压力守卫；用于在 OOM 测试中占满剩余堆空间，并在 `quiesce` 阶段优先释放以恢复常规内存分配。
    pub(super) pressure: Option<HeapPressure>,
}

impl Fixture<SuiteBaseline> for StackOwner {
    /// 在普通任务上下文中记录资源基线并创建独立的回环网络栈实例。
    ///
    /// 若初始化过程中发生错误，已创建的局部 `NetStackOwner` 会在当前可睡眠任务上下文中正常执行 `Drop` 清理。
    fn set_up(context: &mut TestContext<'_>, _: &SuiteBaseline) -> Result<Self, TestError> {
        let before = baseline();
        let owner = operation(context, NetStack::try_new_lo())?;
        let net = Arc::clone(owner.stack());
        let release = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("network stack release allocation failed"))?;
        Ok(Self {
            before,
            owner: Some(owner),
            net,
            release,
            tcp_candidates: Vec::new(),
            tcp_observers: Vec::new(),
            pressure: None,
        })
    }

    /// 在测试主逻辑结束、框架回收子任务之前执行静默化（Quiesce）清理。
    ///
    /// 依次释放堆内存压力守卫、清空暂存的套接字强引用，并将 `release` 原子标志置为 `true`
    /// 以放行可能正在自旋等待的门控子任务，随后由 `ktest` 框架通知并等待所有派生的测试子任务退出。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        drop(self.pressure.take());
        self.tcp_candidates.clear();
        self.release.store(true, Ordering::Release);
        Ok(())
    }

    /// 在所有测试子任务彻底退出并释放各自内核栈之后，关停网络栈后台服务并校验资源基线。
    ///
    /// 先显式丢弃 `NetStackOwner` 等待后台网络 Work 任务全部执行完毕并退出，
    /// 再释放夹具自身的全部剩余引用，最后对比测试前后的物理页/堆分配及任务元数据基线，确认无任何资源泄漏。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.before;
        drop(self.owner.take());
        drop(self);
        let after = baseline();
        context.diagnostic(format_args!(
            "network-stack resources-before={:?} resources-after={:?}",
            before, after
        ));
        context.expect_eq(after.0, before.0);
        context.expect_eq(after.1, before.1);
        if after != before {
            return Err(TestError("network stack resources did not converge"));
        }
        Ok(())
    }
}
