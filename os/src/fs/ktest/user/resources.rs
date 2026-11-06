//! 进程身份表跨页扩容、并发进程/线程规模扩展与按 UID 计费的资源限额（`RLIMIT_NPROC` / 实时信号排队）测试套件。
//!
//! # 模块职责与架构定位
//! 本模块定义 `resource_limits` 内核测试套件（`#[ktest::suite]`），负责验证进程身份图（`TaskGraph`）
//! 在突破单页 512 槽位上限时的动态增长、未发布候选身份（`PreparedProcess`）丢弃后的 PID 号码复用，
//! 以及通过运行用户态 `/test/selftests/process/resource_limits` 自测程序验证大规模并发进程/线程创建与退出回收、
//! 按用户（UID）统计的 `RLIMIT_NPROC` 进程数限额与实时信号排队配额。
//!
//! # 核心设计与关键布局
//! ## `TaskGraph` PID 槽位跨 512 边界增长与复用
//! ```text
//! [阶段 1: 连续预留 600 个 PreparedProcess]
//!   PID 1 ..= 512 (第一批槽位)  --->  PID 513 ..= 600 (跨 512 边界动态扩容)
//!
//! [阶段 2: 释放槽位 511 (对应 PID = 512)]
//!   drop(identities[511])  --->  槽位 512 归还空闲池，旧 Pid 对象 (old) 失效
//!
//! [阶段 3: 再次申请 1 个 PreparedProcess]
//!   立即复用最小空闲号 PID = 512，生成全新 Pid 对象 (replacement != old)
//!   graph.lookup_pid(&old) == None（防止 ABA 悬垂引用误命中新进程）
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块不直接获取自旋锁；所有进程表操作均通过 `TaskGraph` 内部锁保护的公开接口进行。
//! - 用户态自测用例通过 `UserOwner::run_selftest_named` 执行，确保所有子进程、线程完成 `wait` 并彻底退出内核栈后，
//!   再汇合管道日志收集线程并归还测试夹具资源。

use super::*;
use crate::task::process::{ProcessError, MAX_PID};
use crate::task::signal::{Sighand, SignalInheritance};
use alloc::vec::Vec;

/// 用户态进程资源限额与规模扩展自测程序路径。
const PROGRAM: &[u8] = b"/test/selftests/process/resource_limits";

#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-resources"), resources("root"), timeout_ms = 180000)]
pub(crate) mod resource_limits {
    use super::*;

    /// 测试目标：验证 `TaskGraph` 在连续预留 600 个未发布进程身份时能够跨越 512 槽位边界正常增长，并在丢弃中途预留项后立即复用其 PID 号码且不会发生旧 `Pid` 对象句柄碰撞。
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 新建独立的 `TaskGraph` 与默认信号处理表，连续调用 `prepare_process` 预留 600 个进程身份（PID 1..=600）；
    /// 2. When: 保存第 512 号身份（下标 511）的旧 `Pid` 引用并释放该 `PreparedProcess`，随后再次调用 `prepare_process`；
    /// 3. Then: 新分配的身份立即复用 PID 512，但其 `Pid` 对象指针与旧对象不同，`lookup_pid(&old)` 返回 `None`，且未发布身份不可被 `try_wait` 观察到。
    #[test]
    fn reservations_cross_512(context: &mut TestContext<'_>, _: &mut UserOwner) -> TestResult {
        let graph = TaskGraph::try_new(MAX_PID, RootRole::Kernel)
            .map_err(|_| TestError("resource number graph preparation failed"))?;
        let actions =
            Sighand::try_new().map_err(|_| TestError("resource actions preparation failed"))?;
        let mut identities = Vec::new();
        identities
            .try_reserve_exact(600)
            .map_err(|_| TestError("resource candidate list allocation failed"))?;

        // Step 1: 连续预留 600 个未发布的进程身份，验证分配的 TID 从 1 严格递增至 600。
        for index in 0..600 {
            let prepared = graph
                .prepare_process(
                    None,
                    actions.clone(),
                    0,
                    SignalInheritance::EMPTY,
                    crate::task::signal::SIGCHLD,
                )
                .map_err(|_| TestError("resource identity growth failed"))?;
            context.expect_eq(prepared.identity().tid(), index + 1);
            identities.push(Some(prepared));
        }

        // Step 2: 释放第 512 号预留槽位并立即再次申请，验证号码 512 被优先复用且新旧 Pid 对象指针不相等。
        let old = identities[511]
            .as_ref()
            .expect("missing reserved PID")
            .identity()
            .tid_object();
        drop(identities[511].take());
        let replacement = graph
            .prepare_process(
                None,
                actions,
                0,
                SignalInheritance::EMPTY,
                crate::task::signal::SIGCHLD,
            )
            .map_err(|_| TestError("resource identity reuse failed"))?;
        context.expect_eq(replacement.identity().tid(), 512);
        context.expect_eq(
            Arc::ptr_eq(&old, &replacement.identity().tid_object()),
            false,
        );
        context.expect_eq(graph.lookup_pid(&old).is_none(), true);
        context.expect_eq(graph.try_wait(None, 512, false), Err(ProcessError::NoChild));

        drop((replacement, identities, old, graph));
        Ok(())
    }

    /// 测试目标：验证系统同时存在超过 512 个活动用户进程时的跨页身份管理、父子等待与退出回收。
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 准备只读根文件系统与 `/dev/console` 环境；
    /// 2. When: 以 `ROOT` 凭据运行 `resource_limits --growth` 自测程序，并发创建超过 512 个子进程；
    /// 3. Then: 自测程序输出 TAP 通过记录并以状态码 0 退出，所有子进程及其内核栈被完整回收。
    #[test]
    fn processes_cross_512(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--growth"], Credentials::ROOT)
    }

    /// 测试目标：验证多核环境下单进程内同时持有超过 512 个用户线程，并在部分线程退出回收的同时继续创建新线程的并发安全性。
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 至少 2 个 CPU 核心的运行环境及已挂载的 `/dev/console`；
    /// 2. When: 运行 `resource_limits --threads`，先创建 520 个并发线程，在释放其中 128 个线程的同时继续新建线程至总计持有 652 个线程，最后全部 `join`；
    /// 3. Then: 所有线程正常汇合退出，无 TID 泄漏或内核栈残留，主进程以状态码 0 退出。
    #[test]
    #[ktest(min_cpus = 2, timeout_ms = 180000)]
    fn threads_cross_512_and_retirement(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--threads"], Credentials::ROOT)
    }

    /// 测试目标：验证特权用户（`ROOT`）与普通用户（`UID=1000, GID=1000`）下的 `RLIMIT_NPROC` 进程数配额限制及实时（RT）信号排队计费边界。
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 准备 `/dev/console` 环境，分别传入 `ROOT` 与非特权 `UID 1000` 凭据参数；
    /// 2. When: 运行 `resource_limits --limits` 执行用户级进程配额与实时信号排队自测；
    /// 3. Then: 自测程序验证配额耗尽时的 `EAGAIN` 错误与权限豁免规则符合预期，并以状态码 0 退出。
    #[test]
    #[case("root", Credentials::ROOT)]
    #[case("same_uid", Credentials { uid: 1000, gid: 1000 })]
    fn user_limits_and_realtime(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        credentials: Credentials,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--limits"], credentials)
    }
}
