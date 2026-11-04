//! 身份增长、共同限额和 RT 实例复用原用户树的停止、wait 和真实离栈回收。

use super::*;
use crate::task::process::{ProcessError, MAX_PID};
use crate::task::signal::{Sighand, SignalInheritance};
use alloc::vec::Vec;

const PROGRAM: &[u8] = b"/test/selftests/process/resource_limits";

#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-resources"), resources("root"), timeout_ms = 180000)]
pub(crate) mod resource_limits {
    use super::*;

    /// 600 个未发表身份跨页增长，放弃候选后立即复用号码且旧对象不命中新身份。
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

    /// 真实进程同时存在超过 512 个，收集 TAP 注释后完整消费子树与内核执行尾部。
    #[test]
    fn processes_cross_512(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--growth"], Credentials::ROOT)
    }

    /// 先持有 520 个真实线程，释放 128 个时继续新建，最终持有 652 个后完整 join。
    #[test]
    #[ktest(min_cpus = 2, timeout_ms = 180000)]
    fn threads_cross_512_and_retirement(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--threads"], Credentials::ROOT)
    }

    /// 保留两类凭据的真实 NPROC 与排队计费检查，TAP 输出由收集任务转成注释。
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
