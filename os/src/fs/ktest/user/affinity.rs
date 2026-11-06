//! 用户态进程 CPU 亲和性（CPU Affinity）与 `getcpu` 系统调用集成测试模块。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统与用户态集成测试层（`fs::ktest::user`），负责加载并运行
//! `/test/selftests/process/affinity` 用户态自测程序，端到端验证 `sched_getaffinity`、
//! `sched_setaffinity` 与 `getcpu` 系统调用在多核（SMP）调度器下的行为。
//! 上游由 `fs::ktest::user::anchor` 注册到 `ktest` 框架，下游依赖 `UserOwner` 夹具
//! 准备 ext4 根文件系统、挂载 `/dev` 控制台并回收用户进程树。
//!
//! # 核心验证流程
//! ```text
//! [ktest 线程]                              [用户态 selftest 进程树]
//!      |                                                |
//!      +-- prepare_tty(4) 挂载 ext4 与 /dev             |
//!      +-- run_selftest_named(credentials) -----------> | 1. 校验短/长亲和性掩码边界
//!      |                                                | 2. 逐核绑定并验证跨核迁移与 getcpu
//!      |                                                | 3. 验证 fork/clone 子任务掩码继承
//!      |                                                | 4. 验证对远端正在运行任务的跨核迁移交接
//!      +<-- wait() 收集退出状态与 TAP 输出 -------------+
//! ```
//!
//! # 锁与并发契约
//! - 测试夹具在独立内核线程上下文运行，通过 `TaskGraph::wait` 阻塞等待用户进程退出；
//! - 跨核迁移由调度器运行队列锁与 IPI 协同完成，测试线程本身不直接持有调度器内部自旋锁。

use super::*;

/// 用户态 CPU 亲和性行为测试套件。
#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user-elf", "user-affinity", "cpu-smp"), resources("root"), timeout_ms = 90000)]
pub(crate) mod behavior {
    use super::*;

    /// 验证 `root` 与同 UID 普通凭据下的 CPU 亲和性掩码校验、跨核迁移、子任务继承与远端运行态任务迁移。
    ///
    /// - **测试目标**：确认 `sched_setaffinity` / `sched_getaffinity` / `getcpu` 在特权与非特权同 UID 场景下均符合 Linux ABI 契约。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：通过 `prepare_tty` 挂载只读根文件系统与 `/dev` 控制台视图；
    ///   2. When：以指定凭据（`ROOT` 或 `uid=1000, gid=1000`）启动 `/test/selftests/process/affinity` 自测程序；
    ///   3. Then：自测程序依次完成掩码长度边界检查、逐核迁移与 `getcpu` 核对、`fork`/`clone` 掩码继承检查以及对其他正在运行线程的跨核设置，全部子测试通过并以状态码 0 退出。
    #[test]
    #[case("root", Credentials::ROOT)]
    #[case("same_uid", Credentials { uid: 1000, gid: 1000 })]
    fn masks_migration_and_getcpu(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        credentials: Credentials,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(
            context,
            b"/test/selftests/process/affinity",
            &[],
            credentials,
        )
    }
}
