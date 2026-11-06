//! 用户态执行测试套件的任务生命周期管理夹具（`Users`）与多任务同步控制器（`Controls`）。
//!
//! # 模块职责与架构定位
//! 本模块服务于 `os/src/ktest/user_execution` 下的 `entry` 与 `vm` 两个测试套件：
//! - **上游调用者**：各用户态执行与虚拟内存测试用例（通过 `Fixture` 生命周期及 `Users` / `Controls` 辅助方法调用）。
//! - **下游依赖**：`crate::ktest` 子任务作用域原语（`prepare_result`、`ChildHandle`、`ChildToken`）、
//!   `crate::task::UserThread`、`crate::sched` 调度状态查询接口，以及 `execution_core` 的内存/元数据基线采样接口。
//!
//! # 核心设计与三阶段拆卸生命周期（Three-Phase Teardown）
//! 用户态测试用例会派生最多 4 个绑定了 `UserThread`（或普通内核逻辑）的子任务，并可能阻塞在 `Completion` 门控或睡眠读写锁上。
//! 为确保在回收用户地址空间（`Mm`）与物理页之前，所有执行者均已从用户态返回内核态并彻底退出其内核栈，
//! `Users` 夹具严格划分以下三个阶段：
//!
//! ```text
//!   set_up()                      quiesce()                         tear_down()
//! +-----------------------+    +-------------------------------+    +---------------------------------+
//! | 1. 采样初始资源基线   |    | 1. 置位 stop = true (Release) |    | 1. 断言全部子任务均已退出       |
//! |    initial            | -> | 2. 完成全部 gates 唤醒阻塞者  | -> | 2. 释放 handles / stop / controls|
//! | 2. 初始化 stop = false|    | 3. 轮询等待所有 handles 退出  |    | 3. 重新采样资源基线 actual      |
//! | 3. 清空 4 个任务槽位  |    |    各自内核栈 (try_join 就绪) |    |    并断言与 initial 完全一致    |
//! +-----------------------+    +-------------------------------+    +---------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `Controls::tasks[slot]` 是叶子级自旋锁（`SpinLock`），持锁期间仅拷贝或更新 `Option<Weak<Task>>` 指针，绝不睡眠或嵌套其他锁。
//! - `Controls::blocked()` 先获取 `tasks[slot]` 自旋锁克隆弱引用并立即释放锁，随后在锁外升级为 `Arc<Task>` 并读取调度器状态，避免持自旋锁访问调度器结构。
//! - `Controls::lock` 为可睡眠读写锁（`SleepRwLock`），仅供普通可睡眠任务上下文中的测试逻辑竞争使用，绝不在持有自旋锁期间获取。

use alloc::sync::{Arc, Weak};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::ktest::execution_core::{baseline, until, Baseline};
use crate::ktest::execution_services::SuiteBaseline;
use crate::ktest::{prepare_result, ChildHandle, ChildToken};
use crate::sync::{Completion, SleepRwLock, SpinLock};
use crate::task::UserThread;
use crate::{sched, task};
use ::ktest::{Fixture, TestContext, TestError, TestResult};

/// 用户态执行与虚拟内存测试套件共用的测试夹具（Fixture）。
///
/// 负责跟踪单次测试用例派生的最多 4 个子任务、统一广播停止信号，并在所有子任务退出内核栈后校验内存与工作队列元数据是否完全归还。
pub(super) struct Users {
    /// 测试用例开始前采样的物理内存六域统计与工作队列元数据基线。
    initial: Baseline,
    /// 跨任务共享的停止标志；在 `quiesce` 阶段置为 `true`（`Ordering::Release`），通知子任务在返回内核态后尽快退出。
    stop: Arc<AtomicBool>,
    /// 固定 4 槽位的测试子任务句柄数组，用于在 `joined` 与 `quiesce` 中观察子任务是否已退出各自内核栈。
    handles: [Option<ChildHandle>; 4],
    /// 可选的多任务阻塞与并发交错同步控制器；若用例启用，则在 `quiesce` 时自动唤醒其全部门控。
    controls: Option<Arc<Controls>>,
}

impl Fixture<SuiteBaseline> for Users {
    /// 在普通可睡眠任务上下文中采样初始资源基线，并创建单次测试用例专属的停止标志与空任务槽位。
    fn set_up(_: &mut TestContext<'_>, _: &SuiteBaseline) -> Result<Self, TestError> {
        let initial = baseline()?;
        let stop = Arc::try_new(AtomicBool::new(false))
            .map_err(|_| TestError("NoMemory: user execution controls"))?;
        Ok(Self {
            initial,
            stop,
            handles: core::array::from_fn(|_| None),
            controls: None,
        })
    }

    /// 广播停止标志、唤醒所有同步门控，并等待本用例派生的全部子任务彻底退出各自的内核栈。
    ///
    /// 运行用户态代码的子任务仅在陷入（Trap/Interrupt）返回内核 Rust 栈后才检查 `stop` 标志；
    /// 在 `try_join()` 确认子任务已完全离开内核栈之前，夹具保持所有控制对象与句柄存活，防止提前释放导致悬垂引用。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        // Step 1: 发布停止信号，并唤醒可能阻塞在 `Controls::gates` 上的所有等待任务。
        self.stop.store(true, Ordering::Release);
        if let Some(controls) = &self.controls {
            for gate in &controls.gates {
                gate.complete();
            }
        }
        // Step 2: 轮询让出 CPU，直至所有已发布的子任务执行完毕并完成离栈交接。
        until(
            || {
                self.handles
                    .iter()
                    .flatten()
                    .all(|handle| handle.try_join().is_some())
            },
            "user execution did not leave its original Task stack",
        )
    }

    /// 在所有子任务均已退出内核栈后销毁夹具持有的全部句柄与控制结构，并校验系统资源是否恢复至初始基线。
    ///
    /// 先显式释放 `handles`、`stop` 与 `controls` 以归还最后一个强引用（`Arc`）持有的内存，
    /// 再采样物理内存六域统计（present/reserved/metadata/buddy/pcp/slab）与工作队列元数据并与 `initial` 逐项比对。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let Self {
            initial,
            stop,
            handles,
            controls,
        } = self;
        assert!(handles
            .iter()
            .flatten()
            .all(|handle| handle.try_join().is_some()));
        drop(handles);
        drop(stop);
        drop(controls);
        let actual = baseline()?;
        context.diagnostic(format_args!(
            "USER RESOURCES initial={:?} final={:?}",
            initial, actual
        ));
        if actual == initial {
            Ok(())
        } else {
            Err(TestError("user execution resources did not converge"))
        }
    }
}

impl Users {
    /// 在指定槽位 `slot` 上绑定私有 `UserThread` 并派生执行用户态测试闭包的子任务。
    ///
    /// # 调用约束与参数
    /// - 调用方在普通可睡眠任务上下文中调用；若同一 `slot` 曾运行过子任务，旧子任务必须已退出内核栈（`try_join().is_some()`）。
    /// - `user`：待绑定到新子任务上的用户态线程上下文与地址空间对象。
    /// - `body`：在新子任务内核栈上运行的测试逻辑，接收 `Arc<UserThread>`、`ChildToken` 与共享 `stop` 标志。
    pub(super) fn start(
        &mut self,
        slot: usize,
        user: Arc<UserThread>,
        body: impl FnOnce(Arc<UserThread>, ChildToken, Arc<AtomicBool>) -> Result<i32, TestError>
            + Send
            + 'static,
    ) -> TestResult {
        assert!(self.handles[slot]
            .as_ref()
            .is_none_or(|handle| handle.try_join().is_some()));
        let owned = Arc::clone(&user);
        let stop = Arc::clone(&self.stop);
        let prepared = prepare_result(move |token| body(owned, token, stop))?;
        prepared.bind_user(user);
        self.handles[slot] = Some(prepared.publish()?);
        Ok(())
    }

    /// 等待槽位 `slot` 上的子任务彻底退出内核栈，并断言其返回值等于 `expected`。
    ///
    /// 子任务的返回值仅在其完全离开内核栈后才由 `ChildHandle::try_join()` 暴露；若子任务返回 `Err(TestError)`，则由外层 `ktest` 作用域捕获并上报。
    pub(super) fn joined(&self, slot: usize, expected: i32) -> TestResult {
        let handle = self.handles[slot]
            .as_ref()
            .expect("user child was not published");
        until(|| handle.try_join().is_some(), "user child did not finish")?;
        assert_eq!(handle.try_join(), Some(expected));
        Ok(())
    }

    /// 在指定槽位 `slot` 上派生不绑定 `UserThread` 的普通内核测试子任务（例如用于配合测试睡眠读写锁竞争）。
    ///
    /// 若同一 `slot` 曾运行过子任务，旧子任务必须已退出内核栈；新任务的句柄同样纳入 `handles[slot]` 由夹具统一停止与回收。
    pub(super) fn start_task(
        &mut self,
        slot: usize,
        body: impl FnOnce(ChildToken, Arc<AtomicBool>) -> Result<i32, TestError> + Send + 'static,
    ) -> TestResult {
        assert!(self.handles[slot]
            .as_ref()
            .is_none_or(|handle| handle.try_join().is_some()));
        let stop = Arc::clone(&self.stop);
        self.handles[slot] = Some(prepare_result(move |token| body(token, stop))?.publish()?);
        Ok(())
    }
}

/// 多任务虚拟内存并发测试的阻塞门控与调度状态观察控制器。
///
/// 提供固定 4 槽位的完成门控、就绪标志、弱任务引用及共享阶段计数/睡眠读写锁，用于精确编排跨任务缺页与锁竞争时序。
pub(super) struct Controls {
    /// 每个参与槽位专属的单次/可完成门控，用于阻塞子任务直至协调者放行或 `quiesce` 阶段统一唤醒。
    pub(super) gates: [Completion; 4],
    /// 每个参与槽位到达预定同步点后置位的原子就绪标志。
    pub(super) ready: [AtomicBool; 4],
    /// 记录各槽位子任务 `Weak<Task>` 的自旋锁数组；使用弱引用避免延长任务对象的生命周期。
    tasks: [SpinLock<Option<Weak<task::Task>>>; 4],
    /// 跨任务推进测试步骤的原子阶段计数器。
    pub(super) phase: AtomicUsize,
    /// 供多任务并发读写竞争测试使用的睡眠读写锁（初始受保护值为 `0x9817`）。
    pub(super) lock: SleepRwLock<u64>,
}

impl Controls {
    /// 分配并初始化多任务同步控制器，同时将其强引用注册到 `fixture.controls` 以便异常退出时由 `quiesce` 自动唤醒门控。
    pub(super) fn new(fixture: &mut Users) -> Result<Arc<Self>, TestError> {
        let controls = Arc::try_new(Self {
            gates: core::array::from_fn(|_| Completion::new()),
            ready: core::array::from_fn(|_| AtomicBool::new(false)),
            tasks: core::array::from_fn(|_| SpinLock::new(None)),
            phase: AtomicUsize::new(0),
            lock: SleepRwLock::new(0x9817),
        })
        .map_err(|_| TestError("NoMemory: user VM controls"))?;
        fixture.controls = Some(Arc::clone(&controls));
        Ok(controls)
    }

    /// 将当前正在执行的内核任务以弱引用（`Weak<Task>`）形式记录到 `slot` 槽位中，供协调者后续查询其调度状态。
    pub(super) fn record(&self, slot: usize) {
        *self.tasks[slot].lock() = Some(Arc::downgrade(&task::current_task()));
    }

    /// 检查 `slot` 槽位记录的任务当前在调度器中是否已真正进入阻塞（`State::Blocked`）状态并让出 CPU。
    ///
    /// 先在 `SpinLock` 临界区内克隆 `Weak<Task>` 并立即释放自旋锁，随后升级强引用查询调度状态，避免持锁跨层调用。
    pub(super) fn blocked(&self, slot: usize) -> bool {
        let task = self.tasks[slot]
            .lock()
            .clone()
            .and_then(|task| task.upgrade());
        task.is_some_and(|task| sched::test_state(&task) == task::State::Blocked)
    }
}
