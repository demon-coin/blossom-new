//! 用户态 `futex`（快速用户空间互斥体）与内核测试协调器的端到端集成测试。
//!
//! # 模块定位与职责
//! 本模块位于文件系统与用户态运行时集成测试层（`fs::ktest::user`），
//! 负责加载并运行用户态测试程序 `/test/bin/user-futex`，通过内核测试协调器（`UserOwner`）
//! 与用户态进程协同，验证 `futex` 系统调用在各种并发与生命周期场景下的内核语义。
//! 上游由用户态测试入口（`fs::ktest::user`）按子模式（`w`/`p`/`n`/`a`/`t`/`s`/`d`/`r`）调度，
//! 下游依赖虚拟内存空间（`Mm`）、任务与进程图（`TaskGraph` / `UserThread`）、
//! 调度器状态观测（`sched::test_state`）以及 `futex` 哈希桶快照接口（`futex::test_snapshot`）。
//!
//! # 核心协调机制与共享控制页布局
//! 测试协调器与用户态测试程序通过映射在 `FUTEX_CONTROL` 的共享控制页交换检查点（Checkpoint）
//! 与放行门禁（Gate）信号。仅凭用户态写入的检查点状态字不足以证明线程已在内核中完成阻塞入队，
//! 因此协调器必须同时观测到：
//! 1. `futex` 哈希桶快照中对应键（Key）的等待者计数为 `1`；
//! 2. 线程绑定的底层调度任务（`Task`）状态已切换为 `State::Blocked`；
//!
//! 才会向放行门禁字写入下一阶段编号以唤醒或推进后续操作。
//!
//! `FUTEX_CONTROL` 共享控制页关键偏移布局如下：
//! ```text
//! +-----------------------+-------------------------------------------------------+
//! | 字节偏移 (Offset)     | 用途说明                                              |
//! +-----------------------+-------------------------------------------------------+
//! | +0  (usize)           | 主受测子线程/子进程的 TID（由用户态发布）             |
//! | +8  (u32)             | 用户态当前到达的协议阶段号（Phase Checkpoint）        |
//! | +12 (u32)             | 内核协调器向用户态下发的放行门禁号（Release Gate）    |
//! | +16 (usize)           | 第二受测子线程的 TID（用于 `r` 模式的多线程重排队）   |
//! | +32 (u32)             | 信号处理函数已执行标志（用于 `s` 模式的信号中断重入） |
//! | +64 (u32)             | 默认受测 `futex` 字（部分模式使用 +68 作为重排队目标）|
//! +-----------------------+-------------------------------------------------------+
//! ```
//!
//! # 锁顺序与并发契约
//! - 本模块作为测试协调器运行在独立内核任务上下文中，不直接持有 `futex` 哈希桶自旋锁或页表锁。
//! - 调用 `mm.load_user_u32`、`mm.store_user_u32` 与 `futex::test_snapshot` 时，内部各自遵循
//!   `Mm` 地址空间锁与 `futex` 桶自旋锁的既定获取顺序，两次调用之间不跨锁嵌套。

use super::*;
use crate::task::futex::{self, FutexSnapshot};

impl UserOwner {
    /// 等待用户态测试根线程推进到指定的协议阶段号（`phase`）。
    ///
    /// 仅用于同步用户态控制流已到达指定协议位置；目标线程是否已在内核 `futex` 队列中完成登记并阻塞，
    /// 需随后调用 [`Self::futex_waiting`] 结合哈希桶快照与调度器状态进一步确认。
    /// 若根线程在到达检查点前提前退出（`root.mm().is_none()`），则立即返回错误。
    fn futex_phase(mm: &Mm, root: &task::UserThread, phase: u32) -> TestResult {
        until("futex user did not reach its checkpoint", || {
            mm.load_user_u32(FUTEX_CONTROL + 8) == Ok(phase) || root.mm().is_none()
        })?;
        if mm.load_user_u32(FUTEX_CONTROL + 8) != Ok(phase) {
            return Err(TestError("futex user exited before its checkpoint"));
        }
        Ok(())
    }

    /// 从共享控制页指定偏移（`FUTEX_CONTROL + slot`）读取用户态发布的子线程 TID，
    /// 并在任务图（`TaskGraph`）中查找对应的内核用户线程句柄（`UserThread`）。
    pub(super) fn futex_child(
        &self,
        mm: &Mm,
        slot: usize,
    ) -> Result<Arc<task::UserThread>, TestError> {
        let pid = mm
            .read_usize(FUTEX_CONTROL + slot)
            .map_err(|_| TestError("futex checkpoint TID is inaccessible"))?;
        self.graph()
            .lookup_thread(pid)
            .ok_or(TestError("futex checkpoint child disappeared"))
    }

    /// 轮询等待目标用户线程在指定地址的 `futex` 键上完成内核入队登记且调度状态进入 `State::Blocked`，
    /// 随后返回该 `futex` 桶的快照（包含首个等待者的 `WaitId` 与超时截止时间 `deadline`）。
    ///
    /// 共享内存中的阶段提示字只能在 `futex_wait` 系统调用发起前更新，不能直接证明线程已进入睡眠；
    /// 必须同时观测到 `snapshot.count == 1` 与 `sched::test_state(&executor) == State::Blocked`，
    /// 才能消除用户态刚写完标志但尚未真正进入内核阻塞队列的竞态窗口。
    pub(super) fn futex_waiting(
        mm: &Arc<Mm>,
        user: &task::UserThread,
        address: usize,
        private: bool,
        mask: u32,
    ) -> Result<FutexSnapshot, TestError> {
        let executor = user
            .executor()
            .ok_or(TestError("futex checkpoint executor disappeared"))?;
        let mut observed = None;
        until("futex user did not register and block", || {
            if let Ok(snapshot) = futex::test_snapshot(mm, address, private, mask) {
                if snapshot.count == 1 && sched::test_state(&executor) == State::Blocked {
                    observed = Some(snapshot);
                    return true;
                }
            }
            user.mm().is_none()
        })?;
        observed.ok_or(TestError("futex user exited before registering"))
    }

    /// 运行 `/test/bin/user-futex` 测试程序并根据子模式（`mode`）执行对应的内核侧协调与断言。
    ///
    /// - **测试目标**：验证私有/共享 `futex` 唤醒、跨进程键隔离、共享匿名映射别名（Alias）、
    ///   线程退出 `clear_child_tid`、信号中断自动重启（`ERESTARTSYS`）、`SIGSTOP`/`SIGCONT` 期间超时参数页卸载，
    ///   以及 `FUTEX_CMP_REQUEUE` / `FUTEX_WAIT_BITSET` 在重排队与取消时的 `WaitId` 和 `deadline` 保持语义。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 加载并启动 `/test/bin/user-futex`，等待根线程发布阶段 1 检查点并解析受测子线程；
    ///   2. 根据 `mode[0]` 分支观测子线程阻塞状态、注入跨地址空间映射或内核信号、核对哈希桶快照与等待者元数据；
    ///   3. 回收全部子进程并等待根进程以状态码 `0` 正常退出，断言地址空间引用计数归零。
    pub(super) fn futex_program(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
    ) -> TestResult {
        let image = self.load_named(context, b"/test/bin/user-futex", mode)?;
        let mm = Arc::clone(&image.mm);
        let fs_context = self.path.context().clone();
        self.start_program(context, image, fs_context, true, false, Credentials::ROOT)?;
        let pid = self
            .running
            .as_ref()
            .expect("futex program was not published")
            .pid;
        let root = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("futex root binding disappeared"))?;

        // Step 1: 等待用户态根线程完成初始化并到达阶段 1 检查点，确定受测 futex 虚拟地址。
        Self::futex_phase(&mm, &root, 1)?;
        let address = if mode[0] == b'w' {
            0x4600_0000
        } else {
            FUTEX_CONTROL + 64
        };
        if mode[0] == b't' {
            Self::futex_waiting(&mm, &root, address, false, u32::MAX)?;
        }
        let child = self.futex_child(&mm, 0)?;
        let child_mm = child.mm().ok_or(TestError("futex child MM disappeared"))?;

        // Step 2: 按测试子模式执行对应的阻塞状态观测、信号注入与哈希桶快照校验。
        match mode[0] {
            b'w' => {
                // 模式 'w'：验证基本的私有 futex 等待与唤醒。
                Self::futex_waiting(&child_mm, &child, address, true, u32::MAX)?;
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex wake checkpoint release failed"))?;
            }
            b'p' | b'n' => {
                // 模式 'p' / 'n'：验证不同地址空间下的私有键（'p'）或独立匿名映射（'n'）键隔离，
                // 确认父进程相同虚拟地址上的 futex 桶计数为 0，随后验证信号唤醒被阻塞的子进程。
                let private = mode[0] == b'p';
                let address = if private { address } else { 0x4600_0000 };
                Self::futex_waiting(&child_mm, &child, address, private, u32::MAX)?;
                let isolated = futex::test_snapshot(&mm, address, private, u32::MAX)
                    .map_err(|_| TestError("isolated futex key lookup failed"))?;
                context.expect_eq(isolated.count, 0);
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex isolation checkpoint release failed"))?;
                Self::futex_phase(&mm, &root, 2)?;
                let child_pid = child.identity().expect("futex child has no identity").tid();
                self.graph()
                    .send_kernel(child_pid, crate::task::signal::SIGUSR1)
                    .map_err(|_| TestError("futex isolation signal request failed"))?;
                mm.store_user_u32(FUTEX_CONTROL + 12, 2)
                    .map_err(|_| TestError("futex isolation signal release failed"))?;
            }
            b'a' => {
                // 模式 'a'：在子进程地址空间中为 FUTEX_CONTROL 建立不同虚拟地址（0x4500_0000）的共享匿名映射别名，
                // 验证非私有（shared）futex 通过不同虚拟地址访问同一物理页时共享相同的内核 futex 键与 WaitId。
                child_mm
                    .test_shared_alias(FUTEX_CONTROL, 0x4500_0000, 4096)
                    .map_err(|error| {
                        context.diagnostic(format_args!("user-futex-alias error={:?}", error));
                        TestError("shared anonymous futex alias preparation failed")
                    })?;
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex alias checkpoint release failed"))?;
                Self::futex_phase(&mm, &root, 2)?;

                let alias = Self::futex_waiting(&child_mm, &child, 0x4500_0040, false, u32::MAX)?;
                let original = futex::test_snapshot(&mm, address, false, u32::MAX)
                    .map_err(|_| TestError("shared futex original key lookup failed"))?;
                context.expect_eq(original.count, 1);
                context.expect_eq(original.first_wait_id, alias.first_wait_id);
                mm.store_user_u32(FUTEX_CONTROL + 12, 2)
                    .map_err(|_| TestError("futex shared-key wake release failed"))?;
            }
            b't' => {
                // 模式 't'：验证线程创建时设置的 clear_child_tid 地址在子线程退出时由内核自动清零并触发 futex 唤醒。
                context.expect_eq(
                    mm.load_user_u32(address),
                    Ok(child.identity().expect("join child has no identity").tid() as u32),
                );
                context.expect_eq(mm.user_count(), 2);
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex child exit release failed"))?;
            }
            b's' => {
                // 模式 's'：验证无超时的 futex_wait 被安装有 SA_RESTART 的 SIGUSR1 中断后，
                // 执行完用户态信号处理函数并自动重启系统调用，重新入队后生成新的 WaitId 且无超时截止时间。
                let first = Self::futex_waiting(&child_mm, &child, address, true, u32::MAX)?;
                let child_pid = child
                    .identity()
                    .expect("restart child has no identity")
                    .tid();
                self.graph()
                    .send_kernel(child_pid, crate::task::signal::SIGUSR1)
                    .map_err(|_| TestError("futex restart signal request failed"))?;
                until("futex restart handler did not execute", || {
                    mm.load_user_u32(FUTEX_CONTROL + 32) == Ok(1) || child.mm().is_none()
                })?;

                let restarted = Self::futex_waiting(&child_mm, &child, address, true, u32::MAX)?;
                context.expect_eq(first.first_wait_id == restarted.first_wait_id, false);
                context.expect_eq(restarted.first_deadline, None);
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex restarted wake release failed"))?;
            }
            b'd' => {
                // 模式 'd'：验证带相对超时的 futex_wait 在内核登记绝对截止时间（deadline）后，
                // 即使子进程收到 SIGSTOP 暂停、父进程卸载了原本存放 timespec 的用户页（0x4700_0000），
                // 且绝对截止时间在暂停期间已到期，SIGCONT 恢复后仍按已登记的超时正常返回，不会再次读取已卸载的用户页。
                let first = Self::futex_waiting(&child_mm, &child, address, true, u32::MAX)?;
                let deadline = first
                    .first_deadline
                    .ok_or(TestError("timed futex registration has no deadline"))?;
                let child_pid = child
                    .identity()
                    .expect("deadline child has no identity")
                    .tid();

                self.graph()
                    .send_kernel(child_pid, crate::task::signal::SIGSTOP)
                    .map_err(|_| TestError("futex deadline stop request failed"))?;
                let parent = root
                    .identity()
                    .expect("deadline parent has no identity")
                    .group();
                let selector = WaitSelector::Pid(
                    self.graph()
                        .pid_object(child_pid)
                        .ok_or(TestError("futex child identity disappeared"))?,
                );
                let mut stopped = None;
                let mut wait_error = None;
                until("futex child did not acknowledge group stop", || match self
                    .graph()
                    .try_wait_event(&parent, &selector, 0x0100_0003)
                {
                    Ok(Some(event)) => {
                        stopped = Some(event);
                        true
                    }
                    Ok(None) => false,
                    Err(error) => {
                        wait_error = Some(error);
                        true
                    }
                })?;
                if let Some(error) = wait_error {
                    context.diagnostic(format_args!("futex-stop-wait error={:?}", error));
                    return Err(TestError("futex stopped event lookup failed"));
                }
                let stopped = stopped.ok_or(TestError("futex stopped event is absent"))?;
                context.expect_eq(stopped.code, 5);
                context.expect_eq(stopped.status, 19);

                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex timeout unmap release failed"))?;
                Self::futex_phase(&mm, &root, 2)?;
                context.expect_eq(mm.read_u64(0x4700_0000), Err(VmError::Unmapped));
                until("futex original deadline was not reached", || {
                    deadline <= crate::time::now()
                })?;
                context.diagnostic(format_args!(
                    "user-futex-deadline first-deadline={} now={} stopped={} timeout-page-unmapped=1",
                    deadline.as_nanos(), crate::time::now().as_nanos(), stopped.status
                ));

                self.graph()
                    .send_kernel(child_pid, crate::task::signal::SIGCONT)
                    .map_err(|_| TestError("futex deadline continue request failed"))?;
                mm.store_user_u32(FUTEX_CONTROL + 12, 2)
                    .map_err(|_| TestError("futex deadline continue release failed"))?;
            }
            b'r' => {
                // 模式 'r'：验证 FUTEX_WAIT_BITSET 按位掩码选择性唤醒，以及 FUTEX_CMP_REQUEUE 在源键与目标键之间
                // 迁移等待者时完整保留原有的 WaitId 与绝对超时 deadline，且被重排队后的等待者仍可正常被信号或超时取消。
                Self::futex_waiting(&child_mm, &child, address, true, 1)?;
                let second = self.futex_child(&mm, 16)?;
                let first = Self::futex_waiting(&mm, &second, address, true, 2)?;
                let wait_id = first
                    .first_wait_id
                    .ok_or(TestError("bitset wait has no identity"))?;
                let deadline = first
                    .first_deadline
                    .ok_or(TestError("bitset wait has no deadline"))?;
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex bitset selection release failed"))?;
                for (phase, current) in [(2u32, address + 4), (3, address)].iter().copied() {
                    Self::futex_phase(&mm, &root, phase)?;
                    let moved = Self::futex_waiting(&mm, &second, current, true, 2)?;
                    context.expect_eq(moved.first_wait_id, Some(wait_id));
                    context.expect_eq(moved.first_deadline, Some(deadline));
                    let previous = if phase == 2 { address } else { address + 4 };
                    let empty = futex::test_snapshot(&mm, previous, true, u32::MAX)
                        .map_err(|_| TestError("requeue previous key lookup failed"))?;
                    context.expect_eq(empty.count, 0);
                    context.diagnostic(format_args!(
                        "user-futex-requeue phase={} wait-id={:?} deadline={} current={:#x}",
                        phase,
                        moved.first_wait_id,
                        deadline.as_nanos(),
                        current
                    ));
                    mm.store_user_u32(FUTEX_CONTROL + 12, phase)
                        .map_err(|_| TestError("futex requeue checkpoint release failed"))?;
                }

                for (phase, mask, signal) in [(4u32, 4u32, true), (6, 8, false)].iter().copied() {
                    Self::futex_phase(&mm, &root, phase)?;
                    let waiting = self.futex_child(&mm, 0)?;
                    let first = Self::futex_waiting(&mm, &waiting, address, true, mask)?;
                    let wait_id = first
                        .first_wait_id
                        .ok_or(TestError("requeue wait has no identity"))?;
                    let deadline = first
                        .first_deadline
                        .ok_or(TestError("requeue wait has no deadline"))?;
                    mm.store_user_u32(FUTEX_CONTROL + 12, phase)
                        .map_err(|_| TestError("futex cancellation requeue release failed"))?;
                    Self::futex_phase(&mm, &root, phase + 1)?;
                    let moved = Self::futex_waiting(&mm, &waiting, address + 4, true, mask)?;
                    context.expect_eq(moved.first_wait_id, Some(wait_id));
                    context.expect_eq(moved.first_deadline, Some(deadline));
                    let empty = futex::test_snapshot(&mm, address, true, u32::MAX)
                        .map_err(|_| TestError("requeue cancelled source key lookup failed"))?;
                    context.expect_eq(empty.count, 0);
                    if signal {
                        let waiting_pid = waiting
                            .identity()
                            .expect("requeue child has no identity")
                            .tid();
                        self.graph()
                            .send_kernel(waiting_pid, crate::task::signal::SIGUSR1)
                            .map_err(|_| TestError("futex requeue cancellation signal failed"))?;
                    } else {
                        until("requeued futex deadline was not reached", || {
                            deadline <= crate::time::now()
                        })?;
                    }
                    context.diagnostic(format_args!(
                        "user-futex-requeue-cancel phase={} wait-id={:?} deadline={} signal={}",
                        phase + 1,
                        moved.first_wait_id,
                        deadline.as_nanos(),
                        signal
                    ));
                    mm.store_user_u32(FUTEX_CONTROL + 12, phase + 1)
                        .map_err(|_| {
                            TestError("futex requeue cancellation checkpoint release failed")
                        })?;
                }
            }
            _ => unreachable!("unknown futex fixture mode"),
        }

        // Step 3: 等待用户态根进程退出，回收所有子进程并验证地址空间用户引用计数彻底归零。
        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        let cancelling = event.status != 0;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;

        context.expect_eq(mm.user_count(), 0);
        context.diagnostic(format_args!(
            "user-futex-result mode={:?} status={} joined=1 mm-users={}",
            mode,
            event.status,
            mm.user_count()
        ));
        Ok(())
    }
}
