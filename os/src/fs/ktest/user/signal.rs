//! 用户态信号投递、文件页等待、时钟抢占与子进程退出通知的集成测试模块。
//!
//! ## 模块职责与架构定位
//! 本模块隶属于 `fs::ktest::user` 测试套件，用于验证内核信号子系统（`task::signal`）与页面缓存（Page Cache）、
//! 调度器时钟中断（Timer Preemption）、跨核函数调用（IPI CSD）以及进程克隆（`clone`）/暂停（`sigsuspend`）
//! 之间的协同行为。上游由 `user.rs` 测试用例入口调用，下游驱动 `/test/bin/user-signal` 用户态测试程序。
//!
//! ## 核心机制与用户栈同步协议
//! 测试协调者（内核测试线程）通过读取和修改用户初始栈帧下方预留的控制字（Marker），与用户态程序进行阶段同步：
//!
//! ```text
//! 高地址 (初始用户栈指针 SP: riscv64 gpr[2] / loongarch64 gpr[3])
//! +---------------------------------------------------------------+
//! | SP - 8  (单字模式: "r" / "i")                                 |
//! |   - 0x51a2: 用户态已进入文件 read 前置检查点                   |
//! |   - 0x51a1: 用户态已进入无限循环体                             |
//! +---------------------------------------------------------------+
//! | SP - 16 (双字检查点模式: "e" / "u")                           |
//! |   - [marker + 0..8]  (usize): 子进程/子线程的 PID              |
//! |   - [marker + 8..12] (u32)  : 阶段状态机控制字 (Phase Gate)    |
//! |       1 -> 子任务已就绪，等待内核测试线程观测或回收             |
//! |       2 -> 内核已放行第一阶段，通知父进程继续执行               |
//! |       3 -> 第二阶段就绪或父进程已完成共享参数页解除映射         |
//! |       4 -> 内核放行第二阶段                                     |
//! +---------------------------------------------------------------+
//! 低地址
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 1. 文件页预占与填充遵循 `Inode::lock_invalidate()` 读锁先于 `Folio` 内容初始化的顺序，并在唤醒等待者后及时释放失效锁。
//! 2. 跨核信号发送（`cpu::call`）期间，调用方不得持有目标核回调可能竞争的任务图或信号锁，且栈上参数必须在同步等待完成前保持有效。

use super::*;
use crate::mm::reclaim::ReclaimAttempt;

impl UserOwner {
    /// 预读 `/test/bin/user-signal` 可执行文件的全部数据页到页面缓存中。
    ///
    /// 这样可以确保后续启动用户进程执行指令时不会触发磁盘缺页 I/O，从而避免将代码段的页面加载等待误判为用户数据读取阻塞。
    pub(super) fn warm_signal_program(&self, context: &TestContext<'_>) -> TestResult {
        let program = self.file(context, b"/test/bin/user-signal")?;
        let mut bytes = [0u8; 4096];
        let mut offset = 0;
        loop {
            let count = checked(context, program.read_at(offset, &mut bytes))?;
            if count == 0 {
                return Ok(());
            }
            offset += count as u64;
        }
    }

    /// 测试目标：验证用户线程在等待页面缓存填充（不可中断磁盘读路径）期间收到 `SIGALRM` 信号时的挂起与延迟交付行为。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 预先在 `/test/data.bin` 的第 0 页分配并发布一个尚未填充数据的物理页（`Folio`），由测试线程持有其填充凭证与 `invalidate` 锁，并预热可执行文件页面。
    /// 2. When: 启动用户程序读取该文件，观测到用户线程写入 `0x51a2` 标记且调度状态变为 `Blocked` 后，向其发送 `SIGALRM` 信号。
    /// 3. Then: 验证信号被标记为待处理（`interruption_pending` 为 `true`），但正在等待页面填充的线程仍保持 `Blocked` 状态；随后由测试线程完成磁盘页读取并唤醒页面，验证用户程序处理信号后以状态码 `0` 正常退出。
    pub(super) fn signal_during_file_wait(&mut self, context: &mut TestContext<'_>) -> TestResult {
        let path = checked(
            context,
            self.path
                .vfs()
                .resolve(self.path.context(), b"/test/data.bin", ResolveMode::Follow),
        )?;
        let inode = checked(context, path.inode())?.clone();
        // Step 1: 预占文件第 0 页的页面缓存槽位并持有未完成的读凭证，使后续用户态 read 必然在此页上阻塞。
        let mut attempt = ReclaimAttempt::new();
        let mut prepared = Some(checked(
            context,
            inode.mapping().prepare_file(&inode, 0, &mut attempt),
        )?);
        let invalidate = checked(context, inode.lock_invalidate())?;
        let (folio, read) = checked(
            context,
            inode
                .mapping()
                .publish_file(&inode, 0, &mut prepared)
                .map_err(FsError::Io),
        )?
        .ok_or(TestError("fresh file candidate retired before publication"))?;
        let read = read.ok_or(TestError("fresh canonical file page already filled"))?;

        // Step 2: 预热测试程序指令页，加载并启动用户态进程（模式 "r"）。
        self.warm_signal_program(context)?;
        let image = self.load_named(context, b"/test/bin/user-signal", b"r")?;
        #[cfg(target_arch = "riscv64")]
        let marker = image.context.gpr[2] - 8;
        #[cfg(target_arch = "loongarch64")]
        let marker = image.context.gpr[3] - 8;
        let mm = Arc::clone(&image.mm);
        let fs_context = self.path.context().clone();
        self.start_program(context, image, fs_context, true, false, Credentials::ROOT)?;
        let pid = self
            .running
            .as_ref()
            .expect("file program is not published")
            .pid;
        let user = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("file reader user binding disappeared"))?;
        let executor = user
            .executor()
            .ok_or(TestError("file reader executor disappeared"))?;

        // Step 3: 等待用户线程进入系统调用并实际阻塞在页面缓存等待队列上，随后发送 SIGALRM。
        until("user file reader did not block on canonical page", || {
            let mut marker_bytes = [0u8; 8];
            let progress = mm.copyin(&mut marker_bytes, marker);
            progress.error.is_none()
                && u64::from_le_bytes(marker_bytes) == 0x51a2
                && sched::test_state(&executor) == State::Blocked
        })?;
        self.graph()
            .send_kernel(pid, crate::task::signal::SIGALRM)
            .map_err(|error| {
                context.diagnostic(format_args!("user-file-signal-send error={:?}", error));
                TestError("user file signal request failed")
            })?;
        context.expect_eq(sched::test_state(&executor), State::Blocked);
        let signals = user
            .signals()
            .ok_or(TestError("file reader signals disappeared"))?;
        context.expect_eq(
            signals.interruption_pending(crate::wait::WaitMode::Interruptible),
            true,
        );

        // Step 4: 执行底层块设备读取以填满该物理页，唤醒用户线程并验证其完整读取数据及处理信号后退出。
        checked(context, fill_file_page(&inode, 0, read, &invalidate))?;
        checked(context, folio.wait_ready().map_err(FsError::Io))?;
        drop(invalidate);
        drop(prepared);

        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        self.reap_children(context, false)?;
        context.diagnostic(format_args!(
            "user-file-signal canonical-page-wait=1 dma-inflight-at-send=0 delivered-bytes=65536"
        ));
        Ok(())
    }

    /// 测试目标：验证在远端 CPU 上执行纯用户态死循环的进程能够被本地时钟中断抢占，并正确响应跨核发送的 `SIGKILL` 信号终止。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 将用户态死循环程序（模式 `"i"`）与一个同核内核观察线程绑定到不同于当前协调核的目标 CPU（`target`）上运行。
    /// 2. When: 先发布观察线程再发布用户进程；用户进程进入死循环并写入 `0x51a1` 后，依赖硬件时钟中断触发抢占使同核观察线程得以运行并确认进度；随后通过 `cpu::call` 在协调核上向该进程发送 `SIGKILL`。
    /// 3. Then: 验证进程因收到信号 9（`SIGKILL`）终止，且运行期间记录到的用户态时钟中断次数大于 0。
    pub(super) fn preempt_infinite(&mut self, context: &mut TestContext<'_>) -> TestResult {
        let image = self.load_named(context, b"/test/bin/user-signal", b"i")?;
        #[cfg(target_arch = "riscv64")]
        let marker = image.context.gpr[2] - 8;
        #[cfg(target_arch = "loongarch64")]
        let marker = image.context.gpr[3] - 8;
        let mm = Arc::clone(&image.mm);
        let runtime = checked(
            context,
            UserRuntime::try_new(
                self.path.vfs().clone(),
                self.path.context().clone(),
                Credentials::ROOT,
                self.net.clone(),
            ),
        )?;
        // Step 1: 选择异核作为目标 CPU，准备用户态死循环任务与同核内核观察任务。
        let coordinator = cpu::current().id;
        let target = (coordinator + 1) % cpu::count();
        let prepared =
            task::prepare_program(self.graph(), None, image, runtime, Placement::Cpu(target))
                .map_err(|_| TestError("infinite user preparation failed"))?;
        let pid = prepared.pid();
        let user = Arc::downgrade(&prepared.user());
        let ready = Arc::try_new(Completion::new())
            .map_err(|_| TestError("infinite observer completion allocation failed"))?;
        let worker_ready = Arc::clone(&ready);
        let observer = task::prepare_kernel_on(target, move || {
            let mut bytes = [0u8; 8];
            let result = loop {
                let progress = mm.copyin(&mut bytes, marker);
                if progress.error.is_some() {
                    break -1;
                }
                if u64::from_le_bytes(bytes) == 0x51a1 {
                    break 1;
                }
                if user.upgrade().is_none_or(|user| user.mm().is_none()) {
                    break -2;
                }
                sched::yield_now();
            };
            worker_ready.complete();
            result
        })
        .map_err(|_| TestError("infinite same-CPU observer preparation failed"))?;

        // Step 2: 先将同核观察者发布为就绪态，再发布用户任务，确保后续只有目标核的时钟中断抢占能切回观察者。
        self.observer = Some(observer.publish());
        // 观察者任务已提前就绪，避免后续发布观察者时的跨核唤醒 IPI 替代硬件时钟中断抢占。
        let join = prepared
            .publish()
            .map_err(|_| TestError("infinite user publication failed"))?;
        self.running = Some(RunningProgram {
            pid,
            target: Some(target),
            coordinator,
            expected_task: Some(join.task_id()),
            join,
        });

        // Step 3: 等待同核观察者被时钟中断抢占唤醒并确认用户程序已写入 0x51a1 进度标记。
        ready.wait();
        let advanced = self.observer.take().unwrap().join();
        context.expect_eq(advanced, 1);

        /// 跨核同步调用（IPI CSD）发送 `SIGKILL` 信号的参数与结果载体。
        struct RemoteKill {
            /// 待操作的目标任务图强引用。
            graph: Arc<TaskGraph>,
            /// 接收信号的目标进程 PID。
            pid: usize,
            /// 实际执行回调的发送端 CPU 编号。
            sender: usize,
            /// 内核信号发送接口的执行结果。
            result: Option<Result<(), crate::task::signal::SignalError>>,
        }
        /// 在指定协调核的 IPI 回调上下文中向目标进程发送 `SIGKILL` 信号。
        ///
        /// # Safety
        /// 1. `argument` 必须指向调用栈上有效、对齐且独占访问的 [`RemoteKill`] 实例，且其生命周期持续到同步 `cpu::call` 返回之后。
        /// 2. 回调在关中断的 IPI 上下文中执行，内部仅调用非阻塞的信号投递逻辑，不睡眠且不访问用户态内存。
        unsafe fn send_remote_kill(argument: *mut ()) {
            // SAFETY:
            // 1. 调用方传入了有效且对齐的 `&mut kill` 指针。
            // 2. `cpu::call` 通过 Release/Acquire 内存序同步，且调用线程在 `cpu::call` 返回前阻塞等待，不会并发读写 `kill` 的任何字段。
            let request = unsafe { &mut *argument.cast::<RemoteKill>() };
            request.sender = cpu::current().id;
            request.result = Some(
                request
                    .graph
                    .send_kernel(request.pid, crate::task::signal::SIGKILL),
            );
        }
        let mut kill = RemoteKill {
            graph: Arc::clone(self.graph()),
            pid,
            sender: usize::MAX,
            result: None,
        };
        // Step 4: 在原协调核上发起跨核信号发送，随后等待目标进程被终止并校验退出状态与时钟中断计数。
        // SAFETY:
        // 1. `coordinator` 是当前在线的 CPU 编号，且当前线程不持有 `send_remote_kill` 所需的任何自旋锁，可正常响应 IPI。
        // 2. `kill` 位于当前栈帧且持有 `TaskGraph` 的 `Arc` 强引用，在 `cpu::call` 同步等待返回前始终有效且无别名冲突。
        unsafe {
            cpu::call(
                coordinator,
                send_remote_kill,
                (&mut kill as *mut RemoteKill).cast(),
            )
        };
        context.expect_eq(kill.sender, coordinator);
        context.expect_eq(kill.sender == target, false);
        kill.result
            .expect("remote kill callback did not complete")
            .map_err(|error| {
                context.diagnostic(format_args!("user-remote-kill-send error={:?}", error));
                TestError("remote user kill request failed")
            })?;

        let event = self.wait(context)?;
        context.expect_eq(event.status, 9);
        context.expect_eq(0 < event.user_timer_interrupts, true);
        self.reap_children(context, false)?;
        Ok(())
    }

    /// 启动指定模式的 `/test/bin/user-signal` 测试程序，并返回其地址空间引用、栈上检查点控制字地址及主线程对象。
    fn start_signal_checkpoint(
        &mut self,
        context: &TestContext<'_>,
        mode: &[u8],
    ) -> Result<(Arc<crate::mm::space::Mm>, usize, Arc<task::UserThread>), TestError> {
        let image = self.load_named(context, b"/test/bin/user-signal", mode)?;
        #[cfg(target_arch = "riscv64")]
        let marker = image.context.gpr[2] - 16;
        #[cfg(target_arch = "loongarch64")]
        let marker = image.context.gpr[3] - 16;
        let mm = Arc::clone(&image.mm);
        let fs_context = self.path.context().clone();
        self.start_program(context, image, fs_context, true, false, Credentials::ROOT)?;
        let pid = self
            .running
            .as_ref()
            .expect("signal program is not published")
            .pid;
        let user = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("signal checkpoint user disappeared"))?;
        Ok((mm, marker, user))
    }

    /// 测试目标：验证 `clone` 创建的子任务在配置零退出信号与自定义退出信号时的进程等待回收与通知语义。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 启动模式为 `"e"` 的用户程序，在栈检查点 `marker` 处建立父子阶段同步。
    /// 2. When: 依次在阶段 `1` 和阶段 `3` 等待用户程序创建子任务并发布其 PID，由内核代表父进程组执行 `wait` 回收子任务并确认其完全退出内核栈后，将阶段号递增放行用户态继续检查信号状态。
    /// 3. Then: 两个阶段的子任务均以状态码 `0` 正常回收，最终用户主进程完成全部断言并以状态码 `0` 退出。
    pub(super) fn signal_clone_exit_notification(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        let (mm, marker, user) = self.start_signal_checkpoint(context, b"e")?;
        let parent = user
            .identity()
            .expect("signal parent has no identity")
            .group();
        for phase in [1u32, 3u32].iter().copied() {
            until("clone exit notification checkpoint was not reached", || {
                mm.load_user_u32(marker + 8) == Ok(phase) || user.mm().is_none()
            })?;
            if mm.load_user_u32(marker + 8) != Ok(phase) {
                break;
            }
            let pid = mm
                .read_usize(marker)
                .map_err(|_| TestError("clone checkpoint PID is inaccessible"))?;
            let child = self
                .graph()
                .wait(Some(&parent), pid)
                .map_err(|_| TestError("clone checkpoint child wait failed"))?;
            context.expect_eq(child.status, 0);
            context.diagnostic(format_args!(
                "signal-exit-notification phase={} pid={} child-status={} joined=1",
                phase, pid, child.status
            ));
            mm.store_user_u32(marker + 8, phase + 1)
                .map_err(|_| TestError("clone checkpoint release failed"))?;
        }

        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        self.reap_children(context, event.status != 0)?;
        Ok(())
    }

    /// 测试目标：验证 `rt_sigsuspend` 临时信号掩码生效期间遇到作业控制暂停（`SIGSTOP`）与恢复（`SIGCONT`）时的重启行为及故障返回路径。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 启动模式为 `"u"` 的用户程序，等待其创建的共享地址空间子线程进入 `rt_sigsuspend` 并到达检查点 `1`。
    /// 2. When: 确认子线程临时信号掩码已设为 `0x800` 且处于 `Blocked` 状态后，发送 `SIGSTOP` 促使其进入进程组停止状态并由父进程组观测到停止事件（`status == 19`）；随后通知父线程解除 `0x4300_1000` 处的参数页映射，最后发送 `SIGCONT` 恢复子线程。
    /// 3. Then: 子线程从停止状态恢复后处理挂起状态并校验系统调用返回值，最终整个测试程序以状态码 `0` 退出。
    pub(super) fn signal_suspend_restart_mask(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        let (mm, marker, user) = self.start_signal_checkpoint(context, b"u")?;
        let parent = user
            .identity()
            .expect("suspend parent has no identity")
            .group();
        until("suspend child checkpoint was not reached", || {
            mm.load_user_u32(marker + 8) == Ok(1) || user.mm().is_none()
        })?;
        if user.mm().is_none() {
            let event = self.wait(context)?;
            context.expect_eq(event.status, 0);
            return Ok(());
        }

        // Step 1: 读取子线程 PID，等待其设置 0x800 临时掩码并在 sigsuspend 中进入 Blocked 状态。
        let pid = mm
            .read_usize(marker)
            .map_err(|_| TestError("suspend checkpoint PID is inaccessible"))?;
        let child = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("suspend child user disappeared"))?;
        let signals = child.signals().expect("suspend child has no signals");
        let executor = child
            .executor()
            .ok_or(TestError("suspend child executor disappeared"))?;
        until(
            "sigsuspend did not register its temporary mask and wait",
            || {
                (signals.mask() == 0x800 && sched::test_state(&executor) == State::Blocked)
                    || child.mm().is_none()
            },
        )?;
        if child.mm().is_none() {
            return Err(TestError("suspend child exited before signal checkpoint"));
        }

        // Step 2: 向子线程发送 SIGSTOP，并轮询验证父进程组可观测到 CLD_STOPPED (code=5, status=19) 事件。
        self.graph()
            .send_kernel(pid, crate::task::signal::SIGSTOP)
            .map_err(|_| TestError("suspend child stop request failed"))?;
        let selector = WaitSelector::Pid(
            self.graph()
                .pid_object(pid)
                .ok_or(TestError("suspend child identity disappeared"))?,
        );
        let mut stopped = None;
        let mut wait_error = None;
        until(
            "suspend child did not acknowledge the group stop",
            || match self.graph().try_wait_event(&parent, &selector, 0x0100_0003) {
                Ok(Some(event)) => {
                    stopped = Some(event);
                    true
                }
                Ok(None) => false,
                Err(error) => {
                    wait_error = Some(error);
                    true
                }
            },
        )?;
        if let Some(error) = wait_error {
            context.diagnostic(format_args!("suspend-stop-wait error={:?}", error));
            return Err(TestError("suspend stopped event lookup failed"));
        }
        let stopped = stopped.ok_or(TestError("suspend stopped event is absent"))?;
        context.expect_eq(stopped.code, 5);
        context.expect_eq(stopped.status, 19);

        // Step 3: 放行用户态父线程解除共享参数页 (0x4300_1000) 的映射，确认该虚拟地址已变为 Unmapped。
        mm.store_user_u32(marker + 8, 2)
            .map_err(|_| TestError("suspend parent unmap release failed"))?;
        until(
            "suspend parent did not unmap the shared argument page",
            || mm.load_user_u32(marker + 8) == Ok(3) || user.mm().is_none(),
        )?;
        if mm.load_user_u32(marker + 8) != Ok(3) {
            return Err(TestError("suspend parent exited before unmap checkpoint"));
        }
        context.expect_eq(
            mm.read_u64(0x4300_1000),
            Err(crate::mm::space::VmError::Unmapped),
        );
        context.diagnostic(format_args!(
            "signal-suspend-checkpoint temporary-mask=0x800 stopped={} parent-unmapped=1",
            stopped.status
        ));

        // Step 4: 发送 SIGCONT 恢复子线程执行，等待测试进程完成全部检查并正常退出。
        self.graph()
            .send_kernel(pid, crate::task::signal::SIGCONT)
            .map_err(|_| TestError("suspend child continue request failed"))?;
        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        self.reap_children(context, event.status != 0)?;
        Ok(())
    }
}
