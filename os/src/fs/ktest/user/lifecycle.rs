//! 用户进程与线程生命周期测试模块（涵盖在途块 I/O 取消、多线程 `exec` 接管、`vfork`、亲缘过继、`wait4` 竞争与 `SIGCHLD` 自动回收）。
//!
//! # 模块职责与架构定位
//! 本模块作为 `UserOwner` 测试夹具的扩展实现，驱动磁盘上的 `/test/bin/user-lifecycle` 与 `/test/bin/user-signal`
//! 用户态测试程序，验证进程树（`TaskGraph`）、用户线程（`UserThread`）、地址空间（`Mm`）及运行时资源（`UserRuntime`）
//! 在复杂生命周期转换中的内核状态不变量。
//! 上游由用户态测试套件入口调用，下游通过检查 `TaskGraph` 引用关系、`Mm::user_count` 引用计数、调度器 `State`
//! 以及块设备在途请求快照（`test_snapshot`）核对生命周期收尾。
//!
//! # 核心设计与关键布局
//! ## 用户态共享控制页（`FUTEX_CONTROL = 0x4400_0000`）偏移布局
//! 内核测试夹具与用户态 `user-lifecycle` 程序通过映射在 `0x4400_0000` 的共享匿名页进行检查点同步与结果传递：
//!
//! ```text
//! 偏移量 (Bytes)  宽度     用途说明
//! --------------  -------  ---------------------------------------------------------------
//! +0              usize/4  子线程/子进程 0 的 TID（或 `p` 模式下原父进程组的 TGID）
//! +8              u32      主检查点状态字（用户态写入 1/3/4/100 表示到达阶段，内核轮询确认）
//! +12             u32      内核放行控制字（`lifecycle_go` 写入阶段号，或作为子进程阻塞的 futex 字）
//! +16             u32      子进程/孤儿进程槽位 1 的 TID（`p` 模式）
//! +24             u32      第一个 `wait4` 返回值或 `WNOHANG` 预检返回值
//! +28             u32      第二个 `wait4` 返回值或子进程离栈后的最终 `wait4` 返回值
//! +32             u32      第二个子线程 TID（`p`/`w` 模式）或 `wait4` 退出状态码（`d`/`e`/`a` 模式）
//! +40             u32      工作线程检查点（`w` 模式）或 `SIGCHLD` 信号捕获结果（`d`/`e`/`a` 模式）
//! +64             u32      `CLONE_CHILD_CLEARTID` 绑定的用户态地址（`exec` 时应被清零解除而不误写）
//! +68             u32      同组对等线程阻塞等待的私有 futex 控制字（`n`/`g` 模式）
//! ```
//!
//! ## 非主线程 `execve` 接管状态转换
//! ```text
//! [exec 前：多线程进程组 (TGID = T0)]
//!   root (Leader, TID=T0, Task=K0)  ---\
//!                                       +---> 共享 old_mm (user_count = 2)
//!   child (Peer,  TID=T1, Task=K1)  ---/
//!
//! [child 调用 execve 成功后]
//!   root (Task=K0) 终止并退出内核栈 (State::Exited)，释放 old_mm (user_count = 0)
//!   child (原 UserThread, Task=K1) 继承组主身份 (TID 变为 T0, TGID = T0)，绑定 new_mm (user_count = 1)
//!   旧 TID 对象 (T1) 从 TaskGraph 注销
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块不直接持有自旋锁；所有跨线程同步均通过 `until` 轮询用户态共享页（`load_user_u32`）、
//!   `futex::wake` 以及 `TaskGraph` 内部方法完成。
//! - 在 `cancel_inflight_file_read` 中，调用 `disk.test_suspend_completions()` 挂起块设备完成通知后，
//!   必须在发起 `request_root_exit(9)` 并验证阻塞状态后调用 `disk.test_resume_completions()` 恢复完成通知，
//!   否则等待 I/O 收尾的用户线程无法退出内核栈，导致后续 `wait` 死锁。

use super::*;
use crate::task::futex;

/// 用户态生命周期综合测试程序路径。
const PROGRAM: &[u8] = b"/test/bin/user-lifecycle";

/// 生命周期测试启动或 `execve` 映像切换后，由测试夹具持有的地址空间（`Mm`）与用户线程（`UserThread`）句柄对。
type LifecycleImage = (Arc<Mm>, Arc<task::UserThread>);

impl UserOwner {
    /// 测试用户线程阻塞在块设备在途读请求（in-flight I/O）期间收到进程组致命退出请求（`SIGKILL`）：
    /// 验证线程在底层磁盘 I/O 完成前保持 `State::Blocked` 且不提前释放地址空间（避免 DMA 踩内存），
    /// 待磁盘完成恢复后再唤醒退出并完整回收地址空间与内核栈。
    pub(super) fn cancel_inflight_file_read(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
        // Step 1: 预热信号测试程序并加载读文件模式的进程映像，计算用户栈顶附近的进度标记地址。
        let _data = self.file(context, b"/test/data.bin")?;
        self.warm_signal_program(context)?;
        let image = self.load_named(context, b"/test/bin/user-signal", b"r")?;
        #[cfg(target_arch = "riscv64")]
        let marker = image.context.gpr[2] - 8;
        #[cfg(target_arch = "loongarch64")]
        let marker = image.context.gpr[3] - 8;
        let mm = Arc::clone(&image.mm);
        let disk = self.path.owner.disk()?.clone();
        let legacy = matches!(
            disk.identity(),
            DiskIdentity::Virtio(VirtioIdentity::Mmio { version: 1, .. })
        );
        let first_used = block::tests::used_index(&disk, legacy);
        let before = disk.test_snapshot();

        // Step 2: 暂停块设备的完成中断处理，启动用户进程发起文件读取，轮询直到该线程真正提交块请求并进入阻塞态。
        disk.test_suspend_completions();
        let fs_context = self.path.context().clone();
        self.start_program(context, image, fs_context, true, false, Credentials::ROOT)?;
        let pid = self.running.as_ref().unwrap().pid;
        let user = self
            .graph()
            .lookup_thread(pid)
            .ok_or(TestError("取消时用户读者绑定消失"))?;
        let executor = user
            .executor()
            .ok_or(TestError("取消时用户读者执行者消失"))?;

        until("用户数据读未在真实在途请求上阻塞", || {
            let mut marker_bytes = [0u8; 8];
            let progress = mm.copyin(&mut marker_bytes, marker);
            progress.error.is_none()
                && u64::from_le_bytes(marker_bytes) == 0x51a2
                && sched::test_state(&executor) == State::Blocked
                && disk.test_snapshot().inflight != 0
                && block::tests::used_index(&disk, legacy) != first_used
        })?;
        // Step 3: 向进程组发送 SIGKILL (9) 退出请求，验证在块请求未完成期间线程仍保持 Blocked 且 Mm 引用计数仍为 1。
        let inflight = disk.test_snapshot().inflight;
        context.expect_eq(before.submitted < disk.test_snapshot().submitted, true);
        context.expect_eq(mm.user_count(), 1);
        self.graph().request_root_exit(9);
        context.expect_eq(sched::test_state(&executor), State::Blocked);
        context.expect_eq(mm.user_count(), 1);
        context.expect_eq(disk.test_snapshot().inflight, inflight);

        // Step 4: 恢复块设备完成通知，等待用户进程以信号 9 状态退出并回收内核栈，验证 Mm 引用计数与在途 I/O 全部归零。
        disk.test_resume_completions();
        let event = self.wait(context)?;
        context.expect_eq(event.status, 9);
        self.reap_children(context, true)?;

        context.expect_eq(mm.user_count(), 0);
        disk.test_flush_completions();
        context.expect_eq(disk.test_snapshot().inflight, 0);
        context.diagnostic(format_args!(
            "user-cancel inflight-at-request={} real-used=1 blocked-mm-users=1 status={} joined=1 final-mm-users=0 final-inflight=0",
            inflight, event.status
        ));
        Ok(())
    }

    /// 启动指定模式参数（`mode`）的 `user-lifecycle` 测试进程，并返回其初始地址空间（`Mm`）与根用户线程句柄。
    /// 当模式为非主线程 `exec`（`n` 或 `c`）时，清除 `expected_task` 固定绑定以允许非主线程接管进程组主身份。
    fn start_lifecycle(
        &mut self,
        context: &TestContext<'_>,
        mode: &[u8],
    ) -> Result<LifecycleImage, TestError> {
        let image = self.load_named(context, PROGRAM, mode)?;
        let mm = Arc::clone(&image.mm);
        self.start_program(
            context,
            image,
            self.path.context().clone(),
            false,
            false,
            Credentials::ROOT,
        )?;
        let running = self.running.as_mut().unwrap();
        if matches!(mode, b"n" | b"c") {
            running.expected_task = None;
        }
        let root = self
            .graph()
            .lookup_thread(self.running.as_ref().unwrap().pid)
            .ok_or(TestError("生命周期根执行者已退出"))?;
        Ok((mm, root))
    }

    /// 轮询等待用户态进程在共享控制页 `FUTEX_CONTROL + offset` 处写入预期的检查点值 `value`；若进程在此之前释放了地址空间则报错返回。
    fn lifecycle_checkpoint(
        mm: &Mm,
        user: &task::UserThread,
        offset: usize,
        value: u32,
    ) -> TestResult {
        until("生命周期程序未到检查点", || {
            mm.load_user_u32(FUTEX_CONTROL + offset) == Ok(value) || user.mm().is_none()
        })?;
        if mm.load_user_u32(FUTEX_CONTROL + offset) != Ok(value) {
            return Err(TestError("生命周期程序提前退出"));
        }
        Ok(())
    }

    /// 向用户态共享控制页的放行字（`FUTEX_CONTROL + 12`）写入指定阶段值 `value`，通知用户态测试程序进入下一阶段。
    fn lifecycle_go(mm: &Mm, value: u32) -> TestResult {
        mm.store_user_u32(FUTEX_CONTROL + 12, value)
            .map_err(|_| TestError("生命周期控制页不可写"))
    }

    /// 等待候选用户线程列表中的任一线程通过 `execve` 切换到全新的地址空间（与 `old` 不同）并在新共享页写入就绪标记 `100`；
    /// 若所有候选线程均在未完成切换前退出（例如多线程并发 `exec` 被致命信号终止），则返回 `Ok(None)`。
    fn lifecycle_image(
        old: &Arc<Mm>,
        users: &[&Arc<task::UserThread>],
    ) -> Result<Option<LifecycleImage>, TestError> {
        let mut image = None;
        until("exec未提交新映像或真实退出", || {
            let mut alive = false;
            for user in users {
                if let Some(mm) = user.mm() {
                    alive = true;
                    if !Arc::ptr_eq(&mm, old) && mm.load_user_u32(FUTEX_CONTROL + 8) == Ok(100) {
                        image = Some((mm, Arc::clone(*user)));
                        return true;
                    }
                }
            }
            !alive
        })?;
        Ok(image)
    }

    /// 等待根进程退出并校验其退出状态码是否属于调用方预期的 `expected_statuses` 集合；若出现非预期退出码则广播终止信号，最后等待所有子任务彻底退出内核栈。
    fn finish_lifecycle(
        &mut self,
        context: &mut TestContext<'_>,
        mode: &[u8],
        expected_statuses: &[i32],
    ) -> Result<WaitEvent, TestError> {
        let event = self.wait(context)?;
        let cancelling = !expected_statuses.contains(&event.status);
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        context.diagnostic(format_args!(
            "user-lifecycle-result mode={:?} task={} status={} joined=1",
            mode,
            event.task_id.as_usize(),
            event.status
        ));
        Ok(event)
    }

    /// 测试多线程进程组中的 `execve` 接管语义：
    /// - 模式 `n`（确定性非主线程 `exec`）：子线程调用 `execve` 后原地接管进程组主（Leader）的 TID/TGID 身份与执行任务（`Task`），原主线程终止并退出内核栈，旧地址空间引用归零。
    /// - 模式 `c`（多线程竞争 `exec`）：验证最终要么由唯一胜出线程切换至新映像并接管组主身份，要么因致命信号正常终止并释放全部地址空间。
    pub(super) fn lifecycle_exec(&mut self, context: &mut TestContext<'_>) -> TestResult {
        for mode in [b"n" as &[u8], b"c"] {
            // Step 1: 启动指定模式的生命周期进程并等待子线程创建就绪，记录切换前的身份、执行任务与运行时资源句柄。
            let (old_mm, root) = self.start_lifecycle(context, mode)?;
            Self::lifecycle_checkpoint(&old_mm, &root, 8, 1)?;
            let child = self.futex_child(&old_mm, 0)?;
            let root_identity = root.identity().ok_or(TestError("exec根身份已解除"))?;
            let child_identity = child.identity().ok_or(TestError("exec线程身份已解除"))?;
            let old_tid = child_identity.tid_object();
            let group = root_identity.group();
            let child_task = child.executor().ok_or(TestError("exec线程执行者已解除"))?;
            let root_task = root.executor().ok_or(TestError("exec根执行者已解除"))?;
            let root_runtime = root.runtime().ok_or(TestError("exec根资源已解除"))?;
            let child_runtime = child.runtime().ok_or(TestError("exec线程资源已解除"))?;
            context.expect_eq(old_mm.user_count(), 2);
            if mode == b"n" {
                Self::futex_waiting(&old_mm, &root, FUTEX_CONTROL + 68, true, u32::MAX)?;
            }

            // Step 2: 放行用户态执行 execve，等待新映像提交或进程组因竞争退出。
            Self::lifecycle_go(&old_mm, 1)?;
            let image = Self::lifecycle_image(&old_mm, &[&root, &child])?;
            if let Some((new_mm, winner)) = image {
                // Step 3a: 若成功切换至新映像，核对胜出线程已接管 TGID 身份、复用原文件表与信号处理表、释放旧 Mm 并注销旧子线程 TID。
                let identity = winner.identity().ok_or(TestError("新映像缺少原身份"))?;
                let executor = winner.executor().ok_or(TestError("新映像缺少原执行者"))?;
                let runtime = winner.runtime().ok_or(TestError("新映像缺少运行资源"))?;
                let original = if Arc::ptr_eq(&winner, &child) {
                    &child_runtime
                } else {
                    &root_runtime
                };

                context.expect_eq(identity.tid(), group.tgid());
                context.expect_eq(identity.tgid(), group.tgid());
                context.expect_eq(identity.group().pgid(), group.pgid());
                context.expect_eq(identity.group().sid(), group.sid());
                context.expect_eq(Arc::ptr_eq(&identity.group(), &group), true);
                context.expect_eq(Arc::ptr_eq(&runtime.files, &original.files), true);
                context.expect_eq(Arc::ptr_eq(&runtime.sighand, &original.sighand), true);
                context.expect_eq(runtime.files.user_count(), 1);
                context.expect_eq(runtime.sighand.user_count(), 1);
                context.expect_eq(old_mm.user_count(), 0);
                context.expect_eq(new_mm.user_count(), 1);
                context.expect_eq(self.graph().lookup_pid(&old_tid).is_none(), true);
                let live = self
                    .graph()
                    .lookup_thread(group.tgid())
                    .ok_or(TestError("新leader未发表到原组号码"))?;
                context.expect_eq(Arc::ptr_eq(&live, &winner), true);
                if mode == b"n" {
                    context.expect_eq(Arc::ptr_eq(&winner, &child), true);
                    context.expect_eq(executor.id(), child_task.id());
                    context.expect_eq(sched::test_state(&root_task), State::Exited);
                }
                self.running.as_mut().unwrap().expected_task = Some(executor.id());
                context.diagnostic(format_args!(
                    "user-exec-image mode={:?} tid={} actual-task={} old-mm-users=0 new-mm-users=1",
                    mode,
                    identity.tid(),
                    executor.id().as_usize()
                ));

                // Step 4a: 放行新映像运行至结束并验证正常退出码 0。
                Self::lifecycle_go(&new_mm, 100)?;
                let event = self.finish_lifecycle(context, mode, &[0])?;
                context.expect_eq(event.status, 0);
            } else {
                // Step 3b: 若在竞争模式 `c` 下被信号终止，验证退出状态码为预期的致命信号（9 或 11）。
                let expected_statuses: &[i32] = if mode == b"c" { &[9, 11] } else { &[] };
                let event = self.finish_lifecycle(context, mode, expected_statuses)?;
                context.expect_eq(mode == b"c" && matches!(event.status, 9 | 11), true);
                context.diagnostic(format_args!("user-exec-race fatal-status={}", event.status));
            }
            context.expect_eq(old_mm.user_count(), 0);
        }
        Ok(())
    }

    /// 测试存在外部进程（`CLONE_FILES | CLONE_SIGHAND` 跨进程组共享者）时调用 `execve` 的资源解耦行为：
    /// 验证执行 `execve` 的进程为自身克隆私有的文件描述符表（`files`）与信号处理表（`sighand`），清除 `clear_child_tid` 绑定，
    /// 同时保持外部共享者持有的原表与旧地址空间不受影响。
    pub(super) fn lifecycle_resources(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 启动 `x` 模式进程并等待其创建共享 files/sighand/fs 与旧 Mm 的子进程，核对各项资源的初始共享引用计数均为 2。
        let (old_mm, root) = self.start_lifecycle(context, b"x")?;
        Self::lifecycle_checkpoint(&old_mm, &root, 8, 1)?;
        let child = self.futex_child(&old_mm, 0)?;
        let root_runtime = root.runtime().ok_or(TestError("共享exec根资源已解除"))?;
        let before = child.runtime().ok_or(TestError("共享exec子资源已解除"))?;
        context.expect_eq(Arc::ptr_eq(&root_runtime.files, &before.files), true);
        context.expect_eq(Arc::ptr_eq(&root_runtime.sighand, &before.sighand), true);
        context.expect_eq(Arc::ptr_eq(&root_runtime.fs, &before.fs), true);
        context.expect_eq(before.files.user_count(), 2);
        context.expect_eq(before.sighand.user_count(), 2);
        context.expect_eq(old_mm.user_count(), 2);

        // Step 2: 放行子进程执行 execve 切换至新映像，验证子进程已换用独立的 files/sighand/Mm，而根进程仍持有原资源且引用计数降为 1。
        Self::lifecycle_go(&old_mm, 1)?;
        let (new_mm, _) =
            Self::lifecycle_image(&old_mm, &[&child])?.ok_or(TestError("共享exec未提交新映像"))?;
        let after = child.runtime().ok_or(TestError("共享exec新资源已解除"))?;
        let unchanged = root.runtime().ok_or(TestError("外部共享者提前退出"))?;
        context.expect_eq(Arc::ptr_eq(&after.files, &before.files), false);
        context.expect_eq(Arc::ptr_eq(&after.sighand, &before.sighand), false);
        context.expect_eq(Arc::ptr_eq(&after.fs, &before.fs), true);
        context.expect_eq(Arc::ptr_eq(&unchanged.files, &before.files), true);
        context.expect_eq(Arc::ptr_eq(&unchanged.sighand, &before.sighand), true);
        context.expect_eq(before.files.user_count(), 1);
        context.expect_eq(before.sighand.user_count(), 1);
        context.expect_eq(after.files.user_count(), 1);
        context.expect_eq(after.sighand.user_count(), 1);
        context.expect_eq(old_mm.user_count(), 1);
        context.expect_eq(new_mm.user_count(), 1);
        context.expect_eq(old_mm.load_user_u32(FUTEX_CONTROL + 64), Ok(0));
        context.diagnostic(format_args!(
            "user-exec-shared private-files=1 private-sighand=1 old-mm-users=1 clear-tid=0"
        ));

        // Step 3: 放行新映像退出并回收整棵进程树，验证新旧地址空间引用计数最终全部归零。
        Self::lifecycle_go(&new_mm, 100)?;
        let event = self.finish_lifecycle(context, b"x", &[0])?;
        context.expect_eq(event.status, 0);
        context.expect_eq(old_mm.user_count(), 0);
        context.expect_eq(new_mm.user_count(), 0);
        Ok(())
    }

    /// 测试 `vfork` 父子进程的地址空间借用与同步语义：
    /// - 模式 `z`：子进程直接调用 `_exit` 时唤醒父进程。
    /// - 模式 `v`：子进程阻塞期间父进程保持 `State::Blocked`，子进程完成 `execve` 或退出释放旧 `Mm` 时唤醒父进程。
    /// - 模式 `k`：父进程在等待 `vfork` 完成期间被 `SIGKILL` 杀死后立即退出并释放自身对旧 `Mm` 的引用，子进程过继给根进程（PID 0）并继续独立完成执行。
    pub(super) fn lifecycle_vfork(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 运行 `z` 模式验证基础 vfork + _exit 唤醒流程。
        let event = self.run_named(context, PROGRAM, b"z", false, 0)?;
        context.expect_eq(event.status, 0);

        for mode in [b"v" as &[u8], b"k"] {
            // Step 2: 在 `v`（正常完成）和 `k`（父进程被 SIGKILL 取消）模式下，验证子进程未释放旧 Mm 前父进程处于 Blocked 状态且旧 Mm 引用计数为 2。
            let (old_mm, root) = self.start_lifecycle(context, mode)?;
            Self::lifecycle_checkpoint(&old_mm, &root, 8, 1)?;
            let child = self.futex_child(&old_mm, 0)?;
            let executor = root.executor().ok_or(TestError("vfork父执行者已解除"))?;
            until("vfork父未在完成关系上阻塞", || {
                sched::test_state(&executor) == State::Blocked || root.mm().is_none()
            })?;
            context.expect_eq(sched::test_state(&executor), State::Blocked);
            context.expect_eq(old_mm.user_count(), 2);

            if mode == b"k" {
                // Step 3a (`k` 模式): 向阻塞中的父进程发送 SIGKILL，验证父进程退出内核栈且子进程未受牵连、其父 PID 过继为 0，随后放行子进程独立退出。
                let pid = self.running.as_ref().unwrap().pid;
                self.graph()
                    .send_kernel(pid, crate::task::signal::SIGKILL)
                    .map_err(|_| TestError("vfork父取消信号发送失败"))?;
                let event = self.wait(context)?;
                context.expect_eq(event.status, 9);
                context.expect_eq(sched::test_state(&executor), State::Exited);
                context.expect_eq(root.mm().is_none(), true);
                context.expect_eq(old_mm.user_count(), 1);
                let identity = child.identity().ok_or(TestError("vfork取消误杀子程序"))?;
                context.expect_eq(self.graph().parent_pid(&identity), 0);
                context.diagnostic(format_args!(
                    "user-vfork-cancel parent-joined=1 child-mm-users=1 reparent=0"
                ));
                Self::lifecycle_go(&old_mm, 1)?;
                self.reap_children(context, false)?;
            } else {
                // Step 3b (`v` 模式): 放行子进程完成 vfork 周期，验证父子进程正常退出并归零旧 Mm 引用计数。
                Self::lifecycle_go(&old_mm, 1)?;
                let event = self.finish_lifecycle(context, mode, &[0])?;
                context.expect_eq(event.status, 0);
            }
            context.expect_eq(old_mm.user_count(), 0);
        }
        Ok(())
    }

    /// 测试多线程进程组的主线程（Leader）提前退出后的亲缘关系维持、PID 角色占用与复用，以及 `exit_group` 终止同组全部线程的行为：
    /// - 模式 `p`：主线程通过 `SYS_exit` 单独退出但同组对等线程（`peer`）仍存活时，子进程（`orphan`）的父 PID 仍保持为该组 TGID，且该 TGID/PGID/SID 号码不可被外部复用或提前 `wait` 消费；
    ///   待最后一个组内成员 `peer` 退出后，`orphan` 才正式过继给根进程（PID 0），原 PID 号码释放并在后续创建新进程时分配给全新的 `Pid` 对象。
    /// - 模式 `g`：调用 `exit_group` 时唤醒并终止阻塞在 futex 上的同组对等线程，整组以指定状态码退出。
    pub(super) fn lifecycle_roles(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1 (`p` 模式): 启动进程树，等待原父进程组的主线程（Leader）单独调用 exit 退出，同时保持同组 peer 线程存活。
        let (mm, root) = self.start_lifecycle(context, b"p")?;
        Self::lifecycle_checkpoint(&mm, &root, 8, 1)?;
        let old_number = mm
            .read_usize(FUTEX_CONTROL)
            .map_err(|_| TestError("原父组号码不可读"))?;
        let old_pid = self
            .graph()
            .pid_object(old_number)
            .ok_or(TestError("leader退出提前释放TGID角色"))?;
        let orphan = self.futex_child(&mm, 16)?;
        let peer = self.futex_child(&mm, 32)?;
        let root_identity = root.identity().ok_or(TestError("亲缘根身份已解除"))?;
        let orphan_identity = orphan.identity().ok_or(TestError("孤儿身份已解除"))?;
        let peer_identity = peer.identity().ok_or(TestError("原父peer身份已解除"))?;
        until("原leader尚未正常退出", || {
            self.graph().lookup_thread(old_number).is_none()
        })?;
        // Step 2: 验证只要同组 peer 线程存活，子进程（orphan）的父 PID 仍维持为 old_number，且 PGID/SID 查询有效、父进程对该组的非阻塞 wait 返回 None。
        context.expect_eq(self.graph().parent_pid(&orphan_identity), old_number);
        context.expect_eq(peer_identity.tgid(), old_number);
        context.expect_eq(
            self.graph().get_pgid(&root_identity, old_number),
            Ok(old_number),
        );
        context.expect_eq(
            self.graph().get_sid(&root_identity, old_number),
            Ok(old_number),
        );
        let selector = WaitSelector::Pid(Arc::clone(&old_pid));
        context.expect_eq(
            self.graph()
                .try_wait_event(&root_identity.group(), &selector, 5),
            Ok(None),
        );

        // Step 3: 放行同组最后一个线程 peer 退出，验证 orphan 过继给根进程（parent_pid == 0）且 old_pid 从图中注销，随后回收 orphan。
        Self::lifecycle_go(&mm, 1)?;
        Self::lifecycle_checkpoint(&mm, &root, 8, 3)?;
        context.expect_eq(self.graph().parent_pid(&orphan_identity), 0);
        context.expect_eq(self.graph().lookup_pid(&old_pid).is_none(), true);
        let orphan_pid = orphan_identity.tgid();
        Self::lifecycle_go(&mm, 3)?;
        let event = self
            .graph()
            .wait(None, orphan_pid)
            .map_err(|_| TestError("真实转亲子组回收失败"))?;
        context.expect_eq(event.status, 0);
        context.expect_eq(orphan.mm().is_none(), true);

        // Step 4: 放行用户态新建进程复用 old_number 号码，验证新分配的 Pid 对象与旧对象指针不同但数值相同，且继承根进程的 PGID/SID。
        Self::lifecycle_go(&mm, 4)?;
        Self::lifecycle_checkpoint(&mm, &root, 8, 4)?;
        let replacement = self.futex_child(&mm, 16)?;
        let identity = replacement
            .identity()
            .ok_or(TestError("复用号码身份未发表"))?;
        let new_pid = identity.tid_object();
        context.expect_eq(new_pid.number(), old_number);
        context.expect_eq(Arc::ptr_eq(&old_pid, &new_pid), false);
        context.expect_eq(self.graph().lookup_pid(&old_pid).is_none(), true);
        let looked_up = self
            .graph()
            .lookup_pid(&new_pid)
            .ok_or(TestError("新号码对象未命中新执行者"))?;
        context.expect_eq(Arc::ptr_eq(&looked_up, &replacement), true);
        context.expect_eq(identity.group().pgid(), root_identity.group().pgid());
        context.expect_eq(identity.group().sid(), root_identity.group().sid());
        context.diagnostic(format_args!(
            "user-role-reuse pid={} old-object-matches=0 orphan-joined=1 inherited-root-roles=1",
            old_number
        ));
        Self::lifecycle_go(&mm, 5)?;
        let event = self.finish_lifecycle(context, b"p", &[0])?;
        context.expect_eq(event.status, 0);
        context.expect_eq(mm.user_count(), 0);

        // Step 5 (`g` 模式): 启动包含 futex 阻塞线程的进程组并触发 exit_group(5)，验证阻塞线程被一并终止且地址空间引用归零。
        let (mm, root) = self.start_lifecycle(context, b"g")?;
        Self::lifecycle_checkpoint(&mm, &root, 8, 1)?;
        let peer = self.futex_child(&mm, 0)?;
        Self::futex_waiting(&mm, &peer, FUTEX_CONTROL + 68, true, u32::MAX)?;
        Self::lifecycle_go(&mm, 1)?;
        let event = self.finish_lifecycle(context, b"g", &[5 << 8])?;
        context.expect_eq(event.status, 5 << 8);
        context.expect_eq(peer.mm().is_none(), true);
        context.expect_eq(mm.user_count(), 0);
        Ok(())
    }

    /// 测试同一进程组内的两个线程（`root` 与 `worker`）并发调用 `wait4` 等待同一个子进程（`child`）退出时的唤醒与消费互斥：
    /// 验证两个等待者均登记到 `TaskGraph` 等待队列并进入 `State::Blocked`，子进程被唤醒退出后仅有一个等待者成功消费退出事件（返回 0），另一个返回 `-ECHILD`（`-10`）。
    pub(super) fn lifecycle_wait(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 启动 `w` 模式进程，等待子进程在 futex 上阻塞，并确认主线程与工作线程均已登记到图等待队列且进入 Blocked 状态。
        let (mm, root) = self.start_lifecycle(context, b"w")?;
        Self::lifecycle_checkpoint(&mm, &root, 8, 1)?;
        Self::lifecycle_checkpoint(&mm, &root, 40, 1)?;
        let child = self.futex_child(&mm, 0)?;
        let worker = self.futex_child(&mm, 32)?;
        let child_mm = child.mm().ok_or(TestError("wait计算子Mm已解除"))?;
        let root_task = root.executor().ok_or(TestError("wait根执行者已解除"))?;
        let worker_task = worker.executor().ok_or(TestError("wait线程执行者已解除"))?;
        Self::futex_waiting(&child_mm, &child, FUTEX_CONTROL + 12, false, u32::MAX)?;
        until("两个wait未在真实图队列阻塞", || {
            (self.graph().waiters().test_members() == 2
                && sched::test_state(&root_task) == State::Blocked
                && sched::test_state(&worker_task) == State::Blocked)
                || root.mm().is_none()
        })?;
        context.expect_eq(self.graph().waiters().test_members(), 2);
        context.expect_eq(sched::test_state(&root_task), State::Blocked);
        context.expect_eq(sched::test_state(&worker_task), State::Blocked);
        context.diagnostic(format_args!(
            "user-wait-observers registered=2 blocked=2 child-futex=1"
        ));

        // Step 2: 放行并唤醒子进程使其退出，等待整个进程组收尾。
        Self::lifecycle_go(&mm, 1)?;
        let count = futex::wake(&mm, FUTEX_CONTROL + 12, false, 1, u32::MAX)
            .map_err(|_| TestError("wait计算子真实futex唤醒失败"))?;
        context.expect_eq(count, 1);

        let event = self.finish_lifecycle(context, b"w", &[0])?;
        context.expect_eq(event.status, 0);
        // Step 3: 读取两个等待线程记录的返回值，验证恰好一个成功（0）、一个得到 -ECHILD（-10），且等待队列与 Mm 引用全部清空。
        let first = mm
            .load_user_u32(FUTEX_CONTROL + 24)
            .map_err(|_| TestError("第一wait结果不可读"))? as i32;
        let second = mm
            .load_user_u32(FUTEX_CONTROL + 28)
            .map_err(|_| TestError("第二wait结果不可读"))? as i32;
        context.expect_eq(matches!((first, second), (0, -10) | (-10, 0)), true);
        context.expect_eq(self.graph().waiters().test_members(), 0);
        context.expect_eq(mm.user_count(), 0);
        context.diagnostic(format_args!(
            "user-wait-consumed first={} second={} mm-users=0",
            first, second
        ));
        Ok(())
    }

    /// 等待 `self.lifecycle_child` 中记录的自动回收子进程彻底退出内核栈（`JoinObserver` 就绪），并消费返回其内核任务退出码。
    pub(super) fn lifecycle_child_left_stack(&mut self) -> Result<Option<i32>, TestError> {
        if let Some(observer) = &self.lifecycle_child {
            until("自动回收子尚未离栈", || {
                observer.try_join().is_some()
            })?;
        }
        Ok(self.lifecycle_child.take().map(|observer| observer.join()))
    }

    /// 测试 `SIGCHLD` 默认处理（`d` 模式）、显式忽略 `SIG_IGN`（`e` 模式）与设置 `SA_NOCLDWAIT`（`a` 模式）下的子进程等待与自动回收语义：
    /// 验证子进程在 futex 阻塞存活期间父进程调用 `wait4(WNOHANG)` 均返回 0（不误报 `-ECHILD`）；
    /// 子进程唤醒退出并彻底离开内核栈后，自动回收模式（`e`/`a`）立即释放子进程资源并使后续 `wait4` 返回 `-ECHILD`，而默认模式（`d`）保留僵尸状态供父进程正常回收。
    pub(super) fn lifecycle_sigchld(&mut self, context: &mut TestContext<'_>) -> TestResult {
        for mode in [b"d" as &[u8], b"e", b"a"] {
            // Step 1: 启动指定模式的进程，获取子进程的内核栈退出观察器（JoinObserver），验证子进程阻塞存活期间父进程 WNOHANG 预检结果为 0。
            let (mm, root) = self.start_lifecycle(context, mode)?;
            Self::lifecycle_checkpoint(&mm, &root, 8, 1)?;
            let child = self.futex_child(&mm, 0)?;
            let child_mm = child.mm().ok_or(TestError("自动回收子Mm提前解除"))?;
            let executor = child
                .executor()
                .ok_or(TestError("自动回收子执行者提前解除"))?;
            let task_id = executor.id();
            self.lifecycle_child = Some(executor.join_observer());
            drop(executor);
            Self::futex_waiting(&child_mm, &child, FUTEX_CONTROL + 12, false, u32::MAX)?;
            context.expect_eq(mm.load_user_u32(FUTEX_CONTROL + 24), Ok(0));

            // Step 2: 唤醒子进程使其退出，并通过 JoinObserver 确认子进程已彻底离开内核栈且释放了自身的 Mm 与 Runtime。
            Self::lifecycle_go(&mm, 1)?;
            let count = futex::wake(&mm, FUTEX_CONTROL + 12, false, 1, u32::MAX)
                .map_err(|_| TestError("自动回收子真实futex唤醒失败"))?;
            context.expect_eq(count, 1);
            context.expect_eq(self.lifecycle_child_left_stack()?, Some(23 << 8));
            context.expect_eq(child.mm().is_none(), true);
            context.expect_eq(child.runtime().is_none(), true);
            context.expect_eq(child_mm.user_count(), 0);

            // Step 3: 放行父进程执行收尾 wait4 与信号检查，读取共享页中的测试结果并验证进程组正常退出。
            Self::lifecycle_go(&mm, 2)?;
            Self::lifecycle_checkpoint(&mm, &root, 8, 3)?;
            let wait_result =
                mm.load_user_u32(FUTEX_CONTROL + 28)
                    .map_err(|_| TestError("自动回收用户wait结果不可读"))? as i32;
            let wait_status = mm
                .load_user_u32(FUTEX_CONTROL + 32)
                .map_err(|_| TestError("自动回收用户wait状态不可读"))?;
            let signal_result =
                mm.load_user_u32(FUTEX_CONTROL + 40)
                    .map_err(|_| TestError("自动回收用户信号结果不可读"))? as i32;
            context.diagnostic(format_args!(
                "user-sigchld mode={:?} child-task={} alive-wnohang=0 child-left-stack=1 wait-result={} wait-status={} signal-result={}",
                mode, task_id.as_usize(), wait_result, wait_status, signal_result
            ));
            Self::lifecycle_go(&mm, 3)?;
            let event = self.finish_lifecycle(context, mode, &[0])?;
            context.expect_eq(event.status, 0);
            context.expect_eq(mm.user_count(), 0);
        }
        Ok(())
    }
}
