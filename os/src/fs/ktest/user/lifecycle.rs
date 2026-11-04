//! 普通磁盘程序完成生命周期操作；夹具只观察真实绑定、登记与离栈，释放普通控制字。

use super::*;
use crate::task::futex;

const PROGRAM: &[u8] = b"/test/bin/user-lifecycle";

/// 生命周期准备和 exec 观察共同保活的实际映像与用户线程。
type LifecycleImage = (Arc<Mm>, Arc<task::UserThread>);

impl UserOwner {
    /// 真实请求完成尚未消费时只请求组退出，恢复原完成后再wait和离栈join。
    pub(super) fn cancel_inflight_file_read(
        &mut self,
        context: &mut TestContext<'_>,
    ) -> TestResult {
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
        let inflight = disk.test_snapshot().inflight;
        context.expect_eq(before.submitted < disk.test_snapshot().submitted, true);
        context.expect_eq(mm.user_count(), 1);
        self.graph().request_root_exit(9);
        context.expect_eq(sched::test_state(&executor), State::Blocked);
        context.expect_eq(mm.user_count(), 1);
        context.expect_eq(disk.test_snapshot().inflight, inflight);
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

    /// 本例仍由原设备夹具拥有整棵用户树，非leader交接不固定组的CPU或执行者。
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

    /// 共享页只作为普通程序检查点，退出事实仍由原MM绑定和wait/join确认。
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

    /// 放行只写真实共享控制字，不替用户执行被验收的syscall。
    fn lifecycle_go(mm: &Mm, value: u32) -> TestResult {
        mm.store_user_u32(FUTEX_CONTROL + 12, value)
            .map_err(|_| TestError("生命周期控制页不可写"))
    }

    /// 新映像必须换到另一真实Mm并自行建立私有页；竞争fatal单独交由wait判定。
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

    /// 只在完整 wait 状态不属于调用者明确接受的结果时取消，预期信号退出仍正常回收离栈。
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

    /// 同一实际UserThread/Task转为组主身份，原leader必须完成离栈，不能另建执行者。
    pub(super) fn lifecycle_exec(&mut self, context: &mut TestContext<'_>) -> TestResult {
        for mode in [b"n" as &[u8], b"c"] {
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
            Self::lifecycle_go(&old_mm, 1)?;
            let image = Self::lifecycle_image(&old_mm, &[&root, &child])?;
            if let Some((new_mm, winner)) = image {
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
                Self::lifecycle_go(&new_mm, 100)?;
                let event = self.finish_lifecycle(context, mode, &[0])?;
                context.expect_eq(event.status, 0);
            } else {
                let expected_statuses: &[i32] = if mode == b"c" { &[9, 11] } else { &[] };
                let event = self.finish_lifecycle(context, mode, expected_statuses)?;
                context.expect_eq(mode == b"c" && matches!(event.status, 9 | 11), true);
                context.diagnostic(format_args!("user-exec-race fatal-status={}", event.status));
            }
            context.expect_eq(old_mm.user_count(), 0);
        }
        Ok(())
    }

    /// 外部共享者保原表、动作和旧Mm，新映像只私有化实际仍共享的资源。
    pub(super) fn lifecycle_resources(&mut self, context: &mut TestContext<'_>) -> TestResult {
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
        Self::lifecycle_go(&new_mm, 100)?;
        let event = self.finish_lifecycle(context, b"x", &[0])?;
        context.expect_eq(event.status, 0);
        context.expect_eq(old_mm.user_count(), 0);
        context.expect_eq(new_mm.user_count(), 0);
        Ok(())
    }

    /// 父在旧Mm释放时恢复而非等待新程序退出；取消后子仍能独立完成与转亲。
    pub(super) fn lifecycle_vfork(&mut self, context: &mut TestContext<'_>) -> TestResult {
        let event = self.run_named(context, PROGRAM, b"z", false, 0)?;
        context.expect_eq(event.status, 0);
        for mode in [b"v" as &[u8], b"k"] {
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
                Self::lifecycle_go(&old_mm, 1)?;
                let event = self.finish_lifecycle(context, mode, &[0])?;
                context.expect_eq(event.status, 0);
            }
            context.expect_eq(old_mm.user_count(), 0);
        }
        Ok(())
    }

    /// leader先退出保同组亲缘，最后成员退出才转亲并释放数值角色。
    pub(super) fn lifecycle_roles(&mut self, context: &mut TestContext<'_>) -> TestResult {
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

    /// 两个真实图等待者观察后竞争同一事件，普通用户程序验证copyout与最终CPU账。
    pub(super) fn lifecycle_wait(&mut self, context: &mut TestContext<'_>) -> TestResult {
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
        Self::lifecycle_go(&mm, 1)?;
        let count = futex::wake(&mm, FUTEX_CONTROL + 12, false, 1, u32::MAX)
            .map_err(|_| TestError("wait计算子真实futex唤醒失败"))?;
        context.expect_eq(count, 1);
        let event = self.finish_lifecycle(context, b"w", &[0])?;
        context.expect_eq(event.status, 0);
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

    /// 独立Join观察保留本例离栈清理责任，自动回收不依赖图中的退出对象。
    pub(super) fn lifecycle_child_left_stack(&mut self) -> Result<Option<i32>, TestError> {
        if let Some(observer) = &self.lifecycle_child {
            until("自动回收子尚未离栈", || {
                observer.try_join().is_some()
            })?;
        }
        Ok(self.lifecycle_child.take().map(|observer| observer.join()))
    }

    /// 子在原futex阻塞时核WNOHANG，真实离栈后才允许用户区分退出状态与ECHILD。
    pub(super) fn lifecycle_sigchld(&mut self, context: &mut TestContext<'_>) -> TestResult {
        for mode in [b"d" as &[u8], b"e", b"a"] {
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
            Self::lifecycle_go(&mm, 1)?;
            let count = futex::wake(&mm, FUTEX_CONTROL + 12, false, 1, u32::MAX)
                .map_err(|_| TestError("自动回收子真实futex唤醒失败"))?;
            context.expect_eq(count, 1);
            context.expect_eq(self.lifecycle_child_left_stack()?, Some(23 << 8));
            context.expect_eq(child.mm().is_none(), true);
            context.expect_eq(child.runtime().is_none(), true);
            context.expect_eq(child_mm.user_count(), 0);
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
