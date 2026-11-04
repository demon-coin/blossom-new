//! 信号控制点保留真实页等待、临时掩码、timer 抢占与跨核发送的原观察协议。

use super::*;
use crate::mm::reclaim::ReclaimAttempt;

impl UserOwner {
    /// 暖入同一磁盘程序的全部页，避免把指令页I/O误作用户数据读等待。
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

    /// 保活canonical页的唯一填充责任，确认用户read实际Blocked后发信号，再从原盘完成页。
    pub(super) fn signal_during_file_wait(&mut self, context: &mut TestContext<'_>) -> TestResult {
        let path = checked(
            context,
            self.path
                .vfs()
                .resolve(self.path.context(), b"/test/data.bin", ResolveMode::Follow),
        )?;
        let inode = checked(context, path.inode())?.clone();
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

    /// 同CPU的普通内核Task只有在用户发表无限循环标记后才确认推进，远核再请求SIGKILL。
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
        self.observer = Some(observer.publish());
        // 所有目标CPU的Ready工作在用户开始前发表，观察者发表的IPI不能代替timer抢占。
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
        ready.wait();
        let advanced = self.observer.take().unwrap().join();
        context.expect_eq(advanced, 1);
        struct RemoteKill {
            graph: Arc<TaskGraph>,
            pid: usize,
            sender: usize,
            result: Option<Result<(), crate::task::signal::SignalError>>,
        }
        /// 原协调核在同步CSD中发送信号，不依赖等待后可能迁核的suite Task位置。
        ///
        /// # Safety
        /// argument须指向同步call返回前固定且独占的RemoteKill；回调不睡眠、不访问用户内存。
        unsafe fn send_remote_kill(argument: *mut ()) {
            // SAFETY: call的同步完成Release/Acquire保活栈参数，调用者返回前不访问这些字段。
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
        // SAFETY: 原协调核在线且不同于用户目标核；当前Task可服务IPI、不持回调所需锁，
        // 栈参数与graph拥有保留至同步call的Acquire完成，回调只发送信号、不睡眠或做uaccess。
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

    /// 两个具体信号回归共用原发表入口，控制字在父初始栈内，不能充当退出完成事实。
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

    /// 正常父资格消费并确认子成员离栈后，才允许用户检查真实零/非零退出信号。
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

    /// 先观测真实临时mask/Blocked和组停止事件，再由共享VM的用户父解除参数页。
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
        self.graph()
            .send_kernel(pid, crate::task::signal::SIGCONT)
            .map_err(|_| TestError("suspend child continue request failed"))?;
        let event = self.wait(context)?;
        context.expect_eq(event.status, 0);
        self.reap_children(context, event.status != 0)?;
        Ok(())
    }
}
