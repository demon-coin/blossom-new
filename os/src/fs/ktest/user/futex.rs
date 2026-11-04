//! 用户 futex 协调只放行已登记且真正 Blocked 的线程，保留原 WaitId、期限与 Mm 归还观察。

use super::*;
use crate::task::futex::{self, FutexSnapshot};

impl UserOwner {
    /// 用户进度只表示协议位置；真正的等待登记另由桶快照和Task状态确认。
    fn futex_phase(mm: &Mm, root: &task::UserThread, phase: u32) -> TestResult {
        until("futex user did not reach its checkpoint", || {
            mm.load_user_u32(FUTEX_CONTROL + 8) == Ok(phase) || root.mm().is_none()
        })?;
        if mm.load_user_u32(FUTEX_CONTROL + 8) != Ok(phase) {
            return Err(TestError("futex user exited before its checkpoint"));
        }
        Ok(())
    }

    /// 从用户已经发表的真实TID取得原绑定，不建立第二份身份或等待状态。
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

    /// 同时观察实际登记与Blocked，再读取原WaitId/期限；提示字不是睡眠证明。
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

    /// 普通ELF完成实际WAIT/WAKE和waitid；夹具只在真实登记检查点释放用户动作。
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
        match mode[0] {
            b'w' => {
                Self::futex_waiting(&child_mm, &child, address, true, u32::MAX)?;
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex wake checkpoint release failed"))?;
            }
            b'p' | b'n' => {
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
                context.expect_eq(
                    mm.load_user_u32(address),
                    Ok(child.identity().expect("join child has no identity").tid() as u32),
                );
                context.expect_eq(mm.user_count(), 2);
                mm.store_user_u32(FUTEX_CONTROL + 12, 1)
                    .map_err(|_| TestError("futex child exit release failed"))?;
            }
            b's' => {
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
