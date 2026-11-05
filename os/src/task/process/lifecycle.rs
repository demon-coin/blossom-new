//! exec 身份交接、线程退出、转亲与 wait；僵尸和离栈观察保留各自独立拥有。

use super::*;

impl TaskGraph {
    /// 当前实际Task独占exec，其他成员在自己的合法点退出并真正离栈后才交接TID。
    pub(crate) fn de_thread(&self, identity: &UserIdentity) -> Result<(), ProcessError> {
        sched::assert_switchable();
        let owner = crate::task::current_task_id().ok_or(ProcessError::NoProcess)?;
        let capacity = {
            let mut slots = self.slots.lock();
            let process = self.exec_process(&mut slots, identity, owner)?;
            if process.exec_owner.is_some() {
                return Err(ProcessError::Busy);
            }
            process.exec_owner = Some(owner);
            slots.len()
        };
        for index in 0..capacity {
            let member = {
                let slots = self.slots.lock();
                slots[index]
                    .thread
                    .as_ref()
                    .filter(|thread| {
                        Arc::ptr_eq(&thread.group, &identity.group)
                            && thread.execution.join.task_id() != owner
                    })
                    .map(|thread| (thread.signals.clone(), thread.user.clone()))
            };
            if let Some((signals, user)) = member {
                signals.request_exec_exit();
                if let Some(user) = user.upgrade() {
                    user.kick();
                }
            }
        }
        // 已停止成员使用图的控制等待源，private通知之外还须唤醒其原停驻登记。
        self.control_waiters.wake_all();
        loop {
            let completed = {
                let mut slots = self.slots.lock();
                let process = self.exec_process(&mut slots, identity, owner)?;
                if process.exec_owner != Some(owner) {
                    return Err(ProcessError::Busy);
                }
                (process.members == 1).then(|| process.completed.take())
            };
            if let Some(mut completed) = completed {
                // 成员已解绑资源；此处没有本栈Waiter，逐个确认真正归还执行栈。
                while let Some(mut execution) = completed.take() {
                    completed = execution.next.take();
                    execution.join.join();
                }
                break;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 图和栈节点保活，登记先于node/waiter析构；登记后重查防止早通知。
            let _registration = unsafe { self.control_waiters.register(node.as_mut()) };
            {
                let mut slots = self.slots.lock();
                let process = self.exec_process(&mut slots, identity, owner)?;
                if process.exec_owner != Some(owner) {
                    return Err(ProcessError::Busy);
                }
                if process.members == 1 {
                    continue;
                }
            }
            waiter.park();
        }
        let released = {
            let mut slots = self.slots.lock();
            let process = self.exec_process(&mut slots, identity, owner)?;
            if process.exec_owner != Some(owner) || process.members != 1 {
                return Err(ProcessError::Busy);
            }
            process.leader_task_id = owner;
            process.leader_status = None;
            process.control_event = None;
            let leader_charge = process.leader_charge.take();
            identity.group.relations.lock().exit_signal = SIGCHLD;
            let mut tid = identity.tid.lock();
            if Arc::ptr_eq(&tid, &identity.group.pid) {
                (None, None, leader_charge)
            } else {
                let old_index = tid.number - 1;
                let new_index = identity.tgid() - 1;
                assert!(
                    slots[new_index].thread.is_none(),
                    "exec leader still has an executing peer"
                );
                slots[new_index].thread = slots[old_index].thread.take();
                attach_role(&mut slots, &identity.group.pid, TID);
                let old = core::mem::replace(&mut *tid, identity.group.pid.clone());
                let released = detach_role(&mut slots, &old, TID);
                (Some(old), released, leader_charge)
            }
        };
        drop(released);
        Ok(())
    }

    /// 新映像已在同一Task发表后释放exec控制，并记录父setpgid不可越过的exec边界。
    pub(crate) fn complete_exec(&self, identity: &UserIdentity) -> Result<(), ProcessError> {
        let owner = crate::task::current_task_id().ok_or(ProcessError::NoProcess)?;
        {
            let mut slots = self.slots.lock();
            let process = self.exec_process(&mut slots, identity, owner)?;
            if process.exec_owner != Some(owner)
                || process.members != 1
                || identity.tid() != identity.tgid()
            {
                return Err(ProcessError::Busy);
            }
            process.did_exec = true;
            process.exec_owner = None;
        }
        self.control_waiters.wake_all();
        Ok(())
    }

    /// 关系锁内核对原组、私有身份和当前实际Task，不把同号复用或组退出视为成功。
    fn exec_process<'a>(
        &self,
        slots: &'a mut Numbers,
        identity: &UserIdentity,
        owner: TaskId,
    ) -> Result<&'a mut ProcessEntry, ProcessError> {
        if !ptr::eq(self, identity.graph.as_ref()) {
            return Err(ProcessError::NoProcess);
        }
        let thread = slots
            .get(
                identity
                    .tid()
                    .checked_sub(1)
                    .ok_or(ProcessError::NoProcess)?,
            )
            .and_then(|slot| slot.thread.as_ref())
            .ok_or(ProcessError::NoProcess)?;
        if !Arc::ptr_eq(&thread.group, &identity.group)
            || !Arc::ptr_eq(&thread.signals, &identity.signals)
            || thread.execution.join.task_id() != owner
        {
            return Err(ProcessError::NoProcess);
        }
        let process = slots
            .get_mut(identity.tgid() - 1)
            .and_then(|slot| slot.process.as_mut())
            .filter(|process| Arc::ptr_eq(&process.group, &identity.group))
            .ok_or(ProcessError::NoProcess)?;
        if process.members == 0
            || process.exit.is_some()
            || identity.group.signals.exit_request().is_some()
            || identity.signals.exec_exit_requested()
        {
            return Err(ProcessError::Busy);
        }
        Ok(process)
    }

    /// 最后成员锁外停稳定时器并转完全部直属子，随后重查父处置发表最终退出事件。
    pub(crate) fn exit(
        &self,
        identity: &UserIdentity,
        status: i32,
        user_interrupts: usize,
        user_timer_interrupts: usize,
    ) {
        assert!(
            ptr::eq(self, identity.graph.as_ref()),
            "identity belongs to another graph"
        );
        let signal = status & WAIT_SIGNAL_MASK;
        assert!(
            status & !WAIT_STATUS_MASK == 0
                && (status & EXIT_STATUS_MASK == 0
                    || (status & (EXIT_STATUS_MASK << EXIT_STATUS_SHIFT) == 0
                        && (1..=SIGNAL_COUNT as i32).contains(&signal))),
            "invalid Linux exit status"
        );
        let (user, group, released, exit, stopped, parent, hangup, charge) = {
            let mut slots = self.slots.lock();
            let old_pgid = identity.group.pgid_object();
            let old_orphan = orphaned_process_group(&slots, &old_pgid);
            let ThreadEntry {
                user,
                group,
                mut execution,
                signals,
                charge,
            } = slots[identity.tid() - 1]
                .thread
                .take()
                .expect("exiting thread is not published");
            assert!(
                Arc::ptr_eq(&group, &identity.group),
                "exiting stale thread identity"
            );
            let process = slots[identity.tgid() - 1]
                .process
                .as_mut()
                .expect("exiting thread has no process");
            assert!(
                Arc::ptr_eq(&process.group, &identity.group),
                "exiting stale group"
            );
            assert!(process.exit.is_none(), "exiting a reaped process");
            let charge = if identity.tid() == identity.tgid() {
                assert!(
                    process.leader_charge.is_none(),
                    "leader NPROC charge already retired"
                );
                process.leader_charge = Some(charge);
                None
            } else {
                Some(charge)
            };
            let task_id = execution.join.task_id();
            if identity.tid() == identity.tgid()
                && !signals.exec_exit_requested()
                && !process.exec_owner.is_some_and(|owner| owner != task_id)
            {
                process.leader_status = Some(status);
            }
            if process.exec_owner == Some(task_id) {
                process.exec_owner = None;
            }
            process.members = process
                .members
                .checked_sub(1)
                .expect("group member underflow");
            process.user_interrupts = process
                .user_interrupts
                .checked_add(user_interrupts)
                .expect("group user interrupt count overflow");
            process.user_timer_interrupts = process
                .user_timer_interrupts
                .checked_add(user_timer_interrupts)
                .expect("group user timer count overflow");
            execution.next = process.completed.take();
            process.completed = Some(execution);
            let stopped = group
                .signals
                .remove_member(&signals)
                .map(|signum| child_event(&group, CLD_STOPPED, signum as i32));
            if let Some(event) = stopped {
                process.control_event = Some(event);
            }
            let last = process.members == 0;
            assert!(
                !last || !group.is_system_init() || self.closing.load(Ordering::Relaxed) != RUNNING,
                "system init exited without shutdown or kernel cancellation"
            );
            let exit = last.then(|| WaitEvent {
                pid: identity.tgid(),
                status: group
                    .signals
                    .exit_request()
                    .or(process.leader_status)
                    .unwrap_or(status),
                user_interrupts: process.user_interrupts,
                user_timer_interrupts: process.user_timer_interrupts,
                task_id: process.leader_task_id,
                cpu: crate::cpu::current().id,
            });
            let parent_number = if last { None } else { process.parent };
            let parent = parent_number
                .and_then(|pid| slots[pid - 1].process.as_ref())
                .map(|parent| parent.group.clone());
            let released = detach_role(&mut slots, &identity.tid_object(), TID);
            let hangup = if last {
                let relations = group.relations.lock();
                let old_sid = relations.sid.clone();
                let hangup = relations
                    .controlling_tty
                    .filter(|_| Arc::ptr_eq(&relations.sid, &group.pid))
                    .map(|tty| (tty, old_sid.clone()));
                drop(relations);
                notify_new_orphan_locked(&mut slots, &old_pgid, old_orphan);
                if !group.is_system_init() {
                    notify_unlinked_children_locked(
                        &mut slots,
                        identity.tgid(),
                        &old_pgid,
                        &old_sid,
                    );
                }
                hangup
            } else {
                None
            };
            (user, group, released, exit, stopped, parent, hangup, charge)
        };
        drop(user);
        drop(released);
        drop(charge);
        identity.signals.reclaim_pending();
        group.signals.reclaim_pending();
        if let Some(exit) = exit {
            // 先在锁外确认定时回调尾部，随后才允许用户wait回收组身份与时间拥有。
            group.time.stop();
            if let Some((_tty, session)) = hangup {
                tty::hangup_session(self, &session);
            }
            self.flush_group_notifications();
            self.reparent_children(&group);
            let (parent, mut retired) = {
                let mut slots = self.slots.lock();
                let process = slots[identity.tgid() - 1]
                    .process
                    .as_mut()
                    .expect("last exiting group disappeared before timer stop");
                assert!(Arc::ptr_eq(&process.group, &group));
                assert_eq!(process.members, 0);
                assert!(
                    process.exec_owner.is_none(),
                    "retiring group has an exec owner"
                );
                assert!(
                    process.exit.is_none(),
                    "retiring group already has an exit event"
                );
                assert!(
                    !group.is_system_init() || self.closing.load(Ordering::Relaxed) != RUNNING,
                    "system init retired without shutdown or kernel cancellation"
                );
                let parent_number = process.parent;
                assert!(
                    !slots
                        .iter()
                        .filter_map(|slot| slot.process.as_ref())
                        .any(|child| child.parent == Some(group.tgid())),
                    "parent exit published before child reparenting"
                );
                let process = slots[group.tgid() - 1]
                    .process
                    .as_mut()
                    .expect("retiring group disappeared");
                process.exit = Some(exit);
                // 父可能在timer.stop期间最后退出并转亲；最终发布只使用此刻的关系与动作表。
                let signum = group.relations.lock().exit_signal;
                let parent = parent_number
                    .filter(|_| signum == SIGCHLD)
                    .and_then(|pid| slots[pid - 1].process.as_ref())
                    .map(|parent| parent.group.clone());
                let autoreap = parent.as_ref().is_some_and(|parent| {
                    parent.signals.deliver_child_exit(
                        child_signal_info(SIGCHLD, exit_child_event(&group, exit.status)),
                        slots
                            .iter()
                            .filter_map(|slot| slot.thread.as_ref())
                            .filter(|thread| Arc::ptr_eq(&thread.group, parent))
                            .map(|thread| &thread.signals),
                    )
                });
                let retired = autoreap.then(|| reap_child(&mut slots, identity.tgid() - 1, false));
                (parent, retired)
            };
            if let Some(parent) = parent {
                parent.signals.notify();
            }
            // 无退出信号或已自动回收仍须唤醒原wait，让其重查事件或返回ECHILD。
            self.waiters.wake_all();
            if let Some(retired) = retired.as_mut() {
                self.complete_retired_notification(retired);
            }
            // 小完成观察只释放不join；scheduler继续持有本Task与栈至真实离栈。
            drop(retired);
        } else if let Some(event) = stopped {
            send_parent_event(parent.as_deref(), event);
            self.waiters.wake_all();
        }
        self.control_waiters.wake_all();
        self.flush_group_notifications();
        drop(group);
    }

    /// 逐槽转亲并递送已有完整僵尸事件，每次关系与处置同锁提交，最后拥有锁外释放。
    fn reparent_children(&self, group: &ThreadGroup) {
        let capacity = self.slots.lock().len();
        for index in 0..capacity {
            let (parent, mut retired) = {
                let mut slots = self.slots.lock();
                let father = slots[group.tgid() - 1]
                    .process
                    .as_ref()
                    .expect("retiring parent disappeared before reparenting");
                assert!(ptr::eq(father.group.as_ref(), group));
                assert_eq!(father.members, 0);
                assert!(father.exit.is_none() && father.exec_owner.is_none());
                if slots[index]
                    .process
                    .as_ref()
                    .is_none_or(|child| child.parent != Some(group.tgid()))
                {
                    continue;
                }
                let parent = if self.root_role == RootRole::System
                    && self.closing.load(Ordering::Relaxed) == RUNNING
                {
                    let root = slots[0]
                        .process
                        .as_ref()
                        .expect("system reaper disappeared");
                    assert!(
                        root.group.is_system_init()
                            && root.group.tgid() == 1
                            && root.parent.is_none()
                            && 0 < root.members
                            && root.exit.is_none(),
                        "system reaper is not a live root"
                    );
                    Some(root.group.clone())
                } else {
                    None
                };
                let child = slots[index].process.as_mut().expect("child disappeared");
                child.parent = parent.as_ref().map(|parent| parent.tgid());
                if self.root_role == RootRole::System {
                    child.group.relations.lock().exit_signal = SIGCHLD;
                }
                let child_group = child.group.clone();
                let event = child.exit;
                let autoreap = match (parent.as_ref(), event) {
                    (Some(parent), Some(event)) => parent.signals.deliver_child_exit(
                        child_signal_info(SIGCHLD, exit_child_event(&child_group, event.status)),
                        slots
                            .iter()
                            .filter_map(|slot| slot.thread.as_ref())
                            .filter(|thread| Arc::ptr_eq(&thread.group, parent))
                            .map(|thread| &thread.signals),
                    ),
                    _ => false,
                };
                // 原槽或移出的回收拥有持续保活child_group，临时引用不会在图锁中成为最后拥有。
                let retired = autoreap.then(|| reap_child(&mut slots, index, false));
                (parent.filter(|_| event.is_some()), retired)
            };
            if let Some(parent) = parent {
                parent.signals.notify();
            }
            if let Some(retired) = retired.as_mut() {
                self.complete_retired_notification(retired);
            }
            drop(retired);
        }
        self.waiters.wake_all();
    }

    /// 用户等待只消费退出信息，不等待成员离栈；nowait保留事件，具体子PID只允许一次消费。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "用户候选准备和 PID 资源测试确认未发表或已回收身份没有退出事件"
        )
    )]
    pub(crate) fn try_wait(
        &self,
        parent: Option<&ThreadGroup>,
        pid: usize,
        nowait: bool,
    ) -> Result<Option<WaitEvent>, ProcessError> {
        self.collect_child(parent, pid, nowait, false)
            .map(|child| child.map(|child| child.event))
    }

    /// 同锁消费亲缘与事件，kernel等待保留离栈链，用户消费把未确认观察转给原槽。
    fn collect_child(
        &self,
        parent: Option<&ThreadGroup>,
        pid: usize,
        nowait: bool,
        kernel_join: bool,
    ) -> Result<Option<CollectedChild>, ProcessError> {
        let mut collected = {
            let mut slots = self.slots.lock();
            let parent_number = match parent {
                Some(parent) => Some(
                    self.live_parent(&slots, parent)
                        .map_err(|_| ProcessError::NoChild)?,
                ),
                None => None,
            };
            let index = pid.checked_sub(1).ok_or(ProcessError::NoChild)?;
            let slot = slots.get(index).ok_or(ProcessError::NoChild)?;
            let process = slot.process.as_ref().ok_or(ProcessError::NoChild)?;
            if process.parent != parent_number {
                return Err(ProcessError::NoChild);
            }
            let Some(event) = process.exit else {
                return Ok(None);
            };
            if nowait {
                return Ok(Some(CollectedChild {
                    event,
                    process: None,
                    notification_parent: None,
                    _released: [None, None, None],
                }));
            }
            if let Some(parent) = parent {
                parent.time.add_reaped_child(&process.group.time);
            }
            reap_child(&mut slots, index, kernel_join)
        };
        self.complete_retired_notification(&mut collected);
        self.waiters.wake_all();
        Ok(Some(collected))
    }

    /// 内核协调者先消费组事件，再在原登记已撤除后确认各成员离栈，不能嵌套两个Waiter。
    pub(crate) fn wait(
        &self,
        parent: Option<&ThreadGroup>,
        pid: usize,
    ) -> Result<WaitEvent, ProcessError> {
        let mut collected = self.wait_collection(parent, pid)?;
        collected
            .process
            .as_mut()
            .expect("wait collected no process")
            .join_members();
        Ok(collected.event)
    }

    /// 登记后复查关闭早通知窗口；返回拥有快照时本栈登记与Waiter已全部结束。
    fn wait_collection(
        &self,
        parent: Option<&ThreadGroup>,
        pid: usize,
    ) -> Result<CollectedChild, ProcessError> {
        sched::assert_switchable();
        loop {
            if let Some(collected) = self.collect_child(parent, pid, false, true)? {
                return Ok(collected);
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self借用保活固定源；node固定当前Task栈，registration先于
            // node/waiter析构，复查关闭早通知窗口，返回前撤链后才可等待成员join。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if let Some(collected) = self.collect_child(parent, pid, false, true)? {
                return Ok(collected);
            }
            waiter.park();
        }
    }

    /// 回收kernel根的真实树；无残余组时仍逐个确认用户wait与autoreap留下的原执行者离栈。
    pub(crate) fn wait_root(&self) -> Result<WaitEvent, ProcessError> {
        let (pid, mut completed) = {
            let mut slots = self.slots.lock();
            let pid = slots
                .iter()
                .filter_map(|slot| slot.process.as_ref())
                .find(|process| process.parent.is_none())
                .map(|process| process.group.tgid());
            let mut completed = None;
            if pid.is_none() {
                assert!(
                    slots.iter().all(|slot| slot.process.is_none()),
                    "process tree has no root"
                );
                for slot in slots.iter_mut() {
                    while let Some(mut execution) = slot.completed.take() {
                        slot.completed = execution.next.take();
                        execution.next = completed.take();
                        completed = Some(execution);
                    }
                }
            }
            (pid, completed)
        };
        if let Some(pid) = pid {
            return self.wait(None, pid);
        }
        // 原图已无已发表组，节点全部移出锁后才能建立join等待登记。
        while let Some(mut execution) = completed.take() {
            completed = execution.next.take();
            execution.join.join();
        }
        Err(ProcessError::NoChild)
    }
}
