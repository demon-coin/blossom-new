//! `execve` 线程组收拢与身份交接、线程退出、孤儿过继（Reparenting）及子进程等待（`wait`）。
//!
//! ## 模块职责与架构定位
//! 本模块实现 [`super::TaskGraph`] 中涉及进程/线程消亡与身份演变的核心生命周期协议。
//! - **上游调用者**：`exec` 子系统调用 [`super::TaskGraph::de_thread`] 收拢同组其余线程并接管组首 `TID`，在新映像装载完毕后调用 [`super::TaskGraph::complete_exec`] 释放独占状态；用户线程退出入口调用 [`super::TaskGraph::exit`] 解绑线程、过继子进程并投递 `SIGCHLD`；系统调用 `wait4`/`waitid` 与内核协调器分别通过 `try_wait`、`wait`、`wait_root` 回收子进程。
//! - **下游依赖**：`crate::sched` 与 `crate::wait` 提供阻塞等待队列；`crate::fs::tty` 在会话首进程退出时执行控制终端挂断（`hangup_session`）；`crate::task::signal` 处理信号队列回收、`SA_NOCLDWAIT` 自动回收判定与唤醒；`crate::task::time` 停止进程组定时器并累加已回收子进程的 CPU 时间。
//!
//! ## 核心设计与状态流转
//! 1. **僵尸状态（`ProcessEntry::exit`）与内核栈完成观察（`PreparedExecution`）分离**：
//!    用户线程调用 `exit()` 时仍运行在自己的内核栈上，真正的内核栈释放发生在后续调度器切回 CPU 静态调度栈之后。因此：
//!    - 用户态 `wait4`/`waitid` 或 `SA_NOCLDWAIT` 自动回收只需读取并销毁 `ProcessEntry` 僵尸记录以立即归还 PID 号码，而将尚未确认切离内核栈的 `PreparedExecution` 链表转移到 `Slot::completed` 暂存，不在用户 `wait` 中阻塞等待内核栈释放；
//!    - `de_thread`（多线程 `execve`）与内核协调器（`wait` / `wait_root`）则会显式遍历 `PreparedExecution` 调用 `join()`，确保所有旧线程彻底切离内核栈后再复用组首身份或销毁整个任务图。
//! 2. **多线程 `execve` 身份交接与组退出流转**：
//!
//! ```text
//! [多线程 execve: de_thread -> complete_exec]
//!   调用线程抢占 exec_owner ──► 向同组其余线程发出 exec_exit 请求并唤醒
//!     ──► 在 control_waiters 上睡眠，等待 process.members 降至 1
//!     ──► 锁外逐个 join() 确认其余线程已彻底切离各自内核栈
//!     ──► 若调用者非组首，将其 ThreadEntry 移至 tgid 槽位并接管组首 Pid 对象（重置 exit_signal = SIGCHLD）
//!     ──► 装载新映像后调用 complete_exec() 置位 did_exec = true 并清空 exec_owner
//!
//! [最后一个线程退出: exit()]
//!   槽位锁内摘除 ThreadEntry、归还 TID 角色、累加中断计数（此时暂不写入 process.exit）
//!     ──► 锁外停止组定时器 group.time.stop()，确保异步定时回调彻底执行完毕并退出
//!     ──► 若为拥有控制终端的会话首进程，锁外执行 tty::hangup_session()
//!     ──► 锁外调用 reparent_children() 将全部直属子进程过继给 PID 1（System 模式）或内核根（Kernel 模式）
//!     ──► 重新加锁发布 process.exit，检查当前父进程的 SIGCHLD 处置（若触发 autoreap 则当场回收）
//!     ──► 锁外唤醒父进程信号队列与全局 waiters / control_waiters
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - **锁层级顺序**：`TaskGraph::slots` ➔ `ThreadGroup::relations` / `UserIdentity::tid` ➔ 信号子系统内部锁（`SignalGroup` / `ThreadSignals`）。
//! - **严禁持自旋锁睡眠或等待回调**：`group.time.stop()`、`tty::hangup_session()`、`Waiter::park()`、`JoinObserver::join()` 以及可能释放最后一个强引用的 `drop()` 均必须在释放 `TaskGraph::slots` 自旋锁后执行。
//! - **禁止嵌套 `Waiter`**：同一内核任务同一时刻只能激活一个栈上 `Waiter`。因此 `de_thread`、`wait` 与 `wait_root` 必须先结束在 `WaitQueue` 上的 `Waiter` 作用域，随后才能调用内部同样使用 `Waiter` 的 `execution.join.join()`。

use super::*;

impl TaskGraph {
    /// 在多线程进程执行 `execve` 前终止同组所有其他线程，并在它们彻底切离内核栈后将调用线程切换为线程组组首。
    ///
    /// 必须在普通可睡眠任务上下文中由当前运行线程调用。若同组已有其他线程正在执行 `execve`、或进程组已收到退出请求，则返回 [`ProcessError::Busy`]。
    pub(crate) fn de_thread(&self, identity: &UserIdentity) -> Result<(), ProcessError> {
        sched::assert_switchable();
        let owner = crate::task::current_task_id().ok_or(ProcessError::NoProcess)?;
        // Step 1: 在关系锁内校验调用线程身份并登记 exec_owner，排斥并发的同组 execve 或新线程创建。
        let capacity = {
            let mut slots = self.slots.lock();
            let process = self.exec_process(&mut slots, identity, owner)?;
            if process.exec_owner.is_some() {
                return Err(ProcessError::Busy);
            }
            process.exec_owner = Some(owner);
            slots.len()
        };

        // Step 2: 逐槽扫描同组的其他存活线程，请求它们在内核安全点退出并打断其阻塞状态。
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
        // 唤醒停驻在作业控制停止队列上的同组成员，使其尽快响应 exec_exit 请求并退出。
        self.control_waiters.wake_all();

        // Step 3: 等待同组其余成员全部走完 exit() 将 members 减至 1，随后在无 Waiter 状态下逐个等待它们切离内核栈。
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
                // 此时当前栈上未激活任何 Waiter，可以安全调用内部同样使用 Waiter 的 join() 等待旧线程归还内核栈。
                while let Some(mut execution) = completed.take() {
                    completed = execution.next.take();
                    execution.join.join();
                }
                break;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `&self` 在整个调用期间保持有效，`self.control_waiters` 队列不会被销毁；
            // 2. `node` 通过 `pin!` 固定在当前任务栈帧上，`_registration` 守卫先于 `node` 与 `waiter` 析构并自动摘除链表节点；
            // 3. 注册后立即重新获取 `self.slots` 复查 `process.members == 1`，消除检查与睡眠之间的唤醒丢失窗口。
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

        // Step 4: 重置组首元数据与退出信号为 SIGCHLD；若调用线程原本不是组首，将其槽位迁移至 tgid 并接管组首 Pid 对象。
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

        // Step 5: 在关系锁外释放被替换的旧 TID 引用及旧组首遗留的 NPROC 计费凭据。
        drop(released);
        Ok(())
    }

    /// 在新程序映像装载并绑定到当前任务后，标记进程已完成 `execve`（`did_exec = true`）并解除 `exec_owner` 独占锁。
    ///
    /// 此后父进程调用 `setpgid` 修改该子进程组 ID 将按 POSIX 规范被拒绝（返回 `EACCES`）。
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

    /// 在持有 `self.slots` 锁的前提下，校验 `identity` 属于本图、对应槽位仍绑定当前 `owner` 任务，且进程组未处于退出状态。
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

    /// 处理当前用户线程退出：摘除线程运行登记、归还 `TID` 角色，并在线程组最后一个成员退出时完成定时器停止、会话终端挂断、子进程过继与父进程通知。
    ///
    /// 参数 `status` 为符合 Linux `wait` 编码格式的 16 位退出状态字，`user_interrupts` 与 `user_timer_interrupts` 为本线程累计的用户态中断与时钟中断次数。调用时不能持有任何自旋锁；函数返回后调用方将进入调度器最终切离当前内核栈。
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

        // Step 1: 在关系锁内摘除当前线程槽位，归还非组首 NPROC 配额，累加统计信息并判断是否为组内最后一个成员。
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

        // Step 2: 在自旋锁外释放线程弱引用、解绑的 TID 对象、非组首计费凭据并清理排队信号节点。
        drop(user);
        drop(released);
        drop(charge);
        identity.signals.reclaim_pending();
        group.signals.reclaim_pending();

        if let Some(exit) = exit {
            // Step 3: 若为组内最后一个成员，先在锁外停止组定时器（等待正在执行的定时回调彻底退出）、挂断控制终端并过继所有直属子进程。
            group.time.stop();
            if let Some((_tty, session)) = hangup {
                tty::hangup_session(self, &session);
            }
            self.flush_group_notifications();
            self.reparent_children(&group);

            // Step 4: 重新获取关系锁发布最终退出事件 `process.exit`，并根据此刻父进程的信号处置决定投递 SIGCHLD 或直接自动回收（autoreap）。
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
                // 在锁外执行 timer.stop() 期间原父进程可能也已退出并把本进程过继给新父进程，因此必须重新读取最新的 parent 与 exit_signal。
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
            // 即使不发送 SIGCHLD 或已被 SA_NOCLDWAIT 自动回收，也必须唤醒阻塞在 wait 上的父线程，使其重新扫描子进程并及时返回 ECHILD。
            self.waiters.wake_all();
            if let Some(retired) = retired.as_mut() {
                self.complete_retired_notification(retired);
            }
            // 此处仅销毁已自动回收进程的元数据而不调用 join_members()，因为当前线程正运行在自己的内核栈上，将由调度器在最终切出后回收栈页。
            drop(retired);
        } else if let Some(event) = stopped {
            send_parent_event(parent.as_deref(), event);
            self.waiters.wake_all();
        }

        self.control_waiters.wake_all();
        self.flush_group_notifications();
        drop(group);
    }

    /// 将正在退出的线程组 `group` 名下所有直属子进程过继给系统 `init` 进程（`System` 模式）或转为无父根进程（`Kernel` 模式）。
    ///
    /// 对过继前已处于僵尸态的子进程，向新父进程补发 `SIGCHLD` 并按新父进程的 `SA_NOCLDWAIT` 处置执行自动回收；每次槽位变更独立加锁，析构与唤醒均在锁外执行。
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
                // 若未触发自动回收则原槽位继续持有 child.group，若触发自动回收则所有权转移至 retired，确保不会在持有 slots 自旋锁期间释放最后一个强引用。
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

    /// 非阻塞地尝试读取或回收指定 `pid` 子进程的退出事件（供内核测试及候选回收路径检查退出状态）。
    ///
    /// 当 `nowait` 为 `true` 时仅拷贝退出事件快照而不移除僵尸记录；为 `false` 时回收僵尸记录但将未切离内核栈的观察节点暂存回槽位，不在当前上下文阻塞等待内核栈释放。
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

    /// 在关系锁内校验父子关系并提取指定 `pid` 子进程的退出事件。
    ///
    /// 当 `nowait == false` 时将子进程 CPU 时间并入父进程并调用 `reap_child` 移除僵尸槽位：`kernel_join == true` 时保留 `PreparedExecution` 链表供内核调用方在锁外 `join`；`kernel_join == false` 时将该链表挂回 `Slot::completed` 延迟清理。
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

    /// 阻塞等待指定 `pid` 子进程退出，回收其僵尸槽位并逐一等待其全部成员线程彻底切离内核栈（供内核协调器使用）。
    ///
    /// 先在 `wait_collection` 中完成事件等待并销毁本层 `Waiter`，随后再调用 `join_members()` 等待各成员线程归还内核栈，避免在同一任务上嵌套两个 `Waiter`。
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

    /// 在普通可睡眠上下文中循环等待并回收指定 `pid` 子进程的僵尸记录，返回时当前栈上的 `WaitQueue` 注册节点与 `Waiter` 均已析构。
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
            // SAFETY:
            // 1. `&self` 在整个方法内有效，`self.waiters` 队列生命周期覆盖 `_registration`；
            // 2. `node` 通过 `pin!` 固定在当前栈帧上，`_registration` 在本轮循环末尾或提前 `return` 时先于 `node` 和 `waiter` 析构并摘除节点；
            // 3. 注册完成后立即再次调用 `collect_child` 复查退出状态，防止错过注册前刚刚发生的唤醒通知。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if let Some(collected) = self.collect_child(parent, pid, false, true)? {
                return Ok(collected);
            }
            waiter.park();
        }
    }

    /// 等待并回收任务图中的无父根进程；若根进程已不存在，则收集并等待所有由用户 `wait` 或自动回收遗留在槽位中的线程彻底切离内核栈，最后返回 [`ProcessError::NoChild`]。
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
        // 图内已无任何进程记录，在释放 slots 自旋锁后逐个等待残余线程完成内核栈切换与释放。
        while let Some(mut execution) = completed.take() {
            completed = execution.next.take();
            execution.join.join();
        }
        Err(ProcessError::NoChild)
    }
}
