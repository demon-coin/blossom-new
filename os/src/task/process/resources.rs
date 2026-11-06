//! 线程组资源限额（`rlimit`）与按真实用户（UID）聚合的进程/信号计费。
//!
//! ## 模块职责与架构定位
//! 本模块管理进程组的四类核心资源软硬上限（`RLIMIT_STACK`、`RLIMIT_NPROC`、`RLIMIT_NOFILE`、`RLIMIT_SIGPENDING`），并按真实用户 UID 全局统计活跃线程与排队信号的用量。
//! - **上游调用者**：`process` 模块在准备新进程/线程（`prepare_process`、`prepare_thread`）和信号入队（`charge_signal`）时申请配额；系统调用层通过 [`super::TaskGraph::prlimit`] 读取或更新指定线程组的资源限额；`exec`、虚拟内存与文件表子系统通过 `ThreadGroup::resource_limit` 查询软硬上限。
//! - **下游依赖**：`crate::sync::SpinLock` 保护全局 UID 弱引用索引表与各线程组的限额表；`crate::task::exec::DEFAULT_STACK_BYTES` 提供初始用户栈默认软上限。
//!
//! ## 核心设计与计费生命周期
//! 1. **按 UID 跨图共享的弱索引账户（`UserAccount`）**：
//!    全局表 `ACCOUNTS` 仅保存 `(uid, Weak<UserAccount>)`，同一真实 `uid` 共享同一个原子计数器实例。当该用户下所有 `ThreadGroup`、`ProcessCharge` 与 `SignalCharge` 全部释放后，`UserAccount::drop` 自动将其从全局弱索引表中移除，长期运行不会残留空闲 UID 表项。
//! 2. **基于 RAII 凭据的精确计费与退费**：
//!    仅真实的线程成员与排队信号节点占用配额，诊断持有的 `Arc<Pid>`、进程组 `PGID`、会话 `SID` 以及等待内核栈回收的 `PreparedExecution` 均不占用配额。
//!
//! ```text
//! UserAccount { uid, processes: AtomicU64, queued_signals: AtomicU64 }
//!   ▲                                        ▲
//!   ├─ ProcessCharge (占用 1 个 NPROC 单位)  └─ SignalCharge (占用 1 个 SIGPENDING 单位)
//!        │                                        │
//!        ├─ 普通非组首线程：退出时在锁外立即 Drop 退费
//!        ├─ 线程组首线程（leader）：退出时转入 ProcessEntry::leader_charge，
//!        │   直到父进程 wait 回收僵尸、自动回收（autoreap）或非组首线程 exec 接管时才 Drop 退费
//!        └─ 信号队列节点：出队、忽略或线程退出清理时在业务自旋锁外 Drop 退费
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - **锁层级顺序**：`TaskGraph::slots` ➔ `ThreadGroup::limits`（见 [`super::TaskGraph::prlimit`]）。严禁在持有 `ThreadGroup::limits` 或 `ACCOUNTS` 时反向获取 `TaskGraph::slots`。
//! - **锁外分配与析构**：`UserAccount::get` 在 `ACCOUNTS` 自旋锁外预分配 `Arc<UserAccount>` 与扩容后的 `Vec` 目录；`UserAccount::drop`、`ProcessCharge::drop` 和 `SignalCharge::drop` 均在业务自旋锁外执行，避免持锁期间触发堆释放。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

use super::{ProcessError, TaskGraph, UserIdentity};
use crate::sync::SpinLock;

/// 单个进程允许设置的文件描述符数量硬上限最大值（`1,048,576`，防止无界扩容文件表耗尽内核内存）。
pub(crate) const MAX_NOFILE: u64 = 1_048_576;
/// 新建根进程默认的打开文件描述符数量软硬限额（软限额 `1024`，硬限额 `4096`）。
pub(crate) const DEFAULT_NOFILE: ResourceLimit = ResourceLimit {
    current: 1024,
    maximum: 4096,
};

/// 内核支持的进程级资源限额类别（作为 [`Limits`] 固定数组的下标使用）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub(crate) enum Resource {
    /// 用户主线程栈空间最大字节数（对应 `RLIMIT_STACK`，单位：字节）。
    Stack,
    /// 同一真实用户（UID）允许同时存在的线程/进程总数上限（对应 `RLIMIT_NPROC`，单位：个）。
    Nproc,
    /// 单个文件描述符表允许分配的最大描述符编号上限加一（对应 `RLIMIT_NOFILE`，单位：个）。
    NoFile,
    /// 同一真实用户（UID）允许同时挂起的排队信号节点总数上限（对应 `RLIMIT_SIGPENDING`，单位：个）。
    SigPending,
}

/// 单项资源的软限额（`rlim_cur`）与硬限额（`rlim_max`）配置对。
///
/// 不变量：`current <= maximum`；`u64::MAX`（即 Linux ABI 的 `RLIM_INFINITY`）表示无软件上限。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ResourceLimit {
    /// 当前生效的软限额（`rlim_cur`），内核在资源分配与计费时实际检查此阈值。
    pub(crate) current: u64,
    /// 软限额可调高的天花板硬限额（`rlim_max`）；非特权用户（`uid != 0`）只能单向调低硬限额，特权用户可自由调整。
    pub(crate) maximum: u64,
}

/// 线程组共享的四项资源限额表（由 `ThreadGroup::limits` 自旋锁保护，`fork` 时整表复制给子进程）。
#[derive(Clone, Copy)]
pub(super) struct Limits([ResourceLimit; 4]);

impl Limits {
    /// 构造根进程的默认资源限额表：`Stack` 软限额为 `DEFAULT_STACK_BYTES`（8 MiB）、硬限额无限制，`NoFile` 为 `1024/4096`，`Nproc` 与 `SigPending` 默认无限制（`u64::MAX`）。
    pub(super) fn new() -> Self {
        let infinity = ResourceLimit {
            current: u64::MAX,
            maximum: u64::MAX,
        };
        Self([
            ResourceLimit {
                current: crate::task::exec::DEFAULT_STACK_BYTES,
                maximum: u64::MAX,
            },
            infinity,
            DEFAULT_NOFILE,
            infinity,
        ])
    }

    /// 读取指定资源类别当前的软硬限额副本。
    pub(super) fn get(&self, resource: Resource) -> ResourceLimit {
        self.0[resource as usize]
    }

    /// 将指定资源类别的软硬限额更新为已通过合法性与权限校验的新值。
    pub(super) fn set(&mut self, resource: Resource, value: ResourceLimit) {
        self.0[resource as usize] = value;
    }
}

/// 按真实用户 UID 聚合的运行时资源记账实体。
pub(super) struct UserAccount {
    /// 该记账账户对应的真实用户标识符（UID）。
    uid: u32,
    /// 该用户当前占用的 `NPROC` 计数（存活线程数加上尚未被 `wait` 回收的僵尸组首数）。
    processes: AtomicU64,
    /// 该用户当前已入队且尚未消费或清理的实时/排队信号节点总数。
    queued_signals: AtomicU64,
}

/// 全局真实用户记账弱引用注册表，将活跃 `uid` 映射到对应的 `Weak<UserAccount>`。
static ACCOUNTS: SpinLock<Vec<(u32, Weak<UserAccount>)>> = SpinLock::new(Vec::new());

impl UserAccount {
    /// 查找或创建指定 `uid` 的共享记账账户。
    ///
    /// 在未持有业务自旋锁的可分配上下文中调用；若首次创建或需要扩容全局注册表但堆内存不足，则返回 [`ProcessError::NoMemory`]。
    pub(super) fn get(uid: u32) -> Result<Arc<Self>, ProcessError> {
        // Step 1: 快路径——若全局表中已有存活的同 UID 账户，直接升级强引用返回。
        {
            let accounts = ACCOUNTS.lock();
            if let Some(account) = accounts
                .iter()
                .find(|(id, _)| *id == uid)
                .and_then(|(_, account)| account.upgrade())
            {
                return Ok(account);
            }
        }

        // Step 2: 慢路径——在自旋锁外预分配候选 UserAccount，避免持锁期间进入堆分配器。
        let candidate = Arc::try_new(Self {
            uid,
            processes: AtomicU64::new(0),
            queued_signals: AtomicU64::new(0),
        })
        .map_err(|_| ProcessError::NoMemory)?;

        // Step 3: 加锁复查并发插入；若全局 Vec 容量已满，则在锁外预留新容量后重试交换。
        let mut prepared = Vec::new();
        loop {
            let needed = {
                let mut accounts = ACCOUNTS.lock();
                if let Some((_, weak)) = accounts.iter_mut().find(|(id, _)| *id == uid) {
                    if let Some(account) = weak.upgrade() {
                        return Ok(account);
                    }
                    let retired = core::mem::replace(weak, Arc::downgrade(&candidate));
                    drop(accounts);
                    drop(retired);
                    return Ok(candidate);
                }
                if accounts.len() < accounts.capacity() {
                    accounts.push((uid, Arc::downgrade(&candidate)));
                    return Ok(candidate);
                }
                if accounts.len() < prepared.capacity() {
                    prepared.append(&mut accounts);
                    core::mem::swap(&mut *accounts, &mut prepared);
                    accounts.push((uid, Arc::downgrade(&candidate)));
                    return Ok(candidate);
                }
                accounts.len() + 1
            };
            prepared
                .try_reserve_exact(needed)
                .map_err(|_| ProcessError::NoMemory)?;
        }
    }

    /// 按调用方软限额 `limit` 为单个新线程扣除 1 个 `NPROC` 配额，成功则返回 RAII 计费凭据 [`ProcessCharge`]。
    ///
    /// 特权用户（`uid == 0`）仍如实累加 `processes` 用量，但不受 `limit` 软限额拦截；普通用户达到 `limit` 时返回 [`ProcessError::NoPid`]（对应 `EAGAIN`）。
    pub(super) fn process(self: &Arc<Self>, limit: u64) -> Result<ProcessCharge, ProcessError> {
        charge(
            &self.processes,
            if self.uid == 0 { u64::MAX } else { limit },
        )
        .map_err(|_| ProcessError::NoPid)?;
        Ok(ProcessCharge(Arc::clone(self)))
    }

    /// 按目标组软限额 `limit` 为单个待入队的排队信号扣除 1 个 `SIGPENDING` 配额，超限时返回 `Err(())`。
    pub(super) fn signal(self: &Arc<Self>, limit: u64) -> Result<SignalCharge, ()> {
        charge(&self.queued_signals, limit)?;
        Ok(SignalCharge(Arc::clone(self)))
    }
}

impl Drop for UserAccount {
    /// 当该 UID 的最后一个强引用释放时，断言所有计费均已归零，并从全局 `ACCOUNTS` 注册表中移除对应的弱引用槽位。
    fn drop(&mut self) {
        assert_eq!(self.processes.load(Ordering::Relaxed), 0);
        assert_eq!(self.queued_signals.load(Ordering::Relaxed), 0);

        let (retired, directory) = {
            let mut accounts = ACCOUNTS.lock();
            let index = accounts
                .iter()
                .position(|(_, weak)| core::ptr::eq(weak.as_ptr(), self));
            let retired = index.map(|index| accounts.swap_remove(index));
            let directory = if accounts.is_empty() {
                core::mem::take(&mut *accounts)
            } else {
                Vec::new()
            };
            (retired, directory)
        };

        drop(retired);
        drop(directory);
    }
}

/// 在当前计数值严格小于 `limit` 且不溢出的前提下原子加 1；达到配额上限时返回 `Err(())`。
fn charge(counter: &AtomicU64, limit: u64) -> Result<(), ()> {
    counter
        .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
            (count < limit).then(|| count.checked_add(1)).flatten()
        })
        .map(|_| ())
        .map_err(|_| ())
}

/// 单个线程或待回收僵尸组首占用的 `NPROC` 配额 RAII 凭据，持有所属 [`UserAccount`] 的强引用。
pub(super) struct ProcessCharge(Arc<UserAccount>);
/// 单个已入队信号节点占用的 `SIGPENDING` 配额 RAII 凭据，持有目标 [`UserAccount`] 的强引用。
pub(crate) struct SignalCharge(Arc<UserAccount>);

impl Drop for ProcessCharge {
    /// 释放线程或回收僵尸组首时原子归还 1 个 `NPROC` 用量（要求必须在业务自旋锁外析构）。
    fn drop(&mut self) {
        assert!(self.0.processes.fetch_sub(1, Ordering::AcqRel) != 0);
    }
}

impl Drop for SignalCharge {
    /// 信号队列节点销毁时原子归还 1 个 `SIGPENDING` 用量（要求必须在信号锁与关系锁外析构）。
    fn drop(&mut self) {
        assert!(self.0.queued_signals.fetch_sub(1, Ordering::AcqRel) != 0);
    }
}

impl TaskGraph {
    /// 读取或原子替换目标进程组（`pid == 0` 表示调用者自身所在线程组）的指定资源限额，并返回修改前的旧软硬限额（对应 `prlimit64` 系统调用）。
    ///
    /// 锁获取顺序为先获取 `TaskGraph::slots` 校验调用者与目标均为存活线程组并核对 UID 权限，再获取目标 `ThreadGroup::limits` 完成读写。非特权用户（`uid != 0`）仅允许操作相同 UID 的目标且不得调高硬限额，`NoFile` 硬限额不得超过 [`MAX_NOFILE`]。
    pub(crate) fn prlimit(
        &self,
        caller: &UserIdentity,
        pid: usize,
        resource: Resource,
        new: Option<ResourceLimit>,
    ) -> Result<ResourceLimit, ProcessError> {
        // Step 1: 校验新限额自身的数值合法性（软限额不得高于硬限额，NOFILE 不得超过系统上限）。
        if new.is_some_and(|limit| limit.maximum < limit.current) {
            return Err(ProcessError::InvalidArgument);
        }
        if resource == Resource::NoFile && new.is_some_and(|limit| MAX_NOFILE < limit.maximum) {
            return Err(ProcessError::Permission);
        }

        // Step 2: 在图关系锁下确认调用者与目标线程组均处于存活状态，并检查跨用户操作权限。
        let slots = self.slots.lock();
        let group = self.identity_target(&slots, caller, pid)?;
        self.live_parent(&slots, group)
            .map_err(|_| ProcessError::NoProcess)?;
        if caller.group.uid != 0 && caller.group.uid != group.uid {
            return Err(ProcessError::Permission);
        }

        // Step 3: 按 Graph -> Limits 锁顺序读取旧限额，校验非 root 硬限额单调不增后提交新值。
        let mut limits = group.limits.lock();
        let old = limits.get(resource);
        if let Some(new) = new {
            if caller.group.uid != 0 && old.maximum < new.maximum {
                return Err(ProcessError::Permission);
            }
            limits.set(resource, new);
        }
        Ok(old)
    }
}
