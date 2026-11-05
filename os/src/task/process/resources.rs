//! 线程组限额和真实用户显式计费；诊断 Arc、PGID 与 SID 不承担资源用量。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

use super::{ProcessError, TaskGraph, UserIdentity};
use crate::sync::SpinLock;

pub(crate) const MAX_NOFILE: u64 = 1_048_576;
pub(crate) const DEFAULT_NOFILE: ResourceLimit = ResourceLimit {
    current: 1024,
    maximum: 4096,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub(crate) enum Resource {
    Stack,
    Nproc,
    NoFile,
    SigPending,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ResourceLimit {
    pub(crate) current: u64,
    pub(crate) maximum: u64,
}

#[derive(Clone, Copy)]
pub(super) struct Limits([ResourceLimit; 4]);

impl Limits {
    /// 教学默认不以当前存储容量限制线程数与排队实例数。
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

    /// 组限额锁内读取同一资源的软硬值。
    pub(super) fn get(&self, resource: Resource) -> ResourceLimit {
        self.0[resource as usize]
    }

    /// 已完成权限与大小检查的替换不影响其他资源。
    pub(super) fn set(&mut self, resource: Resource, value: ResourceLimit) {
        self.0[resource as usize] = value;
    }
}

pub(super) struct UserAccount {
    uid: u32,
    processes: AtomicU64,
    queued_signals: AtomicU64,
}

static ACCOUNTS: SpinLock<Vec<(u32, Weak<UserAccount>)>> = SpinLock::new(Vec::new());

impl UserAccount {
    /// 同一真实 uid 跨图共享计数，候选对象和索引容量均在锁外准备。
    pub(super) fn get(uid: u32) -> Result<Arc<Self>, ProcessError> {
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

        let candidate = Arc::try_new(Self {
            uid,
            processes: AtomicU64::new(0),
            queued_signals: AtomicU64::new(0),
        })
        .map_err(|_| ProcessError::NoMemory)?;

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

    /// root 初始用户仍计实际用量，但不因 NPROC 软限额拒绝创建。
    pub(super) fn process(self: &Arc<Self>, limit: u64) -> Result<ProcessCharge, ProcessError> {
        charge(
            &self.processes,
            if self.uid == 0 { u64::MAX } else { limit },
        )
        .map_err(|_| ProcessError::NoPid)?;
        Ok(ProcessCharge(Arc::clone(self)))
    }

    /// 排队额度按目标真实用户聚合，只有实际入队节点持有该责任。
    pub(super) fn signal(self: &Arc<Self>, limit: u64) -> Result<SignalCharge, ()> {
        charge(&self.queued_signals, limit)?;
        Ok(SignalCharge(Arc::clone(self)))
    }
}

impl Drop for UserAccount {
    /// 最后真实或诊断拥有结束后撤弱索引，空目录也在锁外归还。
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

/// 原子检查及增加用量，有限额度与内存分配失败保持不同错误。
fn charge(counter: &AtomicU64, limit: u64) -> Result<(), ()> {
    counter
        .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
            (count < limit).then(|| count.checked_add(1)).flatten()
        })
        .map(|_| ())
        .map_err(|_| ())
}

pub(super) struct ProcessCharge(Arc<UserAccount>);
pub(crate) struct SignalCharge(Arc<UserAccount>);

impl Drop for ProcessCharge {
    /// 真实线程或待 reap leader 退休时归还，诊断身份不持有本对象。
    fn drop(&mut self) {
        assert!(self.0.processes.fetch_sub(1, Ordering::AcqRel) != 0);
    }
}

impl Drop for SignalCharge {
    /// 节点在所有业务短锁外释放时归还用户排队用量。
    fn drop(&mut self) {
        assert!(self.0.queued_signals.fetch_sub(1, Ordering::AcqRel) != 0);
    }
}

impl TaskGraph {
    /// 在同一活身份事务中读取或替换目标组资源限额，返回提交前的完整旧值。
    /// 锁序为 Graph 后组限额；用户字节复制与既有 Mm 映射不属于此事务。
    pub(crate) fn prlimit(
        &self,
        caller: &UserIdentity,
        pid: usize,
        resource: Resource,
        new: Option<ResourceLimit>,
    ) -> Result<ResourceLimit, ProcessError> {
        if new.is_some_and(|limit| limit.maximum < limit.current) {
            return Err(ProcessError::InvalidArgument);
        }
        if resource == Resource::NoFile && new.is_some_and(|limit| MAX_NOFILE < limit.maximum) {
            return Err(ProcessError::Permission);
        }

        let slots = self.slots.lock();
        let group = self.identity_target(&slots, caller, pid)?;
        self.live_parent(&slots, group)
            .map_err(|_| ProcessError::NoProcess)?;
        if caller.group.uid != 0 && caller.group.uid != group.uid {
            return Err(ProcessError::Permission);
        }

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
