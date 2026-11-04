//! 用户实例按号 FIFO，内核 reserve 保留其插入位置；摘除节点由锁外拥有者释放。

use alloc::boxed::Box;
use core::ptr;

use super::{signal_bit, SignalError, SignalInfo, SIGNAL_COUNT, SIGRTMIN};
use crate::task::process::{Resource, SignalCharge, ThreadGroup};

pub(super) struct QueuedSignal {
    info: SignalInfo,
    charge: Option<SignalCharge>,
    next: Option<Box<QueuedSignal>>,
}

pub(crate) struct PreparedSignal {
    node: Option<Box<QueuedSignal>>,
}

impl PreparedSignal {
    /// 内核发送只使用每号内联 reserve，不申请堆节点。
    pub(crate) fn kernel() -> Self {
        Self { node: None }
    }

    /// 用户节点在关系锁和信号锁外准备，计费留到真实入队事务。
    pub(crate) fn user(info: SignalInfo) -> Result<Self, SignalError> {
        let node = Box::try_new(QueuedSignal {
            info,
            charge: None,
            next: None,
        })
        .map_err(|_| SignalError::NoMemory)?;
        Ok(Self { node: Some(node) })
    }
}

struct Queue {
    head: Option<Box<QueuedSignal>>,
    tail: *mut QueuedSignal,
    len: usize,
}

// SAFETY: tail 只指向本队列独占 Box，队列的所有访问均由所属 pending 的信号锁串行化。
unsafe impl Send for Queue {}

impl Queue {
    /// 空队列不拥有任何节点或尾指针。
    fn new() -> Self {
        Self {
            head: None,
            tail: ptr::null_mut(),
            len: 0,
        }
    }

    /// 移入锁外准备节点，尾指针只访问同一队列中仍被 Box 保活的末项。
    fn push(&mut self, mut node: Box<QueuedSignal>) {
        assert!(node.next.is_none());
        let tail = ptr::from_mut(node.as_mut());
        if self.tail.is_null() {
            self.head = Some(node);
        } else {
            // SAFETY: 非空队列 tail 指向本队列末个稳定 Box，持唯一队列可变借用且末项 next 为空。
            unsafe {
                (*self.tail).next = Some(node);
            }
        }
        self.tail = tail;
        self.len += 1;
    }

    /// 移出首节点且断开 next，最后拥有由调用者在短锁外释放。
    fn pop(&mut self) -> Option<Box<QueuedSignal>> {
        let mut node = self.head.take()?;
        self.head = node.next.take();
        self.len -= 1;
        if self.head.is_none() {
            self.tail = ptr::null_mut();
        }
        Some(node)
    }
}

pub(super) struct Removed {
    pub(super) info: SignalInfo,
    node: Option<Box<QueuedSignal>>,
}

pub(super) struct Retired(Option<Box<QueuedSignal>>);

impl Retired {
    /// 空退休链可在取得任何业务锁前构造。
    pub(super) fn new() -> Self {
        Self(None)
    }

    /// 将摘除节点并入退休链，不进行析构或分配。
    pub(super) fn add(&mut self, mut removed: Removed) -> SignalInfo {
        if let Some(mut node) = removed.node.take() {
            node.next = self.0.take();
            self.0 = Some(node);
        }
        removed.info
    }
}

impl Drop for Retired {
    /// 大队列迭代释放，避免递归占用内核栈。
    fn drop(&mut self) {
        while let Some(mut node) = self.0.take() {
            self.0 = node.next.take();
        }
    }
}

pub(super) struct Pending {
    queues: [Queue; SIGNAL_COUNT],
    reserve: [Option<SignalInfo>; SIGNAL_COUNT],
    before_reserve: [usize; SIGNAL_COUNT],
    retired: Retired,
    pub(super) bits: u64,
}

impl Pending {
    /// 号码位及 reserve 内联，用户实例数量不由信号编号数限制。
    pub(super) fn new() -> Self {
        Self {
            queues: core::array::from_fn(|_| Queue::new()),
            reserve: [None; SIGNAL_COUNT],
            before_reserve: [0; SIGNAL_COUNT],
            retired: Retired::new(),
            bits: 0,
        }
    }

    /// 处置、阻塞和退出已在原事务重查后，才按目标用户和本次 soft 限额计费。
    pub(super) fn push(
        &mut self,
        info: SignalInfo,
        prepared: &mut PreparedSignal,
        target: Option<&ThreadGroup>,
    ) -> Result<(), SignalError> {
        let bit = signal_bit(info.signum)?;
        if info.signum < SIGRTMIN && self.bits & bit != 0 {
            return Ok(());
        }
        let index = info.signum as usize - 1;
        if prepared.node.is_some() {
            let target = target.expect("queued signal has no target account");
            let charge =
                target.charge_signal(target.resource_limit(Resource::SigPending).current)?;
            let mut node = prepared.node.take().expect("prepared signal disappeared");
            node.charge = Some(charge);
            self.queues[index].push(node);
        } else if self.reserve[index].is_none() {
            self.reserve[index] = Some(info);
            self.before_reserve[index] = self.queues[index].len;
        }
        self.bits |= bit;
        Ok(())
    }

    /// 最低号码优先，同号 reserve 前的用户节点必须先被领取。
    pub(super) fn take(&mut self, allowed: u64) -> Option<Removed> {
        let bits = self.bits & allowed;
        if bits == 0 {
            return None;
        }
        let index = bits.trailing_zeros() as usize;
        let removed = if self.reserve[index].is_some() && self.before_reserve[index] == 0 {
            Removed {
                info: self.reserve[index].take().unwrap(),
                node: None,
            }
        } else {
            let node = self.queues[index]
                .pop()
                .expect("signal bit has no queued instance");
            if self.reserve[index].is_some() {
                self.before_reserve[index] -= 1;
            }
            Removed {
                info: node.info,
                node: Some(node),
            }
        };
        if self.queues[index].len == 0 && self.reserve[index].is_none() {
            self.bits &= !(1u64 << index);
        }
        Some(removed)
    }

    /// 取消实例只移入退休链，调用者在 Graph 和信号锁外提取并释放。
    pub(super) fn discard(&mut self, mask: u64) {
        while let Some(removed) = self.take(mask) {
            self.retired.add(removed);
        }
    }

    /// 移出完整退休链，锁内不触发节点或账号最后析构。
    pub(super) fn retired(&mut self) -> Retired {
        core::mem::replace(&mut self.retired, Retired::new())
    }
}

impl Drop for Pending {
    /// 未发表候选及整图最终析构也使用迭代节点回收。
    fn drop(&mut self) {
        self.discard(u64::MAX);
    }
}
