//! 普通任务的非递归睡眠互斥；数据同步由占有位的 Acquire/Release 完成。

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::sched;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// 可在持有期间阻塞或迁核的普通任务锁，不保证公平性，也不提供优先级继承。
pub struct SleepMutex<T> {
    occupied: AtomicBool,
    waiters: WaitQueue,
    value: UnsafeCell<T>,
}

// SAFETY: 移动独占锁只转移 T 的拥有权；登记中的节点借用锁，禁止同时移动其队列存储。
unsafe impl<T: Send> Send for SleepMutex<T> {}

// SAFETY: 只有成功 Acquire 占有位的 guard 能访问 T，唯一 guard 的 Drop Release 后才允许下一持有者；
// guard 不可 Send/Sync，不能跨任务共享引用；T: Send 允许数据在不同任务间先后转交，队列自行同步链接。
unsafe impl<T: Send> Sync for SleepMutex<T> {}

impl<T> SleepMutex<T> {
    /// 在静态或普通存储中建立未占有的锁，不分配等待节点。
    pub const fn new(value: T) -> Self {
        Self {
            occupied: AtomicBool::new(false),
            waiters: WaitQueue::new(),
            value: UnsafeCell::new(value),
        }
    }

    /// 普通任务在可切换环境中尝试一次占有；竞争失败不创建 guard，也不修改锁的占有状态。
    pub fn try_lock(&self) -> Option<SleepMutexGuard<'_, T>> {
        sched::assert_switchable();
        if self
            .occupied
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return None;
        }
        Some(SleepMutexGuard {
            mutex: self,
            _task: PhantomData,
        })
    }

    /// 普通任务等待独占数据；登记后和每次恢复后重新取锁，通知本身不授予占有权。
    pub fn lock(&self) -> SleepMutexGuard<'_, T> {
        if let Some(guard) = self.try_lock() {
            return guard;
        }

        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 的共享借用保住源存储，pin! 保持栈节点地址稳定；registration 为唯一局部拥有者，
            // 从不被遗忘，所有返回及本轮退出都先析构它真实撤链，再析构节点与 waiter。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            if let Some(guard) = self.try_lock() {
                return guard;
            }
            waiter.park();
            if let Some(guard) = self.try_lock() {
                return guard;
            }
            // 登记先撤销，再由 waiter 结束本轮；未取得资源时下一轮使用新的等待身份。
        }
    }
}

/// 唯一持有者只属于取得它的 Task，不绑定物理 CPU，也不能转交或共享给另一 Task。
pub struct SleepMutexGuard<'a, T> {
    mutex: &'a SleepMutex<T>,
    _task: PhantomData<*mut ()>,
}

impl<T> Deref for SleepMutexGuard<'_, T> {
    type Target = T;

    /// 在原持有 Task 中借用数据，引用不能活过 guard。
    fn deref(&self) -> &T {
        // SAFETY: guard 只在 CAS Acquire 成功后创建，占有位在它析构前保持为 true；
        // guard 不可跨任务共享，&self 借用排除同时通过此 guard 获取可变引用。
        unsafe { &*self.mutex.value.get() }
    }
}

impl<T> DerefMut for SleepMutexGuard<'_, T> {
    /// 在原持有 Task 中独占借用数据，可变引用由 guard 的可变借用限制。
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: 唯一成功 CAS 对应唯一 guard；&mut self 排除它的其他借用，
        // 私有 UnsafeCell 仅由该 guard 访问，锁借用保证数据仍然存活。
        unsafe { &mut *self.mutex.value.get() }
    }
}

impl<T> Drop for SleepMutexGuard<'_, T> {
    /// 原 Task 先 Release 归还占有权，再唤醒一个等待者重新竞争，不跨源锁通知。
    fn drop(&mut self) {
        self.mutex.occupied.store(false, Ordering::Release);
        self.mutex.waiters.wake_one();
    }
}
