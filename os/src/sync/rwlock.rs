//! 普通任务的睡眠读写锁；等待写者关闭新读者准入，数据交接由短状态锁同步。

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::pin::pin;

use crate::sched;
use crate::sync::SpinLock;
use crate::wait::{WaitNode, WaitQueue, Waiter};

struct State {
    readers: usize,
    writer: bool,
    waiting_writers: usize,
}

/// 允许多个读者或唯一写者持有期间睡眠、迁核；非递归，不保证 FIFO 或优先级继承。
pub struct SleepRwLock<T> {
    state: SpinLock<State>,
    readers: WaitQueue,
    writers: WaitQueue,
    value: UnsafeCell<T>,
}

// SAFETY: 独占移动锁只转移 T 的拥有权；活 guard 和登记节点都借用锁，阻止同时移动源存储。
unsafe impl<T: Send> Send for SleepRwLock<T> {}

// SAFETY: 状态锁串行全部准入与归还，写者排除所有读写者；T: Send 允许写侧跨 Task 交接，
// T: Sync 允许不同 Task 同时读 T。队列自行同步，guard 不可转交或共享给其他 Task。
unsafe impl<T: Send + Sync> Sync for SleepRwLock<T> {}

impl<T> SleepRwLock<T> {
    /// 建立未占有的锁，不分配等待节点或等待控制块。
    pub const fn new(value: T) -> Self {
        Self {
            state: SpinLock::new(State {
                readers: 0,
                writer: false,
                waiting_writers: 0,
            }),
            readers: WaitQueue::new(),
            writers: WaitQueue::new(),
            value: UnsafeCell::new(value),
        }
    }

    /// 普通可切换 Task 尝试共享读取；有活写者或等待写者时不增加读者计数。
    pub fn try_read(&self) -> Option<ReadGuard<'_, T>> {
        sched::assert_switchable();
        let mut state = self.state.lock();
        if state.writer || state.waiting_writers != 0 {
            return None;
        }
        state.readers = state
            .readers
            .checked_add(1)
            .expect("rwlock reader count overflow");
        Some(ReadGuard {
            lock: self,
            _task: PhantomData,
        })
    }

    /// 普通 Task 等待共享读取；每轮登记后重查准入，通知不授予读侧拥有权。
    pub fn read(&self) -> ReadGuard<'_, T> {
        if let Some(guard) = self.try_read() {
            return guard;
        }

        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 的借用保住队列，pin! 固定栈节点；唯一 registration 从不被遗忘，
            // 返回或展开均先真实撤链，再释放 node 和 waiter，全程不持状态锁登记。
            let _registration = unsafe { self.readers.register(node.as_mut()) };
            if let Some(guard) = self.try_read() {
                return guard;
            }
            waiter.park();
            if let Some(guard) = self.try_read() {
                return guard;
            }
        }
    }

    /// 普通可切换 Task 尝试独占写入；已有持有者或等待写者时不抢占其准入机会。
    pub fn try_write(&self) -> Option<WriteGuard<'_, T>> {
        sched::assert_switchable();
        let mut state = self.state.lock();
        if state.writer || state.readers != 0 || state.waiting_writers != 0 {
            return None;
        }
        state.writer = true;
        Some(WriteGuard {
            lock: self,
            _task: PhantomData,
        })
    }

    /// 普通 Task 不可中断地等待独占写入；整轮等待只拥有一份阻止新读者的计数责任。
    pub fn write(&self) -> WriteGuard<'_, T> {
        if let Some(guard) = self.try_write() {
            return guard;
        }

        let mut waiting = WriterWait::new(self);
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 的借用和栈 Pin 保住源与节点；唯一 registration 从不被遗忘，
            // 每轮退出先撤链、释放节点和 waiter，最后才析构整轮 waiting 的计数责任。
            let _registration = unsafe { self.writers.register(node.as_mut()) };
            if let Some(guard) = waiting.try_acquire() {
                return guard;
            }
            waiter.park();
            if let Some(guard) = waiting.try_acquire() {
                return guard;
            }
        }
    }

    /// 根据放锁前的等待写者计数通知对应队列；状态可已改变，接收者仍必须重查准入。
    fn wake_waiters(&self, waiting_writers: usize) {
        if waiting_writers != 0 {
            self.writers.wake_one();
        } else {
            self.readers.wake_all();
        }
    }
}

/// 整轮写等待的唯一计数责任，独立于可多次重建的栈等待身份。
struct WriterWait<'a, T> {
    lock: &'a SleepRwLock<T>,
    pending: bool,
}

impl<'a, T> WriterWait<'a, T> {
    /// 先关闭新读者准入；随后登记并重查占有状态，覆盖登记之前发生的释放。
    fn new(lock: &'a SleepRwLock<T>) -> Self {
        let mut state = lock.state.lock();
        state.waiting_writers = state
            .waiting_writers
            .checked_add(1)
            .expect("rwlock waiting writer count overflow");
        Self {
            lock,
            pending: true,
        }
    }

    /// 同一状态锁内把等待责任转成唯一写侧拥有权，成功后析构不再扣等待计数。
    fn try_acquire(&mut self) -> Option<WriteGuard<'a, T>> {
        let mut state = self.lock.state.lock();
        if state.writer || state.readers != 0 {
            return None;
        }
        let remaining = state
            .waiting_writers
            .checked_sub(1)
            .expect("rwlock waiting writer count underflow");
        state.waiting_writers = remaining;
        state.writer = true;
        self.pending = false;
        Some(WriteGuard {
            lock: self.lock,
            _task: PhantomData,
        })
    }
}

impl<T> Drop for WriterWait<'_, T> {
    /// 取消或展开时唯一归还等待计数；没有占有者时续唤写者，最后等待写者退出时恢复读者。
    fn drop(&mut self) {
        if !self.pending {
            return;
        }

        let wake = {
            let mut state = self.lock.state.lock();
            state.waiting_writers = state
                .waiting_writers
                .checked_sub(1)
                .expect("rwlock waiting writer count underflow");
            (!state.writer && (state.readers == 0 || state.waiting_writers == 0))
                .then_some(state.waiting_writers)
        };

        if let Some(waiting_writers) = wake {
            self.lock.wake_waiters(waiting_writers);
        }
    }
}

/// 读侧拥有权只属于取得它的 Task，可随该 Task 迁核但不可转交或共享 guard。
pub struct ReadGuard<'a, T> {
    lock: &'a SleepRwLock<T>,
    _task: PhantomData<*mut ()>,
}

impl<T> Deref for ReadGuard<'_, T> {
    type Target = T;

    /// 在原持有 Task 中借用只读数据，借用不超过 guard 的生命周期。
    fn deref(&self) -> &T {
        // SAFETY: 此 guard 对应未归还的读者计数，所有写者因此被排除；并发跨 Task 读侧只可能
        // 经 T: Sync 的锁共享发生，私有 value 无其他访问入口，guard 借用保住数据存储。
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> Drop for ReadGuard<'_, T> {
    /// 归还本份读者计数，最后读者放开后才在状态锁外通知等待者。
    fn drop(&mut self) {
        let wake = {
            let mut state = self.lock.state.lock();
            state.readers = state
                .readers
                .checked_sub(1)
                .expect("rwlock reader count underflow");
            (state.readers == 0).then_some(state.waiting_writers)
        };

        if let Some(waiting_writers) = wake {
            self.lock.wake_waiters(waiting_writers);
        }
    }
}

/// 唯一写侧拥有权只属于取得它的 Task，持有期间允许睡眠或迁核。
pub struct WriteGuard<'a, T> {
    lock: &'a SleepRwLock<T>,
    _task: PhantomData<*mut ()>,
}

impl<T> Deref for WriteGuard<'_, T> {
    type Target = T;

    /// 在原持有 Task 中借用数据，引用不超过写侧拥有权的生命周期。
    fn deref(&self) -> &T {
        // SAFETY: writer 位仅在没有读写者时置位，并保持到此唯一 guard 析构；guard 不可跨 Task
        // 共享，&self 借用排除同时经该 guard 取得可变引用，锁借用保住数据存储。
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for WriteGuard<'_, T> {
    /// 在原持有 Task 中独占借用数据，可变借用由 guard 的可变借用限制。
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: writer 位排除其他读写 guard；&mut self 排除此 guard 的其他借用，私有 value
        // 只经持有 guard 访问，锁借用保证数据仍有效。
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T> Drop for WriteGuard<'_, T> {
    /// 归还写侧拥有权，优先在状态锁外通知等待写者；无等待写者时恢复全部登记读者。
    fn drop(&mut self) {
        let waiting_writers = {
            let mut state = self.lock.state.lock();
            state.writer = false;
            state.waiting_writers
        };
        self.lock.wake_waiters(waiting_writers);
    }
}
