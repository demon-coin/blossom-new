//! 写者优先的可睡眠读写锁（`SleepRwLock`）模块。
//!
//! # 模块职责与架构定位
//! 本模块为普通内核任务提供允许多个读者并发共享访问、或单个写者独占修改数据的可睡眠同步原语。
//! 上游供文件系统目录树、Inode 元数据、网络或进程表等读多写少且临界区内可能阻塞的子系统使用；
//! 下游依赖 `SpinLock` 保护内部读写计数状态机，并依赖 `WaitQueue` 分别管理阻塞中的读者队列与写者队列。
//!
//! # 核心设计与写者防饥饿状态机
//! 为了防止源源不断到达的新读者导致写者无限期饥饿，`SleepRwLock` 维护了 `waiting_writers` 计数器：
//! 一旦有写者进入慢路径等待（`waiting_writers > 0`），后续所有新到达的读者（`try_read` / `read`）都会被拒绝准入，
//! 直到所有已入场的读者全部退出且等待中的写者完成写入。
//!
//! ```text
//!   State 状态迁移与准入规则（由内部 SpinLock<State> 串行化保护）：
//!
//!   ┌───────────────────────────────────────────────────────────────┐
//!   │ 空闲状态: writer == false, readers == 0                       │
//!   └───┬───────────────────────────────────────────┬───────────────┘
//!       │ try_read / read                           │ try_write / write
//!       │ (仅当 waiting_writers == 0 时准入)         │ (置 writer = true)
//!       ▼                                           ▼
//!   ┌───────────────────────────────┐       ┌───────────────────────────────┐
//!   │ 共享读状态:                   │       │ 独占写状态:                   │
//!   │ writer == false, readers > 0  │       │ writer == true, readers == 0  │
//!   │                               │       │                               │
//!   │ • 若新写者调用 write() 阻塞： │       │ • 所有新读者与新写者均阻塞    │
//!   │   waiting_writers += 1        │       │ • WriteGuard::drop() 时：     │
//!   │   关闭后续新读者的准入通道    │       │   writer = false              │
//!   │ • 最后一个 ReadGuard::drop()：│       │   优先唤醒 1 名等待写者，     │
//!   │   优先唤醒 1 名等待写者       │       │   若无等待写者则唤醒全部读者  │
//!   └───────────────────────────────┘       └───────────────────────────────┘
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **内部自旋锁与等待队列的加锁顺序**：
//!    - 准入检查与计数更新在 `self.state`（`SpinLock<State>`）内完成；
//!    - 等待队列的节点登记（`register`）和唤醒广播（`wake_one` / `wake_all`）均在释放 `self.state` **之后**进行，从而避免产生 `state` 自旋锁与 `WaitQueue` 内部自旋锁之间的嵌套。
//! 2. **上下文与非递归限制**：
//!    - 仅允许在可调度的普通任务上下文中调用 `read` / `write` / `try_read` / `try_write`；
//!    - 锁不支持递归获取，也不支持将 [`ReadGuard`] 原地升级为 [`WriteGuard`]；同一任务在持有读锁期间再次请求写锁会导致死锁。

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::pin::pin;

use crate::sched;
use crate::sync::SpinLock;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// 读写锁内部受自旋锁保护的准入状态机计数。
struct State {
    /// 当前正在持有 [`ReadGuard`] 的活跃读者数量。
    readers: usize,
    /// 当前是否存在正在持有 [`WriteGuard`] 的活跃写者。
    writer: bool,
    /// 正在 `write()` 慢路径中排队等待获取写锁的写者数量；非零时禁止新读者准入以防止写者饥饿。
    waiting_writers: usize,
}

/// 允许持锁期间睡眠或跨核调度的写者优先读写锁。
///
/// 支持多个读者同时持有 [`ReadGuard`] 共享只读数据，或单个写者持有 [`WriteGuard`] 独占修改数据。
/// 内部状态转换与数据交接的内存同步由 `state` 自旋锁统一完成。
pub struct SleepRwLock<T> {
    /// 保护活跃读者数、写者占用标志及等待写者数的关中断自旋锁。
    state: SpinLock<State>,
    /// 因存在活跃写者或等待写者而阻塞的读者等待队列。
    readers: WaitQueue,
    /// 因存在活跃读者或其他活跃写者而阻塞的写者等待队列。
    writers: WaitQueue,
    /// 受读写锁保护的底层数据容器。
    value: UnsafeCell<T>,
}

// SAFETY:
// 1. 按值移动 `SleepRwLock<T>` 需要独占所有权，编译器借用检查保证此时没有任何存活的 `ReadGuard`、`WriteGuard`
//    或正在借用 `&self` 的等待队列节点；
// 2. 跨线程移动锁本身只转移内部数据 `T` 的所有权，因此当 `T: Send` 时 `SleepRwLock<T>` 可安全 `Send`。
unsafe impl<T: Send> Send for SleepRwLock<T> {}

// SAFETY:
// 1. 所有准入与释放操作均通过 `self.state` 自旋锁串行化，保证写者（`writer == true`）与任何其他读者或写者严格互斥；
// 2. `T: Send` 允许不同任务先后通过 `WriteGuard` 独占修改数据，`T: Sync` 允许多个任务同时通过 `ReadGuard` 共享读取 `&T`；
// 3. `ReadGuard` 与 `WriteGuard` 均包含 `PhantomData<*mut ()>`（`!Send + !Sync`），无法跨任务转移或共享守卫本身。
unsafe impl<T: Send + Sync> Sync for SleepRwLock<T> {}

impl<T> SleepRwLock<T> {
    /// 创建一个处于空闲状态的睡眠读写锁，支持 `const` 静态初始化。
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

    /// 在可调度的普通任务上下文中尝试非阻塞地获取共享读锁。
    ///
    /// 当且仅当目前没有活跃写者（`!state.writer`）且没有排队等待的写者（`state.waiting_writers == 0`）时，
    /// 递增活跃读者计数并返回 [`ReadGuard`]；否则立即返回 `None`。
    pub fn try_read(&self) -> Option<ReadGuard<'_, T>> {
        sched::assert_switchable();
        let mut state = self.state.lock();
        // 若已有活跃写者或等待中的写者，则拒绝新读者准入，防止写者饥饿。
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

    /// 阻塞当前任务直至获取共享读锁，返回 [`ReadGuard`]。
    ///
    /// 只能在允许调度的普通任务上下文中调用。若当前存在活跃写者或等待写者，
    /// 当前任务会登记到 `readers` 队列并挂起，待写者释放锁并唤醒读者队列后重新检查准入条件。
    pub fn read(&self) -> ReadGuard<'_, T> {
        // Step 1: 快路径——尝试直接获取读锁，无写者竞争时立即返回。
        if let Some(guard) = self.try_read() {
            return guard;
        }

        // Step 2: 慢路径——在读者等待队列上循环登记并等待唤醒。
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `&self` 共享借用保证 `self.readers` 等待队列在整个循环期间有效；
            // 2. `pin!` 将等待节点 `node` 固定在当前栈帧上，保证挂起期间地址稳定；
            // 3. `_registration` 从不被 `mem::forget`，在函数返回或本轮循环结束时必定先于 `node` 和 `waiter` 析构，
            //    将节点安全地从 `self.readers` 链表中移除。
            let _registration = unsafe { self.readers.register(node.as_mut()) };
            // Step 3: 登记后、睡眠前再次检查准入条件，防止在登记前写者恰好释放锁导致丢失唤醒。
            if let Some(guard) = self.try_read() {
                return guard;
            }
            // Step 4: 挂起当前任务让出 CPU，等待最后一个写者或取消等待的写者广播唤醒。
            waiter.park();
            // Step 5: 唤醒后再次尝试获取读锁；若此时又有新写者抢先进入，则析构本轮登记并继续下一轮等待。
            if let Some(guard) = self.try_read() {
                return guard;
            }
        }
    }

    /// 在可调度的普通任务上下文中尝试非阻塞地获取独占写锁。
    ///
    /// 当且仅当目前没有活跃写者、没有活跃读者、且没有已在排队等待的写者时，
    /// 将 `writer` 置为 `true` 并返回 [`WriteGuard`]；否则立即返回 `None`。
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

    /// 阻塞当前任务直至获取独占写锁，返回 [`WriteGuard`]。
    ///
    /// 只能在允许调度的普通任务上下文中调用。进入慢路径时会先通过 [`WriterWait`] 递增 `waiting_writers` 计数
    /// 以阻止后续新读者准入，然后在 `writers` 队列上挂起等待，直到所有活跃读者和前序写者全部退出。
    pub fn write(&self) -> WriteGuard<'_, T> {
        // Step 1: 快路径——尝试直接获取写锁，锁完全空闲时立即返回。
        if let Some(guard) = self.try_write() {
            return guard;
        }

        // Step 2: 慢路径——构造 `WriterWait` 守卫，将 `waiting_writers` 加 1 以关闭新读者准入通道。
        let mut waiting = WriterWait::new(self);
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `&self` 借用与 `pin!` 分别保证了 `self.writers` 队列与栈上 `node` 节点的生命周期和地址稳定；
            // 2. `_registration` 在每轮迭代退出或函数返回时先于 `node` 和 `waiter` 析构，确保节点从队列中安全撤除；
            // 3. `waiting` 声明在循环外部，仅在整次 `write()` 结束或异常展开时析构一次。
            let _registration = unsafe { self.writers.register(node.as_mut()) };
            // Step 3: 登记后、睡眠前尝试将等待资格转化为写锁持有权，防止错过登记前的解锁通知。
            if let Some(guard) = waiting.try_acquire() {
                return guard;
            }
            // Step 4: 挂起当前任务，等待前序持有者释放锁时调用 `wake_waiters` 唤醒。
            waiter.park();
            // Step 5: 唤醒后再次尝试获取写锁。
            if let Some(guard) = waiting.try_acquire() {
                return guard;
            }
        }
    }

    /// 在释放 `state` 自旋锁之后，根据快照中的等待写者数量唤醒对应的等待队列。
    ///
    /// 若仍有写者在排队（`waiting_writers != 0`），优先唤醒一名写者；
    /// 若已无等待写者，则唤醒全部排队中的读者并发进入。
    fn wake_waiters(&self, waiting_writers: usize) {
        if waiting_writers != 0 {
            self.writers.wake_one();
        } else {
            self.readers.wake_all();
        }
    }
}

/// 维护单个写者在 `write()` 慢路径等待期间对 `waiting_writers` 计数的 RAII 守卫。
///
/// 将“占用一份等待写者名额”与循环内每轮重建的栈上 `WaitNode` 解耦：
/// 若成功获取写锁，由 `try_acquire()` 原子地将 `waiting_writers` 减 1 并置 `writer = true`；
/// 若在等待期间发生取消或栈展开，则由 `Drop` 自动归还 `waiting_writers` 计数并补发必要的唤醒。
struct WriterWait<'a, T> {
    /// 指向所属读写锁的引用。
    lock: &'a SleepRwLock<T>,
    /// 标记当前守卫是否仍持有尚未转化为 [`WriteGuard`] 的 `waiting_writers` 计数。
    pending: bool,
}

impl<'a, T> WriterWait<'a, T> {
    /// 在 `state` 自旋锁保护下将 `waiting_writers` 加 1，阻止后续新读者获取读锁。
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

    /// 尝试在同一个 `state` 自旋锁临界区内将等待写者资格转化为独占 [`WriteGuard`]。
    ///
    /// 若当前既无活跃写者也无活跃读者，则递减 `waiting_writers`、置位 `state.writer = true`，
    /// 并将 `self.pending` 设为 `false`（防止 `WriterWait::drop` 重复递减计数）。
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
    /// 当写者等待尚未成功获取锁便退出（如异常展开）时，归还 `waiting_writers` 计数并唤醒受阻的等待者。
    fn drop(&mut self) {
        // 若已通过 `try_acquire` 成功转化为 `WriteGuard`，计数已扣除，直接返回。
        if !self.pending {
            return;
        }

        // 在自旋锁内扣减 `waiting_writers`，并判断是否需要补发唤醒：
        // - 若当前无活跃写者且无活跃读者，唤醒下一名等待写者或全部等待读者；
        // - 若当前有活跃读者但本写者是最后一个退出的等待写者（`waiting_writers == 0`），唤醒此前被挡住的等待读者。
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

/// [`SleepRwLock`] 的共享读锁 RAII 守卫。
///
/// 持有期间允许当前任务睡眠或跨 CPU 核心迁移，但守卫本身绑定于获取它的任务（`!Send + !Sync`）。
/// 析构时递减活跃读者计数，最后一个退出的读者负责唤醒等待中的写者。
pub struct ReadGuard<'a, T> {
    /// 指向所属睡眠读写锁的引用。
    lock: &'a SleepRwLock<T>,
    /// 零开销类型标记，将守卫约束为 `!Send` 和 `!Sync`，禁止跨任务移交或共享该守卫。
    _task: PhantomData<*mut ()>,
}

impl<T> Deref for ReadGuard<'_, T> {
    type Target = T;

    /// 获取受读写锁保护数据的只读共享引用。
    fn deref(&self) -> &T {
        // SAFETY:
        // 1. 构造 `ReadGuard` 时已在 `state` 自旋锁下递增 `state.readers`，且在 `Drop` 前始终保持 `readers > 0`，
        //    从而排除了任何写者将 `state.writer` 置为 `true` 或获取 `&mut T` 的可能；
        // 2. 多个并发读者仅获取共享引用 `&T`，且 `Sync` 实现要求 `T: Sync`；
        // 3. `self.lock` 在借用期 `'a` 内有效，底层数据 `value` 已初始化且存活。
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> Drop for ReadGuard<'_, T> {
    /// 递减活跃读者计数；若当前读者是最后一个退出的读者，则在释放状态自旋锁后唤醒等待队列。
    fn drop(&mut self) {
        // Step 1: 在状态自旋锁内将 `readers` 减 1，并记录归零时的 `waiting_writers` 数量。
        let wake = {
            let mut state = self.lock.state.lock();
            state.readers = state
                .readers
                .checked_sub(1)
                .expect("rwlock reader count underflow");
            (state.readers == 0).then_some(state.waiting_writers)
        };

        // Step 2: 在释放 `state` 自旋锁后，若活跃读者已清零，唤醒等待中的写者（或读者）。
        if let Some(waiting_writers) = wake {
            self.lock.wake_waiters(waiting_writers);
        }
    }
}

/// [`SleepRwLock`] 的独占写锁 RAII 守卫。
///
/// 持有期间保证没有任何其他活跃读者或写者，允许当前任务在临界区内睡眠或跨核迁移，
/// 但守卫本身不可跨任务传递或共享（`!Send + !Sync`）。
pub struct WriteGuard<'a, T> {
    /// 指向所属睡眠读写锁的引用。
    lock: &'a SleepRwLock<T>,
    /// 零开销类型标记，将守卫约束为 `!Send` 和 `!Sync`，禁止跨任务移交或共享该守卫。
    _task: PhantomData<*mut ()>,
}

impl<T> Deref for WriteGuard<'_, T> {
    type Target = T;

    /// 获取受读写锁保护数据的共享引用。
    fn deref(&self) -> &T {
        // SAFETY:
        // 1. `WriteGuard` 仅在 `state.writer == false && state.readers == 0` 时将 `state.writer` 置为 `true` 后创建，
        //    并在本守卫 `Drop` 前排除了任何其他 `ReadGuard` 或 `WriteGuard` 的存在；
        // 2. `_task: PhantomData<*mut ()>` 保证守卫不可跨任务共享，`&self` 借用排除了通过本守卫同时获取 `&mut T`；
        // 3. `self.lock` 在借用期 `'a` 内有效，保证 `value` 内存合法且无可变别名冲突。
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for WriteGuard<'_, T> {
    /// 获取受读写锁保护数据的独占可变引用。
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY:
        // 1. `state.writer == true` 保证了当前全局仅有这一个 `WriteGuard` 存在，无任何其他读者或写者；
        // 2. `&mut self` 独占可变借用排除了通过当前守卫产生的任何其他引用；
        // 3. `self.lock.value` 为私有字段，仅通过守卫访问，满足 `&mut T` 的独占无别名要求。
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T> Drop for WriteGuard<'_, T> {
    /// 清除写者占用标志，并在释放状态自旋锁后优先唤醒等待中的写者（若无等待写者则唤醒全部等待读者）。
    fn drop(&mut self) {
        // Step 1: 在状态自旋锁内将 `writer` 置回 `false`，并读取当前排队的写者数量。
        let waiting_writers = {
            let mut state = self.lock.state.lock();
            state.writer = false;
            state.waiting_writers
        };
        // Step 2: 在状态自旋锁外唤醒等待队列。
        self.lock.wake_waiters(waiting_writers);
    }
}
