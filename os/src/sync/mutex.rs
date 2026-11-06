//! 可睡眠互斥锁（`SleepMutex`）模块。
//!
//! # 模块职责与架构定位
//! 本模块为普通内核任务（Task）提供可在持锁期间主动睡眠、阻塞 I/O 或发生跨核调度的非递归互斥锁。
//! 上游调用者包括虚拟文件系统（VFS）、块设备缓存、进程地址空间等需要在临界区内等待慢速操作的高层子系统；
//! 下游依赖调度器检查接口（`sched::assert_switchable`）与底层等待队列（`wait::WaitQueue`）。
//!
//! # 核心设计与等待状态机
//! `SleepMutex` 采用“原子占用位（`AtomicBool`）+ 栈上等待队列（`WaitQueue`）”的两阶段竞争设计：
//! 唤醒操作（`wake_one`）仅负责将睡眠任务重新放入调度就绪队列，**不直接移交锁的拥有权**；
//! 被唤醒的任务必须重新通过 CAS 竞争 `occupied` 位（Barging / 非公平抢占模型），以消除唤醒与新到达任务之间的护航效应。
//!
//! ```text
//!   Task 调用 lock()
//!         │
//!         ▼
//!   ┌───────────────────────────────┐      CAS(false -> true) 成功
//!   │ 1. 快路径 try_lock()           │ ─────────────────────────────► 返回 SleepMutexGuard
//!   └─────────────┬─────────────────┘                                     │
//!                 │ 已被占用                                              │ 持锁访问 value
//!                 ▼                                                       ▼
//!   ┌───────────────────────────────┐                               Drop 释放锁：
//!   │ 2. 栈上创建 Waiter + WaitNode │                               1. occupied.store(false, Release)
//!   │    登记到 waiters 队列         │                               2. waiters.wake_one()
//!   └─────────────┬─────────────────┘
//!                 │
//!                 ▼
//!   ┌───────────────────────────────┐      CAS 成功（撤链后返回）
//!   │ 3. 睡前二次检查 try_lock()     │ ─────────────────────────────► 返回 SleepMutexGuard
//!   └─────────────┬─────────────────┘
//!                 │ 仍被占用（消除登记前恰好解锁的丢失唤醒窗口）
//!                 ▼
//!   ┌───────────────────────────────┐
//!   │ 4. waiter.park() 让出 CPU     │ ◄── 持有者 Drop 时 wake_one() 唤醒
//!   └─────────────┬─────────────────┘
//!                 │ 恢复运行后再次 try_lock()；若被其他任务抢先占锁，
//!                 └─► 析构本轮登记（从等待队列撤链）并进入下一轮循环
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **禁止在原子/中断上下文中加锁**：`try_lock` 与 `lock` 均要求当前处于可任务切换的普通任务上下文（通过 `sched::assert_switchable()` 检查），严禁在中断处理函数、持有 `SpinLock`、`IrqSaveGuard` 或 `PreemptGuard` 期间调用。
//! 2. **内部锁顺序**：`SleepMutex` 内部仅在操作 `waiters` 等待队列时短暂获取 `WaitQueue` 的内部自旋锁；在持有 `SleepMutexGuard` 期间允许继续获取其他优先级更低的 `SleepMutex` 或 `SpinLock`。
//! 3. **非递归与同任务绑定**：同一任务不可对同一把 `SleepMutex` 重复加锁（否则将阻塞等待自己释放而死锁）；`SleepMutexGuard` 为 `!Send + !Sync`，只能由加锁的同一个任务在同一执行流中释放。

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::sched;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// 允许持锁任务睡眠或跨 CPU 迁移的非递归睡眠互斥锁。
///
/// 通过 `occupied` 原子布尔位的 `Acquire` / `Release` 内存序完成受保护数据 `T` 的跨任务同步，
/// 竞争失败的任务在 `waiters` 等待队列上挂起让出 CPU。不保证先来先服务（FIFO）公平性，也不提供优先级继承。
pub struct SleepMutex<T> {
    /// 锁的占用状态位：`true` 表示当前已有任务持有锁，`false` 表示锁处于空闲状态。
    occupied: AtomicBool,
    /// 等待获取该锁的任务队列，在持锁者释放锁时唤醒队首等待者重新参与竞争。
    waiters: WaitQueue,
    /// 受互斥锁保护的内部数据容器，仅在成功将 `occupied` 从 `false` 置为 `true` 后才允许访问。
    value: UnsafeCell<T>,
}

// SAFETY:
// 1. 按值移动 `SleepMutex<T>` 需要独占所有权，此时编译器借用检查器保证不存在任何存活的 `SleepMutexGuard`
//    或正在借用 `&self` 注册等待节点的任务；
// 2. 跨线程转移锁本身仅涉及内部数据 `T` 的所有权转移，因此只要 `T: Send`，`SleepMutex<T>` 即可安全 `Send`。
unsafe impl<T: Send> Send for SleepMutex<T> {}

// SAFETY:
// 1. 任何时刻最多只有一个任务能通过 `compare_exchange(..., Acquire, ...)` 将 `occupied` 从 `false` 置为 `true` 并构造 `SleepMutexGuard`；
// 2. 持有者释放锁时通过 `store(false, Ordering::Release)` 发布对 `value` 的全部修改，下一任持有者通过 `Acquire` 建立 happens-before 同步关系；
// 3. `SleepMutexGuard` 包含 `PhantomData<*mut ()>`，因此 `!Send + !Sync`，不可能在多个任务之间同时共享同一个守卫；
// 4. 既然不同任务只能先后互斥地独占访问 `T`，只要 `T: Send`（允许跨任务传递数据所有权/可变访问权），`SleepMutex<T>` 即可安全地在多线程间共享引用（`Sync`）。
unsafe impl<T: Send> Sync for SleepMutex<T> {}

impl<T> SleepMutex<T> {
    /// 创建一个处于未加锁状态的睡眠互斥锁，支持 `const` 静态初始化。
    pub const fn new(value: T) -> Self {
        Self {
            occupied: AtomicBool::new(false),
            waiters: WaitQueue::new(),
            value: UnsafeCell::new(value),
        }
    }

    /// 在可调度的普通任务上下文中尝试非阻塞地获取一次锁。
    ///
    /// 若锁当前空闲，则原子地将其标记为已占用并返回 [`SleepMutexGuard`]；
    /// 若锁已被其他任务持有，则立即返回 `None`，不会阻塞当前任务。
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

    /// 阻塞当前任务直至获取互斥锁，返回独占访问数据的 [`SleepMutexGuard`]。
    ///
    /// 只能在允许调度的普通任务上下文中调用。若锁已被占用，当前任务会将自己登记到等待队列并睡眠，
    /// 被唤醒后重新尝试竞争锁，直到成功获取为止。
    pub fn lock(&self) -> SleepMutexGuard<'_, T> {
        // Step 1: 快路径——先尝试一次无阻塞 CAS 获取锁，无竞争时直接返回，避免构造等待节点。
        if let Some(guard) = self.try_lock() {
            return guard;
        }

        // Step 2: 慢路径——循环入队等待并重新竞争，直到成功获取 `occupied` 占用位。
        loop {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `&self` 的共享借用保证了 `self.waiters` 等待队列在整个循环迭代期间有效；
            // 2. `pin!` 将 `node` 固定在当前任务栈帧上，保证其内存地址在挂起期间稳定不变；
            // 3. `_registration` 是栈上的局部变量且从不调用 `mem::forget`，无论是 `return guard` 提前返回
            //    还是本轮循环结束，都会按逆序先析构 `_registration` 将 `node` 从 `self.waiters` 链表中移除，
            //    随后才析构 `node` 与 `waiter`，不会留下悬垂链表节点。
            let _registration = unsafe { self.waiters.register(node.as_mut()) };
            // Step 3: 入队登记后、真正睡眠前再次尝试获取锁，防止持锁者在登记前刚刚释放锁导致丢失唤醒。
            if let Some(guard) = self.try_lock() {
                return guard;
            }
            // Step 4: 挂起当前任务让出 CPU，等待持锁者 `Drop` 时调用 `wake_one()` 唤醒。
            waiter.park();
            // Step 5: 被唤醒后再次竞争锁；若成功则返回，若被其他任务抢占则析构本轮 `_registration` 并进入下一轮等待。
            if let Some(guard) = self.try_lock() {
                return guard;
            }
        }
    }
}

/// [`SleepMutex`] 的 RAII 作用域守卫，代表当前任务对受保护数据的独占访问权。
///
/// 持有该守卫的任务可以在临界区内主动睡眠或被调度器迁移到其他物理 CPU 核心运行，
/// 但该守卫本身不能跨任务传递或共享（`!Send + !Sync`）。析构时自动释放锁并唤醒一名等待者。
pub struct SleepMutexGuard<'a, T> {
    /// 指向所属睡眠互斥锁的引用，生命周期受限于加锁借用期 `'a`。
    mutex: &'a SleepMutex<T>,
    /// 零开销类型标记，将守卫约束为 `!Send` 和 `!Sync`，禁止跨任务移交锁持有权。
    _task: PhantomData<*mut ()>,
}

impl<T> Deref for SleepMutexGuard<'_, T> {
    type Target = T;

    /// 获取受互斥锁保护数据的共享引用。
    fn deref(&self) -> &T {
        // SAFETY:
        // 1. `SleepMutexGuard` 仅在 `try_lock` 中通过 `Ordering::Acquire` 成功将 `occupied` 置为 `true` 后创建，
        //    并在本守卫 `Drop` 前始终保持 `occupied == true`，排除了任何其他任务同时构造守卫；
        // 2. `_task: PhantomData<*mut ()>` 保证守卫不可跨任务共享，`&self` 共享借用排除了通过同一守卫同时进行可变借用；
        // 3. `self.mutex` 引用在 `'a` 期内有效，因此 `value.get()` 指向的 `T` 已初始化且无并发可变别名。
        unsafe { &*self.mutex.value.get() }
    }
}

impl<T> DerefMut for SleepMutexGuard<'_, T> {
    /// 获取受互斥锁保护数据的独占可变引用。
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY:
        // 1. `occupied` 原子位的 `Acquire` CAS 保证当前全局仅有这一个存活的 `SleepMutexGuard`；
        // 2. `&mut self` 独占可变借用排除了通过当前守卫产生的任何其他共享或可变引用；
        // 3. `self.mutex.value` 为私有字段，外部只能通过本守卫访问，满足独占可变引用的无别名要求。
        unsafe { &mut *self.mutex.value.get() }
    }
}

impl<T> Drop for SleepMutexGuard<'_, T> {
    /// 释放互斥锁的占用状态，并唤醒等待队列中的一名等待任务重新参与竞争。
    fn drop(&mut self) {
        // Step 1: 使用 Release 内存序清除占用位，确保临界区内对 `value` 的所有读写对下一任获锁者可见。
        self.mutex.occupied.store(false, Ordering::Release);
        // Step 2: 唤醒等待队列中的一个任务（若有），使其从 `park()` 返回并重新尝试 `try_lock()`。
        self.mutex.waiters.wake_one();
    }
}
