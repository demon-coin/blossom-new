//! 内核基础同步原语模块。
//!
//! # 模块职责与架构定位
//! 本模块提供全内核共享的底层并发控制与互斥原语，是调度器（`sched`）、内存管理（`mm`）、
//! 虚拟文件系统（`fs`）以及设备驱动的基础设施：
//! - **CPU 局部中断与抢占控制**：`IrqSaveGuard` 与 `PreemptGuard` 控制当前 CPU 核心的中断开关与抢占深度。
//! - **关中断自旋锁**：`SpinLock` 在获取自旋锁前自动保存并关闭本地中断，适合持锁时间极短、不可睡眠的临界区（包括中断上下文与调度器内部）。
//! - **可睡眠同步原语**：重导出子模块中的睡眠互斥锁（`SleepMutex`）、睡眠读写锁（`SleepRwLock`）与完成量（`Completion`），供可睡眠的普通任务使用。
//!
//! # 核心设计与生命周期嵌套顺序
//! `SpinLockGuard` 的加锁与释放严格遵循“先关中断后加锁，先放锁后恢复中断”的嵌套状态机，
//! 防止同核中断处理程序重入当前 CPU 正在持有的自旋锁而导致死锁：
//!
//! ```text
//!   SpinLock::lock()                           SpinLockGuard::drop()
//! ┌──────────────────────────┐               ┌──────────────────────────┐
//! │ 1. IrqSaveGuard::new()   │               │ 1. ManuallyDrop::drop()  │
//! │    保存并关闭本地 IRQ    │               │    先释放底层 spin::Mutex│
//! ├──────────────────────────┤               ├──────────────────────────┤
//! │ 2. inner.lock()          │  ══ 临界区 ══► │ 2. _irq 自动析构         │
//! │    自旋获取互斥锁        │               │    恢复进入前的本地 IRQ  │
//! └──────────────────────────┘               └──────────────────────────┘
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **睡眠锁与自旋锁的层级顺序**：可以**先持有可睡眠锁**（`SleepMutex` / `SleepRwLock`），在内部**再短暂获取自旋锁**（`SpinLock`）；**绝对禁止**在持有 `SpinLock`、`IrqSaveGuard` 或 `PreemptGuard` 期间调用任何可能睡眠或触发任务切换的操作（如 `SleepMutex::lock`、`SleepRwLock::read/write`、`Completion::wait`）。
//! 2. **守卫跨核限制**：`IrqSaveGuard`、`PreemptGuard` 与 `SpinLockGuard` 均包含 `!Send` / `!Sync` 标记或等价约束，严格绑定在获取它们的当前物理 CPU 上，禁止跨任务传递或在持锁期间发生跨核迁移。

pub mod completion;
pub mod mutex;
pub mod rwlock;

pub use completion::Completion;
pub use mutex::{SleepMutex, SleepMutexGuard};
pub use rwlock::{ReadGuard, SleepRwLock, WriteGuard};

use core::marker::PhantomData;
use core::mem::ManuallyDrop;
use core::ops::{Deref, DerefMut};

/// 保存当前 CPU 中断状态并关闭本地中断的 RAII 守卫。
///
/// 在构造时关闭当前 CPU 的本地外部/时钟中断，并在析构时精确恢复进入前的中断开启状态，
/// 支持嵌套使用。由于中断状态属于当前物理 CPU 核心，该守卫不可跨线程传递或共享（`!Send + !Sync`）。
pub struct IrqSaveGuard {
    /// 进入临界区前由硬件 CSR 保存的原始中断使能状态位。
    saved: usize,
    /// 零开销类型标记，将守卫标记为 `!Send` 和 `!Sync`，禁止跨任务或跨 CPU 移动。
    _local: PhantomData<*mut ()>,
}

impl IrqSaveGuard {
    /// 关闭当前 CPU 的本地中断，并返回保存了进入前中断状态的守卫。
    ///
    /// 可在任意上下文（普通任务、内核线程、中断处理函数）中调用，支持多层嵌套。
    pub fn new() -> Self {
        Self {
            saved: crate::arch::irq_save(),
            _local: PhantomData,
        }
    }
}

impl Drop for IrqSaveGuard {
    /// 退出临界区时，将当前 CPU 的本地中断状态恢复为构造此守卫前的状态。
    fn drop(&mut self) {
        // SAFETY:
        // 1. `_local: PhantomData<*mut ()>` 保证了 `IrqSaveGuard` 既不是 `Send` 也不是 `Sync`；
        // 2. 自 `new()` 关闭本地中断以来，当前执行流无法因时钟中断抢占而迁移到其他 CPU；
        // 3. `self.saved` 是当前同一物理 CPU 上由 `arch::irq_save()` 读取的合法状态字。
        unsafe { crate::arch::irq_restore(self.saved) };
    }
}

/// 禁止当前 CPU 发生抢占式任务切换的 RAII 守卫。
///
/// 持有期间递增当前 CPU 的 `preempt_depth` 计数器。与 `IrqSaveGuard` 不同，
/// `PreemptGuard` 在持有期间**允许响应硬件中断**，但阻止中断返回或同步路径触发抢占式任务切换，
/// 从而保证当前任务始终驻留在同一物理 CPU 上，安全访问 Per-CPU 数据结构。
pub struct PreemptGuard {
    /// 当前物理 CPU 的 Per-CPU 控制块静态引用。
    local: &'static crate::cpu::CpuLocal,
    /// 零开销类型标记，将守卫标记为 `!Send` 和 `!Sync`，禁止跨任务或跨 CPU 移动。
    _local: PhantomData<*mut ()>,
}

impl PreemptGuard {
    /// 递增当前 CPU 的抢占禁止深度计数，进入不可抢占临界区。
    ///
    /// 内部短暂关闭本地中断以原子地读取当前 CPU 控制块并递增 `preempt_depth`，
    /// 随后立即恢复中断状态。必须在当前 CPU 的 Per-CPU 结构完成初始化后调用。
    pub fn new() -> Self {
        // Step 1: 临时关闭本地中断，防止在获取当前 CPU 引用与递增计数之间被抢占迁核。
        let _irq = IrqSaveGuard::new();
        let local = crate::cpu::current();
        // Step 2: 递增当前 CPU 的抢占禁止计数；离开此作用域时 `_irq` 自动恢复原中断状态。
        crate::cpu::increment_depth(&local.preempt_depth);
        Self {
            local,
            _local: PhantomData,
        }
    }
}

impl Drop for PreemptGuard {
    /// 递减当前 CPU 的抢占禁止深度计数，退出不可抢占段。
    fn drop(&mut self) {
        // 临时关闭本地中断以保护 Per-CPU 计数器的递减操作。
        let _irq = IrqSaveGuard::new();
        crate::cpu::decrement_depth(&self.local.preempt_depth);
    }
}

/// 自动关闭本地中断的自旋互斥锁（SpinLock）。
///
/// 适用于持锁时间极短、不可睡眠的共享状态保护（例如调度队列、内存分配器元数据、中断与任务共享的数据）。
/// 每次加锁时都会自动保存并关闭当前 CPU 的本地中断，防止同一 CPU 上的中断处理函数因争用同一把锁而死锁。
pub struct SpinLock<T> {
    /// 底层的自旋互斥锁实现，包含受保护的数据 `T` 与原子自旋状态。
    inner: spin::Mutex<T>,
}

impl<T> SpinLock<T> {
    /// 创建一个处于未加锁状态的自旋锁，支持 `const` 静态初始化。
    pub const fn new(value: T) -> Self {
        Self {
            inner: spin::Mutex::new(value),
        }
    }

    /// 保存并关闭当前 CPU 的本地中断，然后自旋等待直至获取锁。
    ///
    /// 返回的 [`SpinLockGuard`] 在持有期间保持本地中断关闭，禁止在持锁期间睡眠或触发调度。
    pub fn lock(&self) -> SpinLockGuard<'_, T> {
        // Step 1: 先保存并关闭本地中断，再尝试加锁，防止获锁后立即被同核中断打断并重入死锁。
        let irq = IrqSaveGuard::new();
        // Step 2: 自旋获取底层互斥锁，并用 ManuallyDrop 包装以精确控制 Drop 释放顺序。
        let guard = ManuallyDrop::new(self.inner.lock());
        SpinLockGuard { guard, _irq: irq }
    }

    /// 保存并关闭本地中断后尝试非阻塞地获取一次锁。
    ///
    /// 若锁已被其他执行流持有，则立即恢复进入前的中断状态并返回 `None`；
    /// 若成功获取锁，则返回持有中断保存状态的 [`SpinLockGuard`]。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "启动、MM 和 panic 展开测试验证竞争失败及 guard 归还后的 IRQ 状态"
        )
    )]
    pub fn try_lock(&self) -> Option<SpinLockGuard<'_, T>> {
        self.try_lock_with_irq(IrqSaveGuard::new())
    }

    /// 接管调用方已创建的 [`IrqSaveGuard`] 并尝试非阻塞地获取一次锁。
    ///
    /// 若获锁成功，中断守卫的所有权转移给返回的 [`SpinLockGuard`]；
    /// 若获锁失败，立即释放传入的 `irq` 守卫以恢复本地中断状态，并返回 `None`。
    pub(crate) fn try_lock_with_irq(&self, irq: IrqSaveGuard) -> Option<SpinLockGuard<'_, T>> {
        match self.inner.try_lock() {
            Some(guard) => Some(SpinLockGuard {
                guard: ManuallyDrop::new(guard),
                _irq: irq,
            }),
            None => {
                drop(irq);
                None
            }
        }
    }
}

/// [`SpinLock`] 的 RAII 作用域守卫，持有期间保持当前 CPU 本地中断关闭。
///
/// 析构时严格按照“先释放自旋锁、再恢复本地中断状态”的顺序执行，
/// 确保锁刚释放或尚未释放时不会因本地中断提前开启而引发同核中断重入死锁。
pub struct SpinLockGuard<'a, T> {
    /// 底层 `spin::MutexGuard`，使用 `ManuallyDrop` 包装以便在 `_irq` 恢复中断前显式释放锁。
    guard: ManuallyDrop<spin::MutexGuard<'a, T>>,
    /// 保存了加锁前当前 CPU 中断状态的守卫，在 `Drop::drop` 执行完毕后按字段顺序自动析构。
    _irq: IrqSaveGuard,
}

impl<T> Deref for SpinLockGuard<'_, T> {
    type Target = T;

    /// 获取受自旋锁保护数据的共享引用。
    fn deref(&self) -> &T {
        &self.guard
    }
}

impl<T> DerefMut for SpinLockGuard<'_, T> {
    /// 获取受自旋锁保护数据的独占可变引用。
    fn deref_mut(&mut self) -> &mut T {
        &mut self.guard
    }
}

impl<T> Drop for SpinLockGuard<'_, T> {
    /// 先释放底层自旋锁，随后由编译器自动析构 `_irq` 字段以恢复当前 CPU 的中断状态。
    fn drop(&mut self) {
        // SAFETY:
        // 1. `self.guard` 在构造 `SpinLockGuard` 时被装入 `ManuallyDrop`，此前从未被手动丢弃；
        // 2. `Drop::drop` 仅在守卫生命周期结束时调用一次，调用后 `ManuallyDrop` 不会再次自动析构内部值；
        // 3. 在此显式释放底层自旋锁之后，结构体字段 `self._irq` 才会接着析构并恢复本地中断，
        //    从而保证恢复中断时当前 CPU 已不再持有该自旋锁。
        unsafe { ManuallyDrop::drop(&mut self.guard) };
    }
}
