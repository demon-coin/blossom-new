//! 短临界区只借用已有 spin 锁；锁释放以后才恢复当前 CPU 的原 IRQ 状态。

pub mod completion;
pub mod mutex;
pub mod rwlock;

pub use completion::Completion;
pub use mutex::{SleepMutex, SleepMutexGuard};
pub use rwlock::{ReadGuard, SleepRwLock, WriteGuard};

use core::marker::PhantomData;
use core::mem::ManuallyDrop;
use core::ops::{Deref, DerefMut};

/// 保存当前 CPU 中断状态的局部临界区，不能跨 CPU 移动。
pub struct IrqSaveGuard {
    saved: usize,
    _local: PhantomData<*mut ()>,
}

impl IrqSaveGuard {
    /// 关闭本地 IRQ，并保存进入前的状态。
    pub fn new() -> Self {
        Self {
            saved: crate::arch::irq_save(),
            _local: PhantomData,
        }
    }
}

impl Drop for IrqSaveGuard {
    /// 临界区结束后恢复同核进入时的 IRQ 位，不修改其他架构状态。
    fn drop(&mut self) {
        // SAFETY: guard 不可 Send/Sync，保存值来自同核 irq_save；IRQ 关闭期间不迁移 CPU。
        unsafe { crate::arch::irq_restore(self.saved) };
    }
}

/// CPU 初始化后的显式不可抢占段，可服务 IRQ，但不能跨任务切换。
pub struct PreemptGuard {
    local: &'static crate::cpu::CpuLocal,
    _local: PhantomData<*mut ()>,
}

impl PreemptGuard {
    /// 记录本 CPU 抢占禁止深度，临时关 IRQ 仅保护进入动作。
    pub fn new() -> Self {
        let _irq = IrqSaveGuard::new();
        let local = crate::cpu::current();
        crate::cpu::increment_depth(&local.preempt_depth);
        Self {
            local,
            _local: PhantomData,
        }
    }
}

impl Drop for PreemptGuard {
    /// 结束同核不可抢占段，恢复 IRQ 只由局部 IRQ guard 负责，不自动调度。
    fn drop(&mut self) {
        let _irq = IrqSaveGuard::new();
        crate::cpu::decrement_depth(&self.local.preempt_depth);
    }
}

/// 保护不能睡眠或长期等待的共享状态。
pub struct SpinLock<T> {
    inner: spin::Mutex<T>,
}

impl<T> SpinLock<T> {
    /// 在静态初始化中建立一把具体对象的短锁。
    pub const fn new(value: T) -> Self {
        Self {
            inner: spin::Mutex::new(value),
        }
    }

    /// 先保存并关闭 IRQ，再进入锁域。
    pub fn lock(&self) -> SpinLockGuard<'_, T> {
        let irq = IrqSaveGuard::new();
        let guard = ManuallyDrop::new(self.inner.lock());
        SpinLockGuard { guard, _irq: irq }
    }

    /// 关闭 IRQ 后仅尝试一次取锁；竞争失败立即恢复原状态，成功仍先解锁再恢复 IRQ。
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

    /// 接管固定当前 CPU 的 IRQ guard，仅尝试一次取锁；失败恢复原 IRQ，成功先解锁再恢复。
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

/// 持锁期间保持进入前的 IRQ 状态，退出顺序为解锁、恢复 IRQ。
pub struct SpinLockGuard<'a, T> {
    guard: ManuallyDrop<spin::MutexGuard<'a, T>>,
    _irq: IrqSaveGuard,
}

impl<T> Deref for SpinLockGuard<'_, T> {
    type Target = T;

    /// 借用锁内对象，不延长超过 guard 的生命周期。
    fn deref(&self) -> &T {
        &self.guard
    }
}

impl<T> DerefMut for SpinLockGuard<'_, T> {
    /// 在当前短锁独占期间借用可变对象。
    fn deref_mut(&mut self) -> &mut T {
        &mut self.guard
    }
}

impl<T> Drop for SpinLockGuard<'_, T> {
    /// 先释放短锁，再由字段析构恢复本核 IRQ，避免 handler 重入仍被持有的锁。
    fn drop(&mut self) {
        // SAFETY: guard 在构造时放入 ManuallyDrop，尚未手动析构；只在此释放一次，
        // 随后 _irq 析构，guard 字段不会再次析构。
        unsafe { ManuallyDrop::drop(&mut self.guard) };
    }
}
