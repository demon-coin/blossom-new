//! 辅核（AP）动态内核栈槽位管理（`StackSlot`）。
//!
//! # 模块职责与架构定位
//! 本模块管理每个 CPU 在 `CpuLocal` 中持有的内核启动与调度栈槽位：
//! - 上游调用者：`cpu` 模块在初始化主核（BSP）静态栈、为辅核（AP）批量分配连续物理页栈以及回收启动失败的辅核栈时调用。
//! - 下游依赖：依赖物理页分配器（`mm::alloc_pages` / `OwnedPages`）分配与自动归还物理页，依赖 `arch::phys_to_virt` 计算直接映射虚拟地址。
//!
//! # 核心设计与生命周期状态机
//! 1. **主核与辅核栈的区别**：
//!    - 主核（CPU 0）在链接期即拥有 `.boot.stacks` 段中的静态栈 `BOOT_STACK`，其 `StackSlot` 初始化时 `top` 非零，`pages` 为 `None`。
//!    - 辅核（CPU 1..N）在早期 `cpu::initialize` 时 `top` 为 `0`、`pages` 为 `None`；待内存管理子系统（MM）建立后，由 `prepare_slots` 统一分配连续物理页（大小 `STACK_SIZE = 64 KiB`），全部成功后发布栈顶虚拟地址 `top`。
//! 2. **栈槽状态转换**：
//!    ```text
//!    [空槽: top == 0, pages == None]
//!           │
//!           │ prepare_slots() 分配 2^order 个连续物理页
//!           ▼
//!    [已分配未发布: top == 0, pages == Some(OwnedPages)]
//!           │
//!           ├─(若后续槽位分配失败)─> release_unstarted() ──> 锁外 drop(OwnedPages) 归还物理页
//!           │
//!           │ 全部辅核分配成功，Release 写入 top
//!           ▼
//!    [已发布: top != 0, pages == Some(OwnedPages)]
//!           │
//!           ├─(该辅核尚未交给固件且前序辅核启动失败)─> release_unstarted() 回收
//!           └─(已交给固件启动)─> 随 CpuLocal 永久驻留，不可再回收
//!    ```
//!
//! # 锁获取顺序与并发契约
//! - `StackSlot::pages` 使用独立的自旋锁（`SpinLock`）保护，持锁期间仅做 `Option<OwnedPages>` 的读写或 `take()`。
//! - 释放物理页（`drop(pages)`）必须在释放 `self.pages` 自旋锁之后执行，避免在持有栈槽自旋锁时重入伙伴系统分配器锁。

use alloc::alloc::AllocError;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::cpu::STACK_SIZE;
use crate::mm::{alloc_pages, OwnedPages, PAGE_SIZE};
use crate::sync::SpinLock;

/// 单个 CPU 的内核栈槽位，保存已发布的栈顶虚拟地址以及动态分配物理页的所有权。
pub(crate) struct StackSlot {
    /// 已发布的栈顶高地址（直接映射虚拟地址，单位：字节）；为 `0` 表示尚未准备好可供启动使用的栈。
    top: AtomicUsize,
    /// 辅核动态分配的连续物理页所有权（由自旋锁保护）；主核使用链接期静态栈，此字段恒为 `None`。
    pages: SpinLock<Option<OwnedPages>>,
}

impl StackSlot {
    /// 构造一个栈槽：主核传入静态栈顶地址 `top`，辅核传入 `0` 等待后续动态分配。
    pub(crate) const fn new(top: usize) -> Self {
        Self {
            top: AtomicUsize::new(top),
            pages: SpinLock::new(None),
        }
    }

    /// 读取当前已发布的栈顶虚拟地址（`Acquire` 顺序）；返回 `0` 表示栈尚未就绪。
    pub(crate) fn top(&self) -> usize {
        self.top.load(Ordering::Acquire)
    }

    /// 返回辅核动态栈在直接映射区的虚拟起始地址与字节长度 `(base_virt, len)`；主核静态栈或未分配槽返回 `None`。
    #[cfg(feature = "ktest")]
    pub(crate) fn allocation(&self) -> Option<(usize, usize)> {
        self.pages
            .lock()
            .as_ref()
            .map(|pages| (crate::arch::phys_to_virt(pages.phys_addr()), pages.len()))
    }

    /// 撤销从未交给辅核使用的动态栈：将 `top` 清零并在自旋锁外将物理页归还给页分配器（对空槽调用为无操作）。
    ///
    /// # Safety
    /// 1. 调用方必须独占当前槽位的准备与释放过程，且当前槽位不能是主核的静态栈。
    /// 2. 该槽位对应的栈内存及栈顶地址必须从未交给任何辅核或固件使用，也不存在任何并发指针访问该栈。
    /// 3. 若全局启动记录（`BOOT_RECORDS`）曾写入过该栈顶，调用前必须先将启动记录中的 `stack_top` 清零。
    pub(crate) unsafe fn release_unstarted(&self) {
        self.top.store(0, Ordering::Release);
        let pages = self.pages.lock().take();
        drop(pages);
    }
}

/// 为迭代器中的所有辅核栈槽批量分配连续物理页，全部成功后统一发布栈顶地址；若中途分配失败则回滚并归还已分配的物理页。
///
/// # Safety
/// 1. 物理页分配器与永久可写的直接映射区（Direct Mapping）必须已经建立，且当前上下文允许分配与释放物理页。
/// 2. `slots` 及其克隆必须产生同一组有限、无重复引用的 `StackSlot` 集合，所有槽位初始均为空（`top == 0` 且 `pages == None`）。
/// 3. 调用方（BSP）必须独占这些槽位的准备过程，期间没有固件或其他 CPU 并发访问它们。
/// 4. 准备成功后，只要对应的辅核被唤醒并开始使用该栈，`StackSlot` 就必须在内核生命周期内保持有效且不可释放。
pub(crate) unsafe fn prepare_slots<'a>(
    slots: impl Iterator<Item = &'a StackSlot> + Clone,
) -> Result<(), AllocError> {
    // Step 1: 校验所有目标槽位均处于未发布、未持有物理页的初始空状态。
    for slot in slots.clone() {
        assert_eq!(slot.top(), 0, "CPU stack is already published");
        assert!(slot.pages.lock().is_none(), "CPU stack already owns pages");
    }

    // Step 2: 按 STACK_SIZE 对应的伙伴系统阶数（order）逐槽分配连续物理页；若任一槽分配失败，回滚释放此前已分配的所有槽位。
    let order = (STACK_SIZE / PAGE_SIZE).trailing_zeros() as usize;
    for slot in slots.clone() {
        let Some(pages) = alloc_pages(order, false) else {
            for candidate in slots.clone() {
                // SAFETY:
                // 1. 当前处于 Step 2 分配阶段，所有候选槽位的 `top` 仍为 0，尚未写入任何启动记录或交给固件。
                // 2. 调用方（BSP）独占整个 `slots` 集合，仅需回收刚刚在本轮循环前几步分配的 `OwnedPages`。
                unsafe { candidate.release_unstarted() };
            }
            return Err(AllocError);
        };
        *slot.pages.lock() = Some(pages);
    }

    // Step 3: 所有槽位的物理页均已分配成功，计算每个栈的高地址端（栈顶）并以 Release 顺序发布。
    for slot in slots {
        let top = {
            let pages = slot.pages.lock();
            let pages = pages.as_ref().expect("prepared CPU stack lost its pages");
            crate::arch::phys_to_virt(pages.phys_addr()) + pages.len()
        };
        slot.top.store(top, Ordering::Release);
    }
    Ok(())
}
