//! 用户 Sv39 根借用永久内核高半映射，低半页表由 MM 独占拥有。

use crate::{
    arch::riscv64::{
        PAGE_SHIFT, PHYSICAL_TO_PTE_SHIFT, PTE_ACCESSED, PTE_DIRTY, PTE_EXEC, PTE_LEAF,
        PTE_PPN_SHIFT, PTE_READ, PTE_USER, PTE_VALID, PTE_WRITE, PT_HALF_ENTRIES, SATP_MODE_SV39,
        SATP_PPN_MASK, SV39_LEVELS, SV39_VA_BITS,
    },
    mm::{user_table::PagePermissions, PAGE_SIZE},
};
use core::{arch::asm, ptr};

pub const USER_ADDRESS_END: usize = 1 << (SV39_VA_BITS - 1);
pub const USER_STACK_TOP: usize = USER_ADDRESS_END - PAGE_SIZE;
pub const SIGNAL_RETURN_PAGE: usize = USER_ADDRESS_END - PAGE_SIZE;
pub const USER_LEVELS: usize = SV39_LEVELS;
const PPN_MASK: usize = SATP_PPN_MASK << PTE_PPN_SHIFT;
const PTE_FLAGS_MASK: usize = (1 << PTE_PPN_SHIFT) - 1;
const USER_BACKING_FLAGS: usize = PTE_USER | PTE_ACCESSED;

/// 未映射的 Sv39 项在所有层级均为零。
pub const fn user_empty_entry(_level: usize) -> usize {
    0
}

/// 编码仅含 V 位的非叶目录，物理地址合法性由调用者作往返检查。
pub const fn user_table_entry(physical: usize) -> usize {
    ((physical >> PHYSICAL_TO_PTE_SHIFT) & PPN_MASK) | PTE_VALID
}

/// 仅在非叶层识别有效且没有 R/W/X 的目录项。
pub const fn user_entry_is_table(entry: usize, level: usize) -> bool {
    0 < level && entry & (PTE_VALID | PTE_LEAF) == PTE_VALID
}

/// 从硬件 PPN 字段取得页对齐物理地址。
pub const fn user_entry_physical(entry: usize, _level: usize) -> usize {
    (entry & PPN_MASK) << PHYSICAL_TO_PTE_SHIFT
}

/// 编码用户叶项；无权限页保留 PPN 并关闭 V，Sv39 纯写权限无效。
pub const fn user_leaf_entry(physical: usize, permissions: PagePermissions) -> usize {
    if !permissions.read && permissions.write {
        return 0;
    }
    // V=0 时 U/A 由软件保留，用于区分无权限叶与没有 backing 的空槽。
    if !permissions.read && !permissions.execute {
        return ((physical >> PHYSICAL_TO_PTE_SHIFT) & PPN_MASK) | USER_BACKING_FLAGS;
    }
    ((physical >> PHYSICAL_TO_PTE_SHIFT) & PPN_MASK)
        | (USER_BACKING_FLAGS | PTE_VALID)
        | ((permissions.read as usize) * PTE_READ)
        | ((permissions.write as usize) * PTE_WRITE)
        | ((permissions.execute as usize) * PTE_EXEC)
        | ((permissions.write as usize) * PTE_DIRTY)
}

/// 解码有效用户叶或保留 backing 的无权限叶，目录与非法 R/W 组合不属于用户叶。
pub const fn user_leaf_permissions(entry: usize) -> Option<PagePermissions> {
    if entry & PTE_FLAGS_MASK == USER_BACKING_FLAGS {
        return Some(PagePermissions {
            read: false,
            write: false,
            execute: false,
        });
    }
    if entry & (PTE_VALID | PTE_USER) != (PTE_VALID | PTE_USER)
        || entry & PTE_LEAF == 0
        || entry & (PTE_READ | PTE_WRITE) == PTE_WRITE
    {
        return None;
    }
    Some(PagePermissions {
        read: entry & PTE_READ != 0,
        write: entry & PTE_WRITE != 0,
        execute: entry & PTE_EXEC != 0,
    })
}

/// 保留原 PFN、读执行权限及访问位，只撤销写许可和保守脏位。
pub const fn user_leaf_mkclean(entry: usize) -> usize {
    entry & !(PTE_WRITE | PTE_DIRTY)
}

/// 当前 D 表达写授权时预置的保守脏义务，不表示精确 store 次数。
pub const fn user_leaf_dirty(entry: usize) -> bool {
    entry & PTE_DIRTY != 0
}

/// 清空用户低半，并借用已经发表且不再修改的最终内核根高半。
///
/// # Safety
/// root 必须指向独占、可写、4KiB 对齐的完整页；最终内核表已安装并永久存活，
/// 此根尚未发表给硬件。调用者不得取得共享高半目录的拥有权或修改其内容。
pub unsafe fn initialize_user_root(root: *mut usize) {
    // SAFETY: 调用者保证新根的独占与整页容量；FINAL_TABLES 在引导完成后只读，
    // 第零张是实际最终根，高半的子表由永久静态表池保活。
    unsafe {
        assert_ne!(ptr::read(ptr::addr_of!(super::RV_FINAL_SATP)), 0);
        let kernel = ptr::addr_of!(super::FINAL_TABLES).cast::<usize>();
        ptr::write_bytes(root, 0, PT_HALF_ENTRIES);
        ptr::copy_nonoverlapping(
            kernel.add(PT_HALF_ENTRIES),
            root.add(PT_HALF_ENTRIES),
            PT_HALF_ENTRIES,
        );
    }
}

/// 在 IRQ 关闭的调度交接段安装 ASID0 用户根并清除本核旧翻译。
///
/// # Safety
/// root_phys 必须为已初始化、4KiB 对齐的 Sv39 根；完整表和所有映射的页仍被保活。
/// 内核高半必须包含当前代码、栈与 CpuLocal；本核 IRQ 已关闭，MM 活动归属由调用者管理。
pub unsafe fn activate_user_root(root_phys: usize) {
    assert_eq!(root_phys & (PAGE_SIZE - 1), 0);
    assert!(!super::irq_enabled());
    let satp = SATP_MODE_SV39 | (root_phys >> PAGE_SHIFT);
    // SAFETY: 调用者保证根及当前内核映射存活；ASID 固定零，完整 sfence 同时排序新 PTE。
    unsafe {
        asm!("fence rw, rw", "csrw satp, {satp}", "sfence.vma", satp = in(reg) satp, options(nostack));
    }
}

/// 在 IRQ 关闭时返回永久内核根，清除旧用户翻译后才允许释放活动 MM。
pub fn activate_kernel_root() {
    assert!(!super::irq_enabled());
    // SAFETY: 最终根在 AP 运行前已发表且永久存活；其代码、栈和直映覆盖当前执行资源。
    unsafe {
        let satp = ptr::read(ptr::addr_of!(super::RV_FINAL_SATP));
        assert_ne!(satp, 0);
        asm!("fence rw, rw", "csrw satp, {satp}", "sfence.vma", satp = in(reg) satp, options(nostack));
    }
}

/// 清除本核所有旧翻译并排序此前 PTE 修改，供同步 CSD 与切表使用。
pub fn flush_user_translations() {
    // SAFETY: 仅失效本核缓存且排序内存，不读取或改变映射页的拥有权。
    unsafe {
        asm!("sfence.vma", options(nostack));
    }
}

/// 同步本核指令缓存；远端执行者须通过既有同步 CSD 在各自核调用。
pub fn sync_user_icache() {
    // SAFETY: fence.i 仅同步本核取指与已完成的数据写，不访问未映射内存。
    unsafe {
        asm!("fence.i", options(nostack));
    }
}
