//! RISC-V Sv39 用户页表编码与硬件根表切换原语。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::riscv64` 架构抽象层，向上为 `mm::user_table`（硬件页表管理）与 `mm::address_space`
//! （进程地址空间生命周期与 TLB 维护）提供 Sv39 页表项（PTE）编解码、用户根表初始化、根表切换以及本地
//! TLB / 指令缓存刷新接口。
//!
//! # 核心设计与地址空间布局
//! RISC-V Sv39 采用 39 位有符号扩展虚拟地址和 3 级页表（每级 512 项，每项 8 字节）：
//!
//! ```text
//! 虚拟地址空间划分 (Sv39):
//! +-------------------------------------------------------------------+
//! | 0xffff_ffc0_0000_0000 ..= 0xffff_ffff_ffff_ffff                   |
//! | 内核高半空间（根页表项 256..512）：直接借用静态只读的最终内核根表 |
//! +-------------------------------------------------------------------+
//! | 0x0000_0040_0000_0000 ..  0xffff_ffc0_0000_0000                   |
//! | 非规范地址空洞（Non-canonical Hole，硬件禁止访问）                |
//! +-------------------------------------------------------------------+
//! | 0x0000_0000_0000_0000 ..  0x0000_0040_0000_0000 (256 GiB)         |
//! | 用户低半空间（根页表项 0..256）：由进程 Mm 独占拥有并管理下级表页 |
//! |   └── 顶部最后一页 (USER_ADDRESS_END - PAGE_SIZE):                |
//! |       信号返回蹦床页 (SIGNAL_RETURN_PAGE) / 用户栈顶边界          |
//! +-------------------------------------------------------------------+
//!
//! Sv39 页表项 (64-bit PTE) 关键位域:
//!  63      54 53                  10 9   8 7 6 5 4 3 2 1 0
//! +----------+----------------------+-----+-+-+-+-+-+-+-+-+
//! | Reserved |      PPN[2:0]        | RSW |D|A|G|U|X|W|R|V|
//! +----------+----------------------+-----+-+-+-+-+-+-+-+-+
//! ```
//!
//! ## 页表项状态约定
//! - **未映射空槽（Empty）**：全 0（`V=0` 且无任何保留标志）。
//! - **非叶目录项（Table）**：`V=1, R=0, W=0, X=0`，仅存在于非 0 层（Level 1 / Level 2）。
//! - **有效用户叶项（Valid Leaf）**：`V=1, U=1, A=1`，且 `R/W/X` 至少有一位为 1（禁止 `R=0, W=1` 的保留组合）；
//!   授予写权限（`W=1`）时同步预置脏位（`D=1`），避免依赖硬件自动置位特性并保守标记写回义务。
//! - **无权限驻留叶项（`PROT_NONE` Backing Leaf）**：`V=0, U=1, A=1, R=0, W=0, X=0`，保留物理页号 `PPN`。
//!   硬件遇到 `V=0` 会触发缺页异常，而软件可通过 `U|A` 标志区分“已绑定物理页但禁止访问”与“全 0 空槽”。
//!
//! # 锁获取顺序与并发契约
//! - 本模块不持有任何锁。软件对用户页表槽位的修改由上层 `Mm` 的页表自旋锁串行化。
//! - `activate_user_root` 与 `activate_kernel_root` 必须在**本地中断关闭（IRQ-off）**的上下文中执行，
//!   确保根表切换与后续 `sfence.vma` 期间不会重入切换或访问已释放的地址空间。

use crate::{
    arch::riscv64::{
        PAGE_SHIFT, PHYSICAL_TO_PTE_SHIFT, PTE_ACCESSED, PTE_DIRTY, PTE_EXEC, PTE_LEAF,
        PTE_PPN_SHIFT, PTE_READ, PTE_USER, PTE_VALID, PTE_WRITE, PT_HALF_ENTRIES, SATP_MODE_SV39,
        SATP_PPN_MASK, SV39_LEVELS, SV39_VA_BITS,
    },
    mm::{user_table::PagePermissions, PAGE_SIZE},
};
use core::{arch::asm, ptr};

/// 用户虚拟地址空间的排他上界（256 GiB，`0x0000_0040_0000_0000`）。
pub const USER_ADDRESS_END: usize = 1 << (SV39_VA_BITS - 1);
/// 用户栈的最高可用虚拟地址上界（避开顶部的信号返回蹦床页，单位：字节）。
pub const USER_STACK_TOP: usize = USER_ADDRESS_END - PAGE_SIZE;
/// 信号处理函数执行完毕后触发 `rt_sigreturn` 的内核映射蹦床页起始虚拟地址。
pub const SIGNAL_RETURN_PAGE: usize = USER_ADDRESS_END - PAGE_SIZE;
/// Sv39 用户页表的总层级数（3 级：Level 2 根目录、Level 1 中间目录、Level 0 叶表）。
pub const USER_LEVELS: usize = SV39_LEVELS;
/// PTE 中 44 位物理页号字段（`PPN[2:0]`，位 `10..54`）的掩码。
const PPN_MASK: usize = SATP_PPN_MASK << PTE_PPN_SHIFT;
/// PTE 低 10 位硬件属性与软件标志字段（位 `0..10`）的掩码。
const PTE_FLAGS_MASK: usize = (1 << PTE_PPN_SHIFT) - 1;
/// 已绑定用户物理页的叶项基础标志（`U | A`）；在 `V=0` 时作为 `PROT_NONE` 驻留页的软件识别标记。
const USER_BACKING_FLAGS: usize = PTE_USER | PTE_ACCESSED;

/// 返回指定页表层级中表示“未映射空槽”的页表项值。
///
/// 在普通任务或持锁上下文中均可调用（纯常量计算）。Sv39 各级空页表项统一编码为全 0。
pub const fn user_empty_entry(_level: usize) -> usize {
    0
}

/// 将下一级页表页的物理地址编码为 Sv39 非叶目录项（`V=1, R=0, W=0, X=0`）。
///
/// 纯常量计算函数。调用方（`mm::user_table`）负责通过 `user_entry_physical` 往返校验物理地址的对齐与位宽合法性。
pub const fn user_table_entry(physical: usize) -> usize {
    ((physical >> PHYSICAL_TO_PTE_SHIFT) & PPN_MASK) | PTE_VALID
}

/// 判断给定页表项在指定层级是否为指向下一级页表的非叶目录项。
///
/// 仅当层级大于 0（非叶层），且 `V=1` 同时 `R=0, W=0, X=0` 时返回 `true`。
pub const fn user_entry_is_table(entry: usize, level: usize) -> bool {
    0 < level && entry & (PTE_VALID | PTE_LEAF) == PTE_VALID
}

/// 从页表项的 `PPN` 字段提取 4KiB 页对齐的物理地址（单位：字节）。
pub const fn user_entry_physical(entry: usize, _level: usize) -> usize {
    (entry & PPN_MASK) << PHYSICAL_TO_PTE_SHIFT
}

/// 将物理页地址与访问权限编码为 Sv39 用户叶页表项。
///
/// - 若请求非法的“只写不读”（`!read && write`），Sv39 规范将其视为保留编码，返回 0。
/// - 若请求无访问权限（`PROT_NONE`，即 `!read && !write && !execute`），写入物理页号并置 `U | A`、清除 `V`，
///   使硬件访问触发缺页异常，同时软件仍能找回底层物理页。
/// - 若存在有效权限，置 `V | U | A` 及对应的 `R/W/X` 位；包含写权限时同步置 `D=1`。
pub const fn user_leaf_entry(physical: usize, permissions: PagePermissions) -> usize {
    if !permissions.read && permissions.write {
        return 0;
    }
    // V=0 时硬件忽略其余位，软件利用 U|A 标记“已绑定物理页但无访问权限（PROT_NONE）”，以区别于全 0 空槽。
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

/// 从页表项解码用户叶页的访问权限；若为空槽、非叶目录项或非法编码则返回 `None`。
///
/// 识别两类合法的用户叶项：
/// 1. `V=0` 且低 10 位恰好为 `U | A` 的无权限驻留页（返回全 `false` 的 `PagePermissions`）；
/// 2. `V=1, U=1`、含至少一个 `R/W/X` 位且不为“只写不读”的有效用户叶页。
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

/// 清除用户叶页表项的写权限（`W`）与脏位（`D`），保留物理页号、读/执行权限及其他标志位。
///
/// 用于写时复制（COW）或脏页写回前的只读保护重映射。
pub const fn user_leaf_mkclean(entry: usize) -> usize {
    entry & !(PTE_WRITE | PTE_DIRTY)
}

/// 检查用户叶页表项是否带有脏位（`D=1`）。
///
/// 内核在授予写权限时即预置 `D=1`，因此该位反映页面是否曾获得写授权并承担写回义务，而非精确的硬件写入次数。
pub const fn user_leaf_dirty(entry: usize) -> bool {
    entry & PTE_DIRTY != 0
}

/// 初始化新分配的用户根页表页：清零低半部分（用户空间 0..256 项），并将永久内核根表的高半部分（内核空间 256..512 项）复制到新根表。
///
/// 在进程地址空间创建期间调用。由于内核高半部分顶级目录项在引导完成后固定只读，各进程根表可直接共享这 256 个顶级表项。
///
/// # Safety
/// 1. `root` 必须指向由调用方独占拥有、可写、4KiB 页对齐的一整页物理内存（512 个 `usize` 容量），且尚未安装到任何 CPU 的 `satp` 寄存器。
/// 2. 引导阶段的最终内核页表（`FINAL_TABLES` 及 `RV_FINAL_SATP`）必须已构建完毕并在内核生命周期内永久有效。
/// 3. 调用方仅借用高半部分内核顶级目录项用于硬件地址翻译，严禁释放或修改高半部分指向的内核子页表。
pub unsafe fn initialize_user_root(root: *mut usize) {
    // SAFETY:
    // 1. 调用方契约保证 `root` 指向独占、4KiB 对齐的完整页表页（512 个 `usize`），低 256 项清零与高 256 项拷贝均在页边界内且不重叠；
    // 2. 前置 `assert_ne!` 确认 `RV_FINAL_SATP` 已在启动阶段发布，此时 `FINAL_TABLES[0]`（最终内核根表）只读且永久存活。
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

/// 在本地中断关闭（IRQ-off）的上下文切换或地址空间激活阶段，将指定用户根页表安装到当前 CPU 的 `satp` 寄存器并刷新本地 TLB。
///
/// 当前实现固定使用 ASID 0，每次切换根表后执行全量 `sfence.vma`，确保新页表项写入对硬件页表遍历器可见并清除旧地址空间的非全局翻译缓存。
///
/// # Safety
/// 1. `root_phys` 必须是已通过 `initialize_user_root` 初始化、4KiB 页对齐的 Sv39 根页表物理地址。
/// 2. 该根页表及其映射的所有可达下级表页与物理数据页在当前 CPU 切换到其他根表前必须保持存活。
/// 3. 根页表的高半部分必须包含完整的内核代码、当前内核栈及 `CpuLocal` 映射。
/// 4. 调用时当前 CPU 必须已关闭本地中断（`!irq_enabled()`），且上层 `Mm` 已登记当前 CPU 的活动引用。
pub unsafe fn activate_user_root(root_phys: usize) {
    assert_eq!(root_phys & (PAGE_SIZE - 1), 0);
    assert!(!super::irq_enabled());
    let satp = SATP_MODE_SV39 | (root_phys >> PAGE_SHIFT);
    // SAFETY:
    // 1. 前置 `assert_eq!` 与 `assert!` 已验证 `root_phys` 按 4KiB 对齐且当前 CPU 已关闭中断；
    // 2. 调用方保证 `root_phys` 包含完整的内核高半映射并在激活期间保活；
    // 3. `fence rw, rw` 确保此前对页表内存的修改在切换 `satp` 前全局可见，随后的 `sfence.vma` 刷新本地 TLB。
    unsafe {
        asm!("fence rw, rw", "csrw satp, {satp}", "sfence.vma", satp = in(reg) satp, options(nostack));
    }
}

/// 在本地中断关闭（IRQ-off）时将当前 CPU 的 `satp` 切回永久内核根页表，并刷新本地 TLB。
///
/// 在释放或卸载进程 `Mm` 前必须先调用本函数脱离用户根表，确保硬件不再访问即将回收的用户页表页。
pub fn activate_kernel_root() {
    assert!(!super::irq_enabled());
    // SAFETY:
    // 1. 前置 `assert!` 已确认本地中断关闭，且内部 `assert_ne!` 确认 `RV_FINAL_SATP` 已在引导阶段初始化；
    // 2. `RV_FINAL_SATP` 指向的静态内核根页表永久存活，完整覆盖当前内核代码、栈、直映区与 `CpuLocal`；
    // 3. `fence rw, rw` 与 `sfence.vma` 保证切换前后内存访问有序并彻底清除旧用户地址空间的本地 TLB 项。
    unsafe {
        let satp = ptr::read(ptr::addr_of!(super::RV_FINAL_SATP));
        assert_ne!(satp, 0);
        asm!("fence rw, rw", "csrw satp, {satp}", "sfence.vma", satp = in(reg) satp, options(nostack));
    }
}

/// 刷新当前 CPU 的全部 TLB 翻译缓存，并确保此前对页表项（PTE）的内存修改对后续指令与页表遍历器可见。
///
/// 供本核页表修改后直接调用，或由跨核同步 IPI（Call Single Data, CSD）回调在远端 CPU 上执行。
pub fn flush_user_translations() {
    // SAFETY: `sfence.vma` 仅失效当前 CPU 的地址翻译缓存并序列化页表读写，不改变当前映射内容或内存生命周期。
    unsafe {
        asm!("sfence.vma", options(nostack));
    }
}

/// 同步当前 CPU 的指令缓存（I-Cache），使其与此前完成的内存数据写入保持一致。
///
/// 当内核向包含可执行权限（`PROT_EXEC`）的用户代码页写入指令字节后，需在本核及通过跨核同步 IPI 在远端核上调用。
pub fn sync_user_icache() {
    // SAFETY: `fence.i` 仅同步当前 CPU 的取指流水线与已提交的数据存储，不解引用任何内存地址。
    unsafe {
        asm!("fence.i", options(nostack));
    }
}
