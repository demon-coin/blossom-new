//! 三层 4KiB 用户页表及本核成对 TLB；内核继续只使用 PLV0 DMW。

use core::arch::asm;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::arch::loongarch64::register as reg;
use crate::mm::{PAGE_SHIFT, PAGE_SIZE};

use super::{csr_write, irq_enabled, virt_to_phys};
use crate::mm::user_table::PagePermissions;

pub(super) const PT_INDEX_BITS: usize = 9;
pub(super) const PT_ENTRIES: usize = PAGE_SIZE / core::mem::size_of::<usize>();
pub(super) const PHYSICAL_BITS: usize = 48;
pub(super) const USER_ADDRESS_BITS: usize = PAGE_SHIFT + USER_LEVELS * PT_INDEX_BITS;
pub const USER_ADDRESS_END: usize = 1 << USER_ADDRESS_BITS;
pub const USER_STACK_TOP: usize = USER_ADDRESS_END - PAGE_SIZE;
pub const SIGNAL_RETURN_PAGE: usize = USER_ADDRESS_END - PAGE_SIZE;
pub const USER_LEVELS: usize = 3;

const VALID: usize = 1;
const DIRTY: usize = 1 << 1;
const USER: usize = reg::PLV_USER << 2;
const CACHED: usize = 1 << 4;
const PRESENT: usize = 1 << 7;
const WRITE: usize = 1 << 8;
const PROTNONE: usize = 1 << 10;
const NO_READ: usize = 1 << 61;
const NO_EXEC: usize = 1 << 62;
const PWCL_PTWIDTH_SHIFT: usize = 5;
const PWCL_DIR0BASE_SHIFT: usize = 10;
const PWCL_DIR0WIDTH_SHIFT: usize = 15;
const PWCH_DIR2WIDTH_SHIFT: usize = 6;
const TLBIDX_PS_SHIFT: usize = 24;
const INVTLB_ALL_NONGLOBAL: usize = 0x3;

const PHYSICAL: usize = ((1usize << PHYSICAL_BITS) - 1) & !(PAGE_SIZE - 1);

#[repr(C, align(4096))]
struct InvalidTable([usize; PT_ENTRIES]);

// refill 在 DA 下访问这些永久物理页；初始化完成后不再修改。
#[no_mangle]
static mut LA_INVALID_ROOT: InvalidTable = InvalidTable([0; PT_ENTRIES]);
static mut INVALID_MIDDLE: InvalidTable = InvalidTable([0; PT_ENTRIES]);
static mut INVALID_LEAF: InvalidTable = InvalidTable([0; PT_ENTRIES]);
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// 在 BSP 独占启动段逐项写入常驻 invalid 链，不在内核栈上构造表数组。
///
/// # Safety
/// 仅 BSP 可在 AP 发布前、IRQ 关闭时调用一次；三张静态页的永久 DMW 映射必须有效。
pub(super) unsafe fn initialize_invalid_tables() {
    assert!(
        !INITIALIZED.load(Ordering::Relaxed),
        "invalid tables initialized twice"
    );
    let root = addr_of_mut!(LA_INVALID_ROOT).cast::<usize>();
    let middle = addr_of_mut!(INVALID_MIDDLE).cast::<usize>();
    let leaf = addr_of_mut!(INVALID_LEAF).cast::<usize>();
    for index in 0..PT_ENTRIES {
        // SAFETY: BSP 独占三张静态对齐页，AP 尚未启动；index 严格处于各页边界内。
        unsafe {
            leaf.add(index).write(0);
            middle.add(index).write(virt_to_phys(leaf as usize));
            root.add(index).write(virt_to_phys(middle as usize));
        }
    }
    INITIALIZED.store(true, Ordering::Release);
}

/// 返回该层空目录的永久物理 sentinel，叶层的无效项为零。
pub fn user_empty_entry(level: usize) -> usize {
    match level {
        0 => 0,
        1 => virt_to_phys(addr_of_mut!(INVALID_LEAF) as usize),
        2 => virt_to_phys(addr_of_mut!(INVALID_MIDDLE) as usize),
        _ => panic!("invalid user page-table level"),
    }
}

/// 编码下级表物理地址；目录没有叶权限位。
pub fn user_table_entry(physical: usize) -> usize {
    physical
}

/// 识别本层拥有的真实下级表，排除常驻空目录。
pub fn user_entry_is_table(entry: usize, level: usize) -> bool {
    level != 0 && entry != user_empty_entry(level)
}

/// 从目录或叶取页物理地址，丢弃权限及软件位。
pub fn user_entry_physical(entry: usize, _level: usize) -> usize {
    entry & PHYSICAL
}

/// 编码已驻留 PLV3 叶；软件可写和硬件 DIRTY 同步，NR/NX 独立表达访问权限。
pub fn user_leaf_entry(physical: usize, permissions: PagePermissions) -> usize {
    if !permissions.read && !permissions.write && !permissions.execute {
        return physical | USER | CACHED | PROTNONE | NO_READ | NO_EXEC;
    }
    physical
        | VALID
        | USER
        | CACHED
        | PRESENT
        | if permissions.write { DIRTY | WRITE } else { 0 }
        | if permissions.read { 0 } else { NO_READ }
        | if permissions.execute { 0 } else { NO_EXEC }
}

/// 解码已驻留叶的三个用户权限，空叶和目录不表示用户映射。
pub fn user_leaf_permissions(entry: usize) -> Option<PagePermissions> {
    if entry & (PROTNONE | USER) == (PROTNONE | USER) {
        return Some(PagePermissions {
            read: false,
            write: false,
            execute: false,
        });
    }
    if entry & (PRESENT | VALID | USER) != (PRESENT | VALID | USER) {
        return None;
    }
    if (entry & WRITE != 0) != (entry & DIRTY != 0) {
        return None;
    }
    Some(PagePermissions {
        read: entry & NO_READ == 0,
        write: entry & WRITE != 0,
        execute: entry & NO_EXEC == 0,
    })
}

/// 原子降权须同时撤掉软件 WRITE 与硬件 DIRTY，完整保留 PFN、NR/NX 和其他位。
pub fn user_leaf_mkclean(entry: usize) -> usize {
    entry & !(WRITE | DIRTY)
}

/// DIRTY 在写授权时预置，表示保守脏义务而非精确 store 次数。
pub fn user_leaf_dirty(entry: usize) -> bool {
    entry & DIRTY != 0
}

/// 将新根全部初始化为空目录；用户表不借用任何内核 DMW 页表。
///
/// # Safety
/// root 指向独占、可写且 4KiB 对齐的一整页；硬件及其他执行者尚不能访问该页。
pub unsafe fn initialize_user_root(root: *mut usize) {
    assert!(
        INITIALIZED.load(Ordering::Acquire),
        "invalid tables are not initialized"
    );
    for index in 0..PT_ENTRIES {
        // SAFETY: 调用者保证一整页独占可写，index 小于每页 512 项，写入永久下级 sentinel。
        unsafe { root.add(index).write(user_empty_entry(2)) };
    }
}

/// 为本核安装 12/21/30 位起点、各 9 位索引的三级 walk，关闭硬件自动 walker。
pub(super) fn initialize_current_cpu() {
    assert!(
        INITIALIZED.load(Ordering::Acquire),
        "invalid tables are not initialized"
    );
    assert!(!irq_enabled(), "page walker setup requires IRQ off");
    // PWCL 的 PT/DIR1 分别对应叶和中间层；PWCH 的 DIR3 对应根。
    // 三层覆盖 39 位地址，不启用零宽度的 DIR2/DIR4；PTE 宽度为 64 位。
    // SAFETY: 当前 PLV0/IRQ-off，代码和栈仅依赖 DMW；静态 invalid 链已经发布。
    unsafe {
        csr_write::<{ reg::CSR_PWCL }>(
            PAGE_SHIFT
                | (PT_INDEX_BITS << PWCL_PTWIDTH_SHIFT)
                | ((PAGE_SHIFT + PT_INDEX_BITS) << PWCL_DIR0BASE_SHIFT)
                | (PT_INDEX_BITS << PWCL_DIR0WIDTH_SHIFT),
        );
        csr_write::<{ reg::CSR_PWCH }>(
            (PAGE_SHIFT + 2 * PT_INDEX_BITS) | (PT_INDEX_BITS << PWCH_DIR2WIDTH_SHIFT),
        );
        csr_write::<{ reg::CSR_STLBPS }>(PAGE_SHIFT);
        csr_write::<{ reg::CSR_TLBIDX }>(PAGE_SHIFT << TLBIDX_PS_SHIFT);
        csr_write::<{ reg::CSR_TLBREHI }>(PAGE_SHIFT);
        csr_write::<{ reg::CSR_PGDH }>(virt_to_phys(addr_of_mut!(LA_INVALID_ROOT) as usize));
    }
    activate_kernel_root();
}

/// 在当前 IRQ-off 核安装 ASID0 用户根，再清除旧根全部本地非全局翻译。
///
/// # Safety
/// root_phys 是有效三级表根；调用者保活整个可达表及叶到装入另一根并完成失效之后。
/// 当前须 PLV0、IRQ 关闭，代码、栈和 CpuLocal 的永久 DMW 映射必须有效。
pub unsafe fn activate_user_root(root_phys: usize) {
    assert!(!irq_enabled(), "user root switch requires IRQ off");
    assert_eq!(root_phys & (PAGE_SIZE - 1), 0, "unaligned user root");
    // SAFETY: 调用者保活根和可达页，PGDL 不参与当前 PLV0 DMW 翻译；ASID 固定为零。
    unsafe {
        csr_write::<{ reg::CSR_PGDL }>(root_phys);
        csr_write::<{ reg::CSR_ASID }>(0);
    }
    flush_user_translations();
}

/// 回到永久空根并清除 ASID0 的本地翻译；调用者须在 IRQ 关闭的 PLV0 执行。
pub fn activate_kernel_root() {
    assert!(!irq_enabled(), "kernel root switch requires IRQ off");
    // SAFETY: 静态根及下级 invalid 页永久有效，内核仅使用 DMW；当前调用者保证 IRQ-off。
    unsafe { activate_user_root(virt_to_phys(addr_of_mut!(LA_INVALID_ROOT) as usize)) };
}

/// 完成页表写入并失效本核全部非全局项，保留内核 DMW 不受影响。
pub fn flush_user_translations() {
    // SAFETY: 当前 PLV0；invtlb 0x3 清除 Global=0 的全部项，前置 dbar 排序页表写入。
    unsafe {
        asm!("dbar 0", "invtlb {operation}, $zero, $zero", operation = const INVTLB_ALL_NONGLOBAL, options(nostack))
    };
}

/// 排序用户代码写入并同步本核取指视图。
pub fn sync_user_icache() {
    // SAFETY: 当前为 PLV0，屏障不解引用用户地址；ibar 同步此前写入的指令。
    unsafe { asm!("dbar 0", "ibar 0", options(nostack)) };
}
