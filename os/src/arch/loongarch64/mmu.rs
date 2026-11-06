//! LoongArch64 用户态三级页表（Sv39 等价布局）与本地 TLB 管理模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 `arch::loongarch64` 硬件抽象层，负责管理用户态（`PLV3`）虚拟地址空间的三级页表编码、
//! 永久空页表哨兵链（Invalid Sentinel Tables）初始化、页表遍历控制寄存器（`PWCL`/`PWCH`）配置，
//! 以及本地 CPU 的 TLB 刷新与指令缓存（I-Cache）同步。
//! - **上游调用者**：虚拟内存子系统（`mm::user_table`、`mm::space`）调用本模块的页表项编解码与根切换接口；
//!   CPU 启动入口（`arch::loongarch64`）调用本模块初始化静态空表与每核页表遍历寄存器；
//!   TLB 重填异常入口（`trap::la_kernel_refill`）在直接地址翻译模式（`DA=1`）下使用 `lddir` 指令遍历本模块维护的页表。
//! - **下游依赖**：依赖 `register` 模块定义的 CSR 编号与权限位常量，以及 `mm` 模块的基础页大小定义。
//!
//! # 核心设计与地址空间布局
//! 内核态（`PLV0`）始终通过直接映射配置窗口（`DMW0` 强序设备映射、`DMW1` 可缓存物理内存映射）访问物理内存，
//! 不依赖页表；只有用户态（`PLV3`）的低半区虚拟地址（`0x0000_0000_0000_0000 .. 0x0000_0080_0000_0000`，共 512 GiB）
//! 通过 `CSR_PGDL` 指向的三级页表进行翻译。高半区页表基址 `CSR_PGDH` 固定指向永久无效根表 `LA_INVALID_ROOT`。
//!
//! ## 39 位用户虚拟地址与三级页表拆分（4 KiB 页）
//! ```text
//! 63          39 38       30 29       21 20       12 11         0
//! +-------------+-----------+-----------+-----------+------------+
//! |  必须全为 0 |   DIR2    |   DIR0    |    PT     | PageOffset |
//! |   (25 bits) | (Level 2) | (Level 1) | (Level 0) | (12 bits)  |
//! +-------------+-----------+-----------+-----------+------------+
//!                     |           |           |
//!                     v           v           v
//!       CSR_PGDL -> [根表] ---> [中间表] ---> [叶表 (偶奇配对装入 TLBRELO0/1)]
//! ```
//!
//! ## 永久无效页表哨兵链（Invalid Sentinel Chain）
//! LoongArch 的 TLB 重填异常运行在直接地址翻译模式（`CRMD.DA=1, PG=0`）下，使用硬件 `lddir` 指令逐级取下级页表基址。
//! 为了避免在重填汇编快路径中对每一级目录做空指针分支判断，本模块在启动时建立了一条三层静态全零哨兵链：
//! ```text
//! LA_INVALID_ROOT (Level 2, 512 项全部指向 INVALID_MIDDLE 物理地址)
//!       |
//!       v
//! INVALID_MIDDLE  (Level 1, 512 项全部指向 INVALID_LEAF 物理地址)
//!       |
//!       v
//! INVALID_LEAF    (Level 0, 512 项全部为 0，即 Valid=0, Present=0)
//! ```
//! 任何未分配下级表的目录槽都填入对应层级的哨兵表物理地址。这样无论访问哪个未映射的虚拟地址，
//! 重填快路径都能无分支地走完三级页表（两次 `lddir` 加上叶级偶奇项 `ld.d`）并将全 0 叶表项装入 TLB，随后由硬件自然触发标准的页无效异常（PIL/PIS/PIF）交由内核缺页处理。
//!
//! ## LoongArch64 64 位叶页表项（PTE）位域布局
//! ```text
//! 63 62 61 60         48 47                    12 11  10   9  8  7  6 5  4   3  2  1 0
//! +--+--+--+------------+------------------------+---+----+--+--+--+---+----+----+--+-+
//! |R |NX|NR|  Reserved  |      PPN (物理页号)    |Rsv|PROT|R |W |P | G |MAT |PLV |D |V|
//! |  |  |  |            |       [47:12]          |   |NONE|  |  |  |   |[5:4|[3:2|  | |
//! +--+--+--+------------+------------------------+---+----+--+--+--+---+----+----+--+-+
//!  - Bit 0 (V / VALID): 硬件有效位。PROT_NONE 页置 0，其余已映射用户页置 1。
//!  - Bit 1 (D / DIRTY): 硬件脏位（决定是否允许 Store 指令直接写入而不触发 PME）。与 Bit 8 (WRITE) 同步设置。
//!  - Bits 3:2 (PLV): 特权级，用户页固定为 PLV_USER (3)。
//!  - Bits 5:4 (MAT): 存储访问类型，普通内存固定为可缓存一致映射 CACHED (1)。
//!  - Bit 7 (P / PRESENT): 物理页驻留位。
//!  - Bit 8 (W / WRITE): 软件可写标志，始终与硬件 DIRTY 位同步置位或清除。
//!  - Bit 10 (PROTNONE): 软件保留位，标记已建立映射但读写执行权限全关（PROT_NONE）的用户页。
//!  - Bit 61 (NR / NO_READ): 不可读位（置 1 禁止读取，置 0 允许读取）。
//!  - Bit 62 (NX / NO_EXEC): 不可执行位（置 1 禁止取指，置 0 允许取指）。
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块自身不持有自旋锁或睡眠锁。
//! - `INITIALIZED` 原子标志仅在主核（BSP）单线程引导阶段由 `initialize_invalid_tables` 写入一次（`Release`），
//!   后续各核通过 `Acquire` 或 `Relaxed` 只读断言。
//! - 页表项的并发读写由上层 `Mm` 的页表自旋锁（PTE `SpinLock`）或原子操作保证；
//!   `activate_user_root`、`activate_kernel_root` 与 `initialize_current_cpu` 必须在当前核关闭本地中断（IRQ off）的 `PLV0` 上下文中调用。

use core::arch::asm;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::arch::loongarch64::register as reg;
use crate::mm::{PAGE_SHIFT, PAGE_SIZE};

use super::{csr_write, irq_enabled, virt_to_phys};
use crate::mm::user_table::PagePermissions;

/// 单级页表索引占用的比特数（4 KiB 页含 512 项，对应 9 位索引）。
pub(super) const PT_INDEX_BITS: usize = 9;
/// 单张 4 KiB 页表包含的 `usize`（64 位）表项数量（512 项）。
pub(super) const PT_ENTRIES: usize = PAGE_SIZE / core::mem::size_of::<usize>();
/// 架构支持的物理地址有效位宽（48 位，最大寻址 256 TiB 物理空间）。
pub(super) const PHYSICAL_BITS: usize = 48;
/// 用户态虚拟地址空间有效位宽（`12 + 3 * 9 = 39` 位，共 512 GiB）。
pub(super) const USER_ADDRESS_BITS: usize = PAGE_SHIFT + USER_LEVELS * PT_INDEX_BITS;
/// 用户态虚拟地址空间上界（`1 << 39 = 0x0000_0080_0000_0000`）。
pub const USER_ADDRESS_END: usize = 1 << USER_ADDRESS_BITS;
/// 用户栈默认栈顶地址（紧邻最高一页信号跳板页之下）。
pub const USER_STACK_TOP: usize = USER_ADDRESS_END - PAGE_SIZE;
/// 用户态信号处理返回跳板页（`rt_sigreturn`）的固定虚拟基址（位于用户空间最高一页）。
pub const SIGNAL_RETURN_PAGE: usize = USER_ADDRESS_END - PAGE_SIZE;
/// 用户页表总层级数（3 级：根表 Level 2、中间表 Level 1、叶表 Level 0）。
pub const USER_LEVELS: usize = 3;

/// 叶表项有效位（`V`，Bit 0）：置 1 表示 TLB 表项有效且可参与地址翻译。
const VALID: usize = 1;
/// 叶表项脏位（`D`，Bit 1）：置 1 表示允许直接写入该页而不触发页修改异常（PME）。
const DIRTY: usize = 1 << 1;
/// 叶表项用户特权级字段（`PLV=3`，Bits 3:2）：允许 `PLV3` 用户态访问。
const USER: usize = reg::PLV_USER << 2;
/// 叶表项存储访问类型字段（`MAT=1`，Bit 4）：可缓存的一致性普通内存（Coherent Cached）。
const CACHED: usize = 1 << 4;
/// 叶表项物理页驻留位（`P`，Bit 7）：标识物理页在内存中已驻留。
const PRESENT: usize = 1 << 7;
/// 叶表项可写位（`W`，Bit 8）：标识该映射拥有写权限，本实现中始终与 `DIRTY` 同步。
const WRITE: usize = 1 << 8;
/// 软件定义的无权限映射标志位（Bit 10）：标识 `PROT_NONE`（已映射但无读/写/执行权限）的叶表项。
const PROTNONE: usize = 1 << 10;
/// 叶表项禁止读取位（`NR`，Bit 61）：置 1 表示禁止读取（Load）访问。
const NO_READ: usize = 1 << 61;
/// 叶表项禁止执行位（`NX`，Bit 62）：置 1 表示禁止取指（Fetch/Execute）访问。
const NO_EXEC: usize = 1 << 62;
/// `CSR_PWCL` 中叶表索引宽度字段（`PTWidth`）的起始位偏移。
const PWCL_PTWIDTH_SHIFT: usize = 5;
/// `CSR_PWCL` 中第一级目录起始位字段（`Dir1_base` / 代码中记为 `DIR0BASE`）的起始位偏移。
const PWCL_DIR0BASE_SHIFT: usize = 10;
/// `CSR_PWCL` 中第一级目录索引宽度字段（`Dir1_width` / 代码中记为 `DIR0WIDTH`）的起始位偏移。
const PWCL_DIR0WIDTH_SHIFT: usize = 15;
/// `CSR_PWCH` 中第三级目录索引宽度字段（`Dir3_width` / 代码中记为 `DIR2WIDTH`）的起始位偏移。
const PWCH_DIR2WIDTH_SHIFT: usize = 6;
/// `CSR_TLBIDX` 中页大小字段（`PS`）的起始位偏移（Bit 24）。
const TLBIDX_PS_SHIFT: usize = 24;
/// `invtlb` 指令操作码 `0x3`：清除当前 CPU 上所有非全局（`G=0`）的 TLB 表项。
const INVTLB_ALL_NONGLOBAL: usize = 0x3;

/// 从页表项中提取 4 KiB 对齐物理页基址（PPN）的位掩码（Bits `[47:12]`）。
const PHYSICAL: usize = ((1usize << PHYSICAL_BITS) - 1) & !(PAGE_SIZE - 1);

/// 4 KiB 对齐的静态无效页表容器，用于构建永久空目录哨兵链。
#[repr(C, align(4096))]
struct InvalidTable([usize; PT_ENTRIES]);

/// 全局永久无效根页表（Level 2）：所有表项指向 `INVALID_MIDDLE` 的物理地址。
/// TLB 重填异常在 `DA=1` 模式下只读访问此页；BSP 启动初始化完成后永不修改。
#[no_mangle]
static mut LA_INVALID_ROOT: InvalidTable = InvalidTable([0; PT_ENTRIES]);
/// 全局永久无效中间页表（Level 1）：所有表项指向 `INVALID_LEAF` 的物理地址。
static mut INVALID_MIDDLE: InvalidTable = InvalidTable([0; PT_ENTRIES]);
/// 全局永久无效叶页表（Level 0）：所有表项均为 0（`Valid=0`），使未映射访问平稳触发缺页异常。
static mut INVALID_LEAF: InvalidTable = InvalidTable([0; PT_ENTRIES]);
/// 标记静态无效页表哨兵链是否已由主核（BSP）完成一次性初始化。
static INITIALIZED: AtomicBool = AtomicBool::new(false);
/// 初始化全局永久无效页表哨兵链（`LA_INVALID_ROOT -> INVALID_MIDDLE -> INVALID_LEAF`）。
///
/// 在主核（BSP）独占启动阶段逐项原地写入三张静态页表，避免在内核栈上构造大数组。
///
/// # Safety
/// - 只能在主核（BSP）启动阶段、唤醒从核（AP）之前且本地中断关闭（IRQ off）时调用一次。
/// - 调用时内核的可缓存直接映射窗口（`DMW1`）必须已生效，以保证三张静态页表的虚拟地址可合法读写并正确换算为物理地址。
pub(super) unsafe fn initialize_invalid_tables() {
    assert!(
        !INITIALIZED.load(Ordering::Relaxed),
        "invalid tables initialized twice"
    );
    let root = addr_of_mut!(LA_INVALID_ROOT).cast::<usize>();
    let middle = addr_of_mut!(INVALID_MIDDLE).cast::<usize>();
    let leaf = addr_of_mut!(INVALID_LEAF).cast::<usize>();
    // Step 1: 逐项填充三张 4 KiB 静态页表，使根表指向中间表、中间表指向全零叶表。
    for index in 0..PT_ENTRIES {
        // SAFETY:
        // 1. 前置 `assert!(!INITIALIZED)` 配合调用方契约保证当前处于 BSP 单核独占启动阶段，AP 尚未启动且中断已关闭，对 `static mut` 不存在并发访问。
        // 2. `index` 范围为 `0..512`（`PT_ENTRIES`），指针偏移严格落在各自 4 KiB 对齐的 `InvalidTable` 数组边界内。
        unsafe {
            leaf.add(index).write(0);
            middle.add(index).write(virt_to_phys(leaf as usize));
            root.add(index).write(virt_to_phys(middle as usize));
        }
    }
    // Step 2: 发布初始化完成标志，供后续页表操作与各核初始化检查。
    INITIALIZED.store(true, Ordering::Release);
}

/// 返回指定页表层级（`level`）对应的空表项哨兵值。
///
/// - `level == 0`（叶表层）：返回 `0`（无效叶表项）。
/// - `level == 1`（中间目录层）：返回全局全零叶表 `INVALID_LEAF` 的物理基址。
/// - `level == 2`（根目录层）：返回全局无效中间表 `INVALID_MIDDLE` 的物理基址。
/// - 其他层级：触发 panic。
pub fn user_empty_entry(level: usize) -> usize {
    match level {
        0 => 0,
        1 => virt_to_phys(addr_of_mut!(INVALID_LEAF) as usize),
        2 => virt_to_phys(addr_of_mut!(INVALID_MIDDLE) as usize),
        _ => panic!("invalid user page-table level"),
    }
}

/// 将下级页表的物理基址编码为目录表项（LoongArch 目录项直接存放下级表物理页基址，不含权限位）。
pub fn user_table_entry(physical: usize) -> usize {
    physical
}

/// 判断给定层级（`level`）的页表项是否指向由当前地址空间动态分配的真实下级页表（排除叶层与全局静态空目录哨兵）。
pub fn user_entry_is_table(entry: usize, level: usize) -> bool {
    level != 0 && entry != user_empty_entry(level)
}

/// 从目录项或叶表项中提取 4 KiB 对齐的物理页基址，屏蔽全部低位权限与高位不可读/不可执行标志。
pub fn user_entry_physical(entry: usize, _level: usize) -> usize {
    entry & PHYSICAL
}

/// 将物理页基址与访问权限编码为用户态（`PLV3`）叶页表项。
///
/// - 若读、写、执行权限全为 `false`（即 `PROT_NONE`），编码为不带 `VALID`/`PRESENT` 但带有 `PROTNONE | USER | NO_READ | NO_EXEC` 的保留项，
///   使硬件访问触发缺页异常的同时，软件仍能区分“已映射但无权限”与“未映射”。
/// - 若至少包含一项权限，则置位 `VALID | USER | CACHED | PRESENT`；当允许写入时同步置位软件 `WRITE` 与硬件 `DIRTY`；
///   当禁止读取或禁止执行时分别置位 `NO_READ`（Bit 61）或 `NO_EXEC`（Bit 62）。
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

/// 从叶页表项解码用户态读、写、执行权限；若为空表项、目录项或状态位不一致则返回 `None`。
pub fn user_leaf_permissions(entry: usize) -> Option<PagePermissions> {
    // Step 1: 识别显式标记为 PROT_NONE 的无权限用户页。
    if entry & (PROTNONE | USER) == (PROTNONE | USER) {
        return Some(PagePermissions {
            read: false,
            write: false,
            execute: false,
        });
    }
    // Step 2: 校验常规驻留用户叶表项必须同时具备 PRESENT、VALID 与 USER 标志，且软件 WRITE 与硬件 DIRTY 必须一致。
    if entry & (PRESENT | VALID | USER) != (PRESENT | VALID | USER) {
        return None;
    }
    if (entry & WRITE != 0) != (entry & DIRTY != 0) {
        return None;
    }
    // Step 3: 根据 NO_READ、WRITE、NO_EXEC 位还原权限结构。
    Some(PagePermissions {
        read: entry & NO_READ == 0,
        write: entry & WRITE != 0,
        execute: entry & NO_EXEC == 0,
    })
}

/// 清除叶页表项的写权限与脏标志（同时清除 `WRITE` 与 `DIRTY` 位），保留物理页号及读/执行权限，用于写时复制（COW）或页面回写降权。
pub fn user_leaf_mkclean(entry: usize) -> usize {
    entry & !(WRITE | DIRTY)
}

/// 检查叶页表项是否带有硬件脏标志（`DIRTY` 位在授予写权限时一并置位，作为页面可能已被写入的保守判据）。
pub fn user_leaf_dirty(entry: usize) -> bool {
    entry & DIRTY != 0
}

/// 将新分配的用户根页表（Level 2）全部填充为指向永久无效中间表（`INVALID_MIDDLE`）的空目录项。
///
/// 用户页表仅负责 `PLV3` 低半区地址空间，不需要也不共享任何内核态映射（内核始终通过 `DMW` 窗口直接访问）。
///
/// # Safety
/// - `root` 必须指向由调用者独占拥有、可写且按 4 KiB（`PAGE_SIZE`）对齐的一整页物理内存（通过内核 `DMW` 虚拟地址访问）。
/// - 在本函数返回前，该页尚未装入任何 CPU 的 `CSR_PGDL` 或被其他线程并发访问。
pub unsafe fn initialize_user_root(root: *mut usize) {
    assert!(
        INITIALIZED.load(Ordering::Acquire),
        "invalid tables are not initialized"
    );
    for index in 0..PT_ENTRIES {
        // SAFETY:
        // 1. 调用方契约保证 `root` 指向独占可写的 4 KiB 页（共 `PT_ENTRIES = 512` 个 `usize` 槽位），`index < PT_ENTRIES` 不会越界。
        // 2. 前置 `assert!(INITIALIZED)` 保证全局静态无效表已初始化完成，`user_empty_entry(2)` 返回有效的物理哨兵地址。
        unsafe { root.add(index).write(user_empty_entry(2)) };
    }
}

/// 初始化当前 CPU 的页表遍历控制寄存器（`PWCL`/`PWCH`）、TLB 页大小寄存器与默认内核空根表。
///
/// 必须在本地中断关闭（IRQ off）的 `PLV0` 上下文中调用。
pub(super) fn initialize_current_cpu() {
    assert!(
        INITIALIZED.load(Ordering::Acquire),
        "invalid tables are not initialized"
    );
    assert!(!irq_enabled(), "page walker setup requires IRQ off");
    // Step 1: 配置三级页表遍历宽度（Bits [20:12] 为叶表 PT，Bits [29:21] 为中间目录，Bits [38:30] 为根目录）及 4 KiB 页大小。
    // SAFETY:
    // 1. 前置断言已确认 `INITIALIZED` 为真且本地中断已关闭（IRQ off），当前运行在 `PLV0` 内核态。
    // 2. 内核指令、栈与静态数据均通过 `DMW` 窗口访问，修改 `PWCL`/`PWCH`/`STLBPS`/`TLBIDX`/`TLBREHI`/`PGDH` 不会影响当前执行流。
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
    // Step 2: 将低半区页表基址 CSR_PGDL 也指向永久无效根表并清空本地非全局 TLB。
    activate_kernel_root();
}

/// 在当前 CPU 上切换用户根页表（写入 `CSR_PGDL` 并将 `CSR_ASID` 置 0），随后刷新本地所有非全局 TLB 表项。
///
/// # Safety
/// - `root_phys` 必须是 4 KiB 对齐的有效三级用户根页表物理基址。
/// - 调用方必须保证该根页表及其所有可达的下级页表、数据页在切换为其他根页表并完成 TLB 刷新之前始终保持存活（不被释放或挪作他用）。
/// - 当前必须处于 `PLV0` 内核态且本地中断已关闭（IRQ off），内核代码、栈及 `CpuLocal` 均位于 `DMW` 窗口内。
pub unsafe fn activate_user_root(root_phys: usize) {
    assert!(!irq_enabled(), "user root switch requires IRQ off");
    assert_eq!(root_phys & (PAGE_SIZE - 1), 0, "unaligned user root");
    // SAFETY:
    // 1. 前置断言确保本地中断关闭且 `root_phys` 按页对齐；调用方契约保证 `root_phys` 指向存活的有效三级页表根。
    // 2. 当前运行在 `PLV0` 且仅使用 `DMW` 映射，更新 `CSR_PGDL` 与 `CSR_ASID` 不会破坏内核自身寻址。
    unsafe {
        csr_write::<{ reg::CSR_PGDL }>(root_phys);
        csr_write::<{ reg::CSR_ASID }>(0);
    }
    flush_user_translations();
}

/// 将当前 CPU 的用户页表基址（`CSR_PGDL`）切回全局永久无效根表 `LA_INVALID_ROOT`，并刷新本地非全局 TLB。
///
/// 调用者必须处于本地中断关闭（IRQ off）的 `PLV0` 内核态。
pub fn activate_kernel_root() {
    assert!(!irq_enabled(), "kernel root switch requires IRQ off");
    // SAFETY:
    // 1. 前置 `assert!(!irq_enabled())` 保证本地中断已关闭。
    // 2. `LA_INVALID_ROOT` 是静态 4 KiB 对齐且永久存活的无效根表，满足 `activate_user_root` 的对齐与生命周期要求。
    unsafe { activate_user_root(virt_to_phys(addr_of_mut!(LA_INVALID_ROOT) as usize)) };
}

/// 等待此前页表内存写入完成，并清除当前 CPU 上所有非全局（`G=0`）的用户态 TLB 表项（内核 `DMW` 映射不受影响）。
pub fn flush_user_translations() {
    // SAFETY:
    // 1. 当前运行在 `PLV0` 内核态，具备执行 `invtlb` 特权指令的权限。
    // 2. 前置 `dbar 0` 确保此前对内存页表项的修改对硬件可见，`invtlb 0x3` 仅失效非全局 TLB 项，不影响内核 `DMW` 直接映射窗口。
    unsafe {
        asm!("dbar 0", "invtlb {operation}, $zero, $zero", operation = const INVTLB_ALL_NONGLOBAL, options(nostack))
    };
}

/// 将此前写入物理内存的用户态指令数据刷出并同步当前 CPU 的指令缓存（I-Cache）取指流水线。
pub fn sync_user_icache() {
    // SAFETY:
    // 1. `dbar 0` 与 `ibar 0` 为无内存解引用副作用的屏障指令，可在 `PLV0` 安全执行且不触碰栈空间（`nostack`）。
    // 2. `dbar 0` 确保此前对代码页的数据写入全部完成，`ibar 0` 使当前核后续取指能够看到最新写入的指令。
    unsafe { asm!("dbar 0", "ibar 0", options(nostack)) };
}
