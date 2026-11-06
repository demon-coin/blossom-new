//! RISC-V 64 位的物理地址引导入口、临时 Sv39 页表构建与高半核虚拟地址续接。
//!
//! # 模块职责与架构定位
//! 本模块承载内核在开启 MMU 前后的第一阶段引导代码：
//! - **主核（BSP）**：固件（OpenSBI / U-Boot）跳转到物理入口 [`__primary_entry_phys`] 后，调用
//!   [`rv_initialize_boot_memory`] 清零 `.bss` 段并填写临时启动页表（`RV_BOOT_ROOT` 与 `RV_BOOT_IMAGE`），
//!   随后写入 `satp` 开启 Sv39 分页并跳转至高地址入口 [`rv_primary_high`]，最终进入 [`crate::primary_main`]。
//! - **从核（AP）**：由主核通过 SBI HSM 唤醒，从物理入口 [`rv_secondary_entry`] 进入，先借用主核已构建完毕且
//!   保持只读的临时启动页表跳入高地址入口 [`rv_secondary_high`]，再立即切换为主核已发布的最终内核页表
//!   （`RV_FINAL_SATP`），随后进入 [`crate::secondary_main`]。
//!
//! # 核心机制：两阶段启动页表与虚拟地址过渡布局
//! 在开启分页的那一条 `csrw satp` 指令前后，CPU 的 PC 仍指向物理地址，而下一条跳转目标已是高半核虚拟地址。
//! 因此临时根页表 `RV_BOOT_ROOT` 必须同时包含**恒等映射（Identity Mapping）**与**高半核映射**：
//!
//! ```text
//! 临时启动页表 (RV_BOOT_ROOT + RV_BOOT_IMAGE) 映射布局
//! +----------------------------------+-----------------------------------+-------------------+
//! | 虚拟地址范围                     | 物理地址目标                      | 页表粒度与权限    |
//! +----------------------------------+-----------------------------------+-------------------+
//! | [ram_phys_base, +1GiB)           | [ram_phys_base, +1GiB) (恒等映射) | L2 1GiB 大页 RWX  |
//! | [mmio_virt_base, +1GiB)          | [0x0, +1GiB) (早期低端外设窗口)   | L2 1GiB 大页 RW-  |
//! | [direct_map_virt_base, +64GiB)   | [ram_phys_base, +64GiB) (RAM直映) | 64 个 L2 1GiB RWX |
//! | [kernel_link_virt, +1GiB)        | 指向二级表 RV_BOOT_IMAGE:         | L2 -> L1 表项     |
//! |   └─ 512 x 2MiB 连续槽位         | [kernel_load_phys, +1GiB)         | 512 个 L1 2MiB RWX|
//! +----------------------------------+-----------------------------------+-------------------+
//!
//! 主核 (BSP) 与从核 (AP) 引导控制流：
//!   [OpenSBI 固件]                           [SBI HSM 唤醒从核]
//!         | (MMU Off, a0..a3=固件参数)               | (MMU Off, a1=*CpuBoot 物理地址)
//!         v                                          v
//! __primary_entry_phys                      rv_secondary_entry
//!   ├─ 关中断，设物理 BOOT_STACK              ├─ 关中断，从 CpuBoot 读 sp/tp (高地址值)
//!   ├─ 调 rv_initialize_boot_memory           ├─ 装填 RV_BOOT_ROOT 至 satp 并 sfence.vma
//!   │    ├─ 清零 .bss 与两张临时页表          └─ jr -> rv_secondary_high
//!   │    └─ 填恒等映射 + 直映 + 镜像映射                   |
//!   ├─ 装填临时 satp 并 sfence.vma                         v
//!   └─ jr -> rv_primary_high                 rv_secondary_high
//!                |                             ├─ 设 gp，加载 RV_FINAL_SATP 切换最终页表
//!                v                             ├─ sfence.vma (此后动态分配的 AP 栈可安全访问)
//!         rv_primary_high                      └─ tail -> secondary_main(tp)
//!           ├─ 设 gp，切高地址 BOOT_STACK
//!           └─ tail -> primary_main(a0..a3)
//! ```
//!
//! # 锁与并发契约
//! - `rv_initialize_boot_memory` 仅在主核单核运行、从核尚未启动时执行一次；临时页表一旦写入 `satp`，
//!   在内核整个生命周期内保持只读，供后续所有被唤醒的从核无锁共享使用。
//! - 从核在 `rv_secondary_entry` 中虽然将高地址栈指针装入 `sp`，但在 `rv_secondary_high` 完成最终页表
//!   （`RV_FINAL_SATP`）切换与 `sfence.vma` 之前，**严禁发生任何栈内存读写**。

use crate::arch::{
    boot_clear_words, boot_store_word,
    riscv64::{
        PageTable, KERNEL_DATA_FLAGS, PAGE_SHIFT, PGDIR_SHIFT, PHYSICAL_TO_PTE_SHIFT, PMD_SIZE,
        PTE_EXEC, PTE_VALID, PT_ENTRIES, PT_INDEX_MASK, RV_FINAL_SATP, SATP_MODE_SHIFT,
        SATP_MODE_SV39, SR_SIE,
    },
};
use crate::cpu::{CpuBoot, BOOT_STACK, STACK_SIZE};
use crate::platform::LINK_LAYOUT;
use crate::{primary_main, secondary_main};
use core::{
    arch::naked_asm,
    mem::{offset_of, size_of},
};

extern "C" {
    static __bss_phys_start: u8;
    static __bss_phys_end: u8;
    static __bootstrap_root_phys: u8;
    static __bootstrap_image_phys: u8;
    static __bootstrap_init_phys: u8;
    #[link_name = "__global_pointer$"]
    static GLOBAL_POINTER: u8;
}

/// 在主核物理栈上清零 `.bss` 段与启动页表缓冲区，并构建供主核与从核共用的 Sv39 临时页表。
///
/// 返回可直接写入 `satp` 寄存器的 64 位值（`MODE = Sv39` | `PPN = root >> 12`）。
///
/// # Safety
/// 1. 必须在本地中断已关闭（`sstatus.SIE == 0`）、MMU 尚未开启（`satp == 0`）的唯一主核（BSP）上调用一次。
/// 2. `bss_start..bss_end`、`root` 与 `image` 必须全部位于当前可直接读写的物理 RAM 中：
///    - `bss_start` 与 `bss_end` 按机器字（8 字节）对齐且满足 `bss_start <= bss_end`；
///    - `root` 与 `image` 各自指向一张 4KiB 对齐、大小为 `size_of::<PageTable>()` 的独占物理页；
///    - 上述三个物理区间互不重叠，且均不与当前正在使用的物理 `BOOT_STACK`、引导代码段或固件保留区重叠。
/// 3. 本函数编译后按高虚拟地址链接，但在开启分页前通过物理地址别名执行；函数体内严禁直接访问高地址静态变量、
///    普通 `.bss` 全局对象、线程本地存储（TLS）或未经物理重定位的辅助函数。
#[inline(never)]
#[no_mangle]
#[link_section = ".text.boot.init"]
unsafe extern "C" fn rv_initialize_boot_memory(
    bss_start: usize,
    bss_end: usize,
    root: usize,
    image: usize,
) -> usize {
    // 物理地址执行阶段不能解引用高虚拟地址处的结构体符号；此处将布局常量在编译期折叠为立即数。
    const RAM: usize = LINK_LAYOUT.ram_phys_base as usize;
    const DIRECT_SLOT: usize =
        ((LINK_LAYOUT.direct_map_virt_base >> PGDIR_SHIFT) & PT_INDEX_MASK as u64) as usize;
    const MMIO_SLOT: usize =
        ((LINK_LAYOUT.mmio_virt_base >> PGDIR_SHIFT) & PT_INDEX_MASK as u64) as usize;
    const KERNEL_SLOT: usize =
        ((LINK_LAYOUT.kernel_link_virt >> PGDIR_SHIFT) & PT_INDEX_MASK as u64) as usize;
    const IMAGE: usize = LINK_LAYOUT.kernel_load_phys as usize;
    const TABLE_BYTES: usize = size_of::<PageTable>();
    const WORD_BYTES: usize = size_of::<usize>();
    const DIRECT_LEAVES: usize = 64;
    const GIB_LEAF_BYTES: usize = 1 << PGDIR_SHIFT;
    // 临时 RAM 叶表项同时置位 V/R/W/X/A/D；临时 MMIO 叶表项去掉 X（可执行）权限。
    const RAM_LEAF_FLAGS: usize = KERNEL_DATA_FLAGS | PTE_EXEC | PTE_VALID;
    const MMIO_LEAF_FLAGS: usize = KERNEL_DATA_FLAGS | PTE_VALID;

    // Step 1: 清零主核 .bss 段以及两张临时启动页表（root 根表与 image 二级表）。
    // SAFETY: 调用方契约保证了 `bss_start..bss_end`、`root` 与 `image` 均按字对齐、互不重叠且不覆盖当前活动栈。
    unsafe {
        boot_clear_words(bss_start, bss_end);
        boot_clear_words(root, root.wrapping_add(TABLE_BYTES));
        boot_clear_words(image, image.wrapping_add(TABLE_BYTES));
    }

    // Step 2: 在根页表中建立 RAM 起始 1GiB 的恒等映射，确保 `csrw satp` 开启分页后的下一条物理取指不断流。
    let ram_leaf = (RAM >> PHYSICAL_TO_PTE_SHIFT) | RAM_LEAF_FLAGS;
    // SAFETY: `RAM >> PGDIR_SHIFT` 落在 `0..512` 范围内，`root` 由主核独占且尚未写入 `satp`。
    unsafe {
        boot_store_word(
            root.wrapping_add((RAM >> PGDIR_SHIFT).wrapping_mul(WORD_BYTES)),
            ram_leaf,
        )
    };
    // Step 3: 在根页表中建立高半核连续 64GiB 的物理内存线性直映窗口（64 个 1GiB L2 大页）。
    let mut index = 0usize;
    while index < DIRECT_LEAVES {
        let slot = DIRECT_SLOT.wrapping_add(index).wrapping_mul(WORD_BYTES);
        let leaf =
            ram_leaf.wrapping_add(index.wrapping_mul(GIB_LEAF_BYTES >> PHYSICAL_TO_PTE_SHIFT));
        // SAFETY: `DIRECT_SLOT + index` 落在根页表的 512 个槽位之内，`root` 由当前主核独占。
        unsafe { boot_store_word(root.wrapping_add(slot), leaf) };
        index = index.wrapping_add(1);
    }
    // Step 4: 在根页表中映射早期 MMIO 窗口（1GiB 大页），并将内核镜像高地址槽位指向二级表 `image`。
    // SAFETY: `MMIO_SLOT` 与 `KERNEL_SLOT` 均为编译期确定的合法根表索引（`< 512`），`image` 已在 Step 1 清零。
    unsafe {
        boot_store_word(
            root.wrapping_add(MMIO_SLOT.wrapping_mul(WORD_BYTES)),
            MMIO_LEAF_FLAGS,
        );
        boot_store_word(
            root.wrapping_add(KERNEL_SLOT.wrapping_mul(WORD_BYTES)),
            (image >> PHYSICAL_TO_PTE_SHIFT) | PTE_VALID,
        );
    }
    // Step 5: 填充内核镜像二级表 `image` 的全部 512 个 2MiB 大页表项，覆盖从 `IMAGE` 开始的 1GiB 物理空间。
    index = 0;
    while index < PT_ENTRIES {
        let leaf = ((IMAGE >> PHYSICAL_TO_PTE_SHIFT) | RAM_LEAF_FLAGS)
            .wrapping_add(index.wrapping_mul(PMD_SIZE >> PHYSICAL_TO_PTE_SHIFT));
        // SAFETY: 循环条件保证 `index < 512`，写入偏移严格落在 `image` 指向的 4KiB 物理页内。
        unsafe { boot_store_word(image.wrapping_add(index.wrapping_mul(WORD_BYTES)), leaf) };
        index = index.wrapping_add(1);
    }
    SATP_MODE_SV39 | (root >> PAGE_SHIFT)
}

/// 主核（BSP）物理地址启动入口：初始化物理栈与临时页表，开启 Sv39 分页后跳转到高虚拟地址。
///
/// # Safety
/// 1. 必须由平台固件（如 OpenSBI）在唯一主核的 S-mode 下跳入，且 `a0`–`a3` 保存固件传入的原始启动参数（如 hart ID 与 DTB 物理地址）。
/// 2. 内核镜像与链接脚本声明的保留物理内存必须已完整加载且可读写，`BOOT_STACK` 尚未被任何其他核使用。
#[naked]
#[no_mangle]
#[link_section = ".text.boot.entry"]
unsafe extern "C" fn __primary_entry_phys(
    _arg0: usize,
    _arg1: usize,
    _arg2: usize,
    _arg3: usize,
) -> ! {
    // SAFETY:
    // 1. 当前处于主核单核 S-mode 引导起点，首先关闭 `sie`、`sstatus.SIE`、`satp` 与 `sscratch`。
    // 2. 通过 PC 相对寻址（`lla t0, 2f`）从代码段本地文字池加载 `BOOT_STACK`、`.bss` 与临时页表的物理地址，
    //    调用 `rv_initialize_boot_memory` 完成清零与建表后，执行 `fence rw, rw` 与 `sfence.vma` 同步页表写入并开启分页。
    unsafe {
        naked_asm!(
            ".option push",
            ".option norelax",
            "csrw sie, zero",
            "csrci sstatus, {sie}",
            "csrw satp, zero",
            "csrw sscratch, zero",
            "mv s0, a0",
            "mv s1, a1",
            "mv s2, a2",
            "mv s3, a3",
            "lla t0, 2f",
            "ld sp, 0(t0)",
            "ld a0, 8(t0)",
            "ld a1, 16(t0)",
            "ld a2, 24(t0)",
            "ld a3, 32(t0)",
            "li t1, {stack_size}",
            "add sp, sp, t1",
            "ld t1, 48(t0)",
            "jalr ra, t1, 0",
            "fence rw, rw",
            "sfence.vma",
            "csrw satp, a0",
            "lla t0, 2f",
            "ld t0, 40(t0)",
            "jr t0",
            ".balign 8",
            "2:",
            ".quad {boot_stack} + {physical_offset}",
            ".quad {bss_start}",
            ".quad {bss_end}",
            ".quad {root}",
            ".quad {image}",
            ".quad {high}",
            ".quad {initialize_phys}",
            ".option pop",
            boot_stack = sym BOOT_STACK,
            physical_offset = const LINK_LAYOUT.kernel_load_phys.wrapping_sub(LINK_LAYOUT.kernel_link_virt),
            bss_start = sym __bss_phys_start,
            bss_end = sym __bss_phys_end,
            root = sym __bootstrap_root_phys,
            image = sym __bootstrap_image_phys,
            high = sym rv_primary_high,
            initialize_phys = sym __bootstrap_init_phys,
            stack_size = const STACK_SIZE,
            sie = const SR_SIE,
        );
    }
}

/// 从核（AP）物理地址启动入口：从 `CpuBoot` 读取本核栈与 `CpuLocal`，借用临时启动页表跳入高地址。
///
/// # Safety
/// 1. 只能由 SBI HSM `hart_start` 在目标从核的 S-mode 下跳入，`a0` 为硬件 hart ID，`a1` 为 [`CpuBoot`] 的物理地址。
/// 2. 主核必须已完成临时启动页表（`__bootstrap_root_phys`）的构建并保持其不可变，同时已发布 `a1` 指向的 [`CpuBoot`] 记录。
/// 3. `CpuBoot` 中的 `stack_top` 与 `local` 必须由当前从核独占且在内核生命周期内持续有效；
///    在跳转至 `rv_secondary_high` 切换为最终页表（`RV_FINAL_SATP`）前，严禁解引用动态分配的从核栈。
#[naked]
#[no_mangle]
#[link_section = ".text.boot.secondary"]
unsafe extern "C" fn rv_secondary_entry(_hardware_id: usize, _record: *const CpuBoot) -> ! {
    // SAFETY:
    // 1. 主核在调用 `start_cpu` 前已执行 `fence rw, rw`，保证 `a1` 指向的物理 `CpuBoot` 记录及临时页表对当前从核可见。
    // 2. `ld sp` 与 `ld tp` 仅将高虚拟地址装入寄存器，在进入 `rv_secondary_high` 完成最终页表切换前不进行任何栈访存。
    unsafe {
        naked_asm!(
            ".option push",
            ".option norelax",
            "csrw sie, zero",
            "csrci sstatus, {sie}",
            "csrw sscratch, zero",
            "ld sp, {boot_stack}(a1)",
            "ld tp, {boot_local}(a1)",
            "lla t0, 2f",
            "ld t0, 0(t0)",
            "srli t0, t0, {page_shift}",
            "li t1, {satp_mode}",
            "slli t1, t1, {satp_mode_shift}",
            "or t0, t0, t1",
            "sfence.vma",
            "csrw satp, t0",
            "lla t0, 2f",
            "ld t0, 8(t0)",
            "jr t0",
            ".balign 8",
            "2:",
            ".quad {root}",
            ".quad {high}",
            ".option pop",
            boot_stack = const offset_of!(CpuBoot, stack_top),
            boot_local = const offset_of!(CpuBoot, local),
            root = sym __bootstrap_root_phys,
            high = sym rv_secondary_high,
            sie = const SR_SIE,
            page_shift = const PAGE_SHIFT,
            satp_mode = const SATP_MODE_SV39 >> SATP_MODE_SHIFT,
            satp_mode_shift = const SATP_MODE_SHIFT,
        );
    }
}

/// 主核（BSP）高虚拟地址续接入口：初始化全局指针 `gp`、切换到高地址引导栈并尾调用 [`primary_main`]。
///
/// # Safety
/// 只能由 [`__primary_entry_phys`] 在临时启动页表（`RV_BOOT_ROOT`）已写入 `satp` 且生效后跳入，
/// 且进入时 `s0`–`s3` 仍保存着固件传入的四个原始启动参数。
#[naked]
#[no_mangle]
unsafe extern "C" fn rv_primary_high() -> ! {
    // SAFETY:
    // 1. 临时启动页表已映射内核高地址镜像段与 `BOOT_STACK` 的高地址别名。
    // 2. 物理阶段的 `rv_initialize_boot_memory` 早已返回，当前栈帧不再需要保留，因此可直接将 `sp` 重置为高地址 `BOOT_STACK` 栈顶。
    unsafe {
        naked_asm!(
            ".option push",
            ".option norelax",
            "la gp, {gp}",
            "la sp, {stack}",
            "li t0, {stack_size}",
            "add sp, sp, t0",
            "mv tp, zero",
            "mv a0, s0",
            "mv a1, s1",
            "mv a2, s2",
            "mv a3, s3",
            "tail {main}",
            ".option pop",
            gp = sym GLOBAL_POINTER,
            stack = sym BOOT_STACK,
            stack_size = const STACK_SIZE,
            main = sym primary_main,
        );
    }
}

/// 从核（AP）高虚拟地址续接入口：切换至最终内核页表（`RV_FINAL_SATP`），随后尾调用 [`secondary_main`]。
///
/// # Safety
/// 只能由 [`rv_secondary_entry`] 在临时启动页表生效后跳入；进入时 `sp` 与 `tp` 必须已装载当前从核独占的栈顶与 `CpuLocal` 高地址，
/// 且主核必须已完成 `RV_FINAL_SATP` 的构建与发布。
#[naked]
#[no_mangle]
unsafe extern "C" fn rv_secondary_high() -> ! {
    // SAFETY:
    // 1. `RV_FINAL_SATP` 位于内核镜像数据段，在临时启动页表下可直接通过高地址读取。
    // 2. 执行 `csrw satp` 与 `sfence.vma` 激活最终页表后，从核的动态栈（`sp`）与 `CpuLocal`（`tp`）均已建立完整映射，
    //    此后尾调用 `secondary_main` 进行常规栈操作与 Rust 初始化是安全的。
    unsafe {
        naked_asm!(
            ".option push",
            ".option norelax",
            "la gp, {gp}",
            "la t0, {satp}",
            "ld t0, 0(t0)",
            "csrw satp, t0",
            "sfence.vma",
            "mv a0, tp",
            "tail {main}",
            ".option pop",
            gp = sym GLOBAL_POINTER,
            satp = sym RV_FINAL_SATP,
            main = sym secondary_main,
        );
    }
}
