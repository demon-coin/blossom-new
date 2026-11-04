//! 物理引导与高地址续接；临时页表在 BSP 构建后保持不变，供 AP 进入分页。

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

/// 在物理栈上清零 BSP 独占内存，并建立供 BSP/AP 共用的 Sv39 临时映射。
///
/// # Safety
/// 必须在关闭 IRQ、SATP 为零的唯一 BSP 上调用；四个地址均为当前可写物理 RAM。
/// BSS 端点按机器字对齐且有序，root/image 各指向一张独占的完整对齐页表；
/// 三个区域互不重叠，且不包含本次调用的 BOOT_STACK、代码与保存区。
/// 布局输入必须满足链接器约束，所有表项和地址推进均落在对应的完整对象内。
/// 代码按高地址链接以保持展开记录可表示；分页前只能从物理别名执行，
/// 不得引用高地址实体、普通 BSS、TLS 或其他辅助函数。
#[inline(never)]
#[no_mangle]
#[link_section = ".text.boot.init"]
unsafe extern "C" fn rv_initialize_boot_memory(
    bss_start: usize,
    bss_end: usize,
    root: usize,
    image: usize,
) -> usize {
    // 物理代码不能物化高地址的布局结构体；每个字段在编译期折叠为整数。
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
    // 临时 RAM 叶保持 V/R/W/X/A/D；MMIO 叶只去掉 X，不改变原硬件属性。
    const RAM_LEAF_FLAGS: usize = KERNEL_DATA_FLAGS | PTE_EXEC | PTE_VALID;
    const MMIO_LEAF_FLAGS: usize = KERNEL_DATA_FLAGS | PTE_VALID;

    // SAFETY: 调用者提供独占、对齐且互不重叠的 BSS 与两张完整物理页表，活动栈在区间外。
    unsafe {
        boot_clear_words(bss_start, bss_end);
        boot_clear_words(root, root.wrapping_add(TABLE_BYTES));
        boot_clear_words(image, image.wrapping_add(TABLE_BYTES));
    }

    let ram_leaf = (RAM >> PHYSICAL_TO_PTE_SHIFT) | RAM_LEAF_FLAGS;
    // SAFETY: 固定 RAM 布局的低映射索引在 root 内，页表尚未交给硬件或 AP。
    unsafe {
        boot_store_word(
            root.wrapping_add((RAM >> PGDIR_SHIFT).wrapping_mul(WORD_BYTES)),
            ram_leaf,
        )
    };
    let mut index = 0usize;
    while index < DIRECT_LEAVES {
        let slot = DIRECT_SLOT.wrapping_add(index).wrapping_mul(WORD_BYTES);
        let leaf =
            ram_leaf.wrapping_add(index.wrapping_mul(GIB_LEAF_BYTES >> PHYSICAL_TO_PTE_SHIFT));
        // SAFETY: 固定直映起槽和 64 个连续槽均在 root 内，完整表由 BSP 独占。
        unsafe { boot_store_word(root.wrapping_add(slot), leaf) };
        index = index.wrapping_add(1);
    }
    // SAFETY: 固定 MMIO/镜像槽均在 root 内；image 是完整、已清零且未发布的物理页表。
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
    index = 0;
    while index < PT_ENTRIES {
        let leaf = ((IMAGE >> PHYSICAL_TO_PTE_SHIFT) | RAM_LEAF_FLAGS)
            .wrapping_add(index.wrapping_mul(PMD_SIZE >> PHYSICAL_TO_PTE_SHIFT));
        // SAFETY: index 严格小于 512，每次完整字存储均在 BSP 独占的 image 页内。
        unsafe { boot_store_word(image.wrapping_add(index.wrapping_mul(WORD_BYTES)), leaf) };
        index = index.wrapping_add(1);
    }
    SATP_MODE_SV39 | (root >> PAGE_SHIFT)
}

/// 从固件物理入口建立 BSP 栈，调用物理初始化后连续切换到高地址。
///
/// # Safety
/// 固件必须在唯一 BSP 的 S-mode 进入，a0–a3 保存四个原始固件参数；
/// 镜像与链接保留 RAM 已装载且可写，BOOT_STACK 未被其他执行者使用。
#[naked]
#[no_mangle]
#[link_section = ".text.boot.entry"]
unsafe extern "C" fn __primary_entry_phys(
    _arg0: usize,
    _arg1: usize,
    _arg2: usize,
    _arg3: usize,
) -> ! {
    // SAFETY: 本入口独占 BSP 机器状态；本地 literal 只读取链接地址，物理栈不在 BSS 清零区。
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

/// 从 HSM 的物理 CpuBoot 读取目的核资源，仅借 BSP 已完成的临时页表进入高地址。
///
/// # Safety
/// BSP 必须已发布不可变 bootstrap 表和 a1 指向的 CpuBoot；其中栈与 CpuLocal
/// 属于当前 AP 且在整个内核生命周期有效。动态栈在安装最终 SATP 前不得访问。
#[naked]
#[no_mangle]
#[link_section = ".text.boot.secondary"]
unsafe extern "C" fn rv_secondary_entry(_hardware_id: usize, _record: *const CpuBoot) -> ! {
    // SAFETY: HSM 提供有效物理记录；ld sp 只装载栈地址，不在尚未映射的动态栈上访存。
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

/// 在镜像高地址恢复 BSP 的 GP、原静态栈和四个固件参数后进入普通 Rust。
///
/// # Safety
/// 只能由物理 BSP 入口在 bootstrap SATP 生效后跳入，s0–s3 仍持原固件参数。
#[naked]
#[no_mangle]
unsafe extern "C" fn rv_primary_high() -> ! {
    // SAFETY: bootstrap 已覆盖当前代码及 BOOT_STACK；物理初始化已退栈，可重置同一栈的高别名。
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

/// 在首次动态栈访存前安装最终页表，再以目的 CPU 的 TP 进入 AP Rust 初始化。
///
/// # Safety
/// 只能由 AP 物理入口跳入；SP/TP 来自该 AP 的永久 CpuBoot，最终 SATP 已由 BSP 发布。
#[naked]
#[no_mangle]
unsafe extern "C" fn rv_secondary_high() -> ! {
    // SAFETY: 最终 SATP 所在镜像由 bootstrap 可读；切表和 sfence 后目的栈与 CpuLocal 才供 Rust 使用。
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
