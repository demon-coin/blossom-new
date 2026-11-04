//! 主辅核机器入口；建立 DMW 与受信栈后才进入普通 Rust。

use crate::arch::boot_clear_words;
use crate::arch::loongarch64::register as reg;
use crate::cpu::{CpuBoot, BOOT_STACK, STACK_SIZE};
use crate::platform::LINK_LAYOUT;
use core::arch::naked_asm;
use core::mem::offset_of;

extern "C" {
    static __bss_start: u8;
    static __bss_end: u8;
}

/// 保存四个固件参数，建立缓存 RAM 与强序 MMIO 的 DMW 后跳入高地址续接。
///
/// # Safety
/// 只能由固件在 BSP 的 PLV0 引导阶段进入，尚无并行内核访问；镜像和启动栈必须已装载于链接地址对应 RAM。
/// a0–a3 必须保持固件原值，入口不能依赖现有栈、BSS 或 CpuLocal。
#[naked]
#[no_mangle]
#[link_section = ".text.entry.primary"]
unsafe extern "C" fn __primary_entry(_arg0: usize, _arg1: usize, _arg2: usize, _arg3: usize) -> ! {
    // SAFETY: 固件提供独占 BSP 与有效镜像；无栈代码只设置 PLV0 DMW，并以 s0–s3 保留完整参数。
    unsafe {
        naked_asm!(
            r#"
    move $s0, $a0
    move $s1, $a1
    move $s2, $a2
    move $s3, $a3
    li.w $t0, {reg_crmd_ie}
    csrxchg $zero, $t0, {csr_crmd}
    csrwr $zero, {csr_ks0}
    li.d $t0, {direct_base}
    ori $t0, $t0, {reg_dmw_kernel_ram}
    csrwr $t0, {csr_dmw1}
    li.d $t0, {mmio_base}
    ori $t0, $t0, {reg_dmw_plv0}
    csrwr $t0, {csr_dmw0}
    csrwr $zero, {csr_dmw2}
    csrwr $zero, {csr_dmw3}
    la.abs $t0, {high}
    jirl $zero, $t0, 0
"#,
            direct_base = const LINK_LAYOUT.direct_map_virt_base,
            mmio_base = const LINK_LAYOUT.mmio_virt_base,
            high = sym la_primary_high,
            csr_crmd = const reg::CSR_CRMD,
            csr_dmw0 = const reg::CSR_DMW0,
            csr_dmw1 = const reg::CSR_DMW1,
            csr_dmw2 = const reg::CSR_DMW2,
            csr_dmw3 = const reg::CSR_DMW3,
            csr_ks0 = const reg::CSR_KS0,
            reg_crmd_ie = const reg::CRMD_IE,
            reg_dmw_kernel_ram = const reg::DMW_KERNEL_RAM,
            reg_dmw_plv0 = const reg::DMW_PLV0,
        );
    }
}

/// 在缓存高别名建立 BSP 栈，清零 BSS，再将原四参数交给主核 Rust 入口。
///
/// # Safety
/// 只能从主核裸入口跳入；DMW 已有效、IRQ 关闭，s0–s3 是原固件参数，BSS 仍由 BSP 独占。
#[naked]
unsafe extern "C" fn la_primary_high() -> ! {
    // SAFETY: 前驱已建立高地址取指；原启动栈位于 BSS 外，普通 C ABI 调用保持 s0–s3 不变。
    unsafe {
        naked_asm!(
            r#"
    # DATF/DATM 一致缓存保证 DA refill 能看到内核写入的叶。
    li.w $t0, {reg_crmd_boot}
    csrwr $t0, {csr_crmd}
    la.abs $sp, {stack}
    li.d $t0, {stack_size}
    add.d $sp, $sp, $t0
    la.abs $a0, {bss_start}
    la.abs $a1, {bss_end}
    bl {clear_bss}
    move $a0, $s0
    move $a1, $s1
    move $a2, $s2
    move $a3, $s3
    b {primary_main}
"#,
            stack = sym BOOT_STACK,
            stack_size = const STACK_SIZE,
            bss_start = sym __bss_start,
            bss_end = sym __bss_end,
            clear_bss = sym la_clear_bss,
            primary_main = sym crate::primary_main,
            csr_crmd = const reg::CSR_CRMD,
            reg_crmd_boot = const reg::CRMD_BOOT,
        );
    }
}

/// 在 BSP 的高地址启动栈上清空链接器给出的 BSS 半开区间，不访问尚未初始化的全局状态。
///
/// # Safety
/// start/end 必须是同一独占可写 RAM 区间的八字节对齐端点且 start <= end；该区间不得包含当前栈。
unsafe extern "C" fn la_clear_bss(start: usize, end: usize) {
    // SAFETY: BSP 高续接传入真实链接端点，启动栈位于独立的 .boot.stacks，AP 尚未释放。
    unsafe { boot_clear_words(start, end) };
}

/// 从固件 mailbox 物理 PC 建立辅核 DMW，再跳至高地址续接读取永久启动记录。
///
/// # Safety
/// 只能由 BSP 为尚未启动的 AP 释放；mailbox 必须已发表本核 CpuBoot 或板级栈和 CpuLocal，相关资源永久保活。
#[naked]
#[no_mangle]
#[link_section = ".text.entry.secondary"]
unsafe extern "C" fn __secondary_entry() -> ! {
    // SAFETY: BSP 已发表唯一 AP 启动资源；此段不使用现有 SP，也不清 BSS 或重建共享对象。
    unsafe {
        naked_asm!(
            r#"
    li.w $t0, {reg_crmd_ie}
    csrxchg $zero, $t0, {csr_crmd}
    csrwr $zero, {csr_ks0}
    li.d $t0, {direct_base}
    ori $t0, $t0, {reg_dmw_kernel_ram}
    csrwr $t0, {csr_dmw1}
    li.d $t0, {mmio_base}
    ori $t0, $t0, {reg_dmw_plv0}
    csrwr $t0, {csr_dmw0}
    csrwr $zero, {csr_dmw2}
    csrwr $zero, {csr_dmw3}
    la.abs $t0, {high}
    jirl $zero, $t0, 0
"#,
            direct_base = const LINK_LAYOUT.direct_map_virt_base,
            mmio_base = const LINK_LAYOUT.mmio_virt_base,
            high = sym la_secondary_high,
            csr_crmd = const reg::CSR_CRMD,
            csr_dmw0 = const reg::CSR_DMW0,
            csr_dmw1 = const reg::CSR_DMW1,
            csr_dmw2 = const reg::CSR_DMW2,
            csr_dmw3 = const reg::CSR_DMW3,
            csr_ks0 = const reg::CSR_KS0,
            reg_crmd_ie = const reg::CRMD_IE,
            reg_dmw_kernel_ram = const reg::DMW_KERNEL_RAM,
            reg_dmw_plv0 = const reg::DMW_PLV0,
        );
    }
}

/// 在辅核高地址取指后启用 PG，读取板级 mailbox 或原 CpuBoot，再进入本核 Rust 初始化。
///
/// # Safety
/// 只能由辅核入口跳入；DMW 已建立，mailbox 中的栈满足 16 字节 ABI 对齐，CpuLocal 属于本核且已完整初始化。
#[naked]
unsafe extern "C" fn la_secondary_high() -> ! {
    // SAFETY: 前驱保持 IRQ 关闭并建立 RAM 直映；BSP 保活对应启动记录、动态栈和本核 CpuLocal。
    unsafe {
        naked_asm!(
            r#"
    li.w $t0, {reg_crmd_boot}
    csrwr $t0, {csr_crmd}
    .if {board_platform}
    li.w $t0, {reg_iocsr_mailbox1}
    iocsrrd.d $sp, $t0
    li.w $t0, {reg_iocsr_mailbox2}
    iocsrrd.d $a0, $t0
    .else
    li.w $t0, {reg_iocsr_mailbox1}
    iocsrrd.d $t1, $t0
    li.d $t0, {direct_base}
    or $t1, $t1, $t0
    ld.d $sp, $t1, {boot_stack}
    ld.d $a0, $t1, {boot_local}
    .endif
    b {secondary_main}
"#,
            board_platform = const cfg!(blossom_platform = "ls2k1000la") as usize,
            direct_base = const LINK_LAYOUT.direct_map_virt_base,
            boot_stack = const offset_of!(CpuBoot, stack_top),
            boot_local = const offset_of!(CpuBoot, local),
            secondary_main = sym crate::secondary_main,
            csr_crmd = const reg::CSR_CRMD,
            reg_crmd_boot = const reg::CRMD_BOOT,
            reg_iocsr_mailbox1 = const reg::IOCSR_MAILBOX1,
            reg_iocsr_mailbox2 = const reg::IOCSR_MAILBOX2,
        );
    }
}
