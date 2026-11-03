//! 主辅核机器入口；建立 DMW 与受信栈后才进入 Rust。

use core::arch::global_asm;
use core::mem::offset_of;

global_asm!(
    // 主核先保留固件参数，再建立缓存与 MMIO 的高地址别名。
    r#"
.section .text.entry,"ax",@progbits

.balign 64
.global __primary_entry
__primary_entry:
    # 清 BSS 与建立 DMW 前保存完整固件参数，CPU 身份另从 CSR 读取。
    move $s0, $a0
    move $s1, $a1
    move $s2, $a2
    move $s3, $a3
    li.w $t0, 4
    csrxchg $zero, $t0, 0
    # KS0 的零值表示尚未安装本核 CpuLocal。
    csrwr $zero, 0x30
    la.abs $t0, __direct_base
    ori $t0, $t0, 0x11
    csrwr $t0, 0x181
    la.abs $t0, __mmio_base
    ori $t0, $t0, 1
    csrwr $t0, 0x180
    csrwr $zero, 0x182
    csrwr $zero, 0x183
    # 高别名取指前先建立 DMW，兼容固件仍处于分页模式的物理入口。
    la.abs $t0, .Lprimary_high
    jirl $zero, $t0, 0
"#,
    // 高地址取指后切 PG、清 BSS，并将原四个参数交还主核入口。
    r#"
.Lprimary_high:
    # DATF/DATM 取一致缓存：refill 在 DA 下读页表必须看到内核 DMW 写回的叶。
    li.w $t0, 0xb0
    csrwr $t0, 0
    la.abs $sp, BOOT_STACK
    li.d $t0, {stack_size}
    add.d $sp, $sp, $t0
    la.abs $t0, __bss_start
    la.abs $t1, __bss_end
1:
    bgeu $t0, $t1, 2f
    st.d $zero, $t0, 0
    addi.d $t0, $t0, 8
    b 1b
2:
    move $a0, $s0
    move $a1, $s1
    move $a2, $s2
    move $a3, $s3
    b primary_main

"#,
    // 辅核从 mailbox 物理 PC 起步，先建立与主核相同的 DMW。
    r#"
.balign 64
.global __secondary_entry
__secondary_entry:
    li.w $t0, 4
    csrxchg $zero, $t0, 0
    # 辅核安装 CpuLocal 前也保持可检查的空基址。
    csrwr $zero, 0x30
    la.abs $t0, __direct_base
    ori $t0, $t0, 0x11
    csrwr $t0, 0x181
    la.abs $t0, __mmio_base
    ori $t0, $t0, 1
    csrwr $t0, 0x180
    csrwr $zero, 0x182
    csrwr $zero, 0x183
    # mailbox 指向物理 PC；高别名建立以后才切换 CRMD 的分页状态。
    la.abs $t0, .Lsecondary_high
    jirl $zero, $t0, 0
"#,
    // 板平台直接读栈与 CpuLocal；QEMU 从永久 CpuBoot 记录读取。
    r#"
.Lsecondary_high:
    # DATF/DATM 取一致缓存：refill 在 DA 下读页表必须看到内核 DMW 写回的叶。
    li.w $t0, 0xb0
    csrwr $t0, 0
    .if {board_platform}
    li.w $t0, 0x1028
    iocsrrd.d $sp, $t0
    li.w $t0, 0x1030
    iocsrrd.d $a0, $t0
    .else
    li.w $t0, 0x1028
    iocsrrd.d $t1, $t0
    la.abs $t0, __direct_base
    or $t1, $t1, $t0
    ld.d $sp, $t1, {boot_stack}
    ld.d $a0, $t1, {boot_local}
    .endif
    b secondary_main
"#,
    stack_size = const crate::cpu::STACK_SIZE,
    boot_stack = const offset_of!(crate::cpu::CpuBoot, stack_top),
    boot_local = const offset_of!(crate::cpu::CpuBoot, local),
    board_platform = const cfg!(blossom_platform = "ls2k1000la") as usize,
);
