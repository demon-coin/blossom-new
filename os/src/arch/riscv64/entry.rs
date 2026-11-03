//! 物理引导与高地址续接；临时页表在 BSP 构建后保持不变，供 AP 进入分页。

use crate::cpu::CpuBoot;
use core::{arch::global_asm, mem::offset_of};

// 固件入口尚无 Rust 栈；清 BSS、构表、启分页的顺序由此片段独占。
global_asm!(
    // 物理 BSP 入口先关闭旧中断与分页，保留固件参数并清理 BSS/临时表。
    r#"
.section .text.boot,"ax",@progbits

.option push
.option norelax
.macro address reg, symbol
    lla \reg, 991f
    ld \reg, 0(\reg)
    j 992f
    .balign 8
991: .quad \symbol
992:
.endm
.balign 4
.global __primary_entry_phys
__primary_entry_phys:
    csrw sie, zero
    csrci sstatus, 2
    csrw satp, zero
    csrw sscratch, zero
    // 清 BSS 与构表只使用临时寄存器，四个固件参数留在 callee-saved 寄存器。
    mv s0, a0
    mv s1, a1
    mv s2, a2
    mv s3, a3
    address t0, __bss_phys_start
    address t1, __bss_phys_end
1:  bgeu t0, t1, 2f
    sd zero, 0(t0)
    addi t0, t0, 8
    j 1b
2:  address t0, __bootstrap_root_phys
    address t1, __bootstrap_image_phys
    mv t2, t0
    mv t4, t1
    li t3, 512
3:  sd zero, 0(t2)
    sd zero, 0(t4)
    addi t2, t2, 8
    addi t4, t4, 8
    addi t3, t3, -1
    bnez t3, 3b
"#,
    // 建立临时低地址、RAM/MMIO 直映及镜像映射后，连续切入高地址代码。
    r#"
    // 临时低映射仅供进入分页的连续取指，永不修改这份 AP 表。
    address t2, __phys_ram_base
    srli t3, t2, 2
    ori t3, t3, 0xcf
    srli t4, t2, 30
    slli t4, t4, 3
    add t4, t0, t4
    sd t3, 0(t4)
    address t4, __direct_base
    srli t4, t4, 30
    andi t4, t4, 511
    slli t4, t4, 3
    add t4, t0, t4
    li t5, 64
4:  sd t3, 0(t4)
    li t6, 0x10000000
    add t3, t3, t6
    addi t4, t4, 8
    addi t5, t5, -1
    bnez t5, 4b
    address t4, __mmio_base
    srli t4, t4, 30
    andi t4, t4, 511
    slli t4, t4, 3
    add t4, t0, t4
    li t3, 0xc7
    sd t3, 0(t4)
    address t4, __kernel_virt_start
    srli t4, t4, 30
    andi t4, t4, 511
    slli t4, t4, 3
    add t4, t0, t4
    srli t3, t1, 2
    ori t3, t3, 1
    sd t3, 0(t4)
    address t2, __kernel_phys_start
    srli t2, t2, 2
    ori t2, t2, 0xcf
    li t3, 512
5:  sd t2, 0(t1)
    li t4, 0x80000
    add t2, t2, t4
    addi t1, t1, 8
    addi t3, t3, -1
    bnez t3, 5b
    fence rw, rw
    srli t0, t0, 12
    li t1, 8
    slli t1, t1, 60
    or t0, t0, t1
    sfence.vma
    csrw satp, t0
    address t0, rv_primary_high
    jr t0
"#,
    // AP 从 HSM 的物理 CpuBoot 取目的核栈与 CpuLocal，再使用不可变 bootstrap 表。
    r#"
.global rv_secondary_entry
rv_secondary_entry:
    csrw sie, zero
    csrci sstatus, 2
    csrw sscratch, zero
    // HSM a1 是 CpuBoot 的物理指针，不是 DTB。
    ld sp, {boot_stack}(a1)
    ld tp, {boot_local}(a1)
    address t0, __bootstrap_root_phys
    srli t0, t0, 12
    li t1, 8
    slli t1, t1, 60
    or t0, t0, t1
    sfence.vma
    csrw satp, t0
    address t0, rv_secondary_high
    jr t0
.option pop
.text

"#,
    // 高地址 BSP 建立首个 Rust 栈；AP 在访问动态栈前换入已发布的最终 SATP。
    r#"
.balign 4
rv_primary_high:
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop
    la sp, BOOT_STACK
    li t0, {stack_size}
    add sp, sp, t0
    mv tp, zero
    mv a0, s0
    mv a1, s1
    mv a2, s2
    mv a3, s3
    tail primary_main
rv_secondary_high:
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop
    la t0, RV_FINAL_SATP
    ld t0, 0(t0)
    csrw satp, t0
    sfence.vma
    mv a0, tp
    tail secondary_main
"#,
    boot_stack = const offset_of!(CpuBoot, stack_top),
    boot_local = const offset_of!(CpuBoot, local),
    stack_size = const crate::cpu::STACK_SIZE,
);
