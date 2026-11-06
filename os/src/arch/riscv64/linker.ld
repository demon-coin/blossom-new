OUTPUT_ARCH(riscv)
ENTRY(__primary_entry_phys)
INCLUDE platform-layout.ld
/*
 * RISC-V 64 (Sv39) 内核链接脚本。
 *
 * 内存布局与地址空间约定：
 * - 基础物理页大小（__page_size）为 4KiB，每级页表索引占 9 位（512 项）。
 * - 二级大页（PMD / Mega Page，__pmd_size）为 2MiB；一级目录页（PGDIR / Giga Page，__pgdir_size）为 1GiB。
 * - `.boot` 段保持物理地址直接执行（VMA == LMA == __kernel_phys_start），完成启动页表构建后再跳转到高半核虚拟地址（__kernel_virt_start）。
 * - 整个内核镜像（代码、只读数据、读写数据、启动栈与页表、BSS）按 2MiB 叶项映射，总大小必须限制在单张 L1 页表（1GiB 覆盖范围，即 `RV_BOOT_IMAGE` 的 512 个 2MiB 槽位）之内。
 */
HIDDEN(__page_size = 4096);
HIDDEN(__pt_index_bits = 9);
HIDDEN(__pmd_size = __page_size << __pt_index_bits);
HIDDEN(__pgdir_size = __pmd_size << __pt_index_bits);
PHDRS { boot PT_LOAD FLAGS(5); text PT_LOAD FLAGS(5); rodata PT_LOAD FLAGS(4); data PT_LOAD FLAGS(6); }
SECTIONS {
  . = __kernel_phys_start;
  .boot : {
    KEEP(*(.text.boot.entry))
    KEEP(*(.text.boot.secondary))
  } :boot
  . = __kernel_virt_start + ALIGN(SIZEOF(.boot), __pmd_size);
  .text : AT(ADDR(.text) - __kernel_virt_start + __kernel_phys_start) {
    PROVIDE(__executable_start = .);
    . = ALIGN(4);
    KEEP(*(.text.trap.entry))
    KEEP(*(.text.boot.init))
    *(.text .text.*)
    PROVIDE(__etext = .);
  } :text
  . = ALIGN(__page_size);
  .rodata : AT(ADDR(.rodata) - __kernel_virt_start + __kernel_phys_start) {
    PROVIDE(__rodata_start = .);
    *(.rodata .rodata.*) *(.data.rel.ro .data.rel.ro.*)
  } :rodata
  INCLUDE ktest-sections.ld
  PROVIDE(__rodata_end = .);
  . = ALIGN(__page_size);
  .data : AT(ADDR(.data) - __kernel_virt_start + __kernel_phys_start) {
    *(.data .data.*) *(.sdata .sdata.*)
    PROVIDE(__global_pointer$ = . + 0x800);
  } :data
  . = ALIGN(__page_size);
  __boot_reserved_phys_start = . - __kernel_virt_start + __kernel_phys_start;
  .boot.resources (NOLOAD) : AT(ADDR(.boot.resources) - __kernel_virt_start + __kernel_phys_start) {
    *(.boot.stacks) . = ALIGN(__page_size); *(.boot.tables)
  } :data
  __boot_reserved_phys_end = . - __kernel_virt_start + __kernel_phys_start;
  . = ALIGN(16);
  __bss_start = .;
  .bss (NOLOAD) : AT(ADDR(.bss) - __kernel_virt_start + __kernel_phys_start) { *(.bss .bss.*) *(.sbss .sbss.*) *(COMMON) }
  . = ALIGN(16);
  __bss_end = .;
  __kernel_virt_end = .;
  __kernel_phys_end = . - __kernel_virt_start + __kernel_phys_start;
  __bootstrap_root_phys = RV_BOOT_ROOT - __kernel_virt_start + __kernel_phys_start;
  __bootstrap_image_phys = RV_BOOT_IMAGE - __kernel_virt_start + __kernel_phys_start;
  __bootstrap_init_phys = rv_initialize_boot_memory - __kernel_virt_start + __kernel_phys_start;
  __bss_phys_start = __bss_start - __kernel_virt_start + __kernel_phys_start;
  __bss_phys_end = __bss_end - __kernel_virt_start + __kernel_phys_start;
  __secondary_entry_phys = rv_secondary_entry;
}
ASSERT(SIZEOF(.boot) <= __pmd_size, "physical entry exceeds bootstrap section")
ASSERT((rv_trap_entry & 3) == 0, "kernel trap entry must be 4-byte aligned")
ASSERT(__kernel_phys_end - __kernel_phys_start < __pgdir_size, "image exceeds bootstrap mapping")
