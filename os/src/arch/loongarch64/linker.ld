OUTPUT_ARCH(loongarch)
ENTRY(__primary_entry)
INCLUDE platform-layout.ld
HIDDEN(SECONDARY_ENTRY_ALIGNMENT = 64);
HIDDEN(BOOT_CLEAR_WORD_BYTES = 8);
/*
 * LoongArch64 内核链接脚本。
 *
 * 内存布局与硬件约定：
 * - 内核运行在 PLV0 直接映射配置窗口（DMW，基址为 __direct_base），所有段的虚拟地址（VMA）均为物理地址（LMA）加上 __direct_base。
 * - 异常入口（la_kernel_trap、la_user_trap）与 TLB 重填入口（la_kernel_refill）受硬件 CSR.EENTRY / CSR.TLBRENTRY 低 12 位为零的限制，必须按 4KiB（4096 字节）对齐。
 * - PHDRS 中的 FLAGS 在链接器解析程序头阶段直接求值（尚未解析后续 HIDDEN 符号），因此直接使用 ELF 标准权限位常量：5 = PF_R|PF_X（可读可执行），4 = PF_R（只读），6 = PF_R|PF_W（可读写）。
 */
PHDRS {
    text PT_LOAD FLAGS(5);
    rodata PT_LOAD FLAGS(4);
    data PT_LOAD FLAGS(6);
}
SECTIONS {
    . = __direct_base + __kernel_phys_start;
    .text : AT(ADDR(.text) - __direct_base) {
        PROVIDE(__executable_start = .);
        KEEP(*(.text.entry.primary))
        . = ALIGN(SECONDARY_ENTRY_ALIGNMENT);
        KEEP(*(.text.entry.secondary))
        . = ALIGN(4096);
        KEEP(*(.text.vector.kernel_trap))
        . = ALIGN(4096);
        KEEP(*(.text.vector.kernel_refill))
        . = ALIGN(4096);
        KEEP(*(.text.vector.user_trap))
        *(.text .text.*)
        PROVIDE(__etext = .);
    } :text
    . = ALIGN(4096);
    .rodata : AT(ADDR(.rodata) - __direct_base) {
        PROVIDE(__rodata_start = .);
        *(.rodata .rodata.*) *(.data.rel.ro .data.rel.ro.*)
    } :rodata
    INCLUDE ktest-sections.ld
    PROVIDE(__rodata_end = .);
    . = ALIGN(4096);
    .data : AT(ADDR(.data) - __direct_base) { *(.data .data.*) *(.sdata .sdata.*) } :data
    . = ALIGN(4096);
    __boot_reserved_phys_start = . - __direct_base;
    .boot (NOLOAD) : AT(ADDR(.boot) - __direct_base) {
        KEEP(*(.boot.stacks))
        KEEP(*(.boot.tables))
    } :data
    . = ALIGN(4096);
    __boot_reserved_phys_end = . - __direct_base;
    .bss (NOLOAD) : AT(ADDR(.bss) - __direct_base) {
        __bss_start = .;
        *(.bss .bss.*) *(.sbss .sbss.*) *(COMMON)
        . = ALIGN(BOOT_CLEAR_WORD_BYTES);
        __bss_end = .;
    } :data
    . = ALIGN(4096);
    __kernel_virt_end = .;
    __kernel_phys_end = . - __direct_base;
    __primary_entry_phys = __primary_entry - __direct_base;
    __secondary_entry_phys = __secondary_entry - __direct_base;
}

ASSERT((la_kernel_trap & 0xfff) == 0, "la_kernel_trap must be 4KiB aligned for EENTRY");
ASSERT((la_kernel_refill & 0xfff) == 0, "la_kernel_refill must be 4KiB aligned for TLBRENTRY");
ASSERT((la_user_trap & 0xfff) == 0, "la_user_trap must be 4KiB aligned for EENTRY");
