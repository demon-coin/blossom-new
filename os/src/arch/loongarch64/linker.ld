OUTPUT_ARCH(loongarch)
ENTRY(__primary_entry)
INCLUDE platform-layout.ld
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
        . = ALIGN(64);
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
        . = ALIGN(8);
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
