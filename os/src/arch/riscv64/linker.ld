OUTPUT_ARCH(riscv)
ENTRY(__primary_entry_phys)
INCLUDE platform-layout.ld
PHDRS { boot PT_LOAD FLAGS(5); text PT_LOAD FLAGS(5); rodata PT_LOAD FLAGS(4); data PT_LOAD FLAGS(6); }
SECTIONS {
  . = __kernel_phys_start;
  .boot : {
    KEEP(*(.text.boot.entry))
    KEEP(*(.text.boot.secondary))
  } :boot
  . = __kernel_virt_start + ALIGN(SIZEOF(.boot), 0x200000);
  .text : AT(ADDR(.text) - __kernel_virt_start + __kernel_phys_start) {
    PROVIDE(__executable_start = .);
    . = ALIGN(4);
    KEEP(*(.text.trap.entry))
    KEEP(*(.text.boot.init))
    *(.text .text.*)
    PROVIDE(__etext = .);
  } :text
  . = ALIGN(4096);
  .rodata : AT(ADDR(.rodata) - __kernel_virt_start + __kernel_phys_start) {
    PROVIDE(__rodata_start = .);
    *(.rodata .rodata.*) *(.data.rel.ro .data.rel.ro.*)
  } :rodata
  INCLUDE ktest-sections.ld
  PROVIDE(__rodata_end = .);
  . = ALIGN(4096);
  .data : AT(ADDR(.data) - __kernel_virt_start + __kernel_phys_start) {
    *(.data .data.*) *(.sdata .sdata.*)
    PROVIDE(__global_pointer$ = . + 0x800);
  } :data
  . = ALIGN(4096);
  __boot_reserved_phys_start = . - __kernel_virt_start + __kernel_phys_start;
  .boot.resources (NOLOAD) : AT(ADDR(.boot.resources) - __kernel_virt_start + __kernel_phys_start) {
    *(.boot.stacks) . = ALIGN(4096); *(.boot.tables)
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
ASSERT(SIZEOF(.boot) <= 0x200000, "physical entry exceeds bootstrap section")
ASSERT((rv_trap_entry & 3) == 0, "kernel trap entry must be 4-byte aligned")
ASSERT(__kernel_phys_end - __kernel_phys_start < 0x40000000, "image exceeds bootstrap mapping")
