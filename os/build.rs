use std::env;
use std::fs;
use std::path::PathBuf;

#[path = "src/platform/layout.rs"]
mod layout;

const DEFAULT_MAX_CPUS: usize = 12;

/// 校验目标架构、平台与测试功能组合，将共同布局常量写入 OUT_DIR 并登记重建条件。
fn main() {
    let arch = env::var("CARGO_CFG_TARGET_ARCH").expect("Cargo target architecture is missing");
    let platform = env::var("BLOSSOM_PLATFORM").unwrap_or_else(|_| "qemu-virt".into());
    let testing = env::var_os("CARGO_FEATURE_KTEST").is_some();
    assert!(
        !testing || platform == "qemu-virt",
        "kernel tests require BLOSSOM_PLATFORM=qemu-virt"
    );
    let layout = match (arch.as_str(), platform.as_str()) {
        ("riscv64", "qemu-virt") => layout::RISCV64_QEMU_VIRT,
        ("riscv64", "visionfive2") => layout::RISCV64_VISIONFIVE2,
        ("loongarch64", "qemu-virt") => layout::LOONGARCH64_QEMU_VIRT,
        ("loongarch64", "ls2k1000la") => layout::LOONGARCH64_LS2K1000LA,
        _ => panic!("unsupported kernel target/platform: {}/{}", arch, platform),
    };
    println!("cargo:rerun-if-env-changed=CARGO_CFG_TARGET_ARCH");
    println!("cargo:rerun-if-env-changed=BLOSSOM_PLATFORM");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_KTEST");
    println!("cargo:rerun-if-env-changed=BLOSSOM_MAX_CPUS");
    println!("cargo:rerun-if-changed=src/platform/layout.rs");
    println!("cargo:rustc-check-cfg=cfg(blossom_platform, values(\"qemu-virt\", \"visionfive2\", \"ls2k1000la\"))");
    println!("cargo:rustc-cfg=blossom_platform=\"{platform}\"");
    let script = match arch.as_str() {
        "riscv64" => "src/arch/riscv64/linker.ld",
        "loongarch64" => "src/arch/loongarch64/linker.ld",
        _ => panic!("unsupported kernel target: {}", arch),
    };
    let output = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo output directory is missing"));
    let max_cpus = match env::var("BLOSSOM_MAX_CPUS") {
        Ok(value) => {
            assert!(
                !value.is_empty() && value.bytes().all(|byte| byte.is_ascii_digit()),
                "BLOSSOM_MAX_CPUS must be a positive integer"
            );
            value
                .parse::<usize>()
                .expect("BLOSSOM_MAX_CPUS is out of range")
        }
        Err(env::VarError::NotPresent) => DEFAULT_MAX_CPUS,
        Err(error) => panic!("BLOSSOM_MAX_CPUS cannot be read: {}", error),
    };
    assert!(0 < max_cpus, "BLOSSOM_MAX_CPUS must be a positive integer");
    max_cpus
        .checked_add(usize::BITS as usize - 1)
        .expect("BLOSSOM_MAX_CPUS bitmap size overflows");
    fs::write(
        output.join("cpu-capacity.rs"),
        format!("pub const MAX_CPUS: usize = {max_cpus};\n"),
    )
    .expect("CPU capacity cannot be written");
    let linker_layout = format!(
        "__blossom_platform = {identity};\n\
         __kernel_phys_start = {physical:#x};\n\
         __kernel_virt_start = {virtual_start:#x};\n\
         __direct_base = {direct:#x};\n\
         __mmio_base = {mmio:#x};\n\
         __phys_ram_base = {ram_base:#x};\n",
        identity = layout.platform,
        physical = layout.kernel_load_phys,
        virtual_start = layout.kernel_link_virt,
        direct = layout.direct_map_virt_base,
        mmio = layout.mmio_virt_base,
        ram_base = layout.ram_phys_base,
    );
    fs::write(output.join("platform-layout.ld"), linker_layout)
        .expect("platform linker layout cannot be written");
    let sections = if testing {
        r#"
  . = ALIGN(8);
  .ktest.suites : AT(ADDR(.ktest.suites) - __kernel_virt_start + __kernel_phys_start) {
    __ktest_suites_start = .;
    KEEP(*(.ktest.suites))
    __ktest_suites_end = .;
  } :rodata
  . = ALIGN(8);
  .eh_frame : AT(ADDR(.eh_frame) - __kernel_virt_start + __kernel_phys_start) {
    __eh_frame = .;
    KEEP(*(.eh_frame .eh_frame.*))
    LONG(0)
    __eh_frame_end = .;
  } :rodata
  .eh_frame_hdr : AT(ADDR(.eh_frame_hdr) - __kernel_virt_start + __kernel_phys_start) {
    KEEP(*(.eh_frame_hdr .eh_frame_hdr.*))
  } :rodata
  .gcc_except_table : AT(ADDR(.gcc_except_table) - __kernel_virt_start + __kernel_phys_start) {
    __gcc_except_table_start = .;
    KEEP(*(.gcc_except_table .gcc_except_table.*))
    __gcc_except_table_end = .;
  } :rodata
"#
    } else {
        "/DISCARD/ : { *(.eh_frame .eh_frame.* .eh_frame_hdr .eh_frame_hdr.* .gcc_except_table .gcc_except_table.*) }\n"
    };
    fs::write(output.join("ktest-sections.ld"), sections)
        .expect("linker sections cannot be written");
    println!("cargo:rustc-link-arg=-L{}", output.display());
    if testing {
        println!("cargo:rustc-link-arg=--undefined-glob=__ktest_register__*");
    }
    println!("cargo:rustc-link-arg=-T{}", script);
    println!("cargo:rerun-if-changed={}", script);
}
