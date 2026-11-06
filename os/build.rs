//! 内核构建脚本（Cargo Build Script）。
//!
//! # 模块职责与架构定位
//! 本脚本在宿主编译期运行，处于 `cargo` 构建管线与内核链接器（`ld`）及源码编译器（`rustc`）之间：
//! - **上游输入**：读取 Cargo 目标架构（`CARGO_CFG_TARGET_ARCH`）、测试开关（`CARGO_FEATURE_KTEST`）、
//!   目标板型环境变量（`BLOSSOM_PLATFORM`）、最大 CPU 核数（`BLOSSOM_MAX_CPUS`）以及
//!   `src/platform/layout.rs` 中的平台地址布局表。
//! - **下游输出**：向 `OUT_DIR` 写出 `cpu-capacity.rs`（供 `boot.rs` `include!`）、
//!   `platform-layout.ld`（供架构链接脚本 `INCLUDE`）和 `ktest-sections.ld`（控制测试与栈回溯段保留），
//!   并通过 `cargo:rustc-*` 指令配置编译条件（`blossom_platform`）和链接脚本参数。
//!
//! # 核心设计与构建产物布局
//! ```text
//! 环境变量 (CARGO_CFG_TARGET_ARCH, BLOSSOM_PLATFORM, BLOSSOM_MAX_CPUS, CARGO_FEATURE_KTEST)
//!       │
//!       ▼
//!   build.rs ──► rustc cfg: blossom_platform="qemu-virt" | "visionfive2" | "ls2k1000la"
//!       │
//!       ├──► $OUT_DIR/cpu-capacity.rs     (定义 pub const MAX_CPUS: usize)
//!       ├──► $OUT_DIR/platform-layout.ld  (定义 __kernel_phys_start / __kernel_virt_start 等链接符号)
//!       ├──► $OUT_DIR/ktest-sections.ld   (ktest 开启时保留 .ktest.suites 与 .eh_frame*，否则 /DISCARD/)
//!       └──► rustc-link-arg: -L$OUT_DIR -Tsrc/arch/<arch>/linker.ld
//! ```
//!
//! # 并发与构建契约
//! 构建脚本在单进程编译前阶段顺序执行，不涉及运行期锁；通过显式登记 `cargo:rerun-if-*`
//! 保证环境变量或布局源码变更时触发增量重编。

use std::env;
use std::fs;
use std::path::PathBuf;

#[path = "src/platform/layout.rs"]
mod layout;

/// 未显式设置 `BLOSSOM_MAX_CPUS` 时的默认最大 CPU 核心数上限。
const DEFAULT_MAX_CPUS: usize = 12;

/// 构建脚本入口：校验目标架构、开发板平台与内核测试（`ktest`）特性的组合合法性，
/// 在 `OUT_DIR` 中生成 CPU 容量常量与链接器脚本片段，并向 Cargo 输出编译及链接指令。
fn main() {
    // Step 1: 读取目标架构、平台与测试开关，拦截不支持的架构/板型组合（ktest 仅支持 qemu-virt）。
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

    // Step 2: 注册增量构建触发条件，并向 rustc 声明合法的 blossom_platform 条件编译值。
    println!("cargo:rerun-if-env-changed=CARGO_CFG_TARGET_ARCH");
    println!("cargo:rerun-if-env-changed=BLOSSOM_PLATFORM");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_KTEST");
    println!("cargo:rerun-if-env-changed=BLOSSOM_MAX_CPUS");
    println!("cargo:rerun-if-changed=src/platform/layout.rs");
    println!("cargo:rustc-check-cfg=cfg(blossom_platform, values(\"qemu-virt\", \"visionfive2\", \"ls2k1000la\"))");
    println!("cargo:rustc-cfg=blossom_platform=\"{platform}\"");

    // Step 3: 选择架构主链接脚本，解析并校验 BLOSSOM_MAX_CPUS（须为正整数且按 usize 位图对齐时不溢出），
    //         生成 $OUT_DIR/cpu-capacity.rs。
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

    // Step 4: 将选定平台的物理加载地址、高地址虚拟基址、直映窗口与 MMIO 窗口写入 $OUT_DIR/platform-layout.ld。
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

    // Step 5: 根据是否启用 ktest 生成异常表与测试套件段脚本（启用 panic unwind 测试时保留 .eh_frame*，
    //         普通构建直接丢弃以减小镜像体积），最后传入链接器搜索路径与链接脚本。
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
