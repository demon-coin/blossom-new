//! LoongArch64 主核（BSP）与辅核（AP）机器引导入口。
//!
//! ## 模块职责与架构定位
//! 本模块位于内核最底层的硬件启动边界，提供主核入口 `__primary_entry` 与辅核入口 `__secondary_entry`。
//! 上游由固件（QEMU 固件或开发板 PMON/U-Boot）直接跳转到物理入口地址，或由 BSP 通过 IOCSR Mailbox + IPI
//! 唤醒辅核进入；下游在完成直接映射配置、栈指针初始化和 BSS 清零后，进入 Rust 主入口 `crate::primary_main`
//! 或辅核入口 `crate::secondary_main`。
//!
//! ## 启动阶段地址空间与状态转换
//! ```text
//! [固件 / 硬件复位 (PLV0, DA 直接地址翻译或固件初始映射)]
//!        │
//!        ▼
//! __primary_entry / __secondary_entry (物理地址区 .text.entry.*)
//!   1. 关闭本核全局中断 (CRMD.IE = 0)，清空 KS0 (Per-CPU 基址寄存器)
//!   2. 配置 DMW1 (0x9000_0000_0000_0000, Coherent Cached 内核 RAM 直映窗口)
//!   3. 配置 DMW0 (0x8000_0000_0000_0000, Strongly-Ordered Uncached 设备 MMIO 窗口)
//!   4. 清零 DMW2 / DMW3，通过绝对跳转进入高地址虚拟窗口
//!        │
//!        ▼
//! la_primary_high / la_secondary_high (DMW1 高地址虚拟区)
//!   5. 写入 CRMD_BOOT：启用 PG 映射地址模式，并将 DATF/DATM 设为一致可缓存 (CC)
//!   6. 主核：设置 BOOT_STACK 栈顶 -> 调用 la_clear_bss 清零 .bss -> 进入 primary_main(a0..a3)
//!      辅核：从 IOCSR Mailbox 读取栈顶与 CpuLocal 指针 -> 进入 secondary_main(local)
//! ```
//!
//! ## 并发与锁契约
//! - 本模块运行于各核独立的早期启动阶段，全程保持本核硬件中断关闭（`CRMD.IE = 0`），不获取任何自旋锁或睡眠锁。
//! - `.bss` 段清零由主核在释放任何辅核之前独占完成；辅核启动时直接读取主核已通过内存屏障发布的 `CpuBoot` 记录或 Mailbox 槽位。

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

/// 主核（BSP）裸机启动入口：保存固件传递的四个启动参数，配置内核 RAM 与 MMIO 的直接映射窗口（DMW），随后跳转到高地址虚拟空间继续执行。
///
/// # Safety
/// - 只能由固件在主核（BSP）处于 PLV0 特权级的初始引导阶段调用一次，此时系统中不得存在并发访问内核内存的其他核心。
/// - 内核镜像与启动栈必须已按链接脚本布局加载到对应的物理 RAM 中。
/// - `$a0`–`$a3` 承载固件传入的原始引导参数，本裸函数在建立好栈与 DMW 窗口前不得访问任何栈内存、`.bss` 全局变量或 `CpuLocal` 结构。
#[naked]
#[no_mangle]
#[link_section = ".text.entry.primary"]
unsafe extern "C" fn __primary_entry(_arg0: usize, _arg1: usize, _arg2: usize, _arg3: usize) -> ! {
    // SAFETY: 1. 固件保证当前处于主核 PLV0 特权级且内核镜像已装入对应物理内存；
    // 2. 汇编序列不依赖栈指针（SP）或全局数据，仅把固件参数暂存到 callee-saved 寄存器 $s0–$s3，
    //    关闭中断并配置好 PLV0 的 DMW0/DMW1 窗口后立即跳转到高地址符号 `la_primary_high`。
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

/// 主核高地址续接入口：切换至分页/DMW 映射模式（PG），初始化主核启动栈并清零 `.bss` 段，最后将固件原始参数转交 `primary_main`。
///
/// # Safety
/// - 只能由 `__primary_entry` 在配置好 DMW0/DMW1 且关闭本核中断（`CRMD.IE = 0`）后通过高地址跳转进入。
/// - `$s0`–`$s3` 必须保存着 `__primary_entry` 捕获的四个固件启动参数，且此时没有任何辅核（AP）在并发运行或访问 `.bss` 段。
#[naked]
unsafe extern "C" fn la_primary_high() -> ! {
    // SAFETY: 1. `__primary_entry` 已建立覆盖内核镜像的 DMW1 映射并跳入高地址，此时启用 `CRMD_BOOT`（PG 模式）不会丢失取指映射；
    // 2. `BOOT_STACK` 位于 `.bss` 段之外的独立 `.boot.stacks` 节，清零 `.bss` 不会破坏当前栈帧；
    // 3. `la_clear_bss` 遵循 C 调用约定，不会篡改保存在 `$s0`–`$s3` 中的固件启动参数。
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

/// 在主核高地址启动栈上按机器字清零链接脚本给出的 `.bss` 左闭右开区间 `[start, end)`。
///
/// # Safety
/// - `start` 与 `end` 必须指向同一块由主核独占、可写的内核 RAM 区域，满足 `start <= end` 且按 `usize`（8 字节）对齐。
/// - `[start, end)` 区间绝对不能与当前正在使用的主核启动栈（`BOOT_STACK`）重叠。
unsafe extern "C" fn la_clear_bss(start: usize, end: usize) {
    // SAFETY: 1. `la_primary_high` 传入链接器生成的 `__bss_start` 与 `__bss_end`，保证区间合法且对齐；
    // 2. 主核启动栈位于独立的 `.boot.stacks` 段，与 `.bss` 互不重叠，且此时辅核尚未唤醒，无并发访问冲突。
    unsafe { boot_clear_words(start, end) };
}

/// 辅核（AP）裸机物理入口：由固件在收到主核发送的启动 Mailbox 与 IPI 后从物理地址跳入，负责关闭本核中断、建立 DMW0/DMW1 窗口并跳转到高地址续接函数。
///
/// # Safety
/// - 只能由主核（BSP）通过 `start_cpu` 为尚未启动的辅核配置好 IOCSR Mailbox 后触发进入，执行特权级必须为 PLV0。
/// - 主核在发送启动 IPI 前必须已通过内存屏障发布目标核专属的 `CpuBoot` 启动记录（或板级 Mailbox 槽位中的栈顶与 `CpuLocal` 地址），且这些内存区域在内核生命周期内永久有效。
#[naked]
#[no_mangle]
#[link_section = ".text.entry.secondary"]
unsafe extern "C" fn __secondary_entry() -> ! {
    // SAFETY: 1. 主核已为当前辅核准备好独立的启动资源，且固件保证进入时处于 PLV0 特权级；
    // 2. 本段无栈汇编不读写栈内存、不触碰共享的 `.bss` 段，仅配置本核私有的 CRMD、KS0 与 DMW0..DMW3 寄存器并跳入高地址。
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

/// 辅核高地址续接入口：启用 PG 映射模式，从 IOCSR Mailbox 读取主核发布的内核栈顶与 `CpuLocal` 指针，随后进入 `secondary_main`。
///
/// # Safety
/// - 只能由 `__secondary_entry` 在关闭本核中断并配置好 DMW0/DMW1 窗口后跳入。
/// - IOCSR Mailbox 中存放的栈顶地址必须非空、由本核独占且满足 16 字节 ABI 对齐；`CpuLocal` 指针必须指向主核已为本核完整初始化的永久有效实例。
#[naked]
unsafe extern "C" fn la_secondary_high() -> ! {
    // SAFETY: 1. `__secondary_entry` 已关闭中断并建立好覆盖内核 RAM 的 DMW1 窗口；
    // 2. 在 LS2K1000LA 板级平台上，Mailbox 1 与 Mailbox 2 直接存放已转换好的高地址栈顶与 `CpuLocal` 指针；
    //    在 QEMU virt 平台上，Mailbox 1 存放 `CpuBoot` 物理地址，或上 `direct_map_virt_base` 即可通过 DMW1 安全读取 `stack_top` 与 `local` 字段。
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
