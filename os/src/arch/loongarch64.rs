//! LoongArch64 架构抽象层（HAL）总入口：直接映射窗口（DMW）、CSR/IOCSR 访问、异常与中断控制器（PCH-PIC/EXTIOI 与 LIOINTC）、单次定时器、多核启动（SMP）及电源管理。
//!
//! ## 模块职责与架构定位
//! 本模块是 `crate::arch` 在 `loongarch64` 目标架构下的顶层实现，统一封装了 QEMU `virt` 平台与龙芯 LS2K1000LA 开发板的核心硬件原语。
//! - **上游调用方**：内核引导（`boot`）、多核管理（`cpu`）、调度器（`sched`）、虚拟内存（`mm`）、中断子系统（`irq`）、时钟管理（`time`）以及测试框架（`ktest`）。
//! - **下游子模块**：引导汇编（`entry`）、异常向量（`trap`）、任务切换（`switch`）、用户态陷入与返回（`user`）、用户内存拷贝（`uaccess`）、页表与 TLB 管理（`mmu`）、非对齐访存模拟（`ale`）、信号栈帧（`signal`）、空闲等待（`idle`）、板级中断控制器（`liointc`）、实时钟（`rtc`）与硬件计数器（`counter`）。
//!
//! ## 虚拟地址空间与直接映射窗口（DMW）布局
//! 内核运行于 PLV0 特权级并启用分页映射模式（`CRMD.PG = 1`），通过两个直接映射配置窗口（DMW0 / DMW1）覆盖物理内存与外设，无需为内核线性映射建立页表：
//! ```text
//! 虚拟地址范围                                  映射机制     缓存属性                   用途
//! ───────────────────────────────────────────────────────────────────────────────────────────────────
//! 0x9000_0000_0000_0000 .. +物理地址空间        CSR_DMW1     Coherent Cached (CC)       内核代码、数据、物理页直映 (phys_to_virt)
//! 0x8000_0000_0000_0000 .. +物理地址空间        CSR_DMW0     Strongly-ordered Uncached  MMIO 外设寄存器访问 (mmio_address)
//! 0x0000_0000_0000_0000 .. 0x0000_007f_ffff_ffff CSR_PGDL     三级页表 (PGD->PMD->PT)    PLV3 用户态地址空间 (39-bit, 512 GiB)
//! ```
//!
//! ## 外部中断路由拓扑
//! ```text
//! [QEMU virt 平台]
//! 外设硬中断源 (0..63) ──► PCH-PIC (MMIO 电平触发) ──► EXTIOI (IOCSR 向量路由至 BSP) ──► CPU HWI1 (CSR_ESTAT.IS)
//!
//! [LS2K1000LA 板级平台]
//! 外设硬中断源 (0..63) ──► 双组 32 源 LIOINTC (MMIO) ──────────────────────────────────► CPU HWI0 / HWI1 (CSR_ESTAT.IS)
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! - **中断控制器自旋锁（`EXTERNAL_CONTROLLER`）**：保护 QEMU `virt` 平台下 PCH-PIC 与 EXTIOI 共享 32 位寄存器的读-改-写操作。
//! - **锁层级顺序**：上层中断描述符锁（`irq` 子系统） ➔ `EXTERNAL_CONTROLLER`（或 `liointc` 内部控制器锁）。在持有 `EXTERNAL_CONTROLLER` 期间严禁回调上层驱动或分配内存。
//! - **Per-CPU 指针（`CSR_KS0`）**：各 CPU 核心在关中断状态下通过 `install_cpu_local` 将本核专属的 `CpuLocal` 地址写入 `CSR_KS0`，内核态任意时刻可通过 `cpu_local()` 无锁读取。
use crate::boot::BootInfo;
use crate::platform::LINK_LAYOUT;
use crate::sched::context::TaskContext;
use crate::sync::SpinLock;
use crate::syscall::Errno;
use core::arch::asm;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// 当前目标架构的规范化名称标识。
pub const NAME: &str = "loongarch64";

// ============================================================================
// QEMU virt 平台 PCH-PIC (Platform Controller Hub Programmable Interrupt Controller) 寄存器偏移
// ============================================================================

/// PCH-PIC 中断屏蔽寄存器偏移（每 bit 对应 1 个中断源，1 = 屏蔽，0 = 放行）。
const PCH_PIC_MASK: usize = 0x20;
/// PCH-PIC HT-MSI 转发使能寄存器偏移（置 1 将中断通过 HyperTransport MSI 转发至 EXTIOI）。
const PCH_PIC_HTMSI_ENABLE: usize = 0x40;
/// PCH-PIC 触发方式配置寄存器偏移（0 = 电平触发 Level，1 = 边沿触发 Edge）。
const PCH_PIC_EDGE: usize = 0x60;
/// PCH-PIC 边沿/锁存中断清除寄存器偏移（写 1 清除对应位）。
const PCH_PIC_CLEAR: usize = 0x80;
/// PCH-PIC 中断源目标路由表起始偏移（每个中断源占 1 字节）。
const PCH_PIC_ROUTE: usize = 0x100;
/// PCH-PIC HT-MSI 中断向量映射表起始偏移（每个中断源占 1 字节，指定发往 EXTIOI 的向量号）。
const PCH_PIC_VECTOR: usize = 0x200;
/// PCH-PIC 触发极性配置寄存器偏移（0 = 高电平/上升沿有效，1 = 低电平/下降沿有效）。
const PCH_PIC_POLARITY: usize = 0x3e0;
/// 32 位 I/O 寄存器的字节宽度（4 字节）。
const IO_WORD_BYTES: usize = core::mem::size_of::<u32>();
/// 每个 32 位寄存器字包含的 16 位 NODEMAP 表项数量（2 项）。
const NODEMAP_ENTRIES_PER_WORD: usize = IO_WORD_BYTES / core::mem::size_of::<u16>();
/// PCH-PIC 支持的外部中断源总数（2 个 32 位字，共 64 个中断源）。
const PCH_PIC_SOURCE_COUNT: u32 = 2 * u32::BITS;
/// PCH-PIC 路由至 HT 中断通道的路由属性值。
const PCH_PIC_ROUTE_HT: u8 = 1;

// ============================================================================
// QEMU virt 平台 EXTIOI (Extended I/O Interrupt Controller) IOCSR 相对偏移与常量
// 注意：以下偏移均相对于 BootInfo 给出的 `parent_controller.start` 基址。
// ============================================================================

/// EXTIOI 节点映射（NODEMAP）寄存器起始偏移。
const EXTIOI_NODEMAP: usize = 0xa0;
/// EXTIOI 处理器核中断引脚映射（IPMAP）寄存器起始偏移。
const EXTIOI_IPMAP: usize = 0xc0;
/// EXTIOI 中断源使能（ENABLE）位图寄存器起始偏移。
const EXTIOI_ENABLE: usize = 0x200;
/// EXTIOI 中断轮转分发（BOUNCE）位图寄存器起始偏移。
const EXTIOI_BOUNCE: usize = 0x280;
/// EXTIOI 中断状态（ISR，挂起状态与写 1 清零）寄存器起始偏移。
const EXTIOI_ISR: usize = 0x400;
/// EXTIOI 中断源核心路由（COREISR/ROUTE）字节表起始偏移。
const EXTIOI_ROUTE: usize = 0x800;
/// EXTIOI 每个 NUMA 节点包含的物理 CPU 核心数（4 核）。
const EXTIOI_CORES_PER_NODE: usize = 4;
/// EXTIOI 路由字节中节点编号（Node ID）字段的左移位数（高 4 位表示节点，低 4 位为核位图）。
const EXTIOI_ROUTE_NODE_SHIFT: usize = 4;
/// EXTIOI 路由字节可编码的最大节点数量（16 个节点）。
const EXTIOI_NODE_COUNT: usize = 1 << EXTIOI_ROUTE_NODE_SHIFT;
/// EXTIOI 汇总上报给 CPU 的根硬件中断线编号（对应 `CPU_HWI1`）。
const EXTIOI_ROOT_INTERRUPT: u32 = reg::CPU_HWI1;
/// EXTIOI IPMAP 字节中将中断组映射到 `HWI1` 的掩码值（Bit 1 对应 HWI1）。
const EXTIOI_IPMAP_HWI1: u32 = 2;
/// IOCSR 杂项功能控制寄存器编号（`0x420`）。
const IOCSR_MISC_FUNCTION: usize = 0x420;
/// `IOCSR_MISC_FUNCTION` 中启用 EXTIOI 控制器的使能位（Bit 48）。
const IOCSR_EXTIOI_ENABLE: u64 = 1 << 48;

// ============================================================================
// QEMU virt 平台 ACPI GED (Generic Event Device) 电源管理寄存器定义
// ============================================================================

/// QEMU `virt` 平台 GED 电源控制寄存器的物理基地址。
const GED_BASE: usize = 0x100e001c;
/// GED 睡眠控制寄存器（`sleep_ctl`）相对于 `GED_BASE` 的字节偏移。
const GED_SLEEP_CONTROL: usize = 0;
/// GED 复位控制寄存器（`reset`）相对于 `GED_BASE` 的字节偏移。
#[cfg(not(feature = "ktest"))]
const GED_RESET: usize = 2;
/// GED 睡眠控制寄存器中的执行睡眠使能位（`SLP_EN`，Bit 5）。
const GED_SLEEP_ENABLE: u8 = 1 << 5;
/// GED 睡眠控制寄存器中的 S5 软关机状态编码（`SLP_TYP = 5`，位于 Bit 4:2）。
const GED_SLEEP_S5: u8 = 5 << 2;
/// 向 GED 复位寄存器触发系统重启的写入特征值。
#[cfg(not(feature = "ktest"))]
const GED_RESET_VALUE: u8 = 0x42;

pub mod ale;
mod counter;
mod entry;
mod idle;
mod liointc;
mod mmu;
mod register;
#[cfg(feature = "ktest")]
mod register_test;
mod rtc;
use register as reg;
#[cfg(feature = "ktest")]
pub(crate) use register::{CRMD_IE, CSR_CRMD, CSR_DMW0, CSR_DMW1, CSR_EUEN, CSR_KS0};
pub(crate) use register::{
    ECODE_ADE, ECODE_ALE, ECODE_BCE, ECODE_BRK, ECODE_FPE, ECODE_PPI, EUEN_FP, EUEN_LASX, EUEN_LSX,
    FCSR_CAUSE_SHIFT, FCSR_ENABLE_MASK,
};
mod signal;
mod switch;
mod trap;
mod uaccess;
mod user;
use crate::arch::loongarch64::trap::{la_kernel_refill, la_kernel_trap};
pub use counter::{apply_counter_epoch, establish_counter_epoch};
pub use rtc::board_rtc_calendar;
use switch::la_switch_context;
#[cfg(feature = "ktest")]
use {register_test::la_test_register_preservation, switch::la_test_task_context_preservation};

pub use mmu::{
    activate_kernel_root, activate_user_root, flush_user_translations, initialize_user_root,
    sync_user_icache, user_empty_entry, user_entry_is_table, user_entry_physical, user_leaf_dirty,
    user_leaf_entry, user_leaf_mkclean, user_leaf_permissions, user_table_entry,
    SIGNAL_RETURN_PAGE, USER_ADDRESS_END, USER_LEVELS, USER_STACK_TOP,
};
pub(crate) use signal::SIGNAL_MIN_STACK;
pub use uaccess::{copy_user_bytes, load_user_u32, store_user_u32};
pub use user::{run_user, signal_return_code, UserContext};

#[cfg(feature = "ktest")]
pub(crate) use user::{
    exit_offset, fault_offset, loop_offset, loop_spin_offset, user_code, FpRegister,
};

/// 内核态（PLV0）普通中断与同步异常的保存栈帧布局。
///
/// 由 `trap::la_kernel_trap` 在当前内核栈上压栈构造，并作为 `&mut KernelTrapFrame` 传给 `loongarch_trap`。
#[repr(C, align(16))]
struct KernelTrapFrame {
    /// 32 个通用整数寄存器（`$r0`–`$r31`），其中 `gpr[3]` 保存异常发生前的内核栈指针 `$sp`。
    gpr: [usize; 32],
    /// 异常返回地址寄存器（`CSR_ERA`），记录被打断指令的 PC；在 `idle` 窗口被中断时会被改写为跳出窗口的出口地址。
    era: usize,
    /// 异常前模式信息寄存器（`CSR_PRMD`），记录被打断时的特权级（`PPLV`）与中断使能状态（`PIE`）。
    prmd: usize,
    /// 异常状态寄存器（`CSR_ESTAT`），包含待处理的硬件中断位图（`IS`）以及异常主/子类型码（`Ecode`/`EsubCode`）。
    estat: usize,
    /// 出错虚地址寄存器（`CSR_BADV`），在地址相关异常时记录触发故障的虚拟地址。
    badv: usize,
    /// 扩展单元使能寄存器（`CSR_EUEN`），决定进出异常时是否保存与恢复下方的浮点寄存器字段。
    euen: usize,
    /// 当前 CPU 的 `CpuLocal` 基址（从 `CSR_KS0` 保存并在 `ertn` 前恢复）。
    cpu_base: usize,
    /// 32 个 64 位标量浮点寄存器（`$f0`–`$f31`）；仅当 `euen & EUEN_FP != 0` 时由汇编写入，否则保持未初始化状态。
    fpr: MaybeUninit<[u64; 32]>,
    /// 浮点控制与状态寄存器（`FCSR0`）；仅当 `euen & EUEN_FP != 0` 时由汇编写入。
    fcsr: MaybeUninit<usize>,
    /// 8 个浮点条件标志寄存器（`$fcc0`–`$fcc7`）；仅当 `euen & EUEN_FP != 0` 时由汇编写入。
    fcc: MaybeUninit<[usize; 8]>,
}

/// 内核态致命 TLB 重填异常（TLB Refill）专用的保存栈帧布局。
///
/// 硬件触发 TLB 重填时使用专用的 `TLBRERA`、`TLBRPRMD` 与 `TLBRBADV` 寄存器，不修改普通异常的 `ERA`/`PRMD`/`BADV`。
/// 当内核态（PLV0）意外触发不可恢复的 TLB 重填故障时，由 `trap::la_kernel_refill` 在内核栈上构造此帧并交由 `loongarch_refill` 报告。
#[repr(C, align(16))]
struct RefillFrame {
    /// 32 个通用整数寄存器（`$r0`–`$r31`），其中 `gpr[3]` 保存重填异常发生前的内核栈指针 `$sp`。
    gpr: [usize; 32],
    /// TLB 重填异常返回地址寄存器（`CSR_TLBRERA`），其最低位为 `IsTLBR` 标志位，高位为故障指令 PC。
    era: usize,
    /// TLB 重填异常前模式信息寄存器（`CSR_TLBRPRMD`）。
    prmd: usize,
    /// TLB 重填出错虚地址寄存器（`CSR_TLBRBADV`），记录触发重填的虚拟地址。
    badv: usize,
}

extern "C" {
    /// 链接脚本导出的辅核物理入口符号（对应 `entry::__secondary_entry` 的物理装载地址）。
    static __secondary_entry_phys: u8;
    /// 在开启本核中断（`CRMD.IE = 1`）到执行 `idle 0` 指令的受控窗口内原子复查唤醒标志；若窗口内发生中断，则由 `loongarch_trap` 将 `ERA` 重定向到 `la_idle_window_exit` 立即退出等待。
    fn la_idle_once_asm(
        kick_pending: *const AtomicUsize,
        wake_pending: *const AtomicBool,
        #[cfg(feature = "ktest")] wait_completions: *const AtomicUsize,
    );
    /// `idle` 汇编等待窗口的起始指令地址标号（闭区间起点）。
    static la_idle_window_start: u8;
    /// `idle` 汇编等待窗口的结束指令地址标号（开区间终点，紧随 `idle 0` 指令之后）。
    static la_idle_window_end: u8;
    /// 当中断发生在 `[la_idle_window_start, la_idle_window_end)` 窗口内时，异常返回地址被重定向到的退出清理标号。
    static la_idle_window_exit: u8;
}

/// 读取当前 CPU 核心上编号为 `R` 的控制状态寄存器（CSR）。
///
/// 调用环境必须处于 PLV0 内核特权级，寄存器编号 `R` 由编译期常量指定。
#[inline]
fn csr_read<const R: u16>() -> usize {
    let value;
    // SAFETY: 1. 内核主辅核引导入口均保证 Rust 代码运行在 PLV0 特权级；
    // 2. 本模块传入的编译期常量 `R` 均为 LoongArch64 架构规范定义的可读 CSR，且 `csrrd` 不访问内存或栈。
    unsafe {
        asm!("csrrd {value}, {reg}", value = out(reg) value, reg = const R, options(nomem, nostack));
    }
    value
}

/// 向当前 CPU 核心上编号为 `R` 的控制状态寄存器（CSR）写入新值。
///
/// # Safety
/// - 调用时必须处于 PLV0 特权级，且 `R` 必须是架构合法的可写 CSR 编号。
/// - 写入的 `value` 不得破坏当前正在执行的内核代码映射（DMW0/DMW1）、当前内核栈可达性或 `CSR_KS0` 中的 Per-CPU 指针约定。
#[inline]
unsafe fn csr_write<const R: u16>(value: usize) {
    // SAFETY: 调用方保证当前处于 PLV0 特权级、`R` 为合法可写 CSR，且写入值满足对应寄存器的映射与中断状态不变量。
    unsafe {
        asm!("csrwr {value}, {reg}", value = inout(reg) value => _, reg = const R, options(nostack));
    }
}

/// 通过 `iocsrrd.w` 指令读取指定 32 位 IOCSR 寄存器的值。
#[inline]
fn iocsr_read32(reg: usize) -> u32 {
    let value: usize;
    // SAFETY: 1. 内核运行于 PLV0 特权级，具备执行 `iocsrrd.w` 指令的权限；
    // 2. 本模块调用点传入的 `reg` 均为已验证平台的合法 32 位 IOCSR 地址，指令不触碰栈空间。
    unsafe {
        asm!("iocsrrd.w {value}, {reg}", value = out(reg) value, reg = in(reg) reg, options(nostack));
    }
    value as u32
}

/// 通过 `iocsrrd.d` 指令读取指定 64 位 IOCSR 寄存器的值。
#[inline]
fn iocsr_read64(reg: usize) -> u64 {
    let value;
    // SAFETY: 1. 内核运行于 PLV0 特权级，具备执行 `iocsrrd.d` 指令的权限；
    // 2. 调用点仅传入平台有效的 64 位 IOCSR 寄存器编号，指令不访问普通内存或栈。
    unsafe {
        asm!("iocsrrd.d {value}, {reg}", value = out(reg) value, reg = in(reg) reg, options(nostack));
    }
    value
}

/// 通过 `iocsrwr.w` 指令向指定 32 位 IOCSR 寄存器写入 32 位值。
#[inline]
fn iocsr_write32(reg: usize, value: u32) {
    // SAFETY: 1. 内核运行于 PLV0 特权级；
    // 2. 本模块调用点仅向已验证的 IPI 或 EXTIOI 32 位 IOCSR 寄存器写入控制字，且对共享路由寄存器的读-改-写已由 `EXTERNAL_CONTROLLER` 自旋锁串行化。
    unsafe {
        asm!("iocsrwr.w {value}, {reg}", value = in(reg) value as usize, reg = in(reg) reg, options(nostack));
    }
}

/// 通过 `iocsrwr.d` 指令向指定 64 位 IOCSR 寄存器写入 64 位值。
#[inline]
fn iocsr_write64(reg: usize, value: u64) {
    // SAFETY: 1. 内核运行于 PLV0 特权级；
    // 2. 调用点仅向合法的 64 位 IOCSR（如 `IOCSR_MAIL_SEND` 或 `IOCSR_MISC_FUNCTION`）写入数据，Mailbox 传递的指针生命周期由 `start_cpu` 的调用契约保证。
    unsafe {
        asm!("iocsrwr.d {value}, {reg}", value = in(reg) value, reg = in(reg) reg, options(nostack));
    }
}

/// 执行全序数据栅障指令（`dbar 0`），确保屏障前后的内存与设备 I/O 访问严格按序完成。
fn barrier() {
    // SAFETY: `dbar 0` 为无内存解引用副作用的架构同步屏障指令，在 PLV0 与任意特权级下均可安全执行。
    unsafe {
        asm!("dbar 0", options(nostack));
    }
}

/// 执行 DMA 写完成屏障（`dbar 0x0a`），确保 CPU 对一致性 DMA 缓冲区的写入在通知设备前已全局可见。
pub(super) fn dma_write_barrier() {
    // SAFETY: `dbar 0x0a` 仅对屏障前后的写操作施加完成型顺序约束，不访问栈；保留默认内存副作用以阻止编译器重排访存。
    unsafe { asm!("dbar {hint}", hint = const reg::DBAR_COMPLETION_WRITE, options(nostack)) };
}

/// 执行 DMA 读完成屏障（`dbar 0x05`），确保在观察到设备完成状态后再读取 DMA 缓冲区中的数据。
pub(super) fn dma_read_barrier() {
    // SAFETY: `dbar 0x05` 仅对屏障前后的读操作施加完成型顺序约束，不访问栈；保留默认内存副作用以阻止编译器重排读取。
    unsafe { asm!("dbar {hint}", hint = const reg::DBAR_COMPLETION_READ, options(nostack)) };
}

/// 执行双向全功能 DMA 内存屏障（`dbar 0x00`），同时约束屏障前后的全部内存与 MMIO 读写顺序。
pub(super) fn dma_barrier() {
    // SAFETY: `dbar 0x00` 是架构完整数据屏障，不访问栈；未标记 `nomem`/`readonly`，可同时充当编译器内存屏障。
    unsafe { asm!("dbar 0x00", options(nostack)) };
}

/// 刷新非一致性 DMA 缓存区间（当前 LoongArch64 平台仅使用硬件缓存一致性 DMA，不支持非一致性缓存维护操作）。
pub fn dma_cache_flush(_: usize, _: usize) -> Result<(), &'static str> {
    Err("LoongArch noncoherent DMA cache maintenance is unsupported")
}

/// 从 `CSR_CPUID` 寄存器提取当前物理 CPU 的硬件核心编号（忽略固件传入的引导寄存器参数）。
pub fn boot_hardware_id(_: [usize; 4]) -> usize {
    csr_read::<{ reg::CSR_CPUID }>() & reg::HARDWARE_CPU_ID_MASK
}

/// 将物理 RAM 地址转换为通过 `CSR_DMW1`（一致可缓存）窗口访问的内核高地址虚拟地址。
pub fn phys_to_virt(pa: usize) -> usize {
    pa | LINK_LAYOUT.direct_map_virt_base as usize
}

/// 将位于 `CSR_DMW1` 直接映射窗口内的内核虚拟地址还原为物理 RAM 地址（仅适用于 DMW1 线性映射区地址）。
pub fn virt_to_phys(va: usize) -> usize {
    va & !(LINK_LAYOUT.direct_map_virt_base as usize)
}

/// 将外设物理地址转换为通过 `CSR_DMW0`（强序非缓存）窗口访问的内核 MMIO 虚拟地址。
pub fn mmio_address(pa: usize) -> usize {
    pa | LINK_LAYOUT.mmio_virt_base as usize
}

/// 校验主核引导入口配置的 `CRMD.PG` 与 `DMW0`/`DMW1` 窗口状态，初始化全局无效页表链并绑定早期控制台资源。
///
/// # Safety
/// - 只能由主核（BSP）在硬件中断关闭、任何辅核（AP）尚未唤醒的引导阶段调用一次。
/// - 调用前必须已由汇编入口启用 `CRMD_PG` 并配置好 `CSR_DMW0` 与 `CSR_DMW1`，且全局 `BootInfo` 已完成发布。
pub unsafe fn finish_boot_mappings(_: &BootInfo) {
    assert_eq!(
        csr_read::<{ reg::CSR_CRMD }>() & reg::CRMD_ADDRESS_MODE,
        reg::CRMD_PG
    );
    assert_eq!(
        csr_read::<{ reg::CSR_DMW1 }>(),
        phys_to_virt(0) | reg::DMW_KERNEL_RAM
    );
    assert_eq!(
        csr_read::<{ reg::CSR_DMW0 }>(),
        mmio_address(0) | reg::DMW_PLV0
    );
    // SAFETY: 1. 上方断言已确认当前处于 PG 模式且 DMW0/DMW1 直映窗口有效；
    // 2. 调用方保证此时由主核在释放辅核前独占执行，无并发访问 `LA_INVALID_ROOT` 页表的风险。
    unsafe { mmu::initialize_invalid_tables() };
    crate::console::configure_resources(crate::boot::info());
}

/// 将当前 CPU 专属的 `CpuLocal` 结构指针安装到 `CSR_KS0` 寄存器中。
///
/// # Safety
/// - `local` 必须非空、满足 `CpuLocal` 对齐要求、已完成完整初始化，且位于永久有效的 DMW1 内核映射区内。
/// - 必须在本核硬件中断关闭（`CRMD.IE == 0`）状态下调用，且每个物理 CPU 核心必须安装各自独立的 `CpuLocal` 实例。
pub unsafe fn install_cpu_local(local: *const crate::cpu::CpuLocal) {
    // SAFETY: 调用方保证当前处于本核 PLV0 关中断状态，且 `local` 是本核独占、永久有效的 `CpuLocal` 地址。
    unsafe { csr_write::<{ reg::CSR_KS0 }>(local as usize) };
}

/// 从 `CSR_KS0` 寄存器读取当前物理 CPU 核心的 `CpuLocal` 结构指针。
pub fn cpu_local() -> *const crate::cpu::CpuLocal {
    csr_read::<{ reg::CSR_KS0 }>() as *const _
}

/// 在关中断的调度临界区内将当前内核任务的整数上下文保存至 `prev`，并切换到 `next` 指向的任务上下文。
///
/// 切换过程保留当前物理 CPU 的 `CSR_KS0` 不变，因此任务即使跨核恢复也能正确读取所在 CPU 的 `CpuLocal`。
///
/// # Safety
/// - 调用方必须对 `prev` 拥有独占写权限、对 `next` 拥有独占读权限；两个指针均须非空、对齐且不重叠。
/// - `next` 必须已初始化为合法的可恢复任务现场，且双方关联的内核栈均满足 16 字节 ABI 对齐并在各自再次恢复前保持有效。
/// - 调用时本核中断必须关闭（`!irq_enabled()`）、扩展单元必须关闭（`CSR_EUEN == 0`），且不得持有普通自旋锁、中断/抢占守卫或指向当前核 `CpuLocal` 的跨切换借用。
pub unsafe fn switch_context(prev: *mut TaskContext, next: *const TaskContext) {
    assert!(!irq_enabled(), "context switch requires IRQ off");
    assert_eq!(
        csr_read::<{ reg::CSR_EUEN }>(),
        0,
        "context switch requires EUEN off"
    );
    // SAFETY: 1. 上方断言已验证本核中断关闭（`CRMD.IE == 0`）且扩展单元未开启（`CSR_EUEN == 0`）；
    // 2. 调用方保证 `prev` 与 `next` 指向合法、独占且栈对齐的 `TaskContext`，满足 `la_switch_context` 的全部前置条件。
    unsafe { la_switch_context(prev, next) };
}

/// 初始化当前 CPU 核心的异常入口、TLB 重填入口、页表遍历器、定时器与核间中断（IPI），并根据平台类型使能对应的中断线。
pub fn init_current_cpu() {
    irq_save();
    // SAFETY: 1. 引导汇编入口已建立 PLV0 特权级、DMW0/DMW1 窗口与有效内核栈，且上方 `irq_save()` 已关闭本核硬件中断；
    // 2. `la_kernel_trap` 位于 DMW1 虚拟映射区，`la_kernel_refill` 经 `virt_to_phys` 转换为物理入口地址，均满足硬件向量寄存器的格式要求。
    unsafe {
        csr_write::<{ reg::CSR_EENTRY }>(la_kernel_trap as usize);
        csr_write::<{ reg::CSR_TLBRENTRY }>(virt_to_phys(la_kernel_refill as usize));
        let external = if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
            reg::IRQ_HWI0 | reg::IRQ_HWI1
        } else {
            reg::IRQ_HWI1
        };
        csr_write::<{ reg::CSR_ECFG }>(reg::IRQ_TIMER | reg::IRQ_IPI | external);
        csr_write::<{ reg::CSR_EUEN }>(0);
    }
    mmu::initialize_current_cpu();
    disable_timer();
    iocsr_write32(reg::IOCSR_IPI_CLEAR, u32::MAX);
    iocsr_write32(reg::IOCSR_IPI_ENABLE, u32::MAX);
    barrier();
}

/// 在 PLV0 下原子清除当前 CPU 的全局中断使能位（`CRMD.IE`），并返回修改前的原始 `CSR_CRMD` 值。
///
/// 内联汇编带有默认编译器内存屏障，保证临界区内的内存读写不会被编译器重排到关中断指令之前。
pub fn irq_save() -> usize {
    let old;
    // SAFETY: 1. 内核运行于 PLV0 特权级；固定将掩码 `CRMD_IE`（4）放入 `$t0`（$r12），避开 `csrxchg` 指令禁止作为掩码寄存器的 `$r0`/`$r1`；
    // 2. `inout(reg) 0usize => old` 将 `CRMD.IE` 清零并接收旧的 `CSR_CRMD` 值，且寄存器分配器保证 `old` 与固定寄存器 `$t0` 互不冲突；
    // 3. 不声明 `nomem`/`readonly`，确保编译器不会将临界区内的访存指令重排越过关中断边界。
    unsafe {
        asm!("csrxchg {old}, $t0, {crmd}", old = inout(reg) 0usize => old, in("$t0") reg::CRMD_IE, crmd = const reg::CSR_CRMD, options(nostack));
    }
    old
}

/// 根据配对 `irq_save()` 保存的 `saved` 状态字，在 PLV0 下恢复当前 CPU 的全局中断使能位（`CRMD.IE`）。
///
/// 内联汇编带有默认编译器内存屏障，保证临界区内的内存访问与锁释放操作全部在恢复中断之前完成。
///
/// # Safety
/// - `saved` 必须来自同一颗物理 CPU 上配对的 `irq_save()` 返回值，且在保存与恢复之间不得发生跨核迁移。
/// - 若存在嵌套的 `irq_save()`，必须严格按后进先出（LIFO）逆序调用 `irq_restore()`。
/// - 在恢复开启中断前，必须已释放所有要求在关中断状态下持有的自旋锁或 Per-CPU 借用。
pub unsafe fn irq_restore(saved: usize) {
    // SAFETY: 1. 调用方保证在同一 CPU 上按逆序恢复且临界区已结束；
    // 2. 固定使用 `$t0`（$r12）承载掩码 `CRMD_IE`，仅修改 `CSR_CRMD` 的 `IE` 位，不影响其他模式位；
    // 3. 保留汇编默认内存副作用，阻止编译器将临界区内的访存重排到恢复中断之后。
    unsafe {
        asm!("csrxchg {value}, $t0, {crmd}", value = inout(reg) (saved & reg::CRMD_IE) => _, in("$t0") reg::CRMD_IE, crmd = const reg::CSR_CRMD, options(nostack));
    }
}

/// 查询当前 CPU 核心的全局硬件中断使能位（`CRMD.IE`）是否处于开启状态。
pub fn irq_enabled() -> bool {
    csr_read::<{ reg::CSR_CRMD }>() & reg::CRMD_IE != 0
}

/// 在 PLV0 下无条件开启当前 CPU 核心的全局硬件中断（将 `CRMD.IE` 置 1）。
pub fn enable_interrupts() {
    // SAFETY: 1. 调用前当前 CPU 已通过 `init_current_cpu` 安装好异常向量表与中断控制器状态；
    // 2. 固定使用 `$t0`（$r12）承载掩码 `CRMD_IE`，仅置位 `CSR_CRMD.IE`，并保留编译器内存屏障。
    unsafe {
        asm!("csrxchg {value}, $t0, {crmd}", value = inout(reg) reg::CRMD_IE => _, in("$t0") reg::CRMD_IE, crmd = const reg::CSR_CRMD, options(nostack));
    }
}

/// 执行 `idle 0` 指令使当前 CPU 核心进入低功耗休眠，直至收到硬件中断唤醒。
pub fn wait_for_interrupt() {
    // SAFETY: 在 PLV0 特权级下执行 `idle 0` 仅挂起本核流水线等待中断，不访问栈或修改任何内存映射。
    unsafe {
        asm!("idle 0", options(nostack));
    }
}

/// 在调度器空闲循环中执行一次受控的开中断休眠等待（解决“检查唤醒条件”与“执行 `idle 0`”之间的丢中断竞态）。
///
/// 汇编函数 `la_idle_once_asm` 在开启 `CRMD.IE` 后会立即复查 `kick_pending` 与 `wake_pending`：
/// - 若发现已有待处理事件，立即关闭 `CRMD.IE` 并返回；
/// - 若在开启中断后、执行 `idle 0` 前或执行期间收到中断，`loongarch_trap` 会将异常返回地址 `ERA` 修改为 `la_idle_window_exit`，从而在中断处理完毕后直接跳过 `idle 0`、关闭中断并返回调度器重新扫描就绪队列。
///
/// # Safety
/// - 只能由运行在本核永久引导栈/调度栈（boot/scheduler stack）上的空闲循环调用，调用时本核中断必须关闭（`!irq_enabled()`）。
/// - 调用点不得持有任何自旋锁、`IrqSaveGuard`、`BottomHalfGuard` 或 `PreemptGuard`。
/// - `kick_pending`（及 `ktest` 下的 `wait_completions`）必须指向本核永久有效的原子字段，`wake_pending` 在本次调用期间必须保持地址有效且仅通过原子指令访问。
pub(crate) unsafe fn idle_once(
    kick_pending: &AtomicUsize,
    wake_pending: &AtomicBool,
    #[cfg(feature = "ktest")] wait_completions: &AtomicUsize,
) {
    assert!(!irq_enabled(), "idle entry requires IRQ off");
    // SAFETY: 1. 上方断言已确认进入时本核中断关闭，调用方保证当前位于本核永久调度栈且未持有任何锁；
    // 2. `kick_pending`、`wake_pending`（以及 `ktest` 下的 `wait_completions`）均为有效原子变量指针，汇编窗口在返回前必定重新关闭 `CRMD.IE`。
    unsafe {
        #[cfg(feature = "ktest")]
        la_idle_once_asm(kick_pending, wake_pending, wait_completions);
        #[cfg(not(feature = "ktest"))]
        la_idle_once_asm(kick_pending, wake_pending);
    };
    assert!(!irq_enabled(), "idle exit has IRQ enabled");
}

/// 通过 `rdtime.d` 指令读取 LoongArch64 架构的 64 位恒定频率硬件计数器（Stable Counter）当前计数值。
pub fn read_counter() -> u64 {
    let value;
    // SAFETY: `rdtime.d` 只读取 CPU 内部只读计时器并写入目标通用寄存器，不访问内存或栈。
    unsafe {
        asm!("rdtime.d {value}, $zero", value = out(reg) value, options(nomem, nostack));
    }
    value
}

/// 执行 `cpucfg` 指令读取指定字号 `index` 的处理器配置信息寄存器（保留低 32 位有效配置字）。
pub(crate) fn cpucfg(index: usize) -> u64 {
    let value: usize;
    // SAFETY: `cpucfg` 为只读查询 CPU 硬件特性的无副作用指令，不访问内存或栈。
    unsafe {
        asm!("cpucfg {value}, {index}", value = out(reg) value, index = in(reg) index, options(nomem, nostack));
    }
    value as u64 & u64::from(u32::MAX)
}

/// 根据当前 CPU 的 `CPUCFG` 特性字，返回进入用户态时允许开启的扩展单元使能掩码（`CSR_EUEN`）。
///
/// 按硬件支持梯度依次启用：无浮点（`0`）、基础标量浮点（`EUEN_SCALAR`）、128 位 LSX 向量（`EUEN_SIMD`）或 256 位 LASX 宽向量（`EUEN_WIDE_SIMD`）。
pub(crate) fn user_euen() -> usize {
    let config = cpucfg(reg::CPUCFG_FEATURES);
    if config & reg::CPUCFG_FP == 0 {
        return 0;
    }
    if config & reg::CPUCFG_LSX == 0 {
        return reg::EUEN_SCALAR;
    }
    if config & reg::CPUCFG_LASX == 0 {
        return reg::EUEN_SIMD;
    }
    reg::EUEN_WIDE_SIMD
}

/// 返回当前 CPU 用户态上下文中每个浮点/向量寄存器包含的 64 位通道（`u64` lane）数量（0 = 无浮点，1 = FP64，2 = LSX 128-bit，4 = LASX 256-bit）。
pub(crate) fn user_fp_lanes() -> usize {
    match user_euen() {
        reg::EUEN_WIDE_SIMD => 4,
        reg::EUEN_SIMD => 2,
        reg::EUEN_SCALAR => 1,
        _ => 0,
    }
}

/// 在早期引导阶段通过读取 `CPUCFG.4`（基准晶振频率）与 `CPUCFG.5`（倍频系数 `mul` / 分频系数 `div`）计算恒定计数器的时钟频率（单位：Hz）。
pub fn boot_counter_frequency() -> u64 {
    let base = cpucfg(reg::CPUCFG_COUNTER_BASE);
    let ratio = cpucfg(reg::CPUCFG_COUNTER_RATIO);
    let mul = ratio & reg::CPUCFG_COUNTER_MUL_MASK;
    let div = ratio >> reg::CPUCFG_COUNTER_DIV_SHIFT;
    assert!(
        base != 0 && mul != 0 && div != 0,
        "invalid CPUCFG counter frequency"
    );
    base * mul / div
}

/// 返回引导阶段探测并记录在 `BootInfo` 中的硬件计数器频率（单位：Hz）。
#[cfg(feature = "ktest")]
pub fn counter_frequency() -> u64 {
    crate::boot::info().counter_frequency
}

/// 将绝对到期计数器值 `deadline` 转换为相对当前时间的周期差，向上对齐到 4-tick 硬件粒度后写入 `CSR_TCFG` 编程单次定时器中断。
pub fn program_timer(deadline: u64) {
    disable_timer();
    let delta = deadline
        .saturating_sub(read_counter())
        .max(reg::TIMER_QUANTUM);
    let value = delta
        .saturating_add(reg::TIMER_QUANTUM - 1)
        .min(reg::TIMER_MAX_DELTA)
        & !(reg::TIMER_QUANTUM - 1);
    // SAFETY: 1. 当前运行于 PLV0 特权级，仅修改本核私有的 `CSR_TCFG` 定时器配置寄存器；
    // 2. `value` 已按 `TIMER_QUANTUM`（4）对齐并限制在 `TIMER_MAX_DELTA` 范围内，低 2 位仅置位 `TCFG_ENABLE`（单次触发模式）。
    unsafe {
        csr_write::<{ reg::CSR_TCFG }>(value as usize | reg::TCFG_ENABLE);
    }
}

/// 关闭当前 CPU 核心的架构定时器并向 `CSR_TICLR` 写 1 清除已挂起的定时器中断。
pub fn disable_timer() {
    // SAFETY: `CSR_TCFG` 与 `CSR_TICLR` 均为本核私有的定时器控制寄存器，在 PLV0 下写 0 停用定时器、写 `TICLR_CLEAR` 清中断标志不会影响内存映射。
    unsafe {
        csr_write::<{ reg::CSR_TCFG }>(0);
        csr_write::<{ reg::CSR_TICLR }>(reg::TICLR_CLEAR);
    }
}

/// 将物理硬件 CPU 编号 `hardware_id` 编码为 `IOCSR_IPI_SEND` / `IOCSR_MAIL_SEND` 所需的 32 位目标控制字（包含阻塞等待位 `IPI_SEND_BLOCKING`）。
///
/// 若 `hardware_id` 超出 10 位硬件寻址上限（`HARDWARE_CPU_ID_COUNT = 1024`），返回 `None`。
pub(crate) const fn ipi_target(hardware_id: usize) -> Option<u32> {
    if hardware_id < reg::HARDWARE_CPU_ID_COUNT {
        Some(reg::IPI_SEND_BLOCKING | ((hardware_id as u32) << reg::IPI_SEND_CPU_SHIFT))
    } else {
        None
    }
}

/// 通过 `IOCSR_MAIL_SEND` 分高低两个 32 位子槽向目标 CPU（`target`）的第 `mailbox` 号（`0..4`）64 位硬件邮箱写入 `data`。
fn mail_send(target: u32, mailbox: usize, data: usize) {
    assert!(
        mailbox < reg::MAILBOX_COUNT,
        "IOCSR mailbox index is out of range"
    );
    let target = u64::from(target);
    iocsr_write64(
        reg::IOCSR_MAIL_SEND,
        (data as u64 & reg::MAIL_SEND_HIGH_MASK)
            | target
            | (((mailbox * 2 + 1) as u64) << reg::MAIL_SEND_BOX_SHIFT),
    );
    iocsr_write64(
        reg::IOCSR_MAIL_SEND,
        ((data as u64) << reg::MAIL_SEND_DATA_SHIFT)
            | target
            | (((mailbox * 2) as u64) << reg::MAIL_SEND_BOX_SHIFT),
    );
}

/// 向指定硬件编号 `hardware_id` 的辅核（AP）投递启动参数与物理入口地址，并发送核间中断（IPI）将其唤醒。
///
/// - 在 LS2K1000LA 板级平台上：向 Mailbox 1 写入辅核栈顶虚拟地址 `stack_top`，向 Mailbox 2 写入辅核 `CpuLocal` 指针 `local`，向 Mailbox 0 写入 `__secondary_entry_phys`。
/// - 在 QEMU `virt` 平台上：向 Mailbox 1 写入 `CpuBoot` 启动记录的物理地址 `boot_record_physical`，向 Mailbox 0 写入 `__secondary_entry_phys`。
///
/// # Safety
/// - 只能由主核（BSP）对尚未启动的已枚举辅核串行调用一次。
/// - `boot_record_physical` 必须指向物理内存中已完整初始化、对齐且永久有效的 `CpuBoot` 结构。
/// - `CpuBoot` 中关联的辅核栈必须由目标核独占且满足 16 字节 ABI 对齐；`local` 必须指向目标核专属且已初始化的 `CpuLocal` 实例。
pub unsafe fn start_cpu(hardware_id: usize, boot_record_physical: usize) -> Result<(), isize> {
    let target = ipi_target(hardware_id).ok_or(-(Errno::InvalidArgument as isize))?;
    barrier();
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        // SAFETY: 调用方保证 `boot_record_physical` 是已完整初始化且永久有效的 `CpuBoot` 物理地址，通过 `phys_to_virt` 在 DMW1 窗口下可安全读取。
        let record =
            unsafe { &*(phys_to_virt(boot_record_physical) as *const crate::cpu::CpuBoot) };
        mail_send(target, 1, record.stack_top);
        mail_send(target, 2, record.local);
    } else {
        mail_send(target, 1, boot_record_physical);
    }
    let entry: usize;
    // SAFETY: `la.abs` 仅将链接脚本定义的物理入口符号 `__secondary_entry_phys` 的地址加载到寄存器中，不访问内存或栈。
    unsafe {
        asm!("la.abs {entry}, {symbol}", entry = out(reg) entry, symbol = sym __secondary_entry_phys, options(nomem, nostack));
    }
    mail_send(target, 0, entry);
    barrier();
    iocsr_write32(reg::IOCSR_IPI_SEND, target);
    Ok(())
}

/// 在执行完数据栅障后，向硬件编号为 `hardware_id` 的目标 CPU 发送核间中断（触发目标核 IPI 状态寄存器的 Bit 1）。
pub fn send_ipi(hardware_id: usize) {
    let target = ipi_target(hardware_id).expect("IOCSR IPI target is out of range");
    barrier();
    iocsr_write32(reg::IOCSR_IPI_SEND, target | 1);
}

/// 向 QEMU `virt` 平台 PCH-PIC 控制器的指定寄存器偏移 `offset` 写入 32 位值 `value`。
fn pic_write(offset: usize, value: u32) {
    // SAFETY: 1. `BootInfo` 中的 `interrupts.controller` 物理范围已在启动时校验通过，且本文件调用点传入的 `offset` 均按 4 字节对齐并落在控制器窗口内；
    // 2. `mmio_address` 通过永久有效的强序非缓存窗口（`CSR_DMW0`）映射该外设地址，且调用方已持有 `EXTERNAL_CONTROLLER` 锁。
    unsafe {
        core::ptr::write_volatile(
            (mmio_address(crate::boot::info().interrupts.controller.start) + offset) as *mut u32,
            value,
        );
    }
}

/// 从 QEMU `virt` 平台 PCH-PIC 控制器的指定寄存器偏移 `offset` 读取 32 位值。
fn pic_read(offset: usize) -> u32 {
    // SAFETY: 1. `BootInfo` 中的 `interrupts.controller` 物理范围已在启动时校验通过，调用点 `offset` 按 4 字节对齐且位于控制器窗口内；
    // 2. `mmio_address` 通过永久有效的 `CSR_DMW0` 强序窗口访问寄存器。
    unsafe {
        core::ptr::read_volatile(
            (mmio_address(crate::boot::info().interrupts.controller.start) + offset) as *const u32,
        )
    }
}

/// 将 `BootInfo` 中记录的 EXTIOI 父控制器基地址与相对寄存器偏移 `offset` 相加，得到对应的 IOCSR 寄存器编号。
fn ext_reg(offset: usize) -> usize {
    crate::boot::info().interrupts.parent_controller.start + offset
}

/// 保护 QEMU `virt` 平台 PCH-PIC 与 EXTIOI 共享寄存器并发读-改-写操作的自旋锁。
static EXTERNAL_CONTROLLER: SpinLock<()> = SpinLock::new(());

/// 计算外部中断源编号 `source`（`0..64`）在 32 位分组寄存器中的字节偏移量（`0` 或 `4`）与对应位掩码（`1 << (source % 32)`）。
fn source_word(source: u32) -> (usize, u32) {
    assert!(source < crate::boot::info().interrupts.source_count);
    (
        source as usize / u32::BITS as usize * IO_WORD_BYTES,
        1 << (source % u32::BITS),
    )
}

/// 将目标 CPU 的硬件编号 `hardware_id` 编码为 EXTIOI 路由字节（高 4 位为节点号 `node`，低 4 位为节点内 4 核位图 `1 << core`）。
pub(crate) const fn extioi_route(hardware_id: usize) -> Option<u8> {
    let node = hardware_id / EXTIOI_CORES_PER_NODE;
    let core = hardware_id % EXTIOI_CORES_PER_NODE;
    if node < EXTIOI_NODE_COUNT {
        Some(((node as u8) << EXTIOI_ROUTE_NODE_SHIFT) | (1u8 << core))
    } else {
        None
    }
}

/// 根据当前平台类型初始化外部中断控制器（QEMU `virt` 下初始化 PCH-PIC + EXTIOI 级联，LS2K1000LA 板级下初始化双组 LIOINTC），并将所有外部中断源默认屏蔽且路由至主核（BSP）。
///
/// # Safety
/// - 只能由主核（BSP）在本核硬件中断关闭且任何辅核（AP）尚未唤醒的引导阶段调用一次。
/// - `info` 必须指向已校验通过的全局 `BootInfo`，其中记录的中断控制器 MMIO/IOCSR 地址范围必须有效且已由 `CSR_DMW0` 映射。
pub unsafe fn external_init(info: &BootInfo) {
    // Step 1: 若当前运行在 LS2K1000LA 开发板上，直接委派给板级 LIOINTC 驱动完成双组控制器初始化。
    if info.platform == crate::boot::Platform::Ls2k1000la {
        liointc::initialize(info);
        return;
    }

    // Step 2: 校验 QEMU virt 平台下 PCH-PIC 的中断源数量（64 个）以及主核在 EXTIOI 位图模式下的可达性（必须位于 Node 0）。
    assert_eq!(info.interrupts.source_count, PCH_PIC_SOURCE_COUNT);
    let route = extioi_route(info.bsp_hardware_id).expect("EXTIOI target node is out of range");
    // QEMU 传统 EXTIOI 位图模式仅解码 Node 0 的 4 核位图；断言主核位于 Node 0 以确保外部中断被精确投递到主核。
    assert_eq!(
        info.bsp_hardware_id / EXTIOI_CORES_PER_NODE,
        0,
        "EXTIOI bitmap mode cannot route to a remote node"
    );
    assert_eq!(info.interrupts.root_interrupt, EXTIOI_ROOT_INTERRUPT);

    // Step 3: 逐个初始化全部 64 个外部中断源：在 PCH-PIC 中屏蔽中断、启用 HT-MSI 转发、设为高电平触发，并一比一映射到 EXTIOI 向量后路由给主核。
    let _controller = EXTERNAL_CONTROLLER.lock();
    for source in 0..info.interrupts.source_count {
        let (word, bit) = source_word(source);
        pic_write(PCH_PIC_MASK + word, pic_read(PCH_PIC_MASK + word) | bit);
        pic_write(
            PCH_PIC_HTMSI_ENABLE + word,
            pic_read(PCH_PIC_HTMSI_ENABLE + word) | bit,
        );
        pic_write(PCH_PIC_EDGE + word, pic_read(PCH_PIC_EDGE + word) & !bit);
        pic_write(
            PCH_PIC_POLARITY + word,
            pic_read(PCH_PIC_POLARITY + word) & !bit,
        );
        // SAFETY: 1. 循环变量 `source < 64`，`PCH_PIC_ROUTE + source` 与 `PCH_PIC_VECTOR + source` 两个单字节寄存器均完整落在已验证的 PCH-PIC MMIO 窗口内；
        // 2. 当前已持有 `EXTERNAL_CONTROLLER` 自旋锁且辅核尚未启动，独占访问保证无数据竞争。
        unsafe {
            let base = mmio_address(info.interrupts.controller.start);
            core::ptr::write_volatile(
                (base + PCH_PIC_ROUTE + source as usize) as *mut u8,
                PCH_PIC_ROUTE_HT,
            );
            core::ptr::write_volatile(
                (base + PCH_PIC_VECTOR + source as usize) as *mut u8,
                source as u8,
            );
        }
        let route_reg = ext_reg(EXTIOI_ROUTE + (source as usize & !(IO_WORD_BYTES - 1)));
        let shift = source % IO_WORD_BYTES as u32 * u8::BITS;
        let route = u32::from(route) << shift;
        iocsr_write32(
            route_reg,
            (iocsr_read32(route_reg) & !(u32::from(u8::MAX) << shift)) | route,
        );
        iocsr_write32(
            ext_reg(EXTIOI_ENABLE + word),
            iocsr_read32(ext_reg(EXTIOI_ENABLE + word)) & !bit,
        );
        iocsr_write32(
            ext_reg(EXTIOI_BOUNCE + word),
            iocsr_read32(ext_reg(EXTIOI_BOUNCE + word)) | bit,
        );
        iocsr_write32(ext_reg(EXTIOI_ISR + word), bit);
    }

    // Step 4: 配置 EXTIOI 的中断引脚映射（IPMAP）与节点映射（NODEMAP），将两组 32 向量全部汇总到 CPU 的 HWI1 中断引脚。
    for group in 0..(PCH_PIC_SOURCE_COUNT / u32::BITS) as usize {
        let ip_reg = ext_reg(EXTIOI_IPMAP + group / IO_WORD_BYTES * IO_WORD_BYTES);
        let shift = group % IO_WORD_BYTES * u8::BITS as usize;
        iocsr_write32(
            ip_reg,
            (iocsr_read32(ip_reg) & !(u32::from(u8::MAX) << shift)) | (EXTIOI_IPMAP_HWI1 << shift),
        );
        let node_reg = ext_reg(EXTIOI_NODEMAP + group / NODEMAP_ENTRIES_PER_WORD * IO_WORD_BYTES);
        let shift = group % NODEMAP_ENTRIES_PER_WORD * u16::BITS as usize;
        iocsr_write32(
            node_reg,
            (iocsr_read32(node_reg) & !(u32::from(u16::MAX) << shift)) | ((1 << group) << shift),
        );
    }

    // Step 5: 在 `IOCSR_MISC_FUNCTION` 中置位 `IOCSR_EXTIOI_ENABLE` 全局启用 EXTIOI，并执行数据栅障确保配置生效。
    iocsr_write64(
        IOCSR_MISC_FUNCTION,
        iocsr_read64(IOCSR_MISC_FUNCTION) | IOCSR_EXTIOI_ENABLE,
    );
    barrier();
}

/// 在外部中断控制器自旋锁临界区内屏蔽指定的外部中断源 `source`，不影响相邻中断源的使能状态。
pub fn external_mask(source: u32) {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        liointc::set_enabled(source, false);
        return;
    }
    let (word, bit) = source_word(source);
    let _controller = EXTERNAL_CONTROLLER.lock();
    pic_write(PCH_PIC_MASK + word, pic_read(PCH_PIC_MASK + word) | bit);
    let reg = ext_reg(EXTIOI_ENABLE + word);
    iocsr_write32(reg, iocsr_read32(reg) & !bit);
    barrier();
}

/// 在外部中断控制器自旋锁临界区内解除对外部中断源 `source` 的屏蔽（先清除 PCH-PIC 残留锁存并开启 EXTIOI 准入，最后放行 PCH-PIC 掩码）。
pub fn external_unmask(source: u32) {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        liointc::set_enabled(source, true);
        return;
    }
    let (word, bit) = source_word(source);
    let _controller = EXTERNAL_CONTROLLER.lock();
    pic_write(PCH_PIC_CLEAR + word, bit);
    let reg = ext_reg(EXTIOI_ENABLE + word);
    iocsr_write32(reg, iocsr_read32(reg) | bit);
    pic_write(PCH_PIC_MASK + word, pic_read(PCH_PIC_MASK + word) & !bit);
    barrier();
}

/// 读取外部中断控制器的挂起状态与使能掩码，认领（claim）并返回当前编号最小的一个待处理外部中断源。
pub fn external_claim() -> Option<u32> {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        return liointc::claim();
    }
    let _controller = EXTERNAL_CONTROLLER.lock();
    let enabled = iocsr_read32(ext_reg(EXTIOI_ENABLE)) as u64
        | ((iocsr_read32(ext_reg(EXTIOI_ENABLE + IO_WORD_BYTES)) as u64) << u32::BITS);
    let pending = iocsr_read64(ext_reg(EXTIOI_ISR)) & enabled;
    if pending == 0 {
        None
    } else {
        Some(pending.trailing_zeros())
    }
}

/// 完成（complete）指定外部中断源 `source` 的处理周期：在 EXTIOI 中写 1 清除对应的 ISR 挂起位（板级平台则委派给 `liointc::complete`）。
///
/// # Safety
/// - `source` 必须来自当前 CPU 核心早先通过 `external_claim()` 认领且尚未完成的外部中断源。
/// - 调用时该中断源必须处于已屏蔽状态、外设驱动中断处理函数已执行完毕，且调用方持有该中断源的描述符锁。
pub unsafe fn external_complete(source: u32) {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        liointc::complete(source);
        return;
    }
    let (word, bit) = source_word(source);
    let _controller = EXTERNAL_CONTROLLER.lock();
    iocsr_write32(ext_reg(EXTIOI_ISR + word), bit);
    barrier();
}

/// 在用户进程与文件系统已安全停稳后，通知所有其他在线 CPU 核心停机，随后向板级 syscon 电源管理控制器或 QEMU `virt` GED 写入关机命令。
///
/// 调用上下文：必须处于可响应中断的普通内核上下文中，未持有任何自旋锁，也无待完成的跨核调用（CSD）任务。
#[cfg(not(feature = "ktest"))]
pub fn shutdown() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        board_action(crate::boot::info().board.poweroff);
    }
    ged_action(GED_SLEEP_CONTROL, GED_SLEEP_ENABLE | GED_SLEEP_S5)
}

/// 在用户进程与文件系统已安全停稳后，停止所有在线 CPU 核心并关闭本核中断源进入永久休眠，不触发硬件断电。
///
/// 调用上下文与 `shutdown()` 相同。
#[cfg(not(feature = "ktest"))]
pub fn halt() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    disable_stop_sources();
    crate::console::board_stop();
    permanent_idle()
}

/// 在用户进程与文件系统已安全停稳后，停止所有其他在线 CPU 核心，随后向板级 syscon 电源管理控制器或 QEMU `virt` GED 写入系统复位重启命令。
///
/// 调用上下文与 `shutdown()` 相同。
#[cfg(not(feature = "ktest"))]
pub fn restart() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        board_action(crate::boot::info().board.reboot);
    }
    ged_action(GED_RESET, GED_RESET_VALUE)
}

/// `ktest` 紧急停机出口：在遭遇不可恢复的内核异常或 panic 时，不等待其他 CPU 核心或文件系统同步，立即停止当前核（板级）或触发 QEMU GED 关机。
#[cfg(feature = "ktest")]
pub fn emergency_shutdown() -> ! {
    if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        disable_stop_sources();
        crate::console::board_stop_current_cpu();
        permanent_idle();
    }
    ged_action(GED_SLEEP_CONTROL, GED_SLEEP_ENABLE | GED_SLEEP_S5)
}

/// `ktest` 正常测试完成出口：在 QEMU 测试套件运行完毕后，直接向 GED 睡眠控制寄存器写入 S5 断电命令退出模拟器。
#[cfg(feature = "ktest")]
pub fn test_shutdown() -> ! {
    ged_action(GED_SLEEP_CONTROL, GED_SLEEP_ENABLE | GED_SLEEP_S5)
}

/// 供停核 IPI 处理路径调用：关闭当前 CPU 核心的全部唤醒中断源，以 `Release` 内存序置位 `stopped` 确认标志后进入永久 `idle` 循环。
pub(crate) fn stop_current_cpu(stopped: &AtomicBool) -> ! {
    disable_stop_sources();
    stopped.store(true, Ordering::Release);
    permanent_idle()
}

/// 在 CPU 核心进入永久停机状态前，关闭本核全局中断（`CRMD.IE`）、停用架构定时器并清零 `CSR_ECFG` 中断使能掩码。
fn disable_stop_sources() {
    irq_save();
    disable_timer();
    // SAFETY: 当前运行在 PLV0 特权级，`CSR_ECFG` 为本核私有中断配置寄存器；写 0 仅屏蔽本核所有本地与外部中断源，不影响地址空间映射。
    unsafe { csr_write::<{ reg::CSR_ECFG }>(0) };
    barrier();
}

/// 关闭本核中断源后，向 QEMU `virt` 平台 GED 电源控制器的字节偏移 `offset` 写入控制值 `value`，随后进入永久休眠。
fn ged_action(offset: usize, value: u8) -> ! {
    disable_stop_sources();
    // SAFETY: 1. QEMU `virt` 平台的 `GED_BASE`（`0x100e001c`）位于 `CSR_DMW0` 强序非缓存窗口内；
    // 2. 本模块仅传入 `GED_SLEEP_CONTROL`（偏移 0）或 `GED_RESET`（偏移 2），符合 GED 硬件的 8 位寄存器访问规范。
    unsafe {
        core::ptr::write_volatile(mmio_address(GED_BASE + offset) as *mut u8, value);
    }
    barrier();
    permanent_idle()
}

/// 执行板级设备树（DeviceTree）提供的 `syscon` 电源管理寄存器读-改-写动作（关机或重启），随后进入永久休眠。
#[cfg(not(feature = "ktest"))]
fn board_action(action: Option<crate::boot::SysconAction>) -> ! {
    disable_stop_sources();
    crate::console::board_stop();
    if let Some(action) = action {
        let address = mmio_address(action.registers.start) + action.offset as usize;
        // SAFETY: 1. 引导阶段的设备树解析已校验 `action.registers` 范围合法且 `offset` 满足 4 字节对齐；
        // 2. 调用前已通过 `stop_other_online_cpus()` 停止其余核心并关闭了本核中断，对该 32 位 MMIO 寄存器的读-改-写不存在并发竞争。
        unsafe {
            let pointer = address as *mut u32;
            let old = pointer.read_volatile();
            pointer.write_volatile((old & !action.mask) | (action.value & action.mask));
        }
        barrier();
    }
    permanent_idle()
}

/// 在本核中断源已彻底关闭的前提下无限循环执行 `wait_for_interrupt()`，永不返回。
fn permanent_idle() -> ! {
    loop {
        wait_for_interrupt();
    }
}

#[cfg(feature = "ktest")]
/// `ktest` 故障注入辅助函数：在内核态执行全 1 的未定义指令字（`0xffff_ffff`），触发同步非法指令异常（`ECODE_INE`）。
pub fn trigger_fault() -> ! {
    // SAFETY: 本函数专用于测试内核异常捕获路径，故意执行未定义指令触发硬件陷入 `la_kernel_trap`，不破坏任何 Rust 内存安全不变量。
    unsafe {
        asm!(".word 0xffffffff", options(noreturn));
    }
}

/// `ktest` 故障注入辅助函数：在内核态读取既不在 DMW 窗口内、也未建立 TLB 映射的低地址 `0x1000`，触发内核态致命 TLB 重填异常。
#[cfg(feature = "ktest")]
pub fn trigger_unmapped_fault() -> ! {
    // SAFETY: 1. 内核运行于 `CRMD.PG = 1` 分页模式，虚拟地址 `0x1000` 不属于 DMW0/DMW1 且内核从未向 TLB 装填该低地址映射；
    // 2. 通过内联汇编直接执行 `ld.d` 指令触发硬件 TLB 重填陷入 `la_kernel_refill`，避免在 Rust 层面构造并解引用悬垂裸指针。
    unsafe {
        asm!("ld.d {value}, {address}, 0", value = out(reg) _, address = in(reg) 0x1000usize, options(nostack));
    }
    panic!("unmapped load unexpectedly returned");
}

/// 内核态（PLV0）中断与同步异常的 C ABI 总分派入口（由 `trap::la_kernel_trap` 调用）。
///
/// - 若 `ESTAT.Ecode != 0`，说明内核态发生了不可恢复的同步异常（如非法指令、页故障等），立即调用 `ktest::fatal_event` 或 `console::fatal` 打印寄存器现场并停机。
/// - 若 `ESTAT.Ecode == 0`，说明是硬件中断：先检查被打断的 `ERA` 是否落在 `[la_idle_window_start, la_idle_window_end)` 空闲等待窗口内，若是则将 `ERA` 重写为 `la_idle_window_exit` 以防止中断返回后再次进入 `idle 0` 睡死，最后调用 `handle_hardware_interrupt` 分派中断。
#[no_mangle]
extern "C" fn loongarch_trap(frame: &mut KernelTrapFrame) {
    // Step 1: 检查异常主类型码 Ecode；非 0 表示内核态发生同步异常，直接输出致命错误诊断并终止。
    if (frame.estat >> reg::ESTAT_ECODE_SHIFT) & reg::ESTAT_ECODE_MASK != 0 {
        #[cfg(feature = "ktest")]
        crate::ktest::fatal_event(
            if (frame.estat >> reg::ESTAT_ECODE_SHIFT) & reg::ESTAT_ECODE_MASK == reg::ECODE_INE {
                "illegal"
            } else {
                "panic"
            },
            format_args!(
                "synchronous trap pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
                frame.era, frame.gpr[3], frame.estat, frame.badv
            ),
        );
        #[cfg(not(feature = "ktest"))]
        crate::console::fatal(frame.era, frame.gpr[3], frame.estat, frame.badv);
    }

    // Step 2: 若中断发生在 `la_idle_once_asm` 的开中断等待窗口内，将返回地址 `frame.era` 推进到退出标号 `la_idle_window_exit`。
    let (idle_start, idle_end) = (
        core::ptr::addr_of!(la_idle_window_start) as usize,
        core::ptr::addr_of!(la_idle_window_end) as usize,
    );
    if idle_start <= frame.era && frame.era < idle_end {
        frame.era = core::ptr::addr_of!(la_idle_window_exit) as usize;
    }
    // Step 3: 分派并处理定时器、核间中断（IPI）及外部设备中断。
    handle_hardware_interrupt(frame.estat);
}

/// 在本核硬件中断关闭且持有 `HardIrqGuard` 的上下文中，依次检查并处理已挂起且已使能的定时器中断、IPI 核间中断与外部设备中断。
fn handle_hardware_interrupt(estat: usize) {
    let _hardirq = crate::cpu::HardIrqGuard::enter();
    let pending = estat & csr_read::<{ reg::CSR_ECFG }>() & reg::ESTAT_IRQ_MASK;
    if pending & reg::IRQ_TIMER != 0 {
        disable_timer();
        crate::cpu::handle_timer();
    }
    if pending & reg::IRQ_IPI != 0 {
        let bits = iocsr_read32(reg::IOCSR_IPI_STATUS);
        iocsr_write32(reg::IOCSR_IPI_CLEAR, bits);
        barrier();
        crate::cpu::handle_ipi();
    }

    let external = if crate::platform::current() == crate::boot::Platform::Ls2k1000la {
        reg::IRQ_HWI0 | reg::IRQ_HWI1
    } else {
        reg::IRQ_HWI1
    };
    if pending & external != 0 {
        crate::irq::handle_external();
    }
}

/// 内核态（PLV0）致命 TLB 重填异常的 C ABI 报告入口（由 `trap::la_kernel_refill` 在内核态触发未映射地址访问时调用）。
///
/// 提取 `TLBRERA` 中的故障指令地址与最低位 `IsTLBR` 标志，输出诊断信息并终止系统。
#[no_mangle]
extern "C" fn loongarch_refill(frame: &RefillFrame) -> ! {
    // TLB 重填异常不更新普通 `CSR_ESTAT.Ecode`，此处将 `TLBRERA.IsTLBR` 位作为 `cause` 输出以标识重填异常。
    #[cfg(feature = "ktest")]
    crate::ktest::fatal_event(
        "refill",
        format_args!(
            "refill trap pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
            frame.era & !reg::TLBRERA_IS_TLBR,
            frame.gpr[3],
            frame.era & reg::TLBRERA_IS_TLBR,
            frame.badv
        ),
    );
    #[cfg(not(feature = "ktest"))]
    crate::console::fatal(
        frame.era & !reg::TLBRERA_IS_TLBR,
        frame.gpr[3],
        frame.era & reg::TLBRERA_IS_TLBR,
        frame.badv,
    )
}

/// `ktest` 专用测试入口：触发一次本核硬件定时器中断，验证 `la_kernel_trap` 在指定浮点使能模式（`fp`）与 8 位条件标志位图（`fcc`）下能否完整保存并恢复寄存器现场。
///
/// - **测试目标**：检验内核中断入口在 `CSR_EUEN.FPE` 开启或关闭时，对通用整数寄存器、`CSR_KS0`、标量浮点寄存器（`$f0`–`$f31`）、`FCSR0` 及 8 个 `FCC` 条件标志的保存与恢复完整性。
/// - **场景步骤（Given-When-Then）**：
///   1. Given：使用 `IrqSaveGuard` 保存本核中断状态，记录当前 CPU 的 `timer_interrupts` 计数并计算单次定时器的对齐触发周期 `ticks`；
///   2. When：调用 `la_test_register_preservation` 装填整数/浮点测试位图、编程定时器并开中断等待一次定时器中断陷入 `la_kernel_trap`；
///   3. Then：中断返回后校验所有寄存器位图未受损且 `timer_interrupts` 计数已递增。
///
/// # Safety
/// - 当前 CPU 必须已完成 `init_current_cpu` 初始化，拥有合法的内核栈、`CpuLocal` 与定时器中断路由。
/// - 调用期间必须绑定在当前 CPU 上（不可跨核迁移），不得持有与定时器中断处理冲突的自旋锁，且调用点不存在正在使用的浮点上下文。
/// - 调用方在测试结束后必须在同核关中断状态下调用 `crate::time::reprogram_current()` 恢复正常的系统时钟事件。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn ktest_register_pattern(fp: bool, fcc: u8) -> bool {
    use core::sync::atomic::Ordering;
    let _irq = crate::sync::IrqSaveGuard::new();
    let counter = &crate::cpu::current().timer_interrupts;
    let before = counter.load(Ordering::Acquire);
    let ticks = ((counter_frequency() / 10000).max(0x600) as usize
        + (reg::TIMER_QUANTUM as usize - 1))
        & !(reg::TIMER_QUANTUM as usize - 1);
    // SAFETY: 1. 调用方保证当前核已安装好异常入口且固定在本核执行，无活跃浮点上下文或死锁风险；
    // 2. `counter` 指向本核 `CpuLocal` 中永久有效的原子计数器，`ticks` 已按 `TIMER_QUANTUM` 对齐，`fp` 取值为 0 或 1。
    let good =
        unsafe { la_test_register_preservation(counter, ticks, usize::from(fcc), usize::from(fp)) }
            != 0;
    good && before < counter.load(Ordering::Acquire)
}

/// `ktest` 专用测试入口：在普通可调度内核任务中调用 `la_test_task_context_preservation` 探针，验证 `yield_now` 前后被调用者保存寄存器与栈指针的完整性。
#[cfg(feature = "ktest")]
pub fn test_task_context_preservation() -> bool {
    assert!(
        crate::task::current_task_id().is_some(),
        "context probe requires a running task"
    );
    // SAFETY: 1. 上方断言已确认当前处于普通可调度内核任务上下文；
    // 2. 传入 `crate::sched::yield_now` 作为切换函数入口，汇编探针会完整保存并恢复调用者的 Callee-saved 寄存器与 16 字节对齐栈帧。
    unsafe { la_test_task_context_preservation(crate::sched::yield_now as usize) != 0 }
}

/// `ktest` 专用测试入口：在不开启浮点扩展单元（`fp = 0`）的纯整数模式下触发一次硬件定时器中断，验证全部通用整数寄存器、`$sp` 与 `CSR_KS0` 在中断前后的保存恢复完整性。
///
/// # Safety
/// - 当前 CPU 必须已安装好异常入口、`CpuLocal` 与定时器中断，且调用点未持有自旋锁、抢占守卫或活跃浮点状态。
/// - 测试期间会临时开启本核中断并编程本核定时器，调用上下文不得允许任务跨核迁移。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn test_integer_register_preservation() -> usize {
    let saved = irq_save();
    let counter = &crate::cpu::current().timer_interrupts;
    let before = counter.load(core::sync::atomic::Ordering::Acquire);
    let ticks = ((counter_frequency() / 1000).max(reg::TIMER_QUANTUM) as usize
        + (reg::TIMER_QUANTUM as usize - 1))
        & !(reg::TIMER_QUANTUM as usize - 1);
    // SAFETY: 1. 调用方保证当前核中断向量与内核栈可正常响应定时器中断，`counter` 为本核永久有效的原子计数器；
    // 2. 传入 `fp = 0` 使测试汇编完全跳过浮点指令，并在返回前恢复调用者的整数寄存器与关中断状态。
    let good = unsafe { la_test_register_preservation(counter, ticks, 0, 0) };
    assert!(good != 0, "integer timer trap register preservation failed");
    assert!(
        before < counter.load(core::sync::atomic::Ordering::Acquire),
        "integer timer trap was not observed"
    );
    crate::time::reprogram_current();
    // SAFETY: `saved` 来自本函数入口在同一 CPU 上调用的 `irq_save()`，且定时器测试汇编已恢复关中断现场并重编程了生产时钟。
    unsafe { irq_restore(saved) };
    1
}
