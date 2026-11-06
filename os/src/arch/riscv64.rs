//! RISC-V 64 位（Sv39）架构核心支持模块。
//!
//! # 模块职责与架构定位
//! 本模块作为 `riscv64` 架构层的根模块（通过 `crate::arch` 导出），向上层内核子系统（引导、内存管理、
//! 进程调度、中断与定时器、设备驱动）提供统一的硬件抽象接口，并协调 `entry`、`mmu`、`trap`、`switch`、
//! `user`、`signal`、`uaccess`、`rtc` 等子模块。支持 QEMU `virt` 虚拟机与 StarFive VisionFive 2
//! （JH7110）真实开发板两类平台。
//!
//! # 核心机制与硬件位域布局
//! 1. **Sv39 页表项（PTE）与 `satp` 寄存器格式**：
//!    采用三级页表（L2 1GiB、L1 2MiB、L0 4KiB），虚拟地址有效位宽 39 位（高半核高 25 位全为 1）：
//!    ```text
//!    Sv39 PTE (64-bit):
//!    +-------+-------+-+---+-+---+-+-+-+-+-+-+-+-+
//!    | 63:54 | 53:10 |9| 8 |7| 6 |5|4|3|2|1|0|
//!    +-------+-------+-+---+-+---+-+-+-+-+-+-+-+-+
//!    | Rsrvd |  PPN  | RSW |D| A |G|U|X|W|R|V|
//!    +-------+-------+-+---+-+---+-+-+-+-+-+-+-+-+
//!      - V (bit 0): 有效位
//!      - R/W/X (bits 1..=3): 读/写/执行权限；全 0 表示指向下一级页表的非叶节点
//!      - U (bit 4): 用户态可访问
//!      - A/D (bits 6..=7): 访问位（Accessed）与脏位（Dirty）
//!      - PPN (bits 53:10): 44 位物理页号（等于物理地址右移 12 位后再左移 10 位）
//!
//!    satp CSR (64-bit):
//!    +---------------+------------------+----------------------------+
//!    | 63:60 (MODE)  | 59:44 (ASID)     | 43:0 (PPN)                 |
//!    +---------------+------------------+----------------------------+
//!    | 8 = Sv39      | 0 (未使用)       | 根页表 4KiB 物理页号       |
//!    +---------------+------------------+----------------------------+
//!    ```
//! 2. **最终内核页表（`FINAL_TABLES`）静态预算与 `no-map` 扣除**：
//!    主核在 `finish_boot_mappings` 中根据设备树（DTB）解析出的真实 RAM 区间、保留的 `no-map` 空洞以及
//!    板载外设（UART、PLIC、RTC、SiFive ccache、时钟/复位/引脚控制器、MMC、GMAC、AHCI、VirtIO、PCI）
//!    构建最终内核页表。满足 2MiB 对齐且跨度不少于 2MiB 的区间优先采用 L1 2MiB 大页，其余拆分为 L0 4KiB 小页。
//! 3. **PLIC 外部中断控制器固定路由与掩码策略**：
//!    `external_init` 将所有合法外部中断源的使能位固定绑定到主核（CPU 0）的 S-mode context，并将阈值设为 0。
//!    运行期的中断屏蔽（`external_mask`）与解屏蔽（`external_unmask`）通过将对应中断源的 `priority` 设为 `0` 或 `1`
//!    实现，从而在不改动使能路由位图的前提下保留已完成 `claim` 的中断源执行 `complete` 的资格。
//! 4. **SBI 固件调用边界**：
//!    仅使用标准 SBI v0.2+ 的四个扩展：`TIME`（定时器编程）、`IPI`（核间软件中断）、`HSM`（从核唤醒）与
//!    `SRST`（整机关机与冷重启）。
//!
//! # 锁获取顺序与并发契约
//! - `FINAL_TABLES` 与 `RV_FINAL_SATP` 仅在主核关中断且从核尚未通过 HSM 唤醒前由主核独占构建并发布；
//!   发布后在内核生命周期内只读。
//! - PLIC 的每个外部中断源 `priority` 寄存器由上层 `crate::irq` 中对应中断号描述符的自旋锁串行化保护；
//!   PLIC 的 S-mode `claim/complete` 寄存器按 CPU 分离，仅由本核在关中断的硬中断上下文（`HardIrqGuard`）中访问。
//! - `tp` 寄存器在 `install_cpu_local` 完成后恒指向当前物理核的 `CpuLocal`，严禁跨核共享或修改。
use crate::{
    boot::{
        BootInfo, Region, MAX_DEVICE_CLOCKS, MAX_DEVICE_PINS, MAX_DEVICE_RESETS,
        MAX_MEMORY_REGIONS, MAX_PCI_RANGES, MAX_RESERVED_REGIONS, MAX_VIRTIO_MMIO,
        PCI_SPACE_CODE_MASK, PCI_SPACE_MEM32, PCI_SPACE_MEM64,
    },
    cpu::CpuLocal,
    mm::{PAGE_SHIFT, PAGE_SIZE},
    platform::LINK_LAYOUT,
    sched::context::TaskContext,
};
use core::{
    arch::asm,
    mem::{size_of, MaybeUninit},
    ptr,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

mod entry;
mod mmu;
#[cfg(feature = "ktest")]
mod registers;
mod rtc;
mod signal;
mod switch;
mod trap;
mod uaccess;
mod user;
pub use mmu::{
    activate_kernel_root, activate_user_root, flush_user_translations, initialize_user_root,
    sync_user_icache, user_empty_entry, user_entry_is_table, user_entry_physical, user_leaf_dirty,
    user_leaf_entry, user_leaf_mkclean, user_leaf_permissions, user_table_entry,
    SIGNAL_RETURN_PAGE, USER_ADDRESS_END, USER_LEVELS, USER_STACK_TOP,
};
#[cfg(feature = "ktest")]
use registers::rv_register_roundtrip;
pub use rtc::board_rtc_calendar;
pub(crate) use signal::SIGNAL_MIN_STACK;
use switch::rv_switch_context;
#[cfg(feature = "ktest")]
use switch::rv_task_context_roundtrip;
use trap::rv_trap_entry;
pub use uaccess::{copy_user_bytes, load_user_u32, store_user_u32};
#[cfg(feature = "ktest")]
pub use user::{exit_offset, fault_offset, loop_offset, loop_spin_offset, user_code};
pub use user::{run_user, signal_return_code, UserContext};

// Sv39 页表几何参数与页表项（PTE）位域掩码：每级索引占 9 位（512 项），PPN 从 bit 10 开始。
pub(crate) const PT_INDEX_BITS: usize = 9;
const PT_ENTRIES: usize = PAGE_SIZE / size_of::<usize>();
pub(crate) const PT_INDEX_MASK: usize = PT_ENTRIES - 1;
const PT_HALF_ENTRIES: usize = PT_ENTRIES / 2;
pub(crate) const SV39_LEVELS: usize = 3;
const SV39_VA_BITS: usize = PAGE_SHIFT + PT_INDEX_BITS * SV39_LEVELS;
const PMD_SHIFT: usize = PAGE_SHIFT + PT_INDEX_BITS;
const PMD_SIZE: usize = 1 << PMD_SHIFT;
const PGDIR_SHIFT: usize = PMD_SHIFT + PT_INDEX_BITS;
pub(crate) const PTE_PPN_SHIFT: usize = 10;
const PHYSICAL_TO_PTE_SHIFT: usize = PAGE_SHIFT - PTE_PPN_SHIFT;

pub(crate) const PTE_VALID: usize = 1 << 0;
pub(crate) const PTE_READ: usize = 1 << 1;
pub(crate) const PTE_WRITE: usize = 1 << 2;
pub(crate) const PTE_EXEC: usize = 1 << 3;
const PTE_USER: usize = 1 << 4;
const PTE_ACCESSED: usize = 1 << 6;
const PTE_DIRTY: usize = 1 << 7;
const PTE_LEAF: usize = PTE_READ | PTE_WRITE | PTE_EXEC;
const KERNEL_DATA_FLAGS: usize = PTE_READ | PTE_WRITE | PTE_ACCESSED | PTE_DIRTY;

pub(crate) const SATP_MODE_SHIFT: usize = 60;
pub(crate) const SATP_MODE_SV39: usize = 8 << SATP_MODE_SHIFT;
pub(crate) const SATP_PPN_MASK: usize = (1usize << 44) - 1;

pub(crate) const SR_SIE: usize = 1 << 1;
pub(crate) const SR_SPIE: usize = 1 << 5;
pub(crate) const SR_SPP: usize = 1 << 8;
pub(crate) const SR_FS: usize = 3 << 13;
pub(crate) const SR_FS_DIRTY: usize = SR_FS;
pub(crate) const SR_UXL_64: usize = 2 << 32;

const CAUSE_IRQ_FLAG: usize = 1 << (usize::BITS - 1);
const IRQ_S_SOFT: usize = 1;
const IRQ_S_TIMER: usize = 5;
pub(crate) const IRQ_S_EXT: usize = 9;
const SIE_KERNEL: usize = (1 << IRQ_S_SOFT) | (1 << IRQ_S_TIMER) | (1 << IRQ_S_EXT);
const SIP_SSIP: usize = 1 << IRQ_S_SOFT;

pub(crate) const EXC_INST_MISALIGNED: usize = 0;
pub(crate) const EXC_INST_ACCESS: usize = 1;
pub(crate) const EXC_BREAKPOINT: usize = 3;
pub(crate) const EXC_LOAD_MISALIGNED: usize = 4;
pub(crate) const EXC_LOAD_ACCESS: usize = 5;
pub(crate) const EXC_STORE_MISALIGNED: usize = 6;
pub(crate) const EXC_STORE_ACCESS: usize = 7;
#[cfg(feature = "ktest")]
const EXC_INST_ILLEGAL: usize = 2;
const EXC_SYSCALL: usize = 8;
const EXC_INST_PAGE_FAULT: usize = 12;
const EXC_LOAD_PAGE_FAULT: usize = 13;
const EXC_STORE_PAGE_FAULT: usize = 15;

pub(crate) const SIFIVE_CCACHE_LINE_SIZE: usize = 64;
pub(crate) const SIFIVE_CCACHE_FLUSH64: usize = 0x200;
const SIFIVE_CCACHE_FLUSH64_END: usize = SIFIVE_CCACHE_FLUSH64 + size_of::<u64>();

pub(crate) const PLIC_MAX_SOURCES: u32 = 1024;
pub(crate) const PLIC_ENABLE_BASE: usize = 0x2000;
pub(crate) const PLIC_ENABLE_STRIDE: usize = 0x80;
pub(crate) const PLIC_CONTEXT_BASE: usize = 0x20_0000;
pub(crate) const PLIC_CONTEXT_STRIDE: usize = 0x1000;
pub(crate) const PLIC_CLAIM_OFFSET: usize = size_of::<u32>();
const PLIC_REGISTER_BYTES: usize = size_of::<u32>();
const PLIC_ENABLE_BITS: usize = u32::BITS as usize;
const PLIC_ENABLE_WORDS: usize = PLIC_ENABLE_STRIDE / PLIC_REGISTER_BYTES;

const SBI_EXT_TIME: usize = 0x5449_4d45;
const SBI_EXT_IPI: usize = 0x73_5049;
const SBI_EXT_HSM: usize = 0x48_534d;
const SBI_EXT_SRST: usize = 0x5352_5354;
const SBI_EXT_TIME_SET_TIMER: usize = 0;
const SBI_EXT_IPI_SEND_IPI: usize = 0;
const SBI_EXT_HSM_HART_START: usize = 0;
const SBI_EXT_SRST_RESET: usize = 0;
const SBI_SRST_SHUTDOWN: usize = 0;
#[cfg(not(feature = "ktest"))]
const SBI_SRST_COLD_REBOOT: usize = 1;
const SBI_SRST_REASON_NONE: usize = 0;

/// 当前编译目标的架构名称常量。
pub const NAME: &str = "riscv64";

extern "C" {
    static __kernel_phys_end: u8;
    static __secondary_entry_phys: u8;
}

// RISC-V medany 代码模型的 PC 相对寻址范围（±2GiB）无法直接跨越虚拟高半核与物理低地址；
// 此宏将符号绝对值放入当前代码段的 8 字节对齐文字池（literal pool）并通过 `ld` 读出。
macro_rules! address {
    ($name:ident) => {{
        let value: usize;
        // SAFETY:
        // 1. 文字池标签 `2f` 与指令同处已映射的内核只读代码段，且通过 `.balign 8` 保证 8 字节对齐。
        // 2. 链接器负责将 `$name` 的符号地址填入 `.quad`，汇编随后立即 `j 3f` 跳过该数据字，不会将其作为指令执行。
        unsafe {
            asm!("lla {value}, 2f", "ld {value}, 0({value})", "j 3f",
                 ".balign 8", "2: .quad {symbol}", "3:",
                 value = out(reg) value, symbol = sym $name,
                 options(nostack, readonly));
        }
        value
    }};
}

/// 4KiB 对齐的 Sv39 单级页表结构（内部元组字段 `0` 为按虚拟地址 9 位索引排列的 512 个 64 位 PTE 数组）。
#[repr(C, align(4096))]
struct PageTable([usize; PT_ENTRIES]);
#[no_mangle]
#[link_section = ".boot.tables"]
static mut RV_BOOT_ROOT: PageTable = PageTable([0; PT_ENTRIES]);
#[no_mangle]
#[link_section = ".boot.tables"]
static mut RV_BOOT_IMAGE: PageTable = PageTable([0; PT_ENTRIES]);
#[link_section = ".boot.tables"]
static mut FINAL_TABLES: [PageTable; MAX_BOOT_TABLES] =
    [const { PageTable([0; PT_ENTRIES]) }; MAX_BOOT_TABLES];
#[no_mangle]
static mut RV_FINAL_SATP: usize = 0;

/// 内核态陷入（S-mode Trap）时在当前内核栈上保存的完整寄存器现场。
///
/// 由 `trap::rv_trap_entry` 汇编入口按固定偏移填充，并作为可变引用传递给 [`rv_handle_trap`]。
#[repr(C, align(16))]
struct KernelTrapFrame {
    /// 32 个通用整数寄存器 `x0`–`x31`（其中 `gpr[0]` 恒写 0，`gpr[2]` 保存陷入前的内核栈指针 `sp`）。
    gpr: [usize; 32],
    /// 陷入发生时的程序计数器（`sepc` CSR）。
    sepc: usize,
    /// 陷入发生时的处理器状态寄存器（`sstatus` CSR）。
    sstatus: usize,
    /// 陷入原因寄存器（`scause` CSR，最高位区分异步中断与同步异常）。
    scause: usize,
    /// 陷入附加值寄存器（`stval` CSR，如缺页地址或非法指令内容）。
    stval: usize,
    /// 32 个 64 位双精度浮点寄存器 `f0`–`f31`。
    /// 当陷入前 `sstatus.FS` 为 `Off` 时汇编跳过保存，因此使用 `MaybeUninit` 避免 Rust 侧产生未初始化读取。
    fpr: MaybeUninit<[u64; 32]>,
    /// 浮点控制与状态寄存器（`fcsr`）。同样仅在陷入前 `sstatus.FS != Off` 时由汇编保存。
    fcsr: MaybeUninit<usize>,
}

/// 在本地中断关闭的调度切换临界区内切换整数调用现场，恢复后继续使用目标 CPU 的 `CpuLocal`。
///
/// # Safety
/// 1. 调用方必须已关闭本地中断（`!irq_enabled()`），且不得持有普通自旋锁、`IrqSaveGuard` 或抢占守卫跨越此调用。
/// 2. 当前 CPU 的 `sstatus.FS` 必须为 `Off`（普通内核任务不持有未保存的浮点寄存器状态）。
/// 3. `prev` 与 `next` 必须指向有效、对齐且互不重叠的 [`TaskContext`]：`prev` 由当前切出任务独占可写，
///    `next` 已完成初始化且其保存的栈指针按 16 字节对齐、返回地址有效；两个任务的内核栈必须在各自恢复或退出前保持存活。
pub unsafe fn switch_context(prev: *mut TaskContext, next: *const TaskContext) {
    assert!(!irq_enabled(), "task switch requires IRQ disabled");
    let status: usize;
    // SAFETY: 当前运行在 S-mode，`csrr sstatus` 仅读取本核状态寄存器以断言浮点单元处于关闭状态，不读写内存或栈。
    unsafe { asm!("csrr {0}, sstatus", out(reg) status, options(nomem, nostack)) };
    assert_eq!(status & SR_FS, 0, "task switch requires FS Off");
    // SAFETY: 前置 `assert!` 已确认本地中断关闭且 `sstatus.FS == Off`；其余指针有效性、栈对齐与生命周期由调用方契约保证。
    unsafe { rv_switch_context(prev, next) };
}

/// 在可调度的普通内核任务中跨越一次真实的 `yield_now` 验证全部被调用者保存寄存器、`sp` 与返回地址完整性。
#[cfg(feature = "ktest")]
pub fn test_task_context_preservation() -> bool {
    // SAFETY:
    // 1. 当前由 `ktest` 在可调度的普通内核任务上下文中调用，Rust 函数入口保证当前栈满足 16 字节 ABI 对齐。
    // 2. 内部调用的 `yield_now` 与 `switch_context` 会验证任务状态、中断开关与 `sstatus.FS == Off`。
    // 3. `rv_task_context_roundtrip` 在返回前会完整恢复本函数的被调用者保存寄存器，并保持当前 CPU 的 `tp` 不变。
    unsafe { rv_task_context_roundtrip() != 0 }
}

/// 执行一致性 DMA（Coherent DMA）写内存屏障，确保 CPU 先前对 DMA 缓冲区的写入在后续描述符发布前全局可见。
///
/// 若后续操作涉及通知外设 MMIO 寄存器，还须配合完整的 [`dma_barrier`]。
pub(super) fn dma_write_barrier() {
    // SAFETY: S-mode 下执行 `fence w, w` 仅对内存写操作排序，不修改寄存器或地址空间；
    // 省略 `nomem`/`readonly` 选项可同时充当编译器内存屏障。
    unsafe { asm!("fence w, w", options(nostack)) };
}

/// 执行一致性 DMA（Coherent DMA）读内存屏障，确保在观察到设备完成标志后再读取 DMA 缓冲区负载数据。
pub(super) fn dma_read_barrier() {
    // SAFETY: S-mode 下执行 `fence r, r` 仅对内存读操作排序；省略 `nomem`/`readonly` 同时阻止编译器重排读访存。
    unsafe { asm!("fence r, r", options(nostack)) };
}

/// 对普通内存（`rw`）与外设 MMIO（`io`）执行全序双向屏障（`fence iorw, iorw`）。
pub(super) fn dma_barrier() {
    // SAFETY: S-mode 下执行 `fence iorw, iorw` 强制排序屏障前后的全部内存与 MMIO 读写，同时阻止编译器跨屏障重排访存。
    unsafe { asm!("fence iorw, iorw", options(nostack)) };
}

/// 对非一致性 DMA 缓冲区关联的 64 字节对齐物理缓存行执行 SiFive L2 Composable Cache（ccache）写回并失效（Clean + Invalidate）。
///
/// 仅在 VisionFive 2（JH7110）平台上受支持；若平台不匹配、物理区间未按缓存行对齐或超出有效 RAM 范围则返回 `Err`。
pub fn dma_cache_flush(physical: usize, length: usize) -> Result<(), &'static str> {
    // Step 1: 校验当前平台为 VisionFive 2，且物理区间满足 64 字节缓存行对齐、FLUSH64 窗口可用并完全落在有效 RAM 内。
    let info = crate::boot::info();
    if info.platform != crate::boot::Platform::VisionFive2 {
        return Err("RISC-V noncoherent DMA cache maintenance is unsupported");
    }
    if info.board.cache_line_size != SIFIVE_CCACHE_LINE_SIZE
        || (physical | length) & (SIFIVE_CCACHE_LINE_SIZE - 1) != 0
    {
        return Err("ccache DMA range is not cacheline aligned");
    }
    let end = physical
        .checked_add(length)
        .ok_or("ccache DMA range overflows")?;
    let resource = info.board.ccache;
    if resource.start & (size_of::<u64>() - 1) != 0
        || resource.end - resource.start < SIFIVE_CCACHE_FLUSH64_END
    {
        return Err("ccache FLUSH64 register is unavailable");
    }
    if !info.memory[..info.memory_count]
        .iter()
        .any(|ram| ram.start <= physical && end <= ram.end)
    {
        return Err("ccache DMA range is outside RAM");
    }

    // Step 2: 在写回缓存行前后各执行一次全序 I/O 屏障，并按 64 字节步长向 ccache FLUSH64 寄存器写入物理地址。
    dma_barrier();
    for line in (physical..end).step_by(SIFIVE_CCACHE_LINE_SIZE) {
        // SAFETY:
        // 1. 前置检查已确认当前平台为 VisionFive 2，`resource` 包含完整的 `FLUSH64` 寄存器窗口且起始地址 8 字节对齐。
        // 2. `finish_boot_mappings` 已在最终内核页表中永久映射 `board.ccache` 的 MMIO 窗口。
        // 3. `physical..end` 已验证落在有效物理 RAM 内且按 64 字节缓存行对齐，向 `FLUSH64` 写入物理地址仅触发硬件缓存刷回与失效。
        unsafe {
            ((mmio_address(resource.start) + SIFIVE_CCACHE_FLUSH64) as *mut u64)
                .write_volatile(line as u64)
        };
    }
    dma_barrier();
    Ok(())
}

/// 从固件传入的原始启动参数数组（`a0`–`a3`）中提取主核（BSP）的硬件 hart ID（位于 `a0`）。
pub fn boot_hardware_id(args: [usize; 4]) -> usize {
    args[0]
}

/// 将物理 RAM 地址转换为高半核线性直映区（Direct Map）虚拟地址。
///
/// 仅执行常数偏移算术换算，调用方在解引用前需确保目标物理页属于已建立直映的有效 RAM（且不在 `no-map` 保留区内）。
pub fn phys_to_virt(pa: usize) -> usize {
    (LINK_LAYOUT.direct_map_virt_base as usize)
        .wrapping_add(pa.wrapping_sub(LINK_LAYOUT.ram_phys_base as usize))
}

/// 将内核镜像高地址或高半核 RAM 线性直映区虚拟地址转换为对应的物理地址。
pub fn virt_to_phys(va: usize) -> usize {
    if LINK_LAYOUT.kernel_link_virt as usize <= va {
        va - LINK_LAYOUT.kernel_link_virt as usize + LINK_LAYOUT.kernel_load_phys as usize
    } else {
        va.wrapping_sub(LINK_LAYOUT.direct_map_virt_base as usize)
            .wrapping_add(LINK_LAYOUT.ram_phys_base as usize)
    }
}

/// 将外设寄存器物理地址转换为高半核 MMIO 虚拟地址窗口中的对应地址。
///
/// 仅执行地址加偏计算，调用方在解引用前需确保该外设区间已由 `finish_boot_mappings` 映射。
pub fn mmio_address(pa: usize) -> usize {
    (LINK_LAYOUT.mmio_virt_base as usize).wrapping_add(pa)
}

/// 获取静态最终页表池 `FINAL_TABLES` 中第 `index` 张页表的首元素裸指针（不创建 Rust 引用以避免别名冲突）。
///
/// # Safety
/// 1. `index` 必须严格小于 [`MAX_BOOT_TABLES`]。
/// 2. 调用方（主核引导阶段）必须独占尚未激活或正在构建的 `FINAL_TABLES` 池。
unsafe fn table(index: usize) -> *mut usize {
    // SAFETY: 调用方契约保证 `index < MAX_BOOT_TABLES` 且当前主核独占 `FINAL_TABLES` 静态数组。
    unsafe {
        ptr::addr_of_mut!(FINAL_TABLES)
            .cast::<PageTable>()
            .add(index)
            .cast()
    }
}

/// 将页对齐的物理地址区间 `[pa, end)` 映射到以 `va` 起始的连续 Sv39 高半核虚拟地址区间，按需从 `FINAL_TABLES` 分配子页表。
///
/// 若当前 2MiB 边界满足大页条件（见 [`huge_leaf`]），则直接写入 L1 2MiB 大页叶表项；否则向下分配 L0 页表并写入 4KiB 叶表项。
///
/// # Safety
/// 1. 只能由主核在 `FINAL_TABLES` 尚未写入 `satp` 前独占调用。
/// 2. `va`、`pa`、`end` 必须按 4KiB 页对齐，满足 `pa <= end`，且 `[va, va + (end - pa))` 属于合法的 Sv39 高半核规范（canonical）虚拟地址。
/// 3. 根表 `FINAL_TABLES[0]` 及已分配的各级非叶表项必须全部指向 `FINAL_TABLES` 池内的页表，且本次映射不得与已有叶表项重叠。
/// 4. `*next` 必须为 `1..=MAX_BOOT_TABLES` 中第一张尚未分配的页表索引。
unsafe fn map_range(mut va: usize, mut pa: usize, end: usize, flags: usize, next: &mut usize) {
    // SAFETY:
    // 1. 调用方契约保证主核独占 `FINAL_TABLES`，且 `va`/`pa`/`end` 页对齐、不与既有叶表项重叠。
    // 2. 各级索引通过 `& PT_INDEX_MASK` 限制在 `0..512` 内；分配新子表前通过 `assert!(*next < MAX_BOOT_TABLES)` 检查容量。
    // 3. 非叶表项中的物理页号均由本函数从 `FINAL_TABLES` 池内页表换算写入，启动阶段的临时页表已覆盖整个镜像与表池的线性直映访问。
    unsafe {
        while pa < end {
            let huge = huge_leaf(va, pa, end);
            let target = if huge { 1 } else { 0 };
            let mut current = table(0);
            for level in (target + 1..SV39_LEVELS).rev() {
                let slot =
                    current.add((va >> (PAGE_SHIFT + PT_INDEX_BITS * level)) & PT_INDEX_MASK);
                let mut pte = ptr::read(slot);
                if pte == 0 {
                    assert!(
                        *next < MAX_BOOT_TABLES,
                        "Sv39 boot page table pool exhausted"
                    );
                    let child = table(*next);
                    *next += 1;
                    pte = (virt_to_phys(child as usize) >> PHYSICAL_TO_PTE_SHIFT) | PTE_VALID;
                    ptr::write(slot, pte);
                }
                assert_eq!(pte & PTE_LEAF, 0, "Sv39 mapping levels overlap");
                current = phys_to_virt((pte >> PTE_PPN_SHIFT) << PAGE_SHIFT) as *mut usize;
            }
            let leaf = current.add((va >> (PAGE_SHIFT + PT_INDEX_BITS * target)) & PT_INDEX_MASK);
            assert_eq!(ptr::read(leaf), 0, "Sv39 leaf mappings overlap");
            ptr::write(leaf, (pa >> PHYSICAL_TO_PTE_SHIFT) | flags | PTE_VALID);
            let step = if huge { PMD_SIZE } else { PAGE_SIZE };
            pa += step;
            va += step;
        }
    }
}

/// 判断当前映射游标 `(va, pa)` 到终点 `end` 的剩余前缀是否满足 L1 2MiB 大页映射条件。
///
/// 页表池容量预算（[`mapping_table_count`]）与实际写入页表项（[`map_range`]）共用此判定规则。
fn huge_leaf(va: usize, pa: usize, end: usize) -> bool {
    (va | pa) & (PMD_SIZE - 1) == 0 && PMD_SIZE <= end - pa
}

/// 启动阶段待写入最终内核页表的单段连续虚拟到物理映射描述符。
#[derive(Clone, Copy)]
struct BootMapping {
    /// 映射起始的 Sv39 高半核虚拟地址（按 4KiB 页对齐）。
    virtual_start: usize,
    /// 映射对应的半开物理地址区间 `[start, end)`（按 4KiB 页对齐）。
    physical: Region,
    /// 页表叶项权限与状态标志位（如 `KERNEL_DATA_FLAGS` 或叠加 `PTE_EXEC`）。
    flags: usize,
}

// 每个外设初始化配置（DeviceSetup）最多引用两组（entries 与 assigned）各 8 个时钟及其父时钟源、
// 3 个复位控制器和 16 个引脚复用（pinctrl）控制器窗口。
// UART 仅包含时钟配置，而 RTC、MMC、GMAC 各自拥有完整的 DeviceSetup；重复的控制器窗口在加入映射表时会自动合并。
// MMC 卡检测 GPIO 额外预留 1 个独立的控制器窗口配额。
const MAX_CLOCK_MAPPINGS: usize = 2 * (MAX_DEVICE_CLOCKS + MAX_DEVICE_CLOCKS);
const MAX_SETUP_MAPPINGS: usize = MAX_CLOCK_MAPPINGS + MAX_DEVICE_RESETS + MAX_DEVICE_PINS;
// 固定板级外设窗口共 6 个（UART、PLIC、RTC、ccache、PMC、PLL）；板载原生外设另含 MMC/GMAC 各 2 个窗口与 AHCI 1 个窗口。
const MAX_FIXED_DEVICE_MAPPINGS: usize = 6;
const MAX_NATIVE_DEVICE_MAPPINGS: usize = 2 + 2 + 1;
const MAX_SETUP_DEVICES: usize = 3;
const MAX_BOOT_MAPPINGS: usize = 1
    + MAX_MEMORY_REGIONS
    + MAX_RESERVED_REGIONS
    + MAX_FIXED_DEVICE_MAPPINGS
    + MAX_VIRTIO_MMIO
    + 1
    + MAX_PCI_RANGES
    + MAX_NATIVE_DEVICE_MAPPINGS
    + MAX_CLOCK_MAPPINGS
    + MAX_SETUP_DEVICES * MAX_SETUP_MAPPINGS
    + 1;
// 互不重叠的物理 RAM 区间在扣除 no-map 保留区并集后，每段 no-map 空洞至多将一个连续 RAM 区间一分为二，
// 因此分裂后的总片段数不超过 `MAX_MEMORY_REGIONS + MAX_RESERVED_REGIONS`，且每段仍保持 VA 与 PA 的 2MiB 页内偏移一致。
// Sv39 高半核最多使用 256 张 L1 页表；每个映射片段的首尾非 2MiB 对齐边界至多各消耗 1 张 L0 页表。
const MAX_BOOT_BRANCHES: usize = PT_HALF_ENTRIES + 2 * MAX_BOOT_MAPPINGS;
const MAX_BOOT_TABLES: usize = 1 + MAX_BOOT_BRANCHES;
// 页表预算统计时，L0 与 L1 的去重键共用同一个数组；最高位标签仅用于区分 L0 与 L1 层级，不会写入硬件 PTE。
const BOOT_BRANCH_L0_TAG: usize = 1 << (usize::BITS - 1);

/// 遍历 [`BootInfo`] 中标记为 `no-map` 的保留内存区，并将每段区间向外扩展到 4KiB 页边界。
///
/// 向外取整可确保任何与 `no-map` 保留区存在部分重叠的物理页都不会被映射到内核线性直映区。
fn no_map_regions(info: &BootInfo) -> impl Iterator<Item = Region> + '_ {
    info.reserved[..info.reserved_count]
        .iter()
        .zip(&info.reserved_no_map[..info.reserved_count])
        .filter(|(_, no_map)| **no_map)
        .map(|(region, _)| {
            assert!(region.start < region.end, "no-map region is empty");
            Region {
                start: region.start & !(PAGE_SIZE - 1),
                end: region
                    .end
                    .checked_add(PAGE_SIZE - 1)
                    .expect("no-map page rounding overflows")
                    & !(PAGE_SIZE - 1),
            }
        })
}

/// 在构建最终页表前校验所有 `no-map` 物理区间是否与正在使用的内核镜像、DTB 或 initrd 发生冲突。
///
/// 内核镜像区间包含静态页表池、主核启动栈及全部 `.bss` 段；一旦被 `no-map` 挖空会导致切换页表后立即缺页崩溃。
/// 未标记 `no-map` 的普通保留区仍保留线性直映，仅由物理页分配器跳过。
pub(crate) fn validate_boot_no_map(info: &BootInfo, kernel: Region) -> Result<(), &'static str> {
    for region in no_map_regions(info) {
        if region.overlaps(kernel) {
            return Err("no-map overlaps the live kernel image, tables, or stack");
        }
        if region.overlaps(info.dtb) {
            return Err("no-map overlaps the live DTB");
        }
        if info.initrd.is_some_and(|initrd| region.overlaps(initrd)) {
            return Err("no-map overlaps the live initrd");
        }
    }
    Ok(())
}

/// 按地址升序生成单个物理 `ram` 区间扣除全部 `no-map` 保留区并集后的可直映子区间迭代器。
///
/// 算法直接在无序的 `no-map` 列表上推进游标，无需额外分配缓冲区或对保留区数组排序。
pub(crate) fn direct_map_regions(
    info: &BootInfo,
    mut ram: Region,
) -> impl Iterator<Item = Region> + '_ {
    assert!(
        ram.start < ram.end && (ram.start | ram.end) & (PAGE_SIZE - 1) == 0,
        "RAM bank is not page aligned"
    );
    core::iter::from_fn(move || {
        while ram.start < ram.end {
            let start = ram.start;
            let mut end = ram.end;
            let mut covered_end = start;
            // 扫描所有与剩余 RAM 相交的 no-map 区间：若覆盖当前起点则推进 covered_end，否则收紧下一个可用片段的终点 end。
            for hole in no_map_regions(info) {
                if hole.end <= start || ram.end <= hole.start {
                    continue;
                }
                if hole.start <= start {
                    covered_end = covered_end.max(hole.end.min(ram.end));
                } else {
                    end = end.min(hole.start);
                }
            }
            if start < covered_end {
                ram.start = covered_end;
                continue;
            }
            ram.start = end;
            return Some(Region { start, end });
        }
        None
    })
}

/// 校验单段映射的页对齐、2MiB 同余性、Sv39 高半核规范地址格式及虚拟区间不重叠性，并追加到 `mappings` 数组。
fn add_mapping(
    mappings: &mut [BootMapping; MAX_BOOT_MAPPINGS],
    count: &mut usize,
    virtual_start: usize,
    physical: Region,
    flags: usize,
) {
    assert!(
        physical.start < physical.end
            && (physical.start | physical.end | virtual_start) & (PAGE_SIZE - 1) == 0
    );
    assert_eq!(
        (virtual_start ^ physical.start) & (PMD_SIZE - 1),
        0,
        "Sv39 boot mapping offsets differ"
    );
    let virtual_end = virtual_start
        .checked_add(physical.end - physical.start)
        .expect("Sv39 mapping address overflows");
    for address in [virtual_start, virtual_end - 1] {
        assert_eq!(
            address >> (SV39_VA_BITS - 1),
            usize::MAX >> (SV39_VA_BITS - 1),
            "Sv39 kernel mapping is not canonical high-half"
        );
    }

    for old in &mappings[..*count] {
        let old_end = old.virtual_start + (old.physical.end - old.physical.start);
        assert!(
            virtual_end <= old.virtual_start || old_end <= virtual_start,
            "Sv39 mappings overlap"
        );
    }

    assert!(*count < mappings.len(), "Sv39 mapping capacity exceeded");
    mappings[*count] = BootMapping {
        virtual_start,
        physical,
        flags,
    };
    *count += 1;
}

/// 将 [`BootInfo`] 中所有物理 RAM 区间（扣除 `no-map` 空洞后）加入待建表的映射数组。
fn add_ram_mappings(
    info: &BootInfo,
    mappings: &mut [BootMapping; MAX_BOOT_MAPPINGS],
    count: &mut usize,
) {
    for &ram in &info.memory[..info.memory_count] {
        for region in direct_map_regions(info, ram) {
            add_mapping(
                mappings,
                count,
                phys_to_virt(region.start),
                region,
                KERNEL_DATA_FLAGS,
            );
        }
    }
}

/// 将外设物理寄存器范围向外扩展到 4KiB 页边界并加入 MMIO 映射表；若已被既有 MMIO 映射完全覆盖则跳过。
fn add_device_mapping(
    mappings: &mut [BootMapping; MAX_BOOT_MAPPINGS],
    count: &mut usize,
    region: Region,
) {
    if region.start == region.end {
        return;
    }
    let physical = Region {
        start: region.start & !(PAGE_SIZE - 1),
        end: region
            .end
            .checked_add(PAGE_SIZE - 1)
            .expect("MMIO page rounding overflows")
            & !(PAGE_SIZE - 1),
    };
    if mappings[..*count].iter().any(|mapping| {
        mapping.virtual_start == mmio_address(mapping.physical.start)
            && mapping.physical.start <= physical.start
            && physical.end <= mapping.physical.end
    }) {
        return;
    }
    add_mapping(
        mappings,
        count,
        mmio_address(physical.start),
        physical,
        KERNEL_DATA_FLAGS,
    );
}

/// 将外设所需的时钟、复位与引脚控制器（pinctrl）寄存器窗口加入 MMIO 映射表。
fn add_setup_mappings(
    mappings: &mut [BootMapping; MAX_BOOT_MAPPINGS],
    count: &mut usize,
    setup: &crate::boot::DeviceSetup,
) {
    for region in setup.clocks.entries[..setup.clocks.count]
        .iter()
        .chain(setup.clocks.assigned[..setup.clocks.assigned_count].iter())
        .flat_map(|clock| [Some(clock.reference), clock.parent])
        .flatten()
        .map(|reference| reference.provider)
        .chain(
            setup.resets[..setup.reset_count]
                .iter()
                .map(|reference| reference.provider),
        )
        .chain(setup.pins[..setup.pin_count].iter().map(|pin| pin.provider))
        .filter(|region| region.start < region.end)
    {
        add_device_mapping(mappings, count, region);
    }
}

/// 统计给定的映射集合总共需要消耗多少张 Sv39 页表（含 1 张 L2 根表以及所有触及的唯一 L1/L0 子页表）。
///
/// 在真正修改 `FINAL_TABLES` 前预检所需的页表总数，若超出静态池容量则触发 `assert!`。
fn mapping_table_count(mappings: &[BootMapping]) -> usize {
    let mut branches = [usize::MAX; MAX_BOOT_BRANCHES];
    let mut count = 0;
    for mapping in mappings {
        let mut va = mapping.virtual_start;
        let mut pa = mapping.physical.start;
        while pa < mapping.physical.end {
            let huge = huge_leaf(va, pa, mapping.physical.end);
            let keys = [va >> PGDIR_SHIFT, BOOT_BRANCH_L0_TAG | (va >> PMD_SHIFT)];
            for &key in &keys[..if huge { 1 } else { 2 }] {
                if !branches[..count].contains(&key) {
                    assert!(
                        count < branches.len(),
                        "Sv39 boot page table pool exhausted"
                    );
                    branches[count] = key;
                    count += 1;
                }
            }
            // 同一个 2MiB 虚拟槽位内的后续 4KiB 小页共享同一张 L0 页表，游标可直接跨到当前 2MiB 边界或区间终点。
            let step = (PMD_SIZE - (va & (PMD_SIZE - 1))).min(mapping.physical.end - pa);
            va += step;
            pa += step;
        }
    }
    1 + count
}

/// 在 `ktest` 测试的独立 [`BootInfo`] 快照上执行 RAM `no-map` 分裂与页表池容量预检，返回 `(映射片段数, 所需页表总数)`。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_ram_mapping_budget(info: &BootInfo) -> (usize, usize) {
    let empty = BootMapping {
        virtual_start: 0,
        physical: Region { start: 0, end: 0 },
        flags: 0,
    };
    let mut mappings = [empty; MAX_BOOT_MAPPINGS];
    let mut count = 0;
    add_ram_mappings(info, &mut mappings, &mut count);
    (count, mapping_table_count(&mappings[..count]))
}

/// 在唤醒从核（AP）前构建并激活最终的 Sv39 内核页表（涵盖内核镜像、RAM 直映区以及全部板载/总线外设 MMIO 窗口）。
///
/// # Safety
/// 1. 只能由主核（BSP）在本地中断已关闭、当前仍运行在临时启动页表（`RV_BOOT_ROOT`）上且从核尚未启动时调用一次。
/// 2. `info` 必须准确描述当前平台的物理 RAM 与外设布局，且有效 RAM 区间必须完整覆盖当前栈、`CpuLocal`、`FINAL_TABLES` 及内核镜像。
/// 3. 调用期间不得有任何其他执行流并发访问 `FINAL_TABLES` 或 `RV_FINAL_SATP`。
pub unsafe fn finish_boot_mappings(info: &BootInfo) {
    // Step 1: 校验 no-map 保留区不会挖空存活的内核镜像、DTB 或 initrd。
    let kernel = Region {
        start: LINK_LAYOUT.kernel_load_phys as usize,
        end: address!(__kernel_phys_end)
            .checked_add(PAGE_SIZE - 1)
            .expect("kernel page rounding overflows")
            & !(PAGE_SIZE - 1),
    };
    validate_boot_no_map(info, kernel).expect("Sv39 boot no-map conflict");

    // Step 2: 收集内核高地址镜像映射与扣除 no-map 后的物理 RAM 线性直映区间。
    let empty = BootMapping {
        virtual_start: 0,
        physical: Region { start: 0, end: 0 },
        flags: 0,
    };
    let mut mappings = [empty; MAX_BOOT_MAPPINGS];
    let mut count = 0;
    add_mapping(
        &mut mappings,
        &mut count,
        LINK_LAYOUT.kernel_link_virt as usize,
        kernel,
        KERNEL_DATA_FLAGS | PTE_EXEC,
    );
    add_ram_mappings(info, &mut mappings, &mut count);

    // Step 3: 收集固定板级外设（UART、PLIC、RTC、ccache、PMC、PLL）以及 MMC、GMAC、AHCI、VirtIO、PCI 的 MMIO 窗口。
    for region in [
        info.interrupts.uart,
        info.interrupts.controller,
        info.rtc,
        info.board.ccache,
        info.board.pmc,
        info.board.clock_inputs.pll_registers,
    ]
    .iter()
    .copied()
    .filter(|region| region.start < region.end)
    {
        add_device_mapping(&mut mappings, &mut count, region);
    }
    for reference in info.uart.clocks.entries[..info.uart.clocks.count]
        .iter()
        .chain(info.uart.clocks.assigned[..info.uart.clocks.assigned_count].iter())
        .flat_map(|clock| [Some(clock.reference), clock.parent])
        .flatten()
    {
        if reference.provider.start < reference.provider.end {
            add_device_mapping(&mut mappings, &mut count, reference.provider);
        }
    }

    add_setup_mappings(&mut mappings, &mut count, &info.board.rtc_setup);
    if let Some(mmc) = info.board.mmc {
        add_device_mapping(&mut mappings, &mut count, mmc.registers);
        add_device_mapping(&mut mappings, &mut count, mmc.syscon);
        add_setup_mappings(&mut mappings, &mut count, &mmc.setup);
        if let Some(input) = mmc.card_detect {
            add_device_mapping(&mut mappings, &mut count, input.provider);
        }
    }
    if let Some(gmac) = info.board.gmac {
        add_device_mapping(&mut mappings, &mut count, gmac.registers);
        add_device_mapping(&mut mappings, &mut count, gmac.syscon);
        add_setup_mappings(&mut mappings, &mut count, &gmac.setup);
    }
    if let Some(ahci) = info.board.ahci {
        add_device_mapping(&mut mappings, &mut count, ahci.registers);
    }

    for resource in &info.virtio_mmio[..info.virtio_mmio_count] {
        add_device_mapping(&mut mappings, &mut count, resource.registers);
    }
    if info.pci.configuration.start < info.pci.configuration.end {
        add_device_mapping(&mut mappings, &mut count, info.pci.configuration);
    }
    for range in &info.pci.ranges[..info.pci.range_count] {
        if matches!(
            range.flags & PCI_SPACE_CODE_MASK,
            PCI_SPACE_MEM32 | PCI_SPACE_MEM64
        ) {
            add_device_mapping(&mut mappings, &mut count, range.cpu);
        }
    }

    // Step 4: 预算页表池需求，清零并填写 FINAL_TABLES，发布 RV_FINAL_SATP 并切换当前主核的 satp。
    let required = mapping_table_count(&mappings[..count]);
    // SAFETY:
    // 1. 当前主核在关中断且从核未启动的状态下独占 `FINAL_TABLES` 与 `RV_FINAL_SATP`。
    // 2. `add_mapping` 与 `mapping_table_count` 已完成虚拟区间不重叠校验与页表池容量核对（`required <= MAX_BOOT_TABLES`）。
    // 3. 新页表完整包含当前正在执行的内核高地址镜像段与主核栈所在的 RAM 直映区，因此 `csrw satp` 与 `sfence.vma` 后取指与访存平滑衔接。
    unsafe {
        ptr::write_bytes(
            ptr::addr_of_mut!(FINAL_TABLES).cast::<u8>(),
            0,
            size_of::<[PageTable; MAX_BOOT_TABLES]>(),
        );
        let mut next = 1;
        for mapping in &mappings[..count] {
            map_range(
                mapping.virtual_start,
                mapping.physical.start,
                mapping.physical.end,
                mapping.flags,
                &mut next,
            );
        }
        assert_eq!(next, required, "Sv39 page table budget mismatch");
        RV_FINAL_SATP = SATP_MODE_SV39 | (virt_to_phys(table(0) as usize) >> PAGE_SHIFT);
        asm!("fence rw, rw", "csrw satp, {0}", "sfence.vma", in(reg) RV_FINAL_SATP, options(nostack));
    }

    // Step 5: 最终 MMIO 映射生效后，通知控制台子系统绑定串口与复位资源。
    crate::console::configure_resources(info);
}

/// 将当前物理核的 [`CpuLocal`] 基地址写入 `tp`（`x4`）寄存器，供后续普通内核代码与中断处理路径读取。
///
/// # Safety
/// 1. `local` 必须非空、满足 `CpuLocal` 对齐要求，且指向已完成初始化、在内核整个生命周期内永久存活的本核专属 `CpuLocal` 实例。
/// 2. 调用时本核必须处于本地中断关闭状态，且严禁将同一个 `CpuLocal` 实例安装给多个不同的 CPU。
pub unsafe fn install_cpu_local(local: *const CpuLocal) {
    // SAFETY: 调用方契约保证 `local` 独占且永久有效；内核约定 `tp` 寄存器专用于存放本核 `CpuLocal` 指针。
    unsafe {
        asm!("mv tp, {0}", in(reg) local, options(nostack));
    }
}

/// 读取当前物理核 `tp` 寄存器中保存的 [`CpuLocal`] 裸指针。
pub fn cpu_local() -> *const CpuLocal {
    let value;
    // SAFETY: 仅读取 `tp` 寄存器数值而不解引用；若尚未安装（值为 0），上层 `cpu::current` 会拦截空指针。
    unsafe {
        asm!("mv {0}, tp", out(reg) value, options(nomem, nostack));
    }
    value
}

/// 初始化当前 CPU 的 S-mode 陷阱向量与中断使能掩码：安装 [`rv_trap_entry`]、清零 `sscratch`、关闭浮点单元并开启软件/定时器/外部中断源。
pub fn init_current_cpu() {
    // SAFETY:
    // 1. 主从核引导入口保证调用时处于 S-mode，内核镜像与本核栈已建立有效映射。
    // 2. `stvec` 指向遵循 `KernelTrapFrame` 协议的 `rv_trap_entry`，`sscratch` 置 0 表示当前处于 S-mode 内核态。
    // 3. 清除 `sstatus.FS` 确保普通内核代码不使用浮点寄存器；写入 `sie` 仅开放具体中断源，全局中断仍受 `sstatus.SIE` 控制。
    unsafe {
        asm!(
            "csrw stvec, {trap}", "csrw sscratch, zero", "csrci sip, {ssip}",
            "li t0, {fs}", "csrc sstatus, t0", "li t0, {sources}", "csrw sie, t0",
            trap = in(reg) rv_trap_entry as usize, ssip = const SIP_SSIP,
            fs = const SR_FS, sources = const SIE_KERNEL, out("t0") _, options(nostack)
        );
    }
    disable_timer();
}

/// 原子地读取当前 `sstatus` 并清除 `SIE` 位以关闭本核中断，返回关闭前的完整 `sstatus` 值。
///
/// 内联汇编默认带编译器内存屏障（未声明 `nomem`），可防止临界区内的内存读写被编译器重排到关中断指令之前。
pub fn irq_save() -> usize {
    let saved;
    // SAFETY: 内核运行于 S-mode，`csrrci` 原子清除本核 `sstatus.SIE` 并返回原值供配对的 `irq_restore` 使用。
    unsafe {
        asm!("csrrci {saved}, sstatus, {sie}", saved = out(reg) saved, sie = const SR_SIE, options(nostack));
    }
    saved
}

/// 根据配对 [`irq_save`] 保存的状态值恢复本核 `sstatus.SIE` 中断使能位。
///
/// # Safety
/// 1. `saved` 必须来自当前同一物理核上配对的 [`irq_save`] 返回值。
/// 2. 临界区内的受保护内存访问与自旋锁释放必须已完成；嵌套保存必须严格按后进先出（LIFO）逆序恢复。
pub unsafe fn irq_restore(saved: usize) {
    // SAFETY: 调用方契约保证在同一 CPU 上按逆序恢复且临界区已结束；分支仅修改 `sstatus.SIE` 位，不篡改其他状态位。
    unsafe {
        if saved & SR_SIE != 0 {
            asm!("csrsi sstatus, {sie}", sie = const SR_SIE, options(nostack));
        } else {
            asm!("csrci sstatus, {sie}", sie = const SR_SIE, options(nostack));
        }
    }
}

/// 查询当前 CPU 的本地中断是否处于开启状态（即 `sstatus.SIE != 0`）。
pub fn irq_enabled() -> bool {
    let s: usize;
    // SAFETY: S-mode 下读取 `sstatus` CSR 无任何内存或状态副作用。
    unsafe {
        asm!("csrr {0}, sstatus", out(reg) s, options(nomem, nostack));
    }
    s & SR_SIE != 0
}

/// 开启当前 CPU 的本地中断（置位 `sstatus.SIE`）。
///
/// 内联汇编隐含编译器内存屏障，保证此前完成的初始化写入在中断开启前全部落地。
pub fn enable_interrupts() {
    // SAFETY: 调用本函数前，主从核引导流程均已完成 `CpuLocal`、`stvec` 陷阱入口与中断控制器初始化。
    unsafe {
        asm!("csrsi sstatus, {sie}", sie = const SR_SIE, options(nostack));
    }
}

/// 执行 `wfi`（Wait for Interrupt）指令使当前 CPU 进入低功耗等待，直到本地有待处理中断事件唤醒。
pub fn wait_for_interrupt() {
    // SAFETY: S-mode 下执行 `wfi` 仅挂起流水线等待中断事件，本核栈与 `stvec` 陷阱入口已在引导阶段就绪。
    unsafe {
        asm!("wfi", options(nomem, nostack));
    }
}

/// 在本地中断关闭（`sstatus.SIE == 0`）的状态下原子检查唤醒标志并执行一次无丢失唤醒的 `wfi` 休眠，随后短暂开启中断响应挂起事件。
///
/// # Safety
/// 1. 仅允许在当前 CPU 专属的引导栈或调度器空闲栈（Idle Stack）顶层循环中调用：调用时必须满足 `!irq_enabled()`，
///    且未持有任何自旋锁、下半部（Bottom Half）守卫或抢占守卫。
/// 2. `kick_pending`（及 `ktest` 下的 `wait_completions`）必须为当前 CPU 的永久原子字段，`wake_pending` 在本次调用期间必须保持有效。
/// 3. 当前 CPU 的 `stvec` 陷阱入口及 `sie` 中的具体中断源（IPI、Timer、External）必须已使能。
pub(crate) unsafe fn idle_once(
    kick_pending: &AtomicUsize,
    wake_pending: &AtomicBool,
    #[cfg(feature = "ktest")] wait_completions: &AtomicUsize,
) {
    assert!(!irq_enabled(), "idle entry requires IRQ off");
    if kick_pending.load(Ordering::Acquire) == 0 && !wake_pending.load(Ordering::Acquire) {
        // SAFETY:
        // 1. 前置 `assert!` 确认 `sstatus.SIE == 0`，因此在检查 `kick_pending`/`wake_pending` 到执行 `wfi` 之间不会被本核中断抢占。
        // 2. 根据 RISC-V 特权架构规范，当 `sstatus.SIE == 0` 但 `sie` 中对应中断源已使能时，新到达的 IPI/定时器中断会使 `sip`
        //    挂起并直接唤醒或跳过 `wfi`，从而消除“检查标志后、休眠前”的唤醒丢失竞态（Lost Wakeup）。
        unsafe { asm!("wfi", options(nostack)) };
        #[cfg(feature = "ktest")]
        wait_completions.fetch_add(1, Ordering::Release);
    }

    // SAFETY:
    // 1. 调用方契约保证当前未持有任何锁且栈允许响应中断。
    // 2. 置位 `sstatus.SIE` 执行一条 `nop` 后立即清除 `SIE`，让硬件立刻陷入并处理刚才唤醒 `wfi` 的挂起中断，处理返回后恢复关中断状态。
    unsafe {
        asm!(
            "csrsi sstatus, {sie}",
            "nop",
            "csrci sstatus, {sie}",
            sie = const SR_SIE,
            options(nostack)
        )
    };
}

/// 建立硬件时间计数器的基准纪元（Epoch）。
///
/// RISC-V 的 `time` CSR 由平台级统一时钟源（CLINT / ACLINT mtime）驱动，所有物理核天生共享同一时间轴，因此无需校准每核偏移。
pub fn establish_counter_epoch() {}

/// 将时间基准纪元补偿值应用到当前 CPU。
///
/// RISC-V 各核共享统一的 `time` 计数器，无需写入每核偏移寄存器。
///
/// # Safety
/// 仅作为与多架构（如 LoongArch64）统一启动流程的空操作接口，无硬件前置要求。
pub unsafe fn apply_counter_epoch() {}

/// 读取当前硬件时间计数器（`time` CSR，通过 `rdtime` 伪指令获取）。
pub fn read_counter() -> u64 {
    let value;
    // SAFETY: OpenSBI 固件已在启动时向 S-mode 开放 `time` CSR 读取权限（`mcounteren.TM = 1`），`rdtime` 不访问内存也不改变处理器状态。
    unsafe {
        asm!("rdtime {0}", out(reg) value, options(nomem, nostack));
    }
    value
}

/// 返回设备树（DTB）中解析出的硬件时间计数器频率（`timebase-frequency`，单位：Hz）。
#[cfg(feature = "ktest")]
pub fn counter_frequency() -> u64 {
    crate::boot::info().counter_frequency
}

/// 按照 RISC-V SBI v0.2+ 二进制调用规范触发 `ecall` 请求 M-mode 固件服务。
///
/// 寄存器约定：`a7` 为扩展 ID（EID），`a6` 为功能 ID（FID），`a0`–`a2` 为输入参数；
/// 返回时 `a0` 为错误码（`0` 表示 `SBI_SUCCESS`），`a1` 为返回值。
fn sbi(
    extension: usize,
    function: usize,
    mut a0: usize,
    mut a1: usize,
    a2: usize,
) -> Result<usize, isize> {
    // SAFETY:
    // 1. 当前运行在 S-mode，`ecall` 陷入 M-mode OpenSBI 固件，寄存器输入输出严格遵循 SBI v0.2+ ABI。
    // 2. 本模块仅对 `TIME`、`IPI`、`HSM`、`SRST` 四个扩展发起调用；其中仅 `HSM` 传入物理地址，其有效性由 `start_cpu` 调用方契约保证。
    unsafe {
        asm!("ecall", inlateout("a0") a0, inlateout("a1") a1, in("a2") a2, in("a3") 0usize, in("a4") 0usize, in("a5") 0usize, in("a6") function, in("a7") extension, options(nostack));
    }
    if a0 == 0 {
        Ok(a1)
    } else {
        Err(a0 as isize)
    }
}

/// 通过 SBI `TIME` 扩展编程当前 CPU 的下一次定时器中断绝对到期时刻（`time` 计数值）。
pub fn program_timer(deadline: u64) {
    sbi(
        SBI_EXT_TIME,
        SBI_EXT_TIME_SET_TIMER,
        deadline as usize,
        0,
        0,
    )
    .expect("SBI TIME failed");
}

/// 将定时器比较值设为 `u64::MAX`，从而清除当前 CPU 的 `sip.STIP` 挂起位并屏蔽下一次定时器中断。
pub fn disable_timer() {
    program_timer(u64::MAX);
}

/// 通过 SBI `HSM`（Hart State Management）扩展唤醒指定的从核（AP），使其跳转到 [`__secondary_entry_phys`] 并传入 [`CpuBoot`] 的物理地址。
///
/// # Safety
/// 1. 必须由主核串行唤醒当前处于 `STOPPED` 状态的目标物理核（`hardware_id`）。
/// 2. `boot_record_physical` 必须按 `CpuBoot` 对齐、已完成初始化且在内核生命周期内永久有效。
/// 3. `CpuBoot` 内记录的从核栈顶必须按 16 字节对齐，且栈与 `CpuLocal` 由该目标核独占。
/// 4. 临时启动页表（`RV_BOOT_ROOT`）与最终内核页表（`RV_FINAL_SATP`）均已构建完毕并映射了目标核的代码、栈与 `CpuLocal`。
pub unsafe fn start_cpu(hardware_id: usize, boot_record_physical: usize) -> Result<(), isize> {
    // SAFETY: 调用方契约已初始化 `CpuBoot` 记录、从核栈与页表；此处的 `fence rw, rw` 确保这些内存写入在固件唤醒目标核前全局可见。
    unsafe {
        asm!("fence rw, rw", options(nostack));
    }
    sbi(
        SBI_EXT_HSM,
        SBI_EXT_HSM_HART_START,
        hardware_id,
        address!(__secondary_entry_phys),
        boot_record_physical,
    )
    .map(|_| ())
}

/// 在发布核间软件队列数据后，通过 SBI `IPI` 扩展向目标物理核（`hardware_id`）发送核间软件中断。
pub fn send_ipi(hardware_id: usize) {
    // SAFETY: `fence rw, rw` 仅排序内存读写，确保当前 CPU 先前写入软件队列的数据在目标核收到 IPI 中断后可见。
    unsafe {
        asm!("fence rw, rw", options(nostack));
    }
    sbi(SBI_EXT_IPI, SBI_EXT_IPI_SEND_IPI, 1, hardware_id, 0).expect("SBI IPI failed");
}

/// 从已映射的 PLIC（平台级中断控制器）MMIO 窗口读取偏移 `offset` 处的 32 位寄存器。
///
/// # Safety
/// 1. 必须在 `finish_boot_mappings` 完成 PLIC MMIO 映射之后调用。
/// 2. `offset` 必须按 4 字节对齐，且 `[offset, offset + 4)` 完整落在设备树描述的 PLIC 有效寄存器窗口内。
/// 3. 若读取的是 `claim` 寄存器，必须由该 context 对应的所属 CPU 调用，防止跨核争抢同一个 context 的中断声明。
unsafe fn mmio_read(offset: usize) -> u32 {
    let region = crate::boot::info().interrupts.controller;
    // SAFETY: 调用方契约保证 PLIC MMIO 已映射且 `offset` 为窗口内 4 字节对齐的合法 32 位寄存器偏移。
    unsafe { ptr::read_volatile((mmio_address(region.start) + offset) as *const u32) }
}

/// 向已映射的 PLIC MMIO 窗口偏移 `offset` 处写入一个 32 位寄存器值，并执行 `fence iorw, iorw` 同步设备状态。
///
/// # Safety
/// 1. 必须在 `finish_boot_mappings` 完成 PLIC MMIO 映射之后调用。
/// 2. `offset` 必须按 4 字节对齐且完整落在 PLIC 有效寄存器窗口内，`value` 必须符合目标寄存器的硬件语义。
/// 3. 对共享中断源 `priority` 寄存器的写入必须由上层对应中断描述符的自旋锁串行化；对 `enable` 位图的写入仅限主核单核初始化阶段执行。
unsafe fn mmio_write(offset: usize, value: u32) {
    let region = crate::boot::info().interrupts.controller;
    // SAFETY: 调用方契约保证 PLIC 窗口已映射、`offset` 4 字节对齐且并发访问已受锁或初始化阶段独占保护；写后 `fence` 保证 MMIO 立即生效。
    unsafe {
        ptr::write_volatile((mmio_address(region.start) + offset) as *mut u32, value);
        asm!("fence iorw, iorw", options(nostack));
    }
}

/// 计算当前 CPU 对应的 PLIC S-mode `claim/complete` 寄存器在 PLIC 窗口内的字节偏移。
fn claim_offset() -> usize {
    PLIC_CONTEXT_BASE
        + crate::boot::info().interrupts.plic_contexts[crate::cpu::current().id]
            * PLIC_CONTEXT_STRIDE
        + PLIC_CLAIM_OFFSET
}

/// 初始化 PLIC 外部中断控制器：清零全部 CPU 的使能位与阈值，将所有中断源初始优先级置 0（屏蔽），并仅在主核（CPU 0）S-mode context 上使能合法中断源路由。
///
/// # Safety
/// 1. `info` 必须包含已验证的 PLIC 控制器范围与各核 S-mode context 编号，且 PLIC MMIO 窗口已完成映射。
/// 2. `info.interrupts.source_count` 必须落在 `2..=PLIC_MAX_SOURCES`（1024）范围内，所有 context 的 `enable`/`threshold`/`claim` 寄存器均落在映射窗口内。
/// 3. 只能由主核在本地中断关闭且从核尚未启动时调用一次，调用期间不得与任何 `external_claim`/`external_complete`/`external_mask` 并发。
pub unsafe fn external_init(info: &BootInfo) {
    assert!(1 < info.interrupts.source_count && info.interrupts.source_count <= PLIC_MAX_SOURCES);
    let words = (info.interrupts.source_count as usize).div_ceil(PLIC_ENABLE_BITS);
    // SAFETY:
    // 1. 前置 `assert!` 确认中断源数量不超过 `PLIC_MAX_SOURCES`，且调用方契约保证主核此时独占整个 PLIC 寄存器窗口。
    // 2. 先清空所有 CPU S-mode context 的 `enable` 位图并将 `threshold` 设为 0，再将 `1..source_count` 的 `priority` 置 0，
    //    最后仅在 `plic_contexts[0]`（主核）开启 `1..source_count` 的使能位（跳过恒为 0 的保留源 0）。
    unsafe {
        // Step 1: 清零所有在线 CPU S-mode context 的中断使能位图，并将优先级阈值（threshold）设为 0。
        for &context in &info.interrupts.plic_contexts[..info.cpu_count] {
            for word in 0..PLIC_ENABLE_WORDS {
                mmio_write(
                    PLIC_ENABLE_BASE + context * PLIC_ENABLE_STRIDE + word * PLIC_REGISTER_BYTES,
                    0,
                );
            }
            mmio_write(PLIC_CONTEXT_BASE + context * PLIC_CONTEXT_STRIDE, 0);
        }

        // Step 2: 将所有合法外部中断源（1..source_count）的初始优先级（priority）设为 0（默认处于屏蔽状态）。
        for source in 1..info.interrupts.source_count {
            mmio_write(source as usize * PLIC_REGISTER_BYTES, 0);
        }

        // Step 3: 将所有合法非零中断源固定使能到主核（CPU 0）的 S-mode context；后续掩码开关仅调整各源的 priority。
        for word in 0..words {
            let remaining = info.interrupts.source_count as usize - word * PLIC_ENABLE_BITS;
            let mut bits = if PLIC_ENABLE_BITS <= remaining {
                u32::MAX
            } else {
                (1u32 << remaining) - 1
            };
            if word == 0 {
                bits &= !1;
            }
            mmio_write(
                PLIC_ENABLE_BASE
                    + info.interrupts.plic_contexts[0] * PLIC_ENABLE_STRIDE
                    + word * PLIC_REGISTER_BYTES,
                bits,
            );
        }
    }
}

/// 屏蔽指定的 PLIC 外部中断源 `source`：将其硬件优先级（`priority`）置为 `0`。
///
/// 只修改该中断源自身的 `priority` 寄存器而不撤销主核 context 的 `enable` 位，从而保证此前已被 `claim` 的同一中断源仍能正常执行 `complete`。
pub fn external_mask(source: u32) {
    assert!(0 < source && source < crate::boot::info().interrupts.source_count);
    // SAFETY: 前置 `assert!` 保证 `source` 落在有效范围内，每个中断源拥有独立的 4 字节 `priority` 寄存器并由上层 IRQ 描述符锁串行化。
    unsafe { mmio_write(source as usize * PLIC_REGISTER_BYTES, 0) };
}

/// 解除对指定 PLIC 外部中断源 `source` 的屏蔽：将其硬件优先级（`priority`）恢复为 `1`。
///
/// 由于主核 S-mode context 的 `threshold` 为 `0` 且 `enable` 位常开，优先级设为 `1` 后任何待处理中断都会立即向主核发起通知。
pub fn external_unmask(source: u32) {
    assert!(0 < source && source < crate::boot::info().interrupts.source_count);
    // SAFETY: 前置 `assert!` 保证 `source` 合法，独立的 `priority` 寄存器访问由上层 IRQ 描述符锁串行化保护。
    unsafe { mmio_write(source as usize * PLIC_REGISTER_BYTES, 1) };
}

/// 从当前 CPU 的 PLIC S-mode context 读取 `claim` 寄存器以获取最高优先级的待处理外部中断源号；若返回 `0`（无待处理中断）则返回 `None`。
pub fn external_claim() -> Option<u32> {
    // SAFETY: PLIC 已在引导阶段完成映射与初始化，`claim_offset()` 根据本核 `CpuLocal.id` 唯一定位当前 CPU 的 S-mode `claim` 寄存器。
    let source = unsafe { mmio_read(claim_offset()) };
    if source == 0 {
        None
    } else {
        Some(source)
    }
}

/// 向当前 CPU 的 PLIC S-mode `complete` 寄存器写回中断源号 `source`，通知硬件该外部中断已处理完毕。
///
/// # Safety
/// 1. `source` 必须来自当前同一 CPU 上尚未完成（Complete）的一次真实 [`external_claim`] 返回值。
/// 2. 调用方必须持有对应中断描述符的锁，严禁跨核完成或对同一次 `claim` 重复调用 `external_complete`。
pub unsafe fn external_complete(source: u32) {
    assert!(0 < source && source < crate::boot::info().interrupts.source_count);
    // SAFETY: 前置 `assert!` 验证了 `source` 范围，调用方契约保证 `source` 来自本核尚未完成的 `claim`，且主核固定 `enable` 路由始终保持开启。
    unsafe { mmio_write(claim_offset(), source) };
}

/// 正常关机流程：通知并等待所有其他在线 CPU 停止后，排空控制台并调用 SBI `SRST` 执行整机断电（Shutdown）。
///
/// 仅限在可响应中断/IPI、未持有业务锁且无挂起跨核调用（CSD）责任的普通任务上下文中调用。
#[cfg(not(feature = "ktest"))]
pub fn shutdown() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    if crate::platform::current() == crate::boot::Platform::VisionFive2 {
        disable_stop_sources();
    }
    crate::console::board_stop();
    system_reset(SBI_SRST_SHUTDOWN)
}

/// 整机挂起（Halt）流程：在 QEMU `virt` 平台上直接执行关机断电；在真实开发板上等待其他在线核停止并关闭本地中断源后进入无限 `wfi` 休眠。
#[cfg(not(feature = "ktest"))]
pub fn halt() -> ! {
    if crate::platform::current() == crate::boot::Platform::QemuVirt {
        shutdown();
    }
    let _stop = crate::cpu::stop_other_online_cpus();
    disable_stop_sources();
    crate::console::board_stop();
    loop {
        wait_for_interrupt();
    }
}

/// 整机冷重启（Cold Reboot）流程：等待所有其他在线 CPU 停止并完成控制台收尾后，调用 SBI `SRST` 请求硬件冷重启。
#[cfg(not(feature = "ktest"))]
pub fn restart() -> ! {
    let _stop = crate::cpu::stop_other_online_cpus();
    if crate::platform::current() == crate::boot::Platform::VisionFive2 {
        disable_stop_sources();
    }
    crate::console::board_stop();
    system_reset(SBI_SRST_COLD_REBOOT)
}

/// 致命错误下的紧急停机：在 VisionFive 2 开发板上关闭当前核中断源并输出当前核终止标记后进入死循环休眠（避免在从核故障时跨核等待死锁）；在 QEMU 上直接请求 SBI 关机。
#[cfg(feature = "ktest")]
pub fn emergency_shutdown() -> ! {
    if crate::platform::current() == crate::boot::Platform::VisionFive2 {
        disable_stop_sources();
        crate::console::board_stop_current_cpu();
        loop {
            wait_for_interrupt();
        }
    }
    system_reset(SBI_SRST_SHUTDOWN)
}

/// `ktest` 测试套件执行完毕后的退出入口：直接调用 SBI `SRST` 关闭 QEMU 虚拟机。
#[cfg(feature = "ktest")]
pub fn test_shutdown() -> ! {
    system_reset(SBI_SRST_SHUTDOWN)
}

/// 响应跨核停机请求：关闭当前 CPU 的全部中断与定时器事件，以 `Release` 内存序将 `stopped` 置为 `true` 后永久进入 `wfi` 循环。
pub(crate) fn stop_current_cpu(stopped: &AtomicBool) -> ! {
    disable_stop_sources();
    stopped.store(true, Ordering::Release);
    loop {
        wait_for_interrupt();
    }
}

/// 在 CPU 永久停机前关闭本地全局中断、定时器及 `sie` 中断使能掩码，并执行全序 I/O 屏障。
fn disable_stop_sources() {
    irq_save();
    disable_timer();
    // SAFETY: 当前运行在 S-mode 且调用方即将永久停止本核执行，清零 `sie` 可阻止任何中断源再次唤醒后的中断陷入。
    unsafe { asm!("csrw sie, zero", options(nostack)) };
    dma_barrier();
}

/// 调用 SBI `SRST`（System Reset）扩展执行指定类型（`reset_type`）的整机复位或关机；若固件意外返回则触发 `panic!`。
fn system_reset(reset_type: usize) -> ! {
    sbi(
        SBI_EXT_SRST,
        SBI_EXT_SRST_RESET,
        reset_type,
        SBI_SRST_REASON_NONE,
        0,
    )
    .expect("SBI SRST failed");
    panic!("SBI SRST unexpectedly returned");
}

#[cfg(feature = "ktest")]
/// 执行全零非法指令（`.word 0`）以主动触发 S-mode 非法指令同步异常，供 `ktest` 异常诊断路径测试使用。
pub fn trigger_fault() -> ! {
    // SAFETY: 调用前内核 `stvec` 陷阱入口已安装；`.word 0` 在 RISC-V 中恒为非法指令，陷入 `rv_kernel_fault` 后终止执行且永不返回。
    unsafe {
        asm!(".word 0", options(noreturn));
    }
}

/// S-mode 内核态陷阱的 Rust 总入口（由 `trap::rv_trap_entry` 汇编直接调用）。
///
/// 根据 `scause` 最高位区分同步异常（进入 [`rv_kernel_fault`]）与异步硬件中断（进入 [`rv_dispatch_interrupt`]）。
#[no_mangle]
extern "C" fn rv_handle_trap(frame: &mut KernelTrapFrame) {
    if frame.scause & CAUSE_IRQ_FLAG == 0 {
        rv_kernel_fault(frame.sepc, frame.gpr[2], frame.scause, frame.stval);
    }
    rv_dispatch_interrupt(frame.scause, frame.sepc, frame.gpr[2], frame.stval);
}

/// 处理 S-mode 内核态同步异常：输出故障寄存器现场（`pc`、`sp`、`cause`、`stval`）并终止系统运行。
extern "C" fn rv_kernel_fault(pc: usize, sp: usize, cause: usize, bad: usize) -> ! {
    #[cfg(feature = "ktest")]
    crate::ktest::fatal_event(
        if cause == EXC_INST_ILLEGAL {
            "illegal"
        } else {
            "panic"
        },
        format_args!(
            "synchronous trap pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
            pc, sp, cause, bad
        ),
    );
    #[cfg(not(feature = "ktest"))]
    crate::console::fatal(pc, sp, cause, bad);
}

/// 在本地中断关闭的内核陷阱栈上分派 S-mode 异步中断（定时器中断、核间软件中断、PLIC 外部中断）。
///
/// 进入时通过 [`crate::cpu::HardIrqGuard::enter`] 标记硬中断上下文，并在函数返回前自动退出并触发必要的下半部处理。
fn rv_dispatch_interrupt(cause: usize, pc: usize, sp: usize, bad: usize) {
    let _hard_irq = crate::cpu::HardIrqGuard::enter();
    match cause {
        value if value == CAUSE_IRQ_FLAG | IRQ_S_TIMER => {
            disable_timer();
            crate::cpu::handle_timer();
        }
        value if value == CAUSE_IRQ_FLAG | IRQ_S_SOFT => {
            // SAFETY: 当前处于 S-mode 软件中断处理路径，清除 `sip.SSIP` 挂起位后执行 `fence rw, rw` 以配对发送方 `send_ipi` 的发布屏障。
            unsafe {
                asm!("csrci sip, {ssip}", "fence rw, rw", ssip = const SIP_SSIP, options(nostack));
            }
            crate::cpu::handle_ipi();
        }
        value if value == CAUSE_IRQ_FLAG | IRQ_S_EXT => crate::irq::handle_external(),
        _ => {
            #[cfg(feature = "ktest")]
            crate::ktest::fatal_event(
                "panic",
                format_args!(
                    "unknown interrupt pc={:#x} sp={:#x} cause={:#x} bad={:#x}",
                    pc, sp, cause, bad
                ),
            );
            #[cfg(not(feature = "ktest"))]
            crate::console::fatal(pc, sp, cause, bad)
        }
    }
}

/// 在 `ktest` 中编程一次真实的本核定时器中断，验证陷阱入口对整数及可选双精度浮点（`fp = true`）寄存器特征值的保存与恢复。
///
/// `_fcc` 参数仅为兼容 LoongArch64 的浮点条件码接口而保留，RISC-V 无此寄存器故忽略。
///
/// # Safety
/// 1. 当前 CPU 的 `stvec` 陷阱入口、定时器中断源、`CpuLocal` 与有效内核栈必须已就绪，且调用期间禁止任务迁移或持有中断处理路径会获取的锁。
/// 2. CPU 必须支持双精度浮点扩展（`D`），且调用时无活跃的浮点上下文（当 `fp == true` 时汇编会改写 `f0`–`f31`）。
/// 3. 调用方必须在同一 CPU 的关中断状态下调用 `time::reprogram_current()` 恢复生产环境的定时器到期时间。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn ktest_register_pattern(fp: bool, _fcc: u8) -> bool {
    let _irq = crate::sync::IrqSaveGuard::new();
    let counter = &crate::cpu::current().timer_interrupts;
    let expected = counter.load(Ordering::Acquire) + 1;
    program_timer(read_counter() + counter_frequency() / 100);
    // SAFETY:
    // 1. 调用方契约保证当前固定在本核执行、陷阱与定时器已就绪且未持有冲突锁。
    // 2. `counter` 指向本核 `CpuLocal` 中的永久 `AtomicUsize`，`usize::from(fp)` 仅传入 `0` 或 `1`，汇编在返回前会恢复进入时的关中断状态与 ABI 寄存器。
    let good = unsafe { rv_register_roundtrip(counter, expected, usize::from(fp)) } != 0;
    good && expected <= counter.load(Ordering::Acquire)
}

/// 在普通内核任务中连续触发两次真实的定时器中断，验证 `sstatus.FS = Off` 下全部通用整数寄存器、`sp`、`tp` 与陷阱返回路径的完整性。
///
/// # Safety
/// 1. 当前 CPU 的陷阱入口、定时器与 `CpuLocal` 必须已安装，调用方不得持有自旋锁或抢占守卫，且当前无活跃浮点现场。
/// 2. 测试过程中会临时开启本核中断并重编程本核定时器，在此期间当前任务不得被迁移到其他 CPU。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn test_integer_register_preservation() -> usize {
    let saved = irq_save();
    let counter = &crate::cpu::current().timer_interrupts;
    let mut rounds = 0;
    for _ in 0..2 {
        let expected = counter.load(Ordering::Acquire) + 1;
        program_timer(read_counter() + counter_frequency() / 1000);
        // SAFETY:
        // 1. 调用方契约保证陷阱与定时器已就绪、未持锁且不发生跨核迁移；`counter` 为本核 `CpuLocal` 的永久原子计数器。
        // 2. 第三个参数传 `0` 表示仅测试整数寄存器现场，汇编内部短暂开启中断等待定时器触发后恢复进入时的关中断状态。
        assert!(
            unsafe { rv_register_roundtrip(counter, expected, 0) } != 0,
            "integer timer trap register preservation failed"
        );
        assert!(
            expected <= counter.load(Ordering::Acquire),
            "integer timer trap was not observed"
        );
        rounds += 1;
    }
    crate::time::reprogram_current();
    // SAFETY: `saved` 来自本函数开头在同一 CPU 上保存的 `irq_save()`，此时测试汇编已返回且未持有任何锁。
    unsafe { irq_restore(saved) };
    rounds
}
