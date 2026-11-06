//! 龙芯 2K1000LA（LoongArch64 双核 SoC）开发板的固件交接解析与设备树资源发现模块。
//!
//! # 模块职责与架构定位
//! 本模块位于平台抽象层（`crate::platform`）下，承担两项核心任务：
//! 1. **固件引导参数交接（[`boot_parameters`]）**：在 LoongArch64 引导最早期，从固件传入的 `a0..a3`
//!    寄存器中解析出扁平设备树（DTB）与内核命令行的物理地址，兼容 UHI（Unified Hosting Interface）、
//!    显式 `a3` DTB 传参以及 U-Boot `bootm` 的 EFI64 系统表（`EFI_SYSTEM_TABLE`）交接协议。
//! 2. **板级设备树资源发现（[`discover`]）**：在识别出当前平台为 `Platform::Ls2k1000la` 后，解析并校验
//!    双 Bank 传统 I/O 中断控制器（`loongson,liointc-2.0`）、RTC、电源管理控制器（PMC）以及片上外设
//!    （支持标准 Loongson PCI 拓扑或无 PCI Host 的纯 MMIO 设备树绑定），写入 [`BootInfo`]。
//!
//! # 核心设计与硬件拓扑
//! ## 1. 固件交接与 EFI64 系统表布局
//! LoongArch 固件可能传入物理地址，或带直接映射窗口（DMW，`0x8000_...` / `0x9000_...`）前缀的虚拟地址。
//! 当通过 EFI64 系统表传递 DTB 时，内存结构如下：
//!
//! ```text
//!   a0..a3 固件参数:
//!   +-----------------------------------------------------------------------+
//!   | 分支 A (UHI):       a0 == usize::MAX - 1, a1 = DTB 地址               |
//!   | 分支 B (显式 a3):   a0 <= 1, a1 = cmdline, a3 = DTB 地址 (非 0)       |
//!   | 分支 C (EFI64):     a0 <= 1, a1 = cmdline, a2 = EFI_SYSTEM_TABLE 地址 |
//!   +-----------------------------------------------------------------------+
//!                                                     |
//!                                                     v
//!   EFI_SYSTEM_TABLE (最小 0x78 字节):
//!   +------------------+----------------------------------------------------+
//!   | 0x00..0x08       | Signature = 0x5453_5953_2049_4249 ("IBI SYST")     |
//!   | 0x0c..0x10       | HeaderSize (>= 0x78)                               |
//!   | 0x68..0x70       | NumberOfTableEntries (nr_tables)                   |
//!   | 0x70..0x78       | ConfigurationTable 数组指针                        |
//!   +------------------+----------------------------------------------------+
//!                                                     |
//!                                                     v
//!   EFI_CONFIGURATION_TABLE 数组 (每项 24 字节):
//!   +------------------+----------------------------------------------------+
//!   | 0x00..0x10 (16B) | VendorGuid (匹配唯一 EFI_FDT_GUID)                 |
//!   | 0x10..0x18 (8B)  | VendorTable -> DTB 物理/DMW 地址                   |
//!   +------------------+----------------------------------------------------+
//! ```
//!
//! ## 2. 双 Bank LIOINTC 中断路由与片上 PCI/MMIO 拓扑
//! 2K1000LA 包含两个 `liointc-2.0` 控制器（Bank 0 与 Bank 1），每个 Bank 管理 32 个高电平触发中断源，
//! 分别汇总到 CPU 中断控制器的父中断线 IP2（cause = 2）与 IP3（cause = 3），拼接为 0..64 的统一外部中断域：
//!
//! ```text
//!                  +---------------------------------------------+
//!                  |   loongson,cpu-interrupt-controller (CPU)   |
//!                  +---------------------------------------------+
//!                             ^                       ^
//!                   cause = 2 |                       | cause = 3
//!              +--------------+-------+       +-------+--------------+
//!              | LIOINTC0 (Bank 0)    |       | LIOINTC1 (Bank 1)    |
//!              | main(0x40), isr0/1   |       | main(0x40), isr0/1   |
//!              | parent_map=[~0,0,0,0]|       | parent_map=[0,~0,0,0]|
//!              | 外部 IRQ 编号: 0..31 |       | 外部 IRQ 编号: 32..63|
//!              +----------------------+       +----------------------+
//!                         ^                               ^
//!                         +---------------+---------------+
//!                                         |
//!            +----------------------------+----------------------------+
//!            | 模式 1: 片上 PCI 拓扑 (loongson,ls2k-pci)               | 模式 2: 原生 MMIO 拓扑
//!            | - CFG0 (类型 0 配置窗口) + CFG1 (类型 1 配置窗口)       | (无 PCI Host 时回退)
//!            | - 子节点 sata@8,0 (BDF 00:08.0) -> AHCI                 | - AHCI MMIO 节点
//!            | - 子节点 ethernet@3,0 (BDF 00:03.0) -> GMAC0            | - ethernet0 DWMAC 节点
//!            +---------------------------------------------------------+
//!
//!   PCI 子节点 5-cell reg 格式: [ <phys.hi (BDF 位于 bits[23:8])>, 0, 0, 0, 0 ]
//! ```
//!
//! # 锁获取顺序与并发契约
//! 本模块仅在引导早期单核（BSP）串行初始化阶段运行，不获取任何运行期锁。固件交接阶段直接借用只读的
//! 固件内存切片，解析完成后仅保留归一化后的物理地址，不跨越初始化阶段保留裸切片引用。

#[cfg(target_arch = "loongarch64")]
use core::convert::{TryFrom, TryInto};

use super::resources;
use super::{
    cells, compatible, device_region, enabled, identity_address, interrupt_parent, number, parent,
    property, regions, require_controller, single_region, word, Fdt, Node, FDT_CELL_BYTES,
    IRQ_TYPE_LEVEL_HIGH, PCI_RANGE_BUS_OFFSET, PCI_RANGE_BYTES, PCI_RANGE_CPU_OFFSET,
    PCI_RANGE_SIZE_OFFSET,
};
use crate::boot::{
    AhciResources, BootInfo, LioIntcResources, PciConfiguration, PciFunctionResource, PciIntxRoute,
    PciRange, Region, SysconAction, PCI_BUS_SHIFT, PCI_DEVICE_COUNT, PCI_DEVICE_SHIFT,
    PCI_FUNCTION_COUNT, PCI_FUNCTION_SHIFT, PCI_SPACE_CODE_MASK, PCI_SPACE_IO, PCI_SPACE_MEM32,
    PCI_SPACE_MEM64,
};
#[cfg(target_arch = "loongarch64")]
use crate::platform::layout::{
    DMW_ADDRESS_END, DMW_ADDRESS_MASK, DMW_CACHED_VSEG, DMW_UNCACHED_VSEG, DMW_VSEG_SHIFT,
};

/// 固件交接阶段解析并归一化后的引导参数物理地址。
///
/// 解析完成后仅保存纯物理地址数值（已剥离 DMW 虚拟段前缀），不再借用 EFI 系统表或配置表内存。
#[cfg(target_arch = "loongarch64")]
pub(super) struct FirmwareParameters {
    /// 扁平设备树（DTB）在物理内存中的起始地址（字节，非零）。
    pub(super) dtb: usize,
    /// 可选的内核命令行以 NUL 结尾字符串的物理起始地址（字节，非零）。
    pub(super) command_line: Option<usize>,
}

#[cfg(target_arch = "loongarch64")]
const EFI_SYSTEM_TABLE_SIGNATURE: u64 = 0x5453_5953_2049_4249;
#[cfg(target_arch = "loongarch64")]
const EFI_SYSTEM_TABLE_SIZE: usize = 0x78;
#[cfg(target_arch = "loongarch64")]
const EFI_HEADER_SIZE_OFFSET: usize = 0x0c;
#[cfg(target_arch = "loongarch64")]
const EFI_NR_TABLES_OFFSET: usize = 0x68;
#[cfg(target_arch = "loongarch64")]
const EFI_TABLES_OFFSET: usize = 0x70;
#[cfg(target_arch = "loongarch64")]
const EFI_CONFIG_TABLE_SIZE: usize = 24;
#[cfg(target_arch = "loongarch64")]
const EFI_CONFIG_TABLE_POINTER_OFFSET: usize = 16;
#[cfg(target_arch = "loongarch64")]
const EFI_FDT_GUID: [u8; 16] = [
    0xd5, 0x21, 0xb6, 0xb1, 0x9c, 0xf1, 0xa5, 0x41, 0x83, 0x0b, 0xd9, 0x15, 0x2c, 0x69, 0xaa, 0xe0,
];

#[cfg(target_arch = "loongarch64")]
const UHI_FDT_HANDOFF: usize = usize::MAX - 1;

const LIOINTC_FIRST_PARENT_CAUSE: u32 = 2;
const LIOINTC_LAST_PARENT_CAUSE: u32 = 3;
const LIOINTC_REGISTER_BYTES: usize = 0x40;
const LIOINTC_ISR_BYTES: usize = 2 * core::mem::size_of::<u32>();

const PCI_BDF_MASK: u32 = ((u8::MAX as u32) << PCI_BUS_SHIFT)
    | (((PCI_DEVICE_COUNT - 1) as u32) << PCI_DEVICE_SHIFT)
    | (((PCI_FUNCTION_COUNT - 1) as u32) << PCI_FUNCTION_SHIFT);

/// 解析 LoongArch64 固件传入的四个引导寄存器（`a0..a3`），提取 DTB 与内核命令行的物理地址。
///
/// 调用上下文：引导最早期单核（BSP）环境，在正式解析 FDT 前由 `platform` 初始化入口调用。
/// 支持三种固件交接协议：
/// 1. UHI 协议：`a0 == usize::MAX - 1`，`a1` 为 DTB 地址，无独立命令行指针。
/// 2. 显式 `a3` 传参：`a0 <= 1` 且 `a3 != 0`，`a3` 为 DTB 地址，`a1`（若非零）为命令行地址。
/// 3. EFI64 系统表传参：`a0 <= 1`、`a3 == 0` 且 `a2 != 0`，从 `a2` 指向的 `EFI_SYSTEM_TABLE` 中查找 FDT。
///
/// # Safety
/// 1. 当走 EFI64 分支（`args[0] <= 1 && args[3] == 0 && args[2] != 0`）时，调用方与固件必须保证 `args[2]`
///    指向的 `EFI_SYSTEM_TABLE` 头部及其声明的 `ConfigurationTable` 数组在本次函数执行期间位于已映射的
///    有效 RAM 中、可读且不会发生并发修改。
/// 2. 返回的 `dtb` 与 `command_line` 物理地址对应的内存区域必须由调用方继续保持存活且不被覆盖，
///    直到后续 FDT 解析与命令行拷贝全部完成。
#[cfg(target_arch = "loongarch64")]
pub(super) unsafe fn boot_parameters(args: [usize; 4]) -> Result<FirmwareParameters, &'static str> {
    // Step 1: 优先识别 UHI 标准交接（a0 = -2，a1 = DTB 地址）。
    if args[0] == UHI_FDT_HANDOFF {
        return Ok(FirmwareParameters {
            dtb: physical(args[1])?,
            command_line: None,
        });
    }

    // Step 2: 在传统/EFI 引导协议（a0 <= 1）下提取可选的 a1 内核命令行地址。
    let command_line = if args[0] <= 1 && args[1] != 0 {
        Some(physical(args[1])?)
    } else {
        None
    };

    // Step 3: 优先使用 a3 显式传入的 DTB 地址；若 a3 为 0，则从 a2 的 EFI64 系统表中检索 FDT GUID。
    let dtb = if args[3] != 0 {
        physical(args[3])?
    } else if args[0] <= 1 && args[2] != 0 {
        // SAFETY: 前置条件已确认当前处于 EFI64 交接分支（args[0] <= 1 且 args[2] != 0）；
        // 根据本函数的 # Safety 契约，固件与调用方保证 args[2] 指向的 EFI 系统表及配置表在解析期间可读且无并发修改。
        unsafe { efi_dtb(args[2])? }
    } else {
        return Err("LoongArch firmware did not supply a supported OS DTB handoff");
    };
    Ok(FirmwareParameters { dtb, command_line })
}

/// 从 EFI64 系统表（`EFI_SYSTEM_TABLE`）的配置表数组中查找唯一匹配的 `EFI_FDT_GUID`，返回 DTB 物理地址。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`boot_parameters`] 调用。
///
/// # Safety
/// 1. 调用方必须保证 `address` 指向的 `EFI_SYSTEM_TABLE` 结构体（含完整 `HeaderSize` 字节）及
///    `ConfigurationTable` 指向的 `NumberOfTableEntries` 个配置表项位于已建立 DMW 映射的物理内存中。
/// 2. 上述固件内存区域在本次函数执行期间必须可读、对齐合法且不存在任何并发写入。
#[cfg(target_arch = "loongarch64")]
unsafe fn efi_dtb(address: usize) -> Result<usize, &'static str> {
    // Step 1: 借用固定最小长度（0x78 字节）的系统表头部，校验签名与声明的 HeaderSize。
    // SAFETY: 调用方契约保证 address 处至少 EFI_SYSTEM_TABLE_SIZE 字节的系统表头部已映射、可读且无并发修改。
    let header = unsafe { firmware_bytes(address, EFI_SYSTEM_TABLE_SIZE)? };
    if u64::from_le_bytes(header[..8].try_into().unwrap()) != EFI_SYSTEM_TABLE_SIGNATURE {
        return Err("EFI system table signature is invalid");
    }
    let header_size = u32::from_le_bytes(
        header[EFI_HEADER_SIZE_OFFSET..EFI_HEADER_SIZE_OFFSET + 4]
            .try_into()
            .unwrap(),
    ) as usize;
    if header_size < EFI_SYSTEM_TABLE_SIZE {
        return Err("EFI system table header is too small");
    }
    // SAFETY: 前置检查已确认 header_size >= EFI_SYSTEM_TABLE_SIZE，调用方契约保证系统表声明的完整头部范围可读且不变；
    // 此处调用 firmware_bytes 校验完整 header_size 在物理与 DMW 虚拟地址空间内不发生算术溢出或越界。
    unsafe { firmware_bytes(address, header_size)? };

    // Step 2: 读取配置表项数量与配置表数组基地址，检查总字节数不发生乘法溢出后借用整个配置表数组。
    let count = usize::try_from(u64::from_le_bytes(
        header[EFI_NR_TABLES_OFFSET..EFI_NR_TABLES_OFFSET + 8]
            .try_into()
            .unwrap(),
    ))
    .map_err(|_| "EFI configuration table count is not representable")?;
    let tables_address = usize::try_from(u64::from_le_bytes(
        header[EFI_TABLES_OFFSET..EFI_TABLES_OFFSET + 8]
            .try_into()
            .unwrap(),
    ))
    .map_err(|_| "EFI configuration table address is not representable")?;
    let size = count
        .checked_mul(EFI_CONFIG_TABLE_SIZE)
        .ok_or("EFI configuration table array size overflows")?;
    // SAFETY: checked_mul 已排除长度溢出，调用方契约保证 tables_address 指向的 count 个配置表项可读且无并发修改。
    let tables = unsafe { firmware_bytes(tables_address, size)? };

    // Step 3: 遍历所有 24 字节配置表项，匹配唯一的 EFI_FDT_GUID 并将其表指针归一化为物理地址。
    let mut dtb = None;
    for entry in tables.chunks_exact(EFI_CONFIG_TABLE_SIZE) {
        if entry[..EFI_CONFIG_TABLE_POINTER_OFFSET] != EFI_FDT_GUID {
            continue;
        }
        if dtb.is_some() {
            return Err("EFI FDT configuration table GUID is duplicated");
        }
        let pointer = usize::try_from(u64::from_le_bytes(
            entry[EFI_CONFIG_TABLE_POINTER_OFFSET..].try_into().unwrap(),
        ))
        .map_err(|_| "EFI FDT address is not representable")?;
        dtb = Some(physical(pointer)?);
    }
    dtb.ok_or("EFI FDT configuration table GUID is missing")
}

/// 校验固件地址与长度在物理及 DMW 虚拟空间内的可表示性，并构造只读字节切片。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`efi_dtb`] 调用。
///
/// # Safety
/// 1. 调用方必须保证归一化后的物理区间 `[physical(address), physical(address) + size)` 位于真实 RAM 中，
///    且通过内核 DMW 映射（`crate::arch::phys_to_virt`）可安全读取。
/// 2. 在返回的切片生命周期 `'a` 内，该内存区域内容必须已初始化且不存在任何并发写入。
#[cfg(target_arch = "loongarch64")]
unsafe fn firmware_bytes<'a>(address: usize, size: usize) -> Result<&'a [u8], &'static str> {
    let address = physical(address)?;
    let end = address
        .checked_add(size)
        .ok_or("firmware physical storage range overflows")?;
    if (DMW_ADDRESS_END as usize) < end || (isize::MAX as usize) < size {
        return Err("firmware storage range is not representable");
    }
    let pointer = crate::arch::phys_to_virt(address) as *const u8;
    if (pointer as usize).checked_add(size).is_none() {
        return Err("firmware virtual storage range overflows");
    }
    // SAFETY:
    // 1. `physical(address)?` 已保证物理地址非零，转换后的 DMW 虚拟地址 `pointer` 非空且滿足 `u8` 的 1 字节对齐；
    // 2. 前置判断已确认 `size <= isize::MAX` 且物理/虚拟起止地址加法均不溢出、不超过 `DMW_ADDRESS_END`；
    // 3. 调用方契约保证该段固件内存已在 DMW 映射下可读且在生命周期 `'a` 内无并发修改。
    Ok(unsafe { core::slice::from_raw_parts(pointer, size) })
}

/// 将固件传入的物理地址或 Cached/Uncached DMW 窗口地址归一化为非零物理地址。
///
/// 调用上下文：引导早期单核（BSP）环境。拒绝不在 `0`、`DMW_UNCACHED_VSEG`、`DMW_CACHED_VSEG`
/// 三个窗口内的非法段前缀，以及掩码后为 0 的空地址。
#[cfg(target_arch = "loongarch64")]
fn physical(address: usize) -> Result<usize, &'static str> {
    if !matches!(
        address >> DMW_VSEG_SHIFT,
        0 | DMW_UNCACHED_VSEG | DMW_CACHED_VSEG
    ) {
        return Err("firmware address is neither physical nor a supported DMW alias");
    }
    let address = address & DMW_ADDRESS_MASK as usize;
    if address == 0 {
        return Err("firmware address is null");
    }
    Ok(address)
}

/// 从设备树中发现并校验龙芯 2K1000LA 开发板的 LIOINTC、RTC、PMC 及 PCI/MMIO 外设资源。
///
/// 调用上下文：引导早期单核（BSP）环境，由 `platform::discover` 在识别到 `Platform::Ls2k1000la` 时调用。
///
/// 参数：
/// - `tree`：扁平设备树只读视图。
/// - `uart`：已选定的控制台串口节点，用于解析其在双 Bank LIOINTC 下的统一中断号。
/// - `info`：待填充的全局引导信息结构。
///
/// 返回值：
/// - `Ok(())`：所有板级中断控制器、电源控制器与存储/网络外设均通过绑定校验并已写入 `info`。
/// - `Err(&'static str)`：LIOINTC 缺失或路由不匹配、CPU 核号越界、RTC/PMC 缺失或外设绑定非法。
pub(super) fn discover<'a>(
    tree: &'a Fdt<'a>,
    uart: Node<'a>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    // Step 1: 扫描并校验两个 liointc-2.0 中断控制器（分别对应 CPU 父中断 cause 2 和 cause 3）。
    let mut banks = [None; 2];
    for node in tree
        .all_nodes()
        .filter(|node| enabled(*node) && compatible(*node, "loongson,liointc-2.0"))
    {
        require_controller(node, "loongson,liointc-2.0", 2)?;
        identity_address(tree, node)?;
        let root = interrupt_parent(tree, node)?;
        require_controller(root, "loongson,cpu-interrupt-controller", 1)?;
        let cause = super::irq(node, root)?;
        if !(LIOINTC_FIRST_PARENT_CAUSE..=LIOINTC_LAST_PARENT_CAUSE).contains(&cause) {
            return Err("FDT LIOINTC parent cause is unsupported");
        }
        let index = (cause - LIOINTC_FIRST_PARENT_CAUSE) as usize;
        if banks[index].is_some() {
            return Err("FDT LIOINTC parent cause is duplicated");
        }

        let mut registers = regions(node)?;
        let main = registers
            .next()
            .ok_or("FDT LIOINTC main registers are missing")??;
        let isr0 = registers
            .next()
            .ok_or("FDT LIOINTC CPU0 ISR is missing")??;
        let isr1 = registers
            .next()
            .ok_or("FDT LIOINTC CPU1 ISR is missing")??;
        if registers.next().is_some()
            || main.end - main.start != LIOINTC_REGISTER_BYTES
            || isr0.end - isr0.start != LIOINTC_ISR_BYTES
            || isr1.end - isr1.start != LIOINTC_ISR_BYTES
        {
            return Err("FDT LIOINTC register geometry is unsupported");
        }

        let map = property(node, "loongson,parent_int_map")?;
        if map.len() != 4 * FDT_CELL_BYTES {
            return Err("FDT LIOINTC parent masks are truncated");
        }
        let mut parent_map = [0; 4];
        for (slot, value) in parent_map.iter_mut().enumerate() {
            *value = word(map, slot * FDT_CELL_BYTES)?;
            if *value != if slot == index { u32::MAX } else { 0 } {
                return Err("FDT LIOINTC source routing is unsupported");
            }
        }
        info.board.liointc[index] = LioIntcResources {
            registers: main,
            isr: [isr0, isr1],
            parent_interrupt: cause,
            parent_map,
        };
        banks[index] = Some(node);
    }

    // Step 2: 汇总双 Bank LIOINTC（共 64 个中断源），解析串口中断号并校验双核 LA264 CPU 物理编号（0..2）。
    let banks = [
        banks[0].ok_or("FDT LIOINTC0 is missing")?,
        banks[1].ok_or("FDT LIOINTC1 is missing")?,
    ];
    info.interrupts.controller = info.board.liointc[0].registers;
    info.interrupts.parent_controller = info.board.liointc[1].registers;
    info.interrupts.root_interrupt = LIOINTC_FIRST_PARENT_CAUSE;
    info.interrupts.source_count = info.board.liointc.len() as u32 * u32::BITS;
    info.interrupts.uart_source = lio_irq(tree, uart, banks)?;
    if info.hardware_ids[..info.cpu_count]
        .iter()
        .any(|id| 2 <= *id)
    {
        return Err("FDT LA264 hardware CPU ID exceeds the dual-core board");
    }

    // Step 3: 发现唯一的 RTC 与电源管理控制器（PMC），提取 syscon-reboot / syscon-poweroff 子节点配置。
    let rtc = unique(tree, "loongson,ls2k1000-rtc")?;
    info.rtc = single_region(tree, rtc)?;

    let pmc = unique(tree, "loongson,ls2k1000-pmc")?;
    info.board.pmc = single_region(tree, pmc)?;
    for node in pmc.children() {
        if compatible(node, "syscon-reboot") {
            info.board.reboot = Some(power_action(node, info.board.pmc)?);
        }
        if compatible(node, "syscon-poweroff") {
            info.board.poweroff = Some(power_action(node, info.board.pmc)?);
        }
    }

    // Step 4: 发现片上 PCI 拓扑（或无 PCI Host 时的原生 MMIO AHCI/GMAC 外设）。
    discover_pci(tree, banks, info)
}

/// 解析 PMC 子节点（`syscon-reboot` 或 `syscon-poweroff`）中的寄存器偏移、掩码与触发值。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`discover`] 调用。校验偏移量按 4 字节对齐、
/// 不超出 PMC 寄存器窗口范围，且写入值不超出掩码覆盖的位。
fn power_action(node: Node<'_>, registers: Region) -> Result<SysconAction, &'static str> {
    let offset = number(node, "offset")?;
    let mask = number(node, "mask")?;
    let value = resources::optional_number(node, "value")?.unwrap_or(mask);
    if offset % 4 != 0
        || registers.end - registers.start < offset as usize + 4
        || value & !mask != 0
    {
        return Err("FDT syscon power action is invalid");
    }
    Ok(SysconAction {
        registers,
        offset,
        mask,
        value,
    })
}

/// 在设备树中查找唯一启用且匹配指定 `compatible` 字符串的板级控制器节点。
///
/// 调用上下文：引导早期单核（BSP）环境。若节点缺失或存在多个启用实例则返回错误。
fn unique<'a>(tree: &'a Fdt<'a>, binding: &str) -> Result<Node<'a>, &'static str> {
    let mut nodes = tree
        .all_nodes()
        .filter(|node| enabled(*node) && compatible(*node, binding));
    let node = nodes
        .next()
        .ok_or("FDT required LA controller is missing")?;
    if nodes.next().is_some() {
        return Err("FDT required LA controller is duplicated");
    }
    Ok(node)
}

/// 将外设节点在 LIOINTC Bank 0 或 Bank 1 下的局部中断号转换为统一的 `0..64` 外部中断编号。
///
/// 调用上下文：引导早期单核（BSP）环境。支持解析 `interrupts-extended`（3 cells：phandle + source + flags）
/// 与传统 `interrupts`（2 cells：source + flags），要求触发方式必须为高电平触发（`IRQ_TYPE_LEVEL_HIGH`）。
fn lio_irq<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    banks: [Node<'a>; 2],
) -> Result<u32, &'static str> {
    let (controller, source, flags) = if let Some(extended) = node.property("interrupts-extended") {
        if extended.value.len() < 3 * FDT_CELL_BYTES
            || extended.value.len() % (3 * FDT_CELL_BYTES) != 0
        {
            return Err("FDT LIOINTC extended interrupt is truncated");
        }
        let controller = tree
            .find_phandle(word(extended.value, 0)?)
            .ok_or("FDT LIOINTC interrupt parent is missing")?;
        (
            controller,
            word(extended.value, FDT_CELL_BYTES)?,
            word(extended.value, 2 * FDT_CELL_BYTES)?,
        )
    } else {
        let controller = interrupt_parent(tree, node)?;
        let raw = property(node, "interrupts")?;
        if raw.len() != 2 * FDT_CELL_BYTES {
            return Err("FDT LIOINTC interrupt is not two cells");
        }
        (controller, word(raw, 0)?, word(raw, FDT_CELL_BYTES)?)
    };
    let index = banks
        .iter()
        .position(|bank| bank.name.as_ptr() == controller.name.as_ptr())
        .ok_or("FDT device is not connected to selected LIOINTC")?;
    if u32::BITS <= source || flags != IRQ_TYPE_LEVEL_HIGH {
        return Err("FDT LIOINTC device source or trigger is unsupported");
    }
    Ok(index as u32 * u32::BITS + source)
}

/// 解析龙芯 2K1000LA 的片上 PCI 主桥（`loongson,ls2k-pci`）及其集成的 SATA（`sata@8,0`）与网卡（`ethernet@3,0`）功能。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`discover`] 调用。若设备树中不存在任何启用的 PCI Host，
/// 则自动转入 [`discover_mmio`] 解析纯 MMIO 绑定形式的外设。
fn discover_pci<'a>(
    tree: &'a Fdt<'a>,
    banks: [Node<'a>; 2],
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    // Step 1: 查找唯一的 loongson,ls2k-pci 主桥；若不存在任何 PCI 节点，则回退到纯 MMIO 外设发现流程。
    let mut hosts = tree
        .all_nodes()
        .filter(|node| enabled(*node) && compatible(*node, "loongson,ls2k-pci"));
    let Some(host) = hosts.next() else {
        if tree.all_nodes().any(|node| {
            enabled(node) && node.property("device_type").and_then(|p| p.as_str()) == Some("pci")
        }) {
            return Err("FDT enabled LA PCI host has an unsupported binding");
        }
        return discover_mmio(tree, banks, info);
    };
    if hosts.next().is_some() {
        return Err("FDT Loongson PCI host is duplicated");
    }

    // Step 2: 提取 Loongson 专用 CFG0（类型 0 配置空间）与 CFG1（类型 1 配置空间）窗口及总线号范围。
    identity_address(tree, host)?;
    let mut registers = regions(host)?;
    info.pci.configuration = registers.next().ok_or("FDT Loongson CFG0 is missing")??;
    info.pci.configuration_extended = registers.next().ok_or("FDT Loongson CFG1 is missing")??;
    if registers.next().is_some() {
        return Err("FDT Loongson PCI has unsupported configuration windows");
    }
    info.pci.kind = PciConfiguration::Loongson;
    let buses = property(host, "bus-range")?;
    if buses.len() != 2 * FDT_CELL_BYTES
        || word(buses, 0)? != 0
        || (u8::MAX as u32) < word(buses, FDT_CELL_BYTES)?
    {
        return Err("FDT Loongson PCI bus range is invalid");
    }
    info.pci.bus_start = 0;
    info.pci.bus_end = word(buses, FDT_CELL_BYTES)?;

    // Step 3: 解析 PCI 主桥的 ranges 地址转换窗口（I/O、32 位内存、64 位内存），用于后续 BAR 分配与校验。
    let ranges = property(host, "ranges")?;
    if ranges.is_empty() || ranges.len() % PCI_RANGE_BYTES != 0 {
        return Err("FDT Loongson PCI ranges are truncated");
    }
    for range in ranges.chunks_exact(PCI_RANGE_BYTES) {
        if info.pci.range_count == info.pci.ranges.len() {
            return Err("FDT Loongson PCI range capacity exceeded");
        }
        let flags = word(range, 0)?;
        if !matches!(
            flags & PCI_SPACE_CODE_MASK,
            PCI_SPACE_IO | PCI_SPACE_MEM32 | PCI_SPACE_MEM64
        ) {
            return Err("FDT Loongson PCI window kind is unsupported");
        }
        let cpu = Region::from_size(
            cells(&range[PCI_RANGE_CPU_OFFSET..PCI_RANGE_SIZE_OFFSET])?,
            cells(&range[PCI_RANGE_SIZE_OFFSET..PCI_RANGE_BYTES])?,
        )?;
        device_region(cpu, info)?;
        info.pci.ranges[info.pci.range_count] = PciRange {
            flags,
            bus_start: cells(&range[PCI_RANGE_BUS_OFFSET..PCI_RANGE_CPU_OFFSET])?,
            cpu,
        };
        info.pci.range_count += 1;
    }

    // Step 4: 提取 PCI 主桥下的 sata@8,0（AHCI）与 ethernet@3,0（GMAC0）子节点，记录其 BDF 身份与固定中断路由。
    info.pci.intx_mask = [PCI_BDF_MASK, 0, 0, 0];
    let sata = host
        .children()
        .find(|node| node.name == "sata@8,0" && enabled(*node))
        .ok_or("FDT selected SATA function is missing or disabled")?;
    let ahci = pci_function(tree, sata, banks)?;
    add_route(info, ahci)?;
    info.board.ahci = Some(AhciResources {
        pci: Some(ahci),
        interrupt_source: ahci.interrupt_source,
        dma_coherent: ahci.dma_coherent,
        ports_implemented: resources::optional_number(sata, "ports-implemented")?,
        ..AhciResources::default()
    });

    let mac = host
        .children()
        .find(|node| node.name == "ethernet@3,0" && enabled(*node))
        .ok_or("FDT selected GMAC0 function is missing or disabled")?;
    let identity = pci_function(tree, mac, banks)?;
    add_route(info, identity)?;
    let mut gmac = resources::gmac(tree, mac, false)?;
    gmac.pci = Some(identity);
    gmac.interrupt_source = identity.interrupt_source;
    info.board.gmac = Some(gmac);
    Ok(())
}

/// 解析 PCI 子节点的 5-cell `reg` 属性，提取其配置空间 BDF（Bus/Device/Function）标识、LIOINTC 中断号与 DMA 一致性属性。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`discover_pci`] 调用。
/// 根据 IEEE 1275 PCI 绑定规范，片上 PCI 功能节点的 `reg` 首 cell 存放 BDF 编码，后 4 个 cell（基地址与长度）
/// 均为 0；实际的控制器寄存器基地址（BAR）将在内核 PCI 枚举阶段从 PCI 配置空间读取或分配。
fn pci_function<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    banks: [Node<'a>; 2],
) -> Result<PciFunctionResource, &'static str> {
    let reg = property(node, "reg")?;
    if reg.len() != 5 * FDT_CELL_BYTES || reg[FDT_CELL_BYTES..].iter().any(|byte| *byte != 0) {
        return Err("FDT selected PCI function reg is not configuration identity");
    }
    let address = word(reg, 0)?;
    if address & !PCI_BDF_MASK != 0 {
        return Err("FDT PCI configuration identity is invalid");
    }
    Ok(PciFunctionResource {
        bus: (address >> PCI_BUS_SHIFT) as u8,
        device: ((address >> PCI_DEVICE_SHIFT) & u32::from(PCI_DEVICE_COUNT - 1)) as u8,
        function: ((address >> PCI_FUNCTION_SHIFT) & u32::from(PCI_FUNCTION_COUNT - 1)) as u8,
        interrupt_source: lio_irq(tree, node, banks)?,
        dma_coherent: resources::coherent(tree, node, false)?,
    })
}

/// 将片上 PCI 功能的完整 BDF 标识与设备树中声明的 LIOINTC 中断源写入 `info.pci.intx_routes` 路由表。
///
/// 调用上下文：引导早期单核（BSP）环境，由 [`discover_pci`] 调用。2K1000LA 片上集成的 PCI 外设
/// 采用固定硬连线直连 LIOINTC，不依赖 PCI 配置空间中的 `Interrupt Line` 寄存器。
fn add_route(info: &mut BootInfo, function: PciFunctionResource) -> Result<(), &'static str> {
    if info.pci.intx_route_count == info.pci.intx_routes.len() {
        return Err("FDT PCI function route capacity exceeded");
    }
    info.pci.intx_routes[info.pci.intx_route_count] = PciIntxRoute {
        child: [
            ((function.bus as u32) << PCI_BUS_SHIFT)
                | ((function.device as u32) << PCI_DEVICE_SHIFT)
                | ((function.function as u32) << PCI_FUNCTION_SHIFT),
            0,
            0,
            0,
        ],
        source: function.interrupt_source,
    };
    info.pci.intx_route_count += 1;
    Ok(())
}

/// 在未启用 PCI 主桥的设备树中，直接按平台 MMIO 绑定解析 AHCI 与千兆网卡（`ethernet0`）资源。
///
/// 调用上下文：引导早期单核（BSP）环境，当 [`discover_pci`] 检测到设备树不含任何启用的 PCI Host 时调用。
/// 此时要求 AHCI 节点不能挂在 `device_type = "pci"` 的父总线之下，且必须提供完整的 MMIO 寄存器窗口。
fn discover_mmio<'a>(
    tree: &'a Fdt<'a>,
    banks: [Node<'a>; 2],
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    let mut sata_nodes = tree.all_nodes().filter(|node| {
        enabled(*node)
            && [
                "loongson,ls-ahci",
                "loongson,ls2k1000-ahci",
                "loongson,2k1000-ahci",
                "snps,spear-ahci",
            ]
            .iter()
            .any(|binding| compatible(*node, binding))
    });
    let sata = sata_nodes
        .next()
        .ok_or("FDT native AHCI MMIO binding is missing")?;
    if sata_nodes.next().is_some() {
        return Err("FDT native AHCI MMIO binding is duplicated");
    }
    // Step 1: 校验 AHCI 节点不位于 PCI 总线下，提取并验证其 MMIO 寄存器窗口、DMA 掩码与中断号。
    let bus = parent(tree, sata).ok_or("FDT AHCI parent bus is missing")?;
    if bus.property("device_type").and_then(|p| p.as_str()) == Some("pci") {
        return Err("FDT AHCI PCI identity cannot be used as MMIO");
    }

    let registers = single_region(tree, sata)?;
    device_region(registers, info)?;
    if registers.end - registers.start < crate::drivers::ahci::MMIO_BYTES {
        return Err("FDT AHCI MMIO window is too short");
    }

    let dma_mask = sata
        .property("dma-mask")
        .map(|p| cells(p.value).map(|value| value as u64))
        .transpose()?;
    if dma_mask.is_some_and(|mask| mask < u32::MAX as u64) {
        return Err("FDT AHCI DMA mask is narrower than 32 bits");
    }
    info.board.ahci = Some(AhciResources {
        registers,
        pci: None,
        interrupt_source: lio_irq(tree, sata, banks)?,
        dma_coherent: resources::coherent(tree, sata, false)?,
        dma_mask,
        ports_implemented: resources::optional_number(sata, "ports-implemented")?,
    });

    // Step 2: 解析别名 ethernet0 指向的 DWMAC1000 MMIO 网卡节点，校验其寄存器窗口长度与 LIOINTC 中断号。
    let mac = resources::alias(tree, "ethernet0")?;
    if !(compatible(mac, "snps,dwmac-3.70a") || compatible(mac, "snps,arc-dwmac-3.70a")) {
        return Err("FDT ethernet0 is not a DWMAC1000 MMIO binding");
    }
    let mut gmac = resources::gmac(tree, mac, false)?;
    gmac.registers = single_region(tree, mac)?;
    device_region(gmac.registers, info)?;
    if gmac.registers.end - gmac.registers.start < crate::drivers::dwmac::DWMAC1000_MMIO_MIN_BYTES {
        return Err("FDT GMAC MMIO window is too short");
    }
    gmac.interrupt_source = lio_irq(tree, mac, banks)?;
    info.board.gmac = Some(gmac);
    Ok(())
}
