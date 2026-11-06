//! 平台早期固件交接与扁平设备树（FDT / DTB）一次性无堆资源发现。
//!
//! # 模块职责与架构定位
//! 本模块位于架构启动入口（`crate::arch`）与物理内存管理器（`crate::mm`）及设备驱动（`crate::drivers`）之间，
//! 在动态内存分配器（Heap / Buddy）尚未建立的早期启动阶段，对固件传入的设备树二进制块（DTB）与启动参数执行严格边界校验和单次扫描，
//! 将物理内存拓扑、保留内存区间、CPU 硬件 ID、中断控制器路由、主控制台 UART、RTC、VirtIO-MMIO、PCI ECAM 及板级外设资源汇总写入全局 [`BootInfo`] 快照：
//! - **上游调用者**：由引导核（BSP）在早期启动入口中通过 [`discover`] 唯一调用一次；或由 `ktest` 单元测试通过 `ktest_*` 入口在栈上独立验证。
//! - **下游消费者**：`crate::boot`（发布只读全局启动信息）、`crate::mm`（按可用 RAM 与保留区间初始化物理页帧分配器）、`crate::arch` / `crate::drivers`（初始化中断控制器、定时器、串口、块设备与网卡）。
//!
//! # 核心设计与数据布局
//!
//! ## 1. 扁平设备树（FDT v17）二进制布局与前置校验（[`validate`]）
//! ```text
//! +-------------------------------------------------------------------+
//! | Header (40 bytes, FDT_MAGIC = 0xd00dfeed, version = 17)           |
//! +-------------------------------------------------------------------+
//! | Memory Reservation Block (8 字节对齐, (addr: u64, size: u64) 列表, |
//! |                           以全 0 的 16 字节条目结尾)               |
//! +-------------------------------------------------------------------+
//! | Structure Block (4 字节对齐, FDT_BEGIN_NODE / FDT_PROPERTY /       |
//! |                  FDT_END_NODE / FDT_NOP 词法流, 以 FDT_END 结尾)   |
//! +-------------------------------------------------------------------+
//! | Strings Block (以 NUL 结尾的属性名称字符串池)                      |
//! +-------------------------------------------------------------------+
//! ```
//! 在调用第三方 `fdt` 库遍历节点前，[`validate`] 先对上述三个数据块做不重叠检查、对齐检查、最大深度（`FDT_MAX_DEPTH = 64`）检查及字符串 NUL 终止检查，防止损坏的固件数据触发解析器越界或死循环。
//!
//! ## 2. 物理内存与保留区间发现流水线（[`discover_tree`]）
//! ```text
//! [1. 登记静态保留区]
//!     ├── DTB 自身占用的物理区间 (LoongArch QEMU 额外保留低 1 MiB 固件区)
//!     ├── FDT Header 中的 Memory Reservation Block 条目
//!     └── /reserved-memory 下带固定 "reg" 的静态子节点 (区分普通保留与 "no-map" 整页禁映射)
//!          │
//!          ▼
//! [2. 发现可用 DRAM 与固件低地址保护]
//!     ├── 收集所有启用且 device_type = "memory" 的物理内存段 (拒绝重叠段)
//!     └── 真实开发板 (VF2 / LS2K1000LA) 自动将内核装载地址之前的同段低地址 RAM 标记为保留，
//!         保护 OpenSBI / PMON / UEFI 驻留代码与交接数据不被后续页帧分配器分配
//!          │
//!          ▼
//! [3. 发现 CPU、时钟频率、/chosen (bootargs + initrd) 与平台外设]
//!     ├── 登记 initrd 物理区间为保留区
//!     └── 分发至 QEMU Virt / VisionFive 2 / LS2K1000LA 专属外设发现逻辑
//!          │
//!          ▼
//! [4. 动态保留内存选址 (discover_dynamic_reserved_memory)]
//!     └── 为 /reserved-memory 下仅声明 "size" (无 "reg") 的节点在可用 RAM 窗口内
//!         按对齐要求搜索不与已有保留区及内核镜像冲突的物理区间并登记
//! ```
//!
//! # 锁顺序与并发契约
//! - 本模块不持有任何自旋锁或睡眠锁。
//! - [`discover`] 只能在引导核（BSP）关闭本地中断、从核（AP）尚未唤醒的单执行流环境下调用一次，独占初始化全局 [`BootInfo`] 槽位；一旦发布为只读引用，运行期不再修改。

use crate::boot::{
    BootInfo, PciConfiguration, PciIntxRoute, PciRange, Platform, Region, VirtioMmioResource,
    MAX_CPUS, MAX_INTERRUPT_SOURCES, MAX_MEMORY_REGIONS, MAX_VIRTIO_MMIO, PCI_INTX_PIN_COUNT,
    PCI_SPACE_CODE_MASK, PCI_SPACE_IO, PCI_SPACE_MEM32, PCI_SPACE_MEM64,
};
use crate::mm::PAGE_SIZE;
use fdt::{node::FdtNode, Fdt};

pub mod layout;
mod ls2k1000la;
mod resources;
mod visionfive2;

/// 当前编译目标架构与板型对应的内核链接地址布局常量（与 `build.rs` 生成链接脚本时使用的配置严格一致）。
pub const LINK_LAYOUT: layout::KernelLinkLayout = if cfg!(target_arch = "riscv64") {
    if cfg!(blossom_platform = "visionfive2") {
        layout::RISCV64_VISIONFIVE2
    } else {
        layout::RISCV64_QEMU_VIRT
    }
} else if cfg!(blossom_platform = "ls2k1000la") {
    layout::LOONGARCH64_LS2K1000LA
} else {
    layout::LOONGARCH64_QEMU_VIRT
};

/// 返回编译期选定的目标平台枚举值（纯编译期常量，不读取运行期固件数据）。
pub const fn current() -> Platform {
    if cfg!(blossom_platform = "visionfive2") {
        Platform::VisionFive2
    } else if cfg!(blossom_platform = "ls2k1000la") {
        Platform::Ls2k1000la
    } else {
        Platform::QemuVirt
    }
}

/// RISC-V 64 平台早期调试串口（UART0）的固定物理基地址（`0x1000_0000`）。
#[cfg(target_arch = "riscv64")]
pub const EARLY_UART_BASE: usize = 0x1000_0000;
/// LoongArch64 平台早期调试串口的固定物理基地址（龙芯 2K1000LA 为 `0x1fe2_0000`，QEMU `virt` 为 `0x1fe0_01e0`）。
#[cfg(target_arch = "loongarch64")]
pub const EARLY_UART_BASE: usize = if cfg!(blossom_platform = "ls2k1000la") {
    0x1fe2_0000
} else {
    0x1fe0_01e0
};

/// 内核允许解析的设备树二进制块（DTB）最大字节长度（2 MiB）。
const MAX_DTB_SIZE: usize = 2 * 1024 * 1024;
/// `/reserved-memory` 动态预留节点未显式声明 `alignment` 属性时采用的默认字节对齐（64 字节）。
const DEFAULT_RESERVED_ALIGNMENT: usize = 64;

/// 扁平设备树（FDT）头部魔数（`0xd00d_feed`）。
const FDT_MAGIC: u32 = 0xd00d_feed;
/// 内核要求设备树必须满足的规范版本号（v17）。
const FDT_VERSION: u32 = 17;
/// 设备树单个 cell 的字节宽度（4 字节，对应大端 `u32`）。
const FDT_CELL_BYTES: usize = core::mem::size_of::<u32>();
/// 设备树双 cell 的字节宽度（8 字节，对应大端 `u64`）。
const FDT_DOUBLE_CELL_BYTES: usize = 2 * FDT_CELL_BYTES;
/// 扁平设备树（FDT）标准头部的总字节长度（40 字节，共 10 个 cell）。
const FDT_HEADER_BYTES: usize = 10 * FDT_CELL_BYTES;
/// FDT 头部中 `totalsize` 字段的字节偏移（4）。
const FDT_TOTAL_SIZE: usize = FDT_CELL_BYTES;
/// FDT 头部中 `off_dt_struct`（结构块偏移）字段的字节偏移（8）。
const FDT_STRUCTURE_OFFSET: usize = 2 * FDT_CELL_BYTES;
/// FDT 头部中 `off_dt_strings`（字符串块偏移）字段的字节偏移（12）。
const FDT_STRINGS_OFFSET: usize = 3 * FDT_CELL_BYTES;
/// FDT 头部中 `off_mem_rsvmap`（内存保留块偏移）字段的字节偏移（16）。
const FDT_RESERVATIONS_OFFSET: usize = 4 * FDT_CELL_BYTES;
/// FDT 头部中 `version`（版本号）字段的字节偏移（20）。
const FDT_VERSION_OFFSET: usize = 5 * FDT_CELL_BYTES;
/// FDT 头部中 `last_comp_version`（最低兼容版本号）字段的字节偏移（24）。
const FDT_LAST_COMPATIBLE_VERSION: usize = 6 * FDT_CELL_BYTES;
/// FDT 头部中 `size_dt_strings`（字符串块字节长度）字段的字节偏移（32）。
const FDT_STRINGS_SIZE: usize = 8 * FDT_CELL_BYTES;
/// FDT 头部中 `size_dt_struct`（结构块字节长度）字段的字节偏移（36）。
const FDT_STRUCTURE_SIZE: usize = 9 * FDT_CELL_BYTES;
/// FDT 内存保留块（Memory Reservation Block）起始偏移及内部条目的字节对齐要求（8 字节）。
const FDT_RESERVATION_ALIGNMENT: usize = core::mem::size_of::<u64>();
/// FDT 内存保留块单个条目（64 位起始物理地址 + 64 位字节长度）的字节大小（16 字节）。
const FDT_RESERVATION_BYTES: usize = 2 * FDT_RESERVATION_ALIGNMENT;
/// FDT 结构块允许的最大节点嵌套深度（64 层），用于栈上定长状态跟踪并防止恶意深层嵌套。
const FDT_MAX_DEPTH: usize = 64;

/// FDT 结构块词法标记：节点开始（`FDT_BEGIN_NODE = 1`）。
const FDT_BEGIN_NODE: u32 = 1;
/// FDT 结构块词法标记：节点结束（`FDT_END_NODE = 2`）。
const FDT_END_NODE: u32 = 2;
/// FDT 结构块词法标记：属性定义（`FDT_PROPERTY = 3`）。
const FDT_PROPERTY: u32 = 3;
/// FDT 结构块词法标记：空操作填充（`FDT_NOP = 4`）。
const FDT_NOP: u32 = 4;
/// FDT 结构块词法标记：结构块结束（`FDT_END = 9`）。
const FDT_END: u32 = 9;

/// PCI 主桥 `ranges` 属性单条地址映射记录的字节长度（7 个 cell，共 28 字节：3 cell 子地址 + 2 cell CPU 物理地址 + 2 cell 长度）。
const PCI_RANGE_BYTES: usize = 7 * FDT_CELL_BYTES;
/// PCI `ranges` 条目中 64 位 PCI 总线地址字段的起始字节偏移（跳过首个标志 cell，偏移 4 字节）。
const PCI_RANGE_BUS_OFFSET: usize = FDT_CELL_BYTES;
/// PCI `ranges` 条目中 64 位 CPU 物理地址字段的起始字节偏移（偏移 12 字节）。
const PCI_RANGE_CPU_OFFSET: usize = 3 * FDT_CELL_BYTES;
/// PCI `ranges` 条目中 64 位窗口大小字段的起始字节偏移（偏移 20 字节）。
const PCI_RANGE_SIZE_OFFSET: usize = 5 * FDT_CELL_BYTES;
/// PCI `interrupt-map` 条目中父中断控制器 phandle 字段的字节偏移（前 4 个 cell 为 3 cell PCI 地址 + 1 cell INTx 引脚号，偏移 16 字节）。
const PCI_INTX_PARENT_OFFSET: usize = 4 * FDT_CELL_BYTES;
/// PCI `interrupt-map` 条目中父控制器中断源编号（source）字段的字节偏移（偏移 20 字节）。
const PCI_INTX_SOURCE_OFFSET: usize = PCI_INTX_PARENT_OFFSET + FDT_CELL_BYTES;
/// 仅包含 1 个父中断 cell（省略触发方式标志）的旧版 PCI `interrupt-map` 单条记录字节长度（24 字节）。
const PCI_INTX_LEGACY_BYTES: usize = PCI_INTX_SOURCE_OFFSET + FDT_CELL_BYTES;
/// PCIe ECAM 配置空间中单条总线（Bus）占用的地址位移量（$2^{20} = 1\text{ MiB}$ / bus）。
const PCI_ECAM_BUS_SHIFT: u32 = 20;

/// 设备树中断触发类型标志：高电平触发（`IRQ_TYPE_LEVEL_HIGH = 4`）。
const IRQ_TYPE_LEVEL_HIGH: u32 = 4;
/// 单个 VirtIO-MMIO 设备寄存器窗口的标准字节长度与对齐粒度（4 KiB）。
const VIRTIO_MMIO_WINDOW_BYTES: usize = 0x1000;
/// QEMU `virt` 平台 PCI 主桥标准的 `interrupt-map-mask` 四元组（掩取设备号 bits `[12:11]` 与全部 3 位引脚号）。
const QEMU_PCI_INTX_MASK: [u32; 4] = [0x1800, 0, 0, 7];

/// RISC-V QEMU `virt` 平台 Goldfish RTC 寄存器窗口的字节长度与对齐粒度（4 KiB）。
#[cfg(target_arch = "riscv64")]
const GOLDFISH_RTC_WINDOW_BYTES: usize = 0x1000;
/// LoongArch64 平台 LS7A RTC 寄存器窗口的字节长度与对齐粒度（256 字节）。
#[cfg(target_arch = "loongarch64")]
const LS7A_RTC_WINDOW_BYTES: usize = 0x100;
/// LoongArch64 QEMU `virt` 虚拟机固件放置 DTB 的固定物理地址（`0x10_0000`，即 1 MiB 处）。
#[cfg(target_arch = "loongarch64")]
const QEMU_LOONGARCH_DTB_PHYS: usize = 0x10_0000;

/// 绑定到同一设备树切片生命周期的 FDT 节点类型别名。
type Node<'a> = FdtNode<'a, 'a>;

/// 从字节切片 `bytes` 的指定偏移 `at` 处读取一个 32 位大端整数（1 个 FDT cell）；若偏移计算溢出或剩余字节不足 4 字节则返回错误。
fn word(bytes: &[u8], at: usize) -> Result<u32, &'static str> {
    let end = at
        .checked_add(FDT_CELL_BYTES)
        .ok_or("FDT field offset overflows")?;
    let value = bytes.get(at..end).ok_or("FDT field is truncated")?;
    Ok(u32::from_be_bytes([value[0], value[1], value[2], value[3]]))
}

/// 将 0、1 或 2 个大端 cell 的字节切片转换为 `usize` 地址或大小数值（空切片对应 `#size-cells = 0`，返回 `0`）。
fn cells(bytes: &[u8]) -> Result<usize, &'static str> {
    match bytes.len() {
        0 => Ok(0),
        FDT_CELL_BYTES => Ok(word(bytes, 0)? as usize),
        FDT_DOUBLE_CELL_BYTES => Ok((((word(bytes, 0)? as u64) << u32::BITS)
            | word(bytes, FDT_CELL_BYTES)? as u64) as usize),
        _ => Err("FDT address or size is not one or two cells"),
    }
}

/// 从字节切片头部借用首个以 NUL（`\0`）结尾的 UTF-8 字符串切片；若缺少终止符或包含非法 UTF-8 序列则返回错误。
fn cstr(bytes: &[u8]) -> Result<&str, &'static str> {
    let end = bytes
        .iter()
        .position(|b| *b == 0)
        .ok_or("FDT string is not NUL terminated")?;
    core::str::from_utf8(&bytes[..end]).map_err(|_| "FDT string is not valid UTF-8")
}

/// 在完整 DTB 字节切片中安全截取指定偏移与长度的数据块（要求起始偏移不侵入 40 字节的 FDT 头部且结束位置不越界）。
fn block(bytes: &[u8], offset: usize, size: usize) -> Result<&[u8], &'static str> {
    if offset < FDT_HEADER_BYTES {
        return Err("FDT block overlaps the header");
    }

    let end = offset
        .checked_add(size)
        .ok_or("FDT block range overflows")?;
    bytes
        .get(offset..end)
        .ok_or("FDT block exceeds the total size")
}

/// 在交由 `fdt` 库解析前，对原始 DTB 字节流执行完整的二进制结构、块边界、对齐、嵌套深度与关键属性格式校验。
///
/// - **调用上下文**：[`discover_tree`] 或测试辅助函数在构造 `Fdt` 实例前首先调用。
/// - **参数**：`bytes` 为待验证的完整 DTB 字节切片。
/// - **返回值**：全部结构与边界合法时返回 `Ok(())`，发现任何截断、重叠或格式违例时返回静态错误描述。
fn validate(bytes: &[u8]) -> Result<(), &'static str> {
    // Step 1: 校验 40 字节头部的魔数、总长度（不超过 2 MiB 且等于切片长度）以及 FDT v17 版本号。
    if word(bytes, 0)? != FDT_MAGIC {
        return Err("FDT magic is invalid");
    }
    let total = word(bytes, FDT_TOTAL_SIZE)? as usize;
    if !(FDT_HEADER_BYTES..=MAX_DTB_SIZE).contains(&total) || total != bytes.len() {
        return Err("FDT total size is invalid or truncated");
    }
    if word(bytes, FDT_VERSION_OFFSET)? != FDT_VERSION
        || FDT_VERSION < word(bytes, FDT_LAST_COMPATIBLE_VERSION)?
    {
        return Err("FDT version is not v17");
    }

    // Step 2: 提取结构块（Structure）、字符串块（Strings）与内存保留块（Reservations），校验其对齐要求并扫描保留块终止项，确保三个数据块互不重叠。
    let structure_start = word(bytes, FDT_STRUCTURE_OFFSET)? as usize;
    let strings_start = word(bytes, FDT_STRINGS_OFFSET)? as usize;
    let reserve_start = word(bytes, FDT_RESERVATIONS_OFFSET)? as usize;
    let structure = block(
        bytes,
        structure_start,
        word(bytes, FDT_STRUCTURE_SIZE)? as usize,
    )?;
    let strings = block(
        bytes,
        strings_start,
        word(bytes, FDT_STRINGS_SIZE)? as usize,
    )?;
    if structure_start % FDT_CELL_BYTES != 0 || reserve_start % FDT_RESERVATION_ALIGNMENT != 0 {
        return Err("FDT structure or reservation block is unaligned");
    }

    let mut reserve_end = reserve_start;
    loop {
        let entry = block(bytes, reserve_end, FDT_RESERVATION_BYTES)?;
        reserve_end += FDT_RESERVATION_BYTES;
        if entry.iter().all(|b| *b == 0) {
            break;
        }
    }

    let spans = [
        (structure_start, structure_start + structure.len()),
        (strings_start, strings_start + strings.len()),
        (reserve_start, reserve_end),
    ];
    for i in 0..spans.len() {
        for j in i + 1..spans.len() {
            if spans[i].0 < spans[j].1 && spans[j].0 < spans[i].1 {
                return Err("FDT blocks overlap");
            }
        }
    }

    // Step 3: 顺序扫描结构块状态机，检查节点名 ASCII 合法性、最大树深度、属性必须位于子节点之前、字符串偏移有效以及关键属性（aliases、#*cells、compatible 等）格式合法。
    let mut at = 0;
    let mut depth = 0usize;
    let mut children = [false; FDT_MAX_DEPTH];
    let mut aliases = [false; FDT_MAX_DEPTH];
    let mut root_seen = false;
    loop {
        let token = word(structure, at)?;
        at += FDT_CELL_BYTES;
        match token {
            FDT_BEGIN_NODE => {
                if FDT_MAX_DEPTH - 1 <= depth || (depth == 0 && root_seen) {
                    return Err("FDT node depth exceeds capacity or has multiple roots");
                }

                let name = cstr(structure.get(at..).ok_or("FDT node name is truncated")?)?;
                if !name.is_ascii() || (depth == 0 && !name.is_empty()) {
                    return Err("FDT node name or root is invalid");
                }
                at = at
                    .checked_add((name.len() + FDT_CELL_BYTES) & !(FDT_CELL_BYTES - 1))
                    .ok_or("FDT node name size overflows")?;
                if structure.len() < at {
                    return Err("FDT node name padding is truncated");
                }

                if 0 < depth {
                    children[depth - 1] = true;
                }
                children[depth] = false;
                aliases[depth] = depth == 1 && name == "aliases";
                depth += 1;
                root_seen = true;
            }
            FDT_END_NODE => {
                depth = depth
                    .checked_sub(1)
                    .ok_or("FDT end-node token is unmatched")?;
            }
            FDT_PROPERTY => {
                if depth == 0 || children[depth - 1] {
                    return Err("FDT property is outside the property section");
                }

                let len = word(structure, at)? as usize;
                let name_at = word(structure, at + FDT_CELL_BYTES)? as usize;
                at += 2 * FDT_CELL_BYTES;
                let name = cstr(
                    strings
                        .get(name_at..)
                        .ok_or("FDT property name offset exceeds the string block")?,
                )?;
                let end = at.checked_add(len).ok_or("FDT property size overflows")?;
                let value = structure
                    .get(at..end)
                    .ok_or("FDT property value is truncated")?;

                if aliases[depth - 1] && (name.contains('/') || !cstr(value)?.starts_with('/')) {
                    return Err("FDT aliases require slash-free names and absolute paths");
                }
                if matches!(name, "#address-cells" | "#size-cells" | "#interrupt-cells")
                    && (value.len() != FDT_CELL_BYTES || 3 < word(value, 0)?)
                {
                    return Err("FDT cell count exceeds the supported range");
                }
                if matches!(
                    name,
                    "compatible" | "status" | "device_type" | "stdout-path"
                ) {
                    let mut list = value;
                    while !list.is_empty() {
                        let text = cstr(list)?;
                        list = &list[text.len() + 1..];
                    }
                }

                at = end
                    .checked_add(FDT_CELL_BYTES - 1)
                    .ok_or("FDT property padding overflows")?
                    & !(FDT_CELL_BYTES - 1);
                if structure.len() < at {
                    return Err("FDT property padding is truncated");
                }
            }
            FDT_NOP => {}
            FDT_END => {
                if depth != 0 || !root_seen || structure[at..].iter().any(|b| *b != 0) {
                    return Err("FDT structure end token is invalid");
                }
                return Ok(());
            }
            _ => return Err("FDT structure contains an unknown token"),
        }
    }
}

/// 读取节点上必须存在的属性值字节切片；若属性缺失则返回错误。
fn property<'a>(node: Node<'a>, name: &str) -> Result<&'a [u8], &'static str> {
    node.property(name)
        .map(|p| p.value)
        .ok_or("FDT required resource property is missing")
}

/// 读取节点上必须存在的单 cell（4 字节大端 `u32`）整数属性。
fn number(node: Node<'_>, name: &str) -> Result<u32, &'static str> {
    let value = property(node, name)?;
    if value.len() != FDT_CELL_BYTES {
        return Err("FDT integer property is not one cell");
    }
    word(value, 0)
}

/// 按设备树规范判断节点是否处于启用状态：未声明 `status` 属性，或 `status` 值为 `"okay"` / `"ok"` 时返回 `true`。
fn enabled(node: Node<'_>) -> bool {
    node.property("status")
        .is_none_or(|p| matches!(p.as_str(), Some("okay" | "ok")))
}

/// 判断节点的 `compatible` 字符串列表中是否包含指定的驱动绑定标识 `expected`。
fn compatible(node: Node<'_>, expected: &str) -> bool {
    node.compatible()
        .is_some_and(|c| c.all().any(|s| s == expected))
}

/// 在已校验的设备树中查找指定节点 `node` 的直接父节点（通过比较子节点名称切片的原始指针身份精确定位）。
fn parent<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Option<Node<'a>> {
    tree.all_nodes().find(|candidate| {
        candidate
            .children()
            .any(|child| child.name.as_ptr() == node.name.as_ptr())
    })
}

/// 沿设备树的中断父链（优先解析节点自身的 `interrupt-parent` phandle，未声明则向结构父节点回溯）查找直接管理该节点的中断控制器节点。
fn interrupt_parent<'a>(tree: &'a Fdt<'a>, mut node: Node<'a>) -> Result<Node<'a>, &'static str> {
    loop {
        if node.property("interrupt-parent").is_some() {
            return tree
                .find_phandle(number(node, "interrupt-parent")?)
                .ok_or("FDT interrupt-parent phandle has no controller");
        }
        node = parent(tree, node).ok_or("FDT interrupt-parent relationship is missing")?;
    }
}

/// 校验中断控制器节点处于启用状态、兼容指定的 `expected` 绑定、声明了 `interrupt-controller` 属性且 `#interrupt-cells` 等于 `count`。
fn require_controller(node: Node<'_>, expected: &str, count: u32) -> Result<(), &'static str> {
    if !enabled(node)
        || !compatible(node, expected)
        || node.property("interrupt-controller").is_none()
    {
        return Err("FDT interrupt chain is not the selected platform controller");
    }
    if number(node, "#interrupt-cells")? != count {
        return Err("FDT interrupt cell count does not match the controller binding");
    }
    Ok(())
}

/// 按照父节点声明的 `#address-cells` 与 `#size-cells` 宽度，遍历当前节点的 `reg` 属性并逐段构造不溢出的物理地址区间 [`Region`]。
fn regions(
    node: Node<'_>,
) -> Result<impl Iterator<Item = Result<Region, &'static str>> + '_, &'static str> {
    let raw = node
        .raw_reg()
        .ok_or("FDT resource node has no reg property")?;
    let value = property(node, "reg")?;
    let first = node
        .raw_reg()
        .and_then(|mut r| r.next())
        .ok_or("FDT reg is empty or truncated")?;
    let stride = first.address.len() + first.size.len();
    if stride == 0
        || value.len() % stride != 0
        || first.address.is_empty()
        || 2 * FDT_CELL_BYTES < first.address.len()
        || first.size.is_empty()
        || 2 * FDT_CELL_BYTES < first.size.len()
    {
        return Err("FDT reg address or size format is invalid");
    }
    Ok(raw.map(|reg| Region::from_size(cells(reg.address)?, cells(reg.size)?)))
}

/// 沿节点向根节点的祖先总线链逐层检查 `ranges` 属性，验证该节点的所有 `reg` 资源均位于恒等映射（子地址等于父物理地址）的总线窗口内。
fn identity_address<'a>(tree: &'a Fdt<'a>, mut node: Node<'a>) -> Result<(), &'static str> {
    while let Some(bus) = parent(tree, node) {
        if bus.name != "/" {
            let ranges = property(bus, "ranges")?;
            if !ranges.is_empty() {
                let ancestor = parent(tree, bus).ok_or("FDT bus has no parent")?;
                let child_cells = bus.cell_sizes().address_cells;
                let parent_cells = ancestor.cell_sizes().address_cells;
                let size_cells = bus.cell_sizes().size_cells;
                let stride = (child_cells + parent_cells + size_cells) * FDT_CELL_BYTES;
                if child_cells != parent_cells || stride == 0 || ranges.len() % stride != 0 {
                    return Err("FDT bus ranges format is unsupported");
                }

                for resource in regions(node)? {
                    let resource = resource?;
                    let mut covered = false;
                    for range in ranges.chunks_exact(stride) {
                        let width = child_cells * FDT_CELL_BYTES;
                        let start = cells(&range[..width])?;
                        if start != cells(&range[width..width * 2])? {
                            return Err("FDT resource bus is not identity mapped");
                        }
                        let window = Region::from_size(start, cells(&range[width * 2..])?)?;
                        covered |= window.start <= resource.start && resource.end <= window.end;
                    }
                    if !covered {
                        return Err("FDT resource is outside its bus windows");
                    }
                }
            }
        }
        node = bus;
    }
    Ok(())
}

/// 验证节点位于恒等映射总线上，并提取其 `reg` 属性中唯一的一段寄存器物理地址区间（若缺失或包含多段则返回错误）。
fn single_region<'a>(tree: &'a Fdt<'a>, node: Node<'a>) -> Result<Region, &'static str> {
    identity_address(tree, node)?;
    let mut ranges = regions(node)?;
    let first = ranges.next().ok_or("FDT reg has no resource range")??;
    if ranges.next().is_some() {
        return Err("FDT device has multiple unsupported register ranges");
    }
    Ok(first)
}

/// 验证节点位于恒等映射总线上，并按索引 `index` 提取其 `reg` 属性中的第 `index` 段寄存器物理地址区间。
#[cfg(target_arch = "riscv64")]
fn region_at<'a>(tree: &'a Fdt<'a>, node: Node<'a>, index: usize) -> Result<Region, &'static str> {
    identity_address(tree, node)?;
    regions(node)?
        .nth(index)
        .ok_or("FDT register resource index is missing")?
}

/// 从地址下限 `minimum` 开始，在 `[minimum, limit)` 范围内查找并合并下一段连续的可用物理内存（将相邻或首尾相接的 RAM 段合并为单个连续区间）。
fn next_ram_region(info: &BootInfo, minimum: usize, limit: usize) -> Option<Region> {
    if limit <= minimum {
        return None;
    }
    let memory = &info.memory[..info.memory_count];
    let start = memory
        .iter()
        .filter(|ram| minimum < ram.end && ram.start < limit)
        .map(|ram| ram.start.max(minimum))
        .min()?;
    let mut end = start;

    loop {
        let next = memory
            .iter()
            .filter(|ram| ram.start <= end && start < ram.end)
            .map(|ram| ram.end.min(limit))
            .max()
            .unwrap_or(end);
        if next == end {
            return Some(Region { start, end });
        }
        end = next;
    }
}

/// 计算保留内存区间在碰撞检测时实际占用的物理范围：若启用了 `no-map`（`page_aligned = true`），则向两端扩展至整页（`PAGE_SIZE`）边界，避免同一物理页内混用禁映射区与普通内存。
fn reservation_extent(region: Region, page_aligned: bool) -> Result<Region, &'static str> {
    if !page_aligned {
        return Ok(region);
    }
    let mask = PAGE_SIZE - 1;
    let end = region
        .end
        .checked_add(mask)
        .ok_or("FDT reserved-memory page boundary overflows")?
        & !mask;
    Ok(Region {
        start: region.start & !mask,
        end,
    })
}

/// 将候选物理地址向上对齐到 2 的幂次边界 `alignment`；若加法溢出则返回 `None`。
fn align_reserved_address(address: usize, alignment: usize) -> Option<usize> {
    address
        .checked_add(alignment - 1)
        .map(|value| value & !(alignment - 1))
}

/// 在指定的物理地址窗口 `window` 内搜索一段大小为 `size`、满足 `alignment` 对齐且不与任何已登记保留区或内核镜像重叠的可用 RAM 区间。
///
/// - **调用上下文**：处理 `/reserved-memory` 动态尺寸预留请求（[`discover_dynamic_reserved_memory`]）或 `ktest` 测试。
/// - **参数**：`info` 为已收集 RAM 与静态保留区的启动快照；`size` 为请求字节数；`alignment` 为 2 的幂次对齐字节数；`window` 为允许分配的地址范围；`no_map` 表示该请求是否携带 `no-map` 属性。
/// - **返回值**：找到可用区间时返回 `Ok(Some(region))`，窗口内无足够连续空间时返回 `Ok(None)`，页边界计算溢出时返回 `Err`。
fn find_reserved_region(
    info: &BootInfo,
    size: usize,
    alignment: usize,
    window: Region,
    no_map: bool,
) -> Result<Option<Region>, &'static str> {
    let kernel = crate::boot::kernel_region();
    let mut cursor = window.start.max(PAGE_SIZE);

    while let Some(ram) = next_ram_region(info, cursor, window.end) {
        let Some(mut start) = align_reserved_address(ram.start, alignment) else {
            return Ok(None);
        };

        while let Some(end) = start.checked_add(size).filter(|end| *end <= ram.end) {
            let candidate = Region { start, end };
            let mut next = start;
            for (index, region) in info.reserved[..info.reserved_count].iter().enumerate() {
                let occupied = reservation_extent(*region, no_map || info.reserved_no_map[index])?;
                if candidate.overlaps(occupied) {
                    next = next.max(occupied.end);
                }
            }
            // 内核自身镜像在 discover_tree 返回后才会由调用方正式写入保留表，选址时必须主动避开内核代码、数据与启动栈区间。
            let occupied = reservation_extent(kernel, no_map)?;
            if candidate.overlaps(occupied) {
                next = next.max(occupied.end);
            }

            if next == start {
                return Ok(Some(candidate));
            }
            let Some(aligned) = align_reserved_address(next, alignment) else {
                break;
            };
            start = aligned;
        }
        cursor = ram.end;
    }
    Ok(None)
}

/// 按 `/reserved-memory` 节点规定的字节宽度 `width` 读取动态预留子节点上的 `size` 或 `alignment` 属性。
fn reserved_value(node: Node<'_>, name: &str, width: usize) -> Result<usize, &'static str> {
    let value = property(node, name)?;
    if value.len() != width {
        return Err("FDT dynamic reserved-memory cell width is invalid");
    }
    cells(value)
}

/// 遍历 `/reserved-memory` 下所有仅声明 `size`（未指定静态 `reg`）的动态预留子节点，在可用物理内存中为其分配不冲突的区间并登记到 `info` 保留表中。
///
/// - **调用上下文**：在静态保留区、内核低地址保护区及 `initrd` 全部登记完毕后，由 [`discover_tree`] 末尾调用。
/// - **参数**：`tree` 为设备树；`info` 为待更新的启动信息快照。
/// - **返回值**：所有动态预留请求均成功分配并登记时返回 `Ok(())`；空间不足或属性非法时返回 `Err`。
fn discover_dynamic_reserved_memory(
    tree: &Fdt<'_>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    let Some(reserved) = tree
        .find_node("/reserved-memory")
        .filter(|node| enabled(*node))
    else {
        return Ok(());
    };

    let mut requests = reserved
        .children()
        .filter(|node| enabled(*node) && node.property("reg").is_none());
    let Some(first) = requests.next() else {
        return Ok(());
    };
    let root = tree.find_node("/").ok_or("FDT root is missing")?;
    let address_cells = number(reserved, "#address-cells")?;
    let size_cells = number(reserved, "#size-cells")?;
    if !matches!(address_cells, 1 | 2)
        || !matches!(size_cells, 1 | 2)
        || address_cells != number(root, "#address-cells")?
        || size_cells != number(root, "#size-cells")?
    {
        return Err("FDT reserved-memory cells do not match the root");
    }

    let address_width = address_cells as usize * FDT_CELL_BYTES;
    let size_width = size_cells as usize * FDT_CELL_BYTES;
    for node in core::iter::once(first).chain(requests) {
        let size = reserved_value(node, "size", size_width)?;
        if size == 0 {
            return Err("FDT dynamic reserved-memory size is zero");
        }
        let alignment = if node.property("alignment").is_some() {
            reserved_value(node, "alignment", address_width)?
        } else {
            0
        };
        let alignment = if alignment == 0 {
            DEFAULT_RESERVED_ALIGNMENT
        } else {
            alignment
        };
        if !alignment.is_power_of_two() {
            return Err("FDT dynamic reserved-memory alignment is invalid");
        }
        let no_map = node.property("no-map").is_some();

        let region = if let Some(ranges) = node.property("alloc-ranges") {
            let stride = address_width + size_width;
            if ranges.value.len() % stride != 0 {
                return Err("FDT reserved-memory alloc-ranges is truncated");
            }
            let mut selected = None;
            for range in ranges.value.chunks_exact(stride) {
                let window = Region::from_size(
                    cells(&range[..address_width])?,
                    cells(&range[address_width..])?,
                )?;
                selected = find_reserved_region(info, size, alignment, window, no_map)?;
                if selected.is_some() {
                    break;
                }
            }
            selected
        } else {
            find_reserved_region(
                info,
                size,
                alignment,
                Region {
                    start: 0,
                    end: usize::MAX,
                },
                no_map,
            )?
        }
        .ok_or("FDT dynamic reserved-memory has no available range")?;

        if no_map {
            info.reserve_no_map(region)?;
        } else {
            info.reserve(region)?;
        }
    }
    Ok(())
}

/// 从设备节点的 `interrupts` 属性首 cell 读取中断源编号，并校验属性总字节长度与所属中断控制器的 `#interrupt-cells` 声明一致。
fn irq(node: Node<'_>, controller: Node<'_>) -> Result<u32, &'static str> {
    let value = property(node, "interrupts")?;
    if value.len() != number(controller, "#interrupt-cells")? as usize * FDT_CELL_BYTES {
        return Err("FDT UART or root interrupt size does not match its binding");
    }
    word(value, 0)
}

/// 解析完整 DTB 字节流，将物理内存、静态与动态保留区、CPU 拓扑、时钟频率、启动参数、控制台与外设资源就地填入 `info`。
///
/// 直接通过可变引用 `&mut BootInfo` 填充目标槽位，避免在容量受限的早期启动栈上按值返回或复制大型结构体。
///
/// - **调用上下文**：由 [`discover`]（早期启动）或 [`ktest_discover_tree`]（测试）调用。
/// - **参数**：`bytes` 为完整 DTB 字节切片；`info` 为独占的启动信息快照槽位；`dtb_address` 为 DTB 在物理内存中的起始地址。
/// - **返回值**：全部核心资源发现与校验通过时返回 `Ok(())`，否则返回静态错误描述。
fn discover_tree(
    bytes: &[u8],
    info: &mut BootInfo,
    dtb_address: usize,
) -> Result<(), &'static str> {
    // Step 1: 执行 FDT 二进制结构预校验，并登记 DTB 自身、FDT 头部内存保留块及 /reserved-memory 下的静态保留区。
    validate(bytes)?;
    let tree = Fdt::new(bytes).map_err(|_| "FDT parser rejected the input")?;

    info.dtb = Region::from_size(dtb_address, tree.total_size())?;
    info.reserve(info.dtb)?;
    #[cfg(target_arch = "loongarch64")]
    if current() == Platform::QemuVirt {
        info.reserve(Region::from_size(0, QEMU_LOONGARCH_DTB_PHYS)?)?;
    }

    for reservation in tree.memory_reservations() {
        info.reserve(Region::from_size(
            reservation.address() as usize,
            reservation.size(),
        )?)?;
    }

    if let Some(reserved) = tree
        .find_node("/reserved-memory")
        .filter(|node| enabled(*node))
    {
        if reserved
            .property("ranges")
            .is_some_and(|p| !p.value.is_empty())
        {
            return Err("FDT reserved-memory is not an identity address space");
        }
        for child in reserved.children().filter(|n| enabled(*n)) {
            identity_address(&tree, child)?;
            if child.property("reg").is_none() {
                if child.property("size").is_none() {
                    return Err("FDT reserved-memory node has neither reg nor size");
                }
                continue;
            }
            let no_map = child.property("no-map").is_some();
            for region in regions(child)? {
                if no_map {
                    info.reserve_no_map(region?)?;
                } else {
                    info.reserve(region?)?;
                }
            }
        }
    }

    // Step 2: 收集所有启用的物理内存（memory）节点区间，校验互不重叠且完整包含运行时 DTB；在真实开发板上额外保留内核装载地址之前的固件低地址 RAM。
    for node in tree.all_nodes().filter(|n| enabled(*n)) {
        if node.property("device_type").and_then(|p| p.as_str()) != Some("memory") {
            continue;
        }
        identity_address(&tree, node)?;
        for region in regions(node)? {
            if info.memory_count == MAX_MEMORY_REGIONS {
                return Err("FDT RAM range capacity exceeded");
            }
            let region = region?;
            if info.memory[..info.memory_count]
                .iter()
                .any(|old| old.overlaps(region))
            {
                return Err("FDT RAM ranges overlap");
            }
            info.memory[info.memory_count] = region;
            info.memory_count += 1;
        }
    }

    if info.memory_count == 0 {
        return Err("FDT has no RAM resources");
    }
    if !info.memory[..info.memory_count]
        .iter()
        .any(|r| r.start <= info.dtb.start && info.dtb.end <= r.end)
    {
        return Err("runtime DTB is outside the described RAM");
    }

    if info.platform != Platform::QemuVirt {
        let kernel = crate::boot::kernel_region();
        let ram = info.memory[..info.memory_count]
            .iter()
            .find(|ram| ram.start <= kernel.start && kernel.end <= ram.end)
            .ok_or("kernel load region is outside actual RAM")?;
        // 真实开发板的固件（如 OpenSBI、PMON 或 UEFI 驻留服务及交接表）位于同一低地址 RAM 段且在内核装载地址之前，必须在物理页帧分配器建立前标记为保留以防覆盖。
        if ram.start < kernel.start {
            info.reserve(Region {
                start: ram.start,
                end: kernel.start,
            })?;
        }
    }

    // Step 3: 扫描 /cpus 节点收集所有启用的应用处理器硬件 ID（在 VisionFive 2 上跳过无 MMU 的 S7 监控核），并将当前引导核（BSP）交换至逻辑 CPU 0 号槽位，随后记录硬件计时器频率。
    let cpus = tree.find_node("/cpus").ok_or("FDT cpus node is missing")?;
    if !matches!(number(cpus, "#address-cells")?, 1 | 2) || number(cpus, "#size-cells")? != 0 {
        return Err("FDT CPU reg cell format is invalid");
    }
    for node in cpus.children().filter(|n| enabled(*n)) {
        if node.property("device_type").and_then(|p| p.as_str()) != Some("cpu") {
            continue;
        }

        let reg = property(node, "reg")?;
        if reg.len() != number(cpus, "#address-cells")? as usize * FDT_CELL_BYTES {
            return Err("FDT CPU reg is empty, truncated, or contains multiple IDs");
        }
        let id = cells(reg)?;
        if current() == Platform::VisionFive2 && compatible(node, "sifive,s7") {
            continue;
        }
        if info.hardware_ids[..info.cpu_count].contains(&id) {
            return Err("FDT CPU hardware IDs are duplicated");
        }
        if info.cpu_count == MAX_CPUS {
            return Err("FDT CPU count exceeds the compiled capacity");
        }

        info.hardware_ids[info.cpu_count] = id;
        info.cpu_count += 1;
    }

    let bsp_index = info.hardware_ids[..info.cpu_count]
        .iter()
        .position(|id| *id == info.bsp_hardware_id)
        .ok_or("FDT enabled CPUs do not include the BSP")?;
    info.hardware_ids.swap(0, bsp_index);

    #[cfg(target_arch = "riscv64")]
    {
        info.counter_frequency = number(cpus, "timebase-frequency")
            .map_err(|_| "FDT cpus timebase-frequency is missing or invalid")?
            as u64;
    }
    #[cfg(target_arch = "loongarch64")]
    {
        info.counter_frequency = crate::arch::boot_counter_frequency();
    }
    if info.counter_frequency == 0 {
        return Err("hardware counter frequency is zero");
    }

    // Step 4: 解析 /chosen 节点的启动命令行（bootargs）、初始内存盘（initrd）与主控制台 UART，随后分发至对应平台的专属外设发现函数，最后为动态保留内存请求分配物理区间。
    let chosen = tree
        .find_node("/chosen")
        .ok_or("FDT chosen node is missing")?;
    discover_bootargs(chosen, info)?;
    discover_initrd(chosen, info)?;

    let uart = resources::stdout_uart(&tree, chosen)?;
    info.interrupts.uart = single_region(&tree, uart)?;
    if info.interrupts.uart.start != EARLY_UART_BASE {
        return Err("FDT chosen UART differs from the early UART");
    }
    resources::discover_uart(&tree, chosen, uart, info)?;

    match current() {
        Platform::QemuVirt => discover_qemu(&tree, cpus, uart, info)?,
        Platform::VisionFive2 => visionfive2::discover(&tree, cpus, uart, info)?,
        Platform::Ls2k1000la => ls2k1000la::discover(&tree, uart, info)?,
    }

    discover_dynamic_reserved_memory(&tree, info)?;
    Ok(())
}

/// 测试辅助入口：对给定的 DTB 字节切片执行 FDT 二进制结构与格式校验（[`validate`]）。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_validate(bytes: &[u8]) -> Result<(), &'static str> {
    validate(bytes)
}

/// 测试辅助入口：在调用方提供的栈/堆槽位 `output` 上初始化 `BootInfo`，并对给定的 DTB 字节切片执行完整的设备树资源发现（[`discover_tree`]）。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_discover_tree<'a>(
    bytes: &[u8],
    raw_args: [usize; 4],
    bsp: usize,
    dtb_address: usize,
    output: &'a mut core::mem::MaybeUninit<BootInfo>,
) -> Result<&'a mut BootInfo, &'static str> {
    // SAFETY:
    // 1. `output` 为调用方传入的独占可变引用（`&'a mut MaybeUninit<BootInfo>`），保证指针非空、对齐合法且无并发别名；
    // 2. `BootInfo::initialize_at` 对结构体的全部字段完成逐字段写入初始化，随后 `assume_init_mut()` 才构造可变引用；且 `BootInfo` 所有字段均为 Copy 基础类型/定长数组，不含需要 Drop 的资源。
    let info = unsafe {
        BootInfo::initialize_at(output.as_mut_ptr(), raw_args, bsp);
        output.assume_init_mut()
    };
    discover_tree(bytes, info, dtb_address)?;
    Ok(info)
}

/// 测试辅助入口：校验 DTB 字节流并解析指定节点路径 `path` 的 DMA 缓存一致性属性与父链（[`resources::coherent`]）。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_dma_coherent(
    bytes: &[u8],
    path: &str,
    default_coherent: bool,
) -> Result<bool, &'static str> {
    validate(bytes)?;
    let tree = Fdt::new(bytes).map_err(|_| "FDT parse failed")?;
    let node = tree.find_node(path).ok_or("FDT DMA test node is missing")?;
    resources::coherent(&tree, node, default_coherent)
}

/// 测试辅助入口：在给定的只读 `BootInfo` 快照上测试动态保留内存选址算法（[`find_reserved_region`]）。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_find_reserved_region(
    info: &BootInfo,
    size: usize,
    alignment: usize,
    window: Region,
    no_map: bool,
) -> Result<Option<Region>, &'static str> {
    find_reserved_region(info, size, alignment, window, no_map)
}

/// 从 `/chosen` 节点读取可选的 `bootargs` 属性，验证其为单个以 NUL 结尾且不含内部 NUL 字节的非空字符串后，复制到 `info.bootargs` 缓冲区。
fn discover_bootargs(chosen: Node<'_>, info: &mut BootInfo) -> Result<(), &'static str> {
    let Some(property) = chosen.property("bootargs") else {
        return Ok(());
    };
    let value = property.value;
    if value.is_empty()
        || info.bootargs.len() < value.len()
        || value.last() != Some(&0)
        || value[..value.len() - 1].contains(&0)
    {
        return Err("FDT bootargs is not a bounded NUL-terminated command line");
    }

    info.bootargs[..value.len()].copy_from_slice(value);
    info.bootargs_len = value.len() - 1;
    Ok(())
}

/// 从 LoongArch 固件传递的物理地址复制启动命令行字符串到 `info.bootargs`。
///
/// 在真实开发板（`append = true`）上，若该命令行非空且尚未作为完整子串出现在已有 `bootargs` 中，则以空格分隔追加在末尾；在 QEMU（`append = false`）上则直接覆盖替换。
///
/// # Safety
/// 1. `address` 对应的物理内存必须已在当前地址空间建立线性直映（可通过 `crate::arch::phys_to_virt` 直接读取）；
/// 2. 从 `address` 起直至首个 NUL（`\0`）终止符（或最多 `info.bootargs.len()` 字节）的连续内存区间必须可读、在复制期间不被并发修改，且与 `info` 所在内存不重叠。
#[cfg(target_arch = "loongarch64")]
unsafe fn discover_command_line(
    info: &mut BootInfo,
    address: usize,
    append: bool,
) -> Result<(), &'static str> {
    let pointer = crate::arch::phys_to_virt(address) as *const u8;
    if address.checked_add(info.bootargs.len()).is_none()
        || (pointer as usize)
            .checked_add(info.bootargs.len())
            .is_none()
    {
        return Err("firmware command line address overflows");
    }

    let mut length = 0;
    loop {
        if length == info.bootargs.len() {
            return Err("firmware command line exceeds boot snapshot capacity");
        }
        // SAFETY:
        // 1. 前置检查已确认 `length < info.bootargs.len()` 且 `pointer + info.bootargs.len()` 不会发生地址溢出；
        // 2. 调用方通过 `# Safety` 契约保证从 `pointer` 起至首个 NUL 或容量上限内的字节在当前线性映射下可读且无并发修改。
        if unsafe { pointer.add(length).read() } == 0 {
            break;
        }
        length += 1;
    }

    // SAFETY:
    // 1. 上方循环已逐字节验证 `[pointer, pointer + length)` 范围内全部可读且不含越界偏移；
    // 2. 调用方契约保证该固件命令行存储在此函数返回前保持有效、不变，且与 `info` 无内存别名重叠。
    let command = unsafe { core::slice::from_raw_parts(pointer, length) };
    if append
        && (command.is_empty()
            || info
                .bootargs()
                .windows(command.len())
                .any(|window| window == command))
    {
        return Ok(());
    }
    let start = if append { info.bootargs_len } else { 0 };
    let separator = usize::from(append && start != 0);
    let end = start + separator + command.len();
    if info.bootargs.len() <= end {
        return Err("firmware command line exceeds boot snapshot capacity");
    }

    if separator != 0 {
        info.bootargs[start] = b' ';
    }
    info.bootargs[start + separator..end].copy_from_slice(command);
    info.bootargs[end] = 0;
    info.bootargs_len = end;
    Ok(())
}

/// 解析 `/chosen` 节点的 `linux,initrd-start` 与 `linux,initrd-end` 属性对，验证初始内存盘（initrd）完整落在可用 RAM 内且不与 DTB 或内核镜像重叠后，将其登记为保留内存并记录到 `info.initrd`。
fn discover_initrd(chosen: Node<'_>, info: &mut BootInfo) -> Result<(), &'static str> {
    let start = chosen.property("linux,initrd-start");
    let end = chosen.property("linux,initrd-end");
    let (start, end) = match (start, end) {
        (None, None) => return Ok(()),
        (Some(start), Some(end)) => (cells(start.value)?, cells(end.value)?),
        _ => return Err("FDT initrd requires both start and end"),
    };
    if start == 0 || end <= start {
        return Err("FDT initrd range is invalid");
    }
    let region = Region { start, end };

    if !info.memory[..info.memory_count]
        .iter()
        .any(|ram| ram.start <= start && end <= ram.end)
        || region.overlaps(info.dtb)
        || region.overlaps(crate::boot::kernel_region())
    {
        return Err("FDT initrd is outside RAM or overlaps boot data");
    }

    info.reserve(region)?;
    info.initrd = Some(region);
    Ok(())
}

/// 发现 QEMU `virt` 虚拟机平台（RISC-V 64 与 LoongArch64）的中断控制器拓扑、PCI ECAM 主桥、VirtIO-MMIO 设备槽及 RTC 实时时钟。
///
/// - **调用上下文**：当编译平台为 [`Platform::QemuVirt`] 时，由 [`discover_tree`] 调用。
/// - **参数**：`tree` 为设备树；`cpus` 为 `/cpus` 节点；`uart` 为主控制台节点；`info` 为待填充的启动信息快照。
/// - **返回值**：所有平台控制器与外设资源校验通过时返回 `Ok(())`。
fn discover_qemu(
    tree: &Fdt<'_>,
    cpus: Node<'_>,
    uart: Node<'_>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    // Step 1: 沿 UART 的中断父链定位外部中断控制器（RISC-V 为 PLIC，LoongArch64 为 PCH-PIC -> EIOINTC -> CPU INTC 三级级联），校验绑定并记录寄存器窗口与中断号。
    let controller = interrupt_parent(tree, uart)?;
    info.interrupts.uart_source = irq(uart, controller)?;
    info.interrupts.controller = single_region(tree, controller)?;
    #[cfg(target_arch = "riscv64")]
    discover_plic(tree, cpus, controller, info)?;
    #[cfg(target_arch = "loongarch64")]
    {
        let _ = cpus;
        require_controller(controller, "loongson,pch-pic-1.0", 2)?;
        info.interrupts.source_count = MAX_INTERRUPT_SOURCES as u32;
        if word(property(uart, "interrupts")?, FDT_CELL_BYTES)? != IRQ_TYPE_LEVEL_HIGH
            || MAX_INTERRUPT_SOURCES as u32 <= info.interrupts.uart_source
        {
            return Err("FDT UART is not a supported PCH-PIC level-high source");
        }
        let extioi = interrupt_parent(tree, controller)?;
        require_controller(extioi, "loongson,ls2k2000-eiointc", 1)?;
        info.interrupts.parent_controller = single_region(tree, extioi)?;
        let root = interrupt_parent(tree, extioi)?;
        require_controller(root, "loongson,cpu-interrupt-controller", 1)?;
        info.interrupts.root_interrupt = irq(extioi, root)?;
        if 8 <= info.interrupts.root_interrupt {
            return Err("FDT EXTIOI CPU root interrupt exceeds the supported range");
        }
    }

    // Step 2: 依次发现 PCI ECAM 主桥、VirtIO-MMIO 设备槽位（RISC-V）以及平台实时时钟（RTC）。
    discover_pci(tree, controller, info)?;
    discover_virtio_mmio(tree, controller, info)?;
    discover_rtc(tree, info)
}

/// 校验新增外设的寄存器或地址窗口 `region` 不与任何物理 RAM 段、中断控制器、串口、RTC、PCI 窗口、VirtIO-MMIO 槽位或板级外设寄存器重叠。
fn device_region(region: Region, info: &BootInfo) -> Result<(), &'static str> {
    if info.memory[..info.memory_count]
        .iter()
        .any(|old| old.overlaps(region))
        || info.interrupts.uart.overlaps(region)
        || info.interrupts.controller.overlaps(region)
        || info.rtc.overlaps(region)
        || info.pci.configuration.overlaps(region)
        || info.pci.configuration_extended.overlaps(region)
        || info.board.ccache.overlaps(region)
        || info.board.pmc.overlaps(region)
        || info
            .board
            .mmc
            .is_some_and(|device| device.registers.overlaps(region))
        || info
            .board
            .ahci
            .is_some_and(|device| device.registers.overlaps(region))
        || info
            .board
            .gmac
            .is_some_and(|device| device.registers.overlaps(region))
        || info.pci.ranges[..info.pci.range_count]
            .iter()
            .any(|old| old.cpu.overlaps(region))
        || info.virtio_mmio[..info.virtio_mmio_count]
            .iter()
            .any(|old| old.registers.overlaps(region))
    {
        return Err("FDT device resource overlaps RAM or another device window");
    }
    Ok(())
}

/// 在设备树中查找唯一启用的平台实时时钟（RISC-V QEMU 为 `google,goldfish-rtc`，LoongArch64 为 `loongson,ls7a-rtc`），校验其寄存器窗口对齐与大小后记录到 `info.rtc`。
fn discover_rtc(tree: &Fdt<'_>, info: &mut BootInfo) -> Result<(), &'static str> {
    #[cfg(target_arch = "riscv64")]
    let (binding, alignment, size) = (
        "google,goldfish-rtc",
        GOLDFISH_RTC_WINDOW_BYTES,
        GOLDFISH_RTC_WINDOW_BYTES,
    );
    #[cfg(target_arch = "loongarch64")]
    let (binding, alignment, size) = (
        "loongson,ls7a-rtc",
        LS7A_RTC_WINDOW_BYTES,
        LS7A_RTC_WINDOW_BYTES,
    );

    let mut devices = tree
        .all_nodes()
        .filter(|node| enabled(*node) && compatible(*node, binding));
    let node = devices.next().ok_or("FDT enabled RTC is missing")?;
    if devices.next().is_some() {
        return Err("FDT has multiple enabled RTC devices");
    }

    let registers = single_region(tree, node)?;
    if registers.start % alignment != 0 || registers.end - registers.start != size {
        return Err("FDT RTC register window has invalid size or alignment");
    }
    device_region(registers, info)?;
    info.rtc = registers;
    Ok(())
}

/// 校验外部设备中断源编号 `source` 是否在当前中断控制器容量范围内（在 RISC-V 平台上同时拒绝保留的 PLIC 0 号中断源）。
fn device_source(source: u32, info: &BootInfo) -> Result<(), &'static str> {
    #[cfg(target_arch = "riscv64")]
    if source == 0 {
        return Err("FDT device interrupt uses reserved PLIC source zero");
    }
    if info.interrupts.source_count <= source {
        return Err("FDT device interrupt source exceeds the controller capacity");
    }
    Ok(())
}

/// 扫描设备树中所有启用且兼容 `virtio,mmio` 的节点，校验其 PLIC 中断路由与 4 KiB 对齐寄存器窗口，按物理基地址升序存入 `info.virtio_mmio`。
fn discover_virtio_mmio(
    tree: &Fdt<'_>,
    controller: Node<'_>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    for node in tree
        .all_nodes()
        .filter(|n| enabled(*n) && compatible(*n, "virtio,mmio"))
    {
        if info.virtio_mmio_count == MAX_VIRTIO_MMIO {
            return Err("FDT VirtIO-MMIO slot capacity exceeded");
        }

        require_controller(controller, "riscv,plic0", 1)?;
        let parent = interrupt_parent(tree, node)?;
        if parent.name.as_ptr() != controller.name.as_ptr() {
            return Err("FDT VirtIO-MMIO interrupt parent is not the selected PLIC");
        }

        let registers = single_region(tree, node)?;
        if registers.start % VIRTIO_MMIO_WINDOW_BYTES != 0
            || registers.end - registers.start != VIRTIO_MMIO_WINDOW_BYTES
        {
            return Err("FDT VirtIO-MMIO register window is not one aligned page");
        }
        device_region(registers, info)?;

        let interrupt_source = irq(node, parent)?;
        device_source(interrupt_source, info)?;

        info.virtio_mmio[info.virtio_mmio_count] = VirtioMmioResource {
            registers,
            interrupt_source,
        };
        info.virtio_mmio_count += 1;
    }

    info.virtio_mmio[..info.virtio_mmio_count]
        .sort_unstable_by_key(|resource| resource.registers.start);
    Ok(())
}

/// 发现通用 PCIe ECAM 主桥（`pci-host-ecam-generic`），记录配置空间窗口、总线号范围（`bus-range`）、I/O 与内存地址映射窗口（`ranges`）以及 INTx 中断路由表。
///
/// 此阶段仅登记主桥资源与地址窗口，不主动枚举 PCI 设备或访问配置空间。
fn discover_pci(
    tree: &Fdt<'_>,
    controller: Node<'_>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    // Step 1: 查找唯一启用的 pci-host-ecam-generic 节点，读取并校验其 ECAM 配置空间寄存器区间及 #address-cells / #size-cells 格式。
    let mut hosts = tree
        .all_nodes()
        .filter(|n| enabled(*n) && compatible(*n, "pci-host-ecam-generic"));
    let host = hosts.next().ok_or("FDT PCI ECAM host is missing")?;
    if hosts.next().is_some() {
        return Err("FDT has multiple unsupported PCI hosts");
    }

    let configuration = single_region(tree, host)?;
    device_region(configuration, info)?;
    info.pci.configuration = configuration;
    info.pci.kind = PciConfiguration::Ecam;

    if number(host, "#address-cells")? != 3 || number(host, "#size-cells")? != 2 {
        return Err("FDT PCI ranges cell format is invalid");
    }
    let parent = parent(tree, host).ok_or("FDT PCI host has no parent bus")?;
    if parent.cell_sizes().address_cells != 2 {
        return Err("FDT PCI physical addresses are not two cells");
    }

    // Step 2: 解析 bus-range 属性，确认总线号在 0..=255 范围内，且 ECAM 寄存器窗口大小与总线数量（每条总线 1 MiB）及对齐严格匹配。
    let buses = property(host, "bus-range")?;
    if buses.len() != 2 * FDT_CELL_BYTES {
        return Err("FDT PCI bus-range is truncated");
    }
    info.pci.bus_start = word(buses, 0)?;
    info.pci.bus_end = word(buses, FDT_CELL_BYTES)?;
    if info.pci.bus_end < info.pci.bus_start || (u8::MAX as u32) < info.pci.bus_end {
        return Err("FDT PCI bus-range is invalid");
    }

    let configuration_size =
        ((info.pci.bus_end - info.pci.bus_start + 1) as usize) << PCI_ECAM_BUS_SHIFT;
    if configuration.start % (1 << PCI_ECAM_BUS_SHIFT) != 0
        || configuration.end - configuration.start != configuration_size
    {
        return Err("FDT PCI ECAM window does not match the declared bus range");
    }

    // Step 3: 遍历 ranges 属性，提取 I/O、32 位内存与 64 位内存空间映射窗口，校验 CPU 物理区间无冲突且同一 PCI 地址空间内子窗口互不重叠。
    let ranges = property(host, "ranges")?;
    if ranges.is_empty() || ranges.len() % PCI_RANGE_BYTES != 0 {
        return Err("FDT PCI ranges is truncated");
    }
    for range in ranges.chunks_exact(PCI_RANGE_BYTES) {
        if info.pci.range_count == info.pci.ranges.len() {
            return Err("FDT PCI window capacity exceeded");
        }
        let flags = word(range, 0)?;
        if !matches!(
            flags & PCI_SPACE_CODE_MASK,
            PCI_SPACE_IO | PCI_SPACE_MEM32 | PCI_SPACE_MEM64
        ) {
            return Err("FDT PCI window is neither IO nor memory");
        }
        let bus_start = cells(&range[PCI_RANGE_BUS_OFFSET..PCI_RANGE_CPU_OFFSET])?;
        let cpu = Region::from_size(
            cells(&range[PCI_RANGE_CPU_OFFSET..PCI_RANGE_SIZE_OFFSET])?,
            cells(&range[PCI_RANGE_SIZE_OFFSET..PCI_RANGE_BYTES])?,
        )?;
        let bus = Region::from_size(bus_start, cpu.end - cpu.start)?;
        device_region(cpu, info)?;

        for old in &info.pci.ranges[..info.pci.range_count] {
            let old_io = old.flags & PCI_SPACE_CODE_MASK == PCI_SPACE_IO;
            let new_io = flags & PCI_SPACE_CODE_MASK == PCI_SPACE_IO;
            let old_bus = Region::from_size(old.bus_start, old.cpu.end - old.cpu.start)?;
            if old_io == new_io && old_bus.overlaps(bus) {
                return Err("FDT PCI bus windows overlap in the same address space");
            }
        }

        info.pci.ranges[info.pci.range_count] = PciRange {
            flags,
            bus_start,
            cpu,
        };
        info.pci.range_count += 1;
    }

    // Step 4: 解析 PCI INTx 传统中断路由表（interrupt-map 与 interrupt-map-mask）。
    discover_pci_intx(tree, host, controller, info)?;
    Ok(())
}

/// 解析 PCI 主桥的 `interrupt-map-mask` 与 `interrupt-map` 属性，验证并提取 INTx 引脚到外部中断控制器源的路由表。
fn discover_pci_intx(
    tree: &Fdt<'_>,
    host: Node<'_>,
    controller: Node<'_>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    // Step 1: 校验主桥的 #interrupt-cells（若声明则必须为 1，代表单个 INTx 引脚号）与 4 cell 的 interrupt-map-mask。
    // RISC-V 与 LoongArch QEMU 主桥均以 3 个 PCI 地址 cell 加 1 个 INTx 引脚 cell 构成匹配键。
    if host.property("#interrupt-cells").is_some() && number(host, "#interrupt-cells")? != 1 {
        return Err("FDT PCI host interrupt cells do not describe one INTx pin");
    }

    let mask = property(host, "interrupt-map-mask")?;
    if mask.len() != PCI_INTX_PARENT_OFFSET {
        return Err("FDT PCI interrupt-map-mask is not four cells");
    }
    for i in 0..info.pci.intx_mask.len() {
        info.pci.intx_mask[i] = word(mask, i * FDT_CELL_BYTES)?;
    }
    if info.pci.intx_mask != QEMU_PCI_INTX_MASK {
        return Err("FDT PCI interrupt-map-mask differs from the selected host profile");
    }

    // Step 2: 根据父中断控制器的 #interrupt-cells 确定单条路由记录的字节跨度（stride）：
    // 若父控制器声明 1 个 cell（如 PLIC）或旧版固件省略了触发标志，则每条记录只含中断源编号（24 字节）；
    // 若父控制器声明 2 个 cell（如 PCH-PIC 新版固件），则每条记录末尾追加高电平触发标志（28 字节）。
    let map = property(host, "interrupt-map")?;
    if map.is_empty() {
        return Err("FDT PCI interrupt-map is empty");
    }
    let parent = tree
        .find_phandle(word(map, PCI_INTX_PARENT_OFFSET)?)
        .ok_or("FDT PCI interrupt-map parent phandle has no controller")?;
    if parent.name.as_ptr() != controller.name.as_ptr() {
        return Err("FDT PCI interrupt-map parent is not the selected controller");
    }
    let declared =
        PCI_INTX_SOURCE_OFFSET + FDT_CELL_BYTES * number(parent, "#interrupt-cells")? as usize;
    let stride = if map.len() % declared == 0 {
        declared
    } else if map.len() % PCI_INTX_LEGACY_BYTES == 0 {
        PCI_INTX_LEGACY_BYTES
    } else {
        return Err("FDT PCI interrupt-map is not a complete route table");
    };

    // Step 3: 逐条解析路由记录，验证 INTx 引脚号（1..=4）、父控制器一致性、高电平触发方式及掩码后路由键的唯一性，存入 info.pci.intx_routes。
    for entry in map.chunks_exact(stride) {
        if info.pci.intx_route_count == info.pci.intx_routes.len() {
            return Err("FDT PCI INTx route capacity exceeded");
        }
        let child = [
            word(entry, 0)?,
            word(entry, FDT_CELL_BYTES)?,
            word(entry, 2 * FDT_CELL_BYTES)?,
            word(entry, 3 * FDT_CELL_BYTES)?,
        ];
        if !(1..=u32::from(PCI_INTX_PIN_COUNT)).contains(&child[3]) {
            return Err("FDT PCI interrupt-map contains an invalid INTx pin");
        }

        let parent = tree
            .find_phandle(word(entry, PCI_INTX_PARENT_OFFSET)?)
            .ok_or("FDT PCI interrupt-map parent phandle has no controller")?;
        if parent.name.as_ptr() != controller.name.as_ptr() {
            return Err("FDT PCI interrupt-map parent is not the selected controller");
        }
        if PCI_INTX_LEGACY_BYTES < stride
            && word(entry, PCI_INTX_LEGACY_BYTES)? != IRQ_TYPE_LEVEL_HIGH
        {
            return Err("FDT PCI INTx route is not level-high");
        }
        let source = word(entry, PCI_INTX_SOURCE_OFFSET)?;
        device_source(source, info)?;

        if info.pci.intx_routes[..info.pci.intx_route_count]
            .iter()
            .any(|old| {
                (0..child.len()).all(|i| ((old.child[i] ^ child[i]) & info.pci.intx_mask[i]) == 0)
            })
        {
            return Err("FDT PCI interrupt-map contains duplicate masked keys");
        }

        info.pci.intx_routes[info.pci.intx_route_count] = PciIntxRoute { child, source };
        info.pci.intx_route_count += 1;
    }
    Ok(())
}

/// 解析 RISC-V 平台级中断控制器（PLIC，`riscv,plic0` 或 `sifive,plic-1.0.0`）的中断源数量、各启用 CPU 的 S-mode（Supervisor）上下文槽位及寄存器窗口边界。
///
/// - **调用上下文**：RISC-V 架构下由 [`discover_qemu`] 或 VisionFive 2 板级发现逻辑调用。
/// - **参数**：`tree` 为设备树；`cpus` 为 `/cpus` 节点；`plic` 为 PLIC 节点；`info` 为待更新的启动信息快照。
/// - **返回值**：成功时填充 `info.interrupts` 的源数量、每个逻辑核的 `plic_contexts` 及根中断号并返回 `Ok(())`。
#[cfg(target_arch = "riscv64")]
fn discover_plic(
    tree: &Fdt<'_>,
    cpus: Node<'_>,
    plic: Node<'_>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
    // Step 1: 校验 PLIC 驱动绑定、中断源总数（riscv,ndev）以及寄存器基地址的 4 字节对齐。
    let binding = if current() == Platform::VisionFive2 {
        "sifive,plic-1.0.0"
    } else {
        "riscv,plic0"
    };
    require_controller(plic, binding, 1)?;

    let sources = number(plic, "riscv,ndev")?;
    if info.interrupts.uart_source == 0 || sources < info.interrupts.uart_source {
        return Err("FDT UART source exceeds the PLIC capacity");
    }
    if MAX_INTERRUPT_SOURCES as u32 <= sources
        || info.interrupts.controller.start % core::mem::size_of::<u32>() != 0
    {
        return Err("FDT PLIC register layout is unsupported");
    }
    info.interrupts.source_count = sources + 1;

    // Step 2: 遍历所有已启用的应用 CPU 节点，收集每个逻辑 CPU 对应的本地中断控制器（riscv,cpu-intc）phandle 并校验唯一性。
    let mut handles = [0; MAX_CPUS];
    for cpu in cpus.children().filter(|n| enabled(*n)) {
        if cpu.property("device_type").and_then(|p| p.as_str()) != Some("cpu") {
            continue;
        }
        let id = cells(property(cpu, "reg")?)?;
        if current() == Platform::VisionFive2 && compatible(cpu, "sifive,s7") {
            continue;
        }
        let logical = info.hardware_ids[..info.cpu_count]
            .iter()
            .position(|old| *old == id)
            .ok_or("PLIC CPU is not in the enabled CPU table")?;
        let intc = cpu
            .children()
            .find(|n| compatible(*n, "riscv,cpu-intc"))
            .ok_or("FDT CPU local interrupt controller is missing")?;
        require_controller(intc, "riscv,cpu-intc", 1)?;
        handles[logical] = number(intc, "phandle")?;
        if handles[logical] == 0
            || handles
                .iter()
                .enumerate()
                .any(|(i, value)| i != logical && *value == handles[logical])
        {
            return Err("FDT CPU interrupt phandle is zero or duplicated");
        }
    }

    // Step 3: 扫描 PLIC 的 interrupts-extended 属性，定位每个逻辑 CPU 对应的 S-mode 外部中断（IRQ_S_EXT）上下文索引。
    let extended = property(plic, "interrupts-extended")
        .map_err(|_| "FDT PLIC interrupts-extended is missing")?;
    if extended.is_empty() || extended.len() % (2 * FDT_CELL_BYTES) != 0 {
        return Err("FDT PLIC context description is truncated");
    }
    let mut found = [false; MAX_CPUS];
    for (context, entry) in extended.chunks_exact(2 * FDT_CELL_BYTES).enumerate() {
        let handle = word(entry, 0)?;
        let cause = word(entry, FDT_CELL_BYTES)?;
        // OpenSBI 等固件会将 M-mode 上下文条目的中断原因（cause）标记为 u32::MAX（-1）；该条目在硬件上仍占据一个 PLIC context 编号，因此跳过但保持枚举索引递增。
        if cause == u32::MAX {
            continue;
        }
        let intc = tree
            .find_phandle(handle)
            .ok_or("PLIC context references a missing CPU controller")?;
        require_controller(intc, "riscv,cpu-intc", 1)?;
        if cause != crate::arch::IRQ_S_EXT as u32 {
            continue;
        }
        if let Some(logical) = handles[..info.cpu_count]
            .iter()
            .position(|old| *old == handle)
        {
            if found[logical] {
                return Err("FDT CPU has duplicate supervisor PLIC contexts");
            }
            info.interrupts.plic_contexts[logical] = context;
            found[logical] = true;
        }
    }

    // Step 4: 确认所有启用 CPU 均已找到唯一的 S-mode PLIC 上下文，并验证每个上下文的 Enable 与 Claim/Complete 寄存器偏移均未超出 PLIC 寄存器物理窗口。
    if found[..info.cpu_count].iter().any(|present| !present) {
        return Err("FDT PLIC has no supervisor context for an enabled CPU");
    }

    let size = info.interrupts.controller.end - info.interrupts.controller.start;
    for context in &info.interrupts.plic_contexts[..info.cpu_count] {
        let claim_end = crate::arch::PLIC_CONTEXT_BASE
            + context * crate::arch::PLIC_CONTEXT_STRIDE
            + crate::arch::PLIC_CLAIM_OFFSET
            + core::mem::size_of::<u32>();
        let enable_end = crate::arch::PLIC_ENABLE_BASE
            + context * crate::arch::PLIC_ENABLE_STRIDE
            + (sources as usize / u32::BITS as usize + 1) * core::mem::size_of::<u32>();
        if size < claim_end.max(enable_end) {
            return Err("FDT PLIC context exceeds its register window");
        }
    }

    info.interrupts.root_interrupt = crate::arch::IRQ_S_EXT as u32;
    Ok(())
}

/// 从架构启动入口传入的固件交接参数中定位并校验设备树（DTB）与启动命令行，在全局静态 [`BootInfo`] 槽位中就地完成一次性资源发现。
///
/// 直接填充全局静态槽位并返回其可变引用，避免在容量受限的早期引导栈上构造或拷贝大型 `BootInfo` 结构体。
///
/// # Safety
/// 1. **执行上下文与唯一性**：必须且只能由引导核（BSP）在本地中断已关闭、从核（AP）尚未唤醒、且全局 `BootInfo` 槽位尚未被借用或发布的启动早期调用一次。
/// 2. **地址空间映射**：调用前 BSP 必须已建立覆盖固件 DTB、固件系统表（若适用）及启动命令行所在物理内存的线性直映（可通过 `crate::arch::phys_to_virt` 访问）。
/// 3. **DTB 存储有效性**：固件提供的 DTB 物理地址所指向的内存必须至少包含可读的 40 字节 FDT 头部，且头部 `totalsize` 字段声明的完整字节区间在本次调用返回前保持有效、只读不变，并与全局 `BootInfo` 槽位不重叠。
/// 4. **板级交接表与命令行有效性（LoongArch64）**：
///    - 在龙芯 2K1000LA 开发板上，`raw_args` 必须符合 UHI 引导协议、显式 `a3` DTB 传递或 EFI64 系统表交接协议；EFI 系统表头部及其 `nr_tables` 个配置表项在参数解析完成前必须可读且不变；
///    - 固件提供的启动命令行指针（或 QEMU LoongArch64 下 `a1` 传入的物理地址，含物理地址 0）直至首个 NUL 终止符或 `bootargs` 容量上限的字节区间，在复制完成前必须可读、不变且与 `BootInfo` 槽位不重叠。
pub unsafe fn discover(
    raw_args: [usize; 4],
    bsp_hardware_id: usize,
) -> Result<&'static mut BootInfo, &'static str> {
    // Step 1: 提取当前架构与平台的固件交接参数（RISC-V 从 a1 获取 DTB 物理地址；LoongArch64 从 QEMU 固定地址或板级固件交接协议获取 DTB 与命令行地址）。
    #[cfg(target_arch = "riscv64")]
    let physical = raw_args[1];
    #[cfg(target_arch = "loongarch64")]
    let parameters = if current() == Platform::QemuVirt {
        ls2k1000la::FirmwareParameters {
            dtb: QEMU_LOONGARCH_DTB_PHYS,
            command_line: Some(raw_args[1]),
        }
    } else {
        // SAFETY: 调用方通过本函数的 `# Safety` 契约第 2、4 条保证 `raw_args` 所指向的板级固件交接结构（EFI 系统表及配置表数组）在当前线性映射下可读且无并发修改。
        unsafe { ls2k1000la::boot_parameters(raw_args)? }
    };
    #[cfg(target_arch = "loongarch64")]
    let physical = parameters.dtb;
    // Step 2: 校验 DTB 物理地址非空、满足 8 字节对齐，并通过线性映射读取 40 字节 FDT 头部，验证魔数与总长度（40 B ..= 2 MiB）合法且不发生地址溢出。
    if physical == 0
        || physical % FDT_RESERVATION_ALIGNMENT != 0
        || physical.checked_add(FDT_HEADER_BYTES).is_none()
    {
        return Err("firmware DTB physical address is zero or unaligned");
    }

    let pointer = crate::arch::phys_to_virt(physical) as *const u8;
    if (pointer as usize).checked_add(FDT_HEADER_BYTES).is_none() {
        return Err("firmware DTB header virtual address overflows");
    }

    // SAFETY:
    // 1. 前置检查已确认 `physical` 非零、8 字节对齐，且 `physical + 40` 与 `pointer + 40` 均不溢出；
    // 2. 调用方通过 `# Safety` 契约第 2、3 条保证 DTB 头部 40 字节在当前映射下可读、不变且不与 `BootInfo` 重叠。
    let header = unsafe { core::slice::from_raw_parts(pointer, FDT_HEADER_BYTES) };
    if word(header, 0)? != FDT_MAGIC {
        return Err("firmware DTB magic is invalid");
    }
    let total = word(header, FDT_TOTAL_SIZE)? as usize;
    if !(FDT_HEADER_BYTES..=MAX_DTB_SIZE).contains(&total)
        || physical.checked_add(total).is_none()
        || (pointer as usize).checked_add(total).is_none()
    {
        return Err("firmware DTB size is invalid or address overflows");
    }

    // SAFETY:
    // 1. 上方已校验头部魔数，且确认 `total` 在 `40..=MAX_DTB_SIZE` 范围内、`physical + total` 与 `pointer + total` 均不溢出；
    // 2. 调用方通过 `# Safety` 契约第 3 条保证固件声明的 `total` 字节完整 DTB 存储可读、不变且不与 `BootInfo` 重叠。
    let bytes = unsafe { core::slice::from_raw_parts(pointer, total) };

    // Step 3: 初始化全局静态 BootInfo 槽位，执行完整的设备树资源发现，并在 LoongArch64 平台上合并或复制固件命令行。
    // SAFETY: 调用方通过 `# Safety` 契约第 1 条保证当前为 BSP 单核关中断环境且全局 `BootInfo` 槽位首次初始化、无任何并发引用。
    let info = unsafe { crate::boot::prepare(raw_args, bsp_hardware_id) };
    discover_tree(bytes, info, physical)?;

    #[cfg(target_arch = "loongarch64")]
    if let Some(command_line) = parameters.command_line {
        // SAFETY: 调用方通过 `# Safety` 契约第 2、4 条保证固件命令行物理地址已建立线性映射，且至首个 NUL 或容量上限的字节区间可读、不变并与 `info` 不重叠。
        unsafe {
            discover_command_line(info, command_line, current() != Platform::QemuVirt)?;
        }
    }
    Ok(info)
}
