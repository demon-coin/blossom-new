//! 平台内核链接布局与高半核虚拟地址空间常量定义。
//!
//! # 模块职责与架构定位
//! 本模块是构建脚本（`build.rs`）与内核运行时（`crate::platform`、`crate::arch`）共享的纯常量定义层：
//! - **构建期上游**：`build.rs` 直接引入本文件，根据目标架构与板型生成链接脚本符号（如装载物理基址、虚拟链接基址及 `__blossom_platform` 标识）。
//! - **运行期下游**：内核早期启动代码、物理/虚拟地址换算函数（`phys_to_virt`、`virt_to_phys`、`mmio_to_virt`）以及龙芯 2K1000LA 固件地址归一化逻辑读取本模块常量。
//!
//! # 核心地址空间与位域布局
//!
//! ## 1. RISC-V 64 (Sv39) 内核高地址空间布局
//! ```text
//! 虚拟地址空间 (Sv39 高半核)                        物理地址空间
//! +----------------------------------+
//! | 0xffff_ffff_8000_0000            | ----------> kernel_load_phys
//! | 内核高地址镜像 (kernel_link_virt) |             (QEMU: 0x8020_0000 / VF2: 0x4020_0000)
//! +----------------------------------+
//! | 0xffff_ffd6_0000_0000            | ----------> ram_phys_base
//! | 线性直映窗口 (direct_map_virt_base)|             (QEMU: 0x8000_0000 / VF2: 0x4000_0000)
//! +----------------------------------+
//! | 0xffff_ffc0_0000_0000            | ----------> 0x0000_0000
//! | 设备寄存器窗口 (mmio_virt_base)    |             (物理设备地址 0 起点)
//! +----------------------------------+
//! ```
//!
//! ## 2. LoongArch64 直接映射配置窗口 (DMW) 地址编码
//! ```text
//! 63      60 59                                                  0
//! +---------+----------------------------------------------------+
//! |  VSEG   |                物理地址偏移 (60 bits)                |
//! +---------+----------------------------------------------------+
//!   4 bits                   DMW_ADDRESS_MASK (0x0fff_ffff_ffff_ffff)
//!
//! - VSEG = 0x9 (DMW_CACHED_VSEG)  : 可缓存线性直映窗口 (direct_map_virt_base = 0x9000_0000_0000_0000)
//! - VSEG = 0x8 (DMW_UNCACHED_VSEG): 强序非缓存设备窗口 (mmio_virt_base       = 0x8000_0000_0000_0000)
//! ```
//!
//! # 锁顺序与并发契约
//! 本模块仅包含编译期不可变常量（`const`），不涉及任何运行时可变状态或锁，可在构建脚本、早期未分页环境、普通任务及中断上下文中无约束访问。

/// 内核链接与地址空间映射布局描述符。
///
/// 所有地址字段均固定使用 `u64` 表示 64 位目标机的地址数值（单位：字节），
/// 避免受构建脚本（`build.rs`）所在宿主机指针宽度影响。内核镜像的结束地址由链接器在链接阶段计算生成。
#[derive(Clone, Copy)]
pub struct KernelLinkLayout {
    /// 写入 ELF 符号 `__blossom_platform` 的平台编号：QEMU Virt 为 `0`，VisionFive2 为 `1`，LS2K1000LA 为 `2`。
    #[allow(
        dead_code,
        reason = "build.rs 将平台编号写入链接符号，内核直接读取构建平台配置"
    )]
    pub platform: u8,
    /// 内核镜像在物理内存中的最低装载起始地址（单位：字节）；在 RISC-V 平台上也是早期物理启动段（`.text.boot`）的起点。
    #[allow(
        dead_code,
        reason = "build.rs 生成装载地址，RISC-V 地址换算也读取；LoongArch 使用 DMW"
    )]
    pub kernel_load_phys: u64,
    /// 内核主镜像在高半核虚拟地址空间中的链接起始地址（单位：字节）；在 RISC-V 平台上对应物理启动段之后的内核正文起点。
    #[allow(
        dead_code,
        reason = "build.rs 生成链接地址，RISC-V 地址换算也读取；LoongArch 使用 DMW"
    )]
    pub kernel_link_virt: u64,
    /// 物理内存线性直映窗口的虚拟基址（单位：字节）。
    ///
    /// 在 RISC-V 上对应 `ram_phys_base` 物理地址的映射起点；在 LoongArch64 上对应物理地址 `0` 的可缓存直接映射窗口（DMW VSEG `0x9`）。
    pub direct_map_virt_base: u64,
    /// 物理地址 `0` 对应的非缓存设备寄存器（MMIO）虚拟窗口基址（单位：字节）。
    pub mmio_virt_base: u64,
    /// 线性直映窗口对应的起始物理地址（单位：字节）。
    ///
    /// 在 RISC-V 上为 DRAM 起始物理基址，在 LoongArch64 上固定为 `0`。实际可分配的物理内存区间仍由启动阶段的固件设备树（FDT）发现结果决定。
    #[allow(
        dead_code,
        reason = "build.rs 生成 RAM 基址，RISC-V 直映换算读取；LoongArch 从物理零开始映射"
    )]
    pub ram_phys_base: u64,
}

/// RISC-V 64 架构 QEMU `virt` 虚拟机平台的链接与地址布局常量。
pub const RISCV64_QEMU_VIRT: KernelLinkLayout = KernelLinkLayout {
    platform: 0,
    kernel_load_phys: 0x8020_0000,
    kernel_link_virt: 0xffff_ffff_8000_0000,
    direct_map_virt_base: 0xffff_ffd6_0000_0000,
    mmio_virt_base: 0xffff_ffc0_0000_0000,
    ram_phys_base: 0x8000_0000,
};

/// RISC-V 64 架构 StarFive VisionFive 2（JH7110 SoC）开发板平台的链接与地址布局常量。
pub const RISCV64_VISIONFIVE2: KernelLinkLayout = KernelLinkLayout {
    platform: 1,
    kernel_load_phys: 0x4020_0000,
    kernel_link_virt: 0xffff_ffff_8000_0000,
    direct_map_virt_base: 0xffff_ffd6_0000_0000,
    mmio_virt_base: 0xffff_ffc0_0000_0000,
    ram_phys_base: 0x4000_0000,
};

/// LoongArch64 架构 QEMU `virt` 虚拟机平台的链接与地址布局常量。
pub const LOONGARCH64_QEMU_VIRT: KernelLinkLayout = KernelLinkLayout {
    platform: 0,
    kernel_load_phys: 0x9000_0000,
    kernel_link_virt: 0x9000_0000_9000_0000,
    direct_map_virt_base: 0x9000_0000_0000_0000,
    mmio_virt_base: 0x8000_0000_0000_0000,
    ram_phys_base: 0,
};

/// LoongArch64 架构龙芯 2K1000LA 开发板平台的链接与地址布局常量。
pub const LOONGARCH64_LS2K1000LA: KernelLinkLayout = KernelLinkLayout {
    platform: 2,
    kernel_load_phys: 0x0020_0000,
    kernel_link_virt: 0x9000_0000_0020_0000,
    direct_map_virt_base: 0x9000_0000_0000_0000,
    mmio_virt_base: 0x8000_0000_0000_0000,
    ram_phys_base: 0,
};

/// LoongArch64 直接映射窗口（DMW）虚拟段号（`VSEG`）在 64 位虚拟地址中的起始位偏移（第 60 位）。
///
/// DMW 通过最高 4 位段号（bits `[63:60]`）匹配窗口配置，低 60 位（bits `[59:0]`）直接作为物理地址偏移，独立于页表物理地址宽度。
pub(crate) const DMW_VSEG_SHIFT: u32 = 60;
/// 单个 LoongArch64 DMW 窗口可覆盖的物理地址空间上限（$2^{60}$ 字节）。
pub(crate) const DMW_ADDRESS_END: u64 = 1 << DMW_VSEG_SHIFT;
/// 提取 LoongArch64 DMW 虚拟地址低 60 位物理偏移量的位掩码（`0x0fff_ffff_ffff_ffff`）。
#[allow(
    dead_code,
    reason = "LS2K 固件地址归一化读取，宿主 build.rs 只生成链接布局"
)]
pub(crate) const DMW_ADDRESS_MASK: u64 = DMW_ADDRESS_END - 1;
/// LoongArch64 可缓存线性直映窗口（Cached DMW）的 4 位段号（`0x9`）。
#[allow(
    dead_code,
    reason = "LS2K 固件地址校验读取，宿主 build.rs 不解析固件地址"
)]
pub(crate) const DMW_CACHED_VSEG: usize =
    (LOONGARCH64_QEMU_VIRT.direct_map_virt_base >> DMW_VSEG_SHIFT) as usize;
/// LoongArch64 非缓存设备映射窗口（Uncached MMIO DMW）的 4 位段号（`0x8`）。
#[allow(
    dead_code,
    reason = "LS2K 固件地址校验读取，宿主 build.rs 不解析固件地址"
)]
pub(crate) const DMW_UNCACHED_VSEG: usize =
    (LOONGARCH64_QEMU_VIRT.mmio_virt_base >> DMW_VSEG_SHIFT) as usize;
