//! 链接输入与内核地址换算共用的四平台常量；镜像末端仍由链接器计算。

/// 地址字段保存目标的 64 位数值，不依赖构建脚本宿主的指针宽度。
#[derive(Clone, Copy)]
pub struct KernelLinkLayout {
    /// ELF 的 __blossom_platform：QEMU 为 0，VisionFive2 为 1，LS2K1000LA 为 2。
    #[allow(
        dead_code,
        reason = "build.rs 将平台编号写入链接符号，内核直接读取构建平台配置"
    )]
    pub platform: u8,
    /// 内核镜像最低装载物理地址，RISC-V 的物理启动段也从此处开始。
    #[allow(
        dead_code,
        reason = "build.rs 生成装载地址，RISC-V 地址换算也读取；LoongArch 使用 DMW"
    )]
    pub kernel_load_phys: u64,
    /// 内核高地址镜像的链接基址，RISC-V 正文位于物理启动段之后。
    #[allow(
        dead_code,
        reason = "build.rs 生成链接地址，RISC-V 地址换算也读取；LoongArch 使用 DMW"
    )]
    pub kernel_link_virt: u64,
    /// RISC-V 对应 ram_phys_base；LoongArch 对应物理地址零的缓存 DMW。
    pub direct_map_virt_base: u64,
    /// 物理设备地址零对应的非缓存设备窗口。
    pub mmio_virt_base: u64,
    /// RISC-V 直映窗口的物理起点；实际可分配 RAM 范围仍由固件资源决定。
    #[allow(
        dead_code,
        reason = "build.rs 生成 RAM 基址，RISC-V 直映换算读取；LoongArch 从物理零开始映射"
    )]
    pub ram_phys_base: u64,
}

pub const RISCV64_QEMU_VIRT: KernelLinkLayout = KernelLinkLayout {
    platform: 0,
    kernel_load_phys: 0x8020_0000,
    kernel_link_virt: 0xffff_ffff_8000_0000,
    direct_map_virt_base: 0xffff_ffd6_0000_0000,
    mmio_virt_base: 0xffff_ffc0_0000_0000,
    ram_phys_base: 0x8000_0000,
};

pub const RISCV64_VISIONFIVE2: KernelLinkLayout = KernelLinkLayout {
    platform: 1,
    kernel_load_phys: 0x4020_0000,
    kernel_link_virt: 0xffff_ffff_8000_0000,
    direct_map_virt_base: 0xffff_ffd6_0000_0000,
    mmio_virt_base: 0xffff_ffc0_0000_0000,
    ram_phys_base: 0x4000_0000,
};

pub const LOONGARCH64_QEMU_VIRT: KernelLinkLayout = KernelLinkLayout {
    platform: 0,
    kernel_load_phys: 0x9000_0000,
    kernel_link_virt: 0x9000_0000_9000_0000,
    direct_map_virt_base: 0x9000_0000_0000_0000,
    mmio_virt_base: 0x8000_0000_0000_0000,
    ram_phys_base: 0,
};

pub const LOONGARCH64_LS2K1000LA: KernelLinkLayout = KernelLinkLayout {
    platform: 2,
    kernel_load_phys: 0x0020_0000,
    kernel_link_virt: 0x9000_0000_0020_0000,
    direct_map_virt_base: 0x9000_0000_0000_0000,
    mmio_virt_base: 0x8000_0000_0000_0000,
    ram_phys_base: 0,
};

// DMW 顶部段号与低60位地址部分分开解释，不能套用页表物理地址宽度。
pub(crate) const DMW_VSEG_SHIFT: u32 = 60;
pub(crate) const DMW_ADDRESS_END: u64 = 1 << DMW_VSEG_SHIFT;
#[allow(
    dead_code,
    reason = "LS2K 固件地址归一化读取，宿主 build.rs 只生成链接布局"
)]
pub(crate) const DMW_ADDRESS_MASK: u64 = DMW_ADDRESS_END - 1;
#[allow(
    dead_code,
    reason = "LS2K 固件地址校验读取，宿主 build.rs 不解析固件地址"
)]
pub(crate) const DMW_CACHED_VSEG: usize =
    (LOONGARCH64_QEMU_VIRT.direct_map_virt_base >> DMW_VSEG_SHIFT) as usize;
#[allow(
    dead_code,
    reason = "LS2K 固件地址校验读取，宿主 build.rs 不解析固件地址"
)]
pub(crate) const DMW_UNCACHED_VSEG: usize =
    (LOONGARCH64_QEMU_VIRT.mmio_virt_base >> DMW_VSEG_SHIFT) as usize;
