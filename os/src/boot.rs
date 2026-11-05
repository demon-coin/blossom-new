//! 固件资源与内核保留区的不可变快照，由 BSP 在释放 AP 前发布。

use core::mem::MaybeUninit;
use core::ptr;
use spin::Once;

include!(concat!(env!("OUT_DIR"), "/cpu-capacity.rs"));
pub const MAX_BOOTARGS_BYTES: usize = 4096;
pub const MAX_MEMORY_REGIONS: usize = 32;
pub const MAX_RESERVED_REGIONS: usize = 128;
pub const MAX_VIRTIO_MMIO: usize = 8;
pub(crate) const MAX_DEVICE_CLOCKS: usize = 8;
pub(crate) const MAX_DEVICE_RESETS: usize = 3;
pub(crate) const MAX_DEVICE_PINS: usize = 16;
pub(crate) const MAX_PCI_RANGES: usize = 4;

// PCI OF phys.hi 的地址空间编码与 BDF 字段；MEM32/MEM64 共享内存地址域。
pub(crate) const PCI_SPACE_CODE_MASK: u32 = 0x0300_0000;
pub(crate) const PCI_SPACE_IO: u32 = 0x0100_0000;
pub(crate) const PCI_SPACE_MEM32: u32 = 0x0200_0000;
pub(crate) const PCI_SPACE_MEM64: u32 = 0x0300_0000;
pub(crate) const PCI_BUS_SHIFT: u32 = 16;
pub(crate) const PCI_DEVICE_SHIFT: u32 = 11;
pub(crate) const PCI_FUNCTION_SHIFT: u32 = 8;
pub(crate) const PCI_DEVICE_COUNT: u8 = 32;
pub(crate) const PCI_FUNCTION_COUNT: u8 = 8;
pub(crate) const PCI_INTX_PIN_COUNT: u8 = 4;
/// 所选控制器的 source 表容量，PLIC 包含保留的零号源。
pub const MAX_INTERRUPT_SOURCES: usize = if cfg!(blossom_platform = "visionfive2") {
    136 + 1
} else if cfg!(target_arch = "riscv64") {
    95 + 1
} else {
    64
};

/// 构建目标选定的平台身份，运行期不根据固件内容猜测板型。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Platform {
    #[default]
    QemuVirt,
    VisionFive2,
    Ls2k1000la,
}

/// 半开物理地址区间。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Region {
    pub start: usize,
    pub end: usize,
}

impl Region {
    /// 构造外部资源区间，在解析边界拒绝地址溢出和空区间。
    pub fn from_size(start: usize, size: usize) -> Result<Self, &'static str> {
        let end = start.checked_add(size).ok_or("physical region overflows")?;
        if end <= start {
            return Err("physical region is empty");
        }
        Ok(Self { start, end })
    }

    /// 判断两个半开区间是否相交。
    pub fn overlaps(self, other: Self) -> bool {
        self.start < other.end && other.start < self.end
    }
}

/// QEMU 实际提供的 UART 和中断控制器资源。
#[derive(Clone, Copy, Debug)]
pub struct InterruptResources {
    pub uart: Region,
    pub uart_source: u32,
    /// 合法 source 索引的开区间上界；PLIC 的零号不用于设备。
    pub source_count: u32,
    pub controller: Region,
    pub parent_controller: Region,
    pub root_interrupt: u32,
    #[allow(
        dead_code,
        reason = "RISC-V PLIC 按每核 context 访问 claim 和 enable，LoongArch 使用独立中断控制器"
    )]
    pub plic_contexts: [usize; MAX_CPUS],
}

impl Default for InterruptResources {
    /// 空控制器快照在发现时逐字段填充，PLIC槽数由编译容量决定。
    fn default() -> Self {
        Self {
            uart: Region::default(),
            uart_source: 0,
            source_count: 0,
            controller: Region::default(),
            parent_controller: Region::default(),
            root_interrupt: 0,
            plic_contexts: [0; MAX_CPUS],
        }
    }
}

/// UART 的访问几何和输入时钟；可编程时钟由实际 provider 引用描述。
#[derive(Clone, Copy, Debug, Default)]
pub struct UartResources {
    pub reg_io_width: u32,
    pub reg_shift: u32,
    pub clock_frequency: Option<u64>,
    #[allow(
        dead_code,
        reason = "RISC-V 映射和板 UART 准备消费时钟 provider，其他平台保留同一固件资源快照"
    )]
    pub clocks: DeviceClocks,
    pub baud: u32,
    pub dw_apb: bool,
}

/// 时钟或复位控制器中的一个具名绑定项。
#[derive(Clone, Copy, Debug, Default)]
pub struct ResourceReference {
    pub provider: Region,
    pub id: u32,
}

/// 时钟引用及该设备实际要求的父源与频率。
#[derive(Clone, Copy, Debug, Default)]
pub struct ClockResource {
    pub reference: ResourceReference,
    pub parent: Option<ResourceReference>,
    pub rate: Option<u64>,
}

/// 所选设备使用的有界时钟表，顺序保持 clock-names 的绑定顺序。
#[derive(Clone, Copy, Debug, Default)]
pub struct DeviceClocks {
    pub entries: [ClockResource; MAX_DEVICE_CLOCKS],
    pub count: usize,
    pub assigned: [ClockResource; MAX_DEVICE_CLOCKS],
    pub assigned_count: usize,
}

/// 引脚复用值及同组电气设置，数值沿实际 pinctrl 绑定保留。
#[derive(Clone, Copy, Debug, Default)]
pub struct PinResource {
    pub provider: Region,
    pub mux: u32,
    pub pull_up: bool,
    pub bias_disable: bool,
    pub drive_strength: Option<u32>,
    pub input_enable: Option<bool>,
    pub schmitt_enable: Option<bool>,
    pub slew_rate: Option<u32>,
}

/// GPIO 输入的实际控制器、引脚及电平有效方向，不承载外设总线复用。
#[derive(Clone, Copy, Debug, Default)]
pub struct GpioInputResource {
    #[allow(
        dead_code,
        reason = "VF2 的卡检测 GPIO 准备及映射消费 provider，其他平台没有同一 GPIO 控制器"
    )]
    pub provider: Region,
    #[allow(
        dead_code,
        reason = "VF2 卡检测按实际 GPIO pin 读取电平，其他平台使用控制器原生卡检测"
    )]
    pub pin: u32,
    #[allow(
        dead_code,
        reason = "SD 卡检测按固件 GPIO 极性解释实际电平，具体平台可没有外部 CD GPIO"
    )]
    pub active_low: bool,
}

/// 原生设备初始化需要的时钟、复位与引脚资源。
#[derive(Clone, Copy, Debug, Default)]
pub struct DeviceSetup {
    pub clocks: DeviceClocks,
    pub resets: [ResourceReference; MAX_DEVICE_RESETS],
    pub reset_count: usize,
    pub pins: [PinResource; MAX_DEVICE_PINS],
    pub pin_count: usize,
}

/// LIOINTC 每组的主寄存器、每核状态和真实父 cause。
#[derive(Clone, Copy, Debug, Default)]
pub struct LioIntcResources {
    pub registers: Region,
    #[allow(
        dead_code,
        reason = "LoongArch LIOINTC 用本核 ISR 窗口分发，RISC-V 使用 PLIC"
    )]
    pub isr: [Region; 2],
    #[allow(
        dead_code,
        reason = "LoongArch LIOINTC 按固件父 cause 路由硬中断，RISC-V 使用 PLIC"
    )]
    pub parent_interrupt: u32,
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "固件测试逐字段比较 LIOINTC 父路由表，生产解析时已经核定支持的路由"
        )
    )]
    pub parent_map: [u32; 4],
}

/// 固件树声明的 syscon 电源动作。
#[derive(Clone, Copy, Debug)]
pub struct SysconAction {
    #[allow(
        dead_code,
        reason = "LoongArch 最终关机或重启访问固件 syscon 窗口，RISC-V 使用 SBI 动作"
    )]
    pub registers: Region,
    #[allow(
        dead_code,
        reason = "LoongArch 最终电源动作使用固件指定 syscon 偏移，RISC-V 使用 SBI 动作"
    )]
    pub offset: u32,
    #[allow(
        dead_code,
        reason = "LoongArch syscon 电源动作只改固件 mask 指定的位，RISC-V 使用 SBI 动作"
    )]
    pub mask: u32,
    #[allow(
        dead_code,
        reason = "LoongArch syscon 电源动作写入固件指定值，RISC-V 使用 SBI 动作"
    )]
    pub value: u32,
}

/// SD 控制器与卡接口资源，DMA mask 只约束本设备。
#[derive(Clone, Copy, Debug, Default)]
pub struct MmcResources {
    pub registers: Region,
    pub interrupt_source: u32,
    pub setup: DeviceSetup,
    #[allow(
        dead_code,
        reason = "RISC-V 映射 VF2 MMC 的 syscon 资源，其他架构不访问该板窗口"
    )]
    pub syscon: Region,
    #[expect(
        dead_code,
        reason = "资源快照保存 DT 的 MMC syscon 选择字段；当前 SD 驱动不写该选择，不能把它解释为已执行的硬件设置"
    )]
    pub sysreg: [u32; 3],
    pub bus_width: u32,
    pub max_frequency: u32,
    pub fifo_depth: Option<u32>,
    pub data_offset: u32,
    pub card_detect: Option<GpioInputResource>,
    pub cd_inverted: bool,
    pub disable_wp: bool,
    pub post_power_on_delay_ms: u32,
    pub dma_coherent: bool,
    pub dma_mask: u64,
}

/// 原生 PCI 功能身份；BAR 必须在唯一 PCI 枚举后解析。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciFunctionResource {
    pub bus: u8,
    pub device: u8,
    pub function: u8,
    pub interrupt_source: u32,
    pub dma_coherent: bool,
}

/// AHCI 的直接 MMIO 或 PCI 身份，PCI 资源由唯一库存解析 BAR 后补全。
#[derive(Clone, Copy, Debug, Default)]
pub struct AhciResources {
    pub registers: Region,
    pub pci: Option<PciFunctionResource>,
    pub interrupt_source: u32,
    pub dma_coherent: bool,
    pub dma_mask: Option<u64>,
    pub ports_implemented: Option<u32>,
}

/// GMAC 的 MMIO 或 PCI 身份，以及实际 PHY 与时钟配置。
#[derive(Clone, Copy, Debug, Default)]
pub struct GmacResources {
    pub registers: Region,
    pub pci: Option<PciFunctionResource>,
    pub interrupt_source: u32,
    pub setup: DeviceSetup,
    pub syscon: Region,
    pub sysreg: [u32; 2],
    pub mac_address: Option<[u8; 6]>,
    pub phy_address: u32,
    pub phy_mode: PhyMode,
    pub rx_delay_ps: Option<u32>,
    pub tx_delay_ps: Option<u32>,
    pub phy_tx_clock_adjust: bool,
    pub phy_tx_inverted: [bool; 3],
    pub phy_clock_drive_ua: Option<u32>,
    pub phy_data_drive_ua: Option<u32>,
    pub tx_use_rgmii_clock: bool,
    pub dma_coherent: bool,
    pub dma_mask: u64,
}

/// 所选 MAC 支持的实际板接口模式。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PhyMode {
    #[default]
    Rgmii,
    RgmiiId,
    RgmiiRxid,
    RgmiiTxid,
    Rmii,
}

/// JH7110 所选 PLL 的真实输入频率与寄存器窗口，供设备动作读取当前时钟树。
#[derive(Clone, Copy, Debug, Default)]
pub struct BoardClockInputs {
    pub oscillator_hz: Option<u64>,
    pub pll_registers: Region,
}

/// 板级独有资源；QEMU 不创建虚假的原生设备。
#[derive(Clone, Copy, Debug, Default)]
pub struct BoardResources {
    pub clock_inputs: BoardClockInputs,
    pub ccache: Region,
    pub cache_line_size: usize,
    pub liointc: [LioIntcResources; 2],
    pub pmc: Region,
    #[allow(
        dead_code,
        reason = "RISC-V RTC 准备消费真实时钟和复位资源，LoongArch RTC 不使用此板设置"
    )]
    pub rtc_setup: DeviceSetup,
    pub reboot: Option<SysconAction>,
    pub poweroff: Option<SysconAction>,
    pub mmc: Option<MmcResources>,
    pub ahci: Option<AhciResources>,
    pub gmac: Option<GmacResources>,
}

/// PCI host 的实际配置访问协议，缺 host 时不构造 CAM。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PciConfiguration {
    #[default]
    Absent,
    Ecam,
    Loongson,
}

/// 已选 PCI host 提供的总线地址到物理窗口映射。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciRange {
    pub flags: u32,
    pub bus_start: usize,
    pub cpu: Region,
}

/// 启用的 VirtIO-MMIO 槽资源，设备类型由普通初始化读取。
#[derive(Clone, Copy, Debug, Default)]
pub struct VirtioMmioResource {
    pub registers: Region,
    pub interrupt_source: u32,
}

/// PCI 子地址与 INTx pin 经 host 掩码匹配到的控制器源。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciIntxRoute {
    pub child: [u32; 4],
    pub source: u32,
}

/// 记录真实 PCI 配置与窗口，不在启动阶段执行枚举。
#[derive(Clone, Copy, Debug, Default)]
pub struct PciResources {
    pub configuration: Region,
    pub kind: PciConfiguration,
    pub configuration_extended: Region,
    pub ranges: [PciRange; MAX_PCI_RANGES],
    pub range_count: usize,
    pub bus_start: u32,
    pub bus_end: u32,
    pub intx_mask: [u32; 4],
    pub intx_routes: [PciIntxRoute; 16],
    pub intx_route_count: usize,
}

impl PciResources {
    /// 以真实 BDF 高 cell 和 pin 匹配 host INTx 表，不使用 interrupt-line 字节。
    pub fn interrupt_source(
        &self,
        bus: u8,
        device: u8,
        function: u8,
        pin: u8,
    ) -> Result<u32, &'static str> {
        if (bus as u32) < self.bus_start
            || self.bus_end < bus as u32
            || PCI_DEVICE_COUNT <= device
            || PCI_FUNCTION_COUNT <= function
            || !(1..=PCI_INTX_PIN_COUNT).contains(&pin)
        {
            return Err("PCI INTx BDF or pin is out of range");
        }
        let child = [
            ((bus as u32) << PCI_BUS_SHIFT)
                | ((device as u32) << PCI_DEVICE_SHIFT)
                | ((function as u32) << PCI_FUNCTION_SHIFT),
            0,
            0,
            pin as u32,
        ];
        self.intx_routes[..self.intx_route_count]
            .iter()
            .find(|route| {
                (0..child.len()).all(|i| ((route.child[i] ^ child[i]) & self.intx_mask[i]) == 0)
            })
            .map(|route| route.source)
            .ok_or("PCI INTx has no host interrupt-map route")
    }
}

/// 一次解析完成、由 BSP 发布的启动资源快照。
#[derive(Clone, Copy, Debug)]
pub struct BootInfo {
    pub platform: Platform,
    pub raw_args: [usize; 4],
    pub bsp_hardware_id: usize,
    pub cpu_count: usize,
    pub hardware_ids: [usize; MAX_CPUS],
    pub memory_count: usize,
    pub memory: [Region; MAX_MEMORY_REGIONS],
    pub reserved_count: usize,
    pub reserved: [Region; MAX_RESERVED_REGIONS],
    /// 与 reserved 同索引；仅固件明确声明 no-map 的区间禁止最终 RAM 直映。
    pub reserved_no_map: [bool; MAX_RESERVED_REGIONS],
    pub dtb: Region,
    pub initrd: Option<Region>,
    pub uart: UartResources,
    pub board: BoardResources,
    pub interrupts: InterruptResources,
    /// 唯一启用 RTC 的物理寄存器区间，运行期只用于 BSP 初始日历采样。
    pub rtc: Region,
    pub pci: PciResources,
    pub virtio_mmio: [VirtioMmioResource; MAX_VIRTIO_MMIO],
    pub virtio_mmio_count: usize,
    pub bootargs: [u8; MAX_BOOTARGS_BYTES],
    pub bootargs_len: usize,
    pub counter_frequency: u64,
}

impl BootInfo {
    /// 逐字段初始化独占槽，不在早期栈构造或复制整个CPU容量相关快照。
    ///
    /// # Safety
    /// pointer必须指向对齐、可写且独占的Self存储，期间没有读取者或存活字段需要析构。
    pub(crate) unsafe fn initialize_at(
        pointer: *mut Self,
        raw_args: [usize; 4],
        bsp_hardware_id: usize,
    ) {
        // SAFETY: 调用者提供完整独占存储；addr_of_mut不对未初始化对象形成引用，
        // 每个字段写入合法的具体类型值，全部写完以后调用者才可形成Self引用。
        unsafe {
            ptr::addr_of_mut!((*pointer).platform).write(crate::platform::current());
            ptr::addr_of_mut!((*pointer).raw_args).write(raw_args);
            ptr::addr_of_mut!((*pointer).bsp_hardware_id).write(bsp_hardware_id);
            ptr::addr_of_mut!((*pointer).cpu_count).write(0);
            ptr::addr_of_mut!((*pointer).hardware_ids).write([0; MAX_CPUS]);
            ptr::addr_of_mut!((*pointer).memory_count).write(0);
            ptr::addr_of_mut!((*pointer).memory).write([Region::default(); MAX_MEMORY_REGIONS]);
            ptr::addr_of_mut!((*pointer).reserved_count).write(0);
            ptr::addr_of_mut!((*pointer).reserved).write([Region::default(); MAX_RESERVED_REGIONS]);
            ptr::addr_of_mut!((*pointer).reserved_no_map).write([false; MAX_RESERVED_REGIONS]);
            ptr::addr_of_mut!((*pointer).dtb).write(Region::default());
            ptr::addr_of_mut!((*pointer).initrd).write(None);
            ptr::addr_of_mut!((*pointer).uart).write(UartResources::default());
            ptr::addr_of_mut!((*pointer).board).write(BoardResources::default());
            ptr::addr_of_mut!((*pointer).interrupts).write(InterruptResources::default());
            ptr::addr_of_mut!((*pointer).rtc).write(Region::default());
            ptr::addr_of_mut!((*pointer).pci).write(PciResources::default());
            ptr::addr_of_mut!((*pointer).virtio_mmio)
                .write([VirtioMmioResource::default(); MAX_VIRTIO_MMIO]);
            ptr::addr_of_mut!((*pointer).virtio_mmio_count).write(0);
            ptr::addr_of_mut!((*pointer).bootargs).write([0; MAX_BOOTARGS_BYTES]);
            ptr::addr_of_mut!((*pointer).bootargs_len).write(0);
            ptr::addr_of_mut!((*pointer).counter_frequency).write(0);
        }
    }

    /// 借用固件命令行的实际字节前缀，不包含终止 NUL 或补齐空间。
    pub fn bootargs(&self) -> &[u8] {
        &self.bootargs[..self.bootargs_len]
    }

    /// 登记阻止分配但仍允许直映的普通保留资源。
    pub fn reserve(&mut self, region: Region) -> Result<(), &'static str> {
        self.reserve_with_attribute(region, false)
    }

    /// 登记同时禁止分配和最终 RAM 直映的固件保留资源。
    pub fn reserve_no_map(&mut self, region: Region) -> Result<(), &'static str> {
        self.reserve_with_attribute(region, true)
    }

    /// 容量检查成功后一次提交区间和属性，失败不修改任何已登记记录。
    fn reserve_with_attribute(&mut self, region: Region, no_map: bool) -> Result<(), &'static str> {
        if MAX_RESERVED_REGIONS <= self.reserved_count {
            return Err("boot reservation capacity exceeded");
        }
        self.reserved[self.reserved_count] = region;
        self.reserved_no_map[self.reserved_count] = no_map;
        self.reserved_count += 1;
        Ok(())
    }
}

static mut BOOT_STORAGE: MaybeUninit<BootInfo> = MaybeUninit::uninit();
static BOOT_INFO: Once<&'static BootInfo> = Once::new();

/// BSP在永久槽准备唯一可变快照，发现及内核保留完成后由publish冻结。
///
/// # Safety
/// BSP只能在IRQ关闭、AP未释放时调用一次，且存储尚无任何引用；返回拥有不得复制或泄漏可变别名。
pub(crate) unsafe fn prepare(
    raw_args: [usize; 4],
    bsp_hardware_id: usize,
) -> &'static mut BootInfo {
    assert!(BOOT_INFO.get().is_none(), "BootInfo is already published");
    let pointer = ptr::addr_of_mut!(BOOT_STORAGE).cast::<BootInfo>();
    // SAFETY: BSP调用契约独占永久槽，MaybeUninit允许逐字段写入；完成初始化后才建立唯一可变借用。
    unsafe {
        BootInfo::initialize_at(pointer, raw_args, bsp_hardware_id);
        &mut *pointer
    }
}

/// 从附近高地址链接符号取得完整内核物理保留区，包含栈和启动表。
pub fn kernel_region() -> Region {
    extern "C" {
        static __kernel_virt_start: u8;
        static __kernel_virt_end: u8;
    }
    Region {
        start: crate::arch::virt_to_phys(core::ptr::addr_of!(__kernel_virt_start) as usize),
        end: crate::arch::virt_to_phys(core::ptr::addr_of!(__kernel_virt_end) as usize),
    }
}

/// 冻结实际固件资源与当前内核保留区，发布永久只读借用而不复制整个快照。
///
/// # Safety
/// 只能由BSP在AP尚未释放、IRQ关闭时执行一次。info必须是prepare返回的唯一借用，
/// 来自选定机器的实际固件，已验证全部资源并包含内核保留区；发布后不得再修改存储。
pub unsafe fn publish(info: &'static mut BootInfo) {
    assert!(BOOT_INFO.get().is_none(), "BootInfo is already published");
    let info: &'static BootInfo = info;
    BOOT_INFO.call_once(|| info);
}

/// 借用已发布的真实启动资源；发布前调用会在这个边界失败。
pub fn info() -> &'static BootInfo {
    BOOT_INFO
        .get()
        .copied()
        .expect("BootInfo has not been published")
}
