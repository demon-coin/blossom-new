//! 2K1000LA 的 LIOINTC、Loongson PCI 与原生外设绑定。

use core::convert::{TryFrom, TryInto};

use super::resources;
use super::{
    cells, compatible, device_region, enabled, identity_address, interrupt_parent, number, parent,
    property, regions, require_controller, single_region, word, Fdt, Node,
};
use crate::boot::{
    AhciResources, BootInfo, LioIntcResources, PciConfiguration, PciFunctionResource, PciIntxRoute,
    PciRange, Region, SysconAction,
};

/// 固件交接中已归一化的物理地址，解析完成后不保留 EFI 表借用。
pub(super) struct FirmwareParameters {
    pub(super) dtb: usize,
    pub(super) command_line: Option<usize>,
}

const EFI_SYSTEM_TABLE_SIGNATURE: u64 = 0x5453_5953_2049_4249;
const EFI_SYSTEM_TABLE_SIZE: usize = 0x78;
const EFI_HEADER_SIZE_OFFSET: usize = 0x0c;
const EFI_NR_TABLES_OFFSET: usize = 0x68;
const EFI_TABLES_OFFSET: usize = 0x70;
const EFI_CONFIG_TABLE_SIZE: usize = 24;
const EFI_CONFIG_TABLE_POINTER_OFFSET: usize = 16;
const EFI_FDT_GUID: [u8; 16] = [
    0xd5, 0x21, 0xb6, 0xb1, 0x9c, 0xf1, 0xa5, 0x41, 0x83, 0x0b, 0xd9, 0x15, 0x2c, 0x69, 0xaa, 0xe0,
];

/// 一次解释 UHI、显式 a3 DTB 或 EFI64 bootm 交接，不扫描物理内存。
///
/// # Safety
/// 固件须保证所选 ABI 引用的 EFI 系统表、其声明的完整头部与配置表数组在解析期间可读、
/// 无并发修改，且对应 RAM 映射已经建立；内容校验不能证明指针可读。
/// 返回的 DTB 与命令行仍由调用者保活，直到完整 FDT 解析和命令行复制结束。
pub(super) unsafe fn boot_parameters(args: [usize; 4]) -> Result<FirmwareParameters, &'static str> {
    if args[0] == usize::MAX - 1 {
        return Ok(FirmwareParameters {
            dtb: physical(args[1])?,
            command_line: None,
        });
    }
    let command_line = if args[0] <= 1 && args[1] != 0 {
        Some(physical(args[1])?)
    } else {
        None
    };
    let dtb = if args[3] != 0 {
        physical(args[3])?
    } else if args[0] <= 1 && args[2] != 0 {
        // SAFETY: 调用者保证 EFI 系统表及其声明的配置表在本次解析期间可读且不变。
        unsafe { efi_dtb(args[2])? }
    } else {
        return Err("LoongArch firmware did not supply a supported OS DTB handoff");
    };
    Ok(FirmwareParameters { dtb, command_line })
}

/// 从 EFI64 系统表的配置表数组取得唯一 FDT GUID 对应的物理地址。
///
/// # Safety
/// 调用者须保活系统表声明的完整头部与 nr_tables 个配置表项，保证解析期间可读且无并发修改；
/// 所有物理地址的当前 RAM 映射须已建立，返回的 DTB 另由上层保活并校验。
unsafe fn efi_dtb(address: usize) -> Result<usize, &'static str> {
    // SAFETY: 调用者保证 EFI64 系统表的固定字段可读且不变；此处只借用最小布局。
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
    // SAFETY: 调用者保证头部声明的存储范围可读且不变；长度仅作表示范围校验，不证明可读性。
    unsafe { firmware_bytes(address, header_size)? };
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
    // SAFETY: 调用者保证 nr_tables 个 EFI64 表项可读且不变；乘法及地址表示范围在借用前检查。
    let tables = unsafe { firmware_bytes(tables_address, size)? };
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

/// 检查固件存储的地址表示范围后借用字节，不推断实际可读性。
///
/// # Safety
/// 调用者须保证指定范围在返回借用存续期间可读且无并发修改，并已建立当前 RAM 映射。
unsafe fn firmware_bytes<'a>(address: usize, size: usize) -> Result<&'a [u8], &'static str> {
    let address = physical(address)?;
    let end = address
        .checked_add(size)
        .ok_or("firmware physical storage range overflows")?;
    if (1usize << 60) < end || (isize::MAX as usize) < size {
        return Err("firmware storage range is not representable");
    }
    let pointer = crate::arch::phys_to_virt(address) as *const u8;
    if (pointer as usize).checked_add(size).is_none() {
        return Err("firmware virtual storage range overflows");
    }
    // SAFETY: 可读性、无并发写及借用寿命由调用者保证；非空地址、切片长度与地址加法已检查。
    Ok(unsafe { core::slice::from_raw_parts(pointer, size) })
}

/// 固件可传物理地址或 cached/uncached DMW 地址；其他地址类别拒绝。
fn physical(address: usize) -> Result<usize, &'static str> {
    if !matches!(address >> 60, 0 | 8 | 9) {
        return Err("firmware address is neither physical nor a supported DMW alias");
    }
    let address = address & 0x0fff_ffff_ffff_ffff;
    if address == 0 {
        return Err("firmware address is null");
    }
    Ok(address)
}

/// 由实际 LIOINTC 与 PCI 身份构造板快照，PCI reg 从不作为 BAR。
pub(super) fn discover<'a>(
    tree: &'a Fdt<'a>,
    uart: Node<'a>,
    info: &mut BootInfo,
) -> Result<(), &'static str> {
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
        if !(2..=3).contains(&cause) {
            return Err("FDT LIOINTC parent cause is unsupported");
        }
        let index = (cause - 2) as usize;
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
            || main.end - main.start != 0x40
            || isr0.end - isr0.start != 8
            || isr1.end - isr1.start != 8
        {
            return Err("FDT LIOINTC register geometry is unsupported");
        }
        let map = property(node, "loongson,parent_int_map")?;
        if map.len() != 16 {
            return Err("FDT LIOINTC parent masks are truncated");
        }
        let mut parent_map = [0; 4];
        for (slot, value) in parent_map.iter_mut().enumerate() {
            *value = word(map, slot * 4)?;
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
    let banks = [
        banks[0].ok_or("FDT LIOINTC0 is missing")?,
        banks[1].ok_or("FDT LIOINTC1 is missing")?,
    ];
    info.interrupts.controller = info.board.liointc[0].registers;
    info.interrupts.parent_controller = info.board.liointc[1].registers;
    info.interrupts.root_interrupt = 2;
    info.interrupts.source_count = 64;
    info.interrupts.uart_source = lio_irq(tree, uart, banks)?;
    if info.hardware_ids[..info.cpu_count]
        .iter()
        .any(|id| 2 <= *id)
    {
        return Err("FDT LA264 hardware CPU ID exceeds the dual-core board");
    }
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
    discover_pci(tree, banks, info)
}

/// 读取一个板级 syscon 动作并验证寄存器范围。
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

/// 取得唯一的实际控制器绑定。
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

/// 将真实 LIOINTC bank/source 规范为单一外部 IRQ 域编号。
fn lio_irq<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    banks: [Node<'a>; 2],
) -> Result<u32, &'static str> {
    let (controller, source, flags) = if let Some(extended) = node.property("interrupts-extended") {
        if extended.value.len() < 12 || extended.value.len() % 12 != 0 {
            return Err("FDT LIOINTC extended interrupt is truncated");
        }
        let controller = tree
            .find_phandle(word(extended.value, 0)?)
            .ok_or("FDT LIOINTC interrupt parent is missing")?;
        (
            controller,
            word(extended.value, 4)?,
            word(extended.value, 8)?,
        )
    } else {
        let controller = interrupt_parent(tree, node)?;
        let raw = property(node, "interrupts")?;
        if raw.len() != 8 {
            return Err("FDT LIOINTC interrupt is not two cells");
        }
        (controller, word(raw, 0)?, word(raw, 4)?)
    };
    let index = banks
        .iter()
        .position(|bank| bank.name.as_ptr() == controller.name.as_ptr())
        .ok_or("FDT device is not connected to selected LIOINTC")?;
    if 32 <= source || flags != 4 {
        return Err("FDT LIOINTC device source or trigger is unsupported");
    }
    Ok(index as u32 * 32 + source)
}

/// 收集 Loongson CFG0/CFG1、窗口与所选集成功能资源，保留 SATA 节点显式声明的端口图。
fn discover_pci<'a>(
    tree: &'a Fdt<'a>,
    banks: [Node<'a>; 2],
    info: &mut BootInfo,
) -> Result<(), &'static str> {
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
    identity_address(tree, host)?;
    let mut registers = regions(host)?;
    info.pci.configuration = registers.next().ok_or("FDT Loongson CFG0 is missing")??;
    info.pci.configuration_extended = registers.next().ok_or("FDT Loongson CFG1 is missing")??;
    if registers.next().is_some() {
        return Err("FDT Loongson PCI has unsupported configuration windows");
    }
    info.pci.kind = PciConfiguration::Loongson;
    let buses = property(host, "bus-range")?;
    if buses.len() != 8 || word(buses, 0)? != 0 || 255 < word(buses, 4)? {
        return Err("FDT Loongson PCI bus range is invalid");
    }
    info.pci.bus_start = 0;
    info.pci.bus_end = word(buses, 4)?;
    let ranges = property(host, "ranges")?;
    if ranges.is_empty() || ranges.len() % 28 != 0 {
        return Err("FDT Loongson PCI ranges are truncated");
    }
    for range in ranges.chunks_exact(28) {
        if info.pci.range_count == info.pci.ranges.len() {
            return Err("FDT Loongson PCI range capacity exceeded");
        }
        let flags = word(range, 0)?;
        if !matches!(flags & 0x0300_0000, 0x0100_0000 | 0x0200_0000 | 0x0300_0000) {
            return Err("FDT Loongson PCI window kind is unsupported");
        }
        let cpu = Region::from_size(cells(&range[12..20])?, cells(&range[20..28])?)?;
        device_region(cpu, info)?;
        info.pci.ranges[info.pci.range_count] = PciRange {
            flags,
            bus_start: cells(&range[4..12])?,
            cpu,
        };
        info.pci.range_count += 1;
    }
    info.pci.intx_mask = [0x00ff_ff00, 0, 0, 0];
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

/// PCI 五 cell reg 中只有首 cell 的 BDF 构成配置身份，其零 size 不建立 MMIO。
fn pci_function<'a>(
    tree: &'a Fdt<'a>,
    node: Node<'a>,
    banks: [Node<'a>; 2],
) -> Result<PciFunctionResource, &'static str> {
    let reg = property(node, "reg")?;
    if reg.len() != 20 || reg[4..].iter().any(|byte| *byte != 0) {
        return Err("FDT selected PCI function reg is not configuration identity");
    }
    let address = word(reg, 0)?;
    if address & 0xff00_00ff != 0 {
        return Err("FDT PCI configuration identity is invalid");
    }
    Ok(PciFunctionResource {
        bus: (address >> 16) as u8,
        device: ((address >> 11) & 31) as u8,
        function: ((address >> 8) & 7) as u8,
        interrupt_source: lio_irq(tree, node, banks)?,
        dma_coherent: resources::coherent(tree, node, false)?,
    })
}

/// 所选集成功能按完整 BDF 取得 DTS 中固定连线，不使用配置 interrupt-line。
fn add_route(info: &mut BootInfo, function: PciFunctionResource) -> Result<(), &'static str> {
    if info.pci.intx_route_count == info.pci.intx_routes.len() {
        return Err("FDT PCI function route capacity exceeded");
    }
    info.pci.intx_routes[info.pci.intx_route_count] = PciIntxRoute {
        child: [
            ((function.bus as u32) << 16)
                | ((function.device as u32) << 11)
                | ((function.function as u32) << 8),
            0,
            0,
            0,
        ],
        source: function.interrupt_source,
    };
    info.pci.intx_route_count += 1;
    Ok(())
}

/// 无 PCI host 的自定义树只接受真实 MMIO 绑定，不把 PCI 子节点的 reg 重新解释。
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
    let bus = parent(tree, sata).ok_or("FDT AHCI parent bus is missing")?;
    if bus.property("device_type").and_then(|p| p.as_str()) == Some("pci") {
        return Err("FDT AHCI PCI identity cannot be used as MMIO");
    }
    let registers = single_region(tree, sata)?;
    device_region(registers, info)?;
    if registers.end - registers.start < 0x1100 {
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
    let mac = resources::alias(tree, "ethernet0")?;
    if !(compatible(mac, "snps,dwmac-3.70a") || compatible(mac, "snps,arc-dwmac-3.70a")) {
        return Err("FDT ethernet0 is not a DWMAC1000 MMIO binding");
    }
    let mut gmac = resources::gmac(tree, mac, false)?;
    gmac.registers = single_region(tree, mac)?;
    device_region(gmac.registers, info)?;
    if gmac.registers.end - gmac.registers.start < 0x105c {
        return Err("FDT GMAC MMIO window is too short");
    }
    gmac.interrupt_source = lio_irq(tree, mac, banks)?;
    info.board.gmac = Some(gmac);
    Ok(())
}
