//! AHCI 控制器（HBA）与端口 MMIO 寄存器定义、强序访问及硬件初始化序列。
//!
//! # 模块职责与架构定位
//! 本模块位于 AHCI 驱动内部（`drivers::ahci::registers`），封装对 AHCI ABAR（AHCI Base Memory
//! Register，PCI BAR5 或平台 MMIO 窗口）的 volatile 寄存器读写与底层硬件状态转换：
//! - **上游调用者**：`drivers::ahci` 在设备探测、中断应答、命令下发及引擎停止状态机中调用
//!   [`Host`] 与 [`Port`] 的方法。
//! - **下游依赖**：依赖架构层 `crate::arch::mmio_address` 获取设备强序映射虚拟地址，依赖
//!   `crate::time` 与 `crate::timer` 实现有限时间的忙等轮询与链路复位延时。
//!
//! # 核心设计与寄存器布局
//!
//! ## 1. AHCI ABAR 内存映射（HBA Memory Space）布局
//!
//! ```text
//! +------------------+--------------------------------------------------------+
//! | Offset           | Region / Register                                      |
//! +------------------+--------------------------------------------------------+
//! | 0x0000 .. 0x002B | Generic Host Control (全局主机控制区)                  |
//! |   0x00: CAP      | Host Capabilities (64-bit DMA、端口数上限等)            |
//! |   0x04: GHC      | Global Host Control (AE: AHCI 使能, IE: 全局中断, HR)  |
//! |   0x08: HOST_IS  | Interrupt Status (各端口中断汇总位图，W1C)             |
//! |   0x0C: PI       | Ports Implemented (实际实现的端口位图)                 |
//! |   0x10: VERSION  | AHCI Version (如 0x0001_0200 表示 v1.2)                |
//! |   0x24: CAP2     | Host Capabilities Extended (BOH: BIOS/OS 交接支持)     |
//! |   0x28: BOHC     | BIOS/OS Handoff Control and Status                     |
//! +------------------+--------------------------------------------------------+
//! | 0x002C .. 0x00FF | Reserved / Vendor Specific / NVMHCI                    |
//! +------------------+--------------------------------------------------------+
//! | 0x0100 .. 0x10FF | Port Control Registers (Port 0..31，每个端口 0x80 B)   |
//! |   +0x00: CLB/U   | Command List Base Address (1K 对齐物理地址低/高 32 位) |
//! |   +0x08: FB/U    | Received FIS Base Address (256B 对齐物理地址低/高32位) |
//! |   +0x10: PORT_IS | Port Interrupt Status (端口中断状态，W1C)              |
//! |   +0x14: IE      | Port Interrupt Enable (端口中断使能掩码)               |
//! |   +0x18: CMD     | Command and Status (ST, FRE, FR, CR, SUD, POD, ICC)    |
//! |   +0x20: TFD     | Task File Data (包含 ATA Status: BSY, DRQ, DF, ERR)    |
//! |   +0x24: SIG     | Port Signature (SATA 磁盘为 0x0000_0101)               |
//! |   +0x28: SSTS    | SATA Status (SCR0: IPM 电源状态、SPD 速率、DET 检测)   |
//! |   +0x2C: SCTL    | SATA Control (SCR2: DET 链路复位控制、IPM 低功耗禁用)  |
//! |   +0x30: SERR    | SATA Error (SCR1: 链路与协议错误位，W1C)               |
//! |   +0x34: SACT    | SATA Active (NCQ 活动槽位图，本驱动非 NCQ 恒为 0)      |
//! |   +0x38: CI      | Command Issue (命令槽门铃位图，写 1 触发硬件拉取命令)  |
//! |   +0x40: FBS     | FIS-based Switching Control                            |
//! +------------------+--------------------------------------------------------+
//! ```
//!
//! ## 2. 端口 DMA 引擎启停顺序约束（`PxCMD`）
//! 根据 AHCI 规范，端口命令列表 DMA 引擎（`ST`/`CR`）与 FIS 接收 DMA 引擎（`FRE`/`FR`）的停止顺序严格受限：
//! 1. 必须先清除 `CMD.ST`（位 0），并轮询等待硬件将 `CMD.CR`（Command List Running，位 15）清零；
//! 2. 确认 `CR == 0` 后，才允许清除 `CMD.FRE`（位 4），随后轮询等待 `CMD.FR`（FIS Receive Running，位 14）清零；
//! 3. 只有当 `CR == 0` 且 `FR == 0` 均确认后，硬件才彻底释放对命令列表、命令表、Received FIS 及数据缓冲区的 DMA 访问权。
//!
//! # 锁与并发契约
//! - [`Host`] 与 [`Port`] 自身为轻量级 `Copy` 句柄，不持有锁。
//! - 探测阶段（[`Ahci::prepare`]）在发布设备前由单一初始化任务独占访问。
//! - 运行阶段由上层 `Ahci::state` 自旋锁与 `Ahci::control` 睡眠锁协同串行化寄存器写入序列。

use crate::arch;
use crate::block::BlockError;
use crate::boot::Region;
use crate::time::{self, Duration};
use crate::timer;
use core::hint::spin_loop;
use core::ptr;

/// 单个 AHCI MMIO 寄存器的字节宽度（4 字节 / 32 位）。
pub(super) const REGISTER_BYTES: usize = core::mem::size_of::<u32>();
/// AHCI 规范支持的最大端口数量（32 个端口）。
pub(super) const MAX_PORTS: u32 = 32;
/// 端口 0 控制寄存器区相对于 ABAR 基址的字节偏移（`0x100`）。
pub(super) const PORT_BASE: usize = 0x100;
/// 单个端口控制寄存器区的字节长度（`0x80` = 128 字节）。
pub(super) const PORT_BYTES: usize = 0x80;

/// `CAP.NP`（位 4:0）掩码：指示控制器支持的最大端口索引（实际端口槽数减 1）。
pub(super) const CAP_PORTS_MASK: u32 = 0x1f;
/// `CAP.S64A`（位 31）：控制器是否支持 64 位物理地址 DMA。
pub(super) const CAP_64BIT: u32 = 1 << 31;
/// `CAP.SSS`（位 27）：控制器是否支持交错上电旋转（Staggered Spin-up）。
const CAP_STAGGERED_SPINUP: u32 = 1 << 27;
/// `CAP.FBSS`（位 16）：控制器是否支持基于 FIS 的交换（FIS-based Switching）。
const CAP_FIS_SWITCHING: u32 = 1 << 16;
/// `CAP2.BOH`（位 0）：控制器是否支持 BIOS/OS 所有权交接协议。
const CAP2_BIOS_HANDOFF: u32 = 1;
/// 支持扩展能力寄存器 `CAP2` 与 `BOHC` 的最低 AHCI 版本号（v1.2，`0x0001_0200`）。
const VERSION_1_2: u32 = 0x0001_0200;
/// `GHC.HR`（位 0）：HBA 全局软件复位请求位（写 1 触发复位，硬件完成后自动清零）。
const HOST_RESET: u32 = 1;
/// `BOHC.BOS`（位 0）：BIOS 持有控制器所有权标志。
const BIOS_OWNED: u32 = 1;
/// `BOHC.OOS`（位 1）：操作系统请求/持有控制器所有权标志。
const OS_OWNED: u32 = 1 << 1;
/// `BOHC.BB`（位 4）：BIOS 正在处理所有权交接清理工作的忙标志。
const BIOS_BUSY: u32 = 1 << 4;

/// `PxCMD.ASP`（位 27）：激进休眠电源管理标志（Aggressive Slumber / Partial）。
const CMD_AGGRESSIVE_SLUMBER: u32 = 1 << 27;
/// `PxCMD.ALPE`（位 26）：激进链路电源管理使能位（Aggressive Link Power Management Enable）。
const CMD_LINK_POWER_MANAGEMENT: u32 = 1 << 26;
/// `PxCMD.ICC`（位 31:28）掩码：接口通信控制状态字段（Interface Communication Control）。
const CMD_ICC_MASK: u32 = 0xf << 28;
/// `PxCMD.ICC` 设为 Active（值 `1 << 28`）：请求 SATA PHY 进入活动链路状态。
const CMD_ICC_ACTIVE: u32 = 1 << 28;
/// `PxCMD.POD`（位 2）：设备供电使能位（Power On Device）。
const CMD_POWER_ON: u32 = 1 << 2;
/// `PxCMD.SUD`（位 1）：设备旋转启动位（Spin-Up Device）。
const CMD_SPIN_UP: u32 = 1 << 1;

/// 标准 SATA ATA 磁盘设备的端口签名值（`PxSIG = 0x0000_0101`）。
pub(super) const ATA_SIGNATURE: u32 = 0x0000_0101;
/// `PxSSTS.DET` / `PxSCTL.DET`（位 3:0）掩码：设备检测与 PHY 状态/控制字段。
const SATA_DET_MASK: u32 = 0xf;
/// `PxSSTS.SPD` / `PxSCTL.SPD`（位 7:4）掩码：SATA 接口协商速率字段。
const SATA_SPD_MASK: u32 = 0xf << 4;
/// `PxSSTS.IPM` / `PxSCTL.IPM`（位 11:8）掩码：接口电源管理状态/禁用字段。
const SATA_IPM_MASK: u32 = 0xf << 8;
/// `PxSSTS.DET == 3`：已检测到设备且物理层（PHY）通信已建立。
const SATA_DET_PRESENT: u32 = 3;
/// `PxSCTL.DET == 1`：发起 SATA COMRESET 链路初始化序列。
const SATA_DET_RESET: u32 = 1;
/// `PxSSTS.IPM == 1`（位 11:8 值为 1）：接口处于 Active 活动电源状态。
const SATA_IPM_ACTIVE: u32 = 1 << 8;
/// `PxSCTL.IPM == 3`（位 11:8 值为 3）：禁止接口转入 Partial 或 Slumber 低功耗状态。
const SATA_DISABLE_LOW_POWER: u32 = 3 << 8;
/// COMRESET 期间保持 `SCTL.DET = 1` 的最短持续时间（1 毫秒，满足 SATA 规范要求）。
const LINK_RESET_HOLD: Duration = Duration::from_millis(1);
/// 释放 COMRESET（写 `SCTL.DET = 0`）后等待控制器与 PHY 恢复的间隔时间（200 毫秒）。
const LINK_RESUME_DELAY: Duration = Duration::from_millis(200);

/// HBA 全局寄存器偏移 `0x00`：Host Capabilities (`CAP`)。
pub(super) const CAP: usize = 0x00;
/// HBA 全局寄存器偏移 `0x04`：Global Host Control (`GHC`)。
pub(super) const GHC: usize = 0x04;
/// HBA 全局寄存器偏移 `0x08`：Interrupt Status (`IS`，各端口中断挂起汇总，写 1 清零)。
pub(super) const HOST_IS: usize = 0x08;
/// HBA 全局寄存器偏移 `0x0C`：Ports Implemented (`PI`，已实现端口位图)。
pub(super) const PI: usize = 0x0c;
/// HBA 全局寄存器偏移 `0x10`：AHCI Version (`VS`)。
pub(super) const VERSION: usize = 0x10;
/// HBA 全局寄存器偏移 `0x24`：Host Capabilities Extended (`CAP2`)。
pub(super) const CAP2: usize = 0x24;
/// HBA 全局寄存器偏移 `0x28`：BIOS/OS Handoff Control and Status (`BOHC`)。
pub(super) const BOHC: usize = 0x28;

/// 端口寄存器偏移 `0x00`：Command List Base Address Low (`PxCLB`)。
pub(super) const CLB: usize = 0x00;
/// 端口寄存器偏移 `0x04`：Command List Base Address Upper 32-bits (`PxCLBU`)。
pub(super) const CLBU: usize = 0x04;
/// 端口寄存器偏移 `0x08`：FIS Base Address Low (`PxFB`)。
pub(super) const FB: usize = 0x08;
/// 端口寄存器偏移 `0x0C`：FIS Base Address Upper 32-bits (`PxFBU`)。
pub(super) const FBU: usize = 0x0c;
/// 端口寄存器偏移 `0x10`：Port Interrupt Status (`PxIS`，写 1 清零)。
pub(super) const PORT_IS: usize = 0x10;
/// 端口寄存器偏移 `0x14`：Port Interrupt Enable (`PxIE`)。
pub(super) const IE: usize = 0x14;
/// 端口寄存器偏移 `0x18`：Port Command and Status (`PxCMD`)。
pub(super) const CMD: usize = 0x18;
/// 端口寄存器偏移 `0x20`：Port Task File Data (`PxTFD`，低 8 位为 ATA Status，次 8 位为 Error)。
pub(super) const TFD: usize = 0x20;
/// 端口寄存器偏移 `0x24`：Port Signature (`PxSIG`)。
pub(super) const SIG: usize = 0x24;
/// 端口寄存器偏移 `0x28`：Port SATA Status (`PxSSTS` / `SCR0`)。
pub(super) const SSTS: usize = 0x28;
/// 端口寄存器偏移 `0x2C`：Port SATA Control (`PxSCTL` / `SCR2`)。
pub(super) const SCTL: usize = 0x2c;
/// 端口寄存器偏移 `0x30`：Port SATA Error (`PxSERR` / `SCR1`，写 1 清零)。
pub(super) const SERR: usize = 0x30;
/// 端口寄存器偏移 `0x34`：Port SATA Active (`PxSACT`)。
pub(super) const SACT: usize = 0x34;
/// 端口寄存器偏移 `0x38`：Port Command Issue (`PxCI`)。
pub(super) const CI: usize = 0x38;
/// 端口寄存器偏移 `0x40`：Port FIS-based Switching Control (`PxFBS`)。
pub(super) const FBS: usize = 0x40;

/// `GHC.AE`（位 31）：AHCI 模式使能位（AHCI Enable）。
pub(super) const AE: u32 = 1 << 31;
/// `GHC.IE`（位 1）：HBA 全局中断使能位（Interrupt Enable）。
pub(super) const HOST_IE: u32 = 1 << 1;
/// `PxCMD.ST`（位 0）：启动/停止端口命令列表处理引擎（Start）。
pub(super) const ST: u32 = 1;
/// `PxCMD.FRE`（位 4）：FIS 接收 DMA 使能位（FIS Receive Enable）。
pub(super) const FRE: u32 = 1 << 4;
/// `PxCMD.FR`（位 14）：FIS 接收引擎运行状态只读标志（FIS Receive Running）。
pub(super) const FR: u32 = 1 << 14;
/// `PxCMD.CR`（位 15）：命令列表 DMA 引擎运行状态只读标志（Command List Running）。
pub(super) const CR: u32 = 1 << 15;
/// `PxTFD` 忙状态掩码：包含 `BSY`（位 7）或 `DRQ`（位 3），置位表示设备尚未空闲。
pub(super) const TASK_BUSY: u32 = super::ata::STATUS_BUSY | super::ata::STATUS_DRQ;
/// `PxTFD` 错误状态掩码：包含 `ERR`（位 0）或 `DF`（位 5），置位表示命令执行失败。
pub(super) const TASK_ERROR: u32 = super::ata::STATUS_ERROR | super::ata::STATUS_DEVICE_FAULT;

/// `PxIS.TFES`（位 30）：Task File Error Status（设备返回错误状态 FIS）。
const IRQ_TASK_ERROR: u32 = 1 << 30;
/// `PxIS.HBFS`（位 29）：Host Bus Fatal Error Status（主机总线致命错误，如 PCI 目标中止）。
const IRQ_HOST_BUS_ERROR: u32 = 1 << 29;
/// `PxIS.HBDS`（位 28）：Host Bus Data Error Status（主机总线数据传输校验错误）。
const IRQ_HOST_BUS_DATA_ERROR: u32 = 1 << 28;
/// `PxIS.IFS`（位 27）：Interface Fatal Error Status（SATA 接口致命传输错误）。
const IRQ_INTERFACE_ERROR: u32 = 1 << 27;
/// `PxIS.INFS`（位 26）：Interface Non-fatal Error Status（SATA 接口非致命错误）。
const IRQ_INTERFACE_NONFATAL: u32 = 1 << 26;
/// `PxIS.OFS`（位 24）：Overflow Status（设备传输字节数超出 PRDT 描述的总容量）。
const IRQ_OVERFLOW: u32 = 1 << 24;
/// `PxIS.PRCS`（位 22）：PhyRdy Change Status（PHY 就绪状态发生变化）。
const IRQ_PHY_READY_CHANGE: u32 = 1 << 22;
/// `PxIS.PCS`（位 6）：Port Connect Change Status（端口连接状态变化）。
const IRQ_CONNECT_CHANGE: u32 = 1 << 6;
/// `PxIS.DPS`（位 5）：Descriptor Processed（带中断标志的 PRDT 描述符已传输完毕）。
const IRQ_DESCRIPTOR_DONE: u32 = 1 << 5;
/// `PxIS.UFS`（位 4）：Unknown FIS Interrupt（收到未知类型的 FIS）。
const IRQ_UNKNOWN_FIS: u32 = 1 << 4;
/// `PxIS.SDBS`（位 3）：Set Device Bits Interrupt（收到 Set Device Bits FIS）。
const IRQ_SET_DEVICE_BITS_FIS: u32 = 1 << 3;
/// `PxIS.DSS`（位 2）：DMA Setup FIS Interrupt（收到 DMA Setup FIS）。
const IRQ_DMA_SETUP_FIS: u32 = 1 << 2;
/// `PxIS.PSS`（位 1）：PIO Setup FIS Interrupt（收到 PIO Setup FIS，如 IDENTIFY 数据传输完成）。
const IRQ_PIO_SETUP_FIS: u32 = 1 << 1;
/// `PxIS.DHRS`（位 0）：Device to Host Register FIS Interrupt（收到 D2H Register FIS，常规 DMA 命令完成）。
const IRQ_D2H_REGISTER_FIS: u32 = 1;
/// 指示命令正常推进或完成的端口中断标志集合（`DPS | SDBS | DSS | PSS | DHRS`）。
pub(super) const COMPLETION: u32 = IRQ_DESCRIPTOR_DONE
    | IRQ_SET_DEVICE_BITS_FIS
    | IRQ_DMA_SETUP_FIS
    | IRQ_PIO_SETUP_FIS
    | IRQ_D2H_REGISTER_FIS;
/// 指示控制器、总线、链路或任务文件发生异常的端口中断标志集合。
pub(super) const ERROR: u32 = IRQ_TASK_ERROR
    | IRQ_HOST_BUS_ERROR
    | IRQ_HOST_BUS_DATA_ERROR
    | IRQ_INTERFACE_ERROR
    | IRQ_INTERFACE_NONFATAL
    | IRQ_OVERFLOW
    | IRQ_PHY_READY_CHANGE
    | IRQ_CONNECT_CHANGE
    | IRQ_UNKNOWN_FIS;
/// 端口工作期间开启的全部中断使能掩码（包含完成事件与错误事件）。
pub(super) const INTERRUPTS: u32 = COMPLETION | ERROR;

/// 释放 COMRESET 写回 `SCTL.DET = 0` 时的最大重试次数（部分硬件控制器会忽略紧随复位后的首次写入）。
const LINK_RESUME_TRIES: usize = 5;

/// AHCI 主机适配器（HBA）全局 MMIO 映射窗口句柄。
#[derive(Clone, Copy)]
pub(super) struct Host {
    /// 经 `arch::mmio_address` 转换后的 ABAR 内核虚拟基地址（满足 4 字节对齐）。
    base: usize,
    /// 映射窗口的字节总长度（至少覆盖 `PORT_BASE` 及已声明实现的端口窗口）。
    len: usize,
}

/// 单个 AHCI 端口的 MMIO 寄存器访问句柄。
#[derive(Clone, Copy)]
pub(super) struct Port {
    /// 所属的 HBA 全局窗口句柄。
    host: Host,
    /// 端口硬件编号（`0 .. MAX_PORTS`）。
    index: u32,
}

impl Host {
    /// 校验物理 MMIO 区域边界并构造 HBA 寄存器访问窗口。
    ///
    /// `region` 必须是已经解析完成的 AHCI ABAR 物理地址区间（而非未解引用的 PCI 配置空间地址）。
    ///
    /// # Safety
    /// 1. 调用方必须确保 `region` 对应真实的 AHCI 控制器 ABAR 物理地址范围，且已在内核页表中建立永久强序（Uncached / Device）映射；
    /// 2. 在返回的 `Host` 及派生出的 [`Port`] 存活期间，该控制器由当前驱动独占，固件或其他驱动不会并发修改其寄存器。
    pub(super) unsafe fn new(region: Region) -> Result<Self, BlockError> {
        let len = region
            .end
            .checked_sub(region.start)
            .ok_or(BlockError::InvalidRange)?;
        if region.start == 0 || region.start % REGISTER_BYTES != 0 || len < PORT_BASE {
            return Err(BlockError::InvalidRange);
        }
        let base = arch::mmio_address(region.start);
        base.checked_add(len).ok_or(BlockError::InvalidRange)?;
        Ok(Self { base, len })
    }

    /// 从 HBA 全局窗口指定字节偏移 `offset` 处读取一个 32 位小端寄存器。
    pub(super) fn read(self, offset: usize) -> u32 {
        assert!(offset % REGISTER_BYTES == 0 && offset <= self.len - REGISTER_BYTES);
        // SAFETY:
        // 1. `Host::new` 的调用契约保证了 `[self.base, self.base + self.len)` 处于有效的设备 MMIO 映射内；
        // 2. 上方 `assert!` 保证了 `offset` 满足 4 字节对齐且未越过窗口上界。
        u32::from_le(unsafe { ptr::read_volatile((self.base + offset) as *const u32) })
    }

    /// 向 HBA 全局窗口指定字节偏移 `offset` 处写入一个 32 位小端寄存器值 `value`。
    pub(super) fn write(self, offset: usize, value: u32) {
        assert!(offset % REGISTER_BYTES == 0 && offset <= self.len - REGISTER_BYTES);
        // SAFETY:
        // 1. `Host::new` 的调用契约保证了 `[self.base, self.base + self.len)` 处于可写的设备 MMIO 映射内；
        // 2. 上方 `assert!` 保证了 `offset` 满足 4 字节对齐且未越过窗口上界，并发互斥由上层准备期独占或 `state` 自旋锁保证。
        unsafe { ptr::write_volatile((self.base + offset) as *mut u32, value.to_le()) };
    }

    /// 获取指定索引 `index` 的端口控制寄存器窗口（每个端口占 128 字节）。
    pub(super) fn port(self, index: u32) -> Result<Port, BlockError> {
        if MAX_PORTS <= index || self.len < PORT_BASE + (index as usize + 1) * PORT_BYTES {
            return Err(BlockError::InvalidRange);
        }
        Ok(Port { host: self, index })
    }

    /// 执行 AHCI HBA 控制器初始化：开启 AHCI 模式、完成 BIOS/OS 所有权交接、全局复位并停止所有已实现端口。
    ///
    /// 返回 `(cap, ports)`，即控制器的 `CAP` 能力字与校验通过的已实现端口位图 `PI`。
    pub(super) fn initialize(self, firmware_ports: Option<u32>) -> Result<(u32, u32), BlockError> {
        // Step 1: 开启 AHCI 模式（GHC.AE = 1）并屏蔽全局中断（GHC.IE = 0）。
        self.write(GHC, (self.read(GHC) | AE) & !HOST_IE);
        if self.read(GHC) & AE == 0 {
            return Err(BlockError::DeviceFault);
        }

        // Step 2: 读取 CAP 与 PI 寄存器，结合平台固件提供的端口掩码校验已实现端口范围的合法性。
        let cap = self.read(CAP);
        if cap == u32::MAX {
            return Err(BlockError::DeviceFault);
        }
        let last_port = cap & CAP_PORTS_MASK;
        self.port(last_port)?;
        let legal = u32::MAX >> (MAX_PORTS - 1 - last_port);
        let actual = self.read(PI);
        let ports = match firmware_ports {
            Some(ports) if ports != 0 && ports & !legal == 0 => {
                if actual != 0 && actual != ports {
                    return Err(BlockError::InvalidRange);
                }
                ports
            }
            Some(_) => return Err(BlockError::InvalidRange),
            None if actual != 0 && actual & !legal == 0 => actual,
            None => return Err(BlockError::NotReady),
        };

        // Step 3: 若控制器版本 >= 1.2 且支持 BIOS/OS 交接（CAP2.BOH），置位 OOS 并等待 BIOS 释放所有权。
        if VERSION_1_2 <= self.read(VERSION) && self.read(CAP2) & CAP2_BIOS_HANDOFF != 0 {
            self.write(BOHC, self.read(BOHC) | OS_OWNED);
            if !poll(Duration::from_secs(2), || {
                self.read(BOHC) & (BIOS_OWNED | BIOS_BUSY) == 0
            }) {
                return Err(BlockError::Busy);
            }
        }

        // Step 4: 触发 HBA 全局复位（GHC.HR = 1），等待复位完成，并回写恢复固件配置的 CAP 与 PI 寄存器。
        self.write(GHC, AE | HOST_RESET);
        if !poll(Duration::from_secs(1), || self.read(GHC) & HOST_RESET == 0) {
            return Err(BlockError::DeviceFault);
        }
        self.write(GHC, AE);
        self.write(CAP, cap);
        self.write(PI, ports);
        if self.read(GHC) & AE == 0 || self.read(PI) != ports {
            return Err(BlockError::DeviceFault);
        }

        // Step 5: 遍历所有已实现端口，关闭端口中断、停止命令与 FIS 接收 DMA 引擎，并清空残留中断状态。
        for index in 0..=last_port {
            if ports & (1 << index) == 0 {
                continue;
            }
            let port = self.port(index)?;
            port.write(IE, 0);
            if !port.stop_polled() {
                return Err(BlockError::StopUnconfirmed);
            }
            port.clear_pending();
        }
        Ok((cap, ports))
    }
}

impl Port {
    /// 读取当前端口内偏移 `offset`（`< 0x80`）处的 32 位寄存器。
    pub(super) fn read(self, offset: usize) -> u32 {
        assert!(offset < PORT_BYTES);
        self.host
            .read(PORT_BASE + self.index as usize * PORT_BYTES + offset)
    }

    /// 向当前端口内偏移 `offset`（`< 0x80`）处的 32 位寄存器写入 `value`。
    pub(super) fn write(self, offset: usize, value: u32) {
        assert!(offset < PORT_BYTES);
        self.host
            .write(PORT_BASE + self.index as usize * PORT_BYTES + offset, value);
    }

    /// 检查当前端口的 SATA 物理链路是否已建立并处于 Active 状态，且设备签名为标准 ATA 磁盘。
    pub(super) fn connected(self) -> bool {
        self.read(SSTS) & (SATA_IPM_MASK | SATA_DET_MASK) == SATA_IPM_ACTIVE | SATA_DET_PRESENT
            && self.read(SIG) == ATA_SIGNATURE
    }

    /// 为端口设备上电、启动旋转（若支持交错上电），并禁用 SATA 链路低功耗休眠模式。
    pub(super) fn power_up(self, cap: u32) {
        let mut command = self.read(CMD);
        command &= !(CMD_ICC_MASK | CMD_LINK_POWER_MANAGEMENT | CMD_AGGRESSIVE_SLUMBER);
        command |= CMD_ICC_ACTIVE | CMD_POWER_ON;
        if cap & CAP_STAGGERED_SPINUP != 0 {
            command |= CMD_SPIN_UP;
        }
        self.write(CMD, command);
        self.write(
            SCTL,
            (self.read(SCTL) & !SATA_IPM_MASK) | SATA_DISABLE_LOW_POWER,
        );
        if cap & CAP_FIS_SWITCHING != 0 {
            self.write(FBS, 0);
        }
        let _ = self.read(CMD);
    }

    /// 在端口停止状态下配置命令列表（`CLB`/`CLBU`）与接收 FIS（`FB`/`FBU`）的 DMA 地址，并开启 FIS 接收引擎（`CMD.FRE`）。
    ///
    /// 调用方必须保持 `list` 与 `fis` 对应的 DMA 物理页有效，直到后续确认端口 `CR == 0` 且 `FR == 0`。
    pub(super) fn start_fis_rx(self, list: u64, fis: u64) {
        self.write(CLBU, (list >> 32) as u32);
        self.write(CLB, list as u32);
        self.write(FBU, (fis >> 32) as u32);
        self.write(FB, fis as u32);
        self.write(CMD, self.read(CMD) | FRE);
        let _ = self.read(CMD);
    }

    /// 在可睡眠任务上下文中发起 SATA COMRESET 链路复位序列，并等待 PHY 重新建立连接。
    ///
    /// 执行期间需保持 FIS 接收引擎（`FRE`）开启以便接收设备复位后的初始 D2H Register FIS（更新签名与任务文件），
    /// 同时保持命令引擎（`ST`）处于停止状态。
    pub(super) fn comreset(self) -> Result<(), BlockError> {
        // Step 1: 置位 SCTL.DET = 1 发起 COMRESET，保持至少 1ms。
        let sctl = self.read(SCTL);
        self.write(
            SCTL,
            (sctl & SATA_SPD_MASK) | SATA_DISABLE_LOW_POWER | SATA_DET_RESET,
        );
        let _ = self.read(SCTL);
        timer::sleep(LINK_RESET_HOLD);

        // Step 2: 写回 SCTL.DET = 0 结束复位信号，每次尝试等待 200ms 后读回确认（兼容忽略首次写入的控制器）。
        let mut restored = sctl;
        for _ in 0..LINK_RESUME_TRIES {
            self.write(SCTL, (sctl & SATA_SPD_MASK) | SATA_DISABLE_LOW_POWER);
            timer::sleep(LINK_RESUME_DELAY);
            restored = self.read(SCTL);
            if restored & (SATA_IPM_MASK | SATA_DET_MASK) == SATA_DISABLE_LOW_POWER {
                break;
            }
        }
        if restored & (SATA_IPM_MASK | SATA_DET_MASK) != SATA_DISABLE_LOW_POWER {
            return Err(BlockError::NotReady);
        }

        // Step 3: 轮询等待 SSTS.DET 变为 3（已检测到设备且 PHY 通信恢复）。
        if !poll(Duration::from_secs(1), || {
            self.read(SSTS) & SATA_DET_MASK == SATA_DET_PRESENT
        }) {
            return Err(BlockError::NotReady);
        }
        Ok(())
    }

    /// 检查端口任务文件数据寄存器（`PxTFD`）中的 ATA `BSY` 位是否已清零（即设备已完成复位初始化）。
    pub(super) fn taskfile_ready(self) -> bool {
        self.read(TFD) & super::ata::STATUS_BUSY == 0
    }

    /// 通过写 1 清零（W1C）清除当前端口挂起的 `PxSERR` 错误位、`PxIS` 中断状态位以及 HBA 汇总位。
    pub(super) fn clear_pending(self) {
        self.write(SERR, self.read(SERR));
        self.write(PORT_IS, self.read(PORT_IS));
        self.ack_host();
    }

    /// 通过写 1 清零（W1C）仅应答 HBA 全局 `HOST_IS` 寄存器中属于当前端口的位，不影响其他端口的中断挂起状态。
    pub(super) fn ack_host(self) {
        self.host.write(HOST_IS, 1 << self.index);
    }

    /// 在设备初始化/探测阶段通过忙等轮询停止端口 DMA 引擎。
    ///
    /// 严格按照“先清 `ST` 等待 `CR == 0`，再清 `FRE` 等待 `FR == 0`”的顺序推进；
    /// 若任一阶段超时未停稳则返回 `false`。
    pub(super) fn stop_polled(self) -> bool {
        self.write(IE, 0);
        self.write(CMD, self.read(CMD) & !ST);
        if !poll(Duration::from_millis(500), || self.read(CMD) & CR == 0) {
            return false;
        }
        self.write(CMD, self.read(CMD) & !FRE);
        poll(Duration::from_secs(1), || self.read(CMD) & FR == 0)
    }

    /// 开启 HBA 全局中断使能位（`GHC.IE`），并读回寄存器以刷新 MMIO 写缓冲。
    pub(super) fn enable_host_irq(self) {
        self.host.write(GHC, self.host.read(GHC) | AE | HOST_IE);
        let _ = self.host.read(GHC);
    }
}

/// 在给定的超时时间 `timeout` 内忙等轮询谓词 `ready`，仅用于初始化/探测阶段的短时寄存器状态等待。
pub(super) fn poll(timeout: Duration, mut ready: impl FnMut() -> bool) -> bool {
    let deadline = time::now()
        .checked_add(timeout)
        .expect("AHCI deadline overflow");
    loop {
        if ready() {
            return true;
        }
        if deadline <= time::now() {
            return false;
        }
        spin_loop();
    }
}

/// 判断端口 `PxSSTS` 状态值是否明确表示无设备连接（`DET == 0`）。
///
/// 若 `DET` 为其他非就绪值（如 `1` 仅检测到设备但 PHY 未通信、`4` PHY 离线或 `0xFFFFFFFF` MMIO 失效），
/// 则返回 `false`，由上层作为硬件未就绪报错而非正常缺盘。
pub(super) fn media_absent(status: u32) -> bool {
    status & SATA_DET_MASK == 0
}
