//! IEEE 802.3 Clause 22 MDIO 管理总线与裕太微（Motorcomm）YT8531 / YT8511 千兆 PHY 驱动模块。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `drivers::dwmac` 内部，供外层 `Dwmac` 在初始化准备过程与后台定时链路采样过程中调用，向下依赖 `Registers` 提供的 MAC MMIO 读写能力以及 `timer::sleep` 可睡眠延时。
//! 主要职责包括：
//! 1. 封装 DWMAC4 与 DWMAC1000 两代控制器的 Clause 22 MDIO 地址与数据寄存器事务；
//! 2. 识别并软复位 VisionFive 2 板载 YT8531 与 LS2K1000LA 板载 YT8511 千兆以太网 PHY；
//! 3. 根据板级设备树（FDT）规范化的接口模式、RGMII 时钟延时、驱动电流及反相参数配置厂商扩展寄存器；
//! 4. 启动铜缆自协商并周期性采样协商出的最高公共速率与双工模式。
//!
//! 许可与修改声明见本目录 `LICENSE`。
//!
//! ## MDIO 地址命令寄存器位域与厂商扩展页机制
//!
//! ```text
//! DWMAC4 MDIO Address 寄存器 (偏移 0x200)：
//!  31      26 25    21 20    16 15  12 11   8 7  4 3   2  1    0
//! +----------+--------+--------+------+------+----+-----+----+----+
//! | Reserved |   PA   |  RDA   | Rsvd |  CR  |Rsvd| GOC |C45E| GB |
//! +----------+--------+--------+------+------+----+-----+----+----+
//!   PA[25:21]: 5 位 PHY 地址        RDA[20:16]: 5 位 Clause 22 寄存器地址
//!   CR[11:8] : CSR 时钟分频档位     GOC[3:2]  : 01 = 写, 11 = 读     GB[0]: 1 = 忙
//!
//! DWMAC1000 GMII Address 寄存器 (偏移 0x010)：
//!  31                16 15    11 10     6  5  4    2  1    0
//! +--------------------+--------+--------+---+------+----+----+
//! |      Reserved      |   PA   |   GR   |Rsv|  CR  | GW | GB |
//! +--------------------+--------+--------+---+------+----+----+
//!   PA[15:11]: 5 位 PHY 地址        GR[10:6]  : 5 位 Clause 22 寄存器地址
//!   CR[4:2]  : CSR 时钟分频档位     GW[1]     : 1 = 写, 0 = 读       GB[0]: 1 = 忙
//!
//! Motorcomm 厂商扩展寄存器两步间接访问协议：
//!   Step 1: 向 Clause 22 寄存器 0x1E (YTPHY_PAGE_SELECT) 写入 16 位扩展寄存器地址；
//!   Step 2: 读写 Clause 22 寄存器 0x1F (YTPHY_PAGE_DATA) 访问目标扩展寄存器数据。
//! ```
//!
//! ## 时钟前提与并发串行化契约
//! - **RXC 输入时钟保持不变量**：Synopsys DWMAC 的 DMA 软件复位（`SWR`）同步逻辑依赖 PHY 持续输出 RGMII 接收参考时钟（RXC）。因此 `Phy::prepare` 必须在首次触发 MAC DMA 软复位之前完成，解除 PHY 掉电/隔离状态，并关闭无网线载波时的自动休眠与 RXC 时钟门控，确保即便拔出网线也能随时完成 DMA 软复位。
//! - **单执行流无锁串行访问**：`Phy` 仅在设备未公开前的 `Dwmac::prepare` 任务中，或运行期唯一的 CPU0 `progress` 工作队列任务中访问；硬件中断处理函数（ISR）绝不触碰 MDIO 总线。因此 `0x1E` 选页与 `0x1F` 数据读写的复合扩展事务天然串行，不会被其他 MDIO 请求穿插破坏。

use crate::{
    boot::{GmacResources, PhyMode},
    drivers::dwmac::{Generation, GmacError, Registers},
    time::{self, Duration},
    timer,
};

const YT8511: u32 = 0x0000_010a;
const YT8531: u32 = 0x4f51_e91b;
pub(crate) const PHY_ADDRESS_COUNT: u32 = 1 << 5;
pub(super) const SPEED_10: u16 = 10;
pub(super) const SPEED_100: u16 = 100;
pub(super) const SPEED_1000: u16 = 1000;

const MII_BMCR: u8 = 0;
const MII_BMSR: u8 = 1;
const MII_PHYSID1: u8 = 2;
const MII_PHYSID2: u8 = 3;
const MII_ADVERTISE: u8 = 4;
const MII_LPA: u8 = 5;
const MII_CTRL1000: u8 = 9;
const MII_STAT1000: u8 = 10;
const MII_ESTATUS: u8 = 15;

const BMCR_RESET: u16 = 1 << 15;
const BMCR_LOOPBACK: u16 = 1 << 14;
const BMCR_ANENABLE: u16 = 1 << 12;
const BMCR_POWER_DOWN: u16 = 1 << 11;
const BMCR_ISOLATE: u16 = 1 << 10;
const BMCR_ANRESTART: u16 = 1 << 9;
const BMSR_LINK_STATUS: u16 = 1 << 2;
const BMSR_ANEG_CAPABLE: u16 = 1 << 3;
const BMSR_ANEG_COMPLETE: u16 = 1 << 5;
const BMSR_EXTENDED_STATUS: u16 = 1 << 8;
const BMSR_TO_ADVERTISE_SHIFT: u32 = 6;
const ADVERTISE_CSMA: u16 = 1;
const ADVERTISE_10HALF: u16 = 1 << 5;
const ADVERTISE_10FULL: u16 = 1 << 6;
const ADVERTISE_100HALF: u16 = 1 << 7;
const ADVERTISE_100FULL: u16 = 1 << 8;
const ADVERTISE_COPPER: u16 =
    ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL;
const ADVERTISE_SELECTOR_MASK: u16 = 0x1f;
const ADVERTISE_PAUSE: u16 = 1 << 10;
const ADVERTISE_ASYM_PAUSE: u16 = 1 << 11;
const ADVERTISE_CONFIG_MASK: u16 =
    ADVERTISE_SELECTOR_MASK | ADVERTISE_COPPER | ADVERTISE_PAUSE | ADVERTISE_ASYM_PAUSE;
const LPA_REMOTE_FAULT: u16 = 1 << 13;
const ADVERTISE_1000HALF: u16 = 1 << 8;
const ADVERTISE_1000FULL: u16 = 1 << 9;
const ADVERTISE_1000: u16 = ADVERTISE_1000HALF | ADVERTISE_1000FULL;
const ESTATUS_TO_ADVERTISE_SHIFT: u32 = 4;
const LPA_1000_MASTER_SLAVE_FAIL: u16 = 1 << 15;
const LPA_1000_TO_ADVERTISE_SHIFT: u32 = 2;

const PHY_RESET_TIMEOUT: Duration = Duration::from_millis(500);
const MDIO_TIMEOUT: Duration = Duration::from_millis(20);
const MDIO_BUSY: u32 = 1;

const DWMAC4_MDIO_ADDRESS: usize = 0x200;
const DWMAC4_MDIO_DATA: usize = 0x204;
const DWMAC4_MDIO_PHY_SHIFT: u32 = 21;
const DWMAC4_MDIO_REGISTER_SHIFT: u32 = 16;
const DWMAC4_MDIO_CLOCK_SHIFT: u32 = 8;
const DWMAC4_MDIO_WRITE: u32 = 1 << 2;
const DWMAC4_MDIO_READ: u32 = 3 << 2;

const DWMAC1000_MDIO_ADDRESS: usize = 0x10;
const DWMAC1000_MDIO_DATA: usize = 0x14;
const DWMAC1000_MDIO_PHY_SHIFT: u32 = 11;
const DWMAC1000_MDIO_REGISTER_SHIFT: u32 = 6;
const DWMAC1000_MDIO_CLOCK_SHIFT: u32 = 2;
const DWMAC1000_MDIO_WRITE: u32 = 1 << 1;

const CSR_60_100M: u32 = 0;
pub(super) const CSR_100_150M: u32 = 1;
const CSR_20_35M: u32 = 2;
const CSR_35_60M: u32 = 3;
const CSR_150_250M: u32 = 4;
const CSR_250_300M: u32 = 5;

const YTPHY_PAGE_SELECT: u8 = 0x1e;
const YTPHY_PAGE_DATA: u8 = 0x1f;
const YT8511_EXT_CLOCK_GATE: u16 = 0x0c;
const YT8511_EXT_DELAY_DRIVE: u16 = 0x0d;
const YT8511_EXT_SLEEP_CONTROL: u16 = 0x27;
const YT8511_CLOCK_125M: u16 = (1 << 2) | (1 << 1);
const YT8511_CLOCK_SELECT_MASK: u16 = 3 << 1;
const YT8511_PLL_ON_SLEEP: u16 = 1 << 14;
const YT8511_DELAY_RX: u16 = 1;
const YT8511_DELAY_GE_TX_MASK: u16 = 0xf << 4;
const YT8511_DELAY_GE_TX_ENABLE: u16 = 0xf << 4;
const YT8511_DELAY_GE_TX_DISABLE: u16 = 2 << 4;
const YT8511_DELAY_FE_TX_MASK: u16 = 0xf << 12;
const YT8511_DELAY_FE_TX_ENABLE: u16 = 0xf << 12;
const YT8511_DELAY_FE_TX_DISABLE: u16 = 2 << 12;

const YTPHY_CHIP_CONFIG: u16 = 0xa001;
const YTPHY_RX_CLOCK_DELAY_ENABLE: u16 = 1 << 8;
const YTPHY_RGMII_CONFIG1: u16 = 0xa003;
const YTPHY_TX_CLOCK_INVERTED: u16 = 1 << 14;
const YTPHY_RX_DELAY_SHIFT: u32 = 10;
const YTPHY_DELAY_MASK: u16 = 0xf;
const YTPHY_RX_DELAY_MASK: u16 = YTPHY_DELAY_MASK << YTPHY_RX_DELAY_SHIFT;

const YT8531_LDO_VOLTAGE_SHIFT: u32 = 4;
const YT8531_LDO_VOLTAGE_MASK: u16 = 3;
const YT8531_LDO_3V3: u16 = 0;
const YT8531_LDO_1V8: u16 = 2;
const YTPHY_PAD_DRIVE_STRENGTH: u16 = 0xa010;
const YT8531_RX_CLOCK_DRIVE_SHIFT: u32 = 13;
const YT8531_RX_CLOCK_DRIVE_MASK: u16 = 7 << YT8531_RX_CLOCK_DRIVE_SHIFT;
const YT8531_RX_DATA_DRIVE_HIGH: u16 = 1 << 12;
const YT8531_RX_DATA_DRIVE_LOW_SHIFT: u32 = 4;
const YT8531_RX_DATA_DRIVE_LOW_MASK: u16 = 3 << YT8531_RX_DATA_DRIVE_LOW_SHIFT;
const YT8531_RX_DATA_DRIVE_HIGH_SHIFT: u32 = 10;
const YT8531_DRIVE_CODE_HIGH: u16 = 4;
const YT8531_DRIVE_CODE_LOW: u16 = 3;
const YT8531_RX_DRIVE_DEFAULT: u16 = 3;
const YTPHY_SLEEP_CONTROL: u16 = 0x27;
const YTPHY_SLEEP_ENABLE: u16 = 1 << 15;
const YTPHY_CLOCK_GATING: u16 = 0x0c;
const YTPHY_RX_CLOCK_GATE: u16 = 1 << 12;

const YTPHY_DELAY_STEP_PS: u32 = 150;
const YTPHY_DELAY_MAX_PS: u32 = YTPHY_DELAY_MASK as u32 * YTPHY_DELAY_STEP_PS;
const YTPHY_RX_EXTRA_DELAY_PS: u32 = 1900;
const YTPHY_DELAY_DEFAULT_PS: u32 = 13 * YTPHY_DELAY_STEP_PS;

/// 自协商建立的以太网物理链路参数。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct Link {
    /// 协商达成的链路速率，单位为 Mbps（取值为 `10`、`100` 或 `1000`）。
    pub(super) speed: u16,
    /// 是否工作在全双工模式（`true` 为全双工，`false` 为半双工）。
    pub(super) full_duplex: bool,
}

/// Clause 22 MDIO 管理通道与板载裕太微 PHY 控制器实例。
///
/// 该结构体的所有方法均包含基于 `timer::sleep` 的有界等待，只能在可睡眠任务上下文（初始化任务或唯一的 `progress` Work）中串行调用，严禁在硬中断上下文中访问。
pub(super) struct Phy {
    /// 所属 MAC 控制器的 MMIO 寄存器访问句柄，用于发起底层 MDIO 读写事务。
    registers: Registers,
    /// PHY 在 MDIO 管理总线上的 5 位物理地址（有效范围 `0..32`）。
    address: u8,
    /// 根据 MAC CSR 输入时钟频率选定的 MDC 分频编码值，保证 MDC 时钟不超过 2.5 MHz。
    clock: u32,
    /// 从 `MII_PHYSID1` 与 `MII_PHYSID2` 读出的 32 位厂商与型号标识码（如 `YT8531` 或 `YT8511`）。
    id: u32,
    /// 链路速率变化时是否需要动态调整 YT8531 的 RGMII TX 时钟反相配置。
    adjust_tx: bool,
    /// 分别对应 10M / 100M / 1000M 三档速率（下标 `0` / `1` / `2`）下是否启用 TX 时钟相位反转。
    inverted: [bool; 3],
}

impl Phy {
    /// 在首次触发 MAC DMA 软件复位前探测、软复位并初始化板载 PHY（运行于可睡眠准备任务上下文）。
    ///
    /// 调用前板级时钟与外设复位须已由 `board_setup` 启用。本函数会解除 PHY 的掉电与 MII 隔离状态，并配置芯片在无网线载波时继续维持 RGMII RXC 时钟输出，确保后续 MAC DMA 软件复位不会因缺少参考时钟而超时挂起。
    pub(super) fn prepare(
        registers: Registers,
        resources: &GmacResources,
        csr_divider: u32,
    ) -> Result<Self, GmacError> {
        if PHY_ADDRESS_COUNT <= resources.phy_address || CSR_250_300M < csr_divider {
            return Err(GmacError::Resource("PHY address or CSR divider is invalid"));
        }
        let mut phy = Self {
            registers,
            address: resources.phy_address as u8,
            clock: csr_divider,
            id: 0,
            adjust_tx: resources.phy_tx_clock_adjust,
            inverted: resources.phy_tx_inverted,
        };
        // Step 1: 读取 32 位 PHY 硬件标识符，校验其与当前平台代数的预期板载芯片（YT8531 / YT8511）一致。
        phy.id =
            (u32::from(phy.read(MII_PHYSID1)?) << u16::BITS) | u32::from(phy.read(MII_PHYSID2)?);
        let expected = match registers.generation {
            Generation::Dwmac4 => YT8531,
            Generation::Dwmac1000 => YT8511,
        };
        if phy.id != expected {
            return Err(GmacError::Resource("selected GMAC PHY ID is unsupported"));
        }

        // Step 2: 发起 Clause 22 软件复位并轮询等待完成；写入仅含 BMCR_RESET 的值会同时清除掉电（POWER_DOWN）与隔离（ISOLATE）位。
        // MDIO 管理时钟由 MAC 侧 MDC 独立驱动，因此复位完成不依赖外部网线连接或自协商状态。
        phy.write(MII_BMCR, BMCR_RESET)?;
        let deadline = time::now().checked_add(PHY_RESET_TIMEOUT).unwrap();
        while phy.read(MII_BMCR)? & BMCR_RESET != 0 {
            if deadline <= time::now() {
                return Err(GmacError::Io("PHY reset timed out"));
            }
            timer::sleep(Duration::from_millis(1));
        }

        // Step 3: 应用芯片专属的 RGMII 时钟延时、驱动强度及防断钟睡眠配置，最后写入能力通告并重启自协商。
        match phy.id {
            YT8531 => phy.configure_yt8531(resources)?,
            YT8511 => phy.configure_yt8511(resources)?,
            _ => unreachable!(),
        }
        phy.advertise()?;
        crate::klog!(
            crate::klog::Level::Info,
            "GMAC PHY id={:#010x} address={} csr_divider={}",
            phy.id,
            phy.address,
            csr_divider
        );
        Ok(phy)
    }

    /// 采样当前物理链路状态与自协商结果（运行于 CPU0 `progress` 工作队列上下文）。
    ///
    /// 若网线未插好或自协商尚未完成则返回 `Ok(None)`；协商完成时按优先级返回本端与对端共同支持的最高速率及双工模式。
    pub(super) fn sample(&self) -> Result<Option<Link>, GmacError> {
        // Step 1: 连续读取两次 BMSR；BMSR 的链路状态位为低电平锁存（Latch-low），首次读取清除历史掉线锁存，第二次获取实时载波状态。
        self.read(MII_BMSR)?;
        let status = self.read(MII_BMSR)?;
        if status == 0 || status == u16::MAX {
            return Err(GmacError::Io("PHY status is unavailable"));
        }
        if status & BMSR_LINK_STATUS == 0 {
            return Ok(None);
        }
        let control = self.read(MII_BMCR)?;
        if control & BMCR_ANENABLE == 0 {
            return Err(GmacError::Io("PHY autonegotiation unexpectedly disabled"));
        }
        if status & BMSR_ANEG_COMPLETE == 0 {
            return Ok(None);
        }

        // Step 2: 读取本端通告能力与对端链路伙伴能力（含千兆控制/状态寄存器），检查远端故障与主从协商错误后求取最高公共模式。
        let local = self.read(MII_ADVERTISE)?;
        let partner = self.read(MII_LPA)?;
        if partner & LPA_REMOTE_FAULT != 0 {
            return Err(GmacError::Io("PHY partner reports remote fault"));
        }
        let gigabit = self.read(MII_CTRL1000)?;
        let peer_gigabit = self.read(MII_STAT1000)?;
        if peer_gigabit & LPA_1000_MASTER_SLAVE_FAIL != 0 {
            return Err(GmacError::Io("PHY master slave resolution failed"));
        }
        negotiated(local, partner, gigabit, peer_gigabit)
            .map(Some)
            .ok_or(GmacError::Io("PHY has no common negotiated mode"))
    }

    /// 在 MAC 收发已暂停的前提下，根据最新协商速率更新 YT8531 的 RGMII 发送时钟反相配置。
    pub(super) fn configure_link(&self, link: Link) -> Result<(), GmacError> {
        if self.adjust_tx {
            let index = match link.speed {
                SPEED_10 => 0,
                SPEED_100 => 1,
                SPEED_1000 => 2,
                _ => unreachable!(),
            };
            self.modify_extended(
                YTPHY_RGMII_CONFIG1,
                YTPHY_TX_CLOCK_INVERTED,
                if self.inverted[index] {
                    YTPHY_TX_CLOCK_INVERTED
                } else {
                    0
                },
            )?;
        }
        Ok(())
    }

    /// 根据硬件实际支持的铜缆速率配置自协商通告寄存器（关闭驱动未实现的流控帧通告），并触发重新自协商。
    fn advertise(&self) -> Result<(), GmacError> {
        let status = self.read(MII_BMSR)?;
        if status & BMSR_ANEG_CAPABLE == 0 {
            return Err(GmacError::Resource("PHY does not support autonegotiation"));
        }
        let advertisement =
            ADVERTISE_CSMA | ((status >> BMSR_TO_ADVERTISE_SHIFT) & ADVERTISE_COPPER);
        if advertisement & ADVERTISE_COPPER == 0 {
            return Err(GmacError::Resource("PHY has no supported copper mode"));
        }

        self.write(
            MII_ADVERTISE,
            (self.read(MII_ADVERTISE)? & !ADVERTISE_CONFIG_MASK) | advertisement,
        )?;
        let extended = if status & BMSR_EXTENDED_STATUS != 0 {
            self.read(MII_ESTATUS)?
        } else {
            0
        };
        self.write(
            MII_CTRL1000,
            (self.read(MII_CTRL1000)? & !ADVERTISE_1000)
                | ((extended >> ESTATUS_TO_ADVERTISE_SHIFT) & ADVERTISE_1000),
        )?;

        self.write(
            MII_BMCR,
            (self.read(MII_BMCR)? & !(BMCR_POWER_DOWN | BMCR_ISOLATE | BMCR_LOOPBACK))
                | BMCR_ANENABLE
                | BMCR_ANRESTART,
        )
    }

    /// 配置 YT8511 扩展寄存器：启用 125 MHz 参考时钟与 RGMII 收发延时档位，强制休眠时保持 PLL 运行，并在退出前恢复原先的扩展页选择寄存器值。
    fn configure_yt8511(&self, resources: &GmacResources) -> Result<(), GmacError> {
        let (rx, tx) = delay_directions(resources.phy_mode)?;
        if resources.rx_delay_ps.is_some()
            || resources.tx_delay_ps.is_some()
            || resources.phy_clock_drive_ua.is_some()
            || resources.phy_data_drive_ua.is_some()
            || resources.phy_tx_clock_adjust
            || resources.phy_tx_inverted.iter().any(|value| *value)
        {
            return Err(GmacError::Resource(
                "YT8511 does not support requested numeric PHY settings",
            ));
        }

        // 保存当前扩展页选择，通过闭包配置 125M 时钟、千兆/百兆 TX 延时与休眠 PLL 保活，最后无条件恢复页选择寄存器。
        let old = self.read(YTPHY_PAGE_SELECT)?;
        let result = (|| {
            self.write(YTPHY_PAGE_SELECT, YT8511_EXT_CLOCK_GATE)?;
            let value = self.read(YTPHY_PAGE_DATA)?;
            self.write(
                YTPHY_PAGE_DATA,
                (value & !(YT8511_DELAY_GE_TX_MASK | YT8511_CLOCK_SELECT_MASK | YT8511_DELAY_RX))
                    | YT8511_CLOCK_125M
                    | if rx { YT8511_DELAY_RX } else { 0 }
                    | if tx {
                        YT8511_DELAY_GE_TX_ENABLE
                    } else {
                        YT8511_DELAY_GE_TX_DISABLE
                    },
            )?;
            self.write(YTPHY_PAGE_SELECT, YT8511_EXT_DELAY_DRIVE)?;
            self.write(
                YTPHY_PAGE_DATA,
                (self.read(YTPHY_PAGE_DATA)? & !YT8511_DELAY_FE_TX_MASK)
                    | if tx {
                        YT8511_DELAY_FE_TX_ENABLE
                    } else {
                        YT8511_DELAY_FE_TX_DISABLE
                    },
            )?;
            self.write(YTPHY_PAGE_SELECT, YT8511_EXT_SLEEP_CONTROL)?;
            self.write(
                YTPHY_PAGE_DATA,
                self.read(YTPHY_PAGE_DATA)? | YT8511_PLL_ON_SLEEP,
            )
        })();
        let restore = self.write(YTPHY_PAGE_SELECT, old);
        result.and(restore)
    }

    /// 配置 YT8531 扩展寄存器：设定 RGMII 收发精确皮秒延时、按 LDO 供电电压配置引脚驱动电流，并关闭自动休眠与无载波时的 RXC 接收时钟门控。
    fn configure_yt8531(&self, resources: &GmacResources) -> Result<(), GmacError> {
        // Step 1: 计算 RGMII RX/TX 时钟延时档位及 1900ps 额外偏移开关，写入芯片配置与 RGMII 配置寄存器。
        let (rx_enabled, tx_enabled) = delay_directions(resources.phy_mode)?;
        let (rx, extra) = delay_value(
            resources.rx_delay_ps.unwrap_or(YTPHY_DELAY_DEFAULT_PS),
            true,
        )?;
        let (tx, _) = delay_value(
            resources.tx_delay_ps.unwrap_or(YTPHY_DELAY_DEFAULT_PS),
            false,
        )?;
        self.modify_extended(
            YTPHY_CHIP_CONFIG,
            YTPHY_RX_CLOCK_DELAY_ENABLE,
            if rx_enabled && extra {
                YTPHY_RX_CLOCK_DELAY_ENABLE
            } else {
                0
            },
        )?;
        self.modify_extended(
            YTPHY_RGMII_CONFIG1,
            YTPHY_RX_DELAY_MASK | YTPHY_DELAY_MASK,
            if rx_enabled {
                rx << YTPHY_RX_DELAY_SHIFT
            } else {
                0
            } | if tx_enabled { tx } else { 0 },
        )?;

        // Step 2: 读取芯片 LDO 供电电压引脚采样值，换算并配置 RX 时钟与数据线的焊盘驱动电流档位。
        let voltage = (self.read_extended(YTPHY_CHIP_CONFIG)? >> YT8531_LDO_VOLTAGE_SHIFT)
            & YT8531_LDO_VOLTAGE_MASK;
        let clock = drive_strength(voltage, resources.phy_clock_drive_ua)?;
        let data = drive_strength(voltage, resources.phy_data_drive_ua)?;
        self.modify_extended(
            YTPHY_PAD_DRIVE_STRENGTH,
            YT8531_RX_CLOCK_DRIVE_MASK | YT8531_RX_DATA_DRIVE_HIGH | YT8531_RX_DATA_DRIVE_LOW_MASK,
            (clock << YT8531_RX_CLOCK_DRIVE_SHIFT)
                | ((data & YT8531_DRIVE_CODE_HIGH) << YT8531_RX_DATA_DRIVE_HIGH_SHIFT)
                | ((data & YT8531_DRIVE_CODE_LOW) << YT8531_RX_DATA_DRIVE_LOW_SHIFT),
        )?;

        // Step 3: 关闭节能自动休眠与 RX 时钟门控，确保网线拔出期间 PHY 仍持续向 MAC 提供 RGMII RXC 时钟，使 MAC 软件复位随时能够完成。
        self.modify_extended(YTPHY_SLEEP_CONTROL, YTPHY_SLEEP_ENABLE, 0)?;
        self.modify_extended(YTPHY_CLOCK_GATING, YTPHY_RX_CLOCK_GATE, 0)
    }

    /// 在可睡眠任务上下文中发起一次 Clause 22 标准寄存器读取事务，返回 16 位寄存器值。
    fn read(&self, register: u8) -> Result<u16, GmacError> {
        self.transaction(register, None)
    }

    /// 在可睡眠任务上下文中发起一次 Clause 22 标准寄存器写入事务，等待硬件忙标志清除后返回。
    fn write(&self, register: u8, value: u16) -> Result<(), GmacError> {
        self.transaction(register, Some(value)).map(|_| ())
    }

    /// 执行底层 Clause 22 MDIO 读或写事务：按控制器代数编码 PHY 地址、寄存器号与 MDC 分频档位，并在提交前后同步等待总线空闲。
    fn transaction(&self, register: u8, value: Option<u16>) -> Result<u16, GmacError> {
        let (address, data, command) = match self.registers.generation {
            Generation::Dwmac4 => (
                DWMAC4_MDIO_ADDRESS,
                DWMAC4_MDIO_DATA,
                (u32::from(self.address) << DWMAC4_MDIO_PHY_SHIFT)
                    | (u32::from(register) << DWMAC4_MDIO_REGISTER_SHIFT)
                    | (self.clock << DWMAC4_MDIO_CLOCK_SHIFT)
                    | if value.is_some() {
                        DWMAC4_MDIO_WRITE
                    } else {
                        DWMAC4_MDIO_READ
                    },
            ),
            Generation::Dwmac1000 => (
                DWMAC1000_MDIO_ADDRESS,
                DWMAC1000_MDIO_DATA,
                (u32::from(self.address) << DWMAC1000_MDIO_PHY_SHIFT)
                    | (u32::from(register) << DWMAC1000_MDIO_REGISTER_SHIFT)
                    | (self.clock << DWMAC1000_MDIO_CLOCK_SHIFT)
                    | if value.is_some() {
                        DWMAC1000_MDIO_WRITE
                    } else {
                        0
                    },
            ),
        };

        self.wait_idle(address)?;
        if let Some(value) = value {
            self.registers.write(data, u32::from(value));
        }
        self.registers.write(address, command | MDIO_BUSY);
        self.wait_idle(address)?;
        Ok(self.registers.read(data) as u16)
    }

    /// 轮询等待 MDIO 地址寄存器中的 `MDIO_BUSY` 位清零，每次轮询间隔睡眠 1 毫秒；超过 `MDIO_TIMEOUT`（20 毫秒）仍未完成则返回 I/O 超时错误。
    fn wait_idle(&self, offset: usize) -> Result<(), GmacError> {
        let deadline = time::now().checked_add(MDIO_TIMEOUT).unwrap();
        while self.registers.read(offset) & MDIO_BUSY != 0 {
            if deadline <= time::now() {
                return Err(GmacError::Io("MDIO transaction timed out"));
            }
            timer::sleep(Duration::from_millis(1));
        }
        Ok(())
    }

    /// 通过 `0x1E` 选页寄存器与 `0x1F` 数据寄存器读取裕太微 PHY 的 16 位厂商扩展寄存器（由上层单执行流保证两次 MDIO 写读之间无并发穿插）。
    fn read_extended(&self, register: u16) -> Result<u16, GmacError> {
        self.write(YTPHY_PAGE_SELECT, register)?;
        self.read(YTPHY_PAGE_DATA)
    }

    /// 对裕太微 PHY 的厂商扩展寄存器执行“读-改-写”操作，仅替换 `mask` 覆盖的位域，保留其余硬件引脚采样（Strap）与默认电气配置。
    fn modify_extended(&self, register: u16, mask: u16, value: u16) -> Result<(), GmacError> {
        let current = self.read_extended(register)?;
        self.write(YTPHY_PAGE_SELECT, register)?;
        self.write(YTPHY_PAGE_DATA, (current & !mask) | value)
    }
}

/// 将 MAC CSR 输入时钟频率（单位：Hz）映射为 MDIO 控制器的 6 档标准分频编码，确保输出的 MDC 管理时钟频率不超过 IEEE 802.3 规定的 2.5 MHz 上限。
pub(super) fn clock_range(frequency: u64) -> Result<u32, GmacError> {
    match frequency {
        20_000_000..35_000_000 => Ok(CSR_20_35M),
        35_000_000..60_000_000 => Ok(CSR_35_60M),
        60_000_000..100_000_000 => Ok(CSR_60_100M),
        100_000_000..150_000_000 => Ok(CSR_100_150M),
        150_000_000..250_000_000 => Ok(CSR_150_250M),
        250_000_000..=300_000_000 => Ok(CSR_250_300M),
        _ => Err(GmacError::Resource(
            "MAC CSR clock is outside supported MDIO ranges",
        )),
    }
}

/// 根据设备树指定的 RGMII 接口模式，返回是否需要由 PHY 芯片内部开启接收方向与发送方向的时钟延时 `(rx_delay_enable, tx_delay_enable)`。
fn delay_directions(mode: PhyMode) -> Result<(bool, bool), GmacError> {
    match mode {
        PhyMode::Rgmii => Ok((false, false)),
        PhyMode::RgmiiId => Ok((true, true)),
        PhyMode::RgmiiRxid => Ok((true, false)),
        PhyMode::RgmiiTxid => Ok((false, true)),
        PhyMode::Rmii => Err(GmacError::Resource("selected PHY requires RGMII")),
    }
}

/// 将皮秒（ps）为单位的 RGMII 时钟延时精确换算为 YT8531 的 4 位步进档位（每档 150 ps）及是否启用额外 1900 ps RX 基础偏移；无法精确整除或越界时返回资源错误。
const fn delay_value(picoseconds: u32, receive: bool) -> Result<(u16, bool), GmacError> {
    if picoseconds <= YTPHY_DELAY_MAX_PS && picoseconds % YTPHY_DELAY_STEP_PS == 0 {
        Ok(((picoseconds / YTPHY_DELAY_STEP_PS) as u16, false))
    } else if receive
        && YTPHY_RX_EXTRA_DELAY_PS <= picoseconds
        && picoseconds <= YTPHY_RX_EXTRA_DELAY_PS + YTPHY_DELAY_MAX_PS
        && (picoseconds - YTPHY_RX_EXTRA_DELAY_PS) % YTPHY_DELAY_STEP_PS == 0
    {
        Ok((
            ((picoseconds - YTPHY_RX_EXTRA_DELAY_PS) / YTPHY_DELAY_STEP_PS) as u16,
            true,
        ))
    } else {
        Err(GmacError::Resource("YT8531 delay cannot be represented"))
    }
}

/// 根据 YT8531 芯片实测的 LDO IO 电压档位（3.3V 或 1.8V），将目标微安（uA）驱动电流精确映射为 3 位驱动强度编码；未指定时返回默认档位 `3`。
fn drive_strength(voltage: u16, microamps: Option<u32>) -> Result<u16, GmacError> {
    let Some(microamps) = microamps else {
        return Ok(YT8531_RX_DRIVE_DEFAULT);
    };
    let values = match voltage {
        YT8531_LDO_3V3 => [3070, 4080, 4370, 4680, 5020, 5450, 5740, 6140],
        YT8531_LDO_1V8 => [1200, 2100, 2700, 2910, 3110, 3600, 3970, 4350],
        _ => {
            return Err(GmacError::Resource(
                "YT8531 LDO voltage strap is unsupported",
            ))
        }
    };
    values
        .iter()
        .position(|value| *value == microamps)
        .map(|index| index as u16)
        .ok_or(GmacError::Resource(
            "YT8531 drive current cannot be represented",
        ))
}

/// 求取本端通告值与对端能力值的交集，按 `1000M > 100M > 10M` 以及同速率下“全双工优先于半双工”的顺序返回协商达成的链路模式。
const fn negotiated(local: u16, partner: u16, gigabit: u16, peer_gigabit: u16) -> Option<Link> {
    let common = local & partner;
    let gigabit_common = gigabit & (peer_gigabit >> LPA_1000_TO_ADVERTISE_SHIFT);
    let (speed, full_duplex) = if gigabit_common & ADVERTISE_1000FULL != 0 {
        (SPEED_1000, true)
    } else if gigabit_common & ADVERTISE_1000HALF != 0 {
        (SPEED_1000, false)
    } else if common & ADVERTISE_100FULL != 0 {
        (SPEED_100, true)
    } else if common & ADVERTISE_100HALF != 0 {
        (SPEED_100, false)
    } else if common & ADVERTISE_10FULL != 0 {
        (SPEED_10, true)
    } else if common & ADVERTISE_10HALF != 0 {
        (SPEED_10, false)
    } else {
        return None;
    };
    Some(Link { speed, full_duplex })
}

// 编译期静态断言：验证 YT8531 皮秒延时换算边界与 Clause 22 千兆/百兆自协商优先级选择逻辑。
const _: () = {
    assert!(matches!(delay_value(1900, true), Ok((0, true))));
    assert!(matches!(delay_value(300, false), Ok((2, false))));
    assert!(delay_value(1900, false).is_err());
    assert!(delay_value(1499, true).is_err());
    assert!(matches!(
        negotiated(0x01e1, 0x01e1, 0x0200, 0x0800),
        Some(Link {
            speed: 1000,
            full_duplex: true
        })
    ));
    assert!(matches!(
        negotiated(0x01e1, 0x0081, 0, 0),
        Some(Link {
            speed: 100,
            full_duplex: false
        })
    ));
    assert!(negotiated(1, 1, 0, 0).is_none());
};
