//! Clause 22 管理总线与两块板实际使用的 Motorcomm PHY。
//! 本地重写管理事务与 PHY 配置，许可与修改声明见本目录 LICENSE。

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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct Link {
    pub(super) speed: u16,
    pub(super) full_duplex: bool,
}

/// MDIO 只由准备任务或唯一 progress Work 访问，硬中断不获取此对象。
pub(super) struct Phy {
    registers: Registers,
    address: u8,
    clock: u32,
    id: u32,
    adjust_tx: bool,
    inverted: [bool; 3],
}

impl Phy {
    /// 在首次 MAC DMA 复位前配置 PHY；CSR/MDIO 时钟和外设复位须已由板设置启用。
    /// 返回前撤销掉电/隔离并保持无载波时的 RXC，后续运行和停止不得关闭这些输入时钟。
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
        phy.id =
            (u32::from(phy.read(MII_PHYSID1)?) << u16::BITS) | u32::from(phy.read(MII_PHYSID2)?);
        let expected = match registers.generation {
            Generation::Dwmac4 => YT8531,
            Generation::Dwmac1000 => YT8511,
        };
        if phy.id != expected {
            return Err(GmacError::Resource("selected GMAC PHY ID is unsupported"));
        }

        // Clause 22 软复位清除 POWER_DOWN，写入值也明确清除 ISOLATE；
        // 管理时钟由 MAC 的 MDC 提供，复位等待不依赖网线或自协商完成。
        phy.write(MII_BMCR, BMCR_RESET)?;
        let deadline = time::now().checked_add(PHY_RESET_TIMEOUT).unwrap();
        while phy.read(MII_BMCR)? & BMCR_RESET != 0 {
            if deadline <= time::now() {
                return Err(GmacError::Io("PHY reset timed out"));
            }
            timer::sleep(Duration::from_millis(1));
        }

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

    /// BMSR latch-low 连读两次，只有真实协商完成才返回公共最高能力。
    pub(super) fn sample(&self) -> Result<Option<Link>, GmacError> {
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

    /// MAC 已暂停后只在协商模式改变时更新 YT8531 的 TX 时钟相位。
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

    /// 仅广告实际 BMSR/扩展状态支持的铜缆模式，未实现的 pause 不对对端宣称。
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

    /// YT8511 恢复页选择并在睡眠中保持 PLL，RGMII-ID 使用芯片定义的 RX/TX 延时档。
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

    /// YT8531 按实际延时和驱动属性配置，并禁止自动睡眠和无载波时的 RXC 门控。
    fn configure_yt8531(&self, resources: &GmacResources) -> Result<(), GmacError> {
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

        // 禁止自动睡眠切断 RX 时钟；链路断开后仍须允许 MAC 的软件复位完成。
        self.modify_extended(YTPHY_SLEEP_CONTROL, YTPHY_SLEEP_ENABLE, 0)?;
        self.modify_extended(YTPHY_CLOCK_GATING, YTPHY_RX_CLOCK_GATE, 0)
    }

    /// Clause 22 读在普通可睡上下文有界等待，不在 IRQ 中轮询。
    fn read(&self, register: u8) -> Result<u16, GmacError> {
        self.transaction(register, None)
    }

    /// Clause 22 写只有 busy 真正清除才结束本次管理事务。
    fn write(&self, register: u8, value: u16) -> Result<(), GmacError> {
        self.transaction(register, Some(value)).map(|_| ())
    }

    /// 两代管理地址字段和 GOC/MW 编码分开，MDC 分频来自已验证的输入率。
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

    /// 单事务超时报告真实 MDIO 故障，不返回伪造的全零 PHY 状态。
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

    /// 扩展地址锁由唯一管理执行者承担，选择与数据读不被其它 MDIO 事务穿插。
    fn read_extended(&self, register: u16) -> Result<u16, GmacError> {
        self.write(YTPHY_PAGE_SELECT, register)?;
        self.read(YTPHY_PAGE_DATA)
    }

    /// 只修改所需厂商位，其余 strap 与电气设置保持原值。
    fn modify_extended(&self, register: u16, mask: u16, value: u16) -> Result<(), GmacError> {
        let current = self.read_extended(register)?;
        self.write(YTPHY_PAGE_SELECT, register)?;
        self.write(YTPHY_PAGE_DATA, (current & !mask) | value)
    }
}

/// 六种标准 CSR 范围确保 MDC 不超过 2.5MHz，未支持的输入率保资源错误。
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

/// 接口模式指定由 PHY 加延时的方向，板修订不由驱动猜测。
fn delay_directions(mode: PhyMode) -> Result<(bool, bool), GmacError> {
    match mode {
        PhyMode::Rgmii => Ok((false, false)),
        PhyMode::RgmiiId => Ok((true, true)),
        PhyMode::RgmiiRxid => Ok((true, false)),
        PhyMode::RgmiiTxid => Ok((false, true)),
        PhyMode::Rmii => Err(GmacError::Resource("selected PHY requires RGMII")),
    }
}

/// 非法数值不沿最近档位取整；额外 RX 位只用于可精确表示的 1900ps 偏置。
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

/// 电流档由芯片实际 LDO strap 选择，缺省采用该 PHY 定义的驱动档3。
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

/// Clause 22 的共同模式按 1000/100/10 与全双工优先顺序选择。
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

// 纯协议检查随编译执行，不需要实板或伪造管理寄存器。
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
