//! AHCI 强序寄存器访问；端口窗口只由已经解析的 ABAR 派生。

use crate::arch;
use crate::block::BlockError;
use crate::boot::Region;
use crate::time::{self, Duration};
use crate::timer;
use core::hint::spin_loop;
use core::ptr;

pub(super) const REGISTER_BYTES: usize = core::mem::size_of::<u32>();
pub(super) const MAX_PORTS: u32 = 32;
pub(super) const PORT_BASE: usize = 0x100;
pub(super) const PORT_BYTES: usize = 0x80;
pub(super) const CAP_PORTS_MASK: u32 = 0x1f;
pub(super) const CAP_64BIT: u32 = 1 << 31;
const CAP_STAGGERED_SPINUP: u32 = 1 << 27;
const CAP_FIS_SWITCHING: u32 = 1 << 16;
const CAP2_BIOS_HANDOFF: u32 = 1;
const VERSION_1_2: u32 = 0x0001_0200;
const HOST_RESET: u32 = 1;
const BIOS_OWNED: u32 = 1;
const OS_OWNED: u32 = 1 << 1;
const BIOS_BUSY: u32 = 1 << 4;
const CMD_AGGRESSIVE_SLUMBER: u32 = 1 << 27;
const CMD_LINK_POWER_MANAGEMENT: u32 = 1 << 26;
const CMD_ICC_MASK: u32 = 0xf << 28;
const CMD_ICC_ACTIVE: u32 = 1 << 28;
const CMD_POWER_ON: u32 = 1 << 2;
const CMD_SPIN_UP: u32 = 1 << 1;
pub(super) const ATA_SIGNATURE: u32 = 0x0000_0101;
const SATA_DET_MASK: u32 = 0xf;
const SATA_SPD_MASK: u32 = 0xf << 4;
const SATA_IPM_MASK: u32 = 0xf << 8;
const SATA_DET_PRESENT: u32 = 3;
const SATA_DET_RESET: u32 = 1;
const SATA_IPM_ACTIVE: u32 = 1 << 8;
const SATA_DISABLE_LOW_POWER: u32 = 3 << 8;
const LINK_RESET_HOLD: Duration = Duration::from_millis(1);
const LINK_RESUME_DELAY: Duration = Duration::from_millis(200);
pub(super) const CAP: usize = 0x00;
pub(super) const GHC: usize = 0x04;
pub(super) const HOST_IS: usize = 0x08;
pub(super) const PI: usize = 0x0c;
pub(super) const VERSION: usize = 0x10;
pub(super) const CAP2: usize = 0x24;
pub(super) const BOHC: usize = 0x28;
pub(super) const CLB: usize = 0x00;
pub(super) const CLBU: usize = 0x04;
pub(super) const FB: usize = 0x08;
pub(super) const FBU: usize = 0x0c;
pub(super) const PORT_IS: usize = 0x10;
pub(super) const IE: usize = 0x14;
pub(super) const CMD: usize = 0x18;
pub(super) const TFD: usize = 0x20;
pub(super) const SIG: usize = 0x24;
pub(super) const SSTS: usize = 0x28;
pub(super) const SCTL: usize = 0x2c;
pub(super) const SERR: usize = 0x30;
pub(super) const SACT: usize = 0x34;
pub(super) const CI: usize = 0x38;
pub(super) const FBS: usize = 0x40;
pub(super) const AE: u32 = 1 << 31;
pub(super) const HOST_IE: u32 = 1 << 1;
pub(super) const ST: u32 = 1;
pub(super) const FRE: u32 = 1 << 4;
pub(super) const FR: u32 = 1 << 14;
pub(super) const CR: u32 = 1 << 15;
pub(super) const TASK_BUSY: u32 = super::ata::STATUS_BUSY | super::ata::STATUS_DRQ;
pub(super) const TASK_ERROR: u32 = super::ata::STATUS_ERROR | super::ata::STATUS_DEVICE_FAULT;
const IRQ_TASK_ERROR: u32 = 1 << 30;
const IRQ_HOST_BUS_ERROR: u32 = 1 << 29;
const IRQ_HOST_BUS_DATA_ERROR: u32 = 1 << 28;
const IRQ_INTERFACE_ERROR: u32 = 1 << 27;
const IRQ_INTERFACE_NONFATAL: u32 = 1 << 26;
const IRQ_OVERFLOW: u32 = 1 << 24;
const IRQ_PHY_READY_CHANGE: u32 = 1 << 22;
const IRQ_CONNECT_CHANGE: u32 = 1 << 6;
const IRQ_DESCRIPTOR_DONE: u32 = 1 << 5;
const IRQ_UNKNOWN_FIS: u32 = 1 << 4;
const IRQ_SET_DEVICE_BITS_FIS: u32 = 1 << 3;
const IRQ_DMA_SETUP_FIS: u32 = 1 << 2;
const IRQ_PIO_SETUP_FIS: u32 = 1 << 1;
const IRQ_D2H_REGISTER_FIS: u32 = 1;
pub(super) const COMPLETION: u32 = IRQ_DESCRIPTOR_DONE
    | IRQ_SET_DEVICE_BITS_FIS
    | IRQ_DMA_SETUP_FIS
    | IRQ_PIO_SETUP_FIS
    | IRQ_D2H_REGISTER_FIS;
pub(super) const ERROR: u32 = IRQ_TASK_ERROR
    | IRQ_HOST_BUS_ERROR
    | IRQ_HOST_BUS_DATA_ERROR
    | IRQ_INTERFACE_ERROR
    | IRQ_INTERFACE_NONFATAL
    | IRQ_OVERFLOW
    | IRQ_PHY_READY_CHANGE
    | IRQ_CONNECT_CHANGE
    | IRQ_UNKNOWN_FIS;
pub(super) const INTERRUPTS: u32 = COMPLETION | ERROR;

/// 写回 SControl.DET 时允许的尝试次数；部分控制器会忽略首次写入。
const LINK_RESUME_TRIES: usize = 5;

#[derive(Clone, Copy)]
pub(super) struct Host {
    base: usize,
    len: usize,
}

#[derive(Clone, Copy)]
pub(super) struct Port {
    host: Host,
    index: u32,
}

impl Host {
    /// 建立永久强序窗口；资源必须已经是实际 ABAR，不能传 PCI 配置身份。
    ///
    /// # Safety
    /// 调用者独占控制器，整个物理窗口已永久映射为设备内存，固件不再并发访问。
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

    /// 在已核对的永久设备窗口内执行一个小端寄存器读取。
    pub(super) fn read(self, offset: usize) -> u32 {
        assert!(offset % REGISTER_BYTES == 0 && offset <= self.len - REGISTER_BYTES);
        // SAFETY: new 的调用契约保住完整设备映射；偏移和对齐在本次访问前已检查。
        u32::from_le(unsafe { ptr::read_volatile((self.base + offset) as *const u32) })
    }

    /// 写入设备寄存器；协议序列由拥有者的短锁或准备期独占保证。
    pub(super) fn write(self, offset: usize, value: u32) {
        assert!(offset % REGISTER_BYTES == 0 && offset <= self.len - REGISTER_BYTES);
        // SAFETY: new 保证永久可写设备映射，偏移在完整窗口内且满足 u32 对齐。
        unsafe { ptr::write_volatile((self.base + offset) as *mut u32, value.to_le()) };
    }

    /// 从 CAP 限定的端口号取得完整的 128 字节端口窗口。
    pub(super) fn port(self, index: u32) -> Result<Port, BlockError> {
        if MAX_PORTS <= index || self.len < PORT_BASE + (index as usize + 1) * PORT_BYTES {
            return Err(BlockError::InvalidRange);
        }
        Ok(Port { host: self, index })
    }

    /// 完成固件所有权交接并恢复复位前真实 CAP/PI，不制造不存在的端口。
    pub(super) fn initialize(self, firmware_ports: Option<u32>) -> Result<(u32, u32), BlockError> {
        self.write(GHC, (self.read(GHC) | AE) & !HOST_IE);
        if self.read(GHC) & AE == 0 {
            return Err(BlockError::DeviceFault);
        }
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
        if VERSION_1_2 <= self.read(VERSION) && self.read(CAP2) & CAP2_BIOS_HANDOFF != 0 {
            self.write(BOHC, self.read(BOHC) | OS_OWNED);
            if !poll(Duration::from_secs(2), || {
                self.read(BOHC) & (BIOS_OWNED | BIOS_BUSY) == 0
            }) {
                return Err(BlockError::Busy);
            }
        }
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
    /// 读取本端口寄存器；端口边界由 Host::port 一次验证。
    pub(super) fn read(self, offset: usize) -> u32 {
        assert!(offset < PORT_BYTES);
        self.host
            .read(PORT_BASE + self.index as usize * PORT_BYTES + offset)
    }

    /// 写入本端口寄存器，不将端口状态与 host 状态混用。
    pub(super) fn write(self, offset: usize, value: u32) {
        assert!(offset < PORT_BYTES);
        self.host
            .write(PORT_BASE + self.index as usize * PORT_BYTES + offset, value);
    }

    /// 确认链路已经协商为活动 SATA 设备，ATAPI 与端口倍增器不进入本驱动。
    pub(super) fn connected(self) -> bool {
        self.read(SSTS) & (SATA_IPM_MASK | SATA_DET_MASK) == SATA_IPM_ACTIVE | SATA_DET_PRESENT
            && self.read(SIG) == ATA_SIGNATURE
    }

    /// 保留板固件的链路速度，取消低功耗入口并请求设备上电旋转。
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

    /// 端口停止时编程控制页地址并开启 FIS 接收；调用者须保活控制页直到确认停止。
    pub(super) fn start_fis_rx(self, list: u64, fis: u64) {
        self.write(CLBU, (list >> 32) as u32);
        self.write(CLB, list as u32);
        self.write(FBU, (fis >> 32) as u32);
        self.write(FB, fis as u32);
        self.write(CMD, self.read(CMD) | FRE);
        let _ = self.read(CMD);
    }

    /// 在可睡眠任务中复位链路并等待设备在线，期间保持 FIS 接收开启且命令引擎停止。
    pub(super) fn comreset(self) -> Result<(), BlockError> {
        let sctl = self.read(SCTL);
        self.write(
            SCTL,
            (sctl & SATA_SPD_MASK) | SATA_DISABLE_LOW_POWER | SATA_DET_RESET,
        );
        let _ = self.read(SCTL);
        timer::sleep(LINK_RESET_HOLD);
        // 写入 DET=0 后需等待 200ms 再读回；个别控制器会忽略首次写入。
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
        if !poll(Duration::from_secs(1), || {
            self.read(SSTS) & SATA_DET_MASK == SATA_DET_PRESENT
        }) {
            return Err(BlockError::NotReady);
        }
        Ok(())
    }

    /// 复位后的任务文件只以 BSY 清零判断设备已经就绪。
    pub(super) fn taskfile_ready(self) -> bool {
        self.read(TFD) & super::ata::STATUS_BUSY == 0
    }

    /// 按 W1C 应答本端口的错误和中断，再应答对应 host 位。
    pub(super) fn clear_pending(self) {
        self.write(SERR, self.read(SERR));
        self.write(PORT_IS, self.read(PORT_IS));
        self.ack_host();
    }

    /// 只清本端口的 host 状态，不认领共享线上其它端口的原因。
    pub(super) fn ack_host(self) {
        self.host.write(HOST_IS, 1 << self.index);
    }

    /// 只在准备期轮询停止；ST、CR、FRE、FR 的先后顺序不能合并。
    pub(super) fn stop_polled(self) -> bool {
        self.write(IE, 0);
        self.write(CMD, self.read(CMD) & !ST);
        if !poll(Duration::from_millis(500), || self.read(CMD) & CR == 0) {
            return false;
        }
        self.write(CMD, self.read(CMD) & !FRE);
        poll(Duration::from_secs(1), || self.read(CMD) & FR == 0)
    }

    /// 开放 host 的全局中断位，端口屏蔽仍由本叶单独控制。
    pub(super) fn enable_host_irq(self) {
        self.host.write(GHC, self.host.read(GHC) | AE | HOST_IE);
        let _ = self.host.read(GHC);
    }
}

/// 准备期的有限寄存器等待不创建任务、定时器或运行期轮询路径。
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

/// SATA DET=0 明确表示未检测到设备；链路建立中和 PHY 离线必须保留真实错误。
pub(super) fn media_absent(status: u32) -> bool {
    status & SATA_DET_MASK == 0
}
