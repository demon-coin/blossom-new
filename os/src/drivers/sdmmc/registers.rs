//! 32 位 DW-MMC 寄存器与命令位；MMIO 的映射和独占由控制器拥有者证明。

pub(super) const CTRL: usize = 0x00;
pub(super) const PWREN: usize = 0x04;
pub(super) const CLKDIV: usize = 0x08;
pub(super) const CLKSRC: usize = 0x0c;
pub(super) const CLKENA: usize = 0x10;
pub(super) const TMOUT: usize = 0x14;
pub(super) const CTYPE: usize = 0x18;
pub(super) const BLKSIZ: usize = 0x1c;
pub(super) const BYTCNT: usize = 0x20;
pub(super) const INTMASK: usize = 0x24;
pub(super) const CMDARG: usize = 0x28;
pub(super) const CMD: usize = 0x2c;
pub(super) const RESP0: usize = 0x30;
pub(super) const MINTSTS: usize = 0x40;
pub(super) const RINTSTS: usize = 0x44;
pub(super) const STATUS: usize = 0x48;
pub(super) const FIFOTH: usize = 0x4c;
pub(super) const CDETECT: usize = 0x50;
pub(super) const WRTPRT: usize = 0x54;
pub(super) const TCBCNT: usize = 0x5c;
pub(super) const TBBCNT: usize = 0x60;
pub(super) const VERID: usize = 0x6c;
pub(super) const HCON: usize = 0x70;
pub(super) const UHS: usize = 0x74;
pub(super) const BMOD: usize = 0x80;
pub(super) const PLDMND: usize = 0x84;
pub(super) const DBADDR: usize = 0x88;
pub(super) const IDSTS: usize = 0x8c;
pub(super) const IDINTEN: usize = 0x90;

pub(super) const CONTROLLER_RESET: u32 = 1;
pub(super) const FIFO_RESET: u32 = 1 << 1;
pub(super) const DMA_RESET: u32 = 1 << 2;
pub(super) const RESET: u32 = CONTROLLER_RESET | FIFO_RESET | DMA_RESET;
pub(super) const INT_ENABLE: u32 = 1 << 4;
pub(super) const DMA_ENABLE: u32 = 1 << 5;
pub(super) const USE_IDMAC: u32 = 1 << 25;
pub(super) const BMOD_RESET: u32 = 1;
pub(super) const BMOD_ENABLE: u32 = 1 << 7;
pub(super) const BMOD_FIXED_BURST: u32 = 1 << 1;
pub(super) const DATA_BUSY: u32 = 1 << 9;
pub(super) const DMA_REQ: u32 = 1 << 31;
pub(super) const CMD_DONE: u32 = 1 << 2;
pub(super) const DATA_OVER: u32 = 1 << 3;
pub(super) const RESPONSE_TIMEOUT: u32 = 1 << 8;
const RESPONSE_ERROR: u32 = 1 << 1;
const RESPONSE_CRC_ERROR: u32 = 1 << 6;
const DATA_CRC_ERROR: u32 = 1 << 7;
const DATA_TIMEOUT: u32 = 1 << 9;
const HOST_TIMEOUT: u32 = 1 << 10;
const FIFO_ERROR: u32 = 1 << 11;
const HARDWARE_LOCK_ERROR: u32 = 1 << 12;
const START_BIT_ERROR: u32 = 1 << 13;
const END_BIT_ERROR: u32 = 1 << 15;
pub(super) const ERRORS: u32 = RESPONSE_ERROR
    | RESPONSE_CRC_ERROR
    | DATA_CRC_ERROR
    | RESPONSE_TIMEOUT
    | DATA_TIMEOUT
    | HOST_TIMEOUT
    | FIFO_ERROR
    | HARDWARE_LOCK_ERROR
    | START_BIT_ERROR
    | END_BIT_ERROR;
pub(super) const IRQ_MASK: u32 = ERRORS | CMD_DONE | DATA_OVER;
pub(super) const IDMAC_DONE: u32 = 1 << 30;
pub(super) const IDMAC_ERROR: u32 = 1 << 31;
const IDMAC_FATAL_BUS_ERROR: u32 = 1 << 2;
const IDMAC_DESCRIPTOR_UNAVAILABLE: u32 = 1 << 4;
const IDMAC_CARD_ERROR: u32 = 1 << 5;
const IDMAC_ABNORMAL_SUMMARY: u32 = 1 << 9;
const IDMAC_NORMAL_SUMMARY: u32 = 1 << 8;
pub(super) const IDMAC_TRANSMIT: u32 = 1;
pub(super) const IDMAC_RECEIVE: u32 = 1 << 1;
pub(super) const IDMAC_COMPLETION: u32 = IDMAC_TRANSMIT | IDMAC_RECEIVE;
pub(super) const IDMAC_ERRORS: u32 = IDMAC_FATAL_BUS_ERROR
    | IDMAC_DESCRIPTOR_UNAVAILABLE
    | IDMAC_CARD_ERROR
    | IDMAC_ABNORMAL_SUMMARY;
pub(super) const IDMAC_IRQ: u32 = IDMAC_ERRORS | IDMAC_NORMAL_SUMMARY | IDMAC_COMPLETION;
pub(super) const START: u32 = 1 << 31;
pub(super) const HOLD: u32 = 1 << 29;
pub(super) const UPDATE_CLOCK: u32 = 1 << 21;
pub(super) const INITIALIZE: u32 = 1 << 15;
pub(super) const STOP: u32 = 1 << 14;
pub(super) const WAIT_DATA: u32 = 1 << 13;
pub(super) const WRITE: u32 = 1 << 10;
pub(super) const DATA: u32 = 1 << 9;
pub(super) const CRC: u32 = 1 << 8;
pub(super) const LONG: u32 = 1 << 7;
pub(super) const RESPONSE: u32 = 1 << 6;

pub(super) const REGISTER_BYTES: usize = core::mem::size_of::<u32>();
pub(super) const CLOCK_ENABLE: u32 = 1;
pub(super) const CLOCK_DIVIDER_MASK: u32 = 0xff;
pub(super) const CLOCK_DIVIDER_FACTOR: u64 = 2;
pub(super) const CARD_POWER_ENABLE: u32 = 1;
pub(super) const CARD_DETECT_ABSENT: u32 = 1;
pub(super) const CARD_WRITE_PROTECTED: u32 = 1;
pub(super) const CTYPE_1BIT: u32 = 0;
pub(super) const CTYPE_4BIT: u32 = 1;
pub(super) const HCON_ADDRESS_64BIT: u32 = 1 << 27;
pub(super) const HCON_DMA_INTERFACE_SHIFT: u32 = 16;
pub(super) const HCON_DMA_INTERFACE_MASK: u32 = 3;
pub(super) const HCON_DATA_WIDTH_SHIFT: u32 = 7;
pub(super) const HCON_DATA_WIDTH_MASK: u32 = 7;
pub(super) const VERSION_MASK: u32 = 0xffff;
pub(super) const VERSION_240A: u32 = 0x240a;
pub(super) const DATA_OFFSET_LEGACY: usize = 0x100;
pub(super) const DATA_OFFSET_240A: usize = 0x200;
pub(super) const FIFO_RX_WATERMARK_SHIFT: u32 = 16;
pub(super) const FIFO_WATERMARK_MASK: u32 = 0xfff;
pub(crate) const FIFO_MIN_DEPTH: u32 = 2;
pub(crate) const FIFO_MAX_DEPTH: u32 = FIFO_WATERMARK_MASK + 1;
pub(super) const FIFO_COUNT_SHIFT: u32 = 17;
pub(super) const FIFO_COUNT_MASK: u32 = 0x1fff;

/// 验证后的永久 MMIO 窗口；方法只接受本模块公开的对齐偏移。
pub(super) struct Registers(pub usize);

impl Registers {
    /// 读取独占映射内的寄存器；并发 W1C 和控制更新由外层短锁协调。
    pub fn read(&self, offset: usize) -> u32 {
        // SAFETY: prepare 验证整个寄存器窗口，拥有者保活映射；偏移只来自本模块常量。
        unsafe { ((self.0 + offset) as *const u32).read_volatile() }
    }

    /// 写入独占映射内的寄存器，不把普通原子操作当设备访问。
    pub fn write(&self, offset: usize, value: u32) {
        // SAFETY: prepare 验证整个寄存器窗口，拥有者保活映射；偏移只来自本模块常量。
        unsafe { ((self.0 + offset) as *mut u32).write_volatile(value) }
    }

    /// 在调用方持有寄存器短锁或尚未发表时更新控制位。
    pub fn update(&self, offset: usize, clear: u32, set: u32) {
        self.write(offset, (self.read(offset) & !clear) | set);
    }

    /// 读取 R2 的低字在前原始排列。
    pub fn response(&self) -> [u32; 4] {
        core::array::from_fn(|index| self.read(RESP0 + index * REGISTER_BYTES))
    }
}
