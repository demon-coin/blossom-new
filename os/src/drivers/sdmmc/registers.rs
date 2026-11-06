//! Synopsys DesignWare Mobile Storage Host Controller (DW-MMC) 32 位寄存器定义与 MMIO 访问封装。
//!
//! # 模块职责与架构定位
//! 本模块位于 `drivers::sdmmc` 驱动内部，负责定义 DW-MMC 控制器及内置 32 位链式 IDMAC 的全部
//! MMIO 寄存器字节偏移、控制位、状态位与命令标志位，并提供轻量级的寄存器读写封装 [`Registers`]。
//! 上游仅由 `drivers::sdmmc` 和 `drivers::sdmmc::protocol` 调用，不对外暴露硬件细节。
//!
//! # 核心寄存器与位域布局
//!
//! ## 1. 寄存器空间布局（32 位 IDMAC 变体）
//! ```text
//! Offset  Name      Description
//! ------  --------  ------------------------------------------------------
//! 0x00    CTRL      全局控制（软复位 CONTROLLER/FIFO/DMA、总中断与 IDMAC 使能）
//! 0x04    PWREN     卡电源开关
//! 0x08    CLKDIV    时钟分频器（Divider0 位于 bit[7:0]，实际分频 = 2 * div）
//! 0x0c    CLKSRC    时钟源选择
//! 0x10    CLKENA    卡时钟输出使能
//! 0x14    TMOUT     响应与数据超时计数器
//! 0x18    CTYPE     卡总线宽度（0 = 1-bit，1 = 4-bit）
//! 0x1c    BLKSIZ    块大小（通常为 512 字节，读取 SCR 时为 8 字节）
//! 0x20    BYTCNT    本次传输总字节数
//! 0x24    INTMASK   主机中断屏蔽（1 = 允许产生中断）
//! 0x28    CMDARG    32 位命令参数
//! 0x2c    CMD       命令下发与传输模式控制（写 START 位触发硬件执行）
//! 0x30..  RESP0..3  136 位长响应或 48 位短响应寄存器组（4 x 32-bit）
//! 0x40    MINTSTS   屏蔽后中断状态
//! 0x44    RINTSTS   原始中断状态（写 1 清零 W1C）
//! 0x48    STATUS    控制器实时状态（DATA_BUSY、FIFO 计数 FCNT、DMA_REQ 等）
//! 0x4c    FIFOTH    FIFO 收发水线阈值
//! 0x50    CDETECT   原生卡检测引脚状态（0 = 在位，1 = 缺卡）
//! 0x54    WRTPRT    原生写保护引脚状态（1 = 写保护）
//! 0x5c    TCBCNT    卡侧已传输字节数
//! 0x60    TBBCNT    主机侧 FIFO/总线已传输字节数
//! 0x6c    VERID     Synopsys IP 版本号（0x240a 起 FIFO 数据窗口迁至 0x200）
//! 0x70    HCON      硬件综合配置参数（总线位宽、DMA 接口类型、地址位宽）
//! 0x74    UHS       UHS-I 电压与 DDR 控制
//! 0x80    BMOD      IDMAC 总线模式（软复位 SWR、突发模式 FB、使能 DE）
//! 0x84    PLDMND    IDMAC 轮询需求触发（写入任意值唤醒挂起的描述符读取）
//! 0x88    DBADDR    IDMAC 描述符表首地址（32 位物理/设备地址）
//! 0x8c    IDSTS     IDMAC 状态寄存器（写 1 清零 W1C）
//! 0x90    IDINTEN   IDMAC 中断使能
//! 0x100 / 0x200     FIFO 数据读写窗口（旧版 < 2.40a 为 0x100，>= 2.40a 为 0x200）
//! ```
//!
//! ## 2. `CMD` 命令寄存器关键位域（Offset `0x2c`）
//! ```text
//!  31    29         21        15    14     13        10     9     8     7      6    5..0
//! +-----+----+-----+---------+-----+----+------+----+-----+----+-----+------+-----+------+
//! |START|HOLD| ... |UPD_CLOCK|INIT |STOP|WAIT_D| .. |WRITE|DATA| CRC | LONG |RESP |INDEX |
//! +-----+----+-----+---------+-----+----+------+----+-----+----+-----+------+-----+------+
//! ```
//!
//! # 并发与锁契约
//! [`Registers`] 仅封装對已映射 MMIO 虚拟基址的 `read_volatile` / `write_volatile` 操作，自身不持锁：
//! - 设备初始化（`Sdmmc::initialize`）期间，控制器尚未发布到块设备层也未开放中断，由初始化任务独占访问。
//! - 运行期（`publish` 之后），对 `CTRL`、`INTMASK`、`RINTSTS`、`BMOD`、`IDSTS`、`IDINTEN` 等共享控制与
//!   状态寄存器的并发读写（如工作线程下发命令与中断处理函数清中断）必须在外层持有 `Sdmmc::registers`
//!   自旋锁临界区后执行。

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

/// 已通过边界与对齐校验的 DW-MMC 永久 MMIO 寄存器窗口封装。
///
/// 内部元组字段 `0` 保存控制器寄存器窗口的内核虚拟基地址（字节对齐的 `usize`）。
pub(super) struct Registers(pub usize);

impl Registers {
    /// 读取指定字节偏移处的 32 位寄存器值。
    ///
    /// 调用方需传入本模块定义的 4 字节对齐寄存器偏移常量；运行期涉及中断与控制并发访问时由外层自旋锁保护。
    pub fn read(&self, offset: usize) -> u32 {
        // SAFETY:
        // 1. `Sdmmc::prepare` 在构造 `Registers` 前已校验 `self.0` 满足 8 字节对齐，且映射区间长度
        //    至少覆盖到 `IDINTEN + REGISTER_BYTES`（0x94 字节）。
        // 2. 本方法仅由 `sdmmc` 模块以本文件定义的 4 字节对齐寄存器偏移常量调用，目标地址不越界且对齐合法。
        // 3. 启动期创建的内核永久 MMIO 映射在系统运行期间始终有效。
        unsafe { ((self.0 + offset) as *const u32).read_volatile() }
    }

    /// 向指定字节偏移处的 32 位寄存器写入值。
    ///
    /// 使用 `write_volatile` 确保每次写入都产生实际的设备总线事务，不被编译器优化合并或重排。
    pub fn write(&self, offset: usize, value: u32) {
        // SAFETY:
        // 1. `Sdmmc::prepare` 已验证 `self.0` 的对齐与 MMIO 窗口覆盖范围。
        // 2. `offset` 仅来自本模块的 4 字节对齐寄存器偏移常量，不会越界。
        // 3. 读-改-写或 W1C 清除操作在并发场景下由外层 `Sdmmc::registers` 自旋锁串行化。
        unsafe { ((self.0 + offset) as *mut u32).write_volatile(value) }
    }

    /// 对指定寄存器执行“先清除 `clear` 掩码位、再置位 `set` 掩码位”的读-改-写更新。
    ///
    /// 在控制器已发布运行期调用时，调用方必须持有 `Sdmmc::registers` 自旋锁以防并发修改丢失。
    pub fn update(&self, offset: usize, clear: u32, set: u32) {
        self.write(offset, (self.read(offset) & !clear) | set);
    }

    /// 依次读取 `RESP0..RESP3` 四个 32 位响应寄存器，按低字在前（`RESP0` 在下标 0）返回原始数组。
    pub fn response(&self) -> [u32; 4] {
        core::array::from_fn(|index| self.read(RESP0 + index * REGISTER_BYTES))
    }
}
