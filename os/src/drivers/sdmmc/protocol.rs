//! SD 卡协议解析、容量计算、FIFO 几何参数推导与 32 位链式 IDMAC 描述符构建。
//!
//! # 模块职责与架构定位
//! 本模块位于 `drivers::sdmmc` 内部，将不直接访问 MMIO 寄存器的纯协议计算与数据布局逻辑从控制器状态机中分离出来：
//! - **SD 协议响应与寄存器解析**：拼接 128 位 `CSD` 寄存器位流、计算 SDSC（CSD v1）与 SDHC/SDXC（CSD v2）扇区容量、
//!   解析 `TRAN_SPEED` 最大传输频率、检查 `R1` 状态错误位与就绪状态（`READY_FOR_DATA` + `TRAN`）、换算命令地址参数。
//! - **DW-MMC FIFO 几何推导**：根据控制器版本号（`VERID`）、硬件综合配置（`HCON`）与水线寄存器（`FIFOTH`）确定
//!   FIFO 数据窗口偏移、单次读写位宽（2/4/8 字节）及可读字节数。
//! - **32 位链式 IDMAC 描述符表构建**：为固定 64 KiB 跳板缓冲区（Bounce Buffer）生成最多 16 项、每项最大 4 KiB 的
//!   链式 DMA 描述符。
//!
//! # 核心数据结构与位域布局
//!
//! ## 1. 32 位链式 IDMAC 描述符（`Descriptor`，16 字节对齐）
//! ```text
//! Offset  Field    Bits     Description
//! ------  -------  -------  ---------------------------------------------------------
//! 0x00    control  [31]     OWN: 1 = 归 IDMAC 硬件所有，传输完成后由硬件清零为 0
//!                  [4]      CH (CHAINED): 1 = 第二个地址字段指向下一项描述符地址
//!                  [3]      FS (FIRST): 1 = 本次传输的首个描述符
//!                  [2]      LD (LAST): 1 = 本次传输的末尾描述符
//!                  [1]      DIC (DISABLE_IRQ): 1 = 禁止本项完成时触发中间描述符中断
//! 0x04    size     [12:0]   BS1: 本段数据缓冲区字节长度（本驱动每段上限为 4096 字节）
//! 0x08    buffer   [31:0]   BAP1: 本段数据缓冲区的 32 位设备物理地址（4 KiB 对齐）
//! 0x0c    next     [31:0]   BAP2/Next: 下一项描述符的 32 位设备物理地址（末项为 0）
//! ```
//!
//! ## 2. SD `CSD` 寄存器容量编码（DW-MMC `RESP0..3` 拼接为 `u128`）
//! ```text
//! - CSD Version 1.0 (SDSC, CSD[127:126] == 0b00):
//!   READ_BL_LEN = CSD[83:80] (9..=11, 对应 512..=2048 B)
//!   C_SIZE      = CSD[73:62] (12-bit)
//!   C_SIZE_MULT = CSD[49:47] (3-bit)
//!   Capacity (Bytes) = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2) * 2^(READ_BL_LEN)
//!
//! - CSD Version 2.0 (SDHC/SDXC, CSD[127:126] == 0b01):
//!   C_SIZE      = CSD[69:48] (22-bit)
//!   Capacity (Bytes) = (C_SIZE + 1) * 512 KiB
//! ```
//!
//! # 锁与并发契约
//! 本模块所有函数均为无全局可变状态的纯函数。其中 [`descriptors`] 修改由调用方传入的描述符表可变引用，
//! 调用方（`Sdmmc::transfer`）需持有 `Sdmmc::command` 睡眠互斥锁并确保硬件 DMA 处于停止状态。

use crate::block::{BlockError, SECTOR_BYTES};
use crate::drivers::sdmmc::registers::{
    DATA_OFFSET_240A, DATA_OFFSET_LEGACY, FIFO_COUNT_MASK, FIFO_COUNT_SHIFT, FIFO_MAX_DEPTH,
    FIFO_MIN_DEPTH, FIFO_RX_WATERMARK_SHIFT, FIFO_WATERMARK_MASK, HCON_DATA_WIDTH_MASK,
    HCON_DATA_WIDTH_SHIFT, VERSION_240A, VERSION_MASK,
};
use core::convert::TryFrom;

pub(super) const BOUNCE_BYTES: usize = 64 * 1024;
/// 链式 IDMAC 单个描述符段的数据字节上限（4 KiB），与内核虚拟页大小保持独立定义。
const DESCRIPTOR_DATA_BYTES: usize = 4096;
pub(super) const DESCRIPTORS: usize = BOUNCE_BYTES / DESCRIPTOR_DATA_BYTES;
pub(super) const DESC_BYTES: usize = DESCRIPTORS * core::mem::size_of::<Descriptor>();

pub(super) const OWN: u32 = 1 << 31;
const FIRST_DESCRIPTOR: u32 = 1 << 3;
const LAST_DESCRIPTOR: u32 = 1 << 2;
const CHAINED_DESCRIPTOR: u32 = 1 << 4;
const DISABLE_DESCRIPTOR_IRQ: u32 = 1 << 1;
const DESCRIPTOR_ALIGNMENT: u64 = 16;

pub(super) const GO_IDLE_STATE: u32 = 0;
pub(super) const ALL_SEND_CID: u32 = 2;
pub(super) const SEND_RELATIVE_ADDR: u32 = 3;
pub(super) const APP_SET_BUS_WIDTH: u32 = 6;
pub(super) const SELECT_CARD: u32 = 7;
pub(super) const SEND_IF_COND: u32 = 8;
pub(super) const SEND_CSD: u32 = 9;
pub(super) const STOP_TRANSMISSION: u32 = 12;
pub(super) const SEND_STATUS: u32 = 13;
pub(super) const SET_BLOCKLEN: u32 = 16;
pub(super) const READ_SINGLE_BLOCK: u32 = 17;
pub(super) const READ_MULTIPLE_BLOCK: u32 = 18;
pub(super) const WRITE_BLOCK: u32 = 24;
pub(super) const WRITE_MULTIPLE_BLOCK: u32 = 25;
pub(super) const APP_OP_COND: u32 = 41;
pub(super) const APP_SEND_SCR: u32 = 51;
pub(super) const APP_CMD: u32 = 55;

pub(super) const IF_COND_CHECK_PATTERN: u32 = 0xaa;
pub(super) const IF_COND_VOLTAGE_27_36: u32 = 1 << 8;
pub(super) const IF_COND_ARGUMENT: u32 = IF_COND_VOLTAGE_27_36 | IF_COND_CHECK_PATTERN;
pub(super) const IF_COND_RESPONSE_MASK: u32 = 0xfff;
pub(super) const OCR_VOLTAGE_27_36: u32 = 0x00ff_8000;
pub(super) const OCR_HIGH_CAPACITY: u32 = 1 << 30;
pub(super) const OCR_POWER_UP_COMPLETE: u32 = 1 << 31;

pub(super) const R1_APP_CMD: u32 = 1 << 5;
const R1_STATUS_ERRORS: u32 = 0xfff9_a000;
const R1_AUTHENTICATION_SEQUENCE_ERROR: u32 = 1 << 3;
const R1_ERROR_MASK: u32 = R1_STATUS_ERRORS | R1_AUTHENTICATION_SEQUENCE_ERROR;
const R1_READY_FOR_DATA: u32 = 1 << 8;
const R1_STATE_SHIFT: u32 = 9;
const R1_STATE_MASK: u32 = 15;
const R1_STATE_TRAN: u32 = 4;

const R6_COM_CRC_ERROR: u32 = 1 << 15;
const R6_ILLEGAL_COMMAND: u32 = 1 << 14;
const R6_ERROR: u32 = 1 << 13;
pub(super) const R6_ERRORS: u32 = R6_COM_CRC_ERROR | R6_ILLEGAL_COMMAND | R6_ERROR;
pub(super) const RCA_SHIFT: u32 = 16;
pub(super) const RCA_MASK: u32 = (u16::MAX as u32) << RCA_SHIFT;

pub(super) const SCR_BYTES: usize = core::mem::size_of::<u64>();
pub(super) const SCR_STRUCTURE_SHIFT: u32 = 60;
pub(super) const SCR_BUS_WIDTH_1: u64 = 1 << 48;
pub(super) const SCR_BUS_WIDTH_4: u64 = 1 << 50;
pub(super) const SD_BUS_WIDTH_4: u32 = 2;
pub(super) const CSD_WRITE_PROTECT: u128 = (1 << 13) | (1 << 12);

pub(super) const IDENTIFICATION_HZ: u32 = 400_000;
const DEFAULT_SPEED_HZ: u32 = 25_000_000;

const CSD_STRUCTURE_SHIFT: u32 = 126;
const CSD_READ_BLOCK_LENGTH_SHIFT: u32 = 80;
const CSD_READ_BLOCK_LENGTH_MASK: u128 = 15;
const CSD_V1_SIZE_SHIFT: u32 = 62;
const CSD_V1_SIZE_MASK: u128 = 0xfff;
const CSD_SIZE_MULTIPLIER_SHIFT: u32 = 47;
const CSD_SIZE_MULTIPLIER_MASK: u128 = 7;
const CSD_SIZE_MULTIPLIER_BIAS: u32 = 2;
const CSD_V2_SIZE_SHIFT: u32 = 48;
const CSD_V2_SIZE_MASK: u128 = 0x3f_ffff;
const CSD_V2_CAPACITY_UNIT_BYTES: u64 = 512 * 1024;
const CSD_V2_SECTORS_PER_SIZE: u64 = CSD_V2_CAPACITY_UNIT_BYTES / SECTOR_BYTES;

const CSD_TRANSFER_SPEED_SHIFT: u32 = 96;
const TRANSFER_SPEED_RESERVED: usize = 1 << 7;
const TRANSFER_SPEED_UNIT_MASK: usize = 7;
const TRANSFER_SPEED_VALUE_SHIFT: u32 = 3;
const TRANSFER_SPEED_VALUE_MASK: usize = 15;
const TRANSFER_SPEED_VALUE_SCALE: u32 = 10;
const MIN_READ_BLOCK_BITS: u32 = SECTOR_BYTES.trailing_zeros();
const MAX_READ_BLOCK_BITS: u32 = 11;

/// DW-MMC 32 位内部 DMA（IDMAC）的 4 字（16 字节）链式描述符格式。
///
/// 描述符表所在的控制页在启动 DMA 前整体执行 `sync_for_device` 移交设备访问权，
/// 在 DMA 停止后整体执行 `sync_for_cpu` 收回 CPU 访问权，从而避免相邻描述符共享同一缓存行时的并发伪共享问题。
#[repr(C, align(16))]
#[derive(Clone, Copy, Default)]
pub(super) struct Descriptor {
    /// 控制与状态标志字（含 `OWN`、`FIRST_DESCRIPTOR`、`LAST_DESCRIPTOR`、`CHAINED_DESCRIPTOR` 等位）。
    pub control: u32,
    /// 本段数据缓冲区的传输字节数（最大 `DESCRIPTOR_DATA_BYTES` 即 4096 字节）。
    pub size: u32,
    /// 本段数据缓冲区在 32 位设备地址空间中的起始物理地址。
    pub buffer: u32,
    /// 链式模式下指向下一项描述符的 32 位设备物理地址；若本项为末项则为 0。
    pub next: u32,
}

/// 将 `RESP0..RESP3` 四个 32 位寄存器值按小端字序拼接为完整的 128 位 `CSD` 寄存器位流。
///
/// DW-MMC 硬件在接收 R2 长响应时已自动剥离 SD 总线上的起始位与传输方向位，因此 `RESP0` 最低位直接对应 `CSD[0]`。
pub(super) fn csd_bits(response: [u32; 4]) -> u128 {
    response.iter().enumerate().fold(0, |bits, (i, word)| {
        bits | ((*word as u128) << (i * u32::BITS as usize))
    })
}

/// 解析 128 位 `CSD` 寄存器并结合 `OCR` 协商出的寻址模式，计算 SD 卡总容量（以 512 字节扇区为单位）。
///
/// - `csd`：由 [`csd_bits`] 拼接得到的 128 位 CSD 位域。
/// - `block_addressed`：`true` 表示卡在 `ACMD41` 阶段返回了 `OCR_HIGH_CAPACITY`（SDHC/SDXC 块寻址），
///   `false` 表示标准容量 SDSC 字节寻址。
/// - 返回值：成功时返回非零的 512 字节扇区总数；若 CSD 结构版本与寻址模式不匹配、块长度非法或字节寻址超出 32 位范围，
///   则返回对应的 [`BlockError`]。
pub(super) fn capacity(csd: u128, block_addressed: bool) -> Result<u64, BlockError> {
    // Step 1: 根据 CSD_STRUCTURE 版本号与寻址模式匹配对应的容量计算公式。
    let sectors = match (csd >> CSD_STRUCTURE_SHIFT, block_addressed) {
        (0, false) => {
            let read_length =
                ((csd >> CSD_READ_BLOCK_LENGTH_SHIFT) & CSD_READ_BLOCK_LENGTH_MASK) as u32;
            if !(MIN_READ_BLOCK_BITS..=MAX_READ_BLOCK_BITS).contains(&read_length) {
                return Err(BlockError::Unsupported);
            }
            let size = ((csd >> CSD_V1_SIZE_SHIFT) & CSD_V1_SIZE_MASK) as u64 + 1;
            let multiplier = ((csd >> CSD_SIZE_MULTIPLIER_SHIFT) & CSD_SIZE_MULTIPLIER_MASK) as u32
                + CSD_SIZE_MULTIPLIER_BIAS;
            (size << (multiplier + read_length)) / SECTOR_BYTES
        }
        (1, true) => {
            (((csd >> CSD_V2_SIZE_SHIFT) & CSD_V2_SIZE_MASK) as u64 + 1) * CSD_V2_SECTORS_PER_SIZE
        }
        _ => return Err(BlockError::Unsupported),
    };
    // Step 2: 校验总扇区数非零，且字节寻址卡（SDSC）的总字节数不超过 32 位命令参数上限（4 GiB）。
    if sectors == 0 || (!block_addressed && (1u64 << u32::BITS) < sectors * SECTOR_BYTES) {
        return Err(BlockError::InvalidRange);
    }
    Ok(sectors)
}

/// 解析 `CSD` 寄存器中的 `TRAN_SPEED` 字段（bits `[103:96]`），计算默认速度模式下的最高卡时钟频率（单位：Hz）。
///
/// 本驱动工作在标准默认速度模式（Default Speed，不发送 `CMD6` 切换高速模式），因此返回值上限钳位在 `DEFAULT_SPEED_HZ`（25 MHz）。
pub(super) fn transfer_frequency(csd: u128) -> Result<u32, BlockError> {
    let encoded = ((csd >> CSD_TRANSFER_SPEED_SHIFT) & u8::MAX as u128) as usize;
    let units = [100_000u32, 1_000_000, 10_000_000, 100_000_000];
    let values = [
        0u32, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80,
    ];
    if encoded & TRANSFER_SPEED_RESERVED != 0
        || units.len() - 1 < (encoded & TRANSFER_SPEED_UNIT_MASK)
        || values[(encoded >> TRANSFER_SPEED_VALUE_SHIFT) & TRANSFER_SPEED_VALUE_MASK] == 0
    {
        return Err(BlockError::Unsupported);
    }
    Ok(
        (units[encoded & TRANSFER_SPEED_UNIT_MASK] / TRANSFER_SPEED_VALUE_SCALE
            * values[(encoded >> TRANSFER_SPEED_VALUE_SHIFT) & TRANSFER_SPEED_VALUE_MASK])
            .min(DEFAULT_SPEED_HZ),
    )
}

/// 检查 32 位 `R1` 卡状态响应字中是否包含协议或卡内部错误位。
///
/// 若存在任何 `R1_ERROR_MASK` 错误位则返回 `Err(BlockError::Io)`；否则原样返回响应字，
/// 供调用方进一步检查 `READY_FOR_DATA` 和 `CURRENT_STATE` 状态。
pub(super) fn check_r1(response: u32) -> Result<u32, BlockError> {
    if response & R1_ERROR_MASK != 0 {
        Err(BlockError::Io)
    } else {
        Ok(response)
    }
}

/// 判断 `R1` 响应字是否表明 SD 卡已回到传输就绪状态（`READY_FOR_DATA` 置位且 `CURRENT_STATE == TRAN`）。
///
/// 只有当卡处于 `TRAN` 状态且缓冲区就绪时，主机才能安全发起下一次数据读写或确认前一次写入已完成落盘编程。
pub(super) fn ready(response: u32) -> bool {
    response & R1_READY_FOR_DATA != 0
        && (response >> R1_STATE_SHIFT) & R1_STATE_MASK == R1_STATE_TRAN
}

/// 将 512 字节逻辑扇区号 `sector` 转换为读写命令所需的 32 位 `CMDARG` 地址参数。
///
/// - `block_addressed == true`（SDHC/SDXC）：命令参数直接使用 512 字节块索引。
/// - `block_addressed == false`（SDSC）：命令参数使用字节偏移（`sector * 512`）。
/// - 若计算溢出或超出 `u32::MAX`，返回 `Err(BlockError::InvalidRange)`。
pub(super) fn address(sector: u64, block_addressed: bool) -> Result<u32, BlockError> {
    let value = if block_addressed {
        sector
    } else {
        sector
            .checked_mul(SECTOR_BYTES)
            .ok_or(BlockError::InvalidRange)?
    };
    u32::try_from(value).map_err(|_| BlockError::InvalidRange)
}

/// 根据控制器版本（`VERID`）、硬件配置（`HCON`）、板级显式偏移与 MMIO 窗口大小，计算 FIFO 数据寄存器偏移与单次访问字节宽度。
///
/// 返回 `(fifo_offset, fifo_width)`，其中 `fifo_width` 为 2、4 或 8 字节，且保证 `[fifo_offset, fifo_offset + fifo_width)`
/// 对齐合法并完全落在 MMIO 映射区间 `span` 内。
pub(super) fn fifo_geometry(
    version: u32,
    hcon: u32,
    data_offset: u32,
    span: usize,
) -> Result<(usize, usize), BlockError> {
    let offset = if data_offset != 0 {
        data_offset as usize
    } else if VERSION_240A <= (version & VERSION_MASK) {
        DATA_OFFSET_240A
    } else {
        DATA_OFFSET_LEGACY
    };
    let width = match (hcon >> HCON_DATA_WIDTH_SHIFT) & HCON_DATA_WIDTH_MASK {
        0 => 2,
        1 => 4,
        2 => 8,
        _ => return Err(BlockError::Unsupported),
    };
    if offset % width != 0 || offset.checked_add(width).is_none_or(|end| span < end) {
        return Err(BlockError::InvalidRange);
    }
    Ok((offset, width))
}

/// 确定控制器 FIFO 深度（以 FIFO item 数量为单位）。
///
/// 若平台资源显式指定了 `explicit` 深度则优先采用；否则从复位后尚未改写的 `FIFOTH` 寄存器
/// `RX_WMARK` 字段推导（`depth = RX_WMARK + 1`）。深度必须在 `[FIFO_MIN_DEPTH, FIFO_MAX_DEPTH]`（2..=4096）范围内。
pub(super) fn fifo_depth(explicit: Option<u32>, fifoth: u32) -> Result<u32, BlockError> {
    let depth =
        explicit.unwrap_or_else(|| 1 + ((fifoth >> FIFO_RX_WATERMARK_SHIFT) & FIFO_WATERMARK_MASK));
    if !(FIFO_MIN_DEPTH..=FIFO_MAX_DEPTH).contains(&depth) {
        return Err(BlockError::Unsupported);
    }
    Ok(depth)
}

/// 根据 `STATUS` 寄存器中的 `FCNT`（当前 FIFO 内的 item 数量）和单个 item 字节宽度，计算当前可安全用 PIO 读取的字节数。
///
/// 返回值不会超过剩余待接收字节数 `remaining`。
pub(super) fn fifo_readable_bytes(status: u32, item_bytes: usize, remaining: usize) -> usize {
    let items = ((status >> FIFO_COUNT_SHIFT) & FIFO_COUNT_MASK) as usize;
    (items * item_bytes).min(remaining)
}

/// 为固定 64 KiB 跳板缓冲区构建最多 16 项的 32 位链式 IDMAC 描述符表，返回实际启用的描述符项数。
///
/// # 约束与边界检查
/// - `bytes` 必须非零、不超过 `BOUNCE_BYTES`（64 KiB），且为 512 字节扇区大小的整数倍。
/// - `buffer`（跳板缓冲区设备地址）必须按 4 KiB（`DESCRIPTOR_DATA_BYTES`）对齐，确保每个描述符段不跨越 4 KiB 边界。
/// - `table_address`（描述符表设备地址）必须按 16 字节（`Descriptor` 大小）对齐。
/// - 描述符表与数据缓冲区的末尾地址均不得超出 32 位物理地址空间上限（4 GiB）。
pub(super) fn descriptors(
    table: &mut [Descriptor; DESCRIPTORS],
    table_address: u64,
    buffer: u64,
    bytes: usize,
) -> Result<usize, BlockError> {
    // Step 1: 校验传输长度、扇区对齐、缓冲区 4 KiB 对齐、描述符表 16 字节对齐以及 32 位地址空间上界。
    if bytes == 0
        || BOUNCE_BYTES < bytes
        || bytes % SECTOR_BYTES as usize != 0
        || buffer % DESCRIPTOR_DATA_BYTES as u64 != 0
        || table_address % DESCRIPTOR_ALIGNMENT != 0
    {
        return Err(BlockError::InvalidRange);
    }
    let limit = u32::MAX as u64 + 1;
    if limit
        < buffer
            .checked_add(bytes as u64)
            .ok_or(BlockError::InvalidRange)?
        || limit
            < table_address
                .checked_add(DESC_BYTES as u64)
                .ok_or(BlockError::InvalidRange)?
    {
        return Err(BlockError::InvalidRange);
    }

    // Step 2: 按每段最大 4 KiB 切分请求，依次填充有效链式描述符并将未使用的尾部项清零。
    let used = bytes.div_ceil(DESCRIPTOR_DATA_BYTES);
    for (index, descriptor) in table.iter_mut().enumerate() {
        *descriptor = if index < used {
            let last = index + 1 == used;
            Descriptor {
                control: OWN
                    | if index == 0 { FIRST_DESCRIPTOR } else { 0 }
                    | if last {
                        LAST_DESCRIPTOR
                    } else {
                        CHAINED_DESCRIPTOR | DISABLE_DESCRIPTOR_IRQ
                    },
                size: (bytes - index * DESCRIPTOR_DATA_BYTES).min(DESCRIPTOR_DATA_BYTES) as u32,
                buffer: (buffer + (index * DESCRIPTOR_DATA_BYTES) as u64) as u32,
                next: if last {
                    0
                } else {
                    (table_address + ((index + 1) * core::mem::size_of::<Descriptor>()) as u64)
                        as u32
                },
            }
        } else {
            Descriptor::default()
        };
    }
    Ok(used)
}

#[cfg(feature = "ktest")]
#[ktest::suite(context = "Boot", tags("driver", "sdmmc"), timeout_ms = 1000)]
mod behavior {
    use super::*;

    /// 测试目标：验证 `capacity`（CSD v1/v2 容量解析）、`address`（字节/块寻址换算）以及 `ready`/`check_r1` 状态判断。
    /// 场景步骤：构造合法与非法的 CSD v1/v2 位流、边界扇区号及 R1 状态字，分别调用对应纯函数。
    /// 预期结果：合法输入返回精确扇区数或命令地址，版本与寻址不匹配返回 `Unsupported`，32 位字节寻址溢出返回 `InvalidRange`。
    #[test]
    fn capacity_and_address() {
        let v1 = (10u128 << 80) | (4095u128 << 62) | (7u128 << 47);
        let v2 = (1u128 << 126) | (0x3fffu128 << 48);
        assert_eq!(capacity(v1, false), Ok(4_194_304));
        assert_eq!(capacity(v2, true), Ok(16_777_216));
        assert_eq!(capacity(v1, true), Err(BlockError::Unsupported));
        assert_eq!(capacity(2u128 << 126, true), Err(BlockError::Unsupported));

        assert_eq!(address(0x7f_ffff, false), Ok(0xffff_fe00));
        assert_eq!(address(0x80_0000, false), Err(BlockError::InvalidRange));
        assert_eq!(address(u32::MAX as u64, true), Ok(u32::MAX));

        assert!(ready((4 << 9) | (1 << 8)));
        assert!(!ready((7 << 9) | (1 << 8)));
        assert_eq!(check_r1(1 << 22), Err(BlockError::Io));
    }

    /// 测试目标：验证 `fifo_readable_bytes` 在 64 位（8 字节）与 32 位（4 字节）FIFO 位宽下对可读字节数的折算。
    /// 场景步骤：传入含 0、1、2 个 FIFO item 的 `STATUS` 值及不同的 `item_bytes` 与 `remaining` 上限。
    /// 预期结果：返回字节数等于 `items * item_bytes` 与 `remaining` 的较小值。
    #[test]
    fn scr_fifo_item_units() {
        let one_item = 1 << 17;
        let two_items = 2 << 17;
        assert_eq!(fifo_readable_bytes(one_item, 8, 8), 8);
        assert_eq!(fifo_readable_bytes(one_item, 4, 8), 4);
        assert_eq!(fifo_readable_bytes(two_items, 4, 8), 8);
        assert_eq!(fifo_readable_bytes(one_item, 4, 4), 4);
        assert_eq!(fifo_readable_bytes(0, 8, 8), 0);
        assert_eq!(fifo_readable_bytes(one_item, 8, 0), 0);
        assert_eq!(fifo_readable_bytes(two_items, 8, 8), 8);
    }

    /// 测试目标：验证 `fifo_geometry` 对新旧控制器版本 FIFO 偏移与位宽的校验，以及 `fifo_depth` 的深度推导。
    /// 场景步骤：分别测试 `VERID` 0x2409/0x240a、不同 `HCON` 位宽编码、窗口越界、未对齐偏移及显式/隐式 `FIFOTH` 深度。
    /// 预期结果：合法配置返回正确的 `(offset, width)` 与深度；窗口不足或未对齐返回 `InvalidRange`；非法位宽或深度返回 `Unsupported`。
    #[test]
    fn fifo_geometry_and_depth() {
        assert_eq!(fifo_geometry(0x2409, 0, 0, 0x102), Ok((0x100, 2)));
        assert_eq!(fifo_geometry(0x2409, 1 << 7, 0, 0x104), Ok((0x100, 4)));
        assert_eq!(fifo_geometry(0x2409, 2 << 7, 0, 0x108), Ok((0x100, 8)));
        assert_eq!(fifo_geometry(0x240a, 0, 0, 0x202), Ok((0x200, 2)));
        assert_eq!(fifo_geometry(0x240a, 1 << 7, 0, 0x204), Ok((0x200, 4)));
        assert_eq!(fifo_geometry(0x240a, 2 << 7, 0, 0x208), Ok((0x200, 8)));
        assert_eq!(
            fifo_geometry(0x2409, 2 << 7, 0, 0x107),
            Err(BlockError::InvalidRange)
        );
        assert_eq!(
            fifo_geometry(0x240a, 2 << 7, 0, 0x207),
            Err(BlockError::InvalidRange)
        );

        assert_eq!(fifo_geometry(0x240a, 1 << 7, 0x100, 0x104), Ok((0x100, 4)));
        assert_eq!(
            fifo_geometry(0x2409, 1 << 7, 0x102, 0x108),
            Err(BlockError::InvalidRange)
        );
        assert_eq!(
            fifo_geometry(0x240a, 3 << 7, 0, 0x208),
            Err(BlockError::Unsupported)
        );

        assert_eq!(fifo_depth(Some(2), 0), Ok(2));
        assert_eq!(fifo_depth(Some(4096), 0), Ok(4096));
        assert_eq!(fifo_depth(Some(0), u32::MAX), Err(BlockError::Unsupported));
        assert_eq!(fifo_depth(Some(1), u32::MAX), Err(BlockError::Unsupported));
        assert_eq!(
            fifo_depth(Some(4097), 1 << 16),
            Err(BlockError::Unsupported)
        );
        assert_eq!(fifo_depth(None, 0), Err(BlockError::Unsupported));
        assert_eq!(fifo_depth(None, 1 << 16), Ok(2));
        assert_eq!(fifo_depth(None, 31 << 16), Ok(32));
        assert_eq!(fifo_depth(None, u32::MAX), Ok(4096));
    }

    /// 测试目标：验证 `descriptors` 在 64 KiB 满载传输及 32 位地址空间边界处的链式描述符构建与越界拦截。
    /// 场景步骤：用紧贴 4 GiB 上界的合法缓冲区构建 16 段描述符，再分别测试描述符表越界、数据区越界与首地址未对齐场景。
    /// 预期结果：合法请求生成首尾标志正确的 16 项描述符链；任何越界或未对齐输入均返回 `Err(BlockError::InvalidRange)`。
    #[test]
    fn descriptor_boundaries() {
        let mut table = [Descriptor::default(); DESCRIPTORS];
        assert_eq!(
            descriptors(&mut table, 0x1000, 0xffff_0000, BOUNCE_BYTES),
            Ok(16)
        );
        assert_eq!(table[0].control, OWN | (1 << 3) | (1 << 4) | (1 << 1));
        assert_eq!(table[15].control, OWN | (1 << 2));
        assert_eq!(table[15].buffer, 0xffff_f000);
        assert_eq!(table[15].size, 4096);
        assert_eq!(table[15].next, 0);
        assert_eq!(
            descriptors(&mut table, 0xffff_fff0, 0x1000, 512),
            Err(BlockError::InvalidRange)
        );
        assert_eq!(
            descriptors(&mut table, 0x1000, 0xffff_f000, 8192),
            Err(BlockError::InvalidRange)
        );
        assert_eq!(
            descriptors(&mut table, 0x1000, 0x1001, 512),
            Err(BlockError::InvalidRange)
        );
    }
}
