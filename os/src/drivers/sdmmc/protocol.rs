//! SD 卡响应、容量及固定 IDMAC 链的纯计算。

use crate::block::{BlockError, SECTOR_BYTES};
use crate::drivers::sdmmc::registers::{
    DATA_OFFSET_240A, DATA_OFFSET_LEGACY, FIFO_COUNT_MASK, FIFO_COUNT_SHIFT, FIFO_MAX_DEPTH,
    FIFO_MIN_DEPTH, FIFO_RX_WATERMARK_SHIFT, FIFO_WATERMARK_MASK, HCON_DATA_WIDTH_MASK,
    HCON_DATA_WIDTH_SHIFT, VERSION_240A, VERSION_MASK,
};
use core::convert::TryFrom;

pub(super) const BOUNCE_BYTES: usize = 64 * 1024;
/// 链式 IDMAC 每段的硬件数据上限，与内核页大小分别定义。
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

/// 32 位 IDMAC 的四字链式格式；控制页整体交权，避免相邻描述符的 cacheline 别名。
#[repr(C, align(16))]
#[derive(Clone, Copy, Default)]
pub(super) struct Descriptor {
    pub control: u32,
    pub size: u32,
    pub buffer: u32,
    pub next: u32,
}

/// 从 RESP0 最低位开始组合完整 R2；DW 控制器已去掉线路上的起始及传输位。
pub(super) fn csd_bits(response: [u32; 4]) -> u128 {
    response.iter().enumerate().fold(0, |bits, (i, word)| {
        bits | ((*word as u128) << (i * u32::BITS as usize))
    })
}

/// 校验 CSD 结构与 OCR 寻址模式，返回 512 字节扇区容量。
pub(super) fn capacity(csd: u128, block_addressed: bool) -> Result<u64, BlockError> {
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
    if sectors == 0 || (!block_addressed && (1u64 << u32::BITS) < sectors * SECTOR_BYTES) {
        return Err(BlockError::InvalidRange);
    }
    Ok(sectors)
}

/// 默认速度模式不发 CMD6 切换；CSD 的传输率仍限制主机时钟。
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

/// 返回 SD 的 R1 错误，保留 READY 和 CURRENT_STATE 供调用者判定卡完成。
pub(super) fn check_r1(response: u32) -> Result<u32, BlockError> {
    if response & R1_ERROR_MASK != 0 {
        Err(BlockError::Io)
    } else {
        Ok(response)
    }
}

/// READY_FOR_DATA 与 TRAN 同时成立才允许下一个请求或持久完成。
pub(super) fn ready(response: u32) -> bool {
    response & R1_READY_FOR_DATA != 0
        && (response >> R1_STATE_SHIFT) & R1_STATE_MASK == R1_STATE_TRAN
}

/// SDSC 使用字节地址；SDHC/SDXC 使用 512 字节块地址，溢出不得截断。
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

/// 复位后按版本和 HCON 一次确定 FIFO 入口，检查一个完整 item 的对齐和窗口。
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

/// 显式深度优先；缺省只解释改写前 FIFOTH 的 RX_WMARK，拒绝单 item FIFO。
pub(super) fn fifo_depth(explicit: Option<u32>, fifoth: u32) -> Result<u32, BlockError> {
    let depth =
        explicit.unwrap_or_else(|| 1 + ((fifoth >> FIFO_RX_WATERMARK_SHIFT) & FIFO_WATERMARK_MASK));
    if !(FIFO_MIN_DEPTH..=FIFO_MAX_DEPTH).contains(&depth) {
        return Err(BlockError::Unsupported);
    }
    Ok(depth)
}

/// STATUS 的 FCNT 按 HCON 指定的 FIFO item 计数；调用者传已核对的 2/4/8 字节宽度。
pub(super) fn fifo_readable_bytes(status: u32, item_bytes: usize, remaining: usize) -> usize {
    let items = ((status >> FIFO_COUNT_SHIFT) & FIFO_COUNT_MASK) as usize;
    (items * item_bytes).min(remaining)
}

/// 固定页对齐 bounce 最多使用十六个描述符，每段不跨 4 KiB 边界。
pub(super) fn descriptors(
    table: &mut [Descriptor; DESCRIPTORS],
    table_address: u64,
    buffer: u64,
    bytes: usize,
) -> Result<usize, BlockError> {
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

    /// CSD 两种容量与字节寻址边界调用生产算法，不把纯计算结果当作卡识别证据。
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

    /// 一个 64 位 FIFO item 已包含完整 SCR，32 位 FIFO 的一个 item 只允许读取四字节。
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

    /// 旧新版入口、全部合法宽度及完整窗口采用生产校验，缺省深度不读取 HCON。
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

    /// 固定 bounce 的十六段链包含 32 位地址末端，超限链不得截断为低地址。
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
