//! SD 卡响应、容量及固定 IDMAC 链的纯计算。

use crate::block::BlockError;
use core::convert::TryFrom;

pub(super) const BOUNCE_BYTES: usize = 64 * 1024;
pub(super) const DESCRIPTORS: usize = BOUNCE_BYTES / 4096;
pub(super) const DESC_BYTES: usize = DESCRIPTORS * core::mem::size_of::<Descriptor>();
pub(super) const OWN: u32 = 1 << 31;

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
    response
        .iter()
        .enumerate()
        .fold(0, |bits, (i, word)| bits | ((*word as u128) << (i * 32)))
}

/// 校验 CSD 结构与 OCR 寻址模式，返回 512 字节扇区容量。
pub(super) fn capacity(csd: u128, block_addressed: bool) -> Result<u64, BlockError> {
    let sectors = match (csd >> 126, block_addressed) {
        (0, false) => {
            let read_length = ((csd >> 80) & 15) as u32;
            if !(9..=11).contains(&read_length) {
                return Err(BlockError::Unsupported);
            }
            let size = ((csd >> 62) & 0xfff) as u64 + 1;
            let multiplier = ((csd >> 47) & 7) as u32 + 2;
            (size << (multiplier + read_length)) / 512
        }
        (1, true) => (((csd >> 48) & 0x3f_ffff) as u64 + 1) * 1024,
        _ => return Err(BlockError::Unsupported),
    };
    if sectors == 0 || (!block_addressed && (1u64 << 32) < sectors * 512) {
        return Err(BlockError::InvalidRange);
    }
    Ok(sectors)
}

/// 默认速度模式不发 CMD6 切换；CSD 的传输率仍限制主机时钟。
pub(super) fn transfer_frequency(csd: u128) -> Result<u32, BlockError> {
    let encoded = ((csd >> 96) & 0xff) as usize;
    let units = [100_000u32, 1_000_000, 10_000_000, 100_000_000];
    let values = [
        0u32, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80,
    ];
    if encoded & 0x80 != 0 || 3 < (encoded & 7) || values[(encoded >> 3) & 15] == 0 {
        return Err(BlockError::Unsupported);
    }
    Ok((units[encoded & 7] / 10 * values[(encoded >> 3) & 15]).min(25_000_000))
}

/// 返回 SD 的 R1 错误，保留 READY 和 CURRENT_STATE 供调用者判定卡完成。
pub(super) fn check_r1(response: u32) -> Result<u32, BlockError> {
    if response & 0xfff9_a008 != 0 {
        Err(BlockError::Io)
    } else {
        Ok(response)
    }
}

/// READY_FOR_DATA 与 TRAN 同时成立才允许下一个请求或持久完成。
pub(super) fn ready(response: u32) -> bool {
    response & (1 << 8) != 0 && (response >> 9) & 15 == 4
}

/// SDSC 使用字节地址；SDHC/SDXC 使用 512 字节块地址，溢出不得截断。
pub(super) fn address(sector: u64, block_addressed: bool) -> Result<u32, BlockError> {
    let value = if block_addressed {
        sector
    } else {
        sector.checked_mul(512).ok_or(BlockError::InvalidRange)?
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
    } else if 0x240a <= (version & 0xffff) {
        0x200
    } else {
        0x100
    };
    let width = match (hcon >> 7) & 7 {
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
    let depth = explicit.unwrap_or_else(|| 1 + ((fifoth >> 16) & 0xfff));
    if !(2..=4096).contains(&depth) {
        return Err(BlockError::Unsupported);
    }
    Ok(depth)
}

/// STATUS 的 FCNT 按 HCON 指定的 FIFO item 计数；调用者传已核对的 2/4/8 字节宽度。
pub(super) fn fifo_readable_bytes(status: u32, item_bytes: usize, remaining: usize) -> usize {
    let items = ((status >> 17) & 0x1fff) as usize;
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
        || bytes % 512 != 0
        || buffer % 4096 != 0
        || table_address % 16 != 0
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
    let used = (bytes + 4095) / 4096;
    for (index, descriptor) in table.iter_mut().enumerate() {
        *descriptor = if index < used {
            let last = index + 1 == used;
            Descriptor {
                control: OWN
                    | if index == 0 { 1 << 3 } else { 0 }
                    | if last { 1 << 2 } else { (1 << 4) | (1 << 1) },
                size: (bytes - index * 4096).min(4096) as u32,
                buffer: (buffer + (index * 4096) as u64) as u32,
                next: if last {
                    0
                } else {
                    (table_address + ((index + 1) * 16) as u64) as u32
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
