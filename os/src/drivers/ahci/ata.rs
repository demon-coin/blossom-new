//! IDENTIFY 的容量和能力与命令 FIS 编码，不包含请求或内存运行时。

use crate::block::{BlkOperation, BlockError, QueueLimits, NATIVE_TAG_COUNT, SECTOR_BYTES};

pub(super) const IDENTIFY: u8 = 0xec;
pub(super) const MAX_BYTES: usize = 64 * 1024;
pub(super) const MAX_PRDT: usize = 56;
pub(super) const PRDT_BYTES: usize = 4 * 1024 * 1024;
pub(super) const IDENTIFY_WORDS: usize = 256;
pub(super) const IDENTIFY_BYTES: usize = IDENTIFY_WORDS * core::mem::size_of::<u16>();
pub(super) const H2D_FIS_BYTES: usize = 20;
pub(super) const STATUS_BUSY: u32 = 1 << 7;
pub(super) const STATUS_DRQ: u32 = 1 << 3;
pub(super) const STATUS_DEVICE_FAULT: u32 = 1 << 5;
pub(super) const STATUS_ERROR: u32 = 1;
const READ_DMA_EXT: u8 = 0x25;
const WRITE_DMA_EXT: u8 = 0x35;
const READ_DMA: u8 = 0xc8;
const WRITE_DMA: u8 = 0xca;
const FLUSH_CACHE_EXT: u8 = 0xea;
const FLUSH_CACHE: u8 = 0xe7;
const FIS_TYPE_REG_H2D: u8 = 0x27;
const FIS_COMMAND: u8 = 1 << 7;
const DEVICE_LBA: u8 = 1 << 6;
const LBA28_BITS: u32 = 28;
const LBA48_BITS: u32 = 48;
const LBA28_MAX_COUNT: u64 = 1 << u8::BITS;
const LBA48_MAX_COUNT: u64 = 1 << u16::BITS;
const ID_CONFIG: usize = 0;
const ID_CAPABILITY: usize = 49;
const ID_COMMAND_SET_2: usize = 83;
const ID_LBA_CAPACITY: usize = 60;
const ID_LBA_CAPACITY_2: usize = 100;
const ID_SECTOR_SIZE: usize = 106;
const ID_LOGICAL_SECTOR_SIZE: usize = 117;
const ID_NON_ATA: u16 = 1 << 15;
const ID_DMA_SUPPORTED: u16 = 1 << 8;
const ID_LBA_SUPPORTED: u16 = 1 << 9;
const ID_VALID_MASK: u16 = (1 << 15) | (1 << 14);
const ID_VALID: u16 = 1 << 14;
const ID_LBA48_SUPPORTED: u16 = 1 << 10;
const ID_FLUSH_SUPPORTED: u16 = 1 << 12;
const ID_FLUSH_EXT_SUPPORTED: u16 = 1 << 13;
const ID_LONG_LOGICAL_SECTOR: u16 = 1 << 12;

#[derive(Clone, Copy)]
pub(super) struct Identify {
    pub(super) logical_bytes: u32,
    pub(super) logical_sectors: u64,
    pub(super) lba48: bool,
    pub(super) flush: Option<u8>,
}

impl Identify {
    /// 只接受正常 ATA DMA/LBA 设备，扩展字段须带有效位；容量按设备 logical sector 保存。
    pub(super) fn parse(words: &[u16; IDENTIFY_WORDS]) -> Result<Self, BlockError> {
        if words[ID_CONFIG] & ID_NON_ATA != 0
            || words[ID_CAPABILITY] & (ID_DMA_SUPPORTED | ID_LBA_SUPPORTED)
                != ID_DMA_SUPPORTED | ID_LBA_SUPPORTED
        {
            return Err(BlockError::Unsupported);
        }
        let extended_valid = words[ID_COMMAND_SET_2] & ID_VALID_MASK == ID_VALID;
        let lba48 = extended_valid && words[ID_COMMAND_SET_2] & ID_LBA48_SUPPORTED != 0;
        let logical_sectors = if lba48 {
            u64::from(words[ID_LBA_CAPACITY_2])
                | (u64::from(words[ID_LBA_CAPACITY_2 + 1]) << 16)
                | (u64::from(words[ID_LBA_CAPACITY_2 + 2]) << 32)
                | (u64::from(words[ID_LBA_CAPACITY_2 + 3]) << 48)
        } else {
            u64::from(words[ID_LBA_CAPACITY]) | (u64::from(words[ID_LBA_CAPACITY + 1]) << 16)
        };
        let address_limit = if lba48 {
            1_u64 << LBA48_BITS
        } else {
            1_u64 << LBA28_BITS
        };
        if logical_sectors == 0 || address_limit < logical_sectors {
            return Err(BlockError::Corrupt);
        }
        let logical_bytes = if words[ID_SECTOR_SIZE] & (ID_VALID_MASK | ID_LONG_LOGICAL_SECTOR)
            == ID_VALID | ID_LONG_LOGICAL_SECTOR
        {
            let word_count = u32::from(words[ID_LOGICAL_SECTOR_SIZE])
                | (u32::from(words[ID_LOGICAL_SECTOR_SIZE + 1]) << 16);
            word_count
                .checked_mul(core::mem::size_of::<u16>() as u32)
                .ok_or(BlockError::Unsupported)?
        } else {
            SECTOR_BYTES as u32
        };
        if logical_bytes < SECTOR_BYTES as u32
            || MAX_BYTES < logical_bytes as usize
            || !logical_bytes.is_power_of_two()
        {
            return Err(BlockError::Unsupported);
        }
        logical_sectors
            .checked_mul(u64::from(logical_bytes / SECTOR_BYTES as u32))
            .ok_or(BlockError::Corrupt)?;
        // FLUSH 和 LBA48 是独立能力，不能从寻址方式推定持久化命令可用。
        let flush = if extended_valid && words[ID_COMMAND_SET_2] & ID_FLUSH_EXT_SUPPORTED != 0 {
            Some(FLUSH_CACHE_EXT)
        } else if extended_valid && words[ID_COMMAND_SET_2] & ID_FLUSH_SUPPORTED != 0 {
            Some(FLUSH_CACHE)
        } else {
            None
        };
        Ok(Self {
            logical_bytes,
            logical_sectors,
            lba48,
            flush,
        })
    }

    /// 暴露一个硬件命令额度和八个软件完成槽，避免可睡回调占住唯一硬件通道。
    pub(super) fn limits(self) -> QueueLimits {
        QueueLimits {
            capacity_sectors: self.logical_sectors
                * u64::from(self.logical_bytes / SECTOR_BYTES as u32),
            max_bytes: MAX_BYTES,
            max_segments: MAX_PRDT,
            max_segment_bytes: PRDT_BYTES as u32,
            logical_block_size: self.logical_bytes,
            dma_boundary_mask: u64::MAX,
            readonly: false,
            flush: self.flush.is_some(),
            queue_depth: 1,
            tag_count: NATIVE_TAG_COUNT,
        }
    }

    /// 将公共 512B sector 换成设备 LBA，只生成非 NCQ DMA 与真实受支持的 FLUSH。
    pub(super) fn command(
        self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
    ) -> Result<[u8; H2D_FIS_BYTES], BlockError> {
        if operation == BlkOperation::Flush {
            if sector != 0 || bytes != 0 {
                return Err(BlockError::InvalidRange);
            }
            return self.flush.map(fis).ok_or(BlockError::Unsupported);
        }
        let offset = sector
            .checked_mul(SECTOR_BYTES)
            .ok_or(BlockError::InvalidRange)?;
        let logical = u64::from(self.logical_bytes);
        if offset % logical != 0 || bytes as u64 % logical != 0 {
            return Err(BlockError::Misaligned);
        }
        let lba = offset / logical;
        let count = bytes as u64 / logical;
        let max_count = if self.lba48 {
            LBA48_MAX_COUNT
        } else {
            LBA28_MAX_COUNT
        };
        if count == 0 || max_count < count || MAX_BYTES < bytes {
            return Err(BlockError::InvalidRange);
        }
        let end = lba.checked_add(count).ok_or(BlockError::InvalidRange)?;
        if self.logical_sectors < end {
            return Err(BlockError::InvalidRange);
        }
        let opcode = match (operation, self.lba48) {
            (BlkOperation::Read, true) => READ_DMA_EXT,
            (BlkOperation::Write, true) => WRITE_DMA_EXT,
            (BlkOperation::Read, false) => READ_DMA,
            (BlkOperation::Write, false) => WRITE_DMA,
            (BlkOperation::Flush, _) => unreachable!(),
        };
        let mut command = fis(opcode);
        command[4] = lba as u8;
        command[5] = (lba >> 8) as u8;
        command[6] = (lba >> 16) as u8;
        command[7] = DEVICE_LBA;
        if self.lba48 {
            command[8] = (lba >> 24) as u8;
            command[9] = (lba >> 32) as u8;
            command[10] = (lba >> 40) as u8;
            command[13] = (count >> 8) as u8;
        } else {
            command[7] |= ((lba >> 24) & 0xf) as u8;
        }
        command[12] = count as u8;
        Ok(command)
    }
}

/// Register H2D FIS 的 C 位表示 ATA 命令；其余字节由具体命令填充。
pub(super) fn fis(opcode: u8) -> [u8; H2D_FIS_BYTES] {
    let mut command = [0; H2D_FIS_BYTES];
    command[0] = FIS_TYPE_REG_H2D;
    command[1] = FIS_COMMAND;
    command[2] = opcode;
    command
}

/// 以真实协议函数检查 4Kn、LBA48 高位、独立 FLUSH 与不支持字段，供原内核测试调用。
#[cfg(feature = "ktest")]
pub(super) fn self_check() {
    let mut words = [0; 256];
    words[49] = 0x300;
    words[83] = 0x4000 | (1 << 10);
    words[102] = 1;
    words[106] = 0x5000;
    words[117] = 2048;
    let id = Identify::parse(&words).unwrap();
    assert_eq!(id.logical_bytes, 4096);
    assert_eq!(id.limits().capacity_sectors, (1_u64 << 32) * 8);
    assert!(!id.limits().flush);
    assert_eq!(
        id.command(BlkOperation::Flush, 0, 0),
        Err(BlockError::Unsupported)
    );
    let lba = 0x0102_0304_u64;
    let read = id.command(BlkOperation::Read, lba * 8, 8192).unwrap();
    assert_eq!(&read[4..11], &[4, 3, 2, 0x40, 1, 0, 0]);
    assert_eq!(&read[12..14], &[2, 0]);
    assert_eq!(
        id.command(BlkOperation::Read, 1, 4096),
        Err(BlockError::Misaligned)
    );
    words[83] |= 1 << 12;
    assert_eq!(
        Identify::parse(&words)
            .unwrap()
            .command(BlkOperation::Flush, 0, 0)
            .unwrap()[2],
        0xe7
    );
    words[83] |= 1 << 13;
    assert_eq!(
        Identify::parse(&words)
            .unwrap()
            .command(BlkOperation::Flush, 0, 0)
            .unwrap()[2],
        0xea
    );
    words[117] = 0;
    assert!(Identify::parse(&words).is_err());
}
