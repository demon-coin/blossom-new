//! ATA 设备识别（IDENTIFY）解析与命令帧信息结构（FIS）编码模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 AHCI 驱动内部（`drivers::ahci::ata`），为上层 AHCI 控制器（`super::Ahci`）
//! 提供纯无状态的 ATA/SATA 协议解析与命令构造能力：
//! - **上游调用者**：`drivers::ahci` 在设备探测阶段调用 [`Identify::parse`] 解析 512 字节的
//!   IDENTIFY 数据字，并在块 I/O 提交阶段调用 [`Identify::command`] 编码 20 字节的
//!   Register Host-to-Device (H2D) FIS。
//! - **下游依赖**：仅依赖通用块设备层（`crate::block`）的请求类型、队列限额与错误定义，
//!   不涉及任何 MMIO 寄存器访问、DMA 内存管理或并发锁状态。
//!
//! # 核心设计与协议布局
//!
//! ## 1. Register Host-to-Device (H2D) FIS 布局（20 字节 / 5 个 DWORD）
//! AHCI 命令表（Command Table）开头的 CFIS 区域存放发送给 SATA 设备的命令帧，格式如下：
//!
//! ```text
//! +------+-----------------+-----------------+-----------------+-----------------+
//! | Byte |        3        |        2        |        1        |        0        |
//! +------+-----------------+-----------------+-----------------+-----------------+
//! | DW0  | Features (7:0)  | Command Opcode  | C=1 | Port Mult | FIS Type (0x27) |
//! | DW1  | Device (LBA=1)  | LBA (23:16)     | LBA (15:8)      | LBA (7:0)       |
//! | DW2  | Features (15:8) | LBA (47:40)     | LBA (39:32)     | LBA (31:24)     |
//! | DW3  | Control         | ICC             | Count (15:8)    | Count (7:0)     |
//! | DW4  | Reserved (0)    | Reserved (0)    | Reserved (0)    | Reserved (0)    |
//! +------+-----------------+-----------------+-----------------+-----------------+
//! ```
//! - **LBA28 模式**：使用 `LBA (23:0)`，并将 `LBA (27:24)` 填入 `Device` 字段的低 4 位；扇区数使用 `Count (7:0)`。
//! - **LBA48 模式**：完整使用 `LBA (47:0)` 与 `Count (15:0)`。
//!
//! ## 2. 扇区粒度转换与能力独立性
//! - 块设备子系统统一以 512 字节（[`SECTOR_BYTES`]）作为标准扇区单位；本模块负责将其按设备实际的
//!   逻辑扇区大小（如 512B 或 4Kn 即 4096B）换算为设备 LBA 与扇区计数，并拦截未对齐请求。
//! - 磁盘写缓存刷新（`FLUSH CACHE` / `FLUSH CACHE EXT`）与 LBA48 寻址在 ATA 规范中属于独立能力位，
//!   必须根据 IDENTIFY Word 83 分别判定，不可由 LBA48 推导刷新支持。
//!
//! # 锁与并发契约
//! 本模块所有结构体与函数均为纯值计算（`Copy` 值语义），无内部可变状态或锁，可在任意上下文安全调用。

use crate::block::{BlkOperation, BlockError, QueueLimits, NATIVE_TAG_COUNT, SECTOR_BYTES};

/// ATA `IDENTIFY DEVICE` 命令操作码（PIO Data-In，但在 AHCI 中通过 DMA 引擎接收数据）。
pub(super) const IDENTIFY: u8 = 0xec;
/// 单次 AHCI 传输允许的最大数据字节数（64 KiB）。
pub(super) const MAX_BYTES: usize = 64 * 1024;
/// 单个命令表分配的最大物理区域描述符表（PRDT）项数。
pub(super) const MAX_PRDT: usize = 56;
/// 单个 PRDT 表项硬件支持的最大字节数（4 MiB）。
pub(super) const PRDT_BYTES: usize = 4 * 1024 * 1024;
/// ATA `IDENTIFY DEVICE` 返回的数据字（16-bit word）数量。
pub(super) const IDENTIFY_WORDS: usize = 256;
/// ATA `IDENTIFY DEVICE` 返回的数据总字节数（512 字节）。
pub(super) const IDENTIFY_BYTES: usize = IDENTIFY_WORDS * core::mem::size_of::<u16>();
/// Register Host-to-Device (H2D) FIS 的固定字节长度（20 字节）。
pub(super) const H2D_FIS_BYTES: usize = 20;

/// ATA 状态寄存器 `BSY`（Busy，位 7）：设备正忙，任务文件寄存器不可访问。
pub(super) const STATUS_BUSY: u32 = 1 << 7;
/// ATA 状态寄存器 `DRQ`（Data Request，位 3）：设备请求或正在进行数据传输。
pub(super) const STATUS_DRQ: u32 = 1 << 3;
/// ATA 状态寄存器 `DF`（Device Fault，位 5）：设备发生内部故障。
pub(super) const STATUS_DEVICE_FAULT: u32 = 1 << 5;
/// ATA 状态寄存器 `ERR`（Error，位 0）：上一条命令执行出错。
pub(super) const STATUS_ERROR: u32 = 1;

/// 48-bit LBA DMA 读取命令（`READ DMA EXT`）。
const READ_DMA_EXT: u8 = 0x25;
/// 48-bit LBA DMA 写入命令（`WRITE DMA EXT`）。
const WRITE_DMA_EXT: u8 = 0x35;
/// 28-bit LBA DMA 读取命令（`READ DMA`）。
const READ_DMA: u8 = 0xc8;
/// 28-bit LBA DMA 写入命令（`WRITE DMA`）。
const WRITE_DMA: u8 = 0xca;
/// 48-bit LBA 易失性写缓存刷新命令（`FLUSH CACHE EXT`）。
const FLUSH_CACHE_EXT: u8 = 0xea;
/// 28-bit LBA 易失性写缓存刷新命令（`FLUSH CACHE`）。
const FLUSH_CACHE: u8 = 0xe7;
/// SATA FIS 类型：Register Host-to-Device（主机到设备的寄存器帧，值 `0x27`）。
const FIS_TYPE_REG_H2D: u8 = 0x27;
/// Register H2D FIS 字节 1 的 `C` 位（位 7）：置 1 表示更新 Command 寄存器并触发命令执行。
const FIS_COMMAND: u8 = 1 << 7;
/// ATA Device 寄存器的 LBA 模式选择位（位 6）：置 1 表示采用 LBA 寻址而非 CHS 寻址。
const DEVICE_LBA: u8 = 1 << 6;

/// LBA28 寻址模式的地址位宽（28 位，最大寻址 128 GiB @ 512B/扇区）。
const LBA28_BITS: u32 = 28;
/// LBA48 寻址模式的地址位宽（48 位，最大寻址 128 PiB @ 512B/扇区）。
const LBA48_BITS: u32 = 48;
/// LBA28 模式下单条命令允许的最大逻辑扇区计数（256 个扇区）。
const LBA28_MAX_COUNT: u64 = 1 << u8::BITS;
/// LBA48 模式下单条命令允许的最大逻辑扇区计数（65536 个扇区）。
const LBA48_MAX_COUNT: u64 = 1 << u16::BITS;

/// IDENTIFY Word 0：常规配置位（General Configuration）。
const ID_CONFIG: usize = 0;
/// IDENTIFY Word 49：设备能力位（Capabilities，含 DMA 与 LBA 支持标志）。
const ID_CAPABILITY: usize = 49;
/// IDENTIFY Word 83：支持的命令集与特性第二组（Command Set Supported，含 LBA48 与 FLUSH）。
const ID_COMMAND_SET_2: usize = 83;
/// IDENTIFY Word 60..=61：LBA28 模式下的总可寻址逻辑扇区数（u32 小端）。
const ID_LBA_CAPACITY: usize = 60;
/// IDENTIFY Word 100..=103：LBA48 模式下的总可寻址逻辑扇区数（u64 小端）。
const ID_LBA_CAPACITY_2: usize = 100;
/// IDENTIFY Word 106：物理扇区大小与逻辑扇区大小关系描述符。
const ID_SECTOR_SIZE: usize = 106;
/// IDENTIFY Word 117..=118：每个逻辑扇区包含的 16-bit 字数（当逻辑扇区大于 512B 时有效）。
const ID_LOGICAL_SECTOR_SIZE: usize = 117;

/// Word 0 位 15：置 1 表示非 ATA 设备（如 ATAPI 设备）。
const ID_NON_ATA: u16 = 1 << 15;
/// Word 49 位 8：置 1 表示设备支持 DMA 传输。
const ID_DMA_SUPPORTED: u16 = 1 << 8;
/// Word 49 位 9：置 1 表示设备支持 LBA 寻址。
const ID_LBA_SUPPORTED: u16 = 1 << 9;
/// Word 83 / Word 106 位 15:14 掩码：用于校验该扩展字内容是否有效（要求位 15 为 0、位 14 为 1）。
const ID_VALID_MASK: u16 = (1 << 15) | (1 << 14);
/// Word 83 / Word 106 有效标志预期值（位 14 置 1，位 15 清 0）。
const ID_VALID: u16 = 1 << 14;
/// Word 83 位 10：置 1 表示支持 48-bit LBA 寻址特性集。
const ID_LBA48_SUPPORTED: u16 = 1 << 10;
/// Word 83 位 12：置 1 表示支持 `FLUSH CACHE` 命令。
const ID_FLUSH_SUPPORTED: u16 = 1 << 12;
/// Word 83 位 13：置 1 表示支持 `FLUSH CACHE EXT` 命令。
const ID_FLUSH_EXT_SUPPORTED: u16 = 1 << 13;
/// Word 106 位 12：置 1 表示设备逻辑扇区大小大于 256 个字（即大于 512 字节，需读取 Word 117..=118）。
const ID_LONG_LOGICAL_SECTOR: u16 = 1 << 12;

/// 解析后的 ATA 设备几何参数与命令能力摘要。
#[derive(Clone, Copy)]
pub(super) struct Identify {
    /// 设备单个逻辑扇区的字节大小（如 512 或 4096），必须是 2 的幂且在 `[512, MAX_BYTES]` 范围内。
    pub(super) logical_bytes: u32,
    /// 设备按 `logical_bytes` 计数的总逻辑扇区数。
    pub(super) logical_sectors: u64,
    /// 设备是否支持 48-bit LBA 寻址（决定使用 `*_DMA_EXT` 还是 28-bit `*_DMA` 命令）。
    pub(super) lba48: bool,
    /// 设备支持的写缓存刷新操作码（`Some(FLUSH_CACHE_EXT)`、`Some(FLUSH_CACHE)` 或 `None`）。
    pub(super) flush: Option<u8>,
}

impl Identify {
    /// 解析 256 个字的 `IDENTIFY DEVICE` 原始数据并提取磁盘容量、扇区大小与命令集能力。
    ///
    /// 仅接受同时支持 DMA 与 LBA 的 ATA 磁盘设备；若设备为 ATAPI、缺失基础能力、
    /// 扇区大小不受支持或容量字段越界损坏，则返回相应的 [`BlockError`]。
    pub(super) fn parse(words: &[u16; IDENTIFY_WORDS]) -> Result<Self, BlockError> {
        // Step 1: 验证是否为支持 DMA 和 LBA 的标准 ATA 设备（排除 ATAPI 等非 ATA 设备）。
        if words[ID_CONFIG] & ID_NON_ATA != 0
            || words[ID_CAPABILITY] & (ID_DMA_SUPPORTED | ID_LBA_SUPPORTED)
                != ID_DMA_SUPPORTED | ID_LBA_SUPPORTED
        {
            return Err(BlockError::Unsupported);
        }

        // Step 2: 检查 Word 83 扩展能力字的有效性标志，判定是否支持 LBA48，并读取对应的总逻辑扇区数。
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

        // Step 3: 解析逻辑扇区字节数（默认 512B；若 Word 106 指示长逻辑扇区，则从 Word 117..=118 读取字数并换算）。
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
        // 确保换算为 512B 标准扇区总数时不会发生 u64 整数溢出。
        logical_sectors
            .checked_mul(u64::from(logical_bytes / SECTOR_BYTES as u32))
            .ok_or(BlockError::Corrupt)?;

        // Step 4: 独立判定写缓存刷新命令支持情况（FLUSH 与 LBA48 相互独立，优先选择 FLUSH CACHE EXT）。
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

    /// 根据解析出的设备参数构造块设备层所需的队列限额（[`QueueLimits`]）。
    ///
    /// 本驱动采用单硬件命令槽串行提交（`queue_depth = 1`），同时向块设备层暴露
    /// [`NATIVE_TAG_COUNT`]（8 个）软件请求槽位，使上层完成回调在释放硬件通道后仍可并发流转。
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

    /// 将块设备层的通用请求（以 512B 扇区为单位）校验并编码为 20 字节的 Register H2D FIS。
    ///
    /// # 参数
    /// - `operation`: 块操作类型（`Read`、`Write` 或 `Flush`）。
    /// - `sector`: 起始 512B 扇区号（`Flush` 操作必须为 0）。
    /// - `bytes`: 传输字节数（`Flush` 操作必须为 0；读写操作必须对齐到设备 `logical_bytes`）。
    pub(super) fn command(
        self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
    ) -> Result<[u8; H2D_FIS_BYTES], BlockError> {
        // Step 1: 处理零长度的写缓存刷新请求。
        if operation == BlkOperation::Flush {
            if sector != 0 || bytes != 0 {
                return Err(BlockError::InvalidRange);
            }
            return self.flush.map(fis).ok_or(BlockError::Unsupported);
        }

        // Step 2: 将 512B 扇区偏移与字节长度换算为设备逻辑扇区（LBA）与扇区数，校验对齐与容量边界。
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

        // Step 3: 选择非 NCQ 的 DMA 读写操作码，并按 LBA48 或 LBA28 格式填入 FIS 字段。
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

/// 构造指定 ATA 命令操作码的基础 Register H2D FIS（置位 `C=1`，其余参数字段清零）。
pub(super) fn fis(opcode: u8) -> [u8; H2D_FIS_BYTES] {
    let mut command = [0; H2D_FIS_BYTES];
    command[0] = FIS_TYPE_REG_H2D;
    command[1] = FIS_COMMAND;
    command[2] = opcode;
    command
}

/// 内核单元测试辅助函数：验证 4Kn 逻辑扇区解析、LBA48 高位编码、FLUSH 能力独立判定及非法参数拦截。
///
/// - **测试目标**：确保 [`Identify::parse`] 与 [`Identify::command`] 严格遵循 ATA 规范。
/// - **场景步骤（Given-When-Then）**：
///   1. 构造 4096B 逻辑扇区、支持 LBA48 但未置位 FLUSH 的 IDENTIFY 字数组，验证扇区换算与 `Flush` 拒绝；
///   2. 对 4Kn 设备发起对齐与未对齐的 `Read` 命令，检查 48-bit LBA 字节序列与 `Misaligned` 错误；
///   3. 依次置位 Word 83 的 `FLUSH CACHE`（位 12）与 `FLUSH CACHE EXT`（位 13），验证操作码 `0xe7` 与 `0xea` 选择；
///   4. 将长逻辑扇区字数清零，验证非法几何参数被拒绝。
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
