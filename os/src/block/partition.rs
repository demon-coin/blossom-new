//! 块设备路径解析与磁盘分区发现（MBR / GPT）。
//!
//! # 模块职责与架构定位
//! 本模块作为块设备层向上层引导与存储挂载子系统暴露的只读定位层，承担两项核心职责：
//! 1. **规范设备名解析与格式化**：在 `/dev/vdX[N]`、`/dev/sdX[N]`、`/dev/mmcblkM[pN]`、`/dev/ram0[pN]`
//!    字节路径与强类型选择器 [`DeviceSelector`] 之间双向转换，供内核命令行 `root=` 参数及挂载接口精确定位目标卷。
//! 2. **有界分区表发现**：通过只读适配器封装整盘 [`BlockView`]，复用 `gpt_disk_io` / `gpt_disk_types` 解析标准 MBR 与 GPT 1.0 分区表，
//!    并将不同设备逻辑块大小（如 512B 或 4KiB）的逻辑块地址（LBA）统一换算为块层通用的 512 字节扇区范围 [`Partition`]。
//!
//! # 核心设计与磁盘分区表布局
//! 分区发现严格区分“无分区表的裸卷（Raw Disk）”与“分区表损坏”：若 LBA 0 无 `0x55AA` 签名则返回空列表（整盘作为裸卷使用）；
//! 一旦识别出 MBR 或保护性 GPT 签名，任何越界、保留位非零或 CRC32 校验失败均立即报错返回，绝不静默降级为整盘裸卷，防止误挂载损坏的头部。
//!
//! ## 1. 传统主引导记录（MBR）布局（仅支持 4 个主分区，拒绝扩展分区链）
//! ```text
//!   LBA 0 (512B 头部位于首个逻辑块起始处)
//!   +-----------------------+-----------------------------------+--------------+
//!   | Bootstrap (446 Bytes) | 4 Primary Partition Entries (64B) | Sig 0x55 0xAA|
//!   +-----------------------+-----------------------------------+--------------+
//!   LBA 1 .. capacity - 1 : 各主分区数据区（保留原始槽位号 1..=4，跳过空槽位）
//! ```
//!
//! ## 2. GUID 分区表（GPT 1.0）布局（以设备 `logical_block_size` 为 LBA 单位）
//! ```text
//!   LBA 0                  LBA 1                LBA 2..end     first_usable .. last_usable    capacity - 1
//!   +--------------------+--------------------+--------------+------------------------------+--------------+
//!   | Protective MBR     | Primary GPT Header | Entry Array  | Usable Partition Data Blocks | Backup GPT   |
//!   | (OS Type = 0xEE)   | (Header CRC32)     | (Array CRC32)| (Partition 1..N, 闭区间)      | Header       |
//!   +--------------------+--------------------+--------------+------------------------------+--------------+
//!   输出换算：Partition.start_sector = LBA * (logical_block_size / 512)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块自身不定义任何自旋锁或互斥锁，所有中间缓冲区均在任务栈或局部堆向量中分配。
//! - [`discover`] 必须在普通可睡眠任务上下文调用：底层通过 `BlockView::read_exact_at` 向 `Disk` 提交同步读 `Bio` 并睡眠等待 I/O 完成，
//!   调用方在调用期间严禁持有自旋锁或处于关中断上下文。

use alloc::vec::Vec;
use core::convert::TryFrom;
use core::fmt;
use gpt_disk_io::{BlockIo, Disk as GptDisk, DiskError};
use gpt_disk_types::{BlockSize, GptHeader, GptHeaderRevision, Lba, MasterBootRecord};

use crate::block::{BlockError, BlockView, SECTOR_BYTES};

pub(crate) const LETTERED_DISK_COUNT: u8 = b'z' - b'a' + 1;

const MBR_SIGNATURE: [u8; 2] = [0x55, 0xaa];
const MBR_GPT_PROTECTIVE: u8 = 0xee;
const DOS_EXTENDED_PARTITION: u8 = 0x05;
const WIN98_EXTENDED_PARTITION: u8 = 0x0f;
const LINUX_EXTENDED_PARTITION: u8 = 0x85;

const GPT_PRIMARY_HEADER_LBA: u64 = 1;
const GPT_FIRST_ENTRY_LBA: u64 = GPT_PRIMARY_HEADER_LBA + 1;

/// 块设备的物理命名域分类。
///
/// 不同总线与介质各自维护独立的设备序号空间（例如内存盘固定为 `ram0`，不挤占 VirtIO 磁盘的字母编号）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeviceName {
    /// VirtIO 块设备，携带从 `0` 开始的字母偏移（`0` 对应 `/dev/vda`，上限为 `25` 即 `/dev/vdz`）。
    Virtio(u8),
    /// 唯一保留物理内存盘（固定对应 `/dev/ram0`）。
    Ram,
    /// SD/MMC 块设备，携带控制器/卡槽编号（如 `0` 对应 `/dev/mmcblk0`）。
    Mmc(u8),
    /// SCSI / SATA（AHCI）块设备，携带从 `0` 开始的字母偏移（`0` 对应 `/dev/sda`，上限为 `25` 即 `/dev/sdz`）。
    Scsi(u8),
}

/// 精确标识整块物理磁盘或其上某个分区的设备选择器。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceSelector {
    /// 目标物理设备所属命名域与设备序号。
    pub device: DeviceName,
    /// 目标分区号（从 `1` 开始计数）；`None` 表示直接选择整块物理磁盘。
    pub partition: Option<u32>,
}

impl DeviceSelector {
    /// 解析 `/dev/...` 格式的规范设备路径字节串，构造对应的 [`DeviceSelector`]。
    ///
    /// 无锁纯计算函数，可在任意上下文调用。严格要求规范格式：拒绝缺失 `/dev/` 前缀、带前导零的数字、
    /// `0` 号分区、悬空的 `p` 分隔符以及超出整数上限的编号，非法输入统一返回 [`BlockError::InvalidRange`]。
    pub fn parse(path: &[u8]) -> Result<Self, BlockError> {
        // Step 1: 剥离固定的 `/dev/` 前缀，并根据设备类名前缀拆分出物理设备标识与分区后缀切片。
        let name = path
            .strip_prefix(b"/dev/")
            .ok_or(BlockError::InvalidRange)?;
        let (device, suffix) = if let Some(rest) = name.strip_prefix(b"vd") {
            let (letter, suffix) = rest.split_first().ok_or(BlockError::InvalidRange)?;
            if !letter.is_ascii_lowercase() {
                return Err(BlockError::InvalidRange);
            }
            (DeviceName::Virtio(*letter - b'a'), suffix)
        } else if let Some(rest) = name.strip_prefix(b"sd") {
            let (letter, suffix) = rest.split_first().ok_or(BlockError::InvalidRange)?;
            if !letter.is_ascii_lowercase() {
                return Err(BlockError::InvalidRange);
            }
            (DeviceName::Scsi(*letter - b'a'), suffix)
        } else if let Some(rest) = name.strip_prefix(b"mmcblk") {
            let split = rest.iter().position(|byte| *byte == b'p');
            let (number, suffix) = match split {
                Some(index) => (&rest[..index], &rest[index + 1..]),
                None => (rest, &b""[..]),
            };
            let number = u8::try_from(decimal(number)?).map_err(|_| BlockError::InvalidRange)?;
            if split.is_some() && suffix.is_empty() {
                return Err(BlockError::InvalidRange);
            }
            (DeviceName::Mmc(number), suffix)
        } else if name == b"ram0" {
            (DeviceName::Ram, &b""[..])
        } else if let Some(suffix) = name.strip_prefix(b"ram0p") {
            if suffix.is_empty() {
                return Err(BlockError::InvalidRange);
            }
            (DeviceName::Ram, suffix)
        } else {
            return Err(BlockError::InvalidRange);
        };

        // Step 2: 解析可选的分区号后缀；空后缀代表整盘，非空后缀必须为从 1 开始的规范十进制整数。
        let partition = if suffix.is_empty() {
            None
        } else {
            let number = decimal(suffix)?;
            if number == 0 {
                return Err(BlockError::InvalidRange);
            }
            Some(number)
        };
        Ok(Self { device, partition })
    }
}

impl fmt::Display for DeviceSelector {
    /// 将设备选择器格式化为规范的 `/dev/...` 完整路径（可直接用于内核引导参数展示与日志输出）。
    ///
    /// 对于以数字结尾的设备名（`mmcblkN`、`ram0`），在追加分区号前会自动插入分隔符 `'p'`。
    fn fmt(&self, output: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.device {
            DeviceName::Virtio(index) if index < LETTERED_DISK_COUNT => {
                write!(output, "/dev/vd{}", char::from(b'a' + index))?
            }
            DeviceName::Scsi(index) if index < LETTERED_DISK_COUNT => {
                write!(output, "/dev/sd{}", char::from(b'a' + index))?
            }
            DeviceName::Ram => output.write_str("/dev/ram0")?,
            DeviceName::Mmc(index) => write!(output, "/dev/mmcblk{}", index)?,
            _ => return Err(fmt::Error),
        }
        if let Some(number) = self.partition {
            if matches!(self.device, DeviceName::Mmc(_) | DeviceName::Ram) {
                output.write_str("p")?;
            }
            write!(output, "{}", number)?;
        }
        Ok(())
    }
}

/// 将无符号 ASCII 数字切片解析为 `u32`，拒绝空串、多位数字的前导零（如 `"01"`）以及算术溢出。
fn decimal(bytes: &[u8]) -> Result<u32, BlockError> {
    if bytes.is_empty() || (1 < bytes.len() && bytes[0] == b'0') {
        return Err(BlockError::InvalidRange);
    }
    let mut value = 0_u32;
    for byte in bytes {
        if !byte.is_ascii_digit() {
            return Err(BlockError::InvalidRange);
        }
        value = value
            .checked_mul(10)
            .and_then(|value| value.checked_add(u32::from(*byte - b'0')))
            .ok_or(BlockError::InvalidRange)?;
    }
    Ok(value)
}

/// 从磁盘分区表解析出的单个有效分区范围描述。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Partition {
    /// 分区在分区表中的原始 1-based 编号（MBR 主分区为 `1..=4`，GPT 为条目索引 `+ 1`，保留空洞不重排）。
    pub number: u32,
    /// 分区相对于整盘起始位置的起始扇区偏移（固定以 512 字节 `SECTOR_BYTES` 为单位）。
    pub start_sector: u64,
    /// 分区包含的总扇区数（固定以 512 字节 `SECTOR_BYTES` 为单位，非零）。
    pub sectors: u64,
}

impl Partition {
    /// 校验设备逻辑块（LBA）范围未越过整盘边界，并按 `sector_ratio` 换算为统一的 512 字节扇区单位。
    ///
    /// `start_sector` 与 `sectors` 参数输入时以设备逻辑块（LBA）为单位，`capacity` 为整盘总逻辑块数；
    /// 分区必须从非零 LBA 开始（LBA 0 为 MBR/保护头占用），且末尾允许恰好抵达整盘最后一个逻辑块。
    fn new(
        number: u32,
        start_sector: u64,
        sectors: u64,
        capacity: u64,
        sector_ratio: u64,
    ) -> Result<Self, BlockError> {
        let end = start_sector
            .checked_add(sectors)
            .ok_or(BlockError::InvalidRange)?;
        if number == 0 || start_sector == 0 || sectors == 0 || capacity < end {
            return Err(BlockError::InvalidRange);
        }

        let start_sector = start_sector
            .checked_mul(sector_ratio)
            .ok_or(BlockError::InvalidRange)?;
        let sectors = sectors
            .checked_mul(sector_ratio)
            .ok_or(BlockError::InvalidRange)?;
        Ok(Self {
            number,
            start_sector,
            sectors,
        })
    }
}

/// 将块层 [`BlockError`]（元组字段 `0`）适配为满足 `gpt_disk_io::BlockIo` 所需 `Display` 约束的内部包装类型。
#[derive(Debug)]
struct ViewIoError(BlockError);

impl fmt::Display for ViewIoError {
    /// 输出内部 [`BlockError`] 的调试文本，供 `gpt_disk_io` 错误特征约束使用。
    fn fmt(&self, output: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(output, "{:?}", self.0)
    }
}

impl From<ViewIoError> for BlockError {
    /// 拆包还原底层的 [`BlockError`]。
    fn from(error: ViewIoError) -> Self {
        error.0
    }
}

/// 将整盘 [`BlockView`] 适配为 `gpt_disk_io::BlockIo` 只读接口的适配器。
struct ViewIo<'a> {
    /// 被扫描的底层整盘只读/读写扇区视图借用。
    view: &'a BlockView,
    /// 设备硬件报告的逻辑块大小（字节，如 512B 或 4096B），作为分区表 LBA 寻址单位。
    block_size: BlockSize,
}

impl BlockIo for ViewIo<'_> {
    type Error = ViewIoError;

    /// 返回分区表寻址所使用的设备物理/逻辑块大小。
    fn block_size(&self) -> BlockSize {
        self.block_size
    }

    /// 计算当前视图在设备 `block_size` 粒度下的总逻辑块数；若总容量无法整除逻辑块大小则报错。
    fn num_blocks(&mut self) -> Result<u64, Self::Error> {
        let ratio = self.block_size.to_u64() / SECTOR_BYTES;
        if self.view.sectors() % ratio != 0 {
            return Err(ViewIoError(BlockError::InvalidRange));
        }
        Ok(self.view.sectors() / ratio)
    }

    /// 按设备逻辑块地址 `start_lba` 从底层 `BlockView` 同步读取整数个逻辑块到 `dst`。
    ///
    /// 在普通可睡眠任务上下文执行，越界检查与底层错误均由 `BlockView::read_exact_at` 原样传播。
    fn read_blocks(&mut self, start_lba: Lba, dst: &mut [u8]) -> Result<(), Self::Error> {
        if dst.len()
            % self
                .block_size
                .to_usize()
                .ok_or(ViewIoError(BlockError::InvalidRange))?
            != 0
        {
            return Err(ViewIoError(BlockError::Misaligned));
        }
        let offset = start_lba
            .0
            .checked_mul(self.block_size.to_u64())
            .ok_or(ViewIoError(BlockError::InvalidRange))?;
        self.view.read_exact_at(offset, dst).map_err(ViewIoError)
    }

    /// 分区发现过程严格只读，任何写入请求一律拒绝并返回 [`BlockError::ReadOnly`]。
    fn write_blocks(&mut self, _: Lba, _: &[u8]) -> Result<(), Self::Error> {
        Err(ViewIoError(BlockError::ReadOnly))
    }

    /// 只读适配器不产生脏数据，刷新操作直接成功返回，避免向底层设备发送多余的硬件 FLUSH 命令。
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// 扫描整盘视图 `view` 上的 MBR 或 GPT 分区表，返回全部有效分区的 512B 扇区范围列表。
///
/// 必须在普通可睡眠任务上下文调用，且 `view` 必须是起始扇区为 `0` 的整盘视图（禁止对子分区递归扫描）。
/// - 若首块不存在 `0x55AA` 引导签名，说明介质未分区（如直接格式化的裸文件系统镜像），返回空 `Vec`；
/// - 若存在合法的 MBR 或保护性 GPT 签名，但分区项越界、包含不支持的扩展分区或 CRC 校验失败，则直接返回对应 `Err`。
pub fn discover(view: &BlockView) -> Result<Vec<Partition>, BlockError> {
    if view.start_sector() != 0 {
        return Err(BlockError::InvalidRange);
    }
    let block_size =
        BlockSize::new(view.limits().logical_block_size).ok_or(BlockError::Unsupported)?;
    discover_io(ViewIo { view, block_size })
}

/// 基于通用 `BlockIo` 抽象执行 MBR / GPT 分区发现的核心流程（同时供生产 `BlockView` 与单元测试内存切片复用）。
fn discover_io<Io: BlockIo>(mut io: Io) -> Result<Vec<Partition>, BlockError>
where
    BlockError: From<Io::Error>,
{
    // Step 1: 校验设备逻辑块大小为 2 的幂，并为读取单个逻辑块准备缓冲区（512B 直接走栈数组，更大块按需堆分配）。
    let block_size = io.block_size();
    if !block_size.to_u32().is_power_of_two() {
        return Err(BlockError::Unsupported);
    }
    let capacity = io.num_blocks().map_err(BlockError::from)?;
    let block_bytes = block_size.to_usize().ok_or(BlockError::InvalidRange)?;
    let mut inline = [0_u8; core::mem::size_of::<MasterBootRecord>()];
    let mut storage = Vec::new();
    let block = if block_bytes == inline.len() {
        inline.as_mut_slice()
    } else {
        storage
            .try_reserve_exact(block_bytes)
            .map_err(|_| BlockError::NoMemory)?;
        storage.resize(block_bytes, 0);
        storage.as_mut_slice()
    };

    // Step 2: 读取 LBA 0 并提取前 512 字节的 MBR 结构；若无 0x55AA 签名则判定为无分区表的整盘裸卷。
    io.read_blocks(Lba(0), block).map_err(BlockError::from)?;
    // SAFETY:
    // 1. `block_bytes` 至少为 512 字节（`BlockSize` 下限为 512），且 `io.read_blocks` 成功返回保证了 `block` 前 512 字节已完整初始化；
    // 2. `MasterBootRecord` 是 `gpt_disk_types` 定义的 `#[repr(C)]` 512 字节纯值结构（Copy / 无引用 / 任意位模式合法）；
    // 3. 使用 `read_unaligned` 按字节读取，不要求 `block` 缓冲区的起始地址满足特定对齐。
    let mbr = unsafe { core::ptr::read_unaligned(block.as_ptr().cast::<MasterBootRecord>()) };
    if mbr.signature != MBR_SIGNATURE {
        return Ok(Vec::new());
    }
    // Step 3: 若任一主分区槽位标记为 0xEE（GPT Protective MBR），则转入 GPT 1.0 主表解析；否则按传统 4 主分区 MBR 解析。
    if mbr
        .partitions
        .iter()
        .any(|partition| partition.os_indicator == MBR_GPT_PROTECTIVE)
    {
        let mut disk = GptDisk::new(io).map_err(disk_error)?;
        return discover_gpt(&mut disk, capacity, block_size, block);
    }
    discover_mbr(&mbr, capacity, block_size.to_u64() / SECTOR_BYTES)
}

/// 解析传统 MBR 的 4 个主分区表项，保留原始槽位编号（`1..=4`）；遇到扩展分区类型（`0x05`/`0x0F`/`0x85`）返回 [`BlockError::Unsupported`]。
fn discover_mbr(
    mbr: &MasterBootRecord,
    capacity: u64,
    sector_ratio: u64,
) -> Result<Vec<Partition>, BlockError> {
    let mut partitions = Vec::new();
    partitions
        .try_reserve_exact(mbr.partitions.len())
        .map_err(|_| BlockError::NoMemory)?;
    for (index, entry) in mbr.partitions.iter().enumerate() {
        if entry.os_indicator == 0 {
            continue;
        }
        if matches!(
            entry.os_indicator,
            DOS_EXTENDED_PARTITION | WIN98_EXTENDED_PARTITION | LINUX_EXTENDED_PARTITION
        ) {
            return Err(BlockError::Unsupported);
        }
        partitions.push(Partition::new(
            index as u32 + 1,
            u64::from(entry.starting_lba.to_u32()),
            u64::from(entry.size_in_lba.to_u32()),
            capacity,
            sector_ratio,
        )?);
    }
    Ok(partitions)
}

/// 读取并校验 GPT 主表头与分区条目数组，提取所有已使用且位于可用数据区内的分区。
fn discover_gpt<Io: BlockIo>(
    disk: &mut GptDisk<Io>,
    capacity: u64,
    block_size: BlockSize,
    block: &mut [u8],
) -> Result<Vec<Partition>, BlockError>
where
    BlockError: From<Io::Error>,
{
    // Step 1: 读取 LBA 1 处的主 GPT 表头，校验签名、版本、头 CRC32 以及分区条目数组不与可用数据区重叠。
    let header = disk.read_primary_gpt_header(block).map_err(disk_error)?;
    validate_header(&header, capacity, block)?;
    let layout = header
        .get_partition_entry_array_layout()
        .map_err(|_| BlockError::Corrupt)?;
    let blocks = layout
        .num_blocks(block_size)
        .ok_or(BlockError::InvalidRange)?;
    let end = layout
        .start_lba
        .0
        .checked_add(blocks)
        .ok_or(BlockError::InvalidRange)?;
    if layout.num_entries == 0
        || layout.start_lba.0 < GPT_FIRST_ENTRY_LBA
        || header.first_usable_lba.to_u64() < end
    {
        return Err(BlockError::InvalidRange);
    }

    // Step 2: 按块对齐大小分配缓冲区，读取完整分区条目数组并校验其整体 CRC32。
    let bytes = layout
        .num_bytes_rounded_to_block_as_usize(block_size)
        .ok_or(BlockError::InvalidRange)?;
    let mut storage = Vec::new();
    storage
        .try_reserve_exact(bytes)
        .map_err(|_| BlockError::NoMemory)?;
    storage.resize(bytes, 0);
    let array = disk
        .read_gpt_partition_entry_array(layout, &mut storage)
        .map_err(disk_error)?;
    if array.calculate_crc32() != header.partition_entry_array_crc32 {
        return Err(BlockError::Checksum);
    }

    // Step 3: 顺序扫描全部条目，跳过空 GUID 槽位，校验已用分区的闭区间 `[start, last]` 落在 `[first_usable_lba, last_usable_lba]` 内并换算为 512B 扇区。
    let mut partitions = Vec::new();
    for index in 0..layout.num_entries {
        let entry = array
            .get_partition_entry(index)
            .ok_or(BlockError::Corrupt)?;
        if !entry.is_used() {
            continue;
        }
        let start = entry.starting_lba.to_u64();
        let last = entry.ending_lba.to_u64();
        if start < header.first_usable_lba.to_u64() || header.last_usable_lba.to_u64() < last {
            return Err(BlockError::InvalidRange);
        }
        let sectors = last
            .checked_sub(start)
            .and_then(|length| length.checked_add(1))
            .ok_or(BlockError::InvalidRange)?;
        partitions
            .try_reserve(1)
            .map_err(|_| BlockError::NoMemory)?;
        partitions.push(Partition::new(
            index + 1,
            start,
            sectors,
            capacity,
            block_size.to_u64() / SECTOR_BYTES,
        )?);
    }
    Ok(partitions)
}

/// 校验 GPT 1.0 主表头的签名、版本号、头长度、CRC32 校验和以及数据区与备份头的 LBA 拓扑边界。
///
/// 当 `header_size` 大于固定 92 字节且不超过单个逻辑块长度时，块内扩展头字节一并纳入标准 CRC32 校验。
fn validate_header(header: &GptHeader, capacity: u64, block: &[u8]) -> Result<(), BlockError> {
    // Step 1: 校验 GPT 签名、1.0 版本号、保留字段清零以及 header_size 落在 `[92, block.len()]` 范围内。
    if !header.is_signature_valid()
        || header.revision != GptHeaderRevision::VERSION_1_0
        || header.reserved.to_u32() != 0
    {
        return Err(BlockError::Corrupt);
    }
    let size = header.header_size.to_u32() as usize;
    let fixed_size = core::mem::size_of::<GptHeader>();
    if size < fixed_size || block.len() < size {
        return Err(BlockError::Corrupt);
    }
    // Step 2: 计算并核对主表头 CRC32（92 字节标准头直接计算，扩展头通过零值替换 checksum 字段流式计算）。
    let actual_crc = if size == fixed_size {
        header.calculate_header_crc32().0.to_u32()
    } else {
        extended_header_crc(block, size)
    };
    if actual_crc != header.header_crc32.0.to_u32() {
        return Err(BlockError::Checksum);
    }

    // Step 3: 校验主表头位于 LBA 1、可用数据区起止合法，且备份表头恰好位于整盘最后一个逻辑块（`capacity - 1`）。
    let first = header.first_usable_lba.to_u64();
    let last = header.last_usable_lba.to_u64();
    let alternate = header.alternate_lba.to_u64();
    if header.my_lba.to_u64() != GPT_PRIMARY_HEADER_LBA
        || first < GPT_FIRST_ENTRY_LBA
        || last < first
        || alternate <= last
        || alternate.checked_add(1) != Some(capacity)
    {
        return Err(BlockError::InvalidRange);
    }
    Ok(())
}

/// 计算大于 92 字节的扩展 GPT 表头 CRC32 校验和。
///
/// 按照 UEFI GPT 规范，计算时将偏移 `16..20` 处的 `header_crc32` 字段视作 4 个零字节处理；
/// 调用方须预先保证 `92 <= size <= block.len()`。
fn extended_header_crc(block: &[u8], size: usize) -> u32 {
    const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&gpt_disk_types::Crc32::ALGORITHM);
    const CHECKSUM_OFFSET: usize = 16;
    const CHECKSUM_BYTES: usize = core::mem::size_of::<u32>();
    let mut digest = CRC.digest();
    digest.update(&block[..CHECKSUM_OFFSET]);
    digest.update(&[0_u8; CHECKSUM_BYTES]);
    digest.update(&block[CHECKSUM_OFFSET + CHECKSUM_BYTES..size]);
    digest.finalize()
}

/// 将 `gpt_disk_io::DiskError` 映射为块层 [`BlockError`]（保留底层 I/O 错误，将算术溢出或缓冲越界归为 `InvalidRange`）。
fn disk_error<E: fmt::Debug + fmt::Display>(error: DiskError<E>) -> BlockError
where
    BlockError: From<E>,
{
    match error {
        DiskError::Io(error) => error.into(),
        DiskError::Overflow | DiskError::BufferTooSmall => BlockError::InvalidRange,
        DiskError::BlockSizeSmallerThanPartitionEntry => BlockError::Unsupported,
    }
}

#[cfg(feature = "ktest")]
mod tests {
    use super::*;
    use gpt_disk_io::{BlockIoAdapter, SliceBlockIoError};
    use gpt_disk_types::{
        GptPartitionEntry, GptPartitionEntryArray, GptPartitionType, LbaLe, U32Le,
    };
    use ktest::{TestContext, TestError, TestResult};

    impl From<SliceBlockIoError> for BlockError {
        /// 将内存切片 `BlockIo` 适配器的越界/访问错误转换为 [`BlockError::Io`]。
        fn from(_: SliceBlockIoError) -> Self {
            BlockError::Io
        }
    }

    /// 测试夹具辅助函数：在内存中构造包含 128 个逻辑块的标准 GPT 磁盘镜像，并在第 3 号槽位（索引 2）写入起止为 `[start, last]` 的数据分区。
    fn gpt_image(block_size: BlockSize, start: u64, last: u64) -> Result<Vec<u8>, TestError> {
        let block_bytes = block_size
            .to_usize()
            .ok_or(TestError("GPT image block size overflows"))?;
        let mut image = alloc::vec![0_u8; 128 * block_bytes];
        let mut disk = GptDisk::new(BlockIoAdapter::new(image.as_mut_slice(), block_size))
            .map_err(|_| TestError("GPT image adapter failed"))?;
        let mut header = GptHeader {
            my_lba: LbaLe::from_u64(1),
            alternate_lba: LbaLe::from_u64(127),
            first_usable_lba: LbaLe::from_u64(34),
            last_usable_lba: LbaLe::from_u64(94),
            partition_entry_lba: LbaLe::from_u64(2),
            number_of_partition_entries: U32Le::from_u32(128),
            ..GptHeader::default()
        };
        let layout = header
            .get_partition_entry_array_layout()
            .map_err(|_| TestError("GPT image layout failed"))?;
        let mut bytes = alloc::vec![0_u8; 32 * 512];
        let mut array = GptPartitionEntryArray::new(layout, block_size, &mut bytes)
            .map_err(|_| TestError("GPT image array failed"))?;
        *array
            .get_partition_entry_mut(2)
            .ok_or(TestError("GPT image entry missing"))? = GptPartitionEntry {
            partition_type_guid: GptPartitionType::BASIC_DATA,
            starting_lba: LbaLe::from_u64(start),
            ending_lba: LbaLe::from_u64(last),
            ..GptPartitionEntry::default()
        };
        header.partition_entry_array_crc32 = array.calculate_crc32();
        header.update_header_crc32();
        let mut block = alloc::vec![0_u8; block_bytes];
        disk.write_protective_mbr(&mut block)
            .map_err(|_| TestError("GPT image MBR failed"))?;
        disk.write_primary_gpt_header(&header, &mut block)
            .map_err(|_| TestError("GPT image header failed"))?;
        disk.write_gpt_partition_entry_array(&array)
            .map_err(|_| TestError("GPT image entries failed"))?;
        array.set_start_lba(Lba(95));
        disk.write_gpt_partition_entry_array(&array)
            .map_err(|_| TestError("GPT image backup entries failed"))?;
        header.my_lba = LbaLe::from_u64(127);
        header.alternate_lba = LbaLe::from_u64(1);
        header.partition_entry_lba = LbaLe::from_u64(95);
        header.update_header_crc32();
        disk.write_secondary_gpt_header(&header, &mut block)
            .map_err(|_| TestError("GPT image backup header failed"))?;
        drop(disk);
        Ok(image)
    }

    #[ktest::suite(context = "Thread", tags("block", "partition"), timeout_ms = 15000)]
    mod normal {
        use super::*;

        /// 测试目标：验证无签名裸卷与传统主分区 MBR 在 512B 及 4KiB 逻辑块粒度下的识别与容量边界检查。
        /// 场景步骤：
        /// 1. 对全零内存镜像执行 `discover_io`，验证无 `0x55AA` 签名时返回空分区列表；
        /// 2. 仅在第 4 个主分区槽位写入覆盖 `LBA 1..=15`（直至整盘末尾）的 `0x83` 类型数据分区，验证保留原始槽位号 `4` 并正确按比例换算 512B 扇区；
        /// 3. 将整盘容量缩小为 `15` 个逻辑块使该分区越界 1 块，验证返回 `Err(BlockError::InvalidRange)`。
        #[test]
        fn raw_and_primary_mbr_boundaries(context: &mut TestContext<'_>) -> TestResult {
            for block_size in [BlockSize::BS_512, BlockSize::BS_4096] {
                let block_bytes = block_size
                    .to_usize()
                    .ok_or(TestError("MBR block size overflows"))?;
                let ratio = block_size.to_u64() / SECTOR_BYTES;
                let mut image = alloc::vec![0_u8; 16 * block_bytes];
                context.expect_eq(
                    discover_io(BlockIoAdapter::new(image.as_slice(), block_size)),
                    Ok(Vec::new()),
                );
                let mut mbr = MasterBootRecord {
                    signature: [0x55, 0xaa],
                    ..MasterBootRecord::default()
                };
                mbr.partitions[3].os_indicator = 0x83;
                mbr.partitions[3].starting_lba = U32Le::from_u32(1);
                mbr.partitions[3].size_in_lba = U32Le::from_u32(15);
                {
                    let mut disk =
                        GptDisk::new(BlockIoAdapter::new(image.as_mut_slice(), block_size))
                            .map_err(|_| TestError("MBR image adapter failed"))?;
                    let mut block = alloc::vec![0_u8; block_bytes];
                    disk.write_mbr(&mbr, &mut block)
                        .map_err(|_| TestError("MBR image write failed"))?;
                }
                context.expect_eq(
                    discover_io(BlockIoAdapter::new(image.as_slice(), block_size)),
                    Ok(alloc::vec![Partition {
                        number: 4,
                        start_sector: ratio,
                        sectors: 15 * ratio
                    }]),
                );
                context.expect_eq(discover_mbr(&mbr, 15, ratio), Err(BlockError::InvalidRange));
            }
            Ok(())
        }

        /// 测试目标：验证 GPT 可用数据区闭区间边界、主头/分区数组 CRC32 校验以及扩展头字节篡改检测。
        /// 场景步骤：
        /// 1. 构造 `usable = [34, 94]` 且分区恰好占满 `[34, 94]` 的 512B GPT 镜像，验证解析出 3 号分区（起始 34，共 61 扇区）；
        /// 2. 分别翻转主表头字节与分区条目数组字节，验证均返回 `Err(BlockError::Checksum)` 而非降级为空列表；
        /// 3. 构造越过 `first_usable_lba`、`last_usable_lba` 或起止倒置的分区，验证均返回 `Err(BlockError::InvalidRange)`；
        /// 4. 将 `header_size` 扩展至 96 字节并重算 CRC，验证通过后翻转第 95 字节，确认扩展头区域同样受 CRC 保护。
        #[test]
        fn gpt_crc_and_usable_boundaries(context: &mut TestContext<'_>) -> TestResult {
            let mut image = gpt_image(BlockSize::BS_512, 34, 94)?;
            context.expect_eq(
                discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_512)),
                Ok(alloc::vec![Partition {
                    number: 3,
                    start_sector: 34,
                    sectors: 61
                }]),
            );

            image[512 + 24] ^= 1;
            context.expect_eq(
                discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_512)),
                Err(BlockError::Checksum),
            );
            image[512 + 24] ^= 1;
            image[1024 + 128 * 2 + 40] ^= 1;
            context.expect_eq(
                discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_512)),
                Err(BlockError::Checksum),
            );
            for (start, last) in [(33, 94), (34, 95), (94, 34)] {
                let image = gpt_image(BlockSize::BS_512, start, last)?;
                context.expect_eq(
                    discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_512)),
                    Err(BlockError::InvalidRange),
                );
            }

            let mut image = gpt_image(BlockSize::BS_512, 34, 94)?;
            image[512 + 12..512 + 16].copy_from_slice(&96_u32.to_le_bytes());
            image[512 + 92..512 + 96].copy_from_slice(&[1, 2, 3, 4]);
            let crc = extended_header_crc(&image[512..1024], 96);
            image[512 + 16..512 + 20].copy_from_slice(&crc.to_le_bytes());
            context.expect_eq(
                discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_512)),
                Ok(alloc::vec![Partition {
                    number: 3,
                    start_sector: 34,
                    sectors: 61
                }]),
            );
            image[512 + 95] ^= 1;
            context.expect_eq(
                discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_512)),
                Err(BlockError::Checksum),
            );
            Ok(())
        }

        /// 测试目标：验证 4KiB 逻辑扇区下的 LBA 到 512B 扇区单次换算，以及跨完整 4KiB 逻辑块的扩展 GPT 头长度校验。
        /// 场景步骤：
        /// 1. 在 4KiB 逻辑块 GPT 镜像上依次设置 `header_size` 为 `92`、`512`、`4096` 并更新对应 CRC，验证均精确换算出 `start_sector = 34 * 8`、`sectors = 61 * 8`；
        /// 2. 翻转 `header_size = 4096` 时的块末最后一个字节（偏移 8191），验证返回 `Err(BlockError::Checksum)`；
        /// 3. 将 `header_size` 设为小于固定头的 `91` 或超出单块的 `4097`，验证均返回 `Err(BlockError::Corrupt)`。
        #[test]
        fn logical_sector_and_extended_header(context: &mut TestContext<'_>) -> TestResult {
            let mut image = gpt_image(BlockSize::BS_4096, 34, 94)?;
            for size in [92_u32, 512, 4096] {
                image[4096 + 12..4096 + 16].copy_from_slice(&size.to_le_bytes());
                let crc = extended_header_crc(&image[4096..8192], size as usize);
                image[4096 + 16..4096 + 20].copy_from_slice(&crc.to_le_bytes());
                context.expect_eq(
                    discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_4096)),
                    Ok(alloc::vec![Partition {
                        number: 3,
                        start_sector: 34 * 8,
                        sectors: 61 * 8
                    }]),
                );
            }
            image[8191] ^= 1;
            context.expect_eq(
                discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_4096)),
                Err(BlockError::Checksum),
            );
            for size in [91_u32, 4097] {
                image[4096 + 12..4096 + 16].copy_from_slice(&size.to_le_bytes());
                context.expect_eq(
                    discover_io(BlockIoAdapter::new(image.as_slice(), BlockSize::BS_4096)),
                    Err(BlockError::Corrupt),
                );
            }
            Ok(())
        }

        /// 测试目标：验证 `/dev/` 设备路径解析器对四类设备命名域的正向匹配与非法后缀拒绝能力。
        /// 场景步骤：
        /// 1. 解析合法整盘与带分区路径（`vdb`、`vda4`、`ram0`、`ram0p1`、`mmcblk12p3`、`sdz2`），核对生成的 `DeviceName` 与 `partition`；
        /// 2. 输入零号分区（`vda0`）、双字母磁盘（`vdaa`）、前导零（`mmcblk01`、`sda01`）、空 `p` 后缀（`mmcblk0p`）、非法内存盘号（`ram1`）及 `u32` 溢出分区号，验证全部返回 `Err(BlockError::InvalidRange)`。
        #[test]
        fn exact_device_names(context: &mut TestContext<'_>) -> TestResult {
            for (name, device, partition) in [
                (b"/dev/vdb".as_slice(), DeviceName::Virtio(1), None),
                (b"/dev/vda4", DeviceName::Virtio(0), Some(4)),
                (b"/dev/ram0", DeviceName::Ram, None),
                (b"/dev/ram0p1", DeviceName::Ram, Some(1)),
                (b"/dev/mmcblk12p3", DeviceName::Mmc(12), Some(3)),
                (b"/dev/sdz2", DeviceName::Scsi(25), Some(2)),
            ] {
                context.expect_eq(
                    DeviceSelector::parse(name),
                    Ok(DeviceSelector { device, partition }),
                );
            }
            for name in [
                b"/dev/vda0".as_slice(),
                b"/dev/vdaa",
                b"/dev/mmcblk01",
                b"/dev/mmcblk0p",
                b"/dev/sda01",
                b"/dev/ram1",
                b"/dev/sda4294967296",
            ] {
                context.expect_eq(DeviceSelector::parse(name), Err(BlockError::InvalidRange));
            }
            Ok(())
        }
    }
}
