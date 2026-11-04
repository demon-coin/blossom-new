//! 精确设备名与有界分区发现；格式和校验复用 GPT 库，读盘仍由原 BlockView 完成。

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

/// 物理设备命名域；RAM 不占用 VirtIO 的既有顺序编号。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeviceName {
    Virtio(u8),
    Ram,
    Mmc(u8),
    Scsi(u8),
}

/// 分区号从一开始，None 明确表示整盘，查找时不推测文件系统位置。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceSelector {
    pub device: DeviceName,
    pub partition: Option<u32>,
}

impl DeviceSelector {
    /// 解析完整 /dev 名字，拒绝非规范十进制、零号分区和不完整后缀。
    pub fn parse(path: &[u8]) -> Result<Self, BlockError> {
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
    /// 生成可直接用于 bootargs 的规范完整设备名。
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

/// 解析不带符号或前导零的十进制，溢出保持输入错误。
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

/// 分区表中的原始编号和相对整盘的 512B sector 范围。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Partition {
    pub number: u32,
    pub start_sector: u64,
    pub sectors: u64,
}

impl Partition {
    /// 校验逻辑 LBA 范围并一次换算为 512B sector，末尾等于整盘容量仍合法。
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

#[derive(Debug)]
struct ViewIoError(BlockError);

impl fmt::Display for ViewIoError {
    /// 库错误保留原块层错误供调用者诊断。
    fn fmt(&self, output: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(output, "{:?}", self.0)
    }
}

impl From<ViewIoError> for BlockError {
    /// 解开只用于满足库 Display 约束的错误包装。
    fn from(error: ViewIoError) -> Self {
        error.0
    }
}

struct ViewIo<'a> {
    view: &'a BlockView,
    block_size: BlockSize,
}

impl BlockIo for ViewIo<'_> {
    type Error = ViewIoError;

    /// 分区表 LBA 采用设备真实 logical block，输出范围另行归一为 512B sector。
    fn block_size(&self) -> BlockSize {
        self.block_size
    }

    /// 容量受当前视图限制，库不能读取相邻设备或分区。
    fn num_blocks(&mut self) -> Result<u64, Self::Error> {
        let ratio = self.block_size.to_u64() / SECTOR_BYTES;
        if self.view.sectors() % ratio != 0 {
            return Err(ViewIoError(BlockError::InvalidRange));
        }
        Ok(self.view.sectors() / ratio)
    }

    /// 将库 LBA 一次换算为视图内字节偏移，复用原受限读取和错误传播。
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

    /// 发现接口不授予修改分区表的权限。
    fn write_blocks(&mut self, _: Lba, _: &[u8]) -> Result<(), Self::Error> {
        Err(ViewIoError(BlockError::ReadOnly))
    }

    /// 此只读适配器没有待写数据，库析构无需向设备提交 FLUSH。
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// 普通可睡眠 Task 在整盘启动后发现分区，未识别表时只保留原 raw 整盘。
/// 已识别 MBR/GPT 的范围或校验错误原样返回，不改作 raw 根。
pub fn discover(view: &BlockView) -> Result<Vec<Partition>, BlockError> {
    if view.start_sector() != 0 {
        return Err(BlockError::InvalidRange);
    }
    let block_size =
        BlockSize::new(view.limits().logical_block_size).ok_or(BlockError::Unsupported)?;
    discover_io(ViewIo { view, block_size })
}

/// 共用生产与内存输入的分区读取路径；输出不持有额外设备拥有者。
fn discover_io<Io: BlockIo>(mut io: Io) -> Result<Vec<Partition>, BlockError>
where
    BlockError: From<Io::Error>,
{
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
    io.read_blocks(Lba(0), block).map_err(BlockError::from)?;
    // SAFETY: MasterBootRecord 是库声明的 512B POD，所有位模式有效且没有引用或析构；
    // block 包含已初始化的完整对象字节，read_unaligned 不要求其起始地址对齐。
    let mbr = unsafe { core::ptr::read_unaligned(block.as_ptr().cast::<MasterBootRecord>()) };
    if mbr.signature != MBR_SIGNATURE {
        return Ok(Vec::new());
    }
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

/// 保留四主分区原编号；扩展分区链不在此读取接口的支持范围。
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

/// 使用库结构、数组读取及 CRC，检查主表和数据区的正常容量边界。
fn discover_gpt<Io: BlockIo>(
    disk: &mut GptDisk<Io>,
    capacity: u64,
    block_size: BlockSize,
    block: &mut [u8],
) -> Result<Vec<Partition>, BlockError>
where
    BlockError: From<Io::Error>,
{
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

/// 校验 GPT 1.0 的主头和数据范围，扩展头字节参与同一标准 CRC。
fn validate_header(header: &GptHeader, capacity: u64, block: &[u8]) -> Result<(), BlockError> {
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
    let actual_crc = if size == fixed_size {
        header.calculate_header_crc32().0.to_u32()
    } else {
        extended_header_crc(block, size)
    };
    if actual_crc != header.header_crc32.0.to_u32() {
        return Err(BlockError::Checksum);
    }
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

/// 标准 CRC 算法沿库定义，头内 checksum 字段按规范作为四个零字节参与计算。
/// 调用者已核 92 <= size <= block.len()，不复制整个头或重新实现 CRC 算法。
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

/// 库的 I/O 错误保留块层原因，内部布局错误不被转换为 raw 发现结果。
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
        /// 内存输入的真实访问错误保持为发现失败。
        fn from(_: SliceBlockIoError) -> Self {
            BlockError::Io
        }
    }

    /// 用库写入普通 GPT 输入，主表 CRC 和条目边界与生产解析共享。
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

        /// 原始卷不伪造分区，MBR 保留有洞的编号并允许分区抵达最后一个 sector。
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
                let mut mbr = MasterBootRecord::default();
                mbr.signature = [0x55, 0xaa];
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

        /// GPT 普通数据区首尾包含在分区内，头和数组校验错误均禁止 raw 回退。
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

        /// 4KiB 逻辑 LBA 只换算一次，完整逻辑扇区内的 GPT 头字节均参与校验。
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

        /// 精确名字覆盖各命名域和分区，拒绝零号、歧义及溢出后缀。
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
