//! ELF64 小端解码与最终映像区间计划；文件、匿名零页和必要合成页共用现有 Mm。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::cmp::{max, min};
use core::convert::TryInto;

use crate::arch;
use crate::fs::FileRef;
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;
use crate::mm::{alloc_pages, PAGE_SIZE};

use super::{read_exact, ExecError, MAX_PATH_BYTES, RWX};

pub(super) const ELF_HEADER_BYTES: usize = 64;
const PHDR_BYTES: usize = 56;
const PT_LOAD: u32 = 1;
const PT_INTERP: u32 = 3;
const PT_GNU_STACK: u32 = 0x6474_e551;
const ET_DYN: u16 = 3;
const PF_R: u32 = 4;
const PF_W: u32 = 2;
const PF_X: u32 = 1;
pub(super) const MAIN_DYN_BASE: usize = 0x2000_0000;
pub(super) const INTERP_DYN_BASE: usize = 0x3000_0000;

/// 只保存装载、入口和初始栈真正消费的映像描述。
pub(super) struct Elf {
    pub(super) interp: Option<Vec<u8>>,
    pub(super) bias: usize,
    segments: Vec<Segment>,
    pub(super) entry: usize,
    pub(super) phdr: Option<usize>,
    pub(super) phent: usize,
    pub(super) phnum: usize,
    phoff: u64,
    table_bytes: usize,
    pub(super) end: usize,
    pub(super) executable_stack: bool,
}

/// 文件前缀和匿名零尾都按原程序头顺序参与最终字节覆盖。
struct Segment {
    start: usize,
    file_end: usize,
    end: usize,
    offset: u64,
    flags: u32,
}

/// 程序头只解码固定前缀，较长表项的扩展尾不参与装载。
struct ProgramHeader {
    kind: u32,
    flags: u32,
    offset: u64,
    address: usize,
    filesz: usize,
    memsz: usize,
    align: usize,
}

impl ProgramHeader {
    /// 从已完整读取的 ELF64 小端程序头前缀取得各字段。
    fn decode(bytes: &[u8; PHDR_BYTES]) -> Self {
        Self {
            kind: word32(bytes, 0),
            flags: word32(bytes, 4),
            offset: word64(bytes, 8),
            address: word64(bytes, 16) as usize,
            filesz: word64(bytes, 32) as usize,
            memsz: word64(bytes, 40) as usize,
            align: word64(bytes, 48) as usize,
        }
    }

    /// LOAD 的文件前缀必须可读取，且能容纳于同一可表示的内存区间。
    fn segment(&self, size: u64) -> Result<Segment, ExecError> {
        if self.memsz < self.filesz {
            return Err(ExecError::Format);
        }
        if self.filesz != 0 {
            file_range(self.offset, self.filesz, size)?;
        }
        Ok(Segment {
            start: self.address,
            file_end: self
                .address
                .checked_add(self.filesz)
                .ok_or(ExecError::Format)?,
            end: self
                .address
                .checked_add(self.memsz)
                .ok_or(ExecError::Format)?,
            offset: self.offset,
            flags: self.flags,
        })
    }
}

impl Elf {
    /// 逐项读取实际程序头，保留声明顺序；未消费的元数据不施加额外格式政策。
    pub(super) fn read(
        file: &FileRef,
        header: &[u8; ELF_HEADER_BYTES],
        dyn_base: usize,
    ) -> Result<Self, ExecError> {
        let size = file.stat()?.size;
        let phent = word16(header, 54) as usize;
        let phnum = word16(header, 56) as usize;
        let phoff = word64(header, 32);
        let table_bytes = phnum.checked_mul(phent).ok_or(ExecError::Format)?;
        if header[..6] != *b"\x7fELF\x02\x01"
            || size < ELF_HEADER_BYTES as u64
            || phent < PHDR_BYTES
        {
            return Err(ExecError::Format);
        }
        file_range(phoff, table_bytes, size)?;
        let dynamic = word16(header, 16) == ET_DYN;
        let mut alignment = PAGE_SIZE;
        let mut segments = Vec::new();
        let mut interp = None;
        let mut executable_stack = false;
        for index in 0..phnum {
            let mut bytes = [0; PHDR_BYTES];
            read_exact(file, phoff + (index * phent) as u64, &mut bytes)?;
            let ph = ProgramHeader::decode(&bytes);
            match ph.kind {
                PT_LOAD => {
                    let segment = ph.segment(size)?;
                    if segment.start != segment.end {
                        if dynamic && 1 < ph.align {
                            alignment = common_alignment(alignment, ph.align)?;
                        }
                        segments.try_reserve(1).map_err(|_| ExecError::NoMemory)?;
                        segments.push(segment);
                    }
                }
                PT_INTERP => {
                    if interp.is_some() {
                        return Err(ExecError::Format);
                    }
                    file_range(ph.offset, ph.filesz, size)?;
                    interp = Some(read_interpreter(file, &ph)?);
                }
                PT_GNU_STACK => executable_stack |= ph.flags & PF_X != 0,
                _ => {}
            }
        }
        if segments.is_empty() {
            return Err(ExecError::Format);
        }
        let bias = if dynamic {
            align_up(dyn_base, alignment)?
        } else {
            0
        };
        let mut end = 0;
        for segment in &mut segments {
            segment.start = segment.start.checked_add(bias).ok_or(ExecError::Format)?;
            segment.file_end = segment
                .file_end
                .checked_add(bias)
                .ok_or(ExecError::Format)?;
            segment.end = segment.end.checked_add(bias).ok_or(ExecError::Format)?;
            if arch::USER_ADDRESS_END < page_end(segment.end)? {
                return Err(ExecError::Vm(VmError::InvalidRange));
            }
            end = max(end, segment.end);
        }
        Ok(Self {
            interp,
            bias,
            segments,
            entry: (word64(header, 24) as usize)
                .checked_add(bias)
                .ok_or(ExecError::Format)?,
            phdr: None,
            phent,
            phnum,
            phoff,
            table_bytes,
            end,
            executable_stack,
        })
    }

    /// 按实际 LOAD 页范围检查栈及 guard 的占用，映像高地址空洞不占用栈。
    pub(super) fn overlaps(&self, start: usize, end: usize) -> bool {
        self.segments.iter().any(|segment| {
            page_start(segment.start) < end
                && start < page_end(segment.end).expect("validated LOAD page end")
        })
    }
}

/// 只验证当前读取的文件区间，不检查未消费头中的地址或对齐标签。
fn file_range(offset: u64, length: usize, size: u64) -> Result<(), ExecError> {
    let end = offset.checked_add(length as u64).ok_or(ExecError::Format)?;
    if size < end {
        return Err(ExecError::Format);
    }
    Ok(())
}

/// 解释器路径使用现有路径长度 ABI，并且必须完整以唯一 NUL 终止。
fn read_interpreter(file: &FileRef, ph: &ProgramHeader) -> Result<Vec<u8>, ExecError> {
    if ph.filesz < 2 || MAX_PATH_BYTES < ph.filesz {
        return Err(ExecError::Format);
    }
    let mut pathname = Vec::new();
    pathname
        .try_reserve_exact(ph.filesz)
        .map_err(|_| ExecError::NoMemory)?;
    pathname.resize(ph.filesz, 0);
    read_exact(file, ph.offset, &mut pathname)?;
    if pathname.pop() != Some(0) || pathname.contains(&0) {
        return Err(ExecError::Format);
    }
    Ok(pathname)
}

/// 公倍数允许非幂二声明，仅在真实布局对齐无法表示时失败。
fn common_alignment(left: usize, right: usize) -> Result<usize, ExecError> {
    let (mut a, mut b) = (left, right);
    while b != 0 {
        (a, b) = (b, a % b);
    }
    (left / a).checked_mul(right).ok_or(ExecError::Format)
}

/// 通用向上对齐不依赖对齐值为二的幂。
fn align_up(address: usize, alignment: usize) -> Result<usize, ExecError> {
    let remainder = address % alignment;
    if remainder == 0 {
        Ok(address)
    } else {
        address
            .checked_add(alignment - remainder)
            .ok_or(ExecError::Format)
    }
}

/// 计划只借用候选映像，真正发表的 VMA 和 Mm 独立持有对应文件。
struct LoadSegment<'a> {
    file: &'a FileRef,
    segment: &'a Segment,
}

#[derive(Clone, Copy)]
enum Source<'a> {
    Anonymous,
    File(&'a FileRef, u64),
    Copy,
}

struct Mapping<'a> {
    start: usize,
    end: usize,
    flags: u32,
    source: Source<'a>,
}

impl Mapping<'_> {
    /// 相邻区间仅在权限和真实来源连续时合并，不跨过空洞。
    fn joins(&self, next: &Self) -> bool {
        if self.end != next.start || self.flags != next.flags {
            return false;
        }
        match (self.source, next.source) {
            (Source::Anonymous, Source::Anonymous) | (Source::Copy, Source::Copy) => true,
            (Source::File(left, offset), Source::File(right, next_offset)) => {
                same_file(left, right) && offset + (self.end - self.start) as u64 == next_offset
            }
            _ => false,
        }
    }
}

/// 不同 OFD 仍可指向同一 canonical inode，文件字节来源按该真实身份比较。
fn same_file(left: &FileRef, right: &FileRef) -> bool {
    Arc::ptr_eq(left, right)
        || left
            .mapping_inode()
            .expect("ELF source lost mapping inode")
            .ptr_eq(
                right
                    .mapping_inode()
                    .expect("ELF source lost mapping inode"),
            )
}

/// 两映像先组成唯一非重叠计划，再一次安装；后声明字节及零尾覆盖此前内容。
pub(super) fn map_images(
    mm: &mut Mm,
    file: &FileRef,
    main: &mut Elf,
    interpreter: Option<(&FileRef, &Elf)>,
) -> Result<(), ExecError> {
    let count = main.segments.len() + interpreter.map_or(0, |(_, image)| image.segments.len());
    let mut loads = Vec::new();
    loads
        .try_reserve_exact(count)
        .map_err(|_| ExecError::NoMemory)?;
    loads.extend(
        main.segments
            .iter()
            .map(|segment| LoadSegment { file, segment }),
    );
    if let Some((file, image)) = interpreter {
        loads.extend(
            image
                .segments
                .iter()
                .map(|segment| LoadSegment { file, segment }),
        );
    }
    let phdr = find_phdr(&loads, file, main);
    let plan = mapping_plan(&loads)?;
    for mapping in plan {
        let current = permissions(mapping.flags);
        match mapping.source {
            Source::Anonymous => {
                mm.map_anonymous(mapping.start, mapping.end - mapping.start, current, RWX)?
            }
            Source::File(file, offset) => mm.map_file(
                mapping.start,
                mapping.end - mapping.start,
                current,
                RWX,
                Arc::clone(file),
                offset,
                true,
            )?,
            Source::Copy => {
                for base in (mapping.start..mapping.end).step_by(PAGE_SIZE) {
                    map_edge(mm, &loads, base, current)?;
                }
            }
        }
    }
    main.phdr = phdr;
    Ok(())
}

/// 只有段起止及文件尾所在页能改变来源；计划空间随段数增长，不随跨度增长。
fn mapping_plan<'a>(loads: &[LoadSegment<'a>]) -> Result<Vec<Mapping<'a>>, ExecError> {
    let mut boundaries = Vec::new();
    let capacity = loads.len().checked_mul(3 * 2).ok_or(ExecError::NoMemory)?;
    boundaries
        .try_reserve_exact(capacity)
        .map_err(|_| ExecError::NoMemory)?;
    for load in loads {
        for address in [load.segment.start, load.segment.file_end, load.segment.end] {
            boundaries.push(page_start(address));
            boundaries.push(page_end(address)?);
        }
    }
    boundaries.sort_unstable();
    boundaries.dedup();
    let mut plan: Vec<Mapping<'a>> = Vec::new();
    plan.try_reserve_exact(boundaries.len())
        .map_err(|_| ExecError::NoMemory)?;
    for pair in boundaries.windows(2) {
        let Some((flags, source)) = page_plan(loads, pair[0]) else {
            continue;
        };
        let next = Mapping {
            start: pair[0],
            end: pair[1],
            flags,
            source,
        };
        if let Some(previous) = plan.last_mut() {
            if previous.joins(&next) {
                previous.end = next.end;
                continue;
            }
        }
        plan.push(next);
    }
    Ok(plan)
}

/// 返回当前位置的最终文件来源和下一个字节边界，BSS 与页内空洞都提供零。
fn contents_at<'a>(
    loads: &[LoadSegment<'a>],
    address: usize,
    end: usize,
) -> (usize, Option<(&'a FileRef, u64)>) {
    let mut stop = end;
    let mut source = None;
    for load in loads {
        let segment = load.segment;
        for boundary in [segment.start, segment.file_end, segment.end] {
            if address < boundary {
                stop = min(stop, boundary);
            }
        }
        if segment.start <= address && address < segment.end {
            source = if address < segment.file_end {
                Some((load.file, segment.offset + (address - segment.start) as u64))
            } else {
                None
            };
        }
    }
    (stop, source)
}

/// 页权限取所有参与声明的并集，只有最终整页同源对齐文件才能延迟文件缺页。
fn page_plan<'a>(loads: &[LoadSegment<'a>], base: usize) -> Option<(u32, Source<'a>)> {
    let end = base + PAGE_SIZE;
    let mut flags = 0;
    let mut present = false;
    for load in loads {
        let segment = load.segment;
        if segment.start < end && base < segment.end {
            present = true;
            flags |= segment.flags & (PF_R | PF_W | PF_X);
        }
    }
    if !present {
        return None;
    }
    let (_, first) = contents_at(loads, base, end);
    let mut cursor = base;
    while cursor < end {
        let (stop, source) = contents_at(loads, cursor, end);
        let same = match (first, source) {
            (None, None) => true,
            (Some((file, offset)), Some((next_file, next_offset))) => {
                same_file(file, next_file)
                    && offset.checked_add((cursor - base) as u64) == Some(next_offset)
            }
            _ => false,
        };
        if !same {
            return Some((flags, Source::Copy));
        }
        cursor = stop;
    }
    let source = match first {
        None => Source::Anonymous,
        Some((file, offset)) if offset % PAGE_SIZE as u64 == 0 => Source::File(file, offset),
        Some(_) => Source::Copy,
    };
    Some((flags, source))
}

/// 仅实际保留原文件程序头字节的映射产生 AT_PHDR，缺失保持 None，不施加 bias。
fn find_phdr(loads: &[LoadSegment<'_>], file: &FileRef, image: &Elf) -> Option<usize> {
    if image.table_bytes == 0 {
        return None;
    }
    image.segments.iter().find_map(|segment| {
        let address = if segment.offset <= image.phoff {
            segment
                .start
                .checked_add((image.phoff - segment.offset) as usize)
        } else {
            segment
                .start
                .checked_sub((segment.offset - image.phoff) as usize)
        }?;
        let end = address.checked_add(image.table_bytes)?;
        let mut cursor = address;
        while cursor < end {
            let (stop, source) = contents_at(loads, cursor, end);
            let (mapped_file, offset) = source?;
            if !same_file(file, mapped_file) || offset != image.phoff + (cursor - address) as u64 {
                return None;
            }
            cursor = stop;
        }
        Some(address)
    })
}

/// 合成页只分配必要的边缘或不能直映的来源，文件读取失败时仍由候选拥有归还原页。
fn map_edge(
    mm: &mut Mm,
    loads: &[LoadSegment<'_>],
    base: usize,
    current: PagePermissions,
) -> Result<(), ExecError> {
    let mut pages = alloc_pages(0, true).ok_or(ExecError::NoMemory)?;
    {
        // SAFETY: pages 独占已完整清零且尚未映射的 order0 页；此切片只在本次填充中存在，
        // 没有用户、设备或其他内核访问者，填充结束后才把原页转交 Mm。
        let bytes =
            unsafe { core::slice::from_raw_parts_mut(pages.as_mut_ptr().cast::<u8>(), PAGE_SIZE) };
        let mut cursor = base;
        let end = base + PAGE_SIZE;
        while cursor < end {
            let (stop, source) = contents_at(loads, cursor, end);
            if let Some((file, offset)) = source {
                read_exact(file, offset, &mut bytes[cursor - base..stop - base])?;
            }
            cursor = stop;
        }
    }
    // SAFETY: 整页由初始清零及最终来源的精确文件读取完整初始化；所有原始切片已结束，
    // mm 尚未发表，pages 唯一拥有只消费一次，成功后数据和页表生命周期由 Mm 管理。
    unsafe { mm.map_owned(base, pages, current, RWX) }?;
    Ok(())
}

/// 写权限转换时表达 RV 页表的真实 R 要求，不以 W-only 标签拒绝映像。
fn permissions(flags: u32) -> PagePermissions {
    let write = flags & PF_W != 0;
    PagePermissions {
        read: flags & PF_R != 0 || (cfg!(target_arch = "riscv64") && write),
        write,
        execute: flags & PF_X != 0,
    }
}

/// 页下界用于把实际 LOAD 字节转换为硬件页范围。
fn page_start(address: usize) -> usize {
    address & !(PAGE_SIZE - 1)
}

/// 页上界的加法仅在可表示时成立。
pub(super) fn page_end(address: usize) -> Result<usize, ExecError> {
    align_up(address, PAGE_SIZE)
}

/// 解码完整头中固定位置的小端 16 位字段。
fn word16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap())
}

/// 解码完整头中固定位置的小端 32 位字段。
fn word32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

/// 解码完整头中固定位置的小端 64 位字段。
fn word64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}
