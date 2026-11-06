//! ELF64 小端可执行映像解析与页级映射规划器。
//!
//! ## 模块职责与架构定位
//! 本模块是 `execve` 装载器的子模块（`crate::task::exec::elf`），负责解析主可执行文件与可选动态链接器（`PT_INTERP`）的 ELF64 头部及程序头表（Program Header Table），并将所有 `PT_LOAD` 段转换为无重叠的虚拟内存映射计划安装到目标 [`Mm`] 中。
//!
//! ## 核心机制：`PT_LOAD` 段边界离散化与三类页面映射规划
//!
//! 一个或多个 `PT_LOAD` 段可能出现页内未对齐、文件数据与 BSS 零尾共享同一物理页、甚至多个段在同一页内重叠覆盖的情况。本模块通过 [`mapping_plan`] 与 [`page_plan`] 将所有段的起止边界离散化，将每个虚拟页归类为三种 [`Source`] 之一：
//!
//! ```text
//!   ELF PT_LOAD 段内存视图：
//!   |<------------------- memsz ------------------->|
//!   |<------- filesz ------->|<--- BSS 零填充尾 --->|
//!   +-----+------------------+-----+----------------+-----+
//!   | 页0 |        页1..页K        |      页K+1     | 页M |
//!   +-----+------------------+-----+----------------+-----+
//!      │               │                   │           │
//!      ▼               ▼                   ▼           ▼
//!   Source::Copy   Source::File       Source::Copy  Source::Anonymous
//!   (首部未对齐页)  (整页对齐且同源，  (文件尾与BSS   (纯BSS匿名零页，
//!    预分配物理页   直接建立私有文件   交界页，预读   按需零页映射)
//!    读取文件片段)  VMA 按需缺页)      文件前缀+清零)
//! ```
//!
//! - **`Source::File`（整页文件直接映射）**：当且仅当整个虚拟页完全由同一个底层 Inode 的连续文件字节覆盖，且文件偏移量满足 `PAGE_SIZE` 对齐时，建立私有文件映射（`mm.map_file`），支持后续按需缺页加载。
//! - **`Source::Anonymous`（整页匿名零页映射）**：当整个虚拟页全部落在 `filesz..memsz`（BSS 段）或段内未覆盖的零填充区域时，建立匿名按需零页映射（`mm.map_anonymous`）。
//! - **`Source::Copy`（边缘/混合合成页）**：当页内包含段起止边缘、文件与 BSS 交界、非页对齐的文件偏移或被后续 `PT_LOAD` 段部分覆盖时，由 [`map_edge`] 立即分配清零物理页，按声明顺序将各段文件字节读入页内对应偏移（后声明的段覆盖先声明的段），再通过 `mm.map_owned` 直接挂入页表。
//!
//! ## 并发与内存安全契约
//! - **调用方保证写排斥**：调用 [`Elf::read`] 与 [`map_images`] 前，上游 [`super::load_elf`] 已经持有主程序与动态链接器文件的 `ExecAccess` 写排斥守卫，确保解析与映射期间底层文件大小和内容不会被并发修改。
//! - **合成页物理内存所有权**：在 [`map_edge`] 中分配的物理页在完成内核态文件读取后，立即销毁临时可变字节切片并将物理页唯一所有权移交给尚未发布的局部 `Mm`，不存在并发访问或悬垂指针风险。

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

/// ELF64 文件头（`Elf64_Ehdr`）的固定字节长度（64 字节）。
pub(super) const ELF_HEADER_BYTES: usize = 64;
/// ELF64 程序头表项（`Elf64_Phdr`）的最小固定解码长度（56 字节）。
const PHDR_BYTES: usize = 56;

/// `PT_LOAD`：需要装载到内存中的程序段。
const PT_LOAD: u32 = 1;
/// `PT_INTERP`：指定动态链接器（解释器）路径的程序段。
const PT_INTERP: u32 = 3;
/// `PT_GNU_STACK`：指定用户栈是否需要可执行权限的 GNU 扩展段。
const PT_GNU_STACK: u32 = 0x6474_e551;
/// `ET_DYN`：位置无关可执行文件（PIE）或共享对象类型。
const ET_DYN: u16 = 3;

/// `PF_R`：程序段可读标志位。
const PF_R: u32 = 4;
/// `PF_W`：程序段可写标志位。
const PF_W: u32 = 2;
/// `PF_X`：程序段可执行标志位。
const PF_X: u32 = 1;

/// 主程序为 `ET_DYN`（PIE）时使用的默认装载基地址下界（`0x2000_0000`）。
pub(super) const MAIN_DYN_BASE: usize = 0x2000_0000;
/// 动态链接器（`PT_INTERP`）为 `ET_DYN` 时使用的默认装载基地址下界（`0x3000_0000`）。
pub(super) const INTERP_DYN_BASE: usize = 0x3000_0000;

/// 解析后的 ELF64 可执行映像元数据与装载段列表。
pub(super) struct Elf {
    /// `PT_INTERP` 段指定的动态链接器路径（不含末尾 NUL 字节）。
    pub(super) interp: Option<Vec<u8>>,
    /// 位置无关映像（`ET_DYN`）附加的虚拟地址偏移量（`ET_EXEC` 固定为 `0`）。
    pub(super) bias: usize,
    /// 已加上 `bias` 偏移的非空 `PT_LOAD` 装载段列表（保持原程序头声明顺序）。
    segments: Vec<Segment>,
    /// 已加上 `bias` 偏移的程序入口虚拟地址（`e_entry + bias`）。
    pub(super) entry: usize,
    /// 映射到用户空间中的程序头表虚拟地址（用于填充 `AT_PHDR`；若程序头表未被完整原样映射则为 `None`）。
    pub(super) phdr: Option<usize>,
    /// 单个程序头表项的字节大小（`e_phentsize`，用于填充 `AT_PHENT`）。
    pub(super) phent: usize,
    /// 程序头表项总数（`e_phnum`，用于填充 `AT_PHNUM`）。
    pub(super) phnum: usize,
    /// 程序头表在文件中的起始字节偏移（`e_phoff`）。
    phoff: u64,
    /// 程序头表占用的总字节数（`phnum * phent`）。
    table_bytes: usize,
    /// 所有 `PT_LOAD` 段中最高的结束虚拟地址（用于计算初始 `brk` 用户堆起点）。
    pub(super) end: usize,
    /// 是否声明了可执行栈（由 `PT_GNU_STACK` 段的 `PF_X` 标志决定）。
    pub(super) executable_stack: bool,
}

/// 单个 `PT_LOAD` 段在虚拟地址空间与文件中的区间描述。
struct Segment {
    /// 段在用户虚拟地址空间中的起始地址（已含 `bias`，单位：字节）。
    start: usize,
    /// 段内由文件数据初始化的结束虚拟地址（`start + filesz`，单位：字节）。
    file_end: usize,
    /// 段在内存中的结束虚拟地址（`start + memsz`，`file_end..end` 为 BSS 零填充区，单位：字节）。
    end: usize,
    /// 段数据在源文件中的起始偏移量（单位：字节）。
    offset: u64,
    /// 段的原始权限标志位（`PF_R | PF_W | PF_X`）。
    flags: u32,
}

/// 解码后的单个 ELF64 程序头表项（`Elf64_Phdr`）。
struct ProgramHeader {
    /// 段类型（`p_type`，如 `PT_LOAD`、`PT_INTERP`、`PT_GNU_STACK`）。
    kind: u32,
    /// 段权限标志（`p_flags`，如 `PF_R`、`PF_W`、`PF_X`）。
    flags: u32,
    /// 段内容在文件中的字节偏移（`p_offset`）。
    offset: u64,
    /// 段的期望虚拟地址（`p_vaddr`）。
    address: usize,
    /// 段在文件中的字节长度（`p_filesz`）。
    filesz: usize,
    /// 段在内存中的字节长度（`p_memsz`）。
    memsz: usize,
    /// 段的地址对齐要求（`p_align`）。
    align: usize,
}

impl ProgramHeader {
    /// 从 56 字节的 ELF64 小端程序头原始字节数组中解码各字段。
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

    /// 校验 `PT_LOAD` 段的文件范围与内存大小关系（要求 `memsz >= filesz` 且不发生地址溢出），转换为 [`Segment`]。
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
    /// 校验 ELF64 文件头并遍历解析所有程序头表项，提取 `PT_LOAD` 段、`PT_INTERP` 解释器路径与 `PT_GNU_STACK` 属性。
    ///
    /// 对于 `ET_DYN` 位置无关映像，根据所有非空 `PT_LOAD` 段的最小公倍对齐要求向上对齐 `dyn_base` 作为装载基址偏移 `bias`。
    pub(super) fn read(
        file: &FileRef,
        header: &[u8; ELF_HEADER_BYTES],
        dyn_base: usize,
    ) -> Result<Self, ExecError> {
        // Step 1: 校验 64 位小端 ELF 魔数、头部尺寸以及程序头表在文件内的范围合法性。
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

        // Step 2: 顺序读取并解码每个程序头，收集非空 PT_LOAD 段、唯一 PT_INTERP 路径以及可执行栈标志。
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

        // Step 3: 计算 ET_DYN 映像的装载偏移 bias，将所有段地址重定位并校验是否超出用户地址空间上限。
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

    /// 检查给定的虚拟地址范围 `[start, end)` 是否与本映像任一 `PT_LOAD` 段占用的页范围重叠。
    pub(super) fn overlaps(&self, start: usize, end: usize) -> bool {
        self.segments.iter().any(|segment| {
            page_start(segment.start) < end
                && start < page_end(segment.end).expect("validated LOAD page end")
        })
    }
}

/// 校验文件偏移区间 `[offset, offset + length)` 是否未发生整数溢出且完全落在文件总大小 `size` 以内。
fn file_range(offset: u64, length: usize, size: u64) -> Result<(), ExecError> {
    let end = offset.checked_add(length as u64).ok_or(ExecError::Format)?;
    if size < end {
        return Err(ExecError::Format);
    }
    Ok(())
}

/// 根据 `PT_INTERP` 程序头从文件中读取动态链接器路径（要求以单个 NUL 结尾且内部不含额外 NUL 字节）。
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

/// 使用欧几里得算法计算两个对齐值的最小公倍数（支持非 2 的幂的段对齐声明）。
fn common_alignment(left: usize, right: usize) -> Result<usize, ExecError> {
    let (mut a, mut b) = (left, right);
    while b != 0 {
        (a, b) = (b, a % b);
    }
    (left / a).checked_mul(right).ok_or(ExecError::Format)
}

/// 将虚拟地址 `address` 向上对齐到 `alignment` 的整数倍（支持任意正整数对齐粒度）。
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

/// 绑定了源文件引用的待装载 `PT_LOAD` 段视图。
struct LoadSegment<'a> {
    /// 该段所属的可执行文件或动态链接器文件句柄。
    file: &'a FileRef,
    /// 该段的虚拟地址与文件偏移描述。
    segment: &'a Segment,
}

/// 规划后的虚拟内存区间的数据来源类型。
#[derive(Clone, Copy)]
enum Source<'a> {
    /// 纯匿名零页映射（如 BSS 段或段内零填充页）。
    Anonymous,
    /// 页对齐的连续文件背景映射（绑定源文件引用与页对齐的起始文件偏移）。
    File(&'a FileRef, u64),
    /// 需要立即分配物理页并按字节拼接填充的边缘或混合来源页。
    Copy,
}

/// 合并后的连续虚拟页映射规划项。
struct Mapping<'a> {
    /// 映射区间的起始虚拟地址（页对齐）。
    start: usize,
    /// 映射区间的结束虚拟地址（页对齐）。
    end: usize,
    /// 合并后的 ELF 段权限标志（`PF_R | PF_W | PF_X`）。
    flags: u32,
    /// 该区间的数据填充方式。
    source: Source<'a>,
}

impl Mapping<'_> {
    /// 判断当前映射区间 `self` 是否可以与紧邻的下一区间 `next` 合并为单个连续映射。
    ///
    /// 仅当两区间虚拟地址首尾相接、权限完全一致，且同为匿名映射、同为合成拷贝页，或指向同一底层 Inode 且文件偏移连续时才允许合并。
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

/// 判断两个打开文件引用是否指向同一个底层可映射 Inode。
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

/// 将主程序与可选动态链接器的所有 `PT_LOAD` 段统一规划为不重叠的页面映射并安装到 `mm` 中。
///
/// 若多个段在同一虚拟地址范围发生重叠，后声明的段字节（含 BSS 零尾）会覆盖先声明的段内容；
/// 同时检测主程序头表是否完整保留在映射中，以填写 `main.phdr`（供 `AT_PHDR` 使用）。
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

/// 提取所有 `PT_LOAD` 段的页对齐边界点，对每个相邻边界区间生成并合并页面映射规划 [`Mapping`]。
///
/// 由于只有段的起始地址、`file_end` 和结束地址所在页可能改变映射来源或权限，
/// 离散边界点数量仅与段数成线性关系（最多 `6 * loads.len()`），不随段跨越的虚拟地址长度增长。
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

/// 查询虚拟地址 `address` 处在 `[address, end)` 范围内最终生效的字节来源（文件偏移或 `None` 表示零字节）以及下一个内容变化边界地址。
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

/// 分析单个虚拟页 `[base, base + PAGE_SIZE)` 的合并权限与映射来源类型 [`Source`]。
///
/// - 若没有任何 `PT_LOAD` 段覆盖该页，返回 `None`；
/// - 若整页具有单一连续来源：全为零则返回 [`Source::Anonymous`]，来自同一文件且文件偏移按页对齐则返回 [`Source::File`]；
/// - 其余情况（包含段边缘、文件偏移未对齐或多段拼接）均返回 [`Source::Copy`]。
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

/// 查找主程序的程序头表在用户虚拟地址空间中的映射地址（用于设置辅助向量 `AT_PHDR`）。
///
/// 仅当某个 `PT_LOAD` 段完整覆盖了文件的 `[phoff, phoff + table_bytes)` 区间，且该区间未被后续段覆盖改写时，才返回对应的用户虚拟地址。
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

/// 为单个边缘或混合来源虚拟页 `[base, base + PAGE_SIZE)` 分配物理页，按最终覆盖关系读入文件片段后挂入 `mm`。
fn map_edge(
    mm: &mut Mm,
    loads: &[LoadSegment<'_>],
    base: usize,
    current: PagePermissions,
) -> Result<(), ExecError> {
    let mut pages = alloc_pages(0, true).ok_or(ExecError::NoMemory)?;
    {
        // SAFETY:
        // 1. `pages` 是刚刚通过 `alloc_pages(0, true)` 分配并清零的 order-0 物理页（恰为 `PAGE_SIZE` 字节），当前由本函数独占持有；
        // 2. `bytes` 可变切片的作用域严格限制在当前代码块内，期间该物理页尚未映射到任何页表，无其他 CPU、用户线程或 DMA 访问；
        // 3. 代码块结束时 `bytes` 切片立即销毁，之后才在下方将 `pages` 移交给 `mm.map_owned`。
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

    // SAFETY:
    // 1. `pages` 整页在分配时已清零，且上方代码块已将所有需要从文件加载的字节片段精确填入对应偏移，整页内存完全初始化；
    // 2. 指向 `pages` 的临时可变切片 `bytes` 已离开作用域销毁，且 `mm` 是尚未发布给任何运行任务的私有候选地址空间；
    // 3. `pages` 的唯一所有权在此处移交给 `mm` 管理；若 `read_exact` 在上方提前失败返回，`pages` 会随栈帧自动析构归还伙伴分配器。
    unsafe { mm.map_owned(base, pages, current, RWX) }?;
    Ok(())
}

/// 将 ELF 程序头权限标志（`PF_R`/`PF_W`/`PF_X`）转换为页表权限 [`PagePermissions`]。
///
/// 在 RISC-V 64 架构下，硬件页表规范不支持只写不可读（`W=1, R=0`）组合，因此若带有 `PF_W` 则自动开启读权限。
fn permissions(flags: u32) -> PagePermissions {
    let write = flags & PF_W != 0;
    PagePermissions {
        read: flags & PF_R != 0 || (cfg!(target_arch = "riscv64") && write),
        write,
        execute: flags & PF_X != 0,
    }
}

/// 将虚拟地址向下对齐所在页的起始边界。
fn page_start(address: usize) -> usize {
    address & !(PAGE_SIZE - 1)
}

/// 将虚拟地址向上对齐到页边界；若发生算术溢出则返回 [`ExecError::Format`]。
pub(super) fn page_end(address: usize) -> Result<usize, ExecError> {
    align_up(address, PAGE_SIZE)
}

/// 从字节切片指定偏移处读取小端序 `u16`。
fn word16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap())
}

/// 从字节切片指定偏移处读取小端序 `u32`。
fn word32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

/// 从字节切片指定偏移处读取小端序 `u64`。
fn word64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}
