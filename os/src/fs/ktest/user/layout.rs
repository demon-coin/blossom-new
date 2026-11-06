//! 用户态 ELF64 程序加载器、初始用户栈布局（`argc`/`argv`/`envp`/`auxv`）与私有写隔离集成测试。
//!
//! # 模块定位与职责
//! 本模块位于文件系统与用户态运行时集成测试层（`fs::ktest::user`），
//! 负责在内核将 ELF 可执行映像加载为 [`LoadedImage`]（尚未发布给调度器运行前）以及实际启动运行后，
//! 对用户地址空间（`Mm`）、页缓存（Page Cache）与初始用户栈进行全方位校验。
//! 上游由用户态测试套件调度（涵盖 `run`、`direct_low`、`direct_sparse`、`direct_overlap` 等测试入口），
//! 下游依赖 VFS 文件读写（`FileRef`）、内存文件系统（`MemoryFs`）、ELF 加载器（`LoadedImage`）
//! 以及虚拟内存空间与页面回收接口（`Mm` / `FolioRef` / `ReclaimAttempt`）。
//!
//! # 核心内存与初始用户栈布局
//! 64 位 Linux ABI 兼容的用户态初始栈（自栈指针 `sp` 向高地址生长）布局如下：
//! ```text
//! 低地址 (sp, 16 字节对齐)
//! +---------------------------------------------------------------+
//! | sp + 0   : argc = 2                                           |
//! | sp + 8   : argv[0] 指针 -> "/test/bin/user-smoke\0"           |
//! | sp + 16  : argv[1] 指针 -> "raw\0"                            |
//! | sp + 24  : argv[2] = NULL (0)                                 |
//! | sp + 32  : envp[0] 指针 -> "USER_SMOKE=1\0"                   |
//! | sp + 40  : envp[1] = NULL (0)                                 |
//! +---------------------------------------------------------------+
//! | sp + 48  : auxv[0] (a_type, a_val) 辅助向量表起始             |
//! |   ...    : AT_PHDR(3), AT_PHENT(4), AT_PHNUM(5), AT_PAGESZ(6),|
//! |            AT_ENTRY(9), AT_HWCAP(16), AT_CLKTCK(17),          |
//! |            AT_RANDOM(25), AT_EXECFN(31) ...                   |
//! |   ...    : AT_NULL (0, 0) 辅助向量终止项                      |
//! +---------------------------------------------------------------+
//! | 环境变量字符串、参数字符串与 16 字节随机数载荷区              |
//! +---------------------------------------------------------------+
//! 高地址 (USER_STACK_TOP)
//! ```
//!
//! 此外，本模块还通过内联汇编模板（`user_elf_direct_start` ~ `user_elf_direct_end`）动态拼装
//! 特殊边界的 ELF64 映像（写入 `tmpfs` 的 `/plain/direct`），验证：
//! - 低地址入口（`0x0` 页零映射与 `0x100` 非页对齐入口）及跨页扩展程序头表；
//! - 超大稀疏 BSS 段（`64 MiB + 4 KiB`）的按需缺页（Demand Paging）与未触碰页零驻留；
//! - 多个重叠 `PT_LOAD` 段的文件数据与 BSS 清零按声明顺序的后段覆盖前段规则。
//!
//! # 锁顺序与并发契约
//! - 访问用户地址空间一律通过 `mm.copyin` / `mm.copyout` / `mm.read_usize` 等安全访问接口完成，
//!   不在内核中直接构造指向用户可写页面的 Rust 切片引用。
//! - 验证私有写隔离（[`verify_private_write`]）时，先通过 `file_page` 获取底层文件页缓存的 `FolioRef`
//!   强引用以防止测试期间被内存回收器驱逐，随后再调用 `mm.copyout` 触发写时复制或私有页写入，避免锁重入。

use alloc::vec::Vec;
use core::cmp::min;
use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;

use crate::block::AccessMode;
use crate::fs::ktest::user::{UserOwner, PROGRAM};
use crate::fs::memfs::{Filesystem, MemoryFs, MemoryFsKind};
use crate::fs::{open_path_with_flags, FileIoContext, FileRef, ResolveMode};
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::Mm;
use crate::mm::{FolioRef, PAGE_SIZE};
use crate::task;
use crate::task::exec::LoadedImage;
use ::ktest::{TestContext, TestError, TestResult};

/// 从磁盘 ELF 程序头表解析出的单个 `PT_LOAD` 可加载段描述符。
#[derive(Clone, Copy)]
struct Segment {
    /// 段内容在 ELF 文件中的起始字节偏移（`p_offset`）。
    offset: u64,
    /// 段在用户虚拟地址空间中的起始虚拟地址（`p_vaddr`）。
    start: usize,
    /// 段在 ELF 文件中所占的字节长度（`p_filesz`）。
    filesz: usize,
    /// 段在用户虚拟内存中映射的总字节长度（`p_memsz`，超出 `filesz` 的部分为清零的 BSS 区）。
    memsz: usize,
}

/// 受测三程序头 ELF 可执行文件的关键布局元数据。
struct Layout {
    /// 程序入口虚拟地址（`e_entry`）。
    entry: usize,
    /// 程序头表在用户虚拟地址空间中的映射地址（`AT_PHDR`）。
    phdr: usize,
    /// 两个 `PT_LOAD` 段描述符：依次为只读可执行代码段（`segments[0]`）与可读写数据/BSS 段（`segments[1]`）。
    segments: [Segment; 2],
}

/// 包装底层 `Result`，在出错时向测试上下文输出包含操作名称、目标地址和具体错误的诊断日志，并转换为 [`TestError`]。
fn checked<T, E: Debug>(
    context: &TestContext<'_>,
    operation: &'static str,
    address: u64,
    result: Result<T, E>,
) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!(
            "user-layout operation={} address={:#x} error={:?}",
            operation, address, error
        ));
        TestError(operation)
    })
}

/// 从文件指定偏移 `offset` 处精确读取 `dst.len()` 字节，若发生错误或短读（Short Read）则触发测试失败。
fn read_file(context: &TestContext<'_>, file: &FileRef, offset: u64, dst: &mut [u8]) -> TestResult {
    let count = checked(
        context,
        "ELF positional read failed",
        offset,
        file.read_at(offset, dst),
    )?;
    assert_eq!(
        count,
        dst.len(),
        "ELF short read at file offset {offset:#x}"
    );
    Ok(())
}

/// 通过 `Mm::copyin` 从尚未投入运行的用户地址空间 `address` 处读取指定长度字节到内核缓冲区 `dst`。
fn read_mm(context: &TestContext<'_>, mm: &Mm, address: usize, dst: &mut [u8]) -> TestResult {
    let progress = mm.copyin(dst, address);
    if progress.error.is_some() || progress.copied != dst.len() {
        context.diagnostic(format_args!(
            "user-layout copyin address={:#x} requested={} copied={} error={:?}",
            address,
            dst.len(),
            progress.copied,
            progress.error
        ));
    }
    assert_eq!(progress.error, None, "ELF copyin failed at {address:#x}");
    assert_eq!(
        progress.copied,
        dst.len(),
        "ELF copyin incomplete at {address:#x}"
    );
    Ok(())
}

/// 从已读入内存的 ELF 元数据字节切片 `bytes` 的 `offset` 偏移处解码一个 64 位小端无符号整数。
fn elf_word(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}

/// 读取并校验磁盘上受测 ELF64 可执行文件的 ELF 头与程序头表（包含 1 个 `PT_PHDR` 和 2 个 `PT_LOAD`），提取 [`Layout`] 元数据。
fn layout(context: &TestContext<'_>, file: &FileRef) -> Result<Layout, TestError> {
    // Step 1: 读取 64 字节 ELF64 文件头，校验魔数、64 位小端标识以及程序头表项大小（56）和数量（3）。
    let mut header = [0; 64];
    read_file(context, file, 0, &mut header)?;
    assert_eq!(
        &header[..7],
        b"\x7fELF\x02\x01\x01",
        "disk ELF identification"
    );
    assert_eq!(
        u16::from_le_bytes(header[54..56].try_into().unwrap()),
        56,
        "disk ELF PHENT"
    );
    assert_eq!(
        u16::from_le_bytes(header[56..58].try_into().unwrap()),
        3,
        "disk ELF PHNUM"
    );

    // Step 2: 读取全部 3 个程序头表项，校验第 0 项为 PT_PHDR，后 2 项依次为 R-X 代码段与 RW- 数据段。
    let phoff = elf_word(&header, 32);
    let mut table = [0; 168];
    read_file(context, file, phoff, &mut table)?;
    assert_eq!(
        u32::from_le_bytes(table[..4].try_into().unwrap()),
        6,
        "disk PT_PHDR"
    );
    let segments = core::array::from_fn(|index| {
        let bytes = &table[(index + 1) * 56..(index + 2) * 56];
        assert_eq!(
            u32::from_le_bytes(bytes[..4].try_into().unwrap()),
            1,
            "disk PT_LOAD index={index}"
        );
        assert_eq!(
            u32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            if index == 0 { 5 } else { 6 },
            "disk PT_LOAD permissions index={index}"
        );
        Segment {
            offset: elf_word(bytes, 8),
            start: elf_word(bytes, 16) as usize,
            filesz: elf_word(bytes, 32) as usize,
            memsz: elf_word(bytes, 40) as usize,
        }
    });
    // Step 3: 校验 PT_PHDR 位于代码段文件映射范围内，且数据段具备非页对齐边界及跨越多页的初始化数据与 BSS。
    let text = segments[0];
    let phdr = text.start
        + usize::try_from(
            phoff
                .checked_sub(text.offset)
                .expect("PHDR precedes text file range"),
        )
        .expect("PHDR offset exceeds target word");
    assert_eq!(
        elf_word(&table, 16) as usize,
        phdr,
        "PT_PHDR address from real LOAD"
    );
    assert_eq!(
        elf_word(&table, 32),
        table.len() as u64,
        "PT_PHDR file bytes"
    );
    assert!(
        phdr + table.len() <= text.start + text.filesz,
        "PHDR is outside text file bytes"
    );

    let data = segments[1];
    assert!(data.filesz < data.memsz, "real writable LOAD has no BSS");
    assert_ne!(
        data.start & (PAGE_SIZE - 1),
        0,
        "real data start lost page edge"
    );
    assert_ne!(
        (data.start + data.filesz) & (PAGE_SIZE - 1),
        0,
        "real data end lost page edge"
    );
    assert!(
        2 * PAGE_SIZE < data.filesz,
        "real file data does not cross pages"
    );
    assert!(
        2 * PAGE_SIZE < data.memsz - data.filesz,
        "real BSS does not cross pages"
    );
    assert_ne!(
        (data.start + data.memsz) & (PAGE_SIZE - 1),
        0,
        "real BSS end lost page edge"
    );
    Ok(Layout {
        entry: elf_word(&header, 24) as usize,
        phdr,
        segments,
    })
}

/// 逐块比对已加载映像（`image.mm`）中所有 `PT_LOAD` 段的虚拟内存字节与磁盘文件对应字节（以及 BSS 段的零填充字节）是否完全一致。
fn verify_segments(
    context: &TestContext<'_>,
    file: &FileRef,
    image: &LoadedImage,
    layout: &Layout,
) -> TestResult {
    assert_eq!(
        image.context.pc, layout.entry,
        "loaded entry differs from disk ELF"
    );
    let mut actual = [0; 512];
    let mut expected = [0; 512];
    for segment in &layout.segments {
        let mut cursor = 0;
        while cursor < segment.memsz {
            let count = min(actual.len(), segment.memsz - cursor);
            expected[..count].fill(0);
            let file_count = min(count, segment.filesz.saturating_sub(cursor));
            if 0 < file_count {
                read_file(
                    context,
                    file,
                    segment.offset + cursor as u64,
                    &mut expected[..file_count],
                )?;
            }
            read_mm(
                context,
                &image.mm,
                segment.start + cursor,
                &mut actual[..count],
            )?;
            for index in 0..count {
                assert_eq!(
                    actual[index],
                    expected[index],
                    "LOAD byte mismatch va={:#x} file_offset={:#x} filesz={} memsz={}",
                    segment.start + cursor + index,
                    segment.offset + (cursor + index) as u64,
                    segment.filesz,
                    segment.memsz
                );
            }
            cursor += count;
        }
    }
    Ok(())
}

/// 从 64 位用户地址空间的指定虚拟地址 `address` 处读取一个机器字（`usize`）。
fn stack_word(context: &TestContext<'_>, mm: &Mm, address: usize) -> Result<usize, TestError> {
    checked(
        context,
        "initial stack word read failed",
        address as u64,
        mm.read_usize(address),
    )
}

/// 验证用户栈地址 `address` 处的以 NUL 结尾的字节串与 `expected` 完全一致。
fn stack_string(context: &TestContext<'_>, mm: &Mm, address: usize, expected: &[u8]) -> TestResult {
    let mut bytes = [0; 32];
    assert!(
        expected.len() < bytes.len(),
        "fixture string exceeds local comparison buffer"
    );
    assert_ne!(address, 0, "initial stack string pointer is NULL");
    read_mm(context, mm, address, &mut bytes[..expected.len() + 1])?;
    assert_eq!(
        &bytes[..expected.len()],
        expected,
        "initial stack string at {address:#x}"
    );
    assert_eq!(
        bytes[expected.len()],
        0,
        "initial stack string lacks NUL at {address:#x}"
    );
    Ok(())
}

/// 校验已加载映像的初始用户栈指针（`sp`）对齐、`argc`/`argv`/`envp` 布局以及辅助向量表（`auxv`）各项键值与硬件能力标志。
fn verify_stack(context: &TestContext<'_>, image: &LoadedImage, layout: &Layout) -> TestResult {
    // Step 1: 获取架构相关的初始用户栈指针寄存器，验证 16 字节对齐以及 argc、argv 和 envp 内容。
    #[cfg(target_arch = "riscv64")]
    let sp = image.context.gpr[2];
    #[cfg(target_arch = "loongarch64")]
    let sp = image.context.gpr[3];
    assert_eq!(sp & 15, 0, "initial SP is not 16-byte aligned: {sp:#x}");
    assert_eq!(stack_word(context, &image.mm, sp)?, 2, "initial argc");
    stack_string(
        context,
        &image.mm,
        stack_word(context, &image.mm, sp + 8)?,
        PROGRAM,
    )?;
    stack_string(
        context,
        &image.mm,
        stack_word(context, &image.mm, sp + 16)?,
        b"raw",
    )?;
    assert_eq!(
        stack_word(context, &image.mm, sp + 24)?,
        0,
        "argv terminator"
    );
    stack_string(
        context,
        &image.mm,
        stack_word(context, &image.mm, sp + 32)?,
        b"USER_SMOKE=1",
    )?;
    assert_eq!(
        stack_word(context, &image.mm, sp + 40)?,
        0,
        "envp terminator"
    );

    // Step 2: 根据当前架构独立计算预期的 AT_HWCAP 硬件能力掩码（LoongArch64 直接读取硬件 CPUCFG2 寄存器）。
    #[cfg(target_arch = "riscv64")]
    let hwcap = (1 << 8) | (1 << 12) | 1 | (1 << 5) | (1 << 3) | (1 << 2);
    #[cfg(target_arch = "loongarch64")]
    let hwcap = {
        // 直接读取 CPUCFG2 硬件配置字并按扩展依赖链（FP -> LSX -> LASX）推导预期的用户态 HWCAP 位。
        let config = crate::arch::cpucfg(2);
        let fp = config & 1 != 0;
        let lsx = fp && config & (1 << 6) != 0;
        let lasx = lsx && config & (1 << 7) != 0;
        1 | (usize::from(fp) << 3) | (usize::from(lsx) << 4) | (usize::from(lasx) << 5)
    };
    // Step 3: 扫描栈上的辅助向量表（auxv），验证所有必需项、AT_RANDOM 栈内范围、AT_EXECFN 路径以及 AT_NULL 终止符。
    let required = [
        (3, layout.phdr),
        (4, 56),
        (5, 3),
        (6, PAGE_SIZE),
        (7, 0),
        (8, 0),
        (9, layout.entry),
        (11, 0),
        (12, 0),
        (13, 0),
        (14, 0),
        (16, hwcap),
        (17, 100),
        (23, 0),
    ];
    let mut found = [false; 14];
    let mut execfn_found = false;
    let mut random_found = false;
    for slot in 0..32 {
        let address = sp + 48 + slot * 16;
        let kind = stack_word(context, &image.mm, address)?;
        let value = stack_word(context, &image.mm, address + 8)?;
        if kind == 0 {
            assert_eq!(value, 0, "AT_NULL value");
            for (index, (kind, _)) in required.iter().enumerate() {
                assert!(found[index], "missing required auxv kind={}", kind);
            }
            assert!(execfn_found, "missing AT_EXECFN");
            assert!(random_found, "missing AT_RANDOM");
            return Ok(());
        }
        assert_ne!(kind, 33, "auxv declares unavailable vDSO");
        if kind == 25 {
            assert!(!random_found, "duplicate AT_RANDOM");
            random_found = true;
            assert!(
                sp <= value && value <= crate::arch::USER_STACK_TOP - 16,
                "AT_RANDOM is outside the initialized user stack: {:#x}",
                value
            );
            let mut bytes = [0; 16];
            read_mm(context, &image.mm, value, &mut bytes)?;
        }
        if kind == 31 {
            assert!(!execfn_found, "duplicate AT_EXECFN");
            execfn_found = true;
            stack_string(context, &image.mm, value, PROGRAM)?;
        }
        if let Some(index) = required.iter().position(|(expected, _)| *expected == kind) {
            assert!(!found[index], "duplicate required auxv kind={}", kind);
            found[index] = true;
            assert_eq!(value, required[index].1, "auxv kind={kind}");
        }
    }
    Err(TestError("initial auxv lacks bounded AT_NULL terminator"))
}

/// 对可执行文件中数据段起始页至文件末尾的全部内容建立字节快照，并持有所涉页缓存 [`FolioRef`] 强引用以防止测试期间被回收。
fn snapshot(
    context: &TestContext<'_>,
    file: &FileRef,
    data: Segment,
) -> Result<(u64, Vec<u8>, Vec<FolioRef>), TestError> {
    let start = data.offset & !(PAGE_SIZE as u64 - 1);
    let size = checked(context, "ELF stat failed", 0, file.stat())?.size;
    let count = usize::try_from(
        size.checked_sub(start)
            .expect("data page starts beyond ELF EOF"),
    )
    .map_err(|_| TestError("ELF cache comparison range exceeds target word"))?;
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(count)
        .map_err(|_| TestError("NoMemory: ELF cache byte snapshot"))?;
    bytes.resize(count, 0);
    read_file(context, file, start, &mut bytes)?;

    let pages = count.div_ceil(PAGE_SIZE);
    let mut folios = Vec::new();
    folios
        .try_reserve_exact(pages)
        .map_err(|_| TestError("NoMemory: ELF cache folio owners"))?;
    let mut attempt = ReclaimAttempt::new();
    for index in 0..pages {
        let page = start / PAGE_SIZE as u64 + index as u64;
        folios.push(checked(
            context,
            "ELF cache folio read failed",
            page * PAGE_SIZE as u64,
            file.mapping_inode().unwrap().file_page(page, &mut attempt),
        )?);
    }
    Ok((start, bytes, folios))
}

/// 验证对已加载映像的数据段与 BSS 段执行用户空间写入（`copyout`）时具备私有写隔离性，不会污染底层文件页缓存。
fn verify_private_write(
    context: &TestContext<'_>,
    file: &FileRef,
    image: &LoadedImage,
    data: Segment,
) -> TestResult {
    // Step 1: 获取底层文件数据区的字节快照并锁定对应的页缓存 Folio。
    let (file_start, before, folios) = snapshot(context, file, data)?;
    let pattern = [0xa5; 512];
    let mut observed = [0; 512];
    let mut cursor = 0;
    while cursor < data.memsz {
        let count = min(pattern.len(), data.memsz - cursor);
        let address = data.start + cursor;
        let progress = image.mm.copyout(address, &pattern[..count]);
        if progress.error.is_some() || progress.copied != count {
            context.diagnostic(format_args!(
                "user-layout private-write address={:#x} requested={} copied={} error={:?}",
                address, count, progress.copied, progress.error
            ));
        }
        assert_eq!(
            progress.error, None,
            "private data/BSS copyout at {address:#x}"
        );
        assert_eq!(
            progress.copied, count,
            "private data/BSS copyout prefix at {address:#x}"
        );
        read_mm(context, &image.mm, address, &mut observed[..count])?;
        for (index, byte) in observed[..count].iter().enumerate() {
            assert_eq!(
                *byte,
                0xa5,
                "private data/BSS write readback va={:#x}",
                address + index
            );
        }
        cursor += count;
    }

    // Step 2: 重新通过文件系统接口读取底层文件，确认刚才对用户地址空间的写入未污染任何文件缓存字节。
    for (block, expected) in before.chunks(observed.len()).enumerate() {
        let offset = file_start + (block * observed.len()) as u64;
        read_file(context, file, offset, &mut observed[..expected.len()])?;
        for (index, byte) in expected.iter().enumerate() {
            assert_eq!(
                observed[index],
                *byte,
                "private write polluted file cache offset={:#x}",
                offset + index as u64
            );
        }
    }
    drop(folios);
    Ok(())
}

/// 运行标准磁盘 ELF 映像布局、初始栈、私有写隔离与用户态进程执行退出的综合测试。
///
/// - **测试目标**：验证 ELF 加载器正确映射代码段/数据段/BSS 段与初始栈，私有段写入与底层文件页缓存完全隔离，
///   且重新加载的干净映像可在独立用户任务中正常运行、经历硬件时钟中断并以状态码 `0` 退出。
/// - **场景步骤（Given-When-Then）**：
///   1. 加载受测 ELF 映像，依次校验段内容（[`verify_segments`]）、初始栈与 `auxv`（[`verify_stack`]）以及私有写隔离（[`verify_private_write`]），随后丢弃被写过的映像；
///   2. 重新加载干净映像并启动为用户进程，等待其退出；
///   3. 断言退出码为 `0`、执行期间观测到用户态中断与时钟中断计数大于 `0`，且用户任务 ID 与内核协调器任务不同。
pub(super) fn run(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
    fixture.prepare(context)?;
    let file = fixture.file(context, PROGRAM)?;
    let image = fixture.load(context)?;
    let layout = layout(context, &file)?;
    verify_segments(context, &file, &image, &layout)?;
    verify_stack(context, &image, &layout)?;
    verify_private_write(context, &file, &image, layout.segments[1])?;
    drop(image);

    drop(file);

    let pristine = fixture.load(context)?;
    let coordinator = task::current_task().id();
    fixture.start(context, pristine)?;
    let event = fixture.wait(context)?;
    context.diagnostic(format_args!(
        "user-layout actual-exit pid={} status={} user_interrupts={} task_id={:?} coordinator={:?}",
        event.pid, event.status, event.user_interrupts, event.task_id, coordinator
    ));
    assert_eq!(
        event.status, 0,
        "ordinary ELF user program did not exit normally with status 0"
    );
    assert!(
        0 < event.user_interrupts,
        "ordinary ELF never observed a real user interrupt"
    );
    assert!(
        0 < event.user_timer_interrupts,
        "ordinary ELF never observed a real user timer interrupt"
    );
    assert_ne!(
        event.task_id, coordinator,
        "user ELF ran as the kernel coordinator Task"
    );
    Ok(())
}

// 嵌入各架构用于直接构造裸 ELF 测试映像的汇编代码片段（定义符号 `user_elf_direct_start` 与 `user_elf_direct_end`）。
#[cfg(target_arch = "riscv64")]
core::arch::global_asm!(include_str!(
    "../../../../../tests/user/elf/direct-riscv64.S"
));
#[cfg(target_arch = "loongarch64")]
core::arch::global_asm!(include_str!(
    "../../../../../tests/user/elf/direct-loongarch64.S"
));

extern "C" {
    /// 内联汇编裸测试程序机器码的起始字节符号。
    static user_elf_direct_start: u8;
    /// 内联汇编裸测试程序机器码的结束字节符号（紧随最后一条指令之后）。
    static user_elf_direct_end: u8;
}

/// 用于在内存中动态拼装自定义 ELF64 文件的单个 `PT_LOAD` 段规格。
struct DirectSegment<'a> {
    /// 段在用户虚拟地址空间中的起始地址（`p_vaddr`）。
    start: usize,
    /// 段的 ELF 程序头权限与扩展标志（`p_flags`）。
    flags: u32,
    /// 写入 ELF 文件的段初始化字节切片（长度即 `p_filesz`）。
    bytes: &'a [u8],
    /// 段在用户虚拟内存中的总字节长度（`p_memsz`）。
    memsz: usize,
}

/// 传递给内联汇编裸测试程序的单字节内存读写校验项。
#[derive(Clone, Copy)]
struct ByteCheck {
    /// 待校验的用户虚拟地址。
    address: usize,
    /// 裸测试程序在覆写为 `0x7e` 之前预期在该地址读到的初始字节值。
    expected: u8,
}

/// 动态构造的裸 ELF 测试文件在 `tmpfs` 中的存放路径。
const DIRECT_PATH: &[u8] = b"/plain/direct";
/// 裸测试程序全部校验通过后调用 `exit(37)` 对应的 `wait` 状态字（`37 << 8`）。
const DIRECT_STATUS: i32 = 37 << 8;

/// 稀疏与重叠段测试中数据/BSS 段的默认起始虚拟地址。
const DIRECT_DATA: usize = 0x20000;
/// 稀疏段测试中超大 BSS 段的虚拟内存字节长度（`64 MiB + 1 页`）。
const LARGE_BSS: usize = 64 * 1024 * 1024 + PAGE_SIZE;
/// 稀疏段测试中远端可写数据段的起始虚拟地址。
const DISTANT_DATA: usize = 0x12000000;

/// 将内联汇编裸测试程序的机器码与末尾追加的 [`ByteCheck`] 校验表拼装为完整的段载荷字节序列。
///
/// 裸程序启动后会遍历该校验表：逐项检查 `address` 处的初始字节是否等于 `expected`，
/// 随后向该地址写入 `0x7e` 并读回确认，全部通过后以退出码 `37` 退出。
fn direct_payload(checks: &[ByteCheck]) -> Result<Vec<u8>, TestError> {
    let start = core::ptr::addr_of!(user_elf_direct_start);
    let end = core::ptr::addr_of!(user_elf_direct_end) as usize;
    // SAFETY:
    // 1. `user_elf_direct_start` 与 `user_elf_direct_end` 由上方 `global_asm!` 在同一只读代码节中相邻定义，`end >= start`；
    // 2. 该区间属于内核只读静态镜像，在整个内核生命周期内有效、对齐且不会被并发修改。
    let code = unsafe { core::slice::from_raw_parts(start, end - start as usize) };
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(code.len() + 8 + checks.len() * 16)
        .map_err(|_| TestError("NoMemory: direct ELF payload"))?;
    bytes.extend_from_slice(code);
    bytes.extend_from_slice(&(checks.len() as u64).to_le_bytes());
    for check in checks {
        bytes.extend_from_slice(&(check.address as u64).to_le_bytes());
        bytes.extend_from_slice(&[check.expected, 0x7e, 0, 0, 0, 0, 0, 0]);
    }
    Ok(bytes)
}

/// 根据给定的入口地址、可加载段列表与扩展头标志，在内存中生成合法的 ELF64 可执行文件字节流。
///
/// 当 `extended` 为 `true` 时，构造 `p_hentsize = 64`、`e_phnum = 80`（跨页程序头表）以及额外的非 `PT_LOAD` 厂商段头和节表字段，
/// 用于验证加载器忽略无关扩展头与未映射 `PT_PHDR` 的能力。
fn direct_elf(
    entry: usize,
    segments: &[DirectSegment<'_>],
    extended: bool,
) -> Result<Vec<u8>, TestError> {
    let phent = if extended { 64 } else { 56 };
    let phnum = if extended { 80 } else { segments.len() };
    let size = 8192
        + segments
            .iter()
            .map(|segment| segment.bytes.len())
            .sum::<usize>();
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(size)
        .map_err(|_| TestError("NoMemory: direct ELF file"))?;
    bytes.resize(size, 0);

    bytes[..7].copy_from_slice(b"\x7fELF\x02\x01\x01");
    bytes[16..18].copy_from_slice(&2u16.to_le_bytes());
    #[cfg(target_arch = "riscv64")]
    bytes[18..20].copy_from_slice(&243u16.to_le_bytes());
    #[cfg(target_arch = "loongarch64")]
    bytes[18..20].copy_from_slice(&258u16.to_le_bytes());
    bytes[20..24].copy_from_slice(&1u32.to_le_bytes());
    bytes[24..32].copy_from_slice(&(entry as u64).to_le_bytes());
    bytes[32..40].copy_from_slice(&64u64.to_le_bytes());
    bytes[52..54].copy_from_slice(&64u16.to_le_bytes());
    bytes[54..56].copy_from_slice(&(phent as u16).to_le_bytes());
    bytes[56..58].copy_from_slice(&(phnum as u16).to_le_bytes());
    if extended {
        bytes[7] = 0x61;
        bytes[8] = 3;
        bytes[40..48].copy_from_slice(&0x400000u64.to_le_bytes());
        bytes[48..52].copy_from_slice(&0x10000000u32.to_le_bytes());
        bytes[58..60].copy_from_slice(&64u16.to_le_bytes());
        bytes[60..62].copy_from_slice(&1u16.to_le_bytes());
        for index in segments.len()..phnum {
            let base = 64 + index * phent;
            bytes[base..base + 4].copy_from_slice(&0x70000001u32.to_le_bytes());
            bytes[base + 8..base + 16].copy_from_slice(&0x800000u64.to_le_bytes());
            bytes[base + 32..base + 40].copy_from_slice(&16u64.to_le_bytes());
        }
    }

    let mut offset = 8192;
    for (index, segment) in segments.iter().enumerate() {
        let base = 64 + index * phent;
        bytes[base..base + 4].copy_from_slice(&1u32.to_le_bytes());
        bytes[base + 4..base + 8].copy_from_slice(&segment.flags.to_le_bytes());
        for (field, value) in [
            (8, offset),
            (16, segment.start),
            (32, segment.bytes.len()),
            (40, segment.memsz),
            (48, 1),
        ] {
            bytes[base + field..base + field + 8].copy_from_slice(&(value as u64).to_le_bytes());
        }
        bytes[offset..offset + segment.bytes.len()].copy_from_slice(segment.bytes);
        offset += segment.bytes.len();
    }
    Ok(bytes)
}

/// 在 `/plain` 上挂载读写 `tmpfs`，将构造好的 ELF 字节写入 `0644` 权限的 `/plain/direct` 文件，关闭写入句柄后调用 ELF 加载器返回 [`LoadedImage`]。
fn prepare_direct(
    context: &TestContext<'_>,
    fixture: &mut UserOwner,
    bytes: &[u8],
) -> Result<LoadedImage, TestError> {
    fixture.prepare(context)?;
    let vfs = fixture.path.vfs();
    let fs_context = fixture.path.context();
    let covered = checked(
        context,
        "direct ELF mountpoint lookup failed",
        0,
        vfs.resolve(fs_context, b"/plain", ResolveMode::Follow),
    )?;
    let memory = checked(
        context,
        "direct ELF tmpfs preparation failed",
        0,
        MemoryFs::try_new(MemoryFsKind::Tmpfs, 0o755),
    )?;
    checked(
        context,
        "direct ELF tmpfs mount failed",
        0,
        vfs.mount(&covered, Filesystem::Memory(memory), AccessMode::ReadWrite),
    )?;

    let file = checked(
        context,
        "direct ELF creation failed",
        0,
        vfs.open_with(
            fs_context,
            fs_context.cwd(),
            DIRECT_PATH,
            (0, 0),
            Some((0o644, 0)),
            true,
            false,
            |path, _| open_path_with_flags(vfs, path, 2, FileIoContext::Kernel),
        ),
    )?;
    assert_eq!(
        checked(
            context,
            "direct ELF write failed",
            0,
            file.write_at(0, bytes)
        )?,
        bytes.len(),
        "direct ELF short write"
    );
    assert_eq!(
        checked(context, "direct ELF stat failed", 0, file.stat())?.mode & 0o777,
        0o644,
        "direct ELF file mode"
    );
    drop(file);
    fixture.load_named(context, DIRECT_PATH, b"raw")
}

/// 从已加载裸 ELF 映像的初始用户栈指针出发，跳过 `argc`、`argv` 和 `envp`，校验辅助向量表中的 `AT_PHDR`、`AT_PHENT`、`AT_PHNUM` 和 `AT_ENTRY`。
fn verify_direct_auxv(
    context: &TestContext<'_>,
    image: &LoadedImage,
    entry: usize,
    phent: usize,
    phnum: usize,
) -> TestResult {
    #[cfg(target_arch = "riscv64")]
    let sp = image.context.gpr[2];
    #[cfg(target_arch = "loongarch64")]
    let sp = image.context.gpr[3];
    let argc = stack_word(context, &image.mm, sp)?;
    let mut cursor = sp + (argc + 2) * 8;
    while stack_word(context, &image.mm, cursor)? != 0 {
        cursor += 8;
    }
    cursor += 8;

    let required = [(3, 0), (4, phent), (5, phnum), (9, entry)];
    let mut found = [false; 4];
    for _ in 0..32 {
        let kind = stack_word(context, &image.mm, cursor)?;
        let value = stack_word(context, &image.mm, cursor + 8)?;
        if kind == 0 {
            assert_eq!(found, [true; 4], "direct ELF missing auxv");
            return Ok(());
        }
        if let Some(index) = required.iter().position(|(expected, _)| *expected == kind) {
            assert!(!found[index], "direct ELF duplicate auxv");
            found[index] = true;
            assert_eq!(value, required[index].1, "direct ELF auxv kind={kind}");
        }
        cursor += 16;
    }
    Err(TestError("direct ELF auxv lacks AT_NULL"))
}

/// 启动已加载的裸 ELF 映像并等待其退出，断言退出状态为 [`DIRECT_STATUS`]，且所有受检地址处均保留了用户态写入的 `0x7e`。
fn execute_direct(
    context: &mut TestContext<'_>,
    fixture: &mut UserOwner,
    image: LoadedImage,
    checks: &[ByteCheck],
) -> TestResult {
    let mm = image.mm.clone();
    fixture.start(context, image)?;
    let event = fixture.wait(context)?;
    assert_eq!(event.status, DIRECT_STATUS, "direct ELF actual user exit");
    for check in checks {
        let mut actual = [0];
        read_mm(context, &mm, check.address, &mut actual)?;
        assert_eq!(
            actual,
            [0x7e],
            "direct ELF user store at {:#x}",
            check.address
        );
    }
    Ok(())
}

/// 验证低虚拟地址入口（`0x0` 零页或 `0x100` 非页对齐入口）及跨页扩展程序头表的 ELF 加载与执行。
///
/// - **测试目标**：确认 ELF 加载器允许映射虚拟地址 `0x0` 及页内偏移入口 `0x100`，且在程序头表跨页、未包含在任何 `PT_LOAD` 段中时，
///   能正确将 `AT_PHDR` 置为 `0` 并正常执行用户指令直至退出。
/// - **场景步骤（Given-When-Then）**：
///   1. 生成入口为 `entry`（`0x0` 或 `0x100`）的裸 ELF 文件并写入 `tmpfs` 加载；
///   2. 校验入口 PC、指令字节与初始栈 `auxv` 元数据；
///   3. 运行该用户进程并断言以状态码 `37`（[`DIRECT_STATUS`]）正常退出。
pub(super) fn direct_low(
    context: &mut TestContext<'_>,
    fixture: &mut UserOwner,
    entry: usize,
) -> TestResult {
    let payload = direct_payload(&[])?;
    let extended = entry == 0x100;
    let bytes = direct_elf(
        entry,
        &[DirectSegment {
            start: entry,
            flags: if extended { 0x10000005 } else { 5 },
            bytes: &payload,
            memsz: payload.len(),
        }],
        extended,
    )?;

    let image = prepare_direct(context, fixture, &bytes)?;
    assert_eq!(image.context.pc, entry, "direct ELF low entry");
    let mut actual = [0; 16];
    read_mm(context, &image.mm, entry, &mut actual)?;
    assert_eq!(&actual, &payload[..16], "direct ELF low instruction bytes");
    verify_direct_auxv(
        context,
        &image,
        entry,
        if extended { 64 } else { 56 },
        if extended { 80 } else { 1 },
    )?;
    execute_direct(context, fixture, image, &[])
}

/// 在不触发缺页分配的前提下遍历页表，统计超大 BSS 段虚拟地址区间内当前已实际建立物理页映射的驻留页数。
fn bss_resident(mm: &Mm) -> usize {
    (DIRECT_DATA..DIRECT_DATA + LARGE_BSS)
        .step_by(PAGE_SIZE)
        .filter(|address| mm.mapping(*address).is_some())
        .count()
}

/// 验证超大稀疏 BSS 段（`64 MiB + 4 KiB`）与大跨度虚拟地址空洞的按需缺页（Demand Paging）加载行为。
///
/// - **测试目标**：确认 ELF 加载器对超大 BSS 段采用惰性按需映射而非在 `execve` 时一次性分配全部物理页，
///   且用户程序运行期间仅对实际访问过的 3 个 BSS 页面建立物理映射，段间地址空洞始终保持未映射。
/// - **场景步骤（Given-When-Then）**：
///   1. 构造包含 `64 MiB + 4 KiB` 纯 BSS 段与远端数据段的 ELF 文件并完成加载，断言初始 BSS 驻留页数为 `0` 且中间空洞未映射；
///   2. 运行用户程序，仅访问 BSS 首字节、中间 32 MiB 处、末字节及远端数据段首字节；
///   3. 进程退出后断言 BSS 驻留页数精确等于 `3`，且段间空洞仍无映射。
pub(super) fn direct_sparse(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
    let checks = [
        ByteCheck {
            address: DIRECT_DATA,
            expected: 0,
        },
        ByteCheck {
            address: DIRECT_DATA + 32 * 1024 * 1024,
            expected: 0,
        },
        ByteCheck {
            address: DIRECT_DATA + LARGE_BSS - 1,
            expected: 0,
        },
        ByteCheck {
            address: DISTANT_DATA,
            expected: 0x7b,
        },
    ];
    let payload = direct_payload(&checks)?;
    let bytes = direct_elf(
        0x10000,
        &[
            DirectSegment {
                start: 0x10000,
                flags: 5,
                bytes: &payload,
                memsz: payload.len(),
            },
            DirectSegment {
                start: DIRECT_DATA,
                flags: 6,
                bytes: &[],
                memsz: LARGE_BSS,
            },
            DirectSegment {
                start: DISTANT_DATA,
                flags: 6,
                bytes: &[0x7b],
                memsz: PAGE_SIZE,
            },
        ],
        false,
    )?;
    let image = prepare_direct(context, fixture, &bytes)?;
    let mm = image.mm.clone();
    assert_eq!(bss_resident(&mm), 0, "large BSS was eagerly populated");
    assert!(
        mm.mapping(0x8000000).is_none(),
        "sparse ELF hole was populated"
    );

    execute_direct(context, fixture, image, &checks)?;
    assert_eq!(
        bss_resident(&mm),
        3,
        "large BSS residency differs from actual touches"
    );
    assert!(
        mm.mapping(0x8000000).is_none(),
        "sparse ELF hole became resident"
    );
    Ok(())
}

/// 验证多个虚拟地址范围相互重叠的 `PT_LOAD` 段在加载与运行时的字节覆盖优先级。
///
/// - **测试目标**：确认当多个 `PT_LOAD` 段在相同页或跨页区域发生重叠时，加载器严格按照程序头声明顺序依次应用每个段的文件内容与 BSS 清零区域（后声明的段覆盖先声明的段）。
/// - **场景步骤（Given-When-Then）**：
///   1. 构造 3 个在 `DIRECT_DATA` 区域存在多层部分重叠与 BSS 尾覆盖的数据段并加载映像；
///   2. 在启动前直接读取用户地址空间，断言整个重叠区域的所有字节与按序覆盖预期完全一致；
///   3. 启动用户程序校验各边界点初始值并覆写为 `0x7e`，退出后确认仅受检边界字节变为 `0x7e`，其余字节未受破坏。
pub(super) fn direct_overlap(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
    let addresses = [
        0x207ff, 0x20800, 0x2100f, 0x21010, 0x2102f, 0x21030, 0x2104f, 0x21050, 0x2181f, 0x21820,
        0x2187f, 0x21880,
    ];
    let mut expected = [0x11; 2 * PAGE_SIZE + 128];
    expected[0x800..0x1820].fill(0x22);
    expected[0x1820..0x1880].fill(0);
    expected[0x1010..0x1030].fill(0x33);
    expected[0x1030..0x1050].fill(0);
    let checks = addresses.map(|address| ByteCheck {
        address,
        expected: expected[address - DIRECT_DATA],
    });

    let payload = direct_payload(&checks)?;
    let bytes = direct_elf(
        0x10000,
        &[
            DirectSegment {
                start: 0x10000,
                flags: 5,
                bytes: &payload,
                memsz: payload.len(),
            },
            DirectSegment {
                start: DIRECT_DATA,
                flags: 4,
                bytes: &[0x11; 2 * PAGE_SIZE + 128],
                memsz: expected.len(),
            },
            DirectSegment {
                start: DIRECT_DATA + 0x800,
                flags: 6,
                bytes: &[0x22; PAGE_SIZE + 32],
                memsz: PAGE_SIZE + 128,
            },
            DirectSegment {
                start: DIRECT_DATA + 0x1010,
                flags: 4,
                bytes: &[0x33; 32],
                memsz: 64,
            },
        ],
        false,
    )?;

    let image = prepare_direct(context, fixture, &bytes)?;
    let mm = image.mm.clone();
    let mut actual = [0; 2 * PAGE_SIZE + 128];
    read_mm(context, &mm, DIRECT_DATA, &mut actual)?;
    assert_eq!(actual, expected, "overlapping LOAD file/BSS precedence");

    execute_direct(context, fixture, image, &checks)?;
    for address in addresses {
        expected[address - DIRECT_DATA] = 0x7e;
    }
    read_mm(context, &mm, DIRECT_DATA, &mut actual)?;
    assert_eq!(
        actual, expected,
        "overlapping LOAD user stores changed other bytes"
    );
    Ok(())
}
