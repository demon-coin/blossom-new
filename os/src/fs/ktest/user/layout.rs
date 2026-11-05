//! 从真实磁盘 ELF 验证未发表布局、私有写隔离与实际用户退出。

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

#[derive(Clone, Copy)]
struct Segment {
    offset: u64,
    start: usize,
    filesz: usize,
    memsz: usize,
}

struct Layout {
    entry: usize,
    phdr: usize,
    segments: [Segment; 2],
}

/// 保留底层具体错误及消费地址，框架失败消息只标识当前检查边界。
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

/// 单次按位置读取必须交付完整请求，短读保留文件位置与实际长度。
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

/// 经正式 uaccess 读取未 Ready 的 Mm，不创建用户内存的 Rust 切片。
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

/// 解码完整 ELF64 元数据的固定小端字段，调用位置均位于已读完的头表内。
fn elf_word(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}

/// 只读取本真实三程序头夹具的元数据，段边界和入口均来自磁盘。
fn layout(context: &TestContext<'_>, file: &FileRef) -> Result<Layout, TestError> {
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

/// 逐字节消费全部 LOAD 文件前缀和零尾，块读取同样覆盖非页齐及跨页边缘。
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

/// 读取本 64 位用户 ABI 的一个机器字，错误保留原 Mm 访问诊断。
fn stack_word(context: &TestContext<'_>, mm: &Mm, address: usize) -> Result<usize, TestError> {
    checked(
        context,
        "initial stack word read failed",
        address as u64,
        mm.read_usize(address),
    )
}

/// 比较参数的完整字节与唯一结尾 NUL，不保留任何用户页借用。
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

/// 按真实消费者栈位置检查必要 auxv、完整初始化字节区及实际硬件能力，保留参数与文件派生地址核对。
fn verify_stack(context: &TestContext<'_>, image: &LoadedImage, layout: &Layout) -> TestResult {
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
    #[cfg(target_arch = "riscv64")]
    let hwcap = (1 << 8) | (1 << 12) | 1 | (1 << 5) | (1 << 3) | (1 << 2);
    #[cfg(target_arch = "loongarch64")]
    let hwcap = {
        // CPUCFG2 是独立硬件事实；按扩展依赖核对用户 ABI，不回读 exec 的能力计算结果。
        let config = crate::arch::cpucfg(2);
        let fp = config & 1 != 0;
        let lsx = fp && config & (1 << 6) != 0;
        let lasx = lsx && config & (1 << 7) != 0;
        1 | (usize::from(fp) << 3) | (usize::from(lsx) << 4) | (usize::from(lasx) << 5)
    };
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

/// 保活数据首尾及中间的真实缓存页，并按位置保存该文件余下字节供私有写后比较；页快照共用一次回收预算。
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

/// 对未发表映像的全部 data/BSS 私有写并读回，再逐字节检查仍保活的文件缓存。
fn verify_private_write(
    context: &TestContext<'_>,
    file: &FileRef,
    image: &LoadedImage,
    data: Segment,
) -> TestResult {
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

/// 从真实盘正式装载，检查私有布局后丢弃修改映像，再运行重新装载的普通用户程序。
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

#[cfg(target_arch = "riscv64")]
core::arch::global_asm!(include_str!(
    "../../../../../tests/user/elf/direct-riscv64.S"
));
#[cfg(target_arch = "loongarch64")]
core::arch::global_asm!(include_str!(
    "../../../../../tests/user/elf/direct-loongarch64.S"
));

extern "C" {
    static user_elf_direct_start: u8;
    static user_elf_direct_end: u8;
}

struct DirectSegment<'a> {
    start: usize,
    flags: u32,
    bytes: &'a [u8],
    memsz: usize,
}

#[derive(Clone, Copy)]
struct ByteCheck {
    address: usize,
    expected: u8,
}

const DIRECT_PATH: &[u8] = b"/plain/direct";
const DIRECT_STATUS: i32 = 37 << 8;
const DIRECT_DATA: usize = 0x20000;
const LARGE_BSS: usize = 64 * 1024 * 1024 + PAGE_SIZE;
const DISTANT_DATA: usize = 0x12000000;

/// 把普通裸程序及其字节检查表写入映像，程序检查旧值、写入0x7e并读回后才退出37。
fn direct_payload(checks: &[ByteCheck]) -> Result<Vec<u8>, TestError> {
    let start = core::ptr::addr_of!(user_elf_direct_start);
    let end = core::ptr::addr_of!(user_elf_direct_end) as usize;
    // SAFETY: 两个汇编链接符号界定同一永久只读片段，片段内字节已初始化，且end在start之后；
    // 这里只读取内核静态字节，随后复制到真实文件，不借用用户映射内容。
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

/// 生成实际ELF64文件；扩展实例的程序头跨页，vendor头与节表字段均不参与LOAD。
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

/// 沿已有内存文件测试的挂载与写入方式准备0644文件，写OFD关闭后才调用正式exec入口。
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

/// 从真实初始栈遍历argv/envp后消费auxv，不使用旧三程序头夹具的布局解析器。
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

/// 发表普通用户Task并消费真实wait及离栈join；用户检查过的每个字节须仍保留写入结果。
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

/// 页零与0x100入口由0644文件实际执行；扩展实例同时消费跨页PHDR和未映射PHDR的auxv。
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

/// 只检查硬件叶而不触发缺页，统计大BSS的实际驻留页。
fn bss_resident(mm: &Mm) -> usize {
    (DIRECT_DATA..DIRECT_DATA + LARGE_BSS)
        .step_by(PAGE_SIZE)
        .filter(|address| mm.mapping(*address).is_some())
        .count()
}

/// 超过旧加载量与跨度的正常映像只访问三个BSS页和一个远端文件页，未触页必须保持不驻留。
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

/// 后段文件和BSS覆盖都按原声明顺序生成独立预期，并逐字节核对跨页结果。
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
