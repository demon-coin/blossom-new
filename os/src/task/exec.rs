//! 磁盘主映像与真实 ELF 解释器在同一私有 Mm 中准备，文件整页同源，边缘页先初始化。

mod arguments;
mod elf;
mod stack;

pub(crate) use self::arguments::Arguments;

use alloc::sync::Arc;
use core::convert::TryInto;

use self::elf::{page_end, Elf, ELF_HEADER_BYTES, INTERP_DYN_BASE, MAIN_DYN_BASE};

use crate::arch::{self, UserContext};
use crate::block::BlockError;
use crate::fs::{
    self, ExecAccess, FileIoContext, FileRef, FsContext, FsError, ReadBuffer, ResolveMode, Vfs,
};
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::{PagePermissions, PageTableError};
use crate::mm::{alloc_pages, PAGE_SIZE};
use crate::task::Credentials;

use crate::fs::PATH_MAX as MAX_PATH_BYTES;
pub(crate) const DEFAULT_STACK_BYTES: u64 = 8 * 1024 * 1024;
const SCRIPT_HEADER_BYTES: usize = 256;
const MAX_INTERPRETER_DEPTH: usize = 5;
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
const RX: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: true,
};
const RWX: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: true,
};

/// 格式、参数预算和实际文件、VM、分配失败保持可区分。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ExecError {
    Format,
    ArgumentsTooBig,
    TooManyInterpreters,
    NoMemory,
    File(FsError),
    Vm(VmError),
}

impl From<FsError> for ExecError {
    /// 文件层的具体错误不转换为格式错误或空数据。
    fn from(error: FsError) -> Self {
        Self::File(error)
    }
}

impl From<VmError> for ExecError {
    /// 保留 VM 所报告的权限、范围、文件及分配原因。
    fn from(error: VmError) -> Self {
        Self::Vm(error)
    }
}

impl From<PageTableError> for ExecError {
    /// 私有根和原页安装失败保持实际页表原因。
    fn from(error: PageTableError) -> Self {
        match error {
            PageTableError::NoMemory => Self::NoMemory,
            other => Self::Vm(other.into()),
        }
    }
}

pub(crate) struct LoadedImage {
    pub mm: Arc<Mm>,
    pub context: UserContext,
}

/// 在同一 root/cwd 和栈限额快照下准备 ELF 或脚本链，成功前不改变正在运行的映像。
pub(crate) fn prepare(
    vfs: &Vfs,
    context: &FsContext,
    mut args: Arguments,
    credentials: Credentials,
) -> Result<LoadedImage, ExecError> {
    let (mut file, mut access) = open_executable(vfs, context, args.filename())?;
    for depth in 0..=MAX_INTERPRETER_DEPTH {
        let mut header = [0; SCRIPT_HEADER_BYTES];
        read_prefix(&file, 0, &mut header[..ELF_HEADER_BYTES])?;
        if header[..2] != *b"#!" {
            return load_elf(
                vfs,
                context,
                file,
                access,
                (&header[..ELF_HEADER_BYTES]).try_into().unwrap(),
                &args,
                credentials,
            );
        }
        read_prefix(
            &file,
            ELF_HEADER_BYTES as u64,
            &mut header[ELF_HEADER_BYTES..],
        )?;
        let (interpreter, optional) = script_interpreter(&header)?;
        args.prepend_interpreter(interpreter, optional, depth == 0)?;
        let next = open_executable(vfs, context, &args.argv[0])?;
        // 新文件已经取得写排斥，接替后才放掉上一个脚本的文件与执行写占用。
        (file, access) = next;
    }
    Err(ExecError::TooManyInterpreters)
}

/// 沿当前 root/cwd 取得真实可映射文件，首读前保留原 inode 的执行写占用。
fn open_executable(
    vfs: &Vfs,
    context: &FsContext,
    pathname: &[u8],
) -> Result<(FileRef, ExecAccess), ExecError> {
    let path = vfs.resolve_from(context, context.cwd(), pathname, ResolveMode::Follow, None)?;
    let file = fs::open_path(vfs, path)?;
    let access = file
        .mapping_inode()
        .ok_or(ExecError::File(FsError::Permission))?
        .deny_writes()?;
    Ok((file, access))
}

/// 只将空格和 tab 当作分隔符，拒绝截断的解释器路径，余下内容作为一个可选参数。
fn script_interpreter(
    header: &[u8; SCRIPT_HEADER_BYTES],
) -> Result<(&[u8], Option<&[u8]>), ExecError> {
    let newline = header
        .iter()
        .take_while(|byte| **byte != 0)
        .position(|byte| *byte == b'\n');
    let mut end = newline.unwrap_or(SCRIPT_HEADER_BYTES - 1);
    if newline.is_none() {
        let name = header[2..=end]
            .iter()
            .position(|byte| !matches!(*byte, b' ' | b'\t'))
            .map(|index| index + 2)
            .ok_or(ExecError::Format)?;
        if !header[name..=end]
            .iter()
            .any(|byte| matches!(*byte, b' ' | b'\t' | 0))
        {
            return Err(ExecError::Format);
        }
    }
    while 2 < end && matches!(header[end - 1], b' ' | b'\t') {
        end -= 1;
    }
    let start = header[2..=end]
        .iter()
        .position(|byte| !matches!(*byte, b' ' | b'\t'))
        .map(|index| index + 2)
        .filter(|start| *start < end)
        .ok_or(ExecError::Format)?;
    let separator = header[start..=end]
        .iter()
        .position(|byte| matches!(*byte, b' ' | b'\t' | 0))
        .map(|index| index + start);
    let name_end = separator.unwrap_or(end);
    let optional = separator
        .filter(|index| header[*index] != 0)
        .and_then(|separator| {
            header[separator..=end]
                .iter()
                .position(|byte| !matches!(*byte, b' ' | b'\t'))
                .map(|index| separator + index)
        })
        .map(|start| {
            let stop = header[start..end]
                .iter()
                .position(|byte| *byte == 0)
                .map_or(end, |index| start + index);
            &header[start..stop]
        });
    Ok((&header[start..name_end], optional))
}

/// 首读前取得主 inode 写排斥，消费参数持有的限额与同一文件系统快照，失败归还候选拥有。
#[cfg_attr(
    not(feature = "ktest"),
    expect(
        dead_code,
        reason = "用户准备、真实 OOM 和写排斥测试直接装载候选映像并核对失败回收"
    )
)]
pub(crate) fn load(
    vfs: &Vfs,
    context: &FsContext,
    file: FileRef,
    args: &Arguments,
    credentials: Credentials,
) -> Result<LoadedImage, ExecError> {
    let access = file
        .mapping_inode()
        .ok_or(ExecError::Format)?
        .deny_writes()?;
    let mut header = [0; ELF_HEADER_BYTES];
    read_exact(&file, 0, &mut header)?;
    load_elf(vfs, context, file, access, &header, args, credentials)
}

/// 所有入口共用两映像与限额快照；全部准备结束前不发表 Mm 或改变原用户现场。
fn load_elf(
    vfs: &Vfs,
    fs_context: &FsContext,
    file: FileRef,
    access: ExecAccess,
    header: &[u8; ELF_HEADER_BYTES],
    args: &Arguments,
    credentials: Credentials,
) -> Result<LoadedImage, ExecError> {
    let mut elf = Elf::read(&file, header, MAIN_DYN_BASE)?;
    let interpreter = if let Some(pathname) = &elf.interp {
        let (file, access) = open_executable(vfs, fs_context, pathname)?;
        let mut header = [0; ELF_HEADER_BYTES];
        read_exact(&file, 0, &mut header)?;
        let image = Elf::read(&file, &header, INTERP_DYN_BASE)?;
        if image.interp.is_some() {
            return Err(ExecError::Format);
        }
        Some((file, access, image))
    } else {
        None
    };
    let mut mm = Mm::try_new()?;
    elf::map_images(
        &mut mm,
        &file,
        &mut elf,
        interpreter.as_ref().map(|(file, _, image)| (file, image)),
    )?;
    let (entry, base) = interpreter
        .as_ref()
        .map_or((elf.entry, 0), |(_, _, image)| (image.entry, image.bias));
    let stack_permissions = if elf.executable_stack
        || interpreter
            .as_ref()
            .is_some_and(|(_, _, image)| image.executable_stack)
    {
        RWX
    } else {
        RW
    };
    map_return_page(&mut mm)?;
    let sp = stack::make_stack(
        &mm,
        &elf,
        interpreter.as_ref().map(|(_, _, image)| image),
        base,
        args,
        credentials.uid,
        credentials.gid,
        stack_permissions,
    )?;
    let heap = page_end(elf.end)?;
    mm.set_heap(heap, heap)?;
    let context = UserContext::new(entry, sp);
    mm.bind_executable(file, access);
    if let Some((file, access, _)) = interpreter {
        mm.bind_interpreter(file, access);
    }
    let mm = Arc::try_new(mm).map_err(|_| ExecError::NoMemory)?;
    Ok(LoadedImage { mm, context })
}

/// 沿真实正前缀读到缓冲尾或 EOF，返回实际字节数，未读取部分保持调用者的初始化值。
fn read_prefix(file: &FileRef, offset: u64, bytes: &mut [u8]) -> Result<usize, ExecError> {
    let mut copied = 0;
    while copied < bytes.len() {
        let read = file.read_at_into(
            offset + copied as u64,
            &mut ReadBuffer::kernel(&mut bytes[copied..]),
            FileIoContext::Kernel,
        )?;
        if read == 0 {
            break;
        }
        assert!(
            read <= bytes.len() - copied,
            "file read exceeded requested bytes"
        );
        copied += read;
    }
    Ok(copied)
}

/// 完整读取沿同一前缀算法，EOF 前不足请求长度报告 ShortIo，真实文件错误直接返回。
fn read_exact(file: &FileRef, offset: u64, bytes: &mut [u8]) -> Result<(), ExecError> {
    if read_prefix(file, offset, bytes)? < bytes.len() {
        return Err(ExecError::File(FsError::Io(BlockError::ShortIo)));
    }
    Ok(())
}

/// 返回页仅承载真实信号 ABI 指令，固定 RX，不宣称为 vDSO。
fn map_return_page(mm: &mut Mm) -> Result<(), ExecError> {
    let code = arch::signal_return_code();
    assert!(
        code.len() <= PAGE_SIZE,
        "signal return code exceeds its page"
    );
    let mut pages = alloc_pages(0, true).ok_or(ExecError::NoMemory)?;
    // SAFETY: pages 独占完整清零且尚未映射的 order0 页，code 永久可读且与候选页不重叠；
    // 写入范围不超过候选页，此原始访问在 map_owned 转交拥有之前结束。
    unsafe {
        core::ptr::copy_nonoverlapping(code.as_ptr(), pages.as_mut_ptr().cast::<u8>(), code.len())
    };
    // SAFETY: 整页仍完整初始化，全部原始访问已结束；mm 未发表，原页唯一拥有消费一次。
    unsafe { mm.map_owned(arch::SIGNAL_RETURN_PAGE, pages, RX, RX) }?;
    Ok(())
}
