//! `execve` 可执行映像装载器：解析 Shebang（`#!`）脚本链与 ELF64 可执行文件，在全新的私有 [`Mm`] 地址空间中构建主程序、动态链接器、信号返回页与初始用户栈。
//!
//! ## 模块职责与架构定位
//! 本模块位于用户任务执行切换层（`crate::task::user`）与底层虚拟内存（`crate::mm`）、虚拟文件系统（`crate::fs`）之间：
//! - **上游调用者**：内核初始进程启动路径以及 `sys_execve` 系统调用路径，传入已快照化的 [`Arguments`]、当前任务的文件系统上下文 [`FsContext`] 与凭据 [`Credentials`]。
//! - **下游依赖**：调用 VFS 解析路径并获取可执行文件的写排斥凭据（[`ExecAccess`]），调用子模块 [`elf`] 解析并映射主映像与动态链接器（`PT_INTERP`），调用子模块 [`stack`] 构造符合 System V ABI 的初始用户栈与辅助向量（`auxv`），最终产出尚未发布的 [`LoadedImage`]。
//!
//! ## 核心机制：Shebang 脚本展开与虚拟地址空间布局
//!
//! ### 1. Shebang（`#!`）解释器递归展开流程
//! ```text
//!   prepare(filename, argv, envp)
//!     │
//!     ├── open_executable(pathname) ──► 解析路径 + deny_writes() 锁定 inode 禁止并发写入
//!     │
//!     └── 循环检查文件头（最多 MAX_INTERPRETER_DEPTH = 5 层）：
//!           ├── 非 "#!" 开头 ──► 进入 load_elf() 装载 ELF 主映像与可选 PT_INTERP 动态链接器
//!           └── 以 "#!" 开头 ──► 解析解释器路径与单个可选参数
//!                                ├── args.prepend_interpreter(...) 改写 argv 前缀
//!                                └── 打开下一层解释器并获取写排斥后，释放上一层脚本文件
//! ```
//!
//! ### 2. 新建私有 [`Mm`] 用户虚拟地址空间布局
//! ```text
//!   高地址  arch::USER_ADDRESS_END
//!           +------------------------------------------+
//!           | 信号返回跳板页 (SIGNAL_RETURN_PAGE, RX)    | ──► 存放 sigreturn 系统调用指令序列
//!           +------------------------------------------+
//!           | 初始用户栈 (USER_STACK_TOP 向下生长)       | ──► envp/argv 字符串、AT_RANDOM、auxv、envp[]、argv[]、argc
//!           |   ... 预留启动栈空间 (headroom) ...       |
//!           +------------------------------------------+
//!           | 动态链接器 (PT_INTERP, 基址 0x3000_0000)  | ──► 若存在 ET_DYN 解释器（如 ld-musl / ld-linux）
//!           +------------------------------------------+
//!           | 用户堆 (brk, 紧跟主映像末尾页对齐上界)     | ──► 初始 start == end == page_end(elf.end)
//!           | 主程序映像 (ET_EXEC 或 PIE @ 0x2000_0000) | ──► PT_LOAD 代码段、数据段与 BSS
//!   低地址  +------------------------------------------+
//! ```
//!
//! ## 锁获取顺序与并发契约
//! 1. **可执行文件写排斥（`ETXTBSY` 保护）**：在首次读取文件头之前，必须先通过 `inode.deny_writes()` 获取 [`ExecAccess`]，阻止其他任务同时以写模式打开或截断正在装载的可执行文件。在脚本链迭代时，先获取下一层解释器的写排斥，再释放上一层脚本的 [`ExecAccess`]。
//! 2. **离线准备与无副作用失败回滚**：整个 [`prepare`] 与 [`load_elf`] 过程都在新分配的局部私有 [`Mm`] 上操作，不持有当前任务的进程图锁或线程状态锁。若中途发生格式错误、I/O 错误或内存不足（OOM），局部 [`Mm`] 与 [`ExecAccess`] 随栈帧析构自动释放物理页并恢复 inode 可写计数，调用方原有的地址空间与执行现场保持完好。

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

/// 默认用户主栈软限制大小（8 MiB，单位：字节）。
pub(crate) const DEFAULT_STACK_BYTES: u64 = 8 * 1024 * 1024;
/// Shebang（`#!`）脚本首行最大解析长度（256 字节）。
const SCRIPT_HEADER_BYTES: usize = 256;
/// Shebang（`#!`）脚本链式嵌套解释器的最大递归层数。
const MAX_INTERPRETER_DEPTH: usize = 5;

/// 可读写、不可执行的页权限常量（`rw-`）。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};
/// 可读、可执行、不可写的页权限常量（`r-x`）。
const RX: PagePermissions = PagePermissions {
    read: true,
    write: false,
    execute: true,
};
/// 可读、可写、可执行的页权限常量（`rwx`）。
const RWX: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: true,
};

/// 可执行映像解析与装载阶段可能返回的错误类型。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ExecError {
    /// 可执行文件格式非法（对应 `ENOEXEC`，如非法的 ELF 魔数、损坏的程序头或截断的 `#!` 行）。
    Format,
    /// 参数、环境变量或路径总字节数超出栈预算限制（对应 `E2BIG`）。
    ArgumentsTooBig,
    /// `#!` 脚本解释器嵌套层数超过 [`MAX_INTERPRETER_DEPTH`]（对应 `ELOOP`）。
    TooManyInterpreters,
    /// 内核堆或物理页分配失败（对应 `ENOMEM`）。
    NoMemory,
    /// 底层文件系统或块设备 I/O 错误（如文件不存在、无权限、正在被写入或读取截断）。
    File(FsError),
    /// 虚拟内存映射或用户空间访问错误。
    Vm(VmError),
}

impl From<FsError> for ExecError {
    /// 将文件系统错误直接包装为 [`ExecError::File`]。
    fn from(error: FsError) -> Self {
        Self::File(error)
    }
}

impl From<VmError> for ExecError {
    /// 将虚拟内存操作错误包装为 [`ExecError::Vm`]。
    fn from(error: VmError) -> Self {
        Self::Vm(error)
    }
}

impl From<PageTableError> for ExecError {
    /// 将页表操作错误转换为 [`ExecError`]（内存不足转为 [`ExecError::NoMemory`]，其余转为 [`ExecError::Vm`]）。
    fn from(error: PageTableError) -> Self {
        match error {
            PageTableError::NoMemory => Self::NoMemory,
            other => Self::Vm(other.into()),
        }
    }
}

/// 装载就绪但尚未绑定到具体任务的可执行映像。
pub(crate) struct LoadedImage {
    /// 装载了主程序、可选动态链接器、信号返回页与初始用户栈的独立地址空间。
    pub mm: Arc<Mm>,
    /// 设置好程序入口地址（`sepc`/`era`）与初始用户栈指针（`sp`）的用户态陷入上下文。
    pub context: UserContext,
}

/// 解析可执行路径（支持 `#!` 脚本链式展开）并装载最终的 ELF 映像。
///
/// 在可睡眠的任务上下文中调用。所有准备工作均在新建的候选地址空间中进行；
/// 若返回错误，已分配的内存和文件写排斥守卫会自动释放，不会影响调用任务当前正在运行的地址空间。
pub(crate) fn prepare(
    vfs: &Vfs,
    context: &FsContext,
    mut args: Arguments,
    credentials: Credentials,
) -> Result<LoadedImage, ExecError> {
    let (mut file, mut access) = open_executable(vfs, context, args.filename())?;
    for depth in 0..=MAX_INTERPRETER_DEPTH {
        let mut header = [0; SCRIPT_HEADER_BYTES];
        // Step 1: 先读取前 64 字节（ELF 头大小）；若不是 "#!" 脚本，直接按 ELF 映像装载。
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

        // Step 2: 若是 "#!" 脚本，补齐剩余脚本头字节，解析解释器路径与可选参数并改写 argv。
        read_prefix(
            &file,
            ELF_HEADER_BYTES as u64,
            &mut header[ELF_HEADER_BYTES..],
        )?;
        let (interpreter, optional) = script_interpreter(&header)?;
        args.prepend_interpreter(interpreter, optional, depth == 0)?;
        let next = open_executable(vfs, context, &args.argv[0])?;
        // 新解释器文件已成功打开并获取写排斥守卫后，再替换并释放上一层脚本的文件句柄与写排斥。
        (file, access) = next;
    }
    Err(ExecError::TooManyInterpreters)
}

/// 在当前文件系统上下文（`root`/`cwd`）下解析路径、打开可执行文件，并在首次读取前锁定其底层 Inode 禁止写入。
///
/// 在可睡眠的任务上下文中调用。若目标不是支持内存映射的常规文件，返回权限错误。
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

/// 解析 `#!` 脚本首行，提取解释器路径与最多一个可选参数。
///
/// 仅将空格（`' '`）和水平制表符（`'\t'`）视为分隔符：
/// - 若缓冲区内未出现换行符且解释器路径紧贴末尾无分隔符（路径被截断），返回 [`ExecError::Format`]；
/// - 解释器路径之后去除首尾空白的剩余整段内容（不含 NUL 字节）合并作为单个可选参数返回。
fn script_interpreter(
    header: &[u8; SCRIPT_HEADER_BYTES],
) -> Result<(&[u8], Option<&[u8]>), ExecError> {
    // Step 1: 在首个 NUL 字节前查找换行符位置；若无换行符，检查解释器路径是否因超出缓冲区而被截断。
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

    // Step 2: 剥离行尾空白字符，定位 "#!" 之后的解释器路径起止偏移。
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

    // Step 3: 若解释器路径后仍有非空白字符，将其余部分（截断至首个 NUL）作为单个可选参数。
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

/// 从已打开的文件对象直接装载 ELF 可执行映像（供内核启动及 `ktest` 单元测试使用）。
///
/// 在首次读取前获取底层 Inode 的写排斥守卫 [`ExecAccess`]；若装载失败，所有临时分配的地址空间与写排斥守卫自动释放。
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

/// 在全新私有 [`Mm`] 中装载 ELF 主程序与可选的 `PT_INTERP` 动态链接器，并构建初始用户栈。
///
/// 在可睡眠的任务上下文中调用。完成全部段映射、信号返回跳板页映射、初始用户栈构造及堆起点初始化后，
/// 将主程序与解释器的文件句柄及 [`ExecAccess`] 绑定到 [`Mm`] 生命周期内保活。
fn load_elf(
    vfs: &Vfs,
    fs_context: &FsContext,
    file: FileRef,
    access: ExecAccess,
    header: &[u8; ELF_HEADER_BYTES],
    args: &Arguments,
    credentials: Credentials,
) -> Result<LoadedImage, ExecError> {
    // Step 1: 解析主 ELF 映像程序头；若包含 PT_INTERP 段，打开并解析动态链接器映像（禁止解释器再次嵌套 PT_INTERP）。
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

    // Step 2: 创建候选私有地址空间，统一规划并映射主程序与动态链接器的所有 PT_LOAD 段，计算实际入口地址。
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

    // Step 3: 根据主映像与解释器的 PT_GNU_STACK 标志决定用户栈权限，映射信号返回页并构造初始用户栈与初始堆边界。
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

    // Step 4: 初始化用户态入口上下文，将可执行文件与写排斥守卫绑定至 Mm 并封装返回。
    let context = UserContext::new(entry, sp);
    mm.bind_executable(file, access);
    if let Some((file, access, _)) = interpreter {
        mm.bind_interpreter(file, access);
    }

    let mm = Arc::try_new(mm).map_err(|_| ExecError::NoMemory)?;
    Ok(LoadedImage { mm, context })
}

/// 从文件指定偏移处循环读取数据，直到填满 `bytes` 缓冲区或到达文件末尾（EOF）。
///
/// 返回实际读取的字节数；若文件提前到达 EOF，缓冲区尾部未被覆盖的字节保持调用方传入时的初始值。
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

/// 从文件指定偏移处精确填满整个 `bytes` 缓冲区；若在填满前遇到 EOF，则返回 `ShortIo` 错误。
fn read_exact(file: &FileRef, offset: u64, bytes: &mut [u8]) -> Result<(), ExecError> {
    if read_prefix(file, offset, bytes)? < bytes.len() {
        return Err(ExecError::File(FsError::Io(BlockError::ShortIo)));
    }
    Ok(())
}

/// 在候选地址空间的固定虚拟地址 [`arch::SIGNAL_RETURN_PAGE`] 处分配并映射只读可执行的信号返回跳板页。
fn map_return_page(mm: &mut Mm) -> Result<(), ExecError> {
    let code = arch::signal_return_code();
    assert!(
        code.len() <= PAGE_SIZE,
        "signal return code exceeds its page"
    );
    let mut pages = alloc_pages(0, true).ok_or(ExecError::NoMemory)?;
    // SAFETY:
    // 1. `pages` 是刚刚通过 `alloc_pages(0, true)` 分配并清零的单个物理页（4096 字节），当前由本函数唯一持有且尚未映射；
    // 2. `code` 为内核静态只读切片，前方的 `assert!(code.len() <= PAGE_SIZE)` 保证了拷贝长度不越界且源与目标内存绝不重叠；
    // 3. 裸指针拷贝在此代码块内立即完成，随后才将物理页所有权移交给 `mm.map_owned`。
    unsafe {
        core::ptr::copy_nonoverlapping(code.as_ptr(), pages.as_mut_ptr().cast::<u8>(), code.len())
    };
    // SAFETY:
    // 1. `pages` 整页已在分配时清零且前段写入了合法指令字节，不存在未初始化内存；
    // 2. 上方对 `pages` 的裸指针写入已全部结束，且 `mm` 是尚未发布给任何线程的局部私有地址空间；
    // 3. `pages` 所有权在此处唯一一次移交给 `mm` 管理。
    unsafe { mm.map_owned(arch::SIGNAL_RETURN_PAGE, pages, RX, RX) }?;
    Ok(())
}
