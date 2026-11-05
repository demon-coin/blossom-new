//! 明确选择启动盘、根权限与用户系统根路径，复用正式挂载和文件表。

use alloc::sync::Arc;

use crate::block::partition::DeviceSelector;
use crate::block::{AccessMode, BlockView};
use crate::boot::{self, Platform};
#[cfg(feature = "ktest")]
use crate::fs::ext4::crc::crc32c;
use crate::fs::{Ext4Volume, Filesystem, FsContext, FsError, InodeCache, ResolveMode, Vfs};
#[cfg(feature = "ktest")]
use crate::fs::{FileLike, FileTable, OpenOptions};
#[cfg(feature = "ktest")]
use crate::mm::PAGE_SIZE;

const METADATA_CACHE_CAPACITY: usize = 64;
const DENTRY_CAPACITY: usize = 128;
const MOUNT_CAPACITY: usize = 12;

/// 启动参数选中的精确设备和分区；是否存在由已发现的视图表检查。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct BootDevices<'a> {
    pub root: DeviceSelector,
    pub extra: Option<DeviceSelector>,
    pub root_mode: AccessMode,
    pub rootdir: &'a [u8],
    pub init: &'a [u8],
}

/// 按 ASCII 空白消费各参数的最后值，QEMU 默认使用卷内根目录，开发板要求明确根设备和根目录。
/// 设备解析失败和开发板非根目录选择立即返回，不猜分区或切换介质。
pub(crate) fn boot_devices(arguments: &[u8]) -> Result<BootDevices<'_>, FsError> {
    let mut root = None;
    let mut extra = None;
    let mut root_mode = AccessMode::ReadOnly;
    let mut rootdir = None;
    let mut init = None;
    for argument in arguments.split(|byte| byte.is_ascii_whitespace() || *byte == 0x0b) {
        if let Some(value) = argument.strip_prefix(b"root=") {
            root = Some(value);
        } else if let Some(value) = argument.strip_prefix(b"extra=") {
            extra = Some(value);
        } else if let Some(value) = argument.strip_prefix(b"rootdir=") {
            rootdir = Some(value);
        } else if let Some(value) = argument.strip_prefix(b"init=") {
            init = Some(value);
        } else if argument == b"ro" {
            root_mode = AccessMode::ReadOnly;
        } else if argument == b"rw" {
            root_mode = AccessMode::ReadWrite;
        }
    }

    let board = boot::info().platform != Platform::QemuVirt;
    let root = match root {
        Some(root) => root,
        None if !board => b"/dev/vdb",
        None => return Err(FsError::MissingRoot),
    };
    let rootdir = match rootdir {
        Some(rootdir) if !board || rootdir == b"/" => rootdir,
        None if !board => b"/",
        _ => return Err(FsError::MissingRoot),
    };
    Ok(BootDevices {
        root: DeviceSelector::parse(root).map_err(|_| FsError::MissingRoot)?,
        extra: extra
            .map(DeviceSelector::parse)
            .transpose()
            .map_err(|_| FsError::MissingRoot)?,
        root_mode,
        rootdir,
        init: init.unwrap_or(b"/sbin/init"),
    })
}

/// 普通可睡眠 Task 挂一次根卷，按真实目录建立 root=cwd 的初始视图，再挂显式只读附盘。
/// 卷与路径只保活原设备及缓存；目录或附盘准备失败原样返回，不改选盘或物理根。
pub(crate) fn mount_disks(
    root: BlockView,
    extra: Option<BlockView>,
    inodes: Arc<InodeCache>,
    root_mode: AccessMode,
    rootdir: &[u8],
) -> Result<(Vfs, FsContext), FsError> {
    let volume = Ext4Volume::open(root, root_mode, METADATA_CACHE_CAPACITY)?;
    let vfs = Vfs::try_new(
        Filesystem::Ext4(volume),
        inodes,
        DENTRY_CAPACITY,
        MOUNT_CAPACITY,
        root_mode,
    )?;

    let physical_root = vfs.root();
    let physical_context = FsContext::new(physical_root.clone(), physical_root)?;
    let root = vfs.resolve(&physical_context, rootdir, ResolveMode::Follow)?;
    let context = FsContext::new(root.clone(), root)?;
    drop(physical_context);

    if let Some(extra) = extra {
        let covered = vfs.resolve(&context, b"/mnt/extra", ResolveMode::Follow)?;
        let volume = Ext4Volume::open(extra, AccessMode::ReadOnly, METADATA_CACHE_CAPACITY)?;
        vfs.mount(&covered, Filesystem::Ext4(volume), AccessMode::ReadOnly)?;
    }
    Ok((vfs, context))
}

/// 实际交付的完整文件长度与标准最终取反 CRC32C，空文件摘要为零。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct FileSummary {
    pub bytes: u64,
    pub crc32c: u32,
}

/// 在普通可睡眠 Task 中只读打开并完整读取文件，所有返回分支都关闭本次描述符。
/// 调用者通过自有表独占本次新开编号直至关闭，期间不得由其他任务关闭或替换它。
/// 本次取得的文件引用在表锁外使用和释放；读取错误优先于关闭错误。
#[cfg(feature = "ktest")]
pub(crate) fn read_summary(
    files: &FileTable,
    vfs: &Vfs,
    context: &FsContext,
    pathname: &[u8],
) -> Result<FileSummary, FsError> {
    let fd = files.open(
        vfs,
        context,
        pathname,
        OpenOptions::default(),
        crate::task::process::DEFAULT_NOFILE.current as usize,
    )?;

    let result = files
        .lookup(fd)
        .and_then(|file| summarize_file(file.as_ref()));
    let closed = files.close(fd);

    let summary = result?;
    closed?;
    Ok(summary)
}

/// 用一页任务栈缓冲顺序读取同一打开实例，读到 EOF 后核对真实 inode 大小。
/// 正前缀仍继续读取，后次错误原样返回，不把部分交付冒充完整文件。
#[cfg(feature = "ktest")]
fn summarize_file(file: &dyn FileLike) -> Result<FileSummary, FsError> {
    let size = file.stat()?.size;
    let mut buffer = [0; PAGE_SIZE];
    let mut bytes = 0_u64;
    let mut crc = !0_u32;
    loop {
        let count = file.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        bytes = bytes.checked_add(count as u64).ok_or(FsError::Corrupt)?;
        crc = crc32c(crc, &buffer[..count]);
    }

    if bytes != size {
        return Err(FsError::Corrupt);
    }
    Ok(FileSummary {
        bytes,
        crc32c: !crc,
    })
}
