//! 系统启动块设备选择、根文件系统挂载与初始进程命名空间组装。
//!
//! # 模块职责与架构定位
//! 本模块衔接内核命令行参数解析、块设备发现层与虚拟文件系统（VFS）：
//! - **上游调用者**：内核主启动流程（构造初始用户进程的 `Vfs` 与 `FsContext`）以及内核集成测试套件（`ktest`）。
//! - **下游依赖**：块设备分区选择器（`block::partition::DeviceSelector`）、`Ext4Volume` 磁盘卷、以及 VFS 挂载树（`Vfs` / `FsContext`）。
//!
//! # 核心设计与挂载拓扑组装流程
//! ```text
//!   命令行字节流 (bootargs)
//!     |
//!     | boot_devices(): 解析 root= / extra= / rootdir= / init= / ro / rw
//!     v
//!   BootDevices
//!     |
//!     | mount_disks(root, extra, inodes, root_mode, rootdir)
//!     v
//!   1. Ext4Volume::open(root) -> Vfs::try_new() 建立物理根 "/"
//!   2. 在物理根下解析 `rootdir` -> 建立初始进程 FsContext (root = cwd = rootdir)
//!   3. 若指定了 `extra` 块设备 -> 解析 "/mnt/extra" 并以 ReadOnly 模式挂载附属 Ext4Volume
//! ```
//!
//! # 锁与并发契约
//! - `boot_devices` 为无锁纯解析函数。
//! - `mount_disks` 与 `read_summary` 必须在普通可睡眠任务上下文中调用（底层涉及块设备同步读取、ext4 超级块加载与目录树遍历锁）。

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

/// 单个挂载 ext4 卷的元数据块缓存最大容量（单位：块）。
const METADATA_CACHE_CAPACITY: usize = 64;
/// 系统 VFS 目录项缓存（`DentryCache`）的最大槽位容量（单位：个）。
const DENTRY_CAPACITY: usize = 128;
/// 系统 VFS 挂载表允许的最大挂载点数量（单位：个）。
const MOUNT_CAPACITY: usize = 12;

/// 从内核启动命令行（bootargs）中解析出的根文件系统与初始进程配置。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct BootDevices<'a> {
    /// 根文件系统所在的块设备或分区选择器（由 `root=` 指定，QEMU 平台默认为 `/dev/vdb`）。
    pub root: DeviceSelector,
    /// 可选的附属只读数据盘或分区选择器（由 `extra=` 指定，挂载于 `/mnt/extra`）。
    pub extra: Option<DeviceSelector>,
    /// 根文件系统的挂载访问模式（`ro` 对应 `AccessMode::ReadOnly`，`rw` 对应 `AccessMode::ReadWrite`，默认只读）。
    pub root_mode: AccessMode,
    /// 初始进程在物理根卷内绑定的根目录路径字节切片（由 `rootdir=` 指定，真实开发板要求必须为 `"/"`）。
    pub rootdir: &'a [u8],
    /// 初始用户态进程（PID 1）的可执行文件路径字节切片（由 `init=` 指定，默认为 `"/sbin/init"`）。
    pub init: &'a [u8],
}

/// 解析内核启动命令行字节流 `arguments`，提取根块设备、附属块设备、读写模式、系统根目录与 `init` 路径。
///
/// # 解析规则与平台约束
/// - 按 ASCII 空白字符或垂直制表符（`0x0b`）切分参数；若同一键出现多次，以最后一次出现的值为准。
/// - 在 `QemuVirt` 平台上，`root=` 缺省回退为 `"/dev/vdb"`，`rootdir=` 缺省回退为 `"/"`。
/// - 在真实开发板平台上，必须显式传入合法的 `root=` 且 `rootdir=` 必须为 `"/"`，否则返回 `FsError::MissingRoot`。
pub(crate) fn boot_devices(arguments: &[u8]) -> Result<BootDevices<'_>, FsError> {
    let mut root = None;
    let mut extra = None;
    let mut root_mode = AccessMode::ReadOnly;
    let mut rootdir = None;
    let mut init = None;
    // Step 1: 顺序扫描命令行 token，记录各配置项最后一次出现的值。
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

    // Step 2: 根据平台类型（QEMU vs 真实开发板）应用默认值或强制校验必选的 `root=` 与 `rootdir=`。
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
    // Step 3: 解析块设备选择器并组装返回。
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

/// 打开根块设备上的 ext4 文件系统，构建 VFS 实例与初始进程的 `FsContext`，并按需挂载附属只读盘。
///
/// # 调用上下文
/// 必须在普通可睡眠任务上下文中调用（执行 ext4 超级块读取与路径解析 I/O）。
pub(crate) fn mount_disks(
    root: BlockView,
    extra: Option<BlockView>,
    inodes: Arc<InodeCache>,
    root_mode: AccessMode,
    rootdir: &[u8],
) -> Result<(Vfs, FsContext), FsError> {
    // Step 1: 在根块设备上打开 ext4 卷，并初始化固定容量的 Vfs 实例。
    let volume = Ext4Volume::open(root, root_mode, METADATA_CACHE_CAPACITY)?;
    let vfs = Vfs::try_new(
        Filesystem::Ext4(volume),
        inodes,
        DENTRY_CAPACITY,
        MOUNT_CAPACITY,
        root_mode,
    )?;

    // Step 2: 从物理挂载根出发解析 `rootdir`，将解析得到的目录同时设为初始进程上下文的根目录（root）与当前工作目录（cwd）。
    let physical_root = vfs.root();
    let physical_context = FsContext::new(physical_root.clone(), physical_root)?;
    let root = vfs.resolve(&physical_context, rootdir, ResolveMode::Follow)?;
    let context = FsContext::new(root.clone(), root)?;
    drop(physical_context);

    // Step 3: 若配置了附属块设备 `extra`，在新建的进程根视图下解析 `/mnt/extra` 并将附属 ext4 卷以只读方式挂载上去。
    if let Some(extra) = extra {
        let covered = vfs.resolve(&context, b"/mnt/extra", ResolveMode::Follow)?;
        let volume = Ext4Volume::open(extra, AccessMode::ReadOnly, METADATA_CACHE_CAPACITY)?;
        vfs.mount(&covered, Filesystem::Ext4(volume), AccessMode::ReadOnly)?;
    }
    Ok((vfs, context))
}

/// 内核测试读取完整文件后生成的长度与 CRC32C 校验摘要。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct FileSummary {
    /// 实际顺序读取到文件末尾（EOF）的累计字节数（单位：字节）。
    pub bytes: u64,
    /// 文件全部内容经标准 CRC32C（Castagnoli，初值全 1、结果按位取反）计算的 32 位校验值；空文件为 `0`。
    pub crc32c: u32,
}

/// 在内核测试中通过进程文件描述符表 `files` 只读打开 `pathname`，顺序读取完整内容计算 `FileSummary`，并在返回前关闭该文件描述符。
///
/// # 调用上下文与契约
/// - 必须在普通可睡眠任务上下文中调用。
/// - 调用方必须保证在本次调用返回前，没有其他并发任务关闭或篡改 `files` 中新分配的文件描述符槽位。
/// - 无论读取成功或失败，函数都会尝试关闭新打开的描述符；若读取与关闭同时失败，优先返回读取阶段的错误。
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

/// 使用单页大小（`PAGE_SIZE`）的栈上缓冲区循环读取打开文件 `file` 直至 EOF，计算累计字节数与 CRC32C，并与 `stat()` 报告的 Inode 大小比对。
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
