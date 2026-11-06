//! 系统标识查询、文件系统挂载与卸载、全局脏页同步以及系统关机系统调用实现。
//!
//! ## 模块职责与架构定位
//! - **上游调用者**：`crate::syscall` 主分派器，处理 `uname`、`sysinfo`、`statfs`、`fstatfs`、
//!   `mount`、`umount2`、`sync` 与 `reboot` 系统调用。
//! - **下游依赖**：
//!   - `crate::mm`：提供物理页帧统计（`mm::stats`）与用户态内存安全拷贝（`copyout` / `read_c_string`）。
//!   - `crate::fs` 与 `crate::storage`：提供虚拟文件系统路径解析、挂载表管理以及底层块设备视图查找。
//!   - `crate::task::process`：提供进程关系图（`ProcessGraph`）与系统级关机请求（`request_shutdown`）。
//!
//! ## 关键 ABI 内存布局
//!
//! ### 1. `utsname` 布局（共 390 字节，6 个固定长度字段）
//! ```text
//! +--------+----------+---------+---------+---------+------------+
//! | 0..65  | 65..130  | 130..195| 195..260| 260..325| 325..390   |
//! |sysname | nodename | release | version | machine | domainname |
//! +--------+----------+---------+---------+---------+------------+
//! ```
//! 每个字段占 65 字节，以 NUL (`\0`) 结尾，剩余字节全部填零。
//!
//! ### 2. LP64 `sysinfo` 布局（共 112 字节）
//! ```text
//! 偏移(B)  大小(B)  字段含义
//! 0..8     8        uptime (系统启动以来的秒数，向上取整)
//! 8..32    24       loads[3] (1/5/15 分钟平均负载，当前填 0)
//! 32..40   8        totalram (可用总物理页数)
//! 40..48   8        freeram (伙伴系统空闲页 + 每 CPU 缓存页数)
//! 48..80   32       sharedram / bufferram / totalswap / freeswap (填 0)
//! 80..82   2        procs (当前存活任务数 u16)
//! 82..88   6        显式对齐填充 (填 0)
//! 88..104  16       totalhigh / freehigh (高端内存统计，填 0)
//! 104..108 4        mem_unit (内存容量单位：PAGE_SIZE 字节)
//! 108..112 4        尾部填充 (填 0)
//! ```
//!
//! ### 3. LP64 `statfs` 布局（共 120 字节）
//! ```text
//! 偏移(B)  大小(B)  字段含义
//! 0..8     8        f_type (文件系统魔数)
//! 8..16    8        f_bsize (最优传输块大小)
//! 16..24   8        f_blocks (文件系统总数据块数)
//! 24..32   8        f_bfree (空闲块总数)
//! 32..40   8        f_bavail (非特权用户可用空闲块数)
//! 40..48   8        f_files (总文件节点数)
//! 48..56   8        f_ffree (空闲文件节点数)
//! 56..64   8        f_fsid (文件系统标识，8 字节)
//! 64..72   8        f_namelen (文件名最大长度 NAME_MAX)
//! 72..80   8        f_frsize (基础片段大小，同 block_size)
//! 80..88   8        f_flags (挂载标志位：ST_VALID | ST_RDONLY)
//! 88..120  32       f_spare[4] (保留字段，填 0)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 访问进程的 `runtime.fs` 自旋锁仅用于短暂克隆 `Vfs` 和路径上下文（`FsContext`）的强引用，
//!   随后立即释放锁；绝不在持有 `runtime.fs` 锁期间执行路径解析、块设备 I/O 或用户空间内存拷贝。
//! - `copyout` 和 `read_c_string` 可能触发用户页缺页异常并睡眠，调用时不得持有任何自旋锁。

use crate::block::partition::DeviceSelector;
use crate::block::AccessMode;
use crate::fs::memfs::{FilesystemStat, MemoryFs};
use crate::fs::{devfs, procfs, Ext4Volume, Filesystem, Path, ResolveMode, NAME_MAX, PATH_MAX};
use crate::storage::Storage;
use crate::syscall::error::{copyout, Errno, SysResult};
use crate::syscall::Outcome;
use crate::task::process::{RootRole, ShutdownOperation};
use crate::task::UserThread;
use crate::{mm, sched, time};

const MOUNT_STRING_LIMIT: usize = 4096;
const MOUNT_METADATA_CAPACITY: usize = 64;

const MS_RDONLY: usize = 1;
const MS_REMOUNT: usize = 32;
const MS_SILENT: usize = 0x8000;
const UMOUNT_NOFOLLOW: usize = 8;

const STATFS_SIZE: usize = 120;
const ST_RDONLY: u64 = 1;
const ST_VALID: u64 = 0x20;

const REBOOT_MAGIC1: u32 = 0xfee1_dead;
const REBOOT_MAGIC2: u32 = 672_274_793;
const REBOOT_MAGIC2A: u32 = 85_072_278;
const REBOOT_MAGIC2B: u32 = 369_367_448;
const REBOOT_MAGIC2C: u32 = 537_993_216;

const REBOOT_CMD_CAD_OFF: u32 = 0;
const REBOOT_CMD_RESTART: u32 = 0x0123_4567;
const REBOOT_CMD_HALT: u32 = 0xcdef_0123;
const REBOOT_CMD_POWER_OFF: u32 = 0x4321_fedc;

const UTS_FIELD_LEN: usize = 65;
#[cfg(target_arch = "riscv64")]
const UTS_MACHINE: &[u8] = b"riscv64";
#[cfg(target_arch = "loongarch64")]
const UTS_MACHINE: &[u8] = b"loongarch64";
static UTS_NAME: [u8; UTS_FIELD_LEN * 6] = uname_bytes();

/// `uname` 系统调用：向用户空间写入固定的内核与主机标识信息（`struct utsname`）。
///
/// - **调用上下文**：普通可睡眠用户任务上下文，未持有自旋锁。
/// - **参数**：`raw[0]` 为用户空间 `struct utsname` 缓冲区的起始虚拟地址。
/// - **返回值**：成功返回 `Ok(0)`；若用户缓冲区地址非法且无法通过缺页修复，保留已写入的前缀字节并返回 `Err(Errno::BadAddress)`（即 `EFAULT`）。
pub(super) fn uname(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let mm = user.mm().expect("uname has no MM");
    copyout(&mm, raw[0], &UTS_NAME)?;
    Ok(0)
}

/// `sysinfo` 系统调用：采样当前物理内存统计、系统运行秒数与存活任务数量，并按 LP64 ABI 格式写入用户空间。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。内存页统计、时钟与任务数均先在栈上通过原子操作或极短的自旋锁临界区完成快照组装，向用户空间复制字节期间不持有调度器锁或内存管理锁。
/// - **参数**：`raw[0]` 为用户空间 `struct sysinfo` 缓冲区的起始虚拟地址（112 字节）。
/// - **返回值**：成功返回 `Ok(0)`；用户地址无效时返回 `Err(Errno::BadAddress)`。
pub(super) fn sysinfo(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let memory = mm::stats();
    let elapsed = time::now().as_nanos();
    let uptime = elapsed.div_ceil(time::NSEC_PER_SEC);
    // Step 1: 收集前 10 个 8 字节长整型字段（偏移 0..80），容量以物理页为单位，mem_unit 设为 PAGE_SIZE。
    let fields = [
        uptime,
        0,
        0,
        0,
        (memory.present_pages - memory.reserved_pages - memory.metadata_pages) as u64,
        (memory.buddy_free_pages + memory.pcp_cached_pages) as u64,
        0,
        0,
        0,
        0,
    ];

    // Step 2: 将所有字段编码为小端字节数组，整个 112 字节数组预先清零以避免泄露栈上未初始化的填充字节。
    let mut bytes = [0u8; 112];
    for (index, value) in fields.iter().enumerate() {
        bytes[index * 8..index * 8 + 8].copy_from_slice(&value.to_le_bytes());
    }
    // 按 LP64 struct sysinfo 布局：procs (u16) 位于偏移 80，totalhigh/freehigh 位于偏移 88/96（保持为 0），
    // mem_unit (u32) 位于偏移 104。当前内核未启用交换分区（swap）与高端内存（highmem）。
    bytes[80..82].copy_from_slice(&(sched::live_task_count() as u16).to_le_bytes());
    bytes[104..108].copy_from_slice(&(mm::PAGE_SIZE as u32).to_le_bytes());

    // Step 3: 将组装完毕的 112 字节一次性拷贝到用户空间。
    let mm = user.mm().expect("sysinfo has no MM");
    copyout(&mm, raw[0], &bytes)?;
    Ok(0)
}

/// 在编译期构造 `struct utsname` 的 6 个固定字段（每个字段 65 字节，以 NUL 结尾且剩余部分填零），不依赖宿主或 Rust 结构体的内存对齐布局。
const fn uname_bytes() -> [u8; UTS_FIELD_LEN * 6] {
    let fields: [&[u8]; 6] = [
        b"Linux",
        b"blossom",
        b"6.12.0-blossom",
        env!("CARGO_PKG_VERSION").as_bytes(),
        UTS_MACHINE,
        b"(none)",
    ];
    let mut bytes = [0; UTS_FIELD_LEN * 6];
    let mut field = 0;
    while field < fields.len() {
        assert!(fields[field].len() < UTS_FIELD_LEN);
        let mut index = 0;
        while index < fields[field].len() {
            bytes[field * UTS_FIELD_LEN + index] = fields[field][index];
            index += 1;
        }
        field += 1;
    }
    bytes
}

/// `statfs` 系统调用：根据用户传入的路径名解析目标挂载点（跟随末尾符号链接），并将所属文件系统的容量统计写入用户空间。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。
/// - **参数**：`raw[0]` 为以 NUL 结尾的路径字符串用户地址，`raw[1]` 为输出 `struct statfs` 的用户缓冲区地址。
/// - **返回值**：成功返回 `Ok(0)`；路径解析或内存拷贝失败时返回对应 `Errno`。
pub(super) fn statfs(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let mm = user.mm().expect("statfs has no MM");
    let pathname = mm.read_c_string(raw[0], PATH_MAX)?;
    let runtime = user.runtime().expect("statfs has no runtime");
    let (vfs, context) = {
        let fs = runtime.fs.lock();
        (fs.vfs.clone(), fs.context.clone())
    };
    let path = vfs.resolve_from(
        &context,
        context.cwd(),
        &pathname,
        ResolveMode::Follow,
        Some((runtime.credentials.uid, runtime.credentials.gid)),
    )?;
    write_statfs(user, &path, raw[1])
}

/// `fstatfs` 系统调用：根据已打开的文件描述符查询其所属文件系统的容量统计，并写入用户空间。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。
/// - **参数**：`raw[0]` 为文件描述符（按 32 位有符号整数校验），`raw[1]` 为输出 `struct statfs` 的用户缓冲区地址。
/// - **返回值**：成功返回 `Ok(0)`；描述符无效返回 `Err(Errno::BadFd)`；若文件对象不关联具体的文件系统路径（如匿名管道或套接字），返回 `Err(Errno::NoSyscall)`。
pub(super) fn fstatfs(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let fd = raw[0] as i32;
    if fd < 0 {
        return Err(Errno::BadFd);
    }

    let runtime = user.runtime().expect("fstatfs has no runtime");
    let file = runtime.files.lookup(fd as usize)?;
    let path = file.path().ok_or(Errno::NoSyscall)?;
    write_statfs(user, path, raw[1])
}

/// 查询 `path` 所在挂载实例的文件系统统计信息，编码为 120 字节的 `statfs` 结构后复制到用户空间 `address`。
fn write_statfs(user: &UserThread, path: &Path, address: usize) -> SysResult {
    let stat = path.filesystem().statfs()?;
    let bytes = encode_statfs(stat, path.mount_mode());
    let mm = user.mm().expect("statfs copyout has no MM");
    copyout(&mm, address, &bytes)?;
    Ok(0)
}

/// 将内核文件系统统计数据 `stat` 与挂载访问模式 `mode` 编码为 `asm-generic` LP64 ABI 的 120 字节 `struct statfs` 字节数组，所有保留字段填零。
fn encode_statfs(stat: FilesystemStat, mode: AccessMode) -> [u8; STATFS_SIZE] {
    let flags = ST_VALID
        | if mode == AccessMode::ReadOnly {
            ST_RDONLY
        } else {
            0
        };
    // 按 LP64 struct statfs 布局：前 7 项各占 8 字节；偏移 56 的 f_fsid 占 8 字节，其后依次为 f_namelen、f_frsize 和 f_flags，尾部保留区填 0。
    let fields = [
        stat.fs_type,
        stat.block_size,
        stat.blocks,
        stat.free,
        stat.available,
        stat.files,
        stat.freefiles,
        stat.device,
        NAME_MAX as u64,
        stat.block_size,
        flags,
    ];
    let mut bytes = [0u8; STATFS_SIZE];
    for (index, value) in fields.iter().enumerate() {
        bytes[index * 8..index * 8 + 8].copy_from_slice(&value.to_le_bytes());
    }
    bytes
}

/// `mount` 系统调用：挂载新的文件系统实例（支持 `ext4`、`tmpfs`、`proc`、`devfs`），或对已挂载的 `tmpfs` 执行重新挂载（`MS_REMOUNT`）以调整读写模式与配额选项。
///
/// - **调用上下文**：普通可睡眠用户任务上下文，仅允许有效 UID 为 0 的特权进程调用。
/// - **参数**：`raw[0]` 为源设备路径地址，`raw[1]` 为挂载目标目录路径地址，`raw[2]` 为文件系统类型字符串地址，`raw[3]` 为挂载标志位（`MS_*`），`raw[4]` 为挂载选项字符串地址。
/// - **返回值**：成功返回 `Ok(0)`；非 root 用户返回 `Err(Errno::NotPermitted)`；标志位、设备或路径非法时返回对应 `Errno`。
pub(super) fn mount(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    // Step 1: 校验调用者特权（UID == 0）以及挂载标志位的合法性。
    let runtime = user.runtime().expect("mount has no runtime");
    if runtime.credentials.uid != 0 {
        return Err(Errno::NotPermitted);
    }
    let remount = raw[3] & MS_REMOUNT != 0;
    if raw[3] & !(MS_RDONLY | MS_SILENT | MS_REMOUNT) != 0 || (!remount && raw[2] == 0) {
        return Err(Errno::InvalidArgument);
    }

    // Step 2: 从用户空间读取源设备、目标路径、文件系统类型与挂载选项字符串。
    let mm = user.mm().expect("mount has no MM");
    let source = if remount || raw[0] == 0 {
        None
    } else {
        Some(mm.read_c_string(raw[0], PATH_MAX)?)
    };
    let target = mm.read_c_string(raw[1], PATH_MAX)?;
    let kind = if remount {
        None
    } else {
        Some(mm.read_c_string(raw[2], MOUNT_STRING_LIMIT)?)
    };
    let options = if raw[4] == 0 {
        None
    } else {
        Some(mm.read_c_string(raw[4], MOUNT_STRING_LIMIT)?)
    };
    let options = options.as_deref().unwrap_or(b"");
    let requested_mode = if raw[3] & MS_RDONLY != 0 {
        AccessMode::ReadOnly
    } else {
        AccessMode::ReadWrite
    };

    // Step 3: 若为 MS_REMOUNT，解析目标路径并在原挂载实例上更新访问模式与 tmpfs 配额参数。
    if remount {
        let (vfs, context) = {
            let fs = runtime.fs.lock();
            (fs.vfs.clone(), fs.context.clone())
        };
        let path = vfs.resolve_from(
            &context,
            context.cwd(),
            &target,
            ResolveMode::Follow,
            Some((runtime.credentials.uid, runtime.credentials.gid)),
        )?;
        path.reconfigure_tmpfs(requested_mode, options)?;
        return Ok(0);
    }

    // Step 4: 根据文件系统类型构造新的后端文件系统实例（打开块设备上的 ext4 卷或初始化内存文件系统）。
    let (filesystem, mode) = match kind.as_deref().expect("new mount has no type") {
        b"ext4" => {
            if !options.is_empty() {
                return Err(Errno::InvalidArgument);
            }
            let source = source.as_deref().ok_or(Errno::NoDevice)?;
            let selector = DeviceSelector::parse(source).map_err(|_| Errno::NoDevice)?;
            let storage = Storage::get().ok_or(Errno::NoDevice)?;
            let view = storage.lookup(selector).ok_or(Errno::NoDevice)?;
            let volume = Ext4Volume::open(view, requested_mode, MOUNT_METADATA_CAPACITY)?;
            (Filesystem::Ext4(volume), requested_mode)
        }
        b"tmpfs" => {
            let filesystem = MemoryFs::try_tmpfs(options)?;
            (Filesystem::Memory(filesystem), requested_mode)
        }
        b"proc" | b"devfs" => {
            if !options.is_empty() {
                return Err(Errno::InvalidArgument);
            }
            let filesystem = if kind.as_deref() == Some(b"proc".as_slice()) {
                procfs::try_new(0o555)?
            } else {
                devfs::try_new(0o755)?
            };
            (Filesystem::Memory(filesystem), AccessMode::ReadOnly)
        }
        _ => return Err(Errno::NoDevice),
    };

    // Step 5: 解析挂载点路径并将新文件系统实例注册到 VFS 挂载表中。
    let (vfs, context) = {
        let fs = runtime.fs.lock();
        (fs.vfs.clone(), fs.context.clone())
    };
    let path = vfs.resolve_from(
        &context,
        context.cwd(),
        &target,
        ResolveMode::Follow,
        Some((runtime.credentials.uid, runtime.credentials.gid)),
    )?;

    vfs.mount(&path, filesystem, mode)?;
    Ok(0)
}

/// `umount2` 系统调用：卸载指定路径上的子挂载点。
///
/// - **调用上下文**：普通可睡眠用户任务上下文，仅允许 UID 为 0 的特权进程调用。
/// - **参数**：`raw[0]` 为目标挂载点路径地址，`raw[1]` 为卸载标志位（仅支持 `0` 或 `UMOUNT_NOFOLLOW`）。
/// - **返回值**：成功返回 `Ok(0)`；权限不足返回 `Err(Errno::NotPermitted)`；标志位非法、目标非挂载根或挂载点正忙时返回对应 `Errno`。
pub(super) fn umount(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("umount has no runtime");
    if runtime.credentials.uid != 0 {
        return Err(Errno::NotPermitted);
    }
    if raw[1] & !UMOUNT_NOFOLLOW != 0 {
        return Err(Errno::InvalidArgument);
    }

    let mm = user.mm().expect("umount has no MM");
    let target = mm.read_c_string(raw[0], PATH_MAX)?;
    let (vfs, context) = {
        let fs = runtime.fs.lock();
        (fs.vfs.clone(), fs.context.clone())
    };
    let mode = if raw[1] & UMOUNT_NOFOLLOW != 0 {
        ResolveMode::NoFollow
    } else {
        ResolveMode::Follow
    };
    let path = vfs.resolve_from(
        &context,
        context.cwd(),
        &target,
        mode,
        Some((runtime.credentials.uid, runtime.credentials.gid)),
    )?;

    vfs.unmount_path(path)?;
    Ok(0)
}

/// `sync` 系统调用：将当前 VFS 命名空间下所有可写挂载点的脏缓存数据刷写到底层存储设备。
///
/// - **调用上下文**：普通可睡眠用户任务上下文。按照 Linux `sync(2)` 规范，该调用等待同步完成但固定返回成功，底层写回错误由各打开文件描述符的 `fsync` 独立报告。
pub(super) fn sync(user: &UserThread) -> SysResult {
    let runtime = user.runtime().expect("sync has no runtime");
    let vfs = runtime.fs.lock().vfs.clone();
    let _ = vfs.sync();
    Ok(0)
}

/// `reboot` 系统调用：校验调用者 root 权限与 reboot 魔数，并向系统级进程关系图发起停机或重启请求。
///
/// - **调用上下文**：普通用户任务上下文，要求调用者 UID 为 0 且所处进程命名空间的根角色为 `RootRole::System`。
/// - **参数**：`raw[0]` 与 `raw[1]` 为防误触魔数（`REBOOT_MAGIC1` / `REBOOT_MAGIC2*`），`raw[2]` 为操作命令码（`REBOOT_CMD_*`）。
/// - **返回值**：若为 `REBOOT_CMD_CAD_OFF` 直接返回 `Outcome::Return(0)`；若发起停机/重启成功，返回 `Outcome::GroupExit(0)` 使当前线程组退栈并由内核完成停机流程。
pub(super) fn reboot(user: &UserThread, raw: &[usize; 6]) -> Outcome {
    // Step 1: 验证 root 权限与 reboot 双魔数。
    let runtime = user.runtime().expect("reboot has no runtime");
    if runtime.credentials.uid != 0 {
        return Outcome::Return(Errno::NotPermitted.result());
    }
    let magic1 = raw[0] as u32;
    let magic2 = raw[1] as u32;
    let command = raw[2] as u32;
    if magic1 != REBOOT_MAGIC1
        || !matches!(
            magic2,
            REBOOT_MAGIC2 | REBOOT_MAGIC2A | REBOOT_MAGIC2B | REBOOT_MAGIC2C
        )
    {
        return Outcome::Return(Errno::InvalidArgument.result());
    }
    let operation = match command {
        // 串口控制台无 Ctrl-Alt-Del 硬键事件源，CAD 始终处于禁用状态，因此 CAD_OFF 直接返回成功。
        REBOOT_CMD_CAD_OFF => return Outcome::Return(0),
        REBOOT_CMD_RESTART => ShutdownOperation::Restart,
        REBOOT_CMD_HALT => ShutdownOperation::Halt,
        REBOOT_CMD_POWER_OFF => ShutdownOperation::PowerOff,
        _ => return Outcome::Return(Errno::InvalidArgument.result()),
    };

    // Step 2: 确认当前进程处于根系统进程图（而非隔离容器），向进程图登记关机动作并触发当前进程组退出。
    let identity = user.identity().expect("reboot has no identity");
    let graph = identity.graph();
    if graph.root_role() != RootRole::System {
        return Outcome::Return(Errno::InvalidArgument.result());
    }

    match graph.request_shutdown(operation) {
        Ok(()) => Outcome::GroupExit(0),
        Err(error) => Outcome::Return(Errno::from(error).result()),
    }
}
