//! 系统身份、挂载、同步与关机准入消费当前用户的真实 MM、VFS、凭据和身份图。

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

/// 向本次 MM 交付固定系统身份；用户访问可处理缺页并睡眠，部分故障保留已写前缀并返回 EFAULT。
pub(super) fn uname(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let mm = user.mm().expect("uname has no MM");
    copyout(&mm, raw[0], &UTS_NAME)?;
    Ok(0)
}

/// 交付 LP64 sysinfo 的实际 RAM、运行时间及活执行者快照，复制期间不持调度或 MM 锁。
pub(super) fn sysinfo(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let memory = mm::stats();
    let elapsed = time::now().as_nanos();
    let uptime = elapsed.div_ceil(time::NSEC_PER_SEC);
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

    let mut bytes = [0u8; 112];
    for (index, value) in fields.iter().enumerate() {
        bytes[index * 8..index * 8 + 8].copy_from_slice(&value.to_le_bytes());
    }
    // ABI: procs 位于80，两个high字段位于88/96，mem_unit位于104；显式填零全部padding。
    // 没有 swap 或 highmem；共享页、buffer 和 load average 尚无独立统计来源。
    bytes[80..82].copy_from_slice(&(sched::live_task_count() as u16).to_le_bytes());
    bytes[104..108].copy_from_slice(&(mm::PAGE_SIZE as u32).to_le_bytes());

    let mm = user.mm().expect("sysinfo has no MM");
    copyout(&mm, raw[0], &bytes)?;
    Ok(0)
}

/// 在编译期构造六个 NUL 结尾、余部为零的固定身份字段，不依赖 Rust 结构体布局。
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

/// 一次导入路径并沿当前 root/cwd 跟随链接，采样所选实际挂载后在锁外交付容量。
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

/// 一次取得 fd 的原 OFD并借其 Path；无效 fd 返回 EBADF，未具文件系统来源的对象保 ENOSYS。
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

/// 后端采样结束后一次交付完整字节，用户缺页和部分写入故障沿原 copyout 返回 EFAULT。
fn write_statfs(user: &UserThread, path: &Path, address: usize) -> SysResult {
    let stat = path.filesystem().statfs()?;
    let bytes = encode_statfs(stat, path.mount_mode());
    let mm = user.mm().expect("statfs copyout has no MM");
    copyout(&mm, address, &bytes)?;
    Ok(0)
}

/// 编码 asm-generic LP64 的 120 字节布局，所有保留字节为零，不创建用户结构体引用。
fn encode_statfs(stat: FilesystemStat, mode: AccessMode) -> [u8; STATFS_SIZE] {
    let flags = ST_VALID
        | if mode == AccessMode::ReadOnly {
            ST_RDONLY
        } else {
            0
        };
    // ABI: 前七项各占八字节；fsid 是低、高两个四字节字，之后依次为 namelen、frsize 和 flags。
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

/// 新挂载构造真实后端；tmpfs 重配只修改目标原实例的已出现额度字段。
pub(super) fn mount(user: &UserThread, raw: &[usize; 6]) -> SysResult {
    let runtime = user.runtime().expect("mount has no runtime");
    if runtime.credentials.uid != 0 {
        return Err(Errno::NotPermitted);
    }
    let remount = raw[3] & MS_REMOUNT != 0;
    if raw[3] & !(MS_RDONLY | MS_SILENT | MS_REMOUNT) != 0 || (!remount && raw[2] == 0) {
        return Err(Errno::InvalidArgument);
    }

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

/// 普通卸载仅接受零或 NOFOLLOW，消费本次查找所得真实子挂载根及其 owning 引用。
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

/// 等待当前 VFS 全部真实挂载完成同步；系统调用不报告后端错误，OFD 错误样本保持独立。
pub(super) fn sync(user: &UserThread) -> SysResult {
    let runtime = user.runtime().expect("sync has no runtime");
    let vfs = runtime.fs.lock().vfs.clone();
    let _ = vfs.sync();
    Ok(0)
}

/// 先核真实 uid 与 32 位 reboot 参数，停机意图交给系统图，当前用户只沿组退出离栈。
pub(super) fn reboot(user: &UserThread, raw: &[usize; 6]) -> Outcome {
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
        // 串口平台没有 CAD 键事件源，其禁用状态固定；重复禁用不另造无人读取的开关。
        REBOOT_CMD_CAD_OFF => return Outcome::Return(0),
        REBOOT_CMD_RESTART => ShutdownOperation::Restart,
        REBOOT_CMD_HALT => ShutdownOperation::Halt,
        REBOOT_CMD_POWER_OFF => ShutdownOperation::PowerOff,
        _ => return Outcome::Return(Errno::InvalidArgument.result()),
    };

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
