//! `procfs` 进程与系统状态伪文件系统实现。
//!
//! ## 模块职责与架构定位
//! 本模块负责在内存文件系统（[`MemoryFs`]）之上构建只读的 `/proc` 目录树（[`try_new`]），
//! 并在用户或内核打开具体信息节点时即时采样相关子系统状态，生成不可变的文本快照（[`SnapshotFile`]）。
//! 上游调用者为 VFS 挂载与打开路径（`Vfs` 初始化挂载 `/proc`，打开 `MemorySpecial` 节点时调用 [`open`]）；
//! 下游依赖启动信息（[`crate::boot`]）、CPU 与中断统计（[`crate::cpu`]、[`crate::irq`]）、
//! 物理内存统计（[`crate::mm`]）、时间与调度账务（[`crate::time`]、[`crate::accounting`]）、
//! VFS 挂载表以及当前进程所属的网络协议栈（[`crate::net::NetStack`]）。
//!
//! ## `/proc` 目录结构与按打开采样（Snapshot-on-Open）机制
//!
//! ```text
//! /proc (MemoryFsKind::Procfs, 只读)
//!  ├── cmdline     (0444) ──► MemorySpecial::Cmdline    ──► 启动命令行参数
//!  ├── mounts      (0444) ──► MemorySpecial::Mounts     ──► 当前根目录可见的挂载表
//!  ├── meminfo     (0444) ──► MemorySpecial::Meminfo    ──► 物理内存与 Slab 页统计
//!  ├── uptime      (0444) ──► MemorySpecial::Uptime     ──► 系统单调运行时间与累计空闲时间
//!  ├── interrupts  (0444) ──► MemorySpecial::Interrupts ──► 各在线 CPU 的定时器/IPI/外部中断计数
//!  └── net/        (0555)
//!       ├── dev    (0444) ──► MemorySpecial::NetDev     ──► 当前网络命名空间的网卡收发统计
//!       └── route  (0444) ──► MemorySpecial::NetRoute   ──► 当前网络命名空间的 IPv4 路由表
//!
//! 打开与读取生命周期:
//!   open("/proc/...", O_RDONLY)
//!       │
//!       ├─► 1. 从对应内核子系统提取瞬时状态并格式化为 Vec<u8> (SnapshotText)
//!       └─► 2. 封装为 SnapshotFile { path, bytes, cursor: 0, flags }
//!               │
//!               ├─► read / lseek : 在 cursor (SleepMutex) 保护下移动游标并拷贝快照切片
//!               └─► pread        : 无锁按指定 offset 直接读取不可变的 bytes 快照
//! ```
//!
//! 每次调用 `open` 都会生成一份独立的文本快照，后续在该文件描述符上的多次 `read` / `lseek`
//! 始终观察同一份稳定文本，避免分批读取时因系统状态变化导致行内容错位；若需观察最新状态，需重新 `open`。
//!
//! ## 锁获取顺序与并发契约
//! - **快照生成阶段（`open`）**：在获取各子系统（VFS 挂载表、网络协议栈、内存分配器、中断控制器）内部锁时，仅拷贝标量统计数据，随后在锁外使用 [`SnapshotText`] 执行字符串格式化与堆内存扩容，缩短子系统锁的持有时间。
//! - **快照读取阶段（`read_into` / `seek`）**：仅获取 [`SnapshotFile::cursor`]（[`SleepMutex`]）保护游标推进；`SnapshotFile` 不反向持有 `Vfs` 或 `NetStack` 引用。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt::{self, Write};
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::boot::{self, MAX_CPUS};
use crate::fs::file::{require_read, set_nonblocking, set_status_flags, stat_path};
use crate::fs::memfs::{MemoryFs, MemoryFsKind, MemorySpecial};
use crate::fs::{
    DirectoryEntry, FileIoContext, FileLike, FileRef, FileStat, FsError, InodeRef, Path,
    ReadBuffer, SeekFrom, Vfs, WriteBuffer, O_ACCMODE, O_RDONLY, O_TRUNC,
};
use crate::net::route::{IPV4_ADDRESS_BITS, RTF_GATEWAY, RTF_UP};
use crate::net::{NetStack, INTERFACE_COUNT};
use crate::sync::SleepMutex;
use crate::{accounting, cpu, irq, mm, time};

/// 生成 `/proc` 文本快照时预分配的初始缓冲区容量（单位：字节）。
const SNAPSHOT_INITIAL_CAPACITY: usize = 256;
/// 每 KiB 包含的字节数（1024 字节），用于 `/proc/meminfo` 页数到 `kB` 的换算。
const BYTES_PER_KIB: usize = 1024;
/// `/proc/uptime` 小数部分的百分之一秒刻度数（100 ticks/s）。
const UPTIME_TICKS_PER_SEC: u64 = 100;
/// `/proc/uptime` 每个百分之一秒刻度对应的纳秒数（$10^7\text{ ns}$）。
const UPTIME_TICK_NANOS: u64 = time::NSEC_PER_SEC / UPTIME_TICKS_PER_SEC;

/// `/proc` 动态信息节点的打开文件描述（Open File Description）。
///
/// 在 `open` 时一次性生成完整文本并保存在 `bytes` 中；同一打开实例的多个复制描述符共享读取游标 `cursor`，
/// 只有重新打开文件才会重新采样系统状态。
pub struct SnapshotFile {
    /// 打开该节点时的挂载路径与目录项引用，用于 `fstat` 查询属性及维持挂载点生命周期。
    path: Path,
    /// 在 `open` 时刻采样并格式化完成的不可变文本内容。
    bytes: Vec<u8>,
    /// 顺序读写与 `lseek` 共享的字节偏移游标（单位：字节），由可睡眠互斥锁保护。
    cursor: SleepMutex<u64>,
    /// 打开文件状态标志（如 `O_RDONLY`、`O_NONBLOCK`），支持原子读取与更新。
    flags: AtomicU32,
}

/// 支持内存分配失败传播（Fallible Allocation）的快照文本格式化缓冲器。
///
/// 由于 `core::fmt::Write` 接口仅能返回无载荷的 `fmt::Error`，本结构体在扩容失败或长度溢出时
/// 将具体 [`FsError`] 暂存在 `error` 字段中，并在格式化结束调用 [`SnapshotText::finish`] 时还原。
struct SnapshotText {
    /// 累积已格式化 UTF-8 文本的动态字节数组。
    bytes: Vec<u8>,
    /// 格式化过程中首次遇到的内存分配或长度溢出错误。
    error: Option<FsError>,
}

impl SnapshotText {
    /// 创建快照文本缓冲器并预分配 [`SNAPSHOT_INITIAL_CAPACITY`]（256 字节）的初始容量。
    fn try_new() -> Result<Self, FsError> {
        let mut bytes = Vec::new();
        bytes
            .try_reserve_exact(SNAPSHOT_INITIAL_CAPACITY)
            .map_err(|_| FsError::NoMemory)?;
        Ok(Self { bytes, error: None })
    }

    /// 结束格式化并提取完整字节数组；若格式化过程中发生错误，则返回暂存的 [`FsError`]。
    fn finish(self, result: fmt::Result) -> Result<Vec<u8>, FsError> {
        match result {
            Ok(()) => Ok(self.bytes),
            Err(_) => Err(self
                .error
                .expect("snapshot formatting failed without source error")),
        }
    }
}

impl Write for SnapshotText {
    /// 在追加字符串片段前先执行长度溢出检查与可失败容量预留（`try_reserve`），失败时记录具体错误并中断格式化。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let result = self
            .bytes
            .len()
            .checked_add(text.len())
            .ok_or(FsError::FileTooLarge)
            .and_then(|_| {
                self.bytes
                    .try_reserve(text.len())
                    .map_err(|_| FsError::NoMemory)
            });
        if let Err(error) = result {
            self.error = Some(error);
            return Err(fmt::Error);
        }
        self.bytes.extend_from_slice(text.as_bytes());
        Ok(())
    }
}

/// 创建并初始化只读的 `procfs` 内存文件系统实例，在 `/proc` 及 `/proc/net` 下安装固定信息节点。
///
/// 各节点仅登记对应的 [`MemorySpecial`] 角色标识，不在挂载阶段提前生成任何快照文本。
pub fn try_new(root_mode: u16) -> Result<Arc<MemoryFs>, FsError> {
    let fs = MemoryFs::try_new(MemoryFsKind::Procfs, root_mode)?;
    let root = fs.root();
    fs.initialize_special(&root, b"cmdline", MemorySpecial::Cmdline, 0o444, 0)?;
    fs.initialize_special(&root, b"mounts", MemorySpecial::Mounts, 0o444, 0)?;
    fs.initialize_special(&root, b"meminfo", MemorySpecial::Meminfo, 0o444, 0)?;
    fs.initialize_special(&root, b"uptime", MemorySpecial::Uptime, 0o444, 0)?;
    fs.initialize_special(&root, b"interrupts", MemorySpecial::Interrupts, 0o444, 0)?;

    let net = fs.initialize_directory(&root, b"net", 0o555)?;
    fs.initialize_special(&net, b"dev", MemorySpecial::NetDev, 0o444, 0)?;
    fs.initialize_special(&net, b"route", MemorySpecial::NetRoute, 0o444, 0)?;
    Ok(fs)
}

/// 打开 `/proc` 下的动态信息节点并生成当前时刻的文本快照。
///
/// - 调用上下文：普通可睡眠任务上下文。
/// - 参数：
///   - `path`：目标节点的解析路径。
///   - `flags`：打开标志位；必须为只读（`O_RDONLY`）且不含 `O_TRUNC`，否则返回 [`FsError::ReadOnly`]。
///   - `special`：节点对应的动态数据源类型。
///   - `vfs`：全局虚拟文件系统引用，用于生成 `/proc/mounts`。
///   - `caller`：调用方上下文；访问 `/proc/mounts` 时取调用方根目录过滤可见挂载点，访问 `/proc/net/*` 时取调用方所属的网络协议栈。
/// - 返回值：成功返回包含完整快照文本的 [`FileRef`]。
pub fn open(
    path: Path,
    flags: u32,
    special: MemorySpecial,
    vfs: &Vfs,
    caller: FileIoContext<'_>,
) -> Result<FileRef, FsError> {
    if flags & O_ACCMODE != O_RDONLY || flags & O_TRUNC != 0 {
        return Err(FsError::ReadOnly);
    }

    let bytes = match special {
        MemorySpecial::Cmdline => cmdline()?,
        MemorySpecial::Mounts => {
            let root = match caller {
                FileIoContext::Kernel => vfs.root(),
                FileIoContext::User(thread) => {
                    let runtime = thread.runtime().ok_or(FsError::InvalidOperation)?;
                    let fs = runtime.fs.lock();
                    fs.context.root().clone()
                }
            };
            vfs.mounts_text(&root)?
        }
        MemorySpecial::Meminfo => meminfo()?,
        MemorySpecial::Uptime => uptime()?,
        MemorySpecial::Interrupts => interrupts()?,
        MemorySpecial::NetDev | MemorySpecial::NetRoute => {
            let runtime = match caller {
                FileIoContext::User(thread) => thread.runtime().ok_or(FsError::InvalidOperation)?,
                FileIoContext::Kernel => return Err(FsError::InvalidOperation),
            };
            let net = runtime.net.as_ref().ok_or(FsError::NetworkUnreachable)?;
            if special == MemorySpecial::NetDev {
                net_dev(net)?
            } else {
                net_route(net)?
            }
        }
        _ => return Err(FsError::InvalidOperation),
    };

    Ok(Arc::try_new(SnapshotFile {
        path,
        bytes,
        cursor: SleepMutex::new(0),
        flags: AtomicU32::new(flags),
    })
    .map_err(|_| FsError::NoMemory)?)
}

/// 生成 `/proc/cmdline` 快照：原样拷贝固件/引导器传入的启动命令行字节并在末尾追加换行符 `\n`。
fn cmdline() -> Result<Vec<u8>, FsError> {
    let bootargs = boot::info().bootargs();
    let length = bootargs.len().checked_add(1).ok_or(FsError::FileTooLarge)?;
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(length)
        .map_err(|_| FsError::NoMemory)?;
    bytes.extend_from_slice(bootargs);
    bytes.push(b'\n');
    Ok(bytes)
}

/// 生成 `/proc/interrupts` 快照：按当前在线 CPU 列输出本地定时器中断、核间中断（IPI）以及已触发外部中断源的累计次数。
fn interrupts() -> Result<Vec<u8>, FsError> {
    // Step 1: 采样当前在线的 CPU 掩码，固定本次输出的列集合。
    let mut online = [false; MAX_CPUS];
    for (owner, column) in online.iter_mut().enumerate().take(cpu::count()) {
        *column = cpu::get(owner).online.load(Ordering::Acquire);
    }

    // Step 2: 在中断控制器锁外按行格式化表头、本地中断（timer/IPI）与外部中断源计数。
    let mut text = SnapshotText::try_new()?;
    let result = (|| {
        text.write_str("     ")?;
        for (owner, &is_online) in online.iter().enumerate() {
            if is_online {
                write!(text, "CPU{:<8}", owner)?;
            }
        }
        text.write_char('\n')?;

        for (number, label) in [(0, "timer"), (1, "IPI")] {
            write!(text, "{:>3}: ", number)?;
            for (owner, &is_online) in online.iter().enumerate() {
                if is_online {
                    let local = cpu::get(owner);
                    let count = if number == 0 {
                        local.timer_interrupts.load(Ordering::Relaxed)
                    } else {
                        local.ipi_interrupts.load(Ordering::Relaxed)
                    };
                    write!(text, "{:10} ", count)?;
                }
            }
            writeln!(text, "{}", label)?;
        }

        for source in 0..irq::source_count() {
            let Some(snapshot) = irq::source_snapshot(source) else {
                continue;
            };
            if !snapshot.observed {
                continue;
            }
            write!(text, "{:>3}: ", source + 2)?;
            for (owner, &is_online) in online.iter().enumerate() {
                if is_online {
                    write!(text, "{:10} ", snapshot.interrupts[owner])?;
                }
            }
            writeln!(text, "external source {}", source)?;
        }
        Ok(())
    })();
    text.finish(result)
}

/// 生成 `/proc/net/dev` 快照：先从网络栈拷贝各网卡接口的收发统计快照，再格式化为标准 Linux `/proc/net/dev` 表格。
///
/// 当前硬件未单独统计的 FIFO、帧错误、压缩、组播、冲突与载波错误列固定输出 `0`。
fn net_dev(net: &NetStack) -> Result<Vec<u8>, FsError> {
    let interfaces = net.interface_snapshots();
    let mut snapshots = [None; INTERFACE_COUNT];
    for (slot, interface) in snapshots.iter_mut().zip(interfaces.iter()) {
        if let Some(interface) = interface {
            *slot = Some((interface.id, net.statistics(interface.id)?));
        }
    }

    let mut text = SnapshotText::try_new()?;
    let result = (|| {
        text.write_str(concat!(
            "Inter-|   Receive                            ",
            "                    |  Transmit\n",
            " face |bytes    packets errs drop fifo frame ",
            "compressed multicast|bytes    packets errs ",
            "drop fifo colls carrier compressed\n",
        ))?;
        for (interface, stats) in snapshots.iter().flatten() {
            writeln!(
                text,
                "{:>6}: {:7} {:7} {:4} {:4} {:4} {:5} {:10} {:9} {:8} {:7} {:4} {:4} {:4} {:5} {:7} {:10}",
                interface.name(),
                stats.rx_bytes,
                stats.rx_packets,
                stats.rx_errors,
                stats.rx_dropped,
                0, 0, 0, 0,
                stats.tx_bytes,
                stats.tx_packets,
                stats.tx_errors,
                stats.tx_dropped,
                0, 0, 0, 0,
            )?;
        }
        Ok(())
    })();
    text.finish(result)
}

/// 生成 `/proc/net/route` 快照：读取当前网络栈的 IPv4 路由表快照，将目的网络、网关与子网掩码按小端序 8 位十六进制格式输出。
fn net_route(net: &NetStack) -> Result<Vec<u8>, FsError> {
    let routes = net.route_snapshots();
    let mut text = SnapshotText::try_new()?;
    let result = (|| {
        text.write_str(
            "Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT\n",
        )?;
        for route in routes.iter().flatten() {
            let mask = if route.prefix_len == 0 {
                0
            } else {
                u32::MAX << (IPV4_ADDRESS_BITS - route.prefix_len)
            };
            let flags = if route.up { RTF_UP } else { 0 }
                | if route.gateway.is_some() {
                    RTF_GATEWAY
                } else {
                    0
                };
            writeln!(
                text,
                "{}\t{:08X}\t{:08X}\t{:04X}\t0\t0\t0\t{:08X}\t0\t0\t0",
                route.interface.name(),
                u32::from_le_bytes(route.destination),
                u32::from_le_bytes(route.gateway.unwrap_or([0; 4])),
                flags,
                u32::from_le_bytes(mask.to_be_bytes()),
            )?;
        }
        Ok(())
    })();
    text.finish(result)
}

/// 生成 `/proc/meminfo` 快照：根据物理页分配器统计计算受管总内存（`MemTotal`）、空闲内存（`MemFree`）及 Slab 占用（`Slab`），单位为 `kB`。
fn meminfo() -> Result<Vec<u8>, FsError> {
    let stats = mm::stats();
    let managed = stats.present_pages - stats.reserved_pages - stats.metadata_pages;
    let free = stats.buddy_free_pages + stats.pcp_cached_pages;
    let kb_per_page = mm::PAGE_SIZE / BYTES_PER_KIB;
    let mut text = SnapshotText::try_new()?;
    let result = write!(
        text,
        "MemTotal: {:8} kB\nMemFree:  {:8} kB\nSlab:     {:8} kB\n",
        managed * kb_per_page,
        free * kb_per_page,
        stats.slab_backing_pages * kb_per_page
    );
    text.finish(result)
}

/// 生成 `/proc/uptime` 快照：输出系统启动以来的单调运行秒数与所有 CPU 累计空闲（idle）秒数（均保留两位小数）。
fn uptime() -> Result<Vec<u8>, FsError> {
    let elapsed = time::now().as_nanos();
    let idle = (0..cpu::count())
        .map(|owner| u128::from(accounting::snapshot(owner).idle_ns))
        .sum::<u128>();
    let mut text = SnapshotText::try_new()?;
    let result = writeln!(
        text,
        "{}.{:02} {}.{:02}",
        elapsed / time::NSEC_PER_SEC,
        elapsed / UPTIME_TICK_NANOS % UPTIME_TICKS_PER_SEC,
        idle / u128::from(time::NSEC_PER_SEC),
        idle / u128::from(UPTIME_TICK_NANOS) % u128::from(UPTIME_TICKS_PER_SEC)
    );
    text.finish(result)
}

impl SnapshotFile {
    /// 从不可变快照 `self.bytes` 的指定偏移 `offset` 处向目标缓冲区拷贝数据。
    ///
    /// 无需持有内容锁；若拷贝途中发生用户态缺页错误，在已成功拷贝部分字节时返回已拷贝长度，在首字节即失败时返回错误。
    fn read_snapshot(&self, offset: u64, target: &mut ReadBuffer<'_>) -> Result<usize, FsError> {
        if self.bytes.len() as u64 <= offset || target.copied() == target.len() {
            return Ok(0);
        }
        let start = target.copied();
        let source = &self.bytes[offset as usize..];
        match target.push(source) {
            Ok(count) => Ok(count),
            Err(error) if target.copied() == start => Err(error),
            Err(_) => Ok(target.copied() - start),
        }
    }
}

impl FileLike for SnapshotFile {
    /// `/proc` 快照文件支持 `pread` 按显式字节偏移读取。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 从当前游标位置顺序读取快照内容，并在成功拷贝后推进共享游标。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        let mut cursor = self.cursor.lock();
        let count = self.read_snapshot(*cursor, target)?;
        *cursor += count as u64;
        Ok(count)
    }

    /// 从指定字节偏移 `offset` 处读取快照内容（`pread`），不修改共享游标。
    fn read_at_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        self.read_snapshot(offset, target)
    }

    /// 返回打开该节点时的 [`Path`]，用于查询底层 Inode 属性并维持挂载点生命周期。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// `/proc` 动态快照文件不支持 `mmap` 页缓存映射，返回 `None`。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 原子读取当前打开文件实例的状态标志位。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 更新 `fcntl(F_SETFL)` 允许修改的文件状态标志位，不改变只读访问模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除当前打开实例的 `O_NONBLOCK` 非阻塞标志位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// `/proc` 节点为只读快照，不支持截断操作，返回 [`FsError::ReadOnly`]。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::ReadOnly)
    }

    /// `/proc` 内存快照无持久化后端，不支持 `fsync` / `fdatasync`，返回 [`FsError::InvalidOperation`]。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 根据快照文本字节长度计算并更新读取游标位置；若目标位置为负数或超出 `i64::MAX`，则返回 [`FsError::InvalidOperation`] 且不修改游标。
    fn seek(&self, from: SeekFrom) -> Result<u64, FsError> {
        let mut cursor = self.cursor.lock();
        let position = match from {
            SeekFrom::Start(position) => Some(position),
            SeekFrom::Current(delta) => cursor.checked_add_signed(delta),
            SeekFrom::End(delta) => (self.bytes.len() as u64).checked_add_signed(delta),
        }
        .ok_or(FsError::InvalidOperation)?;
        if (i64::MAX as u64) < position {
            return Err(FsError::InvalidOperation);
        }

        *cursor = position;
        Ok(position)
    }

    /// 查询文件元数据：Inode 身份与权限取自底层 `/proc` 节点，`size` 设为当前打开实例已生成的快照字节长度，`blocks` 为 `0`。
    fn stat(&self) -> Result<FileStat, FsError> {
        let mut stat = stat_path(&self.path)?;
        stat.size = self.bytes.len() as u64;
        stat.blocks = 0;
        Ok(stat)
    }

    /// 快照文件不是目录，拒绝 `readdir` 并返回 [`FsError::NotDirectory`]。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 快照文件不是目录，拒绝目录遍历并返回 [`FsError::NotDirectory`]。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// `/proc` 快照文件固定为只读，任何写入尝试均返回 [`FsError::BadFd`]。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }

    /// `/proc` 快照文件固定为只读，按偏移写入同样返回 [`FsError::BadFd`]。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }
}
