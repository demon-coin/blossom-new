//! 动态信息在每次打开时取实际来源快照，OFD 不反向保有 VFS。

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

const SNAPSHOT_INITIAL_CAPACITY: usize = 256;
const BYTES_PER_KIB: usize = 1024;
const UPTIME_TICKS_PER_SEC: u64 = 100;
const UPTIME_TICK_NANOS: u64 = time::NSEC_PER_SEC / UPTIME_TICKS_PER_SEC;

/// 每个实际打开实例拥有本次文本和共享游标，重新打开才观察新的系统状态。
pub struct SnapshotFile {
    path: Path,
    bytes: Vec<u8>,
    cursor: SleepMutex<u64>,
    flags: AtomicU32,
}

/// 格式化按实际文本 fallible 增长，原始分配错误保留到格式化边界返回。
struct SnapshotText {
    bytes: Vec<u8>,
    error: Option<FsError>,
}

impl SnapshotText {
    /// 预留小信息文件所需空间，拒绝无法建立本次快照的打开。
    fn try_new() -> Result<Self, FsError> {
        let mut bytes = Vec::new();
        bytes
            .try_reserve_exact(SNAPSHOT_INITIAL_CAPACITY)
            .map_err(|_| FsError::NoMemory)?;
        Ok(Self { bytes, error: None })
    }

    /// 完整格式化才移交文本，不交出已截断或分配失败的内容。
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
    /// 每个格式片段先检查长度运算并准备容量，然后完整追加。
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

/// 建立受保护的 procfs 权威目录，固定节点只保存来源角色，不提前生成快照。
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

/// 可睡眠上下文生成独立快照；网络来源取本次用户域，VFS 与网络引用均不移交 OFD。
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

/// 复制固件命令行的完整原字节并追加一个换行，不解释内容或改变编码。
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

/// 固定本次 online CPU 列，逐行复制实际计数；源锁外有界渲染，不承诺跨核同时冻结。
fn interrupts() -> Result<Vec<u8>, FsError> {
    let mut online = [false; MAX_CPUS];
    for (owner, column) in online.iter_mut().enumerate().take(cpu::count()) {
        *column = cpu::get(owner).online.load(Ordering::Acquire);
    }

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

/// 先复制接口和 Root 提供的计数，再锁外格式化；未监测的 FIFO、frame、压缩、组播、碰撞及硬件 TX 错误为零。
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

/// 路由来自当次库 CIDR 与默认项，地址和掩码按目标 little-endian 十六进制输出，未监测列为零。
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

/// 只输出现有账务能给出的可管理 RAM、空页与 slab backing，不估计可用量。
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

/// 单调运行时间与所有 CPU 已结算 idle 总账分别输出，idle 不钳制到运行时间。
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
    /// 不持源内容锁交付不可变文本，用户故障返回已交付的本次前缀。
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
    /// 本次打开的稳定快照允许显式字节位置，不能授予只读实例写权限。
    fn positioned_io(&self) -> bool {
        true
    }

    /// 顺序读取与复制描述符共享游标，实际交付后才推进位置。
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

    /// 显式位置读取只消费本次快照，不改变共享游标。
    fn read_at_into(
        &self,
        offset: u64,
        target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        require_read(self.flags())?;
        self.read_snapshot(offset, target)
    }

    /// 保留实际 proc 节点路径以支持属性和挂载寿命。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 动态文本快照没有普通文件页映射能力。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 独立打开各自保存状态，复制 OFD 共享原子字段。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Relaxed)
    }

    /// 只改变公共协议允许的状态，不改变只读访问模式。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        set_status_flags(&self.flags, raw)
    }

    /// 只原子修改本快照 OFD 的非阻塞位，不改变快照位置和追加状态。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        set_nonblocking(&self.flags, enabled)
    }

    /// 动态信息不能截断。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::ReadOnly)
    }

    /// proc 快照没有可提交的持久化操作。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 对本次文本长度计算非负 loff_t，失败不修改位置。
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

    /// inode 提供身份和权限，长度准确反映本 OFD 已生成的完整快照。
    fn stat(&self) -> Result<FileStat, FsError> {
        let mut stat = stat_path(&self.path)?;
        stat.size = self.bytes.len() as u64;
        stat.blocks = 0;
        Ok(stat)
    }

    /// 普通动态文本节点不提供目录枚举。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 快照 OFD 不能作为目录起点。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 只读 OFD 即使空写也返回描述符访问模式错误。
    fn write_from(
        &self,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }

    /// 按位置写同样不访问输入端。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::BadFd)
    }
}
