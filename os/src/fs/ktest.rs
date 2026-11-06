//! 文件系统子系统集成测试根模块（测试夹具 `Owner`、并发任务调度与内存/设备回收校验）。
//!
//! # 模块职责与架构定位
//! 本模块作为 `fs` 子系统的 `ktest` 顶层入口，负责：
//! 1. 聚合 ext4 磁盘格式解析、Inode/页缓存单飞并发、路径查找、文件描述符表、后台回写、
//!    管道/TTY/Epoll 事件源以及伪文件系统等全部子测试套件的编译期锚点（[`anchors`]）；
//! 2. 实现统一的设备与文件系统测试夹具 [`Owner`]，管理 VirtIO 块设备、完成工作队列（`WorkQueue`）、
//!    `InodeCache`、预分配并发工作线程以及两阶段内存泄漏基线比对。
//! - **上游调用者**：`ktest` 框架总表（通过 [`anchors`] 链接全部文件系统测试套件）。
//! - **下游子模块**：`concurrency`、`epoll`、`file`、`format`、`mem_mapping`、`path`、`pipe`、
//!   `poll_source`、`pseudo`、`system`、`tty`、`user`。
//!
//! # 核心设计与两阶段资源基线校验生命周期
//! 每个基于 [`Owner`] 的测试用例遵循严格的准备、执行、静止化（Quiesce）与销毁（Teardown）生命周期，
//! 在两个关键边界上校验内存分配器的统计数据（`MemoryStats`）完全恢复到基线：
//!
//! ```text
//! [Suite 级: DeviceInventory::set_up]
//!    |-- 扫描 PCI/MMIO 总线发现 VirtIO 块设备，等待全局常驻工作队列进入空闲阻塞态
//!    v
//! [Case 级: Owner::prepare]
//!    |-- 1. settled() 记录第一层内存基线: `initial` (设备准备前)
//!    |-- 2. 准备 VirtioDevice、独占 WorkQueue 与 Disk 实例
//!    |-- 3. settled() 记录第二层内存基线: `filesystem_baseline` (设备就绪、FS 对象创建前)
//!    |-- 4. 创建 InodeCache、Observation 控制块与 3 个预分配内核线程，启动块设备队列
//!    v
//! [Case 正文执行 & 并发 Job 派发]
//!    v
//! [Case 级: Owner::quiesce]
//!    |-- Step 1 (quiesce_jobs): 置位 cancelled、唤醒门控、恢复设备完成回调、等待 3 个工作线程退出内核栈，
//!    |                          清空任务槽并释放 InodeCache 强引用，建立 InodeCacheWatch
//!    |-- Step 2 (quiesce_devices): 等待所有 Inode 异步回收完毕 (wait_retired)，
//!                                  校验内存恢复至 `filesystem_baseline`，随后停止 Disk 与 WorkQueue
//!    v
//! [Case 级: Owner::tear_down]
//!    |-- 析构 Owner 释放 Disk 与 WorkQueue，校验内存完全恢复至 `initial` 基线
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`Observation`] 中的 `tasks`、`jobs`、`results` 三把自旋锁（`SpinLock`）彼此独立，从不嵌套获取；
//!   严禁在持有上述自旋锁期间睡眠或等待 [`Observation::gate`]。
//! - [`fill_file_page`] 要求调用方已持有目标 Inode 的 `InvalidateGuard` 共享锁，防止页加载中途与截断/失效操作发生竞争。

use ::ktest::{self, Fixture, Resource, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::block::{self, AccessMode, BlockError, BlockView, Disk, DiskIdentity, DiskRef};
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::DeviceInventory;
use crate::fs::ext4::{codec, Ext4Volume};
use crate::fs::inode::{InodeCache, InodeCacheWatch, InodeRef, InvalidateGuard};
use crate::fs::{
    DentryRef, DescriptorFlags, FileRef, FileTable, FsContext, FsError, InodeKind, OpenOptions,
    Path, ResolveMode, Vfs,
};
use crate::mm::{FolioRead, FolioRef};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, JoinHandle, PreparedTask, State, Task};
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, drivers, mm, sched, time, timer};

mod concurrency;
mod epoll;
mod file;
mod format;
mod mem_mapping;
mod path;
mod pipe;
mod poll_source;
mod pseudo;
mod system;
pub(crate) mod tty;
mod user;

/// 测试镜像中 `pattern` 文件的标准字节大小（65573 字节，跨越多个 4 KiB 页且末尾非整页对齐）。
const PATTERN_SIZE: u64 = 65573;
/// 测试镜像中大目录包含的目录项数量（12000 项，用于验证 HTree/DX 索引目录查找与遍历）。
const DIRECTORY_ENTRIES: usize = 12000;
/// 测试镜像中深层 Extent 树文件包含的盘区数量（1500 个，用于验证多级 Extent 树解析）。
const DEEP_EXTENTS: u64 = 1500;

/// 汇总并返回文件系统各子测试套件的编译期注册槽位引用，防止链接器丢弃测试段。
pub(crate) fn anchors() -> (
    [&'static ::ktest::SuiteSlot; 31],
    [&'static ::ktest::SuiteSlot; 7],
    [&'static ::ktest::SuiteSlot; 9],
    [&'static ::ktest::SuiteSlot; 6],
) {
    (
        [
            format::codec::__ktest_anchor(),
            format::volume::__ktest_anchor(),
            format::identity::__ktest_anchor(),
            format::bounded::__ktest_anchor(),
            format::oom::__ktest_anchor(),
            format::bad_super::__ktest_anchor(),
            format::bad_group::__ktest_anchor(),
            format::bad_inode::__ktest_anchor(),
            format::bad_extent::__ktest_anchor(),
            format::bad_directory::__ktest_anchor(),
            format::bad_dx::__ktest_anchor(),
            format::bad_directory_layout::__ktest_anchor(),
            format::bad_dx_layout::__ktest_anchor(),
            format::bad_geometry::__ktest_anchor(),
            format::bad_features::__ktest_anchor(),
            format::bad_mapping::__ktest_anchor(),
            format::prefix::__ktest_anchor(),
            concurrency::inode_loading::__ktest_anchor(),
            concurrency::file_loading::__ktest_anchor(),
            concurrency::failed_loading::__ktest_anchor(),
            user::anchor(),
            user::write_anchor(),
            user::namespace_anchor(),
            user::readahead_anchor(),
            user::background_anchor(),
            poll_source::source::__ktest_anchor(),
            pseudo::anchors()[0],
            mem_mapping::anchors()[0],
            pipe::anchor(),
            tty::anchor(),
            epoll::anchor(),
        ],
        path::anchors(),
        file::anchors(),
        system::anchors(),
    )
}

/// 在 10 秒超时上限内以 1 毫秒间隔轮询条件闭包 `ready`，若超时仍未满足则返回带 `label` 的测试错误。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(10))
        .ok_or(TestError("filesystem observation deadline overflow"))?;

    while !ready() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }

    Ok(())
}

/// 等待除当前 CPU 外的所有其他 CPU 进入空闲态，并冲刷、收缩 SLUB 与每 CPU 页缓存（PCP），返回整理后的内存统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("filesystem resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;

    mm::flush_all();
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 整理全局内存分配器缓存后，校验当前内存统计是否与测试开始前记录的 `before` 基线完全一致，确保无物理页或堆对象泄漏。
fn recovered(
    context: &mut TestContext<'_>,
    label: &'static str,
    before: mm::MemoryStats,
) -> TestResult {
    let after = settled()?;
    context.diagnostic(format_args!(
        "fs-resources boundary={} six-domains-before={:?} six-domains-after={:?}",
        label, before, after
    ));
    if after != before {
        return Err(TestError(label));
    }
    context.expect_eq(after, before);
    Ok(())
}

/// 校验切片 `bytes` 中的每个字节是否严格匹配文件逻辑偏移 `offset` 处的确定性测试图样公式。
fn patterned(bytes: &[u8], offset: u64) -> bool {
    bytes.iter().enumerate().all(|(index, byte)| {
        let at = offset + index as u64;
        *byte
            == 37u8
                .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                .wrapping_add((at % 512) as u8)
    })
}

/// 将文件系统错误 `FsError` 转换并输出到测试诊断日志中，再包装为统一的 `TestError`。
fn checked<T>(context: &TestContext<'_>, result: Result<T, FsError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("fs-operation error={:?}", error));
        TestError("filesystem operation failed")
    })
}

/// 在持有 Inode 失效保护锁（`InvalidateGuard`）的前提下，解析文件逻辑页号 `index` 的块映射并从磁盘卷异步填充该文件页。
fn fill_file_page(
    inode: &InodeRef,
    index: u64,
    read: FolioRead,
    _invalidate: &InvalidateGuard<'_>,
) -> Result<(), FsError> {
    match crate::fs::writeback::prepare_file_targets(inode, index, &(0..u64::MAX), None) {
        Ok((size, targets)) => inode.disk_volume()?.fill_page(index, size, &targets, read),
        Err(error) => Err(crate::fs::ext4::finish_error(read, error)),
    }
}

impl Fixture for DeviceInventory {
    /// 扫描总线发现全部设备，并等待全局常驻工作队列服务进入空闲阻塞态。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let inventory = drivers::discover().map_err(|error| {
            context.diagnostic(format_args!("fs-discovery error={:?}", error));
            TestError("filesystem discovery failed")
        })?;
        crate::workqueue::testing::wait_services_blocked();
        Ok(inventory)
    }
}

/// 从设备清单 `inventory` 中筛选 VirtIO 块设备（MMIO 与 PCI），校验设备总数等于测试用例声明的 `count`，并初始化每个块设备的传输层。
fn devices(
    context: &TestContext<'_>,
    inventory: &DeviceInventory,
    count: usize,
) -> Result<Vec<VirtioDevice>, TestError> {
    let pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    if inventory.mmio_blocks.len() + pci.clone().count() != count {
        return Err(TestError(
            "filesystem attached disks differ from declared roles",
        ));
    }

    // Step 1: 预留设备向量容量，依次初始化所有 MMIO 传输层的 VirtIO 块设备。
    let mut prepared = Vec::new();
    prepared
        .try_reserve_exact(count)
        .map_err(|_| TestError("filesystem device slots OOM"))?;
    for candidate in &inventory.mmio_blocks {
        prepared.push(
            // SAFETY:
            // 1. `ktest` 框架串行执行各个测试用例，前一个用例已在 `quiesce`/`tear_down` 中完成设备停止、硬件复位、中断注销与工作队列排空。
            // 2. `inventory` 由外层测试套件在整个套件周期内保持存活，当前用例独占该 MMIO 候选设备的寄存器映射与控制页。
            unsafe { VirtioDevice::prepare_mmio(candidate) }.map_err(|error| {
                context.diagnostic(format_args!("fs-MMIO-prepare error={:?}", error));
                TestError("filesystem MMIO preparation failed")
            })?,
        );
    }

    // Step 2: 依次初始化所有 PCI 传输层的 VirtIO 块设备。
    for device in pci {
        prepared.push(
            // SAFETY:
            // 1. `device` 来自 `inventory.pci` 已枚举的真实 PCI 功能，且前序用例已通过 `Disk::stop` 归还硬件访问权。
            // 2. 当前用例独占该 PCI 设备的 BAR 映射与配置空间访问，直到本用例在 `quiesce_devices` 中停止设备并排空中断和工作队列。
            unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }.map_err(|error| {
                context.diagnostic(format_args!("fs-PCI-prepare error={:?}", error));
                TestError("filesystem PCI preparation failed")
            })?,
        );
    }
    Ok(prepared)
}

/// 派发给预分配内核测试线程执行的异步文件系统测试任务类型。
enum Job {
    /// 通过 `InodeCache::iget` 加载或获取指定编号的 Inode。
    Get {
        /// 目标 Inode 缓存实例。
        cache: Arc<InodeCache>,
        /// 所属的 ext4 磁盘卷。
        volume: Arc<Ext4Volume>,
        /// 待获取的 Inode 编号。
        number: u64,
    },
    /// 通过 `Inode::read_at` 读取指定逻辑页号的数据并校验页缓存状态。
    Read {
        /// 目标文件 Inode。
        inode: InodeRef,
        /// 待读取的文件逻辑页索引（以 `PAGE_SIZE` 为单位）。
        page: u64,
    },
    /// 手动准备并发布文件页到 `FileMapping`，可在发盘前受控暂停以配合并发单飞测试。
    Fill {
        /// 目标文件 Inode。
        inode: InodeRef,
        /// 待填充的文件逻辑页索引。
        page: u64,
    },
    /// 在堆内存耗尽（OOM）压力下验证已缓存 Inode/文件页命中不受影响且未缓存加载返回 `NoMemory`。
    Oom {
        /// 目标 Inode 缓存实例。
        cache: Arc<InodeCache>,
        /// 所属的 ext4 磁盘卷。
        volume: Arc<Ext4Volume>,
        /// 已预先加载到缓存中的热 Inode。
        inode: InodeRef,
        /// 尚未加载到缓存中的冷 Inode 编号。
        empty_number: u64,
        /// 堆内存压力控制器。
        pressure: crate::cache::ktest::HeapPressure,
    },
    /// 在目录项缓存（`DentryCache`）中查找指定父目录下的单层子项名称。
    Lookup {
        /// 所属虚拟文件系统实例。
        vfs: Arc<Vfs>,
        /// 父目录项引用。
        parent: DentryRef,
        /// 待查找的子项字节名称。
        name: &'static [u8],
    },
    /// 通过 `Vfs::resolve` 解析完整路径名。
    Resolve {
        /// 所属虚拟文件系统实例。
        vfs: Arc<Vfs>,
        /// 路径解析上下文（含根目录与当前工作目录）。
        context: FsContext,
        /// 待解析的路径字节串。
        pathname: &'static [u8],
        /// 路径解析模式。
        mode: ResolveMode,
    },
    /// 在堆内存耗尽（OOM）压力下验证目录项缓存对已缓存项的命中与未缓存项的优雅失败。
    LookupOom {
        /// 所属虚拟文件系统实例。
        vfs: Arc<Vfs>,
        /// 父目录项引用。
        parent: DentryRef,
        /// 已预先缓存在 dcache 中的目录项。
        cached: DentryRef,
        /// 尚未缓存的冷目录项名称。
        name: &'static [u8],
        /// 堆内存压力控制器。
        pressure: crate::cache::ktest::HeapPressure,
    },
    /// 通过文件描述符表读取指定描述符的数据并校验内容图样。
    FileRead {
        /// 目标文件描述符表。
        table: Arc<FileTable>,
        /// 待读取的文件描述符编号。
        fd: usize,
        /// 请求读取的字节长度。
        len: usize,
        /// 预期此次读取对应的文件逻辑起始偏移（用于校验图样）。
        expected_offset: u64,
    },
    /// 在文件描述符表中将源描述符精确复制到目标描述符槽位（`dup_exact`）。
    FileDupExact {
        /// 目标文件描述符表。
        table: Arc<FileTable>,
        /// 源文件描述符编号。
        source: usize,
        /// 目标文件描述符编号。
        target: usize,
    },
    /// 在堆内存耗尽（OOM）压力下验证文件描述符表的槽位复用、扩容失败回滚与已打开文件读取。
    FileOom {
        /// 主测试文件描述符表（存在内联空闲槽位）。
        table: Arc<FileTable>,
        /// 所属虚拟文件系统实例。
        vfs: Arc<Vfs>,
        /// 路径解析上下文。
        context: FsContext,
        /// 已打开的文件描述符编号。
        fd: usize,
        /// 已打开的底层文件对象引用。
        cached: FileRef,
        /// 用于测试 OOM 下新打开失败的文件路径。
        pathname: &'static [u8],
        /// 堆内存压力控制器。
        pressure: crate::cache::ktest::HeapPressure,
    },
    /// 等待门控放行后，向同一打开文件连续追加写入多条固定长度记录。
    Append {
        /// 目标打开文件引用（以追加模式打开）。
        file: FileRef,
        /// 启动同步门控。
        gate: Arc<Completion>,
        /// 该工作线程专属的字节特征标记。
        marker: u8,
        /// 计划追加写入的 512 字节记录条数。
        records: usize,
    },
    /// 在堆内存耗尽（OOM）压力下验证对已缓存脏页的覆盖写与对未缓存冷页的分配写行为。
    WriteOom {
        /// 目标可写文件引用。
        file: FileRef,
        /// 堆内存压力控制器。
        pressure: crate::cache::ktest::HeapPressure,
    },
    /// 运行绑定了文件内存映射（`mmap`）的用户态测试线程并执行指定方向的访存。
    User {
        /// 目标用户态线程。
        thread: Arc<crate::task::UserThread>,
        /// 用户态访存方向（读/写/执行）。
        access: crate::arch::UserAccess,
    },
}

/// 异步测试任务 [`Job`] 执行完毕后写入结果槽的返回值枚举。
enum Returned {
    /// 追加写入任务成功写入的总字节数。
    Written(usize),
    /// 写路径 OOM 测试结果。
    WriteOom {
        /// 堆耗尽证据 `(是否耗尽本地堆, 是否验证分配失败)`。
        evidence: (bool, bool),
        /// 对已驻留热页写入的结果。
        warm: Result<usize, FsError>,
        /// 对未驻留冷页写入的结果。
        cold: Result<usize, FsError>,
    },
    /// `Job::Get` 获取到的 Inode 引用。
    Inode(InodeRef),
    /// `Job::Lookup` 获取到的目录项引用。
    Dentry(DentryRef),
    /// `Job::Resolve` 解析得到的路径对象。
    Path(Path),
    /// `Job::Read` 或 `Job::Fill` 读取并锁定的文件页及校验结果。
    Page {
        /// 文件页缓存中的物理页（Folio）引用。
        folio: FolioRef,
        /// 实际读取的字节数。
        bytes: usize,
        /// 页内数据是否通过确定性图样校验。
        valid: bool,
    },
    /// Inode 与文件页缓存 OOM 测试结果。
    Oom {
        /// 执行该任务的 CPU 编号。
        producer_cpu: usize,
        /// 堆耗尽证据。
        evidence: (bool, bool),
        /// 再次获取热 Inode 是否返回同一 `InodeRef` 实例。
        same_inode: bool,
        /// 再次获取热 Inode 是否共享同一 `FileMapping` 地址。
        same_mapping: bool,
        /// 从热页读取到的字节数。
        bytes: usize,
        /// 热页数据是否匹配预期图样。
        valid: bool,
        /// OOM 下加载冷 Inode 返回的错误。
        inode_error: Option<FsError>,
        /// OOM 下读取冷文件页返回的错误。
        page_error: Option<FsError>,
        /// 热文件页的物理地址。
        address: usize,
        /// 测试前块设备累计提交请求数。
        submitted_before: usize,
        /// 测试后块设备累计提交请求数。
        submitted_after: usize,
    },
    /// 目录项缓存 OOM 测试结果。
    LookupOom {
        /// 执行该任务的 CPU 编号。
        producer_cpu: usize,
        /// 堆耗尽证据。
        evidence: (bool, bool),
        /// OOM 下查找已缓存目录项是否返回同一 `DentryRef` 实例。
        same_dentry: bool,
        /// OOM 下查找未缓存目录项返回的错误。
        error: Option<FsError>,
        /// OOM 查找前 dcache 中的条目数。
        len_before: usize,
        /// OOM 查找后 dcache 中的条目数。
        len_after: usize,
        /// 测试前块设备累计提交请求数。
        submitted_before: usize,
        /// 测试后块设备累计提交请求数。
        submitted_after: usize,
    },
    /// `Job::FileRead` 通过文件描述符读取的结果。
    FileRead {
        /// 实际读取的字节数。
        bytes: usize,
        /// 读取内容是否匹配预期图样。
        valid: bool,
    },
    /// `Job::FileDupExact` 复制得到的文件描述符编号。
    FileDescriptor(usize),
    /// 文件描述符表 OOM 测试结果。
    FileOom {
        /// 执行该任务的 CPU 编号。
        producer_cpu: usize,
        /// 堆耗尽证据。
        evidence: (bool, bool),
        /// 查找与复用槽位发布后的文件对象是否均指向原打开文件实例。
        same_ofd: bool,
        /// 从已打开文件读取的字节数。
        bytes: usize,
        /// 读取内容是否匹配预期图样。
        valid: bool,
        /// OOM 下调用 `open` 打开新文件返回的错误。
        new_error: Option<FsError>,
        /// OOM 下在现有容量内成功预留并发布的描述符编号。
        reserved_fd: usize,
        /// 关闭 `reserved_fd` 后再次预留复用的描述符编号。
        reused_fd: usize,
        /// 满载描述符表在 OOM 下尝试扩容（`reserve` 与 `dup_exact`）返回的错误数组。
        growth_errors: [Option<FsError>; 2],
        /// 释放内存压力后满载描述符表成功扩容分配到的描述符编号。
        growth_reused_fd: usize,
        /// 满载描述符表的初始内联槽位容量。
        growth_initial_slots: usize,
        /// 测试前块设备累计提交请求数。
        submitted_before: usize,
        /// 测试后块设备累计提交请求数。
        submitted_after: usize,
    },
    /// `Job::User` 用户态文件映射访存测试结果。
    User(Result<file::VmAccessResult, TestError>),
}

/// 跨任务并发观测与同步控制块，由主测试线程与最多 3 个并发工作线程共享。
struct Observation {
    /// 取消标志：置为 `true` 时通知所有工作线程尽快中止循环并退出。
    cancelled: AtomicBool,
    /// 同步门控：用于在单飞加载钩子或 `Job::Fill` 关键停点处暂停工作线程，待主线程完成并发观察后再放行。
    gate: Completion,
    /// 是否在 Inode 加载钩子或文件页 `Fill` 发布后阻塞首个加载者线程。
    hold_loader: AtomicBool,
    /// 已进入加载暂停点的加载者线程计数。
    loader_entered: AtomicUsize,
    /// 标记各个槽位的工作线程是否已取出 `Job` 并正式进入业务执行阶段。
    entered: [AtomicBool; 3],
    /// 各个槽位工作线程的内核 `Task` 句柄，由 `tasks` 自旋锁保护。
    tasks: SpinLock<[Option<Arc<Task>>; 3]>,
    /// 各个槽位待执行的 `Job` 槽位，由 `jobs` 自旋锁保护。
    jobs: SpinLock<[Option<Job>; 3]>,
    /// 各个槽位工作线程执行完毕后存放的返回值槽位，由 `results` 自旋锁保护。
    results: SpinLock<[Option<Result<Returned, FsError>>; 3]>,
}

impl Observation {
    /// 分配并初始化一个空的并发观测控制块。
    fn prepare() -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            cancelled: AtomicBool::new(false),
            gate: Completion::new(),
            hold_loader: AtomicBool::new(false),
            loader_entered: AtomicUsize::new(0),
            entered: [const { AtomicBool::new(false) }; 3],
            tasks: SpinLock::new([const { None }; 3]),
            jobs: SpinLock::new([const { None }; 3]),
            results: SpinLock::new([const { None }; 3]),
        })
        .map_err(|_| TestError("filesystem observation OOM"))
    }

    /// 判断槽位 `index` 的工作线程是否已进入业务函数且当前在调度器中处于 `Blocked` 睡眠态。
    fn blocked(&self, index: usize) -> bool {
        if !self.entered[index].load(Ordering::Acquire) {
            return false;
        }
        let task = self.tasks.lock()[index].clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

/// 执行单个异步测试任务 `Job`，根据任务变体分派到对应的文件系统或虚拟内存操作并返回 `Returned`。
fn perform(job: Job, observer: &Observation) -> Result<Returned, FsError> {
    let cancelled = &observer.cancelled;
    match job {
        Job::Append {
            file,
            gate,
            marker,
            records,
        } => {
            gate.wait();
            let mut bytes = [marker; 512];
            let mut accepted = 0;
            for index in 0..records {
                if cancelled.load(Ordering::Acquire) {
                    return Err(FsError::LoadAborted);
                }
                bytes[1] = index as u8;
                let count = file.write(&bytes)?;
                if count != bytes.len() {
                    return Err(FsError::Corrupt);
                }
                accepted += count;
            }
            Ok(Returned::Written(accepted))
        }
        Job::WriteOom { file, mut pressure } => {
            let evidence = {
                let _pin = crate::sync::PreemptGuard::new();
                pressure.exhaust()
            };
            let warm = file.write_at(0, &[0x6d; 17]);
            let cold = file.write_at(4096, &[0x9b; 17]);
            drop(pressure);
            Ok(Returned::WriteOom {
                evidence,
                warm,
                cold,
            })
        }
        Job::User { thread, access } => Ok(Returned::User(file::run_vm_access(
            thread, access, cancelled,
        ))),
        Job::FileRead {
            table,
            fd,
            len,
            expected_offset,
        } => {
            if mm::PAGE_SIZE < len {
                return Err(FsError::InvalidOperation);
            }
            let mut data = [0xa7; mm::PAGE_SIZE];
            let bytes = table.read(fd, &mut data[..len])?;
            Ok(Returned::FileRead {
                bytes,
                valid: patterned(&data[..bytes], expected_offset),
            })
        }
        Job::FileDupExact {
            table,
            source,
            target,
        } => table
            .dup_exact(source, target, 1024)
            .map(Returned::FileDescriptor),
        Job::FileOom {
            table,
            vfs,
            context,
            fd,
            cached,
            pathname,
            mut pressure,
        } => {
            let root = vfs.root();
            let disk = root.inode()?.disk_volume()?.disk();
            let submitted_before = disk.test_snapshot().submitted;
            let growth = FileTable::try_new()?;
            let base = growth
                .reserve(4097)?
                .publish(cached.clone(), DescriptorFlags::default());
            let initial_slots = growth.allocated_slots();
            for _ in 1..initial_slots {
                growth.dup(base, 4097)?;
            }

            let evidence = pressure.exhaust();
            let growth_errors = [
                growth.reserve(4097).err(),
                growth.dup_exact(base, 4096, 4097).err(),
            ];
            let same = table.lookup(fd)?;
            let mut data = [0xa7; mm::PAGE_SIZE];
            let bytes = same.read_at(0, &mut data)?;
            let reservation = table.reserve(1024)?;
            let reserved_fd = reservation.publish(cached.clone(), DescriptorFlags::default());
            let published = table.lookup(reserved_fd)?;
            let same_ofd = Arc::ptr_eq(&same, &cached) && Arc::ptr_eq(&published, &cached);
            table.close(reserved_fd)?;
            let new_error = table
                .open(&vfs, &context, pathname, OpenOptions::default(), 1024)
                .err();
            let reservation = table.reserve(1024)?;
            let reused_fd = reservation.fd();
            drop(reservation);

            drop(pressure);
            let growth_reused_fd = growth.reserve(4097)?.fd();
            let returned = Returned::FileOom {
                producer_cpu: cpu::current().id,
                evidence,
                same_ofd,
                bytes,
                valid: patterned(&data[..bytes], 0),
                new_error,
                reserved_fd,
                reused_fd,
                growth_errors,
                growth_reused_fd,
                growth_initial_slots: initial_slots,
                submitted_before,
                submitted_after: disk.test_snapshot().submitted,
            };
            Ok(returned)
        }
        Job::Lookup { vfs, parent, name } => vfs
            .dcache()
            .lookup(vfs.inodes(), &parent, name)
            .map(Returned::Dentry),
        Job::Resolve {
            vfs,
            context,
            pathname,
            mode,
        } => vfs.resolve(&context, pathname, mode).map(Returned::Path),
        Job::LookupOom {
            vfs,
            parent,
            cached,
            name,
            mut pressure,
        } => {
            let disk = parent.inode()?.disk_volume()?.disk();
            let submitted_before = disk.test_snapshot().submitted;
            let len_before = vfs.dcache().len();
            let evidence = pressure.exhaust();
            let same = vfs
                .dcache()
                .lookup(vfs.inodes(), &parent, cached.name().as_slice())?;
            let error = vfs.dcache().lookup(vfs.inodes(), &parent, name).err();
            let returned = Returned::LookupOom {
                producer_cpu: cpu::current().id,
                evidence,
                same_dentry: same.ptr_eq(&cached),
                error,
                len_before,
                len_after: vfs.dcache().len(),
                submitted_before,
                submitted_after: disk.test_snapshot().submitted,
            };
            drop(pressure);
            Ok(returned)
        }
        Job::Get {
            cache,
            volume,
            number,
        } => cache.iget(&volume, number).map(Returned::Inode),
        Job::Read { inode, page } => {
            let offset = page
                .checked_mul(mm::PAGE_SIZE as u64)
                .ok_or(FsError::Corrupt)?;
            let mut bytes = [0xa7; mm::PAGE_SIZE];
            let count = inode.read_at(offset, &mut bytes)?;
            let _invalidate = inode.lock_invalidate()?;
            let folio = inode
                .mapping()
                .cached_folio(page)
                .ok_or(FsError::Corrupt)?
                .with_file_owner(inode.clone());
            folio.wait_ready()?;
            Ok(Returned::Page {
                folio,
                bytes: count,
                valid: patterned(&bytes[..count], offset),
            })
        }
        Job::Fill { inode, page } => {
            // Step 1: 在不持有 Inode 失效锁的情况下预分配物理页与页缓存槽位。
            let mut attempt = mm::reclaim::ReclaimAttempt::new();
            let mut prepared = Some(inode.mapping().prepare_file(&inode, page, &mut attempt)?);
            let result = (|| {
                // Step 2: 获取 Inode 失效共享锁，将新页以未就绪（Locked）状态发布到 FileMapping。
                let invalidate = inode.lock_invalidate()?;
                if cancelled.load(Ordering::Acquire) {
                    return Err(FsError::LoadAborted);
                }
                let (folio, read) = inode
                    .mapping()
                    .publish_file(&inode, page, &mut prepared)?
                    .ok_or(FsError::Corrupt)?;
                let folio = folio.with_file_owner(inode.clone());
                let read = read.ok_or(FsError::Corrupt)?.with_file_owner(inode.clone());
                // Step 3: 若测试开启了 hold_loader，在发起块 I/O 前暂停，供主线程构造同页并发等待者。
                if observer.hold_loader.load(Ordering::Acquire) {
                    observer.loader_entered.fetch_add(1, Ordering::AcqRel);
                    observer.gate.wait();
                }
                if cancelled.load(Ordering::Acquire) {
                    return Err(crate::fs::ext4::finish_error(
                        read,
                        FsError::Io(BlockError::QueueStopped),
                    ));
                }

                // Step 4: 提交底层块读取请求，等待页面就绪并校验数据图样。
                fill_file_page(&inode, page, read, &invalidate)?;
                folio.wait_ready()?;
                let mut bytes = [0xa7; mm::PAGE_SIZE];
                folio.copy_to(0, &mut bytes)?;
                let valid = patterned(&bytes, page * mm::PAGE_SIZE as u64);
                Ok(Returned::Page {
                    folio,
                    bytes: mm::PAGE_SIZE,
                    valid,
                })
            })();

            drop(prepared);
            result
        }
        Job::Oom {
            cache,
            volume,
            inode,
            empty_number,
            mut pressure,
        } => {
            let submitted_before = volume.disk().test_snapshot().submitted;
            let evidence = pressure.exhaust();
            let same = cache.iget(&volume, inode.number())?;
            let mut data = [0xa7; mm::PAGE_SIZE];
            let bytes = same.read_at(0, &mut data)?;
            let valid = patterned(&data[..bytes], 0);
            let inode_error = cache.iget(&volume, empty_number).err();
            let page_error = inode.read_at(mm::PAGE_SIZE as u64, &mut data).err();
            let folio = inode
                .mapping()
                .cached_folio(0)
                .ok_or(FsError::Corrupt)?
                .with_file_owner(inode.clone());
            folio.wait_ready()?;
            let returned = Returned::Oom {
                producer_cpu: cpu::current().id,
                evidence,
                same_inode: same.ptr_eq(&inode),
                same_mapping: core::ptr::eq(same.mapping(), inode.mapping()),
                bytes,
                valid,
                inode_error,
                page_error,
                address: folio.phys_addr(),
                submitted_before,
                submitted_after: volume.disk().test_snapshot().submitted,
            };
            drop(pressure);
            Ok(returned)
        }
    }
}

/// 文件系统测试用例的核心资源拥有者与生命周期夹具（Fixture）。
struct Owner {
    /// 用例准备块设备之前记录的第一层全局内存统计基线。
    initial: mm::MemoryStats,
    /// 块设备与完成工作队列就绪后、创建文件系统缓存与任务前记录的第二层内存统计基线。
    filesystem_baseline: mm::MemoryStats,
    /// 当前用例独占准备的块设备实例列表（首元素为主根盘 `root`）。
    disks: Vec<DiskRef>,
    /// 供当前用例块设备投递 I/O 完成回调的专用工作队列。
    queue: WorkQueue,
    /// 当前用例使用的 Inode 缓存实例；在 `quiesce_jobs` 阶段被取出释放。
    cache: Option<Arc<InodeCache>>,
    /// `InodeCache` 释放后留下的退役观察哨，用于在 `quiesce_devices` 中等待所有 Inode 彻底销毁。
    cache_watch: Option<InodeCacheWatch>,
    /// 并发任务同步与结果收集控制块。
    observation: Option<Arc<Observation>>,
    /// 可选的堆内存压力控制器（仅当用例带有 `"fs-oom"` 标签时创建）。
    pressure: Option<crate::cache::ktest::HeapPressure>,
    /// 最多 3 个预先分配好内核栈但尚未发布到调度器的后台测试任务槽位。
    prepared: [Option<PreparedTask>; 3],
    /// 已通过 `publish` 启动的后台测试任务连接句柄列表。
    joins: Vec<JoinHandle>,
    /// 块设备启动阶段遇到的首个错误（若有），用于在清理前安全保留设备所有权。
    activation_error: Option<BlockError>,
}

impl Owner {
    /// 为单个测试用例准备块设备、完成工作队列、`InodeCache` 及 3 个预分配内核线程，并记录两层内存基线。
    fn prepare(
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        // Step 1: 采集设备准备前的初始内存基线，并根据用例声明的磁盘角色数量准备 VirtIO 设备与完成工作队列。
        let initial = settled()?;
        let count = [Resource::Root, Resource::Extra, Resource::Scratch]
            .iter()
            .filter(|role| context.case.attributes.resources.contains(*role))
            .count();
        let devices = devices(context, inventory, count)?;
        let max_tags = devices
            .iter()
            .map(|device| usize::from(device.info().tag_count))
            .max()
            .expect("filesystem attached disk count validated before preparation");
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: max_tags + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("filesystem completion domain OOM"))?;

        let mut disks = Vec::new();
        let disk_result = (|| {
            disks
                .try_reserve_exact(count)
                .map_err(|_| TestError("filesystem Disk slots OOM"))?;
            for device in devices {
                disks.push(
                    Disk::prepare(device, &queue)
                        .map_err(|_| TestError("filesystem Disk preparation failed"))?,
                );
            }
            Ok(())
        })();
        if let Err(error) = disk_result {
            for disk in &disks {
                disk.stop()
                    .map_err(|_| TestError("unpublished filesystem Disk stop failed"))?;
            }
            queue.close_sync();
            return Err(error);
        }

        // Step 2: 采集设备就绪后的文件系统内存基线，并创建 InodeCache、Observation 与 3 个预分配内核任务。
        let resources = (|| {
            let filesystem_baseline = settled()?;
            let pressure = if context.case.attributes.tags.contains(&"fs-oom") {
                Some(crate::cache::ktest::HeapPressure::prepare()?)
            } else {
                None
            };
            let inode_capacity = if context
                .case
                .attributes
                .tags
                .contains(&"fs-path-cache-budget")
            {
                2
            } else if context.case.attributes.tags.contains(&"fs-path")
                || context.case.attributes.tags.contains(&"fs-system")
            {
                64
            } else {
                8
            };
            let cache = Arc::try_new(
                InodeCache::try_new(inode_capacity)
                    .map_err(|_| TestError("inode cache preparation failed"))?,
            )
            .map_err(|_| TestError("inode cache owner OOM"))?;
            let observation = Observation::prepare()?;
            let loader = observation.clone();
            let hook: Arc<dyn Fn(&InodeRef) + Send + Sync> = Arc::try_new(move |_: &InodeRef| {
                if loader.hold_loader.load(Ordering::Acquire) {
                    loader.loader_entered.fetch_add(1, Ordering::AcqRel);
                    loader.gate.wait();
                }
            })
            .map_err(|_| TestError("inode load observer OOM"))?;
            cache.ktest_set_load_hook(Some(hook));

            let mut prepared = [const { None }; 3];
            let mut joins = Vec::new();
            joins
                .try_reserve_exact(3)
                .map_err(|_| TestError("filesystem native join slots OOM"))?;
            for (index, slot) in prepared.iter_mut().enumerate() {
                let observer = observation.clone();
                *slot = Some(
                    task::prepare_kernel_on(index % cpu::count(), move || {
                        observer.tasks.lock()[index] = Some(task::current_task());
                        let job = observer.jobs.lock()[index].take();
                        if observer.cancelled.load(Ordering::Acquire) {
                            drop(job);
                            return 0;
                        }
                        observer.entered[index].store(true, Ordering::Release);
                        let Some(job) = job else {
                            return 1;
                        };
                        let result = perform(job, &observer);
                        observer.results.lock()[index] = Some(result);
                        0
                    })
                    .map_err(|_| TestError("filesystem native task preparation failed"))?,
                );
            }
            Ok((
                filesystem_baseline,
                cache,
                observation,
                pressure,
                prepared,
                joins,
            ))
        })();
        let (filesystem_baseline, cache, observation, pressure, prepared, joins) = match resources {
            Ok(resources) => resources,
            Err(error) => {
                for disk in &disks {
                    disk.stop()
                        .map_err(|_| TestError("unpublished filesystem resources stop failed"))?;
                }
                queue.close_sync();
                return Err(error);
            }
        };

        // Step 3: 激活全部块设备队列并返回构造完毕的夹具实例。
        let activation_error = disks.iter().find_map(|disk| disk.start().err());
        Ok(Self {
            initial,
            filesystem_baseline,
            disks,
            queue,
            cache: Some(cache),
            cache_watch: None,
            observation: Some(observation),
            pressure,
            prepared,
            joins,
            activation_error,
        })
    }

    /// 获取当前用例的主块设备（`root` 盘）引用；若设备激活失败则返回错误。
    fn disk(&self) -> Result<&DiskRef, TestError> {
        if self.activation_error.is_some() {
            return Err(TestError("filesystem Disk activation failed"));
        }
        self.disks
            .first()
            .ok_or(TestError("filesystem root Disk missing"))
    }

    /// 获取当前用例的 `InodeCache` 共享引用。
    fn cache(&self) -> &Arc<InodeCache> {
        self.cache
            .as_ref()
            .expect("filesystem cache consumed before body")
    }

    /// 获取当前用例的并发观测控制块 `Observation` 共享引用。
    fn observer(&self) -> &Arc<Observation> {
        self.observation
            .as_ref()
            .expect("filesystem observation consumed before body")
    }

    /// 在主块设备上以只读模式打开 `Ext4Volume`（元数据缓存容量设为 64 块）。
    fn open(&self, context: &TestContext<'_>) -> Result<Arc<Ext4Volume>, TestError> {
        checked(
            context,
            Ext4Volume::open(
                BlockView::whole(self.disk()?.clone()),
                AccessMode::ReadOnly,
                64,
            ),
        )
    }

    /// 加载卷的根目录 Inode 并在其中按字节名 `name` 查找并返回目标子项的 `InodeRef`。
    fn file(
        &self,
        context: &TestContext<'_>,
        volume: &Arc<Ext4Volume>,
        name: &[u8],
    ) -> Result<InodeRef, TestError> {
        let root = checked(context, self.cache().iget(volume, codec::ROOT_INODE))?;
        checked(context, root.lookup(self.cache(), name))
    }

    /// 将任务 `job` 放入槽位 `index`，并将对应的预分配内核线程发布到调度器开始执行。
    fn publish(&mut self, index: usize, job: Job) {
        self.observer().jobs.lock()[index] = Some(job);
        self.joins.push(
            self.prepared[index]
                .take()
                .expect("filesystem native task already published")
                .publish(),
        );
    }

    /// 等待所有已发布的后台测试线程退出内核栈，并回收校验其退出码均为 0。
    fn join_tasks(&mut self) -> TestResult {
        for join in &self.joins {
            until("filesystem producer did not leave original stack", || {
                join.try_join().is_some()
            })?;
        }

        for join in self.joins.drain(..) {
            if join.join() != 0 {
                return Err(TestError("filesystem native producer failed"));
            }
        }
        Ok(())
    }

    /// 第一阶段静止化：通知并等待所有后台测试任务退出，排空块设备完成回调，清空任务结果槽并释放 `InodeCache` 主引用。
    fn quiesce_jobs(&mut self) -> TestResult {
        let observer = self.observer().clone();
        observer.cancelled.store(true, Ordering::Release);
        observer.hold_loader.store(false, Ordering::Release);
        observer.gate.complete();
        drop(core::mem::take(&mut self.prepared));
        drop(self.pressure.take());
        for disk in &self.disks {
            disk.test_resume_completions();
        }
        self.join_tasks()?;

        for disk in &self.disks {
            disk.test_flush_completions();
        }
        let jobs = core::mem::replace(&mut *observer.jobs.lock(), [const { None }; 3]);
        let results = core::mem::replace(&mut *observer.results.lock(), [const { None }; 3]);
        let tasks = core::mem::replace(&mut *observer.tasks.lock(), [const { None }; 3]);
        drop(jobs);
        drop(results);
        drop(tasks);

        if let Some(cache) = self.cache.take() {
            cache.ktest_set_load_hook(None);
            assert!(self.cache_watch.is_none());
            self.cache_watch = Some(cache.watch());
            drop(cache);
        }
        drop(self.observation.take());
        drop(observer);
        drop(core::mem::take(&mut self.joins));
        Ok(())
    }

    /// 第二阶段静止化：等待 `InodeCache` 中所有 Inode 彻底退役，校验文件系统层内存恢复至 `filesystem_baseline`，最后停止块设备与工作队列。
    fn quiesce_devices(&mut self, context: &mut TestContext<'_>) -> TestResult {
        if let Some(watch) = self.cache_watch.as_ref() {
            let result = watch.wait_retired();
            if result == Err(FsError::Io(BlockError::StopUnconfirmed)) {
                return Err(TestError(
                    "canonical inode stop unconfirmed; retaining original owners",
                ));
            }
            checked(context, result)?;
            drop(self.cache_watch.take());
        }

        recovered(
            context,
            "filesystem-before-disk-stop",
            self.filesystem_baseline,
        )?;

        for disk in &self.disks {
            disk.stop().map_err(|_| {
                TestError("filesystem stop cannot converge; retaining original DMA owner")
            })?;
            context.expect_eq(disk.test_snapshot().inflight, 0);
        }

        self.queue.close_sync();
        Ok(())
    }

    /// 依次执行后台任务静止化与块设备静止化。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.quiesce_jobs()?;
        self.quiesce_devices(context)
    }

    /// 析构夹具释放全部块设备与工作队列，并校验全局内存恢复至用例开始前的 `initial` 基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let initial = self.initial;
        drop(self);
        recovered(context, "filesystem-after-disk-drop", initial)
    }
}

impl Fixture<DeviceInventory> for Owner {
    /// 初始化当前测试用例的 `Owner` 夹具。
    fn set_up(
        context: &mut TestContext<'_>,
        inventory: &DeviceInventory,
    ) -> Result<Self, TestError> {
        Self::prepare(context, inventory)
    }

    /// 在测试用例结束或中止时停止后台任务与块设备，并校验文件系统层内存无泄漏。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        Owner::quiesce(self, context)
    }

    /// 销毁夹具并校验块设备与工作队列层内存完全归还至初始基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        Owner::tear_down(self, context)
    }
}
