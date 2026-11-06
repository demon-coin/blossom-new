//! 页缓存（`FolioRef` / `FolioIndex`）与块设备元数据缓存（`MetadataCache` / `BufferRef`）内核回归测试套件。
//!
//! # 模块职责与架构定位
//! 本模块作为 `cache` 子系统的 `ktest` 集成测试模块，结合底层 VirtIO 块设备驱动（`VirtioDevice` / `Disk`）、
//! 物理页分配器（`mm`）、多核任务调度（`task` / `sched`）以及 ext4 元数据编解码器（`fs::ext4::codec`），
//! 对页缓存与元数据缓存的核心并发语义、生命周期与异常边界进行系统性验证：
//! 1. **物理页包装与稀疏索引（`folio`）**：验证 `FolioRef` 的 0 阶单页约束、引用计数与物理页释放归还，以及 `FolioIndex` AVL 稀疏索引在顺序/逆序/大跨度键下的插入、OOM 命中保活与按引用计数修剪（Prune）。
//! 2. **多粒度元数据子页视图（`metadata_views` / `mutable_metadata` / `metadata_records`）**：验证 1KiB / 2KiB / 4KiB 块视图（`BufferRef`）的页内偏移映射、跨盘隔离、扇区级按需补齐（保留已有脏扇区不被后续读覆盖）以及同页多条 ext4 Inode 记录的并发互斥修改与 CRC32C 校验。
//! 3. **单飞合并加载与多键并发（`metadata_singleflight` / `file_singleflight` / `metadata_parallel` / `file_parallel`）**：验证相同键并发缺失（Miss）时仅发起单次块设备 I/O 请求、等待者进入阻塞态（`State::Blocked`）并复用同一物理页，以及不同键并发缺失时双请求同时在途（Inflight）。
//! 4. **队列停止、准备阶段 OOM 与容量上限（`stopped_failure` / `prepare_oom` / `bounded` / `oom`）**：验证块队列停止（`QueueStopped`）或 Bio 准备阶段内存耗尽（`NoMemory`）对同页等待者的唤醒与错误传播，以及固定容量元数据缓存淘汰与物理页耗尽下的缓存命中保证。
//!
//! # 核心状态机：Folio 单飞（Singleflight）加载与并发修改契约
//! ```text
//!   FolioIndex::get_or_prepare(key) / MetadataCache::get(block, size)
//!           │
//!           ├──► [首次 Miss：分配 Folio 占位页]
//!           │         │
//!           │         ├── 返回 (FolioRef, Some(FolioRead)) ──► 唯一加载者调用 Disk::load_folio()
//!           │         │                                              │
//!           │         │         ┌── 成功：标记扇区就绪，唤醒全部同页等待者
//!           │         │         └── 失败（QueueStopped / NoMemory）：记录错误码并标记完结，唤醒全部等待者返回同一错误
//!           │         │
//!           └──► [并发同键命中未就绪页]
//!                     │
//!                     └── 返回 (FolioRef, None) ──► 调用 FolioRef::wait_ready() 进入 State::Blocked
//!
//!   同页并发修改互斥（MetadataRecords）：
//!   BufferRef::update() ──► 获取 Folio 内容睡眠锁（Folio::lock_content）──► 执行闭包修改并标记脏扇区 ──► 释放内容锁
//! ```
//!
//! # 锁获取顺序与资源回收契约
//! - **观察状态自旋锁与 Folio 内容锁顺序**：测试夹具中的 `Observation::results` / `tasks` / `read` / `pressure` 均为短临界区 `SpinLock`；在读取或修改 Folio 实际内容（会获取 `Folio` 内部睡眠内容锁）之前，必须先释放 `Observation` 自旋锁（参见 `Owner::take_results`），严禁在持有自旋锁时获取睡眠锁或触发阻塞。
//! - **生命周期与六维内存基线核对**：每个测试用例结束时先执行 `quiesce`（取消未发布任务、唤醒门闩、恢复并排空完成队列、等待全部工作任务退出内核栈、按需丢弃私有脏元数据、停止块设备并关闭工作队列），随后在 `tear_down` 中销毁夹具并通过 `recovered` 校验伙伴系统、PCP 与 SLUB 六维内存统计完全回到初始基线。

use ::ktest::{self, Fixture, TestContext, TestError, TestResult};
use alloc::alloc::{alloc, dealloc};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::Layout;
use core::hint::black_box;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::block::{
    self, AccessMode, BioAllocator, BlockError, BlockView, Disk, DiskRef, IoResult,
};
use crate::cache::{BufferRef, FolioIndex, MetadataCache};
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VIRTIO_PCI_BLOCK_LEGACY, VIRTIO_PCI_BLOCK_MODERN,
    VIRTIO_VENDOR_ID,
};
use crate::drivers::DeviceInventory;
use crate::fs::ext4::{codec, Ext4Volume};
use crate::fs::{FsError, InodeMetadata};
use crate::mm::folio::{FolioRead, FolioRef};
use crate::sync::{Completion, SpinLock};
use crate::task::{self, JoinHandle, PreparedTask, State, Task};
use crate::workqueue::{WorkQueue, WorkQueueConfig};
use crate::{cpu, mm, sched, time, timer};

/// 文件页与元数据测试默认使用的设备字节偏移（第 64 页，即 256KiB 处）。
const OFFSET: u64 = 64 * mm::PAGE_SIZE as u64;
/// 内存压力测试中暂存 0 阶物理页句柄的向量预留容量上限。
const PRESSURE_CAPACITY: usize = 65536;

/// 返回本模块全部 12 个缓存测试套件的注册槽数组，供父级 `cache` 模块统一导出。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 12] {
    [
        folio::__ktest_anchor(),
        metadata_views::__ktest_anchor(),
        metadata_singleflight::__ktest_anchor(),
        file_singleflight::__ktest_anchor(),
        metadata_parallel::__ktest_anchor(),
        file_parallel::__ktest_anchor(),
        stopped_failure::__ktest_anchor(),
        bounded::__ktest_anchor(),
        oom::__ktest_anchor(),
        prepare_oom::__ktest_anchor(),
        mutable_metadata::__ktest_anchor(),
        metadata_records::__ktest_anchor(),
    ]
}

/// 在 5 秒超时上限内以 1 毫秒间隔轮询条件闭包 `ready`，条件满足返回 `Ok(())`，超时则返回以 `label` 命名的测试错误。
fn until(label: &'static str, mut ready: impl FnMut() -> bool) -> TestResult {
    let deadline = time::now()
        .checked_add(time::Duration::from_secs(5))
        .ok_or(TestError("cache deadline overflow"))?;
    while !ready() {
        if deadline <= time::now() {
            return Err(TestError(label));
        }
        timer::sleep(time::Duration::from_millis(1));
    }
    Ok(())
}

/// 等待除当前核心外的所有 CPU 进入空闲态后，冲刷全部 CPU 的本地缓存、收缩 SLUB 空闲 slab 并排空每 CPU 页面缓存（PCP），返回当前物理内存六维统计快照。
fn settled() -> Result<mm::MemoryStats, TestError> {
    until("cache resource CPUs did not settle", || {
        (0..cpu::count())
            .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
    })?;
    mm::flush_all();
    mm::shrink_all();
    mm::drain_all_pages();
    Ok(mm::stats())
}

/// 等待系统后台状态静默并回收缓存页后，断言当前物理内存六维统计与测试开始前的 `baseline` 完全一致，并输出诊断日志。
fn recovered(context: &mut TestContext<'_>, baseline: mm::MemoryStats) -> TestResult {
    let after = settled()?;
    context.expect_eq(after, baseline);
    context.diagnostic(format_args!(
        "cache-resources six-domains-before={:?} six-domains-after={:?} real-stop-join-cleanup=1",
        baseline, after
    ));
    Ok(())
}

/// 校验切片 `bytes` 的长度等于 `len`、包含非零字节，且每个字节均符合基于盘标签 `tag` 与盘偏移 `offset` 生成的确定性测试数据图样。
fn patterned(bytes: &[u8], tag: u8, offset: u64, len: usize) -> bool {
    bytes.len() == len
        && bytes.iter().any(|byte| *byte != 0)
        && bytes.iter().enumerate().all(|(index, byte)| {
            let at = offset + index as u64;
            *byte
                == tag
                    .wrapping_mul(29)
                    .wrapping_add(((at / 512) as u8).wrapping_mul(17))
                    .wrapping_add((at % 512) as u8)
        })
}

/// 从测试套件共享的设备清单 `inventory` 中筛选出指定数量（`count`）的 VirtIO 块设备（MMIO 与 PCI），并依次执行第一阶段准备（`prepare_mmio` / `prepare_pci`）。
fn devices(inventory: &DeviceInventory, count: usize) -> Result<Vec<VirtioDevice>, TestError> {
    let pci = inventory.pci.devices().iter().filter(|device| {
        device.vendor_id == VIRTIO_VENDOR_ID
            && matches!(
                device.device_id,
                VIRTIO_PCI_BLOCK_LEGACY | VIRTIO_PCI_BLOCK_MODERN
            )
    });
    if inventory.mmio_blocks.len() + pci.clone().count() != count {
        return Err(TestError(
            "cache attached device count differs from declared resources",
        ));
    }

    let mut prepared = Vec::new();
    prepared
        .try_reserve_exact(count)
        .map_err(|_| TestError("cache device owner allocation failed"))?;
    for candidate in &inventory.mmio_blocks {
        // SAFETY:
        // 1. `candidate` 来自测试套件级共享保活的 `DeviceInventory`，其 MMIO 寄存器窗口在内核启动时已建立永久强序映射。
        // 2. 前序测试用例已在 `quiesce` 中调用 `Disk::stop()` 完成硬件复位与中断注销，当前用例独占该 MMIO 块设备直至自身 `quiesce` 结束。
        prepared.push(
            unsafe { VirtioDevice::prepare_mmio(candidate) }
                .map_err(|_| TestError("cache MMIO preparation failed"))?,
        );
    }
    for device in pci {
        // SAFETY:
        // 1. `device` 来自 `inventory.pci`，其 PCI 配置空间与 BAR 映射在内核运行期持续有效。
        // 2. 前序测试用例已完成设备停止与后台工作队列同步清理，当前用例独占该 PCI BDF 直至测试结束。
        prepared.push(
            unsafe { VirtioDevice::prepare_pci(&inventory.pci, device) }
                .map_err(|_| TestError("cache PCI preparation failed"))?,
        );
    }
    Ok(prepared)
}

/// `folio` 测试套件的测试夹具，负责管理单页 `FolioRef` 与高阶物理页块的生命周期及内存基线核对。
struct Pages {
    /// 测试用例启动前采样的系统物理内存六维统计基线。
    baseline: mm::MemoryStats,
    /// 测试持有的最多两个 `FolioRef` 槽位，用于验证克隆共享与引用计数递减归还。
    folios: [Option<FolioRef>; 2],
    /// 预分配的一块 1 阶（2 页）连续物理页，用于验证 `FolioRef::from_pages` 对高阶页块的拒绝与原样退回。
    higher: Option<mm::OwnedPages>,
}

impl Fixture for Pages {
    /// 初始化 `Pages` 夹具：记录内存基线，并预先分配一个 0 阶 `FolioRef` 与一个 1 阶 `OwnedPages`。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled()?;
        let pages = mm::alloc_pages(0, false).ok_or(TestError("folio order0 preparation OOM"))?;
        let folio = FolioRef::from_pages(pages).map_err(|_| TestError("folio rejected order0"))?;
        let higher = mm::alloc_pages(1, false).ok_or(TestError("folio order1 preparation OOM"))?;
        Ok(Self {
            baseline,
            folios: [Some(folio), None],
            higher: Some(higher),
        })
    }

    /// 拆卸 `Pages` 夹具：释放内部持有的全部 `FolioRef` 与 `OwnedPages` 后，核对物理内存六维统计完全恢复至 `baseline`。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        recovered(context, baseline)
    }
}

impl Pages {
    /// 验证 `FolioRef` 克隆后的物理页帧号（PFN）一致性、最后一个强引用释放时的物理页回收，以及 `FolioRef::from_pages` 拒绝高阶物理页时完整保留原页块与数据内容。
    fn identity(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 克隆首个 0 阶 FolioRef，验证两个句柄指向同一物理地址且引用计数为 2；释放首个句柄后物理页尚未归还，释放第二个句柄后伙伴系统空闲页数增加 1。
        let before = settled()?;
        let first = self.folios[0].as_ref().unwrap();
        let address = first.phys_addr();
        self.folios[1] = Some(first.clone());
        context.expect_eq(self.folios[1].as_ref().unwrap().phys_addr(), address);
        context.expect_eq(self.folios[0].as_ref().unwrap().ref_count(), 2);
        drop(self.folios[0].take());
        context.expect_eq(self.folios[1].as_ref().unwrap().ref_count(), 1);
        context.expect_eq(settled()?, before);
        drop(self.folios[1].take());
        let mut expected = before;
        expected.buddy_free_pages += 1;
        context.expect_eq(settled()?, expected);

        // Step 2: 将 1 阶（2 页）物理页块填充为 0xa7 后传入 FolioRef::from_pages，验证其被拒绝并原样返回 OwnedPages 且块内字节未被篡改，释放后空闲页数再增加 2。
        let mut higher = self.higher.take().unwrap();
        let address = higher.phys_addr();
        let len = higher.len();
        // SAFETY:
        // 1. `higher` 是当前测试独占持有的 1 阶 `OwnedPages`，尚未转换为 `FolioRef` 或移交任何设备 DMA。
        // 2. `higher.as_mut_ptr()` 指向该连续物理页块的直接映射起始地址，写入字节数 `len == higher.len()` 严格在已分配页块边界内。
        unsafe { higher.as_mut_ptr().cast::<u8>().write_bytes(0xa7, len) };
        match FolioRef::from_pages(higher) {
            Err(mut original) => {
                context.expect_eq(original.phys_addr(), address);
                context.expect_eq(original.order(), 1);
                context.expect_eq(original.len(), len);
                // SAFETY:
                // 1. `original` 是 `FolioRef::from_pages` 失败后原样退回的独占 `OwnedPages`，其全部 `len` 字节已在上方由 `write_bytes(0xa7, len)` 完成初始化。
                // 2. 切片借用仅在当前表达式内只读检查字节值，检查结束后才将 `original` 移回 `self.higher`，不存在可变别名冲突。
                let unchanged =
                    unsafe { core::slice::from_raw_parts(original.as_mut_ptr().cast::<u8>(), len) }
                        .iter()
                        .all(|byte| *byte == 0xa7);
                context.expect_eq(unchanged, true);
                self.higher = Some(original);
            }
            Ok(folio) => {
                self.folios[0] = Some(folio);
                return Err(TestError("folio accepted higher-order pages"));
            }
        }
        drop(self.higher.take());
        expected.buddy_free_pages += 2;
        context.expect_eq(settled()?, expected);
        Ok(())
    }
}

/// 缓存集成测试套件的具体测试场景枚举，决定夹具 `Owner` 需要准备的块设备数量、缓存类型、访问模式与工作任务数。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    /// 双块设备（`root` 与 `extra`）上的 1KiB / 2KiB / 4KiB 元数据子页视图读取与跨盘物理页隔离测试。
    Views,
    /// 元数据缓存（`MetadataCache`）相同块号并发读取的单飞（Singleflight）合并测试。
    MetadataFlight,
    /// 文件页稀疏索引（`FolioIndex`）相同页索引并发读取的单飞（Singleflight）合并测试。
    FileFlight,
    /// 元数据缓存不同块号并发读取的双请求并行在途测试。
    MetadataIndependent,
    /// 文件页稀疏索引不同页索引并发读取的双请求并行在途测试。
    Independent,
    /// 块设备队列停止（`BlockError::QueueStopped`）时文件页加载失败及同页阻塞等待者错误传播测试。
    Failure,
    /// 固定容量上限（容量为 2）的元数据缓存淘汰拒绝与无界文件页稀疏索引修剪测试。
    Bounded,
    /// 块 I/O 请求准备阶段（`Disk::load_folio`）遭遇物理页与 SLUB 堆双重耗尽（OOM）时的零提交回滚与等待者唤醒测试。
    PrepareOom,
    /// 物理页耗尽（OOM）期间既有元数据/文件缓存命中读取与释放压力后缺失加载恢复测试。
    Oom,
    /// 私有测试盘上 1KiB / 2KiB / 4KiB 重叠元数据视图的按扇区读取补齐与脏数据保留测试。
    MutableMetadata,
    /// 同一物理页内两条 256 字节 ext4 Inode 元数据记录的并发互斥更新与 CRC32C 校验保持测试。
    MetadataRecords,
}

/// 工作任务或测试用例从缓存获取到的页结果封装（元数据缓冲区视图或文件物理页引用）。
enum Returned {
    /// 元数据缓存返回的块视图引用（`BufferRef`）。
    Buffer(BufferRef),
    /// 文件页稀疏索引返回的物理页引用（`FolioRef`）。
    Folio(FolioRef),
}

impl Returned {
    /// 获取该缓存结果底层承载数据的 `FolioRef` 物理页引用（零拷贝观察物理页身份）。
    fn folio(&self) -> &FolioRef {
        match self {
            Self::Buffer(buffer) => buffer.folio(),
            Self::Folio(folio) => folio,
        }
    }

    /// 校验该缓存页或缓冲区视图中的完整 `PAGE_SIZE` 字节数据是否符合盘偏移 `offset` 处的测试图样。
    fn valid(&self, offset: u64) -> Result<bool, BlockError> {
        match self {
            Self::Buffer(buffer) => {
                buffer.with_read(|bytes| patterned(bytes, 1, offset, mm::PAGE_SIZE))
            }
            Self::Folio(folio) => {
                let mut bytes = [0_u8; mm::PAGE_SIZE];
                folio.copy_to(0, &mut bytes)?;
                Ok(patterned(&bytes, 1, offset, mm::PAGE_SIZE))
            }
        }
    }
}

/// 同时耗尽伙伴系统物理页与全部 SLUB 堆尺寸类别的内存压力注入器（使用侵入式单链表管理已分配堆块，加压期间零额外堆分配）。
pub(crate) struct HeapPressure {
    /// 已从伙伴系统抽干并持有的物理页块列表（在 `prepare` 阶段已按系统总页数预留容量）。
    pages: Vec<mm::OwnedPages>,
    /// 每个 SLUB 大小类别（对应 `mm::SLUB_SIZES`）已分配对象的侵入式单链表头指针（以 `usize` 地址存储，0 表示空链表；每个堆块首字存放下一节点地址）。
    heap: [usize; mm::SLUB_SIZES.len()],
}

impl HeapPressure {
    /// 在启动设备或进入压力区前预先创建 `HeapPressure`，并按系统物理总页数预留 `pages` 向量容量。
    pub(crate) fn prepare() -> Result<Self, TestError> {
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(mm::stats().present_pages)
            .map_err(|_| TestError("cache prepare-OOM pressure slots unavailable"))?;
        Ok(Self {
            pages,
            heap: [0; mm::SLUB_SIZES.len()],
        })
    }

    /// 耗尽系统全部可用物理页与所有 `SLUB_SIZES` 类别的空闲堆对象，并通过一次实际分配探测返回 `(物理页是否耗尽, 堆对象是否耗尽)`。
    pub(crate) fn exhaust(&mut self) -> (bool, bool) {
        // Step 1: 收缩 SLUB 并排空 PCP 缓存，随后依次按最大阶与 0 阶抽干伙伴系统中的全部空闲物理页。
        mm::shrink_all();
        mm::drain_all_pages();
        for order in [mm::MAX_ORDER, 0] {
            while let Some(pages) = mm::alloc_pages(order, false) {
                assert!(
                    self.pages.len() < self.pages.capacity(),
                    "cache pressure capacity"
                );
                self.pages.push(pages);
            }
        }

        // Step 2: 遍历所有 SLUB 尺寸类别持续调用全局分配器，将分配出的每个内存块首字写入当前链表头并挂入侵入式单链表，直到返回空指针。
        for (index, size) in mm::SLUB_SIZES.iter().copied().enumerate() {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("cache heap pressure layout");
            loop {
                // SAFETY:
                // 1. `layout` 的大小来自非空的 `mm::SLUB_SIZES[index]`，对齐为 `align_of::<usize>()`（2 的幂），满足 `alloc` 对非零有效布局的要求。
                // 2. 返回的非空指针由当前 `HeapPressure` 实例独占接管并在 `Drop` 中按相同 `layout` 释放。
                let pointer = black_box(unsafe { alloc(layout) });
                if pointer.is_null() {
                    break;
                }
                // SAFETY:
                // 1. `pointer` 刚刚由 `alloc(layout)` 成功分配且非空，`mm::SLUB_SIZES` 中最小类别不小于 `size_of::<usize>()`（8 字节），且对齐满足 `align_of::<usize>()`。
                // 2. 该内存块由当前链表独占，将首字作为 `usize` 写入前一个链表头地址不会越界或产生并发别名。
                unsafe { pointer.cast::<usize>().write(self.heap[index]) };
                self.heap[index] = pointer as usize;
            }
        }

        // Step 3: 再次排空 PCP 后分别对 0 阶物理页与全部 SLUB 类别（含大页块）做一次分配探测，验证两者均返回空（OOM）。
        mm::drain_all_pages();
        let pages_failed = black_box(mm::alloc_pages(0, false)).is_none();
        let mut heap_failed = true;
        for size in mm::SLUB_SIZES
            .iter()
            .copied()
            .chain(core::iter::once(mm::PAGE_SIZE * 2))
        {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("cache heap probe layout");
            // SAFETY:
            // `layout` 具有非零大小与 `usize` 对齐，调用全局分配器探测当前堆是否已耗尽；若意外分配成功，下方立即在同一作用域内调用 `dealloc` 归还。
            let pointer = black_box(unsafe { alloc(layout) });
            if !pointer.is_null() {
                heap_failed = false;
                // SAFETY:
                // `pointer` 是紧邻上方 `alloc(layout)` 刚返回的非空独占指针，未产生任何外部引用，按完全相同的 `layout` 立即释放一次。
                unsafe { dealloc(pointer, layout) };
            }
        }
        (pages_failed, heap_failed)
    }
}

impl Drop for HeapPressure {
    /// 析构压力注入器：遍历每个 SLUB 尺寸类别的侵入式链表并按原 `Layout` 逐一释放堆块，随后由字段析构自动归还 `pages` 中的全部物理页。
    fn drop(&mut self) {
        for (index, size) in mm::SLUB_SIZES.iter().copied().enumerate() {
            let layout = Layout::from_size_align(size, core::mem::align_of::<usize>())
                .expect("cache heap release layout");
            while self.heap[index] != 0 {
                let pointer = self.heap[index] as *mut u8;
                // SAFETY:
                // 1. `pointer` 来自 `exhaust()` 中对同一类别 `layout` 调用 `alloc` 所得的非空指针，且其首字已在入链时由 `write` 初始化为下一节点地址。
                // 2. 每个节点仅在 `drop` 循环中出链并调用 `dealloc(pointer, layout)` 一次，布局与分配时完全一致。
                unsafe {
                    self.heap[index] = pointer.cast::<usize>().read();
                    dealloc(pointer, layout);
                }
            }
        }
    }
}

/// 在 `PrepareOom` 场景的 0 号控制任务中执行：先通过 `HeapPressure::exhaust` 耗尽系统物理页与 SLUB 堆，再调用 `Disk::load_folio` 触发准备阶段 OOM，待其返回后释放内存压力。
fn prepare_oom_read(observation: &Observation, disk: &DiskRef) -> Result<(), BlockError> {
    let mut pressure = observation
        .pressure
        .lock()
        .take()
        .expect("cache prepared heap pressure missing");
    let evidence = pressure.exhaust();
    *observation.oom_evidence.lock() = Some(evidence);
    let read = observation
        .read
        .lock()
        .take()
        .expect("cache prepare-OOM original read missing");
    let result = if evidence == (true, true) {
        disk.load_folio(OFFSET, read, Vec::new_in(BioAllocator::Heap))
    } else {
        // 若探测发现物理页或堆未能彻底耗尽，则直接析构 `FolioRead` 取消本次读取并报错；主测试线程随后会检查 `oom_evidence` 并判定测试失败。
        drop(read);
        Err(BlockError::QueueStopped)
    };
    drop(pressure);
    result
}

/// 多核缓存测试工作任务与主测试线程之间的同步与结果观察站。
struct Observation {
    /// 取消标志：由 `quiesce` 置为 `true`，通知尚未进入缓存操作的工作任务提前退出。
    cancelled: AtomicBool,
    /// 启动门闩：在 `Failure` 与 `PrepareOom` 场景中阻塞 0 号加载任务，直到其他同页等待者进入阻塞态且前置条件就绪后才放行。
    gate: Completion,
    /// 记录最多 3 个工作任务是否已跨过门闩并实际进入被测缓存/块设备调用路径。
    entered: [AtomicBool; 3],
    /// 记录最多 3 个工作任务的 `Arc<Task>` 句柄（由自旋锁保护），供主测试线程查询任务调度状态是否为 `State::Blocked`。
    tasks: SpinLock<[Option<Arc<Task>>; 3]>,
    /// 记录最多 3 个工作任务执行缓存/块读取后的返回结果（由自旋锁保护）。
    results: SpinLock<[Option<Result<Returned, BlockError>>; 3]>,
    /// 暂存 `Failure` 或 `PrepareOom` 场景下在 `set_up` 阶段预先创建的唯一 `FolioRead` 加载所有权凭证（由自旋锁保护）。
    read: SpinLock<Option<FolioRead>>,
    /// 暂存 `PrepareOom` 场景下预先准备的 `HeapPressure` 内存压力注入器（由自旋锁保护）。
    pressure: SpinLock<Option<HeapPressure>>,
    /// 记录 `PrepareOom` 场景下 `HeapPressure::exhaust` 返回的 `(物理页耗尽, 堆耗尽)` 探测证据（由自旋锁保护）。
    oom_evidence: SpinLock<Option<(bool, bool)>>,
}

impl Observation {
    /// 在启动任何工作任务之前分配并初始化 `Observation` 共享观察站。
    fn prepare(
        read: Option<FolioRead>,
        pressure: Option<HeapPressure>,
    ) -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            cancelled: AtomicBool::new(false),
            gate: Completion::new(),
            entered: [const { AtomicBool::new(false) }; 3],
            tasks: SpinLock::new([const { None }; 3]),
            results: SpinLock::new([const { None }; 3]),
            read: SpinLock::new(read),
            pressure: SpinLock::new(pressure),
            oom_evidence: SpinLock::new(None),
        })
        .map_err(|_| TestError("cache observer preparation OOM"))
    }

    /// 检查第 `index` 号工作任务是否已进入被测缓存路径且当前在调度器中处于 `State::Blocked` 阻塞睡眠态。
    fn blocked(&self, index: usize) -> bool {
        if !self.entered[index].load(Ordering::Acquire) {
            return false;
        }
        let task = self.tasks.lock()[index].clone();
        task.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }
}

/// 通用缓存集成测试夹具的核心资源持有者，统一管理块设备、完成工作队列、元数据/文件页缓存、并发工作任务及内存压力句柄。
struct Owner {
    /// 测试用例启动前在系统静默状态下记录的物理内存六维统计基线。
    baseline: mm::MemoryStats,
    /// 主测试块设备引用（在 `MetadataRecords` 场景下与 `extra` 交换后指向私有可写测试盘）。
    disk: DiskRef,
    /// 可选的第二块测试块设备引用（仅在 `Views` 与 `MetadataRecords` 双盘场景下启用）。
    extra: Option<DiskRef>,
    /// 驱动主/副块设备异步 I/O 完成回调的内核工作队列。
    queue: WorkQueue,
    /// 绑定在主块设备 `disk` 上的共享元数据缓存实例。
    metadata: Option<Arc<MetadataCache>>,
    /// 绑定在第二块设备 `extra` 上的共享元数据缓存实例。
    extra_metadata: Option<Arc<MetadataCache>>,
    /// 供文件页缓存测试使用的 `FolioIndex` 稀疏页索引实例。
    mapping: Arc<FolioIndex>,
    /// 主测试线程与后台并发工作任务之间共享的同步门闩与执行结果观察站。
    observation: Arc<Observation>,
    /// 测试用例在主线程侧持有的最多 3 个缓存页或缓冲区引用槽位，用于控制页面存活期。
    held: [Option<Returned>; 3],
    /// 已预先分配内核栈但尚未发布到调度器的后台工作任务列表。
    prepared: Vec<PreparedTask>,
    /// 已发布到调度器正在运行或等待回收的后台工作任务连接句柄列表。
    joins: Vec<JoinHandle>,
    /// `Oom` 场景下为耗尽伙伴系统空闲页而暂存的物理页块列表。
    pressure: Vec<mm::OwnedPages>,
    /// 主块设备 `disk` 是否为 Legacy VirtIO MMIO（v1）设备，用于计算 VirtQueue 环布局。
    legacy: bool,
    /// 设备激活阶段的预设错误码（正常运行路径下为 `None`）。
    activation_error: Option<BlockError>,
    /// 主块设备 VirtQueue 可用环 `avail.idx` 字段在内核虚拟地址空间中的只读观察地址（单位：字节地址）。
    avail_address: usize,
    /// 当前场景是否在私有测试盘上产生了无需回写落盘的内存态脏元数据（`MutableMetadata` 与 `MetadataRecords` 为 `true`）。
    private_metadata: bool,
}

impl Owner {
    /// 根据测试场景 `scenario` 从设备清单 `resources` 中初始化块设备、完成工作队列及配套缓存与任务资源。
    fn prepare(scenario: Scenario, resources: &DeviceInventory) -> Result<Self, TestError> {
        // Step 1: 采样系统内存基线，按场景需求取出 1 或 2 个 VirtIO 块设备并校验逻辑块大小约束。
        let baseline = settled()?;
        let mut inventory = devices(
            resources,
            if matches!(scenario, Scenario::Views | Scenario::MetadataRecords) {
                2
            } else {
                1
            },
        )?;
        let device = inventory.remove(0);
        let info = device.info();
        let unit = info.logical_block_size as usize;
        if !matches!(unit, 512 | 1024 | 2048 | 4096)
            || (scenario == Scenario::Views
                && (1024 < unit
                    || inventory
                        .iter()
                        .any(|device| 1024 < device.info().logical_block_size)))
        {
            return Err(TestError(
                "cache views require logical block <= 1KiB; page cases require <= 4KiB",
            ));
        }

        // Step 2: 计算主设备 VirtQueue avail.idx 观察地址，创建完成工作队列并启动主块设备。
        let legacy = matches!(device.identity(), VirtioIdentity::Mmio { version: 1, .. });
        let ring = device.control_regions().0;
        let layout = virtio_drivers::queue::RingLayout::new(info.queue_depth, legacy)
            .map_err(|_| TestError("cache real ring layout invalid"))?;
        let avail_address =
            ring.cpu.as_ptr() as usize + layout.avail_offset() + core::mem::size_of::<u16>();
        let queue = WorkQueue::try_new(WorkQueueConfig {
            max_active: usize::from(info.tag_count) + 1,
            ordered: false,
            reclaim: true,
        })
        .map_err(|_| TestError("cache completion domain preparation OOM"))?;
        let disk = match Disk::prepare(device, &queue) {
            Ok(disk) => disk,
            Err(_) => {
                queue.close_sync();
                return Err(TestError("cache Disk preparation failed"));
            }
        };
        if disk.start().is_err() {
            disk.stop()
                .map_err(|_| TestError("cache failed activation stop failed"))?;
            drop(disk);
            queue.close_sync();
            return Err(TestError("cache Disk activation failed"));
        }

        // Step 3: 继续初始化副块设备、元数据/文件缓存、并发工作任务与压力容器；若中途失败则在内部按逆序停止设备并关闭工作队列。
        let owner = Self::prepare_resources(
            scenario,
            baseline,
            disk.clone(),
            queue,
            legacy,
            inventory,
            avail_address,
        )?;
        Ok(owner)
    }

    /// 在主块设备已启动的前提下，准备可选副块设备、文件系统占用凭证、元数据/文件页缓存以及预构建的并发内核任务；若任何步骤失败则按逆序停止设备并清理队列。
    fn prepare_resources(
        scenario: Scenario,
        baseline: mm::MemoryStats,
        mut disk: DiskRef,
        queue: WorkQueue,
        legacy: bool,
        mut inventory: Vec<VirtioDevice>,
        avail_address: usize,
    ) -> Result<Self, TestError> {
        // Step 1: 若当前场景包含第二块设备，则准备并启动副块设备；在 MetadataRecords 场景下区分只读 ext4 根盘与私有测试盘。
        let mut extra = match inventory
            .pop()
            .map(|device| Disk::prepare(device, &queue))
            .transpose()
        {
            Ok(extra) => extra,
            Err(_) => {
                disk.stop()
                    .map_err(|_| TestError("unpublished cache Disk stop failed"))?;
                drop(disk);
                queue.close_sync();
                return Err(TestError("cache extra Disk preparation failed"));
            }
        };
        let resources = (|| {
            if let Some(extra) = &extra {
                extra
                    .start()
                    .map_err(|_| TestError("extra cache activation failed"))?;
            }
            if scenario == Scenario::MetadataRecords {
                let first_root =
                    Ext4Volume::open(BlockView::whole(disk.clone()), AccessMode::ReadOnly, 8)
                        .is_ok();
                let second_root = Ext4Volume::open(
                    BlockView::whole(extra.as_ref().unwrap().clone()),
                    AccessMode::ReadOnly,
                    8,
                )
                .is_ok();
                if first_root == second_root {
                    return Err(TestError(
                        "record case requires exactly one real ext4 root and one scratch",
                    ));
                }
                if first_root {
                    core::mem::swap(&mut disk, extra.as_mut().unwrap());
                }
            }

            // Step 2: 按场景申请块设备的文件系统访问凭证（只读或读写），创建对应的 MetadataCache 与 FolioIndex。
            let capacity = if scenario == Scenario::Bounded { 2 } else { 8 };
            let metadata = if matches!(
                scenario,
                Scenario::Views
                    | Scenario::MetadataFlight
                    | Scenario::MetadataIndependent
                    | Scenario::Bounded
                    | Scenario::Oom
                    | Scenario::MutableMetadata
                    | Scenario::MetadataRecords
            ) {
                let mode = if matches!(
                    scenario,
                    Scenario::MutableMetadata | Scenario::MetadataRecords
                ) {
                    AccessMode::ReadWrite
                } else {
                    AccessMode::ReadOnly
                };
                let claim = Disk::claim_fs(&BlockView::whole(disk.clone()), mode)
                    .map_err(|_| TestError("metadata FS claim failed"))?;
                Some(
                    MetadataCache::shared(&claim, capacity)
                        .map_err(|_| TestError("metadata preparation failed"))?,
                )
            } else {
                None
            };
            let mapping = Arc::try_new(FolioIndex::new(None, false))
                .map_err(|_| TestError("mapping owner preparation OOM"))?;
            let extra_metadata = extra
                .as_ref()
                .map(|disk| {
                    let claim =
                        Disk::claim_fs(&BlockView::whole(disk.clone()), AccessMode::ReadOnly)?;
                    MetadataCache::shared(&claim, 8)
                })
                .transpose()
                .map_err(|_| TestError("extra metadata preparation failed"))?;

            // Step 3: 为 Failure 与 PrepareOom 场景预先插入第 64 页占位 Folio 并获取唯一 FolioRead 凭证，同时按需准备内存压力注入器与观察站。
            let (failure_folio, read) =
                if matches!(scenario, Scenario::Failure | Scenario::PrepareOom) {
                    let (folio, read) = mapping
                        .get_or_prepare(64, None)
                        .map_err(|_| TestError("failure placeholder preparation failed"))?;
                    (Some(folio), read)
                } else {
                    (None, None)
                };
            let pressure = if scenario == Scenario::PrepareOom {
                Some(HeapPressure::prepare()?)
            } else {
                None
            };
            let observation = Observation::prepare(read, pressure)?;
            let count = match scenario {
                Scenario::MetadataFlight
                | Scenario::FileFlight
                | Scenario::MetadataIndependent
                | Scenario::Independent => 2,
                Scenario::Failure | Scenario::PrepareOom => 3,
                _ => 0,
            };
            let mut prepared = Vec::new();
            let mut joins = Vec::new();
            prepared
                .try_reserve_exact(
                    if matches!(
                        scenario,
                        Scenario::MutableMetadata | Scenario::MetadataRecords
                    ) {
                        2
                    } else {
                        count
                    },
                )
                .map_err(|_| TestError("cache prepared Task slots OOM"))?;
            joins
                .try_reserve_exact(
                    if matches!(
                        scenario,
                        Scenario::MutableMetadata | Scenario::MetadataRecords
                    ) {
                        2
                    } else {
                        count
                    },
                )
                .map_err(|_| TestError("cache join slots OOM"))?;
            let mut pressure = Vec::new();
            if scenario == Scenario::Oom {
                pressure
                    .try_reserve_exact(PRESSURE_CAPACITY)
                    .map_err(|_| TestError("cache pressure handles OOM"))?;
            }

            // Step 4: 按场景所需并发度预创建绑定到不同 CPU 的内核任务（PreparedTask），定义各任务在门闩放行后的缓存访问逻辑。
            for index in 0..count {
                let observer = observation.clone();
                let task_disk = disk.clone();
                let cache = if matches!(
                    scenario,
                    Scenario::MetadataFlight | Scenario::MetadataIndependent
                ) {
                    metadata.clone()
                } else {
                    None
                };
                let file = mapping.clone();
                let failure_folio = failure_folio.clone();
                prepared.push(
                    task::prepare_kernel_on(index % cpu::count(), move || {
                        observer.tasks.lock()[index] = Some(task::current_task());
                        if matches!(scenario, Scenario::Failure | Scenario::PrepareOom)
                            && index == 0
                        {
                            observer.gate.wait();
                        }
                        if observer.cancelled.load(Ordering::Acquire) {
                            return 0;
                        }
                        observer.entered[index].store(true, Ordering::Release);
                        let key = if matches!(
                            scenario,
                            Scenario::Independent | Scenario::MetadataIndependent
                        ) {
                            64 + index as u64
                        } else {
                            64
                        };
                        let result = if matches!(
                            scenario,
                            Scenario::MetadataFlight | Scenario::MetadataIndependent
                        ) {
                            cache
                                .as_ref()
                                .unwrap()
                                .get(key, mm::PAGE_SIZE, None)
                                .map(Returned::Buffer)
                        } else if scenario == Scenario::PrepareOom && index == 0 {
                            prepare_oom_read(&observer, &task_disk)
                                .map(|()| Returned::Folio(failure_folio.unwrap()))
                        } else if scenario == Scenario::Failure && index == 0 {
                            let read = observer
                                .read
                                .lock()
                                .take()
                                .expect("failure original read owner missing");
                            task_disk
                                .load_folio(OFFSET, read, Vec::new_in(BioAllocator::Heap))
                                .map(|()| Returned::Folio(failure_folio.unwrap()))
                        } else {
                            match file.get_or_prepare(key, None) {
                                Ok((folio, Some(read))) => task_disk
                                    .load_folio(
                                        key * mm::PAGE_SIZE as u64,
                                        read,
                                        Vec::new_in(BioAllocator::Heap),
                                    )
                                    .map(|()| Returned::Folio(folio)),
                                Ok((folio, None)) => {
                                    folio.wait_ready().map(|()| Returned::Folio(folio))
                                }
                                Err(error) => Err(error),
                            }
                        };
                        observer.results.lock()[index] = Some(result);
                        0
                    })
                    .map_err(|_| TestError("cache native Task preparation failed"))?,
                );
            }
            let mut held = [const { None }; 3];
            held[0] = failure_folio.map(Returned::Folio);
            Ok((
                metadata,
                mapping,
                extra_metadata,
                observation,
                prepared,
                joins,
                pressure,
                held,
            ))
        })();
        // Step 5: 汇总全部已就绪资源构造 Owner；若上述闭包报错则停止副/主块设备并同步关闭工作队列。
        match resources {
            Ok((
                metadata,
                mapping,
                extra_metadata,
                observation,
                prepared,
                joins,
                pressure,
                held,
            )) => Ok(Self {
                baseline,
                disk,
                extra,
                queue,
                metadata,
                extra_metadata,
                mapping,
                observation,
                prepared,
                joins,
                pressure,
                held,
                legacy,
                activation_error: None,
                avail_address,
                private_metadata: matches!(
                    scenario,
                    Scenario::MutableMetadata | Scenario::MetadataRecords
                ),
            }),
            Err(error) => {
                if let Some(extra) = &extra {
                    extra
                        .stop()
                        .map_err(|_| TestError("unpublished extra cache Disk stop failed"))?;
                }
                drop(extra);
                disk.stop()
                    .map_err(|_| TestError("unpublished cache Disk stop failed"))?;
                drop(disk);
                queue.close_sync();
                Err(error)
            }
        }
    }

    /// 检查块设备激活状态是否正常，作为各测试方法入口的前置校验。
    fn ready(&self) -> TestResult {
        if self.activation_error.is_some() {
            Err(TestError("cache Disk activation failed"))
        } else {
            Ok(())
        }
    }

    /// 从预创建任务队列头部取出一个 `PreparedTask` 并发布到调度器运行，将其 `JoinHandle` 存入已预留容量的 `joins` 向量。
    fn publish_one(&mut self) {
        self.joins.push(self.prepared.remove(0).publish());
    }

    /// 轮询等待全部已发布的工作任务彻底退出内核栈后，逐一消费 `JoinHandle` 并检查任务返回值是否均为 0。
    fn join_tasks(&mut self) -> TestResult {
        for join in &self.joins {
            until("cache producer did not leave stack", || {
                join.try_join().is_some()
            })?;
        }
        let mut failed = false;
        for join in self.joins.drain(..) {
            failed |= join.join() != 0;
        }
        if failed {
            Err(TestError("cache native producer failed"))
        } else {
            Ok(())
        }
    }

    /// 轮询等待主块设备硬件 VirtQueue 的 `used.idx` 推进到目标值 `target`，确认设备已完成对应数量的 I/O 请求。
    fn used(&self, target: u16) -> TestResult {
        until("cache real used did not progress", || {
            block::tests::used_index(&self.disk, self.legacy) == target
        })
    }

    /// 将前两个工作任务在 `Observation::results` 中记录的读取结果移入夹具的 `held` 槽位（先释放观察站自旋锁，再由后续测试逻辑访问可能获取睡眠锁的页面内容）。
    fn take_results(&mut self) -> TestResult {
        for index in 0..2 {
            let result = self.observation.results.lock()[index].take();
            self.held[index] = Some(
                result
                    .ok_or(TestError("cache producer result absent"))?
                    .map_err(|_| TestError("cache real read failed"))?,
            );
        }
        Ok(())
    }

    /// 验证同键并发读取的单飞（Singleflight）合并：首个读取者与同键等待者均进入 `State::Blocked`，底层块设备仅提交 1 次请求，完成回调恢复后两者共享同一物理页并读出完整数据。
    fn singleflight(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 暂停块设备完成回调分发，依次发布 0 号加载任务与 1 号同键等待任务，确认两者均进入 Blocked 状态且仅产生 1 次块 I/O 提交。
        self.disk.test_suspend_completions();
        let before = self.disk.test_snapshot();
        let used = block::tests::used_index(&self.disk, self.legacy);
        self.publish_one();
        self.used(used.wrapping_add(1))?;
        until("cache read owner did not block", || {
            self.observation.blocked(0)
        })?;
        self.publish_one();
        until("cache same-key waiter did not block", || {
            self.observation.blocked(1)
        })?;
        context.expect_eq(self.disk.test_snapshot().submitted - before.submitted, 1);
        context.expect_eq(self.disk.test_snapshot().inflight, 1);

        // Step 2: 恢复块设备完成回调并等待两个任务退出内核栈，验证两个任务获取到相同的物理页帧号（PFN）且页面数据符合预期图样。
        self.disk.test_resume_completions();
        self.join_tasks()?;
        self.disk.test_flush_completions();
        self.take_results()?;
        let mut address = None;
        for returned in self.held[..2].iter().flatten() {
            if let Some(address) = address {
                context.expect_eq(returned.folio().phys_addr(), address);
            }
            address = Some(returned.folio().phys_addr());
            context.expect_eq(returned.valid(OFFSET), Ok(true));
        }
        context.expect_eq(self.disk.test_snapshot().submitted - before.submitted, 1);
        Ok(())
    }

    /// 验证不同键并发读取的并行度：在块设备完成回调暂停窗口内，两个独立键的读取任务同时处于 `State::Blocked` 且在设备上保持 2 个在途请求，恢复后各自填入不同物理页。
    fn independent(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 暂停块设备完成回调，同时发布两个访问相邻键的工作任务，确认硬件已处理 2 个请求、两个任务均处于 Blocked 且在途请求数为 2。
        self.disk.test_suspend_completions();
        let before = self.disk.test_snapshot();
        let used = block::tests::used_index(&self.disk, self.legacy);
        self.publish_one();
        self.publish_one();
        self.used(used.wrapping_add(2))?;
        until("independent file reads did not block", || {
            self.observation.blocked(0) && self.observation.blocked(1)
        })?;
        context.expect_eq(self.disk.test_snapshot().submitted - before.submitted, 2);
        context.expect_eq(self.disk.test_snapshot().inflight, 2);

        // Step 2: 恢复完成回调并回收任务，验证两个结果位于不同物理页且各自内容符合对应偏移的测试图样。
        self.disk.test_resume_completions();
        self.join_tasks()?;
        self.disk.test_flush_completions();
        self.take_results()?;
        let mut first = None;
        for (index, returned) in self.held[..2].iter().flatten().enumerate() {
            if let Some(address) = first {
                context.expect_eq(returned.folio().phys_addr() != address, true);
            }
            first = Some(returned.folio().phys_addr());
            context.expect_eq(
                returned.valid(OFFSET + index as u64 * mm::PAGE_SIZE as u64),
                Ok(true),
            );
        }
        Ok(())
    }

    /// 验证块设备停止（`BlockError::QueueStopped`）时的缓存失败传播：0 号加载者在队列停止后提交失败，正在阻塞等待同一物理页的 1 号与 2 号任务同时被唤醒并收到 `QueueStopped` 错误。
    fn failure(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 发布全部 3 个任务（0 号任务在 gate 门闩处等待，1 号和 2 号任务命中占位 Folio 后在 wait_ready 中进入 Blocked 状态）。
        self.publish_one();
        self.publish_one();
        self.publish_one();
        until("failed file waiters did not block", || {
            self.observation.blocked(1) && self.observation.blocked(2)
        })?;

        // Step 2: 停止块设备队列并放行 gate 门闩，0 号任务调用 load_folio 立即被拒绝且未新增任何硬件提交。
        self.disk
            .stop()
            .map_err(|_| TestError("cache failure real stop failed"))?;
        let before = self.disk.test_snapshot().submitted;
        self.observation.gate.complete();
        self.join_tasks()?;
        context.expect_eq(self.disk.test_snapshot().submitted, before);

        // Step 3: 验证全部 3 个任务均收到 QueueStopped 错误，且该 Folio 标记为已完成但携带错误码，后续缓存命中或读取同样返回 QueueStopped。
        for result in &*self.observation.results.lock() {
            context.expect_eq(matches!(result, Some(Err(BlockError::QueueStopped))), true);
        }
        let folio = self.held[0].as_ref().unwrap().folio();
        context.expect_eq(folio.is_ready(), true);
        context.expect_eq(folio.wait_ready(), Err(BlockError::QueueStopped));
        context.expect_eq(
            folio.copy_to(0, &mut [0_u8; mm::PAGE_SIZE]),
            Err(BlockError::QueueStopped),
        );
        let (hit, owner) = self
            .mapping
            .get_or_prepare(64, None)
            .map_err(|_| TestError("failure hit disappeared"))?;
        context.expect_eq(hit.phys_addr(), folio.phys_addr());
        context.expect_eq(owner.is_none(), true);
        context.expect_eq(hit.wait_ready(), Err(BlockError::QueueStopped));
        context.diagnostic(format_args!(
            "cache-failure real-QueueStopped=1 hardware-EIO-covered=0"
        ));
        Ok(())
    }

    /// 验证主盘与副盘在 1KiB、2KiB、4KiB 三种块粒度下的元数据子页视图：校验页内偏移、数据图样以及不同块设备之间的物理页隔离。
    fn views(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        let extra = self
            .extra_metadata
            .as_ref()
            .ok_or(TestError("extra metadata absent"))?;
        for size in [1024, 2048, 4096] {
            let offset = 33 * size as u64;
            let root = self
                .metadata
                .as_ref()
                .unwrap()
                .get(33, size, None)
                .map_err(|_| TestError("root metadata view read failed"))?;
            let second = extra
                .get(33, size, None)
                .map_err(|_| TestError("extra metadata view read failed"))?;
            context.expect_eq(root.len(), size);
            context.expect_eq(root.offset(), offset as usize % mm::PAGE_SIZE);
            context.expect_eq(
                root.with_read(|bytes| patterned(bytes, 1, offset, size)),
                Ok(true),
            );
            context.expect_eq(
                second.with_read(|bytes| patterned(bytes, 2, offset, size)),
                Ok(true),
            );
            context.expect_eq(root.folio().phys_addr() != second.folio().phys_addr(), true);
        }
        Ok(())
    }

    /// 在稀疏页索引 `mapping` 中查找或分配第 `index` 页，若当前调用成为唯一加载者则向主块设备发起 `load_folio`，最后等待页面就绪并返回 `FolioRef`。
    fn file_page(&self, index: u64) -> Result<FolioRef, BlockError> {
        let (folio, read) = self.mapping.get_or_prepare(index, None)?;
        if let Some(read) = read {
            self.disk.load_folio(
                index * mm::PAGE_SIZE as u64,
                read,
                Vec::new_in(BioAllocator::Heap),
            )?;
        }
        folio.wait_ready()?;
        Ok(folio)
    }

    /// 验证容量上限为 2 的元数据缓存淘汰与满额拒绝语义，并对比验证无容量上限的文件页稀疏索引（`FolioIndex`）可同时保有任意数量的活跃物理页。
    fn bounded(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 在容量为 2 的 MetadataCache 中同时持有第 64、65 两块，此时请求第 66 块因无空闲可淘汰槽位而返回 NoMemory，且 prune_unused 返回 0。
        self.held[0] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(64, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("bounded first metadata read failed"))?,
        ));
        self.held[1] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(65, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("bounded second metadata read failed"))?,
        ));
        self.disk.test_flush_completions();
        context.expect_eq(
            matches!(
                self.metadata.as_ref().unwrap().get(66, mm::PAGE_SIZE, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );
        context.expect_eq(self.metadata.as_ref().unwrap().prune_unused(), 0);

        // Step 2: 释放第 64 块引用后再次请求第 66 块，缓存自动回收无外部引用的干净旧页并成功加载第 66 块；全部释放后 prune_unused 回收剩余 2 页。
        drop(self.held[0].take());
        self.held[0] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(66, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("bounded clean metadata victim did not retire"))?,
        ));
        context.expect_eq(self.metadata.as_ref().unwrap().len(), 2);
        context.expect_eq(
            self.held[0]
                .as_ref()
                .unwrap()
                .valid(66 * mm::PAGE_SIZE as u64),
            Ok(true),
        );
        drop(self.held[0].take());
        drop(self.held[1].take());
        self.disk.test_flush_completions();
        context.expect_eq(self.metadata.as_ref().unwrap().prune_unused(), 2);
        context.expect_eq(self.metadata.as_ref().unwrap().len(), 0);

        // Step 3: 在无容量限制的 FolioIndex 中同时持有第 64、65、66 三个文件页，验证三者均可共存且内容正确，释放全部外部引用后 prune_unused 一次性回收 3 页。
        self.held[0] = Some(Returned::Folio(
            self.file_page(64)
                .map_err(|_| TestError("bounded first file read failed"))?,
        ));
        self.held[1] = Some(Returned::Folio(
            self.file_page(65)
                .map_err(|_| TestError("bounded second file read failed"))?,
        ));
        self.disk.test_flush_completions();
        self.held[2] =
            Some(Returned::Folio(self.file_page(66).map_err(|_| {
                TestError("sparse third live file page failed")
            })?));
        context.expect_eq(self.mapping.prune_unused(None), 0);
        for (index, held) in self.held.iter().flatten().enumerate() {
            context.expect_eq(
                held.valid((64 + index as u64) * mm::PAGE_SIZE as u64),
                Ok(true),
            );
        }

        drop(self.held[0].take());
        drop(self.held[1].take());
        drop(self.held[2].take());
        self.disk.test_flush_completions();
        context.expect_eq(self.mapping.prune_unused(None), 3);
        context.expect_eq(self.mapping.len(), 0);
        Ok(())
    }

    /// 排空每 CPU 页面缓存后依次按 10 阶与 0 阶分配并持有系统全部空闲物理页，验证伙伴系统与 PCP 空闲页归零且后续 0 阶分配返回 `None`。
    fn exhaust_pages(&mut self, context: &mut TestContext<'_>) -> TestResult {
        mm::drain_all_pages();
        for order in [10, 0] {
            loop {
                if self.pressure.len() == self.pressure.capacity() {
                    return Err(TestError("cache pressure handle capacity exhausted"));
                }
                let Some(pages) = mm::alloc_pages(order, false) else {
                    break;
                };
                self.pressure.push(pages);
            }
        }
        let stats = mm::stats();
        context.expect_eq(stats.buddy_free_pages + stats.pcp_cached_pages, 0);
        context.expect_eq(mm::alloc_pages(0, false).is_none(), true);
        Ok(())
    }

    /// 验证稀疏页索引 `FolioIndex` 在物理页耗尽（OOM）时的缺失报错与内存释放后的单飞恢复能力。
    fn reclaim_under_oom(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 耗尽系统物理页后对第 80 页调用 get_or_prepare，验证因无法分配新 Folio 而返回 NoMemory。
        let index = 80;
        self.exhaust_pages(context)?;
        context.expect_eq(
            matches!(
                self.mapping.get_or_prepare(index, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );

        // Step 2: 释放物理页压力后再次请求第 80 页，验证成功分配占位 Folio 并返回唯一 FolioRead 凭证，并发同键请求复用该占位页，完成块读取后数据校验通过。
        self.pressure.clear();
        let (folio, read) = self
            .mapping
            .get_or_prepare(index, None)
            .map_err(|_| TestError("sparse miss did not recover after releasing pressure"))?;
        let read = read.ok_or(TestError("recovered miss did not own its fill"))?;
        let (hit, duplicate) = self
            .mapping
            .get_or_prepare(index, None)
            .map_err(|_| TestError("recovered placeholder disappeared"))?;
        context.expect_eq(hit.phys_addr(), folio.phys_addr());
        context.expect_eq(duplicate.is_none(), true);
        context.expect_eq(folio.is_ready(), false);
        self.disk
            .load_folio(
                index * mm::PAGE_SIZE as u64,
                read,
                Vec::new_in(BioAllocator::Heap),
            )
            .map_err(|_| TestError("recovered original folio real read failed"))?;
        context.expect_eq(
            Returned::Folio(folio).valid(index * mm::PAGE_SIZE as u64),
            Ok(true),
        );
        Ok(())
    }

    /// 验证在系统物理页完全耗尽（OOM）期间，已驻留缓存的元数据块与文件页仍可零分配命中并正常读取，而未缓存的新键缺失请求则返回 `BlockError::NoMemory`。
    fn oom(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 预先加载第 64 号元数据块与第 64 号文件页并保持引用。
        self.held[0] = Some(Returned::Buffer(
            self.metadata
                .as_ref()
                .unwrap()
                .get(64, mm::PAGE_SIZE, None)
                .map_err(|_| TestError("OOM warm metadata read failed"))?,
        ));
        self.held[1] = Some(Returned::Folio(
            self.file_page(64)
                .map_err(|_| TestError("OOM warm file read failed"))?,
        ));
        self.disk.test_flush_completions();

        // Step 2: 耗尽系统全部空闲物理页，验证再次访问第 64 号元数据块与文件页均直接命中原物理页且内容可读，而访问未缓存的第 65 号块/页则返回 NoMemory。
        self.exhaust_pages(context)?;
        let buffer = self
            .metadata
            .as_ref()
            .unwrap()
            .get(64, mm::PAGE_SIZE, None)
            .map_err(|_| TestError("metadata hit allocated under OOM"))?;
        context.expect_eq(
            buffer.folio().phys_addr(),
            self.held[0].as_ref().unwrap().folio().phys_addr(),
        );
        context.expect_eq(
            buffer.with_read(|bytes| patterned(bytes, 1, OFFSET, mm::PAGE_SIZE)),
            Ok(true),
        );
        let (folio, read) = self
            .mapping
            .get_or_prepare(64, None)
            .map_err(|_| TestError("file hit allocated under OOM"))?;
        context.expect_eq(read.is_none(), true);
        context.expect_eq(
            folio.phys_addr(),
            self.held[1].as_ref().unwrap().folio().phys_addr(),
        );
        context.expect_eq(Returned::Folio(folio).valid(OFFSET), Ok(true));
        context.expect_eq(
            matches!(
                self.metadata.as_ref().unwrap().get(65, mm::PAGE_SIZE, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );
        context.expect_eq(
            matches!(
                self.mapping.get_or_prepare(65, None),
                Err(BlockError::NoMemory)
            ),
            true,
        );
        self.pressure.clear();
        Ok(())
    }

    /// 以只读方式读取主块设备 VirtQueue 可用环中的 `avail.idx` 字段，用于验证准备阶段失败时未向硬件环发布任何新描述符。
    fn avail_index(&self) -> u16 {
        // SAFETY:
        // 1. `self.avail_address` 在 `Owner::prepare` 中由块设备控制页基址加上经过 `RingLayout::new` 校验的 `avail_offset() + size_of::<u16>()` 计算得出，满足 `u16` 对齐且位于有效映射页内。
        // 2. 该方法仅在 `self.ready()` 成功之后、`self.disk.stop()` 调用之前的测试正文中执行，期间 `self.disk` 持续保活底层 VirtQueue 控制页映射，只读 `read_volatile` 不会与驱动写路径产生数据竞争或悬垂解引用。
        u16::from_le(unsafe { core::ptr::read_volatile(self.avail_address as *const u16) })
    }

    /// 验证块读取准备阶段遭遇物理页与 SLUB 堆双重耗尽（`PrepareOom`）时的原子回滚与等待者唤醒：不向块设备提交或通知任何请求，且所有同页阻塞等待者均被唤醒并收到 `BlockError::NoMemory`。
    fn prepare_oom(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 发布全部 3 个工作任务，等待 1 号与 2 号同键等待任务在占位 Folio 上进入 Blocked 阻塞态。
        self.publish_one();
        self.publish_one();
        self.publish_one();
        until("prepare-OOM file waiters did not block", || {
            self.observation.blocked(1) && self.observation.blocked(2)
        })?;
        // Step 2: 记录当前块设备提交计数与 VirtQueue avail.idx，放行 gate 门闩让 0 号任务在物理页与堆双重耗尽下调用 Disk::load_folio。
        let before = self.disk.test_snapshot();
        let avail = self.avail_index();
        self.observation.gate.complete();
        self.join_tasks()?;

        // Step 3: 验证物理页与堆耗尽探测均为 true，3 个任务全部收到 NoMemory 错误，占位 Folio 处于已完结错误态，且块设备提交数、通知数、在途数与 avail.idx 均未发生任何变化。
        let evidence = self
            .observation
            .oom_evidence
            .lock()
            .as_ref()
            .copied()
            .ok_or(TestError("prepare-OOM allocator evidence absent"))?;
        context.expect_eq(evidence, (true, true));
        for result in &*self.observation.results.lock() {
            context.expect_eq(matches!(result, Some(Err(BlockError::NoMemory))), true);
        }
        let folio = self.held[0].as_ref().unwrap().folio();
        context.expect_eq(folio.is_ready(), true);
        context.expect_eq(folio.wait_ready(), Err(BlockError::NoMemory));
        context.expect_eq(
            folio.copy_to(0, &mut [0_u8; mm::PAGE_SIZE]),
            Err(BlockError::NoMemory),
        );
        let (hit, read) = self
            .mapping
            .get_or_prepare(64, None)
            .map_err(|_| TestError("prepare-OOM live placeholder disappeared"))?;
        context.expect_eq(hit.phys_addr(), folio.phys_addr());
        context.expect_eq(read.is_none(), true);
        context.expect_eq(hit.wait_ready(), Err(BlockError::NoMemory));
        let after = self.disk.test_snapshot();
        context.expect_eq(after.submitted, before.submitted);
        context.expect_eq(after.notifications, before.notifications);
        context.expect_eq(after.inflight, before.inflight);
        context.expect_eq(self.avail_index(), avail);
        context.diagnostic(format_args!(
            "cache-prepare-OOM real-page-NULL={} real-heap-NULL={} native-Blocked-waiters=2 original-PFN={} submitted-before={} submitted-after={} avail-before={} avail-after={} hardware-EIO-covered=0",
            evidence.0, evidence.1, folio.phys_addr(), before.submitted, after.submitted, avail, self.avail_index(),
        ));
        Ok(())
    }

    /// 验证同一物理页上 1KiB、2KiB、4KiB 重叠元数据视图的按扇区加载与脏数据保留语义：后续读取更大范围视图时仅从磁盘补齐未就绪扇区，绝不覆盖同页内已由 CPU 修改的脏扇区。
    fn overlapping_updates(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        if 1024 < self.disk.limits().logical_block_size {
            return Err(TestError("overlap case requires logical block <= 1KiB"));
        }
        // Step 1: 读取页偏移 1024..2048 处的 1KiB 子视图（块号 1），验证仅该 1KiB 扇区范围就绪而其余扇区为 NotReady；将其修改为 0xa5 后验证脏扇区位图为 0b0000_1100，且只读凭证下的修改被拒绝。
        let cache = self.metadata.as_ref().unwrap();
        let claim = Disk::claim_fs(&BlockView::whole(self.disk.clone()), AccessMode::ReadWrite)
            .map_err(|_| TestError("private metadata RW claim failed"))?;
        let same = MetadataCache::shared(&claim, 8)
            .map_err(|_| TestError("shared same-Disk metadata cache disappeared"))?;
        context.expect_eq(MetadataCache::shared(&claim, 7).is_err(), true);
        let tiny = cache
            .get(1, 1024, None)
            .map_err(|_| TestError("private 1KiB metadata read failed"))?;
        let pfn = tiny.folio().phys_addr();
        context.expect_eq(tiny.folio().range_ready(1024, 1024), Ok(()));
        context.expect_eq(tiny.folio().range_ready(0, 1024), Err(BlockError::NotReady));
        context.expect_eq(
            tiny.folio().range_ready(2048, 2048),
            Err(BlockError::NotReady),
        );
        context.expect_eq(tiny.folio().is_ready(), false);
        tiny.update(&claim, |bytes| bytes.fill(0xa5))
            .map_err(|_| TestError("private tiny update failed"))?;
        context.expect_eq(tiny.folio().dirty_sectors(), 0b0000_1100);
        let ro = Disk::claim_fs(&BlockView::whole(self.disk.clone()), AccessMode::ReadOnly)
            .map_err(|_| TestError("private metadata RO claim failed"))?;
        context.expect_eq(
            tiny.update(&ro, |bytes| bytes.fill(0xee)),
            Err(BlockError::ReadOnly),
        );

        // Step 2: 读取页偏移 0..2048 处的 2KiB 视图（块号 0），验证复用同一物理页、前 1KiB 从磁盘补齐为 0 而后 1KiB 保留已修改的 0xa5；随后将前 512 字节修改为 0x19。
        let two = same
            .get(0, 2048, None)
            .map_err(|_| TestError("private 2KiB metadata read failed"))?;
        context.expect_eq(two.folio().phys_addr(), pfn);
        context.expect_eq(
            two.with_read(|bytes| {
                bytes[..1024].iter().all(|b| *b == 0) && bytes[1024..].iter().all(|b| *b == 0xa5)
            }),
            Ok(true),
        );
        two.update(&claim, |bytes| bytes[..512].fill(0x19))
            .map_err(|_| TestError("private 2KiB metadata update failed"))?;

        // Step 3: 读取整页 4KiB 视图，验证后半页 2048..4096 扇区补齐后，0..512（0x19）与 1024..2048（0xa5）两处脏数据均完好保留，脏扇区位图更新为 0b0000_1111，且释放视图句柄后脏页不会被 prune_unused 淘汰。
        let super_view = cache
            .get(0, mm::PAGE_SIZE, None)
            .map_err(|_| TestError("private super metadata read failed"))?;
        context.expect_eq(super_view.folio().phys_addr(), pfn);
        context.expect_eq(
            super_view.with_read(|bytes| {
                bytes[..512].iter().all(|b| *b == 0x19)
                    && bytes[512..1024].iter().all(|b| *b == 0)
                    && bytes[1024..2048].iter().all(|b| *b == 0xa5)
                    && bytes[2048..].iter().all(|b| *b == 0)
            }),
            Ok(true),
        );
        context.expect_eq(
            tiny.with_read(|bytes| bytes.iter().all(|b| *b == 0xa5)),
            Ok(true),
        );
        context.expect_eq(super_view.folio().dirty_sectors(), 0b0000_1111);
        context.expect_eq(super_view.folio().writeback_sectors(), 0);

        drop(super_view);
        drop(two);
        drop(tiny);
        drop(same);
        context.expect_eq(cache.prune_unused(), 0);
        context.expect_eq(cache.len(), 1);
        context.diagnostic(format_args!(
            "metadata-overlap private-scratch=1 physical-page=0 PFN={} windows=1/2/4KiB partial-not-ready=1 dirty-retained-after-read=1 dirty-prune-refused=1 CPU-content-only=1 file-write-fsync-covered=0",
            pfn,
        ));
        Ok(())
    }

    /// 验证同一物理页内两条 256 字节 ext4 Inode 记录的并发互斥更新与 CRC32C 校验保持：从只读 ext4 根盘读取两条有效 Inode 原始记录复制到私有测试盘的同一 4KiB 元数据页中，由两个内核任务并发更新各自记录并校验内容睡眠锁的阻塞互斥、Extent 区域保持以及更新后的完整 CRC32C 校验和。
    fn concurrent_records(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.ready()?;
        // Step 1: 以只读模式打开 ext4 根卷，定位根目录 Inode（2 号）与 "/pattern" 文件 Inode，通过块组描述符与 Inode 表读取两条 256 字节原始 Inode 记录并校验初始 CRC32C。
        let volume = Ext4Volume::open(
            BlockView::whole(self.extra.as_ref().unwrap().clone()),
            AccessMode::ReadOnly,
            8,
        )
        .map_err(|_| TestError("record case real root open failed"))?;
        let g = volume.geometry();
        if g.inode_size != 256 || g.features_ro_compat & codec::METADATA_CSUM == 0 {
            return Err(TestError(
                "record case requires real 256-byte checksummed inode records",
            ));
        }
        let root = volume
            .load_inode(2, None)
            .map_err(|_| TestError("record root inode failed"))?;
        let pattern = volume
            .lookup(&InodeMetadata::from_data(&root), &root.block, b"pattern")
            .map_err(|_| TestError("record pattern lookup failed"))?
            .ok_or(TestError(
                "record case requires existing root pattern fixture",
            ))?;
        let source_cache = MetadataCache::shared(volume.claim(), 8)
            .map_err(|_| TestError("record source metadata cache unavailable"))?;
        let numbers = [root.number, pattern];
        let mut original = [[0u8; 256]; 2];
        let mut before = [InodeMetadata::from_data(&root); 2];
        for index in 0..2 {
            let number = numbers[index];
            let group = (number - 1) / g.inodes_per_group;
            let descriptor = group * g.descriptor_size as u64;
            let block = g.first_data_block + 1 + descriptor / g.block_size as u64;
            let offset = (descriptor % g.block_size as u64) as usize;
            // 块组描述符位于保留元数据区，直接通过卷关联的 MetadataCache 读取并解码 Inode 表起始块号。
            let table = source_cache
                .get(block, g.block_size, None)
                .map_err(|_| TestError("record group descriptor read failed"))?
                .with_read(|bytes| {
                    codec::decode_group(g, group, codec::bytes(bytes, offset, g.descriptor_size)?)
                })
                .map_err(|_| TestError("record descriptor view failed"))?
                .map_err(|_| TestError("record descriptor decode failed"))?;
            let byte = ((number - 1) % g.inodes_per_group) * g.inode_size as u64;
            let buffer = volume
                .metadata_block(table + byte / g.block_size as u64)
                .map_err(|_| TestError("record inode table read failed"))?;
            buffer
                .with_read(|bytes| {
                    original[index].copy_from_slice(
                        &bytes[(byte % g.block_size as u64) as usize
                            ..(byte % g.block_size as u64) as usize + 256],
                    );
                })
                .map_err(|_| TestError("record source copy failed"))?;
            let decoded = codec::decode_inode(g, number, &original[index])
                .map_err(|_| TestError("record source checksum failed"))?;
            before[index] = InodeMetadata::from_data(&decoded);
        }

        // Step 2: 构造两条 Inode 的目标元数据更新（修改权限位、UID、GID 与访问时间戳），并复制原始记录到私有测试盘第 0 页的 0..256 与 256..512 字节区间。
        let mut after = before;
        for index in 0..2 {
            after[index].mode ^= 0o020;
            after[index].uid = 0x1234_abcd + index as u32;
            after[index].gid = 0xabcd_1234 + index as u32;
            after[index].atime.seconds += 7 + index as i64;
            codec::validate_inode_metadata_update(&before[index], &after[index])
                .map_err(|_| TestError("record expected metadata not representable"))?;
        }

        let claim = Disk::claim_fs(&BlockView::whole(self.disk.clone()), AccessMode::ReadWrite)
            .map_err(|_| TestError("private record RW claim failed"))?;
        let cache = self.metadata.as_ref().unwrap().clone();
        let buffer = cache
            .get(0, mm::PAGE_SIZE, None)
            .map_err(|_| TestError("private record scratch read failed"))?;
        buffer
            .update(&claim, |bytes| {
                bytes[..256].copy_from_slice(&original[0]);
                bytes[256..512].copy_from_slice(&original[1]);
            })
            .map_err(|_| TestError("private record initialization failed"))?;

        // Step 3: 预创建两个分别更新 0..256 与 256..512 记录的内核任务；主线程先持有该 Folio 的内容睡眠锁（lock_content），发布两个任务并确认它们均因等待同一把内容锁而进入 State::Blocked，随后释放内容锁让两个任务依次完成补丁更新与重算 CRC32C。
        for index in 0..2 {
            let target = buffer.clone();
            let task_claim = claim.clone();
            let task_volume = volume.clone();
            let observer = self.observation.clone();
            let old = before[index];
            let next = after[index];
            self.prepared.push(
                task::prepare_kernel_on(index % cpu::count(), move || {
                    observer.tasks.lock()[index] = Some(task::current_task());
                    observer.entered[index].store(true, Ordering::Release);
                    match target.update(&task_claim, |bytes| {
                        codec::patch_inode_metadata(
                            task_volume.geometry(),
                            &mut bytes[index * 256..(index + 1) * 256],
                            &old,
                            &next,
                        )
                    }) {
                        Ok(Ok(())) => 0,
                        _ => 1,
                    }
                })
                .map_err(|_| TestError("record update Task preparation failed"))?,
            );
        }

        let content = buffer.folio().lock_content();
        self.publish_one();
        self.publish_one();
        let blocked = until(
            "record update tasks did not block on common content",
            || {
                self.observation.entered[0].load(Ordering::Acquire)
                    && self.observation.entered[1].load(Ordering::Acquire)
                    && self.observation.blocked(0)
                    && self.observation.blocked(1)
            },
        );
        drop(content);
        blocked?;
        self.join_tasks()?;

        // Step 4: 重新读取该 4KiB 共享页，使用 ext4 编解码器验证两条记录均解码为更新后的元数据、块映射区域（偏移 40..100）未被破坏、翻转末尾校验字节会被 CRC32C 拒绝，且只读源卷上的原始 Inode 未发生任何改变。
        let (decoded, roots_preserved, corrupt_rejected) = buffer
            .with_read(|bytes| {
                let mut decoded = [Err(FsError::Corrupt); 2];
                let mut roots_preserved = [false; 2];
                let mut corrupt_rejected = [false; 2];
                for index in 0..2 {
                    let record = &bytes[index * 256..(index + 1) * 256];
                    decoded[index] = codec::decode_inode(g, numbers[index], record)
                        .map(|inode| InodeMetadata::from_data(&inode));
                    roots_preserved[index] = record[40..100] == original[index][40..100];
                    let mut corrupt = [0u8; 256];
                    corrupt.copy_from_slice(record);
                    corrupt[255] ^= 1;
                    corrupt_rejected[index] =
                        codec::decode_inode(g, numbers[index], &corrupt).is_err();
                }
                (decoded, roots_preserved, corrupt_rejected)
            })
            .map_err(|_| TestError("record final common view failed"))?;
        for index in 0..2 {
            context.expect_eq(decoded[index], Ok(after[index]));
            context.expect_eq(roots_preserved[index], true);
            context.expect_eq(corrupt_rejected[index], true);
        }

        for index in 0..2 {
            let unchanged = volume
                .load_inode(numbers[index], None)
                .map_err(|_| TestError("record original root reread failed"))?;
            context.expect_eq(InodeMetadata::from_data(&unchanged), before[index]);
        }
        let pfn = buffer.folio().phys_addr();
        drop(buffer);
        context.expect_eq(cache.prune_unused(), 0);
        context.diagnostic(format_args!(
            "metadata-records private-scratch=1 PFN={} native-blocked=2 shared-page-records=2 production-codec=1 full-CRC=1 root-unchanged=1 file-write-fsync-covered=0",
            pfn,
        ));
        Ok(())
    }

    /// 无论测试主体成功或中途失败均执行的统一静默化清理流程：取消未发布任务、唤醒门闩、释放内存压力、恢复并排空块设备完成回调、等待全部工作任务退出内核栈、丢弃私有测试盘上的内存态脏页、停止块设备并同步关闭工作队列。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        // Step 1: 置位取消标志并触发门闩，丢弃未发布的任务与内存压力句柄，恢复主/副块设备的完成回调分发。
        self.observation.cancelled.store(true, Ordering::Release);
        self.observation.gate.complete();
        drop(core::mem::take(&mut self.prepared));
        self.pressure.clear();
        let pressure = self.observation.pressure.lock().take();
        drop(pressure);
        self.disk.test_resume_completions();
        if let Some(extra) = &self.extra {
            extra.test_resume_completions();
        }

        // Step 2: 等待全部已发布的工作任务退出内核栈并回收 JoinHandle，排空主/副块设备工作队列中的剩余完成项。
        if let Some(join) = self.joins.first() {
            until("cache read owner did not leave stack", || {
                join.try_join().is_some()
            })?;
        }
        let read = self.observation.read.lock().take();
        drop(read);
        self.join_tasks()?;
        self.disk.test_flush_completions();
        if let Some(extra) = &self.extra {
            extra.test_flush_completions();
        }

        // Step 3: 清空主线程与观察站持有的全部 BufferRef / FolioRef 引用；若当前场景在私有测试盘上修改了元数据页，则安全丢弃内存态脏页并释放缓存实例。
        self.held.fill_with(|| None);
        let results =
            core::mem::replace(&mut *self.observation.results.lock(), [const { None }; 3]);
        drop(results);
        if self.private_metadata {
            // SAFETY:
            // 1. `private_metadata` 仅在 `MutableMetadata` 与 `MetadataRecords` 两个使用私有测试盘的场景中为 `true`，这两个场景仅在缓存页上做内存态修改，未挂载可写文件系统或发起回写。
            // 2. 上方已完成全部并发工作任务的 `join_tasks()` 与块设备 `test_flush_completions()`，且清空了 `self.held` 与 `self.observation.results`，保证没有任何外部任务或句柄仍持有该 `MetadataCache` 的页面引用。
            unsafe { self.metadata.as_ref().unwrap().discard_private() }
                .map_err(|_| TestError("private metadata detach retained a live owner"))?;
        }
        drop(self.metadata.take());
        drop(self.extra_metadata.take());

        // Step 4: 停止主/副块设备硬件队列，同步关闭完成工作队列，并断言设备在途请求数归零。
        self.disk
            .stop()
            .map_err(|_| TestError("cache stop cannot converge; retaining original DMA owner"))?;
        if let Some(extra) = &self.extra {
            extra
                .stop()
                .map_err(|_| TestError("extra cache stop cannot converge"))?;
        }
        self.queue.close_sync();
        context.expect_eq(self.disk.test_snapshot().inflight, 0);
        if let Some(extra) = &self.extra {
            context.expect_eq(extra.test_snapshot().inflight, 0);
        }
        Ok(())
    }

    /// 在 `quiesce` 已完成全部异步任务、中断与工作队列清理后，销毁 `Owner` 释放全部缓存与设备句柄，并校验系统物理内存六维统计完全恢复至初始基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let baseline = self.baseline;
        drop(self);
        recovered(context, baseline)
    }
}

macro_rules! fixture {
    ($name:ident, $scenario:ident) => {
        /// 针对指定缓存测试场景封装 `Owner` 的 `ktest` 夹具新类型包装。
        struct $name(Owner);
        impl Fixture<DeviceInventory> for $name {
            /// 从套件级共享设备清单初始化当前测试场景专属的块设备、缓存与并发任务资源。
            fn set_up(
                _: &mut TestContext<'_>,
                inventory: &DeviceInventory,
            ) -> Result<Self, TestError> {
                Owner::prepare(Scenario::$scenario, inventory).map(Self)
            }
            /// 在测试主体结束后执行异步任务回收、缓存清理与块设备停止；若无法安全收敛则返回错误以保留现场。
            fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
                self.0.quiesce(context)
            }
            /// 在全部异步活动静止后销毁夹具，并核对伙伴系统、PCP 与 SLUB 六维内存统计恢复至测试前基线。
            fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
                self.0.tear_down(context)
            }
        }
    };
}
fixture!(Views, Views);
fixture!(MetadataFlight, MetadataFlight);
fixture!(FileFlight, FileFlight);
fixture!(Independent, Independent);
fixture!(MetadataIndependent, MetadataIndependent);
fixture!(Failure, Failure);
fixture!(Bounded, Bounded);
fixture!(Oom, Oom);
fixture!(PrepareOom, PrepareOom);
fixture!(MutableMetadata, MutableMetadata);
fixture!(MetadataRecords, MetadataRecords);

#[ktest::suite(fixture = Pages, context = "Thread", tags("cache", "folio"), timeout_ms = 15000)]
mod folio {
    use super::*;
    /// - **测试目标**：验证 `FolioRef` 的单页（0 阶）物理地址保真性、克隆共享与最后一个强引用释放时的物理页回收，以及对高阶（1 阶）物理页块的拒绝与原样退回。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 夹具已分配一个 0 阶 `FolioRef` 与一个 1 阶 `OwnedPages`；
    ///   2. When 克隆 0 阶 `FolioRef` 后依次释放两个引用，并将填充了 `0xa7` 的 1 阶页块传入 `FolioRef::from_pages`；
    ///   3. Then 两个 `FolioRef` 物理地址一致且仅在最后一个强引用释放后使伙伴系统空闲页加 1；1 阶页块被拒绝并原样退回、内容保持 `0xa7`，释放后空闲页再加 2。
    #[test]
    fn original_page_lifetime(context: &mut TestContext<'_>, fixture: &mut Pages) -> TestResult {
        fixture.identity(context)
    }

    /// - **测试目标**：验证 `FolioIndex` 稀疏页索引在升序、降序与大跨度稀疏键（含 `u64::MAX`）三种插入顺序下的 AVL 平衡性、物理页保活、OOM 期间缓存命中以及按外部引用计数修剪（`prune_unused`）。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 分别按升序（`0..96`）、降序（`95..=0`）和大跨度稀疏键（`2^40 + i*8191` 及 `u64::MAX`）三轮向 `FolioIndex` 插入 96 个页面并填充各自序号字节；
    ///   2. When 在首轮耗尽系统全部物理页与 SLUB 堆后查询已存在的第 48 号键，随后遍历全部 96 个键校验物理地址与字节内容，最后持有首、中、尾 3 个页面的外部引用并先后调用两次 `prune_unused`；
    ///   3. Then OOM 期间查询已缓存键无需分配内存即可命中原物理页；全部 96 个键均命中原物理地址且首字节匹配；首次修剪回收未被引用的 93 页，释放剩余 3 个外部引用后第二次修剪回收最后 3 页。
    #[test]
    fn sparse_index_insert_hit_and_prune(
        context: &mut TestContext<'_>,
        _: &mut Pages,
    ) -> TestResult {
        // Step 1: 依次按升序、降序与大跨度稀疏键执行三轮测试，每轮插入 96 个 Folio 并通过 FolioRead 完成字节初始化。
        for pass in 0..3 {
            let mapping = FolioIndex::new(None, false);
            let key = |index: usize| match pass {
                0 => index as u64,
                1 => (95 - index) as u64,
                _ if index == 95 => u64::MAX,
                _ => (1u64 << 40) + index as u64 * 8191,
            };
            let mut addresses = [0usize; 96];
            for (index, address) in addresses.iter_mut().enumerate() {
                let (folio, read) = mapping
                    .get_or_prepare(key(index), None)
                    .map_err(|_| TestError("sparse AVL insertion failed"))?;
                let mut read = read.ok_or(TestError("sparse miss did not own initialization"))?;
                for byte in read.bytes_mut() {
                    byte.write(index as u8);
                }
                // SAFETY:
                // 1. 当前 `read` 是刚刚由 `mapping.get_or_prepare` 为新建占位页返回的唯一 `FolioRead` 凭证，从未提交给任何块设备 DMA。
                // 2. 上方循环已将该页全部 `PAGE_SIZE` 字节逐一写入初始化，调用 `complete` 后不再通过 `read` 访问可写切片，满足页面就绪发布的内存安全要求。
                unsafe {
                    read.complete(IoResult {
                        completed: mm::PAGE_SIZE,
                        error: None,
                    });
                }
                *address = folio.phys_addr();
            }

            // Step 2: 在第一轮（pass == 0）中耗尽系统物理页与 SLUB 堆，验证在 OOM 状态下查询已驻留索引的键仍能零分配命中已有物理页。
            if pass == 0 {
                let mut pressure = HeapPressure::prepare()?;
                let evidence = pressure.exhaust();
                let result = mapping.get_or_prepare(key(48), None);
                drop(pressure);
                context.expect_eq(evidence, (true, true));
                let (hit, read) =
                    result.map_err(|_| TestError("sparse hit allocated under real OOM"))?;
                context.expect_eq(hit.phys_addr(), addresses[48]);
                context.expect_eq(read.is_none(), true);
            }
            // Step 3: 遍历全部 96 个键验证物理地址与页面首字节内容完好，随后验证外部持有 3 个 FolioRef 时 prune_unused 仅回收其余 93 页，释放后回收剩余 3 页。
            for (index, address) in addresses.iter().enumerate() {
                let (hit, read) = mapping
                    .get_or_prepare(key(index), None)
                    .map_err(|_| TestError("sparse AVL hit lost original page"))?;
                context.expect_eq(hit.phys_addr(), *address);
                context.expect_eq(read.is_none(), true);
                let mut byte = [0u8; 1];
                context.expect_eq(hit.copy_to(0, &mut byte), Ok(()));
                context.expect_eq(byte[0], index as u8);
            }

            let first = mapping
                .get_or_prepare(key(0), None)
                .map_err(|_| TestError("sparse first pin failed"))?
                .0;
            let middle = mapping
                .get_or_prepare(key(48), None)
                .map_err(|_| TestError("sparse middle pin failed"))?
                .0;
            let last = mapping
                .get_or_prepare(key(95), None)
                .map_err(|_| TestError("sparse last pin failed"))?
                .0;
            context.expect_eq(mapping.prune_unused(None), 93);
            drop(first);
            drop(middle);
            drop(last);
            context.expect_eq(mapping.prune_unused(None), 3);
        }
        context.diagnostic(format_args!(
            "sparse-index orders=ascending/descending/sparse pages-per-order=96 real-OOM-hit=1 same-PFN-and-content=1 prune-live=93 prune-final=3"
        ));
        Ok(())
    }
}

#[ktest::suite(fixture = Views, suite_resources = DeviceInventory, context = "Thread", tags("cache", "metadata"), resources("root", "extra"), timeout_ms = 15000)]
mod metadata_views {
    use super::*;
    /// - **测试目标**：验证 `MetadataCache` 在两个独立块设备（`root` 与 `extra`）上读取 1KiB、2KiB、4KiB 子页视图时的页内偏移计算、数据图样完整性与跨设备物理页隔离。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 已启动两块填充了不同测试图样标签（`tag = 1` 与 `tag = 2`）的 VirtIO 块设备及其各自的 `MetadataCache`；
    ///   2. When 依次以 1024、2048、4096 字节粒度读取两块设备上的第 33 号逻辑块视图；
    ///   3. Then 每次返回的 `BufferRef` 长度与页内偏移均符合预期，字节内容分别匹配两块设备的图样公式，且两块设备的同号视图底层位于不同物理页。
    #[test]
    fn complete_pattern_1_2_4k(context: &mut TestContext<'_>, fixture: &mut Views) -> TestResult {
        fixture.0.views(context)
    }
}

#[ktest::suite(fixture = MetadataFlight, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "metadata", "B14"), resources("root"), timeout_ms = 15000)]
mod metadata_singleflight {
    use super::*;
    /// - **测试目标**：验证 `MetadataCache` 在多核并发访问同一未缓存块号时的单飞（Singleflight）合并加载语义。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 暂停块设备完成回调分发，先后在两个 CPU 上发布读取第 64 号元数据块的工作任务；
    ///   2. When 首个任务向硬件提交块读取并阻塞，第二个同键任务随后进入缓存查询并同样进入 `State::Blocked`，接着恢复块设备完成回调；
    ///   3. Then 整个过程中块设备仅产生 1 次硬件提交（`inflight == 1`），完成后两个任务被唤醒并获取到物理地址相同的 `BufferRef` 且数据校验通过。
    #[test]
    fn blocked_waiter_one_submit(
        context: &mut TestContext<'_>,
        fixture: &mut MetadataFlight,
    ) -> TestResult {
        fixture.0.singleflight(context)
    }
}

#[ktest::suite(fixture = FileFlight, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "file", "B14"), resources("root"), timeout_ms = 15000)]
mod file_singleflight {
    use super::*;
    /// - **测试目标**：验证文件页稀疏索引 `FolioIndex` 在多核并发请求同一缺失页索引时的单飞（Singleflight）合并加载与物理页复用。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 暂停块设备完成回调分发，先后在两个 CPU 上发布读取第 64 号文件页的工作任务；
    ///   2. When 0 号任务获得唯一 `FolioRead` 凭证并发起 `Disk::load_folio` 阻塞，1 号任务命中同一未就绪 `FolioRef` 并在 `wait_ready` 上进入 `State::Blocked`，随后恢复完成回调；
    ///   3. Then 底层块设备仅收到 1 次 I/O 提交，两个任务完成后持有相同物理页帧号（PFN）的 `FolioRef` 且页面内容符合第 64 页测试图样。
    #[test]
    fn blocked_waiter_one_submit(
        context: &mut TestContext<'_>,
        fixture: &mut FileFlight,
    ) -> TestResult {
        fixture.0.singleflight(context)
    }
}

#[ktest::suite(fixture = MetadataIndependent, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "metadata", "B15"), resources("root"), timeout_ms = 15000)]
mod metadata_parallel {
    use super::*;
    /// - **测试目标**：验证 `MetadataCache` 在并发读取不同块号时能够同时向块设备发起多个独立在途（Inflight）请求。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 暂停块设备完成回调分发，同时发布两个分别读取第 64 号与第 65 号元数据块的工作任务；
    ///   2. When 等待硬件 VirtQueue 处理完 2 个请求且两个工作任务均进入 `State::Blocked` 后，恢复块设备完成回调；
    ///   3. Then 暂停窗口内块设备提交增量与在途请求数均为 2，完成后两个任务各自获得不同物理页上的 `BufferRef` 且内容分别匹配第 64、65 页图样。
    #[test]
    fn independent_keys_two_inflight(
        context: &mut TestContext<'_>,
        fixture: &mut MetadataIndependent,
    ) -> TestResult {
        fixture.0.independent(context)
    }
}

#[ktest::suite(fixture = Independent, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "file", "B15"), resources("root"), timeout_ms = 15000)]
mod file_parallel {
    use super::*;
    /// - **测试目标**：验证文件页稀疏索引 `FolioIndex` 在并发读取不同页索引时支持多请求并行在途加载。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 暂停块设备完成回调分发，同时发布两个分别读取第 64 号与第 65 号文件页的工作任务；
    ///   2. When 等待硬件 VirtQueue 的 `used.idx` 推进 2 且两个工作任务均进入 `State::Blocked` 后，恢复完成回调；
    ///   3. Then 暂停窗口内块设备同时保持 2 个在途请求，完成后两个任务获得不同物理地址的 `FolioRef` 且各自页面内容校验通过。
    #[test]
    fn independent_keys_two_inflight(
        context: &mut TestContext<'_>,
        fixture: &mut Independent,
    ) -> TestResult {
        fixture.0.independent(context)
    }
}

#[ktest::suite(fixture = Failure, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2, tags("cache", "file", "QueueStopped", "B11", "B14"), resources("root"), timeout_ms = 15000)]
mod stopped_failure {
    use super::*;
    /// - **测试目标**：验证当块设备队列已停止（`BlockError::QueueStopped`）时，文件页加载失败能够唤醒全部同页阻塞等待者并一致传播 `QueueStopped` 错误。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 预先在 `FolioIndex` 中准备第 64 页占位 `Folio`，让 1 号和 2 号工作任务在 `wait_ready` 上进入 `State::Blocked`，0 号加载任务在门闩处等待；
    ///   2. When 调用 `disk.stop()` 停止块设备队列后放行门闩，让 0 号任务调用 `Disk::load_folio`；
    ///   3. Then 块设备未增加任何硬件提交，0 号任务与两个阻塞等待任务全部返回 `Err(BlockError::QueueStopped)`，且后续对该 `Folio` 的 `wait_ready` 与 `copy_to` 均返回 `QueueStopped`。
    #[test]
    fn stopped_read_wakes_all(context: &mut TestContext<'_>, fixture: &mut Failure) -> TestResult {
        fixture.0.failure(context)
    }
}

#[ktest::suite(fixture = Bounded, suite_resources = DeviceInventory, context = "Thread", tags("cache", "prune", "B23"), resources("root"), timeout_ms = 15000)]
mod bounded {
    use super::*;
    /// - **测试目标**：验证固定容量上限（2 页）的 `MetadataCache` 在全部页面均被外部持有时拒绝新缺失请求（返回 `NoMemory`）、释放后可淘汰干净旧页，以及无容量上限的 `FolioIndex` 可同时保有超过 2 个活跃文件页。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 容量为 2 的 `MetadataCache` 与无容量上限的 `FolioIndex`；
    ///   2. When 在 `MetadataCache` 中同时持有第 64、65 块时请求第 66 块，随后释放第 64 块再请求第 66 块，最后在 `FolioIndex` 中同时加载并持有第 64、65、66 三个文件页；
    ///   3. Then `MetadataCache` 满额且无空闲页可淘汰时返回 `Err(BlockError::NoMemory)`，释放一页后成功淘汰并加载第 66 块；`FolioIndex` 则可同时持有全部 3 个文件页并在引用全部释放后通过 `prune_unused` 悉数回收。
    #[test]
    fn clean_prune_and_live_no_memory(
        context: &mut TestContext<'_>,
        fixture: &mut Bounded,
    ) -> TestResult {
        fixture.0.bounded(context)
    }
}

#[ktest::suite(fixture = Oom, suite_resources = DeviceInventory, context = "Thread", tags("cache", "oom", "B23"), resources("root"), timeout_ms = 15000)]
mod oom {
    use super::*;
    /// - **测试目标**：验证在系统物理页完全耗尽（OOM）时，已驻留 `MetadataCache` 与 `FolioIndex` 的热缓存项仍可正常命中并读取原数据，而未缓存的新键缺失请求则返回 `BlockError::NoMemory`。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 预先读取并持有第 64 号元数据块与第 64 号文件页，随后抽干伙伴系统与 PCP 中的全部空闲物理页；
    ///   2. When 在 0 空闲页状态下再次请求已缓存的第 64 号元数据块/文件页，以及未缓存的第 65 号元数据块/文件页；
    ///   3. Then 第 64 号元数据块与文件页均直接命中原物理地址且数据校验通过，第 65 号缺失请求均返回 `Err(BlockError::NoMemory)`。
    #[test]
    fn existing_hits_read_under_real_oom(
        context: &mut TestContext<'_>,
        fixture: &mut Oom,
    ) -> TestResult {
        fixture.0.oom(context)
    }

    /// - **测试目标**：验证 `FolioIndex` 在物理页耗尽期间缺失失败不会残留损坏状态，且在内存压力解除后能够正常分配占位页并完成单飞块读取。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 抽干系统全部空闲物理页后对第 80 号文件页调用 `get_or_prepare` 触发 `NoMemory`；
    ///   2. When 释放物理页压力后再次对第 80 号文件页连续调用两次 `get_or_prepare`，并使用首次调用返回的 `FolioRead` 执行 `Disk::load_folio`；
    ///   3. Then 压力解除后的首次调用成功分配占位页并获得唯一 `FolioRead` 凭证，第二次并发调用命中同一占位页（`read.is_none()`），块读取完成后页面数据校验通过。
    #[test]
    fn sparse_miss_recovers_after_real_oom(
        context: &mut TestContext<'_>,
        fixture: &mut Oom,
    ) -> TestResult {
        fixture.0.reclaim_under_oom(context)
    }
}

#[ktest::suite(fixture = PrepareOom, suite_resources = DeviceInventory, context = "Thread", min_cpus = 2,
    tags("cache", "file", "prepare-oom", "B14"), resources("root"), timeout_ms = 60000)]
mod prepare_oom {
    use super::*;
    /// - **测试目标**：验证 `Disk::load_folio` 在准备块请求阶段遭遇物理页与 SLUB 堆双重耗尽（OOM）时，不会向硬件环发布任何请求，且能唤醒全部同页阻塞等待者并返回 `BlockError::NoMemory`。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 第 64 页占位 `Folio` 上已有 2 个工作任务进入 `State::Blocked` 等待就绪，0 号任务持有唯一 `FolioRead` 凭证在门闩处等待；
    ///   2. When 放行门闩后 0 号任务通过 `HeapPressure::exhaust` 抽干全部物理页与 SLUB 堆对象，随即调用 `Disk::load_folio`；
    ///   3. Then 分配器耗尽探测证据为 `(true, true)`，块设备提交计数、通知计数、在途计数与 `avail.idx` 均保持不变，0 号加载者与 2 个等待者全部返回 `Err(BlockError::NoMemory)`。
    #[test]
    fn unsubmitted_read_wakes_all(
        context: &mut TestContext<'_>,
        fixture: &mut PrepareOom,
    ) -> TestResult {
        fixture.0.prepare_oom(context)
    }
}

#[ktest::suite(fixture = MutableMetadata, suite_resources = DeviceInventory, context = "Thread",
    tags("cache", "metadata"), resources("scratch"), timeout_ms = 15000)]
mod mutable_metadata {
    use super::*;
    /// - **测试目标**：验证同一物理页上 1KiB、2KiB、4KiB 重叠元数据视图的按需扇区读取与脏扇区保护机制，确保后续扩大范围的块读取不会覆盖同页内已由 CPU 修改的脏扇区。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 在私有测试盘上先读取页内偏移 `1024..2048` 的 1KiB 视图并填充为 `0xa5`（标记脏扇区 `0b0000_1100`）；
    ///   2. When 依次读取同一物理页上偏移 `0..2048` 的 2KiB 视图（将前 512 字节修改为 `0x19`）以及偏移 `0..4096` 的整页 4KiB 视图，最后释放全部视图句柄并调用 `prune_unused()`；
    ///   3. Then 三种粒度视图共享同一物理页，每次扩大读取仅从磁盘补齐尚未就绪的扇区（全 0），`0..512`（`0x19`）与 `1024..2048`（`0xa5`）的脏数据完好保留，脏扇区位图为 `0b0000_1111`，且包含脏扇区的元数据页拒绝被 `prune_unused` 回收。
    #[test]
    fn overlapping_partial_views_preserve_updates(
        context: &mut TestContext<'_>,
        fixture: &mut MutableMetadata,
    ) -> TestResult {
        fixture.0.overlapping_updates(context)
    }
}

#[ktest::suite(fixture = MetadataRecords, suite_resources = DeviceInventory, context = "Thread",
    tags("cache", "metadata"), resources("root", "scratch"), timeout_ms = 15000)]
mod metadata_records {
    use super::*;
    /// - **测试目标**：验证位于同一物理页内的两条 256 字节 ext4 Inode 元数据记录在多核并发更新时的内容锁互斥、Extent 块映射保留与完整 CRC32C 校验和重算。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given 从只读 ext4 根盘读取根目录与 `/pattern` 文件的两条 256 字节原始 Inode 记录，复制到私有测试盘第 0 页的 `0..256` 与 `256..512` 区间，并由主线程预先锁住该页内容睡眠锁（`lock_content`）；
    ///   2. When 发布两个分别调用 `codec::patch_inode_metadata` 更新各自 Inode 记录的内核任务，确认两者均在内容锁上进入 `State::Blocked` 后释放内容锁，等待两个任务完成更新；
    ///   3. Then 两条记录均通过 `codec::decode_inode` 完整 CRC32C 校验并解码出更新后的元数据，记录内偏移 `40..100` 的 Extent 树区域保持不变，篡改任意 1 比特均触发校验失败，且只读源盘上的原始 Inode 未被修改。
    #[test]
    fn common_page_record_updates_keep_both_checksums(
        context: &mut TestContext<'_>,
        fixture: &mut MetadataRecords,
    ) -> TestResult {
        fixture.0.concurrent_records(context)
    }
}
