//! 块设备协议后端适配层（Block Hardware Backends）。
//!
//! # 模块职责与架构定位
//! 本模块位于通用块设备核心（`Disk`）与底层硬件驱动（`VirtioDevice`、`Sdmmc`、`Ahci` 以及内存盘 `Ram`）之间，
//! 向上为 `Disk` 提供统一的协议控制准备、请求提交、完成结果提取、中断认领以及设备复位接口。
//! 逻辑父请求（`Bio`）的拆分、合并、DMA 生命周期跟踪与睡眠等待均由上层 `Disk` 统一负责；
//! 本模块只处理各设备传输协议的差异。
//!
//! # 核心设计与两类后端执行流
//! 后端按硬件并发模型分为两类路径：
//! ```text
//!   1. 多深度环形队列后端（VirtIO-Blk）
//!      Preparing 槽准备 PDU 头 (SlotControl::prepare)
//!          │
//!          ▼ (Disk.queue 自旋锁内)
//!      Backend::submit() ──► 转换 SG 段并写入 VirtQueue，记录 head -> tag 映射
//!          │
//!          ▼ (锁外敲门铃 Notification::notify，设备异步 DMA)
//!      IRQ / 轮询 ──► Backend::acknowledge() & Backend::pop_completion()
//!          │
//!          ▼
//!      SlotControl::complete() 读取 1B status 并校验 used.len，返回通用 IoResult
//!
//!   2. 单通道工作队列后端（RAM / SDMMC / AHCI，硬件深度 1 + 8 个软件槽）
//!      Backend::submit() ──► (Disk.queue 自旋锁内) 仅将软件槽号 tag 压入 pending 队列
//!          │
//!          ▼ (唤醒 Disk.progress_work，在可睡眠任务上下文锁外执行)
//!      Backend::next_work_request() & work_target() ──► 弹出 tag 并克隆控制器句柄
//!          │
//!          ▼ (释放 Disk.queue 自旋锁后执行同步传输)
//!      - RAM  : CPU 直接在保留物理内存与 Bio 页面之间复制字节
//!      - SDMMC: 控制器内部使用 Bounce Buffer 发起单命令 DMA + IRQ 睡眠等待
//!      - AHCI : 控制器内部构建 PRDT 对 Bio 物理页直接发起单端口 SATA DMA
//! ```
//!
//! # VirtIO-Blk 单槽协议数据单元（PDU）布局
//! 每个软件槽 `tag` 在连续的 `pdu` 一致性 DMA 区域内占据固定步长 `PDU_STRIDE`：
//! ```text
//!   base + tag * PDU_STRIDE
//!   +------------------------------------+------------------+---------------+
//!   | VirtIO BlkReq Header (16 Bytes)    |  Padding         | Status (1 B)  |
//!   | type (u32), reserved (u32),        |                  | (@ STATUS_    |
//!   | sector (u64) [Driver -> Device]    |                  |  OFFSET)      |
//!   +------------------------------------+------------------+---------------+
//!                                                            [Device -> Driver]
//! ```
//! VirtIO 的 `used.len` 表示设备写入缓冲区的总字节数：对于写/刷新请求为 `1`（仅状态字节），
//! 对于读请求为 `数据字节数 + 1`；`SlotControl::complete` 负责剥离该协议字节并校验短读或协议异常。
//!
//! # 锁获取顺序与并发契约
//! - **`Backend` 自身无独立锁**：对 `&mut Backend` 的一切访问均受外层 `Disk.queue` 自旋锁（`SpinLock<QueueState>`）串行保护，
//!   临界区极短且严禁睡眠、内存分配或大块数据复制。
//! - **锁层级顺序**：当后端方法内部调用控制器接口（如 `Sdmmc::acknowledge`、`Ahci::set_interrupts`）时，
//!   锁顺序严格为 `Disk.queue`（自旋锁） -> 控制器内部寄存器/状态自旋锁（如 `Sdmmc.registers`、`Ahci.state`）。
//! - **可睡眠锁隔离**：控制器内部的睡眠互斥锁（`Sdmmc.command`、`Ahci.control`）以及 VirtIO 门铃通知，
//!   必须在释放 `Disk.queue` 自旋锁之后，通过 `WorkTarget` 或 `Notification` 在锁外调用。

use crate::block::{
    BlkOperation, BlockError, IoResult, QueueLimits, NATIVE_TAG_COUNT, SECTOR_BYTES,
};
use crate::boot::Region;
use crate::boot::{AhciResources, MmcResources};
use crate::drivers::ahci::Ahci;
use crate::drivers::sdmmc::Sdmmc;
use crate::drivers::virtio::{
    VirtioDevice, VirtioIdentity, VirtioInitError, VIRTIO_CONTROL_SEGMENTS, VIRTIO_MAX_QUEUE_DEPTH,
    VIRTIO_MIN_QUEUE_DEPTH, VIRTIO_STATUS_BYTES,
};
use crate::irq::IrqNumber;
use crate::mm::dma::{DmaDomain, KernelHal, PayloadRange};
use alloc::collections::VecDeque;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::convert::TryFrom;
use core::ptr;
use virtio_drivers::device::blk::{decode_status, prepare_pdu, BlkInfo, PDU_STRIDE, STATUS_OFFSET};
use virtio_drivers::transport::{DeviceStatus, InterruptStatus, Notification};
use virtio_drivers::{DmaRegion, Hal, MappedSegment};

const RAM_MAX_BYTES: usize = 64 * 1024;
const RAM_MAX_SEGMENTS: usize = 16;

/// 块设备的物理硬件身份标识。
///
/// 上层构造 `Bio` 与挂载文件系统时用该标识精确匹配目标物理磁盘，防止跨设备误交请求。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DiskIdentity {
    /// VirtIO 块设备身份（区分 MMIO 物理基址或 PCI BDF 拓扑）。
    Virtio(VirtioIdentity),
    /// 专用保留物理内存盘身份（`start` 为保留物理内存起始字节地址，`length` 为保留物理内存总字节长度）。
    Ram { start: usize, length: usize },
    /// SD/MMC 控制器块设备身份（`physical` 为 SD/MMC 控制器寄存器窗口的起始物理字节地址）。
    Sdmmc { physical: usize },
    /// AHCI SATA 控制器块设备身份（`physical` 为 AHCI 控制器 ABAR 寄存器窗口的起始物理字节地址）。
    Ahci { physical: usize },
}

/// 单个软件请求槽（Slot）持有的设备协议控制区视图。
///
/// 该视图仅保存协议头/状态区的访问句柄，不持有业务数据（Payload）页面；
/// 底层 DMA 内存的所有权始终归 `Backend` 持有，直至 `Disk::stop` 确认硬件彻底停止后才释放。
#[derive(Clone, Copy)]
pub(super) enum SlotControl {
    /// VirtIO 协议控制视图（`pdu` 为整块 PDU 控制区的 DMA 内存描述符，`tag` 为当前软件请求槽编号 `0..tag_count`）。
    Virtio { pdu: DmaRegion, tag: u16 },
    /// 无需在软件槽阶段预填 DMA 协议头的后端（RAM / SDMMC / AHCI）。
    Plain,
}

impl SlotControl {
    /// 在请求槽处于独占 `Preparing` 阶段时，向协议控制区写入本次请求的命令头。
    ///
    /// 在普通可睡眠任务上下文、未持有 `Disk.queue` 自旋锁时调用，仅初始化协议元数据，不复制业务数据。
    pub(super) fn prepare(self, operation: BlkOperation, sector: u64) -> Result<(), BlockError> {
        if let Self::Virtio { pdu, tag } = self {
            // SAFETY:
            // 1. 调用方通过 `Disk::reserve_slot` 独占持有该 `tag`（槽状态为 `SlotState::Preparing`），
            //    没有其他 CPU 或硬件并发读写 `tag * PDU_STRIDE` 处的协议单元；
            // 2. `pdu` 指向的 DMA 控制页由 `VirtioDevice` 分配并保活至 `Disk::stop` 完成硬件复位之后；
            // 3. `tag < limits.tag_count`，位于 `VirtioDevice` 预分配的 `pdu` 容量范围内。
            unsafe { prepare_pdu(pdu, tag, virtio_operation(operation), sector) }
                .map_err(BlockError::from)?;
        }
        Ok(())
    }

    /// 在硬件交还 `used` 环项后读取协议状态字节，并将传输长度归一化为块层通用的 [`IoResult`]。
    ///
    /// 在持有 `Disk.queue` 自旋锁消费完成队列（如中断下半部或收割循环）时调用。
    /// `bytes` 为请求期望传输的数据字节数，`used` 为 VirtIO `used` 环报告的设备写入总字节数（含 1 字节状态码）。
    pub(super) fn complete(self, operation: BlkOperation, bytes: usize, used: u32) -> IoResult {
        let Self::Virtio { pdu, tag } = self else {
            panic!("plain slot completion must come from its progress Work");
        };
        // Step 1: 插入 DMA 读内存屏障，确保能看到设备在更新 used 环之前写入的 status 字节与业务数据。
        KernelHal::dma_read_barrier();
        // SAFETY:
        // 1. 调用方 `Backend::pop_completion` 已从硬件 `used` 环弹出对应描述符链，设备对该槽 PDU 的 DMA 写入已经结束；
        // 2. 调用方持有 `Disk.queue` 自旋锁，且该槽即将转入 `Finishing` 状态，在完成收尾前不会被重新分配或复用；
        // 3. `pdu` 底层 DMA 控制页在设备存活期间始终有效，`tag * PDU_STRIDE + STATUS_OFFSET` 严格处于已分配范围内。
        let status = unsafe {
            ptr::read_volatile(
                pdu.cpu
                    .as_ptr()
                    .add(usize::from(tag) * PDU_STRIDE + STATUS_OFFSET),
            )
        };
        if let Err(error) = decode_status(status) {
            return IoResult {
                completed: 0,
                error: Some(error.into()),
            };
        }

        // Step 2: 剥离 VirtIO 协议附加的 1 字节状态位，核对设备实际写入字节数以识别正常完成、短读或设备协议故障。
        let used = used as usize;
        let expected = if operation == BlkOperation::Read {
            bytes + VIRTIO_STATUS_BYTES
        } else {
            VIRTIO_STATUS_BYTES
        };
        if used == expected {
            IoResult {
                completed: bytes,
                error: None,
            }
        } else if operation == BlkOperation::Read && used < expected {
            IoResult {
                completed: used.saturating_sub(VIRTIO_STATUS_BYTES),
                error: Some(BlockError::ShortIo),
            }
        } else {
            IoResult {
                completed: 0,
                error: Some(BlockError::DeviceFault),
            }
        }
    }
}

/// 中断服务例程（ISR）认领设备中断后归一化得到的中断原因摘要。
///
/// 在硬中断上下文中由 [`Backend::acknowledge`] 生成，整个认领过程不睡眠、不分配内存、不复制业务数据。
pub(super) struct IrqCause {
    /// 本设备是否实际触发并认领了此次中断（用于共享中断线判定）。
    pub(super) claimed: bool,
    /// 是否包含队列请求完成事件（提示上层立即排空 VirtIO `used` 环）。
    pub(super) completion: bool,
    /// 设备是否报告配置变更或进入 `DEVICE_NEEDS_RESET` / `FAILED` 故障状态。
    pub(super) fault: bool,
}

/// VirtIO 块设备后端运行期状态与预分配工作表。
///
/// 所有动态数组均在 `Backend::virtio` 构造阶段一次性精确预分配，保证稳态 I/O 提交路径零堆分配。
pub(super) struct VirtioBackend {
    /// 底层 VirtIO 块设备实例（持有传输层、VirtQueue 环形缓冲区与控制 DMA 页）。
    device: VirtioDevice,
    /// VirtQueue 描述符链头索引（`head`，长度 `queue_depth`）到软件槽号（`tag`）的在途映射表。
    heads: Vec<Option<u16>>,
    /// 按软件槽号（`tag`，长度 `tag_count`）预分配的物理散聚段（Scatter-Gather）转换缓冲区。
    segments: Vec<Vec<MappedSegment>>,
    /// 按软件槽号（`tag`，长度 `tag_count`）记录的在途请求操作类型与期望数据字节数。
    requests: Vec<Option<(BlkOperation, usize)>>,
}

/// 块设备底层硬件后端枚举。
///
/// 统一封装内核支持的四类块存储介质；嵌入在 `Disk` 的 `QueueState` 中，由 `Disk.queue` 自旋锁保护。
#[expect(
    clippy::large_enum_variant,
    reason = "Disk 内嵌后端直接拥有已准备的传输和队列，避免单独装箱增加准备期分配与失败回收分支"
)]
pub(super) enum Backend {
    /// VirtIO 块设备后端（支持多深度异步环形队列）。
    Virtio(VirtioBackend),
    /// 基于保留物理内存区间的同步复制内存盘后端（无物理中断与设备 DMA）。
    Ram {
        /// 内存盘独占的物理地址半开区间 `[start, end)`（字节，512B 扇区对齐）。
        region: Region,
        /// 后端是否已收到停止请求并拒绝后续排队与拉取。
        stopped: bool,
        /// 等待 `Disk.progress_work` 顺序执行的已提交软件槽号（`tag`）FIFO 队列。
        pending: VecDeque<u16>,
    },
    /// SD/MMC 卡控制器后端（单硬件命令通道 + Bounce Buffer DMA）。
    Sdmmc {
        /// 引用计数的 SD/MMC 控制器实例，支持克隆到锁外工作队列任务中执行命令。
        controller: Arc<Sdmmc>,
        /// 控制器 MMIO 寄存器起始物理地址（字节），用于构造 `DiskIdentity`。
        physical: usize,
        /// 后端是否已进入停止阶段。
        stopped: bool,
        /// 等待 `Disk.progress_work` 顺序下发给控制器的软件槽号（`tag`）FIFO 队列。
        pending: VecDeque<u16>,
    },
    /// AHCI SATA 控制器后端（单端口非 NCQ 命令通道 + 物理页直接 PRDT DMA）。
    Ahci {
        /// 引用计数的 AHCI 控制器实例，支持克隆到锁外工作队列任务中执行命令。
        controller: Arc<Ahci>,
        /// 控制器 ABAR 寄存器窗口起始物理地址（字节），用于构造 `DiskIdentity`。
        physical: usize,
        /// 后端是否已进入停止阶段。
        stopped: bool,
        /// 等待 `Disk.progress_work` 顺序下发给控制器的软件槽号（`tag`）FIFO 队列。
        pending: VecDeque<u16>,
    },
}

/// 供 `Disk.progress_work` 在释放 `Disk.queue` 自旋锁后执行单次命令的后端句柄。
///
/// 自旋锁临界区内仅复制轻量级 `Region` 或克隆控制器 `Arc`，耗时的内存复制或设备命令等待在锁外可睡眠上下文完成。
pub(super) enum WorkTarget {
    /// 内存盘目标，携带独占保留物理地址范围。
    Ram(Region),
    /// SD/MMC 控制器目标。
    Sdmmc(Arc<Sdmmc>),
    /// AHCI 控制器目标。
    Ahci(Arc<Ahci>),
}

impl Backend {
    /// 在向硬件发布队列前构造 VirtIO 后端，并按队列限额精确预分配全部索引表与 SG 缓冲区。
    ///
    /// 在普通可睡眠任务上下文调用；预分配确保后续 I/O 提交与完成路径无需动态申请堆内存。
    pub(super) fn virtio(
        device: VirtioDevice,
        limits: QueueLimits,
        #[cfg(feature = "ktest")] mut observation: Option<
            &mut Option<(u16, crate::drivers::virtio::TestPrepareSnapshot)>,
        >,
    ) -> Result<Self, BlockError> {
        let mut heads = Vec::new();
        heads
            .try_reserve_exact(usize::from(limits.queue_depth))
            .map_err(|_| BlockError::NoMemory)?;
        heads.resize(usize::from(limits.queue_depth), None);

        let mut segments = Vec::new();
        segments
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        let mut requests = Vec::new();
        requests
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        requests.resize(usize::from(limits.tag_count), None);

        for _tag in 0..limits.tag_count {
            let mut sg = Vec::new();
            sg.try_reserve_exact(limits.max_segments).map_err(|_| {
                #[cfg(feature = "ktest")]
                if let Some(output) = observation.as_deref_mut() {
                    *output = Some((_tag, device.test_prepare_snapshot()));
                }
                BlockError::NoMemory
            })?;
            segments.push(sg);
        }
        Ok(Self::Virtio(VirtioBackend {
            device,
            heads,
            segments,
            requests,
        }))
    }

    /// 校验物理地址区间对齐并构造基于保留物理内存的 RAM 磁盘后端。
    ///
    /// 在普通可睡眠任务上下文调用。`region` 必须为非空且按 512 字节扇区对齐的独占物理内存区间
    /// （如启动阶段保留的 initrd，或由外层 `unsafe` 调用方在 `Disk::stop` 彻底完成前持续保活且无其他映射别名冲突的物理页）。
    pub(super) fn ram(region: Region) -> Result<Self, BlockError> {
        if region.end <= region.start || (region.end - region.start) % SECTOR_BYTES as usize != 0 {
            return Err(BlockError::InvalidRange);
        }

        let pending = pending_queue()?;
        Ok(Self::Ram {
            region,
            stopped: false,
            pending,
        })
    }

    /// 根据板级 MMC 资源初始化 SD/MMC 后端并返回对应的设备队列限额。
    ///
    /// 在普通可睡眠任务上下文调用。若卡槽检测（GPIO / CDETECT）确认物理无卡则返回 `Ok(None)`；
    /// 硬件通信故障或内存不足则原样返回 `Err`。
    ///
    /// # Safety
    /// 1. 调用方必须独占 `resources` 描述的控制器硬件及其关联的时钟、复位、引脚复用与卡检测 GPIO 资源，
    ///    且所有相关 MMIO 窗口均已建立永久有效的内核强序映射；
    /// 2. 外层 `Disk` 在调用 `publish` 启用设备中断前必须完成 IRQ 处理函数注册，
    ///    并在销毁后端前通过 `Disk::stop` 确保硬件复位确认及所有在途中断、工作队列任务彻底退出。
    pub(super) unsafe fn sdmmc(
        resources: &MmcResources,
    ) -> Result<Option<(Self, QueueLimits)>, BlockError> {
        let pending = pending_queue()?;
        // SAFETY: 本函数的 `# Safety` 前置条件完整覆盖了 `Sdmmc::prepare` 所需的硬件独占、
        // 永久 MMIO 映射以及先注册 IRQ 再发布、确认停止后才释放的生命周期契约。
        let Some(controller) = unsafe { Sdmmc::prepare(resources) }? else {
            return Ok(None);
        };
        let limits = controller.limits();
        Ok(Some((
            Self::Sdmmc {
                controller,
                physical: resources.registers.start,
                stopped: false,
                pending,
            },
            limits,
        )))
    }

    /// 根据枚举解析后的 PCI/MMIO 资源初始化 AHCI 后端并返回对应的设备队列限额。
    ///
    /// 在普通可睡眠任务上下文调用。仅当端口扫描确认无物理 SATA 介质时返回 `Ok(None)`；
    /// 硬件错误、对齐错误或内存不足则原样返回 `Err`。
    ///
    /// # Safety
    /// 1. 调用方必须独占 `resources` 描述的 AHCI 控制器 ABAR 窗口与目标端口，系统固件已移交控制权，
    ///    且完整 ABAR MMIO 窗口已建立永久有效的内核强序映射；
    /// 2. `resources` 中的中断号与 DMA 地址域参数必须与真实硬件拓扑一致；
    /// 3. 外层 `Disk` 在调用 `publish` 前必须注册 IRQ 处理函数，并保活本后端直至硬件确认停止且所有外部访问结束。
    pub(super) unsafe fn ahci(
        resources: &AhciResources,
    ) -> Result<Option<(Self, QueueLimits)>, BlockError> {
        let pending = pending_queue()?;
        // SAFETY: 本函数的 `# Safety` 前置条件完整覆盖了 `Ahci::prepare` 所需的 ABAR 独占强序映射、
        // 真实 PCI/DMA 资源参数以及停止确认前持续保活控制器的生命周期契约。
        let Some(controller) = unsafe { Ahci::prepare(resources) }? else {
            return Ok(None);
        };
        let limits = controller.limits();
        Ok(Some((
            Self::Ahci {
                controller,
                physical: resources.registers.start,
                stopped: false,
                pending,
            },
            limits,
        )))
    }

    /// 返回当前后端对应的物理设备身份标识 [`DiskIdentity`]。
    pub(super) fn identity(&self) -> DiskIdentity {
        match self {
            Self::Virtio(v) => DiskIdentity::Virtio(v.device.identity()),
            Self::Ram { region, .. } => DiskIdentity::Ram {
                start: region.start,
                length: region.end - region.start,
            },
            Self::Sdmmc { physical, .. } => DiskIdentity::Sdmmc {
                physical: *physical,
            },
            Self::Ahci { physical, .. } => DiskIdentity::Ahci {
                physical: *physical,
            },
        }
    }

    /// 计算给定物理内存区间的 RAM 磁盘队列限额（单条 CPU 复制执行通道 + 8 个软件请求槽）。
    pub(super) fn ram_limits(region: Region) -> Result<QueueLimits, BlockError> {
        if region.end <= region.start || (region.end - region.start) % SECTOR_BYTES as usize != 0 {
            return Err(BlockError::InvalidRange);
        }
        Ok(QueueLimits {
            capacity_sectors: ((region.end - region.start) / SECTOR_BYTES as usize) as u64,
            max_bytes: RAM_MAX_BYTES,
            max_segments: RAM_MAX_SEGMENTS,
            max_segment_bytes: RAM_MAX_BYTES as u32,
            logical_block_size: SECTOR_BYTES as u32,
            dma_boundary_mask: u64::MAX,
            readonly: false,
            flush: true,
            queue_depth: 1,
            tag_count: NATIVE_TAG_COUNT,
        })
    }

    /// 返回业务数据物理页（Payload Folio）直接进行设备 DMA 时所需的地址域 [`DmaDomain`]。
    ///
    /// VirtIO 与 AHCI 由硬件直接 DMA 访问业务物理页，需返回对应 `DmaDomain` 以执行地址转换与缓存同步；
    /// RAM 盘由 CPU 直接复制，SDMMC 通过控制器内部独立的 Bounce Buffer 中转，业务页均不向设备暴露 DMA 映射，因此返回 `None`。
    pub(super) fn domain(&self) -> Option<DmaDomain> {
        match self {
            Self::Virtio(_) => Some(DmaDomain::qemu_coherent()),
            Self::Ram { .. } | Self::Sdmmc { .. } => None,
            Self::Ahci { controller, .. } => Some(controller.domain()),
        }
    }

    /// 为指定编号 `tag` 的软件请求槽生成协议控制区视图 [`SlotControl`]。
    pub(super) fn slot_control(&self, tag: u16) -> SlotControl {
        match self {
            Self::Virtio(v) => SlotControl::Virtio {
                pdu: v.device.control_regions().1,
                tag,
            },
            Self::Ram { .. } | Self::Sdmmc { .. } | Self::Ahci { .. } => SlotControl::Plain,
        }
    }

    /// 返回后端关联的硬件中断号；无物理中断的纯内存盘（`Ram`）返回 `None`。
    pub(super) fn interrupt(&self) -> Option<IrqNumber> {
        match self {
            Self::Virtio(v) => Some(v.device.interrupt()),
            Self::Ram { .. } => None,
            Self::Sdmmc { controller, .. } => Some(controller.interrupt()),
            Self::Ahci { controller, .. } => Some(controller.interrupt()),
        }
    }

    /// 向硬件正式发布控制内存并使能设备工作状态。
    ///
    /// 在 `Disk::start` 持有控制睡眠锁与 `Disk.queue` 自旋锁时调用（IRQ 处理函数已预先注册但尚未使能中断线）。
    /// 若发布失败，后端所有权仍保留在 `Disk` 中，交由后续 `Disk::stop` 安全回收。
    pub(super) fn publish(&mut self) -> Result<(), BlockError> {
        match self {
            Self::Virtio(v) => v.device.publish().map_err(initialization_error),
            Self::Ram { stopped, .. } => {
                assert!(!*stopped);
                Ok(())
            }
            Self::Sdmmc { controller, .. } => controller.publish(),
            Self::Ahci { controller, .. } => controller.publish(),
        }
    }

    /// 检查后端当前是否具备接纳含 `segments` 个物理散聚段的子请求的硬件额度。
    ///
    /// 在持有 `Disk.queue` 自旋锁时调用：VirtIO 需检查 VirtQueue 剩余空闲描述符是否足以容纳数据段与 2 个控制段（Header + Status）；
    /// 其余由工作队列驱动的后端只要尚未停止即可入队（总并发度已由软件槽数量 `tag_count` 限制）。
    pub(super) fn can_submit(&self, segments: usize) -> bool {
        match self {
            Self::Virtio(v) => {
                segments + usize::from(VIRTIO_CONTROL_SEGMENTS)
                    <= v.device.driver().available_descriptors()
            }
            Self::Ram { stopped, .. }
            | Self::Sdmmc { stopped, .. }
            | Self::Ahci { stopped, .. } => !*stopped,
        }
    }

    /// 在 `Disk.queue` 自旋锁保护下向后端提交一个已完成准备的软件槽请求。
    ///
    /// - 对于 `Virtio`：将物理页范围映射为设备 DMA 段并填入 VirtQueue（门铃通知留到锁外 `unplug` 执行）；
    /// - 对于 `Ram` / `Sdmmc` / `Ahci`：仅将槽号 `tag` 推入 `pending` 队列，实际数据传输交由锁外的 `Disk.progress_work` 异步推进。
    pub(super) fn submit(
        &mut self,
        tag: u16,
        operation: BlkOperation,
        bytes: usize,
        ranges: &[PayloadRange],
    ) -> Result<(), BlockError> {
        match self {
            Self::Virtio(v) => {
                let sg = &mut v.segments[usize::from(tag)];
                sg.clear();
                for range in ranges {
                    sg.push(MappedSegment {
                        address: DmaDomain::qemu_coherent()
                            .map(range.physical, range.length as usize)
                            .map_err(|_| BlockError::InvalidRange)?,
                        length: range.length,
                        direction: range.direction,
                    });
                }
                // SAFETY:
                // 1. 调用方在 `Preparing` 阶段独占该 `tag`，已通过 `SlotControl::prepare` 填妥 PDU 请求头，
                //    且承载数据的物理页已完成 DMA 访问权移交（缓存刷写完毕、在完成前无 CPU 并发读写）；
                // 2. `sg` 中的设备地址均由上方 `DmaDomain::qemu_coherent().map` 校验生成；
                // 3. 当前持有 `Disk.queue` 自旋锁，排斥并发的队列操作与设备 `reset`。
                let head = unsafe {
                    v.device
                        .driver_mut()
                        .submit(tag, virtio_operation(operation), sg)
                }
                .map_err(|error| match error {
                    virtio_drivers::Error::QueueFull
                    | virtio_drivers::Error::DeviceNeedsReset
                    | virtio_drivers::Error::WrongToken => BlockError::DeviceFault,
                    other => other.into(),
                })?;
                assert!(
                    v.heads[usize::from(head)].is_none(),
                    "submitted head is already owned"
                );
                v.heads[usize::from(head)] = Some(tag);
                assert!(
                    v.requests[usize::from(tag)]
                        .replace((operation, bytes))
                        .is_none(),
                    "submitted software tag is already owned"
                );
                Ok(())
            }
            Self::Ram {
                stopped, pending, ..
            }
            | Self::Sdmmc {
                stopped, pending, ..
            }
            | Self::Ahci {
                stopped, pending, ..
            } => {
                if *stopped {
                    return Err(BlockError::QueueStopped);
                }
                assert!(
                    pending.len() < pending.capacity(),
                    "block software queue exceeds prepared slots"
                );
                pending.push_back(tag);
                Ok(())
            }
        }
    }

    /// 从硬件完成队列弹出一个已结束的请求，返回其软件槽号 `tag` 与归一化后的 [`IoResult`]。
    ///
    /// 在持有 `Disk.queue` 自旋锁时调用。仅 VirtIO 后端通过此路径从 `used` 环回收请求；
    /// 工作队列驱动的后端由 `progress_work` 直接写回槽完成结果，此处固定返回 `Ok(None)`。
    pub(super) fn pop_completion(&mut self) -> Result<Option<(u16, IoResult)>, BlockError> {
        let Self::Virtio(v) = self else {
            return Ok(None);
        };
        // SAFETY: 当前持有 `Disk.queue` 自旋锁，唯一可变访问 `VirtioDevice` 的驱动队列，
        // 且队列处于存活阶段，底层 VirtQueue 环形缓冲区 DMA 内存保持有效且未并发复位。
        let used = unsafe { v.device.driver_mut() }
            .pop_used()
            .map_err(|_| BlockError::DeviceFault)?;
        let Some(used) = used else {
            return Ok(None);
        };
        let tag = v
            .heads
            .get_mut(usize::from(used.head))
            .and_then(Option::take)
            .ok_or(BlockError::DeviceFault)?;
        let (operation, bytes) = v.requests[usize::from(tag)]
            .take()
            .ok_or(BlockError::DeviceFault)?;
        let control = SlotControl::Virtio {
            pdu: v.device.control_regions().1,
            tag,
        };
        Ok(Some((tag, control.complete(operation, bytes, used.len))))
    }

    /// 开启或抑制本设备自身的完成中断通知（不操作中断控制器上的共享物理 IRQ 线）。
    ///
    /// 在持有 `Disk.queue` 自旋锁时调用，用于中断收割期间抑制重复中断风暴或在故障时静默设备。
    pub(super) fn set_interrupts(&mut self, enabled: bool) {
        match self {
            Self::Virtio(v) => {
                // SAFETY: 当前持有 `Disk.queue` 自旋锁串行访问 VirtQueue，环形缓冲区 DMA 内存有效且未复位。
                unsafe { v.device.driver_mut() }.set_interrupts(enabled);
            }
            Self::Sdmmc { controller, .. } => controller.set_interrupts(enabled),
            Self::Ahci { controller, .. } => controller.set_interrupts(enabled),
            Self::Ram { .. } => {}
        }
    }

    /// 重新开启设备中断通知并复查在此窗口内是否又有新完成的请求到达，防止漏掉早到的完成事件。
    ///
    /// 在持有 `Disk.queue` 自旋锁且单轮中断收割达到预算上限或排空时调用。返回 `true` 表示仍有待处理工作，需调度续处理。
    pub(super) fn arm_and_check(&mut self) -> bool {
        match self {
            Self::Virtio(v) => {
                // SAFETY: 当前持有 `Disk.queue` 自旋锁独占访问 VirtQueue 的 `avail`/`used` 环标志与索引，无并发竞争。
                unsafe { v.device.driver_mut() }.enable_interrupts_and_recheck()
            }
            Self::Ram { .. } => false,
            Self::Sdmmc { controller, .. } => controller.arm_and_check(),
            Self::Ahci { controller, .. } => controller.arm_and_check(),
        }
    }

    /// 在硬中断服务例程（ISR）中读取并清除本设备的中断状态寄存器，返回归一化后的 [`IrqCause`]。
    ///
    /// 在持有 `Disk.queue` 自旋锁的关中断上下文调用；只做寄存器认领与唤醒投递，不睡眠、不轮询等待、不复制数据。
    pub(super) fn acknowledge(&mut self) -> IrqCause {
        match self {
            Self::Virtio(v) => {
                let status = v.device.ack_interrupt();
                IrqCause {
                    claimed: !status.is_empty(),
                    completion: status.contains(InterruptStatus::QUEUE),
                    fault: status.contains(InterruptStatus::CONFIG)
                        || v.device
                            .status()
                            .intersects(DeviceStatus::DEVICE_NEEDS_RESET | DeviceStatus::FAILED),
                }
            }
            Self::Sdmmc { controller, .. } => IrqCause {
                claimed: controller.acknowledge(),
                completion: false,
                fault: false,
            },
            Self::Ahci { controller, .. } => IrqCause {
                claimed: controller.acknowledge(),
                completion: false,
                fault: false,
            },
            Self::Ram { .. } => IrqCause {
                claimed: false,
                completion: false,
                fault: false,
            },
        }
    }

    /// 若后端需要在锁外敲击硬件门铃（Doorbell），返回对应的 [`Notification`] 令牌。
    ///
    /// 在持有 `Disk.queue` 自旋锁的 `Disk::unplug` 中调用；仅当 VirtIO 设备未抑制通知时返回 `Some`。
    pub(super) fn notification(&self) -> Option<Notification> {
        match self {
            Self::Virtio(v) if v.device.driver().should_notify() => {
                Some(v.device.driver().notification())
            }
            _ => None,
        }
    }

    /// 判断该后端是否通过 `Disk.progress_work` 工作队列任务在锁外推进请求执行。
    ///
    /// `Ram`、`Sdmmc`、`Ahci` 返回 `true`；异步环形队列驱动的 `Virtio` 返回 `false`。
    pub(super) fn uses_work(&self) -> bool {
        !matches!(self, Self::Virtio(_))
    }

    /// 提取供 `Disk.progress_work` 在释放 `Disk.queue` 自旋锁后执行数据传输的目标句柄 [`WorkTarget`]。
    ///
    /// 克隆控制器 `Arc` 后，工作队列任务可在锁外可睡眠上下文执行命令，同时不阻碍硬中断继续通过 `Backend` 认领中断事件。
    pub(super) fn work_target(&self) -> Option<WorkTarget> {
        match self {
            Self::Ram { region, .. } => Some(WorkTarget::Ram(*region)),
            Self::Sdmmc { controller, .. } => Some(WorkTarget::Sdmmc(Arc::clone(controller))),
            Self::Ahci { controller, .. } => Some(WorkTarget::Ahci(Arc::clone(controller))),
            Self::Virtio(_) => None,
        }
    }

    /// 从待执行队列头部弹出下一个软件请求槽号 `tag`，供唯一的 `Disk.progress_work` 顺序执行。
    ///
    /// 在持有 `Disk.queue` 自旋锁时调用；若后端已标记 `stopped` 或队列为空则返回 `None`。
    pub(super) fn next_work_request(&mut self) -> Option<u16> {
        match self {
            Self::Ram {
                pending,
                stopped: false,
                ..
            }
            | Self::Sdmmc {
                pending,
                stopped: false,
                ..
            }
            | Self::Ahci {
                pending,
                stopped: false,
                ..
            } => pending.pop_front(),
            _ => None,
        }
    }

    /// 向底层硬件发出停止/复位请求，并阻止后续软件队列继续弹出新请求。
    ///
    /// 由 `Disk` 停机流程在持有 `Disk.queue` 自旋锁、且已确认所有锁外门铃通知与 `Executing` 命令退出后调用。
    pub(super) fn request_reset(&mut self) {
        match self {
            Self::Virtio(v) => v.device.request_reset(),
            Self::Ram { stopped, .. } => *stopped = true,
            Self::Sdmmc {
                controller,
                stopped,
                ..
            } => {
                *stopped = true;
                controller.request_reset();
            }
            Self::Ahci {
                controller,
                stopped,
                ..
            } => {
                *stopped = true;
                controller.request_reset();
            }
        }
    }

    /// 检查底层硬件是否已确认复位完成且彻底停止一切 DMA 访问。
    ///
    /// 在持有 `Disk.queue` 自旋锁的复位轮询循环中调用；只有返回 `true` 后，上层才可安全收回在途物理页并释放控制器内存。
    pub(super) fn reset_complete(&mut self) -> bool {
        match self {
            Self::Virtio(v) => v.device.reset_complete(),
            Self::Ram { stopped, .. } => *stopped,
            Self::Sdmmc { controller, .. } => controller.reset_complete(),
            Self::Ahci { controller, .. } => controller.reset_complete(),
        }
    }

    /// 单元测试辅助接口：借用内部 `VirtioDevice` 实例以检查协议层状态。
    #[cfg(feature = "ktest")]
    pub(super) fn test_virtio(&self) -> &VirtioDevice {
        match self {
            Self::Virtio(v) => &v.device,
            _ => panic!("VirtIO fixture requires a VirtIO backend"),
        }
    }

    /// 单元测试辅助接口：统计 VirtIO 描述符头映射表中当前在途（In-flight）的硬件请求数量。
    #[cfg(feature = "ktest")]
    pub(super) fn test_inflight(&self) -> usize {
        match self {
            Self::Virtio(v) => v.heads.iter().flatten().count(),
            _ => 0,
        }
    }
}

/// 为单硬件通道后端预分配固定容量为 [`NATIVE_TAG_COUNT`]（8）的软件待执行槽队列，避免稳态提交时动态扩容。
fn pending_queue() -> Result<VecDeque<u16>, BlockError> {
    let mut pending = VecDeque::new();
    pending
        .try_reserve_exact(usize::from(NATIVE_TAG_COUNT))
        .map_err(|_| BlockError::NoMemory)?;
    Ok(pending)
}

/// 校验 VirtIO 设备报告的配置参数 [`BlkInfo`]，并换算为块层统一的 [`QueueLimits`]。
///
/// 负责扣除每个请求固定占用的 2 个控制描述符（请求头与 1 字节状态尾），并将单请求最大字节数向下对齐到逻辑块大小且不超过 `u32` 协议上限。
pub(super) fn virtio_limits(info: BlkInfo) -> Result<QueueLimits, BlockError> {
    // Step 1: 校验容量、队列深度（2 的幂且在支持区间内）、描述符数量与逻辑块对齐的合法性。
    if info.capacity_sectors == 0
        || info.capacity_sectors.checked_mul(SECTOR_BYTES).is_none()
        || !info.queue_depth.is_power_of_two()
        || !(VIRTIO_MIN_QUEUE_DEPTH..=VIRTIO_MAX_QUEUE_DEPTH).contains(&info.queue_depth)
        || info.tag_count == 0
        || info.queue_depth < info.tag_count
        || info.max_segments == 0
        || info.queue_depth - VIRTIO_CONTROL_SEGMENTS < info.max_segments
        || info.max_segment_bytes == 0
        || info.logical_block_size < SECTOR_BYTES as u32
        || !info.logical_block_size.is_power_of_two()
    {
        return Err(BlockError::InvalidRange);
    }

    // Step 2: 计算单次硬件请求最大数据字节数（预留 1B status 空间以防 used.len 溢出 u32，并按逻辑块大小向下取整）。
    let logical = u64::from(info.logical_block_size);
    let raw = (u64::from(info.max_segment_bytes) * u64::from(info.max_segments))
        .min(u64::from(u32::MAX) - VIRTIO_STATUS_BYTES as u64);
    let max_bytes =
        usize::try_from(raw / logical * logical).map_err(|_| BlockError::InvalidRange)?;
    if max_bytes == 0 {
        return Err(BlockError::Unsupported);
    }

    Ok(QueueLimits {
        capacity_sectors: info.capacity_sectors,
        max_bytes,
        max_segments: usize::from(info.max_segments),
        max_segment_bytes: info.max_segment_bytes,
        logical_block_size: info.logical_block_size,
        dma_boundary_mask: u64::MAX,
        readonly: info.readonly,
        flush: info.flush,
        queue_depth: info.queue_depth,
        tag_count: info.tag_count,
    })
}

/// 将块层通用的操作枚举 [`BlkOperation`] 转换为 `virtio_drivers` 协议层对应的操作码。
fn virtio_operation(operation: BlkOperation) -> virtio_drivers::device::blk::BlkOperation {
    match operation {
        BlkOperation::Read => virtio_drivers::device::blk::BlkOperation::Read,
        BlkOperation::Write => virtio_drivers::device::blk::BlkOperation::Write,
        BlkOperation::Flush => virtio_drivers::device::blk::BlkOperation::Flush,
    }
}

/// 将 VirtIO 发布阶段的初始化错误 [`VirtioInitError`] 映射为块层统一的 [`BlockError`]。
fn initialization_error(error: VirtioInitError) -> BlockError {
    match error {
        VirtioInitError::Protocol(error) => error.into(),
        VirtioInitError::NoMemory { .. } => BlockError::NoMemory,
        _ => BlockError::DeviceFault,
    }
}
