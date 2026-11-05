//! 硬件后端只管理协议、提交与真实收权，逻辑父请求由 Disk 统一拥有。

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

/// 物理身份保留原传输身份，专用保留 RAM 使用其真实物理范围。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DiskIdentity {
    Virtio(VirtioIdentity),
    Ram { start: usize, length: usize },
    Sdmmc { physical: usize },
    Ahci { physical: usize },
}

/// 每个槽的协议控制视图不持 payload，设备 owner 覆盖其访问寿命。
#[derive(Clone, Copy)]
pub(super) enum SlotControl {
    Virtio { pdu: DmaRegion, tag: u16 },
    Plain,
}

impl SlotControl {
    /// 准备尚未发表的协议控制，不在 IRQ 锁内复制 payload。
    pub(super) fn prepare(self, operation: BlkOperation, sector: u64) -> Result<(), BlockError> {
        if let Self::Virtio { pdu, tag } = self {
            // SAFETY: Preparing 独占此 tag，完整 PDU owner 保活且尚无设备引用。
            unsafe { prepare_pdu(pdu, tag, virtio_operation(operation), sector) }
                .map_err(BlockError::from)?;
        }
        Ok(())
    }

    /// 真实 used 收权后归一状态和长度，公共路径不解释协议附加字节。
    pub(super) fn complete(self, operation: BlkOperation, bytes: usize, used: u32) -> IoResult {
        let Self::Virtio { pdu, tag } = self else {
            panic!("plain slot completion must come from its progress Work");
        };
        KernelHal::dma_read_barrier();
        // SAFETY: 本槽已取得真实 used，原 PDU owner 尚存且 tag 在完成尾前不可复用。
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

/// ISR 原因已经归一，后端不得在认领中睡眠或复制数据。
pub(super) struct IrqCause {
    pub(super) claimed: bool,
    pub(super) completion: bool,
    pub(super) fault: bool,
}

pub(super) struct VirtioBackend {
    device: VirtioDevice,
    heads: Vec<Option<u16>>,
    segments: Vec<Vec<MappedSegment>>,
    requests: Vec<Option<(BlkOperation, usize)>>,
}

/// 封闭集合只列出已经实际实现的设备；RAM 没有 DMA 控制页或 token。
#[expect(
    clippy::large_enum_variant,
    reason = "Disk 内嵌后端直接拥有已准备的传输和队列，避免单独装箱增加准备期分配与失败回收分支"
)]
pub(super) enum Backend {
    Virtio(VirtioBackend),
    Ram {
        region: Region,
        stopped: bool,
        pending: VecDeque<u16>,
    },
    Sdmmc {
        controller: Arc<Sdmmc>,
        physical: usize,
        stopped: bool,
        pending: VecDeque<u16>,
    },
    Ahci {
        controller: Arc<Ahci>,
        physical: usize,
        stopped: bool,
        pending: VecDeque<u16>,
    },
}

/// 短锁只克隆稳定控制器拥有者，唯一 progress Work 在锁外完成一次真实命令。
pub(super) enum WorkTarget {
    Ram(Region),
    Sdmmc(Arc<Sdmmc>),
    Ahci(Arc<Ahci>),
}

impl Backend {
    /// 在发表前预留 VirtIO 的 head 索引和全部转换 SG，不在提交时分配。
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

    /// 保存完整、独占且直接映射有效的 RAM 范围；来源可为 MM 接管前保留的 initrd，
    /// 或由调用者保活到真实 stop、Work/IRQ 和全部 CPU 复制尾部结束的独占原页。
    /// 发表期间调用者及固件不得访问这些字节，也不得建立用户或其他设备映射；
    /// 访问和原拥有者寿命由外层 unsafe 准备入口的调用者保证。
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

    /// 从唯一资源准备实际 SD；真实无卡返回 None，其余初始化错误保持原类别。
    ///
    /// # Safety
    /// 调用者须独占启动快照中的真实控制器及其 syscon/clock/reset/pin/GPIO，保其完整永久映射；
    /// 外层 Disk 在 publish 前登记 IRQ，之后保活本后端到停止与全部执行尾部。
    pub(super) unsafe fn sdmmc(
        resources: &MmcResources,
    ) -> Result<Option<(Self, QueueLimits)>, BlockError> {
        let pending = pending_queue()?;
        // SAFETY: 外层调用者传递资源独占、永久映射及 IRQ/停止的完整寿命义务。
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

    /// 从唯一库存解析后的 ABAR/IRQ 准备真实 AHCI；只有真实无介质返回 None。
    ///
    /// # Safety
    /// 调用者独占实际 ABAR 和端口，窗口永久强序映射，固件已退出；PCI 和 DMA 域资源真实有效。
    /// IRQ action 在 publish 前登记，本 owner 保活至真实停止与全部外部访问尾部。
    pub(super) unsafe fn ahci(
        resources: &AhciResources,
    ) -> Result<Option<(Self, QueueLimits)>, BlockError> {
        let pending = pending_queue()?;
        // SAFETY: 外层调用者证明已解析资源、永久映射、独占功能和停止寿命。
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

    /// 返回本后端实际硬件身份，不将 RAM 伪装为传输设备。
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

    /// RAM 的单复制通道与八个回调槽分别限额。
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

    /// 仅在原页需要真实设备访问时返回地址域。
    pub(super) fn domain(&self) -> Option<DmaDomain> {
        match self {
            Self::Virtio(_) => Some(DmaDomain::qemu_coherent()),
            Self::Ram { .. } | Self::Sdmmc { .. } => None,
            Self::Ahci { controller, .. } => Some(controller.domain()),
        }
    }

    /// 每槽只保存控制区视图，原 owner 仍留在后端直到 stop 真尾。
    pub(super) fn slot_control(&self, tag: u16) -> SlotControl {
        match self {
            Self::Virtio(v) => SlotControl::Virtio {
                pdu: v.device.control_regions().1,
                tag,
            },
            Self::Ram { .. } | Self::Sdmmc { .. } | Self::Ahci { .. } => SlotControl::Plain,
        }
    }

    /// 没有物理中断的 CPU 后端不登记伪 IRQ。
    pub(super) fn interrupt(&self) -> Option<IrqNumber> {
        match self {
            Self::Virtio(v) => Some(v.device.interrupt()),
            Self::Ram { .. } => None,
            Self::Sdmmc { controller, .. } => Some(controller.interrupt()),
            Self::Ahci { controller, .. } => Some(controller.interrupt()),
        }
    }

    /// 发表全部控制内存，失败保留后端供原 stop 结束拥有。
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

    /// 判断本次准备是否有真实提交额度，不将软件槽数当作硬件描述符数。
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

    /// 在短队列锁下发表一个预备请求；CPU 后端只接受元数据，复制交原 progress Work。
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
                // SAFETY: caller 独占 tag，控制已准备，原页访问已交权，queue 锁排斥 reset。
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

    /// 取一次真实完成并消耗其协议 token，公共核心只见原软件槽号。
    pub(super) fn pop_completion(&mut self) -> Result<Option<(u16, IoResult)>, BlockError> {
        let Self::Virtio(v) = self else {
            return Ok(None);
        };
        // SAFETY: queue 锁唯一串行后端，控制内存保活且尚未停止。
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

    /// 只改变本设备通知，不改共享 IRQ 线。
    pub(super) fn set_interrupts(&mut self, enabled: bool) {
        match self {
            Self::Virtio(v) => {
                // SAFETY: queue 锁串行控制，原 ring owner 仍有效。
                unsafe { v.device.driver_mut() }.set_interrupts(enabled);
            }
            Self::Sdmmc { controller, .. } => controller.set_interrupts(enabled),
            Self::Ahci { controller, .. } => controller.set_interrupts(enabled),
            Self::Ram { .. } => {}
        }
    }

    /// 重装并复查关闭早完成竞态，CPU 后端由 Work 直接推进。
    pub(super) fn arm_and_check(&mut self) -> bool {
        match self {
            Self::Virtio(v) => {
                // SAFETY: 同一 queue 锁保护 flags 与 used 索引，没有借用跨锁。
                unsafe { v.device.driver_mut() }.enable_interrupts_and_recheck()
            }
            Self::Ram { .. } => false,
            Self::Sdmmc { controller, .. } => controller.arm_and_check(),
            Self::Ahci { controller, .. } => controller.arm_and_check(),
        }
    }

    /// 在 ISR 中只认领自身原因和故障，不进行长复制或卡状态等待。
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

    /// 返回可锁外执行的真实门铃；RAM 不产生假通知。
    pub(super) fn notification(&self) -> Option<Notification> {
        match self {
            Self::Virtio(v) if v.device.driver().should_notify() => {
                Some(v.device.driver().notification())
            }
            _ => None,
        }
    }

    /// 锁外命令路径不调用 VirtIO 的中断抑制或 used 消费。
    pub(super) fn uses_work(&self) -> bool {
        !matches!(self, Self::Virtio(_))
    }

    /// 取得稳定原 owner 供锁外执行，IRQ 同时仍可通过后端认领真实 mailbox。
    pub(super) fn work_target(&self) -> Option<WorkTarget> {
        match self {
            Self::Ram { region, .. } => Some(WorkTarget::Ram(*region)),
            Self::Sdmmc { controller, .. } => Some(WorkTarget::Sdmmc(Arc::clone(controller))),
            Self::Ahci { controller, .. } => Some(WorkTarget::Ahci(Arc::clone(controller))),
            Self::Virtio(_) => None,
        }
    }

    /// 单一 progress Work 顺序认领硬件通道，通道释放先于可睡 EndIo 分派。
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

    /// 原 stop 已阻新门铃且等所有 CPU 访问尾，随后请求真实设备停止。
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

    /// 只有真实硬件停止确认或已无 CPU 访问的 RAM 才可释放原 owner。
    pub(super) fn reset_complete(&mut self) -> bool {
        match self {
            Self::Virtio(v) => v.device.reset_complete(),
            Self::Ram { stopped, .. } => *stopped,
            Self::Sdmmc { controller, .. } => controller.reset_complete(),
            Self::Ahci { controller, .. } => controller.reset_complete(),
        }
    }

    /// 测试只借真实 VirtIO 协议 owner，不将后端差异散入生产路径。
    #[cfg(feature = "ktest")]
    pub(super) fn test_virtio(&self) -> &VirtioDevice {
        match self {
            Self::Virtio(v) => &v.device,
            _ => panic!("VirtIO fixture requires a VirtIO backend"),
        }
    }

    /// 测试观察真实 token 索引，不改变协议状态。
    #[cfg(feature = "ktest")]
    pub(super) fn test_inflight(&self) -> usize {
        match self {
            Self::Virtio(v) => v.heads.iter().flatten().count(),
            _ => 0,
        }
    }
}

/// 硬件深度一的设备仍保八个有限软件槽，不在稳态提交时申请队列存储。
fn pending_queue() -> Result<VecDeque<u16>, BlockError> {
    let mut pending = VecDeque::new();
    pending
        .try_reserve_exact(usize::from(NATIVE_TAG_COUNT))
        .map_err(|_| BlockError::NoMemory)?;
    Ok(pending)
}

/// VirtIO 的 ring、status 附加字节和 descriptor 上限只在此边界解释。
pub(super) fn virtio_limits(info: BlkInfo) -> Result<QueueLimits, BlockError> {
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

/// 块语义只在真实协议边界转换一次。
fn virtio_operation(operation: BlkOperation) -> virtio_drivers::device::blk::BlkOperation {
    match operation {
        BlkOperation::Read => virtio_drivers::device::blk::BlkOperation::Read,
        BlkOperation::Write => virtio_drivers::device::blk::BlkOperation::Write,
        BlkOperation::Flush => virtio_drivers::device::blk::BlkOperation::Flush,
    }
}

/// 保留真正初始化失败，不丢弃可能已发表的后端拥有者。
fn initialization_error(error: VirtioInitError) -> BlockError {
    match error {
        VirtioInitError::Protocol(error) => error.into(),
        VirtioInitError::NoMemory { .. } => BlockError::NoMemory,
        _ => BlockError::DeviceFault,
    }
}
