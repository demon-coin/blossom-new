//! 固定请求槽与真实 ring head 共同维持块 I/O 的唯一拥有关系。

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::pin::{pin, Pin};
use core::ptr;

use crate::block::backend::{virtio_limits, Backend, SlotControl, WorkTarget};
use virtio_drivers::Hal;

use crate::block::bio::{BioArc, BioInner};
use crate::block::{
    AccessMode, Bio, BlkOperation, BlockError, DiskIdentity, FsClaim, QueueLimits, MAX_BATCH,
    SECTOR_BYTES,
};
use crate::cache::MetadataPages;
use crate::drivers::virtio::VirtioDevice;
use crate::irq::{self, IrqRegistration};
use crate::mm::dma::{DmaDomain, KernelHal, PayloadRange};
use crate::sync::{SleepMutex, SpinLock};
use crate::wait::{WaitNode, WaitQueue, Waiter};
use crate::workqueue::{Work, WorkQueue};

pub(super) const IRQ_BUDGET: usize = 8;

pub type DiskRef = Pin<Arc<Disk>>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum QueuePhase {
    Prepared,
    Live,
    Draining,
    Faulting,
    Resetting,
    DmaStopped,
    Stopped,
    Quarantined,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum SlotState {
    Free,
    Preparing,
    Submitted,
    Executing,
    Finishing,
}

pub(super) struct QueueState {
    pub(super) device: Option<Backend>,
    pub(super) phase: QueuePhase,
    pub(super) failure: Option<BlockError>,
    pub(super) generators: usize,
    pub(super) writer_generators: usize,
    pub(super) active_slots: usize,
    pub(super) writing_slots: usize,
    pub(super) notify_active: usize,
    pub(super) pending_notify: bool,
    pub(super) mounts: usize,
    pub(super) slot_states: Vec<SlotState>,
    pub(super) claim_head: *const crate::block::stop::FsClaimInner,
    pub(super) raw_writers: *const BioInner,
    pub(super) slot_writes: Vec<Option<(u64, u64)>>,
    #[cfg(feature = "ktest")]
    counters: TestSnapshot,
    #[cfg(feature = "ktest")]
    test_paused: bool,
    #[cfg(feature = "ktest")]
    pub(super) ktest_stop: StopTestState,
    #[cfg(feature = "ktest")]
    ktest_folio_dma: Option<(usize, u64, usize, usize)>,
    #[cfg(feature = "ktest")]
    ktest_completion_error: Option<(TestCompletionKind, BlockError, Option<usize>)>,
    #[cfg(feature = "ktest")]
    // 只匹配目标文件原页的 Data-WB，不改变真实 used ring 消费。
    ktest_hold_file_writeback_page: Option<usize>,
    #[cfg(feature = "ktest")]
    // 已收真实 used 但仍须走原 finish_slot 的软件完成责任。
    ktest_held_file_writeback_slots: [Option<u16>; MAX_BATCH],
}

impl QueueState {
    /// 已接受生成者在排空阶段仍能继续发表，故障之后不能新增设备引用。
    fn accepts_children(&self) -> bool {
        matches!(self.phase, QueuePhase::Live | QueuePhase::Draining)
    }

    /// 停机或故障返回队列事实，不让消费者把额度暂满误作设备错误。
    fn stopped_error(&self) -> BlockError {
        self.failure.unwrap_or(BlockError::QueueStopped)
    }
}

pub(super) struct RequestPart {
    pub(super) bio: BioArc,
    pub(super) offset: usize,
    pub(super) length: usize,
    // 独立保占用到 slot 资源和队列计数归还；父 ready 不终止此责任。
    _fs_claim: Option<FsClaim>,
}

struct SlotResources {
    sg: Vec<PayloadRange>,
    parts: [Option<RequestPart>; MAX_BATCH],
    part_count: usize,
    operation: BlkOperation,
    sector: u64,
    bytes: usize,
    forced_error: Option<BlockError>,
    completion: Option<crate::block::IoResult>,
    #[cfg(feature = "ktest")]
    ktest_read_prefix: Option<(usize, BlockError)>,
    configured: bool,
}

pub(super) struct SlotStorage {
    resources: UnsafeCell<SlotResources>,
    pub(super) completion_work: Option<Pin<Arc<Work>>>,
    pub(super) tag: u16,
    pub(super) disk: *const Disk,
    control: SlotControl,
}

// SAFETY: resources 只由 Preparing guard、唯一 Executing Work 或真实完成的 Finishing 认领者访问；
// 固定 Vec 不再增长，context 在登记前写定，Work 与 IRQ 尾部结束前 Disk owner 保活。
unsafe impl Send for SlotStorage {}
// SAFETY: 同一槽的两个独占访问期由 queue 状态转换隔开；不向外提供共享资源引用。
unsafe impl Sync for SlotStorage {}

/// 一个固定设备 owner 保存全部槽、控制页与回调上下文，不能从 Arc 中移出后移动。
pub struct Disk {
    pub(super) queue: SpinLock<QueueState>,
    pub(super) control: SleepMutex<()>,
    pub(super) irq: SpinLock<Option<IrqRegistration>>,
    pub(super) progress_work: Option<Pin<Arc<Work>>>,
    pub(super) slots: Vec<SlotStorage>,
    pub(super) available: WaitQueue,
    pub(super) quiescent: WaitQueue,
    pub(super) limits: QueueLimits,
    pub(super) identity: DiskIdentity,
    pub(super) domain: Option<DmaDomain>,
    metadata_cache: SpinLock<Weak<MetadataPages>>,
    pub(super) _pinned: PhantomPinned,
}

// SAFETY: 协议元数据由 queue 锁串行，原生命令由唯一 progress Work 和控制器内部锁串行；
// 固定 context 的寿命由显式 stop、执行尾、IRQ 注销及 Work close_sync 共同终止。
unsafe impl Send for Disk {}
// SAFETY: 安全接口不暴露 DMA 引用；跨 Task 的准备和完成只访问分别认领的槽。
unsafe impl Sync for Disk {}

/// 仅验收镜像保存真实队列事件计数，不参与正常请求协议。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct TestSnapshot {
    pub submitted: usize,
    pub completed: usize,
    pub data_submitted: usize,
    pub metadata_submitted: usize,
    pub flush_submitted: usize,
    /// 实际接受且承载文件原页读入的设备请求数。
    pub file_read_submitted: usize,
    pub peak_inflight: usize,
    pub notifications: usize,
    pub resource_parks: usize,
    pub splits: usize,
    pub merges: usize,
    pub budget_continuations: usize,
    pub inflight: usize,
    /// 原 IRQ action 实际读取并确认本设备 ISR 的次数。
    #[cfg(feature = "ktest")]
    pub irq_calls: usize,
    /// 实际 ISR 含 QUEUE 位的次数，与完成请求数分开。
    #[cfg(feature = "ktest")]
    pub irq_queue: usize,
    /// 实际 ISR 完全为空的次数，不包含仅配置变更的中断。
    #[cfg(feature = "ktest")]
    pub irq_empty: usize,
}

/// 一次性错误仅在匹配请求的真实 used 被认领后替换上层完成结果。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TestCompletionKind {
    Data,
    Metadata,
    Flush,
    FileRead,
}

#[cfg(feature = "ktest")]
impl TestCompletionKind {
    /// 按原 folio 来源区分读入与 WB，raw 请求不参与文件内容分类。
    fn matches(self, resources: &SlotResources) -> bool {
        match self {
            Self::Flush => resources.operation == BlkOperation::Flush,
            Self::FileRead => {
                resources.operation == BlkOperation::Read
                    && resources
                        .parts
                        .iter()
                        .flatten()
                        .any(|part| part.bio.ktest_file_read(part.offset) == Some(true))
            }
            Self::Data | Self::Metadata => {
                resources.operation == BlkOperation::Write
                    && resources.parts.iter().flatten().any(|part| {
                        part.bio.ktest_writeback_metadata(part.offset)
                            == Some(self == Self::Metadata)
                    })
            }
        }
    }
}

/// 停机用例只观察本盘生产转换；拒读确认开关不伪造硬件状态。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct StopTestState {
    pub(super) pause_used: bool,
    pub(super) withhold_confirmation: bool,
    pub(super) published: usize,
    pub(super) finished: usize,
    pub(super) reset_requests: usize,
    pub(super) reset_confirmations: usize,
}

impl Disk {
    /// 在普通 Task 中完整准备固定表与全部 Work；此时硬件尚不知道任何队列地址。
    pub fn prepare(device: VirtioDevice, workqueue: &WorkQueue) -> Result<DiskRef, BlockError> {
        let limits = virtio_limits(device.info())?;
        Self::prepare_with_limits(
            device,
            workqueue,
            limits,
            #[cfg(feature = "ktest")]
            None,
        )
    }

    /// 只有构造期能降低测试限额，固定 Disk 发表后不再修改共享能力。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_prepare_with_limits(
        device: VirtioDevice,
        workqueue: &WorkQueue,
        max_bytes: usize,
        max_segments: usize,
        max_segment_bytes: u32,
        dma_boundary_mask: u64,
    ) -> Result<DiskRef, BlockError> {
        let mut limits = virtio_limits(device.info())?;
        if max_bytes < limits.logical_block_size as usize
            || limits.max_bytes < max_bytes
            || max_bytes % limits.logical_block_size as usize != 0
            || max_segments == 0
            || limits.max_segments < max_segments
            || max_segment_bytes == 0
            || limits.max_segment_bytes < max_segment_bytes
            || (dma_boundary_mask != u64::MAX
                && !dma_boundary_mask.wrapping_add(1).is_power_of_two())
        {
            return Err(BlockError::InvalidRange);
        }

        limits.max_bytes = max_bytes;
        limits.max_segments = max_segments;
        limits.max_segment_bytes = max_segment_bytes;
        limits.dma_boundary_mask = dma_boundary_mask;
        Self::prepare_with_limits(
            device,
            workqueue,
            limits,
            #[cfg(feature = "ktest")]
            None,
        )
    }

    /// 串行验收通过本次调用的栈输出保存实际 SG reserve 失败，不武装全局分配器。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_prepare_observed(
        device: VirtioDevice,
        workqueue: &WorkQueue,
        observation: &mut Option<(u16, crate::drivers::virtio::TestPrepareSnapshot)>,
    ) -> Result<DiskRef, BlockError> {
        assert!(observation.is_none(), "stale SG preparation observation");
        let limits = virtio_limits(device.info())?;
        Self::prepare_with_limits(device, workqueue, limits, Some(observation))
    }

    /// 锁外构造唯一固定表，生产和受控限额镜像共用全部实际资源路径。
    fn prepare_with_limits(
        device: VirtioDevice,
        workqueue: &WorkQueue,
        limits: QueueLimits,
        #[cfg(feature = "ktest")] observation: Option<
            &mut Option<(u16, crate::drivers::virtio::TestPrepareSnapshot)>,
        >,
    ) -> Result<DiskRef, BlockError> {
        assert!(
            !device.requires_reset(),
            "preparing an already published disk"
        );
        let backend = Backend::virtio(
            device,
            limits,
            #[cfg(feature = "ktest")]
            observation,
        )?;
        Self::prepare_backend(backend, workqueue, limits)
    }

    /// 让唯一 RAM 磁盘访问专用范围，不转移原范围的归还权或创建 DMA 控制区。
    ///
    /// # Safety
    /// region 须覆盖完整、独占且直接映射有效的 RAM。来源可以是 MM 接管前保留的 initrd，
    /// 或调用者持有的独占原页；原页拥有者必须保活到本 Disk 真实 stop、Work/IRQ 及全部
    /// CPU 复制尾部结束。发表期间调用者和固件不得访问该范围，也不得将其映射给用户或
    /// 其他设备；只持有原页而保留冲突的 CPU 裸别名不能满足此契约。
    pub unsafe fn prepare_ram(
        region: crate::boot::Region,
        workqueue: &WorkQueue,
    ) -> Result<DiskRef, BlockError> {
        let limits = Backend::ram_limits(region)?;
        Self::prepare_backend(Backend::ram(region)?, workqueue, limits)
    }

    /// 用同一固定请求核心准备真实 SD，原 payload 只交 CPU 复制访问期；无卡不建立 Disk。
    ///
    /// # Safety
    /// 资源来自唯一快照，控制器及其 clock/reset/pin/GPIO/syscon 完整永久映射且由调用者独占；
    /// 外部 owner 必须保本 Disk 到 IRQ、命令、Work 和真实停止全部结束。
    pub(crate) unsafe fn prepare_sdmmc(
        resources: &crate::boot::MmcResources,
        workqueue: &WorkQueue,
    ) -> Result<Option<DiskRef>, BlockError> {
        // SAFETY: 调用者传递真实资源独占、映射和 IRQ/停止的完整寿命义务。
        let Some((backend, limits)) = unsafe { Backend::sdmmc(resources) }? else {
            return Ok(None);
        };
        Self::prepare_backend(backend, workqueue, limits).map(Some)
    }

    /// 用真实 AHCI 地址域准备直接原页请求；真实无介质不建立 Disk，身份仍取同一库存。
    ///
    /// # Safety
    /// ABAR、IRQ、DMA 范围须来自唯一资源解析，固件已退出且完整强序映射永久有效；
    /// 调用者独占本端口，保本 Disk 到全部 DMA、IRQ、Work 和真实停止尾部结束。
    pub(crate) unsafe fn prepare_ahci(
        resources: &crate::boot::AhciResources,
        workqueue: &WorkQueue,
    ) -> Result<Option<DiskRef>, BlockError> {
        // SAFETY: 调用者证明解析后的真实功能资源和完整拥有寿命。
        let Some((backend, limits)) = unsafe { Backend::ahci(resources) }? else {
            return Ok(None);
        };
        Self::prepare_backend(backend, workqueue, limits).map(Some)
    }

    /// 构造协议无关的固定软件槽和预留完成任务，所有 fallible 准备先于硬件发表。
    fn prepare_backend(
        backend: Backend,
        workqueue: &WorkQueue,
        limits: QueueLimits,
    ) -> Result<DiskRef, BlockError> {
        let identity = backend.identity();
        let domain = backend.domain();
        let mut slots = Vec::new();
        let mut slot_states = Vec::new();
        let mut slot_writes = Vec::new();
        slots
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        slot_states
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        slot_writes
            .try_reserve_exact(usize::from(limits.tag_count))
            .map_err(|_| BlockError::NoMemory)?;
        for tag in 0..limits.tag_count {
            let mut sg = Vec::new();
            sg.try_reserve_exact(limits.max_segments)
                .map_err(|_| BlockError::NoMemory)?;
            slots.push(SlotStorage {
                resources: UnsafeCell::new(SlotResources {
                    sg,
                    parts: core::array::from_fn(|_| None),
                    part_count: 0,
                    operation: BlkOperation::Read,
                    sector: 0,
                    bytes: 0,
                    forced_error: None,
                    completion: None,
                    #[cfg(feature = "ktest")]
                    ktest_read_prefix: None,
                    configured: false,
                }),
                completion_work: None,
                tag,
                disk: ptr::null(),
                control: backend.slot_control(tag),
            });
            slot_states.push(SlotState::Free);
            slot_writes.push(None);
        }

        let disk = Arc::try_new(Self {
            queue: SpinLock::new(QueueState {
                device: Some(backend),
                phase: QueuePhase::Prepared,
                failure: None,
                generators: 0,
                writer_generators: 0,
                active_slots: 0,
                writing_slots: 0,
                notify_active: 0,
                pending_notify: false,
                mounts: 0,
                slot_states,
                claim_head: ptr::null(),
                raw_writers: ptr::null(),
                slot_writes,
                #[cfg(feature = "ktest")]
                counters: TestSnapshot::default(),
                #[cfg(feature = "ktest")]
                test_paused: false,
                #[cfg(feature = "ktest")]
                ktest_stop: StopTestState::default(),
                #[cfg(feature = "ktest")]
                ktest_folio_dma: None,
                #[cfg(feature = "ktest")]
                ktest_completion_error: None,
                #[cfg(feature = "ktest")]
                ktest_hold_file_writeback_page: None,
                #[cfg(feature = "ktest")]
                ktest_held_file_writeback_slots: [None; MAX_BATCH],
            }),
            control: SleepMutex::new(()),
            irq: SpinLock::new(None),
            progress_work: None,
            slots,
            available: WaitQueue::new(),
            quiescent: WaitQueue::new(),
            limits,
            identity,
            domain,
            metadata_cache: SpinLock::new(Weak::new()),
            _pinned: PhantomPinned,
        })
        .map_err(|_| BlockError::NoMemory)?;

        let raw = Arc::into_raw(disk);
        // SAFETY: into_raw 的唯一强引用尚未发表；构造过程中不队列化 Work，不注册 IRQ。
        // 原 Arc 立刻恢复，失败仍以 Prepared 正常析构；固定 Vec 不再重新分配。
        let disk = unsafe { Pin::new_unchecked(Arc::from_raw(raw)) };
        let progress = Work::try_new_reserved(workqueue, 0, progress_callback, raw as usize)
            .map_err(|_| BlockError::NoMemory)?;
        // SAFETY: 还没有第二份 Disk 引用或异步票；写入不移动 Disk 或槽 Vec。
        unsafe { (*raw.cast_mut()).progress_work = Some(progress) };
        for index in 0..usize::from(limits.tag_count) {
            // SAFETY: 固定 Vec 已完整构造且从不增长；仅未发表构造者访问此槽。
            let slot = unsafe { &mut (&mut (*raw.cast_mut()).slots)[index] };
            slot.disk = raw;
            slot.completion_work = Some(
                Work::try_new_reserved(
                    workqueue,
                    0,
                    completion_callback,
                    slot as *const SlotStorage as usize,
                )
                .map_err(|_| BlockError::NoMemory)?,
            );
        }

        Ok(disk)
    }

    /// 串行注册真实 disabled action，再发表硬件；失败仍保留 owner 供显式停止。
    pub fn start(&self) -> Result<(), BlockError> {
        let _control = self.control.lock();
        let number = {
            let q = self.queue.lock();
            if q.phase != QueuePhase::Prepared {
                return Err(BlockError::NotReady);
            }
            q.device
                .as_ref()
                .expect("prepared disk has no device")
                .interrupt()
        };

        if let Some(number) = number {
            // SAFETY: 私有构造器返回 pinned Arc，外部 owner 显式 stop 并同步注销后才释放。
            let registration =
                unsafe { irq::register(number, irq_callback, self as *const Self as *mut ()) }
                    .map_err(|_| BlockError::Busy)?;
            *self.irq.lock() = Some(registration);
        }

        let result = {
            let mut q = self.queue.lock();
            let result = q
                .device
                .as_mut()
                .expect("prepared disk has no device")
                .publish();
            match result {
                Ok(()) => {
                    let backend = q.device.as_mut().expect("published disk has no backend");
                    if backend.uses_work() {
                        backend.set_interrupts(true);
                    }
                    q.phase = QueuePhase::Live;
                    Ok(())
                }
                Err(error) => {
                    q.failure = Some(error);
                    q.phase = QueuePhase::Faulting;
                    Err(error)
                }
            }
        };

        if result.is_ok() {
            if let Some(registration) = self.irq.lock().as_ref() {
                registration.enable();
            }
        }
        result
    }

    /// 返回 Bio 构造时绑定的真实设备身份。
    pub fn identity(&self) -> DiskIdentity {
        self.identity
    }

    /// 返回本次协商的限额，不在请求路径重新读取配置。
    pub fn limits(&self) -> QueueLimits {
        self.limits
    }

    /// 取得本磁盘尚存活的共同元数据缓存；不在块层构造或解析文件系统。
    pub(crate) fn metadata_cache(&self) -> Option<Arc<MetadataPages>> {
        self.metadata_cache.lock().upgrade()
    }

    /// 候选由缓存层锁外构造，调用方须保证属于本磁盘并检查预算相容；短锁双查只发布弱引用。
    /// 未采用的候选仍归调用方，旧弱引用在锁外释放。
    pub(crate) fn intern_metadata_cache(
        &self,
        candidate: &Arc<MetadataPages>,
    ) -> Arc<MetadataPages> {
        let retired = {
            let mut slot = self.metadata_cache.lock();
            if let Some(existing) = slot.upgrade() {
                return existing;
            }
            core::mem::replace(&mut *slot, Arc::downgrade(candidate))
        };
        drop(retired);
        Arc::clone(candidate)
    }

    /// 析构者只摘自身 payload 身份的弱槽，复用后的新登记保持不变；返回的 Weak 须锁外释放。
    pub(crate) fn unregister_metadata_cache(
        &self,
        identity: *const MetadataPages,
    ) -> Option<Weak<MetadataPages>> {
        let mut slot = self.metadata_cache.lock();
        if slot.as_ptr() == identity {
            Some(core::mem::take(&mut *slot))
        } else {
            None
        }
    }

    /// 非 slot 的 Task 最后义务复用原域，并在 Bio 构造期预留独占执行栈及一次性尾部。
    pub(super) fn prepare_finalizer(
        &self,
        callback: fn(Pin<&Work>),
        context: usize,
    ) -> Result<Pin<Arc<Work>>, BlockError> {
        self.progress_work
            .as_ref()
            .expect("prepared disk has no progress work")
            .try_new_related_reserved(callback, context)
            .map_err(|_| BlockError::NoMemory)
    }

    /// 普通生成者或其独立完成 Work 在真实回调尾部归还同一 accepted 责任。
    pub(super) fn finish_generation_tail(&self, bio: &BioInner) {
        {
            let mut q = self.queue.lock();
            assert!(q.generators != 0, "block generator tail ended twice");
            q.generators -= 1;
            if bio.operation() == BlkOperation::Write {
                q.writer_generators -= 1;
                let mut link = &mut q.raw_writers as *mut *const BioInner;
                // SAFETY: accepted 生成者保活整条链，所有链接只在此 queue 锁下修改。
                unsafe {
                    while !(*link).is_null() {
                        if ptr::eq(*link, bio) {
                            *link = *bio.raw_writer_next.get();
                            *bio.raw_writer_next.get() = ptr::null();
                            break;
                        }
                        link = (*(*link)).raw_writer_next.get();
                    }
                }
            }
        }
        self.quiescent.wake_all();
    }

    /// raw 准入没有文件系统授权，与 FS 提交共用原生成者和 slot 核心。
    #[cfg(feature = "ktest")]
    pub(super) fn admit<'a>(
        self: &'a DiskRef,
        bio: Bio,
    ) -> Result<ActiveBio<'a>, (Bio, BlockError)> {
        self.admit_from(bio, None)
    }

    /// Live 下核对实际磁盘和写来源，再在同一队列短锁中接受 root。
    /// claim 由借用来源保活到锁外移交，复杂最后 put 不发生在准入锁中。
    pub(super) fn admit_from<'a>(
        self: &'a DiskRef,
        bio: Bio,
        claim: Option<&FsClaim>,
    ) -> Result<ActiveBio<'a>, (Bio, BlockError)> {
        {
            let mut q = self.queue.lock();
            let error = if !matches!(q.phase, QueuePhase::Live) {
                Some(q.stopped_error())
            } else if bio.inner.target() != self.identity
                || claim.is_some_and(|claim| {
                    !ptr::eq(&**claim.disk(), &**self)
                        || (bio.inner.operation() != BlkOperation::Flush
                            && !claim
                                .view()
                                .contains_bytes(bio.inner.sector() * SECTOR_BYTES, bio.inner.len()))
                })
            {
                Some(BlockError::InvalidRange)
            } else if bio.inner.operation() == BlkOperation::Write
                && (self.limits.readonly
                    || match claim {
                        Some(claim) => claim.mode() != AccessMode::ReadWrite,
                        None => crate::block::stop::mounted_write_conflicts(
                            &q,
                            (
                                bio.inner.sector(),
                                bio.inner.sector()
                                    + (bio.inner.len() / SECTOR_BYTES as usize) as u64,
                            ),
                        ),
                    })
            {
                Some(BlockError::ReadOnly)
            } else {
                None
            };
            if let Some(error) = error {
                return Err((bio, error));
            }

            q.generators += 1;
            if bio.inner.operation() == BlkOperation::Write {
                q.writer_generators += 1;
                if claim.is_none() {
                    // SAFETY: 本 Bio 尚未发表，accepted 生成者保活到同锁摘链；不在锁内增加 Arc。
                    unsafe {
                        *bio.inner.raw_writer_next.get() = q.raw_writers;
                    }
                    q.raw_writers = &*bio.inner;
                }
            }
        }

        if let Err(error) = bio.inner.begin() {
            self.finish_generation_tail(&bio.inner);
            return Err((bio, error));
        }
        if let Some(claim) = claim {
            bio.inner.retain_fs_claim(claim);
        }
        Ok(ActiveBio {
            bio: bio.inner,
            offset: 0,
            disk: self,
            fs_claim: claim.cloned(),
            finished: false,
        })
    }

    /// 先 kick 已接受前缀，再栈登记、复查、真实 park；排空只拒新 root。
    pub(super) fn reserve_slot(&self) -> Result<PreparingSlot<'_>, BlockError> {
        loop {
            if let Some(slot) = self.try_reserve_slot()? {
                return Ok(slot);
            }
            self.unplug();
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 与栈 pin 节点一直保活，registration 在本轮任何退出前先撤链。
            let registration = unsafe { self.available.register(node.as_mut()) };
            if let Some(slot) = self.try_reserve_slot()? {
                return Ok(slot);
            }
            self.count_park();
            waiter.park();
            drop(registration);
        }
    }

    /// 在唯一 queue 锁中把空闲 tag 交给一个准备者，不触碰任何槽资源。
    fn try_reserve_slot(&self) -> Result<Option<PreparingSlot<'_>>, BlockError> {
        let mut q = self.queue.lock();
        if !q.accepts_children() {
            return Err(q.stopped_error());
        }
        let Some(index) = q
            .slot_states
            .iter()
            .position(|state| *state == SlotState::Free)
        else {
            return Ok(None);
        };
        q.slot_states[index] = SlotState::Preparing;
        q.active_slots += 1;
        Ok(Some(PreparingSlot {
            disk: self,
            tag: index as u16,
            published: false,
        }))
    }

    /// 在锁内认领唯一通知责任，锁外 doorbell 的真实尾部阻止并发 reset。
    pub(super) fn unplug(&self) {
        let notification = {
            let mut q = self.queue.lock();
            if !q.accepts_children() || !q.pending_notify {
                return;
            }
            q.pending_notify = false;
            let backend = q.device.as_ref().expect("live disk has no device");
            if backend.uses_work() {
                drop(q);
                self.queue_progress();
                return;
            }
            let Some(notification) = backend.notification() else {
                return;
            };
            q.notify_active += 1;
            notification
        };
        // SAFETY: notify_active 保持设备与映射有效，reset 先关闭认领并等此计数归零。
        // 门铃前屏障发布描述符；门铃后屏障还必须先完成设备输出，再向另一CPU归还尾部计数。
        unsafe { notification.notify::<KernelHal>() };
        KernelHal::dma_barrier();
        {
            let mut q = self.queue.lock();
            q.notify_active -= 1;
            #[cfg(feature = "ktest")]
            {
                q.counters.notifications += 1;
            }
        }
        self.quiescent.wake_all();
    }

    /// 关闭发表和通知认领；原生命令仍执行时保留 IRQ，由执行尾或真实 reset 关闭。
    pub(super) fn mark_fault(&self, error: BlockError) {
        let claimed = {
            let mut q = self.queue.lock();
            if matches!(
                q.phase,
                QueuePhase::Prepared | QueuePhase::Live | QueuePhase::Draining
            ) {
                q.failure = Some(error);
                q.phase = QueuePhase::Faulting;
                q.pending_notify = false;
                let executing = q
                    .slot_states
                    .iter()
                    .any(|state| *state == SlotState::Executing);
                if let Some(device) = q.device.as_mut() {
                    if !device.uses_work() || !executing {
                        device.set_interrupts(false);
                    }
                }
                true
            } else {
                false
            }
        };
        self.available.wake_all();
        self.quiescent.wake_all();
        if claimed {
            self.queue_progress();
        }
    }

    /// 续处理使用预建 Work 的现存票协议，不在 IRQ 分配。
    pub(super) fn queue_progress(&self) {
        if let Some(work) = &self.progress_work {
            let _ = work.as_ref().queue();
        }
    }

    /// 本设备真实 ISR 决定共享线处理归属，配置变更与 NEEDS_RESET 都进入故障停机。
    fn interrupt(&self) -> bool {
        let (cause, live) = {
            let mut q = self.queue.lock();
            let live = q.accepts_children();
            let Some(device) = q.device.as_mut() else {
                return false;
            };
            let cause = device.acknowledge();
            #[cfg(feature = "ktest")]
            {
                q.counters.irq_calls += 1;
                if cause.completion {
                    q.counters.irq_queue += 1;
                } else if !cause.claimed {
                    q.counters.irq_empty += 1;
                }
            }
            (cause, live)
        };
        if live && cause.fault {
            self.request_retire(BlockError::DeviceFault);
        }
        if live && cause.completion && !cause.fault {
            self.drain_used();
        }
        cause.claimed || cause.fault
    }

    /// 唯一 progress Work 在锁外执行 RAM 复制或一个真实原生命令，ISR 保持稳态 IRQ。
    fn progress_requests(&self) {
        crate::sched::assert_switchable();
        for _ in 0..IRQ_BUDGET {
            let (tag, target) = {
                let mut q = self.queue.lock();
                if !q.accepts_children() {
                    return;
                }
                let backend = q.device.as_mut().expect("live disk has no backend");
                let Some(tag) = backend.next_work_request() else {
                    return;
                };
                let target = backend
                    .work_target()
                    .expect("queued Work request has no execution target");
                assert_eq!(q.slot_states[usize::from(tag)], SlotState::Submitted);
                q.slot_states[usize::from(tag)] = SlotState::Executing;
                (tag, target)
            };
            // SAFETY: Submitted→Executing 唯一认领固定槽；stop 等执行尾部，IRQ 只操作 mailbox，
            // 不读取或修改本槽；原 parts 保 accepted payload 及实际 FS 占用。
            let resources = unsafe { &mut *self.slots[usize::from(tag)].resources.get() };
            let native = !matches!(&target, WorkTarget::Ram(_));
            let result = match target {
                WorkTarget::Ram(region) => {
                    let start = region.start + (resources.sector * SECTOR_BYTES) as usize;
                    // SAFETY: RAM 范围独占且不与 payload 重叠，正式限额已核完整物理范围；
                    // child 保来源到复制结束，此唯一 Work 不并行复制同一 RAM 通道。
                    unsafe {
                        copy_cpu_parts(resources, crate::arch::phys_to_virt(start) as *mut u8)
                    }
                }
                WorkTarget::Sdmmc(controller) => {
                    // SAFETY: 原 payload 在 CPU 复制期，child 保所有来源；bounce 由控制器独占，
                    // 回调只复制有界 parts，不分配、睡眠或反调设备，全部 IRQ 保持启用。
                    unsafe {
                        controller.execute(
                            resources.operation,
                            resources.sector,
                            resources.bytes,
                            |bounce| {
                                let result = copy_cpu_parts(resources, bounce);
                                match result.error {
                                    Some(error) => Err(error),
                                    None => Ok(result.completed),
                                }
                            },
                        )
                    }
                }
                WorkTarget::Ahci(controller) => {
                    // SAFETY: Bio 已按该控制器唯一 domain 交权，SG 只引用 accepted 原范围；
                    // Executing 和 controller Arc 保全部 owning 到硬件停止确认，CPU 不访问内容。
                    unsafe {
                        controller.execute(
                            resources.operation,
                            resources.sector,
                            resources.bytes,
                            &resources.sg,
                        )
                    }
                }
            };
            if result.error == Some(BlockError::StopUnconfirmed) {
                let mut q = self.queue.lock();
                assert_eq!(q.slot_states[usize::from(tag)], SlotState::Executing);
                q.failure = Some(BlockError::StopUnconfirmed);
                q.phase = QueuePhase::Quarantined;
                q.pending_notify = false;
                drop(q);
                // 停止未确认时不移出 parts、不完成 Bio、不收 payload；原后端仍在永久拥有槽。
                self.available.wake_all();
                self.quiescent.wake_all();
                return;
            }
            resources.completion = Some(result);
            {
                let mut q = self.queue.lock();
                assert_eq!(q.slot_states[usize::from(tag)], SlotState::Executing);
                q.slot_states[usize::from(tag)] = SlotState::Finishing;
            }
            self.quiescent.wake_all();
            self.available.wake_all();
            if native {
                if let Some(error) = result.error {
                    self.request_retire(error);
                }
            }
            // execute 已归还硬件通道，再分派可能等待同盘 Read 的可睡完成者。
            self.dispatch_slot(tag);
        }
        self.queue_progress();
    }

    /// 最多消费八个真实 used；重装并复查后由预建 Work 接续，不依赖下一次 IRQ。
    fn drain_used(&self) {
        let mut tags = [None; IRQ_BUDGET];
        let mut count = 0;
        let mut fault = None;
        let mut continuation = false;
        {
            let mut q = self.queue.lock();
            if !q.accepts_children() {
                return;
            }
            if q.device
                .as_ref()
                .expect("live disk has no backend")
                .uses_work()
            {
                return;
            }
            #[cfg(feature = "ktest")]
            if q.test_paused {
                return;
            }
            #[cfg(feature = "ktest")]
            if q.ktest_stop.pause_used {
                return;
            }
            q.device
                .as_mut()
                .expect("live disk has no device")
                .set_interrupts(false);
            while count < IRQ_BUDGET {
                let used = q
                    .device
                    .as_mut()
                    .expect("live disk has no device")
                    .pop_completion();
                match used {
                    Ok(Some((tag, result))) => {
                        if q.slot_states.get(usize::from(tag)) != Some(&SlotState::Submitted) {
                            fault = Some(BlockError::DeviceFault);
                            break;
                        }
                        q.slot_states[usize::from(tag)] = SlotState::Finishing;
                        // SAFETY: Submitted→Finishing 唯一认领，真实 used 已结束本槽设备访问；
                        // queue 锁保持槽不可复用，仅写本轮完成元数据，不访问 payload 内容。
                        let resources =
                            unsafe { &mut *self.slots[usize::from(tag)].resources.get() };
                        resources.completion = Some(result);
                        #[cfg(feature = "ktest")]
                        if let Some((kind, error, prefix)) = q.ktest_completion_error {
                            if kind.matches(resources) {
                                q.ktest_completion_error = None;
                                if let Some(completed) = prefix {
                                    resources.ktest_read_prefix = Some((completed, error));
                                } else {
                                    resources.forced_error = Some(error);
                                }
                            }
                        }
                        #[cfg(feature = "ktest")]
                        {
                            let held_page = q.ktest_hold_file_writeback_page;
                            let hold = held_page.is_some_and(|physical| {
                                resources.operation == BlkOperation::Write
                                    && resources.parts.iter().flatten().any(|part| {
                                        part.bio.ktest_writeback_metadata(part.offset)
                                            == Some(false)
                                            && part
                                                .bio
                                                .ktest_folio_origin(part.offset)
                                                .is_some_and(|(origin, _)| origin == physical)
                                    })
                            });
                            if hold {
                                // used 已推进 ring；仅保留目标 folio 的软件 WB 责任，其他 slot 继续派发。
                                let slot = q
                                    .ktest_held_file_writeback_slots
                                    .iter_mut()
                                    .find(|slot| slot.is_none())
                                    .expect("held file writeback slots exhausted");
                                *slot = Some(tag);
                            } else {
                                tags[count] = Some(tag);
                            }
                        }
                        #[cfg(not(feature = "ktest"))]
                        {
                            tags[count] = Some(tag);
                        }
                        count += 1;
                    }
                    Ok(None) => break,
                    Err(_) => {
                        fault = Some(BlockError::DeviceFault);
                        break;
                    }
                }
            }
            if fault.is_none() {
                continuation = q
                    .device
                    .as_mut()
                    .expect("live disk has no device")
                    .arm_and_check();
                #[cfg(feature = "ktest")]
                if continuation {
                    q.counters.budget_continuations += 1;
                }
            }
        }

        if let Some(error) = fault {
            self.mark_fault(error);
        }
        // descriptor 已经归还；准备者可以继续使用自己的 SG，无需等待回调释放 tag。
        if 0 < count {
            self.available.wake_all();
        }
        for tag in tags[..count].iter().flatten() {
            self.dispatch_slot(*tag);
        }
        if continuation {
            self.queue_progress();
        }
    }

    /// Task parts 始终交专用槽 Work，进度和 reset 只分派；轻完成仍直接结束。
    pub(super) fn dispatch_slot(&self, tag: u16) {
        let slot = &self.slots[usize::from(tag)];
        // SAFETY: 调用者唯一拥有 Submitted→Finishing 的认领，本槽不可准备或二次派发。
        let needs_task = unsafe {
            (*slot.resources.get())
                .parts
                .iter()
                .flatten()
                .any(|part| part.bio.needs_task())
        };
        if needs_task {
            slot.completion_work
                .as_ref()
                .expect("slot has no completion work")
                .as_ref()
                .queue()
                .expect("active slot completion work is closed");
        } else {
            self.finish_slot(tag);
        }
    }

    /// 在 Finishing 独占期锁外读取稳定 PDU，按真实前缀结束 parts，最后才归还 tag。
    pub(super) fn finish_slot(&self, tag: u16) {
        let slot = &self.slots[usize::from(tag)];
        // SAFETY: 本函数由唯一完成认领者或其专用 Work 调用；Finishing 阻止 prepare/reuse。
        let resources = unsafe { &mut *slot.resources.get() };
        let (completed, error) = completion_result(resources);
        if error == Some(BlockError::DeviceFault) {
            self.mark_fault(BlockError::DeviceFault);
        }

        let writing = resources.operation == BlkOperation::Write;
        let mut remaining = completed;
        let parts = core::mem::replace(&mut resources.parts, [const { None }; MAX_BATCH]);
        for part in parts.iter().flatten() {
            let progress = remaining.min(part.length);
            remaining -= progress;
            let part_error = if progress < part.length || part.length == 0 {
                error
            } else {
                None
            };
            BioInner::finish_child(&part.bio, part.offset, part.length, progress, part_error);
        }

        resources.sg.clear();
        resources.part_count = 0;
        resources.configured = false;
        resources.forced_error = None;
        resources.completion = None;
        #[cfg(feature = "ktest")]
        {
            resources.ktest_read_prefix = None;
        }

        // 从此处起不再访问本轮 slot 数据；其他 producer 在状态清空后可以马上重用。
        {
            let mut q = self.queue.lock();
            assert_eq!(q.slot_states[usize::from(tag)], SlotState::Finishing);
            q.slot_states[usize::from(tag)] = SlotState::Free;
            q.active_slots -= 1;
            if writing {
                q.writing_slots -= 1;
                q.slot_writes[usize::from(tag)] = None;
            }
            #[cfg(feature = "ktest")]
            {
                q.ktest_stop.finished += 1;
            }
            #[cfg(feature = "ktest")]
            {
                q.counters.completed += 1;
                q.counters.inflight -= 1;
            }
        }
        self.available.wake_all();
        self.quiescent.wake_all();
        // 完成观察者可早于 slot 尾被唤醒，所有 part 的占用仍保留到计数归还后。
        drop(parts);
    }

    /// 只有真实 reset 已确认才能撤 Submitted 的设备拥有期；已认领完成槽不会重复完成。
    pub(super) fn fail_submitted_after_reset(&self, error: BlockError) {
        for index in 0..self.slots.len() {
            let claimed = {
                let mut q = self.queue.lock();
                assert_eq!(q.phase, QueuePhase::DmaStopped);
                if q.slot_states[index] == SlotState::Submitted {
                    q.slot_states[index] = SlotState::Finishing;
                    // SAFETY: 真实 reset 已确认并屏障，本次转换唯一认领，设备不能再访问。
                    unsafe {
                        (*self.slots[index].resources.get()).forced_error = Some(error);
                    }
                    true
                } else {
                    false
                }
            };
            if claimed {
                self.dispatch_slot(index as u16);
            }
        }
        self.available.wake_all();
        self.quiescent.wake_all();
    }

    /// 返回验收镜像的真实计数快照，不读取活动 DMA。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_snapshot(&self) -> TestSnapshot {
        self.queue.lock().counters
    }

    /// 对下一次真实文件 WB used 注入一次上层完成错误，不改变硬件或准入。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_data(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::Data, error, None);
    }

    /// 对下一次真实文件原页读入的 used 注入一次上层完成错误。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_file_read(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::FileRead, error, None);
    }

    /// 真实文件读入成功 used 后仅交付指定扇区前缀，剩余部分保留一次软件完成错误。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_file_read_after(&self, completed: usize, error: BlockError) {
        assert!(0 < completed && completed % SECTOR_BYTES as usize == 0);
        self.test_arm_completion_error(TestCompletionKind::FileRead, error, Some(completed));
    }

    /// 对下一次真实共同元数据 used 注入一次完成错误。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_metadata(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::Metadata, error, None);
    }

    /// 对下一次真实 FLUSH used 注入一次完成错误，未协商能力仍走原拒绝。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_fail_next_flush(&self, error: BlockError) {
        self.test_arm_completion_error(TestCompletionKind::Flush, error, None);
    }

    /// 测试控制只保存本 Disk 的一次性匹配，不伪造设备停止或 reset 确认。
    #[cfg(feature = "ktest")]
    fn test_arm_completion_error(
        &self,
        kind: TestCompletionKind,
        error: BlockError,
        prefix: Option<usize>,
    ) {
        assert!(matches!(
            error,
            BlockError::Io | BlockError::ShortIo | BlockError::NoMemory
        ));
        let mut q = self.queue.lock();
        assert!(
            q.accepts_children(),
            "arming completion error on stopped disk"
        );
        assert!(
            q.ktest_completion_error.is_none(),
            "completion error already armed"
        );
        q.ktest_completion_error = Some((kind, error, prefix));
    }

    /// 观察一次性匹配是否仍未被真实 used 消费。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_completion_error_pending(&self) -> bool {
        self.queue.lock().ktest_completion_error.is_some()
    }

    /// 只在本例全部请求和回调结束后捕获实际 Work 票尾，不能用 idle 或定长sleep代替。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_flush_completions(&self) {
        if let Some(work) = &self.progress_work {
            work.as_ref().flush();
        }
        for slot in &self.slots {
            if let Some(work) = &slot.completion_work {
                work.as_ref().flush();
            }
        }
    }

    /// 成功发表后才累计额外 child 与跨 Bio 合并的真实数量。
    #[cfg(feature = "ktest")]
    pub(super) fn record_generation(&self, splits: usize, merges: usize) {
        let mut q = self.queue.lock();
        q.counters.splits += splits;
        q.counters.merges += merges;
    }

    /// 指定文件原页真实 Data-WB 的used到达后暂留其软件完成责任；其他I/O仍沿原IRQ与完成路径。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_hold_file_writeback(&self, folio: &crate::mm::FolioRef) {
        let mut q = self.queue.lock();
        assert!(
            q.accepts_children(),
            "holding file writeback on a stopped disk"
        );
        assert!(
            !q.test_paused,
            "file writeback hold cannot combine with queue pause"
        );
        assert!(
            q.ktest_hold_file_writeback_page.is_none()
                && q.ktest_held_file_writeback_slots
                    .iter()
                    .all(Option::is_none),
            "file writeback hold is already active"
        );
        q.ktest_hold_file_writeback_page = Some(folio.phys_addr());
    }

    /// 只有原页身份匹配的真实 used slot 已进入 Finishing 时才返回真。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_file_writeback_is_held(&self, folio: &crate::mm::FolioRef) -> bool {
        let q = self.queue.lock();
        q.ktest_hold_file_writeback_page == Some(folio.phys_addr())
            && q.ktest_held_file_writeback_slots
                .iter()
                .any(Option::is_some)
    }

    /// 暂停本 queue 的完成消费，仍确认真实 ISR，不 mask 共享源也不伪造 used。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_suspend_completions(&self) {
        let mut q = self.queue.lock();
        assert!(q.accepts_children(), "pausing a stopped disk");
        q.test_paused = true;
        q.device
            .as_mut()
            .expect("live disk has no device")
            .set_interrupts(false);
    }

    /// 恢复 queue 完成并重派已收 used 的暂留 slot，再用预建 Work 接续其他真实 used。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_resume_completions(&self) {
        let (pending, held) = {
            let mut q = self.queue.lock();
            q.test_paused = false;
            let held =
                core::mem::replace(&mut q.ktest_held_file_writeback_slots, [None; MAX_BATCH]);
            q.ktest_hold_file_writeback_page = None;
            let pending = if q.accepts_children() {
                q.device
                    .as_mut()
                    .expect("live disk has no device")
                    .arm_and_check()
            } else {
                false
            };
            (pending, held)
        };

        for tag in core::iter::IntoIterator::into_iter(held).flatten() {
            self.dispatch_slot(tag);
        }
        if pending {
            self.queue_progress();
        }
    }

    /// 已有真实 used 时交预建进度 Work；保持 NO_INTERRUPT 到生产 drain 重装。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_resume_via_progress(&self) {
        self.queue.lock().test_paused = false;
        self.queue_progress();
    }

    /// 本例暂停真实 used 消费；仍允许设备 DMA 与 ISR，不改变共享 IRQ 线。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_hold_used(&self) {
        let mut q = self.queue.lock();
        assert_eq!(q.phase, QueuePhase::Live);
        q.ktest_stop.pause_used = true;
        q.device
            .as_mut()
            .expect("live stop fixture")
            .set_interrupts(false);
    }

    /// 撤销本例暂停并交原进度 Work 消费真实 used；故障阶段仍不收未确认 DMA。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_release_used(&self) {
        self.queue.lock().ktest_stop.pause_used = false;
        self.queue_progress();
    }

    /// 返回最近成功发表的单 folio 首 SG：原页物理地址、设备地址、页内偏移和字节数。
    /// 观测者须串行安排该盘 producer，后续原页 I/O 会替换这一记录。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_last_folio_dma(&self) -> Option<(usize, u64, usize, usize)> {
        self.queue.lock().ktest_folio_dma
    }

    /// 只在登记后复查仍无额度、即将调用真实 park 时计数。
    fn count_park(&self) {
        #[cfg(feature = "ktest")]
        {
            self.queue.lock().counters.resource_parks += 1;
        }
    }
}

impl Drop for Disk {
    /// 析构不代替显式停止，IRQ 与 Work raw context 必须先完成全部真实尾部。
    fn drop(&mut self) {
        let q = self.queue.lock();
        assert!(
            matches!(q.phase, QueuePhase::Prepared | QueuePhase::Stopped),
            "dropping an active disk"
        );
        assert!(self.irq.lock().is_none(), "dropping a registered disk");
        assert_eq!(q.generators, 0);
        assert_eq!(q.active_slots, 0);
        assert_eq!(q.notify_active, 0);
    }
}

pub(super) struct ActiveBio<'a> {
    pub(super) bio: BioArc,
    pub(super) offset: usize,
    disk: &'a DiskRef,
    finished: bool,
    fs_claim: Option<FsClaim>,
}

impl ActiveBio<'_> {
    /// 普通 Task 结束父生成者，未知 callback 返回后才减少 accepted 责任。
    pub(super) fn finish(mut self, error: Option<(usize, BlockError)>) {
        self.finish_inner(error);
    }

    /// 若生成者是最后义务，将可睡尾部交独立 Work，不阻塞其它父的生成结束。
    fn finish_inner(&mut self, error: Option<(usize, BlockError)>) {
        if self.finished {
            return;
        }
        let deferred = BioInner::finish_generation(&self.bio, error, Some(self.disk));
        self.finished = true;
        if !deferred {
            self.disk.finish_generation_tail(&self.bio);
        }
        // 延后完成由 BioState 继续保活；同步完成在 accepted 队列尾后才解除本引用。
        drop(self.fs_claim.take());
    }
}

impl Drop for ActiveBio<'_> {
    /// 任意生成错误出口仍结束生成者义务，错误位置保已经完整发表的逻辑前缀。
    fn drop(&mut self) {
        if !self.finished {
            self.disk.unplug();
        }
        self.finish_inner(Some((self.offset, BlockError::QueueStopped)));
    }
}

pub(super) struct PreparingSlot<'a> {
    disk: &'a Disk,
    tag: u16,
    published: bool,
}

impl PreparingSlot<'_> {
    /// guard 唯一拥有 Preparing，资源引用不能长于 guard 的当前借用。
    fn resources(&self) -> &SlotResources {
        // SAFETY: guard 唯一且尚未发表，设备和其他 CPU 都不访问此槽资源。
        unsafe { &*self.disk.slots[usize::from(self.tag)].resources.get() }
    }

    /// Preparing 只有一个 Task 修改固定 SG 和 parts，queue 锁不携带这些引用。
    fn resources_mut(&mut self) -> &mut SlotResources {
        // SAFETY: &mut guard 排除同一准备者内别名；状态尚未 Submitted/Finishing。
        unsafe { &mut *self.disk.slots[usize::from(self.tag)].resources.get() }
    }

    /// 借用当前准备中的小 SG 元数据，不借 DMA payload。
    pub(super) fn sg(&self) -> &[PayloadRange] {
        &self.resources().sg
    }

    /// SG capacity 已在 prepare 完整预留，调用者不得 reserve 或超出 limits 增长。
    pub(super) fn sg_mut(&mut self) -> &mut Vec<PayloadRange> {
        &mut self.resources_mut().sg
    }

    /// 固定 parts 限额供显式 batch 判定，尚未登记任何 child。
    pub(super) fn part_count(&self) -> usize {
        self.resources().part_count
    }

    /// 保存一个原 Bio 的有界责任，child 仅在真正发表前登记。
    pub(super) fn push_part(
        &mut self,
        bio: BioArc,
        offset: usize,
        length: usize,
    ) -> Result<(), BlockError> {
        let resources = self.resources_mut();
        if resources.part_count == MAX_BATCH
            || bio.len() < offset.checked_add(length).ok_or(BlockError::InvalidRange)?
        {
            return Err(BlockError::InvalidRange);
        }
        let fs_claim = bio.fs_claim();
        resources.parts[resources.part_count] = Some(RequestPart {
            bio,
            offset,
            length,
            _fs_claim: fs_claim,
        });
        resources.part_count += 1;
        Ok(())
    }

    /// 在 queue 锁外准备本轮稳定 PDU，原视图没有 driver 借用跨锁别名。
    pub(super) fn configure(
        &mut self,
        operation: BlkOperation,
        sector: u64,
        bytes: usize,
    ) -> Result<(), BlockError> {
        let limits = self.disk.limits;
        let resources = self.resources();
        if resources.part_count == 0
            || limits.max_segments < resources.sg.len()
            || limits.max_bytes < bytes
        {
            return Err(BlockError::InvalidRange);
        }
        let sg_bytes = resources
            .sg
            .iter()
            .try_fold(0usize, |total, segment| {
                total.checked_add(segment.length as usize)
            })
            .ok_or(BlockError::InvalidRange)?;
        let part_bytes = resources
            .parts
            .iter()
            .flatten()
            .try_fold(0usize, |total, part| total.checked_add(part.length))
            .ok_or(BlockError::InvalidRange)?;
        if sg_bytes != bytes
            || part_bytes != bytes
            || (operation == BlkOperation::Flush
                && (sector != 0 || bytes != 0 || !resources.sg.is_empty()))
            || (operation != BlkOperation::Flush
                && (bytes == 0 || bytes % limits.logical_block_size as usize != 0))
        {
            return Err(BlockError::InvalidRange);
        }

        let slot = &self.disk.slots[usize::from(self.tag)];
        slot.control.prepare(operation, sector)?;
        let resources = self.resources_mut();
        resources.operation = operation;
        resources.sector = sector;
        resources.bytes = bytes;
        resources.forced_error = None;
        resources.completion = None;
        #[cfg(feature = "ktest")]
        {
            resources.ktest_read_prefix = None;
        }
        resources.configured = true;
        Ok(())
    }

    /// descriptor 暂满保留整份锁外准备，先 kick、栈登记复查，再真实 park。
    pub(super) fn publish(mut self) -> Result<(), BlockError> {
        if !self.resources().configured {
            return Err(BlockError::InvalidRange);
        }

        loop {
            if self.try_publish()? {
                self.published = true;
                return Ok(());
            }
            self.disk.unplug();
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 栈 pin 节点和 disk 借用覆盖此轮，registration 在任何退出先撤链。
            let registration = unsafe { self.disk.available.register(node.as_mut()) };
            if self.try_publish()? {
                self.published = true;
                return Ok(());
            }
            self.disk.count_park();
            waiter.park();
            drop(registration);
        }
    }

    /// queue 锁中登记 child 并发表真实 avail，安装 head→tag 后才让 IRQ 观察。
    fn try_publish(&mut self) -> Result<bool, BlockError> {
        let disk = self.disk;
        let tag = self.tag;
        let result = {
            let mut q = disk.queue.lock();
            let resources = self.resources();
            if !q.accepts_children() {
                return Err(q.stopped_error());
            }
            assert_eq!(q.slot_states[usize::from(tag)], SlotState::Preparing);
            if !q
                .device
                .as_ref()
                .expect("live disk has no device")
                .can_submit(resources.sg.len())
            {
                return Ok(false);
            }
            for part in resources.parts.iter().flatten() {
                part.bio.register_child(part.offset, part.length);
            }
            let result = q.device.as_mut().expect("live disk has no device").submit(
                tag,
                resources.operation,
                resources.bytes,
                &resources.sg,
            );
            if result.is_ok() {
                for part in resources.parts.iter().flatten() {
                    part.bio.accept_child(part.offset, part.length);
                }
                q.slot_states[usize::from(tag)] = SlotState::Submitted;
                q.pending_notify = true;
                if resources.operation == BlkOperation::Write {
                    q.writing_slots += 1;
                    if resources
                        .parts
                        .iter()
                        .flatten()
                        .any(|part| part._fs_claim.is_none())
                    {
                        q.slot_writes[usize::from(tag)] = Some((
                            resources.sector,
                            resources.sector + (resources.bytes / SECTOR_BYTES as usize) as u64,
                        ));
                    }
                }
                #[cfg(feature = "ktest")]
                {
                    q.ktest_stop.published += 1;
                    if let Some((part, segment)) =
                        resources.parts[0].as_ref().zip(resources.sg.first())
                    {
                        if let Some((physical, offset)) = part.bio.ktest_folio_origin(part.offset) {
                            q.ktest_folio_dma = Some((
                                physical,
                                segment.physical as u64,
                                offset,
                                segment.length as usize,
                            ));
                        }
                    }
                }
                #[cfg(feature = "ktest")]
                {
                    q.counters.submitted += 1;
                    if TestCompletionKind::Data.matches(resources) {
                        q.counters.data_submitted += 1;
                    }
                    if TestCompletionKind::Metadata.matches(resources) {
                        q.counters.metadata_submitted += 1;
                    }
                    if TestCompletionKind::Flush.matches(resources) {
                        q.counters.flush_submitted += 1;
                    }
                    if TestCompletionKind::FileRead.matches(resources) {
                        q.counters.file_read_submitted += 1;
                    }
                    q.counters.inflight += 1;
                    q.counters.peak_inflight = q.counters.peak_inflight.max(q.counters.inflight);
                }
            }
            result
        };

        match result {
            Ok(_) => Ok(true),
            Err(error) => {
                let fault = error == BlockError::DeviceFault;
                let resources = self.resources();
                for part in resources.parts.iter().flatten() {
                    BioInner::finish_child(&part.bio, part.offset, part.length, 0, Some(error));
                }
                if fault {
                    disk.request_retire(error);
                }
                disk.unplug();
                Err(error)
            }
        }
    }
}

impl Drop for PreparingSlot<'_> {
    /// 未发表槽只清普通准备数据，绝不读取或归还任何活动 DMA。
    fn drop(&mut self) {
        if self.published {
            return;
        }
        let disk = self.disk;
        let tag = self.tag;
        let resources = self.resources_mut();
        let parts = core::mem::replace(&mut resources.parts, [const { None }; MAX_BATCH]);
        resources.sg.clear();
        resources.part_count = 0;
        resources.configured = false;

        {
            let mut q = disk.queue.lock();
            assert_eq!(q.slot_states[usize::from(tag)], SlotState::Preparing);
            q.slot_states[usize::from(tag)] = SlotState::Free;
            q.active_slots -= 1;
        }
        disk.available.wake_all();
        disk.quiescent.wake_all();
        drop(parts);
    }
}

/// 协议后端归一真实完成，受控前缀只在成功的真实请求上生效。
fn completion_result(resources: &SlotResources) -> (usize, Option<BlockError>) {
    if let Some(error) = resources.forced_error {
        return (0, Some(error));
    }
    let result = resources
        .completion
        .expect("finishing slot has no normalized result");
    #[cfg(feature = "ktest")]
    if result.error.is_none() && resources.operation == BlkOperation::Read {
        if let Some((completed, error)) = resources.ktest_read_prefix {
            assert!(
                completed < resources.bytes,
                "controlled prefix has no failed tail"
            );
            return (completed, Some(error));
        }
    }
    (result.completed, result.error)
}

/// IRQ context 来自 pinned Disk，注销同步返回前由外部控制 owner 保活。
///
/// # Safety
/// context 必须是 register 保存的原 Disk，登记寿命内不可移动或析构。
unsafe fn irq_callback(context: *mut ()) -> bool {
    // SAFETY: IRQ 注册契约保活固定 Disk，执行引用直到此 handler 返回才归零。
    unsafe { &*context.cast::<Disk>() }.interrupt()
}

/// 普通 Work 续处理预算或执行唯一故障 reset，不取得外部 control 锁也不关闭自己。
fn progress_callback(work: Pin<&Work>) {
    // SAFETY: 外部 stop 在关闭此 Work 的全部票尾部后才释放 pinned Disk。
    let disk = unsafe { &*(work.context() as *const Disk) };
    let (phase, uses_work) = {
        let q = disk.queue.lock();
        (q.phase, q.device.as_ref().is_some_and(Backend::uses_work))
    };
    if matches!(
        phase,
        QueuePhase::Faulting | QueuePhase::Resetting | QueuePhase::DmaStopped
    ) {
        let _ = disk.fault_reset();
    } else if matches!(phase, QueuePhase::Live | QueuePhase::Draining) {
        if uses_work {
            disk.progress_requests();
        } else {
            disk.drain_used();
        }
    }
}

/// 专用完成 Work 只消费其本轮 Finishing 责任，槽释放后不再读任何旧数据。
fn completion_callback(work: Pin<&Work>) {
    // SAFETY: SlotStorage 位于固定 Vec，Disk 外部 owner 保活至此 Work close_sync 尾部。
    let slot = unsafe { &*(work.context() as *const SlotStorage) };
    let tag = slot.tag;
    // SAFETY: disk context 构造时写定，Work 关闭同步返回前 Disk 不释放。
    let disk = unsafe { &*slot.disk };
    disk.finish_slot(tag);
}

/// 一个已认领 child 的全部 parts 复制到连续 CPU 范围，错误保已完整复制的连续前缀。
///
/// # Safety
/// 调用者须独占覆盖 resources.bytes 的连续范围，且它不与任何原 payload 重叠。
/// 原 child 与 CPU 访问期保活到函数返回，唯一 progress Work 串行化通道和相交范围。
unsafe fn copy_cpu_parts(resources: &SlotResources, pointer: *mut u8) -> crate::block::IoResult {
    let mut copied = 0;
    for part in resources.parts.iter().flatten() {
        if part.length == 0 {
            continue;
        }
        // SAFETY: 计划范围已由 parts 总长度验证，原 child 未结束且对应 CPU 访问期仍有效；
        // 目标/源连续范围由调用者独占，固定槽资源不被 IRQ 或另一个 Work 访问。
        if let Err(error) = unsafe {
            part.bio
                .copy_cpu(part.offset, part.length, pointer.add(copied))
        } {
            return crate::block::IoResult {
                completed: copied,
                error: Some(error),
            };
        }
        copied += part.length;
    }
    crate::block::IoResult {
        completed: copied,
        error: None,
    }
}
