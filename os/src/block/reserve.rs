//! 同一请求核心的有限控制来源；原始存储只在真实分配器尾部结束后重用。

use alloc::alloc::Global;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::{AllocError, Allocator, Layout};
use core::pin::pin;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU64, Ordering};

use crate::block::bio::BioInner;
use crate::block::{BlockError, MAX_BATCH};
use crate::mm::dma::MappedPages;
use crate::mm::FolioRef;
use crate::sched;
use crate::sync::SpinLock;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// 每份 WB 父的映射数量同时限定其原始 Vec 存储。
pub(crate) const WB_MAPPINGS: usize = MAX_BATCH;
const BIO_SLOTS: usize = MAX_BATCH + 1;
const AUX_SLOT: usize = MAX_BATCH;

/// 普通与预留映射使用同一移动容器及分配器合同。
pub(crate) type BioMemory = Vec<MappedPages, BioAllocator>;

/// 一份真实 Global 分配的释放权；借用视图不复制释放责任。
struct RawBlock {
    pointer: NonNull<u8>,
    layout: Layout,
}

// SAFETY: raw 块始终由所属源独占释放；分配状态由源短锁保护，
// 已借出的块只由其真实 Arc/Vec 访问，源不解引用其中的用户值。
unsafe impl Send for RawBlock {}

impl RawBlock {
    /// 启用前在全部源锁外准备一份真实非零布局。
    fn allocate(layout: Layout) -> Result<Self, AllocError> {
        let memory = Global.allocate(layout)?;
        Ok(Self {
            pointer: memory.cast(),
            layout,
        })
    }

    /// 借出与真实请求布局一致的原始字节，不改变释放权。
    fn view(&self) -> NonNull<[u8]> {
        NonNull::slice_from_raw_parts(self.pointer, self.layout.size())
    }
}

impl Drop for RawBlock {
    /// 最后源拥有在锁外按原布局归还唯一 Global。
    fn drop(&mut self) {
        // SAFETY: 源末次拥有保证全部 Arc/Vec/lease 已结束；原首址及原布局
        // 从同一 Global.allocate 保存，raw 表没有第二份释放拥有。
        unsafe { Global.deallocate(self.pointer, self.layout) };
    }
}

/// 原块与真实 allocator 在途责任分开，不以对象 ready 推断归还。
struct Allocation {
    block: Option<RawBlock>,
    active: bool,
    retired_once: bool,
}

impl Allocation {
    /// 初始化尚未取得原块、没有分配拥有的记录。
    const fn new() -> Self {
        Self {
            block: None,
            active: false,
            retired_once: false,
        }
    }

    /// 检查原块已经准备且没有仍活的 allocator 拥有。
    fn available(&self) -> bool {
        self.block.is_some() && !self.active
    }
}

/// 整体准入 book 不含 Bio/Vec 借用；每次构造另有短 lease。
struct BioSlot {
    control: Allocation,
    payload: Allocation,
    booked: bool,
    leased: bool,
}

impl BioSlot {
    /// 初始化没有任何借出责任的完整单元。
    const fn new() -> Self {
        Self {
            control: Allocation::new(),
            payload: Allocation::new(),
            booked: false,
            leased: false,
        }
    }

    /// 两份分配器责任与构造 lease 均结束后才可再次构造。
    fn reusable(&self) -> bool {
        !self.leased && self.control.available() && self.payload.available()
    }
}

/// metadata 原页的基准拥有与不可变 Node 控制存储属于同一个单元。
struct MetadataSlot {
    folio: FolioRef,
    control: Allocation,
    leased: bool,
}

struct ReserveState {
    bios: [BioSlot; BIO_SLOTS],
    metadata: Vec<MetadataSlot>,
    ready: bool,
    stop_sequence: u64,
}

/// 有限 IO 原始存储不反向拥有请求、节点、缓存、卷或工作对象。
pub(crate) struct BioReserve {
    state: SpinLock<ReserveState>,
    available: WaitQueue,
}

/// 元数据 IO 保留本次业务进入前的真实来源边界，借用不制造另一份源拥有。
#[derive(Clone, Copy)]
pub(crate) struct MetadataIoContext<'a> {
    reserve: &'a Arc<BioReserve>,
    boundary: u64,
    view: Option<(&'a AtomicU64, u64)>,
}

impl<'a> MetadataIoContext<'a> {
    /// 已捕获的轮边界可由同一 storage 重建借用，禁止改采后继 activate 的当前值。
    pub(crate) fn from_boundary(reserve: &'a Arc<BioReserve>, boundary: u64) -> Self {
        Self {
            reserve,
            boundary,
            view: None,
        }
    }

    /// 保留本卷的单调停止边界，另一卷退休不会取消同物理来源上的当前业务。
    pub(crate) fn for_view(
        reserve: &'a Arc<BioReserve>,
        sequence: &'a AtomicU64,
        boundary: u64,
    ) -> Self {
        Self {
            reserve,
            boundary: reserve.state.lock().stop_sequence,
            view: Some((sequence, boundary)),
        }
    }

    /// 核同一真实源，跨 cache 的数值相同不能替代身份。
    pub(crate) fn belongs_to(self, reserve: &Arc<BioReserve>) -> bool {
        Arc::ptr_eq(self.reserve, reserve)
    }

    /// 开始内容读入前重查旧边界；已接受 IO 不由此检查撤销。
    pub(crate) fn check(self) -> Result<(), BlockError> {
        let state = self.reserve.state.lock();
        if !state.ready
            || state.stop_sequence != self.boundary
            || self
                .view
                .is_some_and(|(sequence, boundary)| sequence.load(Ordering::Acquire) != boundary)
        {
            Err(BlockError::NotReady)
        } else {
            Ok(())
        }
    }
}

/// 两种具体块只用于同一 Bio 控制与其映射 Vec。
#[derive(Clone, Copy)]
pub(crate) enum BioBlock {
    Control,
    Payload,
}

/// Heap 与有限来源共同实现标准 Arc/Vec 的真实 allocator 合同。
#[derive(Clone)]
pub(crate) enum BioAllocator {
    Heap,
    Reserved {
        reserve: Arc<BioReserve>,
        slot: usize,
        block: BioBlock,
    },
}

impl BioAllocator {
    /// 从映射来源派生同一完整单元的控制来源，不增加另一个池。
    pub(super) fn control(&self) -> Self {
        match self {
            Self::Heap => Self::Heap,
            Self::Reserved { reserve, slot, .. } => Self::Reserved {
                reserve: reserve.clone(),
                slot: *slot,
                block: BioBlock::Control,
            },
        }
    }

    /// 返回具体来源的有限映射上限；普通 Heap 仍按需构造。
    pub(super) fn mapping_limit(&self) -> usize {
        match self {
            Self::Heap => usize::MAX,
            Self::Reserved { slot, .. } => slot_mappings(*slot),
        }
    }
}

// SAFETY: Heap 完整转发同一 Global；Reserved 仅交对应单元的原布局块，
// 每个块只有一份真实分配拥有，最后 deallocate 在源仍活时归还状态。
unsafe impl Allocator for BioAllocator {
    /// 分配只消费已经授予的单元；ready 后绝不向堆补池。
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => Global.allocate(layout),
            Self::Reserved {
                reserve,
                slot,
                block,
            } => reserve.allocate_bio(*slot, *block, layout),
        }
    }

    /// 仅消费此前从本来源得到的同一首址与布局。
    unsafe fn deallocate(&self, pointer: NonNull<u8>, layout: Layout) {
        match self {
            Self::Heap => {
                // SAFETY: 调用者交出同一 Global 分配的真实首址和原布局。
                unsafe { Global.deallocate(pointer, layout) };
            }
            Self::Reserved {
                reserve,
                slot,
                block,
            } => {
                reserve.release_bio(*slot, *block, pointer, layout);
            }
        }
    }

    /// 普通 Vec 保留 Global realloc；固定单元不允许扩容。
    unsafe fn grow(
        &self,
        pointer: NonNull<u8>,
        old_layout: Layout,
        new_layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => {
                // SAFETY: 原拥有及新布局沿标准 Allocator::grow 合同移交 Global。
                unsafe { Global.grow(pointer, old_layout, new_layout) }
            }
            Self::Reserved { .. } => Err(AllocError),
        }
    }

    /// 普通清零增长仍走 Global，预留映射容量保持固定。
    unsafe fn grow_zeroed(
        &self,
        pointer: NonNull<u8>,
        old_layout: Layout,
        new_layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => {
                // SAFETY: 参数和移交的唯一拥有满足标准 grow_zeroed 合同。
                unsafe { Global.grow_zeroed(pointer, old_layout, new_layout) }
            }
            Self::Reserved { .. } => Err(AllocError),
        }
    }

    /// 普通缩容转发原 Global；预留单元不改变容量或原布局。
    unsafe fn shrink(
        &self,
        pointer: NonNull<u8>,
        old_layout: Layout,
        new_layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        match self {
            Self::Heap => {
                // SAFETY: 参数和移交的唯一拥有满足标准 shrink 合同。
                unsafe { Global.shrink(pointer, old_layout, new_layout) }
            }
            Self::Reserved { .. } => Err(AllocError),
        }
    }
}

impl BioReserve {
    /// 启用前准备九份 Bio 存储及容量内 metadata 基准；Node 布局由实际 Arc 请求提供。
    pub(crate) fn try_new(
        capacity: usize,
        hot: impl IntoIterator<Item = FolioRef>,
    ) -> Result<Arc<Self>, BlockError> {
        sched::assert_switchable();
        if capacity == 0 {
            return Err(BlockError::InvalidRange);
        }
        let mut metadata = Vec::new();
        metadata
            .try_reserve_exact(capacity)
            .map_err(|_| BlockError::NoMemory)?;
        let mut hot = hot.into_iter();
        for _ in 0..capacity {
            metadata.push(MetadataSlot {
                folio: match hot.next() {
                    Some(folio) => folio,
                    None => FolioRef::allocate_metadata().ok_or(BlockError::NoMemory)?,
                },
                control: Allocation::new(),
                leased: false,
            });
        }
        if hot.next().is_some() {
            return Err(BlockError::InvalidRange);
        }
        let reserve = Arc::try_new(Self {
            state: SpinLock::new(ReserveState {
                bios: core::array::from_fn(|_| BioSlot::new()),
                metadata,
                ready: false,
                stop_sequence: 0,
            }),
            available: WaitQueue::new(),
        })
        .map_err(|_| BlockError::NoMemory)?;
        for slot in 0..BIO_SLOTS {
            let allocator = BioAllocator::Reserved {
                reserve: reserve.clone(),
                slot,
                block: BioBlock::Control,
            };
            let control = Arc::<BioInner, BioAllocator>::try_new_uninit_in(allocator)
                .map_err(|_| BlockError::NoMemory)?;
            drop(control);
            let layout = Layout::array::<MappedPages>(slot_mappings(slot))
                .map_err(|_| BlockError::NoMemory)?;
            let payload = RawBlock::allocate(layout).map_err(|_| BlockError::NoMemory)?;
            reserve.state.lock().bios[slot].payload.block = Some(payload);
        }
        Ok(reserve)
    }

    /// 返回原 cache 已选定的真实 metadata 页容量。
    #[cfg(feature = "ktest")]
    pub(crate) fn capacity(&self) -> usize {
        self.state.lock().metadata.len()
    }

    /// 克隆原基准页；克隆只增加真实引用，不重新申请物理页。
    pub(crate) fn metadata_folio(&self, slot: usize) -> FolioRef {
        self.state.lock().metadata[slot].folio.clone()
    }

    /// 按真实原页身份查本 cache 的库存，不恢复任何 raw Arc 来源。
    pub(crate) fn metadata_slot_for(&self, folio: &FolioRef) -> Option<usize> {
        let physical = folio.phys_addr();
        self.state
            .lock()
            .metadata
            .iter()
            .position(|slot| slot.folio.phys_addr() == physical)
    }

    /// 明确该页是否有本源基准；这不是单独的回收资格。
    pub(crate) fn metadata_has_baseline(&self, folio: &FolioRef) -> bool {
        self.metadata_slot_for(folio).is_some()
    }

    /// 所有 genuine 控制准备和 bootstrap 票尾结束后一次开放来源。
    pub(crate) fn mark_ready(&self) {
        let mut state = self.state.lock();
        assert!(!state.ready, "reserve published twice");
        assert!(state
            .bios
            .iter()
            .all(|slot| slot.reusable() && !slot.booked && slot.control.retired_once));
        assert!(state
            .metadata
            .iter()
            .all(|slot| slot.control.block.is_some() && slot.control.retired_once && !slot.leased));
        state.ready = true;
    }

    /// Node allocator 的实际 Layout 首次请求仅能发生在未发表准备期。
    pub(crate) fn allocate_metadata_control(
        &self,
        slot: usize,
        layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        let building = {
            let mut state = self.state.lock();
            let building = !state.ready;
            let metadata = &mut state.metadata[slot];
            if !building && !metadata.leased {
                return Err(AllocError);
            }
            if let Some(result) = use_prepared(&mut metadata.control, layout)? {
                return Ok(result);
            }
            if !building {
                return Err(AllocError);
            }
            metadata.control.active = true;
            building
        };
        assert!(building);
        let allocation = RawBlock::allocate(layout);
        let mut state = self.state.lock();
        match allocation {
            Ok(block) => {
                let view = block.view();
                state.metadata[slot].control.block = Some(block);
                Ok(view)
            }
            Err(error) => {
                state.metadata[slot].control.active = false;
                Err(error)
            }
        }
    }

    /// 最后 Node Weak 才归还控制块，原页 reset 由 cache 的独占协议另行完成。
    ///
    /// # Safety
    /// pointer/layout 必须是本 slot 此前真实 allocate 返回且尚未归还的同一分配。
    pub(crate) unsafe fn deallocate_metadata_control(
        &self,
        slot: usize,
        pointer: NonNull<u8>,
        layout: Layout,
    ) {
        {
            let mut state = self.state.lock();
            release_allocation(&mut state.metadata[slot].control, pointer, layout);
        }
        self.notify_available();
    }

    /// 无空单元时保真实 NoMemory，不等待自己仍持的 Node/Buffer。
    pub(crate) fn try_metadata_slot(self: &Arc<Self>) -> Result<MetadataLease, BlockError> {
        let mut state = self.state.lock();
        if !state.ready {
            return Err(BlockError::NotReady);
        }
        let Some(slot) = state
            .metadata
            .iter_mut()
            .enumerate()
            .find_map(|(index, slot)| {
                if !slot.leased
                    && slot.control.available()
                    && slot.folio.metadata_inventory_available()
                {
                    slot.leased = true;
                    Some(index)
                } else {
                    None
                }
            })
        else {
            return Err(BlockError::NoMemory);
        };
        Ok(MetadataLease {
            reserve: self.clone(),
            slot,
        })
    }

    /// 普通新业务只在自身入口捕获一次真实源边界，受限轮从原 storage 延续它。
    pub(crate) fn metadata_io_context(self: &Arc<Self>) -> MetadataIoContext<'_> {
        MetadataIoContext::from_boundary(self, self.state.lock().stop_sequence)
    }

    /// 锁外唤醒控制归还或原页回到基准的等待者；通知不代替重查。
    pub(crate) fn notify_available(&self) {
        self.available.wake_all();
    }

    /// 在 FS 业务锁外预定至少一份完整 WB 单元，不为凑满而保未提交对象。
    pub(crate) fn acquire_wb_lane(
        self: &Arc<Self>,
        stopped: impl Fn() -> bool,
    ) -> Result<BioLane, BlockError> {
        self.wait_available(|| {
            if stopped() {
                return Err(BlockError::QueueStopped);
            }
            let mut state = self.state.lock();
            if !state.ready {
                return Err(BlockError::NotReady);
            }
            let mut slots = [0; MAX_BATCH];
            let mut count = 0;
            for (index, slot) in state.bios[..MAX_BATCH].iter_mut().enumerate() {
                if !slot.booked && slot.reusable() {
                    slot.booked = true;
                    slots[count] = index;
                    count += 1;
                }
            }
            Ok((count != 0).then(|| BioLane {
                reserve: self.clone(),
                slots,
                count,
            }))
        })
    }

    /// 独立读取/FLUSH 单元先取得，再允许认领 metadata 内容锁。
    pub(crate) fn acquire_aux(
        self: &Arc<Self>,
        context: MetadataIoContext<'_>,
    ) -> Result<BioLease, BlockError> {
        assert!(context.belongs_to(self), "foreign metadata IO context");
        self.wait_available(|| {
            let mut state = self.state.lock();
            if state.stop_sequence != context.boundary
                || context.view.is_some_and(|(sequence, boundary)| {
                    sequence.load(Ordering::Acquire) != boundary
                })
            {
                return Err(BlockError::NotReady);
            }
            if !state.ready {
                return Err(BlockError::NotReady);
            }
            let slot = &mut state.bios[AUX_SLOT];
            if slot.booked || !slot.reusable() {
                return Ok(None);
            }
            slot.leased = true;
            Ok(Some(BioLease {
                reserve: self.clone(),
                slot: AUX_SLOT,
            }))
        })
    }

    /// 原等待源和栈登记覆盖重查窗口，不申请等待对象或使用 timeout 轮询。
    fn wait_available<T>(
        &self,
        mut check: impl FnMut() -> Result<Option<T>, BlockError>,
    ) -> Result<T, BlockError> {
        sched::assert_switchable();
        loop {
            if let Some(value) = check()? {
                return Ok(value);
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 调用者保持源拥有；pin 固定栈节点，所有返回都先撤登记。
            let _registration = unsafe { self.available.register(node.as_mut()) };
            if let Some(value) = check()? {
                return Ok(value);
            }
            waiter.park();
        }
    }

    /// 只消费已准备原块；初始控制分配在全部源锁外进行。
    fn allocate_bio(
        &self,
        slot: usize,
        block: BioBlock,
        layout: Layout,
    ) -> Result<NonNull<[u8]>, AllocError> {
        {
            let mut state = self.state.lock();
            let building = !state.ready;
            let bio = &mut state.bios[slot];
            if !building && !bio.leased {
                return Err(AllocError);
            }
            let allocation = bio_allocation(bio, block);
            if let Some(result) = use_prepared(allocation, layout)? {
                return Ok(result);
            }
            if !building {
                return Err(AllocError);
            }
            allocation.active = true;
        }
        let allocation = RawBlock::allocate(layout);
        let mut state = self.state.lock();
        let record = bio_allocation(&mut state.bios[slot], block);
        match allocation {
            Ok(block) => {
                let view = block.view();
                record.block = Some(block);
                Ok(view)
            }
            Err(error) => {
                record.active = false;
                Err(error)
            }
        }
    }

    /// 每个真实 Arc/Vec deallocate 只归自身责任，book 与 lease 另行结束。
    fn release_bio(&self, slot: usize, block: BioBlock, pointer: NonNull<u8>, layout: Layout) {
        {
            let mut state = self.state.lock();
            release_allocation(
                bio_allocation(&mut state.bios[slot], block),
                pointer,
                layout,
            );
        }
        self.notify_available();
    }

    /// 读取真实存储布局与占用，不将 observer 尚活的控制算作空单元。
    #[cfg(feature = "ktest")]
    pub(crate) fn snapshot(&self) -> ReserveSnapshot {
        let state = self.state.lock();
        ReserveSnapshot {
            ready: state.ready,
            metadata_capacity: state.metadata.len(),
            free_wb: state.bios[..MAX_BATCH]
                .iter()
                .filter(|slot| !slot.booked && slot.reusable())
                .count(),
            aux_free: state.bios[AUX_SLOT].reusable(),
            booked: state.bios.iter().filter(|slot| slot.booked).count(),
            leased: state.bios.iter().filter(|slot| slot.leased).count(),
            control_live: state.bios.iter().filter(|slot| slot.control.active).count(),
            payload_live: state.bios.iter().filter(|slot| slot.payload.active).count(),
            bio_control: state.bios[0]
                .control
                .block
                .as_ref()
                .map(|block| block.layout),
            wb_payload: state.bios[0]
                .payload
                .block
                .as_ref()
                .map(|block| block.layout),
            aux_payload: state.bios[AUX_SLOT]
                .payload
                .block
                .as_ref()
                .map(|block| block.layout),
            node_control: state.metadata[0]
                .control
                .block
                .as_ref()
                .map(|block| block.layout),
            metadata_control_live: state
                .metadata
                .iter()
                .filter(|slot| slot.control.active)
                .count(),
        }
    }

    /// 只观察原来源等待队列的实际成员，不创建另一份等待账。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_waiting(&self) -> usize {
        self.available.test_members()
    }
}

impl Drop for BioReserve {
    /// 最后源拥有只核尾部并析构原块与基准页，不发起 FS/IO 或同步停止。
    fn drop(&mut self) {
        let state = self.state.lock();
        assert!(state.bios.iter().all(|slot| !slot.booked
            && !slot.leased
            && !slot.control.active
            && !slot.payload.active));
        assert!(state
            .metadata
            .iter()
            .all(|slot| !slot.leased && !slot.control.active));
    }
}

/// 业务锁前授予的整体准入，不保存任何活 Bio/Vec 或页内容借用。
pub(crate) struct BioLane {
    reserve: Arc<BioReserve>,
    slots: [usize; MAX_BATCH],
    count: usize,
}

impl BioLane {
    /// 返回本次真正取得的完整单元数，未取得的份额不伪装成八份。
    pub(crate) fn len(&self) -> usize {
        self.count
    }

    /// 非阻塞探测本 book 的一个单元；已有未提交父时绝不能为了凑满而等待。
    pub(crate) fn try_lease(
        &self,
        index: usize,
        stopped: impl Fn() -> bool,
    ) -> Result<Option<BioLease>, BlockError> {
        assert!(index < self.count);
        if stopped() {
            return Err(BlockError::QueueStopped);
        }
        let slot = self.slots[index];
        let mut state = self.reserve.state.lock();
        let bio = &mut state.bios[slot];
        assert!(bio.booked);
        if !bio.reusable() {
            return Ok(None);
        }
        bio.leased = true;
        Ok(Some(BioLease {
            reserve: self.reserve.clone(),
            slot,
        }))
    }

    /// 仅在本波零父时等待任一已 book 单元，真实 weak 尾不阻塞其他已可用份额前进。
    pub(crate) fn lease_available(
        &self,
        stopped: impl Fn() -> bool,
    ) -> Result<(usize, BioLease), BlockError> {
        self.reserve.wait_available(|| {
            if stopped() {
                return Err(BlockError::QueueStopped);
            }
            let mut state = self.reserve.state.lock();
            for (index, slot) in self.slots[..self.count].iter().copied().enumerate() {
                let bio = &mut state.bios[slot];
                assert!(bio.booked);
                if bio.reusable() {
                    bio.leased = true;
                    return Ok(Some((
                        index,
                        BioLease {
                            reserve: self.reserve.clone(),
                            slot,
                        },
                    )));
                }
            }
            Ok(None)
        })
    }
}

impl Drop for BioLane {
    /// 开放准入不清真实分配责任；未 deallocate 的单元仍不可借。
    fn drop(&mut self) {
        {
            let mut state = self.reserve.state.lock();
            for slot in &self.slots[..self.count] {
                assert!(state.bios[*slot].booked);
                state.bios[*slot].booked = false;
            }
        }
        self.reserve.notify_available();
    }
}

/// 每次构造的短拥有，只允许从一个完整单元取得原映射存储。
pub(crate) struct BioLease {
    reserve: Arc<BioReserve>,
    slot: usize,
}

impl BioLease {
    /// 以实际 allocator 建立已有块的 Vec 拥有，不复制 payload。
    pub(crate) fn memory(&self) -> Result<BioMemory, BlockError> {
        let mut memory = Vec::new_in(BioAllocator::Reserved {
            reserve: self.reserve.clone(),
            slot: self.slot,
            block: BioBlock::Payload,
        });
        memory
            .try_reserve_exact(slot_mappings(self.slot))
            .map_err(|_| BlockError::NoMemory)?;
        Ok(memory)
    }
}

impl Drop for BioLease {
    /// 仅结束构造 lease；所有实际 Arc/Vec owning 仍阻止物理重用。
    fn drop(&mut self) {
        {
            let mut state = self.reserve.state.lock();
            assert!(state.bios[self.slot].leased);
            state.bios[self.slot].leased = false;
        }
        self.reserve.notify_available();
    }
}

/// Node 构造前的单元借用；不在 guard 中保存 Node 或有效内容别名。
pub(crate) struct MetadataLease {
    reserve: Arc<BioReserve>,
    slot: usize,
}

impl MetadataLease {
    /// 返回实际单元号供同一 IndexAllocator 保留来源。
    pub(crate) fn slot(&self) -> usize {
        self.slot
    }

    /// 借出原基准的真实克隆；caller 在 Node 发表前完成独占 reset。
    pub(crate) fn folio(&self) -> FolioRef {
        self.reserve.metadata_folio(self.slot)
    }
}

impl Drop for MetadataLease {
    /// 构造成功或失败都结束借出，控制及原页占用仍分别真实重查。
    fn drop(&mut self) {
        {
            let mut state = self.reserve.state.lock();
            assert!(state.metadata[self.slot].leased);
            state.metadata[self.slot].leased = false;
        }
        self.reserve.notify_available();
    }
}

/// 验收读取的真实 Layout/责任；统计本身不提供 IO 停止或重用许可。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy, Debug)]
pub(crate) struct ReserveSnapshot {
    pub ready: bool,
    pub metadata_capacity: usize,
    pub free_wb: usize,
    pub aux_free: bool,
    pub booked: usize,
    pub leased: usize,
    pub control_live: usize,
    pub payload_live: usize,
    pub bio_control: Option<Layout>,
    pub wb_payload: Option<Layout>,
    pub aux_payload: Option<Layout>,
    pub node_control: Option<Layout>,
    pub metadata_control_live: usize,
}

/// 单元几何只有 WB 八映射与 aux 一映射两种具体取值。
fn slot_mappings(slot: usize) -> usize {
    if slot == AUX_SLOT {
        1
    } else {
        WB_MAPPINGS
    }
}

/// 选择完整单元内的一份真实分配事实，不复制块拥有。
fn bio_allocation(slot: &mut BioSlot, block: BioBlock) -> &mut Allocation {
    match block {
        BioBlock::Control => &mut slot.control,
        BioBlock::Payload => &mut slot.payload,
    }
}

/// 已准备块只允许原布局及一份实际分配拥有。
fn use_prepared(
    allocation: &mut Allocation,
    layout: Layout,
) -> Result<Option<NonNull<[u8]>>, AllocError> {
    if allocation.active {
        return Err(AllocError);
    }
    let Some(block) = &allocation.block else {
        return Ok(None);
    };
    if block.layout != layout {
        return Err(AllocError);
    }
    allocation.active = true;
    Ok(Some(block.view()))
}

/// allocator 调用已经证明最后拥有结束，源另核原首址/布局恰好归还一次。
fn release_allocation(allocation: &mut Allocation, pointer: NonNull<u8>, layout: Layout) {
    assert!(allocation.active, "reserve allocation ended twice");
    let block = allocation.block.as_ref().expect("reserve block missing");
    assert_eq!(block.pointer, pointer, "reserve allocation source mismatch");
    assert_eq!(block.layout, layout, "reserve allocation layout mismatch");
    allocation.active = false;
    allocation.retired_once = true;
}
