//! 从实际 RAM 直映建立永久页描述符；空洞仅占 section 索引。

use core::cell::UnsafeCell;
use core::marker::PhantomPinned;
use core::mem::size_of;
use core::slice;
use core::sync::atomic::AtomicUsize;

use crate::boot::MAX_MEMORY_REGIONS;
use crate::mm::anon::AnonState;
use crate::mm::bootmem::{BootMemory, BootStats, PageRange, PAGE_SHIFT, RANGE_CAPACITY};
use crate::mm::folio::FolioState;
use crate::mm::{OwnedPages, PAGE_SIZE};
use crate::sync::{SpinLock, SpinLockGuard};

const SECTION_SIZE_BITS: usize = 27;
const PAGES_PER_SECTION: usize = 1 << (SECTION_SIZE_BITS - PAGE_SHIFT);
type Section<'a> = Option<&'a [PageDesc]>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PageKind {
    Absent,
    Reserved,
    Metadata,
    Available,
}

pub(crate) struct PageDesc {
    kind: PageKind,
    /// 仅由唯一所属 zone 的锁域访问，不以启动分类代替自由链成员身份。
    pub(crate) buddy: UnsafeCell<BuddyLink>,
    /// 块内每个 PFN 关联实际头；SLUB 独占建块后以 Release 发布，定位方以 Acquire 读取。
    /// 无关联时为 usize::MAX；销毁时须先结束所有对象与成员责任，再清除整块关联。
    pub(crate) slab_head: AtomicUsize,
    /// 只锁定用途访问令牌，取锁不会借用槽内的 folio 或等待源。
    usage_lock: SpinLock<()>,
    /// Empty 只表示没有 slab、folio 或匿名页用途，不表示属于自由链。
    usage: UnsafeCell<PageUse>,
    _pinned: PhantomPinned,
}

/// 同一页头只能拥有一种用途，原始连续块归还权保留在实际用途内。
enum PageUse {
    Empty,
    Slab(SlabState),
    Folio(FolioState),
    Anon(AnonState),
}

/// 页用途的短锁令牌；只授予已确认用途的视图，不暴露整个槽的可变借用。
pub(crate) struct PageUseGuard<'a> {
    descriptor: &'a PageDesc,
    _lock: SpinLockGuard<'a, ()>,
}

impl PageUseGuard<'_> {
    /// 持令牌读取用途标签；活 folio 的共享投影不会因再次取锁而失效。
    fn value(&self) -> &PageUse {
        // SAFETY: guard 持有本描述符唯一用途锁，安装和移出也持同锁；
        // 活 folio 只修改内部 UnsafeCell，不创建整个用途槽的可变借用。
        unsafe { &*self.descriptor.usage.get() }
    }

    /// 借用真实 SLUB 视图；其他用途不暴露 slab 字段。
    pub(crate) fn slab(&self) -> Option<&SlabState> {
        match self.value() {
            PageUse::Slab(state) => Some(state),
            PageUse::Empty | PageUse::Folio(_) | PageUse::Anon(_) => None,
        }
    }

    /// 确认 Slab 后才授予独占视图，保持原 CPU/node 成员锁顺序。
    pub(crate) fn slab_mut(&mut self) -> Option<&mut SlabState> {
        self.slab()?;
        // SAFETY: 用途锁排除安装和移出，标签已确认为 Slab，没有活 folio 投影；
        // 可变 guard 借用结束全部先前视图，返回借用不能超出当前 guard。
        match unsafe { &mut *self.descriptor.usage.get() } {
            PageUse::Slab(state) => Some(state),
            PageUse::Empty | PageUse::Folio(_) | PageUse::Anon(_) => unreachable!(),
        }
    }

    /// 借用固定 folio 的共享状态，内部共享头仍须由本令牌保护。
    pub(crate) fn folio(&self) -> &FolioState {
        match self.value() {
            PageUse::Folio(state) => state,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Anon(_) => {
                panic!("live folio lost its page use")
            }
        }
    }

    /// 借用用途锁下匿名页状态，返回视图不能超过当前 guard。
    pub(crate) fn anon(&self) -> &AnonState {
        match self.value() {
            PageUse::Anon(state) => state,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Folio(_) => {
                panic!("live anonymous page lost its page use")
            }
        }
    }

    /// 确认匿名用途后修改真实引用头，不借用任何活 folio 或固定等待源。
    pub(crate) fn anon_mut(&mut self) -> &mut AnonState {
        self.anon();
        // SAFETY: 用途锁排除安装和移出，标签已确认为 Anon，槽中无活 folio；
        // 匿名状态不向锁外投影，可变 guard 借用结束全部短视图，返回借用绑定 guard。
        match unsafe { &mut *self.descriptor.usage.get() } {
            PageUse::Anon(state) => state,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Folio(_) => unreachable!(),
        }
    }

    /// 仅在空槽安装用途，已有用途必须先结束全部访问责任并移出。
    fn install(&mut self, value: PageUse) {
        assert!(
            matches!(self.value(), PageUse::Empty),
            "owned page already has a use"
        );
        // SAFETY: 当前槽为空，锁排除竞争安装；可变 guard 借用结束已有短视图，
        // 空槽没有 folio 投影或等待源责任，写入不覆盖活拥有者。
        unsafe { self.descriptor.usage.get().write(value) };
    }

    /// 在未发布对象的空槽安装唯一 slab 拥有者。
    pub(crate) fn install_slab(&mut self, state: SlabState) {
        self.install(PageUse::Slab(state));
    }

    /// 在首引用发布前将 folio 拥有者固定到空槽。
    pub(crate) fn install_folio(&mut self, state: FolioState) {
        self.install(PageUse::Folio(state));
    }

    /// 在首引用发布前将匿名基本页的原归还权安装到空槽。
    pub(crate) fn install_anon(&mut self, state: AnonState) {
        self.install(PageUse::Anon(state));
    }

    /// 移出已摘离且没有活对象的 slab，原页须在全部 MM 锁外归还。
    pub(crate) fn take_slab(&mut self) -> Option<SlabState> {
        let state = self.slab()?;
        assert_eq!(state.inuse, 0, "removed slab retains live objects");
        assert_eq!(
            state.location,
            SlabLocation::Detached,
            "removed slab retains a list owner"
        );
        // SAFETY: 已确认 Slab，没有 folio 共享投影；活对象与成员责任均已结束，
        // 可变 guard 借用结束全部短视图，锁排除其他 slab 访问和用途变化。
        match unsafe { self.descriptor.usage.get().replace(PageUse::Empty) } {
            PageUse::Slab(state) => Some(state),
            PageUse::Empty | PageUse::Folio(_) | PageUse::Anon(_) => unreachable!(),
        }
    }

    /// 引用数归零后移出匿名页的原拥有者，调用方须在全部 MM 锁外归还。
    pub(crate) fn take_anon(&mut self) -> OwnedPages {
        assert_eq!(
            self.anon().references,
            0,
            "removed anonymous page retains references"
        );
        // SAFETY: 标签已确认为 Anon 且真实引用归零，没有锁外状态投影；
        // 可变 guard 借用结束短视图，用途锁排除竞争访问，不移动任何固定 folio。
        match unsafe { self.descriptor.usage.get().replace(PageUse::Empty) } {
            PageUse::Anon(state) => state.backing,
            PageUse::Empty | PageUse::Slab(_) | PageUse::Folio(_) => unreachable!(),
        }
    }

    /// 静止末引用取走原页并原地析构固定 folio；原页须在用途锁外归还。
    ///
    /// # Safety
    /// 引用数必须为零，内容锁、初始化/DMA、Dirty、WB、等待登记、内容借用和固定投影
    /// 必须已经结束。不得用析构丢弃尚有写入或持久化责任的原页。
    pub(crate) unsafe fn take_folio(&mut self) -> OwnedPages {
        // SAFETY: caller 已结束全部 folio 访问和写入责任，用途锁排除用途变化；
        // 可变 guard 借用结束锁内视图，take_backing 再核无锁、无 Dirty/WB，
        // 只取走未固定的 backing，不移动等待源。
        let backing = match unsafe { &mut *self.descriptor.usage.get() } {
            PageUse::Folio(state) => state.take_backing(),
            PageUse::Empty | PageUse::Slab(_) | PageUse::Anon(_) => {
                panic!("live folio lost its page use")
            }
        };
        // SAFETY: backing 已移交返回值，所有其他责任已结束；原地析构满足
        // 等待源的 Pin 承诺，随后在已析构槽写入 Empty，不析构或移动原页。
        unsafe {
            self.descriptor.usage.get().drop_in_place();
            self.descriptor.usage.get().write(PageUse::Empty);
        }
        backing
    }
}

#[derive(Clone, Copy)]
pub(crate) struct BuddyLink {
    pub(crate) order: usize,
    pub(crate) previous: usize,
    pub(crate) next: usize,
}

/// 唯一成员责任；Active 表达 frozen，只有 NodePartial 属于 node 链。
/// flush 摘离 active 后的 CPU ticket 延续其责任，普通执行流 deactivate 才改变角色。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SlabLocation {
    Detached,
    Active(usize),
    CpuPartial(usize),
    NodePartial,
    Full,
}

/// 实际 slab 头的共享状态；成员链同时受对应 CPU/node 锁与本头锁保护。
/// backing 保存唯一整体归还权与实际 order。销毁须先从头锁 take 本状态，
/// 退出全部 MM 锁并清除块内 head 关联后，才能析构 backing。
pub(crate) struct SlabState {
    pub(crate) backing: OwnedPages,
    pub(crate) cache: usize,
    pub(crate) objects: usize,
    /// Active 时包含交给 CPU local freelist 的对象，不能用作用户 live 数。
    pub(crate) inuse: usize,
    /// 已释放对象首 word 构成的 freelist；仍为 live 的对象不属于此链。
    pub(crate) shared: usize,
    pub(crate) location: SlabLocation,
    pub(crate) next: usize,
    pub(crate) previous: usize,
}

// SAFETY: kind 发布后不可变，buddy 的全部读写由唯一所属 zone 锁互斥；
// slab_head 仅原子访问，SLUB 与匿名页状态由独立 usage_lock 互斥；folio 固定
// 用途仅修改同锁保护的内部共享头，取锁只借用独立令牌，内容锁与等待源维持固定地址。
// 生产描述符永久发布后不移动；最后引用才结束用途，原页归还在用途锁外。
unsafe impl Sync for PageDesc {}

impl PageDesc {
    /// 建立无用途、无内容锁责任的描述符；生产发布后永久固定其地址。
    pub(crate) const fn new(kind: PageKind) -> Self {
        Self {
            kind,
            buddy: UnsafeCell::new(BuddyLink {
                order: usize::MAX,
                previous: usize::MAX,
                next: usize::MAX,
            }),
            slab_head: AtomicUsize::new(usize::MAX),
            usage_lock: SpinLock::new(()),
            usage: UnsafeCell::new(PageUse::Empty),
            _pinned: PhantomPinned,
        }
    }

    /// 取得用途短锁令牌，不创建覆盖槽内 folio 的独占借用。
    pub(crate) fn usage(&self) -> PageUseGuard<'_> {
        PageUseGuard {
            descriptor: self,
            _lock: self.usage_lock.lock(),
        }
    }

    /// 取得页的不可变启动来源类别；Available 不表示当前在自由链中。
    pub(crate) fn kind(&self) -> PageKind {
        self.kind
    }
}

pub(crate) struct PageMap<'a> {
    sections: &'a [Section<'a>],
    free: &'a [PageRange],
    stats: BootStats,
    present_sections: usize,
    #[cfg(feature = "ktest")]
    metadata: &'a [PageRange],
}

#[derive(Clone, Copy)]
struct SectionGroup {
    start: usize,
    end: usize,
    backing: PageRange,
}

impl<'a> PageMap<'a> {
    /// 按 section 和槽内偏移定位 PFN，不读取启动区间表。
    pub(crate) fn get(&self, pfn: usize) -> Option<&PageDesc> {
        self.sections
            .get(pfn / PAGES_PER_SECTION)?
            .as_ref()?
            .get(pfn % PAGES_PER_SECTION)
    }

    /// 返回元数据全部扣除后的 RAM 分类计数。
    pub(crate) fn stats(&self) -> BootStats {
        self.stats
    }

    /// 返回按实际 RAM 最大端点建立的根索引长度。
    pub(crate) fn section_count(&self) -> usize {
        self.sections.len()
    }

    /// 返回实际拥有描述符 backing 的 section 数量。
    pub(crate) fn present_sections(&self) -> usize {
        self.present_sections
    }

    /// 返回下一次页分配器交接所需的最终自由范围，不授予释放权限。
    pub(crate) fn free_ranges(&self) -> &[PageRange] {
        self.free
    }

    /// 借用永久发布的 section 与 metadata 范围，供独立用例核对物理页分类。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_layout_parts(&self) -> (&[Section<'a>], &[PageRange]) {
        (self.sections, self.metadata)
    }

    /// 借用独立测试描述符与范围；不发布，也不授予物理页访问权限。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_map(sections: &'a [Section<'a>], free: &'a [PageRange]) -> Self {
        Self {
            sections,
            free,
            stats: BootStats {
                present_pages: 0,
                reserved_pages: 0,
                metadata_pages: 0,
                free_pages: free.iter().map(|range| range.end - range.start).sum(),
            },
            present_sections: sections.iter().filter(|section| section.is_some()).count(),
            metadata: &[],
        }
    }
}

/// 将字节数向上转换成 backing 页数，拒绝指针跨度不可表达的布局。
fn backing_pages(bytes: usize) -> Result<usize, &'static str> {
    if bytes == 0 || (isize::MAX as usize) < bytes {
        return Err("page metadata layout is invalid");
    }
    Ok(bytes.div_ceil(PAGE_SIZE))
}

/// 顺序推进规范化区间游标；初始化按递增 PFN 访问每个列表一次。
fn cursor_contains(ranges: &[PageRange], cursor: &mut usize, pfn: usize) -> bool {
    while *cursor < ranges.len() && ranges[*cursor].end <= pfn {
        *cursor += 1;
    }
    *cursor < ranges.len() && ranges[*cursor].start <= pfn
}

/// 在独占 backing 中逐项写入范围后，创建永久只读快照。
///
/// # Safety
/// address 必须非空且按 PageRange 对齐，即使 ranges 为空也必须如此；目标必须
/// 位于单个分配内，拥有至少 ranges.len() 个槽的独占、永久可写映射 RAM，
/// 字节跨度不超过 isize::MAX，地址加跨度不得回绕。目标不得与 ranges 或任何
/// 已建立引用的对象重叠；写入后不得复用或修改这些槽。
unsafe fn write_ranges(address: usize, ranges: &[PageRange]) -> &'static [PageRange] {
    let destination = address as *mut PageRange;
    for (index, range) in ranges.iter().enumerate() {
        // SAFETY: 调用者提供独占且足够大的对齐 backing；index 位于其槽范围内。
        unsafe { destination.add(index).write(*range) };
    }
    // SAFETY: 所有目标槽已写入有效 PageRange；调用者保证映射永久且不再修改。
    unsafe { slice::from_raw_parts(destination, ranges.len()) }
}

/// 先扣除全部元数据页，再按递增 PFN 填写 present section 的描述符和根索引。
///
/// # Safety
/// 只能由 BSP 在 IRQ 关闭、AP 尚未释放时调用；memory 必须来自当前机器真实
/// BootInfo，包含内核、栈、页表、DTB 和固件保留资源。其可用 RAM 的直映必须
/// 永久可写且一一对应；每次分配的连续页必须在直映中构成单个连续分配，
/// 起址非空且按 PAGE_SIZE 对齐，整个分配的虚拟地址范围不得回绕。
/// 本函数取得的页必须独占，不得与 memory 自身或其他存活引用重叠，之后
/// backing 不得释放或重新分配，kind/索引/区间快照不得修改；buddy 字段仅允许
/// 由唯一所属 zone 锁域改变，页用途仅允许由其头锁访问，head 关联采用原子协议；
/// 永久发布后不得移动描述符；folio 等待源在活用途期间按引用协议固定。
/// 失败时已扣除的 backing 仍保留，调用者必须终止启动。
pub(crate) unsafe fn initialize(memory: &mut BootMemory) -> Result<PageMap<'static>, &'static str> {
    let mut groups = [SectionGroup {
        start: 0,
        end: 0,
        backing: PageRange { start: 0, end: 0 },
    }; MAX_MEMORY_REGIONS];
    let mut group_count = 0;
    for range in memory.ram() {
        let start = range.start / PAGES_PER_SECTION;
        let end = range.end.div_ceil(PAGES_PER_SECTION);
        if group_count != 0 && start <= groups[group_count - 1].end {
            groups[group_count - 1].end = groups[group_count - 1].end.max(end);
        } else {
            groups[group_count].start = start;
            groups[group_count].end = end;
            group_count += 1;
        }
    }
    if group_count == 0 {
        return Err("page map has no RAM sections");
    }
    let section_count = groups[group_count - 1].end;
    let section_bytes = section_count
        .checked_mul(size_of::<Section<'static>>())
        .ok_or("page root size overflows")?;
    // Section 与 PageRange 均按 usize 对齐，索引后可直接放范围快照。
    let range_bytes = RANGE_CAPACITY
        .checked_mul(size_of::<PageRange>())
        .ok_or("page range storage overflows")?;
    let root_bytes = section_bytes
        .checked_add(range_bytes)
        .ok_or("page root size overflows")?;
    #[cfg(feature = "ktest")]
    let root_bytes = root_bytes
        .checked_add(range_bytes)
        .ok_or("page root size overflows")?;
    let root = memory.allocate(backing_pages(root_bytes)?, 1)?;
    let section_descriptor_bytes = PAGES_PER_SECTION
        .checked_mul(size_of::<PageDesc>())
        .ok_or("section descriptor size overflows")?;
    let mut present_sections = 0;
    for group in &mut groups[..group_count] {
        let count = group.end - group.start;
        let bytes = count
            .checked_mul(section_descriptor_bytes)
            .ok_or("page descriptor size overflows")?;
        group.backing = memory.allocate(backing_pages(bytes)?, 1)?;
        present_sections += count;
    }

    let root_address = crate::arch::phys_to_virt(root.start * PAGE_SIZE);
    let root_pointer = root_address as *mut Section<'static>;
    let mut group_index = 0;
    let mut ram_cursor = 0;
    let mut reserved_cursor = 0;
    let mut metadata_cursor = 0;
    for section_index in 0..section_count {
        while group_index < group_count && groups[group_index].end <= section_index {
            group_index += 1;
        }
        let descriptors = if group_index < group_count && groups[group_index].start <= section_index
        {
            let group = groups[group_index];
            let offset = (section_index - group.start) * section_descriptor_bytes;
            let address = crate::arch::phys_to_virt(group.backing.start * PAGE_SIZE) + offset;
            let destination = address as *mut PageDesc;
            for index in 0..PAGES_PER_SECTION {
                let pfn = section_index * PAGES_PER_SECTION + index;
                let ram = cursor_contains(memory.ram(), &mut ram_cursor, pfn);
                let reserved = cursor_contains(memory.reserved(), &mut reserved_cursor, pfn);
                let metadata = cursor_contains(memory.metadata(), &mut metadata_cursor, pfn);
                let kind = if !ram {
                    PageKind::Absent
                } else if reserved {
                    PageKind::Reserved
                } else if metadata {
                    PageKind::Metadata
                } else {
                    PageKind::Available
                };
                // SAFETY: group backing 独占且永久，offset 将本 section 定位到
                // 不重叠的 section_descriptor_bytes 子范围，PageDesc 对齐由页
                // 对齐起址与类型大小保证；add 仅在此 section 内，每槽首次写入。
                unsafe { destination.add(index).write(PageDesc::new(kind)) };
            }
            // SAFETY: 本 section 全部槽已写入有效值，跨度由 backing_pages 限制；
            // 该子范围属于一个永久分配，后续 section 写入不与本 slice 重叠。
            Some(unsafe { slice::from_raw_parts(destination, PAGES_PER_SECTION) })
        } else {
            None
        };
        // SAFETY: root 已独占预留且按 Section 对齐；每个索引槽仅写入一次合法值。
        unsafe { root_pointer.add(section_index).write(descriptors) };
    }
    // SAFETY: 根数组全部槽已初始化，跨度不超过 root_bytes，永久且不再修改；
    // 后续快照仅写 root 的未被 sections 引用的尾部。
    let sections = unsafe { slice::from_raw_parts(root_pointer, section_count) };
    // SAFETY: root 后部留有 RANGE_CAPACITY 个对齐槽，不与 sections 重叠；
    // BootMemory 的范围容量不超过该上界，写完后仅发布只读引用。
    let free = unsafe { write_ranges(root_address + section_bytes, memory.free()) };
    #[cfg(feature = "ktest")]
    // SAFETY: 测试布局额外预留另一组 RANGE_CAPACITY 槽，与根和 free 快照不重叠。
    let metadata = unsafe {
        write_ranges(
            root_address + section_bytes + range_bytes,
            memory.metadata(),
        )
    };
    Ok(PageMap {
        sections,
        free,
        stats: memory.stats(),
        present_sections,
        #[cfg(feature = "ktest")]
        metadata,
    })
}
