//! 单 Normal zone 的连续页块自由链，元数据仅在 zone 锁域内改变。

#[cfg(feature = "ktest")]
use core::mem::MaybeUninit;
use spin::Once;

#[cfg(feature = "ktest")]
use crate::mm::page::PageDesc;
use crate::mm::page::{BuddyLink, PageKind, PageMap};
use crate::sync::SpinLock;

pub(crate) const MAX_ORDER: usize = 10;
pub(crate) const BATCH_CAPACITY: usize = 8;
const NONE: usize = usize::MAX;

/// 唯一连续块释放权；移动不会复制归还责任，不自动归还。
pub(crate) struct Block {
    head: usize,
    order: usize,
}

impl Block {
    /// 返回连续块首 PFN，不授予额外释放权。
    pub(crate) fn head_pfn(&self) -> usize {
        self.head
    }

    /// 返回分配时的阶，归还必须保留此值。
    pub(crate) fn order(&self) -> usize {
        self.order
    }

    /// 恢复已从拥有者或 PCP 链转移的真实整体块唯一归还权，不改变 zone 元数据。
    ///
    /// # Safety
    /// head 必须是本 buddy 原分配且仍 live 的整体块首 PFN，order 必须是原阶；
    /// 原拥有凭证已消费或页刚从 PCP 链摘离，全部访问已结束，块从未归还，
    /// 不在任何 buddy、PCP 或 slab 自由链中，也无另一份有效归还凭证。
    pub(crate) unsafe fn from_raw(head: usize, order: usize) -> Self {
        Self { head, order }
    }
}

struct Zone<'a> {
    memory: &'a PageMap<'a>,
    heads: [usize; MAX_ORDER + 1],
    free: usize,
}

impl<'a> Zone<'a> {
    /// 建立尚未接管自由范围的唯一锁域，之后仅通过其锁 guard 调用方法。
    ///
    /// # Safety
    /// memory 的所有描述符不得同时归属其他 zone；其 buddy 字段仅由此锁域访问。
    unsafe fn empty(memory: &'a PageMap<'a>) -> Self {
        Self {
            memory,
            heads: [NONE; MAX_ORDER + 1],
            free: 0,
        }
    }

    /// 持有所属 zone 锁时读取自由头状态，每个 PFN 独立通过 PageMap 定位。
    fn link(&self, pfn: usize) -> BuddyLink {
        let desc = self.memory.get(pfn).expect("buddy PFN has no descriptor");
        // SAFETY: Zone 唯一拥有该描述符的锁域，调用者持有此 zone guard；
        // 仅复制已初始化字段，不产生可变引用，也不访问其他描述符数组。
        unsafe { desc.buddy.get().read() }
    }

    /// 持有所属 zone 锁时替换一个自由头的链状态。
    fn set_link(&mut self, pfn: usize, link: BuddyLink) {
        let desc = self.memory.get(pfn).expect("buddy PFN has no descriptor");
        // SAFETY: Zone 唯一拥有该描述符的锁域，当前 guard 排除其他字段访问；
        // UnsafeCell 内目标已初始化且永不释放，不与不可变 kind 字段重叠。
        unsafe { desc.buddy.get().write(link) };
    }

    /// 将真实自由块加入对应阶链，并在同一临界区增加自由页数。
    fn insert(&mut self, head: usize, order: usize) {
        let next = self.heads[order];
        if next != NONE {
            let mut link = self.link(next);
            link.previous = head;
            self.set_link(next, link);
        }
        self.set_link(
            head,
            BuddyLink {
                order,
                previous: NONE,
                next,
            },
        );
        self.heads[order] = head;
        self.free += 1 << order;
    }

    /// 以头内双向链 O(1) 摘块，同时清除自由头身份和相应自由页数。
    fn remove(&mut self, head: usize, order: usize) {
        let link = self.link(head);
        if link.previous == NONE {
            self.heads[order] = link.next;
        } else {
            let mut previous = self.link(link.previous);
            previous.next = link.next;
            self.set_link(link.previous, previous);
        }
        if link.next != NONE {
            let mut next = self.link(link.next);
            next.previous = link.previous;
            self.set_link(link.next, next);
        }
        self.set_link(
            head,
            BuddyLink {
                order: NONE,
                previous: NONE,
                next: NONE,
            },
        );
        self.free -= 1 << order;
    }

    /// 仅接管最终 free_ranges，按绝对 PFN 对齐拆成不越界的最大阶块。
    fn initialize(&mut self) {
        let memory = self.memory;
        for range in memory.free_ranges() {
            let mut head = range.start;
            while head < range.end {
                let mut order = (head.trailing_zeros() as usize).min(MAX_ORDER);
                while range.end - head < (1 << order) {
                    order -= 1;
                }
                self.insert(head, order);
                head += 1 << order;
            }
        }
    }

    /// 摘下最小适合阶并逐次拆分；失败不改变链，也不拼接零散页。
    fn allocate(&mut self, order: usize) -> Option<Block> {
        if MAX_ORDER < order {
            return None;
        }

        let mut found = order;
        while found <= MAX_ORDER && self.heads[found] == NONE {
            found += 1;
        }
        if MAX_ORDER < found {
            return None;
        }

        let head = self.heads[found];
        self.remove(head, found);
        while order < found {
            found -= 1;
            self.insert(head + (1 << found), found);
        }
        Some(Block { head, order })
    }

    /// 在同一自由链中选择完全落入 PFN 窗口的原阶块，失败不摘取或丢弃任何页。
    fn allocate_in(&mut self, order: usize, start: usize, end: usize) -> Option<Block> {
        if MAX_ORDER < order || end <= start {
            return None;
        }

        let count = 1usize << order;
        for found in order..=MAX_ORDER {
            let mut head = self.heads[found];
            while head != NONE {
                let first = head.max(start).checked_add(count - 1)? & !(count - 1);
                let block_end = head + (1usize << found);
                if first < end && count <= end - first && first + count <= block_end {
                    self.remove(head, found);
                    let mut chosen = head;
                    let mut split = found;
                    while order < split {
                        split -= 1;
                        let right = chosen + (1usize << split);
                        if first < right {
                            self.insert(right, split);
                        } else {
                            self.insert(chosen, split);
                            chosen = right;
                        }
                    }
                    return Some(Block {
                        head: chosen,
                        order,
                    });
                }
                head = self.link(head).next;
            }
        }
        None
    }

    /// 归还原连续块，只与同一描述符锁域中的同阶真实自由头合并。
    ///
    /// # Safety
    /// 调用者拥有本 zone 曾分配的完整 head/order 块，已结束访问且只归还一次；
    /// 该块当前不在 buddy、PCP 或 slab 的任何自由链中。
    unsafe fn release(&mut self, mut head: usize, mut order: usize) {
        while order < MAX_ORDER {
            let partner = head ^ (1 << order);
            let Some(desc) = self.memory.get(partner) else {
                break;
            };
            if desc.kind() != PageKind::Available || self.link(partner).order != order {
                break;
            }
            self.remove(partner, order);
            head = head.min(partner);
            order += 1;
        }
        self.insert(head, order);
    }
}

static ZONE: Once<SpinLock<Zone<'static>>> = Once::new();

// 独立测试元数据仅在借用期间由此锁独占，初始化不构造或复制巨型栈数组。
// 该 arena 不发布到全局 PageMap、不提供物理页 backing，也不参加生产页分配。
#[cfg(feature = "ktest")]
static TEST_DESCRIPTORS: SpinLock<[MaybeUninit<PageDesc>; 1 << MAX_ORDER]> =
    SpinLock::new([const { MaybeUninit::uninit() }; 1 << MAX_ORDER]);

/// 在 BSP 独占启动期一次性接管永久 PageMap 的最终自由页。
///
/// # Safety
/// memory 与其描述符必须永久发布；只能调用一次，IRQ 关闭、无并发 alloc/free，
/// free_ranges 尚未移交任何拥有者，描述符 buddy 字段不归其他 zone 锁域管理。
pub(crate) unsafe fn initialize(memory: &'static PageMap<'static>) {
    assert!(ZONE.get().is_none(), "buddy initialized twice");
    ZONE.call_once(|| {
        // SAFETY: 调用者保证描述符永久且此前未被任何 zone 接管。
        let zone = SpinLock::new(unsafe { Zone::empty(memory) });
        zone.lock().initialize();
        zone
    });
}

/// 不睡眠地申请真实连续块；返回前释放 zone 锁，不在锁内清页。
pub(crate) fn allocate(order: usize) -> Option<Block> {
    ZONE.get()
        .expect("buddy is not initialized")
        .lock()
        .allocate(order)
}

/// 在原 zone 一次选择可达连续页，不使用 PCP 或反复分配再丢弃的搜索。
pub(crate) fn allocate_in(order: usize, start: usize, end: usize) -> Option<Block> {
    ZONE.get()
        .expect("buddy is not initialized")
        .lock()
        .allocate_in(order, start, end)
}

/// 在一份 zone guard 下逐项摘取独立零阶块，不足 count 时保留已成功取得的块。
/// output 必须全为空；固定表仅承担移动型 Block 的临时归还责任，不要求页相邻。
pub(crate) fn allocate_batch(output: &mut [Option<Block>; BATCH_CAPACITY], count: usize) -> usize {
    assert!(
        count <= BATCH_CAPACITY,
        "buddy batch exceeds fixed capacity"
    );
    assert!(
        output.iter().all(Option::is_none),
        "buddy batch output is occupied"
    );

    let mut zone = ZONE.get().expect("buddy is not initialized").lock();
    let mut allocated = 0;
    for slot in &mut output[..count] {
        let Some(block) = zone.allocate(0) else {
            break;
        };
        *slot = Some(block);
        allocated += 1;
    }
    allocated
}

/// 在一份 zone guard 下消耗固定表中所有零阶块并归还，空槽不产生释放动作。
///
/// # Safety
/// 所有非空槽持有本 buddy 分配的不同零阶页唯一归还权，页访问已结束；
/// 页已从 PCP 链摘离，当前不属于任何 buddy、PCP 或 slab 自由链。
pub(crate) unsafe fn release_batch(blocks: &mut [Option<Block>; BATCH_CAPACITY]) {
    assert!(
        blocks.iter().flatten().all(|block| block.order == 0),
        "buddy batch contains a nonzero order"
    );

    let mut zone = ZONE.get().expect("buddy is not initialized").lock();
    for slot in blocks {
        if let Some(block) = slot.take() {
            // SAFETY: 调用者保证每份 Block 对应已摘离且访问结束的唯一零阶页；
            // take 消耗此槽归还权，本批全程持有这些页所属的同一 zone guard。
            unsafe { zone.release(block.head, 0) };
        }
    }
}

/// 将整体释放权归还唯一 zone，按原阶恢复自由链。
///
/// # Safety
/// 调用者拥有 allocate 所返同一块全部释放权和原 order；必须已结束所有访问，
/// 未归还过，且块当前不在任何 buddy、PCP 或 slab 自由链中。
pub(crate) unsafe fn release(head_pfn: usize, order: usize) {
    // SAFETY: 调用者承担完整块的唯一归还责任；固定 ZONE 即原分配锁域。
    unsafe {
        ZONE.get()
            .expect("buddy is not initialized")
            .lock()
            .release(head_pfn, order)
    };
}

/// 在 zone 锁内取得 buddy 自由页快照，不包括其他缓存保留页。
pub(crate) fn free_pages() -> usize {
    ZONE.get().expect("buddy is not initialized").lock().free
}

/// 独立受控 zone 的唯一句柄表，析构在原 zone guard 下归还尚存的所有块。
#[cfg(feature = "ktest")]
pub(crate) struct TestZone<'a, 'b> {
    zone: &'a mut Zone<'b>,
    blocks: [Option<Block>; 16],
}

#[cfg(feature = "ktest")]
impl TestZone<'_, '_> {
    /// 将生产 allocate 的真实凭证先登记，随后用例才可断言其身份。
    pub(crate) fn allocate(&mut self, index: usize, order: usize) -> bool {
        assert!(self.blocks[index].is_none());
        self.blocks[index] = self.zone.allocate(order);
        self.blocks[index].is_some()
    }

    /// 登记生产受限选择算法返回的唯一块，用于边界与归还断言。
    pub(crate) fn allocate_in(
        &mut self,
        index: usize,
        order: usize,
        start: usize,
        end: usize,
    ) -> bool {
        assert!(self.blocks[index].is_none());
        self.blocks[index] = self.zone.allocate_in(order, start, end);
        self.blocks[index].is_some()
    }

    /// 返回仍 live 块的身份和原阶，不复制其归还权。
    pub(crate) fn block(&self, index: usize) -> (usize, usize) {
        let block = self.blocks[index].as_ref().unwrap();
        (block.head, block.order)
    }

    /// 消费原凭证，按原阶执行生产合并算法。
    pub(crate) fn release(&mut self, index: usize) {
        let block = self.blocks[index].take().unwrap();
        // SAFETY: block 是此 zone 分配后仍 live 的唯一凭证，测试从不访问物理页。
        unsafe { self.zone.release(block.head, block.order) };
    }

    /// 读取当前 guard 下的真实自由页计数。
    pub(crate) fn free(&self) -> usize {
        self.zone.free
    }
}

#[cfg(feature = "ktest")]
impl Drop for TestZone<'_, '_> {
    /// 正文 panic 同样消费所有受控池凭证，不把裸 PFN 泄漏到全局页池。
    fn drop(&mut self) {
        for slot in &mut self.blocks {
            if let Some(block) = slot.take() {
                // SAFETY: 表中每槽仅拥有此 zone 的一个未归还块，未访问物理页。
                unsafe { self.zone.release(block.head, block.order) };
            }
        }
    }
}

/// 借用独立测试元数据运行实际 Zone 算法；回调结束前全部凭证及 zone 借用结束。
#[cfg(feature = "ktest")]
pub(crate) fn with_test_zone(fragmented: bool, body: impl FnOnce(&mut TestZone<'_, '_>)) {
    use crate::mm::bootmem::PageRange;
    let mut storage = TEST_DESCRIPTORS.lock();
    for (pfn, slot) in storage.iter_mut().enumerate() {
        slot.write(PageDesc::new(if fragmented {
            match pfn {
                3 | 11 => PageKind::Reserved,
                7 | 15 => PageKind::Absent,
                _ => PageKind::Available,
            }
        } else {
            PageKind::Available
        }));
    }

    // SAFETY: storage guard 独占已初始化数组，所有 map/zone 借用先于该 guard 销毁。
    let descriptors =
        unsafe { core::slice::from_raw_parts(storage.as_ptr().cast::<PageDesc>(), storage.len()) };
    let high = [const { PageDesc::new(PageKind::Available) }; 2];
    let sections = [Some(descriptors), None, Some(high.as_slice())];
    let whole = [PageRange {
        start: 0,
        end: 1 << MAX_ORDER,
    }];
    let pieces = [
        PageRange { start: 0, end: 3 },
        PageRange { start: 4, end: 7 },
        PageRange { start: 8, end: 11 },
        PageRange { start: 12, end: 15 },
        PageRange {
            start: 65536,
            end: 65538,
        },
    ];
    let memory = PageMap::test_map(&sections, if fragmented { &pieces } else { &whole });
    // SAFETY: 独立元数据只归属此局部 zone，不与全局 PageMap 或另一 zone 重叠。
    let lock = SpinLock::new(unsafe { Zone::empty(&memory) });
    let mut zone = lock.lock();
    zone.initialize();
    let mut owner = TestZone {
        zone: &mut zone,
        blocks: [const { None }; 16],
    };
    body(&mut owner);
}
