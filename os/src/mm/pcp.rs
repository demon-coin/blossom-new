//! 每 CPU 零阶页缓存；页内链和计数由 PCP 短锁保护，批量移动只按 PCP→zone 取锁。

use spin::Once;

use crate::arch;
use crate::boot::MAX_CPUS;
use crate::cpu;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::buddy::{self, Block, BATCH_CAPACITY};
use crate::sync::{IrqSaveGuard, SpinLock};

const NONE: usize = usize::MAX;
// managed 的约千分之一先限到 1 MiB，再按 high/batch 比例缩小。
const MANAGED_BATCH_SHIFT: usize = 10;
const MAX_BATCH_BYTES: usize = 1 << 20;
const HIGH_BATCH_RATIO: usize = 4;

struct Pageset {
    head: usize,
    count: usize,
    batch: usize,
    high: usize,
}

impl Pageset {
    /// 建立不持有任何页的固定参数缓存，之后只能在本 pageset guard 下访问。
    fn new(batch: usize) -> Self {
        Self {
            head: NONE,
            count: 0,
            batch,
            high: HIGH_BATCH_RATIO * batch,
        }
    }

    /// 接管一个已结束访问的真实零阶页；先写有效链字，再发布链头和计数。
    ///
    /// # Safety
    /// head 属于本 buddy 的有效零阶页，调用者交出唯一归还权且所有访问已结束；
    /// 该页不在任何自由链中，永久可写直映仍有效，调用者持有本 pageset guard。
    unsafe fn push(&mut self, head: usize) {
        let next = arch::phys_to_virt(head * PAGE_SIZE) as *mut usize;
        // SAFETY: 调用者交出整页唯一访问和归还权，页首满足 usize 对齐；
        // 直映永久可写，当前 PCP guard 排除链访问，先写入有效 usize 再发布头。
        unsafe { next.write(self.head) };
        self.head = head;
        self.count += 1;
    }

    /// 在同一 PCP 锁域摘离链头并恢复唯一零阶 Block，不取得 zone 锁。
    fn pop(&mut self) -> Option<Block> {
        if self.head == NONE {
            return None;
        }
        let head = self.head;
        let next = arch::phys_to_virt(head * PAGE_SIZE) as *const usize;
        // SAFETY: 只有 push 发布真实持有页，且发布前已写入有效 usize 链字；
        // 当前 PCP guard 排除另一摘取或修改，读取时页面仍完全由本 PCP 持有。
        self.head = unsafe { next.read() };
        self.count -= 1;
        // SAFETY: head 刚从本 PCP 链摘离，原 buddy 零阶页的唯一归还权未复制；
        // 原阶为 0，链字读取已结束，页没有其他访问者、有效凭证或自由链成员。
        Some(unsafe { Block::from_raw(head, 0) })
    }

    /// 在 PCP→zone 锁序下补充最多 batch 个独立页，接受真实不足整批的结果。
    fn refill(&mut self) {
        let mut blocks = core::array::from_fn(|_| None);
        let allocated = buddy::allocate_batch(&mut blocks, self.batch);
        for slot in &mut blocks[..allocated] {
            let block = slot.take().expect("buddy batch count has no block");
            // SAFETY: 刚取得的 Block 持有真实零阶页唯一归还权，尚无页内容访问；
            // 当前持有 PCP guard，移动入链后此不自动 Drop 的 Block 不再被使用。
            unsafe { self.push(block.head_pfn()) };
        }
    }

    /// 在一份 PCP guard 下摘归至多 batch 个页，计数与链移动同步完成。
    fn drain_batch(&mut self, limit: usize) -> usize {
        let count = self.count.min(self.batch).min(limit);
        if count == 0 {
            return 0;
        }
        let mut blocks: [Option<Block>; BATCH_CAPACITY] = core::array::from_fn(|_| None);
        for slot in &mut blocks[..count] {
            *slot = Some(self.pop().expect("PCP count has no cached page"));
        }
        // SAFETY: pop 已逐页摘离本 PCP 链并恢复不同真实页的唯一 Block；
        // 页内容无人访问，批量归还消耗这些凭证，持锁顺序仅为 PCP→zone。
        unsafe { buddy::release_batch(&mut blocks) };
        count
    }
}

static PAGESETS: [Once<SpinLock<Pageset>>; MAX_CPUS] = [const { Once::new() }; MAX_CPUS];

/// 由初始 managed 页数推导固定小批量，不维护依赖 vmstat 的自适应参数。
fn batch_size(managed: usize) -> usize {
    let raw = ((managed >> MANAGED_BATCH_SHIFT).min(MAX_BATCH_BYTES / PAGE_SIZE)
        / HIGH_BATCH_RATIO)
        .clamp(1, BATCH_CAPACITY);
    let rounded = 1usize << (usize::BITS - 1 - (raw + raw / 2).leading_zeros());
    (rounded - 1).max(1)
}

/// 在 buddy 发布后，由 BSP 逐槽建立实际 CPU 的缓存，不申请堆或缓存页。
///
/// # Safety
/// buddy 和 CPU 表必须已初始化；仅 BSP 可在 IRQ 关闭、AP 未释放且无分配/归还时
/// 调用一次，所有可管理页的直接映射须永久可写且一一对应。
pub(crate) unsafe fn initialize() {
    assert!(PAGESETS[0].get().is_none(), "PCP initialized twice");
    let batch = batch_size(buddy::free_pages());
    for owner in cpu::actual_set().iter() {
        PAGESETS[owner].call_once(|| SpinLock::new(Pageset::new(batch)));
    }
}

/// 借用已经发布的实际 CPU 缓存，未初始化的槽不能参与分配或排空。
fn pageset(owner: usize) -> &'static SpinLock<Pageset> {
    PAGESETS[owner].get().expect("PCP is not initialized")
}

/// 保存一次 IRQ 状态固定当前 CPU，再尝试缓存锁；命中不取 zone，竞争时直接申请 buddy 零阶页。
pub(crate) fn allocate() -> Option<Block> {
    let cached = {
        let irq = IrqSaveGuard::new();
        if let Some(mut pageset) = pageset(cpu::current().id).try_lock_with_irq(irq) {
            if pageset.count == 0 {
                pageset.refill();
            }
            pageset.pop()
        } else {
            None
        }
    };
    cached.or_else(|| buddy::allocate(0))
}

/// 保存一次 IRQ 状态固定当前释放 CPU，再尝试缓存锁接收真实零阶页，达到 high 时归还一份有限 batch。
///
/// # Safety
/// 调用者交出本页唯一归还权，保留真实零阶块头，所有访问已结束且未归还过；
/// 页不在任何 buddy、PCP 或 slab 自由链中，永久可写直映仍有效。
pub(crate) unsafe fn release(head_pfn: usize) {
    let cached = {
        let irq = IrqSaveGuard::new();
        if let Some(mut pageset) = pageset(cpu::current().id).try_lock_with_irq(irq) {
            // SAFETY: 调用者交出未归还真实零阶页唯一归还权，全部访问已结束；
            // IRQ-save 固定当前 CPU，PCP guard 排除本地及远端链访问。
            unsafe { pageset.push(head_pfn) };
            if pageset.high <= pageset.count {
                let batch = pageset.batch;
                pageset.drain_batch(batch);
            }
            true
        } else {
            false
        }
    };
    if !cached {
        // SAFETY: try_lock_with_irq 失败没有接管页面，调用者的唯一零阶归还权仍完整；
        // 所有 PCP/IRQ guard 已退出，该页没有被插入任何自由链。
        unsafe { buddy::release(head_pfn, 0) };
    }
}

/// 从调用核逐一锁住各 CPU 缓存，按有限 batch 直接归还，不发 IPI。
/// 每核只处理首次快照数量，批次间放锁；并发新增缓存不延长本次排空。
/// 参与者静止时全部缓存回到 buddy，并发继续运行时只保证此次有界移动。
pub fn drain_all_pages() {
    for owner in cpu::actual_set().iter() {
        let pageset = pageset(owner);
        let mut remaining = pageset.lock().count;
        while 0 < remaining {
            let drained = pageset.lock().drain_batch(remaining);
            if drained == 0 {
                break;
            }
            remaining -= drained;
        }
    }
}

/// 持每份 PCP 锁求缓存快照，停止所有分配与归还后才是精确总数。
pub(crate) fn cached_pages() -> usize {
    cpu::actual_set()
        .iter()
        .map(|owner| pageset(owner).lock().count)
        .sum()
}

/// 持指定 CPU 的 PCP 锁读取真实缓存计数，不将并发快照当成全局账务。
#[cfg(feature = "ktest")]
pub(crate) fn cpu_cached_pages(cpu: usize) -> usize {
    assert!(cpu < cpu::count(), "PCP CPU index is out of range");
    pageset(cpu).lock().count
}

/// 提供生产固定参数供正常流测试核对真实 partial refill 与 high 边界。
#[cfg(feature = "ktest")]
pub(crate) fn parameters() -> (usize, usize) {
    let pageset = pageset(0).lock();
    (pageset.batch, pageset.high)
}
