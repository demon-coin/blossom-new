use crate::arch;
use crate::mm::bootmem::PAGE_SIZE;
use crate::mm::buddy::{self, Block};
use crate::mm::pcp;
use core::mem::{ManuallyDrop, MaybeUninit};

/// 持有一个真实连续页块的唯一归还权；移动或复制原始地址不会复制拥有权。
pub struct OwnedPages {
    block: Block,
}

impl OwnedPages {
    /// 返回块头物理地址，不授予该整数地址的解引用或归还权限。
    pub fn phys_addr(&self) -> usize {
        self.block.head_pfn() * PAGE_SIZE
    }

    /// 返回整块字节长度；成功申请的块始终至少包含一页。
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        PAGE_SIZE << self.order()
    }

    /// 返回唯一归还所需的原始阶数。
    pub fn order(&self) -> usize {
        self.block.order()
    }

    /// 返回整块直接映射的原始写指针；读取前须完成初始化，所有访问须在归还前结束。
    pub fn as_mut_ptr(&mut self) -> *mut MaybeUninit<u8> {
        arch::phys_to_virt(self.phys_addr()) as *mut MaybeUninit<u8>
    }

    /// 消耗页拥有者，将仍 live 整体块的唯一归还权交给原指针与原阶协议。
    /// 返回后不得再使用旧拥有者；接收者须保留原阶，最终恢复一次拥有并结束全部访问。
    pub(crate) fn into_raw(self) -> *mut u8 {
        let pages = ManuallyDrop::new(self);
        arch::phys_to_virt(pages.phys_addr()) as *mut u8
    }

    /// 从先前消费的原 live 整体块恢复唯一拥有者，不重新分配或更改自由链。
    ///
    /// # Safety
    /// pointer 必须是本类型 into_raw 返回的原直映块首址，order 必须是同块原阶；
    /// 块仍 live 且从未归还，全部访问已结束，无其他有效拥有凭证，亦不在任何
    /// buddy、PCP 或 slab 自由链中。恢复后不得继续用旧裸指针访问或再次恢复。
    pub(crate) unsafe fn from_raw(pointer: *mut u8, order: usize) -> Self {
        let head = arch::virt_to_phys(pointer as usize) / PAGE_SIZE;
        // SAFETY: 调用者保证这是 into_raw 转移的原 live 整体块，直映逆转换得到
        // 同一原首 PFN，order 未改变；访问已结束，唯一归还权不属于其他凭证或自由链。
        let block = unsafe { Block::from_raw(head, order) };
        Self { block }
    }
}

impl Drop for OwnedPages {
    /// 在全部原始访问结束后，单页交给当前 CPU 缓存，其余块按原阶一次归还 buddy。
    fn drop(&mut self) {
        if self.block.order() == 0 {
            // SAFETY: block 来自生产页入口且不能复制；本对象持有该单页唯一归还权，
            // 页未进入其他空闲链，原始指针使用者须在析构前结束全部访问。
            unsafe { pcp::release(self.block.head_pfn()) };
        } else {
            // SAFETY: block 来自 buddy 成功摘块且不能复制，保存了整块原阶；
            // 本对象持有唯一归还权，原始指针使用者须在析构前结束全部访问。
            unsafe { buddy::release(self.block.head_pfn(), self.block.order()) };
        }
    }
}

/// 非阻塞地申请一个连续页块；单页使用当前 CPU 缓存，其他阶直接请求 buddy。
/// 不足或阶数超限返回 None，不睡眠或自动回收；清零在全部锁及 IRQ guard 退出后执行。
/// 内存验收武装时，成功清零的普通 BSP 调用还须在锁外等待对应 AP 同票完成，才允许返回。
pub fn alloc_pages(order: usize, zeroed: bool) -> Option<OwnedPages> {
    #[cfg(feature = "ktest")]
    let mut test_window = crate::mm::ktest::AllocationWindow::enter();
    let block = if order == 0 {
        pcp::allocate()
    } else {
        buddy::allocate(order)
    }?;

    let mut pages = OwnedPages { block };
    if zeroed {
        let len = pages.len();
        #[cfg(feature = "ktest")]
        test_window.arm_timer();
        // SAFETY: 页入口返回已离开全部空闲链的真实连续 RAM 块，当前拥有者独占
        // 全部 len 字节，BSP 已保证永久可写直映；全部 zone/PCP 锁与 IRQ guard 已退出。
        unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0, len) };
    }
    Some(pages)
}

/// 从原 buddy 的物理半开窗口取得完整连续块，清零发生在 zone 锁退出之后。
/// 窗口两端向内取整，不自动排空 PCP、不睡眠或调用另一分配器。
pub fn alloc_pages_in(
    order: usize,
    zeroed: bool,
    physical_start: usize,
    physical_end: usize,
) -> Option<OwnedPages> {
    let start = physical_start.checked_add(PAGE_SIZE - 1)? / PAGE_SIZE;
    let end = physical_end / PAGE_SIZE;
    let block = buddy::allocate_in(order, start, end)?;

    let mut pages = OwnedPages { block };
    if zeroed {
        let len = pages.len();
        // SAFETY: 原 buddy 已摘下且唯一移交完整连续块，全部 zone 锁已释放；
        // 本拥有者独占 len 字节，启动 MM 已建立永久可写 RAM 直映。
        unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(0, len) };
    }
    Some(pages)
}
