//! 匿名基本页的共享归还权；用户字节只经原始访问，不建立 Rust 内容引用。

use alloc::boxed::Box;
use alloc::sync::Arc;

use core::pin::Pin;

use crate::arch;
use crate::mm::page::PageDesc;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::VmError;
use crate::mm::{pages, OwnedPages, PAGE_SIZE};
use crate::sync::SpinLock;

/// 用途锁下保存原 order0 拥有者和真实引用数，不含用户内容视图。
pub(crate) struct AnonState {
    pub(crate) backing: OwnedPages,
    pub(crate) references: usize,
}

/// 永久描述符上的一份匿名页引用；只保活原页，不授予内容独占或访问权限。
/// 映射持有者须在最后引用释放前撤离硬件访问并完成所需的 TLB 同步。
pub struct AnonPageRef {
    descriptor: Pin<&'static PageDesc>,
}

impl AnonPageRef {
    /// 明确 fault 目标沿同一物理页入口慢路准备，仍在全部内容和映射锁外清零。
    pub(crate) fn try_new_reclaim(attempt: &mut ReclaimAttempt) -> Option<Self> {
        let backing = attempt.allocate_pages(0, true)?;
        // SAFETY: 同一生产页入口返回完整清零的独占 order0 页，尚无映射或设备，
        // 清零已经结束；慢路不会改变原拥有者及用途发表的协议。
        Some(unsafe { Self::from_owned(backing) })
    }

    /// 将完整初始化的独占基本页交给描述符用途，发布首份共享引用。
    ///
    /// # Safety
    /// backing 必须是 order0 原拥有者，整页字节已初始化；所有排他引用、
    /// 裸指针访问和设备访问均已结束，尚无硬件映射或其他内容访问责任。
    /// 后续内容访问必须使用原始访问原语并由上层维护方向、范围和并发协议，
    /// 不得从共享用户字节构造普通 Rust 引用；归还前必须结束全部硬件访问。
    pub unsafe fn from_owned(backing: OwnedPages) -> Self {
        assert_eq!(backing.order(), 0, "anonymous backing is not order0");
        let descriptor = pages()
            .get(backing.phys_addr() / PAGE_SIZE)
            .expect("anonymous PFN has no descriptor");
        {
            let mut usage = descriptor.usage();
            usage.install_anon(AnonState {
                backing,
                references: 1,
            });
        }
        // SAFETY: 生产 PageMap 一次发布永久描述符，随后不移动；首引用发布前
        // 已安装唯一匿名用途，所有用途变化都由同一用途锁与真实计数串行。
        let descriptor = unsafe { Pin::new_unchecked(descriptor) };
        Self { descriptor }
    }

    /// 返回保活原页的物理身份；整数地址不授予内容访问或再次归还权限。
    pub fn phys_addr(&self) -> usize {
        self.descriptor.usage().anon().backing.phys_addr()
    }

    /// 返回原页永久直映首址；解引用须另行满足用户访问方向、范围和并发协议。
    pub fn direct_address(&self) -> usize {
        arch::phys_to_virt(self.phys_addr())
    }
}

impl Clone for AnonPageRef {
    /// 在原引用保活期间增加用途锁下真实计数，不分配或复制用户字节。
    fn clone(&self) -> Self {
        let mut usage = self.descriptor.usage();
        let state = usage.anon_mut();
        state.references = state
            .references
            .checked_add(1)
            .expect("anonymous reference count overflow");
        Self {
            descriptor: self.descriptor,
        }
    }
}

impl Drop for AnonPageRef {
    /// 最后引用在用途锁内移出原拥有者，退出锁和 IRQ guard 后按原阶归页。
    fn drop(&mut self) {
        let discarded = {
            let mut usage = self.descriptor.usage();
            let state = usage.anon_mut();
            state.references = state
                .references
                .checked_sub(1)
                .expect("anonymous reference count underflow");
            if state.references == 0 {
                Some(usage.take_anon())
            } else {
                None
            }
        };
        drop(discarded);
    }
}

/// 一份共享匿名对象拥有按页索引保存的原页；各 Mm 另持自己的驻留与页表责任。
pub(crate) struct SharedAnon {
    pages: SpinLock<SharedPages>,
}

struct SharedPage {
    index: u64,
    page: AnonPageRef,
    next: Option<Box<SharedPage>>,
}

#[derive(Default)]
struct SharedPages {
    head: Option<Box<SharedPage>>,
}

impl SharedPages {
    /// 在对象短锁内按稳定页索引查找，不以当前 VA 或 PFN 作为对象身份。
    fn find(&self, index: u64) -> Option<&AnonPageRef> {
        // ponytail: 有限共享匿名页使用具体线性链；页数成为热点时再替换索引。
        let mut cursor = self.head.as_deref();
        while let Some(entry) = cursor {
            if entry.index == index {
                return Some(&entry.page);
            }
            cursor = entry.next.as_deref();
        }
        None
    }
}

impl Drop for SharedPages {
    /// 对象最后引用已消失，逐节点释放原页引用，避免共享页数变成析构栈深度。
    fn drop(&mut self) {
        while let Some(mut entry) = self.head.take() {
            self.head = entry.next.take();
            drop(entry);
        }
    }
}

impl SharedAnon {
    /// 在映射发表前准备独立共享对象，不提前申请尚未访问的用户页。
    pub(crate) fn try_new() -> Result<Arc<Self>, VmError> {
        Arc::try_new(Self {
            pages: SpinLock::new(SharedPages::default()),
        })
        .map_err(|_| VmError::NoMemory)
    }

    /// fault 在锁外准备共享原页及节点，与本请求目录页共用有限回收预算。
    pub(crate) fn page(
        &self,
        index: u64,
        attempt: &mut ReclaimAttempt,
    ) -> Result<AnonPageRef, VmError> {
        if let Some(page) = self.pages.lock().find(index).cloned() {
            return Ok(page);
        }
        let candidate = AnonPageRef::try_new_reclaim(attempt)
            .ok_or(VmError::NoMemory)
            .and_then(|page| {
                Box::try_new(SharedPage {
                    index,
                    page,
                    next: None,
                })
                .map_err(|_| VmError::NoMemory)
            });
        let mut pages = self.pages.lock();
        if let Some(page) = pages.find(index).cloned() {
            drop(pages);
            drop(candidate);
            return Ok(page);
        }
        let mut candidate = match candidate {
            Ok(candidate) => candidate,
            Err(error) => {
                drop(pages);
                return Err(error);
            }
        };
        let page = candidate.page.clone();
        candidate.next = pages.head.take();
        pages.head = Some(candidate);
        drop(pages);
        Ok(page)
    }
}
