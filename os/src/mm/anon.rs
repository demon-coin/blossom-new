//! 匿名物理页（Anonymous Page）的引用计数管理与共享匿名映射后备对象（`SharedAnon`）。
//!
//! ## 模块职责与架构定位
//! 本模块管理不关联磁盘文件的用户态匿名内存页（如用户栈、堆、私有/共享 `MAP_ANONYMOUS` 映射）：
//! - **上游调用者**：虚拟内存地址空间管理模块（`crate::mm::space`）在处理缺页异常（Page Fault）、
//!   `fork` 写时复制（CoW）共享、以及 `mmap(MAP_SHARED | MAP_ANONYMOUS)` 时调用本模块。
//! - **下游依赖**：物理页描述符表（`crate::mm::page::PageDesc`）与带内存回收的页分配入口
//!   （`crate::mm::reclaim::ReclaimAttempt`）。
//!
//! ## 核心设计与生命周期状态机
//! 1. **基于永久页描述符的引用计数（`AnonPageRef`）**：
//!    单张零阶（Order-0）物理页在转为匿名页时，将其 `OwnedPages` 所有权与引用计数 `references`
//!    存入该页对应 `PageDesc` 的 `usage` 槽位（`AnonState`）。`AnonPageRef` 仅持有指向静态
//!    `PageDesc` 的 `Pin<&'static PageDesc>`，克隆时递增 `references`，最后一个引用析构时从
//!    `PageDesc` 取出 `AnonState` 并在锁外释放底层物理页。
//!
//! ```text
//! OwnedPages (order = 0, 已清零)
//!       |
//!       | AnonPageRef::from_owned()
//!       v
//! PageDesc.usage = Anon(AnonState { backing, references: 1 })
//!       |
//!       +-- clone() --> references += 1 (CoW 共享或 MAP_SHARED 多进程映射)
//!       +-- drop()  --> references -= 1
//!                         |-- > 0: 继续驻留
//!                         |-- == 0: usage.take_anon() -> 锁外释放 OwnedPages
//! ```
//!
//! 2. **用户态内存安全契约**：
//!    由于同一张匿名物理页可能被多个进程并发映射，或与用户态发生并发读写，`AnonPageRef`
//!    绝不向内核暴露 `&[u8]` 或 `&mut [u8]` 引用，避免违反 Rust 别名规则（Aliasing Rules）。
//!    内核访问匿名页内容必须通过 `phys_addr()` / `direct_address()` 配合原始指针完成。
//!
//! 3. **共享匿名映射对象（`SharedAnon`）**：
//!    按对象内部的页索引（`index: u64`）组织已分配的 `AnonPageRef` 单向链表，采用“锁外预分配 +
//!    持锁二次检查插入”模式，避免在持有自旋锁期间执行物理页分配与清零。
//!
//! ## 锁获取顺序与并发契约
//! - **锁层级顺序**：`SharedAnon::pages` 自旋锁 ➔ `PageDesc::usage` 自旋锁 ➔ 底层页释放（`pcp`/`buddy` 锁）。
//! - **锁外释放原则**：无论是 `AnonPageRef::drop` 回收最后一页，还是 `SharedAnon::page` 并发竞争失败
//!   丢弃候选页，都必须在释放 `usage` 或 `SharedAnon::pages` 自旋锁之后再触发析构。

use alloc::boxed::Box;
use alloc::sync::Arc;

use core::pin::Pin;

use crate::arch;
use crate::mm::page::PageDesc;
use crate::mm::reclaim::ReclaimAttempt;
use crate::mm::space::VmError;
use crate::mm::{pages, OwnedPages, PAGE_SIZE};
use crate::sync::SpinLock;

/// 存储在物理页描述符（`PageDesc`）用途槽位中的匿名页状态，受 `PageDesc::usage` 自旋锁保护。
pub(crate) struct AnonState {
    /// 底层零阶（Order-0）物理页的独占所有权凭证。
    pub(crate) backing: OwnedPages,
    /// 当前存活的 [`AnonPageRef`] 强引用计数；归零时将从描述符中取出并释放 `backing`。
    pub(crate) references: usize,
}

/// 匿名物理页的引用计数句柄。
///
/// 仅负责维持底层零阶物理页的生命周期，不直接提供 Rust 内容切片引用。
/// 持有硬件页表映射的模块（`Mm`）必须在释放最后一个 `AnonPageRef` 前解除页表映射并完成必要的 TLB 刷新。
pub struct AnonPageRef {
    /// 指向该匿名页所属全局静态物理页描述符的固定引用。
    descriptor: Pin<&'static PageDesc>,
}

impl AnonPageRef {
    /// 通过带回收预算的物理页分配器申请一张已清零的零阶页，并封装为首个 [`AnonPageRef`]。
    ///
    /// 物理页分配与整页清零均在锁外执行，不会阻塞其他页描述符或地址空间锁。
    pub(crate) fn try_new_reclaim(attempt: &mut ReclaimAttempt) -> Option<Self> {
        let backing = attempt.allocate_pages(0, true)?;
        // SAFETY:
        // 1. `attempt.allocate_pages(0, true)` 返回新分配且整页已清零的零阶（order = 0）独占物理页；
        // 2. 该页尚未建立任何硬件页表映射或 DMA 访问，清零操作已结束，符合 `from_owned` 的前置条件。
        Some(unsafe { Self::from_owned(backing) })
    }

    /// 将一张已完成初始化的独占零阶物理页注册到对应 `PageDesc` 的匿名页槽位，并返回首个共享引用。
    ///
    /// # Safety
    /// 1. `backing` 必须是阶数为 0（`order == 0`）的独占物理页，且整页内容已完成初始化（如清零或拷贝完毕）；
    /// 2. 调用前对该页的所有独占引用、裸指针写入及设备访问均已结束，且尚未暴露给并发硬件页表映射；
    /// 3. 后续对该页内容的访问必须通过裸指针原语进行并由上层同步，不得构造跨越并发写者的 Rust 切片引用；
    /// 4. 在最后一个 `AnonPageRef` 释放前，上层必须已解除所有指向该物理页的页表映射并刷新 TLB。
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

        // SAFETY:
        // 1. 全局 `PageMap` 在启动期一次性初始化后永不移动或销毁，`descriptor` 具有 `'static` 生命周期且地址固定；
        // 2. 上方已在 `descriptor.usage()` 锁下安装了初始引用计数为 1 的 `AnonState`。
        let descriptor = unsafe { Pin::new_unchecked(descriptor) };
        Self { descriptor }
    }

    /// 返回该匿名页的物理起始地址（单位：字节）。
    pub fn phys_addr(&self) -> usize {
        self.descriptor.usage().anon().backing.phys_addr()
    }

    /// 返回该匿名页在内核高半区直接映射窗口中的虚拟起始地址（单位：字节）。
    pub fn direct_address(&self) -> usize {
        arch::phys_to_virt(self.phys_addr())
    }
}

impl Clone for AnonPageRef {
    /// 在持有 `PageDesc::usage` 自旋锁下递增匿名页引用计数（仅增加计数，不拷贝物理页内容）。
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
    /// 递减匿名页引用计数；若递减后归零，则在 `PageDesc::usage` 锁内移出 [`AnonState`] 并在锁外释放物理页。
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

/// 跨进程或跨映射共享的匿名内存后备对象（用于 `MAP_SHARED | MAP_ANONYMOUS` 及其 `fork` 继承）。
///
/// 按对象内页偏移索引（`index`）保存已分配的 [`AnonPageRef`]，各个进程的地址空间（`Mm`）各自管理页表项映射。
pub(crate) struct SharedAnon {
    /// 受自旋锁保护的已分配共享匿名页链表。
    pages: SpinLock<SharedPages>,
}

/// `SharedAnon` 内部单向链表的单个节点，记录某一页偏移对应的匿名物理页。
struct SharedPage {
    /// 该页在共享匿名区域内的页索引（从区域起始处算起的页号偏移，单位：页）。
    index: u64,
    /// 对应物理页的引用计数句柄。
    page: AnonPageRef,
    /// 指向链表中下一个共享页节点的独占指针。
    next: Option<Box<SharedPage>>,
}

/// `SharedAnon` 内部维护的共享匿名页单向链表头封装。
#[derive(Default)]
struct SharedPages {
    /// 单向链表头节点；为空时为 `None`。
    head: Option<Box<SharedPage>>,
}

impl SharedPages {
    /// 在持有 `SharedAnon::pages` 自旋锁的前提下，按页偏移索引 `index` 线性查找对应的 [`AnonPageRef`]。
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
    /// 循环摘除链表头节点并逐一释放，防止长链表递归析构导致内核栈溢出。
    fn drop(&mut self) {
        while let Some(mut entry) = self.head.take() {
            self.head = entry.next.take();
            drop(entry);
        }
    }
}

impl SharedAnon {
    /// 创建一个空的共享匿名后备对象（采用按需分配策略，在首次缺页访问前不预先分配物理页）。
    pub(crate) fn try_new() -> Result<Arc<Self>, VmError> {
        Arc::try_new(Self {
            pages: SpinLock::new(SharedPages::default()),
        })
        .map_err(|_| VmError::NoMemory)
    }

    /// 获取指定页索引 `index` 对应的匿名物理页引用；若尚未分配则在锁外分配并插入链表。
    pub(crate) fn page(
        &self,
        index: u64,
        attempt: &mut ReclaimAttempt,
    ) -> Result<AnonPageRef, VmError> {
        // Step 1: 快路径——先加锁查询目标页索引是否已经存在。
        if let Some(page) = self.pages.lock().find(index).cloned() {
            return Ok(page);
        }

        // Step 2: 锁外预分配——在不持有 `self.pages` 自旋锁的情况下分配清零物理页与链表节点。
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

        // Step 3: 持锁二次检查——若等待期间其他 CPU 已插入同一页索引，则在锁外丢弃刚才预分配的候选页。
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
        // Step 4: 头插发布新节点并返回克隆出的页引用。
        let page = candidate.page.clone();
        candidate.next = pages.head.take();
        pages.head = Some(candidate);
        drop(pages);
        Ok(page)
    }
}
