//! 页缓存物理页帧状态（Folio）管理、字节/扇区范围状态跟踪、内容锁与读写 I/O 凭证模块。
//!
//! ## 模块职责与架构定位
//! 本模块将单张 4KiB 物理页（order-0 `OwnedPages`）封装为引用计数的页缓存单元（`FolioRef`），
//! 为上层文件页缓存（File Page Cache）和块设备元数据缓存（Metadata Cache）提供统一的页面生命周期、
//! 有效范围追踪、脏扇区（Dirty）与在途回写（Writeback）位图管理，以及 CPU/DMA 并发访问同步。
//! - **上游调用者**：文件系统缓存层（`crate::cache`）、Inode 映射与后台回写（`crate::fs`）、
//!   缺页异常处理与用户地址空间映射（`crate::mm`）以及块设备 DMA 提交/完成层（`crate::block`）。
//! - **下游依赖**：物理页描述符表（`crate::mm::page::PageDesc`）、伙伴物理页分配器（`alloc_pages`）、
//!   任务等待队列（`crate::wait::WaitQueue`）与后台脏页统计域（`crate::fs::background`）。
//!
//! ## 核心设计与关键状态布局
//!
//! ### 1. 页描述符内联固定状态（Pinned `FolioState`）
//! 每张被用作 Folio 的物理页，其状态 `FolioState` 直接内联存储在全局静态 `PageDesc` 的用途槽（`PageUse`）中。
//! 只要引用计数 `references > 0`，`FolioState` 在内存中的地址就永久固定（Pin），从而保证内嵌的
//! `WaitQueue` 链表头和原子标志位在等待者睡眠期间绝不会移动或提前析构。
//!
//! ### 2. 4KiB 页面内的扇区（512B）位图与原子标志编码
//! 单张 4KiB 页面划分为 8 个 512 字节扇区（`SECTORS_PER_PAGE = 8`），`dirty`、`writeback`、
//! `file_prepared` 及元数据 `valid` 均使用 `u8` 位图按扇区粒度精确追踪：
//!
//! ```text
//!   4KiB 物理页扇区位图 (u8: dirty / writeback / file_prepared / Metadata::valid)
//!  ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
//!  │ Bit 7  │ Bit 6  │ Bit 5  │ Bit 4  │ Bit 3  │ Bit 2  │ Bit 1  │ Bit 0  │
//!  ├────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┤
//!  │3584..  │3072..  │2560..  │2048..  │1536..  │1024..  │ 512..  │   0..  │
//!  │  4096B │  3584B │  3072B │  2560B │  2048B │  1536B │  1024B │   512B │
//!  └────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘
//!
//!   FolioState::flags 原子状态字 (AtomicU8)
//!  ┌─────────────────────────┬───────────────┬───────────────┬────────────────┐
//!  │        Bits 7..3        │   Bit 2 (4)   │   Bit 1 (2)   │   Bit 0 (1)    │
//!  ├─────────────────────────┼───────────────┼───────────────┼────────────────┤
//!  │       保留 (0)          │   ACTIVATE    │  REFERENCED   │ CONTENT_LOCKED │
//!  │                         │ 二次访问晋升  │ 首次访问标记  │ 页内容独占锁位 │
//!  └─────────────────────────┴───────────────┴───────────────┴────────────────┘
//! ```
//!
//! ### 3. 角色分离与读写凭证状态机
//! - **文件页（`FolioContents::File`）**：允许通过页表映射给用户态共享读写。内核仅通过 `copy_user_bytes`
//!   对其执行原始字节拷贝，绝不构造 `&[u8]` / `&mut [u8]` Rust 切片，从根本上避免用户态并发写造成 Rust 别名未定义行为。
//! - **元数据页（`FolioContents::Metadata`）**：终身禁止映射给用户态，且在 `writeback != 0` 期间排斥获取内容锁，
//!   因此持有 `FolioContent` 时可安全借用 `&[u8]` / `&mut [u8]` 切片。
//! - **读入凭证（`FolioRead`）与回写凭证（`FolioWriteback`）**：采用 RAII 线性所有权模型。当物理页移交块设备执行
//!   DMA 期间，凭证对象持有一份 `FolioRef` 强引用（连同所属 `InodeRef` 或 `Arc<MetadataCache>`）；
//!   必须在设备中断/队列重置彻底结束并收回物理页 DMA 访问权后，才能调用 `complete` 发布结果或释放凭证。
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. **高层缓存/文件系统锁**（如 `Inode` 截断/失效锁、缓存索引锁 `index`、LRU 队列锁）
//! 2. **页内容睡眠锁（`CONTENT_LOCKED` / `FolioContent`）**：仅限可睡眠任务获取；等待期间不持有 `PageDesc` 自旋锁。
//! 3. **页描述符用途自旋锁（`PageDesc::usage` -> `PageUseGuard`）**：保护 `FolioShared` 的短临界区自旋锁，
//!    持锁期间严禁睡眠、严禁执行大块内存拷贝或直接唤醒调度器（唤醒等待队列或释放 `OwnedPages`/`FolioOwner`
//!    必须在释放 `PageUseGuard` 之后执行）。

use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::marker::{PhantomData, PhantomPinned};
use core::mem::{ManuallyDrop, MaybeUninit};
use core::pin::{pin, Pin};
use core::slice;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::arch;
use crate::block::{BlockError, IoResult, SECTOR_BYTES};
use crate::cache::{MetadataCache, FILE_BLOCK_BYTES_1K, FILE_BLOCK_BYTES_2K, FILE_BLOCK_BYTES_4K};
use crate::fs::background::{self, WritebackDomain};
use crate::fs::memfs::MemoryPageCharge;
use crate::fs::InodeRef;
use crate::mm::page::{PageDesc, PageUseGuard};
use crate::mm::{alloc_pages, pages, OwnedPages, PAGE_SIZE};
use crate::sched;
use crate::wait::{WaitNode, WaitQueue, Waiter};

/// 块设备标准扇区字节数（512 字节）。
const SECTOR_SIZE: usize = SECTOR_BYTES as usize;
/// 单张 4KiB 物理页包含的 512 字节扇区数量（8 个扇区）。
const SECTORS_PER_PAGE: usize = PAGE_SIZE / SECTOR_SIZE;

/// `FolioState::flags` 位掩码：页内容锁（置 1 表示当前有任务持有 `FolioContent` 或有在途 `FolioRead` 正在填充）。
const CONTENT_LOCKED: u8 = 1;
/// `FolioState::flags` 位掩码：页面近期被访问过一次（用于 LRU 二次机会老化算法）。
const REFERENCED: u8 = 2;
/// `FolioState::flags` 位掩码：页面在已处于 `REFERENCED` 状态下再次被访问，提示 LRU 将其晋升到活跃队列。
const ACTIVATE: u8 = 4;

/// 4KiB 物理页内已成功初始化的连续字节区间 `[offset, offset + len)`。
///
/// 由于页大小为 4096 字节，经过边界校验后的页内偏移量与长度均可无损表示为 `u16`。
#[derive(Clone, Copy)]
struct InitializedRange {
    /// 已初始化区间在页内的起始字节偏移（单位：字节，范围 `0..4096`）。
    offset: u16,
    /// 已初始化区间的字节长度（单位：字节，范围 `1..=4096`）。
    len: u16,
}

/// Folio 承载的具体内容角色及其初始化状态。
///
/// 页面在安装为 Folio 时即确定为文件页（`File`）或元数据页（`Metadata`），生命周期内角色永不改变。
enum FolioContents {
    /// 文件数据缓存页（只允许初始化一次有效字节范围；若是内存文件系统页则附带配额凭证）。
    File {
        /// 首次填充读入的结果：`None` 表示尚未开始或尚未完成初始化，`Some(Ok(range))` 表示已就绪区间，
        /// `Some(Err(err))` 表示读入失败。
        initialized: Option<Result<InitializedRange, BlockError>>,
        /// 若该页属于内存文件系统（memfs），则持有对应的内存页配额凭证，随最后一个 `FolioRef` 释放时归还。
        memory_charge: Option<MemoryPageCharge>,
    },
    /// 块设备元数据缓存页（按 512 字节扇区粒度独立追踪有效性与 I/O 错误）。
    Metadata {
        /// 8 位扇区有效位图：第 `i` 位为 1 表示页内第 `i` 个 512B 扇区已完成初始化且数据有效。
        valid: u8,
        /// 页内各 512B 扇区最近一次读入失败的具体块设备错误记录。
        errors: [Option<BlockError>; SECTORS_PER_PAGE],
    },
}

/// 受所属页描述符自旋锁（`PageUseGuard`）保护的 Folio 可变元数据头。
struct FolioShared {
    /// 当前存活的强引用（`FolioRef`）计数；归零时析构 `FolioState` 并归还底层物理页。
    references: usize,
    /// 页面内容角色与初始化状态（文件页或元数据页）。
    contents: FolioContents,
    /// 8 位脏扇区位图：第 `i` 位为 1 表示第 `i` 个 512B 扇区已被修改且尚未提交回写。
    dirty: u8,
    /// 8 位在途回写扇区位图：第 `i` 位为 1 表示第 `i` 个 512B 扇区正处于块设备回写（Writeback）过程中。
    writeback: u8,
    /// 文件页中已完成底层文件系统块分配与零初始化准备的 512B 扇区位图。
    file_prepared: u8,
}

/// 内联存储在全局 `PageDesc` 用途槽中的 Folio 核心状态。
///
/// 在 `references > 0` 期间，该结构体在全局页描述符数组中保持地址固定（Pinned），
/// 确保并发任务在 `waiters` 上睡眠或操作 `flags` 原子位时内存地址稳定有效。
pub(crate) struct FolioState {
    /// 底层 order-0 物理页的所有权凭证；仅在最后一个引用释放或显式摘除时取走归还给伙伴系统。
    backing: Option<OwnedPages>,
    /// 受所属 `PageDesc::usage()` 自旋锁保护的共享元数据。
    shared: UnsafeCell<FolioShared>,
    /// 原子状态位（含 `CONTENT_LOCKED`、`REFERENCED`、`ACTIVATE`），支持与自旋锁配合的无锁快路操作。
    flags: AtomicU8,
    /// 等待该页内容锁释放或在途回写完成的任务等待队列。
    waiters: WaitQueue,
    /// 标记该结构体不可移动（`!Unpin`），保障侵入式等待队列节点的内存安全。
    _pinned: PhantomPinned,
}

// SAFETY:
// 1. `backing` 在 Folio 存活期间（`references > 0`）仅被只读读取物理地址，直到最后一个引用归零并在独占状态下才被取走；
// 2. `shared` 仅在持有所属 `PageDesc` 的 `PageUseGuard` 自旋锁时通过 `shared()` / `shared_mut()` 访问，互斥性由自旋锁保证；
// 3. `flags` 为原子类型 `AtomicU8`，`waiters` 内部自带自旋锁保护等待链表，且整个 `FolioState` 在存活期间固定于静态 `PageDesc` 中不发生移动。
unsafe impl Sync for FolioState {}

/// 外部句柄附带的缓存域强引用锚点，用于在页面存活或 I/O 在途期间保活所属 Inode 或元数据缓存。
///
/// 缓存索引树（Index）内部持有的基准 `FolioRef` 不携带此字段（保持为 `None`），从而打破“Inode/Cache -> 索引 -> FolioRef -> Inode/Cache”的循环引用。
#[derive(Clone)]
enum FolioOwner {
    /// 文件缓存页所属的 VFS Inode 强引用。
    File(InodeRef),
    /// 元数据缓存页所属的块设备元数据缓存强引用。
    Metadata(Arc<MetadataCache>),
}

/// 指向单张 4KiB 页缓存物理页的引用计数句柄。
///
/// 外部调用者拿到的 `FolioRef` 会通过 `owner` 同时保活所属 `InodeRef` 或 `Arc<MetadataCache>`；
/// 缓存索引内部保存的 `FolioRef` 则令 `owner = None` 以避免循环引用。
pub struct FolioRef {
    /// 指向全局静态页描述符表中对应 `PageDesc` 的固定引用。
    descriptor: Pin<&'static PageDesc>,
    /// 可选的所属缓存域拥有者（外部句柄与在途 I/O 凭证持有，内部索引不持有）。
    owner: Option<FolioOwner>,
}

/// 页面初始读入（填充）操作的独占凭证，同时持有一份 `FolioRef` 强引用与该页的 `CONTENT_LOCKED` 锁责任。
///
/// - 若在调用 `complete` 前被丢弃（Drop），会自动以 `BlockError::QueueStopped` 发布失败状态并解锁唤醒等待者。
/// - 当页面交由块设备 DMA 读取时，DMA 层必须托管整个 `FolioRead` 直到硬件完成传输并交还物理页访问权，随后才能调用 `complete` 或析构。
pub struct FolioRead {
    /// 被填充的目标 Folio 引用（若已交设备，还附带 `FolioOwner` 保活所属 Inode 或元数据缓存）。
    folio: FolioRef,
    /// 本次读入负责填充的页内起始偏移（单位：字节）。
    offset: usize,
    /// 本次读入负责填充的字节长度（单位：字节）。
    len: usize,
    /// 标记本次读入是否已通过 `publish` 发布最终结果并释放了 `CONTENT_LOCKED`。
    completed: bool,
}

/// 页面脏扇区回写（Writeback）操作的独占凭证，持有一份 `FolioRef` 强引用并追踪本轮回写的扇区范围。
///
/// - 在块设备 DMA 传输彻底完成并收回物理页访问权之前，严禁提前结束本轮回写。
/// - 若在提交设备前因准备失败而析构（`completed == false`），`Drop` 实现会将 `original_dirty` 恢复回页面的 `dirty` 位图，确保脏数据不会静默丢失。
pub struct FolioWriteback {
    /// 正在回写的目标 Folio 引用（保活底层物理页及所属 Inode/元数据缓存）。
    folio: FolioRef,
    /// 本轮回写在页内的起始字节偏移（单位：字节，按 512B 扇区对齐）。
    offset: u16,
    /// 本轮回写的字节长度（单位：字节，按 512B 扇区对齐）。
    len: u16,
    /// 标记本轮回写是否已调用 `finish` 清除 `writeback` 位图并完成结算。
    completed: bool,
    /// 启动本轮回写时从页面 `dirty` 位图中摘除的原始脏扇区掩码，用于在回写取消或部分未受理时恢复脏位。
    original_dirty: u8,
    /// 所属块设备卷的回写统计域强引用，使中断完成路径无需升级弱引用即可直接更新卷级回写账目。
    writeback_domain: Option<Arc<WritebackDomain>>,
    /// 标记本轮回写是否由后台回写线程发起（用于区分后台轮次 I/O 流量统计）。
    background: bool,
}

/// 普通可睡眠任务持有的页内容独占锁守卫（Guard）。
///
/// - 对于**文件页**，持有此守卫仅串行化内核态的填充、清零与 `copy_to`/`copy_from` 操作，不排斥用户态通过共享可写页表直接执行内存写入，因此文件页只允许通过汇编拷贝访问，不提供 Rust 切片。
/// - 对于**元数据页**，由于不存在用户态映射且排斥在途回写，持有此守卫可安全借用 `&[u8]` 或 `&mut [u8]`。
/// - 持有此守卫期间严禁执行可能触发缺页异常（Page Fault）的用户空间内存拷贝，以免发生递归加锁死锁。
pub struct FolioContent<'a> {
    /// 被锁定的目标 `FolioRef` 借用。
    folio: &'a FolioRef,
    /// 标记当前守卫是否仍持有 `CONTENT_LOCKED` 解锁责任（在 `begin_metadata_read` 中会将锁责任转移给 `FolioRead` 并置为 `false`）。
    locked: bool,
    /// 限制 `FolioContent` 为 `!Send` 且 `!Sync`，确保仅在获取锁的当前任务上下文中使用和释放。
    _task_local: PhantomData<*mut ()>,
}

impl FolioState {
    /// 在持有所属 `PageUseGuard` 的前提下，获取 `FolioShared` 的共享不可变引用。
    fn shared<'a>(usage: &'a PageUseGuard<'_>) -> &'a FolioShared {
        let state = usage.folio();
        // SAFETY:
        // 1. `usage` 是当前 `PageDesc` 用途自旋锁的存活守卫，所有对 `shared` 的写入都必须持有同一把自旋锁；
        // 2. 返回引用的生命周期 `'a` 绑定到 `&'a PageUseGuard`，在此期间不可能存在对 `shared` 的可变借用。
        unsafe { &*state.shared.get() }
    }

    /// 在持有所属 `PageUseGuard` 可变借用的前提下，获取 `FolioShared` 的独占可变引用。
    ///
    /// 仅对 `UnsafeCell<FolioShared>` 进行内部可变投影，不创建 `&mut FolioState`，从而不违反 `Pin<&FolioState>` 的共享别名约束。
    fn shared_mut<'a>(usage: &'a mut PageUseGuard<'_>) -> &'a mut FolioShared {
        let state = usage.folio();
        // SAFETY:
        // 1. `&'a mut PageUseGuard` 保证了当前线程独占持有该 `PageDesc` 的用途自旋锁，没有任何其他并发读写者访问 `shared`；
        // 2. 仅通过 `UnsafeCell::get` 构造内部 `FolioShared` 的可变引用，不会移动外层 `FolioState` 或其中的 `WaitQueue`。
        unsafe { &mut *state.shared.get() }
    }

    /// 检查页内指定字节区间 `[offset, offset + len)` 的数据是否已完成初始化且无 I/O 错误。
    ///
    /// 调用方负责在调用前持有内容锁（`CONTENT_LOCKED`）或预先排除 `CONTENT_LOCKED` 置位状态。
    fn range_ready(shared: &FolioShared, offset: usize, len: usize) -> Result<(), BlockError> {
        match &shared.contents {
            FolioContents::File { initialized, .. } => {
                let range = initialized.ok_or(BlockError::NotReady)??;
                let start = usize::from(range.offset);
                let end = start + usize::from(range.len);
                if offset < start || end < offset + len {
                    return Err(BlockError::InvalidRange);
                }
            }
            FolioContents::Metadata { valid, errors } => {
                let sectors = sector_mask(offset, len);
                for error in &errors[offset / SECTOR_SIZE..=(offset + len - 1) / SECTOR_SIZE] {
                    if let Some(error) = *error {
                        return Err(error);
                    }
                }
                if valid & sectors != sectors {
                    return Err(BlockError::NotReady);
                }
            }
        }
        Ok(())
    }

    /// 当所有 `FolioRef` 引用计数归零且无内容锁、无脏数据、无在途回写时，从状态槽中取出底层物理页所有权。
    pub(crate) fn take_backing(&mut self) -> OwnedPages {
        assert_eq!(
            self.shared.get_mut().references,
            0,
            "removed folio retains references"
        );
        assert!(
            self.flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0,
            "removed folio retains a content owner"
        );
        let shared = self.shared.get_mut();
        assert_eq!(shared.dirty, 0, "removed folio retains dirty contents");
        assert_eq!(shared.writeback, 0, "removed folio retains writeback");
        self.backing.take().expect("folio backing already removed")
    }
}

impl FolioRef {
    /// 为外部使用的文件页句柄绑定所属的 `InodeRef` 强引用，消费并返回绑定后的 `FolioRef`。
    ///
    /// 仅用于缓存索引向外部返回句柄或构造 I/O 凭证时保活 Inode；若重复绑定同一 Inode 则保持幂等，
    /// 若尝试绑定不同 Inode 或绑定到元数据页则直接 panic。
    pub(crate) fn with_file_owner(mut self, owner: InodeRef) -> Self {
        self.attach_file_owner(owner);
        self
    }

    /// 就地为当前 `FolioRef` 绑定所属 `InodeRef`，并校验页面角色与 Inode 身份一致性。
    fn attach_file_owner(&mut self, owner: InodeRef) {
        {
            let usage = self.descriptor.usage();
            assert!(
                matches!(
                    &FolioState::shared(&usage).contents,
                    FolioContents::File { .. }
                ),
                "file owner attached to metadata folio"
            );
        }
        match &self.owner {
            None => self.owner = Some(FolioOwner::File(owner)),
            Some(FolioOwner::File(existing)) => {
                assert!(existing.ptr_eq(&owner), "folio file owner changed");
            }
            Some(FolioOwner::Metadata(_)) => panic!("folio owner role changed"),
        }
    }

    /// 为外部使用的元数据页句柄绑定所属的 `Arc<MetadataCache>` 强引用，消费并返回绑定后的 `FolioRef`。
    ///
    /// 仅用于向缓存外部或 I/O 凭证交付句柄时保活 `MetadataCache`，内部索引不持有该强引用以避免循环引用。
    pub(crate) fn with_metadata_owner(mut self, owner: Arc<MetadataCache>) -> Self {
        self.attach_metadata_owner(owner);
        self
    }

    /// 就地为当前 `FolioRef` 绑定所属 `Arc<MetadataCache>`，并校验页面角色与缓存实例指针一致性。
    fn attach_metadata_owner(&mut self, owner: Arc<MetadataCache>) {
        {
            let usage = self.descriptor.usage();
            assert!(
                matches!(
                    &FolioState::shared(&usage).contents,
                    FolioContents::Metadata { .. }
                ),
                "metadata owner attached to file folio"
            );
        }
        match &self.owner {
            None => self.owner = Some(FolioOwner::Metadata(owner)),
            Some(FolioOwner::Metadata(existing)) => {
                assert!(
                    Arc::ptr_eq(existing, &owner),
                    "folio metadata owner changed"
                );
            }
            Some(FolioOwner::File(_)) => panic!("folio owner role changed"),
        }
    }

    /// 若当前句柄绑定了元数据缓存拥有者，则返回其 `&Arc<MetadataCache>` 借用。
    pub(crate) fn metadata_owner(&self) -> Option<&Arc<MetadataCache>> {
        match &self.owner {
            Some(FolioOwner::Metadata(owner)) => Some(owner),
            None | Some(FolioOwner::File(_)) => None,
        }
    }

    /// 将单张 order-0 物理页（`OwnedPages`）封装为文件缓存页 `FolioRef`。
    ///
    /// 若传入的 `backing` 阶数不为 0，则原样通过 `Err(backing)` 退回给调用者。
    pub fn from_pages(backing: OwnedPages) -> Result<Self, OwnedPages> {
        Self::install(
            backing,
            FolioContents::File {
                initialized: None,
                memory_charge: None,
            },
        )
    }

    /// 将单张 order-0 物理页（`OwnedPages`）封装为块设备元数据缓存页 `FolioRef`。
    ///
    /// 元数据页按 512B 扇区独立追踪有效位与错误，且终身禁止映射到用户地址空间。
    pub fn from_metadata_pages(backing: OwnedPages) -> Result<Self, OwnedPages> {
        Self::install(
            backing,
            FolioContents::Metadata {
                valid: 0,
                errors: [None; SECTORS_PER_PAGE],
            },
        )
    }

    /// 将预先申请的内存文件系统（memfs）页面配额凭证绑定到当前文件页中，并撤销磁盘文件缓存页计数。
    ///
    /// 必须在该页面被发布到文件索引树之前完成调用。
    pub(crate) fn attach_memory_charge(&mut self, charge: MemoryPageCharge) {
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        let FolioContents::File { memory_charge, .. } = &mut shared.contents else {
            panic!("memory charge attached to metadata folio");
        };
        assert!(
            memory_charge.is_none(),
            "folio already owns a memory charge"
        );
        *memory_charge = Some(charge);
        background::account_file_page(false);
    }

    /// 在物理页对应的全局 `PageDesc` 空闲用途槽中安装 `FolioState`，并构造首个 `FolioRef`。
    fn install(backing: OwnedPages, contents: FolioContents) -> Result<Self, OwnedPages> {
        if backing.order() != 0 {
            return Err(backing);
        }

        let file_role = matches!(
            &contents,
            FolioContents::File {
                memory_charge: None,
                ..
            }
        );
        let descriptor = pages()
            .get(backing.phys_addr() / PAGE_SIZE)
            .expect("folio PFN has no descriptor");
        {
            let mut usage = descriptor.usage();
            usage.install_folio(FolioState {
                backing: Some(backing),
                shared: UnsafeCell::new(FolioShared {
                    references: 1,
                    contents,
                    dirty: 0,
                    writeback: 0,
                    file_prepared: 0,
                }),
                flags: AtomicU8::new(0),
                waiters: WaitQueue::new(),
                _pinned: PhantomPinned,
            });
            if file_role {
                background::account_file_page(true);
            }
        }

        // SAFETY:
        // 1. 全局 `pages()` 描述符数组在内核启动时一次性初始化于静态内存区域，所有 `PageDesc` 具有 `'static` 生命周期且物理/虚拟地址永不改变；
        // 2. 将 `descriptor` 包装为 `Pin<&'static PageDesc>` 满足固定地址要求，内嵌 `FolioState` 的固定性由引用计数与 `PageUse` 协议共同维护。
        let descriptor = unsafe { Pin::new_unchecked(descriptor) };
        Ok(Self {
            descriptor,
            owner: None,
        })
    }

    /// 从伙伴分配器申请一页未清零的 order-0 物理页，并将其初始化为文件缓存页 `FolioRef`。
    pub fn allocate() -> Option<Self> {
        let backing = alloc_pages(0, false)?;
        Some(
            Self::from_pages(backing)
                .unwrap_or_else(|_| unreachable!("order0 allocation changed order")),
        )
    }

    /// 从伙伴分配器申请一页未清零的 order-0 物理页，并将其初始化为元数据缓存页 `FolioRef`。
    pub fn allocate_metadata() -> Option<Self> {
        let backing = alloc_pages(0, false)?;
        Some(
            Self::from_metadata_pages(backing)
                .unwrap_or_else(|_| unreachable!("order0 allocation changed order")),
        )
    }

    /// 返回底层 4KiB 物理页的起始物理地址（字节），用于页表映射、身份比对或块设备 DMA 映射。
    pub fn phys_addr(&self) -> usize {
        self.state()
            .backing
            .as_ref()
            .expect("live folio lost backing")
            .phys_addr()
    }

    /// 读取当前 Folio 的强引用计数快照（仅用于内核测试断言）。
    #[cfg(feature = "ktest")]
    pub fn ref_count(&self) -> usize {
        FolioState::shared(&self.descriptor.usage()).references
    }

    /// 判断当前 Folio 是否为绑定了 `MemoryPageCharge` 的内存文件系统（memfs）文件页。
    pub(crate) fn is_memory(&self) -> bool {
        let usage = self.descriptor.usage();
        matches!(
            &FolioState::shared(&usage).contents,
            FolioContents::File {
                memory_charge: Some(_),
                ..
            }
        )
    }

    /// 检查页面初始读入流程是否已结束（仅用于内核测试）。
    ///
    /// 对于文件页，只要发布过一次读入结果（含成功或失败）且未持有内容锁即返回 `true`；
    /// 对于元数据页，要求全部 8 个扇区均已有效或已记录具体错误且未持有内容锁。
    #[cfg(feature = "ktest")]
    pub fn is_ready(&self) -> bool {
        let usage = self.descriptor.usage();
        if usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED != 0 {
            return false;
        }
        match &FolioState::shared(&usage).contents {
            FolioContents::File { initialized, .. } => initialized.is_some(),
            FolioContents::Metadata { valid, errors } => (0..SECTORS_PER_PAGE)
                .all(|sector| valid & (1 << sector) != 0 || errors[sector].is_some()),
        }
    }

    /// 认领文件缓存页的首次读入填充权，锁定页面内容并返回 `FolioRead` 凭证。
    ///
    /// 文件页在整个生命周期内只允许初始化一次非空字节区间 `[offset, offset + len)`；
    /// 若该页已完成初始化或当前已有其他任务持有 `CONTENT_LOCKED`，则返回 `Err(BlockError::Busy)`。
    pub fn begin_read(&self, offset: usize, len: usize) -> Result<FolioRead, BlockError> {
        validate_range(offset, len)?;
        let state = self.state();
        {
            let mut usage = self.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            match &shared.contents {
                FolioContents::File {
                    initialized: None, ..
                } => {}
                FolioContents::File { .. } => return Err(BlockError::Busy),
                FolioContents::Metadata { .. } => return Err(BlockError::InvalidRange),
            }
            if state.flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0 {
                return Err(BlockError::Busy);
            }
            shared.references = shared
                .references
                .checked_add(1)
                .expect("folio reference count overflow");
            state.flags.fetch_or(CONTENT_LOCKED, Ordering::Acquire);
        }
        Ok(FolioRead {
            folio: Self {
                descriptor: self.descriptor,
                owner: None,
            },
            offset,
            len,
            completed: false,
        })
    }

    /// 在元数据页中锁定内容并认领区间 `[offset, offset + len)` 内首段连续尚未初始化的扇区。
    ///
    /// - `offset` 与 `len` 必须按 512 字节扇区对齐。
    /// - 若扫描过程中遇到某扇区曾记录过读入错误，则立即返回该具体错误。
    /// - 若请求区间内的所有扇区均已有效（`valid`），则释放内容锁并返回 `Ok(None)`。
    /// - 若存在未初始化扇区，则截取首段连续未初始化且无错误的子区间构造 `FolioRead`，并将内容锁责任直接移交给该凭证。
    pub fn begin_metadata_read(
        &self,
        offset: usize,
        len: usize,
    ) -> Result<Option<FolioRead>, BlockError> {
        validate_range(offset, len)?;
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }

        let mut content = self.lock_content();
        let range = {
            let mut usage = self.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            let FolioContents::Metadata { valid, errors } = &shared.contents else {
                return Err(BlockError::InvalidRange);
            };
            // Step 1: 跳过区间前端已经有效（valid）的扇区；若遇到历史 I/O 错误则立即中止并返回错误。
            let end = (offset + len) / SECTOR_SIZE;
            let mut first = offset / SECTOR_SIZE;
            while first < end {
                if let Some(error) = errors[first] {
                    return Err(error);
                }
                if valid & (1 << first) == 0 {
                    break;
                }
                first += 1;
            }
            if first == end {
                return Ok(None);
            }
            // Step 2: 从首个未就绪扇区开始，向后合并连续未初始化且无错误的扇区，避免覆盖后续已有效的扇区数据。
            let mut last = first + 1;
            while last < end && valid & (1 << last) == 0 && errors[last].is_none() {
                last += 1;
            }
            shared.references = shared
                .references
                .checked_add(1)
                .expect("folio reference count overflow");
            (first * SECTOR_SIZE, (last - first) * SECTOR_SIZE)
        };
        // Step 3: 将 `FolioContent` 持有的 `CONTENT_LOCKED` 锁责任直接移交给 `FolioRead` 凭证，
        // 使读入操作可以跨任务或交给异步 DMA 中断完成，而无需在 `FolioContent::drop` 中提前解锁。
        content.locked = false;
        Ok(Some(FolioRead {
            folio: Self {
                descriptor: self.descriptor,
                owner: None,
            },
            offset: range.0,
            len: range.1,
            completed: false,
        }))
    }

    /// 阻塞等待当前页面的内容锁释放，并检查页面初始数据是否已就绪。
    ///
    /// 对于文件页，检查首次初始化是否成功；对于元数据页，检查整页（`0..PAGE_SIZE`）全部扇区是否均有效。
    pub fn wait_ready(&self) -> Result<(), BlockError> {
        let _content = self.lock_content();
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        match &shared.contents {
            FolioContents::File { initialized, .. } => {
                initialized.ok_or(BlockError::NotReady)?.map(|_| ())
            }
            FolioContents::Metadata { .. } => FolioState::range_ready(shared, 0, PAGE_SIZE),
        }
    }

    /// 非阻塞检查页内字节区间 `[offset, offset + len)` 是否已初始化就绪且当前未被锁定。
    pub fn range_ready(&self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let usage = self.descriptor.usage();
        if usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED != 0 {
            return Err(BlockError::NotReady);
        }
        FolioState::range_ready(FolioState::shared(&usage), offset, len)
    }

    /// 在可睡眠任务中获取页面内容锁，并将页内 `[offset, offset + dst.len())` 的已初始化数据拷贝到内核缓冲区 `dst`。
    ///
    /// 若 `dst` 为空切片，仅校验偏移量合法性而不加锁访问页面。
    pub fn copy_to(&self, offset: usize, dst: &mut [u8]) -> Result<(), BlockError> {
        validate_copy_range(offset, dst.len())?;
        if dst.is_empty() {
            return Ok(());
        }
        self.lock_content().copy_to(offset, dst)
    }

    /// 在可睡眠任务中阻塞获取页面内容独占锁，返回 `FolioContent` 守卫。
    ///
    /// 采用“先尝试加锁 -> 失败则将栈上固定节点登记到 `WaitQueue` -> 二次检查锁状态 -> 睡眠等待”的标准协议，
    /// 确保不会因唤醒信号与睡眠操作交错而发生丢唤醒（Lost Wakeup）。
    pub fn lock_content(&self) -> FolioContent<'_> {
        sched::assert_switchable();
        let state = self.state();
        loop {
            if let Some(content) = self.try_lock_content() {
                return content;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self` 持有一份存活的 `FolioRef` 强引用，保证静态 `PageDesc` 内的 `state.waiters` 在整个等待期间地址固定且不被析构；
            // 2. `node` 通过 `pin!` 宏固定在当前任务栈帧上，`_registration` 守卫在循环每次迭代结束或提前 `return` 时都会先从链表摘除节点，随后才析构 `node` 与 `waiter`。
            let _registration = unsafe { state.waiters.register(node.as_mut()) };
            if let Some(content) = self.try_lock_content() {
                return content;
            }
            waiter.park();
        }
    }

    /// 尝试非阻塞地获取页面内容锁（`CONTENT_LOCKED`）。
    ///
    /// 若当前已有其他任务持有 `CONTENT_LOCKED`，或者当前页面为正在执行块设备回写（`writeback != 0`）的元数据页，
    /// 则立即返回 `None` 而不睡眠等待。
    pub(crate) fn try_lock_content(&self) -> Option<FolioContent<'_>> {
        let usage = self.descriptor.usage();
        let state = usage.folio();
        let shared = FolioState::shared(&usage);
        if state.flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0
            || matches!(&shared.contents, FolioContents::Metadata { .. }) && shared.writeback != 0
        {
            return None;
        }
        state.flags.fetch_or(CONTENT_LOCKED, Ordering::Acquire);
        Some(FolioContent {
            folio: self,
            locked: true,
            _task_local: PhantomData,
        })
    }

    /// 检查页面当前是否仅由单一持有者引用（`references == 1`），且无在途回写、无内容锁占用。
    pub(crate) fn is_cache_only(&self) -> bool {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        shared.references == 1
            && shared.writeback == 0
            && usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0
    }

    /// 标记页面被访问过一次，驱动二次机会（Second-Chance）LRU 老化状态机：
    /// 首次访问置 `REFERENCED`，若 `REFERENCED` 已置位则同时置 `ACTIVATE`。
    pub(crate) fn mark_accessed(&self) {
        let _ = self
            .state()
            .flags
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |flags| {
                let access = if flags & REFERENCED == 0 {
                    REFERENCED
                } else {
                    REFERENCED | ACTIVATE
                };
                Some(flags | access)
            });
    }

    /// 原子读取并清除页面的 `REFERENCED` 与 `ACTIVATE` 访问标志位，返回 `(是否曾被引用, 是否应晋升活跃队列)`。
    pub(crate) fn take_accessed(&self) -> (bool, bool) {
        let previous = self
            .state()
            .flags
            .fetch_and(!(REFERENCED | ACTIVATE), Ordering::Relaxed);
        (previous & REFERENCED != 0, previous & ACTIVATE != 0)
    }

    /// 显式消费当前 `FolioRef` 强引用；若此次释放使引用计数归零并实际回收了一页磁盘文件缓存页，则返回 `1`，否则返回 `0`。
    ///
    /// 所属 `owner`（`InodeRef` 或 `Arc<MetadataCache>`）会在释放 `PageDesc` 自旋锁之后再析构，避免锁序反转。
    pub(crate) fn release(self) -> usize {
        let mut this = ManuallyDrop::new(self);
        let freed = this.put();
        drop(this.owner.take());
        freed
    }

    /// 递减一次 `FolioRef` 引用计数；当最后一个强引用被释放（`references == 0`）时，摘除 `FolioState` 并归还物理页。
    fn put(&mut self) -> usize {
        let discarded = {
            let mut usage = self.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            shared.references = shared
                .references
                .checked_sub(1)
                .expect("folio reference count underflow");
            if shared.references == 0 {
                assert_eq!(
                    shared.dirty, 0,
                    "last folio reference retains dirty contents"
                );
                assert_eq!(
                    shared.writeback, 0,
                    "last folio reference retains writeback"
                );
                let file = matches!(
                    &shared.contents,
                    FolioContents::File {
                        memory_charge: None,
                        ..
                    }
                );
                let charge = match &mut shared.contents {
                    FolioContents::File { memory_charge, .. } => memory_charge.take(),
                    FolioContents::Metadata { .. } => None,
                };
                assert_eq!(
                    usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED,
                    0,
                    "last folio reference retains a content owner"
                );
                // SAFETY:
                // 1. 前置检查已确认 `references == 0`、`dirty == 0`、`writeback == 0` 且 `CONTENT_LOCKED == 0`；
                // 2. 所有在途读入（`FolioRead`）、回写（`FolioWriteback`）及等待队列上的任务都各自持有 `FolioRef` 强引用，
                //    因此 `references == 0` 证明已不存在任何并发访问者或 `Pin<&FolioState>` 借用；
                // 3. 在 `PageUseGuard` 保护下调用 `take_folio()` 原地析构 `FolioState` 并抽出 `OwnedPages`，随后在锁外释放物理页。
                Some((unsafe { usage.take_folio() }, file, charge))
            } else {
                None
            }
        };

        if let Some(FolioOwner::Metadata(cache)) = &self.owner {
            cache.notify_metadata_available();
        }
        if let Some((backing, file, charge)) = discarded {
            drop(backing);
            drop(charge);
            if file {
                background::account_file_page(false);
            }
            usize::from(file)
        } else {
            0
        }
    }

    /// 检查元数据缓存页是否处于可被空闲池复用的干净静止状态（仅剩库存基准引用 `references == 1`、无脏扇区、无回写、无内容锁且未绑定外部 owner）。
    pub(crate) fn metadata_inventory_available(&self) -> bool {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        shared.references == 1
            && matches!(&shared.contents, FolioContents::Metadata { .. })
            && shared.dirty == 0
            && shared.writeback == 0
            && usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0
            && self.owner.is_none()
    }

    /// 检查缓存索引中的页面是否处于无外部使用的静止状态（引用数等于索引基准引用加上可选的元数据库存基准引用，且无在途回写与内容锁）。
    pub(crate) fn index_unused(&self, metadata_baseline: bool, allow_dirty: bool) -> bool {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        shared.references == 1 + usize::from(metadata_baseline)
            && (allow_dirty || shared.dirty == 0)
            && shared.writeback == 0
            && usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED == 0
    }

    /// 在持有缓存索引锁与 LRU 锁的最终临界区内，校验干净文件页是否仍无外部引用，并在满足条件时执行 `retire` 摘除闭包。
    ///
    /// `scanner` 为回收扫描器额外持有的同一页引用（若存在则允许总引用数为 2，否则为 1）；
    /// `content` 为调用方可能已持有的本页内容锁守卫。`retire` 闭包在 `PageDesc` 自旋锁内执行，因此严禁在闭包内执行内存分配、睡眠或析构物理页。
    pub(crate) fn retire_clean<T>(
        &self,
        scanner: Option<&Self>,
        content: Option<&FolioContent<'_>>,
        retire: impl FnOnce() -> T,
    ) -> Option<T> {
        if scanner.is_some_and(|folio| {
            !core::ptr::eq(self.descriptor.get_ref(), folio.descriptor.get_ref())
        }) {
            return None;
        }
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        if shared.references != 1 + usize::from(scanner.is_some()) {
            return None;
        }
        let our_content = content.is_some_and(|guard| {
            guard.locked
                && core::ptr::eq(self.descriptor.get_ref(), guard.folio.descriptor.get_ref())
        });
        if (usage.folio().flags.load(Ordering::Acquire) & CONTENT_LOCKED != 0) != our_content
            || shared.dirty != 0
            || shared.writeback != 0
            || !matches!(
                &shared.contents,
                FolioContents::File {
                    initialized: Some(Ok(_)),
                    ..
                }
            )
        {
            return None;
        }
        Some(retire())
    }

    /// 重置空闲元数据缓存页的所有扇区有效位（`valid = 0`）与错误记录，准备将其重新分配给新的元数据块使用。
    pub(crate) fn reset_metadata_inventory(&self) {
        let content = self
            .try_lock_content()
            .expect("metadata inventory acquired a busy folio");
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(
            shared.references, 2,
            "metadata inventory retains an external owner"
        );
        assert_eq!(shared.dirty, 0, "metadata inventory retains dirty contents");
        assert_eq!(shared.writeback, 0, "metadata inventory retains writeback");
        match &mut shared.contents {
            FolioContents::Metadata { valid, errors } => {
                *valid = 0;
                *errors = [None; SECTORS_PER_PAGE];
            }
            FolioContents::File { .. } => panic!("metadata inventory has a file folio"),
        }
        drop(usage);
        drop(content);
    }

    /// 当元数据页内指定扇区窗口 `[offset, offset + len)` 的最后一个块缓冲区引用结束时，清除该窗口的有效位与历史错误记录。
    ///
    /// 此函数仅修改 `FolioShared` 元数据位图而不访问物理页字节，支持在中断上下文中调用。
    ///
    /// # Safety
    /// 1. 调用方必须持有对应元数据块的地址准入锁，确保 `[offset, offset + len)` 窗口当前没有任何存活的缓冲区借用、在途读入（`FolioRead`）或在途回写（`FolioWriteback`）；
    /// 2. 在当前回收流程释放准入锁之前，不得允许新的任务重新认领该扇区窗口；
    /// 3. 同页内的其他扇区窗口可能正处于并发读写或 DMA 传输中，本操作仅通过位掩码修改目标扇区的状态位。
    pub(crate) unsafe fn retire_metadata(&self, offset: usize, len: usize) {
        assert!(validate_range(offset, len).is_ok());
        assert_eq!(offset % SECTOR_SIZE, 0);
        assert_eq!(len % SECTOR_SIZE, 0);
        let mask = sector_mask(offset, len);
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(
            shared.dirty & mask,
            0,
            "retired metadata retains dirty contents"
        );
        assert_eq!(
            shared.writeback & mask,
            0,
            "retired metadata retains writeback"
        );
        let FolioContents::Metadata { valid, errors } = &mut shared.contents else {
            panic!("metadata retirement received a file folio");
        };
        *valid &= !mask;
        errors[offset / SECTOR_SIZE..(offset + len) / SECTOR_SIZE].fill(None);
    }

    /// 返回当前页面 8 个 512B 扇区的脏位图快照（`dirty`）。
    pub fn dirty_sectors(&self) -> u8 {
        FolioState::shared(&self.descriptor.usage()).dirty
    }

    /// 返回当前页面 8 个 512B 扇区的在途回写位图快照（`writeback`）。
    pub fn writeback_sectors(&self) -> u8 {
        FolioState::shared(&self.descriptor.usage()).writeback
    }

    /// 在同一次 `PageDesc` 自旋锁临界区内原子读取 `(dirty, writeback)` 位图快照，防止在将脏位转为回写位的交界处出现观测漏失。
    pub(crate) fn writeback_state(&self) -> (u8, u8) {
        let usage = self.descriptor.usage();
        let shared = FolioState::shared(&usage);
        (shared.dirty, shared.writeback)
    }

    /// 在可睡眠任务中阻塞等待当前页面的所有在途块设备回写（`writeback`）全部完成。
    pub fn wait_writeback(&self) {
        sched::assert_switchable();
        let state = self.state();
        loop {
            if self.writeback_sectors() == 0 {
                return;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self` 持有的 `FolioRef` 强引用确保 `state.waiters` 在静态 `PageDesc` 内地址固定且存活；
            // 2. `node` 通过 `pin!` 固定在当前任务栈上，且所有退出分支都会先析构 `_registration` 将节点从等待队列移除，再析构栈上的 `node` 与 `waiter`。
            let _registration = unsafe { state.waiters.register(node.as_mut()) };
            if self.writeback_sectors() == 0 {
                return;
            }
            waiter.park();
        }
    }

    /// 在持有 `PageDesc` 自旋锁期间，根据 `dirty` 与 `writeback` 位图的前后变化更新全局及所属磁盘卷的回写统计计数。
    fn account_state(&self, old_dirty: u8, old_wb: u8, shared: &FolioShared) {
        if matches!(
            &shared.contents,
            FolioContents::File {
                memory_charge: Some(_),
                ..
            }
        ) {
            return;
        }
        background::account_folio(old_dirty, shared.dirty, old_wb, shared.writeback);
        if let Some(FolioOwner::File(inode)) = &self.owner {
            inode
                .disk_volume()
                .expect("disk folio accounting has no volume")
                .writeback_domain()
                .account_folio(old_dirty, shared.dirty, old_wb, shared.writeback);
        }
    }

    /// 在释放 `PageDesc` 自旋锁之后，向所属磁盘卷的回写域通知脏页状态变化；若 `relieved` 为 `true`（脏页/回写压力减轻），则唤醒因脏页限流而阻塞的写入任务。
    fn notify_dirty_state(&self, relieved: bool) {
        if self.is_memory() {
            return;
        }
        if let Some(FolioOwner::File(inode)) = &self.owner {
            let volume = inode.disk_volume().expect("disk dirty folio has no volume");
            let domain = volume.writeback_domain();
            if self.dirty_sectors() != 0 {
                domain.notify_dirty(volume);
            }
            if relieved {
                domain.wake_writers();
            }
        }
    }

    /// 获取指向当前 Folio 内部状态 `Pin<&FolioState>` 的固定借用。
    ///
    /// 先在短暂的 `PageUseGuard` 自旋锁下获取 `FolioState` 地址，随后释放自旋锁并返回绑定到 `&self` 生命周期的 `Pin` 引用，
    /// 从而允许调用者在自旋锁外安全地操作原子标志 `flags` 或在 `waiters` 等待队列上睡眠。
    fn state(&self) -> Pin<&FolioState> {
        let pointer = {
            let usage = self.descriptor.usage();
            usage.folio() as *const FolioState
        };
        // SAFETY:
        // 1. `FolioRef::install` 在发布首个 `FolioRef` 之前已完成 `FolioState` 的初始化写入，且存活期间仅通过 `UnsafeCell` 修改 `FolioShared`，绝不创建 `&mut FolioState` 或移动状态；
        // 2. `&self` 保证当前至少存在一份强引用（`references >= 1`），因此在返回借用的生命周期内不可能触发最后一个引用释放（`take_folio`）；
        // 3. `PageDesc` 永久固定于静态内存中，故 `pointer` 指向的 `FolioState` 在 `&self` 存活期间始终有效且地址固定。
        unsafe { Pin::new_unchecked(&*pointer) }
    }
}

impl Clone for FolioRef {
    /// 克隆 `FolioRef` 句柄：递增底层 `FolioShared::references` 引用计数，并同步克隆可选的 `FolioOwner`（不复制物理页数据）。
    fn clone(&self) -> Self {
        let owner = self.owner.clone();
        let mut usage = self.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        shared.references = shared
            .references
            .checked_add(1)
            .expect("folio reference count overflow");
        Self {
            descriptor: self.descriptor,
            owner,
        }
    }
}

impl Drop for FolioRef {
    /// 释放 `FolioRef` 句柄：递减引用计数；若为最后一个强引用则归还底层物理页，随后由编译器自动析构 `owner`。
    fn drop(&mut self) {
        self.put();
    }
}

impl FolioContent<'_> {
    /// 返回当前内容锁守卫所锁定的 `&FolioRef` 借用。
    pub(crate) fn folio(&self) -> &FolioRef {
        self.folio
    }

    /// 在持有内容锁期间读取当前页面的在途回写扇区位图（`writeback`）。
    pub(crate) fn writeback_sectors(&self) -> u8 {
        self.folio.writeback_sectors()
    }

    /// 读取文件页内已完成底层文件系统块分配与零初始化准备的 512B 扇区位图（`file_prepared`）。
    pub(crate) fn file_prepared_sectors(&self) -> u8 {
        FolioState::shared(&self.folio.descriptor.usage()).file_prepared
    }

    /// 将文件页内已完成底层磁盘块分配和初始化的区间 `[offset, offset + len)` 标记到 `file_prepared` 位图中。
    ///
    /// 调用方必须同时持有所属 Inode 的截断/失效锁与本页内容锁，且 `offset` 与 `len` 必须按 512 字节扇区对齐。
    pub(crate) fn mark_file_prepared(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }

        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        if !matches!(&shared.contents, FolioContents::File { .. }) {
            return Err(BlockError::InvalidRange);
        }
        FolioState::range_ready(shared, offset, len)?;
        shared.file_prepared |= sector_mask(offset, len);
        Ok(())
    }

    /// 将文件页内位于当前文件有效长度（`0..valid_bytes`，按文件系统块大小向上取整）范围内、且已在 `file_prepared` 中准备好的块标记为脏（`dirty`）。
    ///
    /// 未映射的文件空洞（Hole）或超出文件末尾（EOF）的块不在 `file_prepared` 中，因此不会被标记为脏扇区。
    /// 若该页为内存文件系统（memfs）页面，则直接返回 `Ok(())`。
    pub(crate) fn mark_prepared_dirty(
        &mut self,
        valid_bytes: usize,
        block_size: usize,
    ) -> Result<(), BlockError> {
        if PAGE_SIZE < valid_bytes
            || !matches!(
                block_size,
                FILE_BLOCK_BYTES_1K | FILE_BLOCK_BYTES_2K | FILE_BLOCK_BYTES_4K
            )
        {
            return Err(BlockError::InvalidRange);
        }
        if self.folio.is_memory() {
            return Ok(());
        }

        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        if !matches!(&shared.contents, FolioContents::File { .. }) {
            return Err(BlockError::InvalidRange);
        }
        if valid_bytes == 0 {
            return Ok(());
        }
        let end = valid_bytes.div_ceil(block_size) * block_size;
        FolioState::range_ready(shared, 0, end)?;
        let old_dirty = shared.dirty;
        shared.dirty |= shared.file_prepared & sector_mask(0, end);
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(false);
        Ok(())
    }

    /// 将文件页内已初始化的字节区间 `[offset, offset + len)` 清零（不修改脏位图）。
    ///
    /// 调用方在截断或空洞清零时应先撤销用户态共享可写映射并完成 TLB 刷新，再调用本方法。
    pub(crate) fn zero_file_range(&mut self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_copy_range(offset, len)?;
        if len == 0 {
            return Ok(());
        }
        {
            let usage = self.folio.descriptor.usage();
            let shared = FolioState::shared(&usage);
            if !matches!(&shared.contents, FolioContents::File { .. }) {
                return Err(BlockError::InvalidRange);
            }
            FolioState::range_ready(shared, offset, len)?;
        }

        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        let zeros = [0u8; SECTOR_SIZE];
        for part in (0..len).step_by(zeros.len()) {
            let count = (len - part).min(zeros.len());
            // SAFETY:
            // 1. 当前任务持有 `FolioContent` 守卫，保证底层物理页存活且串行化内核态内容修改，前文 `range_ready` 已验证 `[offset, offset + len)` 位于单页已初始化范围内；
            // 2. `zeros` 是栈上独立的 512B 数组，与目标物理页的直接映射虚拟地址绝不重叠；
            // 3. 使用 `arch::copy_user_bytes` 执行原始字节写入，避免对可能存在用户态映射的文件页构造 `&mut [u8]` 切片。
            unsafe { arch::copy_user_bytes((address + part) as *mut u8, zeros.as_ptr(), count) };
        }
        Ok(())
    }

    /// 在文件缩小（Truncate）时处理新的文件尾页：将页内偏移 `valid` 之后的尾部字节清零，并清除已超出新文件大小的文件系统块的 `dirty` 与 `file_prepared` 位。
    ///
    /// 若 `valid` 落在一个文件系统块的中间，则保留该部分块（`0..retained`）的准备状态，并在该块已准备的前提下将其标记为脏，以便后续将该块的尾零状态持久化到磁盘。
    ///
    /// # Safety
    /// 1. 调用方必须持有所属 Inode 的写互斥锁、失效（invalidate）独占锁及映射回写互斥锁，且新的文件大小已发布；
    /// 2. 该页的所有用户态共享可写页表映射必须已撤销并完成跨核 TLB 同步，且页上不存在任何在途回写（`writeback == 0`）；
    /// 3. 被截断丢弃的尾部整块确定不再需要落盘持久化。
    pub(crate) unsafe fn truncate_file_tail(
        &mut self,
        valid: usize,
        block_size: usize,
    ) -> Result<(), BlockError> {
        if valid == 0
            || PAGE_SIZE <= valid
            || !matches!(
                block_size,
                FILE_BLOCK_BYTES_1K | FILE_BLOCK_BYTES_2K | FILE_BLOCK_BYTES_4K
            )
        {
            return Err(BlockError::InvalidRange);
        }
        assert_eq!(
            self.writeback_sectors(),
            0,
            "file truncate retains writeback"
        );
        // Step 1: 将新文件末尾 `valid` 到页尾 `PAGE_SIZE` 的字节全部清零。
        self.zero_file_range(valid, PAGE_SIZE - valid)?;
        // Step 2: 计算按文件系统块大小向上取整后仍需保留的字节边界，清除已删除尾块的 dirty 与 file_prepared 位。
        let retained = valid.div_ceil(block_size) * block_size;
        let keep = sector_mask(0, retained);
        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        let old_dirty = shared.dirty;
        shared.dirty &= keep;
        shared.file_prepared &= keep;
        // Step 3: 若新 EOF 落在某个已准备好的文件系统块内部，将该块标记为脏以确保尾部清零能回写落盘。
        if valid % block_size != 0 {
            let start = valid / block_size * block_size;
            shared.dirty |= shared.file_prepared & sector_mask(start, block_size);
        }
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(true);
        Ok(())
    }

    /// 在持有内容锁期间检查页内字节区间 `[offset, offset + len)` 是否已初始化就绪且无 I/O 错误。
    pub fn range_ready(&self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let usage = self.folio.descriptor.usage();
        FolioState::range_ready(FolioState::shared(&usage), offset, len)
    }

    /// 在持有内容锁期间，将页内已初始化区间 `[offset, offset + dst.len())` 的字节原始拷贝到调用方独占缓冲区 `dst`。
    pub fn copy_to(&self, offset: usize, dst: &mut [u8]) -> Result<(), BlockError> {
        validate_copy_range(offset, dst.len())?;
        if dst.is_empty() {
            return Ok(());
        }
        self.range_ready(offset, dst.len())?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY:
        // 1. `self` 守卫保活底层物理页，且 `self.range_ready` 已确认 `[offset, offset + dst.len())` 在页内已初始化；
        // 2. `dst` 为调用方独占的可变字节切片，位于普通内核栈或堆内存中，与页缓存物理页不重叠；
        // 3. 通过 `arch::copy_user_bytes` 进行原始指针拷贝，不构造文件页的 `&[u8]` 引用，即使并发存在用户态共享页表写入也不会触发 Rust 别名未定义行为。
        unsafe { arch::copy_user_bytes(dst.as_mut_ptr(), address as *const u8, dst.len()) };
        Ok(())
    }

    /// 在持有内容锁期间，先将页内 `[offset, offset + src.len())` 覆盖的扇区标记为脏（`dirty`），再从内核切片 `src` 将字节原始拷贝到页内。
    pub fn copy_from(&mut self, offset: usize, src: &[u8]) -> Result<(), BlockError> {
        validate_copy_range(offset, src.len())?;
        if src.is_empty() {
            return Ok(());
        }
        self.mark_dirty(offset, src.len())?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY:
        // 1. `self` 可变守卫保活底层物理页并串行化内核态写入，`self.mark_dirty` 已验证 `[offset, offset + src.len())` 位于页内已初始化范围内；
        // 2. `src` 为有效的只读内核切片，与目标物理页内存不重叠；
        // 3. 通过 `arch::copy_user_bytes` 执行有界原始字节写入，不构造文件页的 `&mut [u8]` 引用。
        unsafe { arch::copy_user_bytes(address as *mut u8, src.as_ptr(), src.len()) };
        Ok(())
    }

    /// 借用元数据页内已初始化区间 `[offset, offset + len)` 的只读字节切片 `&[u8]`。
    ///
    /// 仅限 `FolioContents::Metadata` 角色调用；若页面为文件页或目标扇区存在在途回写，则返回错误。
    /// 持有返回切片期间严禁执行可能触发缺页异常的用户空间访问。
    pub fn metadata_bytes(&self, offset: usize, len: usize) -> Result<&[u8], BlockError> {
        self.metadata_range(offset, len)?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY:
        // 1. `self.metadata_range` 已确认该页为终身 `Metadata` 角色（绝无用户态映射）、无在途回写，且 `[offset, offset + len)` 全部扇区已初始化有效；
        // 2. `self` 内容锁守卫排斥任何其他内核态 CPU 修改或读入 DMA，返回的不可变切片生命周期绑定到 `&self`。
        Ok(unsafe { slice::from_raw_parts(address as *const u8, len) })
    }

    /// 借用元数据页内已初始化区间 `[offset, offset + len)` 的可变字节切片 `&mut [u8]`，并在返回前将对应扇区标记为脏（`dirty`）。
    ///
    /// 仅限 `FolioContents::Metadata` 角色调用；若目标扇区存在在途回写则返回 `Err(BlockError::Busy)`。
    pub fn metadata_bytes_mut(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<&mut [u8], BlockError> {
        self.metadata_range(offset, len)?;
        self.mark_dirty(offset, len)?;
        let address = arch::phys_to_virt(self.folio.phys_addr()) + offset;
        // SAFETY:
        // 1. `self.metadata_range` 与 `self.mark_dirty` 已确认该页为 `Metadata` 角色、目标区间已初始化且无在途回写；
        // 2. `&mut self` 确保独占持有当前页面的 `CONTENT_LOCKED` 内容锁，排斥任何其他 CPU 读写或块设备 DMA 访问，故构造绑定到 `&mut self` 的 `&mut [u8]` 满足独占别名要求。
        Ok(unsafe { slice::from_raw_parts_mut(address as *mut u8, len) })
    }

    /// 在文件截断删除或缓存失效流程中，作废整页的初始化状态、错误记录、`dirty` 位图与 `file_prepared` 位图。
    ///
    /// # Safety
    /// 1. 调用方必须已将该页从缓存索引中摘除并阻止新任务获取，且已彻底结束该页的所有块设备 I/O、回写（`writeback == 0`）及用户态页表映射（含跨核 TLB 同步）；
    /// 2. 该页承载的数据必须属于未发布的临时候选页或已被文件系统协议正式删除的数据，确实不再需要落盘持久化。
    pub unsafe fn discard(&mut self) {
        assert!(self.locked, "discard without a content guard");
        let mut usage = self.folio.descriptor.usage();
        assert!(
            usage.folio().flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0,
            "discard lost content lock"
        );
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(shared.writeback, 0, "discard retains writeback");
        let old_dirty = shared.dirty;
        // 当前守卫独占持有 `CONTENT_LOCKED`，保证不存在并发的 `FolioRead` 初始化操作；
        // 重置页面初始化状态并清空脏扇区与已准备块位图。
        match &mut shared.contents {
            FolioContents::File { initialized, .. } => *initialized = None,
            FolioContents::Metadata { valid, errors } => {
                *valid = 0;
                errors.fill(None);
            }
        }
        shared.dirty = 0;
        shared.file_prepared = 0;
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(true);
    }

    /// 在文件系统释放元数据磁盘块时，精确作废元数据页内 `[offset, offset + len)` 扇区窗口的有效位、错误记录与脏位，同时保留同页其余扇区的状态。
    ///
    /// # Safety
    /// 1. 对应磁盘块必须已从文件系统块树中正式解除引用并禁止新任务通过旧映射获取，且该窗口上的所有缓冲区借用与在途回写（`writeback & mask == 0`）均已结束；
    /// 2. 必须在将物理块号归还给磁盘空闲位图之前调用本函数清除脏位，且不得用于丢弃已向用户承诺持久化的有效元数据修改。
    pub(crate) unsafe fn forget_metadata(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }
        let mask = sector_mask(offset, len);
        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        assert_eq!(
            shared.writeback & mask,
            0,
            "forget retains metadata writeback"
        );
        let FolioContents::Metadata { valid, errors } = &mut shared.contents else {
            return Err(BlockError::InvalidRange);
        };
        *valid &= !mask;
        errors[offset / SECTOR_SIZE..(offset + len) / SECTOR_SIZE].fill(None);
        let old_dirty = shared.dirty;
        shared.dirty &= !mask;
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        Ok(())
    }

    /// 校验当前页面为 `Metadata` 角色，且区间 `[offset, offset + len)` 已初始化就绪且未处于回写状态。
    fn metadata_range(&self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let usage = self.folio.descriptor.usage();
        let shared = FolioState::shared(&usage);
        if !matches!(&shared.contents, FolioContents::Metadata { .. }) {
            return Err(BlockError::InvalidRange);
        }
        if shared.writeback & sector_mask(offset, len) != 0 {
            return Err(BlockError::Busy);
        }
        FolioState::range_ready(shared, offset, len)
    }

    /// 在持有内容锁期间，将页内已初始化区间 `[offset, offset + len)` 的脏扇区（`dirty`）原子转移到在途回写位图（`writeback`），并生成 `FolioWriteback` 凭证。
    ///
    /// - 若目标区间内没有任何脏扇区（`dirty & mask == 0`），则返回 `Ok(None)`。
    /// - 若目标区间与已有在途回写重叠（`writeback & mask != 0`），则返回 `Err(BlockError::Busy)`。
    ///
    /// # Safety
    /// 1. 若为文件页，调用方在调用前必须已撤销该页所有用户态共享可写页表映射的写权限（mkclean）并完成跨核 TLB 同步，防止回写期间用户态静默改写正在 DMA 的内存却不置脏位；
    /// 2. 返回的 `FolioWriteback` 凭证一旦提交给块设备，必须在硬件 DMA 传输彻底结束后才能调用 `complete`。
    pub(crate) unsafe fn begin_writeback(
        &mut self,
        offset: usize,
        len: usize,
    ) -> Result<Option<FolioWriteback>, BlockError> {
        validate_range(offset, len)?;
        if self.folio.is_memory() {
            return Err(BlockError::Unsupported);
        }
        if offset % SECTOR_SIZE != 0 || len % SECTOR_SIZE != 0 {
            return Err(BlockError::Misaligned);
        }

        let mask = sector_mask(offset, len);
        let folio = self.folio.clone();
        let original_dirty;
        {
            let mut usage = self.folio.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            if shared.writeback & mask != 0 {
                return Err(BlockError::Busy);
            }
            if shared.dirty & mask == 0 {
                return Ok(None);
            }
            FolioState::range_ready(shared, offset, len)?;
            original_dirty = shared.dirty & mask;
            let old_dirty = shared.dirty;
            let old_wb = shared.writeback;
            shared.writeback |= mask;
            shared.dirty &= !mask;
            self.folio.account_state(old_dirty, old_wb, shared);
        }

        background::note_writeback_start();
        if let Some(FolioOwner::File(inode)) = &self.folio.owner {
            inode
                .disk_volume()
                .expect("disk writeback folio has no volume")
                .writeback_domain()
                .note_writeback_start();
        }
        Ok(Some(FolioWriteback {
            folio,
            offset: offset as u16,
            len: len as u16,
            completed: false,
            original_dirty,
            writeback_domain: None,
            background: false,
        }))
    }

    /// 在持有内容锁期间，将页内已初始化区间 `[offset, offset + len)` 覆盖的扇区标记为脏（`dirty`），并更新回写域统计。
    pub(crate) fn mark_dirty(&mut self, offset: usize, len: usize) -> Result<(), BlockError> {
        validate_range(offset, len)?;
        let mut usage = self.folio.descriptor.usage();
        let shared = FolioState::shared_mut(&mut usage);
        FolioState::range_ready(shared, offset, len)?;
        if matches!(
            &shared.contents,
            FolioContents::File {
                memory_charge: Some(_),
                ..
            }
        ) {
            return Ok(());
        }
        let mask = sector_mask(offset, len);
        if matches!(&shared.contents, FolioContents::Metadata { .. })
            && shared.writeback & mask != 0
        {
            return Err(BlockError::Busy);
        }
        let old_dirty = shared.dirty;
        shared.dirty |= mask;
        self.folio
            .account_state(old_dirty, shared.writeback, shared);
        drop(usage);

        self.folio.notify_dirty_state(false);
        Ok(())
    }
}

impl Drop for FolioContent<'_> {
    /// 释放 `FolioContent` 守卫：若当前守卫仍持有 `CONTENT_LOCKED` 锁责任，则清除锁标志位并在自旋锁外唤醒等待队列上的所有任务。
    fn drop(&mut self) {
        if !self.locked {
            return;
        }
        let state = self.folio.state();
        {
            let usage = self.folio.descriptor.usage();
            assert!(
                usage.folio().flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0,
                "folio content lock lost"
            );
            state.flags.fetch_and(!CONTENT_LOCKED, Ordering::Release);
        }

        state.waiters.wake_all();
    }
}

impl FolioWriteback {
    /// 为回写凭证绑定所属磁盘卷的 `Arc<WritebackDomain>` 统计域，并标记是否为后台回写任务发起的I/O。
    pub(crate) fn with_writeback_domain(
        mut self,
        domain: &Arc<WritebackDomain>,
        background: bool,
    ) -> Self {
        if !matches!(&self.folio.owner, Some(FolioOwner::File(_))) {
            domain.note_writeback_start();
        }
        self.writeback_domain = Some(domain.clone());
        self.background = background;
        self
    }

    /// 返回正在回写的底层 4KiB 物理页起始物理地址（仅用于构建块设备 DMA 映射）。
    pub(crate) fn phys_addr(&self) -> usize {
        self.folio.phys_addr()
    }

    /// 返回本轮回写区间在页内的起始字节偏移（单位：字节）。
    pub(crate) fn offset(&self) -> usize {
        usize::from(self.offset)
    }

    /// 返回本轮回写区间的字节长度（单位：字节）。
    pub(crate) fn len(&self) -> usize {
        usize::from(self.len)
    }

    /// 检查当前回写凭证对应的页面是否为元数据页（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) fn is_metadata(&self) -> bool {
        matches!(
            &FolioState::shared(&self.folio.descriptor.usage()).contents,
            FolioContents::Metadata { .. }
        )
    }

    /// 检查当前回写凭证是否已完成结算。
    pub(crate) fn is_completed(&self) -> bool {
        self.completed
    }

    /// 在块设备回写 DMA 彻底结束后结算本轮回写：记录可能发生的写回错误、清除 `writeback` 位、恢复未受理尾部的脏位并更新卷级统计。
    ///
    /// # Safety
    /// 1. 块设备对本凭证范围内物理页的全部 DMA 读取必须已通过硬件完成中断（used ring）或确认的控制器重置彻底结束；
    /// 2. `accepted` 必须为块设备层实际受理提交的连续前缀字节数（按 512B 扇区对齐），`result` 为该前缀的实际传输结果；
    /// 3. 严禁仅凭软件层面的超时或取消而未确认硬件停止访问时调用此函数。
    pub(crate) unsafe fn complete(&mut self, accepted: usize, result: IoResult) {
        assert!(accepted <= self.len() && result.completed <= accepted);
        assert_eq!(accepted % SECTOR_SIZE, 0, "unaligned writeback acceptance");
        if !self.completed {
            let error = result
                .error
                .or_else(|| (result.completed < accepted).then_some(BlockError::ShortIo));
            if 0 < accepted {
                if let Some(error) = error {
                    match &self.folio.owner {
                        Some(FolioOwner::File(inode)) => {
                            inode.mapping().record_wb_error(error.into())
                        }
                        Some(FolioOwner::Metadata(cache)) => cache.record_wb_error(error.into()),
                        None => {}
                    }
                }
            }
            self.finish(accepted);
            if 0 < accepted {
                background::note_completion(accepted, result.completed);
                if let Some(FolioOwner::File(inode)) = &self.folio.owner {
                    inode
                        .disk_volume()
                        .expect("disk writeback completion has no volume")
                        .writeback_domain()
                        .note_completion(accepted, result.completed, error.map(Into::into));
                } else if let Some(domain) = &self.writeback_domain {
                    domain.note_completion(accepted, result.completed, error.map(Into::into));
                }
                if self.background {
                    self.writeback_domain
                        .as_ref()
                        .expect("background writeback has no domain")
                        .note_round_io(accepted, result.completed);
                }
            }
        }
    }

    /// 在 `PageDesc` 自旋锁内清除本轮回写的 `writeback` 位图，并将未被设备受理的尾部区间（`unaccepted`）恢复原始脏位，随后在锁外唤醒等待者。
    fn finish(&mut self, accepted: usize) {
        let mask = sector_mask(self.offset(), self.len());
        let unaccepted = if accepted < self.len() {
            sector_mask(self.offset() + accepted, self.len() - accepted)
        } else {
            0
        };
        let state = self.folio.state();
        {
            let mut usage = self.folio.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            assert_eq!(shared.writeback & mask, mask, "folio writeback range lost");
            let old_dirty = shared.dirty;
            let old_wb = shared.writeback;
            shared.dirty |= self.original_dirty & unaccepted;
            shared.writeback &= !mask;
            self.folio.account_state(old_dirty, old_wb, shared);
            self.completed = true;
        }

        state.waiters.wake_all();
        self.folio.notify_dirty_state(accepted == 0);
    }
}

impl Drop for FolioWriteback {
    /// 若 `FolioWriteback` 在提交设备前被丢弃（`!self.completed`），自动将全部原始脏扇区恢复到页面的 `dirty` 位图中并清除 `writeback` 位。
    fn drop(&mut self) {
        if !self.completed {
            self.finish(0);
        }
    }
}

impl FolioRead {
    /// 在提交块设备 DMA 前，为文件读入凭证绑定所属 `InodeRef` 强引用，确保整个 DMA 读入及中断完成期间 Inode 保持存活。
    pub(crate) fn with_file_owner(mut self, owner: InodeRef) -> Self {
        self.folio.attach_file_owner(owner);
        self
    }

    /// 在提交块设备 DMA 前，为元数据读入凭证绑定所属 `Arc<MetadataCache>` 强引用，确保整个 DMA 读入期间元数据缓存保持存活。
    pub(crate) fn with_metadata_owner(mut self, owner: Arc<MetadataCache>) -> Self {
        self.folio.attach_metadata_owner(owner);
        self
    }

    /// 返回当前读入凭证锁定的底层 4KiB 物理页起始物理地址（用于构建块设备 DMA 映射）。
    pub fn phys_addr(&self) -> usize {
        self.folio.phys_addr()
    }

    /// 返回本次读入负责填充的页内起始偏移（单位：字节）。
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// 返回本次读入负责填充的非零字节长度（单位：字节）。
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.len
    }

    /// 检查本次读入凭证是否已发布结果并释放了内容锁。
    pub(crate) fn is_completed(&self) -> bool {
        self.completed
    }

    /// 检查当前读入凭证对应的页面是否为文件页（仅用于内核测试）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_is_file(&self) -> bool {
        matches!(
            &FolioState::shared(&self.folio.descriptor.usage()).contents,
            FolioContents::File { .. }
        )
    }

    /// 在通过 CPU 同步填充页面时，获取本次读入区间 `[offset, offset + len)` 的未初始化可变字节切片 `&mut [MaybeUninit<u8>]`。
    ///
    /// 当 `FolioRead` 移交给块设备执行异步 DMA 期间，禁止调用此方法创建 CPU 端可变切片，以免与设备 DMA 写入产生数据竞争。
    pub fn bytes_mut(&mut self) -> &mut [MaybeUninit<u8>] {
        assert!(
            !self.completed,
            "completed folio read cannot write contents"
        );
        let address = arch::phys_to_virt(self.phys_addr()) + self.offset;
        // SAFETY:
        // 1. `self` 独占持有页面的 `CONTENT_LOCKED` 锁且 `!self.completed`，该区间尚未对外发布为有效，因此不存在用户态映射或并发的 `FolioContent` 访问；
        // 2. 对于元数据页，`begin_metadata_read` 仅认领未初始化的连续扇区，与同页其他已有效扇区不重叠；
        // 3. 使用 `MaybeUninit<u8>` 表达未初始化内存，返回切片的生命周期受 `&mut self` 约束且不越出 4KiB 页边界。
        unsafe { slice::from_raw_parts_mut(address as *mut MaybeUninit<u8>, self.len) }
    }

    /// 在块设备 DMA 读入或 CPU 填充彻底完成后，发布初始化结果、释放 `CONTENT_LOCKED` 并唤醒等待队列上的所有任务。
    ///
    /// 若重复调用则直接忽略，保持首次发布的终态不变。
    ///
    /// # Safety
    /// 1. 针对本凭证区间 `[offset, offset + len)` 的全部块设备 DMA 写入或 CPU 填充写入必须已彻底完成，且此后不再向该初始化区间写入；
    /// 2. 当 `result.error.is_none()` 且 `result.completed == self.len` 时，调用方必须保证区间内的全部 `self.len` 个字节均已完成有效初始化。
    pub unsafe fn complete(&mut self, result: IoResult) {
        if self.completed {
            return;
        }
        let initialized = match result.error {
            Some(error) => Err(error),
            None if result.completed == self.len => Ok(InitializedRange {
                offset: self.offset as u16,
                len: self.len as u16,
            }),
            None => Err(BlockError::ShortIo),
        };
        self.publish(initialized);
    }

    /// 在 `PageDesc` 自旋锁内写入文件页或元数据扇区的初始化结果，释放 `CONTENT_LOCKED` 标志，并在自旋锁外唤醒等待队列。
    fn publish(&mut self, initialized: Result<InitializedRange, BlockError>) {
        let state = self.folio.state();
        {
            let mut usage = self.folio.descriptor.usage();
            let shared = FolioState::shared_mut(&mut usage);
            assert!(
                state.flags.load(Ordering::Relaxed) & CONTENT_LOCKED != 0,
                "folio read lost its content lock"
            );
            match &mut shared.contents {
                FolioContents::File {
                    initialized: result,
                    ..
                } => {
                    assert!(result.is_none(), "folio result already published");
                    *result = Some(initialized);
                }
                FolioContents::Metadata { valid, errors } => {
                    let sectors = sector_mask(self.offset, self.len);
                    assert_eq!(
                        *valid & sectors,
                        0,
                        "metadata read overwrites valid contents"
                    );
                    for stored_error in &mut errors
                        [self.offset / SECTOR_SIZE..(self.offset + self.len) / SECTOR_SIZE]
                    {
                        assert!(
                            stored_error.is_none(),
                            "metadata read overwrites failed contents"
                        );
                        if let Err(error) = initialized {
                            *stored_error = Some(error);
                        }
                    }
                    if initialized.is_ok() {
                        *valid |= sectors;
                    }
                }
            }
            self.completed = true;
            state.flags.fetch_and(!CONTENT_LOCKED, Ordering::Release);
        }

        state.waiters.wake_all();
    }
}

impl Drop for FolioRead {
    /// 若 `FolioRead` 在完成前被提前丢弃（`!self.completed`），自动以 `Err(BlockError::QueueStopped)` 发布失败结果并释放内容锁唤醒等待者。
    fn drop(&mut self) {
        if !self.completed {
            self.publish(Err(BlockError::QueueStopped));
        }
    }
}

/// 校验页内非空字节区间 `[offset, offset + len)` 是否合法（要求 `len > 0` 且不发生算术溢出、不超出 `PAGE_SIZE`）。
fn validate_range(offset: usize, len: usize) -> Result<(), BlockError> {
    if len == 0 || offset.checked_add(len).is_none_or(|end| PAGE_SIZE < end) {
        return Err(BlockError::InvalidRange);
    }
    Ok(())
}

/// 校验拷贝操作的页内字节区间 `[offset, offset + len)`：允许 `len == 0` 且 `offset <= PAGE_SIZE` 的零长边界请求，非空请求同 `validate_range`。
fn validate_copy_range(offset: usize, len: usize) -> Result<(), BlockError> {
    if len == 0 {
        return if offset <= PAGE_SIZE {
            Ok(())
        } else {
            Err(BlockError::InvalidRange)
        };
    }
    validate_range(offset, len)
}

/// 将经过合法性校验的非空页内字节区间 `[offset, offset + len)` 转换为覆盖的 512 字节扇区 8 位掩码。
fn sector_mask(offset: usize, len: usize) -> u8 {
    let first = offset / SECTOR_SIZE;
    let last = (offset + len - 1) / SECTOR_SIZE;
    (((1_u16 << (last - first + 1)) - 1) << first) as u8
}
