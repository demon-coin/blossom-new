//! 逻辑块请求（`Bio`）的状态机、物理页 DMA 所有权流转与父子请求完成聚合。
//!
//! # 模块职责与架构定位
//! `Bio` 代表一次上层（文件系统、页缓存读入/写回或直接块读写）发起的连续扇区逻辑 I/O 请求。
//! 它持有本次 I/O 涉及的全部物理页映射（[`BioMemory`]），在提交到 [`Disk`] 时可由拆分器按硬件队列限额
//! 拆分为 1 至多个连续的硬件子请求（child `RequestPart`）。提交线程（生成器）与所有已登记的子请求共同承担
//! 引用计数式的完成义务（`remaining`），仅在最后一项义务结束时一次性收回物理页 DMA 访问权、执行 `EndIo` 回调并唤醒等待者。
//!
//! # 核心状态机（`Phase`）与义务计数流转
//! ```text
//!                      ┌─── reject() (未准入即失败，物理页未移交设备) ───┐
//!                      │                                                 ▼
//!   try_new() ──► [ Phase::New ] ──► begin() ──► [ Phase::Beginning ]    │
//!                                      │  移交物理页 DMA/CPU 访问权      │
//!                                      │  remaining = 1 (生成器义务)     │
//!                                      ▼                                 │
//!                              [ Phase::Active ]                         │
//!                                │  register_child(): remaining += 1     │
//!                                │  accept_child(): 记录硬件实际接收范围 │
//!                                │  finish_child():   remaining -= 1     │
//!                                │  finish_generation():                 │
//!                                │    generating = false, remaining -= 1 │
//!                                ▼  (当 remaining == 0)                  │
//!                           [ Phase::Finishing ] ◄───────────────────────┘
//!                                │  1. 收回物理页 DMA/CPU 访问权
//!                                │  2. 执行 EndIo 回调 (Inline 或 Work)
//!                                │  3. 更新 Folio 读入/写回状态
//!                                ▼
//!                           [ Phase::Complete ] (ready = true, 唤醒等待队列)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `Disk.queue`（`SpinLock`）──► `BioInner.state`（`SpinLock`）：在 `accept_child` 等路径中允许先持物理队列锁再获取 `BioInner.state`；禁止反向获取。
//! - `BioInner.raw_writer_next` 裸链表指针不经过 `BioInner.state`，仅在所属目标 `Disk.queue` 自旋锁保护下读写。
//! - 持 `BioInner.state` 自旋锁期间严禁执行 DMA 缓存同步、`EndIo` 回调、页框状态更新（`complete_folio`）或释放 `FsClaim` / `DiskRef`；这些重操作必须先将数据移出锁外执行。

use alloc::sync::Arc;
use core::pin::{pin, Pin};
use core::sync::atomic::{AtomicBool, Ordering};

use core::cell::UnsafeCell;

use crate::block::{BioAllocator, BioMemory};
use crate::block::{
    BlkOperation, BlockError, Disk, DiskIdentity, DiskRef, EndIo, FsClaim, IoResult,
};
use crate::fs::background::WritebackDomain;
use crate::mm::dma::{DmaDomain, MappedPages, PayloadRange};
use crate::sched;
use crate::sync::SpinLock;
use crate::wait::{WaitNode, WaitQueue, Waiter};
use crate::workqueue::Work;

/// 逻辑块 I/O 请求的唯一提交句柄（不可克隆，消费后移交队列避免重复提交）。
pub struct Bio {
    /// 指向底层共享控制块与自定义分配器的强引用。
    pub(super) inner: BioArc,
}

/// `Bio` 的只读完成观察句柄，允许多个调用方克隆并等待结果，不持有 `Disk` 强引用且无法重复提交请求。
#[derive(Clone)]
pub struct BioCompletion {
    /// 与原 `Bio` 共享同一个底层控制块，用于查询结果、睡眠等待或单次取回物理页容器。
    inner: BioArc,
}

/// `Bio` 生命周期内的内部阶段状态机（受 `BioInner.state` 自旋锁保护）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Phase {
    /// 刚构造完成、尚未进入物理队列准入，物理页访问权仍在 CPU 手中。
    New,
    /// 正在由唯一准入任务将物理页访问权移交给 DMA 设备（或内存盘 CPU 拷贝通道）。
    Beginning,
    /// 已完成准入，正在由生成器拆分登记子请求并由硬件异步执行。
    Active,
    /// 生成器与全部已登记子请求均已结束（或请求在准入前被拒绝），正在锁外收回物理页访问权并执行完成回调。
    Finishing,
    /// 物理页回收、回调与 `Folio` 结算全部完成，最终结果已发布且等待队列已唤醒。
    Complete,
}

/// `BioInner` 中受自旋锁保护的可变账本状态。
struct BioState {
    /// 当前所处的生命周期阶段。
    phase: Phase,
    /// 本次请求携带的物理页映射集合；在 `Beginning`（移交访问权期间）、`Finishing`（收回与回调期间）或被调用方 `take_memory` 领走后为 `None`。
    memory: Option<BioMemory>,
    /// 尚未完成的义务总数（包含 1 份生成器拆分义务 + 已登记但未完成的硬件子请求数）；归零时触发唯一完成流程。
    remaining: usize,
    /// 生成器是否仍在继续拆分并登记子请求。
    generating: bool,
    /// 已按逻辑顺序向本 `Bio` 登记拆分的连续字节末尾偏移（单位：字节）。
    registered_end: usize,
    /// 已登记的子请求总个数（用于校验 `Flush` 单次生成与非空提交）。
    registered_children: usize,
    /// 底层驱动 `submit` 已实际接收的连续字节末尾偏移（单位：字节）。
    accepted_end: usize,
    /// 所有失败子请求或生成错误中偏移最小的一处 `(失败起始字节偏移, 错误码)`，用于计算连续成功前缀。
    first_error: Option<(usize, BlockError)>,
    /// 请求进入 `Finishing` / `Complete` 后固化保存的最终完成结果。
    result: Option<IoResult>,
    /// 当非槽位路径（如生成器最后结束）将 `EndIo::Task` 异步移交给 `finalizer_work` 时，临时持有的 `DiskRef` 以推迟归还队列生成器计数。
    completion_owner: Option<DiskRef>,
    /// 文件系统提交路径绑定的挂载占用凭证，延续到父请求及所有子请求脱离队列后再释放。
    fs_claim: Option<FsClaim>,
    /// 派发 `finalizer_work` 前暂存的 `BioArc` 裸强引用票据，由工作队列回调取走并恢复为 `BioArc`。
    finalizer_ticket: Option<BioRawTicket>,
    /// 文件系统后台写回路径绑定的脏页流量统计域。
    writeback_domain: Option<Arc<WritebackDomain>>,
}

/// 逻辑块请求的共享核心控制块，分离构造期固定的不可变寻址参数与自旋锁保护的动态完成账本。
pub(super) struct BioInner {
    /// 构造时绑定的目标物理块设备唯一身份标识（不持有 `Disk` 强引用以避免循环引用）。
    target: DiskIdentity,
    /// 目标设备的 DMA 域参数；内存盘等纯 CPU 拷贝后端为 `None`。
    domain: Option<DmaDomain>,
    /// 直接块写请求在目标 `Disk.queue` 活跃写链表中的侵入式后继指针（仅在目标 `Disk.queue` 锁保护下访问）。
    pub(super) raw_writer_next: UnsafeCell<*const BioInner>,
    /// 请求的块操作类型（读、写或刷盘）。
    operation: BlkOperation,
    /// 请求在物理整盘上的起始扇区号（单位：512 字节 `SECTOR_BYTES`）。
    sector: u64,
    /// 请求负载的总字节长度（`Flush` 操作为 0）。
    len: usize,
    /// 构造时传入的物理页映射段（`MappedPages`）数量。
    segment_count: usize,
    /// 可选的请求完成异步回调。
    end_io: Option<EndIo>,
    /// 当携带 `EndIo::Task` 时在构造期预分配的独立工作项，用于处理被拒绝或由生成器最后结束时的可睡眠回调。
    finalizer_work: Option<Pin<Arc<Work>>>,
    /// 保护动态阶段、子请求计数与物理页容器的自旋锁。
    state: SpinLock<BioState>,
    /// 标记请求是否已彻底进入 `Phase::Complete` 的原子标志（写入用 `Release`，观察用 `Acquire`）。
    ready: AtomicBool,
    /// 供 `BioCompletion::wait` 阻塞等待完成通知的任务等待队列。
    waiters: WaitQueue,
}

// SAFETY:
// 1. `raw_writer_next` 仅在目标 `Disk.queue` 自旋锁临界区内读写，且在途生成器持有 `BioArc` 保证链表节点存活；
// 2. 其余可变字段全部受 `state` 自旋锁保护，物理页 DMA/CPU 访问权在最后一个子请求与生成器全部结束后才移交回 CPU。
unsafe impl Send for BioInner {}
// SAFETY:
// 1. 跨线程共享 `&BioInner` 不会在无锁状态下暴露底层物理页内容；
// 2. `raw_writer_next` 裸指针由同一个目标 `Disk.queue` 自旋锁串行化，其余内部可变状态均由 `state`、`ready` 与 `waiters` 同步原语保护。
unsafe impl Sync for BioInner {}

/// 携带预留池/堆分配器状态（[`BioAllocator`]）的 `BioInner` 强引用智能指针类型。
pub(super) type BioArc = Arc<BioInner, BioAllocator>;

/// 用于跨工作队列上下文无损传递一份 `BioArc` 强引用所有权及其底层分配器的裸票据。
struct BioRawTicket {
    /// 由 `Arc::into_raw_with_allocator` 转出的 `BioInner` 裸指针（持有一份未释放的强引用计数）。
    pointer: *const BioInner,
    /// 与该 `Arc` 实例绑定的原始分配器句柄。
    allocator: BioAllocator,
}

// SAFETY:
// 1. `pointer` 与 `allocator` 来自真实的 `BioArc` 强引用拆解，等价于跨线程移动一个 `BioArc`（`BioInner: Send + Sync` 且 `BioAllocator: Send`）；
// 2. 票据存入与取出均受 `BioInner.state` 自旋锁保护，仅由单次排队的 `finalizer_callback` 消费并还原为 `BioArc`。
unsafe impl Send for BioRawTicket {}

impl BioRawTicket {
    /// 拆解一份 `BioArc` 强引用，在不改变引用计数的前提下保存其裸指针与原始分配器。
    fn from_arc(owner: BioArc) -> Self {
        let (pointer, allocator) = Arc::into_raw_with_allocator(owner);
        Self { pointer, allocator }
    }

    /// 消费本票据，将其持有的强引用与分配器一次性还原为 `BioArc`。
    fn into_arc(self) -> BioArc {
        // SAFETY:
        // 1. `self.pointer` 与 `self.allocator` 由 `Self::from_arc` 调用 `Arc::into_raw_with_allocator` 配对产生；
        // 2. `into_arc` 按值消费 `self`，保证同一张票据只会被调用一次 `Arc::from_raw_in`，不会发生双重释放。
        unsafe { Arc::from_raw_in(self.pointer, self.allocator) }
    }
}

impl Bio {
    /// 在普通可睡眠任务上下文（未持有任何队列自旋锁）中校验映射范围，并预分配 `Bio` 控制块与所需的完成工作项。
    ///
    /// 若校验或内存预分配失败，本函数不会向设备提交任何请求，并会以当前准备错误代为结算各 `MappedPages` 关联的 `Folio` 状态。
    ///
    /// # Safety
    /// 1. 对于 `Write` 请求，输入映射覆盖的全部物理页字节必须已完成初始化；对于独占物理页来源，调用方持有的全部可变/裸指针别名必须在移交期间停止访问，直到请求完成收回访问权；
    /// 2. 对于共享文件页写回（Writeback）来源，调用方必须已完成脏页写保护清理并持有写回凭证，映射期间不允许导出可变内容切片；
    /// 3. 每个输入 `MappedPages` 当前必须处于 CPU 拥有状态且未结算，构造成功后其设备访问周期由本 `Bio` 接管；对于 `Read` 请求，短读或错误返回时未覆盖的尾部字节仍处于未初始化状态；
    /// 4. 若 `end_io` 回调内部调用 `MappedPages::bytes`，调用方必须自行保证对应独占映射全范围已初始化且无并发修改；两种回调均不得泄露切片引用或在返回后继续解引用底层地址；
    /// 5. `EndIo::Inline` 回调严禁睡眠、动态分配阻塞资源或递归调用本 `Disk` 的 `stop`；`EndIo::Task` 回调允许睡眠，但严禁等待本 `Bio` 自身完成或等待所属 `Disk` 停机，且目标 `Disk` 的共享工作队列必须保持存活至本 `Bio` 完成。
    pub(crate) unsafe fn try_new(
        disk: &Disk,
        operation: BlkOperation,
        sector: u64,
        mut memory: BioMemory,
        attributes: u32,
        end_io: Option<EndIo>,
    ) -> Result<Self, BlockError> {
        sched::assert_switchable();
        let prepared = (|| {
            // Step 1: 确认所有物理页映射当前归 CPU 所有且未完成结算，并校验队列范围、对齐及目标 DMA 域可达性。
            let mut len = 0_usize;
            for mapped in &memory {
                if mapped.is_device_owned() || mapped.is_folio_complete() {
                    return Err(BlockError::Busy);
                }
                len = len
                    .checked_add(mapped.len())
                    .ok_or(BlockError::InvalidRange)?;
            }
            disk.limits().validate(
                operation,
                sector,
                len,
                memory.iter().map(MappedPages::range),
                attributes,
            )?;
            if let Some(domain) = disk.domain {
                for mapped in &memory {
                    mapped.segment(domain).map_err(BlockError::from)?;
                }
            }

            // Step 2: 使用输入内存容器配套的控制块分配器构造 BioInner，避免在 I/O 提交与完成热路径上再次做可失败分配。
            let allocator = memory.allocator().control();
            let mut uninit = Arc::<BioInner, BioAllocator>::try_new_uninit_in(allocator)
                .map_err(|_| BlockError::NoMemory)?;
            Arc::get_mut(&mut uninit)
                .expect("new bio has another owner")
                .write(BioInner {
                    target: disk.identity(),
                    domain: disk.domain,
                    raw_writer_next: UnsafeCell::new(core::ptr::null()),
                    operation,
                    sector,
                    len,
                    segment_count: memory.len(),
                    end_io,
                    finalizer_work: None,
                    state: SpinLock::new(BioState {
                        phase: Phase::New,
                        memory: None,
                        remaining: 0,
                        generating: false,
                        registered_end: 0,
                        registered_children: 0,
                        accepted_end: 0,
                        first_error: None,
                        result: None,
                        completion_owner: None,
                        fs_claim: None,
                        finalizer_ticket: None,
                        writeback_domain: None,
                    }),
                    ready: AtomicBool::new(false),
                    waiters: WaitQueue::new(),
                });
            // SAFETY: `uninit` 是刚分配的唯一未公开 `Arc` 实例，上方 `write` 已完整初始化 `BioInner` 的全部字段。
            let inner = unsafe { uninit.assume_init() };
            // Step 3: 若携带可睡眠的 EndIo::Task 回调，提前在目标磁盘的工作队列域中预留专用 finalizer_work 与内核执行栈。
            if inner.needs_task() {
                let ticket = BioRawTicket::from_arc(inner);
                let raw = ticket.pointer;
                let owner = ticket.into_arc();
                let work = disk.prepare_finalizer(finalizer_callback, raw as usize)?;
                // SAFETY:
                // 1. `owner` 尚未对外返回，当前不存在任何其它线程或中断持有 `BioInner` 的引用；
                // 2. `work` 刚构造且尚未入队运行，`raw` 指向的 `finalizer_work` 字段此时由当前栈帧独占写入。
                unsafe { (*raw.cast_mut()).finalizer_work = Some(work) };
                return Ok(owner);
            }
            Ok(inner)
        })();

        match prepared {
            Ok(inner) => {
                inner.state.lock().memory = Some(memory);
                Ok(Self { inner })
            }
            Err(error) => {
                // Step 4 (失败清理): 请求尚未移交设备，直接在本地以准备失败错误结算所有输入映射关联的 Folio 读入/写回状态。
                for mapped in &mut memory {
                    if !mapped.is_device_owned() {
                        // SAFETY:
                        // 1. 上方 `!mapped.is_device_owned()` 确认物理页仍归 CPU 所有，且请求从未发布到硬件队列；
                        // 2. 调用方 `# Safety` 契约保证外部无冲突别名，传入 `completed: 0` 不会将未读入的数据页误标为已初始化。
                        unsafe {
                            mapped.complete_folio(IoResult {
                                completed: 0,
                                error: Some(error),
                            })
                        };
                    }
                }
                Err(error)
            }
        }
    }

    /// 创建一个指向同一底层请求的完成观察句柄（仅克隆 `Arc` 指针，不复制任何物理页负载）。
    pub fn completion(&self) -> BioCompletion {
        BioCompletion {
            inner: Arc::clone(&self.inner),
        }
    }
}

impl BioCompletion {
    /// 在普通可睡眠任务上下文中阻塞等待 `Bio` 完成，并返回最终 `IoResult`。
    ///
    /// 采用标准的“先检查完成态 -> 注册栈上等待节点 -> 二次复查完成态 -> 睡眠挂起”协议，防止错过唤醒信号。
    pub fn wait(&self) -> IoResult {
        sched::assert_switchable();
        loop {
            if let Some(result) = self.result() {
                return result;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self.inner` 持有 `BioInner` 的强引用，保证等待队列 `waiters` 在整个循环期间有效；
            // 2. `node` 通过 `pin!` 固定在当前栈帧上，`_registration` 守卫在每次循环迭代或提前 `return` 时都会先于 `node` 和 `waiter` 析构摘链，不会被 `mem::forget` 遗忘。
            let _registration = unsafe { self.inner.waiters.register(node.as_mut()) };
            if let Some(result) = self.result() {
                return result;
            }
            waiter.park();
        }
    }

    /// 非阻塞读取请求的最终完成结果；若请求尚未彻底完成则返回 `None`，多次调用或克隆句柄均可重复读取同一结果。
    pub fn result(&self) -> Option<IoResult> {
        if !self.is_complete() {
            return None;
        }
        self.inner.state.lock().result
    }

    /// 通过 `Acquire` 内存序检查请求是否已彻底完成（保证观察到物理页 DMA 访问权收回及 `EndIo` 回调的全部内存副作用）。
    pub fn is_complete(&self) -> bool {
        self.inner.ready.load(Ordering::Acquire)
    }

    /// 在请求完成后将原始物理页容器（[`BioMemory`]）移出给唯一调用者；若请求未完成或已被取走则返回 `Err(BlockError::Busy)`。
    ///
    /// 对于 `Read` 失败或短读场景，`IoResult::completed` 之后的物理页尾部字节仍可能未初始化，调用方仅能按实际完成前缀读取或通过 `MaybeUninit` 安全写入。
    pub(crate) fn take_memory(&self) -> Result<BioMemory, BlockError> {
        if !self.is_complete() {
            return Err(BlockError::Busy);
        }
        self.inner
            .state
            .lock()
            .memory
            .take()
            .ok_or(BlockError::Busy)
    }

    /// 测试辅助接口：降级获取底层控制块的弱引用（用于验证所有父子强引用及自定义分配器槽位是否已彻底释放）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_weak(&self) -> impl Send + Sync + 'static {
        Arc::downgrade(&self.inner)
    }

    /// 测试辅助接口：阻塞等待预分配的 `finalizer_work` 工作项彻底执行完毕并退出。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_flush_finalizer(&self) {
        if let Some(work) = &self.inner.finalizer_work {
            work.as_ref().flush();
        }
    }

    /// 测试辅助接口：读取当前已登记的子请求个数、已登记字节末尾与底层驱动已实际接收的字节末尾。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_acceptance(&self) -> (usize, usize, usize) {
        let state = self.inner.state.lock();
        (
            state.registered_children,
            state.registered_end,
            state.accepted_end,
        )
    }

    /// 测试辅助接口：获取 `finalizer_work` 预留的专用工作线程 `Task` 引用（若存在）。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_finalizer_task(&self) -> Option<Arc<crate::task::Task>> {
        self.inner
            .finalizer_work
            .as_ref()
            .and_then(|work| work.ktest_reserved_task())
    }

    /// 测试辅助接口：检查工作队列管理器是否已完成对 `finalizer_work` 预留线程的 `join` 回收。
    #[cfg(feature = "ktest")]
    pub(crate) fn ktest_finalizer_joined(&self) -> bool {
        self.inner
            .finalizer_work
            .as_ref()
            .is_some_and(|work| work.ktest_reserved_joined())
    }
}

impl BioInner {
    /// 返回构造时绑定的目标物理磁盘身份标识。
    pub(super) fn target(&self) -> DiskIdentity {
        self.target
    }

    /// 返回本请求的逻辑块操作类型（所有拆分出的子请求均沿用该操作）。
    pub(super) fn operation(&self) -> BlkOperation {
        self.operation
    }

    /// 返回本请求在物理整盘上的起始扇区号（单位：512 字节 `SECTOR_BYTES`）。
    pub(super) fn sector(&self) -> u64 {
        self.sector
    }

    /// 返回本请求的数据负载总字节数（`Flush` 请求为 0）。
    pub(super) fn len(&self) -> usize {
        self.len
    }

    /// 返回构造时传入的原始物理页映射段（`MappedPages`）个数。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "批量读取测试按原页数观察每个父 Bio 的准备和完成责任"
        )
    )]
    pub(super) fn segment_count(&self) -> usize {
        self.segment_count
    }

    /// 按下标复制第 `index` 个映射段的物理地址与长度元数据（`PayloadRange`），不借出底层物理页引用。
    pub(super) fn segment(&self, index: usize) -> Option<PayloadRange> {
        self.state
            .lock()
            .memory
            .as_ref()?
            .get(index)
            .map(MappedPages::range)
    }

    /// 供内存盘（RAM disk）后端在已登记子请求的执行期内，通过 CPU 在物理页负载与内存盘后备区间之间拷贝数据。
    ///
    /// 每次仅短暂获取 `state` 自旋锁以生成当前映射段的拷贝计划，实际字节拷贝在锁外执行。
    ///
    /// # Safety
    /// 1. 调用方必须持有对应子请求 `[offset, offset + length)` 的唯一未完成执行权，且在该子请求调用 `finish_child` 前不得提前结束 CPU 访问期；
    /// 2. `ram` 指针必须在 `[ram, ram + length)` 范围内有效、可读写（按读/写方向），由当前拷贝通道独占访问，且与本 `Bio` 的物理页负载内存无任何重叠。
    pub(super) unsafe fn copy_cpu(
        &self,
        offset: usize,
        length: usize,
        ram: *mut u8,
    ) -> Result<(), BlockError> {
        let mut index = 0;
        let mut base = 0;
        let mut copied = 0;
        while copied < length {
            let (plan, part_len, next_base) = {
                let state = self.state.lock();
                assert_eq!(state.phase, Phase::Active);
                let mapped = state
                    .memory
                    .as_ref()
                    .expect("CPU copy lost original memory")
                    .get(index)
                    .ok_or(BlockError::InvalidRange)?;
                let end = base + mapped.len();
                if end <= offset + copied {
                    index += 1;
                    base = end;
                    continue;
                }
                let within = offset + copied - base;
                let part_len = (mapped.len() - within).min(length - copied);
                // SAFETY:
                // 1. 当前子请求尚未完成（`state.phase == Phase::Active`），`memory` 底层物理页保持存活且 CPU 访问期尚未结束；
                // 2. 切片 `[within, within + part_len)` 完全落在当前子请求独占负责的范围内，释放自旋锁后拷贝过程不再借用 `MappedPages` 元数据。
                let plan =
                    unsafe { mapped.cpu_copy_range(within, part_len) }.map_err(BlockError::from)?;
                (plan, part_len, end)
            };
            // SAFETY:
            // 1. 调用方 `# Safety` 契约保证 `ram.add(copied)` 起 `part_len` 字节有效、独占且不与物理页负载重叠；
            // 2. 当前子请求完成前 `MappedPages` 维持 CPU 访问态，保证 `plan` 目标物理页有效且无并发别名冲突。
            unsafe {
                match self.operation {
                    BlkOperation::Read => plan.copy_from(ram.add(copied)),
                    BlkOperation::Write => plan.copy_to(ram.add(copied)),
                    BlkOperation::Flush => unreachable!("Flush has no CPU payload"),
                }
            }
            .map_err(BlockError::from)?;
            copied += part_len;
            base = next_base;
            index += 1;
        }
        Ok(())
    }

    /// 测试辅助接口：将 `Bio` 负载内的逻辑字节偏移 `offset` 映射到对应的底层 `Folio` 物理首址与页内偏移。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_folio_origin(&self, offset: usize) -> Option<(usize, usize)> {
        let state = self.state.lock();
        let mut base = 0;
        for mapped in state.memory.as_ref()? {
            let end = base + mapped.len();
            if base <= offset && offset < end {
                return Some((
                    mapped.ktest_folio_phys_addr()?,
                    mapped.offset() + offset - base,
                ));
            }
            base = end;
        }
        None
    }

    /// 测试辅助接口：查询负载字节偏移 `offset` 所在映射段是否为元数据写回（Metadata Writeback）来源，用于故障注入匹配。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_writeback_metadata(&self, offset: usize) -> Option<bool> {
        let state = self.state.lock();
        let mut base = 0;
        for mapped in state.memory.as_ref()? {
            let end = base + mapped.len();
            if base <= offset && offset < end {
                return mapped.ktest_writeback_metadata();
            }
            base = end;
        }
        None
    }

    /// 测试辅助接口：查询负载字节偏移 `offset` 所在映射段是否为文件页缓存读入（File Read）来源。
    #[cfg(feature = "ktest")]
    pub(super) fn ktest_file_read(&self, offset: usize) -> Option<bool> {
        let state = self.state.lock();
        let mut base = 0;
        for mapped in state.memory.as_ref()? {
            let end = base + mapped.len();
            if base <= offset && offset < end {
                return Some(mapped.ktest_file_read());
            }
            base = end;
        }
        None
    }

    /// 判断本请求是否注册了必须在可睡眠工作线程中执行的 `EndIo::Task` 回调。
    pub(super) fn needs_task(&self) -> bool {
        matches!(self.end_io, Some(EndIo::Task(_)))
    }

    /// 在请求进入队列准入前（`Phase::New`），关联文件系统后台写回流量统计域。
    pub(super) fn track_writeback(&self, domain: &Arc<WritebackDomain>) {
        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::New);
        assert!(state.writeback_domain.is_none());
        state.writeback_domain = Some(domain.clone());
    }

    /// 在文件系统请求通过队列准入后（`Phase::Active`）、登记任何子请求前，保存所属挂载占用凭证（`FsClaim`）。
    pub(super) fn retain_fs_claim(&self, claim: &FsClaim) {
        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Active);
        assert!(state.fs_claim.is_none());
        state.fs_claim = Some(claim.clone());
    }

    /// 为正在构造的硬件子请求（`RequestPart`）克隆一份挂载占用凭证，确保槽位执行期覆盖挂载生命周期。
    pub(super) fn fs_claim(&self) -> Option<FsClaim> {
        self.state.lock().fs_claim.clone()
    }

    /// 在普通可睡眠任务上下文中执行唯一准入：建立生成器初始义务，并在锁外将全部物理页的访问权移交给 DMA 设备（或内存盘 CPU 通道）。
    ///
    /// 若中途某个映射段的 DMA 准备或缓存维护失败，立即回滚前面已移交的映射段并将状态重置回 `Phase::New`。
    pub(super) fn begin(&self) -> Result<(), BlockError> {
        sched::assert_switchable();
        // Step 1: 在自旋锁内将阶段切为 Beginning、初始化 remaining = 1（生成器义务），并临时取出物理页容器。
        let mut memory = {
            let mut state = self.state.lock();
            assert_eq!(state.phase, Phase::New, "bio admitted twice");
            state.phase = Phase::Beginning;
            state.remaining = 1;
            state.generating = true;
            state
                .memory
                .take()
                .expect("bio memory missing at admission")
        };

        // Step 2: 在自旋锁外逐段执行 DMA 缓存刷写/失效或内存盘 CPU 访问登记；一旦出错则回滚已开启的前缀段。
        let mut started = 0;
        let mut failure = None;
        for mapped in &mut memory {
            // SAFETY:
            // 1. 当前独占处于 `Phase::Beginning`，尚未登记或发布任何硬件子请求；
            // 2. `Bio::try_new` 的构造契约已保证写数据全部初始化且外部 CPU 冲突别名已终止。
            let result = unsafe {
                match self.domain {
                    Some(domain) => mapped.begin_device_access(domain),
                    None => {
                        mapped.begin_cpu_access();
                        Ok(())
                    }
                }
            };
            if let Err(error) = result {
                failure = Some(BlockError::from(error));
                break;
            }
            started += 1;
        }
        if failure.is_some() {
            for mapped in &mut memory[..started] {
                // SAFETY: 请求仍处于 `Phase::Beginning`，尚未向任何硬件队列或工作线程暴露该映射段，可立即对称撤回访问权。
                unsafe {
                    if self.domain.is_some() {
                        mapped
                            .finish_device_access()
                            .expect("unpublished DMA ownership rollback failed");
                    } else {
                        mapped.finish_cpu_access();
                    }
                }
            }
        }

        // Step 3: 重新加锁归还物理页容器；失败时回退到 Phase::New，成功时切入 Phase::Active 允许拆分登记子请求。
        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Beginning);
        state.memory = Some(memory);
        if let Some(error) = failure {
            state.phase = Phase::New;
            state.remaining = 0;
            state.generating = false;
            return Err(error);
        }
        state.phase = Phase::Active;
        Ok(())
    }

    /// 在将硬件子请求写入设备可用环之前，按父请求内的连续字节顺序登记一个 `[offset, offset + length)` 子请求并递增未完成义务计数 `remaining`。
    pub(super) fn register_child(&self, offset: usize, length: usize) {
        let end = offset
            .checked_add(length)
            .expect("bio child range overflow");
        assert!(end <= self.len);
        assert!(0 < length || self.operation == BlkOperation::Flush);

        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Active);
        assert!(state.generating, "bio child registered after generation");
        assert_eq!(
            offset, state.registered_end,
            "bio child ranges are not contiguous"
        );
        if self.operation == BlkOperation::Flush {
            assert_eq!(state.registered_children, 0, "flush registered twice");
        }
        state.registered_end = end;
        state.registered_children = state
            .registered_children
            .checked_add(1)
            .expect("bio child count overflow");
        state.remaining = state
            .remaining
            .checked_add(1)
            .expect("bio pending count overflow");
    }

    /// 在底层驱动 `submit` 成功后（仍持有目标 `Disk.queue` 自旋锁，中断处理尚无法抢先认领完成），推进已实际提交给硬件的连续字节范围并累加写回统计量。
    pub(super) fn accept_child(&self, offset: usize, length: usize) {
        let end = offset.checked_add(length).expect("bio acceptance overflow");
        let mut state = self.state.lock();
        assert_eq!(state.phase, Phase::Active);
        assert!(state.generating && end <= state.registered_end);
        assert_eq!(
            offset, state.accepted_end,
            "bio acceptance is not contiguous"
        );
        let mut base = 0;
        for mapped in state.memory.as_mut().expect("active bio memory missing") {
            let mapped_end = base + mapped.len();
            let start = offset.max(base);
            let stop = end.min(mapped_end);
            if start < stop {
                mapped.accept_range(start - base, stop - start);
            }
            base = mapped_end;
        }
        state.accepted_end = end;
        if let Some(domain) = &state.writeback_domain {
            domain.note_submitted(length);
        }
    }

    /// 在硬件中断收回请求槽、确认控制器重置完成或子请求提交前即失败时，结算单个子请求 `[offset, offset + length)` 的完成字节与错误。
    ///
    /// 若本子请求是整个 `Bio` 最后一个未完成的义务（此时生成器早已结束），则直接在当前上下文（中断或槽位专属 `completion_work`）中触发父请求收尾。
    pub(super) fn finish_child(
        bio: &BioArc,
        offset: usize,
        length: usize,
        completed: usize,
        error: Option<BlockError>,
    ) {
        let end = offset
            .checked_add(length)
            .expect("bio child range overflow");
        assert!(end <= bio.len && completed <= length);
        let error = error.or_else(|| (completed < length).then_some(BlockError::ShortIo));
        let completion = {
            let mut state = bio.state.lock();
            assert_eq!(state.phase, Phase::Active);
            assert!(end <= state.registered_end);
            assert!(
                usize::from(state.generating) < state.remaining,
                "bio child ended twice"
            );
            if let Some(error) = error {
                state.record_error(offset + completed, error);
            }
            let mut base = 0;
            for mapped in state.memory.as_mut().expect("active bio memory missing") {
                let mapped_end = base + mapped.len();
                let start = offset.max(base);
                let stop = end.min(mapped_end);
                if start < stop {
                    let progress = (offset + completed).saturating_sub(start).min(stop - start);
                    let mapped_error = if progress < stop - start || completed == length {
                        error
                    } else {
                        None
                    };
                    mapped.record_writeback_result(
                        start - base,
                        stop - start,
                        progress,
                        mapped_error,
                    );
                }
                base = mapped_end;
            }
            state.end_obligation(bio.len)
        };

        if let Some((memory, result)) = completion {
            Self::complete(bio, memory, result, true, None, false);
        }
    }

    /// 在普通可睡眠任务上下文中结束生成器的子请求拆分义务；若所有已登记子请求此前均已完成，则由当前生成器负责触发父请求收尾。
    ///
    /// 返回 `true` 表示本请求携带可睡眠的 `EndIo::Task` 回调，且收尾工作与队列生成器计数归还责任已异步移交给预分配的 `finalizer_work`。
    pub(super) fn finish_generation(
        bio: &BioArc,
        error: Option<(usize, BlockError)>,
        owner: Option<&DiskRef>,
    ) -> bool {
        sched::assert_switchable();
        let completion = {
            let mut state = bio.state.lock();
            assert_eq!(state.phase, Phase::Active);
            assert!(state.generating, "bio generation ended twice");
            if let Some((offset, error)) = error {
                assert!(offset <= state.registered_end);
                state.record_error(offset, error);
            } else {
                assert_eq!(
                    state.registered_end, bio.len,
                    "bio generation is incomplete"
                );
                assert!(0 < state.registered_children, "bio generated no child");
            }
            state.generating = false;
            state.end_obligation(bio.len)
        };

        if let Some((memory, result)) = completion {
            return Self::complete(bio, memory, result, true, owner, true);
        }
        false
    }

    /// 在普通可睡眠任务上下文中直接拒绝尚未准入（仍处于 `Phase::New`，物理页从未移交设备）的 `Bio`，在锁外执行回调并唤醒等待者。
    pub(super) fn reject(bio: &BioArc, error: BlockError) {
        sched::assert_switchable();
        let memory = {
            let mut state = bio.state.lock();
            assert_eq!(state.phase, Phase::New, "rejecting an admitted bio");
            state.phase = Phase::Finishing;
            state.memory.take().expect("rejected bio memory missing")
        };
        Self::complete(
            bio,
            memory,
            IoResult {
                completed: 0,
                error: Some(error),
            },
            false,
            None,
            true,
        );
    }

    /// 当全部义务归零或准入前被拒绝时，统一在锁外收回物理页 DMA/CPU 访问权，并根据上下文立即完成或派发 `finalizer_work`。
    ///
    /// 参数 `device_owned` 指明物理页此前是否已通过 `begin` 移交给设备；参数 `queue_task` 指明遇到 `EndIo::Task` 时是否需要排入 `finalizer_work`
    /// （槽位完成路径已在槽位专属 `Work` 线程上运行，传 `false` 直接同步执行回调；生成器或拒绝路径传 `true` 避免阻塞当前提交批次）。
    fn complete(
        bio: &BioArc,
        mut memory: BioMemory,
        result: IoResult,
        device_owned: bool,
        owner: Option<&DiskRef>,
        queue_task: bool,
    ) -> bool {
        // Step 1: 若物理页访问权曾移交给设备/CPU 拷贝通道，在自旋锁外统一执行缓存失效与访问权回收。
        if device_owned {
            for mapped in &mut memory {
                // SAFETY:
                // 1. `end_obligation` 仅在 `remaining == 0` 且 `!generating` 时将 `memory` 唯一移出，不会与其他线程并发回收；
                // 2. 所有已提交硬件的子请求均已在收到完成中断或确认硬件重置停止 DMA 后调用 `finish_child`，设备不再访问这些物理页。
                unsafe {
                    if bio.domain.is_some() {
                        mapped
                            .finish_device_access()
                            .expect("completed DMA cache synchronization failed");
                    } else {
                        mapped.finish_cpu_access();
                    }
                };
            }
        }

        // Step 2: 若当前不在槽位专属 Work 中且请求携带可睡眠的 EndIo::Task 回调，打包裸强引用票据并排入预分配的 finalizer_work 异步收尾。
        if queue_task && bio.needs_task() {
            let ticket = BioRawTicket::from_arc(Arc::clone(bio));
            let owned = ticket.pointer;
            {
                let mut state = bio.state.lock();
                assert_eq!(state.phase, Phase::Finishing);
                assert!(
                    state.memory.is_none()
                        && state.result.is_none()
                        && state.completion_owner.is_none()
                );
                state.memory = Some(memory);
                state.result = Some(result);
                // 此时所有子请求均已从设备槽位移除，Disk 不再持有指向本 Bio 的反向强引用，暂存 DiskRef 不会形成引用环。
                state.completion_owner = owner.cloned();
                assert!(state.finalizer_ticket.is_none());
                state.finalizer_ticket = Some(ticket);
            }
            let work = bio
                .finalizer_work
                .as_ref()
                .expect("Task bio has no prepared finalizer");
            assert_eq!(owned as usize, work.context());
            assert!(
                work.as_ref()
                    .queue()
                    .expect("bio finalizer domain closed before producers stopped"),
                "bio finalizer queued twice"
            );
            return true;
        }

        // Step 3: 无需额外排队时，直接在当前上下文执行回调、结算 Folio 并发布完成状态。
        drop(bio.finalize(memory, result));
        false
    }

    /// 在自旋锁外依次执行 `EndIo` 完成回调、结算各物理页 `Folio` 的读入/写回结果，随后切入 `Phase::Complete` 唤醒所有等待任务并返回挂载占用凭证以便在锁外释放。
    fn finalize(&self, mut memory: BioMemory, result: IoResult) -> Option<FsClaim> {
        // Step 1: 物理页访问权已归还 CPU，调用注册的完成回调（只读借出结果与映射切片）。
        match self.end_io {
            Some(EndIo::Inline(callback)) => callback(&result, &memory),
            Some(EndIo::Task(callback)) => {
                // 子请求最后结束时运行在槽位预留的 completion_work 线程上；生成器/拒绝路径运行在 Bio 预留的 finalizer_work 线程上。
                sched::assert_switchable();
                callback(&result, &memory);
            }
            None => {}
        }

        // Step 2: 按父请求的连续成功字节前缀结算每个映射段关联的 Folio（唤醒页缓存读等待者或清除页写回标志并记录写回错误）。
        let mut remaining = result.completed;
        for mapped in &mut memory {
            let completed = remaining.min(mapped.len());
            remaining = remaining.saturating_sub(mapped.len());
            // SAFETY:
            // 1. `complete` 已收回全部物理页的设备访问权，且上方 `EndIo` 回调已返回、不再持有任何映射借用；
            // 2. 传入的 `completed` 严格截取自父请求的连续成功前缀，写回路径使用此前 `accept_child` 与 `finish_child` 登记的真实区间状态。
            unsafe {
                mapped.complete_folio(IoResult {
                    completed,
                    error: result.error,
                })
            };
        }

        // Step 3: 将物理页容器与最终结果存回状态锁，通过 Release 屏障设置 ready 标志并唤醒等待队列，最后将 FsClaim 移出锁外释放。
        let claim = {
            let mut state = self.state.lock();
            assert_eq!(state.phase, Phase::Finishing);
            assert!(state.memory.is_none() && state.result.is_none());
            state.memory = Some(memory);
            state.result = Some(result);
            state.phase = Phase::Complete;
            state.fs_claim.take()
        };
        self.ready.store(true, Ordering::Release);
        self.waiters.wake_all();
        claim
    }
}

/// `BioInner.finalizer_work` 的工作队列入口函数：在可睡眠工作线程中还原 `BioArc` 强引用，执行 `finalize` 收尾，并代生成器归还目标 `Disk` 的活跃生成计数与挂载占用。
fn finalizer_callback(work: Pin<&Work>) {
    let pointer = work.context() as *const BioInner;
    // SAFETY:
    // 1. `BioInner::complete` 在将本 `work` 入队前，已构造一份 `BioRawTicket`（持有一份未释放的 `BioArc` 强引用）存入 `state.finalizer_ticket`；
    // 2. 在下方加锁取走 `ticket` 并还原为 `BioArc` 之前，该强引用保证 `pointer` 指向的 `BioInner` 持续有效且对齐。
    let inner = unsafe { &*pointer };
    let (ticket, memory, result, owner) = {
        let mut state = inner.state.lock();
        assert_eq!(state.phase, Phase::Finishing);
        let ticket = state
            .finalizer_ticket
            .take()
            .expect("finalizer lost its raw ticket");
        assert_eq!(ticket.pointer, pointer);
        (
            ticket,
            state.memory.take().expect("finalizer memory missing"),
            state.result.take().expect("finalizer result missing"),
            state.completion_owner.take(),
        )
    };

    let bio = ticket.into_arc();
    let claim = bio.finalize(memory, result);
    if let Some(owner) = owner {
        owner.finish_generation_tail(&bio);
    }
    drop(claim);
}

impl BioState {
    /// 记录偏移最小的首个失败位置与错误码，确保后续乱序成功的子请求不会越过前部失败点夸大连续成功字节数。
    fn record_error(&mut self, offset: usize, error: BlockError) {
        if self.first_error.is_none_or(|(first, _)| offset < first) {
            self.first_error = Some((offset, error));
        }
    }

    /// 递减一项未完成义务计数 `remaining`；仅当计数归零（生成器与全部已登记子请求均已结束）时将阶段推进到 `Phase::Finishing`，并移出物理页容器与汇总后的 `IoResult` 交由锁外收尾。
    fn end_obligation(&mut self, len: usize) -> Option<(BioMemory, IoResult)> {
        assert!(0 < self.remaining, "bio obligation ended twice");
        self.remaining -= 1;
        if self.remaining != 0 {
            return None;
        }
        assert!(!self.generating);
        self.phase = Phase::Finishing;
        let result = match self.first_error {
            Some((completed, error)) => IoResult {
                completed,
                error: Some(error),
            },
            None => IoResult {
                completed: len,
                error: None,
            },
        };
        Some((
            self.memory.take().expect("completed bio memory missing"),
            result,
        ))
    }
}
