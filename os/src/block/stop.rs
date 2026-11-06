//! 块设备安全停机（Stop / Reset）与文件系统扇区范围占用（`FsClaim`）管理。
//!
//! ## 模块职责与架构定位
//! 本模块位于块设备子系统的生命周期控制路径，负责两项紧密相关的核心职责：
//! 1. **文件系统范围占用仲裁（`FsClaim`）**：上游缓存层（`crate::cache`）与文件系统层在挂载或
//!    访问块视图（`BlockView`）前，通过 [`Disk::claim_fs`] 登记扇区区间与读写模式；与并发的裸设备
//!    写入（Raw Write）及其他挂载点在同一物理磁盘的 `queue` 自旋锁下互斥仲裁。当最后一个 `FsClaim`
//!    引用被释放（Drop）时，自动通知元数据页缓存（`MetadataPages`）失效不再被任何占用覆盖的扇区。
//! 2. **设备停机与硬件复位（`Disk::stop` / `Disk::fault_reset`）**：按严格顺序排空在途请求生成者、
//!    等待门铃通知与后端命令执行退出、发起并轮询硬件复位（Reset）、执行 DMA 屏障，最后同步注销
//!    中断（IRQ）与关闭工作队列任务（Work）。若硬件复位超时未确认，则将队列转入隔离状态
//!    （`Quarantined`），保留底层设备与在途 DMA 缓冲区所有权，防止硬件 DMA 踩坏已回收内存。
//!
//! ## 核心状态机：停机与故障复位流程
//! ```text
//!   Prepared ──────────────────────────────┐
//!                                          ▼
//!   Live ──(stop: mounts == 0)──► Draining ──(generators/slots/notify == 0)──► Resetting
//!    │                               │                                           │
//!    └──(设备故障)──────────────► Faulting ─────────(fault_reset 认领)───────────┘
//!                                                                                │
//!                                     ┌────────(2s 超时未确认 reset_complete)────┤
//!                                     ▼                                          ▼
//!                                Quarantined                                DmaStopped
//!                        (永久保留 DMA 页面与设备，                        (DMA 屏障已生效，
//!                         拒绝回收以防 DMA 踩内存)                          清理在途 Submitted 槽，
//!                                                                           同步注销 IRQ 与 Work)
//!                                                                                │
//!                                                                                ▼
//!                                                                             Stopped
//! ```
//!
//! ## 锁获取顺序（Lock Ordering）与并发契约
//! 1. `Disk::control`（`SleepMutex`）：串行化外部任务发起的 `start()` 与 `stop()` 操作；进度工作任务
//!    （`progress_work`）在执行 `fault_reset()` 时**不获取** `control` 锁，避免与正在等待队列排空的
//!    `stop()` 死锁。
//! 2. `Disk::queue`（`SpinLock<QueueState>`）：保护队列阶段（`QueuePhase`）、在途槽位、裸写链表
//!    （`raw_writers`）与文件系统占用链表（`claim_head`）。持锁期间严禁睡眠。
//! 3. `MetadataPages` 内部索引锁与缓冲页状态锁：在 `FsClaimInner::drop` 中，保持持有 `Disk::queue`
//!    自旋锁的同时调用 `MetadataPages::retire_range`（锁顺序：`Disk::queue` ➔ `MetadataPages` 内部自旋锁），
//!    确保在扇区缓存失效完成前，不会有新的裸写入或新挂载并发准入该扇区范围。
//! 4. `Disk::irq`（`SpinLock`）：仅在脱离 `Disk::queue` 锁后获取以取出 `IrqRegistration`，随后在锁外
//!    调用 `unregister_sync()` 阻塞等待中断处理函数彻底退出。

use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::ptr;
use core::time::Duration;

use crate::arch;
use crate::block::disk::{Disk, DiskRef, QueuePhase, QueueState};
use crate::block::{AccessMode, BlockError, BlockView, SECTOR_BYTES};
use crate::cache::MetadataPages;
use crate::wait::{WaitNode, Waiter};
use crate::{sched, time, timer};

/// 等待硬件复位完成的最大超时时间（2 秒）。
const RESET_TIMEOUT: Duration = Duration::from_secs(2);
/// 轮询硬件复位完成状态的睡眠间隔（1 毫秒）。
const RESET_POLL_INTERVAL: Duration = Duration::from_millis(1);

/// 文件系统对块设备扇区范围的占用凭证。
///
/// 多个 `FsClaim` 克隆实例共享同一个底层 [`FsClaimInner`] 引用计数，克隆不会增加物理磁盘上的
/// `mounts` 占用计数。当最后一个 `FsClaim` 被释放（Drop）时，会自动失效无其他占用覆盖的元数据缓存页并摘除占用记录。
#[derive(Clone)]
pub struct FsClaim {
    /// 引用计数的内部占用节点，保活至最后一个克隆体以及关联的所有子请求槽位释放完毕。
    inner: Arc<FsClaimInner>,
}

/// 文件系统扇区范围占用的内部状态节点（侵入式单向链表节点）。
pub(super) struct FsClaimInner {
    /// 本次占用绑定的块设备扇区视图（包含目标磁盘引用、起始扇区与扇区总数）。
    view: BlockView,
    /// 本次占用的访问模式（只读 `ReadOnly` 或读写 `ReadWrite`）。
    mode: AccessMode,
    /// 标记该节点是否已成功挂入物理磁盘的 `claim_head` 链表并增加了 `mounts` 计数。
    claimed: bool,
    /// 指向同一物理磁盘上下一个活跃 `FsClaimInner` 的链表指针，由 `view.disk().queue` 自旋锁保护。
    next: UnsafeCell<*const FsClaimInner>,
}

// SAFETY:
// 1. `view` 与 `mode` 在构造后只读；
// 2. `next` 裸指针仅在持有 `view.disk().queue` 自旋锁时读写，且节点在从链表摘除前始终由 `Arc<FsClaimInner>` 保活。
unsafe impl Send for FsClaimInner {}
// SAFETY:
// 1. 公开方法仅读取不可变的 `view` 和 `mode`；
// 2. 侵入式链表指针 `next` 的并发修改完全由同一把物理磁盘 `queue` 自旋锁串行化。
unsafe impl Sync for FsClaimInner {}

impl FsClaim {
    /// 返回本次文件系统占用的访问模式（`ReadOnly` 或 `ReadWrite`）。
    pub fn mode(&self) -> AccessMode {
        self.inner.mode
    }

    /// 返回本次占用所属的物理块设备引用。
    pub fn disk(&self) -> &DiskRef {
        self.inner.view.disk()
    }

    /// 返回本次占用覆盖的块设备扇区视图（[`BlockView`]）。
    pub fn view(&self) -> &BlockView {
        &self.inner.view
    }
}

impl Disk {
    /// 在普通可睡眠任务上下文中串行停止块设备。
    ///
    /// # 调用契约与返回值
    /// - 必须在可调度的普通任务上下文中调用，严禁在本设备的完成回调、中断处理函数或进度 `Work` 中调用（否则等待自身退出将导致死锁）。
    /// - 若磁盘上仍存在活跃的文件系统占用（`mounts != 0`），返回 `Err(BlockError::Busy)`。
    /// - 若硬件复位超时未能确认 DMA 停止，队列转入 `Quarantined` 状态并返回 `Err(BlockError::StopUnconfirmed)`，此时底层设备与 DMA 内存保持原状不予释放。
    /// - 若停机前设备已发生硬件故障但成功完成复位与清理，最终返回 `Err(BlockError::DeviceFault)`。
    pub fn stop(&self) -> Result<(), BlockError> {
        sched::assert_switchable();
        // Step 1: 获取 `control` 睡眠锁串行化外部控制操作，并在 `queue` 自旋锁内检查挂载计数与切换阶段。
        let _control = self.control.lock();
        {
            let mut queue = self.queue.lock();
            if queue.phase == QueuePhase::Quarantined {
                return Err(BlockError::StopUnconfirmed);
            }
            if queue.mounts != 0 {
                return Err(BlockError::Busy);
            }
            match queue.phase {
                QueuePhase::Stopped => return stop_result(&queue),
                QueuePhase::Quarantined => return Err(BlockError::StopUnconfirmed),
                QueuePhase::Prepared => queue.phase = QueuePhase::DmaStopped,
                QueuePhase::Live => queue.phase = QueuePhase::Draining,
                _ => {}
            }
        }
        // Step 2: 唤醒等待槽位的任务并刷出待发送通知，循环等待已接纳请求排空并执行硬件复位。
        self.available.wake_all();
        self.unplug();
        loop {
            self.wait_quiescent_until(|queue| {
                queue.phase != QueuePhase::Draining || accepted_finished(queue)
            });
            let reset = {
                let mut queue = self.queue.lock();
                match queue.phase {
                    QueuePhase::Draining => {
                        assert!(accepted_finished(&queue));
                        queue.phase = QueuePhase::Resetting;
                        queue.pending_notify = false;
                        true
                    }
                    QueuePhase::Quarantined => return Err(BlockError::StopUnconfirmed),
                    QueuePhase::DmaStopped => break,
                    QueuePhase::Faulting | QueuePhase::Resetting => false,
                    _ => panic!("invalid block stop phase"),
                }
            };
            if reset {
                self.reset_claimed()?;
            } else {
                self.fault_reset()?;
            }
        }
        // Step 3: 硬件 DMA 已确认停止，等待全部软件完成回调与通知彻底退出，然后同步注销 IRQ 与关闭 Work。
        self.wait_quiescent_until(accepted_finished);
        let registration = { self.irq.lock().take() };
        if let Some(registration) = registration {
            registration.unregister_sync();
        }
        if let Some(work) = &self.progress_work {
            work.as_ref().close_sync();
        }
        for slot in &self.slots {
            if let Some(work) = &slot.completion_work {
                work.as_ref().close_sync();
            }
        }

        // Step 4: 释放底层后端设备控制器，将队列阶段标记为最终的 `Stopped` 并唤醒所有等待者。
        let device = { self.queue.lock().device.take() };
        drop(device);
        let result = {
            let mut queue = self.queue.lock();
            assert_eq!(queue.phase, QueuePhase::DmaStopped);
            assert!(accepted_finished(&queue));
            queue.phase = QueuePhase::Stopped;
            stop_result(&queue)
        };
        self.quiescent.wake_all();
        result
    }

    /// 在设备发生故障（`Faulting`）时抢占并执行硬件复位，并在 DMA 停止后将所有已提交（`Submitted`）请求置为失败。
    ///
    /// 此函数通过 `QueuePhase` 状态转换保证只有一个执行流认领 `Resetting` 阶段，且不获取外部 `control` 睡眠锁，
    /// 因此既可由进度工作任务（`progress_work`）调用，也可由外部 `stop()` 调用。IRQ 注销与 `Work` 关闭仍由外部 `stop()` 负责。
    pub(super) fn fault_reset(&self) -> Result<(), BlockError> {
        sched::assert_switchable();
        loop {
            let reset = {
                let mut queue = self.queue.lock();
                match queue.phase {
                    QueuePhase::Faulting => {
                        queue.phase = QueuePhase::Resetting;
                        queue.pending_notify = false;
                        true
                    }
                    QueuePhase::Resetting => false,
                    QueuePhase::DmaStopped => {
                        drop(queue);
                        self.fail_submitted_after_reset(BlockError::DeviceFault);
                        return Ok(());
                    }
                    QueuePhase::Quarantined => return Err(BlockError::StopUnconfirmed),
                    QueuePhase::Prepared
                    | QueuePhase::Live
                    | QueuePhase::Draining
                    | QueuePhase::Stopped => return Ok(()),
                }
            };
            if reset {
                self.reset_claimed()?;
                self.fail_submitted_after_reset(BlockError::DeviceFault);
                return Ok(());
            }
            self.wait_quiescent_until(|queue| queue.phase != QueuePhase::Resetting);
        }
    }

    /// 从中断处理函数或任务上下文报告设备故障，立即关闭新请求准入并调度预分配的进度 `Work` 异步执行复位（不在当前上下文睡眠）。
    pub(super) fn request_retire(&self, error: BlockError) {
        self.mark_fault(error);
    }

    /// 由唯一将队列置为 `Resetting` 阶段的认领者调用：等待门铃通知和正在执行的后端命令退出后，发起并轮询硬件复位。
    ///
    /// 在硬件状态寄存器确认复位完成且执行完 `arch::dma_barrier()` 之前，绝不回收任何在途物理页；若超时则将队列转入 `Quarantined`。
    fn reset_claimed(&self) -> Result<(), BlockError> {
        // Step 1: 等待所有活跃的门铃通知（`notify_active`）和正在同步执行后端命令的槽位（`Executing`）全部退出。
        self.wait_quiescent_until(|queue| {
            queue.phase == QueuePhase::Quarantined
                || (queue.notify_active == 0
                    && !queue
                        .slot_states
                        .iter()
                        .any(|state| *state == crate::block::disk::SlotState::Executing))
        });
        // Step 2: 向后端硬件发出复位请求（如向 VirtIO 状态寄存器写 0）。
        {
            let mut queue = self.queue.lock();
            if queue.phase == QueuePhase::Quarantined {
                return Err(BlockError::StopUnconfirmed);
            }
            assert_eq!(queue.phase, QueuePhase::Resetting);
            queue
                .device
                .as_mut()
                .expect("resetting disk has no device")
                .request_reset();
            #[cfg(feature = "ktest")]
            {
                queue.ktest_stop.reset_requests += 1;
            }
        }
        // Step 3: 在 `RESET_TIMEOUT`（2 秒）期限内轮询硬件复位是否完成；若超时则转入 `Quarantined` 隔离阶段。
        let deadline = time::now()
            .checked_add(RESET_TIMEOUT)
            .expect("block reset deadline overflows monotonic time");
        loop {
            if deadline <= time::now() {
                {
                    let mut queue = self.queue.lock();
                    assert_eq!(queue.phase, QueuePhase::Resetting);
                    queue.phase = QueuePhase::Quarantined;
                    queue.failure = Some(BlockError::StopUnconfirmed);
                }
                self.available.wake_all();
                self.quiescent.wake_all();
                return Err(BlockError::StopUnconfirmed);
            }
            let stopped = {
                let mut queue = self.queue.lock();
                assert_eq!(queue.phase, QueuePhase::Resetting);
                // 测试钩子：当 `withhold_confirmation` 置位时模拟硬件迟迟不返回复位确认，以测试超时隔离路径。
                #[cfg(feature = "ktest")]
                let withheld = queue.ktest_stop.withhold_confirmation;
                #[cfg(not(feature = "ktest"))]
                let withheld = false;
                let stopped = !withheld
                    && queue
                        .device
                        .as_mut()
                        .expect("resetting disk has no device")
                        .reset_complete();
                if stopped {
                    arch::dma_barrier();
                    queue.phase = QueuePhase::DmaStopped;
                    #[cfg(feature = "ktest")]
                    {
                        queue.ktest_stop.reset_confirmations += 1;
                    }
                }
                stopped
            };
            if stopped {
                self.available.wake_all();
                self.quiescent.wake_all();
                return Ok(());
            }
            timer::sleep(RESET_POLL_INTERVAL);
        }
    }

    /// 在 `quiescent` 等待队列上睡眠，直到谓词 `done` 对 `QueueState` 返回 `true`。
    ///
    /// 采用标准的“注册栈上等待节点 ➔ 加锁复查条件 ➔ 锁外 `park()` 睡眠”模式，防止唤醒丢失。
    fn wait_quiescent_until(&self, done: impl Fn(&QueueState) -> bool) {
        if done(&self.queue.lock()) {
            return;
        }
        loop {
            let waiter = Waiter::new();
            let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `self.quiescent` 属于当前借用的 `Disk`，在本次函数调用期间始终有效；
            // 2. `node` 固定（Pin）在当前栈帧上，且返回的 `registration` 守卫在本轮循环退出前必定执行 `drop` 将节点从等待队列摘除，不会悬垂。
            let registration = unsafe { self.quiescent.register(node.as_mut()) };
            if done(&self.queue.lock()) {
                return;
            }
            waiter.park();
            drop(registration);
            drop(waiter);
            if done(&self.queue.lock()) {
                return;
            }
        }
    }

    /// 申请对指定块视图（`view`）和访问模式（`mode`）的文件系统扇区占用凭证（[`FsClaim`]）。
    ///
    /// # 仲裁规则
    /// - 若请求 `ReadWrite` 模式但底层设备为只读，返回 `Err(BlockError::ReadOnly)`。
    /// - 在同一物理磁盘的 `queue` 自旋锁下检查：
    ///   1. 队列必须处于 `QueuePhase::Live` 阶段；
    ///   2. 目标扇区范围不得与任何在途的裸设备写请求（`raw_writers` 或 `slot_writes`）重叠；
    ///   3. 允许对完全相同的扇区范围并发申请占用（供上层单飞合并同一卷的挂载候选），但拒绝与已有占用存在**部分重叠且任意一方为可写**的冲突请求，冲突时返回 `Err(BlockError::Busy)`。
    pub fn claim_fs(view: &BlockView, mode: AccessMode) -> Result<FsClaim, BlockError> {
        let disk = view.disk();
        if mode == AccessMode::ReadWrite && disk.limits.readonly {
            return Err(BlockError::ReadOnly);
        }

        let mut owner = Arc::try_new(FsClaimInner {
            view: view.clone(),
            mode,
            claimed: false,
            next: UnsafeCell::new(ptr::null()),
        })
        .map_err(|_| BlockError::NoMemory)?;

        {
            let inner = Arc::get_mut(&mut owner).expect("unpublished FS claim is shared");
            let mut queue = disk.queue.lock();
            let span = (view.start_sector(), view.start_sector() + view.sectors());
            if queue.phase != QueuePhase::Live
                || raw_write_conflicts(&queue, span)
                || claim_conflicts(&queue, view, mode)
            {
                return Err(BlockError::Busy);
            }
            // SAFETY:
            // 1. `owner` 刚刚分配且尚未对外发布，`Arc::get_mut` 证明当前拥有唯一可变访问权；
            // 2. 当前持有 `disk.queue` 自旋锁，串行化对 `claim_head` 链表的所有读写；
            // 3. `inner.claimed` 置为 `true` 后，节点在 `FsClaimInner::drop` 持同一把锁将其从链表摘除前始终由 `Arc` 保活。
            unsafe {
                *inner.next.get() = queue.claim_head;
            }
            queue.claim_head = inner;
            queue.mounts += 1;
            inner.claimed = true;
        }
        Ok(FsClaim { inner: owner })
    }
}

impl FsClaimInner {
    /// 在持有物理磁盘 `queue` 自旋锁的前提下，扫描 `self.view` 覆盖的扇区区间，跳过仍被其他活跃 `FsClaim`
    /// 覆盖的子区间，仅将本 `FsClaim` 独占负责的扇区段从共享元数据缓存（`pages`）中淘汰失效。
    fn retire_metadata(&self, queue: &QueueState, pages: &MetadataPages) {
        let mut cursor = self.view.start_sector();
        let end = cursor + self.view.sectors();
        while cursor < end {
            let mut covered_end = cursor;
            let mut next_start = end;
            let mut node = queue.claim_head;
            while !node.is_null() {
                // SAFETY:
                // 1. 调用方持有 `queue` 自旋锁；
                // 2. 链表中的所有 `FsClaimInner` 节点在从链表摘除前都需要在 `Drop::drop` 中获取同一把 `queue` 锁，因此当前指针指向的节点必定存活且有效。
                let claim = unsafe { &*node };
                if !ptr::eq(claim, self) {
                    let start = claim.view.start_sector();
                    let last = start + claim.view.sectors();
                    if start <= cursor && cursor < last {
                        covered_end = covered_end.max(last.min(end));
                    } else if cursor < start {
                        next_start = next_start.min(start);
                    }
                }
                // SAFETY: 当前持有 `queue` 自旋锁，保护链表节点的 `next` 指针读取。
                node = unsafe { *claim.next.get() };
            }
            if cursor < covered_end {
                cursor = covered_end;
            } else {
                // SAFETY:
                // 1. 上述链表扫描已确认 `[cursor, next_start)` 扇区区间不再被任何其他活跃 `FsClaim` 覆盖；
                // 2. 当前正在执行本 `FsClaimInner` 的 `Drop`（最后一个强引用已释放），表明关联的所有请求槽位、元数据读入与回写均已结束；
                // 3. 当前仍持有 `queue` 自旋锁且尚未将 `self` 从 `claim_head` 摘除，因此在 `retire_range` 完成前不会有冲突的裸写入或新 `FsClaim` 准入该区间。
                unsafe { pages.retire_range(cursor, next_start) };
                cursor = next_start;
            }
        }
    }
}

impl Drop for FsClaimInner {
    /// 当最后一个强引用（`Arc<FsClaimInner>`）被释放时，在 `queue` 自旋锁保护下失效不再被其他占用覆盖的元数据缓存范围，
    /// 将自身从 `claim_head` 链表摘除并递减 `mounts` 计数，最后在锁外唤醒等待队列。
    fn drop(&mut self) {
        if !self.claimed {
            return;
        }
        let pages = self.view.disk().metadata_cache();
        {
            let mut queue = self.view.disk().queue.lock();
            assert!(0 < queue.mounts, "block mount claim underflow");
            // 先在持有 `queue` 锁时失效本占用独占的元数据缓存扇区，随后再从链表摘除以维持互斥。
            if let Some(pages) = &pages {
                self.retire_metadata(&queue, pages);
            }
            let mut link = &mut queue.claim_head as *mut *const FsClaimInner;
            // SAFETY:
            // 1. 当前持有 `queue` 自旋锁，独占访问 `claim_head` 及整条链表的 `next` 指针；
            // 2. 链表中每个已登记节点在执行到此处的摘除逻辑前均未完成 `Drop`，内存保持有效。
            unsafe {
                while !(*link).is_null() {
                    if ptr::eq(*link, self) {
                        *link = *self.next.get();
                        break;
                    }
                    link = (*(*link)).next.get();
                }
            }
            queue.mounts -= 1;
        }
        self.view.disk().available.wake_all();
        self.view.disk().quiescent.wake_all();
    }
}

/// 判断两个左闭右开扇区区间 `[a.0, a.1)` 与 `[b.0, b.1)` 是否存在交集（紧邻但不重叠的区间返回 `false`）。
pub(super) fn overlaps(a: (u64, u64), b: (u64, u64)) -> bool {
    a.0 < b.1 && b.0 < a.1
}

/// 检查待准入的裸设备写入扇区区间 `span` 是否与任何已登记的文件系统占用（`claim_head`）重叠。
///
/// 调用方必须持有对应物理磁盘的 `queue` 自旋锁。
pub(super) fn mounted_write_conflicts(queue: &QueueState, span: (u64, u64)) -> bool {
    let mut node = queue.claim_head;
    while !node.is_null() {
        // SAFETY: 调用方持有 `queue` 自旋锁，链表节点在持同一把锁摘除前由各自的 `Arc<FsClaimInner>` 保活。
        let claim = unsafe { &*node };
        if overlaps(
            span,
            (
                claim.view.start_sector(),
                claim.view.start_sector() + claim.view.sectors(),
            ),
        ) {
            return true;
        }
        // SAFETY: 调用方持有 `queue` 自旋锁，保护 `next` 链表指针读取。
        node = unsafe { *claim.next.get() };
    }
    false
}

/// 检查待申请的文件系统占用 `(view, mode)` 是否与已有占用存在部分重叠的写冲突。
///
/// 完全相同的扇区范围允许并发申请（供上层归并共享同一挂载视图），但范围不同且存在交集时，只要任一方为 `ReadWrite` 即视为冲突。
fn claim_conflicts(queue: &QueueState, view: &BlockView, mode: AccessMode) -> bool {
    let mut node = queue.claim_head;
    while !node.is_null() {
        // SAFETY: 调用方持有 `queue` 自旋锁，链表节点在持同一把锁摘除前由各自的 `Arc<FsClaimInner>` 保活。
        let claim = unsafe { &*node };
        if !view.same_range(&claim.view)
            && (mode == AccessMode::ReadWrite || claim.mode == AccessMode::ReadWrite)
            && overlaps(
                (view.start_sector(), view.start_sector() + view.sectors()),
                (
                    claim.view.start_sector(),
                    claim.view.start_sector() + claim.view.sectors(),
                ),
            )
        {
            return true;
        }
        // SAFETY: 调用方持有 `queue` 自旋锁，保护 `next` 链表指针读取。
        node = unsafe { *claim.next.get() };
    }
    false
}

/// 检查目标扇区区间 `span` 是否与任何在途的裸设备写请求生成者（`raw_writers`）或已提交的裸写请求槽位（`slot_writes`）重叠。
fn raw_write_conflicts(queue: &QueueState, span: (u64, u64)) -> bool {
    let mut node = queue.raw_writers;
    while !node.is_null() {
        // SAFETY: 调用方持有 `queue` 自旋锁，`raw_writers` 链表中的 `BioInner` 节点由已准入的请求生成者保活，直至在持同一把锁的 `finish_generation_tail` 中摘除。
        let bio = unsafe { &*node };
        if overlaps(
            span,
            (
                bio.sector(),
                bio.sector() + (bio.len() / SECTOR_BYTES as usize) as u64,
            ),
        ) {
            return true;
        }
        // SAFETY: 调用方持有 `queue` 自旋锁，保护 `raw_writer_next` 链表指针读取。
        node = unsafe { *bio.raw_writer_next.get() };
    }
    queue
        .slot_writes
        .iter()
        .flatten()
        .any(|other| overlaps(span, *other))
}

/// 判断所有已准入的请求生成者（含父 Bio 完成回调）、活跃请求槽位以及门铃通知是否均已彻底执行完毕并退出。
fn accepted_finished(queue: &QueueState) -> bool {
    queue.generators == 0 && queue.active_slots == 0 && queue.notify_active == 0
}

/// 根据队列是否记录过故障错误，返回停机操作的最终结果。
fn stop_result(queue: &QueueState) -> Result<(), BlockError> {
    if queue.failure.is_some() {
        Err(BlockError::DeviceFault)
    } else {
        Ok(())
    }
}
