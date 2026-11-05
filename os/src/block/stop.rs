//! 设备停机先终止 DMA，再等待本设备的中断与工作执行尾部。

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

const RESET_TIMEOUT: Duration = Duration::from_secs(2);
const RESET_POLL_INTERVAL: Duration = Duration::from_millis(1);

/// 文件系统占用共享同一个真实磁盘及模式，克隆不增加队列占用数。
#[derive(Clone)]
pub struct FsClaim {
    inner: Arc<FsClaimInner>,
}

pub(super) struct FsClaimInner {
    view: BlockView,
    mode: AccessMode,
    claimed: bool,
    next: UnsafeCell<*const FsClaimInner>,
}

// SAFETY: next 只在 view.disk.queue 锁下访问，强引用覆盖发表到摘链的整个寿命。
unsafe impl Send for FsClaimInner {}
// SAFETY: 范围和模式发表后不变，裸链只由同一物理队列锁修改。
unsafe impl Sync for FsClaimInner {}

impl FsClaim {
    /// 借用本次占用的访问模式，不从设备可写能力推导视图权限。
    pub fn mode(&self) -> AccessMode {
        self.inner.mode
    }

    /// 借用实际磁盘拥有者，调用方不得以设备身份值伪造该占用。
    pub fn disk(&self) -> &DiskRef {
        self.inner.view.disk()
    }

    /// 借用本次真实范围，克隆仍共享同一个占用登记。
    pub fn view(&self) -> &BlockView {
        &self.inner.view
    }
}

impl Disk {
    /// 外部普通 Task 串行停止设备；不得由本设备的完成回调或进度 Work 调用。
    /// 活挂载返回 Busy，无法确认 reset 时保留原设备及全部 DMA 拥有者。
    pub fn stop(&self) -> Result<(), BlockError> {
        sched::assert_switchable();
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

    /// 由队列 phase 唯一认领故障 reset，不等待外部 stop 正持有的 control 锁。
    /// 进度回调只完成 DMA 停止与失败移交，IRQ 注销及 Work 关闭留给外部 stop。
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

    /// IRQ 或任务报告真实设备故障后闭门并排预建进度 Work，不在报告者中睡眠。
    pub(super) fn request_retire(&self, error: BlockError) {
        self.mark_fault(error);
    }

    /// 唯一 Resetting 认领者等通知真尾部，再请求并有限轮询真实 reset 完成。
    /// 状态读回零和完整设备屏障以前不收回任何在途页面。
    fn reset_claimed(&self) -> Result<(), BlockError> {
        self.wait_quiescent_until(|queue| {
            queue.phase == QueuePhase::Quarantined
                || (queue.notify_active == 0
                    && !queue
                        .slot_states
                        .iter()
                        .any(|state| *state == crate::block::disk::SlotState::Executing))
        });
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
                // 测试控制者只拒绝读取/承认确认；真实 reset 写入和单调期限不变。
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

    /// 登记固定栈等待节点后复查队列事实，所有 park 和通知均在队列锁外。
    fn wait_quiescent_until(&self, done: impl Fn(&QueueState) -> bool) {
        if done(&self.queue.lock()) {
            return;
        }
        loop {
            let waiter = Waiter::new();
            let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
            // SAFETY: self 保活永久源，栈节点固定到 registration 析构，登记不会被遗忘。
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

    /// 实际范围占用与已接受 raw 写、slot 写在同一物理队列锁下仲裁。
    /// 相同范围允许并发取得 canonical 候选，部分重叠的可写卷拒绝。
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
            // SAFETY: 本 inner 尚未发表，queue 锁串行链表，Arc 在 drop 摘链前保活节点。
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
    /// 在 queue 锁内扣除其他活占用，只退休本 claim 最后负责的 sector 段。
    fn retire_metadata(&self, queue: &QueueState, pages: &MetadataPages) {
        let mut cursor = self.view.start_sector();
        let end = cursor + self.view.sectors();
        while cursor < end {
            let mut covered_end = cursor;
            let mut next_start = end;
            let mut node = queue.claim_head;
            while !node.is_null() {
                // SAFETY: queue 锁保护链，本 claim 尚未摘离；其他节点同样由各自
                // Arc 的完整 Drop 保活到同锁摘离，即使其最后强引用也正在结束。
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
                // SAFETY: next 仅由同一 queue 锁访问，当前节点仍在链上。
                node = unsafe { *claim.next.get() };
            }
            if cursor < covered_end {
                cursor = covered_end;
            } else {
                // SAFETY: [cursor, next_start) 没有其他活 claim，本 claim 的最后
                // Drop 证明其 Buffer/Read/WB 已结束；仍持 queue 锁并保留链上占用，
                // 完成精确失效前 raw 写与新 claim 均不能重新准入。
                unsafe { pages.retire_range(cursor, next_start) };
                cursor = next_start;
            }
        }
    }
}

impl Drop for FsClaimInner {
    /// 最后责任先在 queue→index→用途锁下失效无其他占用的范围，再撤链并锁外通知。
    fn drop(&mut self) {
        if !self.claimed {
            return;
        }
        let pages = self.view.disk().metadata_cache();
        {
            let mut queue = self.view.disk().queue.lock();
            assert!(0 < queue.mounts, "block mount claim underflow");
            // 内容锁可能属于相邻卷的读入或 WB；失效只改用途锁内的范围状态。
            if let Some(pages) = &pages {
                self.retire_metadata(&queue, pages);
            }
            let mut link = &mut queue.claim_head as *mut *const FsClaimInner;
            // SAFETY: queue 锁保护完整链，所有发表节点在摘链前由自己的 Arc 保活。
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

/// 半开范围只在存在真实公共扇区时冲突，紧邻 scratch 可正常写入。
pub(super) fn overlaps(a: (u64, u64), b: (u64, u64)) -> bool {
    a.0 < b.1 && b.0 < a.1
}

/// raw 写准入不能绕过任何活 FS 范围，调用者持对应物理队列锁。
pub(super) fn mounted_write_conflicts(queue: &QueueState, span: (u64, u64)) -> bool {
    let mut node = queue.claim_head;
    while !node.is_null() {
        // SAFETY: queue 锁和 claim 的 Arc 协议保活当前节点，范围发表后不变。
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
        // SAFETY: 同一 queue 锁保护下一节点指针。
        node = unsafe { *claim.next.get() };
    }
    false
}

/// FS 候选允许相同范围的 canonical 竞争，部分重叠的写权限不能独立授予。
fn claim_conflicts(queue: &QueueState, view: &BlockView, mode: AccessMode) -> bool {
    let mut node = queue.claim_head;
    while !node.is_null() {
        // SAFETY: 调用者持物理队列锁，节点直到同锁摘链前保活。
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
        // SAFETY: 同一 queue 锁保护链接。
        node = unsafe { *claim.next.get() };
    }
    false
}

/// 已接受生成者与仍在回调尾的 slot 都阻止重叠范围新占用。
fn raw_write_conflicts(queue: &QueueState, span: (u64, u64)) -> bool {
    let mut node = queue.raw_writers;
    while !node.is_null() {
        // SAFETY: accepted 生成者保活节点，只有同一 queue 锁中的尾部才摘链。
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
        // SAFETY: raw_writer_next 只由本物理队列锁访问。
        node = unsafe { *bio.raw_writer_next.get() };
    }
    queue
        .slot_writes
        .iter()
        .flatten()
        .any(|other| overlaps(span, *other))
}

/// 生成者计数覆盖父完成回调，slot 计数覆盖每轮资源使用，通知计数覆盖真实门铃尾部。
fn accepted_finished(queue: &QueueState) -> bool {
    queue.generators == 0 && queue.active_slots == 0 && queue.notify_active == 0
}

/// 停机完成保留故障语义；重复 stop 不重复关闭 IRQ 或 Work。
fn stop_result(queue: &QueueState) -> Result<(), BlockError> {
    if queue.failure.is_some() {
        Err(BlockError::DeviceFault)
    } else {
        Ok(())
    }
}
