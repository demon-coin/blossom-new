//! 持续型 Poll 事件源（`PollWaitHead`）的多轮唤醒、同步注销与生命周期回归测试。
//!
//! # 模块定位与职责
//! 本模块位于文件系统测试子系统（`fs::ktest`）中，负责验证 `crate::fs::poll::PollWaitHead` 与 `PollNotify` 接收端（Sink）在跨线程并发通知与动态注销场景下的内存安全与引用计数契约。
//! 上游由 `ktest` 测试运行器直接调度；下游调用 `PollWaitHead`、`PollNotify`、`Completion` 同步原语以及内核线程（`task::prepare_kernel_on`）接口。
//!
//! # 核心机制与交互时序
//! 持续型事件源在一次注册（`register`）后可跨越多轮事件广播，直到调用方主动销毁返回的注册句柄（`PollRegistration`）才从等待链表中摘除：
//!
//! ```text
//!   主测试线程 (Consumer / Owner)                后台通知线程 (Producer)
//!   ─────────────────────────────                ───────────────────────
//!   head.register(sink) ──► 挂入等待链表
//!                                                1. head.notify(POLLIN)
//!   等待 delivered & first  ◄───────────────────    first.complete()
//!   校验累计掩码: POLLIN (1 次)
//!   next_round.complete()   ───────────────────► 2. head.notify(POLLOUT)
//!   等待 second             ◄───────────────────    second.complete()
//!   校验累计掩码: POLLIN | POLLOUT (2 次)
//!   drop(registration)      ──► 同步从链表摘除
//!   release.complete()      ───────────────────► 3. head.notify(POLLHUP) (空操作)
//!   drop(sink) & drop(head)
//!   校验 sink_weak.upgrade().is_none() (零泄漏)
//! ```
//!
//! # 锁与同步契约
//! - `RecordingSink::notify` 在 `PollWaitHead` 内部自旋锁持锁或通知遍历期间被调用，因此仅执行无阻塞的原子位或（`AtomicU32::fetch_or`）、原子计数递增（`AtomicUsize::fetch_add`）以及 `Completion::complete()` 唤醒，不获取任何睡眠锁。
//! - 主线程与后台线程通过四个单向 `Completion` 门闩（`first`、`next_round`、`second`、`release`）严格定序，确保注销动作在第二轮通知完成后、第三轮通知发起前确定性生效。

use alloc::sync::{Arc, Weak};
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use ::ktest::{TestContext, TestError, TestResult};

use crate::fs::poll::{PollMask, PollNotify, PollWaitHead};
use crate::sync::Completion;
use crate::{cpu, task};

/// 记录型 Poll 通知接收器，用于统计收到的事件位掩码与通知次数。
struct RecordingSink {
    /// 累计收到的 Poll 事件位掩码（按位或累加，使用 `Release`/`Acquire` 内存序同步）。
    mask: AtomicU32,
    /// 累计收到 `notify` 回调的次数。
    notifications: AtomicUsize,
    /// 首次通知送达时触发的完成门闩，供测试主线程同步等待。
    delivered: Completion,
}

impl RecordingSink {
    /// 创建一个初始掩码与通知计数均为 0 的 `RecordingSink` 实例。
    fn new() -> Self {
        Self {
            mask: AtomicU32::new(0),
            notifications: AtomicUsize::new(0),
            delivered: Completion::new(),
        }
    }
}

impl PollNotify for RecordingSink {
    /// 接收来自 `PollWaitHead` 的事件通知，以原子方式累加事件掩码与计数并触发 `delivered` 完成信号。
    fn notify(&self, mask: PollMask) {
        self.mask.fetch_or(mask.0, Ordering::Release);
        self.notifications.fetch_add(1, Ordering::Relaxed);
        self.delivered.complete();
    }
}

/// `PollWaitHead` 持续通知与同步注销测试套件。
#[ktest::suite(
    context = "Thread",
    tags("fs-poll", "fs-poll-source"),
    timeout_ms = 15000
)]
pub(crate) mod source {
    use super::*;

    /// 测试目标：验证同一个 `PollRegistration` 可连续接收多轮事件通知，且在销毁注册句柄同步注销后，后续通知不再触达原接收器，最终接收器引用计数归零释放。
    /// 场景步骤：
    /// 1. 创建 `PollWaitHead` 与 `RecordingSink`，保存 `sink` 的 `Weak` 弱引用后将其注册到等待队列头。
    /// 2. 启动后台内核线程依次发送三轮通知（`POLLIN`、`POLLOUT`、`POLLHUP`），由多个 `Completion` 门闩控制节奏。
    /// 3. 主线程在前两轮通知后分别断言累计次数为 1、2 且掩码正确叠加；随后 `drop(registration)` 同步注销并放行第三轮 `POLLHUP` 通知。
    /// 4. 等待后台线程结束，验证第三轮通知未被 `sink` 收到，且在丢弃 `sink` 与 `head` 后 `Weak::upgrade()` 返回 `None`。
    #[test]
    fn persistent_source_rounds_unregister_and_last_release(
        context: &mut TestContext<'_>,
    ) -> TestResult {
        // Step 1: 分配 PollWaitHead、RecordingSink 及跨线程同步门闩，并将 sink 注册到等待头。
        let head = Arc::try_new(PollWaitHead::new())
            .map_err(|_| TestError("poll source head allocation failed"))?;
        let sink = Arc::try_new(RecordingSink::new())
            .map_err(|_| TestError("poll source sink allocation failed"))?;
        let sink_weak: Weak<RecordingSink> = Arc::downgrade(&sink);
        let sink_role: Arc<dyn PollNotify> = sink.clone();
        let registration = head
            .register(sink_role)
            .map_err(|_| TestError("poll source registration failed"))?;
        let first = Arc::try_new(Completion::new())
            .map_err(|_| TestError("poll source first gate allocation failed"))?;
        let next_round = Arc::try_new(Completion::new())
            .map_err(|_| TestError("poll source next-round gate allocation failed"))?;
        let second = Arc::try_new(Completion::new())
            .map_err(|_| TestError("poll source second gate allocation failed"))?;
        let release = Arc::try_new(Completion::new())
            .map_err(|_| TestError("poll source release gate allocation failed"))?;
        let task_head = Arc::clone(&head);
        let task_first = Arc::clone(&first);
        let task_release = Arc::clone(&release);
        let task_next_round = Arc::clone(&next_round);
        let task_second = Arc::clone(&second);

        // Step 2: 启动后台线程，按门闩时序分三轮触发 POLLIN、POLLOUT 和注销后的 POLLHUP。
        let join = task::prepare_kernel_on(cpu::current().id, move || {
            task_head.notify(PollMask::POLLIN);
            task_first.complete();
            task_next_round.wait();
            task_head.notify(PollMask::POLLOUT);
            task_second.complete();
            task_release.wait();
            task_head.notify(PollMask::POLLHUP);
            0
        })
        .map_err(|_| TestError("poll source task preparation failed"))?
        .publish();

        // Step 3: 等待并校验前两轮通知的掩码累加结果，随后销毁注册句柄完成同步注销。
        sink.delivered.wait();
        first.wait();
        let first_observed = (
            sink.notifications.load(Ordering::Acquire),
            sink.mask.load(Ordering::Acquire),
        );

        next_round.complete();
        second.wait();
        let second_observed = (
            sink.notifications.load(Ordering::Acquire),
            sink.mask.load(Ordering::Acquire),
        );

        drop(registration);
        // Step 4: 放行第三轮通知并汇合后台线程，验证注销后未再收到通知且最终对象引用计数彻底归零。
        release.complete();
        let result = join.join();
        context.expect_eq(result, 0);
        context.expect_eq(first_observed, (1, PollMask::POLLIN.0));
        context.expect_eq(
            second_observed,
            (2, (PollMask::POLLIN | PollMask::POLLOUT).0),
        );
        context.expect_eq(sink.notifications.load(Ordering::Acquire), 2);
        context.expect_eq(sink.mask.load(Ordering::Acquire), second_observed.1);

        drop(sink);
        drop(head);
        context.expect_eq(sink_weak.upgrade().is_none(), true);
        Ok(())
    }
}
