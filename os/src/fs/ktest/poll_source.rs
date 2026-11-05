//! 持续 poll 源的真实 Task/Completion 生命周期回归。

use alloc::sync::{Arc, Weak};
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use ::ktest::{TestContext, TestError, TestResult};

use crate::fs::poll::{PollMask, PollNotify, PollWaitHead};
use crate::sync::Completion;
use crate::{cpu, task};

struct RecordingSink {
    mask: AtomicU32,
    notifications: AtomicUsize,
    delivered: Completion,
}

impl RecordingSink {
    /// 建立消费源通知的固定观察者，不拥有源头或登记句柄。
    fn new() -> Self {
        Self {
            mask: AtomicU32::new(0),
            notifications: AtomicUsize::new(0),
            delivered: Completion::new(),
        }
    }
}

impl PollNotify for RecordingSink {
    /// 只发布本次通知的位和完成 token，不查询源状态或取得睡眠锁。
    fn notify(&self, mask: PollMask) {
        self.mask.fetch_or(mask.0, Ordering::Release);
        self.notifications.fetch_add(1, Ordering::Relaxed);
        self.delivered.complete();
    }
}

/// 持续源必须跨多轮通知消费、同步注销并释放最后节点拥有。
#[ktest::suite(
    context = "Thread",
    tags("fs-poll", "fs-poll-source"),
    timeout_ms = 15000
)]
pub(crate) mod source {
    use super::*;

    /// 同一登记接收前两轮通知，同步注销后第三轮不得再访问旧 sink。
    #[test]
    fn persistent_source_rounds_unregister_and_last_release(
        context: &mut TestContext<'_>,
    ) -> TestResult {
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
