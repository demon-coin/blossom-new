//! 完成计数保存可消费 token；等待通知不替代 token 的 Acquire 消费。

use core::pin::pin;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::sched;
use crate::wait::{WaitNode, WaitQueue, WaitResult, Waiter};

/// 每次 complete 增加一个完成 token，每次成功等待只消费一个。
pub struct Completion {
    tokens: AtomicUsize,
    waiters: WaitQueue,
}

impl Completion {
    /// 建立没有完成 token 的同步对象，不为等待者分配存储。
    pub const fn new() -> Self {
        Self {
            tokens: AtomicUsize::new(0),
            waiters: WaitQueue::new(),
        }
    }

    /// Release 发布一个 token 后通知一名等待者；计数耗尽直接报告溢出，不丢弃完成。
    pub fn complete(&self) {
        self.tokens
            .fetch_update(Ordering::Release, Ordering::Relaxed, |tokens| {
                tokens.checked_add(1)
            })
            .expect("completion token count overflow");
        self.waiters.wake_one();
    }

    /// 非阻塞尝试消费一个 token，只有成功消费才 Acquire 对应完成数据。
    pub fn try_wait(&self) -> bool {
        self.tokens
            .fetch_update(Ordering::Acquire, Ordering::Relaxed, |tokens| {
                tokens.checked_sub(1)
            })
            .is_ok()
    }

    /// 普通任务在可切换环境中阻塞，只有消费到一个 token 才返回。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行与工作队列测试用真实阻塞 token 协调回调和清理尾部"
        )
    )]
    pub fn wait(&self) {
        sched::assert_switchable();
        if self.try_wait() {
            return;
        }

        loop {
            let consumed = {
                let waiter = Waiter::new();
                let mut node = pin!(WaitNode::new(waiter.waker()));
                // SAFETY: self 的共享借用保住源存储，pin! 保持栈节点地址稳定；registration 为唯一局部拥有者，
                // 从不被遗忘，作用域退出先析构它真实撤链，再析构节点与 waiter，成功返回也经过此清理。
                let _registration = unsafe { self.waiters.register(node.as_mut()) };
                if self.try_wait() {
                    true
                } else {
                    waiter.park();
                    self.try_wait()
                }
            };
            if consumed {
                // 已消费者仍登记时可能接到下一 token 的通知；撤登记后把剩余 token 的唤醒转交。
                if self.is_ready() {
                    self.waiters.wake_one();
                }
                return;
            }
        }
    }

    /// 用户父只响应致命取消，真实完成token优先，撤除登记后才返回。
    pub fn wait_killable(&self) -> WaitResult {
        sched::assert_switchable();
        loop {
            if self.try_wait() {
                return WaitResult::Woken;
            }
            let result = {
                let waiter = Waiter::new_killable();
                let mut node = pin!(WaitNode::new(waiter.waker()));
                // SAFETY: self保活完成存储；node固定当前栈，registration先于node/waiter撤除。
                let _registration = unsafe { self.waiters.register(node.as_mut()) };
                if self.try_wait() {
                    Some(WaitResult::Woken)
                } else {
                    let result = waiter.park_signal();
                    if self.try_wait() {
                        Some(WaitResult::Woken)
                    } else {
                        (result == WaitResult::Interrupted).then_some(result)
                    }
                }
            };
            if let Some(result) = result {
                if self.is_ready() {
                    self.waiters.wake_one();
                }
                return result;
            }
        }
    }

    /// Acquire 观察是否有 token，不消费且不保证其他消费者不会先取走它。
    pub fn is_ready(&self) -> bool {
        self.tokens.load(Ordering::Acquire) != 0
    }

    /// 只观察真实等待源成员，用于确认登记与最终撤链，不消费完成 token。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_waiting(&self) -> usize {
        self.waiters.test_members()
    }
}
