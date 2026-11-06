//! 基于完成令牌计数（Token）的事件完成量（`Completion`）模块。
//!
//! # 模块职责与架构定位
//! 本模块提供类似计数信号量的事件完成同步原语，用于协调异步 I/O 回调、工作队列任务（Workqueue）、
//! 线程/进程退出收敛等“生产者通知完成、消费者等待完成”的场景。
//! 上游供块设备 `Bio` 完成等待、工作队列刷新、进程 `wait` / 线程汇合等子系统调用；
//! 下游依赖原子计数器（`AtomicUsize`）与等待队列（`WaitQueue`）。
//!
//! # 核心设计与完成令牌状态机
//! 每次调用 [`Completion::complete`] 会以 `Release` 内存序递增一个可消费的完成令牌（`tokens`），并唤醒一名等待者；
//! 每次成功完成等待（[`Completion::try_wait`] / [`Completion::wait`] / [`Completion::wait_killable`]）会以 `Acquire` 内存序消费一个令牌。
//!
//! ```text
//!   生产者 (IRQ / Work / 子任务)              Completion                  消费者 (等待任务)
//!  ──────────────────────────────         ──────────────────          ────────────────────────
//!   写入结果数据
//!        │
//!        ▼
//!   complete():
//!   1. tokens.fetch_add(1, Release) ───►  [ tokens: usize ]  ◄─── 1. try_wait():
//!   2. waiters.wake_one() ─────────────►  [ waiters 队列  ]          tokens.fetch_sub(1, Acquire)
//!                                                 │               2. 若 tokens == 0：
//!                                                 │                  在栈上注册 WaitNode 并挂起
//!                                                 └─────────────► 3. 被唤醒并成功消费 1 个 token 后：
//!                                                                    先析构 _registration 撤出等待队列；
//!                                                                    若发现仍有剩余 token (is_ready())，
//!                                                                    接力调用 wake_one() 唤醒下一名等待者
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 1. **生产端上下文无关性**：`complete()`、`try_wait()` 与 `is_ready()` 仅执行原子指令与等待队列唤醒（内部使用关中断 `SpinLock`），可在**任意上下文**（包括硬件中断处理函数、持自旋锁期间或普通任务）中安全调用。
//! 2. **等待端可切换要求**：`wait()` 与 `wait_killable()` 可能挂起当前任务，**只能在可调度的普通任务上下文中调用**，严禁在中断上下文、持有 `SpinLock`、`IrqSaveGuard` 或 `PreemptGuard` 期间调用。
//! 3. **唤醒接力契约**：当等待任务尚未从 `waiters` 撤链时，若连续到达多个 `complete()`，`wake_one()` 可能重复命中同一个尚在栈上的等待节点；因此等待者在撤除队列登记后必须检查 `is_ready()`，若仍有剩余令牌则主动调用 `wake_one()` 将唤醒信号传递给队列中的后续等待者。

use core::pin::pin;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::sched;
use crate::wait::{WaitNode, WaitQueue, WaitResult, Waiter};

/// 基于原子令牌计数的完成同步原语。
///
/// 每次 [`Completion::complete`] 存入一个完成令牌，每次等待操作消费一个完成令牌，
/// 支持先触发 `complete` 后调用 `wait`（此时 `wait` 会直接消费已有令牌而无需阻塞）。
pub struct Completion {
    /// 当前尚未被消费的完成令牌数量；通过 `Release` 累加与 `Acquire` 递减同步生产者的前置内存写入。
    tokens: AtomicUsize,
    /// 当 `tokens == 0` 时阻塞等待令牌到达的任务等待队列。
    waiters: WaitQueue,
}

impl Completion {
    /// 创建一个初始令牌数为 0、等待队列为空的 [`Completion`]，支持 `const` 静态初始化。
    pub const fn new() -> Self {
        Self {
            tokens: AtomicUsize::new(0),
            waiters: WaitQueue::new(),
        }
    }

    /// 发布一个完成令牌，并唤醒等待队列中的一名等待者。
    ///
    /// 使用 `Ordering::Release` 递增 `tokens`，保证调用 `complete()` 之前的所有内存写入
    /// 对随后通过 `try_wait` / `wait` 消费该令牌的任务完全可见。若令牌计数发生 `usize` 溢出则直接 panic。
    pub fn complete(&self) {
        self.tokens
            .fetch_update(Ordering::Release, Ordering::Relaxed, |tokens| {
                tokens.checked_add(1)
            })
            .expect("completion token count overflow");
        self.waiters.wake_one();
    }

    /// 尝试非阻塞地消费一个完成令牌。
    ///
    /// 若当前 `tokens > 0`，则以 `Ordering::Acquire` 将其减 1 并返回 `true`（同时建立与 `complete()` 的内存同步）；
    /// 若当前 `tokens == 0`，则不修改计数并返回 `false`。
    pub fn try_wait(&self) -> bool {
        self.tokens
            .fetch_update(Ordering::Acquire, Ordering::Relaxed, |tokens| {
                tokens.checked_sub(1)
            })
            .is_ok()
    }

    /// 不可中断地阻塞当前任务，直到成功消费到一个完成令牌为止。
    ///
    /// 只能在允许任务切换的普通任务上下文中调用。
    #[cfg_attr(
        not(feature = "ktest"),
        expect(
            dead_code,
            reason = "执行与工作队列测试用真实阻塞 token 协调回调和清理尾部"
        )
    )]
    pub fn wait(&self) {
        sched::assert_switchable();
        // Step 1: 快路径——若已有现成令牌，直接消费并返回，无需入队。
        if self.try_wait() {
            return;
        }

        // Step 2: 慢路径——循环在栈上注册等待节点并挂起，直到成功消费到一个令牌。
        loop {
            let consumed = {
                let waiter = Waiter::new();
                let mut node = pin!(WaitNode::new(waiter.waker()));
                // SAFETY:
                // 1. `&self` 共享借用保证 `self.waiters` 队列在整个内部作用域期间有效；
                // 2. `pin!` 将 `node` 固定在当前任务栈上，挂起期间内存地址保持不变；
                // 3. `_registration` 在离开该内部代码块时必定先于 `node` 和 `waiter` 析构，
                //    将 `node` 从 `self.waiters` 链表中移除，绝不遗留悬垂节点。
                let _registration = unsafe { self.waiters.register(node.as_mut()) };
                // Step 3: 登记后、睡眠前再次尝试消费令牌，防止在登记前恰好有 `complete()` 发生而丢失唤醒。
                if self.try_wait() {
                    true
                } else {
                    // Step 4: 挂起当前任务，等待 `complete()` 调用 `wake_one()` 唤醒后再次尝试消费令牌。
                    waiter.park();
                    self.try_wait()
                }
            };
            if consumed {
                // Step 5: 此时 `_registration` 已析构（当前节点已从等待队列撤除）。
                // 若本节点在撤链前曾吸收了多次 `complete()` 的 `wake_one()` 通知且仍有剩余令牌，
                // 主动接力唤醒队列中的下一名等待者，防止后续等待者漏收唤醒。
                if self.is_ready() {
                    self.waiters.wake_one();
                }
                return;
            }
        }
    }

    /// 可被致命信号（如 `SIGKILL` 或进程组退出）打断地等待消费一个完成令牌。
    ///
    /// 只能在允许任务切换的普通任务上下文中调用：
    /// - 若成功消费到一个完成令牌，返回 [`WaitResult::Woken`]；
    /// - 若在尚未消费到令牌时收到致命终止请求，则在撤除等待队列登记后返回 [`WaitResult::Interrupted`]。
    pub fn wait_killable(&self) -> WaitResult {
        sched::assert_switchable();
        loop {
            // Step 1: 优先尝试直接消费已有令牌；即使当前任务已有待处理信号，已就绪的完成事件仍优先消费。
            if self.try_wait() {
                return WaitResult::Woken;
            }
            // Step 2: 在内部作用域中注册可响应致命信号的等待节点。
            let result = {
                let waiter = Waiter::new_killable();
                let mut node = pin!(WaitNode::new(waiter.waker()));
                // SAFETY:
                // 1. `&self` 共享借用保证 `self.waiters` 队列在内部作用域内有效；
                // 2. `pin!` 将 `node` 固定在当前栈帧上；
                // 3. 离开内部作用域时 `_registration` 先于 `node` 和 `waiter` 析构，确保从等待队列安全撤链。
                let _registration = unsafe { self.waiters.register(node.as_mut()) };
                // Step 3: 登记后二次检查令牌，覆盖登记前的并发 `complete()`。
                if self.try_wait() {
                    Some(WaitResult::Woken)
                } else {
                    // Step 4: 进入可被致命信号唤醒的睡眠；返回后优先尝试消费令牌，无令牌且确系被信号打断时才返回 Interrupted。
                    let result = waiter.park_signal();
                    if self.try_wait() {
                        Some(WaitResult::Woken)
                    } else {
                        (result == WaitResult::Interrupted).then_some(result)
                    }
                }
            };
            if let Some(result) = result {
                // Step 5: 当前节点已从队列撤除；若池中仍有未消费令牌，接力唤醒下一名等待者。
                if self.is_ready() {
                    self.waiters.wake_one();
                }
                return result;
            }
        }
    }

    /// 以 `Ordering::Acquire` 检查当前是否存在至少一个未消费的完成令牌。
    ///
    /// 该方法只做无副作用观测，不会消费令牌，也不保证返回 `true` 后令牌不会被其他并发消费者先行取走。
    pub fn is_ready(&self) -> bool {
        self.tokens.load(Ordering::Acquire) != 0
    }

    /// 返回当前登记在等待队列中的等待节点数量（仅用于内核单元测试检查入队与撤链状态）。
    #[cfg(feature = "ktest")]
    pub(crate) fn test_waiting(&self) -> usize {
        self.waiters.test_members()
    }
}
