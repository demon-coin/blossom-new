//! 内核日志后台串口打印服务（Log Printer Task）。
//!
//! ## 模块职责与架构定位
//! 本模块运行在引导核（CPU 0）上的一个常驻普通内核任务中，负责将 `klog::ring` 中的结构化日志异步刷写到控制台串口（UART）：
//! - **上游触发方**：各核 `klog::write` 提交日志后向 CPU 0 发送 `KICK_LOG_PRINTER` 核间中断；CPU 0 的 `cpu::handle_ipi` 调用 [`wake_printer`] 唤醒本服务与 `/dev/kmsg` 读者。
//! - **下游依赖**：`klog::ring::Cursor`（拉取稳定日志副本）与 `console` 子系统（非阻塞写入 UART FIFO 并在满时等待发送中断）。
//!
//! ## 核心设计与输出状态流
//! 后台任务通过“先登记等待队列、再二次检查业务条件”的事件驱动协议阻塞与恢复，无需定时器周期轮询；单条日志在任务栈上分三段按字节流式送入 UART：
//!
//! ```text
//!  [ 等待新日志 read_next ]
//!    ├─ cursor.read() 命中 ──────────────────────────────┐
//!    └─ 为空：注册 RECORD_WAIT ──► 复查 cursor.read() ──► park() 睡眠
//!                                                        │ (IPI wake_printer 唤醒)
//!    ┌───────────────────────────────────────────────────┘
//!    ▼
//!  [ 栈上格式化固定协议头 ]
//!    "KLOG seq=<u64> level=<STR> ns=<u64> cpu=<usize> task=<usize|none> truncated=<0|1> lost=<u64> "
//!    │
//!    ▼
//!  [ 分段流式写入 UART (offset: 0 .. header.len + body.len + 1) ]
//!    ┌──────────────────┬──────────────────┬──────┐
//!    │   header bytes   │    body bytes    │ '\n' │
//!    └──────────────────┴──────────────────┴──────┘
//!    循环调用 console::try_write(remaining):
//!      - accepted > 0 ──► offset += accepted，继续发送剩余后缀
//!      - accepted == 0 ─► 注册 console::tx_wait_queue() ──► arm_tx_interrupt() ──► park() 等待 TX FIFO 空闲中断
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 等待队列锁（`RECORD_WAIT`、`console::tx_wait_queue()`、`testing::PROGRESS`）与 `ring::RING` 自旋锁、串口硬件锁相互独立，绝不嵌套获取。
//! - 在 `ktest` 模式下，`testing::OBSERVED` 与 `testing::SERVICE` 均为短自旋锁：锁临界区内只读写纯标量或克隆 `Weak<Task>`，严禁在持锁期间升级 `Arc<Task>`、查询调度器任务状态或进入睡眠，防止与调度器锁发生反转死锁。

use alloc::sync::Arc;
use core::alloc::AllocError;
use core::fmt::Write;
use core::pin::pin;

use crate::console;
use crate::fs::poll::PollWaitHead;
use crate::task;
use crate::wait::{WaitNode, WaitQueue, Waiter};

use super::{Cursor, ReadRecord, StackText};

/// 后台打印任务等待新日志入环的事件队列（由 IPI 路径中的 `wake_printer` 唤醒）。
static RECORD_WAIT: WaitQueue = WaitQueue::new();
/// 单条日志结构化协议头部在任务栈上的最大字节容量（足以容纳全部固定字段与 64 位十进制最大值）。
const HEADER_TEXT_CAPACITY: usize = 256;

/// 在引导核开启中断及启动从核之前，一次性初始化 `/dev/kmsg` 读者事件源并创建绑定在 CPU 0 上的后台日志打印任务。
pub fn initialize_service() -> Result<(), AllocError> {
    assert!(
        super::READER_SOURCE.get().is_none(),
        "klog service initialized twice"
    );
    let reader_source = Arc::try_new(PollWaitHead::new())?;
    let prepared = task::prepare_kernel_on(0, run)?;
    super::READER_SOURCE.call_once(|| reader_source);
    drop(prepared.publish());
    Ok(())
}

/// 由 CPU 0 的核间中断处理路径在消费 `KICK_LOG_PRINTER` 标志后调用：广播通知 `/dev/kmsg` 读者（无需等待串口慢速输出）并唤醒后台打印任务。
pub(crate) fn wake_printer() {
    super::wake_readers();
    RECORD_WAIT.wake_one();
}

/// 从游标读取下一条可用日志记录；若当前无新记录则在 `RECORD_WAIT` 上阻塞等待。
///
/// 采用标准的“注册等待节点 ➔ 二次复查环形缓冲区 ➔ 睡眠”两阶段协议，消除检查为空与入睡之间错失 `wake_printer` 唤醒信号的竞态窗口。
fn read_next(cursor: &mut Cursor) -> ReadRecord {
    loop {
        // Step 1: 快速路径直接尝试读取，若已有未消费记录则无需构造等待节点。
        if let Some(record) = cursor.read() {
            return record;
        }

        // Step 2: 慢速路径：将栈上固定的等待节点挂入 RECORD_WAIT，随后复查游标以防入队前刚有新日志到达。
        let record = {
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `RECORD_WAIT` 是 `'static` 全局等待队列，生命周期覆盖整个内核运行期；
            // 2. `node` 由 `pin!` 固定在当前栈帧内，地址稳定；
            // 3. `_registration` 为局部变量且未被 `mem::forget`，根据 Rust 逆序析构规则，无论本作用域正常退出还是栈展开，`_registration` 都会先于 `node` 和 `waiter` 析构并在队列锁内完成摘链。
            let _registration = unsafe { RECORD_WAIT.register(node.as_mut()) };
            if let Some(record) = cursor.read() {
                Some(record)
            } else {
                waiter.park();
                None
            }
        };
        if let Some(record) = record {
            return record;
        }
    }
}

/// 当 UART 发送缓冲区（TX FIFO）已满时，挂入串口发送等待队列并开启 TX 空闲中断，等待硬件腾出空间。
///
/// 返回值表示本次调用是否实际进入了 `park()` 阻塞睡眠（若开启中断时发现硬件恰好已腾出空间，则立即返回 `false` 重试写入）。
fn wait_for_tx() -> bool {
    let waiter = Waiter::new();
    let mut node = pin!(WaitNode::new(waiter.waker()));
    // SAFETY:
    // 1. `console::tx_wait_queue()` 返回 `'static` 串口发送等待队列，永久有效；
    // 2. `node` 由 `pin!` 固定在当前函数栈帧上；
    // 3. `_registration` 未被 `mem::forget`，在本函数返回前必定先析构并从等待队列摘除 `node`，随后 `node` 与 `waiter` 才依次析构。
    let _registration = unsafe { console::tx_wait_queue().register(node.as_mut()) };
    // 开启串口发送中断的同时复查硬件是否已有空闲槽位；仅当仍无空间（返回 false）时才睡眠等待中断唤醒。
    if !console::arm_tx_interrupt() {
        waiter.park();
        #[cfg(feature = "ktest")]
        testing::after_tx_wait();
        true
    } else {
        false
    }
}

/// 后台日志打印任务的主循环入口（常驻运行于普通内核任务上下文，永不主动退出）。
fn run() -> i32 {
    #[cfg(feature = "ktest")]
    testing::install_service();
    let mut cursor = Cursor::oldest();
    loop {
        // Step 1: 阻塞或立即拉取下一条日志记录的栈上独立副本，此后即使原环形槽位被新日志覆盖也不受影响。
        let current = read_next(&mut cursor);
        let record = &current.record;

        // Step 2: 在任务栈上拼装协议头：`KLOG seq=... level=... ns=... cpu=... task=... truncated=... lost=... `。
        let mut header = StackText::<HEADER_TEXT_CAPACITY>::new();
        write!(
            header,
            "KLOG seq={} level={} ns={} cpu={} task=",
            record.seq,
            record.level.as_str(),
            record.monotonic_ns,
            record.cpu
        )
        .expect("log header formatting failed");
        match record.task {
            Some(task) => write!(header, "{}", task.as_usize()),
            None => header.write_str("none"),
        }
        .expect("log task formatting failed");
        write!(
            header,
            " truncated={} lost={} ",
            u8::from(record.truncated),
            current.lost
        )
        .expect("log metadata formatting failed");
        assert!(!header.is_truncated(), "log header capacity exhausted");

        // Step 3: 按“协议头 -> UTF-8 正文 -> 换行符”三段逻辑视图，通过字节偏移 `offset` 逐段向 UART 非阻塞写出。
        let header = header.as_bytes();
        let body = record.text().as_bytes();
        let length = header.len() + body.len() + 1;
        let mut offset = 0;
        #[cfg(feature = "ktest")]
        testing::begin_record(record.seq, length, current.lost);

        while offset < length {
            let remaining = if offset < header.len() {
                &header[offset..]
            } else if offset < header.len() + body.len() {
                &body[offset - header.len()..]
            } else {
                b"\n"
            };
            let accepted = console::try_write(remaining);
            assert!(
                accepted <= remaining.len(),
                "UART accepted an invalid prefix"
            );
            // 若硬件 FIFO 暂时无法接收任何字节，挂起等待 TX 中断；否则推进已发送偏移。
            if accepted == 0 {
                #[cfg(not(feature = "ktest"))]
                wait_for_tx();
                #[cfg(feature = "ktest")]
                if wait_for_tx() {
                    testing::pause_after_partial_write(offset, length);
                }
            } else {
                offset += accepted;
                #[cfg(feature = "ktest")]
                testing::advance(offset, accepted);
            }
        }
        #[cfg(feature = "ktest")]
        testing::finish_record(record.seq);
    }
}

/// 内核自测（`ktest`）专用的打印服务进度观测与确定性暂停控制接口。
#[cfg(feature = "ktest")]
pub mod testing {
    use alloc::sync::{Arc, Weak};
    use core::pin::pin;
    use core::sync::atomic::{AtomicUsize, Ordering};

    use crate::sched;
    use crate::sync::{Completion, SpinLock};
    use crate::task::{self, State, Task};
    use crate::wait::{WaitNode, WaitQueue, Waiter};

    /// 后台日志打印任务当前输出进度的标量观测快照。
    #[derive(Clone, Copy)]
    pub(crate) struct Snapshot {
        /// 当前正在向 UART 输出的日志序号（空闲未在输出任何记录时为 `0`）。
        pub seq: u64,
        /// 当前记录已成功送入 UART 的字节偏移量（单位：字节，涵盖协议头、正文与末尾换行符）。
        pub offset: usize,
        /// 当前记录需要输出的总字节长度（单位：字节；空闲时为 `0`）。
        pub length: usize,
        /// 服务启动以来累计成功送入 UART 的总字节数（单位：字节）。
        pub bytes: u64,
        /// 打印任务游标累计检测到的环形缓冲区覆盖丢失条数。
        pub gaps: u64,
        /// 服务启动以来已完整输出完毕（含换行符）的日志记录总条数。
        pub progress: u64,
        /// 打印任务已完成输出的最后一条序号加 `1`（初始为 `1`，供 `catch_up` 判断追赶进度）。
        pub next_seq: u64,
        /// 打印任务因 UART TX FIFO 满而实际进入阻塞等待的累计次数。
        pub tx_waits: u64,
    }

    /// 保护打印进度快照的自旋锁。
    static OBSERVED: SpinLock<Snapshot> = SpinLock::new(Snapshot {
        seq: 0,
        offset: 0,
        length: 0,
        bytes: 0,
        gaps: 0,
        progress: 0,
        next_seq: 1,
        tx_waits: 0,
    });
    /// 后台打印任务的弱引用（避免测试模块额外持有强引用干预任务生命周期）。
    static SERVICE: SpinLock<Option<Weak<Task>>> = SpinLock::new(None);
    /// 供测试控制线程等待特定序号输出完成的事件队列。
    static PROGRESS: WaitQueue = WaitQueue::new();
    /// 单次输出停点状态机原子字：`0`（未武装）➔ `ARMED(1)` ➔ `PAUSED(2)` ➔ `RELEASED(3)` ➔ `0`。
    static PAUSE: AtomicUsize = AtomicUsize::new(0);
    /// 打印任务在停点阻塞等待控制线程放行的同步原语。
    static RESUME: Completion = Completion::new();

    /// 测试可在后台打印路径上注入的确定性暂停位置。
    #[derive(Clone, Copy)]
    pub(crate) enum PausePoint {
        /// 在当前记录已写出部分字节且经历过一次 UART TX 阻塞唤醒后、继续写出剩余字节前暂停。
        AfterPartialWrite,
    }
    /// `PAUSE` 状态值：测试已武装停点，等待打印任务命中断点条件。
    const ARMED: usize = 1;
    /// `PAUSE` 状态值：打印任务已命中停点并准备/正在 `RESUME` 上阻塞。
    const PAUSED: usize = 2;
    /// `PAUSE` 状态值：控制线程已发出放行信号，等待打印任务唤醒后归零复位。
    const RELEASED: usize = 3;
    /// 等待后台打印服务进入空闲阻塞状态的超时上限（单位：秒）。
    const SERVICE_TIMEOUT_SECONDS: u64 = 10;

    /// 在后台打印任务启动入口记录其 `Weak<Task>` 句柄，供测试查询调度状态。
    pub(super) fn install_service() {
        let task = task::current_task();
        *SERVICE.lock() = Some(Arc::downgrade(&task));
    }

    /// 在打印任务取得一条新日志并拼好协议头后，记录当前输出序号、总字节长度并累加覆盖丢失条数。
    pub(super) fn begin_record(seq: u64, length: usize, lost: u64) {
        let mut state = OBSERVED.lock();
        state.seq = seq;
        state.offset = 0;
        state.length = length;
        state.gaps = state
            .gaps
            .checked_add(lost)
            .expect("log gap counter overflow");
    }

    /// 在 UART 实际接收一批字节后，更新当前记录的已发送偏移并累加历史总发送字节数。
    pub(super) fn advance(offset: usize, accepted: usize) {
        let mut state = OBSERVED.lock();
        state.offset = offset;
        state.bytes = state
            .bytes
            .checked_add(accepted as u64)
            .expect("log byte counter overflow");
    }

    /// 在打印任务从 UART TX 等待队列实际睡眠并唤醒返回后，递增 TX 阻塞等待计数。
    pub(super) fn after_tx_wait() {
        let mut state = OBSERVED.lock();
        state.tx_waits = state
            .tx_waits
            .checked_add(1)
            .expect("log TX wait counter overflow");
    }

    /// 在整条日志（含末尾换行符）全部送入 UART 后清空当前活动记录状态、推进 `next_seq`，并在锁外唤醒所有等待追赶进度的测试线程。
    pub(super) fn finish_record(seq: u64) {
        {
            let mut state = OBSERVED.lock();
            state.seq = 0;
            state.offset = 0;
            state.length = 0;
            state.next_seq = seq.checked_add(1).expect("log sequence exhausted");
            state.progress = state
                .progress
                .checked_add(1)
                .expect("log progress counter overflow");
        }
        PROGRESS.wake_all();
    }

    /// 若测试已武装 `AfterPartialWrite` 停点，且当前记录已写出部分字节但尚未写完，则在 `RESUME` 上阻塞打印任务直到控制线程放行。
    pub(super) fn pause_after_partial_write(offset: usize, length: usize) {
        if offset == 0 || length <= offset {
            return;
        }
        if PAUSE
            .compare_exchange(ARMED, PAUSED, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            RESUME.wait();
            assert_eq!(
                PAUSE.compare_exchange(RELEASED, 0, Ordering::AcqRel, Ordering::Acquire),
                Ok(RELEASED),
                "log resume token without release"
            );
        }
    }

    /// 获取后台打印任务当前的输出进度快照副本。
    pub(crate) fn snapshot() -> Snapshot {
        *OBSERVED.lock()
    }

    /// 获取后台打印任务的临时强引用（在 `SERVICE` 自旋锁外执行 `Weak::upgrade`，避免持锁触发引用计数析构）。
    pub(crate) fn service_task() -> Option<Arc<Task>> {
        let service = SERVICE.lock().clone();
        service.and_then(|weak| weak.upgrade())
    }

    /// 查询后台打印任务在调度器中的当前运行状态（如 `Running`、`Runnable`、`Blocked`）。
    pub(crate) fn service_state() -> Option<State> {
        service_task().map(|task| sched::test_state(&task))
    }

    /// 武装一次性的后台打印暂停点；同一测试夹具必须等上一次停点完全恢复并复位到 `0` 后才能再次武装。
    pub(crate) fn arm_pause(point: PausePoint) {
        match point {
            PausePoint::AfterPartialWrite => {
                assert!(
                    PAUSE
                        .compare_exchange(0, ARMED, Ordering::AcqRel, Ordering::Acquire)
                        .is_ok(),
                    "log pause already armed"
                );
            }
        }
    }

    /// 检查打印任务是否已进入 `PAUSED` 停点状态（若需确认其已在调度器层面休眠，调用方还应配合 `service_state()` 检查 `State::Blocked`）。
    pub(crate) fn is_paused() -> bool {
        PAUSE.load(Ordering::Acquire) == PAUSED
    }

    /// 放行已命中断点的后台打印任务：将状态从 `PAUSED` 推进至 `RELEASED` 并投递 `RESUME` 完成信号（调用时不得持有打印服务恢复所需的锁）。
    pub(crate) fn release_pause() {
        assert_eq!(
            PAUSE.compare_exchange(PAUSED, RELEASED, Ordering::AcqRel, Ordering::Acquire),
            Ok(PAUSED),
            "log printer has not reached its pause"
        );
        RESUME.complete();
    }

    /// 取消尚未命中的停点武装，或唤醒放行已经停在断点上的打印任务（用于测试夹具清理路径）。
    ///
    /// - 若当前为 `0`（未武装）或 `RELEASED`（已放行），幂等直接返回；
    /// - 若当前为 `ARMED`，原子撤回至 `0`，不向 `RESUME` 发送多余的完成令牌；
    /// - 若当前为 `PAUSED`，原子推进至 `RELEASED` 并调用 `RESUME.complete()` 唤醒打印任务。
    ///
    /// 调用方在调用本函数后仍需等待自身测试生产者结束，并通过 `catch_up` / `wait_service_blocked` 确认打印服务排空停稳后方可重新武装。
    pub(crate) fn cancel_pause() {
        loop {
            let state = PAUSE.load(Ordering::Acquire);
            let next = match state {
                0 | RELEASED => return,
                ARMED => 0,
                PAUSED => RELEASED,
                _ => unreachable!("invalid log pause state"),
            };
            if PAUSE
                .compare_exchange(state, next, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                if state == PAUSED {
                    RESUME.complete();
                }
                return;
            }
        }
    }

    /// 阻塞当前测试任务，直到后台打印任务完成 `[1, boundary)` 范围内所有日志记录的串口输出。
    ///
    /// `boundary` 为调用方预先采样到的排他右边界序号（如 `klog::next_sequence()`），等待过程仅关注该固定边界，不受后续新产生日志的干扰。
    pub(crate) fn catch_up(boundary: u64) {
        sched::assert_switchable();
        assert!(1 <= boundary && boundary <= super::super::next_sequence());
        loop {
            if boundary <= snapshot().next_seq {
                return;
            }
            let waiter = Waiter::new();
            let mut node = pin!(WaitNode::new(waiter.waker()));
            // SAFETY:
            // 1. `PROGRESS` 是 `'static` 全局等待队列，永久存活；
            // 2. `node` 由 `pin!` 固定在当前循环迭代的栈帧内；
            // 3. `_registration` 未被 `mem::forget`，在每轮迭代返回或跳出前必定先析构并从 `PROGRESS` 队列摘链，随后才析构 `node` 与 `waiter`。
            let _registration = unsafe { PROGRESS.register(node.as_mut()) };
            if boundary <= snapshot().next_seq {
                return;
            }
            waiter.park();
        }
    }

    /// 在有限超时（`SERVICE_TIMEOUT_SECONDS`）内主动让出 CPU，等待后台打印任务排空全部已发布日志并在 `RECORD_WAIT` 上进入 `State::Blocked` 休眠状态。
    pub(crate) fn wait_service_blocked() {
        sched::assert_switchable();
        let start = crate::arch::read_counter();
        let limit = crate::arch::counter_frequency()
            .checked_mul(SERVICE_TIMEOUT_SECONDS)
            .expect("log service deadline overflow");
        loop {
            let observed = snapshot();
            if observed.length == 0
                && super::super::next_sequence() <= observed.next_seq
                && service_state() == Some(State::Blocked)
            {
                return;
            }
            assert!(
                crate::arch::read_counter().wrapping_sub(start) < limit,
                "log service did not block"
            );
            sched::yield_now();
        }
    }
}
