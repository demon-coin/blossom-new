//! 每核执行状态（`CpuLocal`）、多核启动握手、核间中断（IPI）与跨核同步函数调用（CSD）。
//!
//! # 模块职责与架构定位
//! 本模块维护系统中每个在线逻辑 CPU 的核心私有状态与跨核协调原语：
//! - 上游调用者：
//!   - 启动入口（`boot`）调用 `initialize`、`prepare_secondary_stacks`、`start_secondaries`、`publish_online`、`wait_until_started` 完成主辅核启动握手。
//!   - 调度器（`sched`）、定时器（`time` / `timer`）、工作队列（`workqueue`）、内存管理（`mm` TLB 击落）通过 `current()`、`get(id)`、`kick()` 与 `call()` 访问本核上下文或向远端核发起通知/同步回调。
//!   - 硬中断与软中断入口使用 `HardIrqGuard`、`BottomHalfGuard`、`handle_timer`、`handle_ipi` 维护中断嵌套深度并分发定时器/IPI 事件。
//! - 下游依赖：调用 `arch` 模块安装每核基址寄存器（RISC-V `tp` / LoongArch `$r21`）、发送硬件 IPI、启动辅核及执行低功耗等待（`WFI`/`IDLE`）。
//!
//! # 核心设计与关键状态机
//! 1. **主辅核启动与栈准备流程**：
//!    ```text
//!    BSP: cpu::initialize(info)
//!      ├─ 构造 CPU_LOCALS[0..cpu_count]
//!      ├─ 填写 BOOT_RECORDS[0] (使用静态 BOOT_STACK) 并安装 BSP 每核基址
//!      ▼
//!    BSP: mm::initialize() 建立物理页分配器与直接映射区
//!      ▼
//!    BSP: cpu::prepare_secondary_stacks()
//!      ├─ 为 AP (1..cpu_count) 分配 64 KiB 动态连续页栈并回填 BOOT_RECORDS[id].stack_top
//!      ▼
//!    BSP: cpu::start_secondaries() ──(固件唤醒 AP i，传入 BOOT_RECORDS[i] 物理地址)──┐
//!      │                                                                           ▼
//!      │ 自旋等待 local.online == true (5s 超时)                             AP i: 切换至 stack_top，安装 local 基址
//!      │                                                                     AP i: 开启本地中断后调用 publish_online()
//!      ◄───────────────────────────────────────────────────────────────────────────┘
//!    ```
//! 2. **跨核同步调用（Call Single Data, `call`）栈借用生命周期**：
//!    `cpu::call(target, callback, argument)` 在调用方当前栈帧上直接构造并固定（`pin!`）`Call` 节点，将其链入目标核的 `CpuLocal::calls` 单向链表并发送硬件 IPI，随后在本地自旋等待 `request.done` 变为 `true`：
//!    ```text
//!    调用核 (Sender Stack)                          目标核 (Target IPI Handler)
//!    ┌──────────────────────────┐
//!    │ pin!(Call {              │
//!    │   callback, argument,    │
//!    │   done: false            │
//!    │ })                       │
//!    └────────────┬─────────────┘
//!                 │ 1. calls.lock().push(node)
//!                 │ 2. arch::send_ipi(target)
//!                 ▼                                 ┌──────────────────────────────┐
//!          while !done.load(Acquire)                │ 3. calls.lock().pop()        │
//!            spin_loop()                            │ 4. 锁外执行 callback(arg)     │
//!                 ▲                                 │ 5. done.store(true, Release) │
//!                 └─────────(Acquire 同步)──────────┴──────────────────────────────┘
//!          栈帧安全弹出并销毁 Call 节点
//!    ```
//!    由于发送方在观察到 `done == true`（`Acquire`）之前绝不会退出 `cpu::call` 栈帧，且目标核在 `done.store(true, Release)` 之后不再解引用 `request` 指针，从而无需堆分配即可保证跨核请求节点的内存安全。
//! 3. **核间唤醒原因位（`kick_reasons`）**：
//!    - `KICK_RESCHEDULE` (bit 0)：请求目标核置位 `need_resched` 并在安全点触发抢占调度。
//!    - `KICK_TIMER_REPROGRAM` (bit 1)：请求目标核重新计算并编程本地硬件比较器（Timer Deadline）。
//!    - `KICK_LOG_PRINTER` (bit 2)：唤醒日志打印任务。
//!    - `KICK_STOP` (bit 3)：关机/重启前通知目标核永久停机。
//!    - `KICK_SCHED_WAKE` (bit 4)：仅将目标核从 `WFI`/`IDLE` 低功耗状态唤醒以重新检查就绪队列，不强制置位正在运行任务的 `need_resched`。
//!
//! # 锁获取顺序与并发契约
//! - `CpuLocal::calls` 自旋锁仅保护 `CallQueue` 链表头尾指针的入队（`push`）与出队（`pop`），严禁在持有 `calls` 锁期间执行 `callback`。
//! - 发起跨核同步调用 `cpu::call` 的 CPU 必须保持本地硬中断开启（或能够响应 IPI），且不得持有目标 `callback` 内部需要获取的自旋锁，否则当两核互相发起 `cpu::call` 时会导致死锁。
//! - `CpuLocal` 中的 `scheduler_ctx`、`current_task`、`active_mm` 仅允许所属 CPU 在本地硬中断关闭（IRQ-off）的上下文切换临界区内直接访问。

mod set;
pub(crate) mod stack;
#[cfg(feature = "ktest")]
pub(crate) use set::AtomicCpuSet;
pub use set::CpuSet;

use alloc::alloc::AllocError;
use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use spin::Once;

use crate::boot::{BootInfo, MAX_CPUS};
use crate::cpu::stack::StackSlot;
use crate::sched::context::TaskContext;
use crate::sync::{IrqSaveGuard, SpinLock};
use crate::task::Task;
use core::marker::PhantomData;

/// 每个 CPU 启动与调度内核栈及普通任务内核栈的字节大小（64 KiB）。
pub const STACK_SIZE: usize = 64 * 1024;

/// IPI 软件原因位：请求目标核置位 `need_resched` 并在下一个安全点执行抢占调度。
pub(crate) const KICK_RESCHEDULE: usize = 1 << 0;
/// IPI 软件原因位：请求目标核根据定时器轮（`TimerBase`）重新编程本地硬件定时器。
pub(crate) const KICK_TIMER_REPROGRAM: usize = 1 << 1;
/// IPI 软件原因位：请求唤醒内核日志异步输出任务。
pub(crate) const KICK_LOG_PRINTER: usize = 1 << 2;
/// IPI 软件原因位：通知目标核在处理完挂起的跨核调用后永久停机。
const KICK_STOP: usize = 1 << 3;
/// IPI 软件原因位：仅唤醒处于 idle 等待的目标核重新检查就绪队列，不置位 `need_resched`。
pub(crate) const KICK_SCHED_WAKE: usize = 1 << 4;

/// 内核自测停点状态：未武装（空闲）。
#[cfg(feature = "ktest")]
const TEST_PAUSE_IDLE: usize = 0;
/// 内核自测停点状态：已武装，等待目标核到达指定边界。
#[cfg(feature = "ktest")]
const TEST_PAUSE_ARMED: usize = 1;
/// 内核自测停点状态：目标核已到达边界并正在自旋等待放行。
#[cfg(feature = "ktest")]
const TEST_PAUSE_REACHED: usize = 2;
/// 内核自测停点状态：控制核已发出放行许可，目标核即将恢复为 `TEST_PAUSE_IDLE`。
#[cfg(feature = "ktest")]
const TEST_PAUSE_RELEASED: usize = 3;

/// 链接期静态保留的主核（BSP）启动栈包装类型（按 16 字节对齐，元组字段 `0` 为固定 `STACK_SIZE` 字节的栈内存数组）。
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub struct BootStack(pub [u8; STACK_SIZE]);

/// 链接在 `.boot.stacks` 段中的主核（BSP）唯一静态启动栈实例。
#[no_mangle]
#[link_section = ".boot.stacks"]
#[used]
pub static mut BOOT_STACK: BootStack = BootStack([0; STACK_SIZE]);

/// 传递给辅核（AP）汇编启动入口的只读启动参数记录（按 C ABI 布局，供汇编按固定偏移读取）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CpuBoot {
    /// 该 CPU 专属内核启动栈的栈顶虚拟地址（单位：字节）。
    pub stack_top: usize,
    /// 该 CPU 对应的 [`CpuLocal`] 静态实例虚拟地址（单位：字节）。
    pub local: usize,
}

/// 跨核同步函数调用（[`call`]）在调用方栈帧上分配的侵入式单向链表节点。
struct Call {
    /// 需要在目标 CPU 上执行的回调函数指针。
    callback: unsafe fn(*mut ()),
    /// 传递给 `callback` 的上下文参数裸指针。
    argument: *mut (),
    /// 指向队列中下一个 `Call` 节点的链表指针（仅在持有目标核 `CpuLocal::calls` 自旋锁时读写）。
    next: UnsafeCell<*mut Call>,
    /// 标记目标 CPU 是否已执行完 `callback`；目标核以 `Release` 置为 `true`，发送核以 `Acquire` 轮询等待。
    done: AtomicBool,
}

/// 每个 CPU 私有的跨核同步调用单向 FIFO 队列（由 `CpuLocal::calls` 自旋锁保护）。
struct CallQueue {
    /// 队列头部节点指针（`pop` 从此处取出；为空队列时为 `null_mut()`）。
    head: *mut Call,
    /// 队列尾部节点指针（`push` 向此处追加；为空队列时为 `null_mut()`）。
    tail: *mut Call,
}

// SAFETY:
// 1. `CallQueue` 的 `head` 与 `tail` 裸指针仅在持有 `CpuLocal::calls` 自旋锁时被访问。
// 2. 入队的 `Call` 节点固定在发起方栈上，在目标核完成回调并置位 `done`（Release）之前，发起方绝不会销毁该栈帧。
unsafe impl Send for CallQueue {}

impl CallQueue {
    /// 构造一个初始为空的跨核调用队列。
    const fn new() -> Self {
        Self {
            head: ptr::null_mut(),
            tail: ptr::null_mut(),
        }
    }

    /// 将固定在发送方栈上的请求节点 `call` 追加到队尾。
    ///
    /// # Safety
    /// 1. 调用方必须持有保护本队列的 `CpuLocal::calls` 自旋锁。
    /// 2. `call` 必须指向有效的 `Call` 节点（`next` 已初始化为 `null_mut()`、`done` 为 `false`），未同时处于任何其他队列中，且在目标核将 `done` 置为 `true` 之前始终保持地址有效。
    unsafe fn push(&mut self, call: *mut Call) {
        if self.tail.is_null() {
            self.head = call;
        } else {
            // SAFETY:
            // 1. `self.tail` 非空说明尾节点仍在当前队列中且尚未被目标核 `pop` 完成，发送方栈帧仍在等待 `done` 标志，因此 `self.tail` 指向的 `Call` 节点有效。
            // 2. 当前持有 `CpuLocal::calls` 自旋锁，独占访问 `(*self.tail).next`。
            unsafe { *(*self.tail).next.get() = call };
        }
        self.tail = call;
    }

    /// 从队首弹出一个待处理的请求节点指针；若队列为空则返回 `None`。
    ///
    /// # Safety
    /// 1. 调用方必须持有保护本队列的 `CpuLocal::calls` 自旋锁。
    /// 2. 队列中的所有节点均由 [`Self::push`] 按栈生命周期协议入队，在各自 `done` 标志被置为 `true` 前均保持有效。
    unsafe fn pop(&mut self) -> Option<*mut Call> {
        let call = self.head;
        if call.is_null() {
            return None;
        }
        // SAFETY:
        // 1. `call`（即 `self.head`）非空，说明该节点尚未被弹出处理、其 `done` 仍为 `false`，发送方栈上的 `Call` 实例仍然存活。
        // 2. 当前持有 `CpuLocal::calls` 自旋锁，独占读取 `(*call).next` 并更新 `head`/`tail`。
        self.head = unsafe { *(*call).next.get() };
        if self.head.is_null() {
            self.tail = ptr::null_mut();
        }
        Some(call)
    }
}

/// 单个逻辑 CPU 的每核状态控制块（Per-CPU Data），在内核生命周期内静态驻留于 `CPU_LOCALS` 中。
pub struct CpuLocal {
    /// 内核分配的连续逻辑 CPU 编号（范围 `0..count()`，其中 `0` 恒为主核 BSP）。
    pub id: usize,
    /// 硬件平台原生 CPU 标识符（RISC-V Hart ID 或 LoongArch 物理核编号）。
    pub hardware_id: usize,
    /// 本核专属的启动与调度器内核栈槽位。
    stack: StackSlot,
    /// 本核是否已完成早期初始化、开启中断并上线（由本核 `Release` 置位，其他核 `Acquire` 读取）。
    pub online: AtomicBool,
    /// 本核累计处理的本地硬件定时器中断次数。
    pub timer_interrupts: AtomicUsize,
    /// 本核累计处理的核间中断（IPI）次数。
    pub ipi_interrupts: AtomicUsize,
    /// 本核调度器主循环的上下文保存区（仅由本核在关中断状态下执行任务切换时访问）。
    pub(crate) scheduler_ctx: UnsafeCell<TaskContext>,
    /// 本核当前正在运行的普通任务指针（无任务或运行在调度器栈上时为 `null`，仅由本核在关中断下访问）。
    pub(crate) current_task: UnsafeCell<*const Task>,
    /// 本核当前正在运行的任务 ID（为 `0` 表示当前处于 idle/调度器上下文，供跨核或中断处理无锁读取）。
    pub(crate) current_task_id: AtomicUsize,
    /// 本核当前激活的用户地址空间引用（用于惰性 TLB 管理与生命周期维持，仅由本核在关中断下修改）。
    pub(crate) active_mm: UnsafeCell<Option<Arc<crate::mm::space::Mm>>>,
    /// 本核当前的硬中断（Hard IRQ）嵌套深度（`0` 表示不在硬中断中）。
    pub(crate) hardirq_depth: AtomicUsize,
    /// 本核当前的软中断/底半部（Bottom Half）嵌套深度（`0` 表示不在底半部中）。
    pub(crate) bh_depth: AtomicUsize,
    /// 本核当前的显式禁抢占（`PreemptGuard`）嵌套深度（`0` 表示未显式禁用抢占）。
    pub(crate) preempt_depth: AtomicUsize,
    /// 本核的 CPU 时间统计账本（User / System / IRQ / Idle 记账）。
    pub(crate) accounting: crate::accounting::CpuAccount,
    /// 本核的硬件时钟事件源状态（记录下一次本地定时器到期时间）。
    pub(crate) time_event: crate::time::LocalEvent,
    /// 本核的软件定时器容器（管理挂载在本核上的到期回调定时器）。
    pub(crate) timer_base: crate::timer::TimerBase,
    /// 本核绑定的工作队列（Workqueue）内核工作线程池状态。
    pub(crate) worker_pool: crate::workqueue::WorkerPool,
    /// 抢占重调度标志：为 `true` 时提示当前任务在下一个安全点（中断返回或显式让出）调用调度器。
    pub(crate) need_resched: AtomicBool,
    /// 标记本核当前是否正处于调度器 idle 循环中。
    pub(crate) idle: AtomicBool,
    /// 待本核 IPI 处理函数消费的软件原因位掩码（`KICK_*` 常量的按位或）。
    kick_reasons: AtomicUsize,
    /// 本核已完成永久停机（`KICK_STOP`）的确认标志；一旦置为 `true` 即永不恢复。
    stopped: AtomicBool,
    /// 内核自测专用：idle 边界单次停点状态机（`TEST_PAUSE_*`）。
    #[cfg(feature = "ktest")]
    test_idle_pause: AtomicUsize,
    /// 内核自测专用：IPI 处理函数边界单次停点状态机（`TEST_PAUSE_*`）。
    #[cfg(feature = "ktest")]
    test_ipi_pause: AtomicUsize,
    /// 内核自测专用：本核调用 `idle_once` 进入架构低功耗等待入口的累计次数。
    #[cfg(feature = "ktest")]
    test_idle_waits: AtomicUsize,
    /// 内核自测专用：本核实际执行硬件 `WFI`/`IDLE` 指令并被唤醒返回的累计次数。
    #[cfg(feature = "ktest")]
    test_wait_completions: AtomicUsize,
    /// 内核自测专用：本核在 IPI 中实际响应并执行 `KICK_TIMER_REPROGRAM` 的累计次数。
    #[cfg(feature = "ktest")]
    test_timer_reprograms: AtomicUsize,
    /// 保护本核跨核同步调用请求队列（`CallQueue`）的自旋锁。
    calls: SpinLock<CallQueue>,
    /// 内核自测专用：本核调用 `publish_online` 时的栈上局部变量地址探针（用于验证 AP 确实运行在分配给它的专属栈上）。
    #[cfg(feature = "ktest")]
    pub(crate) boot_stack_probe: AtomicUsize,
    /// 内核自测专用：本核调用 `publish_online` 时从硬件寄存器读回的 `CpuLocal` 基址探针。
    #[cfg(feature = "ktest")]
    pub(crate) boot_local_probe: AtomicUsize,
    /// 内核自测专用：本核调用 `publish_online` 时的硬件中断开启状态探针。
    #[cfg(feature = "ktest")]
    pub(crate) boot_irq_ready: AtomicBool,
    /// 内核自测专用：标记本核当前是否正在执行 `cpu::call` 同步回调。
    #[cfg(feature = "ktest")]
    pub(crate) ktest_callback_active: AtomicBool,
}

// SAFETY:
// 1. `scheduler_ctx`、`current_task` 与 `active_mm` 三个 `UnsafeCell` 字段仅由所属 CPU 自身在本地硬中断关闭（IRQ-off）的临界区内读写，远端 CPU 绝不直接访问。
// 2. 其余供跨核访问的字段均为原子类型（`Atomic*`）或由 `SpinLock` 内部同步。
unsafe impl Sync for CpuLocal {}

// SAFETY:
// 1. `CpuLocal` 仅在单核早期初始化（`initialize`）写入 `Once<CpuLocal>` 时发生按值构造，当时尚未开启调度或跨核共享。
// 2. 发布到静态表 `CPU_LOCALS` 后不再移动，其内部裸指针与 `UnsafeCell` 字段遵循本核独占与关中断访问协议。
unsafe impl Send for CpuLocal {}

impl CpuLocal {
    /// 构造指定逻辑编号 `id`、硬件编号 `hardware_id` 与初始栈顶 `stack_top` 的 `CpuLocal` 实例。
    const fn new(id: usize, hardware_id: usize, stack_top: usize) -> Self {
        Self {
            id,
            hardware_id,
            stack: StackSlot::new(stack_top),
            online: AtomicBool::new(false),
            timer_interrupts: AtomicUsize::new(0),
            ipi_interrupts: AtomicUsize::new(0),
            scheduler_ctx: UnsafeCell::new(TaskContext::empty()),
            current_task: UnsafeCell::new(ptr::null()),
            current_task_id: AtomicUsize::new(0),
            active_mm: UnsafeCell::new(None),
            hardirq_depth: AtomicUsize::new(0),
            bh_depth: AtomicUsize::new(0),
            preempt_depth: AtomicUsize::new(0),
            accounting: crate::accounting::CpuAccount::new(),
            time_event: crate::time::LocalEvent::new(),
            timer_base: crate::timer::TimerBase::new(),
            worker_pool: crate::workqueue::WorkerPool::new(),
            need_resched: AtomicBool::new(false),
            idle: AtomicBool::new(false),
            kick_reasons: AtomicUsize::new(0),
            stopped: AtomicBool::new(false),
            #[cfg(feature = "ktest")]
            test_idle_pause: AtomicUsize::new(TEST_PAUSE_IDLE),
            #[cfg(feature = "ktest")]
            test_ipi_pause: AtomicUsize::new(TEST_PAUSE_IDLE),
            #[cfg(feature = "ktest")]
            test_idle_waits: AtomicUsize::new(0),
            #[cfg(feature = "ktest")]
            test_wait_completions: AtomicUsize::new(0),
            #[cfg(feature = "ktest")]
            test_timer_reprograms: AtomicUsize::new(0),
            calls: SpinLock::new(CallQueue::new()),
            #[cfg(feature = "ktest")]
            boot_stack_probe: AtomicUsize::new(0),
            #[cfg(feature = "ktest")]
            boot_local_probe: AtomicUsize::new(0),
            #[cfg(feature = "ktest")]
            boot_irq_ready: AtomicBool::new(false),
            #[cfg(feature = "ktest")]
            ktest_callback_active: AtomicBool::new(false),
        }
    }

    /// 返回本核已发布的启动与调度栈顶虚拟地址（单位：字节）。
    pub fn stack_top(&self) -> usize {
        self.stack.top()
    }

    /// 返回本核动态分配栈的直接映射起始虚拟地址与字节数 `(base_virt, len)`；主核静态栈返回 `None`。
    #[cfg(feature = "ktest")]
    pub(crate) fn stack_allocation(&self) -> Option<(usize, usize)> {
        self.stack.allocation()
    }

    /// 非破坏性地检查本核是否存在尚未处理的 `kick_reasons` 软件通知位（不清除标志位）。
    pub(crate) fn kick_pending(&self) -> bool {
        self.kick_reasons.load(Ordering::Acquire) != 0
    }

    /// 在调度器确认无就绪任务且复查队列后，执行一次底层硬件低功耗等待（`WFI`/`IDLE`）；唤醒并处理完中断后仍以关中断状态返回。
    ///
    /// # Safety
    /// 1. `self` 必须严格等于当前正在执行的 CPU（`current()`）。
    /// 2. 调用时必须运行在本核的调度器栈上，本地硬中断处于关闭状态（IRQ-off），且当前没有普通任务上下文、未持有任何自旋锁或 `IrqSaveGuard`/`PreemptGuard`。
    /// 3. 调用方必须已准备好允许底层架构在等待期间临时开启硬中断并执行中断处理函数。
    pub(crate) unsafe fn idle_once(&self) {
        assert!(core::ptr::eq(self, current()), "idle uses another CPU");
        #[cfg(feature = "ktest")]
        self.test_idle_waits.fetch_add(1, Ordering::Release);
        // SAFETY:
        // 1. 前置 `assert!` 与调用方契约保证了 `self` 为本核 `CpuLocal`，当前处于关中断、无锁且允许临时响应硬中断的调度器栈边界。
        // 2. `self.kick_reasons`、`self.need_resched` 及 `self.test_wait_completions` 均为 `'static` 原子字段，架构层在关中断下做最终复查后才进入硬件等待。
        unsafe {
            #[cfg(feature = "ktest")]
            crate::arch::idle_once(
                &self.kick_reasons,
                &self.need_resched,
                &self.test_wait_completions,
            );
            #[cfg(not(feature = "ktest"))]
            crate::arch::idle_once(&self.kick_reasons, &self.need_resched);
        };
    }
}

/// 全局每核状态表：按逻辑 CPU 编号索引，使用 `Once` 逐槽就地构造以避免在启动栈上产生整个数组的按值临时拷贝。
static CPU_LOCALS: [Once<CpuLocal>; MAX_CPUS] = [const { Once::new() }; MAX_CPUS];
/// 全局辅核启动参数表：由主核在唤醒辅核前填充栈顶与 `CpuLocal` 地址，供辅核汇编启动入口按物理地址读取。
static mut BOOT_RECORDS: [CpuBoot; MAX_CPUS] = [CpuBoot {
    stack_top: 0,
    local: 0,
}; MAX_CPUS];

/// 初始化全局 `CPU_LOCALS` 表与早期启动记录 `BOOT_RECORDS`（仅为主核设置静态栈顶，并安装主核的每核基址寄存器）。
///
/// # Safety
/// 1. 只能由主核（BSP，逻辑编号 0）在本地硬中断关闭、所有辅核（AP）尚未释放启动时调用唯一一次。
/// 2. `info` 必须包含已校验的 `cpu_count`（`1..=MAX_CPUS`）与对应的 `hardware_ids`，且主核静态栈 `BOOT_STACK` 的虚拟地址映射有效。
pub unsafe fn initialize(info: &BootInfo) {
    assert!(
        CPU_LOCALS.iter().all(|local| local.get().is_none()),
        "CPU table is already initialized"
    );

    // Step 1: 逐个构造每个在线逻辑 CPU 的 CpuLocal 实例（主核使用 BOOT_STACK 静态栈顶，辅核栈顶暂置为 0）并写入 BOOT_RECORDS。
    let bsp_stack_top = ptr::addr_of!(BOOT_STACK) as usize + STACK_SIZE;
    for (id, slot) in CPU_LOCALS[..info.cpu_count].iter().enumerate() {
        let stack_top = if id == 0 { bsp_stack_top } else { 0 };
        let local = slot.call_once(|| CpuLocal::new(id, info.hardware_ids[id], stack_top));
        // SAFETY:
        // 1. `id < info.cpu_count <= MAX_CPUS`，下标在 `BOOT_RECORDS` 数组有效范围内。
        // 2. 当前由主核在单核启动阶段独占执行，没有任何辅核并发读取 `BOOT_RECORDS`。
        unsafe {
            ptr::addr_of_mut!(BOOT_RECORDS)
                .cast::<CpuBoot>()
                .add(id)
                .write(CpuBoot {
                    stack_top: local.stack_top(),
                    local: local as *const CpuLocal as usize,
                });
        }
    }

    // Step 2: 将逻辑 0 号核（BSP）的 CpuLocal 基址安装到当前 CPU 的架构专用寄存器中。
    // SAFETY: 逻辑 CPU 0 的 `Once<CpuLocal>` 已在上方循环中完成初始化并具有 `'static` 生命周期，且当前硬中断处于关闭状态。
    unsafe { crate::arch::install_cpu_local(get(0)) };
}

/// 在内存管理子系统（MM）初始化完成后，为所有辅核（`1..count()`）分配专属内核栈并回填 `BOOT_RECORDS` 中的 `stack_top`。
///
/// # Safety
/// 1. 只能由主核（BSP，逻辑 CPU 0）在本地硬中断关闭、物理页分配器与永久可写直接映射区已建立后调用。
/// 2. `cpu::initialize` 必须已经完成，所有辅核（AP）尚未被释放启动，且全部辅核的 `StackSlot` 仍处于未分配的空状态。
/// 3. 调用期间不得有任何固件或其他 CPU 并发读取 `BOOT_RECORDS`。
pub(crate) unsafe fn prepare_secondary_stacks() -> Result<(), AllocError> {
    assert_eq!(current().id, 0, "only BSP may prepare AP stacks");
    assert!(
        !crate::arch::irq_enabled(),
        "AP stack preparation requires IRQ off"
    );
    // Step 1: 为所有辅核（1..count()）批量分配 64 KiB 连续物理页栈并发布到各自的 StackSlot。
    // SAFETY: 调用方保证 MM 已就绪且所有辅核尚未启动；`1..count()` 映射到互不重叠的 `'static CpuLocal` 槽位，由 BSP 独占准备。
    unsafe { stack::prepare_slots((1..count()).map(|id| &get(id).stack)) }?;

    // Step 2: 将准备好的辅核栈顶虚拟地址写入对应的 BOOT_RECORDS[id].stack_top。
    for id in 1..count() {
        // SAFETY:
        // 1. `id` 在 `1..count()` 范围内，对应有效的 `BOOT_RECORDS` 槽位。
        // 2. 所有辅核尚未启动，BSP 独占修改 `BOOT_RECORDS[id].stack_top`，而 `local` 字段保持不变。
        unsafe {
            ptr::addr_of_mut!(
                (*ptr::addr_of_mut!(BOOT_RECORDS).cast::<CpuBoot>().add(id)).stack_top
            )
            .write(get(id).stack_top());
        }
    }
    Ok(())
}

/// 获取当前正在执行的 CPU 的 [`CpuLocal`] 静态引用；若当前核尚未安装每核基址则立即 panic。
pub fn current() -> &'static CpuLocal {
    let local = crate::arch::cpu_local();
    // SAFETY:
    // 1. 未安装基址时架构层保证寄存器值为 0（空指针），`as_ref()` 会安全返回 `None` 并触发 `expect` panic。
    // 2. 一旦由 `install_cpu_local` 或辅核启动入口装入非空指针，该指针必然指向 `CPU_LOCALS` 中永久存活的 `CpuLocal` 实例，且陷入（Trap）前后的上下文保存/恢复保证其始终指向本核实例。
    unsafe { local.as_ref() }.expect("CPU local base has not been installed")
}

/// 返回启动阶段解析出的在线逻辑 CPU 总数。
pub fn count() -> usize {
    crate::boot::info().cpu_count
}

/// 返回包含所有在线逻辑 CPU（`0..count()`）的 [`CpuSet`] 位图集合。
pub fn actual_set() -> CpuSet {
    CpuSet::prefix(count())
}

/// 按逻辑 CPU 编号 `id`（`0..count()`）获取对应的 [`CpuLocal`] 静态引用。
pub fn get(id: usize) -> &'static CpuLocal {
    CPU_LOCALS[..count()][id]
        .get()
        .expect("CPU local has not been initialized")
}

/// 在本核完成本地中断控制器与时钟初始化并开启硬中断后，以 `Release` 顺序将 `online` 标志置为 `true`，通知正在等待的 BSP 本核已就绪。
pub fn publish_online() {
    let local = current();
    #[cfg(feature = "ktest")]
    {
        let stack_probe = 0u8;
        local
            .boot_stack_probe
            .store(ptr::addr_of!(stack_probe) as usize, Ordering::Relaxed);
        local
            .boot_local_probe
            .store(crate::arch::cpu_local() as usize, Ordering::Relaxed);
        local
            .boot_irq_ready
            .store(crate::arch::irq_enabled(), Ordering::Relaxed);
    }
    local.online.store(true, Ordering::Release);
}

/// 在辅核已上线但尚未进入调度器前，停留在永久启动栈上等待主核发布全局启动许可 `ready`。
///
/// 等待期间通过 `arch::idle_once` 进入低功耗状态并允许响应硬中断与 IPI（例如跨核 TLB 击落或同步回调），返回时保持原有的开中断状态。
/// 发布 `ready` 的主核必须先以 `Release` 顺序将 `ready` 置为 `true`，再向所有等待核发送 `kick`。
pub(crate) fn wait_until_started(ready: &AtomicBool) {
    let local = current();
    assert!(
        crate::arch::irq_enabled(),
        "CPU startup wait requires IRQ enabled"
    );
    assert!(
        local.online.load(Ordering::Acquire)
            && local.current_task_id.load(Ordering::Relaxed) == 0
            && local.hardirq_depth.load(Ordering::Relaxed) == 0
            && local.bh_depth.load(Ordering::Relaxed) == 0
            && local.preempt_depth.load(Ordering::Relaxed) == 0,
        "CPU startup wait requires online boot context"
    );

    while !ready.load(Ordering::Acquire) {
        let saved = crate::arch::irq_save();
        if !ready.load(Ordering::Acquire) {
            // SAFETY:
            // 1. 前置 `assert!` 确认本核已上线且运行在永久启动栈上，无普通任务、未持有任何锁且各项中断/禁抢占嵌套深度均为 0。
            // 2. `irq_save()` 关闭本地中断后没有任何依赖关中断状态的可变借用，允许 `arch::idle_once` 在硬件等待期间临时开启并响应硬中断/IPI。
            // 3. `local.kick_reasons`、`local.test_wait_completions` 与调用方传入的 `ready` 在整个等待期间均保持地址有效且仅通过原子操作访问。
            unsafe {
                #[cfg(feature = "ktest")]
                crate::arch::idle_once(&local.kick_reasons, ready, &local.test_wait_completions);
                #[cfg(not(feature = "ktest"))]
                crate::arch::idle_once(&local.kick_reasons, ready);
            }
        }
        // SAFETY: `saved` 来自同一循环迭代内本核调用的 `arch::irq_save()`；期间未发生栈切换或跨核迁移，且无任何临界区借用跨越中断状态恢复。
        unsafe { crate::arch::irq_restore(saved) };
    }
}

/// 由主核（BSP）串行唤醒所有辅核（`1..count()`），并在每个辅核启动后等待其发布 `online` 标志（单核超时时间为 5 秒）。
///
/// 若某个辅核启动调用失败或等待上线超时，则在 panic 前仅回收尚未尝试启动的后续辅核（`id + 1..count()`）内核栈。
///
/// # Safety
/// 1. `CPU_LOCALS`、所有辅核专属内核栈（`prepare_secondary_stacks`）以及 `BOOT_RECORDS` 必须均已准备就绪，且对应的虚拟与物理地址映射永久有效。
/// 2. 当前必须由主核（BSP）独占调用，所有辅核尚未启动；一旦将某个辅核的启动记录物理地址交给固件（`arch::start_cpu`），无论成功或超时，都不得再修改该辅核的 `BOOT_RECORDS` 或回收该辅核正在使用的栈。
/// 3. 等待辅核上线期间，若辅核会向主核发起跨核同步调用（`cpu::call`），主核必须能够响应 IPI 且不得持有辅核初始化所需的锁。
pub unsafe fn start_secondaries() {
    const AP_ONLINE_TIMEOUT_SECS: u64 = 5;

    for id in 1..count() {
        let local = get(id);
        // Step 1: 计算目标辅核启动记录 `BOOT_RECORDS[id]` 的物理地址并请求底层固件/架构接口唤醒该辅核。
        // SAFETY: `id` 位于已初始化的 `1..count()` 范围内，`BOOT_RECORDS[id]` 静态存在且已在 `prepare_secondary_stacks` 中填好栈顶。
        let record = unsafe { ptr::addr_of!(BOOT_RECORDS).cast::<CpuBoot>().add(id) as usize };
        let physical = crate::arch::virt_to_phys(record);
        // SAFETY: 当前核为唯一启动协调者（BSP），`physical` 是 `BOOT_RECORDS[id]` 的真实物理地址，其指向的专属栈与 `CpuLocal` 均已就绪且永久有效。
        if let Err(error) = unsafe { crate::arch::start_cpu(local.hardware_id, physical) } {
            // SAFETY: 辅核按 `1..count()` 串行启动，当前仅尝试到 `id`，因此 `id + 1..count()` 的记录和栈从未交给固件或任何辅核使用。
            unsafe { release_unstarted_suffix(id + 1) };
            panic!(
                "AP start failed: hardware={} error={}",
                local.hardware_id, error
            );
        }

        // Step 2: 在 5 秒超时窗口内自旋等待目标辅核完成早期初始化并置位 `local.online`。
        let deadline = crate::arch::read_counter()
            + crate::boot::info().counter_frequency * AP_ONLINE_TIMEOUT_SECS;
        while !local.online.load(Ordering::Acquire) {
            if deadline <= crate::arch::read_counter() {
                // SAFETY: 超时的第 `id` 号辅核可能已经在固件中开始使用其栈，因此绝不回收 `id` 的栈，仅回收从未尝试启动的后缀 `id + 1..count()`。
                unsafe { release_unstarted_suffix(id + 1) };
                panic!("AP online timed out: hardware={}", local.hardware_id);
            }
            spin_loop();
        }
    }
}

/// 清空从未尝试启动的辅核后缀（`first..count()`）在 `BOOT_RECORDS` 中的栈顶记录，并将它们预分配的物理页归还给页分配器。
///
/// # Safety
/// 1. 只能由独占辅核启动流程的主核（BSP）调用。
/// 2. 逻辑编号在 `first..count()` 范围内的所有辅核必须从未调用过 `arch::start_cpu`，不存在任何固件或 CPU 正在或即将访问这些记录与内核栈。
unsafe fn release_unstarted_suffix(first: usize) {
    for id in first..count() {
        // SAFETY: 调用方保证 `first..count()` 内的启动记录从未交给固件且由 BSP 独占访问；先清空 `stack_top` 再释放底层物理页。
        unsafe {
            ptr::addr_of_mut!(
                (*ptr::addr_of_mut!(BOOT_RECORDS).cast::<CpuBoot>().add(id)).stack_top
            )
            .write(0);
        }
        // SAFETY: `id` 属于从未启动过的辅核后缀，上方已将 `BOOT_RECORDS[id].stack_top` 清零，没有任何并发访问者使用该栈。
        unsafe { get(id).stack.release_unstarted() };
    }
}

/// 本地硬件定时器硬中断入口：递增本核定时器中断计数、若当前有普通任务运行则置位 `need_resched` 抢占标志，并推进时钟子系统与软件定时器（不在中断栈上直接切换任务）。
pub fn handle_timer() {
    let local = current();
    assert!(
        0 < local.hardirq_depth.load(Ordering::Relaxed),
        "timer handler requires hard IRQ context"
    );
    local.timer_interrupts.fetch_add(1, Ordering::Relaxed);
    if local.current_task_id.load(Ordering::Relaxed) != 0 {
        local.need_resched.store(true, Ordering::Release);
    }
    crate::time::handle_timer();
    #[cfg(feature = "ktest")]
    crate::mm::ktest::timer_irq();
}

/// 内核自测专用的 RAII 守卫：标记当前 CPU 正在执行 `cpu::call` 同步回调，并在析构时恢复嵌套前的原状态。
#[cfg(feature = "ktest")]
struct CallbackGuard {
    /// 当前正在执行回调的 CPU 状态块引用。
    local: &'static CpuLocal,
    /// 进入本次回调前 `ktest_callback_active` 的旧值（支持嵌套恢复）。
    previous: bool,
    /// 标记本守卫不可跨线程/跨核传递（`!Send` / `!Sync`）。
    _local: PhantomData<*mut ()>,
}

#[cfg(feature = "ktest")]
impl CallbackGuard {
    /// 在本地硬中断已关闭的上下文中进入同步回调执行段，将本核 `ktest_callback_active` 置为 `true`。
    fn enter() -> Self {
        assert!(!crate::arch::irq_enabled());
        let local = current();
        Self {
            previous: local.ktest_callback_active.swap(true, Ordering::Relaxed),
            local,
            _local: PhantomData,
        }
    }
}

#[cfg(feature = "ktest")]
impl Drop for CallbackGuard {
    /// 退出同步回调执行段，将本核 `ktest_callback_active` 恢复为进入前的旧值。
    fn drop(&mut self) {
        self.local
            .ktest_callback_active
            .store(self.previous, Ordering::Relaxed);
    }
}

/// 核间中断（IPI）处理函数：在底层硬件已清除 IPI 挂起位后，原子取走并处理 `kick_reasons` 软件原因位，随后排空执行本核 `calls` 队列中的跨核同步调用（CSD）。
pub fn handle_ipi() {
    let local = current();
    local.ipi_interrupts.fetch_add(1, Ordering::Relaxed);

    // Step 1: 原子地交换清零并取出当前累积的所有软件通知原因位（`KICK_SCHED_WAKE` 在此即被消费，无需置位 `need_resched`）。
    // 若在后续处理期间远端 CPU 再次调用 `kick`，新原因位会重新写入 `kick_reasons` 并伴随新一轮硬件 IPI。
    let reasons = local.kick_reasons.swap(0, Ordering::AcqRel);
    #[cfg(feature = "ktest")]
    test_pause_boundary(&local.test_ipi_pause);
    if reasons & KICK_RESCHEDULE != 0 {
        local.need_resched.store(true, Ordering::Release);
    }
    if reasons & KICK_TIMER_REPROGRAM != 0 {
        crate::time::reprogram_current();
        #[cfg(feature = "ktest")]
        local.test_timer_reprograms.fetch_add(1, Ordering::Release);
    }
    if reasons & KICK_LOG_PRINTER != 0 {
        crate::klog::wake_printer();
    }

    // Step 2: 循环从本核 `calls` 队列中弹出跨核同步调用请求，在自旋锁外执行回调并以 Release 顺序置位 `done` 唤醒发送核。
    loop {
        // SAFETY: `local.calls.lock()` 持有保护该队列的自旋锁，且所有入队节点在 `done` 置为 `true` 前均固定在发送方栈上保持有效。
        let request = unsafe { local.calls.lock().pop() };
        let request = match request {
            Some(request) => request,
            None => break,
        };
        #[cfg(feature = "ktest")]
        let _callback = CallbackGuard::enter();
        // SAFETY:
        // 1. `request` 刚刚从队列弹出且其 `done` 仍为 `false`，发起 `cpu::call` 的发送核仍在自旋等待，因此 `*request` 节点及 `argument` 保持有效。
        // 2. `done.store(true, Ordering::Release)` 是本核对该节点的最后一次访问，确保回调的所有内存写入对发送核 `Acquire` 轮询可见。
        unsafe {
            ((*request).callback)((*request).argument);
            (*request).done.store(true, Ordering::Release);
        }
    }

    // Step 3: 若收到永久停机请求（`KICK_STOP`），在排空所有跨核回调后通知底层架构停止当前 CPU。
    if reasons & KICK_STOP != 0 {
        crate::arch::stop_current_cpu(&local.stopped);
    }
}

/// 在目标逻辑 CPU `target` 上同步执行一次回调函数 `callback(argument)`，并阻塞等待其执行完毕后返回。
///
/// 若 `target` 即为当前 CPU，则在本地关中断（`IrqSaveGuard`）保护下直接执行回调；若为远端 CPU，则在当前栈帧上固定 `Call` 节点入队并发送 IPI，自旋等待远端完成。
///
/// # Safety
/// 1. `target` 必须是在线逻辑 CPU 范围内的有效编号（`0..count()`），且目标核已上线能够响应 IPI。
/// 2. `callback` 与 `argument` 指向的数据必须在 `call` 返回前始终保持有效，且对 `argument` 的并发访问满足线程安全要求。
/// 3. `callback` 在中断关闭上下文（IPI 或本地 `IrqSaveGuard`）中运行，严禁睡眠、阻塞、访问用户态内存或递归发起可能造成环路等待的跨核同步调用。
/// 4. 当向远端核发起调用时，调用方当前必须能够响应本地 IPI（本地硬中断开启），且不得持有目标 `callback` 执行所需的任何锁，否则会导致死锁。
pub unsafe fn call(target: usize, callback: unsafe fn(*mut ()), argument: *mut ()) {
    // Step 1: 若目标即为当前 CPU，直接在本地关中断临界区内同步执行回调并返回。
    if target == current().id {
        let _irq = IrqSaveGuard::new();
        #[cfg(feature = "ktest")]
        let _callback = CallbackGuard::enter();
        // SAFETY: 调用方契约保证 `callback` 与 `argument` 有效，且 `_irq` 保证了本核关中断执行环境。
        unsafe { callback(argument) };
        return;
    }

    // Step 2: 在当前栈帧上构造并固定（pin）请求节点 `Call`，将其推入目标核的 `calls` 队列并发送硬件 IPI，随后自旋等待 `done` 变为 `true`。
    let local = get(target);
    let pinned = core::pin::pin!(Call {
        callback,
        argument,
        next: UnsafeCell::new(ptr::null_mut()),
        done: AtomicBool::new(false),
    });
    let request = pinned.as_ref().get_ref();
    let node = request as *const Call as *mut Call;
    // SAFETY:
    // 1. `node` 指向通过 `pin!` 固定在当前栈帧上的 `Call` 实例，且下方 `while !request.done.load(Acquire)` 循环保证在目标核完成并置位 `done` 前本函数绝不返回。
    // 2. `local.calls.lock()` 独占持有目标核队列自旋锁。
    unsafe { local.calls.lock().push(node) };
    crate::arch::send_ipi(local.hardware_id);
    while !request.done.load(Ordering::Acquire) {
        spin_loop();
    }
}

/// 遍历并统计目标逻辑 CPU 的 `calls` 队列中尚未被弹出的跨核调用请求数量（供内核自测观测并发交错使用）。
#[cfg(feature = "ktest")]
pub fn pending_calls(target: usize) -> usize {
    let queue = get(target).calls.lock();
    let mut node = queue.head;
    let mut count = 0;
    while !node.is_null() {
        count += 1;
        // SAFETY: 当前持有 `get(target).calls` 自旋锁，且所有在队列中的节点均尚未置位 `done`，其发送方栈帧保证存活。
        node = unsafe { *(*node).next.get() };
    }
    count
}

/// 向目标逻辑 CPU `target` 发布软件原因位 `reason`（`KICK_*` 掩码），若目标核已上线则立即向其发送硬件 IPI（异步通知，不等待目标核处理）。
pub(crate) fn kick(target: usize, reason: usize) {
    assert!(
        reason != 0 && reason & !kick_mask() == 0,
        "invalid CPU kick reason"
    );
    let local = get(target);
    local.kick_reasons.fetch_or(reason, Ordering::Release);
    if local.online.load(Ordering::Acquire) {
        crate::arch::send_ipi(local.hardware_id);
    }
}

/// 返回所有合法 `KICK_*` 软件通知原因位的按位或掩码。
fn kick_mask() -> usize {
    KICK_RESCHEDULE | KICK_TIMER_REPROGRAM | KICK_LOG_PRINTER | KICK_STOP | KICK_SCHED_WAKE
}

/// 在系统关机或重启前，由当前协调核向所有其他在线 CPU 发送 `KICK_STOP` 停机请求并自旋等待它们全部确认停机，返回固定当前协调核的 [`crate::sync::PreemptGuard`]。
///
/// 调用时本核必须开启硬中断、处于普通任务或主流程上下文（不在硬中断或底半部中），且不得持有任何可能阻碍其他核处理 IPI 退出的锁。
#[cfg(not(feature = "ktest"))]
pub(crate) fn stop_other_online_cpus() -> crate::sync::PreemptGuard {
    assert!(crate::arch::irq_enabled(), "CPU stop requires IRQ enabled");
    let guard = crate::sync::PreemptGuard::new();
    let coordinator = current();
    assert!(
        coordinator.online.load(Ordering::Acquire),
        "CPU stop coordinator is offline"
    );
    assert_eq!(
        coordinator.hardirq_depth.load(Ordering::Relaxed),
        0,
        "CPU stop in hard IRQ"
    );
    assert_eq!(
        coordinator.bh_depth.load(Ordering::Relaxed),
        0,
        "CPU stop in bottom half"
    );

    // Step 1: 收集除当前协调核以外所有已上线的逻辑 CPU 集合。
    let mut targets = CpuSet::empty();
    for id in 0..count() {
        let local = get(id);
        if id != coordinator.id && local.online.load(Ordering::Acquire) {
            assert!(
                !local.stopped.load(Ordering::Acquire),
                "CPU stop acknowledgement reused"
            );
            targets.insert(id);
        }
    }

    // Step 2: 向所有目标 CPU 发送 KICK_STOP 核间中断请求。
    for id in targets.iter() {
        kick(id, KICK_STOP);
    }

    // Step 3: 自旋等待所有目标 CPU 在各自的 IPI 处理末尾置位 `stopped` 确认标志。
    for id in targets.iter() {
        while !get(id).stopped.load(Ordering::Acquire) {
            spin_loop();
        }
    }

    crate::early_println!(
        "CPU stop complete: coordinator={} acknowledged={:?}",
        coordinator.id,
        targets
    );
    guard
}

/// 硬中断（Hard IRQ）上下文嵌套深度 RAII 守卫：进入时递增本核 `hardirq_depth` 并更新第一层中断记账，析构时递减深度。
pub struct HardIrqGuard {
    /// 当前正在处理硬中断的 CPU 状态块引用。
    local: &'static CpuLocal,
    /// 标记本守卫不可跨线程/跨核传递（`!Send` / `!Sync`）。
    _local: PhantomData<*mut ()>,
}

impl HardIrqGuard {
    /// 在本地硬中断已关闭的陷阱入口处进入硬中断上下文，递增 `hardirq_depth`（若为最外层硬中断则通知 `accounting::irq_enter`）。
    pub fn enter() -> Self {
        assert!(
            !crate::arch::irq_enabled(),
            "hard IRQ entry has IRQ enabled"
        );
        let local = current();
        increment_depth(&local.hardirq_depth);
        if local.hardirq_depth.load(Ordering::Relaxed) == 1 {
            crate::accounting::irq_enter();
        }
        Self {
            local,
            _local: PhantomData,
        }
    }
}

impl Drop for HardIrqGuard {
    /// 退出本次硬中断上下文，递减本核 `hardirq_depth`（若退出最外层硬中断则通知 `accounting::irq_exit`，不在中断返回路径直接调用调度器）。
    fn drop(&mut self) {
        assert!(!crate::arch::irq_enabled(), "hard IRQ exit has IRQ enabled");
        if self.local.hardirq_depth.load(Ordering::Relaxed) == 1 {
            crate::accounting::irq_exit();
        }
        decrement_depth(&self.local.hardirq_depth);
    }
}

/// 软中断/底半部（Bottom Half）执行深度 RAII 守卫：持守卫期间允许响应硬中断，但禁止发生普通任务抢占或睡眠切换。
pub struct BottomHalfGuard {
    /// 当前正在执行底半部的 CPU 状态块引用。
    local: &'static CpuLocal,
    /// 标记本守卫不可跨线程/跨核传递（`!Send` / `!Sync`）。
    _local: PhantomData<*mut ()>,
}

impl BottomHalfGuard {
    /// 进入底半部上下文，在关中断保护下递增本核 `bh_depth`。
    pub fn enter() -> Self {
        let _irq = IrqSaveGuard::new();
        let local = current();
        increment_depth(&local.bh_depth);
        Self {
            local,
            _local: PhantomData,
        }
    }
}

impl Drop for BottomHalfGuard {
    /// 退出底半部上下文，在关中断保护下递减本核 `bh_depth`。
    fn drop(&mut self) {
        let _irq = IrqSaveGuard::new();
        decrement_depth(&self.local.bh_depth);
    }
}

/// 将指定的原子嵌套深度计数器加 1；若发生算术溢出则立即 panic。
pub(crate) fn increment_depth(depth: &AtomicUsize) {
    depth
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
            value.checked_add(1)
        })
        .expect("execution depth overflow");
}

/// 将指定的原子嵌套深度计数器减 1；若计数器已为 0（发生下溢）则立即 panic。
pub(crate) fn decrement_depth(depth: &AtomicUsize) {
    depth
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
            value.checked_sub(1)
        })
        .expect("execution depth underflow");
}

/// 内核自测可武装的确定性并发停点位置枚举。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) enum TestPause {
    /// 调度器发布 `idle = true` 并释放就绪队列锁之后、进入最终检查与 `WFI`/`IDLE` 之前的边界。
    Idle,
    /// `handle_ipi` 取走 `kick_reasons` 之后、开始处理原因位与 `calls` 队列之前的边界。
    Ipi,
}

/// 获取目标逻辑 CPU `target` 在指定停点位置 `boundary` 对应的原子状态机引用。
#[cfg(feature = "ktest")]
fn test_pause_state(target: usize, boundary: TestPause) -> &'static AtomicUsize {
    let local = get(target);
    match boundary {
        TestPause::Idle => &local.test_idle_pause,
        TestPause::Ipi => &local.test_ipi_pause,
    }
}

/// 为目标 CPU `target` 的指定边界 `boundary` 武装一次性的测试停点（将状态从 `TEST_PAUSE_IDLE` 切换为 `TEST_PAUSE_ARMED`）。
#[cfg(feature = "ktest")]
pub(crate) fn test_arm_pause(target: usize, boundary: TestPause) {
    test_pause_state(target, boundary)
        .compare_exchange(
            TEST_PAUSE_IDLE,
            TEST_PAUSE_ARMED,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .expect("CPU test pause already armed");
}

/// 检查目标 CPU `target` 是否已经到达并停留在指定边界 `boundary`（状态为 `TEST_PAUSE_REACHED`）。
#[cfg(feature = "ktest")]
pub(crate) fn test_is_paused(target: usize, boundary: TestPause) -> bool {
    test_pause_state(target, boundary).load(Ordering::Acquire) == TEST_PAUSE_REACHED
}

/// 放行已停留在指定边界 `boundary` 的目标 CPU `target`（将状态从 `TEST_PAUSE_REACHED` 切换为 `TEST_PAUSE_RELEASED`）。
#[cfg(feature = "ktest")]
pub(crate) fn test_release_pause(target: usize, boundary: TestPause) {
    test_pause_state(target, boundary)
        .compare_exchange(
            TEST_PAUSE_REACHED,
            TEST_PAUSE_RELEASED,
            Ordering::Release,
            Ordering::Relaxed,
        )
        .expect("CPU test pause has not been reached");
}

/// 取消尚未触发的测试停点或直接放行已暂停的目标 CPU（供测试清理路径安全复位停点状态机）。
#[cfg(feature = "ktest")]
pub(crate) fn test_cancel_pause(target: usize, boundary: TestPause) {
    let state = test_pause_state(target, boundary);
    loop {
        let observed = state.load(Ordering::Acquire);
        let replacement = match observed {
            TEST_PAUSE_IDLE | TEST_PAUSE_RELEASED => return,
            TEST_PAUSE_ARMED => TEST_PAUSE_IDLE,
            TEST_PAUSE_REACHED => TEST_PAUSE_RELEASED,
            _ => panic!("CPU test pause has invalid state"),
        };
        if state
            .compare_exchange(observed, replacement, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            return;
        }
    }
}

/// 若当前边界停点已被武装（`TEST_PAUSE_ARMED`），则将其切换为 `TEST_PAUSE_REACHED` 并在关中断无锁状态下自旋等待控制核放行（最长等待 5 秒后超时 panic）。
#[cfg(feature = "ktest")]
fn test_pause_boundary(state: &AtomicUsize) {
    const TIMEOUT: crate::time::Duration = crate::time::Duration::from_secs(5);

    if state
        .compare_exchange(
            TEST_PAUSE_ARMED,
            TEST_PAUSE_REACHED,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_err()
    {
        return;
    }
    assert!(
        !crate::arch::irq_enabled(),
        "CPU test pause requires IRQ off"
    );
    let deadline = crate::time::now()
        .checked_add(TIMEOUT)
        .expect("CPU test pause deadline overflows");
    while state.load(Ordering::Acquire) != TEST_PAUSE_RELEASED {
        assert!(crate::time::now() < deadline, "CPU test pause timed out");
        spin_loop();
    }
    state.store(TEST_PAUSE_IDLE, Ordering::Release);
}

/// 在调度器发布 `idle = true` 并释放队列锁后、进入最终唤醒原因复查前触发的内核自测停点钩子。
#[cfg(feature = "ktest")]
pub(crate) fn test_idle_boundary() {
    test_pause_boundary(&current().test_idle_pause);
}

/// 读取目标逻辑 CPU `target` 当前尚未消费的 `kick_reasons` 软件原因位掩码（供内核自测使用）。
#[cfg(feature = "ktest")]
pub(crate) fn test_pending_kicks(target: usize) -> usize {
    get(target).kick_reasons.load(Ordering::Acquire)
}

/// 读取目标逻辑 CPU `target` 在 IPI 中实际执行定时器重编程（`KICK_TIMER_REPROGRAM`）的累计次数（供内核自测使用）。
#[cfg(feature = "ktest")]
pub(crate) fn test_timer_reprograms(target: usize) -> usize {
    get(target).test_timer_reprograms.load(Ordering::Acquire)
}

/// 读取目标逻辑 CPU `target` 调用 `idle_once` 进入低功耗等待入口的累计次数（供内核自测使用）。
#[cfg(feature = "ktest")]
pub(crate) fn test_idle_waits(target: usize) -> usize {
    get(target).test_idle_waits.load(Ordering::Acquire)
}

/// 读取指定逻辑 CPU `owner` 实际执行硬件 `WFI`/`IDLE` 指令并被唤醒返回的累计次数（供内核自测使用）。
#[cfg(feature = "ktest")]
pub(crate) fn test_wait_completions(owner: usize) -> usize {
    get(owner).test_wait_completions.load(Ordering::Acquire)
}
