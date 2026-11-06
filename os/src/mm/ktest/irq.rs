//! 物理页分配器锁外清零窗口的硬件定时器中断（Timer IRQ）重入与跨核并发协同测试。
//!
//! ## 模块职责与架构定位
//! 本模块验证物理页分配器 `mm::alloc_pages(order, true)` 在退出底层分配器自旋锁与关中断守卫之后、
//! 执行整块内存清零期间的并发安全性：
//! - **上游挂钩入口**：
//!   1. `frame_allocator::alloc_pages` 在函数入口创建 [`AllocationWindow`]，并在摘出空闲块、释放内部分配锁后调用 `arm_timer()`；
//!   2. `cpu::handle_timer` 在硬件定时器中断上下文中调用 [`timer_irq`]。
//! - **下游协作模块**：复用 `mm::ktest::smp::SmpFixture` 管理多核测试任务，调用 `mm::alloc_pages`、
//!   `mm::drain_all_pages`、`arch::program_timer` 与 `time::reprogram_current`。
//!
//! ## 核心机制与同票（Ticket）窗口时序
//! 测试通过递增票号（`ticket`）将 CPU0 上的锁外清零窗口、本地硬件时钟中断处理函数以及从核（AP）并发释放/排空绑定在同一时间窗内：
//!
//! ```text
//! CPU0 (目标测试任务)                  CPU0 硬件 Timer IRQ             AP 1..N (远端工作任务)
//! ───────────────────                 ────────────────────            ──────────────────────
//! alloc_pages(MAX_ORDER, true)
//!   ├─ enter(): 领取 ticket = T, 置 window = true
//!   ├─ buddy::allocate() (取锁并释放锁，重开本地 IRQ)
//!   ├─ arm_timer():
//!   │    ├─ 向 target AP 发布 requests[ap] = T ─────────────────────► 唤醒：校验 window == true 且 ticket == T
//!   │    └─ program_timer(now + 1) ──► 触发硬中断进入 timer_irq()       ├─ 异核释放 SLUB 对象与零阶物理页
//!   ├─ write_bytes(0, len) (锁外清零)    ├─ 校验关中断与 hardirq_depth  ├─ 调用 drain_all_pages() 排空缓存
//!   │                                    ├─ 嵌套 alloc_pages(0) + drop  └─ 发布 completions[ap] = T
//!   │                                    └─ 记录 completed += 1               │
//!   └─ Drop(AllocationWindow):  ◄─────────────────────────────────────────────┘
//!        ├─ 自旋等待 completions[ap] == T，推进 target = ap + 1
//!        └─ 置 window = false，返回清零完毕的 OwnedPages
//! ```
//!
//! ## 锁获取顺序与生命周期安全契约
//! - **无锁原子协调**：中断路径（`timer_irq`）与窗口钩子（`AllocationWindow`）完全通过 `Control` 中的原子变量通信，
//!   在硬中断内绝不获取测试控制结构的自旋锁，避免与被中断的普通任务死锁。
//! - **`Control` 指针生命周期**：全局静态指针 `ACTIVE` 仅在 `IrqFixture::set_up` 中指向由夹具 `Arc<Control>` 保活的实例；
//!   在 `quiesce` 阶段先关闭 `armed` 标志并停止子任务，再通过同步跨核调用 `cpu::call(0, disarm, ...)` 在 CPU0 关中断临界区内清空 `ACTIVE`，
//!   确保任何正在执行的 CPU0 定时器中断处理函数彻底退出后，才在 `tear_down`（所有子任务已 `join` 完毕）中释放 `Arc<Control>`。

use crate::mm::ktest::smp::{SmpFixture, State, WORK};
use crate::{
    arch, cpu,
    mm::{self, buddy, pcp, OwnedPages},
    sync::IrqSaveGuard,
};
use alloc::sync::Arc;
use core::{
    hint::black_box,
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicUsize, Ordering},
};
use ktest::{Fixture, TestContext, TestError, TestResult};

/// 单次测试运行期间允许编程触发硬件定时器中断与窗口采样的最大轮数上限。
const BUDGET: usize = 16;
/// 系统支持的最大 CPU 核心槽位数（与 `boot::MAX_CPUS` 对齐）。
const MAX: usize = crate::boot::MAX_CPUS;
/// 指向当前激活的 [`Control`] 实例的全局原子裸指针；无活动测试时为空指针。
static ACTIVE: AtomicPtr<Control> = AtomicPtr::new(ptr::null_mut());

/// 协调锁外清零窗口、本地定时器中断重入以及跨核同步释放的共享控制块。
struct Control {
    /// 标记当前测试是否已武装窗口捕获（仅当为 `true` 时允许 `enter` 和 `timer_irq` 采样）。
    armed: AtomicBool,
    /// 允许触发窗口捕获的 CPU0 目标测试任务的任务 ID（`TaskId` 数值），排除后台无关任务干扰。
    owner: AtomicUsize,
    /// 标记 CPU0 目标任务当前是否正处于 `alloc_pages` 的锁外清零窗口内（防止重入嵌套重复领票）。
    window: AtomicBool,
    /// 当前已发出的窗口票号计数器（从 `1` 开始单调递增，每次成功进入窗口加 1）。
    ticket: AtomicUsize,
    /// 最近一次已在 `timer_irq` 中执行过分配探测的窗口票号，保证每个票号最多探测一次。
    irq_ticket: AtomicUsize,
    /// 下一次窗口打开时负责执行跨核并发释放/排空的从核 CPU 编号（单核模式或全部完成后等于 `cpu::count()`）。
    target: AtomicUsize,
    /// 在窗口内调用 `arch::program_timer` 编程硬件定时器的累计次数。
    programs: AtomicUsize,
    /// 硬中断成功命中有效窗口并进入分配器探测逻辑的累计次数。
    handlers: AtomicUsize,
    /// 硬中断命中目标任务活动清零窗口的累计次数。
    hits: AtomicUsize,
    /// 硬中断内完成嵌套 `alloc_pages(0, false)` 与释放且中断状态保持正确的成功次数。
    completed: AtomicUsize,
    /// 中断上下文不变量违例、分配失败或跨核等待超时等异步错误计数（避免在中断或 `Drop` 中直接 panic）。
    failures: AtomicUsize,
    /// 从核在同票清零窗口内完成的跨核 SLUB 对象释放累计次数。
    remote_frees: AtomicUsize,
    /// 从核在同票清零窗口内完成的跨核 `drain_all_pages` 累计次数。
    remote_drains: AtomicUsize,
    /// 按目标 CPU 下标索引的跨核任务唤醒请求槽，存放当前窗口票号（`0` 表示尚未请求）。
    requests: [AtomicUsize; MAX],
    /// 按目标 CPU 下标索引的跨核任务完成回执槽，从核完成后写入对应票号。
    completions: [AtomicUsize; MAX],
    /// 托管 CPU0 主循环刚分配出的连续物理页块，确保断言失败提前展开时仍由夹具安全回收物理页。
    live_pages: crate::sync::SpinLock<Option<OwnedPages>>,
    /// 底层多核测试夹具共享状态（提供停止标志 `stop`、同步屏障及跨核对象/页槽位）。
    state: Arc<State>,
}
impl Control {
    /// 创建初始未武装、各项窗口计数清零的中断重入测试控制块。
    fn new(state: Arc<State>) -> Self {
        Self {
            armed: AtomicBool::new(false),
            owner: AtomicUsize::new(0),
            window: AtomicBool::new(false),
            ticket: AtomicUsize::new(0),
            target: AtomicUsize::new(1),
            irq_ticket: AtomicUsize::new(0),
            programs: AtomicUsize::new(0),
            handlers: AtomicUsize::new(0),
            hits: AtomicUsize::new(0),
            completed: AtomicUsize::new(0),
            failures: AtomicUsize::new(0),
            remote_frees: AtomicUsize::new(0),
            remote_drains: AtomicUsize::new(0),
            requests: [const { AtomicUsize::new(0) }; MAX],
            completions: [const { AtomicUsize::new(0) }; MAX],
            live_pages: crate::sync::SpinLock::new(None),
            state,
        }
    }
}

/// 嵌入在 `mm::alloc_pages` 栈帧上的锁外清零窗口 RAII 守卫。
///
/// 仅当 CPU0 上的指定测试任务在开中断状态下进入 `alloc_pages` 时才会激活并领取非零票号；
/// 硬中断内部的嵌套 `alloc_pages` 调用因处于关中断状态而快速跳过。
pub(crate) struct AllocationWindow {
    /// 指向当前夹具保活的 [`Control`] 实例的只读指针；未激活时为空指针。
    control: *const Control,
    /// 本次 `alloc_pages` 调用分配到的非零窗口票号；`0` 表示当前调用未进入受测窗口。
    ticket: usize,
    /// 本次窗口同步唤醒的目标从核 CPU 编号；`0` 表示本轮无需等待从核回执。
    target: usize,
}
impl AllocationWindow {
    /// 在 `mm::alloc_pages` 入口处尝试为 CPU0 受测任务开启新的清零观察窗口。
    ///
    /// 调用上下文：任意调用 `alloc_pages` 的任务或中断上下文；非 CPU0、关中断（如硬中断内重入）、
    /// 非目标任务或已在窗口内时均立即返回 `ticket == 0` 的空守卫。
    pub(crate) fn enter() -> Self {
        let mut window = Self {
            control: ptr::null(),
            ticket: 0,
            target: 0,
        };
        if cpu::current().id != 0 || !arch::irq_enabled() {
            return window;
        }
        let pointer = ACTIVE.load(Ordering::Acquire);
        if pointer.is_null() {
            return window;
        }
        // SAFETY:
        // 1. `ACTIVE` 仅在 `IrqFixture::set_up` 中写入由 `IrqFixture::control`（`Arc<Control>`）保活的有效指针；
        // 2. 当前 CPU0 开启中断且若匹配 `control.owner` 则属于夹具子任务，在 `quiesce` 撤除 `ACTIVE` 及 `tear_down` 释放 `Arc<Control>` 前指针始终有效。
        let control = unsafe { &*pointer };
        if !control.armed.load(Ordering::Acquire)
            || crate::task::current_task_id().map(|id| id.as_usize())
                != Some(control.owner.load(Ordering::Acquire))
            || control.window.swap(true, Ordering::AcqRel)
        {
            return window;
        }
        window.control = pointer;
        window.ticket = control.ticket.fetch_add(1, Ordering::Relaxed) + 1;
        window
    }
    /// 在物理块已摘出、分配器自旋锁已全部释放且即将开始整块清零前，发布跨核请求并编程立即到期的本地定时器。
    ///
    /// 调用上下文：仅在普通可抢占且本地中断开启的任务上下文（`alloc_pages` 清零前）调用。
    pub(crate) fn arm_timer(&mut self) {
        if self.ticket == 0 {
            return;
        }
        // SAFETY:
        // 1. `self.ticket != 0` 仅由 `Self::enter` 在验证 `ACTIVE` 非空且当前任务为受测子任务后设置；
        // 2. 当前子任务栈帧尚未退出，夹具持有的 `Arc<Control>` 在所有子任务 `join` 之前始终有效。
        let control = unsafe { &*self.control };
        assert!(arch::irq_enabled());
        let target = control.target.load(Ordering::Acquire);
        if target < cpu::count() {
            self.target = target;
            assert!(0 < target);
            assert_eq!(control.requests[target].load(Ordering::Acquire), 0);
            control.requests[target].store(self.ticket, Ordering::Release);
        }
        if control.programs.fetch_add(1, Ordering::Relaxed) < BUDGET {
            arch::program_timer(arch::read_counter().wrapping_add(1));
        }
    }
}
impl Drop for AllocationWindow {
    /// 在 `alloc_pages` 返回前等待同票从核完成并发释放与排空，随后关闭当前清零窗口。
    ///
    /// 若遇到协作停止信号或等待超时，仅累加 `failures` 计数而不触发二次 panic。
    fn drop(&mut self) {
        if self.ticket == 0 {
            return;
        }
        // SAFETY:
        // 1. `self.ticket != 0` 保证 `self.control` 来自 `Self::enter` 捕获的有效 `Control` 地址；
        // 2. 当前受测子任务仍处于活动栈帧上，夹具在所有子任务 `join` 完毕前不会销毁 `Control`。
        let control = unsafe { &*self.control };
        if self.target != 0 {
            let end = arch::read_counter().wrapping_add(arch::counter_frequency() * 10);
            while control.completions[self.target].load(Ordering::Acquire) != self.ticket {
                if control.state.stop.load(Ordering::Acquire) || end <= arch::read_counter() {
                    control.failures.fetch_add(1, Ordering::Relaxed);
                    break;
                }
                core::hint::spin_loop();
            }
            control.target.store(self.target + 1, Ordering::Release);
        }
        control.window.store(false, Ordering::Release);
    }
}

/// 硬件定时器中断钩子：在 CPU0 清零窗口被中断时，从硬中断上下文嵌套调用零阶物理页分配与释放。
///
/// 调用上下文：仅由 `cpu::handle_timer` 在本地中断已硬件关闭且 `hardirq_depth > 0` 的硬中断上下文中调用。
pub(crate) fn timer_irq() {
    if cpu::current().id != 0 {
        return;
    }
    let pointer = ACTIVE.load(Ordering::Acquire);
    if pointer.is_null() {
        return;
    }
    // SAFETY:
    // 1. 本函数仅在 CPU0（上方已过滤 `cpu::current().id == 0`）的关中断硬中断上下文中执行；
    // 2. `IrqFixture::quiesce` 会通过同步跨核调用 `cpu::call(0, disarm, ...)` 在 CPU0 上清空 `ACTIVE`，该同步调用必等当前 CPU0 硬中断处理函数彻底执行完毕退出后才能运行，而 `Arc<Control>` 在更晚的 `tear_down` 中才会释放。
    let control = unsafe { &*pointer };
    if !control.armed.load(Ordering::Acquire)
        || crate::task::current_task_id().map(|id| id.as_usize())
            != Some(control.owner.load(Ordering::Acquire))
    {
        return;
    }
    if !control.window.load(Ordering::Acquire) {
        return;
    }
    // 系统时钟（timekeeper）可能在同一清零窗口内触发多次定时器中断；通过原子交换确保每个窗口票号最多执行一次嵌套分配探测。
    let ticket = control.ticket.load(Ordering::Acquire);
    if BUDGET < ticket || control.irq_ticket.swap(ticket, Ordering::Relaxed) == ticket {
        return;
    }
    control.handlers.fetch_add(1, Ordering::Relaxed);
    control.hits.fetch_add(1, Ordering::Relaxed);
    if arch::irq_enabled() || cpu::current().hardirq_depth.load(Ordering::Relaxed) == 0 {
        control.failures.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let Some(pages) = mm::alloc_pages(0, false) else {
        control.failures.fetch_add(1, Ordering::Relaxed);
        return;
    };
    drop(pages);
    if arch::irq_enabled() {
        control.failures.fetch_add(1, Ordering::Relaxed);
        return;
    }
    control.completed.fetch_add(1, Ordering::Release);
}

/// 管理硬件定时器中断重入测试生命周期与资源回收的测试夹具。
pub(crate) struct IrqFixture {
    /// 底层多核测试夹具，负责记录物理内存六域基线、管理子任务派生及共享页/对象槽位。
    memory: SmpFixture,
    /// 由夹具强引用保活的中断窗口控制块，在 `ACTIVE` 清空且所有子任务退出后释放。
    control: Arc<Control>,
}
impl Fixture for IrqFixture {
    /// 预先分配多核控制状态与 `Control` 控制块并注册至全局 `ACTIVE`（此时 `armed` 为 `false`，尚未拦截分配）。
    fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let memory = SmpFixture::set_up(context, &())?;
        let control = Arc::try_new(Control::new(Arc::clone(&memory.state)))
            .map_err(|_| TestError("NoMemory: IRQ memory controls"))?;
        let pointer = Arc::as_ptr(&control).cast_mut();
        ACTIVE
            .compare_exchange(
                ptr::null_mut(),
                pointer,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .map_err(|_| TestError("memory IRQ fixture already active"))?;
        Ok(Self { memory, control })
    }
    /// 关闭窗口拦截标志、通知多核子任务停止，并在 CPU0 上同步撤除全局钩子且恢复系统时钟下一次硬件定时事件。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.control.armed.store(false, Ordering::Release);
        self.memory.quiesce(context)?;
        // SAFETY:
        // 1. 启动主核 CPU0 始终在线，`disarm` 仅操作全局原子变量 `ACTIVE` 并重编程本核时钟，不解引用传入的空上下文指针；
        // 2. `cpu::call` 为同步屏障调用，保证返回时 CPU0 上此前任何正在运行的定时器硬中断均已彻底执行完毕退出，且 `ACTIVE` 已清空。
        unsafe { cpu::call(0, disarm, ptr::null_mut()) };
        Ok(())
    }
    /// 在测试框架已完成全部子任务 `join` 后，校验无残留活动窗口、释放托管页与控制块，并核对物理内存六域统计完全恢复基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        if self.control.window.load(Ordering::Acquire) {
            return Err(TestError("allocator window retained after join"));
        }
        let pages = self.control.live_pages.lock().take();
        drop(pages);
        let Self { memory, control } = self;
        drop(control);
        memory.tear_down(context)
    }
}

/// 在 CPU0 关中断回调上下文中清空全局 `ACTIVE` 观察指针，并按系统时钟队列重新编程当前 CPU 的最近到期硬件定时器。
///
/// # Safety
/// 1. 必须作为同步 `cpu::call(0, ...)` 的回调函数在在线的 CPU0 上调用，且调用期间本地硬件中断已关闭；
/// 2. 内核时间子系统（`crate::time`）必须已完成初始化。
unsafe fn disarm(_: *mut ()) {
    ACTIVE.store(ptr::null_mut(), Ordering::Release);
    crate::time::reprogram_current();
}

/// 物理页分配锁外清零窗口的本地定时器中断重入与跨核同票并发操作测试套件。
#[ktest::suite(fixture = IrqFixture, tags("mm", "timer", "irq"), timeout_ms = 30000)]
mod reentry {
    use super::*;
    /// 验证单核场景下，硬件定时器中断打断 `alloc_pages` 锁外清零窗口并在硬中断中安全完成嵌套零阶页分配与释放。
    ///
    /// - **测试目标**：确认 `alloc_pages` 在清零大块内存前已完全释放分配器自旋锁并重开中断，硬中断重入分配器不会导致死锁或中断状态错乱。
    /// - **场景步骤**：在 CPU0 上启动受测任务循环执行 `BUDGET` 次最大阶清零页分配（`alloc_pages(MAX_ORDER, true)`），每次进入清零窗口时编程定时器触发本地硬中断，在 `timer_irq` 中嵌套执行 `alloc_pages(0, false)` 并立即释放。
    /// - **预期结果**：硬中断成功命中清零窗口，嵌套分配与释放零失败完成，分配返回的页内容已清零，中断使能状态恢复正常。
    #[test]
    fn local_timer(_: &mut TestContext<'_>, f: &mut IrqFixture) -> TestResult {
        run(f, false)
    }
    /// 验证多核场景下，CPU0 锁外清零窗口同时承受本地定时器硬中断重入与从核（AP）跨核 SLUB 释放、零阶页释放及全核 PCP 排空。
    ///
    /// - **测试目标**：检验 CPU0 大块清零、硬中断嵌套分配与远端核心并发释放/排空在同一时间窗口交错执行时的锁安全与内存账本一致性。
    /// - **场景步骤**：预先在 CPU0 为每个从核准备 1 个 SLUB 对象与 1 张零阶页；CPU0 循环执行最大阶清零分配并在窗口内按票号依次唤醒各个从核，从核校验窗口处于打开状态后执行跨核释放与 `drain_all_pages()` 并回填完成票号，CPU0 在退出该次 `alloc_pages` 前确认从核已完成同票操作。
    /// - **预期结果**：本地硬中断重入探测与所有从核的同票释放、排空均在对应 `alloc_pages` 返回前无错误完成，退出后物理内存六域统计完全回归基线。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_same_ticket(_: &mut TestContext<'_>, f: &mut IrqFixture) -> TestResult {
        run(f, true)
    }
    /// 编排本地定时器中断重入测试主流程；当 `remote` 为 `true` 时额外派生从核任务在同票窗口内执行跨核释放与 PCP 排空。
    fn run(f: &mut IrqFixture, remote: bool) -> TestResult {
        let participants = if remote { cpu::count() - 1 } else { 0 };
        // Step 1: 若启用跨核模式，在每个从核（CPU 1..=participants）派生工作任务等待窗口票号通知。
        for id in 1..=participants {
            let control = Arc::clone(&f.control);
            f.memory.spawn(id, move |state, token| {
                state.wait(&token, || control.requests[id].load(Ordering::Acquire) != 0)?;
                let ticket = control.requests[id].load(Ordering::Acquire);
                assert!(control.window.load(Ordering::Acquire));
                assert_eq!(control.ticket.load(Ordering::Acquire), ticket);
                state.release_object(id - 1);
                control.remote_frees.fetch_add(1, Ordering::Relaxed);
                let pages = state.pages[id * WORK].lock().take().unwrap();
                drop(pages);
                assert!(0 < pcp::cpu_cached_pages(id));
                mm::drain_all_pages();
                assert_eq!(pcp::cpu_cached_pages(id), 0);
                control.remote_drains.fetch_add(1, Ordering::Relaxed);
                control.completions[id].store(ticket, Ordering::Release);
                state.done[id].store(1, Ordering::Release);
                state.park(&token)
            })?;
        }
        // Step 2: 在 CPU0 派生主受测任务，准备跨核资源、武装窗口钩子并循环执行最大阶清零分配。
        let control = Arc::clone(&f.control);
        f.memory.spawn(0, move |state, token| {
            state.wait(&token, || state.phase.load(Ordering::Acquire) == 1)?;
            for id in 1..=participants {
                state.allocate_object(id - 1);
                let pages = mm::alloc_pages(0, false).unwrap();
                *state.pages[id * WORK].lock() = Some(pages);
            }
            control.owner.store(
                crate::task::current_task_id().unwrap().as_usize(),
                Ordering::Release,
            );
            control
                .target
                .store(if remote { 1 } else { cpu::count() }, Ordering::Release);
            control.armed.store(true, Ordering::Release);
            for _ in 0..BUDGET {
                let pages = black_box(mm::alloc_pages(buddy::MAX_ORDER, true)).unwrap();
                *control.live_pages.lock() = Some(pages);
                let mut slot = control.live_pages.lock();
                let pages = slot.as_mut().unwrap();
                // SAFETY:
                // 1. 上方 `mm::alloc_pages(buddy::MAX_ORDER, true)` 已将整块连续物理页清零初始化；
                // 2. 当前持有 `control.live_pages` 自旋锁且独占该 `OwnedPages` 实例，读取首字节期间无任何并发写者且内存保持存活。
                assert_eq!(
                    black_box(unsafe { pages.as_mut_ptr().cast::<u8>().read() }),
                    0
                );
                let pages = slot.take();
                drop(slot);
                drop(pages);
            }
            // Step 3: 解除窗口武装，在关中断保护下恢复系统时钟硬件定时器，并逐项校验中断命中率与跨核完成票号。
            control.armed.store(false, Ordering::Release);
            let _irq = IrqSaveGuard::new();
            crate::time::reprogram_current();
            drop(_irq);
            assert!(!control.window.load(Ordering::Acquire));
            assert_eq!(control.failures.load(Ordering::Acquire), 0);
            let hits = control.hits.load(Ordering::Acquire);
            let handlers = control.handlers.load(Ordering::Acquire);
            assert!(0 < hits && hits <= handlers && handlers <= BUDGET);
            assert_eq!(hits, control.completed.load(Ordering::Acquire));
            assert_eq!(control.programs.load(Ordering::Acquire), BUDGET);
            assert_eq!(control.remote_frees.load(Ordering::Acquire), participants);
            assert_eq!(control.remote_drains.load(Ordering::Acquire), participants);
            let mut previous = 0;
            for id in 1..=participants {
                let requested = control.requests[id].load(Ordering::Acquire);
                assert!(previous < requested && requested <= BUDGET);
                assert_eq!(control.completions[id].load(Ordering::Acquire), requested);
                previous = requested;
            }
            assert_eq!(control.target.load(Ordering::Acquire), cpu::count());
            state.done[0].store(1, Ordering::Release);
            state.park(&token)
        })?;
        // Step 4: 等待全部参与任务就绪后统一放行第 1 阶段，并阻塞等待所有任务执行完毕。
        f.memory
            .until(|| f.memory.state.ready.load(Ordering::Acquire) == participants + 1)?;
        f.memory.state.phase.store(1, Ordering::Release);
        f.memory.until(|| {
            f.memory.state.done[..participants + 1]
                .iter()
                .all(|done| done.load(Ordering::Acquire) == 1)
        })
    }
}
