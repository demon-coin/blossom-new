//! 测试执行上下文绑定、子任务作用域管理与运行时生命周期服务（`RuntimeServices`）。
//!
//! # 模块职责与架构定位
//! 本模块负责在内核执行流中定位当前活跃的测试用例与 panic 捕获边界，并为测试主体提供多核子任务派生、
//! 协作停止、退出等待以及生命周期阶段上报服务。
//! - 上游调用者：`ktest::runner`（创建并驱动 `BootGuard`、`ArcScope` 与 `RuntimeServices`）、
//!   `ktest::panic`（查询与替换当前捕获边界 `Boundary`）以及各内核测试用例（调用 `spawn_on` 等 API）。
//! - 下游依赖：`crate::task`（内核线程预备、发布与 `JoinHandle` 等待）与 `ktest::report`（控制帧输出）。
//!
//! # 核心设计与状态机
//! ## 1. 双模式上下文定位
//! - **Boot 模式**：在调度器与内核堆启用前或单核受控阶段运行于 BSP（CPU 0），使用全局静态原子变量
//!   （`BOOT_ENABLED`、`BOOT_BOUNDARY`、`BOOT_CASE`、`BOOT_ARMED`）记录当前测试身份与捕获边界，全程零堆分配。
//! - **Thread 模式**：在多核调度器启用后运行于普通内核线程 `Task`，将 `ArcScope` 指针与栈上 `Boundary`
//!   指针直接保存在 `Task` 自身的 `ktest_scope` 和 `ktest_boundary` 原子槽位中。当线程在 CPU 间迁移时，
//!   测试身份与 panic 捕获边界随 `Task` 自动迁移。
//! - **异步上下文隔离**：硬中断（`hardirq_depth > 0`）、软中断/底半部（`bh_depth > 0`）以及未绑定测试作用域的
//!   异步回调（`ktest_callback_active`）严禁借用被打断线程的 `Boundary` 展开栈帧；但致命错误报告允许通过
//!   `active_case` 只读查询被打断线程所属的用例 ID。
//!
//! ## 2. 子任务作用域（`ArcScope`）与用例生命周期
//! ```text
//!   ArcScope::new (在进入可能触发 OOM 的测试主体前，一次性预分配固定 CHILDREN 槽位)
//!        |
//!        v
//!   Phase::Setup  --->  Phase::Body (若配置 expected_fatal，此时置位 armed 标志)
//!                           |
//!                           +---> prepare_child (锁外分配内核栈与任务结构)
//!                           +---> PreparedChild::publish (持 children 自旋锁登记槽位并唤醒)
//!                           v
//!   Phase::Quiesce : 调用 stop_children() 置位 closed 与 stop，拒绝新子任务发布并通知协作退出
//!        |
//!        v
//!   Phase::Join    : 调用 join_children() 逐槽取出 JoinHandle，在自旋锁外阻塞等待子线程彻底退出栈帧
//!        |
//!        v
//!   Phase::Cleanup : 执行 Fixture::tear_down 释放测试资源，最后调用 RuntimeServices::finish 输出结果
//! ```
//!
//! # 锁获取顺序与并发契约
//! - `with_task` 通过 `IrqSaveGuard` 关闭本地中断后读取 `cpu::current().current_task`；闭包内严禁触发调度切换。
//! - `ArcScope` 包含两把独立的自旋锁：`children: SpinLock<Children>` 与 `failure: SpinLock<Option<Failure>>`。
//!   两把锁严禁嵌套获取，且持锁临界区内严禁执行内存分配、任务休眠或 `JoinHandle::join`。

use super::{
    panic::{self, Boundary},
    report,
};
use crate::sync::{IrqSaveGuard, SpinLock};
use ::ktest::{
    CaseDesc, CaseOutcome, Caught, Failure, JoinOutcome, PanicMessage, Phase, Services, TestError,
    TestResult,
};
use alloc::sync::Arc;
use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};

/// 子任务槽位的最小默认容量。
const MIN_CHILDREN: usize = 64;
/// 每个测试作用域支持的最大并发子任务槽位数（至少为 `MAX_CPUS + 1`，以容纳每核一个工作线程外加一个协调线程）。
const CHILDREN: usize = if MIN_CHILDREN < crate::boot::MAX_CPUS + 1 {
    crate::boot::MAX_CPUS + 1
} else {
    MIN_CHILDREN
};

/// 表示当前不处于任何具体测试用例（例如处于测试套件级 `setup`/`cleanup` 阶段）的哨兵 ID。
const NO_CASE: u32 = u32::MAX;

/// 标记 BSP（CPU 0）当前是否已进入 Boot 测试上下文。
static BOOT_ENABLED: AtomicBool = AtomicBool::new(false);
/// Boot 测试上下文中当前生效的栈上 panic 捕获边界指针。
static BOOT_BOUNDARY: AtomicPtr<Boundary> = AtomicPtr::new(ptr::null_mut());
/// Boot 测试上下文中当前正在执行的测试用例 ID（无用例时为 [`NO_CASE`]）。
static BOOT_CASE: AtomicU32 = AtomicU32::new(NO_CASE);
/// 标记 Boot 测试上下文当前是否处于预期发生致命错误（`expected_fatal`）的测试主体阶段。
static BOOT_ARMED: AtomicBool = AtomicBool::new(false);

/// BSP（CPU 0）专用的 Boot 测试上下文守卫。
///
/// 仅允许在 BSP 的受控启动调用链中创建，不依赖内存管理子系统、内核堆或 `Task` 结构体。
pub(crate) struct BootGuard {
    /// 阻止守卫实现 `Send`/`Sync`，确保其生命周期严格绑定在当前执行流栈帧。
    _local: PhantomData<*mut ()>,
}

impl BootGuard {
    /// 进入 Boot 测试上下文并独占 BSP 的全局捕获槽位。
    ///
    /// 若当前 CPU 不是 BSP（`id != 0`）或已有活跃的 `BootGuard`，将直接触发致命错误终止。
    pub(crate) fn enter() -> Self {
        if crate::cpu::current().id != 0 || BOOT_ENABLED.swap(true, Ordering::AcqRel) {
            fatal("invalid Boot boundary owner");
        }
        Self {
            _local: PhantomData,
        }
    }
}

impl Drop for BootGuard {
    /// 退出 Boot 测试上下文，校验栈上捕获边界已全部卸载并重置全局状态。
    fn drop(&mut self) {
        if !BOOT_BOUNDARY.load(Ordering::Relaxed).is_null() {
            fatal("Boot boundary survived scope");
        }
        BOOT_CASE.store(NO_CASE, Ordering::Relaxed);
        BOOT_ENABLED.store(false, Ordering::Release);
    }
}

/// 在关中断且禁止调度的临界区内借用当前 CPU 上正在运行的 `Task` 引用。
///
/// # 参数
/// - `body`：接收当前 `Option<&Task>` 的短期闭包，闭包内严禁触发上下文切换。
fn with_task<T>(body: impl FnOnce(Option<&crate::task::Task>) -> T) -> T {
    let _irq = IrqSaveGuard::new();
    // SAFETY:
    // 1. `_irq` 已关闭当前 CPU 本地中断，期间不会发生抢占或更新 `current_task`；
    // 2. `current_task` 仅由当前 CPU 在调度切换时写入，此时读取 `UnsafeCell` 无并发写冲突。
    let pointer = unsafe { *crate::cpu::current().current_task.get() };
    // SAFETY:
    // 1. 非空 `pointer` 指向的 `Task` 由运行队列或当前执行流持有强引用保活，在离开 Running 状态前不会释放；
    // 2. 借用生命周期严格限制在关中断且不发生调度的 `body` 闭包内部。
    body(unsafe { pointer.as_ref() })
}

/// 检查当前执行流是否处于允许安装或查询 panic 捕获边界的同步上下文。
///
/// 硬中断、软中断/底半部（BH/IPI/CSD）以及标记了 `ktest_callback_active` 的异步回调
/// 均不得借用被打断线程的捕获边界。
fn boundary_allowed() -> bool {
    let local = crate::cpu::current();
    local.hardirq_depth.load(Ordering::Relaxed) == 0
        && local.bh_depth.load(Ordering::Relaxed) == 0
        && !local.ktest_callback_active.load(Ordering::Relaxed)
}

/// 获取当前执行流栈顶安装的 panic 捕获边界指针。
///
/// 供底层 panic 展开路径调用，全程不获取调度锁、不分配内存。若当前处于异步中断上下文或未绑定测试作用域，返回空指针。
pub(crate) fn current_boundary() -> *mut Boundary {
    let _irq = IrqSaveGuard::new();
    if !boundary_allowed() {
        return ptr::null_mut();
    }
    with_task(|task| match task {
        Some(task) if !task.ktest_scope.load(Ordering::Relaxed).is_null() => {
            task.ktest_boundary.load(Ordering::Relaxed).cast()
        }
        Some(_) => ptr::null_mut(),
        None if crate::cpu::current().id == 0 && BOOT_ENABLED.load(Ordering::Relaxed) => {
            BOOT_BOUNDARY.load(Ordering::Relaxed)
        }
        None => ptr::null_mut(),
    })
}

/// 替换当前执行流的活跃 panic 捕获边界指针，并返回替换前的外层边界指针。
///
/// # Safety
/// 1. 当 `new` 非空时，必须指向当前执行栈帧中已固定（Pinned）且在对应 `catch` 作用域结束前持续有效的 [`Boundary`]；
/// 2. 调用方必须在退出捕获作用域时按严格的栈逆序（LIFO）将外层原指针恢复回去，且不得跨线程或跨 CPU 移交该边界。
pub(crate) unsafe fn replace_boundary(new: *mut Boundary) -> *mut Boundary {
    let _irq = IrqSaveGuard::new();
    if !boundary_allowed() {
        fatal("boundary installation in asynchronous context");
    }
    with_task(|task| match task {
        Some(task) if !task.ktest_scope.load(Ordering::Relaxed).is_null() => task
            .ktest_boundary
            .swap(new.cast(), Ordering::Relaxed)
            .cast(),
        None if crate::cpu::current().id == 0 && BOOT_ENABLED.load(Ordering::Relaxed) => {
            BOOT_BOUNDARY.swap(new, Ordering::Relaxed)
        }
        _ => fatal("boundary installation outside test owner"),
    })
}

/// 查询当前 CPU 上正在执行的测试用例 ID。
///
/// 供致命错误（如异常陷阱、OOM、未捕获 panic）报告路径定位故障归属的测试用例。
/// 允许在中断处理函数中读取被打断线程关联的用例 ID，但绝不允许从中断上下文发起栈展开。
pub(crate) fn active_case() -> Option<u32> {
    let id = with_task(|task| {
        if let Some(task) = task {
            let pointer = task.ktest_scope.load(Ordering::Acquire).cast::<ArcScope>();
            if pointer.is_null() {
                NO_CASE
            } else {
                // SAFETY:
                // 1. `ktest_scope` 由当前线程栈上的 `ScopeGuard` 安装，在 `ScopeGuard::drop` 撤下槽位前始终持有 `Arc<ArcScope>` 强引用；
                // 2. `with_task` 已关闭本地中断，当前线程不会在此次读取期间退出或销毁 `ScopeGuard`。
                unsafe { (*pointer).case.load(Ordering::Relaxed) }
            }
        } else if crate::cpu::current().id == 0 && BOOT_ENABLED.load(Ordering::Acquire) {
            BOOT_CASE.load(Ordering::Relaxed)
        } else {
            NO_CASE
        }
    });
    (id != NO_CASE).then_some(id)
}

/// 报告测试框架内部不变量违例并终止系统运行。
///
/// 根据故障原因字符串自动归类为 `"double-panic"`、`"abi"` 或 `"panic"` 致命事件。
pub(crate) fn fatal(reason: &'static str) -> ! {
    let kind = if reason.contains("double panic") || reason.contains("nested catch") {
        "double-panic"
    } else if reason.contains("ABI") {
        "abi"
    } else {
        "panic"
    };
    report::fatal_event(kind, format_args!("{}", reason))
}

/// 子任务句柄表，由 [`ArcScope::children`] 自旋锁保护。
struct Children {
    /// 是否已请求停止并关闭新子任务发布通道。
    closed: bool,
    /// 固定容量的子任务 [`crate::task::JoinHandle`] 槽位数组。
    slots: [Option<crate::task::JoinHandle>; CHILDREN],
}

/// Thread 测试用例的共享作用域状态。
///
/// 在测试用例启动前一次性完成堆分配，统一管理用例身份、协作停止标志、子任务生命周期以及首个子任务失败记录。
pub(crate) struct ArcScope {
    /// 当前正在执行的测试用例 ID（套件级阶段为 [`NO_CASE`]）。
    case: AtomicU32,
    /// 协作停止标志：置位后通知所有子任务通过 [`ChildToken::stop_requested`] 尽快退出循环。
    stop: AtomicBool,
    /// 标记当前用例是否正处于预期触发致命错误（`expected_fatal`）的 `Body` 阶段。
    armed: AtomicBool,
    /// 标记所有已发布的子任务是否均已完成 `join` 回收。
    joined: AtomicBool,
    /// 已发布子任务的句柄槽位表；临界区极短，不可睡眠。
    children: SpinLock<Children>,
    /// 记录首个失败或发生 panic 的子任务错误详情。
    failure: SpinLock<Option<Failure>>,
}

impl ArcScope {
    /// 创建并初始化指定用例 ID 的测试作用域。
    ///
    /// 所有状态与子任务登记槽位在此处一次性完成堆分配，使得后续即使进入测试 OOM 的主体阶段，
    /// 子任务协作停止、错误记录与 `join` 回收也无需再申请堆内存。
    pub(crate) fn new(id: u32) -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            case: AtomicU32::new(id),
            stop: AtomicBool::new(false),
            armed: AtomicBool::new(false),
            joined: AtomicBool::new(false),
            children: SpinLock::new(Children {
                closed: false,
                slots: [const { None }; CHILDREN],
            }),
            failure: SpinLock::new(None),
        })
        .map_err(|_| TestError("NoMemory: test scope"))
    }

    /// 发出协作停止信号并关闭子任务发布通道。
    ///
    /// 调用后禁止任何新的 [`PreparedChild::publish`]，已发布的子任务仍保留在槽位中等待后续 [`Self::join`] 回收。
    fn stop(&self) {
        self.children.lock().closed = true;
        self.stop.store(true, Ordering::Release);
    }

    /// 逐个等待作用域内所有已发布的子任务彻底退出内核栈，并返回汇总结果。
    ///
    /// 每次仅在自旋锁临界区内取出一个 [`crate::task::JoinHandle`]，随后释放自旋锁再阻塞等待线程退出。
    /// 即使测试用例代码中丢弃了 [`ChildHandle`]，父作用域依然通过此方法保证所有子任务在清理阶段前完全回收。
    fn join(&self) -> Result<JoinOutcome, TestError> {
        if !self.children.lock().closed {
            return Err(TestError("join without cooperative stop"));
        }

        for index in 0..CHILDREN {
            let handle = self.children.lock().slots[index].take();
            if let Some(handle) = handle {
                handle.join();
            }
        }
        self.joined.store(true, Ordering::Release);
        Ok(JoinOutcome {
            failure: *self.failure.lock(),
        })
    }

    /// 记录子任务发生的首次失败或 panic。
    ///
    /// 仅保留第一个到达的失败记录；子任务失败独立记录在此账本中，不会充当父线程的 `should_panic` 预期结果。
    fn record(&self, failure: Failure) {
        let mut slot = self.failure.lock();
        if slot.is_none() {
            *slot = Some(failure);
        }
    }
}

impl Drop for ArcScope {
    /// 析构时校验所有已发布的子任务均已通过 [`ArcScope::join`] 回收，防止残留后台线程访问已释放的测试资源。
    fn drop(&mut self) {
        if self.children.lock().slots.iter().any(Option::is_some) {
            fatal("dropping test scope before all children joined");
        }
    }
}

/// 将 [`ArcScope`] 绑定到当前 `Task` 测试槽位的守卫。
///
/// 仅修改当前 `Task` 的 `ktest_scope` 裸指针槽位，在析构时自动恢复原槽位值并校验 `ktest_boundary` 栈平衡。
pub(crate) struct ScopeGuard<'a> {
    /// 当前绑定到线程的测试作用域强引用借用，保证槽位存续期间 `ArcScope` 不会被释放。
    scope: &'a Arc<ArcScope>,
    /// 安装前 `Task::ktest_scope` 槽位中的旧指针值。
    previous: *mut (),
    /// 安装时 `Task::ktest_boundary` 槽位中的捕获边界指针快照，用于退出时校验边界未泄漏。
    boundary: *mut (),
    /// 阻止守卫跨线程传递，确保在同一个 `Task` 上安装与卸载。
    _local: PhantomData<*mut ()>,
}

impl<'a> ScopeGuard<'a> {
    /// 将 `scope` 安装到当前运行 `Task` 的 `ktest_scope` 槽位中。
    pub(crate) fn install(scope: &'a Arc<ArcScope>) -> Self {
        let (previous, boundary) = with_task(|task| {
            let task = task.unwrap_or_else(|| fatal("Thread scope outside Task"));
            (
                task.ktest_scope
                    .swap(Arc::as_ptr(scope).cast_mut().cast(), Ordering::AcqRel),
                task.ktest_boundary.load(Ordering::Relaxed),
            )
        });
        Self {
            scope,
            previous,
            boundary,
            _local: PhantomData,
        }
    }
}

impl Drop for ScopeGuard<'_> {
    /// 卸载当前 `Task` 的 `ktest_scope` 槽位并恢复先前保存的指针，同时断言捕获边界与所有者一致。
    fn drop(&mut self) {
        let pointer = Arc::as_ptr(self.scope).cast_mut().cast();
        with_task(|task| {
            let task = task.unwrap_or_else(|| fatal("scope removed outside Task"));
            if task.ktest_boundary.load(Ordering::Relaxed) != self.boundary {
                fatal("boundary survived Thread scope");
            }
            if task.ktest_scope.swap(self.previous, Ordering::AcqRel) != pointer {
                fatal("Thread scope owner mismatch");
            }
        });
    }
}

/// 传递给测试子任务的协作控制令牌。
///
/// 子任务通过该令牌查询父作用域是否已发出停止请求，无需直接借用父线程的测试夹具（Fixture）或上下文。
#[derive(Clone)]
pub(crate) struct ChildToken {
    /// 所属测试作用域的强引用。
    scope: Arc<ArcScope>,
}

impl ChildToken {
    /// 查询父测试作用域是否已发出协作停止请求（例如进入 `Quiesce` 阶段）。
    ///
    /// 长时间循环的子任务必须在每次迭代的工作边界检查此标志并及时退出。
    pub(crate) fn stop_requested(&self) -> bool {
        self.scope.stop.load(Ordering::Acquire)
    }
}

/// 测试子任务的观察与等待句柄。
///
/// 内部仅持有底层的 [`crate::task::JoinObserver`]，用于查询或等待子任务退出状态；
/// 即使测试主体直接丢弃此句柄，父 [`ArcScope`] 仍持有唯一的 [`crate::task::JoinHandle`] 并负责最终回收。
pub(crate) struct ChildHandle {
    /// 底层任务的完成状态观察器。
    observer: crate::task::JoinObserver,
}

impl ChildHandle {
    /// 非阻塞地轮询子任务是否已彻底退出内核栈。
    ///
    /// 若子任务已退出则返回 `Some(exit_code)`，否则返回 `None`。
    pub(crate) fn try_join(&self) -> Option<i32> {
        self.observer.try_join()
    }

    /// 获取子任务的全局唯一任务 ID（[`crate::task::TaskId`]）。
    pub(crate) fn task_id(&self) -> crate::task::TaskId {
        self.observer.task_id()
    }

    /// 在普通测试任务上下文中阻塞等待子任务彻底退出内核栈，并返回其退出码。
    ///
    /// 若子任务内部发生错误或 panic，错误详情仍保留在父作用域的失败账本中。
    pub(crate) fn join(&self) -> Result<i32, TestError> {
        if !boundary_allowed() {
            return Err(TestError("child join outside ordinary test Task"));
        }
        Ok(self.observer.join())
    }
}

/// 在当前 Thread 测试作用域下派生一个绑定到指定 `cpu` 的无返回值子任务。
pub(crate) fn spawn_on(
    cpu: usize,
    body: impl FnOnce(ChildToken) -> TestResult + Send + 'static,
) -> Result<ChildHandle, TestError> {
    spawn_result_on(cpu, move |token| body(token).map(|()| 0))
}

/// 在当前 Thread 测试作用域下派生一个绑定到指定 `cpu` 且带 `i32` 退出码的子任务。
pub(crate) fn spawn_result_on(
    cpu: usize,
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<ChildHandle, TestError> {
    prepare_result_on(cpu, body)?.publish()
}

/// 在当前 Thread 测试作用域下派生一个由调度器自由选核（`Placement::Any`）且带 `i32` 退出码的子任务。
pub(crate) fn spawn_result(
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<ChildHandle, TestError> {
    prepare_result(body)?.publish()
}

/// 预分配一个由调度器自由选核（`Placement::Any`）的子任务，暂不放入运行队列。
///
/// 适用于需要先批量完成多任务内存分配、再统一发布启动的并发测试场景。
pub(crate) fn prepare_result(
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<PreparedChild, TestError> {
    prepare_child(crate::task::Placement::Any, body)
}

/// 查询当前 Thread 测试作用域中是否已有子任务记录了失败或 panic。
///
/// 供测试夹具在释放依赖栅栏（Gate）或清理前置资源前判断是否存在子任务异常。
pub(crate) fn child_failed() -> bool {
    assert!(
        boundary_allowed(),
        "child failure query outside ordinary test Task"
    );
    with_task(|task| {
        let pointer = task
            .expect("child failure query outside Task")
            .ktest_scope
            .load(Ordering::Acquire)
            .cast::<ArcScope>();
        assert!(!pointer.is_null(), "child failure query outside test scope");
        // SAFETY:
        // 1. 当前线程栈上的 `ScopeGuard` 保证 `ktest_scope` 指向的 `ArcScope` 在整个测试期间有效；
        // 2. `with_task` 已关闭本地中断，临时共享引用不会逃逸出闭包，仅加锁读取 `failure` 状态。
        unsafe { &*pointer }.failure.lock().is_some()
    })
}

/// 预分配一个绑定到指定在线 `cpu` 的子任务，暂不放入运行队列。
///
/// 若目标 `cpu` 编号越界或尚未在线，返回 `Err(TestError)`；失败或取消时，已预备的资源在自旋锁外安全释放。
pub(crate) fn prepare_result_on(
    cpu: usize,
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<PreparedChild, TestError> {
    if cpu >= crate::cpu::count() || !crate::cpu::get(cpu).online.load(Ordering::Acquire) {
        return Err(TestError("child CPU is not online"));
    }
    prepare_child(crate::task::Placement::Cpu(cpu), body)
}

/// 按指定的 CPU 放置策略预分配测试子任务并包装异常捕获入口。
fn prepare_child(
    placement: crate::task::Placement,
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<PreparedChild, TestError> {
    // Step 1: 校验当前处于允许派生的同步测试任务上下文，并克隆父任务绑定的 ArcScope。
    if !boundary_allowed() {
        return Err(TestError("spawn outside ordinary test Task"));
    }

    let scope = with_task(|task| {
        let pointer = task
            .map_or(ptr::null_mut(), |task| {
                task.ktest_scope.load(Ordering::Acquire)
            })
            .cast::<ArcScope>();
        if pointer.is_null() {
            return None;
        }
        // SAFETY:
        // 1. 当前线程栈上的 `ScopeGuard` 持有 `Arc<ArcScope>`，在退出当前函数前引用计数至少为 1；
        // 2. `with_task` 关闭中断防止并发卸载槽位，`increment_strong_count` 与 `from_raw` 配对构造新的 `Arc` 实例。
        Some(unsafe {
            Arc::increment_strong_count(pointer);
            Arc::from_raw(pointer)
        })
    })
    .ok_or(TestError("spawn requires Thread test scope"))?;

    // Step 2: 预检作用域是否已关闭或槽位已满（最终发布时会在锁内再次复核）。
    {
        let slots = scope.children.lock();
        if slots.closed {
            drop(slots);
            return Err(TestError("child scope already stopped"));
        }
        if slots.slots.iter().all(Option::is_some) {
            drop(slots);
            return Err(TestError("child scope capacity exhausted"));
        }
    }

    // Step 3: 构造子任务入口闭包，自动为子线程安装 ScopeGuard 与 panic 捕获边界，并将错误或 panic 记入父作用域账本。
    let owned = Arc::clone(&scope);
    let entry = move || {
        let _installed = ScopeGuard::install(&owned);
        let token = ChildToken {
            scope: Arc::clone(&owned),
        };
        let caught = if cfg!(panic = "unwind") {
            panic::catch(|| body(token))
        } else {
            Ok(body(token))
        };
        match caught {
            Ok(Ok(value)) => value,
            Ok(Err(error)) => {
                owned.record(Failure::Returned(error));
                0
            }
            Err(record) => {
                report::diagnostic(format_args!("child panic: {:?}", record));
                owned.record(Failure::Panic(PanicMessage::new(
                    record.message(),
                    record.message_truncated(),
                )));
                0
            }
        }
    };

    // Step 4: 在自旋锁外分配内核线程栈与控制块（允许睡眠或返回 OOM 错误）。
    let prepared = match placement {
        crate::task::Placement::Any => crate::task::prepare_kernel(entry),
        crate::task::Placement::Cpu(cpu) => crate::task::prepare_kernel_on(cpu, entry),
    }
    .map_err(|_| TestError("NoMemory: child Task"))?;
    Ok(PreparedChild { scope, prepared })
}

/// 已完成资源预分配但尚未放入调度队列的测试子任务。
///
/// 独占持有底层 [`crate::task::PreparedTask`]；只有调用 [`Self::publish`] 成功后，
/// 才会将该子任务的 `join` 回收责任移交给父 [`ArcScope`]。
pub(crate) struct PreparedChild {
    /// 所属的父测试作用域。
    scope: Arc<ArcScope>,
    /// 底层已分配好内核栈与任务结构的预备线程对象。
    prepared: crate::task::PreparedTask,
}

impl PreparedChild {
    /// 在子任务发布运行前为其绑定用户态线程上下文（[`crate::task::UserThread`]）。
    pub(crate) fn bind_user(&self, user: Arc<crate::task::UserThread>) {
        self.prepared.bind_user(user);
    }

    /// 将预备子任务登记到父测试作用域的槽位表中并正式唤醒加入调度器。
    ///
    /// 在 `children` 自旋锁内原子地完成停止状态检查、空槽分配与任务发布；
    /// 若作用域已关闭或槽位已满，则在释放自旋锁后丢弃并析构未发布的任务对象。
    pub(crate) fn publish(self) -> Result<ChildHandle, TestError> {
        let mut slots = self.scope.children.lock();
        if slots.closed {
            drop(slots);
            return Err(TestError("child scope already stopped"));
        }
        let slot = match slots.slots.iter().position(Option::is_none) {
            Some(slot) => slot,
            None => {
                drop(slots);
                return Err(TestError("child scope capacity exhausted"));
            }
        };

        let handle = self.prepared.publish();
        let observer = handle.observer();
        slots.slots[slot] = Some(handle);
        drop(slots);
        Ok(ChildHandle { observer })
    }
}

/// 单个测试用例或套件级钩子（setup/cleanup）的运行时服务实现。
///
/// 实现 `::ktest::Services` trait，向测试框架驱动层提供异常捕获、诊断输出、生命周期阶段切换、
/// 子任务停止/回收以及失败终止上报功能。
pub(crate) struct RuntimeServices<'a> {
    /// 当前测试用例的静态元数据描述符。
    pub(crate) case: &'static CaseDesc,
    /// 当前测试用例的运行时编号（套件级钩子为 [`NO_CASE`]）。
    pub(crate) id: u32,
    /// Thread 模式下预分配的作用域引用；Boot 模式下为 `None`。
    pub(crate) scope: Option<&'a Arc<ArcScope>>,
    /// 当前已推进到的测试生命周期阶段。
    phase: core::cell::Cell<Option<Phase>>,
    /// 标记当前用例是否已输出最终结果或触发终止，防止重复结案。
    finished: core::cell::Cell<bool>,
    /// 当 `id == NO_CASE` 时，区分当前处于套件级 `cleanup`（`true`）还是 `setup`（`false`）。
    suite_cleanup: core::cell::Cell<bool>,
    /// 所属测试套件的名称。
    suite: &'static str,
}

impl<'a> RuntimeServices<'a> {
    /// 构造绑定到指定用例或套件钩子的 [`RuntimeServices`] 实例。
    pub(crate) fn new(
        case: &'static CaseDesc,
        id: u32,
        scope: Option<&'a Arc<ArcScope>>,
        suite: &'static str,
    ) -> Self {
        Self {
            case,
            id,
            scope,
            phase: core::cell::Cell::new(None),
            finished: core::cell::Cell::new(false),
            suite_cleanup: core::cell::Cell::new(false),
            suite,
        }
    }

    /// 激活当前测试用例上下文。
    ///
    /// Boot 模式下将用例 ID 写入全局 `BOOT_CASE` 并清除 `BOOT_ARMED`；
    /// Thread 模式下用例 ID 已保存在各自的 [`ArcScope`] 中。
    pub(crate) fn activate(&self) {
        if self.scope.is_none() {
            BOOT_CASE.store(self.id, Ordering::Release);
            BOOT_ARMED.store(false, Ordering::Release);
        }
    }

    /// 在驱动层已提前发出 `Setup` 控制行后，同步更新本地阶段状态以避免重复输出。
    pub(crate) fn mark_setup(&self) {
        self.phase.set(Some(Phase::Setup));
    }

    /// 在测试用例正常走完生命周期且所有子任务完成 `join` 后，输出最终测试结果帧。
    pub(crate) fn finish(&self, ordinal: usize, outcome: &CaseOutcome) {
        if self.id == NO_CASE || self.finished.get() {
            fatal("duplicate or suite case final");
        }
        if self
            .scope
            .is_some_and(|scope| !scope.joined.load(Ordering::Acquire))
        {
            fatal("case final before children joined");
        }
        match self.phase.get() {
            // 若停留在 Join 阶段而未进入 Cleanup，说明 setup 阶段失败未构造出 Fixture，补发 Cleanup 阶段以保持协议完整。
            Some(Phase::Join) => self.phase(Phase::Cleanup),
            Some(Phase::Cleanup) => {}
            _ => fatal("case final before lifecycle completed"),
        }
        self.finished.set(true);
        report::result(ordinal, self.case, outcome, None);
    }
}

impl Services for RuntimeServices<'_> {
    /// 执行测试阶段回调并捕获同步 Rust panic（在 `panic = "unwind"` 构建下）。
    fn catch(&self, callback: &mut dyn FnMut() -> TestResult) -> Caught {
        if cfg!(panic = "abort") {
            return Caught::Returned(callback());
        }
        match panic::catch(callback) {
            Ok(result) => Caught::Returned(result),
            Err(record) => {
                report::diagnostic(format_args!("panic: {:?}", record));
                Caught::Panicked(PanicMessage::new(
                    record.message(),
                    record.message_truncated(),
                ))
            }
        }
    }

    /// 输出测试作者通过 `TestContext::diagnostic` 提交的格式化诊断文本。
    fn diagnostic(&self, message: core::fmt::Arguments<'_>) {
        report::diagnostic(message);
    }

    /// 切换并上报当前测试生命周期阶段，同时管理 `expected_fatal` 武装状态。
    ///
    /// 仅在进入 `Phase::Body` 且用例声明了 `expected_fatal` 时开启武装标志（`armed`），
    /// 离开 `Phase::Body` 后立即解除武装，防止 `Setup`/`Cleanup` 阶段的意外故障被误判为符合预期。
    fn phase(&self, phase: Phase) {
        if self.finished.get() {
            fatal("case phase after final");
        }
        if self.id == NO_CASE {
            if phase == Phase::Quiesce {
                self.suite_cleanup.set(true);
            }
            self.phase.set(Some(phase));
            self.activate();
            return;
        }
        if self.phase.get() == Some(phase) {
            return;
        }
        report::phase(self.id, phase);
        self.phase.set(Some(phase));
        if phase == Phase::Body {
            if let Some(kind) = self.case.expected_fatal {
                report::armed(self.id, kind);
                if let Some(scope) = self.scope {
                    scope.armed.store(true, Ordering::Release);
                } else {
                    BOOT_ARMED.store(true, Ordering::Release);
                }
            }
        } else if let Some(scope) = self.scope {
            scope.armed.store(false, Ordering::Release);
        } else {
            BOOT_ARMED.store(false, Ordering::Release);
        }
    }

    /// 通知当前测试作用域内的所有子任务协作停止，并禁止继续派生新子任务。
    fn stop_children(&self) {
        if let Some(scope) = self.scope {
            scope.stop();
        }
    }

    /// 阻塞等待当前测试作用域内的所有子任务退出内核栈，并返回子任务失败汇总。
    fn join_children(&self) -> Result<JoinOutcome, TestError> {
        self.scope
            .map_or(Ok(JoinOutcome { failure: None }), |scope| scope.join())
    }

    /// 当清理阶段（`Quiesce`、`Join` 或 `Cleanup`）发生失败或无法恢复时，上报终止协议帧并停机。
    fn terminate(&self, outcome: &CaseOutcome) -> ! {
        if self.finished.replace(true) {
            fatal("duplicate case termination");
        }
        let phase = match self.phase.get() {
            Some(phase @ (Phase::Quiesce | Phase::Join | Phase::Cleanup)) => phase,
            _ => fatal("termination outside cleanup phase"),
        };
        let mut preserved = *outcome;
        // 若在 Quiesce 或 Join 阶段提前失败，上层尚未拿到 JoinOutcome；此处仅读取已记录的子任务失败账本，不强行阻塞等待未退出的子任务。
        if preserved.child.is_none() {
            if let Some(scope) = self.scope {
                preserved.child = *scope.failure.lock();
            }
        }
        if self.id == NO_CASE {
            let stage = if self.suite_cleanup.get() {
                "cleanup"
            } else {
                "setup"
            };
            report::suite_abort(self.suite, stage, &preserved);
        }
        report::case_abort(self.suite, self.id, phase, &preserved)
    }
}
