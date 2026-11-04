//! 捕获定位随真实 Task 迁核；子任务登记和报告空间在 OOM 主体之前预备。
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

const MIN_CHILDREN: usize = 64;
// 每核参与者之外保留一个协调 Task；普通非逐核用例仍有原来的最小登记预算。
const CHILDREN: usize = if MIN_CHILDREN < crate::boot::MAX_CPUS + 1 {
    crate::boot::MAX_CPUS + 1
} else {
    MIN_CHILDREN
};
const NO_CASE: u32 = u32::MAX;
static BOOT_ENABLED: AtomicBool = AtomicBool::new(false);
static BOOT_BOUNDARY: AtomicPtr<Boundary> = AtomicPtr::new(ptr::null_mut());
static BOOT_CASE: AtomicU32 = AtomicU32::new(NO_CASE);
static BOOT_ARMED: AtomicBool = AtomicBool::new(false);

/// 只能在 BSP 的受控 Boot 调用链安装，无 MM、Task 或堆依赖。
pub(crate) struct BootGuard {
    _local: PhantomData<*mut ()>,
}
impl BootGuard {
    /// 拒绝第二执行者共享 BSP 的固定捕获槽。
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
    fn drop(&mut self) {
        if !BOOT_BOUNDARY.load(Ordering::Relaxed).is_null() {
            fatal("Boot boundary survived scope");
        }
        BOOT_CASE.store(NO_CASE, Ordering::Relaxed);
        BOOT_ENABLED.store(false, Ordering::Release);
    }
}

/// 无锁定位只在 IRQ-off 且不切换的窄访问中借用活登记 Task。
fn with_task<T>(body: impl FnOnce(Option<&crate::task::Task>) -> T) -> T {
    let _irq = IrqSaveGuard::new();
    // SAFETY: 当前 CPU 独占 current_task 的更新；关 IRQ 且本回调不能切换，活登记保活非空指针。
    let pointer = unsafe { *crate::cpu::current().current_task.get() };
    // SAFETY: 非空对象由活登记拥有直到离开 Running，借用不逃逸 IRQ-off 回调。
    body(unsafe { pointer.as_ref() })
}
/// 硬 IRQ、BH/IPI/CSD 和无登记 worker 不得捕获被打断的 Task 边界。
fn boundary_allowed() -> bool {
    let local = crate::cpu::current();
    local.hardirq_depth.load(Ordering::Relaxed) == 0
        && local.bh_depth.load(Ordering::Relaxed) == 0
        && !local.ktest_callback_active.load(Ordering::Relaxed)
}
/// 低层 panic 仅读取原槽，不取调度锁、不分配、不遍历。
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
/// 替换当前执行者唯一边界；返回外层原槽。
/// # Safety
/// new 非空时必须指向当前栈中固定且存活的 Boundary；调用者逆序恢复，不跨执行者移交。
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
/// 应急故障定位允许 IRQ 控制者读取被打断测试身份，但绝不允许从 IRQ 展开。
pub(crate) fn active_case() -> Option<u32> {
    let id = with_task(|task| {
        if let Some(task) = task {
            let pointer = task.ktest_scope.load(Ordering::Acquire).cast::<ArcScope>();
            if pointer.is_null() {
                NO_CASE
            } else {
                // SAFETY: 安装者和子入口持 Arc，Task 活登记保活，IRQ-off 期间不能撤槽。
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
/// 仅表示当前 body 已完成 armed 声明，真实控制者仍必须独立确定故障类型。
pub(crate) fn fatal_armed() -> bool {
    with_task(|task| {
        if let Some(task) = task {
            let pointer = task.ktest_scope.load(Ordering::Acquire).cast::<ArcScope>();
            // SAFETY: 当前 Task 的安装 guard/子入口保活，IRQ-off 回调期间不能撤槽。
            !pointer.is_null() && unsafe { (*pointer).armed.load(Ordering::Acquire) }
        } else {
            crate::cpu::current().id == 0
                && BOOT_ENABLED.load(Ordering::Acquire)
                && BOOT_ARMED.load(Ordering::Acquire)
        }
    })
}
/// 展开内部不变量故障按实际原因归类，不读取 armed 期望。
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

struct Children {
    closed: bool,
    slots: [Option<crate::task::JoinHandle>; CHILDREN],
}
/// 固定容量、实际拥有的子任务作用域；关闭后不再发表任务。
pub(crate) struct ArcScope {
    case: AtomicU32,
    stop: AtomicBool,
    armed: AtomicBool,
    joined: AtomicBool,
    children: SpinLock<Children>,
    failure: SpinLock<Option<Failure>>,
}
impl ArcScope {
    /// 所有报告/停止/登记空间预先一次分配，之后收敛不申请存储。
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
    /// 合作停止同时关闭新子任务发表；既有任务保留真实 join 责任。
    fn stop(&self) {
        self.children.lock().closed = true;
        self.stop.store(true, Ordering::Release);
    }
    /// 锁外逐个等待真实离栈；丢观察 handle 不影响这个拥有关系。
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
    /// 第一个子失败独立保留，不能满足父 body 的 should_panic。
    fn record(&self, failure: Failure) {
        let mut slot = self.failure.lock();
        if slot.is_none() {
            *slot = Some(failure);
        }
    }
}
impl Drop for ArcScope {
    /// 丢失 join 责任是框架不变量故障，不能隐式释放仍被执行者使用的对象。
    fn drop(&mut self) {
        if self.children.lock().slots.iter().any(Option::is_some) {
            fatal("dropping test scope before all children joined");
        }
    }
}

/// 原 Task 的窄槽安装 guard，不拥有也不移动 Task。
pub(crate) struct ScopeGuard<'a> {
    scope: &'a Arc<ArcScope>,
    previous: *mut (),
    boundary: *mut (),
    _local: PhantomData<*mut ()>,
}
impl<'a> ScopeGuard<'a> {
    /// Arc 的稳定分配和本地 borrow 保活到恢复原槽以后。
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

/// 子任务只有合作停止和稳定身份，不借用父 fixture 或上下文。
#[derive(Clone)]
pub(crate) struct ChildToken {
    scope: Arc<ArcScope>,
}
impl ChildToken {
    /// 主体必须在真实工作边界检查停止请求。
    pub(crate) fn stop_requested(&self) -> bool {
        self.scope.stop.load(Ordering::Acquire)
    }
}
/// 观察 handle 只保留原完成状态；丢弃不影响父 scope 的唯一 join 责任。
pub(crate) struct ChildHandle {
    observer: crate::task::JoinObserver,
}
impl ChildHandle {
    /// Some 表示原任务已经离栈，值始终来自原 JoinState。
    pub(crate) fn try_join(&self) -> Option<i32> {
        self.observer.try_join()
    }
    /// 返回原任务的稳定身份，不保活任务或执行栈。
    pub(crate) fn task_id(&self) -> crate::task::TaskId {
        self.observer.task_id()
    }
    /// 普通测试任务锁外等待真实离栈；子失败仍独立保留在父账本。
    pub(crate) fn join(&self) -> Result<i32, TestError> {
        if !boundary_allowed() {
            return Err(TestError("child join outside ordinary test Task"));
        }
        Ok(self.observer.join())
    }
}
/// 从当前 Thread 测试作用域发表无返回值的真实固定 CPU Task。
pub(crate) fn spawn_on(
    cpu: usize,
    body: impl FnOnce(ChildToken) -> TestResult + Send + 'static,
) -> Result<ChildHandle, TestError> {
    spawn_result_on(cpu, move |token| body(token).map(|()| 0))
}

/// 发表带真实退出值的子任务，作用域仍拥有唯一原 JoinHandle。
pub(crate) fn spawn_result_on(
    cpu: usize,
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<ChildHandle, TestError> {
    prepare_result_on(cpu, body)?.publish()
}

/// 发表由原调度器选核的子任务，保留原 Any/SMP 与退出结果。
pub(crate) fn spawn_result(
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<ChildHandle, TestError> {
    prepare_result(body)?.publish()
}

/// 预备原 Any 子任务以便同批发表，不引入测试固定核策略。
pub(crate) fn prepare_result(
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<PreparedChild, TestError> {
    prepare_child(crate::task::Placement::Any, body)
}

/// 只读当前 Thread scope 的真实子失败账本，供释放依赖 gate 之前选择失败清理。
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
        // SAFETY: ScopeGuard 在原 Task 槽保活该 Arc；IRQ-off 回调不切换，仅读原短锁账本。
        unsafe { &*pointer }.failure.lock().is_some()
    })
}

/// 预备而不发表子任务；失败或取消时未发表入口在短锁外析构。
pub(crate) fn prepare_result_on(
    cpu: usize,
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<PreparedChild, TestError> {
    if cpu >= crate::cpu::count() || !crate::cpu::get(cpu).online.load(Ordering::Acquire) {
        return Err(TestError("child CPU is not online"));
    }
    prepare_child(crate::task::Placement::Cpu(cpu), body)
}

/// 两种正式 Placement 共享作用域准入、锁外预备与捕获。
fn prepare_child(
    placement: crate::task::Placement,
    body: impl FnOnce(ChildToken) -> Result<i32, TestError> + Send + 'static,
) -> Result<PreparedChild, TestError> {
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
        // SAFETY: ScopeGuard/child 的 Arc 在撤槽前保活；IRQ-off 不切换，新增强引用不分配。
        Some(unsafe {
            Arc::increment_strong_count(pointer);
            Arc::from_raw(pointer)
        })
    })
    .ok_or(TestError("spawn requires Thread test scope"))?;
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
    // 已登记的原调用 Task 保住预备尾部；未发表入口不能开始执行，失败析构允许睡眠。
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
    let prepared = match placement {
        crate::task::Placement::Any => crate::task::prepare_kernel(entry),
        crate::task::Placement::Cpu(cpu) => crate::task::prepare_kernel_on(cpu, entry),
    }
    .map_err(|_| TestError("NoMemory: child Task"))?;
    Ok(PreparedChild { scope, prepared })
}

/// 独占未发表 Task；只有成功 publish 才移交 scope 的 join 责任。
pub(crate) struct PreparedChild {
    scope: Arc<ArcScope>,
    prepared: crate::task::PreparedTask,
}
impl PreparedChild {
    /// 在 scope 发表之前关联真实用户拥有，仍由原子任务捕获和 join 协议负责退出。
    pub(crate) fn bind_user(&self, user: Arc<crate::task::UserThread>) {
        self.prepared.bind_user(user);
    }

    /// 同一短锁线性化 stop、容量检查与真实发表；拒绝对象在锁外归还。
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

/// 一个实例的服务，Boot 无堆；Thread 的 scope 预先分配并由 Task 携带。
pub(crate) struct RuntimeServices<'a> {
    pub(crate) case: &'static CaseDesc,
    pub(crate) id: u32,
    pub(crate) scope: Option<&'a Arc<ArcScope>>,
    phase: core::cell::Cell<Option<Phase>>,
    finished: core::cell::Cell<bool>,
    suite_cleanup: core::cell::Cell<bool>,
    suite: &'static str,
}
impl<'a> RuntimeServices<'a> {
    /// 每个作用域保留 suite 名；只有 NO_CASE 哨兵属于 suite 自身。
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
    /// Boot 固定 case 身份在进入实例时更新，Thread 身份已在各自 ArcScope 中。
    pub(crate) fn activate(&self) {
        if self.scope.is_none() {
            BOOT_CASE.store(self.id, Ordering::Release);
            BOOT_ARMED.store(false, Ordering::Release);
        }
    }
    /// runner 已输出 setup 时同步服务本地阶段，避免重复控制行。
    pub(crate) fn mark_setup(&self) {
        self.phase.set(Some(Phase::Setup));
    }
    /// typed dispatch 返回且真实 join 完成后才发布普通终态；abort 永不返回到这里。
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
            // Join 返回而没有 Cleanup 表示 setup 未构造 fixture；没有活对象需要 tear_down。
            Some(Phase::Join) => self.phase(Phase::Cleanup),
            Some(Phase::Cleanup) => {}
            _ => fatal("case final before lifecycle completed"),
        }
        self.finished.set(true);
        report::result(ordinal, self.case, outcome, None);
    }
}
impl Services for RuntimeServices<'_> {
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
    fn diagnostic(&self, message: core::fmt::Arguments<'_>) {
        report::diagnostic(message);
    }
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
        } else {
            if let Some(scope) = self.scope {
                scope.armed.store(false, Ordering::Release);
            } else {
                BOOT_ARMED.store(false, Ordering::Release);
            }
        }
    }
    fn stop_children(&self) {
        if let Some(scope) = self.scope {
            scope.stop();
        }
    }
    fn join_children(&self) -> Result<JoinOutcome, TestError> {
        self.scope
            .map_or(Ok(JoinOutcome { failure: None }), |scope| scope.join())
    }
    fn terminate(&self, outcome: &CaseOutcome) -> ! {
        if self.finished.replace(true) {
            fatal("duplicate case termination");
        }
        let phase = match self.phase.get() {
            Some(phase @ (Phase::Quiesce | Phase::Join | Phase::Cleanup)) => phase,
            _ => fatal("termination outside cleanup phase"),
        };
        let mut preserved = *outcome;
        // quiesce/join 失败时 API 尚未取得 JoinOutcome；只读已有账本，绝不擅自 join 活 fixture。
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
