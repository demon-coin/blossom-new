//! 固定在线 CPU 的执行基础；请求节点由同步调用者拥有，完成前不离开其栈。

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

pub const STACK_SIZE: usize = 64 * 1024;

pub(crate) const KICK_RESCHEDULE: usize = 1 << 0;
pub(crate) const KICK_TIMER_REPROGRAM: usize = 1 << 1;
pub(crate) const KICK_LOG_PRINTER: usize = 1 << 2;
const KICK_STOP: usize = 1 << 3;
/// 仅唤醒 idle 核重查 ready 队列，不向可能已经运行的新任务请求抢占。
pub(crate) const KICK_SCHED_WAKE: usize = 1 << 4;

#[cfg(feature = "ktest")]
const TEST_PAUSE_IDLE: usize = 0;
#[cfg(feature = "ktest")]
const TEST_PAUSE_ARMED: usize = 1;
#[cfg(feature = "ktest")]
const TEST_PAUSE_REACHED: usize = 2;
#[cfg(feature = "ktest")]
const TEST_PAUSE_RELEASED: usize = 3;

/// 链接期保留的主核启动栈，不依赖页分配器。
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub struct BootStack(pub [u8; STACK_SIZE]);

/// 主核入口使用的唯一静态栈。
#[no_mangle]
#[link_section = ".boot.stacks"]
#[used]
pub static mut BOOT_STACK: BootStack = BootStack([0; STACK_SIZE]);

/// AP 使用的不可变启动记录；记录、独占栈和 CpuLocal 在内核生命周期内保留。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CpuBoot {
    pub stack_top: usize,
    pub local: usize,
}

struct Call {
    callback: unsafe fn(*mut ()),
    argument: *mut (),
    next: UnsafeCell<*mut Call>,
    done: AtomicBool,
}

struct CallQueue {
    head: *mut Call,
    tail: *mut Call,
}

// SAFETY: 队列指针仅在短锁内访问；调用者固定节点地址并等到 release 完成才结束节点生命周期。
unsafe impl Send for CallQueue {}

impl CallQueue {
    /// 创建尚无请求的队列，头尾在同一锁域内维护。
    const fn new() -> Self {
        Self {
            head: ptr::null_mut(),
            tail: ptr::null_mut(),
        }
    }

    /// 发布一个固定地址的请求，由目标 CPU 负责取出并完成。
    ///
    /// # Safety
    /// 调用者持有所属队列锁，call 未在其他队列中且在完成确认前保持有效；
    /// 队列中已有节点也受同一生命周期协议约束。
    unsafe fn push(&mut self, call: *mut Call) {
        if self.tail.is_null() {
            self.head = call;
        } else {
            // SAFETY: 非空尾节点仍在本队列中；持锁独占其 next，发送者尚未收到完成。
            unsafe { *(*self.tail).next.get() = call };
        }
        self.tail = call;
    }

    /// 在锁域中移除队首，返回后由调用者在锁外执行动作。
    ///
    /// # Safety
    /// 调用者持有所属队列锁，所有节点保持有效直到各自的完成确认。
    unsafe fn pop(&mut self) -> Option<*mut Call> {
        let call = self.head;
        if call.is_null() {
            return None;
        }
        // SAFETY: 非空队首由 push 发布且尚未完成；锁保护 next 的读取与头尾更新。
        self.head = unsafe { *(*call).next.get() };
        if self.head.is_null() {
            self.tail = ptr::null_mut();
        }
        Some(call)
    }
}

/// 一个 CPU 的不可变身份和真正需要同步的执行状态。
pub struct CpuLocal {
    pub id: usize,
    pub hardware_id: usize,
    stack: StackSlot,
    pub online: AtomicBool,
    pub timer_interrupts: AtomicUsize,
    pub ipi_interrupts: AtomicUsize,
    pub(crate) scheduler_ctx: UnsafeCell<TaskContext>,
    pub(crate) current_task: UnsafeCell<*const Task>,
    pub(crate) current_task_id: AtomicUsize,
    pub(crate) active_mm: UnsafeCell<Option<Arc<crate::mm::space::Mm>>>,
    pub(crate) hardirq_depth: AtomicUsize,
    pub(crate) bh_depth: AtomicUsize,
    pub(crate) preempt_depth: AtomicUsize,
    pub(crate) accounting: crate::accounting::CpuAccount,
    pub(crate) time_event: crate::time::LocalEvent,
    pub(crate) timer_base: crate::timer::TimerBase,
    pub(crate) worker_pool: crate::workqueue::WorkerPool,
    pub(crate) need_resched: AtomicBool,
    pub(crate) idle: AtomicBool,
    kick_reasons: AtomicUsize,
    /// 永久停机的完成确认；只发布一次，不作为 CPU 恢复或 online 的替代状态。
    stopped: AtomicBool,
    #[cfg(feature = "ktest")]
    test_idle_pause: AtomicUsize,
    #[cfg(feature = "ktest")]
    test_ipi_pause: AtomicUsize,
    #[cfg(feature = "ktest")]
    test_idle_waits: AtomicUsize,
    #[cfg(feature = "ktest")]
    test_wait_completions: AtomicUsize,
    #[cfg(feature = "ktest")]
    test_timer_reprograms: AtomicUsize,
    calls: SpinLock<CallQueue>,
    #[cfg(feature = "ktest")]
    pub(crate) boot_stack_probe: AtomicUsize,
    #[cfg(feature = "ktest")]
    pub(crate) boot_local_probe: AtomicUsize,
    #[cfg(feature = "ktest")]
    pub(crate) boot_irq_ready: AtomicBool,
    #[cfg(feature = "ktest")]
    pub(crate) ktest_callback_active: AtomicBool,
}

// SAFETY: scheduler_ctx/current_task 仅由所属 CPU 在 IRQ 关闭的调度交接中访问；
// active_mm 仅由所属 CPU 的 IRQ-off 交接修改，旧根失效后移出，最后拥有在锁外释放。
// 其余共享字段为原子或短锁，远端 CPU 不读写所属 CPU 的私有现场。
unsafe impl Sync for CpuLocal {}

// SAFETY: CPU 对象只在尚未安装基址、未执行调度时移动到永久 Once 存储；
// 发布后的静态对象不再移动，现场访问限制由所属 CPU 与 IRQ-off 协议保持。
unsafe impl Send for CpuLocal {}

impl CpuLocal {
    /// 构造 CPU 身份和调度状态；主核使用静态栈，辅核等待 MM 建立后准备栈。
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

    /// 返回可供本核启动和调度使用的已发布栈顶。
    pub fn stack_top(&self) -> usize {
        self.stack.top()
    }

    /// 返回本核动态栈实际拥有的直映起点和字节数，主核静态栈返回空。
    #[cfg(feature = "ktest")]
    pub(crate) fn stack_allocation(&self) -> Option<(usize, usize)> {
        self.stack.allocation()
    }

    /// 只观察未处理原因；最终等待检查不得消费通知。
    pub(crate) fn kick_pending(&self) -> bool {
        self.kick_reasons.load(Ordering::Acquire) != 0
    }

    /// 在最终队列与提示复查之后等待一次硬件事件，返回保持 IRQ 关闭。
    ///
    /// # Safety
    /// self 必须属于本核；必须在 scheduler 栈上、IRQ 关闭且无普通任务、锁或
    /// IRQ/preempt guard 时调用。调用者已发表 idle 并复查队列，允许临时服务 IRQ。
    pub(crate) unsafe fn idle_once(&self) {
        assert!(core::ptr::eq(self, current()), "idle uses another CPU");
        #[cfg(feature = "ktest")]
        self.test_idle_waits.fetch_add(1, Ordering::Release);
        // SAFETY: 调用者保证本核无锁 IRQ-off 边界；字段永久有效，架构重查但不清通知，
        // 临时开放 IRQ 时不存在依赖 IRQ 关闭的普通借用；测试计数仅以原子访问。
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

// 按槽构造，避免整个 CPU 表的按值临时副本叠加到启动栈。
static CPU_LOCALS: [Once<CpuLocal>; MAX_CPUS] = [const { Once::new() }; MAX_CPUS];
static mut BOOT_RECORDS: [CpuBoot; MAX_CPUS] = [CpuBoot {
    stack_top: 0,
    local: 0,
}; MAX_CPUS];

/// 构造 CPU 表和早期启动记录，仅主核填写静态栈顶，并安装主核的每核基址。
///
/// # Safety
/// 只能由 BSP 在 AP 未释放、IRQ 关闭时执行一次；info 是已发布的实际资源，
/// cpu_count 和 hardware_ids 已验证。启动栈映射有效，且不存在 AP 对记录的访问。
pub unsafe fn initialize(info: &BootInfo) {
    assert!(
        CPU_LOCALS.iter().all(|local| local.get().is_none()),
        "CPU table is already initialized"
    );
    let bsp_stack_top = ptr::addr_of!(BOOT_STACK) as usize + STACK_SIZE;
    for (id, slot) in CPU_LOCALS[..info.cpu_count].iter().enumerate() {
        let stack_top = if id == 0 { bsp_stack_top } else { 0 };
        let local = slot.call_once(|| CpuLocal::new(id, info.hardware_ids[id], stack_top));
        // SAFETY: id 来自已验证的有效 CPU 范围；BSP 独占静态记录，AP 尚未读取。
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
    // SAFETY: 逻辑 0 是 BSP；Once 中的对象已完成构造且永久存在，IRQ 尚未开启。
    unsafe { crate::arch::install_cpu_local(get(0)) };
}

/// 在 MM 建立后准备全部实际辅核栈，成功后填写启动记录，主核记录保持不变。
///
/// # Safety
/// 只能由 BSP 在 IRQ 关闭、MM 页分配器与永久可写直映已建立后调用。
/// CPU 表已构造，全部 AP 尚未释放，调用者独占启动记录与栈准备；
/// 全部辅核槽尚为空，记录不得被固件或其他 CPU 并发读取。
pub(crate) unsafe fn prepare_secondary_stacks() -> Result<(), AllocError> {
    assert_eq!(current().id, 0, "only BSP may prepare AP stacks");
    assert!(
        !crate::arch::irq_enabled(),
        "AP stack preparation requires IRQ off"
    );
    // SAFETY: 调用者保证 MM 已建立且全部 AP 未释放；逻辑编号的有限范围无重复，
    // 每个槽属于永久 CpuLocal，准备和启动记录写入均由唯一 BSP 独占。
    unsafe { stack::prepare_slots((1..count()).map(|id| &get(id).stack)) }?;
    for id in 1..count() {
        // SAFETY: 目标 AP 尚未释放，BSP 独占该有效记录，只填写准备成功的栈顶；
        // local 指向的永久对象保持原值，栈页由同一 CpuLocal 永久保活。
        unsafe {
            ptr::addr_of_mut!(
                (*ptr::addr_of_mut!(BOOT_RECORDS).cast::<CpuBoot>().add(id)).stack_top
            )
            .write(get(id).stack_top());
        }
    }
    Ok(())
}

/// 借用当前核的稳定 CPU 对象；未安装基址时直接报告初始化错误。
pub fn current() -> &'static CpuLocal {
    let local = crate::arch::cpu_local();
    // SAFETY: BSP 入口清空基址；RV AP 只从永久启动记录预装指针，installer 只写有效对象，
    // trap 恢复同一被打断基址。空值由 as_ref 返回 None，不构造空引用。
    unsafe { local.as_ref() }.expect("CPU local base has not been installed")
}

/// 返回已解析的固定 CPU 数量。
pub fn count() -> usize {
    crate::boot::info().cpu_count
}

/// 返回固件实际发现的逻辑 CPU 集合，与编译容量和硬件 ID 分离。
pub fn actual_set() -> CpuSet {
    CpuSet::prefix(count())
}

/// 借用有效逻辑编号的 CPU；构造时序和编号错误只在此对象边界报告。
pub fn get(id: usize) -> &'static CpuLocal {
    CPU_LOCALS[..count()][id]
        .get()
        .expect("CPU local has not been initialized")
}

/// 在本核已能服务中断后发布上线，使等待者的 acquire 观察对应真实可用状态。
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

/// 在本核永久启动栈等待执行许可，硬件停等期间仍可服务 IRQ/IPI，返回时恢复原 IRQ 状态。
/// 仅用于尚未进入调度器的在线 CPU；ready 在等待期间保持地址稳定，
/// 发布者必须先 Release 写入许可，再向等待核发送 kick。
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
            // SAFETY: 本核已上线且仍在永久启动栈，无普通任务、锁或执行深度 guard；
            // 本次 irq_save 后没有依赖 IRQ-off 的可变借用，允许架构临时服务 IRQ。
            // CpuLocal 字段永久稳定，ready 的共享借用覆盖整个等待；发布后 kick
            // 与架构的最终原子重查共同覆盖检查到硬件停等的窗口，测试计数仅原子访问。
            unsafe {
                #[cfg(feature = "ktest")]
                crate::arch::idle_once(&local.kick_reasons, ready, &local.test_wait_completions);
                #[cfg(not(feature = "ktest"))]
                crate::arch::idle_once(&local.kick_reasons, ready);
            }
        }
        // SAFETY: saved 来自本轮同核 irq_save；期间不切换栈或迁核，
        // 架构等待保持 IRQ-off 返回，且没有锁、guard 或临界区借用跨越恢复。
        unsafe { crate::arch::irq_restore(saved) };
    }
}

/// 由 BSP 串行释放 AP，等待每核能服务中断；失败只归还尚未尝试启动的后缀栈。
///
/// # Safety
/// CPU 表、全部独占栈和启动记录已准备，本核为唯一 BSP 启动者。
/// 全部 AP 尚未启动；记录交给固件后不得再改写或回收对应栈，映射永久有效。
/// 等待期间若 AP 会向 BSP 提交同步 CSD，本核须能服务对应 IPI；不得持有目标初始化所需的锁。
pub unsafe fn start_secondaries() {
    const AP_ONLINE_TIMEOUT_SECS: u64 = 5;

    for id in 1..count() {
        let local = get(id);
        // SAFETY: id 位于已构造的 CPU 范围，静态记录已发布且不会被回收。
        let record = unsafe { ptr::addr_of!(BOOT_RECORDS).cast::<CpuBoot>().add(id) as usize };
        let physical = crate::arch::virt_to_phys(record);
        // SAFETY: 本核是唯一启动者；记录已指向准备成功的独占栈与永久 CpuLocal，
        // physical 是记录的真实物理地址。调用后即保留目标记录和栈，不根据错误回收。
        if let Err(error) = unsafe { crate::arch::start_cpu(local.hardware_id, physical) } {
            // SAFETY: 串行启动仅尝试至 id，后缀从未交给固件，也没有 CPU 访问其栈。
            unsafe { release_unstarted_suffix(id + 1) };
            panic!(
                "AP start failed: hardware={} error={}",
                local.hardware_id, error
            );
        }
        let deadline = crate::arch::read_counter()
            + crate::boot::info().counter_frequency * AP_ONLINE_TIMEOUT_SECS;
        while !local.online.load(Ordering::Acquire) {
            if deadline <= crate::arch::read_counter() {
                // SAFETY: 超时目标可能正在使用栈；仅回收从未尝试启动的后缀。
                unsafe { release_unstarted_suffix(id + 1) };
                panic!("AP online timed out: hardware={}", local.hardware_id);
            }
            spin_loop();
        }
    }
}

/// 清空尚未交给固件的后缀记录，并在各槽短锁外归还对应栈页。
///
/// # Safety
/// 只能由独占启动过程的 BSP 调用；first 及之后的 CPU 从未尝试启动，
/// 不存在固件、CPU 或其他裸地址使用者访问这些记录和栈。此前记录必须保持不变。
unsafe fn release_unstarted_suffix(first: usize) {
    for id in first..count() {
        // SAFETY: 调用者保证后缀记录未交给固件且由 BSP 独占；只撤下栈顶，
        // 永久 local 地址不变，也不触碰此前已借给 AP 的记录。
        unsafe {
            ptr::addr_of_mut!(
                (*ptr::addr_of_mut!(BOOT_RECORDS).cast::<CpuBoot>().add(id)).stack_top
            )
            .write(0);
        }
        // SAFETY: 本槽位于从未尝试启动的后缀，记录已清空且无其他栈访问者。
        unsafe { get(id).stack.release_unstarted() };
    }
}

/// 硬 timer 只记录中断并提示普通任务在安全点让出，不在 IRQ 栈上调度。
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

/// 仅测试镜像屏蔽同步callback内捕获；不改变硬IRQ或BH的生产记账。
#[cfg(feature = "ktest")]
struct CallbackGuard {
    local: &'static CpuLocal,
    previous: bool,
    _local: PhantomData<*mut ()>,
}

#[cfg(feature = "ktest")]
impl CallbackGuard {
    /// 原callback分派已关IRQ，标志只属于当前固定CPU并保存嵌套原状态。
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
    /// 先解除本次callback的捕获屏蔽，再由外层IRQ guard恢复原状态。
    fn drop(&mut self) {
        self.local
            .ktest_callback_active
            .store(self.previous, Ordering::Relaxed);
    }
}

/// 在硬件通知已清除后消费软件原因、执行 CSD；调度唤醒不请求抢占，永久停机在队列处理后消费。
pub fn handle_ipi() {
    let local = current();
    local.ipi_interrupts.fetch_add(1, Ordering::Relaxed);
    // 硬件已确认后一次取走旧位，KICK_SCHED_WAKE 随之消费而不改变 need_resched；
    // 处理期间远端再置位仍留在原子中，并发送新的 IPI。
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
    loop {
        // SAFETY: 队列短锁覆盖 pop；入队协议保证所有未完成节点仍然存活。
        let request = unsafe { local.calls.lock().pop() };
        let request = match request {
            Some(request) => request,
            None => break,
        };
        #[cfg(feature = "ktest")]
        let _callback = CallbackGuard::enter();
        // SAFETY: 请求已取出但尚未发布完成，发送者仍保留节点和参数；动作由 call 的
        // unsafe 契约约束。完成是最后一次访问节点，随后仅继续处理队列。
        unsafe {
            ((*request).callback)((*request).argument);
            (*request).done.store(true, Ordering::Release);
        }
    }
    if reasons & KICK_STOP != 0 {
        crate::arch::stop_current_cpu(&local.stopped);
    }
}

/// 在目标 CPU 上同步执行一次短动作，真正完成后才归还参数拥有者。
///
/// # Safety
/// target 必须是在线表中的逻辑编号；callback 和 argument 在返回前保持有效。
/// 参数的共享写必须同步。callback 不得睡眠、访问用户内存或递归同步调用。
/// 调用者必须能服务本地 IPI，且不能持有 callback 需要的锁。
pub unsafe fn call(target: usize, callback: unsafe fn(*mut ()), argument: *mut ()) {
    if target == current().id {
        let _irq = IrqSaveGuard::new();
        #[cfg(feature = "ktest")]
        let _callback = CallbackGuard::enter();
        // SAFETY: caller 提供有效动作和参数；本核短 IRQ-save 域覆盖整个直接动作。
        unsafe { callback(argument) };
        return;
    }
    let local = get(target);
    let pinned = core::pin::pin!(Call {
        callback,
        argument,
        next: UnsafeCell::new(ptr::null_mut()),
        done: AtomicBool::new(false),
    });
    let request = pinned.as_ref().get_ref();
    let node = request as *const Call as *mut Call;
    // SAFETY: pinned 固定节点地址，参数义务由 caller 承担；队列锁覆盖链接更新，
    // 本函数在 acquire 观察完成前不离开此栈帧。
    unsafe { local.calls.lock().push(node) };
    crate::arch::send_ipi(local.hardware_id);
    while !request.done.load(Ordering::Acquire) {
        spin_loop();
    }
}

/// 测试只读真实队列深度，用于确定交错已经发生，不维护第二份计数。
#[cfg(feature = "ktest")]
pub fn pending_calls(target: usize) -> usize {
    let queue = get(target).calls.lock();
    let mut node = queue.head;
    let mut count = 0;
    while !node.is_null() {
        count += 1;
        // SAFETY: 队列锁保护全部链接，未完成节点的拥有者尚未结束其生命周期。
        node = unsafe { *(*node).next.get() };
    }
    count
}

/// 先发布固定软件原因再发送硬件 IPI；允许持调度锁调用，不等待目标执行。
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

/// 校验全部有效软件原因，包括仅唤醒 idle 的提示和永久停核请求。
fn kick_mask() -> usize {
    KICK_RESCHEDULE | KICK_TIMER_REPROGRAM | KICK_LOG_PRINTER | KICK_STOP | KICK_SCHED_WAKE
}

/// 在生命周期停止生产者后永久停稳其它 online 核，返回的 guard 固定协调核直到整机动作。
/// 调用者不得持业务锁，所有用户已离栈、FS 已退休且不再有必须继续运行的 CSD 责任；
/// 本核必须能服务 IPI，CPU 集合不得继续上线，返回后不能恢复普通执行或重用停稳确认。
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
    for id in targets.iter() {
        kick(id, KICK_STOP);
    }
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

/// 在真实硬中断分支内记录嵌套，析构只结束本次硬中断。
pub struct HardIrqGuard {
    local: &'static CpuLocal,
    _local: PhantomData<*mut ()>,
}

impl HardIrqGuard {
    /// 在 CPU 基址已安装且 IRQ 关闭的 handler 中进入硬中断执行段。
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
    /// 保持原 IRQ 关闭状态并结束本 CPU 的硬中断嵌套，不触发调度。
    fn drop(&mut self) {
        assert!(!crate::arch::irq_enabled(), "hard IRQ exit has IRQ enabled");
        if self.local.hardirq_depth.load(Ordering::Relaxed) == 1 {
            crate::accounting::irq_exit();
        }
        decrement_depth(&self.local.hardirq_depth);
    }
}

/// 底半部允许服务硬 IRQ，但不能切换普通任务。
pub struct BottomHalfGuard {
    local: &'static CpuLocal,
    _local: PhantomData<*mut ()>,
}

impl BottomHalfGuard {
    /// 在已安装 CPU 上进入底半部，短 IRQ-save 只保护深度修改。
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
    /// 结束同核底半部，不因深度归零而调用调度器。
    fn drop(&mut self) {
        let _irq = IrqSaveGuard::new();
        decrement_depth(&self.local.bh_depth);
    }
}

/// 原子执行深度不能溢出，违反嵌套协议立即报告。
pub(crate) fn increment_depth(depth: &AtomicUsize) {
    depth
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
            value.checked_add(1)
        })
        .expect("execution depth overflow");
}

/// 匹配一次进入，禁止下溢掩盖跨上下文析构。
pub(crate) fn decrement_depth(depth: &AtomicUsize) {
    depth
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
            value.checked_sub(1)
        })
        .expect("execution depth underflow");
}

/// 可武装的一次真实边界停点，状态不参与生产调度或通知协议。
#[cfg(feature = "ktest")]
#[derive(Clone, Copy)]
pub(crate) enum TestPause {
    Idle,
    Ipi,
}

/// 选择固定 CPU 上独立的验收停点状态。
#[cfg(feature = "ktest")]
fn test_pause_state(target: usize, boundary: TestPause) -> &'static AtomicUsize {
    let local = get(target);
    match boundary {
        TestPause::Idle => &local.test_idle_pause,
        TestPause::Ipi => &local.test_ipi_pause,
    }
}

/// 武装尚未使用的单次停点，测试必须由另一 CPU 负责及时释放。
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

/// 观察目标是否已经在真实 IRQ-off 边界停下。
#[cfg(feature = "ktest")]
pub(crate) fn test_is_paused(target: usize, boundary: TestPause) -> bool {
    test_pause_state(target, boundary).load(Ordering::Acquire) == TEST_PAUSE_REACHED
}

/// 释放已经到达的停点，不改变软件通知或硬件 pending。
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

/// 撤未到达的停点或放行实际暂停者；不消费 pending，也不等待旧执行者退出。
/// 控制者在另一普通 Task、无退出依赖锁；原 handler/idle 真收敛以前不得重新武装。
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

/// 仅在已武装时停留于本核无锁边界，以统一单调时钟限制验收等待。
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

/// 在 idle 发表且放开调度锁后、最终队列与提示检查之前提供单次停点。
#[cfg(feature = "ktest")]
pub(crate) fn test_idle_boundary() {
    test_pause_boundary(&current().test_idle_pause);
}

/// 读取真实原因位以验证 handler 处理期间新通知未被抹掉。
#[cfg(feature = "ktest")]
pub(crate) fn test_pending_kicks(target: usize) -> usize {
    get(target).kick_reasons.load(Ordering::Acquire)
}

/// 读取真实重编程原因的已处理次数，不以置位次数代替收件执行。
#[cfg(feature = "ktest")]
pub(crate) fn test_timer_reprograms(target: usize) -> usize {
    get(target).test_timer_reprograms.load(Ordering::Acquire)
}

/// 读取进入架构等待协议的次数，软件 pending 可使入口直接返回而不执行等待指令。
#[cfg(feature = "ktest")]
pub(crate) fn test_idle_waits(target: usize) -> usize {
    get(target).test_idle_waits.load(Ordering::Acquire)
}

/// 读取本核真正执行 WFI/IDLE 并返回的次数，软件提示跳过等待时不增加。
#[cfg(feature = "ktest")]
pub(crate) fn test_wait_completions(owner: usize) -> usize {
    get(owner).test_wait_completions.load(Ordering::Acquire)
}
