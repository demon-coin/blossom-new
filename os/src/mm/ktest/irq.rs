//! 真实 timer IRQ 在生产页分配的锁外清零窗口重入，同票远端操作在调用返回前完成。

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

const BUDGET: usize = 16;
const MAX: usize = crate::boot::MAX_CPUS;
static ACTIVE: AtomicPtr<Control> = AtomicPtr::new(ptr::null_mut());

struct Control {
    armed: AtomicBool,
    owner: AtomicUsize,
    window: AtomicBool,
    ticket: AtomicUsize,
    irq_ticket: AtomicUsize,
    target: AtomicUsize,
    programs: AtomicUsize,
    handlers: AtomicUsize,
    hits: AtomicUsize,
    completed: AtomicUsize,
    failures: AtomicUsize,
    remote_frees: AtomicUsize,
    remote_drains: AtomicUsize,
    requests: [AtomicUsize; MAX],
    completions: [AtomicUsize; MAX],
    live_pages: crate::sync::SpinLock<Option<OwnedPages>>,
    state: Arc<State>,
}
impl Control {
    /// 全部窗口计数属于本例控制对象，后台 Task 不会占用票号。
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

/// 活窗只借用 fixture 保活的 Control，所有原 Task join 后才允许消费其 Arc。
pub(crate) struct AllocationWindow {
    control: *const Control,
    ticket: usize,
    target: usize,
}
impl AllocationWindow {
    /// 页入口仅捕获当前精确原 Task 的普通 IRQ 开启调用，IRQ 内嵌套入口不占票。
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
        // SAFETY: ACTIVE 只在成功 fixture 保活时发布，quiesce 清除并同步真实 IRQ，最后 join 后才 Drop。
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
    /// backing 成功、全部 MM guard 退出后发布同票请求并真实编程硬件 timer。
    pub(crate) fn arm_timer(&mut self) {
        if self.ticket == 0 {
            return;
        }
        // SAFETY: 非零票持有本 fixture 活窗，原 child join 前 Control 保活。
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
    /// 同票远端完成先于真实 alloc_pages 返回；合作停止与失败只记错误，不在 Drop 二次 panic。
    fn drop(&mut self) {
        if self.ticket == 0 {
            return;
        }
        // SAFETY: 当前原 child 尚未离栈，fixture 的稳定 Control Arc 仍保活。
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

/// 只由真实 cpu timer IRQ 路径调用，普通函数调用不能成为本例 IRQ 证据。
pub(crate) fn timer_irq() {
    if cpu::current().id != 0 {
        return;
    }
    let pointer = ACTIVE.load(Ordering::Acquire);
    if pointer.is_null() {
        return;
    }
    // SAFETY: timer IRQ 仅 CPU0 读取；quiesce 的同步 CPU0 callback 等实际 handler 尾部结束才准 Drop。
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
    // 正式 timekeeper 仍可能产生同窗多个 IRQ；每活票只承担一次有限 allocator 探测。
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

pub(crate) struct IrqFixture {
    memory: SmpFixture,
    control: Arc<Control>,
}
impl Fixture for IrqFixture {
    /// 预备拥有控制、原页与对象槽，IRQ 尚未武装，不在压力窗口申请控制存储。
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
    /// 停止具体窗口源并同步真实 CPU0 IRQ 尾部，再恢复 timekeeper 原最早硬件事件。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.control.armed.store(false, Ordering::Release);
        self.memory.quiesce(context)?;
        // SAFETY: CPU0 在线，callback 不借用栈或 fixture，只撤全局观察源并重查真实事件；同步 call 等其执行尾部。
        unsafe { cpu::call(0, disarm, ptr::null_mut()) };
        Ok(())
    }
    /// 框架已真实 join 全部原 Task，才消费窗口页、Control 和六域资源。
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

/// CPU0 真实同步 callback 在 IRQ 禁止的本核临界区撤观察源，随后原 timer 继续调度。
///
/// # Safety
/// 必须由当前在线 CPU0 的同步 cpu::call callback 调用，IRQ 关闭且 timekeeper 已初始化。
unsafe fn disarm(_: *mut ()) {
    ACTIVE.store(ptr::null_mut(), Ordering::Release);
    crate::time::reprogram_current();
}

#[ktest::suite(fixture = IrqFixture, tags("mm", "timer", "irq"), timeout_ms = 30000)]
mod reentry {
    use super::*;
    /// 单核实际 timer allocator 重入，不冒称跨核操作。
    #[test]
    fn local_timer(_: &mut TestContext<'_>, f: &mut IrqFixture) -> TestResult {
        run(f, false)
    }
    /// 每个真实 AP 同票 free 与 drain 在 CPU0 原分配返回之前完成。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_same_ticket(_: &mut TestContext<'_>, f: &mut IrqFixture) -> TestResult {
        run(f, true)
    }
    /// 本例拥有全部异步源；真实页入口 hook 和真实 timer IRQ 形成可观察同票窗口。
    fn run(f: &mut IrqFixture, remote: bool) -> TestResult {
        let participants = if remote { cpu::count() - 1 } else { 0 };
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
                // SAFETY: 原 zeroed 入口已写满整块，owning 槽在检查期间保活，没有并发读写。
                assert_eq!(
                    black_box(unsafe { pages.as_mut_ptr().cast::<u8>().read() }),
                    0
                );
                let pages = slot.take();
                drop(slot);
                drop(pages);
            }
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
