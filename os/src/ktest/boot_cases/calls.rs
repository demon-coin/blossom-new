//! 每例独立的同步请求、启动门和完成观测；callback 不借用父 fixture 栈。

use crate::ktest::boot_cases::hardware::wait;
use crate::ktest::{spawn_on, ChildHandle};
use crate::sync::PreemptGuard;
use crate::{arch, boot, cpu};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

struct Control {
    go: AtomicBool,
    stop: AtomicBool,
    ready: AtomicUsize,
    entered: AtomicBool,
    queued: AtomicBool,
    fault: AtomicBool,
    executed: cpu::AtomicCpuSet,
    hits: [AtomicUsize; boot::MAX_CPUS],
    returned: [AtomicBool; boot::MAX_CPUS],
    results: [AtomicUsize; boot::MAX_CPUS],
}

struct Calls {
    state: Arc<Control>,
    handles: [Option<ChildHandle>; boot::MAX_CPUS],
}

impl Fixture for Calls {
    /// 固定容量在任何真实请求发表前准备，主体失败仍由 fixture 保活所有门。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let state = Arc::try_new(Control {
            go: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            ready: AtomicUsize::new(0),
            entered: AtomicBool::new(false),
            queued: AtomicBool::new(false),
            fault: AtomicBool::new(false),
            executed: cpu::AtomicCpuSet::new(),
            hits: core::array::from_fn(|_| AtomicUsize::new(0)),
            returned: core::array::from_fn(|_| AtomicBool::new(false)),
            results: core::array::from_fn(|_| AtomicUsize::new(usize::MAX)),
        })
        .map_err(|_| TestError("NoMemory: synchronous call controls"))?;
        Ok(Self {
            state,
            handles: core::array::from_fn(|_| None),
        })
    }

    /// 合作关闭等待门，已经发表的同步 call 必须实际返回，不能超时释放参数。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.state.stop.store(true, Ordering::Release);
        self.state.go.store(true, Ordering::Release);
        Ok(())
    }

    /// 框架已 join 全部原 Task，确认没有活同步请求才消费控制 owner。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        assert!(self
            .handles
            .iter()
            .flatten()
            .all(|h| h.try_join().is_some()));
        Ok(())
    }
}

struct Payload {
    state: Arc<Control>,
    sender: usize,
    target: usize,
    result: usize,
    arrival: bool,
}

/// callback 在目标真实IRQ-save域记录一次动作，并写发送者仍拥有的同步参数。
///
/// # Safety
/// argument 指向同步 cpu::call 尚未完成的独占 Payload，期间地址固定；callback 不睡眠或递归call。
unsafe fn identity(argument: *mut ()) {
    // SAFETY: 唯一同步调用者持有此栈参数直到 release/acquire 完成，其他核不读取可变字段。
    let payload = unsafe { &mut *argument.cast::<Payload>() };
    if arch::irq_enabled() || cpu::current().id != payload.target {
        payload.state.fault.store(true, Ordering::Release);
    }
    payload.state.hits[payload.sender].fetch_add(1, Ordering::Relaxed);
    payload
        .state
        .executed
        .insert(cpu::current().id, Ordering::Release);
    payload.result = cpu::current().id + 0x100;
}

/// callback 有限暂停期间发送者不得提前返回；arrival 观察生产FIFO中后续真实节点。
///
/// # Safety
/// 参数与执行环境满足 identity 契约，state 强引用存活至同步返回。
unsafe fn delayed(argument: *mut ()) {
    // SAFETY: 同步请求已取出但尚未发布完成，调用者栈与控制对象仍保活。
    let payload = unsafe { &mut *argument.cast::<Payload>() };
    payload.state.entered.store(true, Ordering::Release);
    if payload.arrival {
        let end = arch::read_counter() + arch::counter_frequency() * 5;
        while cpu::pending_calls(payload.target) != cpu::count() - 2 {
            if payload.state.stop.load(Ordering::Acquire) {
                break;
            }
            if end <= arch::read_counter() {
                payload.state.fault.store(true, Ordering::Release);
                break;
            }
            core::hint::spin_loop();
        }
        payload.state.queued.store(
            cpu::pending_calls(payload.target) == cpu::count() - 2,
            Ordering::Release,
        );
    }
    let end = arch::read_counter() + (arch::counter_frequency() / 500).max(1);
    while arch::read_counter() < end {
        if payload.state.returned[payload.sender].load(Ordering::Acquire) {
            payload.state.fault.store(true, Ordering::Release);
        }
        core::hint::spin_loop();
    }
    // SAFETY: 原同步动作仍未发布完成，参数存活和独占修改条件与 identity 相同。
    unsafe { identity(argument) };
    // SAFETY: identity 已返回但 cpu::call 还未发布 done，本栈参数仍可借用。
    let payload = unsafe { &*argument.cast::<Payload>() };
    if payload.state.returned[payload.sender].load(Ordering::Acquire) {
        payload.state.fault.store(true, Ordering::Release);
    }
}

impl Calls {
    /// 在真实固定核Task上创建独立同步参数，Task自身持state强引用。
    fn sender(
        &mut self,
        sender: usize,
        target: usize,
        delay: bool,
        arrival: bool,
        follower: bool,
    ) -> TestResult {
        let state = Arc::clone(&self.state);
        self.handles[sender] = Some(spawn_on(sender, move |token| {
            state.ready.fetch_add(1, Ordering::Release);
            while !state.go.load(Ordering::Acquire) {
                if token.stop_requested() || state.stop.load(Ordering::Acquire) {
                    return Ok(());
                }
                crate::sched::yield_now();
            }
            if follower {
                while !state.entered.load(Ordering::Acquire) {
                    if token.stop_requested() || state.stop.load(Ordering::Acquire) {
                        return Ok(());
                    }
                    crate::sched::yield_now();
                }
            }
            if token.stop_requested() || state.stop.load(Ordering::Acquire) {
                return Ok(());
            }

            let _preempt = PreemptGuard::new();
            assert_eq!(cpu::current().id, sender);
            let mut payload = Payload {
                state: Arc::clone(&state),
                sender,
                target,
                result: usize::MAX,
                arrival,
            };
            // SAFETY: target 是已上线CPU，发送者可服务IPI、无callback所需锁；payload与
            // callback控制对象一直保活至真实同步返回，不因timeout放弃借用。
            unsafe {
                cpu::call(
                    target,
                    if delay { delayed } else { identity },
                    (&mut payload as *mut Payload).cast(),
                )
            };
            state.returned[sender].store(true, Ordering::Release);
            assert!(arch::irq_enabled());
            assert_eq!(payload.result, target + 0x100);
            assert_eq!(state.hits[sender].load(Ordering::Acquire), 1);
            state.results[sender].store(payload.result, Ordering::Release);
            Ok(())
        })?);
        Ok(())
    }

    /// 启动门先等待本轮全部Task到达，保证pair/fan-in有真正并发发送者。
    fn release(&self, count: usize) -> TestResult {
        wait(
            || self.state.ready.load(Ordering::Acquire) == count,
            "call sender did not reach gate",
        )?;
        self.state.go.store(true, Ordering::Release);
        Ok(())
    }

    /// 消费真实离栈观测后核查目标身份、次数和不提前返回账本。
    fn finish(&self, context: &mut TestContext<'_>, sender: usize, target: usize) -> TestResult {
        wait(
            || self.handles[sender].as_ref().unwrap().try_join().is_some(),
            "synchronous call did not complete",
        )?;

        context.expect_eq(
            self.state.results[sender].load(Ordering::Acquire),
            target + 0x100,
        );
        context.expect_eq(self.state.hits[sender].load(Ordering::Acquire), 1);
        assert!(self.state.returned[sender].load(Ordering::Acquire));
        assert!(
            !self.state.fault.load(Ordering::Acquire),
            "callback IRQ/lifetime/arrival invariant failed"
        );
        Ok(())
    }
}

/// 成对核互发；奇数末核仍向在线 BSP 发送一次真实远程请求。
fn pair_target(id: usize) -> usize {
    let paired = id ^ 1;
    if paired < cpu::count() {
        paired
    } else {
        0
    }
}

#[::ktest::suite(context = "Thread", fixture = Calls, tags("boot", "smp-call"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// 每个实际核自调用，callback IRQ关闭且sender恢复开启。
    #[test]
    fn self_call(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 0..cpu::count() {
            fixture.sender(id, id, false, false, false)?;
        }

        fixture.release(cpu::count())?;
        for id in 0..cpu::count() {
            fixture.finish(context, id, id)?;
        }
        context.expect_eq(
            fixture.state.executed.load(Ordering::Acquire),
            cpu::actual_set(),
        );
        Ok(())
    }

    /// 每对在线 CPU 同时互发，奇数末核向 BSP 发送，所有调用者等待时仍服务 IPI。
    #[test]
    #[ktest(min_cpus = 2)]
    fn pair(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 0..cpu::count() {
            fixture.sender(id, pair_target(id), false, false, false)?;
        }

        fixture.release(cpu::count())?;
        for id in 0..cpu::count() {
            fixture.finish(context, id, pair_target(id))?;
        }
        let targets = if cpu::count() % 2 == 0 {
            cpu::actual_set()
        } else {
            cpu::CpuSet::prefix(cpu::count() - 1)
        };
        context.expect_eq(fixture.state.executed.load(Ordering::Acquire), targets);
        Ok(())
    }

    /// 全部核沿环向下一核发送，真实远端 callback 必须覆盖每个逻辑编号。
    #[test]
    #[ktest(min_cpus = 2)]
    fn ring(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 0..cpu::count() {
            fixture.sender(id, (id + 1) % cpu::count(), false, false, false)?;
        }

        fixture.release(cpu::count())?;
        for id in 0..cpu::count() {
            fixture.finish(context, id, (id + 1) % cpu::count())?;
        }
        context.expect_eq(
            fixture.state.executed.load(Ordering::Acquire),
            cpu::actual_set(),
        );
        Ok(())
    }

    /// 全部AP同时向BSP发送，每个发送者独立核一次结果可见性。
    #[test]
    #[ktest(min_cpus = 2)]
    fn fan_in(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        for id in 1..cpu::count() {
            fixture.sender(id, 0, false, false, false)?;
        }

        fixture.release(cpu::count() - 1)?;
        for id in 1..cpu::count() {
            fixture.finish(context, id, 0)?;
        }
        Ok(())
    }

    /// 单核自调用也经过同一有限callback暂停，不冒称远端路径。
    #[test]
    fn delayed_local(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        fixture.sender(0, 0, true, false, false)?;
        fixture.release(1)?;
        fixture.finish(context, 0, 0)?;
        assert!(fixture.state.entered.load(Ordering::Acquire));
        Ok(())
    }

    /// AP调用BSP在真实callback暂停之后才返回，参数不能提前结束生命周期。
    #[test]
    #[ktest(min_cpus = 2)]
    fn delayed_remote(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        fixture.sender(1, 0, true, false, false)?;
        fixture.release(1)?;
        fixture.finish(context, 1, 0)?;
        assert!(fixture.state.entered.load(Ordering::Acquire));
        Ok(())
    }

    /// 生产FIFO确认处理期间到达的两个新请求，原IPI处理循环必须再次取出它们。
    #[test]
    #[ktest(min_cpus = 4)]
    fn processing_arrival(context: &mut TestContext<'_>, fixture: &mut Calls) -> TestResult {
        fixture.sender(1, 0, true, true, false)?;
        for id in 2..cpu::count() {
            fixture.sender(id, 0, false, false, true)?;
        }

        fixture.release(cpu::count() - 1)?;
        for id in 1..cpu::count() {
            fixture.finish(context, id, 0)?;
        }
        assert!(fixture.state.queued.load(Ordering::Acquire));
        Ok(())
    }
}
