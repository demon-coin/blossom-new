//! 单一 UART 的真实 RX 与同步退役；H/ACK 由原有窄 io 协议拥有。

use crate::irq::IrqRegistration;
use crate::ktest::boot_cases::hardware::wait;
use crate::ktest::{active_case, control_read_byte, spawn_on, ChildHandle};
use crate::{arch, boot, console, irq};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

struct Action {
    registration: UnsafeCell<Option<IrqRegistration>>,
    alive: AtomicBool,
    concurrent: AtomicBool,
    entered: AtomicBool,
    exited: AtomicBool,
    returned: AtomicBool,
    stop: AtomicBool,
    fault: AtomicBool,
    hits: AtomicUsize,
}
// SAFETY: registration 只在未开放IRQ且没有子任务时写一次；开放后只通过共享token
// 内部锁访问。所有其他共享可变字段为原子，fixture直到同步注销和真实join以后才释放对象。
unsafe impl Sync for Action {}

impl Action {
    /// 在取得UART之前准备稳定context，后续真实handler从不借fixture栈。
    fn new() -> Result<Arc<Self>, TestError> {
        Arc::try_new(Self {
            registration: UnsafeCell::new(None),
            alive: AtomicBool::new(true),
            concurrent: AtomicBool::new(false),
            entered: AtomicBool::new(false),
            exited: AtomicBool::new(false),
            returned: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            fault: AtomicBool::new(false),
            hits: AtomicUsize::new(0),
        })
        .map_err(|_| TestError("NoMemory: UART context"))
    }

    /// 无其他Arc拥有者且未开放准入时一次登记，context固定到同步注销/join。
    fn register(owner: &mut Arc<Self>) -> TestResult {
        assert_eq!(
            Arc::strong_count(owner),
            1,
            "UART registration requires exclusive owner"
        );
        let state = &**owner;
        // SAFETY: &mut owner排除同时安装，唯一强引用且本模块不产生Weak；
        // token尚未安装，不存在IRQ/child访问，检查只读已初始化Option。
        assert!(
            unsafe { (*state.registration.get()).is_none() },
            "UART action already installed"
        );
        let number = irq::map_source(boot::info().interrupts.uart_source)
            .map_err(|_| TestError("UART source outside domain"))?;
        // SAFETY: producer已停止，state由fixture唯一Arc固定拥有，handler仅在token安装后开放。
        let token = unsafe {
            irq::register(
                number,
                uart_action,
                (state as *const Self).cast_mut().cast(),
            )
        }
        .map_err(|_| TestError("UART action slots exhausted"))?;
        // SAFETY: registration尚未enable，无handler/child访问；此槽在对象生命周期内仅写一次。
        unsafe {
            *state.registration.get() = Some(token);
        }
        Ok(())
    }

    /// 已安装后取得共享token，不提供可变访问或撤销其存储。
    fn token(&self) -> &IrqRegistration {
        // SAFETY: 只在register成功后调用，槽不再改写；token本身用内部锁同步。
        unsafe {
            (*self.registration.get())
                .as_ref()
                .expect("UART token not installed")
        }
    }

    /// 对部分setup与正常fixture都停止真实producer并同步释放执行引用。
    fn stop(&self) {
        self.stop.store(true, Ordering::Release);
        console::disable_interrupts();
        // SAFETY: 槽只在setup独占阶段写，停止执行者与handler均只读；None表示尚未登记。
        if let Some(token) = unsafe { &*self.registration.get() } {
            token.unregister_sync();
        }
        self.alive.store(false, Ordering::Release);
    }
}

/// 消费一个真实硬件字节后停止IER，使同通道ACK保持在FIFO而不进入被测计数。
///
/// # Safety
/// argument是fixture稳定Arc context，真实UART IRQ持执行引用；fixture同步注销前不得释放。
unsafe fn uart_action(argument: *mut ()) -> bool {
    // SAFETY: 注册上下文一直保活到自己的执行引用归零，字段共享访问遵守原子与token锁。
    let state = unsafe { &*argument.cast::<Action>() };
    if !state.alive.load(Ordering::Acquire) {
        state.fault.store(true, Ordering::Release);
    }
    // SAFETY: 真实非嵌套UART action，不持PORT；此生产窄入口只把RX预算改为一个字节。
    let handled = unsafe { console::ktest_uart_handler_one(core::ptr::null_mut()) };
    if !handled {
        return false;
    }
    state.hits.fetch_add(1, Ordering::Release);
    if state.concurrent.load(Ordering::Acquire) {
        state.entered.store(true, Ordering::Release);
        let limit = arch::read_counter() + arch::counter_frequency() * 5;
        while state.token().is_enabled() && !state.stop.load(Ordering::Acquire) {
            if limit <= arch::read_counter() {
                state.fault.store(true, Ordering::Release);
                break;
            }
            core::hint::spin_loop();
        }
        let end = arch::read_counter() + (arch::counter_frequency() / 500).max(1);
        while arch::read_counter() < end {
            if !state.alive.load(Ordering::Acquire) || state.returned.load(Ordering::Acquire) {
                state.fault.store(true, Ordering::Release);
            }
            core::hint::spin_loop();
        }
    }
    state.exited.store(true, Ordering::Release);
    handled
}

struct IoStage {
    byte: u8,
    before: usize,
    handshake: bool,
}

struct Uart {
    old: Arc<Action>,
    new: Arc<Action>,
    remover: Option<ChildHandle>,
    stage: Option<IoStage>,
}

/// 已同步停止fixture action以后，重新登记同一个生产UART默认处理者。
fn restore_default() -> TestResult {
    let number = irq::map_source(boot::info().interrupts.uart_source)
        .map_err(|_| TestError("UART source outside domain"))?;
    // SAFETY: fixture的action已同步注销；默认handler不使用context且生产端口永久有效。
    let token = unsafe { irq::register(number, console::uart_handler, core::ptr::null_mut()) }
        .map_err(|_| TestError("default UART registration failed"))?;
    console::testing::restore_uart_registration(token)?;
    console::enable_receive_interrupts();
    Ok(())
}

impl Fixture for Uart {
    /// 在转移唯一默认token前准备全部context；setup失败也恢复真实默认action。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let mut old = Action::new()?;
        let new = Action::new()?;
        let initial = console::testing::take_uart_registration()?;
        console::disable_interrupts();
        initial.unregister_sync();
        if let Err(error) = Action::register(&mut old) {
            restore_default()?;
            return Err(error);
        }
        old.token().enable();
        Ok(Self {
            old,
            new,
            remover: None,
            stage: None,
        })
    }

    /// 先释放handler停止条件、停真实producer并同步注销，再完成唯一通道剩余ACK。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.old.stop();
        self.new.stop();
        if self.stage.is_some() {
            self.finish_stage()?;
        }
        Ok(())
    }

    /// 已join实际注销Task且执行引用为零后释放context，最后归还默认UART owner。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        assert!(self
            .remover
            .as_ref()
            .is_none_or(|handle| handle.try_join().is_some()));
        assert!(self.stage.is_none());
        restore_default()
    }
}

/// Castagnoli单字节CRC；协议digest直接绑定这次实际输入。
fn crc(byte: u8) -> u32 {
    let mut value = 0xffff_ffffu32 ^ u32::from(byte);
    for _ in 0..8 {
        value = (value >> 1) ^ if value & 1 != 0 { 0x82f6_3b78 } else { 0 };
    }
    !value
}

/// 无IRQ竞争时读取控制字节，有界等待不能被业务日志刷新延长。
fn control_byte() -> Result<u8, TestError> {
    wait(console::receive_pending, "UART control input timed out")?;
    Ok(control_read_byte())
}

impl Uart {
    /// 窄阶段先停止IER，实际H确认后才允许被测硬件RX进入。
    fn begin_stage(&mut self, byte: u8) -> TestResult {
        assert!(self.stage.is_none());
        console::disable_interrupts();
        let id = active_case().ok_or(TestError("UART IO outside case"))?;
        self.stage = Some(IoStage {
            byte,
            before: console::received(),
            handshake: false,
        });
        crate::ktest::report::write(format_args!(
            "  # KTEST io {} rx 1 {:08x} 0\n",
            id,
            crc(byte)
        ));
        assert_eq!(control_byte()?, b'H', "UART IO handshake mismatch");
        self.stage.as_mut().unwrap().handshake = true;
        Ok(())
    }

    /// ACK不算测试payload；未消费payload在失败清理时只为结束raw ownership读走。
    fn finish_stage(&mut self) -> TestResult {
        console::disable_interrupts();
        let stage = self.stage.as_mut().ok_or(TestError("UART stage absent"))?;
        if !stage.handshake {
            if control_byte()? != b'H' {
                return Err(TestError("UART cleanup handshake mismatch"));
            }
            stage.handshake = true;
        }
        if console::received() == stage.before && control_byte()? != stage.byte {
            return Err(TestError("UART cleanup payload mismatch"));
        }

        let mut line = [0u8; 64];
        let mut length = 0;
        loop {
            let byte = control_byte()?;
            if byte == b'\n' {
                break;
            }
            if length == line.len() {
                return Err(TestError("UART ACK exceeds bound"));
            }
            line[length] = byte;
            length += 1;
        }
        let digits = line[..length]
            .strip_prefix(b"KTEST IO OK ")
            .ok_or(TestError("host rejected UART IO"))?;
        let mut id = 0u32;
        if digits.is_empty() {
            return Err(TestError("UART ACK case absent"));
        }
        for &byte in digits {
            if !byte.is_ascii_digit() {
                return Err(TestError("UART ACK case malformed"));
            }
            id = id
                .checked_mul(10)
                .and_then(|n| n.checked_add(u32::from(byte - b'0')))
                .ok_or(TestError("UART ACK case overflows"))?;
        }
        if Some(id) != active_case() {
            return Err(TestError("UART ACK case mismatch"));
        }
        self.stage = None;
        Ok(())
    }

    /// 原始payload必须恰由真实IRQ处理一次；控制H和ACK没有进入计数。
    fn receive_stage(&mut self, action: &Arc<Action>, byte: u8) -> TestResult {
        let before = self
            .stage
            .as_ref()
            .ok_or(TestError("UART stage absent"))?
            .before;
        wait(
            || before < console::received(),
            "UART IRQ did not receive payload",
        )?;
        assert_eq!(console::received(), before + 1);
        assert_eq!(console::last_byte(), byte);
        action.token().disable_sync();
        assert!(action.exited.load(Ordering::Acquire));
        assert!(!action.fault.load(Ordering::Acquire));
        self.finish_stage()
    }

    /// 同轮旧context停止后验证实际pending，再把字节交给新context和新token。
    fn lifecycle(&mut self, concurrent: bool) -> TestResult {
        let old = Arc::clone(&self.old);
        old.concurrent.store(concurrent, Ordering::Release);
        if concurrent {
            let state = Arc::clone(&old);
            self.remover = Some(spawn_on(1, move |token| {
                while !state.entered.load(Ordering::Acquire) {
                    if token.stop_requested() || state.stop.load(Ordering::Acquire) {
                        return Ok(());
                    }
                    crate::sched::yield_now();
                }
                console::disable_interrupts();
                state.token().unregister_sync();
                state.returned.store(true, Ordering::Release);
                assert!(state.exited.load(Ordering::Acquire));
                Ok(())
            })?);
        }
        self.begin_stage(b'L')?;
        console::enable_receive_interrupts();
        self.receive_stage(&old, b'L')?;
        if concurrent {
            wait(
                || self.remover.as_ref().unwrap().try_join().is_some(),
                "UART remover did not leave stack",
            )?;
            assert!(old.returned.load(Ordering::Acquire));
            assert!(old.exited.load(Ordering::Acquire));
        } else {
            old.token().unregister_sync();
        }

        old.alive.store(false, Ordering::Release);
        assert!(!old.token().is_enabled());
        let hits = old.hits.load(Ordering::Acquire);
        let before = console::received();
        self.begin_stage(b'R')?;
        wait(
            console::receive_pending,
            "unregistered UART input did not arrive",
        )?;
        assert_eq!(console::received(), before);
        assert_eq!(old.hits.load(Ordering::Acquire), hits);

        Action::register(&mut self.new)?;
        let new = Arc::clone(&self.new);
        console::enable_receive_interrupts();
        new.token().enable();
        old.token().disable_sync();
        old.token().unregister_sync();
        assert!(new.token().is_enabled());
        assert!(!old.token().is_enabled());
        self.receive_stage(&new, b'R')?;
        assert_eq!(new.hits.load(Ordering::Acquire), 1);
        assert_eq!(old.hits.load(Ordering::Acquire), hits);
        Ok(())
    }
}

#[::ktest::suite(context = "Thread", fixture = Uart, tags("boot", "uart"), resources("uart"), timeout_ms = 30000)]
pub(crate) mod cases {
    use super::*;

    /// 真实宿主U输入只由UART IRQ消费一次。
    #[test]
    fn normal_rx(_: &mut TestContext<'_>, fixture: &mut Uart) -> TestResult {
        let old = Arc::clone(&fixture.old);
        fixture.begin_stage(b'U')?;
        console::enable_receive_interrupts();
        fixture.receive_stage(&old, b'U')?;
        assert_eq!(old.hits.load(Ordering::Acquire), 1);
        Ok(())
    }

    /// M先在真实设备FIFO pending，屏蔽期间无handler，解屏蔽才消费。
    #[test]
    fn masked_pending(_: &mut TestContext<'_>, fixture: &mut Uart) -> TestResult {
        let old = Arc::clone(&fixture.old);
        old.token().disable_sync();
        let hits = old.hits.load(Ordering::Acquire);
        let before = console::received();
        fixture.begin_stage(b'M')?;
        console::enable_receive_interrupts();
        wait(console::receive_pending, "masked UART input did not arrive")?;
        assert_eq!(console::received(), before);
        assert_eq!(old.hits.load(Ordering::Acquire), hits);

        old.token().enable();
        fixture.receive_stage(&old, b'M')?;
        assert_eq!(old.hits.load(Ordering::Acquire), hits + 1);
        Ok(())
    }

    /// 本核同步注销L并以新context消费已pending的R；不冒称AP并发。
    #[test]
    fn lifecycle_local(_: &mut TestContext<'_>, fixture: &mut Uart) -> TestResult {
        fixture.lifecycle(false)
    }

    /// AP在BSP真实handler内同步注销，返回前handler离开；随后验证旧token不能影响新context。
    #[test]
    #[ktest(min_cpus = 2)]
    fn lifecycle_concurrent(_: &mut TestContext<'_>, fixture: &mut Uart) -> TestResult {
        fixture.lifecycle(true)
    }
}
