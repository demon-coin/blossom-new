//! H/ACK 轮询与单字节真实 RX 分开拥有；固定 Arc context 到同步注销后才释放。

use crate::irq::{IrqNumber, IrqRegistration};
use crate::ktest::{active_case, control_read_byte};
use crate::{boot, console, irq};
use ::ktest::{TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::sync::atomic::{AtomicUsize, Ordering};

struct State {
    hits: AtomicUsize,
}
struct Stage {
    byte: u8,
    before: usize,
    hits: usize,
    handshake: bool,
}

pub(super) struct Uart {
    state: Arc<State>,
    token: IrqRegistration,
    source: IrqNumber,
    stage: Option<Stage>,
}

/// 只由真实外部 IRQ domain 调用，窄生产入口在消费 payload 时关闭 IER。
///
/// # Safety
/// argument 指向 fixture 的稳定 Arc State，register 的执行引用由同步注销结束；
/// 此 handler 不持 PORT 或退出所需的外部锁，不等待、不分配。
unsafe fn action(argument: *mut ()) -> bool {
    // SAFETY: fixture 到 unregister_sync 返回一直保活原 Arc，State 的唯一可变字段是原子。
    let state = unsafe { &*argument.cast::<State>() };
    // SAFETY: 从原外部 IRQ 调用且未持 PORT；永久 UART 映射有效，RX 预算为单字节。
    let handled = unsafe { console::ktest_uart_handler_one(core::ptr::null_mut()) };
    if handled {
        state.hits.fetch_add(1, Ordering::Release);
    }
    handled
}

/// 原测试 action 已同步停止且 IER 关闭后重建真正 default token，不重开 Closed 旧 token。
fn restore_default(source: IrqNumber) -> TestResult {
    // SAFETY: default handler 没有借用 context，永久 UART 映射有效，测试 action 已注销。
    let token = unsafe { irq::register(source, console::uart_handler, core::ptr::null_mut()) }
        .map_err(|_| TestError("multi-IRQ default UART registration failed"))?;
    console::testing::restore_uart_registration(token)?;
    console::enable_receive_interrupts();
    Ok(())
}

/// Castagnoli 单字节 CRC 直接绑定本阶段实际 payload。
fn crc(byte: u8) -> u32 {
    let mut value = 0xffff_ffffu32 ^ u32::from(byte);
    for _ in 0..8 {
        value = (value >> 1) ^ if value & 1 != 0 { 0x82f6_3b78 } else { 0 };
    }
    !value
}

/// IER 关闭且测试 token 没有准入时独占消费控制字节。
fn control_byte() -> Result<u8, TestError> {
    super::until(
        "multi-IRQ UART control input timed out",
        console::receive_pending,
    )?;
    Ok(control_read_byte())
}

impl Uart {
    /// 全部可失败 context 分配先于真实 default token 转移；注册失败也恢复原 UART 职责。
    pub(super) fn prepare() -> Result<Self, TestError> {
        let state = Arc::try_new(State {
            hits: AtomicUsize::new(0),
        })
        .map_err(|_| TestError("multi-IRQ UART context allocation failed"))?;
        let source = irq::map_source(boot::info().interrupts.uart_source)
            .map_err(|_| TestError("multi-IRQ UART source invalid"))?;

        let initial = console::testing::take_uart_registration()?;
        console::disable_interrupts();
        initial.unregister_sync();

        // SAFETY: state 已稳定到 Arc 原地址，无其它 producer；返回后 fixture 保活到同步注销。
        let registration =
            unsafe { irq::register(source, action, Arc::as_ptr(&state).cast_mut().cast()) };
        let token = match registration {
            Ok(token) => token,
            Err(_) => {
                restore_default(source)?;
                return Err(TestError("multi-IRQ UART action registration failed"));
            }
        };
        Ok(Self {
            state,
            token,
            source,
            stage: None,
        })
    }

    /// 返回本次真实 BootInfo 映射的 UART source，不构造第二控制通道。
    pub(super) fn source(&self) -> IrqNumber {
        self.source
    }

    /// 测试 action 尚未 enable；H 从原 FIFO 轮询消费后才开放 RX producer。
    pub(super) fn begin(&mut self, byte: u8) -> TestResult {
        assert!(self.stage.is_none(), "multi-IRQ UART stage already active");
        self.token.disable_sync();
        console::disable_interrupts();
        let id = active_case().ok_or(TestError("multi-IRQ UART IO outside active case"))?;
        self.stage = Some(Stage {
            byte,
            before: console::received(),
            hits: self.state.hits.load(Ordering::Acquire),
            handshake: false,
        });
        crate::ktest::report::write(format_args!(
            "  # KTEST io {} rx 1 {:08x} 0\n",
            id,
            crc(byte)
        ));
        if control_byte()? != b'H' {
            return Err(TestError("multi-IRQ UART handshake mismatch"));
        }
        self.stage.as_mut().unwrap().handshake = true;
        console::enable_receive_interrupts();
        Ok(())
    }

    /// 原 FIFO payload 已 pending 时才开放原 action，不软件调用 UART handler。
    pub(super) fn enable(&self) {
        self.token.enable();
    }

    /// 一次真实 IRQ 恰消费一字节，关闭 IER 后 ACK 不进入 RECEIVED 或 LAST_BYTE。
    pub(super) fn receive(&mut self, context: &mut TestContext<'_>, byte: u8) -> TestResult {
        let stage = self
            .stage
            .as_ref()
            .ok_or(TestError("multi-IRQ UART stage absent"))?;
        let before = stage.before;
        let hits = stage.hits;
        super::until(
            "multi-IRQ UART payload was not received by hardware IRQ",
            || before < console::received(),
        )?;

        self.token.disable_sync();
        context.expect_eq(console::received(), before + 1);
        context.expect_eq(console::last_byte(), byte);
        context.expect_eq(self.state.hits.load(Ordering::Acquire), hits + 1);

        self.finish()?;
        context.expect_eq(console::received(), before + 1);
        context.expect_eq(console::last_byte(), byte);
        Ok(())
    }

    /// 失败清理可以读走未被 IRQ 消费的 payload，只为结束 raw ownership，不算真实接收证据。
    fn finish(&mut self) -> TestResult {
        const ACK_LINE_BYTES: usize = 64;
        console::disable_interrupts();
        let stage = self
            .stage
            .as_mut()
            .ok_or(TestError("multi-IRQ UART stage absent"))?;
        if !stage.handshake {
            if control_byte()? != b'H' {
                return Err(TestError("multi-IRQ cleanup handshake mismatch"));
            }
            stage.handshake = true;
        }
        if console::received() == stage.before && control_byte()? != stage.byte {
            return Err(TestError("multi-IRQ cleanup payload mismatch"));
        }

        let mut line = [0u8; ACK_LINE_BYTES];
        let mut length = 0;
        loop {
            let byte = control_byte()?;
            if byte == b'\n' {
                break;
            }
            if length == line.len() {
                return Err(TestError("multi-IRQ UART ACK exceeds bound"));
            }
            line[length] = byte;
            length += 1;
        }

        let digits = line[..length]
            .strip_prefix(b"KTEST IO OK ")
            .ok_or(TestError("host rejected multi-IRQ UART IO"))?;
        if digits.is_empty() {
            return Err(TestError("multi-IRQ UART ACK case absent"));
        }
        let mut id = 0u32;
        for &byte in digits {
            if !byte.is_ascii_digit() {
                return Err(TestError("multi-IRQ UART ACK case malformed"));
            }
            id = id
                .checked_mul(10)
                .and_then(|value| value.checked_add(u32::from(byte - b'0')))
                .ok_or(TestError("multi-IRQ UART ACK case overflows"))?;
        }
        if Some(id) != active_case() {
            return Err(TestError("multi-IRQ UART ACK case mismatch"));
        }

        self.stage = None;
        Ok(())
    }

    /// 先停止 IER 和同步注销自己的 action，再完成剩余 ACK 并恢复真正 default token。
    pub(super) fn quiesce(&mut self) -> TestResult {
        console::disable_interrupts();
        self.token.unregister_sync();
        if self.stage.is_some() {
            self.finish()?;
        }
        restore_default(self.source)
    }
}
