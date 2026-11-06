//! 多设备外部中断测试中的 UART 单字节硬件中断接收与宿主机握手控制辅助模块。
//!
//! # 模块职责与架构定位
//! 本模块作为 [`super::MultiIrq`] 测试夹具的子组件，负责在“两块 VirtIO 块设备 + 串口（UART）”并发外部中断测试期间，临时接管系统唯一控制台 UART 的外部中断注册项（`IrqRegistration`）。
//! 它将宿主机的测试控制协议（轮询读取 `'H'` 握手字节与 `"KTEST IO OK <case>\n"` 确认行）同被测的单字节硬件接收中断（RX IRQ）严格分离，验证 UART 接收中断与块设备中断并发挂起和派发时的正确性。
//!
//! # 协议阶段与中断门控状态机
//! ```text
//!   Uart::prepare()
//!     |-- 取出系统默认 UART IrqRegistration，关闭 UART IER，同步注销默认 handler
//!     +-- 注册测试专用 `action` 回调（初始处于 disable 门控状态）
//!           |
//!           v
//!   Uart::begin(byte)
//!     |-- 关闭 token 门控与 UART IER
//!     |-- 向宿主机输出 `# KTEST io <case> rx 1 <crc32c> 0` 请求注入单字节
//!     |-- 轮询等待并读取宿主机发回的握手字节 `'H'`
//!     +-- 开启 UART 硬件接收中断（IER），此时测试 token 仍关闭，FIFO 中挂起待测字节
//!           |
//!           v
//!   Uart::enable()
//!     +-- 开放测试 token 门控，允许外部中断控制器派发 UART IRQ
//!           |
//!           v
//!   action() [硬件中断上下文]
//!     |-- 调用 `console::ktest_uart_handler_one` 精确读取 1 字节并自动关闭 UART IER
//!     +-- 递增 `State::hits` 原子计数
//!           |
//!           v
//!   Uart::receive(byte) / Uart::finish()
//!     |-- 确认 `console::received()` 与 `State::hits` 均精确增加 1，且末字节等于 `byte`
//!     +-- 在关闭 IER 与 token 门控的状态下，轮询读完宿主机发回的 `KTEST IO OK <id>\n` 确认帧
//!           |
//!           v
//!   Uart::quiesce()
//!     +-- 同步注销测试 token，清理残留阶段，重新注册并开启系统默认 `console::uart_handler`
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 中断处理函数 [`action`] 运行在外部硬中断上下文中，内部调用 `console::ktest_uart_handler_one` 时会短暂获取控制台内部端口锁，因此测试线程在开启 token 门控期间严禁持有阻塞该 handler 的锁。
//! - `State` 通过 `Arc` 保持固定堆地址，直到 `quiesce()` 中调用 `unregister_sync()` 确保没有任何 CPU 核还在执行 [`action`] 后才随 `Uart` 释放。

use crate::irq::{IrqNumber, IrqRegistration};
use crate::ktest::{active_case, control_read_byte};
use crate::{boot, console, irq};
use ::ktest::{TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::sync::atomic::{AtomicUsize, Ordering};

/// 测试专用 UART 中断处理函数的共享计数状态。
struct State {
    /// 测试中断处理函数 [`action`] 成功从 UART FIFO 消费单字节有效载荷的次数。
    hits: AtomicUsize,
}

/// 单次宿主机 UART 字节注入测试阶段的快照与握手状态。
struct Stage {
    /// 本阶段请求宿主机发送的目标测试字节。
    byte: u8,
    /// 本阶段开始前 `console::received()` 的全局已接收字节计数快照。
    before: usize,
    /// 本阶段开始前 `State::hits` 的测试中断命中计数快照。
    hits: usize,
    /// 是否已通过轮询通道成功收到宿主机发来的 `'H'` 握手字节。
    handshake: bool,
}

/// 多设备中断测试的 UART 中断接管与协议驱动器。
pub(super) struct Uart {
    /// 与中断处理函数 [`action`] 共享的原子命中计数状态，在同步注销前始终保持存活。
    state: Arc<State>,
    /// 测试专用 UART 中断处理函数的注册凭据，用于门控开关与同步注销。
    token: IrqRegistration,
    /// 从启动信息（`BootInfo`）映射得到的 UART 外部中断号。
    source: IrqNumber,
    /// 当前正在进行的单字节接收测试阶段；未启动或已收尾时为 `None`。
    stage: Option<Stage>,
}

/// 测试专用的 UART 外部硬中断处理函数，每次触发最多消费 1 个字节并在消费后关闭 UART 接收中断使能（IER）。
///
/// # Safety
/// 1. `argument` 必须是由 [`Uart::prepare`] 传入的 `Arc::as_ptr(&state)` 转换而来的有效 `State` 指针，且在 `token.unregister_sync()` 返回前始终保持有效。
/// 2. 本函数仅由外部中断控制器在硬中断上下文中调用，调用时当前 CPU 不得持有控制台串口锁（`PORT`），且函数内部不得睡眠或动态分配内存。
unsafe fn action(argument: *mut ()) -> bool {
    // SAFETY:
    // 1. `Uart::prepare` 将 `Arc<State>` 的裸指针作为注册参数传入，且 `Uart::quiesce` 在释放 `self.state` 前会先执行 `self.token.unregister_sync()`，保证此处解引用期间 `State` 实例始终存活。
    // 2. `State` 仅包含 `AtomicUsize`，支持多线程/中断并发共享访问。
    let state = unsafe { &*argument.cast::<State>() };
    // SAFETY:
    // 1. 本函数在真实外部中断上下文中执行，未持有控制台 `PORT` 自旋锁。
    // 2. 内核初始化阶段已建立永久有效的 UART MMIO/端口映射，`ktest_uart_handler_one` 最多读取 1 字节并在读取后关闭 IER，使后续宿主机 ACK 帧不会被中断处理函数误吞。
    let handled = unsafe { console::ktest_uart_handler_one(core::ptr::null_mut()) };
    if handled {
        state.hits.fetch_add(1, Ordering::Release);
    }
    handled
}

/// 在测试中断处理函数已同步注销且 UART IER 关闭后，重新注册系统默认的控制台 UART 中断处理函数并恢复接收中断。
fn restore_default(source: IrqNumber) -> TestResult {
    // SAFETY:
    // 1. `console::uart_handler` 不解引用传入的上下文指针（传入 `null_mut()` 安全）。
    // 2. 调用本函数前已关闭 UART 硬件中断并同步注销了测试 handler，不会产生并发冲突。
    let token = unsafe { irq::register(source, console::uart_handler, core::ptr::null_mut()) }
        .map_err(|_| TestError("multi-IRQ default UART registration failed"))?;
    console::testing::restore_uart_registration(token)?;
    console::enable_receive_interrupts();
    Ok(())
}

/// 计算单字节有效载荷的 CRC-32C（Castagnoli）校验值，用于构造发送给宿主机的 `KTEST io` 指令。
fn crc(byte: u8) -> u32 {
    let mut value = 0xffff_ffffu32 ^ u32::from(byte);
    for _ in 0..8 {
        value = (value >> 1) ^ if value & 1 != 0 { 0x82f6_3b78 } else { 0 };
    }
    !value
}

/// 在 UART 硬件接收中断已关闭且测试中断 token 处于禁用状态时，以轮询方式从串口读取 1 个宿主机控制字节。
fn control_byte() -> Result<u8, TestError> {
    super::until(
        "multi-IRQ UART control input timed out",
        console::receive_pending,
    )?;
    Ok(control_read_byte())
}

impl Uart {
    /// 分配测试状态并接管系统默认 UART 中断注册项；若注册测试 handler 失败则自动恢复默认 UART 中断处理函数。
    pub(super) fn prepare() -> Result<Self, TestError> {
        // Step 1: 预先分配共享状态并解析 UART 中断源编号，确保在触碰全局默认中断注册项前完成所有可失败的内存分配。
        let state = Arc::try_new(State {
            hits: AtomicUsize::new(0),
        })
        .map_err(|_| TestError("multi-IRQ UART context allocation failed"))?;
        let source = irq::map_source(boot::info().interrupts.uart_source)
            .map_err(|_| TestError("multi-IRQ UART source invalid"))?;

        // Step 2: 取出系统默认 UART 中断注册项，关闭硬件中断使能并等待默认 handler 彻底退出。
        let initial = console::testing::take_uart_registration()?;
        console::disable_interrupts();
        initial.unregister_sync();

        // Step 3: 注册测试专用单字节中断处理函数 `action`；若注册失败则立即恢复默认注册项。
        // SAFETY:
        // 1. `state` 由 `Arc` 管理，其堆地址在 `Arc` 存活期间固定不变。
        // 2. 注册成功后 `state` 保存在返回的 `Uart` 结构体中，直到 `quiesce()` 调用 `unregister_sync()` 后才会被释放；若注册失败，`action` 从未激活。
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

    /// 返回当前平台 `BootInfo` 映射的 UART 外部中断号。
    pub(super) fn source(&self) -> IrqNumber {
        self.source
    }

    /// 启动一轮单字节 UART 硬件中断接收测试：发送 `KTEST io` 请求，轮询消费 `'H'` 握手字节，并在测试 token 保持禁用的前提下开启 UART 接收中断（IER）。
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

    /// 开放测试中断注册项的门控，允许已在 FIFO 中挂起的待测字节触发外部中断并调用 [`action`]。
    pub(super) fn enable(&self) {
        self.token.enable();
    }

    /// 等待硬件中断精确接收 1 个预期字节 `byte`，校验接收计数与中断命中次数，随后收尾消费宿主机的 `KTEST IO OK` 确认行。
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

    /// 完成当前 UART 测试阶段的协议收尾：在关闭硬件中断后清理可能残留的握手/有效载荷字节，并读取校验宿主机的 `KTEST IO OK <case_id>` 确认行。
    fn finish(&mut self) -> TestResult {
        const ACK_LINE_BYTES: usize = 64;
        console::disable_interrupts();
        let stage = self
            .stage
            .as_mut()
            .ok_or(TestError("multi-IRQ UART stage absent"))?;
        // Step 1: 若在握手完成前测试即中止，先排空宿主机已发出的 `'H'` 握手字节。
        if !stage.handshake {
            if control_byte()? != b'H' {
                return Err(TestError("multi-IRQ cleanup handshake mismatch"));
            }
            stage.handshake = true;
        }
        // Step 2: 若测试异常中断导致有效载荷字节未被硬件 IRQ 消费，由轮询通道将其读走，避免污染后续 ACK 行解析。
        if console::received() == stage.before && control_byte()? != stage.byte {
            return Err(TestError("multi-IRQ cleanup payload mismatch"));
        }

        // Step 3: 逐字节轮询读取以 '\n' 结尾的宿主机确认行，并校验 `KTEST IO OK <active_case>` 格式。
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

    /// 停机清理：关闭 UART 硬件中断并同步注销测试中断处理函数，排空未完成的宿主机协议阶段，最后恢复系统默认 UART 中断注册项。
    pub(super) fn quiesce(&mut self) -> TestResult {
        console::disable_interrupts();
        self.token.unregister_sync();
        if self.stage.is_some() {
            self.finish()?;
        }
        restore_default(self.source)
    }
}
