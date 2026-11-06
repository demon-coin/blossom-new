//! 控制台串口（UART）硬件接收中断、屏蔽挂起及中断处理函数同步注销与切换测试。
//!
//! # 模块职责与架构定位
//! 本模块隶属于启动与硬件自检子系统（`boot_cases`），验证唯一控制台串口（`console`）与外部中断分发器（`irq`）协同工作的端到端行为：
//! - **上游调用者**：由内核测试框架（`ktest`）在 `Thread` 阶段串行调度执行（声明独占 `"uart"` 资源，避免与其他串口用例争用同一物理通道）。
//! - **下游依赖**：调用 `console::testing` 暂时接管默认串口中断注册句柄，通过 `irq::register` / `IrqRegistration` 注册测试专用中断回调 [`uart_action`]，并配合宿主机评测器完成测试负载字节的收发对账。
//!
//! # 核心设计与串口测试握手协议
//! 由于内核测试日志输出、宿主机控制握手与被测串口硬件输入复用同一条物理 UART 通道，本模块采用三段式帧隔离协议，确保控制字节不会混入被测中断计数：
//!
//! ```text
//!   内核 (Uart Fixture)                                宿主机评测器 (Host Harness)
//!   -------------------                                ---------------------------
//!   1. console::disable_interrupts() 关闭 IER
//!   2. 输出 "# KTEST io <id> rx 1 <crc32> 0\n"  ────>  解析请求，准备单字节负载与确认帧
//!   3. 轮询读取控制字节并校验 == b'H'           <────  发送握手字节 'H'，随后发送 <byte> + "KTEST IO OK <id>\n"
//!   4. 开启 IER 与 IrqRegistration 准入
//!   5. 硬中断触发 uart_action:
//!      ktest_uart_handler_one 读取 1 个 <byte>
//!      并立即在 PORT 锁内将 IER 清零！           <────  <byte> 被中断精确消费 1 次；后续 ACK 留在硬件 FIFO
//!   6. disable_sync() 确认回调彻底退出
//!   7. finish_stage() 轮询读走 "KTEST IO OK <id>\n" 完成本轮对账
//! ```
//!
//! 在跨核并发注销场景（`lifecycle_concurrent`）中，主核（CPU 0）进入 [`uart_action`] 后自旋等待注册句柄被禁用，同时辅核（CPU 1）调用 `unregister_sync()`，以此验证 `unregister_sync()` 必然阻塞至在途中断回调彻底执行完毕退出后才返回，杜绝回调上下文悬垂引用（UAF）。
//!
//! # 锁获取顺序与并发契约
//! - `console::PORT` 自旋锁与 `IrqRegistration` 内部自旋锁互不嵌套：[`uart_action`] 调用的 `ktest_uart_handler_one` 在返回前已释放 `PORT` 锁；外部调用 `enable`、`disable_sync`、`unregister_sync` 时均不持有 `PORT` 锁。
//! - [`Action::registration`] 仅在中断尚未启用、无子任务的独占初始化阶段写入一次，此后所有线程与中断上下文仅通过共享引用读取，由 `IrqRegistration` 内部自旋锁保证并发安全。
//! - 夹具析构（[`Uart::tear_down`]）前必须确保并发注销子任务已完成 `join` 且所有测试 `IrqRegistration` 已同步注销，最后恢复内核默认的 `console::uart_handler`。

use crate::irq::IrqRegistration;
use crate::ktest::boot_cases::hardware::wait;
use crate::ktest::{active_case, control_read_byte, spawn_on, ChildHandle};
use crate::{arch, boot, console, irq};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::sync::Arc;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// 测试专用的串口中断回调共享上下文，记录中断命中次数、并发交错轨迹及异常违例标志。
struct Action {
    /// 当前上下文绑定的外部中断注册句柄；仅在 [`Action::register`] 独占阶段写入一次，后续只读访问。
    registration: UnsafeCell<Option<IrqRegistration>>,
    /// 标记当前上下文是否仍处于有效存活期；若在同步注销后仍被中断回调观察到为 `false`，则置位 `fault`。
    alive: AtomicBool,
    /// 是否启用跨核并发注销测试模式（为 `true` 时，中断回调会在消费字节后自旋等待外部注销信号以拉长在途窗口）。
    concurrent: AtomicBool,
    /// 并发模式下，标记中断回调已进入在途自旋窗口，通知辅核注销任务可以开始调用 `unregister_sync()`。
    entered: AtomicBool,
    /// 标记中断回调即将执行完毕退出的最后一个原子观测点，用于断言 `unregister_sync()` 返回前回调已退出。
    exited: AtomicBool,
    /// 并发模式下，标记辅核上的 `unregister_sync()` 调用已经返回；若在途回调观察到此标志为 `true` 则说明同步等待失效。
    returned: AtomicBool,
    /// 紧急停止信号，通知正在自旋等待的中断回调或辅助子任务立即中止等待并退出。
    stop: AtomicBool,
    /// 生命周期或并发同步违例标志（例如回调访问了已退役上下文、等待超时或 `unregister_sync` 提前返回）。
    fault: AtomicBool,
    /// 当前中断回调成功从硬件消费测试负载字节的累计次数。
    hits: AtomicUsize,
}
// SAFETY:
// 1. `registration` 仅在 [`Action::register`] 中满足 `Arc::strong_count == 1` 且尚未向中断控制器开放准入时写入一次；
//    启用中断及派生子任务后只做共享只读访问，且 `IrqRegistration` 自身通过内部自旋锁保证 `Sync`。
// 2. 其余所有可变状态均为原子类型（`AtomicBool` / `AtomicUsize`），且夹具直到 `unregister_sync` 完成并回收子任务后才释放 `Arc<Action>`。
unsafe impl Sync for Action {}

impl Action {
    /// 在堆上分配并初始化处于存活状态的测试中断回调上下文，使中断处理函数无需借用测试栈帧。
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

    /// 在独占 `Arc` 所有权且尚未启用串口接收中断的前提下，向外部中断域注册 [`uart_action`] 并保存注册句柄。
    fn register(owner: &mut Arc<Self>) -> TestResult {
        assert_eq!(
            Arc::strong_count(owner),
            1,
            "UART registration requires exclusive owner"
        );
        let state = &**owner;
        // SAFETY:
        // 1. 前置断言 `Arc::strong_count(owner) == 1` 且入参为 `&mut Arc<Self>`，结合本模块从不创建 `Weak` 引用，保证当前线程独占访问 `state`。
        // 2. 此时尚未向中断域注册该 `state`，不存在并发的中断回调或子任务访问，只读检查已初始化的 `Option` 安全无别名冲突。
        assert!(
            unsafe { (*state.registration.get()).is_none() },
            "UART action already installed"
        );
        let number = irq::map_source(boot::info().interrupts.uart_source)
            .map_err(|_| TestError("UART source outside domain"))?;
        // SAFETY:
        // 1. 调用方在 `set_up` 或 `lifecycle` 中已提前调用 `console::disable_interrupts()` 关闭串口中断源。
        // 2. `state` 位于 `Arc` 堆内存中地址稳定，且夹具在 `quiesce` / `lifecycle` 中必先调用 `unregister_sync()` 等待在途回调全部退出后才会释放该 `Arc`。
        // 3. `irq::register` 创建的新槽位初始为 `enabled = false`，在下方写完 `registration` 并显式 `enable()` 前不会触发 `uart_action`。
        let token = unsafe {
            irq::register(
                number,
                uart_action,
                (state as *const Self).cast_mut().cast(),
            )
        }
        .map_err(|_| TestError("UART action slots exhausted"))?;
        // SAFETY:
        // 1. 当前仍持有唯一强引用的 `&mut Arc<Self>`，且新注册的 `token` 尚未调用 `enable()`，没有任何中断回调或子任务并发读取 `registration`。
        // 2. 上方 `is_none()` 断言保证了每个 `Action` 实例生命周期内仅在此处写入一次。
        unsafe {
            *state.registration.get() = Some(token);
        }
        Ok(())
    }

    /// 获取已安装的外部中断注册句柄共享引用（调用前必须已完成 [`Action::register`]）。
    fn token(&self) -> &IrqRegistration {
        // SAFETY:
        // 1. 仅在 `Action::register` 成功写入 `Some(token)` 之后调用，此后 `registration` 槽位永不再被修改。
        // 2. 返回的共享引用生命周期受 `&self` 约束，`IrqRegistration` 内部使用自旋锁同步所有并发操作。
        unsafe {
            (*self.registration.get())
                .as_ref()
                .expect("UART token not installed")
        }
    }

    /// 发出停止通知、关闭串口硬件中断并同步注销外部中断回调，等待所有在途执行彻底退出后将上下文标为退役。
    fn stop(&self) {
        self.stop.store(true, Ordering::Release);
        console::disable_interrupts();
        // SAFETY:
        // 1. `registration` 仅在独占初始化阶段（`Action::register`）写入一次，进入运行与清理阶段后所有执行流均只读访问。
        // 2. 若 `set_up` 中途尚未调用 `register` 即触发清理，此处安全读出 `None` 并跳过注销。
        if let Some(token) = unsafe { &*self.registration.get() } {
            token.unregister_sync();
        }
        self.alive.store(false, Ordering::Release);
    }
}

/// 测试专用的 UART 外部中断处理函数：单次仅读取 1 个接收字节并立即关闭硬件 `IER`，将同通道后续的 `ACK` 帧留在硬件 FIFO 中。
///
/// 调用上下文：外部硬中断处理上下文（由 `irq::handle_external` 在描述符锁外调用，本地 CPU 中断处于关闭状态）。
///
/// # Safety
/// 1. `argument` 必须是由 [`Action::register`] 传入的、`Arc<Action>` 承载的有效非空指针。
/// 2. 在外部中断域对本回调槽位持有的活跃执行计数归零（即 `unregister_sync()` 返回）之前，调用方不得释放该 `Arc<Action>`。
unsafe fn uart_action(argument: *mut ()) -> bool {
    // Step 1: 还原共享上下文引用并校验存活标志，随后调用单字节预算入口读取 1 个串口数据并自动关闭 IER。
    // SAFETY:
    // 1. 本函数 `# Safety` 契约及 `irq` 子系统的引用计数协议保证 `argument` 指向的 `Action` 在本函数返回前始终有效。
    // 2. `Action` 实现了 `Sync`，内部字段均通过原子操作或只读共享引用访问，不存在可变别名冲突。
    let state = unsafe { &*argument.cast::<Action>() };
    if !state.alive.load(Ordering::Acquire) {
        state.fault.store(true, Ordering::Release);
    }
    // SAFETY:
    // 1. 当前处于非嵌套的 UART 外部中断回调中，未持有 `console::PORT` 自旋锁。
    // 2. `ktest_uart_handler_one` 不使用传入的参数指针，且生产控制台端口资源已在内核引导期完成初始化。
    let handled = unsafe { console::ktest_uart_handler_one(core::ptr::null_mut()) };
    if !handled {
        return false;
    }
    state.hits.fetch_add(1, Ordering::Release);

    // Step 2: 若处于跨核并发注销测试模式，先通知辅核已进入回调，再自旋等待外部禁用句柄，并在短窗口内持续监测 `unregister_sync` 是否违规提前返回。
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

/// 单轮宿主机串口 I/O 测试事务的阶段跟踪记录。
struct IoStage {
    /// 本轮请求宿主机发送的期望单字节测试负载。
    byte: u8,
    /// 发起本轮请求前 `console::received()` 的历史累计接收字节数基线。
    before: usize,
    /// 是否已经从串口轮询读到宿主机发回的起始握手字节 `'H'`。
    handshake: bool,
}

/// 串口中断与生命周期测试夹具，管理新旧两个测试回调上下文、并发注销子任务以及默认驱动恢复。
struct Uart {
    /// 初始阶段安装的第一个测试中断回调上下文（用于常规接收、屏蔽测试及注销切换中的旧实例）。
    old: Arc<Action>,
    /// 生命周期切换测试中预先分配的第二个测试中断回调上下文（用于在旧实例注销后接管待处理字节）。
    new: Arc<Action>,
    /// 跨核并发注销测试（`lifecycle_concurrent`）中派生的辅核注销子任务句柄。
    remover: Option<ChildHandle>,
    /// 当前正在进行但尚未读完宿主机 `ACK` 确认行的串口 I/O 阶段状态；空闲时为 `None`。
    stage: Option<IoStage>,
}

/// 在测试回调均已同步注销后，重新向外部中断域注册内核默认的 `console::uart_handler` 并恢复接收中断。
///
/// 调用上下文：`Thread` 阶段夹具初始化回滚或 `tear_down` 清理路径。
fn restore_default() -> TestResult {
    let number = irq::map_source(boot::info().interrupts.uart_source)
        .map_err(|_| TestError("UART source outside domain"))?;
    // SAFETY:
    // 1. 调用前测试夹具已同步注销自定义回调，控制台串口已无测试回调竞争。
    // 2. `console::uart_handler` 忽略上下文指针参数，且底层串口硬件资源在整个内核生命周期内永久有效。
    let token = unsafe { irq::register(number, console::uart_handler, core::ptr::null_mut()) }
        .map_err(|_| TestError("default UART registration failed"))?;
    console::testing::restore_uart_registration(token)?;
    console::enable_receive_interrupts();
    Ok(())
}

impl Fixture for Uart {
    /// 预先分配测试所需的回调上下文，摘除并同步注销默认串口中断处理函数，随后挂载并启用 `old` 测试回调；若中途失败则恢复默认处理函数。
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

    /// 停止 `old` 与 `new` 两个测试回调上下文并同步等待在途中断退出；若用例中途失败遗留未完成的握手阶段，则排空残留负载与 `ACK` 帧以恢复串口通道干净状态。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.old.stop();
        self.new.stop();
        if self.stage.is_some() {
            self.finish_stage()?;
        }
        Ok(())
    }

    /// 确认并发注销子任务已退出且串口 I/O 事务已结清后，释放测试上下文并将串口控制权归还给默认驱动。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        assert!(self
            .remover
            .as_ref()
            .is_none_or(|handle| handle.try_join().is_some()));
        assert!(self.stage.is_none());
        restore_default()
    }
}

/// 计算单个输入字节的标准 CRC-32C（Castagnoli，多项式 `0x82f6_3b78`）校验值，供宿主机核对请求负载。
fn crc(byte: u8) -> u32 {
    let mut value = 0xffff_ffffu32 ^ u32::from(byte);
    for _ in 0..8 {
        value = (value >> 1) ^ if value & 1 != 0 { 0x82f6_3b78 } else { 0 };
    }
    !value
}

/// 在串口接收中断已关闭的前提下，有界等待硬件接收就绪并轮询读取 1 个宿主机控制字节。
fn control_byte() -> Result<u8, TestError> {
    wait(console::receive_pending, "UART control input timed out")?;
    Ok(control_read_byte())
}

impl Uart {
    /// 启动一轮单字节串口接收测试事务：先关闭硬件 `IER`，向宿主机发送 `KTEST io` 指令并轮询消费起始握手字节 `'H'`，使紧随其后的测试负载字节留给被测路径接收。
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

    /// 结束当前串口接收测试事务：关闭硬件 `IER`，在异常清理路径下补齐未读完的 `'H'` 或未消费的负载字节，最后轮询读取并校验宿主机发回的 `KTEST IO OK <id>\n` 确认帧。
    fn finish_stage(&mut self) -> TestResult {
        // Step 1: 关闭串口硬件中断，若用例在完成握手或消费负载前提前中止，先按序读走残留的 'H' 与负载字节以对齐帧边界。
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

        // Step 2: 逐字节轮询读取以 '\n' 结尾的宿主机确认行，并校验前缀与当前活动用例编号完全一致。
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

    /// 等待指定的测试中断回调 `action` 精确接收 1 个负载字节 `byte`，同步禁用该回调以确认其已完整退出，最后完成本轮 `ACK` 帧核销。
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

    /// 执行中断处理函数注销与新句柄接管的完整生命周期验证（`concurrent = false` 为本核顺序注销，`concurrent = true` 为辅核在回调执行期间并发调用 `unregister_sync`）。
    fn lifecycle(&mut self, concurrent: bool) -> TestResult {
        // Step 1: 若为并发模式，先在 CPU 1 上派生注销子任务，等待 `old` 回调进入自旋窗口后发起 `unregister_sync()`。
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
        // Step 2: 发起第一轮负载 'L'，验证 `old` 回调成功接收字节，并完成同步注销（并发模式下等待 CPU 1 注销子任务汇合，断言回调在 `unregister_sync` 返回前已退出）。
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

        // Step 3: 将 `old` 标为退役后发起第二轮负载 'R'；此时中断源无任何已启用回调，验证字节停留在硬件 FIFO 中挂起且 `old` 不会被误触发。
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

        // Step 4: 注册并启用全新的 `new` 回调上下文，同时对已注销的旧句柄再次调用 `disable_sync` / `unregister_sync` 验证幂等且不干扰新句柄，最后确认挂起的 'R' 精确由 `new` 消费。
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

    /// 验证常规状态下宿主机发送的单字节负载由串口接收中断精确消费一次。
    ///
    /// - **测试目标**：确认外部中断路由、[`uart_action`] 单字节配额控制以及宿主机 `H`/`ACK` 帧隔离协议协同工作正常。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：测试夹具已挂载并启用 `old` 回调上下文，完成负载 `'U'` 的 `'H'` 起始握手；
    ///   2. When：开启 UART 硬件接收中断（`IER`）等待硬件触发外部中断；
    ///   3. Then：`old` 回调精确执行 1 次并读到 `'U'`，后续 `ACK` 帧留在硬件 FIFO 中由轮询核销通过。
    #[test]
    fn normal_rx(_: &mut TestContext<'_>, fixture: &mut Uart) -> TestResult {
        let old = Arc::clone(&fixture.old);
        fixture.begin_stage(b'U')?;
        console::enable_receive_interrupts();
        fixture.receive_stage(&old, b'U')?;
        assert_eq!(old.hits.load(Ordering::Acquire), 1);
        Ok(())
    }

    /// 验证外部中断源被屏蔽（Disable）期间到达的串口字节保持在硬件 FIFO 挂起，解除屏蔽后立即触发中断完成接收。
    ///
    /// - **测试目标**：确认 `IrqRegistration::disable_sync` 能有效在中断控制器层屏蔽该源且不丢失硬件 FIFO 中的待收字节，`enable` 后能立即恢复分发。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：先对 `old` 注册句柄调用 `disable_sync()` 屏蔽外部中断源，随后发起负载 `'M'` 并开启 UART 侧 `IER`；
    ///   2. When：轮询观察到串口硬件 `LSR.DATA_READY` 已置位（字节已到达 FIFO），检查此时接收计数与回调命中数均未增长，再调用 `old.token().enable()` 解除屏蔽；
    ///   3. Then：解除屏蔽后外部中断立即触发，`old` 回调精确消费字节 `'M'` 并使命中计数加 1。
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

    /// 验证单核顺序场景下同步注销旧中断回调、硬件字节无主挂起以及注册新回调接管消费的完整生命周期。
    ///
    /// - **测试目标**：确认在本地线程顺序调用 `unregister_sync()` 后，旧回调彻底不再响应后续串口输入，且新注册的 `Action` 能无损接管已在硬件 FIFO 中挂起的字节，同时对已关闭的旧句柄重复禁用/注销保持幂等无害。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：`old` 回调正常接收首轮字节 `'L'`，随后调用 `old.token().unregister_sync()` 将其注销并标为退役；
    ///   2. When：发起第二轮字节 `'R'` 并等待其到达硬件 FIFO，确认无回调消费后注册并启用 `new` 回调，期间再次调用旧句柄的 `disable_sync()` 与 `unregister_sync()`；
    ///   3. Then：旧句柄操作不影响新句柄状态，挂起的 `'R'` 由 `new` 回调精确消费 1 次，且 `old` 未发生任何违例或额外命中。
    #[test]
    fn lifecycle_local(_: &mut TestContext<'_>, fixture: &mut Uart) -> TestResult {
        fixture.lifecycle(false)
    }

    /// 验证多核并发场景下辅核在主核中断处理函数正在执行时调用 `unregister_sync()` 的同步屏障保证与槽位安全复用。
    ///
    /// - **测试目标**：确认当 CPU 0 正在锁外执行 [`uart_action`] 时，CPU 1 并发发起的 `unregister_sync()` 必然阻塞等待至 CPU 0 彻底退出回调后才返回，防止提前释放回调上下文导致悬垂指针（UAF），并验证此后新注册回调接管挂起字节及旧句柄幂等性。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：至少 2 个在线 CPU；在 CPU 1 上派生注销任务，随后触发字节 `'L'` 使 CPU 0 进入 `old` 中断回调并停留在受控观测窗口；
    ///   2. When：CPU 1 检测到 `entered == true` 后调用 `old.token().unregister_sync()`，CPU 0 观察到句柄禁用后在短暂自旋监测期内持续检查 `alive` 与 `returned` 标志，随后退出回调；接着发送字节 `'R'` 并切换至新注册的 `new` 回调；
    ///   3. Then：在 CPU 0 退出回调前 `unregister_sync()` 从未提前返回（`fault == false`），CPU 1 注销任务顺利汇合，且后续挂起的 `'R'` 由 `new` 回调精确消费 1 次。
    #[test]
    #[ktest(min_cpus = 2)]
    fn lifecycle_concurrent(_: &mut TestContext<'_>, fixture: &mut Uart) -> TestResult {
        fixture.lifecycle(true)
    }
}
