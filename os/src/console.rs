//! 选定串口的低层拥有者；正常访问共享一把端口短锁，故障输出不等待业务锁。

use crate::sync::SpinLock;
use crate::wait::WaitQueue;
use core::convert::TryFrom;
use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use spin::Once;

struct PortResources {
    base: usize,
    width: u32,
    shift: u32,
    baud_base: u32,
    dw_apb: bool,
    /// 平台为控制台声明的工作波特率；线路速率由固件建立，内核首次打开不改写。
    console_baud: u32,
}

static RESOURCES: Once<PortResources> = Once::new();

const DATA: usize = 0;
const IER: usize = 1;
const FCR: usize = 2;
const IIR: usize = 2;
const LCR: usize = 3;
const LSR: usize = 5;
const MSR: usize = 6;
pub(crate) const DATA_READY: u8 = 1;
pub(crate) const UART_LSR_OVERRUN: u8 = 1 << 1;
pub(crate) const UART_LSR_PARITY: u8 = 1 << 2;
pub(crate) const UART_LSR_FRAME: u8 = 1 << 3;
pub(crate) const UART_LSR_BREAK: u8 = 1 << 4;
const TRANSMIT_READY: u8 = 0x20;
const RECEIVE_INTERRUPT: u8 = 1;
const TRANSMIT_INTERRUPT: u8 = 2;
#[cfg(feature = "ktest")]
const THRE_INTERRUPT: u8 = 2;
const TX_BUDGET: usize = 16;
const MCR: usize = 4;
// DLL/DLM 与 DATA/IER 共用偏移，访问前必须先置 LCR 的 DLAB，之后立即恢复。
const DLL: usize = 0;
const DLM: usize = 1;
const DLAB: u8 = 0x80;
const TRANSMITTER_EMPTY: u8 = 0x40;
const DW_USR: usize = 0x1f;
const DW_BUSY: u8 = 1;
const LOOPBACK: u8 = 1 << 4;
const LCR_STICK_PARITY: u8 = 1 << 5;
const DATA_TERMINAL_READY: u8 = 1;
const REQUEST_TO_SEND: u8 = 2;
pub(crate) const UART_LCR_WLEN5: u8 = 0;
pub(crate) const UART_LCR_WLEN6: u8 = 1;
pub(crate) const UART_LCR_WLEN7: u8 = 2;
pub(crate) const UART_LCR_WLEN8: u8 = 3;
pub(crate) const UART_LCR_STOP: u8 = 1 << 2;
pub(crate) const UART_LCR_PARITY: u8 = 1 << 3;
pub(crate) const UART_LCR_EPAR: u8 = 1 << 4;
const FCR_ENABLE_FIFO: u8 = 1;
const FCR_CLEAR_RECEIVER: u8 = 1 << 1;
const FCR_CLEAR_TRANSMITTER: u8 = 1 << 2;
// 保留 IIR 的 no-interrupt 位，DW busy-detect 的编码也使用最低位。
const IIR_STATUS_MASK: u8 = 0x0f;
const IIR_NO_INTERRUPT: u8 = 1;
const IIR_MODEM_STATUS: u8 = 0;
const IIR_BUSY: u8 = 7;
const IIR_RX_TIMEOUT: u8 = 0x0c;
const UART_CLOCK_DIVISOR: u64 = 16;
const QEMU_RISCV_BAUD_BASE: u32 = 399_193;
const QEMU_LOONGARCH_BAUD_BASE: u32 = 115_200;
const RX_IRQ_BUDGET: usize = 16;

// 整条应急消息最多读取这些次 LSR，不依赖时钟或其他 CPU 的进度。
const EMERGENCY_POLL_BUDGET: usize = 10_000;

static PORT: SpinLock<RxState> = SpinLock::new(RxState::new());
static TX_WAIT: WaitQueue = WaitQueue::new();
static RECEIVED: AtomicUsize = AtomicUsize::new(0);
// MCR 的 DTR/RTS 位在部分板卡不可回读，差异只报告一次以免每次打开都刷屏。
static READBACK_WARNED: AtomicBool = AtomicBool::new(false);
static LAST_BYTE: AtomicU8 = AtomicU8::new(0);

/// 中断已从 FIFO 取出的一个真实字节及其同步读取的 LSR。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RxSample {
    /// 硬件 DATA 原始字节，不把 NUL 或 FF 当作空位。
    pub byte: u8,
    /// 读取 DATA 前同步取得的 LSR，携带该字节的线路错误位。
    pub status: u8,
}

/// TTY 预先验证的串口寄存器目标；调用者保证分频可表示、行控已合法且 DLAB 为零。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SerialConfig {
    /// 目标分频值，写入 DLL/DLM。
    pub divisor: u16,
    /// 目标 LCR，携带 5–8 位字长、停止位与校验位，不含 DLAB。
    pub line_control: u8,
    /// CREAD 是否要求打开接收中断。
    pub receiver: bool,
    /// 是否置 DTR 输出线。
    pub dtr: bool,
    /// 是否置 RTS 输出线。
    pub rts: bool,
}

/// 端口锁内的硬件状态；分频来自最近一次已确认配置，其余字段直接回读。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SerialSnapshot {
    /// 最近一次 DLL/DLM 真实回读的分频值。
    pub divisor: u16,
    /// 回读 LCR。
    pub line_control: u8,
    /// 回读 IER。
    pub ier: u8,
    /// 回读 MCR。
    pub mcr: u8,
    /// 回读 LSR。
    pub lsr: u8,
    /// 本平台固定 QEMU 串口 baudbase。
    pub baud_base: u32,
}

/// 软件接收环容量；固定项数，不随终端模式或测试阶段改变。
const RX_CAPACITY: usize = 1024;

/// 端口锁内唯一的接收状态；IRQ 和 DW 配置隔离窗采样，行规程工作消费原环。
struct RxState {
    samples: [RxSample; RX_CAPACITY],
    head: usize,
    tail: usize,
    len: usize,
    overrun: usize,
    /// 最近一次受确认的分频和 LCR，避免相同线路参数反复中断收发。
    line_settings: Option<(u16, u8)>,
    #[cfg(feature = "ktest")]
    quota: Option<usize>,
    #[cfg(feature = "ktest")]
    test_control_owned: bool,
    #[cfg(feature = "ktest")]
    tty_attached: bool,
}

impl RxState {
    /// 建立空接收环；线路回读、测试配额、控制声明与 TTY 接收附着初始均为空。
    const fn new() -> Self {
        Self {
            samples: [RxSample { byte: 0, status: 0 }; RX_CAPACITY],
            head: 0,
            tail: 0,
            len: 0,
            overrun: 0,
            line_settings: None,
            #[cfg(feature = "ktest")]
            quota: None,
            #[cfg(feature = "ktest")]
            test_control_owned: false,
            #[cfg(feature = "ktest")]
            tty_attached: false,
        }
    }

    /// 环未满时入队一个中断采样，环满时只计真实丢弃。
    fn push(&mut self, sample: RxSample) {
        if self.len < RX_CAPACITY {
            self.samples[self.head] = sample;
            self.head = (self.head + 1) % RX_CAPACITY;
            self.len += 1;
        } else {
            self.overrun += 1;
        }
    }

    /// 按生产顺序取出最多调用者缓冲区长度的采样，返回实际取出数。
    fn pop(&mut self, out: &mut [RxSample]) -> usize {
        let mut count = 0;
        while count < out.len() && 0 < self.len {
            out[count] = self.samples[self.tail];
            self.tail = (self.tail + 1) % RX_CAPACITY;
            self.len -= 1;
            count += 1;
        }
        count
    }

    /// 消费一个测试配额；收齐本段最后一个字节时清配额并返回真。
    #[cfg(feature = "ktest")]
    fn consume_quota(&mut self) -> bool {
        if let Some(remaining) = self.quota {
            if 1 < remaining {
                self.quota = Some(remaining - 1);
                false
            } else {
                self.quota = None;
                true
            }
        } else {
            false
        }
    }
}

/// 固件资源发表前仅使用编译平台的早期 UART 几何；正常路径消费同一设备快照。
fn register(offset: usize) -> (usize, u32) {
    if let Some(resource) = RESOURCES.get() {
        return (
            crate::arch::mmio_address(resource.base) + (offset << resource.shift),
            resource.width,
        );
    }
    let shift = if cfg!(blossom_platform = "visionfive2") {
        2
    } else {
        0
    };
    let width = if cfg!(blossom_platform = "visionfive2") {
        4
    } else {
        1
    };
    (
        crate::arch::mmio_address(crate::platform::EARLY_UART_BASE) + (offset << shift),
        width,
    )
}

/// 按资源声明的访问宽度读取寄存器；窄 UART 数据位保留低八位。
fn read_register(offset: usize) -> u8 {
    let (address, width) = register(offset);
    // SAFETY: 早期编译资源与已验证 DT 资源指向同一永久 UART 映射；
    // 宽度仅为 1/4，四字节模式的基址和 stride 均对齐，offset 来自有效寄存器编号。
    unsafe {
        match width {
            1 => (address as *const u8).read_volatile(),
            4 => (address as *const u32).read_volatile() as u8,
            _ => unreachable!(),
        }
    }
}

/// 按 UART 真实总线宽度写入寄存器，不把四字节设备拆成字节访问。
fn write_register(offset: usize, value: u8) {
    let (address, width) = register(offset);
    // SAFETY: 寄存器范围、对齐和永久映射与 read_register 相同；
    // 普通访问由 PORT 串行，故障路径不建立设备内存引用。
    unsafe {
        match width {
            1 => (address as *mut u8).write_volatile(value),
            4 => (address as *mut u32).write_volatile(u32::from(value)),
            _ => unreachable!(),
        }
    }
}

/// BSP 在资源和永久映射建立后一次发表 UART 几何与实际输入时钟。
pub(crate) fn configure_resources(info: &crate::boot::BootInfo) {
    assert!(
        RESOURCES.get().is_none(),
        "UART resources already configured"
    );
    assert_eq!(info.interrupts.uart.start, crate::platform::EARLY_UART_BASE);
    assert!(matches!(info.uart.reg_io_width, 1 | 4));
    let baud_base = if info.platform == crate::boot::Platform::QemuVirt {
        if cfg!(target_arch = "riscv64") {
            QEMU_RISCV_BAUD_BASE
        } else {
            QEMU_LOONGARCH_BAUD_BASE
        }
    } else {
        let clock = info
            .uart
            .clock_frequency
            .expect("UART input clock rate is unavailable");
        u32::try_from(clock / UART_CLOCK_DIVISOR).expect("UART baud base exceeds u32")
    };
    assert!(0 < baud_base, "UART input clock is zero");
    RESOURCES.call_once(|| PortResources {
        base: info.interrupts.uart.start,
        width: info.uart.reg_io_width,
        shift: info.uart.reg_shift,
        baud_base,
        dw_apb: info.uart.dw_apb,
        console_baud: info.uart.baud,
    });
}

/// 判断当前串口是否具有 DesignWare 的 USR 与 busy-detect 语义。
fn dw_apb() -> bool {
    RESOURCES
        .get()
        .map_or(cfg!(blossom_platform = "visionfive2"), |resource| {
            resource.dw_apb
        })
}

/// 持有 PORT 且 DW 已隔离外部 RX 并确认空闲时切换 LCR，禁止未进入 DLAB 就访问分频别名。
fn write_line_control(value: u8) {
    write_register(LCR, value);
    if dw_apb() {
        assert_eq!(
            read_register(LCR) & !LCR_STICK_PARITY,
            value & !LCR_STICK_PARITY,
            "UART line control write was not accepted"
        );
    }
}

/// 保存调用者读取的 LSR 及对应 RBR 字节；DLAB 须清零且没有测试控制者。
/// LSR 错误位读后清，必须消费该快照后才能再读 LSR；本入口不增加 IRQ 计数。
fn preserve_pending_receive(port: &mut RxState, status: u8) -> bool {
    if status & DATA_READY == 0 {
        return false;
    }
    let byte = read_register(DATA);
    port.push(RxSample { byte, status });
    true
}

/// 持有 PORT 时仅更新真正变化的线路参数，返回实际分频及需要通知的接收进展。
fn configure_line(port: &mut RxState, config: SerialConfig, saved_mcr: u8) -> (u16, bool) {
    if port.line_settings == Some((config.divisor, config.line_control)) {
        return (config.divisor, false);
    }
    let mut received = false;
    if dw_apb() {
        #[cfg(feature = "ktest")]
        assert!(
            !port.test_control_owned,
            "DW UART line reconfiguration requires released receive control"
        );
        // 只排空已经进入硬件的 TX；PORT 阻止其它输出者继续填充，不等待软件输出队列。
        loop {
            let status = read_register(LSR);
            received |= preserve_pending_receive(port, status);
            if status & TRANSMITTER_EMPTY != 0 {
                break;
            }
            spin_loop();
        }
        // BUSY 包含 RBR 数据。先隔离外部 RX，再保存残留采样，使空闲等待不依赖被阻止的 IRQ。
        write_register(MCR, saved_mcr | LOOPBACK);
        loop {
            let status = read_register(LSR);
            received |= preserve_pending_receive(port, status);
            if read_register(DW_USR) & DW_BUSY == 0 {
                break;
            }
            spin_loop();
        }
    }
    write_line_control(config.line_control | DLAB);
    write_register(DLL, config.divisor as u8);
    write_register(DLM, (config.divisor >> 8) as u8);
    let low = read_register(DLL);
    let high = read_register(DLM);
    write_line_control(config.line_control);
    let divisor = u16::from(low) | (u16::from(high) << 8);
    port.line_settings = Some((divisor, read_register(LCR)));
    (divisor, received)
}

/// 已停其它核和本核事件后关闭 UART 中断并排空末尾停止诊断；不确认物理断电，返回后不得再启动输出者。
#[cfg(not(feature = "ktest"))]
pub(crate) fn board_stop() {
    let platform = match crate::platform::current() {
        crate::boot::Platform::QemuVirt => return,
        crate::boot::Platform::VisionFive2 => "visionfive2",
        crate::boot::Platform::Ls2k1000la => "ls2k1000la",
    };
    let _port = PORT.lock();
    write_register(IER, 0);
    PortWriter
        .write_fmt(format_args!(
            "Board {}: other online CPUs stopped; local event sources disabled.\n",
            platform
        ))
        .expect("board stop diagnostic failed");
    while read_register(LSR) & TRANSMITTER_EMPTY == 0 {
        spin_loop();
    }
}

/// 当前核的事件源已关闭时输出有界致命停止诊断；不等待端口锁，也不声称其它核或设备已停止。
#[cfg(feature = "ktest")]
pub(crate) fn board_stop_current_cpu() {
    let platform = match crate::platform::current() {
        crate::boot::Platform::QemuVirt => return,
        crate::boot::Platform::VisionFive2 => "visionfive2",
        crate::boot::Platform::Ls2k1000la => "ls2k1000la",
    };
    write_register(IER, 0);
    let _ = emergency_writer().write_fmt(format_args!(
        "Board {}: fatal stop on current CPU; other CPUs and devices may remain active.\n",
        platform
    ));
}

/// 等待真实 TX 空位后写一个字节，保持输出顺序。
fn put_byte(byte: u8) {
    while read_register(LSR) & TRANSMIT_READY == 0 {
        spin_loop();
    }
    write_register(DATA, byte);
}

struct PortWriter;

impl Write for PortWriter {
    /// 按原始 UTF-8 字节输出一个有限字符串，不申请内存。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for byte in text.bytes() {
            put_byte(byte);
        }
        Ok(())
    }
}

struct EmergencyWriter<R, W> {
    remaining: usize,
    read_status: R,
    write_byte: W,
}

impl<R: FnMut() -> u8, W: FnMut(u8)> Write for EmergencyWriter<R, W> {
    /// 每次状态读取消耗整条消息的共享预算；耗尽后丢弃剩余字节并报告格式化错误。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for byte in text.bytes() {
            loop {
                if self.remaining == 0 {
                    return Err(fmt::Error);
                }
                self.remaining -= 1;
                if (self.read_status)() & TRANSMIT_READY != 0 {
                    break;
                }
                spin_loop();
            }
            (self.write_byte)(byte);
        }
        Ok(())
    }
}

/// 在栈上建立一次应急输出预算；UART 映射由所有 Rust 引导入口建立并永久保留。
fn emergency_writer() -> impl Write {
    EmergencyWriter {
        remaining: EMERGENCY_POLL_BUDGET,
        read_status: || read_register(LSR),
        write_byte: |byte| write_register(DATA, byte),
    }
}

/// 独立验证一种真实 writer 预算场景，不将多个旧断言组绑定为一个测试实例。
#[cfg(feature = "ktest")]
fn emergency_budget(mode: u8) -> bool {
    match mode {
        0 => {
            let mut reads = 0;
            let mut writes = 0;
            let stalled = {
                let mut writer = EmergencyWriter {
                    remaining: EMERGENCY_POLL_BUDGET,
                    read_status: || {
                        reads += 1;
                        0
                    },
                    write_byte: |_| writes += 1,
                };
                writer.write_str("a").is_err() && writer.write_str("b").is_err()
            };
            stalled && reads == EMERGENCY_POLL_BUDGET && writes == 0
        }
        1 => {
            let mut reads = 0;
            let mut output = [0; 4];
            let mut written = 0;
            let shared = {
                let mut writer = EmergencyWriter {
                    remaining: 5,
                    read_status: || {
                        reads += 1;
                        if reads == 1 || reads == 4 {
                            0
                        } else {
                            TRANSMIT_READY
                        }
                    },
                    write_byte: |byte| {
                        if written < output.len() {
                            output[written] = byte;
                        }
                        written += 1;
                    },
                };
                writer.write_str("ab").is_ok()
                    && writer.write_str("cd").is_err()
                    && writer.write_str("e").is_err()
            };
            shared && reads == 5 && written == 3 && output == *b"abc\0"
        }
        2 => {
            let mut reads = 0;
            let mut output = [0; 4];
            let mut written = 0;
            let ready = {
                let mut writer = EmergencyWriter {
                    remaining: 3,
                    read_status: || {
                        reads += 1;
                        TRANSMIT_READY
                    },
                    write_byte: |byte| {
                        if written < output.len() {
                            output[written] = byte;
                        }
                        written += 1;
                    },
                };
                writer.write_str("ab").is_ok() && writer.write_str("cd").is_err()
            };
            ready && reads == 3 && written == 3 && output == *b"abc\0"
        }
        _ => false,
    }
}

/// 接管中断与 FIFO；DW 保留固件线路参数，直到 TTY 在正式配置窗内更新。
pub fn initialize() {
    let _port = PORT.lock();
    write_register(IER, 0);
    if !dw_apb() {
        write_line_control(UART_LCR_WLEN8);
    }
    write_register(
        FCR,
        FCR_ENABLE_FIFO | FCR_CLEAR_RECEIVER | FCR_CLEAR_TRANSMITTER,
    );
}

/// 引导阶段同步输出，不获取任务/PID、分配内存或重新探测硬件。
pub fn early_write(arguments: fmt::Arguments<'_>) {
    let _port = PORT.lock();
    PortWriter
        .write_fmt(arguments)
        .expect("UART formatting failed");
}

/// 测试控制者轮询消费 RX；调用前未开放接收中断，或已在同一端口锁下关闭 IER。
/// 已登记的 action 不得在本次控制字节握手期间重新开启 RX，避免与 FIFO 消费竞争。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_read_byte() -> u8 {
    loop {
        let _port = PORT.lock();
        if read_register(LSR) & DATA_READY != 0 {
            return read_register(DATA);
        }
        drop(_port);
        spin_loop();
    }
}

/// 真实 H 已由默认 RX IRQ 消费后，只关闭接收中断，让后续 ACK 留在 FIFO。
/// 同一 fixture 独占控制输入直到 ACK 轮询完成；TX 的实际中断与唤醒路径保持开放。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_disable_receive_interrupts() {
    let _port = PORT.lock();
    write_register(IER, read_register(IER) & !RECEIVE_INTERRUPT);
}

/// 真实故障控制者使用整条消息的有界预算，不等待 PORT 或任何任务进度。
#[cfg(feature = "ktest")]
pub(crate) fn emergency_write(arguments: fmt::Arguments<'_>) {
    let _ = emergency_writer().write_fmt(arguments);
}

/// 在端口短锁内最多提交十六个实际可接受字节，无等待、格式化或换行转换；接受字节后在锁外提示行规程复查实际发送队列。
pub fn try_write(bytes: &[u8]) -> usize {
    let accepted = {
        let _port = PORT.lock();
        let mut accepted = 0;
        for &byte in bytes.iter().take(TX_BUDGET) {
            if read_register(LSR) & TRANSMIT_READY == 0 {
                break;
            }
            write_register(DATA, byte);
            accepted += 1;
        }
        #[cfg(feature = "ktest")]
        {
            testing::COUNTERS
                .accepted
                .fetch_add(accepted, Ordering::Relaxed);
            if accepted == 0 && !bytes.is_empty() {
                testing::COUNTERS
                    .zero_writes
                    .fetch_add(1, Ordering::Relaxed);
            }
        }
        accepted
    };
    // klog 与行规程共享同一端口；THRE 中断可能已被另一方关闭，成功写入后必须提示复查实际队列。
    if 0 < accepted {
        crate::fs::tty::schedule_transmit();
    }
    accepted
}

/// 返回永久发送等待源；登记节点后才允许使能并复查硬件条件。
pub(crate) fn tx_wait_queue() -> &'static WaitQueue {
    &TX_WAIT
}

/// 登记者在同一端口锁内合并发送使能并重读真实空位，已有空位时撤销无必要通知。
pub(crate) fn arm_tx_interrupt() -> bool {
    let _port = PORT.lock();
    let ier = read_register(IER);
    write_register(IER, ier | TRANSMIT_INTERRUPT);
    let ready = read_register(LSR) & TRANSMIT_READY != 0;
    if ready {
        write_register(IER, ier & !TRANSMIT_INTERRUPT);
    }
    ready
}

/// 开启真实接收中断，源的注册与屏蔽由 IRQ 描述符负责。
pub fn enable_receive_interrupts() {
    let _port = PORT.lock();
    write_register(IER, read_register(IER) | RECEIVE_INTERRUPT);
}

/// 在同步注销 action 前停止 UART 的 RX/TX 中断产生，不消费 FIFO 中的待收字节。
#[cfg(feature = "ktest")]
pub fn disable_interrupts() {
    let _port = PORT.lock();
    write_register(IER, 0);
}

/// 返回硬件资源的真实 baudbase；QEMU 的 serial_mm_init 参数不等同 DT 时钟除十六。
pub fn baud_base() -> u32 {
    RESOURCES
        .get()
        .expect("UART resources not configured")
        .baud_base
}

/// 返回平台为控制台声明的工作波特率，供首次打开时保持既有线路速率。
pub fn console_baud() -> u32 {
    RESOURCES
        .get()
        .expect("UART resources not configured")
        .console_baud
}

/// 只读真实 LSR 的 TEMT 位；不用时钟延时代替硬件发送空标志。
pub fn transmitter_empty() -> bool {
    let _port = PORT.lock();
    read_register(LSR) & TRANSMITTER_EMPTY != 0
}

/// 从软件接收环取出最多调用者缓冲区长度的真实采样，不触碰硬件 FIFO。
pub fn read_received(samples: &mut [RxSample]) -> usize {
    PORT.lock().pop(samples)
}

/// 在端口短锁内丢弃软件接收环全部未消费采样；仅清环指针与长度，不读硬件 FIFO，不改测试控制、配额、丢弃计数、接收统计与发送路径。
pub fn discard_received() {
    let mut port = PORT.lock();
    port.head = 0;
    port.tail = 0;
    port.len = 0;
}

/// 在 PORT 内提交预验证的串口参数，恢复实际 MCR/IER 后才允许普通 DATA/IER 访问；配置窗采样在锁外通知原行规程工作。
pub fn configure_serial(config: SerialConfig) -> SerialSnapshot {
    assert_eq!(
        config.line_control & DLAB,
        0,
        "UART line control must not set DLAB"
    );
    let mut port = PORT.lock();
    let saved_ier = read_register(IER);
    let saved_mcr = read_register(MCR);
    write_register(IER, 0);
    let (divisor, received) = configure_line(&mut port, config, saved_mcr);
    let mut mcr = saved_mcr & !(DATA_TERMINAL_READY | REQUEST_TO_SEND);
    if config.dtr {
        mcr |= DATA_TERMINAL_READY;
    }
    if config.rts {
        mcr |= REQUEST_TO_SEND;
    }
    write_register(MCR, mcr);
    #[cfg(feature = "ktest")]
    let effective_receiver = if port.test_control_owned {
        config.receiver && port.quota.is_some()
    } else {
        config.receiver
    };
    #[cfg(not(feature = "ktest"))]
    let effective_receiver = config.receiver;
    let mut ier = saved_ier & TRANSMIT_INTERRUPT;
    if effective_receiver {
        ier |= RECEIVE_INTERRUPT;
    }
    write_register(IER, ier);
    let snapshot = SerialSnapshot {
        divisor,
        line_control: read_register(LCR),
        ier: read_register(IER),
        mcr: read_register(MCR),
        lsr: read_register(LSR),
        baud_base: baud_base(),
    };
    // DLAB 未清会让 DATA/IER 变成分频别名，后续输出与配置都会落到错误寄存器；
    // 该后置条件必须在释放端口锁前成立，避免其它消费者取得端口后继续用错寄存器组。
    assert_eq!(
        snapshot.line_control & DLAB,
        0,
        "UART line control left DLAB set"
    );
    drop(port);
    if received {
        crate::fs::tty::schedule_receive();
    }
    // 常规配置不以回读相等作为致命条件；保留完整差异作为硬件事实。
    if (snapshot.divisor != config.divisor
        || snapshot.line_control != config.line_control
        || snapshot.ier != ier
        || snapshot.mcr != mcr)
        && !READBACK_WARNED.swap(true, Ordering::Relaxed)
    {
        crate::early_println!(
            "UART readback differs exp div={} lc={:#x} ier={:#x} mcr={:#x} got div={} lc={:#x} ier={:#x} mcr={:#x}",
            config.divisor,
            config.line_control,
            ier,
            mcr,
            snapshot.divisor,
            snapshot.line_control,
            snapshot.ier,
            snapshot.mcr,
        );
    }
    snapshot
}

/// 先核对本端口真实 IIR，仅处理自身的有限 RX/TX，并在锁外通知等待者。
///
/// # Safety
/// 只作为当前 UART 的 IRQ action 调用；端口映射已建立，当前是非嵌套硬中断，
/// 且调用者没有持有 PORT。此 handler 不使用 context。
pub unsafe fn uart_handler(_context: *mut ()) -> bool {
    handle_uart::<RX_IRQ_BUDGET, false>()
}

/// 测试 action 仅消费一个真实 payload 字节，在同一端口锁内立即关闭 RX/TX 中断。
///
/// # Safety
/// 仅作为已登记的真实 UART IRQ action 调用，遵守 uart_handler 的入口要求。
/// 原 default action 已停止并同步注销；调用者先轮询消费 H，再开启 RX，且本阶段
/// 仅允许一个 payload 字节，直到轮询 ACK 完成前不得再次开启 UART 中断。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn ktest_uart_handler_one(_context: *mut ()) -> bool {
    handle_uart::<1, true>()
}

/// 两个入口共享实际 IIR/RX/TX 流程；普通构建始终将真实采样送入行规程，测试构建只在 TTY 接收附着时入软件环并在锁外通知，原计数、配额和发送流程保持独立。
#[inline(always)]
fn handle_uart<const RX_BUDGET: usize, const STOP_RX: bool>() -> bool {
    let (produced, notify_tx) = {
        let mut port = PORT.lock();
        let first_interrupt = read_register(IIR) & IIR_STATUS_MASK;
        if first_interrupt == IIR_BUSY && dw_apb() {
            let _ = read_register(DW_USR);
            return true;
        }
        if first_interrupt & IIR_NO_INTERRUPT != 0 {
            return false;
        }
        if first_interrupt == IIR_RX_TIMEOUT && dw_apb() && read_register(LSR) & DATA_READY == 0 {
            let _ = read_register(DATA);
            return true;
        }
        if first_interrupt == IIR_MODEM_STATUS {
            let _ = read_register(MSR);
        }
        let mut produced = 0;
        #[cfg(feature = "ktest")]
        let tty_attached = port.tty_attached;
        #[cfg(not(feature = "ktest"))]
        let tty_attached = true;
        if read_register(IER) & RECEIVE_INTERRUPT != 0 {
            for _ in 0..RX_BUDGET {
                let status = read_register(LSR);
                if status & DATA_READY == 0 {
                    break;
                }
                let byte = read_register(DATA);
                if tty_attached {
                    port.push(RxSample { byte, status });
                    produced += 1;
                }
                if STOP_RX {
                    // 必须紧跟 DATA 读取并留在 PORT 锁内；不得轮询或读取后续 ACK 字节。
                    write_register(IER, 0);
                }
                #[cfg(feature = "ktest")]
                let quota_done = port.consume_quota();
                LAST_BYTE.store(byte, Ordering::Relaxed);
                RECEIVED.fetch_add(1, Ordering::Release);
                #[cfg(feature = "ktest")]
                if quota_done {
                    // 本段最后一个字节已收齐；同锁只关接收位，保留发送中断。
                    write_register(IER, read_register(IER) & !RECEIVE_INTERRUPT);
                    break;
                }
            }
        }
        let last_interrupt = read_register(IIR) & IIR_STATUS_MASK;
        let ier = read_register(IER);
        let ready = read_register(LSR) & TRANSMIT_READY != 0;
        let notify_tx = ier & TRANSMIT_INTERRUPT != 0 && ready;
        if notify_tx {
            write_register(IER, ier & !TRANSMIT_INTERRUPT);
            #[cfg(feature = "ktest")]
            if first_interrupt == THRE_INTERRUPT || last_interrupt == THRE_INTERRUPT {
                testing::COUNTERS
                    .tx_interrupts
                    .fetch_add(1, Ordering::Relaxed);
            }
        }
        // IIR 读取确认真实 THRE；接收优先级较高时仍以真实 LSR 关闭已满足的 TX 请求。
        let _ = (first_interrupt, last_interrupt);
        (produced, notify_tx)
    };
    if 0 < produced {
        crate::fs::tty::schedule_receive();
    }
    if notify_tx {
        let woken = TX_WAIT.wake_all();
        #[cfg(feature = "ktest")]
        testing::COUNTERS.wakes.fetch_add(woken, Ordering::Relaxed);
        let _ = woken;
        crate::fs::tty::schedule_transmit();
    }
    true
}

/// 返回由真实接收中断消费的字节数量。
#[cfg(feature = "ktest")]
pub fn received() -> usize {
    RECEIVED.load(Ordering::Acquire)
}

/// 返回最后一个已经由中断消费的原始字节。
#[cfg(feature = "ktest")]
pub fn last_byte() -> u8 {
    LAST_BYTE.load(Ordering::Relaxed)
}

/// 测试仅观察硬件已到达的数据，不从轮询路径消费 RX。
#[cfg(feature = "ktest")]
pub fn receive_pending() -> bool {
    let _port = PORT.lock();
    read_register(LSR) & DATA_READY != 0
}

/// 在共享轮询预算内记录真实致命异常现场并停止，不等待故障核持有的锁。
#[cfg(not(feature = "ktest"))]
pub fn fatal(pc: usize, sp: usize, cause: usize, bad_address: usize) -> ! {
    let _ = crate::arch::irq_save();
    let _ = emergency_writer().write_fmt(format_args!(
        "FATAL pc={:#x} sp={:#x} cause={:#x} bad={:#x}\n",
        pc, sp, cause, bad_address,
    ));
    loop {
        crate::arch::wait_for_interrupt();
    }
}

/// 在共享轮询预算内打印内部不变量失败的原始位置和原因，随后永久停止当前核。
#[cfg(not(feature = "ktest"))]
pub fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    let _ = crate::arch::irq_save();
    let _ = emergency_writer().write_fmt(format_args!("FAIL panic: {}\n", info));
    loop {
        crate::arch::wait_for_interrupt();
    }
}

/// 使用唯一 UART 端口输出引导阶段换行信息。
#[macro_export]
macro_rules! early_println {
    () => { $crate::console::early_write(format_args!("\n")) };
    ($($argument:tt)*) => {
        $crate::console::early_write(format_args!("{}\n", format_args!($($argument)*)))
    };
}

/// 测试观察唯一真实端口活动；ktest 另保存可移动的 default action 责任，不建立第二套驱动。
#[cfg(feature = "ktest")]
pub mod testing {
    use super::*;

    #[cfg(feature = "ktest")]
    static UART_REGISTRATION: SpinLock<Option<crate::irq::IrqRegistration>> = SpinLock::new(None);

    /// 取走唯一 default UART action 的真实 token，不改变设备 IER 或 action 准入。
    ///
    /// 仅 Thread 控制者调用；同 token 暂停时须 disable_sync 后归还，替换 action 时须
    /// unregister_sync 后登记并归还新 token。空槽返回错误；不得持有 PORT 或回调退出所需锁。
    #[cfg(feature = "ktest")]
    pub fn take_uart_registration() -> Result<crate::irq::IrqRegistration, ::ktest::TestError> {
        UART_REGISTRATION.lock().take().ok_or(::ktest::TestError(
            "default UART registration is not installed",
        ))
    }

    /// 首次登记、同步暂停后归还同一 token，或替换 action 后归还新 default token，并开放其准入。
    ///
    /// 传入 token 必须属于当前 UART 的永久 uart_handler/null context。同 token 归还前已
    /// disable_sync；替换前已停止并同步注销旧/测试 action。UART RX IER 保持关闭，发送位
    /// 可以保留；本函数不改 IER。允许首次登记的 BSP 或后续 Thread 控制者调用，不得从
    /// 同线 handler 或持有其退出所需的 PORT/其他锁调用。双拥有时保留原槽责任，拒绝的
    /// token 与不可启用的 token 均在槽锁外同步注销后返回错误，不遗失活 action 责任。
    #[cfg(feature = "ktest")]
    pub fn restore_uart_registration(
        registration: crate::irq::IrqRegistration,
    ) -> ::ktest::TestResult {
        let mut slot = UART_REGISTRATION.lock();
        let error = if slot.is_some() {
            ::ktest::TestError("default UART registration is already installed")
        } else {
            // enable/is_enabled 仅取短锁，不等待 handler；handler 不访问此拥有者槽。
            registration.enable();
            if registration.is_enabled() {
                *slot = Some(registration);
                return Ok(());
            }
            ::ktest::TestError("default UART registration cannot be enabled")
        };
        drop(slot);
        registration.unregister_sync();
        Err(error)
    }

    /// 独立检查真实应急 writer 的预算：0 停滞，1 跨字符串共享，2 恒可发送。
    /// 这是确定性 writer 算法验收，不伪称 UART 硬件 IRQ 或硬件 LSR。
    #[cfg(feature = "ktest")]
    pub(crate) fn emergency_budget(mode: u8) -> bool {
        super::emergency_budget(mode)
    }

    #[cfg(feature = "ktest")]
    pub(super) struct Counters {
        pub(super) accepted: AtomicUsize,
        pub(super) zero_writes: AtomicUsize,
        pub(super) tx_interrupts: AtomicUsize,
        pub(super) wakes: AtomicUsize,
    }

    #[cfg(feature = "ktest")]
    pub(super) static COUNTERS: Counters = Counters {
        accepted: AtomicUsize::new(0),
        zero_writes: AtomicUsize::new(0),
        tx_interrupts: AtomicUsize::new(0),
        wakes: AtomicUsize::new(0),
    };

    #[cfg(feature = "ktest")]
    #[derive(Clone, Copy)]
    pub struct Snapshot {
        pub accepted: usize,
        pub zero_writes: usize,
        pub tx_interrupts: usize,
        pub wakes: usize,
    }

    /// 读取实际接受、拒写、THRE 中断与成功通知计数，不读取或消费 RX。
    #[cfg(feature = "ktest")]
    pub fn snapshot() -> Snapshot {
        Snapshot {
            accepted: COUNTERS.accepted.load(Ordering::Relaxed),
            zero_writes: COUNTERS.zero_writes.load(Ordering::Relaxed),
            tx_interrupts: COUNTERS.tx_interrupts.load(Ordering::Relaxed),
            wakes: COUNTERS.wakes.load(Ordering::Relaxed),
        }
    }

    /// 在同一端口锁内保存并返回原 RX 中断位，声明本段测试接收控制并关闭 RX，发送位原样保留。
    /// 调用前不得已有控制或残留配额；失败不清除控制，由用例中止处理，不发表新的配额。
    #[cfg(feature = "ktest")]
    pub fn claim_receive_control() -> bool {
        let mut port = super::PORT.lock();
        assert!(
            !port.test_control_owned,
            "UART test receive control is already owned"
        );
        assert!(
            port.quota.is_none(),
            "previous UART test quota must be consumed before claiming control"
        );
        let ier = read_register(IER);
        port.test_control_owned = true;
        write_register(IER, ier & !RECEIVE_INTERRUPT);
        ier & RECEIVE_INTERRUPT != 0
    }

    /// 在 PORT 锁内核对测试控制已独占、接收 producer 已关且配额为空，不改变状态。
    fn check_receive_control(port: &RxState) -> ::ktest::TestResult {
        if !port.test_control_owned {
            return Err(::ktest::TestError("UART test receive control is not owned"));
        }
        if read_register(IER) & RECEIVE_INTERRUPT != 0 {
            return Err(::ktest::TestError(
                "UART RX must be closed before synchronization",
            ));
        }
        if port.quota.is_some() {
            return Err(::ktest::TestError(
                "UART receive quota must be consumed before synchronization",
            ));
        }
        Ok(())
    }

    /// 在独占测试控制且 RX 关闭、配额为空时同步唯一 default action 的完整回调尾部。
    /// 仅可服务 IRQ 的 Thread 控制者调用，不得持回调退出所需锁；锁外暂停并恢复同一
    /// 未注销 token，设备 IER 与计数不变。失败保留接收控制，由调用者按真实错误结束拥有。
    pub fn synchronize_receive_irq() -> ::ktest::TestResult {
        crate::sched::assert_switchable();
        {
            let port = PORT.lock();
            check_receive_control(&port)?;
        }
        let registration = take_uart_registration()?;
        registration.disable_sync();
        restore_uart_registration(registration)
    }

    /// 同步原 IRQ 尾部后在 PORT 锁内提交测试接收去向；调用前须已 claim 且 RX 关闭、配额为空。
    /// 附着要求旧软件环为空；撤附着保留已接受采样供原 Work 完成，发送与设备配置不变。
    pub fn set_tty_receive(attached: bool) -> ::ktest::TestResult {
        synchronize_receive_irq()?;
        let mut port = PORT.lock();
        check_receive_control(&port)?;
        if attached && port.len != 0 {
            return Err(::ktest::TestError(
                "UART software ring must be empty before TTY receive attachment",
            ));
        }
        port.tty_attached = attached;
        Ok(())
    }

    /// 在同一端口锁内核对前段已收尾后发表本段剩余接收配额并打开真实接收中断；零长度不发表配额也不打开接收。
    /// 调用前已声明本段测试接收控制、关闭接收中断、轮询消费本段 H，并清空前段行规程工作与软件环；本函数只置接收位，发送中断位原样保留。最后字节关接收后控制保持，直到 ACK 被轮询读走再释放。
    #[cfg(feature = "ktest")]
    pub fn arm_receive_bytes(length: usize) {
        let mut port = super::PORT.lock();
        assert!(
            port.test_control_owned,
            "UART test receive control must be claimed before arming"
        );
        let ier = read_register(IER);
        assert_eq!(
            ier & RECEIVE_INTERRUPT,
            0,
            "UART RX must be closed before arming a test segment"
        );
        assert_eq!(
            port.len, 0,
            "UART software ring must be empty before arming a test segment"
        );
        assert!(
            port.quota.is_none(),
            "previous UART test quota must be consumed before arming"
        );
        if 0 < length {
            port.quota = Some(length);
            write_register(IER, ier | RECEIVE_INTERRUPT);
        } else {
            port.quota = None;
        }
    }

    /// 返回当前测试段剩余接收配额；无配额或已收齐时为 None。
    #[cfg(feature = "ktest")]
    pub fn receive_quota_remaining() -> Option<usize> {
        super::PORT.lock().quota
    }

    /// 只观察软件接收环是否为空，供测试阶段边界核对，不取出任何采样。
    #[cfg(feature = "ktest")]
    pub fn receive_queue_empty() -> bool {
        super::PORT.lock().len == 0
    }

    /// 在同一端口锁内核对本段已收尾后释放测试接收控制，按调用者保存的真实接收状态恢复 RX，发送位原样保留。
    /// 调用前配额已耗尽、软件环已空且 ACK 已被轮询读走；TTY 传当前 CREAD，通用控制握手传 claim 保存的原 RX 位。
    #[cfg(feature = "ktest")]
    pub fn release_receive_control(receiver: bool) {
        let mut port = super::PORT.lock();
        assert!(
            port.test_control_owned,
            "UART test receive control is not owned"
        );
        assert!(
            port.quota.is_none(),
            "UART test quota must be consumed before releasing control"
        );
        assert_eq!(
            port.len, 0,
            "UART software ring must be empty before releasing test control"
        );
        port.test_control_owned = false;
        let mut ier = read_register(IER) & !RECEIVE_INTERRUPT;
        if receiver {
            ier |= RECEIVE_INTERRUPT;
        }
        write_register(IER, ier);
    }
}
