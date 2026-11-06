//! 16550 / Synopsys DesignWare APB UART 控制台与串口底层驱动模块。
//!
//! # 模块职责与架构定位
//! 本模块是内核唯一控制台串口（UART）的底层硬件拥有者，向上层提供三类核心服务：
//! 1. **早期与应急输出（[`early_write`]、[`fatal`]、[`panic`]、[`board_stop`]）**：在内存分配器、调度器或
//!    设备树尚未就绪时提供同步轮询输出；在内核崩溃（Panic）或致命异常时提供无需获取自旋锁、带轮询次数上限
//!    （[`EMERGENCY_POLL_BUDGET`]）的应急输出，防止死锁或因硬件故障永久卡死。
//! 2. **TTY 行规程与内核日志的异步收发支撑（[`try_write`]、[`read_received`]、[`configure_serial`]）**：
//!    上层 `crate::fs::tty` 与 `crate::klog` 通过本模块非阻塞地向硬件发送 FIFO 写入字节，并在中断处理函数
//!    （[`uart_handler`]）将接收字节与线路状态（LSR）采样存入软件环形缓冲区（[`RxState`]）后唤醒工作任务消费。
//! 3. **内核自测（`ktest`）控制通道与测试观测（[`testing`]）**：在启用 `ktest` 特性时，支持宿主机评测器与
//!    内核测试框架在同一个物理串口上复用“命令握手字节（`H` / `ACK`）”与“被测负载字节”，并提供精确的接收配额控制。
//!
//! # 核心设计与寄存器/状态机布局
//! ## 1. 16550 / DesignWare APB UART 寄存器窗口与 DLAB 复用布局
//! 串口寄存器地址由 `base + (offset << reg_shift)` 计算，支持 1 字节（标准 NS16550A）或 4 字节（JH7110
//! DesignWare APB UART）总线访问宽度。当线路控制寄存器（`LCR`）最高位 `DLAB`（Bit 7）置 1 时，偏移 0 和 1
//! 从数据收发/中断使能切换为波特率分频锁存器（`DLL`/`DLM`）：
//!
//! ```text
//!   寄存器逻辑偏移 | DLAB = 0 (常规收发模式)        | DLAB = 1 (波特率分频配置模式)
//!   ---------------+--------------------------------+-------------------------------
//!   0x00           | RBR (读接收) / THR (写发送)    | DLL (分频系数低 8 位)
//!   0x01           | IER (中断使能: RX=bit0,TX=bit1)| DLM (分频系数高 8 位)
//!   0x02           | IIR (读中断标识) / FCR (写FIFO)| IIR / FCR
//!   0x03           | LCR (字长/停止位/校验/DLAB位)  | LCR (bit7 = DLAB = 1)
//!   0x04           | MCR (DTR=bit0, RTS=bit1, LOOP) | MCR
//!   0x05           | LSR (DR=bit0, THRE=bit5, TEMT) | LSR (读操作会清除错误位)
//!   0x06           | MSR (调制解调器状态)           | MSR
//!   0x1f (仅 DW)   | USR (DesignWare 状态: BUSY=b0) | USR
//! ```
//!
//! ## 2. DesignWare APB UART 忙状态（Busy-Detect）与安全重配流程
//! 在 Synopsys DesignWare APB UART（如 VisionFive 2）上，若在串口仍处于收发忙状态（`USR.BUSY == 1`）时改写
//! `LCR`，硬件会触发 `IIR_BUSY`（`0x07`）中断并拒绝写入。因此 [`configure_line`] 按以下状态机安全切换线路参数：
//!
//! ```text
//!   [持 PORT 自旋锁, 关 IER]
//!            |
//!            v
//!   1. 轮询 LSR 直至 TEMT=1 (排空硬件发送移位寄存器，期间順带保存已到达的 RBR 字节)
//!            |
//!            v
//!   2. 置位 MCR.LOOPBACK (隔离外部 RX 引脚，防止新字节继续进入移位器导致 BUSY 不落)
//!            |
//!            v
//!   3. 轮询 LSR 与 DW_USR 直至 USR.BUSY=0 (保存隔离前残留的 RBR 采样)
//!            |
//!            v
//!   4. 置 LCR.DLAB=1 -> 写 DLL/DLM -> 读回确认 -> 清 LCR.DLAB=0 -> 恢复 MCR 与 IER
//! ```
//!
//! # 锁获取顺序与并发契约
//! - **`PORT`（[`SpinLock<RxState>`]）**：保护 UART 寄存器组的互斥访问（特别是 `LCR.DLAB` 切换窗口与 `LSR`
//!   读后清错误位与 `DATA` 读取的配对）以及软件接收环形缓冲区。该自旋锁在持锁期间自动关闭本地中断，属于最底层叶子锁。
//! - **`testing::UART_REGISTRATION`（仅 `ktest`）**：保护默认 UART 中断注册句柄（`IrqRegistration`）的存取。
//! - **严禁在持有 `PORT` 锁时调用外部通知或睡眠接口**：`crate::fs::tty::schedule_receive`、
//!   `crate::fs::tty::schedule_transmit`、`TX_WAIT.wake_all` 以及 `IrqRegistration::disable_sync` / `unregister_sync`
//!   **必须**在释放 `PORT` 自旋锁之后调用，否则会导致中断处理函数或工作队列重入死锁。

use crate::sync::SpinLock;
use crate::wait::WaitQueue;
use core::convert::TryFrom;
use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};
use spin::Once;

/// 串口硬件的只读几何参数与时钟配置快照，在引导期由 [`configure_resources`] 一次性初始化。
struct PortResources {
    /// 串口寄存器组的物理基地址（字节）。
    base: usize,
    /// 单个寄存器的 MMIO 总线访问宽度（字节，仅支持 `1` 或 `4`）。
    width: u32,
    /// 相邻逻辑寄存器编号之间的地址左移位数（字节偏移 = `offset << shift`）。
    shift: u32,
    /// 波特率分频器基准频率（Hz，等于输入时钟频率除以 16，即分频系数为 1 时的最高波特率）。
    baud_base: u32,
    /// 是否为具有 `DW_USR` 寄存器与忙检测（Busy-Detect）语义的 Synopsys DesignWare APB UART。
    dw_apb: bool,
    /// 设备树或平台声明的控制台默认工作波特率（bps）；固件已在引导前配好线路，内核首次打开终端时保持该速率。
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
// DLL/DLM 与 DATA/IER 共享逻辑偏移 0 和 1；访问前必须先置位 LCR 的 DLAB 位，访问后立即清零恢复。
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

// IIR 低 4 位编码中断原因，其中 bit 0 为 1 表示无挂起中断；DesignWare 的 busy-detect (0x07) 同样落在低 4 位。
const IIR_STATUS_MASK: u8 = 0x0f;
const IIR_NO_INTERRUPT: u8 = 1;
const IIR_MODEM_STATUS: u8 = 0;
const IIR_BUSY: u8 = 7;
const IIR_RX_TIMEOUT: u8 = 0x0c;

const UART_CLOCK_DIVISOR: u64 = 16;
const QEMU_RISCV_BAUD_BASE: u32 = 399_193;
const QEMU_LOONGARCH_BAUD_BASE: u32 = 115_200;

const RX_IRQ_BUDGET: usize = 16;

// 单次应急打印允许轮询读取 LSR 的最大次数上限，避免硬件故障时死循环，且不依赖时钟或其他 CPU。
const EMERGENCY_POLL_BUDGET: usize = 10_000;

static PORT: SpinLock<RxState> = SpinLock::new(RxState::new());
static TX_WAIT: WaitQueue = WaitQueue::new();
static RECEIVED: AtomicUsize = AtomicUsize::new(0);
// 部分硬件（如无调制解调器控制线的精简 UART）不支持回读 MCR 的 DTR/RTS 位，不一致告警仅打印一次以防刷屏。
static READBACK_WARNED: AtomicBool = AtomicBool::new(false);
static LAST_BYTE: AtomicU8 = AtomicU8::new(0);

/// 中断处理函数或线路重配流程从硬件接收 FIFO 中取出的单个字节及其关联的线路状态样本。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RxSample {
    /// 从硬件 `DATA`（`RBR`）寄存器读取的原始 8 位数据（`0x00` 或 `0xff` 均为有效数据字节）。
    pub byte: u8,
    /// 读取该 `DATA` 字节前紧邻读取的 `LSR` 寄存器快照，保留与该字节对应的帧错误、奇偶校验错误、Break 或硬件溢出位。
    pub status: u8,
}

/// TTY 层请求应用到串口硬件的目标线路配置参数。
///
/// 调用方需保证 `divisor` 非零、字长与校验位合法，且 `line_control` 未置位 `DLAB`（Bit 7 为 0）。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SerialConfig {
    /// 写入 `DLL`/`DLM` 的 16 位波特率分频系数（`baud_base / target_baud`）。
    pub divisor: u16,
    /// 待写入 `LCR` 寄存器的控制字节（包含 5..=8 位数据位长、停止位与奇偶校验位，且 `DLAB` 位必须为 0）。
    pub line_control: u8,
    /// 终端 `termios` 的 `CREAD` 是否开启；为 `true` 时允许使能硬件接收中断（`RECEIVE_INTERRUPT`）。
    pub receiver: bool,
    /// 是否拉高数据终端就绪（`DTR`）调制解调器控制线。
    pub dtr: bool,
    /// 是否拉高请求发送（`RTS`）调制解调器控制线。
    pub rts: bool,
}

/// 在 `PORT` 自旋锁保护下回读到的串口硬件寄存器状态快照。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SerialSnapshot {
    /// 最近一次在 `DLAB = 1` 窗口内从 `DLL`/`DLM` 实际回读到的 16 位分频系数。
    pub divisor: u16,
    /// 从硬件 `LCR` 寄存器回读的值。
    pub line_control: u8,
    /// 从硬件 `IER` 寄存器回读的值。
    pub ier: u8,
    /// 从硬件 `MCR` 寄存器回读的值。
    pub mcr: u8,
    /// 从硬件 `LSR` 寄存器回读的值。
    pub lsr: u8,
    /// 当前串口硬件的波特率分频基准频率（Hz，等于输入时钟除以 16）。
    pub baud_base: u32,
}

/// 软件接收环形缓冲区的固定容量（采样个数）。
const RX_CAPACITY: usize = 1024;

/// 由 `PORT` 自旋锁保护的串口接收环形缓冲区与线路状态。
///
/// 中断处理函数与 DesignWare 线路重配窗口负责向缓冲区推入 [`RxSample`]，TTY 行规程工作任务负责弹出消费。
struct RxState {
    /// 存放接收字节与 `LSR` 状态的固定大小环形数组。
    samples: [RxSample; RX_CAPACITY],
    /// 下一个新采样写入的环形缓冲区下标（`0..RX_CAPACITY`）。
    head: usize,
    /// 下一个待消费采样读取的环形缓冲区下标（`0..RX_CAPACITY`）。
    tail: usize,
    /// 当前环形缓冲区内尚未被上层取走的采样数量（`0..=RX_CAPACITY`）。
    len: usize,
    /// 因软件环形缓冲区已满（`len == RX_CAPACITY`）而被迫丢弃的采样累计次数。
    overrun: usize,
    /// 最近一次已写入硬件并确认的 `(divisor, lcr)` 组合，用于在参数未变时跳过耗时的硬件重配与排空等待。
    line_settings: Option<(u16, u8)>,
    /// `ktest` 模式下当前测试段允许通过接收中断消费的剩余字节配额；耗尽时在锁内自动关闭接收中断。
    #[cfg(feature = "ktest")]
    quota: Option<usize>,
    /// `ktest` 模式下是否已由测试框架独占接管接收控制权。
    #[cfg(feature = "ktest")]
    test_control_owned: bool,
    /// `ktest` 模式下接收到的采样是否同时送入 TTY 软件环形缓冲区并通知行规程。
    #[cfg(feature = "ktest")]
    tty_attached: bool,
}
impl RxState {
    /// 创建初始为空的串口接收状态；环形缓冲区为空，且未记录历史线路配置或测试控制状态。
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

    /// 当软件环形缓冲区未满时入队一个接收采样；若已满则递增 `overrun` 溢出丢包计数。
    fn push(&mut self, sample: RxSample) {
        if self.len < RX_CAPACITY {
            self.samples[self.head] = sample;
            self.head = (self.head + 1) % RX_CAPACITY;
            self.len += 1;
        } else {
            self.overrun += 1;
        }
    }

    /// 按先进先出（FIFO）顺序从环形缓冲区弹出最多 `out.len()` 个采样写入 `out`，返回实际取出的采样个数。
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

    /// 在 `ktest` 模式下扣减 1 字节的测试接收配额；当配额恰好减至 0 时清除配额并返回 `true`。
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

/// 计算指定逻辑寄存器编号对应的内核 MMIO 虚拟地址与总线访问字节宽度。
///
/// 调用上下文：任意上下文（含早期引导、常规收发及 Panic 应急路径）。
/// 在 [`configure_resources`] 公布设备树资源前，使用编译期目标平台的默认早期串口几何参数。
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

/// 按串口寄存器声明的总线宽度读取单个逻辑寄存器，并返回低 8 位有效值。
///
/// 调用上下文：持有 `PORT` 自旋锁的常规路径、中断处理函数或无锁的 Panic 应急输出路径。
fn read_register(offset: usize) -> u8 {
    let (address, width) = register(offset);
    // SAFETY:
    // 1. 引导早期硬编码基址与 [`configure_resources`] 校验后的基址相等，且均已通过内核永久 MMIO 映射暴露；
    // 2. `width` 仅取 1 或 4，4 字节宽度下基址与 `offset << 2` 均满足 `u32` 的 4 字节对齐要求；
    // 3. `offset` 均来自本模块定义的合法 UART 寄存器常量，使用 `read_volatile` 符合设备寄存器无别名读取要求。
    unsafe {
        match width {
            1 => (address as *const u8).read_volatile(),
            4 => (address as *const u32).read_volatile() as u8,
            _ => unreachable!(),
        }
    }
}

/// 按串口寄存器声明的总线宽度向单个逻辑寄存器写入 8 位值（4 字节宽度下零扩展为 `u32` 写入）。
///
/// 调用上下文：持有 `PORT` 自旋锁的常规路径、中断处理函数或无锁的 Panic 应急输出路径。
fn write_register(offset: usize, value: u8) {
    let (address, width) = register(offset);
    // SAFETY:
    // 1. 寄存器地址位于永久建立的内核 MMIO 映射区域内，`width` 为 1 或 4 且满足对应类型的对齐要求；
    // 2. 常规运行期访问由 `PORT` 自旋锁串行化，崩溃应急路径仅作单向 `write_volatile` 硬件寄存器写入，不创建 Rust 引用。
    unsafe {
        match width {
            1 => (address as *mut u8).write_volatile(value),
            4 => (address as *mut u32).write_volatile(u32::from(value)),
            _ => unreachable!(),
        }
    }
}

/// 在主核（BSP）完成设备树解析与永久页表映射后，一次性公布控制台串口的寄存器几何参数与时钟基准。
///
/// 调用上下文：引导早期单核（BSP）初始化阶段，仅可调用一次。
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

/// 判断当前平台串口是否为具有 `DW_USR` 状态寄存器与 Busy-Detect 机制的 Synopsys DesignWare APB UART。
fn dw_apb() -> bool {
    RESOURCES
        .get()
        .map_or(cfg!(blossom_platform = "visionfive2"), |resource| {
            resource.dw_apb
        })
}

/// 向线路控制寄存器（`LCR`）写入新值，并在 DesignWare UART 上立即回读校验除粘性奇偶位外的控制位已生效。
///
/// 调用上下文：必须持有 `PORT` 自旋锁；在 DesignWare UART 上调用方需先完成发送排空与接收回环隔离。
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

/// 若传入的 `LSR` 状态字表明接收数据就绪（`DATA_READY`），则立即读取 `DATA` 寄存器并将字节与该 `LSR` 快照压入接收环。
///
/// 调用上下文：必须持有 `PORT` 自旋锁且 `LCR.DLAB == 0`。由于读取 `LSR` 会自动清除硬件中的溢出/奇偶/帧错误标志，
/// 调用方在读取 `LSR` 后必须先通过本函数保存对应字节与错误位快照，随后才能再次读取 `LSR`。
fn preserve_pending_receive(port: &mut RxState, status: u8) -> bool {
    if status & DATA_READY == 0 {
        return false;
    }
    let byte = read_register(DATA);
    port.push(RxSample { byte, status });
    true
}

/// 在持有 `PORT` 自旋锁的前提下更新串口分频系数与 `LCR` 线路控制参数，返回回读确认的分频值及期间是否收到新数据。
///
/// 调用上下文：由 [`configure_serial`] 在持有 `PORT` 自旋锁且已关闭 `IER` 中断时调用。
/// 若目标 `(divisor, line_control)` 与上次已生效的配置相同，则直接返回而不触碰硬件。
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
        // Step 1 (DesignWare): 轮询等待硬件发送移位器与发送 FIFO 彻底排空（TEMT = 1），期间顺带拾取已到达的接收字节。
        loop {
            let status = read_register(LSR);
            received |= preserve_pending_receive(port, status);
            if status & TRANSMITTER_EMPTY != 0 {
                break;
            }
            spin_loop();
        }

        // Step 2 (DesignWare): 开启 MCR 回环模式以隔离外部 RX 引脚，排空残留接收字节并等待控制器退出 BUSY 状态。
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

    // Step 3: 置位 DLAB 进入分频锁存器窗口，写入 16 位分频值并回读确认，随后清除 DLAB 恢复常规寄存器视图。
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

/// 在真实开发板关机或重启前，关闭串口中断并同步输出停机诊断信息，等待硬件发送移位器彻底排空。
///
/// 调用上下文：其它从核（AP）与本核本地事件源均已停止后的最终停机阶段（非 `ktest` 构建）。
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

/// 在 `ktest` 构建的当前 CPU 致命停止路径中，关闭串口中断并通过有界轮询输出停机诊断信息。
///
/// 调用上下文：当前核本地中断与事件源已关闭的异常/关机末尾；不获取 `PORT` 自旋锁，防止与其他核死锁。
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

/// 轮询等待发送保持寄存器就绪（`LSR.THRE == 1`）后向硬件写入单个字节。
///
/// 调用上下文：引导早期输出或停机诊断路径，调用方已持有 `PORT` 自旋锁。
fn put_byte(byte: u8) {
    while read_register(LSR) & TRANSMIT_READY == 0 {
        spin_loop();
    }
    write_register(DATA, byte);
}

/// 持有 `PORT` 自旋锁时的同步阻塞格式化输出器。
struct PortWriter;

impl Write for PortWriter {
    /// 按字节顺序将字符串同步轮询写入串口发送寄存器，不分配任何堆内存。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for byte in text.bytes() {
            put_byte(byte);
        }
        Ok(())
    }
}

/// 带总轮询次数预算的应急格式化输出器，用于 Panic 和致命异常路径以防硬件挂死。
struct EmergencyWriter<R, W> {
    /// 当前整条消息剩余允许调用 `read_status` 的最大次数。
    remaining: usize,
    /// 读取串口线路状态寄存器（`LSR`）的闭包。
    read_status: R,
    /// 向串口数据寄存器（`DATA`）写入单个字节的闭包。
    write_byte: W,
}

impl<R: FnMut() -> u8, W: FnMut(u8)> Write for EmergencyWriter<R, W> {
    /// 在共享的轮询预算内输出字符串；每次查询状态消耗 1 次预算，预算耗尽时立即中断并返回 `fmt::Error`。
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

/// 创建绑定到真实串口寄存器、带 [`EMERGENCY_POLL_BUDGET`] 次状态轮询上限的栈上应急输出器。
fn emergency_writer() -> impl Write {
    EmergencyWriter {
        remaining: EMERGENCY_POLL_BUDGET,
        read_status: || read_register(LSR),
        write_byte: |byte| write_register(DATA, byte),
    }
}

/// 单元测试辅助函数：验证 [`EmergencyWriter`] 在阻塞（0）、跨次调用共享预算（1）及始终就绪（2）三种场景下的预算扣减行为。
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

/// 初始化控制台串口硬件：关闭所有串口中断、在非 DesignWare 串口上设为 8N1，并启用及清空硬件收发 FIFO。
///
/// 调用上下文：引导早期单核（BSP）初始化阶段。DesignWare UART 保留固件已建立的线路控制参数，直到后续 TTY 层正式配置。
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

/// 引导阶段的同步格式化输出入口，在 `PORT` 自旋锁保护下直接轮询发送，不依赖调度器、堆分配或中断。
pub fn early_write(arguments: fmt::Arguments<'_>) {
    let _port = PORT.lock();
    PortWriter
        .write_fmt(arguments)
        .expect("UART formatting failed");
}

/// 在 `ktest` 模式下由测试控制线程轮询读取单个串口控制字节（如宿主机握手命令 `H` 或确认字节 `ACK`）。
///
/// 调用上下文：调用前必须确保硬件接收中断（`RECEIVE_INTERRUPT`）已关闭，且期间没有任何中断回调会重新开启接收中断，
/// 防止中断处理函数与本轮询循环竞争消费硬件接收 FIFO 中的字节。
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

/// 在 `ktest` 模式下仅清除 `IER` 的接收中断使能位，保留发送中断使能位不变。
///
/// 调用上下文：当宿主机发出的起始握手字节 `H` 已被接收中断消费后，测试框架调用本函数关闭接收中断，
/// 使后续到达的确认字节 `ACK` 保留在硬件 FIFO 中供 [`ktest_read_byte`] 轮询读取。
#[cfg(feature = "ktest")]
pub(crate) fn ktest_disable_receive_interrupts() {
    let _port = PORT.lock();
    write_register(IER, read_register(IER) & !RECEIVE_INTERRUPT);
}

/// 在 `ktest` 故障报告路径中使用带轮询预算的应急输出器打印格式化诊断文本，不获取 `PORT` 自旋锁。
#[cfg(feature = "ktest")]
pub(crate) fn emergency_write(arguments: fmt::Arguments<'_>) {
    let _ = emergency_writer().write_fmt(arguments);
}

/// 在 `PORT` 自旋锁临界区内非阻塞地向硬件发送 FIFO 写入最多 [`TX_BUDGET`]（16）个原始字节，返回实际写入的字节数。
///
/// 调用上下文：由 TTY 行规程或内核日志（`klog`）发送路径调用；本函数不做阻塞等待、格式化或换行符转换。
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
    // 内核日志（klog）与 TTY 行规程共享同一个物理串口；由于另一方可能已关闭 THRE 发送中断，
    // 本次成功向硬件写入字节后必须在释放 PORT 锁后通知 TTY 行规程复查其待发送队列。
    if 0 < accepted {
        crate::fs::tty::schedule_transmit();
    }
    accepted
}

/// 返回全局串口发送等待队列（[`TX_WAIT`]）的静态引用。
///
/// 调用上下文：发送方在因硬件 FIFO 满而准备休眠前，应先将等待节点登记到该等待队列，再调用 [`arm_tx_interrupt`]。
pub(crate) fn tx_wait_queue() -> &'static WaitQueue {
    &TX_WAIT
}

/// 在 `PORT` 自旋锁保护下开启发送保持寄存器空（`THRE`）中断并立即复查 `LSR.THRE` 状态。
///
/// 调用上下文：当 `try_write` 因发送 FIFO 已满返回 0 后，调用方在登记等待队列或准备让出 CPU 前调用。
/// 若开启中断后复查发现 `LSR.THRE` 已经就绪（返回 `true`），则立即撤回 `TRANSMIT_INTERRUPT` 使能位以避免多余中断。
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

/// 在 `PORT` 自旋锁保护下开启串口硬件接收中断（`IER.RECEIVE_INTERRUPT`）。
pub fn enable_receive_interrupts() {
    let _port = PORT.lock();
    write_register(IER, read_register(IER) | RECEIVE_INTERRUPT);
}

/// 在 `ktest` 模式下将串口 `IER` 寄存器清零以关闭全部收发中断，不读取或丢弃硬件 FIFO 中已有的待收字节。
///
/// 调用上下文：在同步注销（`unregister_sync`）串口中断处理函数之前调用，确保硬件不再产生新的中断请求。
#[cfg(feature = "ktest")]
pub fn disable_interrupts() {
    let _port = PORT.lock();
    write_register(IER, 0);
}

/// 返回当前串口的波特率分频基准频率（`baud_base`，单位 Hz）。
///
/// 注意：在 QEMU `virt` 平台上返回 QEMU `serial_mm_init` 模拟器的固定基准频率，真机平台返回输入时钟频率除以 16。
pub fn baud_base() -> u32 {
    RESOURCES
        .get()
        .expect("UART resources not configured")
        .baud_base
}

/// 返回平台或设备树为控制台声明的默认工作波特率（bps），供 TTY 首次初始化时保持与引导固件一致的线路速率。
pub fn console_baud() -> u32 {
    RESOURCES
        .get()
        .expect("UART resources not configured")
        .console_baud
}

/// 在 `PORT` 自旋锁保护下读取硬件 `LSR` 的 `TEMT`（Bit 6）标志，判断发送 FIFO 与发送移位寄存器是否已全部排空。
pub fn transmitter_empty() -> bool {
    let _port = PORT.lock();
    read_register(LSR) & TRANSMITTER_EMPTY != 0
}

/// 在 `PORT` 自旋锁保护下从软件接收环形缓冲区弹出最多 `samples.len()` 个采样写入 `samples`，返回实际取出的数量。
pub fn read_received(samples: &mut [RxSample]) -> usize {
    PORT.lock().pop(samples)
}

/// 在 `PORT` 自旋锁保护下清空软件接收环形缓冲区中的所有未消费采样（重置 `head`、`tail`、`len` 为 0）。
///
/// 调用上下文：终端执行输入刷新（如 `TCIFLUSH`）时调用；仅清空软件环形缓冲区，不读取硬件 FIFO，也不修改测试配额或统计计数。
pub fn discard_received() {
    let mut port = PORT.lock();
    port.head = 0;
    port.tail = 0;
    port.len = 0;
}

/// 在 `PORT` 自旋锁保护下应用 [`SerialConfig`] 串口参数，并在释放锁后唤醒 TTY 接收任务处理重配期间捕获的字节。
///
/// 调用上下文：由 TTY 层在初始化或处理 `tcsetattr` 等终端配置变更时调用。要求 `config.line_control` 的 `DLAB` 位必须为 0。
pub fn configure_serial(config: SerialConfig) -> SerialSnapshot {
    assert_eq!(
        config.line_control & DLAB,
        0,
        "UART line control must not set DLAB"
    );

    // Step 1: 获取 PORT 锁，保存原 IER/MCR 并暂时关闭串口中断，随后调用 configure_line 更新分频系数与 LCR。
    let mut port = PORT.lock();
    let saved_ier = read_register(IER);
    let saved_mcr = read_register(MCR);
    write_register(IER, 0);
    let (divisor, received) = configure_line(&mut port, config, saved_mcr);

    // Step 2: 根据配置更新 MCR 的 DTR/RTS 控制线，并恢复 IER（保留原发送中断位，按 CREAD 与测试控制状态决定接收中断位）。
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

    // Step 3: 回读寄存器生成状态快照，断言 DLAB 已清零后释放 PORT 自旋锁。
    let snapshot = SerialSnapshot {
        divisor,
        line_control: read_register(LCR),
        ier: read_register(IER),
        mcr: read_register(MCR),
        lsr: read_register(LSR),
        baud_base: baud_base(),
    };
    // 若释放锁前 DLAB 仍未清零，偏移 0 和 1 将停留在分频锁存器视图，导致后续 DATA/IER 读写访问错误寄存器。
    assert_eq!(
        snapshot.line_control & DLAB,
        0,
        "UART line control left DLAB set"
    );
    drop(port);

    // Step 4: 在锁外通知 TTY 行规程消费重配窗口内收到的字节，并对首次出现的寄存器回读差异打印单次诊断日志。
    if received {
        crate::fs::tty::schedule_receive();
    }

    // 部分板卡或模拟器对 MCR 等控制位不支持完整回读，回读差异仅做一次性日志记录而不视为致命错误。
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

/// 默认的控制台串口外部中断处理函数（每次中断最多读取 [`RX_IRQ_BUDGET`] 个接收字节，不主动关闭接收中断）。
///
/// # Safety
/// 1. 仅可作为当前控制台 UART 硬件中断源的 IRQ 处理回调注册并调用，调用时串口永久 MMIO 映射必须已建立。
/// 2. 调用方必须处于本地中断已关闭的硬中断上下文，且在调用本函数前未持有 `PORT` 自旋锁（避免同核重入死锁）。
/// 3. `_context` 参数在本函数中不使用，允许传入空指针。
pub unsafe fn uart_handler(_context: *mut ()) -> bool {
    handle_uart::<RX_IRQ_BUDGET, false>()
}

/// `ktest` 专用的单字节测试中断处理函数：最多从硬件读取 1 个接收字节，并在读取后立即在 `PORT` 锁内将 `IER` 清零。
///
/// # Safety
/// 1. 必须满足 [`uart_handler`] 的全部安全前提条件（串口映射已建立、处于非重入硬中断上下文且未持有 `PORT` 锁）。
/// 2. 注册并启用本回调前，默认的 [`uart_handler`] 必须已停止并完成同步注销（`unregister_sync`），保证同一时刻只有一个回调处理串口中断。
/// 3. 调用方须先通过轮询读走握手命令字节 `H` 再开启接收中断；本回调读取 1 个负载字节并关闭 `IER` 后，在轮询读走后续 `ACK` 字节前不得再次开启串口中断。
#[cfg(feature = "ktest")]
pub(crate) unsafe fn ktest_uart_handler_one(_context: *mut ()) -> bool {
    handle_uart::<1, true>()
}

/// 串口中断处理的核心实现，由常量泛型控制单次接收字节上限（`RX_BUDGET`）以及是否在收到首个字节后立即关闭全部中断（`STOP_RX`）。
///
/// 调用上下文：硬中断上下文，由 [`uart_handler`] 或 [`ktest_uart_handler_one`] 调用。
#[inline(always)]
fn handle_uart<const RX_BUDGET: usize, const STOP_RX: bool>() -> bool {
    let (produced, notify_tx) = {
        // Step 1: 获取 PORT 自旋锁，读取 IIR 处理 DesignWare 忙检测（读 USR 清除）、无挂起中断、空超时及调制解调器状态中断。
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

        // Step 2: 若接收中断已使能，循环读取最多 RX_BUDGET 个字节与配对的 LSR 快照，按需压入软件接收环并更新计数与测试配额。
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
                    // 单字节测试模式下必须在读取 DATA 后立即在同一个 PORT 临界区内清零 IER，把后续 ACK 字节留在硬件 FIFO 中。
                    write_register(IER, 0);
                }
                #[cfg(feature = "ktest")]
                let quota_done = port.consume_quota();
                LAST_BYTE.store(byte, Ordering::Relaxed);
                RECEIVED.fetch_add(1, Ordering::Release);
                #[cfg(feature = "ktest")]
                if quota_done {
                    // 当前测试段的接收配额已全部收齐，在锁内仅关闭接收中断位，保留发送中断使能位。
                    write_register(IER, read_register(IER) & !RECEIVE_INTERRUPT);
                    break;
                }
            }
        }

        // Step 3: 复查 IIR、IER 与 LSR 发送就绪位；若发送中断已使能且 THR 已空，则关闭发送中断位以防水平触发风暴，并记录唤醒标志。
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
        // 读取 IIR 可清除部分硬件的 THRE 中断挂起状态；即使高优先级的接收中断掩盖了 IIR 编码，也根据 LSR 实际空位一并完成发送通知。
        let _ = (first_interrupt, last_interrupt);
        (produced, notify_tx)
    };

    // Step 4: 释放 PORT 自旋锁后，在锁外唤醒等待发送的任务并调度 TTY 收发工作队列。
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

/// 返回自系统启动以来由串口接收中断累计读取的字节总数（`Acquire` 内存序）。
#[cfg(feature = "ktest")]
pub fn received() -> usize {
    RECEIVED.load(Ordering::Acquire)
}

/// 返回最近一次由串口接收中断读取到的原始数据字节。
#[cfg(feature = "ktest")]
pub fn last_byte() -> u8 {
    LAST_BYTE.load(Ordering::Relaxed)
}

/// 在 `PORT` 自旋锁保护下检查硬件 `LSR.DATA_READY` 位是否置位（仅观测是否有待收数据，不读取 `DATA` 寄存器）。
#[cfg(feature = "ktest")]
pub fn receive_pending() -> bool {
    let _port = PORT.lock();
    read_register(LSR) & DATA_READY != 0
}

/// 内核致命CPU异常（Fatal Trap）终结入口：关闭本地中断，在共享轮询预算内打印异常寄存器现场，随后永久休眠当前核。
///
/// 调用上下文：不可恢复的硬件异常上下文（非 `ktest` 构建）；不获取 `PORT` 锁，避免在持锁崩溃时发生二次死锁。
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

/// 内核 Panic 终结入口：关闭本地中断，在共享轮询预算内输出 Panic 位置与原因，随后永久休眠当前核。
///
/// 调用上下文：`#[panic_handler]` 崩溃处理上下文（非 `ktest` 构建）；不获取 `PORT` 锁。
#[cfg(not(feature = "ktest"))]
pub fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    let _ = crate::arch::irq_save();
    let _ = emergency_writer().write_fmt(format_args!("FAIL panic: {}\n", info));
    loop {
        crate::arch::wait_for_interrupt();
    }
}

/// 向控制台串口同步打印带换行符的早期引导日志宏（内部调用 [`early_write`]）。
#[macro_export]
macro_rules! early_println {
    () => { $crate::console::early_write(format_args!("\n")) };
    ($($argument:tt)*) => {
        $crate::console::early_write(format_args!("{}\n", format_args!($($argument)*)))
    };
}

/// 内核自测（`ktest`）专用的串口观测、中断注册句柄移交与分段接收配额控制子模块。
#[cfg(feature = "ktest")]
pub mod testing {
    use super::*;

    #[cfg(feature = "ktest")]
    static UART_REGISTRATION: SpinLock<Option<crate::irq::IrqRegistration>> = SpinLock::new(None);

    /// 取出保存的默认串口中断注册句柄（[`crate::irq::IrqRegistration`]），供测试线程暂停或临时替换中断回调。
    ///
    /// 调用上下文：仅允许在可调度的测试控制线程中调用，且调用时不得持有 `PORT` 锁或阻塞中断回调退出的其他锁。
    /// 取出句柄后，调用方必须负责在测试结束前通过 [`restore_uart_registration`] 归还默认句柄。
    #[cfg(feature = "ktest")]
    pub fn take_uart_registration() -> Result<crate::irq::IrqRegistration, ::ktest::TestError> {
        UART_REGISTRATION.lock().take().ok_or(::ktest::TestError(
            "default UART registration is not installed",
        ))
    }

    /// 首次安装或归还默认串口中断注册句柄（[`uart_handler`]），并在槽锁内启用该中断回调准入。
    ///
    /// 调用上下文：由引导主核（BSP）首次注册时，或由测试控制线程在完成 `disable_sync` / 卸载测试专用回调后调用；
    /// 调用时不得处于串口中断回调内部，也不得持有 `PORT` 自旋锁。若槽位已被占用或句柄无法启用，则在释放槽锁后
    /// 同步注销（`unregister_sync`）传入的句柄并返回错误，避免泄漏活动的 IRQ 注册项。
    #[cfg(feature = "ktest")]
    pub fn restore_uart_registration(
        registration: crate::irq::IrqRegistration,
    ) -> ::ktest::TestResult {
        let mut slot = UART_REGISTRATION.lock();
        let error = if slot.is_some() {
            ::ktest::TestError("default UART registration is already installed")
        } else {
            // enable/is_enabled 仅获取 IRQ 描述符内部自旋锁，不会阻塞等待中断处理函数退出。
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

    /// 执行 [`EmergencyWriter`] 轮询预算单元测试（`mode`：`0` 持续忙超时，`1` 跨多次写入共享预算，`2` 始终就绪）。
    #[cfg(feature = "ktest")]
    pub(crate) fn emergency_budget(mode: u8) -> bool {
        super::emergency_budget(mode)
    }

    /// `ktest` 模式下统计串口发送路径行为的原子计数器集合。
    #[cfg(feature = "ktest")]
    pub(super) struct Counters {
        /// [`try_write`] 成功写入硬件 `DATA` 寄存器的累计字节数。
        pub(super) accepted: AtomicUsize,
        /// [`try_write`] 传入非空切片但因 `LSR.THRE == 0` 而写入 0 字节的累计次数。
        pub(super) zero_writes: AtomicUsize,
        /// 中断处理函数观察到 `THRE` 发送中断并完成处理的累计次数。
        pub(super) tx_interrupts: AtomicUsize,
        /// 发送中断唤醒 [`TX_WAIT`] 等待队列中睡眠任务的累计数量。
        pub(super) wakes: AtomicUsize,
    }

    #[cfg(feature = "ktest")]
    pub(super) static COUNTERS: Counters = Counters {
        accepted: AtomicUsize::new(0),
        zero_writes: AtomicUsize::new(0),
        tx_interrupts: AtomicUsize::new(0),
        wakes: AtomicUsize::new(0),
    };

    /// `ktest` 模式下串口发送路径统计计数器的时间点快照。
    #[cfg(feature = "ktest")]
    #[derive(Clone, Copy)]
    pub struct Snapshot {
        /// [`try_write`] 成功写入硬件 `DATA` 寄存器的累计字节数。
        pub accepted: usize,
        /// [`try_write`] 传入非空切片但因硬件发送未就绪而返回 0 的累计次数。
        pub zero_writes: usize,
        /// 中断处理函数处理 `THRE` 发送就绪中断的累计次数。
        pub tx_interrupts: usize,
        /// 发送就绪中断从 [`TX_WAIT`] 唤醒的任务累计数量。
        pub wakes: usize,
    }

    /// 读取当前串口发送路径统计计数器的快照（不影响接收缓冲区或硬件寄存器状态）。
    #[cfg(feature = "ktest")]
    pub fn snapshot() -> Snapshot {
        Snapshot {
            accepted: COUNTERS.accepted.load(Ordering::Relaxed),
            zero_writes: COUNTERS.zero_writes.load(Ordering::Relaxed),
            tx_interrupts: COUNTERS.tx_interrupts.load(Ordering::Relaxed),
            wakes: COUNTERS.wakes.load(Ordering::Relaxed),
        }
    }
    /// 在 `PORT` 自旋锁内声明独占测试接收控制权，关闭硬件 `RECEIVE_INTERRUPT` 并返回此前接收中断是否开启。
    ///
    /// 调用上下文：测试框架开始一段串口接收测试或握手前调用；调用前不得已处于测试控制状态或留有未消费完的配额。
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

    /// 在持有 `PORT` 锁的前提下校验当前已声明测试接收控制权、硬件接收中断已关闭且接收配额已清零。
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

    /// 等待当前正在执行的默认串口中断处理函数彻底执行完毕并退出。
    ///
    /// 调用上下文：必须处于可调度的线程上下文，已通过 [`claim_receive_control`] 取得控制权、关闭硬件接收中断且配额已耗尽；
    /// 调用时不得持有 `PORT` 锁。内部通过临时取出默认注册句柄、执行 `disable_sync` 再重新启用归还，确保此前已经在途的
    /// 中断回调彻底退出。
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

    /// 在等待在途串口中断处理函数彻底退出后，设置后续中断接收到的字节是否送入 TTY 软件环形缓冲区。
    ///
    /// 调用上下文：已获取测试接收控制权、关闭硬件接收中断且配额已清零的可调度线程。当 `attached` 为 `true` 时，
    /// 要求软件接收环形缓冲区当前必须为空。
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

    /// 在 `PORT` 自旋锁内设置本段测试允许接收中断消费的字节配额 `length`，并在 `length > 0` 时开启硬件接收中断。
    ///
    /// 调用上下文：调用方须已取得测试接收控制权、关闭硬件接收中断、通过轮询读走握手命令字节 `H`，且已排空软件接收环。
    /// 当中断处理函数收满 `length` 个字节后会自动在锁内关闭接收中断，使紧随其后的确认字节 `ACK` 留在硬件 FIFO 中供轮询读取。
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

    /// 返回当前测试段尚未被接收中断消费的剩余字节配额；若未设置配额或已全部收齐则返回 `None`。
    #[cfg(feature = "ktest")]
    pub fn receive_quota_remaining() -> Option<usize> {
        super::PORT.lock().quota
    }

    /// 判断软件接收环形缓冲区当前是否为空（仅检查 `len == 0`，不弹出任何数据）。
    #[cfg(feature = "ktest")]
    pub fn receive_queue_empty() -> bool {
        super::PORT.lock().len == 0
    }

    /// 在确认本段测试配额已耗尽且软件接收环已清空后，释放测试接收控制权并根据 `receiver` 恢复硬件接收中断使能位。
    ///
    /// 调用上下文：测试段结束且已通过轮询读走握手确认字节 `ACK` 后调用；保留 `IER` 中的发送中断使能位不变。
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
