//! 全局唯一串口终端（TTY）字符设备端点、工作队列驱动与控制终端会话管理。
//!
//! # 模块职责与架构定位
//! 本模块将底层 8250/16550 串口控制台（`crate::console`）、`N_TTY` 行规程状态机（[`ntty`]）以及进程/会话控制终端关系（`crate::task::process::TaskGraph`）整合为 VFS 字符设备：
//! - **上游调用者**：
//!   - VFS 设备打开入口：将 `/dev/console`（`MemorySpecial::Console`）、`/dev/ttyS0`（`MemorySpecial::TtySerial`）与 `/dev/tty`（`MemorySpecial::ControllingTty`）统一绑定到同一个常驻 [`Endpoint`]；
//!   - 系统调用层：通过 `FileLike` / `Pollable` 接口执行 `read`、`write`、`ioctl`、`poll` / `epoll`；
//!   - 串口中断处理程序：通过 [`schedule_receive`] 与 [`schedule_transmit`] 将底半部处理提交到 TTY 专用工作队列；
//!   - 进程退出路径：会话首进程（Session Leader）退出时调用 [`hangup_session`] 触发控制终端挂断。
//! - **下游依赖**：[`ntty::NttyState`]（输入/回显/输出三队列与行规程）、[`termios`]（配置编解码與波特率换算）、`crate::console`（串口寄存器配置与收发缓冲）、`crate::task::process::TaskGraph`（控制终端权限检查与进程组信号发送）。
//!
//! # 核心架构与数据流
//!
//! ```text
//!  UART RX 中断 ──► schedule_receive() ──┐
//!  UART TX 中断 ──► schedule_transmit() ─┼──► WorkQueue (run_work, CPU0)
//!                                        │      ├─ 1. 持 copy 锁批量拉取 console RX 采样
//!                                        │      ├─ 2. 持 state 锁执行 NttyState::receive_sample & process_echo
//!                                        │      ├─ 3. 锁外向控制终端前台进程组发送 SIGINT/SIGQUIT/SIGTSTP
//!                                        │      ├─ 4. pump_transmit() 非阻塞写入 UART TX FIFO，满则开启 THRE 中断
//!                                        │      └─ 5. 唤醒 read_head / write_head 上的阻塞任务与 epoll 实例
//!
//!  用户态 read()  ──► check_read (SIGTTIN) ──► reader 睡眠锁 ──► canonical_read / noncanonical_read
//!  用户态 write() ──► check_write(SIGTTOU) ──► writer 睡眠锁 ──► 持 copy + state 锁经 OPOST 入队 ──► schedule_transmit()
//! ```
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! 为避免死锁及在自旋锁内睡眠，本模块严格遵守以下锁层级顺序（由外向内）：
//!
//! ```text
//!   reader / writer (SleepMutex，长耗时读写与 TCSADRAIN/TCSAFLUSH 串行化)
//!         │
//!         ▼
//!   copy (SleepMutex，单次缓冲段拷贝、输入刷新、termios/winsize 提交、控制终端变更与 TX 泵串行化)
//!         │
//!         ├──► TaskGraph::relations (SpinLock) ──► Endpoint::control (SpinLock)
//!         │
//!         ├──► Endpoint::state (SpinLock<NttyState>，行规程内存队列，严禁跨用户拷贝或持锁调 console I/O)
//!         │
//!         ├──► Endpoint::winsize (SpinLock<Winsize>)
//!         │
//!         └──► crate::console 端口自旋锁（configure_serial / try_write / arm_tx_interrupt）
//! ```
//! - **信号投递铁律**：任何向进程组发送信号的操作（`SIGINT`、`SIGQUIT`、`SIGTSTP`、`SIGHUP`、`SIGCONT`、`SIGWINCH`）必须在释放 `state`、`winsize` 及 `copy` 锁之后执行。
//! - **唤醒通知铁律**：`PollWaitHead::notify` 必须在释放 `state` 与 `copy` 锁之后调用；`HangupState::notify` 作为注册在 `hangup_head` 上的同步回调，仅短暂获取 `state` 自旋锁递减 `opens` 计数，绝不获取睡眠锁或任务图锁。

pub mod ntty;
pub mod termios;

use alloc::sync::{Arc, Weak};
use core::ops::Range;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::time::Duration;
use spin::Once;

use crate::console::RxSample;
use crate::fs::epoll::WatchHead;
use crate::fs::file::{
    FileIoContext, FileLike, FileRef, FileStat, ReadBuffer, SeekFrom, WriteBuffer,
};
use crate::fs::memfs::MemorySpecial;
use crate::fs::poll::{
    PollContext, PollMask, PollNotify, PollRegistration, PollTable, PollWaitHead, Pollable,
};
use crate::fs::{
    DirectoryEntry, FsError, InodeRef, Path, O_ACCMODE, O_DIRECTORY, O_NOCTTY, O_NONBLOCK, O_WRONLY,
};
use crate::sync::{SleepMutex, SpinLock};
use crate::task::process::{Pid, TaskGraph, TerminalControl};
use crate::task::signal::{SignalError, SIGCONT, SIGHUP, SIGTTIN, SIGTTOU, SIGWINCH};
use crate::task::UserThread;
use crate::time::{self, MonotonicInstant};
use crate::wait::WaitResult;
use crate::workqueue::{Work, WorkQueue, WorkQueueConfig};

use ntty::{NttyState, SEGMENT_BYTES};
use termios::{
    Termios, TCIFLUSH, TCIOFF, TCIOFLUSH, TCION, TCOFLUSH, TCOOFF, TCOON, TCSADRAIN, TCSAFLUSH,
    TCSANOW, VMIN, VTIME,
};

/// `ioctl` 命令：读取当前终端的 `struct termios` 配置（`TCGETS`）。
const TCGETS: u32 = 0x5401;
/// `ioctl` 命令：立即设置终端的 `struct termios` 配置（`TCSETS`，等价于 `TCSANOW`）。
const TCSETS: u32 = 0x5402;
/// `ioctl` 命令：等待输出排空后设置 `struct termios` 配置（`TCSETSW`，等价于 `TCSADRAIN`）。
const TCSETSW: u32 = 0x5403;
/// `ioctl` 命令：等待输出排空并清空未读输入后设置 `struct termios` 配置（`TCSETSF`，等价于 `TCSAFLUSH`）。
const TCSETSF: u32 = 0x5404;
/// `ioctl` 命令：等待输出排空或发送线路 BREAK（`TCSBRK`）。
const TCSBRK: u32 = 0x5409;
/// `ioctl` 命令：软件流控暂停/恢复或发送 XOFF/XON 字符（`TCXONC`）。
const TCXONC: u32 = 0x540a;
/// `ioctl` 命令：清空未读输入队列、未发送输出队列或两者（`TCFLSH`）。
const TCFLSH: u32 = 0x540b;

/// `ioctl` 命令：将当前终端设为调用会话的控制终端（`TIOCSCTTY`）。
const TIOCSCTTY: u32 = 0x540e;
/// `ioctl` 命令：查询控制终端的前台进程组 ID（`TIOCGPGRP`）。
const TIOCGPGRP: u32 = 0x540f;
/// `ioctl` 命令：设置控制终端的前台进程组 ID（`TIOCSPGRP`）。
const TIOCSPGRP: u32 = 0x5410;
/// `ioctl` 命令：查询终端窗口行列与像素尺寸（`TIOCGWINSZ`）。
const TIOCGWINSZ: u32 = 0x5413;
/// `ioctl` 命令：设置终端窗口行列与像素尺寸，变化时向前台进程组发送 `SIGWINCH`（`TIOCSWINSZ`）。
const TIOCSWINSZ: u32 = 0x5414;
/// `ioctl` 命令：主动放弃当前会话的控制终端（`TIOCNOTTY`）。
const TIOCNOTTY: u32 = 0x5422;
/// `ioctl` 命令：查询控制终端所属会话的会话 ID（`TIOCGSID`）。
const TIOCGSID: u32 = 0x5429;

/// 工作队列单轮从底层串口软件接收环最多拉取的采样数。
const WORK_RX_BUDGET: usize = 64;
/// `write` 系统调用单次从用户缓冲区拉取的字节分段大小（单位：字节）。
const WRITE_SEGMENT: usize = 512;
/// 单个 `TtyFile` 在 `PollTable` 中注册的等待队列头数量（读等待头 + 写等待头，共 2 个）。
const POLL_SOURCES: usize = 2;
/// `termios` 中 `VTIME` 每个单位对应的毫秒数（0.1 秒 = 100 毫秒）。
const VTIME_MILLIS: u64 = 100;

/// 全局常驻的唯一串口 TTY 端点，管理行规程状态、并发串行锁、等待队列与控制终端绑定。
pub(crate) struct Endpoint {
    /// 指向全局静态 `N_TTY` 行规程状态的自旋锁引用（持锁时间极短，不可睡眠）。
    state: &'static SpinLock<NttyState>,
    /// 串行化并发 `read` 系统调用的睡眠互斥锁，确保单次逻辑读取期间不会被其他读者插队拆分。
    reader: SleepMutex<()>,
    /// 串行化并发 `write` 系统调用与有序配置变更（`TCSETSW` / `TCSETSF`）的睡眠互斥锁。
    writer: SleepMutex<()>,
    /// 串行化单段数据收发、输入/输出队列刷新、硬件寄存器重配与控制终端事务的内部睡眠互斥锁。
    copy: SleepMutex<()>,
    /// 可读事件（`POLLIN` / `POLLRDNORM`）及读者锁释放的等待队列头。
    read_head: Arc<PollWaitHead>,
    /// 可写事件（`POLLOUT` / `POLLWRNORM`）、输出排空及写者锁释放的等待队列头。
    write_head: Arc<PollWaitHead>,
    /// 当前绑定的控制终端关系（保存所属任务图弱引用、控制会话 PID 与前台进程组 PID）。
    control: SpinLock<Option<TerminalControl>>,
    /// 会话首进程退出触发终端挂断（Hangup）时通知所有活跃非控制台打开实例的等待队列头。
    hangup_head: Arc<PollWaitHead>,
    /// 当前终端窗口尺寸（由 `copy` 睡眠锁与本自旋锁共同保护）。
    winsize: SpinLock<termios::Winsize>,
}

impl Endpoint {
    /// 返回控制终端绑定关系的自旋锁引用（由 `TaskGraph` 按 `relations -> control` 锁序访问）。
    pub(crate) fn control(&self) -> &SpinLock<Option<TerminalControl>> {
        &self.control
    }
}

/// 单个非 `/dev/console` 打开文件描述（OFD）的会话挂断状态跟踪器。
///
/// 注册在 `Endpoint::hangup_head` 上；当控制会话首进程退出触发 `POLLHUP` 广播时，
/// 将 `hungup` 原子置为 `true`（不可逆），并立即归还该 OFD 持有的活跃打开计数 `state.opens`。
struct HangupState {
    /// 该打开文件描述是否已被永久挂断。
    hungup: AtomicBool,
}

impl PollNotify for HangupState {
    /// 响应 `hangup_head` 广播的 `POLLHUP` 事件：原子标记本 OFD 已挂断，并在 `state` 自旋锁下将活跃打开数减一。
    fn notify(&self, mask: PollMask) {
        if mask.contains(PollMask::POLLHUP) && !self.hungup.swap(true, Ordering::AcqRel) {
            let mut state = endpoint().state.lock();
            assert!(0 < state.opens, "tty hangup open count underflow");
            state.opens -= 1;
        }
    }
}

/// 全局常驻的 TTY 后台服务容器（包含端点实例、专用工作队列与绑定的异步工作项）。
struct Service {
    /// 全局唯一 TTY 端点。
    endpoint: Endpoint,
    /// 专用于执行 TTY 收发与回显底半部的内核工作队列。
    _queue: WorkQueue,
    /// 绑定在 CPU 0 上的常驻工作项（执行 [`run_work`]）。
    work: Pin<Arc<Work>>,
}

/// 全局唯一的 `N_TTY` 行规程状态静态实例。
static STATE: SpinLock<NttyState> = SpinLock::new(NttyState::new());

/// 全局唯一的 TTY 服务延迟初始化容器。
static SERVICE: Once<Service> = Once::new();

/// 获取已初始化的全局 TTY 服务引用。
fn service() -> &'static Service {
    SERVICE.get().expect("tty service is not initialized")
}

/// 获取全局常驻的 TTY 端点引用。
pub(crate) fn endpoint() -> &'static Endpoint {
    &service().endpoint
}

/// `Endpoint::reader` 睡眠锁的 RAII 守卫：析构时先释放 `reader` 锁，再通知 `read_head` 唤醒其他排队等待 `reader` 锁的读者。
struct ReaderGuard {
    /// 底层 `reader` 睡眠互斥锁守卫。
    inner: Option<crate::sync::SleepMutexGuard<'static, ()>>,
}

impl Drop for ReaderGuard {
    /// 先释放 `reader` 睡眠锁，再向 `read_head` 发送可读通知以唤醒竞争同一把 `reader` 锁的等待线程。
    fn drop(&mut self) {
        if let Some(guard) = self.inner.take() {
            drop(guard);
            endpoint()
                .read_head
                .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        }
    }
}

/// `Endpoint::writer` 睡眠锁的 RAII 守卫：析构时先释放 `writer` 锁，再通知 `write_head` 唤醒其他排队等待 `writer` 锁的写者或配置変更者。
struct WriterGuard {
    /// 底层 `writer` 睡眠互斥锁守卫。
    inner: Option<crate::sync::SleepMutexGuard<'static, ()>>,
}

impl Drop for WriterGuard {
    /// 先释放 `writer` 睡眠锁，再向 `write_head` 发送可写通知以唤醒竞争同一把 `writer` 锁的等待线程。
    fn drop(&mut self) {
        if let Some(guard) = self.inner.take() {
            drop(guard);
            endpoint()
                .write_head
                .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
        }
    }
}

/// 初始化全局 TTY 常驻端点、专用工作队列与异步收发工作任务（在内核工作队列子系统就绪后调用一次）。
pub fn initialize_service() -> Result<(), core::alloc::AllocError> {
    // 端点的初始速度采用平台声明的控制台速率，避免首次打开把线路从既有速率改走。
    {
        let speed = termios::speed_for_baud(crate::console::console_baud())
            .expect("console baud is not representable in termios");
        let mut state = STATE.lock();
        state.termios.cflag = (state.termios.cflag & !termios::CBAUD) | speed;
    }

    let read_head = Arc::try_new(PollWaitHead::new())?;
    let write_head = Arc::try_new(PollWaitHead::new())?;
    let hangup_head = Arc::try_new(PollWaitHead::new())?;
    let queue = WorkQueue::try_new(WorkQueueConfig {
        max_active: 1,
        ordered: false,
        reclaim: false,
    })?;
    let work = Work::try_new(&queue, 0, run_work, 0)?;

    SERVICE.call_once(move || Service {
        endpoint: Endpoint {
            state: &STATE,
            reader: SleepMutex::new(()),
            writer: SleepMutex::new(()),
            copy: SleepMutex::new(()),
            read_head,
            write_head,
            control: SpinLock::new(None),
            hangup_head,
            winsize: SpinLock::new(termios::Winsize::initial()),
        },
        _queue: queue,
        work,
    });
    Ok(())
}

/// 由串口接收中断在释放端口锁后调用：向 TTY 工作队列提交异步任务，将软件接收环中的采样送入行规程处理。
pub fn schedule_receive() {
    if let Some(service) = SERVICE.get() {
        let _ = service.work.as_ref().queue();
    }
}

/// 由串口发送保持寄存器空（THRE）中断或用户写入路径在端口锁外调用：若行规程仍有待发送输出，则调度 TTY 工作任务继续向硬件泵送数据。
pub fn schedule_transmit() {
    let Some(service) = SERVICE.get() else {
        return;
    };
    if service.endpoint.state.lock().output_empty() {
        return;
    }
    let _ = service.work.as_ref().queue();
}

/// TTY 工作队列回调函数：批量拉取串口接收采样、展开回显指令、投递前台信号，并向 UART 硬件非阻塞发送输出数据。
fn run_work(_work: Pin<&Work>) {
    let service = service();
    // Step 1: 获取 copy 锁，抓取当前前台进程组快照并从底层 console 接收环批量拉取最多 WORK_RX_BUDGET 个采样。
    let _copy = service.endpoint.copy.lock();
    let mut samples = [RxSample { byte: 0, status: 0 }; WORK_RX_BUDGET];
    let foreground = foreground_snapshot();
    let mut signals = [0u8; WORK_RX_BUDGET];
    let mut signal_count = 0;
    let count = crate::console::read_received(&mut samples);
    let mut read_changed = false;
    let mut write_changed = false;
    let mut progressed = false;
    {
        // Step 2: 在 state 自旋锁内将接收到的采样逐个送入行规程，并展开回显队列。
        let mut state = service.endpoint.state.lock();
        if 0 < count {
            progressed = true;
        }
        if state.opens == 0 {
            // 当前没有任何活跃打开实例时丢弃新接收的采样，但继续展开此前已入队的回显操作。
        } else {
            for sample in samples[..count].iter() {
                let outcome = state.receive_sample(*sample);
                read_changed |= outcome.read_changed;
                write_changed |= outcome.write_changed;
                if let Some(signal) = outcome.signal {
                    signals[signal_count] = signal;
                    signal_count += 1;
                }
            }
        }

        if !state.echo.is_empty() {
            write_changed = true;
        }
        let echo_before = state.echo.len();
        let output_before = state.output.len();
        let reprint_before = state.pending_reprint.as_ref().map(|p| p.done);
        let echo_pending = state.process_echo();
        let reprint_after = state.pending_reprint.as_ref().map(|p| p.done);
        if state.echo.len() != echo_before
            || state.output.len() != output_before
            || reprint_before != reprint_after
        {
            progressed = true;
        }
        if echo_pending {
            write_changed = true;
        }
    }
    drop(_copy);

    // Step 3: 在释放 state 和 copy 锁后，向控制终端的前台进程组投递输入字符触发的信号（如 SIGINT/SIGQUIT/SIGTSTP）。
    if let Some((graph, foreground)) = foreground {
        for &signum in &signals[..signal_count] {
            send_foreground(&graph, &foreground, signum);
        }
    }

    // Step 4: 非阻塞地将输出队列中的字节写入 UART 发送 FIFO，并唤醒等待可读/可写事件的任务。
    let pumped = pump_transmit(service, false);
    if pumped {
        progressed = true;
    }
    if read_changed {
        service
            .endpoint
            .read_head
            .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
    }
    if write_changed {
        service
            .endpoint
            .write_head
            .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
    }

    // Step 5: 若本轮取得了实际进展，且仍有未读完的 RX 批次、未展开完的回显或未受流控阻塞的待发输出，则继续排队下一轮工作任务。
    if progressed {
        let state = service.endpoint.state.lock();
        let more_rx = 0 < count && count == WORK_RX_BUDGET;
        let more_echo = !state.echo.is_empty();
        let more_tx =
            !state.output_empty() && !state.stopped && !state.tco_stopped && !state.flusho();
        drop(state);
        if more_rx || more_echo || more_tx {
            let _ = service.work.as_ref().queue();
        }
    }
}

/// 非阻塞地将行规程输出队列或高优先级字符 `xchar` 泵入底层 UART 发送 FIFO，返回是否实际发送了至少一个字节。
///
/// 锁获取顺序契约：
/// 1. 全程持有 `copy` 睡眠锁，防止与 `TCFLSH` / `TCSETS*` 并发修改队首；
/// 2. 短暂获取 `state` 自旋锁预览队首字节（或处理 `FLUSHO` 清空），随即释放 `state` 自旋锁；
/// 3. 在不持有 `state` 的状态下调用 `console::try_write`（内部获取串口硬件锁）；
/// 4. 若硬件 FIFO 已满（写入 0 字节），则开启 UART THRE 发送空中断并在硬件已就绪时重新入队工作项；
/// 5. 若硬件写入成功，再次短暂获取 `state` 自旋锁将刚发送的字节从队首弹出，并通知 `write_head` 唤醒等待可写空间的写者。
fn pump_transmit(service: &'static Service, force: bool) -> bool {
    let _copy = service.endpoint.copy.lock();
    let mut pumped = false;
    loop {
        /// 预览待发送的下一字节类型。
        enum Peek {
            /// 高优先级流控字符（`VSTOP` / `VSTART`）。
            Xchar(u8),
            /// 普通输出队列队首字节。
            Output(u8),
            /// 当前无字节可发（队列为空、已处于 `FLUSHO` 丢弃态或被流控暂停）。
            None,
        }
        let peeked = {
            let mut state = service.endpoint.state.lock();
            if state.flusho() {
                state.output.clear();
                state.echo.clear();
                state.xchar = None;
                state.pending_reprint = None;
                Peek::None
            } else if let Some(xchar) = state.xchar {
                Peek::Xchar(xchar)
            } else if state.stopped || state.tco_stopped {
                Peek::None
            } else if let Some(byte) = state.output.peek() {
                Peek::Output(byte)
            } else {
                Peek::None
            }
        };
        let byte = match peeked {
            Peek::Xchar(byte) | Peek::Output(byte) => byte,
            Peek::None => {
                if force {
                    service
                        .endpoint
                        .write_head
                        .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
                }
                return pumped;
            }
        };

        if crate::console::try_write(&[byte]) == 0 {
            if crate::console::arm_tx_interrupt() {
                let _ = service.work.as_ref().queue();
            }
            return pumped;
        }

        {
            let mut state = service.endpoint.state.lock();
            match peeked {
                Peek::Xchar(_) => {
                    state.xchar = None;
                }
                Peek::Output(_) => {
                    state.output.pop();
                }
                Peek::None => {}
            }
        }
        pumped = true;
        service
            .endpoint
            .write_head
            .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
    }
}

/// 等待当前已提交到 TTY 工作队列的异步任务执行完毕并退出（仅用于内核集成测试同步点）。
#[cfg(feature = "ktest")]
pub fn flush_work() {
    service().work.as_ref().flush();
}

/// TTY 字符设备的打开文件描述（OFD）实例。
///
/// 无论是通过 `/dev/console`、`/dev/ttyS0` 还是 `/dev/tty` 打开，底层均共享同一个常驻 [`Endpoint`]，
/// 但每个 `TtyFile` 独立维护自己的访问标志、`epoll` 反向监听链以及会话挂断状态。
pub struct TtyFile {
    /// 打开时对应的设备节点路径（用于 `fstat` 和 `/proc/self/fd` 路径展示）。
    path: Path,
    /// 打开文件描述的访问模式与状态标志（如 `O_RDWR`、`O_NONBLOCK`）。
    flags: AtomicU32,
    /// 挂载在此打开文件实例上的 `epoll` 监控项链表头。
    watches: Arc<WatchHead>,
    /// 标识本实例是否已成功递增过 `NttyState::opens` 活跃打开计数（用于 `Drop` 时判断是否需要递减）。
    opened: bool,
    /// 是否通过 `/dev/console` 打开（为 `true` 时不参与会话挂断，也不受后台写 `TOSTOP` 信号限制）。
    console: bool,
    /// 非 `/dev/console` 打开实例持有的永久挂断状态标志与 `hangup_head` 回调注册守卫。
    hangup: Option<(Arc<HangupState>, PollRegistration)>,
}

impl TtyFile {
    /// 检查本打开文件实例是否已因控制会话退出而处于永久挂断（Hangup）状态。
    fn hungup(&self) -> bool {
        self.hangup
            .as_ref()
            .is_some_and(|(state, _)| state.hungup.load(Ordering::Acquire))
    }

    /// 执行读取前的前置检查：
    /// 1. 若本打开实例已被挂断，返回 `Ok(false)`（调用方应立即返回 `0` 表示 EOF）；
    /// 2. 若调用方属于控制终端的后台进程组，按 POSIX 规则向其发送 `SIGTTIN` 或返回 `EIO`。
    fn check_read(&self, caller: FileIoContext<'_>) -> Result<bool, FsError> {
        if self.hungup() {
            return Ok(false);
        }
        check_access(caller, SIGTTIN)?;
        Ok(true)
    }

    /// 执行写入前的前置检查：
    /// 1. 若本打开实例已被挂断，返回 `Err(FsError::DeviceIo)`（`EIO`）；
    /// 2. 若非 `/dev/console` 且当前 `termios` 启用了 `TOSTOP`，则检查后台进程组写入权限（必要时触发 `SIGTTOU`）。
    fn check_write(&self, caller: FileIoContext<'_>) -> Result<(), FsError> {
        if self.hungup() {
            return Err(FsError::DeviceIo);
        }
        let tostop = endpoint().state.lock().termios.lflag & termios::TOSTOP != 0;
        if !self.console && tostop {
            check_access(caller, SIGTTOU)?;
        }
        Ok(())
    }

    /// 为需要阻塞等待的读写操作分配 `PollContext` 与 `PollTable`，并注册到端点的 `read_head` 与 `write_head` 等待队列。
    ///
    /// 调用方在完成注册后、进入睡眠（`park_until`）前，必须重新检查业务状态以防止丢失唤醒（Lost Wakeup）。
    fn prepare_wait(&self) -> Result<(Arc<PollContext>, PollTable), FsError> {
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let mut table = PollTable::new(sink, POLL_SOURCES)?;
        self.poll(Some(&mut table))?;
        Ok((context, table))
    }
}

impl Drop for TtyFile {
    /// 关闭最后一个指向本打开文件描述的引用（`Arc<TtyFile>` 被 Drop）时：
    /// 1. 释放所有关联的 `epoll` 监控项并注销 `hangup_head` 回调；
    /// 2. 若本实例尚未被会话挂断提前扣减 `opens` 计数，则递减 `state.opens`；
    /// 3. 当 `state.opens` 归零且 `termios` 启用了 `HUPCL` 时，拉低串口硬件的 DTR 与 RTS 控制线并关闭接收器。
    fn drop(&mut self) {
        self.watches.release();
        let hungup = self.hangup.take().is_some_and(|(state, registration)| {
            drop(registration);
            state.hungup.load(Ordering::Acquire)
        });
        if !self.opened || hungup {
            return;
        }
        let copy = endpoint().copy.lock();
        let config = {
            let mut state = endpoint().state.lock();
            assert!(0 < state.opens, "tty open count underflow");
            state.opens -= 1;
            if state.opens != 0 || state.termios.cflag & termios::HUPCL == 0 {
                return;
            }
            let dropped = state.dtr || state.rts;
            state.dtr = false;
            state.rts = false;
            if !dropped {
                return;
            }
            crate::console::SerialConfig {
                divisor: state.divisor,
                line_control: termios::line_control(state.termios.cflag),
                receiver: false,
                dtr: false,
                rts: false,
            }
        };
        let _ = copy;
        crate::console::configure_serial(config);
    }
}

impl Pollable for TtyFile {
    /// 向 `PollTable` 注册端点的读写等待队列，并根据当前挂断状态或行规程队列状态返回就绪事件掩码。
    fn poll(&self, table: Option<&mut PollTable>) -> Result<PollMask, FsError> {
        if let Some(table) = table {
            table.register(&endpoint().read_head)?;
            table.register(&endpoint().write_head)?;
        }

        if self.hungup() {
            return Ok(PollMask::POLLIN
                | PollMask::POLLOUT
                | PollMask::POLLERR
                | PollMask::POLLHUP
                | PollMask::POLLRDNORM
                | PollMask::POLLWRNORM);
        }
        let state = endpoint().state.lock();
        let mut mask = PollMask::default();
        if state.poll_readable() {
            mask |= PollMask::POLLIN | PollMask::POLLRDNORM;
        }
        if state.poll_writable() {
            mask |= PollMask::POLLOUT | PollMask::POLLWRNORM;
        }
        Ok(mask)
    }
}

/// 打开 `/dev/console`、`/dev/ttyS0` 或 `/dev/tty` 特殊设备节点并返回 `TtyFile` 实例。
///
/// 执行步骤：
/// 1. 校验设备类型与打开标志（拒绝 `O_DIRECTORY`），并预分配 `WatchHead`、`HangupState` 与 `TtyFile` 内存；
/// 2. 获取 `copy` 睡眠锁：
///    - 若打开的是 `/dev/tty`（`ControllingTty`），校验调用进程当前是否已将本端点作为控制终端，否则返回 `ENXIO`（`NoDeviceAddress`）；
///    - 为非 `/dev/console` 打开实例在 `hangup_head` 上注册挂断通知回调；
///    - 若通过 `/dev/ttyS0` 打开、未指定 `O_NOCTTY` 且非只写（`!O_WRONLY`），则在满足会话首进程条件时自动获取控制终端；
/// 3. 根据当前 `termios` 激活串口硬件（拉高 DTR/RTS 并使能接收），将 `state.opens` 加 1 后返回文件引用。
pub fn open(
    path: Path,
    flags: u32,
    caller: FileIoContext<'_>,
    special: MemorySpecial,
) -> Result<FileRef, FsError> {
    // Step 1: 校验特殊设备类型与打开标志，预分配 epoll 监听头、挂断状态与 TtyFile 实例。
    let console = match special {
        MemorySpecial::Console => true,
        MemorySpecial::TtySerial | MemorySpecial::ControllingTty => false,
        _ => return Err(FsError::InvalidOperation),
    };
    if flags & O_DIRECTORY != 0 {
        return Err(FsError::NotDirectory);
    }

    let watches = Arc::try_new(WatchHead::new()).map_err(|_| FsError::NoMemory)?;
    let hangup_state = if console {
        None
    } else {
        Some(
            Arc::try_new(HangupState {
                hungup: AtomicBool::new(false),
            })
            .map_err(|_| FsError::NoMemory)?,
        )
    };
    let mut file = Arc::try_new(TtyFile {
        path,
        flags: AtomicU32::new(flags),
        watches,
        opened: false,
        console,
        hangup: None,
    })
    .map_err(|_| FsError::NoMemory)?;

    // Step 2: 在 copy 锁保护下校验 /dev/tty 控制终端绑定、注册挂断回调并尝试自动获取控制终端。
    let copy = endpoint().copy.lock();
    let identity = match caller {
        FileIoContext::User(user) => user.identity(),
        FileIoContext::Kernel => None,
    };
    if special == MemorySpecial::ControllingTty {
        let bound = identity
            .as_ref()
            .and_then(|identity| identity.graph().controlling_tty(identity));
        if !bound.is_some_and(|bound| core::ptr::eq(bound, endpoint())) {
            return Err(FsError::NoDeviceAddress);
        }
    }
    let hangup = if let Some(state) = hangup_state {
        let sink: Arc<dyn PollNotify> = state.clone();
        let registration = endpoint().hangup_head.register(sink)?;
        Some((state, registration))
    } else {
        None
    };
    let config = {
        let state = endpoint().state.lock();
        termios::serial_config(state.termios, state.divisor, true, true)
            .ok_or(FsError::InvalidOperation)?
    };
    if special == MemorySpecial::TtySerial && flags & O_NOCTTY == 0 && flags & O_ACCMODE != O_WRONLY
    {
        if let Some(identity) = &identity {
            identity.graph().acquire_tty(identity, endpoint(), true)?;
        }
    }

    // Step 3: 应用串口硬件配置（拉高 DTR/RTS 并更新分频），递增活跃打开计数并提交打开状态。
    let snapshot = crate::console::configure_serial(config);
    {
        let mut state = endpoint().state.lock();
        state.dtr = config.dtr;
        state.rts = config.rts;
        state.divisor = snapshot.divisor;
        state.opens += 1;
    }
    Arc::get_mut(&mut file)
        .expect("tty candidate shared")
        .opened = true;
    Arc::get_mut(&mut file)
        .expect("tty candidate shared")
        .hangup = hangup;
    drop(copy);
    Ok(file)
}

impl FileLike for TtyFile {
    /// 从终端读取输入字节并写入 `target` 缓冲区。
    ///
    /// 执行步骤：
    /// 1. 校验打开文件描述具备可读权限，且未处于挂断状态或受后台进程组 `SIGTTIN` 限制；
    /// 2. 获取 `reader` 睡眠锁以串行化并发读取请求；
    /// 3. 根据当前 `termios` 的 `ICANON` 标志分发到规范模式行读取（[`canonical_read`]）或非规范模式读取（[`noncanonical_read`]）。
    fn read_into(
        &self,
        target: &mut ReadBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        crate::fs::file::require_read(self.flags())?;
        if target.len() == target.copied() {
            return Ok(0);
        }
        if !self.check_read(caller)? {
            return Ok(0);
        }

        let Some(_reader) = acquire_reader(self, caller)? else {
            return Ok(0);
        };
        let termios = endpoint().state.lock().termios;
        if termios.lflag & termios::ICANON != 0 {
            canonical_read(self, target, caller)
        } else {
            noncanonical_read(self, target, caller, termios)
        }
    }

    /// 终端字符设备为流式设备，不支持带偏移量的定位读取（`pread`），恒返回 `FsError::IllegalSeek`。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 返回打开本终端文件时对应的设备节点路径。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// 终端字符设备不支持 `mmap` 页缓存文件映射。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 返回 `Pollable` 接口引用，支持 `poll` / `ppoll` / `select` / `epoll` 事件多路复用。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    /// 返回挂载在本打开文件实例上的 `epoll` 监控链表头。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        Some(&self.watches)
    }

    /// 处理终端 `ioctl` 控制命令（包括 `termios` 读写、排空与流控、队列清空、控制终端与前台进程组管理、窗口尺寸读写）。
    ///
    /// - 若本打开实例已挂断，`TIOCSPGRP` 返回 `ENOTTY`，其余命令返回 `EIO`；
    /// - 对于会修改终端配置或流控状态的命令，先执行后台进程组访问检查（可能触发 `SIGTTOU`）；
    /// - 携带用户空间指针参数的命令仅允许在 `FileIoContext::User` 上下文中通过用户地址空间安全拷贝执行。
    fn ioctl(
        &self,
        command: u32,
        argument: usize,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        if self.hungup() {
            return Err(if command == TIOCSPGRP {
                FsError::NotTty
            } else {
                FsError::DeviceIo
            });
        }
        let changes = matches!(
            command,
            TCSETS | TCSETSW | TCSETSF | TCXONC | TCFLSH | TIOCSPGRP
        ) || command == TCSBRK && argument != 0;
        if changes {
            check_access(caller, SIGTTOU).map_err(|error| match (command, error) {
                (TIOCSPGRP, FsError::DeviceIo) => FsError::NotTty,
                (_, error) => error,
            })?;
        }

        let FileIoContext::User(user) = caller else {
            return Err(match command {
                TCGETS | TCSETS | TCSETSW | TCSETSF | TCSBRK | TCXONC | TCFLSH | TIOCSCTTY
                | TIOCNOTTY | TIOCGPGRP | TIOCSPGRP | TIOCGSID | TIOCGWINSZ | TIOCSWINSZ => {
                    FsError::InvalidOperation
                }
                _ => FsError::NotTty,
            });
        };

        match command {
            TCGETS => ioctl_get(user, argument),
            TCSETS => ioctl_set(self, user, argument, TCSANOW, caller),
            TCSETSW => ioctl_set(self, user, argument, TCSADRAIN, caller),
            TCSETSF => ioctl_set(self, user, argument, TCSAFLUSH, caller),
            TCSBRK => {
                if argument == 0 {
                    return Err(FsError::InvalidOperation);
                }
                drain_output(caller, Some(self)).map(|()| 0)
            }
            TCXONC => endpoint_flow(argument as u32, Some(self)),
            TCFLSH => endpoint_flush(argument as u32, Some(self)).map(|()| 0),
            TIOCSCTTY | TIOCNOTTY | TIOCGPGRP | TIOCSPGRP | TIOCGSID => {
                ioctl_control(self, user, command, argument)
            }
            TIOCGWINSZ => {
                let bytes = endpoint().winsize.lock().to_bytes();
                copy_ioctl_out(user, argument, &bytes)
            }
            TIOCSWINSZ => ioctl_winsize(self, user, argument),
            _ => Err(FsError::NotTty),
        }
    }

    /// 读取当前打开文件描述的访问模式与状态标志。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Acquire)
    }

    /// 更新打开文件描述的可变状态标志（如 `F_SETFL`）。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        crate::fs::file::set_status_flags(&self.flags, raw)
    }

    /// 原子设置或清除本打开文件实例的非阻塞标志（`O_NONBLOCK`）。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        crate::fs::file::set_nonblocking(&self.flags, enabled)
    }

    /// 终端字符设备不支持文件长度截断（`ftruncate`）。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 终端字符设备不支持 `fsync` / `fdatasync`。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// 终端字符设备为流式设备，不支持 `lseek` 定位。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 查询打开时对应设备节点的 `FileStat` 元数据。
    fn stat(&self) -> Result<FileStat, FsError> {
        crate::fs::file::stat_path(&self.path)
    }

    /// 终端字符设备非目录，不支持读取目录项。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// 终端字符设备非目录，不支持枚举目录项。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 将 `source` 中的用户数据按当前 `termios` 输出模式加工后写入终端输出队列，返回实际消费的源字节数。
    ///
    /// 执行步骤：
    /// 1. 校验可写权限、挂断状态以及 `TOSTOP` 后台写权限；
    /// 2. 获取 `writer` 睡眠锁（若处于非阻塞模式且锁被占用则立即返回 `WouldBlock`）；
    /// 3. 按 `WRITE_SEGMENT`（512 字节）分段从用户缓冲区拉取数据，在 `copy` + `state` 锁保护下经 `output_byte` 推入输出队列；
    /// 4. 每次提交后触发 [`schedule_transmit`] 启动异步硬件发送；若输出队列已满且尚未写入任何字节，则在保持持有 `writer` 锁的前提下睡眠等待输出队列腾出空间。
    fn write_from(
        &self,
        source: &mut WriteBuffer<'_>,
        caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        crate::fs::file::require_write(self.flags())?;
        let start = source.copied();
        if source.len() == start {
            return Ok(0);
        }
        self.check_write(caller)?;

        let interruptible = matches!(caller, FileIoContext::User(_));
        // Step 1: 尝试非阻塞获取 writer 串行锁；若失败且非 O_NONBLOCK，则在等待队列上睡眠直至锁释放。
        let mut writer = match endpoint().writer.try_lock() {
            Some(guard) => Some(WriterGuard { inner: Some(guard) }),
            None => {
                if self.flags() & O_NONBLOCK != 0 {
                    return Err(FsError::WouldBlock);
                }
                None
            }
        };
        let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
        // Step 2: 循环等待获取 writer 睡眠锁，每次唤醒后重新校验挂断状态与 TOSTOP 后台写权限。
        while writer.is_none() {
            if waiting.is_none() {
                waiting = Some(self.prepare_wait()?);
            }
            let (context, _) = waiting.as_ref().expect("tty writer wait without context");
            context.clear_triggered();
            self.check_write(caller)?;
            if let Some(guard) = endpoint().writer.try_lock() {
                writer = Some(WriterGuard { inner: Some(guard) });
                break;
            }
            let result = context.park_until(interruptible, None);
            self.check_write(caller)?;
            if result == WaitResult::Interrupted {
                return Err(FsError::Interrupted);
            }
        }

        let mut waiting = waiting;
        let _writer = writer.take().expect("tty writer was not acquired");
        let mut accepted = 0;
        let mut segment = [0u8; WRITE_SEGMENT];
        let mut pending_buf = [0u8; WRITE_SEGMENT];
        let mut pending_len = 0;
        let mut pending_off = 0;
        // Step 3: 分段从 WriteBuffer 拉取字节并送入行规程输出队列。
        loop {
            if let Err(error) = check_hangup(Some(self)) {
                return progress(accepted, error);
            }
            if pending_off == pending_len {
                let remaining = source.len() - source.copied();
                if remaining == 0 {
                    return Ok(accepted);
                }
                let count = remaining.min(segment.len());
                match source.pull(&mut segment[..count]) {
                    Ok(0) => return progress(accepted, FsError::BadAddress),
                    Ok(prepared) => {
                        pending_buf[..prepared].copy_from_slice(&segment[..prepared]);
                        pending_len = prepared;
                        pending_off = 0;
                    }
                    Err(error) => return progress(accepted, error),
                }
            }

            // Step 4: 在持有 writer 锁的前提下短暂获取 copy 与 state 锁提交输出字节；若处于 FLUSHO 状态则直接视为已消费。
            let consumed = {
                let _copy = endpoint().copy.lock();
                if let Err(error) = check_hangup(Some(self)) {
                    return progress(accepted, error);
                }
                let mut state = endpoint().state.lock();
                let termios = state.termios;
                let mut consumed = 0;
                for &byte in pending_buf[pending_off..pending_len].iter() {
                    if state.flusho() {
                        consumed += 1;
                        continue;
                    }
                    if !state.output_byte(termios, byte) {
                        break;
                    }
                    consumed += 1;
                }
                consumed
            };
            pending_off += consumed;
            accepted += consumed;
            schedule_transmit();
            endpoint()
                .write_head
                .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);

            if pending_off < pending_len {
                if self.flags() & O_NONBLOCK != 0 {
                    return progress(accepted, FsError::WouldBlock);
                }
                if 0 < accepted {
                    return Ok(accepted);
                }
                // Step 5: 当输出队列空间不足且尚未消费任何字节时，保持持有 writer 锁并睡眠等待输出队列腾出空间。
                if waiting.is_none() {
                    waiting = Some(self.prepare_wait()?);
                }
                let (context, _) = waiting.as_ref().expect("tty space wait without context");
                context.clear_triggered();
                let ready = {
                    let state = endpoint().state.lock();
                    state.output_can_accept(state.termios, pending_buf[pending_off])
                };
                if !ready {
                    let result = context.park_until(interruptible, None);
                    self.check_write(caller)?;
                    if result == WaitResult::Interrupted {
                        return Err(FsError::Interrupted);
                    }
                }
                continue;
            }
            pending_len = 0;
            pending_off = 0;
            if source.copied() == source.len() {
                return Ok(accepted);
            }
        }
    }

    /// 终端字符设备为流式设备，不支持带偏移量的定位写入（`pwrite`），恒返回 `FsError::IllegalSeek`。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
}

/// 处理带部分进度的 I/O 返回值：若已成功传输的字节数 `count > 0`（Partial Progress），优先返回已传输字节数；仅在零进度时返回错误 `error`。
fn progress(count: usize, error: FsError) -> Result<usize, FsError> {
    if count == 0 {
        Err(error)
    } else {
        Ok(count)
    }
}

/// 获取终端 `reader` 睡眠锁并返回 [`ReaderGuard`]。
///
/// - 若本打开实例在获取锁前或等待期间被挂断，返回 `Ok(None)`（调用方应按 EOF 返回 0）；
/// - 若存在其他读者持有 `reader` 锁，非阻塞模式下返回 `FsError::WouldBlock`，阻塞模式下在 `read_head` 上睡眠等待并在每次唤醒后重新检查挂断与后台进程组权限（`SIGTTIN`）。
fn acquire_reader(
    file: &TtyFile,
    caller: FileIoContext<'_>,
) -> Result<Option<ReaderGuard>, FsError> {
    if !file.check_read(caller)? {
        return Ok(None);
    }
    if let Some(guard) = endpoint().reader.try_lock() {
        return Ok(Some(ReaderGuard { inner: Some(guard) }));
    }
    if file.flags() & O_NONBLOCK != 0 {
        return Err(FsError::WouldBlock);
    }

    let (context, _table) = file.prepare_wait()?;
    loop {
        context.clear_triggered();
        if !file.check_read(caller)? {
            return Ok(None);
        }
        if let Some(guard) = endpoint().reader.try_lock() {
            return Ok(Some(ReaderGuard { inner: Some(guard) }));
        }
        let result = context.park_until(matches!(caller, FileIoContext::User(_)), None);
        if !file.check_read(caller)? {
            return Ok(None);
        }
        if result == WaitResult::Interrupted {
            return Err(FsError::Interrupted);
        }
    }
}

/// 为有序配置修改（`TCSETSW` / `TCSETSF`）获取 `writer` 睡眠锁并返回 [`WriterGuard`]。
///
/// 等待期间在每次唤醒后重新检查当前打开文件实例是否已被会话挂断。
fn acquire_writer(interruptible: bool, file: Option<&TtyFile>) -> Result<WriterGuard, FsError> {
    check_hangup(file)?;
    if let Some(guard) = endpoint().writer.try_lock() {
        return Ok(WriterGuard { inner: Some(guard) });
    }

    let context = PollContext::try_new()?;
    let sink: Arc<dyn PollNotify> = context.clone();
    let mut table = PollTable::new(sink, POLL_SOURCES)?;
    table.register(&endpoint().read_head)?;
    table.register(&endpoint().write_head)?;
    loop {
        context.clear_triggered();
        check_hangup(file)?;
        if let Some(guard) = endpoint().writer.try_lock() {
            return Ok(WriterGuard { inner: Some(guard) });
        }
        let result = context.park_until(interruptible, None);
        check_hangup(file)?;
        if result == WaitResult::Interrupted {
            return Err(FsError::Interrupted);
        }
    }
}

/// 执行规范模式（`ICANON`）下的终端读取。
///
/// 执行逻辑：
/// 1. 在持有 `copy` 睡眠锁期间，先短暂获取 `state` 自旋锁预览当前首条已提交记录（`peek_canonical`）；
/// 2. 若等待期间 `ICANON` 已被并发关闭，则释放 `copy` 锁并无缝切换到 [`noncanonical_read`]；
/// 3. 若已有已提交记录或 `VEOF` 标志，在释放 `state` 自旋锁但仍持有 `copy` 锁的状态下将数据推入用户 `ReadBuffer`，随后再次获取 `state` 锁仅消费实际拷贝成功的字节数（`consume_canonical`）；
/// 4. 若尚无已提交记录，释放 `copy` 锁并根据 `O_NONBLOCK` 标志返回 `WouldBlock` 或进入睡眠等待。
fn canonical_read(
    file: &TtyFile,
    target: &mut ReadBuffer<'_>,
    caller: FileIoContext<'_>,
) -> Result<usize, FsError> {
    let start = target.copied();
    let interruptible = matches!(caller, FileIoContext::User(_));
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
    let mut interrupted = false;
    let mut segment = [0u8; SEGMENT_BYTES];
    loop {
        if let Some((context, _)) = &waiting {
            context.clear_triggered();
        }
        // Step 1: 获取 copy 锁并校验挂断与后台读权限，再短暂加 state 锁预览已提交的规范行。
        let copy = endpoint().copy.lock();
        match file.check_read(caller) {
            Ok(true) => {}
            Ok(false) => return Ok(target.copied() - start),
            Err(error) => return progress(target.copied() - start, error),
        }
        let (peeked, eof, switched) = {
            let state = endpoint().state.lock();
            if state.termios.lflag & termios::ICANON == 0 {
                (0, false, true)
            } else {
                let count = (target.len() - target.copied()).min(segment.len());
                let (peeked, eof) = state.peek_canonical(&mut segment[..count]);
                (peeked, eof, false)
            }
        };
        if switched {
            drop(copy);
            let termios = endpoint().state.lock().termios;
            return noncanonical_read(file, target, caller, termios);
        }
        // Step 2: 若存在可交付字节或遇到 VEOF，向目标缓冲区拷贝并按实际成功字节数从输入队列消费。
        if 0 < peeked || eof {
            let before = target.copied();
            let result = target.push(&segment[..peeked]);
            let delivered = target.copied() - before;
            {
                let mut state = endpoint().state.lock();
                state.consume_canonical(delivered, eof && delivered == 0);
            }
            drop(copy);
            endpoint()
                .read_head
                .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
            endpoint()
                .write_head
                .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
            if let Err(error) = result {
                return progress(target.copied() - start, error);
            }
            if eof && target.copied() == start {
                return Ok(0);
            }
            return Ok(target.copied() - start);
        }

        // Step 3: 尚无完整行可读，释放 copy 锁后按非阻塞或可中断睡眠处理。
        drop(copy);
        if file.flags() & O_NONBLOCK != 0 {
            return Err(FsError::WouldBlock);
        }
        if interrupted {
            return Err(FsError::Interrupted);
        }
        if let Some((context, _)) = &waiting {
            interrupted = context.park_until(interruptible, None) == WaitResult::Interrupted;
        } else {
            waiting = Some(file.prepare_wait()?);
        }
    }
}

/// 非规范模式单次分段读取的内部结果状态。
enum RawDeliver {
    /// 本段成功交付的字节数，或用户缓冲区拷贝错误（调用方需结合此前已拷贝的前缀字节数统一处理）。
    Delivered(Result<usize, FsError>),
    /// 终端配置已并发切换回规范模式（`ICANON`），若当前尚无读取进度则调用方应转入 [`canonical_read`]。
    Switched,
    /// 本打开文件描述已被永久挂断，调用方应立即返回此前已成功交付的字节数。
    Hungup,
}

/// 非规范模式下执行单次分段预览、用户缓冲区写入与输入队列消费（全程在 `copy` 睡眠锁保护下完成）。
fn deliver_raw(
    file: &TtyFile,
    caller: FileIoContext<'_>,
    target: &mut ReadBuffer<'_>,
    segment: &mut [u8],
) -> RawDeliver {
    let copy = endpoint().copy.lock();
    match file.check_read(caller) {
        Ok(true) => {}
        Ok(false) => return RawDeliver::Hungup,
        Err(error) => return RawDeliver::Delivered(Err(error)),
    }
    let peeked = {
        let state = endpoint().state.lock();
        if state.termios.lflag & termios::ICANON != 0 {
            return RawDeliver::Switched;
        }
        let want = (target.len() - target.copied()).min(segment.len());
        state.peek_raw(&mut segment[..want])
    };
    if peeked == 0 {
        drop(copy);
        return RawDeliver::Delivered(Ok(0));
    }

    let before = target.copied();
    let result = target.push(&segment[..peeked]);
    let delivered = target.copied() - before;
    {
        let mut state = endpoint().state.lock();
        state.consume_raw(delivered);
    }
    drop(copy);
    match result {
        Ok(_) => RawDeliver::Delivered(Ok(delivered)),
        Err(error) => RawDeliver::Delivered(Err(error)),
    }
}

/// 执行非规范模式（`!ICANON`）下的终端读取，根据 `VMIN` 与 `VTIME` 的四种组合分发处理：
/// - `VMIN == 0, VTIME == 0`：立即返回当前可用字节（无字节时立即返回 0）；
/// - `VMIN > 0,  VTIME == 0`：阻塞等待直至累计读取到 `min(VMIN, request)` 个字节（[`noncanonical_min`]）；
/// - `VMIN == 0, VTIME > 0` ：从调用开始计时 `VTIME * 100ms`，一有字节到达立即返回，超时返回 0（[`noncanonical_timeout`]）；
/// - `VMIN > 0,  VTIME > 0` ：收到首个字节后启动字节间超时定时器 `VTIME * 100ms`，集齐 `min(VMIN, request)` 字节或字符间超时即返回（[`noncanonical_min_timeout`]）。
fn noncanonical_read(
    file: &TtyFile,
    target: &mut ReadBuffer<'_>,
    caller: FileIoContext<'_>,
    termios: Termios,
) -> Result<usize, FsError> {
    let start = target.copied();
    let request = target.len() - start;
    let min = usize::from(termios.cc[VMIN]);
    let time = termios.cc[VTIME];
    if min == 0 && time == 0 {
        let mut segment = [0u8; SEGMENT_BYTES];
        match deliver_raw(file, caller, target, &mut segment) {
            RawDeliver::Hungup => return Ok(target.copied() - start),
            RawDeliver::Delivered(Ok(count)) => return Ok(count),
            RawDeliver::Delivered(Err(error)) => return progress(target.copied() - start, error),
            RawDeliver::Switched => return canonical_read(file, target, caller),
        }
    }
    if min != 0 && time == 0 {
        return noncanonical_min(file, target, caller, min.min(request));
    }
    if min == 0 {
        return noncanonical_timeout(file, target, caller, time);
    }
    noncanonical_min_timeout(file, target, caller, min.min(request), time)
}

/// 非规范模式读取（`VMIN > 0 && VTIME == 0`）：阻塞等待直至至少读取到 `minimum`（即 `min(VMIN, 请求长度)`）个字节。
/// 若中途被信号中断或发生错误，只要已读取到部分字节（Partial Progress）就返回已读长度。
fn noncanonical_min(
    file: &TtyFile,
    target: &mut ReadBuffer<'_>,
    caller: FileIoContext<'_>,
    minimum: usize,
) -> Result<usize, FsError> {
    let start = target.copied();
    let interruptible = matches!(caller, FileIoContext::User(_));
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
    let mut interrupted = false;
    let mut segment = [0u8; SEGMENT_BYTES];
    loop {
        if let Some((context, _)) = &waiting {
            context.clear_triggered();
        }
        match deliver_raw(file, caller, target, &mut segment) {
            RawDeliver::Hungup => return Ok(target.copied() - start),
            RawDeliver::Switched => {
                if start < target.copied() {
                    return Ok(target.copied() - start);
                }
                return canonical_read(file, target, caller);
            }
            RawDeliver::Delivered(Ok(0)) => {}
            RawDeliver::Delivered(Ok(_)) => {
                if minimum <= target.copied() - start {
                    return Ok(target.copied() - start);
                }
                continue;
            }
            RawDeliver::Delivered(Err(error)) => {
                return progress(target.copied() - start, error);
            }
        }

        if file.flags() & O_NONBLOCK != 0 {
            return progress(target.copied() - start, FsError::WouldBlock);
        }
        if interrupted {
            return progress(target.copied() - start, FsError::Interrupted);
        }
        if let Some((context, _)) = &waiting {
            interrupted = context.park_until(interruptible, None) == WaitResult::Interrupted;
        } else {
            waiting = Some(file.prepare_wait()?);
        }
    }
}

/// 非规范模式读取（`VMIN == 0 && VTIME > 0`）：设置从本次 `read` 开始计算的绝对总截止时间（`VTIME * 100ms`）；
/// 一旦读到任意数量字节即返回，若超时仍无数据则返回 `Ok(0)`。
fn noncanonical_timeout(
    file: &TtyFile,
    target: &mut ReadBuffer<'_>,
    caller: FileIoContext<'_>,
    time: u8,
) -> Result<usize, FsError> {
    let start = target.copied();
    let interruptible = matches!(caller, FileIoContext::User(_));
    let deadline = time::now()
        .checked_add(Duration::from_millis(u64::from(time) * VTIME_MILLIS))
        .ok_or(FsError::InvalidOperation)?;
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
    let mut segment = [0u8; SEGMENT_BYTES];
    loop {
        if let Some((context, _)) = &waiting {
            context.clear_triggered();
        }
        match deliver_raw(file, caller, target, &mut segment) {
            RawDeliver::Hungup => return Ok(target.copied() - start),
            RawDeliver::Switched => {
                if start < target.copied() {
                    return Ok(target.copied() - start);
                }
                return canonical_read(file, target, caller);
            }
            RawDeliver::Delivered(Ok(0)) => {}
            RawDeliver::Delivered(Ok(_)) => return Ok(target.copied() - start),
            RawDeliver::Delivered(Err(error)) => {
                return progress(target.copied() - start, error);
            }
        }

        if file.flags() & O_NONBLOCK != 0 {
            return progress(target.copied() - start, FsError::WouldBlock);
        }
        if deadline <= time::now() {
            return Ok(target.copied() - start);
        }
        if let Some((context, _)) = &waiting {
            if context.park_until(interruptible, Some(deadline)) == WaitResult::Interrupted {
                match file.check_read(caller) {
                    Ok(true) => {}
                    Ok(false) => return Ok(target.copied() - start),
                    Err(error) => return progress(target.copied() - start, error),
                }
                return progress(target.copied() - start, FsError::Interrupted);
            }
        } else {
            waiting = Some(file.prepare_wait()?);
        }
    }
}

/// 非规范模式读取（`VMIN > 0 && VTIME > 0`）：首个字节到达前无限期等待；
/// 每当收到新字节时将字符间超时截止时间重置为 `now + VTIME * 100ms`，直至累计读满 `minimum` 字节或字符间超时到期。
fn noncanonical_min_timeout(
    file: &TtyFile,
    target: &mut ReadBuffer<'_>,
    caller: FileIoContext<'_>,
    minimum: usize,
    time: u8,
) -> Result<usize, FsError> {
    let start = target.copied();
    let interruptible = matches!(caller, FileIoContext::User(_));
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
    let mut interrupted = false;
    let mut deadline: Option<MonotonicInstant> = None;
    let mut segment = [0u8; SEGMENT_BYTES];
    loop {
        if let Some((context, _)) = &waiting {
            context.clear_triggered();
        }
        match deliver_raw(file, caller, target, &mut segment) {
            RawDeliver::Hungup => return Ok(target.copied() - start),
            RawDeliver::Switched => {
                if start < target.copied() {
                    return Ok(target.copied() - start);
                }
                return canonical_read(file, target, caller);
            }
            RawDeliver::Delivered(Ok(0)) => {}
            RawDeliver::Delivered(Ok(_)) => {
                deadline = Some(
                    time::now()
                        .checked_add(Duration::from_millis(u64::from(time) * VTIME_MILLIS))
                        .ok_or(FsError::InvalidOperation)?,
                );
                if minimum <= target.copied() - start {
                    return Ok(target.copied() - start);
                }
                continue;
            }
            RawDeliver::Delivered(Err(error)) => {
                return progress(target.copied() - start, error);
            }
        }

        if file.flags() & O_NONBLOCK != 0 {
            return progress(target.copied() - start, FsError::WouldBlock);
        }
        if interrupted {
            return progress(target.copied() - start, FsError::Interrupted);
        }
        if 0 < target.copied() - start {
            if let Some(limit) = deadline {
                if limit <= time::now() {
                    return Ok(target.copied() - start);
                }
            }
        }
        if let Some((context, _)) = &waiting {
            let result = context.park_until(interruptible, deadline);
            match file.check_read(caller) {
                Ok(true) => {}
                Ok(false) => return Ok(target.copied() - start),
                Err(error) => return progress(target.copied() - start, error),
            }
            if result == WaitResult::Interrupted {
                interrupted = true;
            } else if 0 < target.copied() - start {
                if let Some(limit) = deadline {
                    if limit <= time::now() {
                        return Ok(target.copied() - start);
                    }
                }
            }
        } else {
            waiting = Some(file.prepare_wait()?);
        }
    }
}

/// 处理 `TCGETS`：将当前生效的 36 字节 `termios` 配置完整拷贝到用户空间指针 `argument`。
fn ioctl_get(user: &UserThread, argument: usize) -> Result<usize, FsError> {
    let mm = user.mm().ok_or(FsError::BadAddress)?;
    let bytes = endpoint().state.lock().termios.to_bytes();
    let progress = mm.copyout(argument, &bytes);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(FsError::BadAddress);
    }
    Ok(0)
}

/// 处理 `TCSETS` / `TCSETSW` / `TCSETSF`：从用户空间拷贝 36 字节 `termios` 并按指定动作（`TCSANOW` / `TCSADRAIN` / `TCSAFLUSH`）校验后提交。
fn ioctl_set(
    file: &TtyFile,
    user: &UserThread,
    argument: usize,
    action: u32,
    caller: FileIoContext<'_>,
) -> Result<usize, FsError> {
    let mm = user.mm().ok_or(FsError::BadAddress)?;
    let mut bytes = [0u8; termios::TERMIOS_BYTES];
    let progress = mm.copyin(&mut bytes, argument);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(FsError::BadAddress);
    }

    let termios = Termios::from_bytes(bytes)?;
    apply_termios(termios, action, caller, Some(file))?;
    Ok(0)
}

/// 获取当前终端 `Termios` 配置的一致性快照（供内核测试读取）。
#[cfg(feature = "ktest")]
pub fn termios() -> Termios {
    endpoint().state.lock().termios
}

/// 在检查后台访问权限（`SIGTTOU`）后按 `action` 应用新的 `Termios` 配置（供内核测试调用）。
#[cfg(feature = "ktest")]
pub fn set_termios(
    termios: Termios,
    action: u32,
    caller: FileIoContext<'_>,
) -> Result<(), FsError> {
    check_access(caller, SIGTTOU)?;
    apply_termios(termios, action, caller, None)
}

/// 根据 `action`（`TCSANOW` / `TCSADRAIN` / `TCSAFLUSH`）协调输出队列排空与 `writer` 锁获取，并提交新的 `Termios` 配置。
///
/// - `TCSANOW`：无需等待 `writer` 锁或输出排空，直接获取 `copy` 锁提交；
/// - `TCSADRAIN` / `TCSAFLUSH`：先调用 [`drain_output`] 等待软件队列与 UART 发送移位寄存器排空，再获取 `writer` 与 `copy` 锁复查是否真正排空后提交配置。
fn apply_termios(
    termios: Termios,
    action: u32,
    caller: FileIoContext<'_>,
    file: Option<&TtyFile>,
) -> Result<(), FsError> {
    if TCSAFLUSH < action {
        return Err(FsError::InvalidOperation);
    }
    termios.validate()?;
    if action == TCSANOW {
        return commit_termios(termios, action, None, file);
    }

    drain_output(caller, file)?;
    loop {
        let writer = acquire_writer(matches!(caller, FileIoContext::User(_)), file)?;
        let copy = endpoint().copy.lock();
        let pending = {
            let state = endpoint().state.lock();
            !state.output_empty()
        } || !crate::console::transmitter_empty();
        if pending {
            drop(copy);
            drop(writer);
            drain_output(caller, file)?;
            continue;
        }
        let result = commit_termios(termios, action, Some(copy), file);
        drop(writer);
        return result;
    }
}

/// 在持有 `copy` 睡眠锁的前提下将新的 `Termios` 应用到串口硬件与 `N_TTY` 行规程状态。
///
/// 执行步骤：
/// 1. 短暂获取 `state` 自旋锁计算串口寄存器目标配置（`SerialConfig`）及 DTR/RTS 线状态（`B0` 挂断时拉低，从 `B0` 恢复到非零波特率且有活跃打开时拉高）；
/// 2. 释放 `state` 锁后调用 `console::configure_serial` 配置底层 UART 寄存器并回读生效的分频系数；
/// 3. 再次获取 `state` 锁更新 `ICANON` 边界、`termios`、软件流控状态、DTR/RTS、`TCSAFLUSH` 输入清空与 `FLUSHO` 输出清空；
/// 4. 释放 `copy` 锁后唤醒读写等待队列并触发发送调度。
fn commit_termios(
    termios: Termios,
    action: u32,
    held: Option<crate::sync::SleepMutexGuard<'static, ()>>,
    file: Option<&TtyFile>,
) -> Result<(), FsError> {
    let copy = held.unwrap_or_else(|| endpoint().copy.lock());
    check_hangup(file)?;
    // Step 1: 在 state 自旋锁内根据新旧波特率与 DTR/RTS 状态生成硬件寄存器配置目标。
    let (config, canonical, hung_up, restore, old_ixon) = {
        let state = endpoint().state.lock();
        let canonical = termios.lflag & termios::ICANON != 0;
        let current_divisor = state.divisor;
        let old_baud = termios::baud_rate(state.termios.output_baud());
        let new_baud = termios::baud_rate(termios.output_baud());
        let hung_up = new_baud == Some(0);
        let restore = old_baud == Some(0) && !hung_up && 0 < state.opens;
        let old_ixon = state.termios.iflag & termios::IXON != 0;
        let dtr = if hung_up {
            false
        } else if restore {
            true
        } else {
            state.dtr
        };
        let rts = if hung_up {
            false
        } else if restore {
            true
        } else {
            state.rts
        };
        let config = termios::serial_config(termios, current_divisor, dtr, rts)
            .ok_or(FsError::InvalidOperation)?;
        (config, canonical, hung_up, restore, old_ixon)
    };

    // Step 2: 在 state 锁外更新 UART 硬件寄存器，随后将新配置与回读分频写入行规程状态。
    let snapshot = crate::console::configure_serial(config);
    {
        let mut state = endpoint().state.lock();
        if (state.termios.lflag & termios::ICANON != 0) != canonical {
            state.set_canonical(canonical);
        }
        state.termios = termios;
        if old_ixon && termios.iflag & termios::IXON == 0 && !state.tco_stopped {
            state.stopped = false;
        }
        if hung_up {
            state.dtr = false;
            state.rts = false;
        } else if restore {
            state.dtr = true;
            state.rts = true;
        }
        if action == TCSAFLUSH {
            flush_input_locked(&mut state);
        }
        if termios.lflag & termios::FLUSHO != 0 {
            state.flush_output();
        }
        state.divisor = snapshot.divisor;
    }
    drop(copy);

    // Step 3: 锁外通知读写等待者并调度发送队列。
    endpoint()
        .read_head
        .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
    endpoint()
        .write_head
        .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
    schedule_transmit();
    Ok(())
}

/// 按选择器（`TCIFLUSH` / `TCOFLUSH` / `TCIOFLUSH`）清空终端输入或输出队列（供内核测试调用）。
#[cfg(feature = "ktest")]
pub fn flush(selector: u32) -> Result<(), FsError> {
    endpoint_flush(selector, None)
}

/// 清空行规程输入队列并同时丢弃底层串口软件接收环中尚未拉取的采样（调用方需同时持有 `copy` 与 `state` 锁）。
fn flush_input_locked(state: &mut ntty::NttyState) {
    state.flush_input();
    crate::console::discard_received();
}

/// 执行 `TCFLSH` 队列刷新操作：在 `copy` 锁保护下复查打开实例挂断状态，清空指定方向的缓冲区并唤醒读写等待队列。
fn endpoint_flush(selector: u32, file: Option<&TtyFile>) -> Result<(), FsError> {
    {
        let _copy = endpoint().copy.lock();
        check_hangup(file)?;
        let mut state = endpoint().state.lock();
        match selector {
            TCIFLUSH => flush_input_locked(&mut state),
            TCOFLUSH => state.flush_output(),
            TCIOFLUSH => {
                flush_input_locked(&mut state);
                state.flush_output();
            }
            _ => return Err(FsError::InvalidOperation),
        }
    }

    endpoint()
        .read_head
        .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
    endpoint()
        .write_head
        .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
    Ok(())
}

/// 执行 `TCXONC` 软件流控动作（`TCOOFF` 暂停本地输出、`TCOON` 恢复本地输出、`TCIOFF`/`TCION` 向远端发送高优先级 `VSTOP`/`VSTART` 字符）。
fn endpoint_flow(action: u32, file: Option<&TtyFile>) -> Result<usize, FsError> {
    match action {
        TCOOFF => {
            let _copy = endpoint().copy.lock();
            check_hangup(file)?;
            let mut state = endpoint().state.lock();
            state.tco_stopped = true;
            state.stopped = true;
            Ok(0)
        }
        TCOON => {
            {
                let _copy = endpoint().copy.lock();
                check_hangup(file)?;
                let mut state = endpoint().state.lock();
                state.tco_stopped = false;
                state.stopped = false;
            }
            schedule_transmit();
            endpoint()
                .write_head
                .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
            Ok(0)
        }
        TCIOFF | TCION => {
            let byte = {
                let state = endpoint().state.lock();
                if action == TCIOFF {
                    state.termios.cc[termios::VSTOP]
                } else {
                    state.termios.cc[termios::VSTART]
                }
            };
            if byte == 0 {
                return Err(FsError::InvalidOperation);
            }
            {
                let _copy = endpoint().copy.lock();
                check_hangup(file)?;
                let mut state = endpoint().state.lock();
                state.xchar = Some(byte);
            }
            schedule_transmit();
            Ok(0)
        }
        _ => Err(FsError::InvalidOperation),
    }
}

/// 执行内核态流控动作（供内核集成测试调用）。
#[cfg(feature = "ktest")]
pub fn flow(action: u32) -> Result<(), FsError> {
    endpoint_flow(action, None).map(|_| ())
}

/// 在检查后台写权限（`SIGTTOU`）后等待所有终端输出排空（供内核集成测试调用）。
#[cfg(feature = "ktest")]
pub fn drain(caller: FileIoContext<'_>) -> Result<(), FsError> {
    check_access(caller, SIGTTOU)?;
    drain_output(caller, None)
}

/// 等待行规程软件输出队列与底层 UART 发送移位寄存器（`TEMT`）全部排空。
///
/// 执行分两个阶段：
/// 1. **等待软件队列排空**：在端点的 `read_head` 与 `write_head` 上阻塞等待，直到 `state.output_empty()` 为真；
/// 2. **等待硬件移位寄存器排空**：软件队列清空后，若 `console::transmitter_empty()` 尚未就绪，则按当前分频系数与帧格式计算单字符传输时间（[`termios::character_nanos`]），按单字符时长定时睡眠复查直至硬件完全空闲。
fn drain_output(caller: FileIoContext<'_>, file: Option<&TtyFile>) -> Result<(), FsError> {
    let interruptible = matches!(caller, FileIoContext::User(_));
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
    // Step 1: 阻塞等待软件输出队列、回显队列、高优先级字符与挂起重印全部排空。
    loop {
        if let Some((context, _)) = &waiting {
            context.clear_triggered();
        }
        check_hangup(file)?;
        let empty = endpoint().state.lock().output_empty();
        if empty {
            break;
        }
        if let Some((context, _)) = &waiting {
            if context.park_until(interruptible, None) == WaitResult::Interrupted {
                check_hangup(file)?;
                return Err(FsError::Interrupted);
            }
        } else {
            let context = PollContext::try_new()?;
            let sink: Arc<dyn PollNotify> = context.clone();
            let mut table = PollTable::new(sink, POLL_SOURCES)?;
            table.register(&endpoint().read_head)?;
            table.register(&endpoint().write_head)?;
            waiting = Some((context, table));
        }
    }

    // Step 2: 软件队列已空，按单字符传输周期睡眠等待硬件发送移位寄存器（TEMT）彻底排空。
    loop {
        check_hangup(file)?;
        if crate::console::transmitter_empty() {
            return Ok(());
        }
        let (divisor, cflag) = {
            let state = endpoint().state.lock();
            (state.divisor, state.termios.cflag)
        };
        let nanos = termios::character_nanos(divisor, crate::console::baud_base(), cflag)
            .ok_or(FsError::InvalidOperation)?;
        let deadline = time::now()
            .checked_add(Duration::from_nanos(nanos))
            .ok_or(FsError::InvalidOperation)?;
        if interruptible {
            if crate::timer::sleep_until_interruptible(deadline) == WaitResult::Interrupted {
                check_hangup(file)?;
                return Err(FsError::Interrupted);
            }
        } else {
            crate::timer::sleep_until(deadline);
        }
    }
}

/// 检查用户态调用者是否具有访问控制终端的前台权限；若属于后台进程组，则由 `TaskGraph::check_terminal_access` 发送 `signum`（`SIGTTIN` 或 `SIGTTOU`）或返回 `EIO`。
fn check_access(caller: FileIoContext<'_>, signum: u8) -> Result<(), FsError> {
    if let FileIoContext::User(user) = caller {
        let identity = user.identity().ok_or(FsError::NotTty)?;
        identity
            .graph()
            .check_terminal_access(&identity, endpoint(), signum)?;
    }
    Ok(())
}

/// 获取当前控制终端绑定的任务图弱引用与前台进程组 PID 快照。
fn foreground_snapshot() -> Option<(Weak<TaskGraph>, Arc<Pid>)> {
    endpoint()
        .control
        .lock()
        .as_ref()
        .map(|control| (control.graph.clone(), control.foreground.clone()))
}

/// 向控制终端的前台进程组发送信号 `signum`（若前台进程组内已无存活进程则静默忽略 `SignalError::NoProcess`）。
fn send_foreground(graph: &Weak<TaskGraph>, foreground: &Arc<Pid>, signum: u8) {
    if let Some(graph) = graph.upgrade() {
        match graph.send_process_group(foreground, signum) {
            Ok(()) | Err(SignalError::NoProcess) => {}
            Err(error) => panic!("terminal foreground signal failed: {:?}", error),
        }
    }
}

/// 在解除控制终端关联后，向原控制终端的前台进程组依次发送 `SIGHUP` 与 `SIGCONT` 信号。
fn notify_disassociated(control: TerminalControl) {
    send_foreground(&control.graph, &control.foreground, SIGHUP);
    send_foreground(&control.graph, &control.foreground, SIGCONT);
}

/// 当控制会话的最后一个会话首进程（Session Leader）退出时，挂断并解绑该会话的控制终端。
///
/// 执行步骤：
/// 1. 在 `copy` 锁保护下调用 `TaskGraph::release_tty_session` 解除会话与终端端点的控制关系；
/// 2. 清空行规程输入/输出队列，拉低串口硬件 DTR/RTS 线、关闭接收器并丢弃底层接收缓冲；
/// 3. 向 `hangup_head` 广播 `POLLHUP`，将当时所有活跃的非 `/dev/console` 打开实例原子标记为永久挂断并扣减其 `opens` 计数；
/// 4. 释放 `copy` 锁后唤醒所有读写等待者、向原前台进程组发送 `SIGHUP` + `SIGCONT`，并等待挂断前已排队的异步工作任务执行完毕。
pub(crate) fn hangup_session(graph: &TaskGraph, session: &Arc<Pid>) {
    let copy = endpoint().copy.lock();
    let Some(control) = graph.release_tty_session(endpoint(), session) else {
        return;
    };

    let config = {
        let mut state = endpoint().state.lock();
        state.flush_input();
        state.flush_output();
        state.stopped = false;
        state.tco_stopped = false;
        state.dtr = false;
        state.rts = false;
        crate::console::SerialConfig {
            divisor: state.divisor,
            line_control: termios::line_control(state.termios.cflag),
            receiver: false,
            dtr: false,
            rts: false,
        }
    };
    crate::console::configure_serial(config);
    crate::console::discard_received();

    endpoint().hangup_head.notify(PollMask::POLLHUP);
    let boundary = service().work.as_ref().capture_boundary();
    drop(copy);

    endpoint()
        .read_head
        .notify(PollMask::POLLIN | PollMask::POLLHUP);
    endpoint()
        .write_head
        .notify(PollMask::POLLOUT | PollMask::POLLHUP);
    notify_disassociated(control);
    service().work.as_ref().flush_boundary(boundary);
}

/// 从用户线程地址空间将 `argument` 指向的 `N` 字节参数完整拷贝到内核数组；若未完整拷贝则返回 `FsError::BadAddress`（`EFAULT`）。
fn copy_ioctl_in<const N: usize>(user: &UserThread, argument: usize) -> Result<[u8; N], FsError> {
    let mm = user.mm().ok_or(FsError::BadAddress)?;
    let mut bytes = [0u8; N];
    let progress = mm.copyin(&mut bytes, argument);
    if progress.error.is_some() || progress.copied != N {
        return Err(FsError::BadAddress);
    }
    Ok(bytes)
}

/// 将内核字节切片 `bytes` 完整拷贝到用户线程地址空间的 `argument` 指针处；若未完整拷贝则返回 `FsError::BadAddress`（`EFAULT`）。
fn copy_ioctl_out(user: &UserThread, argument: usize, bytes: &[u8]) -> Result<usize, FsError> {
    let mm = user.mm().ok_or(FsError::BadAddress)?;
    let progress = mm.copyout(argument, bytes);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(FsError::BadAddress);
    }
    Ok(0)
}

/// 处理控制终端与进程组相关的 `ioctl` 命令（`TIOCSCTTY`、`TIOCNOTTY`、`TIOCSPGRP`、`TIOCGPGRP`、`TIOCGSID`）。
///
/// 严格遵守 `copy -> TaskGraph::relations -> Endpoint::control` 锁获取顺序，并在释放 `copy` 锁后再执行用户空间写出或发送信号。
fn ioctl_control(
    file: &TtyFile,
    user: &UserThread,
    command: u32,
    argument: usize,
) -> Result<usize, FsError> {
    let number = if command == TIOCSPGRP {
        Some(termios::decode_pid(copy_ioctl_in(user, argument)?)?)
    } else {
        None
    };

    let identity = user.identity().ok_or(FsError::NotTty)?;
    let copy = endpoint().copy.lock();
    if file.hungup() {
        return Err(if command == TIOCSPGRP {
            FsError::NotTty
        } else {
            FsError::DeviceIo
        });
    }

    match command {
        TIOCSCTTY => {
            crate::fs::file::require_read(file.flags()).map_err(|_| FsError::NotPermitted)?;
            identity.graph().acquire_tty(&identity, endpoint(), false)?;
        }
        TIOCNOTTY => {
            let control = identity.graph().disassociate_tty(&identity, endpoint())?;
            drop(copy);
            if let Some(control) = control {
                notify_disassociated(control);
            }
            return Ok(0);
        }
        TIOCSPGRP => {
            identity.graph().set_terminal_foreground(
                &identity,
                endpoint(),
                number.expect("pgrp not copied"),
            )?;
        }
        TIOCGPGRP | TIOCGSID => {
            let (session, foreground) = identity.graph().terminal_ids(&identity, endpoint())?;
            let number = if command == TIOCGSID {
                session.number()
            } else {
                foreground.number()
            };
            drop(copy);
            return copy_ioctl_out(user, argument, &termios::encode_pid(number));
        }
        _ => unreachable!("invalid terminal control command"),
    }
    Ok(0)
}

/// 处理 `TIOCSWINSZ`：从用户空间拷贝 8 字节窗口尺寸并在 `copy` 锁下更新 `winsize`；若尺寸发生实际变化，则在释放锁后向前台进程组发送 `SIGWINCH`。
fn ioctl_winsize(file: &TtyFile, user: &UserThread, argument: usize) -> Result<usize, FsError> {
    let size = termios::Winsize::from_bytes(copy_ioctl_in(user, argument)?);
    let copy = endpoint().copy.lock();
    if file.hungup() {
        return Err(FsError::DeviceIo);
    }
    let mut current = endpoint().winsize.lock();
    if *current == size {
        return Ok(0);
    }
    *current = size;
    drop(current);
    let foreground = foreground_snapshot();
    drop(copy);

    if let Some((graph, foreground)) = foreground {
        send_foreground(&graph, &foreground, SIGWINCH);
    }
    Ok(0)
}

/// 检查可选的 `TtyFile` 实例是否已被会话挂断；若已挂断则返回 `FsError::DeviceIo`（`EIO`）。
fn check_hangup(file: Option<&TtyFile>) -> Result<(), FsError> {
    if file.is_some_and(TtyFile::hungup) {
        return Err(FsError::DeviceIo);
    }
    Ok(())
}
