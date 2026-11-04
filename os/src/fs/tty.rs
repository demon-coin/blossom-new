//! 唯一 UART 的常驻行规程端点；输入、回显与输出各有固定有界队列。

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

/// TCGETS 命令。
const TCGETS: u32 = 0x5401;
/// TCSETS 命令。
const TCSETS: u32 = 0x5402;
/// TCSETSW 命令。
const TCSETSW: u32 = 0x5403;
/// TCSETSF 命令。
const TCSETSF: u32 = 0x5404;
/// TCSBRK 命令。
const TCSBRK: u32 = 0x5409;
/// TCXONC 命令。
const TCXONC: u32 = 0x540a;
/// TCFLSH 命令。
const TCFLSH: u32 = 0x540b;
/// 控制终端与共同窗口 ABI 命令。
const TIOCSCTTY: u32 = 0x540e;
const TIOCGPGRP: u32 = 0x540f;
const TIOCSPGRP: u32 = 0x5410;
const TIOCGWINSZ: u32 = 0x5413;
const TIOCSWINSZ: u32 = 0x5414;
const TIOCNOTTY: u32 = 0x5422;
const TIOCGSID: u32 = 0x5429;
/// 工作每轮最多消费的软件采样数。
const WORK_RX_BUDGET: usize = 64;
/// 写出片段固定 512 字节。
const WRITE_SEGMENT: usize = 512;
const POLL_SOURCES: usize = 2;
const VTIME_MILLIS: u64 = 100;

/// 常驻端点。
pub(crate) struct Endpoint {
    /// 行规程短锁状态，指向永久静态对象。
    state: &'static SpinLock<NttyState>,
    /// 一次逻辑 read 的串行锁。
    reader: SleepMutex<()>,
    /// 用户输出与有序 setter 的串行锁。
    writer: SleepMutex<()>,
    /// 短复制与输入 flush 串行锁。
    copy: SleepMutex<()>,
    /// 读持续源。
    read_head: Arc<PollWaitHead>,
    /// 写持续源。
    write_head: Arc<PollWaitHead>,
    /// 控制关系只保存稳定身份与原图弱引用。
    control: SpinLock<Option<TerminalControl>>,
    /// 每次打开永久挂断的唯一持续通知源。
    hangup_head: Arc<PollWaitHead>,
    /// 窗口四字段由 copy 串行整体提交。
    winsize: SpinLock<termios::Winsize>,
}

impl Endpoint {
    /// 图关系事务按 Graph→relations→control 顺序读取与移交控制拥有。
    pub(crate) fn control(&self) -> &SpinLock<Option<TerminalControl>> {
        &self.control
    }
}

/// 挂断只允许一次永久转换；源回调同时退休这一份 active opens。
struct HangupState {
    hungup: AtomicBool,
}

impl PollNotify for HangupState {
    /// 仅在已发表 open 的源通知中调用，不取 Graph 或任何睡眠锁。
    fn notify(&self, mask: PollMask) {
        if mask.contains(PollMask::POLLHUP) && !self.hungup.swap(true, Ordering::AcqRel) {
            let mut state = endpoint().state.lock();
            assert!(0 < state.opens, "tty hangup open count underflow");
            state.opens -= 1;
        }
    }
}

/// 静态服务。
struct Service {
    /// 常驻端点。
    endpoint: Endpoint,
    /// TTY 工作域。
    _queue: WorkQueue,
    /// TTY 固定工作。
    work: Pin<Arc<Work>>,
}

/// 唯一行规程状态的永久静态对象。
static STATE: SpinLock<NttyState> = SpinLock::new(NttyState::new());

static SERVICE: Once<Service> = Once::new();

/// 返回已发表的常驻服务。
fn service() -> &'static Service {
    SERVICE.get().expect("tty service is not initialized")
}

/// 返回常驻端点。
pub(crate) fn endpoint() -> &'static Endpoint {
    &service().endpoint
}

/// 一次逻辑 read 的串行 guard，析构先释锁再通知读源。
struct ReaderGuard {
    /// 底层串行锁 guard。
    inner: Option<crate::sync::SleepMutexGuard<'static, ()>>,
}

impl Drop for ReaderGuard {
    /// 先释放串行锁，再通知读源唤醒竞争者。
    fn drop(&mut self) {
        if let Some(guard) = self.inner.take() {
            drop(guard);
            endpoint()
                .read_head
                .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
        }
    }
}

/// 用户输出串行 guard，析构先释锁再通知写源。
struct WriterGuard {
    /// 底层串行锁 guard。
    inner: Option<crate::sync::SleepMutexGuard<'static, ()>>,
}

impl Drop for WriterGuard {
    /// 先释放串行锁，再通知写源唤醒竞争者。
    fn drop(&mut self) {
        if let Some(guard) = self.inner.take() {
            drop(guard);
            endpoint()
                .write_head
                .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
        }
    }
}

/// 在原工作队列初始化后准备常驻端点、域与 CPU0 工作。
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

/// 接收中断在端口锁外提示行规程消费软件环。
pub fn schedule_receive() {
    if let Some(service) = SERVICE.get() {
        let _ = service.work.as_ref().queue();
    }
}

/// 发送就绪或成功写入在端口锁外提示行规程复查实际发送队列。
pub fn schedule_transmit() {
    let Some(service) = SERVICE.get() else {
        return;
    };
    if service.endpoint.state.lock().output_empty() {
        return;
    }
    let _ = service.work.as_ref().queue();
}

/// TTY 工作回调：批处理 RX、展开回显，再非阻塞泵实际 TX。
fn run_work(_work: Pin<&Work>) {
    let service = service();
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
        let mut state = service.endpoint.state.lock();
        if 0 < count {
            progressed = true;
        }
        if state.opens == 0 {
            // 无打开实例时丢弃新采样，已接受回显仍继续展开。
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
    if let Some((graph, foreground)) = foreground {
        for &signum in &signals[..signal_count] {
            send_foreground(&graph, &foreground, signum);
        }
    }
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
    // 只在有真实进度且可继续时重提；STOP/背压依 START RX/THRE 通知。
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

/// 非阻塞泵实际 TX，不等用户 read、不等 Ctrl-Q、不取 writer 锁，返回是否真实发送。
/// 锁序：全程持 copy，短取 state peek 后释放 state，仍持 copy 调 try_write
/// （它只取有限 PORT，锁外 schedule_transmit 取 state 不反取 copy），成功后
/// 短取 state 消费队首；flush/settings 同样持 copy 才可移除队首。
/// 端口满时 arm 真实 THRE 中断，已就绪立即重提，不定期轮询。
fn pump_transmit(service: &'static Service, force: bool) -> bool {
    let _copy = service.endpoint.copy.lock();
    let mut pumped = false;
    loop {
        enum Peek {
            /// 优先字符。
            Xchar(u8),
            /// 输出队首字节。
            Output(u8),
            /// 丢弃中或流控中，无可发送字节。
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

/// 阶段边界等待真实执行票尾部，常规 read/poll 不得借它泵硬件。
#[cfg(feature = "ktest")]
pub fn flush_work() {
    service().work.as_ref().flush();
}

/// TTY 打开实例，各路径共享同端点。
pub struct TtyFile {
    /// 真实设备路径。
    path: Path,
    /// OFD 访问模式与状态。
    flags: AtomicU32,
    /// 反向 watch 头。
    watches: Arc<WatchHead>,
    /// 是否已计入端点打开数。
    opened: bool,
    /// console 不参加永久挂断，也不受普通 TOSTOP 输出限制。
    console: bool,
    /// 同一 OFD 的永久状态与同步源登记，dup/fork 不增加责任。
    hangup: Option<(Arc<HangupState>, PollRegistration)>,
}

impl TtyFile {
    /// 已挂断的打开不会因端点重开而恢复。
    fn hungup(&self) -> bool {
        self.hangup
            .as_ref()
            .is_some_and(|(state, _)| state.hungup.load(Ordering::Acquire))
    }

    /// 无复制进度的 read 边界先核永久状态，再核真实后台访问。
    fn check_read(&self, caller: FileIoContext<'_>) -> Result<bool, FsError> {
        if self.hungup() {
            return Ok(false);
        }
        check_access(caller, SIGTTIN)?;
        Ok(true)
    }

    /// 只对非 console 的 TOSTOP 普通输出检查后台访问。
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
    /// 确需阻塞时建立有限源登记，之后必须重新扫描业务状态。
    fn prepare_wait(&self) -> Result<(Arc<PollContext>, PollTable), FsError> {
        let context = PollContext::try_new()?;
        let sink: Arc<dyn PollNotify> = context.clone();
        let mut table = PollTable::new(sink, POLL_SOURCES)?;
        self.poll(Some(&mut table))?;
        Ok((context, table))
    }
}

impl Drop for TtyFile {
    /// 先同步撤 watch 与挂断回调，再归还仍活跃的 opened 责任。
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
    /// 首轮登记读写两个真实源，再取独立的 read/MIN 与写谓词。
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

/// 在 copy 下解析三个真实名字，所有可失败准备完成后才发表 active open。
pub fn open(
    path: Path,
    flags: u32,
    caller: FileIoContext<'_>,
    special: MemorySpecial,
) -> Result<FileRef, FsError> {
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
    /// 按规范/非规范与 MIN/TIME 交付输入，只消费复制成功的正前缀。
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

    /// TTY 没有可定位位置。
    fn read_at_into(
        &self,
        _offset: u64,
        _target: &mut ReadBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// 返回真实设备路径。
    fn path(&self) -> Option<&Path> {
        Some(&self.path)
    }

    /// TTY 没有文件映射能力。
    fn mapping_inode(&self) -> Option<&InodeRef> {
        None
    }

    /// 提供读写两个真实持续源。
    fn pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    /// 反向登记属于此打开实例。
    fn watch_head(&self) -> Option<&Arc<WatchHead>> {
        Some(&self.watches)
    }

    /// 只对 User 上下文复制指针命令，Kernel 裸指针无可证明存储契约。
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

    /// 返回 OFD 访问模式与状态。
    fn flags(&self) -> u32 {
        self.flags.load(Ordering::Acquire)
    }

    /// 只改变支持的可变状态位。
    fn set_status_flags(&self, raw: u32) -> Result<(), FsError> {
        crate::fs::file::set_status_flags(&self.flags, raw)
    }

    /// 只原子修改 TTY OFD 的非阻塞位，不改变终端状态和追加位。
    fn set_nonblocking(&self, enabled: bool) -> Result<(), FsError> {
        crate::fs::file::set_nonblocking(&self.flags, enabled)
    }

    /// TTY 不能截断。
    fn truncate(&self, _new_size: u64) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// TTY 没有 fsync 语义。
    fn sync_range(&self, _range: Range<u64>, _data_only: bool) -> Result<(), FsError> {
        Err(FsError::InvalidOperation)
    }

    /// TTY 没有共享字节位置。
    fn seek(&self, _from: SeekFrom) -> Result<u64, FsError> {
        Err(FsError::IllegalSeek)
    }

    /// stat 来自真实路径。
    fn stat(&self) -> Result<FileStat, FsError> {
        crate::fs::file::stat_path(&self.path)
    }

    /// TTY 不提供目录项。
    fn readdir(&self) -> Result<Option<DirectoryEntry>, FsError> {
        Err(FsError::NotDirectory)
    }

    /// TTY 不能作为目录枚举起点。
    fn for_each_dirent(
        &self,
        _emit: &mut dyn FnMut(&DirectoryEntry) -> Result<bool, FsError>,
    ) -> Result<(), FsError> {
        Err(FsError::NotDirectory)
    }

    /// 按输出模式加工输入并返回实际接受字节；一次逻辑写保 writer 跨空间等待。
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
        // 先快扫 writer 与空间，确需等待才建上下文；writer 竞争与队列满分开处理。
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
        // writer 未得时用独立上下文等串行锁，不与队列满混用同一 clear/park。
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
            // 持 writer 短取 copy/state 提交；输出满时释 copy/state 但保 writer 等空间。
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
                // 零进度阻塞：保 writer 跨空间等待，复查完整 reserve 后 park。
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

    /// TTY 不能按位置写入。
    fn write_at_from(
        &self,
        _offset: u64,
        _source: &mut WriteBuffer<'_>,
        _caller: FileIoContext<'_>,
    ) -> Result<usize, FsError> {
        Err(FsError::IllegalSeek)
    }
}

/// 无进度返回具体错误，已有正前缀优先保留。
fn progress(count: usize, error: FsError) -> Result<usize, FsError> {
    if count == 0 {
        Err(error)
    } else {
        Ok(count)
    }
}

/// 竞争 reader 只等待现有读写源，每次醒后重查此 OFD 与真实后台身份。
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

/// setter 等待 writer 时使用同一持续源，每次醒后检查自己的永久挂断。
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

/// 规范模式 read：持同一 copy 跨 peek→push→consume，park 前结束 guard。
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

/// 非规范交付结果：本段交付字节数或已切规范。
enum RawDeliver {
    /// 本段交付字节数；Err 保原始复制错误，调用者按总前缀合成。
    Delivered(Result<usize, FsError>),
    /// 模式已切规范，无进度时调用者转规范读。
    Switched,
    /// 此打开已永久挂断，调用者保留此前已经交付的正前缀。
    Hungup,
}

/// 每轮在 copy 下核挂断与后台身份，再短查模式及交付成功字节。
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

/// 非规范模式 read：四类 MIN/TIME 各有本次 read 局部 deadline。
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

/// MIN>0/TIME==0：等待 min(MIN,count) 字节；信号无进度才中断。
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

/// MIN==0/TIME>0：从本次 read 开始一次绝对总 deadline；信号返回已有前缀。
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

/// MIN>0/TIME>0：首字节前无 timer，实际新字节进度后更新间隔 deadline。
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

/// 从真实 Mm 复制 36 字节 termios 指针参数；完整 36 字节才成功。
fn ioctl_get(user: &UserThread, argument: usize) -> Result<usize, FsError> {
    let mm = user.mm().ok_or(FsError::BadAddress)?;
    let bytes = endpoint().state.lock().termios.to_bytes();
    let progress = mm.copyout(argument, &bytes);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(FsError::BadAddress);
    }
    Ok(0)
}

/// 从真实 Mm 复制并提交 36 字节 termios，action 0/1/2 对应 NOW/DRAIN/FLUSH。
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

/// 返回当前 termios 一致快照。
#[cfg(feature = "ktest")]
pub fn termios() -> Termios {
    endpoint().state.lock().termios
}

/// typed setter 只在入口检查后台访问，再共用实际配置提交。
#[cfg(feature = "ktest")]
pub fn set_termios(
    termios: Termios,
    action: u32,
    caller: FileIoContext<'_>,
) -> Result<(), FsError> {
    check_access(caller, SIGTTOU)?;
    apply_termios(termios, action, caller, None)
}

/// NOW 不等待 writer；DRAIN/FLUSH 等实际输出，醒后只重查本打开的挂断。
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

/// 在 copy 下先算硬件目标、释 state 后配硬件，回读成功再发表软件。
fn commit_termios(
    termios: Termios,
    action: u32,
    held: Option<crate::sync::SleepMutexGuard<'static, ()>>,
    file: Option<&TtyFile>,
) -> Result<(), FsError> {
    let copy = held.unwrap_or_else(|| endpoint().copy.lock());
    check_hangup(file)?;
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
    endpoint()
        .read_head
        .notify(PollMask::POLLIN | PollMask::POLLRDNORM);
    endpoint()
        .write_head
        .notify(PollMask::POLLOUT | PollMask::POLLWRNORM);
    schedule_transmit();
    Ok(())
}

/// 按选择器丢弃输入、输出或全部队列。
#[cfg(feature = "ktest")]
pub fn flush(selector: u32) -> Result<(), FsError> {
    endpoint_flush(selector, None)
}

/// 持 copy 清行规程输入并丢弃软件 RX 环已排队样本。
fn flush_input_locked(state: &mut ntty::NttyState) {
    state.flush_input();
    crate::console::discard_received();
}

/// raw flush 已完成一次后台检查；copy 内防止挂断后修改后继 open 的队列。
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

/// raw flow 已完成一次后台检查，实际提交在 copy 下重查永久状态。
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

/// Kernel typed 流控与 raw 请求共用实际提交，不推断用户身份。
#[cfg(feature = "ktest")]
pub fn flow(action: u32) -> Result<(), FsError> {
    endpoint_flow(action, None).map(|_| ())
}

/// typed drain 在边界检查后台访问，实际输出等待与 raw ioctl 共用。
#[cfg(feature = "ktest")]
pub fn drain(caller: FileIoContext<'_>) -> Result<(), FsError> {
    check_access(caller, SIGTTOU)?;
    drain_output(caller, None)
}

/// 等待软件队列与真实 TEMT，原持续源不增加 provider 登记。
fn drain_output(caller: FileIoContext<'_>, file: Option<&TtyFile>) -> Result<(), FsError> {
    let interruptible = matches!(caller, FileIoContext::User(_));
    let mut waiting: Option<(Arc<PollContext>, PollTable)> = None;
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
    // 软件队列已空，使用最近已确认的分频安排复查，不为 drain 再切换 DLAB。
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

/// 用户访问检查只取得原身份；Kernel 控制不推断某个用户调用者。
fn check_access(caller: FileIoContext<'_>, signum: u8) -> Result<(), FsError> {
    if let FileIoContext::User(user) = caller {
        let identity = user.identity().ok_or(FsError::NotTty)?;
        identity
            .graph()
            .check_terminal_access(&identity, endpoint(), signum)?;
    }
    Ok(())
}

/// copy 保护的同批输入共用一份稳定前台快照，不取行规程或端口锁。
fn foreground_snapshot() -> Option<(Weak<TaskGraph>, Arc<Pid>)> {
    endpoint()
        .control
        .lock()
        .as_ref()
        .map(|control| (control.graph.clone(), control.foreground.clone()))
}

/// 没有活收件者是合法结果，其余图协议错误不得静默吞掉。
fn send_foreground(graph: &Weak<TaskGraph>, foreground: &Arc<Pid>, signum: u8) {
    if let Some(graph) = graph.upgrade() {
        match graph.send_process_group(foreground, signum) {
            Ok(()) | Err(SignalError::NoProcess) => {}
            Err(error) => panic!("terminal foreground signal failed: {:?}", error),
        }
    }
}

/// 解除控制关系后按原稳定目标发送 HUP、CONT，不把解除误作物理挂断。
fn notify_disassociated(control: TerminalControl) {
    send_foreground(&control.graph, &control.foreground, SIGHUP);
    send_foreground(&control.graph, &control.foreground, SIGCONT);
}

/// 最后 session leader 退出只退休原会话和当时打开，后继 open 有独立状态。
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

/// 完整目标结构 copyin，失败不允许任何软件或图状态提交。
fn copy_ioctl_in<const N: usize>(user: &UserThread, argument: usize) -> Result<[u8; N], FsError> {
    let mm = user.mm().ok_or(FsError::BadAddress)?;
    let mut bytes = [0u8; N];
    let progress = mm.copyin(&mut bytes, argument);
    if progress.error.is_some() || progress.copied != N {
        return Err(FsError::BadAddress);
    }
    Ok(bytes)
}

/// 输出完整初始化目标字节，不将裸 Kernel 指针解引用。
fn copy_ioctl_out(user: &UserThread, argument: usize, bytes: &[u8]) -> Result<usize, FsError> {
    let mm = user.mm().ok_or(FsError::BadAddress)?;
    let progress = mm.copyout(argument, bytes);
    if progress.error.is_some() || progress.copied != bytes.len() {
        return Err(FsError::BadAddress);
    }
    Ok(0)
}

/// 控制请求只在 copy→Graph→relations/control 事务内更改原稳定关系。
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

/// 八字节 copyin 完成后整体提交；只对当次真实变化通知稳定前台。
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

/// 共享提交路径只复查永久 OFD 状态，不重复后台 setter 边界检查。
fn check_hangup(file: Option<&TtyFile>) -> Result<(), FsError> {
    if file.is_some_and(TtyFile::hungup) {
        return Err(FsError::DeviceIo);
    }
    Ok(())
}
