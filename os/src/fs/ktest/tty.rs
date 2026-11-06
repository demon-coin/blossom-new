//! TTY 行规程（`NttyState`）、终端配置编解码（`Termios` / `Winsize`）与双工串口（UART）集成测试模块。
//!
//! # 模块职责与架构定位
//! 本模块位于文件系统测试层（`fs::ktest::tty`），同时提供两类测试能力：
//! 1. **行规程与 VFS 设备接口测试**：验证 `NttyState` 在规范模式（Canonical）与非规范模式（Raw）下的
//!    输入编辑、信号生成、流控、回显、满环边界，以及 `/dev/ttyS0`、`/dev/console`、`/dev/tty` 的打开与 `ioctl` 契约；
//! 2. **宿主双工串口（UART Stage）协同基础设施**：导出 `UartStage` 与 `SerialTtyOwner`，供本模块及
//!    `fs::ktest::user::tty` 等用户态串口测试与宿主测试运行器进行带配额与 CRC32C 校验的串口收发握手。
//!
//! # 宿主双工串口握手协议（`UartStage` 状态流转）
//! ```text
//! [内核测试线程]                                      [宿主 ktest 运行器]
//!       |                                                      |
//!       +-- UartStage::begin(rx, tx, rx_delay, tx_delay)       |
//!       |     1. 接管串口接收控制权，同步在途接收中断并冲刷工作队列 |
//!       |     2. 输出 "# KTEST io <id> duplex <rx_len> <rx_crc> ... \n"
//!       | ---------------------------------------------------> |
//!       | <--------------------------------------------------- +-- 回复单字节握手 'H'
//!       |     3. 开启 rx.len() 字节接收配额 (arm_receive_bytes)  |
//!       | <================ 串口 RX/TX 双工收发 ================> |
//!       +-- UartStage::finish()                                |
//!       |     1. 等待 RX 配额耗尽、工作队列排空且 TX 发送移位器排空 |
//!       | <--------------------------------------------------- +-- 发送 "KTEST IO OK <id>\n"
//!       |     2. 校验用例编号匹配，按当前 Termios::CREAD 释放控制权 |
//! ```
//!
//! # 锁与资源回收顺序契约
//! - `SerialTtyOwner` 在套件级（`TtySuite`）启用串口到 TTY 子系统的接收挂钩，并在所有用例结束后的 `quiesce` 阶段关闭挂钩、排空残留工作项与发送队列；
//! - 单条用例（`TtyOwner`）在 `quiesce` 中必须先等待所有辅助内核线程退出（`join`）并确认 `UartStage` 已正常调用 `finish()` 归还串口控制权，随后才关闭打开的 `FileRef` 与 `Vfs` 视图。

use ::ktest::{TestContext, TestError, TestResult};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::console::RxSample;
use crate::fs::file::FileIoContext;
use crate::fs::ktest::{checked, recovered, settled, until};
use crate::fs::tty::ntty::NttyState;
use crate::fs::tty::termios::{
    Termios, BRKINT, CREAD, CS8, ECHO, ECHOCTL, ECHOE, ECHOK, ECHOKE, HUPCL, ICANON, ICRNL, IEXTEN,
    IMAXBEL, ISIG, ISTRIP, IUTF8, IXON, ONLCR, OPOST, TAB3, VMIN, VTIME,
};
use crate::fs::FileRef;
use crate::sync::SpinLock;
use crate::task::{JoinHandle, State, Task};
use crate::{cpu, mm, sched, task};

/// 计算给定字节切片的 Castagnoli CRC32C 校验和（包含首尾按位取反）。
fn crc32c(bytes: &[u8]) -> u32 {
    let mut state = u32::MAX;
    for byte in bytes {
        state ^= u32::from(*byte);
        for _ in 0..8 {
            state = (state >> 1) ^ if state & 1 != 0 { 0x82f6_3b78 } else { 0 };
        }
    }
    !state
}

/// 轮询等待串口控制通道就绪并读取单个控制字节；若超时则返回 `TestError`。
fn control_byte() -> Result<u8, TestError> {
    until("tty UART control input timed out", || {
        crate::console::receive_pending()
    })?;
    Ok(crate::ktest::control_read_byte())
}

/// 宿主双工串口（UART）测试阶段的会话控制器，负责收发声明握手、接收配额管理与完成确认（ACK）校验。
///
/// 正常测试路径必须成对调用 `begin` 与 `finish`，仅当 `finish` 成功校验宿主 ACK 后才将状态重置为非活动。
/// 若用例中途失败退出，`owned` 保持为 `true`，不在 `quiesce` 中跳过握手或强行重置，而是直接触发用例中止以防止污染后续串口流。
pub struct UartStage {
    /// 标记当前阶段是否正持有串口双工测试通道的控制权。
    owned: bool,
}

impl UartStage {
    /// 创建一个尚未进入双工测试阶段的 `UartStage` 控制器。
    pub fn new() -> Self {
        Self { owned: false }
    }

    /// 开启一段宿主双工串口测试阶段。
    ///
    /// 先接管串口接收控制权、同步在途接收中断并确认软件接收环为空，随后向宿主输出双工参数声明、等待 `'H'` 握手字节，最后开放 `rx.len()` 字节的接收配额。
    pub fn begin(
        &mut self,
        _context: &TestContext<'_>,
        rx: &[u8],
        tx: &[u8],
        rx_delay_ms: u32,
        tx_delay_ms: u32,
    ) -> TestResult {
        if self.owned {
            return Err(TestError("tty UART stage is already active"));
        }
        let id = crate::ktest::active_case().ok_or(TestError("tty UART IO outside case"))?;
        self.owned = true;
        // Step 1: 接管串口接收控制权，同步在途接收中断并冲刷 TTY 接收工作队列，确保软件接收环为空。
        crate::console::testing::claim_receive_control();
        crate::console::testing::synchronize_receive_irq()?;
        crate::fs::tty::flush_work();
        if !crate::console::testing::receive_queue_empty() {
            return Err(TestError("tty RX ring is not empty before stage"));
        }

        // Step 2: 向宿主发送包含期望收发字节数、CRC32C 校验值与延迟参数的双工测试声明，并等待宿主返回 'H' 握手确认。
        crate::ktest::report::write(format_args!(
            "  # KTEST io {} duplex {} {:08x} {} {} {:08x} {}\n",
            id,
            rx.len(),
            crc32c(rx),
            rx_delay_ms,
            tx.len(),
            crc32c(tx),
            tx_delay_ms
        ));
        if control_byte()? != b'H' {
            return Err(TestError("tty duplex handshake mismatch"));
        }
        // Step 3: 握手成功后开放指定字节数的 TTY 串口接收配额。
        crate::console::testing::arm_receive_bytes(rx.len());
        Ok(())
    }

    /// 结束当前双工串口测试阶段：等待接收配额耗尽、排空接收工作队列与发送缓冲区、读取并校验宿主 `"KTEST IO OK <id>"` 确认行，最后按当前 `CREAD` 配置释放串口接收控制权。
    pub fn finish(&mut self) -> TestResult {
        if !self.owned {
            return Err(TestError("tty UART stage is not active"));
        }
        // Step 1: 等待预设的串口接收配额被全部消费完毕。
        until("tty RX quota was not consumed", || {
            crate::console::testing::receive_quota_remaining().is_none()
        })?;
        // Step 2: 循环冲刷 TTY 工作队列直至控制台软件接收环完全排空（单次工作队列执行存在批处理预算上限，因此需循环至环空）。
        loop {
            crate::fs::tty::flush_work();
            if crate::console::testing::receive_queue_empty() {
                break;
            }
        }
        // Step 3: 等待 TTY 输出队列与硬件发送移位器（TEMT）排空，并再次冲刷工作队列。
        crate::fs::tty::drain(FileIoContext::Kernel)
            .map_err(|_| TestError("tty drain failed before ACK"))?;
        crate::fs::tty::flush_work();

        // Step 4: 从控制通道逐字节读取宿主返回的确认行（以 '\n' 结尾），校验 "KTEST IO OK <case_id>" 与当前活动用例 ID 一致。
        let mut line = [0u8; 64];
        let mut length = 0;
        loop {
            let byte = control_byte()?;
            if byte == b'\n' {
                break;
            }
            if length == line.len() {
                return Err(TestError("tty ACK exceeds bound"));
            }
            line[length] = byte;
            length += 1;
        }
        let digits = line[..length]
            .strip_prefix(b"KTEST IO OK ")
            .ok_or(TestError("host rejected tty duplex"))?;
        if digits.is_empty() {
            return Err(TestError("tty ACK case absent"));
        }
        let mut id = 0u32;
        for &byte in digits {
            if !byte.is_ascii_digit() {
                return Err(TestError("tty ACK case malformed"));
            }
            id = id
                .checked_mul(10)
                .and_then(|n| n.checked_add(u32::from(byte - b'0')))
                .ok_or(TestError("tty ACK case overflows"))?;
        }
        if Some(id) != crate::ktest::active_case() {
            return Err(TestError("tty ACK case mismatch"));
        }

        // Step 5: 根据当前 Termios 的 CREAD 标志恢复串口接收状态，并释放测试阶段独占控制权。
        let receiver = crate::fs::tty::termios().cflag & crate::fs::tty::termios::CREAD != 0;
        crate::console::testing::release_receive_control(receiver);
        self.owned = false;
        Ok(())
    }

    /// 返回当前是否正处于尚未结束的双工串口测试阶段。
    pub fn active(&self) -> bool {
        self.owned
    }
}

impl Default for UartStage {
    /// 创建默认的非活动 `UartStage` 实例。
    fn default() -> Self {
        Self::new()
    }
}

/// 串口 TTY 接收通道的套件级挂载守卫，负责在测试套件运行期间启用串口到 TTY 行规程的输入转发，并在套件结束时安全解绑。
pub(super) struct SerialTtyOwner {
    /// 标记当前是否已启用串口到 TTY 子系统的接收转发。
    attached: bool,
}

impl SerialTtyOwner {
    /// 在套件初始化阶段接管串口接收控制权、启用 TTY 接收转发、排空残留输入并恢复默认 `Termios` 配置。
    pub(super) fn attach() -> Result<Self, TestError> {
        crate::console::testing::claim_receive_control();
        crate::console::testing::set_tty_receive(true)?;
        Self::finish_receive()?;
        let receiver = crate::fs::tty::termios().cflag & CREAD != 0;
        crate::console::testing::release_receive_control(receiver);
        Ok(Self { attached: true })
    }

    /// 在套件所有测试用例结束后关闭新的 TTY 串口接收转发，排空已接收字节与待发送回显后恢复原始串口控制状态。
    pub(super) fn stop(&mut self) -> TestResult {
        if !self.attached {
            return Ok(());
        }
        crate::console::testing::claim_receive_control();
        crate::console::testing::set_tty_receive(false)?;
        Self::finish_receive()?;
        let receiver = crate::fs::tty::termios().cflag & CREAD != 0;
        crate::console::testing::release_receive_control(receiver);
        self.attached = false;
        Ok(())
    }

    /// 在串口接收生产端已停止的前提下，循环冲刷工作队列与接收环、等待输出排空（`drain`），并通过 `TCSAFLUSH` 重置为默认 `Termios`。
    fn finish_receive() -> TestResult {
        loop {
            crate::fs::tty::flush_work();
            if crate::console::testing::receive_queue_empty() {
                break;
            }
        }
        crate::fs::tty::drain(FileIoContext::Kernel)
            .map_err(|_| TestError("tty drain failed at receive handoff"))?;
        crate::fs::tty::set_termios(
            Termios::default(),
            crate::fs::tty::termios::TCSAFLUSH,
            FileIoContext::Kernel,
        )
        .map_err(|_| TestError("tty default termios failed at receive handoff"))?;
        crate::fs::tty::flush_work();
        Ok(())
    }
}

/// TTY 测试套件的共享资源夹具，管理套件级串口接收挂载状态以及内存与任务元数据基线。
struct TtySuite {
    /// 套件级串口 TTY 接收通道挂载守卫。
    serial: SerialTtyOwner,
    /// 套件开始前的物理内存与堆分配六域统计基线。
    memory: mm::MemoryStats,
    /// 套件开始前的任务与工作队列元数据统计基线。
    metadata: crate::workqueue::testing::MetadataStats,
}

impl ::ktest::Fixture for TtySuite {
    /// 等待后台日志、定时器与工作队列服务进入阻塞稳态，挂载串口 TTY 接收通道并记录套件级资源基线。
    fn set_up(_context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        crate::klog::testing::wait_service_blocked();
        until("tty timer services did not block", || {
            (0..cpu::count()).all(|id| {
                crate::timer::testing::service_task(id)
                    .is_some_and(|task| sched::test_state(&task) == crate::task::State::Blocked)
            })
        })?;
        let mut serial = SerialTtyOwner::attach()?;
        crate::workqueue::testing::wait_services_blocked();
        let memory = match settled() {
            Ok(memory) => memory,
            Err(error) => {
                serial.stop()?;
                return Err(error);
            }
        };
        Ok(Self {
            serial,
            memory,
            metadata: crate::workqueue::testing::metadata_stats(),
        })
    }

    /// 在套件内全部测试用例完成后关闭串口 TTY 接收转发并排空残留数据。
    fn quiesce(&mut self, _context: &mut TestContext<'_>) -> TestResult {
        self.serial.stop()
    }

    /// 校验串口接收已解绑，并断言套件运行前后的内存六域统计与任务元数据统计完全归零复原。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        assert!(
            !self.serial.attached,
            "tty suite receive owner is still attached"
        );
        recovered(context, "tty-suite-last-release", self.memory)?;
        let actual = crate::workqueue::testing::metadata_stats();
        context.diagnostic(format_args!(
            "tty-suite metadata-before={:?} metadata-after={:?}",
            self.metadata, actual
        ));
        context.expect_eq(actual, self.metadata);
        if actual != self.metadata {
            return Err(TestError("tty suite task metadata leaked"));
        }
        Ok(())
    }
}

/// 单条 TTY 测试用例的资源与执行环境夹具。
struct TtyOwner {
    /// 本用例开始前的内存六域统计基线。
    before: mm::MemoryStats,
    /// 本用例开始前的任务与工作队列元数据统计基线。
    metadata: crate::workqueue::testing::MetadataStats,
    /// 本用例启动的辅助内核任务汇合句柄列表。
    joins: Vec<JoinHandle>,
    /// 当前受观察辅助内核任务的弱引用槽位，用于检测该任务是否已进入 `State::Blocked`。
    observer: Option<Arc<SpinLock<Option<Weak<Task>>>>>,
    /// 本用例的宿主双工串口阶段控制器。
    stage: UartStage,
    /// 本用例按需创建的独立 `devfs` 虚拟文件系统实例。
    vfs: Option<Arc<crate::fs::Vfs>>,
    /// 本用例的 VFS 路径解析上下文（根目录与当前工作目录）。
    context: Option<crate::fs::FsContext>,
    /// 本用例已打开的 TTY 设备文件引用列表。
    files: Vec<FileRef>,
}

impl TtyOwner {
    /// 等待本用例启动的所有辅助内核任务执行完毕并脱离内核栈，同时清空任务观察器。
    fn join(&mut self, context: &mut TestContext<'_>) {
        while let Some(join) = self.joins.pop() {
            context.expect_eq(join.join(), 0);
        }
        self.observer = None;
    }

    /// 在当前 CPU 上创建并启动一个辅助内核任务，在执行 `action` 前先将其 `Weak<Task>` 写入 `observer` 以便主测试线程观测其阻塞状态。
    fn run(&mut self, action: impl FnOnce() -> i32 + Send + 'static) -> TestResult {
        let observer =
            Arc::try_new(SpinLock::new(None)).map_err(|_| TestError("tty task observer OOM"))?;
        let observed = observer.clone();
        let prepared = task::prepare_kernel_on(cpu::current().id, move || {
            *observed.lock() = Some(Arc::downgrade(&task::current_task()));
            action()
        })
        .map_err(|_| TestError("tty task preparation OOM"))?;
        self.observer = Some(observer);
        self.joins.push(prepared.publish());
        Ok(())
    }

    /// 检查最近启动的受观察辅助内核任务当前是否已处于调度器 `State::Blocked` 状态。
    fn blocked(&self) -> bool {
        self.observer
            .as_ref()
            .and_then(|observer| observer.lock().as_ref().and_then(Weak::upgrade))
            .is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 为本用例创建独立的内存 `devfs` 实例、Inode 缓存与 `Vfs` 根视图，并保存在夹具中。
    fn prepare_view(&mut self, context: &TestContext<'_>) -> TestResult {
        let fs = checked(context, crate::fs::devfs::try_new(0o755))?;
        let cache = Arc::try_new(checked(context, crate::fs::InodeCache::try_new(16))?)
            .map_err(|_| TestError("tty inode cache owner OOM"))?;
        let vfs = Arc::try_new(checked(
            context,
            crate::fs::Vfs::try_new(
                crate::fs::memfs::Filesystem::Memory(fs),
                cache,
                16,
                1,
                crate::block::AccessMode::ReadOnly,
            ),
        )?)
        .map_err(|_| TestError("tty VFS owner OOM"))?;
        let fs_context = checked(context, crate::fs::FsContext::new(vfs.root(), vfs.root()))?;
        self.context = Some(fs_context);
        self.vfs = Some(vfs);
        Ok(())
    }

    /// 在本用例的 `devfs` 视图中解析并按指定标志位打开目标 TTY 设备节点（如 `b"/ttyS0"` 或 `b"/console"`），将返回的 `FileRef` 存入 `self.files`。
    fn open_tty(&mut self, context: &TestContext<'_>, name: &[u8], flags: u32) -> TestResult {
        if self.vfs.is_none() {
            self.prepare_view(context)?;
        }
        let vfs = self.vfs.as_ref().expect("tty view was not prepared");
        let fs_context = self.context.as_ref().expect("tty context was not prepared");
        let path = checked(
            context,
            vfs.resolve(fs_context, name, crate::fs::ResolveMode::Follow),
        )?;
        let file = checked(
            context,
            crate::fs::open_path_with_flags(vfs, path, flags, FileIoContext::Kernel),
        )?;
        self.files.push(file);
        Ok(())
    }

    /// 按索引借用本用例已打开的 TTY 设备文件引用。
    fn tty(&self, index: usize) -> &FileRef {
        &self.files[index]
    }

    /// 关闭并清空本用例打开的所有 TTY 文件引用，并释放 `FsContext` 与 `Vfs` 视图（须在所有辅助子任务退出后调用）。
    fn release(&mut self) {
        self.files.clear();
        drop(self.context.take());
        drop(self.vfs.take());
    }
}

impl ::ktest::Fixture<TtySuite> for TtyOwner {
    /// 在单条 TTY 用例开始前记录内存六域与任务元数据基线，并初始化空的任务、串口阶段与文件视图容器。
    fn set_up(_context: &mut TestContext<'_>, _suite: &TtySuite) -> Result<Self, TestError> {
        Ok(Self {
            before: settled()?,
            metadata: crate::workqueue::testing::metadata_stats(),
            joins: Vec::new(),
            observer: None,
            stage: UartStage::new(),
            vfs: None,
            context: None,
            files: Vec::new(),
        })
    }

    /// 在单条用例结束时汇合所有辅助子任务、确认串口双工阶段已正常关闭，随后释放本用例打开的 TTY 文件与 VFS 视图。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.join(context);
        if self.stage.active() {
            return Err(TestError("tty stage is still active at quiesce"));
        }
        self.release();
        Ok(())
    }

    /// 销毁用例夹具并校验内存六域统计与任务/工作队列元数据是否完全恢复至用例开始前的基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        let before = self.before;
        let metadata = self.metadata;
        drop(self);
        recovered(context, "tty-case-last-release", before)?;
        let actual = crate::workqueue::testing::metadata_stats();
        context.diagnostic(format_args!(
            "tty-case metadata-before={:?} metadata-after={:?}",
            metadata, actual
        ));
        context.expect_eq(actual, metadata);
        if actual != metadata {
            return Err(TestError("tty case task metadata leaked"));
        }
        Ok(())
    }
}

/// 导出 TTY 测试套件的注册锚点供父模块挂载。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    cases::__ktest_anchor()
}

/// TTY 行规程、终端 ABI 编解码与串口 I/O 集成测试套件。
#[ktest::suite(fixture = TtyOwner, suite_resources = TtySuite, context = "Thread", tags("fs-tty"), timeout_ms = 45000)]
pub(crate) mod cases {
    use super::*;

    /// 验证 `pid_t` 编解码边界、`Winsize` 窗口尺寸四字段序列化以及 `Termios`（含 `TOSTOP`）往返转换。
    ///
    /// - **测试目标**：确认终端控制 `ioctl` 使用的 `pid_t`、`Winsize` 和 `Termios` 二进制编解码器严格保留全部字段并拒绝负数 PID。
    #[test]
    fn control_abi_codecs_preserve_all_fields(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        use crate::fs::tty::termios::{decode_pid, encode_pid, Winsize, TOSTOP};
        use crate::fs::FsError;
        assert_eq!(decode_pid(encode_pid(0)), Ok(0));
        assert_eq!(
            decode_pid(encode_pid(i32::MAX as usize)),
            Ok(i32::MAX as usize)
        );
        assert_eq!(
            decode_pid((-1i32).to_le_bytes()),
            Err(FsError::InvalidOperation)
        );
        assert_eq!(
            decode_pid(i32::MIN.to_le_bytes()),
            Err(FsError::InvalidOperation)
        );

        assert_eq!(Winsize::initial().to_bytes(), [24, 0, 80, 0, 0, 0, 0, 0]);
        let bytes = [1, 2, 3, 4, 5, 6, 7, 8];
        let size = Winsize::from_bytes(bytes);
        assert_eq!(
            (size.rows, size.columns, size.xpixel, size.ypixel),
            (0x201, 0x403, 0x605, 0x807)
        );
        assert_eq!(size.to_bytes(), bytes);

        let mut config = Termios::default();
        config.lflag |= TOSTOP;
        assert_eq!(Termios::from_bytes(config.to_bytes()), Ok(config));
        Ok(())
    }

    /// 验证输入信号字符（`VINTR` / `VQUIT` / `VSUSP`）与 `ISTRIP`、`VLNEXT` 字面转义、`NOFLSH` 及 `BRKINT` / `IGNBRK` 的交互顺序和冲刷边界。
    ///
    /// - **测试目标**：确认信号触发时默认清空已缓存的输入/输出队列但保留同批后续输入；启用 `VLNEXT` 时将信号字节作为普通数据入队；启用 `NOFLSH` 时只产生信号而不清空输入队列。
    #[test]
    fn input_signals_preserve_order_and_flush_boundary(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        use crate::fs::tty::termios::{IGNBRK, NOFLSH, VLNEXT};
        let mut state = NttyState::new();
        state.termios.iflag = ISTRIP;
        state.termios.lflag = ISIG;
        for (byte, signal) in [(0x83, 2), (28, 3), (26, 20)] {
            state.receive_sample(RxSample {
                byte: b'o',
                status: 1,
            });
            state.output.push(b'O');
            state.xchar = Some(b'X');
            let outcome = state.receive_sample(RxSample { byte, status: 1 });
            assert_eq!(outcome.signal, Some(signal));
            assert!(outcome.read_changed && outcome.write_changed);
            assert!(state.input.is_empty() && state.output_empty());
            state.receive_sample(RxSample {
                byte: b'n',
                status: 1,
            });
            let mut bytes = [0; 4];
            assert_eq!(state.peek_raw(&mut bytes), 1);
            assert_eq!(bytes[0], b'n');
            state.flush_input();
        }

        state.termios.lflag = ICANON | ISIG | IEXTEN;
        state.receive_sample(RxSample {
            byte: state.termios.cc[VLNEXT],
            status: 1,
        });
        assert_eq!(
            state.receive_sample(RxSample { byte: 3, status: 1 }).signal,
            None
        );
        assert_eq!(state.input.len(), 1);
        state.termios.lflag = ISIG | NOFLSH;
        assert_eq!(
            state.receive_sample(RxSample { byte: 3, status: 1 }).signal,
            Some(2)
        );
        assert_eq!(state.input.len(), 1);

        state.termios.iflag = BRKINT;
        assert_eq!(
            state
                .receive_sample(RxSample {
                    byte: 0,
                    status: 0x10
                })
                .signal,
            Some(2)
        );
        assert_eq!(state.input.len(), 1);
        state.termios.lflag = ISIG;
        assert_eq!(
            state
                .receive_sample(RxSample {
                    byte: 0,
                    status: 0x10
                })
                .signal,
            Some(2)
        );
        assert!(state.input.is_empty());

        state.termios.iflag |= IGNBRK;
        assert_eq!(
            state
                .receive_sample(RxSample {
                    byte: 0,
                    status: 0x10
                })
                .signal,
            None
        );

        state.termios.iflag = 0;
        state.termios.lflag = 0;
        assert_eq!(
            state.receive_sample(RxSample { byte: 3, status: 1 }).signal,
            None
        );
        assert_eq!(state.input.len(), 1);
        Ok(())
    }

    /// 验证在内核调用上下文（`FileIoContext::Kernel`）下传入裸指针执行用户态专属 `ioctl` 命令或打开 `/dev/tty` 时，返回明确的错误码。
    ///
    /// - **测试目标**：确认需要用户地址空间或进程控制终端上下文的操作在 `FileIoContext::Kernel` 下被安全拒绝（分别返回 `InvalidOperation` 与 `NoDeviceAddress`）。
    #[test]
    fn control_requests_reject_kernel_pointer_context(
        context: &mut TestContext<'_>,
        fixture: &mut TtyOwner,
    ) -> TestResult {
        use crate::fs::FsError;
        fixture.open_tty(context, b"/ttyS0", 2)?;
        for command in [0x540e, 0x5422, 0x540f, 0x5410, 0x5429, 0x5413, 0x5414] {
            assert_eq!(
                fixture
                    .tty(0)
                    .ioctl(command, usize::MAX, FileIoContext::Kernel)
                    .err(),
                Some(FsError::InvalidOperation)
            );
        }

        let vfs = fixture.vfs.as_ref().expect("tty view missing");
        let fs_context = fixture.context.as_ref().expect("tty context missing");
        let path = checked(
            context,
            vfs.resolve(fs_context, b"/tty", crate::fs::ResolveMode::Follow),
        )?;
        assert_eq!(
            crate::fs::open_path_with_flags(vfs, path, 2, FileIoContext::Kernel).err(),
            Some(FsError::NoDeviceAddress)
        );
        Ok(())
    }

    use crate::fs::SeekFrom;

    /// 验证 `Termios::default()` 的 36 字节二进制序列化与反序列化结果同标准 Linux 默认终端属性一致。
    #[test]
    fn termios_codec_matches_linux_default(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let termios = Termios::default();
        let bytes = termios.to_bytes();
        assert_eq!(bytes.len(), 36);
        let decoded = Termios::from_bytes(bytes).map_err(|_| TestError("tty termios rejected"))?;
        assert_eq!(decoded, termios);
        assert_eq!(
            decoded.cc,
            [3, 28, 127, 21, 4, 0, 1, 0, 17, 19, 26, 0, 18, 15, 23, 22, 0, 0, 0]
        );
        assert_eq!(decoded.iflag, ICRNL | IXON);
        assert_eq!(decoded.oflag, OPOST | ONLCR);
        assert_eq!(decoded.cflag, 0x0f | CS8 | CREAD | HUPCL);
        assert_eq!(
            decoded.lflag,
            ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN
        );
        Ok(())
    }

    /// 验证 `Termios::from_bytes` 拒绝包含未实现标志位（如特定延迟掩码、大小写转换或硬件流控位）的配置字节流。
    #[test]
    fn termios_rejects_unselected_bits(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut termios = Termios::default();
        termios.iflag |= 0x200;
        assert!(Termios::from_bytes(termios.to_bytes()).is_err());
        let mut termios = Termios::default();
        termios.oflag |= 0x02;
        assert!(Termios::from_bytes(termios.to_bytes()).is_err());
        let mut termios = Termios::default();
        termios.lflag |= 0x04;
        assert!(Termios::from_bytes(termios.to_bytes()).is_err());
        let mut termios = Termios::default();
        termios.cflag |= 0x8000_0000;
        assert!(Termios::from_bytes(termios.to_bytes()).is_err());
        let mut termios = Termios::default();
        termios.cflag |= 0x1000;
        assert!(Termios::from_bytes(termios.to_bytes()).is_err());
        Ok(())
    }

    /// 验证规范模式（`ICANON`）下收到换行符 `'\n'` 后立即提交当前行记录并可供读取。
    #[test]
    fn canonical_record_newline(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        for &byte in b"hi\n" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        let mut out = [0u8; 16];
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (3, false));
        assert_eq!(&out[..length], b"hi\n");
        state.consume_canonical(length, eof);
        Ok(())
    }

    /// 验证规范模式下 `VEOF`（Ctrl-D）在空行与非空行中的不同提交语义。
    ///
    /// - **测试目标**：确认空行收到 `VEOF` 时产生一次零长度读取（`eof = true`），非空行收到 `VEOF` 时立即提交已输入的字符且不将 `0x04` 控制字节本身混入返回数据。
    #[test]
    fn canonical_eof_empty_and_content(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.receive_sample(RxSample {
            byte: 4,
            status: 0x01,
        });
        let mut out = [0u8; 16];
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (0, true));
        state.consume_canonical(length, eof);
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (0, false));

        state.input.clear();
        for &byte in b"ab\x04" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (2, false));
        assert_eq!(&out[..length], b"ab");
        Ok(())
    }

    /// 验证规范模式下 `VERASE`（`0x7f`）擦除前一个单字节 ASCII 字符。
    #[test]
    fn canonical_erase_single(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        for &byte in b"ab\x7f\n" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        let mut out = [0u8; 16];
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!(&out[..length], b"a\n");
        state.consume_canonical(length, eof);
        Ok(())
    }

    /// 验证启用 `IUTF8` 时，`VERASE` 按 UTF-8 多字节字符边界回退整字符（如 2 字节的 `'é'`），而非仅删除单个尾字节。
    #[test]
    fn canonical_erase_utf8_boundary(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.iflag |= IUTF8;
        for byte in [0xc3, 0xa9, 0x7f, b'\n'] {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        let mut out = [0u8; 16];
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!(&out[..length], b"\n");
        state.consume_canonical(length, eof);
        Ok(())
    }

    /// 验证输入路径上的 `ICRNL`（回车 `'\r'` 映射为换行 `'\n'`）与 `ISTRIP`（剥离最高位第 7 位）转换语义。
    #[test]
    fn input_mapping_and_strip(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.lflag &= !ICANON;
        state.receive_sample(RxSample {
            byte: b'\r',
            status: 0x01,
        });
        let mut out = [0u8; 8];
        let peeked = state.peek_raw(&mut out);
        assert_eq!(peeked, 1);
        assert_eq!(out[0], b'\n');
        state.consume_raw(peeked);

        state.input.clear();
        state.termios.iflag |= ISTRIP;
        state.receive_sample(RxSample {
            byte: 0xc3,
            status: 0x01,
        });
        let peeked = state.peek_raw(&mut out);
        assert_eq!(peeked, 1);
        assert_eq!(out[0], 0x43);
        state.consume_raw(peeked);
        Ok(())
    }

    /// 验证软件流控（`IXON`）下收到 `VSTOP`（Ctrl-S，`19`）暂停输出但不阻塞后续输入入队，收到 `VSTART`（Ctrl-Q，`17`）恢复输出。
    #[test]
    fn input_flow_stop_continues_rx(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.receive_sample(RxSample {
            byte: 19,
            status: 0x01,
        });
        assert!(state.stopped);
        state.receive_sample(RxSample {
            byte: b'a',
            status: 0x01,
        });
        assert_eq!(state.input.len(), 1);
        state.receive_sample(RxSample {
            byte: 17,
            status: 0x01,
        });
        assert!(!state.stopped);
        Ok(())
    }

    /// 验证输出处理开启 `OPOST | ONLCR` 时，单个换行符 `'\n'` 被扩展为 `"\r\n"` 两字节写入输出缓冲区。
    #[test]
    fn output_onlcr_expands_lf(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        let termios = state.termios;
        assert!(state.output_byte(termios, b'\n'));
        assert_eq!(state.output.len(), 2);
        Ok(())
    }

    /// 验证 `TAB3` 将水平制表符 `'\t'` 按当前列位置展开为空格，且当输出缓冲区剩余空间不足以容纳展开后的全部字节时不消费该输入字节。
    #[test]
    fn output_tab3_and_full_reserve(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.oflag |= TAB3;
        let termios = state.termios;
        assert!(state.output_byte(termios, b'\t'));
        assert_eq!(state.output.len(), 8);

        state.output.clear();
        let termios = state.termios;
        while state.output.space() != 1 {
            assert!(state.output.push(b'x'));
        }
        assert!(!state.output_byte(termios, b'\n'));
        assert_eq!(state.output.space(), 1);
        Ok(())
    }

    /// 验证在输出暂停状态下连续收到 `VDISCARD`（Ctrl-O）时，第一次立即清空待发送输出、回显与重印快照并置位 `FLUSHO`，第二次清除 `FLUSHO` 标志，且恢复输出后不影响输入队列顺序与新输出写入。
    #[test]
    fn vdiscard_toggles_output_flush(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        use crate::fs::tty::termios::{FLUSHO, IXANY, NOFLSH, VDISCARD, VREPRINT};

        let mut state = NttyState::new();
        state.termios.oflag = OPOST;
        state.termios.iflag |= IXANY;
        state.termios.lflag |= NOFLSH;
        state.tco_stopped = true;
        state.stopped = true;
        let termios = state.termios;
        assert!(state.output_byte(termios, b'O'));
        state.xchar = Some(b'X');
        state.receive_sample(RxSample {
            byte: b'a',
            status: 0x01,
        });
        state.receive_sample(RxSample {
            byte: termios.cc[VREPRINT],
            status: 0x01,
        });
        assert_eq!(state.output.peek(), Some(b'O'));
        assert!(!state.echo.is_empty());
        assert_eq!(state.xchar, Some(b'X'));
        let reprint = state.pending_reprint.as_ref().unwrap();
        assert_eq!(reprint.len, 1);
        assert_eq!(reprint.bytes[0], b'a');
        assert_eq!(reprint.done, 0);

        // 先通过前面的输入生成待发送回显与重印快照，随后关闭 ECHO 标志，以便单独观测 VDISCARD 对已有待发送状态的清理效果。
        state.termios.lflag &= !ECHO;
        let termios = state.termios;
        assert!(!state.flusho());
        state.receive_sample(RxSample {
            byte: termios.cc[VDISCARD],
            status: 0x01,
        });
        assert!(state.flusho());
        assert_eq!(state.termios.lflag, termios.lflag | FLUSHO);
        assert_eq!(state.termios.iflag, termios.iflag);
        assert!(state.output.is_empty());
        assert!(state.echo.is_empty());
        assert!(state.xchar.is_none());
        assert!(state.pending_reprint.is_none());
        assert_eq!(state.input.len(), 1);
        assert_eq!(state.input.edit_len(), 1);
        assert!(state.stopped && state.tco_stopped);
        assert!(state.output_can_accept(state.termios, b'N'));

        state.receive_sample(RxSample {
            byte: termios.cc[VDISCARD],
            status: 0x01,
        });
        assert!(!state.flusho());
        assert_eq!(state.termios.lflag, termios.lflag);
        assert_eq!(state.termios.iflag, termios.iflag);
        assert!(state.stopped && state.tco_stopped);
        assert!(state.output_empty());
        for &byte in b"b\n" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        let mut input = [0u8; 3];
        assert_eq!(state.peek_canonical(&mut input), (3, false));
        assert_eq!(&input, b"ab\n");
        state.consume_canonical(3, false);
        assert!(state.input.is_empty());

        state.tco_stopped = false;
        state.stopped = false;
        assert!(!state.process_echo());
        assert!(state.output_can_accept(state.termios, b'N'));
        assert!(state.output_byte(state.termios, b'N'));
        assert_eq!(state.output.pop(), Some(b'N'));
        assert_eq!(state.output.pop(), None);
        assert!(state.output_empty());
        assert_eq!(state.overrun, 0);
        Ok(())
    }

    /// 验证规范模式下 `poll_readable()` 在行未提交前返回 `false`，收到换行符提交后返回 `true`。
    #[test]
    fn poll_canonical_committed(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        assert!(!state.poll_readable());
        state.receive_sample(RxSample {
            byte: b'a',
            status: 0x01,
        });
        assert!(!state.poll_readable());
        state.receive_sample(RxSample {
            byte: b'\n',
            status: 0x01,
        });
        assert!(state.poll_readable());
        Ok(())
    }

    /// 验证非规范模式（`!ICANON`）下 `poll_readable()` 仅当输入队列字节数达到 `c_cc[VMIN]` 门限时才返回 `true`。
    #[test]
    fn poll_noncanonical_min(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.lflag &= !ICANON;
        state.termios.cc[VMIN] = 4;
        state.termios.cc[VTIME] = 0;
        state.receive_sample(RxSample {
            byte: b'a',
            status: 0x01,
        });
        assert!(!state.poll_readable());
        for &byte in b"bcd" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        assert!(state.poll_readable());
        Ok(())
    }

    /// 验证输入缓冲区满（4096 字节）后再收到新字节时：丢弃新字节、递增 `overrun` 溢出计数，并在启用 `IMAXBEL` 时向输出队列压入响铃字节。
    #[test]
    fn imaxbel_and_overrun_are_real(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.lflag = 0;
        state.termios.iflag = IMAXBEL;
        for _ in 0..4096 {
            state.receive_sample(RxSample {
                byte: b'x',
                status: 0x01,
            });
        }
        assert_eq!(state.input.len(), 4096);
        let before = state.output.len();
        state.receive_sample(RxSample {
            byte: b'y',
            status: 0x01,
        });
        assert_eq!(state.input.len(), 4096);
        assert_eq!(state.output.len(), before + 1);
        assert_eq!(state.overrun, 1);
        Ok(())
    }

    /// 验证启用 `BRKINT`（且未启用 `ISIG`）时，收到串口 Break 状态条件会在本地清空输入队列与回显队列。
    #[test]
    fn brkint_flushes_locally(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.iflag |= BRKINT;
        for &byte in b"ab" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        assert_ne!(state.input.len(), 0);
        state.receive_sample(RxSample {
            byte: 0,
            status: 0x10,
        });
        assert_eq!(state.input.len(), 0);
        assert!(state.echo.is_empty());
        Ok(())
    }

    /// 验证打开 `/dev/ttyS0` 与 `/dev/console` 后的设备号（`rdev`）以及不支持的文件操作（`seek`、`sync_range`、`truncate`、未知或内核态 `ioctl`）返回预期错误码。
    #[test]
    fn real_open_capabilities_poll_and_stat(
        context: &mut TestContext<'_>,
        fixture: &mut TtyOwner,
    ) -> TestResult {
        fixture.open_tty(context, b"/ttyS0", 2)?;
        fixture.open_tty(context, b"/console", 2)?;
        let first = fixture.tty(0);
        let second = fixture.tty(1);
        let stat = checked(context, first.stat())?;
        assert_eq!(stat.rdev, 0x440);
        let stat = checked(context, second.stat())?;
        assert_eq!(stat.rdev, 0x501);
        assert_eq!(
            first.seek(SeekFrom::Start(0)).err(),
            Some(crate::fs::FsError::IllegalSeek)
        );
        assert_eq!(
            first.sync_range(0..1, false).err(),
            Some(crate::fs::FsError::InvalidOperation)
        );
        assert_eq!(
            first.truncate(0).err(),
            Some(crate::fs::FsError::InvalidOperation)
        );
        assert_eq!(
            first.ioctl(0xffff, 0, FileIoContext::Kernel).err(),
            Some(crate::fs::FsError::NotTty)
        );
        // 内核上下文（FileIoContext::Kernel）对需要用户态缓冲区的已识别 ioctl 命令返回 InvalidOperation。
        assert_eq!(
            first.ioctl(0x5401, 0, FileIoContext::Kernel).err(),
            Some(crate::fs::FsError::InvalidOperation)
        );
        Ok(())
    }

    /// 验证在 4095 字节满编辑行上触发 `VKILL`（Ctrl-U，`21`）时，按列宽精确生成 4095 组退格擦除序列 `"\x08 \x08"`，且后续输入的字符 `'y'` 严格排在擦除序列之后输出。
    #[test]
    fn kill_line_op_emits_exact_backspaces(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.oflag = 0;
        for _ in 0..4095 {
            state.receive_sample(RxSample {
                byte: b'x',
                status: 0x01,
            });
        }
        let mut offset = 0;
        while offset < 4095 {
            let _ = state.process_echo();
            let mut progressed = false;
            while let Some(byte) = state.output.pop() {
                progressed = true;
                assert_eq!(byte, b'x');
                offset += 1;
            }
            assert!(progressed || offset == 4095);
        }

        state.receive_sample(RxSample {
            byte: 21,
            status: 0x01,
        });
        assert_eq!(state.input.edit_len(), 0);
        state.receive_sample(RxSample {
            byte: b'y',
            status: 0x01,
        });

        let mut groups = 0;
        let mut phase = 0;
        let mut seen_y = false;
        loop {
            let _ = state.process_echo();
            let mut progressed = false;
            while let Some(byte) = state.output.pop() {
                progressed = true;
                if groups < 4095 {
                    assert_eq!(byte, [0x08, b' ', 0x08][phase]);
                    phase += 1;
                    if phase == 3 {
                        phase = 0;
                        groups += 1;
                    }
                } else {
                    assert_eq!(byte, b'y');
                    seen_y = true;
                }
            }
            if seen_y {
                break;
            }
            assert!(progressed);
        }
        assert_eq!(groups, 4095);
        Ok(())
    }

    /// 验证在 4090 字节未提交行上触发 `VREPRINT`（Ctrl-R，`18`）并紧跟输入 `'Z'` 时，输出流严格按照“原 4090 字节回显 → `^R\n` → 4090 字节重印快照 → `'Z'`”的顺序分段生成。
    #[test]
    fn reprint_snapshot_order_with_control_prefix(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.oflag = 0;
        for _ in 0..4090 {
            state.receive_sample(RxSample {
                byte: b'q',
                status: 0x01,
            });
        }
        state.receive_sample(RxSample {
            byte: 18,
            status: 0x01,
        });
        state.receive_sample(RxSample {
            byte: b'Z',
            status: 0x01,
        });

        let mut offset = 0;
        let total = 4090 + 3 + 4090 + 1;
        loop {
            let _ = state.process_echo();
            let mut progressed = !state.echo.is_empty() || state.pending_reprint.is_some();
            while let Some(byte) = state.output.pop() {
                progressed = true;
                let expected = if offset < 4090 {
                    b'q'
                } else if offset == 4090 {
                    b'^'
                } else if offset == 4091 {
                    b'R'
                } else if offset == 4092 {
                    b'\n'
                } else if offset < 8183 {
                    b'q'
                } else {
                    b'Z'
                };
                assert_eq!(byte, expected);
                offset += 1;
            }
            if offset == total {
                break;
            }
            assert!(progressed);
        }
        Ok(())
    }

    /// 验证当输入环被 4096 字节已提交记录占满时，紧随其后的 `VEOF` 被挂起（阻塞后续普通字节入队），一旦读取消费腾出 1 字节空间即优先发布该 `EOF` 标记，随后才接纳新的输入字符。
    #[test]
    fn pending_eof_blocks_data_then_publishes_first(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        for _ in 0..4095 {
            state.receive_sample(RxSample {
                byte: b'a',
                status: 0x01,
            });
        }
        state.receive_sample(RxSample {
            byte: b'\n',
            status: 0x01,
        });
        assert_eq!(state.input.len(), 4096);
        state.receive_sample(RxSample {
            byte: 4,
            status: 0x01,
        });
        let overrun = state.overrun;
        state.receive_sample(RxSample {
            byte: b'Z',
            status: 0x01,
        });
        assert_eq!(state.input.len(), 4096);
        assert!(overrun < state.overrun);

        // Step 1: 短读消费 1 个字节腾出环空间，此时挂起的 VEOF 立即占位发布；环再次填满，后续输入的 'Z' 仍触发溢出（overrun）。
        let mut out = [0u8; 4096];
        let (length, eof) = state.peek_canonical(&mut out[..1]);
        assert_eq!((length, eof), (1, false));
        assert_eq!(out[0], b'a');
        state.consume_canonical(length, eof);
        let overrun = state.overrun;
        state.receive_sample(RxSample {
            byte: b'Z',
            status: 0x01,
        });
        assert!(overrun < state.overrun);

        // Step 2: 读完旧记录剩余的 4095 字节，紧接着读出零长度的 EOF 记录；此时输入环排空，新输入的 "Z\n" 正常入队并交付。
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!(length, 4095);
        assert!(!eof);
        state.consume_canonical(length, eof);
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (0, true));
        state.consume_canonical(length, eof);
        state.receive_sample(RxSample {
            byte: b'Z',
            status: 0x01,
        });
        state.receive_sample(RxSample {
            byte: b'\n',
            status: 0x01,
        });
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (2, false));
        assert_eq!(&out[..length], b"Z\n");
        Ok(())
    }

    /// 验证纯原始模式（Raw Mode）下输入环可写满全部 4096 字节，且包含 `0x00`、`0xff` 及各类控制字符在内的所有字节均原样无损读出。
    #[test]
    fn raw_full_queue_reads_exact_bytes(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.iflag = 0;
        state.termios.lflag = 0;
        state.termios.oflag = 0;
        for i in 0..4096 {
            state.receive_sample(RxSample {
                byte: (i & 0xff) as u8,
                status: 0x01,
            });
        }
        assert_eq!(state.input.len(), 4096);
        assert!(state.poll_readable());

        let mut out = [0u8; 4096];
        let peeked = state.peek_raw(&mut out);
        assert_eq!(peeked, 4096);
        for (i, &byte) in out.iter().enumerate() {
            assert_eq!(byte, (i & 0xff) as u8);
        }
        state.consume_raw(peeked);
        assert!(state.input.is_empty());
        Ok(())
    }

    /// 验证规范模式下连续输入 4096 个普通字符 `'a'` 时，第 4096 个 `'a'` 因需预留行尾定界符槽位而触发 `overrun`（仅保留 4095 个 `'a'`），随后输入的 `'\n'` 可填入第 4096 格并通过多次短读完整拼回。
    #[test]
    fn canonical_full_line_eol_and_short_read(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        for _ in 0..4096 {
            state.receive_sample(RxSample {
                byte: b'a',
                status: 0x01,
            });
        }
        // 第 4096 个 'a' 超出规范模式单行 4095 数据字节上限，计入 overrun 而不写入接收环，为行尾换行符保留最后 1 个槽位。
        assert_eq!(state.input.len(), 4095);
        assert_eq!(state.overrun, 1);
        assert!(!state.poll_readable());
        state.receive_sample(RxSample {
            byte: b'\n',
            status: 0x01,
        });
        assert!(state.poll_readable());
        assert_eq!(state.input.len(), 4096);

        let mut out = [0u8; 4096];
        let mut got = [0u8; 4096];
        let mut filled = 0;
        while filled < 4096 {
            let (length, eof) = state.peek_canonical(&mut out[..100.min(4096 - filled)]);
            assert!(!eof);
            assert!(0 < length);
            got[filled..filled + length].copy_from_slice(&out[..length]);
            state.consume_canonical(length, eof);
            filled += length;
        }
        assert!(got[..4095].iter().all(|&b| b == b'a'));
        assert_eq!(got[4095], b'\n');
        Ok(())
    }

    /// 验证规范模式下缓冲区内连续存放两条已提交行记录（`"hi\n"` 与 `"bye\n"`）时，单次读取严格在第一条记录末尾截断，不会越过行边界读取第二条记录。
    #[test]
    fn canonical_two_records_short_read(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        for &byte in b"hi\nbye\n" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        let mut out = [0u8; 16];
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (3, false));
        assert_eq!(&out[..length], b"hi\n");
        state.consume_canonical(length, eof);
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (4, false));
        assert_eq!(&out[..length], b"bye\n");
        Ok(())
    }

    /// 验证规范模式下输入 `"ab\x04"`（含 `VEOF` 提交）后切换至非规范模式（Raw Mode）时，仅交付数据字节 `"ab"`，已作为行尾定界符消费的 `VEOF` 不会变成额外的数据字节。
    #[test]
    fn data_eof_to_raw_retires_token(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        for &byte in b"ab\x04" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        state.set_canonical(false);
        let mut out = [0u8; 8];
        let peeked = state.peek_raw(&mut out);
        assert_eq!(peeked, 2);
        assert_eq!(&out[..peeked], b"ab");
        Ok(())
    }

    /// 验证规范模式下仅收到单个空 `VEOF`（`0x04`）后立即切换至非规范模式时，该空 `EOF` 标记被清理，非规范模式下无任何字节可读。
    #[test]
    fn empty_eof_to_raw_reads_nothing(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.receive_sample(RxSample {
            byte: 4,
            status: 0x01,
        });
        state.set_canonical(false);
        let mut out = [0u8; 8];
        assert_eq!(state.peek_raw(&mut out), 0);
        assert!(!state.poll_readable());
        Ok(())
    }

    /// 验证非规范模式下作为普通数据入队的 `0x04` 字节，在切换回规范模式后仍保持为普通未提交数据字节，不会被事后重新解释为 `VEOF`。
    #[test]
    fn raw_0x04_to_canonical_stays_data(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.lflag &= !ICANON;
        state.termios.iflag = 0;
        for &byte in b"a\x04b" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        state.set_canonical(true);
        let mut out = [0u8; 8];
        let (length, eof) = state.peek_canonical(&mut out);
        assert_eq!((length, eof), (3, false));
        assert_eq!(&out[..length], b"a\x04b");
        Ok(())
    }

    /// 验证在 `TCOOFF` 暂停串口输出并写满输出缓冲区导致写线程阻塞后，调用 `TCFLSH` 清空输出队列能够立即唤醒阻塞写者并使其写入新字节，随后 `TCOON` 恢复输出可将新字节通过 UART 通道发送至宿主。
    #[test]
    #[ktest(resources("uart"))]
    fn blocked_writer_wakes_on_output_flush(
        context: &mut TestContext<'_>,
        fixture: &mut TtyOwner,
    ) -> TestResult {
        fixture.open_tty(context, b"/ttyS0", 1)?;
        checked(context, crate::fs::tty::flush(2))?;
        checked(context, crate::fs::tty::flow(0))?;
        let file = fixture.tty(0).clone();
        let page = [b'w'; 1024];
        for _ in 0..4 {
            context.expect_eq(checked(context, file.write(&page))?, 1024);
        }

        let writer = file.clone();
        fixture.run(move || match writer.write(b"z") {
            Ok(1) => 0,
            _ => 1,
        })?;
        until("tty writer did not really block", || fixture.blocked())?;
        checked(context, crate::fs::tty::flush(1))?;
        fixture.join(context);

        fixture.stage.begin(context, b"", b"z", 0, 0)?;
        checked(context, crate::fs::tty::flow(1))?;
        fixture.stage.finish()?;
        Ok(())
    }

    /// 验证启用 `INPCK | PARMRK` 时奇偶校验错误与 Break 条件的三字节转义序列（`0xff 0x00 <byte>`）以及 `0xff` 双写转义（`0xff 0xff`），且当输入环剩余空间不足以容纳完整转义序列时原子拒绝整组字节并计入 `overrun`。
    #[test]
    fn parity_marking_is_atomic_at_capacity(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        use crate::fs::tty::termios::{INPCK, PARMRK};
        let mut state = NttyState::new();
        state.termios.iflag = INPCK | PARMRK;
        state.termios.lflag = 0;
        state.termios.oflag = 0;
        state.receive_sample(RxSample {
            byte: 0xff,
            status: 0x05,
        });
        state.receive_sample(RxSample {
            byte: 0,
            status: 0x11,
        });
        let mut bytes = [0; 4096];
        assert_eq!(state.peek_raw(&mut bytes), 6);
        assert_eq!(&bytes[..6], &[0xff, 0, 0xff, 0xff, 0, 0]);
        state.consume_raw(6);

        for _ in 0..4094 {
            state.receive_sample(RxSample {
                byte: b'a',
                status: 0x01,
            });
        }
        state.receive_sample(RxSample {
            byte: 0xff,
            status: 0x05,
        });
        assert_eq!(state.overrun, 1);
        assert_eq!(state.peek_raw(&mut bytes), 4094);
        assert!(bytes[..4094].iter().all(|&byte| byte == b'a'));
        state.consume_raw(4094);
        assert_eq!(state.peek_raw(&mut bytes), 0);
        Ok(())
    }

    /// 验证启用 `ISTRIP` 时，最高位为 1 的回车符（`0x8d`）和流控字符（`0x93` / `0x91`）先剥离最高位变为 `'\r'`、Ctrl-S、Ctrl-Q，随后再正常触发 `ICRNL` 换行映射与 `IXON` 流控暂停/恢复。
    #[test]
    fn strip_precedes_mapping_and_flow_control(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.iflag = ISTRIP | ICRNL | IXON;
        state.termios.lflag = ICANON;
        state.receive_sample(RxSample {
            byte: b'a',
            status: 0x01,
        });
        state.receive_sample(RxSample {
            byte: 0x8d,
            status: 0x01,
        });
        let mut bytes = [0; 4];
        assert_eq!(state.peek_canonical(&mut bytes), (2, false));
        assert_eq!(&bytes[..2], b"a\n");

        state.receive_sample(RxSample {
            byte: 0x93,
            status: 0x01,
        });
        assert!(state.stopped);
        state.receive_sample(RxSample {
            byte: 0x91,
            status: 0x01,
        });
        assert!(!state.stopped);
        assert_eq!(state.input.len(), 2);
        Ok(())
    }

    /// 验证当输入环中已有未读完的已提交旧记录（如 4 字节 `"old\n"`）占用空间时，新编辑行的最大数据长度仍以整个 4096 字节环的剩余空间减 1 计算，确保最后 1 个槽位始终保留给行尾换行符 `'\n'`。
    #[test]
    fn canonical_ending_slot_includes_old_records(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        let mut state = NttyState::new();
        state.termios.iflag = 0;
        state.termios.lflag = ICANON;
        for &byte in b"old\n" {
            state.receive_sample(RxSample { byte, status: 0x01 });
        }
        for _ in 0..4092 {
            state.receive_sample(RxSample {
                byte: b'n',
                status: 0x01,
            });
        }
        state.receive_sample(RxSample {
            byte: b'\n',
            status: 0x01,
        });
        assert_eq!(state.overrun, 1);
        let mut bytes = [0; 4096];
        assert_eq!(state.peek_canonical(&mut bytes), (4, false));
        assert_eq!(&bytes[..4], b"old\n");
        state.consume_canonical(4, false);
        assert_eq!(state.peek_canonical(&mut bytes), (4092, false));
        assert!(bytes[..4091].iter().all(|&byte| byte == b'n'));
        assert_eq!(bytes[4091], b'\n');
        state.consume_canonical(4092, false);
        assert!(!state.poll_readable());
        Ok(())
    }

    /// 验证启用 `ONOCR`（在列 0 处抑制回车符 `'\r'` 输出）时，即使 `VREPRINT` 重印快照中包含大量被抑制输出的 `'\r'`，重印游标仍能正常推进直至完成，不会卡死或丢失后续字符 `'Z'` 的回显。
    #[test]
    fn reprint_suppressed_carriage_returns_advance(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        use crate::fs::tty::termios::ONOCR;
        let mut state = NttyState::new();
        state.termios.iflag = 0;
        state.termios.oflag = OPOST | ONOCR;
        state.termios.lflag = ICANON | ECHO | IEXTEN;
        for _ in 0..200 {
            state.receive_sample(RxSample {
                byte: b'\r',
                status: 0x01,
            });
        }
        state.receive_sample(RxSample {
            byte: 18,
            status: 0x01,
        });
        state.receive_sample(RxSample {
            byte: b'Z',
            status: 0x01,
        });
        let mut output = Vec::new();
        loop {
            state.process_echo();
            while let Some(byte) = state.output.pop() {
                output.push(byte);
            }
            if state.echo.is_empty() && state.pending_reprint.is_none() {
                break;
            }
        }
        assert_eq!(output, [18, b'\n', b'Z']);
        Ok(())
    }

    /// 验证在没有任何进程打开 TTY 文件描述符的情况下，串口连续收到超过单次工作队列预算（64 项）的 65 字节输入时，后台工作队列仍会自动续调排空全部硬件接收字节，阶段结束时软件接收环为空。
    #[test]
    #[ktest(resources("uart"))]
    fn rx_without_open_continues_past_work_budget(
        context: &mut TestContext<'_>,
        fixture: &mut TtyOwner,
    ) -> TestResult {
        checked(context, crate::fs::tty::flush(2))?;
        fixture.stage.begin(context, &[b'z'; 65], b"", 0, 0)?;
        fixture.stage.finish()?;
        assert!(crate::console::testing::receive_queue_empty());
        Ok(())
    }

    /// 验证当最后一个打开的 TTY 文件描述符关闭时，此前已被行规程接收并生成的 4098 字节控制字符回显（2049 个 `"^A"`）在 `TCOON` 恢复输出后仍能通过 UART 通道完整发送给宿主。
    #[test]
    #[ktest(resources("uart"))]
    fn accepted_echo_finishes_after_last_close(
        context: &mut TestContext<'_>,
        fixture: &mut TtyOwner,
    ) -> TestResult {
        fixture.open_tty(context, b"/ttyS0", 2)?;
        let config = Termios {
            iflag: 0,
            oflag: 0,
            lflag: ICANON | ECHO | ECHOCTL,
            ..Termios::default()
        };
        checked(
            context,
            crate::fs::tty::set_termios(config, 2, FileIoContext::Kernel),
        )?;

        checked(context, crate::fs::tty::flow(0))?;
        let output = b"^A".repeat(2049);
        fixture.stage.begin(context, &[1; 2049], &output, 0, 0)?;
        until("TTY close-tail RX quota did not finish", || {
            crate::console::testing::receive_quota_remaining().is_none()
        })?;
        loop {
            crate::fs::tty::flush_work();
            if crate::console::testing::receive_queue_empty() {
                break;
            }
        }

        checked(context, crate::fs::tty::flush(0))?;
        fixture.release();
        checked(context, crate::fs::tty::flow(1))?;
        fixture.stage.finish()?;
        assert!(crate::console::transmitter_empty());
        Ok(())
    }

    /// 验证在启用 `PARMRK` 且将行结束符 `VEOL` / `VEOL2` 设为 `0xff` 时，行尾结束符所需的双字节转义（`0xff 0xff`）可使用输入环的最后两个槽位提交整行；若剩余空间不足 2 字节，则原子拒绝该结束符、递增 `overrun` 并保持已提交旧记录与当前未提交编辑行完好。
    #[test]
    fn parmrk_record_ending_uses_last_slot(
        _context: &mut TestContext<'_>,
        _fixture: &mut TtyOwner,
    ) -> TestResult {
        use crate::fs::tty::ntty::INPUT_CAPACITY;
        use crate::fs::tty::termios::{PARMRK, VEOL, VEOL2};

        let mut bytes = [0; INPUT_CAPACITY];
        for ending in [VEOL, VEOL2] {
            for prefix in [b"".as_slice(), b"old\n".as_slice()] {
                for extra in [0, 1] {
                    let mut state = NttyState::new();
                    state.termios.iflag = PARMRK;
                    state.termios.lflag = ICANON | if ending == VEOL2 { IEXTEN } else { 0 };
                    state.termios.cc[ending] = 0xff;
                    for &byte in prefix {
                        state.receive_sample(RxSample { byte, status: 0x01 });
                    }
                    let data_len = INPUT_CAPACITY - prefix.len() - 2 + extra;
                    for _ in 0..data_len {
                        state.receive_sample(RxSample {
                            byte: b'a',
                            status: 0x01,
                        });
                    }
                    assert_eq!(state.input.len(), prefix.len() + data_len);
                    state.receive_sample(RxSample {
                        byte: 0xff,
                        status: 0x01,
                    });
                    if extra == 0 {
                        assert_eq!(state.overrun, 0);
                        assert_eq!(state.input.len(), INPUT_CAPACITY);
                        assert_eq!(state.input.committed_len(), INPUT_CAPACITY);
                    } else {
                        assert_eq!(state.overrun, 1);
                        assert_eq!(state.input.len(), INPUT_CAPACITY - 1);
                        assert_eq!(state.input.committed_len(), prefix.len());
                    }

                    if !prefix.is_empty() {
                        assert_eq!(state.peek_canonical(&mut bytes), (prefix.len(), false));
                        assert_eq!(&bytes[..prefix.len()], prefix);
                        state.consume_canonical(prefix.len(), false);
                    }

                    if extra == 0 {
                        let record_len = data_len + 2;
                        assert_eq!(state.peek_canonical(&mut bytes), (record_len, false));
                        assert!(bytes[..data_len].iter().all(|&byte| byte == b'a'));
                        assert_eq!(&bytes[data_len..record_len], &[0xff, 0xff]);
                        state.consume_canonical(record_len, false);
                        assert!(state.input.is_empty());
                    } else {
                        assert_eq!(state.peek_canonical(&mut bytes), (0, false));
                        assert_eq!(state.input.edit_len(), data_len);
                        assert!((0..data_len).all(|offset| {
                            state.input.bytes[(state.input.tail + offset) % INPUT_CAPACITY] == b'a'
                        }));
                    }
                    assert!(!state.poll_readable());
                    assert!(state.echo.is_empty());
                }
            }
        }
        Ok(())
    }
}
