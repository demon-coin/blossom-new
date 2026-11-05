//! TTY 行规程的原生队列/定时/配置消费者与宿主 duplex 阶段。

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

/// 计算 Castagnoli CRC32C，不带最终取反由边界负责。
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

/// 从轮询路径读取一个控制字节。
fn control_byte() -> Result<u8, TestError> {
    until("tty UART control input timed out", || {
        crate::console::receive_pending()
    })?;
    Ok(crate::ktest::control_read_byte())
}

/// 宿主 duplex 阶段的真实拥有者，负责声明、quota 与 ACK。
/// 成功路径必须 begin→finish，finish 成功后才 inactive；body 错误或 fixture 退出时
/// 保持 active，不在 quiesce 中伪清或补读 ACK，原 case-abort 负责终止本 VM。
pub struct UartStage {
    /// 本阶段是否仍拥有 raw 通道。
    owned: bool,
}

impl UartStage {
    /// 建立未拥有任何阶段的对象。
    pub fn new() -> Self {
        Self { owned: false }
    }

    /// 声明本段 duplex；先同步旧 IRQ 全尾、核空环，再发送宿主声明并打开配额。
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
        crate::console::testing::claim_receive_control();
        crate::console::testing::synchronize_receive_irq()?;
        crate::fs::tty::flush_work();
        if !crate::console::testing::receive_queue_empty() {
            return Err(TestError("tty RX ring is not empty before stage"));
        }

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
        crate::console::testing::arm_receive_bytes(rx.len());
        Ok(())
    }

    /// quota 耗尽后排空环、drain、读 ACK，再按 CREAD 释放控制。
    pub fn finish(&mut self) -> TestResult {
        if !self.owned {
            return Err(TestError("tty UART stage is not active"));
        }
        until("tty RX quota was not consumed", || {
            crate::console::testing::receive_quota_remaining().is_none()
        })?;
        // 阶段边界允许重复 flush 至环空，Work 票前缀非全 RX 消完。
        loop {
            crate::fs::tty::flush_work();
            if crate::console::testing::receive_queue_empty() {
                break;
            }
        }
        crate::fs::tty::drain(FileIoContext::Kernel)
            .map_err(|_| TestError("tty drain failed before ACK"))?;
        crate::fs::tty::flush_work();

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

        let receiver = crate::fs::tty::termios().cflag & crate::fs::tty::termios::CREAD != 0;
        crate::console::testing::release_receive_control(receiver);
        self.owned = false;
        Ok(())
    }

    /// 返回本阶段是否仍拥有 raw 通道。
    pub fn active(&self) -> bool {
        self.owned
    }
}

impl Default for UartStage {
    /// 建立未拥有阶段的对象。
    fn default() -> Self {
        Self::new()
    }
}

/// 本组测试拥有唯一串口的 TTY 接收附着，常驻端点、Work 与 IRQ 仍由生产服务保活。
pub(super) struct SerialTtyOwner {
    attached: bool,
}

impl SerialTtyOwner {
    /// 在首例基线前同步旧接收尾部，恢复合法配置后按真实 CREAD 解除临时控制。
    pub(super) fn attach() -> Result<Self, TestError> {
        crate::console::testing::claim_receive_control();
        crate::console::testing::set_tty_receive(true)?;
        Self::finish_receive()?;
        let receiver = crate::fs::tty::termios().cflag & CREAD != 0;
        crate::console::testing::release_receive_control(receiver);
        Ok(Self { attached: true })
    }

    /// 全部任务、文件与 raw 阶段结束后停止新 TTY RX，真实排完已接受字节才恢复 Raw。
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

    /// RX producer 已同步停止后，等待环、Work、echo/output 与 TEMT，再清退休输入。
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

struct TtySuite {
    /// 本组真实接收附着的结束责任。
    serial: SerialTtyOwner,
    /// 同一 VM 的整组资源基线。
    memory: mm::MemoryStats,
    /// 同一 VM 的任务元数据基线。
    metadata: crate::workqueue::testing::MetadataStats,
}

impl ::ktest::Fixture for TtySuite {
    /// 常驻服务停稳并附着 TTY 接收后保存同一 VM 的整组资源。
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

    /// 全部 case 已释放自己的拥有后，真实停止本组接收并恢复 Raw。
    fn quiesce(&mut self, _context: &mut TestContext<'_>) -> TestResult {
        self.serial.stop()
    }

    /// 接收已恢复 Raw 后核第二层六域及任务元数据。
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

struct TtyOwner {
    /// 本例资源基线。
    before: mm::MemoryStats,
    /// 本例任务元数据基线。
    metadata: crate::workqueue::testing::MetadataStats,
    /// 真实子任务句柄。
    joins: Vec<JoinHandle>,
    /// 子任务弱身份观察。
    observer: Option<Arc<SpinLock<Option<Weak<Task>>>>>,
    /// 宿主阶段拥有者。
    stage: UartStage,
    /// 本例 devfs 视图。
    vfs: Option<Arc<crate::fs::Vfs>>,
    /// 本例文件系统上下文。
    context: Option<crate::fs::FsContext>,
    /// 本例打开的真实 TTY 文件。
    files: Vec<FileRef>,
}

impl TtyOwner {
    /// 显式 join 全部子任务并结束观察拥有。
    fn join(&mut self, context: &mut TestContext<'_>) {
        while let Some(join) = self.joins.pop() {
            context.expect_eq(join.join(), 0);
        }
        self.observer = None;
    }

    /// 发表记录弱身份的真实内核任务。
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

    /// 观察子任务是否真实阻塞。
    fn blocked(&self) -> bool {
        self.observer
            .as_ref()
            .and_then(|observer| observer.lock().as_ref().and_then(Weak::upgrade))
            .is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 建立本例独立 devfs 并保存视图与上下文。
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

    /// 用生产路径解析并打开真实 TTY，保存 FileRef 供正文调用。
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

    /// 借用本例已打开的 TTY。
    fn tty(&self, index: usize) -> &FileRef {
        &self.files[index]
    }

    /// 释放本例视图与文件，子任务已离栈后调用。
    fn release(&mut self) {
        self.files.clear();
        drop(self.context.take());
        drop(self.vfs.take());
    }
}

impl ::ktest::Fixture<TtySuite> for TtyOwner {
    /// 每例在任何临时对象准备前保存资源。
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

    /// 子任务离栈、阶段 inactive 后才释放本例文件与视图。
    fn quiesce(&mut self, context: &mut TestContext<'_>) -> TestResult {
        self.join(context);
        if self.stage.active() {
            return Err(TestError("tty stage is still active at quiesce"));
        }
        self.release();
        Ok(())
    }

    /// 全部拥有结束后核本例完整六域与元数据。
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

/// 父模块只保留此真实宏锚点。
pub(super) fn anchor() -> &'static ::ktest::SuiteSlot {
    cases::__ktest_anchor()
}

#[ktest::suite(fixture = TtyOwner, suite_resources = TtySuite, context = "Thread", tags("fs-tty"), timeout_ms = 45000)]
pub(crate) mod cases {
    use super::*;

    /// 四字段完整转换与 signed pid_t 边界直接消费正式 ABI codec。
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

    /// 信号效果保持 strip、literal-next、NOFLSH、BRKINT 与同批后继合法输入。
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

    /// Kernel 裸指针与无用户身份的 tty 打开只返回正式边界错误。
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

    /// 默认 termios 的 36 字节编解码与 Linux 默认值一致。
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

    /// 未选择的延迟、大小写与流控位返回真实 EINVAL。
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

    /// 规范记录的换行提交语义正确。
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

    /// 空 EOF 只产生一次零 read，含内容 EOF 提交内容不交控制字节。
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

    /// ERASE 擦除单字节。
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

    /// IUTF8 擦除只回退真实字符边界。
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

    /// CR/LF 映射与 ISTRIP 语义正确。
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

    /// Ctrl-S 停输出但 RX 继续，Ctrl-Q 恢复。
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

    /// 输出 ONLCR 扩展 LF 为两字节。
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

    /// 输出 TAB3 按列展开，满时不消费输入。
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

    /// 连续 VDISCARD 在停发且不泵输出时立即退休旧待发状态，恢复后保输入顺序和新输出。
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

        // 旧回显与快照由真实输入生成；关闭后继回显，单独观察丢弃边界的退休效果。
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

    /// poll 规范未提交不就绪，提交后就绪。
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

    /// poll 非规范 MIN 门限。
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

    /// IMAXBEL 在输入满时响铃，overrun 计数真实。
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

    /// BRKINT 本地清空输入与回显，不冒称信号递送。
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

    /// 真实打开 /ttyS0 与 /console，以设备号和操作错误核对各自文件语义。
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
        // Kernel 上下文对已识别命令报 InvalidOperation，不解引用裸指针。
        assert_eq!(
            first.ioctl(0x5401, 0, FileIoContext::Kernel).err(),
            Some(crate::fs::FsError::InvalidOperation)
        );
        Ok(())
    }

    /// 4095 长行 VKILL 按列差生成精确退格，后继 y 有序。
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

    /// 4090 行 REPRINT 操作留头，后继 Z 入尾，展开保序。
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

    /// 满队 VEOF 挂起，短读腾格即发表，新 Z 后于旧记录与 EOF。
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

        // 短读 1 腾格，EOF 即发表，后继 Z 仍满应 overrun。
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

        // 读完旧记录余部，EOF 零记录在前，新 Z 换行后交付。
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

    /// 真 raw 满队 4096 字节可读，NUL/FF/控制字节原样交付。
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

    /// canonical 4095 数据+LF 满行，短读拼回含 LF 完整行。
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
        // 第 4096 个 a 超 4095 数据上限计 overrun，不入环。
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

    /// 两条记录短读不越界。
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

    /// 数据+EOF 切 raw 后只交付数据字节，已解释 VEOF 不变 NUL。
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

    /// 空 EOF 切 raw 后无数据可读。
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

    /// raw 0x04 切 canonical 后保持数据字节，不重解释为 EOF。
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

    /// TCOOFF 下写满阻塞，TCFLSH 输出唤醒阻塞写者，TCOON 后排空。
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

    /// 校验错误与break标记完整保序，近满环不发表半个错误序列。
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

    /// 高位CR和STOP/START先strip，再执行映射或流控解释。
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

    /// 已提交旧记录占用容量时，新编辑尾仍为实际LF保留总环末格。
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

    /// ONOCR抑制重显中的CR时快照游标仍完成，后继字节不丢失。
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

    /// 没有打开OFD时仍跨64项预算退休真实RX，结束阶段核软件环为空。
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

    /// 末OFD关闭后已接受的4098字节回显尾部仍通过真实UART完整发送。
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

    /// PARMRK 的 VEOL/VEOL2 扩展可用总环末两格，容量不足时整组拒存且保留旧记录。
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
