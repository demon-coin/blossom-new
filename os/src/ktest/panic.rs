//! 测试执行栈上的无堆 DWARF 捕获边界；异常只沿同一 Rust 调用链传播。
use core::cell::UnsafeCell;
use core::fmt::{self, Write};
use core::marker::PhantomPinned;
use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::pin::pin;
use core::ptr;
use unwinding::abi::{UnwindException, UnwindReasonCode};
use unwinding::panicking::{self, Exception};

const TRUNCATED: &str = " [truncated]";
const PANIC_MESSAGE_CAPACITY: usize = 512;
const PANIC_FILE_CAPACITY: usize = 256;

struct PanicText<const N: usize> {
    bytes: [u8; N],
    len: usize,
    truncated: bool,
}

impl<const N: usize> PanicText<N> {
    /// 初始化固定消息空间，不依赖内核堆或启动阶段。
    const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            truncated: false,
        }
    }

    /// 只借出已经完整写入且始终保持 UTF-8 边界的前缀。
    fn as_str(&self) -> &str {
        // SAFETY: write_str 只复制 UTF-8 完整前缀及 ASCII 标记，len 从不超过 N。
        unsafe { core::str::from_utf8_unchecked(&self.bytes[..self.len]) }
    }

    /// 保留现有有效前缀并追加截断标记，容量不足和格式化失败共用这一终态。
    fn mark_truncated(&mut self) {
        if self.truncated {
            return;
        }
        let capacity = N - TRUNCATED.len();
        if capacity < self.len {
            let mut end = capacity;
            while !self.as_str().is_char_boundary(end) {
                end -= 1;
            }
            self.len = end;
        }
        let end = self.len + TRUNCATED.len();
        self.bytes[self.len..end].copy_from_slice(TRUNCATED.as_bytes());
        self.len = end;
        self.truncated = true;
    }
}

impl<const N: usize> Write for PanicText<N> {
    /// 在首次溢出时保留 UTF-8 前缀和显式截断标记，后续片段不替换原始消息。
    fn write_str(&mut self, text: &str) -> fmt::Result {
        if self.truncated {
            return Ok(());
        }
        if text.len() <= N - self.len {
            let end = self.len + text.len();
            self.bytes[self.len..end].copy_from_slice(text.as_bytes());
            self.len = end;
            return Ok(());
        }
        // 两处实例容量均大于标记长度；截断已有前缀也不能截断多字节字符。
        let capacity = N - TRUNCATED.len();
        if capacity < self.len {
            let mut end = capacity;
            while !self.as_str().is_char_boundary(end) {
                end -= 1;
            }
            self.len = end;
        }
        let mut take = (capacity - self.len).min(text.len());
        while !text.is_char_boundary(take) {
            take -= 1;
        }
        let end = self.len + take;
        self.bytes[self.len..end].copy_from_slice(&text.as_bytes()[..take]);
        self.len = end;
        self.mark_truncated();
        Ok(())
    }
}

/// 已被真实展开器归还的 panic 诊断；不保留 PanicInfo 或原格式化参数的借用。
pub struct PanicRecord {
    message: PanicText<PANIC_MESSAGE_CAPACITY>,
    file: PanicText<PANIC_FILE_CAPACITY>,
    pub line: u32,
    pub column: u32,
}

impl PanicRecord {
    /// 预备独立的有界消息及源位置缓冲，未发生 panic 时不格式化。
    const fn new() -> Self {
        Self {
            message: PanicText::new(),
            file: PanicText::new(),
            line: 0,
            column: 0,
        }
    }

    /// 返回原消息的 UTF-8 前缀；被截断时末尾包含显式标记。
    pub fn message(&self) -> &str {
        self.message.as_str()
    }

    /// 返回复制的源文件名，缺少源位置时为空。
    pub fn file(&self) -> &str {
        self.file.as_str()
    }

    /// 表示原始 panic 消息未完整保存，预期消息匹配必须拒绝把该前缀当成完整值。
    pub fn message_truncated(&self) -> bool {
        self.message.truncated
    }

    /// 表示源文件名被截断，不影响已经保存的行列号。
    pub fn file_truncated(&self) -> bool {
        self.file.truncated
    }

    /// 只对完整原消息作字节相等匹配，不允许截断前缀满足预期 panic。
    pub fn matches_message(&self, expected: &str) -> bool {
        !self.message.truncated && self.message() == expected
    }
}

impl fmt::Debug for PanicRecord {
    /// 诊断只展示有效文本和源位置，不展开固定缓冲中的未使用尾部。
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("PanicRecord")
            .field("message", &self.message())
            .field("message_truncated", &self.message_truncated())
            .field("file", &self.file())
            .field("file_truncated", &self.file_truncated())
            .field("line", &self.line)
            .field("column", &self.column)
            .finish()
    }
}

/// 固定地址的真实异常头；初始化和单次移交由其所属 Boundary 控制。
#[repr(C, align(16))]
pub struct ExceptionStorage {
    header: MaybeUninit<UnwindException>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Armed,
    Recording,
    InFlight,
    Returned,
    Finished,
}

struct BoundaryState {
    previous: *mut Boundary,
    phase: Phase,
    exception: ExceptionStorage,
    record: MaybeUninit<PanicRecord>,
    fp: MaybeUninit<FpState>,
}

/// catch 栈帧中被 pin 的边界；父定位器只保存其裸指针，不能移动或访问内部状态。
pub struct Boundary {
    state: UnsafeCell<BoundaryState>,
    _pinned: PhantomPinned,
}

impl Boundary {
    /// 每层嵌套捕获预备自己的存储，直到异常已归还才撤下定位器中的指针。
    fn new(previous: *mut Boundary) -> Self {
        Self {
            state: UnsafeCell::new(BoundaryState {
                previous,
                phase: Phase::Armed,
                exception: ExceptionStorage {
                    header: MaybeUninit::uninit(),
                },
                record: MaybeUninit::uninit(),
                fp: MaybeUninit::uninit(),
            }),
            _pinned: PhantomPinned,
        }
    }
}

struct ExceptionToken(*mut Boundary);

// SAFETY: token 只能由 raise 为当前被 pin 的 Boundary 创建；wrap 交出该边界内
// 唯一真实 header，unwrap 先校验当前边界和同一指针，再归还一次。异常存活时边界不移动。
unsafe impl Exception for ExceptionToken {
    const CLASS: [u8; 8] = *b"BLSMKTS1";

    /// 将已经初始化的唯一异常头交给展开器，保持原指针与异常存储地址一致。
    fn wrap(this: Self) -> *mut UnwindException {
        if this.0 != crate::ktest::current_boundary() || this.0.is_null() {
            crate::ktest::fatal("panic exception owner mismatch");
        }
        // SAFETY: token 私有构造且定位器仍指向同一存活的 pin 边界；无跨调用保留的 &mut。
        unsafe {
            let state = &mut *(*this.0).state.get();
            if state.phase != Phase::Recording {
                crate::ktest::fatal("panic exception slot reentry");
            }
            state.phase = Phase::InFlight;
            state.exception.header.as_mut_ptr()
        }
    }

    /// 展开器只归还当前异常的原始 header，不能由 payload 或未知异常猜测所属对象。
    ///
    /// # Safety
    /// exception 必须是展开器归还的 CLASS 匹配异常指针，且没有并发使用者。
    unsafe fn unwrap(exception: *mut UnwindException) -> Self {
        let boundary = crate::ktest::current_boundary();
        if boundary.is_null() {
            crate::ktest::fatal("panic exception without boundary");
        }
        // SAFETY: 定位器保证当前指针只指向存活的 pin 边界；比较 header 地址之前
        // 不解引用外来 exception，InFlight 到 Returned 转移拒绝重复归还。
        unsafe {
            let state = &mut *(*boundary).state.get();
            if state.phase != Phase::InFlight || exception != state.exception.header.as_mut_ptr() {
                crate::ktest::fatal("panic exception return mismatch");
            }
            state.phase = Phase::Returned;
        }
        Self(boundary)
    }
}

/// 捕获当前 Boot/Task 栈内同步 Rust panic，并完整执行到该边界之间的局部 Drop。
/// 主体可在正常阶段迁核；捕获必须位于 C/trap/Task 入口之内，展开期间禁止调度。
/// 父定位器负责排除 IRQ/IPI、未知服务任务和 Boot 非 BSP 调用。
#[expect(
    clippy::result_large_err,
    reason = "Boot 捕获及真实 OOM 的 panic 路径必须无堆保存固定诊断，装箱会引入分配失败并破坏展开回收"
)]
pub fn catch<T>(body: impl FnOnce() -> T) -> Result<T, PanicRecord> {
    if is_unwinding() {
        crate::ktest::fatal("nested catch during panic unwind");
    }
    let boundary = pin!(Boundary::new(crate::ktest::current_boundary()));
    let pointer = boundary.as_ref().get_ref() as *const Boundary as *mut Boundary;
    // SAFETY: pin 栈临时量持续存活到 catch 返回；定位器必须按当前 Boot/Task 唯一保存。
    let previous = unsafe { crate::ktest::replace_boundary(pointer) };
    // SAFETY: 当前边界已经 pin 且通过 UnsafeCell 独占状态；此引用不跨 body 调用。
    if previous != unsafe { (*(*pointer).state.get()).previous } {
        crate::ktest::fatal("panic boundary installation mismatch");
    }
    let result = panicking::catch_unwind::<ExceptionToken, T, _>(body);
    if crate::ktest::current_boundary() != pointer {
        crate::ktest::fatal("panic boundary stack mismatch");
    }
    // SAFETY: body 与展开器均已返回；异常生命周期覆盖的 pin 存储仍在本栈帧，
    // 没有其他线程使用该边界，Rust 引用只在本次处理内存在。
    let result = unsafe {
        let state = &mut *(*pointer).state.get();
        let result = match result {
            Ok(value) if state.phase == Phase::Armed => Ok(value),
            Err(Some(token)) if token.0 == pointer && state.phase == Phase::Returned => {
                state.fp.assume_init_read().restore();
                Err(state.record.assume_init_read())
            }
            _ => crate::ktest::fatal("panic unwind did not return owned exception"),
        };
        state.phase = Phase::Finished;
        result
    };
    // SAFETY: 当前边界已归还异常且不再参与展开；previous 是仍存活的外层 pin 边界或空。
    if unsafe { crate::ktest::replace_boundary(previous) } != pointer {
        crate::ktest::fatal("panic boundary removal mismatch");
    }
    result
}

/// 查询当前真实边界的展开状态，供调度入口拒绝清理期间的 yield/park。
pub fn is_unwinding() -> bool {
    let pointer = crate::ktest::current_boundary();
    if pointer.is_null() {
        return false;
    }
    // SAFETY: 父定位器只返回当前执行者存活的 pin 边界，不返回其他 Task 或 trap 的指针。
    let phase = unsafe { (*(*pointer).state.get()).phase };
    matches!(phase, Phase::Recording | Phase::InFlight | Phase::Returned)
}

/// 为当前边界记录固定诊断并交给真实 DWARF 展开器；第二次 panic 永远不可恢复。
/// 调用者必须是允许展开的 Rust panic handler，定位器不允许跨 trap/IRQ/切栈使用边界。
pub fn raise(info: &PanicInfo<'_>) -> ! {
    let boundary = crate::ktest::current_boundary();
    if boundary.is_null() {
        let kind = if info.can_unwind() { "panic" } else { "abi" };
        crate::ktest::fatal_event(kind, format_args!("uncaught panic: {}", info));
    }
    // SAFETY: 定位器给出当前稳定边界；只通过裸指针读取独立 phase 字段，
    // 不创建覆盖 record 的 &mut，因此 Display 格式化中的递归 panic 仍能安全终止。
    let state = unsafe { (*boundary).state.get() };
    // Rust 的 C 边界在已有展开中报告 can_unwind=false；真实 ABI 错误不能误归为普通双 panic。
    if !info.can_unwind() {
        crate::ktest::fatal("panic cannot unwind across ABI boundary");
    }
    // SAFETY: phase 不与格式化器借用的 record 重叠；当前执行者独占该非原子状态。
    if unsafe { ptr::addr_of!((*state).phase).read() } != Phase::Armed {
        crate::ktest::fatal("double panic during unwind");
    }
    // SAFETY: Armed 表示没有活动异常或 record 借用；先标记 Recording 再准备消息。
    unsafe {
        (*state).phase = Phase::Recording;
        (*state).record.write(PanicRecord::new());
    }
    // 格式化可能调用被测 Display；状态已先标记，递归 panic 不能覆盖第一份记录。
    // SAFETY: 只在当前边界格式化一次；递归 panic 只检查 phase 并终止，不重借 record。
    unsafe {
        let record = (*(*boundary).state.get()).record.assume_init_mut();
        if write!(record.message, "{}", info.message()).is_err() {
            record.message.mark_truncated();
        }
        if let Some(location) = info.location() {
            let _ = record.file.write_str(location.file());
            record.line = location.line();
            record.column = location.column();
        }
    }
    // 仅在开始展开时启用目标 ABI 所需 FP；正常主体维持原 FP Off 调度约束。
    // SAFETY: 边界已处于 Recording，调度必须拒绝该状态，随后不再允许迁核。
    unsafe {
        let state = &mut *(*boundary).state.get();
        state.fp.write(FpState::enable());
        // UnwindException 中整数、Option<fn> 及其私有字段的零位模式均有效。
        ptr::write_bytes(state.exception.header.as_mut_ptr(), 0, 1);
    }
    let code = panicking::begin_panic(ExceptionToken(boundary));
    // 正常路径从 landing pad 返回 catch；begin_panic 返回表示查找/清理失败，不能恢复主体。
    match code {
        UnwindReasonCode::END_OF_STACK => crate::ktest::fatal("panic unwind reached end of stack"),
        UnwindReasonCode::FATAL_PHASE1_ERROR => crate::ktest::fatal("panic unwind phase1 failed"),
        _ => crate::ktest::fatal("panic unwind phase2 failed"),
    }
}

struct FpState {
    control: usize,
    fcsr: usize,
}

impl FpState {
    /// 暂时启用真实目标的浮点单元并保存控制状态，不改变目标 ABI 或 per-CPU 寄存器。
    fn enable() -> Self {
        #[cfg(target_arch = "riscv64")]
        {
            let control: usize;
            let fcsr: usize;
            // SAFETY: 本核 S-mode 可以设置 FS；调度已由 Recording 状态禁止，
            // 先开启 FS 才读取 FCSR，不改 SIE 或当前 CPU 的 tp。
            unsafe {
                core::arch::asm!("csrrs {control}, sstatus, {mask}",
                    control = out(reg) control, mask = in(reg) crate::arch::SR_FS, options(nostack));
                core::arch::asm!("frcsr {fcsr}", fcsr = out(reg) fcsr, options(nostack));
            }
            Self {
                control: control & crate::arch::SR_FS,
                fcsr,
            }
        }
        #[cfg(target_arch = "loongarch64")]
        {
            let control: usize;
            let fcsr: usize;
            let mut enable = crate::arch::EUEN_FP;
            // SAFETY: PLV0 使用掩码只开启 EUEN.FPE；展开不能调度，KS0 不变。
            // FCSR 在 FPE 开启后才读取，不改变其他扩展单元或 IRQ 开关。
            unsafe {
                core::arch::asm!("csrxchg {enable}, $t0, {euen}",
                    enable = inout(reg) enable, euen = const crate::arch::CSR_EUEN,
                    in("$t0") crate::arch::EUEN_FP, options(nostack));
                control = enable & crate::arch::EUEN_FP;
                core::arch::asm!("movfcsr2gr {fcsr}, $fcsr0", fcsr = out(reg) fcsr, options(nostack));
            }
            Self { control, fcsr }
        }
    }

    /// 在 catch 已拿回异常之后恢复原 FP 控制位，不能恢复展开前的 IRQ 状态或旧 CPU 基址。
    fn restore(self) {
        #[cfg(target_arch = "riscv64")]
        // SAFETY: raise 已启用 FS，当前仍是同 CPU 的展开尾部；仅恢复 FCSR/FS，
        // 其余 sstatus 位保留 Drop 已完成的 IRQ/锁状态变化。
        unsafe {
            core::arch::asm!("fscsr {fcsr}", fcsr = in(reg) self.fcsr, options(nostack));
            core::arch::asm!("csrc sstatus, {mask}", "csrs sstatus, {control}",
                mask = in(reg) crate::arch::SR_FS, control = in(reg) self.control, options(nostack));
        }
        #[cfg(target_arch = "loongarch64")]
        // SAFETY: FPE 仍开启，FCSR 先恢复；EUEN 只交换 FPE，KS0 和 IRQ 不变。
        unsafe {
            core::arch::asm!("movgr2fcsr $fcsr0, {fcsr}", fcsr = in(reg) self.fcsr, options(nostack));
            core::arch::asm!("csrxchg {control}, $t0, {euen}",
                control = inout(reg) self.control => _, euen = const crate::arch::CSR_EUEN,
                in("$t0") crate::arch::EUEN_FP, options(nostack));
        }
    }
}
