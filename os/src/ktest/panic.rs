//! 测试执行栈上的零堆分配 DWARF 栈展开捕获边界（`Boundary`）。
//!
//! # 模块职责与架构定位
//! 本模块为内核测试框架提供同步 Rust `panic!` 的捕获与恢复机制（`catch` 与 `raise`）：
//! - 上游调用者：`ktest::context::RuntimeServices`、子任务入口包装闭包以及全局 `#[panic_handler]`。
//! - 下游依赖：`unwinding` crate（基于 DWARF `.eh_frame` 的两阶段栈展开器）与 `ktest::context`
//!   （按当前 `Boot` 或 `Task` 槽位存取栈顶 `Boundary` 指针）。
//!
//! # 核心设计与状态机
//! ## 1. 栈上固定容量异常存储（零堆依赖）
//! 在 Boot 早期阶段或模拟内存耗尽（OOM）的测试用例中，触发 `panic!` 时内核堆可能不可用。
//! 因此 [`Boundary`] 直接固定（Pin）在 `catch` 调用栈帧上，内联包含：
//! - 16 字节对齐的 Itanium ABI 异常头 [`ExceptionStorage`]（`UnwindException`）；
//! - 固定容量为 512 字节的消息缓冲与 256 字节的文件名缓冲 [`PanicRecord`]；
//! - 架构相关的浮点控制与状态寄存器快照 [`FpState`]。
//!
//! ## 2. 捕获边界状态机（`Phase`）
//! ```text
//!   catch() 在栈上 Pin 住 Boundary 并通过 replace_boundary() 压入当前执行流槽位
//!        |
//!        v
//!   Phase::Armed  ---(主体正常返回 Ok)---> Phase::Finished ---> 弹出 Boundary 并返回 Ok(T)
//!        |
//!        | 触发 raise(PanicInfo)
//!        v
//!   Phase::Recording : 格式化消息到栈上 PanicRecord，开启硬件 FPU 并保存原 FpState，清零异常头
//!        |
//!        | panicking::begin_panic(ExceptionToken) -> Exception::wrap()
//!        v
//!   Phase::InFlight  : DWARF 展开器沿调用栈逐帧执行局部变量 Drop（此期间 is_unwinding()==true，禁止调度与嵌套 catch）
//!        |
//!        | 命中 catch_unwind 着陆垫 (Landing Pad) -> Exception::unwrap()
//!        v
//!   Phase::Returned  : 校验归还的异常头地址匹配，恢复原 FpState 并读取 PanicRecord
//!        |
//!        v
//!   Phase::Finished  : 弹出 Boundary 恢复外层边界指针，向调用者返回 Err(PanicRecord)
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 每个 [`Boundary`] 严格归属于创建它的执行栈帧，通过 `ktest::current_boundary` 仅对当前执行流可见，无跨核并发访问。
//! - 当处于 `Recording`、`InFlight` 或 `Returned` 阶段时，`is_unwinding()` 返回 `true`；调度器在入口处检查此标志，
//!   禁止在栈展开执行 `Drop` 期间发生线程切换、休眠或跨核迁移。
use core::cell::UnsafeCell;
use core::fmt::{self, Write};
use core::marker::PhantomPinned;
use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::pin::pin;
use core::ptr;
use unwinding::abi::{UnwindException, UnwindReasonCode};
use unwinding::panicking::{self, Exception};

/// 消息或源文件路径超出固定缓冲区容量时追加在末尾的 ASCII 截断后缀。
const TRUNCATED: &str = " [truncated]";
/// 单条 panic 格式化消息的最大字节容量（含截断后缀）。
const PANIC_MESSAGE_CAPACITY: usize = 512;
/// 触发 panic 的源文件路径最大字节容量（含截断后缀）。
const PANIC_FILE_CAPACITY: usize = 256;

/// 固定容量、保证 UTF-8 字符边界完整的栈上文本缓冲区。
struct PanicText<const N: usize> {
    /// 固定大小的字节存储数组，仅前 `len` 个字节为有效 UTF-8 文本。
    bytes: [u8; N],
    /// 当前已写入的有效 UTF-8 字节长度（始终满足 `len <= N`）。
    len: usize,
    /// 标记写入内容是否因超出容量 `N` 或格式化错误而被截断。
    truncated: bool,
}

impl<const N: usize> PanicText<N> {
    /// 创建一个空的固定容量文本缓冲区，不依赖内核堆分配。
    const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            truncated: false,
        }
    }

    /// 获取当前已写入的有效 UTF-8 字符串切片。
    fn as_str(&self) -> &str {
        // SAFETY:
        // 1. `write_str` 和 `mark_truncated` 在写入或回退截断时均通过 `is_char_boundary` 保证了 UTF-8 字符边界完整；
        // 2. `self.len` 始终不超过数组容量 `N`。
        unsafe { core::str::from_utf8_unchecked(&self.bytes[..self.len]) }
    }

    /// 将缓冲区标记为已截断，并在保留有效 UTF-8 前缀的前提下在末尾追加 [`TRUNCATED`] 标记。
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
    /// 向固定容量缓冲区追加字符串片段；若剩余空间不足，则按 UTF-8 字符边界截断并追加 [`TRUNCATED`]。
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
        // Step 1: 预留出末尾 TRUNCATED 标记所需的字节数，若现有已写入长度超出可用容量则回退到最近的 UTF-8 字符边界。
        let capacity = N - TRUNCATED.len();
        if capacity < self.len {
            let mut end = capacity;
            while !self.as_str().is_char_boundary(end) {
                end -= 1;
            }
            self.len = end;
        }
        // Step 2: 从本次新写入的 text 中截取能够放入剩余空间的最大完整 UTF-8 前缀，随后追加截断标记。
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

/// 由栈展开器捕获并归还的独立 panic 诊断记录。
///
/// 所有文本均已复制保存在固定容量的栈缓冲内，不借用 `PanicInfo` 或原始格式化参数的生命周期。
pub struct PanicRecord {
    /// 格式化后的 panic 消息文本缓冲。
    message: PanicText<PANIC_MESSAGE_CAPACITY>,
    /// 触发 panic 的源文件路径文本缓冲。
    file: PanicText<PANIC_FILE_CAPACITY>,
    /// 触发 panic 的源代码行号（无位置信息时为 0）。
    pub line: u32,
    /// 触发 panic 的源代码列号（无位置信息时为 0）。
    pub column: u32,
}

impl PanicRecord {
    /// 创建空的 [`PanicRecord`] 缓冲区。
    const fn new() -> Self {
        Self {
            message: PanicText::new(),
            file: PanicText::new(),
            line: 0,
            column: 0,
        }
    }

    /// 返回捕获到的 panic 消息字符串（若发生过截断，末尾包含 `" [truncated]"`）。
    pub fn message(&self) -> &str {
        self.message.as_str()
    }

    /// 返回触发 panic 的源文件路径（若无位置信息则为空字符串）。
    pub fn file(&self) -> &str {
        self.file.as_str()
    }

    /// 返回 panic 消息是否因超出 [`PANIC_MESSAGE_CAPACITY`] 或格式化失败而被截断。
    pub fn message_truncated(&self) -> bool {
        self.message.truncated
    }

    /// 返回源文件路径是否因超出 [`PANIC_FILE_CAPACITY`] 而被截断。
    pub fn file_truncated(&self) -> bool {
        self.file.truncated
    }

    /// 检查捕获到的 panic 消息是否未经截断且与 `expected` 完全一致。
    ///
    /// 若原始消息曾发生截断，即使截断后的前缀与 `expected` 相同也会返回 `false`。
    pub fn matches_message(&self, expected: &str) -> bool {
        !self.message.truncated && self.message() == expected
    }
}

impl fmt::Debug for PanicRecord {
    /// 格式化有效消息文本与源代码行列号，忽略固定缓冲区尾部未使用的字节。
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

/// 满足 Itanium Unwind ABI 16 字节对齐要求的栈上异常头存储槽。
#[repr(C, align(16))]
pub struct ExceptionStorage {
    /// 由 [`Boundary`] 在触发展开时清零初始化并移交给 `unwinding` 库的底层异常头。
    header: MaybeUninit<UnwindException>,
}

/// 栈上 panic 捕获边界的内部生命周期阶段。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// 边界已安装到当前执行流，等待测试主体正常返回或触发首次 panic。
    Armed,
    /// 正在 `raise` 中向边界写入 [`PanicRecord`] 并初始化异常头（此阶段再次触发 panic 将直接判定为 double panic）。
    Recording,
    /// 异常头已通过 [`ExceptionToken::wrap`] 移交给 DWARF 展开器，正在沿栈帧执行 `Drop`。
    InFlight,
    /// DWARF 展开器已到达 `catch_unwind` 着陆垫并通过 [`ExceptionToken::unwrap`] 归还异常头。
    Returned,
    /// `catch` 已完成状态回收与浮点寄存器恢复，准备卸载边界并返回结果。
    Finished,
}

/// [`Boundary`] 内部由 [`UnsafeCell`] 包装的可变状态。
struct BoundaryState {
    /// 安装本边界前保存的外层捕获边界指针，用于支持嵌套 `catch` 并在退出时恢复。
    previous: *mut Boundary,
    /// 当前捕获边界所处的展开状态机阶段。
    phase: Phase,
    /// 供底层 DWARF 展开器使用的内联异常头存储。
    exception: ExceptionStorage,
    /// 发生 panic 时在 `Recording` 阶段写入的诊断记录。
    record: MaybeUninit<PanicRecord>,
    /// 开始栈展开前保存的硬件浮点控制与状态寄存器快照。
    fp: MaybeUninit<FpState>,
}

/// 固定（Pinned）在 `catch` 调用栈帧上的无堆 panic 捕获边界。
///
/// 上下文定位器（`ktest::context`）仅保存指向本结构体的裸指针；结构体自身不可移动，且仅由当前执行流在受控阶段访问。
pub struct Boundary {
    /// 边界内部状态槽。
    state: UnsafeCell<BoundaryState>,
    /// 标记结构体为 `!Unpin`，确保其在栈帧上的内存地址在 `catch` 期间恒定不变。
    _pinned: PhantomPinned,
}

impl Boundary {
    /// 创建处于 [`Phase::Armed`] 初始状态的捕获边界，并记录外层边界指针 `previous`。
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

/// 传递给 `unwinding::panicking` 的异常所有权令牌（元组字段 `0` 为指向持有该异常头的栈上 [`Boundary`] 裸指针）。
struct ExceptionToken(*mut Boundary);

// SAFETY:
// 1. `ExceptionToken` 仅由 `raise` 针对当前栈顶已固定（Pinned）的 `Boundary` 构造；
// 2. `wrap` 校验令牌与当前活跃边界一致，将状态从 `Recording` 推进到 `InFlight` 并交出内联 `UnwindException` 指针；
// 3. `unwrap` 在着陆垫校验异常头地址与 `InFlight` 状态匹配后将其推进到 `Returned`，整个展开期间栈帧不会移动。
unsafe impl Exception for ExceptionToken {
    const CLASS: [u8; 8] = *b"BLSMKTS1";

    /// 将当前边界内已清零初始化的 [`UnwindException`] 裸指针移交给底层展开器。
    fn wrap(this: Self) -> *mut UnwindException {
        if this.0 != crate::ktest::current_boundary() || this.0.is_null() {
            crate::ktest::fatal("panic exception owner mismatch");
        }
        // SAFETY:
        // 1. 前置检查已确认 `this.0` 非空且等于当前执行流栈顶安装的 `Boundary`；
        // 2. 当前处于同步展开启动路径，没有其他可变引用同时访问 `state`，可变借用在函数返回前即结束。
        unsafe {
            let state = &mut *(*this.0).state.get();
            if state.phase != Phase::Recording {
                crate::ktest::fatal("panic exception slot reentry");
            }
            state.phase = Phase::InFlight;
            state.exception.header.as_mut_ptr()
        }
    }

    /// 在着陆垫收回展开器归还的异常头指针，校验其归属于当前栈顶边界后还原为 [`ExceptionToken`]。
    ///
    /// # Safety
    /// `exception` 必须是由底层展开器针对匹配 `CLASS` 归还的异常头指针，且此时展开器不再并发访问该异常头。
    unsafe fn unwrap(exception: *mut UnwindException) -> Self {
        let boundary = crate::ktest::current_boundary();
        if boundary.is_null() {
            crate::ktest::fatal("panic exception without boundary");
        }
        // SAFETY:
        // 1. `current_boundary()` 返回当前执行流栈帧上存活且已固定的 `Boundary` 指针；
        // 2. 仅比较 `exception` 地址是否等于本边界内联的 `header` 地址，不解引用未知外部指针；
        // 3. `InFlight -> Returned` 单向状态转换确保同一个异常头不会被重复归还。
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

/// 在当前 Boot 或 Task 执行栈上安装捕获边界并执行 `body`，捕获同步触发的 Rust `panic!`。
///
/// - 测试主体在正常执行期间允许跨核迁移；
/// - 若触发 `panic!`，DWARF 展开器将沿调用栈完整执行到本边界之间的局部变量 `Drop`，展开期间禁止调度切换；
/// - 中断处理上下文（IRQ/IPI）及未绑定测试作用域的后台线程由 `ktest::context` 排除，不允许进入捕获展开。
#[expect(
    clippy::result_large_err,
    reason = "Boot 捕获及真实 OOM 的 panic 路径必须无堆保存固定诊断，装箱会引入分配失败并破坏展开回收"
)]
pub fn catch<T>(body: impl FnOnce() -> T) -> Result<T, PanicRecord> {
    // Step 1: 拒绝在栈展开清理（Drop）期间嵌套发起新的 catch。
    if is_unwinding() {
        crate::ktest::fatal("nested catch during panic unwind");
    }

    // Step 2: 在当前栈帧上 Pin 住新的 Boundary，并将其安装到当前执行流的捕获槽位。
    let boundary = pin!(Boundary::new(crate::ktest::current_boundary()));
    let pointer = boundary.as_ref().get_ref() as *const Boundary as *mut Boundary;
    // SAFETY:
    // 1. `boundary` 已通过 `pin!` 固定在当前函数栈帧上，在 `catch` 返回前持续有效；
    // 2. 在函数末尾退出前会配对调用 `replace_boundary(previous)` 恢复外层边界。
    let previous = unsafe { crate::ktest::replace_boundary(pointer) };
    // SAFETY: `pointer` 指向当前栈帧独占的 `Boundary`，且在进入 `body` 前无其他引用存在。
    if previous != unsafe { (*(*pointer).state.get()).previous } {
        crate::ktest::fatal("panic boundary installation mismatch");
    }

    // Step 3: 调用 unwinding 库的 catch_unwind 执行测试主体。
    let result = panicking::catch_unwind::<ExceptionToken, T, _>(body);
    if crate::ktest::current_boundary() != pointer {
        crate::ktest::fatal("panic boundary stack mismatch");
    }

    // Step 4: 校验边界状态机阶段，若捕获到异常则恢复硬件浮点控制状态并取出 PanicRecord。
    // SAFETY:
    // 1. `body` 与栈展开过程均已返回，`pointer` 指向的本地栈上 `Boundary` 仍然存活且无其他引用；
    // 2. 当 `state.phase == Phase::Returned` 时，`raise` 已完成了 `fp` 与 `record` 的初始化写入。
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

    // Step 5: 弹出当前边界，将外层捕获边界指针恢复到执行流槽位中。
    // SAFETY: 当前边界已完成异常回收并标记为 `Finished`，`previous` 为原先保存的外层固定边界指针或空指针。
    if unsafe { crate::ktest::replace_boundary(previous) } != pointer {
        crate::ktest::fatal("panic boundary removal mismatch");
    }
    result
}

/// 查询当前执行流是否正处于栈展开清理过程中（供调度器入口拒绝展开期间的 `yield`/`park`）。
pub fn is_unwinding() -> bool {
    let pointer = crate::ktest::current_boundary();
    if pointer.is_null() {
        return false;
    }
    // SAFETY: `current_boundary()` 仅返回当前执行流栈上存活的固定 `Boundary` 指针，读取其 `phase` 字段无并发竞争。
    let phase = unsafe { (*(*pointer).state.get()).phase };
    matches!(phase, Phase::Recording | Phase::InFlight | Phase::Returned)
}

/// 记录 panic 诊断信息并启动 DWARF 两阶段栈展开；若无捕获边界或发生二次 panic 则直接终止系统。
///
/// 由全局 `#[panic_handler]` 在 `panic = "unwind"` 模式下调用。
pub fn raise(info: &PanicInfo<'_>) -> ! {
    // Step 1: 查找当前执行流的捕获边界；若未安装边界（如异步中断或非测试上下文），直接上报未捕获致命事件。
    let boundary = crate::ktest::current_boundary();
    if boundary.is_null() {
        let kind = if info.can_unwind() { "panic" } else { "abi" };
        crate::ktest::fatal_event(kind, format_args!("uncaught panic: {}", info));
    }
    // SAFETY: `boundary` 由 `current_boundary()` 返回，指向当前栈上有效的 `Boundary`；
    // 此处仅获取 `*mut BoundaryState` 裸指针，暂不创建覆盖整个结构体的 `&mut`，从而允许在后续 `Display` 格式化递归 panic 时安全读取 `phase`。
    let state = unsafe { (*boundary).state.get() };

    // Step 2: 检查 ABI 展开许可与边界状态，拦截跨 `extern "C"` 边界的不可展开 panic 以及展开期间的二次 panic。
    if !info.can_unwind() {
        crate::ktest::fatal("panic cannot unwind across ABI boundary");
    }
    // SAFETY: `phase` 字段独立于 `record`，且仅由当前执行流在同步上下文中读写。
    if unsafe { ptr::addr_of!((*state).phase).read() } != Phase::Armed {
        crate::ktest::fatal("double panic during unwind");
    }

    // Step 3: 将状态切换为 Recording 并初始化 PanicRecord 缓冲区。
    // SAFETY: 前置检查已确认 `phase == Phase::Armed`，此时无活跃异常或 `record` 借用。
    unsafe {
        (*state).phase = Phase::Recording;
        (*state).record.write(PanicRecord::new());
    }

    // Step 4: 将 PanicInfo 的消息与源码位置格式化写入固定容量缓冲（若被测类型的 Display 实现内部再次 panic，Step 2 会因 phase == Recording 立即拦截为 double-panic）。
    // SAFETY:
    // 1. `record` 已在 Step 3 完成初始化；
    // 2. 若格式化过程中发生递归 panic，递归调用在 Step 2 检查 `phase` 即终止，不会再次可变借用 `record`。
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

    // Step 5: 临时开启硬件浮点单元（DWARF 展开器在保存/恢复寄存器上下文时可能访问浮点寄存器），并清零初始化异常头。
    // SAFETY: 此时 `phase` 已处于 `Recording`（禁止调度切换与跨核迁移），且 `record` 的可变借用已结束。
    unsafe {
        let state = &mut *(*boundary).state.get();
        state.fp.write(FpState::enable());
        // `UnwindException` 由整型字段与可选函数指针组成，全零位模式为合法初始态。
        ptr::write_bytes(state.exception.header.as_mut_ptr(), 0, 1);
    }

    // Step 6: 启动 DWARF 栈展开；正常情况下会直接跳转到 `catch` 的着陆垫，若返回则说明栈展开失败。
    let code = panicking::begin_panic(ExceptionToken(boundary));
    match code {
        UnwindReasonCode::END_OF_STACK => crate::ktest::fatal("panic unwind reached end of stack"),
        UnwindReasonCode::FATAL_PHASE1_ERROR => crate::ktest::fatal("panic unwind phase1 failed"),
        _ => crate::ktest::fatal("panic unwind phase2 failed"),
    }
}

/// 栈展开期间临时开启硬件浮点单元（FPU）所保存的控制与状态寄存器快照。
///
/// 内核线程默认保持 FPU 关闭（`FS=Off` / `EUEN.FPE=0`），而 DWARF 展开器在展开栈帧时可能保存或恢复浮点寄存器，
/// 因此在进入展开前临时开启 FPU，并在 `catch` 收回异常后恢复原始浮点开关与控制字。
struct FpState {
    /// 架构浮点使能控制位快照（RISC-V 为 `sstatus.FS`，LoongArch64 为 ` CSR.EUEN.FPE`）。
    control: usize,
    /// 浮点控制与状态寄存器（`fcsr` / `$fcsr0`）的原始值。
    fcsr: usize,
}

impl FpState {
    /// 开启当前 CPU 的硬件浮点单元并返回开启前的控制位与 `fcsr` 快照。
    fn enable() -> Self {
        #[cfg(target_arch = "riscv64")]
        {
            let control: usize;
            let fcsr: usize;
            // SAFETY:
            // 1. 当前运行于 RISC-V S-mode，且 `Recording` 状态已禁止线程调度迁移；
            // 2. 先通过 `csrrs` 置位 `sstatus.FS` 开启浮点单元，再执行 `frcsr` 读取浮点控制状态寄存器，不修改 `SIE` 中断位或 `tp` 寄存器。
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
            // SAFETY:
            // 1. 当前运行于 LoongArch64 PLV0 特权级，且 `Recording` 状态已禁止线程调度迁移；
            // 2. 通过 `csrxchg` 仅置位 `CSR_EUEN` 中的 `EUEN_FP` 位，开启浮点单元后再执行 `movfcsr2gr` 读取 `$fcsr0`，不影响中断状态或 `KS0`。
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

    /// 在 `catch` 捕获完成且异常已归还后，恢复展开前保存的 `fcsr` 与浮点单元使能控制位。
    ///
    /// 仅修改浮点相关控制位，保留栈展开执行 `Drop` 期间对中断开关或锁状态所做的合法变更。
    fn restore(self) {
        #[cfg(target_arch = "riscv64")]
        // SAFETY:
        // 1. `raise` 已开启 `sstatus.FS` 且展开全程未发生跨核迁移；
        // 2. 先恢复 `fcsr`，再通过掩码清除并写回 `sstatus.FS` 位，不触碰 `sstatus` 中的其他状态位（如 `SIE`）。
        unsafe {
            core::arch::asm!("fscsr {fcsr}", fcsr = in(reg) self.fcsr, options(nostack));
            core::arch::asm!("csrc sstatus, {mask}", "csrs sstatus, {control}",
                mask = in(reg) crate::arch::SR_FS, control = in(reg) self.control, options(nostack));
        }
        #[cfg(target_arch = "loongarch64")]
        // SAFETY:
        // 1. `raise` 已开启 `EUEN.FPE` 且展开全程未发生跨核迁移；
        // 2. 先恢复 `$fcsr0`，再通过 `csrxchg` 仅恢复 `CSR_EUEN` 中的 `EUEN_FP` 位。
        unsafe {
            core::arch::asm!("movgr2fcsr $fcsr0, {fcsr}", fcsr = in(reg) self.fcsr, options(nostack));
            core::arch::asm!("csrxchg {control}, $t0, {euen}",
                control = inout(reg) self.control => _, euen = const crate::arch::CSR_EUEN,
                in("$t0") crate::arch::EUEN_FP, options(nostack));
        }
    }
}
