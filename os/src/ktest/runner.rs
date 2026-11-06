//! 内核测试框架（`ktest`）的早期命令握手、测试套件调度执行器与确定性随机数种子派生模块。
//!
//! ## 模块职责与架构定位
//! 本模块负责在内核启动早期与宿主机测试运行器完成串口控制握手，读取并解析宿主机下发的单条测试命令（`catalog` 或 `run ...`），
//! 根据目标测试套件请求的执行上下文（`Context::Boot` 或 `Context::Thread`）将其分派到早期启动栈或内核线程栈上运行，
//! 并在存在用户态网络对端资源（`Resource::NetUser`）时协调每个用例开始/结束边界的宿主机串口双向握手。
//! - **上游调用者**：内核主入口在架构与 CPU 初始化完成后调用 `early()`，随后根据 `requested_context()` 调用 `run_boot()` 或在启动调度器后调用 `spawn_thread()`。
//! - **下游依赖**：`ktest::registry`（测试注册表加载与目录输出）、`ktest::context`（测试运行时服务与子任务作用域 `ArcScope`）、`ktest::report`（KTAP 控制行输出）及 `console::testing`（串口接收控制权切换）。
//!
//! ## 核心设计与宿主机握手/执行状态机
//! ```text
//! [early() 启动早期握手]
//!        │
//!        ├── 1. 强制链接各子系统测试锚点 (anchors) 并校验 Registry
//!        ├── 2. 串口输出 "KTEST READY 1 arch=... cpus=... mode=... panic=..."
//!        └── 3. 轮询读取宿主机单行命令:
//!               ├─► "catalog" ──► Registry::catalog() 流式输出 JSON 目录并关机
//!               └─► "run <suite> <seed> <repeat> <c0,c1,...>" ──► 返回 RunCommand
//!                                                                      │
//!        ┌─────────────────────────────────────────────────────────────┘
//!        ▼
//! [按 Context 分派执行 execute()]
//!   ├─ Context::Boot   ──► run_boot() 在启动栈直接进入 execute()
//!   └─ Context::Thread ──► spawn_thread() 在可迁移内核线程中进入 execute()
//!                                │
//!                                ▼
//!                 [Suite / Case 生命周期循环]
//!   suite_begin -> (可选套件级 ArcScope) -> 调用 (suite.run)(&mut context, &mut runner)
//!     └─► Runner::run() 遍历选中的每个 Case:
//!           1. 检查 CPU 数量等静态适用条件 (不满足则输出 SKIP)
//!           2. [若含 NetUser] 独占串口 RX -> 输出 case-begin -> 等待 "KTEST PEER BEGIN <id>"
//!           3. 创建用例级 ArcScope 与 RuntimeServices，派生 SHA-256 确定性种子
//!           4. 调用 dispatch 执行用例 Setup -> Body -> Quiesce -> Join -> Cleanup
//!           5. [若含 NetUser] 独占串口 RX -> 输出用例结果 -> 等待 "KTEST PEER END <id>"
//!   全部完成后输出套件汇总结果并调用 test_shutdown() 关机。
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `COMMAND_READ` 原子布尔量保证每次虚拟机生命周期内 `early()` 最多只读取一次宿主机命令。
//! - 在 `Runner::peer_boundary` 握手期间，会通过 `console::testing::claim_receive_control()` 临时接管串口接收权，同步排空在途的接收中断与 TTY 后台工作队列（`tty::flush_work()`），待收到宿主机精确回执后再释放接收控制权。

use super::{
    context::{ArcScope, BootGuard, RuntimeServices, ScopeGuard},
    registry::Registry,
    report,
};
use ::ktest::{
    CaseDesc, CaseOutcome, CaseRunner, Context, Failure, Phase, Resource, Status, SuiteDesc,
    TestContext, TestError, TestResult,
};
use core::convert::TryInto;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

/// 单次 `run` 命令最多允许同时选择的测试用例数量上限。
const MAX_SELECTED_CASES: usize = 64;
/// 宿主机下发的单条测试控制命令行的最大字节容量（含结尾换行符 `\n`）。
const COMMAND_CAPACITY: usize = 1024;
/// 宿主机在对端同步边界返回的 `KTEST PEER BEGIN/END <id>\n` 回执行的最大字节容量。
const PEER_ACK_CAPACITY: usize = 64;

/// 标记宿主机早期测试命令是否已被读取的原子标志，防止重复调用 `early()`。
static COMMAND_READ: AtomicBool = AtomicBool::new(false);

/// 已解析并完成合法性校验的单次测试运行命令。
///
/// 仅保存定长的套件与用例数字 ID 数组及轻量级 `Registry` 句柄，无需依赖堆内存分配。
#[derive(Clone, Copy)]
pub(crate) struct RunCommand {
    /// 待运行的测试套件在注册表中的序号（`0..registry.count`）。
    pub(crate) suite: u32,
    /// 宿主机指定的基础随机种子（用于派生每个用例的确定性伪随机种子）。
    pub(crate) seed: u64,
    /// 当前运行的重复轮次序号（与 `seed` 和用例名共同参与种子哈希派生）。
    pub(crate) repeat: u32,
    /// 本次选中要依次执行的套件内测试用例序号数组（有效前缀长度为 `count`）。
    cases: [u32; MAX_SELECTED_CASES],
    /// `cases` 数组中实际选中的测试用例数量（`1..=MAX_SELECTED_CASES`）。
    count: usize,
    /// 已校验通过的只读测试套件注册表句柄。
    registry: Registry,
}

/// 在内核启动早期加载测试注册表、输出就绪握手行并读取解析宿主机的单行测试命令。
///
/// 调用上下文：在本地 CPU 与基础架构设施初始化完毕、且串口接收中断（UART RX IRQ）尚未交由常规控制台接管前调用。
/// 若宿主机下发 `"catalog"` 命令，本函数会直接流式导出全部测试目录并关机，不再返回。
pub(crate) fn early() -> RunCommand {
    // Step 1: 确保整个内核生命周期内只读取一次测试命令，并显式保留所有测试模块的链接锚点以防被链接器优化剔除。
    if COMMAND_READ.swap(true, Ordering::AcqRel) {
        super::context::fatal("test command read twice");
    }

    core::hint::black_box(super::boot_cases::anchors());
    core::hint::black_box(super::execution_core::anchors());
    core::hint::black_box(super::execution_services::anchors());
    core::hint::black_box(super::user_execution::anchors());
    core::hint::black_box(crate::drivers::virtio::ktest_prepare::anchors());
    core::hint::black_box(crate::block::ktest_irq::anchors());
    core::hint::black_box(crate::cache::ktest::anchors());
    core::hint::black_box(crate::fs::ktest::anchors());
    core::hint::black_box(super::network::anchors());
    core::hint::black_box(crate::drivers::virtio_net::ktest::native::__ktest_anchor());

    // Step 2: 校验 ELF 只读段中的测试套件注册表，并向宿主机发送 KTEST READY 握手通告。
    let registry = Registry::load().unwrap_or_else(|e| super::context::fatal(e.0));
    let arch = if cfg!(target_arch = "riscv64") {
        "riscv64"
    } else {
        "loongarch64"
    };
    let mode = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    let panic = if cfg!(panic = "unwind") {
        "unwind"
    } else {
        "abort"
    };
    report::write(format_args!(
        "KTEST READY 1 arch={} cpus={} mode={} panic={}\n",
        arch,
        crate::cpu::count(),
        mode,
        panic
    ));

    // Step 3: 从控制串口同步轮询读取宿主机下发的单行命令（以 '\n' 结尾，上限 1024 字节）。
    let mut input = [0u8; COMMAND_CAPACITY];
    let mut len = 0;
    loop {
        if len == input.len() {
            super::context::fatal("test command exceeds 1024 bytes");
        }
        let byte = super::control_read_byte();
        input[len] = byte;
        len += 1;
        if byte == b'\n' {
            break;
        }
    }

    // Step 4: 处理 catalog 目录导出命令，或解析校验 run 命令的目标套件、用例序号及 Case 级虚拟机隔离约束。
    let text = core::str::from_utf8(&input[..len - 1])
        .unwrap_or_else(|_| super::context::fatal("test command is not UTF-8"));
    if text == "catalog" {
        registry.catalog(arch, crate::cpu::count() as u32, mode, panic);
    }

    let command = parse(text, registry).unwrap_or_else(|e| super::context::fatal(e.0));
    let suite = registry
        .suite(command.suite)
        .unwrap_or_else(|e| super::context::fatal(e.0));
    for &id in &command.cases[..command.count] {
        if id as usize >= suite.cases.len() {
            super::context::fatal("unknown case ID");
        }
    }
    if command.count != 1
        && command.cases[..command.count]
            .iter()
            .any(|&id| suite.cases[id as usize].attributes.isolation == ::ktest::Isolation::Case)
    {
        super::context::fatal("case isolation requires independent VM");
    }
    command
}

/// 查询 `command` 选中的测试套件所要求的运行上下文（`Context::Boot` 或 `Context::Thread`）。
///
/// 供内核启动主流程在初始化内存管理（`mm`）与任务调度器（`task`）之前决定直接运行 Boot 测试还是继续完整启动流程。
pub(crate) fn requested_context(command: &RunCommand) -> Context {
    command
        .registry
        .suite(command.suite)
        .unwrap_or_else(|e| super::context::fatal(e.0))
        .defaults
        .context
}

/// 在内核早期启动栈（`Context::Boot`）上直接运行指定的测试套件，执行完毕后关机退出。
///
/// 调用上下文：在内存管理、文件系统或任务调度器尚未统一初始化前由主启动核直接调用；若套件实际需要某些底层子系统，由其自身夹具按需初始化。
pub(crate) fn run_boot(command: RunCommand) -> ! {
    if requested_context(&command) != Context::Boot {
        super::context::fatal("Thread suite dispatched on Boot stack");
    }
    let _boot = BootGuard::enter();
    execute(command)
}

/// 创建并启动一个可跨核迁移的内核线程（`Task`），在该线程栈上同步运行指定的 `Context::Thread` 测试套件。
///
/// 调用上下文：在内存管理与任务调度器初始化完成后调用。返回用于等待测试主线程结束的 `JoinHandle`。
pub(crate) fn spawn_thread(command: RunCommand) -> Result<crate::task::JoinHandle, TestError> {
    if requested_context(&command) != Context::Thread {
        return Err(TestError("Boot suite dispatched on Thread stack"));
    }
    crate::task::spawn_kernel(move || execute(command))
        .map_err(|_| TestError("NoMemory: suite Task"))
}

/// 解析形如 `"run <suite> <seed> <repeat> <case0,case1,...>"` 的测试控制命令行。
///
/// 严格拒绝未知命令前缀、多余字段、重复用例序号、空用例列表以及任何十进制整数溢出。
fn parse(text: &str, registry: Registry) -> Result<RunCommand, TestError> {
    // Step 1: 按单个空格切分命令头与固定位置参数（suite、seed、repeat、cases 列表）。
    let mut fields = text.split(' ');
    if fields.next() != Some("run") {
        return Err(TestError("unknown test command"));
    }
    let suite = number(fields.next().ok_or(TestError("missing suite"))?)?
        .try_into()
        .map_err(|_| TestError("suite overflow"))?;
    let seed = number(fields.next().ok_or(TestError("missing seed"))?)?;
    let repeat = number(fields.next().ok_or(TestError("missing repeat"))?)?
        .try_into()
        .map_err(|_| TestError("repeat overflow"))?;
    let list = fields.next().ok_or(TestError("missing cases"))?;
    if fields.next().is_some() {
        return Err(TestError("extra command fields"));
    }

    // Step 2: 解析以逗号分隔的用例序号列表，检查容量上限（64 个）与重复序号。
    let mut command = RunCommand {
        suite,
        seed,
        repeat,
        cases: [0; MAX_SELECTED_CASES],
        count: 0,
        registry,
    };
    for item in list.split(',') {
        if command.count == MAX_SELECTED_CASES {
            return Err(TestError("more than 64 selected cases"));
        }
        let id = number(item)?
            .try_into()
            .map_err(|_| TestError("case overflow"))?;
        if command.cases[..command.count].contains(&id) {
            return Err(TestError("duplicate selected case"));
        }
        command.cases[command.count] = id;
        command.count += 1;
    }
    Ok(command)
}

/// 将无符号十进制 ASCII 字符串解析为 `u64`（无需堆内存分配，遇到空串、非数字字符或算术溢出时返回错误）。
fn number(text: &str) -> Result<u64, TestError> {
    if text.is_empty() {
        return Err(TestError("empty command integer"));
    }
    let mut value = 0u64;
    for b in text.bytes() {
        if !b.is_ascii_digit() {
            return Err(TestError("nondecimal command integer"));
        }
        value = value
            .checked_mul(10)
            .and_then(|v| v.checked_add((b - b'0') as u64))
            .ok_or(TestError("command integer overflow"))?;
    }
    Ok(value)
}

/// 判断本次运行是否需要在每个测试用例的开始与结束边界与宿主机执行 `KTEST PEER` 串口同步握手。
///
/// 只要测试套件默认资源或本次选中的任意测试用例声明了 `Resource::NetUser`（宿主机用户态网络对端），即返回 `true`。
fn needs_peer(command: &RunCommand, suite: &SuiteDesc) -> bool {
    suite.defaults.resources.contains(&Resource::NetUser)
        || command.cases[..command.count].iter().any(|&id| {
            suite.cases[id as usize]
                .attributes
                .resources
                .contains(&Resource::NetUser)
        })
}

/// 从控制串口同步读取一行宿主机对端握手回执，校验其前缀 `prefix` 与十进制用例序号是否等于期望的 `id`。
///
/// 调用上下文：必须在已由 `Runner::peer_boundary` 独占串口接收控制权后调用。
fn peer_acknowledgement(id: u32, prefix: &[u8]) -> TestResult {
    let mut line = [0u8; PEER_ACK_CAPACITY];
    let mut length = 0;
    loop {
        if length == line.len() {
            return Err(TestError("peer acknowledgement exceeds UART line bound"));
        }
        let byte = super::control_read_byte();
        line[length] = byte;
        length += 1;
        if byte == b'\n' {
            break;
        }
    }
    let digits = line[..length - 1]
        .strip_prefix(prefix)
        .ok_or(TestError("peer acknowledgement phase mismatch"))?;
    let text = core::str::from_utf8(digits)
        .map_err(|_| TestError("peer acknowledgement is not ASCII decimal"))?;
    let received: u32 = number(text)
        .map_err(|_| TestError("peer acknowledgement ID is malformed"))?
        .try_into()
        .map_err(|_| TestError("peer acknowledgement ID overflows"))?;
    if received != id {
        return Err(TestError("peer acknowledgement case ID mismatch"));
    }
    Ok(())
}

/// 单个测试套件执行期间的状态机与用例分派器（实现 `::ktest::CaseRunner`）。
struct Runner {
    /// 当前正在执行的测试运行命令。
    command: RunCommand,
    /// 当前测试套件的静态元数据描述符引用。
    suite: &'static SuiteDesc,
    /// 当前正在处理的测试用例在 `command.cases` 选择列表中的下标（`0..command.count`）。
    position: usize,
    /// 标记当前位置对应的测试用例是否已输出 `# KTEST case-begin` 与 `setup` 阶段事件。
    begun: bool,
    /// 累计记录本套件内已执行的所有测试用例是否全部通过（`Pass` 或 `Skip`）。
    passed: bool,
    /// 标记本次运行是否包含 `Resource::NetUser` 资源，从而需要在每个用例边界执行宿主机对端握手。
    peer: bool,
}

impl Runner {
    /// 检查测试用例 `case` 在当前硬件环境下的静态适用性；若不满足则返回跳过原因（如 `"insufficient CPUs"`）。
    ///
    /// 静态跳过不会触发用例级夹具（Fixture）的构造；`ignore` 属性已由宿主机在筛选阶段处理，一旦下发了确定 ID 即正常执行。
    fn applicability(&self, case: &CaseDesc) -> Option<&'static str> {
        if case.attributes.min_cpus > crate::cpu::count() as u32 {
            return Some("insufficient CPUs");
        }
        None
    }

    /// 在发布用例开始或结束控制事件前后，执行可选的宿主机对端（Peer）串口同步握手。
    ///
    /// 当 `self.peer` 为 `true` 时：
    /// 1. 先独占串口接收控制权，同步等待在途 UART RX 中断与 TTY 工作队列排空，并确认接收缓冲区为空；
    /// 2. 调用 `publish` 输出控制行；
    /// 3. 阻塞等待宿主机返回匹配 `prefix` 与用例 `id` 的回执行，随后归还串口接收控制权。
    fn peer_boundary(&self, id: u32, prefix: &[u8], publish: impl FnOnce()) {
        if !self.peer {
            publish();
            return;
        }

        let receiver = crate::console::testing::claim_receive_control();
        crate::console::testing::synchronize_receive_irq()
            .unwrap_or_else(|error| super::context::fatal(error.0));
        crate::fs::tty::flush_work();
        if !crate::console::testing::receive_queue_empty() {
            super::context::fatal("UART RX ring is not empty before peer boundary");
        }

        publish();
        peer_acknowledgement(id, prefix).unwrap_or_else(|error| super::context::fatal(error.0));
        crate::console::testing::release_receive_control(receiver);
    }

    /// 若当前用例尚未登记开始，则触发可选的 `KTEST PEER BEGIN` 握手并输出 `case-begin` 与 `phase setup` 控制行。
    fn begin(&mut self, id: u32) {
        if !self.begun {
            self.peer_boundary(id, b"KTEST PEER BEGIN ", || report::begin(id));
            report::phase(id, Phase::Setup);
            self.begun = true;
        }
    }

    /// 连续处理从当前 `position` 开始所有因静态条件（如 CPU 数量不足）不适用而需要直接跳过的测试用例。
    ///
    /// 被跳过的用例不调用夹具 `set_up`，但仍按统一的协议顺序完成 `BEGIN` / `SKIP` 结果输出与 `END` 对端握手。
    fn static_prefix(&mut self) {
        while self.position < self.command.count {
            let id = self.command.cases[self.position];
            let case = &self.suite.cases[id as usize];
            let Some(reason) = self.applicability(case) else {
                break;
            };
            self.begin(id);
            let outcome = CaseOutcome {
                status: Status::Skip,
                primary: None,
                child: None,
                cleanup: None,
                expectations: 0,
            };
            self.peer_boundary(id, b"KTEST PEER END ", || {
                report::result(self.position + 1, case, &outcome, Some(reason));
            });
            self.position += 1;
            self.begun = false;
        }
    }
}

impl CaseRunner for Runner {
    /// 遍历并分派执行当前测试套件中所有被选中的测试用例。
    ///
    /// 对每个可运行的测试用例依次建立子任务作用域（`ArcScope`）与运行时服务（`RuntimeServices`），
    /// 派生确定性随机种子后调用生成的套件分派闭包 `dispatch`，最后在宿主机对端 `END` 握手边界内输出用例终态结果。
    fn run(
        &mut self,
        cases: &'static [CaseDesc],
        dispatch: &mut dyn FnMut(&mut TestContext<'_>, usize) -> CaseOutcome,
    ) -> TestResult {
        if !core::ptr::eq(cases, self.suite.cases) {
            return Err(TestError("suite dispatch descriptor identity mismatch"));
        }

        while self.position < self.command.count {
            // Step 1: 先跳过当前排在前面的所有静态不适用（如 CPU 数不足）的用例。
            self.static_prefix();
            if self.position == self.command.count {
                break;
            }
            let id = self.command.cases[self.position];
            let case = &cases[id as usize];
            self.begin(id);

            // Step 2: 若为 Thread 上下文用例，为其分配独立的子任务生命周期管理作用域 ArcScope。
            let scope = if case.attributes.context == Context::Thread {
                match ArcScope::new(id) {
                    Ok(scope) => Some(scope),
                    Err(error) => {
                        let outcome = CaseOutcome {
                            status: Status::Error,
                            primary: Some(Failure::Returned(error)),
                            child: None,
                            cleanup: None,
                            expectations: 0,
                        };
                        self.peer_boundary(id, b"KTEST PEER END ", || {
                            report::result(self.position + 1, case, &outcome, None);
                        });
                        self.position += 1;
                        self.begun = false;
                        self.passed = false;
                        continue;
                    }
                }
            } else {
                None
            };
            let _guard = scope.as_ref().map(ScopeGuard::install);

            // Step 3: 激活当前用例的 RuntimeServices，构造 TestContext 并调用套件生成的类型化分派入口。
            let services = RuntimeServices::new(case, id, scope.as_ref(), self.suite.name);
            services.activate();
            // 用例的 `# KTEST phase <id> setup` 已在上方 `self.begin(id)` 中输出，此处同步内部状态标记。
            services.mark_setup();
            let mut context = TestContext::new(
                &services,
                case,
                case_seed(self.command.seed, self.command.repeat, case.name),
                self.command.repeat,
            );

            let outcome = dispatch(&mut context, id as usize);

            // Step 4: 汇总通过状态，在可选的对端 END 握手保护下发布最终结果并推进到下一个选中用例。
            self.passed &= matches!(outcome.status, Status::Pass | Status::Skip);
            self.peer_boundary(id, b"KTEST PEER END ", || {
                services.finish(self.position + 1, &outcome);
            });
            self.begun = false;
            self.position += 1;
        }
        Ok(())
    }
}

/// 执行由 `command` 指定的测试套件主流程，完成后输出套件终态并调用架构关机接口退出虚拟机。
///
/// 调用上下文：在 `run_boot`（启动栈）或 `spawn_thread`（内核线程栈）中调用。
fn execute(command: RunCommand) -> ! {
    // Step 1: 加载套件描述符，输出 KTAP 套件头部，并检查 NetUser 对端同步是否运行在合法的 Thread 上下文。
    let suite = command
        .registry
        .suite(command.suite)
        .unwrap_or_else(|e| super::context::fatal(e.0));
    report::suite_begin(suite.name, command.count);
    report::write(format_args!("  # KTEST suite-begin setup\n"));
    let mut runner = Runner {
        command,
        suite,
        position: 0,
        begun: false,
        passed: true,
        peer: needs_peer(&command, suite),
    };
    if runner.peer && suite.defaults.context != Context::Thread {
        report::suite_error(
            suite.name,
            "setup",
            Failure::Returned(TestError("net-user peer boundary requires Thread context")),
        );
    }

    // Step 2: 查找首个满足静态适用条件的用例；若全部选中用例均需静态跳过，则无需构造套件级共享资源，直接输出跳过结果并结束。
    let runnable = command.cases[..command.count]
        .iter()
        .find(|&&id| runner.applicability(&suite.cases[id as usize]).is_none())
        .copied();
    let Some(id) = runnable else {
        runner.static_prefix();
        report::complete(suite.name, runner.passed);
    };

    // Step 3: 为套件级 setup/cleanup 建立运行上下文与可选的套件级 ArcScope（使用 u32::MAX 作为套件级哨兵 ID），随后调用 suite.run。
    let case = &suite.cases[id as usize];
    let scope = if suite.defaults.context == Context::Thread {
        Some(ArcScope::new(u32::MAX).unwrap_or_else(|error| {
            report::suite_error(suite.name, "setup", Failure::Returned(error))
        }))
    } else {
        None
    };
    let _guard = scope.as_ref().map(ScopeGuard::install);
    let services = RuntimeServices::new(case, u32::MAX, scope.as_ref(), suite.name);
    services.activate();
    let mut context = TestContext::new(
        &services,
        case,
        case_seed(command.seed, command.repeat, case.name),
        command.repeat,
    );

    let suite_outcome = (suite.run)(&mut context, &mut runner);

    // Step 4: 检查套件级 setup、执行、子任务或清理阶段是否产生错误，验证所有选中用例均已执行完毕，并输出套件最终结果后关机。
    let suite_failure = suite_outcome
        .setup
        .or(suite_outcome.execution)
        .or(suite_outcome.child)
        .or(suite_outcome.cleanup)
        .or(if suite_outcome.expectations != 0 {
            Some(Failure::Expectation)
        } else {
            None
        });
    if let Some(error) = suite_failure {
        report::diagnostic(format_args!("suite outcome: {:?}", suite_outcome));
        let stage = if suite_outcome.setup.is_some() {
            "setup"
        } else if suite_outcome.execution.is_some() {
            "execution"
        } else {
            "cleanup"
        };
        report::suite_error(suite.name, stage, error);
    }
    if runner.position != command.count {
        report::suite_error(
            suite.name,
            "execution",
            Failure::Returned(TestError("suite returned without selected case results")),
        );
    }
    report::complete(suite.name, runner.passed)
}

/// 根据基础种子 `seed`、重复轮次 `repeat` 与测试用例完整名称 `name`，计算 `SHA256("<seed>:<repeat>:<name>")` 摘要的前 8 字节大端 `u64` 作为该用例的确定性随机种子。
///
/// 全程通过栈上流式哈希器计算，无需分配堆内存，且生成的种子与宿主机 Python `hashlib.sha256` 结果严格一致。
pub(crate) fn case_seed(seed: u64, repeat: u32, name: &str) -> u64 {
    let mut hash = Sha256::new();
    let _ = write!(hash, "{}:{}:{}", seed, repeat, name);
    hash.finish()
}

/// SHA-256 哈希算法的单个消息分组字节数（512 位 = 64 字节）。
const SHA256_BLOCK_BYTES: usize = 64;
/// SHA-256 尾部填充的消息比特长度字段所占字节数（64 位 = 8 字节）。
const SHA256_LENGTH_BYTES: usize = core::mem::size_of::<u64>();

/// 零堆分配的流式 SHA-256 哈希计算器，专门用于派生确定性的测试用例种子。
struct Sha256 {
    /// SHA-256 的 8 个 32 位中间链接状态字（`H0..H7`）。
    state: [u32; 8],
    /// 当前尚未凑满 64 字节的输入消息分组缓冲区。
    block: [u8; SHA256_BLOCK_BYTES],
    /// `block` 缓冲区中当前已填入的有效字节数（`0..SHA256_BLOCK_BYTES`）。
    used: usize,
    /// 已累计输入的消息总字节数（按模 $2^{64}$ 计数）。
    bytes: u64,
}

impl Sha256 {
    /// 使用 FIPS 180-4 标准规定的初始哈希值（`H0..H7`）构造新的 `Sha256` 实例。
    fn new() -> Self {
        Self {
            state: [
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
                0x5be0cd19,
            ],
            block: [0; SHA256_BLOCK_BYTES],
            used: 0,
            bytes: 0,
        }
    }

    /// 向哈希计算器追加任意长度的字节切片 `data`，每凑满 64 字节即执行一次分组压缩。
    fn update(&mut self, mut data: &[u8]) {
        self.bytes = self.bytes.wrapping_add(data.len() as u64);
        while !data.is_empty() {
            let count = data.len().min(SHA256_BLOCK_BYTES - self.used);
            self.block[self.used..self.used + count].copy_from_slice(&data[..count]);
            self.used += count;
            data = &data[count..];
            if self.used == SHA256_BLOCK_BYTES {
                self.compress();
                self.used = 0;
            }
        }
    }

    /// 对当前 `self.block` 中的 64 字节消息分组执行标准的 64 轮 SHA-256 压缩函数，并更新 `self.state`。
    fn compress(&mut self) {
        const K: [u32; 64] = [
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
            0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
            0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
            0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
            0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
            0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
            0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
            0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
            0xc67178f2,
        ];
        let mut w = [0u32; 64];
        for (i, chunk) in self.block.chunks_exact(4).enumerate() {
            w[i] = u32::from_be_bytes(chunk.try_into().unwrap());
        }
        for i in 16..64 {
            let a = w[i - 15];
            let b = w[i - 2];
            w[i] = w[i - 16]
                .wrapping_add(a.rotate_right(7) ^ a.rotate_right(18) ^ (a >> 3))
                .wrapping_add(w[i - 7])
                .wrapping_add(b.rotate_right(17) ^ b.rotate_right(19) ^ (b >> 10));
        }

        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = self.state;
        for i in 0..64 {
            let t1 = h
                .wrapping_add(e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25))
                .wrapping_add((e & f) ^ (!e & g))
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let t2 = (a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22))
                .wrapping_add((a & b) ^ (a & c) ^ (b & c));
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }

        for (state, value) in self.state.iter_mut().zip([a, b, c, d, e, f, g, h]) {
            *state = state.wrapping_add(value);
        }
    }

    /// 写入标准的 `0x80` 比特填充、零字节填充以及 64 位大端消息总比特长度，并返回最终哈希摘要的前 8 字节大端 `u64` 值。
    fn finish(mut self) -> u64 {
        let bits = self.bytes.wrapping_mul(8);
        self.update(&[0x80]);
        while self.used != SHA256_BLOCK_BYTES - SHA256_LENGTH_BYTES {
            self.update(&[0]);
        }
        self.update(&bits.to_be_bytes());
        ((self.state[0] as u64) << 32) | self.state[1] as u64
    }
}

impl core::fmt::Write for Sha256 {
    /// 将格式化字符串切片直接作为 UTF-8 字节流送入 `Sha256::update`。
    fn write_str(&mut self, value: &str) -> core::fmt::Result {
        self.update(value.as_bytes());
        Ok(())
    }
}
