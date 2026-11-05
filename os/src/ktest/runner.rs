//! 一次早期命令、同步 typed suite dispatch 和宿主一致的流式实例 seed。

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

const MAX_SELECTED_CASES: usize = 64;
const COMMAND_CAPACITY: usize = 1024;
const PEER_ACK_CAPACITY: usize = 64;

static COMMAND_READ: AtomicBool = AtomicBool::new(false);

/// 只有一个命令的有界静态 ID，不缓存全目录或名称。
#[derive(Clone, Copy)]
pub(crate) struct RunCommand {
    pub(crate) suite: u32,
    pub(crate) seed: u64,
    pub(crate) repeat: u32,
    cases: [u32; MAX_SELECTED_CASES],
    count: usize,
    registry: Registry,
}

/// CPU/arch 已安装且 UART IRQ RX 尚未接管时调用；catalog 在这里真实退出。
pub(crate) fn early() -> RunCommand {
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

/// 供父在 MM/Task 初始化前决定真实 Boot 或 Thread 路径。
pub(crate) fn requested_context(command: &RunCommand) -> Context {
    command
        .registry
        .suite(command.suite)
        .unwrap_or_else(|e| super::context::fatal(e.0))
        .defaults
        .context
}

/// 不初始化 MM、文件系统或调度器；需要这些的具体 Boot suite 自行真实取得。
pub(crate) fn run_boot(command: RunCommand) -> ! {
    if requested_context(&command) != Context::Boot {
        super::context::fatal("Thread suite dispatched on Boot stack");
    }
    let _boot = BootGuard::enter();
    execute(command)
}

/// 在真实可迁核 Task 上同步运行整个 suite，不把 borrowed case closure 伪装 static。
pub(crate) fn spawn_thread(command: RunCommand) -> Result<crate::task::JoinHandle, TestError> {
    if requested_context(&command) != Context::Thread {
        return Err(TestError("Boot suite dispatched on Thread stack"));
    }
    crate::task::spawn_kernel(move || execute(command))
        .map_err(|_| TestError("NoMemory: suite Task"))
}

/// 精确解析固定文法；拒绝多命令、重复项、空列表与所有整数溢出。
fn parse(text: &str, registry: Registry) -> Result<RunCommand, TestError> {
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

/// 不借用外部 parser 或分配字符串的十进制溢出检查。
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

/// 仅按 suite 与全部选中 case 的资源并集决定本 VM 的 peer 握手，不受标签或静态 Skip 影响。
fn needs_peer(command: &RunCommand, suite: &SuiteDesc) -> bool {
    suite.defaults.resources.contains(&Resource::NetUser)
        || command.cases[..command.count].iter().any(|&id| {
            suite.cases[id as usize]
                .attributes
                .resources
                .contains(&Resource::NetUser)
        })
}

/// 关闭 RX 的控制者只消费一条有界完整回执，阶段、十进制 ID 和换行必须与当前边界一致。
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

struct Runner {
    command: RunCommand,
    suite: &'static SuiteDesc,
    position: usize,
    begun: bool,
    passed: bool,
    peer: bool,
}

impl Runner {
    /// 静态不适用不构造 fixture；资源取得错误由具体生产 fixture 真实返回。
    fn applicability(&self, case: &CaseDesc) -> Option<&'static str> {
        // ignore 已由宿主筛选；收到确定 ID（含 include-ignored）就实际执行。
        if case.attributes.min_cpus > crate::cpu::count() as u32 {
            return Some("insufficient CPUs");
        }
        None
    }

    /// 发布边界前独占控制 RX 并同步旧 IRQ/Work，精确回执后才恢复原 RX 状态；失败终止 VM。
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

    /// setup 前发布当前项；需要 peer 的批次先取得真实 BEGIN 回执，才允许 case 准备和生产。
    fn begin(&mut self, id: u32) {
        if !self.begun {
            self.peer_boundary(id, b"KTEST PEER BEGIN ", || report::begin(id));
            report::phase(id, Phase::Setup);
            self.begun = true;
        }
    }

    /// 静态 CPU 不适用不调用设备 setup，仍按本 VM 资源并集完成 BEGIN/END 后才推进下一项。
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
    /// typed dispatch 完成真实清理后保留原终态发布，宿主 END 回执先于位置推进和后继开始。
    fn run(
        &mut self,
        cases: &'static [CaseDesc],
        dispatch: &mut dyn FnMut(&mut TestContext<'_>, usize) -> CaseOutcome,
    ) -> TestResult {
        if !core::ptr::eq(cases, self.suite.cases) {
            return Err(TestError("suite dispatch descriptor identity mismatch"));
        }

        while self.position < self.command.count {
            self.static_prefix();
            if self.position == self.command.count {
                break;
            }
            let id = self.command.cases[self.position];
            let case = &cases[id as usize];
            self.begin(id);
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

            let services = RuntimeServices::new(case, id, scope.as_ref(), self.suite.name);
            services.activate();
            // case 的 setup 控制已在正式 typed fixture 调用之前输出。
            services.mark_setup();
            let mut context = TestContext::new(
                &services,
                case,
                case_seed(self.command.seed, self.command.repeat, case.name),
                self.command.repeat,
            );

            let outcome = dispatch(&mut context, id as usize);
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

/// 调用正式 typed suite 入口，握手只用于含 net-user 的 Thread 批次，不改原终止路径。
fn execute(command: RunCommand) -> ! {
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

    let runnable = command.cases[..command.count]
        .iter()
        .find(|&&id| runner.applicability(&suite.cases[id as usize]).is_none())
        .copied();
    let Some(id) = runnable else {
        runner.static_prefix();
        report::complete(suite.name, runner.passed);
    };

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

/// SHA256("seed:repeat:name") 前八字节大端；数字也流式输入，不建名称 Vec。
pub(crate) fn case_seed(seed: u64, repeat: u32, name: &str) -> u64 {
    let mut hash = Sha256::new();
    let _ = write!(hash, "{}:{}:{}", seed, repeat, name);
    hash.finish()
}

const SHA256_BLOCK_BYTES: usize = 64;
const SHA256_LENGTH_BYTES: usize = core::mem::size_of::<u64>();

struct Sha256 {
    state: [u32; 8],
    block: [u8; SHA256_BLOCK_BYTES],
    used: usize,
    bytes: u64,
}

impl Sha256 {
    /// 只用于固定种子合同的标准 SHA-256 初态。
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

    /// 任意长 UTF-8名称逐块输入，消息长度按SHA标准取模2^64。
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

    /// 64轮标准压缩，只保留固定栈工作空间。
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

    /// 标准1位填充与64位长度，只返回合同所需的前八字节大端值。
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
    fn write_str(&mut self, value: &str) -> core::fmt::Result {
        self.update(value.as_bytes());
        Ok(())
    }
}
