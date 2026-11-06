//! 用户程序参数准备、未发布任务回滚与内存分配失败恢复测试模块。
//!
//! # 模块职责与架构定位
//! 本模块属于用户态执行与文件系统联合内核测试（`fs::ktest::user::preparation`），
//! 上游由 `programs::failed_preparation_keeps_original_program` 测试用例调用，
//! 下游依赖虚拟内存空间（`Mm`）、ELF 装载器（`task::exec`）、进程关系图（`TaskGraph`）
//! 与任务准备管线（`prepare_program`）。
//!
//! 本模块专门验证 `execve` 与进程创建在“真正提交到调度器运行之前”的各个准备阶段：
//! 1. 从用户空间跨页读取 `argv`/`envp` 指针表与字符串时的快照隔离性、非法地址拦截与单串容量上限（`E2BIG`）。
//! 2. 进程号（PID/TID）预留与完整候选程序（`PreparedProgram`）在未发布（unpublished）状态下直接丢弃时的资源回滚与号码立即复用。
//! 3. 在堆内存耗尽（OOM）压力下，参数拷贝与程序准备失败后不会残留幽灵线程或泄漏 PID。
//! 4. 上述所有失败与回滚发生后，原先已加载的正常 ELF 映像仍能不受影响地启动、运行并正常退出。
//!
//! # 核心设计与测试内存布局
//! 测试在原候选映像的用户地址空间中映射了一块匿名暂存区（`SCRATCH`），特意将指针数组与字符串跨越物理页边界摆放，
//! 以检验 `copyin` / `read_c_string` 在跨页及非对齐场景下的正确性：
//!
//! ```text
//! SCRATCH (0x400000) 匿名可读写映射区 (共 40 页 = 32 页字符串容量 + 8 页辅助区)
//! +-----------------------+-------------------------------------------------------+
//! | SCRATCH + 1*PAGE - 4  | ARGV: 跨第 1/2 页边界且非 8 字节自然对齐的 argv 指针表 |
//! | SCRATCH + 2*PAGE - 4  | ENVP: 跨第 2/3 页边界且非 8 字节自然对齐的 envp 指针表 |
//! | SCRATCH + 3*PAGE - 3  | STRING: 跨页起始的参数字符串缓冲区 (延伸至后方 32 页)  |
//! | SCRATCH + 4*PAGE - 2  | RAW: 跨页的非 UTF-8 原始字节参数                      |
//! | SCRATCH + 5*PAGE - 4  | ENVIRONMENT: 跨页的环境变量字符串                     |
//! +-----------------------+-------------------------------------------------------+
//! | SCRATCH + 40*PAGE     | unmapped: 紧邻暂存区末尾的未映射虚拟页边界            |
//! +-----------------------+-------------------------------------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块大部分阶段在普通可睡眠测试线程上下文中顺序执行。
//! - 在 `allocation_failure` 构造堆内存耗尽（OOM）压力时，必须先持有 `PreemptGuard` 关闭本地抢占并固定在当前 CPU 上，
//!   且在释放内存压力（`release_pressure`）前严禁执行任何可能触发磁盘 I/O 或睡眠等待的操作；期间保持中断开启以允许 SLUB 排空所需的核间中断（IPI）。

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::mem::size_of;

use ::ktest::{TestContext, TestError, TestResult};

use crate::arch;
use crate::fs::ktest::user::{UserOwner, PROGRAM};
use crate::fs::FileRef;
use crate::mm::space::{Mm, VmError};
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;
use crate::sync::PreemptGuard;
use crate::task::exec::{self, Arguments, ExecError, LoadedImage};
use crate::task::process::{ProcessError, UserIdentity};
use crate::task::signal::SignalInheritance;
use crate::task::{prepare_program, Credentials, Placement, ProgramError, UserRuntime};

/// 当前架构下单个指针/机器字的字节宽度（64 位架构下为 8 字节）。
const WORD: usize = size_of::<usize>();
/// 用户态测试暂存映射区的起始虚拟地址。
const SCRATCH: usize = 0x400000;
/// 单个参数字符串允许的最大字节数上限（32 个物理页，含末尾 NUL 字节）。
const ARGUMENT_STRING_BYTES: usize = 32 * PAGE_SIZE;
/// 测试暂存映射区的总字节大小（32 页最大字符串空间加 8 页前置跨页测试区）。
const SCRATCH_BYTES: usize = ARGUMENT_STRING_BYTES + 8 * PAGE_SIZE;
/// 刻意设置在页尾前 4 字节处的 `argv` 指针表地址，用于测试非自然对齐与跨页读取。
const ARGV: usize = SCRATCH + PAGE_SIZE - 4;
/// 刻意设置在页尾前 4 字节处的 `envp` 指针表地址，用于测试非自然对齐与跨页读取。
const ENVP: usize = SCRATCH + 2 * PAGE_SIZE - 4;
/// 刻意设置在页尾前 3 字节处的长参数字符串起始地址，用于测试跨页字符串拷贝。
const STRING: usize = SCRATCH + 3 * PAGE_SIZE - 3;
/// 刻意设置在页尾前 2 字节处的原始非 UTF-8 字节串起始地址。
const RAW: usize = SCRATCH + 4 * PAGE_SIZE - 2;
/// 刻意设置在页尾前 4 字节处的环境变量字符串起始地址。
const ENVIRONMENT: usize = SCRATCH + 5 * PAGE_SIZE - 4;

/// 测试暂存区使用的用户页权限：可读、可写、不可执行。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 将 `ExecError` 转换为测试框架错误，并在非内存压力阶段输出详细的诊断日志。
fn checked_exec<T>(
    context: &TestContext<'_>,
    result: Result<T, ExecError>,
) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("user preparation exec error={:?}", error));
        TestError("user preparation exec operation failed")
    })
}

/// 将虚拟内存访问错误 `VmError` 转换为测试框架错误并记录诊断信息。
fn checked_vm<T>(context: &TestContext<'_>, result: Result<T, VmError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("user preparation VM error={:?}", error));
        TestError("user preparation VM operation failed")
    })
}

/// 分配一份包含默认测试程序路径（`PROGRAM`）的独立堆内存字节向量，若堆分配失败则返回 `ExecError::NoMemory`。
fn owned_filename() -> Result<Vec<u8>, ExecError> {
    let mut filename = Vec::new();
    filename
        .try_reserve_exact(PROGRAM.len())
        .map_err(|_| ExecError::NoMemory)?;
    filename.extend_from_slice(PROGRAM);
    Ok(filename)
}

/// 通过标准的用户内存拷贝接口（`Mm::copyout`）向指定用户虚拟地址写入字节切片，并校验全部字节均已完整写入。
fn write_bytes(context: &TestContext<'_>, mm: &Mm, address: usize, bytes: &[u8]) -> TestResult {
    let progress = mm.copyout(address, bytes);
    if let Some(error) = progress.error {
        return checked_vm(context, Err(error));
    }
    if progress.copied != bytes.len() {
        return Err(TestError("user preparation input copy incomplete"));
    }
    Ok(())
}

/// 将一组指针按小端机器字依次写入用户地址空间，并在末尾追加一个值为 `0` 的 NULL 终止指针。
fn write_vector(
    context: &TestContext<'_>,
    mm: &Mm,
    address: usize,
    pointers: &[usize],
) -> TestResult {
    for (index, pointer) in pointers.iter().enumerate() {
        write_bytes(context, mm, address + index * WORD, &pointer.to_le_bytes())?;
    }
    write_bytes(
        context,
        mm,
        address + pointers.len() * WORD,
        &0usize.to_le_bytes(),
    )
}

/// 读取已完成装载但尚未发布运行的 ELF 映像初始用户栈，校验 `argc`、`argv`、`envp` 指针表、NULL 终止符及对应的字符串内容是否与预期完全一致。
fn check_stack(
    context: &mut TestContext<'_>,
    image: &LoadedImage,
    argv: &[&[u8]],
    envp: &[&[u8]],
) -> TestResult {
    #[cfg(target_arch = "riscv64")]
    let sp = image.context.gpr[2];
    #[cfg(target_arch = "loongarch64")]
    let sp = image.context.gpr[3];
    context.expect_eq(checked_vm(context, image.mm.read_usize(sp))?, argv.len());
    let mut slot = sp + WORD;
    for strings in [argv, envp] {
        for expected in strings {
            let pointer = checked_vm(context, image.mm.read_usize(slot))?;
            let actual = image
                .mm
                .read_c_string(pointer, expected.len() + 1)
                .map_err(|error| {
                    context.diagnostic(format_args!(
                        "user preparation stack string error={:?}",
                        error
                    ));
                    TestError("user preparation stack string read failed")
                })?;
            context.expect_eq(actual.as_slice(), *expected);
            slot += WORD;
        }
        context.expect_eq(checked_vm(context, image.mm.read_usize(slot))?, 0);
        slot += WORD;
    }
    Ok(())
}

/// 验证 `Arguments::from_user` 的深拷贝快照隔离性：
/// 在跨页地址构造 `argv` 与 `envp` 并完成内核快照提取后，立即篡改用户空间中原有的指针表与字符串内容，
/// 随后执行 `exec::load` 装载 ELF，验证构建出的新初始用户栈只包含快照提取时的原始数据。
fn snapshot_arguments(
    context: &mut TestContext<'_>,
    fixture: &UserOwner,
    mm: &Mm,
    file: &FileRef,
) -> TestResult {
    let raw: &[u8] = b"raw\xff\x81";
    write_bytes(context, mm, STRING, PROGRAM)?;
    write_bytes(context, mm, STRING + PROGRAM.len(), &[0])?;
    write_bytes(context, mm, RAW, raw)?;
    write_bytes(context, mm, RAW + raw.len(), &[0])?;
    write_bytes(context, mm, ENVIRONMENT, b"USER_SMOKE=1\0")?;
    write_vector(context, mm, ARGV, &[STRING, RAW])?;
    write_vector(context, mm, ENVP, &[ENVIRONMENT])?;

    let arguments = checked_exec(
        context,
        Arguments::from_user(
            mm,
            checked_exec(context, owned_filename())?,
            ARGV,
            ENVP,
            exec::DEFAULT_STACK_BYTES,
        ),
    )?;

    write_vector(context, mm, ARGV, &[])?;
    write_vector(context, mm, ENVP, &[])?;
    write_bytes(context, mm, STRING, b"changed\0")?;
    write_bytes(context, mm, RAW, b"bad\0")?;
    write_bytes(context, mm, ENVIRONMENT, b"broken\0")?;

    let image = checked_exec(
        context,
        exec::load(
            fixture.path.vfs(),
            fixture.path.context(),
            Arc::clone(file),
            &arguments,
            Credentials::ROOT,
        ),
    )?;
    check_stack(context, &image, &[PROGRAM, raw], &[b"USER_SMOKE=1"])
}

/// 验证 `Arguments::from_user` 对各类非法用户内存输入与超长未终止字符串的错误码语义：
/// 1. 指针表或字符串指针位于未映射页、超出用户地址空间上限或发生整数加法溢出时，必须原样返回对应的 `ExecError::Vm(...)`（如 `Unmapped` 或 `InvalidRange`）。
/// 2. 字符串在未映射页边界前只有非 NUL 字节且跨入未映射页时，必须返回 `VmError::Unmapped` 而非误报参数过长。
/// 3. 字符串在已映射页内连续达到 32 页且始终没有 NUL 终止符时，必须返回 `ExecError::ArgumentsTooBig`（`E2BIG`）。
fn invalid_arguments(context: &mut TestContext<'_>, mm: &Mm, source: &[u8]) -> TestResult {
    let unmapped = SCRATCH + SCRATCH_BYTES;
    for (table, cause) in [
        (unmapped, VmError::Unmapped),
        (arch::USER_ADDRESS_END - WORD + 1, VmError::InvalidRange),
        (usize::MAX - WORD + 1, VmError::InvalidRange),
    ] {
        let error = Arguments::from_user(
            mm,
            checked_exec(context, owned_filename())?,
            table,
            0,
            exec::DEFAULT_STACK_BYTES,
        )
        .err();
        context.diagnostic(format_args!(
            "user preparation bad vector address={:#x} error={:?}",
            table, error
        ));
        context.expect_eq(error, Some(ExecError::Vm(cause)));
    }

    for (pointer, cause) in [
        (unmapped, VmError::Unmapped),
        (arch::USER_ADDRESS_END, VmError::InvalidRange),
        (usize::MAX - WORD, VmError::InvalidRange),
    ] {
        write_vector(context, mm, ARGV, &[pointer])?;
        let error = Arguments::from_user(
            mm,
            checked_exec(context, owned_filename())?,
            ARGV,
            0,
            exec::DEFAULT_STACK_BYTES,
        )
        .err();
        context.diagnostic(format_args!(
            "user preparation bad string address={:#x} error={:?}",
            pointer, error
        ));
        context.expect_eq(error, Some(ExecError::Vm(cause)));
    }

    // Step 3: 在未映射边界前 3 字节写入无 NUL 结尾的字符，验证跨入未映射页时精确报告 Unmapped 错误而非 E2BIG。
    write_bytes(context, mm, unmapped - 3, b"abc")?;
    write_vector(context, mm, ARGV, &[unmapped - 3])?;
    context.expect_eq(
        Arguments::from_user(
            mm,
            checked_exec(context, owned_filename())?,
            ARGV,
            0,
            exec::DEFAULT_STACK_BYTES,
        )
        .err(),
        Some(ExecError::Vm(VmError::Unmapped)),
    );

    write_bytes(context, mm, STRING, source)?;
    write_vector(context, mm, ARGV, &[STRING])?;
    let error = Arguments::from_user(
        mm,
        checked_exec(context, owned_filename())?,
        ARGV,
        0,
        exec::DEFAULT_STACK_BYTES,
    )
    .err();
    context.diagnostic(format_args!(
        "user preparation unterminated string error={:?}",
        error
    ));
    context.expect_eq(error, Some(ExecError::ArgumentsTooBig));
    Ok(())
}

/// 验证当 `argv` 指针本身为 `NULL`（0）或指向一个仅含终止符的空表时，
/// `exec::load` 均会在初始用户栈上保底填入一个空字符串 `argv[0] = ""`（`argc = 1`）并维持完整的 NULL 边界。
fn argument_count(
    context: &mut TestContext<'_>,
    fixture: &UserOwner,
    mm: &Mm,
    file: &FileRef,
) -> TestResult {
    write_vector(context, mm, ARGV, &[])?;
    for table in [0, ARGV] {
        let arguments = checked_exec(
            context,
            Arguments::from_user(
                mm,
                checked_exec(context, owned_filename())?,
                table,
                0,
                exec::DEFAULT_STACK_BYTES,
            ),
        )?;
        let image = checked_exec(
            context,
            exec::load(
                fixture.path.vfs(),
                fixture.path.context(),
                Arc::clone(file),
                &arguments,
                Credentials::ROOT,
            ),
        )?;
        check_stack(context, &image, &[b""], &[])?;
    }
    Ok(())
}

/// 验证单个参数字符串的 32 页容量上限边界：
/// 1. 包含末尾 NUL 在内恰好占满 `32 * PAGE_SIZE` 字节的单条参数字符串能够正常提取并装载到初始栈。
/// 2. 将末尾 NUL 推迟 1 字节使其总长度超过 `32 * PAGE_SIZE` 时，即使后方虚拟页依然可读，也必须返回 `ExecError::ArgumentsTooBig`（`E2BIG`）。
fn argument_bytes(
    context: &mut TestContext<'_>,
    fixture: &UserOwner,
    mm: &Mm,
    file: &FileRef,
    source: &[u8],
) -> TestResult {
    let length = ARGUMENT_STRING_BYTES - 1;
    write_bytes(context, mm, STRING, source)?;
    write_bytes(context, mm, STRING + length, &[0])?;
    write_vector(context, mm, ARGV, &[STRING])?;
    let arguments = checked_exec(
        context,
        Arguments::from_user(
            mm,
            checked_exec(context, owned_filename())?,
            ARGV,
            0,
            exec::DEFAULT_STACK_BYTES,
        ),
    )?;
    let image = checked_exec(
        context,
        exec::load(
            fixture.path.vfs(),
            fixture.path.context(),
            Arc::clone(file),
            &arguments,
            Credentials::ROOT,
        ),
    )?;
    check_stack(context, &image, &[&source[..length]], &[])?;
    drop(image);
    drop(arguments);

    write_bytes(context, mm, STRING + length, b"x\0")?;
    let error = Arguments::from_user(
        mm,
        checked_exec(context, owned_filename())?,
        ARGV,
        0,
        exec::DEFAULT_STACK_BYTES,
    )
    .err();
    context.diagnostic(format_args!(
        "user preparation single argument string overflow error={:?}",
        error
    ));
    context.expect_eq(error, Some(ExecError::ArgumentsTooBig));
    Ok(())
}

/// 基于当前测试夹具的文件系统上下文与网络命名空间，创建一份全新的用户态运行资源包（`UserRuntime`）。
fn runtime(context: &TestContext<'_>, fixture: &UserOwner) -> Result<UserRuntime, TestError> {
    UserRuntime::try_new(
        Arc::clone(fixture.path.vfs()),
        fixture.path.context().clone(),
        Credentials::ROOT,
        fixture.net.clone(),
    )
    .map_err(|error| {
        context.diagnostic(format_args!("user preparation runtime error={:?}", error));
        TestError("user preparation runtime unavailable")
    })
}

/// 验证未发布的半成品进程（`PreparedProcess` 与 `PreparedProgram`）在中途放弃时的回滚与 PID 复用语义：
/// 1. 仅预留进程身份（`prepare_process`）时，线程表与 PID 映射表中均不可见，`try_wait` 返回 `NoChild`；丢弃该预留句柄后 PID 立即归还。
/// 2. 完成完整 ELF 装载与任务栈构建（`prepare_program`）后直接丢弃该未发布程序，不仅会复用刚才归还的同一个 PID，且丢弃后再次释放该 PID 供后续分配复用。
///
/// 返回最初预留的 `UserIdentity` 强引用，用于验证即使外部仍持有旧身份结构体引用，也不会阻碍内核回收并复用其底层数值编号。
fn abandon_program(
    context: &mut TestContext<'_>,
    fixture: &UserOwner,
) -> Result<Arc<UserIdentity>, TestError> {
    let graph = fixture.graph();
    let runtime = runtime(context, fixture)?;
    let sighand = Arc::clone(&runtime.sighand);
    let reserved = graph
        .prepare_process(
            None,
            Arc::clone(&sighand),
            Credentials::ROOT.uid,
            SignalInheritance::EMPTY,
            crate::task::signal::SIGCHLD,
        )
        .map_err(|error| {
            context.diagnostic(format_args!(
                "user preparation reservation error={:?}",
                error
            ));
            TestError("user preparation reservation failed")
        })?;
    let old_identity = reserved.identity();
    let number = old_identity.tid();
    let old_pid = old_identity.tid_object();
    context.expect_eq(graph.lookup_thread(number).is_none(), true);
    context.expect_eq(graph.lookup_pid(&old_pid).is_none(), true);
    context.expect_eq(
        graph.try_wait(None, number, true),
        Err(ProcessError::NoChild),
    );
    drop(reserved);

    let image = fixture.load(context)?;
    let program = prepare_program(
        graph,
        None,
        image,
        runtime,
        Placement::Cpu(crate::cpu::current().id),
    )
    .map_err(|error| {
        context.diagnostic(format_args!(
            "user preparation complete candidate error={:?}",
            error
        ));
        TestError("user preparation complete candidate failed")
    })?;
    context.expect_eq(program.pid(), number);
    context.expect_eq(graph.lookup_thread(number).is_none(), true);
    context.expect_eq(graph.lookup_pid(&old_pid).is_none(), true);
    context.expect_eq(
        graph.try_wait(None, number, true),
        Err(ProcessError::NoChild),
    );
    // Step 2: 此时 PreparedProgram 已分配完整的用户地址空间、运行资源与内核栈，但尚未提交到调度器就绪队列；直接 drop 触发未发布回滚。
    drop(program);

    let reused = graph
        .prepare_process(
            None,
            sighand,
            Credentials::ROOT.uid,
            SignalInheritance::EMPTY,
            crate::task::signal::SIGCHLD,
        )
        .map_err(|error| {
            context.diagnostic(format_args!(
                "user preparation cancellation reuse error={:?}",
                error
            ));
            TestError("user preparation cancellation did not return number")
        })?;
    context.expect_eq(reused.identity().tid(), number);
    context.expect_eq(graph.lookup_thread(number).is_none(), true);
    context.expect_eq(graph.lookup_pid(&old_pid).is_none(), true);
    context.diagnostic(format_args!(
        "user preparation complete unpublished candidate dropped pid={}",
        number
    ));
    drop(reused);
    Ok(old_identity)
}

/// 验证在堆内存耗尽（OOM）压力下，参数提取（`Arguments::from_kernel`）与程序准备（`prepare_program`）
/// 能够安全中止并返回 `NoMemory`，且不会泄漏任何半成品线程或占用刚才释放的 PID 编号。
fn allocation_failure(
    context: &mut TestContext<'_>,
    fixture: &mut UserOwner,
    source: &[u8],
    old_identity: &UserIdentity,
) -> TestResult {
    let graph = Arc::clone(fixture.graph());
    let image = fixture.load(context)?;
    let runtime = runtime(context, fixture)?;
    let sighand = Arc::clone(&runtime.sighand);
    let placement = Placement::Cpu(crate::cpu::current().id);
    let old_pid = old_identity.tid_object();
    let number = old_identity.tid();

    // Step 1: 所有涉及磁盘 I/O 的文件读取与 ELF 装载均已在上方提前完成。
    // 在关闭抢占的临界区内抽干当前 CPU 的堆内存分配器以触发 OOM，测完立即释放内存压力，避免影响诊断日志输出。
    let (evidence, arguments, program) = {
        let _pin = PreemptGuard::new();
        let evidence = fixture.pressure().exhaust();
        let arguments = Arguments::from_kernel(
            PROGRAM,
            &[&source[..3 * PAGE_SIZE]],
            &[],
            exec::DEFAULT_STACK_BYTES,
        );
        let program = prepare_program(&graph, None, image, runtime, placement);
        fixture.release_pressure();
        (evidence, arguments, program)
    };

    let argument_error = arguments.err();
    let program_error = program.err();
    context.diagnostic(format_args!(
        "user preparation actual allocation failures evidence={:?} argument={:?} program={:?}",
        evidence, argument_error, program_error,
    ));
    context.expect_eq(evidence, (true, true));
    context.expect_eq(argument_error, Some(ExecError::NoMemory));
    context.expect_eq(
        program_error,
        Some(ProgramError::Identity(ProcessError::NoMemory)),
    );
    context.expect_eq(graph.lookup_thread(number).is_none(), true);
    context.expect_eq(graph.lookup_pid(&old_pid).is_none(), true);
    context.expect_eq(
        graph.try_wait(None, number, true),
        Err(ProcessError::NoChild),
    );

    let reused = graph
        .prepare_process(
            None,
            sighand,
            Credentials::ROOT.uid,
            SignalInheritance::EMPTY,
            crate::task::signal::SIGCHLD,
        )
        .map_err(|error| {
            context.diagnostic(format_args!(
                "user preparation allocation failure reuse error={:?}",
                error
            ));
            TestError("user preparation allocation failure did not return number")
        })?;
    context.expect_eq(reused.identity().tid(), number);
    drop(reused);
    Ok(())
}

/// 执行用户程序准备阶段的全部异常、边界与回滚测试，最后启动最初加载的正常 ELF 映像并等待其以状态码 0 退出。
pub(super) fn run(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
    fixture.prepare(context)?;
    let old_image = fixture.load(context)?;
    checked_vm(
        context,
        old_image.mm.map_anonymous(SCRATCH, SCRATCH_BYTES, RW, RW),
    )?;
    let file = fixture.file(context, PROGRAM)?;
    let mut source = Vec::new();
    source
        .try_reserve_exact(ARGUMENT_STRING_BYTES)
        .map_err(|_| TestError("user preparation source bytes OOM"))?;
    source.resize(ARGUMENT_STRING_BYTES, b'x');

    snapshot_arguments(context, fixture, &old_image.mm, &file)?;
    invalid_arguments(context, &old_image.mm, &source)?;
    argument_count(context, fixture, &old_image.mm, &file)?;
    argument_bytes(context, fixture, &old_image.mm, &file, &source)?;
    let old_identity = abandon_program(context, fixture)?;
    allocation_failure(context, fixture, &source, &old_identity)?;
    drop(source);
    drop(file);

    let number = old_identity.tid();
    let old_pid = old_identity.tid_object();
    fixture.start(context, old_image)?;
    context.expect_eq(fixture.graph().lookup_pid(&old_pid).is_none(), true);
    let event = fixture.wait(context)?;
    context.expect_eq(event.pid, number);
    context.expect_eq(event.status, 0);
    context.expect_eq(fixture.graph().lookup_thread(number).is_none(), true);
    context.expect_eq(fixture.graph().lookup_pid(&old_pid).is_none(), true);
    context.diagnostic(format_args!(
        "user preparation original ELF completed pid={} status={} task={:?} user_interrupts={}",
        event.pid, event.status, event.task_id, event.user_interrupts,
    ));
    drop(old_pid);
    drop(old_identity);
    Ok(())
}
