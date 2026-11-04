//! 真实用户参数、未发表程序和分配失败的归还检查，最后执行原正常磁盘映像。

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

const WORD: usize = size_of::<usize>();
const SCRATCH: usize = 0x400000;
const ARGUMENT_STRING_BYTES: usize = 32 * PAGE_SIZE;
const SCRATCH_BYTES: usize = ARGUMENT_STRING_BYTES + 8 * PAGE_SIZE;
const ARGV: usize = SCRATCH + PAGE_SIZE - 4;
const ENVP: usize = SCRATCH + 2 * PAGE_SIZE - 4;
const STRING: usize = SCRATCH + 3 * PAGE_SIZE - 3;
const RAW: usize = SCRATCH + 4 * PAGE_SIZE - 2;
const ENVIRONMENT: usize = SCRATCH + 5 * PAGE_SIZE - 4;
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

/// 保留具体准备原因，诊断只在没有堆压力时交给原用例上下文。
fn checked_exec<T>(
    context: &TestContext<'_>,
    result: Result<T, ExecError>,
) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("user preparation exec error={:?}", error));
        TestError("user preparation exec operation failed")
    })
}

/// 用户访问准备失败保留原 VM 原因，不把短复制当作完整输入。
fn checked_vm<T>(context: &TestContext<'_>, result: Result<T, VmError>) -> Result<T, TestError> {
    result.map_err(|error| {
        context.diagnostic(format_args!("user preparation VM error={:?}", error));
        TestError("user preparation VM operation failed")
    })
}

/// 为用户指针复制准备独立拥有的文件名，真实分配失败保留为参数准备错误。
fn owned_filename() -> Result<Vec<u8>, ExecError> {
    let mut filename = Vec::new();
    filename
        .try_reserve_exact(PROGRAM.len())
        .map_err(|_| ExecError::NoMemory)?;
    filename.extend_from_slice(PROGRAM);
    Ok(filename)
}

/// 通过正式 uaccess 写入已初始化内核字节，不创建用户内存的 Rust 引用。
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

/// 每个指针用完整初始化的小端机器字写入，终止项与非自然对齐项都经过真实用户页。
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

/// 读取未发表完整 ELF 的真实初始栈，比较拥有快照产生的每个字符串及两个 NULL 边界。
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

/// 指针表与字符串均跨页；复制后修改原表和原串，同一文件系统视图装载时只能使用此前拥有的字节。
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

/// 实际坏表、坏字符串和地址加法边界保留 VM 原因；已映射无 NUL 输入保留 E2BIG。
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
    // 三个可读非 NUL 字节之后确实没有 VMA，跨页失败不能替换为预算耗尽。
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

/// NULL 与显式空 argv 都在真实初始栈上产生一个空 argv[0] 和完整的终止边界。
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

/// 单串含 NUL 可占32页，超过该字节边界仍返回 E2BIG，后方可读页不能改变这个结果。
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

/// 使用已有 FS 上下文与整组共享的网络域预备运行资源，不发表任何用户 Task。
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

/// 保持旧身份引用存活，放弃已完成用户对象与 Task 准备的候选后同图立即复用原号码。
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
    // PreparedProgram 已持有完整用户现场、运行资源和真实执行栈，但未移交 Ready 队列。
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

/// 真耗尽后调用参数复制与程序准备，原压力先释放，再记录真实失败并检查预留号归还。
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
    // 所有源字节、真实文件缓存、候选映像及运行资源已准备；此段无睡眠访问。
    // 保持同一 CPU 的堆类别耗尽状态，仍允许 IRQ 与同步 SLUB 排空所需的 IPI。
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

/// 在原正常映像的 Mm 和文件系统视图中验证坏参数与真实 OOM，候选全部归还后执行该旧映像并等待真实退出。
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
