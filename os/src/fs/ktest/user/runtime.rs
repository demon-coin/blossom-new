//! 原比赛聚合 ELF 从真实族目录建立 FS 快照，装载断言读取实际用户栈和磁盘程序头。

use crate::arch::UserAccess;
use crate::mm::PAGE_SIZE;
use crate::sync::SpinLock;
use core::convert::TryInto;

use crate::fs::ktest::user::*;

const ENTRY: &[u8] = b"/entry-dynamic.exe";
const LIBRARY_ENV: &[u8] = b"LD_LIBRARY_PATH=/lib";

struct Header {
    kind: u16,
    entry: usize,
    phoff: u64,
    phnum: usize,
}

struct Auxv {
    phdr: usize,
    phent: usize,
    phnum: usize,
    base: usize,
    entry: usize,
}

impl UserOwner {
    /// 仅用户 Runtime 取得族根快照，父 PathOwner 保持原盘 root/cwd 和既有失败清理责任。
    pub(super) fn runtime_dynamic(
        &mut self,
        context: &mut TestContext<'_>,
        family: &[u8],
        selector: &[u8],
    ) -> TestResult {
        assert!(
            matches!(selector, b"argv" | b"env"),
            "unknown runtime selector"
        );
        self.prepare(context)?;
        let parent = self.path.context().clone();
        let root = checked(
            context,
            self.path
                .vfs()
                .resolve(&parent, family, ResolveMode::Follow),
        )?;
        let fs_context = checked(context, FsContext::new(root.clone(), root))?;
        let main = open(context, self, &fs_context, ENTRY)?;
        let header = read_header(context, &main)?;
        let mut interpreter_bytes = [0; 4096];
        let interpreter = read_interpreter(context, &main, &header, &mut interpreter_bytes)?;
        let arguments = Arguments::from_kernel(
            ENTRY,
            &[ENTRY, selector],
            &[LIBRARY_ENV],
            exec::DEFAULT_STACK_BYTES,
        )
        .map_err(|error| {
            context.diagnostic(format_args!("runtime-arguments error={:?}", error));
            TestError("runtime arguments preparation failed")
        })?;
        let image = prepare_image(self, context, &fs_context, arguments)?;
        verify_image(
            context,
            &image,
            &main,
            &header,
            Some(interpreter),
            self,
            &fs_context,
            ENTRY,
            &[ENTRY, selector],
        )?;
        drop(main);

        let mm = image.mm.clone();
        self.start_program(context, image, fs_context, false, false, Credentials::ROOT)?;
        let event = self.wait(context)?;
        let cancelling = event.status != 0;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        context.expect_eq(event.status, 0);
        context.expect_eq(mm.user_count(), 0);
        assert!(matches!(
            self.graph().wait_root(),
            Err(crate::task::process::ProcessError::NoChild)
        ));
        assert!(self.path.context().root().ptr_eq(parent.root()));
        assert!(self.path.context().cwd().ptr_eq(parent.cwd()));
        context.diagnostic(format_args!(
            "runtime-result family={:?} selector={:?} status={} joined=1 mm-users={} root-children=0",
            family, selector, event.status, mm.user_count()
        ));
        if cancelling {
            return Err(TestError("real runtime selector exited unsuccessfully"));
        }
        Ok(())
    }

    /// 直接装载同族原 loader，初始 auxv 将它作为主映像，随后用户 loader 自己装入原程序。
    pub(super) fn runtime_loader(
        &mut self,
        context: &mut TestContext<'_>,
        family: &[u8],
    ) -> TestResult {
        self.prepare(context)?;
        let fs_context = family_context(self, context, family)?;
        let main = open(context, self, &fs_context, ENTRY)?;
        let main_header = read_header(context, &main)?;
        let mut interpreter_bytes = [0; 4096];
        let interpreter = read_interpreter(context, &main, &main_header, &mut interpreter_bytes)?;
        let loader = open(context, self, &fs_context, interpreter)?;
        let header = read_header(context, &loader)?;
        assert_eq!(header.kind, 3, "direct loader is not ET_DYN");
        for index in 0..header.phnum {
            let phdr = read_phdr(context, &loader, &header, index)?;
            assert_ne!(phdr_kind(&phdr), 3, "direct loader has PT_INTERP");
        }

        let argv = [interpreter, ENTRY, b"argv" as &[u8]];
        let arguments = runtime_arguments(context, interpreter, &argv)?;
        let image = prepare_image(self, context, &fs_context, arguments)?;
        verify_image(
            context,
            &image,
            &loader,
            &header,
            None,
            self,
            &fs_context,
            interpreter,
            &argv,
        )?;
        drop(loader);
        drop(main);
        self.run_runtime(context, image, fs_context)
    }

    /// fork 的两映像真实执行占用随 Mm 复制；父释放、用户退出与末拥有释放分别观察。
    pub(super) fn runtime_fork(
        &mut self,
        context: &mut TestContext<'_>,
        family: &[u8],
    ) -> TestResult {
        self.prepare(context)?;
        let fs_context = family_context(self, context, family)?;
        let main = open(context, self, &fs_context, ENTRY)?;
        let header = read_header(context, &main)?;
        let mut interpreter_bytes = [0; 4096];
        let interpreter = read_interpreter(context, &main, &header, &mut interpreter_bytes)?;
        let loader = open(context, self, &fs_context, interpreter)?;
        let arguments = runtime_arguments(context, ENTRY, &[ENTRY, b"argv"])?;
        let image = prepare_image(self, context, &fs_context, arguments)?;
        verify_image(
            context,
            &image,
            &main,
            &header,
            Some(interpreter),
            self,
            &fs_context,
            ENTRY,
            &[ENTRY, b"argv"],
        )?;
        assert_denied(&main, &loader);

        let child = Arc::try_new(image.mm.fork().map_err(|error| {
            context.diagnostic(format_args!("runtime-fork error={:?}", error));
            TestError("runtime two-image Mm fork failed")
        })?)
        .map_err(|_| TestError("runtime fork Mm owner OOM"))?;
        let user_context = image.context;
        drop(image.mm);
        assert_denied(&main, &loader);
        let image = LoadedImage {
            mm: child.clone(),
            context: user_context,
        };
        verify_image(
            context,
            &image,
            &main,
            &header,
            Some(interpreter),
            self,
            &fs_context,
            ENTRY,
            &[ENTRY, b"argv"],
        )?;
        self.run_runtime(context, image, fs_context)?;
        assert_eq!(child.user_count(), 0, "runtime fork still has users");
        assert_denied(&main, &loader);

        let weak = Arc::downgrade(&child);
        drop(child);
        assert!(weak.upgrade().is_none(), "runtime fork last Mm still owned");
        assert_writable(context, &main, &loader)?;
        context.diagnostic(format_args!(
            "runtime-fork family={:?} parent-released=1 joined=1 both-denied-until-last-mm=1 both-write-prepared=1",
            family
        ));
        Ok(())
    }

    /// 普通解释器准备错误与真实页堆耗尽不修改旧现场，也不归还旧主文件和 loader 的占用。
    pub(super) fn runtime_failures(
        &mut self,
        context: &mut TestContext<'_>,
        family: &[u8],
    ) -> TestResult {
        self.prepare(context)?;
        let fs_context = family_context(self, context, family)?;
        let main = open(context, self, &fs_context, ENTRY)?;
        let header = read_header(context, &main)?;
        let mut interpreter_bytes = [0; 4096];
        let interpreter = read_interpreter(context, &main, &header, &mut interpreter_bytes)?;
        let loader = open(context, self, &fs_context, interpreter)?;
        let image = prepare_image(
            self,
            context,
            &fs_context,
            runtime_arguments(context, ENTRY, &[ENTRY, b"argv"])?,
        )?;
        verify_image(
            context,
            &image,
            &main,
            &header,
            Some(interpreter),
            self,
            &fs_context,
            ENTRY,
            &[ENTRY, b"argv"],
        )?;
        let original_pc = image.context.pc;
        let original_break = image.mm.brk(0);

        for (pathname, expected) in [
            (b"/elf-missing-interpreter" as &[u8], FsError::NotFound),
            (b"/elf-directory-interpreter" as &[u8], FsError::Permission),
        ] {
            let candidate = open(context, self, &fs_context, pathname)?;
            let arguments = runtime_arguments(context, pathname, &[pathname])?;
            let error =
                exec::prepare(self.path.vfs(), &fs_context, arguments, Credentials::ROOT).err();
            assert_eq!(
                error,
                Some(ExecError::File(expected)),
                "runtime interpreter error"
            );
            drop(checked(
                context,
                candidate.mapping_inode().unwrap().acquire_write(),
            )?);
            assert_denied(&main, &loader);
        }

        let arguments = runtime_arguments(context, ENTRY, &[ENTRY, b"argv"])?;
        let vfs = self.path.vfs().clone();
        let candidate = main.clone();
        let header_folio = main
            .mapping_inode()
            .unwrap()
            .mapping()
            .cached_folio(0)
            .ok_or(TestError("runtime main header folio was not warmed"))?;
        let mut pressure = self
            .path
            .owner
            .pressure
            .take()
            .expect("runtime pressure was not prepared");
        let observed = Arc::try_new(SpinLock::new(None))
            .map_err(|_| TestError("runtime OOM result owner allocation failed"))?;
        let child_observed = observed.clone();
        let candidate_context = fs_context.clone();
        // 控制任务的唯一 join 责任由 fixture 持有，须在文件系统资源比较前完整消费。
        self.observer = Some(
            crate::task::spawn_kernel_on(cpu::current().id, move || {
                let evidence = pressure.exhaust();
                let result = exec::load(
                    &vfs,
                    &candidate_context,
                    candidate,
                    &arguments,
                    Credentials::ROOT,
                );
                drop(pressure);
                *child_observed.lock() = Some((evidence, result));
                0
            })
            .map_err(|_| TestError("runtime OOM controller allocation failed"))?,
        );

        let child = self.observer.take().expect("runtime OOM controller absent");
        context.expect_eq(child.join(), 0);
        let (evidence, result) = observed
            .lock()
            .take()
            .expect("runtime OOM child completed without result");
        drop(observed);
        drop(header_folio);
        assert_eq!(
            evidence,
            (true, true),
            "runtime preparation pressure was incomplete"
        );
        assert_eq!(
            result.err(),
            Some(ExecError::NoMemory),
            "runtime candidate OOM"
        );
        assert_eq!(image.context.pc, original_pc, "runtime old PC changed");
        assert_eq!(image.mm.brk(0), original_break, "runtime old brk changed");
        assert_denied(&main, &loader);
        verify_image(
            context,
            &image,
            &main,
            &header,
            Some(interpreter),
            self,
            &fs_context,
            ENTRY,
            &[ENTRY, b"argv"],
        )?;
        self.run_runtime(context, image, fs_context)?;
        assert_writable(context, &main, &loader)?;
        context.diagnostic(format_args!(
            "runtime-preparation family={:?} missing=NotFound directory=Permission allocation=NoMemory evidence={:?} old-image-exit=0 joined=1 both-write-prepared=1",
            family, evidence
        ));
        Ok(())
    }

    /// 用户退出事件消费后完成真实 join 和余子回收，Mm 观察不代替执行者停止。
    fn run_runtime(
        &mut self,
        context: &mut TestContext<'_>,
        image: LoadedImage,
        fs_context: FsContext,
    ) -> TestResult {
        let mm = image.mm.clone();
        self.start_program(context, image, fs_context, false, false, Credentials::ROOT)?;
        let event = self.wait(context)?;
        let cancelling = event.status != 0;
        if cancelling {
            self.graph().request_root_exit(9);
        }
        self.reap_children(context, cancelling)?;
        assert_eq!(mm.user_count(), 0, "runtime still has active users");
        assert!(matches!(
            self.graph().wait_root(),
            Err(crate::task::process::ProcessError::NoChild)
        ));
        assert_eq!(event.status, 0, "runtime real program failed");
        Ok(())
    }
}

/// 族根只属于本次用户快照，父 PathOwner 的 root 和 cwd 保持原身份。
fn family_context(
    owner: &UserOwner,
    context: &TestContext<'_>,
    family: &[u8],
) -> Result<FsContext, TestError> {
    let root = checked(
        context,
        owner
            .path
            .vfs()
            .resolve(owner.path.context(), family, ResolveMode::Follow),
    )?;
    checked(context, FsContext::new(root.clone(), root))
}

/// 原始参数与唯一库路径进入正式 Arguments，直接 loader 与主程序共用用户栈核对。
fn runtime_arguments(
    context: &TestContext<'_>,
    filename: &[u8],
    argv: &[&[u8]],
) -> Result<Arguments, TestError> {
    Arguments::from_kernel(filename, argv, &[LIBRARY_ENV], exec::DEFAULT_STACK_BYTES).map_err(
        |error| {
            context.diagnostic(format_args!("runtime-arguments error={:?}", error));
            TestError("runtime arguments preparation failed")
        },
    )
}

/// 根调用者显式采用默认栈限额，装载错误保留生产原因。
fn prepare_image(
    owner: &UserOwner,
    context: &TestContext<'_>,
    fs_context: &FsContext,
    arguments: Arguments,
) -> Result<LoadedImage, TestError> {
    exec::prepare(owner.path.vfs(), fs_context, arguments, Credentials::ROOT).map_err(|error| {
        context.diagnostic(format_args!("runtime-image error={:?}", error));
        TestError("real runtime image preparation failed")
    })
}

/// 排写仅由真实 Mm 内的执行占用产生，检查者不创建自己的 ExecAccess。
fn assert_denied(main: &FileRef, loader: &FileRef) {
    for file in [main, loader] {
        assert_eq!(
            file.mapping_inode().unwrap().acquire_write().err(),
            Some(FsError::TextBusy)
        );
    }
}

/// 最后 Mm 释放后取得并结束两份真实 WriteAccess，不修改原程序字节。
fn assert_writable(context: &TestContext<'_>, main: &FileRef, loader: &FileRef) -> TestResult {
    let main_write = checked(context, main.mapping_inode().unwrap().acquire_write())?;
    let loader_write = checked(context, loader.mapping_inode().unwrap().acquire_write())?;
    drop(loader_write);
    drop(main_write);
    Ok(())
}

/// 在同一真实族视图中打开文件，绝对解释器路径不作前缀重写。
fn open(
    context: &TestContext<'_>,
    owner: &UserOwner,
    fs_context: &FsContext,
    pathname: &[u8],
) -> Result<FileRef, TestError> {
    let path = checked(
        context,
        owner
            .path
            .vfs()
            .resolve(fs_context, pathname, ResolveMode::Follow),
    )?;
    checked(context, crate::fs::open_path(owner.path.vfs(), path))
}

/// 真实文件的短读和错误保留位置与已交付长度，不将未读尾部解释为 ELF 元数据。
fn read_file(
    context: &TestContext<'_>,
    file: &FileRef,
    offset: u64,
    bytes: &mut [u8],
) -> TestResult {
    let count = checked(context, file.read_at(offset, bytes))?;
    assert_eq!(count, bytes.len(), "runtime ELF short read at {offset:#x}");
    Ok(())
}

/// 用户地址只通过已拥有 Mm 的 copyin 消费，未发表的映像同样走真实缺页读取。
fn read_mm(context: &TestContext<'_>, mm: &Mm, address: usize, bytes: &mut [u8]) -> TestResult {
    let progress = mm.copyin(bytes, address);
    if progress.error.is_some() || progress.copied != bytes.len() {
        context.diagnostic(format_args!(
            "runtime-copyin address={:#x} copied={} requested={} error={:?}",
            address,
            progress.copied,
            bytes.len(),
            progress.error
        ));
        return Err(TestError("runtime user memory read failed"));
    }
    Ok(())
}

/// ELF64 的固定字段仅从已完整读取的字节数组解码。
fn word(bytes: &[u8], offset: usize) -> usize {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap()) as usize
}

/// 初始栈机器字经 copyin 读取，不借内核生成 auxv 的临时数组证明用户栈。
fn stack_word(context: &TestContext<'_>, mm: &Mm, address: usize) -> Result<usize, TestError> {
    let mut bytes = [0; 8];
    read_mm(context, mm, address, &mut bytes)?;
    Ok(word(&bytes, 0))
}

/// 栈参数比较完整原字节和 NUL，读取范围由本例固定字符串长度决定。
fn stack_string(context: &TestContext<'_>, mm: &Mm, address: usize, expected: &[u8]) -> TestResult {
    let mut bytes = [0; 64];
    assert!(expected.len() < bytes.len());
    read_mm(context, mm, address, &mut bytes[..expected.len() + 1])?;
    assert_eq!(&bytes[..expected.len()], expected, "runtime stack string");
    assert_eq!(bytes[expected.len()], 0, "runtime stack string terminator");
    Ok(())
}

/// 从两架构实际 SP 验证 argc/argv/envp 后逐项取得必要 auxv，重复或缺项均失败。
fn read_auxv(
    context: &TestContext<'_>,
    image: &LoadedImage,
    filename: &[u8],
    argv: &[&[u8]],
) -> Result<Auxv, TestError> {
    #[cfg(target_arch = "riscv64")]
    let sp = image.context.gpr[2];
    #[cfg(target_arch = "loongarch64")]
    let sp = image.context.gpr[3];
    assert_eq!(sp & 15, 0, "runtime stack alignment");
    assert_eq!(
        stack_word(context, &image.mm, sp)?,
        argv.len(),
        "runtime argc"
    );
    let mut cursor = sp + 8;
    for expected in argv {
        stack_string(
            context,
            &image.mm,
            stack_word(context, &image.mm, cursor)?,
            expected,
        )?;
        cursor += 8;
    }
    assert_eq!(
        stack_word(context, &image.mm, cursor)?,
        0,
        "runtime argv terminator"
    );
    cursor += 8;
    stack_string(
        context,
        &image.mm,
        stack_word(context, &image.mm, cursor)?,
        LIBRARY_ENV,
    )?;
    cursor += 8;
    assert_eq!(
        stack_word(context, &image.mm, cursor)?,
        0,
        "runtime envp terminator"
    );
    cursor += 8;

    let kinds = [3, 4, 5, 7, 9, 31];
    let mut values = [None; 6];
    for slot in 0..32 {
        let address = cursor + slot * 16;
        let kind = stack_word(context, &image.mm, address)?;
        let value = stack_word(context, &image.mm, address + 8)?;
        if kind == 0 {
            assert_eq!(value, 0, "runtime AT_NULL");
            assert!(
                values.iter().all(Option::is_some),
                "runtime required auxv missing"
            );
            stack_string(context, &image.mm, values[5].unwrap(), filename)?;
            return Ok(Auxv {
                phdr: values[0].unwrap(),
                phent: values[1].unwrap(),
                phnum: values[2].unwrap(),
                base: values[3].unwrap(),
                entry: values[4].unwrap(),
            });
        }
        if let Some(index) = kinds.iter().position(|expected| *expected == kind) {
            assert!(
                values[index].replace(value).is_none(),
                "runtime duplicate auxv"
            );
        }
    }
    Err(TestError("runtime auxv lacks AT_NULL"))
}

/// 只保留本例需要的真实 ELF64 首头字段，程序头逐项读取，不复制完整映像。
fn read_header(context: &TestContext<'_>, file: &FileRef) -> Result<Header, TestError> {
    let mut bytes = [0; 64];
    read_file(context, file, 0, &mut bytes)?;
    assert_eq!(
        &bytes[..7],
        b"\x7fELF\x02\x01\x01",
        "runtime ELF identification"
    );
    assert_eq!(
        u16::from_le_bytes(bytes[54..56].try_into().unwrap()),
        56,
        "runtime PHENT"
    );
    let kind = u16::from_le_bytes(bytes[16..18].try_into().unwrap());
    assert!(matches!(kind, 2 | 3), "runtime ELF kind");
    let phnum = u16::from_le_bytes(bytes[56..58].try_into().unwrap()) as usize;
    assert!(0 < phnum && phnum <= 128, "runtime ELF PHNUM");
    Ok(Header {
        kind,
        entry: word(&bytes, 24),
        phoff: word(&bytes, 32) as u64,
        phnum,
    })
}

/// 固定大小程序头来自真实文件位置，偏移沿已验证首头计算。
fn read_phdr(
    context: &TestContext<'_>,
    file: &FileRef,
    header: &Header,
    index: usize,
) -> Result<[u8; 56], TestError> {
    let mut bytes = [0; 56];
    read_file(context, file, header.phoff + index as u64 * 56, &mut bytes)?;
    Ok(bytes)
}

/// PT_INTERP 字节由真实主文件给出，只有终止 NUL 前的绝对路径参与正式查找。
fn read_interpreter<'a>(
    context: &TestContext<'_>,
    file: &FileRef,
    header: &Header,
    bytes: &'a mut [u8; 4096],
) -> Result<&'a [u8], TestError> {
    for index in 0..header.phnum {
        let phdr = read_phdr(context, file, header, index)?;
        if u32::from_le_bytes(phdr[..4].try_into().unwrap()) == 3 {
            let length = word(&phdr, 32);
            assert!(1 < length && length <= 4096, "runtime PT_INTERP length");
            read_file(context, file, word(&phdr, 8) as u64, &mut bytes[..length])?;
            assert_eq!(bytes[0], b'/', "runtime interpreter is not absolute");
            assert_eq!(bytes[length - 1], 0, "runtime interpreter terminator");
            assert!(
                !bytes[..length - 1].contains(&0),
                "runtime interpreter interior NUL"
            );
            return Ok(&bytes[..length - 1]);
        }
    }
    Err(TestError("real runtime entry has no PT_INTERP"))
}

/// AT_PHDR 的主文件位置关系推导实际主 bias；解释器 PC 与 AT_BASE 独立核对真实 loader LOAD。
#[expect(
    clippy::too_many_arguments,
    reason = "直接比较装载结果、真实主文件与解释器及原 argv，不把被测结果和独立预期装成同一对象"
)]
fn verify_image(
    context: &TestContext<'_>,
    image: &LoadedImage,
    main: &FileRef,
    header: &Header,
    interpreter: Option<&[u8]>,
    owner: &UserOwner,
    fs_context: &FsContext,
    filename: &[u8],
    argv: &[&[u8]],
) -> TestResult {
    let auxv = read_auxv(context, image, filename, argv)?;
    assert_eq!(auxv.phent, 56, "runtime AT_PHENT");
    assert_eq!(auxv.phnum, header.phnum, "runtime main AT_PHNUM");
    let mut phdr_address = None;
    for index in 0..header.phnum {
        let phdr = read_phdr(context, main, header, index)?;
        let kind = u32::from_le_bytes(phdr[..4].try_into().unwrap());
        let offset = word(&phdr, 8) as u64;
        if kind == 1
            && u32::from_le_bytes(phdr[4..8].try_into().unwrap()) & 4 != 0
            && offset <= header.phoff
            && header.phoff + (header.phnum * 56) as u64 <= offset + word(&phdr, 32) as u64
        {
            phdr_address = Some(word(&phdr, 16) + (header.phoff - offset) as usize);
        }
    }
    let bias = auxv
        .phdr
        .checked_sub(phdr_address.ok_or(TestError("main PHDR has no readable file LOAD"))?)
        .ok_or(TestError("runtime main PHDR bias underflow"))?;

    if header.kind == 2 {
        assert_eq!(bias, 0, "runtime ET_EXEC main bias");
    } else {
        assert!(0 < bias, "runtime ET_DYN main bias");
    }
    assert_eq!(auxv.entry, bias + header.entry, "runtime main AT_ENTRY");
    let mut executable_entry = false;
    for index in 0..header.phnum {
        let phdr = read_phdr(context, main, header, index)?;
        let mut mapped = [0; 56];
        read_mm(context, &image.mm, auxv.phdr + index * 56, &mut mapped)?;
        assert_eq!(mapped, phdr, "runtime mapped main PHDR index={index}");
        executable_entry |= executable_address(&phdr, bias, auxv.entry);
    }
    assert!(
        executable_entry,
        "runtime AT_ENTRY is outside main executable LOAD"
    );
    let main_end = verify_loads(context, &image.mm, main, header, bias)?;
    assert_eq!(
        image.mm.brk(0),
        page_end(main_end),
        "runtime brk is not main end"
    );
    verify_entry(context, &image.mm, main, header, bias, auxv.entry)?;
    if interpreter.is_none() {
        assert_eq!(auxv.base, 0, "direct loader AT_BASE");
        assert_eq!(image.context.pc, auxv.entry, "direct loader initial PC");
        verify_entry(context, &image.mm, main, header, bias, image.context.pc)?;
        context.diagnostic(format_args!(
            "runtime-loader-layout pc={:#x} base=0 bias={:#x} entry={:#x} phdr={:#x} brk={:#x}",
            image.context.pc,
            bias,
            auxv.entry,
            auxv.phdr,
            image.mm.brk(0)
        ));
        return Ok(());
    }

    let interpreter = interpreter.unwrap();
    assert!(0 < auxv.base, "runtime AT_BASE is zero");
    let loader = open(context, owner, fs_context, interpreter)?;
    let loader_header = read_header(context, &loader)?;
    assert_eq!(loader_header.kind, 3, "runtime loader is not ET_DYN");
    assert_eq!(
        image.context.pc,
        auxv.base + loader_header.entry,
        "runtime loader initial PC"
    );
    assert_ne!(
        image.context.pc, auxv.entry,
        "runtime initial PC uses main entry"
    );
    verify_loads(context, &image.mm, &loader, &loader_header, auxv.base)?;
    verify_entry(
        context,
        &image.mm,
        &loader,
        &loader_header,
        auxv.base,
        image.context.pc,
    )?;
    context.diagnostic(format_args!(
        "runtime-layout interpreter={:?} pc={:#x} base={:#x} main-bias={:#x} entry={:#x} phdr={:#x} phnum={} stack-copyin=1",
        interpreter, image.context.pc, auxv.base, bias, auxv.entry, auxv.phdr, auxv.phnum
    ));
    Ok(())
}

/// 入口须位于真实可读可执行 LOAD 的文件前缀，零尾不作为文件指令证据。
fn executable_address(phdr: &[u8; 56], bias: usize, address: usize) -> bool {
    u32::from_le_bytes(phdr[..4].try_into().unwrap()) == 1
        && u32::from_le_bytes(phdr[4..8].try_into().unwrap()) & 5 == 5
        && bias + word(phdr, 16) <= address
        && address < bias + word(phdr, 16) + word(phdr, 32)
}

/// 基本页上界只用于真实 LOAD 末端与主 brk 的几何关系。
fn page_end(address: usize) -> usize {
    address.checked_add(PAGE_SIZE - 1).unwrap() & !(PAGE_SIZE - 1)
}

/// 程序头类型直接解码真实文件字节，测试不消费生产 Elf 的私有字段。
fn phdr_kind(phdr: &[u8; 56]) -> u32 {
    u32::from_le_bytes(phdr[..4].try_into().unwrap())
}

/// 同一页重叠 LOAD 的声明权限取并集，边缘页按真实文件几何计算。
fn page_flags(
    context: &TestContext<'_>,
    file: &FileRef,
    header: &Header,
    bias: usize,
    address: usize,
) -> Result<u32, TestError> {
    let mut flags = 0;
    for index in 0..header.phnum {
        let phdr = read_phdr(context, file, header, index)?;
        if phdr_kind(&phdr) == 1 && word(&phdr, 40) != 0 {
            let start = bias + word(&phdr, 16);
            let end = start + word(&phdr, 40);
            if start & !(PAGE_SIZE - 1) <= address && address < page_end(end) {
                flags |= u32::from_le_bytes(phdr[4..8].try_into().unwrap());
            }
        }
    }
    Ok(flags)
}

/// 正式 fault 与实际页表叶解码共同核许可，拒绝方向不改 VMA；不代替架构硬件 NX 实测。
fn verify_permissions(mm: &Mm, address: usize, flags: u32) {
    for (access, flag) in [
        (UserAccess::Read, 4),
        (UserAccess::Write, 2),
        (UserAccess::Execute, 1),
    ] {
        let expected = if flags & flag != 0 {
            Ok(())
        } else {
            Err(VmError::Permission)
        };
        assert_eq!(
            mm.fault(address, access),
            expected,
            "runtime LOAD permission {address:#x} {access:?}"
        );
    }

    let permissions = mm
        .mapping(address)
        .expect("runtime LOAD leaf absent")
        .permissions;
    assert_eq!(
        (permissions.read, permissions.write, permissions.execute),
        (flags & 4 != 0, flags & 2 != 0, flags & 1 != 0),
        "runtime LOAD hardware leaf permissions {address:#x}",
    );
}

/// 每个真实 LOAD 的边缘和首个内页核权限，完整 BSS 零尾经 copyin 消费，并返回偏移后末端。
fn verify_loads(
    context: &TestContext<'_>,
    mm: &Mm,
    file: &FileRef,
    header: &Header,
    bias: usize,
) -> Result<usize, TestError> {
    let mut max_align = PAGE_SIZE;
    let mut end = 0;
    let mut bss_bytes = 0;
    let mut loads = 0;
    let mut bytes = [0xa7; 512];
    for index in 0..header.phnum {
        let phdr = read_phdr(context, file, header, index)?;
        if phdr_kind(&phdr) != 1 || word(&phdr, 40) == 0 {
            continue;
        }
        loads += 1;
        let align = word(&phdr, 48);
        max_align = max_align.max(align);
        let start = bias.checked_add(word(&phdr, 16)).unwrap();
        let file_end = start.checked_add(word(&phdr, 32)).unwrap();
        let memory_end = start.checked_add(word(&phdr, 40)).unwrap();
        assert!(file_end <= memory_end, "runtime LOAD filesz exceeds memsz");
        if 1 < align {
            assert!(align.is_power_of_two(), "runtime LOAD alignment");
            assert_eq!(
                start % align,
                word(&phdr, 8) % align,
                "runtime biased LOAD alignment"
            );
        }

        let first_page = start & !(PAGE_SIZE - 1);
        let last_page = (memory_end - 1) & !(PAGE_SIZE - 1);
        for address in [
            first_page,
            last_page,
            (first_page + PAGE_SIZE).min(last_page),
        ] {
            verify_permissions(
                mm,
                address,
                page_flags(context, file, header, bias, address)?,
            );
        }

        let mut cursor = file_end;
        while cursor < memory_end {
            let length = bytes.len().min(memory_end - cursor);
            bytes[..length].fill(0xa7);
            read_mm(context, mm, cursor, &mut bytes[..length])?;
            assert!(
                bytes[..length].iter().all(|byte| *byte == 0),
                "runtime LOAD BSS is not zero"
            );
            cursor += length;
            bss_bytes += length;
        }
        end = end.max(memory_end);
    }
    assert!(0 < loads, "runtime has no LOAD");
    assert!(0 < bss_bytes, "runtime real image has no BSS consumer");
    assert_eq!(
        bias % max_align,
        0,
        "runtime bias violates maximum LOAD alignment"
    );
    context.diagnostic(format_args!(
        "runtime-loads kind={} bias={:#x} align={} loads={} bss-bytes={} end={:#x} software-permissions=1 pte-permissions=1",
        header.kind, bias, max_align, loads, bss_bytes, end
    ));
    Ok(end)
}

/// 真实入口的指令前缀同时来自磁盘 LOAD 与候选 Mm，不用地址关系替代实际映射内容。
fn verify_entry(
    context: &TestContext<'_>,
    mm: &Mm,
    file: &FileRef,
    header: &Header,
    bias: usize,
    address: usize,
) -> TestResult {
    for index in 0..header.phnum {
        let phdr = read_phdr(context, file, header, index)?;
        if executable_address(&phdr, bias, address) {
            assert!(address + 4 <= bias + word(&phdr, 16) + word(&phdr, 32));
            let offset = word(&phdr, 8) + address - bias - word(&phdr, 16);
            let mut disk = [0; 4];
            let mut mapped = [0; 4];
            read_file(context, file, offset as u64, &mut disk)?;
            read_mm(context, mm, address, &mut mapped)?;
            assert_eq!(mapped, disk, "runtime initial PC file bytes");
            return Ok(());
        }
    }
    Err(TestError(
        "runtime initial PC is outside executable file LOAD",
    ))
}
