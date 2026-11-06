//! 动态链接 ELF64 程序装载、辅助向量（`auxv`）初始用户栈布局、`PT_INTERP` 解释器映射与写互斥（`ETXTBSY`）生命周期的内核单元测试（ktest）。
//!
//! # 模块职责与架构定位
//! 本模块位于 `os/src/fs/ktest/user/runtime.rs`，作为 `UserOwner` 夹具的方法集合，
//! 专门验证内核 ELF 装载器（`exec::prepare` / `exec::load`）在装载动态链接可执行文件（如 `/entry-dynamic.exe`）与动态链接器（`ld-musl`）时的完整语义。
//! 上游由用户态运行时测试套件调用，下游直接读取磁盘上的 ELF64 文件头/程序头（`Phdr`），并通过 `Mm::copyin`、`Mm::fault` 与 `Mm::mapping` 检验目标虚拟地址空间的内存布局。
//!
//! # 核心设计与动态链接装载内存布局 / 初始用户栈
//!
//! ## 1. 动态链接双映像（主程序 + 解释器）虚拟地址空间与寄存器布局
//! ```text
//! 虚拟地址空间 (Mm)
//! +---------------------------------------------------------------+ 高地址
//! | 初始用户栈 (Initial User Stack, SP 16 字节对齐)               |
//! |   [sp + 0]        : argc                                      |
//! |   [sp + 8 ..]     : argv[0..argc], NULL                       |
//! |   [...]           : envp[0..envc] ("LD_LIBRARY_PATH=/lib"), 0 |
//! |   [...]           : auxv[] 辅助向量键值对:                    |
//! |                       AT_PHDR (3)  -> bias + main.phdr_vaddr  |
//! |                       AT_PHENT(4)  -> 56 (ELF64 Phdr 字节大小)|
//! |                       AT_PHNUM(5)  -> main.e_phnum            |
//! |                       AT_BASE (7)  -> loader_base (无解释器=0)|
//! |                       AT_ENTRY(9)  -> bias + main.e_entry     |
//! |                       AT_EXECFN(31)-> 栈上可执行文件路径字符串|
//! |                       AT_NULL (0)  -> 0                       |
//! |   [...]           : argv / envp / filename 原始 NUL 结尾字符串|
//! +---------------------------------------------------------------+
//! | 动态链接器 (PT_INTERP, ET_DYN) 各 PT_LOAD 段                  |
//! |   基址: AT_BASE (> 0), 初始 PC = AT_BASE + loader.e_entry     |
//! +---------------------------------------------------------------+
//! | 进程堆起点 (brk) = page_end(main 最高 PT_LOAD 结束地址)       |
//! +---------------------------------------------------------------+
//! | 主程序 (ET_EXEC: bias = 0; ET_DYN: bias > 0) 各 PT_LOAD 段    |
//! |   文件数据前缀: [vaddr, vaddr + filesz) 映射磁盘文件内容      |
//! |   BSS 零扩展尾: [vaddr + filesz, vaddr + memsz) 零填充        |
//! +---------------------------------------------------------------+ 低地址
//! ```
//!
//! ## 2. 可执行文件写互斥（`TextBusy` / `ETXTBSY`）引用计数生命周期
//! ```text
//! exec::prepare 装载成功 ──► 主文件与解释器 Inode 均绑定 ExecAccess（acquire_write 返回 TextBusy）
//!   │
//!   ├──► Mm::fork 复制地址空间 ──► 子 Mm 克隆 VMA 中的 ExecAccess 强引用
//!   │                              └──► 释放父 Mm 后，子 Mm 仍存活 ──► 主文件与解释器依旧拒绝写入
//!   │
//!   └──► 用户进程退出且最后一个 Mm 强引用被 Drop ──► ExecAccess 全部释放 ──► acquire_write 恢复成功
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 在内存耗尽（OOM）注入测试中，通过 `spawn_kernel_on` 在独立内核线程中持有 `pressure.exhaust()` 并执行 `exec::load`；
//!   子线程将观测结果写入 `observed`（`SpinLock<Option<...>>`）前必须先释放 `pressure`，父线程通过 `child.join()` 同步回收子线程后再读取结果，避免跨线程死锁或页帧泄漏。

use crate::arch::UserAccess;
use crate::mm::PAGE_SIZE;
use crate::sync::SpinLock;
use core::convert::TryInto;

use crate::fs::ktest::user::*;

/// 动态链接运行时测试的主可执行文件路径。
const ENTRY: &[u8] = b"/entry-dynamic.exe";
/// 动态链接器查找共享库所使用的环境变量字符串。
const LIBRARY_ENV: &[u8] = b"LD_LIBRARY_PATH=/lib";

/// 从磁盘 ELF64 文件头（`Ehdr`）解码出的核心元数据摘要。
struct Header {
    /// ELF 文件类型（`e_type`）：`2` 表示 `ET_EXEC`（固定地址可执行文件），`3` 表示 `ET_DYN`（位置无关可执行文件或共享库）。
    kind: u16,
    /// 程序入口虚拟地址（`e_entry`，未加装载偏移 `bias`）。
    entry: usize,
    /// 程序头表（Program Header Table）在文件中的字节偏移量（`e_phoff`）。
    phoff: u64,
    /// 程序头表中的条目数量（`e_phnum`）。
    phnum: usize,
}

/// 从初始用户栈辅助向量（Auxiliary Vector，`auxv`）中提取的关键条目集合。
struct Auxv {
    /// `AT_PHDR`（类型 3）：主程序程序头表在用户虚拟地址空间中的映射地址。
    phdr: usize,
    /// `AT_PHENT`（类型 4）：单个程序头表条目的字节大小（ELF64 标准为 56 字节）。
    phent: usize,
    /// `AT_PHNUM`（类型 5）：主程序程序头表条目总数。
    phnum: usize,
    /// `AT_BASE`（类型 7）：动态链接器（解释器）的装载基地址；若直接装载无解释器的映像则为 `0`。
    base: usize,
    /// `AT_ENTRY`（类型 9）：主程序在虚拟地址空间中的实际入口地址（`bias + e_entry`）。
    entry: usize,
}

impl UserOwner {
    /// 测试目标：验证在指定测试族目录（`family`）作为独立根命名空间快照时，动态链接程序 `/entry-dynamic.exe` 的完整装载布局、初始栈 `auxv` 及用户态运行退出，且不影响父 `PathOwner` 原有的根目录与当前工作目录。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：初始化夹具并将 `family` 目录解析为新 `FsContext` 的根目录与当前工作目录，打开 `/entry-dynamic.exe` 读取其 ELF64 头与 `PT_INTERP` 解释器路径；
    /// 2. When：构造包含 `LD_LIBRARY_PATH=/lib` 与选择器参数（`"argv"` 或 `"env"`）的 `Arguments`，调用 `prepare_image` 装载双映像，并通过 `verify_image` 逐项校验初始栈、主程序及解释器的 `PT_LOAD` 权限、BSS 清零与入口指令字节，随后启动进程运行；
    /// 3. Then：用户进程以状态码 `0` 正常退出，地址空间活动用户计数归零（`mm.user_count() == 0`），且父 `PathOwner` 的 `root` 与 `cwd` 保持原目录不变。
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
        // Step 1: 为本次运行构造以 `family` 为根的独立文件系统上下文（FsContext），解析主程序头与动态链接器路径。
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
        // Step 2: 装载动态链接程序映像，并核对初始用户栈、auxv、主程序与解释器各 PT_LOAD 段的映射内容和页表权限。
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

        // Step 3: 启动用户进程并等待其退出，验证子任务完全回收且父夹具的 root/cwd 未被篡改。
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

    /// 测试目标：验证直接将动态链接器（`PT_INTERP` 指向的 `ET_DYN` 解释器本身）作为主程序装载时，内核将其作为无解释器的单映像处理（`AT_BASE = 0`），且动态链接器在用户态能够自行加载并执行 `/entry-dynamic.exe`。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：从 `/entry-dynamic.exe` 的 `PT_INTERP` 段读取动态链接器路径，直接打开该动态链接器并验证其为不包含嵌套 `PT_INTERP` 的 `ET_DYN`（`kind == 3`）文件；
    /// 2. When：以 `[interpreter, "/entry-dynamic.exe", "argv"]` 作为命令行参数直接调用 `prepare_image` 装载该动态链接器；
    /// 3. Then：`verify_image` 确认 `AT_BASE == 0`、初始 `PC == AT_ENTRY` 且各段映射正确；随后启动运行，动态链接器在用户态成功加载目标程序并以状态码 `0` 退出。
    pub(super) fn runtime_loader(
        &mut self,
        context: &mut TestContext<'_>,
        family: &[u8],
    ) -> TestResult {
        // Step 1: 从主程序的 PT_INTERP 中提取动态链接器路径，并验证动态链接器自身是无 PT_INTERP 的 ET_DYN 映像。
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

        // Step 2: 将动态链接器作为主映像装载，验证 AT_BASE=0 的直接装载布局并启动运行。
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

    /// 测试目标：验证动态链接双映像（主程序与解释器）的可执行文件写互斥（`FsError::TextBusy`）会随 `Mm::fork` 复制地址空间而延续，直到父 `Mm`、运行进程以及最后一个子 `Mm` 强引用全部释放后才恢复可写。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：装载 `/entry-dynamic.exe` 及其解释器，验证此时对主文件和解释器文件调用 `acquire_write()` 均返回 `Err(FsError::TextBusy)`；
    /// 2. When：调用 `image.mm.fork()` 复制出子地址空间 `child`，并立即 `drop` 原父地址空间 `image.mm`；
    /// 3. Then：尽管父 `Mm` 已销毁，子 `Mm` 仍持有主程序和解释器的执行映射引用，两个文件依旧返回 `TextBusy`；且子 `Mm` 中的栈与代码段映射通过 `verify_image` 完整校验；
    /// 4. When & Then：使用子 `Mm` 启动进程运行至退出并回收任务后，只要测试手中还持有 `child: Arc<Mm>`，两个文件仍保持 `TextBusy`；当 `drop(child)` 释放最后一个 `Mm` 强引用后，主程序与解释器的 `acquire_write()` 立即恢复成功。
    pub(super) fn runtime_fork(
        &mut self,
        context: &mut TestContext<'_>,
        family: &[u8],
    ) -> TestResult {
        // Step 1: 装载主程序与解释器双映像，验证两者均已处于拒绝写入（TextBusy）状态。
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

        // Step 2: 通过 `Mm::fork` 克隆地址空间并立即释放原父 `Mm`，验证子 `Mm` 独立维持双映像内容与 `TextBusy` 写互斥。
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
        // Step 3: 运行子 `Mm` 直至进程退出，验证唯有当最后一个 `Arc<Mm>` 强引用被 Drop 时，主程序与解释器才恢复可写。
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

    /// 测试目标：验证解释器路径错误（不存在或指向目录）以及 `exec::load` 过程中的物理内存耗尽（OOM）不会破坏已装载的旧进程映像现场，也不会泄漏候选文件的执行写互斥锁。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：预先成功装载一份 `/entry-dynamic.exe` 映像 `image`，记录其初始 `PC` 与 `brk`，此时主文件与解释器处于 `TextBusy` 状态；
    /// 2. When（解释器路径异常）：依次尝试装载解释器不存在的 `/elf-missing-interpreter` 与解释器为目录的 `/elf-directory-interpreter`；
    /// 3. Then：分别返回 `ExecError::File(FsError::NotFound)` 与 `ExecError::File(FsError::Permission)`，候选可执行文件本身的写锁未被残留占用（`acquire_write` 成功），且原 `image` 的主文件与解释器仍保持 `TextBusy`；
    /// 4. When（装载期间物理页耗尽）：预热主文件首页缓存后，在独立内核线程中耗尽空闲物理页（`pressure.exhaust()`）并调用 `exec::load`；
    /// 5. Then：`exec::load` 返回 `Err(ExecError::NoMemory)`；原 `image` 的 `PC`、`brk`、内存映像及 `TextBusy` 状态完好无损，随后仍可正常启动运行并以状态码 `0` 退出，最后释放所有写互斥。
    pub(super) fn runtime_failures(
        &mut self,
        context: &mut TestContext<'_>,
        family: &[u8],
    ) -> TestResult {
        // Step 1: 预先装载一份正常的主程序与解释器映像作为受保护的旧现场基准。
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

        // Step 2: 依次测试解释器缺失（NotFound）与解释器为目录（Permission）的装载失败回滚，确认候选文件未泄漏执行写互斥。
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

        // Step 3: 在独立内核线程中注入物理内存耗尽（OOM）并执行 `exec::load`，验证返回 `ExecError::NoMemory`。
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
        // 子线程的唯一 join 句柄保存在 `self.observer` 中，必须在比对文件系统资源基线前完整回收。
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

        // Step 4: 回收 OOM 测试线程并核对结果，验证事先装载的旧映像 `image` 未受任何破坏且能正常执行至退出。
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

    /// 启动已准备好的用户态映像 `image`，等待其退出并回收所有子进程，验证地址空间用户计数归零且退出码为 `0`。
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

/// 将指定测试族路径 `family` 解析为新的独立 `FsContext`（同时作为根目录与当前工作目录），不修改父 `PathOwner` 自身的路径上下文。
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

/// 使用给定的可执行文件路径 `filename`、参数数组 `argv` 以及固定的 `LD_LIBRARY_PATH=/lib` 环境变量构造内核 `Arguments`。
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

/// 以 `ROOT` 凭据调用 `exec::prepare` 装载可执行文件映像，失败时记录诊断信息。
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

/// 断言主程序文件 `main` 与解释器文件 `loader` 当前均因存在活动的执行映射而拒绝写入（返回 `FsError::TextBusy`）。
fn assert_denied(main: &FileRef, loader: &FileRef) {
    for file in [main, loader] {
        assert_eq!(
            file.mapping_inode().unwrap().acquire_write().err(),
            Some(FsError::TextBusy)
        );
    }
}

/// 断言在最后一个 `Mm` 释放后，主程序文件 `main` 与解释器文件 `loader` 均可成功获取并释放写访问权（`WriteAccess`）。
fn assert_writable(context: &TestContext<'_>, main: &FileRef, loader: &FileRef) -> TestResult {
    let main_write = checked(context, main.mapping_inode().unwrap().acquire_write())?;
    let loader_write = checked(context, loader.mapping_inode().unwrap().acquire_write())?;
    drop(loader_write);
    drop(main_write);
    Ok(())
}

/// 在给定的文件系统上下文 `fs_context` 中解析路径 `pathname`（跟随符号链接）并打开文件。
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

/// 从文件 `file` 的指定偏移 `offset` 处读取完整填满 `bytes` 的数据，若发生短读则触发断言失败。
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

/// 通过虚拟地址空间 `mm` 的 `copyin` 接口（按需触发缺页装入）从用户地址 `address` 读取完整填满 `bytes` 的数据。
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

/// 从字节切片 `bytes` 的 `offset` 偏移处按小端序解码一个 64 位机器字并转换为 `usize`。
fn word(bytes: &[u8], offset: usize) -> usize {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap()) as usize
}

/// 通过 `read_mm`（底层 `copyin`）从用户栈地址 `address` 读取一个 8 字节小端序机器字。
fn stack_word(context: &TestContext<'_>, mm: &Mm, address: usize) -> Result<usize, TestError> {
    let mut bytes = [0; 8];
    read_mm(context, mm, address, &mut bytes)?;
    Ok(word(&bytes, 0))
}

/// 从用户栈地址 `address` 读取字节序列，断言其前缀与 `expected` 完全一致且紧随其后为 NUL（`\0`）终止符。
fn stack_string(context: &TestContext<'_>, mm: &Mm, address: usize, expected: &[u8]) -> TestResult {
    let mut bytes = [0; 64];
    assert!(expected.len() < bytes.len());
    read_mm(context, mm, address, &mut bytes[..expected.len() + 1])?;
    assert_eq!(&bytes[..expected.len()], expected, "runtime stack string");
    assert_eq!(bytes[expected.len()], 0, "runtime stack string terminator");
    Ok(())
}

/// 从架构栈指针寄存器（RISC-V `x2` / LoongArch `r3`）出发，在用户栈上依次校验 `argc`、`argv`、`envp` 并解析返回 `Auxv` 辅助向量。
///
/// # 校验规则
/// 1. 栈指针 `sp` 必须满足 16 字节对齐；
/// 2. 栈顶依次存放 `argc`、`argv` 字符串指针数组（以 `0` 结尾）、`envp` 字符串指针数组（包含 `LD_LIBRARY_PATH=/lib` 并以 `0` 结尾）；
/// 3. 随后扫描 `auxv` 键值对直到遇到 `AT_NULL (0, 0)`，确保必需的 6 项辅助向量（`AT_PHDR`、`AT_PHENT`、`AT_PHNUM`、`AT_BASE`、`AT_ENTRY`、`AT_EXECFN`）全部存在且无重复项。
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
    // Step 1: 校验初始栈指针 16 字节对齐，并依次核对栈上的 argc、argv 字符串指针表与 envp 环境变量指针表。
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

    // Step 2: 遍历栈上的辅助向量（auxv）表直至 AT_NULL，校验必需条目完整性与 AT_EXECFN 文件名字符串。
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

/// 读取并校验文件头部的 64 字节 ELF64 文件头（`Ehdr`），提取文件类型、入口地址与程序头表位置。
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

/// 根据已解析的 ELF64 文件头 `header`，从文件中读取第 `index` 个 56 字节的程序头（`Phdr`）。
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

/// 遍历文件的程序头表查找 `PT_INTERP`（类型 `3`）段，读取并校验以 `/` 开头、以单个 NUL 结尾的动态链接器绝对路径。
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

/// 全面校验已装载映像 `image` 的初始栈 `auxv`、主程序与动态链接器的装载偏移（`bias`/`AT_BASE`）、各 `PT_LOAD` 段的页表权限与 BSS 清零、进程 `brk` 起点以及入口指令字节。
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
    // Step 1: 解析初始用户栈上的 auxv，并根据主程序包含程序头表的可读 PT_LOAD 段推导主程序实际装载偏移（bias）。
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
    // Step 2: 校验主程序映射在用户空间的程序头表、各 PT_LOAD 段权限与 BSS 零填充、初始 brk 位置以及 AT_ENTRY 入口指令字节。
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
    // Step 3: 若为无解释器的直接装载模式（如直接运行动态链接器），验证 AT_BASE == 0 且初始 PC 指向主入口后直接返回。
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

    // Step 4: 若存在 PT_INTERP 动态链接器，打开解释器文件并校验 AT_BASE > 0、初始 PC 指向解释器入口以及解释器自身的 PT_LOAD 段映射。
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

/// 判断虚拟地址 `address` 是否落在给定程序头 `phdr`（加上偏移 `bias`）所描述的可读可执行（`PF_R | PF_X == 5`）`PT_LOAD` 段的文件数据前缀范围内。
fn executable_address(phdr: &[u8; 56], bias: usize, address: usize) -> bool {
    u32::from_le_bytes(phdr[..4].try_into().unwrap()) == 1
        && u32::from_le_bytes(phdr[4..8].try_into().unwrap()) & 5 == 5
        && bias + word(phdr, 16) <= address
        && address < bias + word(phdr, 16) + word(phdr, 32)
}

/// 将地址 `address` 向上对齐到页边界（`PAGE_SIZE` 的整数倍）。
fn page_end(address: usize) -> usize {
    address.checked_add(PAGE_SIZE - 1).unwrap() & !(PAGE_SIZE - 1)
}

/// 解码 56 字节 ELF64 程序头 `phdr` 的前 4 字节段类型（`p_type`）。
fn phdr_kind(phdr: &[u8; 56]) -> u32 {
    u32::from_le_bytes(phdr[..4].try_into().unwrap())
}

/// 计算虚拟地址 `address` 所在物理页在所有覆盖该页的非空 `PT_LOAD` 段中的合并权限标志位（`p_flags` 按位或）。
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

/// 通过触发虚拟地址空间 `mm` 的读/写/执行缺页（`Mm::fault`）并查询硬件页表叶子项（`Mm::mapping`），联合校验虚拟地址 `address` 的页权限是否与 ELF 段标志 `flags` 一致。
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

/// 遍历文件的全部非空 `PT_LOAD` 段，校验段对齐约束、首尾及内部页的软件/页表权限、以及 `[filesz, memsz)` 区间内 BSS 段的零填充，返回所有加载段经 `bias` 偏移后的最高结束虚拟地址。
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
    // Step 1: 遍历每个非空 PT_LOAD 段，检查 filesz <= memsz 与段对齐几何关系。
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

        // Step 2: 对当前 PT_LOAD 段的首个页、末尾页及第二个页验证读/写/执行缺页行为与页表 PTE 权限位。
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

        // Step 3: 通过 `read_mm`（copyin）完整读取该段的 `[file_end, memory_end)` BSS 扩展区间，断言所有字节均已清零。
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

/// 校验可执行入口虚拟地址 `address` 处的首条 4 字节指令与磁盘 ELF 文件对应 `PT_LOAD` 段偏移处的原始字节完全一致。
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
