// ============================================================================
// 通用用户态进程、文件系统与信号 ABI 集成测试套件（通过 `include!` 内联进 `user.rs`）
//
// 模块职责与架构定位：
// 本文件定义 `programs` 测试套件模块，上游由 `fs::ktest::user::anchor()` 注册到内核测试框架，
// 下游驱动 `UserOwner` 在共享块设备库存（`UserSuite`）的基础上，为每个测试用例构建独立的文件系统视图与进程关系图。
//
// 核心设计（测试覆盖矩阵与生命周期）：
// ```text
// UserSuite (套件级共享块设备库存，不启动网络或串口中断接收)
//   │
//   └── UserOwner (每个测试用例独立创建的运行夹具)
//         ├── 1. 动态链接与 ELF 装载：musl/glibc 动态链接器、PT_INTERP、ET_DYN、低地址入口、稀疏 BSS、重叠段、准备回滚
//         ├── 2. 管道与多路复用：pipe/pipe2、writev 原子记录、SIGPIPE、poll/select/pselect6/ppoll、epoll (LT/ET/ONESHOT)
//         ├── 3. 运行时环境与系统视图：uname/chroot、utimensat、硬件时钟与睡眠、RLIMIT_STACK、TLS 跨核迁移、#! 脚本、PID 1 孤儿收养
//         ├── 4. 磁盘与设备语义：只读挂载点设备 mmap 拒绝优先级、块/字符设备节点 unlink/rename 后释放磁盘 Inode 位、冷重开写回同步、非 root 目录搜索权限
//         ├── 5. 进程克隆与信号投递：clone 标志轴、保留 fd 隔离、标准/实时信号排队、SA_NODEFER/SA_RESETHAND、ucontext 寄存器/FPU/向量帧修改、COW 信号栈、时钟抢占
//         └── 6. 同步原语与生命周期：futex 等待/唤醒/重排/clear_child_tid、非主线程 execve 接管、vfork、进程组停止/继续、僵尸收割与在途 DMA 读取取消
// ```
// ============================================================================
#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user-elf", "user-abi", "user-signal", "fs-path", "fs-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod programs {
    use super::*;

    /// 测试目标：验证 musl 动态链接器（`ld-musl`）在独立根目录下装载动态链接聚合测试程序，并校验用户栈上的 `argv`、`envp` 及主程序与解释器身份。
    #[test]
    #[case("argv", b"argv" as &[u8])]
    #[case("env", b"env" as &[u8])]
    #[ktest(tags("runtime-dynamic", "runtime-musl"))]
    fn runtime_musl(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        selector: &[u8],
    ) -> TestResult {
        fixture.runtime_dynamic(context, b"/musl", selector)
    }

    /// 测试目标：验证 glibc PIE 动态链接程序在独立根目录下的装载与执行，校验 `argv`/`envp` 传递以及进程退出与内核栈回收。
    #[test]
    #[case("argv", b"argv" as &[u8])]
    #[case("env", b"env" as &[u8])]
    #[ktest(tags("runtime-dynamic", "runtime-glibc"))]
    fn runtime_glibc(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        selector: &[u8],
    ) -> TestResult {
        fixture.runtime_dynamic(context, b"/glibc", selector)
    }

    /// 测试目标：验证将动态链接器（Loader）自身作为不含 `PT_INTERP` 段的 `ET_DYN` 可执行文件直接启动，并由其加载并运行目标动态程序。
    #[test]
    #[case("musl", b"/musl" as &[u8])]
    #[case("glibc", b"/glibc" as &[u8])]
    #[ktest(tags("runtime-loader", "Q05", "B2", "B3"))]
    fn runtime_loader_without_interpreter(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        family: &[u8],
    ) -> TestResult {
        fixture.runtime_loader(context, family)
    }

    /// 测试目标：验证包含主程序与解释器两个可执行映像的进程在 `fork`（非共享 VM）后，父子进程均持有对两个底层 Inode 的执行写拒绝引用（`ETXTBSY`），直到最后一个进程退出后才允许以写模式打开。
    #[test]
    #[case("musl", b"/musl" as &[u8])]
    #[case("glibc", b"/glibc" as &[u8])]
    #[ktest(tags("runtime-fork-deny", "Q06", "B5"))]
    fn runtime_two_images_fork_last_owner(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        family: &[u8],
    ) -> TestResult {
        fixture.runtime_fork(context, family)
    }

    /// 测试目标：验证在 `execve` 加载解释器阶段遭遇普通错误或堆内存耗尽（OOM）时，仅销毁新创建的候选映像，原进程的旧主程序与解释器映像仍保持完好并能正常执行退出。
    #[test]
    #[case("musl", b"/musl" as &[u8])]
    #[case("glibc", b"/glibc" as &[u8])]
    #[ktest(tags("runtime-preparation", "Q06", "B5"), min_cpus = 2)]
    fn runtime_interpreter_failures_preserve_old_image(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        family: &[u8],
    ) -> TestResult {
        fixture.runtime_failures(context, family)
    }

    /// 测试目标：验证 `/dev/kmsg` 只读内核日志设备在多次独立 `open`、共享 `dup` 描述符、环形缓冲区覆盖恢复以及用户缓冲区缺页错误（`EFAULT`）下的读取游标语义。
    #[test]
    #[ktest(tags("observation-user", "kmsg"))]
    fn observation_kmsg(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.observation_mode(context, b"kmsg")
    }

    /// 测试目标：验证 ELF 装载器对段页边界、BSS 段清零、命令行参数栈布局以及页面缓存写时复制（COW）私有性的综合处理。
    #[test]
    #[ktest(min_cpus = 2)]
    fn load_boundaries_stack_and_disk_program(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        layout::run(context, fixture)
    }

    /// 测试目标：验证固定地址 ELF 映像从零页（`0x0`）或低地址（`0x100`）入口启动，且跨页的程序头表（Program Header）与未使用的扩展元数据不会阻碍装载执行。
    #[test]
    #[case("page_zero", 0)]
    #[case("low_vendor_phdr", 0x100)]
    #[ktest(tags("elf-direct-load"), min_cpus = 2)]
    fn direct_elf_low_entry(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        entry: usize,
    ) -> TestResult {
        layout::direct_low(context, fixture, entry)
    }

    /// 测试目标：验证包含超大稀疏 BSS 段与远端 `PT_LOAD` 段的 ELF 映像采用按需缺页分配（Demand Paging），仅实际触碰的虚拟页分配物理内存，并在退出时完整回收。
    #[test]
    #[ktest(tags("elf-direct-load"), min_cpus = 2)]
    fn direct_elf_sparse_bss(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        layout::direct_sparse(context, fixture)
    }

    /// 测试目标：验证合法重叠的 `PT_LOAD` 段在装载时按顺序覆盖写入文件内容并正确清零各自对应的 BSS 尾部字节。
    #[test]
    #[ktest(tags("elf-direct-load"), min_cpus = 2)]
    fn direct_elf_overlapping_segments(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        layout::direct_overlap(context, fixture)
    }

    /// 测试目标：验证 `execve` 与进程准备阶段遭遇 OOM、非法参数地址及未发布放弃（Abandon）回滚后，原先已加载的正常 ELF 程序仍能正常启动并完成退出。
    #[test]
    fn failed_preparation_keeps_original_program(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        preparation::run(context, fixture)
    }

    /// 测试目标：验证常规文件的读写偏移、目录项 `d_off` 游标（Cookie）、只读边界、文件内存映射（`mmap`）及 `brk` 堆扩展的用户态 ABI 行为。
    #[test]
    fn file_delivery_and_mapping_abi(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-io", b"raw", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证父子两个用户进程通过 `fork` 与 `dup` 共享同一个管道打开文件描述（OFD）时，读者在读完最后一个写者写入的全部尾部数据并关闭所有写端后才观察到 EOF。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_fork_dup_full_stream_tail_and_eof(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"d")
    }

    /// 测试目标：验证两个并发写进程各自通过 `writev` 提交总长不超过 `PIPE_BUF`（4096 字节）的多段向量写入时，每次 `writev` 作为原子记录写入管道，读者读出的数据块互不交错。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_small_writev_records_across_writers(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"a")
    }

    /// 测试目标：验证非阻塞管道（`O_NONBLOCK`）在剩余容量不足时返回 `EAGAIN`，以及零长度读写与跨页内存拷贝故障（`EFAULT`）时保留已成功传输的前缀字节数。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_nonblocking_capacity_zero_and_fault_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"n")
    }

    /// 测试目标：验证 `readv`/`writev` 导入 `iovec` 数组后作为单次文件操作执行，正确处理长度溢出边界并按实际传输字节数推进共享文件偏移量。
    #[test]
    #[ktest(tags("fs-pipe", "fs-vector-user"))]
    fn file_vector_import_and_shared_position(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"v")
    }

    /// 测试目标：验证向已无读者的管道写入非零字节时，写者线程收到 `SIGPIPE` 信号并按忽略（返回 `EPIPE`）、默认终止或信号掩码阻塞三种配置分别表现正确语义。
    #[test]
    #[case("ignored", b"i" as &[u8])]
    #[case("default_child", b"s" as &[u8])]
    #[case("blocked_writer", b"b" as &[u8])]
    #[ktest(tags("fs-pipe", "fs-pipe-user", "fs-pipe-signal"))]
    fn pipe_last_reader_sigpipe_and_result(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        mode: &[u8],
    ) -> TestResult {
        fixture.pipe_abi(context, mode)
    }

    /// 测试目标：验证已进入睡眠阻塞态的管道 `read` 被普通信号中断时，根据信号处理函数是否设置 `SA_RESTART` 分别返回 `EINTR` 或在信号返回后自动重启读取。
    #[test]
    #[case("interrupted", b"e" as &[u8])]
    #[case("restart", b"r" as &[u8])]
    #[ktest(tags("fs-pipe", "fs-pipe-user", "fs-pipe-signal"))]
    fn pipe_blocked_read_signal_and_restart(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        mode: &[u8],
    ) -> TestResult {
        fixture.pipe_abi(context, mode)
    }

    /// 测试目标：验证 `pipe2` 在写回用户态数组发生缺页故障（`EFAULT`）或第二个文件描述符槽位耗尽（`EMFILE`）时，完整回滚本次预留的描述符与管道对象，不污染原有描述符表。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe2_output_fault_and_second_slot_exhaustion(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"f")
    }

    /// 测试目标：验证非 root 凭据进程创建的匿名管道端点记录创建者自身的 `uid`/`gid`，并可通过 `fstat` 准确读取。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_creator_credentials_in_anonymous_stat(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"u")
    }

    /// 测试目标：验证对管道文件描述符调用带偏移量的 `pread64`/`pwrite64` 时，优先根据不支持随机寻址的 OFD 能力返回 `ESPIPE`，先于文件读写方向与用户缓冲区地址检查。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user", "fs-pipe-positioned"))]
    fn pipe_positioned_io_error_precedence(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"p")
    }

    /// 测试目标：验证 `poll` 数组、`select` 位图跨度、同一文件描述符重复登记计数以及非法参数错误码的检查优先级。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T10", "T12"))]
    fn poll_select_raw_abi_and_errors(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"a")
    }

    /// 测试目标：验证零文件描述符集合下的定时等待（`poll`/`select`/`pselect6`）准确消耗指定的超时时间，且正常超时返回结果不会被写回剩余时间结构体时的 `EFAULT` 错误覆盖。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T09"))]
    fn zero_fd_deadlines(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.events_abi(context, b"t")
    }

    /// 测试目标：验证 `poll`、`select` 与 `epoll_wait` 三种多路复用等待均在真正进入睡眠阻塞态后，由子进程向管道写入数据触发唤醒，并通过正常事件路径返回就绪。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T11"))]
    fn persistent_event_blocked_notifications(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"b")
    }

    /// 测试目标：验证 `pselect6`/`ppoll`/`epoll_pwait` 原子替换临时信号掩码后，信号处理函数执行期间观察到临时掩码，处理函数返回后恢复原掩码且等待调用本身不受 `SA_RESTART` 自动重启。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T09"))]
    fn temporary_mask_handler_restore(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"s")
    }

    /// 测试目标：验证 `epoll` 事件投递完整保留 64 位 `epoll_data_t` 高低位数据，并在同一管道上验证 `EPOLL_CTL_ADD`/`EPOLL_CTL_MOD` 以及水平触发（LT）、边缘触发（ET）和单次触发（`EPOLLONESHOT`）模式。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T12", "T13", "T15"))]
    fn epoll_ready_modes(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.events_abi(context, b"e")
    }

    /// 测试目标：验证通过 `dup` 复制的文件描述符与关闭后复用的同号新文件描述符在 `epoll` 中保持各自独立的底层打开文件（OFD）身份，且仅当底层 OFD 的最后一个引用关闭时才自动从 `epoll` 注销。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T14", "T18"))]
    fn epoll_dup_fd_reuse_final_close(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"r")
    }

    /// 测试目标：验证 `epoll_wait` 向用户空间写回事件数组时，若首项即发生 `EFAULT` 或在跨页写入部分事件后发生 `EFAULT`，所有尚未成功交付给用户态的就绪事件均会保留在就绪队列中供下次 `epoll_wait` 继续读取。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T17"))]
    fn epoll_output_fault_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"f")
    }

    /// 测试目标：验证用户态 C 程序通过 `uname` 与 `chroot` 系统调用检验无效缓冲区地址故障、切换根目录后旧 `cwd`/已打开目录描述符的相对解析语义，以及父子进程各自独立的根目录视图。
    #[test]
    #[case("root", Credentials::ROOT)]
    #[case("nonroot", Credentials { uid: 1000, gid: 2000 })]
    #[ktest(tags("runtime-view", "user-libc", "fs-path"), timeout_ms = 120000)]
    fn runtime_view(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        credentials: Credentials,
    ) -> TestResult {
        fixture.runtime_view(context, credentials)
    }

    /// 测试目标：验证用户态 C 程序在可写挂载卷上通过 `utimensat` 修改文件时间戳、特殊纳秒值（`UTIME_NOW`/`UTIME_OMIT`）及非特权用户的权限拒绝检查。
    #[test]
    #[case("root", Credentials::ROOT)]
    #[case("nonroot", Credentials { uid: 1000, gid: 2000 })]
    #[ktest(
        tags("runtime-file-times", "user-libc", "fs-path"),
        timeout_ms = 120000
    )]
    fn runtime_file_times(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        credentials: Credentials,
    ) -> TestResult {
        fixture.runtime_file_times(context, credentials)
    }

    /// 测试目标：验证用户态 C 程序读取硬件实时时钟（`CLOCK_REALTIME`）转换日期字段、执行基于 `CLOCK_MONOTONIC` 的绝对时间睡眠，并校验单调时钟与进程 CPU 时间账本各自的推进语义。
    #[test]
    #[ktest(tags("runtime-clock", "user-libc", "fs-path"), timeout_ms = 120000)]
    fn runtime_clock(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.runtime_clock(context)
    }

    /// 测试目标：验证具有 root 凭据的用户态 C 程序查询与修改 `RLIMIT_STACK` 资源限额，并验证该限额对 `fork` 与 `execve` 后新主栈自动增长范围的约束。
    #[test]
    #[ktest(tags("runtime-limits"), min_cpus = 1, timeout_ms = 120000)]
    fn runtime_stack_limits_root(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.runtime_limits(context, Credentials::ROOT)
    }

    /// 测试目标：验证非 root 凭据的用户态 C 程序修改 `RLIMIT_STACK` 硬限额时的权限拒绝（`EPERM`）以及软限额下调后对 `execve` 栈空间的约束。
    #[test]
    #[ktest(tags("runtime-limits"), min_cpus = 1, timeout_ms = 120000)]
    fn runtime_stack_limits_nonroot(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.runtime_limits(
            context,
            Credentials {
                uid: 1000,
                gid: 2000,
            },
        )
    }

    /// 测试目标：验证静态链接的 C 标准库（libc）程序从磁盘启动，完成线程局部存储（TLS）初始化、主栈访问、标准输入输出流读写及基础文件操作。
    #[test]
    #[ktest(tags("T31", "user-libc"), timeout_ms = 120000)]
    fn libc_static_smoke(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.libc_static_smoke(context)
    }

    /// 测试目标：验证同一个用户态线程在两次管道唤醒之间发生跨 CPU 迁移（Migration）时，其线程局部存储（TLS）指针与浮点/向量寄存器上下文保持完好无损。
    #[test]
    #[ktest(min_cpus = 2, tags("T31", "user-libc"), timeout_ms = 120000)]
    fn libc_migration(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.libc_migration(context)
    }

    /// 测试目标：验证以 `#!`（Shebang）开头的脚本文件通过 `execve` 启动时，内核正确解析解释器路径、可选参数并向解释器传递原始脚本路径与环境变量。
    #[test]
    #[ktest(tags("T32", "user-script"), timeout_ms = 120000)]
    fn script_exec(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.script_exec(context)
    }

    /// 测试目标：验证作为命名空间初始进程（PID 1）的系统根进程通过 `fork`、`pipe` 与 `wait` 收养并回收两类孤儿进程（直接子进程派生的孙进程与多线程非主线程派生的孤儿）后，正常发起关机（`PowerOff`）请求。
    #[test]
    #[ktest(tags("T33", "fs-system-user", "fs-pid1"), timeout_ms = 120000)]
    fn system_pid1_adopts_orphans(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.system_pid1_adopts_orphans(context)
    }

    /// 测试目标：验证普通测试夹具下的根进程（非系统级 PID 1）仍可被未捕获的 `SIGKILL` 正常终止，并分别在 root 与非 root 凭据下检验 `reboot` 系统调用的权限检查与非法魔数/命令字错误。
    #[test]
    #[case("root", false)]
    #[case("nonroot", true)]
    #[ktest(tags("T33", "fs-system-user", "fs-pid1"), timeout_ms = 120000)]
    fn fixture_root_signal_semantics(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        nonroot: bool,
    ) -> TestResult {
        fixture.fixture_root_signal_semantics(context, nonroot)
    }

    /// 测试目标：验证当用户进程对只读挂载（`ReadOnly`）的 `devfs` 中以读写模式打开的字符设备请求 `MAP_SHARED | PROT_WRITE` 内存映射时，优先因为设备本身不支持共享写映射能力而返回 `ENODEV`/`EACCES`，先于挂载点只读写保护检查。
    #[test]
    #[ktest(tags("T04", "T30", "fs-device-mapping"))]
    fn device_mapping_capability_precedes_mount_write(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let vfs = fixture.path.vfs();
        let covered = checked(
            context,
            vfs.resolve(fixture.path.context(), b"/plain", ResolveMode::Follow),
        )?;
        let device = checked(context, crate::fs::devfs::try_new(0o755))?;
        checked(
            context,
            vfs.mount(
                &covered,
                crate::fs::Filesystem::Memory(device),
                AccessMode::ReadOnly,
            ),
        )?;

        let event = fixture.run_named(context, b"/test/bin/user-device-map", b"raw", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证磁盘上的字符设备与块设备节点在通过 `unlink` 删除或被 `rename` 覆盖使硬链接数归零后，当最后一个内存 Inode 引用释放时，正确归还磁盘上的 Inode 位图分配位，且不会将设备号（`rdev`）误当作数据块树去释放。
    #[test]
    #[case("unlink_character", false)]
    #[case("replace_block", true)]
    #[ktest(tags("T01", "fs-device-retirement"))]
    fn disk_device_last_reference_releases_inode(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
        replace: bool,
    ) -> TestResult {
        fixture.prepare_mode(context, AccessMode::ReadWrite, 4)?;
        let vfs = fixture.path.vfs();
        let fs_context = fixture.path.context();
        let pathname = if replace {
            b"/device-block".as_slice()
        } else {
            b"/device-char".as_slice()
        };
        let path = checked(
            context,
            vfs.resolve(fs_context, pathname, ResolveMode::Follow),
        )?;
        let inode = checked(context, path.inode())?.clone();
        let number = inode.number();
        let volume = checked(context, inode.disk_volume())?.clone();
        let metadata = checked(context, inode.metadata_snapshot())?;
        context.expect_eq(
            metadata.kind,
            if replace {
                InodeKind::Block
            } else {
                InodeKind::Character
            },
        );
        context.expect_eq(metadata.rdev, if replace { 0x801 } else { 0x103 });
        context.expect_eq(metadata.blocks, 0);
        context.expect_eq(
            checked(context, write::inode_allocation_bit(&volume, number))?,
            true,
        );

        // Step 1: 根据测试用例分支，通过 rename 覆盖或直接 unlink 将目标设备节点的目录硬链接数降为 0。
        if replace {
            drop(checked(
                context,
                vfs.open_with(
                    fs_context,
                    fs_context.cwd(),
                    b"/test/device-replacement",
                    (0, 0),
                    Some((0o644, 0)),
                    true,
                    false,
                    |path, _| Ok(path),
                ),
            )?);
            checked(
                context,
                vfs.rename_from(
                    fs_context,
                    fs_context.cwd(),
                    b"/test/device-replacement",
                    fs_context.cwd(),
                    pathname,
                    false,
                    (0, 0),
                ),
            )?;
            let replacement = checked(
                context,
                vfs.resolve(fs_context, pathname, ResolveMode::Follow),
            )?;
            context.expect_eq(
                checked(
                    context,
                    checked(context, replacement.inode())?.metadata_snapshot(),
                )?
                .kind,
                InodeKind::Regular,
            );
        } else {
            checked(
                context,
                vfs.unlink_from(fs_context, fs_context.cwd(), pathname, (0, 0)),
            )?;
            context.expect_eq(
                vfs.resolve(fs_context, pathname, ResolveMode::Follow).err(),
                Some(FsError::NotFound),
            );
        }
        context.expect_eq(checked(context, inode.metadata_snapshot())?.links, 0);

        // Step 2: 释放内存中持有的 Inode 与 Path 引用，触发 VFS 清理并等待后台写回/回收工作队列完成，验证磁盘 Inode 位图已归零。
        let watch = fixture.path.owner.cache().watch();
        drop(inode);
        drop(path);
        vfs.prune_unused();
        checked(context, watch.wait())?;
        context.expect_eq(watch.contains(number), false);
        context.expect_eq(
            checked(context, write::inode_allocation_bit(&volume, number))?,
            false,
        );
        Ok(())
    }

    /// 测试目标：验证对磁盘上的 ELF 文件执行常规写入并同步落盘后，丢弃缓存冷重开（Cold Reopen）该文件仍能正常装载执行且内容完整。
    #[test]
    #[ktest(min_cpus = 2, tags("W35", "fs-writeback"))]
    fn production_writeback_sync_and_cold_reopen(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.production_writeback_cold_reopen(context)
    }

    /// 测试目标：验证非文件属主（非 root）的用户进程在读取目录、按路径逐层搜索（Search）时的自主访问控制（DAC）权限拦截，以及访问被拒绝后原有目录读取游标与最低空闲文件描述符槽位保持不变。
    #[test]
    fn nonroot_directory_search_and_access(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let denied = checked(context, fixture.file(context, b"/test/io-denied")?.stat())?;
        let no_search = checked(
            context,
            fixture.file(context, b"/test/io-no-search")?.stat(),
        )?;
        context.expect_eq(denied.kind, InodeKind::Directory);
        context.expect_eq(denied.mode & 0o005, 0);
        context.expect_eq(no_search.kind, InodeKind::Directory);
        context.expect_eq(no_search.mode & 0o005, 0o004);

        let credentials = Credentials {
            uid: [1, 2, 3]
                .iter()
                .copied()
                .find(|&uid| uid != denied.uid && uid != no_search.uid)
                .unwrap(),
            gid: [1, 2, 3]
                .iter()
                .copied()
                .find(|&gid| gid != denied.gid && gid != no_search.gid)
                .unwrap(),
        };
        context.diagnostic(format_args!(
            "user-access-credentials uid={} gid={} denied_owner={}:{} no_search_owner={}:{}",
            credentials.uid, credentials.gid, denied.uid, denied.gid, no_search.uid, no_search.gid
        ));

        let event = fixture.run_named_as(
            context,
            b"/test/bin/user-io",
            b"access",
            false,
            0,
            credentials,
        )?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证 `clone` 系统调用在四种典型资源共享组合（传统私有 `fork`、共享地址空间 `CLONE_VM`、独立进程但共享信号处理器 `CLONE_SIGHAND`、以及完整线程组 `CLONE_THREAD`）下的 TLS 设置与资源隔离语义。
    #[test]
    #[ktest(min_cpus = 2)]
    fn clone_tls_and_resource_axes(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        const MODES: &[&[u8]] = &[b"f0", b"v1", b"s2", b"T3"];
        for &mode in MODES {
            let event = fixture.run_named(context, b"/test/bin/user-clone", mode, false, 0)?;
            context.expect_eq(event.status, 0);
        }
        Ok(())
    }

    /// 测试目标：验证当父进程有另一个线程正处于文件描述符槽位 `fd 0` 的并发预留阶段（尚未提交打开文件）时执行 `fork`，子进程不会继承该未提交的预留槽位，因此子进程随后调用 `open` 能够直接分配到 `fd 0`。
    #[test]
    #[ktest(min_cpus = 2)]
    fn fork_does_not_inherit_reserved_descriptor(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-clone", b"r0", true, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证标准信号（合并去重）、线程私有挂起队列与进程组共享挂起队列的优先级、实时信号（RT Signal）按 FIFO 顺序排队以及实时信号队列容量上限耗尽时的处理语义。
    #[test]
    fn signal_pending_and_realtime_queue(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"p", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证仅共享 `CLONE_VM | CLONE_SIGHAND` 但未设置 `CLONE_THREAD` 的两个独立进程各自维护独立的进程组级挂起信号队列（Group Pending），且 `clone` 创建的子进程不会继承父进程已挂起的未决信号。
    #[test]
    fn signal_shared_vm_distinct_group_pending(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"v", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证信号处理动作中的 `SA_NODEFER`（不自动掩蔽当前信号）、自定义 `sa_mask`、`SA_RESETHAND`（进入处理器前重置为默认动作）以及默认忽略信号的实际触发次数。
    #[test]
    fn signal_action_mask_and_selected_consumption(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"a", false, 0)?;
        context.expect_eq(event.status, 0);

        let reset = fixture.run_named(context, b"/test/bin/user-signal", b"x", false, 10)?;
        context.expect_eq(reset.status, 10);
        Ok(())
    }

    /// 测试目标：验证通过 `clone` 创建且指定退出通知信号为 `0` 的子进程在退出时不会误向父进程发送 `SIGCHLD`，而指定了 `SIGCHLD` 的普通子进程退出时仍正常投递信号。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_clone_exit_notification_semantics(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.signal_clone_exit_notification(context)
    }

    /// 测试目标：验证进程在无自定义处理函数的停止/继续（Stop/Continue）信号往返后，若系统调用因参数 `EFAULT` 失败返回，仍能正确恢复调用前的原始信号掩码，防止后续信号处理函数误消费残留的 `saved_mask`。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_sigsuspend_restart_restores_mask(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.signal_suspend_restart_mask(context)
    }

    /// 测试目标：验证信号处理帧（Signal Frame）对全部浮点寄存器、向量寄存器、浮点控制状态寄存器（FCSR/FCC）的保存与恢复，以及在信号处理函数中直接修改 `ucontext` 里的 `PC`/`SP` 后经 `rt_sigreturn` 跳转到新位置继续执行（在 LoongArch64 上同时验证跨用户态时钟中断的向量上下文完整性）。
    #[test]
    fn signal_frame_registers_and_edited_context(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"f", false, 0)?;
        context.expect_eq(event.status, 0);
        #[cfg(target_arch = "loongarch64")]
        context.expect(
            0 < event.user_timer_interrupts,
            file!(),
            line!(),
            format_args!("full vector state did not cross a user timer interrupt"),
        );
        Ok(())
    }

    /// 测试目标：验证匿名缺页、写只读页权限异常、访问超出文件末尾整页映射触发的 `SIGBUS` 以及非法指令触发的 `SIGILL` 均能向信号处理函数提供准确的 `siginfo_t` 与故障指令 `PC`，并在处理函数修正上下文后恢复执行。
    #[test]
    fn signal_synchronous_fault_info_and_resume(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"z", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证子进程在备用信号栈（`sigaltstack`）上嵌套触发信号并写入信号帧时，严格遵守写时复制（COW）隔离，父进程的私有物理页内容不被子进程的内核信号帧写入污染。
    #[test]
    fn signal_nested_altstack_and_frame_cow(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"n", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证用户程序伪造非法的 `rt_sigreturn` 栈帧、将栈指针指向不可写地址后触发信号、或在信号帧中篡改特权状态位时，内核均会安全地以 `SIGSEGV`（信号编号 11）终止该用户进程而不会引发内核 panic。
    #[test]
    fn signal_invalid_frames_terminate(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let returned = fixture.run_named(context, b"/test/bin/user-signal", b"b", false, 11)?;
        context.expect_eq(returned.status, 11);

        let written = fixture.run_named(context, b"/test/bin/user-signal", b"w", false, 11)?;
        context.expect_eq(written.status, 11);

        let privileged = fixture.run_named(context, b"/test/bin/user-signal", b"k", false, 11)?;
        context.expect_eq(privileged.status, 11);
        Ok(())
    }

    /// 测试目标：验证实时定时器信号中断相对睡眠（`nanosleep`）与绝对时钟睡眠（`clock_nanosleep`），以及中断 `waitid` 时根据 `SA_RESTART` 标志分别自动重启或返回 `EINTR`。
    #[test]
    fn signal_sleep_and_wait_restart_policy(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"s", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证相对睡眠在被作业控制停止/继续（Stop/Continue）信号打断并通过 `restart_syscall` 恢复等待时，沿用初次进入睡眠时换算出的原绝对到期时间，而不会将重启当作一次全新的相对时长重新计时。
    #[test]
    fn signal_sleep_restart_keeps_deadline(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"d", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证包含两个线程的子进程组在收到停止/继续信号时，父进程通过 `waitid`（`WSTOPPED | WCONTINUED`）准确观察到组停止与组继续事件，且不会与另一个仅共享 VM 的独立进程组混淆。
    #[test]
    fn signal_group_stop_continue_waitid(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"g", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证当一个 CPU 上的用户进程陷入死循环计算时，本地时钟中断能够正常抢占该用户任务并调度同核上的其他内核任务运行，随后支持从另一个 CPU 发送 `SIGKILL` 将该死循环进程终止。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_timer_preempts_infinite_user_and_remote_kill(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.preempt_infinite(context)
    }

    /// 测试目标：验证用户态与内核态 CPU 时间统计（`times` 系统调用的 `USER_HZ` 滴答数与 `ITIMER_PROF`/`ITIMER_VIRTUAL` 间隔定时器）仅累加进程实际在 CPU 上运行的耗时，不会将睡眠阻塞（Off-CPU）时间计入执行账本。
    #[test]
    fn signal_cpu_time_and_interval_timers(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        let event = fixture.run_named(context, b"/test/bin/user-signal", b"c", false, 0)?;
        context.expect_eq(event.status, 0);
        Ok(())
    }

    /// 测试目标：验证当进程正在执行不可中断的磁盘文件页读取（等待块设备 I/O 完成）期间收到普通信号时，内核会先完成底层的物理页 DMA 读取并将数据拷贝到用户缓冲区，直到系统调用即将返回用户态时才处理挂起的信号。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_wait_keeps_file_read_responsibility(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.signal_during_file_wait(context)
    }

    /// 测试目标：验证 `futex` 在等待字未改变时的正常阻塞与唤醒、两个独立地址空间（`Mm`）同虚拟地址的私有 futex 隔离、以及同一共享匿名页映射到不同虚拟地址时的共享 futex 键值匹配。
    #[test]
    #[ktest(min_cpus = 2)]
    fn futex_wait_wake_and_mapping_keys(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        for mode in [b"w" as &[u8], b"p", b"n", b"a"].iter().copied() {
            fixture.futex_program(context, mode)?;
        }
        Ok(())
    }

    /// 测试目标：验证 `FUTEX_WAIT_BITSET` 与 `FUTEX_CMP_REQUEUE` 组合操作：重排（Requeue）到目标等待队列后仍保留等待者原有的位掩码与超时绝对期限，因超时或信号取消时仅从当前所在队列移除一次，且位掩码唤醒配额只统计位集匹配的等待者。
    #[test]
    #[ktest(min_cpus = 2)]
    fn futex_bitset_requeue_signal_and_timeout(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.futex_program(context, b"r")
    }

    /// 测试目标：验证线程退出时对 `clear_child_tid` 地址执行清零并唤醒其上的 `futex` 等待者（用于 `pthread_join` 同步）；同时验证即使 `clear_child_tid` 指向已解除映射或非法的用户地址，线程也能忽略该写错误并正常完成退出回收。
    #[test]
    #[ktest(min_cpus = 2)]
    fn futex_clear_child_tid_and_bad_exit_address(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.futex_program(context, b"t")
    }

    /// 测试目标：验证 `futex` 等待被带 `SA_RESTART` 的信号中断后能按原参数自动重启，且被停止/继续信号打断的相对超时等待在重启时保留首次计算的绝对到期时间，不会重新读取用户空间中可能已被解除映射的 `timespec` 参数页。
    #[test]
    #[ktest(min_cpus = 2)]
    fn futex_signal_restart_parameters_and_deadline(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        for mode in [b"s" as &[u8], b"d"].iter().copied() {
            fixture.futex_program(context, mode)?;
        }
        Ok(())
    }

    /// 测试目标：验证多线程进程中由非主线程（Non-leader）发起 `execve` 时，能够杀死同组其他线程并复用原主线程的 PID 接管进程身份；同时验证两个线程并发发起 `execve` 或与组致命信号并发时的互斥控制。
    #[test]
    #[ktest(min_cpus = 2)]
    fn exec_nonleader_and_concurrent_control(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_exec(context)
    }

    /// 测试目标：验证当进程与外部进程共享文件描述符表（`CLONE_FILES`）或信号处理器表（`CLONE_SIGHAND`）时执行 `execve`，内核会先解耦（Unshare）对应表而不破坏外部共享者的状态，同时正确处理 `O_CLOEXEC` 关闭与新旧地址空间（`Mm`）交接。
    #[test]
    #[ktest(min_cpus = 2)]
    fn exec_shared_resources_and_mm_handoff(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_resources(context)
    }

    /// 测试目标：验证 `vfork` 创建的子进程在调用 `execve` 或 `_exit` 释放旧地址空间借用后立即唤醒父进程；若父进程在等待期间被致命信号取消，子进程仍能独立完成执行并转交祖父/初始进程收养回收。
    #[test]
    #[ktest(min_cpus = 2)]
    fn vfork_exec_exit_and_parent_cancellation(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_vfork(context)
    }

    /// 测试目标：验证多线程进程的主线程（Thread Group Leader）先行调用 `exit` 退出后，只要同组还有其他存活线程，该进程组仍保有对子进程的父进程等待资格；直到整个线程组全部退出、完成孤儿转亲且僵尸状态被消费后，其 PID/PGID 才允许被新进程复用。
    #[test]
    #[ktest(min_cpus = 2)]
    fn exit_group_parentage_and_pid_role_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_roles(context)
    }

    /// 测试目标：验证两个并发阻塞在 `waitid` 上的等待者：使用 `WNOWAIT` 仅观察状态而不消耗僵尸，不使用 `WNOWAIT` 时只有一个等待者能成功消费该僵尸进程；且当 `waitid` 写回用户态 `siginfo_t` 发生 `EFAULT` 时不会导致僵尸状态丢失或重复消费。
    #[test]
    #[ktest(min_cpus = 2)]
    fn wait_observation_consumption_and_copyout(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_wait(context)
    }

    /// 测试目标：验证当 `SIGCHLD` 处于默认忽略状态时子进程退出仍保留僵尸状态供 `wait` 收集；而显式将 `SIGCHLD` 设为 `SIG_IGN` 或带有 `SA_NOCLDWAIT` 标志时，子进程退出后自动回收且 `wait` 在所有子进程退出后返回 `ECHILD`。
    #[test]
    #[ktest(min_cpus = 2)]
    fn sigchld_autoreap_and_wait_semantics(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_sigchld(context)
    }

    /// 测试目标：验证用户态程序触发未处理的段错误（`SIGSEGV`）、总线错误（`SIGBUS`）或非法指令（`SIGILL`）时，仅以对应的致命信号状态码终止该用户进程并由父进程 `wait` 收集，不会影响内核或其他进程。
    #[test]
    fn user_faults_are_wait_status(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        for (mode, signal) in [(b"s" as &[u8], 11), (b"b", 7), (b"i", 4)] {
            let event =
                fixture.run_named(context, b"/test/bin/user-lifecycle", mode, false, signal)?;
            context.expect_eq(event.status, signal);
        }
        Ok(())
    }

    /// 测试目标：验证当远端 CPU 上的用户线程正处于阻塞式磁盘文件读取（底层块设备 DMA 请求仍在途）时收到致命取消信号，内核必须等待在途的块设备 DMA 完成并安全释放用户地址空间（`Mm`）与内核栈后，才最终完成进程回收。
    #[test]
    #[ktest(min_cpus = 2)]
    fn user_cancel_keeps_inflight_io_and_mm(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.cancel_inflight_file_read(context)
    }
}
